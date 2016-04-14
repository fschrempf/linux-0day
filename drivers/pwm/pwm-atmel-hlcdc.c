/*
 * Copyright (C) 2014 Free Electrons
 * Copyright (C) 2014 Atmel
 *
 * Author: Boris BREZILLON <boris.brezillon@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/atmel-hlcdc.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>

#define ATMEL_HLCDC_PWMCVAL_SHIFT	8
#define ATMEL_HLCDC_PWMCVAL_MASK	GENMASK(15, ATMEL_HLCDC_PWMCVAL_SHIFT)
#define ATMEL_HLCDC_PWMCVAL(x)		(((x) << ATMEL_HLCDC_PWMCVAL_SHIFT) & \
					 ATMEL_HLCDC_PWMCVAL_MASK)
#define ATMEL_HLCDC_PWMPOL		BIT(4)
#define ATMEL_HLCDC_PWMPS_MASK		GENMASK(2, 0)
#define ATMEL_HLCDC_PWMPS_MAX		0x6
#define ATMEL_HLCDC_PWMPS(x)		((x) & ATMEL_HLCDC_PWMPS_MASK)

struct atmel_hlcdc_pwm_errata {
	bool slow_clk_erratum;
	bool div1_clk_erratum;
};

struct atmel_hlcdc_pwm {
	struct pwm_chip chip;
	struct atmel_hlcdc *hlcdc;
	struct clk *cur_clk;
	const struct atmel_hlcdc_pwm_errata *errata;
};

static inline struct atmel_hlcdc_pwm *to_atmel_hlcdc_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct atmel_hlcdc_pwm, chip);
}

static int atmel_hlcdc_pwm_enable(struct atmel_hlcdc_pwm *chip)
{
	struct atmel_hlcdc *hlcdc = chip->hlcdc;
	u32 status;

	regmap_write(hlcdc->regmap, ATMEL_HLCDC_EN, ATMEL_HLCDC_PWM);

	while (true) {
		regmap_read(hlcdc->regmap, ATMEL_HLCDC_SR, &status);

		if ((status & ATMEL_HLCDC_PWM) != 0)
			break;

		usleep_range(1, 10);
	}

	return 0;
}

static void atmel_hlcdc_pwm_disable(struct atmel_hlcdc_pwm *chip)
{
	struct atmel_hlcdc *hlcdc = chip->hlcdc;
	u32 status;

	regmap_write(hlcdc->regmap, ATMEL_HLCDC_DIS, ATMEL_HLCDC_PWM);

	while (true) {
		regmap_read(hlcdc->regmap, ATMEL_HLCDC_SR, &status);

		if ((status & ATMEL_HLCDC_PWM) == 0)
			break;

		usleep_range(1, 10);
	}
}

static int atmel_hlcdc_pwm_apply(struct pwm_chip *c, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	struct atmel_hlcdc_pwm *chip = to_atmel_hlcdc_pwm(c);
	struct atmel_hlcdc *hlcdc = chip->hlcdc;
	struct clk *new_clk = hlcdc->slow_clk;
	unsigned long clk_freq;
	u64 period_ns, duty_ns;
	u32 pwmcfg, pwmcval;
	int pres;

	if (!chip->errata || !chip->errata->slow_clk_erratum) {
		clk_freq = clk_get_rate(new_clk);
		if (!clk_freq)
			return -EINVAL;

		period_ns = (u64)NSEC_PER_SEC * 256;
		do_div(period_ns, clk_freq);
	}

	/* Errata: cannot use slow clk on some IP revisions */
	if ((chip->errata && chip->errata->slow_clk_erratum) ||
	    period_ns > state->period) {
		new_clk = hlcdc->sys_clk;
		clk_freq = clk_get_rate(new_clk);
		if (!clk_freq)
			return -EINVAL;

		period_ns = DIV_ROUND_CLOSEST_ULL((u64)NSEC_PER_SEC * 256,
						  clk_freq);
	}

	for (pres = 0; pres <= ATMEL_HLCDC_PWMPS_MAX; pres++) {
		/* Errata: cannot divide by 1 on some IP revisions */
		if (!pres && chip->errata && chip->errata->div1_clk_erratum)
			continue;

		if ((period_ns << pres) >= state->period)
			break;
	}

	if (pres > ATMEL_HLCDC_PWMPS_MAX)
		return -EINVAL;

	period_ns <<= pres;
	pwmcfg = ATMEL_HLCDC_PWMPS(pres);

	if (new_clk != chip->cur_clk) {
		int ret;

		ret = clk_prepare_enable(new_clk);
		if (ret)
			return ret;
	}

	pwmcval = DIV_ROUND_CLOSEST_ULL((u64)state->duty_cycle * 256,
					period_ns);

	/*
	 * The PWM duty cycle is configurable from 0/256 to 255/256 of the
	 * period cycle. Hence we can't set a duty cycle occupying the
	 * whole period cycle if we're asked to.
	 * Set it to 255 if pwmcval is greater than 256.
	 */
	if (pwmcval > 255)
		pwmcval = 255;

	duty_ns = do_div(period_ns, pwmcval);
	pwmcfg |= ATMEL_HLCDC_PWMCVAL(pwmcval);

	if (state->polarity == PWM_POLARITY_NORMAL)
		pwmcfg |= ATMEL_HLCDC_PWMPOL;

	/* First disable the PWM if requested */
	if (!state->enabled)
		atmel_hlcdc_pwm_disable(chip);

	regmap_update_bits(hlcdc->regmap, ATMEL_HLCDC_CFG(0),
			   ATMEL_HLCDC_CLKPWMSEL,
			   new_clk == hlcdc->sys_clk ?
			   ATMEL_HLCDC_CLKPWMSEL : 0);
	regmap_update_bits(hlcdc->regmap, ATMEL_HLCDC_CFG(6),
			   ATMEL_HLCDC_PWMCVAL_MASK |
			   ATMEL_HLCDC_PWMPS_MASK |
			   ATMEL_HLCDC_PWMPOL,
			   pwmcfg);

	if (state->enabled)
		atmel_hlcdc_pwm_enable(chip);

	/* Disable the previous source clk if we switch to another one */
	if (new_clk != chip->cur_clk) {
		clk_disable_unprepare(chip->cur_clk);
		chip->cur_clk = new_clk;
	}

	state->period = period_ns;
	state->duty_cycle = duty_ns;

	return 0;
}

static void atmel_hlcdc_pwm_get_state(struct pwm_chip *c,
				      struct pwm_device *pwm,
				      struct pwm_state *state)
{
	struct atmel_hlcdc_pwm *chip = to_atmel_hlcdc_pwm(c);
	struct atmel_hlcdc *hlcdc = chip->hlcdc;
	unsigned long clk_freq;
	u64 duty;
	u32 val;

	regmap_read(hlcdc->regmap, ATMEL_HLCDC_SR, &val);
	state->enabled = val & ATMEL_HLCDC_SR;

	regmap_read(hlcdc->regmap, ATMEL_HLCDC_CFG(0), &val);
	if (val & ATMEL_HLCDC_CLKPWMSEL)
		clk_freq = clk_get_rate(hlcdc->sys_clk);
	else
		clk_freq = clk_get_rate(hlcdc->slow_clk);

	regmap_read(hlcdc->regmap, ATMEL_HLCDC_CFG(6), &val);
	if (val & ATMEL_HLCDC_PWMPOL)
		state->polarity = PWM_POLARITY_NORMAL;
	else
		state->polarity = PWM_POLARITY_INVERSED;

	state->period = clk_freq >> (val & ATMEL_HLCDC_PWMPS_MASK);

	duty = (val & ATMEL_HLCDC_PWMCVAL_MASK) >> ATMEL_HLCDC_PWMCVAL_SHIFT;
	state->duty_cycle = DIV_ROUND_CLOSEST_ULL(duty * state->period, 256);
}

static const struct pwm_ops atmel_hlcdc_pwm_ops = {
	.get_state = atmel_hlcdc_pwm_get_state,
	.apply = atmel_hlcdc_pwm_apply,
	.owner = THIS_MODULE,
};

static const struct atmel_hlcdc_pwm_errata atmel_hlcdc_pwm_at91sam9x5_errata = {
	.slow_clk_erratum = true,
};

static const struct atmel_hlcdc_pwm_errata atmel_hlcdc_pwm_sama5d3_errata = {
	.div1_clk_erratum = true,
};

static const struct of_device_id atmel_hlcdc_dt_ids[] = {
	{
		.compatible = "atmel,at91sam9n12-hlcdc",
		/* 9n12 has same errata as 9x5 HLCDC PWM */
		.data = &atmel_hlcdc_pwm_at91sam9x5_errata,
	},
	{
		.compatible = "atmel,at91sam9x5-hlcdc",
		.data = &atmel_hlcdc_pwm_at91sam9x5_errata,
	},
	{
		.compatible = "atmel,sama5d2-hlcdc",
	},
	{
		.compatible = "atmel,sama5d3-hlcdc",
		.data = &atmel_hlcdc_pwm_sama5d3_errata,
	},
	{
		.compatible = "atmel,sama5d4-hlcdc",
		.data = &atmel_hlcdc_pwm_sama5d3_errata,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, atmel_hlcdc_dt_ids);

static int atmel_hlcdc_pwm_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct atmel_hlcdc_pwm *chip;
	struct atmel_hlcdc *hlcdc;
	int ret;

	hlcdc = dev_get_drvdata(dev->parent);

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	ret = clk_prepare_enable(hlcdc->periph_clk);
	if (ret)
		return ret;

	match = of_match_node(atmel_hlcdc_dt_ids, dev->parent->of_node);
	if (match)
		chip->errata = match->data;

	chip->hlcdc = hlcdc;
	chip->chip.ops = &atmel_hlcdc_pwm_ops;
	chip->chip.dev = dev;
	chip->chip.base = -1;
	chip->chip.npwm = 1;
	chip->chip.of_xlate = of_pwm_xlate_with_flags;
	chip->chip.of_pwm_n_cells = 3;
	chip->chip.can_sleep = 1;

	ret = pwmchip_add(&chip->chip);
	if (ret) {
		clk_disable_unprepare(hlcdc->periph_clk);
		return ret;
	}

	platform_set_drvdata(pdev, chip);

	return 0;
}

static int atmel_hlcdc_pwm_remove(struct platform_device *pdev)
{
	struct atmel_hlcdc_pwm *chip = platform_get_drvdata(pdev);
	int ret;

	ret = pwmchip_remove(&chip->chip);
	if (ret)
		return ret;

	clk_disable_unprepare(chip->hlcdc->periph_clk);

	return 0;
}

static const struct of_device_id atmel_hlcdc_pwm_dt_ids[] = {
	{ .compatible = "atmel,hlcdc-pwm" },
	{ /* sentinel */ },
};

static struct platform_driver atmel_hlcdc_pwm_driver = {
	.driver = {
		.name = "atmel-hlcdc-pwm",
		.of_match_table = atmel_hlcdc_pwm_dt_ids,
	},
	.probe = atmel_hlcdc_pwm_probe,
	.remove = atmel_hlcdc_pwm_remove,
};
module_platform_driver(atmel_hlcdc_pwm_driver);

MODULE_ALIAS("platform:atmel-hlcdc-pwm");
MODULE_AUTHOR("Boris Brezillon <boris.brezillon@free-electrons.com>");
MODULE_DESCRIPTION("Atmel HLCDC PWM driver");
MODULE_LICENSE("GPL v2");
