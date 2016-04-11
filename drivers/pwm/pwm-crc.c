/*
 * Copyright (C) 2015 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Shobhit Kumar <shobhit.kumar@intel.com>
 */

#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/pwm.h>

#define PWM0_CLK_DIV		0x4B
#define  PWM_OUTPUT_ENABLE	BIT(7)
#define  PWM_DIV_CLK_0		0x00 /* DIVIDECLK = BASECLK */
#define  PWM_DIV_CLK_100	0x63 /* DIVIDECLK = BASECLK/100 */
#define  PWM_DIV_CLK_128	0x7F /* DIVIDECLK = BASECLK/128 */
#define  PWM_DIV_CLK_MASK	GENMASK(6, 0)

#define PWM0_DUTY_CYCLE		0x4E
#define BACKLIGHT_EN		0x51

#define PWM_MAX_LEVEL		0xFF

#define PWM_BASE_CLK		6000000  /* 6 MHz */
#define PWM_MAX_PERIOD_NS	21333    /* 46.875KHz */

/**
 * struct crystalcove_pwm - Crystal Cove PWM controller
 * @chip: the abstract pwm_chip structure.
 * @regmap: the regmap from the parent device.
 */
struct crystalcove_pwm {
	struct pwm_chip chip;
	struct regmap *regmap;
};

static inline struct crystalcove_pwm *to_crc_pwm(struct pwm_chip *pc)
{
	return container_of(pc, struct crystalcove_pwm, chip);
}

static void crc_pwm_get_state(struct pwm_chip *c,
			      struct pwm_device *pwm,
			      struct pwm_state *pstate)
{
	struct crystalcove_pwm *crc_pwm = to_crc_pwm(c);
	bool enabled = true;
	unsigned int val;
	u64 timens;

	regmap_read(crc_pwm->regmap, PWM0_CLK_DIV, &val);

	if (!(val & PWM_OUTPUT_ENABLE))
		enabled = false;

	timens = (u64)((val & PWM_DIV_CLK_MASK) + 1) * NSEC_PER_SEC;
	do_div(timens, PWM_BASE_CLK);
	pstate->period = timens;

	regmap_read(crc_pwm->regmap, PWM0_DUTY_CYCLE, &val);
	timens = val * pstate->period;
	do_div(timens, PWM_MAX_LEVEL);
	pstate->duty_cycle = timens;

	regmap_read(crc_pwm->regmap, BACKLIGHT_EN, &val);
	if (!(val & 1))
		enabled = false;
}

static int crc_pwm_apply(struct pwm_chip *c, struct pwm_device *pwm,
			 struct pwm_state *nstate)
{
	struct crystalcove_pwm *crc_pwm = to_crc_pwm(c);
	struct pwm_state state;

	if (nstate->polarity == PWM_POLARITY_INVERSED)
		return -ENOTSUPP;

	pwm_get_state(pwm, &state);

	if (state.period != nstate->period) {
		u64 clk_div;

		if (state.enabled) {
			/* changing the clk divisor, need to disable fisrt */
			regmap_write(crc_pwm->regmap, BACKLIGHT_EN, 0);
			state.enabled = false;
		}

		clk_div = PWM_BASE_CLK * nstate->period;
		do_div(clk_div, NSEC_PER_SEC);

		regmap_write(crc_pwm->regmap, PWM0_CLK_DIV,
			     clk_div | PWM_OUTPUT_ENABLE);
	}

	if (state.duty_cycle != nstate->duty_cycle ||
	    state.period != nstate->period) {
		u64 level;

		/* change the pwm duty cycle */
		level = nstate->duty_cycle * PWM_MAX_LEVEL;
		do_div(level, nstate->period);
		regmap_write(crc_pwm->regmap, PWM0_DUTY_CYCLE, level);
	}

	if (state.enabled != nstate->enabled)
		regmap_write(crc_pwm->regmap, BACKLIGHT_EN,
			    nstate->enabled ? 1 : 0);

	/* Update state with the real hardware value */
	crc_pwm_get_state(c, pwm, nstate);

	return 0;
}

static const struct pwm_ops crc_pwm_ops = {
	.get_state = crc_pwm_get_state,
	.apply = crc_pwm_apply,
};

static int crystalcove_pwm_probe(struct platform_device *pdev)
{
	struct crystalcove_pwm *pwm;
	struct device *dev = pdev->dev.parent;
	struct intel_soc_pmic *pmic = dev_get_drvdata(dev);

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &crc_pwm_ops;
	pwm->chip.base = -1;
	pwm->chip.npwm = 1;

	/* get the PMIC regmap */
	pwm->regmap = pmic->regmap;

	platform_set_drvdata(pdev, pwm);

	return pwmchip_add(&pwm->chip);
}

static int crystalcove_pwm_remove(struct platform_device *pdev)
{
	struct crystalcove_pwm *pwm = platform_get_drvdata(pdev);

	return pwmchip_remove(&pwm->chip);
}

static struct platform_driver crystalcove_pwm_driver = {
	.probe = crystalcove_pwm_probe,
	.remove = crystalcove_pwm_remove,
	.driver = {
		.name = "crystal_cove_pwm",
	},
};

builtin_platform_driver(crystalcove_pwm_driver);
