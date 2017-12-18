/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (c) 2017 exceet electronics GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/delay.h>
#include <linux/mtd/fsl_quadspi.h>
#include <linux/mtd/spinand.h>
#include <linux/mtd/mtd.h>

/* SEQID -- we can have 16 seqids at most. */
#define SPINAND_SEQID_PAGE_READ			0
#define SPINAND_SEQID_READ_FROM_CACHE_FAST	1
#define SPINAND_SEQID_READ_FROM_CACHE_X4	2
#define SPINAND_SEQID_WR_ENABLE			3
#define SPINAND_SEQID_WR_DISABLE		4
#define SPINAND_SEQID_GET_FEATURE		5
#define SPINAND_SEQID_SET_FEATURE		6
#define SPINAND_SEQID_BLK_ERASE			7
#define SPINAND_SEQID_PROG_EXC			8
#define SPINAND_SEQID_PROG_LOAD_RDM_DATA_X4	9
#define SPINAND_SEQID_PROG_LOAD_RDM_DATA	10
#define SPINAND_SEQID_PROG_LOAD_X4		12
#define SPINAND_SEQID_PROG_LOAD			12
#define SPINAND_SEQID_READ_ID			13
#define SPINAND_SEQID_RESET			14
#define SPINAND_SEQID_DIE_SELECT		15

struct fsl_qspi_spinand_controller {
	struct spinand_controller ctrl;
	struct fsl_qspi *qspi;
};

#define to_fsl_qspi_spinand_controller(c) \
	container_of(c, struct fsl_qspi_spinand_controller, ctrl)

static irqreturn_t fsl_qspi_irq_handler(int irq, void *dev_id)
{
	struct fsl_qspi *q = dev_id;
	u32 reg;

	/* clear interrupt */
	reg = qspi_readl(q, q->iobase + QUADSPI_FR);
	qspi_writel(q, reg, q->iobase + QUADSPI_FR);

	if (reg & QUADSPI_FR_TFF_MASK)
		complete(&q->c);

	dev_dbg(q->dev, "QUADSPI_FR : 0x%.8x:0x%.8x\n", q->chip_base_addr, reg);
	return IRQ_HANDLED;
}

static void fsl_qspi_init_lut(struct fsl_qspi *qspi)
{
	void __iomem *base = qspi->iobase;
	int rxfifo = qspi->devtype_data->rxfifo;
	u32 lut_base;
	int i;

	fsl_qspi_unlock_lut(qspi);

	/* Clear all the LUT table */
	for (i = 0; i < QUADSPI_LUT_NUM; i++)
		qspi_writel(qspi, 0, base + QUADSPI_LUT_BASE + i * 4);

	/* Reset */
	lut_base = SPINAND_SEQID_RESET * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_RESET), base + QUADSPI_LUT(lut_base));

	/* Read ID */
	lut_base = SPINAND_SEQID_READ_ID * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_READ_ID) | LUT1(FSL_READ, PAD1, SPINAND_MAX_ID_LEN),
			base + QUADSPI_LUT(lut_base));

	/* Select Die */
	lut_base = SPINAND_SEQID_DIE_SELECT * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_DIE_SELECT) | LUT1(FSL_WRITE, PAD1, 0x08),
			base + QUADSPI_LUT(lut_base));

	/* Write Register */
	lut_base = SPINAND_SEQID_SET_FEATURE * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_SET_FEATURE) | LUT1(FSL_WRITE, PAD1, 0x10),
			base + QUADSPI_LUT(lut_base));

	/* Read Register */
	lut_base = SPINAND_SEQID_GET_FEATURE * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_GET_FEATURE) | LUT1(FSL_WRITE, PAD1, 0x08),
			base + QUADSPI_LUT(lut_base));
	qspi_writel(qspi, LUT0(FSL_READ, PAD1, 0x1),
			base + QUADSPI_LUT(lut_base + 1));

	/* Read page to Cache */
	lut_base = SPINAND_SEQID_PAGE_READ * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_PAGE_READ) | LUT1(ADDR, PAD1, 0x18),
		base + QUADSPI_LUT(lut_base));

	/* Read from Cache Fast */
	lut_base = SPINAND_SEQID_READ_FROM_CACHE_FAST * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_READ_FROM_CACHE_FAST) | LUT1(ADDR, PAD1, 0x10),
		base + QUADSPI_LUT(lut_base));
	qspi_writel(qspi, LUT0(DUMMY, PAD1, 0x08) | LUT1(FSL_READ, PAD1, rxfifo),
		base + QUADSPI_LUT(lut_base + 1));

	/* Read from Cache X4 */
	lut_base = SPINAND_SEQID_READ_FROM_CACHE_X4 * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_READ_FROM_CACHE_X4) | LUT1(ADDR, PAD1, 0x10),
		base + QUADSPI_LUT(lut_base));
	qspi_writel(qspi, LUT0(DUMMY, PAD1, 0x08) | LUT1(FSL_READ, PAD4, rxfifo),
		base + QUADSPI_LUT(lut_base + 1));

	/* Write enable */
	lut_base = SPINAND_SEQID_WR_ENABLE * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_WR_ENABLE), base + QUADSPI_LUT(lut_base));

	/* Write disable */
	lut_base = SPINAND_SEQID_WR_DISABLE * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_WR_DISABLE), base + QUADSPI_LUT(lut_base));

	/* Program Data Load Random (write to cache) X4 */
	lut_base = SPINAND_SEQID_PROG_LOAD_RDM_DATA_X4 * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_PROG_LOAD_RDM_DATA_X4) | LUT1(ADDR, PAD1, 0x10),
			base + QUADSPI_LUT(lut_base));
	qspi_writel(qspi, LUT0(FSL_WRITE, PAD4, 0), base + QUADSPI_LUT(lut_base + 1));

	/* Program Data Load Random (write to cache) */
	lut_base = SPINAND_SEQID_PROG_LOAD_RDM_DATA * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_PROG_LOAD_RDM_DATA) | LUT1(ADDR, PAD1, 0x10),
			base + QUADSPI_LUT(lut_base));
	qspi_writel(qspi, LUT0(FSL_WRITE, PAD1, 0), base + QUADSPI_LUT(lut_base + 1));

	/* Program Data Load (write to cache) X4 */
	lut_base = SPINAND_SEQID_PROG_LOAD_X4 * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_PROG_LOAD_X4) | LUT1(ADDR, PAD1, 0x10),
			base + QUADSPI_LUT(lut_base));
	qspi_writel(qspi, LUT0(FSL_WRITE, PAD4, 0), base + QUADSPI_LUT(lut_base + 1));

	/* Program Data Load (write to cache) */
	lut_base = SPINAND_SEQID_PROG_LOAD * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_PROG_LOAD) | LUT1(ADDR, PAD1, 0x10),
			base + QUADSPI_LUT(lut_base));
	qspi_writel(qspi, LUT0(FSL_WRITE, PAD1, 0), base + QUADSPI_LUT(lut_base + 1));

	/* Program Execute */
	lut_base = SPINAND_SEQID_PROG_EXC * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_PROG_EXC) | LUT1(ADDR, PAD1, 0x18),
		base + QUADSPI_LUT(lut_base));

	/* Erase block */
	lut_base = SPINAND_SEQID_BLK_ERASE * 4;
	qspi_writel(qspi, LUT0(CMD, PAD1, SPINAND_CMD_BLK_ERASE) | LUT1(ADDR, PAD1, 0x18),
			base + QUADSPI_LUT(lut_base));

	fsl_qspi_lock_lut(qspi);
}

/* Get the SEQID for the command */
static int fsl_qspi_get_seqid(struct fsl_qspi *qspi, u8 cmd)
{
	switch (cmd) {
	case SPINAND_CMD_RESET:
		return SPINAND_SEQID_RESET;
	case SPINAND_CMD_READ_ID:
		return SPINAND_SEQID_READ_ID;
	case SPINAND_CMD_SET_FEATURE:
		return SPINAND_SEQID_SET_FEATURE;
	case SPINAND_CMD_GET_FEATURE:
		return SPINAND_SEQID_GET_FEATURE;
	case SPINAND_CMD_PAGE_READ:
		return SPINAND_SEQID_PAGE_READ;
	case SPINAND_CMD_READ_FROM_CACHE_FAST:
		return SPINAND_SEQID_READ_FROM_CACHE_FAST;
	case SPINAND_CMD_READ_FROM_CACHE_X4:
		return SPINAND_SEQID_READ_FROM_CACHE_X4;
	case SPINAND_CMD_WR_ENABLE:
		return SPINAND_SEQID_WR_ENABLE;
	case SPINAND_CMD_WR_DISABLE:
		return SPINAND_SEQID_WR_DISABLE;
	case SPINAND_CMD_PROG_LOAD_RDM_DATA_X4:
		return SPINAND_SEQID_PROG_LOAD_RDM_DATA_X4;
	case SPINAND_CMD_PROG_LOAD_RDM_DATA:
		return SPINAND_SEQID_PROG_LOAD_RDM_DATA;
	case SPINAND_CMD_PROG_LOAD_X4:
		return SPINAND_SEQID_PROG_LOAD_X4;
	case SPINAND_CMD_PROG_LOAD:
		return SPINAND_SEQID_PROG_LOAD;
	case SPINAND_CMD_PROG_EXC:
		return SPINAND_SEQID_PROG_EXC;
	case SPINAND_CMD_BLK_ERASE:
		return SPINAND_SEQID_BLK_ERASE;
	case SPINAND_CMD_DIE_SELECT:
		return SPINAND_SEQID_DIE_SELECT;
	default:
		dev_err(qspi->dev, "Unsupported cmd 0x%.2x\n", cmd);
		break;
	}
	return -EINVAL;
}

/*
 * There are two different ways to read out the data from the flash:
 *  the "IP Command Read" and the "AHB Command Read".
 *
 * The IC guy suggests we use the "AHB Command Read" which is faster
 * then the "IP Command Read". (What's more is that there is a bug in
 * the "IP Command Read" in the Vybrid.)
 *
 * After we set up the registers for the "AHB Command Read", we can use
 * the memcpy to read the data directly. A "missed" access to the buffer
 * causes the controller to clear the buffer, and use the sequence pointed
 * by the QUADSPI_BFGENCR[SEQID] to initiate a read from the flash.
 */
static void fsl_qspi_init_abh_read(struct fsl_qspi *qspi, struct spinand_device *spinand)
{
	void __iomem *base = qspi->iobase;
	int seqid;

	/* AHB configuration for access buffer 0/1/2 .*/
	qspi_writel(qspi, QUADSPI_BUFXCR_INVALID_MSTRID, base + QUADSPI_BUF0CR);
	qspi_writel(qspi, QUADSPI_BUFXCR_INVALID_MSTRID, base + QUADSPI_BUF1CR);
	qspi_writel(qspi, QUADSPI_BUFXCR_INVALID_MSTRID, base + QUADSPI_BUF2CR);
	/*
	 * Set ADATSZ with the maximum AHB buffer size to improve the
	 * read performance.
	 */
	qspi_writel(qspi, QUADSPI_BUF3CR_ALLMST_MASK |
			((qspi->devtype_data->ahb_buf_size / 8)
			<< QUADSPI_BUF3CR_ADATSZ_SHIFT),
			base + QUADSPI_BUF3CR);

	/* We only use the buffer3 */
	qspi_writel(qspi, 0, base + QUADSPI_BUF0IND);
	qspi_writel(qspi, 0, base + QUADSPI_BUF1IND);
	qspi_writel(qspi, 0, base + QUADSPI_BUF2IND);

	/* Set the default lut sequence for AHB Read. */
	seqid = fsl_qspi_get_seqid(qspi, spinand->read_cache_op);
	qspi_writel(qspi, seqid << QUADSPI_BFGENCR_SEQID_SHIFT,
		qspi->iobase + QUADSPI_BFGENCR);
}

/* This function is used to prepare and enable QSPI clock */
static int fsl_qspi_clk_prep_enable(struct fsl_qspi *qspi)
{
	int ret;

	ret = clk_prepare_enable(qspi->clk_en);
	if (ret)
		return ret;

	ret = clk_prepare_enable(qspi->clk);
	if (ret) {
		clk_disable_unprepare(qspi->clk_en);
		return ret;
	}

	if (needs_wakeup_wait_mode(qspi))
		pm_qos_add_request(&qspi->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, 0);

	return 0;
}

/* This function is used to disable and unprepare QSPI clock */
static void fsl_qspi_clk_disable_unprep(struct fsl_qspi *qspi)
{
	if (needs_wakeup_wait_mode(qspi))
		pm_qos_remove_request(&qspi->pm_qos_req);

	clk_disable_unprepare(qspi->clk);
	clk_disable_unprepare(qspi->clk_en);
}

static void fsl_qspi_set_map_addr(struct fsl_qspi *qspi, struct spinand_device *spinand)
{
	void __iomem *base = qspi->iobase;
	struct mtd_info *mtd = spinand_to_mtd(spinand);
	struct nand_device *nand = mtd_to_nanddev(mtd);
	u32 size = nanddev_size(nand);

	/*
	 * use full buffer size for a single chip
	 * (parallel mode is not implemented yet)
	 */
	qspi_writel(qspi, size + qspi->memmap_phy, base + QUADSPI_SFA1AD);
	qspi_writel(qspi, size + qspi->memmap_phy, base + QUADSPI_SFA2AD);
	qspi_writel(qspi, size + qspi->memmap_phy, base + QUADSPI_SFB1AD);
	qspi_writel(qspi, size + qspi->memmap_phy, base + QUADSPI_SFB2AD);
}

/*
 * fsl_qspi_spinand_controller_setup - to initialize the QSPI controller
 * @spinand: SPI NAND device structure
 */
static int fsl_qspi_spinand_controller_setup(struct spinand_device *spinand)
{
	struct spinand_controller *spinand_controller;
	struct fsl_qspi_spinand_controller *controller;
	struct fsl_qspi *qspi;
	void __iomem *base;
	int ret;
	u32 reg;

	spinand_controller = spinand->controller.controller;
	controller = to_fsl_qspi_spinand_controller(spinand_controller);
	qspi = controller->qspi;
	base = qspi->iobase;

	/* disable and unprepare clock first */
	fsl_qspi_clk_disable_unprep(qspi);

	/* the default frequency, we will change it in the future. */
	ret = clk_set_rate(qspi->clk, 20000000);
	if (ret)
		return ret;

	ret = fsl_qspi_clk_prep_enable(qspi);
	if (ret)
		return ret;

	if ((qspi->devtype_data->devtype == FSL_QUADSPI_IMX6UL) ||
		(qspi->devtype_data->devtype == FSL_QUADSPI_IMX7D)) {
		/* clear the DDR_EN bit for 6UL and 7D */
		reg = readl(base + QUADSPI_MCR);
		writel(~(QUADSPI_MCR_DDR_EN_MASK) & reg, base + QUADSPI_MCR);
		udelay(1);
	}

	/* Reset the module */
	writel(QUADSPI_MCR_SWRSTSD_MASK | QUADSPI_MCR_SWRSTHD_MASK,
		base + QUADSPI_MCR);
	udelay(1);

	/* Init the LUT table. */
	fsl_qspi_init_lut(qspi);

	/* Disable the module */
	writel(QUADSPI_MCR_MDIS_MASK | QUADSPI_MCR_RESERVED_MASK,
			base + QUADSPI_MCR);

	reg = readl(base + QUADSPI_SMPR);
	writel(reg & ~(QUADSPI_SMPR_FSDLY_MASK
			| QUADSPI_SMPR_FSPHS_MASK
			| QUADSPI_SMPR_HSENA_MASK
			| QUADSPI_SMPR_DDRSMP_MASK), base + QUADSPI_SMPR);

	/* Enable the module */
	writel(QUADSPI_MCR_RESERVED_MASK | QUADSPI_MCR_END_CFG_MASK,
			base + QUADSPI_MCR);

	/* enable the interrupt */
	writel(0xffffffff, qspi->iobase + QUADSPI_FR);
	writel(QUADSPI_RSER_TFIE, qspi->iobase + QUADSPI_RSER);

	return 0;
}

/*
 * fsl_qspi_spinand_controller_setup_late - to late initialize the QSPI controller
 * @spinand: SPI NAND device structure
 */
static int fsl_qspi_spinand_controller_setup_late(struct spinand_device *spinand)
{
	struct spinand_controller *spinand_controller;
	struct fsl_qspi_spinand_controller *controller;
	struct fsl_qspi *qspi;
	int ret;

	spinand_controller = spinand->controller.controller;
	controller = to_fsl_qspi_spinand_controller(spinand_controller);
	qspi = controller->qspi;

	fsl_qspi_set_map_addr(qspi, spinand);

	/* Init for AHB read */
	fsl_qspi_init_abh_read(qspi, spinand);

	return 0;
}

static int fsl_qspi_runcmd(struct fsl_qspi *qspi, u8 cmd, unsigned int addr, int len)
{
	void __iomem *base = qspi->iobase;
	int seqid;
	u32 reg, reg2;
	int err;

	init_completion(&qspi->c);
	dev_dbg(qspi->dev, "to 0x%.8x:0x%.8x, len:%d, cmd:%.2x\n",
			qspi->chip_base_addr, addr, len, cmd);

	/* save the reg */
	reg = qspi_readl(qspi, base + QUADSPI_MCR);

	qspi_writel(qspi, qspi->memmap_phy + qspi->chip_base_addr + addr,
			base + QUADSPI_SFAR);
	qspi_writel(qspi, QUADSPI_RBCT_WMRK_MASK | QUADSPI_RBCT_RXBRD_USEIPS,
			base + QUADSPI_RBCT);
	qspi_writel(qspi, reg | QUADSPI_MCR_CLR_RXF_MASK, base + QUADSPI_MCR);

	do {
		reg2 = qspi_readl(qspi, base + QUADSPI_SR);
		if (reg2 & (QUADSPI_SR_IP_ACC_MASK | QUADSPI_SR_AHB_ACC_MASK)) {
			udelay(1);
			dev_dbg(qspi->dev, "The controller is busy, 0x%x\n", reg2);
			continue;
		}
		break;
	} while (1);

	/* trigger the LUT now */
	seqid = fsl_qspi_get_seqid(qspi, cmd);
	qspi_writel(qspi, (seqid << QUADSPI_IPCR_SEQID_SHIFT) | len,
			base + QUADSPI_IPCR);

	/* Wait for the interrupt. */
	if (!wait_for_completion_timeout(&qspi->c, msecs_to_jiffies(1000))) {
		dev_err(qspi->dev,
			"cmd 0x%.2x timeout, addr@%.8x, FR:0x%.8x, SR:0x%.8x\n",
			cmd, addr, qspi_readl(qspi, base + QUADSPI_FR),
			qspi_readl(qspi, base + QUADSPI_SR));
		err = -ETIMEDOUT;
	} else {
		err = 0;
	}

	/* restore the MCR */
	qspi_writel(qspi, reg, base + QUADSPI_MCR);

	return err;
}

/* Read out the data from the QUADSPI_RBDR buffer registers. */
static void fsl_qspi_read(struct fsl_qspi *qspi, int len, u8 *rxbuf)
{
	u32 tmp;
	int i = 0;

	while (len > 0) {
		tmp = qspi_readl(qspi, qspi->iobase + QUADSPI_RBDR + i * 4);
		tmp = fsl_qspi_endian_xchg(qspi, tmp);
		dev_dbg(qspi->dev, "chip addr:0x%.8x, rcv:0x%.8x\n",
				qspi->chip_base_addr, tmp);

		if (len >= 4) {
			*((u32 *)rxbuf) = tmp;
			rxbuf += 4;
		} else {
			memcpy(rxbuf, &tmp, len);
			break;
		}

		len -= 4;
		i++;
	}
}

static int fsl_qspi_write(struct fsl_qspi *qspi, u8 opcode,
				unsigned int to, u32 *txbuf, unsigned count)
{
	int ret, i, j, s, written = 0;
	u32 tmp;
	int max_tx = qspi->devtype_data->txfifo;
	int towrite = count;

	dev_dbg(qspi->dev, "to 0x%.8x:0x%.8x, len : %d\n",
		qspi->chip_base_addr, to, count);

	while(towrite > 0) {
		s = (towrite > max_tx) ? max_tx : towrite;
		/* clear the TX FIFO. */
		tmp = qspi_readl(qspi, qspi->iobase + QUADSPI_MCR);
		qspi_writel(qspi, tmp | QUADSPI_MCR_CLR_TXF_MASK, qspi->iobase + QUADSPI_MCR);

		/* fill the TX data to the FIFO */
		for (j = 0, i = ((s + 3) / 4); j < i; j++) {
			tmp = fsl_qspi_endian_xchg(qspi, *txbuf);
			qspi_writel(qspi, tmp, qspi->iobase + QUADSPI_TBDR);
			txbuf++;
		}

		/* fill the TXFIFO upto 16 bytes for i.MX7D/i.MX6UL */
		if (needs_fill_txfifo(qspi))
			for (; i < 4; i++)
				qspi_writel(qspi, tmp, qspi->iobase + QUADSPI_TBDR);

		/* Trigger it */
		ret = fsl_qspi_runcmd(qspi, opcode, to + written, s);
		if (ret)
			return ret;

		/*
		 * Hack to avoid clearing the buffer on subsequent writes.
		 * Only clear the buffer on the first partial write and
		 * afterwards use the RANDOM commands to preserve the
		 * content already written.
		 */
		if(opcode == SPINAND_CMD_PROG_LOAD_X4)
			opcode = SPINAND_CMD_PROG_LOAD_RDM_DATA_X4;
		else if(opcode == SPINAND_CMD_PROG_LOAD)
			opcode = SPINAND_CMD_PROG_LOAD_RDM_DATA;

		towrite -= s;
		written += s;
	}

	return ret;
}

static int fsl_qspi_read_ahb(struct fsl_qspi *qspi, loff_t from,
			     size_t len, u_char *buf)
{
	/* if necessary,ioremap buffer before AHB read, */
	if (!qspi->ahb_addr) {
		qspi->memmap_offs = qspi->chip_base_addr + from;
		qspi->memmap_len = len > QUADSPI_MIN_IOMAP ? len : QUADSPI_MIN_IOMAP;

		qspi->ahb_addr = ioremap_nocache(
				qspi->memmap_phy + qspi->memmap_offs,
				qspi->memmap_len);
		if (!qspi->ahb_addr) {
			dev_err(qspi->dev, "ioremap failed\n");
			return -ENOMEM;
		}
	/* ioremap if the data requested is out of range */
	} else if (qspi->chip_base_addr + from < qspi->memmap_offs
			|| qspi->chip_base_addr + from + len >
			qspi->memmap_offs + qspi->memmap_len) {
		iounmap(qspi->ahb_addr);

		qspi->memmap_offs = qspi->chip_base_addr + from;
		qspi->memmap_len = len > QUADSPI_MIN_IOMAP ? len : QUADSPI_MIN_IOMAP;
		qspi->ahb_addr = ioremap_nocache(
				qspi->memmap_phy + qspi->memmap_offs,
				qspi->memmap_len);
		if (!qspi->ahb_addr) {
			dev_err(qspi->dev, "ioremap failed\n");
			return -ENOMEM;
		}
	}

	dev_dbg(qspi->dev, "ahb read from %p, len:%zd\n",
		qspi->ahb_addr + qspi->chip_base_addr + from - qspi->memmap_offs,
		len);

	/* Read out the data directly from the AHB buffer.*/
	memcpy(buf, qspi->ahb_addr + qspi->chip_base_addr + from - qspi->memmap_offs,
		len);

	return 0;
}

/*
 * If we have changed the content of the flash by writing, erasing or
 * reading to the chip buffer, we need to invalidate the AHB buffer.
 * If we do not do so, we may read out the wrong data. The spec tells 
 * us to reset the AHB domain and Serial Flash domain at the same time.
 */
static inline void fsl_qspi_ahb_invalidate(struct fsl_qspi *qspi)
{
	u32 reg;

	reg = qspi_readl(qspi, qspi->iobase + QUADSPI_MCR);
	reg |= QUADSPI_MCR_SWRSTHD_MASK | QUADSPI_MCR_SWRSTSD_MASK;
	qspi_writel(qspi, reg, qspi->iobase + QUADSPI_MCR);

	/*
	 * The minimum delay : 1 AHB + 2 SFCK clocks.
	 * Delay 1 us is enough.
	 */
	udelay(1);

	reg &= ~(QUADSPI_MCR_SWRSTHD_MASK | QUADSPI_MCR_SWRSTSD_MASK);
	qspi_writel(qspi, reg, qspi->iobase + QUADSPI_MCR);
}

/*
 * fsl_qspi_spinand_controller_exec_op - to process a command to send to the
 * SPI NAND by FSL QuadSPI bus
 * @spinand: SPI NAND device structure
 * @op: SPI NAND operation descriptor
 */
static int fsl_qspi_spinand_controller_exec_op(struct spinand_device *spinand,
				   struct spinand_op *op)
{
	struct spinand_controller *spinand_controller;
	struct fsl_qspi_spinand_controller *controller;
	struct fsl_qspi *qspi;
	u32 addr = 0;
	int ret = 0;

	spinand_controller = spinand->controller.controller;
	controller = to_fsl_qspi_spinand_controller(spinand_controller);
	qspi = controller->qspi;

	switch(op->n_addr) {
		case 3:
			addr = op->addr[0] << 16 | op->addr[1] << 8 | op->addr[2];
			break;
		case 2:
			addr = op->addr[0] << 8 | op->addr[1];
			break;
		case 1:
			addr = op->addr[0];
			break;
	}

	switch (op->cmd) {
	case SPINAND_CMD_PAGE_READ:
	case SPINAND_CMD_BLK_ERASE:
	case SPINAND_CMD_RESET:
	case SPINAND_CMD_WR_ENABLE:
	case SPINAND_CMD_WR_DISABLE:
	case SPINAND_CMD_READ_ID:
	case SPINAND_CMD_PROG_EXC:
		ret = fsl_qspi_runcmd(qspi, op->cmd, addr, 0);
		if (op->n_rx)
			fsl_qspi_read(qspi, op->n_rx, op->rx_buf);
		break;
	/*
	 * for some reason reading/writing a register doesn't work when
	 * issueing the command with an address. Therefore we write the
	 * address as one data byte and afterwards read/write the
	 * register content byte.
	 */
	case SPINAND_CMD_GET_FEATURE:
		fsl_qspi_write(qspi, op->cmd, 0, &addr, 1);
		fsl_qspi_read(qspi, op->n_rx, op->rx_buf);
		break;
	case SPINAND_CMD_SET_FEATURE:
		addr = (addr & 0xFF) | ((u32)(op->tx_buf[0]) << 8);
		fsl_qspi_write(qspi, op->cmd, 0, &addr, 2);
		break;
	case SPINAND_CMD_DIE_SELECT:
		fsl_qspi_write(qspi, op->cmd, 0, &addr, 1);
		break;
	case SPINAND_CMD_READ_FROM_CACHE_X4:
	case SPINAND_CMD_READ_FROM_CACHE_FAST:
		ret = fsl_qspi_read_ahb(qspi, addr, op->n_rx, op->rx_buf);
		break;
	case SPINAND_CMD_PROG_LOAD_RDM_DATA_X4:
	case SPINAND_CMD_PROG_LOAD_RDM_DATA:
	case SPINAND_CMD_PROG_LOAD_X4:
	case SPINAND_CMD_PROG_LOAD:
		fsl_qspi_write(qspi, op->cmd, addr, (u32 *)(op->tx_buf), op->n_tx);
		break;
	default:
		dev_err(qspi->dev, "Unsupported cmd 0x%.2x\n", op->cmd);
		break;
	}

	if (ret) {
		dev_err(qspi->dev, "Error running cmd 0x%.2x\n", op->cmd);
		return ret;
	}

	if (op->cmd == SPINAND_CMD_PAGE_READ ||
	    op->cmd == SPINAND_CMD_BLK_ERASE ||
	    op->cmd == SPINAND_CMD_PROG_EXC)
		fsl_qspi_ahb_invalidate(qspi);

	return 0;
}

static struct spinand_controller_ops fsl_qspi_spinand_controller_ops = {
	.exec_op = fsl_qspi_spinand_controller_exec_op,
	.setup = fsl_qspi_spinand_controller_setup,
	.setup_late = fsl_qspi_spinand_controller_setup_late,
};

static int fsl_qspi_spinand_controller_probe(struct platform_device *pdev)
{
	struct fsl_qspi *qspi;
	struct spinand_device *spinand;
	struct fsl_qspi_spinand_controller *controller;
	struct spinand_controller *spinand_controller;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	qspi = devm_kzalloc(dev, sizeof(*qspi), GFP_KERNEL);
	if (!qspi) {
		ret = -ENOMEM;
		goto out;
	}

	qspi->dev = dev;
	qspi->devtype_data = of_device_get_match_data(dev);
	if (!qspi->devtype_data) {
		ret = -ENODEV;
		goto out;
	}
	platform_set_drvdata(pdev, qspi);

	/* find the resources */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "QuadSPI");
	qspi->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(qspi->iobase)) {
		ret = PTR_ERR(qspi->iobase);
		goto out;
	}

	qspi->big_endian = of_property_read_bool(np, "big-endian");
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					"QuadSPI-memory");
	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     res->name)) {
		dev_err(dev, "can't request region for resource %pR\n", res);
		ret = -EBUSY;
		goto out;
	}

	qspi->memmap_phy = res->start;
	qspi->chip_base_addr = 0;

	/* find the clocks */
	qspi->clk_en = devm_clk_get(dev, "qspi_en");
	if (IS_ERR(qspi->clk_en)) {
		ret = PTR_ERR(qspi->clk_en);
		goto out;
	}

	qspi->clk = devm_clk_get(dev, "qspi");
	if (IS_ERR(qspi->clk)) {
		ret = PTR_ERR(qspi->clk);
		goto out;
	}

	ret = fsl_qspi_clk_prep_enable(qspi);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		goto out;
	}

	/* find the irq */
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "failed to get the irq: %d\n", ret);
		goto irq_failed;
	}

	ret = devm_request_irq(dev, ret,
			fsl_qspi_irq_handler, 0, pdev->name, qspi);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		goto irq_failed;
	}

	spinand = devm_spinand_alloc(dev);
	if (IS_ERR(spinand)) {
		ret = PTR_ERR(spinand);
		goto irq_failed;
	}

	controller = devm_kzalloc(dev, sizeof(*controller), GFP_KERNEL);
	if (!controller) {
		ret = -ENOMEM;
		goto irq_failed;
	}

	controller->qspi = qspi;
	spinand_controller = &controller->ctrl;
	spinand_controller->ops = &fsl_qspi_spinand_controller_ops;
	spinand_controller->caps = SPINAND_CAP_RD_X1 | SPINAND_CAP_WR_X1 |
				SPINAND_CAP_RD_X4 | SPINAND_CAP_WR_X4;

	spinand->controller.controller = spinand_controller;
	dev_set_drvdata(dev, spinand);

	ret = spinand_init(spinand, THIS_MODULE);
	if (ret)
		goto irq_failed;

	ret = mtd_device_register(spinand_to_mtd(spinand), NULL, 0);
	if (ret)
		goto irq_failed;

	mutex_init(&qspi->lock);

irq_failed:
	if (ret)
		fsl_qspi_clk_disable_unprep(qspi);
out:
	return ret;
}

static int fsl_qspi_spinand_controller_remove(struct platform_device *pdev)
{
	struct fsl_qspi *qspi = platform_get_drvdata(pdev);
	struct spinand_device *spinand = dev_get_drvdata(qspi->dev);
	int ret;

	ret = mtd_device_unregister(spinand_to_mtd(spinand));
	if (ret)
		return ret;

	spinand_cleanup(spinand);

	/* disable the hardware */
	qspi_writel(qspi, QUADSPI_MCR_MDIS_MASK, qspi->iobase + QUADSPI_MCR);
	qspi_writel(qspi, 0x0, qspi->iobase + QUADSPI_RSER);

	mutex_destroy(&qspi->lock);

	if (qspi->ahb_addr)
		iounmap(qspi->ahb_addr);

	return 0;
}

static const struct of_device_id fsl_qspi_spinand_dt_ids [] = {
	{ .compatible = "fsl,vf610-qspi", .data = (void *)&vybrid_data, },
	{ .compatible = "fsl,imx6sx-qspi", .data = (void *)&imx6sx_data, },
	{ .compatible = "fsl,imx7d-qspi", .data = (void *)&imx7d_data, },
	{ .compatible = "fsl,imx6ul-qspi", .data = (void *)&imx6ul_data, },
	{ .compatible = "fsl,ls1021a-qspi", .data = (void *)&ls1021a_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_qspi_spinand_dt_ids);

static struct platform_driver fsl_qspi_spinand_controller_driver = {
	.driver = {
		.name	= "fsl-quadspi-spinand-controller",
		.bus	= &platform_bus_type,
		.of_match_table = fsl_qspi_spinand_dt_ids,
	},
	.probe	= fsl_qspi_spinand_controller_probe,
	.remove	= fsl_qspi_spinand_controller_remove,
};
module_platform_driver(fsl_qspi_spinand_controller_driver);

MODULE_DESCRIPTION("Freescale QuadSPI NAND controller");
MODULE_AUTHOR("Frieder Schrempf <frieder.schrempf@exceet.de>");
MODULE_LICENSE("GPL v2");
