/*
 * Freescale QuadSPI driver.
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/completion.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/sizes.h>

/* Controller needs driver to swap endian */
#define QUADSPI_QUIRK_SWAP_ENDIAN	(1 << 0)
/* Controller needs 4x internal clock */
#define QUADSPI_QUIRK_4X_INT_CLK	(1 << 1)
/*
 * TKT253890, Controller needs driver to fill txfifo till 16 byte to
 * trigger data transfer even though extern data will not transferred.
 */
#define QUADSPI_QUIRK_TKT253890		(1 << 2)
/* Controller cannot wake up from wait mode, TKT245618 */
#define QUADSPI_QUIRK_TKT245618         (1 << 3)

/* The registers */
#define QUADSPI_MCR			0x00
#define QUADSPI_MCR_RESERVED_SHIFT	16
#define QUADSPI_MCR_RESERVED_MASK	(0xF << QUADSPI_MCR_RESERVED_SHIFT)
#define QUADSPI_MCR_MDIS_SHIFT		14
#define QUADSPI_MCR_MDIS_MASK		(1 << QUADSPI_MCR_MDIS_SHIFT)
#define QUADSPI_MCR_CLR_TXF_SHIFT	11
#define QUADSPI_MCR_CLR_TXF_MASK	(1 << QUADSPI_MCR_CLR_TXF_SHIFT)
#define QUADSPI_MCR_CLR_RXF_SHIFT	10
#define QUADSPI_MCR_CLR_RXF_MASK	(1 << QUADSPI_MCR_CLR_RXF_SHIFT)
#define QUADSPI_MCR_DDR_EN_SHIFT	7
#define QUADSPI_MCR_DDR_EN_MASK		(1 << QUADSPI_MCR_DDR_EN_SHIFT)
#define QUADSPI_MCR_END_CFG_SHIFT	2
#define QUADSPI_MCR_END_CFG_MASK	(3 << QUADSPI_MCR_END_CFG_SHIFT)
#define QUADSPI_MCR_SWRSTHD_SHIFT	1
#define QUADSPI_MCR_SWRSTHD_MASK	(1 << QUADSPI_MCR_SWRSTHD_SHIFT)
#define QUADSPI_MCR_SWRSTSD_SHIFT	0
#define QUADSPI_MCR_SWRSTSD_MASK	(1 << QUADSPI_MCR_SWRSTSD_SHIFT)

#define QUADSPI_IPCR			0x08
#define QUADSPI_IPCR_SEQID_SHIFT	24
#define QUADSPI_IPCR_SEQID_MASK		(0xF << QUADSPI_IPCR_SEQID_SHIFT)

#define QUADSPI_BUF0CR			0x10
#define QUADSPI_BUF1CR			0x14
#define QUADSPI_BUF2CR			0x18
#define QUADSPI_BUFXCR_INVALID_MSTRID	0xe

#define QUADSPI_BUF3CR			0x1c
#define QUADSPI_BUF3CR_ALLMST_SHIFT	31
#define QUADSPI_BUF3CR_ALLMST_MASK	(1 << QUADSPI_BUF3CR_ALLMST_SHIFT)
#define QUADSPI_BUF3CR_ADATSZ_SHIFT		8
#define QUADSPI_BUF3CR_ADATSZ_MASK	(0xFF << QUADSPI_BUF3CR_ADATSZ_SHIFT)

#define QUADSPI_BFGENCR			0x20
#define QUADSPI_BFGENCR_PAR_EN_SHIFT	16
#define QUADSPI_BFGENCR_PAR_EN_MASK	(1 << (QUADSPI_BFGENCR_PAR_EN_SHIFT))
#define QUADSPI_BFGENCR_SEQID_SHIFT	12
#define QUADSPI_BFGENCR_SEQID_MASK	(0xF << QUADSPI_BFGENCR_SEQID_SHIFT)

#define QUADSPI_BUF0IND			0x30
#define QUADSPI_BUF1IND			0x34
#define QUADSPI_BUF2IND			0x38
#define QUADSPI_SFAR			0x100

#define QUADSPI_SMPR			0x108
#define QUADSPI_SMPR_DDRSMP_SHIFT	16
#define QUADSPI_SMPR_DDRSMP_MASK	(7 << QUADSPI_SMPR_DDRSMP_SHIFT)
#define QUADSPI_SMPR_FSDLY_SHIFT	6
#define QUADSPI_SMPR_FSDLY_MASK		(1 << QUADSPI_SMPR_FSDLY_SHIFT)
#define QUADSPI_SMPR_FSPHS_SHIFT	5
#define QUADSPI_SMPR_FSPHS_MASK		(1 << QUADSPI_SMPR_FSPHS_SHIFT)
#define QUADSPI_SMPR_HSENA_SHIFT	0
#define QUADSPI_SMPR_HSENA_MASK		(1 << QUADSPI_SMPR_HSENA_SHIFT)

#define QUADSPI_RBSR			0x10c
#define QUADSPI_RBSR_RDBFL_SHIFT	8
#define QUADSPI_RBSR_RDBFL_MASK		(0x3F << QUADSPI_RBSR_RDBFL_SHIFT)

#define QUADSPI_RBCT			0x110
#define QUADSPI_RBCT_WMRK_MASK		0x1F
#define QUADSPI_RBCT_RXBRD_SHIFT	8
#define QUADSPI_RBCT_RXBRD_USEIPS	(0x1 << QUADSPI_RBCT_RXBRD_SHIFT)

#define QUADSPI_TBSR			0x150
#define QUADSPI_TBDR			0x154
#define QUADSPI_SR			0x15c
#define QUADSPI_SR_IP_ACC_SHIFT		1
#define QUADSPI_SR_IP_ACC_MASK		(0x1 << QUADSPI_SR_IP_ACC_SHIFT)
#define QUADSPI_SR_AHB_ACC_SHIFT	2
#define QUADSPI_SR_AHB_ACC_MASK		(0x1 << QUADSPI_SR_AHB_ACC_SHIFT)

#define QUADSPI_FR			0x160
#define QUADSPI_FR_TFF_MASK		0x1

#define QUADSPI_SFA1AD			0x180
#define QUADSPI_SFA2AD			0x184
#define QUADSPI_SFB1AD			0x188
#define QUADSPI_SFB2AD			0x18c
#define QUADSPI_RBDR			0x200

#define QUADSPI_LUTKEY			0x300
#define QUADSPI_LUTKEY_VALUE		0x5AF05AF0

#define QUADSPI_LCKCR			0x304
#define QUADSPI_LCKER_LOCK		0x1
#define QUADSPI_LCKER_UNLOCK		0x2

#define QUADSPI_RSER			0x164
#define QUADSPI_RSER_TFIE		(0x1 << 0)

#define QUADSPI_LUT_BASE		0x310

/*
 * The definition of the LUT register shows below:
 *
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */
#define OPRND0_SHIFT		0
#define PAD0_SHIFT		8
#define INSTR0_SHIFT		10
#define OPRND1_SHIFT		16

/* Instruction set for the LUT register. */
#define LUT_STOP		0
#define LUT_CMD			1
#define LUT_ADDR		2
#define LUT_DUMMY		3
#define LUT_MODE		4
#define LUT_MODE2		5
#define LUT_MODE4		6
#define LUT_FSL_READ		7
#define LUT_FSL_WRITE		8
#define LUT_JMP_ON_CS		9
#define LUT_ADDR_DDR		10
#define LUT_MODE_DDR		11
#define LUT_MODE2_DDR		12
#define LUT_MODE4_DDR		13
#define LUT_FSL_READ_DDR		14
#define LUT_FSL_WRITE_DDR		15
#define LUT_DATA_LEARN		16

/*
 * The PAD definitions for LUT register.
 *
 * The pad stands for the lines number of IO[0:3].
 * For example, the Quad read need four IO lines, so you should
 * set LUT_PAD4 which means we use four IO lines.
 */
#define LUT_PAD1		0
#define LUT_PAD2		1
#define LUT_PAD4		2

/* Oprands for the LUT register. */
#define ADDR24BIT		0x18
#define ADDR32BIT		0x20

/* Macros for constructing the LUT register. */
#define LUT0(ins, pad, opr)						\
		(((opr) << OPRND0_SHIFT) | ((pad) << PAD0_SHIFT) | \
		((LUT_##ins) << INSTR0_SHIFT))

#define LUT1(ins, pad, opr)	(LUT0(ins, pad, opr) << OPRND1_SHIFT)

/* other macros for LUT register. */
#define QUADSPI_LUT(x)          (QUADSPI_LUT_BASE + (x) * 4)
#define QUADSPI_LUT_NUM		64

/* SEQID -- we can have 16 seqids at most. */
#define SPINAND_SEQID_IP			0
#define SPINAND_SEQID_AHB			1

#define QUADSPI_MIN_IOMAP SZ_4M

enum fsl_qspi_devtype {
	FSL_QUADSPI_VYBRID,
	FSL_QUADSPI_IMX6SX,
	FSL_QUADSPI_IMX7D,
	FSL_QUADSPI_IMX6UL,
	FSL_QUADSPI_LS1021A,
};

struct fsl_qspi_devtype_data {
	enum fsl_qspi_devtype devtype;
	int rxfifo;
	int txfifo;
	int ahb_buf_size;
	int driver_data;
};

static const struct fsl_qspi_devtype_data vybrid_data = {
	.devtype = FSL_QUADSPI_VYBRID,
	.rxfifo = 128,
	.txfifo = 64,
	.ahb_buf_size = 1024,
	.driver_data = QUADSPI_QUIRK_SWAP_ENDIAN,
};

static const struct fsl_qspi_devtype_data imx6sx_data = {
	.devtype = FSL_QUADSPI_IMX6SX,
	.rxfifo = 128,
	.txfifo = 512,
	.ahb_buf_size = 1024,
	.driver_data = QUADSPI_QUIRK_4X_INT_CLK
		       | QUADSPI_QUIRK_TKT245618,
};

static const struct fsl_qspi_devtype_data imx7d_data = {
	.devtype = FSL_QUADSPI_IMX7D,
	.rxfifo = 512,
	.txfifo = 512,
	.ahb_buf_size = 1024,
	.driver_data = QUADSPI_QUIRK_TKT253890
		       | QUADSPI_QUIRK_4X_INT_CLK,
};

static const struct fsl_qspi_devtype_data imx6ul_data = {
	.devtype = FSL_QUADSPI_IMX6UL,
	.rxfifo = 128,
	.txfifo = 512,
	.ahb_buf_size = 1024,
	.driver_data = QUADSPI_QUIRK_TKT253890
		       | QUADSPI_QUIRK_4X_INT_CLK,
};

static struct fsl_qspi_devtype_data ls1021a_data = {
	.devtype = FSL_QUADSPI_LS1021A,
	.rxfifo = 128,
	.txfifo = 64,
	.ahb_buf_size = 1024,
	.driver_data = 0,
};

#define FSL_QSPI_MAX_CHIP	4
struct fsl_qspi {
	struct spi_nor nor[FSL_QSPI_MAX_CHIP];
	void __iomem *iobase;
	void __iomem *ahb_addr;
	u32 memmap_phy;
	u32 memmap_offs;
	u32 memmap_len;
	struct clk *clk, *clk_en;
	struct device *dev;
	struct completion c;
	const struct fsl_qspi_devtype_data *devtype_data;
	u32 nor_size;
	u32 nor_num;
	u32 clk_rate;
	unsigned int chip_base_addr; /* We may support two chips. */
	bool has_second_chip;
	bool big_endian;
	struct mutex lock;
	struct pm_qos_request pm_qos_req;
	u8 cached_ip_command;
	u8 cached_ahb_command;
};

#define FSL_QSPI_FLASH_TYPE_NOR		0
#define FSL_QSPI_FLASH_TYPE_NAND	1

#define FSL_QSPI_OP_IP		0
#define FSL_QSPI_OP_AHB		1

struct fsl_qspi_op {
	u8 cmd;
	u8 cmd_nbits;
	u8 n_addr;
	u8 addr_nbits;
	u8 dummy_bytes;
	u32 addr;
	u32 n_tx;
	const u32 *tx_buf;
	u32 n_rx;
	u8 *rx_buf;
	u8 data_nbits;
	u8 flash_type;
	u8 type;
};

static inline int needs_swap_endian(struct fsl_qspi *q)
{
	return q->devtype_data->driver_data & QUADSPI_QUIRK_SWAP_ENDIAN;
}

static inline int needs_4x_clock(struct fsl_qspi *q)
{
	return q->devtype_data->driver_data & QUADSPI_QUIRK_4X_INT_CLK;
}

static inline int needs_fill_txfifo(struct fsl_qspi *q)
{
	return q->devtype_data->driver_data & QUADSPI_QUIRK_TKT253890;
}

static inline int needs_wakeup_wait_mode(struct fsl_qspi *q)
{
	return q->devtype_data->driver_data & QUADSPI_QUIRK_TKT245618;
}

/*
 * R/W functions for big- or little-endian registers:
 * The qSPI controller's endian is independent of the CPU core's endian.
 * So far, although the CPU core is little-endian but the qSPI have two
 * versions for big-endian and little-endian.
 */
static void qspi_writel(struct fsl_qspi *q, u32 val, void __iomem *addr)
{
	if (q->big_endian)
		iowrite32be(val, addr);
	else
		iowrite32(val, addr);
}

static u32 qspi_readl(struct fsl_qspi *q, void __iomem *addr)
{
	if (q->big_endian)
		return ioread32be(addr);
	else
		return ioread32(addr);
}

/*
 * An IC bug makes us to re-arrange the 32-bit data.
 * The following chips, such as IMX6SLX, have fixed this bug.
 */
static inline u32 fsl_qspi_endian_xchg(struct fsl_qspi *q, u32 a)
{
	return needs_swap_endian(q) ? __swab32(a) : a;
}

static inline void fsl_qspi_unlock_lut(struct fsl_qspi *q)
{
	qspi_writel(q, QUADSPI_LUTKEY_VALUE, q->iobase + QUADSPI_LUTKEY);
	qspi_writel(q, QUADSPI_LCKER_UNLOCK, q->iobase + QUADSPI_LCKCR);
}

static inline void fsl_qspi_lock_lut(struct fsl_qspi *q)
{
	qspi_writel(q, QUADSPI_LUTKEY_VALUE, q->iobase + QUADSPI_LUTKEY);
	qspi_writel(q, QUADSPI_LCKER_LOCK, q->iobase + QUADSPI_LCKCR);
}

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

static int fsl_qspi_set_lut_entry(struct fsl_qspi *q, struct fsl_qspi_op *op, u8 entry)
{
	void __iomem *base = q->iobase;
	int rxfifo = q->devtype_data->rxfifo;
	u32 entry_base = entry * 4;
	u32 lut_reg;
	u32 ntx = op->n_tx, nrx = op->n_rx;

	fsl_qspi_unlock_lut(q);

	/* The first instruction is the command */
	lut_reg = LUT0(CMD, fls(op->cmd_nbits - 1), op->cmd);

	/* The second instruction could be ADDR, WRITE, READ or STOP */
	if(op->n_addr)
		lut_reg |= LUT1(ADDR, fls(op->addr_nbits - 1), op->n_addr * 8);
	else if(ntx) {
		lut_reg |= LUT1(FSL_WRITE, fls(op->data_nbits - 1), ntx * 8);
		ntx = 0;
	}
	else if(nrx) {
		lut_reg |= LUT1(FSL_READ, fls(op->data_nbits - 1), nrx);
		nrx = 0;
	}

	qspi_writel(q, lut_reg, base + QUADSPI_LUT(entry_base));
	lut_reg = 0;

	if(ntx || nrx) {
		/* The third instruction could be WRITE, READ, DUMMY or STOP */
		if(ntx)
			lut_reg = LUT0(FSL_WRITE, fls(op->data_nbits - 1), 0);
		else if(nrx && op->dummy_bytes) {
			lut_reg = LUT0(DUMMY, fls(op->addr_nbits - 1),
				       op->dummy_bytes * 8);
		} else if(nrx) {
			lut_reg = LUT0(FSL_READ, fls(op->data_nbits - 1), nrx);
			nrx = 0;
		}

		/* The fourth instruction could only be READ or STOP */
		if(nrx) {
			lut_reg |= LUT1(FSL_READ, fls(op->data_nbits - 1),
					rxfifo);
		}
	}
	qspi_writel(q, lut_reg, base + QUADSPI_LUT(entry_base + 1));
	/*
	 * The current commandsets for NOR and NAND do not use more than four
	 * instructions per operation, therefore the second half of the LUT
	 * entry remains empty
	 */

	fsl_qspi_lock_lut(q);

	if(entry == SPINAND_SEQID_AHB)
		q->cached_ahb_command = op->cmd;
	else if(entry == SPINAND_SEQID_IP)
		q->cached_ip_command = op->cmd;

	return 0;
}

/* 
 * Get the SEQID for the command and fill
 * the respective LUT entry if necessary.
 */
static int fsl_qspi_get_seqid(struct fsl_qspi *q, struct fsl_qspi_op *op)
{
	switch (op->cmd) {
	case SPINOR_OP_READ_1_1_4:
	case SPINOR_OP_WREN:
	case SPINOR_OP_WRDI:
	case SPINOR_OP_RDSR:
	case SPINOR_OP_SE:
	case SPINOR_OP_BE_4K:
	case SPINOR_OP_CHIP_ERASE:
	case SPINOR_OP_PP:
	case SPINOR_OP_RDID:
	case SPINOR_OP_WRSR:
	case SPINOR_OP_RDCR:
	case SPINOR_OP_EN4B:
	case SPINOR_OP_BRWR:
		break;
	default:
		dev_err(q->dev, "Unsupported cmd 0x%.2x\n", op->cmd);
		return -EINVAL;
	}
	
	if (op->type == FSL_QSPI_OP_AHB) {
	/*
	 * Read commands that use AHB read use a separate entry in the LUT,
	 * as they are triggered by the controller and should not be overridden
	 * to sustain read performance and avoid collisions with other commands.
	 */
		if(q->cached_ahb_command != op->cmd)
			fsl_qspi_set_lut_entry(q, op, SPINAND_SEQID_AHB);
		return SPINAND_SEQID_AHB;
	} else {
	/*
	 * All other commands are stored in the same LUT entry and are
	 * replaced on each transfer.
	 */
		if(q->cached_ip_command != op->cmd)
			fsl_qspi_set_lut_entry(q, op, SPINAND_SEQID_IP);
		return SPINAND_SEQID_IP;
	}

	return 0;
}

static int
fsl_qspi_runcmd(struct fsl_qspi *q, struct fsl_qspi_op *op)
{
	void __iomem *base = q->iobase;
	int seqid;
	u32 reg, reg2;
	int err;

	init_completion(&q->c);
	dev_dbg(q->dev, "to 0x%.8x:0x%.8x, len:%d, cmd:%.2x\n",
			q->chip_base_addr, op->addr, op->n_tx, op->cmd);

	/* save the reg */
	reg = qspi_readl(q, base + QUADSPI_MCR);

	qspi_writel(q, q->memmap_phy + q->chip_base_addr + op->addr,
			base + QUADSPI_SFAR);
	qspi_writel(q, QUADSPI_RBCT_WMRK_MASK | QUADSPI_RBCT_RXBRD_USEIPS,
			base + QUADSPI_RBCT);
	qspi_writel(q, reg | QUADSPI_MCR_CLR_RXF_MASK, base + QUADSPI_MCR);

	do {
		reg2 = qspi_readl(q, base + QUADSPI_SR);
		if (reg2 & (QUADSPI_SR_IP_ACC_MASK | QUADSPI_SR_AHB_ACC_MASK)) {
			udelay(1);
			dev_dbg(q->dev, "The controller is busy, 0x%x\n", reg2);
			continue;
		}
		break;
	} while (1);

	/* trigger the LUT now */
	seqid = fsl_qspi_get_seqid(q, op);
	qspi_writel(q, (seqid << QUADSPI_IPCR_SEQID_SHIFT) | op->n_tx,
			base + QUADSPI_IPCR);

	/* Wait for the interrupt. */
	if (!wait_for_completion_timeout(&q->c, msecs_to_jiffies(1000))) {
		dev_err(q->dev,
			"cmd 0x%.2x timeout, addr@%.8x, FR:0x%.8x, SR:0x%.8x\n",
			op->cmd, op->addr, qspi_readl(q, base + QUADSPI_FR),
			qspi_readl(q, base + QUADSPI_SR));
		err = -ETIMEDOUT;
	} else {
		err = 0;
	}

	/* restore the MCR */
	qspi_writel(q, reg, base + QUADSPI_MCR);

	return err;
}

/* Read out the data from the QUADSPI_RBDR buffer registers. */
static void fsl_qspi_read_data(struct fsl_qspi *q, struct fsl_qspi_op *op)
{
	u32 tmp;
	u8 *rxbuf = op->rx_buf;
	int len = op->n_rx;
	int i = 0;

	while (len > 0) {
		tmp = qspi_readl(q, q->iobase + QUADSPI_RBDR + i * 4);
		tmp = fsl_qspi_endian_xchg(q, tmp);
		dev_dbg(q->dev, "chip addr:0x%.8x, rcv:0x%.8x\n",
				q->chip_base_addr, tmp);

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

/*
 * If we have changed the content of the flash by writing or erasing,
 * we need to invalidate the AHB buffer. If we do not do so, we may read out
 * the wrong data. The spec tells us reset the AHB domain and Serial Flash
 * domain at the same time.
 */
static inline void fsl_qspi_invalid(struct fsl_qspi *q)
{
	u32 reg;

	reg = qspi_readl(q, q->iobase + QUADSPI_MCR);
	reg |= QUADSPI_MCR_SWRSTHD_MASK | QUADSPI_MCR_SWRSTSD_MASK;
	qspi_writel(q, reg, q->iobase + QUADSPI_MCR);

	/*
	 * The minimum delay : 1 AHB + 2 SFCK clocks.
	 * Delay 1 us is enough.
	 */
	udelay(1);

	reg &= ~(QUADSPI_MCR_SWRSTHD_MASK | QUADSPI_MCR_SWRSTSD_MASK);
	qspi_writel(q, reg, q->iobase + QUADSPI_MCR);
}

static ssize_t fsl_qspi_write(struct fsl_qspi *q, struct fsl_qspi_op *op)
{
	int ret, i, j;
	u32 tmp;
	u32 *txbuf = (u32 *)op->tx_buf;

	dev_dbg(q->dev, "to 0x%.8x:0x%.8x, len : %d\n",
		q->chip_base_addr, op->addr, op->n_tx);

	/* clear the TX FIFO. */
	tmp = qspi_readl(q, q->iobase + QUADSPI_MCR);
	qspi_writel(q, tmp | QUADSPI_MCR_CLR_TXF_MASK, q->iobase + QUADSPI_MCR);

	/* fill the TX data to the FIFO */
	for (j = 0, i = ((op->n_tx + 3) / 4); j < i; j++) {
		tmp = fsl_qspi_endian_xchg(q, *txbuf);
		qspi_writel(q, tmp, q->iobase + QUADSPI_TBDR);
		txbuf++;
	}

	/* fill the TXFIFO upto 16 bytes for i.MX7d */
	if (needs_fill_txfifo(q))
		for (; i < 4; i++)
			qspi_writel(q, tmp, q->iobase + QUADSPI_TBDR);

	/* Trigger it */
	ret = fsl_qspi_runcmd(q, op);

	if (ret == 0)
		return op->n_tx;

	return ret;
}

static void fsl_qspi_set_map_addr(struct fsl_qspi *q)
{
	int nor_size = q->nor_size;
	void __iomem *base = q->iobase;

	qspi_writel(q, nor_size + q->memmap_phy, base + QUADSPI_SFA1AD);
	qspi_writel(q, nor_size * 2 + q->memmap_phy, base + QUADSPI_SFA2AD);
	qspi_writel(q, nor_size * 3 + q->memmap_phy, base + QUADSPI_SFB1AD);
	qspi_writel(q, nor_size * 4 + q->memmap_phy, base + QUADSPI_SFB2AD);
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
static void fsl_qspi_init_abh_read(struct fsl_qspi *q)
{
	void __iomem *base = q->iobase;

	/* AHB configuration for access buffer 0/1/2 .*/
	qspi_writel(q, QUADSPI_BUFXCR_INVALID_MSTRID, base + QUADSPI_BUF0CR);
	qspi_writel(q, QUADSPI_BUFXCR_INVALID_MSTRID, base + QUADSPI_BUF1CR);
	qspi_writel(q, QUADSPI_BUFXCR_INVALID_MSTRID, base + QUADSPI_BUF2CR);
	/*
	 * Set ADATSZ with the maximum AHB buffer size to improve the
	 * read performance.
	 */
	qspi_writel(q, QUADSPI_BUF3CR_ALLMST_MASK |
			((q->devtype_data->ahb_buf_size / 8)
			<< QUADSPI_BUF3CR_ADATSZ_SHIFT),
			base + QUADSPI_BUF3CR);

	/* We only use the buffer3 */
	qspi_writel(q, 0, base + QUADSPI_BUF0IND);
	qspi_writel(q, 0, base + QUADSPI_BUF1IND);
	qspi_writel(q, 0, base + QUADSPI_BUF2IND);

	/* Set the default lut sequence for AHB Read. */
	qspi_writel(q, SPINAND_SEQID_AHB << QUADSPI_BFGENCR_SEQID_SHIFT,
		q->iobase + QUADSPI_BFGENCR);
}

/* This function was used to prepare and enable QSPI clock */
static int fsl_qspi_clk_prep_enable(struct fsl_qspi *q)
{
	int ret;

	ret = clk_prepare_enable(q->clk_en);
	if (ret)
		return ret;

	ret = clk_prepare_enable(q->clk);
	if (ret) {
		clk_disable_unprepare(q->clk_en);
		return ret;
	}

	if (needs_wakeup_wait_mode(q))
		pm_qos_add_request(&q->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, 0);

	return 0;
}

/* This function was used to disable and unprepare QSPI clock */
static void fsl_qspi_clk_disable_unprep(struct fsl_qspi *q)
{
	if (needs_wakeup_wait_mode(q))
		pm_qos_remove_request(&q->pm_qos_req);

	clk_disable_unprepare(q->clk);
	clk_disable_unprepare(q->clk_en);

}

/* We use this function to do some basic init for spi_nor_scan(). */
static int fsl_qspi_nor_setup(struct fsl_qspi *q)
{
	void __iomem *base = q->iobase;
	u32 reg;
	int ret;
	int i;

	/* disable and unprepare clock to avoid glitch pass to controller */
	fsl_qspi_clk_disable_unprep(q);

	/* the default frequency, we will change it in the future. */
	ret = clk_set_rate(q->clk, 66000000);
	if (ret)
		return ret;

	ret = fsl_qspi_clk_prep_enable(q);
	if (ret)
		return ret;

	/* Reset the module */
	qspi_writel(q, QUADSPI_MCR_SWRSTSD_MASK | QUADSPI_MCR_SWRSTHD_MASK,
		base + QUADSPI_MCR);
	udelay(1);

	/* Clear all the LUT table */
	fsl_qspi_unlock_lut(q);
	for (i = 0; i < QUADSPI_LUT_NUM; i++)
		qspi_writel(q, 0, base + QUADSPI_LUT_BASE + i * 4);
	fsl_qspi_lock_lut(q);

	/* Disable the module */
	qspi_writel(q, QUADSPI_MCR_MDIS_MASK | QUADSPI_MCR_RESERVED_MASK,
			base + QUADSPI_MCR);

	reg = qspi_readl(q, base + QUADSPI_SMPR);
	qspi_writel(q, reg & ~(QUADSPI_SMPR_FSDLY_MASK
			| QUADSPI_SMPR_FSPHS_MASK
			| QUADSPI_SMPR_HSENA_MASK
			| QUADSPI_SMPR_DDRSMP_MASK), base + QUADSPI_SMPR);

	/* Enable the module */
	qspi_writel(q, QUADSPI_MCR_RESERVED_MASK | QUADSPI_MCR_END_CFG_MASK,
			base + QUADSPI_MCR);

	/* clear all interrupt status */
	qspi_writel(q, 0xffffffff, q->iobase + QUADSPI_FR);

	/* enable the interrupt */
	qspi_writel(q, QUADSPI_RSER_TFIE, q->iobase + QUADSPI_RSER);

	return 0;
}

static int fsl_qspi_nor_setup_last(struct fsl_qspi *q)
{
	unsigned long rate = q->clk_rate;
	int ret;

	if (needs_4x_clock(q))
		rate *= 4;

	/* disable and unprepare clock to avoid glitch pass to controller */
	fsl_qspi_clk_disable_unprep(q);

	ret = clk_set_rate(q->clk, rate);
	if (ret)
		return ret;

	ret = fsl_qspi_clk_prep_enable(q);
	if (ret)
		return ret;

	/* Init for AHB read */
	fsl_qspi_init_abh_read(q);

	return 0;
}

static const struct of_device_id fsl_qspi_dt_ids[] = {
	{ .compatible = "fsl,vf610-qspi", .data = (void *)&vybrid_data, },
	{ .compatible = "fsl,imx6sx-qspi", .data = (void *)&imx6sx_data, },
	{ .compatible = "fsl,imx7d-qspi", .data = (void *)&imx7d_data, },
	{ .compatible = "fsl,imx6ul-qspi", .data = (void *)&imx6ul_data, },
	{ .compatible = "fsl,ls1021a-qspi", .data = (void *)&ls1021a_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_qspi_dt_ids);

static void fsl_qspi_set_base_addr(struct fsl_qspi *q, struct spi_nor *nor)
{
	q->chip_base_addr = q->nor_size * (nor - q->nor);
}

static void fsl_qspi_init_op(struct fsl_qspi_op *op, u8 flash_type, u8 op_type)
{
	memset(&op, 0, sizeof(op));
	op->flash_type = flash_type;
	op->type = op_type;
	op->cmd_nbits = 1;
	op->addr_nbits = 1;
	op->data_nbits = 1;
}

static void fsl_qspi_fill_op_nbits_from_nor_proto(struct fsl_qspi_op *op,
						  enum spi_nor_protocol proto)
{
	op->cmd_nbits = spi_nor_get_protocol_inst_nbits(proto);
	op->addr_nbits = spi_nor_get_protocol_addr_nbits(proto);
	op->data_nbits = spi_nor_get_protocol_data_nbits(proto);
}

static int fsl_qspi_nor_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct fsl_qspi *q = nor->priv;
	struct fsl_qspi_op op;
	int ret;

	fsl_qspi_init_op(&op, FSL_QSPI_FLASH_TYPE_NOR, FSL_QSPI_OP_IP);
	op.cmd = opcode;
	op.n_rx = len;
	op.rx_buf = buf;
	fsl_qspi_fill_op_nbits_from_nor_proto(&op, nor->reg_proto);

	ret = fsl_qspi_runcmd(q, &op);
	if (ret)
		return ret;

	fsl_qspi_read_data(q, &op);
	return 0;
}

static int fsl_qspi_nor_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct fsl_qspi *q = nor->priv;
	struct fsl_qspi_op op;
	int ret;

	fsl_qspi_init_op(&op, FSL_QSPI_FLASH_TYPE_NOR, FSL_QSPI_OP_IP);
	op.cmd = opcode;
	fsl_qspi_fill_op_nbits_from_nor_proto(&op, nor->reg_proto);

	if (!buf) {
		op.n_tx = 1;
		ret = fsl_qspi_runcmd(q, &op);
		if (ret)
			return ret;

		if (opcode == SPINOR_OP_CHIP_ERASE)
			fsl_qspi_invalid(q);

	} else if (len > 0) {
		op.tx_buf = (u32 *)buf;
		op.n_tx = len;
		ret = fsl_qspi_write(q, &op);
		if (ret > 0)
			return 0;
	} else {
		dev_err(q->dev, "invalid cmd %d\n", opcode);
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t fsl_qspi_nor_write(struct spi_nor *nor, loff_t to,
			      size_t len, const u_char *buf)
{
	struct fsl_qspi *q = nor->priv;
	struct fsl_qspi_op op;
	ssize_t ret;

	fsl_qspi_init_op(&op, FSL_QSPI_FLASH_TYPE_NOR, FSL_QSPI_OP_IP);
	op.cmd = nor->program_opcode;
	op.addr = to;
	op.tx_buf = (u32 *)buf;
	op.n_tx = len;
	fsl_qspi_fill_op_nbits_from_nor_proto(&op, nor->write_proto);

	ret = fsl_qspi_write(q, &op);

	/* invalid the data in the AHB buffer. */
	fsl_qspi_invalid(q);
	return ret;
}

static ssize_t fsl_qspi_read_ahb(struct fsl_qspi *qspi, struct fsl_qspi_op *op)
{
	/* if necessary,ioremap buffer before AHB read, */
	if (!qspi->ahb_addr) {
		qspi->memmap_offs = qspi->chip_base_addr + op->addr;
		qspi->memmap_len = op->n_rx > QUADSPI_MIN_IOMAP ? op->n_rx : QUADSPI_MIN_IOMAP;

		qspi->ahb_addr = ioremap_nocache(
				qspi->memmap_phy + qspi->memmap_offs,
				qspi->memmap_len);
		if (!qspi->ahb_addr) {
			dev_err(qspi->dev, "ioremap failed\n");
			return -ENOMEM;
		}
	/* ioremap if the data requested is out of range */
	} else if (qspi->chip_base_addr + op->addr < qspi->memmap_offs
			|| qspi->chip_base_addr + op->addr + op->n_rx >
			qspi->memmap_offs + qspi->memmap_len) {
		iounmap(qspi->ahb_addr);

		qspi->memmap_offs = qspi->chip_base_addr + op->addr;
		qspi->memmap_len = op->n_rx > QUADSPI_MIN_IOMAP ? op->n_rx : QUADSPI_MIN_IOMAP;
		qspi->ahb_addr = ioremap_nocache(
				qspi->memmap_phy + qspi->memmap_offs,
				qspi->memmap_len);
		if (!qspi->ahb_addr) {
			dev_err(qspi->dev, "ioremap failed\n");
			return -ENOMEM;
		}
	}

	dev_dbg(qspi->dev, "ahb read from %p, len:%zd\n",
		qspi->ahb_addr + qspi->chip_base_addr + op->addr - qspi->memmap_offs,
		op->n_rx);

	/* Read out the data directly from the AHB buffer.*/
	memcpy(op->rx_buf, qspi->ahb_addr + qspi->chip_base_addr + op->addr - qspi->memmap_offs,
		op->n_rx);

	return op->n_rx;
}

static ssize_t fsl_qspi_nor_read(struct spi_nor *nor, loff_t from,
			     size_t len, u_char *buf)
{
	struct fsl_qspi *q = nor->priv;
	struct fsl_qspi_op op;

	fsl_qspi_init_op(&op, FSL_QSPI_FLASH_TYPE_NOR, FSL_QSPI_OP_AHB);
	op.cmd = nor->read_opcode;
	op.addr = from;
	op.n_rx = len;
	op.rx_buf = buf;
	fsl_qspi_fill_op_nbits_from_nor_proto(&op, nor->read_proto);

	fsl_qspi_get_seqid(q, &op);

	return fsl_qspi_read_ahb(q, &op);
}

static int fsl_qspi_nor_erase(struct spi_nor *nor, loff_t offs)
{
	struct fsl_qspi *q = nor->priv;
	struct fsl_qspi_op op;
	int ret;

	dev_dbg(nor->dev, "%dKiB at 0x%08x:0x%08x\n",
		nor->mtd.erasesize / 1024, q->chip_base_addr, (u32)offs);

	fsl_qspi_init_op(&op, FSL_QSPI_FLASH_TYPE_NOR, FSL_QSPI_OP_IP);
	op.cmd = nor->erase_opcode;
	op.addr = offs;
	ret = fsl_qspi_runcmd(q, &op);
	if (ret)
		return ret;

	fsl_qspi_invalid(q);
	return 0;
}

static int fsl_qspi_nor_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct fsl_qspi *q = nor->priv;
	int ret;

	mutex_lock(&q->lock);

	ret = fsl_qspi_clk_prep_enable(q);
	if (ret)
		goto err_mutex;

	fsl_qspi_set_base_addr(q, nor);
	return 0;

err_mutex:
	mutex_unlock(&q->lock);
	return ret;
}

static void fsl_qspi_nor_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct fsl_qspi *q = nor->priv;

	fsl_qspi_clk_disable_unprep(q);
	mutex_unlock(&q->lock);
}

static int fsl_qspi_probe(struct platform_device *pdev)
{
	const struct spi_nor_hwcaps hwcaps = {
		.mask = SNOR_HWCAPS_READ_1_1_4 |
			SNOR_HWCAPS_PP,
	};
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct fsl_qspi *q;
	struct resource *res;
	struct spi_nor *nor;
	struct mtd_info *mtd;
	int ret, i = 0;

	q = devm_kzalloc(dev, sizeof(*q), GFP_KERNEL);
	if (!q)
		return -ENOMEM;

	q->nor_num = of_get_child_count(dev->of_node);
	if (!q->nor_num || q->nor_num > FSL_QSPI_MAX_CHIP)
		return -ENODEV;

	q->dev = dev;
	q->devtype_data = of_device_get_match_data(dev);
	if (!q->devtype_data)
		return -ENODEV;
	platform_set_drvdata(pdev, q);

	/* find the resources */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "QuadSPI");
	q->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(q->iobase))
		return PTR_ERR(q->iobase);

	q->big_endian = of_property_read_bool(np, "big-endian");
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					"QuadSPI-memory");
	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     res->name)) {
		dev_err(dev, "can't request region for resource %pR\n", res);
		return -EBUSY;
	}

	q->memmap_phy = res->start;

	/* find the clocks */
	q->clk_en = devm_clk_get(dev, "qspi_en");
	if (IS_ERR(q->clk_en))
		return PTR_ERR(q->clk_en);

	q->clk = devm_clk_get(dev, "qspi");
	if (IS_ERR(q->clk))
		return PTR_ERR(q->clk);

	ret = fsl_qspi_clk_prep_enable(q);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		goto clk_failed;
	}

	/* find the irq */
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "failed to get the irq: %d\n", ret);
		goto irq_failed;
	}

	ret = devm_request_irq(dev, ret,
			fsl_qspi_irq_handler, 0, pdev->name, q);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		goto irq_failed;
	}

	ret = fsl_qspi_nor_setup(q);
	if (ret)
		goto irq_failed;

	if (of_get_property(np, "fsl,qspi-has-second-chip", NULL))
		q->has_second_chip = true;

	mutex_init(&q->lock);

	/* iterate the subnodes. */
	for_each_available_child_of_node(dev->of_node, np) {
		/* skip the holes */
		if (!q->has_second_chip)
			i *= 2;

		nor = &q->nor[i];
		mtd = &nor->mtd;

		nor->dev = dev;
		spi_nor_set_flash_node(nor, np);
		nor->priv = q;

		/* fill the hooks */
		nor->read_reg = fsl_qspi_nor_read_reg;
		nor->write_reg = fsl_qspi_nor_write_reg;
		nor->read = fsl_qspi_nor_read;
		nor->write = fsl_qspi_nor_write;
		nor->erase = fsl_qspi_nor_erase;

		nor->prepare = fsl_qspi_nor_prep;
		nor->unprepare = fsl_qspi_nor_unprep;

		ret = of_property_read_u32(np, "spi-max-frequency",
				&q->clk_rate);
		if (ret < 0)
			goto mutex_failed;

		/* set the chip address for READID */
		fsl_qspi_set_base_addr(q, nor);

		ret = spi_nor_scan(nor, NULL, &hwcaps);
		if (ret)
			goto mutex_failed;

		ret = mtd_device_register(mtd, NULL, 0);
		if (ret)
			goto mutex_failed;

		/* Set the correct NOR size now. */
		if (q->nor_size == 0) {
			q->nor_size = mtd->size;

			/* Map the SPI NOR to accessiable address */
			fsl_qspi_set_map_addr(q);
		}

		/*
		 * The TX FIFO is 64 bytes in the Vybrid, but the Page Program
		 * may writes 265 bytes per time. The write is working in the
		 * unit of the TX FIFO, not in the unit of the SPI NOR's page
		 * size.
		 *
		 * So shrink the spi_nor->page_size if it is larger then the
		 * TX FIFO.
		 */
		if (nor->page_size > q->devtype_data->txfifo)
			nor->page_size = q->devtype_data->txfifo;

		i++;
	}

	/* finish the rest init. */
	ret = fsl_qspi_nor_setup_last(q);
	if (ret)
		goto last_init_failed;

	fsl_qspi_clk_disable_unprep(q);
	return 0;

last_init_failed:
	for (i = 0; i < q->nor_num; i++) {
		/* skip the holes */
		if (!q->has_second_chip)
			i *= 2;
		mtd_device_unregister(&q->nor[i].mtd);
	}
mutex_failed:
	mutex_destroy(&q->lock);
irq_failed:
	fsl_qspi_clk_disable_unprep(q);
clk_failed:
	dev_err(dev, "Freescale QuadSPI probe failed\n");
	return ret;
}

static int fsl_qspi_remove(struct platform_device *pdev)
{
	struct fsl_qspi *q = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < q->nor_num; i++) {
		/* skip the holes */
		if (!q->has_second_chip)
			i *= 2;
		mtd_device_unregister(&q->nor[i].mtd);
	}

	/* disable the hardware */
	qspi_writel(q, QUADSPI_MCR_MDIS_MASK, q->iobase + QUADSPI_MCR);
	qspi_writel(q, 0x0, q->iobase + QUADSPI_RSER);

	mutex_destroy(&q->lock);

	if (q->ahb_addr)
		iounmap(q->ahb_addr);

	return 0;
}

static int fsl_qspi_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int fsl_qspi_resume(struct platform_device *pdev)
{
	int ret;
	struct fsl_qspi *q = platform_get_drvdata(pdev);

	ret = fsl_qspi_clk_prep_enable(q);
	if (ret)
		return ret;

	fsl_qspi_nor_setup(q);
	fsl_qspi_set_map_addr(q);
	fsl_qspi_nor_setup_last(q);

	fsl_qspi_clk_disable_unprep(q);

	return 0;
}

static struct platform_driver fsl_qspi_driver = {
	.driver = {
		.name	= "fsl-quadspi",
		.bus	= &platform_bus_type,
		.of_match_table = fsl_qspi_dt_ids,
	},
	.probe          = fsl_qspi_probe,
	.remove		= fsl_qspi_remove,
	.suspend	= fsl_qspi_suspend,
	.resume		= fsl_qspi_resume,
};
module_platform_driver(fsl_qspi_driver);

MODULE_DESCRIPTION("Freescale QuadSPI Controller Driver");
MODULE_AUTHOR("Freescale Semiconductor Inc.");
MODULE_LICENSE("GPL v2");
