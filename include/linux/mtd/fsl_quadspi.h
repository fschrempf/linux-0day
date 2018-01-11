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
#ifndef __LINUX_MTD_FSL_QUADSPI_H
#define __LINUX_MTD_FSL_QUADSPI_H

#include <linux/pm_qos.h>
#include <linux/io.h>


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
#define LUT_FSL_READ_DDR	14
#define LUT_FSL_WRITE_DDR	15
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
	void __iomem *iobase;
	void __iomem *ahb_addr;
	u32 memmap_phy;
	u32 memmap_offs;
	u32 memmap_len;
	struct clk *clk, *clk_en;
	struct device *dev;
	struct completion c;
	const struct fsl_qspi_devtype_data *devtype_data;
	u32 clk_rate;
	unsigned int chip_base_addr; /* We may support two chips. */
	bool has_second_chip;
	bool big_endian;
	struct mutex lock;
	struct pm_qos_request pm_qos_req;
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
 * An IC bug makes us to re-arrange the 32-bit data.
 * The following chips, such as IMX6SLX, have fixed this bug.
 */
static inline u32 fsl_qspi_endian_xchg(struct fsl_qspi *q, u32 a)
{
	return needs_swap_endian(q) ? __swab32(a) : a;
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

#endif /* __LINUX_MTD_FSL_QUADSPI_H */
