// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 exceet electronics GmbH
 *
 * Authors:
 *	Frieder Schrempf <frieder.schrempf@exceet.de>
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mtd/spinand.h>

#define SPINAND_MFR_WINBOND		0xEF

struct winbond_spinand_info {
	char *name;
	u8 dev_id;
	struct nand_memory_organization memorg;
	struct nand_ecc_req eccreq;
	unsigned int rw_modes;
};

#define WINBOND_SPI_NAND_INFO(nm, did, mo, er, rwm)			\
	{								\
		.name = (nm),						\
		.dev_id = (did),					\
		.memorg = mo,						\
		.eccreq = er,						\
		.rw_modes = (rwm)					\
	}

static const struct winbond_spinand_info winbond_spinand_table[] = {
	WINBOND_SPI_NAND_INFO("W25M02GV", 0xAB,
			     NAND_MEMORG(1, 2048, 64, 64, 1024, 1, 1, 2),
			     NAND_ECCREQ(8, 512),
			     SPINAND_CAP_RD_X1 | SPINAND_CAP_RD_X2 |
			     SPINAND_CAP_RD_X4 | SPINAND_CAP_RD_DUAL |
			     SPINAND_CAP_RD_QUAD |
			     SPINAND_CAP_WR_X1 | SPINAND_CAP_WR_X4),
};

static int winbond_spinand_get_dummy(struct spinand_device *spinand,
				     struct spinand_op *op)
{
	u8 opcode = op->cmd;

	switch (opcode) {
	case SPINAND_CMD_READ_FROM_CACHE:
	case SPINAND_CMD_READ_FROM_CACHE_FAST:
	case SPINAND_CMD_READ_FROM_CACHE_X2:
	case SPINAND_CMD_READ_FROM_CACHE_DUAL_IO:
	case SPINAND_CMD_READ_FROM_CACHE_X4:
	case SPINAND_CMD_READ_ID:
		return 1;

	case SPINAND_CMD_READ_FROM_CACHE_QUAD_IO:
		return 2;

	default:
		return 0;
	}
}

static bool winbond_spinand_scan_id_table(struct spinand_device *spinand,
					  u8 dev_id)
{
	struct mtd_info *mtd = spinand_to_mtd(spinand);
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct winbond_spinand_info *item;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(winbond_spinand_table); i++) {
		item = (struct winbond_spinand_info *)winbond_spinand_table + i;
		if (dev_id != item->dev_id)
			continue;

		nand->memorg = item->memorg;
		nand->eccreq = item->eccreq;
		spinand->rw_modes = item->rw_modes;

		return true;
	}

	return false;
}

static bool winbond_spinand_detect(struct spinand_device *spinand)
{
	u8 *id = spinand->id.data;

	/*
	 * Winbond SPI NAND read ID needs a dummy byte,
	 * so the first byte in raw_id is a dummy.
	 */
	if (id[1] != SPINAND_MFR_WINBOND)
		return false;

	return winbond_spinand_scan_id_table(spinand, id[2]);
}

static void winbond_spinand_set_target_select_op(struct spinand_device *spinand,
						 struct spinand_op *op,
						 const u8 target)
{
	/* 
	 * To select a target/die run the special command with one argument
	 * (address of the die)
	 */
	op->cmd = SPINAND_CMD_TARGET_SELECT;
	op->n_addr = 1;
	op->addr[0] = target;
}

static const struct spinand_manufacturer_ops winbond_spinand_manuf_ops = {
	.detect = winbond_spinand_detect,
	.set_target_select_op = winbond_spinand_set_target_select_op,
};

const struct spinand_manufacturer winbond_spinand_manufacturer = {
	.id = SPINAND_MFR_WINBOND,
	.name = "Winbond",
	.ops = &winbond_spinand_manuf_ops,
};
