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

#define SPINAND_MFR_WINBOND			0xEF
#define SPINAND_WINBOND_STATUS_ECC_0_BIT	BIT(4)
#define SPINAND_WINBOND_STATUS_ECC_1_BIT	BIT(5)
#define SPINAND_WINBOND_STATUS_ECC_MASK		(SPINAND_WINBOND_STATUS_ECC_0_BIT | \
						SPINAND_WINBOND_STATUS_ECC_1_BIT)

struct winbond_spinand_info {
	char *name;
	u8 dev_id;
	struct nand_memory_organization memorg;
	struct nand_ecc_req eccreq;
	unsigned int rw_modes;
	const struct mtd_ooblayout_ops *ooblayout_ops;
	const struct spinand_ecc_engine_ops *ecc_engine_ops;
};

#define WINBOND_SPI_NAND_INFO(nm, did, mo, er, rwm, oobl, eccops)	\
	{								\
		.name = (nm),						\
		.dev_id = (did),					\
		.memorg = mo,						\
		.eccreq = er,						\
		.rw_modes = (rwm),					\
		.ooblayout_ops = oobl,					\
		.ecc_engine_ops = eccops				\
	}

static int winbond_ooblayout64_ecc(struct mtd_info *mtd, int section,
			      struct mtd_oob_region *oobregion)
{
	if (section > 3)
		return -ERANGE;

	oobregion->length = 8;
	oobregion->offset = 8 + section * 16;

	return 0;
}

static int winbond_ooblayout64_free(struct mtd_info *mtd, int section,
			       struct mtd_oob_region *oobregion)
{
	if (section > 3)
		return -ERANGE;

	oobregion->length = 6;
	oobregion->offset = 2 + section * 16;

	return 0;
}

static const struct mtd_ooblayout_ops winbond_ooblayout64_ops = {
	.ecc = winbond_ooblayout64_ecc,
	.free = winbond_ooblayout64_free,
};

static void winbond_get_ecc_status(struct spinand_device *spinand,
				   unsigned int status, unsigned int *corrected,
				   unsigned int *ecc_error)
{
	unsigned int ecc_status = status & SPINAND_WINBOND_STATUS_ECC_MASK;

	*ecc_error = ((ecc_status & SPINAND_WINBOND_STATUS_ECC_1_BIT) != 0);
	switch (ecc_status) {
	case 0:
		*corrected = 0;
		break;
	case SPINAND_WINBOND_STATUS_ECC_0_BIT:
		*corrected = 4;
		break;
	}
}

static const struct spinand_ecc_engine_ops winbond_ecc_engine_ops = {
	.get_status = winbond_get_ecc_status,
};

static const struct winbond_spinand_info winbond_spinand_table[] = {
	WINBOND_SPI_NAND_INFO("W25M02GV", 0xAB,
			     NAND_MEMORG(1, 2048, 64, 64, 1024, 1, 1, 2),
			     NAND_ECCREQ(1, 512),
			     SPINAND_CAP_RD_X1 | SPINAND_CAP_RD_X2 |
			     SPINAND_CAP_RD_X4 | SPINAND_CAP_RD_DUAL |
			     SPINAND_CAP_RD_QUAD |
			     SPINAND_CAP_WR_X1 | SPINAND_CAP_WR_X4,
			     &winbond_ooblayout64_ops,
			     &winbond_ecc_engine_ops),
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
	struct spinand_ecc_engine *ecc_engine;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(winbond_spinand_table); i++) {
		item = (struct winbond_spinand_info *)winbond_spinand_table + i;
		if (dev_id != item->dev_id)
			continue;

		nand->memorg = item->memorg;
		nand->eccreq = item->eccreq;
		spinand->rw_modes = item->rw_modes;

		if (spinand->ecc.type == NAND_ECC_ON_DIE) {
			ecc_engine = devm_kzalloc(spinand->base.mtd.dev.parent,
						  sizeof(*ecc_engine),
						  GFP_KERNEL);
			if (!ecc_engine) {
				pr_err("failed to allocate ecc engine.\n");
				return false;
			}
			ecc_engine->ops = item->ecc_engine_ops;
			ecc_engine->strength = item->eccreq.strength;
			ecc_engine->steps = item->eccreq.step_size;
			mtd_set_ooblayout(mtd, item->ooblayout_ops);
			spinand->ecc.engine = ecc_engine;
		}

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
