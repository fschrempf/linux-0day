/*
 *
 * Copyright (c) 2016-2017 Micron Technology, Inc.
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

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mtd/spinand.h>

#define SPINAND_MFR_MICRON		0x2C

struct micron_spinand_info {
	char *name;
	u8 dev_id;
	struct nand_memory_organization memorg;
	struct nand_ecc_req eccreq;
	unsigned int rw_mode;
};

#define MICRON_SPI_NAND_INFO(nm, did, mo, er, rwm)			\
	{								\
		.name = (nm),						\
		.dev_id = (did),					\
		.memorg = mo,						\
		.eccreq = er,						\
		.rw_mode = (rwm)					\
	}

static const struct micron_spinand_info micron_spinand_table[] = {
	MICRON_SPI_NAND_INFO("MT29F2G01ABAGD", 0x24,
			     NAND_MEMORG(1, 2048, 128, 64, 2048, 2, 1, 1),
			     NAND_ECCREQ(8, 512),
			     SPINAND_RW_COMMON),
};

static int micron_spinand_get_dummy(struct spinand_device *spinand,
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

/**
 * micron_spinand_scan_id_table - scan SPI NAND info in id table
 * @spinand: SPI NAND device structure
 * @id: point to manufacture id and device id
 * Description:
 *   If found in id table, config device with table information.
 */
static bool micron_spinand_scan_id_table(struct spinand_device *spinand,
					 u8 dev_id)
{
	struct mtd_info *mtd = spinand_to_mtd(spinand);
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct micron_spinand_info *item;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(micron_spinand_table); i++) {
		item = (struct micron_spinand_info *)micron_spinand_table + i;
		if (dev_id != item->dev_id)
			continue;

		nand->memorg = item->memorg;
		nand->eccreq = item->eccreq;
		spinand->rw_mode = item->rw_mode;

		return true;
	}

	return false;
}

/**
 * micron_spinand_detect - initialize device related part in spinand_device
 * struct if it is Micron device.
 * @spinand: SPI NAND device structure
 */
static bool micron_spinand_detect(struct spinand_device *spinand)
{
	u8 *id = spinand->id.data;

	/*
	 * Micron SPI NAND read ID need a dummy byte,
	 * so the first byte in raw_id is dummy.
	 */
	if (id[1] != SPINAND_MFR_MICRON)
		return false;

	return micron_spinand_scan_id_table(spinand, id[2]);
}

/**
 * micron_spinand_prepare_op - Fix address for cache operation.
 * @spinand: SPI NAND device structure
 * @op: pointer to spinand_op struct
 * @page: page address
 * @column: column address
 */
static void micron_spinand_adjust_cache_op(struct spinand_device *spinand,
					   const struct nand_page_io_req *req,
					   struct spinand_op *op)
{
	struct nand_device *nand = spinand_to_nand(spinand);
	unsigned int shift;

	/*
	 * No need to specify the plane number if there's only one plane per
	 * LUN.
	 */
	if (nand->memorg.planes_per_lun < 2)
		return;

	/* The plane number is passed in MSB just above the column address */
	shift = fls(nand->memorg.pagesize);
	op->addr[(16 - shift) / 8] |= req->pos.plane << (shift % 8);
	op->dummy_bytes = micron_spinand_get_dummy(spinand, op);
}

static const struct spinand_manufacturer_ops micron_spinand_manuf_ops = {
	.detect = micron_spinand_detect,
	.adjust_cache_op = micron_spinand_adjust_cache_op,
};

const struct spinand_manufacturer micron_spinand_manufacturer = {
	.id = SPINAND_MFR_MICRON,
	.name = "Micron",
	.ops = &micron_spinand_manuf_ops,
};
