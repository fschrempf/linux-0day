/*
 * Copyright (c) 2017 Free Electrons
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
 *
 * Authors:
 *	Boris Brezillon <boris.brezillon@free-electrons.com>
 *	Peter Pan <peterpandong@micron.com>
 */

#define pr_fmt(fmt)	"nand: " fmt

#include <linux/mtd/nand.h>

bool nanddev_isbad(struct nand_device *nand, const struct nand_pos *pos)
{
	if (nanddev_bbt_is_initialized(nand)) {
		unsigned int entry;
		int status;

		entry = nanddev_bbt_pos_to_entry(nand, pos);
		status = nanddev_bbt_get_block_status(nand, entry);
		/* Lazy block status retrieval */
		if (status == NAND_BBT_BLOCK_STATUS_UNKNOWN) {
			if (nand->ops->isbad(nand, pos))
				status = NAND_BBT_BLOCK_FACTORY_BAD;
			else
				status = NAND_BBT_BLOCK_GOOD;

			nanddev_bbt_set_block_status(nand, entry, status);
		}

		if (status == NAND_BBT_BLOCK_WORN ||
		    status == NAND_BBT_BLOCK_FACTORY_BAD)
			return true;

		return false;
	}

	return nand->ops->isbad(nand, pos);
}
EXPORT_SYMBOL_GPL(nanddev_isbad);

/**
 * nanddev_markbad - Write a bad block marker to a block
 * @nand: NAND device
 * @block: block to mark bad
 *
 * Mark a block bad. This function is updating the BBT if available and
 * calls the low-level markbad hook (nand->ops->markbad()) if
 * NAND_BBT_NO_OOB_BBM is not set.
 *
 * Return: 0 in case of success, a negative error code otherwise.
 */
int nanddev_markbad(struct nand_device *nand, const struct nand_pos *pos)
{
	struct mtd_info *mtd = nanddev_to_mtd(nand);
	unsigned int entry;
	int ret = 0;

	if (nanddev_isbad(nand, pos))
		return 0;

	ret = nand->ops->markbad(nand, pos);
	if (ret)
		pr_warn("failed to write BBM to block @%llx (err = %d)\n",
			nanddev_pos_to_offs(nand, pos), ret);

	if (!nanddev_bbt_is_initialized(nand))
		goto out;

	entry = nanddev_bbt_pos_to_entry(nand, pos);
	ret = nanddev_bbt_set_block_status(nand, entry, NAND_BBT_BLOCK_WORN);
	if (ret)
		goto out;

	ret = nanddev_bbt_update(nand);

out:
	if (!ret)
		mtd->ecc_stats.badblocks++;

	return ret;
}
EXPORT_SYMBOL_GPL(nanddev_markbad);

bool nanddev_isreserved(struct nand_device *nand, const struct nand_pos *pos)
{
	unsigned int entry;
	int status;

	if (!nanddev_bbt_is_initialized(nand))
		return false;

	/* Return info from the table */
	entry = nanddev_bbt_pos_to_entry(nand, pos);
	status = nanddev_bbt_get_block_status(nand, entry);
	return status == NAND_BBT_BLOCK_RESERVED;
}
EXPORT_SYMBOL_GPL(nanddev_isreserved);

/**
 * nanddev_erase - Erase a NAND portion
 * @nand: NAND device
 * @block: eraseblock to erase
 *
 * Erase @block block if it's not bad.
 *
 * Return: 0 in case of success, a negative error code otherwise.
 */

int nanddev_erase(struct nand_device *nand, const struct nand_pos *pos)
{
	if (nanddev_isbad(nand, pos) || nanddev_isreserved(nand, pos)) {
		pr_warn("attempt to erase a bad/reserved block @%llx\n",
			nanddev_pos_to_offs(nand, pos));
		return -EIO;
	}

	return nand->ops->erase(nand, pos);
}
EXPORT_SYMBOL_GPL(nanddev_erase);

int nanddev_mtd_erase(struct mtd_info *mtd, struct erase_info *einfo)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct nand_pos pos, last;
	int ret;

	nanddev_offs_to_pos(nand, einfo->addr, &pos);
	nanddev_offs_to_pos(nand, einfo->addr + einfo->len - 1, &last);
	while (nanddev_pos_cmp(&pos, &last) <= 0) {
		ret = nanddev_erase(nand, &pos);
		if (ret) {
			einfo->fail_addr = nanddev_pos_to_offs(nand, &pos);
			einfo->state = MTD_ERASE_FAILED;

			return ret;
		}

		nanddev_pos_next_eraseblock(nand, &pos);
	}

	einfo->state = MTD_ERASE_DONE;

	return 0;
}
EXPORT_SYMBOL_GPL(nanddev_mtd_erase);

/**
 * nanddev_init - Initialize a NAND device
 * @nand: NAND device
 * @memorg: NAND memory organization descriptor
 * @ops: NAND device operations
 *
 * Initialize a NAND device object. Consistency checks are done on @memorg and
 * @ops.
 *
 * Return: 0 in case of success, a negative error code otherwise.
 */
int nanddev_init(struct nand_device *nand, const struct nand_ops *ops,
		 struct module *owner)
{
	struct mtd_info *mtd = nanddev_to_mtd(nand);
	struct nand_memory_organization *memorg = nanddev_get_memorg(nand);

	if (!nand || !ops)
		return -EINVAL;

	if (!ops->erase || !ops->markbad)
		return -EINVAL;

	if (!memorg->bits_per_cell || !memorg->pagesize ||
	    !memorg->pages_per_eraseblock || !memorg->eraseblocks_per_lun ||
	    !memorg->planes_per_lun || !memorg->luns_per_target ||
	    !memorg->ntargets)
		return -EINVAL;

	nand->rowconv.eraseblock_addr_shift = fls(memorg->pagesize);
	nand->rowconv.lun_addr_shift = fls(memorg->eraseblocks_per_lun) +
				       nand->rowconv.eraseblock_addr_shift;

	nand->ops = ops;

	mtd->type = memorg->bits_per_cell == 1 ?
		    MTD_NANDFLASH : MTD_MLCNANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->erasesize = memorg->pagesize * memorg->pages_per_eraseblock;
	mtd->writesize = memorg->pagesize;
	mtd->oobsize = memorg->oobsize;
	mtd->size = nanddev_size(nand);

	return nanddev_bbt_init(nand);
}
EXPORT_SYMBOL_GPL(nanddev_init);

void nanddev_cleanup(struct nand_device *nand)
{
	if (nanddev_bbt_is_initialized(nand))
		nanddev_bbt_cleanup(nand);
}
EXPORT_SYMBOL_GPL(nanddev_cleanup);
