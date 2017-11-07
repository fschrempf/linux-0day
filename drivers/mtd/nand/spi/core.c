// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016-2017 Micron Technology, Inc.
 *
 * Authors:
 *	Peter Pan <peterpandong@micron.com>
 */

#define pr_fmt(fmt)	"spi-nand: " fmt

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/mtd/spinand.h>
#include <linux/slab.h>
#include <linux/of.h>

static inline void spinand_adjust_cache_op(struct spinand_device *spinand,
					   const struct nand_page_io_req *req,
					   struct spinand_op *op)
{
	if (!spinand->manufacturer.manu->ops->adjust_cache_op)
		return;

	spinand->manufacturer.manu->ops->adjust_cache_op(spinand, req, op);
}

static inline int spinand_exec_op(struct spinand_device *spinand,
				  struct spinand_op *op)
{
	return spinand->controller.controller->ops->exec_op(spinand, op);
}

static inline void spinand_op_init(struct spinand_op *op)
{
	memset(op, 0, sizeof(struct spinand_op));
	op->addr_nbits = 1;
	op->data_nbits = 1;
}

static int spinand_read_reg_op(struct spinand_device *spinand, u8 reg, u8 *val)
{
	struct spinand_op op;
	int ret;

	spinand_op_init(&op);
	op.cmd = SPINAND_CMD_GET_FEATURE;
	op.n_addr = 1;
	op.addr[0] = reg;
	op.n_rx = 1;
	op.rx_buf = val;

	ret = spinand_exec_op(spinand, &op);
	if (ret < 0)
		pr_err("failed to read register %d (err = %d)\n", reg, ret);

	return ret;
}

static int spinand_write_reg_op(struct spinand_device *spinand, u8 reg, u8 val)
{
	struct spinand_op op;
	int ret;

	spinand_op_init(&op);
	op.cmd = SPINAND_CMD_SET_FEATURE;
	op.n_addr = 1;
	op.addr[0] = reg;
	op.n_tx = 1;
	op.tx_buf = &val;

	ret = spinand_exec_op(spinand, &op);
	if (ret < 0)
		pr_err("failed to write register %d (err = %d)\n", reg, ret);

	return ret;
}

static int spinand_read_status(struct spinand_device *spinand, u8 *status)
{
	return spinand_read_reg_op(spinand, REG_STATUS, status);
}

static int spinand_get_cfg(struct spinand_device *spinand, u8 *cfg)
{
	return spinand_read_reg_op(spinand, REG_CFG, cfg);
}

static int spinand_set_cfg(struct spinand_device *spinand, u8 cfg)
{
	return spinand_write_reg_op(spinand, REG_CFG, cfg);
}

static void spinand_disable_ecc(struct spinand_device *spinand)
{
	u8 cfg = 0;

	spinand_get_cfg(spinand, &cfg);

	if ((cfg & CFG_ECC_MASK) == CFG_ECC_ENABLE) {
		cfg &= ~CFG_ECC_ENABLE;
		spinand_set_cfg(spinand, cfg);
	}
}

static int spinand_write_enable_op(struct spinand_device *spinand)
{
	struct spinand_op op;

	spinand_op_init(&op);
	op.cmd = SPINAND_CMD_WR_ENABLE;

	return spinand_exec_op(spinand, &op);
}

static int spinand_load_page_op(struct spinand_device *spinand,
				const struct nand_page_io_req *req)
{
	struct nand_device *nand = &spinand->base;
	unsigned int row = nanddev_pos_to_row(nand, &req->pos);
	struct spinand_op op;

	spinand_op_init(&op);
	op.cmd = SPINAND_CMD_PAGE_READ;
	op.n_addr = 3;
	op.addr[0] = row >> 16;
	op.addr[1] = row >> 8;
	op.addr[2] = row;

	return spinand_exec_op(spinand, &op);
}

static int spinand_get_address_bits(u8 opcode)
{
	switch (opcode) {
	case SPINAND_CMD_READ_FROM_CACHE_QUAD_IO:
		return 4;
	case SPINAND_CMD_READ_FROM_CACHE_DUAL_IO:
		return 2;
	default:
		return 1;
	}
}

static int spinand_get_data_bits(u8 opcode)
{
	switch (opcode) {
	case SPINAND_CMD_READ_FROM_CACHE_QUAD_IO:
	case SPINAND_CMD_READ_FROM_CACHE_X4:
	case SPINAND_CMD_PROG_LOAD_X4:
	case SPINAND_CMD_PROG_LOAD_RDM_DATA_X4:
		return 4;
	case SPINAND_CMD_READ_FROM_CACHE_DUAL_IO:
	case SPINAND_CMD_READ_FROM_CACHE_X2:
		return 2;
	default:
		return 1;
	}
}

static int spinand_read_from_cache_op(struct spinand_device *spinand,
				      const struct nand_page_io_req *req)
{
	struct nand_device *nand = &spinand->base;
	struct nand_page_io_req adjreq = *req;
	struct spinand_op op;
	u16 column = 0;
	int ret;

	spinand_op_init(&op);
	op.cmd = spinand->read_cache_op;
	op.n_addr = 2;
	op.addr_nbits = spinand_get_address_bits(spinand->read_cache_op);

	if (req->datalen) {
		adjreq.datalen = nanddev_page_size(nand);
		adjreq.dataoffs = 0;
		adjreq.databuf.in = spinand->buf;
		op.rx_buf = spinand->buf;
		op.n_rx = adjreq.datalen;
	}

	if (req->ooblen) {
		adjreq.ooblen = nanddev_per_page_oobsize(nand);
		adjreq.ooboffs = 0;
		adjreq.oobbuf.in = spinand->oobbuf;
		op.n_rx += nanddev_per_page_oobsize(nand);
		if (!op.rx_buf) {
			op.rx_buf = spinand->oobbuf;
			column = nanddev_page_size(nand);
		}
	}

	op.addr[0] = column >> 8;
	op.addr[1] = column;
	op.data_nbits = spinand_get_data_bits(spinand->read_cache_op);
	spinand_adjust_cache_op(spinand, &adjreq, &op);

	ret = spinand_exec_op(spinand, &op);
	if (ret)
		return ret;

	if (req->datalen)
		memcpy(req->databuf.in, spinand->buf + req->dataoffs,
		       req->datalen);

	if (req->ooblen)
		memcpy(req->oobbuf.in, spinand->oobbuf + req->ooboffs,
		       req->ooblen);

	return 0;
}

static int spinand_write_to_cache_op(struct spinand_device *spinand,
				     const struct nand_page_io_req *req)
{
	struct nand_device *nand = &spinand->base;
	struct nand_page_io_req adjreq = *req;
	struct spinand_op op;
	u16 column = 0;

	spinand_op_init(&op);
	op.cmd = spinand->write_cache_op;
	op.n_addr = 2;

	memset(spinand->buf, 0xff,
	       nanddev_page_size(nand) +
	       nanddev_per_page_oobsize(nand));

	if (req->datalen) {
		memcpy(spinand->buf + req->dataoffs, req->databuf.out,
		       req->datalen);
		adjreq.dataoffs = 0;
		adjreq.datalen = nanddev_page_size(nand);
		adjreq.databuf.out = spinand->buf;
		op.tx_buf = spinand->buf;
		op.n_tx = adjreq.datalen;
	}

	if (req->ooblen) {
		memcpy(spinand->oobbuf + req->ooboffs, req->oobbuf.out,
		       req->ooblen);
		adjreq.ooblen = nanddev_per_page_oobsize(nand);
		adjreq.ooboffs = 0;
		op.n_tx += nanddev_per_page_oobsize(nand);
		if (!op.tx_buf) {
			op.tx_buf = spinand->oobbuf;
			column = nanddev_page_size(nand);
		}
	}

	op.addr[0] = column >> 8;
	op.addr[1] = column;
	op.addr_nbits = spinand_get_address_bits(spinand->write_cache_op);
	op.data_nbits = spinand_get_data_bits(spinand->write_cache_op);
	spinand_adjust_cache_op(spinand, &adjreq, &op);

	return spinand_exec_op(spinand, &op);
}

static int spinand_program_op(struct spinand_device *spinand,
			      const struct nand_page_io_req *req)
{
	struct nand_device *nand = spinand_to_nand(spinand);
	unsigned int row = nanddev_pos_to_row(nand, &req->pos);
	struct spinand_op op;

	spinand_op_init(&op);
	op.cmd = SPINAND_CMD_PROG_EXC;
	op.n_addr = 3;
	op.addr[0] = row >> 16;
	op.addr[1] = row >> 8;
	op.addr[2] = row;

	return spinand_exec_op(spinand, &op);
}

static int spinand_erase_op(struct spinand_device *spinand,
			    const struct nand_pos *pos)
{
	struct nand_device *nand = &spinand->base;
	unsigned int row = nanddev_pos_to_row(nand, pos);
	struct spinand_op op;

	spinand_op_init(&op);
	op.cmd = SPINAND_CMD_BLK_ERASE;
	op.n_addr = 3;
	op.addr[0] = row >> 16;
	op.addr[1] = row >> 8;
	op.addr[2] = row;

	return spinand_exec_op(spinand, &op);
}

static int spinand_wait(struct spinand_device *spinand, u8 *s)
{
	unsigned long timeo =  jiffies + msecs_to_jiffies(400);
	u8 status;

	do {
		spinand_read_status(spinand, &status);
		if ((status & STATUS_OIP_MASK) == STATUS_READY)
			goto out;
	} while (time_before(jiffies, timeo));

	/*
	 * Extra read, just in case the STATUS_READY bit has changed
	 * since our last check
	 */
	spinand_read_status(spinand, &status);
out:
	if (s)
		*s = status;

	return (status & STATUS_OIP_MASK) == STATUS_READY ? 0 :	-ETIMEDOUT;
}

static int spinand_read_id_op(struct spinand_device *spinand, u8 *buf)
{
	struct spinand_op op;

	spinand_op_init(&op);
	op.cmd = SPINAND_CMD_READ_ID;
	op.n_rx = SPINAND_MAX_ID_LEN;
	op.rx_buf = buf;

	return spinand_exec_op(spinand, &op);
}

static int spinand_reset_op(struct spinand_device *spinand)
{
	struct spinand_op op;
	int ret;

	spinand_op_init(&op);
	op.cmd = SPINAND_CMD_RESET;

	ret = spinand_exec_op(spinand, &op);
	if (ret < 0) {
		pr_err("failed to reset the NAND (err = %d)\n", ret);
		goto out;
	}

	ret = spinand_wait(spinand, NULL);

out:
	return ret;
}

static int spinand_lock_block(struct spinand_device *spinand, u8 lock)
{
	return spinand_write_reg_op(spinand, REG_BLOCK_LOCK, lock);
}

static int spinand_read_page(struct spinand_device *spinand,
			     const struct nand_page_io_req *req)
{
	struct nand_device *nand = spinand_to_nand(spinand);
	int ret;

	spinand_load_page_op(spinand, req);

	ret = spinand_wait(spinand, NULL);
	if (ret < 0) {
		pr_err("failed to load page @%llx (err = %d)\n",
		       nanddev_pos_to_offs(nand, &req->pos), ret);
		return ret;
	}

	spinand_read_from_cache_op(spinand, req);

	return 0;
}

static int spinand_write_page(struct spinand_device *spinand,
			      const struct nand_page_io_req *req)
{
	struct nand_device *nand = spinand_to_nand(spinand);
	u8 status;
	int ret = 0;

	spinand_write_enable_op(spinand);
	spinand_write_to_cache_op(spinand, req);
	spinand_program_op(spinand, req);

	ret = spinand_wait(spinand, &status);
	if (!ret && (status & STATUS_P_FAIL_MASK) == STATUS_P_FAIL)
		ret = -EIO;

	if (ret < 0)
		pr_err("failed to program page @%llx (err = %d)\n",
		       nanddev_pos_to_offs(nand, &req->pos), ret);

	return ret;
}

static int spinand_mtd_read(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops)
{
	struct spinand_device *spinand = mtd_to_spinand(mtd);
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct nand_io_iter iter;
	int ret;

	mutex_lock(&spinand->lock);
	nanddev_io_for_each_page(nand, from, ops, &iter) {
		ret = spinand_read_page(spinand, &iter.req);
		if (ret)
			break;

		ops->retlen += iter.req.datalen;
		ops->oobretlen += iter.req.ooblen;
	}
	mutex_unlock(&spinand->lock);

	return ret;
}

static int spinand_mtd_write(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops)
{
	struct spinand_device *spinand = mtd_to_spinand(mtd);
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct nand_io_iter iter;
	int ret = 0;

	mutex_lock(&spinand->lock);
	nanddev_io_for_each_page(nand, to, ops, &iter) {
		ret = spinand_write_page(spinand, &iter.req);
		if (ret)
			break;

		ops->retlen += iter.req.datalen;
		ops->oobretlen += iter.req.ooblen;
	}
	mutex_unlock(&spinand->lock);

	return ret;
}

static bool spinand_isbad(struct nand_device *nand, const struct nand_pos *pos)
{
	struct spinand_device *spinand = nand_to_spinand(nand);
	struct nand_page_io_req req = {
		.pos = *pos,
		.ooblen = 2,
		.ooboffs = 0,
		.oobbuf.in = spinand->oobbuf,
	};

	memset(spinand->oobbuf, 0, 2);
	spinand_read_page(spinand, &req);
	if (spinand->oobbuf[0] != 0xff || spinand->oobbuf[1] != 0xff)
		return true;

	return false;
}

static int spinand_mtd_block_isbad(struct mtd_info *mtd, loff_t offs)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct spinand_device *spinand = nand_to_spinand(nand);
	struct nand_pos pos;
	int ret;

	nanddev_offs_to_pos(nand, offs, &pos);
	mutex_lock(&spinand->lock);
	ret = nanddev_isbad(nand, &pos);
	mutex_unlock(&spinand->lock);

	return ret;
}

static int spinand_markbad(struct nand_device *nand, const struct nand_pos *pos)
{
	struct spinand_device *spinand = nand_to_spinand(nand);
	struct nand_page_io_req req = {
		.pos = *pos,
		.ooboffs = 0,
		.ooblen = 2,
		.oobbuf.out = spinand->oobbuf,
	};

	/* Erase block before marking it bad. */
	spinand_write_enable_op(spinand);
	spinand_erase_op(spinand, pos);

	memset(spinand->oobbuf, 0, 2);
	return spinand_write_page(spinand, &req);
}

static int spinand_mtd_block_markbad(struct mtd_info *mtd, loff_t offs)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct spinand_device *spinand = nand_to_spinand(nand);
	struct nand_pos pos;
	int ret;

	nanddev_offs_to_pos(nand, offs, &pos);
	mutex_lock(&spinand->lock);
	ret = nanddev_markbad(nand, &pos);
	mutex_unlock(&spinand->lock);

	return ret;
}

static int spinand_erase(struct nand_device *nand, const struct nand_pos *pos)
{
	struct spinand_device *spinand = nand_to_spinand(nand);
	u8 status;
	int ret;

	spinand_write_enable_op(spinand);
	spinand_erase_op(spinand, pos);

	ret = spinand_wait(spinand, &status);

	if (!ret && (status & STATUS_E_FAIL_MASK) == STATUS_E_FAIL)
		ret = -EIO;

	if (ret)
		pr_err("failed to erase block %d (err = %d)\n",
		       pos->eraseblock, ret);

	return ret;
}

static int spinand_mtd_erase(struct mtd_info *mtd,
			     struct erase_info *einfo)
{
	struct spinand_device *spinand = mtd_to_spinand(mtd);
	int ret;

	mutex_lock(&spinand->lock);
	ret = nanddev_mtd_erase(mtd, einfo);
	mutex_unlock(&spinand->lock);

	if (!ret)
		mtd_erase_callback(einfo);

	return ret;
}

static int spinand_mtd_block_isreserved(struct mtd_info *mtd, loff_t offs)
{
	struct spinand_device *spinand = mtd_to_spinand(mtd);
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct nand_pos pos;
	int ret;

	nanddev_offs_to_pos(nand, offs, &pos);
	mutex_lock(&spinand->lock);
	ret = nanddev_isreserved(nand, &pos);
	mutex_unlock(&spinand->lock);

	return ret;
}

static void spinand_set_rd_wr_op(struct spinand_device *spinand)
{
	u32 rw_modes = spinand->rw_modes &
		       spinand->controller.controller->caps;

	if (rw_modes & SPINAND_CAP_RD_QUAD)
		spinand->read_cache_op = SPINAND_CMD_READ_FROM_CACHE_QUAD_IO;
	else if (rw_modes & SPINAND_CAP_RD_X4)
		spinand->read_cache_op = SPINAND_CMD_READ_FROM_CACHE_X4;
	else if (rw_modes & SPINAND_CAP_RD_DUAL)
		spinand->read_cache_op = SPINAND_CMD_READ_FROM_CACHE_DUAL_IO;
	else if (rw_modes & SPINAND_CAP_RD_X2)
		spinand->read_cache_op = SPINAND_CMD_READ_FROM_CACHE_X2;
	else
		spinand->read_cache_op = SPINAND_CMD_READ_FROM_CACHE_FAST;

	if (rw_modes & SPINAND_CAP_WR_X4)
		spinand->write_cache_op = SPINAND_CMD_PROG_LOAD_X4;
	else
		spinand->write_cache_op = SPINAND_CMD_PROG_LOAD;
}

static const struct nand_ops spinand_ops = {
	.erase = spinand_erase,
	.markbad = spinand_markbad,
	.isbad = spinand_isbad,
};

static const struct spinand_manufacturer *spinand_manufacturers[] = {
	&micron_spinand_manufacturer,
};

static int spinand_manufacturer_detect(struct spinand_device *spinand)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(spinand_manufacturers); i++) {
		if (spinand_manufacturers[i]->ops->detect(spinand)) {
			spinand->manufacturer.manu = spinand_manufacturers[i];

			return 0;
		}
	}

	return -ENODEV;
}

static int spinand_manufacturer_init(struct spinand_device *spinand)
{
	if (spinand->manufacturer.manu->ops->init)
		return spinand->manufacturer.manu->ops->init(spinand);

	return 0;
}

static void spinand_manufacturer_cleanup(struct spinand_device *spinand)
{
	/* Release manufacturer private data */
	if (spinand->manufacturer.manu->ops->cleanup)
		return spinand->manufacturer.manu->ops->cleanup(spinand);
}

static int spinand_detect(struct spinand_device *spinand)
{
	struct nand_device *nand = &spinand->base;
	int ret;

	spinand_reset_op(spinand);
	spinand_read_id_op(spinand, spinand->id.data);
	spinand->id.len = SPINAND_MAX_ID_LEN;

	ret = spinand_manufacturer_detect(spinand);
	if (ret) {
		pr_err("unknown raw ID %*phN\n",
		       SPINAND_MAX_ID_LEN, spinand->id.data);
		return ret;
	}

	pr_info("%s SPI NAND was found.\n", spinand->manufacturer.manu->name);
	pr_info("%d MiB, block size: %d KiB, page size: %d, OOB size: %d\n",
		(int)(nanddev_size(nand) >> 20),
		nanddev_eraseblock_size(nand) >> 10,
		nanddev_page_size(nand), nanddev_per_page_oobsize(nand));
	return 0;
}

/**
 * devm_spinand_alloc() - allocate SPI NAND device instance
 * @dev: pointer to the parent device (the device owning the newly allocated
 *	 spinand device)
 *
 * Return: a pointer to a spinand device in case of success, an ERR_PTR()
 *	   otherwise. The returned value should be checked with IS_ERR().
 */
struct spinand_device *devm_spinand_alloc(struct device *dev)
{
	struct spinand_device *spinand;
	struct mtd_info *mtd;

	spinand = devm_kzalloc(dev, sizeof(*spinand), GFP_KERNEL);
	if (!spinand)
		return ERR_PTR(-ENOMEM);

	spinand_set_of_node(spinand, dev->of_node);
	mutex_init(&spinand->lock);
	mtd = spinand_to_mtd(spinand);
	mtd->dev.parent = dev;

	return spinand;
}
EXPORT_SYMBOL_GPL(devm_spinand_alloc);

/**
 * spinand_init() - initialize a SPI NAND device
 * @spinand: SPI NAND device
 *
 * Return: 0 in case of success, a negative error code otherwise.
 */
int spinand_init(struct spinand_device *spinand, struct module *owner)
{
	struct mtd_info *mtd = spinand_to_mtd(spinand);
	struct nand_device *nand = mtd_to_nanddev(mtd);
	int ret;

	ret = spinand_detect(spinand);
	if (ret) {
		pr_err("Failed to detect a SPI NAND (err = %d).\n", ret);
		return ret;
	}

	ret = nanddev_init(nand, &spinand_ops, owner);
	if (ret)
		return ret;

	spinand_set_rd_wr_op(spinand);

	/*
	 * Use kzalloc() instead of devm_kzalloc() here, because some drivers
	 * may use this buffer for DMA access.
	 * Memory allocated by devm_ does not guarantee DMA-safe alignment.
	 */
	spinand->buf = kzalloc(nanddev_page_size(nand) +
			       nanddev_per_page_oobsize(nand),
			       GFP_KERNEL);
	if (!spinand->buf)
		return -ENOMEM;

	spinand->oobbuf = spinand->buf + nanddev_page_size(nand);

	ret = spinand_manufacturer_init(spinand);
	if (ret) {
		pr_err("Init of SPI NAND failed (err = %d).\n", ret);
		goto err_free_buf;
	}

	/*
	 * Right now, we don't support ECC, so let the whole oob
	 * area is available for user.
	 */
	mtd->_read_oob = spinand_mtd_read;
	mtd->_write_oob = spinand_mtd_write;
	mtd->_block_isbad = spinand_mtd_block_isbad;
	mtd->_block_markbad = spinand_mtd_block_markbad;
	mtd->_block_isreserved = spinand_mtd_block_isreserved;
	mtd->_erase = spinand_mtd_erase;

	/* After power up, all blocks are locked, so unlock it here. */
	spinand_lock_block(spinand, BL_ALL_UNLOCKED);
	/* Right now, we don't support ECC, so disable on-die ECC */
	spinand_disable_ecc(spinand);

	return 0;

err_free_buf:
	kfree(spinand->buf);
	return ret;
}
EXPORT_SYMBOL_GPL(spinand_init);

/**
 * spinand_cleanup() - cleanup a SPI NAND device
 * @spinand: SPI NAND device
 */
void spinand_cleanup(struct spinand_device *spinand)
{
	struct nand_device *nand = &spinand->base;

	spinand_manufacturer_cleanup(spinand);
	kfree(spinand->buf);
	nanddev_cleanup(nand);
}
EXPORT_SYMBOL_GPL(spinand_cleanup);

MODULE_DESCRIPTION("SPI NAND framework");
MODULE_AUTHOR("Peter Pan<peterpandong@micron.com>");
MODULE_LICENSE("GPL v2");
