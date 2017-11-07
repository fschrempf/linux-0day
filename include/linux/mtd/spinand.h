/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016-2017 Micron Technology, Inc.
 *
 *  Authors:
 *	Peter Pan <peterpandong@micron.com>
 */
#ifndef __LINUX_MTD_SPINAND_H
#define __LINUX_MTD_SPINAND_H

#include <linux/mutex.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>

/**
 * Standard SPI NAND flash commands
 */
#define SPINAND_CMD_RESET			0xff
#define SPINAND_CMD_GET_FEATURE			0x0f
#define SPINAND_CMD_SET_FEATURE			0x1f
#define SPINAND_CMD_PAGE_READ			0x13
#define SPINAND_CMD_READ_FROM_CACHE		0x03
#define SPINAND_CMD_READ_FROM_CACHE_FAST	0x0b
#define SPINAND_CMD_READ_FROM_CACHE_X2		0x3b
#define SPINAND_CMD_READ_FROM_CACHE_DUAL_IO	0xbb
#define SPINAND_CMD_READ_FROM_CACHE_X4		0x6b
#define SPINAND_CMD_READ_FROM_CACHE_QUAD_IO	0xeb
#define SPINAND_CMD_BLK_ERASE			0xd8
#define SPINAND_CMD_PROG_EXC			0x10
#define SPINAND_CMD_PROG_LOAD			0x02
#define SPINAND_CMD_PROG_LOAD_RDM_DATA		0x84
#define SPINAND_CMD_PROG_LOAD_X4		0x32
#define SPINAND_CMD_PROG_LOAD_RDM_DATA_X4	0x34
#define SPINAND_CMD_READ_ID			0x9f
#define SPINAND_CMD_WR_DISABLE			0x04
#define SPINAND_CMD_WR_ENABLE			0x06

/* feature register */
#define REG_BLOCK_LOCK		0xa0
#define REG_CFG			0xb0
#define REG_STATUS		0xc0

/* status register */
#define STATUS_OIP_MASK		BIT(0)
#define STATUS_CRBSY_MASK	BIT(7)
#define STATUS_READY		0
#define STATUS_BUSY		BIT(0)

#define STATUS_E_FAIL_MASK	BIT(2)
#define STATUS_E_FAIL		BIT(2)

#define STATUS_P_FAIL_MASK	BIT(3)
#define STATUS_P_FAIL		BIT(3)

/* configuration register */
#define CFG_ECC_MASK		BIT(4)
#define CFG_ECC_ENABLE		BIT(4)

/* block lock register */
#define BL_ALL_UNLOCKED		0X00

struct spinand_op;
struct spinand_device;

#define SPINAND_MAX_ID_LEN	4

/**
 * struct spinand_id - SPI NAND id structure
 * @data: buffer containing the id bytes. Currently 4 bytes large, but can
 *	  be extended if required.
 * @len: ID length
 *
 * struct_spinand_id->data contains all bytes returned after a READ_ID command,
 * including dummy bytes if the chip does not emit ID bytes right after the
 * READ_ID command. The responsibility to extract real ID bytes is left to
 * struct_manufacurer_ops->detect().
 */
struct spinand_id {
	u8 data[SPINAND_MAX_ID_LEN];
	int len;
};

/**
 * struct spinand_controller_ops - SPI NAND controller operations
 * @exec_op: execute a SPI NAND operation
 */
struct spinand_controller_ops {
	int (*exec_op)(struct spinand_device *spinand,
		       struct spinand_op *op);
};

/**
 * struct manufacurer_ops - SPI NAND manufacturer specific operations
 * @detect: detect a SPI NAND device. Every time a SPI NAND device is probed
 *	    the core calls the struct_manufacurer_ops->detect() hook of each
 *	    registered manufacturer until one of them return true. Note that
 *	    the first thing to check in this hook is that the manufacturer ID
 *	    in struct_spinand_device->id matches the manufacturer whose
 *	    ->detect() hook has been called. Should return true if there's a
 *	    match, false otherwise. When true is returned, the core assumes
 *	    that properties of the NAND chip (spinand->base.memorg and
 *	    spinand->base.eccreq) have been filled.
 * @init: initialize a SPI NAND device
 * @cleanup: cleanup a SPI NAND device
 * @adjust_cache_op: adjust a cache read/write operation. The manufacturer
 *		     driver can for example tweak the address cycles to pass
 *		     a plane ID
 */
struct spinand_manufacturer_ops {
	bool (*detect)(struct spinand_device *spinand);
	int (*init)(struct spinand_device *spinand);
	void (*cleanup)(struct spinand_device *spinand);
	void (*adjust_cache_op)(struct spinand_device *spinand,
				const struct nand_page_io_req *req,
				struct spinand_op *op);
};

/**
 * struct spinand_manufacturer - SPI NAND manufacturer instance
 * @id: manufacturer ID
 * @name: manufacturer name
 * @ops: manufacturer operations
 */
struct spinand_manufacturer {
	u8 id;
	char *name;
	const struct spinand_manufacturer_ops *ops;
};

extern const struct spinand_manufacturer micron_spinand_manufacturer;

#define SPINAND_CAP_RD_X1	BIT(0)
#define SPINAND_CAP_RD_X2	BIT(1)
#define SPINAND_CAP_RD_X4	BIT(2)
#define SPINAND_CAP_RD_DUAL	BIT(3)
#define SPINAND_CAP_RD_QUAD	BIT(4)
#define SPINAND_CAP_WR_X1	BIT(5)
#define SPINAND_CAP_WR_X2	BIT(6)
#define SPINAND_CAP_WR_X4	BIT(7)
#define SPINAND_CAP_WR_DUAL	BIT(8)
#define SPINAND_CAP_WR_QUAD	BIT(9)

/**
 * struct spinand_controller - SPI NAND controller instance
 * @ops: controller operations
 * @caps: controller capabilities (should be a combination of SPINAND_CAP_XXX
 *	  flags)
 */
struct spinand_controller {
	struct spinand_controller_ops *ops;
	u32 caps;
};

/**
 * struct spinand_device - SPI NAND device instance
 * @base: NAND device instance
 * @lock: lock used to serialize accesses to the NAND
 * @id: NAND ID as returned by READ_ID
 * @read_cache_op: opcode for the "read from cache" operation
 * @write_cache_op: opcode for the "write to cache" operation
 * @buf: bounce buffer for data
 * @oobbuf: bounce buffer for OOB data
 * @rw_modes: supported read/write mode (combination of SPINAND_CAP_XXX flags)
 * @controller: SPI NAND controller information
 * @manufacturer: SPI NAND manufacturer information
 */
struct spinand_device {
	struct nand_device base;
	struct mutex lock;
	struct spinand_id id;
	u8 read_cache_op;
	u8 write_cache_op;
	u8 *buf;
	u8 *oobbuf;
	u32 rw_modes;
	struct {
		struct spinand_controller *controller;
		void *priv;
	} controller;
	struct {
		const struct spinand_manufacturer *manu;
		void *priv;
	} manufacturer;
};

/**
 * mtd_to_spinand() - Get the SPI NAND device attached to an MTD instance
 * @mtd: MTD instance
 *
 * Return: the SPI NAND device attached to @mtd.
 */
static inline struct spinand_device *mtd_to_spinand(struct mtd_info *mtd)
{
	return container_of(mtd_to_nanddev(mtd), struct spinand_device, base);
}

/**
 * spinand_to_mtd() - Get the MTD device embedded in a SPI NAND device
 * @spinand: SPI NAND device
 *
 * Return: the MTD device embedded in @spinand.
 */
static inline struct mtd_info *spinand_to_mtd(struct spinand_device *spinand)
{
	return nanddev_to_mtd(&spinand->base);
}

/**
 * nand_to_spinand() - Get the SPI NAND device embedding an NAND object
 * @nand: NAND object
 *
 * Return: the SPI NAND device embedding @nand.
 */
static inline struct spinand_device *nand_to_spinand(struct nand_device *nand)
{
	return container_of(nand, struct spinand_device, base);
}

/**
 * spinand_to_nand() - Get the NAND device embedded in a SPI NAND object
 * @spinand: SPI NAND device
 *
 * Return: the NAND device embedded in @spinand.
 */
static inline struct nand_device *
spinand_to_nand(struct spinand_device *spinand)
{
	return &spinand->base;
}

/**
 * spinand_set_of_node - Attach a DT node to a SPI NAND device
 * @spinand: SPI NAND device
 * @np: DT node
 *
 * Attach a DT node to a SPI NAND device.
 */
static inline void spinand_set_of_node(struct spinand_device *spinand,
				       struct device_node *np)
{
	nanddev_set_of_node(&spinand->base, np);
}

#define SPINAND_MAX_ADDR_LEN	4

/**
 * struct spinand_op - SPI NAND operation
 * @cmd: opcode to send
 * @n_addr: number of address bytes
 * @addr_nbits: number of useful bits in the address cycles
 * @dummy_bytes: number of dummy bytes following address or command cycles
 * @addr: address or dummy bytes buffer
 * @n_tx: size of tx_buf
 * @tx_buf: data to be written
 * @n_rx: size of rx_buf
 * @rx_buf: data to be read
 * @data_nbits: number of bit used to transfer data
 */
struct spinand_op {
	u8 cmd;
	u8 n_addr;
	u8 addr_nbits;
	u8 dummy_bytes;
	u8 addr[SPINAND_MAX_ADDR_LEN];
	u32 n_tx;
	const u8 *tx_buf;
	u32 n_rx;
	u8 *rx_buf;
	u8 data_nbits;
};

struct spinand_device *devm_spinand_alloc(struct device *dev);
int spinand_init(struct spinand_device *spinand, struct module *owner);
void spinand_cleanup(struct spinand_device *spinand);
#endif /* __LINUX_MTD_SPINAND_H */
