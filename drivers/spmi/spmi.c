<<<<<<< HEAD
/*
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
>>>>>>> p9x
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
<<<<<<< HEAD
=======

#define pr_fmt(fmt) "%s: " fmt, __func__

>>>>>>> p9x
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/idr.h>
#include <linux/slab.h>
<<<<<<< HEAD
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/spmi.h>
#include <linux/pm_runtime.h>

#include <dt-bindings/spmi/spmi.h>

static DEFINE_IDA(ctrl_ida);

static void spmi_dev_release(struct device *dev)
{
	struct spmi_device *sdev = to_spmi_device(dev);
	kfree(sdev);
}

static const struct device_type spmi_dev_type = {
	.release	= spmi_dev_release,
};

static void spmi_ctrl_release(struct device *dev)
{
	struct spmi_controller *ctrl = to_spmi_controller(dev);
	ida_simple_remove(&ctrl_ida, ctrl->nr);
	kfree(ctrl);
}

static const struct device_type spmi_ctrl_type = {
	.release	= spmi_ctrl_release,
};

static int spmi_device_match(struct device *dev, struct device_driver *drv)
{
	if (of_driver_match_device(dev, drv))
		return 1;

	if (drv->name)
		return strncmp(dev_name(dev), drv->name,
			       SPMI_NAME_SIZE) == 0;
=======
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/spmi.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

#include "spmi-dbgfs.h"

struct spmii_boardinfo {
	struct list_head	list;
	struct spmi_boardinfo	board_info;
};

static DEFINE_MUTEX(board_lock);
static LIST_HEAD(board_list);
static DEFINE_IDR(ctrl_idr);
static DEFINE_IDA(spmi_devid_ida);
static struct device_type spmi_dev_type;
static struct device_type spmi_ctrl_type;

/* Forward declarations */
struct bus_type spmi_bus_type;
static int spmi_register_controller(struct spmi_controller *ctrl);

/**
 * spmi_busnum_to_ctrl: Map bus number to controller
 * @busnum: bus number
 * Returns controller representing this bus number
 */
struct spmi_controller *spmi_busnum_to_ctrl(u32 bus_num)
{
	struct spmi_controller *ctrl;

	mutex_lock(&board_lock);
	ctrl = idr_find(&ctrl_idr, bus_num);
	mutex_unlock(&board_lock);

	return ctrl;
}
EXPORT_SYMBOL_GPL(spmi_busnum_to_ctrl);

/**
 * spmi_add_controller: Controller bring-up.
 * @ctrl: controller to be registered.
 * A controller is registered with the framework using this API. ctrl->nr is the
 * desired number with which SPMI framework registers the controller.
 * Function will return -EBUSY if the number is in use.
 */
int spmi_add_controller(struct spmi_controller *ctrl)
{
	int	id;

	if (!ctrl)
		return -EINVAL;

	pr_debug("adding controller for bus %d (0x%p)\n", ctrl->nr, ctrl);

	mutex_lock(&board_lock);
	id = idr_alloc(&ctrl_idr, ctrl, ctrl->nr, ctrl->nr + 1, GFP_KERNEL);
	mutex_unlock(&board_lock);

	if (id < 0)
		return id;

	ctrl->nr = id;
	return spmi_register_controller(ctrl);
}
EXPORT_SYMBOL_GPL(spmi_add_controller);

/* Remove a device associated with a controller */
static int spmi_ctrl_remove_device(struct device *dev, void *data)
{
	struct spmi_device *spmidev = to_spmi_device(dev);
	struct spmi_controller *ctrl = data;

	if (dev->type == &spmi_dev_type && spmidev->ctrl == ctrl)
		spmi_remove_device(spmidev);
>>>>>>> p9x

	return 0;
}

/**
<<<<<<< HEAD
 * spmi_device_add() - add a device previously constructed via spmi_device_alloc()
 * @sdev:	spmi_device to be added
 */
int spmi_device_add(struct spmi_device *sdev)
{
	struct spmi_controller *ctrl = sdev->ctrl;
	int err;

	dev_set_name(&sdev->dev, "%d-%02x", ctrl->nr, sdev->usid);

	err = device_add(&sdev->dev);
	if (err < 0) {
		dev_err(&sdev->dev, "Can't add %s, status %d\n",
			dev_name(&sdev->dev), err);
		goto err_device_add;
	}

	dev_dbg(&sdev->dev, "device %s registered\n", dev_name(&sdev->dev));

err_device_add:
	return err;
}
EXPORT_SYMBOL_GPL(spmi_device_add);

/**
 * spmi_device_remove(): remove an SPMI device
 * @sdev:	spmi_device to be removed
 */
void spmi_device_remove(struct spmi_device *sdev)
{
	device_unregister(&sdev->dev);
}
EXPORT_SYMBOL_GPL(spmi_device_remove);
=======
 * spmi_del_controller: Controller tear-down.
 * @ctrl: controller to be removed.
 *
 * Controller added with the above API is torn down using this API.
 */
int spmi_del_controller(struct spmi_controller *ctrl)
{
	struct spmi_controller *found;

	if (!ctrl)
		return -EINVAL;

	/* Check that the ctrl has been added */
	mutex_lock(&board_lock);
	found = idr_find(&ctrl_idr, ctrl->nr);
	mutex_unlock(&board_lock);
	if (found != ctrl)
		return -EINVAL;

	/* Remove all the clients associated with this controller */
	mutex_lock(&board_lock);
	bus_for_each_dev(&spmi_bus_type, NULL, ctrl, spmi_ctrl_remove_device);
	mutex_unlock(&board_lock);

	spmi_dfs_del_controller(ctrl);

	mutex_lock(&board_lock);
	idr_remove(&ctrl_idr, ctrl->nr);
	mutex_unlock(&board_lock);

	init_completion(&ctrl->dev_released);
	device_unregister(&ctrl->dev);
	wait_for_completion(&ctrl->dev_released);

	return 0;
}
EXPORT_SYMBOL_GPL(spmi_del_controller);

#define spmi_ctrl_attr_gr NULL
static void spmi_ctrl_release(struct device *dev)
{
	struct spmi_controller *ctrl = to_spmi_controller(dev);

	complete(&ctrl->dev_released);
}

static struct device_type spmi_ctrl_type = {
	.groups		= spmi_ctrl_attr_gr,
	.release	= spmi_ctrl_release,
};

#define spmi_device_attr_gr NULL
#define spmi_device_uevent NULL
static void spmi_dev_release(struct device *dev)
{
	struct spmi_device *spmidev = to_spmi_device(dev);
	kfree(spmidev);
}

static struct device_type spmi_dev_type = {
	.groups		= spmi_device_attr_gr,
	.uevent		= spmi_device_uevent,
	.release	= spmi_dev_release,
};

/**
 * spmi_alloc_device: Allocate a new SPMI devices.
 * @ctrl: controller to which this device is to be added to.
 * Context: can sleep
 *
 * Allows a driver to allocate and initialize a SPMI device without
 * registering it immediately.  This allows a driver to directly fill
 * the spmi_device structure before calling spmi_add_device().
 *
 * Caller is responsible to call spmi_add_device() on the returned
 * spmi_device.  If the caller needs to discard the spmi_device without
 * adding it, then spmi_dev_put() should be called.
 */
struct spmi_device *spmi_alloc_device(struct spmi_controller *ctrl)
{
	struct spmi_device *spmidev;

	if (!ctrl || !spmi_busnum_to_ctrl(ctrl->nr)) {
		pr_err("Missing SPMI controller\n");
		return NULL;
	}

	spmidev = kzalloc(sizeof(*spmidev), GFP_KERNEL);
	if (!spmidev) {
		dev_err(&ctrl->dev, "unable to allocate spmi_device\n");
		return NULL;
	}

	spmidev->ctrl = ctrl;
	spmidev->dev.parent = ctrl->dev.parent;
	spmidev->dev.bus = &spmi_bus_type;
	spmidev->dev.type = &spmi_dev_type;
	device_initialize(&spmidev->dev);

	return spmidev;
}
EXPORT_SYMBOL_GPL(spmi_alloc_device);

/* Validate the SPMI device structure */
static struct device *get_valid_device(struct spmi_device *spmidev)
{
	struct device *dev;

	if (!spmidev)
		return NULL;

	dev = &spmidev->dev;
	if (dev->bus != &spmi_bus_type || dev->type != &spmi_dev_type)
		return NULL;

	if (!spmidev->ctrl || !spmi_busnum_to_ctrl(spmidev->ctrl->nr))
		return NULL;

	return dev;
}

/**
 * spmi_add_device: Add a new device without register board info.
 * @spmi_dev: spmi_device to be added (registered).
 *
 * Called when device doesn't have an explicit client-driver to be probed, or
 * the client-driver is a module installed dynamically.
 */
int spmi_add_device(struct spmi_device *spmidev)
{
	int rc;
	struct device *dev = get_valid_device(spmidev);
	int id;

	if (!dev) {
		pr_err("invalid SPMI device\n");
		return -EINVAL;
	}

	id = ida_simple_get(&spmi_devid_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		pr_err("No id available status = %d\n", id);
		return id;
	}

	/* Set the device name */
	spmidev->id = id;
	dev_set_name(dev, "%s-%d", spmidev->name, spmidev->id);

	/* Device may be bound to an active driver when this returns */
	rc = device_add(dev);

	if (rc < 0) {
		ida_simple_remove(&spmi_devid_ida, spmidev->id);
		dev_err(dev, "Can't add %s, status %d\n", dev_name(dev), rc);
	} else {
		dev_dbg(dev, "device %s registered\n", dev_name(dev));
	}

	return rc;
}
EXPORT_SYMBOL_GPL(spmi_add_device);

/**
 * spmi_new_device: Instantiates a new SPMI device
 * @ctrl: controller to which this device is to be added to.
 * @info: board information for this device.
 *
 * Returns the new device or NULL.
 */
struct spmi_device *spmi_new_device(struct spmi_controller *ctrl,
					struct spmi_boardinfo const *info)
{
	struct spmi_device *spmidev;
	int rc;

	if (!ctrl || !info)
		return NULL;

	spmidev = spmi_alloc_device(ctrl);
	if (!spmidev)
		return NULL;

	spmidev->name = info->name;
	spmidev->sid  = info->slave_id;
	spmidev->dev.of_node = info->of_node;
	spmidev->dev.platform_data = (void *)info->platform_data;
	spmidev->num_dev_node = info->num_dev_node;
	spmidev->dev_node = info->dev_node;
	spmidev->res = info->res;

	rc = spmi_add_device(spmidev);
	if (rc < 0) {
		spmi_dev_put(spmidev);
		return NULL;
	}

	return spmidev;
}
EXPORT_SYMBOL_GPL(spmi_new_device);

/* spmi_remove_device: Remove the effect of spmi_add_device() */
void spmi_remove_device(struct spmi_device *spmi_dev)
{
	device_unregister(&spmi_dev->dev);
	ida_simple_remove(&spmi_devid_ida, spmi_dev->id);
}
EXPORT_SYMBOL_GPL(spmi_remove_device);

static void spmi_match_ctrl_to_boardinfo(struct spmi_controller *ctrl,
				struct spmi_boardinfo *bi)
{
	struct spmi_device *spmidev;

	spmidev = spmi_new_device(ctrl, bi);
	if (!spmidev)
		dev_err(ctrl->dev.parent, "can't create new device for %s\n",
			bi->name);
}

/**
 * spmi_register_board_info: Board-initialization routine.
 * @bus_num: controller number (bus) on which this device will sit.
 * @info: list of all devices on all controllers present on the board.
 * @n: number of entries.
 * API enumerates respective devices on corresponding controller.
 * Called from board-init function.
 * If controller is not present, only add to boards list
 */
int spmi_register_board_info(int busnum,
			struct spmi_boardinfo const *info, unsigned n)
{
	int i;
	struct spmii_boardinfo *bi;
	struct spmi_controller *ctrl;

	bi = kzalloc(n * sizeof(*bi), GFP_KERNEL);
	if (!bi)
		return -ENOMEM;

	ctrl = spmi_busnum_to_ctrl(busnum);

	for (i = 0; i < n; i++, bi++, info++) {

		memcpy(&bi->board_info, info, sizeof(*info));
		mutex_lock(&board_lock);
		list_add_tail(&bi->list, &board_list);

		if (ctrl)
			spmi_match_ctrl_to_boardinfo(ctrl, &bi->board_info);
		mutex_unlock(&board_lock);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(spmi_register_board_info);

/* ------------------------------------------------------------------------- */
>>>>>>> p9x

static inline int
spmi_cmd(struct spmi_controller *ctrl, u8 opcode, u8 sid)
{
	if (!ctrl || !ctrl->cmd || ctrl->dev.type != &spmi_ctrl_type)
		return -EINVAL;

	return ctrl->cmd(ctrl, opcode, sid);
}

<<<<<<< HEAD
static inline int spmi_read_cmd(struct spmi_controller *ctrl, u8 opcode,
				u8 sid, u16 addr, u8 *buf, size_t len)
=======
static inline int spmi_read_cmd(struct spmi_controller *ctrl,
				u8 opcode, u8 sid, u16 addr, u8 bc, u8 *buf)
>>>>>>> p9x
{
	if (!ctrl || !ctrl->read_cmd || ctrl->dev.type != &spmi_ctrl_type)
		return -EINVAL;

<<<<<<< HEAD
	return ctrl->read_cmd(ctrl, opcode, sid, addr, buf, len);
}

static inline int spmi_write_cmd(struct spmi_controller *ctrl, u8 opcode,
				 u8 sid, u16 addr, const u8 *buf, size_t len)
=======
	return ctrl->read_cmd(ctrl, opcode, sid, addr, bc, buf);
}

static inline int spmi_write_cmd(struct spmi_controller *ctrl,
				u8 opcode, u8 sid, u16 addr, u8 bc, u8 *buf)
>>>>>>> p9x
{
	if (!ctrl || !ctrl->write_cmd || ctrl->dev.type != &spmi_ctrl_type)
		return -EINVAL;

<<<<<<< HEAD
	return ctrl->write_cmd(ctrl, opcode, sid, addr, buf, len);
}

/**
 * spmi_register_read() - register read
 * @sdev:	SPMI device.
 * @addr:	slave register address (5-bit address).
 * @buf:	buffer to be populated with data from the Slave.
 *
 * Reads 1 byte of data from a Slave device register.
 */
int spmi_register_read(struct spmi_device *sdev, u8 addr, u8 *buf)
{
	/* 5-bit register address */
	if (addr > 0x1F)
		return -EINVAL;

	return spmi_read_cmd(sdev->ctrl, SPMI_CMD_READ, sdev->usid, addr,
			     buf, 1);
=======
	return ctrl->write_cmd(ctrl, opcode, sid, addr, bc, buf);
}

/*
 * register read/write: 5-bit address, 1 byte of data
 * extended register read/write: 8-bit address, up to 16 bytes of data
 * extended register read/write long: 16-bit address, up to 8 bytes of data
 */

/**
 * spmi_register_read() - register read
 * @dev: SPMI device.
 * @sid: slave identifier.
 * @ad: slave register address (5-bit address).
 * @buf: buffer to be populated with data from the Slave.
 *
 * Reads 1 byte of data from a Slave device register.
 */
int spmi_register_read(struct spmi_controller *ctrl, u8 sid, u8 addr, u8 *buf)
{
	/* 4-bit Slave Identifier, 5-bit register address */
	if (sid > SPMI_MAX_SLAVE_ID || addr > 0x1F)
		return -EINVAL;

	return spmi_read_cmd(ctrl, SPMI_CMD_READ, sid, addr, 0, buf);
>>>>>>> p9x
}
EXPORT_SYMBOL_GPL(spmi_register_read);

/**
 * spmi_ext_register_read() - extended register read
<<<<<<< HEAD
 * @sdev:	SPMI device.
 * @addr:	slave register address (8-bit address).
 * @buf:	buffer to be populated with data from the Slave.
 * @len:	the request number of bytes to read (up to 16 bytes).
=======
 * @dev: SPMI device.
 * @sid: slave identifier.
 * @ad: slave register address (8-bit address).
 * @len: the request number of bytes to read (up to 16 bytes).
 * @buf: buffer to be populated with data from the Slave.
>>>>>>> p9x
 *
 * Reads up to 16 bytes of data from the extended register space on a
 * Slave device.
 */
<<<<<<< HEAD
int spmi_ext_register_read(struct spmi_device *sdev, u8 addr, u8 *buf,
			   size_t len)
{
	/* 8-bit register address, up to 16 bytes */
	if (len == 0 || len > 16)
		return -EINVAL;

	return spmi_read_cmd(sdev->ctrl, SPMI_CMD_EXT_READ, sdev->usid, addr,
			     buf, len);
=======
int spmi_ext_register_read(struct spmi_controller *ctrl,
				u8 sid, u8 addr, u8 *buf, int len)
{
	/* 4-bit Slave Identifier, 8-bit register address, up to 16 bytes */
	if (sid > SPMI_MAX_SLAVE_ID || len <= 0 || len > 16)
		return -EINVAL;

	return spmi_read_cmd(ctrl, SPMI_CMD_EXT_READ, sid, addr, len - 1, buf);
>>>>>>> p9x
}
EXPORT_SYMBOL_GPL(spmi_ext_register_read);

/**
 * spmi_ext_register_readl() - extended register read long
<<<<<<< HEAD
 * @sdev:	SPMI device.
 * @addr:	slave register address (16-bit address).
 * @buf:	buffer to be populated with data from the Slave.
 * @len:	the request number of bytes to read (up to 8 bytes).
=======
 * @dev: SPMI device.
 * @sid: slave identifier.
 * @ad: slave register address (16-bit address).
 * @len: the request number of bytes to read (up to 8 bytes).
 * @buf: buffer to be populated with data from the Slave.
>>>>>>> p9x
 *
 * Reads up to 8 bytes of data from the extended register space on a
 * Slave device using 16-bit address.
 */
<<<<<<< HEAD
int spmi_ext_register_readl(struct spmi_device *sdev, u16 addr, u8 *buf,
			    size_t len)
{
	/* 16-bit register address, up to 8 bytes */
	if (len == 0 || len > 8)
		return -EINVAL;

	return spmi_read_cmd(sdev->ctrl, SPMI_CMD_EXT_READL, sdev->usid, addr,
			     buf, len);
=======
int spmi_ext_register_readl(struct spmi_controller *ctrl,
				u8 sid, u16 addr, u8 *buf, int len)
{
	/* 4-bit Slave Identifier, 16-bit register address, up to 8 bytes */
	if (sid > SPMI_MAX_SLAVE_ID || len <= 0 || len > 8)
		return -EINVAL;

	return spmi_read_cmd(ctrl, SPMI_CMD_EXT_READL, sid, addr, len - 1, buf);
>>>>>>> p9x
}
EXPORT_SYMBOL_GPL(spmi_ext_register_readl);

/**
 * spmi_register_write() - register write
<<<<<<< HEAD
 * @sdev:	SPMI device
 * @addr:	slave register address (5-bit address).
 * @data:	buffer containing the data to be transferred to the Slave.
 *
 * Writes 1 byte of data to a Slave device register.
 */
int spmi_register_write(struct spmi_device *sdev, u8 addr, u8 data)
{
	/* 5-bit register address */
	if (addr > 0x1F)
		return -EINVAL;

	return spmi_write_cmd(sdev->ctrl, SPMI_CMD_WRITE, sdev->usid, addr,
			      &data, 1);
=======
 * @dev: SPMI device.
 * @sid: slave identifier.
 * @ad: slave register address (5-bit address).
 * @buf: buffer containing the data to be transferred to the Slave.
 *
 * Writes 1 byte of data to a Slave device register.
 */
int spmi_register_write(struct spmi_controller *ctrl, u8 sid, u8 addr, u8 *buf)
{
	u8 op = SPMI_CMD_WRITE;

	/* 4-bit Slave Identifier, 5-bit register address */
	if (sid > SPMI_MAX_SLAVE_ID || addr > 0x1F)
		return -EINVAL;

	return spmi_write_cmd(ctrl, op, sid, addr, 0, buf);
>>>>>>> p9x
}
EXPORT_SYMBOL_GPL(spmi_register_write);

/**
 * spmi_register_zero_write() - register zero write
<<<<<<< HEAD
 * @sdev:	SPMI device.
 * @data:	the data to be written to register 0 (7-bits).
 *
 * Writes data to register 0 of the Slave device.
 */
int spmi_register_zero_write(struct spmi_device *sdev, u8 data)
{
	return spmi_write_cmd(sdev->ctrl, SPMI_CMD_ZERO_WRITE, sdev->usid, 0,
			      &data, 1);
=======
 * @dev: SPMI device.
 * @sid: slave identifier.
 * @data: the data to be written to register 0 (7-bits).
 *
 * Writes data to register 0 of the Slave device.
 */
int spmi_register_zero_write(struct spmi_controller *ctrl, u8 sid, u8 data)
{
	u8 op = SPMI_CMD_ZERO_WRITE;

	/* 4-bit Slave Identifier, 5-bit register address */
	if (sid > SPMI_MAX_SLAVE_ID)
		return -EINVAL;

	return spmi_write_cmd(ctrl, op, sid, 0, 0, &data);
>>>>>>> p9x
}
EXPORT_SYMBOL_GPL(spmi_register_zero_write);

/**
 * spmi_ext_register_write() - extended register write
<<<<<<< HEAD
 * @sdev:	SPMI device.
 * @addr:	slave register address (8-bit address).
 * @buf:	buffer containing the data to be transferred to the Slave.
 * @len:	the request number of bytes to read (up to 16 bytes).
=======
 * @dev: SPMI device.
 * @sid: slave identifier.
 * @ad: slave register address (8-bit address).
 * @buf: buffer containing the data to be transferred to the Slave.
 * @len: the request number of bytes to read (up to 16 bytes).
>>>>>>> p9x
 *
 * Writes up to 16 bytes of data to the extended register space of a
 * Slave device.
 */
<<<<<<< HEAD
int spmi_ext_register_write(struct spmi_device *sdev, u8 addr, const u8 *buf,
			    size_t len)
{
	/* 8-bit register address, up to 16 bytes */
	if (len == 0 || len > 16)
		return -EINVAL;

	return spmi_write_cmd(sdev->ctrl, SPMI_CMD_EXT_WRITE, sdev->usid, addr,
			      buf, len);
=======
int spmi_ext_register_write(struct spmi_controller *ctrl,
				u8 sid, u8 addr, u8 *buf, int len)
{
	u8 op = SPMI_CMD_EXT_WRITE;

	/* 4-bit Slave Identifier, 8-bit register address, up to 16 bytes */
	if (sid > SPMI_MAX_SLAVE_ID || len <= 0 || len > 16)
		return -EINVAL;

	return spmi_write_cmd(ctrl, op, sid, addr, len - 1, buf);
>>>>>>> p9x
}
EXPORT_SYMBOL_GPL(spmi_ext_register_write);

/**
 * spmi_ext_register_writel() - extended register write long
<<<<<<< HEAD
 * @sdev:	SPMI device.
 * @addr:	slave register address (16-bit address).
 * @buf:	buffer containing the data to be transferred to the Slave.
 * @len:	the request number of bytes to read (up to 8 bytes).
=======
 * @dev: SPMI device.
 * @sid: slave identifier.
 * @ad: slave register address (16-bit address).
 * @buf: buffer containing the data to be transferred to the Slave.
 * @len: the request number of bytes to read (up to 8 bytes).
>>>>>>> p9x
 *
 * Writes up to 8 bytes of data to the extended register space of a
 * Slave device using 16-bit address.
 */
<<<<<<< HEAD
int spmi_ext_register_writel(struct spmi_device *sdev, u16 addr, const u8 *buf,
			     size_t len)
{
	/* 4-bit Slave Identifier, 16-bit register address, up to 8 bytes */
	if (len == 0 || len > 8)
		return -EINVAL;

	return spmi_write_cmd(sdev->ctrl, SPMI_CMD_EXT_WRITEL, sdev->usid,
			      addr, buf, len);
=======
int spmi_ext_register_writel(struct spmi_controller *ctrl,
				u8 sid, u16 addr, u8 *buf, int len)
{
	u8 op = SPMI_CMD_EXT_WRITEL;

	/* 4-bit Slave Identifier, 16-bit register address, up to 8 bytes */
	if (sid > SPMI_MAX_SLAVE_ID || len <= 0 || len > 8)
		return -EINVAL;

	return spmi_write_cmd(ctrl, op, sid, addr, len - 1, buf);
>>>>>>> p9x
}
EXPORT_SYMBOL_GPL(spmi_ext_register_writel);

/**
 * spmi_command_reset() - sends RESET command to the specified slave
<<<<<<< HEAD
 * @sdev:	SPMI device.
=======
 * @dev: SPMI device.
 * @sid: slave identifier.
>>>>>>> p9x
 *
 * The Reset command initializes the Slave and forces all registers to
 * their reset values. The Slave shall enter the STARTUP state after
 * receiving a Reset command.
<<<<<<< HEAD
 */
int spmi_command_reset(struct spmi_device *sdev)
{
	return spmi_cmd(sdev->ctrl, SPMI_CMD_RESET, sdev->usid);
=======
 *
 * Returns
 * -EINVAL for invalid Slave Identifier.
 * -EPERM if the SPMI transaction is denied due to permission issues.
 * -EIO if the SPMI transaction fails (parity errors, etc).
 * -ETIMEDOUT if the SPMI transaction times out.
 */
int spmi_command_reset(struct spmi_controller *ctrl, u8 sid)
{
	if (sid > SPMI_MAX_SLAVE_ID)
		return -EINVAL;
	return spmi_cmd(ctrl, SPMI_CMD_RESET, sid);
>>>>>>> p9x
}
EXPORT_SYMBOL_GPL(spmi_command_reset);

/**
<<<<<<< HEAD
 * spmi_command_sleep() - sends SLEEP command to the specified SPMI device
 * @sdev:	SPMI device.
 *
 * The Sleep command causes the Slave to enter the user defined SLEEP state.
 */
int spmi_command_sleep(struct spmi_device *sdev)
{
	return spmi_cmd(sdev->ctrl, SPMI_CMD_SLEEP, sdev->usid);
=======
 * spmi_command_sleep() - sends SLEEP command to the specified slave
 * @dev: SPMI device.
 * @sid: slave identifier.
 *
 * The Sleep command causes the Slave to enter the user defined SLEEP state.
 *
 * Returns
 * -EINVAL for invalid Slave Identifier.
 * -EPERM if the SPMI transaction is denied due to permission issues.
 * -EIO if the SPMI transaction fails (parity errors, etc).
 * -ETIMEDOUT if the SPMI transaction times out.
 */
int spmi_command_sleep(struct spmi_controller *ctrl, u8 sid)
{
	if (sid > SPMI_MAX_SLAVE_ID)
		return -EINVAL;
	return spmi_cmd(ctrl, SPMI_CMD_SLEEP, sid);
>>>>>>> p9x
}
EXPORT_SYMBOL_GPL(spmi_command_sleep);

/**
<<<<<<< HEAD
 * spmi_command_wakeup() - sends WAKEUP command to the specified SPMI device
 * @sdev:	SPMI device.
 *
 * The Wakeup command causes the Slave to move from the SLEEP state to
 * the ACTIVE state.
 */
int spmi_command_wakeup(struct spmi_device *sdev)
{
	return spmi_cmd(sdev->ctrl, SPMI_CMD_WAKEUP, sdev->usid);
=======
 * spmi_command_wakeup() - sends WAKEUP command to the specified slave
 * @dev: SPMI device.
 * @sid: slave identifier.
 *
 * The Wakeup command causes the Slave to move from the SLEEP state to
 * the ACTIVE state.
 *
 * Returns
 * -EINVAL for invalid Slave Identifier.
 * -EPERM if the SPMI transaction is denied due to permission issues.
 * -EIO if the SPMI transaction fails (parity errors, etc).
 * -ETIMEDOUT if the SPMI transaction times out.
 */
int spmi_command_wakeup(struct spmi_controller *ctrl, u8 sid)
{
	if (sid > SPMI_MAX_SLAVE_ID)
		return -EINVAL;
	return spmi_cmd(ctrl, SPMI_CMD_WAKEUP, sid);
>>>>>>> p9x
}
EXPORT_SYMBOL_GPL(spmi_command_wakeup);

/**
<<<<<<< HEAD
 * spmi_command_shutdown() - sends SHUTDOWN command to the specified SPMI device
 * @sdev:	SPMI device.
 *
 * The Shutdown command causes the Slave to enter the SHUTDOWN state.
 */
int spmi_command_shutdown(struct spmi_device *sdev)
{
	return spmi_cmd(sdev->ctrl, SPMI_CMD_SHUTDOWN, sdev->usid);
}
EXPORT_SYMBOL_GPL(spmi_command_shutdown);

static int spmi_drv_probe(struct device *dev)
{
	const struct spmi_driver *sdrv = to_spmi_driver(dev->driver);
	struct spmi_device *sdev = to_spmi_device(dev);
	int err;

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	err = sdrv->probe(sdev);
	if (err)
		goto fail_probe;

	return 0;

fail_probe:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);
	return err;
=======
 * spmi_command_shutdown() - sends SHUTDOWN command to the specified slave
 * @dev: SPMI device.
 * @sid: slave identifier.
 *
 * The Shutdown command causes the Slave to enter the SHUTDOWN state.
 *
 * Returns
 * -EINVAL for invalid Slave Identifier.
 * -EPERM if the SPMI transaction is denied due to permission issues.
 * -EIO if the SPMI transaction fails (parity errors, etc).
 * -ETIMEDOUT if the SPMI transaction times out.
 */
int spmi_command_shutdown(struct spmi_controller *ctrl, u8 sid)
{
	if (sid > SPMI_MAX_SLAVE_ID)
		return -EINVAL;
	return spmi_cmd(ctrl, SPMI_CMD_SHUTDOWN, sid);
}
EXPORT_SYMBOL_GPL(spmi_command_shutdown);

/* ------------------------------------------------------------------------- */

static const struct spmi_device_id *spmi_match(const struct spmi_device_id *id,
		const struct spmi_device *spmi_dev)
{
	while (id->name[0]) {
		if (strncmp(spmi_dev->name, id->name, SPMI_NAME_SIZE) == 0)
			return id;
		id++;
	}
	return NULL;
}

static int spmi_device_match(struct device *dev, struct device_driver *drv)
{
	struct spmi_device *spmi_dev;
	struct spmi_driver *sdrv = to_spmi_driver(drv);

	if (dev->type == &spmi_dev_type)
		spmi_dev = to_spmi_device(dev);
	else
		return 0;

	/* Attempt an OF style match */
	if (of_driver_match_device(dev, drv))
		return 1;

	if (sdrv->id_table)
		return spmi_match(sdrv->id_table, spmi_dev) != NULL;

	if (drv->name)
		return strncmp(spmi_dev->name, drv->name, SPMI_NAME_SIZE) == 0;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int spmi_legacy_suspend(struct device *dev, pm_message_t mesg)
{
	struct spmi_device *spmi_dev = NULL;
	struct spmi_driver *driver;
	if (dev->type == &spmi_dev_type)
		spmi_dev = to_spmi_device(dev);

	if (!spmi_dev || !dev->driver)
		return 0;

	driver = to_spmi_driver(dev->driver);
	if (!driver->suspend)
		return 0;

	return driver->suspend(spmi_dev, mesg);
}

static int spmi_legacy_resume(struct device *dev)
{
	struct spmi_device *spmi_dev = NULL;
	struct spmi_driver *driver;
	if (dev->type == &spmi_dev_type)
		spmi_dev = to_spmi_device(dev);

	if (!spmi_dev || !dev->driver)
		return 0;

	driver = to_spmi_driver(dev->driver);
	if (!driver->resume)
		return 0;

	return driver->resume(spmi_dev);
}

static int spmi_pm_suspend(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_suspend(dev);
	else
		return spmi_legacy_suspend(dev, PMSG_SUSPEND);
}

static int spmi_pm_resume(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_resume(dev);
	else
		return spmi_legacy_resume(dev);
}

#else
#define spmi_pm_suspend		NULL
#define spmi_pm_resume		NULL
#endif

static const struct dev_pm_ops spmi_pm_ops = {
	.suspend = spmi_pm_suspend,
	.resume = spmi_pm_resume,
	SET_RUNTIME_PM_OPS(
		pm_generic_suspend,
		pm_generic_resume,
		pm_generic_runtime_idle
		)
};
struct bus_type spmi_bus_type = {
	.name		= "spmi",
	.match		= spmi_device_match,
	.pm		= &spmi_pm_ops,
};
EXPORT_SYMBOL_GPL(spmi_bus_type);

struct device spmi_dev = {
	.init_name = "spmi",
};

static int spmi_drv_probe(struct device *dev)
{
	const struct spmi_driver *sdrv = to_spmi_driver(dev->driver);

	return sdrv->probe(to_spmi_device(dev));
>>>>>>> p9x
}

static int spmi_drv_remove(struct device *dev)
{
	const struct spmi_driver *sdrv = to_spmi_driver(dev->driver);

<<<<<<< HEAD
	pm_runtime_get_sync(dev);
	sdrv->remove(to_spmi_device(dev));
	pm_runtime_put_noidle(dev);

	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);
	return 0;
}

static int spmi_drv_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	int ret;

	ret = of_device_uevent_modalias(dev, env);
	if (ret != -ENODEV)
		return ret;

	return 0;
}

static struct bus_type spmi_bus_type = {
	.name		= "spmi",
	.match		= spmi_device_match,
	.probe		= spmi_drv_probe,
	.remove		= spmi_drv_remove,
	.uevent		= spmi_drv_uevent,
};

/**
 * spmi_controller_alloc() - Allocate a new SPMI device
 * @ctrl:	associated controller
 *
 * Caller is responsible for either calling spmi_device_add() to add the
 * newly allocated controller, or calling spmi_device_put() to discard it.
 */
struct spmi_device *spmi_device_alloc(struct spmi_controller *ctrl)
{
	struct spmi_device *sdev;

	sdev = kzalloc(sizeof(*sdev), GFP_KERNEL);
	if (!sdev)
		return NULL;

	sdev->ctrl = ctrl;
	device_initialize(&sdev->dev);
	sdev->dev.parent = &ctrl->dev;
	sdev->dev.bus = &spmi_bus_type;
	sdev->dev.type = &spmi_dev_type;
	return sdev;
}
EXPORT_SYMBOL_GPL(spmi_device_alloc);

/**
 * spmi_controller_alloc() - Allocate a new SPMI controller
 * @parent:	parent device
 * @size:	size of private data
 *
 * Caller is responsible for either calling spmi_controller_add() to add the
 * newly allocated controller, or calling spmi_controller_put() to discard it.
 * The allocated private data region may be accessed via
 * spmi_controller_get_drvdata()
 */
struct spmi_controller *spmi_controller_alloc(struct device *parent,
					      size_t size)
{
	struct spmi_controller *ctrl;
	int id;

	if (WARN_ON(!parent))
		return NULL;

	ctrl = kzalloc(sizeof(*ctrl) + size, GFP_KERNEL);
	if (!ctrl)
		return NULL;

	device_initialize(&ctrl->dev);
	ctrl->dev.type = &spmi_ctrl_type;
	ctrl->dev.bus = &spmi_bus_type;
	ctrl->dev.parent = parent;
	ctrl->dev.of_node = parent->of_node;
	spmi_controller_set_drvdata(ctrl, &ctrl[1]);

	id = ida_simple_get(&ctrl_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		dev_err(parent,
			"unable to allocate SPMI controller identifier.\n");
		spmi_controller_put(ctrl);
		return NULL;
	}

	ctrl->nr = id;
	dev_set_name(&ctrl->dev, "spmi-%d", id);

	dev_dbg(&ctrl->dev, "allocated controller 0x%p id %d\n", ctrl, id);
	return ctrl;
}
EXPORT_SYMBOL_GPL(spmi_controller_alloc);

static void of_spmi_register_devices(struct spmi_controller *ctrl)
{
	struct device_node *node;
	int err;

	if (!ctrl->dev.of_node)
		return;

	for_each_available_child_of_node(ctrl->dev.of_node, node) {
		struct spmi_device *sdev;
		u32 reg[2];

		dev_dbg(&ctrl->dev, "adding child %s\n", node->full_name);

		err = of_property_read_u32_array(node, "reg", reg, 2);
		if (err) {
			dev_err(&ctrl->dev,
				"node %s err (%d) does not have 'reg' property\n",
				node->full_name, err);
			continue;
		}

		if (reg[1] != SPMI_USID) {
			dev_err(&ctrl->dev,
				"node %s contains unsupported 'reg' entry\n",
				node->full_name);
			continue;
		}

		if (reg[0] >= SPMI_MAX_SLAVE_ID) {
			dev_err(&ctrl->dev,
				"invalid usid on node %s\n",
				node->full_name);
			continue;
		}

		dev_dbg(&ctrl->dev, "read usid %02x\n", reg[0]);

		sdev = spmi_device_alloc(ctrl);
		if (!sdev)
			continue;

		sdev->dev.of_node = node;
		sdev->usid = (u8) reg[0];

		err = spmi_device_add(sdev);
		if (err) {
			dev_err(&sdev->dev,
				"failure adding device. status %d\n", err);
			spmi_device_put(sdev);
		}
	}
}

/**
 * spmi_controller_add() - Add an SPMI controller
 * @ctrl:	controller to be registered.
 *
 * Register a controller previously allocated via spmi_controller_alloc() with
 * the SPMI core.
 */
int spmi_controller_add(struct spmi_controller *ctrl)
{
	int ret;

	/* Can't register until after driver model init */
	if (WARN_ON(!spmi_bus_type.p))
		return -EAGAIN;

	ret = device_add(&ctrl->dev);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_OF))
		of_spmi_register_devices(ctrl);

	dev_dbg(&ctrl->dev, "spmi-%d registered: dev:%p\n",
		ctrl->nr, &ctrl->dev);

	return 0;
};
EXPORT_SYMBOL_GPL(spmi_controller_add);

/* Remove a device associated with a controller */
static int spmi_ctrl_remove_device(struct device *dev, void *data)
{
	struct spmi_device *spmidev = to_spmi_device(dev);
	if (dev->type == &spmi_dev_type)
		spmi_device_remove(spmidev);
	return 0;
}

/**
 * spmi_controller_remove(): remove an SPMI controller
 * @ctrl:	controller to remove
 *
 * Remove a SPMI controller.  Caller is responsible for calling
 * spmi_controller_put() to discard the allocated controller.
 */
void spmi_controller_remove(struct spmi_controller *ctrl)
{
	int dummy;

	if (!ctrl)
		return;

	dummy = device_for_each_child(&ctrl->dev, NULL,
				      spmi_ctrl_remove_device);
	device_del(&ctrl->dev);
}
EXPORT_SYMBOL_GPL(spmi_controller_remove);

/**
 * spmi_driver_register() - Register client driver with SPMI core
 * @sdrv:	client driver to be associated with client-device.
 *
 * This API will register the client driver with the SPMI framework.
 * It is typically called from the driver's module-init function.
 */
int spmi_driver_register(struct spmi_driver *sdrv)
{
	sdrv->driver.bus = &spmi_bus_type;
	return driver_register(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(spmi_driver_register);

static void __exit spmi_exit(void)
{
	bus_unregister(&spmi_bus_type);
}
module_exit(spmi_exit);

static int __init spmi_init(void)
{
	return bus_register(&spmi_bus_type);
}
postcore_initcall(spmi_init);

MODULE_LICENSE("GPL v2");
=======
	return sdrv->remove(to_spmi_device(dev));
}

static void spmi_drv_shutdown(struct device *dev)
{
	const struct spmi_driver *sdrv = to_spmi_driver(dev->driver);

	sdrv->shutdown(to_spmi_device(dev));
}

/**
 * spmi_driver_register: Client driver registration with SPMI framework.
 * @drv: client driver to be associated with client-device.
 *
 * This API will register the client driver with the SPMI framework.
 * It is called from the driver's module-init function.
 */
int spmi_driver_register(struct spmi_driver *drv)
{
	drv->driver.bus = &spmi_bus_type;

	if (drv->probe)
		drv->driver.probe = spmi_drv_probe;

	if (drv->remove)
		drv->driver.remove = spmi_drv_remove;

	if (drv->shutdown)
		drv->driver.shutdown = spmi_drv_shutdown;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(spmi_driver_register);

static int spmi_register_controller(struct spmi_controller *ctrl)
{
	int ret = 0;

	/* Can't register until after driver model init */
	if (WARN_ON(!spmi_bus_type.p)) {
		ret = -EAGAIN;
		goto exit;
	}

	dev_set_name(&ctrl->dev, "spmi-%d", ctrl->nr);
	ctrl->dev.bus = &spmi_bus_type;
	ctrl->dev.type = &spmi_ctrl_type;
	ret = device_register(&ctrl->dev);
	if (ret)
		goto exit;

	dev_dbg(&ctrl->dev, "Bus spmi-%d registered: dev:0x%p\n",
					ctrl->nr, &ctrl->dev);

	spmi_dfs_add_controller(ctrl);
	return 0;

exit:
	mutex_lock(&board_lock);
	idr_remove(&ctrl_idr, ctrl->nr);
	mutex_unlock(&board_lock);
	return ret;
}

static void __exit spmi_exit(void)
{
	device_unregister(&spmi_dev);
	bus_unregister(&spmi_bus_type);
}

static int __init spmi_init(void)
{
	int retval;

	retval = bus_register(&spmi_bus_type);
	if (!retval)
		retval = device_register(&spmi_dev);

	if (retval)
		bus_unregister(&spmi_bus_type);

	return retval;
}
postcore_initcall(spmi_init);
module_exit(spmi_exit);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
>>>>>>> p9x
MODULE_DESCRIPTION("SPMI module");
MODULE_ALIAS("platform:spmi");
