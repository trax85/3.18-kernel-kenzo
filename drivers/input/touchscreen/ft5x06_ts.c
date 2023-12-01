/*
 *
 * FocalTech ft5x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
<<<<<<< HEAD
 * Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
=======
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 * Copyright (C) 2016 XiaoMi, Inc.
>>>>>>> p9x
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
<<<<<<< HEAD
=======
#include <linux/proc_fs.h>
>>>>>>> p9x
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
<<<<<<< HEAD
#include <linux/input/ft5x06_ts.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

=======
#include <linux/mutex.h>
#include <linux/input/ft5x06_ts.h>
#include <linux/uaccess.h>
#include "lct_tp_fm_info.h"
#include "lct_ctp_upgrade.h"
#if defined(CONFIG_TOUCHSCREEN_GESTURE)
#include "lct_ctp_gesture.h"
#endif
#include "lct_ctp_selftest.h"
>>>>>>> p9x

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

<<<<<<< HEAD
#if defined(CONFIG_FT_SECURE_TOUCH)
#include <linux/completion.h>
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
static irqreturn_t ft5x06_ts_interrupt(int irq, void *data);
=======
#if defined(CONFIG_TOUCHSCREEN_GESTURE)
#endif

#define FT_PROC_DEBUG
#if defined(FT_PROC_DEBUG)
#define FTS_FACTORYMODE_VALUE		0x40
#define FTS_WORKMODE_VALUE		0x00
#endif
#define TP_CHIPER_ID			0x54

#ifdef FT_GESTURE
#define GESTURE_LEFT			0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    	0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    	0x30
#define GESTURE_W		    	0x31
#define GESTURE_M		    	0x32
#define GESTURE_E		    	0x33
#define GESTURE_C		    	0x34
#define GESTURE_S		    	0x46
#define GESTURE_V		    	0x54
#define GESTURE_Z		    	0x65

#define FTS_GESTRUE_POINTS 				255
#define FTS_GESTRUE_POINTS_ONETIME  	62
#define FTS_GESTRUE_POINTS_HEADER 		8
#define FTS_GESTURE_OUTPUT_ADRESS 		0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 	4

unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};
#endif

#if defined(CONFIG_TOUCHSCREEN_GESTURE)
extern int ctp_get_gesture_data(void);
extern void ctp_set_gesture_data(int value);
#endif

#define CTP_PROC_LOCKDOWN_FILE "tp_lockdown_info"
char tp_lockdown_info[128];
u8 uc_tp_vendor_id;

static struct i2c_client *fts_proc_entry_i2c_client;

static unsigned char firmware_data[] = {
#include "FT5346_LQ_CX865_Biel0x3b_V20_D01_20151023_app.i"
};
static unsigned char firmware_data_biel[] = {
#include "FT5346_LQ_CX865_Biel0x3b_V20_D01_20151023_app.i"
};
static unsigned char firmware_data_mutton[] = {
#include "FT5346_LQ_CX865_Mutton0x53_Sharp_Black_V11_D01_20151022_app.i"
};
static unsigned char firmware_data_ofilm[] = {
#include "FT5346_LQ_L8650_Ofilm0x51_V02_D01_20151228_app.i"
};


#ifdef FTS_SCAP_TEST
#include "ft5x06_mcap_test_lib.h"
#endif

#ifdef CONFIG_WAKE_GESTURES
#include <linux/wake_gestures.h>
>>>>>>> p9x
#endif

#define FT_DRIVER_VERSION	0x02

#define FT_META_REGS		3
#define FT_ONE_TCH_LEN		6
#define FT_TCH_LEN(x)		(FT_META_REGS + FT_ONE_TCH_LEN * x)

#define FT_PRESS		0x7F
#define FT_MAX_ID		0x0F
#define FT_TOUCH_X_H_POS	3
#define FT_TOUCH_X_L_POS	4
#define FT_TOUCH_Y_H_POS	5
#define FT_TOUCH_Y_L_POS	6
#define FT_TD_STATUS		2
#define FT_TOUCH_EVENT_POS	3
#define FT_TOUCH_ID_POS		5
#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2

<<<<<<< HEAD
/* register address*/
=======
/*register address*/
>>>>>>> p9x
#define FT_REG_DEV_MODE		0x00
#define FT_DEV_MODE_REG_CAL	0x02
#define FT_REG_ID		0xA3
#define FT_REG_PMODE		0xA5
#define FT_REG_FW_VER		0xA6
#define FT_REG_FW_VENDOR_ID	0xA8
#define FT_REG_POINT_RATE	0x88
#define FT_REG_THGROUP		0x80
#define FT_REG_ECC		0xCC
#define FT_REG_RESET_FW		0x07
#define FT_REG_FW_MIN_VER	0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3

<<<<<<< HEAD
/* gesture register address*/
#define FT_REG_GESTURE_ENABLE	0xD0
#define FT_REG_GESTURE_OUTPUT	0xD3

/* gesture register bits*/
#define FT_GESTURE_DOUBLECLICK_COORD_X		100
#define FT_GESTURE_DOUBLECLICK_COORD_Y		100
#define FT_GESTURE_WAKEUP_TIMEOUT		500
#define FT_GESTURE_DEFAULT_TRACKING_ID		0x0A
#define FT_GESTURE_DOUBLECLICK_ID		0x24
#define FT_GESTURE_POINTER_NUM_MAX		128
#define FT_GESTURE_POINTER_SIZEOF		4
#define FT_GESTURE_ID_FLAG_SIZE			1
#define FT_GESTURE_POINTER_NUM_FLAG_SIZE	1
/* 6 bytes are taken to mark which gesture is supported in firmware */
#define FT_GESTURE_SET_FLAG_SIZE		6
#define I2C_TRANSFER_MAX_BYTE			255
#define FT_GESTURE_DATA_HEADER	(FT_GESTURE_ID_FLAG_SIZE + \
				FT_GESTURE_POINTER_NUM_FLAG_SIZE + \
				FT_GESTURE_SET_FLAG_SIZE)

=======
>>>>>>> p9x
/* power register bits*/
#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_MONITOR	0x01
#define FT_PMODE_STANDBY	0x02
#define FT_PMODE_HIBERNATE	0x03
#define FT_FACTORYMODE_VALUE	0x40
#define FT_WORKMODE_VALUE	0x00
#define FT_RST_CMD_REG1		0xFC
#define FT_RST_CMD_REG2		0xBC
#define FT_READ_ID_REG		0x90
#define FT_ERASE_APP_REG	0x61
#define FT_ERASE_PANEL_REG	0x63
#define FT_FW_START_REG		0xBF

#define FT_STATUS_NUM_TP_MASK	0x0F

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FT_8BIT_SHIFT		8
#define FT_4BIT_SHIFT		4
#define FT_FW_NAME_MAX_LEN	50
<<<<<<< HEAD
=======
#define FT_LOCKDOWN_LEN	128

>>>>>>> p9x

#define FT5316_ID		0x0A
#define FT5306I_ID		0x55
#define FT6X06_ID		0x06
#define FT6X36_ID		0x36
<<<<<<< HEAD
=======
#define FT5X46_ID		0x54
>>>>>>> p9x

#define FT_UPGRADE_AA		0xAA
#define FT_UPGRADE_55		0x55

#define FT_FW_MIN_SIZE		8
<<<<<<< HEAD
#define FT_FW_MAX_SIZE		32768
=======
#define FT_FW_MAX_SIZE		65536
>>>>>>> p9x

/* Firmware file is not supporting minor and sub minor so use 0 */
#define FT_FW_FILE_MAJ_VER(x)	((x)->data[(x)->size - 2])
#define FT_FW_FILE_MIN_VER(x)	0
#define FT_FW_FILE_SUB_MIN_VER(x) 0
#define FT_FW_FILE_VENDOR_ID(x)	((x)->data[(x)->size - 1])

#define FT_FW_FILE_MAJ_VER_FT6X36(x)	((x)->data[0x10a])
#define FT_FW_FILE_VENDOR_ID_FT6X36(x)	((x)->data[0x108])

/**
* Application data verification will be run before upgrade flow.
* Firmware image stores some flags with negative and positive value
* in corresponding addresses, we need pick them out do some check to
* make sure the application data is valid.
*/
#define FT_FW_CHECK(x, ts_data) \
	(ts_data->family_id == FT6X36_ID ? \
	(((x)->data[0x104] ^ (x)->data[0x105]) == 0xFF \
	&& ((x)->data[0x106] ^ (x)->data[0x107]) == 0xFF) : \
	(((x)->data[(x)->size - 8] ^ (x)->data[(x)->size - 6]) == 0xFF \
	&& ((x)->data[(x)->size - 7] ^ (x)->data[(x)->size - 5]) == 0xFF \
	&& ((x)->data[(x)->size - 3] ^ (x)->data[(x)->size - 4]) == 0xFF))

#define FT_MAX_TRIES		5
#define FT_RETRY_DLY		20

#define FT_MAX_WR_BUF		10
#define FT_MAX_RD_BUF		2
#define FT_FW_PKT_LEN		128
#define FT_FW_PKT_META_LEN	6
#define FT_FW_PKT_DLY_MS	20
#define FT_FW_LAST_PKT		0x6ffa
#define FT_EARSE_DLY_MS		100
#define FT_55_AA_DLY_NS		5000

<<<<<<< HEAD
#define FT_UPGRADE_LOOP		30
=======
#define FT_UPGRADE_LOOP		5
>>>>>>> p9x
#define FT_CAL_START		0x04
#define FT_CAL_FIN		0x00
#define FT_CAL_STORE		0x05
#define FT_CAL_RETRY		100
#define FT_REG_CAL		0x00
#define FT_CAL_MASK		0x70

#define FT_INFO_MAX_LEN		512

#define FT_BLOADER_SIZE_OFF	12
#define FT_BLOADER_NEW_SIZE	30
#define FT_DATA_LEN_OFF_OLD_FW	8
#define FT_DATA_LEN_OFF_NEW_FW	14
#define FT_FINISHING_PKT_LEN_OLD_FW	6
#define FT_FINISHING_PKT_LEN_NEW_FW	12
#define FT_MAGIC_BLOADER_Z7	0x7bfa
#define FT_MAGIC_BLOADER_LZ4	0x6ffa
#define FT_MAGIC_BLOADER_GZF_30	0x7ff4
#define FT_MAGIC_BLOADER_GZF	0x7bf4

#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"

<<<<<<< HEAD
static irqreturn_t ft5x06_ts_interrupt(int irq, void *data);

=======
>>>>>>> p9x
enum {
	FT_BLOADER_VERSION_LZ4 = 0,
	FT_BLOADER_VERSION_Z7 = 1,
	FT_BLOADER_VERSION_GZF = 2,
};

enum {
	FT_FT5336_FAMILY_ID_0x11 = 0x11,
	FT_FT5336_FAMILY_ID_0x12 = 0x12,
	FT_FT5336_FAMILY_ID_0x13 = 0x13,
	FT_FT5336_FAMILY_ID_0x14 = 0x14,
};

<<<<<<< HEAD
#define FT_STORE_TS_INFO(buf, id, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FT_INFO_MAX_LEN, \
				"vendor name = Focaltech\n" \
				"model = 0x%x\n" \
				"fw_version = %d.%d.%d\n", \
				id, fw_maj, fw_min, fw_sub_min)
#define FT_TS_INFO_SYSFS_DIR_NAME "ts_info"
static char *ts_info_buff;

#define FT_STORE_TS_DBG_INFO(buf, id, name, max_tch, group_id, \
			fw_vkey_support, fw_name, fw_maj, fw_min, fw_sub_min) \
=======
#define FT_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
>>>>>>> p9x
			snprintf(buf, FT_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= 0x%x\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FT_DRIVER_VERSION, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)

#define FT_DEBUG_DIR_NAME	"ts_debug"

struct ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct ft5x06_ts_platform_data *pdata;
<<<<<<< HEAD
	struct ft5x06_gesture_platform_data *gesture_pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct mutex ft_clk_io_ctrl_mutex;
	char fw_name[FT_FW_NAME_MAX_LEN];
=======
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	char tp_lockdown_info_temp[FT_LOCKDOWN_LEN];
>>>>>>> p9x
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
<<<<<<< HEAD
=======
	bool keypad_mode;
>>>>>>> p9x
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 fw_vendor_id;
<<<<<<< HEAD
	struct kobject *ts_info_kobj;
#if defined(CONFIG_FB)
	struct work_struct fb_notify_work;
=======
#if defined(CONFIG_FB)
>>>>>>> p9x
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
<<<<<<< HEAD
#if defined(CONFIG_FT_SECURE_TOUCH)
	atomic_t st_enabled;
	atomic_t st_pending_irqs;
	struct completion st_powerdown;
	struct completion st_irq_processed;
	bool st_initialized;
	struct clk *core_clk;
	struct clk *iface_clk;
#endif
};

static int ft5x06_ts_start(struct device *dev);
static int ft5x06_ts_stop(struct device *dev);

#if defined(CONFIG_FT_SECURE_TOUCH)
static void ft5x06_secure_touch_init(struct ft5x06_ts_data *data)
{
	data->st_initialized = 0;

	init_completion(&data->st_powerdown);
	init_completion(&data->st_irq_processed);

	/* Get clocks */
	data->core_clk = devm_clk_get(&data->client->dev, "core_clk");
	if (IS_ERR(data->core_clk)) {
		data->core_clk = NULL;
		dev_warn(&data->client->dev,
			"%s: core_clk is not defined\n", __func__);
	}

	data->iface_clk = devm_clk_get(&data->client->dev, "iface_clk");
	if (IS_ERR(data->iface_clk)) {
		data->iface_clk = NULL;
		dev_warn(&data->client->dev,
			"%s: iface_clk is not defined", __func__);
	}
	data->st_initialized = 1;
}

static void ft5x06_secure_touch_notify(struct ft5x06_ts_data *data)
{
	sysfs_notify(&data->input_dev->dev.kobj, NULL, "secure_touch");
}

static irqreturn_t ft5x06_filter_interrupt(struct ft5x06_ts_data *data)
{
	if (atomic_read(&data->st_enabled)) {
		if (atomic_cmpxchg(&data->st_pending_irqs, 0, 1) == 0) {
			reinit_completion(&data->st_irq_processed);
			ft5x06_secure_touch_notify(data);
			wait_for_completion_interruptible(
						&data->st_irq_processed);
		}
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/*
 * 'blocking' variable will have value 'true' when we want to prevent the driver
 * from accessing the xPU/SMMU protected HW resources while the session is
 * active.
 */
static void ft5x06_secure_touch_stop(struct ft5x06_ts_data *data, bool blocking)
{
	if (atomic_read(&data->st_enabled)) {
		atomic_set(&data->st_pending_irqs, -1);
		ft5x06_secure_touch_notify(data);
		if (blocking)
			wait_for_completion_interruptible(
						&data->st_powerdown);
	}
}

static int ft5x06_clk_prepare_enable(struct ft5x06_ts_data *data)
{
	int ret;

	ret = clk_prepare_enable(data->iface_clk);
	if (ret) {
		dev_err(&data->client->dev,
			"error on clk_prepare_enable(iface_clk):%d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(data->core_clk);
	if (ret) {
		clk_disable_unprepare(data->iface_clk);
		dev_err(&data->client->dev,
			"error clk_prepare_enable(core_clk):%d\n", ret);
	}
	return ret;
}

static void ft5x06_clk_disable_unprepare(struct ft5x06_ts_data *data)
{
	clk_disable_unprepare(data->core_clk);
	clk_disable_unprepare(data->iface_clk);
}

static int ft5x06_bus_get(struct ft5x06_ts_data *data)
{
	int retval;

	mutex_lock(&data->ft_clk_io_ctrl_mutex);
	retval = pm_runtime_get_sync(data->client->adapter->dev.parent);
	if (retval >= 0 &&  data->core_clk != NULL && data->iface_clk != NULL) {
		retval = ft5x06_clk_prepare_enable(data);
		if (retval)
			pm_runtime_put_sync(data->client->adapter->dev.parent);
	}
	mutex_unlock(&data->ft_clk_io_ctrl_mutex);
	return retval;
}

static void ft5x06_bus_put(struct ft5x06_ts_data *data)
{
	mutex_lock(&data->ft_clk_io_ctrl_mutex);
	if (data->core_clk != NULL && data->iface_clk != NULL)
		ft5x06_clk_disable_unprepare(data);
	pm_runtime_put_sync(data->client->adapter->dev.parent);
	mutex_unlock(&data->ft_clk_io_ctrl_mutex);
}

static ssize_t ft5x06_secure_touch_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d", atomic_read(&data->st_enabled));
}

/*
 * Accept only "0" and "1" valid values.
 * "0" will reset the st_enabled flag, then wake up the reading process and
 * the interrupt handler.
 * The bus driver is notified via pm_runtime that it is not required to stay
 * awake anymore.
 * It will also make sure the queue of events is emptied in the controller,
 * in case a touch happened in between the secure touch being disabled and
 * the local ISR being ungated.
 * "1" will set the st_enabled flag and clear the st_pending_irqs flag.
 * The bus driver is requested via pm_runtime to stay awake.
 */
static ssize_t ft5x06_secure_touch_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long value;
	int err = 0;

	if (count > 2)
		return -EINVAL;
	err = kstrtoul(buf, 10, &value);
	if (err != 0)
		return err;

	if (!data->st_initialized)
		return -EIO;

	err = count;
	switch (value) {
	case 0:
		if (atomic_read(&data->st_enabled) == 0)
			break;
		ft5x06_bus_put(data);
		atomic_set(&data->st_enabled, 0);
		ft5x06_secure_touch_notify(data);
		complete(&data->st_irq_processed);
		ft5x06_ts_interrupt(data->client->irq, data);
		complete(&data->st_powerdown);
		break;

	case 1:
		if (atomic_read(&data->st_enabled)) {
			err = -EBUSY;
			break;
		}
		synchronize_irq(data->client->irq);
		if (ft5x06_bus_get(data) < 0) {
			dev_err(&data->client->dev, "ft5x06_bus_get failed\n");
			err = -EIO;
			break;
		}
		reinit_completion(&data->st_powerdown);
		reinit_completion(&data->st_irq_processed);
		atomic_set(&data->st_enabled, 1);
		atomic_set(&data->st_pending_irqs,  0);
		break;

	default:
		dev_err(&data->client->dev, "unsupported value: %lu\n", value);
		err = -EINVAL;
		break;
	}
	return err;
}

/*
 * This function returns whether there are pending interrupts, or
 * other error conditions that need to be signaled to the userspace library,
 * according tot he following logic:
 * - st_enabled is 0 if secure touch is not enabled, returning -EBADF
 * - st_pending_irqs is -1 to signal that secure touch is in being stopped,
 *   returning -EINVAL
 * - st_pending_irqs is 1 to signal that there is a pending irq, returning
 *   the value "1" to the sysfs read operation
 * - st_pending_irqs is 0 (only remaining case left) if the pending interrupt
 *   has been processed, so the interrupt handler can be allowed to continue.
 */
static ssize_t ft5x06_secure_touch_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int val = 0;

	if (atomic_read(&data->st_enabled) == 0)
		return -EBADF;
	if (atomic_cmpxchg(&data->st_pending_irqs, -1, 0) == -1)
		return -EINVAL;
	if (atomic_cmpxchg(&data->st_pending_irqs, 1, 0) == 1)
		val = 1;
	else
		complete(&data->st_irq_processed);
	return scnprintf(buf, PAGE_SIZE, "%u", val);
}
#else
static void ft5x06_secure_touch_init(struct ft5x06_ts_data *data)
{
}
static irqreturn_t ft5x06_filter_interrupt(struct ft5x06_ts_data *data)
{
	return IRQ_NONE;
}
static void ft5x06_secure_touch_stop(struct ft5x06_ts_data *data, bool blocking)
{
}
#endif

static struct device_attribute attrs[] = {
#if defined(CONFIG_FT_SECURE_TOUCH)
		__ATTR(secure_touch_enable, (S_IRUGO | S_IWUSR | S_IWGRP),
				ft5x06_secure_touch_enable_show,
				ft5x06_secure_touch_enable_store),
		__ATTR(secure_touch, S_IRUGO ,
				ft5x06_secure_touch_show, NULL),
#endif
};

static inline bool ft5x06_gesture_support_enabled(void)
{
	return config_enabled(CONFIG_TOUCHSCREEN_FT5X06_GESTURE);
}
=======
#if defined(CONFIG_TOUCHSCREEN_GESTURE)
	int gesture_value;
#endif
#if defined(FTS_SCAP_TEST)
	struct mutex selftest_lock;
#endif
};

struct ft5x06_ts_data *ft5x06_ts = NULL;

extern int is_tp_driver_loaded;

#ifdef CONFIG_WAKE_GESTURES
bool scr_suspended_ft(void) {
	return ft5x06_ts->suspended;
}
#endif

static DEFINE_MUTEX(i2c_rw_access);
>>>>>>> p9x

static int ft5x06_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

<<<<<<< HEAD
=======
	mutex_lock(&i2c_rw_access);

>>>>>>> p9x
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
<<<<<<< HEAD
=======

	mutex_unlock(&i2c_rw_access);

>>>>>>> p9x
	return ret;
}

static int ft5x06_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;
<<<<<<< HEAD

=======
>>>>>>> p9x
	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
<<<<<<< HEAD
=======

	mutex_lock(&i2c_rw_access);

>>>>>>> p9x
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

<<<<<<< HEAD
=======
	mutex_unlock(&i2c_rw_access);

>>>>>>> p9x
	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return ft5x06_i2c_write(client, buf, sizeof(buf));
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return ft5x06_i2c_read(client, &addr, 1, val, 1);
}

<<<<<<< HEAD
#ifdef CONFIG_TOUCHSCREEN_FT5X06_GESTURE
static ssize_t ft5x06_gesture_enable_to_set_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			data->gesture_pdata->gesture_enable_to_set);
}

static ssize_t ft5x06_gesture_enable_to_set_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;

	if (data->suspended)
		return -EINVAL;

	ret = kstrtoul(buf, 16, &value);
	if (ret < 0) {
		dev_err(dev, "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}

	if (1 == value)
		data->gesture_pdata->gesture_enable_to_set = 1;
	else
		data->gesture_pdata->gesture_enable_to_set = 0;
	return size;
}

static DEVICE_ATTR(enable, 0664,
		ft5x06_gesture_enable_to_set_show,
		ft5x06_gesture_enable_to_set_store);

static int ft5x06_entry_pocket(struct device *dev)
{
	return ft5x06_ts_stop(dev);
}

static int ft5x06_leave_pocket(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

	ft5x06_ts_start(dev);
	ft5x0x_write_reg(data->client, FT_REG_GESTURE_ENABLE, 1);
	err = enable_irq_wake(data->client->irq);
	if (err)
		dev_err(&data->client->dev,
			"%s: set_irq_wake failed\n", __func__);
	data->suspended = true;

	return err;
}

static ssize_t gesture_in_pocket_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			data->gesture_pdata->in_pocket);
}

static ssize_t gesture_in_pocket_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;

	ret = kstrtoul(buf, 16, &value);
	if (ret < 0) {
		dev_err(dev, "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}

	if (1 == value && data->gesture_pdata->in_pocket == 0) {
		data->gesture_pdata->in_pocket = 1;
		ft5x06_entry_pocket(dev);
	} else if (0 == value && data->gesture_pdata->in_pocket == 1) {
		ft5x06_leave_pocket(dev);
		data->gesture_pdata->in_pocket = 0;
	}
	return size;
}

static DEVICE_ATTR(pocket, 0664,
		gesture_in_pocket_mode_show,
		gesture_in_pocket_mode_store);

static int ft5x06_report_gesture_doubleclick(struct input_dev *ip_dev)
{
	int i;

	for (i = 0; i < 2; i++) {
		input_mt_slot(ip_dev, FT_GESTURE_DEFAULT_TRACKING_ID);
		input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
		input_report_abs(ip_dev, ABS_MT_POSITION_X,
					FT_GESTURE_DOUBLECLICK_COORD_X);
		input_report_abs(ip_dev, ABS_MT_POSITION_Y,
					FT_GESTURE_DOUBLECLICK_COORD_Y);
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
		input_mt_slot(ip_dev, FT_GESTURE_DEFAULT_TRACKING_ID);
		input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}
	return 0;
}

static int ft5x06_report_gesture(struct i2c_client *i2c_client,
		struct input_dev *ip_dev)
{
	int i, temp, gesture_data_size;
	int gesture_coord_x, gesture_coord_y;
	int ret = -1;
	short pointnum = 0;
	unsigned char buf[FT_GESTURE_POINTER_NUM_MAX *
			FT_GESTURE_POINTER_SIZEOF + FT_GESTURE_DATA_HEADER];

	buf[0] = FT_REG_GESTURE_OUTPUT;
	ret = ft5x06_i2c_read(i2c_client, buf, 1,
				buf, FT_GESTURE_DATA_HEADER);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}

	/* FW support doubleclick */
	if (FT_GESTURE_DOUBLECLICK_ID == buf[0]) {
		ft5x06_report_gesture_doubleclick(ip_dev);
		return 0;
	}

	pointnum = (short)(buf[1]) & 0xff;
	gesture_data_size = pointnum * FT_GESTURE_POINTER_SIZEOF +
			FT_GESTURE_DATA_HEADER;
	buf[0] = FT_REG_GESTURE_OUTPUT;
	temp = gesture_data_size / I2C_TRANSFER_MAX_BYTE;
	for (i = 0; i < temp; i++)
		ret = ft5x06_i2c_read(i2c_client, buf, ((i == 0) ? 1 : 0),
			buf + I2C_TRANSFER_MAX_BYTE * i, I2C_TRANSFER_MAX_BYTE);
	ret = ft5x06_i2c_read(i2c_client, buf, ((temp == 0) ? 1 : 0),
			buf + I2C_TRANSFER_MAX_BYTE * temp,
			gesture_data_size - I2C_TRANSFER_MAX_BYTE * temp);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}

	for (i = 0; i < pointnum; i++) {
		gesture_coord_x = (((s16) buf[FT_GESTURE_DATA_HEADER +
				(FT_GESTURE_POINTER_SIZEOF * i)]) & 0x0F) << 8 |
				(((s16) buf[FT_GESTURE_DATA_HEADER + 1 +
				(FT_GESTURE_POINTER_SIZEOF * i)]) & 0xFF);
		gesture_coord_y = (((s16) buf[FT_GESTURE_DATA_HEADER + 2 +
				(FT_GESTURE_POINTER_SIZEOF * i)]) & 0x0F) << 8 |
				(((s16) buf[FT_GESTURE_DATA_HEADER + 3 +
				(FT_GESTURE_POINTER_SIZEOF * i)]) & 0xFF);
		input_mt_slot(ip_dev, FT_GESTURE_DEFAULT_TRACKING_ID);
		input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
		input_report_abs(ip_dev, ABS_MT_POSITION_X, gesture_coord_x);
		input_report_abs(ip_dev, ABS_MT_POSITION_Y, gesture_coord_y);
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}
	input_mt_slot(ip_dev, FT_GESTURE_DEFAULT_TRACKING_ID);
	input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
	input_mt_report_pointer_emulation(ip_dev, false);
	input_sync(ip_dev);

	return 0;
}
#else
static DEVICE_ATTR(pocket, 0664, NULL, NULL);
static DEVICE_ATTR(enable, 0664, NULL, NULL);

static int ft5x06_report_gesture(struct i2c_client *i2c_client,
		struct input_dev *ip_dev)
{
	return 0;
}
#endif

=======
>>>>>>> p9x
static void ft5x06_update_fw_vendor_id(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VENDOR_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_vendor_id, 1);
	if (err < 0)
		dev_err(&client->dev, "fw vendor id read failed");
}

static void ft5x06_update_fw_ver(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");

	reg_addr = FT_REG_FW_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		dev_err(&client->dev, "fw minor version read failed");

	reg_addr = FT_REG_FW_SUB_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		dev_err(&client->dev, "fw sub minor version read failed");

<<<<<<< HEAD
	dev_info(&client->dev, "Firmware version = %d.%d.%d\n",
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

=======
	dev_info(&client->dev, "%s, Firmware version = 0x%02x.%d.%d\n", __func__,
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

#ifdef FT_GESTURE
static int check_gesture(struct input_dev *dev, int gesture_id)
{
	struct input_dev *pdev = dev;
	u8 report_data = 0;
	int value = ctp_get_gesture_data();

	printk("%s, gesture_id=0x%02x, value=0x%08x\n", __func__, gesture_id, value);

	switch (gesture_id) {
	case GESTURE_DOUBLECLICK:
		report_data = DOUBLE_TAP;
		break;
	case GESTURE_RIGHT:
		report_data = SWIPE_X_RIGHT;
		break;
	case GESTURE_LEFT:
		report_data = SWIPE_X_LEFT;
		break;
	case GESTURE_DOWN:
		report_data = SWIPE_Y_DOWN;
		break;
	case GESTURE_UP:
		report_data = SWIPE_Y_UP;
		break;
	case GESTURE_E:
		report_data = UNICODE_E;
		break;
	case GESTURE_C:
		report_data = UNICODE_C;
		break;
	case GESTURE_W:
		report_data = UNICODE_W;
		break;
	case GESTURE_M:
		report_data = UNICODE_M;
		break;
	case GESTURE_O:
		report_data = UNICODE_O;
		break;
	case GESTURE_S:
		report_data = UNICODE_S;
		break;
	case GESTURE_V:
		report_data = UNICODE_V_DOWN;
		break;
	case GESTURE_Z:
		report_data = UNICODE_Z;
		break;
	default:
		break;
	}

	if (ctp_check_gesture_needed(report_data)) {
		ctp_set_gesture_data(report_data);

		input_report_key(pdev, BTN_TOUCH, 0);
		input_report_key(pdev, KEY_GESTURE, 1);
		input_sync(pdev);
		input_report_key(pdev, KEY_GESTURE, 0);
		input_sync(pdev);
	}

	return 0;
}

static int ft5x06_read_Touchdata(struct ft5x06_ts_data *data)
{
	struct ft5x06_ts_data *pdata = data;
	unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
	int ret = -1;
	int i = 0;
	int gesture_id = 0;
	short pointnum = 0;
	buf[0] = 0xd3;

	printk("%s\n", __func__);
	pointnum = 0;
	ret = ft5x06_i2c_read(pdata->client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	if (ret < 0) {
		printk("%s read touchdata failed.\n", __func__);
		return ret;
	}

	/* FW */
	gesture_id = (int)buf[0];
	pointnum = (short)(buf[1]) & 0xff;
	buf[0] = 0xd3;

	if ((pointnum * 4 + 8) < 255)
		ret = ft5x06_i2c_read(pdata->client, buf, 1, buf, (pointnum * 4 + 8));
	else {
		ret = ft5x06_i2c_read(pdata->client, buf, 1, buf, 255);
		ret = ft5x06_i2c_read(pdata->client, buf, 0, buf + 255, (pointnum * 4 + 8) - 255);
	}
	if (ret < 0) {
		printk("%s read touchdata failed.\n", __func__);
		return ret;
	}
	check_gesture(pdata->input_dev, gesture_id);
	for (i = 0; i < pointnum; i++) {
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
		8 | (((s16) buf[1 + (4 * i)]) & 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
		8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}
	return -EPERM;
}
#endif


>>>>>>> p9x
static irqreturn_t ft5x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x06_ts_data *data = dev_id;
	struct input_dev *ip_dev;
	int rc, i;
	u32 id, x, y, status, num_touches;
<<<<<<< HEAD
	u8 reg, *buf, gesture_is_active;
	bool update_input = false;
=======
	u8 reg = 0x00, *buf;
	bool update_input = false;
#ifdef FT_GESTURE
	u8 state;
#endif

#ifdef FT_GESTURE
	pr_debug("%s\n", __func__);
	ft5x0x_read_reg(data->client, 0xd0, &state);
	if (state == 1) {
		ft5x06_read_Touchdata(data);
		return IRQ_HANDLED;
	}
#endif
>>>>>>> p9x

	if (!data) {
		pr_err("%s: Invalid data\n", __func__);
		return IRQ_HANDLED;
	}

<<<<<<< HEAD
	if (IRQ_HANDLED == ft5x06_filter_interrupt(data))
		return IRQ_HANDLED;

	ip_dev = data->input_dev;
	buf = data->tch_data;

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support) {
		ft5x0x_read_reg(data->client, FT_REG_GESTURE_ENABLE,
					&gesture_is_active);
		if (gesture_is_active) {
			pm_wakeup_event(&(data->client->dev),
					FT_GESTURE_WAKEUP_TIMEOUT);
			ft5x06_report_gesture(data->client, ip_dev);
			return IRQ_HANDLED;
		}
	}

	/*
	 * Read touch data start from register FT_REG_DEV_MODE.
	 * The touch x/y value start from FT_TOUCH_X_H/L_POS and
	 * FT_TOUCH_Y_H/L_POS in buf.
	 */
	reg = FT_REG_DEV_MODE;
	rc = ft5x06_i2c_read(data->client, &reg, 1, buf, data->tch_data_len);
=======
	ip_dev = data->input_dev;
	buf = data->tch_data;

	rc = ft5x06_i2c_read(data->client, &reg, 1,
			buf, data->tch_data_len);
>>>>>>> p9x
	if (rc < 0) {
		dev_err(&data->client->dev, "%s: read data fail\n", __func__);
		return IRQ_HANDLED;
	}

	for (i = 0; i < data->pdata->num_max_touches; i++) {
<<<<<<< HEAD
		/*
		 * Getting the finger ID of the touch event incase of
		 * multiple touch events
		 */
=======
>>>>>>> p9x
		id = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
		if (id >= FT_MAX_ID)
			break;

		update_input = true;

		x = (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i]);
		y = (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i]);

		status = buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;

		num_touches = buf[FT_TD_STATUS] & FT_STATUS_NUM_TP_MASK;

		/* invalid combination */
		if (!num_touches && !status && !id)
			break;

<<<<<<< HEAD
=======
		if (y == 2100 && data->keypad_mode)
			break;

#ifdef CONFIG_WAKE_GESTURES
		if (data->suspended)
			x += 5000;
#endif

>>>>>>> p9x
		input_mt_slot(ip_dev, id);
		if (status == FT_TOUCH_DOWN || status == FT_TOUCH_CONTACT) {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ip_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ip_dev, ABS_MT_POSITION_Y, y);
		} else {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		}
	}

	if (update_input) {
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}

	return IRQ_HANDLED;
}

<<<<<<< HEAD
static int ft5x06_gpio_configure(struct ft5x06_ts_data *data, bool on)
{
	int err = 0;

	if (on) {
		if (gpio_is_valid(data->pdata->irq_gpio)) {
			err = gpio_request(data->pdata->irq_gpio,
						"ft5x06_irq_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"irq gpio request failed");
				goto err_irq_gpio_req;
			}

			err = gpio_direction_input(data->pdata->irq_gpio);
			if (err) {
				dev_err(&data->client->dev,
					"set_direction for irq gpio failed\n");
				goto err_irq_gpio_dir;
			}
		}

		if (gpio_is_valid(data->pdata->reset_gpio)) {
			err = gpio_request(data->pdata->reset_gpio,
						"ft5x06_reset_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"reset gpio request failed");
				goto err_irq_gpio_dir;
			}

			err = gpio_direction_output(data->pdata->reset_gpio, 0);
			if (err) {
				dev_err(&data->client->dev,
				"set_direction for reset gpio failed\n");
				goto err_reset_gpio_dir;
			}
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}

		return 0;
	}
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		/*
		 * This is intended to save leakage current
		 * only. Even if the call(gpio_direction_input)
		 * fails, only leakage current will be more but
		 * functionality will not be affected.
		 */
		err = gpio_direction_input(data->pdata->reset_gpio);
		if (err) {
			dev_err(&data->client->dev,
				"unable to set direction for gpio [%d]\n",
				data->pdata->irq_gpio);
		}
		gpio_free(data->pdata->reset_gpio);
	}

		return 0;

err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
	return err;
}

=======
>>>>>>> p9x
static int ft5x06_power_on(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

<<<<<<< HEAD
	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

=======
>>>>>>> p9x
	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

<<<<<<< HEAD
=======


	if (gpio_is_valid(data->pdata->power_ldo_gpio)) {
		printk("%s,  power_ldo_gpio\n", __func__);
		gpio_set_value(data->pdata->power_ldo_gpio, 1);
	}



	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}


	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	if (gpio_is_valid(data->pdata->power_ldo_gpio))
		gpio_set_value(data->pdata->power_ldo_gpio, 0);

	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

>>>>>>> p9x
	return rc;
}

static int ft5x06_power_init(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

<<<<<<< HEAD
	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}
=======

>>>>>>> p9x

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

<<<<<<< HEAD
=======


	if (gpio_is_valid(data->pdata->power_ldo_gpio)) {
		printk("%s, power_ldo_gpio\n", __func__);
		rc = gpio_request(data->pdata->power_ldo_gpio, "focaltech_ldo_gpio");
		if (rc) {
			printk("irq gpio request failed\n");
			goto Err_gpio_request;
		}

		rc = gpio_direction_output(data->pdata->power_ldo_gpio, 1);
		if (rc) {
			printk("set_direction for irq gpio failed\n");
			goto free_ldo_gpio;
		}
	}
	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

>>>>>>> p9x
	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
<<<<<<< HEAD
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
=======

reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

reg_vdd_put:
free_ldo_gpio:
	if (gpio_is_valid(data->pdata->power_ldo_gpio))
		gpio_free(data->pdata->power_ldo_gpio);
	regulator_put(data->vdd);
Err_gpio_request:
	return rc;

pwr_deinit:
	if (gpio_is_valid(data->pdata->power_ldo_gpio))
		gpio_free(data->pdata->power_ldo_gpio);

	if (regulator_count_voltages(data->vdd) > 0)
	regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
>>>>>>> p9x

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int ft5x06_ts_pinctrl_init(struct ft5x06_ts_data *ft5x06_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ft5x06_data->ts_pinctrl = devm_pinctrl_get(&(ft5x06_data->client->dev));
	if (IS_ERR_OR_NULL(ft5x06_data->ts_pinctrl)) {
		retval = PTR_ERR(ft5x06_data->ts_pinctrl);
		dev_dbg(&ft5x06_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	ft5x06_data->pinctrl_state_active
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_active)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_active);
		dev_err(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	ft5x06_data->pinctrl_state_suspend
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_suspend);
		dev_err(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	ft5x06_data->pinctrl_state_release
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_release)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_release);
		dev_dbg(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(ft5x06_data->ts_pinctrl);
err_pinctrl_get:
	ft5x06_data->ts_pinctrl = NULL;
	return retval;
}

<<<<<<< HEAD
#ifdef CONFIG_PM
static int ft5x06_ts_start(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}

	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
				data->pinctrl_state_active);
		if (err < 0)
			dev_err(dev, "Cannot get active pinctrl state\n");
	}

	err = ft5x06_gpio_configure(data, true);
	if (err < 0) {
		dev_err(&data->client->dev,
			"failed to put gpios in resue state\n");
		goto err_gpio_configuration;
	}
=======
int ft5x05_gesture_mode_enter(struct i2c_client *client)
{
	struct i2c_client *pclient = client;

	printk("%s\n", __func__);
	ft5x0x_write_reg(pclient, 0xd0, 0x01);
	ft5x0x_write_reg(pclient, 0xd1, 0xff);
	ft5x0x_write_reg(pclient, 0xd2, 0xff);
	ft5x0x_write_reg(pclient, 0xd5, 0xff);
	ft5x0x_write_reg(pclient, 0xd6, 0xff);
	ft5x0x_write_reg(pclient, 0xd7, 0xff);
	ft5x0x_write_reg(pclient, 0xd8, 0xff);

	return 0;
}

int ft5x05_gesture_mode_exit(struct i2c_client *client)
{
	struct i2c_client *pclient = client;
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);

	printk("%s\n", __func__);
	ft5x0x_write_reg(pclient, 0xd0, 0x00);
>>>>>>> p9x

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	msleep(data->pdata->soft_rst_dly);

<<<<<<< HEAD
	enable_irq(data->client->irq);
	data->suspended = false;

	return 0;

err_gpio_configuration:
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_suspend);
		if (err < 0)
			dev_err(dev, "Cannot get suspend pinctrl state\n");
	}
	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err)
			dev_err(dev, "power off failed");
	} else {
		err = ft5x06_power_on(data, false);
		if (err)
			dev_err(dev, "power off failed");
	}
	return err;
}

static int ft5x06_ts_stop(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2];
	int i, err;
=======
	return 0;
}

#ifdef CONFIG_PM
static int ft5x06_ts_suspend(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2], i;
	int err;

#ifdef FT_GESTURE
	if (ctp_get_gesture_control()) {
		ft5x05_gesture_mode_enter(data->client);
		data->suspended = true;
		return 0;
	}
#endif


	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

#ifdef CONFIG_WAKE_GESTURES
	if (device_may_wakeup(dev) && (s2w_switch || dt2w_switch)) {
		ft5x0x_write_reg(data->client, 0xD0, 1);
		err = enable_irq_wake(data->client->irq);
		if (err)
			dev_err(&data->client->dev,
				"%s: set_irq_wake failed\n", __func__);
		data->suspended = true;

		return err;
	}
#endif
>>>>>>> p9x

	disable_irq(data->client->irq);

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		txbuf[0] = FT_REG_PMODE;
		txbuf[1] = FT_PMODE_HIBERNATE;
		ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
	}

	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	} else {
		err = ft5x06_power_on(data, false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	}

<<<<<<< HEAD
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_suspend);
		if (err < 0)
			dev_err(dev, "Cannot get suspend pinctrl state\n");
	}

	err = ft5x06_gpio_configure(data, false);
	if (err < 0) {
		dev_err(&data->client->dev,
			"failed to put gpios in suspend state\n");
		goto gpio_configure_fail;
	}

=======
>>>>>>> p9x
	data->suspended = true;

	return 0;

<<<<<<< HEAD
gpio_configure_fail:
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_active);
		if (err < 0)
			dev_err(dev, "Cannot get active pinctrl state\n");
	}
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err)
			dev_err(dev, "power on failed");
	} else {
		err = ft5x06_power_on(data, true);
		if (err)
			dev_err(dev, "power on failed");
	}
=======
>>>>>>> p9x
pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	enable_irq(data->client->irq);
	return err;
}

<<<<<<< HEAD
static int ft5x06_ts_suspend(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

	ft5x06_secure_touch_stop(data, true);

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support &&
		device_may_wakeup(dev) &&
		data->gesture_pdata->gesture_enable_to_set) {

		ft5x0x_write_reg(data->client, FT_REG_GESTURE_ENABLE, 1);
		err = enable_irq_wake(data->client->irq);
		if (err)
			dev_err(&data->client->dev,
				"%s: set_irq_wake failed\n", __func__);
		data->suspended = true;
		return err;
	}

	return ft5x06_ts_stop(dev);
}

=======
>>>>>>> p9x
static int ft5x06_ts_resume(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

<<<<<<< HEAD
=======
#ifdef CONFIG_WAKE_GESTURES
	int i;
#endif

>>>>>>> p9x
	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}

<<<<<<< HEAD
	ft5x06_secure_touch_stop(data, true);

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support &&
		device_may_wakeup(dev) &&
		!(data->gesture_pdata->in_pocket) &&
		data->gesture_pdata->gesture_enable_to_set) {

		ft5x0x_write_reg(data->client, FT_REG_GESTURE_ENABLE, 0);
=======
#ifdef CONFIG_WAKE_GESTURES
	if (device_may_wakeup(dev) && (s2w_switch || dt2w_switch)) {
		ft5x0x_write_reg(data->client, 0xD0, 0);

		for (i = 0; i < data->pdata->num_max_touches; i++) {
			input_mt_slot(data->input_dev, i);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
		}
		input_mt_report_pointer_emulation(data->input_dev, false);
		input_sync(data->input_dev);

>>>>>>> p9x
		err = disable_irq_wake(data->client->irq);
		if (err)
			dev_err(dev, "%s: disable_irq_wake failed\n",
				__func__);
		data->suspended = false;
<<<<<<< HEAD
		return err;
	}

	err = ft5x06_ts_start(dev);
	if (err < 0)
		return err;

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support &&
		device_may_wakeup(dev) &&
		data->gesture_pdata->in_pocket &&
		data->gesture_pdata->gesture_enable_to_set) {

		ft5x0x_write_reg(data->client, FT_REG_GESTURE_ENABLE, 0);
		err = disable_irq_wake(data->client->irq);
		if (err)
			dev_err(dev, "%s: disable_irq_wake failed\n",
				__func__);
		data->suspended = false;
		data->gesture_pdata->in_pocket = 0;
	}
	return 0;
}

=======

		if (dt2w_switch_changed) {
			dt2w_switch = dt2w_switch_temp;
			dt2w_switch_changed = false;
		}
		if (s2w_switch_changed) {
			s2w_switch = s2w_switch_temp;
			s2w_switch_changed = false;
		}

		return err;
	}
#endif

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);


	udelay(100);
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}
		 udelay(500);
		 udelay(500);
		  udelay(500);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	msleep(data->pdata->soft_rst_dly);

	enable_irq(data->client->irq);
	data->suspended = false;


/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 0;
}



>>>>>>> p9x
static const struct dev_pm_ops ft5x06_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = ft5x06_ts_suspend,
	.resume = ft5x06_ts_resume,
#endif
};

#else
static int ft5x06_ts_suspend(struct device *dev)
{
	return 0;
}

static int ft5x06_ts_resume(struct device *dev)
{
	return 0;
}

#endif

#if defined(CONFIG_FB)
<<<<<<< HEAD
static void fb_notify_resume_work(struct work_struct *work)
{
	struct ft5x06_ts_data *ft5x06_data =
		container_of(work, struct ft5x06_ts_data, fb_notify_work);
	ft5x06_ts_resume(&ft5x06_data->client->dev);
}

=======
>>>>>>> p9x
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x06_ts_data *ft5x06_data =
		container_of(self, struct ft5x06_ts_data, fb_notif);

<<<<<<< HEAD
	if (evdata && evdata->data && ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		if (ft5x06_data->pdata->resume_in_workqueue) {
			if (event == FB_EARLY_EVENT_BLANK &&
						 *blank == FB_BLANK_UNBLANK)
				schedule_work(&ft5x06_data->fb_notify_work);
			else if (event == FB_EVENT_BLANK &&
						 *blank == FB_BLANK_POWERDOWN) {
				flush_work(&ft5x06_data->fb_notify_work);
				ft5x06_ts_suspend(&ft5x06_data->client->dev);
			}
		} else {
			if (event == FB_EVENT_BLANK) {
				if (*blank == FB_BLANK_UNBLANK)
					ft5x06_ts_resume(
						&ft5x06_data->client->dev);
				else if (*blank == FB_BLANK_POWERDOWN)
					ft5x06_ts_suspend(
						&ft5x06_data->client->dev);
			}
		}
=======
	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK || *blank == FB_BLANK_NORMAL)
			ft5x06_ts_resume(&ft5x06_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			ft5x06_ts_suspend(&ft5x06_data->client->dev);
>>>>>>> p9x
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

<<<<<<< HEAD
	/*
	 * During early suspend/late resume, the driver doesn't access xPU/SMMU
	 * protected HW resources. So, there is no compelling need to block,
	 * but notifying the userspace that a power event has occurred is
	 * enough. Hence 'blocking' variable can be set to false.
	 */
	ft5x06_secure_touch_stop(data, false);
=======
>>>>>>> p9x
	ft5x06_ts_suspend(&data->client->dev);
}

static void ft5x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

<<<<<<< HEAD
	ft5x06_secure_touch_stop(data, false);
=======
>>>>>>> p9x
	ft5x06_ts_resume(&data->client->dev);
}
#endif

<<<<<<< HEAD
=======
#if defined(CONFIG_TOUCHSCREEN_GESTURE)
static int lct_ctp_gesture_mode_switch(int enable)
{
	int val = enable, ret = 0;

	if (val)
		ret = ft5x05_gesture_mode_enter(ft5x06_ts->client);
	else
		ret = ft5x05_gesture_mode_exit(ft5x06_ts->client);

	return ret;
}
#endif

int ft5x06_set_cover_mode(int enable)
{
	int ret = 0, val = enable;

	printk("%s, enable=%d\n", __func__, enable);
	if (val)
		ret = ft5x0x_write_reg(ft5x06_ts->client, 0xC1, 0x01);
	else
		ret = ft5x0x_write_reg(ft5x06_ts->client, 0xC1, 0x00);

	return ret;
}

#if defined(CONFIG_TOUCHSCREEN_COVER)
static int lct_ctp_cover_state_switch(int enable)
{
	int val = enable, ret = 0;

	ret = ft5x06_set_cover_mode(val);

	return ret;
}
#endif

#if defined(FTS_SCAP_TEST)

#define FT5X0X_INI_FILEPATH "/system/etc/firmware/"

static int ft5x0x_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s%s", FT5X0X_INI_FILEPATH, config_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x0x_ReadInIData(char *config_name,
			      char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5X0X_INI_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}
static int ft5x0x_get_testparam_from_ini(char *config_name)
{
	char *filedata = NULL;

	int inisize = ft5x0x_GetInISize(config_name);

	pr_info("inisize = %d \n ", inisize);
	if (inisize <= 0) {
		pr_err("%s ERROR:Get firmware size failed\n",
					__func__);
		return -EIO;
	}

	filedata = kmalloc(inisize + 1, GFP_ATOMIC);

	if (ft5x0x_ReadInIData(config_name, filedata)) {
		pr_err("%s() - ERROR: request_firmware failed\n",
					__func__);
		kfree(filedata);
		return -EIO;
	} else {
		pr_info("ft5x0x_ReadInIData successful\n");
	}

	SetParamData(filedata);
	return 0;
}

int focal_i2c_Read(unsigned char *writebuf,
			int writelen, unsigned char *readbuf, int readlen)
{
	unsigned char *p_w_buf = writebuf, *p_r_buf = readbuf;
	int w_len = writelen, r_len = readlen;

	return ft5x06_i2c_read(ft5x06_ts->client, p_w_buf, w_len, p_r_buf, r_len);
}

int focal_i2c_Write(unsigned char *writebuf, int writelen)
{
	unsigned char *pbuf = writebuf;
	int len = writelen;

	return ft5x06_i2c_write(ft5x06_ts->client, pbuf, len);
}

int ft5x06_self_test(void)
{
	int pf_value = 0x00;
	char cfgname[] = "ft5x06_selftest.ini";

	printk("%s\n", __func__);
	mutex_lock(&ft5x06_ts->selftest_lock);

	Init_I2C_Write_Func(focal_i2c_Write);
	Init_I2C_Read_Func(focal_i2c_Read);
	if (ft5x0x_get_testparam_from_ini(cfgname) < 0)
		printk("get testparam from ini failure\n");
	else {
		if (true == StartTestTP()) {
			printk("tp test pass\n");
			pf_value = 0x0;
		} else {
			pf_value = 0x1;
			printk("tp test failure\n");
		}
		FreeTestParamData();
	}
	mutex_unlock(&ft5x06_ts->selftest_lock);
	return pf_value;
}

#endif

>>>>>>> p9x
static int ft5x06_auto_cal(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	u8 temp = 0, i;

	/* set to factory mode */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* start calibration */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_START);
	msleep(2 * data->pdata->soft_rst_dly);
	for (i = 0; i < FT_CAL_RETRY; i++) {
		ft5x0x_read_reg(client, FT_REG_CAL, &temp);
<<<<<<< HEAD
		/* return to normal mode, calibration finish */
=======
		/*return to normal mode, calibration finish */
>>>>>>> p9x
		if (((temp & FT_CAL_MASK) >> FT_4BIT_SHIFT) == FT_CAL_FIN)
			break;
	}

	/*calibration OK */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* store calibration data */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_STORE);
	msleep(2 * data->pdata->soft_rst_dly);

	/* set to normal mode */
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_WORKMODE_VALUE);
	msleep(2 * data->pdata->soft_rst_dly);

	return 0;
}
<<<<<<< HEAD
=======
/**add by sven,starting**/
int hid_to_i2c(struct i2c_client *client)
{
	u8 auc_i2c_write_buf[5] = {0};
	int bRet = 0;

	auc_i2c_write_buf[0] = 0xeb;
	auc_i2c_write_buf[1] = 0xaa;
	auc_i2c_write_buf[2] = 0x09;

	ft5x06_i2c_write(client, auc_i2c_write_buf, 3);

	msleep(10);

	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;

	ft5x06_i2c_read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);
	printk("%s, auc_i2c_write_buf=0x%x, 0x%x, 0x%x\n", __func__, 
		auc_i2c_write_buf[0], auc_i2c_write_buf[1], auc_i2c_write_buf[2]);
	if (0xeb == auc_i2c_write_buf[0] && 0xaa == auc_i2c_write_buf[1] && 0x08 == auc_i2c_write_buf[2])
		bRet = 1;
	else
		bRet = 0;

	return bRet;

}

static int ft5x46_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 fw_ecc;
	int i_ret;

	printk("%s\n", __func__);
	i_ret = hid_to_i2c(client);

	if (i_ret == 0)
		printk("[FTS] hid1 change to i2c fail ! \n");

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);

		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc */
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(2);
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_55);

		msleep(200);

		i_ret = hid_to_i2c(client);

		if (i_ret == 0)
			printk("[FTS] hid%d change to i2c fail ! \n", i);
		msleep(10);

		/*********Step 2:Enter upgrade mode *****/
		w_buf[0] = FT_UPGRADE_55;
		w_buf[1] = FT_UPGRADE_AA;
		i_ret = ft5x06_i2c_write(client, w_buf, 2);

		if (i_ret < 0) {
			printk("[FTS] failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(1);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		r_buf[0] = r_buf[1] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] == 0x54 && r_buf[1] == 0x2c) {
			printk("Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
				i, r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_2);
			break;
		} else {
			printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				r_buf[0], r_buf[1]);
			continue;
		}
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	/******Step 4:erase app and panel paramenter area******/
	printk("Step 4:erase app and panel paramenter area\n");
	w_buf[0] = 0x61;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(1350);

	for (i = 0; i < 15; i++) {
		w_buf[0] = 0x6a;
		r_buf[0] = r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

		if (0xF0 == r_buf[0] && 0xAA == r_buf[1])
			break;
		msleep(50);

	}

	w_buf[0] = 0xB0;
	w_buf[1] = (u8) ((data_len >> 16) & 0xFF);
	w_buf[2] = (u8) ((data_len >> 8) & 0xFF);
	w_buf[3] = (u8) (data_len & 0xFF);

	ft5x06_i2c_write(client, w_buf, 4);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	fw_ecc = 0;
	printk("Step 5:write firmware(FW) to ctpm flash\n");
	temp = 0;
	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_buf[0] = 0xbf;
	pkt_buf[1] = 0x00;

	for (j = 0; j < pkt_num; j++) {
		temp = j * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> 8);
		pkt_buf[3] = (u8) temp;
		pkt_len = FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (pkt_len >> 8);
		pkt_buf[5] = (u8) pkt_len;

		for (i = 0; i < FT_FW_PKT_LEN; i++) {
			pkt_buf[6 + i] = data[j * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}
		ft5x06_i2c_write(client, pkt_buf, FT_FW_PKT_LEN + 6);

		for (i = 0; i < 30; i++) {
			w_buf[0] = 0x6a;
			r_buf[0] = r_buf[1] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

			if ((j + 0x1000) == (((r_buf[0]) << 8) | r_buf[1]))
				break;
			msleep(1);

		}
	}

	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> 8);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> 8);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}
		ft5x06_i2c_write(client, pkt_buf, temp + 6);

		for (i = 0; i < 30; i++) {
			w_buf[0] = 0x6a;
			r_buf[0] = r_buf[1] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

			if ((j + 0x1000) == (((r_buf[0]) << 8) | r_buf[1]))
				break;
			msleep(1);

		}
	}

	msleep(50);

/*********Step 6: read out checksum***********************/
	/*send the opration head */
	printk("Step 6: read out checksum\n");
	w_buf[0] = 0x64;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(300);

	temp = 0;
	w_buf[0] = 0x65;
	w_buf[1] = (u8)(temp >> 16);
	w_buf[2] = (u8)(temp >> 8);
	w_buf[3] = (u8)(temp);
	temp = data_len;
	w_buf[4] = (u8)(temp >> 8);
	w_buf[5] = (u8)(temp);
	i_ret = ft5x06_i2c_write(client, w_buf, 6);
	msleep(data_len/256);

	for (i = 0; i < 100; i++) {
		w_buf[0] = 0x6a;
		r_buf[0] = r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

		if (0xF0 == r_buf[0] && 0x55 == r_buf[1])
			break;
		msleep(1);

	}
	w_buf[0] = 0x66;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					r_buf[0],
					fw_ecc);

		return -EIO;
	}
	printk(KERN_WARNING "checksum %X %X \n", r_buf[0], fw_ecc);

	/*********Step 7: reset the new FW***********************/
	printk("Step 7: reset the new FW\n");
	w_buf[0] = 0x07;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(130);

	return 0;


}

/**add by sven,Ending**/
>>>>>>> p9x

static int ft5x06_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 is_5336_new_bootloader = false;
	u8 is_5336_fwsize_30 = false;
	u8 fw_ecc;

	/* determine firmware size */
	if (*(data + data_len - FT_BLOADER_SIZE_OFF) == FT_BLOADER_NEW_SIZE)
		is_5336_fwsize_30 = true;
	else
		is_5336_fwsize_30 = false;

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);
		/* reset - write 0xaa and 0x55 to reset register */
		if (ts_data->family_id == FT6X06_ID
			|| ts_data->family_id == FT6X36_ID)
			reset_reg = FT_RST_CMD_REG2;
		else
			reset_reg = FT_RST_CMD_REG1;

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
		msleep(info.delay_aa);

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
		if (i <= (FT_UPGRADE_LOOP / 2))
			msleep(info.delay_55 + i * 3);
		else
			msleep(info.delay_55 - (i - (FT_UPGRADE_LOOP / 2)) * 2);

		/* Enter upgrade mode */
		w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, w_buf, 1);
<<<<<<< HEAD
		usleep_range(FT_55_AA_DLY_NS, FT_55_AA_DLY_NS + 1);
=======
		usleep(FT_55_AA_DLY_NS);
>>>>>>> p9x
		w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, w_buf, 1);

		/* check READ_ID */
		msleep(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
<<<<<<< HEAD

=======
>>>>>>> p9x
		if (r_buf[0] != info.upgrade_id_1
			|| r_buf[1] != info.upgrade_id_2) {
			dev_err(&client->dev, "Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
				i, r_buf[0], r_buf[1],
				info.upgrade_id_1, info.upgrade_id_2);
		} else
			break;
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	w_buf[0] = 0xcd;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] <= 4)
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
	else if (r_buf[0] == 7)
		is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
	else if (r_buf[0] >= 0x0f &&
		((ts_data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x14)))
		is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
	else
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

	dev_dbg(&client->dev, "bootloader type=%d, r_buf=0x%x, family_id=0x%x\n",
		is_5336_new_bootloader, r_buf[0], ts_data->family_id);
	/* is_5336_new_bootloader = FT_BLOADER_VERSION_GZF; */

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	if (is_5336_fwsize_30) {
		w_buf[0] = FT_ERASE_PANEL_REG;
		ft5x06_i2c_write(client, w_buf, 1);
	}
	msleep(FT_EARSE_DLY_MS);

	/* program firmware */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4
		|| is_5336_new_bootloader == FT_BLOADER_VERSION_Z7)
		data_len = data_len - FT_DATA_LEN_OFF_OLD_FW;
	else
		data_len = data_len - FT_DATA_LEN_OFF_NEW_FW;

	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send remaining bytes */
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send the finishing packet */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4 ||
		is_5336_new_bootloader == FT_BLOADER_VERSION_Z7) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_OLD_FW; i++) {
			if (is_5336_new_bootloader  == FT_BLOADER_VERSION_Z7)
				temp = FT_MAGIC_BLOADER_Z7 + i;
			else if (is_5336_new_bootloader ==
						FT_BLOADER_VERSION_LZ4)
				temp = FT_MAGIC_BLOADER_LZ4 + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);
		}
	} else if (is_5336_new_bootloader == FT_BLOADER_VERSION_GZF) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_NEW_FW; i++) {
			if (is_5336_fwsize_30)
				temp = FT_MAGIC_BLOADER_GZF_30 + i;
			else
				temp = FT_MAGIC_BLOADER_GZF + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);

		}
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade successful\n");

	return 0;
}

static int ft5x06_fw_upgrade(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min, fw_file_vendor_id;
	bool fw_upgrade = false;

<<<<<<< HEAD
=======
	printk("%s, suspended=%d\n", __func__, data->suspended);
>>>>>>> p9x
	if (data->suspended) {
		dev_err(dev, "Device is in suspend state: Exit FW upgrade\n");
		return -EBUSY;
	}

	rc = request_firmware(&fw, data->fw_name, dev);
	if (rc < 0) {
		dev_err(dev, "Request firmware failed - %s (%d)\n",
						data->fw_name, rc);
		return rc;
	}

	if (fw->size < FT_FW_MIN_SIZE || fw->size > FT_FW_MAX_SIZE) {
		dev_err(dev, "Invalid firmware size (%zu)\n", fw->size);
		rc = -EIO;
		goto rel_fw;
	}

	if (data->family_id == FT6X36_ID) {
		fw_file_maj = FT_FW_FILE_MAJ_VER_FT6X36(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID_FT6X36(fw);
	} else {
		fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID(fw);
	}
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);
<<<<<<< HEAD
	fw_file_vendor_id = FT_FW_FILE_VENDOR_ID(fw);
=======
>>>>>>> p9x

	dev_info(dev, "Current firmware: %d.%d.%d", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	dev_info(dev, "New firmware: %d.%d.%d", fw_file_maj,
				fw_file_min, fw_file_sub_min);

	if (force)
		fw_upgrade = true;
	else if ((data->fw_ver[0] < fw_file_maj) &&
		data->fw_vendor_id == fw_file_vendor_id)
		fw_upgrade = true;

	if (!fw_upgrade) {
		dev_info(dev, "Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	/* start firmware upgrade */
<<<<<<< HEAD
	if (FT_FW_CHECK(fw, data)) {
=======
	if (data->family_id == FT5X46_ID) {
		rc = ft5x46_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0)
			dev_err(dev, "update failed (%d). try later...\n", rc);
		else if (data->pdata->info.auto_cal)
			ft5x06_auto_cal(data->client);
	} else if (FT_FW_CHECK(fw, data)) {
>>>>>>> p9x
		rc = ft5x06_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0)
			dev_err(dev, "update failed (%d). try later...\n", rc);
		else if (data->pdata->info.auto_cal)
			ft5x06_auto_cal(data->client);
	} else {
		dev_err(dev, "FW format error\n");
		rc = -EIO;
	}

	ft5x06_update_fw_ver(data);

<<<<<<< HEAD
	FT_STORE_TS_DBG_INFO(data->ts_info, data->family_id, data->pdata->name,
=======
	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
>>>>>>> p9x
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);
<<<<<<< HEAD
	FT_STORE_TS_INFO(ts_info_buff, data->family_id, data->fw_ver[0],
			 data->fw_ver[1], data->fw_ver[2]);
rel_fw:
	release_firmware(fw);
=======
rel_fw:
	release_firmware(fw);
	printk("%s done\n", __func__);
	return rc;
}

static unsigned char ft5x06_fw_Vid_get_from_boot(struct i2c_client *client)
{
	unsigned char auc_i2c_write_buf[10];
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	unsigned char i = 0;
	unsigned char vid = 0xFF;
	int i_ret;

	i_ret = hid_to_i2c(client);

	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);

		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc */
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(2);
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_55);

		msleep(200);

		i_ret = hid_to_i2c(client);

		if (i_ret == 0)
			printk("[FTS] hid%d change to i2c fail ! \n", i);
		msleep(10);

		/*********Step 2:Enter upgrade mode *****/
		w_buf[0] = FT_UPGRADE_55;
		w_buf[1] = FT_UPGRADE_AA;
		i_ret = ft5x06_i2c_write(client, w_buf, 2);

		if (i_ret < 0) {
			printk("[FTS] failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(1);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		r_buf[0] = r_buf[1] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] == 0x54 && r_buf[1] == 0x2c) {
			/*
			printk("Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
				i, r_buf[0], r_buf[1],info.upgrade_id_1, info.upgrade_id_2);
				*/
			break;
		} else{
			printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				r_buf[0], r_buf[1]);
			continue;
		}
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	printk("FTS_UPGRADE_LOOP ok is  i = %d \n", i);

	msleep(10);
	auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;
	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		auc_i2c_write_buf[2] = 0xd7;
		auc_i2c_write_buf[3] = 0x83;
		i_ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 4);
		if (i_ret < 0) {
			printk("[FTS] Step 4: read lcm id from flash error when i2c write, i_ret = %d\n", i_ret);
			continue;
		}
		i_ret = ft5x06_i2c_read(client, auc_i2c_write_buf, 0, r_buf, 2);
		if (i_ret < 0) {
			printk("[FTS] Step 4: read lcm id from flash error when i2c write, i_ret = %d\n", i_ret);
			continue;
		}

		vid = r_buf[1];

		printk("%s: REG VAL ID1 = 0x%x,ID2 = 0x%x\n", __func__, r_buf[0], r_buf[1]);
		break;
	}

	printk("%s: reset the tp\n", __func__);
	auc_i2c_write_buf[0] = 0x07;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);
	return vid;
}

static unsigned char ft5x06_fw_LockDownInfo_get_from_boot(struct i2c_client *client, char *pProjectCode)
{
	unsigned char auc_i2c_write_buf[10];
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[10] = {0};
	unsigned char i = 0, j = 0;

	int i_ret;

	printk("get_Vid_from_boot, fw_vendor_id=0x%02x\n",  uc_tp_vendor_id);
	i_ret = hid_to_i2c(client);

	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);

		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc */
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(2);
		ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_55);

		msleep(200);

		i_ret = hid_to_i2c(client);

		if (i_ret == 0)
			printk("[FTS] hid%d change to i2c fail ! \n", i);
		msleep(10);

		/*********Step 2:Enter upgrade mode *****/
		w_buf[0] = FT_UPGRADE_55;
		w_buf[1] = FT_UPGRADE_AA;
		i_ret = ft5x06_i2c_write(client, w_buf, 2);

		if (i_ret < 0) {
			printk("[FTS] failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(10);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		r_buf[0] = r_buf[1] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] == 0x54 && r_buf[1] == 0x2c) {
			/*
			printk("Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
				i, r_buf[0], r_buf[1],info.upgrade_id_1, info.upgrade_id_2);
				*/
			break;
		} else{
			printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				r_buf[0], r_buf[1]);
			continue;
		}
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	printk("FTS_UPGRADE_LOOP ok is  i = %d \n", i);

	/********* Step 4: read project code from app param area ***********************/
	msleep(10);
	auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;
	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		auc_i2c_write_buf[2] = 0xd7;
		auc_i2c_write_buf[3] = 0xa0;
		i_ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 4);
		if (i_ret < 0) {
			printk("[FTS] Step 4: read lcm id from flash error when i2c write, i_ret = %d\n", i_ret);
			continue;
		}
			msleep(10);
		i_ret = ft5x06_i2c_read(client, auc_i2c_write_buf, 0, r_buf, 8);
		if (i_ret < 0) {
			printk("[FTS] Step 4: read lcm id from flash error when i2c write, i_ret = %d\n", i_ret);
			continue;
		}

		for (j = 0; j < 8; j++)
			printk("%s: REG VAL = 0x%02x,j=%d\n", __func__, r_buf[j], j);
		sprintf(pProjectCode, "%02x%02x%02x%02x%02x%02x%02x%02x", \
				r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5], r_buf[6], r_buf[7]);
		break;
	}

	printk("%s: reset the tp\n", __func__);
	auc_i2c_write_buf[0] = 0x07;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);
	return 0;
}

static int ft5x06_fw_upgrade_by_array_data(struct device *dev, char *fw_data, int size, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	struct firmware *fw = NULL;
	int rc;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min, fw_file_vendor_id;
	bool fw_upgrade = false;
	char *pfw_data = fw_data;
	int fw_size = size;

	printk("%s, suspended=%d\n", __func__, data->suspended);
	if (data->suspended) {
		dev_err(dev, "Device is in suspend state: Exit FW upgrade\n");
		return -EBUSY;
	}

	fw = kzalloc(sizeof(struct firmware), GFP_KERNEL);
	fw->size = fw_size;
	fw->data = pfw_data;

	if (fw->size < FT_FW_MIN_SIZE || fw->size > FT_FW_MAX_SIZE) {
		dev_err(dev, "Invalid firmware size (%zu)\n", fw->size);
		rc = -EIO;
		goto rel_fw;
	}

	if (data->family_id == FT6X36_ID) {
		fw_file_maj = FT_FW_FILE_MAJ_VER_FT6X36(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID_FT6X36(fw);
	} else {
		fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID(fw);
	}
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);

	dev_info(dev, "Current firmware: 0x%02x.%d.%d", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	dev_info(dev, "New firmware: 0x%02x.%d.%d", fw_file_maj,
				fw_file_min, fw_file_sub_min);
	force = 0;
	if (force)
		fw_upgrade = true;
	else if (data->fw_ver[0] != fw_file_maj)
		fw_upgrade = true;

	if (!fw_upgrade) {
		dev_info(dev, "Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	/* start firmware upgrade */
	if (data->family_id == FT5X46_ID) {
		rc = ft5x46_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0)
			dev_err(dev, "update failed (%d). try later...\n", rc);
	} else if (FT_FW_CHECK(fw, data)) {
		rc = ft5x06_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0)
			dev_err(dev, "update failed (%d). try later...\n", rc);
		else if (data->pdata->info.auto_cal)
			ft5x06_auto_cal(data->client);
	} else {
		dev_err(dev, "FW format error\n");
		rc = -EIO;
	}


	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);
rel_fw:
	kfree(fw);
	printk("%s done\n", __func__);
>>>>>>> p9x
	return rc;
}

static ssize_t ft5x06_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
<<<<<<< HEAD

=======
>>>>>>> p9x
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t ft5x06_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (data->suspended) {
		dev_info(dev, "In suspend state, try again later...\n");
		return size;
	}

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, false);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_update_fw_store);

static ssize_t ft5x06_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, true);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_force_update_fw_store);

static ssize_t ft5x06_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
<<<<<<< HEAD

=======
>>>>>>> p9x
	return snprintf(buf, FT_FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);
}

static ssize_t ft5x06_fw_name_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->fw_name, buf, size);
	if (data->fw_name[size-1] == '\n')
		data->fw_name[size-1] = 0;

	return size;
}

<<<<<<< HEAD
static DEVICE_ATTR(fw_name, 0664, ft5x06_fw_name_show, ft5x06_fw_name_store);

static ssize_t ts_info_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	strlcpy(buf, ts_info_buff, FT_INFO_MAX_LEN);
	return strnlen(buf, FT_INFO_MAX_LEN);
}
static struct kobj_attribute ts_info_attr = __ATTR_RO(ts_info);
=======

static DEVICE_ATTR(fw_name, 0664, ft5x06_fw_name_show, ft5x06_fw_name_store);



static ssize_t ft5x06_lockdown_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = fts_proc_entry_i2c_client;

	struct ft5x06_ts_data *data = i2c_get_clientdata(client);

	size_t ret = -1;

	unsigned char buf1[1000];
	int readlen = 0;
	int rettwo = -1;
	ret = ft5x06_i2c_read(client, NULL, 0, buf1, readlen);
	if (ret < 0) {
		dev_err(&client->dev, "%s:read iic error\n", __func__);
		return rettwo;
	}
	printk("%s,%d:PROC_READ_REGISTER, buf1 = %c\n", __func__, __LINE__, *buf1);


	snprintf(buf, FT_FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);

	ft5x06_fw_LockDownInfo_get_from_boot(client, data->tp_lockdown_info_temp);
	printk("ft5x06_lockdown_show, ft5x46_ctpm_LockDownInfo_get_from_boot, tp_lockdown_info=%s\n", data->tp_lockdown_info_temp);
	if (data->tp_lockdown_info_temp == NULL)
		return  ret;
	return snprintf(buf, FT_LOCKDOWN_LEN - 1, "%s\n", data->tp_lockdown_info_temp);
}


static ssize_t ft5x06_lockdown_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->tp_lockdown_info_temp, buf, size);
	if (data->tp_lockdown_info_temp[size-1] == '\n')
		data->tp_lockdown_info_temp[size-1] = 0;
	return size;
}

static DEVICE_ATTR(tp_lock_down_info, (S_IWUSR|S_IRUGO|S_IWUGO), ft5x06_lockdown_show, ft5x06_lockdown_store);

static ssize_t ft5x06_keypad_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int count;
	char c = data->keypad_mode ? '0' : '1';

	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t ft5x06_keypad_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->keypad_mode = (i == 0);
		return count;
	} else {
		dev_dbg(dev, "keypad_mode write error\n");
		return -EINVAL;
	}
}

static DEVICE_ATTR(keypad_mode, S_IWUSR | S_IRUSR, ft5x06_keypad_mode_show,
		   ft5x06_keypad_mode_store);

static struct attribute *ft5x06_ts_attrs[] = {
	&dev_attr_keypad_mode.attr,
	NULL
};

static const struct attribute_group ft5x06_ts_attr_group = {
	.attrs = ft5x06_ts_attrs,
};

static int ft5x06_proc_init(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;

	int ret = 0;
	char *buf, *path = NULL;
	char *key_disabler_sysfs_node;
	struct proc_dir_entry *proc_entry_tp = NULL;
	struct proc_dir_entry *proc_symlink_tmp  = NULL;

	buf = kzalloc(sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (buf)
		path = "/devices/soc.0/78b8000.i2c/i2c-4/4-0038";

	proc_entry_tp = proc_mkdir("touchpanel", NULL);
	if (proc_entry_tp == NULL) {
		dev_err(&client->dev, "Couldn't create touchpanel dir in procfs\n");
		ret = -ENOMEM;
	}

	key_disabler_sysfs_node = kzalloc(sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (key_disabler_sysfs_node)
		sprintf(key_disabler_sysfs_node, "/sys%s/%s", path, "keypad_mode");
	proc_symlink_tmp = proc_symlink("capacitive_keys_enable",
			proc_entry_tp, key_disabler_sysfs_node);
	if (proc_symlink_tmp == NULL) {
		dev_err(&client->dev, "Couldn't create capacitive_keys_enable symlink\n");
		ret = -ENOMEM;
	}

	kfree(buf);
	kfree(key_disabler_sysfs_node);

	return ret;
}
>>>>>>> p9x

static bool ft5x06_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int ft5x06_debug_data_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_data_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;
	int rc;
<<<<<<< HEAD
	u8 reg = 0;
=======
	u8 reg;
>>>>>>> p9x

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr)) {
		rc = ft5x0x_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, ft5x06_debug_data_get,
			ft5x06_debug_data_set, "0x%02llX\n");

static int ft5x06_debug_addr_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	if (ft5x06_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int ft5x06_debug_addr_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, ft5x06_debug_addr_get,
			ft5x06_debug_addr_set, "0x%02llX\n");

static int ft5x06_debug_suspend_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		ft5x06_ts_suspend(&data->client->dev);
	else
		ft5x06_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_suspend_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, ft5x06_debug_suspend_get,
			ft5x06_debug_suspend_set, "%lld\n");

static int ft5x06_debug_dump_info(struct seq_file *m, void *v)
{
	struct ft5x06_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft5x06_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

#ifdef CONFIG_OF
static int ft5x06_get_dt_coords(struct device *dev, char *name,
				struct ft5x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

<<<<<<< HEAD
=======
	printk("%s\n", __func__);
>>>>>>> p9x
	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = ft5x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"focaltech,no-force-update");
<<<<<<< HEAD
=======

	pdata->fw_auto_update = of_property_read_bool(np,
						"focaltech,fw-auto-update");

>>>>>>> p9x
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
<<<<<<< HEAD
		return pdata->reset_gpio;
=======
		printk("%s, can not get reset_gpio\n", __func__);
	else
		printk("%s, reset_gpio=%d\n", __func__, pdata->reset_gpio);
>>>>>>> p9x

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
<<<<<<< HEAD
		return pdata->irq_gpio;
=======
		printk("%s, can not get irq_gpio\n", __func__);
	else
		printk("%s, irq_gpio=%d\n", __func__, pdata->irq_gpio);

	/* power ldo gpio info*/
	pdata->power_ldo_gpio = of_get_named_gpio_flags(np, "focaltech,power-gpio",
				0, &pdata->power_ldo_gpio_flags);
	if (pdata->power_ldo_gpio < 0)
		printk("%s, can not get power_gpio\n", __func__);
	else
		printk("%s, power_ldo_gpio111111=%d\n", __func__, pdata->power_ldo_gpio);
>>>>>>> p9x

	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
					"focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
						"focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
						"focaltech,ignore-id-check");

<<<<<<< HEAD
	pdata->gesture_support = of_property_read_bool(np,
						"focaltech,gesture-support");

	pdata->resume_in_workqueue = of_property_read_bool(np,
					"focaltech,resume-in-workqueue");

=======
>>>>>>> p9x
	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

<<<<<<< HEAD
=======
	printk("%s done\n", __func__);
>>>>>>> p9x
	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

<<<<<<< HEAD
=======
#if defined(FT_PROC_DEBUG)
#define FTS_PACKET_LENGTH        		128
#define PROC_UPGRADE              		0
#define PROC_READ_REGISTER          	1
#define PROC_WRITE_REGISTER        	2
#define PROC_AUTOCLB                		4
#define PROC_UPGRADE_INFO           	5
#define PROC_WRITE_DATA               	6
#define PROC_READ_DATA                 	7

#define PROC_NAME    "ft5x0x-debug"
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *ft5x0x_proc_entry;



/*interface of write proc*/
static ssize_t ft5x0x_debug_write(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
	struct i2c_client *client = fts_proc_entry_i2c_client;
	unsigned char writebuf[FTS_PACKET_LENGTH];
	int buflen = count;
	int writelen = 0;
	int ret = 0;

	if (copy_from_user(writebuf, (void __user *)buffer, buflen)) {
		dev_err(&client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
		}

	proc_operate_mode = writebuf[0];
	printk("proc_operate_mode = %d\n", proc_operate_mode);
	switch (proc_operate_mode) {
	case PROC_READ_REGISTER:
		printk("%s,%d:PROC_READ_REGISTER\n", __func__, __LINE__);
		writelen = 1;
		ret = ft5x06_i2c_write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		printk("%s,%d:PROC_WRITE_REGISTER\n", __func__, __LINE__);
		writelen = 2;
		ret = ft5x06_i2c_write(client, writebuf + 1, writelen);
		if (ret < 0) {
		dev_err(&client->dev, "%s:write iic error\n", __func__);
		return ret;
		}
		break;
	case PROC_AUTOCLB:
		printk("%s,%d:PROC_AUTOCLB\n", __func__, __LINE__);
		printk("%s: autoclb\n", __func__);
		ft5x06_auto_cal(client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		printk("%s,%d:PROC_READ_DATA,PROC_WRITE_DATA\n", __func__, __LINE__);
		writelen = count - 1;
		ret = ft5x06_i2c_write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		printk("%s,%d:default\n", __func__, __LINE__);
		break;
	}

	return count;
}

/*interface of read proc*/
static ssize_t ft5x0x_debug_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	struct i2c_client *client = fts_proc_entry_i2c_client;
	int ret = 0;
	unsigned char buf[1000];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
	/*after calling ft5x0x_debug_write to upgrade*/
		printk("%s,%d:PROC_UPGRADE\n", __func__, __LINE__);
		regaddr = 0xA6;
		ret = ft5x0x_read_reg(client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = ft5x06_i2c_read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
		return ret;
		}
		printk("%s,%d:PROC_READ_REGISTER, buf = %c\n", __func__, __LINE__, *buf);
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		printk("%s,%d:PROC_READ_DATA\n", __func__, __LINE__);
		readlen = size;
		ret = ft5x06_i2c_read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		}

		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		printk("%s,%d:PROC_WRITE_DATA\n", __func__, __LINE__);
		break;
	default:
		printk("%s,%d:default\n", __func__, __LINE__);
		break;
	}

	memcpy(page, buf, num_read_chars);

	return num_read_chars;
}

static const struct file_operations ft5x0x_debug_ops = {
	.owner = THIS_MODULE,
	.read = ft5x0x_debug_read,
	.write = ft5x0x_debug_write,
};

static int ft5x0x_create_apk_debug_channel(struct i2c_client *client)
{
	ft5x0x_proc_entry = proc_create(PROC_NAME, 0777, NULL, &ft5x0x_debug_ops);

	if (NULL == ft5x0x_proc_entry) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	} else {
		dev_info(&client->dev, "Create proc entry success!\n");
	}
	return 0;
}

static void ft5x0x_release_apk_debug_channel(void)
{
	if (ft5x0x_proc_entry) {
		printk("%s ft5x0x_release_apk_debug_channel jinlin\n", __func__);
		remove_proc_entry(PROC_NAME, NULL);
	}
}
#endif


static unsigned char tp_fw[FT_FW_MAX_SIZE];

static int ft5x06_ctp_upgrade_func(void)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char *fileName = "/mnt/sdcard/CTP_FW.bin";
	char *fileName1 = "/storage/sdcard1/CTP_FW.bin";
	struct inode *inode;
	int fsize = 0;

	if (1) {
		printk(KERN_INFO "[TP] %s: upgrade from file(%s) start!\n", __func__, fileName);
		filp = filp_open(fileName, O_RDONLY, 0);
		if (IS_ERR(filp)) {
			filp = filp_open(fileName1, O_RDONLY, 0);
			if (IS_ERR(filp)) {
				lct_set_ctp_upgrade_status("File no exist");
				printk(KERN_ERR "[TP] %s: open firmware file failed\n", __func__);
				return -EPERM;
			}
		}

		inode = filp->f_dentry->d_inode;
		fsize = inode->i_size;

		if (fsize > sizeof(tp_fw)) {
			printk(KERN_ERR "[TP] %s: firmware size %d is too big\n", __func__, fsize);
			return -EPERM;
		}

		oldfs = get_fs();
		set_fs(get_ds());

		result = filp->f_op->read(filp, tp_fw, fsize, &filp->f_pos);
		if (result < 0) {
			lct_set_ctp_upgrade_status("File read err");
			printk(KERN_ERR "[TP] %s: read firmware file failed\n", __func__);
			return -EPERM;
		}

		set_fs(oldfs);
		filp_close(filp, NULL);

		printk(KERN_INFO "[TP] %s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, tp_fw[0], tp_fw[1], tp_fw[2], tp_fw[3]);

		{
			result = ft5x06_fw_upgrade_by_array_data(&ft5x06_ts->client->dev, tp_fw, fsize, true);
			if (result == 0) {
				lct_set_ctp_upgrade_status("Success");
				return 0;
			} else
				lct_set_ctp_upgrade_status("failed");
		}
	}
	lct_set_ctp_upgrade_status("failed");
	return -EPERM;
}

static void ft5x06_ctp_upgrade_read_ver_func(char *ver)
{
	int cnt = 0;

	if (ver == NULL)
		return;

	ft5x06_update_fw_ver(ft5x06_ts);
	ft5x06_update_fw_vendor_id(ft5x06_ts);

	cnt = sprintf(ver, "vid:0x%03x,fw:0x%03x,ic:%s\n",
		ft5x06_ts->fw_vendor_id, ft5x06_ts->fw_ver[0], "ft5x06");
	return ;
}


extern unsigned char ft5x06_fw_LockDownInfo_get_from_boot(struct i2c_client *client, char *pProjectCode);

>>>>>>> p9x
static int ft5x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata;
<<<<<<< HEAD
	struct ft5x06_gesture_platform_data *gesture_pdata;
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
	struct dentry *temp;
	u8 reg_value = 0;
	u8 reg_addr;
	int err, len, retval, attr_count;
=======
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len;


#ifdef SUPPORT_READ_TP_VERSION
	char fw_version[64];
#endif
	printk("%s, of_node=%s, is_tp_driver_loaded=%d\n", __func__,
		client->dev.of_node->name, is_tp_driver_loaded);

	if (is_tp_driver_loaded == 1) {
		printk("%s, other driver has been loaded\n", __func__);
		return ENODEV;
	}

>>>>>>> p9x

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_platform_data), GFP_KERNEL);
<<<<<<< HEAD
		if (!pdata)
			return -ENOMEM;
=======
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
>>>>>>> p9x

		err = ft5x06_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_data), GFP_KERNEL);
<<<<<<< HEAD
	if (!data)
		return -ENOMEM;
=======
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}
>>>>>>> p9x

	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}

		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,
				data->tch_data_len, GFP_KERNEL);
<<<<<<< HEAD
	if (!data->tch_data)
		return -ENOMEM;
=======
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}
>>>>>>> p9x

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

<<<<<<< HEAD
	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;
=======
	fts_proc_entry_i2c_client = client;
	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;
	ft5x06_ts = data;
>>>>>>> p9x

	input_dev->name = "ft5x06_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);
<<<<<<< HEAD
=======
#if defined(CONFIG_TOUCHSCREEN_GESTURE)
#if defined(CONFIG_CT820_COMMON)
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE);
#else
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
#endif
#endif
>>>>>>> p9x

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
<<<<<<< HEAD
		input_free_device(input_dev);
		return err;
=======
		goto free_inputdev;
>>>>>>> p9x
	}

	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	} else {
		err = ft5x06_power_init(data, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}

<<<<<<< HEAD
	err = ft5x06_ts_pinctrl_init(data);
	if (!err && data->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
=======
		err = ft5x06_ts_pinctrl_init(data);
	if (!err && data->ts_pinctrl) {
>>>>>>> p9x
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_active);
		if (err < 0) {
			dev_err(&client->dev,
				"failed to select pin to active state");
<<<<<<< HEAD
		}
	}

	err = ft5x06_gpio_configure(data, true);
	if (err < 0) {
		dev_err(&client->dev,
			"Failed to configure the gpios\n");
		goto err_gpio_req;
	}

=======
			goto pinctrl_deinit;
		}
	} else {
		goto pwr_off;

	}




	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "ft5x06_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto err_gpio_req;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "ft5x06_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}



>>>>>>> p9x
	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	/* check the controller id */
	reg_addr = FT_REG_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
<<<<<<< HEAD
		goto free_gpio;
=======
		goto free_reset_gpio;
>>>>>>> p9x
	}

	dev_info(&client->dev, "Device ID = 0x%x\n", reg_value);

	if ((pdata->family_id != reg_value) && (!pdata->ignore_id_check)) {
		dev_err(&client->dev, "%s:Unsupported controller\n", __func__);
<<<<<<< HEAD
		goto free_gpio;
=======
		goto free_reset_gpio;
>>>>>>> p9x
	}

	data->family_id = pdata->family_id;

	err = request_threaded_irq(client->irq, NULL,
				ft5x06_ts_interrupt,
<<<<<<< HEAD
	/*
	* the interrupt trigger mode will be set in Device Tree with property
	* "interrupts", so here we just need to set the flag IRQF_ONESHOT
	*/
				IRQF_ONESHOT,
				client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_gpio;
	}

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support) {
		device_init_wakeup(&client->dev, 1);
		gesture_pdata = devm_kzalloc(&client->dev,
				sizeof(struct ft5x06_gesture_platform_data),
				GFP_KERNEL);
		if (!gesture_pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			goto free_gesture_dev;
		}
		data->gesture_pdata = gesture_pdata;
		gesture_pdata->data = data;

		gesture_pdata->gesture_class =
					class_create(THIS_MODULE, "gesture");
		if (IS_ERR(gesture_pdata->gesture_class)) {
			err = PTR_ERR(gesture_pdata->gesture_class);
			dev_err(&client->dev, "Failed to create class.\n");
			goto free_gesture_pdata;
		}

		gesture_pdata->dev = device_create(gesture_pdata->gesture_class,
				NULL, 0, NULL, "gesture_ft5x06");
		if (IS_ERR(gesture_pdata->dev)) {
			err = PTR_ERR(gesture_pdata->dev);
			dev_err(&client->dev, "Failed to create device.\n");
			goto free_gesture_class;
		}

		dev_set_drvdata(gesture_pdata->dev, data);
		err = device_create_file(gesture_pdata->dev,
					&dev_attr_enable);
		if (err) {
			dev_err(gesture_pdata->dev,
					"sys file creation failed\n");
			goto free_gesture_dev;
		}
		err = device_create_file(gesture_pdata->dev,
					&dev_attr_pocket);
		if (err) {
			dev_err(gesture_pdata->dev,
					"sys file creation failed\n");
			goto free_enable_sys;
		}
	}
=======
				pdata->irqflags | IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
				client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_reset_gpio;
	}

#ifdef CONFIG_WAKE_GESTURES
	ft5x06_ts = data;
	device_init_wakeup(&client->dev, 1);
#endif
>>>>>>> p9x

	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
<<<<<<< HEAD
		goto free_pocket_sys;
=======
		goto irq_free;
>>>>>>> p9x
	}

	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_fw_name_sys;
	}

<<<<<<< HEAD
=======
	err = device_create_file(&client->dev, &dev_attr_tp_lock_down_info);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_tp_lock_down_info_sys;
	}

>>>>>>> p9x
	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_update_fw_sys;
	}

<<<<<<< HEAD
=======
	 err = sysfs_create_group(&client->dev.kobj, &ft5x06_ts_attr_group);
	 if (err) {
		dev_err(&client->dev, "Failure %d creating sysfs group\n",err);
		goto free_reset_gpio;
    }

    ft5x06_proc_init(data);

>>>>>>> p9x
	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
		goto free_force_update_fw_sys;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	data->ts_info = devm_kzalloc(&client->dev,
				FT_INFO_MAX_LEN, GFP_KERNEL);
<<<<<<< HEAD
	if (!data->ts_info)
		goto free_debug_dir;
=======
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_debug_dir;
	}

	printk("%s, fw_auto_update=%d, no_force_update=%d\n", __func__,
		data->pdata->fw_auto_update, data->pdata->no_force_update);
	msleep(200);
	ft5x06_update_fw_ver(data);
	ft5x06_update_fw_vendor_id(data);
	uc_tp_vendor_id = data->fw_vendor_id;
	printk("upgrade,fw_vendor_id=0x%02x\n",  data->fw_vendor_id);
	if ((uc_tp_vendor_id != 0x3b) && (uc_tp_vendor_id != 0x53) && (uc_tp_vendor_id != 0x51)) {
		uc_tp_vendor_id = ft5x06_fw_Vid_get_from_boot(client);
		printk("get_Vid_from_boot, fw_vendor_id=0x%02x\n",  uc_tp_vendor_id);
	}
	if (data->pdata->fw_auto_update) {
		mutex_lock(&data->input_dev->mutex);
		if (!data->loading_fw) {
			data->loading_fw = true;
			if (uc_tp_vendor_id == 0x3b)
				memcpy(firmware_data, firmware_data_biel, sizeof(firmware_data_biel));
			else if (uc_tp_vendor_id == 0x53)
				memcpy(firmware_data, firmware_data_mutton, sizeof(firmware_data_mutton));
			else if ((uc_tp_vendor_id == 0x51) || (uc_tp_vendor_id == 0x79))
				memcpy(firmware_data, firmware_data_ofilm, sizeof(firmware_data_ofilm));
			else
				printk("[FTS] FW unmatched,stop upgrade\n");
			ft5x06_fw_upgrade_by_array_data(&client->dev, firmware_data, sizeof(firmware_data), !data->pdata->no_force_update);
			data->loading_fw = false;
		}
		mutex_unlock(&data->input_dev->mutex);
	}

	/*
	ft5x06_fw_LockDownInfo_get_from_boot(client, tp_lockdown_info);
	printk("tpd_probe, ft5x46_ctpm_LockDownInfo_get_from_boot, tp_lockdown_info=%s\n", tp_lockdown_info);
	*/
>>>>>>> p9x

	/*get some register information */
	reg_addr = FT_REG_POINT_RATE;
	ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	reg_addr = FT_REG_THGROUP;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

<<<<<<< HEAD
	dev_dbg(&client->dev, "touch threshold = %d\n", reg_value * 4);

	/*creation touch panel info kobj*/
	data->ts_info_kobj = kobject_create_and_add(FT_TS_INFO_SYSFS_DIR_NAME,
					kernel_kobj);
	if (!data->ts_info_kobj) {
		dev_err(&client->dev, "kobject creation failed.\n");
	} else {
		err = sysfs_create_file(data->ts_info_kobj, &ts_info_attr.attr);
		if (err) {
			kobject_put(data->ts_info_kobj);
			dev_err(&client->dev, "sysfs creation failed.\n");
		} else {
			ts_info_buff = devm_kzalloc(&client->dev,
						 FT_INFO_MAX_LEN, GFP_KERNEL);
			if (!ts_info_buff)
				goto free_debug_dir;
		}
	}

	/*Initialize secure touch */
	ft5x06_secure_touch_init(data);
	ft5x06_secure_touch_stop(data, true);
	mutex_init(&(data->ft_clk_io_ctrl_mutex));

	/* Creation of secure touch sysfs files */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&data->input_dev->dev.kobj,
						&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&client->dev,
				"%s: Failed to create sysfs attributes\n",
							__func__);
			goto free_secure_touch_sysfs;
		}
	}

	ft5x06_update_fw_ver(data);
	ft5x06_update_fw_vendor_id(data);

	FT_STORE_TS_DBG_INFO(data->ts_info, data->family_id, data->pdata->name,
=======
	dev_info(&client->dev, "touch threshold = %d\n", reg_value * 4);
	ft5x06_update_fw_ver(data);
	ft5x06_update_fw_vendor_id(data);

	printk("%s, Firmware version = 0x%02x.%d.%d, fw_vendor_id=0x%02x\n", __func__,
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2], data->fw_vendor_id);
#ifdef SUPPORT_READ_TP_VERSION
	memset(fw_version, 0, sizeof(fw_version));
	sprintf(fw_version, "[FW]0x%x, [IC]FT5346", data->fw_ver[0]);
	init_tp_fm_info(0, fw_version, "FocalTech");
#endif

	lct_ctp_upgrade_int(ft5x06_ctp_upgrade_func, ft5x06_ctp_upgrade_read_ver_func);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
>>>>>>> p9x
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);
<<<<<<< HEAD
	FT_STORE_TS_INFO(ts_info_buff, data->family_id, data->fw_ver[0],
			 data->fw_ver[1], data->fw_ver[2]);
#if defined(CONFIG_FB)
	INIT_WORK(&data->fb_notify_work, fb_notify_resume_work);
=======

#if defined(CONFIG_FB)
>>>>>>> p9x
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = ft5x06_ts_early_suspend;
	data->early_suspend.resume = ft5x06_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
<<<<<<< HEAD
	return 0;

free_secure_touch_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&data->input_dev->dev.kobj,
					&attrs[attr_count].attr);
	}
=======

#if defined(FTS_SCAP_TEST)
	mutex_init(&data->selftest_lock);
	lct_ctp_selftest_int(ft5x06_self_test);
#endif

#if defined(FT_PROC_DEBUG)
		if (ft5x0x_create_apk_debug_channel(client) < 0)
			ft5x0x_release_apk_debug_channel();
#endif

#if defined(CONFIG_TOUCHSCREEN_GESTURE)
	ctp_gesture_switch_init(lct_ctp_gesture_mode_switch);
#endif

#if defined(CONFIG_TOUCHSCREEN_COVER)
	ctp_cover_switch_init(lct_ctp_cover_state_switch);
#endif


	is_tp_driver_loaded = 1;
	printk("%s done\n", __func__);
	return 0;

>>>>>>> p9x
free_debug_dir:
	debugfs_remove_recursive(data->dir);
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
<<<<<<< HEAD
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
free_pocket_sys:
	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support)
		device_remove_file(&client->dev, &dev_attr_pocket);
free_enable_sys:
	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support)
		device_remove_file(&client->dev, &dev_attr_enable);
free_gesture_dev:
	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support)
		device_destroy(gesture_pdata->gesture_class, 0);
free_gesture_class:
	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support)
		class_destroy(gesture_pdata->gesture_class);
free_gesture_pdata:
	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support) {
		devm_kfree(&client->dev, gesture_pdata);
		data->gesture_pdata = NULL;
	}

free_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
err_gpio_req:
=======
free_tp_lock_down_info_sys:
	device_remove_file(&client->dev, &dev_attr_tp_lock_down_info);

free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
irq_free:
	free_irq(client->irq, data);
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
err_gpio_req:
pinctrl_deinit:
>>>>>>> p9x
	if (data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
			devm_pinctrl_put(data->ts_pinctrl);
			data->ts_pinctrl = NULL;
		} else {
			err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_release);
			if (err)
				pr_err("failed to select relase pinctrl state\n");
		}
	}
<<<<<<< HEAD
=======
pwr_off:
>>>>>>> p9x
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft5x06_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft5x06_power_init(data, false);
unreg_inputdev:
	input_unregister_device(input_dev);
<<<<<<< HEAD
=======
	input_dev = NULL;
free_inputdev:
	input_free_device(input_dev);
	printk("%s error, err=%d\n", __func__, err);
>>>>>>> p9x
	return err;
}

static int ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
<<<<<<< HEAD
	int retval, attr_count;

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support) {
		device_init_wakeup(&client->dev, 0);
		device_remove_file(&client->dev, &dev_attr_pocket);
		device_remove_file(&client->dev, &dev_attr_enable);
		device_destroy(data->gesture_pdata->gesture_class, 0);
		class_destroy(data->gesture_pdata->gesture_class);
		devm_kfree(&client->dev, data->gesture_pdata);
		data->gesture_pdata = NULL;
	}

=======
	int retval;

#if defined(FTS_SCAP_TEST)
	mutex_destroy(&data->selftest_lock);
#endif
>>>>>>> p9x
	debugfs_remove_recursive(data->dir);
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
	device_remove_file(&client->dev, &dev_attr_update_fw);
	device_remove_file(&client->dev, &dev_attr_fw_name);
<<<<<<< HEAD

=======
	device_remove_file(&client->dev, &dev_attr_tp_lock_down_info);
>>>>>>> p9x
#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	if (data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
			devm_pinctrl_put(data->ts_pinctrl);
			data->ts_pinctrl = NULL;
		} else {
			retval = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_release);
			if (retval < 0)
				pr_err("failed to select release pinctrl state\n");
		}
	}

<<<<<<< HEAD
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&data->input_dev->dev.kobj,
					&attrs[attr_count].attr);
	}

=======
>>>>>>> p9x
	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		ft5x06_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		ft5x06_power_init(data, false);

<<<<<<< HEAD
	input_unregister_device(data->input_dev);
	kobject_put(data->ts_info_kobj);
=======
#if defined(FT_PROC_DEBUG)
		ft5x0x_release_apk_debug_channel();
#endif

	input_unregister_device(data->input_dev);
	sysfs_remove_group(&client->dev.kobj, &ft5x06_ts_attr_group);

>>>>>>> p9x
	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{ .compatible = "focaltech,5x06",},
	{ },
};
#else
#define ft5x06_match_table NULL
#endif

static struct i2c_driver ft5x06_ts_driver = {
	.probe = ft5x06_ts_probe,
	.remove = ft5x06_ts_remove,
	.driver = {
		   .name = "ft5x06_ts",
		   .owner = THIS_MODULE,
		.of_match_table = ft5x06_match_table,
#ifdef CONFIG_PM
		   .pm = &ft5x06_ts_pm_ops,
#endif
		   },
	.id_table = ft5x06_ts_id,
};

static int __init ft5x06_ts_init(void)
{
	return i2c_add_driver(&ft5x06_ts_driver);
}
module_init(ft5x06_ts_init);

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}
module_exit(ft5x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x06 TouchScreen driver");
MODULE_LICENSE("GPL v2");
