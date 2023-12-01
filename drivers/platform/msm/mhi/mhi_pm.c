<<<<<<< HEAD
/* Copyright (c) 2014-2017, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
>>>>>>> p9x
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/msm_mhi.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/pm_runtime.h>

#include "mhi_sys.h"
#include "mhi.h"
#include "mhi_hwio.h"
<<<<<<< HEAD
#include "mhi_bhi.h"

static const char *const mhi_dev_ctrl_str[MHI_DEV_CTRL_MAXCMD] = {
	[MHI_DEV_CTRL_INIT] = "INIT",
	[MHI_DEV_CTRL_DE_INIT] = "DE-INIT",
	[MHI_DEV_CTRL_SUSPEND] = "SUSPEND",
	[MHI_DEV_CTRL_RESUME] = "RESUME",
	[MHI_DEV_CTRL_POWER_OFF] = "OFF",
	[MHI_DEV_CTRL_POWER_ON] = "ON",
	[MHI_DEV_CTRL_TRIGGER_RDDM] = "TRIGGER RDDM",
	[MHI_DEV_CTRL_RDDM] = "RDDM",
	[MHI_DEV_CTRL_RDDM_KERNEL_PANIC] = "RDDM IN PANIC",
	[MHI_DEV_CTRL_NOTIFY_LINK_ERROR] = "LD",
};

#define TO_MHI_DEV_CTRL_STR(cmd) ((cmd >= MHI_DEV_CTRL_MAXCMD) ? "INVALID" : \
				  mhi_dev_ctrl_str[cmd])

/* Write only sysfs attributes */
static DEVICE_ATTR(MHI_M0, S_IWUSR, NULL, sysfs_init_m0);
static DEVICE_ATTR(MHI_M3, S_IWUSR, NULL, sysfs_init_m3);
=======

/* Write only sysfs attributes */
static DEVICE_ATTR(MHI_M3, S_IWUSR, NULL, sysfs_init_m3);
static DEVICE_ATTR(MHI_M0, S_IWUSR, NULL, sysfs_init_m0);
static DEVICE_ATTR(MHI_RESET, S_IWUSR, NULL, sysfs_init_mhi_reset);
>>>>>>> p9x

/* Read only sysfs attributes */

static struct attribute *mhi_attributes[] = {
<<<<<<< HEAD
	&dev_attr_MHI_M0.attr,
	&dev_attr_MHI_M3.attr,
=======
	&dev_attr_MHI_M3.attr,
	&dev_attr_MHI_M0.attr,
	&dev_attr_MHI_RESET.attr,
>>>>>>> p9x
	NULL,
};

static struct attribute_group mhi_attribute_group = {
	.attrs = mhi_attributes,
};

<<<<<<< HEAD
int mhi_pci_suspend(struct device *dev)
{
	int r = 0;
	struct mhi_device_ctxt *mhi_dev_ctxt = dev_get_drvdata(dev);

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Entered\n");
	/* if rpm status still active then force suspend */
	if (!pm_runtime_status_suspended(dev)) {
		r = mhi_runtime_suspend(dev);
		if (r)
			return r;
	}

	pm_runtime_set_suspended(dev);
	pm_runtime_disable(dev);

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Exit\n");
	return r;
}

static int mhi_pm_initiate_m3(struct mhi_device_ctxt *mhi_dev_ctxt,
			      bool force_m3)
{
	int r = 0;
	enum MHI_PM_STATE new_state;

	read_lock_bh(&mhi_dev_ctxt->pm_xfer_lock);
	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
		"Entered with State:0x%x %s\n",
		mhi_dev_ctxt->mhi_pm_state,
		TO_MHI_STATE_STR(mhi_dev_ctxt->mhi_state));

	/* Link is already disabled */
	if (mhi_dev_ctxt->mhi_pm_state == MHI_PM_DISABLE ||
	   mhi_dev_ctxt->mhi_pm_state == MHI_PM_M3) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Already in M3 State\n");
		read_unlock_bh(&mhi_dev_ctxt->pm_xfer_lock);
		return 0;
	}

	if (unlikely(atomic_read(&mhi_dev_ctxt->counters.device_wake) &&
		     force_m3 == false)) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Busy, Aborting M3\n");
		read_unlock_bh(&mhi_dev_ctxt->pm_xfer_lock);
		return -EBUSY;
	}

	if (unlikely(!MHI_REG_ACCESS_VALID(mhi_dev_ctxt->mhi_pm_state))) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Error, no register access, PM_STATE:0x%x\n",
			mhi_dev_ctxt->mhi_pm_state);
		read_unlock_bh(&mhi_dev_ctxt->pm_xfer_lock);
		return -EIO;
	}

	mhi_dev_ctxt->assert_wake(mhi_dev_ctxt, false);
	read_unlock_bh(&mhi_dev_ctxt->pm_xfer_lock);
	r = wait_event_timeout(*mhi_dev_ctxt->mhi_ev_wq.m0_event,
		mhi_dev_ctxt->mhi_state == MHI_STATE_M0 ||
		mhi_dev_ctxt->mhi_state == MHI_STATE_M1 ||
		mhi_dev_ctxt->mhi_pm_state == MHI_PM_LD_ERR_FATAL_DETECT,
		msecs_to_jiffies(MHI_MAX_STATE_TRANSITION_TIMEOUT));
	if (!r || mhi_dev_ctxt->mhi_pm_state == MHI_PM_LD_ERR_FATAL_DETECT) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Failed to get M0||M1 event or LD pm_state:0x%x state:%s\n",
			mhi_dev_ctxt->mhi_pm_state,
			TO_MHI_STATE_STR(mhi_dev_ctxt->mhi_state));
		return -EIO;
	}

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Allowing M3 State\n");
	write_lock_irq(&mhi_dev_ctxt->pm_xfer_lock);
	mhi_dev_ctxt->deassert_wake(mhi_dev_ctxt);
	new_state = mhi_tryset_pm_state(mhi_dev_ctxt, MHI_PM_M3_ENTER);
	if (unlikely(new_state != MHI_PM_M3_ENTER)) {
		write_unlock_irq(&mhi_dev_ctxt->pm_xfer_lock);
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Error setting PM_STATE from 0x%x to 0x%x\n",
			new_state, MHI_PM_M3_ENTER);
		return -EIO;
	}
	mhi_set_m_state(mhi_dev_ctxt, MHI_STATE_M3);
	write_unlock_irq(&mhi_dev_ctxt->pm_xfer_lock);
	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Waiting for M3 completion.\n");
	r = wait_event_timeout(*mhi_dev_ctxt->mhi_ev_wq.m3_event,
		mhi_dev_ctxt->mhi_state == MHI_STATE_M3 ||
		mhi_dev_ctxt->mhi_pm_state == MHI_PM_LD_ERR_FATAL_DETECT,
		msecs_to_jiffies(MHI_MAX_STATE_TRANSITION_TIMEOUT));
	if (!r || mhi_dev_ctxt->mhi_pm_state == MHI_PM_LD_ERR_FATAL_DETECT) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Failed to get M3 event, timeout, current state:%s\n",
			TO_MHI_STATE_STR(mhi_dev_ctxt->mhi_state));
		return -EIO;
	}

	return 0;
}

static int mhi_pm_initiate_m0(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	int r;
	enum MHI_PM_STATE cur_state;

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
		"Entered with State:0x%x %s\n",
		mhi_dev_ctxt->mhi_pm_state,
		TO_MHI_STATE_STR(mhi_dev_ctxt->mhi_state));

	write_lock_irq(&mhi_dev_ctxt->pm_xfer_lock);
	cur_state = mhi_tryset_pm_state(mhi_dev_ctxt, MHI_PM_M3_EXIT);
	if (unlikely(cur_state != MHI_PM_M3_EXIT)) {
		write_unlock_irq(&mhi_dev_ctxt->pm_xfer_lock);
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Error setting PM_STATE from 0x%x to 0x%x\n",
			cur_state, MHI_PM_M3_EXIT);
		return -EAGAIN;
	}

	/* Set and wait for M0 Event */
	mhi_set_m_state(mhi_dev_ctxt, MHI_STATE_M0);
	write_unlock_irq(&mhi_dev_ctxt->pm_xfer_lock);
	r = wait_event_timeout(*mhi_dev_ctxt->mhi_ev_wq.m0_event,
		mhi_dev_ctxt->mhi_state == MHI_STATE_M0 ||
		mhi_dev_ctxt->mhi_state == MHI_STATE_M1 ||
		mhi_dev_ctxt->mhi_pm_state == MHI_PM_LD_ERR_FATAL_DETECT,
		msecs_to_jiffies(MHI_MAX_STATE_TRANSITION_TIMEOUT));
	if (!r || mhi_dev_ctxt->mhi_pm_state == MHI_PM_LD_ERR_FATAL_DETECT) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Failed to get M0 event, timeout or LD\n");
		r = -EIO;
	} else
		r = 0;

=======
int mhi_pci_suspend(struct pci_dev *pcie_dev, pm_message_t state)
{
	int r = 0;
	struct mhi_device_ctxt *mhi_dev_ctxt = pcie_dev->dev.platform_data;

	if (NULL == mhi_dev_ctxt)
		return -EINVAL;
	mhi_log(MHI_MSG_INFO, "Entered, sys state %d, MHI state %d\n",
			state.event, mhi_dev_ctxt->mhi_state);
	atomic_set(&mhi_dev_ctxt->flags.pending_resume, 1);

	r = mhi_initiate_m3(mhi_dev_ctxt);

	if (!r)
		return r;

	atomic_set(&mhi_dev_ctxt->flags.pending_resume, 0);
	mhi_log(MHI_MSG_INFO, "Exited, ret %d\n", r);
>>>>>>> p9x
	return r;
}

int mhi_runtime_suspend(struct device *dev)
{
	int r = 0;
<<<<<<< HEAD
	struct mhi_device_ctxt *mhi_dev_ctxt = dev_get_drvdata(dev);

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Enter\n");

	mutex_lock(&mhi_dev_ctxt->pm_lock);
	r = mhi_pm_initiate_m3(mhi_dev_ctxt, false);
	if (r) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "abort due to ret:%d\n", r);
		mutex_unlock(&mhi_dev_ctxt->pm_lock);
		return r;
	}
	r = mhi_turn_off_pcie_link(mhi_dev_ctxt, true);
	if (r) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Failed to Turn off link ret:%d\n", r);
	}

	mutex_unlock(&mhi_dev_ctxt->pm_lock);
	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Exited with ret:%d\n", r);

	return r;
}

int mhi_runtime_idle(struct device *dev)
{
	struct mhi_device_ctxt *mhi_dev_ctxt = dev_get_drvdata(dev);

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
		"Entered returning -EBUSY\n");

	/*
	 * RPM framework during runtime resume always calls
	 * rpm_idle to see if device ready to suspend.
	 * If dev.power usage_count count is 0, rpm fw will call
	 * rpm_idle cb to see if device is ready to suspend.
	 * if cb return 0, or cb not defined the framework will
	 * assume device driver is ready to suspend;
	 * therefore, fw will schedule runtime suspend.
	 * In MHI power management, MHI host shall go to
	 * runtime suspend only after entering MHI State M2, even if
	 * usage count is 0.  Return -EBUSY to disable automatic suspend.
	 */
	return -EBUSY;
}

int mhi_runtime_resume(struct device *dev)
{
	int r = 0;
	struct mhi_device_ctxt *mhi_dev_ctxt = dev_get_drvdata(dev);

	mutex_lock(&mhi_dev_ctxt->pm_lock);
	read_lock_bh(&mhi_dev_ctxt->pm_xfer_lock);
	BUG_ON(mhi_dev_ctxt->mhi_pm_state != MHI_PM_M3);
	read_unlock_bh(&mhi_dev_ctxt->pm_xfer_lock);

	/* turn on link */
	r = mhi_turn_on_pcie_link(mhi_dev_ctxt);
	if (r)
		goto rpm_resume_exit;

	r = mhi_pm_initiate_m0(mhi_dev_ctxt);
rpm_resume_exit:
	mutex_unlock(&mhi_dev_ctxt->pm_lock);
	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Exited with :%d\n", r);
	return r;
}

int mhi_pci_resume(struct device *dev)
{
	int r = 0;
	struct mhi_device_ctxt *mhi_dev_ctxt = dev_get_drvdata(dev);

	r = mhi_runtime_resume(dev);
	if (r) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_CRITICAL,
			"Failed to resume link\n");
	} else {
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
	}

	return r;
}

static int mhi_pm_slave_mode_power_on(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	int ret_val;
	u32 timeout = mhi_dev_ctxt->poll_reset_timeout_ms;

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Entered\n");
	mutex_lock(&mhi_dev_ctxt->pm_lock);
	write_lock_irq(&mhi_dev_ctxt->pm_xfer_lock);
	mhi_dev_ctxt->mhi_pm_state = MHI_PM_POR;
	ret_val = set_mhi_base_state(mhi_dev_ctxt);
	write_unlock_irq(&mhi_dev_ctxt->pm_xfer_lock);

	if (ret_val) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Error Setting MHI Base State %d\n", ret_val);
		goto unlock_pm_lock;
	}

	if (mhi_dev_ctxt->base_state != STATE_TRANSITION_BHI) {
		ret_val = -EIO;
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Invalid Base State, cur_state:%s\n",
			state_transition_str(mhi_dev_ctxt->base_state));
		goto unlock_pm_lock;
	}

	reinit_completion(&mhi_dev_ctxt->cmd_complete);
	init_mhi_base_state(mhi_dev_ctxt);

	/*
	 * Keep wake in Active until AMSS, @ AMSS we will
	 * decrement counts
	 */
	read_lock_irq(&mhi_dev_ctxt->pm_xfer_lock);
	mhi_dev_ctxt->assert_wake(mhi_dev_ctxt, false);
	read_unlock_irq(&mhi_dev_ctxt->pm_xfer_lock);

	wait_for_completion_timeout(&mhi_dev_ctxt->cmd_complete,
				    msecs_to_jiffies(timeout));
	if (mhi_dev_ctxt->dev_exec_env != MHI_EXEC_ENV_AMSS)
		ret_val = -EIO;
	else
		ret_val = 0;

	if (ret_val) {
		read_lock_irq(&mhi_dev_ctxt->pm_xfer_lock);
		mhi_dev_ctxt->deassert_wake(mhi_dev_ctxt);
		read_unlock_irq(&mhi_dev_ctxt->pm_xfer_lock);
	}

unlock_pm_lock:

	/* wait for firmware download to complete */
	flush_work(&mhi_dev_ctxt->bhi_ctxt.fw_load_work);

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Exit with ret:%d\n", ret_val);
	mutex_unlock(&mhi_dev_ctxt->pm_lock);
	return ret_val;
}

static void mhi_pm_slave_mode_power_off(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
		"Entered with pm_state:0x%x MHI_STATE:%s\n",
		mhi_dev_ctxt->mhi_pm_state,
		TO_MHI_STATE_STR(mhi_dev_ctxt->mhi_state));

	if (mhi_dev_ctxt->mhi_pm_state == MHI_PM_DISABLE) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"MHI already in disabled state\n");
		return;
	}
	process_disable_transition(MHI_PM_SHUTDOWN_PROCESS, mhi_dev_ctxt);
}

static int mhi_pm_slave_mode_suspend(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	int r;

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Entered\n");
	mutex_lock(&mhi_dev_ctxt->pm_lock);

	r = mhi_pm_initiate_m3(mhi_dev_ctxt, false);
	if (r)
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "abort due to ret:%d\n", r);
	mutex_unlock(&mhi_dev_ctxt->pm_lock);

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Exit with ret:%d\n", r);

	return r;
}

static int mhi_pm_slave_mode_resume(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	int r;

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Entered\n");
	mutex_lock(&mhi_dev_ctxt->pm_lock);

	r = mhi_pm_initiate_m0(mhi_dev_ctxt);
	if (r)
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"M3 exit failed ret:%d\n", r);
	mutex_unlock(&mhi_dev_ctxt->pm_lock);
	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Exit with ret:%d\n", r);

=======
	struct mhi_device_ctxt *mhi_dev_ctxt = dev->platform_data;
	mhi_log(MHI_MSG_INFO, "Runtime Suspend - Entered\n");
	r = mhi_initiate_m3(mhi_dev_ctxt);
	pm_runtime_mark_last_busy(dev);
	mhi_log(MHI_MSG_INFO, "Runtime Suspend - Exited\n");
	return r;
}

int mhi_runtime_resume(struct device *dev)
{
	int r = 0;
	struct mhi_device_ctxt *mhi_dev_ctxt = dev->platform_data;
	mhi_log(MHI_MSG_INFO, "Runtime Resume - Entered\n");
	r = mhi_initiate_m0(mhi_dev_ctxt);
	pm_runtime_mark_last_busy(dev);
	mhi_log(MHI_MSG_INFO, "Runtime Resume - Exited\n");
	return r;
}

int mhi_pci_resume(struct pci_dev *pcie_dev)
{
	int r = 0;
	struct mhi_device_ctxt *mhi_dev_ctxt = pcie_dev->dev.platform_data;
	r = mhi_initiate_m0(mhi_dev_ctxt);
	if (r)
		goto exit;
	r = wait_event_interruptible_timeout(*mhi_dev_ctxt->mhi_ev_wq.m0_event,
			mhi_dev_ctxt->mhi_state == MHI_STATE_M0 ||
			mhi_dev_ctxt->mhi_state == MHI_STATE_M1,
			msecs_to_jiffies(MHI_MAX_SUSPEND_TIMEOUT));
	switch (r) {
	case 0:
		mhi_log(MHI_MSG_CRITICAL,
			"Timeout: No M0 event after %d ms\n",
			MHI_MAX_SUSPEND_TIMEOUT);
		mhi_dev_ctxt->counters.m0_event_timeouts++;
		r = -ETIME;
		break;
	case -ERESTARTSYS:
		mhi_log(MHI_MSG_CRITICAL,
			"Going Down...\n");
		break;
	default:
		mhi_log(MHI_MSG_INFO,
			"Wait complete state: %d\n", mhi_dev_ctxt->mhi_state);
		r = 0;
	}
exit:
	atomic_set(&mhi_dev_ctxt->flags.pending_resume, 0);
>>>>>>> p9x
	return r;
}

int mhi_init_pm_sysfs(struct device *dev)
{
	return sysfs_create_group(&dev->kobj, &mhi_attribute_group);
}

void mhi_rem_pm_sysfs(struct device *dev)
{
	return sysfs_remove_group(&dev->kobj, &mhi_attribute_group);
}

<<<<<<< HEAD
ssize_t sysfs_init_m0(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct mhi_device_ctxt *mhi_dev_ctxt = dev_get_drvdata(dev);

	pm_runtime_get(&mhi_dev_ctxt->pcie_device->dev);
	pm_runtime_mark_last_busy(&mhi_dev_ctxt->pcie_device->dev);
	pm_runtime_put_noidle(&mhi_dev_ctxt->pcie_device->dev);

	return count;
}

ssize_t sysfs_init_m3(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct mhi_device_ctxt *mhi_dev_ctxt = dev_get_drvdata(dev);

	if (atomic_read(&mhi_dev_ctxt->counters.device_wake) == 0) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Schedule RPM suspend");
		pm_runtime_mark_last_busy(&mhi_dev_ctxt->
					  pcie_device->dev);
		pm_request_autosuspend(&mhi_dev_ctxt->
				       pcie_device->dev);
	}
=======
ssize_t sysfs_init_m3(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int r = 0;
	struct mhi_device_ctxt *mhi_dev_ctxt =
		&mhi_devices.device_list[0].mhi_ctxt;
	r = mhi_initiate_m3(mhi_dev_ctxt);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL,
				"Failed to suspend %d\n", r);
		return r;
	}
	if (MHI_STATUS_SUCCESS != mhi_turn_off_pcie_link(mhi_dev_ctxt))
		mhi_log(MHI_MSG_CRITICAL,
				"Failed to turn off link\n");

	return count;
}
ssize_t sysfs_init_mhi_reset(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct mhi_device_ctxt *mhi_dev_ctxt =
		&mhi_devices.device_list[0].mhi_ctxt;
	enum MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_log(MHI_MSG_INFO, "Triggering MHI Reset.\n");
	ret_val = mhi_trigger_reset(mhi_dev_ctxt);
	if (ret_val != MHI_STATUS_SUCCESS)
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to trigger MHI RESET ret %d\n",
			ret_val);
	else
		mhi_log(MHI_MSG_INFO, "Triggered! MHI RESET\n");
	return count;
}
ssize_t sysfs_init_m0(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct mhi_device_ctxt *mhi_dev_ctxt =
		&mhi_devices.device_list[0].mhi_ctxt;
	if (MHI_STATUS_SUCCESS != mhi_turn_on_pcie_link(mhi_dev_ctxt)) {
		mhi_log(MHI_MSG_CRITICAL,
				"Failed to resume link\n");
		return count;
	}
	mhi_initiate_m0(mhi_dev_ctxt);
	mhi_log(MHI_MSG_CRITICAL,
			"Current mhi_state = 0x%x\n",
			mhi_dev_ctxt->mhi_state);
>>>>>>> p9x

	return count;
}

<<<<<<< HEAD
int mhi_turn_off_pcie_link(struct mhi_device_ctxt *mhi_dev_ctxt, bool graceful)
{
	struct pci_dev *pcie_dev;
	int r = 0;

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Entered...\n");
	pcie_dev = mhi_dev_ctxt->pcie_device;

	if (0 == mhi_dev_ctxt->flags.link_up) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Link already marked as down, nothing to do\n");
		goto exit;
	}

	if (graceful) {
		r = pci_save_state(pcie_dev);
		if (r)
			mhi_log(mhi_dev_ctxt, MHI_MSG_CRITICAL,
				"Failed to save pcie state ret: %d\n", r);
		mhi_dev_ctxt->core.pcie_state = pci_store_saved_state(pcie_dev);
		pci_disable_device(pcie_dev);
		r = pci_set_power_state(pcie_dev, PCI_D3hot);
		if (r)
			mhi_log(mhi_dev_ctxt, MHI_MSG_CRITICAL,
				"Failed to set pcie power state to D3hot ret:%d\n",
				r);
	}

	r = msm_pcie_pm_control(MSM_PCIE_SUSPEND,
				pcie_dev->bus->number,
				pcie_dev,
				NULL,
				0);
	if (r)
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Failed to suspend pcie bus ret 0x%x\n", r);

	r = mhi_set_bus_request(mhi_dev_ctxt, 0);
	if (r)
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Failed to set bus freq ret %d\n", r);
	mhi_dev_ctxt->flags.link_up = 0;
exit:
	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Exited...\n");

	return 0;
}

int mhi_turn_on_pcie_link(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	int r = 0;
	struct pci_dev *pcie_dev;

	pcie_dev = mhi_dev_ctxt->pcie_device;

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Entered...\n");
	if (mhi_dev_ctxt->flags.link_up)
		goto exit;

	r  = mhi_set_bus_request(mhi_dev_ctxt, 1);
	if (r)
		mhi_log(mhi_dev_ctxt, MHI_MSG_CRITICAL,
			"Could not set bus frequency ret: %d\n", r);

	r = msm_pcie_pm_control(MSM_PCIE_RESUME, pcie_dev->bus->number,
				pcie_dev, NULL, 0);
	if (r) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_CRITICAL,
			"Failed to resume pcie bus ret %d\n", r);
		goto exit;
	}

	r = pci_set_power_state(pcie_dev, PCI_D0);
	if (r) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Failed to set PCI_D0 state ret:%d\n", r);
		goto exit;
	}
	r = pci_enable_device(pcie_dev);
	if (r) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Failed to enable device ret:%d\n", r);
		goto exit;
	}

	pci_load_and_free_saved_state(pcie_dev,
				      &mhi_dev_ctxt->core.pcie_state);
	pci_restore_state(pcie_dev);
	pci_set_master(pcie_dev);

	mhi_dev_ctxt->flags.link_up = 1;
exit:
	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Exited...\n");
	return r;
}

void mhi_link_state_cb(struct msm_pcie_notify *notify)
{
	struct mhi_device_ctxt *mhi_dev_ctxt = NULL;

	if (!notify || !notify->data) {
		pr_err("%s: incomplete handle received\n", __func__);
		return;
	}

	mhi_dev_ctxt = notify->data;
	switch (notify->event) {
	case MSM_PCIE_EVENT_LINKDOWN:
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Received MSM_PCIE_EVENT_LINKDOWN\n");
		break;
	case MSM_PCIE_EVENT_LINKUP:
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Received MSM_PCIE_EVENT_LINKUP\n");
		mhi_dev_ctxt->counters.link_up_cntr++;
		break;
	case MSM_PCIE_EVENT_WAKEUP:
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Received MSM_PCIE_EVENT_WAKE\n");
		__pm_stay_awake(&mhi_dev_ctxt->w_lock);
		__pm_relax(&mhi_dev_ctxt->w_lock);

		if (mhi_dev_ctxt->flags.mhi_initialized) {
			mhi_dev_ctxt->runtime_get(mhi_dev_ctxt);
			mhi_dev_ctxt->runtime_put(mhi_dev_ctxt);
		}
		break;
	default:
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Received bad link event\n");
		return;
	}
}

int mhi_pm_control_device(struct mhi_device *mhi_device, enum mhi_dev_ctrl ctrl)
{
	struct mhi_device_ctxt *mhi_dev_ctxt = mhi_device->mhi_dev_ctxt;
	unsigned long flags;

	if (!mhi_dev_ctxt)
		return -EINVAL;

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Entered with cmd:%s\n",
		TO_MHI_DEV_CTRL_STR(ctrl));

	switch (ctrl) {
	case MHI_DEV_CTRL_INIT:
		return bhi_probe(mhi_dev_ctxt);
	case MHI_DEV_CTRL_POWER_ON:
		return mhi_pm_slave_mode_power_on(mhi_dev_ctxt);
	case MHI_DEV_CTRL_SUSPEND:
		return mhi_pm_slave_mode_suspend(mhi_dev_ctxt);
	case MHI_DEV_CTRL_RESUME:
		return mhi_pm_slave_mode_resume(mhi_dev_ctxt);
	case MHI_DEV_CTRL_POWER_OFF:
		mhi_pm_slave_mode_power_off(mhi_dev_ctxt);
		break;
	case MHI_DEV_CTRL_TRIGGER_RDDM:
		write_lock_irqsave(&mhi_dev_ctxt->pm_xfer_lock, flags);
		if (!MHI_REG_ACCESS_VALID(mhi_dev_ctxt->mhi_pm_state)) {
			write_unlock_irqrestore(&mhi_dev_ctxt->pm_xfer_lock,
						flags);
			mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
				"failed to trigger rddm, no register access in state:0x%x\n",
				mhi_dev_ctxt->mhi_pm_state);
			return -EIO;
		}
		mhi_set_m_state(mhi_dev_ctxt, MHI_STATE_SYS_ERR);
		write_unlock_irqrestore(&mhi_dev_ctxt->pm_xfer_lock, flags);
		break;
	case MHI_DEV_CTRL_RDDM:
		return bhi_rddm(mhi_dev_ctxt, false);
	case MHI_DEV_CTRL_RDDM_KERNEL_PANIC:
		return bhi_rddm(mhi_dev_ctxt, true);
	case MHI_DEV_CTRL_DE_INIT:
		if (mhi_dev_ctxt->mhi_pm_state != MHI_PM_DISABLE) {
			enum MHI_PM_STATE cur_state;
			/*
			 * If bus master calls DE_INIT before calling POWER_OFF
			 * means a critical failure occurred during POWER_ON
			 * state transition and external PCIe device may not
			 * respond to host.  Force PM state to PCIe linkdown
			 * state prior to starting shutdown process to avoid
			 * accessing PCIe link.
			 */
			write_lock_irq(&mhi_dev_ctxt->pm_xfer_lock);
			cur_state = mhi_tryset_pm_state(mhi_dev_ctxt,
						MHI_PM_LD_ERR_FATAL_DETECT);
			write_unlock_irq(&mhi_dev_ctxt->pm_xfer_lock);
			if (unlikely(cur_state != MHI_PM_LD_ERR_FATAL_DETECT)) {
				mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
					"Failed to transition to state 0x%x from 0x%x\n",
					MHI_PM_LD_ERR_FATAL_DETECT, cur_state);
			}
			process_disable_transition(MHI_PM_SHUTDOWN_PROCESS,
						   mhi_dev_ctxt);
		}
		bhi_exit(mhi_dev_ctxt);
		break;
	case MHI_DEV_CTRL_NOTIFY_LINK_ERROR:
	{
		enum MHI_PM_STATE cur_state;

		write_lock_irq(&mhi_dev_ctxt->pm_xfer_lock);
		cur_state = mhi_tryset_pm_state(mhi_dev_ctxt,
						MHI_PM_LD_ERR_FATAL_DETECT);
		write_unlock_irq(&mhi_dev_ctxt->pm_xfer_lock);
		if (unlikely(cur_state != MHI_PM_LD_ERR_FATAL_DETECT))
			mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
				"Failed to transition to state 0x%x from 0x%x\n",
				MHI_PM_LD_ERR_FATAL_DETECT, cur_state);

		/* wake up all threads that's waiting for state change events */
		complete(&mhi_dev_ctxt->cmd_complete);
		wake_up_interruptible(mhi_dev_ctxt->mhi_ev_wq.bhi_event);
		wake_up(mhi_dev_ctxt->mhi_ev_wq.m0_event);
		wake_up(mhi_dev_ctxt->mhi_ev_wq.m3_event);
		break;
	}
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(mhi_pm_control_device);
=======
enum MHI_STATUS mhi_turn_off_pcie_link(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	int r;
	struct pci_dev *pcie_dev;
	enum MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_log(MHI_MSG_INFO, "Entered...\n");
	pcie_dev = mhi_dev_ctxt->dev_info->pcie_device;
	mutex_lock(&mhi_dev_ctxt->mhi_link_state);
	if (0 == mhi_dev_ctxt->flags.link_up) {
		mhi_log(MHI_MSG_CRITICAL,
			"Link already marked as down, nothing to do\n");
		goto exit;
	}
	/* Disable shadow to avoid restoring D3 hot struct device */
	r = msm_pcie_shadow_control(mhi_dev_ctxt->dev_info->pcie_device, 0);
	if (r)
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to stop shadow config space: %d\n", r);

	r = pci_set_power_state(mhi_dev_ctxt->dev_info->pcie_device, PCI_D3hot);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to set pcie power state to D3 hotret: %x\n", r);
		ret_val = MHI_STATUS_ERROR;
		goto exit;
	}
	r = msm_pcie_pm_control(MSM_PCIE_REQ_EXIT_L1,
			mhi_dev_ctxt->dev_info->pcie_device->bus->number,
			mhi_dev_ctxt->dev_info->pcie_device,
			NULL, 0);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL,
			"Failed failed to exit L1: %x\n", r);
	}

	r = msm_pcie_pm_control(MSM_PCIE_SUSPEND,
			mhi_dev_ctxt->dev_info->pcie_device->bus->number,
			mhi_dev_ctxt->dev_info->pcie_device,
			NULL,
			0);
	if (r)
		mhi_log(MHI_MSG_CRITICAL,
				"Failed to suspend pcie bus ret 0x%x\n", r);
	mhi_dev_ctxt->flags.link_up = 0;
exit:
	mutex_unlock(&mhi_dev_ctxt->mhi_link_state);
	mhi_log(MHI_MSG_INFO, "Exited...\n");
	return MHI_STATUS_SUCCESS;
}

enum MHI_STATUS mhi_turn_on_pcie_link(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	int r = 0;
	struct pci_dev *pcie_dev;
	enum MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	pcie_dev = mhi_dev_ctxt->dev_info->pcie_device;

	mutex_lock(&mhi_dev_ctxt->mhi_link_state);
	mhi_log(MHI_MSG_INFO, "Entered...\n");
	if (mhi_dev_ctxt->flags.link_up)
		goto exit;
	r = msm_pcie_pm_control(MSM_PCIE_RESUME,
			mhi_dev_ctxt->dev_info->pcie_device->bus->number,
			mhi_dev_ctxt->dev_info->pcie_device,
			NULL, 0);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL,
				"Failed to resume pcie bus ret %d\n", r);
		ret_val = MHI_STATUS_ERROR;
		goto exit;
	}

	r = pci_set_power_state(mhi_dev_ctxt->dev_info->pcie_device,
				PCI_D0);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL,
				"Failed to load stored state %d\n", r);
		ret_val = MHI_STATUS_ERROR;
		goto exit;
	}
	r = msm_pcie_recover_config(mhi_dev_ctxt->dev_info->pcie_device);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL,
				"Failed to Recover config space ret: %d\n", r);
		ret_val = MHI_STATUS_ERROR;
		goto exit;
	}
	mhi_dev_ctxt->flags.link_up = 1;
exit:
	mutex_unlock(&mhi_dev_ctxt->mhi_link_state);
	mhi_log(MHI_MSG_INFO, "Exited...\n");
	return ret_val;
}

>>>>>>> p9x
