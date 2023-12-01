/* Copyright (c) 2014-2017, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/slab.h>

#include "mhi_sys.h"

enum MHI_DEBUG_LEVEL mhi_msg_lvl = MHI_MSG_ERROR;
<<<<<<< HEAD

#ifdef CONFIG_MSM_MHI_DEBUG
enum MHI_DEBUG_LEVEL mhi_ipc_log_lvl = MHI_MSG_VERBOSE;
#else
enum MHI_DEBUG_LEVEL mhi_ipc_log_lvl = MHI_MSG_ERROR;
#endif

=======
enum MHI_DEBUG_LEVEL mhi_ipc_log_lvl = MHI_MSG_VERBOSE;
>>>>>>> p9x
unsigned int mhi_log_override;

module_param(mhi_msg_lvl , uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mhi_msg_lvl, "dbg lvl");

module_param(mhi_ipc_log_lvl, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mhi_ipc_log_lvl, "dbg lvl");

<<<<<<< HEAD
const char * const mhi_states_str[MHI_STATE_LIMIT] = {
	[MHI_STATE_RESET] = "RESET",
	[MHI_STATE_READY] = "READY",
	[MHI_STATE_M0] = "M0",
	[MHI_STATE_M1] = "M1",
	[MHI_STATE_M2] = "M2",
	[MHI_STATE_M3] = "M3",
	"Reserved: 0x06",
	[MHI_STATE_BHI] = "BHI",
	[MHI_STATE_SYS_ERR] = "SYS_ERR",
};
=======
module_param(mhi_log_override , uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mhi_log_override, "dbg class");
>>>>>>> p9x

static ssize_t mhi_dbgfs_chan_read(struct file *fp, char __user *buf,
				size_t count, loff_t *offp)
{
	int amnt_copied = 0;
	struct mhi_chan_ctxt *chan_ctxt;
<<<<<<< HEAD
	struct mhi_device_ctxt *mhi_dev_ctxt = fp->private_data;
=======
	struct mhi_device_ctxt *mhi_dev_ctxt =
		&mhi_devices.device_list[0].mhi_ctxt;
>>>>>>> p9x
	uintptr_t v_wp_index;
	uintptr_t v_rp_index;
	int valid_chan = 0;
	struct mhi_chan_ctxt *cc_list;
	struct mhi_client_handle *client_handle;
<<<<<<< HEAD
	struct mhi_client_config *client_config;
	int pkts_queued;

	if (NULL == mhi_dev_ctxt)
		return -EIO;
	cc_list = mhi_dev_ctxt->dev_space.ring_ctxt.cc_list;
	*offp = (u32)(*offp) % MHI_MAX_CHANNELS;

	while (!valid_chan) {
=======

	if (NULL == mhi_dev_ctxt)
		return -EIO;
	cc_list = mhi_dev_ctxt->mhi_ctrl_seg->mhi_cc_list;
	*offp = (u32)(*offp) % MHI_MAX_CHANNELS;

	while (!valid_chan) {
		client_handle = mhi_dev_ctxt->client_handle_list[*offp];
>>>>>>> p9x
		if (*offp == (MHI_MAX_CHANNELS - 1))
			msleep(1000);
		if (!VALID_CHAN_NR(*offp) ||
		    !cc_list[*offp].mhi_trb_ring_base_addr ||
<<<<<<< HEAD
		    !mhi_dev_ctxt->client_handle_list[*offp]) {
=======
		    !client_handle) {
>>>>>>> p9x
			*offp += 1;
			*offp = (u32)(*offp) % MHI_MAX_CHANNELS;
			continue;
		}
<<<<<<< HEAD
		client_handle = mhi_dev_ctxt->client_handle_list[*offp];
		client_config = client_handle->client_config;
=======
>>>>>>> p9x
		valid_chan = 1;
	}

	chan_ctxt = &cc_list[*offp];
	get_element_index(&mhi_dev_ctxt->mhi_local_chan_ctxt[*offp],
			mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].rp,
			&v_rp_index);
	get_element_index(&mhi_dev_ctxt->mhi_local_chan_ctxt[*offp],
			mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].wp,
			&v_wp_index);

<<<<<<< HEAD
	pkts_queued = client_config->chan_info.max_desc -
		get_nr_avail_ring_elements(mhi_dev_ctxt,
					   &mhi_dev_ctxt->
					   mhi_local_chan_ctxt[*offp]) - 1;
	amnt_copied =
	scnprintf(mhi_dev_ctxt->chan_info,
		  MHI_LOG_SIZE,
		  "%s0x%x %s %d %s 0x%x %s 0x%llx %s %p %s %p %s %lu %s %p %s %lu %s %d %s %d %s %u\n",
		  "chan:",
		  (unsigned int)*offp,
		  "pkts from dev:",
		  mhi_dev_ctxt->counters.chan_pkts_xferd[*offp],
		  "state:",
		  chan_ctxt->chstate,
		  "p_base:",
		  chan_ctxt->mhi_trb_ring_base_addr,
		  "v_base:",
		  mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].base,
		  "v_wp:",
		  mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].wp,
		  "index:",
		  v_wp_index,
		  "v_rp:",
		  mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].rp,
		  "index:",
		  v_rp_index,
		  "pkts_queued",
		  pkts_queued,
		  "/",
		  client_config->chan_info.max_desc,
		  "bb_used:",
		  mhi_dev_ctxt->counters.bb_used[*offp]);
=======
	amnt_copied =
	scnprintf(mhi_dev_ctxt->chan_info,
		MHI_LOG_SIZE,
		"%s0x%x %s %d %s 0x%x %s 0x%llx %s %p %s %p %s %lu %s %p %s %lu %s %d %s %d\n",
		"chan:",
		(unsigned int)*offp,
		"pkts from dev:",
		mhi_dev_ctxt->counters.chan_pkts_xferd[*offp],
		"state:",
		chan_ctxt->mhi_chan_state,
		"p_base:",
		chan_ctxt->mhi_trb_ring_base_addr,
		"v_base:",
		mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].base,
		"v_wp:",
		mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].wp,
		"index:",
		v_wp_index,
		"v_rp:",
		mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].rp,
		"index:",
		v_rp_index,
		"pkts_queued",
		get_nr_avail_ring_elements(
		&mhi_dev_ctxt->mhi_local_chan_ctxt[*offp]),
		"/",
		client_handle->chan_info.max_desc);
>>>>>>> p9x

	*offp += 1;

	if (amnt_copied < count)
		return amnt_copied -
			copy_to_user(buf, mhi_dev_ctxt->chan_info, amnt_copied);
	else
		return -ENOMEM;
}

<<<<<<< HEAD
int mhi_dbgfs_open(struct inode *inode, struct file *fp)
{
	fp->private_data = inode->i_private;
	return 0;
}

static const struct file_operations mhi_dbgfs_chan_fops = {
	.read = mhi_dbgfs_chan_read,
	.write = NULL,
	.open = mhi_dbgfs_open,
=======
static const struct file_operations mhi_dbgfs_chan_fops = {
	.read = mhi_dbgfs_chan_read,
	.write = NULL,
>>>>>>> p9x
};

static ssize_t mhi_dbgfs_ev_read(struct file *fp, char __user *buf,
				size_t count, loff_t *offp)
{
	int amnt_copied = 0;
	int event_ring_index = 0;
	struct mhi_event_ctxt *ev_ctxt;
	uintptr_t v_wp_index;
	uintptr_t v_rp_index;
	uintptr_t device_p_rp_index;

<<<<<<< HEAD
	struct mhi_device_ctxt *mhi_dev_ctxt = fp->private_data;
=======
	struct mhi_device_ctxt *mhi_dev_ctxt =
		&mhi_devices.device_list[0].mhi_ctxt;
>>>>>>> p9x
	if (NULL == mhi_dev_ctxt)
		return -EIO;
	*offp = (u32)(*offp) % mhi_dev_ctxt->mmio_info.nr_event_rings;
	event_ring_index = *offp;
<<<<<<< HEAD
	ev_ctxt = &mhi_dev_ctxt->dev_space.ring_ctxt.ec_list[event_ring_index];
=======
	ev_ctxt = &mhi_dev_ctxt->mhi_ctrl_seg->mhi_ec_list[event_ring_index];
>>>>>>> p9x
	if (*offp == (mhi_dev_ctxt->mmio_info.nr_event_rings - 1))
		msleep(1000);

	get_element_index(&mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index],
			mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index].rp,
			&v_rp_index);
	get_element_index(&mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index],
			mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index].wp,
			&v_wp_index);
	get_element_index(&mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index],
			mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index].wp,
			&v_wp_index);
	get_element_index(&mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index],
<<<<<<< HEAD
			(void *)mhi_p2v_addr(mhi_dev_ctxt,
					MHI_RING_TYPE_EVENT_RING,
					event_ring_index,
					ev_ctxt->mhi_event_read_ptr),
					&device_p_rp_index);
=======
			(void *)dma_to_virt(NULL, ev_ctxt->mhi_event_read_ptr),
			&device_p_rp_index);
>>>>>>> p9x

	amnt_copied =
	scnprintf(mhi_dev_ctxt->chan_info,
		MHI_LOG_SIZE,
		"%s 0x%d %s %02x %s 0x%08x %s 0x%08x %s 0x%llx %s %llx %s %lu %s %p %s %p %s %lu %s %p %s %lu\n",
		"Event Context ",
		(unsigned int)event_ring_index,
		"Intmod_T",
		MHI_GET_EV_CTXT(EVENT_CTXT_INTMODT, ev_ctxt),
		"MSI Vector",
		ev_ctxt->mhi_msi_vector,
		"MSI RX Count",
		mhi_dev_ctxt->counters.msi_counter[*offp],
		"p_base:",
		ev_ctxt->mhi_event_ring_base_addr,
		"p_rp:",
		ev_ctxt->mhi_event_read_ptr,
		"index:",
		device_p_rp_index,
		"v_base:",
		mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index].base,
		"v_wp:",
		mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index].wp,
		"index:",
		v_wp_index,
		"v_rp:",
		mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index].rp,
		"index:",
		v_rp_index);

	*offp += 1;
	if (amnt_copied < count)
		return amnt_copied -
			copy_to_user(buf, mhi_dev_ctxt->chan_info, amnt_copied);
	else
		return -ENOMEM;
}

static const struct file_operations mhi_dbgfs_ev_fops = {
	.read = mhi_dbgfs_ev_read,
	.write = NULL,
<<<<<<< HEAD
	.open = mhi_dbgfs_open,
=======
>>>>>>> p9x
};

static ssize_t mhi_dbgfs_state_read(struct file *fp, char __user *buf,
				size_t count, loff_t *offp)
{
	int amnt_copied = 0;
<<<<<<< HEAD
	struct mhi_device_ctxt *mhi_dev_ctxt = fp->private_data;

=======
	struct mhi_device_ctxt *mhi_dev_ctxt =
		&mhi_devices.device_list[0].mhi_ctxt;
>>>>>>> p9x
	if (NULL == mhi_dev_ctxt)
		return -EIO;
	msleep(100);
	amnt_copied =
	scnprintf(mhi_dev_ctxt->chan_info,
<<<<<<< HEAD
		  MHI_LOG_SIZE,
		  "%s %s %s 0x%02x %s %u %s %u %s %u %s %u %s %u %s %u %s %d %s %d %s %d\n",
		  "MHI State:",
		  TO_MHI_STATE_STR(mhi_dev_ctxt->mhi_state),
		  "PM State:",
		  mhi_dev_ctxt->mhi_pm_state,
		  "M0->M1:",
		  mhi_dev_ctxt->counters.m0_m1,
		  "M1->M2:",
		  mhi_dev_ctxt->counters.m1_m2,
		  "M2->M0:",
		  mhi_dev_ctxt->counters.m2_m0,
		  "M0->M3:",
		  mhi_dev_ctxt->counters.m0_m3,
		  "M1->M3:",
		  mhi_dev_ctxt->counters.m1_m3,
		  "M3->M0:",
		  mhi_dev_ctxt->counters.m3_m0,
		  "device_wake:",
		  atomic_read(&mhi_dev_ctxt->counters.device_wake),
		  "usage_count:",
		  atomic_read(&mhi_dev_ctxt->pcie_device->dev.
			      power.usage_count),
		  "outbound_acks:",
		  atomic_read(&mhi_dev_ctxt->counters.outbound_acks));
=======
			MHI_LOG_SIZE,
			"%s %u %s %d %s %d %s %d %s %d %s %d %s %d %s %d %s %d %s %d %s %d, %s, %d, %s %d\n",
			"Our State:",
			mhi_dev_ctxt->mhi_state,
			"M0->M1:",
			mhi_dev_ctxt->counters.m0_m1,
			"M0<-M1:",
			mhi_dev_ctxt->counters.m1_m0,
			"M1->M2:",
			mhi_dev_ctxt->counters.m1_m2,
			"M0<-M2:",
			mhi_dev_ctxt->counters.m2_m0,
			"M0->M3:",
			mhi_dev_ctxt->counters.m0_m3,
			"M0<-M3:",
			mhi_dev_ctxt->counters.m3_m0,
			"M3_ev_TO:",
			mhi_dev_ctxt->counters.m3_event_timeouts,
			"M0_ev_TO:",
			mhi_dev_ctxt->counters.m0_event_timeouts,
			"MSI_d:",
			mhi_dev_ctxt->counters.msi_disable_cntr,
			"MSI_e:",
			mhi_dev_ctxt->counters.msi_enable_cntr,
			"outstanding_acks:",
			atomic_read(&mhi_dev_ctxt->counters.outbound_acks),
			"LPM:",
			mhi_dev_ctxt->enable_lpm);
>>>>>>> p9x
	if (amnt_copied < count)
		return amnt_copied - copy_to_user(buf,
				mhi_dev_ctxt->chan_info, amnt_copied);
	else
		return -ENOMEM;
<<<<<<< HEAD
	return 0;
=======
>>>>>>> p9x
}

static const struct file_operations mhi_dbgfs_state_fops = {
	.read = mhi_dbgfs_state_read,
	.write = NULL,
<<<<<<< HEAD
	.open = mhi_dbgfs_open,
};

=======
};

inline void *mhi_get_virt_addr(struct mhi_meminfo *meminfo)
{
	return (void *)meminfo->va_aligned;
}

inline u64 mhi_get_memregion_len(struct mhi_meminfo *meminfo)
{
	return meminfo->size;
}

enum MHI_STATUS mhi_mallocmemregion(struct mhi_meminfo *meminfo, size_t size)
{
	meminfo->va_unaligned = (uintptr_t)dma_alloc_coherent(
				meminfo->dev,
				size,
				(dma_addr_t *)&(meminfo->pa_unaligned),
				GFP_KERNEL);
	if (!meminfo->va_unaligned)
		return MHI_STATUS_ERROR;
	meminfo->va_aligned = meminfo->va_unaligned;
	meminfo->pa_aligned = meminfo->pa_unaligned;
	meminfo->size = size;
	if ((meminfo->pa_unaligned + size) >= MHI_DATA_SEG_WINDOW_END_ADDR)
		return MHI_STATUS_ERROR;

	if (0 == meminfo->va_unaligned)
		return MHI_STATUS_ERROR;
	mb();
	return MHI_STATUS_SUCCESS;
}

void mhi_freememregion(struct mhi_meminfo *meminfo)
{
	mb();
	dma_free_coherent(meminfo->dev,
			meminfo->size,
			(dma_addr_t *)&meminfo->pa_unaligned,
			GFP_KERNEL);
	meminfo->va_aligned = 0;
	meminfo->pa_aligned = 0;
	meminfo->va_unaligned = 0;
	meminfo->pa_unaligned = 0;
}

>>>>>>> p9x
int mhi_init_debugfs(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	struct dentry *mhi_chan_stats;
	struct dentry *mhi_state_stats;
	struct dentry *mhi_ev_stats;
<<<<<<< HEAD
	const struct pcie_core_info *core = &mhi_dev_ctxt->core;
	char node_name[32];

	snprintf(node_name,
		 sizeof(node_name),
		 "%04x_%02u.%02u.%02u",
		 core->dev_id, core->domain, core->bus, core->slot);

	mhi_dev_ctxt->child =
		debugfs_create_dir(node_name, mhi_dev_ctxt->parent);
	if (mhi_dev_ctxt->child == NULL) {
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Failed to create debugfs parent dir.\n");
=======
	mhi_dev_ctxt->mhi_parent_folder =
					debugfs_create_dir("mhi", NULL);
	if (mhi_dev_ctxt->mhi_parent_folder == NULL) {
		mhi_log(MHI_MSG_INFO, "Failed to create debugfs parent dir.\n");
>>>>>>> p9x
		return -EIO;
	}
	mhi_chan_stats = debugfs_create_file("mhi_chan_stats",
					0444,
<<<<<<< HEAD
					mhi_dev_ctxt->child,
=======
					mhi_dev_ctxt->mhi_parent_folder,
>>>>>>> p9x
					mhi_dev_ctxt,
					&mhi_dbgfs_chan_fops);
	if (mhi_chan_stats == NULL)
		return -ENOMEM;
	mhi_ev_stats = debugfs_create_file("mhi_ev_stats",
					0444,
<<<<<<< HEAD
					mhi_dev_ctxt->child,
=======
					mhi_dev_ctxt->mhi_parent_folder,
>>>>>>> p9x
					mhi_dev_ctxt,
					&mhi_dbgfs_ev_fops);
	if (mhi_ev_stats == NULL)
		goto clean_chan;
	mhi_state_stats = debugfs_create_file("mhi_state_stats",
					0444,
<<<<<<< HEAD
					mhi_dev_ctxt->child,
=======
					mhi_dev_ctxt->mhi_parent_folder,
>>>>>>> p9x
					mhi_dev_ctxt,
					&mhi_dbgfs_state_fops);
	if (mhi_state_stats == NULL)
		goto clean_ev_stats;

	mhi_dev_ctxt->chan_info = kmalloc(MHI_LOG_SIZE, GFP_KERNEL);
	if (mhi_dev_ctxt->chan_info == NULL)
<<<<<<< HEAD
		goto clean_ev_stats;
	return 0;

=======
		goto clean_all;
	return 0;
clean_all:
	debugfs_remove(mhi_state_stats);
>>>>>>> p9x
clean_ev_stats:
	debugfs_remove(mhi_ev_stats);
clean_chan:
	debugfs_remove(mhi_chan_stats);
<<<<<<< HEAD
	debugfs_remove(mhi_dev_ctxt->child);
	return -ENOMEM;
}

uintptr_t mhi_p2v_addr(struct mhi_device_ctxt *mhi_dev_ctxt,
			enum MHI_RING_TYPE type,
			u32 chan, uintptr_t phy_ptr)
{
	uintptr_t virtual_ptr;
	struct mhi_ring_ctxt *cs = &mhi_dev_ctxt->dev_space.ring_ctxt;

	switch (type) {
	case MHI_RING_TYPE_EVENT_RING:
		 virtual_ptr = (uintptr_t)((phy_ptr -
		(uintptr_t)cs->ec_list[chan].mhi_event_ring_base_addr)
			+ mhi_dev_ctxt->mhi_local_event_ctxt[chan].base);
		break;
	case MHI_RING_TYPE_XFER_RING:
		virtual_ptr = (uintptr_t)((phy_ptr -
		(uintptr_t)cs->cc_list[chan].mhi_trb_ring_base_addr)
				+ mhi_dev_ctxt->mhi_local_chan_ctxt[chan].base);
		 break;
	case MHI_RING_TYPE_CMD_RING:
		virtual_ptr = (uintptr_t)((phy_ptr -
		(uintptr_t)cs->cmd_ctxt[chan].mhi_cmd_ring_base_addr)
				+ mhi_dev_ctxt->mhi_local_cmd_ctxt[chan].base);
		break;
	default:
		break;
		}
	return virtual_ptr;
}

dma_addr_t mhi_v2p_addr(struct mhi_device_ctxt *mhi_dev_ctxt,
			enum MHI_RING_TYPE type,
			 u32 chan, uintptr_t va_ptr)
{
	dma_addr_t phy_ptr;
	struct mhi_ring_ctxt *cs = &mhi_dev_ctxt->dev_space.ring_ctxt;

	switch (type) {
	case MHI_RING_TYPE_EVENT_RING:
		phy_ptr = (dma_addr_t)((va_ptr -
		(uintptr_t)mhi_dev_ctxt->mhi_local_event_ctxt[chan].base) +
		(uintptr_t)cs->ec_list[chan].mhi_event_ring_base_addr);
		break;
	case MHI_RING_TYPE_XFER_RING:
		phy_ptr = (dma_addr_t)((va_ptr -
		(uintptr_t)mhi_dev_ctxt->mhi_local_chan_ctxt[chan].base) +
		((uintptr_t)cs->cc_list[chan].mhi_trb_ring_base_addr));
		break;
	case MHI_RING_TYPE_CMD_RING:
		phy_ptr = (dma_addr_t)((va_ptr -
	(uintptr_t)mhi_dev_ctxt->mhi_local_cmd_ctxt[chan].base) +
	((uintptr_t)cs->cmd_ctxt[chan].mhi_cmd_ring_base_addr));
		break;
	default:
		break;
		}
		return phy_ptr;
}
=======
	debugfs_remove(mhi_dev_ctxt->mhi_parent_folder);
	return -ENOMEM;
}
>>>>>>> p9x
