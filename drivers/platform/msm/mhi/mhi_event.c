<<<<<<< HEAD
/* Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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


#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>

#include "mhi.h"
#include "mhi_macros.h"
#include "mhi_sys.h"

int mhi_populate_event_cfg(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	int r, i;
	char dt_prop[MAX_BUF_SIZE];
	const struct device_node *np =
<<<<<<< HEAD
		mhi_dev_ctxt->plat_dev->dev.of_node;
=======
		mhi_dev_ctxt->dev_info->plat_dev->dev.of_node;
>>>>>>> p9x

	r = of_property_read_u32(np, "mhi-event-rings",
			&mhi_dev_ctxt->mmio_info.nr_event_rings);
	if (r) {
<<<<<<< HEAD
		mhi_log(mhi_dev_ctxt, MHI_MSG_CRITICAL,
			"Failed to pull event ring info from DT, %d\n", r);
		return -EINVAL;
=======
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to pull event ring info from DT, %d\n", r);
		goto dt_error;
>>>>>>> p9x
	}
	mhi_dev_ctxt->ev_ring_props =
				kzalloc(sizeof(struct mhi_event_ring_cfg) *
					mhi_dev_ctxt->mmio_info.nr_event_rings,
					GFP_KERNEL);
<<<<<<< HEAD
	if (!mhi_dev_ctxt->ev_ring_props)
		return -ENOMEM;

	for (i = 0; i < mhi_dev_ctxt->mmio_info.nr_event_rings; ++i) {
		u32 dt_configs[6];
		int no_elements;

		scnprintf(dt_prop, MAX_BUF_SIZE, "%s%d", "mhi-event-cfg-", i);
		no_elements = of_property_count_elems_of_size(np, dt_prop,
							sizeof(dt_configs));
		if (no_elements != 1)
			goto dt_error;
		r = of_property_read_u32_array(
					np,
					dt_prop,
					dt_configs,
					sizeof(dt_configs) / sizeof(u32));
		if (r) {
			mhi_log(mhi_dev_ctxt, MHI_MSG_CRITICAL,
=======
	if (!mhi_dev_ctxt->ev_ring_props) {
		r = -ENOMEM;
		goto dt_error;
	}

	for (i = 0; i < mhi_dev_ctxt->mmio_info.nr_event_rings; ++i) {
		scnprintf(dt_prop, MAX_BUF_SIZE, "%s%d", "mhi-event-cfg-", i);
		r = of_property_read_u32_array(np, dt_prop,
					(u32 *)&mhi_dev_ctxt->ev_ring_props[i],
					4);
		if (r) {
			mhi_log(MHI_MSG_CRITICAL,
>>>>>>> p9x
				"Failed to pull ev ring %d info from DT %d\n",
				i, r);
			goto dt_error;
		}
<<<<<<< HEAD
		mhi_dev_ctxt->ev_ring_props[i].nr_desc = dt_configs[0];
		mhi_dev_ctxt->ev_ring_props[i].msi_vec = dt_configs[1];
		mhi_dev_ctxt->ev_ring_props[i].intmod = dt_configs[2];
		mhi_dev_ctxt->ev_ring_props[i].chan = dt_configs[3];
		mhi_dev_ctxt->ev_ring_props[i].priority = dt_configs[4];
		mhi_dev_ctxt->ev_ring_props[i].flags = dt_configs[5];
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"ev ring %d,desc:0x%x,msi:0x%x,intmod%d chan:%u priority:%u flags0x%x\n",
			i,
			mhi_dev_ctxt->ev_ring_props[i].nr_desc,
			mhi_dev_ctxt->ev_ring_props[i].msi_vec,
			mhi_dev_ctxt->ev_ring_props[i].intmod,
			mhi_dev_ctxt->ev_ring_props[i].chan,
			mhi_dev_ctxt->ev_ring_props[i].priority,
			mhi_dev_ctxt->ev_ring_props[i].flags);
=======
		mhi_log(MHI_MSG_INFO,
			"Pulled ev ring %d, desc: 0x%x, msi_vec: 0x%x, intmod %d flags 0x%x\n",
			i, mhi_dev_ctxt->ev_ring_props[i].nr_desc,
			   mhi_dev_ctxt->ev_ring_props[i].msi_vec,
			   mhi_dev_ctxt->ev_ring_props[i].intmod,
			   mhi_dev_ctxt->ev_ring_props[i].flags);
>>>>>>> p9x
		if (GET_EV_PROPS(EV_MANAGED,
			mhi_dev_ctxt->ev_ring_props[i].flags))
			mhi_dev_ctxt->ev_ring_props[i].mhi_handler_ptr =
							mhi_msi_handlr;
		else
			mhi_dev_ctxt->ev_ring_props[i].mhi_handler_ptr =
							mhi_msi_ipa_handlr;
<<<<<<< HEAD
		if (MHI_HW_RING == GET_EV_PROPS(EV_TYPE,
			mhi_dev_ctxt->ev_ring_props[i].flags)) {
			mhi_dev_ctxt->ev_ring_props[i].class = MHI_HW_RING;
			mhi_dev_ctxt->mmio_info.nr_hw_event_rings++;
		} else {
			mhi_dev_ctxt->ev_ring_props[i].class = MHI_SW_RING;
			mhi_dev_ctxt->mmio_info.nr_sw_event_rings++;
		}
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Detected %d SW EV rings and %d HW EV rings out of %d EV rings\n",
			mhi_dev_ctxt->mmio_info.nr_sw_event_rings,
			mhi_dev_ctxt->mmio_info.nr_hw_event_rings,
			mhi_dev_ctxt->mmio_info.nr_event_rings);
	}

	return 0;
dt_error:
	kfree(mhi_dev_ctxt->ev_ring_props);
	mhi_dev_ctxt->ev_ring_props = NULL;
	return -EINVAL;
}

int create_local_ev_ctxt(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	int r = 0;
	int i;

	mhi_dev_ctxt->mhi_local_event_ctxt = kzalloc(sizeof(struct mhi_ring)*
					mhi_dev_ctxt->mmio_info.nr_event_rings,
					GFP_KERNEL);
	if (!mhi_dev_ctxt->mhi_local_event_ctxt)
		return -ENOMEM;

=======
	}
dt_error:
	return r;
}

int create_ev_rings(struct mhi_device_ctxt *mhi_dev_ctxt)
{
	int r = 0, i;
	struct mhi_event_ctxt *ev_ctxt = NULL;

	size_t ctxt_size = sizeof(struct mhi_event_ctxt) *
				    mhi_dev_ctxt->mmio_info.nr_event_rings;
	/* Allocate the event contexts in uncached memory */
	mhi_dev_ctxt->mhi_ctrl_seg->mhi_ec_list =
				dma_alloc_coherent(
				&mhi_dev_ctxt->dev_info->plat_dev->dev,
				ctxt_size,
				&mhi_dev_ctxt->mmio_info.dma_ev_ctxt,
				GFP_KERNEL);
	if (!mhi_dev_ctxt->mhi_ctrl_seg->mhi_ec_list)
		return -ENOMEM;

	mhi_dev_ctxt->mhi_local_event_ctxt = kzalloc(sizeof(struct mhi_ring) *
				     mhi_dev_ctxt->mmio_info.nr_event_rings,
				     GFP_KERNEL);
	if (!mhi_dev_ctxt->mhi_local_event_ctxt) {
		r = -ENOMEM;
		goto free_ec_list;
	}
	mhi_dev_ctxt->counters.ev_counter = kzalloc(sizeof(u32) *
				     mhi_dev_ctxt->mmio_info.nr_event_rings,
				     GFP_KERNEL);
	if (!mhi_dev_ctxt->counters.ev_counter) {
		r = -ENOMEM;
		goto free_local_ec_list;
	}
>>>>>>> p9x
	mhi_dev_ctxt->counters.msi_counter = kzalloc(sizeof(u32) *
				     mhi_dev_ctxt->mmio_info.nr_event_rings,
				     GFP_KERNEL);
	if (!mhi_dev_ctxt->counters.msi_counter) {
		r = -ENOMEM;
<<<<<<< HEAD
		goto free_local_ec_list;
	}

	for (i = 0; i < mhi_dev_ctxt->mmio_info.nr_event_rings; i++) {
		struct mhi_ring *mhi_ring = &mhi_dev_ctxt->
			mhi_local_event_ctxt[i];

		spin_lock_init(&mhi_ring->ring_lock);
		tasklet_init(&mhi_ring->ev_task, mhi_ev_task,
			     (unsigned long)mhi_ring);
		INIT_WORK(&mhi_ring->ev_worker, process_event_ring);
	}

	return r;

free_local_ec_list:
	kfree(mhi_dev_ctxt->mhi_local_event_ctxt);
=======
		goto free_ev_counter;
	}

	mhi_dev_ctxt->mmio_info.dma_ev_rings = kzalloc(sizeof(void *) *
				     mhi_dev_ctxt->mmio_info.nr_event_rings,
				     GFP_KERNEL);
	if (!mhi_dev_ctxt->mmio_info.dma_ev_rings) {
		r = -ENOMEM;
		goto free_msi_counter;
	}
	mhi_log(MHI_MSG_INFO, "Allocated ECABAP at Virt: 0x%p, Phys 0x%lx\n",
			mhi_dev_ctxt->mhi_ctrl_seg->mhi_ec_list,
			(uintptr_t)mhi_dev_ctxt->mmio_info.dma_ev_ctxt);

	/* Allocate event ring elements for each ring */
	for (i = 0; i < mhi_dev_ctxt->mmio_info.nr_event_rings; ++i) {
		dma_addr_t ring_base_addr;
		ev_ctxt = &mhi_dev_ctxt->mhi_ctrl_seg->mhi_ec_list[i];
		mhi_dev_ctxt->mhi_local_event_ctxt[i].base =
			dma_alloc_coherent(
				&mhi_dev_ctxt->dev_info->plat_dev->dev,
				sizeof(union mhi_event_pkt) *
				mhi_dev_ctxt->ev_ring_props[i].nr_desc,
				&ring_base_addr,
				GFP_KERNEL);
		if (!mhi_dev_ctxt->mhi_local_event_ctxt[i].base) {
			r = -ENOMEM;
			goto free_event_ring;
		}

		ev_ctxt->mhi_event_ring_base_addr = ring_base_addr;
		ev_ctxt->mhi_event_read_ptr = ring_base_addr;
		ev_ctxt->mhi_event_write_ptr = ring_base_addr;

		mhi_dev_ctxt->mhi_local_event_ctxt[i].wp =
				mhi_dev_ctxt->mhi_local_event_ctxt[i].base;
		mhi_dev_ctxt->mhi_local_event_ctxt[i].rp =
				mhi_dev_ctxt->mhi_local_event_ctxt[i].base;
		mhi_log(MHI_MSG_INFO, "Allocated Event Ring %d\n", i);
	}
	return r;

free_event_ring:
	for (; i > 0; --i) {
		ev_ctxt = &mhi_dev_ctxt->mhi_ctrl_seg->mhi_ec_list[i];
		dma_free_coherent(&mhi_dev_ctxt->dev_info->plat_dev->dev,
				  sizeof(union mhi_event_pkt *) *
				  mhi_dev_ctxt->ev_ring_props[i].nr_desc,
				  mhi_dev_ctxt->mhi_local_event_ctxt[i].base,
				  ev_ctxt->mhi_event_ring_base_addr);
	}
	kfree(mhi_dev_ctxt->mmio_info.dma_ev_rings);
free_msi_counter:
	kfree(mhi_dev_ctxt->counters.msi_counter);
free_ev_counter:
	kfree(mhi_dev_ctxt->counters.ev_counter);
free_local_ec_list:
	kfree(mhi_dev_ctxt->mhi_local_event_ctxt);
free_ec_list:
	dma_free_coherent(&mhi_dev_ctxt->dev_info->plat_dev->dev,
				ctxt_size,
				mhi_dev_ctxt->mhi_ctrl_seg->mhi_ec_list,
				mhi_dev_ctxt->mmio_info.dma_ev_ctxt);
>>>>>>> p9x
	return r;
}
void ring_ev_db(struct mhi_device_ctxt *mhi_dev_ctxt, u32 event_ring_index)
{
	struct mhi_ring *event_ctxt = NULL;
	u64 db_value = 0;
<<<<<<< HEAD
	unsigned long flags;

	event_ctxt =
		&mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index];
	spin_lock_irqsave(&event_ctxt->ring_lock, flags);
	db_value = mhi_v2p_addr(mhi_dev_ctxt, MHI_RING_TYPE_EVENT_RING,
						event_ring_index,
						(uintptr_t) event_ctxt->wp);
	event_ctxt->db_mode.process_db(mhi_dev_ctxt,
				    mhi_dev_ctxt->mmio_info.event_db_addr,
				    event_ring_index,
				    db_value);
	spin_unlock_irqrestore(&event_ctxt->ring_lock, flags);
}

static int mhi_event_ring_init(struct mhi_event_ctxt *ev_list,
			       struct mhi_ring *ring,
			       struct mhi_device_ctxt *mhi_dev_ctxt,
			       int index,
			       u32 el_per_ring,
			       u32 intmodt_val,
			       u32 msi_vec,
			       enum MHI_BRSTMODE brstmode)
=======
	event_ctxt =
		&mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index];
	db_value = virt_to_dma(NULL, event_ctxt->wp);
	mhi_process_db(mhi_dev_ctxt, mhi_dev_ctxt->mmio_info.event_db_addr,
					event_ring_index, db_value);
}

static enum MHI_STATUS mhi_event_ring_init(struct mhi_event_ctxt *ev_list,
				struct mhi_ring *ring, u32 el_per_ring,
				u32 intmodt_val, u32 msi_vec)
>>>>>>> p9x
{
	ev_list->mhi_event_er_type  = MHI_EVENT_RING_TYPE_VALID;
	ev_list->mhi_msi_vector     = msi_vec;
	ev_list->mhi_event_ring_len = el_per_ring*sizeof(union mhi_event_pkt);
	MHI_SET_EV_CTXT(EVENT_CTXT_INTMODT, ev_list, intmodt_val);
<<<<<<< HEAD
	ring->mhi_dev_ctxt = mhi_dev_ctxt;
	ring->index = index;
	ring->len = ((size_t)(el_per_ring)*sizeof(union mhi_event_pkt));
	ring->el_size = sizeof(union mhi_event_pkt);
	ring->overwrite_en = 0;

	ring->db_mode.db_mode = 1;
	ring->db_mode.brstmode = brstmode;
	switch (ring->db_mode.brstmode) {
	case MHI_BRSTMODE_ENABLE:
		ring->db_mode.process_db = mhi_process_db_brstmode;
		break;
	case MHI_BRSTMODE_DISABLE:
		ring->db_mode.process_db = mhi_process_db_brstmode_disable;
		break;
	default:
		ring->db_mode.process_db = mhi_process_db;
	}

	/* Flush writes to MMIO */
	wmb();
	return 0;
}

void init_event_ctxt_array(struct mhi_device_ctxt *mhi_dev_ctxt)
=======
	ring->len = ((size_t)(el_per_ring)*sizeof(union mhi_event_pkt));
	ring->el_size = sizeof(union mhi_event_pkt);
	ring->overwrite_en = 0;
	/* Flush writes to MMIO */
	wmb();
	return MHI_STATUS_SUCCESS;
}

int init_event_ctxt_array(struct mhi_device_ctxt *mhi_dev_ctxt)
>>>>>>> p9x
{
	int i;
	struct mhi_ring *mhi_local_event_ctxt = NULL;
	struct mhi_event_ctxt *event_ctxt;
<<<<<<< HEAD

	for (i = 0; i < mhi_dev_ctxt->mmio_info.nr_event_rings; ++i) {
		event_ctxt = &mhi_dev_ctxt->dev_space.ring_ctxt.ec_list[i];
		mhi_local_event_ctxt = &mhi_dev_ctxt->mhi_local_event_ctxt[i];
		mhi_event_ring_init(event_ctxt, mhi_local_event_ctxt,
				    mhi_dev_ctxt, i,
				    mhi_dev_ctxt->ev_ring_props[i].nr_desc,
				    mhi_dev_ctxt->ev_ring_props[i].intmod,
				    mhi_dev_ctxt->ev_ring_props[i].msi_vec,
				    GET_EV_PROPS(EV_BRSTMODE,
						 mhi_dev_ctxt->
						 ev_ring_props[i].flags));
	}
=======
	struct mhi_control_seg *mhi_ctrl = mhi_dev_ctxt->mhi_ctrl_seg;

	for (i = 0; i < mhi_dev_ctxt->mmio_info.nr_event_rings; ++i) {
		event_ctxt = &mhi_ctrl->mhi_ec_list[i];
		mhi_local_event_ctxt = &mhi_dev_ctxt->mhi_local_event_ctxt[i];
		mhi_event_ring_init(event_ctxt, mhi_local_event_ctxt,
			mhi_dev_ctxt->ev_ring_props[i].nr_desc,
			mhi_dev_ctxt->ev_ring_props[i].intmod,
			mhi_dev_ctxt->ev_ring_props[i].msi_vec);
	}
	return 0;
>>>>>>> p9x
}

int init_local_ev_ring_by_type(struct mhi_device_ctxt *mhi_dev_ctxt,
		  enum MHI_TYPE_EVENT_RING type)
{
<<<<<<< HEAD
	int ret_val = 0;
	u32 i;

	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Entered\n");
	for (i = 0; i < mhi_dev_ctxt->mmio_info.nr_event_rings; i++) {
		if (GET_EV_PROPS(EV_TYPE,
			mhi_dev_ctxt->ev_ring_props[i].flags) == type) {
=======
	enum MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	u32 i;

	mhi_log(MHI_MSG_INFO, "Entered\n");
	for (i = 0; i < mhi_dev_ctxt->mmio_info.nr_event_rings; i++) {
		if (GET_EV_PROPS(EV_TYPE,
			mhi_dev_ctxt->ev_ring_props[i].flags) == type &&
		    !mhi_dev_ctxt->ev_ring_props[i].state) {
>>>>>>> p9x
			ret_val = mhi_init_local_event_ring(mhi_dev_ctxt,
					mhi_dev_ctxt->ev_ring_props[i].nr_desc,
					i);
			if (ret_val)
				return ret_val;
		}
		ring_ev_db(mhi_dev_ctxt, i);
<<<<<<< HEAD
		mhi_log(mhi_dev_ctxt, MHI_MSG_INFO,
			"Finished ev ring init %d\n", i);
	}
	mhi_log(mhi_dev_ctxt, MHI_MSG_INFO, "Exited\n");
	return 0;
}

int mhi_add_elements_to_event_rings(struct mhi_device_ctxt *mhi_dev_ctxt,
					enum STATE_TRANSITION new_state)
{
	int ret_val = 0;
=======
		mhi_log(MHI_MSG_INFO, "Finished ev ring init %d\n", i);
	}
	mhi_log(MHI_MSG_INFO, "Exited\n");
	return 0;
}

enum MHI_STATUS mhi_add_elements_to_event_rings(
					struct mhi_device_ctxt *mhi_dev_ctxt,
					enum STATE_TRANSITION new_state)
{
	enum MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
>>>>>>> p9x

	switch (new_state) {
	case STATE_TRANSITION_READY:
		ret_val = init_local_ev_ring_by_type(mhi_dev_ctxt,
							MHI_ER_CTRL_TYPE);
		break;
	case STATE_TRANSITION_AMSS:
		ret_val = init_local_ev_ring_by_type(mhi_dev_ctxt,
							MHI_ER_DATA_TYPE);
		break;
	default:
<<<<<<< HEAD
		mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
			"Unrecognized event stage, %d\n", new_state);
		ret_val = -EINVAL;
=======
		mhi_log(MHI_MSG_ERROR,
			"Unrecognized event stage, %d\n", new_state);
		ret_val = MHI_STATUS_ERROR;
>>>>>>> p9x
		break;
	}
	return ret_val;
}

<<<<<<< HEAD
int mhi_init_local_event_ring(struct mhi_device_ctxt *mhi_dev_ctxt,
=======
enum MHI_STATUS mhi_init_local_event_ring(struct mhi_device_ctxt *mhi_dev_ctxt,
>>>>>>> p9x
					u32 nr_ev_el, u32 ring_index)
{
	union mhi_event_pkt *ev_pkt = NULL;
	u32 i = 0;
	unsigned long flags = 0;
<<<<<<< HEAD
	int ret_val = 0;
	struct mhi_ring *event_ctxt =
		&mhi_dev_ctxt->mhi_local_event_ctxt[ring_index];
	spinlock_t *lock = &event_ctxt->ring_lock;

	spin_lock_irqsave(lock, flags);

	mhi_log(mhi_dev_ctxt,
		MHI_MSG_INFO,
		"Initializing event ring %d with %d desc\n",
		ring_index,
		nr_ev_el);

	for (i = 0; i < nr_ev_el - 1; ++i) {
		ret_val = ctxt_add_element(event_ctxt, (void *)&ev_pkt);
		if (0 != ret_val) {
			mhi_log(mhi_dev_ctxt, MHI_MSG_ERROR,
				"Failed to insert el in ev ctxt\n");
			break;
		}
	}
=======
	enum MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	spinlock_t *lock =
		&mhi_dev_ctxt->mhi_ev_spinlock_list[ring_index];
	struct mhi_ring *event_ctxt =
		&mhi_dev_ctxt->mhi_local_event_ctxt[ring_index];

	if (NULL == mhi_dev_ctxt || 0 == nr_ev_el) {
		mhi_log(MHI_MSG_ERROR, "Bad Input data, quitting\n");
		return MHI_STATUS_ERROR;
	}

	spin_lock_irqsave(lock, flags);

	mhi_log(MHI_MSG_INFO, "mmio_addr = 0x%p, mmio_len = 0x%llx\n",
			mhi_dev_ctxt->mmio_info.mmio_addr,
			mhi_dev_ctxt->mmio_info.mmio_len);
	mhi_log(MHI_MSG_INFO, "Initializing event ring %d\n", ring_index);

	for (i = 0; i < nr_ev_el - 1; ++i) {
		ret_val = ctxt_add_element(event_ctxt, (void *)&ev_pkt);
		if (MHI_STATUS_SUCCESS != ret_val) {
			mhi_log(MHI_MSG_ERROR,
				"Failed to insert el in ev ctxt\n");
			ret_val = MHI_STATUS_ERROR;
			break;
		}
	}
	mhi_dev_ctxt->ev_ring_props[ring_index].state = MHI_EVENT_RING_INIT;
>>>>>>> p9x
	spin_unlock_irqrestore(lock, flags);
	return ret_val;
}

void mhi_reset_ev_ctxt(struct mhi_device_ctxt *mhi_dev_ctxt,
				int index)
{
	struct mhi_event_ctxt *ev_ctxt;
	struct mhi_ring *local_ev_ctxt;
<<<<<<< HEAD

	mhi_log(mhi_dev_ctxt, MHI_MSG_VERBOSE,
		"Resetting event index %d\n", index);
	ev_ctxt =
	    &mhi_dev_ctxt->dev_space.ring_ctxt.ec_list[index];
	local_ev_ctxt =
	    &mhi_dev_ctxt->mhi_local_event_ctxt[index];
	spin_lock_irq(&local_ev_ctxt->ring_lock);
=======
	mhi_log(MHI_MSG_VERBOSE, "Resetting event index %d\n", index);
	ev_ctxt =
	    &mhi_dev_ctxt->mhi_ctrl_seg->mhi_ec_list[index];
	local_ev_ctxt =
	    &mhi_dev_ctxt->mhi_local_event_ctxt[index];
>>>>>>> p9x
	ev_ctxt->mhi_event_read_ptr = ev_ctxt->mhi_event_ring_base_addr;
	ev_ctxt->mhi_event_write_ptr = ev_ctxt->mhi_event_ring_base_addr;
	local_ev_ctxt->rp = local_ev_ctxt->base;
	local_ev_ctxt->wp = local_ev_ctxt->base;
<<<<<<< HEAD

	ev_ctxt = &mhi_dev_ctxt->dev_space.ring_ctxt.ec_list[index];
	ev_ctxt->mhi_event_read_ptr = ev_ctxt->mhi_event_ring_base_addr;
	ev_ctxt->mhi_event_write_ptr = ev_ctxt->mhi_event_ring_base_addr;
	spin_unlock_irq(&local_ev_ctxt->ring_lock);
}
=======
	/* Flush writes to MMIO */
	wmb();
}

>>>>>>> p9x
