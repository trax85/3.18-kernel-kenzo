<<<<<<< HEAD
/* Copyright (c) 2012-2019, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
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
 *
 */

<<<<<<< HEAD
#include <asm/dma-iommu.h>
#include <asm/memory.h>
#include <linux/clk/msm-clk.h>
#include <linux/coresight-stm.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/hash.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/smem.h>
#include <soc/qcom/subsystem_restart.h>
#include "hfi_packetization.h"
#include "msm_vidc_debug.h"
#include "venus_hfi.h"
#include "vidc_hfi_io.h"

#define FIRMWARE_SIZE			0X00A00000
#define REG_ADDR_OFFSET_BITMASK	0x000FFFFF
#define QDSS_IOVA_START 0x80001000
=======
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/iommu.h>
#include <linux/qcom_iommu.h>
#include <linux/regulator/consumer.h>
#include <linux/iopoll.h>
#include <linux/coresight-stm.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/smem.h>
#include <asm/memory.h>
#include "hfi_packetization.h"
#include "venus_hfi.h"
#include "vidc_hfi_io.h"
#include "msm_vidc_debug.h"

#define FIRMWARE_SIZE			0X00A00000
#define REG_ADDR_OFFSET_BITMASK	0x000FFFFF
#define VENUS_VERSION_LENGTH 128

#define SHARED_QSIZE 0x1000000
>>>>>>> p9x

static struct hal_device_data hal_ctxt;

#define TZBSP_MEM_PROTECT_VIDEO_VAR 0x8
struct tzbsp_memprot {
	u32 cp_start;
	u32 cp_size;
	u32 cp_nonpixel_start;
	u32 cp_nonpixel_size;
};

struct tzbsp_resp {
	int ret;
};

#define TZBSP_VIDEO_SET_STATE 0xa

/* Poll interval in uS */
#define POLL_INTERVAL_US 50

enum tzbsp_video_state {
	TZBSP_VIDEO_STATE_SUSPEND = 0,
<<<<<<< HEAD
	TZBSP_VIDEO_STATE_RESUME = 1,
	TZBSP_VIDEO_STATE_RESTORE_THRESHOLD = 2,
};

struct tzbsp_video_set_state_req {
	u32 state; /* should be tzbsp_video_state enum value */
	u32 spare; /* reserved for future, should be zero */
};

const struct msm_vidc_gov_data DEFAULT_BUS_VOTE = {
	.data = NULL,
	.data_count = 0,
	.imem_size = 0,
};

const int max_packets = 250;

static void venus_hfi_pm_handler(struct work_struct *work);
static DECLARE_DELAYED_WORK(venus_hfi_pm_work, venus_hfi_pm_handler);
static inline int __resume(struct venus_hfi_device *device);
static inline int __suspend(struct venus_hfi_device *device);
static int __disable_regulators(struct venus_hfi_device *device);
static int __enable_regulators(struct venus_hfi_device *device);
static inline int __prepare_enable_clks(struct venus_hfi_device *device);
static inline void __disable_unprepare_clks(struct venus_hfi_device *device);
static int __scale_clocks_load(struct venus_hfi_device *device, int load,
		struct vidc_clk_scale_data *data,
		unsigned long instant_bitrate);
static void __flush_debug_queue(struct venus_hfi_device *device, u8 *packet);
static int __initialize_packetization(struct venus_hfi_device *device);
static struct hal_session *__get_session(struct venus_hfi_device *device,
		u32 session_id);
static int __iface_cmdq_write(struct venus_hfi_device *device,
					void *pkt);
static int __load_fw(struct venus_hfi_device *device);
static void __unload_fw(struct venus_hfi_device *device);
static int __tzbsp_set_video_state(enum tzbsp_video_state state);


/**
 * Utility function to enforce some of our assumptions.  Spam calls to this
 * in hotspots in code to double check some of the assumptions that we hold.
 */
static inline void __strict_check(struct venus_hfi_device *device)
{
	if (!mutex_is_locked(&device->lock)) {
		dprintk(VIDC_WARN,
			"device->lock mutex is not locked\n");
		WARN_ON(VIDC_DBG_WARN_ENABLE);
	}
}

static inline void __set_state(struct venus_hfi_device *device,
		enum venus_hfi_state state)
{
	device->state = state;
}

static inline bool __core_in_valid_state(struct venus_hfi_device *device)
=======
	TZBSP_VIDEO_STATE_RESUME
};

struct tzbsp_video_set_state_req {
	u32 state; /*shoud be tzbsp_video_state enum value*/
	u32 spare; /*reserved for future, should be zero*/
};

static int venus_hfi_regulator_set_voltage(
	struct venus_hfi_device *device, unsigned long freq,
	struct clock_voltage_info *cv_info);
static void venus_hfi_pm_hndlr(struct work_struct *work);
static DECLARE_DELAYED_WORK(venus_hfi_pm_work, venus_hfi_pm_hndlr);
static int venus_hfi_power_enable(void *dev);
static inline int venus_hfi_power_on(
	struct venus_hfi_device *device);
static int venus_hfi_disable_regulators(struct venus_hfi_device *device);
static int venus_hfi_enable_regulators(struct venus_hfi_device *device);
static inline int venus_hfi_prepare_enable_clks(
	struct venus_hfi_device *device);
static inline void venus_hfi_disable_unprepare_clks(
	struct venus_hfi_device *device);
static void venus_hfi_flush_debug_queue(
	struct venus_hfi_device *device, u8 *packet);
static void venus_hfi_clock_adjust(struct venus_hfi_device *device);

static inline void venus_hfi_set_state(struct venus_hfi_device *device,
		enum venus_hfi_state state)
{
	mutex_lock(&device->write_lock);
	mutex_lock(&device->read_lock);
	device->state = state;
	mutex_unlock(&device->write_lock);
	mutex_unlock(&device->read_lock);
}

static inline bool venus_hfi_core_in_valid_state(
		struct venus_hfi_device *device)
>>>>>>> p9x
{
	return device->state != VENUS_STATE_DEINIT;
}

<<<<<<< HEAD
static void __dump_packet(u8 *packet)
=======
static void venus_hfi_dump_packet(u8 *packet)
>>>>>>> p9x
{
	u32 c = 0, packet_size = *(u32 *)packet;
	const int row_size = 32;
	/* row must contain enough for 0xdeadbaad * 8 to be converted into
	 * "de ad ba ab " * 8 + '\0' */
	char row[3 * row_size];

	for (c = 0; c * row_size < packet_size; ++c) {
		int bytes_to_read = ((c + 1) * row_size > packet_size) ?
			packet_size % row_size : row_size;
		hex_dump_to_buffer(packet + c * row_size, bytes_to_read,
				row_size, 4, row, sizeof(row), false);
		dprintk(VIDC_PKT, "%s\n", row);
	}
}

<<<<<<< HEAD
static void __sim_modify_cmd_packet(u8 *packet, struct venus_hfi_device *device)
=======
static void venus_hfi_sim_modify_cmd_packet(u8 *packet,
				struct venus_hfi_device *device)
>>>>>>> p9x
{
	struct hfi_cmd_sys_session_init_packet *sys_init;
	struct hal_session *session = NULL;
	u8 i;
	phys_addr_t fw_bias = 0;

	if (!device || !packet) {
		dprintk(VIDC_ERR, "Invalid Param\n");
		return;
<<<<<<< HEAD
	} else if (!device->hal_data->firmware_base
=======
	} else if (device->hal_data->firmware_base == 0
>>>>>>> p9x
			|| is_iommu_present(device->res)) {
		return;
	}

	fw_bias = device->hal_data->firmware_base;
	sys_init = (struct hfi_cmd_sys_session_init_packet *)packet;

<<<<<<< HEAD
	session = __get_session(device, sys_init->session_id);
	if (!session) {
		dprintk(VIDC_DBG, "%s :Invalid session id: %x\n",
				__func__, sys_init->session_id);
		return;
	}

=======
	/* Ideally we should acquire device->session_lock. If we acquire
	 * we may go to deadlock with inst->*_lock between two threads.
	 * Ex : in the forward path we acquire inst->internalbufs.lock and
	 * session_lock and in the reverse path, we acquire session_lock and
	 * internalbufs.lock. So this may introduce deadlock. So we are not
	 * doing that. On virtio it is less likely to run two sessions
	 * concurrently. So it should be fine */

	session = hfi_process_get_session(
			&device->sess_head, sys_init->session_id);
	if (!session) {
		dprintk(VIDC_DBG, "%s :Invalid session id : %x\n",
				__func__, sys_init->session_id);
		return;
	}
>>>>>>> p9x
	switch (sys_init->packet_type) {
	case HFI_CMD_SESSION_EMPTY_BUFFER:
		if (session->is_decoder) {
			struct hfi_cmd_session_empty_buffer_compressed_packet
			*pkt = (struct
			hfi_cmd_session_empty_buffer_compressed_packet
			*) packet;
			pkt->packet_buffer -= fw_bias;
		} else {
			struct
			hfi_cmd_session_empty_buffer_uncompressed_plane0_packet
			*pkt = (struct
			hfi_cmd_session_empty_buffer_uncompressed_plane0_packet
			*) packet;
			pkt->packet_buffer -= fw_bias;
		}
		break;
	case HFI_CMD_SESSION_FILL_BUFFER:
	{
		struct hfi_cmd_session_fill_buffer_packet *pkt =
			(struct hfi_cmd_session_fill_buffer_packet *)packet;
		pkt->packet_buffer -= fw_bias;
		break;
	}
	case HFI_CMD_SESSION_SET_BUFFERS:
	{
		struct hfi_cmd_session_set_buffers_packet *pkt =
			(struct hfi_cmd_session_set_buffers_packet *)packet;
<<<<<<< HEAD
		if (pkt->buffer_type == HFI_BUFFER_OUTPUT ||
			pkt->buffer_type == HFI_BUFFER_OUTPUT2) {
=======
		if ((pkt->buffer_type == HFI_BUFFER_OUTPUT) ||
			(pkt->buffer_type == HFI_BUFFER_OUTPUT2)) {
>>>>>>> p9x
			struct hfi_buffer_info *buff;
			buff = (struct hfi_buffer_info *) pkt->rg_buffer_info;
			buff->buffer_addr -= fw_bias;
			if (buff->extra_data_addr >= fw_bias)
				buff->extra_data_addr -= fw_bias;
		} else {
			for (i = 0; i < pkt->num_buffers; i++)
				pkt->rg_buffer_info[i] -= fw_bias;
		}
		break;
	}
	case HFI_CMD_SESSION_RELEASE_BUFFERS:
	{
		struct hfi_cmd_session_release_buffer_packet *pkt =
			(struct hfi_cmd_session_release_buffer_packet *)packet;
<<<<<<< HEAD
		if (pkt->buffer_type == HFI_BUFFER_OUTPUT ||
			pkt->buffer_type == HFI_BUFFER_OUTPUT2) {
=======
		if ((pkt->buffer_type == HFI_BUFFER_OUTPUT) ||
			(pkt->buffer_type == HFI_BUFFER_OUTPUT2)) {
>>>>>>> p9x
			struct hfi_buffer_info *buff;
			buff = (struct hfi_buffer_info *) pkt->rg_buffer_info;
			buff->buffer_addr -= fw_bias;
			buff->extra_data_addr -= fw_bias;
		} else {
			for (i = 0; i < pkt->num_buffers; i++)
				pkt->rg_buffer_info[i] -= fw_bias;
		}
		break;
	}
	case HFI_CMD_SESSION_PARSE_SEQUENCE_HEADER:
	{
		struct hfi_cmd_session_parse_sequence_header_packet *pkt =
			(struct hfi_cmd_session_parse_sequence_header_packet *)
		packet;
		pkt->packet_buffer -= fw_bias;
		break;
	}
	case HFI_CMD_SESSION_GET_SEQUENCE_HEADER:
	{
		struct hfi_cmd_session_get_sequence_header_packet *pkt =
			(struct hfi_cmd_session_get_sequence_header_packet *)
		packet;
		pkt->packet_buffer -= fw_bias;
		break;
	}
	default:
		break;
	}
}

<<<<<<< HEAD
static int __acquire_regulator(struct regulator_info *rinfo)
{
	int rc = 0;

=======
/* Read as "for each 'thing' in a set of 'thingies'" */
#define venus_hfi_for_each_thing(__device, __thing, __thingy) \
	for (__thing = &(__device)->res->__thingy##_set.__thingy##_tbl[0]; \
		__thing < &(__device)->res->__thingy##_set.__thingy##_tbl[0] + \
			(__device)->res->__thingy##_set.count; \
		++__thing) \

#define venus_hfi_for_each_regulator(__device, __rinfo) \
	venus_hfi_for_each_thing(__device, __rinfo, regulator)

#define venus_hfi_for_each_clock(__device, __cinfo) \
	venus_hfi_for_each_thing(__device, __cinfo, clock)

#define venus_hfi_for_each_bus(__device, __binfo) \
	venus_hfi_for_each_thing(__device, __binfo, bus)

static int venus_hfi_acquire_regulator(struct regulator_info *rinfo)
{
	int rc = 0;

	dprintk(VIDC_DBG,
		"Acquire regulator control from HW: %s\n", rinfo->name);

>>>>>>> p9x
	if (rinfo->has_hw_power_collapse) {
		rc = regulator_set_mode(rinfo->regulator,
				REGULATOR_MODE_NORMAL);
		if (rc) {
			/*
			* This is somewhat fatal, but nothing we can do
			* about it. We can't disable the regulator w/o
			* getting it back under s/w control
			*/
			dprintk(VIDC_WARN,
<<<<<<< HEAD
				"Failed to acquire regulator control: %s\n",
					rinfo->name);
		} else {

			dprintk(VIDC_DBG,
					"Acquire regulator control from HW: %s\n",
					rinfo->name);

		}
	}

	if (!regulator_is_enabled(rinfo->regulator)) {
		dprintk(VIDC_WARN, "Regulator is not enabled %s\n",
			rinfo->name);
		WARN_ON(VIDC_DBG_WARN_ENABLE);
	}

	return rc;
}

static int __hand_off_regulator(struct regulator_info *rinfo)
{
	int rc = 0;

	if (rinfo->has_hw_power_collapse) {
		rc = regulator_set_mode(rinfo->regulator,
				REGULATOR_MODE_FAST);
		if (rc) {
			dprintk(VIDC_WARN,
				"Failed to hand off regulator control: %s\n",
					rinfo->name);
		} else {
			dprintk(VIDC_DBG,
					"Hand off regulator control to HW: %s\n",
					rinfo->name);
		}
	}

	return rc;
}

static int __hand_off_regulators(struct venus_hfi_device *device)
=======
				"Failed to acquire regulator control : %s\n",
					rinfo->name);
		}
	}
	WARN_ON(!regulator_is_enabled(rinfo->regulator) && (msm_vidc_debug & VIDC_INFO));
	return rc;
}

static int venus_hfi_hand_off_regulator(struct regulator_info *rinfo)
{
	int rc = 0;

	dprintk(VIDC_DBG,
		"Hand off regulator control to HW: %s\n", rinfo->name);

	if (rinfo->has_hw_power_collapse) {
		rc = regulator_set_mode(rinfo->regulator,
				REGULATOR_MODE_FAST);
		if (rc)
			dprintk(VIDC_WARN,
				"Failed to hand off regulator control : %s\n",
					rinfo->name);
	}
	return rc;
}

static int venus_hfi_hand_off_regulators(struct venus_hfi_device *device)
>>>>>>> p9x
{
	struct regulator_info *rinfo;
	int rc = 0, c = 0;

	venus_hfi_for_each_regulator(device, rinfo) {
<<<<<<< HEAD
		rc = __hand_off_regulator(rinfo);
=======
		rc = venus_hfi_hand_off_regulator(rinfo);
>>>>>>> p9x
		/*
		* If one regulator hand off failed, driver should take
		* the control for other regulators back.
		*/
		if (rc)
			goto err_reg_handoff_failed;
		c++;
	}

	return rc;
err_reg_handoff_failed:
<<<<<<< HEAD
	venus_hfi_for_each_regulator_reverse_continue(device, rinfo, c)
		__acquire_regulator(rinfo);
=======
	venus_hfi_for_each_regulator(device, rinfo) {
		if (!c)
			break;

		venus_hfi_acquire_regulator(rinfo);
		--c;
	}
>>>>>>> p9x

	return rc;
}

<<<<<<< HEAD
static int __write_queue(struct vidc_iface_q_info *qinfo, u8 *packet,
		bool *rx_req_is_set)
{
	struct hfi_queue_header *queue;
	u32 packet_size_in_words, new_write_idx;
	u32 empty_space, read_idx, write_idx;
	u32 *write_ptr;

	if (!qinfo || !packet) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	} else if (!qinfo->q_array.align_virtual_addr) {
=======
static int venus_hfi_acquire_regulators(struct venus_hfi_device *device)
{
	int rc = 0;
	struct regulator_info *rinfo;

	dprintk(VIDC_DBG, "Enabling regulators\n");

	venus_hfi_for_each_regulator(device, rinfo) {
		if (rinfo->has_hw_power_collapse) {
			/*
			 * Once driver has the control, it restores the
			 * previous state of regulator. Hence driver no
			 * need to call regulator_enable for these.
			 */
			rc = venus_hfi_acquire_regulator(rinfo);
			if (rc) {
				dprintk(VIDC_WARN,
						"Failed: Aqcuire control: %s\n",
						rinfo->name);
				break;
			}
		}
	}
	return rc;
}

static int venus_hfi_write_queue(void *info, u8 *packet, u32 *rx_req_is_set)
{
	struct hfi_queue_header *queue;
	u32 packet_size_in_words, new_write_idx;
	struct vidc_iface_q_info *qinfo;
	u32 empty_space, read_idx;
	u32 *write_ptr;

	if (!info || !packet || !rx_req_is_set) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}

	qinfo =	(struct vidc_iface_q_info *) info;
	if (!qinfo || !qinfo->q_array.align_virtual_addr) {
>>>>>>> p9x
		dprintk(VIDC_WARN, "Queues have already been freed\n");
		return -EINVAL;
	}

	queue = (struct hfi_queue_header *) qinfo->q_hdr;
<<<<<<< HEAD
=======

>>>>>>> p9x
	if (!queue) {
		dprintk(VIDC_ERR, "queue not present\n");
		return -ENOENT;
	}

	if (msm_vidc_debug & VIDC_PKT) {
		dprintk(VIDC_PKT, "%s: %pK\n", __func__, qinfo);
<<<<<<< HEAD
		__dump_packet(packet);
	}

	packet_size_in_words = (*(u32 *)packet) >> 2;
	if (!packet_size_in_words || packet_size_in_words >
		qinfo->q_array.mem_size>>2) {
		dprintk(VIDC_ERR, "Invalid packet size\n");
=======
		venus_hfi_dump_packet(packet);
	}

	packet_size_in_words = (*(u32 *)packet) >> 2;
	if (packet_size_in_words == 0) {
		dprintk(VIDC_ERR, "Zero packet size\n");
>>>>>>> p9x
		return -ENODATA;
	}

	read_idx = queue->qhdr_read_idx;
<<<<<<< HEAD
	write_idx = queue->qhdr_write_idx;

	empty_space = (write_idx >=  read_idx) ?
		((qinfo->q_array.mem_size>>2) - (write_idx -  read_idx)) :
		(read_idx - write_idx);
=======

	empty_space = (queue->qhdr_write_idx >=  read_idx) ?
		(queue->qhdr_q_size - (queue->qhdr_write_idx -  read_idx)) :
		(read_idx - queue->qhdr_write_idx);
>>>>>>> p9x
	if (empty_space <= packet_size_in_words) {
		queue->qhdr_tx_req =  1;
		dprintk(VIDC_ERR, "Insufficient size (%d) to write (%d)\n",
					  empty_space, packet_size_in_words);
		return -ENOTEMPTY;
	}

	queue->qhdr_tx_req =  0;

<<<<<<< HEAD
	new_write_idx = write_idx + packet_size_in_words;
	write_ptr = (u32 *)((qinfo->q_array.align_virtual_addr) +
			(write_idx << 2));
	if (write_ptr < (u32 *)qinfo->q_array.align_virtual_addr ||
	    write_ptr > (u32 *)(qinfo->q_array.align_virtual_addr +
	    qinfo->q_array.mem_size)) {
		dprintk(VIDC_ERR, "Invalid write index");
		return -ENODATA;
	}

	if (new_write_idx < (qinfo->q_array.mem_size >> 2)) {
		memcpy(write_ptr, packet, packet_size_in_words << 2);
	} else {
		new_write_idx -= qinfo->q_array.mem_size >> 2;
=======
	new_write_idx = (queue->qhdr_write_idx + packet_size_in_words);
	write_ptr = (u32 *)((qinfo->q_array.align_virtual_addr) +
		(queue->qhdr_write_idx << 2));
	if (new_write_idx < queue->qhdr_q_size) {
		memcpy(write_ptr, packet, packet_size_in_words << 2);
	} else {
		new_write_idx -= queue->qhdr_q_size;
>>>>>>> p9x
		memcpy(write_ptr, packet, (packet_size_in_words -
			new_write_idx) << 2);
		memcpy((void *)qinfo->q_array.align_virtual_addr,
			packet + ((packet_size_in_words - new_write_idx) << 2),
			new_write_idx  << 2);
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	/* Memory barrier to make sure packet is written before updating the
	 * write index */
	mb();
	queue->qhdr_write_idx = new_write_idx;
<<<<<<< HEAD
	if (rx_req_is_set)
		*rx_req_is_set = queue->qhdr_rx_req == 1;
	/* Memory barrier to make sure write index is updated before an
	 * interrupt is raised on venus. */
=======
	*rx_req_is_set = (1 == queue->qhdr_rx_req) ? 1 : 0;
	/*Memory barrier to make sure write index is updated before an
	 * interupt is raised on venus.*/
>>>>>>> p9x
	mb();
	return 0;
}

<<<<<<< HEAD
static void __hal_sim_modify_msg_packet(u8 *packet,
=======
static void venus_hfi_hal_sim_modify_msg_packet(u8 *packet,
>>>>>>> p9x
					struct venus_hfi_device *device)
{
	struct hfi_msg_sys_session_init_done_packet *sys_idle;
	struct hal_session *session = NULL;
	phys_addr_t fw_bias = 0;

	if (!device || !packet) {
		dprintk(VIDC_ERR, "Invalid Param\n");
		return;
<<<<<<< HEAD
	} else if (!device->hal_data->firmware_base
=======
	} else if (device->hal_data->firmware_base == 0
>>>>>>> p9x
			|| is_iommu_present(device->res)) {
		return;
	}

	fw_bias = device->hal_data->firmware_base;
	sys_idle = (struct hfi_msg_sys_session_init_done_packet *)packet;
<<<<<<< HEAD
	session = __get_session(device, sys_idle->session_id);

	if (!session) {
		dprintk(VIDC_DBG, "%s: Invalid session id: %x\n",
				__func__, sys_idle->session_id);
		return;
	}

=======
	if (&device->session_lock) {
		mutex_lock(&device->session_lock);
		session = hfi_process_get_session(
				&device->sess_head, sys_idle->session_id);
		mutex_unlock(&device->session_lock);
	}
	if (!session) {
		dprintk(VIDC_DBG, "%s: Invalid session id : %x\n",
				__func__, sys_idle->session_id);
		return;
	}
>>>>>>> p9x
	switch (sys_idle->packet_type) {
	case HFI_MSG_SESSION_FILL_BUFFER_DONE:
		if (session->is_decoder) {
			struct
			hfi_msg_session_fbd_uncompressed_plane0_packet
			*pkt_uc = (struct
			hfi_msg_session_fbd_uncompressed_plane0_packet
			*) packet;
			pkt_uc->packet_buffer += fw_bias;
		} else {
			struct
			hfi_msg_session_fill_buffer_done_compressed_packet
			*pkt = (struct
			hfi_msg_session_fill_buffer_done_compressed_packet
			*) packet;
			pkt->packet_buffer += fw_bias;
		}
		break;
	case HFI_MSG_SESSION_EMPTY_BUFFER_DONE:
	{
		struct hfi_msg_session_empty_buffer_done_packet *pkt =
		(struct hfi_msg_session_empty_buffer_done_packet *)packet;
		pkt->packet_buffer += fw_bias;
		break;
	}
	case HFI_MSG_SESSION_GET_SEQUENCE_HEADER_DONE:
	{
		struct
		hfi_msg_session_get_sequence_header_done_packet
		*pkt =
		(struct hfi_msg_session_get_sequence_header_done_packet *)
		packet;
		pkt->sequence_header += fw_bias;
		break;
	}
	default:
		break;
	}
}

<<<<<<< HEAD
static int __read_queue(struct vidc_iface_q_info *qinfo, u8 *packet,
		u32 *pb_tx_req_is_set)
=======
static int venus_hfi_read_queue(void *info, u8 *packet, u32 *pb_tx_req_is_set)
>>>>>>> p9x
{
	struct hfi_queue_header *queue;
	u32 packet_size_in_words, new_read_idx;
	u32 *read_ptr;
	u32 receive_request = 0;
<<<<<<< HEAD
	u32 read_idx, write_idx;
	int rc = 0;

	if (!qinfo || !packet || !pb_tx_req_is_set) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	} else if (!qinfo->q_array.align_virtual_addr) {
		dprintk(VIDC_WARN, "Queues have already been freed\n");
		return -EINVAL;
	}

=======
	struct vidc_iface_q_info *qinfo;
	int rc = 0;

	if (!info || !packet || !pb_tx_req_is_set) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}

	qinfo = (struct vidc_iface_q_info *) info;
	if (!qinfo || !qinfo->q_array.align_virtual_addr) {
		dprintk(VIDC_WARN, "Queues have already been freed\n");
		return -EINVAL;
	}
>>>>>>> p9x
	/*Memory barrier to make sure data is valid before
	 *reading it*/
	mb();
	queue = (struct hfi_queue_header *) qinfo->q_hdr;

	if (!queue) {
		dprintk(VIDC_ERR, "Queue memory is not allocated\n");
		return -ENOMEM;
	}

	/*
	 * Do not set receive request for debug queue, if set,
	 * Venus generates interrupt for debug messages even
	 * when there is no response message available.
	 * In general debug queue will not become full as it
	 * is being emptied out for every interrupt from Venus.
	 * Venus will anyway generates interrupt if it is full.
	 */
	if (queue->qhdr_type & HFI_Q_ID_CTRL_TO_HOST_MSG_Q)
		receive_request = 1;

<<<<<<< HEAD
	read_idx = queue->qhdr_read_idx;
	write_idx = queue->qhdr_write_idx;

	if (read_idx == write_idx) {
=======
	if (queue->qhdr_read_idx == queue->qhdr_write_idx) {
>>>>>>> p9x
		queue->qhdr_rx_req = receive_request;
		*pb_tx_req_is_set = 0;
		dprintk(VIDC_DBG,
			"%s queue is empty, rx_req = %u, tx_req = %u, read_idx = %u\n",
			receive_request ? "message" : "debug",
			queue->qhdr_rx_req, queue->qhdr_tx_req,
			queue->qhdr_read_idx);
		return -ENODATA;
	}

	read_ptr = (u32 *)((qinfo->q_array.align_virtual_addr) +
<<<<<<< HEAD
				(read_idx << 2));
	if (read_ptr < (u32 *)qinfo->q_array.align_virtual_addr ||
	    read_ptr > (u32 *)(qinfo->q_array.align_virtual_addr +
	    qinfo->q_array.mem_size - sizeof(*read_ptr))) {
		dprintk(VIDC_ERR, "Invalid read index\n");
		return -ENODATA;
	}

	packet_size_in_words = (*read_ptr) >> 2;
	if (!packet_size_in_words) {
=======
				(queue->qhdr_read_idx << 2));
	packet_size_in_words = (*read_ptr) >> 2;
	if (packet_size_in_words == 0) {
>>>>>>> p9x
		dprintk(VIDC_ERR, "Zero packet size\n");
		return -ENODATA;
	}

<<<<<<< HEAD
	new_read_idx = read_idx + packet_size_in_words;
	if (((packet_size_in_words << 2) <= VIDC_IFACEQ_VAR_HUGE_PKT_SIZE) &&
		read_idx <= (qinfo->q_array.mem_size >> 2)) {
		if (new_read_idx < (qinfo->q_array.mem_size >> 2)) {
			memcpy(packet, read_ptr,
					packet_size_in_words << 2);
		} else {
			new_read_idx -= (qinfo->q_array.mem_size >> 2);
=======
	new_read_idx = queue->qhdr_read_idx + packet_size_in_words;
	if (((packet_size_in_words << 2) <= VIDC_IFACEQ_VAR_HUGE_PKT_SIZE)
			&& queue->qhdr_read_idx <= queue->qhdr_q_size) {
		if (new_read_idx < queue->qhdr_q_size) {
			memcpy(packet, read_ptr,
					packet_size_in_words << 2);
		} else {
			new_read_idx -= queue->qhdr_q_size;
>>>>>>> p9x
			memcpy(packet, read_ptr,
			(packet_size_in_words - new_read_idx) << 2);
			memcpy(packet + ((packet_size_in_words -
					new_read_idx) << 2),
					(u8 *)qinfo->q_array.align_virtual_addr,
					new_read_idx << 2);
		}
	} else {
		dprintk(VIDC_WARN,
<<<<<<< HEAD
			"BAD packet received, read_idx: %#x, pkt_size: %d\n",
			read_idx, packet_size_in_words << 2);
		dprintk(VIDC_WARN, "Dropping this packet\n");
		new_read_idx = write_idx;
		rc = -ENODATA;
	}

	if (new_read_idx != write_idx)
=======
			"BAD packet received, read_idx: 0x%x, pkt_size: %d\n",
			queue->qhdr_read_idx, packet_size_in_words << 2);
		dprintk(VIDC_WARN, "Dropping this packet\n");
		new_read_idx = queue->qhdr_write_idx;
		rc = -ENODATA;
	}

	queue->qhdr_read_idx = new_read_idx;

	if (queue->qhdr_read_idx != queue->qhdr_write_idx)
>>>>>>> p9x
		queue->qhdr_rx_req = 0;
	else
		queue->qhdr_rx_req = receive_request;

<<<<<<< HEAD
	queue->qhdr_read_idx = new_read_idx;

	*pb_tx_req_is_set = (1 == queue->qhdr_tx_req) ? 1 : 0;

	if (msm_vidc_debug & VIDC_PKT) {
		dprintk(VIDC_PKT, "%s: %pK\n", __func__, qinfo);
		__dump_packet(packet);
=======
	*pb_tx_req_is_set = (1 == queue->qhdr_tx_req) ? 1 : 0;

	if ((msm_vidc_debug & VIDC_PKT) &&
		(queue->qhdr_type & HFI_Q_ID_CTRL_TO_HOST_MSG_Q)) {
		dprintk(VIDC_PKT, "%s: %pK\n", __func__, qinfo);
		venus_hfi_dump_packet(packet);
>>>>>>> p9x
	}

	return rc;
}

<<<<<<< HEAD
static int __smem_alloc(struct venus_hfi_device *dev,
			struct vidc_mem_addr *mem, u32 size, u32 align,
			u32 flags, u32 usage)
{
=======
static int venus_hfi_alloc(struct venus_hfi_device *dev, void *mem,
			u32 size, u32 align, u32 flags, u32 usage)
{
	struct vidc_mem_addr *vmem = NULL;
>>>>>>> p9x
	struct msm_smem *alloc = NULL;
	int rc = 0;

	if (!dev || !dev->hal_client || !mem || !size) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}

<<<<<<< HEAD
	dprintk(VIDC_INFO, "start to alloc size: %d, flags: %d\n", size, flags);
	alloc = msm_smem_alloc(dev->hal_client, size, align, flags, usage, 1);
=======
	vmem = (struct vidc_mem_addr *)mem;
	dprintk(VIDC_INFO, "start to alloc: size:%d, Flags: %d\n", size, flags);

	venus_hfi_power_enable(dev);

	alloc = msm_smem_alloc(dev->hal_client, size, align, flags, usage, 1);
	dprintk(VIDC_DBG, "Alloc done\n");
>>>>>>> p9x
	if (!alloc) {
		dprintk(VIDC_ERR, "Alloc failed\n");
		rc = -ENOMEM;
		goto fail_smem_alloc;
	}
<<<<<<< HEAD

	dprintk(VIDC_DBG, "__smem_alloc: ptr = %pK, size = %d\n",
=======
	dprintk(VIDC_DBG, "venus_hfi_alloc: ptr = %pK, size = %d\n",
>>>>>>> p9x
			alloc->kvaddr, size);
	rc = msm_smem_cache_operations(dev->hal_client, alloc,
		SMEM_CACHE_CLEAN);
	if (rc) {
		dprintk(VIDC_WARN, "Failed to clean cache\n");
		dprintk(VIDC_WARN, "This may result in undefined behavior\n");
	}
<<<<<<< HEAD

	mem->mem_size = alloc->size;
	mem->mem_data = alloc;
	mem->align_virtual_addr = alloc->kvaddr;
	mem->align_device_addr = alloc->device_addr;
=======
	vmem->mem_size = alloc->size;
	vmem->mem_data = alloc;
	vmem->align_virtual_addr = alloc->kvaddr;
	vmem->align_device_addr = alloc->device_addr;
>>>>>>> p9x
	return rc;
fail_smem_alloc:
	return rc;
}

<<<<<<< HEAD
static void __smem_free(struct venus_hfi_device *dev, struct msm_smem *mem)
=======
static void venus_hfi_free(struct venus_hfi_device *dev, struct msm_smem *mem)
>>>>>>> p9x
{
	if (!dev || !mem) {
		dprintk(VIDC_ERR, "invalid param %pK %pK\n", dev, mem);
		return;
	}

<<<<<<< HEAD
	msm_smem_free(dev->hal_client, mem);
}

static void __write_register(struct venus_hfi_device *device,
		u32 reg, u32 value)
=======
	if (venus_hfi_power_on(dev))
		dprintk(VIDC_ERR, "%s: Power on failed\n", __func__);

	msm_smem_free(dev->hal_client, mem);
}

static void venus_hfi_write_register(
		struct venus_hfi_device *device, u32 reg, u32 value)
>>>>>>> p9x
{
	u32 hwiosymaddr = reg;
	u8 *base_addr;
	if (!device) {
		dprintk(VIDC_ERR, "Invalid params: %pK\n", device);
		return;
	}
<<<<<<< HEAD

	__strict_check(device);

	if (!device->power_enabled) {
		dprintk(VIDC_WARN,
			"HFI Write register failed : Power is OFF\n");
		WARN_ON(VIDC_DBG_WARN_ENABLE);
=======
	if (device->clk_state != ENABLED_PREPARED) {
		dprintk(VIDC_WARN,
			"HFI Write register failed : Clocks are OFF\n");
>>>>>>> p9x
		return;
	}

	base_addr = device->hal_data->register_base;
<<<<<<< HEAD
	dprintk(VIDC_DBG, "Base addr: %pK, written to: %#x, Value: %#x...\n",
=======
	dprintk(VIDC_DBG, "Base addr: 0x%pK, written to: 0x%x, Value: 0x%x...\n",
>>>>>>> p9x
		base_addr, hwiosymaddr, value);
	base_addr += hwiosymaddr;
	writel_relaxed(value, base_addr);
	wmb();
}

<<<<<<< HEAD
static int __read_register(struct venus_hfi_device *device, u32 reg)
=======
static int venus_hfi_read_register(struct venus_hfi_device *device, u32 reg)
>>>>>>> p9x
{
	int rc = 0;
	u8 *base_addr;
	if (!device) {
		dprintk(VIDC_ERR, "Invalid params: %pK\n", device);
		return -EINVAL;
	}
<<<<<<< HEAD

	__strict_check(device);

	if (!device->power_enabled) {
		dprintk(VIDC_WARN,
			"HFI Read register failed : Power is OFF\n");
		WARN_ON(VIDC_DBG_WARN_ENABLE);
		return -EINVAL;
	}

=======
	if (device->clk_state != ENABLED_PREPARED) {
		dprintk(VIDC_WARN,
			"HFI Read register failed : Clocks are OFF\n");
		return -EINVAL;
	}
>>>>>>> p9x
	base_addr = device->hal_data->register_base;

	rc = readl_relaxed(base_addr + reg);
	rmb();
<<<<<<< HEAD
	dprintk(VIDC_DBG, "Base addr: %pK, read from: %#x, value: %#x...\n",
=======
	dprintk(VIDC_DBG, "Base addr: 0x%pK, read from: 0x%x, value: 0x%x...\n",
>>>>>>> p9x
		base_addr, reg, rc);

	return rc;
}

<<<<<<< HEAD
static void __set_registers(struct venus_hfi_device *device)
=======
static void venus_hfi_set_registers(struct venus_hfi_device *device)
>>>>>>> p9x
{
	struct reg_set *reg_set;
	int i;

	if (!device->res) {
		dprintk(VIDC_ERR,
			"device resources null, cannot set registers\n");
		return;
	}

	reg_set = &device->res->reg_set;
	for (i = 0; i < reg_set->count; i++) {
<<<<<<< HEAD
		__write_register(device, reg_set->reg_tbl[i].reg,
=======
		venus_hfi_write_register(device,
				reg_set->reg_tbl[i].reg,
>>>>>>> p9x
				reg_set->reg_tbl[i].value);
	}
}

<<<<<<< HEAD
/*
 * The existence of this function is a hack for 8996 (or certain Venus versions)
 * to overcome a hardware bug.  Whenever the GDSCs momentarily power collapse
 * (after calling __hand_off_regulators()), the values of the threshold
 * registers (typically programmed by TZ) are incorrectly reset.  As a result
 * reprogram these registers at certain agreed upon points.
 */
static void __set_threshold_registers(struct venus_hfi_device *device)
{
	u32 version = __read_register(device, VIDC_WRAPPER_HW_VERSION);

	version &= ~GENMASK(15, 0);
	if (version != (0x3 << 28 | 0x43 << 16))
		return;

	if (__tzbsp_set_video_state(TZBSP_VIDEO_STATE_RESTORE_THRESHOLD))
		dprintk(VIDC_ERR, "Failed to restore threshold values\n");
}

static void __iommu_detach(struct venus_hfi_device *device)
{
	struct context_bank_info *cb;

	if (!device || !device->res) {
		dprintk(VIDC_ERR, "Invalid parameter: %pK\n", device);
		return;
	}

	list_for_each_entry(cb, &device->res->context_banks, list) {
		if (cb->dev)
			arm_iommu_detach_device(cb->dev);
		if (cb->mapping)
			arm_iommu_release_mapping(cb->mapping);
	}
}

static bool __is_session_supported(unsigned long sessions_supported,
=======
static int venus_hfi_core_start_cpu(struct venus_hfi_device *device)
{
	u32 ctrl_status = 0, count = 0, rc = 0;
	int max_tries = 100;
	venus_hfi_write_register(device,
			VIDC_WRAPPER_INTR_MASK,
			VIDC_WRAPPER_INTR_MASK_A2HVCODEC_BMSK);
	venus_hfi_write_register(device, VIDC_CPU_CS_SCIACMDARG3, 1);

	while (!ctrl_status && count < max_tries) {
		ctrl_status = venus_hfi_read_register(
				device,
				VIDC_CPU_CS_SCIACMDARG0);
		if ((ctrl_status & 0xFE) == 0x4) {
			dprintk(VIDC_ERR, "invalid setting for UC_REGION\n");
			break;
		}
		usleep_range(500, 1000);
		count++;
	}
	if (count >= max_tries)
		rc = -ETIME;
	return rc;
}

static int venus_hfi_iommu_attach(struct venus_hfi_device *device)
{
	int rc = 0;
	struct iommu_domain *domain;
	int i;
	struct iommu_set *iommu_group_set;
	struct iommu_group *group;
	struct iommu_info *iommu_map;

	if (!device || !device->res)
		return -EINVAL;

	iommu_group_set = &device->res->iommu_group_set;
	for (i = 0; i < iommu_group_set->count; i++) {
		iommu_map = &iommu_group_set->iommu_maps[i];
		group = iommu_map->group;
		domain = msm_get_iommu_domain(iommu_map->domain);
		if (IS_ERR_OR_NULL(domain)) {
			dprintk(VIDC_ERR,
				"Failed to get domain: %s\n", iommu_map->name);
			rc = PTR_ERR(domain) ?: -EINVAL;
			break;
		}
		rc = iommu_attach_group(domain, group);
		if (rc) {
			dprintk(VIDC_ERR,
				"IOMMU attach failed: %s\n", iommu_map->name);
			break;
		}
	}
	if (i < iommu_group_set->count) {
		i--;
		for (; i >= 0; i--) {
			iommu_map = &iommu_group_set->iommu_maps[i];
			group = iommu_map->group;
			domain = msm_get_iommu_domain(iommu_map->domain);
			if (group && domain)
				iommu_detach_group(domain, group);
		}
	}
	return rc;
}

static void venus_hfi_iommu_detach(struct venus_hfi_device *device)
{
	struct iommu_group *group;
	struct iommu_domain *domain;
	struct iommu_set *iommu_group_set;
	struct iommu_info *iommu_map;
	int i;

	if (!device || !device->res) {
		dprintk(VIDC_ERR, "Invalid paramter: %pK\n", device);
		return;
	}

	iommu_group_set = &device->res->iommu_group_set;
	for (i = 0; i < iommu_group_set->count; i++) {
		iommu_map = &iommu_group_set->iommu_maps[i];
		group = iommu_map->group;
		domain = msm_get_iommu_domain(iommu_map->domain);
		if (group && domain)
			iommu_detach_group(domain, group);
	}
}

#define BUS_LOAD(__w, __h, __fps) (\
	/* Something's fishy if the width & height aren't macroblock aligned */\
	BUILD_BUG_ON_ZERO(!IS_ALIGNED(__h, 16) || !IS_ALIGNED(__w, 16)) ?: \
	(__h >> 4) * (__w >> 4) * __fps)

static const u32 venus_hfi_bus_table[] = {
	BUS_LOAD(0, 0, 0),
	BUS_LOAD(640, 480, 30),
	BUS_LOAD(640, 480, 60),
	BUS_LOAD(1280, 736, 30),
	BUS_LOAD(1280, 736, 60),
	BUS_LOAD(1920, 1088, 30),
	BUS_LOAD(1920, 1088, 60),
	BUS_LOAD(3840, 2176, 24),
	BUS_LOAD(4096, 2176, 24),
	BUS_LOAD(3840, 2176, 30),
};

static int venus_hfi_get_bus_vector(struct venus_hfi_device *device,
		struct bus_info *bus, int load)
{
	int num_rows = ARRAY_SIZE(venus_hfi_bus_table);
	int i, j;

	for (i = 0; i < num_rows; i++) {
		if (load <= venus_hfi_bus_table[i])
			break;
	}

	j = clamp(i, 0, num_rows - 1);

	/* Ensure bus index remains within the supported range,
	* as specified in the device dtsi file */
	j = clamp(j, 0, bus->pdata->num_usecases - 1);

	dprintk(VIDC_DBG, "Required bus = %d\n", j);
	return j;
}

static bool venus_hfi_is_session_supported(unsigned long sessions_supported,
>>>>>>> p9x
		enum vidc_vote_data_session session_type)
{
	bool same_codec, same_session_type;
	int codec_bit, session_type_bit;
	unsigned long session = session_type;

	if (!sessions_supported || !session)
		return false;

	/* ffs returns a 1 indexed, test_bit takes a 0 indexed...index */
	codec_bit = ffs(session) - 1;
	session_type_bit = codec_bit + 1;

	same_codec = test_bit(codec_bit, &sessions_supported) ==
		test_bit(codec_bit, &session);
	same_session_type = test_bit(session_type_bit, &sessions_supported) ==
		test_bit(session_type_bit, &session);

	return same_codec && same_session_type;
}

<<<<<<< HEAD
bool venus_hfi_is_session_supported(unsigned long sessions_supported,
		enum vidc_vote_data_session session_type)
{
	return __is_session_supported(sessions_supported, session_type);
}

static int __devfreq_target(struct device *devfreq_dev,
		unsigned long *freq, u32 flags)
{
	int rc = 0;
	uint64_t ab = 0;
	struct bus_info *bus = NULL, *temp = NULL;
	struct venus_hfi_device *device = dev_get_drvdata(devfreq_dev);

	venus_hfi_for_each_bus(device, temp) {
		if (temp->dev == devfreq_dev) {
			bus = temp;
			break;
		}
	}

	if (!bus) {
		rc = -EBADHANDLE;
		goto err_unknown_device;
	}

	/*
	 * Clamp for all non zero frequencies. This clamp is necessary to stop
	 * devfreq driver from spamming - Couldn't update frequency - logs, if
	 * the scaled ab value is not part of the frequency table.
	 */
	if (*freq)
		*freq = clamp_t(typeof(*freq), *freq, bus->range[0],
				bus->range[1]);

	/* we expect governors to provide values in kBps form, convert to Bps */
	ab = *freq * 1000;
	rc = msm_bus_scale_update_bw(bus->client, ab, 0);
	if (rc) {
		dprintk(VIDC_ERR, "Failed voting bus %s to ab %llu\n: %d",
				bus->name, ab, rc);
		goto err_unknown_device;
	}

	dprintk(VIDC_PROF, "Voting bus %s to ab %llu\n", bus->name, ab);

	return 0;
err_unknown_device:
	return rc;
}

static int __devfreq_get_status(struct device *devfreq_dev,
		struct devfreq_dev_status *stat)
{
	int rc = 0;
	struct bus_info *bus = NULL, *temp = NULL;
	struct venus_hfi_device *device = dev_get_drvdata(devfreq_dev);

	venus_hfi_for_each_bus(device, temp) {
		if (temp->dev == devfreq_dev) {
			bus = temp;
			break;
		}
	}

	if (!bus) {
		rc = -EBADHANDLE;
		goto err_unknown_device;
	}

	*stat = (struct devfreq_dev_status) {
		.private_data = &device->bus_vote,
		/*
		 * Put in dummy place holder values for upstream govs, our
		 * custom gov only needs .private_data.  We should fill this in
		 * properly if we can actually measure busy_time accurately
		 * (which we can't at the moment)
		 */
		.total_time = 1,
		.busy_time = 1,
		.current_frequency = 0,
	};

err_unknown_device:
	return rc;
}

static int __unvote_buses(struct venus_hfi_device *device)
{
	int rc = 0;
	struct bus_info *bus = NULL;

	venus_hfi_for_each_bus(device, bus) {
		int local_rc = 0;
		unsigned long zero = 0;

		rc = devfreq_suspend_device(bus->devfreq);
		if (rc)
			goto err_unknown_device;

		local_rc = __devfreq_target(bus->dev, &zero, 0);
		rc = rc ?: local_rc;
	}

	if (rc)
		dprintk(VIDC_WARN, "Failed to unvote some buses\n");

err_unknown_device:
	return rc;
}

static int __vote_buses(struct venus_hfi_device *device,
		struct vidc_bus_vote_data *data, int num_data)
{
	int rc = 0;
	struct bus_info *bus = NULL;
	struct vidc_bus_vote_data *new_data = NULL;

	if (!num_data) {
		dprintk(VIDC_DBG, "No vote data available\n");
		goto no_data_count;
=======
static int venus_hfi_vote_bus(struct bus_info *bus, unsigned int bus_vector)
{
	int rc = msm_bus_scale_client_update_request(bus->priv, bus_vector);
	if (!rc) {
		dprintk(VIDC_PROF, "%s bus %s (%s) to vector %d\n",
				bus_vector ? "Voting" : "Unvoting",
				bus->pdata->name,
				bus->passive ? "passive" : "active",
				bus_vector);
	}

	return rc;
}

static int venus_hfi_vote_passive_buses(void *dev,
		struct vidc_bus_vote_data *data, int num_data)
{
	struct venus_hfi_device *device = dev;
	struct bus_info *bus = NULL;
	int rc = 0;

	/*
	 * Neither of these parameters are used (or will be useful in future).
	 * Just keeping these so that the API is consistent with _vote_active\
	 * _buses().
	 */
	(void)data;
	(void)num_data;

	venus_hfi_for_each_bus(device, bus) {
		/* Reject active buses, as those are driven by instance load */
		if (!bus->passive)
			continue;

		/*
		 * XXX: Should probably check *_is_session_supported() prior
		 * to voting but probably overkill at this point.  So skip the
		 * check for now.
		 */
		rc = venus_hfi_vote_bus(bus, 1);
		if (rc) {
			dprintk(VIDC_ERR,
					"Failed voting for passive bus %s: %d\n",
					bus->pdata->name, rc);
			goto vote_fail;
		}
	}

vote_fail:
	return rc;
}

static int venus_hfi_vote_active_buses(void *dev,
		struct vidc_bus_vote_data *data, int num_data)
{
	struct {
		struct bus_info *bus;
		int load;
	} *aggregate_load_table;
	int rc = 0, i = 0, num_bus = 0;
	struct venus_hfi_device *device = dev;
	struct bus_info *bus = NULL;
	struct vidc_bus_vote_data *cached_vote_data = NULL;

	if (!dev) {
		dprintk(VIDC_ERR, "Invalid device\n");
		return -EINVAL;
	} else if (!num_data) {
		/* Meh nothing to do */
		return 0;
>>>>>>> p9x
	} else if (!data) {
		dprintk(VIDC_ERR, "Invalid voting data\n");
		return -EINVAL;
	}

<<<<<<< HEAD
	new_data = kmemdup(data, num_data * sizeof(*new_data), GFP_KERNEL);
	if (!new_data) {
		dprintk(VIDC_ERR, "Can't alloc memory to cache bus votes\n");
=======
        cached_vote_data = device->bus_load.vote_data;
        if (!cached_vote_data) {
                dprintk(VIDC_ERR,"Invalid bus load vote data\n");
                rc = -ENOMEM;
                goto err_no_mem;
        }

	/* Alloc & init the load table */
	num_bus = device->res->bus_set.count;
	aggregate_load_table = kzalloc(sizeof(*aggregate_load_table) * num_bus,
			GFP_TEMPORARY);
	if (!aggregate_load_table) {
		dprintk(VIDC_ERR, "The world is ending (no more memory)\n");
>>>>>>> p9x
		rc = -ENOMEM;
		goto err_no_mem;
	}

<<<<<<< HEAD
no_data_count:
	kfree(device->bus_vote.data);
	device->bus_vote.data = new_data;
	device->bus_vote.data_count = num_data;
	device->bus_vote.imem_size = device->res->imem_size;

	venus_hfi_for_each_bus(device, bus) {
		if (bus && bus->devfreq) {
			/* NOP if already resume */
			rc = devfreq_resume_device(bus->devfreq);
			if (rc)
				goto err_no_mem;

			/* Kick devfreq awake incase _resume() didn't do it */
			bus->devfreq->nb.notifier_call(
				&bus->devfreq->nb, 0, NULL);
		}
	}

err_no_mem:
	return rc;
}

static int venus_hfi_vote_buses(void *dev, struct vidc_bus_vote_data *d, int n)
{
	int rc = 0;
	struct venus_hfi_device *device = dev;

	if (!device)
		return -EINVAL;

	mutex_lock(&device->lock);
	rc = __vote_buses(device, d, n);
	mutex_unlock(&device->lock);

	return rc;

}
static int __core_set_resource(struct venus_hfi_device *device,
		struct vidc_resource_hdr *resource_hdr, void *resource_value)
=======
	i = 0;
	venus_hfi_for_each_bus(device, bus)
		aggregate_load_table[i++].bus = bus;

	/* Aggregate the loads for each bus */
	for (i = 0; i < num_data; ++i) {
		int j = 0;

		for (j = 0; j < num_bus; ++j) {
			bool matches = venus_hfi_is_session_supported(
					aggregate_load_table[j].bus->
						sessions_supported,
					data[i].session);
			/*
			 * VIDC_POWER_NORMAL will be default power mode.
			 * Amend matches variable only if client supplied
			 * power mode is available in the dtsi, if not
			 * avaialable then default power mode (NORMAL)
			 * bus vectors will be picked up.
			 */
			if (matches) {
				if (device->res->power_modes &
						data[i].power_mode) {
					/*
					 * if bus supported power mode is
					 * not the client power mode then
					 * skip voting for the bus.
					 */
					if (!(aggregate_load_table[j].bus->
						power_mode &
						data[i].power_mode))
						matches = false;
				} else {
				    /*
				     * this power mode is not supported by the
				     * chipset, so we need to vote for normal
				     * bus vector only.
				     */
					if (!(aggregate_load_table[j].bus->
						power_mode ==
						VIDC_POWER_NORMAL))
						matches = false;
				}
			}
			if (matches) {
				aggregate_load_table[j].load +=
					data[i].load;
			}
		}
	}

	/* Now vote for each bus */
	for (i = 0; i < num_bus; ++i) {
		int bus_vector = 0;
		struct bus_info *bus = aggregate_load_table[i].bus;
		int load = aggregate_load_table[i].load;

		/* Passive buses aren't meant to be scaled by load */
		if (bus->passive)
			continue;

		/* Let's avoid voting for ocmem if allocation failed.
		 * There's no clean way presently to check which buses are
		 * associated with ocmem. So do a crude check for the bus name,
		 * which relies on the buses being named appropriately. */
		if (!device->resources.ocmem.buf && strnstr(bus->pdata->name,
					"ocmem", strlen(bus->pdata->name))) {
			dprintk(VIDC_DBG, "Skipping voting for %s (no ocmem)\n",
					bus->pdata->name);
			continue;
		}

		bus_vector = venus_hfi_get_bus_vector(device, bus, load);
		rc = venus_hfi_vote_bus(bus, bus_vector);
		if (rc) {
			dprintk(VIDC_ERR, "Failed voting for bus %s @ %d: %d\n",
					bus->pdata->name, bus_vector, rc);
			/* Ignore error and try to vote for the rest */
			rc = 0;
		}
	}

	/* Cache the votes */
	for (i = 0; i < num_data; ++i)
		cached_vote_data[i] = data[i];

	device->bus_load.vote_data = cached_vote_data;
	device->bus_load.vote_data_count = num_data;

	kfree(aggregate_load_table);
err_no_mem:
	return rc;

}

static int venus_hfi_unvote_buses_of_type(struct venus_hfi_device *device,
		bool only_passive)
{
	struct bus_info *bus = NULL;
	int rc = 0;

	venus_hfi_for_each_bus(device, bus) {
		int local_rc = 0;

		if (bus->passive != only_passive)
			continue;

		local_rc = venus_hfi_vote_bus(bus, 0);
		if (local_rc) {
			rc = rc ?: local_rc;
			dprintk(VIDC_ERR,
					"Failed unvoting passive bus %s: %d\n",
					bus->pdata->name, rc);
		}
	}

	return rc;
}

static int venus_hfi_unvote_passive_buses(void *dev)
{
	return venus_hfi_unvote_buses_of_type(dev, true);
}

static int venus_hfi_unvote_active_buses(void *dev)
{
	return venus_hfi_unvote_buses_of_type(dev, false);
}

static int venus_hfi_unvote_buses(void *dev)
{
	venus_hfi_unvote_active_buses(dev);
	venus_hfi_unvote_passive_buses(dev);

	return 0;
}

static int venus_hfi_vote_buses(void *dev,
		struct vidc_bus_vote_data *data, int num_data)
{
	int rc = venus_hfi_vote_passive_buses(dev, data, num_data);
	rc = rc ?: venus_hfi_vote_active_buses(dev, data, num_data);

	if (rc)
		goto fail_vote;

	return 0;
fail_vote:
	venus_hfi_unvote_buses(dev);
	return rc;
}

static int venus_hfi_iface_cmdq_write_nolock(struct venus_hfi_device *device,
					void *pkt);

static int venus_hfi_iface_cmdq_write(struct venus_hfi_device *device,
					void *pkt)
{
	int result = -EPERM;
	if (!device || !pkt) {
		dprintk(VIDC_ERR, "Invalid Params");
		return -EINVAL;
	}

	if (device->res->sw_power_collapsible) {
		dprintk(VIDC_DBG,
			"Cancel and queue delayed work from %s\n",
			__func__);
		cancel_delayed_work_sync(&venus_hfi_pm_work);
		if (!queue_delayed_work(device->venus_pm_workq,
				&venus_hfi_pm_work,
				msecs_to_jiffies(
					msm_vidc_pwr_collapse_delay))) {
			dprintk(VIDC_DBG,
				"PM work already scheduled\n");
		}
	}

	mutex_lock(&device->write_lock);
	result = venus_hfi_iface_cmdq_write_nolock(device, pkt);
	mutex_unlock(&device->write_lock);
	return result;
}

static int venus_hfi_core_set_resource(void *device,
		struct vidc_resource_hdr *resource_hdr, void *resource_value,
		bool locked)
>>>>>>> p9x
{
	struct hfi_cmd_sys_set_resource_packet *pkt;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	int rc = 0;
<<<<<<< HEAD
=======
	struct venus_hfi_device *dev;
>>>>>>> p9x

	if (!device || !resource_hdr || !resource_value) {
		dprintk(VIDC_ERR, "set_res: Invalid Params\n");
		return -EINVAL;
<<<<<<< HEAD
=======
	} else {
		dev = device;
>>>>>>> p9x
	}

	pkt = (struct hfi_cmd_sys_set_resource_packet *) packet;

<<<<<<< HEAD
	rc = call_hfi_pkt_op(device, sys_set_resource,
=======
	rc = call_hfi_pkt_op(dev, sys_set_resource,
>>>>>>> p9x
			pkt, resource_hdr, resource_value);
	if (rc) {
		dprintk(VIDC_ERR, "set_res: failed to create packet\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	rc = __iface_cmdq_write(device, pkt);
=======
	rc = locked ? venus_hfi_iface_cmdq_write(dev, pkt) :
			venus_hfi_iface_cmdq_write_nolock(dev, pkt);
>>>>>>> p9x
	if (rc)
		rc = -ENOTEMPTY;

err_create_pkt:
	return rc;
}

<<<<<<< HEAD
static int __alloc_imem(struct venus_hfi_device *device, unsigned long size)
{
	struct imem *imem = NULL;
	int rc = 0;

	if (!device)
		return -EINVAL;

	imem = &device->resources.imem;
	if (imem->type) {
		dprintk(VIDC_ERR, "IMEM of type %d already allocated\n",
				imem->type);
		return -ENOMEM;
	}

	switch (device->res->imem_type) {
	case IMEM_VMEM:
	{
		phys_addr_t vmem_buffer = 0;

		rc = vmem_allocate(size, &vmem_buffer);
		if (rc) {
			if (rc == -ENOTSUPP) {
				dprintk(VIDC_DBG,
					"Target does not support vmem\n");
				rc = 0;
			}
			goto imem_alloc_failed;
		} else if (!vmem_buffer) {
			rc = -ENOMEM;
			goto imem_alloc_failed;
		}

		imem->vmem = vmem_buffer;
		break;
	}
	case IMEM_NONE:
		rc = 0;
		break;

	default:
		rc = -ENOTSUPP;
		goto imem_alloc_failed;
	}

	imem->type = device->res->imem_type;
	dprintk(VIDC_DBG, "Allocated %ld bytes of IMEM of type %d\n", size,
			imem->type);
	return 0;
imem_alloc_failed:
	imem->type = IMEM_NONE;
	return rc;
}

static int __free_imem(struct venus_hfi_device *device)
{
	struct imem *imem = NULL;
	int rc = 0;

	if (!device)
		return -EINVAL;

	imem = &device->resources.imem;
	switch (imem->type) {
	case IMEM_NONE:
		/* Follow the semantics of free(NULL), which is a no-op. */
		break;
	case IMEM_VMEM:
		vmem_free(imem->vmem);
		break;
	default:
		rc = -ENOTSUPP;
		goto imem_free_failed;
	}

	imem->type = IMEM_NONE;
	return 0;

imem_free_failed:
	return rc;
}

static int __set_imem(struct venus_hfi_device *device, struct imem *imem)
{
	struct vidc_resource_hdr rhdr;
	phys_addr_t addr = 0;
	int rc = 0;

	if (!device || !device->res || !imem) {
		dprintk(VIDC_ERR, "Invalid params, core: %pK, imem: %pK\n",
			device, imem);
		return -EINVAL;
	}

	rhdr.resource_handle = imem; /* cookie */
	rhdr.size = device->res->imem_size;
	rhdr.resource_id = VIDC_RESOURCE_NONE;

	switch (imem->type) {
	case IMEM_VMEM:
		rhdr.resource_id = VIDC_RESOURCE_VMEM;
		addr = imem->vmem;
		break;
	case IMEM_NONE:
		dprintk(VIDC_DBG, "%s Target does not support IMEM", __func__);
		rc = 0;
		goto imem_set_failed;
	default:
		dprintk(VIDC_ERR, "IMEM of type %d unsupported\n", imem->type);
		rc = -ENOTSUPP;
		goto imem_set_failed;
	}

	BUG_ON(!addr);

	rc = __core_set_resource(device, &rhdr, (void *)addr);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to set IMEM on driver\n");
		goto imem_set_failed;
	}

	dprintk(VIDC_DBG,
			"Managed to set IMEM buffer of type %d sized %d bytes at %pa\n",
			rhdr.resource_id, rhdr.size, &addr);

	rc = __vote_buses(device, device->bus_vote.data,
			device->bus_vote.data_count);
	if (rc) {
		dprintk(VIDC_ERR,
				"Failed to vote for buses after setting imem: %d\n",
				rc);
	}

imem_set_failed:
	return rc;
}

static int __tzbsp_set_video_state(enum tzbsp_video_state state)
=======
static int venus_hfi_core_release_resource(void *device,
			struct vidc_resource_hdr *resource_hdr)
{
	struct hfi_cmd_sys_release_resource_packet pkt;
	int rc = 0;
	struct venus_hfi_device *dev;

	if (!device || !resource_hdr) {
		dprintk(VIDC_ERR, "Inv-Params in rel_res\n");
		return -EINVAL;
	} else {
		dev = device;
	}

	rc = call_hfi_pkt_op(dev, sys_release_resource,
			&pkt, resource_hdr);
	if (rc) {
		dprintk(VIDC_ERR, "release_res: failed to create packet\n");
		goto err_create_pkt;
	}

	if (venus_hfi_iface_cmdq_write(dev, &pkt))
		rc = -ENOTEMPTY;

err_create_pkt:
	return rc;
}

static DECLARE_COMPLETION(pc_prep_done);
static DECLARE_COMPLETION(release_resources_done);

static int __alloc_ocmem(struct venus_hfi_device *device)
{
	int rc = 0;
	struct ocmem_buf *ocmem_buffer;
	unsigned long size;

	if (!device || !device->res) {
		dprintk(VIDC_ERR, "%s Invalid param, device: 0x%pK\n",
				__func__, device);
		return -EINVAL;
	}

	size = device->res->ocmem_size;
	if (!size)
		return rc;

	ocmem_buffer = device->resources.ocmem.buf;
	if (!ocmem_buffer || ocmem_buffer->len < size) {
		ocmem_buffer = ocmem_allocate(OCMEM_VIDEO, size);
		if (IS_ERR_OR_NULL(ocmem_buffer)) {
			dprintk(VIDC_ERR,
					"ocmem_allocate failed: %lu\n",
					(unsigned long)ocmem_buffer);
			rc = -ENOMEM;
			device->resources.ocmem.buf = NULL;
			goto ocmem_alloc_failed;
		}
		device->resources.ocmem.buf = ocmem_buffer;
	} else {
		dprintk(VIDC_DBG,
			"OCMEM is enough. reqd: %lu, available: %lu\n",
			size, ocmem_buffer->len);
	}
ocmem_alloc_failed:
	return rc;
}

static int __free_ocmem(struct venus_hfi_device *device)
{
	int rc = 0;

	if (!device || !device->res) {
		dprintk(VIDC_ERR, "%s Invalid param, device: 0x%pK\n",
				__func__, device);
		return -EINVAL;
	}

	if (!device->res->ocmem_size)
		return rc;

	if (device->resources.ocmem.buf) {
		rc = ocmem_free(OCMEM_VIDEO, device->resources.ocmem.buf);
		if (rc)
			dprintk(VIDC_ERR, "Failed to free ocmem\n");
		device->resources.ocmem.buf = NULL;
	}
	return rc;
}

static int __set_ocmem(struct venus_hfi_device *device, bool locked)
{
	struct vidc_resource_hdr rhdr;
	int rc = 0;
	struct on_chip_mem *ocmem;

	if (!device) {
		dprintk(VIDC_ERR, "%s Invalid param, device: 0x%pK\n",
				__func__, device);
		return -EINVAL;
	}

	ocmem = &device->resources.ocmem;
	if (!ocmem->buf) {
		dprintk(VIDC_ERR, "Invalid params, ocmem_buffer: 0x%pK\n",
			ocmem->buf);
		return -EINVAL;
	}

	rhdr.resource_id = VIDC_RESOURCE_OCMEM;
	/*
	 * This handle is just used as a cookie and not(cannot be)
	 * accessed by fw
	 */
	rhdr.resource_handle = (u32)(unsigned long)ocmem;
	rhdr.size = ocmem->buf->len;
	rc = venus_hfi_core_set_resource(device, &rhdr, ocmem->buf, locked);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to set OCMEM on driver\n");
		goto ocmem_set_failed;
	}
	dprintk(VIDC_DBG, "OCMEM set, addr = %lx, size: %ld\n",
		ocmem->buf->addr, ocmem->buf->len);
ocmem_set_failed:
	return rc;
}

static int __unset_ocmem(struct venus_hfi_device *device)
{
	struct vidc_resource_hdr rhdr;
	int rc = 0;

	if (!device) {
		dprintk(VIDC_ERR, "%s Invalid param, device: 0x%pK\n",
				__func__, device);
		rc = -EINVAL;
		goto ocmem_unset_failed;
	}

	if (!device->resources.ocmem.buf) {
		dprintk(VIDC_INFO,
				"%s Trying to unset OCMEM which is not allocated\n",
				__func__);
		rc = -EINVAL;
		goto ocmem_unset_failed;
	}
	rhdr.resource_id = VIDC_RESOURCE_OCMEM;
	/*
	 * This handle is just used as a cookie and not(cannot be)
	 * accessed by fw
	 */
	rhdr.resource_handle = (u32)(unsigned long)&device->resources.ocmem;
	rc = venus_hfi_core_release_resource(device, &rhdr);
	if (rc)
		dprintk(VIDC_ERR, "Failed to unset OCMEM on driver\n");
ocmem_unset_failed:
	return rc;
}

static int __alloc_set_ocmem(struct venus_hfi_device *device, bool locked)
{
	int rc = 0;

	if (!device || !device->res) {
		dprintk(VIDC_ERR, "%s Invalid param, device: 0x%pK\n",
				__func__, device);
		return -EINVAL;
	}

	if (!device->res->ocmem_size)
		return rc;

	rc = __alloc_ocmem(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to allocate ocmem: %d\n", rc);
		goto ocmem_alloc_failed;
	}

	rc = venus_hfi_vote_buses(device, device->bus_load.vote_data,
			device->bus_load.vote_data_count);
	if (rc) {
		dprintk(VIDC_ERR,
				"Failed to scale buses after setting ocmem: %d\n",
				rc);
		goto ocmem_set_failed;
	}

	rc = __set_ocmem(device, locked);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to set ocmem: %d\n", rc);
		goto ocmem_set_failed;
	}
	return rc;
ocmem_set_failed:
	__free_ocmem(device);
ocmem_alloc_failed:
	return rc;
}

static int __unset_free_ocmem(struct venus_hfi_device *device)
{
	int rc = 0;

	if (!device || !device->res) {
		dprintk(VIDC_ERR, "%s Invalid param, device: 0x%pK\n",
				__func__, device);
		return -EINVAL;
	}

	if (!device->res->ocmem_size)
		return rc;

	mutex_lock(&device->write_lock);
	mutex_lock(&device->read_lock);
	rc = venus_hfi_core_in_valid_state(device);
	mutex_unlock(&device->read_lock);
	mutex_unlock(&device->write_lock);

	if (!rc) {
		dprintk(VIDC_WARN,
			"Core is in bad state, Skipping unset OCMEM\n");
		goto core_in_bad_state;
	}

	init_completion(&release_resources_done);
	rc = __unset_ocmem(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to unset OCMEM during PC %d\n", rc);
		goto ocmem_unset_failed;
	}
	rc = wait_for_completion_timeout(&release_resources_done,
			msecs_to_jiffies(msm_vidc_hw_rsp_timeout));
	if (!rc) {
		dprintk(VIDC_ERR,
				"Wait interrupted or timeout for RELEASE_RESOURCES: %d\n",
				rc);
		rc = -EIO;
		goto release_resources_failed;
	}

core_in_bad_state:
	rc = __free_ocmem(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to free OCMEM during PC\n");
		goto ocmem_free_failed;
	}
	return rc;

ocmem_free_failed:
	__set_ocmem(device, true);
release_resources_failed:
ocmem_unset_failed:
	return rc;
}

static inline int venus_hfi_tzbsp_set_video_state(enum tzbsp_video_state state)
>>>>>>> p9x
{
	struct tzbsp_video_set_state_req cmd = {0};
	int tzbsp_rsp = 0;
	int rc = 0;
	struct scm_desc desc = {0};

	desc.args[0] = cmd.state = state;
	desc.args[1] = cmd.spare = 0;
	desc.arginfo = SCM_ARGS(2);

	if (!is_scm_armv8()) {
		rc = scm_call(SCM_SVC_BOOT, TZBSP_VIDEO_SET_STATE, &cmd,
				sizeof(cmd), &tzbsp_rsp, sizeof(tzbsp_rsp));
	} else {
		rc = scm_call2(SCM_SIP_FNID(SCM_SVC_BOOT,
				TZBSP_VIDEO_SET_STATE), &desc);
		tzbsp_rsp = desc.ret[0];
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	if (rc) {
		dprintk(VIDC_ERR, "Failed scm_call %d\n", rc);
		return rc;
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	dprintk(VIDC_DBG, "Set state %d, resp %d\n", state, tzbsp_rsp);
	if (tzbsp_rsp) {
		dprintk(VIDC_ERR,
				"Failed to set video core state to suspend: %d\n",
				tzbsp_rsp);
		return -EINVAL;
	}
<<<<<<< HEAD

	return 0;
}

static inline int __boot_firmware(struct venus_hfi_device *device)
{
	int rc = 0;
	u32 ctrl_status = 0, count = 0, max_tries = 100;

	__write_register(device, VIDC_CTRL_INIT, 0x1);
	while (!ctrl_status && count < max_tries) {
		ctrl_status = __read_register(device, VIDC_CPU_CS_SCIACMDARG0);
		if ((ctrl_status & 0xFE) == 0x4) {
			dprintk(VIDC_ERR, "invalid setting for UC_REGION\n");
			break;
		}

		usleep_range(500, 1000);
		count++;
	}

	if (count >= max_tries) {
		dprintk(VIDC_ERR, "Error booting up vidc firmware\n");
		rc = -ETIME;
	}
	return rc;
}

static struct clock_info *__get_clock(struct venus_hfi_device *device,
=======
	return 0;
}

static inline int venus_hfi_reset_core(struct venus_hfi_device *device)
{
	int rc = 0;
	venus_hfi_write_register(device, VIDC_CTRL_INIT, 0x1);
	rc = venus_hfi_core_start_cpu(device);
	if (rc)
		dprintk(VIDC_ERR, "Failed to start core\n");
	return rc;
}

static struct clock_info *venus_hfi_get_clock(struct venus_hfi_device *device,
>>>>>>> p9x
		char *name)
{
	struct clock_info *vc;

	venus_hfi_for_each_clock(device, vc) {
		if (!strcmp(vc->name, name))
			return vc;
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	dprintk(VIDC_WARN, "%s Clock %s not found\n", __func__, name);

	return NULL;
}

<<<<<<< HEAD
static unsigned long __get_clock_rate(struct clock_info *clock,
	int num_mbs_per_sec, struct vidc_clk_scale_data *data)
{
	int num_rows = clock->count;
	struct load_freq_table *table = clock->load_freq_tbl;
	unsigned long freq = table[0].freq, max_freq = 0;
	int i = 0, j = 0;
	unsigned long instance_freq[VIDC_MAX_SESSIONS] = {0};

	if (!data && !num_rows) {
		freq = 0;
=======
static int venus_hfi_set_clock(struct venus_hfi_device *device,
		unsigned long rate)
{
	struct clock_info *cl;
	int rc = 0;

	venus_hfi_for_each_clock(device, cl) {
		if (cl->count) {/* has_scaling */
			rc = clk_set_rate(cl->clk, rate);
			if (rc) {
				dprintk(VIDC_ERR,
					"Failed to set clock rate %lu %s: %d\n",
					rate, cl->name, rc);
				break;
			}
			dprintk(VIDC_DBG, "set clock rate %lu %s\n",
				rate, cl->name);
		}
	}
	return rc;
}

static unsigned long venus_hfi_get_clock_rate(struct venus_hfi_device *device,
	int num_mbs_per_sec, struct vidc_clk_scale_data *data)
{
	int num_rows = device->res->load_freq_tbl_size;
	struct load_freq_table *table = device->res->load_freq_tbl;
	unsigned long ret = table[0].freq, max_freq = 0;
	int i = 0, j = 0;
	bool matches = false;

	if (!data && !num_rows) {
		ret = 0;
>>>>>>> p9x
		goto print_clk;
	}

	if ((!num_mbs_per_sec || !data) && num_rows) {
<<<<<<< HEAD
		freq = table[num_rows - 1].freq;
=======
		ret = table[num_rows - 1].freq;
>>>>>>> p9x
		goto print_clk;
	}

	for (i = 0; i < num_rows; i++) {
		if (num_mbs_per_sec > table[i].load)
			break;
		for (j = 0; j < data->num_sessions; j++) {
<<<<<<< HEAD
			bool matches = __is_session_supported(
				table[i].supported_codecs, data->session[j]);

			if (!matches)
				continue;
			instance_freq[j] = table[i].freq;
		}
	}
	for (i = 0; i < data->num_sessions; i++)
		max_freq = max(instance_freq[i], max_freq);

	freq = max_freq ? : freq;
print_clk:
	dprintk(VIDC_PROF, "Required clock rate = %lu num_mbs_per_sec %d\n",
					freq, num_mbs_per_sec);
	return freq;
}

static unsigned long __get_clock_rate_with_bitrate(struct clock_info *clock,
		int num_mbs_per_sec, struct vidc_clk_scale_data *data,
		unsigned long instant_bitrate)
{
	int num_rows = clock->count;
	struct load_freq_table *table = clock->load_freq_tbl;
	unsigned long freq = table[0].freq, max_freq = 0;
	unsigned long base_freq, supported_clk[VIDC_MAX_SESSIONS] = {0};
	int i, j;

	if (!data && !num_rows) {
		freq = 0;
		goto print_clk;
	}
	if ((!num_mbs_per_sec || !data) && num_rows) {
		freq = table[num_rows - 1].freq;
		goto print_clk;
	}

	/* Get clock rate based on current load only */
	base_freq = __get_clock_rate(clock, num_mbs_per_sec, data);

	/*
	 * Supported bitrate = 40% of clock frequency
	 * Check if the instant bitrate is supported by the base frequency.
	 * If not, move on to the next frequency which supports the bitrate.
	 */

	for (j = 0; j < data->num_sessions; j++) {
		unsigned long supported_bitrate = 0;

		for (i = num_rows - 1; i >= 0; i--) {
			bool matches = __is_session_supported(
				table[i].supported_codecs, data->session[j]);

			if (!matches)
				continue;
			freq = table[i].freq;

			supported_bitrate = freq * 40/100;
			/*
			 * Store this frequency for each instance, we need
			 * to select the maximum freq among all the instances.
			 */
			if (freq >= base_freq &&
				supported_bitrate >= instant_bitrate) {
				supported_clk[j] = freq;
				break;
			}
		}

		/*
		 * Current bitrate is higher than max supported load.
		 * Select max frequency to handle this load.
		 */
		if (i < 0)
			supported_clk[j] = table[0].freq;
	}

	for (i = 0; i < data->num_sessions; i++)
		max_freq = max(supported_clk[i], max_freq);

	freq = max_freq ? : base_freq;

	if (base_freq == freq)
		dprintk(VIDC_DBG, "Stay at base freq: %lu bitrate = %lu\n",
			freq, instant_bitrate);
	else
		dprintk(VIDC_DBG, "Move up clock freq: %lu bitrate = %lu\n",
			freq, instant_bitrate);
print_clk:
	dprintk(VIDC_PROF, "Required clock rate = %lu num_mbs_per_sec %d\n",
					freq, num_mbs_per_sec);
	return freq;
}

static unsigned long venus_hfi_get_core_clock_rate(void *dev, bool actual_rate)
=======
			matches = venus_hfi_is_session_supported(
				table[i].supported_codecs, data->session[j]);
			if (matches)
				data->freq[j] = table[i].freq;
		}
	}

	for (i = 0; i < data->num_sessions; i++)
		max_freq = max(data->freq[i], max_freq);

	ret = max_freq ? : ret;
print_clk:
	dprintk(VIDC_PROF, "Required clock rate = %lu num_mbs_per_sec %d\n",
			ret, num_mbs_per_sec);
	return ret;
}

static unsigned long venus_hfi_get_core_clock_rate(void *dev)
>>>>>>> p9x
{
	struct venus_hfi_device *device = (struct venus_hfi_device *) dev;
	struct clock_info *vc;

	if (!device) {
		dprintk(VIDC_ERR, "%s Invalid args: %pK\n", __func__, device);
		return -EINVAL;
	}

<<<<<<< HEAD
	if (actual_rate) {
		vc = __get_clock(device, "core_clk");
		if (vc)
			return clk_get_rate(vc->clk);
		else
			return 0;
	} else {
		return device->scaled_rate;
	}
=======
	vc = venus_hfi_get_clock(device, "core_clk");
	if (vc)
		return clk_get_rate(vc->clk);
	else
		return 0;
>>>>>>> p9x
}

static int venus_hfi_suspend(void *dev)
{
	int rc = 0;
	struct venus_hfi_device *device = (struct venus_hfi_device *) dev;

	if (!device) {
		dprintk(VIDC_ERR, "%s invalid device\n", __func__);
		return -EINVAL;
<<<<<<< HEAD
	} else if (!device->res->sw_power_collapsible) {
		return -ENOTSUPP;
	}

	mutex_lock(&device->lock);

=======
	}
	dprintk(VIDC_INFO, "%s\n", __func__);

	mutex_lock(&device->write_lock);
>>>>>>> p9x
	if (device->power_enabled) {
		dprintk(VIDC_DBG, "Venus is busy\n");
		rc = -EBUSY;
	} else {
		dprintk(VIDC_DBG, "Venus is power suspended\n");
		rc = 0;
	}
<<<<<<< HEAD

	mutex_unlock(&device->lock);
=======
	mutex_unlock(&device->write_lock);

>>>>>>> p9x
	return rc;
}

static enum hal_default_properties venus_hfi_get_default_properties(void *dev)
{
	enum hal_default_properties prop = 0;
	struct venus_hfi_device *device = (struct venus_hfi_device *) dev;

	if (!device) {
		dprintk(VIDC_ERR, "%s invalid device\n", __func__);
		return -EINVAL;
	}

<<<<<<< HEAD
	mutex_lock(&device->lock);

	if (device->packetization_type == HFI_PACKETIZATION_3XX)
		prop = HAL_VIDEO_DYNAMIC_BUF_MODE;

	mutex_unlock(&device->lock);
	return prop;
}

static int __halt_axi(struct venus_hfi_device *device)
=======
	if (device->packetization_type == HFI_PACKETIZATION_3XX)
		prop = HAL_VIDEO_DYNAMIC_BUF_MODE;

	return prop;
}

static int venus_hfi_halt_axi(struct venus_hfi_device *device)
>>>>>>> p9x
{
	u32 reg;
	int rc = 0;
	if (!device) {
		dprintk(VIDC_ERR, "Invalid input: %pK\n", device);
		return -EINVAL;
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	/*
	 * Driver needs to make sure that clocks are enabled to read Venus AXI
	 * registers. If not skip AXI HALT.
	 */
<<<<<<< HEAD
	if (!device->power_enabled) {
		dprintk(VIDC_WARN,
			"Clocks are OFF, skipping AXI HALT\n");
		WARN_ON(VIDC_DBG_WARN_ENABLE);
		return -EINVAL;
	}

	/* Halt AXI and AXI IMEM VBIF Access */
	reg = __read_register(device, VENUS_VBIF_AXI_HALT_CTRL0);
	reg |= VENUS_VBIF_AXI_HALT_CTRL0_HALT_REQ;
	__write_register(device, VENUS_VBIF_AXI_HALT_CTRL0, reg);
=======
	if (device->clk_state != ENABLED_PREPARED) {
		dprintk(VIDC_WARN,
			"Clocks are OFF, skipping AXI HALT\n");
		return -EINVAL;
	}

	/* Halt AXI and AXI OCMEM VBIF Access */
	reg = venus_hfi_read_register(device, VENUS_VBIF_AXI_HALT_CTRL0);
	reg |= VENUS_VBIF_AXI_HALT_CTRL0_HALT_REQ;
	venus_hfi_write_register(device, VENUS_VBIF_AXI_HALT_CTRL0, reg);
>>>>>>> p9x

	/* Request for AXI bus port halt */
	rc = readl_poll_timeout(device->hal_data->register_base
			+ VENUS_VBIF_AXI_HALT_CTRL1,
			reg, reg & VENUS_VBIF_AXI_HALT_CTRL1_HALT_ACK,
			POLL_INTERVAL_US,
			VENUS_VBIF_AXI_HALT_ACK_TIMEOUT_US);
	if (rc)
		dprintk(VIDC_WARN, "AXI bus port halt timeout\n");

	return rc;
}

<<<<<<< HEAD
static int __scale_clocks_cycles_per_mb(struct venus_hfi_device *device,
		struct vidc_clk_scale_data *data, unsigned long instant_bitrate)
{
	int rc = 0, i = 0, j = 0;
	struct clock_info *cl;
	struct clock_freq_table *clk_freq_tbl = NULL;
	struct allowed_clock_rates_table *allowed_clks_tbl = NULL;
	struct clock_profile_entry *entry = NULL;
	u64 total_freq = 0, rate = 0;

	clk_freq_tbl = &device->res->clock_freq_tbl;
	allowed_clks_tbl = device->res->allowed_clks_tbl;

	if (!data) {
		dprintk(VIDC_DBG, "%s: NULL scale data\n", __func__);
		total_freq = device->clk_freq;
		goto get_clock_freq;
	}

	device->clk_bitrate = instant_bitrate;

	for (i = 0; i < data->num_sessions; i++) {
		/*
		 * for each active session iterate through all possible
		 * sessions and get matching session's cycles per mb
		 * from dtsi and multiply with the session's load to
		 * get the frequency required for the session.
		 * accumulate all session's frequencies to get the
		 * total clock frequency.
		 */
		for (j = 0; j < clk_freq_tbl->count; j++) {
			bool matched = false;
			u64 freq = 0;

			entry = &clk_freq_tbl->clk_prof_entries[j];

			matched = __is_session_supported(entry->codec_mask,
					data->session[i]);
			if (!matched)
				continue;

			freq = entry->cycles * data->load[i];

			if (data->power_mode[i] == VIDC_POWER_LOW &&
					entry->low_power_factor) {
				/* low_power_factor is in Q16 format */
				freq = (freq * entry->low_power_factor) >> 16;
			}

			total_freq += freq;

			dprintk(VIDC_DBG,
				"session[%d] %#x: cycles (%d), load (%d), freq (%llu), factor (%d)\n",
				i, data->session[i], entry->cycles,
				data->load[i], freq,
				entry->low_power_factor);
		}
	}

get_clock_freq:
	/*
	 * get required clock rate from allowed clock rates table
	 */
	for (i = device->res->allowed_clks_tbl_size - 1; i >= 0; i--) {
		rate = allowed_clks_tbl[i].clock_rate;
		if (rate >= total_freq)
			break;
	}

	venus_hfi_for_each_clock(device, cl) {
		if (!cl->has_scaling)
			continue;

		device->clk_freq = rate;
		rc = clk_set_rate(cl->clk, rate);
		if (rc) {
			dprintk(VIDC_ERR,
				"%s: Failed to set clock rate %llu %s: %d\n",
				__func__, rate, cl->name, rc);
			return rc;
		}
		if (!strcmp(cl->name, "core_clk"))
			device->scaled_rate = rate;

		dprintk(VIDC_DBG,
			"scaling clock %s to %llu (required freq %llu)\n",
			cl->name, rate, total_freq);
=======
static inline int venus_hfi_power_off(struct venus_hfi_device *device)
{
	int rc = 0;

	if (!device) {
		dprintk(VIDC_ERR, "Invalid params: %pK\n", device);
		return -EINVAL;
	}
	if (!device->power_enabled) {
		dprintk(VIDC_DBG, "Power already disabled\n");
		return 0;
	}

	rc = venus_hfi_halt_axi(device);
	if (rc) {
		dprintk(VIDC_WARN, "Failed to halt AXI\n");
		return 0;
	}

	dprintk(VIDC_DBG, "Entering power collapse\n");
	rc = venus_hfi_tzbsp_set_video_state(TZBSP_VIDEO_STATE_SUSPEND);
	if (rc) {
		dprintk(VIDC_WARN, "Failed to suspend video core %d\n", rc);
		goto err_tzbsp_suspend;
	}
	venus_hfi_iommu_detach(device);

	/*
	* For some regulators, driver might have transfered the control to HW.
	* So before touching any clocks, driver should get the regulator
	* control back. Acquire regulators also makes sure that the regulators
	* are turned ON. So driver can touch the clocks safely.
	*/

	rc = venus_hfi_acquire_regulators(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to enable gdsc in %s Err code = %d\n",
			__func__, rc);
		goto err_acquire_regulators;
	}
	venus_hfi_disable_unprepare_clks(device);
	rc = venus_hfi_disable_regulators(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to disable gdsc\n");
		goto err_disable_regulators;
	}

	venus_hfi_unvote_buses(device);
	device->power_enabled = false;
	dprintk(VIDC_INFO, "Venus power collapsed\n");

	return rc;

err_disable_regulators:
	if (venus_hfi_prepare_enable_clks(device))
		dprintk(VIDC_ERR, "Failed prepare_enable_clks\n");
	if (venus_hfi_hand_off_regulators(device))
		dprintk(VIDC_ERR, "Failed hand_off_regulators\n");
err_acquire_regulators:
	if (venus_hfi_iommu_attach(device))
		dprintk(VIDC_ERR, "Failed iommu_attach\n");
	if (venus_hfi_tzbsp_set_video_state(TZBSP_VIDEO_STATE_RESUME))
		dprintk(VIDC_ERR, "Failed TZBSP_RESUME\n");
err_tzbsp_suspend:
	return rc;
}

static inline int venus_hfi_power_on(struct venus_hfi_device *device)
{
	int rc = 0;

	if (!device) {
		dprintk(VIDC_ERR, "Invalid params: %pK\n", device);
		return -EINVAL;
	}
	if (device->power_enabled)
		return 0;

	dprintk(VIDC_DBG, "Resuming from power collapse\n");

	rc = __alloc_ocmem(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to allocate OCMEM");
		return -EINVAL;
	}

	rc = venus_hfi_vote_buses(device, device->bus_load.vote_data,
			device->bus_load.vote_data_count);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to scale buses\n");
		goto err_vote_buses;
	}

	/* At this point driver has the control for all regulators */
	rc = venus_hfi_enable_regulators(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to enable GDSC in %s Err code = %d\n",
			__func__, rc);
		goto err_enable_gdsc;
	}

	rc = venus_hfi_prepare_enable_clks(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to enable clocks\n");
		goto err_enable_clk;
	}

	rc = venus_hfi_set_clock(device, device->clk_freq);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to scale clocks\n");
		goto err_scale_clk;
	}

	/* iommu_attach makes call to TZ for restore_sec_cfg. With this call
	 * TZ accesses the VMIDMT block which needs all the Venus clocks.
	 * While going to power collapse these clocks were turned OFF.
	 * Hence enabling the Venus clocks before iommu_attach call.
	 */

	rc = venus_hfi_iommu_attach(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to attach iommu after power on\n");
		goto err_iommu_attach;
	}

	/* Reboot the firmware */
	rc = venus_hfi_tzbsp_set_video_state(TZBSP_VIDEO_STATE_RESUME);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to resume video core %d\n", rc);
		goto err_set_video_state;
	}

	rc = venus_hfi_hand_off_regulators(device);
	if (rc)
		dprintk(VIDC_WARN, "Failed to handoff control to HW %d\n", rc);

	/*
	 * Re-program all of the registers that get reset as a result of
	 * regulator_disable() and _enable()
	 */
	venus_hfi_set_registers(device);

	venus_hfi_write_register(device, VIDC_UC_REGION_ADDR,
			(u32)device->iface_q_table.align_device_addr);
	venus_hfi_write_register(device, VIDC_UC_REGION_SIZE, SHARED_QSIZE);
	venus_hfi_write_register(device, VIDC_CPU_CS_SCIACMDARG2,
		(u32)device->iface_q_table.align_device_addr);

	if (device->sfr.align_device_addr)
		venus_hfi_write_register(device, VIDC_SFR_ADDR,
				(u32)device->sfr.align_device_addr);
	if (device->qdss.align_device_addr)
		venus_hfi_write_register(device, VIDC_MMAP_ADDR,
				(u32)device->qdss.align_device_addr);

	/* Wait for boot completion */
	rc = venus_hfi_reset_core(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to reset venus core\n");
		goto err_reset_core;
	}

	device->power_enabled = true;

	dprintk(VIDC_INFO, "Resumed from power collapse\n");
	return rc;
err_reset_core:
	venus_hfi_tzbsp_set_video_state(TZBSP_VIDEO_STATE_SUSPEND);
err_set_video_state:
	venus_hfi_iommu_detach(device);
err_iommu_attach:
err_scale_clk:
	venus_hfi_disable_unprepare_clks(device);
err_enable_clk:
	venus_hfi_disable_regulators(device);
err_enable_gdsc:
	venus_hfi_unvote_buses(device);
err_vote_buses:
	device->power_enabled = false;
	dprintk(VIDC_ERR, "Failed to resume from power collapse\n");
	return rc;
}

static int venus_hfi_power_enable(void *dev)
{
	int rc = 0;
	struct venus_hfi_device *device = dev;
	if (!device) {
		dprintk(VIDC_ERR, "Invalid params: %pK\n", device);
		return -EINVAL;
	}
	mutex_lock(&device->write_lock);
	rc = venus_hfi_power_on(device);
	if (rc)
		dprintk(VIDC_ERR, "%s: Failed to enable power\n", __func__);
	mutex_unlock(&device->write_lock);

	return rc;
}

static int venus_hfi_regulator_set_voltage(
		struct venus_hfi_device *device, unsigned long freq,
		struct clock_voltage_info *cv_info)
{
	int rc = 0, i = 0, voltage_idx = -1;
	struct regulator_info *rinfo = NULL;

	if (!device || !cv_info) {
		dprintk(VIDC_WARN, "%s: invalid args %pK %pK\n",
			__func__, device, cv_info);
		return -EINVAL;
	}
	if (!cv_info->count)
		return 0;

	for (i = 0; i < cv_info->count; i++) {
		if (freq == cv_info->cv_table[i].clock_freq) {
			voltage_idx = cv_info->cv_table[i].voltage_idx;
			break;
		}
	}
	if (voltage_idx == -1) {
		dprintk(VIDC_ERR,
			"%s: voltage_idx not found for freq %lu\n",
			__func__, freq);
		return 0;
	}

	venus_hfi_for_each_regulator(device, rinfo) {
		if (strnstr(rinfo->name, "vdd-cx", strlen(rinfo->name))) {
			rc = regulator_set_voltage(rinfo->regulator,
					voltage_idx, INT_MAX);
			if (rc) {
				dprintk(VIDC_ERR,
					"%s: Failed to set voltage_idx %d on %s: %d\n",
					__func__, voltage_idx, rinfo->name, rc);
			} else {
				dprintk(VIDC_DBG,
				"%s: set voltage_idx %d on %s for freq %lu\n",
				__func__, voltage_idx, rinfo->name, freq);
			}
		}
>>>>>>> p9x
	}

	return rc;
}

<<<<<<< HEAD
static int __scale_clocks_load(struct venus_hfi_device *device, int load,
		struct vidc_clk_scale_data *data, unsigned long instant_bitrate)
{
	struct clock_info *cl;

	device->clk_bitrate = instant_bitrate;

	venus_hfi_for_each_clock(device, cl) {
		if (cl->has_scaling) {

			unsigned long rate = 0;
			int rc;
			/*
			 * load_fw and power_on needs to be addressed.
			 * differently. Below check enforces the same.
			 */
			if (!device->clk_bitrate && !data && !load &&
				device->clk_freq)
				rate = device->clk_freq;

			if (!rate) {
				if (!device->clk_bitrate)
					rate = __get_clock_rate(cl, load,
							data);
				else
					rate = __get_clock_rate_with_bitrate(cl,
							load, data,
							instant_bitrate);
			}
			device->clk_freq = rate;
			rc = clk_set_rate(cl->clk, rate);
			if (rc) {
				dprintk(VIDC_ERR,
					"Failed to set clock rate %lu %s: %d\n",
					rate, cl->name, rc);
				return rc;
			}

			if (!strcmp(cl->name, "core_clk"))
				device->scaled_rate = rate;

			dprintk(VIDC_PROF, "Scaling clock %s to %lu\n",
					cl->name, rate);
		}
	}

	return 0;
}

static int __scale_clocks(struct venus_hfi_device *device,
		int load, struct vidc_clk_scale_data *data,
		unsigned long instant_bitrate)
{
	int rc = 0;

	if (device->res->clock_freq_tbl.clk_prof_entries &&
			device->res->allowed_clks_tbl)
		rc = __scale_clocks_cycles_per_mb(device,
				data, instant_bitrate);
	else if (device->res->load_freq_tbl)
		rc = __scale_clocks_load(device, load, data, instant_bitrate);
	else
		dprintk(VIDC_DBG, "Clock scaling is not supported\n");
=======
static int venus_hfi_scale_regulators(struct venus_hfi_device *device,
		struct vidc_clk_scale_data *data)
{
	int rc = 0, i = 0;
	enum vidc_vote_data_session session_vp9d = 0;
	struct clock_voltage_info *cv_info = NULL;
	bool matches = false;

	if (!device || !data) {
		dprintk(VIDC_ERR, "%s: Invalid args %pK, %pK\n",
			__func__, device, data);
		return -EINVAL;
	}

	if (!msm_vidc_regulator_scaling)
		return 0;

	session_vp9d = VIDC_VOTE_DATA_SESSION_VAL(HAL_VIDEO_CODEC_VP9,
					HAL_VIDEO_DOMAIN_DECODER);
	for (i = 0; i < data->num_sessions; i++) {
		matches = venus_hfi_is_session_supported(session_vp9d,
					data->session[i]);
		if (matches)
			break;
	}

	if (matches) {
		/*
		 * vp9 decoder is present, set appropriate voltage level
		 * based on vp9 clock voltage table in dtsi.
		 */
		cv_info = &device->res->cv_info_vp9d;
		if (cv_info->count) {
			u32 max_idx = cv_info->count - 1;

			if (device->clk_freq > (unsigned long)
				cv_info->cv_table[max_idx].clock_freq) {
				/*
				 * device max clock rate is not supported if
				 * vp9 decoder is present, so reduce clock rate
				 * to max vp9 decoder allowed rate.
				 */
				dprintk(VIDC_DBG,
					"%s: reduce clock rate from %ld to %d\n",
					__func__, device->clk_freq, cv_info->
					cv_table[max_idx].clock_freq);
				device->clk_freq = (unsigned long)cv_info->
					cv_table[max_idx].clock_freq;
			}

			rc = venus_hfi_regulator_set_voltage(
					device, device->clk_freq, cv_info);
			if (rc) {
				dprintk(VIDC_WARN,
					"%s: Failed to set vp9 regulators voltage\n",
					__func__);
			}
		} else {
			dprintk(VIDC_DBG, "zero cv_info_vp9d->count\n");
		}
	} else {
		cv_info = &device->res->cv_info;
		if (cv_info->count) {
			rc = venus_hfi_regulator_set_voltage(
					device, device->clk_freq, cv_info);
			if (rc) {
				dprintk(VIDC_WARN,
					"%s: Failed to set regulators voltage\n",
					__func__);
			}
		} else {
			dprintk(VIDC_DBG, "zero cv_info->count\n");
		}
	}
>>>>>>> p9x

	return rc;
}
static int venus_hfi_scale_clocks(void *dev, int load,
<<<<<<< HEAD
					struct vidc_clk_scale_data *data,
					unsigned long instant_bitrate)
=======
		struct vidc_clk_scale_data *data)
>>>>>>> p9x
{
	int rc = 0;
	struct venus_hfi_device *device = dev;

	if (!device) {
		dprintk(VIDC_ERR, "Invalid args: %pK\n", device);
		return -EINVAL;
	}

<<<<<<< HEAD
	mutex_lock(&device->lock);

	if (__resume(device)) {
		dprintk(VIDC_ERR, "Resume from power collapse failed\n");
		rc = -ENODEV;
		goto exit;
	}

	rc = __scale_clocks(device, load, data, instant_bitrate);
exit:
	mutex_unlock(&device->lock);
	return rc;
}

/* Writes into cmdq without raising an interrupt */
static int __iface_cmdq_write_relaxed(struct venus_hfi_device *device,
		void *pkt, bool *requires_interrupt)
{
	struct vidc_iface_q_info *q_info;
	struct vidc_hal_cmd_pkt_hdr *cmd_packet;
	int result = -E2BIG;
=======
	mutex_lock(&device->clock_lock);

	device->clk_freq = venus_hfi_get_clock_rate(device, load, data);

	rc = venus_hfi_scale_regulators(device, data);
	if (rc) {
		dprintk(VIDC_WARN, "%s: Failed to scale regulators\n",
			__func__);
	}

	rc = venus_hfi_set_clock(device, device->clk_freq);

	mutex_unlock(&device->clock_lock);

	return rc;
}

static int venus_hfi_iface_cmdq_write_nolock(struct venus_hfi_device *device,
					void *pkt)
{
	u32 rx_req_is_set = 0;
	struct vidc_iface_q_info *q_info;
	struct vidc_hal_cmd_pkt_hdr *cmd_packet;
	int result = -EPERM;
>>>>>>> p9x

	if (!device || !pkt) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}
<<<<<<< HEAD

	__strict_check(device);

	if (!__core_in_valid_state(device)) {
		dprintk(VIDC_ERR, "%s - fw not in init state\n", __func__);
=======
	WARN(!mutex_is_locked(&device->write_lock),
		"Cmd queue write lock must be acquired");
	if (!venus_hfi_core_in_valid_state(device)) {
		dprintk(VIDC_DBG, "%s - fw not in init state\n", __func__);
>>>>>>> p9x
		result = -EINVAL;
		goto err_q_null;
	}

	cmd_packet = (struct vidc_hal_cmd_pkt_hdr *)pkt;
	device->last_packet_type = cmd_packet->packet_type;

	q_info = &device->iface_queues[VIDC_IFACEQ_CMDQ_IDX];
	if (!q_info) {
		dprintk(VIDC_ERR, "cannot write to shared Q's\n");
		goto err_q_null;
	}

	if (!q_info->q_array.align_virtual_addr) {
		dprintk(VIDC_ERR, "cannot write to shared CMD Q's\n");
		result = -ENODATA;
		goto err_q_null;
	}

<<<<<<< HEAD
	__sim_modify_cmd_packet((u8 *)pkt, device);
	if (__resume(device)) {
		dprintk(VIDC_ERR, "%s: Power on failed\n", __func__);
		goto err_q_write;
	}

	if (!__write_queue(q_info, (u8 *)pkt, requires_interrupt)) {
		if (device->res->sw_power_collapsible) {
			cancel_delayed_work(&venus_hfi_pm_work);
			if (!queue_delayed_work(device->venus_pm_workq,
				&venus_hfi_pm_work,
				msecs_to_jiffies(
				msm_vidc_pwr_collapse_delay))) {
				dprintk(VIDC_DBG,
				"PM work already scheduled\n");
			}
		}

		result = 0;
	} else {
		dprintk(VIDC_ERR, "__iface_cmdq_write: queue full\n");
	}

=======
	venus_hfi_sim_modify_cmd_packet((u8 *)pkt, device);
	if (!venus_hfi_write_queue(q_info, (u8 *)pkt, &rx_req_is_set)) {

		if (venus_hfi_power_on(device)) {
			dprintk(VIDC_ERR, "%s: Power on failed\n", __func__);
			goto err_q_write;
		}
		if (rx_req_is_set)
			venus_hfi_write_register(
				device, VIDC_CPU_IC_SOFTINT,
				1 << VIDC_CPU_IC_SOFTINT_H2A_SHFT);
		result = 0;
	} else {
		dprintk(VIDC_ERR, "venus_hfi_iface_cmdq_write:queue_full\n");
	}
>>>>>>> p9x
err_q_write:
err_q_null:
	return result;
}

<<<<<<< HEAD
static int __iface_cmdq_write(struct venus_hfi_device *device, void *pkt)
{
	bool needs_interrupt = false;
	int rc = __iface_cmdq_write_relaxed(device, pkt, &needs_interrupt);

	if (!rc && needs_interrupt) {
		/* Consumer of cmdq prefers that we raise an interrupt */
		rc = 0;
		__write_register(device, VIDC_CPU_IC_SOFTINT,
				1 << VIDC_CPU_IC_SOFTINT_H2A_SHFT);
	}

	return rc;
}

static int __iface_msgq_read(struct venus_hfi_device *device, void *pkt)
=======
static int venus_hfi_iface_msgq_read(struct venus_hfi_device *device, void *pkt)
>>>>>>> p9x
{
	u32 tx_req_is_set = 0;
	int rc = 0;
	struct vidc_iface_q_info *q_info;

	if (!pkt) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}
<<<<<<< HEAD

	__strict_check(device);

	if (!__core_in_valid_state(device)) {
=======
	mutex_lock(&device->read_lock);
	if (!venus_hfi_core_in_valid_state(device)) {
>>>>>>> p9x
		dprintk(VIDC_DBG, "%s - fw not in init state\n", __func__);
		rc = -EINVAL;
		goto read_error_null;
	}

	if (device->iface_queues[VIDC_IFACEQ_MSGQ_IDX].
		q_array.align_virtual_addr == 0) {
		dprintk(VIDC_ERR, "cannot read from shared MSG Q's\n");
		rc = -ENODATA;
		goto read_error_null;
	}

	q_info = &device->iface_queues[VIDC_IFACEQ_MSGQ_IDX];
<<<<<<< HEAD
	if (!__read_queue(q_info, (u8 *)pkt, &tx_req_is_set)) {
		__hal_sim_modify_msg_packet((u8 *)pkt, device);
		if (tx_req_is_set)
			__write_register(device, VIDC_CPU_IC_SOFTINT,
=======
	if (!venus_hfi_read_queue(q_info, (u8 *)pkt, &tx_req_is_set)) {
		venus_hfi_hal_sim_modify_msg_packet((u8 *)pkt, device);
		if (tx_req_is_set)
			venus_hfi_write_register(
				device, VIDC_CPU_IC_SOFTINT,
>>>>>>> p9x
				1 << VIDC_CPU_IC_SOFTINT_H2A_SHFT);
		rc = 0;
	} else
		rc = -ENODATA;

read_error_null:
<<<<<<< HEAD
	return rc;
}

static int __iface_dbgq_read(struct venus_hfi_device *device, void *pkt)
=======
	mutex_unlock(&device->read_lock);
	return rc;
}

static int venus_hfi_iface_dbgq_read(struct venus_hfi_device *device, void *pkt)
>>>>>>> p9x
{
	u32 tx_req_is_set = 0;
	int rc = 0;
	struct vidc_iface_q_info *q_info;

	if (!pkt) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}
<<<<<<< HEAD

	__strict_check(device);

	if (!__core_in_valid_state(device)) {
=======
	mutex_lock(&device->read_lock);
	if (!venus_hfi_core_in_valid_state(device)) {
>>>>>>> p9x
		dprintk(VIDC_DBG, "%s - fw not in init state\n", __func__);
		rc = -EINVAL;
		goto dbg_error_null;
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	if (device->iface_queues[VIDC_IFACEQ_DBGQ_IDX].
		q_array.align_virtual_addr == 0) {
		dprintk(VIDC_ERR, "cannot read from shared DBG Q's\n");
		rc = -ENODATA;
		goto dbg_error_null;
	}
<<<<<<< HEAD

	q_info = &device->iface_queues[VIDC_IFACEQ_DBGQ_IDX];
	if (!__read_queue(q_info, (u8 *)pkt, &tx_req_is_set)) {
		if (tx_req_is_set)
			__write_register(device, VIDC_CPU_IC_SOFTINT,
=======
	q_info = &device->iface_queues[VIDC_IFACEQ_DBGQ_IDX];
	if (!venus_hfi_read_queue(q_info, (u8 *)pkt, &tx_req_is_set)) {
		if (tx_req_is_set)
			venus_hfi_write_register(
				device, VIDC_CPU_IC_SOFTINT,
>>>>>>> p9x
				1 << VIDC_CPU_IC_SOFTINT_H2A_SHFT);
		rc = 0;
	} else
		rc = -ENODATA;

dbg_error_null:
<<<<<<< HEAD
	return rc;
}

static void __set_queue_hdr_defaults(struct hfi_queue_header *q_hdr)
=======
	mutex_unlock(&device->read_lock);
	return rc;
}

static void venus_hfi_set_queue_hdr_defaults(struct hfi_queue_header *q_hdr)
>>>>>>> p9x
{
	q_hdr->qhdr_status = 0x1;
	q_hdr->qhdr_type = VIDC_IFACEQ_DFLT_QHDR;
	q_hdr->qhdr_q_size = VIDC_IFACEQ_QUEUE_SIZE / 4;
	q_hdr->qhdr_pkt_size = 0;
	q_hdr->qhdr_rx_wm = 0x1;
	q_hdr->qhdr_tx_wm = 0x1;
	q_hdr->qhdr_rx_req = 0x1;
	q_hdr->qhdr_tx_req = 0x0;
	q_hdr->qhdr_rx_irq_status = 0x0;
	q_hdr->qhdr_tx_irq_status = 0x0;
	q_hdr->qhdr_read_idx = 0x0;
	q_hdr->qhdr_write_idx = 0x0;
}

<<<<<<< HEAD
static void __interface_queues_release(struct venus_hfi_device *device)
=======
static void venus_hfi_interface_queues_release(struct venus_hfi_device *device)
>>>>>>> p9x
{
	int i;
	struct hfi_mem_map_table *qdss;
	struct hfi_mem_map *mem_map;
	int num_entries = device->res->qdss_addr_set.count;
<<<<<<< HEAD
	unsigned long mem_map_table_base_addr;
	struct context_bank_info *cb;

=======
	int domain = -1, partition = -1;
	unsigned long mem_map_table_base_addr;

	mutex_lock(&device->write_lock);
	mutex_lock(&device->read_lock);
>>>>>>> p9x
	if (device->qdss.mem_data) {
		qdss = (struct hfi_mem_map_table *)
			device->qdss.align_virtual_addr;
		qdss->mem_map_num_entries = num_entries;
		mem_map_table_base_addr =
			device->qdss.align_device_addr +
			sizeof(struct hfi_mem_map_table);
		qdss->mem_map_table_base_addr =
			(u32)mem_map_table_base_addr;
		if ((unsigned long)qdss->mem_map_table_base_addr !=
			mem_map_table_base_addr) {
			dprintk(VIDC_ERR,
<<<<<<< HEAD
				"Invalid mem_map_table_base_addr %#lx",
				mem_map_table_base_addr);
		}

		mem_map = (struct hfi_mem_map *)(qdss + 1);
		cb = msm_smem_get_context_bank(device->hal_client,
					false, HAL_BUFFER_INTERNAL_CMD_QUEUE);

		for (i = 0; cb && i < num_entries; i++) {
			iommu_unmap(cb->mapping->domain,
						mem_map[i].virtual_addr,
						mem_map[i].size);
		}

		__smem_free(device, device->qdss.mem_data);
	}

	__smem_free(device, device->iface_q_table.mem_data);
	__smem_free(device, device->sfr.mem_data);
=======
				"Invalid mem_map_table_base_addr 0x%lx",
				mem_map_table_base_addr);
		}
		mem_map = (struct hfi_mem_map *)(qdss + 1);
		msm_smem_get_domain_partition(device->hal_client, 0,
			HAL_BUFFER_INTERNAL_CMD_QUEUE, &domain, &partition);
		if (domain >= 0 && partition >= 0) {
			for (i = 0; i < num_entries; i++) {
				msm_iommu_unmap_contig_buffer(
					(unsigned long)
					(mem_map[i].virtual_addr), domain,
					partition, SZ_4K);
			}
		}
		venus_hfi_free(device, device->qdss.mem_data);
	}
	venus_hfi_free(device, device->iface_q_table.mem_data);
	venus_hfi_free(device, device->sfr.mem_data);
>>>>>>> p9x

	for (i = 0; i < VIDC_IFACEQ_NUMQ; i++) {
		device->iface_queues[i].q_hdr = NULL;
		device->iface_queues[i].q_array.mem_data = NULL;
		device->iface_queues[i].q_array.align_virtual_addr = NULL;
		device->iface_queues[i].q_array.align_device_addr = 0;
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	device->iface_q_table.mem_data = NULL;
	device->iface_q_table.align_virtual_addr = NULL;
	device->iface_q_table.align_device_addr = 0;

	device->qdss.mem_data = NULL;
	device->qdss.align_virtual_addr = NULL;
	device->qdss.align_device_addr = 0;

	device->sfr.mem_data = NULL;
	device->sfr.align_virtual_addr = NULL;
	device->sfr.align_device_addr = 0;

	device->mem_addr.mem_data = NULL;
	device->mem_addr.align_virtual_addr = NULL;
	device->mem_addr.align_device_addr = 0;

	msm_smem_delete_client(device->hal_client);
	device->hal_client = NULL;
<<<<<<< HEAD
}

static int __get_qdss_iommu_virtual_addr(struct venus_hfi_device *dev,
		struct hfi_mem_map *mem_map, struct dma_iommu_mapping *mapping)
{
	int i;
	int rc = 0;
	dma_addr_t iova = QDSS_IOVA_START;
=======
	mutex_unlock(&device->read_lock);
	mutex_unlock(&device->write_lock);
}
static int venus_hfi_get_qdss_iommu_virtual_addr(struct venus_hfi_device *dev,
		struct hfi_mem_map *mem_map, int domain, int partition)
{
	int i;
	int rc = 0;
	ion_phys_addr_t iova = 0;
>>>>>>> p9x
	int num_entries = dev->res->qdss_addr_set.count;
	struct addr_range *qdss_addr_tbl = dev->res->qdss_addr_set.addr_tbl;

	if (!num_entries)
		return -ENODATA;

	for (i = 0; i < num_entries; i++) {
<<<<<<< HEAD
		if (mapping) {
			rc = iommu_map(mapping->domain, iova,
					qdss_addr_tbl[i].start,
					qdss_addr_tbl[i].size,
					IOMMU_READ | IOMMU_WRITE);

			if (rc) {
				dprintk(VIDC_ERR,
						"IOMMU QDSS mapping failed for addr %#x\n",
=======
		if (domain >= 0 && partition >= 0) {
			rc = msm_iommu_map_contig_buffer(
				qdss_addr_tbl[i].start, domain, partition,
				qdss_addr_tbl[i].size, SZ_4K, 0, &iova);
			if (rc) {
				dprintk(VIDC_ERR,
						"IOMMU QDSS mapping failed for addr 0x%x\n",
>>>>>>> p9x
						qdss_addr_tbl[i].start);
				rc = -ENOMEM;
				break;
			}
		} else {
			iova =  qdss_addr_tbl[i].start;
		}
<<<<<<< HEAD

=======
>>>>>>> p9x
		mem_map[i].virtual_addr = (u32)iova;
		mem_map[i].physical_addr = qdss_addr_tbl[i].start;
		mem_map[i].size = qdss_addr_tbl[i].size;
		mem_map[i].attr = 0x0;
<<<<<<< HEAD

		iova += mem_map[i].size;
	}

	if (i < num_entries) {
		dprintk(VIDC_ERR,
			"QDSS mapping failed, Freeing other entries %d\n", i);

		for (--i; mapping && i >= 0; i--) {
			iommu_unmap(mapping->domain,
				mem_map[i].virtual_addr,
				mem_map[i].size);
		}
	}

	return rc;
}

static void __setup_ucregion_memory_map(struct venus_hfi_device *device)
{
	__write_register(device, VIDC_UC_REGION_ADDR,
			(u32)device->iface_q_table.align_device_addr);
	__write_register(device, VIDC_UC_REGION_SIZE, SHARED_QSIZE);
	__write_register(device, VIDC_CPU_CS_SCIACMDARG2,
			(u32)device->iface_q_table.align_device_addr);
	__write_register(device, VIDC_CPU_CS_SCIACMDARG1, 0x01);
	if (device->sfr.align_device_addr)
		__write_register(device, VIDC_SFR_ADDR,
				(u32)device->sfr.align_device_addr);
	if (device->qdss.align_device_addr)
		__write_register(device, VIDC_MMAP_ADDR,
				(u32)device->qdss.align_device_addr);
}

static int __interface_queues_init(struct venus_hfi_device *dev)
=======
	}
	if (i < num_entries) {
		dprintk(VIDC_ERR,
			"IOMMU QDSS mapping failed, Freeing entries %d\n", i);

		if (domain >= 0 && partition >= 0) {
			for (--i; i >= 0; i--) {
				msm_iommu_unmap_contig_buffer(
					(unsigned long)
					(mem_map[i].virtual_addr), domain,
					partition, SZ_4K);
			}
		}
	}
	return rc;
}

static int venus_hfi_interface_queues_init(struct venus_hfi_device *dev)
>>>>>>> p9x
{
	struct hfi_queue_table_header *q_tbl_hdr;
	struct hfi_queue_header *q_hdr;
	u32 i;
	int rc = 0;
	struct hfi_mem_map_table *qdss;
	struct hfi_mem_map *mem_map;
	struct vidc_iface_q_info *iface_q;
	struct hfi_sfr_struct *vsfr;
	struct vidc_mem_addr *mem_addr;
	int offset = 0;
	int num_entries = dev->res->qdss_addr_set.count;
<<<<<<< HEAD
	u32 value = 0;
	phys_addr_t fw_bias = 0;
	size_t q_size;
	unsigned long mem_map_table_base_addr;
	struct context_bank_info *cb;

	q_size = SHARED_QSIZE - ALIGNED_SFR_SIZE - ALIGNED_QDSS_SIZE;
	mem_addr = &dev->mem_addr;
	if (!is_iommu_present(dev->res))
		fw_bias = dev->hal_data->firmware_base;
	rc = __smem_alloc(dev, mem_addr, q_size, 1, 0,
=======
	int domain = -1, partition = -1;
	u32 value = 0;
	unsigned long mem_map_table_base_addr;
	phys_addr_t fw_bias = 0;

	mem_addr = &dev->mem_addr;
	if (!is_iommu_present(dev->res))
		fw_bias = dev->hal_data->firmware_base;
	rc = venus_hfi_alloc(dev, (void *) mem_addr,
			QUEUE_SIZE, 1, 0,
>>>>>>> p9x
			HAL_BUFFER_INTERNAL_CMD_QUEUE);
	if (rc) {
		dprintk(VIDC_ERR, "iface_q_table_alloc_fail\n");
		goto fail_alloc_queue;
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	dev->iface_q_table.align_virtual_addr = mem_addr->align_virtual_addr;
	dev->iface_q_table.align_device_addr = mem_addr->align_device_addr -
					fw_bias;
	dev->iface_q_table.mem_size = VIDC_IFACEQ_TABLE_SIZE;
	dev->iface_q_table.mem_data = mem_addr->mem_data;
	offset += dev->iface_q_table.mem_size;

	for (i = 0; i < VIDC_IFACEQ_NUMQ; i++) {
		iface_q = &dev->iface_queues[i];
		iface_q->q_array.align_device_addr = mem_addr->align_device_addr
			+ offset - fw_bias;
		iface_q->q_array.align_virtual_addr =
			mem_addr->align_virtual_addr + offset;
		iface_q->q_array.mem_size = VIDC_IFACEQ_QUEUE_SIZE;
		iface_q->q_array.mem_data = NULL;
		offset += iface_q->q_array.mem_size;
		iface_q->q_hdr = VIDC_IFACEQ_GET_QHDR_START_ADDR(
				dev->iface_q_table.align_virtual_addr, i);
<<<<<<< HEAD
		__set_queue_hdr_defaults(iface_q->q_hdr);
	}

	if ((msm_vidc_fw_debug_mode & HFI_DEBUG_MODE_QDSS) && num_entries) {
		rc = __smem_alloc(dev, mem_addr,
				ALIGNED_QDSS_SIZE, 1, 0,
				HAL_BUFFER_INTERNAL_CMD_QUEUE);
=======
		venus_hfi_set_queue_hdr_defaults(iface_q->q_hdr);
	}
	if ((msm_fw_debug_mode & HFI_DEBUG_MODE_QDSS) && num_entries) {
		rc = venus_hfi_alloc(dev, (void *) mem_addr,
			QDSS_SIZE, 1, 0,
			HAL_BUFFER_INTERNAL_CMD_QUEUE);
>>>>>>> p9x
		if (rc) {
			dprintk(VIDC_WARN,
				"qdss_alloc_fail: QDSS messages logging will not work\n");
			dev->qdss.align_device_addr = 0;
		} else {
			dev->qdss.align_device_addr =
				mem_addr->align_device_addr - fw_bias;
			dev->qdss.align_virtual_addr =
				mem_addr->align_virtual_addr;
<<<<<<< HEAD
			dev->qdss.mem_size = ALIGNED_QDSS_SIZE;
			dev->qdss.mem_data = mem_addr->mem_data;
		}
	}

	rc = __smem_alloc(dev, mem_addr,
			ALIGNED_SFR_SIZE, 1, 0,
=======
			dev->qdss.mem_size = QDSS_SIZE;
			dev->qdss.mem_data = mem_addr->mem_data;
		}
	}
	rc = venus_hfi_alloc(dev, (void *) mem_addr,
			SFR_SIZE, 1, 0,
>>>>>>> p9x
			HAL_BUFFER_INTERNAL_CMD_QUEUE);
	if (rc) {
		dprintk(VIDC_WARN, "sfr_alloc_fail: SFR not will work\n");
		dev->sfr.align_device_addr = 0;
	} else {
		dev->sfr.align_device_addr = mem_addr->align_device_addr -
					fw_bias;
		dev->sfr.align_virtual_addr = mem_addr->align_virtual_addr;
<<<<<<< HEAD
		dev->sfr.mem_size = ALIGNED_SFR_SIZE;
		dev->sfr.mem_data = mem_addr->mem_data;
	}

=======
		dev->sfr.mem_size = SFR_SIZE;
		dev->sfr.mem_data = mem_addr->mem_data;
	}
>>>>>>> p9x
	q_tbl_hdr = (struct hfi_queue_table_header *)
			dev->iface_q_table.align_virtual_addr;
	q_tbl_hdr->qtbl_version = 0;
	q_tbl_hdr->qtbl_size = VIDC_IFACEQ_TABLE_SIZE;
<<<<<<< HEAD
	q_tbl_hdr->qtbl_qhdr0_offset = sizeof(struct hfi_queue_table_header);
	q_tbl_hdr->qtbl_qhdr_size = sizeof(struct hfi_queue_header);
=======
	q_tbl_hdr->qtbl_qhdr0_offset = sizeof(
		struct hfi_queue_table_header);
	q_tbl_hdr->qtbl_qhdr_size = sizeof(
		struct hfi_queue_header);
>>>>>>> p9x
	q_tbl_hdr->qtbl_num_q = VIDC_IFACEQ_NUMQ;
	q_tbl_hdr->qtbl_num_active_q = VIDC_IFACEQ_NUMQ;

	iface_q = &dev->iface_queues[VIDC_IFACEQ_CMDQ_IDX];
	q_hdr = iface_q->q_hdr;
	q_hdr->qhdr_start_addr = (u32)iface_q->q_array.align_device_addr;
	q_hdr->qhdr_type |= HFI_Q_ID_HOST_TO_CTRL_CMD_Q;
	if ((ion_phys_addr_t)q_hdr->qhdr_start_addr !=
		iface_q->q_array.align_device_addr) {
<<<<<<< HEAD
		dprintk(VIDC_ERR, "Invalid CMDQ device address (%pa)",
=======
		dprintk(VIDC_ERR, "Invalid CMDQ device address (0x%pa)",
>>>>>>> p9x
			&iface_q->q_array.align_device_addr);
	}

	iface_q = &dev->iface_queues[VIDC_IFACEQ_MSGQ_IDX];
	q_hdr = iface_q->q_hdr;
	q_hdr->qhdr_start_addr = (u32)iface_q->q_array.align_device_addr;
	q_hdr->qhdr_type |= HFI_Q_ID_CTRL_TO_HOST_MSG_Q;
	if ((ion_phys_addr_t)q_hdr->qhdr_start_addr !=
		iface_q->q_array.align_device_addr) {
<<<<<<< HEAD
		dprintk(VIDC_ERR, "Invalid MSGQ device address (%pa)",
=======
		dprintk(VIDC_ERR, "Invalid MSGQ device address (0x%pa)",
>>>>>>> p9x
			&iface_q->q_array.align_device_addr);
	}

	iface_q = &dev->iface_queues[VIDC_IFACEQ_DBGQ_IDX];
	q_hdr = iface_q->q_hdr;
	q_hdr->qhdr_start_addr = (u32)iface_q->q_array.align_device_addr;
	q_hdr->qhdr_type |= HFI_Q_ID_CTRL_TO_HOST_DEBUG_Q;
	/*
	 * Set receive request to zero on debug queue as there is no
	 * need of interrupt from video hardware for debug messages
	 */
	q_hdr->qhdr_rx_req = 0;
	if ((ion_phys_addr_t)q_hdr->qhdr_start_addr !=
		iface_q->q_array.align_device_addr) {
<<<<<<< HEAD
		dprintk(VIDC_ERR, "Invalid DBGQ device address (%pa)",
=======
		dprintk(VIDC_ERR, "Invalid DBGQ device address (0x%pa)",
>>>>>>> p9x
			&iface_q->q_array.align_device_addr);
	}

	value = (u32)dev->iface_q_table.align_device_addr;
	if ((ion_phys_addr_t)value !=
		dev->iface_q_table.align_device_addr) {
		dprintk(VIDC_ERR,
<<<<<<< HEAD
			"Invalid iface_q_table device address (%pa)",
			&dev->iface_q_table.align_device_addr);
	}
=======
			"Invalid iface_q_table device address (0x%pa)",
			&dev->iface_q_table.align_device_addr);
	}
	venus_hfi_write_register(dev, VIDC_UC_REGION_ADDR, value);
	venus_hfi_write_register(dev, VIDC_UC_REGION_SIZE, SHARED_QSIZE);
	venus_hfi_write_register(dev, VIDC_CPU_CS_SCIACMDARG2, value);
	venus_hfi_write_register(dev, VIDC_CPU_CS_SCIACMDARG1, 0x01);
>>>>>>> p9x

	if (dev->qdss.mem_data) {
		qdss = (struct hfi_mem_map_table *)dev->qdss.align_virtual_addr;
		qdss->mem_map_num_entries = num_entries;
		mem_map_table_base_addr = dev->qdss.align_device_addr +
			sizeof(struct hfi_mem_map_table);
		qdss->mem_map_table_base_addr =
			(u32)mem_map_table_base_addr;
		if ((ion_phys_addr_t)qdss->mem_map_table_base_addr !=
				mem_map_table_base_addr) {
			dprintk(VIDC_ERR,
<<<<<<< HEAD
					"Invalid mem_map_table_base_addr (%#lx)",
					mem_map_table_base_addr);
		}

		mem_map = (struct hfi_mem_map *)(qdss + 1);
		cb = msm_smem_get_context_bank(dev->hal_client, false,
				HAL_BUFFER_INTERNAL_CMD_QUEUE);

		if (!cb) {
			dprintk(VIDC_ERR,
				"%s: failed to get context bank\n", __func__);
			return -EINVAL;
		}

		rc = __get_qdss_iommu_virtual_addr(dev, mem_map, cb->mapping);
		if (rc) {
			dprintk(VIDC_ERR,
				"IOMMU mapping failed, Freeing qdss memdata\n");
			__smem_free(dev, dev->qdss.mem_data);
=======
					"Invalid mem_map_table_base_addr (0x%lx)",
					mem_map_table_base_addr);
		}
		mem_map = (struct hfi_mem_map *)(qdss + 1);
		msm_smem_get_domain_partition(dev->hal_client, 0,
				HAL_BUFFER_INTERNAL_CMD_QUEUE,
				&domain, &partition);
		rc = venus_hfi_get_qdss_iommu_virtual_addr(dev,
				mem_map, domain, partition);
		if (rc) {
			dprintk(VIDC_ERR,
					"IOMMU mapping failed, Freeing qdss memdata\n");
			venus_hfi_free(dev, dev->qdss.mem_data);
>>>>>>> p9x
			dev->qdss.mem_data = NULL;
			dev->qdss.align_virtual_addr = NULL;
			dev->qdss.align_device_addr = 0;
		}
<<<<<<< HEAD

		value = (u32)dev->qdss.align_device_addr;
		if ((ion_phys_addr_t)value !=
				dev->qdss.align_device_addr) {
			dprintk(VIDC_ERR, "Invalid qdss device address (%pa)",
					&dev->qdss.align_device_addr);
		}
	}

	vsfr = (struct hfi_sfr_struct *) dev->sfr.align_virtual_addr;
	vsfr->bufSize = ALIGNED_SFR_SIZE;
	value = (u32)dev->sfr.align_device_addr;
	if ((ion_phys_addr_t)value !=
		dev->sfr.align_device_addr) {
		dprintk(VIDC_ERR, "Invalid sfr device address (%pa)",
			&dev->sfr.align_device_addr);
	}

	__setup_ucregion_memory_map(dev);
=======
		value = (u32)dev->qdss.align_device_addr;
		if ((ion_phys_addr_t)value !=
				dev->qdss.align_device_addr) {
			dprintk(VIDC_ERR, "Invalid qdss device address (0x%pa)",
					&dev->qdss.align_device_addr);
		}
		if (dev->qdss.align_device_addr)
			venus_hfi_write_register(dev, VIDC_MMAP_ADDR, value);
	}

	vsfr = (struct hfi_sfr_struct *) dev->sfr.align_virtual_addr;
	vsfr->bufSize = SFR_SIZE;
	value = (u32)dev->sfr.align_device_addr;
	if ((ion_phys_addr_t)value !=
		dev->sfr.align_device_addr) {
		dprintk(VIDC_ERR, "Invalid sfr device address (0x%pa)",
			&dev->sfr.align_device_addr);
	}
	if (dev->sfr.align_device_addr)
		venus_hfi_write_register(dev, VIDC_SFR_ADDR, value);
>>>>>>> p9x
	return 0;
fail_alloc_queue:
	return -ENOMEM;
}

<<<<<<< HEAD
static int __sys_set_debug(struct venus_hfi_device *device, u32 debug)
=======
static int venus_hfi_sys_set_debug(struct venus_hfi_device *device, u32 debug)
>>>>>>> p9x
{
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	int rc = 0;
	struct hfi_cmd_sys_set_property_packet *pkt =
		(struct hfi_cmd_sys_set_property_packet *) &packet;

	rc = call_hfi_pkt_op(device, sys_debug_config, pkt, debug);
	if (rc) {
		dprintk(VIDC_WARN,
			"Debug mode setting to FW failed\n");
		return -ENOTEMPTY;
	}
<<<<<<< HEAD

	if (__iface_cmdq_write(device, pkt))
=======
	if (venus_hfi_iface_cmdq_write(device, pkt))
>>>>>>> p9x
		return -ENOTEMPTY;
	return 0;
}

<<<<<<< HEAD
static int __sys_set_coverage(struct venus_hfi_device *device, u32 mode)
=======
static int venus_hfi_sys_set_coverage(struct venus_hfi_device *device, u32 mode)
>>>>>>> p9x
{
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	int rc = 0;
	struct hfi_cmd_sys_set_property_packet *pkt =
		(struct hfi_cmd_sys_set_property_packet *) &packet;

	rc = call_hfi_pkt_op(device, sys_coverage_config,
			pkt, mode);
	if (rc) {
		dprintk(VIDC_WARN,
			"Coverage mode setting to FW failed\n");
		return -ENOTEMPTY;
	}
<<<<<<< HEAD

	if (__iface_cmdq_write(device, pkt)) {
		dprintk(VIDC_WARN, "Failed to send coverage pkt to f/w\n");
		return -ENOTEMPTY;
	}

	return 0;
}

static int __sys_set_idle_message(struct venus_hfi_device *device,
=======
	if (venus_hfi_iface_cmdq_write(device, pkt)) {
		dprintk(VIDC_WARN, "Failed to send coverage pkt to f/w\n");
		return -ENOTEMPTY;
	}
	return 0;
}

static int venus_hfi_sys_set_idle_message(struct venus_hfi_device *device,
>>>>>>> p9x
	bool enable)
{
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	struct hfi_cmd_sys_set_property_packet *pkt =
		(struct hfi_cmd_sys_set_property_packet *) &packet;
	if (!enable) {
		dprintk(VIDC_DBG, "sys_idle_indicator is not enabled\n");
		return 0;
	}
<<<<<<< HEAD

	call_hfi_pkt_op(device, sys_idle_indicator, pkt, enable);
	if (__iface_cmdq_write(device, pkt))
=======
	call_hfi_pkt_op(device, sys_idle_indicator, pkt, enable);
	if (venus_hfi_iface_cmdq_write(device, pkt))
>>>>>>> p9x
		return -ENOTEMPTY;
	return 0;
}

<<<<<<< HEAD
static int __sys_set_power_control(struct venus_hfi_device *device,
=======
static int venus_hfi_sys_set_power_control(struct venus_hfi_device *device,
>>>>>>> p9x
	bool enable)
{
	struct regulator_info *rinfo;
	bool supported = false;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	struct hfi_cmd_sys_set_property_packet *pkt =
		(struct hfi_cmd_sys_set_property_packet *) &packet;

	venus_hfi_for_each_regulator(device, rinfo) {
		if (rinfo->has_hw_power_collapse) {
			supported = true;
			break;
		}
	}

	if (!supported)
		return 0;

	call_hfi_pkt_op(device, sys_power_control, pkt, enable);
<<<<<<< HEAD
	if (__iface_cmdq_write(device, pkt))
=======
	if (venus_hfi_iface_cmdq_write(device, pkt))
>>>>>>> p9x
		return -ENOTEMPTY;
	return 0;
}

static int venus_hfi_core_init(void *device)
{
	struct hfi_cmd_sys_init_packet pkt;
	struct hfi_cmd_sys_get_property_packet version_pkt;
	int rc = 0;
	struct list_head *ptr, *next;
	struct hal_session *session = NULL;
	struct venus_hfi_device *dev;
<<<<<<< HEAD

	if (!device) {
=======
	struct clock_voltage_info *cv_info = NULL;

	if (device) {
		dev = device;
	} else {
>>>>>>> p9x
		dprintk(VIDC_ERR, "Invalid device\n");
		return -ENODEV;
	}

<<<<<<< HEAD
	dev = device;
	mutex_lock(&dev->lock);

	rc = __load_fw(dev);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to load Venus FW\n");
		goto err_load_fw;
	}

	__set_state(dev, VENUS_STATE_INIT);

=======
	venus_hfi_set_state(dev, VENUS_STATE_INIT);

	dev->intr_status = 0;

	mutex_lock(&dev->session_lock);
>>>>>>> p9x
	list_for_each_safe(ptr, next, &dev->sess_head) {
		/* This means that session list is not empty. Kick stale
		 * sessions out of our valid instance list, but keep the
		 * list_head inited so that list_del (in the future, called
		 * by session_clean()) will be valid. When client doesn't close
		 * them, then it is a genuine leak which driver can't fix. */
		session = list_entry(ptr, struct hal_session, list);
		list_del_init(&session->list);
	}
<<<<<<< HEAD

	INIT_LIST_HEAD(&dev->sess_head);

	__set_registers(dev);

	if (!dev->hal_client) {
		dev->hal_client = msm_smem_new_client(
				SMEM_ION, dev->res, MSM_VIDC_UNKNOWN);
=======
	INIT_LIST_HEAD(&dev->sess_head);
	mutex_unlock(&dev->session_lock);

	venus_hfi_set_registers(dev);

	if (!dev->hal_client) {
		dev->hal_client = msm_smem_new_client(SMEM_ION, dev->res);
>>>>>>> p9x
		if (dev->hal_client == NULL) {
			dprintk(VIDC_ERR, "Failed to alloc ION_Client\n");
			rc = -ENODEV;
			goto err_core_init;
		}

<<<<<<< HEAD
		dprintk(VIDC_DBG, "Dev_Virt: %pa, Reg_Virt: %pK\n",
			&dev->hal_data->firmware_base,
			dev->hal_data->register_base);

		rc = __interface_queues_init(dev);
=======
		dprintk(VIDC_DBG, "Dev_Virt: 0x%pa, Reg_Virt: 0x%pK\n",
			&dev->hal_data->firmware_base,
			dev->hal_data->register_base);

		rc = venus_hfi_interface_queues_init(dev);
>>>>>>> p9x
		if (rc) {
			dprintk(VIDC_ERR, "failed to init queues\n");
			rc = -ENOMEM;
			goto err_core_init;
		}
	} else {
		dprintk(VIDC_ERR, "hal_client exists\n");
		rc = -EEXIST;
		goto err_core_init;
	}
<<<<<<< HEAD

	rc = __boot_firmware(dev);
=======
	enable_irq(dev->hal_data->irq);
	venus_hfi_write_register(dev, VIDC_CTRL_INIT, 0x1);
	rc = venus_hfi_core_start_cpu(dev);
>>>>>>> p9x
	if (rc) {
		dprintk(VIDC_ERR, "Failed to start core\n");
		rc = -ENODEV;
		goto err_core_init;
	}

<<<<<<< HEAD
=======
	/*
	 * firmware will check below register in sys_init parsing
	 * to see if SW workaround for venus HW bug is enabled
	 */
	cv_info = &dev->res->cv_info;
	if (cv_info->count && msm_vidc_reset_clock_control) {
		u32 ctrl_init;
		dprintk(VIDC_DBG, "video reset clock control enabled\n");
		ctrl_init = venus_hfi_read_register(device, VIDC_CTRL_INIT);
		ctrl_init |= 0x80000000;
		venus_hfi_write_register(dev, VIDC_CTRL_INIT, ctrl_init);
	}

>>>>>>> p9x
	rc =  call_hfi_pkt_op(dev, sys_init, &pkt, HFI_VIDEO_ARCH_OX);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to create sys init pkt\n");
		goto err_core_init;
	}
<<<<<<< HEAD

	if (__iface_cmdq_write(dev, &pkt)) {
=======
	if (venus_hfi_iface_cmdq_write(dev, &pkt)) {
>>>>>>> p9x
		rc = -ENOTEMPTY;
		goto err_core_init;
	}

	rc = call_hfi_pkt_op(dev, sys_image_version, &version_pkt);
<<<<<<< HEAD
	if (rc || __iface_cmdq_write(dev, &version_pkt))
		dprintk(VIDC_WARN, "Failed to send image version pkt to f/w\n");

	if (dev->res->pm_qos_latency_us) {
#ifdef CONFIG_SMP
		dev->qos.type = PM_QOS_REQ_AFFINE_IRQ;
		dev->qos.irq = dev->hal_data->irq;
#endif
		pm_qos_add_request(&dev->qos, PM_QOS_CPU_DMA_LATENCY,
				dev->res->pm_qos_latency_us);
	}

	mutex_unlock(&dev->lock);
	return rc;
err_core_init:
	__set_state(dev, VENUS_STATE_DEINIT);
	__unload_fw(dev);
err_load_fw:
	mutex_unlock(&dev->lock);
	return rc;
}

static int venus_hfi_core_release(void *dev)
{
	struct venus_hfi_device *device = dev;
	int rc = 0;

	if (!device) {
=======
	if (rc || venus_hfi_iface_cmdq_write(dev, &version_pkt))
		dprintk(VIDC_WARN, "Failed to send image version pkt to f/w\n");

	return rc;
err_core_init:
	venus_hfi_set_state(dev, VENUS_STATE_DEINIT);
	disable_irq_nosync(dev->hal_data->irq);
	return rc;
}

static int venus_hfi_core_release(void *device)
{
	struct venus_hfi_device *dev;
	int rc = 0;

	if (device) {
		dev = device;
	} else {
>>>>>>> p9x
		dprintk(VIDC_ERR, "invalid device\n");
		return -ENODEV;
	}

<<<<<<< HEAD
	mutex_lock(&device->lock);

	if (device->res->pm_qos_latency_us &&
		pm_qos_request_active(&device->qos))
		pm_qos_remove_request(&device->qos);
	__set_state(device, VENUS_STATE_DEINIT);
	__unload_fw(device);

	mutex_unlock(&device->lock);

	return rc;
}

static int __get_q_size(struct venus_hfi_device *dev, unsigned int q_index)
=======
	if (dev->hal_client) {
		if (venus_hfi_power_enable(device)) {
			dprintk(VIDC_ERR,
					"%s: Power enable failed\n", __func__);
			return -EIO;
		}
		if (dev->state != VENUS_STATE_DEINIT) {
			mutex_lock(&dev->resource_lock);
			rc = __unset_free_ocmem(dev);
			mutex_unlock(&dev->resource_lock);

			if (rc)
				dprintk(VIDC_ERR,
					"Failed in unset_free_ocmem() in %s, rc : %d\n",
					__func__, rc);
		}

		/* flush debug queue before stop cpu */
		venus_hfi_flush_debug_queue(dev, NULL);

		venus_hfi_write_register(dev, VIDC_CPU_CS_SCIACMDARG3, 0);
		if (!(dev->intr_status & VIDC_WRAPPER_INTR_STATUS_A2HWD_BMSK))
			disable_irq_nosync(dev->hal_data->irq);
		dev->intr_status = 0;
	}
	venus_hfi_set_state(dev, VENUS_STATE_DEINIT);

	dprintk(VIDC_INFO, "HAL exited\n");
	return 0;
}

static int venus_hfi_get_q_size(struct venus_hfi_device *dev,
	unsigned int q_index)
>>>>>>> p9x
{
	struct hfi_queue_header *queue;
	struct vidc_iface_q_info *q_info;
	u32 write_ptr, read_ptr;
<<<<<<< HEAD
=======
	u32 rc = 0;
>>>>>>> p9x

	if (q_index >= VIDC_IFACEQ_NUMQ) {
		dprintk(VIDC_ERR, "Invalid q index: %d\n", q_index);
		return -ENOENT;
	}

	q_info = &dev->iface_queues[q_index];
	if (!q_info) {
		dprintk(VIDC_ERR, "cannot read shared Q's\n");
		return -ENOENT;
	}

	queue = (struct hfi_queue_header *)q_info->q_hdr;
	if (!queue) {
		dprintk(VIDC_ERR, "queue not present\n");
		return -ENOENT;
	}

	write_ptr = (u32)queue->qhdr_write_idx;
	read_ptr = (u32)queue->qhdr_read_idx;
<<<<<<< HEAD
	return read_ptr - write_ptr;
}

static void __core_clear_interrupt(struct venus_hfi_device *device)
=======
	rc = read_ptr - write_ptr;
	return rc;
}

static void venus_hfi_core_clear_interrupt(struct venus_hfi_device *device)
>>>>>>> p9x
{
	u32 intr_status = 0;

	if (!device) {
		dprintk(VIDC_ERR, "%s: NULL device\n", __func__);
		return;
	}

<<<<<<< HEAD
	intr_status = __read_register(device, VIDC_WRAPPER_INTR_STATUS);

	if (intr_status & VIDC_WRAPPER_INTR_STATUS_A2H_BMSK ||
		intr_status & VIDC_WRAPPER_INTR_STATUS_A2HWD_BMSK ||
		intr_status &
			VIDC_CPU_CS_SCIACMDARG0_HFI_CTRL_INIT_IDLE_MSG_BMSK) {
		device->intr_status |= intr_status;
		device->reg_count++;
		dprintk(VIDC_DBG,
			"INTERRUPT for device: %pK: times: %d interrupt_status: %d\n",
=======
	intr_status = venus_hfi_read_register(
			device,
			VIDC_WRAPPER_INTR_STATUS);

	if ((intr_status & VIDC_WRAPPER_INTR_STATUS_A2H_BMSK) ||
		(intr_status & VIDC_WRAPPER_INTR_STATUS_A2HWD_BMSK) ||
		(intr_status &
			VIDC_CPU_CS_SCIACMDARG0_HFI_CTRL_INIT_IDLE_MSG_BMSK)) {
		device->intr_status |= intr_status;
		device->reg_count++;
		dprintk(VIDC_DBG,
			"INTERRUPT for device: 0x%pK: times: %d interrupt_status: %d\n",
>>>>>>> p9x
			device, device->reg_count, intr_status);
	} else {
		device->spur_count++;
		dprintk(VIDC_INFO,
<<<<<<< HEAD
			"SPURIOUS_INTR for device: %pK: times: %d interrupt_status: %d\n",
			device, device->spur_count, intr_status);
	}

	__write_register(device, VIDC_CPU_CS_A2HSOFTINTCLR, 1);
	__write_register(device, VIDC_WRAPPER_INTR_CLEAR, intr_status);
=======
			"SPURIOUS_INTR for device: 0x%pK: times: %d interrupt_status: %d\n",
			device, device->spur_count, intr_status);
	}

	venus_hfi_write_register(device, VIDC_CPU_CS_A2HSOFTINTCLR, 1);
	venus_hfi_write_register(device, VIDC_WRAPPER_INTR_CLEAR, intr_status);
>>>>>>> p9x
	dprintk(VIDC_DBG, "Cleared WRAPPER/A2H interrupt\n");
}

static int venus_hfi_core_ping(void *device)
{
	struct hfi_cmd_sys_ping_packet pkt;
	int rc = 0;
	struct venus_hfi_device *dev;

<<<<<<< HEAD
	if (!device) {
=======
	if (device) {
		dev = device;
	} else {
>>>>>>> p9x
		dprintk(VIDC_ERR, "invalid device\n");
		return -ENODEV;
	}

<<<<<<< HEAD
	dev = device;
	mutex_lock(&dev->lock);

=======
>>>>>>> p9x
	rc = call_hfi_pkt_op(dev, sys_ping, &pkt);
	if (rc) {
		dprintk(VIDC_ERR, "core_ping: failed to create packet\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	if (__iface_cmdq_write(dev, &pkt))
		rc = -ENOTEMPTY;

err_create_pkt:
	mutex_unlock(&dev->lock);
=======
	if (venus_hfi_iface_cmdq_write(dev, &pkt))
		rc = -ENOTEMPTY;

err_create_pkt:
>>>>>>> p9x
	return rc;
}

static int venus_hfi_core_trigger_ssr(void *device,
<<<<<<< HEAD
		enum hal_ssr_trigger_type type)
=======
	enum hal_ssr_trigger_type type)
>>>>>>> p9x
{
	struct hfi_cmd_sys_test_ssr_packet pkt;
	int rc = 0;
	struct venus_hfi_device *dev;

<<<<<<< HEAD
	if (!device) {
=======
	if (device) {
		dev = device;
	} else {
>>>>>>> p9x
		dprintk(VIDC_ERR, "invalid device\n");
		return -ENODEV;
	}

<<<<<<< HEAD
	dev = device;
	mutex_lock(&dev->lock);

=======
>>>>>>> p9x
	rc = call_hfi_pkt_op(dev, ssr_cmd, type, &pkt);
	if (rc) {
		dprintk(VIDC_ERR, "core_ping: failed to create packet\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	if (__iface_cmdq_write(dev, &pkt))
		rc = -ENOTEMPTY;

err_create_pkt:
	mutex_unlock(&dev->lock);
=======
	if (venus_hfi_iface_cmdq_write(dev, &pkt))
		rc = -ENOTEMPTY;

err_create_pkt:
>>>>>>> p9x
	return rc;
}

static int venus_hfi_session_set_property(void *sess,
					enum hal_property ptype, void *pdata)
{
	u8 packet[VIDC_IFACEQ_VAR_LARGE_PKT_SIZE];
	struct hfi_cmd_session_set_property_packet *pkt =
		(struct hfi_cmd_session_set_property_packet *) &packet;
	struct hal_session *session = sess;
	struct venus_hfi_device *device;
	int rc = 0;

	if (!session || !session->device || !pdata) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}
<<<<<<< HEAD

	device = session->device;
	mutex_lock(&device->lock);

	dprintk(VIDC_INFO, "in set_prop,with prop id: %#x\n", ptype);
=======
	device = session->device;

	dprintk(VIDC_INFO, "in set_prop,with prop id: 0x%x\n", ptype);
>>>>>>> p9x

	rc = call_hfi_pkt_op(device, session_set_property,
			pkt, session, ptype, pdata);

	if (rc == -ENOTSUPP) {
		dprintk(VIDC_DBG,
<<<<<<< HEAD
			"set property: unsupported prop id: %#x\n", ptype);
		rc = 0;
		goto err_set_prop;
	} else if (rc) {
		dprintk(VIDC_ERR, "set property: failed to create packet\n");
		rc = -EINVAL;
		goto err_set_prop;
	}

	if (__iface_cmdq_write(session->device, pkt)) {
		rc = -ENOTEMPTY;
		goto err_set_prop;
	}

err_set_prop:
	mutex_unlock(&device->lock);
=======
			"%s Property not permitted ptype = 0x%x\n",
			__func__, ptype);
		return 0;
	} else if (rc) {
		dprintk(VIDC_ERR, "set property: failed to create packet\n");
		return -EINVAL;
	}

	if (venus_hfi_iface_cmdq_write(session->device, pkt))
		return -ENOTEMPTY;

>>>>>>> p9x
	return rc;
}

static int venus_hfi_session_get_property(void *sess,
					enum hal_property ptype)
{
	struct hfi_cmd_session_get_property_packet pkt = {0};
	struct hal_session *session = sess;
	int rc = 0;
	struct venus_hfi_device *device;

	if (!session || !session->device) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}
<<<<<<< HEAD

	device = session->device;
	mutex_lock(&device->lock);
=======
	device = session->device;
>>>>>>> p9x

	dprintk(VIDC_INFO, "%s: property id: %d\n", __func__, ptype);

	rc = call_hfi_pkt_op(device, session_get_property,
				&pkt, session, ptype);
	if (rc) {
		dprintk(VIDC_ERR, "get property profile: pkt failed\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	if (__iface_cmdq_write(session->device, &pkt)) {
		rc = -ENOTEMPTY;
		dprintk(VIDC_ERR, "%s cmdq_write error\n", __func__);
	}

err_create_pkt:
	mutex_unlock(&device->lock);
	return rc;
}

static void __set_default_sys_properties(struct venus_hfi_device *device)
{
	if (__sys_set_debug(device, msm_vidc_fw_debug))
		dprintk(VIDC_WARN, "Setting fw_debug msg ON failed\n");
	if (__sys_set_idle_message(device,
		device->res->sys_idle_indicator || msm_vidc_sys_idle_indicator))
		dprintk(VIDC_WARN, "Setting idle response ON failed\n");
	if (__sys_set_power_control(device, msm_vidc_fw_low_power_mode))
		dprintk(VIDC_WARN, "Setting h/w power collapse ON failed\n");
}

static void __session_clean(struct hal_session *session)
{
	dprintk(VIDC_DBG, "deleted the session: %pK\n", session);
	list_del(&session->list);
	/* Poison the session handle with zeros */
	*session = (struct hal_session){ {0} };
	kfree(session);
}

=======
	if (venus_hfi_iface_cmdq_write(session->device, &pkt)) {
		rc = -ENOTEMPTY;
		dprintk(VIDC_ERR, "%s cmdq_write error\n", __func__);
	}
err_create_pkt:
	return rc;
}

static void venus_hfi_set_default_sys_properties(
		struct venus_hfi_device *device)
{
	if (venus_hfi_sys_set_debug(device, msm_fw_debug))
		dprintk(VIDC_WARN, "Setting fw_debug msg ON failed\n");
	if (venus_hfi_sys_set_idle_message(device,
		device->res->sys_idle_indicator || msm_vidc_sys_idle_indicator))
		dprintk(VIDC_WARN, "Setting idle response ON failed\n");
	if (venus_hfi_sys_set_power_control(device, msm_fw_low_power_mode))
		dprintk(VIDC_WARN, "Setting h/w power collapse ON failed\n");
}

>>>>>>> p9x
static int venus_hfi_session_clean(void *session)
{
	struct hal_session *sess_close;
	struct venus_hfi_device *device;
	if (!session) {
		dprintk(VIDC_ERR, "Invalid Params %s\n", __func__);
		return -EINVAL;
	}
<<<<<<< HEAD

	sess_close = session;
	device = sess_close->device;

	if (!device) {
		dprintk(VIDC_ERR, "Invalid device handle %s\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&device->lock);

	__session_clean(sess_close);
	__flush_debug_queue(device, NULL);

	mutex_unlock(&device->lock);
	return 0;
}

static int venus_hfi_session_init(void *device, void *session_id,
		enum hal_domain session_type, enum hal_video_codec codec_type,
		void **new_session)
{
	struct hfi_cmd_sys_session_init_packet pkt;
	struct venus_hfi_device *dev;
	struct hal_session *s;

	if (!device || !new_session) {
		dprintk(VIDC_ERR, "%s - invalid input\n", __func__);
		return -EINVAL;
	}

	dev = device;
	mutex_lock(&dev->lock);

	s = kzalloc(sizeof(struct hal_session), GFP_KERNEL);
	if (!s) {
		dprintk(VIDC_ERR, "new session fail: Out of memory\n");
		goto err_session_init_fail;
	}

	s->session_id = session_id;
	s->is_decoder = (session_type == HAL_VIDEO_DOMAIN_DECODER);
	s->device = dev;
	s->codec = codec_type;
	s->domain = session_type;
	dprintk(VIDC_DBG,
		"%s: inst %pK, session %pK, codec 0x%x, domain 0x%x\n",
		__func__, session_id, s, s->codec, s->domain);

	list_add_tail(&s->list, &dev->sess_head);

	__set_default_sys_properties(device);

	if (call_hfi_pkt_op(dev, session_init, &pkt,
			s, session_type, codec_type)) {
=======
	sess_close = session;
	device = sess_close->device;
	venus_hfi_flush_debug_queue(sess_close->device, NULL);
	dprintk(VIDC_DBG, "deleted the session: 0x%pK\n",
			sess_close);
	mutex_lock(&device->session_lock);
	list_del(&sess_close->list);
	kfree(sess_close);
	mutex_unlock(&device->session_lock);
	return 0;
}

static void *venus_hfi_session_init(void *device, void *session_id,
		enum hal_domain session_type, enum hal_video_codec codec_type)
{
	struct hfi_cmd_sys_session_init_packet pkt;
	struct hal_session *new_session;
	struct venus_hfi_device *dev;

	if (device) {
		dev = device;
	} else {
		dprintk(VIDC_ERR, "invalid device\n");
		return NULL;
	}

	new_session = (struct hal_session *)
		kzalloc(sizeof(struct hal_session), GFP_KERNEL);
	if (!new_session) {
		dprintk(VIDC_ERR, "new session fail: Out of memory\n");
		return NULL;
	}
	new_session->session_id = session_id;
	if (session_type == 1)
		new_session->is_decoder = 0;
	else if (session_type == 2)
		new_session->is_decoder = 1;
	new_session->device = dev;
	new_session->codec = codec_type;
	new_session->domain = session_type;
	dprintk(VIDC_DBG,
		"%s: inst %pK, session %pK, codec 0x%x, domain 0x%x\n",
		__func__, session_id, new_session,
		new_session->codec, new_session->domain);

	mutex_lock(&dev->session_lock);
	list_add_tail(&new_session->list, &dev->sess_head);
	mutex_unlock(&dev->session_lock);

	venus_hfi_set_default_sys_properties(device);

	if (call_hfi_pkt_op(dev, session_init, &pkt,
			new_session, session_type, codec_type)) {
>>>>>>> p9x
		dprintk(VIDC_ERR, "session_init: failed to create packet\n");
		goto err_session_init_fail;
	}

<<<<<<< HEAD
	*new_session = s;
	if (__iface_cmdq_write(dev, &pkt))
		goto err_session_init_fail;

	mutex_unlock(&dev->lock);
	return 0;

err_session_init_fail:
	if (s)
		__session_clean(s);
	*new_session = NULL;
	mutex_unlock(&dev->lock);
	return -EINVAL;
}

static int __send_session_cmd(struct hal_session *session, int pkt_type)
{
	struct vidc_hal_session_cmd_pkt pkt;
	int rc = 0;
	struct venus_hfi_device *device = session->device;
=======
	if (venus_hfi_iface_cmdq_write(dev, &pkt))
		goto err_session_init_fail;
	return (void *) new_session;

err_session_init_fail:
	venus_hfi_session_clean(new_session);
	return NULL;
}

static int venus_hfi_send_session_cmd(void *session_id,
	 int pkt_type)
{
	struct vidc_hal_session_cmd_pkt pkt;
	int rc = 0;
	struct hal_session *session = session_id;
	struct venus_hfi_device *device;

	if (!session || !session->device) {
		dprintk(VIDC_ERR, "invalid session");
		return -ENODEV;
	}
	device = session->device;
>>>>>>> p9x

	rc = call_hfi_pkt_op(device, session_cmd,
			&pkt, pkt_type, session);
	if (rc == -EPERM)
		return 0;

	if (rc) {
		dprintk(VIDC_ERR, "send session cmd: create pkt failed\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	if (__iface_cmdq_write(session->device, &pkt))
=======
	if (venus_hfi_iface_cmdq_write(session->device, &pkt))
>>>>>>> p9x
		rc = -ENOTEMPTY;

err_create_pkt:
	return rc;
}

static int venus_hfi_session_end(void *session)
{
	struct hal_session *sess;
<<<<<<< HEAD
	struct venus_hfi_device *device;
	int rc = 0;

=======
>>>>>>> p9x
	if (!session) {
		dprintk(VIDC_ERR, "Invalid Params %s\n", __func__);
		return -EINVAL;
	}
<<<<<<< HEAD

	sess = session;
	device = sess->device;

	mutex_lock(&device->lock);

	if (msm_vidc_fw_coverage) {
		if (__sys_set_coverage(sess->device, msm_vidc_fw_coverage))
			dprintk(VIDC_WARN, "Fw_coverage msg ON failed\n");
	}

	rc = __send_session_cmd(session, HFI_CMD_SYS_SESSION_END);

	mutex_unlock(&device->lock);

	return rc;
=======
	sess = session;
	if (msm_fw_coverage) {
		if (venus_hfi_sys_set_coverage(sess->device,
				msm_fw_coverage))
			dprintk(VIDC_WARN, "Fw_coverage msg ON failed\n");
	}
	return venus_hfi_send_session_cmd(session,
		HFI_CMD_SYS_SESSION_END);
>>>>>>> p9x
}

static int venus_hfi_session_abort(void *sess)
{
	struct hal_session *session;
<<<<<<< HEAD
	struct venus_hfi_device *device;
	int rc = 0;
	session = sess;

	if (!session || !session->device) {
		dprintk(VIDC_ERR, "Invalid Params %s\n", __func__);
		return -EINVAL;
	}

	device = session->device;

	mutex_lock(&device->lock);

	__flush_debug_queue(device, NULL);
	rc = __send_session_cmd(session, HFI_CMD_SYS_SESSION_ABORT);

	mutex_unlock(&device->lock);

	return rc;
=======
	session = sess;
	if (!session || !session->device) {
		dprintk(VIDC_ERR, "%s: Invalid Params %pK\n",
			__func__, session);
		return -EINVAL;
	}
	venus_hfi_flush_debug_queue(
		((struct hal_session *)session)->device, NULL);
	return venus_hfi_send_session_cmd(session,
		HFI_CMD_SYS_SESSION_ABORT);
>>>>>>> p9x
}

static int venus_hfi_session_set_buffers(void *sess,
				struct vidc_buffer_addr_info *buffer_info)
{
	struct hfi_cmd_session_set_buffers_packet *pkt;
	u8 packet[VIDC_IFACEQ_VAR_LARGE_PKT_SIZE];
	int rc = 0;
	struct hal_session *session = sess;
	struct venus_hfi_device *device;

	if (!session || !session->device || !buffer_info) {
<<<<<<< HEAD
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}

	device = session->device;
	mutex_lock(&device->lock);

	if (buffer_info->buffer_type == HAL_BUFFER_INPUT) {
		/*
		 * Hardware doesn't care about input buffers being
		 * published beforehand
		 */
		rc = 0;
		goto err_create_pkt;
	}
=======
		dprintk(VIDC_ERR, "%s: Invalid Params, %pK %pK\n",
			__func__, session, buffer_info);
		return -EINVAL;
	}
	device = session->device;

	if (buffer_info->buffer_type == HAL_BUFFER_INPUT)
		return 0;
>>>>>>> p9x

	pkt = (struct hfi_cmd_session_set_buffers_packet *)packet;

	rc = call_hfi_pkt_op(device, session_set_buffers,
			pkt, session, buffer_info);
	if (rc) {
		dprintk(VIDC_ERR, "set buffers: failed to create packet\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	dprintk(VIDC_INFO, "set buffers: %#x\n", buffer_info->buffer_type);
	if (__iface_cmdq_write(session->device, pkt))
		rc = -ENOTEMPTY;

err_create_pkt:
	mutex_unlock(&device->lock);
=======
	dprintk(VIDC_INFO, "set buffers: 0x%x\n", buffer_info->buffer_type);
	if (venus_hfi_iface_cmdq_write(session->device, pkt))
		rc = -ENOTEMPTY;
err_create_pkt:
>>>>>>> p9x
	return rc;
}

static int venus_hfi_session_release_buffers(void *sess,
				struct vidc_buffer_addr_info *buffer_info)
{
	struct hfi_cmd_session_release_buffer_packet *pkt;
	u8 packet[VIDC_IFACEQ_VAR_LARGE_PKT_SIZE];
	int rc = 0;
	struct hal_session *session = sess;
	struct venus_hfi_device *device;

	if (!session || !session->device || !buffer_info) {
<<<<<<< HEAD
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}

	device = session->device;
	mutex_lock(&device->lock);

	if (buffer_info->buffer_type == HAL_BUFFER_INPUT) {
		rc = 0;
		goto err_create_pkt;
	}
=======
		dprintk(VIDC_ERR, "%s: Invalid Params %pK, %pK\n",
			__func__, session, buffer_info);
		return -EINVAL;
	}
	device = session->device;

	if (buffer_info->buffer_type == HAL_BUFFER_INPUT)
		return 0;
>>>>>>> p9x

	pkt = (struct hfi_cmd_session_release_buffer_packet *) packet;

	rc = call_hfi_pkt_op(device, session_release_buffers,
			pkt, session, buffer_info);
	if (rc) {
		dprintk(VIDC_ERR, "release buffers: failed to create packet\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	dprintk(VIDC_INFO, "Release buffers: %#x\n", buffer_info->buffer_type);
	if (__iface_cmdq_write(session->device, pkt))
		rc = -ENOTEMPTY;

err_create_pkt:
	mutex_unlock(&device->lock);
	return rc;
}

static int venus_hfi_session_load_res(void *session)
{
	struct hal_session *sess;
	struct venus_hfi_device *device;
	int rc = 0;

	if (!session) {
		dprintk(VIDC_ERR, "Invalid Params %s\n", __func__);
		return -EINVAL;
	}

	sess = session;
	device = sess->device;

	mutex_lock(&device->lock);
	rc = __send_session_cmd(sess, HFI_CMD_SESSION_LOAD_RESOURCES);
	mutex_unlock(&device->lock);

	return rc;
}

static int venus_hfi_session_release_res(void *session)
{
	struct hal_session *sess;
	struct venus_hfi_device *device;
	int rc = 0;

	if (!session) {
		dprintk(VIDC_ERR, "Invalid Params %s\n", __func__);
		return -EINVAL;
	}

	sess = session;
	device = sess->device;

	mutex_lock(&device->lock);
	rc = __send_session_cmd(sess, HFI_CMD_SESSION_RELEASE_RESOURCES);
	mutex_unlock(&device->lock);

	return rc;
}

static int venus_hfi_session_start(void *session)
{
	struct hal_session *sess;
	struct venus_hfi_device *device;
	int rc = 0;

	if (!session) {
		dprintk(VIDC_ERR, "Invalid Params %s\n", __func__);
		return -EINVAL;
	}

	sess = session;
	device = sess->device;

	mutex_lock(&device->lock);
	rc = __send_session_cmd(sess, HFI_CMD_SESSION_START);
	mutex_unlock(&device->lock);

	return rc;
}

static int venus_hfi_session_continue(void *session)
{
	struct hal_session *sess;
	struct venus_hfi_device *device;
	int rc = 0;

	if (!session) {
		dprintk(VIDC_ERR, "Invalid Params %s\n", __func__);
		return -EINVAL;
	}

	sess = session;
	device = sess->device;

	mutex_lock(&device->lock);
	rc = __send_session_cmd(sess, HFI_CMD_SESSION_CONTINUE);
	mutex_unlock(&device->lock);

	return rc;
}

static int venus_hfi_session_stop(void *session)
{
	struct hal_session *sess;
	struct venus_hfi_device *device;
	int rc = 0;

	if (!session) {
		dprintk(VIDC_ERR, "Invalid Params %s\n", __func__);
		return -EINVAL;
	}

	sess = session;
	device = sess->device;

	mutex_lock(&device->lock);
	rc = __send_session_cmd(sess, HFI_CMD_SESSION_STOP);
	mutex_unlock(&device->lock);

	return rc;
}

static int __session_etb(struct hal_session *session,
		struct vidc_frame_data *input_frame, bool relaxed)
{
	int rc = 0;
	struct venus_hfi_device *device = session->device;

	if (session->is_decoder) {
		struct hfi_cmd_session_empty_buffer_compressed_packet pkt;

		rc = call_hfi_pkt_op(device, session_etb_decoder,
				&pkt, session, input_frame);
		if (rc) {
			dprintk(VIDC_ERR,
					"Session etb decoder: failed to create pkt\n");
			goto err_create_pkt;
		}

		if (!relaxed)
			rc = __iface_cmdq_write(session->device, &pkt);
		else
			rc = __iface_cmdq_write_relaxed(session->device,
					&pkt, NULL);
		if (rc)
			goto err_create_pkt;
	} else {
		struct hfi_cmd_session_empty_buffer_uncompressed_plane0_packet
			pkt;

		rc = call_hfi_pkt_op(device, session_etb_encoder,
					 &pkt, session, input_frame);
		if (rc) {
			dprintk(VIDC_ERR,
					"Session etb encoder: failed to create pkt\n");
			goto err_create_pkt;
		}

		if (!relaxed)
			rc = __iface_cmdq_write(session->device, &pkt);
		else
			rc = __iface_cmdq_write_relaxed(session->device,
					&pkt, NULL);
		if (rc)
			goto err_create_pkt;
	}

=======
	dprintk(VIDC_INFO, "Release buffers: 0x%x\n", buffer_info->buffer_type);
	if (venus_hfi_iface_cmdq_write(session->device, pkt))
		rc = -ENOTEMPTY;
>>>>>>> p9x
err_create_pkt:
	return rc;
}

<<<<<<< HEAD
=======
static int venus_hfi_session_load_res(void *sess)
{
	return venus_hfi_send_session_cmd(sess,
		HFI_CMD_SESSION_LOAD_RESOURCES);
}

static int venus_hfi_session_release_res(void *sess)
{
	return venus_hfi_send_session_cmd(sess,
		HFI_CMD_SESSION_RELEASE_RESOURCES);
}

static int venus_hfi_session_start(void *sess)
{
	return venus_hfi_send_session_cmd(sess,
		HFI_CMD_SESSION_START);
}

static inline int venus_hfi_session_continue(void *sess)
{
	return venus_hfi_send_session_cmd(sess,
		HFI_CMD_SESSION_CONTINUE);
}

static int venus_hfi_session_stop(void *sess)
{
	return venus_hfi_send_session_cmd(sess,
		HFI_CMD_SESSION_STOP);
}

>>>>>>> p9x
static int venus_hfi_session_etb(void *sess,
				struct vidc_frame_data *input_frame)
{
	int rc = 0;
	struct hal_session *session = sess;
	struct venus_hfi_device *device;

	if (!session || !session->device || !input_frame) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}
<<<<<<< HEAD

	device = session->device;
	mutex_lock(&device->lock);
	rc = __session_etb(session, input_frame, false);
	mutex_unlock(&device->lock);
	return rc;
}

static int __session_ftb(struct hal_session *session,
		struct vidc_frame_data *output_frame, bool relaxed)
{
	int rc = 0;
	struct venus_hfi_device *device = session->device;
	struct hfi_cmd_session_fill_buffer_packet pkt;

	rc = call_hfi_pkt_op(device, session_ftb,
			&pkt, session, output_frame);
	if (rc) {
		dprintk(VIDC_ERR, "Session ftb: failed to create pkt\n");
		goto err_create_pkt;
	}

	if (!relaxed)
		rc = __iface_cmdq_write(session->device, &pkt);
	else
		rc = __iface_cmdq_write_relaxed(session->device,
				&pkt, NULL);

=======
	device = session->device;

	if (session->is_decoder) {
		struct hfi_cmd_session_empty_buffer_compressed_packet pkt;

		rc = call_hfi_pkt_op(device, session_etb_decoder,
				&pkt, session, input_frame);
		if (rc) {
			dprintk(VIDC_ERR,
			"Session etb decoder: failed to create pkt\n");
			goto err_create_pkt;
		}
		dprintk(VIDC_DBG, "Q DECODER INPUT BUFFER\n");
		if (venus_hfi_iface_cmdq_write(session->device, &pkt))
			rc = -ENOTEMPTY;
	} else {
		struct hfi_cmd_session_empty_buffer_uncompressed_plane0_packet
			pkt;


		rc = call_hfi_pkt_op(device, session_etb_encoder,
					 &pkt, session, input_frame);
		if (rc) {
			dprintk(VIDC_ERR,
			"Session etb encoder: failed to create pkt\n");
			goto err_create_pkt;
		}
		dprintk(VIDC_DBG, "Q ENCODER INPUT BUFFER\n");
		if (venus_hfi_iface_cmdq_write(session->device, &pkt))
			rc = -ENOTEMPTY;
	}
>>>>>>> p9x
err_create_pkt:
	return rc;
}

static int venus_hfi_session_ftb(void *sess,
				struct vidc_frame_data *output_frame)
{
<<<<<<< HEAD
=======
	struct hfi_cmd_session_fill_buffer_packet pkt;
>>>>>>> p9x
	int rc = 0;
	struct hal_session *session = sess;
	struct venus_hfi_device *device;

	if (!session || !session->device || !output_frame) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}
<<<<<<< HEAD

	device = session->device;
	mutex_lock(&device->lock);
	rc = __session_ftb(session, output_frame, false);
	mutex_unlock(&device->lock);
	return rc;
}

static int venus_hfi_session_process_batch(void *sess,
		int num_etbs, struct vidc_frame_data etbs[],
		int num_ftbs, struct vidc_frame_data ftbs[])
{
	int rc = 0, c = 0;
	struct hal_session *session = sess;
	struct venus_hfi_device *device;
	struct hfi_cmd_session_sync_process_packet pkt;

	if (!session || !session->device) {
		dprintk(VIDC_ERR, "%s: Invalid Params\n", __func__);
		return -EINVAL;
	}

	device = session->device;

	mutex_lock(&device->lock);
	for (c = 0; c < num_ftbs; ++c) {
		rc = __session_ftb(session, &ftbs[c], true);
		if (rc) {
			dprintk(VIDC_ERR, "Failed to queue batched ftb: %d\n",
					rc);
			goto err_etbs_and_ftbs;
		}
	}

	for (c = 0; c < num_etbs; ++c) {
		rc = __session_etb(session, &etbs[c], true);
		if (rc) {
			dprintk(VIDC_ERR, "Failed to queue batched etb: %d\n",
					rc);
			goto err_etbs_and_ftbs;
		}
	}

	rc = call_hfi_pkt_op(device, session_sync_process, &pkt, session);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to create sync packet\n");
		goto err_etbs_and_ftbs;
	}

	if (__iface_cmdq_write(session->device, &pkt))
		rc = -ENOTEMPTY;

err_etbs_and_ftbs:
	mutex_unlock(&device->lock);
=======
	device = session->device;

	rc = call_hfi_pkt_op(device, session_ftb,
			&pkt, session, output_frame);
	if (rc) {
		dprintk(VIDC_ERR, "Session ftb: failed to create pkt\n");
		goto err_create_pkt;
	}

	if (venus_hfi_iface_cmdq_write(session->device, &pkt))
		rc = -ENOTEMPTY;
err_create_pkt:
>>>>>>> p9x
	return rc;
}

static int venus_hfi_session_parse_seq_hdr(void *sess,
					struct vidc_seq_hdr *seq_hdr)
{
	struct hfi_cmd_session_parse_sequence_header_packet *pkt;
	int rc = 0;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	struct hal_session *session = sess;
	struct venus_hfi_device *device;

	if (!session || !session->device || !seq_hdr) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}
<<<<<<< HEAD

	device = session->device;
	mutex_lock(&device->lock);

	pkt = (struct hfi_cmd_session_parse_sequence_header_packet *)packet;
=======
	device = session->device;

	pkt = (struct hfi_cmd_session_parse_sequence_header_packet *) packet;

>>>>>>> p9x
	rc = call_hfi_pkt_op(device, session_parse_seq_header,
			pkt, session, seq_hdr);
	if (rc) {
		dprintk(VIDC_ERR,
		"Session parse seq hdr: failed to create pkt\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	if (__iface_cmdq_write(session->device, pkt))
		rc = -ENOTEMPTY;
err_create_pkt:
	mutex_unlock(&device->lock);
=======
	if (venus_hfi_iface_cmdq_write(session->device, pkt))
		rc = -ENOTEMPTY;
err_create_pkt:
>>>>>>> p9x
	return rc;
}

static int venus_hfi_session_get_seq_hdr(void *sess,
				struct vidc_seq_hdr *seq_hdr)
{
	struct hfi_cmd_session_get_sequence_header_packet *pkt;
	int rc = 0;
	u8 packet[VIDC_IFACEQ_VAR_SMALL_PKT_SIZE];
	struct hal_session *session = sess;
	struct venus_hfi_device *device;

	if (!session || !session->device || !seq_hdr) {
		dprintk(VIDC_ERR, "Invalid Params\n");
		return -EINVAL;
	}
<<<<<<< HEAD

	device = session->device;
	mutex_lock(&device->lock);

	pkt = (struct hfi_cmd_session_get_sequence_header_packet *)packet;
=======
	device = session->device;

	pkt = (struct hfi_cmd_session_get_sequence_header_packet *) packet;

>>>>>>> p9x
	rc = call_hfi_pkt_op(device, session_get_seq_hdr,
			pkt, session, seq_hdr);
	if (rc) {
		dprintk(VIDC_ERR,
				"Session get seq hdr: failed to create pkt\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	if (__iface_cmdq_write(session->device, pkt))
		rc = -ENOTEMPTY;
err_create_pkt:
	mutex_unlock(&device->lock);
=======
	if (venus_hfi_iface_cmdq_write(session->device, pkt))
		rc = -ENOTEMPTY;
err_create_pkt:
>>>>>>> p9x
	return rc;
}

static int venus_hfi_session_get_buf_req(void *sess)
{
	struct hfi_cmd_session_get_property_packet pkt;
	int rc = 0;
	struct hal_session *session = sess;
	struct venus_hfi_device *device;

	if (!session || !session->device) {
		dprintk(VIDC_ERR, "invalid session");
		return -ENODEV;
	}
<<<<<<< HEAD

	device = session->device;
	mutex_lock(&device->lock);
=======
	device = session->device;
>>>>>>> p9x

	rc = call_hfi_pkt_op(device, session_get_buf_req,
			&pkt, session);
	if (rc) {
		dprintk(VIDC_ERR,
				"Session get buf req: failed to create pkt\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	if (__iface_cmdq_write(session->device, &pkt))
		rc = -ENOTEMPTY;
err_create_pkt:
	mutex_unlock(&device->lock);
=======
	if (venus_hfi_iface_cmdq_write(session->device, &pkt))
		rc = -ENOTEMPTY;
err_create_pkt:
>>>>>>> p9x
	return rc;
}

static int venus_hfi_session_flush(void *sess, enum hal_flush flush_mode)
{
	struct hfi_cmd_session_flush_packet pkt;
	int rc = 0;
	struct hal_session *session = sess;
	struct venus_hfi_device *device;

	if (!session || !session->device) {
		dprintk(VIDC_ERR, "invalid session");
		return -ENODEV;
	}
<<<<<<< HEAD

	device = session->device;
	mutex_lock(&device->lock);
=======
	device = session->device;
>>>>>>> p9x

	rc = call_hfi_pkt_op(device, session_flush,
			&pkt, session, flush_mode);
	if (rc) {
		dprintk(VIDC_ERR, "Session flush: failed to create pkt\n");
		goto err_create_pkt;
	}

<<<<<<< HEAD
	if (__iface_cmdq_write(session->device, &pkt))
		rc = -ENOTEMPTY;
err_create_pkt:
	mutex_unlock(&device->lock);
	return rc;
}

static int __check_core_registered(struct hal_device_data core,
		phys_addr_t fw_addr, u8 *reg_addr, u32 reg_size,
		phys_addr_t irq)
=======
	if (venus_hfi_iface_cmdq_write(session->device, &pkt))
		rc = -ENOTEMPTY;
err_create_pkt:
	return rc;
}

static int venus_hfi_check_core_registered(
	struct hal_device_data core, phys_addr_t fw_addr,
	u8 *reg_addr, u32 reg_size, phys_addr_t irq)
>>>>>>> p9x
{
	struct venus_hfi_device *device;
	struct list_head *curr, *next;

	if (core.dev_count) {
		list_for_each_safe(curr, next, &core.dev_head) {
			device = list_entry(curr,
				struct venus_hfi_device, list);
			if (device && device->hal_data->irq == irq &&
				(CONTAINS(device->hal_data->
						firmware_base,
						FIRMWARE_SIZE, fw_addr) ||
				CONTAINS(fw_addr, FIRMWARE_SIZE,
						device->hal_data->
						firmware_base) ||
				CONTAINS(device->hal_data->
						register_base,
						reg_size, reg_addr) ||
				CONTAINS(reg_addr, reg_size,
						device->hal_data->
						register_base) ||
				OVERLAPS(device->hal_data->
						register_base,
						reg_size, reg_addr, reg_size) ||
				OVERLAPS(reg_addr, reg_size,
						device->hal_data->
						register_base, reg_size) ||
				OVERLAPS(device->hal_data->
						firmware_base,
						FIRMWARE_SIZE, fw_addr,
						FIRMWARE_SIZE) ||
				OVERLAPS(fw_addr, FIRMWARE_SIZE,
						device->hal_data->
						firmware_base,
						FIRMWARE_SIZE))) {
				return 0;
			} else {
				dprintk(VIDC_INFO, "Device not registered\n");
				return -EINVAL;
			}
		}
	} else {
		dprintk(VIDC_INFO, "no device Registered\n");
	}
<<<<<<< HEAD

	return -EINVAL;
}

static void __process_fatal_error(
		struct venus_hfi_device *device)
{
	struct msm_vidc_cb_cmd_done cmd_done = {0};

	cmd_done.device_id = device->device_id;
	device->callback(HAL_SYS_ERROR, &cmd_done);
}

static int __prepare_pc(struct venus_hfi_device *device)
{
	int rc = 0;
	struct hfi_cmd_sys_pc_prep_packet pkt;

	rc = call_hfi_pkt_op(device, sys_pc_prep, &pkt);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to create sys pc prep pkt\n");
		goto err_pc_prep;
	}

	if (__iface_cmdq_write(device, &pkt))
		rc = -ENOTEMPTY;
	if (rc)
		dprintk(VIDC_ERR, "Failed to prepare venus for power off");
=======
	return -EINVAL;
}

static void venus_hfi_process_sys_watchdog_timeout(
				struct venus_hfi_device *device)
{
	struct msm_vidc_cb_cmd_done cmd_done;
	memset(&cmd_done, 0, sizeof(struct msm_vidc_cb_cmd_done));
	cmd_done.device_id = device->device_id;
	device->callback(SYS_WATCHDOG_TIMEOUT, &cmd_done);
}

static int venus_hfi_core_pc_prep(void *device)
{
	struct hfi_cmd_sys_pc_prep_packet pkt;
	int rc = 0;
	struct venus_hfi_device *dev = device;

	if (!dev) {
		dprintk(VIDC_ERR, "invalid device\n");
		return -ENODEV;
	}

	rc = call_hfi_pkt_op(dev, sys_pc_prep, &pkt);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to create sys pc prep pkt\n");
		goto err_create_pkt;
	}
	/* Calling write_nolock() with write_lock instead of write()
	*  because write() will cancel and rescheduling power collapse.
	*/
	mutex_lock(&dev->write_lock);
	if (venus_hfi_iface_cmdq_write_nolock(dev, &pkt))
		rc = -ENOTEMPTY;
	mutex_unlock(&dev->write_lock);

err_create_pkt:
	return rc;
}

static int venus_hfi_prepare_pc(struct venus_hfi_device *device)
{
	int rc = 0;
	init_completion(&pc_prep_done);
	rc = venus_hfi_core_pc_prep(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to prepare venus for power off");
		goto err_pc_prep;
	}
	rc = wait_for_completion_timeout(&pc_prep_done,
			msecs_to_jiffies(msm_vidc_hw_rsp_timeout));
	if (!rc) {
		dprintk(VIDC_ERR,
				"Wait interrupted or timeout for PC_PREP_DONE: %d\n",
				rc);
		venus_hfi_flush_debug_queue(device, NULL);
		rc = -EIO;
		goto err_pc_prep;
	}
	rc = 0;
>>>>>>> p9x
err_pc_prep:
	return rc;
}

<<<<<<< HEAD
static void venus_hfi_pm_handler(struct work_struct *work)
{
	int rc = 0;
	u32 wfi_status = 0, idle_status = 0, pc_ready = 0;
	int count = 0;
	const int max_tries = 5;
=======
static void venus_hfi_pm_hndlr(struct work_struct *work)
{
	int rc = 0;
	u32 ctrl_status = 0;
>>>>>>> p9x
	struct venus_hfi_device *device = list_first_entry(
			&hal_ctxt.dev_head, struct venus_hfi_device, list);
	if (!device) {
		dprintk(VIDC_ERR, "%s: NULL device\n", __func__);
		return;
	}

<<<<<<< HEAD
	/*
	 * It is ok to check this variable outside the lock since
	 * it is being updated in this context only
	 */
	if (device->skip_pc_count >= VIDC_MAX_PC_SKIP_COUNT) {
		dprintk(VIDC_WARN, "Failed to PC for %d times\n",
				device->skip_pc_count);
		device->skip_pc_count = 0;
		__process_fatal_error(device);
		return;
	}
	mutex_lock(&device->lock);
	if (!device->power_enabled) {
		dprintk(VIDC_DBG, "%s: Power already disabled\n",
				__func__);
		goto exit;
	}

	rc = __core_in_valid_state(device);
	if (!rc) {
		dprintk(VIDC_WARN,
				"Core is in bad state, Skipping power collapse\n");
		goto skip_power_off;
	}
	pc_ready = __read_register(device, VIDC_CPU_CS_SCIACMDARG0) &
		VIDC_CPU_CS_SCIACMDARG0_HFI_CTRL_PC_READY;
	if (!pc_ready) {
		wfi_status = __read_register(device,
				VIDC_WRAPPER_CPU_STATUS);
		idle_status = __read_register(device,
				VIDC_CPU_CS_SCIACMDARG0);
		if (!(wfi_status & BIT(0)) ||
				!(idle_status & BIT(30))) {
			dprintk(VIDC_WARN, "Skipping PC\n");
			goto skip_power_off;
		}

		rc = __prepare_pc(device);
		if (rc) {
			dprintk(VIDC_WARN, "Failed PC %d\n", rc);
			goto skip_power_off;
		}

		while (count < max_tries) {
			wfi_status = __read_register(device,
					VIDC_WRAPPER_CPU_STATUS);
			pc_ready = __read_register(device,
					VIDC_CPU_CS_SCIACMDARG0);
			if ((wfi_status & BIT(0)) && (pc_ready &
				VIDC_CPU_CS_SCIACMDARG0_HFI_CTRL_PC_READY))
				break;
			usleep_range(1000, 1500);
			count++;
		}

		if (count == max_tries) {
			dprintk(VIDC_ERR,
					"Skip PC. Core is not in right state (%#x, %#x)\n",
					wfi_status, pc_ready);
			goto skip_power_off;
		}
	}

	rc = __suspend(device);
	if (rc)
		dprintk(VIDC_ERR, "Failed venus power off\n");

	/* Cancel pending delayed works if any */
	cancel_delayed_work(&venus_hfi_pm_work);
	device->skip_pc_count = 0;

	mutex_unlock(&device->lock);
	return;

skip_power_off:
	device->skip_pc_count++;
	dprintk(VIDC_WARN, "Skip PC(%d, %#x, %#x, %#x)\n",
		device->skip_pc_count, wfi_status, idle_status, pc_ready);
	queue_delayed_work(device->venus_pm_workq,
			&venus_hfi_pm_work,
			msecs_to_jiffies(msm_vidc_pwr_collapse_delay));
exit:
	mutex_unlock(&device->lock);
	return;
}

static void __dump_venus_debug_registers(struct venus_hfi_device *device)
{
	u32 reg;

	dprintk(VIDC_ERR, "Dumping Venus registers...\n");
	reg = __read_register(device, VENUS_VBIF_XIN_HALT_CTRL1);
	dprintk(VIDC_ERR, "VENUS_VBIF_XIN_HALT_CTRL1: 0x%x\n", reg);

	reg = __read_register(device,
		VIDC_VENUS_WRAPPER_MMCC_VENUS0_POWER_STATUS);
	dprintk(VIDC_ERR,
		"VIDC_VENUS_WRAPPER_MMCC_VENUS0_POWER_STATUS: 0x%x\n", reg);

	reg = __read_register(device, VIDC_WRAPPER_CPU_STATUS);
	dprintk(VIDC_ERR, "VIDC_WRAPPER_CPU_STATUS: 0x%x\n", reg);

	reg = __read_register(device, VIDC_CPU_CS_SCIACMDARG0);
	dprintk(VIDC_ERR, "VIDC_CPU_CS_SCIACMDARG0: 0x%x\n", reg);
}

static void __process_sys_error(struct venus_hfi_device *device)
{
	struct hfi_sfr_struct *vsfr = NULL;

	/* Once SYS_ERROR received from HW, it is safe to halt the AXI.
	 * With SYS_ERROR, Venus FW may have crashed and HW might be
	 * active and causing unnecessary transactions. Hence it is
	 * safe to stop all AXI transactions from venus sub-system. */
	if (__halt_axi(device))
		dprintk(VIDC_WARN, "Failed to halt AXI after SYS_ERROR\n");

	vsfr = (struct hfi_sfr_struct *)device->sfr.align_virtual_addr;
	if (vsfr) {
		void *p = memchr(vsfr->rg_data, '\0', vsfr->bufSize);
		/* SFR isn't guaranteed to be NULL terminated
		   since SYS_ERROR indicates that Venus is in the
		   process of crashing.*/
		if (p == NULL)
			vsfr->rg_data[vsfr->bufSize - 1] = '\0';

		dprintk(VIDC_ERR, "SFR Message from FW: %s\n",
				vsfr->rg_data);
	}
}

static void __flush_debug_queue(struct venus_hfi_device *device, u8 *packet)
{
=======
	if (!device->power_enabled) {
		dprintk(VIDC_DBG, "%s: Power already disabled\n",
				__func__);
		return;
	}

	mutex_lock(&device->write_lock);
	mutex_lock(&device->read_lock);
	rc = venus_hfi_core_in_valid_state(device);
	mutex_unlock(&device->read_lock);
	mutex_unlock(&device->write_lock);

	if (!rc) {
		dprintk(VIDC_WARN,
			"Core is in bad state, Skipping power collapse\n");
		return;
	}

	dprintk(VIDC_DBG, "Prepare for power collapse\n");

	rc = venus_hfi_prepare_pc(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to prepare for PC, rc : %d\n", rc);
		return;
	}

	mutex_lock(&device->write_lock);

	if (device->last_packet_type != HFI_CMD_SYS_PC_PREP) {
		dprintk(VIDC_DBG,
			"Last command (0x%x) is not PC_PREP cmd\n",
			device->last_packet_type);
		goto skip_power_off;
	}

	if (venus_hfi_get_q_size(device, VIDC_IFACEQ_MSGQ_IDX) ||
		venus_hfi_get_q_size(device, VIDC_IFACEQ_CMDQ_IDX)) {
		dprintk(VIDC_DBG, "Cmd/msg queues are not empty\n");
		goto skip_power_off;
	}

	ctrl_status = venus_hfi_read_register(device, VIDC_CPU_CS_SCIACMDARG0);
	if (!(ctrl_status & VIDC_CPU_CS_SCIACMDARG0_HFI_CTRL_PC_READY)) {
		dprintk(VIDC_DBG,
			"Venus is not ready for power collapse (0x%x)\n",
			ctrl_status);
		goto skip_power_off;
	}

	rc = venus_hfi_power_off(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed venus power off\n");
		goto err_power_off;
	}

	mutex_lock(&device->resource_lock);
	rc = __free_ocmem(device);
	mutex_unlock(&device->resource_lock);
	if (rc)
		dprintk(VIDC_ERR,
			"Failed to free OCMEM for PC, rc : %d\n", rc);

	/* Cancel pending delayed works if any */
	cancel_delayed_work(&venus_hfi_pm_work);

	mutex_unlock(&device->write_lock);
	return;

err_power_off:
skip_power_off:

	/*
	* When power collapse is escaped, driver no need to inform Venus.
	* Venus is self-sufficient to come out of the power collapse at
	* any stage. Driver can skip power collapse and continue with
	* normal execution.
	*/

	/* Cancel pending delayed works if any */
	cancel_delayed_work(&venus_hfi_pm_work);
	dprintk(VIDC_WARN, "Power off skipped (0x%x, 0x%x)\n",
		device->last_packet_type, ctrl_status);

	mutex_unlock(&device->write_lock);
	return;
}

static void venus_hfi_process_msg_event_notify(
	struct venus_hfi_device *device, void *packet)
{
	struct hfi_sfr_struct *vsfr = NULL;
	struct hfi_msg_event_notify_packet *event_pkt;
	struct vidc_hal_msg_pkt_hdr *msg_hdr;

	msg_hdr = (struct vidc_hal_msg_pkt_hdr *)packet;
	event_pkt =
		(struct hfi_msg_event_notify_packet *)msg_hdr;
	if (event_pkt && event_pkt->event_id ==
		HFI_EVENT_SYS_ERROR) {

		venus_hfi_set_state(device, VENUS_STATE_DEINIT);

		/* Once SYS_ERROR received from HW, it is safe to halt the AXI.
		 * With SYS_ERROR, Venus FW may have crashed and HW might be
		 * active and causing unnecessary transactions. Hence it is
		 * safe to stop all AXI transactions from venus sub-system. */
		if (venus_hfi_halt_axi(device))
			dprintk(VIDC_WARN,
				"Failed to halt AXI after SYS_ERROR\n");

		vsfr = (struct hfi_sfr_struct *)
				device->sfr.align_virtual_addr;
		if (vsfr) {
			void *p = memchr(vsfr->rg_data, '\0',
							vsfr->bufSize);
			/* SFR isn't guaranteed to be NULL terminated
			since SYS_ERROR indicates that Venus is in the
			process of crashing.*/
			if (p == NULL)
				vsfr->rg_data[vsfr->bufSize - 1] = '\0';
			dprintk(VIDC_ERR, "SFR Message from FW : %s\n",
				vsfr->rg_data);
		}
	}
}

static void venus_hfi_flush_debug_queue(
	struct venus_hfi_device *device, u8 *packet)
{
	struct hfi_msg_sys_coverage_packet *pkt = NULL;
>>>>>>> p9x
	bool local_packet = false;

	if (!device) {
		dprintk(VIDC_ERR, "%s: Invalid params\n", __func__);
		return;
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	if (!packet) {
		packet = kzalloc(VIDC_IFACEQ_VAR_HUGE_PKT_SIZE, GFP_TEMPORARY);
		if (!packet) {
			dprintk(VIDC_ERR, "In %s() Fail to allocate mem\n",
				__func__);
			return;
		}
<<<<<<< HEAD

		local_packet = true;
	}

	while (!__iface_dbgq_read(device, packet)) {
		struct hfi_msg_sys_coverage_packet *pkt =
			(struct hfi_msg_sys_coverage_packet *) packet;
		if (pkt->packet_type == HFI_MSG_SYS_COV) {
			int stm_size = 0;
=======
		local_packet = true;
	}

	while (!venus_hfi_iface_dbgq_read(device, packet)) {
		venus_hfi_clock_adjust(device);
		pkt = (struct hfi_msg_sys_coverage_packet *) packet;
		if (pkt->packet_type == HFI_MSG_SYS_COV) {
			int stm_size = 0;
			dprintk(VIDC_DBG,
				"DbgQ pkt size:%d\n", pkt->msg_size);
>>>>>>> p9x
			stm_size = stm_log_inv_ts(0, 0,
				pkt->rg_msg_data, pkt->msg_size);
			if (stm_size == 0)
				dprintk(VIDC_ERR,
					"In %s, stm_log returned size of 0\n",
					__func__);
		} else {
			struct hfi_msg_sys_debug_packet *pkt =
				(struct hfi_msg_sys_debug_packet *) packet;
			dprintk(VIDC_FW, "%s", pkt->rg_msg_data);
		}
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	if (local_packet)
		kfree(packet);
}

<<<<<<< HEAD
static struct hal_session *__get_session(struct venus_hfi_device *device,
		u32 session_id)
{
	struct hal_session *temp = NULL;

	list_for_each_entry(temp, &device->sess_head, list) {
		if (session_id == hash32_ptr(temp))
			return temp;
	}

	return NULL;
}

static int __response_handler(struct venus_hfi_device *device)
{
	struct msm_vidc_cb_info *packets;
	int packet_count = 0;
	u8 *raw_packet = NULL;
	bool requeue_pm_work = true;

	if (!device || device->state != VENUS_STATE_INIT)
		return 0;

	packets = device->response_pkt;

	raw_packet = kzalloc(VIDC_IFACEQ_VAR_HUGE_PKT_SIZE, GFP_TEMPORARY);
	if (!raw_packet || !packets) {
		dprintk(VIDC_ERR, "%s: Failed to allocate memory\n",  __func__);
		kfree(raw_packet);
		return 0;
	}

	if (device->intr_status & VIDC_WRAPPER_INTR_CLEAR_A2HWD_BMSK) {
		struct hfi_sfr_struct *vsfr = (struct hfi_sfr_struct *)
			device->sfr.align_virtual_addr;
		struct msm_vidc_cb_info info = {
			.response_type = HAL_SYS_WATCHDOG_TIMEOUT,
			.response.cmd = {
				.device_id = device->device_id,
			}
		};

		if (vsfr)
			dprintk(VIDC_ERR, "SFR Message from FW: %s\n",
					vsfr->rg_data);

		__dump_venus_debug_registers(device);
		dprintk(VIDC_ERR, "Received watchdog timeout\n");
		packets[packet_count++] = info;
		goto exit;
	}

	/* Bleed the msg queue dry of packets */
	while (!__iface_msgq_read(device, raw_packet)) {
		void **session_id = NULL;
		struct msm_vidc_cb_info *info = &packets[packet_count++];
		struct vidc_hal_sys_init_done sys_init_done = {0};
		int rc = 0;

		rc = hfi_process_msg_packet(device->device_id,
			(struct vidc_hal_msg_pkt_hdr *)raw_packet, info);
		if (rc) {
			dprintk(VIDC_WARN,
					"Corrupt/unknown packet found, discarding\n");
			--packet_count;
			continue;
		}

		/* Process the packet types that we're interested in */
		switch (info->response_type) {
		case HAL_SYS_ERROR:
			__dump_venus_debug_registers(device);
			__process_sys_error(device);
			break;
		case HAL_SYS_RELEASE_RESOURCE_DONE:
			dprintk(VIDC_DBG, "Received SYS_RELEASE_RESOURCE\n");
			break;
		case HAL_SYS_INIT_DONE:
			dprintk(VIDC_DBG, "Received SYS_INIT_DONE\n");
			/* Video driver intentionally does not unset
			 * IMEM on venus to simplify power collapse.
			 */
			if (__set_imem(device, &device->resources.imem))
				dprintk(VIDC_WARN,
				"Failed to set IMEM. Performance will be impacted\n");
			sys_init_done.capabilities =
				device->sys_init_capabilities;
			hfi_process_sys_init_done_prop_read(
				(struct hfi_msg_sys_init_done_packet *)
					raw_packet, &sys_init_done);
			info->response.cmd.data.sys_init_done = sys_init_done;
			break;
		case HAL_SESSION_LOAD_RESOURCE_DONE:
			/*
			 * Work around for H/W bug, need to re-program these
			 * registers as part of a handshake agreement with the
			 * firmware.  This strictly only needs to be done for
			 * decoder secure sessions, but there's no harm in doing
			 * so for all sessions as it's at worst a NO-OP.
			 */
			__set_threshold_registers(device);
			break;
		default:
			break;
		}

		/* For session-related packets, validate session */
		switch (info->response_type) {
		case HAL_SESSION_LOAD_RESOURCE_DONE:
		case HAL_SESSION_INIT_DONE:
		case HAL_SESSION_END_DONE:
		case HAL_SESSION_ABORT_DONE:
		case HAL_SESSION_START_DONE:
		case HAL_SESSION_STOP_DONE:
		case HAL_SESSION_FLUSH_DONE:
		case HAL_SESSION_SUSPEND_DONE:
		case HAL_SESSION_RESUME_DONE:
		case HAL_SESSION_SET_PROP_DONE:
		case HAL_SESSION_GET_PROP_DONE:
		case HAL_SESSION_PARSE_SEQ_HDR_DONE:
		case HAL_SESSION_RELEASE_BUFFER_DONE:
		case HAL_SESSION_RELEASE_RESOURCE_DONE:
		case HAL_SESSION_PROPERTY_INFO:
			session_id = &info->response.cmd.session_id;
			break;
		case HAL_SESSION_ERROR:
		case HAL_SESSION_GET_SEQ_HDR_DONE:
		case HAL_SESSION_ETB_DONE:
		case HAL_SESSION_FTB_DONE:
			session_id = &info->response.data.session_id;
			break;
		case HAL_SESSION_EVENT_CHANGE:
			session_id = &info->response.event.session_id;
			break;
		case HAL_RESPONSE_UNUSED:
		default:
			session_id = NULL;
			break;
		}

		/*
		 * hfi_process_msg_packet provides a session_id that's a hashed
		 * value of struct hal_session, we need to coerce the hashed
		 * value back to pointer that we can use. Ideally, hfi_process\
		 * _msg_packet should take care of this, but it doesn't have
		 * required information for it
		 */
		if (session_id) {
			struct hal_session *session = NULL;

			if (upper_32_bits((uintptr_t)*session_id) != 0) {
				dprintk(VIDC_WARN,
					"Upper 32 bits of session_id != 0\n");
				WARN_ON(VIDC_DBG_WARN_ENABLE);
			}
			session = __get_session(device,
					(u32)(uintptr_t)*session_id);
			if (!session) {
				dprintk(VIDC_ERR,
						"Received a packet (%#x) for an unrecognized session (%pK), discarding\n",
						info->response_type,
						*session_id);
				--packet_count;
				continue;
			}

			*session_id = session->session_id;
		}

		if (packet_count >= max_packets &&
				__get_q_size(device, VIDC_IFACEQ_MSGQ_IDX)) {
			dprintk(VIDC_WARN,
					"Too many packets in message queue to handle at once, deferring read\n");
			break;
		}

		/* do not read packets after sys error packet */
		if (info->response_type == HAL_SYS_ERROR)
			break;
	}

	if (requeue_pm_work && device->res->sw_power_collapsible) {
		cancel_delayed_work(&venus_hfi_pm_work);
		if (!queue_delayed_work(device->venus_pm_workq,
			&venus_hfi_pm_work,
			msecs_to_jiffies(msm_vidc_pwr_collapse_delay))) {
			dprintk(VIDC_ERR, "PM work already scheduled\n");
		}
	}

exit:
	__flush_debug_queue(device, raw_packet);

	kfree(raw_packet);
	return packet_count;
=======
#define HFI_CTRL_STATUS_CLK_DOWN        0x200
#define HFI_CTRL_INIT_CLK_DOWN          0x2
#define POLL_TRIALS                     100

static void venus_hfi_clock_adjust(struct venus_hfi_device *device)
{
	int rc = 0, i = 0;
	u32 ctrl_status = 0, ctrl_init = 0;
	unsigned long rate = 0;

	if (!device) {
		dprintk(VIDC_ERR, "%s: invalid argsn\n", __func__);
		return;
	}

	if (!msm_vidc_reset_clock_control)
		return;

	if (!device->power_enabled)
		return;

	/* check if venus firmware requested to reduce clock rate */
	ctrl_status = venus_hfi_read_register(device,
					VIDC_CPU_CS_SCIACMDARG0);
	if (!(ctrl_status & HFI_CTRL_STATUS_CLK_DOWN))
		return;

	/* avoid other threads to change the clock rate */
	mutex_lock(&device->clock_lock);

	/* firmware requested to reduce clock rate */
	rate = venus_hfi_get_clock_rate(device, 0, NULL);
	rc = venus_hfi_set_clock(device, rate);
	if (rc) {
		dprintk(VIDC_ERR, "%s: Clocks reduce failed\n", __func__);
		goto unlock;
	}

	/* update firmware that driver reduced clock rate */
	ctrl_init = venus_hfi_read_register(device, VIDC_CTRL_INIT);
	ctrl_init |= HFI_CTRL_INIT_CLK_DOWN;
	venus_hfi_write_register(device, VIDC_CTRL_INIT, ctrl_init);

	/* check if firmware asking to increase clock rate back to normal */
	while (i < POLL_TRIALS) {
		ctrl_status = venus_hfi_read_register(device,
						VIDC_CPU_CS_SCIACMDARG0);
		if (!(ctrl_status & HFI_CTRL_STATUS_CLK_DOWN)) {
			dprintk(VIDC_DBG,
				"%s: increase clock rate to normal\n",
				__func__);
			break;
		}
		i++;
		usleep(i);
	}
	if (i == POLL_TRIALS) {
		dprintk(VIDC_WARN,
			"%s: firmware not requesting to increase clock rate back to normal\n",
			__func__);
	}

	/* update firmware that increasing clock rate back to normal */
	ctrl_init &= ~HFI_CTRL_INIT_CLK_DOWN;
	venus_hfi_write_register(device, VIDC_CTRL_INIT, ctrl_init);

	/* increase clock rate back to normal */
	rc = venus_hfi_set_clock(device, device->clk_freq);
	if (rc) {
		dprintk(VIDC_ERR, "%s: Clocks increase failed\n", __func__);
		goto unlock;
	}

unlock:
	mutex_unlock(&device->clock_lock);

	return;
}

static void venus_hfi_response_handler(struct venus_hfi_device *device)
{
	u8 *packet = NULL;
	u32 rc = 0;
	struct hfi_sfr_struct *vsfr = NULL;

	/*
	 * check for clock adjust request from firmware
	 * for every interrupt
	 */
	venus_hfi_clock_adjust(device);

	packet = kzalloc(VIDC_IFACEQ_VAR_HUGE_PKT_SIZE, GFP_TEMPORARY);
	if (!packet) {
		dprintk(VIDC_ERR, "In %s() Fail to allocate mem\n",  __func__);
		return;
	}

	dprintk(VIDC_INFO, "#####venus_hfi_response_handler#####\n");
	/* Process messages only if device is in valid state*/
	if (device && device->state != VENUS_STATE_DEINIT) {
		if ((device->intr_status &
			VIDC_WRAPPER_INTR_CLEAR_A2HWD_BMSK)) {
			dprintk(VIDC_ERR, "Received: Watchdog timeout %s\n",
				__func__);
			vsfr = (struct hfi_sfr_struct *)
					device->sfr.align_virtual_addr;
			if (vsfr)
				dprintk(VIDC_ERR,
					"SFR Message from FW : %s\n",
						vsfr->rg_data);
			venus_hfi_process_sys_watchdog_timeout(device);
		}

		while (!venus_hfi_iface_msgq_read(device, packet)) {
			/*
			 * check for clock adjust request from firmware
			 * as often as possible
			 */
			venus_hfi_clock_adjust(device);

			/* During SYS_ERROR processing the device state
			*  will be changed to DEINIT. Below check will
			*  make sure no messages messages are read or
			*  processed after processing SYS_ERROR
			*/
			if (device->state == VENUS_STATE_DEINIT) {
				dprintk(VIDC_ERR,
					"core DEINIT'd, stopping q reads\n");
				break;
			}
			rc = hfi_process_msg_packet(device->callback,
				device->device_id,
				(struct vidc_hal_msg_pkt_hdr *) packet,
				&device->sess_head, &device->session_lock);
			if (rc == HFI_MSG_EVENT_NOTIFY) {
				venus_hfi_process_msg_event_notify(
					device, (void *)packet);
			} else if (rc == HFI_MSG_SYS_RELEASE_RESOURCE) {
				dprintk(VIDC_DBG,
					"Received HFI_MSG_SYS_RELEASE_RESOURCE\n");
				complete(&release_resources_done);
			} else if (rc == HFI_MSG_SYS_PC_PREP_DONE) {
				dprintk(VIDC_DBG,
					"Received HFI_MSG_SYS_PC_PREP_DONE\n");
				complete(&pc_prep_done);
			} else if (rc == HFI_MSG_SYS_INIT_DONE) {
				int ret = 0;
				dprintk(VIDC_DBG,
					"Received HFI_MSG_SYS_INIT_DONE\n");
				ret = __alloc_set_ocmem(device, true);
				if (ret)
					dprintk(VIDC_WARN,
						"Failed to allocate OCMEM. Performance will be impacted\n");
			}
		}
		venus_hfi_flush_debug_queue(device, packet);
	} else {
		dprintk(VIDC_DBG, "device (%pK) is in deinit state\n", device);
	}
	kfree(packet);
>>>>>>> p9x
}

static void venus_hfi_core_work_handler(struct work_struct *work)
{
	struct venus_hfi_device *device = list_first_entry(
		&hal_ctxt.dev_head, struct venus_hfi_device, list);
<<<<<<< HEAD
	int num_responses = 0, i = 0;

	mutex_lock(&device->lock);

	dprintk(VIDC_INFO, "Handling interrupt\n");

	if (!__core_in_valid_state(device)) {
		dprintk(VIDC_DBG, "%s - Core not in init state\n", __func__);
		goto err_no_work;
	}

	if (!device->callback) {
		dprintk(VIDC_ERR, "No interrupt callback function: %pK\n",
				device);
		goto err_no_work;
	}

	if (__resume(device)) {
		dprintk(VIDC_ERR, "%s: Power enable failed\n", __func__);
		goto err_no_work;
	}

	__core_clear_interrupt(device);
	num_responses = __response_handler(device);

err_no_work:
	/* We need re-enable the irq which was disabled in ISR handler */
	if (!(device->intr_status & VIDC_WRAPPER_INTR_STATUS_A2HWD_BMSK))
		enable_irq(device->hal_data->irq);

	mutex_unlock(&device->lock);

	/*
	 * Issue the callbacks outside of the locked contex to preserve
	 * re-entrancy.
	 */

	for (i = 0; !IS_ERR_OR_NULL(device->response_pkt) &&
		i < num_responses; ++i) {
		struct msm_vidc_cb_info *r = &device->response_pkt[i];

		if (!__core_in_valid_state(device)) {
			dprintk(VIDC_ERR,
				"Ignore responses from %d to %d as device is in invalid state",
				(i + 1), num_responses);
			break;
		}
		device->callback(r->response_type, &r->response);
	}

	/*
	 * XXX: Don't add any code beyond here.  Reacquiring locks after release
	 * it above doesn't guarantee the atomicity that we're aiming for.
	 */
}

=======

	dprintk(VIDC_INFO, "GOT INTERRUPT\n");
	if (!device->callback) {
		dprintk(VIDC_ERR, "No interrupt callback function: %pK\n",
				device);
		return;
	}
	if (venus_hfi_power_enable(device)) {
		dprintk(VIDC_ERR, "%s: Power enable failed\n", __func__);
		return;
	}
	if (device->res->sw_power_collapsible &&
		device->state != VENUS_STATE_DEINIT) {
		dprintk(VIDC_DBG, "Cancel and queue delayed work from %s\n",
			__func__);
		cancel_delayed_work(&venus_hfi_pm_work);
		if (!queue_delayed_work(device->venus_pm_workq,
			&venus_hfi_pm_work,
			msecs_to_jiffies(msm_vidc_pwr_collapse_delay))) {
			dprintk(VIDC_DBG, "PM work already scheduled\n");
		}
	}
	venus_hfi_core_clear_interrupt(device);
	venus_hfi_response_handler(device);
	if (!(device->intr_status & VIDC_WRAPPER_INTR_STATUS_A2HWD_BMSK))
		enable_irq(device->hal_data->irq);
}
>>>>>>> p9x
static DECLARE_WORK(venus_hfi_work, venus_hfi_core_work_handler);

static irqreturn_t venus_hfi_isr(int irq, void *dev)
{
	struct venus_hfi_device *device = dev;
<<<<<<< HEAD
	dprintk(VIDC_INFO, "Received an interrupt %d\n", irq);
=======
	dprintk(VIDC_INFO, "vidc_hal_isr %d\n", irq);
>>>>>>> p9x
	disable_irq_nosync(irq);
	queue_work(device->vidc_workq, &venus_hfi_work);
	return IRQ_HANDLED;
}

<<<<<<< HEAD
static int __init_regs_and_interrupts(struct venus_hfi_device *device,
=======
static int venus_hfi_init_regs_and_interrupts(
		struct venus_hfi_device *device,
>>>>>>> p9x
		struct msm_vidc_platform_resources *res)
{
	struct hal_data *hal = NULL;
	int rc = 0;

<<<<<<< HEAD
	rc = __check_core_registered(hal_ctxt, res->firmware_base,
			(u8 *)(uintptr_t)res->register_base,
=======
	rc = venus_hfi_check_core_registered(hal_ctxt,
			res->firmware_base,
			(u8 *)(unsigned long)res->register_base,
>>>>>>> p9x
			res->register_size, res->irq);
	if (!rc) {
		dprintk(VIDC_ERR, "Core present/Already added\n");
		rc = -EEXIST;
		goto err_core_init;
	}

	dprintk(VIDC_DBG, "HAL_DATA will be assigned now\n");
	hal = (struct hal_data *)
		kzalloc(sizeof(struct hal_data), GFP_KERNEL);
	if (!hal) {
		dprintk(VIDC_ERR, "Failed to alloc\n");
		rc = -ENOMEM;
		goto err_core_init;
	}
<<<<<<< HEAD

	hal->irq = res->irq;
	hal->firmware_base = res->firmware_base;
	hal->register_base = devm_ioremap_nocache(&res->pdev->dev,
			res->register_base, res->register_size);
	hal->register_size = res->register_size;
	if (!hal->register_base) {
		dprintk(VIDC_ERR,
			"could not map reg addr %pa of size %d\n",
=======
	hal->irq = res->irq;
	hal->firmware_base = res->firmware_base;
	hal->register_base = devm_ioremap_nocache(&res->pdev->dev,
			res->register_base, (unsigned long)res->register_size);
	hal->register_size = res->register_size;
	if (!hal->register_base) {
		dprintk(VIDC_ERR,
			"could not map reg addr 0x%pa of size %d\n",
>>>>>>> p9x
			&res->register_base, res->register_size);
		goto error_irq_fail;
	}

	device->hal_data = hal;
	rc = request_irq(res->irq, venus_hfi_isr, IRQF_TRIGGER_HIGH,
			"msm_vidc", device);
	if (unlikely(rc)) {
		dprintk(VIDC_ERR, "() :request_irq failed\n");
		goto error_irq_fail;
	}
<<<<<<< HEAD

	disable_irq_nosync(res->irq);
	dprintk(VIDC_INFO,
		"firmware_base = %pa, register_base = %pa, register_size = %d\n",
=======
	disable_irq_nosync(res->irq);
	dprintk(VIDC_INFO,
		"firmware_base = 0x%pa, register_base = 0x%pa, register_size = %d\n",
>>>>>>> p9x
		&res->firmware_base, &res->register_base,
		res->register_size);
	return rc;

error_irq_fail:
	kfree(hal);
err_core_init:
	return rc;

}

<<<<<<< HEAD
static inline void __deinit_clocks(struct venus_hfi_device *device)
{
	struct clock_info *cl;

	device->clk_freq = 0;
	venus_hfi_for_each_clock_reverse(device, cl) {
		if (cl->clk) {
			clk_put(cl->clk);
			cl->clk = NULL;
		}
	}
}

static inline int __init_clocks(struct venus_hfi_device *device)
=======
static inline int venus_hfi_init_clocks(struct msm_vidc_platform_resources *res,
		struct venus_hfi_device *device)
>>>>>>> p9x
{
	int rc = 0;
	struct clock_info *cl = NULL;

<<<<<<< HEAD
	if (!device) {
=======
	if (!res || !device) {
>>>>>>> p9x
		dprintk(VIDC_ERR, "Invalid params: %pK\n", device);
		return -EINVAL;
	}

	venus_hfi_for_each_clock(device, cl) {
		int i = 0;

<<<<<<< HEAD
		dprintk(VIDC_DBG, "%s: scalable? %d, count %d\n",
				cl->name, cl->has_scaling, cl->count);
		for (i = 0; i < cl->count; ++i) {
			dprintk(VIDC_DBG,
				"\tload = %d, freq = %d codecs supported %#x\n",
=======
		dprintk(VIDC_DBG, "%s: scalable? %d, gate-able? %d\n", cl->name,
			!!cl->count, cl->has_gating);
		for (i = 0; i < cl->count; ++i) {
			dprintk(VIDC_DBG,
				"\tload = %d, freq = %d codecs supported 0x%x\n",
>>>>>>> p9x
				cl->load_freq_tbl[i].load,
				cl->load_freq_tbl[i].freq,
				cl->load_freq_tbl[i].supported_codecs);
		}
	}

	venus_hfi_for_each_clock(device, cl) {
<<<<<<< HEAD
		if (!cl->clk) {
			cl->clk = clk_get(&device->res->pdev->dev, cl->name);
=======
		if (!strcmp(cl->name, "mem_clk") && !res->ocmem_size) {
			dprintk(VIDC_ERR,
				"Found %s on a target that doesn't support ocmem\n",
				cl->name);
			rc = -ENOENT;
			goto err_found_bad_ocmem;
		}

		if (!cl->clk) {
			cl->clk = devm_clk_get(&res->pdev->dev, cl->name);
>>>>>>> p9x
			if (IS_ERR_OR_NULL(cl->clk)) {
				dprintk(VIDC_ERR,
					"Failed to get clock: %s\n", cl->name);
				rc = PTR_ERR(cl->clk) ?: -EINVAL;
				cl->clk = NULL;
				goto err_clk_get;
			}
		}
	}
<<<<<<< HEAD
=======

>>>>>>> p9x
	device->clk_freq = 0;
	return 0;

err_clk_get:
<<<<<<< HEAD
	__deinit_clocks(device);
	return rc;
}


static inline void __disable_unprepare_clks(struct venus_hfi_device *device)
=======
err_found_bad_ocmem:
	venus_hfi_for_each_clock(device, cl) {
		if (cl->clk)
			clk_put(cl->clk);
	}

	return rc;
}

static inline void venus_hfi_deinit_clocks(struct venus_hfi_device *device)
{
	struct clock_info *cl;
	if (!device) {
		dprintk(VIDC_ERR, "Invalid args\n");
		return;
	}

	device->clk_freq = 0;
	venus_hfi_for_each_clock(device, cl)
		clk_put(cl->clk);
}

static inline void venus_hfi_disable_unprepare_clks(
	struct venus_hfi_device *device)
>>>>>>> p9x
{
	struct clock_info *cl;

	if (!device) {
		dprintk(VIDC_ERR, "Invalid params: %pK\n", device);
		return;
	}

<<<<<<< HEAD
	venus_hfi_for_each_clock(device, cl) {
		 usleep_range(100, 500);
=======
	mutex_lock(&device->clock_lock);
	if (device->clk_state == DISABLED_UNPREPARED) {
		dprintk(VIDC_DBG, "Clocks already unprepared and disabled\n");
		mutex_unlock(&device->clock_lock);
		return;
	}

	/*
	* Make the clock state variable as unprepared before actually
	* unpreparing clocks. This will make sure that when we check
	* the state, we have the right clock state. We are not taking
	* any action based unprepare failures. So it is safe to do
	* before the call. This is also in sync with prepare_enable
	* state update.
	*/

	device->clk_state = DISABLED_UNPREPARED;

	venus_hfi_for_each_clock(device, cl) {
		usleep(100);
>>>>>>> p9x
		dprintk(VIDC_DBG, "Clock: %s disable and unprepare\n",
				cl->name);
		clk_disable_unprepare(cl->clk);
	}
<<<<<<< HEAD
}

static inline int __prepare_enable_clks(struct venus_hfi_device *device)
=======
	mutex_unlock(&device->clock_lock);
}

static inline int venus_hfi_prepare_enable_clks(struct venus_hfi_device *device)
>>>>>>> p9x
{
	struct clock_info *cl = NULL, *cl_fail = NULL;
	int rc = 0;
	if (!device) {
		dprintk(VIDC_ERR, "Invalid params: %pK\n", device);
		return -EINVAL;
	}

<<<<<<< HEAD
	venus_hfi_for_each_clock(device, cl) {
		/*
		 * For the clocks we control, set the rate prior to preparing
		 * them.  Since we don't really have a load at this point, scale
		 * it to the lowest frequency possible
		 */
		if (cl->has_scaling)
			clk_set_rate(cl->clk, clk_round_rate(cl->clk, 0));

=======
	mutex_lock(&device->clock_lock);
	if (device->clk_state == ENABLED_PREPARED) {
		dprintk(VIDC_DBG, "Clocks already prepared and enabled\n");
		mutex_unlock(&device->clock_lock);
		return 0;
	}

	venus_hfi_for_each_clock(device, cl) {
>>>>>>> p9x
		rc = clk_prepare_enable(cl->clk);
		if (rc) {
			dprintk(VIDC_ERR, "Failed to enable clocks\n");
			cl_fail = cl;
<<<<<<< HEAD
=======
			mutex_unlock(&device->clock_lock);
>>>>>>> p9x
			goto fail_clk_enable;
		}

		dprintk(VIDC_DBG, "Clock: %s prepared and enabled\n", cl->name);
	}
<<<<<<< HEAD

	__write_register(device, VIDC_WRAPPER_CLOCK_CONFIG, 0);
	__write_register(device, VIDC_WRAPPER_CPU_CLOCK_CONFIG, 0);
	return rc;

fail_clk_enable:
	venus_hfi_for_each_clock(device, cl) {
		if (cl_fail == cl)
			break;
		usleep_range(100, 500);
=======
	device->clk_state = ENABLED_PREPARED;
	mutex_unlock(&device->clock_lock);

	venus_hfi_write_register(device, VIDC_WRAPPER_CLOCK_CONFIG, 0);
	venus_hfi_write_register(device, VIDC_WRAPPER_CPU_CLOCK_CONFIG, 0);
	return rc;

fail_clk_enable:
	mutex_lock(&device->clock_lock);
	venus_hfi_for_each_clock(device, cl) {
		if (cl_fail == cl)
			break;
		usleep(100);
>>>>>>> p9x
		dprintk(VIDC_ERR, "Clock: %s disable and unprepare\n",
			cl->name);
		clk_disable_unprepare(cl->clk);
	}
<<<<<<< HEAD

	return rc;
}

static void __deinit_bus(struct venus_hfi_device *device)
=======
	device->clk_state = DISABLED_UNPREPARED;
	mutex_unlock(&device->clock_lock);
	return rc;
}

static int venus_hfi_register_iommu_domains(struct venus_hfi_device *device,
					struct msm_vidc_platform_resources *res)
{
	struct iommu_domain *domain;
	int rc = 0, i = 0;
	struct iommu_set *iommu_group_set;
	struct iommu_info *iommu_map;

	if (!device || !res)
		return -EINVAL;

	iommu_group_set = &device->res->iommu_group_set;

	for (i = 0; i < iommu_group_set->count; i++) {
		iommu_map = &iommu_group_set->iommu_maps[i];
		iommu_map->group = iommu_group_find(iommu_map->name);
		if (!iommu_map->group) {
			dprintk(VIDC_DBG, "Failed to find group :%s\n",
				iommu_map->name);
			rc = -EPROBE_DEFER;
			goto fail_group;
		}
		domain = iommu_group_get_iommudata(iommu_map->group);
		if (!domain) {
			dprintk(VIDC_ERR,
				"Failed to get domain data for group %pK\n",
				iommu_map->group);
			rc = -EINVAL;
			goto fail_group;
		}
		iommu_map->domain = msm_find_domain_no(domain);
		if (iommu_map->domain < 0) {
			dprintk(VIDC_ERR,
				"Failed to get domain index for domain %pK\n",
				domain);
			rc = -EINVAL;
			goto fail_group;
		}
	}
	return rc;

fail_group:
	for (--i; i >= 0; i--) {
		iommu_map = &iommu_group_set->iommu_maps[i];
		if (iommu_map->group)
			iommu_group_put(iommu_map->group);
		iommu_map->group = NULL;
		iommu_map->domain = -1;
	}
	return rc;
}

static void venus_hfi_deregister_iommu_domains(struct venus_hfi_device *device)
{
	struct iommu_set *iommu_group_set;
	struct iommu_info *iommu_map;
	int i = 0;

	if (!device)
		return;

	iommu_group_set = &device->res->iommu_group_set;
	for (i = 0; i < iommu_group_set->count; i++) {
		iommu_map = &iommu_group_set->iommu_maps[i];
		if (iommu_map->group)
			iommu_group_put(iommu_map->group);
		iommu_map->group = NULL;
		iommu_map->domain = -1;
	}
}

static void venus_hfi_deinit_bus(struct venus_hfi_device *device)
>>>>>>> p9x
{
	struct bus_info *bus = NULL;
	if (!device)
		return;

<<<<<<< HEAD
	kfree(device->bus_vote.data);
	device->bus_vote = DEFAULT_BUS_VOTE;

	venus_hfi_for_each_bus_reverse(device, bus) {
		devfreq_remove_device(bus->devfreq);
		bus->devfreq = NULL;
		dev_set_drvdata(bus->dev, NULL);

		msm_bus_scale_unregister(bus->client);
		bus->client = NULL;
	}
}

static int __init_bus(struct venus_hfi_device *device)
{
	struct bus_info *bus = NULL;
	int rc = 0;

	if (!device)
		return -EINVAL;

	venus_hfi_for_each_bus(device, bus) {
		struct devfreq_dev_profile profile = {
			.initial_freq = 0,
			.polling_ms = INT_MAX,
			.freq_table = NULL,
			.max_state = 0,
			.target = __devfreq_target,
			.get_dev_status = __devfreq_get_status,
			.exit = NULL,
		};

		/*
		 * This is stupid, but there's no other easy way to ahold
		 * of struct bus_info in venus_hfi_devfreq_*()
		 */
		WARN(dev_get_drvdata(bus->dev), "%s's drvdata already set\n",
				dev_name(bus->dev));
		dev_set_drvdata(bus->dev, device);

		bus->client = msm_bus_scale_register(bus->master, bus->slave,
				bus->name, false);
		if (IS_ERR_OR_NULL(bus->client)) {
			rc = PTR_ERR(bus->client) ?: -EBADHANDLE;
			dprintk(VIDC_ERR, "Failed to register bus %s: %d\n",
					bus->name, rc);
			bus->client = NULL;
			goto err_add_dev;
		}

		bus->devfreq_prof = profile;
		bus->devfreq = devfreq_add_device(bus->dev,
				&bus->devfreq_prof, bus->governor, NULL);
		if (IS_ERR_OR_NULL(bus->devfreq)) {
			rc = PTR_ERR(bus->devfreq) ?: -EBADHANDLE;
			dprintk(VIDC_ERR,
					"Failed to add devfreq device for bus %s and governor %s: %d\n",
					bus->name, bus->governor, rc);
			bus->devfreq = NULL;
			goto err_add_dev;
		}

		/*
		 * Devfreq starts monitoring immediately, since we are just
		 * initializing stuff at this point, force it to suspend
		 */
		devfreq_suspend_device(bus->devfreq);
	}

	device->bus_vote = DEFAULT_BUS_VOTE;
	return 0;

err_add_dev:
	__deinit_bus(device);
	return rc;
}

static void __deinit_regulators(struct venus_hfi_device *device)
{
	struct regulator_info *rinfo = NULL;

	venus_hfi_for_each_regulator_reverse(device, rinfo) {
		if (rinfo->regulator) {
			regulator_put(rinfo->regulator);
			rinfo->regulator = NULL;
		}
	}
}

static int __init_regulators(struct venus_hfi_device *device)
{
	int rc = 0;
	struct regulator_info *rinfo = NULL;

	venus_hfi_for_each_regulator(device, rinfo) {
		rinfo->regulator = regulator_get(&device->res->pdev->dev,
				rinfo->name);
		if (IS_ERR_OR_NULL(rinfo->regulator)) {
			rc = PTR_ERR(rinfo->regulator) ?: -EBADHANDLE;
			dprintk(VIDC_ERR, "Failed to get regulator: %s\n",
					rinfo->name);
			rinfo->regulator = NULL;
			goto err_reg_get;
=======
	venus_hfi_for_each_bus(device, bus) {
		if (bus->priv) {
			msm_bus_scale_unregister_client(
				bus->priv);
			bus->priv = 0;
			dprintk(VIDC_DBG, "Unregistered bus client %s\n",
				bus->pdata->name);
		}
	}

	kfree(device->bus_load.vote_data);
	device->bus_load.vote_data = NULL;
	device->bus_load.vote_data_count = 0;
}

static int venus_hfi_init_bus(struct venus_hfi_device *device)
{
	struct bus_info *bus = NULL;
	int rc = 0;
	if (!device)
		return -EINVAL;


	venus_hfi_for_each_bus(device, bus) {
		const char *name = bus->pdata->name;

		if (!device->res->ocmem_size &&
			strnstr(name, "ocmem", strlen(name))) {
			dprintk(VIDC_ERR,
				"%s found when target doesn't support ocmem\n",
				name);
			rc = -EINVAL;
			goto err_init_bus;
		} else if (bus->passive && bus->pdata->num_usecases != 2) {
			/*
			 * Passive buses can only be "turned on" and "turned
			 * off".  We never scale them based on hardware load,
			 * and are usually used for the purposes of holding
			 * certain clocks high (in case we can't control these
			 * clocks directly).
			 */
			rc = -EINVAL;
			dprintk(VIDC_ERR,
					"Passive buses expected to have only 2 vectors\n");
		}

		bus->priv = msm_bus_scale_register_client(bus->pdata);
		if (!bus->priv) {
			dprintk(VIDC_ERR,
				"Failed to register bus client %s\n", name);
			rc = -EBADHANDLE;
			goto err_init_bus;
		}

		dprintk(VIDC_DBG, "Registered bus client %s\n", name);
	}

        device->bus_load.vote_data = (struct vidc_bus_vote_data *)
                                        kzalloc(sizeof(struct vidc_bus_vote_data)*MAX_SUPPORTED_INSTANCES_COUNT, GFP_KERNEL);

        if (device->bus_load.vote_data == NULL) {
                dprintk(VIDC_ERR,"Failed to allocate memory for vote_data\n");
                rc = -ENOMEM;
                goto err_init_bus;
        }
        device->bus_load.vote_data_count = 0;
	return rc;
err_init_bus:
	venus_hfi_deinit_bus(device);
	return rc;
}

static int venus_hfi_init_regulators(struct venus_hfi_device *device,
		struct msm_vidc_platform_resources *res)
{
	struct regulator_info *rinfo = NULL;

	venus_hfi_for_each_regulator(device, rinfo) {
		rinfo->regulator = devm_regulator_get(&res->pdev->dev,
				rinfo->name);
		if (IS_ERR(rinfo->regulator)) {
			dprintk(VIDC_ERR, "Failed to get regulator: %s\n",
					rinfo->name);
			rinfo->regulator = NULL;
			return -ENODEV;
>>>>>>> p9x
		}
	}

	return 0;
<<<<<<< HEAD

err_reg_get:
	__deinit_regulators(device);
	return rc;
}

static int __init_resources(struct venus_hfi_device *device,
=======
}

static void venus_hfi_deinit_regulators(struct venus_hfi_device *device)
{
	struct regulator_info *rinfo = NULL;

	/* No need to regulator_put. Regulators automatically freed
	 * thanks to devm_regulator_get */
	venus_hfi_for_each_regulator(device, rinfo)
		rinfo->regulator = NULL;
}

static int venus_hfi_init_resources(struct venus_hfi_device *device,
>>>>>>> p9x
				struct msm_vidc_platform_resources *res)
{
	int rc = 0;

<<<<<<< HEAD
	rc = __init_regulators(device);
=======
	device->res = res;
	if (!res) {
		dprintk(VIDC_ERR, "Invalid params: %pK\n", res);
		return -ENODEV;
	}

	rc = venus_hfi_init_regulators(device, res);
>>>>>>> p9x
	if (rc) {
		dprintk(VIDC_ERR, "Failed to get all regulators\n");
		return -ENODEV;
	}

<<<<<<< HEAD
	rc = __init_clocks(device);
=======
	rc = venus_hfi_init_clocks(res, device);
>>>>>>> p9x
	if (rc) {
		dprintk(VIDC_ERR, "Failed to init clocks\n");
		rc = -ENODEV;
		goto err_init_clocks;
	}

<<<<<<< HEAD
	rc = __init_bus(device);
=======
	rc = venus_hfi_init_bus(device);
>>>>>>> p9x
	if (rc) {
		dprintk(VIDC_ERR, "Failed to init bus: %d\n", rc);
		goto err_init_bus;
	}

<<<<<<< HEAD
	device->sys_init_capabilities =
		kzalloc(sizeof(struct msm_vidc_capability)
		* VIDC_MAX_SESSIONS, GFP_TEMPORARY);

	return rc;

err_init_bus:
	__deinit_clocks(device);
err_init_clocks:
	__deinit_regulators(device);
	return rc;
}

static void __deinit_resources(struct venus_hfi_device *device)
{
	__deinit_bus(device);
	__deinit_clocks(device);
	__deinit_regulators(device);
	kfree(device->sys_init_capabilities);
	device->sys_init_capabilities = NULL;
}

static int __protect_cp_mem(struct venus_hfi_device *device)
=======
	rc = venus_hfi_register_iommu_domains(device, res);
	if (rc) {
		if (rc != -EPROBE_DEFER) {
			dprintk(VIDC_ERR,
				"Failed to register iommu domains: %d\n", rc);
		}
		goto err_register_iommu_domain;
	}

	return rc;

err_register_iommu_domain:
	venus_hfi_deinit_bus(device);
err_init_bus:
	venus_hfi_deinit_clocks(device);
err_init_clocks:
	venus_hfi_deinit_regulators(device);
	return rc;
}

static void venus_hfi_deinit_resources(struct venus_hfi_device *device)
{
	venus_hfi_deregister_iommu_domains(device);
	venus_hfi_deinit_bus(device);
	venus_hfi_deinit_clocks(device);
	venus_hfi_deinit_regulators(device);
}

static int venus_hfi_iommu_get_domain_partition(void *dev, u32 flags,
			u32 buffer_type, int *domain, int *partition)
{
	struct venus_hfi_device *device = dev;

	if (!device) {
		dprintk(VIDC_ERR, "%s: Invalid param device: %pK\n",
		 __func__, device);
		return -EINVAL;
	}

	msm_smem_get_domain_partition(device->hal_client, flags, buffer_type,
			domain, partition);
	return 0;
}

static int protect_cp_mem(struct venus_hfi_device *device)
>>>>>>> p9x
{
	struct tzbsp_memprot memprot;
	unsigned int resp = 0;
	int rc = 0;
<<<<<<< HEAD
	struct context_bank_info *cb;
=======
	struct iommu_set *iommu_group_set;
	struct iommu_info *iommu_map;
	int i;
>>>>>>> p9x
	struct scm_desc desc = {0};

	if (!device)
		return -EINVAL;

<<<<<<< HEAD
=======
	iommu_group_set = &device->res->iommu_group_set;
	if (!iommu_group_set) {
		dprintk(VIDC_ERR, "invalid params: %pK\n", iommu_group_set);
		return -EINVAL;
	}

>>>>>>> p9x
	memprot.cp_start = 0x0;
	memprot.cp_size = 0x0;
	memprot.cp_nonpixel_start = 0x0;
	memprot.cp_nonpixel_size = 0x0;

<<<<<<< HEAD
	list_for_each_entry(cb, &device->res->context_banks, list) {
		if (!strcmp(cb->name, "venus_ns")) {
			desc.args[1] = memprot.cp_size =
				cb->addr_range.start;
			dprintk(VIDC_DBG, "%s memprot.cp_size: %#x\n",
				__func__, memprot.cp_size);
		}

		if (!strcmp(cb->name, "venus_sec_non_pixel")) {
			desc.args[2] = memprot.cp_nonpixel_start =
				cb->addr_range.start;
			desc.args[3] = memprot.cp_nonpixel_size =
				cb->addr_range.size;
			dprintk(VIDC_DBG,
				"%s memprot.cp_nonpixel_start: %#x size: %#x\n",
				__func__, memprot.cp_nonpixel_start,
				memprot.cp_nonpixel_size);
=======
	for (i = 0; i < iommu_group_set->count; i++) {
		iommu_map = &iommu_group_set->iommu_maps[i];
		if (strcmp(iommu_map->name, "venus_ns") == 0)
			desc.args[1] = memprot.cp_size =
				iommu_map->addr_range[0].start;

		if (strcmp(iommu_map->name, "venus_sec_non_pixel") == 0) {
			desc.args[2] = memprot.cp_nonpixel_start =
				iommu_map->addr_range[0].start;
			desc.args[3] = memprot.cp_nonpixel_size =
				iommu_map->addr_range[0].size;
		} else if (strcmp(iommu_map->name, "venus_cp") == 0) {
			desc.args[2] = memprot.cp_nonpixel_start =
				iommu_map->addr_range[1].start;
>>>>>>> p9x
		}
	}

	if (!is_scm_armv8()) {
		rc = scm_call(SCM_SVC_MP, TZBSP_MEM_PROTECT_VIDEO_VAR, &memprot,
			sizeof(memprot), &resp, sizeof(resp));
	} else {
		desc.arginfo = SCM_ARGS(4);
		rc = scm_call2(SCM_SIP_FNID(SCM_SVC_MP,
			       TZBSP_MEM_PROTECT_VIDEO_VAR), &desc);
		resp = desc.ret[0];
	}
<<<<<<< HEAD

	if (rc) {
		dprintk(VIDC_ERR, "Failed to protect memory(%d) response: %d\n",
				rc, resp);
	}

=======
	if (rc)
		dprintk(VIDC_ERR,
		"Failed to protect memory , rc is :%d, response : %d\n",
		rc, resp);
>>>>>>> p9x
	trace_venus_hfi_var_done(
		memprot.cp_start, memprot.cp_size,
		memprot.cp_nonpixel_start, memprot.cp_nonpixel_size);
	return rc;
}

<<<<<<< HEAD
static int __disable_regulator(struct regulator_info *rinfo)
=======
static int venus_hfi_disable_regulator(struct regulator_info *rinfo)
>>>>>>> p9x
{
	int rc = 0;

	dprintk(VIDC_DBG, "Disabling regulator %s\n", rinfo->name);

	/*
	* This call is needed. Driver needs to acquire the control back
	* from HW in order to disable the regualtor. Else the behavior
	* is unknown.
	*/

<<<<<<< HEAD
	rc = __acquire_regulator(rinfo);
=======
	rc = venus_hfi_acquire_regulator(rinfo);

>>>>>>> p9x
	if (rc) {
		/* This is somewhat fatal, but nothing we can do
		 * about it. We can't disable the regulator w/o
		 * getting it back under s/w control */
		dprintk(VIDC_WARN,
			"Failed to acquire control on %s\n",
			rinfo->name);

		goto disable_regulator_failed;
	}
<<<<<<< HEAD

=======
>>>>>>> p9x
	rc = regulator_disable(rinfo->regulator);
	if (rc) {
		dprintk(VIDC_WARN,
			"Failed to disable %s: %d\n",
			rinfo->name, rc);
		goto disable_regulator_failed;
	}

<<<<<<< HEAD
=======
	if (msm_vidc_regulator_scaling &&
		strnstr(rinfo->name, "vdd-cx", strlen(rinfo->name))) {

		rc = regulator_set_voltage(rinfo->regulator, 0, INT_MAX);
		if (rc)
			dprintk(VIDC_ERR,
				"%s: Failed to set zero voltage_idx on %s: %d\n",
				__func__, rinfo->name, rc);
		else
			dprintk(VIDC_DBG,
				"%s: set zero voltage_idx on %s\n",
				__func__, rinfo->name);
	}

>>>>>>> p9x
	return 0;
disable_regulator_failed:

	/* Bring attention to this issue */
<<<<<<< HEAD
	WARN_ON(VIDC_DBG_WARN_ENABLE);
	return rc;
}

static int __enable_hw_power_collapse(struct venus_hfi_device *device)
{
	int rc = 0;

	if (!msm_vidc_fw_low_power_mode) {
=======
	WARN_ON(msm_vidc_debug & VIDC_INFO);
	return rc;
}

static int venus_hfi_enable_hw_power_collapse(struct venus_hfi_device *device)
{
	int rc = 0;

	if (!msm_fw_low_power_mode) {
>>>>>>> p9x
		dprintk(VIDC_DBG, "Not enabling hardware power collapse\n");
		return 0;
	}

<<<<<<< HEAD
	rc = __hand_off_regulators(device);
=======
	rc = venus_hfi_hand_off_regulators(device);
>>>>>>> p9x
	if (rc)
		dprintk(VIDC_WARN,
			"%s : Failed to enable HW power collapse %d\n",
				__func__, rc);
	return rc;
}

<<<<<<< HEAD
static int __enable_regulators(struct venus_hfi_device *device)
=======
static int venus_hfi_enable_regulators(struct venus_hfi_device *device)
>>>>>>> p9x
{
	int rc = 0, c = 0;
	struct regulator_info *rinfo;

	dprintk(VIDC_DBG, "Enabling regulators\n");

	venus_hfi_for_each_regulator(device, rinfo) {
		rc = regulator_enable(rinfo->regulator);
		if (rc) {
			dprintk(VIDC_ERR,
					"Failed to enable %s: %d\n",
					rinfo->name, rc);
			goto err_reg_enable_failed;
		}
<<<<<<< HEAD

=======
>>>>>>> p9x
		dprintk(VIDC_DBG, "Enabled regulator %s\n",
				rinfo->name);
		c++;
	}

	return 0;

err_reg_enable_failed:
<<<<<<< HEAD
	venus_hfi_for_each_regulator_reverse_continue(device, rinfo, c)
		__disable_regulator(rinfo);
=======
	venus_hfi_for_each_regulator(device, rinfo) {
		if (!c)
			break;

		venus_hfi_disable_regulator(rinfo);
		--c;
	}
>>>>>>> p9x

	return rc;
}

<<<<<<< HEAD
static int __disable_regulators(struct venus_hfi_device *device)
{
	struct regulator_info *rinfo;
	int rc = 0;

	dprintk(VIDC_DBG, "Disabling regulators\n");

	venus_hfi_for_each_regulator_reverse(device, rinfo)
		__disable_regulator(rinfo);

	return rc;
}

static int __venus_power_on(struct venus_hfi_device *device)
{
	int rc = 0;

	if (device->power_enabled)
		return 0;

	device->power_enabled = true;
	/* Vote for all hardware resources */
	rc = __vote_buses(device, device->bus_vote.data,
			device->bus_vote.data_count);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to vote buses, err: %d\n", rc);
		goto fail_vote_buses;
	}

	rc = __alloc_imem(device, device->res->imem_size);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to allocate IMEM\n");
		goto fail_alloc_imem;
	}

	rc = __enable_regulators(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to enable GDSC, err = %d\n", rc);
		goto fail_enable_gdsc;
	}

	rc = __prepare_enable_clks(device);
=======
static int venus_hfi_disable_regulators(struct venus_hfi_device *device)
{
	struct regulator_info *rinfo;

	dprintk(VIDC_DBG, "Disabling regulators\n");

	venus_hfi_for_each_regulator(device, rinfo)
		venus_hfi_disable_regulator(rinfo);

	return 0;
}

static int venus_hfi_load_fw(void *dev)
{
	int rc = 0;
	struct venus_hfi_device *device = dev;

	if (!device) {
		dprintk(VIDC_ERR, "%s Invalid paramter: %pK\n",
			__func__, device);
		return -EINVAL;
	}

	trace_msm_v4l2_vidc_fw_load_start("msm_v4l2_vidc venus_fw load start");

	rc = venus_hfi_vote_buses(device, device->bus_load.vote_data,
			device->bus_load.vote_data_count);
	if (rc) {
		dprintk(VIDC_ERR,
				"Failed to vote buses when loading firmware: %d\n",
				rc);
		goto fail_vote_buses;
	}

	rc = venus_hfi_enable_regulators(device);
	if (rc) {
		dprintk(VIDC_ERR, "%s : Failed to enable GDSC, Err = %d\n",
			__func__, rc);
		goto fail_enable_gdsc;
	}

	/* iommu_attach makes call to TZ for restore_sec_cfg. With this call
	 * TZ accesses the VMIDMT block which needs all the Venus clocks.
	 */
	rc = venus_hfi_prepare_enable_clks(device);
>>>>>>> p9x
	if (rc) {
		dprintk(VIDC_ERR, "Failed to enable clocks: %d\n", rc);
		goto fail_enable_clks;
	}

<<<<<<< HEAD
	rc = __scale_clocks(device, 0, NULL, 0);
	if (rc) {
		dprintk(VIDC_WARN,
				"Failed to scale clocks, performance might be affected\n");
		rc = 0;
	}
	__write_register(device, VIDC_WRAPPER_INTR_MASK,
			VIDC_WRAPPER_INTR_MASK_A2HVCODEC_BMSK);
	device->intr_status = 0;
	enable_irq(device->hal_data->irq);

	/*
	 * Hand off control of regulators to h/w _after_ enabling clocks.
	 * Note that the GDSC will turn off when switching from normal
	 * (s/w triggered) to fast (HW triggered) unless the h/w vote is
	 * present. Since Venus isn't up yet, the GDSC will be off briefly.
	 */
	if (__enable_hw_power_collapse(device))
		dprintk(VIDC_ERR, "Failed to enabled inter-frame PC\n");

	return rc;

fail_enable_clks:
	__disable_regulators(device);
fail_enable_gdsc:
	__free_imem(device);
fail_alloc_imem:
	__unvote_buses(device);
fail_vote_buses:
	device->power_enabled = false;
	return rc;
}

static void __venus_power_off(struct venus_hfi_device *device, bool halt_axi)
{
	if (!device->power_enabled)
		return;

	if (!(device->intr_status & VIDC_WRAPPER_INTR_STATUS_A2HWD_BMSK))
		disable_irq_nosync(device->hal_data->irq);
	device->intr_status = 0;

	/* Halt the AXI to make sure there are no pending transactions.
	 * Clocks should be unprepared after making sure axi is halted.
	 */
	if (halt_axi && __halt_axi(device))
		dprintk(VIDC_WARN, "Failed to halt AXI\n");

	__disable_unprepare_clks(device);
	if (__disable_regulators(device))
		dprintk(VIDC_WARN, "Failed to disable regulators\n");

	__free_imem(device);

	if (__unvote_buses(device))
		dprintk(VIDC_WARN, "Failed to unvote for buses\n");
	device->power_enabled = false;
}

static inline int __suspend(struct venus_hfi_device *device)
{
	int rc = 0;

	if (!device) {
		dprintk(VIDC_ERR, "Invalid params: %pK\n", device);
		return -EINVAL;
	} else if (!device->power_enabled) {
		dprintk(VIDC_DBG, "Power already disabled\n");
		return 0;
	}

	dprintk(VIDC_DBG, "Entering power collapse\n");

	if (device->res->pm_qos_latency_us &&
		pm_qos_request_active(&device->qos))
		pm_qos_remove_request(&device->qos);

	rc = __tzbsp_set_video_state(TZBSP_VIDEO_STATE_SUSPEND);
	if (rc) {
		dprintk(VIDC_WARN, "Failed to suspend video core %d\n", rc);
		goto err_tzbsp_suspend;
	}

	__venus_power_off(device, true);
	dprintk(VIDC_INFO, "Venus power collapsed\n");
	return rc;

err_tzbsp_suspend:
	return rc;
}

static inline int __resume(struct venus_hfi_device *device)
{
	int rc = 0;

	if (!device) {
		dprintk(VIDC_ERR, "Invalid params: %pK\n", device);
		return -EINVAL;
	} else if (device->power_enabled) {
		dprintk(VIDC_DBG, "Power is already enabled\n");
		goto exit;
	} else if (!__core_in_valid_state(device)) {
		dprintk(VIDC_DBG, "venus_hfi_device in deinit state.");
		return -EINVAL;
	}

	dprintk(VIDC_DBG, "Resuming from power collapse\n");
	rc = __venus_power_on(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to power on venus\n");
		goto err_venus_power_on;
	}

	/* Reboot the firmware */
	rc = __tzbsp_set_video_state(TZBSP_VIDEO_STATE_RESUME);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to resume video core %d\n", rc);
		goto err_set_video_state;
	}

	/*
	 * Re-program all of the registers that get reset as a result of
	 * regulator_disable() and _enable()
	 */
	__set_registers(device);
	__setup_ucregion_memory_map(device);
	/* Wait for boot completion */
	rc = __boot_firmware(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to reset venus core\n");
		goto err_reset_core;
	}

	/*
	 * Work around for H/W bug, need to reprogram these registers once
	 * firmware is out reset
	 */
	__set_threshold_registers(device);

	if (device->res->pm_qos_latency_us) {
#ifdef CONFIG_SMP
		device->qos.type = PM_QOS_REQ_AFFINE_IRQ;
		device->qos.irq = device->hal_data->irq;
#endif
		pm_qos_add_request(&device->qos, PM_QOS_CPU_DMA_LATENCY,
				device->res->pm_qos_latency_us);
	}
	dprintk(VIDC_INFO, "Resumed from power collapse\n");
exit:
	device->skip_pc_count = 0;
	return rc;
err_reset_core:
	__tzbsp_set_video_state(TZBSP_VIDEO_STATE_SUSPEND);
err_set_video_state:
	__venus_power_off(device, true);
err_venus_power_on:
	dprintk(VIDC_ERR, "Failed to resume from power collapse\n");
	return rc;
}

static int __load_fw(struct venus_hfi_device *device)
{
	int rc = 0;
	/* Initialize resources */
	rc = __init_resources(device, device->res);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to init resources: %d\n", rc);
		goto fail_init_res;
	}

	rc = __initialize_packetization(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to initialize packetization\n");
		goto fail_init_pkt;
	}
	trace_msm_v4l2_vidc_fw_load_start("msm_v4l2_vidc venus_fw load start");

	rc = __venus_power_on(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to power on venus in in load_fw\n");
		goto fail_venus_power_on;
	}

	if ((!device->res->use_non_secure_pil && !device->res->firmware_base)
			|| device->res->use_non_secure_pil) {
=======
	rc = venus_hfi_iommu_attach(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to attach iommu\n");
		goto fail_iommu_attach;
	}

	if ((!device->res->use_non_secure_pil && !device->res->firmware_base)
			|| (device->res->use_non_secure_pil)) {

>>>>>>> p9x
		if (!device->resources.fw.cookie)
			device->resources.fw.cookie =
				subsystem_get_with_fwname("venus",
				device->res->fw_name);

		if (IS_ERR_OR_NULL(device->resources.fw.cookie)) {
			dprintk(VIDC_ERR, "Failed to download firmware\n");
			device->resources.fw.cookie = NULL;
			rc = -ENOMEM;
			goto fail_load_fw;
		}
	}
<<<<<<< HEAD

	if (!device->res->use_non_secure_pil && !device->res->firmware_base) {
		rc = __protect_cp_mem(device);
=======
	device->power_enabled = true;

	/* Hand off control of regulators to h/w _after_ enabling clocks */
	venus_hfi_enable_hw_power_collapse(device);

	if (!device->res->use_non_secure_pil && !device->res->firmware_base) {
		rc = protect_cp_mem(device);
>>>>>>> p9x
		if (rc) {
			dprintk(VIDC_ERR, "Failed to protect memory\n");
			goto fail_protect_mem;
		}
	}
<<<<<<< HEAD
	trace_msm_v4l2_vidc_fw_load_end("msm_v4l2_vidc venus_fw load end");
	return rc;
fail_protect_mem:
=======

	trace_msm_v4l2_vidc_fw_load_end("msm_v4l2_vidc venus_fw load end");
	return rc;
fail_protect_mem:
	device->power_enabled = false;
>>>>>>> p9x
	if (device->resources.fw.cookie)
		subsystem_put(device->resources.fw.cookie);
	device->resources.fw.cookie = NULL;
fail_load_fw:
<<<<<<< HEAD
	__venus_power_off(device, true);
fail_venus_power_on:
fail_init_pkt:
	__deinit_resources(device);
fail_init_res:
=======
	venus_hfi_iommu_detach(device);
fail_iommu_attach:
	venus_hfi_disable_unprepare_clks(device);
fail_enable_clks:
	venus_hfi_disable_regulators(device);
fail_enable_gdsc:
	venus_hfi_unvote_buses(device);
fail_vote_buses:
>>>>>>> p9x
	trace_msm_v4l2_vidc_fw_load_end("msm_v4l2_vidc venus_fw load end");
	return rc;
}

<<<<<<< HEAD
static void __unload_fw(struct venus_hfi_device *device)
{
	if (!device->resources.fw.cookie)
		return;

	cancel_delayed_work(&venus_hfi_pm_work);
	if (device->state != VENUS_STATE_DEINIT)
		flush_workqueue(device->venus_pm_workq);

	__vote_buses(device, NULL, 0);
	subsystem_put(device->resources.fw.cookie);
	__interface_queues_release(device);
	__venus_power_off(device, false);
	device->resources.fw.cookie = NULL;
	__deinit_resources(device);
=======
static void venus_hfi_unload_fw(void *dev)
{
	struct venus_hfi_device *device = dev;
	if (!device) {
		dprintk(VIDC_ERR, "%s Invalid paramter: %pK\n",
			__func__, device);
		return;
	}
	if (device->resources.fw.cookie) {
		cancel_delayed_work(&venus_hfi_pm_work);
		flush_workqueue(device->venus_pm_workq);
		subsystem_put(device->resources.fw.cookie);
		venus_hfi_interface_queues_release(dev);
		/* Halt the AXI to make sure there are no pending transactions.
		 * Clocks should be unprepared after making sure axi is halted.
		 */
		if (venus_hfi_halt_axi(device))
			dprintk(VIDC_WARN, "Failed to halt AXI\n");
		/* Detach IOMMU only when AXI is halted */
		venus_hfi_iommu_detach(device);
		venus_hfi_disable_unprepare_clks(device);
		venus_hfi_disable_regulators(device);
		venus_hfi_unvote_buses(device);
		device->power_enabled = false;
		device->resources.fw.cookie = NULL;
	}
>>>>>>> p9x
}

static int venus_hfi_get_fw_info(void *dev, struct hal_fw_info *fw_info)
{
<<<<<<< HEAD
	int i = 0, j = 0;
=======
	int rc = 0, i = 0, j = 0;
>>>>>>> p9x
	struct venus_hfi_device *device = dev;
	u32 smem_block_size = 0;
	u8 *smem_table_ptr;
	char version[VENUS_VERSION_LENGTH];
<<<<<<< HEAD
=======
	const u32 version_string_size = VENUS_VERSION_LENGTH;
>>>>>>> p9x
	const u32 smem_image_index_venus = 14 * 128;

	if (!device || !fw_info) {
		dprintk(VIDC_ERR,
<<<<<<< HEAD
			"%s Invalid parameter: device = %pK fw_info = %pK\n",
			__func__, device, fw_info);
		return -EINVAL;
	}

	mutex_lock(&device->lock);

=======
			"%s Invalid paramter: device = %pK fw_info = %pK\n",
				__func__, device, fw_info);
		return -EINVAL;
	}

>>>>>>> p9x
	smem_table_ptr = smem_get_entry(SMEM_IMAGE_VERSION_TABLE,
			&smem_block_size, 0, SMEM_ANY_HOST_FLAG);
	if (smem_table_ptr &&
			((smem_image_index_venus +
<<<<<<< HEAD
			  VENUS_VERSION_LENGTH) <= smem_block_size))
		memcpy(version,
			smem_table_ptr + smem_image_index_venus,
			VENUS_VERSION_LENGTH);

	while (version[i++] != 'V' && i < VENUS_VERSION_LENGTH)
		;

	if (i == VENUS_VERSION_LENGTH - 1) {
		dprintk(VIDC_WARN, "Venus version string is not proper\n");
		fw_info->version[0] = '\0';
		goto fail_version_string;
	}

	for (i--; i < VENUS_VERSION_LENGTH && j < VENUS_VERSION_LENGTH - 1; i++)
		fw_info->version[j++] = version[i];
	fw_info->version[j] = '\0';

fail_version_string:
	dprintk(VIDC_DBG, "F/W version retrieved : %s\n", fw_info->version);
	fw_info->base_addr = device->hal_data->firmware_base;
	fw_info->register_base = device->res->register_base;
	fw_info->register_size = device->hal_data->register_size;
	fw_info->irq = device->hal_data->irq;

	mutex_unlock(&device->lock);
	return 0;
}

static int venus_hfi_get_core_capabilities(void *dev)
{
	struct venus_hfi_device *device = dev;
	int rc = 0;

	if (!device)
		return -EINVAL;

	mutex_lock(&device->lock);

=======
			  version_string_size) <= smem_block_size))
		memcpy(version,
			smem_table_ptr + smem_image_index_venus,
			version_string_size);

	while (version[i++] != 'V' && i < version_string_size)
		;

	for (i--; i < version_string_size && j < version_string_size; i++)
		fw_info->version[j++] = version[i];
	fw_info->version[version_string_size - 1] = '\0';
	dprintk(VIDC_DBG, "F/W version retrieved : %s\n", fw_info->version);

	fw_info->base_addr = (u32)device->hal_data->firmware_base;
	if ((phys_addr_t)fw_info->base_addr !=
		device->hal_data->firmware_base) {
		dprintk(VIDC_INFO,
				"%s: firmware_base (0x%pa) truncated to 0x%x",
				__func__, &device->hal_data->firmware_base,
				fw_info->base_addr);
	}

	fw_info->register_base = (u32)device->res->register_base;
	if ((phys_addr_t)fw_info->register_base != device->res->register_base) {
		dprintk(VIDC_INFO,
				"%s: register_base (0x%pa) truncated to 0x%x",
				__func__, &device->res->register_base,
				fw_info->register_base);
	}

	fw_info->register_size = device->hal_data->register_size;
	fw_info->irq = device->hal_data->irq;
	return rc;
}

int venus_hfi_get_stride_scanline(int color_fmt,
	int width, int height, int *stride, int *scanlines) {
	*stride = VENUS_Y_STRIDE(color_fmt, width);
	*scanlines = VENUS_Y_SCANLINES(color_fmt, height);
	return 0;
}

int venus_hfi_get_core_capabilities(void)
{
	int rc = 0;
>>>>>>> p9x
	rc = HAL_VIDEO_ENCODER_ROTATION_CAPABILITY |
		HAL_VIDEO_ENCODER_SCALING_CAPABILITY |
		HAL_VIDEO_ENCODER_DEINTERLACE_CAPABILITY |
		HAL_VIDEO_DECODER_MULTI_STREAM_CAPABILITY;
<<<<<<< HEAD

	mutex_unlock(&device->lock);

	return rc;
}

static int __initialize_packetization(struct venus_hfi_device *device)
{
	int rc = 0;
	const char *hfi_version;

	if (!device || !device->res) {
=======
	return rc;
}

static int venus_hfi_initialize_packetization(struct venus_hfi_device *device)
{
	int major_version;
	int rc = 0;

	if (!device) {
>>>>>>> p9x
		dprintk(VIDC_ERR, "%s - invalid param\n", __func__);
		return -EINVAL;
	}

<<<<<<< HEAD
	hfi_version = device->res->hfi_version;

	if (!hfi_version) {
		device->packetization_type = HFI_PACKETIZATION_LEGACY;
	} else if (!strcmp(hfi_version, "3xx")) {
		device->packetization_type = HFI_PACKETIZATION_3XX;
	} else {
		dprintk(VIDC_ERR, "Unsupported hfi version\n");
		return -EINVAL;
	}

=======
	rc = venus_hfi_enable_regulators(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to enable GDSC in %s Err code = %d\n",
			__func__, rc);
		goto exit;
	}

	rc = venus_hfi_prepare_enable_clks(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to enable clocks\n");
		goto exit;
	}

	rc = venus_hfi_read_register(device, VIDC_WRAPPER_HW_VERSION);
	major_version = (rc & VIDC_WRAPPER_HW_VERSION_MAJOR_VERSION_MASK) >>
			VIDC_WRAPPER_HW_VERSION_MAJOR_VERSION_SHIFT;

	venus_hfi_disable_unprepare_clks(device);

	rc = venus_hfi_disable_regulators(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to disable gdsc\n");
		goto exit;
	}

	dprintk(VIDC_DBG, "VENUS H/w Version is %d.%d.%d\n",
		major_version,
		(rc & VIDC_WRAPPER_HW_VERSION_MINOR_VERSION_MASK) >>
		VIDC_WRAPPER_HW_VERSION_MINOR_VERSION_SHIFT,
		rc & VIDC_WRAPPER_HW_VERSION_STEP_VERSION_MASK);

	if (major_version <= 2)
		device->packetization_type = HFI_PACKETIZATION_LEGACY;
	else
		device->packetization_type = HFI_PACKETIZATION_3XX;

>>>>>>> p9x
	device->pkt_ops = hfi_get_pkt_ops_handle(device->packetization_type);
	if (!device->pkt_ops) {
		rc = -EINVAL;
		dprintk(VIDC_ERR, "Failed to get pkt_ops handle\n");
	}
<<<<<<< HEAD

	return rc;
}

static struct venus_hfi_device *__add_device(u32 device_id,
=======
exit:
	return rc;
}

static void *venus_hfi_add_device(u32 device_id,
>>>>>>> p9x
			struct msm_vidc_platform_resources *res,
			hfi_cmd_response_callback callback)
{
	struct venus_hfi_device *hdevice = NULL;
	int rc = 0;

	if (!res || !callback) {
		dprintk(VIDC_ERR, "Invalid Parameters\n");
		return NULL;
	}

	dprintk(VIDC_INFO, "entered , device_id: %d\n", device_id);

	hdevice = (struct venus_hfi_device *)
			kzalloc(sizeof(struct venus_hfi_device), GFP_KERNEL);
	if (!hdevice) {
		dprintk(VIDC_ERR, "failed to allocate new device\n");
<<<<<<< HEAD
		goto exit;
	}

	hdevice->response_pkt = kmalloc_array(max_packets,
				sizeof(*hdevice->response_pkt), GFP_TEMPORARY);
	if (!hdevice->response_pkt) {
		dprintk(VIDC_ERR, "failed to allocate response_pkt\n");
		goto err_cleanup;
	}

	rc = __init_regs_and_interrupts(hdevice, res);
	if (rc)
		goto err_cleanup;

	hdevice->res = res;
=======
		goto err_alloc;
	}

	rc = venus_hfi_init_regs_and_interrupts(hdevice, res);
	if (rc)
		goto err_init_regs;

>>>>>>> p9x
	hdevice->device_id = device_id;
	hdevice->callback = callback;

	hdevice->vidc_workq = create_singlethread_workqueue(
		"msm_vidc_workerq_venus");
	if (!hdevice->vidc_workq) {
		dprintk(VIDC_ERR, ": create vidc workq failed\n");
<<<<<<< HEAD
		goto err_cleanup;
	}

=======
		goto error_createq;
	}
>>>>>>> p9x
	hdevice->venus_pm_workq = create_singlethread_workqueue(
			"pm_workerq_venus");
	if (!hdevice->venus_pm_workq) {
		dprintk(VIDC_ERR, ": create pm workq failed\n");
<<<<<<< HEAD
		goto err_cleanup;
	}

	if (!hal_ctxt.dev_count)
		INIT_LIST_HEAD(&hal_ctxt.dev_head);

	mutex_init(&hdevice->lock);
=======
		goto error_createq_pm;
	}

	mutex_init(&hdevice->read_lock);
	mutex_init(&hdevice->write_lock);
	mutex_init(&hdevice->session_lock);
	mutex_init(&hdevice->resource_lock);
	mutex_init(&hdevice->clock_lock);

	if (hal_ctxt.dev_count == 0)
		INIT_LIST_HEAD(&hal_ctxt.dev_head);

>>>>>>> p9x
	INIT_LIST_HEAD(&hdevice->list);
	INIT_LIST_HEAD(&hdevice->sess_head);
	list_add_tail(&hdevice->list, &hal_ctxt.dev_head);
	hal_ctxt.dev_count++;

<<<<<<< HEAD
	return hdevice;

err_cleanup:
	if (hdevice->vidc_workq)
		destroy_workqueue(hdevice->vidc_workq);
	kfree(hdevice->response_pkt);
	kfree(hdevice);
exit:
	return NULL;
}

static struct venus_hfi_device *__get_device(u32 device_id,
				struct msm_vidc_platform_resources *res,
				hfi_cmd_response_callback callback)
{
=======
	return (void *) hdevice;
error_createq_pm:
	destroy_workqueue(hdevice->vidc_workq);
error_createq:
err_init_regs:
	kfree(hdevice);
err_alloc:
	return NULL;
}

static void *venus_hfi_get_device(u32 device_id,
				struct msm_vidc_platform_resources *res,
				hfi_cmd_response_callback callback)
{
	struct venus_hfi_device *device;
	int rc = 0;

>>>>>>> p9x
	if (!res || !callback) {
		dprintk(VIDC_ERR, "Invalid params: %pK %pK\n", res, callback);
		return NULL;
	}

<<<<<<< HEAD
	return __add_device(device_id, res, callback);
=======
	device = venus_hfi_add_device(device_id, res, &handle_cmd_response);
	if (!device) {
		dprintk(VIDC_ERR, "Failed to create HFI device\n");
		return NULL;
	}

	rc = venus_hfi_init_resources(device, res);
	if (rc) {
		if (rc != -EPROBE_DEFER)
			dprintk(VIDC_ERR, "Failed to init resources: %d\n", rc);
		goto err_fail_init_res;
	}

	rc = venus_hfi_initialize_packetization(device);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to initialize packetization\n");
		goto err_fail_init_res;
	}
	return device;

err_fail_init_res:
	venus_hfi_delete_device(device);
	return ERR_PTR(rc);
>>>>>>> p9x
}

void venus_hfi_delete_device(void *device)
{
	struct venus_hfi_device *close, *tmp, *dev;

<<<<<<< HEAD
	if (!device)
		return;

	dev = (struct venus_hfi_device *) device;

	mutex_lock(&dev->lock);
	__iommu_detach(dev);
	mutex_unlock(&dev->lock);

	list_for_each_entry_safe(close, tmp, &hal_ctxt.dev_head, list) {
		if (close->hal_data->irq == dev->hal_data->irq) {
			hal_ctxt.dev_count--;
			list_del(&close->list);
			destroy_workqueue(close->vidc_workq);
			destroy_workqueue(close->venus_pm_workq);
			free_irq(dev->hal_data->irq, close);
			iounmap(dev->hal_data->register_base);
			kfree(close->hal_data);
			kfree(close->response_pkt);
			kfree(close);
			break;
		}
=======
	if (device) {
		venus_hfi_deinit_resources(device);
		dev = (struct venus_hfi_device *) device;
		list_for_each_entry_safe(close, tmp, &hal_ctxt.dev_head, list) {
			if (close->hal_data->irq == dev->hal_data->irq) {
				hal_ctxt.dev_count--;
				free_irq(dev->hal_data->irq, close);
				list_del(&close->list);
				destroy_workqueue(close->vidc_workq);
				destroy_workqueue(close->venus_pm_workq);
				kfree(close->hal_data);
				kfree(close);
				break;
			}
		}

>>>>>>> p9x
	}
}

static void venus_init_hfi_callbacks(struct hfi_device *hdev)
{
	hdev->core_init = venus_hfi_core_init;
	hdev->core_release = venus_hfi_core_release;
<<<<<<< HEAD
=======
	hdev->core_pc_prep = venus_hfi_core_pc_prep;
>>>>>>> p9x
	hdev->core_ping = venus_hfi_core_ping;
	hdev->core_trigger_ssr = venus_hfi_core_trigger_ssr;
	hdev->session_init = venus_hfi_session_init;
	hdev->session_end = venus_hfi_session_end;
	hdev->session_abort = venus_hfi_session_abort;
	hdev->session_clean = venus_hfi_session_clean;
	hdev->session_set_buffers = venus_hfi_session_set_buffers;
	hdev->session_release_buffers = venus_hfi_session_release_buffers;
	hdev->session_load_res = venus_hfi_session_load_res;
	hdev->session_release_res = venus_hfi_session_release_res;
	hdev->session_start = venus_hfi_session_start;
	hdev->session_continue = venus_hfi_session_continue;
	hdev->session_stop = venus_hfi_session_stop;
	hdev->session_etb = venus_hfi_session_etb;
	hdev->session_ftb = venus_hfi_session_ftb;
<<<<<<< HEAD
	hdev->session_process_batch = venus_hfi_session_process_batch;
=======
>>>>>>> p9x
	hdev->session_parse_seq_hdr = venus_hfi_session_parse_seq_hdr;
	hdev->session_get_seq_hdr = venus_hfi_session_get_seq_hdr;
	hdev->session_get_buf_req = venus_hfi_session_get_buf_req;
	hdev->session_flush = venus_hfi_session_flush;
	hdev->session_set_property = venus_hfi_session_set_property;
	hdev->session_get_property = venus_hfi_session_get_property;
	hdev->scale_clocks = venus_hfi_scale_clocks;
<<<<<<< HEAD
	hdev->vote_bus = venus_hfi_vote_buses;
	hdev->get_fw_info = venus_hfi_get_fw_info;
	hdev->get_core_capabilities = venus_hfi_get_core_capabilities;
=======
	hdev->vote_bus = venus_hfi_vote_active_buses;
	hdev->unvote_bus = venus_hfi_unvote_active_buses;
	hdev->iommu_get_domain_partition = venus_hfi_iommu_get_domain_partition;
	hdev->load_fw = venus_hfi_load_fw;
	hdev->unload_fw = venus_hfi_unload_fw;
	hdev->get_fw_info = venus_hfi_get_fw_info;
	hdev->get_stride_scanline = venus_hfi_get_stride_scanline;
	hdev->get_core_capabilities = venus_hfi_get_core_capabilities;
	hdev->power_enable = venus_hfi_power_enable;
>>>>>>> p9x
	hdev->suspend = venus_hfi_suspend;
	hdev->get_core_clock_rate = venus_hfi_get_core_clock_rate;
	hdev->get_default_properties = venus_hfi_get_default_properties;
}

int venus_hfi_initialize(struct hfi_device *hdev, u32 device_id,
		struct msm_vidc_platform_resources *res,
		hfi_cmd_response_callback callback)
{
	int rc = 0;

	if (!hdev || !res || !callback) {
		dprintk(VIDC_ERR, "Invalid params: %pK %pK %pK\n",
			hdev, res, callback);
		rc = -EINVAL;
		goto err_venus_hfi_init;
	}
<<<<<<< HEAD

	hdev->hfi_device_data = __get_device(device_id, res, callback);
=======
	hdev->hfi_device_data = venus_hfi_get_device(device_id, res, callback);
>>>>>>> p9x

	if (IS_ERR_OR_NULL(hdev->hfi_device_data)) {
		rc = PTR_ERR(hdev->hfi_device_data) ?: -EINVAL;
		goto err_venus_hfi_init;
	}

	venus_init_hfi_callbacks(hdev);

err_venus_hfi_init:
	return rc;
}

