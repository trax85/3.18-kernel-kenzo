<<<<<<< HEAD
/* Copyright (c) 2014-2017, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

/*
 * IPC ROUTER MHI XPRT module.
 */
#include <linux/delay.h>
#include <linux/ipc_router_xprt.h>
#include <linux/module.h>
#include <linux/msm_mhi.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/types.h>
<<<<<<< HEAD
#include <linux/spinlock.h>
=======

>>>>>>> p9x

static int ipc_router_mhi_xprt_debug_mask;
module_param_named(debug_mask, ipc_router_mhi_xprt_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

#define D(x...) do { \
if (ipc_router_mhi_xprt_debug_mask) \
	pr_info(x); \
} while (0)

#define NUM_MHI_XPRTS 1
#define XPRT_NAME_LEN 32
#define IPC_ROUTER_MHI_XPRT_MAX_PKT_SIZE 0x1000
#define IPC_ROUTER_MHI_XPRT_NUM_TRBS 10

/**
<<<<<<< HEAD
 * ipc_router_mhi_addr_map - Struct for virtual address to IPC Router
 *				packet mapping.
 * @list_node: Address mapping list node used by mhi transport map list.
 * @virt_addr: The virtual address in mapping.
 * @pkt: The IPC Router packet for the virtual address
 */
struct ipc_router_mhi_addr_map {
	struct list_head list_node;
	void *virt_addr;
	struct rr_packet *pkt;
};

/**
=======
>>>>>>> p9x
 * ipc_router_mhi_channel - MHI Channel related information
 * @out_chan_id: Out channel ID for use by IPC ROUTER enumerated in MHI driver.
 * @out_handle: MHI Output channel handle.
 * @out_clnt_info: IPC Router callbacks/info to be passed to the MHI driver.
 * @in_chan_id: In channel ID for use by IPC ROUTER enumerated in MHI driver.
 * @in_handle: MHI Input channel handle.
 * @in_clnt_info: IPC Router callbacks/info to be passed to the MHI driver.
 * @state_lock: Lock to protect access to the state information.
 * @out_chan_enabled: State of the outgoing channel.
 * @in_chan_enabled: State of the incoming channel.
<<<<<<< HEAD
=======
 * @bytes_to_tx: Remaining bytes to be transmitted in a packet.
>>>>>>> p9x
 * @bytes_to_rx: Remaining bytes to be received in a packet.
 * @in_skbq_lock: Lock to protect access to the input skbs queue.
 * @in_skbq: Queue containing the input buffers.
 * @max_packet_size: Possible maximum packet size.
 * @num_trbs: Number of TRBs.
 * @mhi_xprtp: Pointer to IPC Router MHI XPRT.
 */
struct ipc_router_mhi_channel {
	enum MHI_CLIENT_CHANNEL out_chan_id;
	struct mhi_client_handle *out_handle;
	struct mhi_client_info_t out_clnt_info;

	enum MHI_CLIENT_CHANNEL in_chan_id;
	struct mhi_client_handle *in_handle;
	struct mhi_client_info_t in_clnt_info;

	struct mutex state_lock;
	bool out_chan_enabled;
	bool in_chan_enabled;
<<<<<<< HEAD
=======
	int bytes_to_tx;
>>>>>>> p9x
	int bytes_to_rx;

	struct mutex in_skbq_lock;
	struct sk_buff_head in_skbq;
	size_t max_packet_size;
	uint32_t num_trbs;
	void *mhi_xprtp;
};

/**
 * ipc_router_mhi_xprt - IPC Router's MHI XPRT structure
 * @list: IPC router's MHI XPRTs list.
 * @ch_hndl: Data Structure to hold MHI Channel information.
 * @xprt_name: Name of the XPRT to be registered with IPC Router.
 * @xprt: IPC Router XPRT structure to contain MHI XPRT specific info.
 * @wq: Workqueue to queue read & other XPRT related works.
 * @read_work: Read Work to perform read operation from MHI Driver.
 * @in_pkt: Pointer to any partially read packet.
 * @write_wait_q: Wait Queue to handle the write events.
 * @sft_close_complete: Variable to indicate completion of SSR handling
 *			by IPC Router.
 * @xprt_version: IPC Router header version supported by this XPRT.
 * @xprt_option: XPRT specific options to be handled by IPC Router.
<<<<<<< HEAD
 * @tx_addr_map_list_lock: The lock to protect the address mapping list for TX
 *			operations.
 * @tx_addr_map_list: Virtual address mapping list for TX operations.
 * @rx_addr_map_list_lock: The lock to protect the address mapping list for RX
 *			operations.
 * @rx_addr_map_list: Virtual address mapping list for RX operations.
=======
>>>>>>> p9x
 */
struct ipc_router_mhi_xprt {
	struct list_head list;
	struct ipc_router_mhi_channel ch_hndl;
	char xprt_name[XPRT_NAME_LEN];
	struct msm_ipc_router_xprt xprt;
	struct workqueue_struct *wq;
	struct work_struct read_work;
	struct rr_packet *in_pkt;
	wait_queue_head_t write_wait_q;
	struct completion sft_close_complete;
	unsigned xprt_version;
	unsigned xprt_option;
<<<<<<< HEAD
	spinlock_t tx_addr_map_list_lock;
	struct list_head tx_addr_map_list;
	spinlock_t rx_addr_map_list_lock;
	struct list_head rx_addr_map_list;
=======
>>>>>>> p9x
};

struct ipc_router_mhi_xprt_work {
	struct ipc_router_mhi_xprt *mhi_xprtp;
	enum MHI_CLIENT_CHANNEL chan_id;
<<<<<<< HEAD
};

static void mhi_xprt_read_data(struct work_struct *work);
static void mhi_xprt_enable_event(struct ipc_router_mhi_xprt_work *xprt_work);
static void mhi_xprt_disable_event(struct ipc_router_mhi_xprt_work *xprt_work);
=======
	struct work_struct work;
};

static void mhi_xprt_read_data(struct work_struct *work);
static void mhi_xprt_enable_event(struct work_struct *work);
static void mhi_xprt_disable_event(struct work_struct *work);
>>>>>>> p9x

/**
 * ipc_router_mhi_xprt_config - Config. Info. of each MHI XPRT
 * @out_chan_id: Out channel ID for use by IPC ROUTER enumerated in MHI driver.
 * @in_chan_id: In channel ID for use by IPC ROUTER enumerated in MHI driver.
 * @xprt_name: Name of the XPRT to be registered with IPC Router.
 * @link_id: Network Cluster ID to which this XPRT belongs to.
 * @xprt_version: IPC Router header version supported by this XPRT.
 */
struct ipc_router_mhi_xprt_config {
	enum MHI_CLIENT_CHANNEL out_chan_id;
	enum MHI_CLIENT_CHANNEL in_chan_id;
	char xprt_name[XPRT_NAME_LEN];
	uint32_t link_id;
	uint32_t xprt_version;
};

#define MODULE_NAME "ipc_router_mhi_xprt"
static DEFINE_MUTEX(mhi_xprt_list_lock_lha1);
static LIST_HEAD(mhi_xprt_list);

/*
<<<<<<< HEAD
 * ipc_router_mhi_release_pkt() - Release a cloned IPC Router packet
 * @ref: Reference to the kref object in the IPC Router packet.
 */
void ipc_router_mhi_release_pkt(struct kref *ref)
{
	struct rr_packet *pkt = container_of(ref, struct rr_packet, ref);

	release_pkt(pkt);
}

/*
 * ipc_router_mhi_xprt_find_addr_map() - Search the mapped virtual address
 * @addr_map_list: The list of address mappings.
 * @addr_map_list_lock: Reference to the lock that protects the @addr_map_list.
 * @addr: The virtual address that needs to be found.
 *
 * Return: The mapped virtual Address if found, NULL otherwise.
 */
void *ipc_router_mhi_xprt_find_addr_map(struct list_head *addr_map_list,
				spinlock_t *addr_map_list_lock, void *addr)
{
	struct ipc_router_mhi_addr_map *addr_mapping;
	struct ipc_router_mhi_addr_map *tmp_addr_mapping;
	unsigned long flags;
	void *virt_addr;

	if (!addr_map_list || !addr_map_list_lock)
		return NULL;
	spin_lock_irqsave(addr_map_list_lock, flags);
	list_for_each_entry_safe(addr_mapping, tmp_addr_mapping,
				addr_map_list, list_node) {
		if (addr_mapping->virt_addr == addr) {
			virt_addr = addr_mapping->virt_addr;
			list_del(&addr_mapping->list_node);
			if (addr_mapping->pkt)
				kref_put(&addr_mapping->pkt->ref,
					ipc_router_mhi_release_pkt);
			kfree(addr_mapping);
			spin_unlock_irqrestore(addr_map_list_lock, flags);
			return virt_addr;
		}
	}
	spin_unlock_irqrestore(addr_map_list_lock, flags);
	IPC_RTR_ERR(
		"%s: Virtual address mapping [%p] not found\n",
		__func__, (void *)addr);
	return NULL;
}

/*
 * ipc_router_mhi_xprt_add_addr_map() - Add a virtual address mapping structure
 * @addr_map_list: The list of address mappings.
 * @addr_map_list_lock: Reference to the lock that protects the @addr_map_list.
 * @pkt: The IPC Router packet that contains the virtual address in skbs.
 * @virt_addr: The virtual address which needs to be added.
 *
 * Return: 0 on success, standard Linux error code otherwise.
 */
int ipc_router_mhi_xprt_add_addr_map(struct list_head *addr_map_list,
				spinlock_t *addr_map_list_lock,
				struct rr_packet *pkt, void *virt_addr)
{
	struct ipc_router_mhi_addr_map *addr_mapping;
	unsigned long flags;

	if (!addr_map_list || !addr_map_list_lock)
		return -EINVAL;
	addr_mapping = kmalloc(sizeof(*addr_mapping), GFP_KERNEL);
	if (!addr_mapping)
		return -ENOMEM;
	addr_mapping->virt_addr = virt_addr;
	addr_mapping->pkt = pkt;
	spin_lock_irqsave(addr_map_list_lock, flags);
	if (addr_mapping->pkt)
		kref_get(&addr_mapping->pkt->ref);
	list_add_tail(&addr_mapping->list_node, addr_map_list);
	spin_unlock_irqrestore(addr_map_list_lock, flags);
	return 0;
}

/*
=======
>>>>>>> p9x
 * mhi_xprt_queue_in_buffers() - Queue input buffers
 * @mhi_xprtp: MHI XPRT in which the input buffer has to be queued.
 * @num_trbs: Number of buffers to be queued.
 *
 * @return: number of buffers queued.
 */
int mhi_xprt_queue_in_buffers(struct ipc_router_mhi_xprt *mhi_xprtp,
			      uint32_t num_trbs)
{
	int i;
	struct sk_buff *skb;
<<<<<<< HEAD
	uint32_t buf_size = mhi_xprtp->ch_hndl.max_packet_size;
	int rc_val = 0;
=======
	dma_addr_t dma_addr;
	uint32_t buf_size = mhi_xprtp->ch_hndl.max_packet_size;
	enum MHI_STATUS rc_val = MHI_STATUS_SUCCESS;
>>>>>>> p9x

	for (i = 0; i < num_trbs; i++) {
		skb = alloc_skb(buf_size, GFP_KERNEL);
		if (!skb) {
			IPC_RTR_ERR("%s: Could not allocate %d SKB(s)\n",
				    __func__, (i + 1));
			break;
		}
<<<<<<< HEAD
		if (ipc_router_mhi_xprt_add_addr_map(
					&mhi_xprtp->rx_addr_map_list,
					&mhi_xprtp->rx_addr_map_list_lock, NULL,
					skb->data) < 0) {
			IPC_RTR_ERR("%s: Could not map %d SKB address\n",
					__func__, (i + 1));
=======
		dma_addr = dma_map_single(NULL, skb->data,
				buf_size, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(NULL, dma_addr)) {
			IPC_RTR_ERR("%s: Failed to map DMA for SKB # %d\n",
				    __func__, (i + 1));
			kfree_skb(skb);
>>>>>>> p9x
			break;
		}
		mutex_lock(&mhi_xprtp->ch_hndl.in_skbq_lock);
		rc_val = mhi_queue_xfer(mhi_xprtp->ch_hndl.in_handle,
<<<<<<< HEAD
					skb->data, buf_size, MHI_EOT);
		if (rc_val) {
			mutex_unlock(&mhi_xprtp->ch_hndl.in_skbq_lock);
			IPC_RTR_ERR("%s: Failed to queue TRB # %d into MHI\n",
				    __func__, (i + 1));
=======
					 dma_addr, buf_size, MHI_EOT);
		if (rc_val != MHI_STATUS_SUCCESS) {
			mutex_unlock(&mhi_xprtp->ch_hndl.in_skbq_lock);
			IPC_RTR_ERR("%s: Failed to queue TRB # %d into MHI\n",
				    __func__, (i + 1));
			dma_unmap_single(NULL, dma_addr,
					 buf_size, DMA_TO_DEVICE);
>>>>>>> p9x
			kfree_skb(skb);
			break;
		}
		skb_queue_tail(&mhi_xprtp->ch_hndl.in_skbq, skb);
		mutex_unlock(&mhi_xprtp->ch_hndl.in_skbq_lock);
	}
	return i;
}

/**
<<<<<<< HEAD
* ipc_router_mhi_set_xprt_version() - Set the IPC Router version in transport
* @xprt:      Reference to the transport structure.
* @version:   The version to be set in transport.
*/
static void ipc_router_mhi_set_xprt_version(struct msm_ipc_router_xprt *xprt,
					   unsigned version)
{
	struct ipc_router_mhi_xprt *mhi_xprtp;

	if (!xprt)
		return;
	mhi_xprtp = container_of(xprt, struct ipc_router_mhi_xprt, xprt);
	mhi_xprtp->xprt_version = version;
}

/**
=======
>>>>>>> p9x
 * ipc_router_mhi_get_xprt_version() - Get IPC Router header version
 *				       supported by the XPRT
 * @xprt: XPRT for which the version information is required.
 *
 * @return: IPC Router header version supported by the XPRT.
 */
static int ipc_router_mhi_get_xprt_version(struct msm_ipc_router_xprt *xprt)
{
	struct ipc_router_mhi_xprt *mhi_xprtp;
	if (!xprt)
		return -EINVAL;
	mhi_xprtp = container_of(xprt, struct ipc_router_mhi_xprt, xprt);

	return (int)mhi_xprtp->xprt_version;
}

/**
 * ipc_router_mhi_get_xprt_option() - Get XPRT options
 * @xprt: XPRT for which the option information is required.
 *
 * @return: Options supported by the XPRT.
 */
static int ipc_router_mhi_get_xprt_option(struct msm_ipc_router_xprt *xprt)
{
	struct ipc_router_mhi_xprt *mhi_xprtp;
	if (!xprt)
		return -EINVAL;
	mhi_xprtp = container_of(xprt, struct ipc_router_mhi_xprt, xprt);

	return (int)mhi_xprtp->xprt_option;
}

/**
 * ipc_router_mhi_write_avail() - Get available write space
 * @xprt: XPRT for which the available write space info. is required.
 *
 * @return: Write space in bytes on success, 0 on SSR.
 */
static int ipc_router_mhi_write_avail(struct msm_ipc_router_xprt *xprt)
{
	int write_avail;
	struct ipc_router_mhi_xprt *mhi_xprtp =
		container_of(xprt, struct ipc_router_mhi_xprt, xprt);

	mutex_lock(&mhi_xprtp->ch_hndl.state_lock);
	if (!mhi_xprtp->ch_hndl.out_chan_enabled)
		write_avail = 0;
	else
		write_avail = mhi_get_free_desc(mhi_xprtp->ch_hndl.out_handle) *
					mhi_xprtp->ch_hndl.max_packet_size;
	mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
	return write_avail;
}

/**
 * ipc_router_mhi_write_skb() - Write a single SKB onto the XPRT
 * @mhi_xprtp: XPRT in which the SKB has to be written.
 * @skb: SKB to be written.
 *
 * @return: return number of bytes written on success,
 *          standard Linux error codes on failure.
 */
static int ipc_router_mhi_write_skb(struct ipc_router_mhi_xprt *mhi_xprtp,
<<<<<<< HEAD
				    struct sk_buff *skb, struct rr_packet *pkt)
=======
				    struct sk_buff *skb)
>>>>>>> p9x
{
	size_t sz_to_write = 0;
	size_t offset = 0;
	int rc;
<<<<<<< HEAD
=======
	dma_addr_t dma_addr;
>>>>>>> p9x

	while (offset < skb->len) {
		wait_event(mhi_xprtp->write_wait_q,
			   mhi_get_free_desc(mhi_xprtp->ch_hndl.out_handle) ||
			   !mhi_xprtp->ch_hndl.out_chan_enabled);
		mutex_lock(&mhi_xprtp->ch_hndl.state_lock);
		if (!mhi_xprtp->ch_hndl.out_chan_enabled) {
			mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
			IPC_RTR_ERR("%s: %s chnl reset\n",
				    __func__, mhi_xprtp->xprt_name);
			return -ENETRESET;
		}

		sz_to_write = min((size_t)(skb->len - offset),
				(size_t)IPC_ROUTER_MHI_XPRT_MAX_PKT_SIZE);
<<<<<<< HEAD
		if (ipc_router_mhi_xprt_add_addr_map(
					&mhi_xprtp->tx_addr_map_list,
					&mhi_xprtp->tx_addr_map_list_lock, pkt,
					skb->data + offset) < 0) {
			IPC_RTR_ERR("%s: Could not map SKB address\n",
					__func__);
			break;
		}

		rc = mhi_queue_xfer(mhi_xprtp->ch_hndl.out_handle,
				    skb->data + offset, sz_to_write,
				    MHI_EOT | MHI_EOB);
		if (rc) {
			mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
			IPC_RTR_ERR("%s: Error queueing mhi_xfer 0x%zx\n",
=======
		dma_addr = dma_map_single(NULL, skb->data + offset,
					  sz_to_write, DMA_TO_DEVICE);
		if (dma_mapping_error(NULL, dma_addr)) {
			mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
			IPC_RTR_ERR("%s: Failed to map DMA 0x%x\n",
				    __func__, sz_to_write);
			return -ENOMEM;
		}
		rc = mhi_queue_xfer(mhi_xprtp->ch_hndl.out_handle,
				     dma_addr, sz_to_write, MHI_EOT | MHI_EOB);
		if (rc != 0) {
			mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
			dma_unmap_single(NULL, dma_addr, sz_to_write,
					 DMA_TO_DEVICE);
			IPC_RTR_ERR("%s: Error queueing mhi_xfer 0x%x\n",
>>>>>>> p9x
				    __func__, sz_to_write);
			return -EFAULT;
		} else {
			offset += sz_to_write;
<<<<<<< HEAD
=======
			mhi_xprtp->ch_hndl.bytes_to_tx += sz_to_write;
>>>>>>> p9x
		}
		mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
	}
	return skb->len;
}

/**
 * ipc_router_mhi_write() - Write to XPRT
 * @data: Data to be written to the XPRT.
 * @len: Length of the data to be written.
 * @xprt: XPRT to which the data has to be written.
 *
 * @return: Data Length on success, standard Linux error codes on failure.
 */
static int ipc_router_mhi_write(void *data,
		uint32_t len, struct msm_ipc_router_xprt *xprt)
{
	struct rr_packet *pkt = (struct rr_packet *)data;
	struct sk_buff *ipc_rtr_pkt;
<<<<<<< HEAD
	struct rr_packet *cloned_pkt;
=======
>>>>>>> p9x
	int rc;
	struct ipc_router_mhi_xprt *mhi_xprtp =
		container_of(xprt, struct ipc_router_mhi_xprt, xprt);

	if (!pkt)
		return -EINVAL;

	if (!len || pkt->length != len)
		return -EINVAL;

<<<<<<< HEAD
	cloned_pkt = clone_pkt(pkt);
	if (!cloned_pkt) {
		pr_err("%s: Error in cloning packet while tx\n", __func__);
		return -ENOMEM;
	}
	D("%s: Ready to write %d bytes\n", __func__, len);
	skb_queue_walk(cloned_pkt->pkt_fragment_q, ipc_rtr_pkt) {
		rc = ipc_router_mhi_write_skb(mhi_xprtp, ipc_rtr_pkt,
						cloned_pkt);
=======
	D("%s: Ready to write %d bytes\n", __func__, len);
	skb_queue_walk(pkt->pkt_fragment_q, ipc_rtr_pkt) {
		rc = ipc_router_mhi_write_skb(mhi_xprtp, ipc_rtr_pkt);
>>>>>>> p9x
		if (rc < 0) {
			IPC_RTR_ERR("%s: Error writing SKB %d\n",
				    __func__, rc);
			break;
		}
	}

<<<<<<< HEAD
	kref_put(&cloned_pkt->ref, ipc_router_mhi_release_pkt);
=======
	wait_event(mhi_xprtp->write_wait_q, !mhi_xprtp->ch_hndl.bytes_to_tx);
>>>>>>> p9x
	if (rc < 0)
		return rc;
	else
		return len;
}

/**
 * mhi_xprt_read_data() - Read work to read from the XPRT
 * @work: Read work to be executed.
 *
 * This function is a read work item queued on a XPRT specific workqueue.
 * The work parameter contains information regarding the XPRT on which this
 * read work has to be performed. The work item keeps reading from the MHI
 * endpoint, until the endpoint returns an error.
 */
static void mhi_xprt_read_data(struct work_struct *work)
{
<<<<<<< HEAD
	void *data_addr;
=======
	dma_addr_t data_addr;
>>>>>>> p9x
	ssize_t data_sz;
	void *skb_data;
	struct sk_buff *skb;
	struct ipc_router_mhi_xprt *mhi_xprtp =
		container_of(work, struct ipc_router_mhi_xprt, read_work);
	struct mhi_result result;
	int rc;

	mutex_lock(&mhi_xprtp->ch_hndl.state_lock);
	if (!mhi_xprtp->ch_hndl.in_chan_enabled) {
		mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
		if (mhi_xprtp->in_pkt)
			release_pkt(mhi_xprtp->in_pkt);
		mhi_xprtp->in_pkt = NULL;
		mhi_xprtp->ch_hndl.bytes_to_rx = 0;
		IPC_RTR_ERR("%s: %s channel reset\n",
			    __func__, mhi_xprtp->xprt.name);
		return;
	}
	mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);

	while (1) {
		rc = mhi_poll_inbound(mhi_xprtp->ch_hndl.in_handle, &result);
<<<<<<< HEAD
		if (rc || !result.buf_addr || !result.bytes_xferd) {
			if (rc != -ENODATA)
				IPC_RTR_ERR("%s: Poll failed %s:%d:%p:%u\n",
					__func__, mhi_xprtp->xprt_name, rc,
					result.buf_addr,
					(unsigned int) result.bytes_xferd);
			break;
		}
		data_addr = result.buf_addr;
=======
		if (rc || !result.payload_buf || !result.bytes_xferd) {
			if (rc != MHI_STATUS_RING_EMPTY)
				IPC_RTR_ERR("%s: Poll failed %s:%d:%p:%zu\n",
					__func__, mhi_xprtp->xprt_name, rc,
					(void *)result.payload_buf,
					result.bytes_xferd);
			break;
		}
		data_addr = result.payload_buf;
>>>>>>> p9x
		data_sz = result.bytes_xferd;

		/* Create a new rr_packet, if first fragment */
		if (!mhi_xprtp->ch_hndl.bytes_to_rx) {
			mhi_xprtp->in_pkt = create_pkt(NULL);
			if (!mhi_xprtp->in_pkt) {
				IPC_RTR_ERR("%s: Couldn't alloc rr_packet\n",
					    __func__);
				return;
			}
			D("%s: Allocated rr_packet\n", __func__);
		}

<<<<<<< HEAD
		skb_data = ipc_router_mhi_xprt_find_addr_map(
					&mhi_xprtp->rx_addr_map_list,
					&mhi_xprtp->rx_addr_map_list_lock,
					data_addr);

		if (!skb_data)
			continue;
=======
		skb_data = dma_to_virt(NULL, data_addr);
		dma_unmap_single(NULL, data_addr, data_sz, DMA_BIDIRECTIONAL);
>>>>>>> p9x
		mutex_lock(&mhi_xprtp->ch_hndl.in_skbq_lock);
		skb_queue_walk(&mhi_xprtp->ch_hndl.in_skbq, skb) {
			if (skb->data == skb_data) {
				skb_unlink(skb, &mhi_xprtp->ch_hndl.in_skbq);
				break;
			}
		}
		mutex_unlock(&mhi_xprtp->ch_hndl.in_skbq_lock);
		skb_put(skb, data_sz);
		skb_queue_tail(mhi_xprtp->in_pkt->pkt_fragment_q, skb);
		mhi_xprtp->in_pkt->length += data_sz;
		if (!mhi_xprtp->ch_hndl.bytes_to_rx)
			mhi_xprtp->ch_hndl.bytes_to_rx =
				ipc_router_peek_pkt_size(skb_data) - data_sz;
		else
			mhi_xprtp->ch_hndl.bytes_to_rx -= data_sz;
		/* Packet is completely read, so notify to router */
		if (!mhi_xprtp->ch_hndl.bytes_to_rx) {
			D("%s: Packet size read %d\n",
			  __func__, mhi_xprtp->in_pkt->length);
			msm_ipc_router_xprt_notify(&mhi_xprtp->xprt,
						IPC_ROUTER_XPRT_EVENT_DATA,
						(void *)mhi_xprtp->in_pkt);
			release_pkt(mhi_xprtp->in_pkt);
			mhi_xprtp->in_pkt = NULL;
		}

		while (mhi_xprt_queue_in_buffers(mhi_xprtp, 1) != 1 &&
		       mhi_xprtp->ch_hndl.in_chan_enabled)
			msleep(100);
	}
}

/**
 * ipc_router_mhi_close() - Close the XPRT
 * @xprt: XPRT which needs to be closed.
 *
 * @return: 0 on success, standard Linux error codes on failure.
 */
static int ipc_router_mhi_close(struct msm_ipc_router_xprt *xprt)
{
	struct ipc_router_mhi_xprt *mhi_xprtp;

	if (!xprt)
		return -EINVAL;
	mhi_xprtp = container_of(xprt, struct ipc_router_mhi_xprt, xprt);

	mutex_lock(&mhi_xprtp->ch_hndl.state_lock);
	mhi_xprtp->ch_hndl.out_chan_enabled = false;
	mhi_xprtp->ch_hndl.in_chan_enabled = false;
	mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
	flush_workqueue(mhi_xprtp->wq);
<<<<<<< HEAD
=======
	mhi_close_channel(mhi_xprtp->ch_hndl.in_handle);
	mhi_close_channel(mhi_xprtp->ch_hndl.out_handle);
>>>>>>> p9x
	return 0;
}

/**
 * mhi_xprt_sft_close_done() - Completion of XPRT reset
 * @xprt: XPRT on which the reset operation is complete.
 *
 * This function is used by IPC Router to signal this MHI XPRT Abstraction
 * Layer(XAL) that the reset of XPRT is completely handled by IPC Router.
 */
static void mhi_xprt_sft_close_done(struct msm_ipc_router_xprt *xprt)
{
	struct ipc_router_mhi_xprt *mhi_xprtp =
		container_of(xprt, struct ipc_router_mhi_xprt, xprt);

	complete_all(&mhi_xprtp->sft_close_complete);
}

/**
 * mhi_xprt_enable_event() - Enable the MHI link for communication
 * @work: Work containing some reference to the link to be enabled.
 *
 * This work is scheduled when the MHI link to the peripheral is up.
 */
<<<<<<< HEAD
static void mhi_xprt_enable_event(struct ipc_router_mhi_xprt_work *xprt_work)
{
=======
static void mhi_xprt_enable_event(struct work_struct *work)
{
	struct ipc_router_mhi_xprt_work *xprt_work =
		container_of(work, struct ipc_router_mhi_xprt_work, work);
>>>>>>> p9x
	struct ipc_router_mhi_xprt *mhi_xprtp = xprt_work->mhi_xprtp;
	int rc;
	bool notify = false;

	if (xprt_work->chan_id == mhi_xprtp->ch_hndl.out_chan_id) {
		rc = mhi_open_channel(mhi_xprtp->ch_hndl.out_handle);
<<<<<<< HEAD
		if (rc) {
			IPC_RTR_ERR("%s Failed to open chan 0x%x, rc %d\n",
				__func__, mhi_xprtp->ch_hndl.out_chan_id, rc);
			return;
=======
		if (rc != MHI_STATUS_SUCCESS) {
			IPC_RTR_ERR("%s Failed to open chan 0x%x, rc %d\n",
				__func__, mhi_xprtp->ch_hndl.out_chan_id, rc);
			goto out_enable_event;
>>>>>>> p9x
		}
		mutex_lock(&mhi_xprtp->ch_hndl.state_lock);
		mhi_xprtp->ch_hndl.out_chan_enabled = true;
		notify = mhi_xprtp->ch_hndl.out_chan_enabled &&
				mhi_xprtp->ch_hndl.in_chan_enabled;
		mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
	} else if (xprt_work->chan_id == mhi_xprtp->ch_hndl.in_chan_id) {
		rc = mhi_open_channel(mhi_xprtp->ch_hndl.in_handle);
<<<<<<< HEAD
		if (rc) {
			IPC_RTR_ERR("%s Failed to open chan 0x%x, rc %d\n",
				__func__, mhi_xprtp->ch_hndl.in_chan_id, rc);
			return;
=======
		if (rc != MHI_STATUS_SUCCESS) {
			IPC_RTR_ERR("%s Failed to open chan 0x%x, rc %d\n",
				__func__, mhi_xprtp->ch_hndl.in_chan_id, rc);
			goto out_enable_event;
>>>>>>> p9x
		}
		mutex_lock(&mhi_xprtp->ch_hndl.state_lock);
		mhi_xprtp->ch_hndl.in_chan_enabled = true;
		notify = mhi_xprtp->ch_hndl.out_chan_enabled &&
				mhi_xprtp->ch_hndl.in_chan_enabled;
		mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
	}

	/* Register the XPRT before receiving any data */
	if (notify) {
		msm_ipc_router_xprt_notify(&mhi_xprtp->xprt,
				   IPC_ROUTER_XPRT_EVENT_OPEN, NULL);
		D("%s: Notified IPC Router of %s OPEN\n",
		  __func__, mhi_xprtp->xprt.name);
	}

	if (xprt_work->chan_id != mhi_xprtp->ch_hndl.in_chan_id)
<<<<<<< HEAD
		return;

	rc = mhi_xprt_queue_in_buffers(mhi_xprtp, mhi_xprtp->ch_hndl.num_trbs);
	if (rc > 0)
		return;
=======
		goto out_enable_event;

	rc = mhi_xprt_queue_in_buffers(mhi_xprtp, mhi_xprtp->ch_hndl.num_trbs);
	if (rc > 0)
		goto out_enable_event;
>>>>>>> p9x

	IPC_RTR_ERR("%s: Could not queue one TRB atleast\n", __func__);
	mutex_lock(&mhi_xprtp->ch_hndl.state_lock);
	mhi_xprtp->ch_hndl.in_chan_enabled = false;
	mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
	if (notify)
		msm_ipc_router_xprt_notify(&mhi_xprtp->xprt,
				   IPC_ROUTER_XPRT_EVENT_CLOSE, NULL);
<<<<<<< HEAD
=======
	mhi_close_channel(mhi_xprtp->ch_hndl.in_handle);
out_enable_event:
	kfree(xprt_work);
>>>>>>> p9x
}

/**
 * mhi_xprt_disable_event() - Disable the MHI link for communication
 * @work: Work containing some reference to the link to be disabled.
 *
 * This work is scheduled when the MHI link to the peripheral is down.
 */
<<<<<<< HEAD
static void mhi_xprt_disable_event(struct ipc_router_mhi_xprt_work *xprt_work)
{
=======
static void mhi_xprt_disable_event(struct work_struct *work)
{
	struct ipc_router_mhi_xprt_work *xprt_work =
		container_of(work, struct ipc_router_mhi_xprt_work, work);
>>>>>>> p9x
	struct ipc_router_mhi_xprt *mhi_xprtp = xprt_work->mhi_xprtp;
	bool notify = false;

	if (xprt_work->chan_id == mhi_xprtp->ch_hndl.out_chan_id) {
		mutex_lock(&mhi_xprtp->ch_hndl.state_lock);
		notify = mhi_xprtp->ch_hndl.out_chan_enabled &&
				mhi_xprtp->ch_hndl.in_chan_enabled;
		mhi_xprtp->ch_hndl.out_chan_enabled = false;
		mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
		wake_up(&mhi_xprtp->write_wait_q);
<<<<<<< HEAD
=======
		mhi_close_channel(mhi_xprtp->ch_hndl.out_handle);
>>>>>>> p9x
	} else if (xprt_work->chan_id == mhi_xprtp->ch_hndl.in_chan_id) {
		mutex_lock(&mhi_xprtp->ch_hndl.state_lock);
		notify = mhi_xprtp->ch_hndl.out_chan_enabled &&
				mhi_xprtp->ch_hndl.in_chan_enabled;
		mhi_xprtp->ch_hndl.in_chan_enabled = false;
		mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
		/* Queue a read work to remove any partially read packets */
		queue_work(mhi_xprtp->wq, &mhi_xprtp->read_work);
		flush_workqueue(mhi_xprtp->wq);
<<<<<<< HEAD
=======
		mhi_close_channel(mhi_xprtp->ch_hndl.in_handle);
>>>>>>> p9x
	}

	if (notify) {
		init_completion(&mhi_xprtp->sft_close_complete);
		msm_ipc_router_xprt_notify(&mhi_xprtp->xprt,
				   IPC_ROUTER_XPRT_EVENT_CLOSE, NULL);
		D("%s: Notified IPC Router of %s CLOSE\n",
		  __func__, mhi_xprtp->xprt.name);
		wait_for_completion(&mhi_xprtp->sft_close_complete);
	}
<<<<<<< HEAD
=======
	kfree(xprt_work);
>>>>>>> p9x
}

/**
 * mhi_xprt_xfer_event() - Function to handle MHI XFER Callbacks
 * @cb_info: Information containing xfer callback details.
 *
 * This function is called when the MHI generates a XFER event to the
 * IPC Router. This function is used to handle events like tx/rx.
 */
static void mhi_xprt_xfer_event(struct mhi_cb_info *cb_info)
{
	struct ipc_router_mhi_xprt *mhi_xprtp;
<<<<<<< HEAD
	void *out_addr;

	mhi_xprtp = (struct ipc_router_mhi_xprt *)(cb_info->result->user_data);
	if (cb_info->chan == mhi_xprtp->ch_hndl.out_chan_id) {
		out_addr = cb_info->result->buf_addr;
		ipc_router_mhi_xprt_find_addr_map(
					&mhi_xprtp->tx_addr_map_list,
					&mhi_xprtp->tx_addr_map_list_lock,
					out_addr);
		wake_up(&mhi_xprtp->write_wait_q);
=======

	mhi_xprtp = (struct ipc_router_mhi_xprt *)(cb_info->result->user_data);
	if (cb_info->chan == mhi_xprtp->ch_hndl.out_chan_id) {
		dma_unmap_single(NULL, (dma_addr_t)cb_info->result->payload_buf,
				cb_info->result->bytes_xferd, DMA_TO_DEVICE);
		mutex_lock(&mhi_xprtp->ch_hndl.state_lock);
		mhi_xprtp->ch_hndl.bytes_to_tx -= cb_info->result->bytes_xferd;
		if (!mhi_xprtp->ch_hndl.bytes_to_tx)
			wake_up(&mhi_xprtp->write_wait_q);
		mutex_unlock(&mhi_xprtp->ch_hndl.state_lock);
>>>>>>> p9x
	} else if (cb_info->chan == mhi_xprtp->ch_hndl.in_chan_id) {
		queue_work(mhi_xprtp->wq, &mhi_xprtp->read_work);
	} else {
		IPC_RTR_ERR("%s: chan_id %d not part of %s\n",
			    __func__, cb_info->chan, mhi_xprtp->xprt_name);
	}
}

/**
 * ipc_router_mhi_xprt_cb() - Callback to notify events on a channel
 * @cb_info: Information containing the details of callback.
 *
 * This function is called by the MHI driver to notify different events
 * like successful tx/rx, SSR events etc.
 */
static void ipc_router_mhi_xprt_cb(struct mhi_cb_info *cb_info)
{
	struct ipc_router_mhi_xprt *mhi_xprtp;
<<<<<<< HEAD
	struct ipc_router_mhi_xprt_work xprt_work;
=======
	struct ipc_router_mhi_xprt_work *xprt_work;
>>>>>>> p9x

	if (cb_info->result == NULL) {
		IPC_RTR_ERR("%s: Result not available in cb_info\n", __func__);
		return;
	}

	mhi_xprtp = (struct ipc_router_mhi_xprt *)(cb_info->result->user_data);
<<<<<<< HEAD
	xprt_work.mhi_xprtp = mhi_xprtp;
	xprt_work.chan_id = cb_info->chan;
	switch (cb_info->cb_reason) {
	case MHI_CB_MHI_SHUTDOWN:
	case MHI_CB_SYS_ERROR:
	case MHI_CB_MHI_DISABLED:
		mhi_xprt_disable_event(&xprt_work);
		break;
	case MHI_CB_MHI_ENABLED:
		mhi_xprt_enable_event(&xprt_work);
=======
	switch (cb_info->cb_reason) {
	case MHI_CB_MHI_ENABLED:
	case MHI_CB_MHI_DISABLED:
		xprt_work = kmalloc(sizeof(*xprt_work), GFP_KERNEL);
		if (!xprt_work) {
			IPC_RTR_ERR("%s: Couldn't handle %d event on %s\n",
				__func__, cb_info->cb_reason,
				mhi_xprtp->xprt_name);
			return;
		}
		xprt_work->mhi_xprtp = mhi_xprtp;
		xprt_work->chan_id = cb_info->chan;
		if (cb_info->cb_reason == MHI_CB_MHI_ENABLED)
			INIT_WORK(&xprt_work->work, mhi_xprt_enable_event);
		else
			INIT_WORK(&xprt_work->work, mhi_xprt_disable_event);
		queue_work(mhi_xprtp->wq, &xprt_work->work);
>>>>>>> p9x
		break;
	case MHI_CB_XFER:
		mhi_xprt_xfer_event(cb_info);
		break;
	default:
		IPC_RTR_ERR("%s: Invalid cb reason %x\n",
			    __func__, cb_info->cb_reason);
	}
}

/**
 * ipc_router_mhi_driver_register() - register for MHI channels
 *
 * @mhi_xprtp: pointer to IPC router mhi xprt structure.
 *
 * @return: 0 on success, standard Linux error codes on error.
 *
 * This function is called when a new XPRT is added.
 */
static int ipc_router_mhi_driver_register(
<<<<<<< HEAD
		struct ipc_router_mhi_xprt *mhi_xprtp, struct device *dev)
{
	int rc;
	const char *node_name = "qcom,mhi";
	struct mhi_client_info_t *mhi_info;

	if (!mhi_is_device_ready(dev, node_name))
		return -EPROBE_DEFER;

	mhi_info = &mhi_xprtp->ch_hndl.out_clnt_info;
	mhi_info->chan = mhi_xprtp->ch_hndl.out_chan_id;
	mhi_info->dev = dev;
	mhi_info->node_name = node_name;
	mhi_info->user_data = mhi_xprtp;
	rc = mhi_register_channel(&mhi_xprtp->ch_hndl.out_handle, mhi_info);
	if (rc) {
		IPC_RTR_ERR("%s: Error %d registering out_chan for %s\n",
			    __func__, rc, mhi_xprtp->xprt_name);
		return -EFAULT;
	}

	mhi_info = &mhi_xprtp->ch_hndl.in_clnt_info;
	mhi_info->chan = mhi_xprtp->ch_hndl.in_chan_id;
	mhi_info->dev = dev;
	mhi_info->node_name = node_name;
	mhi_info->user_data = mhi_xprtp;
	rc = mhi_register_channel(&mhi_xprtp->ch_hndl.in_handle, mhi_info);
	if (rc) {
		mhi_deregister_channel(mhi_xprtp->ch_hndl.out_handle);
		IPC_RTR_ERR("%s: Error %d registering in_chan for %s\n",
			    __func__, rc, mhi_xprtp->xprt_name);
=======
		struct ipc_router_mhi_xprt *mhi_xprtp)
{
	enum MHI_STATUS rc_status;

	rc_status = mhi_register_channel(&mhi_xprtp->ch_hndl.out_handle,
				mhi_xprtp->ch_hndl.out_chan_id, 0,
				&mhi_xprtp->ch_hndl.out_clnt_info,
				(void *)mhi_xprtp);
	if (rc_status != MHI_STATUS_SUCCESS) {
		IPC_RTR_ERR("%s: Error %d registering out_chan for %s\n",
			    __func__, rc_status, mhi_xprtp->xprt_name);
		return -EFAULT;
	}

	rc_status = mhi_register_channel(&mhi_xprtp->ch_hndl.in_handle,
				mhi_xprtp->ch_hndl.in_chan_id, 0,
				&mhi_xprtp->ch_hndl.in_clnt_info,
				(void *)mhi_xprtp);
	if (rc_status != MHI_STATUS_SUCCESS) {
		mhi_deregister_channel(mhi_xprtp->ch_hndl.out_handle);
		IPC_RTR_ERR("%s: Error %d registering in_chan for %s\n",
			    __func__, rc_status, mhi_xprtp->xprt_name);
>>>>>>> p9x
		return -EFAULT;
	}
	return 0;
}

/**
 * ipc_router_mhi_config_init() - init MHI xprt configs
 *
 * @mhi_xprt_config: pointer to MHI xprt configurations.
 *
 * @return: 0 on success, standard Linux error codes on error.
 *
 * This function is called to initialize the MHI XPRT pointer with
 * the MHI XPRT configurations from device tree.
 */
static int ipc_router_mhi_config_init(
<<<<<<< HEAD
			struct ipc_router_mhi_xprt_config *mhi_xprt_config,
			struct device *dev)
=======
	struct ipc_router_mhi_xprt_config *mhi_xprt_config)
>>>>>>> p9x
{
	struct ipc_router_mhi_xprt *mhi_xprtp;
	char wq_name[XPRT_NAME_LEN];
	int rc;

	mhi_xprtp = kzalloc(sizeof(struct ipc_router_mhi_xprt), GFP_KERNEL);
	if (IS_ERR_OR_NULL(mhi_xprtp)) {
		IPC_RTR_ERR("%s: kzalloc() failed for mhi_xprtp:%s\n",
			__func__, mhi_xprt_config->xprt_name);
		return -ENOMEM;
	}

	scnprintf(wq_name, XPRT_NAME_LEN, "MHI_XPRT%x:%x",
		  mhi_xprt_config->out_chan_id, mhi_xprt_config->in_chan_id);
	mhi_xprtp->wq = create_singlethread_workqueue(wq_name);
	if (!mhi_xprtp->wq) {
		IPC_RTR_ERR("%s: %s create WQ failed\n",
			__func__, mhi_xprt_config->xprt_name);
		kfree(mhi_xprtp);
		return -EFAULT;
	}

	INIT_WORK(&mhi_xprtp->read_work, mhi_xprt_read_data);
	init_waitqueue_head(&mhi_xprtp->write_wait_q);
	mhi_xprtp->xprt_version = mhi_xprt_config->xprt_version;
	strlcpy(mhi_xprtp->xprt_name, mhi_xprt_config->xprt_name,
		XPRT_NAME_LEN);

	/* Initialize XPRT operations and parameters registered with IPC RTR */
	mhi_xprtp->xprt.link_id = mhi_xprt_config->link_id;
	mhi_xprtp->xprt.name = mhi_xprtp->xprt_name;
	mhi_xprtp->xprt.get_version = ipc_router_mhi_get_xprt_version;
<<<<<<< HEAD
	mhi_xprtp->xprt.set_version = ipc_router_mhi_set_xprt_version;
=======
>>>>>>> p9x
	mhi_xprtp->xprt.get_option = ipc_router_mhi_get_xprt_option;
	mhi_xprtp->xprt.read_avail = NULL;
	mhi_xprtp->xprt.read = NULL;
	mhi_xprtp->xprt.write_avail = ipc_router_mhi_write_avail;
	mhi_xprtp->xprt.write = ipc_router_mhi_write;
	mhi_xprtp->xprt.close = ipc_router_mhi_close;
	mhi_xprtp->xprt.sft_close_done = mhi_xprt_sft_close_done;
	mhi_xprtp->xprt.priv = NULL;

	/* Initialize channel handle parameters */
	mhi_xprtp->ch_hndl.out_chan_id = mhi_xprt_config->out_chan_id;
	mhi_xprtp->ch_hndl.in_chan_id = mhi_xprt_config->in_chan_id;
	mhi_xprtp->ch_hndl.out_clnt_info.mhi_client_cb = ipc_router_mhi_xprt_cb;
	mhi_xprtp->ch_hndl.in_clnt_info.mhi_client_cb = ipc_router_mhi_xprt_cb;
	mutex_init(&mhi_xprtp->ch_hndl.state_lock);
	mutex_init(&mhi_xprtp->ch_hndl.in_skbq_lock);
	skb_queue_head_init(&mhi_xprtp->ch_hndl.in_skbq);
	mhi_xprtp->ch_hndl.max_packet_size = IPC_ROUTER_MHI_XPRT_MAX_PKT_SIZE;
	mhi_xprtp->ch_hndl.num_trbs = IPC_ROUTER_MHI_XPRT_NUM_TRBS;
	mhi_xprtp->ch_hndl.mhi_xprtp = mhi_xprtp;
<<<<<<< HEAD
	INIT_LIST_HEAD(&mhi_xprtp->tx_addr_map_list);
	spin_lock_init(&mhi_xprtp->tx_addr_map_list_lock);
	INIT_LIST_HEAD(&mhi_xprtp->rx_addr_map_list);
	spin_lock_init(&mhi_xprtp->rx_addr_map_list_lock);

	rc = ipc_router_mhi_driver_register(mhi_xprtp, dev);
=======

	rc = ipc_router_mhi_driver_register(mhi_xprtp);
>>>>>>> p9x
	return rc;
}

/**
 * parse_devicetree() - parse device tree binding
 *
 * @node: pointer to device tree node
 * @mhi_xprt_config: pointer to MHI XPRT configurations
 *
 * @return: 0 on success, -ENODEV on failure.
 */
static int parse_devicetree(struct device_node *node,
		struct ipc_router_mhi_xprt_config *mhi_xprt_config)
{
	int rc;
	uint32_t out_chan_id;
	uint32_t in_chan_id;
	const char *remote_ss;
	uint32_t link_id;
	uint32_t version;
	char *key;

	key = "qcom,out-chan-id";
	rc = of_property_read_u32(node, key, &out_chan_id);
	if (rc)
		goto error;
	mhi_xprt_config->out_chan_id = out_chan_id;

	key = "qcom,in-chan-id";
	rc = of_property_read_u32(node, key, &in_chan_id);
	if (rc)
		goto error;
	mhi_xprt_config->in_chan_id = in_chan_id;

	key = "qcom,xprt-remote";
	remote_ss = of_get_property(node, key, NULL);
	if (!remote_ss)
		goto error;

	key = "qcom,xprt-linkid";
	rc = of_property_read_u32(node, key, &link_id);
	if (rc)
		goto error;
	mhi_xprt_config->link_id = link_id;

	key = "qcom,xprt-version";
	rc = of_property_read_u32(node, key, &version);
	if (rc)
		goto error;
	mhi_xprt_config->xprt_version = version;

	scnprintf(mhi_xprt_config->xprt_name, XPRT_NAME_LEN,
		  "IPCRTR_MHI%x:%x_%s",
		  out_chan_id, in_chan_id, remote_ss);

	return 0;
error:
	IPC_RTR_ERR("%s: missing key: %s\n", __func__, key);
	return -ENODEV;
}

/**
 * ipc_router_mhi_xprt_probe() - Probe an MHI xprt
 * @pdev: Platform device corresponding to MHI xprt.
 *
 * @return: 0 on success, standard Linux error codes on error.
 *
 * This function is called when the underlying device tree driver registers
 * a platform device, mapped to an MHI transport.
 */
static int ipc_router_mhi_xprt_probe(struct platform_device *pdev)
{
	int rc;
	struct ipc_router_mhi_xprt_config mhi_xprt_config;

	if (pdev && pdev->dev.of_node) {
		rc = parse_devicetree(pdev->dev.of_node, &mhi_xprt_config);
		if (rc) {
			IPC_RTR_ERR("%s: failed to parse device tree\n",
				    __func__);
			return rc;
		}

<<<<<<< HEAD
		rc = ipc_router_mhi_config_init(&mhi_xprt_config, &pdev->dev);
=======
		rc = ipc_router_mhi_config_init(&mhi_xprt_config);
>>>>>>> p9x
		if (rc) {
			IPC_RTR_ERR("%s: init failed\n", __func__);
			return rc;
		}
	}
	return rc;
}

static struct of_device_id ipc_router_mhi_xprt_match_table[] = {
	{ .compatible = "qcom,ipc_router_mhi_xprt" },
	{},
};

static struct platform_driver ipc_router_mhi_xprt_driver = {
	.probe = ipc_router_mhi_xprt_probe,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ipc_router_mhi_xprt_match_table,
	},
};

static int __init ipc_router_mhi_xprt_init(void)
{
	int rc;

	rc = platform_driver_register(&ipc_router_mhi_xprt_driver);
	if (rc) {
		IPC_RTR_ERR("%s: ipc_router_mhi_xprt_driver reg. failed %d\n",
			__func__, rc);
		return rc;
	}
	return 0;
}

module_init(ipc_router_mhi_xprt_init);
MODULE_DESCRIPTION("IPC Router MHI XPRT");
MODULE_LICENSE("GPL v2");
