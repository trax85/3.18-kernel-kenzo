/*
<<<<<<< HEAD
 * Copyright (c) 2012-2018, The Linux Foundation. All rights reserved.
=======
 * Copyright (c) 2012-2015,2017 The Linux Foundation. All rights reserved.
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
=======

>>>>>>> p9x
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/pagemap.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/hash.h>
#include <linux/msm_ion.h>
<<<<<<< HEAD
#include <soc/qcom/secure_buffer.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/glink.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/subsystem_restart.h>
=======
#include <soc/qcom/smd.h>
#include <soc/qcom/subsystem_notif.h>
#include <linux/msm_iommu_domains.h>
>>>>>>> p9x
#include <linux/scatterlist.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/dma-contiguous.h>
<<<<<<< HEAD
#include <linux/cma.h>
#include <linux/iommu.h>
#include <linux/qcom_iommu.h>
#include <linux/kref.h>
#include <linux/sort.h>
#include <linux/msm_dma_iommu_mapping.h>
#include <asm/dma-iommu.h>
#include <soc/qcom/scm.h>
#include "adsprpc_compat.h"
#include "adsprpc_shared.h"
#include <soc/qcom/ramdump.h>
#include <linux/workqueue.h>
#include <linux/wait.h>

=======
#include <linux/iommu.h>
#include <linux/kref.h>
#include <linux/sort.h>
#include <asm/dma-iommu.h>
#include "adsprpc_compat.h"
#include "adsprpc_shared.h"
#include <linux/msm_audio_ion.h>
#include <soc/qcom/scm.h>
>>>>>>> p9x

#define TZ_PIL_PROTECT_MEM_SUBSYS_ID 0x0C
#define TZ_PIL_CLEAR_PROTECT_MEM_SUBSYS_ID 0x0D
#define TZ_PIL_AUTH_QDSP6_PROC 1
#define ADSP_MMAP_HEAP_ADDR 4
<<<<<<< HEAD
#define ADSP_MMAP_ADD_PAGES 0x1000

#define FASTRPC_ENOSUCH 39
#define VMID_SSC_Q6     5
#define VMID_ADSP_Q6    6

#define RPC_TIMEOUT	(5 * HZ)
#define OPEN_TIMEOUT    (0.5 * HZ)
#define BALIGN		128
#define NUM_CHANNELS	3		/*1 compute 1 cpz 1 mdsp*/
#define NUM_SESSIONS	8		/*8 compute*/
#define FASTRPC_CTX_MAGIC (0xbeeddeed)
#define FASTRPC_CTX_MAX (256)
#define FASTRPC_CTXID_MASK (0xFF0)

#define IS_CACHE_ALIGNED(x) (((x) & ((L1_CACHE_BYTES)-1)) == 0)

static void file_free_work_handler(struct work_struct *w);

static DECLARE_WAIT_QUEUE_HEAD(wait_queue);

=======
#define AUDIO_ADSP_STREAM_ID	1
#define STREAM_ID	((uint64_t)AUDIO_ADSP_STREAM_ID << 32)
#define RPC_TIMEOUT	(5 * HZ)
#define BALIGN		32
#define NUM_CHANNELS    1 /*8 compute 2 cpz 1 modem*/
#define FASTRPC_CTX_MAGIC (0xbeeddeed)

#define IS_CACHE_ALIGNED(x) (((x) & ((L1_CACHE_BYTES)-1)) == 0)

>>>>>>> p9x
static inline uint64_t buf_page_start(uint64_t buf)
{
	uint64_t start = (uint64_t) buf & PAGE_MASK;
	return start;
}

static inline uint64_t buf_page_offset(uint64_t buf)
{
	uint64_t offset = (uint64_t) buf & (PAGE_SIZE - 1);
	return offset;
}

static inline uint64_t buf_num_pages(uint64_t buf, size_t len)
{
	uint64_t start = buf_page_start(buf) >> PAGE_SHIFT;
	uint64_t end = (((uint64_t) buf + len - 1) & PAGE_MASK) >> PAGE_SHIFT;
	uint64_t nPages = end - start + 1;
	return nPages;
}

static inline uint64_t buf_page_size(uint32_t size)
{
	uint64_t sz = (size + (PAGE_SIZE - 1)) & PAGE_MASK;
<<<<<<< HEAD

=======
>>>>>>> p9x
	return sz > PAGE_SIZE ? sz : PAGE_SIZE;
}

static inline void *uint64_to_ptr(uint64_t addr)
{
	void *ptr = (void *)((uintptr_t)addr);
	return ptr;
}

static inline uint64_t ptr_to_uint64(void *ptr)
{
	uint64_t addr = (uint64_t)((uintptr_t)ptr);
	return addr;
}

struct fastrpc_file;

<<<<<<< HEAD
struct fastrpc_fl {
	struct fastrpc_file *fl;
	struct hlist_node hn;
};

struct fastrpc_buf {
	struct hlist_node hn;
	struct hlist_node hn_rem;
	struct fastrpc_file *fl;
	void *virt;
	uint64_t phys;
	size_t size;
	struct dma_attrs attrs;
	uintptr_t raddr;
	uint32_t flags;
	int remote;
=======
struct fastrpc_buf {
	struct hlist_node hn;
	struct fastrpc_file *fl;
	void *virt;
	dma_addr_t phys;
	size_t size;
	struct ion_handle *handle;
	struct ion_client *client;
>>>>>>> p9x
};

struct fastrpc_ctx_lst;

struct overlap {
	uintptr_t start;
	uintptr_t end;
	int raix;
	uintptr_t mstart;
	uintptr_t mend;
	uintptr_t offset;
};

struct smq_invoke_ctx {
	struct hlist_node hn;
	struct completion work;
	int retval;
	int pid;
	int tgid;
	remote_arg_t *lpra;
	remote_arg64_t *rpra;
	int *fds;
	struct fastrpc_mmap **maps;
	struct fastrpc_buf *buf;
	size_t used;
	struct fastrpc_file *fl;
	uint32_t sc;
	struct overlap *overs;
	struct overlap **overps;
<<<<<<< HEAD
	struct smq_msg msg;
	unsigned int magic;
	uint64_t ctxid;
=======
	unsigned int magic;
>>>>>>> p9x
};

struct fastrpc_ctx_lst {
	struct hlist_head pending;
	struct hlist_head interrupted;
};

struct fastrpc_smmu {
<<<<<<< HEAD
	struct dma_iommu_mapping *mapping;
	int cb;
	int enabled;
	int faults;
};

struct fastrpc_session_ctx {
	struct device *dev;
	struct fastrpc_smmu smmu;
};

struct fastrpc_channel_ctx {
	char *name;
	char *subsys;
	void *chan;
	struct device *dev;
	struct fastrpc_session_ctx session[NUM_SESSIONS];
	struct completion work;
	struct notifier_block nb;
	struct kref kref;
	unsigned long bitmap;
	int channel;
	int sesscount;
	int ssrcount;
	void *handle;
	int prevssrcount;
	int vmid;
	int ramdumpenabled;
	void *remoteheap_ramdump_dev;
	struct glink_link_info link_info;
	void *link_notify_handle;
	struct glink_open_config cfg;
	char *edge;
};

struct fastrpc_apps {
	struct fastrpc_channel_ctx *channel;
=======
	struct device *dev;
	struct dma_iommu_mapping *mapping;
	int cb;
	int enabled;
};

struct fastrpc_chan_ctx {
	smd_channel_t *chan;
	struct device *dev;
	struct completion work;
	struct fastrpc_smmu smmu;
	struct kref kref;
	struct notifier_block nb;
	int ssrcount;
};

struct fastrpc_apps {
	struct fastrpc_chan_ctx channel[NUM_CHANNELS];
	int nchans;
>>>>>>> p9x
	struct cdev cdev;
	struct class *class;
	struct mutex smd_mutex;
	struct smq_phy_page range;
	struct hlist_head maps;
<<<<<<< HEAD
	struct hlist_head fls;
	struct workqueue_struct *wq;
	int pending_free;
	struct work_struct free_work;
	struct mutex flfree_mutex;
=======
>>>>>>> p9x
	dev_t dev_no;
	int compat;
	struct hlist_head drivers;
	spinlock_t hlock;
<<<<<<< HEAD
	struct ion_client *client;
	struct device *dev;
	struct device *modem_cma_dev;
	bool glink;
	spinlock_t ctxlock;
	struct smq_invoke_ctx *ctxtable[FASTRPC_CTX_MAX];
=======
	int32_t domain_id;
	struct device *adsp_mem_device;
>>>>>>> p9x
};

struct fastrpc_mmap {
	struct hlist_node hn;
	struct fastrpc_file *fl;
	struct fastrpc_apps *apps;
	int fd;
	uint32_t flags;
	struct dma_buf *buf;
	struct sg_table *table;
	struct dma_buf_attachment *attach;
<<<<<<< HEAD
	struct ion_handle *handle;
	uint64_t phys;
=======
	uintptr_t phys;
>>>>>>> p9x
	size_t size;
	uintptr_t va;
	size_t len;
	int refs;
	uintptr_t raddr;
<<<<<<< HEAD
	int uncached;
=======
	struct ion_handle *handle;
	struct ion_client *client;
};

struct fastrpc_channel_info {
	char *name;
	char *node;
	char *group;
	char *subsys;
	int channel;
>>>>>>> p9x
};

struct fastrpc_file {
	struct hlist_node hn;
	spinlock_t hlock;
	struct hlist_head maps;
<<<<<<< HEAD
	struct hlist_head cached_bufs;
	struct hlist_head remote_bufs;
	struct fastrpc_ctx_lst clst;
	struct fastrpc_session_ctx *sctx;
	struct fastrpc_buf *init_mem;
=======
	struct hlist_head bufs;
	struct fastrpc_ctx_lst clst;
	struct fastrpc_chan_ctx *chan;
>>>>>>> p9x
	uint32_t mode;
	int tgid;
	int cid;
	int ssrcount;
	struct fastrpc_apps *apps;
<<<<<<< HEAD
	struct mutex map_mutex;
=======
>>>>>>> p9x
};

static struct fastrpc_apps gfa;

<<<<<<< HEAD
static struct fastrpc_channel_ctx gcinfo[NUM_CHANNELS] = {
	{
		.name = "adsprpc-smd",
		.subsys = "adsp",
		.channel = SMD_APPS_QDSP,
		.edge = "lpass",
	},
	{
		.name = "sdsprpc-smd",
		.subsys = "dsps",
		.channel = SMD_APPS_DSPS,
		.edge = "dsps",
	},
	{
		.name = "mdsprpc-smd",
		.subsys = "modem",
		.channel = SMD_APPS_MODEM,
		.edge = "mdsp",
=======
static const struct fastrpc_channel_info gcinfo[NUM_CHANNELS] = {
	{
		.name = "adsprpc-smd",
		.node = "qcom,msm-audio-ion",
		.subsys = "adsp",
		.channel = SMD_APPS_QDSP,
>>>>>>> p9x
	},
};

static void fastrpc_buf_free(struct fastrpc_buf *buf, int cache)
{
<<<<<<< HEAD
	struct fastrpc_file *fl = buf == NULL ? NULL : buf->fl;
	int vmid;

=======
	int err = 0;
	struct fastrpc_file *fl = buf == NULL ? NULL : buf->fl;
>>>>>>> p9x
	if (!fl)
		return;
	if (cache) {
		spin_lock(&fl->hlock);
<<<<<<< HEAD
		hlist_add_head(&buf->hn, &fl->cached_bufs);
		spin_unlock(&fl->hlock);
		return;
	}
	if (buf->remote) {
		spin_lock(&fl->hlock);
		hlist_del_init(&buf->hn_rem);
		spin_unlock(&fl->hlock);
		buf->remote = 0;
		buf->raddr = 0;
	}
	if (!IS_ERR_OR_NULL(buf->virt)) {
		int destVM[1] = {VMID_HLOS};
		int destVMperm[1] = {PERM_READ | PERM_WRITE | PERM_EXEC};

		if (fl->sctx->smmu.cb)
			buf->phys &= ~((uint64_t)fl->sctx->smmu.cb << 32);
		vmid = fl->apps->channel[fl->cid].vmid;
		if (vmid) {
			int srcVM[2] = {VMID_HLOS, vmid};

			hyp_assign_phys(buf->phys, buf_page_size(buf->size),
				srcVM, 2, destVM, destVMperm, 1);
		}
		dma_free_attrs(fl->sctx->dev, buf->size, buf->virt,
			buf->phys, (struct dma_attrs *)&buf->attrs);
	}
	kfree(buf);
}

static void fastrpc_cached_buf_list_free(struct fastrpc_file *fl)
{
	struct fastrpc_buf *buf, *free;

=======
		hlist_add_head(&buf->hn, &fl->bufs);
		spin_unlock(&fl->hlock);
		return;
	}
	if (!IS_ERR_OR_NULL(buf->virt)) {
		VERIFY(err, !msm_audio_ion_free(buf->client, buf->handle));
		if (err) {
			pr_err("error freeing audio ion buffer\n");
			goto bail;
		}
	}
bail:
	kfree(buf);
}

static void fastrpc_buf_list_free(struct fastrpc_file *fl)
{
	struct fastrpc_buf *buf, *free;
>>>>>>> p9x
	do {
		struct hlist_node *n;

		free = NULL;
		spin_lock(&fl->hlock);
<<<<<<< HEAD
		hlist_for_each_entry_safe(buf, n, &fl->cached_bufs, hn) {
=======
		hlist_for_each_entry_safe(buf, n, &fl->bufs, hn) {
>>>>>>> p9x
			hlist_del_init(&buf->hn);
			free = buf;
			break;
		}
		spin_unlock(&fl->hlock);
		if (free)
			fastrpc_buf_free(free, 0);
	} while (free);
}

<<<<<<< HEAD
static void fastrpc_remote_buf_list_free(struct fastrpc_file *fl)
{
	struct fastrpc_buf *buf, *free;

	do {
		struct hlist_node *n;

		free = NULL;
		spin_lock(&fl->hlock);
		hlist_for_each_entry_safe(buf, n, &fl->remote_bufs, hn_rem) {
			free = buf;
			break;
		}
		spin_unlock(&fl->hlock);
		if (free)
			fastrpc_buf_free(free, 0);
	} while (free);
}

=======
>>>>>>> p9x
static void fastrpc_mmap_add(struct fastrpc_mmap *map)
{
	if (map->flags == ADSP_MMAP_HEAP_ADDR) {
		struct fastrpc_apps *me = &gfa;
<<<<<<< HEAD

=======
>>>>>>> p9x
		spin_lock(&me->hlock);
		hlist_add_head(&map->hn, &me->maps);
		spin_unlock(&me->hlock);
	} else {
		struct fastrpc_file *fl = map->fl;
<<<<<<< HEAD

=======
>>>>>>> p9x
		spin_lock(&fl->hlock);
		hlist_add_head(&map->hn, &fl->maps);
		spin_unlock(&fl->hlock);
	}
}

<<<<<<< HEAD
static int fastrpc_mmap_find(struct fastrpc_file *fl, int fd, uintptr_t va,
			size_t len, int mflags, struct fastrpc_mmap **ppmap)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_mmap *match = NULL, *map = NULL;
	struct hlist_node *n;
	if ((va + len) < va)
		return -EOVERFLOW;
	if (mflags == ADSP_MMAP_HEAP_ADDR) {
		spin_lock(&me->hlock);
		hlist_for_each_entry_safe(map, n, &me->maps, hn) {
			if (va >= map->va &&
				va + len <= map->va + map->len &&
				map->fd == fd) {
				map->refs++;
				match = map;
				break;
			}
		}
		spin_unlock(&me->hlock);
	} else {
		spin_lock(&fl->hlock);
		hlist_for_each_entry_safe(map, n, &fl->maps, hn) {
			if (va >= map->va &&
				va + len <= map->va + map->len &&
				map->fd == fd) {
				map->refs++;
				match = map;
				break;
			}
		}
		spin_unlock(&fl->hlock);
	}
	if (match) {
		*ppmap = match;
		return 0;
	}
	return -ENOTTY;
}

static int dma_alloc_memory(dma_addr_t *region_start, void **vaddr, size_t size,
			struct dma_attrs *attrs)
{
	struct fastrpc_apps *me = &gfa;

	if (me->dev == NULL) {
		pr_err("device adsprpc-mem is not initialized\n");
		return -ENODEV;
	}

	*vaddr = dma_alloc_attrs(me->dev, size, region_start,
				 GFP_KERNEL, attrs);
	if (IS_ERR_OR_NULL(*vaddr)) {
		pr_err("adsprpc: %s: %s: dma_alloc_attrs failed for size 0x%zx, returned %pK\n",
				current->comm, __func__, size, (*vaddr));
		return -ENOMEM;
	}
	return 0;
}

=======
>>>>>>> p9x
static int fastrpc_mmap_remove(struct fastrpc_file *fl, uintptr_t va,
			       size_t len, struct fastrpc_mmap **ppmap)
{
	struct fastrpc_mmap *match = NULL, *map = NULL;
<<<<<<< HEAD
	struct hlist_node *n;
	struct fastrpc_apps *me = &gfa;

	spin_lock(&me->hlock);
	hlist_for_each_entry_safe(map, n, &me->maps, hn) {
		if (map->raddr == va &&
			map->raddr + map->len == va + len &&
			map->refs == 1) {
=======
	struct hlist_node *n = NULL;
	struct fastrpc_apps *me = &gfa;
	spin_lock(&me->hlock);
	hlist_for_each_entry_safe(map, n, &me->maps, hn) {
		if (map->raddr == va &&
				map->raddr + map->len == va + len &&
				map->refs == 1) {
>>>>>>> p9x
			match = map;
			hlist_del_init(&map->hn);
			break;
		}
	}
	spin_unlock(&me->hlock);
	if (match) {
		*ppmap = match;
		return 0;
	}
	spin_lock(&fl->hlock);
	hlist_for_each_entry_safe(map, n, &fl->maps, hn) {
		if (map->raddr == va &&
<<<<<<< HEAD
			map->raddr + map->len == va + len &&
			map->refs == 1) {
=======
				map->raddr + map->len == va + len &&
				map->refs == 1) {
>>>>>>> p9x
			match = map;
			hlist_del_init(&map->hn);
			break;
		}
	}
	spin_unlock(&fl->hlock);
	if (match) {
		*ppmap = match;
		return 0;
	}
	return -ENOTTY;
}

static void fastrpc_mmap_free(struct fastrpc_mmap *map)
{
	struct fastrpc_apps *me = &gfa;
<<<<<<< HEAD
	struct fastrpc_file *fl;
	int vmid;

	if (!map)
		return;
	fl = map->fl;
=======
	struct fastrpc_file *fl = NULL;
	if (!map)
		return;
>>>>>>> p9x
	if (map->flags == ADSP_MMAP_HEAP_ADDR) {
		spin_lock(&me->hlock);
		map->refs--;
		if (!map->refs)
			hlist_del_init(&map->hn);
		spin_unlock(&me->hlock);
	} else {
<<<<<<< HEAD
=======
		fl = map->fl;
>>>>>>> p9x
		spin_lock(&fl->hlock);
		map->refs--;
		if (!map->refs)
			hlist_del_init(&map->hn);
		spin_unlock(&fl->hlock);
	}
<<<<<<< HEAD
	if (map->refs > 0)
=======
	if (map->refs)
>>>>>>> p9x
		return;
	if (map->flags == ADSP_MMAP_HEAP_ADDR) {
		DEFINE_DMA_ATTRS(attrs);

<<<<<<< HEAD
		if (me->dev == NULL) {
			pr_err("failed to free remote heap allocation\n");
			return;
		}
		if (map->phys) {
			dma_set_attr(DMA_ATTR_SKIP_ZEROING, &attrs);
			dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
			dma_free_attrs(me->dev, map->size,
					&(map->va), map->phys,	&attrs);
		}
	} else {
		int destVM[1] = {VMID_HLOS};
		int destVMperm[1] = {PERM_READ | PERM_WRITE | PERM_EXEC};

		if (!IS_ERR_OR_NULL(map->handle))
			ion_free(fl->apps->client, map->handle);
		if (fl->sctx->smmu.enabled) {
			if (map->size || map->phys)
				msm_dma_unmap_sg(fl->sctx->dev,
					map->table->sgl,
					map->table->nents, DMA_BIDIRECTIONAL,
					map->buf);
		}
		vmid = fl->apps->channel[fl->cid].vmid;
		if (vmid && map->phys) {
			int srcVM[2] = {VMID_HLOS, vmid};

			hyp_assign_phys(map->phys, buf_page_size(map->size),
				srcVM, 2, destVM, destVMperm, 1);
		}

		if (!IS_ERR_OR_NULL(map->table))
			dma_buf_unmap_attachment(map->attach, map->table,
					DMA_BIDIRECTIONAL);
		if (!IS_ERR_OR_NULL(map->attach))
			dma_buf_detach(map->buf, map->attach);
		if (!IS_ERR_OR_NULL(map->buf))
			dma_buf_put(map->buf);
=======
		dma_set_attr(DMA_ATTR_SKIP_ZEROING, &attrs);
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
		dma_free_attrs(me->adsp_mem_device, map->size,
				&(map->va), map->phys,	&attrs);
	} else {
		ion_unmap_iommu(map->client, map->handle,
				me->domain_id, 0);
		ion_unmap_kernel(map->client, map->handle);
		msm_audio_ion_client_destroy(map->client);
>>>>>>> p9x
	}
	kfree(map);
}

<<<<<<< HEAD
static int fastrpc_mmap_create(struct fastrpc_file *fl, int fd, uintptr_t va,
			size_t len, int mflags, struct fastrpc_mmap **ppmap)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_session_ctx *sess = fl->sctx;
	struct fastrpc_mmap *map = NULL;
	struct dma_attrs attrs;
	dma_addr_t region_start = 0;
	void *region_vaddr = NULL;
	unsigned long flags;
	int err = 0, vmid;

	if (!fastrpc_mmap_find(fl, fd, va, len, mflags, ppmap))
		return 0;
	map = kzalloc(sizeof(*map), GFP_KERNEL);
	VERIFY(err, !IS_ERR_OR_NULL(map));
	if (err)
		goto bail;
	map->flags = mflags;
	map->refs = 1;
	INIT_HLIST_NODE(&map->hn);
	map->fl = fl;
	map->fd = fd;
	if (mflags == ADSP_MMAP_HEAP_ADDR) {
		DEFINE_DMA_ATTRS(rh_attrs);

		dma_set_attr(DMA_ATTR_SKIP_ZEROING, &rh_attrs);
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &rh_attrs);

		map->apps = me;
		map->fl = NULL;
		VERIFY(err, !dma_alloc_memory(&region_start,
			 &region_vaddr, len, &rh_attrs));
		if (err)
			goto bail;
		map->phys = (uintptr_t)region_start;
		map->size = len;
		map->va = (uintptr_t)region_vaddr;
	} else {
		VERIFY(err, !IS_ERR_OR_NULL(map->buf = dma_buf_get(fd)));
		if (err)
			goto bail;
		VERIFY(err, !IS_ERR_OR_NULL(map->attach =
				dma_buf_attach(map->buf, sess->dev)));
		if (err)
			goto bail;
		VERIFY(err, !IS_ERR_OR_NULL(map->table =
				dma_buf_map_attachment(map->attach,
					DMA_BIDIRECTIONAL)));
		if (err)
			goto bail;
		if (sess->smmu.enabled) {
			init_dma_attrs(&attrs);
			dma_set_attr(DMA_ATTR_EXEC_MAPPING, &attrs);
			VERIFY(err, map->table->nents ==
					msm_dma_map_sg_attrs(sess->dev,
					map->table->sgl, map->table->nents,
					DMA_BIDIRECTIONAL, map->buf, &attrs));
			if (err)
				goto bail;
		} else {
			VERIFY(err, map->table->nents == 1);
			if (err)
				goto bail;
		}
		VERIFY(err, !IS_ERR_OR_NULL(map->handle =
				ion_import_dma_buf(fl->apps->client, fd)));
		if (err)
			goto bail;
		VERIFY(err, !ion_handle_get_flags(fl->apps->client,
							map->handle, &flags));
		if (err)
			goto bail;
		map->uncached = !ION_IS_CACHED(flags);
		map->phys = sg_dma_address(map->table->sgl);
		if (sess->smmu.cb) {
			map->phys += ((uint64_t)sess->smmu.cb << 32);
			map->size = sg_dma_len(map->table->sgl);
		} else {
			map->size = buf_page_size(len);
		}
		vmid = fl->apps->channel[fl->cid].vmid;
		if (vmid) {
			int srcVM[1] = {VMID_HLOS};
			int destVM[2] = {VMID_HLOS, vmid};
			int destVMperm[2] = {PERM_READ | PERM_WRITE,
						PERM_READ | PERM_WRITE };

			VERIFY(err, !hyp_assign_phys(map->phys,
					buf_page_size(map->size),
					srcVM, 1, destVM, destVMperm, 2));
			if (err)
				goto bail;
		}
		map->va = va;
	}
	map->len = len;

	fastrpc_mmap_add(map);
	*ppmap = map;

bail:
	if (err && map)
		fastrpc_mmap_free(map);
	return err;
}

static int fastrpc_buf_alloc(struct fastrpc_file *fl, size_t size,
				struct dma_attrs attr, uint32_t rflags,
				int remote, struct fastrpc_buf **obuf)
{
	int err = 0, vmid;
	struct fastrpc_buf *buf = NULL, *fr = NULL;
	struct hlist_node *n;
=======
static int fastrpc_buf_alloc(struct fastrpc_file *fl, size_t size,
			     struct fastrpc_buf **obuf)
{
	int err = 0;
	struct fastrpc_buf *buf = NULL, *fr = NULL;
	struct hlist_node *n = NULL;
	size_t len = 0;
>>>>>>> p9x

	VERIFY(err, size > 0);
	if (err)
		goto bail;

<<<<<<< HEAD
	if (!remote) {
		/* find the smallest buffer that fits in the cache */
		spin_lock(&fl->hlock);
		hlist_for_each_entry_safe(buf, n, &fl->cached_bufs, hn) {
			if (buf->size >= size && (!fr || fr->size > buf->size))
				fr = buf;
		}
		if (fr)
			hlist_del_init(&fr->hn);
		spin_unlock(&fl->hlock);
		if (fr) {
			*obuf = fr;
			return 0;
		}
	}
	buf = NULL;
=======
	/* find the smallest buffer that fits in the cache */
	spin_lock(&fl->hlock);
	hlist_for_each_entry_safe(buf, n, &fl->bufs, hn) {
		if (buf->size >= size && (!fr || fr->size > buf->size))
			fr = buf;
	}
	if (fr)
		hlist_del_init(&fr->hn);
	spin_unlock(&fl->hlock);
	if (fr) {
		*obuf = fr;
		return 0;
	}
>>>>>>> p9x
	VERIFY(err, NULL != (buf = kzalloc(sizeof(*buf), GFP_KERNEL)));
	if (err)
		goto bail;
	INIT_HLIST_NODE(&buf->hn);
<<<<<<< HEAD
=======

	buf->handle = NULL;
	buf->client = NULL;
>>>>>>> p9x
	buf->fl = fl;
	buf->virt = NULL;
	buf->phys = 0;
	buf->size = size;
<<<<<<< HEAD
	memcpy(&buf->attrs, &attr, sizeof(struct dma_attrs));
	buf->flags = rflags;
	buf->raddr = 0;
	buf->remote = 0;
	buf->virt = dma_alloc_attrs(fl->sctx->dev, buf->size,
					 (dma_addr_t *)&buf->phys, GFP_KERNEL,
					 (struct dma_attrs *)&buf->attrs);
	if (IS_ERR_OR_NULL(buf->virt)) {
		/* free cache and retry */
		fastrpc_cached_buf_list_free(fl);
		buf->virt = dma_alloc_attrs(fl->sctx->dev, buf->size,
					 (dma_addr_t *)&buf->phys, GFP_KERNEL,
					 (struct dma_attrs *)&buf->attrs);
		VERIFY(err, !IS_ERR_OR_NULL(buf->virt));
	}
	if (err) {
		err = -ENOMEM;
		pr_err("adsprpc: %s: %s: dma_alloc_attrs failed for size 0x%zx\n",
			current->comm, __func__, size);
		goto bail;
	}
	if (fl->sctx->smmu.cb)
		buf->phys += ((uint64_t)fl->sctx->smmu.cb << 32);
	vmid = fl->apps->channel[fl->cid].vmid;
	if (vmid) {
		int srcVM[1] = {VMID_HLOS};
		int destVM[2] = {VMID_HLOS, vmid};
		int destVMperm[2] = {PERM_READ | PERM_WRITE,
					PERM_READ | PERM_WRITE};

		VERIFY(err, !hyp_assign_phys(buf->phys, buf_page_size(size),
			srcVM, 1, destVM, destVMperm, 2));
		if (err)
			goto bail;
	}

	if (remote) {
		INIT_HLIST_NODE(&buf->hn_rem);
		spin_lock(&fl->hlock);
		hlist_add_head(&buf->hn_rem, &fl->remote_bufs);
		spin_unlock(&fl->hlock);
		buf->remote = remote;
	}
=======

	VERIFY(err, !msm_audio_ion_alloc(DEVICE_NAME, &(buf->client),
				&(buf->handle), buf->size,
				&(buf->phys), &len, &(buf->virt)));
	if (err) {
		pr_err("%s: Error allocating audio ion buf size: %zd\n",
				__func__, buf->size);
		goto bail;
	}
	if (len != buf->size)
		pr_info("allocating memory from audio ion size is %zd\n",
				len);

	buf->size = len;

>>>>>>> p9x
	*obuf = buf;
 bail:
	if (err && buf)
		fastrpc_buf_free(buf, 0);
	return err;
}

static int context_restore_interrupted(struct fastrpc_file *fl,
				       struct fastrpc_ioctl_invoke_fd *invokefd,
				       struct smq_invoke_ctx **po)
{
	int err = 0;
	struct smq_invoke_ctx *ctx = NULL, *ictx = NULL;
<<<<<<< HEAD
	struct hlist_node *n;
	struct fastrpc_ioctl_invoke *invoke = &invokefd->inv;

=======
	struct hlist_node *n = NULL;
	struct fastrpc_ioctl_invoke *invoke = &invokefd->inv;
>>>>>>> p9x
	spin_lock(&fl->hlock);
	hlist_for_each_entry_safe(ictx, n, &fl->clst.interrupted, hn) {
		if (ictx->pid == current->pid) {
			if (invoke->sc != ictx->sc || ictx->fl != fl)
				err = -1;
			else {
				ctx = ictx;
				hlist_del_init(&ctx->hn);
				hlist_add_head(&ctx->hn, &fl->clst.pending);
			}
			break;
		}
	}
	spin_unlock(&fl->hlock);
	if (ctx)
		*po = ctx;
	return err;
}

#define CMP(aa, bb) ((aa) == (bb) ? 0 : (aa) < (bb) ? -1 : 1)
static int overlap_ptr_cmp(const void *a, const void *b)
{
	struct overlap *pa = *((struct overlap **)a);
	struct overlap *pb = *((struct overlap **)b);
	/* sort with lowest starting buffer first */
	int st = CMP(pa->start, pb->start);
	/* sort with highest ending buffer first */
	int ed = CMP(pb->end, pa->end);
	return st == 0 ? ed : st;
}

static int context_build_overlap(struct smq_invoke_ctx *ctx)
{
<<<<<<< HEAD
	int i, err = 0;
=======
	int err = 0, i;
>>>>>>> p9x
	remote_arg_t *lpra = ctx->lpra;
	int inbufs = REMOTE_SCALARS_INBUFS(ctx->sc);
	int outbufs = REMOTE_SCALARS_OUTBUFS(ctx->sc);
	int nbufs = inbufs + outbufs;
	struct overlap max;
<<<<<<< HEAD

=======
>>>>>>> p9x
	for (i = 0; i < nbufs; ++i) {
		ctx->overs[i].start = (uintptr_t)lpra[i].buf.pv;
		ctx->overs[i].end = ctx->overs[i].start + lpra[i].buf.len;
		if (lpra[i].buf.len) {
			VERIFY(err, ctx->overs[i].end > ctx->overs[i].start);
			if (err)
				goto bail;
		}
		ctx->overs[i].raix = i;
		ctx->overps[i] = &ctx->overs[i];
	}
	sort(ctx->overps, nbufs, sizeof(*ctx->overps), overlap_ptr_cmp, NULL);
	max.start = 0;
	max.end = 0;
	for (i = 0; i < nbufs; ++i) {
		if (ctx->overps[i]->start < max.end) {
			ctx->overps[i]->mstart = max.end;
			ctx->overps[i]->mend = ctx->overps[i]->end;
			ctx->overps[i]->offset = max.end -
				ctx->overps[i]->start;
			if (ctx->overps[i]->end > max.end) {
				max.end = ctx->overps[i]->end;
			} else {
				ctx->overps[i]->mend = 0;
				ctx->overps[i]->mstart = 0;
			}
		} else  {
			ctx->overps[i]->mend = ctx->overps[i]->end;
			ctx->overps[i]->mstart = ctx->overps[i]->start;
			ctx->overps[i]->offset = 0;
			max = *ctx->overps[i];
		}
	}
bail:
	return err;
}

#define K_COPY_FROM_USER(err, kernel, dst, src, size) \
	do {\
		if (!(kernel))\
			VERIFY(err, 0 == copy_from_user((dst),\
			(void const __user *)(src),\
							(size)));\
		else\
			memmove((dst), (src), (size));\
	} while (0)

#define K_COPY_TO_USER(err, kernel, dst, src, size) \
	do {\
		if (!(kernel))\
			VERIFY(err, 0 == copy_to_user((void __user *)(dst), \
						(src), (size)));\
		else\
			memmove((dst), (src), (size));\
	} while (0)


static void context_free(struct smq_invoke_ctx *ctx);

static int context_alloc(struct fastrpc_file *fl, uint32_t kernel,
			 struct fastrpc_ioctl_invoke_fd *invokefd,
			 struct smq_invoke_ctx **po)
{
<<<<<<< HEAD
	int err = 0, bufs, ii, size = 0;
	struct fastrpc_apps *me = &gfa;
=======
	int err = 0, bufs, size = 0;
>>>>>>> p9x
	struct smq_invoke_ctx *ctx = NULL;
	struct fastrpc_ctx_lst *clst = &fl->clst;
	struct fastrpc_ioctl_invoke *invoke = &invokefd->inv;

<<<<<<< HEAD
	bufs = REMOTE_SCALARS_LENGTH(invoke->sc);
=======
	bufs = REMOTE_SCALARS_INBUFS(invoke->sc) +
			REMOTE_SCALARS_OUTBUFS(invoke->sc);
>>>>>>> p9x
	size = bufs * sizeof(*ctx->lpra) + bufs * sizeof(*ctx->maps) +
		sizeof(*ctx->fds) * (bufs) +
		sizeof(*ctx->overs) * (bufs) +
		sizeof(*ctx->overps) * (bufs);

	VERIFY(err, NULL != (ctx = kzalloc(sizeof(*ctx) + size, GFP_KERNEL)));
	if (err)
		goto bail;

	INIT_HLIST_NODE(&ctx->hn);
	hlist_add_fake(&ctx->hn);
	ctx->fl = fl;
	ctx->maps = (struct fastrpc_mmap **)(&ctx[1]);
	ctx->lpra = (remote_arg_t *)(&ctx->maps[bufs]);
	ctx->fds = (int *)(&ctx->lpra[bufs]);
	ctx->overs = (struct overlap *)(&ctx->fds[bufs]);
	ctx->overps = (struct overlap **)(&ctx->overs[bufs]);

	K_COPY_FROM_USER(err, kernel, (void *)ctx->lpra, invoke->pra,
					bufs * sizeof(*ctx->lpra));
	if (err)
		goto bail;

	if (invokefd->fds) {
		K_COPY_FROM_USER(err, kernel, ctx->fds, invokefd->fds,
						bufs * sizeof(*ctx->fds));
		if (err)
			goto bail;
	}
	ctx->sc = invoke->sc;
	if (bufs) {
		VERIFY(err, 0 == context_build_overlap(ctx));
		if (err)
			goto bail;
	}
	ctx->retval = -1;
	ctx->pid = current->pid;
	ctx->tgid = current->tgid;
	init_completion(&ctx->work);
	ctx->magic = FASTRPC_CTX_MAGIC;

	spin_lock(&fl->hlock);
	hlist_add_head(&ctx->hn, &clst->pending);
	spin_unlock(&fl->hlock);

<<<<<<< HEAD
	spin_lock(&me->ctxlock);
	for (ii = 0; ii < FASTRPC_CTX_MAX; ii++) {
		if (!me->ctxtable[ii]) {
			me->ctxtable[ii] = ctx;
			ctx->ctxid = (ptr_to_uint64(ctx) & ~0xFFF)|(ii << 4);
			break;
		}
	}
	spin_unlock(&me->ctxlock);
	VERIFY(err, ii < FASTRPC_CTX_MAX);
	if (err) {
		pr_err("adsprpc: out of context memory\n");
		goto bail;
	}

=======
>>>>>>> p9x
	*po = ctx;
bail:
	if (ctx && err)
		context_free(ctx);
	return err;
}

static void context_save_interrupted(struct smq_invoke_ctx *ctx)
{
	struct fastrpc_ctx_lst *clst = &ctx->fl->clst;
<<<<<<< HEAD

=======
>>>>>>> p9x
	spin_lock(&ctx->fl->hlock);
	hlist_del_init(&ctx->hn);
	hlist_add_head(&ctx->hn, &clst->interrupted);
	spin_unlock(&ctx->fl->hlock);
	/* free the cache on power collapse */
<<<<<<< HEAD
	fastrpc_cached_buf_list_free(ctx->fl);
=======
	fastrpc_buf_list_free(ctx->fl);
>>>>>>> p9x
}

static void context_free(struct smq_invoke_ctx *ctx)
{
	int i;
<<<<<<< HEAD
	struct fastrpc_apps *me = &gfa;
=======
>>>>>>> p9x
	int nbufs = REMOTE_SCALARS_INBUFS(ctx->sc) +
		    REMOTE_SCALARS_OUTBUFS(ctx->sc);
	spin_lock(&ctx->fl->hlock);
	hlist_del_init(&ctx->hn);
	spin_unlock(&ctx->fl->hlock);
	for (i = 0; i < nbufs; ++i)
		fastrpc_mmap_free(ctx->maps[i]);
	fastrpc_buf_free(ctx->buf, 1);
	ctx->magic = 0;
<<<<<<< HEAD
	ctx->ctxid = 0;

	spin_lock(&me->ctxlock);
	for (i = 0; i < FASTRPC_CTX_MAX; i++) {
		if (me->ctxtable[i] == ctx) {
			me->ctxtable[i] = NULL;
			break;
		}
	}
	spin_unlock(&me->ctxlock);

=======
>>>>>>> p9x
	kfree(ctx);
}

static void context_notify_user(struct smq_invoke_ctx *ctx, int retval)
{
	ctx->retval = retval;
	complete(&ctx->work);
}

<<<<<<< HEAD
=======

>>>>>>> p9x
static void fastrpc_notify_users(struct fastrpc_file *me)
{
	struct smq_invoke_ctx *ictx;
	struct hlist_node *n;
<<<<<<< HEAD

=======
>>>>>>> p9x
	spin_lock(&me->hlock);
	hlist_for_each_entry_safe(ictx, n, &me->clst.pending, hn) {
		complete(&ictx->work);
	}
	hlist_for_each_entry_safe(ictx, n, &me->clst.interrupted, hn) {
		complete(&ictx->work);
	}
	spin_unlock(&me->hlock);

}

static void fastrpc_notify_drivers(struct fastrpc_apps *me, int cid)
{
	struct fastrpc_file *fl;
	struct hlist_node *n;
<<<<<<< HEAD

=======
>>>>>>> p9x
	spin_lock(&me->hlock);
	hlist_for_each_entry_safe(fl, n, &me->drivers, hn) {
		if (fl->cid == cid)
			fastrpc_notify_users(fl);
	}
	spin_unlock(&me->hlock);

}
<<<<<<< HEAD

=======
>>>>>>> p9x
static void context_list_ctor(struct fastrpc_ctx_lst *me)
{
	INIT_HLIST_HEAD(&me->interrupted);
	INIT_HLIST_HEAD(&me->pending);
}

static void fastrpc_context_list_dtor(struct fastrpc_file *fl)
{
	struct fastrpc_ctx_lst *clst = &fl->clst;
<<<<<<< HEAD
	struct smq_invoke_ctx *ictx = NULL, *ctxfree;
	struct hlist_node *n;

=======
	struct smq_invoke_ctx *ictx = NULL, *ctxfree = NULL;
	struct hlist_node *n;
>>>>>>> p9x
	do {
		ctxfree = NULL;
		spin_lock(&fl->hlock);
		hlist_for_each_entry_safe(ictx, n, &clst->interrupted, hn) {
			hlist_del_init(&ictx->hn);
			ctxfree = ictx;
			break;
		}
		spin_unlock(&fl->hlock);
		if (ctxfree)
			context_free(ctxfree);
	} while (ctxfree);
	do {
		ctxfree = NULL;
		spin_lock(&fl->hlock);
		hlist_for_each_entry_safe(ictx, n, &clst->pending, hn) {
			hlist_del_init(&ictx->hn);
			ctxfree = ictx;
			break;
		}
		spin_unlock(&fl->hlock);
		if (ctxfree)
			context_free(ctxfree);
	} while (ctxfree);
}

static int fastrpc_file_free(struct fastrpc_file *fl);
<<<<<<< HEAD

=======
>>>>>>> p9x
static void fastrpc_file_list_dtor(struct fastrpc_apps *me)
{
	struct fastrpc_file *fl, *free;
	struct hlist_node *n;
<<<<<<< HEAD

=======
>>>>>>> p9x
	do {
		free = NULL;
		spin_lock(&me->hlock);
		hlist_for_each_entry_safe(fl, n, &me->drivers, hn) {
			hlist_del_init(&fl->hn);
			free = fl;
			break;
		}
		spin_unlock(&me->hlock);
		if (free)
			fastrpc_file_free(free);
	} while (free);
}

<<<<<<< HEAD
=======
static int fastrpc_mmap_create(struct fastrpc_file *fl, int fd, uintptr_t va,
		size_t len, int mflags, struct fastrpc_mmap **ppmap);

>>>>>>> p9x
static int get_args(uint32_t kernel, struct smq_invoke_ctx *ctx)
{
	remote_arg64_t *rpra;
	remote_arg_t *lpra = ctx->lpra;
	struct smq_invoke_buf *list;
	struct smq_phy_page *pages, *ipage;
	uint32_t sc = ctx->sc;
	int inbufs = REMOTE_SCALARS_INBUFS(sc);
	int outbufs = REMOTE_SCALARS_OUTBUFS(sc);
	int bufs = inbufs + outbufs;
	uintptr_t args;
	size_t rlen = 0, copylen = 0, metalen = 0;
	int i, inh, oix;
	int err = 0;
	int mflags = 0;

	/* calculate size of the metadata */
	rpra = NULL;
	list = smq_invoke_buf_start(rpra, sc);
	pages = smq_phy_page_start(sc, list);
	ipage = pages;

	for (i = 0; i < bufs; ++i) {
		uintptr_t buf = (uintptr_t)lpra[i].buf.pv;
		size_t len = lpra[i].buf.len;
<<<<<<< HEAD
		if (ctx->fds[i])
			fastrpc_mmap_create(ctx->fl, ctx->fds[i], buf, len,
					    mflags, &ctx->maps[i]);
=======
		if (ctx->fds[i] > 0) {
			VERIFY(err, !fastrpc_mmap_create(ctx->fl,
						ctx->fds[i], buf, len, mflags,
						&ctx->maps[i]));
			if (err) {
				pr_err("error mapping the buffer\n");
				goto bail;
			}
		}
>>>>>>> p9x
		ipage += 1;
	}
	metalen = copylen = (size_t)&ipage[0];
	/* calculate len requreed for copying */
	for (oix = 0; oix < inbufs + outbufs; ++oix) {
		int i = ctx->overps[oix]->raix;
		uintptr_t mstart, mend;
		size_t len = lpra[i].buf.len;

		if (!len)
			continue;
		if (ctx->maps[i])
			continue;
		if (ctx->overps[oix]->offset == 0)
			copylen = ALIGN(copylen, BALIGN);
		mstart = ctx->overps[oix]->mstart;
		mend = ctx->overps[oix]->mend;
		VERIFY(err, (mend - mstart) <= LONG_MAX);
		if (err)
			goto bail;
		copylen += mend - mstart;
		VERIFY(err, copylen >= 0);
		if (err)
			goto bail;
	}
	ctx->used = copylen;

	/* allocate new buffer */
	if (copylen) {
<<<<<<< HEAD
		DEFINE_DMA_ATTRS(ctx_attrs);

		err = fastrpc_buf_alloc(ctx->fl, copylen, ctx_attrs,
					0, 0, &ctx->buf);
=======
		VERIFY(err, !fastrpc_buf_alloc(ctx->fl, copylen, &ctx->buf));
>>>>>>> p9x
		if (err)
			goto bail;
	}
	/* copy metadata */
	rpra = ctx->buf->virt;
	ctx->rpra = rpra;
	list = smq_invoke_buf_start(rpra, sc);
	pages = smq_phy_page_start(sc, list);
	ipage = pages;
	args = (uintptr_t)ctx->buf->virt + metalen;
	for (i = 0; i < bufs; ++i) {
		size_t len = lpra[i].buf.len;
<<<<<<< HEAD

=======
>>>>>>> p9x
		list[i].num = 0;
		list[i].pgidx = 0;
		if (!len)
			continue;
		list[i].num = 1;
		list[i].pgidx = ipage - pages;
		ipage++;
	}
	/* map ion buffers */
	for (i = 0; i < inbufs + outbufs; ++i) {
		struct fastrpc_mmap *map = ctx->maps[i];
		uint64_t buf = ptr_to_uint64(lpra[i].buf.pv);
		size_t len = lpra[i].buf.len;

		rpra[i].buf.pv = 0;
		rpra[i].buf.len = len;
		if (!len)
			continue;
		if (map) {
			struct vm_area_struct *vma;
			uintptr_t offset;
			uint64_t num = buf_num_pages(buf, len);
			int idx = list[i].pgidx;

<<<<<<< HEAD
			down_read(&current->mm->mmap_sem);
			VERIFY(err, NULL != (vma = find_vma(current->mm,
								map->va)));
			if (err) {
				up_read(&current->mm->mmap_sem);
				goto bail;
			}
			offset = buf_page_start(buf) - vma->vm_start;
			up_read(&current->mm->mmap_sem);

			VERIFY(err, offset < (uintptr_t)map->size);
			if (err)
				goto bail;
			pages[idx].addr = map->phys + offset;
=======
			VERIFY(err, NULL != (vma = find_vma(current->mm,
								map->va)));
			if (err)
				goto bail;
			offset = buf_page_start(buf) - vma->vm_start;
			pages[idx].addr = map->phys + offset;
			if (msm_audio_ion_is_smmu_available())
				pages[idx].addr |= STREAM_ID;
>>>>>>> p9x
			pages[idx].size = num << PAGE_SHIFT;
		}
		rpra[i].buf.pv = buf;
	}
	/* copy non ion buffers */
	rlen = copylen - metalen;
	for (oix = 0; oix < inbufs + outbufs; ++oix) {
		int i = ctx->overps[oix]->raix;
		struct fastrpc_mmap *map = ctx->maps[i];
<<<<<<< HEAD
		size_t mlen;
		uint64_t buf;
		size_t len = lpra[i].buf.len;

=======
		size_t mlen = ctx->overps[oix]->mend - ctx->overps[oix]->mstart;
		uint64_t buf;
		size_t len = lpra[i].buf.len;
>>>>>>> p9x
		if (!len)
			continue;
		if (map)
			continue;
		if (ctx->overps[oix]->offset == 0) {
			rlen -= ALIGN(args, BALIGN) - args;
			args = ALIGN(args, BALIGN);
		}
<<<<<<< HEAD
		mlen = ctx->overps[oix]->mend - ctx->overps[oix]->mstart;
=======
>>>>>>> p9x
		VERIFY(err, rlen >= mlen);
		if (err)
			goto bail;
		rpra[i].buf.pv = (args - ctx->overps[oix]->offset);
		pages[list[i].pgidx].addr = ctx->buf->phys -
					    ctx->overps[oix]->offset +
					    (copylen - rlen);
<<<<<<< HEAD
=======
		if (msm_audio_ion_is_smmu_available())
			pages[list[i].pgidx].addr |= STREAM_ID;
>>>>>>> p9x
		pages[list[i].pgidx].addr =
			buf_page_start(pages[list[i].pgidx].addr);
		buf = rpra[i].buf.pv;
		pages[list[i].pgidx].size = buf_num_pages(buf, len) * PAGE_SIZE;
		if (i < inbufs) {
			K_COPY_FROM_USER(err, kernel, uint64_to_ptr(buf),
					lpra[i].buf.pv, len);
			if (err)
				goto bail;
		}
		args = args + mlen;
		rlen -= mlen;
	}

<<<<<<< HEAD
	for (oix = 0; oix < inbufs + outbufs; ++oix) {
		int i = ctx->overps[oix]->raix;
		struct fastrpc_mmap *map = ctx->maps[i];
		if (map && map->uncached)
			continue;
		if (rpra[i].buf.len && ctx->overps[oix]->mstart) {
			if (map && map->handle)
				msm_ion_do_cache_op(ctx->fl->apps->client,
					map->handle,
					uint64_to_ptr(rpra[i].buf.pv),
					rpra[i].buf.len,
					ION_IOC_CLEAN_INV_CACHES);
			else
				dmac_flush_range(uint64_to_ptr(rpra[i].buf.pv),
					uint64_to_ptr(rpra[i].buf.pv
						+ rpra[i].buf.len));
		}
=======
	for (i = 0; i < inbufs; ++i) {
		if (rpra[i].buf.len)
			dmac_flush_range(uint64_to_ptr(rpra[i].buf.pv),
			      uint64_to_ptr(rpra[i].buf.pv + rpra[i].buf.len));
>>>>>>> p9x
	}
	inh = inbufs + outbufs;
	for (i = 0; i < REMOTE_SCALARS_INHANDLES(sc); i++) {
		rpra[inh + i].buf.pv = ptr_to_uint64(ctx->lpra[inh + i].buf.pv);
		rpra[inh + i].buf.len = ctx->lpra[inh + i].buf.len;
		rpra[inh + i].h = ctx->lpra[inh + i].h;
	}
<<<<<<< HEAD

=======
	dmac_flush_range((char *)rpra, (char *)rpra + ctx->used);
>>>>>>> p9x
 bail:
	return err;
}

static int put_args(uint32_t kernel, struct smq_invoke_ctx *ctx,
		    remote_arg_t *upra)
{
	uint32_t sc = ctx->sc;
	remote_arg64_t *rpra = ctx->rpra;
<<<<<<< HEAD
	int i, inbufs, outbufs, outh, size;
=======
	int i, inbufs, outbufs, outh, num;
>>>>>>> p9x
	int err = 0;

	inbufs = REMOTE_SCALARS_INBUFS(sc);
	outbufs = REMOTE_SCALARS_OUTBUFS(sc);
	for (i = inbufs; i < inbufs + outbufs; ++i) {
		if (!ctx->maps[i]) {
			K_COPY_TO_USER(err, kernel,
				ctx->lpra[i].buf.pv,
				uint64_to_ptr(rpra[i].buf.pv),
				rpra[i].buf.len);
			if (err)
				goto bail;
		} else {
			fastrpc_mmap_free(ctx->maps[i]);
			ctx->maps[i] = NULL;
		}
	}
<<<<<<< HEAD
	size = sizeof(*rpra) * REMOTE_SCALARS_OUTHANDLES(sc);
	if (size) {
		outh = inbufs + outbufs + REMOTE_SCALARS_INHANDLES(sc);
		K_COPY_TO_USER(err, kernel, &upra[outh], &rpra[outh], size);
=======
	num = REMOTE_SCALARS_OUTHANDLES(sc);
	if (num) {
		outh = inbufs + outbufs + REMOTE_SCALARS_INHANDLES(sc);
		K_COPY_TO_USER(err, kernel, &upra[outh], &ctx->lpra[outh],
				num * sizeof(*ctx->lpra));
>>>>>>> p9x
		if (err)
			goto bail;
	}
 bail:
	return err;
}

<<<<<<< HEAD
static void inv_args_pre(struct smq_invoke_ctx *ctx)
{
	int i, inbufs, outbufs;
	uint32_t sc = ctx->sc;
	remote_arg64_t *rpra = ctx->rpra;
=======
static void inv_args_pre(uint32_t sc, remote_arg64_t *rpra)
{
	int i, inbufs, outbufs;
>>>>>>> p9x
	uintptr_t end;

	inbufs = REMOTE_SCALARS_INBUFS(sc);
	outbufs = REMOTE_SCALARS_OUTBUFS(sc);
	for (i = inbufs; i < inbufs + outbufs; ++i) {
<<<<<<< HEAD
		struct fastrpc_mmap *map = ctx->maps[i];

=======
>>>>>>> p9x
		if (!rpra[i].buf.len)
			continue;
		if (buf_page_start(ptr_to_uint64((void *)rpra)) ==
				buf_page_start(rpra[i].buf.pv))
			continue;
<<<<<<< HEAD
		if (!IS_CACHE_ALIGNED((uintptr_t)
				uint64_to_ptr(rpra[i].buf.pv))) {
			if (map && map->handle)
				msm_ion_do_cache_op(ctx->fl->apps->client,
					map->handle,
					uint64_to_ptr(rpra[i].buf.pv),
					sizeof(uintptr_t),
					ION_IOC_CLEAN_INV_CACHES);
			else
				dmac_flush_range(
					uint64_to_ptr(rpra[i].buf.pv), (char *)
					uint64_to_ptr(rpra[i].buf.pv + 1));
		}
		end = (uintptr_t)uint64_to_ptr(rpra[i].buf.pv +
							rpra[i].buf.len);
		if (!IS_CACHE_ALIGNED(end)) {
			if (map && map->handle)
				msm_ion_do_cache_op(ctx->fl->apps->client,
						map->handle,
						uint64_to_ptr(end),
						sizeof(uintptr_t),
						ION_IOC_CLEAN_INV_CACHES);
			else
				dmac_flush_range((char *)end,
					(char *)end + 1);
		}
	}
}

static void inv_args(struct smq_invoke_ctx *ctx)
=======
		if (!IS_CACHE_ALIGNED((uintptr_t)uint64_to_ptr(rpra[i].buf.pv)))
			dmac_flush_range(uint64_to_ptr(rpra[i].buf.pv),
				(char *)(uint64_to_ptr(rpra[i].buf.pv + 1)));
		end = (uintptr_t)uint64_to_ptr(rpra[i].buf.pv +
							rpra[i].buf.len);
		if (!IS_CACHE_ALIGNED(end))
			dmac_flush_range((char *)end,
				(char *)end + 1);
	}
}

static void inv_args(struct smq_invoke_ctx* ctx)
>>>>>>> p9x
{
	int i, inbufs, outbufs;
	uint32_t sc = ctx->sc;
	remote_arg64_t *rpra = ctx->rpra;
<<<<<<< HEAD
=======
	int used = ctx->used;
>>>>>>> p9x
	int inv = 0;

	inbufs = REMOTE_SCALARS_INBUFS(sc);
	outbufs = REMOTE_SCALARS_OUTBUFS(sc);
	for (i = inbufs; i < inbufs + outbufs; ++i) {
		struct fastrpc_mmap *map = ctx->maps[i];
<<<<<<< HEAD
		if (map && map->uncached)
			continue;
=======

>>>>>>> p9x
		if (!rpra[i].buf.len)
			continue;
		if (buf_page_start(ptr_to_uint64((void *)rpra)) ==
				buf_page_start(rpra[i].buf.pv)) {
			inv = 1;
			continue;
		}
		if (map && map->handle)
<<<<<<< HEAD
			msm_ion_do_cache_op(ctx->fl->apps->client, map->handle,
				uint64_to_ptr(rpra[i].buf.pv),
				rpra[i].buf.len, ION_IOC_INV_CACHES);
=======
			msm_ion_do_cache_op(map->client, map->handle,
				uint64_to_ptr(rpra[i].buf.pv), rpra[i].buf.len,
				ION_IOC_INV_CACHES);
>>>>>>> p9x
		else
			dmac_inv_range((char *)uint64_to_ptr(rpra[i].buf.pv),
				(char *)uint64_to_ptr(rpra[i].buf.pv
						 + rpra[i].buf.len));
	}

<<<<<<< HEAD
=======
	if (inv || REMOTE_SCALARS_OUTHANDLES(sc))
		dmac_inv_range(rpra, (char *)rpra + used);
>>>>>>> p9x
}

static int fastrpc_invoke_send(struct smq_invoke_ctx *ctx,
			       uint32_t kernel, uint32_t handle)
{
<<<<<<< HEAD
	struct smq_msg *msg = &ctx->msg;
	struct fastrpc_file *fl = ctx->fl;
	int err = 0, len;

	VERIFY(err, NULL != fl->apps->channel[fl->cid].chan);
	if (err)
		goto bail;
	msg->pid = current->tgid;
	msg->tid = current->pid;
	if (kernel)
		msg->pid = 0;
	msg->invoke.header.ctx = ctx->ctxid;
	msg->invoke.header.handle = handle;
	msg->invoke.header.sc = ctx->sc;
	msg->invoke.page.addr = ctx->buf ? ctx->buf->phys : 0;
	msg->invoke.page.size = buf_page_size(ctx->used);

	if (fl->apps->glink) {
		err = glink_tx(fl->apps->channel[fl->cid].chan,
			(void *)&fl->apps->channel[fl->cid], msg, sizeof(*msg),
			GLINK_TX_REQ_INTENT);
	} else {
		spin_lock(&fl->apps->hlock);
		len = smd_write((smd_channel_t *)
				fl->apps->channel[fl->cid].chan,
				msg, sizeof(*msg));
		spin_unlock(&fl->apps->hlock);
		VERIFY(err, len == sizeof(*msg));
	}
 bail:
	return err;
}

static void fastrpc_smd_read_handler(int cid)
{
	struct fastrpc_apps *me = &gfa;
	struct smq_invoke_rsp rsp = {0};
	int ret = 0, err = 0;
	uint32_t index;
=======
	struct smq_msg msg = {0};
	struct fastrpc_file *fl = ctx->fl;
	int err = 0, len;
	msg.pid = current->tgid;
	msg.tid = current->pid;
	if (kernel)
		msg.pid = 0;
	msg.invoke.header.ctx = ptr_to_uint64(ctx);
	msg.invoke.header.handle = handle;
	msg.invoke.header.sc = ctx->sc;
	msg.invoke.page.addr = ctx->buf ? ctx->buf->phys : 0;
	msg.invoke.page.size = buf_page_size(ctx->used);
	if (msm_audio_ion_is_smmu_available()
		&& msg.invoke.page.addr != 0)
		msg.invoke.page.addr |= STREAM_ID;
	spin_lock(&fl->apps->hlock);
	len = smd_write(fl->chan->chan, &msg, sizeof(msg));
	spin_unlock(&fl->apps->hlock);
	VERIFY(err, len == sizeof(msg));
	return err;
}

static void fastrpc_deinit(void)
{
	struct fastrpc_apps *me = &gfa;
	int i;

	for (i = 0; i < NUM_CHANNELS; i++) {
		struct fastrpc_chan_ctx *chan = &me->channel[i];
		if (chan->chan) {
			(void)smd_close(chan->chan);
			chan->chan = 0;
		}
		if (chan->smmu.dev)
			chan->smmu.dev = 0;
		if (chan->smmu.mapping)
			chan->smmu.mapping = 0;
	}
}

static void fastrpc_read_handler(int cid)
{
	struct fastrpc_apps *me = &gfa;
	struct smq_invoke_rsp rsp = {0};
	struct smq_invoke_ctx *ctx;
	int ret = 0, err = 0;
>>>>>>> p9x

	do {
		ret = smd_read_from_cb(me->channel[cid].chan, &rsp,
					sizeof(rsp));
		if (ret != sizeof(rsp))
			break;
<<<<<<< HEAD
		index = (uint32_t)((rsp.ctx & FASTRPC_CTXID_MASK) >> 4);
		VERIFY(err, index < FASTRPC_CTX_MAX);
		if (err)
			goto bail;

		VERIFY(err, !IS_ERR_OR_NULL(me->ctxtable[index]));
		if (err)
			goto bail;

		VERIFY(err, ((me->ctxtable[index]->ctxid == (rsp.ctx)) &&
			me->ctxtable[index]->magic == FASTRPC_CTX_MAGIC));
		if (err)
			goto bail;

		context_notify_user(me->ctxtable[index], rsp.retval);
	} while (ret == sizeof(rsp));

=======
		ctx = (struct smq_invoke_ctx *)(uint64_to_ptr(rsp.ctx));
		VERIFY(err, (ctx && ctx->magic == FASTRPC_CTX_MAGIC));
		if (err)
			goto bail;
		context_notify_user(uint64_to_ptr(rsp.ctx), rsp.retval);
	} while (ret == sizeof(rsp));
>>>>>>> p9x
bail:
	if (err)
			pr_err("adsprpc: invalid response or context\n");
}

static void smd_event_handler(void *priv, unsigned event)
{
	struct fastrpc_apps *me = &gfa;
	int cid = (int)(uintptr_t)priv;

	switch (event) {
	case SMD_EVENT_OPEN:
		complete(&me->channel[cid].work);
		break;
	case SMD_EVENT_CLOSE:
		fastrpc_notify_drivers(me, cid);
		break;
	case SMD_EVENT_DATA:
<<<<<<< HEAD
		fastrpc_smd_read_handler(cid);
=======
		fastrpc_read_handler(cid);
>>>>>>> p9x
		break;
	}
}

static void fastrpc_init(struct fastrpc_apps *me)
{
	int i;
<<<<<<< HEAD

	INIT_HLIST_HEAD(&me->drivers);
	INIT_HLIST_HEAD(&me->fls);
	spin_lock_init(&me->hlock);
	spin_lock_init(&me->ctxlock);
	mutex_init(&me->smd_mutex);
	me->channel = &gcinfo[0];
	for (i = 0; i < NUM_CHANNELS; i++) {
		init_completion(&me->channel[i].work);
		me->channel[i].bitmap = 0;
		me->channel[i].sesscount = 0;
	}
=======
	INIT_HLIST_HEAD(&me->drivers);
	INIT_HLIST_HEAD(&me->maps);
	spin_lock_init(&me->hlock);
	mutex_init(&me->smd_mutex);
	for (i = 0; i < NUM_CHANNELS; i++)
		init_completion(&me->channel[i].work);
>>>>>>> p9x
}

static int fastrpc_release_current_dsp_process(struct fastrpc_file *fl);

static int fastrpc_internal_invoke(struct fastrpc_file *fl, uint32_t mode,
				   uint32_t kernel,
				   struct fastrpc_ioctl_invoke_fd *invokefd)
{
	struct smq_invoke_ctx *ctx = NULL;
	struct fastrpc_ioctl_invoke *invoke = &invokefd->inv;
	int cid = fl->cid;
	int interrupted = 0;
	int err = 0;

<<<<<<< HEAD
	VERIFY(err, fl->sctx != NULL);
	if (err)
		goto bail;
	VERIFY(err, fl->cid >= 0 && fl->cid < NUM_CHANNELS);
	if (err)
		goto bail;

=======
>>>>>>> p9x
	if (!kernel) {
		VERIFY(err, 0 == context_restore_interrupted(fl, invokefd,
								&ctx));
		if (err)
			goto bail;
<<<<<<< HEAD
		if (fl->sctx->smmu.faults)
			err = FASTRPC_ENOSUCH;
		if (err)
			goto bail;
=======
>>>>>>> p9x
		if (ctx)
			goto wait;
	}

	VERIFY(err, 0 == context_alloc(fl, kernel, invokefd, &ctx));
	if (err)
		goto bail;

	if (REMOTE_SCALARS_LENGTH(ctx->sc)) {
		VERIFY(err, 0 == get_args(kernel, ctx));
		if (err)
			goto bail;
	}

<<<<<<< HEAD
	inv_args_pre(ctx);
=======
	inv_args_pre(ctx->sc, ctx->rpra);
>>>>>>> p9x
	if (FASTRPC_MODE_SERIAL == mode)
		inv_args(ctx);
	VERIFY(err, 0 == fastrpc_invoke_send(ctx, kernel, invoke->handle));
	if (err)
		goto bail;
	if (FASTRPC_MODE_PARALLEL == mode)
		inv_args(ctx);
 wait:
	if (kernel)
		wait_for_completion(&ctx->work);
	else {
		interrupted = wait_for_completion_interruptible(&ctx->work);
		VERIFY(err, 0 == (err = interrupted));
		if (err)
			goto bail;
	}
	VERIFY(err, 0 == (err = ctx->retval));
	if (err)
		goto bail;
	VERIFY(err, 0 == put_args(kernel, ctx, invoke->pra));
	if (err)
		goto bail;
 bail:
	if (ctx && interrupted == -ERESTARTSYS)
		context_save_interrupted(ctx);
	else if (ctx)
		context_free(ctx);
	if (fl->ssrcount != fl->apps->channel[cid].ssrcount)
		err = ECONNRESET;
	return err;
}

static int fastrpc_init_process(struct fastrpc_file *fl,
				struct fastrpc_ioctl_init *init)
{
	int err = 0;
	struct fastrpc_ioctl_invoke_fd ioctl;
	struct smq_phy_page pages[1];
<<<<<<< HEAD
	struct fastrpc_mmap *file = NULL, *mem = NULL;
	struct fastrpc_buf *imem = NULL;

	if (init->flags == FASTRPC_INIT_ATTACH) {
		remote_arg_t ra[1];
		int tgid = current->tgid;

=======
	struct fastrpc_mmap *file = 0, *mem = 0;
	int mflags = 0;
	if (init->flags == FASTRPC_INIT_ATTACH) {
		remote_arg_t ra[1];
		int tgid = current->tgid;
>>>>>>> p9x
		ra[0].buf.pv = (void *)&tgid;
		ra[0].buf.len = sizeof(tgid);
		ioctl.inv.handle = 1;
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(0, 1, 0);
		ioctl.inv.pra = ra;
		ioctl.fds = NULL;
		VERIFY(err, !(err = fastrpc_internal_invoke(fl,
			FASTRPC_MODE_PARALLEL, 1, &ioctl)));
		if (err)
			goto bail;
	} else if (init->flags == FASTRPC_INIT_CREATE) {
		remote_arg_t ra[4];
		int fds[4];
<<<<<<< HEAD
		int mflags = 0;
		int memlen;
		DEFINE_DMA_ATTRS(imem_dma_attr);
=======
>>>>>>> p9x
		struct {
			int pgid;
			int namelen;
			int filelen;
			int pageslen;
		} inbuf;
		inbuf.pgid = current->tgid;
<<<<<<< HEAD
		inbuf.namelen = strlen(current->comm) + 1;
		inbuf.filelen = init->filelen;
		VERIFY(err, access_ok(0, (void __user *)init->file,
			init->filelen));
		if (err)
			goto bail;
		if (init->filelen) {
			VERIFY(err, !fastrpc_mmap_create(fl, init->filefd,
				init->file, init->filelen, mflags, &file));
			if (err)
				goto bail;
		}
		inbuf.pageslen = 1;

		VERIFY(err, !init->mem);
		if (err) {
			err = -EINVAL;
			pr_err("adsprpc: %s: %s: ERROR: donated memory allocated in userspace\n",
				current->comm, __func__);
			goto bail;
		}
		memlen = ALIGN(max(1024*1024*3, (int)init->filelen * 4),
						1024*1024);

		dma_set_attr(DMA_ATTR_EXEC_MAPPING, &imem_dma_attr);
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &imem_dma_attr);

		err = fastrpc_buf_alloc(fl, memlen, imem_dma_attr, 0, 0, &imem);
		if (err)
			goto bail;
		fl->init_mem = imem;

=======
		inbuf.namelen = strlen(current->comm);
		inbuf.filelen = init->filelen;
		VERIFY(err, !fastrpc_mmap_create(fl, init->filefd, init->file,
						 init->filelen, mflags, &file));
		if (err)
			goto bail;
		inbuf.pageslen = 1;
		VERIFY(err, !fastrpc_mmap_create(fl, init->memfd, init->mem,
						 init->memlen, mflags, &mem));
		if (err)
			goto bail;
>>>>>>> p9x
		inbuf.pageslen = 1;
		ra[0].buf.pv = (void *)&inbuf;
		ra[0].buf.len = sizeof(inbuf);
		fds[0] = 0;

		ra[1].buf.pv = (void *)current->comm;
		ra[1].buf.len = inbuf.namelen;
		fds[1] = 0;

		ra[2].buf.pv = (void *)init->file;
		ra[2].buf.len = inbuf.filelen;
		fds[2] = init->filefd;

<<<<<<< HEAD
		pages[0].addr = imem->phys;
		pages[0].size = imem->size;
=======
		pages[0].addr = mem->phys;
		if (msm_audio_ion_is_smmu_available())
			pages[0].addr |= STREAM_ID;
		pages[0].size = mem->size;
>>>>>>> p9x
		ra[3].buf.pv = (void *)pages;
		ra[3].buf.len = 1 * sizeof(*pages);
		fds[3] = 0;

		ioctl.inv.handle = 1;
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(6, 4, 0);
		ioctl.inv.pra = ra;
		ioctl.fds = fds;
		VERIFY(err, !(err = fastrpc_internal_invoke(fl,
			FASTRPC_MODE_PARALLEL, 1, &ioctl)));
		if (err)
			goto bail;
	} else {
		err = -ENOTTY;
	}
bail:
	if (mem && err)
		fastrpc_mmap_free(mem);
	if (file)
		fastrpc_mmap_free(file);
	return err;
}

static int fastrpc_release_current_dsp_process(struct fastrpc_file *fl)
{
	int err = 0;
	struct fastrpc_ioctl_invoke_fd ioctl;
	remote_arg_t ra[1];
	int tgid = 0;

<<<<<<< HEAD
	VERIFY(err, fl->apps->channel[fl->cid].chan != NULL);
	if (err)
		goto bail;
=======
>>>>>>> p9x
	tgid = fl->tgid;
	ra[0].buf.pv = (void *)&tgid;
	ra[0].buf.len = sizeof(tgid);
	ioctl.inv.handle = 1;
	ioctl.inv.sc = REMOTE_SCALARS_MAKE(1, 1, 0);
	ioctl.inv.pra = ra;
	ioctl.fds = NULL;
	VERIFY(err, 0 == (err = fastrpc_internal_invoke(fl,
		FASTRPC_MODE_PARALLEL, 1, &ioctl)));
<<<<<<< HEAD
bail:
=======
>>>>>>> p9x
	return err;
}

static int fastrpc_mmap_on_dsp(struct fastrpc_file *fl, uint32_t flags,
<<<<<<< HEAD
					uintptr_t va, uint64_t phys,
					size_t size, uintptr_t *raddr)
=======
			       struct fastrpc_mmap *map)
>>>>>>> p9x
{
	struct fastrpc_ioctl_invoke_fd ioctl;
	struct smq_phy_page page;
	int num = 1;
	remote_arg_t ra[3];
	int err = 0;
	struct {
		int pid;
		uint32_t flags;
		uintptr_t vaddrin;
		int num;
	} inargs;

	struct {
		uintptr_t vaddrout;
	} routargs;
<<<<<<< HEAD

	inargs.pid = current->tgid;
	inargs.vaddrin = (uintptr_t)va;
=======
	inargs.pid = current->tgid;
	inargs.vaddrin = (uintptr_t)map->va;
>>>>>>> p9x
	inargs.flags = flags;
	inargs.num = fl->apps->compat ? num * sizeof(page) : num;
	ra[0].buf.pv = (void *)&inargs;
	ra[0].buf.len = sizeof(inargs);
<<<<<<< HEAD
	page.addr = phys;
	page.size = size;
=======
	page.addr = map->phys;
	if (msm_audio_ion_is_smmu_available() && flags != ADSP_MMAP_HEAP_ADDR)
		page.addr |= STREAM_ID;
	page.size = map->size;
>>>>>>> p9x
	ra[1].buf.pv = (void *)&page;
	ra[1].buf.len = num * sizeof(page);

	ra[2].buf.pv = (void *)&routargs;
	ra[2].buf.len = sizeof(routargs);

	ioctl.inv.handle = 1;
	if (fl->apps->compat)
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(4, 2, 1);
	else
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(2, 2, 1);
	ioctl.inv.pra = ra;
	ioctl.fds = NULL;
	VERIFY(err, 0 == (err = fastrpc_internal_invoke(fl,
		FASTRPC_MODE_PARALLEL, 1, &ioctl)));
<<<<<<< HEAD
	*raddr = (uintptr_t)routargs.vaddrout;
=======
	map->raddr = (uintptr_t)routargs.vaddrout;
>>>>>>> p9x
	if (err)
		goto bail;
	if (flags == ADSP_MMAP_HEAP_ADDR) {
		struct scm_desc desc = {0};
<<<<<<< HEAD

		desc.args[0] = TZ_PIL_AUTH_QDSP6_PROC;
		desc.args[1] = phys;
		desc.args[2] = size;
=======
		desc.args[0] = TZ_PIL_AUTH_QDSP6_PROC;
		desc.args[1] = map->phys;
		desc.args[2] = map->size;
>>>>>>> p9x
		desc.arginfo = SCM_ARGS(3);
		err = scm_call2(SCM_SIP_FNID(SCM_SVC_PIL,
			TZ_PIL_PROTECT_MEM_SUBSYS_ID), &desc);
	}

bail:
	return err;
}

<<<<<<< HEAD
static int fastrpc_munmap_on_dsp_rh(struct fastrpc_file *fl, uint64_t phys,
						size_t size, uint32_t flags)
=======
static int fastrpc_munmap_on_dsp_rh(struct fastrpc_file *fl,
				 struct fastrpc_mmap *map)
>>>>>>> p9x
{
	struct fastrpc_ioctl_invoke_fd ioctl;
	struct scm_desc desc = {0};
	remote_arg_t ra[1];
	int err = 0;
	struct {
		uint8_t skey;
	} routargs;

	ra[0].buf.pv = (void *)&routargs;
	ra[0].buf.len = sizeof(routargs);

	ioctl.inv.handle = 1;
	ioctl.inv.sc = REMOTE_SCALARS_MAKE(7, 0, 1);
	ioctl.inv.pra = ra;
	ioctl.fds = NULL;

	VERIFY(err, 0 == (err = fastrpc_internal_invoke(fl,
			FASTRPC_MODE_PARALLEL, 1, &ioctl)));
	if (err)
		goto bail;
	desc.args[0] = TZ_PIL_AUTH_QDSP6_PROC;
<<<<<<< HEAD
	desc.args[1] = phys;
	desc.args[2] = size;
=======
	desc.args[1] = map->phys;
	desc.args[2] = map->size;
>>>>>>> p9x
	desc.args[3] = routargs.skey;
	desc.arginfo = SCM_ARGS(4);
	err = scm_call2(SCM_SIP_FNID(SCM_SVC_PIL,
		TZ_PIL_CLEAR_PROTECT_MEM_SUBSYS_ID), &desc);

bail:
	return err;
}

<<<<<<< HEAD
static int fastrpc_munmap_on_dsp(struct fastrpc_file *fl, uintptr_t raddr,
				uint64_t phys, size_t size, uint32_t flags)
{
	struct fastrpc_ioctl_invoke_fd ioctl;
	remote_arg_t ra[1];
	int err = 0;
=======
static int fastrpc_munmap_on_dsp(struct fastrpc_file *fl,
				 struct fastrpc_mmap *map)
{
	struct fastrpc_ioctl_invoke_fd ioctl;
	int err = 0;
	remote_arg_t ra[1];
>>>>>>> p9x
	struct {
		int pid;
		uintptr_t vaddrout;
		size_t size;
	} inargs;
<<<<<<< HEAD
	if (flags == ADSP_MMAP_HEAP_ADDR) {
		VERIFY(err, !fastrpc_munmap_on_dsp_rh(fl, phys, size, flags));
=======
	if (map->flags == ADSP_MMAP_HEAP_ADDR) {
		VERIFY(err, !fastrpc_munmap_on_dsp_rh(fl, map));
>>>>>>> p9x
		if (err)
			goto bail;
	}

	inargs.pid = current->tgid;
<<<<<<< HEAD
	inargs.size = size;
	inargs.vaddrout = raddr;
=======
	inargs.size = map->size;
	inargs.vaddrout = map->raddr;
>>>>>>> p9x
	ra[0].buf.pv = (void *)&inargs;
	ra[0].buf.len = sizeof(inargs);

	ioctl.inv.handle = 1;
	if (fl->apps->compat)
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(5, 1, 0);
	else
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(3, 1, 0);
	ioctl.inv.pra = ra;
	ioctl.fds = NULL;
<<<<<<< HEAD
	VERIFY(err, 0 == (err = fastrpc_internal_invoke(fl,
		FASTRPC_MODE_PARALLEL, 1, &ioctl)));
=======

	VERIFY(err, 0 == (err = fastrpc_internal_invoke(fl,
			FASTRPC_MODE_PARALLEL, 1, &ioctl)));

>>>>>>> p9x
bail:
	return err;
}

static int fastrpc_mmap_remove_ssr(struct fastrpc_file *fl)
{
	struct fastrpc_mmap *match = NULL, *map = NULL;
	struct hlist_node *n = NULL;
<<<<<<< HEAD
	int err = 0, ret = 0;
	struct fastrpc_apps *me = &gfa;
	struct ramdump_segment *ramdump_segments_rh = NULL;

=======
	int err = 0;
	struct fastrpc_apps *me = &gfa;
>>>>>>> p9x
	spin_lock(&me->hlock);
	hlist_for_each_entry_safe(map, n, &me->maps, hn) {
			match = map;
			hlist_del_init(&map->hn);
			break;
	}
	spin_unlock(&me->hlock);

	if (match) {
<<<<<<< HEAD
		VERIFY(err, !fastrpc_munmap_on_dsp_rh(fl, match->phys,
						match->size, match->flags));
		if (err)
			goto bail;
		if (me->channel[0].ramdumpenabled) {
			ramdump_segments_rh = kcalloc(1,
				sizeof(struct ramdump_segment), GFP_KERNEL);
			if (ramdump_segments_rh) {
				ramdump_segments_rh->address = match->phys;
				ramdump_segments_rh->size = match->size;
				ret = do_elf_ramdump(
					me->channel[0].remoteheap_ramdump_dev,
					ramdump_segments_rh, 1);
				if (ret < 0)
					pr_err("ADSPRPC: unable to dump heap");
				kfree(ramdump_segments_rh);
			}
		}
=======
		VERIFY(err, !fastrpc_munmap_on_dsp_rh(fl, match));
		if (err)
			goto bail;
>>>>>>> p9x
		fastrpc_mmap_free(match);
	}
bail:
	if (err && match)
		fastrpc_mmap_add(match);
	return err;
}

static int fastrpc_mmap_remove(struct fastrpc_file *fl, uintptr_t va,
			     size_t len, struct fastrpc_mmap **ppmap);

static void fastrpc_mmap_add(struct fastrpc_mmap *map);

<<<<<<< HEAD
static inline void get_fastrpc_ioctl_mmap_64(
			struct fastrpc_ioctl_mmap_64 *mmap64,
			struct fastrpc_ioctl_mmap *immap)
{
	immap->fd = mmap64->fd;
	immap->flags = mmap64->flags;
	immap->vaddrin = (uintptr_t)mmap64->vaddrin;
	immap->size = mmap64->size;
}

static inline void put_fastrpc_ioctl_mmap_64(
			struct fastrpc_ioctl_mmap_64 *mmap64,
			struct fastrpc_ioctl_mmap *immap)
{
	mmap64->vaddrout = (uint64_t)immap->vaddrout;
}

static inline void get_fastrpc_ioctl_munmap_64(
			struct fastrpc_ioctl_munmap_64 *munmap64,
			struct fastrpc_ioctl_munmap *imunmap)
{
	imunmap->vaddrout = (uintptr_t)munmap64->vaddrout;
	imunmap->size = munmap64->size;
}

=======
>>>>>>> p9x
static int fastrpc_internal_munmap(struct fastrpc_file *fl,
				   struct fastrpc_ioctl_munmap *ud)
{
	int err = 0;
	struct fastrpc_mmap *map = NULL;
<<<<<<< HEAD
	struct fastrpc_buf *rbuf = NULL, *free = NULL;
	struct hlist_node *n;

	mutex_lock(&fl->map_mutex);
	spin_lock(&fl->hlock);
	hlist_for_each_entry_safe(rbuf, n, &fl->remote_bufs, hn_rem) {
		if (rbuf->raddr && (rbuf->flags == ADSP_MMAP_ADD_PAGES)) {
			if ((rbuf->raddr == ud->vaddrout) &&
				(rbuf->size == ud->size)) {
				free = rbuf;
				break;
			}
		}
	}
	spin_unlock(&fl->hlock);

	if (free) {
		VERIFY(err, !fastrpc_munmap_on_dsp(fl, free->raddr,
			free->phys, free->size, free->flags));
		if (err)
			goto bail;
		fastrpc_buf_free(rbuf, 0);
		mutex_unlock(&fl->map_mutex);
		return err;
	}
	if (!fastrpc_mmap_remove(fl, ud->vaddrout, ud->size,
				 &map)) {
		VERIFY(err, !fastrpc_munmap_on_dsp(fl, map->raddr,
				map->phys, map->size, map->flags));
=======

	if (!fastrpc_mmap_remove(fl, ud->vaddrout, ud->size,
			&map)) {
		VERIFY(err, !fastrpc_munmap_on_dsp(fl, map));
>>>>>>> p9x
		if (err)
			goto bail;
		fastrpc_mmap_free(map);
	}
bail:
	if (err && map)
		fastrpc_mmap_add(map);
<<<<<<< HEAD
	mutex_unlock(&fl->map_mutex);
=======
	return err;
}

static int fastrpc_mmap_find(struct fastrpc_file *fl, int fd, uintptr_t va,
			size_t len, int mflags, struct fastrpc_mmap **ppmap)
{
	struct fastrpc_mmap *match = NULL, *map = NULL;
	struct hlist_node *n = NULL;

	if (mflags == ADSP_MMAP_HEAP_ADDR) {
		struct fastrpc_apps *me = &gfa;
		spin_lock(&me->hlock);
		hlist_for_each_entry_safe(map, n, &me->maps, hn) {
			if (map->va >= va &&
					map->va + map->len <= va + len &&
					map->fd == fd) {
				map->refs++;
				match = map;
				break;
			}
		}
		spin_unlock(&me->hlock);
	} else {
		spin_lock(&fl->hlock);
		hlist_for_each_entry_safe(map, n, &fl->maps, hn) {
			if (map->va >= va &&
					map->va + map->len <= va + len &&
					map->fd == fd) {
				map->refs++;
				match = map;
				break;
			}
		}
		spin_unlock(&fl->hlock);
	}
	if (match) {
		*ppmap = match;
		return 0;
	}
	return -ENOTTY;
}

static int dma_alloc_memory(phys_addr_t *region_start, size_t size)
{
	struct fastrpc_apps *me = &gfa;
	void *vaddr = NULL;
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_SKIP_ZEROING, &attrs);
	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
	vaddr = dma_alloc_attrs(me->adsp_mem_device, size,
						region_start, GFP_KERNEL,
						&attrs);
	if (!vaddr) {
		pr_err("ADSPRPC: Failed to allocate remote heap memory\n");
		return -ENOTTY;
	}
	return 0;
}

static int fastrpc_mmap_create(struct fastrpc_file *fl, int fd, uintptr_t va,
			size_t len, int mflags, struct fastrpc_mmap **ppmap)
{
	int err = 0;
	struct fastrpc_mmap *map = NULL;
	struct fastrpc_apps *me = &gfa;
	unsigned long ionflag = 0;
	size_t pa_len = 0;
	ion_phys_addr_t paddr = 0;
	void *virtual_addr = NULL;
	phys_addr_t region_start = 0;

	if (!fastrpc_mmap_find(fl, fd, va, len, mflags, ppmap))
		return 0;
	VERIFY(err, map = kzalloc(sizeof(*map), GFP_KERNEL));
	if (err)
		goto bail;
	map->flags = mflags;
	map->client = NULL;
	map->handle = NULL;
	map->fl = fl;
	map->fd = fd;
	if (mflags == ADSP_MMAP_HEAP_ADDR) {
		map->apps = me;
		map->fl = NULL;
		VERIFY(err, !dma_alloc_memory(&region_start, len));
		if (err)
			goto bail;
		map->phys = (uintptr_t)region_start;
		map->size = len;
	} else {
		VERIFY(err, !msm_audio_ion_import(DEVICE_NAME, &(map->client),
						&(map->handle), fd,
						&ionflag, len,
					&paddr, &pa_len, &virtual_addr));
		if (err) {
			pr_err("msm_audio_ion_import failed\n");
			goto bail;
		}
		map->phys = paddr;
		map->size = pa_len;
	}
	map->va = va;
	map->refs = 1;
	map->len = len;
	INIT_HLIST_NODE(&map->hn);
	fastrpc_mmap_add(map);

	*ppmap = map;

bail:
	if (err && map)
		fastrpc_mmap_free(map);
>>>>>>> p9x
	return err;
}

static int fastrpc_internal_mmap(struct fastrpc_file *fl,
				 struct fastrpc_ioctl_mmap *ud)
{
<<<<<<< HEAD

	struct fastrpc_mmap *map = NULL;
	struct fastrpc_buf *rbuf = NULL;
	uintptr_t raddr = 0;
	int err = 0;

	mutex_lock(&fl->map_mutex);

	if (ud->flags == ADSP_MMAP_ADD_PAGES) {
		DEFINE_DMA_ATTRS(dma_attr);

		if (ud->vaddrin) {
			err = -EINVAL;
			pr_err("adsprpc: %s: %s: ERROR: adding user allocated pages is not supported\n",
					current->comm, __func__);
			goto bail;
		}
		dma_set_attr(DMA_ATTR_EXEC_MAPPING, &dma_attr);
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &dma_attr);

		err = fastrpc_buf_alloc(fl, ud->size, dma_attr, ud->flags,
								1, &rbuf);
		if (err)
			goto bail;
		err = fastrpc_mmap_on_dsp(fl, ud->flags, 0,
				rbuf->phys, rbuf->size, &raddr);
		if (err)
			goto bail;
		rbuf->raddr = raddr;
	} else {
		uintptr_t va_to_dsp;

		VERIFY(err, !fastrpc_mmap_create(fl, ud->fd,
				(uintptr_t)ud->vaddrin, ud->size,
				 ud->flags, &map));
		if (err)
			goto bail;

		if (ud->flags == ADSP_MMAP_HEAP_ADDR)
			va_to_dsp = 0;
		else
			va_to_dsp = (uintptr_t)map->va;
		VERIFY(err, 0 == fastrpc_mmap_on_dsp(fl, ud->flags, va_to_dsp,
				map->phys, map->size, &raddr));
		if (err)
			goto bail;
		map->raddr = raddr;
	}
	ud->vaddrout = raddr;
 bail:
	if (err && map)
		fastrpc_mmap_free(map);
	mutex_unlock(&fl->map_mutex);
=======
	struct fastrpc_mmap *map = NULL;
	int err = 0;
	if (!fastrpc_mmap_find(fl, ud->fd, (uintptr_t)ud->vaddrin, ud->size,
			       ud->flags, &map))
		return 0;

	VERIFY(err, !fastrpc_mmap_create(fl, ud->fd,
			(uintptr_t)ud->vaddrin, ud->size, ud->flags, &map));
	if (err)
		goto bail;
	VERIFY(err, 0 == fastrpc_mmap_on_dsp(fl, ud->flags, map));
	if (err)
		goto bail;
	ud->vaddrout = map->raddr;
 bail:
	if (err && map)
		fastrpc_mmap_free(map);
>>>>>>> p9x
	return err;
}

static void fastrpc_channel_close(struct kref *kref)
{
	struct fastrpc_apps *me = &gfa;
<<<<<<< HEAD
	struct fastrpc_channel_ctx *ctx;
	int cid;

	ctx = container_of(kref, struct fastrpc_channel_ctx, kref);
	if (!me->glink) {
		smd_close(ctx->chan);
	} else {
		glink_unregister_link_state_cb(ctx->link_notify_handle);
		glink_close(ctx->chan);
	}
	ctx->chan = NULL;
	mutex_unlock(&me->smd_mutex);
	cid = ctx - &gcinfo[0];
=======
	struct fastrpc_chan_ctx *ctx;
	int cid;

	ctx = container_of(kref, struct fastrpc_chan_ctx, kref);
	smd_close(ctx->chan);
	ctx->chan = NULL;
	mutex_unlock(&me->smd_mutex);
	cid = ctx - &me->channel[0];
>>>>>>> p9x
	pr_info("'closed /dev/%s c %d %d'\n", gcinfo[cid].name,
						MAJOR(me->dev_no), cid);
}

static void fastrpc_context_list_dtor(struct fastrpc_file *fl);

static int fastrpc_file_free(struct fastrpc_file *fl)
{
	struct hlist_node *n;
	struct fastrpc_mmap *map = NULL;
	int cid;

	if (!fl)
		return 0;
	cid = fl->cid;

	spin_lock(&fl->apps->hlock);
	hlist_del_init(&fl->hn);
	spin_unlock(&fl->apps->hlock);

<<<<<<< HEAD
	if (!fl->sctx)
		goto bail;

	(void)fastrpc_release_current_dsp_process(fl);
	if (!IS_ERR_OR_NULL(fl->init_mem))
		fastrpc_buf_free(fl->init_mem, 0);
	fastrpc_context_list_dtor(fl);
	fastrpc_cached_buf_list_free(fl);
=======
	(void)fastrpc_release_current_dsp_process(fl);
	fastrpc_context_list_dtor(fl);
	fastrpc_buf_list_free(fl);
>>>>>>> p9x
	hlist_for_each_entry_safe(map, n, &fl->maps, hn) {
		fastrpc_mmap_free(map);
	}
	if (fl->ssrcount == fl->apps->channel[cid].ssrcount)
		kref_put_mutex(&fl->apps->channel[cid].kref,
				fastrpc_channel_close, &fl->apps->smd_mutex);
<<<<<<< HEAD
	mutex_destroy(&fl->map_mutex);
bail:
	fastrpc_remote_buf_list_free(fl);
	kfree(fl);
	return 0;
}

static int fastrpc_session_alloc(struct fastrpc_channel_ctx *chan, int *session)
{
	struct fastrpc_apps *me = &gfa;
	int idx = 0, err = 0;

	switch (chan->channel) {
	case SMD_APPS_QDSP:
		idx = ffz(chan->bitmap);
		VERIFY(err, idx < chan->sesscount);
		if (err)
			goto bail;
		set_bit(idx, &chan->bitmap);
		break;
	case SMD_APPS_DSPS:
		VERIFY(err, me->dev != NULL);
		if (err)
			goto bail;
		chan->session[0].dev = me->dev;
		break;
	case SMD_APPS_MODEM:
		VERIFY(err, me->modem_cma_dev != NULL);
		if (err)
			goto bail;
		chan->session[0].dev = me->modem_cma_dev;
		break;
	}

	chan->session[idx].smmu.faults = 0;
	*session = idx;
 bail:
	return err;
}

static int fastrpc_session_free(struct fastrpc_channel_ctx *chan, int session)
{
	int err = 0;

	if (chan->sesscount) {
		VERIFY(err, session < chan->sesscount);
		if (err)
			goto bail;
		clear_bit(session, &chan->bitmap);
	}
 bail:
	return err;
}

static bool fastrpc_glink_notify_rx_intent_req(void *h, const void *priv,
						size_t size)
{
	if (0 != glink_queue_rx_intent(h, NULL, size))
		return false;
	return true;
}

static void fastrpc_glink_notify_tx_done(void *handle, const void *priv,
		const void *pkt_priv, const void *ptr)
{
}

static void fastrpc_glink_notify_rx(void *handle, const void *priv,
	const void *pkt_priv, const void *ptr, size_t size)
{
	struct smq_invoke_rsp *rsp = (struct smq_invoke_rsp *)ptr;
	struct fastrpc_apps *me = &gfa;
	uint32_t index;
	int err = 0;

	VERIFY(err, (rsp && size >= sizeof(*rsp)));
	if (err)
		goto bail;

	index = (uint32_t)((rsp->ctx & FASTRPC_CTXID_MASK) >> 4);
	VERIFY(err, index < FASTRPC_CTX_MAX);
	if (err)
		goto bail;

	VERIFY(err, !IS_ERR_OR_NULL(me->ctxtable[index]));
	if (err)
		goto bail;

	VERIFY(err, ((me->ctxtable[index]->ctxid == (rsp->ctx)) &&
		me->ctxtable[index]->magic == FASTRPC_CTX_MAGIC));
	if (err)
		goto bail;

	context_notify_user(me->ctxtable[index], rsp->retval);
bail:
	if (err)
		pr_err("adsprpc: invalid response or context\n");
	glink_rx_done(handle, ptr, true);
}

static void fastrpc_glink_notify_state(void *handle, const void *priv,
				unsigned int event)
{
	struct fastrpc_apps *me = &gfa;
	int cid = (int)(uintptr_t)priv;

	switch (event) {
	case GLINK_CONNECTED:
		complete(&me->channel[cid].work);
		break;
	case GLINK_LOCAL_DISCONNECTED:
		break;
	case GLINK_REMOTE_DISCONNECTED:
		fastrpc_notify_drivers(me, cid);
		break;
	}
}

static int fastrpc_device_release(struct inode *inode, struct file *file)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_file *fl = (struct fastrpc_file *)file->private_data;
	struct fastrpc_fl *pfl = NULL;
	int session, cid;
	int err = 0;

	VERIFY(err, pfl = kzalloc(sizeof(*pfl), GFP_KERNEL));
	if (err)
		goto bail;
	INIT_HLIST_NODE(&pfl->hn);
	if (fl) {
		cid = fl->cid;
		if (fl->sctx) {
			session = fl->sctx - &me->channel[cid].session[0];
			fastrpc_session_free(&me->channel[cid], session);
		}
		mutex_lock(&me->flfree_mutex);
		pfl->fl = fl;
		hlist_add_head(&pfl->hn, &me->fls);
		VERIFY(err, me->wq);
		if (err) {
			hlist_del_init(&pfl->hn);
			mutex_unlock(&me->flfree_mutex);
			goto bail;
		}
		if (!work_busy(&me->free_work)) {
			INIT_WORK(&me->free_work, file_free_work_handler);
			VERIFY(err, queue_work(me->wq, &me->free_work));
			if (err) {
				hlist_del_init(&pfl->hn);
				mutex_unlock(&me->flfree_mutex);
				goto bail;
			}
		}
		me->pending_free++;
		mutex_unlock(&me->flfree_mutex);
		file->private_data = NULL;
	}
bail:
	if (err) {
		fastrpc_file_free(fl);
		kfree(pfl);
	}
	return err;
}


static void file_free_work_handler(struct work_struct *w)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_fl *fl = NULL, *freefl = NULL;
	struct hlist_node *n = NULL;

	while (1) {
		mutex_lock(&me->flfree_mutex);
		hlist_for_each_entry_safe(fl, n, &me->fls, hn) {
			hlist_del_init(&fl->hn);
			freefl = fl;
			break;
		}
		mutex_unlock(&me->flfree_mutex);
		if (freefl) {
			fastrpc_file_free(freefl->fl);
			kfree(freefl);
		}
		mutex_lock(&me->flfree_mutex);

		if (hlist_empty(&me->fls)) {
			me->pending_free = 0;
			wake_up_interruptible_all(&wait_queue);
			mutex_unlock(&me->flfree_mutex);
			break;
		}
		mutex_unlock(&me->flfree_mutex);
	}
	return;
}

static void fastrpc_glink_register_cb(struct glink_link_state_cb_info *cb_info,
					 void *priv)
{
	switch (cb_info->link_state) {
	case GLINK_LINK_STATE_UP:
		if (priv)
			complete(priv);
		break;
	case GLINK_LINK_STATE_DOWN:
		break;
	default:
		pr_err("adsprpc: unknown glnk state %d\n", cb_info->link_state);
		break;
	}
}

static int fastrpc_glink_open(int cid, struct fastrpc_apps *me)
{
	int err = 0;
	struct glink_open_config *cfg = &me->channel[cid].cfg;
	struct glink_link_info *link_info = &me->channel[cid].link_info;

	link_info->edge = gcinfo[cid].edge;
	link_info->transport = "smem";
	link_info->glink_link_state_notif_cb = fastrpc_glink_register_cb;
	me->channel[cid].link_notify_handle = glink_register_link_state_cb(
					&me->channel[cid].link_info,
					(void *)(&me->channel[cid].work));
	VERIFY(err, !IS_ERR_OR_NULL(me->channel[cid].link_notify_handle));
	if (err)
		goto bail;

	VERIFY(err, wait_for_completion_timeout(&me->channel[cid].work,
						RPC_TIMEOUT));
	if (err)
		goto bail;

	cfg->priv = (void *)(uintptr_t)cid;
	cfg->edge = gcinfo[cid].edge;
	cfg->name = FASTRPC_GLINK_GUID;
	cfg->notify_rx = fastrpc_glink_notify_rx;
	cfg->notify_tx_done = fastrpc_glink_notify_tx_done;
	cfg->notify_state = fastrpc_glink_notify_state;
	cfg->notify_rx_intent_req = fastrpc_glink_notify_rx_intent_req;
	VERIFY(err, 0 != (me->channel[cid].chan = glink_open(cfg)));
bail:
	return err;
=======
	kfree(fl);
	return 0;
}
static int fastrpc_device_release(struct inode *inode, struct file *file)
{
	fastrpc_file_free((struct fastrpc_file *)file->private_data);
	file->private_data = NULL;
	return 0;
>>>>>>> p9x
}

static int fastrpc_device_open(struct inode *inode, struct file *filp)
{
	int cid = MINOR(inode->i_rdev);
<<<<<<< HEAD
	int err = 0, session;
	int event;
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_file *fl = NULL;

	if (me->pending_free) {
		event = wait_event_interruptible_timeout(wait_queue,
				!me->pending_free, OPEN_TIMEOUT);
		if (event == 0)
			pr_err("fastrpc:timed out..list is still not empty\n");
	}

=======
	int err = 0;
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_file *fl = NULL;

>>>>>>> p9x
	VERIFY(err, fl = kzalloc(sizeof(*fl), GFP_KERNEL));
	if (err)
		return err;

	filp->private_data = fl;

	mutex_lock(&me->smd_mutex);

	context_list_ctor(&fl->clst);
	spin_lock_init(&fl->hlock);
	INIT_HLIST_HEAD(&fl->maps);
<<<<<<< HEAD
	INIT_HLIST_HEAD(&fl->cached_bufs);
	INIT_HLIST_HEAD(&fl->remote_bufs);
	INIT_HLIST_NODE(&fl->hn);
	fl->tgid = current->tgid;
	fl->apps = me;
	fl->cid = cid;
	fl->init_mem = NULL;

	VERIFY(err, !fastrpc_session_alloc(&me->channel[cid], &session));
	if (err)
		goto bail;
	fl->sctx = &me->channel[cid].session[session];
=======
	INIT_HLIST_HEAD(&fl->bufs);
	INIT_HLIST_NODE(&fl->hn);
	fl->chan = &me->channel[cid];
	fl->cid = cid;
	fl->tgid = current->tgid;
	fl->apps = me;
>>>>>>> p9x

	fl->ssrcount = me->channel[cid].ssrcount;
	if ((kref_get_unless_zero(&me->channel[cid].kref) == 0) ||
	    (me->channel[cid].chan == NULL)) {
<<<<<<< HEAD
		if (me->glink) {
			VERIFY(err, 0 == fastrpc_glink_open(cid, me));
		} else {
			VERIFY(err, !smd_named_open_on_edge(FASTRPC_SMD_GUID,
				    gcinfo[cid].channel,
				    (smd_channel_t **)&me->channel[cid].chan,
				    (void *)(uintptr_t)cid,
				    smd_event_handler));
		}
		if (err)
			goto bail;

		VERIFY(err, wait_for_completion_timeout(&me->channel[cid].work,
							RPC_TIMEOUT));
		if (err) {
			me->channel[cid].chan = NULL;
			goto bail;
		}
		kref_init(&me->channel[cid].kref);
		pr_info("'opened /dev/%s c %d %d'\n", gcinfo[cid].name,
						MAJOR(me->dev_no), cid);
		if (me->channel[cid].ssrcount !=
				 me->channel[cid].prevssrcount) {
			if (fastrpc_mmap_remove_ssr(fl))
				pr_err("ADSPRPC: SSR: Failed to unmap remote heap\n");
			me->channel[cid].prevssrcount =
						me->channel[cid].ssrcount;
		}
	}
	mutex_init(&fl->map_mutex);
=======
		VERIFY(err, !smd_named_open_on_edge(FASTRPC_SMD_GUID,
						    gcinfo[cid].channel,
						    &me->channel[cid].chan,
						    (void *)(uintptr_t)cid,
						    smd_event_handler));
		if (err)
			goto bail;
		VERIFY(err, wait_for_completion_timeout(&me->channel[cid].work,
							 RPC_TIMEOUT));
		if (err)
			goto bail;
		kref_init(&me->channel[cid].kref);
		pr_info("'opened /dev/%s c %d %d'\n", gcinfo[cid].name,
						MAJOR(me->dev_no), cid);
		if (fastrpc_mmap_remove_ssr(fl))
			pr_err("ADSPRPC: SSR: Failed to unmap remote heap\n");
	}
>>>>>>> p9x
	spin_lock(&me->hlock);
	hlist_add_head(&fl->hn, &me->drivers);
	spin_unlock(&me->hlock);

bail:
	mutex_unlock(&me->smd_mutex);

	if (err && fl)
		fastrpc_device_release(inode, filp);
	return err;
}

<<<<<<< HEAD
static int fastrpc_get_info(struct fastrpc_file *fl, uint32_t *info)
{
	int err = 0;

	VERIFY(err, fl && fl->sctx);
	if (err)
		goto bail;
	if (fl->sctx)
		*info = (fl->sctx->smmu.enabled ? 1 : 0);
bail:
	return err;
}

static int fastrpc_internal_control(struct fastrpc_file *fl,
					struct fastrpc_ioctl_control *cp)
{
	int err = 0;

	VERIFY(err, !IS_ERR_OR_NULL(fl) && !IS_ERR_OR_NULL(fl->apps));
	if (err)
		goto bail;
	VERIFY(err, !IS_ERR_OR_NULL(cp));
	if (err)
		goto bail;

	switch (cp->req) {
	case FASTRPC_CONTROL_KALLOC:
		cp->kalloc.kalloc_support = 1;
		break;
	default:
		err = -ENOTTY;
		break;
	}
bail:
	return err;
}

=======
>>>>>>> p9x
static long fastrpc_device_ioctl(struct file *file, unsigned int ioctl_num,
				 unsigned long ioctl_param)
{
	union {
		struct fastrpc_ioctl_invoke_fd invokefd;
		struct fastrpc_ioctl_mmap mmap;
<<<<<<< HEAD
		struct fastrpc_ioctl_mmap_64 mmap64;
		struct fastrpc_ioctl_munmap munmap;
		struct fastrpc_ioctl_munmap_64 munmap64;
		struct fastrpc_ioctl_init init;
		struct fastrpc_ioctl_control cp;
	} p;
	union {
		struct fastrpc_ioctl_mmap mmap;
		struct fastrpc_ioctl_munmap munmap;
	} i;
	void *param = (char *)ioctl_param;
	struct fastrpc_file *fl = (struct fastrpc_file *)file->private_data;
	int size = 0, err = 0;
	uint32_t info;
=======
		struct fastrpc_ioctl_munmap munmap;
		struct fastrpc_ioctl_init init;
	} p;
	void *param = (char *)ioctl_param;
	struct fastrpc_file *fl = (struct fastrpc_file *)file->private_data;
	int size = 0, err = 0;
>>>>>>> p9x

	switch (ioctl_num) {
	case FASTRPC_IOCTL_INVOKE_FD:
	case FASTRPC_IOCTL_INVOKE:
		p.invokefd.fds = 0;
		size = (ioctl_num == FASTRPC_IOCTL_INVOKE) ?
				sizeof(p.invokefd.inv) : sizeof(p.invokefd);
		K_COPY_FROM_USER(err, 0, &p.invokefd, param, size);
		if (err)
			goto bail;
		VERIFY(err, 0 == (err = fastrpc_internal_invoke(fl, fl->mode,
						0, &p.invokefd)));
		if (err)
			goto bail;
		break;
	case FASTRPC_IOCTL_MMAP:
		K_COPY_FROM_USER(err, 0, &p.mmap, param,
						sizeof(p.mmap));
		if (err)
			goto bail;
		VERIFY(err, 0 == (err = fastrpc_internal_mmap(fl, &p.mmap)));
		if (err)
			goto bail;
		K_COPY_TO_USER(err, 0, param, &p.mmap, sizeof(p.mmap));
		if (err)
			goto bail;
		break;
	case FASTRPC_IOCTL_MUNMAP:
		K_COPY_FROM_USER(err, 0, &p.munmap, param,
						sizeof(p.munmap));
		if (err)
			goto bail;
		VERIFY(err, 0 == (err = fastrpc_internal_munmap(fl,
							&p.munmap)));
		if (err)
			goto bail;
		break;
<<<<<<< HEAD
	case FASTRPC_IOCTL_MMAP_64:
		K_COPY_FROM_USER(err, 0, &p.mmap64, param,
						sizeof(p.mmap64));
		if (err)
			goto bail;
		get_fastrpc_ioctl_mmap_64(&p.mmap64, &i.mmap);
		VERIFY(err, 0 == (err = fastrpc_internal_mmap(fl, &i.mmap)));
		if (err)
			goto bail;
		put_fastrpc_ioctl_mmap_64(&p.mmap64, &i.mmap);
		K_COPY_TO_USER(err, 0, param, &p.mmap64, sizeof(p.mmap64));
		if (err)
			goto bail;
		break;
	case FASTRPC_IOCTL_MUNMAP_64:
		K_COPY_FROM_USER(err, 0, &p.munmap64, param,
						sizeof(p.munmap64));
		if (err)
			goto bail;
		get_fastrpc_ioctl_munmap_64(&p.munmap64, &i.munmap);
		VERIFY(err, 0 == (err = fastrpc_internal_munmap(fl,
							&i.munmap)));
		if (err)
			goto bail;
		break;
=======
>>>>>>> p9x
	case FASTRPC_IOCTL_SETMODE:
		switch ((uint32_t)ioctl_param) {
		case FASTRPC_MODE_PARALLEL:
		case FASTRPC_MODE_SERIAL:
			fl->mode = (uint32_t)ioctl_param;
			break;
		default:
			err = -ENOTTY;
			break;
		}
		break;
<<<<<<< HEAD
	case FASTRPC_IOCTL_CONTROL:
		K_COPY_FROM_USER(err, 0, &p.cp, param,
				sizeof(p.cp));
		if (err)
			goto bail;
		VERIFY(err, 0 == (err = fastrpc_internal_control(fl, &p.cp)));
		if (err)
			goto bail;
		if (p.cp.req == FASTRPC_CONTROL_KALLOC) {
			K_COPY_TO_USER(err, 0, param, &p.cp, sizeof(p.cp));
			if (err)
				goto bail;
		}
		break;
	case FASTRPC_IOCTL_GETINFO:
		K_COPY_FROM_USER(err, 0, &info, param, sizeof(info));
		if (err)
			goto bail;
		VERIFY(err, 0 == (err = fastrpc_get_info(fl, &info)));
		if (err)
			goto bail;
		K_COPY_TO_USER(err, 0, param, &info, sizeof(info));
		if (err)
			goto bail;
		break;
	case FASTRPC_IOCTL_INIT:
		VERIFY(err, 0 == copy_from_user(&p.init, param,
						sizeof(p.init)));
=======
	case FASTRPC_IOCTL_INIT:
		K_COPY_FROM_USER(err, 0, &p.init, param, sizeof(p.init));
>>>>>>> p9x
		if (err)
			goto bail;
		VERIFY(err, 0 == fastrpc_init_process(fl, &p.init));
		if (err)
			goto bail;
		break;

	default:
		err = -ENOTTY;
		break;
	}
 bail:
	return err;
}

static int fastrpc_restart_notifier_cb(struct notifier_block *nb,
					unsigned long code,
					void *data)
{
	struct fastrpc_apps *me = &gfa;
<<<<<<< HEAD
	struct fastrpc_channel_ctx *ctx;
	struct notif_data *notifdata = data;
	int cid;

	ctx = container_of(nb, struct fastrpc_channel_ctx, nb);
=======
	struct fastrpc_chan_ctx *ctx;
	int cid;

	ctx = container_of(nb, struct fastrpc_chan_ctx, nb);
>>>>>>> p9x
	cid = ctx - &me->channel[0];
	if (code == SUBSYS_BEFORE_SHUTDOWN) {
		mutex_lock(&me->smd_mutex);
		ctx->ssrcount++;
		if (ctx->chan) {
<<<<<<< HEAD
			if (me->glink) {
				glink_unregister_link_state_cb(
					ctx->link_notify_handle);
				glink_close(ctx->chan);
			} else {
				smd_close(ctx->chan);
			}
			ctx->chan = NULL;
			pr_info("'restart notifier: closed /dev/%s c %d %d'\n",
				 gcinfo[cid].name, MAJOR(me->dev_no), cid);
		}
		mutex_unlock(&me->smd_mutex);
		fastrpc_notify_drivers(me, cid);
	} else if (code == SUBSYS_RAMDUMP_NOTIFICATION) {
		if (me->channel[0].remoteheap_ramdump_dev &&
				notifdata->enable_ramdump) {
			me->channel[0].ramdumpenabled = 1;
		}
=======
			smd_close(ctx->chan);
			ctx->chan = NULL;
			pr_info("'closed /dev/%s c %d %d'\n", gcinfo[cid].name,
						MAJOR(me->dev_no), cid);
		}
		mutex_unlock(&me->smd_mutex);
		fastrpc_notify_drivers(me, cid);
>>>>>>> p9x
	}

	return NOTIFY_DONE;
}

<<<<<<< HEAD
static int fastrpc_smmu_fault_handler(struct iommu_domain *domain,
	struct device *dev, unsigned long iova, int flags, void *token)
{
	struct fastrpc_session_ctx *sess = (struct fastrpc_session_ctx *)token;
	int err = 0;

	VERIFY(err, sess != NULL);
	if (err)
		return err;
	sess->smmu.faults++;
	dev_err(dev, "ADSPRPC context fault: iova=0x%08lx, cb = %d, faults=%d",
					iova, sess->smmu.cb, sess->smmu.faults);
	return 0;
}

=======
>>>>>>> p9x
static const struct file_operations fops = {
	.open = fastrpc_device_open,
	.release = fastrpc_device_release,
	.unlocked_ioctl = fastrpc_device_ioctl,
	.compat_ioctl = compat_fastrpc_device_ioctl,
};

<<<<<<< HEAD
static struct of_device_id fastrpc_match_table[] = {
	{ .compatible = "qcom,msm-fastrpc-adsp", },
	{ .compatible = "qcom,msm-fastrpc-compute-cb", },
	{ .compatible = "qcom,msm-fastrpc-legacy-compute-cb", },
	{ .compatible = "qcom,msm-adsprpc-mem-region", },
	{ .compatible = "qcom,msm-mdsprpc-mem-region", },
	{}
};

static int fastrpc_cb_probe(struct device *dev)
{
	struct fastrpc_channel_ctx *chan;
	struct fastrpc_session_ctx *sess;
	struct of_phandle_args iommuspec;
	const char *name;
	int err = 0, i;
	int disable_htw = 1;

	VERIFY(err, NULL != (name = of_get_property(dev->of_node,
					 "label", NULL)));
	if (err)
		goto bail;
	for (i = 0; i < NUM_CHANNELS; i++) {
		if (!gcinfo[i].name)
			continue;
		if (!strcmp(name, gcinfo[i].name))
			break;
	}
	VERIFY(err, i < NUM_CHANNELS);
	if (err)
		goto bail;
	chan = &gcinfo[i];
	VERIFY(err, chan->sesscount < NUM_SESSIONS);
	if (err)
		goto bail;

	VERIFY(err, !of_parse_phandle_with_args(dev->of_node, "iommus",
						"#iommu-cells", 0, &iommuspec));
	if (err)
		goto bail;
	sess = &chan->session[chan->sesscount];
	sess->smmu.cb = iommuspec.args[0];
	VERIFY(err, !IS_ERR_OR_NULL(sess->smmu.mapping =
				arm_iommu_create_mapping(&platform_bus_type,
						0x80000000, 0x7fffffff)));
	if (err)
		goto bail;
	iommu_domain_set_attr(sess->smmu.mapping->domain,
				DOMAIN_ATTR_COHERENT_HTW_DISABLE,
				&disable_htw);
	iommu_set_fault_handler(sess->smmu.mapping->domain,
				fastrpc_smmu_fault_handler, sess);
	VERIFY(err, !arm_iommu_attach_device(dev, sess->smmu.mapping));
	if (err)
		goto bail;
	sess->dev = dev;
	sess->smmu.enabled = 1;
	chan->sesscount++;
bail:
	return err;
}

static int fastrpc_cb_legacy_probe(struct device *dev)
{
	struct device_node *domains_child_node = NULL;
	struct device_node *ctx_node = NULL;
	struct fastrpc_channel_ctx *chan = NULL;
	struct fastrpc_session_ctx *first_sess = NULL, *sess = NULL;
	const char *name = NULL;
	unsigned int *range = NULL, range_size = 0;
	unsigned int *sids = NULL, sids_size = 0;
	int err = 0, ret = 0, i;
	int disable_htw = 1;

	VERIFY(err, 0 != (domains_child_node = of_get_child_by_name(
			dev->of_node,
			"qcom,msm_fastrpc_compute_cb")));
	if (err)
		goto bail;
	VERIFY(err, 0 != (ctx_node = of_parse_phandle(
			domains_child_node,
			"qcom,adsp-shared-phandle", 0)));
	if (err)
		goto bail;
	VERIFY(err, 0 != of_get_property(domains_child_node,
				"qcom,adsp-shared-sids", &sids_size));
	if (err)
		goto bail;
	VERIFY(err, sids = kzalloc(sids_size, GFP_KERNEL));
	if (err)
		goto bail;
	ret = of_property_read_u32_array(domains_child_node,
					"qcom,adsp-shared-sids",
					sids,
					sids_size/sizeof(unsigned int));
	if (ret)
		goto bail;
	VERIFY(err, 0 != (name = of_get_property(ctx_node, "label", NULL)));
	if (err)
		goto bail;
	VERIFY(err, 0 != of_get_property(domains_child_node,
					"qcom,virtual-addr-pool", &range_size));
	if (err)
		goto bail;
	VERIFY(err, range = kzalloc(range_size, GFP_KERNEL));
	if (err)
		goto bail;
	ret = of_property_read_u32_array(domains_child_node,
					"qcom,virtual-addr-pool",
					range,
					range_size/sizeof(unsigned int));
	if (ret)
		goto bail;

	chan = &gcinfo[0];
	VERIFY(err, chan->sesscount < NUM_SESSIONS);
	if (err)
		goto bail;
	first_sess = &chan->session[chan->sesscount];
	first_sess->dev = msm_iommu_get_ctx(name);
	VERIFY(err, !IS_ERR_OR_NULL(first_sess->smmu.mapping =
			arm_iommu_create_mapping(
				msm_iommu_get_bus(first_sess->dev),
				range[0], range[1])));
	if (err)
		goto bail;
	iommu_domain_set_attr(first_sess->smmu.mapping->domain,
			DOMAIN_ATTR_COHERENT_HTW_DISABLE,
			&disable_htw);
	VERIFY(err, !arm_iommu_attach_device(first_sess->dev,
					first_sess->smmu.mapping));
	if (err)
		goto bail;
	VERIFY(err, (sids_size/sizeof(unsigned int)) <= NUM_SESSIONS);
	if (err)
		goto bail;
	for (i = 0; i < sids_size/sizeof(unsigned int); i++) {
		VERIFY(err, chan->sesscount < NUM_SESSIONS);
		if (err)
			goto bail;
		sess = &chan->session[chan->sesscount];
		sess->smmu.cb = sids[i];
		sess->dev = first_sess->dev;
		sess->smmu.enabled = 1;
		sess->smmu.mapping = first_sess->smmu.mapping;
		chan->sesscount++;
	}
bail:
	kfree(sids);
	kfree(range);
	return err;
}

static int fastrpc_probe(struct platform_device *pdev)
{
	int err = 0;
	struct fastrpc_apps *me = &gfa;
	struct device *dev = &pdev->dev;
	struct smq_phy_page range;
	struct device_node *ion_node, *node;
	struct platform_device *ion_pdev;
	struct cma *cma;
	uint32_t val;

	if (of_device_is_compatible(dev->of_node,
					"qcom,msm-fastrpc-compute-cb"))
		return fastrpc_cb_probe(dev);

	if (of_device_is_compatible(dev->of_node,
					"qcom,msm-fastrpc-legacy-compute-cb"))
		return fastrpc_cb_legacy_probe(dev);

	if (of_device_is_compatible(dev->of_node,
					"qcom,msm-adsprpc-mem-region")) {
		me->dev = dev;
		me->channel[0].remoteheap_ramdump_dev =
				create_ramdump_device("adsp_rh", dev);
		if (IS_ERR_OR_NULL(me->channel[0].remoteheap_ramdump_dev)) {
			pr_err("ADSPRPC: Unable to create adsp-remoteheap ramdump device.\n");
			me->channel[0].remoteheap_ramdump_dev = NULL;
		}
		return 0;
	}

	if (of_device_is_compatible(dev->of_node,
					"qcom,msm-mdsprpc-mem-region")) {
		me->modem_cma_dev = dev;
		range.addr = 0;
		ion_node = of_find_compatible_node(NULL, NULL, "qcom,msm-ion");
		if (ion_node) {
			for_each_available_child_of_node(ion_node, node) {
				if (of_property_read_u32(node, "reg", &val))
					continue;
				if (val != ION_ADSP_HEAP_ID)
					continue;
				ion_pdev = of_find_device_by_node(node);
				if (!ion_pdev)
					break;
				cma = dev_get_cma_area(&ion_pdev->dev);
				if (cma) {
					range.addr = cma_get_base(cma);
					range.size = (size_t)cma_get_size(cma);
				}
				break;
			}
		}
		if (range.addr) {
			int srcVM[1] = {VMID_HLOS};
			int destVM[4] = {VMID_HLOS, VMID_MSS_MSA, VMID_SSC_Q6,
					VMID_ADSP_Q6};
			int destVMperm[4] = {PERM_READ | PERM_WRITE | PERM_EXEC,
					PERM_READ | PERM_WRITE | PERM_EXEC,
					PERM_READ | PERM_WRITE | PERM_EXEC,
					PERM_READ | PERM_WRITE | PERM_EXEC,
					};
			VERIFY(err, !hyp_assign_phys(range.addr, range.size,
					srcVM, 1, destVM, destVMperm, 4));
			if (err)
				goto bail;
		}
		return 0;
	}

	me->glink = of_property_read_bool(dev->of_node, "qcom,fastrpc-glink");
	pr_info("adsprpc: channel link type: %d\n", me->glink);

	VERIFY(err, !of_platform_populate(pdev->dev.of_node,
					  fastrpc_match_table,
					  NULL, &pdev->dev));
	if (err)
		goto bail;
bail:
	return err;
}

static void fastrpc_deinit(void)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_channel_ctx *chan = gcinfo;
	int i, j;

	for (i = 0; i < NUM_CHANNELS; i++, chan++) {
		if (chan->chan) {
			kref_put_mutex(&chan->kref,
				fastrpc_channel_close, &me->smd_mutex);
			chan->chan = NULL;
		}
		for (j = 0; j < NUM_SESSIONS; j++) {
			struct fastrpc_session_ctx *sess = &chan->session[j];

			if (sess->smmu.enabled) {
				arm_iommu_detach_device(sess->dev);
				sess->dev = NULL;
			}
			if (sess->smmu.mapping) {
				arm_iommu_release_mapping(sess->smmu.mapping);
				sess->smmu.mapping = NULL;
			}
		}
	}
}

static struct platform_driver fastrpc_driver = {
	.probe = fastrpc_probe,
	.driver = {
		.name = "fastrpc",
		.owner = THIS_MODULE,
		.of_match_table = fastrpc_match_table,
=======
static int adsp_mem_driver_probe(struct platform_device *pdev)
{
	struct fastrpc_apps *me = &gfa;
	struct device *dev = &pdev->dev;

	if (of_device_is_compatible(dev->of_node,
					"qcom,msm-adsprpc-mem-region")) {
		me->adsp_mem_device = dev;
		return 0;
	}
	return -EINVAL;
}

static struct of_device_id adsp_mem_match_table[] = {
	{ .compatible = "qcom,msm-adsprpc-mem-region" },
	{}
};

static struct platform_driver adsp_memory_driver = {
	.probe = adsp_mem_driver_probe,
	.driver = {
		.name = "msm-adsprpc-mem-region",
		.of_match_table = adsp_mem_match_table,
		.owner = THIS_MODULE,
>>>>>>> p9x
	},
};

static int __init fastrpc_device_init(void)
{
	struct fastrpc_apps *me = &gfa;
<<<<<<< HEAD
=======
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;
	struct iommu_group *group = NULL;
	struct iommu_domain *domain = NULL;
>>>>>>> p9x
	int err = 0, i;

	memset(me, 0, sizeof(*me));

	fastrpc_init(me);
<<<<<<< HEAD
	mutex_init(&me->flfree_mutex);
	me->wq = create_singlethread_workqueue("FILE_FREE");
	INIT_WORK(&me->free_work, file_free_work_handler);

	me->dev = NULL;
	VERIFY(err, 0 == platform_driver_register(&fastrpc_driver));
	if (err)
		goto register_bail;
=======
>>>>>>> p9x
	VERIFY(err, 0 == alloc_chrdev_region(&me->dev_no, 0, NUM_CHANNELS,
					DEVICE_NAME));
	if (err)
		goto alloc_chrdev_bail;
	cdev_init(&me->cdev, &fops);
	me->cdev.owner = THIS_MODULE;
	VERIFY(err, 0 == cdev_add(&me->cdev, MKDEV(MAJOR(me->dev_no), 0),
				NUM_CHANNELS));
	if (err)
		goto cdev_init_bail;
	me->class = class_create(THIS_MODULE, "fastrpc");
	VERIFY(err, !IS_ERR(me->class));
	if (err)
		goto class_create_bail;
	me->compat = (NULL == fops.compat_ioctl) ? 0 : 1;
	for (i = 0; i < NUM_CHANNELS; i++) {
<<<<<<< HEAD
		if (!gcinfo[i].name)
			continue;
=======
>>>>>>> p9x
		me->channel[i].dev = device_create(me->class, NULL,
					MKDEV(MAJOR(me->dev_no), i),
					NULL, gcinfo[i].name);
		VERIFY(err, !IS_ERR(me->channel[i].dev));
		if (err)
			goto device_create_bail;
		me->channel[i].ssrcount = 0;
<<<<<<< HEAD
		me->channel[i].prevssrcount = 0;
		me->channel[i].ramdumpenabled = 0;
		me->channel[i].remoteheap_ramdump_dev = NULL;
		me->channel[i].nb.notifier_call = fastrpc_restart_notifier_cb;
		me->channel[i].handle = subsys_notif_register_notifier(
							gcinfo[i].subsys,
							&me->channel[i].nb);
	}

	me->client = msm_ion_client_create(DEVICE_NAME);
	VERIFY(err, !IS_ERR_OR_NULL(me->client));
	if (err)
		goto device_create_bail;
	return 0;
device_create_bail:
	for (i = 0; i < NUM_CHANNELS; i++) {
		if (IS_ERR_OR_NULL(me->channel[i].dev))
			continue;
		device_destroy(me->class, MKDEV(MAJOR(me->dev_no), i));
		subsys_notif_unregister_notifier(me->channel[i].handle,
						&me->channel[i].nb);
	}
=======
		me->channel[i].nb.notifier_call = fastrpc_restart_notifier_cb,
		(void)subsys_notif_register_notifier(gcinfo[i].subsys,
							&me->channel[i].nb);
		if (!gcinfo[i].node)
			continue;
		node = of_find_compatible_node(NULL, NULL, gcinfo[i].node);
		if (node) {
			pdev = of_find_device_by_node(node);
			if (pdev) {
				me->channel[i].smmu.dev = &pdev->dev;
				me->channel[i].smmu.enabled = 1;
				me->channel[i].smmu.cb = 1;
				dev_dbg(me->channel[i].dev,
					"%s: Using audio Context bank\n",
					__func__);
			}
		}
	}
	group = iommu_group_find("lpass_audio");
	if (!group) {
		pr_debug("Failed to find group lpass_audio deferred\n");
		err = -1;
		goto register_bail;
	}
	domain = iommu_group_get_iommudata(group);
	if (IS_ERR_OR_NULL(domain)) {
		pr_err("Failed to get domain data for group %p\n",
				group);
		err = -1;
		goto register_bail;
	}
	me->domain_id = msm_find_domain_no(domain);
	if (me->domain_id < 0) {
		pr_err("Failed to get domain index for domain %p\n",
				domain);
		err = -1;
		goto register_bail;
	}
	pr_debug("domain=%p, domain_id=%d, group=%p\n", domain,
			me->domain_id, group);
	err = platform_driver_register(&adsp_memory_driver);
	if (err) {
		pr_err("ADSPRPC: Failed to register adsp memory driver");
		goto register_bail;
	}

	return 0;

device_create_bail:
>>>>>>> p9x
	class_destroy(me->class);
class_create_bail:
	cdev_del(&me->cdev);
cdev_init_bail:
	unregister_chrdev_region(me->dev_no, NUM_CHANNELS);
alloc_chrdev_bail:
<<<<<<< HEAD
register_bail:
	fastrpc_deinit();
=======
	fastrpc_deinit();
register_bail:
>>>>>>> p9x
	return err;
}

static void __exit fastrpc_device_exit(void)
{
	struct fastrpc_apps *me = &gfa;
	int i;

	fastrpc_file_list_dtor(me);
	fastrpc_deinit();
<<<<<<< HEAD
	if (me->wq) {
		flush_workqueue(me->wq);
		destroy_workqueue(me->wq);
	}
	for (i = 0; i < NUM_CHANNELS; i++) {
		if (!gcinfo[i].name)
			continue;
		device_destroy(me->class, MKDEV(MAJOR(me->dev_no), i));
		subsys_notif_unregister_notifier(me->channel[i].handle,
=======
	for (i = 0; i < NUM_CHANNELS; i++) {
		device_destroy(me->class, MKDEV(MAJOR(me->dev_no), i));
		subsys_notif_unregister_notifier(gcinfo[i].subsys,
>>>>>>> p9x
						&me->channel[i].nb);
	}
	class_destroy(me->class);
	cdev_del(&me->cdev);
	unregister_chrdev_region(me->dev_no, NUM_CHANNELS);
<<<<<<< HEAD
	ion_client_destroy(me->client);
=======
>>>>>>> p9x
}

late_initcall(fastrpc_device_init);
module_exit(fastrpc_device_exit);

MODULE_LICENSE("GPL v2");
