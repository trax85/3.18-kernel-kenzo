<<<<<<< HEAD
/* Copyright (c) 2011-2018, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2011-2016, The Linux Foundation. All rights reserved.
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
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>
#include <linux/slab.h>
#include <linux/iommu.h>
#include <linux/msm_kgsl.h>
<<<<<<< HEAD
#include <linux/ratelimit.h>
#include <linux/of_platform.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/secure_buffer.h>
#include <stddef.h>
#include <linux/compat.h>
=======
#include <stddef.h>
>>>>>>> p9x

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_mmu.h"
#include "kgsl_sharedmem.h"
#include "kgsl_iommu.h"
#include "adreno_pm4types.h"
#include "adreno.h"
#include "kgsl_trace.h"
#include "kgsl_cffdump.h"
#include "kgsl_pwrctrl.h"

<<<<<<< HEAD
#define _IOMMU_PRIV(_mmu) (&((_mmu)->priv.iommu))

#define ADDR_IN_GLOBAL(_mmu, _a) \
	(((_a) >= KGSL_IOMMU_GLOBAL_MEM_BASE(_mmu)) && \
	 ((_a) < (KGSL_IOMMU_GLOBAL_MEM_BASE(_mmu) + \
	 KGSL_IOMMU_GLOBAL_MEM_SIZE)))

static struct kgsl_mmu_pt_ops iommu_pt_ops;
static bool need_iommu_sync;

const unsigned int kgsl_iommu_reg_list[KGSL_IOMMU_REG_MAX] = {
	0x0,/* SCTLR */
	0x20,/* TTBR0 */
	0x34,/* CONTEXTIDR */
	0x58,/* FSR */
	0x60,/* FAR_0 */
	0x618,/* TLBIALL */
	0x008,/* RESUME */
	0x68,/* FSYNR0 */
	0x6C,/* FSYNR1 */
	0x7F0,/* TLBSYNC */
	0x7F4,/* TLBSTATUS */
};

/*
 * struct kgsl_iommu_addr_entry - entry in the kgsl_iommu_pt rbtree.
 * @base: starting virtual address of the entry
 * @size: size of the entry
 * @node: the rbtree node
 *
 */
struct kgsl_iommu_addr_entry {
	uint64_t base;
	uint64_t size;
	struct rb_node node;
};

static struct kmem_cache *addr_entry_cache;

/*
 * There are certain memory allocations (ringbuffer, memstore, etc) that need to
 * be present at the same address in every pagetable. We call these "global"
 * pagetable entries. There are relatively few of these and they are mostly
 * stable (defined at init time) but the actual number of globals can differ
 * slight depending on the target and implementation.
 *
 * Here we define an array and a simple allocator to keep track of the currently
 * active global entries. Each entry is assigned a unique address inside of a
 * MMU implementation specific "global" region. The addresses are assigned
 * sequentially and never re-used to avoid having to go back and reprogram
 * existing pagetables. The entire list of active entries are mapped and
 * unmapped into every new pagetable as it is created and destroyed.
 *
 * Because there are relatively few entries and they are defined at boot time we
 * don't need to go over the top to define a dynamic allocation scheme. It will
 * be less wasteful to pick a static number with a little bit of growth
 * potential.
 */

#define GLOBAL_PT_ENTRIES 32

struct global_pt_entry {
	struct kgsl_memdesc *memdesc;
	char name[32];
};

static struct global_pt_entry global_pt_entries[GLOBAL_PT_ENTRIES];
static struct kgsl_memdesc *kgsl_global_secure_pt_entry;
static int global_pt_count;
uint64_t global_pt_alloc;
static struct kgsl_memdesc gpu_qdss_desc;

void kgsl_print_global_pt_entries(struct seq_file *s)
{
	int i;

	for (i = 0; i < global_pt_count; i++) {
		struct kgsl_memdesc *memdesc = global_pt_entries[i].memdesc;

		if (memdesc == NULL)
			continue;

		seq_printf(s, "0x%16.16llX-0x%16.16llX %16llu %s\n",
			memdesc->gpuaddr, memdesc->gpuaddr + memdesc->size - 1,
			memdesc->size, global_pt_entries[i].name);
	}
}

static void kgsl_iommu_unmap_globals(struct kgsl_pagetable *pagetable)
{
	unsigned int i;

	for (i = 0; i < global_pt_count; i++) {
		if (global_pt_entries[i].memdesc != NULL)
			kgsl_mmu_unmap(pagetable,
					global_pt_entries[i].memdesc);
	}
}

static void kgsl_iommu_map_globals(struct kgsl_pagetable *pagetable)
{
	unsigned int i;

	for (i = 0; i < global_pt_count; i++) {
		if (global_pt_entries[i].memdesc != NULL) {
			int ret = kgsl_mmu_map(pagetable,
					global_pt_entries[i].memdesc);

			BUG_ON(ret);
		}
	}
}

static void kgsl_iommu_unmap_global_secure_pt_entry(struct kgsl_pagetable
								*pagetable)
{
	struct kgsl_memdesc *entry = kgsl_global_secure_pt_entry;

	if (entry != NULL)
		kgsl_mmu_unmap(pagetable, entry);

}

static void kgsl_map_global_secure_pt_entry(struct kgsl_pagetable *pagetable)
{
	int ret;
	struct kgsl_memdesc *entry = kgsl_global_secure_pt_entry;

	if (entry != NULL) {
		entry->pagetable = pagetable;
		ret = kgsl_mmu_map(pagetable, entry);
		BUG_ON(ret);
	}
}

static void kgsl_iommu_remove_global(struct kgsl_mmu *mmu,
		struct kgsl_memdesc *memdesc)
{
	int i;

	if (memdesc->gpuaddr == 0 || !(memdesc->priv & KGSL_MEMDESC_GLOBAL))
		return;

	for (i = 0; i < global_pt_count; i++) {
		if (global_pt_entries[i].memdesc == memdesc) {
			memdesc->gpuaddr = 0;
			memdesc->priv &= ~KGSL_MEMDESC_GLOBAL;
			global_pt_entries[i].memdesc = NULL;
			return;
		}
	}
}

static void kgsl_iommu_add_global(struct kgsl_mmu *mmu,
		struct kgsl_memdesc *memdesc, const char *name)
{
	if (memdesc->gpuaddr != 0)
		return;

	BUG_ON(global_pt_count >= GLOBAL_PT_ENTRIES);
	BUG_ON((global_pt_alloc + memdesc->size) >= KGSL_IOMMU_GLOBAL_MEM_SIZE);

	memdesc->gpuaddr = KGSL_IOMMU_GLOBAL_MEM_BASE(mmu) + global_pt_alloc;
	memdesc->priv |= KGSL_MEMDESC_GLOBAL;
	global_pt_alloc += memdesc->size;

	global_pt_entries[global_pt_count].memdesc = memdesc;
	strlcpy(global_pt_entries[global_pt_count].name, name,
			sizeof(global_pt_entries[global_pt_count].name));
	global_pt_count++;
}

void kgsl_add_global_secure_entry(struct kgsl_device *device,
					struct kgsl_memdesc *memdesc)
{
	memdesc->gpuaddr = KGSL_IOMMU_SECURE_BASE(&device->mmu);
	kgsl_global_secure_pt_entry = memdesc;
}

struct kgsl_memdesc *kgsl_iommu_get_qdss_global_entry(void)
{
	return &gpu_qdss_desc;
}

static void kgsl_setup_qdss_desc(struct kgsl_device *device)
{
	int result = 0;
	uint32_t gpu_qdss_entry[2];

	if (!of_find_property(device->pdev->dev.of_node,
		"qcom,gpu-qdss-stm", NULL))
		return;

	if (of_property_read_u32_array(device->pdev->dev.of_node,
				"qcom,gpu-qdss-stm", gpu_qdss_entry, 2)) {
		KGSL_CORE_ERR("Failed to read gpu qdss dts entry\n");
		return;
	}

	gpu_qdss_desc.flags = 0;
	gpu_qdss_desc.priv = 0;
	gpu_qdss_desc.physaddr = gpu_qdss_entry[0];
	gpu_qdss_desc.size = gpu_qdss_entry[1];
	gpu_qdss_desc.pagetable = NULL;
	gpu_qdss_desc.ops = NULL;
	gpu_qdss_desc.dev = device->dev->parent;
	gpu_qdss_desc.hostptr = NULL;

	result = memdesc_sg_dma(&gpu_qdss_desc, gpu_qdss_desc.physaddr,
			gpu_qdss_desc.size);
	if (result) {
		KGSL_CORE_ERR("memdesc_sg_dma failed: %d\n", result);
		return;
	}

	kgsl_mmu_add_global(device, &gpu_qdss_desc, "gpu-qdss");
}

static inline void kgsl_cleanup_qdss_desc(struct kgsl_mmu *mmu)
{
	kgsl_iommu_remove_global(mmu, &gpu_qdss_desc);
	kgsl_sharedmem_free(&gpu_qdss_desc);
}


static inline void _iommu_sync_mmu_pc(bool lock)
{
	if (need_iommu_sync == false)
		return;

	if (lock)
		mutex_lock(&kgsl_mmu_sync);
	else
		mutex_unlock(&kgsl_mmu_sync);
}

static void _detach_pt(struct kgsl_iommu_pt *iommu_pt,
			  struct kgsl_iommu_context *ctx)
{
	if (iommu_pt->attached) {
		_iommu_sync_mmu_pc(true);
		iommu_detach_device(iommu_pt->domain, ctx->dev);
		_iommu_sync_mmu_pc(false);
		iommu_pt->attached = false;
	}
}

static int _attach_pt(struct kgsl_iommu_pt *iommu_pt,
			struct kgsl_iommu_context *ctx)
{
	int ret;

	if (iommu_pt->attached)
		return 0;

	_iommu_sync_mmu_pc(true);
	ret = iommu_attach_device(iommu_pt->domain, ctx->dev);
	_iommu_sync_mmu_pc(false);

	if (ret == 0)
		iommu_pt->attached = true;

	return ret;
}

static int _lock_if_secure_mmu(struct kgsl_memdesc *memdesc,
		struct kgsl_mmu *mmu)
{
	struct kgsl_device *device = KGSL_MMU_DEVICE(mmu);

	if (!kgsl_memdesc_is_secured(memdesc))
		return 0;

	if (!kgsl_mmu_is_secured(mmu))
		return -EINVAL;

	mutex_lock(&device->mutex);
	if (kgsl_active_count_get(device)) {
		mutex_unlock(&device->mutex);
		return -EINVAL;
	}

	return 0;
}

static void _unlock_if_secure_mmu(struct kgsl_memdesc *memdesc,
		struct kgsl_mmu *mmu)
{
	struct kgsl_device *device = KGSL_MMU_DEVICE(mmu);

	if (!kgsl_memdesc_is_secured(memdesc) || !kgsl_mmu_is_secured(mmu))
		return;

	kgsl_active_count_put(device);
	mutex_unlock(&device->mutex);
}

static int _iommu_map_sync_pc(struct kgsl_pagetable *pt,
		struct kgsl_memdesc *memdesc,
		uint64_t gpuaddr, phys_addr_t physaddr,
		uint64_t size, unsigned int flags)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	int ret;

	ret = _lock_if_secure_mmu(memdesc, pt->mmu);
	if (ret)
		return ret;

	_iommu_sync_mmu_pc(true);

	ret = iommu_map(iommu_pt->domain, gpuaddr, physaddr, size, flags);

	_iommu_sync_mmu_pc(false);

	_unlock_if_secure_mmu(memdesc, pt->mmu);

	if (ret) {
		KGSL_CORE_ERR("map err: 0x%016llX, 0x%llx, 0x%x, %d\n",
			gpuaddr, size, flags, ret);
		return -ENODEV;
	}

	return 0;
}

static int _iommu_unmap_sync_pc(struct kgsl_pagetable *pt,
		struct kgsl_memdesc *memdesc, uint64_t addr, uint64_t size)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	size_t unmapped = 0;
	int ret;

	ret = _lock_if_secure_mmu(memdesc, pt->mmu);
	if (ret)
		return ret;

	_iommu_sync_mmu_pc(true);

	unmapped = iommu_unmap(iommu_pt->domain, addr, size);

	_iommu_sync_mmu_pc(false);

	_unlock_if_secure_mmu(memdesc, pt->mmu);

	if (unmapped != size) {
		KGSL_CORE_ERR("unmap err: 0x%016llx, 0x%llx, %zd\n",
			addr, size, unmapped);
		return -ENODEV;
	}

	return 0;
}

static int _iommu_map_sg_offset_sync_pc(struct kgsl_pagetable *pt,
		uint64_t addr, struct kgsl_memdesc *memdesc,
		struct scatterlist *sg, int nents,
		uint64_t offset, uint64_t size, unsigned int flags)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	uint64_t offset_tmp = offset;
	uint64_t size_tmp = size;
	size_t mapped = 0;
	unsigned int i;
	struct scatterlist *s;
	phys_addr_t physaddr;
	int ret;

	ret = _lock_if_secure_mmu(memdesc, pt->mmu);
	if (ret)
		return ret;

	_iommu_sync_mmu_pc(true);

	for_each_sg(sg, s, nents, i) {
		/* Iterate until we find the offset */
		if (offset_tmp >= s->length) {
			offset_tmp -= s->length;
			continue;
		}

		/* How much mapping is needed in this sg? */
		if (size < s->length - offset_tmp)
			size_tmp = size;
		else
			size_tmp = s->length - offset_tmp;

		/* Get the phys addr for the offset page */
		if (offset_tmp != 0) {
			physaddr = page_to_phys(nth_page(sg_page(s),
					offset_tmp >> PAGE_SHIFT));
			/* Reset offset_tmp */
			offset_tmp = 0;
		} else
			physaddr = page_to_phys(sg_page(s));

		/* Do the map for this sg */
		ret = iommu_map(iommu_pt->domain, addr + mapped,
				physaddr, size_tmp, flags);
		if (ret)
			break;

		mapped += size_tmp;
		size -= size_tmp;

		if (size == 0)
			break;
	}

	_iommu_sync_mmu_pc(false);

	_unlock_if_secure_mmu(memdesc, pt->mmu);

	if (size != 0) {
		/* Cleanup on error */
		_iommu_unmap_sync_pc(pt, memdesc, addr, mapped);
		KGSL_CORE_ERR("map err: 0x%016llX, %d, %x, %zd\n",
			addr, nents, flags, mapped);
		return  -ENODEV;
	}

	return 0;
}

static int _iommu_map_sg_sync_pc(struct kgsl_pagetable *pt,
		uint64_t addr, struct kgsl_memdesc *memdesc,
		struct scatterlist *sg, int nents,
		unsigned int flags)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	size_t mapped;
	int ret;

	ret = _lock_if_secure_mmu(memdesc, pt->mmu);
	if (ret)
		return ret;

	_iommu_sync_mmu_pc(true);

	mapped = iommu_map_sg(iommu_pt->domain, addr, sg, nents, flags);

	_iommu_sync_mmu_pc(false);

	_unlock_if_secure_mmu(memdesc, pt->mmu);

	if (mapped == 0) {
		KGSL_CORE_ERR("map err: 0x%016llX, %d, %x, %zd\n",
			addr, nents, flags, mapped);
		return  -ENODEV;
	}

	return 0;
=======
static struct kgsl_mmu_pt_ops iommu_pt_ops;

static struct kgsl_iommu_register_list kgsl_iommuv1_reg[KGSL_IOMMU_REG_MAX] = {
	{ 0, 0 },			/* GLOBAL_BASE */
	{ 0x0, 1 },			/* SCTLR */
	{ 0x20, 1 },			/* TTBR0 */
	{ 0x28, 1 },			/* TTBR1 */
	{ 0x58, 1 },			/* FSR */
	{ 0x60, 1 },			/* FAR_0 */
	{ 0x618, 1 },			/* TLBIALL */
	{ 0x008, 1 },			/* RESUME */
	{ 0x68, 1 },			/* FSYNR0 */
	{ 0x6C, 1 },			/* FSYNR1 */
	{ 0x7F0, 1 },			/* TLBSYNC */
	{ 0x7F4, 1 },			/* TLBSTATUS */
	{ 0x2000, 0 }			/* IMPLDEF_MICRO_MMU_CRTL */
};

static struct kgsl_iommu_register_list kgsl_iommuv2_reg[KGSL_IOMMU_REG_MAX] = {
	{ 0, 0 },			/* GLOBAL_BASE */
	{ 0x0, 1 },			/* SCTLR */
	{ 0x20, 1 },			/* TTBR0 */
	{ 0x28, 1 },			/* TTBR1 */
	{ 0x58, 1 },			/* FSR */
	{ 0x60, 1 },			/* FAR_0 */
	{ 0x618, 1 },			/* TLBIALL */
	{ 0x008, 1 },			/* RESUME */
	{ 0x68, 1 },			/* FSYNR0 */
	{ 0x6C, 1 },			/* FSYNR1 */
	{ 0x7F0, 1 },			/* TLBSYNC */
	{ 0x7F4, 1 },			/* TLBSTATUS */
	{ 0x6000, 0 }			/* IMPLDEF_MICRO_MMU_CRTL */
};

static int kgsl_iommu_flush_pt(struct kgsl_mmu *mmu);
static phys_addr_t
kgsl_iommu_get_current_ptbase(struct kgsl_mmu *mmu);

/*
 * kgsl_iommu_get_pt_base_addr - Get the physical address of the pagetable
 * @mmu - Pointer to mmu
 * @pt - kgsl pagetable pointer that contains the IOMMU domain pointer
 *
 */
static phys_addr_t kgsl_iommu_get_pt_base_addr(struct kgsl_mmu *mmu,
						struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt;

	if (pt == NULL)
		return 0;

	iommu_pt = pt->priv;

	return iommu_pt->pt_base;
>>>>>>> p9x
}

/*
 * One page allocation for a guard region to protect against over-zealous
 * GPU pre-fetch
 */

static struct page *kgsl_guard_page;
static struct kgsl_memdesc kgsl_secure_guard_page_memdesc;

/* These functions help find the nearest allocated memory entries on either side
 * of a faulting address. If we know the nearby allocations memory we can
 * get a better determination of what we think should have been located in the
 * faulting region
 */

/*
 * A local structure to make it easy to store the interesting bits for the
 * memory entries on either side of the faulting address
 */

struct _mem_entry {
	uint64_t gpuaddr;
	uint64_t size;
	uint64_t flags;
	unsigned int priv;
	int pending_free;
	pid_t pid;
<<<<<<< HEAD
	char name[32];
};

static void _get_global_entries(uint64_t faultaddr,
		struct _mem_entry *prev,
		struct _mem_entry *next)
{
	int i;
	uint64_t prevaddr = 0;
	struct global_pt_entry *p = NULL;

	uint64_t nextaddr = (uint64_t) -1;
	struct global_pt_entry *n = NULL;

	for (i = 0; i < global_pt_count; i++) {
		uint64_t addr;

		if (global_pt_entries[i].memdesc == NULL)
			continue;

		addr = global_pt_entries[i].memdesc->gpuaddr;
		if ((addr < faultaddr) && (addr > prevaddr)) {
			prevaddr = addr;
			p = &global_pt_entries[i];
		}

		if ((addr > faultaddr) && (addr < nextaddr)) {
			nextaddr = addr;
			n = &global_pt_entries[i];
		}
	}

	if (p != NULL) {
		prev->gpuaddr = p->memdesc->gpuaddr;
		prev->size = p->memdesc->size;
		prev->flags = p->memdesc->flags;
		prev->priv = p->memdesc->priv;
		prev->pid = 0;
		strlcpy(prev->name, p->name, sizeof(prev->name));
	}

	if (n != NULL) {
		next->gpuaddr = n->memdesc->gpuaddr;
		next->size = n->memdesc->size;
		next->flags = n->memdesc->flags;
		next->priv = n->memdesc->priv;
		next->pid = 0;
		strlcpy(next->name, n->name, sizeof(next->name));
	}
}

void __kgsl_get_memory_usage(struct _mem_entry *entry)
{
	kgsl_get_memory_usage(entry->name, sizeof(entry->name), entry->flags);
}

static void _get_entries(struct kgsl_process_private *private,
		uint64_t faultaddr, struct _mem_entry *prev,
		struct _mem_entry *next)
{
	int id;
	struct kgsl_mem_entry *entry;

	uint64_t prevaddr = 0;
	struct kgsl_mem_entry *p = NULL;

	uint64_t nextaddr = (uint64_t) -1;
	struct kgsl_mem_entry *n = NULL;

	idr_for_each_entry(&private->mem_idr, entry, id) {
		uint64_t addr = entry->memdesc.gpuaddr;

		if ((addr < faultaddr) && (addr > prevaddr)) {
			prevaddr = addr;
			p = entry;
		}

		if ((addr > faultaddr) && (addr < nextaddr)) {
			nextaddr = addr;
			n = entry;
		}
	}

	if (p != NULL) {
		prev->gpuaddr = p->memdesc.gpuaddr;
		prev->size = p->memdesc.size;
		prev->flags = p->memdesc.flags;
		prev->priv = p->memdesc.priv;
		prev->pending_free = p->pending_free;
		prev->pid = private->pid;
		__kgsl_get_memory_usage(prev);
	}

	if (n != NULL) {
		next->gpuaddr = n->memdesc.gpuaddr;
		next->size = n->memdesc.size;
		next->flags = n->memdesc.flags;
		next->priv = n->memdesc.priv;
		next->pending_free = n->pending_free;
		next->pid = private->pid;
		__kgsl_get_memory_usage(next);
=======
};

/*
 * Find the closest alloated memory block with an smaller GPU address then the
 * given address
 */

static void _prev_entry(struct kgsl_process_private *priv,
	uint64_t faultaddr, struct _mem_entry *ret)
{
	struct rb_node *node;
	struct kgsl_mem_entry *entry;

	for (node = rb_first(&priv->mem_rb); node; ) {
		entry = rb_entry(node, struct kgsl_mem_entry, node);

		if (entry->memdesc.gpuaddr > faultaddr)
			break;

		/*
		 * If this is closer to the faulting address, then copy
		 * the entry
		 */

		if (entry->memdesc.gpuaddr > ret->gpuaddr) {
			ret->gpuaddr = entry->memdesc.gpuaddr;
			ret->size = entry->memdesc.size;
			ret->flags = entry->memdesc.flags;
			ret->priv = entry->memdesc.priv;
			ret->pending_free = entry->pending_free;
			ret->pid = priv->pid;
		}

		node = rb_next(&entry->node);
	}
}

/*
 * Find the closest alloated memory block with a greater starting GPU address
 * then the given address
 */

static void _next_entry(struct kgsl_process_private *priv,
	uint64_t faultaddr, struct _mem_entry *ret)
{
	struct rb_node *node;
	struct kgsl_mem_entry *entry;

	for (node = rb_last(&priv->mem_rb); node; ) {
		entry = rb_entry(node, struct kgsl_mem_entry, node);

		if (entry->memdesc.gpuaddr < faultaddr)
			break;

		/*
		 * If this is closer to the faulting address, then copy
		 * the entry
		 */

		if (entry->memdesc.gpuaddr < ret->gpuaddr) {
			ret->gpuaddr = entry->memdesc.gpuaddr;
			ret->size = entry->memdesc.size;
			ret->flags = entry->memdesc.flags;
			ret->priv = entry->memdesc.priv;
			ret->pending_free = entry->pending_free;
			ret->pid = priv->pid;
		}

		node = rb_prev(&entry->node);
>>>>>>> p9x
	}
}

static void _find_mem_entries(struct kgsl_mmu *mmu, uint64_t faultaddr,
<<<<<<< HEAD
		struct _mem_entry *preventry, struct _mem_entry *nextentry,
		struct kgsl_context *context)
{
	struct kgsl_process_private *private;
=======
	phys_addr_t ptbase, struct _mem_entry *preventry,
	struct _mem_entry *nextentry)
{
	struct kgsl_process_private *private = NULL, *p;
	int id = kgsl_mmu_get_ptname_from_ptbase(mmu, ptbase);
>>>>>>> p9x

	memset(preventry, 0, sizeof(*preventry));
	memset(nextentry, 0, sizeof(*nextentry));

	/* Set the maximum possible size as an initial value */
	nextentry->gpuaddr = (uint64_t) -1;

<<<<<<< HEAD
	if (ADDR_IN_GLOBAL(mmu, faultaddr)) {
		_get_global_entries(faultaddr, preventry, nextentry);
	} else if (context) {
		private = context->proc_priv;
		spin_lock(&private->mem_lock);
		_get_entries(private, faultaddr, preventry, nextentry);
		spin_unlock(&private->mem_lock);
=======
	mutex_lock(&kgsl_driver.process_mutex);
	list_for_each_entry(p, &kgsl_driver.process_list, list) {
		if (p->pagetable && (p->pagetable->name == id)) {
			if (kgsl_process_private_get(p))
				private = p;
			break;
		}
	}
	mutex_unlock(&kgsl_driver.process_mutex);

	if (private != NULL) {
		spin_lock(&private->mem_lock);
		_prev_entry(private, faultaddr, preventry);
		_next_entry(private, faultaddr, nextentry);
		spin_unlock(&private->mem_lock);

		kgsl_process_private_put(private);
>>>>>>> p9x
	}
}

static void _print_entry(struct kgsl_device *device, struct _mem_entry *entry)
{
<<<<<<< HEAD
=======
	char name[32];
	memset(name, 0, sizeof(name));

	kgsl_get_memory_usage(name, sizeof(name) - 1, entry->flags);

>>>>>>> p9x
	KGSL_LOG_DUMP(device,
		"[%016llX - %016llX] %s %s (pid = %d) (%s)\n",
		entry->gpuaddr,
		entry->gpuaddr + entry->size,
		entry->priv & KGSL_MEMDESC_GUARD_PAGE ? "(+guard)" : "",
		entry->pending_free ? "(pending free)" : "",
<<<<<<< HEAD
		entry->pid, entry->name);
}

static void _check_if_freed(struct kgsl_iommu_context *ctx,
	uint64_t addr, pid_t ptname)
=======
		entry->pid, name);
}

static void _check_if_freed(struct kgsl_iommu_context *ctx,
	uint64_t addr, pid_t pid)
>>>>>>> p9x
{
	uint64_t gpuaddr = addr;
	uint64_t size = 0;
	uint64_t flags = 0;
<<<<<<< HEAD
	pid_t pid;
=======
>>>>>>> p9x

	char name[32];
	memset(name, 0, sizeof(name));

<<<<<<< HEAD
	if (kgsl_memfree_find_entry(ptname, &gpuaddr, &size, &flags, &pid)) {
=======
	if (kgsl_memfree_find_entry(pid, &gpuaddr, &size, &flags)) {
>>>>>>> p9x
		kgsl_get_memory_usage(name, sizeof(name) - 1, flags);
		KGSL_LOG_DUMP(ctx->kgsldev, "---- premature free ----\n");
		KGSL_LOG_DUMP(ctx->kgsldev,
			"[%8.8llX-%8.8llX] (%s) was already freed by pid %d\n",
			gpuaddr, gpuaddr + size, name, pid);
	}
}

<<<<<<< HEAD
static bool
kgsl_iommu_uche_overfetch(struct kgsl_process_private *private,
		uint64_t faultaddr)
{
	int id;
	struct kgsl_mem_entry *entry = NULL;

	spin_lock(&private->mem_lock);
	idr_for_each_entry(&private->mem_idr, entry, id) {
		struct kgsl_memdesc *m = &entry->memdesc;

		if ((faultaddr >= (m->gpuaddr + m->size))
				&& (faultaddr < (m->gpuaddr + m->size + 64))) {
			spin_unlock(&private->mem_lock);
			return true;
		}
	}
	spin_unlock(&private->mem_lock);
	return false;
}

/*
 * Read pagefaults where the faulting address lies within the first 64 bytes
 * of a page (UCHE line size is 64 bytes) and the fault page is preceded by a
 * valid allocation are considered likely due to UCHE overfetch and suppressed.
 */

static bool kgsl_iommu_suppress_pagefault(uint64_t faultaddr, int write,
					struct kgsl_context *context)
{
	/*
	 * If there is no context associated with the pagefault then this
	 * could be a fault on a global buffer. We do not suppress faults
	 * on global buffers as they are mainly accessed by the CP bypassing
	 * the UCHE. Also, write pagefaults are never suppressed.
	 */
	if (!context || write)
		return false;

	return kgsl_iommu_uche_overfetch(context->proc_priv, faultaddr);
}

=======
>>>>>>> p9x
static int kgsl_iommu_fault_handler(struct iommu_domain *domain,
	struct device *dev, unsigned long addr, int flags, void *token)
{
	int ret = 0;
<<<<<<< HEAD
	struct kgsl_pagetable *pt = token;
	struct kgsl_mmu *mmu = pt->mmu;
	struct kgsl_iommu *iommu;
	struct kgsl_iommu_context *ctx;
	u64 ptbase;
	u32 contextidr;
	pid_t tid = 0;
	pid_t ptname;
	struct _mem_entry prev, next;
=======
	struct kgsl_pagetable *default_pt = token;
	struct kgsl_mmu *mmu = default_pt->mmu;
	struct kgsl_iommu *iommu;
	struct kgsl_iommu_context *ctx;
	phys_addr_t ptbase;
	unsigned int pid, fsr;
	struct _mem_entry prev, next;
	unsigned int fsynr0, fsynr1;
>>>>>>> p9x
	int write;
	struct kgsl_device *device;
	struct adreno_device *adreno_dev;
	unsigned int no_page_fault_log = 0;
	unsigned int curr_context_id = 0;
	struct kgsl_context *context;
<<<<<<< HEAD
	char *fault_type = "unknown";

	static DEFINE_RATELIMIT_STATE(_rs,
					DEFAULT_RATELIMIT_INTERVAL,
					DEFAULT_RATELIMIT_BURST);

	if (mmu == NULL)
		return ret;

	iommu = _IOMMU_PRIV(mmu);
	ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	device = KGSL_MMU_DEVICE(mmu);
	adreno_dev = ADRENO_DEVICE(device);

	if (pt->name == KGSL_MMU_SECURE_PT)
		ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_SECURE];

=======

	if (mmu == NULL || mmu->priv == NULL)
		return ret;

	iommu = mmu->priv;
	ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	device = mmu->device;
	adreno_dev = ADRENO_DEVICE(device);

	/*
	 * If mmu fault not set then set it and continue else
	 * exit this function since another thread has already set
	 * it and will execute rest of this function for the fault.
	 */
	if (1 == atomic_cmpxchg(&mmu->fault, 0, 1))
		return 0;

	fsr = KGSL_IOMMU_GET_CTX_REG(iommu, ctx->ctx_id, FSR);
	/*
	 * If fsr is not set then it means that we cleared the fault while the
	 * bottom half called from IOMMU driver is running
	 */
	if (!fsr) {
		atomic_set(&mmu->fault, 0);
		return 0;
	}
>>>>>>> p9x
	/*
	 * set the fault bits and stuff before any printks so that if fault
	 * handler runs then it will know it's dealing with a pagefault.
	 * Read the global current timestamp because we could be in middle of
	 * RB switch and hence the cur RB may not be reliable but global
	 * one will always be reliable
	 */
	kgsl_sharedmem_readl(&device->memstore, &curr_context_id,
		KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL, current_context));

	context = kgsl_context_get(device, curr_context_id);

<<<<<<< HEAD
	write = (flags & IOMMU_FAULT_WRITE) ? 1 : 0;
	if (flags & IOMMU_FAULT_TRANSLATION)
		fault_type = "translation";
	else if (flags & IOMMU_FAULT_PERMISSION)
		fault_type = "permission";

	if (kgsl_iommu_suppress_pagefault(addr, write, context)) {
		iommu->pagefault_suppression_count++;
		kgsl_context_put(context);
		return ret;
	}

	if (context != NULL) {
		/* save pagefault timestamp for GFT */
		set_bit(KGSL_CONTEXT_PRIV_PAGEFAULT, &context->priv);
		tid = context->tid;
=======
	if (context != NULL) {
		/* save pagefault timestamp for GFT */
		set_bit(KGSL_CONTEXT_PRIV_PAGEFAULT, &context->priv);

		kgsl_context_put(context);
		context = NULL;
>>>>>>> p9x
	}

	ctx->fault = 1;

	if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE,
<<<<<<< HEAD
		&adreno_dev->ft_pf_policy) &&
		(flags & IOMMU_FAULT_TRANSACTION_STALLED)) {
		/*
		 * Turn off GPU IRQ so we don't get faults from it too.
		 * The device mutex must be held to change power state
		 */
		mutex_lock(&device->mutex);
		kgsl_pwrctrl_change_state(device, KGSL_STATE_AWARE);
		mutex_unlock(&device->mutex);
	}

	ptbase = KGSL_IOMMU_GET_CTX_REG_Q(ctx, TTBR0);
	contextidr = KGSL_IOMMU_GET_CTX_REG(ctx, CONTEXTIDR);

	ptname = MMU_FEATURE(mmu, KGSL_MMU_GLOBAL_PAGETABLE) ?
		KGSL_MMU_GLOBAL_PT : tid;
	/*
	 * Trace needs to be logged before searching the faulting
	 * address in free list as it takes quite long time in
	 * search and delays the trace unnecessarily.
	 */
	trace_kgsl_mmu_pagefault(ctx->kgsldev, addr,
			ptname, write ? "write" : "read");
=======
		&adreno_dev->ft_pf_policy)) {
		adreno_set_gpu_fault(adreno_dev, ADRENO_IOMMU_PAGE_FAULT);
		/* turn off GPU IRQ so we don't get faults from it too */
		kgsl_pwrctrl_change_state(device, KGSL_STATE_AWARE);
		adreno_dispatcher_schedule(device);
	}

	ptbase = KGSL_IOMMU_GET_CTX_REG_Q(iommu, ctx->ctx_id, TTBR0)
			& KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;

	fsynr0 = KGSL_IOMMU_GET_CTX_REG(iommu, ctx->ctx_id, FSYNR0);
	fsynr1 = KGSL_IOMMU_GET_CTX_REG(iommu, ctx->ctx_id, FSYNR1);

	write = ((fsynr0 & (KGSL_IOMMU_V1_FSYNR0_WNR_MASK <<
			KGSL_IOMMU_V1_FSYNR0_WNR_SHIFT)) ? 1 : 0);

	pid = kgsl_mmu_get_ptname_from_ptbase(mmu, ptbase);
>>>>>>> p9x

	if (test_bit(KGSL_FT_PAGEFAULT_LOG_ONE_PER_PAGE,
		&adreno_dev->ft_pf_policy))
		no_page_fault_log = kgsl_mmu_log_fault_addr(mmu, ptbase, addr);

<<<<<<< HEAD
	if (!no_page_fault_log && __ratelimit(&_rs)) {
		KGSL_MEM_CRIT(ctx->kgsldev,
			"GPU PAGE FAULT: addr = %lX pid= %d\n", addr, ptname);
		KGSL_MEM_CRIT(ctx->kgsldev,
			"context=%s TTBR0=0x%llx CIDR=0x%x (%s %s fault)\n",
			ctx->name, ptbase, contextidr,
			write ? "write" : "read", fault_type);

		/* Don't print the debug if this is a permissions fault */
		if (!(flags & IOMMU_FAULT_PERMISSION)) {
			_check_if_freed(ctx, addr, ptname);

			KGSL_LOG_DUMP(ctx->kgsldev,
				"---- nearby memory ----\n");

			_find_mem_entries(mmu, addr, &prev, &next, context);
			if (prev.gpuaddr)
				_print_entry(ctx->kgsldev, &prev);
			else
				KGSL_LOG_DUMP(ctx->kgsldev, "*EMPTY*\n");

			KGSL_LOG_DUMP(ctx->kgsldev, " <- fault @ %8.8lX\n",
				addr);

			if (next.gpuaddr != (uint64_t) -1)
				_print_entry(ctx->kgsldev, &next);
			else
				KGSL_LOG_DUMP(ctx->kgsldev, "*EMPTY*\n");
		}
	}

=======
	if (!no_page_fault_log) {
		KGSL_MEM_CRIT(ctx->kgsldev,
			"GPU PAGE FAULT: addr = %lX pid = %d\n", addr, pid);
		KGSL_MEM_CRIT(ctx->kgsldev,
		 "context = %d TTBR0 = %pa FSR = %X FSYNR0 = %X FSYNR1 = %X(%s fault)\n",
			ctx->ctx_id, &ptbase, fsr, fsynr0, fsynr1,
			write ? "write" : "read");

		_check_if_freed(ctx, addr, pid);

		KGSL_LOG_DUMP(ctx->kgsldev, "---- nearby memory ----\n");

		_find_mem_entries(mmu, addr, ptbase, &prev, &next);

		if (prev.gpuaddr)
			_print_entry(ctx->kgsldev, &prev);
		else
			KGSL_LOG_DUMP(ctx->kgsldev, "*EMPTY*\n");

		KGSL_LOG_DUMP(ctx->kgsldev, " <- fault @ %8.8lX\n", addr);

		if (next.gpuaddr != (uint64_t) -1)
			_print_entry(ctx->kgsldev, &next);
		else
			KGSL_LOG_DUMP(ctx->kgsldev, "*EMPTY*\n");

	}

	trace_kgsl_mmu_pagefault(ctx->kgsldev, addr,
			kgsl_mmu_get_ptname_from_ptbase(mmu, ptbase),
			write ? "write" : "read");
>>>>>>> p9x

	/*
	 * We do not want the h/w to resume fetching data from an iommu
	 * that has faulted, this is better for debugging as it will stall
<<<<<<< HEAD
	 * the GPU and trigger a snapshot. Return EBUSY error.
	 */
	if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE,
		&adreno_dev->ft_pf_policy) &&
		(flags & IOMMU_FAULT_TRANSACTION_STALLED)) {
		uint32_t sctlr_val;
		ret = -EBUSY;
		/*
		 * Disable context fault interrupts
		 * as we do not clear FSR in the ISR.
		 * Will be re-enabled after FSR is cleared.
		 */
		sctlr_val = KGSL_IOMMU_GET_CTX_REG(ctx, SCTLR);
		sctlr_val &= ~(0x1 << KGSL_IOMMU_SCTLR_CFIE_SHIFT);
		KGSL_IOMMU_SET_CTX_REG(ctx, SCTLR, sctlr_val);

		adreno_set_gpu_fault(adreno_dev, ADRENO_IOMMU_PAGE_FAULT);
		/* Go ahead with recovery*/
		adreno_dispatcher_schedule(device);
	}

	kgsl_context_put(context);
=======
	 * the GPU and trigger a snapshot. To stall the transaction return
	 * EBUSY error.
	 */

	if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE,
		&adreno_dev->ft_pf_policy))
		ret = -EBUSY;

>>>>>>> p9x
	return ret;
}

/*
 * kgsl_iommu_disable_clk() - Disable iommu clocks
 * Disable IOMMU clocks
 */
static void kgsl_iommu_disable_clk(struct kgsl_mmu *mmu)
{
<<<<<<< HEAD
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
=======
	struct kgsl_iommu *iommu = mmu->priv;
>>>>>>> p9x
	int j;

	atomic_dec(&iommu->clk_enable_count);
	BUG_ON(atomic_read(&iommu->clk_enable_count) < 0);

	for (j = (KGSL_IOMMU_MAX_CLKS - 1); j >= 0; j--)
		if (iommu->clks[j])
			clk_disable_unprepare(iommu->clks[j]);
}

/*
 * kgsl_iommu_enable_clk_prepare_enable - Enable the specified IOMMU clock
 * Try 4 times to enable it and then BUG() for debug
 */
static void kgsl_iommu_clk_prepare_enable(struct clk *clk)
{
	int num_retries = 4;

	while (num_retries--) {
		if (!clk_prepare_enable(clk))
			return;
	}

	/* Failure is fatal so BUG() to facilitate debug */
	KGSL_CORE_ERR("IOMMU clock enable failed\n");
	BUG();
}

/*
 * kgsl_iommu_enable_clk - Enable iommu clocks
 * Enable all the IOMMU clocks
 */
static void kgsl_iommu_enable_clk(struct kgsl_mmu *mmu)
{
	int j;
<<<<<<< HEAD
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
=======
	struct kgsl_iommu *iommu = mmu->priv;
>>>>>>> p9x

	for (j = 0; j < KGSL_IOMMU_MAX_CLKS; j++) {
		if (iommu->clks[j])
			kgsl_iommu_clk_prepare_enable(iommu->clks[j]);
	}
	atomic_inc(&iommu->clk_enable_count);
}

<<<<<<< HEAD
/* kgsl_iommu_get_ttbr0 - Get TTBR0 setting for a pagetable */
static u64 kgsl_iommu_get_ttbr0(struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt = pt ? pt->priv : NULL;

	BUG_ON(iommu_pt == NULL);

	return iommu_pt->ttbr0;
}

static bool kgsl_iommu_pt_equal(struct kgsl_mmu *mmu,
				struct kgsl_pagetable *pt,
				u64 ttbr0)
{
	struct kgsl_iommu_pt *iommu_pt = pt ? pt->priv : NULL;
	u64 domain_ttbr0;
=======
/*
 * kgsl_iommu_pt_equal - Check if pagetables are equal
 * @mmu - Pointer to mmu structure
 * @pt - Pointer to pagetable
 * @pt_base - Address of a pagetable that the IOMMU register is
 * programmed with
 *
 * Checks whether the pt_base is equal to the base address of
 * the pagetable which is contained in the pt structure
 * Return - Non-zero if the pagetable addresses are equal else 0
 */
static int kgsl_iommu_pt_equal(struct kgsl_mmu *mmu,
				struct kgsl_pagetable *pt,
				phys_addr_t pt_base)
{
	struct kgsl_iommu_pt *iommu_pt = pt ? pt->priv : NULL;
	phys_addr_t domain_ptbase;
>>>>>>> p9x

	if (iommu_pt == NULL)
		return 0;

<<<<<<< HEAD
	domain_ttbr0 = kgsl_iommu_get_ttbr0(pt);

	return (domain_ttbr0 == ttbr0);
}

/* kgsl_iommu_get_contextidr - query CONTEXTIDR setting for a pagetable */
static u32 kgsl_iommu_get_contextidr(struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt = pt ? pt->priv : NULL;

	BUG_ON(iommu_pt == NULL);

	return iommu_pt->contextidr;
=======
	domain_ptbase = kgsl_iommu_get_pt_base_addr(mmu, pt);

	return (domain_ptbase == pt_base);

}

/*
 * kgsl_iommu_get_ptbase - Get pagetable base address
 * @pt - Pointer to pagetable
 *
 * Returns the physical address of the pagetable base.
 *
 */
static phys_addr_t kgsl_iommu_get_ptbase(struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt = pt ? pt->priv : NULL;

	if (iommu_pt == NULL)
		return 0;

	return kgsl_iommu_get_pt_base_addr(pt->mmu, pt);
>>>>>>> p9x
}

/*
 * kgsl_iommu_destroy_pagetable - Free up reaources help by a pagetable
 * @mmu_specific_pt - Pointer to pagetable which is to be freed
 *
 * Return - void
 */
static void kgsl_iommu_destroy_pagetable(struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	struct kgsl_mmu *mmu = pt->mmu;
<<<<<<< HEAD
	struct kgsl_iommu *iommu;
	struct kgsl_iommu_context  *ctx;

	BUG_ON(!list_empty(&pt->list));

	iommu = _IOMMU_PRIV(mmu);

	if (KGSL_MMU_SECURE_PT == pt->name) {
		ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_SECURE];
		kgsl_iommu_unmap_global_secure_pt_entry(pt);
	}
	else {
		ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
		kgsl_iommu_unmap_globals(pt);
	}

	if (iommu_pt->domain) {
		trace_kgsl_pagetable_destroy(iommu_pt->ttbr0, pt->name);

		_detach_pt(iommu_pt, ctx);
=======

	BUG_ON(!list_empty(&pt->list));

	if (iommu_pt->domain) {
		phys_addr_t domain_ptbase =
					kgsl_iommu_get_pt_base_addr(mmu, pt);
		trace_kgsl_pagetable_destroy(domain_ptbase, pt->name);
>>>>>>> p9x

		iommu_domain_free(iommu_pt->domain);
	}

	kfree(iommu_pt);
<<<<<<< HEAD
}

static void setup_64bit_pagetable(struct kgsl_mmu *mmu,
		struct kgsl_pagetable *pagetable,
		struct kgsl_iommu_pt *pt)
{
	unsigned int secure_global_size = kgsl_global_secure_pt_entry != NULL ?
					kgsl_global_secure_pt_entry->size : 0;
	if (mmu->secured && pagetable->name == KGSL_MMU_SECURE_PT) {
		pt->compat_va_start = KGSL_IOMMU_SECURE_BASE(mmu) +
						secure_global_size;
		pt->compat_va_end = KGSL_IOMMU_SECURE_END(mmu);
		pt->va_start = KGSL_IOMMU_SECURE_BASE(mmu) + secure_global_size;
		pt->va_end = KGSL_IOMMU_SECURE_END(mmu);
	} else {
		pt->compat_va_start = KGSL_IOMMU_SVM_BASE32;
		pt->compat_va_end = KGSL_IOMMU_SECURE_BASE(mmu);
		pt->va_start = KGSL_IOMMU_VA_BASE64;
		pt->va_end = KGSL_IOMMU_VA_END64;
	}

	if (pagetable->name != KGSL_MMU_GLOBAL_PT &&
		pagetable->name != KGSL_MMU_SECURE_PT) {
		if ((BITS_PER_LONG == 32) || is_compat_task()) {
			pt->svm_start = KGSL_IOMMU_SVM_BASE32;
			pt->svm_end = KGSL_IOMMU_SECURE_BASE(mmu);
		} else {
			pt->svm_start = KGSL_IOMMU_SVM_BASE64;
			pt->svm_end = KGSL_IOMMU_SVM_END64;
		}
	}
}

static void setup_32bit_pagetable(struct kgsl_mmu *mmu,
		struct kgsl_pagetable *pagetable,
		struct kgsl_iommu_pt *pt)
{
	unsigned int secure_global_size = kgsl_global_secure_pt_entry != NULL ?
					kgsl_global_secure_pt_entry->size : 0;
	if (mmu->secured) {
		if (pagetable->name == KGSL_MMU_SECURE_PT) {
			pt->compat_va_start = KGSL_IOMMU_SECURE_BASE(mmu) +
						secure_global_size;
			pt->compat_va_end = KGSL_IOMMU_SECURE_END(mmu);
			pt->va_start = KGSL_IOMMU_SECURE_BASE(mmu) +
						secure_global_size;
			pt->va_end = KGSL_IOMMU_SECURE_END(mmu);
		} else {
			pt->va_start = KGSL_IOMMU_SVM_BASE32;
			pt->va_end = KGSL_IOMMU_SECURE_BASE(mmu) +
						secure_global_size;
			pt->compat_va_start = pt->va_start;
			pt->compat_va_end = pt->va_end;
		}
	} else {
		pt->va_start = KGSL_IOMMU_SVM_BASE32;
		pt->va_end = KGSL_IOMMU_GLOBAL_MEM_BASE(mmu);
		pt->compat_va_start = pt->va_start;
		pt->compat_va_end = pt->va_end;
	}

	if (pagetable->name != KGSL_MMU_GLOBAL_PT &&
		pagetable->name != KGSL_MMU_SECURE_PT) {
		pt->svm_start = KGSL_IOMMU_SVM_BASE32;
		pt->svm_end = KGSL_IOMMU_SVM_END32;
	}
}


static struct kgsl_iommu_pt *
_alloc_pt(struct device *dev, struct kgsl_mmu *mmu, struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt;
	struct bus_type *bus = kgsl_mmu_get_bus(dev);

	if (bus == NULL)
		return ERR_PTR(-ENODEV);

	iommu_pt = kzalloc(sizeof(struct kgsl_iommu_pt), GFP_KERNEL);
	if (iommu_pt == NULL)
		return ERR_PTR(-ENOMEM);

	iommu_pt->domain = iommu_domain_alloc(bus);
	if (iommu_pt->domain == NULL) {
		kfree(iommu_pt);
		return ERR_PTR(-ENODEV);
	}

	pt->pt_ops = &iommu_pt_ops;
	pt->priv = iommu_pt;
	pt->fault_addr = ~0ULL;
	iommu_pt->rbtree = RB_ROOT;

	if (MMU_FEATURE(mmu, KGSL_MMU_64BIT))
		setup_64bit_pagetable(mmu, pt, iommu_pt);
	else
		setup_32bit_pagetable(mmu, pt, iommu_pt);


	return iommu_pt;
}

static void _free_pt(struct kgsl_iommu_context *ctx, struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;

	pt->pt_ops = NULL;
	pt->priv = NULL;

	if (iommu_pt == NULL)
		return;

	_detach_pt(iommu_pt, ctx);

	if (iommu_pt->domain != NULL)
		iommu_domain_free(iommu_pt->domain);
	kfree(iommu_pt);
}

static int _init_global_pt(struct kgsl_mmu *mmu, struct kgsl_pagetable *pt)
{
	int ret = 0;
	struct kgsl_iommu_pt *iommu_pt = NULL;
	int disable_htw = !MMU_FEATURE(mmu, KGSL_MMU_COHERENT_HTW);
	unsigned int cb_num;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];

	iommu_pt = _alloc_pt(ctx->dev, mmu, pt);

	if (IS_ERR(iommu_pt))
		return PTR_ERR(iommu_pt);

	iommu_domain_set_attr(iommu_pt->domain,
				DOMAIN_ATTR_COHERENT_HTW_DISABLE, &disable_htw);

	if (kgsl_mmu_is_perprocess(mmu)) {
		ret = iommu_domain_set_attr(iommu_pt->domain,
				DOMAIN_ATTR_PROCID, &pt->name);
		if (ret) {
			KGSL_CORE_ERR("set DOMAIN_ATTR_PROCID failed: %d\n",
					ret);
			goto done;
		}
	}

	ret = _attach_pt(iommu_pt, ctx);
	if (ret)
		goto done;

	iommu_set_fault_handler(iommu_pt->domain,
				kgsl_iommu_fault_handler, pt);

	ret = iommu_domain_get_attr(iommu_pt->domain,
				DOMAIN_ATTR_CONTEXT_BANK, &cb_num);
	if (ret) {
		KGSL_CORE_ERR("get DOMAIN_ATTR_PROCID failed: %d\n",
				ret);
		goto done;
	}

	ctx->cb_num = cb_num;
	ctx->regbase = iommu->regbase + KGSL_IOMMU_CB0_OFFSET
			+ (cb_num << KGSL_IOMMU_CB_SHIFT);

	ret = iommu_domain_get_attr(iommu_pt->domain,
			DOMAIN_ATTR_TTBR0, &iommu_pt->ttbr0);
	if (ret) {
		KGSL_CORE_ERR("get DOMAIN_ATTR_TTBR0 failed: %d\n",
				ret);
		goto done;
	}
	ret = iommu_domain_get_attr(iommu_pt->domain,
			DOMAIN_ATTR_CONTEXTIDR, &iommu_pt->contextidr);
	if (ret) {
		KGSL_CORE_ERR("get DOMAIN_ATTR_CONTEXTIDR failed: %d\n",
				ret);
		goto done;
	}

	kgsl_iommu_map_globals(pt);

done:
	if (ret)
		_free_pt(ctx, pt);

	return ret;
}

static int _init_secure_pt(struct kgsl_mmu *mmu, struct kgsl_pagetable *pt)
{
	int ret = 0;
	struct kgsl_iommu_pt *iommu_pt = NULL;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	int disable_htw = !MMU_FEATURE(mmu, KGSL_MMU_COHERENT_HTW);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_SECURE];
	int secure_vmid = VMID_CP_PIXEL;
	unsigned int cb_num;

	if (!mmu->secured)
		return -EPERM;

	if (!MMU_FEATURE(mmu, KGSL_MMU_HYP_SECURE_ALLOC)) {
		if (!kgsl_mmu_bus_secured(ctx->dev))
			return -EPERM;
	}

	iommu_pt = _alloc_pt(ctx->dev, mmu, pt);

	if (IS_ERR(iommu_pt))
		return PTR_ERR(iommu_pt);

	iommu_domain_set_attr(iommu_pt->domain,
				DOMAIN_ATTR_COHERENT_HTW_DISABLE, &disable_htw);

	ret = iommu_domain_set_attr(iommu_pt->domain,
				    DOMAIN_ATTR_SECURE_VMID, &secure_vmid);
	if (ret) {
		KGSL_CORE_ERR("set DOMAIN_ATTR_SECURE_VMID failed: %d\n", ret);
		goto done;
	}

	ret = _attach_pt(iommu_pt, ctx);

	if (MMU_FEATURE(mmu, KGSL_MMU_HYP_SECURE_ALLOC))
		iommu_set_fault_handler(iommu_pt->domain,
					kgsl_iommu_fault_handler, pt);

	ret = iommu_domain_get_attr(iommu_pt->domain,
				DOMAIN_ATTR_CONTEXT_BANK, &cb_num);
	if (ret) {
		KGSL_CORE_ERR("get DOMAIN_ATTR_PROCID failed: %d\n",
				ret);
		goto done;
	}

	ctx->cb_num = cb_num;
	ctx->regbase = iommu->regbase + KGSL_IOMMU_CB0_OFFSET
			+ (cb_num << KGSL_IOMMU_CB_SHIFT);

	kgsl_map_global_secure_pt_entry(pt);

done:
	if (ret)
		_free_pt(ctx, pt);
	return ret;
}

static int _init_per_process_pt(struct kgsl_mmu *mmu, struct kgsl_pagetable *pt)
{
	int ret = 0;
	struct kgsl_iommu_pt *iommu_pt = NULL;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	int dynamic = 1;
	unsigned int cb_num = ctx->cb_num;
	int disable_htw = !MMU_FEATURE(mmu, KGSL_MMU_COHERENT_HTW);

	iommu_pt = _alloc_pt(ctx->dev, mmu, pt);

	if (IS_ERR(iommu_pt))
		return PTR_ERR(iommu_pt);

	ret = iommu_domain_set_attr(iommu_pt->domain,
				DOMAIN_ATTR_DYNAMIC, &dynamic);
	if (ret) {
		KGSL_CORE_ERR("set DOMAIN_ATTR_DYNAMIC failed: %d\n", ret);
		goto done;
	}
	ret = iommu_domain_set_attr(iommu_pt->domain,
				DOMAIN_ATTR_CONTEXT_BANK, &cb_num);
	if (ret) {
		KGSL_CORE_ERR("set DOMAIN_ATTR_CONTEXT_BANK failed: %d\n", ret);
		goto done;
	}

	ret = iommu_domain_set_attr(iommu_pt->domain,
				DOMAIN_ATTR_PROCID, &pt->name);
	if (ret) {
		KGSL_CORE_ERR("set DOMAIN_ATTR_PROCID failed: %d\n", ret);
		goto done;
	}

	iommu_domain_set_attr(iommu_pt->domain,
				DOMAIN_ATTR_COHERENT_HTW_DISABLE, &disable_htw);

	ret = _attach_pt(iommu_pt, ctx);
	if (ret)
		goto done;

	/* now read back the attributes needed for self programming */
	ret = iommu_domain_get_attr(iommu_pt->domain,
				DOMAIN_ATTR_TTBR0, &iommu_pt->ttbr0);
	if (ret) {
		KGSL_CORE_ERR("get DOMAIN_ATTR_TTBR0 failed: %d\n", ret);
		goto done;
	}

	ret = iommu_domain_get_attr(iommu_pt->domain,
				DOMAIN_ATTR_CONTEXTIDR, &iommu_pt->contextidr);
	if (ret) {
		KGSL_CORE_ERR("get DOMAIN_ATTR_CONTEXTIDR failed: %d\n", ret);
		goto done;
	}

	kgsl_iommu_map_globals(pt);

done:
	if (ret)
		_free_pt(ctx, pt);

	return ret;
}
=======
	iommu_pt = NULL;
}

/* currently only the MSM_IOMMU driver supports secure iommu */
#ifdef CONFIG_MSM_IOMMU
static inline struct bus_type *
get_secure_bus(void)
{
	return &msm_iommu_sec_bus_type;
}
#else
static inline struct bus_type *
get_secure_bus(void)
{
	return NULL;
}
#endif
>>>>>>> p9x

/* kgsl_iommu_init_pt - Set up an IOMMU pagetable */
static int kgsl_iommu_init_pt(struct kgsl_mmu *mmu, struct kgsl_pagetable *pt)
{
<<<<<<< HEAD
	if (pt == NULL)
		return -EINVAL;

	switch (pt->name) {
	case KGSL_MMU_GLOBAL_PT:
		return _init_global_pt(mmu, pt);

	case KGSL_MMU_SECURE_PT:
		return _init_secure_pt(mmu, pt);

	default:
		return _init_per_process_pt(mmu, pt);
	}
}

static struct kgsl_pagetable *kgsl_iommu_getpagetable(struct kgsl_mmu *mmu,
		unsigned long name)
{
	struct kgsl_pagetable *pt;

	if (!kgsl_mmu_is_perprocess(mmu) && (name != KGSL_MMU_SECURE_PT)) {
		name = KGSL_MMU_GLOBAL_PT;
		if (mmu->defaultpagetable != NULL)
			return mmu->defaultpagetable;
	}

	pt = kgsl_get_pagetable(name);
	if (pt == NULL)
		pt = kgsl_mmu_createpagetableobject(mmu, name);

	return pt;
=======
	int ret = 0;
	struct kgsl_iommu_pt *iommu_pt = NULL;
	struct bus_type *bus = &platform_bus_type;
	int disable_htw = 1;

	if (pt == NULL)
		return -EINVAL;

	if (KGSL_MMU_SECURE_PT == pt->name) {
		if (!mmu->secured)
			return -EPERM;

		if (!MMU_FEATURE(mmu, KGSL_MMU_HYP_SECURE_ALLOC)) {
			bus = get_secure_bus();
			if (bus == NULL)
				return -EPERM;
		}
	}

	iommu_pt = kzalloc(sizeof(struct kgsl_iommu_pt), GFP_KERNEL);
	if (!iommu_pt)
		return -ENOMEM;

	iommu_pt->domain = iommu_domain_alloc(bus);
	if (iommu_pt->domain == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	/* Disable coherent HTW, it is not supported by SMMU driver */
	iommu_domain_set_attr(iommu_pt->domain,
			DOMAIN_ATTR_COHERENT_HTW_DISABLE, &disable_htw);

	pt->pt_ops = &iommu_pt_ops;
	pt->priv = iommu_pt;

	if (KGSL_MMU_GLOBAL_PT == pt->name)
		iommu_set_fault_handler(iommu_pt->domain,
				kgsl_iommu_fault_handler, pt);
	else if (KGSL_MMU_SECURE_PT != pt->name)
		ret = iommu_domain_get_attr(iommu_pt->domain,
				DOMAIN_ATTR_PT_BASE_ADDR,
				&iommu_pt->pt_base);
	else
		iommu_pt->pt_base = 0;
err:
	if (ret) {
		if (iommu_pt->domain != NULL)
			iommu_domain_free(iommu_pt->domain);
		kfree(iommu_pt);
	}

	return ret;
}

/*
 * kgsl_detach_pagetable_iommu_domain - Detach the IOMMU from a
 * pagetable
 * @mmu - Pointer to the device mmu structure
 * @priv - Flag indicating whether the private or user context is to be
 * detached
 *
 * Detach the IOMMU with the domain that is contained in the
 * hwpagetable of the given mmu. After detaching the IOMMU is not
 * in use because the PTBR will not be set after a detach
 * Return - void
 */
static void kgsl_detach_pagetable_iommu_domain(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu_pt *iommu_pt;
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_context *ctx;
	int i;

	iommu_pt = mmu->defaultpagetable->priv;
	for (i = 0; i < KGSL_IOMMU_CONTEXT_MAX; i++) {
		ctx = &iommu->ctx[i];
		if (ctx->dev == NULL)
			continue;
		if (mmu->securepagetable && (KGSL_IOMMU_CONTEXT_SECURE == i))
			iommu_pt = mmu->securepagetable->priv;
		if (ctx->attached) {
			iommu_detach_device(iommu_pt->domain, ctx->dev);
			ctx->attached = false;
			KGSL_MEM_INFO(mmu->device,
				"iommu %pK detached from user dev of MMU: %pK\n",
				iommu_pt->domain, mmu);
		}
	}
}

/*
 * _iommu_get_clks - get iommu clocks
 * @mmu - Pointer to the device mmu structure
 */
void _iommu_get_clks(struct kgsl_mmu *mmu)
{
	struct kgsl_device_platform_data *pdata =
		dev_get_platdata(&mmu->device->pdev->dev);
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_context  *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	struct kgsl_device_iommu_data *data = pdata->iommu_data;
	struct msm_iommu_drvdata *drvdata = 0;
	int i;

	/* Init IOMMU clks here */
	if (MMU_FEATURE(mmu, KGSL_MMU_DMA_API)) {
		for (i = 0; i < KGSL_IOMMU_MAX_CLKS; i++)
			iommu->clks[i] = data->clks[i];
	} else {
		drvdata = dev_get_drvdata(ctx->dev->parent);
		iommu->clks[0] = drvdata->pclk;
		iommu->clks[1] = drvdata->clk;
		iommu->clks[2] = drvdata->aclk;
		iommu->clks[3] = iommu->gtcu_iface_clk;
		iommu->clks[4] = iommu->gtbu_clk;
		iommu->clks[5] = iommu->gtbu1_clk;
	}
}

/*
 * kgsl_attach_pagetable_iommu_domain - Attach the IOMMU to a
 * pagetable, i.e set the IOMMU's PTBR to the pagetable address and
 * setup other IOMMU registers for the device so that it becomes
 * active
 * @mmu - Pointer to the device mmu structure
 * @priv - Flag indicating whether the private or user context is to be
 * attached
 *
 * Attach the IOMMU with the domain that is contained in the
 * hwpagetable of the given mmu.
 * Return - 0 on success else error code
 */
static int kgsl_attach_pagetable_iommu_domain(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu_pt *iommu_pt;
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_context *ctx;
	int i, ret = 0;

	/*
	 * If retention is supported, iommu hw retains the configuration
	 * on power collapse. If retention is supported we need to attach
	 * only once.
	 */
	if (MMU_FEATURE(mmu, KGSL_MMU_RETENTION) &&
		iommu->ctx[KGSL_IOMMU_CONTEXT_USER].attached)
		return 0;

	/* Loop through all the iommu devices and attach the domain */
	iommu_pt = mmu->defaultpagetable->priv;
	for (i = 0; i < KGSL_IOMMU_CONTEXT_MAX; i++) {

		ctx = &iommu->ctx[i];
		if (ctx->dev == NULL || ctx->attached)
			continue;

		if (KGSL_IOMMU_CONTEXT_SECURE == i) {
			if (mmu->securepagetable)
				iommu_pt = mmu->securepagetable->priv;
			else
				continue;
		}

		ret = iommu_attach_device(iommu_pt->domain, ctx->dev);
		if (ret) {
			KGSL_MEM_ERR(mmu->device,
					"Failed to attach device, err %d\n",
					ret);
			goto done;
		}
		ctx->attached = true;
		KGSL_MEM_INFO(mmu->device,
				"iommu pt %pK attached to dev %pK, ctx_id %d\n",
				iommu_pt->domain, ctx->dev, ctx->ctx_id);
		if (KGSL_IOMMU_CONTEXT_SECURE != i) {
			ret = iommu_domain_get_attr(iommu_pt->domain,
					DOMAIN_ATTR_PT_BASE_ADDR,
					&iommu_pt->pt_base);
			if (ret) {
				KGSL_CORE_ERR(
				  "pt_base query failed, using global pt\n");
				mmu->features |= KGSL_MMU_GLOBAL_PAGETABLE;
				ret = 0;
			}
		}
	}

	_iommu_get_clks(mmu);

done:
	return ret;
}

/*
 * _get_iommu_ctxs - Get device pointer to IOMMU contexts
 * @mmu - Pointer to mmu device
 *
 * Return - 0 on success else error code
 */
static int _get_iommu_ctxs(struct kgsl_mmu *mmu)
{
	struct kgsl_device_platform_data *pdata =
		dev_get_platdata(&mmu->device->pdev->dev);
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_device_iommu_data *data = pdata->iommu_data;
	struct kgsl_iommu_context *ctx;
	int i;
	int ret = 0;

	for (i = 0; i < data->iommu_ctx_count; i++) {
		if (!strcmp("gfx3d_user", data->iommu_ctxs[i].iommu_ctx_name)) {
			ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
		} else if (!strcmp("gfx3d_secure",
				data->iommu_ctxs[i].iommu_ctx_name)) {
			ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_SECURE];
		} else if (!strcmp("gfx3d_spare",
				data->iommu_ctxs[i].iommu_ctx_name)) {
			continue;
		} else if (!strcmp("gfx3d_priv",
				data->iommu_ctxs[i].iommu_ctx_name)) {
			continue;
		} else {
			KGSL_CORE_ERR("dt: IOMMU context %s is invalid\n",
				data->iommu_ctxs[i].iommu_ctx_name);
			return -EINVAL;
		}

		/* Add ctx name here */
		ctx->name = data->iommu_ctxs[i].iommu_ctx_name;

		/* Add device ptr here */
		if (data->iommu_ctxs[i].dev)
			ctx->dev = data->iommu_ctxs[i].dev;
		else
			ctx->dev = msm_iommu_get_ctx(ctx->name);

		/* Add ctx_id here */
		ctx->ctx_id = data->iommu_ctxs[i].ctx_id;

		ctx->kgsldev = mmu->device;

		if ((!ctx->dev) || IS_ERR(ctx->dev)) {
			ret = (!ctx->dev) ? -EINVAL : PTR_ERR(ctx->dev);
			memset(ctx, 0, sizeof(*ctx));
			KGSL_CORE_ERR(
			   "Failed to initialize iommu contexts, err: %d\n",
			   ret);
		}
	}

	return ret;
}

/*
 * _iommu_set_register_map - Map the IOMMU registers
 */
static int _iommu_set_register_map(struct kgsl_mmu *mmu)
{
	struct kgsl_device_platform_data *pdata =
		dev_get_platdata(&mmu->device->pdev->dev);
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	struct kgsl_device_iommu_data *data = pdata->iommu_data;

	/* set iommu features */
	mmu->features = data->features;

	/* set up the IOMMU register map */
	if (!data->regstart || !data->regsize) {
		KGSL_CORE_ERR("The register range for IOMMU not specified\n");
		return -EINVAL;
	}

	iommu->regbase = ioremap(data->regstart, data->regsize);
	if (iommu->regbase == NULL) {
		KGSL_CORE_ERR("Could not map IOMMU registers 0x%X:0x%x\n",
			data->regstart, data->regsize);
		return -ENOMEM;
	}

	iommu->ahb_base_offset = data->regstart - mmu->device->reg_phys;

	return 0;
}

/*
 * kgsl_iommu_get_default_ttbr0 - Return the ttbr0 value programmed by
 * iommu driver
 * @mmu - Pointer to mmu structure
 * @hostptr - Pointer to the IOMMU register map. This is used to match
 * the iommu device whose lsb value is to be returned
 * @ctx_id - The context bank whose lsb valus is to be returned
 * Return - returns the ttbr0 value programmed by iommu driver
 */
static uint64_t kgsl_iommu_get_default_ttbr0(struct kgsl_mmu *mmu,
				enum kgsl_iommu_context_id ctx_id)
{
	struct kgsl_iommu *iommu = mmu->priv;

	if (iommu->ctx[ctx_id].dev)
		return iommu->ctx[ctx_id].default_ttbr0;

	return 0;
>>>>>>> p9x
}

/*
 * kgsl_iommu_get_reg_ahbaddr - Returns the ahb address of the register
 * @mmu - Pointer to mmu structure
 * @id - The context ID of the IOMMU ctx
 * @reg - The register for which address is required
 *
 * Return - The address of register which can be used in type0 packet
 */
static unsigned int kgsl_iommu_get_reg_ahbaddr(struct kgsl_mmu *mmu,
<<<<<<< HEAD
		int id, unsigned int reg)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[id];

	return ctx->gpu_offset + kgsl_iommu_reg_list[reg];
}

static void _detach_context(struct kgsl_iommu_context *ctx)
{
	struct kgsl_iommu_pt *iommu_pt;

	if (ctx->default_pt == NULL)
		return;

	iommu_pt = ctx->default_pt->priv;

	_detach_pt(iommu_pt, ctx);

	ctx->default_pt = NULL;
}

static void kgsl_iommu_close(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	int i;

	for (i = 0; i < KGSL_IOMMU_CONTEXT_MAX; i++)
		_detach_context(&iommu->ctx[i]);

	kgsl_mmu_putpagetable(mmu->defaultpagetable);
	mmu->defaultpagetable = NULL;

	kgsl_mmu_putpagetable(mmu->securepagetable);
	mmu->securepagetable = NULL;

	if (iommu->regbase != NULL)
		iounmap(iommu->regbase);

	kgsl_sharedmem_free(&kgsl_secure_guard_page_memdesc);

	if (kgsl_guard_page != NULL) {
		__free_page(kgsl_guard_page);
		kgsl_guard_page = NULL;
	}

	kgsl_iommu_remove_global(mmu, &iommu->setstate);
	kgsl_sharedmem_free(&iommu->setstate);
	kgsl_cleanup_qdss_desc(mmu);
}

static int _setstate_alloc(struct kgsl_device *device,
		struct kgsl_iommu *iommu)
{
	int ret;

	ret = kgsl_sharedmem_alloc_contig(device, &iommu->setstate, PAGE_SIZE);

	if (!ret) {
		/* Mark the setstate memory as read only */
		iommu->setstate.flags |= KGSL_MEMFLAGS_GPUREADONLY;

		kgsl_sharedmem_set(device, &iommu->setstate, 0, 0, PAGE_SIZE);
	}

	return ret;
=======
		enum kgsl_iommu_context_id id, enum kgsl_iommu_reg_map reg)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int ctx_id = iommu->ctx[id].ctx_id;

	if (iommu->iommu_reg_list[reg].ctx_reg)
		return iommu->ahb_base_offset +
			iommu->iommu_reg_list[reg].reg_offset +
			(ctx_id << KGSL_IOMMU_CTX_SHIFT) +
			iommu->ctx_ahb_offset;
	else
		return iommu->ahb_base_offset +
			iommu->iommu_reg_list[reg].reg_offset;
>>>>>>> p9x
}

static int kgsl_iommu_init(struct kgsl_mmu *mmu)
{
<<<<<<< HEAD
	struct kgsl_device *device = KGSL_MMU_DEVICE(mmu);
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	int status;

	mmu->features |= KGSL_MMU_PAGED;

	if (ctx->name == NULL) {
		KGSL_CORE_ERR("dt: gfx3d0_user context bank not found\n");
		return -EINVAL;
	}

	status = _setstate_alloc(device, iommu);
	if (status)
		return status;

	/* check requirements for per process pagetables */
	if (ctx->gpu_offset == UINT_MAX) {
		KGSL_CORE_ERR("missing qcom,gpu-offset forces global pt\n");
		mmu->features |= KGSL_MMU_GLOBAL_PAGETABLE;
	}

	if (iommu->version == 1 && iommu->micro_mmu_ctrl == UINT_MAX) {
		KGSL_CORE_ERR(
			"missing qcom,micro-mmu-control forces global pt\n");
		mmu->features |= KGSL_MMU_GLOBAL_PAGETABLE;
	}

	/* Check to see if we need to do the IOMMU sync dance */
	need_iommu_sync = of_property_read_bool(device->pdev->dev.of_node,
		"qcom,gpu-quirk-iommu-sync");

	iommu->regbase = ioremap(iommu->regstart, iommu->regsize);
	if (iommu->regbase == NULL) {
		KGSL_CORE_ERR("Could not map IOMMU registers 0x%lx:0x%x\n",
			iommu->regstart, iommu->regsize);
		status = -ENOMEM;
		goto done;
	}

	if (addr_entry_cache == NULL) {
		addr_entry_cache = KMEM_CACHE(kgsl_iommu_addr_entry, 0);
		if (addr_entry_cache == NULL) {
=======
	/*
	 * intialize device mmu
	 *
	 * call this with the global lock held
	 */
	struct adreno_device *adreno_dev = ADRENO_DEVICE(mmu->device);
	int status = 0;
	struct kgsl_iommu *iommu;
	struct platform_device *pdev = mmu->device->pdev;

	atomic_set(&mmu->fault, 0);
	iommu = kzalloc(sizeof(struct kgsl_iommu), GFP_KERNEL);
	if (!iommu)
		return -ENOMEM;

	mmu->priv = iommu;
	status = _get_iommu_ctxs(mmu);
	if (status)
		goto done;
	status = _iommu_set_register_map(mmu);
	if (status)
		goto done;

	if (of_property_match_string(pdev->dev.of_node, "clock-names",
						"gtcu_iface_clk") >= 0)
		iommu->gtcu_iface_clk = clk_get(&pdev->dev, "gtcu_iface_clk");

	/*
	 * For all IOMMUv2 targets, TBU clk needs to be voted before
	 * issuing TLB invalidate
	 */
	if (of_property_match_string(pdev->dev.of_node, "clock-names",
						"gtbu_clk") >= 0)
		iommu->gtbu_clk = clk_get(&pdev->dev, "gtbu_clk");

	if (of_property_match_string(pdev->dev.of_node, "clock-names",
						"gtbu1_clk") >= 0)
		iommu->gtbu1_clk = clk_get(&pdev->dev, "gtbu1_clk");

	if (kgsl_msm_supports_iommu_v2()) {
		if (adreno_is_a5xx(adreno_dev)) {
			iommu->iommu_reg_list = kgsl_iommuv2_reg;
			iommu->ctx_offset = KGSL_IOMMU_CTX_OFFSET_V2_A5XX;
			iommu->ctx_ahb_offset = KGSL_IOMMU_CTX_OFFSET_V2_A5XX;
		} else {
			iommu->iommu_reg_list = kgsl_iommuv1_reg;
			iommu->ctx_ahb_offset = KGSL_IOMMU_CTX_AHB_OFFSET_V2;
			if (adreno_is_a405v2(adreno_dev))
				iommu->ctx_offset =
					KGSL_IOMMU_CTX_OFFSET_A405V2;
			else
				iommu->ctx_offset = KGSL_IOMMU_CTX_OFFSET_V2;
		}
	}  else {
		iommu->iommu_reg_list = kgsl_iommuv1_reg;
		iommu->ctx_offset = KGSL_IOMMU_CTX_OFFSET_V1;
		iommu->ctx_ahb_offset = KGSL_IOMMU_CTX_OFFSET_V1;
	}

	if (kgsl_guard_page == NULL) {
		kgsl_guard_page = alloc_page(GFP_KERNEL | __GFP_ZERO |
				__GFP_HIGHMEM);
		if (kgsl_guard_page == NULL) {
>>>>>>> p9x
			status = -ENOMEM;
			goto done;
		}
	}

<<<<<<< HEAD
	kgsl_iommu_add_global(mmu, &iommu->setstate, "setstate");
	kgsl_setup_qdss_desc(device);

done:
	if (status)
		kgsl_iommu_close(mmu);

	return status;
}

static int _setup_user_context(struct kgsl_mmu *mmu)
{
	int ret = 0;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	struct kgsl_device *device = KGSL_MMU_DEVICE(mmu);
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_iommu_pt *iommu_pt = NULL;
	unsigned int  sctlr_val;

	if (mmu->defaultpagetable == NULL) {
		mmu->defaultpagetable = kgsl_mmu_getpagetable(mmu,
				KGSL_MMU_GLOBAL_PT);
		/* if we don't have a default pagetable, nothing will work */
		if (IS_ERR(mmu->defaultpagetable)) {
			ret = PTR_ERR(mmu->defaultpagetable);
			mmu->defaultpagetable = NULL;
			return ret;
		} else if (mmu->defaultpagetable == NULL) {
			return -ENOMEM;
		}
	}

	iommu_pt = mmu->defaultpagetable->priv;
	if (iommu_pt == NULL)
		return -ENODEV;

	ret = _attach_pt(iommu_pt, ctx);
	if (ret)
		return ret;

	ctx->default_pt = mmu->defaultpagetable;

	kgsl_iommu_enable_clk(mmu);

	sctlr_val = KGSL_IOMMU_GET_CTX_REG(ctx, SCTLR);

	/*
	 * If pagefault policy is GPUHALT_ENABLE,
	 * 1) Program CFCFG to 1 to enable STALL mode
	 * 2) Program HUPCF to 0 (Stall or terminate subsequent
	 *    transactions in the presence of an outstanding fault)
	 * else
	 * 1) Program CFCFG to 0 to disable STALL mode (0=Terminate)
	 * 2) Program HUPCF to 1 (Process subsequent transactions
	 *    independently of any outstanding fault)
	 */

	sctlr_val = KGSL_IOMMU_GET_CTX_REG(ctx, SCTLR);
	if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE,
				&adreno_dev->ft_pf_policy)) {
		sctlr_val |= (0x1 << KGSL_IOMMU_SCTLR_CFCFG_SHIFT);
		sctlr_val &= ~(0x1 << KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
	} else {
		sctlr_val &= ~(0x1 << KGSL_IOMMU_SCTLR_CFCFG_SHIFT);
		sctlr_val |= (0x1 << KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
	}
	KGSL_IOMMU_SET_CTX_REG(ctx, SCTLR, sctlr_val);
	kgsl_iommu_disable_clk(mmu);

	return 0;
}

static int _setup_secure_context(struct kgsl_mmu *mmu)
{
	int ret;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_SECURE];
	unsigned int cb_num;

	struct kgsl_iommu_pt *iommu_pt;

	if (ctx->dev == NULL || !mmu->secured)
		return 0;

	if (mmu->securepagetable == NULL) {
		mmu->securepagetable = kgsl_mmu_getpagetable(mmu,
						KGSL_MMU_SECURE_PT);
		if (IS_ERR(mmu->securepagetable)) {
			ret = PTR_ERR(mmu->securepagetable);
			mmu->securepagetable = NULL;
			return ret;
		} else if (mmu->securepagetable == NULL) {
			return -ENOMEM;
		}
	}
	iommu_pt = mmu->securepagetable->priv;

	ret = _attach_pt(iommu_pt, ctx);
	if (ret)
		goto done;

	ctx->default_pt = mmu->securepagetable;

	ret = iommu_domain_get_attr(iommu_pt->domain, DOMAIN_ATTR_CONTEXT_BANK,
					&cb_num);
	if (ret) {
		KGSL_CORE_ERR("get CONTEXT_BANK attr, err %d\n", ret);
		goto done;
	}
	ctx->cb_num = cb_num;
done:
	if (ret)
		_detach_context(ctx);
	return ret;
}

static int kgsl_iommu_set_pt(struct kgsl_mmu *mmu, struct kgsl_pagetable *pt);

static int kgsl_iommu_start(struct kgsl_mmu *mmu)
{
	int status;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);

	status = _setup_user_context(mmu);
	if (status)
		return status;

	status = _setup_secure_context(mmu);
	if (status) {
		_detach_context(&iommu->ctx[KGSL_IOMMU_CONTEXT_USER]);
		return status;
	}

	/* Make sure the hardware is programmed to the default pagetable */
	return kgsl_iommu_set_pt(mmu, mmu->defaultpagetable);
}

static int
kgsl_iommu_unmap_offset(struct kgsl_pagetable *pt,
		struct kgsl_memdesc *memdesc, uint64_t addr,
		uint64_t offset, uint64_t size)
{
	/*
	 * All GPU addresses as assigned are page aligned, but some
	 * functions perturb the gpuaddr with an offset, so apply the
	 * mask here to make sure we have the right address.
	 */

	addr = PAGE_ALIGN(addr);

	if (size == 0 || addr == 0)
		return 0;

	return _iommu_unmap_sync_pc(pt, memdesc, addr + offset, size);
}

static int
kgsl_iommu_unmap(struct kgsl_pagetable *pt, struct kgsl_memdesc *memdesc)
{
	uint64_t size = memdesc->size;

	if (kgsl_memdesc_has_guard_page(memdesc))
		size += kgsl_memdesc_guard_page_size(pt->mmu, memdesc);

	return kgsl_iommu_unmap_offset(pt, memdesc, memdesc->gpuaddr, 0,
			size);
}

/**
 * _iommu_map_guard_page - Map iommu guard page
=======
done:
	if (status) {
		kfree(iommu);
		mmu->priv = NULL;
	}
	return status;
}

/*
 * kgsl_iommu_setup_defaultpagetable - Setup the initial defualtpagetable
 * for iommu. This function is only called once during first start, successive
 * start do not call this funciton.
 * @mmu - Pointer to mmu structure
 *
 * Create the  initial defaultpagetable and setup the iommu mappings to it
 * Return - 0 on success else error code
 */
static int kgsl_iommu_setup_defaultpagetable(struct kgsl_mmu *mmu)
{
	int status = 0;

	mmu->defaultpagetable = kgsl_mmu_getpagetable(mmu, KGSL_MMU_GLOBAL_PT);
	/* Return error if the default pagetable doesn't exist */
	if (mmu->defaultpagetable == NULL) {
		status = -ENOMEM;
		goto err;
	}

	if (mmu->secured) {
		mmu->securepagetable = kgsl_mmu_getpagetable(mmu,
				KGSL_MMU_SECURE_PT);
		/* Return error if the secure pagetable doesn't exist */
		if (mmu->securepagetable == NULL) {
			KGSL_DRV_ERR(mmu->device,
			"Unable to create secure pagetable, disable content protection\n");
			status = -ENOMEM;
			goto err;
		}
	}
	return status;
err:
	if (mmu->defaultpagetable) {
		kgsl_mmu_putpagetable(mmu->defaultpagetable);
		mmu->defaultpagetable = NULL;
	}
	return status;
}

static int kgsl_iommu_start(struct kgsl_mmu *mmu)
{
	int status;
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_context *ctx;
	int i;
	int sctlr_val = 0;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(mmu->device);

	if (mmu->defaultpagetable == NULL) {
		status = kgsl_iommu_setup_defaultpagetable(mmu);
		if (status)
			return -ENOMEM;
	}

	status = kgsl_attach_pagetable_iommu_domain(mmu);
	if (status)
		goto done;

	kgsl_map_global_pt_entries(mmu->defaultpagetable);

	kgsl_iommu_enable_clk(mmu);
	KGSL_IOMMU_SET_CTX_REG(iommu, 0, TLBIALL, 1);

	/* Get the lsb value of pagetables set in the IOMMU ttbr0 register as
	 * that value should not change when we change pagetables, so while
	 * changing pagetables we can use this lsb value of the pagetable w/o
	 * having to read it again
	 */
	for (i = 0; i < KGSL_IOMMU_CONTEXT_MAX; i++) {
		ctx = &iommu->ctx[i];

		if (ctx->dev == NULL)
			continue;

		/*
		 *  1) HLOS cannot program secure context bank.
		 *  2) If context bank is not attached skip.
		 */
		if (!ctx->attached || (KGSL_IOMMU_CONTEXT_SECURE == i))
			continue;

		/*
		 * For IOMMU V1 do not halt IOMMU on pagefault if
		 * FT pagefault policy is set accordingly
		 */

		if (!test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE,
				&adreno_dev->ft_pf_policy)) {
			sctlr_val = KGSL_IOMMU_GET_CTX_REG(iommu,
					ctx->ctx_id, SCTLR);

			sctlr_val |= (0x1 << KGSL_IOMMU_SCTLR_HUPCF_SHIFT);

			KGSL_IOMMU_SET_CTX_REG(iommu, ctx->ctx_id,
					SCTLR, sctlr_val);
		}
		ctx->default_ttbr0 = KGSL_IOMMU_GET_CTX_REG_Q(iommu,
					ctx->ctx_id, TTBR0);
	}

	kgsl_iommu_disable_clk(mmu);

done:
	return status;
}

/*
 * kgsl_iommu_flush_tlb_pt_current - Flush IOMMU TLB if pagetable is
 * currently used by GPU.
 * @pt - Pointer to kgsl pagetable structure
 *
 * Return - void
 */
static void kgsl_iommu_flush_tlb_pt_current(struct kgsl_pagetable *pt,
				struct kgsl_memdesc *memdesc)
{
	struct kgsl_iommu *iommu = pt->mmu->priv;

	if (kgsl_memdesc_is_secured(memdesc))
		return;

	mutex_lock(&pt->mmu->device->mutex);
	/*
	 * Flush the tlb only if the iommu device is attached and the pagetable
	 * hasn't been switched yet
	 */
	if (kgsl_mmu_is_perprocess(pt->mmu) &&
		iommu->ctx[KGSL_IOMMU_CONTEXT_USER].attached &&
		kgsl_iommu_pt_equal(pt->mmu, pt,
		kgsl_iommu_get_current_ptbase(pt->mmu)))
		kgsl_iommu_flush_pt(pt->mmu);
	mutex_unlock(&pt->mmu->device->mutex);
}

static int
kgsl_iommu_unmap(struct kgsl_pagetable *pt,
		struct kgsl_memdesc *memdesc)
{
	struct kgsl_device *device = pt->mmu->device;
	int ret = 0;
	uint64_t range = memdesc->size;
	size_t unmapped = 0;
	struct kgsl_iommu_pt *iommu_pt = pt->priv;

	/* All GPU addresses as assigned are page aligned, but some
	   functions purturb the gpuaddr with an offset, so apply the
	   mask here to make sure we have the right address */

	uint64_t gpuaddr = PAGE_ALIGN(memdesc->gpuaddr);

	if (range == 0 || gpuaddr == 0)
		return 0;

	if (kgsl_memdesc_has_guard_page(memdesc))
		range += kgsl_memdesc_guard_page_size(memdesc);

	if (kgsl_memdesc_is_secured(memdesc)) {

		if (!kgsl_mmu_is_secured(pt->mmu))
			return -EINVAL;

		mutex_lock(&device->mutex);
		ret = kgsl_active_count_get(device);
		if (!ret) {
			unmapped = iommu_unmap(iommu_pt->domain, gpuaddr,
					range);
			kgsl_active_count_put(device);
		}
		mutex_unlock(&device->mutex);
	} else
		unmapped = iommu_unmap(iommu_pt->domain, gpuaddr, range);
	if (unmapped != range) {
		KGSL_CORE_ERR(
			"iommu_unmap(%llx, %lld) failed with unmapped size: %zd\n",
			gpuaddr, range, unmapped);
		return -EINVAL;
	}

	/*
	 * We only need to flush the TLB for non-global memory.
	 * This is because global mappings are only removed at pagetable destroy
	 * time, and the pagetable is not active in the TLB at this point.
	 */
	if (!kgsl_memdesc_is_global(memdesc))
		kgsl_iommu_flush_tlb_pt_current(pt, memdesc);

	return ret;
}

/*
 * _create_sg_no_large_pages - Create a sg list from a given sg list w/o
 * greater that 64K pages
 * @memdesc - The memory descriptor containing the sg
 * @nents - [output] the number of entries in the new scatterlist
 *
 * Returns the new sg list else error pointer on failure
 */
struct scatterlist *_create_sg_no_large_pages(struct kgsl_memdesc *memdesc,
					int *nents)
{
	struct page *page;
	struct scatterlist *s, *s_temp, *sg_temp;
	unsigned int sglen_alloc = 0, offset, i, len;
	uint64_t gpuaddr = memdesc->gpuaddr;

	for_each_sg(memdesc->sgt->sgl, s, memdesc->sgt->nents, i) {
		/*
		 * We need to unchunk any fully aligned 1M chunks
		 * to trick the iommu driver into not using 1M
		 * PTEs
		 */
		if (SZ_1M <= s->length
			&& IS_ALIGNED(gpuaddr, SZ_1M)
			&& IS_ALIGNED(sg_phys(s), SZ_1M)) {
			sglen_alloc += ALIGN(s->length, SZ_64K)/SZ_64K;
			/* possible that length is not 64KB aligned */
			sglen_alloc += (s->length & (SZ_64K - 1))/SZ_4K;
		}
		else
			sglen_alloc++;
		gpuaddr += s->length;
	}
	/* No large pages were detected */
	if (sglen_alloc == memdesc->sgt->nents)
		return NULL;

	sg_temp = kgsl_malloc(sglen_alloc * sizeof(struct scatterlist));
	if (NULL == sg_temp)
		return ERR_PTR(-ENOMEM);

	sg_init_table(sg_temp, sglen_alloc);
	s_temp = sg_temp;

	gpuaddr = memdesc->gpuaddr;
	for_each_sg(memdesc->sgt->sgl, s, memdesc->sgt->nents, i) {
		page = sg_page(s);
		if (SZ_1M <= s->length
			&& IS_ALIGNED(gpuaddr, SZ_1M)
			&& IS_ALIGNED(sg_phys(s), SZ_1M)) {
			for (offset = 0; offset < s->length; s_temp++) {
				/* the last chunk might be smaller than 64K */
				len =
				((unsigned int)(s->length - offset) >= SZ_64K) ?
					SZ_64K : SZ_4K;
				sg_set_page(s_temp, page, len, offset);
				offset += len;
			}
		} else {
			sg_set_page(s_temp, page, s->length, 0);
			s_temp++;
		}
		gpuaddr += s->length;
	}
	*nents = sglen_alloc;
	return sg_temp;
}

/**
 * _iommu_add_guard_page - Add iommu guard page
>>>>>>> p9x
 * @pt - Pointer to kgsl pagetable structure
 * @memdesc - memdesc to add guard page
 * @gpuaddr - GPU addr of guard page
 * @protflags - flags for mapping
 *
 * Return 0 on success, error on map fail
 */
<<<<<<< HEAD
static int _iommu_map_guard_page(struct kgsl_pagetable *pt,
				   struct kgsl_memdesc *memdesc,
				   uint64_t gpuaddr,
				   unsigned int protflags)
{
	phys_addr_t physaddr;

	if (!kgsl_memdesc_has_guard_page(memdesc))
		return 0;

	/*
	 * Allocate guard page for secure buffers.
	 * This has to be done after we attach a smmu pagetable.
	 * Allocate the guard page when first secure buffer is.
	 * mapped to save 1MB of memory if CPZ is not used.
	 */
	if (kgsl_memdesc_is_secured(memdesc)) {
		struct scatterlist *sg;
		unsigned int sgp_size = pt->mmu->secure_align_mask + 1;

		if (!kgsl_secure_guard_page_memdesc.sgt) {
			if (kgsl_allocate_user(KGSL_MMU_DEVICE(pt->mmu),
					&kgsl_secure_guard_page_memdesc,
					sgp_size, KGSL_MEMFLAGS_SECURE)) {
				KGSL_CORE_ERR(
					"Secure guard page alloc failed\n");
				return -ENOMEM;
			}
		}

		sg = kgsl_secure_guard_page_memdesc.sgt->sgl;
		physaddr = page_to_phys(sg_page(sg));
	} else {
		if (kgsl_guard_page == NULL) {
			kgsl_guard_page = alloc_page(GFP_KERNEL | __GFP_ZERO |
					__GFP_HIGHMEM);
			if (kgsl_guard_page == NULL)
				return -ENOMEM;
		}

		physaddr = page_to_phys(kgsl_guard_page);
	}

	return _iommu_map_sync_pc(pt, memdesc, gpuaddr, physaddr,
			kgsl_memdesc_guard_page_size(pt->mmu, memdesc),
			protflags & ~IOMMU_WRITE);
}

static unsigned int _get_protection_flags(struct kgsl_memdesc *memdesc)
{
	unsigned int flags = IOMMU_READ | IOMMU_WRITE | IOMMU_NOEXEC;

	if (memdesc->flags & KGSL_MEMFLAGS_GPUREADONLY)
		flags &= ~IOMMU_WRITE;

	if (memdesc->priv & KGSL_MEMDESC_PRIVILEGED)
		flags |= IOMMU_PRIV;

	return flags;
=======
int _iommu_add_guard_page(struct kgsl_pagetable *pt,
						   struct kgsl_memdesc *memdesc,
						   uint64_t gpuaddr,
						   unsigned int protflags)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	phys_addr_t physaddr = page_to_phys(kgsl_guard_page);
	int ret;

	if (kgsl_memdesc_has_guard_page(memdesc)) {

		/*
		 * Allocate guard page for secure buffers.
		 * This has to be done after we attach a smmu pagetable.
		 * Allocate the guard page when first secure buffer is.
		 * mapped to save 1MB of memory if CPZ is not used.
		 */
		if (kgsl_memdesc_is_secured(memdesc)) {
			if (!kgsl_secure_guard_page_memdesc.physaddr) {
				if (kgsl_cma_alloc_secure(pt->mmu->device,
					&kgsl_secure_guard_page_memdesc,
					SZ_1M)) {
					KGSL_CORE_ERR(
					"Secure guard page alloc failed\n");
					return -ENOMEM;
				}
			}

			physaddr = kgsl_secure_guard_page_memdesc.physaddr;
		}

		ret = iommu_map(iommu_pt->domain, gpuaddr, physaddr,
				kgsl_memdesc_guard_page_size(memdesc),
				protflags & ~IOMMU_WRITE);
		if (ret) {
			KGSL_CORE_ERR(
			"iommu_map(addr %016llX, flags %x) err: %d\n",
			gpuaddr, protflags & ~IOMMU_WRITE,
			ret);
			return ret;
		}
	}

	return 0;
>>>>>>> p9x
}

static int
kgsl_iommu_map(struct kgsl_pagetable *pt,
			struct kgsl_memdesc *memdesc)
{
<<<<<<< HEAD
	int ret;
	uint64_t addr = memdesc->gpuaddr;
	uint64_t size = memdesc->size;
	unsigned int flags = _get_protection_flags(memdesc);
	struct sg_table *sgt = NULL;

	/*
	 * For paged memory allocated through kgsl, memdesc->pages is not NULL.
	 * Allocate sgt here just for its map operation. Contiguous memory
	 * already has its sgt, so no need to allocate it here.
	 */
	if (memdesc->pages != NULL)
		sgt = kgsl_alloc_sgt_from_pages(memdesc);
	else
		sgt = memdesc->sgt;

	if (IS_ERR(sgt))
		return PTR_ERR(sgt);

	ret = _iommu_map_sg_sync_pc(pt, addr, memdesc, sgt->sgl,
				sgt->nents, flags);
	if (ret)
		goto done;

	ret = _iommu_map_guard_page(pt, memdesc, addr + size, flags);
	if (ret)
		_iommu_unmap_sync_pc(pt, memdesc, addr, size);

done:
	if (memdesc->pages != NULL)
		kgsl_free_sgt(sgt);

	return ret;
}

static int kgsl_iommu_map_offset(struct kgsl_pagetable *pt,
		uint64_t virtaddr, uint64_t virtoffset,
		struct kgsl_memdesc *memdesc, uint64_t physoffset,
		uint64_t size, uint64_t feature_flag)
{
	int pg_sz;
	unsigned int protflags = _get_protection_flags(memdesc);
	int ret;
	struct sg_table *sgt = NULL;

	pg_sz = (1 << kgsl_memdesc_get_align(memdesc));
	if (!IS_ALIGNED(virtaddr | virtoffset | physoffset | size, pg_sz))
		return -EINVAL;

	if (size == 0)
		return -EINVAL;

	/*
	 * For paged memory allocated through kgsl, memdesc->pages is not NULL.
	 * Allocate sgt here just for its map operation. Contiguous memory
	 * already has its sgt, so no need to allocate it here.
	 */
	if (memdesc->pages != NULL)
		sgt = kgsl_alloc_sgt_from_pages(memdesc);
	else
		sgt = memdesc->sgt;

	if (IS_ERR(sgt))
		return PTR_ERR(sgt);

	ret = _iommu_map_sg_offset_sync_pc(pt, virtaddr + virtoffset,
		memdesc, sgt->sgl, sgt->nents,
		physoffset, size, protflags);

	if (memdesc->pages != NULL)
		kgsl_free_sgt(sgt);

	return ret;
}

/* This function must be called with context bank attached */
static void kgsl_iommu_clear_fsr(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context  *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	unsigned int sctlr_val;

	if (ctx->default_pt != NULL) {
		kgsl_iommu_enable_clk(mmu);
		KGSL_IOMMU_SET_CTX_REG(ctx, FSR, 0xffffffff);
		/*
		 * Re-enable context fault interrupts after clearing
		 * FSR to prevent the interrupt from firing repeatedly
		 */
		sctlr_val = KGSL_IOMMU_GET_CTX_REG(ctx, SCTLR);
		sctlr_val |= (0x1 << KGSL_IOMMU_SCTLR_CFIE_SHIFT);
		KGSL_IOMMU_SET_CTX_REG(ctx, SCTLR, sctlr_val);
		/*
		 * Make sure the above register writes
		 * are not reordered across the barrier
		 * as we use writel_relaxed to write them
		 */
		wmb();
		kgsl_iommu_disable_clk(mmu);
	}
=======
	int ret = 0;
	uint64_t addr;
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	uint64_t size = memdesc->size;
	unsigned int flags = 0;
	struct kgsl_device *device = pt->mmu->device;
	size_t mapped = 0;

	BUG_ON(NULL == iommu_pt);

	BUG_ON(memdesc->gpuaddr > UINT_MAX);

	addr = (unsigned int) memdesc->gpuaddr;

	flags = IOMMU_READ | IOMMU_WRITE;

	/* Set up the protection for the page(s) */
	if (memdesc->flags & KGSL_MEMFLAGS_GPUREADONLY)
		flags &= ~IOMMU_WRITE;

	if (memdesc->priv & KGSL_MEMDESC_PRIVILEGED)
		flags |= IOMMU_PRIV;

	if (kgsl_memdesc_is_secured(memdesc)) {

		if (!kgsl_mmu_is_secured(pt->mmu))
			return -EINVAL;

		mutex_lock(&device->mutex);
		ret = kgsl_active_count_get(device);
		if (!ret) {
			mapped = iommu_map_sg(iommu_pt->domain, addr,
					memdesc->sgt->sgl, memdesc->sgt->nents,
					flags);
			kgsl_active_count_put(device);
		}
		mutex_unlock(&device->mutex);
	} else {
		struct scatterlist *sg_temp = NULL;
		int sg_temp_nents;

		sg_temp = _create_sg_no_large_pages(memdesc, &sg_temp_nents);

		if (IS_ERR(sg_temp))
			return PTR_ERR(sg_temp);

		mapped = iommu_map_sg(iommu_pt->domain, addr,
				sg_temp ? sg_temp : memdesc->sgt->sgl,
				sg_temp ? sg_temp_nents : memdesc->sgt->nents,
				flags);

		kgsl_free(sg_temp);
	}

	if (mapped != size) {
		KGSL_CORE_ERR("iommu_map_sg(%016llX, %lld, %x) err: %zd\n",
				addr, size,	flags, mapped);
		return -ENODEV;
	}

	ret = _iommu_add_guard_page(pt, memdesc, addr + size, flags);
	if (ret)
		/* cleanup the partial mapping */
		iommu_unmap(iommu_pt->domain, addr, size);

	/*
	 *  IOMMU V1 BFBs pre-fetch data beyond what is being used by the core.
	 *  This can include both allocated pages and un-allocated pages.
	 *  If an un-allocated page is cached, and later used (if it has been
	 *  newly dynamically allocated by SW) the SMMU HW should automatically
	 *  re-fetch the pages from memory (rather than using the cached
	 *  un-allocated page). This logic is known as the re-fetch logic.
	 *  In current chips we suspect this re-fetch logic is broken,
	 *  it can result in bad translations which can either cause downstream
	 *  bus errors, or upstream cores being hung (because of garbage data
	 *  being read) -> causing TLB sync stuck issues. As a result SW must
	 *  implement the invalidate+map.
	 *
	 * We only need to flush the TLB for non-global memory.
	 * This is because global mappings are only created at pagetable create
	 * time, and the pagetable is not active in the TLB at this point.
	 */

	if (MMU_FEATURE(pt->mmu, KGSL_MMU_FLUSH_TLB_ON_MAP)
		&& !kgsl_memdesc_is_global(memdesc))
		kgsl_iommu_flush_tlb_pt_current(pt, memdesc);

	return ret;
>>>>>>> p9x
}

static void kgsl_iommu_pagefault_resume(struct kgsl_mmu *mmu)
{
<<<<<<< HEAD
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];

	if (ctx->default_pt != NULL && ctx->fault) {
		/*
		 * Write 1 to RESUME.TnR to terminate the
		 * stalled transaction.
		 */
		KGSL_IOMMU_SET_CTX_REG(ctx, RESUME, 1);
		/*
		 * Make sure the above register writes
		 * are not reordered across the barrier
		 * as we use writel_relaxed to write them
		 */
		wmb();
		ctx->fault = 0;
	}
}

static void kgsl_iommu_stop(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	int i;

	/*
	 * If the iommu supports retention, we don't need
	 * to detach when stopping.
	 */
	if (!MMU_FEATURE(mmu, KGSL_MMU_RETENTION)) {
		for (i = 0; i < KGSL_IOMMU_CONTEXT_MAX; i++)
			_detach_context(&iommu->ctx[i]);
	}
}

static u64
kgsl_iommu_get_current_ttbr0(struct kgsl_mmu *mmu)
{
	u64 val;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	/*
	 * We cannot enable or disable the clocks in interrupt context, this
	 * function is called from interrupt context if there is an axi error
	 */
	if (in_interrupt())
		return 0;

	kgsl_iommu_enable_clk(mmu);
	val = KGSL_IOMMU_GET_CTX_REG_Q(&iommu->ctx[KGSL_IOMMU_CONTEXT_USER],
					TTBR0);
	kgsl_iommu_disable_clk(mmu);
	return val;
=======
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_context *ctx;
	int i;

	if (atomic_read(&mmu->fault) == 0)
		return;

	kgsl_iommu_enable_clk(mmu);
	for (i = 0; i < KGSL_IOMMU_CONTEXT_MAX; i++) {
		ctx = &iommu->ctx[i];
		if (ctx->dev == NULL)
			continue;

		/*
		 *  1) HLOS cannot program secure context bank.
		 *  2) If context bank is not attached skip.
		 */
		if (!ctx->attached || (KGSL_IOMMU_CONTEXT_SECURE == i))
			continue;

		if (ctx->fault) {
			KGSL_IOMMU_SET_CTX_REG(iommu, ctx->ctx_id, RESUME, 1);
			KGSL_IOMMU_SET_CTX_REG(iommu, ctx->ctx_id, FSR, 0);
			ctx->fault = 0;
		}
	}
	kgsl_iommu_disable_clk(mmu);
	atomic_set(&mmu->fault, 0);
}


static void kgsl_iommu_stop(struct kgsl_mmu *mmu)
{
	/*
	 *  stop device mmu
	 *
	 *  call this with the global lock held
	 *  detach iommu attachment
	 */
	/* detach iommu attachment */
	if (!MMU_FEATURE(mmu, KGSL_MMU_RETENTION))
		kgsl_detach_pagetable_iommu_domain(mmu);

	kgsl_iommu_pagefault_resume(mmu);
}

static int kgsl_iommu_close(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;

	if (mmu->defaultpagetable != NULL)
		kgsl_mmu_putpagetable(mmu->defaultpagetable);

	if (iommu->regbase != NULL)
		iounmap(iommu->regbase);

	kfree(iommu);

	if (kgsl_guard_page != NULL) {
		__free_page(kgsl_guard_page);
		kgsl_guard_page = NULL;
	}

	kgsl_sharedmem_free(&kgsl_secure_guard_page_memdesc);

	return 0;
}

static phys_addr_t
kgsl_iommu_get_current_ptbase(struct kgsl_mmu *mmu)
{
	phys_addr_t pt_base;
	struct kgsl_iommu *iommu = mmu->priv;
	/* We cannot enable or disable the clocks in interrupt context, this
	 function is called from interrupt context if there is an axi error */
	if (in_interrupt())
		return 0;
	/* Return the current pt base by reading IOMMU pt_base register */
	kgsl_iommu_enable_clk(mmu);
	pt_base = KGSL_IOMMU_GET_CTX_REG_Q(iommu,
		iommu->ctx[KGSL_IOMMU_CONTEXT_USER].ctx_id, TTBR0);
	kgsl_iommu_disable_clk(mmu);
	return pt_base & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
}

/*
 * kgsl_iommu_flush_pt - Flush the IOMMU pagetable
 * @mmu - Pointer to mmu structure
 */
static int kgsl_iommu_flush_pt(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	unsigned long wait_for_flush;
	unsigned int tlbflush_ctxt =
		iommu->ctx[KGSL_IOMMU_CONTEXT_USER].ctx_id;
	int ret = 0;

	kgsl_iommu_enable_clk(mmu);

	KGSL_IOMMU_SET_CTX_REG(iommu, tlbflush_ctxt, TLBIALL, 1);
	mb();
	/*
	 * Wait for flush to complete by polling the flush
	 * status bit of TLBSTATUS register for not more than
	 * 2 s. After 2s just exit, at that point the SMMU h/w
	 * may be stuck and will eventually cause GPU to hang
	 * or bring the system down.
	 */
	wait_for_flush = jiffies + msecs_to_jiffies(2000);
	KGSL_IOMMU_SET_CTX_REG(iommu, tlbflush_ctxt, TLBSYNC, 0);
	while (KGSL_IOMMU_GET_CTX_REG(iommu, tlbflush_ctxt, TLBSTATUS) &
		(KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE)) {
		if (time_after(jiffies, wait_for_flush)) {
			KGSL_DRV_WARN(mmu->device,
			"Wait limit reached for IOMMU tlb flush\n");
			break;
		}
		cpu_relax();
	}


	/* Disable smmu clock */
	kgsl_iommu_disable_clk(mmu);

	return ret;
>>>>>>> p9x
}

/*
 * kgsl_iommu_set_pt - Change the IOMMU pagetable of the primary context bank
 * @mmu - Pointer to mmu structure
 * @pt - Pagetable to switch to
 *
 * Set the new pagetable for the IOMMU by doing direct register writes
 * to the IOMMU registers through the cpu
 *
 * Return - void
 */
<<<<<<< HEAD
static int kgsl_iommu_set_pt(struct kgsl_mmu *mmu, struct kgsl_pagetable *pt)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	uint64_t ttbr0, temp;
	unsigned int contextidr;
	unsigned long wait_for_flush;

	if ((pt != mmu->defaultpagetable) && !kgsl_mmu_is_perprocess(mmu))
		return 0;

	kgsl_iommu_enable_clk(mmu);

	ttbr0 = kgsl_mmu_pagetable_get_ttbr0(pt);
	contextidr = kgsl_mmu_pagetable_get_contextidr(pt);

	KGSL_IOMMU_SET_CTX_REG_Q(ctx, TTBR0, ttbr0);
	KGSL_IOMMU_SET_CTX_REG(ctx, CONTEXTIDR, contextidr);

	mb();
	temp = KGSL_IOMMU_GET_CTX_REG_Q(ctx, TTBR0);

	KGSL_IOMMU_SET_CTX_REG(ctx, TLBIALL, 1);
	/* make sure the TBLI write completes before we wait */
	mb();
	/*
	 * Wait for flush to complete by polling the flush
	 * status bit of TLBSTATUS register for not more than
	 * 2 s. After 2s just exit, at that point the SMMU h/w
	 * may be stuck and will eventually cause GPU to hang
	 * or bring the system down.
	 */
	wait_for_flush = jiffies + msecs_to_jiffies(2000);
	KGSL_IOMMU_SET_CTX_REG(ctx, TLBSYNC, 0);
	while (KGSL_IOMMU_GET_CTX_REG(ctx, TLBSTATUS) &
		(KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE)) {
		if (time_after(jiffies, wait_for_flush)) {
			KGSL_DRV_WARN(KGSL_MMU_DEVICE(mmu),
			"Wait limit reached for IOMMU tlb flush\n");
			break;
		}
		cpu_relax();
	}

	kgsl_iommu_disable_clk(mmu);
	return 0;
=======
static int kgsl_iommu_set_pt(struct kgsl_mmu *mmu,
				struct kgsl_pagetable *pt)
{
	struct kgsl_iommu *iommu = mmu->priv;
	uint ctx_id = iommu->ctx[KGSL_IOMMU_CONTEXT_USER].ctx_id;
	int temp;
	int ret = 0;
	phys_addr_t pt_base;
	uint64_t pt_val;

	kgsl_iommu_enable_clk(mmu);

	pt_base = kgsl_iommu_get_pt_base_addr(mmu, pt);

	/*
	 * Taking the liberty to spin idle since this codepath
	 * is invoked when we can spin safely for it to be idle
	 */
	ret = adreno_spin_idle(mmu->device);
	if (ret)
		return ret;

	/* get the lsb value which should not change when changing ttbr0 */
	pt_val = kgsl_iommu_get_default_ttbr0(mmu, KGSL_IOMMU_CONTEXT_USER);

	pt_base &= KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
	pt_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
	pt_val |= pt_base;
	KGSL_IOMMU_SET_CTX_REG_Q(iommu, ctx_id, TTBR0, pt_val);

	mb();
	temp = KGSL_IOMMU_GET_CTX_REG_Q(iommu, ctx_id, TTBR0);

	kgsl_iommu_flush_pt(mmu);

	/* Disable smmu clock */
	kgsl_iommu_disable_clk(mmu);

	return ret;
>>>>>>> p9x
}

/*
 * kgsl_iommu_set_pf_policy() - Set the pagefault policy for IOMMU
 * @mmu: Pointer to mmu structure
 * @pf_policy: The pagefault polict to set
 *
 * Check if the new policy indicated by pf_policy is same as current
 * policy, if same then return else set the policy
 */
static int kgsl_iommu_set_pf_policy(struct kgsl_mmu *mmu,
				unsigned long pf_policy)
{
<<<<<<< HEAD
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	struct kgsl_device *device = KGSL_MMU_DEVICE(mmu);
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
=======
	int i;
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_context  *ctx;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(mmu->device);
	int ret = 0;
	unsigned int sctlr_val;
>>>>>>> p9x

	if ((adreno_dev->ft_pf_policy &
		BIT(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE)) ==
		(pf_policy & BIT(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE)))
		return 0;

<<<<<<< HEAD
	/* If not attached, policy will be updated during the next attach */
	if (ctx->default_pt != NULL) {
		unsigned int sctlr_val;

		kgsl_iommu_enable_clk(mmu);

		sctlr_val = KGSL_IOMMU_GET_CTX_REG(ctx, SCTLR);

		if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE, &pf_policy)) {
			sctlr_val |= (0x1 << KGSL_IOMMU_SCTLR_CFCFG_SHIFT);
			sctlr_val &= ~(0x1 << KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
		} else {
			sctlr_val &= ~(0x1 << KGSL_IOMMU_SCTLR_CFCFG_SHIFT);
			sctlr_val |= (0x1 << KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
		}

		KGSL_IOMMU_SET_CTX_REG(ctx, SCTLR, sctlr_val);

		kgsl_iommu_disable_clk(mmu);
	}

	return 0;
}

static struct kgsl_protected_registers *
kgsl_iommu_get_prot_regs(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);

	return &iommu->protect;
}

static struct kgsl_iommu_addr_entry *_find_gpuaddr(
		struct kgsl_pagetable *pagetable, uint64_t gpuaddr)
{
	struct kgsl_iommu_pt *pt = pagetable->priv;
	struct rb_node *node = pt->rbtree.rb_node;

	while (node != NULL) {
		struct kgsl_iommu_addr_entry *entry = rb_entry(node,
			struct kgsl_iommu_addr_entry, node);

		if (gpuaddr < entry->base)
			node = node->rb_left;
		else if (gpuaddr > entry->base)
			node = node->rb_right;
		else
			return entry;
	}

	return NULL;
}

static int _remove_gpuaddr(struct kgsl_pagetable *pagetable,
		uint64_t gpuaddr)
{
	struct kgsl_iommu_pt *pt = pagetable->priv;
	struct kgsl_iommu_addr_entry *entry;

	entry = _find_gpuaddr(pagetable, gpuaddr);

	if (entry != NULL) {
		rb_erase(&entry->node, &pt->rbtree);
		kmem_cache_free(addr_entry_cache, entry);
		return 0;
	}

	return -ENOMEM;
}

static int _insert_gpuaddr(struct kgsl_pagetable *pagetable,
		uint64_t gpuaddr, uint64_t size)
{
	struct kgsl_iommu_pt *pt = pagetable->priv;
	struct rb_node **node, *parent = NULL;
	struct kgsl_iommu_addr_entry *new =
		kmem_cache_alloc(addr_entry_cache, GFP_ATOMIC);

	if (new == NULL)
		return -ENOMEM;

	new->base = gpuaddr;
	new->size = size;

	node = &pt->rbtree.rb_node;

	while (*node != NULL) {
		struct kgsl_iommu_addr_entry *this;

		parent = *node;
		this = rb_entry(parent, struct kgsl_iommu_addr_entry, node);

		if (new->base < this->base)
			node = &parent->rb_left;
		else if (new->base > this->base)
			node = &parent->rb_right;
		else
			BUG();
	}

	rb_link_node(&new->node, parent, node);
	rb_insert_color(&new->node, &pt->rbtree);

	return 0;
}

static uint64_t _get_unmapped_area(struct kgsl_pagetable *pagetable,
		uint64_t bottom, uint64_t top, uint64_t size,
		uint64_t align)
{
	struct kgsl_iommu_pt *pt = pagetable->priv;
	struct rb_node *node = rb_first(&pt->rbtree);
	uint64_t start;

	bottom = ALIGN(bottom, align);
	start = bottom;

	while (node != NULL) {
		uint64_t gap;
		struct kgsl_iommu_addr_entry *entry = rb_entry(node,
			struct kgsl_iommu_addr_entry, node);

		/*
		 * Skip any entries that are outside of the range, but make sure
		 * to account for some that might straddle the lower bound
		 */
		if (entry->base < bottom) {
			if (entry->base + entry->size > bottom)
				start = ALIGN(entry->base + entry->size, align);
			node = rb_next(node);
			continue;
		}

		/* Stop if we went over the top */
		if (entry->base >= top)
			break;

		/* Make sure there is a gap to consider */
		if (start < entry->base) {
			gap = entry->base - start;

			if (gap >= size)
				return start;
		}

		/* Stop if there is no more room in the region */
		if (entry->base + entry->size >= top)
			return (uint64_t) -ENOMEM;

		/* Start the next cycle at the end of the current entry */
		start = ALIGN(entry->base + entry->size, align);
		node = rb_next(node);
	}

	if (start + size <= top)
		return start;

	return (uint64_t) -ENOMEM;
}

static uint64_t _get_unmapped_area_topdown(struct kgsl_pagetable *pagetable,
		uint64_t bottom, uint64_t top, uint64_t size,
		uint64_t align)
{
	struct kgsl_iommu_pt *pt = pagetable->priv;
	struct rb_node *node = rb_last(&pt->rbtree);
	uint64_t end = top;
	uint64_t mask = ~(align - 1);
	struct kgsl_iommu_addr_entry *entry;

	/* Make sure that the bottom is correctly aligned */
	bottom = ALIGN(bottom, align);

	/* Make sure the requested size will fit in the range */
	if (size > (top - bottom))
		return -ENOMEM;

	/* Walk back through the list to find the highest entry in the range */
	for (node = rb_last(&pt->rbtree); node != NULL; node = rb_prev(node)) {
		entry = rb_entry(node, struct kgsl_iommu_addr_entry, node);
		if (entry->base < top)
			break;
	}

	while (node != NULL) {
		uint64_t offset;

		entry = rb_entry(node, struct kgsl_iommu_addr_entry, node);

		/* If the entire entry is below the range the search is over */
		if ((entry->base + entry->size) < bottom)
			break;

		/* Get the top of the entry properly aligned */
		offset = ALIGN(entry->base + entry->size, align);

		/*
		 * Try to allocate the memory from the top of the gap,
		 * making sure that it fits between the top of this entry and
		 * the bottom of the previous one
		 */

		if ((end > size) && (offset < end)) {
			uint64_t chunk = (end - size) & mask;

			if (chunk >= offset)
				return chunk;
		}

		/*
		 * If we get here and the current entry is outside of the range
		 * then we are officially out of room
		 */

		if (entry->base < bottom)
			return (uint64_t) -ENOMEM;

		/* Set the top of the gap to the current entry->base */
		end = entry->base;

		/* And move on to the next lower entry */
		node = rb_prev(node);
	}

	/* If we get here then there are no more entries in the region */
	if ((end > size) && (((end - size) & mask) >= bottom))
		return (end - size) & mask;

	return (uint64_t) -ENOMEM;
}

static uint64_t kgsl_iommu_find_svm_region(struct kgsl_pagetable *pagetable,
		uint64_t start, uint64_t end, uint64_t size,
		uint64_t alignment)
{
	uint64_t addr;

	/* Avoid black holes */
	BUG_ON(end <= start);

	spin_lock(&pagetable->lock);
	addr = _get_unmapped_area_topdown(pagetable,
			start, end, size, alignment);
	spin_unlock(&pagetable->lock);
	return addr;
}

static int kgsl_iommu_set_svm_region(struct kgsl_pagetable *pagetable,
		uint64_t gpuaddr, uint64_t size)
{
	int ret = -ENOMEM;
	struct kgsl_iommu_pt *pt = pagetable->priv;
	struct rb_node *node;

	/* Make sure the requested address doesn't fall in the global range */
	if (ADDR_IN_GLOBAL(pagetable->mmu, gpuaddr) ||
			ADDR_IN_GLOBAL(pagetable->mmu, gpuaddr + size))
		return -ENOMEM;

	spin_lock(&pagetable->lock);
	node = pt->rbtree.rb_node;

	while (node != NULL) {
		uint64_t start, end;
		struct kgsl_iommu_addr_entry *entry = rb_entry(node,
			struct kgsl_iommu_addr_entry, node);

		start = entry->base;
		end = entry->base + entry->size;

		if (gpuaddr  + size <= start)
			node = node->rb_left;
		else if (end <= gpuaddr)
			node = node->rb_right;
		else
			goto out;
	}

	ret = _insert_gpuaddr(pagetable, gpuaddr, size);
out:
	spin_unlock(&pagetable->lock);
	return ret;
}


static int kgsl_iommu_get_gpuaddr(struct kgsl_pagetable *pagetable,
		struct kgsl_memdesc *memdesc)
{
	struct kgsl_iommu_pt *pt = pagetable->priv;
	int ret = 0;
	uint64_t addr, start, end;
	uint64_t size = memdesc->size;
	unsigned int align;

	BUG_ON(kgsl_memdesc_use_cpu_map(memdesc));

	if (memdesc->flags & KGSL_MEMFLAGS_SECURE &&
			pagetable->name != KGSL_MMU_SECURE_PT)
		return -EINVAL;

	if (kgsl_memdesc_has_guard_page(memdesc))
		size += kgsl_memdesc_guard_page_size(pagetable->mmu, memdesc);

	align = 1 << kgsl_memdesc_get_align(memdesc);

	if (memdesc->flags & KGSL_MEMFLAGS_FORCE_32BIT) {
		start = pt->compat_va_start;
		end = pt->compat_va_end;
	} else {
		start = pt->va_start;
		end = pt->va_end;
	}

	spin_lock(&pagetable->lock);

	addr = _get_unmapped_area(pagetable, start, end, size, align);

	if (addr == (uint64_t) -ENOMEM) {
		ret = -ENOMEM;
		goto out;
	}

	ret = _insert_gpuaddr(pagetable, addr, size);
	if (ret == 0) {
		memdesc->gpuaddr = addr;
		memdesc->pagetable = pagetable;
	}

out:
	spin_unlock(&pagetable->lock);
	return ret;
}

static void kgsl_iommu_put_gpuaddr(struct kgsl_memdesc *memdesc)
{
	if (memdesc->pagetable == NULL)
		return;

	spin_lock(&memdesc->pagetable->lock);

	if (_remove_gpuaddr(memdesc->pagetable, memdesc->gpuaddr))
		BUG();

	spin_unlock(&memdesc->pagetable->lock);
}

static int kgsl_iommu_svm_range(struct kgsl_pagetable *pagetable,
		uint64_t *lo, uint64_t *hi, uint64_t memflags)
{
	struct kgsl_iommu_pt *pt = pagetable->priv;
	bool gpu_compat = (memflags & KGSL_MEMFLAGS_FORCE_32BIT) != 0;

	if (lo != NULL)
		*lo = gpu_compat ? pt->compat_va_start : pt->svm_start;
	if (hi != NULL)
		*hi = gpu_compat ? pt->compat_va_end : pt->svm_end;

	return 0;
}

static bool kgsl_iommu_addr_in_range(struct kgsl_pagetable *pagetable,
		uint64_t gpuaddr)
{
	struct kgsl_iommu_pt *pt = pagetable->priv;

	if (gpuaddr == 0)
		return false;

	if (gpuaddr >= pt->va_start && gpuaddr < pt->va_end)
		return true;

	if (gpuaddr >= pt->compat_va_start && gpuaddr < pt->compat_va_end)
		return true;

	if (gpuaddr >= pt->svm_start && gpuaddr < pt->svm_end)
		return true;

	return false;
}

static const struct {
	int id;
	char *name;
} kgsl_iommu_cbs[] = {
	{ KGSL_IOMMU_CONTEXT_USER, "gfx3d_user", },
	{ KGSL_IOMMU_CONTEXT_SECURE, "gfx3d_secure" },
};

static int _kgsl_iommu_cb_probe(struct kgsl_device *device,
		struct kgsl_iommu *iommu, struct device_node *node)
{
	struct platform_device *pdev = of_find_device_by_node(node);
	struct kgsl_iommu_context *ctx = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(kgsl_iommu_cbs); i++) {
		if (!strcmp(node->name, kgsl_iommu_cbs[i].name)) {
			int id = kgsl_iommu_cbs[i].id;

			ctx = &iommu->ctx[id];
			ctx->id = id;
			ctx->cb_num = -1;
			ctx->name = kgsl_iommu_cbs[i].name;

=======
	kgsl_iommu_enable_clk(mmu);

	/* Need to idle device before changing options */
	ret = mmu->device->ftbl->idle(mmu->device);
	if (ret) {
		kgsl_iommu_disable_clk(mmu);
		return ret;
	}

	for (i = 0; i < KGSL_IOMMU_CONTEXT_MAX; i++) {
		ctx = &iommu->ctx[i];
		if (ctx->dev == NULL)
			continue;

		/*
		 *  1) HLOS cannot program secure context bank.
		 *  2) If context bank is not attached skip.
		 */
		if (!ctx->attached || (KGSL_IOMMU_CONTEXT_SECURE == i))
			continue;

		sctlr_val = KGSL_IOMMU_GET_CTX_REG(iommu, ctx->ctx_id, SCTLR);

		if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE, &pf_policy))
			sctlr_val &= ~(0x1 << KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
		else
			sctlr_val |= (0x1 << KGSL_IOMMU_SCTLR_HUPCF_SHIFT);

		KGSL_IOMMU_SET_CTX_REG(iommu, ctx->ctx_id, SCTLR, sctlr_val);
	}

	kgsl_iommu_disable_clk(mmu);
	return ret;
}

/**
 * kgsl_iommu_set_pagefault() - Checks if a IOMMU device has faulted
 * @mmu: MMU pointer of the device
 *
 * This function is called to set the pagefault bits for the device so
 * that recovery can run with pagefault in consideration
 */
static void kgsl_iommu_set_pagefault(struct kgsl_mmu *mmu)
{
	int i;
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_context *ctx;
	unsigned int fsr;

	/* fault already detected then return early */
	if (atomic_read(&mmu->fault))
		return;

	kgsl_iommu_enable_clk(mmu);

	/* Loop through all IOMMU devices to check for fault */
	for (i = 0; i < KGSL_IOMMU_CONTEXT_MAX; i++) {
		ctx = &iommu->ctx[i];
		if (ctx->dev == NULL)
			continue;

		/*
		 *  1) HLOS cannot program secure context bank.
		 *  2) If context bank is not attached skip.
		 */
		if (!ctx->attached || (KGSL_IOMMU_CONTEXT_SECURE == i))
			continue;

		fsr = KGSL_IOMMU_GET_CTX_REG(iommu, ctx->ctx_id, FSR);
		if (fsr) {
			uint64_t far = KGSL_IOMMU_GET_CTX_REG_Q(iommu,
					ctx->ctx_id, FAR);
			kgsl_iommu_fault_handler(NULL, ctx->dev,
				far, 0, mmu->defaultpagetable);
>>>>>>> p9x
			break;
		}
	}

<<<<<<< HEAD
	if (ctx == NULL) {
		KGSL_CORE_ERR("dt: Unknown context label %s\n", node->name);
		return -EINVAL;
	}

	if (ctx->id == KGSL_IOMMU_CONTEXT_SECURE)
		device->mmu.secured = true;

	/* this property won't be found for all context banks */
	if (of_property_read_u32(node, "qcom,gpu-offset", &ctx->gpu_offset))
		ctx->gpu_offset = UINT_MAX;

	ctx->kgsldev = device;

	/* arm-smmu driver we'll have the right device pointer here. */
	if (of_find_property(node, "iommus", NULL)) {
		ctx->dev = &pdev->dev;
	} else {
		ctx->dev = kgsl_mmu_get_ctx(ctx->name);

		if (IS_ERR(ctx->dev))
			return PTR_ERR(ctx->dev);
	}

	return 0;
}

static const struct {
	char *feature;
	int bit;
} kgsl_iommu_features[] = {
	{ "qcom,retention", KGSL_MMU_RETENTION },
	{ "qcom,global_pt", KGSL_MMU_GLOBAL_PAGETABLE },
	{ "qcom,hyp_secure_alloc", KGSL_MMU_HYP_SECURE_ALLOC },
	{ "qcom,force-32bit", KGSL_MMU_FORCE_32BIT },
	{ "qcom,coherent-htw", KGSL_MMU_COHERENT_HTW },
};

static int _kgsl_iommu_probe(struct kgsl_device *device,
		struct device_node *node)
{
	const char *cname;
	struct property *prop;
	u32 reg_val[2];
	int i = 0;
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);
	struct device_node *child;
	struct platform_device *pdev = of_find_device_by_node(node);

	memset(iommu, 0, sizeof(*iommu));

	if (of_device_is_compatible(node, "qcom,kgsl-smmu-v1"))
		iommu->version = 1;
	else
		iommu->version = 2;

	if (of_property_read_u32_array(node, "reg", reg_val, 2)) {
		KGSL_CORE_ERR("dt: Unable to read KGSL IOMMU register range\n");
		return -EINVAL;
	}
	iommu->regstart = reg_val[0];
	iommu->regsize = reg_val[1];

	/* Protecting the SMMU registers is mandatory */
	if (of_property_read_u32_array(node, "qcom,protect", reg_val, 2)) {
		KGSL_CORE_ERR("dt: no iommu protection range specified\n");
		return -EINVAL;
	}
	iommu->protect.base = reg_val[0] / sizeof(u32);
	iommu->protect.range = ilog2(reg_val[1] / sizeof(u32));

	of_property_for_each_string(node, "clock-names", prop, cname) {
		struct clk *c = devm_clk_get(&pdev->dev, cname);

		if (IS_ERR(c)) {
			KGSL_CORE_ERR("dt: Couldn't get clock: %s\n", cname);
			return -ENODEV;
		}
		if (i >= KGSL_IOMMU_MAX_CLKS) {
			KGSL_CORE_ERR("dt: too many clocks defined.\n");
			return -EINVAL;
		}

		iommu->clks[i] = c;
		++i;
	}

	for (i = 0; i < ARRAY_SIZE(kgsl_iommu_features); i++) {
		if (of_property_read_bool(node, kgsl_iommu_features[i].feature))
			device->mmu.features |= kgsl_iommu_features[i].bit;
	}

	if (of_property_read_u32(node, "qcom,micro-mmu-control",
		&iommu->micro_mmu_ctrl))
		iommu->micro_mmu_ctrl = UINT_MAX;

	if (of_property_read_u32(node, "qcom,secure_align_mask",
		&device->mmu.secure_align_mask))
		device->mmu.secure_align_mask = 0xfff;

	/* Fill out the rest of the devices in the node */
	of_platform_populate(node, NULL, NULL, &pdev->dev);

	for_each_child_of_node(node, child) {
		int ret;

		if (!of_device_is_compatible(child, "qcom,smmu-kgsl-cb"))
			continue;

		ret = _kgsl_iommu_cb_probe(device, iommu, child);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct {
	char *compat;
	int (*probe)(struct kgsl_device *device, struct device_node *node);
} kgsl_dt_devices[] = {
	{ "qcom,kgsl-smmu-v1", _kgsl_iommu_probe },
	{ "qcom,kgsl-smmu-v2", _kgsl_iommu_probe },
};

static int kgsl_iommu_probe(struct kgsl_device *device)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(kgsl_dt_devices); i++) {
		struct device_node *node;

		node = of_find_compatible_node(device->pdev->dev.of_node,
			NULL, kgsl_dt_devices[i].compat);

		if (node != NULL)
			return kgsl_dt_devices[i].probe(device, node);
	}

	return -ENODEV;
=======
	kgsl_iommu_disable_clk(mmu);
}

struct kgsl_protected_registers *kgsl_iommu_get_prot_regs(struct kgsl_mmu *mmu)
{
	static struct kgsl_protected_registers iommuv1_regs = { 0x4000, 14 };
	static struct kgsl_protected_registers iommuv2_regs;

	if (kgsl_msm_supports_iommu_v2()) {

		struct kgsl_iommu *iommu = mmu->priv;

		iommuv2_regs.base = iommu->ahb_base_offset >> 2;
		iommuv2_regs.range = 10;
		return &iommuv2_regs;
	}
	else
		return &iommuv1_regs;
>>>>>>> p9x
}

struct kgsl_mmu_ops kgsl_iommu_ops = {
	.mmu_init = kgsl_iommu_init,
	.mmu_close = kgsl_iommu_close,
	.mmu_start = kgsl_iommu_start,
	.mmu_stop = kgsl_iommu_stop,
	.mmu_set_pt = kgsl_iommu_set_pt,
<<<<<<< HEAD
	.mmu_clear_fsr = kgsl_iommu_clear_fsr,
	.mmu_get_current_ttbr0 = kgsl_iommu_get_current_ttbr0,
	.mmu_enable_clk = kgsl_iommu_enable_clk,
	.mmu_disable_clk = kgsl_iommu_disable_clk,
	.mmu_get_reg_ahbaddr = kgsl_iommu_get_reg_ahbaddr,
	.mmu_pt_equal = kgsl_iommu_pt_equal,
	.mmu_set_pf_policy = kgsl_iommu_set_pf_policy,
	.mmu_pagefault_resume = kgsl_iommu_pagefault_resume,
	.mmu_get_prot_regs = kgsl_iommu_get_prot_regs,
	.mmu_init_pt = kgsl_iommu_init_pt,
	.mmu_add_global = kgsl_iommu_add_global,
	.mmu_remove_global = kgsl_iommu_remove_global,
	.mmu_getpagetable = kgsl_iommu_getpagetable,
	.mmu_get_qdss_global_entry = kgsl_iommu_get_qdss_global_entry,
	.probe = kgsl_iommu_probe,
=======
	.mmu_pagefault_resume = kgsl_iommu_pagefault_resume,
	.mmu_get_current_ptbase = kgsl_iommu_get_current_ptbase,
	.mmu_enable_clk = kgsl_iommu_enable_clk,
	.mmu_disable_clk = kgsl_iommu_disable_clk,
	.mmu_get_default_ttbr0 = kgsl_iommu_get_default_ttbr0,
	.mmu_get_reg_ahbaddr = kgsl_iommu_get_reg_ahbaddr,
	.mmu_pt_equal = kgsl_iommu_pt_equal,
	.mmu_get_pt_base_addr = kgsl_iommu_get_pt_base_addr,
	/* These callbacks will be set on some chipsets */
	.mmu_set_pf_policy = kgsl_iommu_set_pf_policy,
	.mmu_set_pagefault = kgsl_iommu_set_pagefault,
	.mmu_get_prot_regs = kgsl_iommu_get_prot_regs,
	.mmu_init_pt = kgsl_iommu_init_pt,
>>>>>>> p9x
};

static struct kgsl_mmu_pt_ops iommu_pt_ops = {
	.mmu_map = kgsl_iommu_map,
	.mmu_unmap = kgsl_iommu_unmap,
	.mmu_destroy_pagetable = kgsl_iommu_destroy_pagetable,
<<<<<<< HEAD
	.get_ttbr0 = kgsl_iommu_get_ttbr0,
	.get_contextidr = kgsl_iommu_get_contextidr,
	.get_gpuaddr = kgsl_iommu_get_gpuaddr,
	.put_gpuaddr = kgsl_iommu_put_gpuaddr,
	.set_svm_region = kgsl_iommu_set_svm_region,
	.find_svm_region = kgsl_iommu_find_svm_region,
	.svm_range = kgsl_iommu_svm_range,
	.addr_in_range = kgsl_iommu_addr_in_range,
	.mmu_map_offset = kgsl_iommu_map_offset,
	.mmu_unmap_offset = kgsl_iommu_unmap_offset,
=======
	.get_ptbase = kgsl_iommu_get_ptbase,
>>>>>>> p9x
};
