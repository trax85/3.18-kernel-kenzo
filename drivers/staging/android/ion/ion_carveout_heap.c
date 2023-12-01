/*
<<<<<<< HEAD
 * drivers/staging/android/ion/ion_carveout_heap.c
 *
 * Copyright (C) 2011 Google, Inc.
=======
 * drivers/gpu/ion/ion_carveout_heap.c
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#include <linux/spinlock.h>
<<<<<<< HEAD
#include <linux/dma-mapping.h>
=======

>>>>>>> p9x
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
<<<<<<< HEAD
#include "ion.h"
#include "ion_priv.h"

=======
#include <linux/seq_file.h>
#include "ion.h"
#include "ion_priv.h"

#include <asm/cacheflush.h>
#include <linux/io.h>
#include <linux/msm_ion.h>

>>>>>>> p9x
struct ion_carveout_heap {
	struct ion_heap heap;
	struct gen_pool *pool;
	ion_phys_addr_t base;
<<<<<<< HEAD
=======
	unsigned long allocated_bytes;
	unsigned long total_size;
>>>>>>> p9x
};

ion_phys_addr_t ion_carveout_allocate(struct ion_heap *heap,
				      unsigned long size,
				      unsigned long align)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);
<<<<<<< HEAD
	unsigned long offset = gen_pool_alloc(carveout_heap->pool, size);

	if (!offset)
		return ION_CARVEOUT_ALLOCATE_FAIL;

=======
	unsigned long offset = gen_pool_alloc_aligned(carveout_heap->pool,
							size, ilog2(align));

	if (!offset) {
		if ((carveout_heap->total_size -
		      carveout_heap->allocated_bytes) >= size)
			pr_debug("%s: heap %s has enough memory (%lx) but"
				" the allocation of size %lx still failed."
				" Memory is probably fragmented.",
				__func__, heap->name,
				carveout_heap->total_size -
				carveout_heap->allocated_bytes, size);
		return ION_CARVEOUT_ALLOCATE_FAIL;
	}

	carveout_heap->allocated_bytes += size;
>>>>>>> p9x
	return offset;
}

void ion_carveout_free(struct ion_heap *heap, ion_phys_addr_t addr,
		       unsigned long size)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);

	if (addr == ION_CARVEOUT_ALLOCATE_FAIL)
		return;
	gen_pool_free(carveout_heap->pool, addr, size);
<<<<<<< HEAD
=======
	carveout_heap->allocated_bytes -= size;
>>>>>>> p9x
}

static int ion_carveout_heap_phys(struct ion_heap *heap,
				  struct ion_buffer *buffer,
				  ion_phys_addr_t *addr, size_t *len)
{
<<<<<<< HEAD
	struct sg_table *table = buffer->priv_virt;
	struct page *page = sg_page(table->sgl);
	ion_phys_addr_t paddr = PFN_PHYS(page_to_pfn(page));

	*addr = paddr;
=======
	*addr = buffer->priv_phys;
>>>>>>> p9x
	*len = buffer->size;
	return 0;
}

static int ion_carveout_heap_allocate(struct ion_heap *heap,
				      struct ion_buffer *buffer,
				      unsigned long size, unsigned long align,
				      unsigned long flags)
{
<<<<<<< HEAD
	struct sg_table *table;
	ion_phys_addr_t paddr;
	int ret;

	if (align > PAGE_SIZE)
		return -EINVAL;

	table = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;
	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret)
		goto err_free;

	paddr = ion_carveout_allocate(heap, size, align);
	if (paddr == ION_CARVEOUT_ALLOCATE_FAIL) {
		ret = -ENOMEM;
		goto err_free_table;
	}

	sg_set_page(table->sgl, pfn_to_page(PFN_DOWN(paddr)), size, 0);
	buffer->priv_virt = table;

	return 0;

err_free_table:
	sg_free_table(table);
err_free:
	kfree(table);
	return ret;
=======
	buffer->priv_phys = ion_carveout_allocate(heap, size, align);
	return buffer->priv_phys == ION_CARVEOUT_ALLOCATE_FAIL ? -ENOMEM : 0;
>>>>>>> p9x
}

static void ion_carveout_heap_free(struct ion_buffer *buffer)
{
	struct ion_heap *heap = buffer->heap;
<<<<<<< HEAD
	struct sg_table *table = buffer->priv_virt;
	struct page *page = sg_page(table->sgl);
	ion_phys_addr_t paddr = PFN_PHYS(page_to_pfn(page));

	ion_heap_buffer_zero(buffer);

	if (ion_buffer_cached(buffer))
		dma_sync_sg_for_device(NULL, table->sgl, table->nents,
							DMA_BIDIRECTIONAL);

	ion_carveout_free(heap, paddr, buffer->size);
	sg_free_table(table);
	kfree(table);
=======

	ion_carveout_free(heap, buffer->priv_phys, buffer->size);
	buffer->priv_phys = ION_CARVEOUT_ALLOCATE_FAIL;
>>>>>>> p9x
}

static struct sg_table *ion_carveout_heap_map_dma(struct ion_heap *heap,
						  struct ion_buffer *buffer)
{
<<<<<<< HEAD
	return buffer->priv_virt;
=======
	size_t chunk_size = buffer->size;

	if (ION_IS_CACHED(buffer->flags))
		chunk_size = PAGE_SIZE;

	return ion_create_chunked_sg_table(buffer->priv_phys, chunk_size,
					buffer->size);
>>>>>>> p9x
}

static void ion_carveout_heap_unmap_dma(struct ion_heap *heap,
					struct ion_buffer *buffer)
{
<<<<<<< HEAD
=======
	if (buffer->sg_table)
		sg_free_table(buffer->sg_table);
	kfree(buffer->sg_table);
	buffer->sg_table = 0;
}

void *ion_carveout_heap_map_kernel(struct ion_heap *heap,
				   struct ion_buffer *buffer)
{
	void *ret_value;

	if (ION_IS_CACHED(buffer->flags))
		ret_value = ioremap_cache(buffer->priv_phys, buffer->size);
	else
		ret_value = ioremap(buffer->priv_phys, buffer->size);

	if (ret_value == NULL)
		return ERR_PTR(-ENOMEM);

	return ret_value;
}

void ion_carveout_heap_unmap_kernel(struct ion_heap *heap,
				    struct ion_buffer *buffer)
{
	iounmap(buffer->vaddr);
	buffer->vaddr = NULL;

	return;
}

int ion_carveout_heap_map_user(struct ion_heap *heap, struct ion_buffer *buffer,
			       struct vm_area_struct *vma)
{
	int ret_value = 0;

	if (!ION_IS_CACHED(buffer->flags))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	ret_value =  remap_pfn_range(vma, vma->vm_start,
			PFN_DOWN(buffer->priv_phys) + vma->vm_pgoff,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);

	return ret_value;
}

static int ion_carveout_print_debug(struct ion_heap *heap, struct seq_file *s,
				    const struct list_head *mem_map)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);

	seq_printf(s, "total bytes currently allocated: 0x%lx\n",
		carveout_heap->allocated_bytes);
	seq_printf(s, "total heap size: 0x%lx\n", carveout_heap->total_size);

	if (mem_map) {
		unsigned long base = carveout_heap->base;
		unsigned long size = carveout_heap->total_size;
		unsigned long end = base+size;
		unsigned long last_end = base;
		struct mem_map_data *data;

		seq_printf(s, "\nMemory Map\n");
		seq_printf(s, "%16.s %14.s %14.s %14.s\n",
			   "client", "start address", "end address",
			   "size");

		list_for_each_entry(data, mem_map, node) {
			const char *client_name = "(null)";

			if (last_end < data->addr) {
				phys_addr_t da;

				da = data->addr-1;
				seq_printf(s,
					"%16.s 0x%14pa 0x%14pa %14lu (0x%lx)\n",
					   "FREE", &last_end, &da,
					   (unsigned long)data->addr-last_end,
					   (unsigned long)data->addr-last_end);
			}

			if (data->client_name)
				client_name = data->client_name;

			seq_printf(s,
				"%16.s 0x%14pa 0x%14pa %14lu (0x%lx)\n",
				   client_name, &data->addr,
				   &data->addr_end,
				   data->size, data->size);
			last_end = data->addr_end+1;
		}
		if (last_end < end) {
			seq_printf(s, "%16.s 0x%14lx 0x%14lx %14lu (0x%lx)\n",
				"FREE",
				last_end, end-1, end-last_end, end-last_end);
		}
	}
	return 0;
>>>>>>> p9x
}

static struct ion_heap_ops carveout_heap_ops = {
	.allocate = ion_carveout_heap_allocate,
	.free = ion_carveout_heap_free,
	.phys = ion_carveout_heap_phys,
<<<<<<< HEAD
	.map_dma = ion_carveout_heap_map_dma,
	.unmap_dma = ion_carveout_heap_unmap_dma,
	.map_user = ion_heap_map_user,
	.map_kernel = ion_heap_map_kernel,
	.unmap_kernel = ion_heap_unmap_kernel,
=======
	.map_user = ion_carveout_heap_map_user,
	.map_kernel = ion_carveout_heap_map_kernel,
	.unmap_kernel = ion_carveout_heap_unmap_kernel,
	.map_dma = ion_carveout_heap_map_dma,
	.unmap_dma = ion_carveout_heap_unmap_dma,
	.print_debug = ion_carveout_print_debug,
>>>>>>> p9x
};

struct ion_heap *ion_carveout_heap_create(struct ion_platform_heap *heap_data)
{
	struct ion_carveout_heap *carveout_heap;
	int ret;

<<<<<<< HEAD
	struct page *page;
	size_t size;

	page = pfn_to_page(PFN_DOWN(heap_data->base));
	size = heap_data->size;

	ion_pages_sync_for_device(NULL, page, size, DMA_BIDIRECTIONAL);

	ret = ion_heap_pages_zero(page, size, pgprot_writecombine(PAGE_KERNEL));
	if (ret)
		return ERR_PTR(ret);

=======
>>>>>>> p9x
	carveout_heap = kzalloc(sizeof(struct ion_carveout_heap), GFP_KERNEL);
	if (!carveout_heap)
		return ERR_PTR(-ENOMEM);

<<<<<<< HEAD
	carveout_heap->pool = gen_pool_create(PAGE_SHIFT, -1);
=======
	carveout_heap->pool = gen_pool_create(12, -1);
>>>>>>> p9x
	if (!carveout_heap->pool) {
		kfree(carveout_heap);
		return ERR_PTR(-ENOMEM);
	}
	carveout_heap->base = heap_data->base;
<<<<<<< HEAD
	gen_pool_add(carveout_heap->pool, carveout_heap->base, heap_data->size,
		     -1);
	carveout_heap->heap.ops = &carveout_heap_ops;
	carveout_heap->heap.type = ION_HEAP_TYPE_CARVEOUT;
	carveout_heap->heap.flags = ION_HEAP_FLAG_DEFER_FREE;
=======
	ret = gen_pool_add(carveout_heap->pool, carveout_heap->base,
			heap_data->size, -1);
	if (ret < 0) {
		gen_pool_destroy(carveout_heap->pool);
		kfree(carveout_heap);
		return ERR_PTR(-EINVAL);
	}
	carveout_heap->heap.ops = &carveout_heap_ops;
	carveout_heap->heap.type = ION_HEAP_TYPE_CARVEOUT;
	carveout_heap->allocated_bytes = 0;
	carveout_heap->total_size = heap_data->size;
>>>>>>> p9x

	return &carveout_heap->heap;
}

void ion_carveout_heap_destroy(struct ion_heap *heap)
{
	struct ion_carveout_heap *carveout_heap =
	     container_of(heap, struct  ion_carveout_heap, heap);

	gen_pool_destroy(carveout_heap->pool);
	kfree(carveout_heap);
	carveout_heap = NULL;
}
