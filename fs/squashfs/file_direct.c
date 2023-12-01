/*
 * Copyright (c) 2013
 * Phillip Lougher <phillip@squashfs.org.uk>
 *
 * This work is licensed under the terms of the GNU GPL, version 2. See
 * the COPYING file in the top-level directory.
 */

#include <linux/fs.h>
#include <linux/vfs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/mutex.h>
<<<<<<< HEAD
#include <linux/mm_inline.h>
=======
>>>>>>> p9x

#include "squashfs_fs.h"
#include "squashfs_fs_sb.h"
#include "squashfs_fs_i.h"
#include "squashfs.h"
#include "page_actor.h"

<<<<<<< HEAD
// Backported from 4.5
#define lru_to_page(head) (list_entry((head)->prev, struct page, lru))

static void release_actor_pages(struct page **page, int pages, int error)
{
	int i;

	for (i = 0; i < pages; i++) {
		if (!page[i])
			continue;
		flush_dcache_page(page[i]);
		if (!error)
			SetPageUptodate(page[i]);
		else {
			SetPageError(page[i]);
			zero_user_segment(page[i], 0, PAGE_CACHE_SIZE);
		}
		unlock_page(page[i]);
		put_page(page[i]);
	}
	kfree(page);
}

/*
 * Create a "page actor" which will kmap and kunmap the
 * page cache pages appropriately within the decompressor
 */
static struct squashfs_page_actor *actor_from_page_cache(
	unsigned int actor_pages, struct page *target_page,
	struct list_head *rpages, unsigned int *nr_pages, int start_index,
	struct address_space *mapping)
{
	struct page **page;
	struct squashfs_page_actor *actor;
	int i, n;
	gfp_t gfp = GFP_KERNEL;

	page = kmalloc_array(actor_pages, sizeof(void *), GFP_KERNEL);
	if (!page)
		return NULL;

	for (i = 0, n = start_index; i < actor_pages; i++, n++) {
		if (target_page == NULL && rpages && !list_empty(rpages)) {
			struct page *cur_page = lru_to_page(rpages);

			if (cur_page->index < start_index + actor_pages) {
				list_del(&cur_page->lru);
				--(*nr_pages);
				if (add_to_page_cache_lru(cur_page, mapping,
							  cur_page->index, gfp))
					put_page(cur_page);
				else
					target_page = cur_page;
			} else
				rpages = NULL;
		}

		if (target_page && target_page->index == n) {
			page[i] = target_page;
			target_page = NULL;
		} else {
			page[i] = grab_cache_page_nowait(mapping, n);
			if (page[i] == NULL)
				continue;
=======
static int squashfs_read_cache(struct page *target_page, u64 block, int bsize,
	int pages, struct page **page);

/* Read separately compressed datablock directly into page cache */
int squashfs_readpage_block(struct page *target_page, u64 block, int bsize)

{
	struct inode *inode = target_page->mapping->host;
	struct squashfs_sb_info *msblk = inode->i_sb->s_fs_info;

	int file_end = (i_size_read(inode) - 1) >> PAGE_CACHE_SHIFT;
	int mask = (1 << (msblk->block_log - PAGE_CACHE_SHIFT)) - 1;
	int start_index = target_page->index & ~mask;
	int end_index = start_index | mask;
	int i, n, pages, missing_pages, bytes, res = -ENOMEM;
	struct page **page;
	struct squashfs_page_actor *actor;
	void *pageaddr;

	if (end_index > file_end)
		end_index = file_end;

	pages = end_index - start_index + 1;

	page = kmalloc_array(pages, sizeof(void *), GFP_KERNEL);
	if (page == NULL)
		return res;

	/*
	 * Create a "page actor" which will kmap and kunmap the
	 * page cache pages appropriately within the decompressor
	 */
	actor = squashfs_page_actor_init_special(page, pages, 0);
	if (actor == NULL)
		goto out;

	/* Try to grab all the pages covered by the Squashfs block */
	for (missing_pages = 0, i = 0, n = start_index; i < pages; i++, n++) {
		page[i] = (n == target_page->index) ? target_page :
			grab_cache_page_nowait(target_page->mapping, n);

		if (page[i] == NULL) {
			missing_pages++;
			continue;
>>>>>>> p9x
		}

		if (PageUptodate(page[i])) {
			unlock_page(page[i]);
			page_cache_release(page[i]);
			page[i] = NULL;
<<<<<<< HEAD
		}
	}

	actor = squashfs_page_actor_init(page, actor_pages, 0,
			release_actor_pages);
	if (!actor) {
		release_actor_pages(page, actor_pages, -ENOMEM);
		kfree(page);
		return NULL;
	}
	return actor;
}

int squashfs_readpages_block(struct page *target_page,
			     struct list_head *readahead_pages,
			     unsigned int *nr_pages,
			     struct address_space *mapping,
			     int page_index, u64 block, int bsize)

{
	struct squashfs_page_actor *actor;
	struct inode *inode = mapping->host;
	struct squashfs_sb_info *msblk = inode->i_sb->s_fs_info;
	int start_index, end_index, file_end, actor_pages, res;
	int mask = (1 << (msblk->block_log - PAGE_CACHE_SHIFT)) - 1;

	/*
	 * If readpage() is called on an uncompressed datablock, we can just
	 * read the pages instead of fetching the whole block.
	 * This greatly improves the performance when a process keep doing
	 * random reads because we only fetch the necessary data.
	 * The readahead algorithm will take care of doing speculative reads
	 * if necessary.
	 * We can't read more than 1 block even if readahead provides use more
	 * pages because we don't know yet if the next block is compressed or
	 * not.
	 */
	if (bsize && !SQUASHFS_COMPRESSED_BLOCK(bsize)) {
		u64 block_end = block + msblk->block_size;

		block += (page_index & mask) * PAGE_SIZE;
		actor_pages = (block_end - block) / PAGE_SIZE;
		if (*nr_pages < actor_pages)
			actor_pages = *nr_pages;
		start_index = page_index;
		bsize = min_t(int, bsize, (PAGE_SIZE * actor_pages)
					  | SQUASHFS_COMPRESSED_BIT_BLOCK);
	} else {
		file_end = (i_size_read(inode) - 1) >> PAGE_SHIFT;
		start_index = page_index & ~mask;
		end_index = start_index | mask;
		if (end_index > file_end)
			end_index = file_end;
		actor_pages = end_index - start_index + 1;
	}

	actor = actor_from_page_cache(actor_pages, target_page,
				      readahead_pages, nr_pages, start_index,
				      mapping);
	if (!actor)
		return -ENOMEM;

	res = squashfs_read_data_async(inode->i_sb, block, bsize, NULL,
				       actor);
	return res < 0 ? res : 0;
=======
			missing_pages++;
		}
	}

	if (missing_pages) {
		/*
		 * Couldn't get one or more pages, this page has either
		 * been VM reclaimed, but others are still in the page cache
		 * and uptodate, or we're racing with another thread in
		 * squashfs_readpage also trying to grab them.  Fall back to
		 * using an intermediate buffer.
		 */
		res = squashfs_read_cache(target_page, block, bsize, pages,
								page);
		if (res < 0)
			goto mark_errored;

		goto out;
	}

	/* Decompress directly into the page cache buffers */
	res = squashfs_read_data(inode->i_sb, block, bsize, NULL, actor);
	if (res < 0)
		goto mark_errored;

	/* Last page may have trailing bytes not filled */
	bytes = res % PAGE_CACHE_SIZE;
	if (bytes) {
		pageaddr = kmap_atomic(page[pages - 1]);
		memset(pageaddr + bytes, 0, PAGE_CACHE_SIZE - bytes);
		kunmap_atomic(pageaddr);
	}

	/* Mark pages as uptodate, unlock and release */
	for (i = 0; i < pages; i++) {
		flush_dcache_page(page[i]);
		SetPageUptodate(page[i]);
		unlock_page(page[i]);
		if (page[i] != target_page)
			page_cache_release(page[i]);
	}

	kfree(actor);
	kfree(page);

	return 0;

mark_errored:
	/* Decompression failed, mark pages as errored.  Target_page is
	 * dealt with by the caller
	 */
	for (i = 0; i < pages; i++) {
		if (page[i] == NULL || page[i] == target_page)
			continue;
		flush_dcache_page(page[i]);
		SetPageError(page[i]);
		unlock_page(page[i]);
		page_cache_release(page[i]);
	}

out:
	kfree(actor);
	kfree(page);
	return res;
}


static int squashfs_read_cache(struct page *target_page, u64 block, int bsize,
	int pages, struct page **page)
{
	struct inode *i = target_page->mapping->host;
	struct squashfs_cache_entry *buffer = squashfs_get_datablock(i->i_sb,
						 block, bsize);
	int bytes = buffer->length, res = buffer->error, n, offset = 0;
	void *pageaddr;

	if (res) {
		ERROR("Unable to read page, block %llx, size %x\n", block,
			bsize);
		goto out;
	}

	for (n = 0; n < pages && bytes > 0; n++,
			bytes -= PAGE_CACHE_SIZE, offset += PAGE_CACHE_SIZE) {
		int avail = min_t(int, bytes, PAGE_CACHE_SIZE);

		if (page[n] == NULL)
			continue;

		pageaddr = kmap_atomic(page[n]);
		squashfs_copy_data(pageaddr, buffer, offset, avail);
		memset(pageaddr + avail, 0, PAGE_CACHE_SIZE - avail);
		kunmap_atomic(pageaddr);
		flush_dcache_page(page[n]);
		SetPageUptodate(page[n]);
		unlock_page(page[n]);
		if (page[n] != target_page)
			page_cache_release(page[n]);
	}

out:
	squashfs_cache_put(buffer);
	return res;
>>>>>>> p9x
}
