/*
 * Copyright (C) 2014 Sergey Senozhatsky.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
<<<<<<< HEAD

#include "zcomp.h"
#include "zcomp_lzo.h"
#ifdef CONFIG_ZRAM_LZ4_COMPRESS
#include "zcomp_lz4.h"
#endif

/*
 * single zcomp_strm backend
 */
struct zcomp_strm_single {
	struct mutex strm_lock;
	struct zcomp_strm *zstrm;
};

/*
 * multi zcomp_strm backend
 */
struct zcomp_strm_multi {
	/* protect strm list */
	spinlock_t strm_lock;
	/* max possible number of zstrm streams */
	int max_strm;
	/* number of available zstrm streams */
	int avail_strm;
	/* list of available strms */
	struct list_head idle_strm;
	wait_queue_head_t strm_wait;
};

static struct zcomp_backend *backends[] = {
	&zcomp_lzo,
#ifdef CONFIG_ZRAM_LZ4_COMPRESS
	&zcomp_lz4,
=======
#include <linux/cpu.h>
#include <linux/crypto.h>

#include "zcomp.h"

static const char * const backends[] = {
	"lzo",
#if IS_ENABLED(CONFIG_CRYPTO_LZ4)
	"lz4",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_DEFLATE)
	"deflate",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_LZ4HC)
	"lz4hc",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_842)
	"842",
>>>>>>> p9x
#endif
	NULL
};

<<<<<<< HEAD
static struct zcomp_backend *find_backend(const char *compress)
{
	int i = 0;
	while (backends[i]) {
		if (sysfs_streq(compress, backends[i]->name))
			break;
		i++;
	}
	return backends[i];
}

static void zcomp_strm_free(struct zcomp *comp, struct zcomp_strm *zstrm)
{
	if (zstrm->private)
		comp->backend->destroy(zstrm->private);
=======
static void zcomp_strm_free(struct zcomp_strm *zstrm)
{
	if (!IS_ERR_OR_NULL(zstrm->tfm))
		crypto_free_comp(zstrm->tfm);
>>>>>>> p9x
	free_pages((unsigned long)zstrm->buffer, 1);
	kfree(zstrm);
}

/*
<<<<<<< HEAD
 * allocate new zcomp_strm structure with ->private initialized by
=======
 * allocate new zcomp_strm structure with ->tfm initialized by
>>>>>>> p9x
 * backend, return NULL on error
 */
static struct zcomp_strm *zcomp_strm_alloc(struct zcomp *comp)
{
<<<<<<< HEAD
	struct zcomp_strm *zstrm = kmalloc(sizeof(*zstrm), GFP_NOIO);
	if (!zstrm)
		return NULL;

	zstrm->private = comp->backend->create();
=======
	struct zcomp_strm *zstrm = kmalloc(sizeof(*zstrm), GFP_KERNEL);
	if (!zstrm)
		return NULL;

	zstrm->tfm = crypto_alloc_comp(comp->name, 0, 0);
>>>>>>> p9x
	/*
	 * allocate 2 pages. 1 for compressed data, plus 1 extra for the
	 * case when compressed size is larger than the original one
	 */
<<<<<<< HEAD
	zstrm->buffer = (void *)__get_free_pages(GFP_NOIO | __GFP_ZERO, 1);
	if (!zstrm->private || !zstrm->buffer) {
		zcomp_strm_free(comp, zstrm);
=======
	zstrm->buffer = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, 1);
	if (IS_ERR_OR_NULL(zstrm->tfm) || !zstrm->buffer) {
		zcomp_strm_free(zstrm);
>>>>>>> p9x
		zstrm = NULL;
	}
	return zstrm;
}

<<<<<<< HEAD
/*
 * get idle zcomp_strm or wait until other process release
 * (zcomp_strm_release()) one for us
 */
static struct zcomp_strm *zcomp_strm_multi_find(struct zcomp *comp)
{
	struct zcomp_strm_multi *zs = comp->stream;
	struct zcomp_strm *zstrm;

	while (1) {
		spin_lock(&zs->strm_lock);
		if (!list_empty(&zs->idle_strm)) {
			zstrm = list_entry(zs->idle_strm.next,
					struct zcomp_strm, list);
			list_del(&zstrm->list);
			spin_unlock(&zs->strm_lock);
			return zstrm;
		}
		/* zstrm streams limit reached, wait for idle stream */
		if (zs->avail_strm >= zs->max_strm) {
			spin_unlock(&zs->strm_lock);
			wait_event(zs->strm_wait, !list_empty(&zs->idle_strm));
			continue;
		}
		/* allocate new zstrm stream */
		zs->avail_strm++;
		spin_unlock(&zs->strm_lock);

		zstrm = zcomp_strm_alloc(comp);
		if (!zstrm) {
			spin_lock(&zs->strm_lock);
			zs->avail_strm--;
			spin_unlock(&zs->strm_lock);
			wait_event(zs->strm_wait, !list_empty(&zs->idle_strm));
			continue;
		}
		break;
	}
	return zstrm;
}

/* add stream back to idle list and wake up waiter or free the stream */
static void zcomp_strm_multi_release(struct zcomp *comp, struct zcomp_strm *zstrm)
{
	struct zcomp_strm_multi *zs = comp->stream;

	spin_lock(&zs->strm_lock);
	if (zs->avail_strm <= zs->max_strm) {
		list_add(&zstrm->list, &zs->idle_strm);
		spin_unlock(&zs->strm_lock);
		wake_up(&zs->strm_wait);
		return;
	}

	zs->avail_strm--;
	spin_unlock(&zs->strm_lock);
	zcomp_strm_free(comp, zstrm);
}

/* change max_strm limit */
static bool zcomp_strm_multi_set_max_streams(struct zcomp *comp, int num_strm)
{
	struct zcomp_strm_multi *zs = comp->stream;
	struct zcomp_strm *zstrm;

	spin_lock(&zs->strm_lock);
	zs->max_strm = num_strm;
	/*
	 * if user has lowered the limit and there are idle streams,
	 * immediately free as much streams (and memory) as we can.
	 */
	while (zs->avail_strm > num_strm && !list_empty(&zs->idle_strm)) {
		zstrm = list_entry(zs->idle_strm.next,
				struct zcomp_strm, list);
		list_del(&zstrm->list);
		zcomp_strm_free(comp, zstrm);
		zs->avail_strm--;
	}
	spin_unlock(&zs->strm_lock);
	return true;
}

static void zcomp_strm_multi_destroy(struct zcomp *comp)
{
	struct zcomp_strm_multi *zs = comp->stream;
	struct zcomp_strm *zstrm;

	while (!list_empty(&zs->idle_strm)) {
		zstrm = list_entry(zs->idle_strm.next,
				struct zcomp_strm, list);
		list_del(&zstrm->list);
		zcomp_strm_free(comp, zstrm);
	}
	kfree(zs);
}

static int zcomp_strm_multi_create(struct zcomp *comp, int max_strm)
{
	struct zcomp_strm *zstrm;
	struct zcomp_strm_multi *zs;

	comp->destroy = zcomp_strm_multi_destroy;
	comp->strm_find = zcomp_strm_multi_find;
	comp->strm_release = zcomp_strm_multi_release;
	comp->set_max_streams = zcomp_strm_multi_set_max_streams;
	zs = kmalloc(sizeof(struct zcomp_strm_multi), GFP_KERNEL);
	if (!zs)
		return -ENOMEM;

	comp->stream = zs;
	spin_lock_init(&zs->strm_lock);
	INIT_LIST_HEAD(&zs->idle_strm);
	init_waitqueue_head(&zs->strm_wait);
	zs->max_strm = max_strm;
	zs->avail_strm = 1;

	zstrm = zcomp_strm_alloc(comp);
	if (!zstrm) {
		kfree(zs);
		return -ENOMEM;
	}
	list_add(&zstrm->list, &zs->idle_strm);
	return 0;
}

static struct zcomp_strm *zcomp_strm_single_find(struct zcomp *comp)
{
	struct zcomp_strm_single *zs = comp->stream;
	mutex_lock(&zs->strm_lock);
	return zs->zstrm;
}

static void zcomp_strm_single_release(struct zcomp *comp,
		struct zcomp_strm *zstrm)
{
	struct zcomp_strm_single *zs = comp->stream;
	mutex_unlock(&zs->strm_lock);
}

static bool zcomp_strm_single_set_max_streams(struct zcomp *comp, int num_strm)
{
	/* zcomp_strm_single support only max_comp_streams == 1 */
	return false;
}

static void zcomp_strm_single_destroy(struct zcomp *comp)
{
	struct zcomp_strm_single *zs = comp->stream;
	zcomp_strm_free(comp, zs->zstrm);
	kfree(zs);
}

static int zcomp_strm_single_create(struct zcomp *comp)
{
	struct zcomp_strm_single *zs;

	comp->destroy = zcomp_strm_single_destroy;
	comp->strm_find = zcomp_strm_single_find;
	comp->strm_release = zcomp_strm_single_release;
	comp->set_max_streams = zcomp_strm_single_set_max_streams;
	zs = kmalloc(sizeof(struct zcomp_strm_single), GFP_KERNEL);
	if (!zs)
		return -ENOMEM;

	comp->stream = zs;
	mutex_init(&zs->strm_lock);
	zs->zstrm = zcomp_strm_alloc(comp);
	if (!zs->zstrm) {
		kfree(zs);
		return -ENOMEM;
	}
	return 0;
=======
bool zcomp_available_algorithm(const char *comp)
{
	int i = 0;

	while (backends[i]) {
		if (sysfs_streq(comp, backends[i]))
			return true;
		i++;
	}

	/*
	 * Crypto does not ignore a trailing new line symbol,
	 * so make sure you don't supply a string containing
	 * one.
	 * This also means that we permit zcomp initialisation
	 * with any compressing algorithm known to crypto api.
	 */
	return crypto_has_comp(comp, 0, 0) == 1;
>>>>>>> p9x
}

/* show available compressors */
ssize_t zcomp_available_show(const char *comp, char *buf)
{
<<<<<<< HEAD
	ssize_t sz = 0;
	int i = 0;

	while (backends[i]) {
		if (sysfs_streq(comp, backends[i]->name))
			sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
					"[%s] ", backends[i]->name);
		else
			sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
					"%s ", backends[i]->name);
		i++;
	}
=======
	bool known_algorithm = false;
	ssize_t sz = 0;
	int i = 0;

	for (; backends[i]; i++) {
		if (!strcmp(comp, backends[i])) {
			known_algorithm = true;
			sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
					"[%s] ", backends[i]);
		} else {
			sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
					"%s ", backends[i]);
		}
	}

	/*
	 * Out-of-tree module known to crypto api or a missing
	 * entry in `backends'.
	 */
	if (!known_algorithm && crypto_has_comp(comp, 0, 0) == 1)
		sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
				"[%s] ", comp);

>>>>>>> p9x
	sz += scnprintf(buf + sz, PAGE_SIZE - sz, "\n");
	return sz;
}

<<<<<<< HEAD
bool zcomp_set_max_streams(struct zcomp *comp, int num_strm)
{
	return comp->set_max_streams(comp, num_strm);
}

struct zcomp_strm *zcomp_strm_find(struct zcomp *comp)
{
	return comp->strm_find(comp);
=======
struct zcomp_strm *zcomp_strm_find(struct zcomp *comp)
{
	return *get_cpu_ptr(comp->stream);
>>>>>>> p9x
}

void zcomp_strm_release(struct zcomp *comp, struct zcomp_strm *zstrm)
{
<<<<<<< HEAD
	comp->strm_release(comp, zstrm);
}

int zcomp_compress(struct zcomp *comp, struct zcomp_strm *zstrm,
		const unsigned char *src, size_t *dst_len)
{
	return comp->backend->compress(src, zstrm->buffer, dst_len,
			zstrm->private);
}

int zcomp_decompress(struct zcomp *comp, const unsigned char *src,
		size_t src_len, unsigned char *dst)
{
	return comp->backend->decompress(src, src_len, dst);
=======
	put_cpu_ptr(comp->stream);
}

int zcomp_compress(struct zcomp_strm *zstrm,
		const void *src, unsigned int *dst_len)
{
	/*
	 * Our dst memory (zstrm->buffer) is always `2 * PAGE_SIZE' sized
	 * because sometimes we can endup having a bigger compressed data
	 * due to various reasons: for example compression algorithms tend
	 * to add some padding to the compressed buffer. Speaking of padding,
	 * comp algorithm `842' pads the compressed length to multiple of 8
	 * and returns -ENOSP when the dst memory is not big enough, which
	 * is not something that ZRAM wants to see. We can handle the
	 * `compressed_size > PAGE_SIZE' case easily in ZRAM, but when we
	 * receive -ERRNO from the compressing backend we can't help it
	 * anymore. To make `842' happy we need to tell the exact size of
	 * the dst buffer, zram_drv will take care of the fact that
	 * compressed buffer is too big.
	 */
	*dst_len = PAGE_SIZE * 2;

	return crypto_comp_compress(zstrm->tfm,
			src, PAGE_SIZE,
			zstrm->buffer, dst_len);
}

int zcomp_decompress(struct zcomp_strm *zstrm,
		const void *src, unsigned int src_len, void *dst)
{
	unsigned int dst_len = PAGE_SIZE;

	return crypto_comp_decompress(zstrm->tfm,
			src, src_len,
			dst, &dst_len);
}

static int __zcomp_cpu_notifier(struct zcomp *comp,
		unsigned long action, unsigned long cpu)
{
	struct zcomp_strm *zstrm;

	switch (action) {
	case CPU_UP_PREPARE:
		if (WARN_ON(*per_cpu_ptr(comp->stream, cpu)))
			break;
		zstrm = zcomp_strm_alloc(comp);
		if (IS_ERR_OR_NULL(zstrm)) {
			pr_err("Can't allocate a compression stream\n");
			return NOTIFY_BAD;
		}
		*per_cpu_ptr(comp->stream, cpu) = zstrm;
		break;
	case CPU_DEAD:
	case CPU_UP_CANCELED:
		zstrm = *per_cpu_ptr(comp->stream, cpu);
		if (!IS_ERR_OR_NULL(zstrm))
			zcomp_strm_free(zstrm);
		*per_cpu_ptr(comp->stream, cpu) = NULL;
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static int zcomp_cpu_notifier(struct notifier_block *nb,
		unsigned long action, void *pcpu)
{
	unsigned long cpu = (unsigned long)pcpu;
	struct zcomp *comp = container_of(nb, typeof(*comp), notifier);

	return __zcomp_cpu_notifier(comp, action, cpu);
}

static int zcomp_init(struct zcomp *comp)
{
	unsigned long cpu;
	int ret;

	comp->notifier.notifier_call = zcomp_cpu_notifier;

	comp->stream = alloc_percpu(struct zcomp_strm *);
	if (!comp->stream)
		return -ENOMEM;

	cpu_notifier_register_begin();
	for_each_online_cpu(cpu) {
		ret = __zcomp_cpu_notifier(comp, CPU_UP_PREPARE, cpu);
		if (ret == NOTIFY_BAD)
			goto cleanup;
	}
	__register_cpu_notifier(&comp->notifier);
	cpu_notifier_register_done();
	return 0;

cleanup:
	for_each_online_cpu(cpu)
		__zcomp_cpu_notifier(comp, CPU_UP_CANCELED, cpu);
	cpu_notifier_register_done();
	return -ENOMEM;
>>>>>>> p9x
}

void zcomp_destroy(struct zcomp *comp)
{
<<<<<<< HEAD
	comp->destroy(comp);
=======
	unsigned long cpu;

	cpu_notifier_register_begin();
	for_each_online_cpu(cpu)
		__zcomp_cpu_notifier(comp, CPU_UP_CANCELED, cpu);
	__unregister_cpu_notifier(&comp->notifier);
	cpu_notifier_register_done();

	free_percpu(comp->stream);
>>>>>>> p9x
	kfree(comp);
}

/*
 * search available compressors for requested algorithm.
 * allocate new zcomp and initialize it. return compressing
 * backend pointer or ERR_PTR if things went bad. ERR_PTR(-EINVAL)
 * if requested algorithm is not supported, ERR_PTR(-ENOMEM) in
 * case of allocation error, or any other error potentially
<<<<<<< HEAD
 * returned by functions zcomp_strm_{multi,single}_create.
 */
struct zcomp *zcomp_create(const char *compress, int max_strm)
{
	struct zcomp *comp;
	struct zcomp_backend *backend;
	int error;

	backend = find_backend(compress);
	if (!backend)
=======
 * returned by zcomp_init().
 */
struct zcomp *zcomp_create(const char *compress)
{
	struct zcomp *comp;
	int error;

	if (!zcomp_available_algorithm(compress))
>>>>>>> p9x
		return ERR_PTR(-EINVAL);

	comp = kzalloc(sizeof(struct zcomp), GFP_KERNEL);
	if (!comp)
		return ERR_PTR(-ENOMEM);

<<<<<<< HEAD
	comp->backend = backend;
	if (max_strm > 1)
		error = zcomp_strm_multi_create(comp, max_strm);
	else
		error = zcomp_strm_single_create(comp);
=======
	comp->name = compress;
	error = zcomp_init(comp);
>>>>>>> p9x
	if (error) {
		kfree(comp);
		return ERR_PTR(error);
	}
	return comp;
}
