#ifndef _ASM_GENERIC_DMA_CONTIGUOUS_H
#define _ASM_GENERIC_DMA_CONTIGUOUS_H

#include <linux/types.h>

<<<<<<< HEAD
static inline void
dma_contiguous_early_fixup(phys_addr_t base, unsigned long size) { }
=======
#include <linux/device.h>
#include <linux/dma-contiguous.h>

static inline struct cma *dev_get_cma_area(struct device *dev)
{
	if (dev && dev->cma_area)
		return dev->cma_area;
	return dma_contiguous_def_area;
}

static inline void dev_set_cma_area(struct device *dev, struct cma *cma)
{
	if (dev)
		dev->cma_area = cma;
}

#endif
#endif
>>>>>>> p9x

#endif
