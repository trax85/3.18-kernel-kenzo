#ifndef _ASM_GENERIC_DMA_CONTIGUOUS_H
#define _ASM_GENERIC_DMA_CONTIGUOUS_H

#include <linux/types.h>
#include <linux/device.h>

static inline void
dma_contiguous_early_fixup(phys_addr_t base, unsigned long size) { }

#endif
