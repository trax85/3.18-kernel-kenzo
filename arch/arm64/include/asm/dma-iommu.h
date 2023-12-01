#ifndef ASMARM_DMA_IOMMU_H
#define ASMARM_DMA_IOMMU_H

#ifdef __KERNEL__

<<<<<<< HEAD
#include <linux/err.h>
=======
>>>>>>> p9x
#include <linux/mm_types.h>
#include <linux/scatterlist.h>
#include <linux/dma-debug.h>
#include <linux/kmemcheck.h>
#include <linux/kref.h>

struct dma_iommu_mapping {
	/* iommu specific data */
	struct iommu_domain	*domain;

	void			*bitmap;
	size_t			bits;
<<<<<<< HEAD
=======
	unsigned int		order;
>>>>>>> p9x
	dma_addr_t		base;

	spinlock_t		lock;
	struct kref		kref;
<<<<<<< HEAD
#ifdef CONFIG_IOMMU_IO_PGTABLE_FAST
	struct dma_fast_smmu_mapping *fast;
#endif
};

#ifdef CONFIG_ARM64_DMA_USE_IOMMU

struct dma_iommu_mapping *
arm_iommu_create_mapping(struct bus_type *bus, dma_addr_t base, size_t size);
=======
};

struct dma_iommu_mapping *
arm_iommu_create_mapping(struct bus_type *bus, dma_addr_t base, size_t size,
			 int order);
>>>>>>> p9x

void arm_iommu_release_mapping(struct dma_iommu_mapping *mapping);

int arm_iommu_attach_device(struct device *dev,
					struct dma_iommu_mapping *mapping);
void arm_iommu_detach_device(struct device *dev);

<<<<<<< HEAD
#else  /* !CONFIG_ARM64_DMA_USE_IOMMU */

static inline struct dma_iommu_mapping *
arm_iommu_create_mapping(struct bus_type *bus, dma_addr_t base, size_t size)
{
	return ERR_PTR(-ENOMEM);
}

static inline void arm_iommu_release_mapping(struct dma_iommu_mapping *mapping)
{
}

static inline int arm_iommu_attach_device(struct device *dev,
			struct dma_iommu_mapping *mapping)
{
	return -ENODEV;
}

static inline void arm_iommu_detach_device(struct device *dev)
{
}

#endif	/* CONFIG_ARM64_DMA_USE_IOMMU */

=======
>>>>>>> p9x
#endif /* __KERNEL__ */
#endif
