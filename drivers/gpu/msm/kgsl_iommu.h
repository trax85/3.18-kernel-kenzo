<<<<<<< HEAD
/* Copyright (c) 2012-2016,2018 The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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
#ifndef __KGSL_IOMMU_H
#define __KGSL_IOMMU_H

<<<<<<< HEAD
#ifdef CONFIG_MSM_IOMMU
#include <linux/qcom_iommu.h>
#endif
#include <linux/of.h>
#include "kgsl.h"

/*
 * These defines control the address range for allocations that
 * are mapped into all pagetables.
 */
#define KGSL_IOMMU_GLOBAL_MEM_SIZE	SZ_8M
#define KGSL_IOMMU_GLOBAL_MEM_BASE32	0xf8000000
#define KGSL_IOMMU_GLOBAL_MEM_BASE64	0xfc000000

#define KGSL_IOMMU_GLOBAL_MEM_BASE(__mmu)	\
	(MMU_FEATURE(__mmu, KGSL_MMU_64BIT) ? \
		KGSL_IOMMU_GLOBAL_MEM_BASE64 : KGSL_IOMMU_GLOBAL_MEM_BASE32)

#define KGSL_IOMMU_SECURE_SIZE SZ_256M
#define KGSL_IOMMU_SECURE_END(_mmu) KGSL_IOMMU_GLOBAL_MEM_BASE(_mmu)
#define KGSL_IOMMU_SECURE_BASE(_mmu)	\
	(KGSL_IOMMU_GLOBAL_MEM_BASE(_mmu) - KGSL_IOMMU_SECURE_SIZE)

#define KGSL_IOMMU_SVM_BASE32		0x300000
#define KGSL_IOMMU_SVM_END32		(0xC0000000 - SZ_16M)

#define KGSL_IOMMU_VA_BASE64		0x500000000ULL
#define KGSL_IOMMU_VA_END64		0x600000000ULL
/*
 * Note: currently we only support 36 bit addresses,
 * but the CPU supports 39. Eventually this range
 * should change to high part of the 39 bit address
 * space just like the CPU.
 */
#define KGSL_IOMMU_SVM_BASE64		0x700000000ULL
#define KGSL_IOMMU_SVM_END64		0x800000000ULL
=======
#include <linux/qcom_iommu.h>
#include "kgsl.h"

/* Pagetable virtual base */
#define KGSL_IOMMU_CTX_OFFSET_V1	0x8000
#define KGSL_IOMMU_CTX_OFFSET_V2	0x9000
#define KGSL_IOMMU_CTX_OFFSET_V2_A5XX	0x8000
#define KGSL_IOMMU_CTX_OFFSET_A405V2	0x8000
#define KGSL_IOMMU_CTX_SHIFT		12

/* IOMMU V2 AHB base is fixed */
#define KGSL_IOMMU_V2_AHB_BASE_OFFSET		0xA000
#define KGSL_IOMMU_V2_AHB_BASE_OFFSET_A405  0x48000
#define KGSL_IOMMU_V2_AHB_BASE_OFFSET_A5XX  0x40000
/* IOMMU_V2 AHB base points to ContextBank1 */
#define KGSL_IOMMU_CTX_AHB_OFFSET_V2   0

/* FSYNR1 V0 fields */
#define KGSL_IOMMU_FSYNR1_AWRITE_MASK		0x00000001
#define KGSL_IOMMU_FSYNR1_AWRITE_SHIFT		8
/* FSYNR0 V1 fields */
#define KGSL_IOMMU_V1_FSYNR0_WNR_MASK		0x00000001
#define KGSL_IOMMU_V1_FSYNR0_WNR_SHIFT		4

/*
 * TTBR0 register fields
 * On arm64 bit mask is not required
 */
#ifdef CONFIG_ARM64
	#define KGSL_IOMMU_CTX_TTBR0_ADDR_MASK	0x0000FFFFFFFFFFFFULL
#else
	#ifdef CONFIG_IOMMU_LPAE
		#define KGSL_IOMMU_CTX_TTBR0_ADDR_MASK_LPAE \
					0x000000FFFFFFFFE0ULL
		#define KGSL_IOMMU_CTX_TTBR0_ADDR_MASK \
					KGSL_IOMMU_CTX_TTBR0_ADDR_MASK_LPAE
	#else
		#define KGSL_IOMMU_CTX_TTBR0_ADDR_MASK	0xFFFFC000
	#endif
#endif
>>>>>>> p9x

/* TLBSTATUS register fields */
#define KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE BIT(0)

/* IMPLDEF_MICRO_MMU_CTRL register fields */
#define KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT  0x00000004
#define KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE  0x00000008

/* SCTLR fields */
#define KGSL_IOMMU_SCTLR_HUPCF_SHIFT		8
<<<<<<< HEAD
#define KGSL_IOMMU_SCTLR_CFCFG_SHIFT		7
#define KGSL_IOMMU_SCTLR_CFIE_SHIFT		6

enum kgsl_iommu_reg_map {
	KGSL_IOMMU_CTX_SCTLR = 0,
	KGSL_IOMMU_CTX_TTBR0,
	KGSL_IOMMU_CTX_CONTEXTIDR,
=======

enum kgsl_iommu_reg_map {
	KGSL_IOMMU_GLOBAL_BASE = 0,
	KGSL_IOMMU_CTX_SCTLR,
	KGSL_IOMMU_CTX_TTBR0,
	KGSL_IOMMU_CTX_TTBR1,
>>>>>>> p9x
	KGSL_IOMMU_CTX_FSR,
	KGSL_IOMMU_CTX_FAR,
	KGSL_IOMMU_CTX_TLBIALL,
	KGSL_IOMMU_CTX_RESUME,
	KGSL_IOMMU_CTX_FSYNR0,
	KGSL_IOMMU_CTX_FSYNR1,
	KGSL_IOMMU_CTX_TLBSYNC,
	KGSL_IOMMU_CTX_TLBSTATUS,
<<<<<<< HEAD
	KGSL_IOMMU_REG_MAX
};

/* Max number of iommu clks per IOMMU unit */
#define KGSL_IOMMU_MAX_CLKS 5
=======
	KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL,
	KGSL_IOMMU_REG_MAX
};

struct kgsl_iommu_register_list {
	unsigned int reg_offset;
	int ctx_reg;
};

/* Max number of iommu clks per IOMMU unit */
#define KGSL_IOMMU_MAX_CLKS 6
>>>>>>> p9x

enum kgsl_iommu_context_id {
	KGSL_IOMMU_CONTEXT_USER = 0,
	KGSL_IOMMU_CONTEXT_SECURE = 1,
<<<<<<< HEAD
	KGSL_IOMMU_CONTEXT_MAX,
};

/* offset at which a nop command is placed in setstate */
#define KGSL_IOMMU_SETSTATE_NOP_OFFSET	1024

/*
 * struct kgsl_iommu_context - Structure holding data about an iommu context
 * bank
 * @dev: pointer to the iommu context's device
 * @name: context name
 * @id: The id of the context, used for deciding how it is used.
 * @cb_num: The hardware context bank number, used for calculating register
 *		offsets.
 * @kgsldev: The kgsl device that uses this context.
 * @fault: Flag when set indicates that this iommu device has caused a page
 * fault
 * @gpu_offset: Offset of this context bank in the GPU register space
 * @default_pt: The default pagetable for this context,
 *		it may be changed by self programming.
=======
	KGSL_IOMMU_CONTEXT_MAX = 2,
};

/**
 * struct kgsl_iommu_ctx - Struct holding context name and id
 * @dev:                Device pointer
 * @iommu_ctx_name:     Context name
 * @ctx_id:             Iommu context ID
 */
struct kgsl_iommu_ctx {
	struct device *dev;
	const char *iommu_ctx_name;
	enum kgsl_iommu_context_id ctx_id;
};

/**
 * struct kgsl_device_iommu_data - Struct holding iommu context data obtained
 * from dtsi file
 * @iommu_ctxs:         Pointer to array of struct holding context name and id
 * @iommu_ctx_count:    Number of contexts defined in the dtsi file
 * @regstart:           Start of iommu registers physical address
 * @regsize:            Size of registers physical address block
 * @clks                Iommu clocks
 * @features            Iommu features, ex RETENTION, DMA API
 */
struct kgsl_device_iommu_data {
	struct kgsl_iommu_ctx *iommu_ctxs;
	int iommu_ctx_count;
	unsigned int regstart;
	unsigned int regsize;
	struct clk *clks[KGSL_IOMMU_MAX_CLKS];
	unsigned int features;
};


#define KGSL_IOMMU_REG(iommu, ctx, REG) \
	((iommu)->regbase + \
	 (iommu)->iommu_reg_list[KGSL_IOMMU_CTX_##REG].reg_offset + \
	 ((ctx) << KGSL_IOMMU_CTX_SHIFT) + (iommu)->ctx_offset)

/* Macros to read/write IOMMU registers */
#define KGSL_IOMMU_SET_CTX_REG_Q(iommu, ctx, REG, val)	\
		writeq_relaxed((val), KGSL_IOMMU_REG(iommu, ctx, REG))

#define KGSL_IOMMU_GET_CTX_REG_Q(iommu, ctx, REG)		\
		readq_relaxed(KGSL_IOMMU_REG(iommu, ctx, REG))

#define KGSL_IOMMU_SET_CTX_REG(iommu, ctx, REG, val)	\
		writel_relaxed((val), KGSL_IOMMU_REG(iommu, ctx, REG))

#define KGSL_IOMMU_GET_CTX_REG(iommu, ctx, REG)		\
		readl_relaxed(KGSL_IOMMU_REG(iommu, ctx, REG))

/* Gets the lsb value of pagetable */
#define KGSL_IOMMMU_PT_LSB(iommu, pt_val)				\
	(pt_val & ~(KGSL_IOMMU_CTX_TTBR0_ADDR_MASK))

/* offset at which a nop command is placed in setstate_memory */
#define KGSL_IOMMU_SETSTATE_NOP_OFFSET	1024

/*
 * struct kgsl_iommu_context - Structure holding data about iommu contexts
 * @dev: Device pointer to iommu context
 * @name: context name
 * @attached: Indicates whether this iommu context is presently attached to
 * a pagetable/domain or not
 * @default_ttbr0: The TTBR0 value set by iommu driver on start up
 * @ctx_id: The hardware context ID for the device
 * are on, else the clocks are off
 * fault: Flag when set indicates that this iommu device has caused a page
 * fault
>>>>>>> p9x
 */
struct kgsl_iommu_context {
	struct device *dev;
	const char *name;
<<<<<<< HEAD
	enum kgsl_iommu_context_id id;
	unsigned int cb_num;
	struct kgsl_device *kgsldev;
	int fault;
	void __iomem *regbase;
	unsigned int gpu_offset;
	struct kgsl_pagetable *default_pt;
=======
	bool attached;
	uint64_t default_ttbr0;
	enum kgsl_iommu_context_id ctx_id;
	struct kgsl_device *kgsldev;
	int fault;
>>>>>>> p9x
};

/*
 * struct kgsl_iommu - Structure holding iommu data for kgsl driver
<<<<<<< HEAD
 * @ctx: Array of kgsl_iommu_context structs
 * @regbase: Virtual address of the IOMMU register base
 * @regstart: Physical address of the iommu registers
 * @regsize: Length of the iommu register region.
 * @setstate: Scratch GPU memory for IOMMU operations
 * @clk_enable_count: The ref count of clock enable calls
 * @clks: Array of pointers to IOMMU clocks
 * @micro_mmu_ctrl: GPU register offset of this glob al register
 * @smmu_info: smmu info used in a5xx preemption
 * @protect: register protection settings for the iommu.
 * @pagefault_suppression_count: Total number of pagefaults
 *				 suppressed since boot.
 */
struct kgsl_iommu {
	struct kgsl_iommu_context ctx[KGSL_IOMMU_CONTEXT_MAX];
	void __iomem *regbase;
	unsigned long regstart;
	unsigned int regsize;
	struct kgsl_memdesc setstate;
	atomic_t clk_enable_count;
	struct clk *clks[KGSL_IOMMU_MAX_CLKS];
	unsigned int micro_mmu_ctrl;
	struct kgsl_memdesc smmu_info;
	unsigned int version;
	struct kgsl_protected_registers protect;
	u32 pagefault_suppression_count;
=======
 * @device: Pointer to KGSL device struct
 * @ctx: Array of kgsl_iommu_context structs
 * @regbase: Virtual address of the IOMMU register base
 * @ahb_base_offset - The base address from where IOMMU registers can be
 * accesed from AHB bus
 * @clk_enable_count: The ref count of clock enable calls
 * @clks: Array of pointers to IOMMU clocks
 * @ctx_offset: The context offset to be added to base address when
 * accessing IOMMU registers from the CPU
 * @ctx_ahb_offset: The context offset to be added to base address when
 * accessing IOMMU registers from the GPU
 * @iommu_reg_list: List of IOMMU registers { offset, map, shift } array
 * @gtcu_iface_clk: The gTCU AHB Clock connected to SMMU
 * @smmu_info: smmu info used in a5xx preemption
 */
struct kgsl_iommu {
	struct kgsl_device *device;
	struct kgsl_iommu_context ctx[KGSL_IOMMU_CONTEXT_MAX];
	void __iomem *regbase;
	unsigned int ahb_base_offset;
	atomic_t clk_enable_count;
	struct clk *clks[KGSL_IOMMU_MAX_CLKS];
	unsigned int ctx_offset;
	unsigned int ctx_ahb_offset;
	struct kgsl_iommu_register_list *iommu_reg_list;
	struct clk *gtcu_iface_clk;
	struct clk *gtbu_clk;
	struct clk *gtbu1_clk;
	struct kgsl_memdesc smmu_info;
>>>>>>> p9x
};

/*
 * struct kgsl_iommu_pt - Iommu pagetable structure private to kgsl driver
 * @domain: Pointer to the iommu domain that contains the iommu pagetable
<<<<<<< HEAD
 * @ttbr0: register value to set when using this pagetable
 * @contextidr: register value to set when using this pagetable
 * @attached: is the pagetable attached?
 * @rbtree: all buffers mapped into the pagetable, indexed by gpuaddr
 * @va_start: Start of virtual range used in this pagetable.
 * @va_end: End of virtual range.
 * @svm_start: Start of shared virtual memory range. Addresses in this
 *		range are also valid in the process's CPU address space.
 * @svm_end: End of the shared virtual memory range.
 * @svm_start: 32 bit compatible range, for old clients who lack bits
 * @svm_end: end of 32 bit compatible range
 */
struct kgsl_iommu_pt {
	struct iommu_domain *domain;
	u64 ttbr0;
	u32 contextidr;
	bool attached;

	struct rb_root rbtree;

	uint64_t va_start;
	uint64_t va_end;
	uint64_t svm_start;
	uint64_t svm_end;
	uint64_t compat_va_start;
	uint64_t compat_va_end;
};

/*
 * offset of context bank 0 from the start of the SMMU register space.
 */
#define KGSL_IOMMU_CB0_OFFSET		0x8000
/* size of each context bank's register space */
#define KGSL_IOMMU_CB_SHIFT		12

/* Macros to read/write IOMMU registers */
extern const unsigned int kgsl_iommu_reg_list[KGSL_IOMMU_REG_MAX];

static inline void __iomem *
kgsl_iommu_reg(struct kgsl_iommu_context *ctx, enum kgsl_iommu_reg_map reg)
{
	BUG_ON(ctx->regbase == NULL);
	BUG_ON(reg >= KGSL_IOMMU_REG_MAX);
	return ctx->regbase + kgsl_iommu_reg_list[reg];
}

#define KGSL_IOMMU_SET_CTX_REG_Q(_ctx, REG, val) \
		writeq_relaxed((val), \
			kgsl_iommu_reg((_ctx), KGSL_IOMMU_CTX_##REG))

#define KGSL_IOMMU_GET_CTX_REG_Q(_ctx, REG) \
		readq_relaxed(kgsl_iommu_reg((_ctx), KGSL_IOMMU_CTX_##REG))

#define KGSL_IOMMU_SET_CTX_REG(_ctx, REG, val) \
		writel_relaxed((val), \
			kgsl_iommu_reg((_ctx), KGSL_IOMMU_CTX_##REG))

#define KGSL_IOMMU_GET_CTX_REG(_ctx, REG) \
		readl_relaxed(kgsl_iommu_reg((_ctx), KGSL_IOMMU_CTX_##REG))

=======
 * @iommu: Pointer to iommu structure
 * @pt_base: physical base pointer of this pagetable.
 */
struct kgsl_iommu_pt {
	struct iommu_domain *domain;
	struct kgsl_iommu *iommu;
	phys_addr_t pt_base;
};

/*
 * kgsl_msm_supports_iommu_v2 - Checks whether IOMMU version is V2 or not
 *
 * Checks whether IOMMU version is V2 or not by parsing nodes.
 * Return: 1 if IOMMU v2 is found else 0
 */
#ifdef CONFIG_OF
static inline int _kgsl_msm_checks_iommu_v2(void)
{
	struct device_node *node;
	node = of_find_compatible_node(NULL, NULL, "qcom,msm-smmu-v2");
	if (!node)
		node = of_find_compatible_node(NULL, NULL, "qcom,smmu-v2");
	if (node) {
		of_node_put(node);
		return 1;
	}
	return 0;
}
#endif

#if !defined(CONFIG_MSM_IOMMU_V0) && defined(CONFIG_OF)
static int soc_supports_v2 = -1;
static inline int kgsl_msm_supports_iommu_v2(void)
{
	if (soc_supports_v2 != -1)
		return soc_supports_v2;

	soc_supports_v2 = _kgsl_msm_checks_iommu_v2();

	return soc_supports_v2;
}
#else
static inline int kgsl_msm_supports_iommu_v2(void)
{
	return 0;
}
#endif
>>>>>>> p9x

#endif
