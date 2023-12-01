#ifndef _ASM_X86_KASAN_H
#define _ASM_X86_KASAN_H

<<<<<<< HEAD
#include <linux/const.h>
#define KASAN_SHADOW_OFFSET _AC(CONFIG_KASAN_SHADOW_OFFSET, UL)

=======
>>>>>>> p9x
/*
 * Compiler uses shadow offset assuming that addresses start
 * from 0. Kernel addresses don't start from 0, so shadow
 * for kernel really starts from compiler's shadow offset +
 * 'kernel address space start' >> KASAN_SHADOW_SCALE_SHIFT
 */
#define KASAN_SHADOW_START      (KASAN_SHADOW_OFFSET + \
					(0xffff800000000000ULL >> 3))
/* 47 bits for kernel address -> (47 - 3) bits for shadow */
#define KASAN_SHADOW_END        (KASAN_SHADOW_START + (1ULL << (47 - 3)))

#ifndef __ASSEMBLY__

<<<<<<< HEAD
#ifdef CONFIG_KASAN
void __init kasan_early_init(void);
void __init kasan_init(void);
#else
static inline void kasan_early_init(void) { }
=======
extern pte_t kasan_zero_pte[];
extern pte_t kasan_zero_pmd[];
extern pte_t kasan_zero_pud[];

#ifdef CONFIG_KASAN
void __init kasan_map_early_shadow(pgd_t *pgd);
void __init kasan_init(void);
#else
static inline void kasan_map_early_shadow(pgd_t *pgd) { }
>>>>>>> p9x
static inline void kasan_init(void) { }
#endif

#endif

#endif
