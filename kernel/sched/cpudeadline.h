#ifndef _LINUX_CPUDL_H
#define _LINUX_CPUDL_H

#include <linux/sched.h>

#define IDX_INVALID     -1

<<<<<<< HEAD
struct cpudl_item {
	u64 dl;
	int cpu;
	int idx;
=======
struct array_item {
	u64 dl;
	int cpu;
>>>>>>> p9x
};

struct cpudl {
	raw_spinlock_t lock;
	int size;
<<<<<<< HEAD
	cpumask_var_t free_cpus;
	struct cpudl_item *elements;
=======
	int cpu_to_idx[NR_CPUS];
	struct array_item elements[NR_CPUS];
	cpumask_var_t free_cpus;
>>>>>>> p9x
};


#ifdef CONFIG_SMP
int cpudl_find(struct cpudl *cp, struct task_struct *p,
	       struct cpumask *later_mask);
void cpudl_set(struct cpudl *cp, int cpu, u64 dl, int is_valid);
int cpudl_init(struct cpudl *cp);
void cpudl_cleanup(struct cpudl *cp);
#else
#define cpudl_set(cp, cpu, dl) do { } while (0)
#define cpudl_init() do { } while (0)
#endif /* CONFIG_SMP */

#endif /* _LINUX_CPUDL_H */
