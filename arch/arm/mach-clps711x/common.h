/*
 * linux/arch/arm/mach-clps711x/common.h
 *
 * Common bits.
 */

#include <linux/reboot.h>

<<<<<<< HEAD
=======
#define CLPS711X_NR_IRQS	(33)
>>>>>>> p9x
#define CLPS711X_NR_GPIO	(4 * 8 + 3)
#define CLPS711X_GPIO(prt, bit)	((prt) * 8 + (bit))

extern void clps711x_map_io(void);
extern void clps711x_init_irq(void);
extern void clps711x_timer_init(void);
<<<<<<< HEAD
extern void clps711x_restart(enum reboot_mode mode, const char *cmd);

/* drivers/irqchip/irq-clps711x.c */
void clps711x_intc_init(phys_addr_t, resource_size_t);
/* drivers/clk/clk-clps711x.c */
void clps711x_clk_init(void __iomem *base);
/* drivers/clocksource/clps711x-timer.c */
void clps711x_clksrc_init(void __iomem *tc1_base, void __iomem *tc2_base,
			  unsigned int irq);
=======
extern void clps711x_handle_irq(struct pt_regs *regs);
extern void clps711x_restart(enum reboot_mode mode, const char *cmd);
>>>>>>> p9x
