/*
 * OpenRISC prom.c
 *
 * Linux architectural port borrowing liberally from similar works of
 * others.  All original copyrights apply as per the original source
 * declaration.
 *
 * Modifications for the OpenRISC architecture:
 * Copyright (C) 2010-2011 Jonas Bonn <jonas@southpole.se>
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 *
 * Architecture specific procedures for creating, accessing and
 * interpreting the device tree.
 *
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/memblock.h>
#include <linux/of_fdt.h>

#include <asm/page.h>
<<<<<<< HEAD
=======
#include <asm/processor.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <asm/mmu.h>
#include <asm/pgtable.h>
#include <asm/sections.h>
#include <asm/setup.h>

extern char cmd_line[COMMAND_LINE_SIZE];
>>>>>>> p9x

void __init early_init_devtree(void *params)
{
	early_init_dt_scan(params);
	memblock_allow_resize();
}
<<<<<<< HEAD
=======

#ifdef CONFIG_BLK_DEV_INITRD
void __init early_init_dt_setup_initrd_arch(u64 start, u64 end)
{
	initrd_start = (unsigned long)__va(start);
	initrd_end = (unsigned long)__va(end);
	initrd_below_start_ok = 1;
}
#endif
>>>>>>> p9x
