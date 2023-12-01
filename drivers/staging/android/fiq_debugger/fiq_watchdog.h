<<<<<<<< HEAD:drivers/staging/android/fiq_debugger/fiq_watchdog.h
/*
 * Copyright (C) 2014 Google, Inc.
========
/* arch/arm/mach-msm/include/mach/hardware.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
>>>>>>>> p9x:arch/arm/mach-msm/include/mach/hardware.h
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

<<<<<<<< HEAD:drivers/staging/android/fiq_debugger/fiq_watchdog.h
#ifndef _FIQ_WATCHDOG_H_
#define _FIQ_WATCHDOG_H_

void fiq_watchdog_triggered(const struct pt_regs *regs, void *svc_sp);
========
#ifndef __ASM_ARCH_MSM_HARDWARE_H
#define __ASM_ARCH_MSM_HARDWARE_H

#define pcibios_assign_all_busses()     1
>>>>>>>> p9x:arch/arm/mach-msm/include/mach/hardware.h

#endif
