/*
 * Copyright (C) 2012 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PLATFORM_DATA_DS2482__
#define __PLATFORM_DATA_DS2482__

struct ds2482_platform_data {
	int		slpz_gpio;
};

<<<<<<<< HEAD:include/linux/platform_data/ds2482.h
#endif /* __PLATFORM_DATA_DS2482__ */
========
#ifdef CONFIG_HAVE_ARCH_HAS_CURRENT_TIMER
#define ARCH_HAS_READ_CURRENT_TIMER
#endif

#endif
>>>>>>>> p9x:arch/arm/mach-msm/include/mach/timex.h
