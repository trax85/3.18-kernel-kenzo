<<<<<<< HEAD
/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
 */
#ifndef __QPNP_HAPTIC_H
<<<<<<< HEAD
#define __QPNP_HAPTIC_H
=======
>>>>>>> p9x

/* interface for the other module to play different sequences */
#ifdef CONFIG_QPNP_HAPTIC
int qpnp_hap_play_byte(u8 data, bool on);
#else
<<<<<<< HEAD
static inline int qpnp_hap_play_byte(u8 data, bool on)
=======
int qpnp_hap_play_byte(u8 data, bool on);
>>>>>>> p9x
{
	return 0;
}
#endif
#endif
