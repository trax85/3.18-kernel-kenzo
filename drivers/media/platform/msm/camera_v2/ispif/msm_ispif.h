<<<<<<< HEAD
/* Copyright (c) 2013-2017, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#ifndef MSM_ISPIF_H
#define MSM_ISPIF_H

#include <linux/clk.h>
#include <linux/io.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_ispif.h>
#include "msm_sd.h"

<<<<<<< HEAD
/* Maximum number of voltage supply for ispif and vfe */
#define ISPIF_VDD_INFO_MAX 2
#define ISPIF_VFE_VDD_INFO_MAX 2

#define ISPIF_CLK_INFO_MAX 27
=======
#define ISPIF_CLK_INFO_MAX 24
>>>>>>> p9x

struct ispif_irq_status {
	uint32_t ispifIrqStatus0;
	uint32_t ispifIrqStatus1;
	uint32_t ispifIrqStatus2;
};

enum msm_ispif_state_t {
	ISPIF_POWER_UP,
	ISPIF_POWER_DOWN,
};
struct ispif_sof_count {
	uint32_t sof_cnt[INTF_MAX];
};

struct ispif_intf_cmd {
	uint32_t intf_cmd;
	uint32_t intf_cmd1;
};

struct ispif_device {
	struct platform_device *pdev;
	struct msm_sd_subdev msm_sd;
<<<<<<< HEAD
	struct resource *irq;
=======
	struct resource *mem;
	struct resource *clk_mux_mem;
	struct resource *irq;
	struct resource *io;
	struct resource *clk_mux_io;
>>>>>>> p9x
	void __iomem *base;
	void __iomem *clk_mux_base;
	struct mutex mutex;
	uint8_t start_ack_pending;
	uint32_t csid_version;
	int enb_dump_reg;
	uint32_t open_cnt;
	struct ispif_sof_count sof_count[VFE_MAX];
	struct ispif_intf_cmd applied_intf_cmd[VFE_MAX];
	enum msm_ispif_state_t ispif_state;
	struct msm_ispif_vfe_info vfe_info;
<<<<<<< HEAD
	struct clk **ahb_clk;
	struct msm_cam_clk_info *ahb_clk_info;
	struct clk **clks;
	struct msm_cam_clk_info *clk_info;
	struct completion reset_complete[VFE_MAX];
	atomic_t reset_trig[VFE_MAX];
=======
	struct clk *ahb_clk[ISPIF_CLK_INFO_MAX];
	struct clk *clk[ISPIF_CLK_INFO_MAX];
	struct completion reset_complete[VFE_MAX];
>>>>>>> p9x
	uint32_t hw_num_isps;
	uint32_t num_ahb_clk;
	uint32_t num_clk;
	uint32_t clk_idx;
	uint32_t ispif_sof_debug;
	uint32_t ispif_rdi0_debug;
	uint32_t ispif_rdi1_debug;
	uint32_t ispif_rdi2_debug;
<<<<<<< HEAD
	struct regulator *ispif_vdd[ISPIF_VDD_INFO_MAX];
	int ispif_vdd_count;
	struct regulator *vfe_vdd[ISPIF_VFE_VDD_INFO_MAX];
	int vfe_vdd_count;
	int stereo_configured[VFE_MAX];
=======
	struct regulator *fs_vfe0;
	struct regulator *fs_vfe1;
	struct regulator *fs_mmagic_camss;
	struct regulator *fs_camss;
>>>>>>> p9x
};
#endif
