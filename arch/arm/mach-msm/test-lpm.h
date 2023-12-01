<<<<<<<< HEAD:arch/arm/boot/dts/qcom/mdm9640-ion.dtsi
/* Copyright (c) 2014, Linux Foundation. All rights reserved.
========
/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
>>>>>>>> p9x:arch/arm/mach-msm/test-lpm.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
<<<<<<<< HEAD:arch/arm/boot/dts/qcom/mdm9640-ion.dtsi
 */

&soc {
	qcom,ion {
		compatible = "qcom,msm-ion";
		#address-cells = <1>;
		#size-cells = <0>;

		qcom,ion-heap@25 {
			reg = <25>;
			qcom,ion-heap-type = "SYSTEM";
		};

	qcom,ion-heap@28 { /* AUDIO HEAP */
		reg = <28>;
		memory-region = <&audio_mem>;
		qcom,ion-heap-type = "DMA";
		};
	};
};
========
 *
 */

#ifndef __ARCH_ARM_MACH_MSM_TEST_LPM_H
#define __ARCH_ARM_MACH_MSM_TEST_LPM_H

struct lpm_test_platform_data {
	struct msm_rpmrs_level *msm_lpm_test_levels;
	int msm_lpm_test_level_count;
	bool use_qtimer;
};
#endif
>>>>>>>> p9x:arch/arm/mach-msm/test-lpm.h
