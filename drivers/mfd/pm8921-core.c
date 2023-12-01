/*
 * Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
<<<<<<< HEAD
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
=======
>>>>>>> p9x
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/ssbi.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>
#include <linux/mfd/core.h>
<<<<<<< HEAD

#define	SSBI_REG_ADDR_IRQ_BASE		0x1BB

#define	SSBI_REG_ADDR_IRQ_ROOT		(SSBI_REG_ADDR_IRQ_BASE + 0)
#define	SSBI_REG_ADDR_IRQ_M_STATUS1	(SSBI_REG_ADDR_IRQ_BASE + 1)
#define	SSBI_REG_ADDR_IRQ_M_STATUS2	(SSBI_REG_ADDR_IRQ_BASE + 2)
#define	SSBI_REG_ADDR_IRQ_M_STATUS3	(SSBI_REG_ADDR_IRQ_BASE + 3)
#define	SSBI_REG_ADDR_IRQ_M_STATUS4	(SSBI_REG_ADDR_IRQ_BASE + 4)
#define	SSBI_REG_ADDR_IRQ_BLK_SEL	(SSBI_REG_ADDR_IRQ_BASE + 5)
#define	SSBI_REG_ADDR_IRQ_IT_STATUS	(SSBI_REG_ADDR_IRQ_BASE + 6)
#define	SSBI_REG_ADDR_IRQ_CONFIG	(SSBI_REG_ADDR_IRQ_BASE + 7)
#define	SSBI_REG_ADDR_IRQ_RT_STATUS	(SSBI_REG_ADDR_IRQ_BASE + 8)

#define	PM_IRQF_LVL_SEL			0x01	/* level select */
#define	PM_IRQF_MASK_FE			0x02	/* mask falling edge */
#define	PM_IRQF_MASK_RE			0x04	/* mask rising edge */
#define	PM_IRQF_CLR			0x08	/* clear interrupt */
#define	PM_IRQF_BITS_MASK		0x70
#define	PM_IRQF_BITS_SHIFT		4
#define	PM_IRQF_WRITE			0x80

#define	PM_IRQF_MASK_ALL		(PM_IRQF_MASK_FE | \
					PM_IRQF_MASK_RE)
=======
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/regulator.h>
#include <linux/leds-pm8xxx.h>
>>>>>>> p9x

#define REG_HWREV		0x002  /* PMIC4 revision */
#define REG_HWREV_2		0x0E8  /* PMIC4 revision 2 */

<<<<<<< HEAD
#define PM8921_NR_IRQS		256

struct pm_irq_chip {
	struct regmap		*regmap;
	spinlock_t		pm_irq_lock;
	struct irq_domain	*irqdomain;
	unsigned int		num_irqs;
	unsigned int		num_blocks;
	unsigned int		num_masters;
	u8			config[0];
=======
#define REG_MPP_BASE		0x050
#define REG_IRQ_BASE		0x1BB

#define REG_TEMP_ALARM_CTRL	0x1B
#define REG_TEMP_ALARM_PWM	0x9B

#define REG_BATT_ALARM_THRESH	0x023
#define REG_BATT_ALARM_CTRL1	0x024
#define REG_BATT_ALARM_CTRL2	0x021
#define REG_BATT_ALARM_PWM_CTRL	0x020

#define PM8921_VERSION_MASK	0xFFF0
#define PM8921_VERSION_VALUE	0x06F0
#define PM8922_VERSION_VALUE	0x0AF0
#define PM8917_VERSION_VALUE	0x0CF0
#define PM8921_REVISION_MASK	0x000F

#define REG_PM8921_PON_CNTRL_3	0x01D
#define PM8921_RESTART_REASON_MASK	0x07

#define SINGLE_IRQ_RESOURCE(_name, _irq) \
{ \
	.name	= _name, \
	.start	= _irq, \
	.end	= _irq, \
	.flags	= IORESOURCE_IRQ, \
}

struct pm8921 {
	struct device					*dev;
	struct pm_irq_chip				*irq_chip;
	struct mfd_cell					*mfd_regulators;
	struct pm8xxx_regulator_core_platform_data	*regulator_cdata;
	u32						rev_registers;
	u8						restart_reason;
>>>>>>> p9x
};

static int pm8xxx_read_block_irq(struct pm_irq_chip *chip, unsigned int bp,
				 unsigned int *ip)
{
	int	rc;

<<<<<<< HEAD
	spin_lock(&chip->pm_irq_lock);
	rc = regmap_write(chip->regmap, SSBI_REG_ADDR_IRQ_BLK_SEL, bp);
	if (rc) {
		pr_err("Failed Selecting Block %d rc=%d\n", bp, rc);
		goto bail;
	}

	rc = regmap_read(chip->regmap, SSBI_REG_ADDR_IRQ_IT_STATUS, ip);
	if (rc)
		pr_err("Failed Reading Status rc=%d\n", rc);
bail:
	spin_unlock(&chip->pm_irq_lock);
	return rc;
}

static int
pm8xxx_config_irq(struct pm_irq_chip *chip, unsigned int bp, unsigned int cp)
{
	int	rc;

	spin_lock(&chip->pm_irq_lock);
	rc = regmap_write(chip->regmap, SSBI_REG_ADDR_IRQ_BLK_SEL, bp);
	if (rc) {
		pr_err("Failed Selecting Block %d rc=%d\n", bp, rc);
		goto bail;
	}

	cp |= PM_IRQF_WRITE;
	rc = regmap_write(chip->regmap, SSBI_REG_ADDR_IRQ_CONFIG, cp);
	if (rc)
		pr_err("Failed Configuring IRQ rc=%d\n", rc);
bail:
	spin_unlock(&chip->pm_irq_lock);
	return rc;
}

static int pm8xxx_irq_block_handler(struct pm_irq_chip *chip, int block)
{
	int pmirq, irq, i, ret = 0;
	unsigned int bits;

	ret = pm8xxx_read_block_irq(chip, block, &bits);
	if (ret) {
		pr_err("Failed reading %d block ret=%d", block, ret);
		return ret;
	}
	if (!bits) {
		pr_err("block bit set in master but no irqs: %d", block);
		return 0;
	}

	/* Check IRQ bits */
	for (i = 0; i < 8; i++) {
		if (bits & (1 << i)) {
			pmirq = block * 8 + i;
			irq = irq_find_mapping(chip->irqdomain, pmirq);
			generic_handle_irq(irq);
		}
	}
	return 0;
}

static int pm8xxx_irq_master_handler(struct pm_irq_chip *chip, int master)
{
	unsigned int blockbits;
	int block_number, i, ret = 0;

	ret = regmap_read(chip->regmap, SSBI_REG_ADDR_IRQ_M_STATUS1 + master,
			  &blockbits);
	if (ret) {
		pr_err("Failed to read master %d ret=%d\n", master, ret);
		return ret;
	}
	if (!blockbits) {
		pr_err("master bit set in root but no blocks: %d", master);
		return 0;
	}

	for (i = 0; i < 8; i++)
		if (blockbits & (1 << i)) {
			block_number = master * 8 + i;	/* block # */
			ret |= pm8xxx_irq_block_handler(chip, block_number);
		}
	return ret;
}

static void pm8xxx_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct pm_irq_chip *chip = irq_desc_get_handler_data(desc);
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	unsigned int root;
	int	i, ret, masters = 0;

	chained_irq_enter(irq_chip, desc);

	ret = regmap_read(chip->regmap, SSBI_REG_ADDR_IRQ_ROOT, &root);
	if (ret) {
		pr_err("Can't read root status ret=%d\n", ret);
		return;
	}

	/* on pm8xxx series masters start from bit 1 of the root */
	masters = root >> 1;

	/* Read allowed masters for blocks. */
	for (i = 0; i < chip->num_masters; i++)
		if (masters & (1 << i))
			pm8xxx_irq_master_handler(chip, i);

	chained_irq_exit(irq_chip, desc);
}

static void pm8xxx_irq_mask_ack(struct irq_data *d)
{
	struct pm_irq_chip *chip = irq_data_get_irq_chip_data(d);
	unsigned int pmirq = irqd_to_hwirq(d);
	u8	block, config;

	block = pmirq / 8;

	config = chip->config[pmirq] | PM_IRQF_MASK_ALL | PM_IRQF_CLR;
	pm8xxx_config_irq(chip, block, config);
}

static void pm8xxx_irq_unmask(struct irq_data *d)
{
	struct pm_irq_chip *chip = irq_data_get_irq_chip_data(d);
	unsigned int pmirq = irqd_to_hwirq(d);
	u8	block, config;

	block = pmirq / 8;

	config = chip->config[pmirq];
	pm8xxx_config_irq(chip, block, config);
}

static int pm8xxx_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	struct pm_irq_chip *chip = irq_data_get_irq_chip_data(d);
	unsigned int pmirq = irqd_to_hwirq(d);
	int irq_bit;
	u8 block, config;

	block = pmirq / 8;
	irq_bit  = pmirq % 8;

	chip->config[pmirq] = (irq_bit << PM_IRQF_BITS_SHIFT)
							| PM_IRQF_MASK_ALL;
	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		if (flow_type & IRQF_TRIGGER_RISING)
			chip->config[pmirq] &= ~PM_IRQF_MASK_RE;
		if (flow_type & IRQF_TRIGGER_FALLING)
			chip->config[pmirq] &= ~PM_IRQF_MASK_FE;
	} else {
		chip->config[pmirq] |= PM_IRQF_LVL_SEL;

		if (flow_type & IRQF_TRIGGER_HIGH)
			chip->config[pmirq] &= ~PM_IRQF_MASK_RE;
		else
			chip->config[pmirq] &= ~PM_IRQF_MASK_FE;
	}

	config = chip->config[pmirq] | PM_IRQF_CLR;
	return pm8xxx_config_irq(chip, block, config);
}

static struct irq_chip pm8xxx_irq_chip = {
	.name		= "pm8xxx",
	.irq_mask_ack	= pm8xxx_irq_mask_ack,
	.irq_unmask	= pm8xxx_irq_unmask,
	.irq_set_type	= pm8xxx_irq_set_type,
	.flags		= IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_SKIP_SET_WAKE,
};

static int pm8xxx_irq_domain_map(struct irq_domain *d, unsigned int irq,
				   irq_hw_number_t hwirq)
{
	struct pm_irq_chip *chip = d->host_data;

	irq_set_chip_and_handler(irq, &pm8xxx_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, chip);
#ifdef CONFIG_ARM
	set_irq_flags(irq, IRQF_VALID);
#else
	irq_set_noprobe(irq);
#endif
	return 0;
}

static const struct irq_domain_ops pm8xxx_irq_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = pm8xxx_irq_domain_map,
};

static const struct regmap_config ssbi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x3ff,
	.fast_io = true,
	.reg_read = ssbi_reg_read,
	.reg_write = ssbi_reg_write
};

static const struct of_device_id pm8921_id_table[] = {
	{ .compatible = "qcom,pm8058", },
	{ .compatible = "qcom,pm8921", },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8921_id_table);

static int pm8921_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	int irq, rc;
	unsigned int val;
	u32 rev;
	struct pm_irq_chip *chip;
	unsigned int nirqs = PM8921_NR_IRQS;
=======
	return ssbi_read(pmic->dev->parent, addr, val, 1);
}

static int pm8921_writeb(const struct device *dev, u16 addr, u8 val)
{
	const struct pm8xxx_drvdata *pm8921_drvdata = dev_get_drvdata(dev);
	const struct pm8921 *pmic = pm8921_drvdata->pm_chip_data;

	return ssbi_write(pmic->dev->parent, addr, &val, 1);
}

static int pm8921_read_buf(const struct device *dev, u16 addr, u8 *buf,
									int cnt)
{
	const struct pm8xxx_drvdata *pm8921_drvdata = dev_get_drvdata(dev);
	const struct pm8921 *pmic = pm8921_drvdata->pm_chip_data;

	return ssbi_read(pmic->dev->parent, addr, buf, cnt);
}

static int pm8921_write_buf(const struct device *dev, u16 addr, u8 *buf,
									int cnt)
{
	const struct pm8xxx_drvdata *pm8921_drvdata = dev_get_drvdata(dev);
	const struct pm8921 *pmic = pm8921_drvdata->pm_chip_data;

	return ssbi_write(pmic->dev->parent, addr, buf, cnt);
}

static int pm8921_read_irq_stat(const struct device *dev, int irq)
{
	const struct pm8xxx_drvdata *pm8921_drvdata = dev_get_drvdata(dev);
	const struct pm8921 *pmic = pm8921_drvdata->pm_chip_data;

	return pm8xxx_get_irq_stat(pmic->irq_chip, irq);
}

static enum pm8xxx_version pm8921_get_version(const struct device *dev)
{
	const struct pm8xxx_drvdata *pm8921_drvdata = dev_get_drvdata(dev);
	const struct pm8921 *pmic = pm8921_drvdata->pm_chip_data;
	enum pm8xxx_version version = -ENODEV;

	if ((pmic->rev_registers & PM8921_VERSION_MASK) == PM8921_VERSION_VALUE)
		version = PM8XXX_VERSION_8921;
	else if ((pmic->rev_registers & PM8921_VERSION_MASK)
			== PM8922_VERSION_VALUE)
		version = PM8XXX_VERSION_8922;
	else if ((pmic->rev_registers & PM8921_VERSION_MASK)
			== PM8917_VERSION_VALUE)
		version = PM8XXX_VERSION_8917;

	return version;
}

static int pm8921_get_revision(const struct device *dev)
{
	const struct pm8xxx_drvdata *pm8921_drvdata = dev_get_drvdata(dev);
	const struct pm8921 *pmic = pm8921_drvdata->pm_chip_data;

	return pmic->rev_registers & PM8921_REVISION_MASK;
}

static u8 pm8921_restart_reason(const struct device *dev)
{
	const struct pm8xxx_drvdata *pm8921_drvdata = dev_get_drvdata(dev);
	const struct pm8921 *pmic = pm8921_drvdata->pm_chip_data;

	return pmic->restart_reason;
}

static struct pm8xxx_drvdata pm8921_drvdata = {
	.pmic_readb		= pm8921_readb,
	.pmic_writeb		= pm8921_writeb,
	.pmic_read_buf		= pm8921_read_buf,
	.pmic_write_buf		= pm8921_write_buf,
	.pmic_read_irq_stat	= pm8921_read_irq_stat,
	.pmic_get_version	= pm8921_get_version,
	.pmic_get_revision	= pm8921_get_revision,
	.pmic_restart_reason	= pm8921_restart_reason,
};

static struct resource gpio_cell_resources[] = {
	[0] = {
		.start = PM8921_IRQ_BLOCK_BIT(PM8921_GPIO_BLOCK_START, 0),
		.end   = PM8921_IRQ_BLOCK_BIT(PM8921_GPIO_BLOCK_START, 0)
			+ PM8921_NR_GPIOS - 1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell gpio_cell = {
	.name		= PM8XXX_GPIO_DEV_NAME,
	.id		= -1,
	.resources	= gpio_cell_resources,
	.num_resources	= ARRAY_SIZE(gpio_cell_resources),
};

static const struct resource adc_cell_resources[] = {
	SINGLE_IRQ_RESOURCE(NULL, PM8921_ADC_EOC_USR_IRQ),
	SINGLE_IRQ_RESOURCE(NULL, PM8921_ADC_BATT_TEMP_WARM_IRQ),
	SINGLE_IRQ_RESOURCE(NULL, PM8921_ADC_BATT_TEMP_COLD_IRQ),
};

static struct mfd_cell adc_cell = {
	.name		= PM8XXX_ADC_DEV_NAME,
	.id		= -1,
	.resources	= adc_cell_resources,
	.num_resources	= ARRAY_SIZE(adc_cell_resources),
};

static struct resource mpp_cell_resources[] = {
	{
		.start	= PM8921_IRQ_BLOCK_BIT(PM8921_MPP_BLOCK_START, 0),
		.end	= PM8921_IRQ_BLOCK_BIT(PM8921_MPP_BLOCK_START, 0)
			  + PM8921_NR_MPPS - 1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct mfd_cell mpp_cell = {
	.name		= PM8XXX_MPP_DEV_NAME,
	.id		= 0,
	.resources	= mpp_cell_resources,
	.num_resources	= ARRAY_SIZE(mpp_cell_resources),
};

static const struct resource rtc_cell_resources[] = {
	[0] = SINGLE_IRQ_RESOURCE(NULL, PM8921_RTC_ALARM_IRQ),
	[1] = {
		.name   = "pmic_rtc_base",
		.start  = PM8921_RTC_BASE,
		.end    = PM8921_RTC_BASE,
		.flags  = IORESOURCE_IO,
	},
};

static struct mfd_cell rtc_cell = {
	.name           = PM8XXX_RTC_DEV_NAME,
	.id             = -1,
	.resources      = rtc_cell_resources,
	.num_resources  = ARRAY_SIZE(rtc_cell_resources),
};

static const struct resource resources_pwrkey[] = {
	SINGLE_IRQ_RESOURCE(NULL, PM8921_PWRKEY_REL_IRQ),
	SINGLE_IRQ_RESOURCE(NULL, PM8921_PWRKEY_PRESS_IRQ),
};

static struct mfd_cell pwrkey_cell = {
	.name		= PM8XXX_PWRKEY_DEV_NAME,
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_pwrkey),
	.resources	= resources_pwrkey,
};

static const struct resource resources_keypad[] = {
	SINGLE_IRQ_RESOURCE(NULL, PM8921_KEYPAD_IRQ),
	SINGLE_IRQ_RESOURCE(NULL, PM8921_KEYSTUCK_IRQ),
};

static struct mfd_cell keypad_cell = {
	.name		= PM8XXX_KEYPAD_DEV_NAME,
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_keypad),
	.resources	= resources_keypad,
};

static struct mfd_cell debugfs_cell = {
	.name		= "pm8xxx-debug",
	.id		= 0,
	.platform_data	= "pm8921-dbg",
	.pdata_size	= sizeof("pm8921-dbg"),
};

static struct mfd_cell pwm_cell = {
	.name           = PM8XXX_PWM_DEV_NAME,
	.id             = -1,
};

static const struct resource charger_cell_resources[] = {
	SINGLE_IRQ_RESOURCE("USBIN_VALID_IRQ", PM8921_USBIN_VALID_IRQ),
	SINGLE_IRQ_RESOURCE("USBIN_OV_IRQ", PM8921_USBIN_OV_IRQ),
	SINGLE_IRQ_RESOURCE("BATT_INSERTED_IRQ", PM8921_BATT_INSERTED_IRQ),
	SINGLE_IRQ_RESOURCE("VBATDET_LOW_IRQ", PM8921_VBATDET_LOW_IRQ),
	SINGLE_IRQ_RESOURCE("USBIN_UV_IRQ", PM8921_USBIN_UV_IRQ),
	SINGLE_IRQ_RESOURCE("VBAT_OV_IRQ", PM8921_VBAT_OV_IRQ),
	SINGLE_IRQ_RESOURCE("CHGWDOG_IRQ", PM8921_CHGWDOG_IRQ),
	SINGLE_IRQ_RESOURCE("VCP_IRQ", PM8921_VCP_IRQ),
	SINGLE_IRQ_RESOURCE("ATCDONE_IRQ", PM8921_ATCDONE_IRQ),
	SINGLE_IRQ_RESOURCE("ATCFAIL_IRQ", PM8921_ATCFAIL_IRQ),
	SINGLE_IRQ_RESOURCE("CHGDONE_IRQ", PM8921_CHGDONE_IRQ),
	SINGLE_IRQ_RESOURCE("CHGFAIL_IRQ", PM8921_CHGFAIL_IRQ),
	SINGLE_IRQ_RESOURCE("CHGSTATE_IRQ", PM8921_CHGSTATE_IRQ),
	SINGLE_IRQ_RESOURCE("LOOP_CHANGE_IRQ", PM8921_LOOP_CHANGE_IRQ),
	SINGLE_IRQ_RESOURCE("FASTCHG_IRQ", PM8921_FASTCHG_IRQ),
	SINGLE_IRQ_RESOURCE("TRKLCHG_IRQ", PM8921_TRKLCHG_IRQ),
	SINGLE_IRQ_RESOURCE("BATT_REMOVED_IRQ", PM8921_BATT_REMOVED_IRQ),
	SINGLE_IRQ_RESOURCE("BATTTEMP_HOT_IRQ", PM8921_BATTTEMP_HOT_IRQ),
	SINGLE_IRQ_RESOURCE("CHGHOT_IRQ", PM8921_CHGHOT_IRQ),
	SINGLE_IRQ_RESOURCE("BATTTEMP_COLD_IRQ", PM8921_BATTTEMP_COLD_IRQ),
	SINGLE_IRQ_RESOURCE("CHG_GONE_IRQ", PM8921_CHG_GONE_IRQ),
	SINGLE_IRQ_RESOURCE("BAT_TEMP_OK_IRQ", PM8921_BAT_TEMP_OK_IRQ),
	SINGLE_IRQ_RESOURCE("COARSE_DET_LOW_IRQ", PM8921_COARSE_DET_LOW_IRQ),
	SINGLE_IRQ_RESOURCE("VDD_LOOP_IRQ", PM8921_VDD_LOOP_IRQ),
	SINGLE_IRQ_RESOURCE("VREG_OV_IRQ", PM8921_VREG_OV_IRQ),
	SINGLE_IRQ_RESOURCE("VBATDET_IRQ", PM8921_VBATDET_IRQ),
	SINGLE_IRQ_RESOURCE("BATFET_IRQ", PM8921_BATFET_IRQ),
	SINGLE_IRQ_RESOURCE("PSI_IRQ", PM8921_PSI_IRQ),
	SINGLE_IRQ_RESOURCE("DCIN_VALID_IRQ", PM8921_DCIN_VALID_IRQ),
	SINGLE_IRQ_RESOURCE("DCIN_OV_IRQ", PM8921_DCIN_OV_IRQ),
	SINGLE_IRQ_RESOURCE("DCIN_UV_IRQ", PM8921_DCIN_UV_IRQ),
};

static const struct resource bms_cell_resources[] = {
	SINGLE_IRQ_RESOURCE("PM8921_BMS_SBI_WRITE_OK", PM8921_BMS_SBI_WRITE_OK),
	SINGLE_IRQ_RESOURCE("PM8921_BMS_CC_THR", PM8921_BMS_CC_THR),
	SINGLE_IRQ_RESOURCE("PM8921_BMS_VSENSE_THR", PM8921_BMS_VSENSE_THR),
	SINGLE_IRQ_RESOURCE("PM8921_BMS_VSENSE_FOR_R", PM8921_BMS_VSENSE_FOR_R),
	SINGLE_IRQ_RESOURCE("PM8921_BMS_OCV_FOR_R", PM8921_BMS_OCV_FOR_R),
	SINGLE_IRQ_RESOURCE("PM8921_BMS_GOOD_OCV", PM8921_BMS_GOOD_OCV),
	SINGLE_IRQ_RESOURCE("PM8921_BMS_VSENSE_AVG", PM8921_BMS_VSENSE_AVG),
};

static struct mfd_cell charger_cell = {
	.name		= PM8921_CHARGER_DEV_NAME,
	.id		= -1,
	.resources	= charger_cell_resources,
	.num_resources	= ARRAY_SIZE(charger_cell_resources),
};

static struct mfd_cell bms_cell = {
	.name		= PM8921_BMS_DEV_NAME,
	.id		= -1,
	.resources	= bms_cell_resources,
	.num_resources	= ARRAY_SIZE(bms_cell_resources),
};

static struct mfd_cell misc_cell = {
	.name           = PM8XXX_MISC_DEV_NAME,
	.id             = -1,
};

static struct mfd_cell leds_cell = {
	.name		= PM8XXX_LEDS_DEV_NAME,
	.id		= -1,
};

static const struct resource thermal_alarm_cell_resources[] = {
	SINGLE_IRQ_RESOURCE("pm8921_tempstat_irq", PM8921_TEMPSTAT_IRQ),
	SINGLE_IRQ_RESOURCE("pm8921_overtemp_irq", PM8921_OVERTEMP_IRQ),
};

static struct pm8xxx_tm_core_data thermal_alarm_cdata = {
	.adc_channel =			CHANNEL_DIE_TEMP,
	.adc_type =			PM8XXX_TM_ADC_PM8XXX_ADC,
	.reg_addr_temp_alarm_ctrl =	REG_TEMP_ALARM_CTRL,
	.reg_addr_temp_alarm_pwm =	REG_TEMP_ALARM_PWM,
	.tm_name =			"pm8921_tz",
	.irq_name_temp_stat =		"pm8921_tempstat_irq",
	.irq_name_over_temp =		"pm8921_overtemp_irq",
};

static struct mfd_cell thermal_alarm_cell = {
	.name		= PM8XXX_TM_DEV_NAME,
	.id		= -1,
	.resources	= thermal_alarm_cell_resources,
	.num_resources	= ARRAY_SIZE(thermal_alarm_cell_resources),
	.platform_data	= &thermal_alarm_cdata,
	.pdata_size	= sizeof(struct pm8xxx_tm_core_data),
};

static const struct resource batt_alarm_cell_resources[] = {
	SINGLE_IRQ_RESOURCE("pm8921_batt_alarm_irq", PM8921_BATT_ALARM_IRQ),
};

static struct pm8xxx_batt_alarm_core_data batt_alarm_cdata = {
	.irq_name		= "pm8921_batt_alarm_irq",
	.reg_addr_threshold	= REG_BATT_ALARM_THRESH,
	.reg_addr_ctrl1		= REG_BATT_ALARM_CTRL1,
	.reg_addr_ctrl2		= REG_BATT_ALARM_CTRL2,
	.reg_addr_pwm_ctrl	= REG_BATT_ALARM_PWM_CTRL,
};

static struct mfd_cell batt_alarm_cell = {
	.name		= PM8XXX_BATT_ALARM_DEV_NAME,
	.id		= -1,
	.resources	= batt_alarm_cell_resources,
	.num_resources	= ARRAY_SIZE(batt_alarm_cell_resources),
	.platform_data	= &batt_alarm_cdata,
	.pdata_size	= sizeof(struct pm8xxx_batt_alarm_core_data),
};

static const struct resource ccadc_cell_resources[] = {
	SINGLE_IRQ_RESOURCE("PM8921_BMS_CCADC_EOC", PM8921_BMS_CCADC_EOC),
};

static struct mfd_cell ccadc_cell = {
	.name		= PM8XXX_CCADC_DEV_NAME,
	.id		= -1,
	.resources	= ccadc_cell_resources,
	.num_resources	= ARRAY_SIZE(ccadc_cell_resources),
};

static struct mfd_cell vibrator_cell = {
	.name           = PM8XXX_VIBRATOR_DEV_NAME,
	.id             = -1,
};

static struct pm8xxx_vreg regulator_data[] = {
	/*   name	     pc_name	    ctrl   test   hpm_min */
	NLDO("8921_l1",      "8921_l1_pc",  0x0AE, 0x0AF, LDO_150),
	NLDO("8921_l2",      "8921_l2_pc",  0x0B0, 0x0B1, LDO_150),
	PLDO("8921_l3",      "8921_l3_pc",  0x0B2, 0x0B3, LDO_150),
	PLDO("8921_l4",      "8921_l4_pc",  0x0B4, 0x0B5, LDO_50),
	PLDO("8921_l5",      "8921_l5_pc",  0x0B6, 0x0B7, LDO_300),
	PLDO("8921_l6",      "8921_l6_pc",  0x0B8, 0x0B9, LDO_600),
	PLDO("8921_l7",      "8921_l7_pc",  0x0BA, 0x0BB, LDO_150),
	PLDO("8921_l8",      "8921_l8_pc",  0x0BC, 0x0BD, LDO_300),
	PLDO("8921_l9",      "8921_l9_pc",  0x0BE, 0x0BF, LDO_300),
	PLDO("8921_l10",     "8921_l10_pc", 0x0C0, 0x0C1, LDO_600),
	PLDO("8921_l11",     "8921_l11_pc", 0x0C2, 0x0C3, LDO_150),
	NLDO("8921_l12",     "8921_l12_pc", 0x0C4, 0x0C5, LDO_150),
	PLDO("8921_l14",     "8921_l14_pc", 0x0C8, 0x0C9, LDO_50),
	PLDO("8921_l15",     "8921_l15_pc", 0x0CA, 0x0CB, LDO_150),
	PLDO("8921_l16",     "8921_l16_pc", 0x0CC, 0x0CD, LDO_300),
	PLDO("8921_l17",     "8921_l17_pc", 0x0CE, 0x0CF, LDO_150),
	NLDO("8921_l18",     "8921_l18_pc", 0x0D0, 0x0D1, LDO_150),
	PLDO("8921_l21",     "8921_l21_pc", 0x0D6, 0x0D7, LDO_150),
	PLDO("8921_l22",     "8921_l22_pc", 0x0D8, 0x0D9, LDO_150),
	PLDO("8921_l23",     "8921_l23_pc", 0x0DA, 0x0DB, LDO_150),
	NLDO1200("8921_l24",		    0x0DC, 0x0DD, LDO_1200),
	NLDO1200("8921_l25",		    0x0DE, 0x0DF, LDO_1200),
	NLDO1200("8921_l26",		    0x0E0, 0x0E1, LDO_1200),
	NLDO1200("8921_l27",		    0x0E2, 0x0E3, LDO_1200),
	NLDO1200("8921_l28",		    0x0E4, 0x0E5, LDO_1200),
	PLDO("8921_l29",     "8921_l29_pc", 0x0E6, 0x0E7, LDO_150),

	/*   name	pc_name       ctrl   test2  clk    sleep  hpm_min */
	SMPS("8921_s1", "8921_s1_pc", 0x1D0, 0x1D5, 0x009, 0x1D2, SMPS_1500),
	SMPS("8921_s2", "8921_s2_pc", 0x1D8, 0x1DD, 0x00A, 0x1DA, SMPS_1500),
	SMPS("8921_s3", "8921_s3_pc", 0x1E0, 0x1E5, 0x00B, 0x1E2, SMPS_1500),
	SMPS("8921_s4", "8921_s4_pc", 0x1E8, 0x1ED, 0x011, 0x1EA, SMPS_1500),

	/*     name	  ctrl fts_cnfg1 pfm  pwr_cnfg  hpm_min */
	FTSMPS("8921_s5", 0x025, 0x02E, 0x026, 0x032, SMPS_2000),
	FTSMPS("8921_s6", 0x036, 0x03F, 0x037, 0x043, SMPS_2000),

	/*   name	pc_name       ctrl   test2  clk    sleep  hpm_min */
	SMPS("8921_s7", "8921_s7_pc", 0x1F0, 0x1F5, 0x012, 0x1F2, SMPS_1500),
	SMPS("8921_s8", "8921_s8_pc", 0x1F8, 0x1FD, 0x013, 0x1FA, SMPS_1500),

	/* name		       pc_name	       ctrl   test */
	VS("8921_lvs1",        "8921_lvs1_pc", 0x060, 0x061),
	VS300("8921_lvs2",		       0x062, 0x063),
	VS("8921_lvs3",        "8921_lvs3_pc", 0x064, 0x065),
	VS("8921_lvs4",        "8921_lvs4_pc", 0x066, 0x067),
	VS("8921_lvs5",        "8921_lvs5_pc", 0x068, 0x069),
	VS("8921_lvs6",        "8921_lvs6_pc", 0x06A, 0x06B),
	VS("8921_lvs7",        "8921_lvs7_pc", 0x06C, 0x06D),
	VS300("8921_usb_otg",		       0x06E, 0x06F),
	VS300("8921_hdmi_mvs",		       0x070, 0x071),

	/*  name	ctrl */
	NCP("8921_ncp", 0x090),
};

/*
 * PM8917 adds 6 LDOs and a boost regulator beyond those available on PM8921.
 * It also replaces SMPS 3 with FTSMPS 3.  PM8917 does not have an NCP.
 */
static struct pm8xxx_vreg pm8917_regulator_data[] = {
	/*   name	     pc_name	    ctrl   test   hpm_min */
	PLDO("8917_l30",     "8917_l30_pc", 0x0A3, 0x0A4, LDO_150),
	PLDO("8917_l31",     "8917_l31_pc", 0x0A5, 0x0A6, LDO_150),
	PLDO("8917_l32",     "8917_l32_pc", 0x0A7, 0x0A8, LDO_150),
	PLDO("8917_l33",     "8917_l33_pc", 0x0C6, 0x0C7, LDO_150),
	PLDO("8917_l34",     "8917_l34_pc", 0x0D2, 0x0D3, LDO_150),
	PLDO("8917_l35",     "8917_l35_pc", 0x0D4, 0x0D5, LDO_300),
	PLDO("8917_l36",     "8917_l36_pc", 0x0A9, 0x0AA, LDO_50),

	/*    name          ctrl */
	BOOST("8917_boost", 0x04B),
};

#define MAX_NAME_COMPARISON_LEN 32

static int match_regulator(enum pm8xxx_version version,
	struct pm8xxx_regulator_core_platform_data *core_data, const char *name)
{
	int found = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(regulator_data); i++) {
		if (regulator_data[i].rdesc.name
		    && strncmp(regulator_data[i].rdesc.name, name,
				MAX_NAME_COMPARISON_LEN) == 0) {
			core_data->is_pin_controlled = false;
			core_data->vreg = &regulator_data[i];
			found = 1;
			break;
		} else if (regulator_data[i].rdesc_pc.name
			   && strncmp(regulator_data[i].rdesc_pc.name, name,
				MAX_NAME_COMPARISON_LEN) == 0) {
			core_data->is_pin_controlled = true;
			core_data->vreg = &regulator_data[i];
			found = 1;
			break;
		}
	}
	if (version == PM8XXX_VERSION_8917) {
		for (i = 0; i < ARRAY_SIZE(pm8917_regulator_data); i++) {
			if (pm8917_regulator_data[i].rdesc.name
			    && strncmp(pm8917_regulator_data[i].rdesc.name,
					name, MAX_NAME_COMPARISON_LEN) == 0) {
				core_data->is_pin_controlled = false;
				core_data->vreg = &pm8917_regulator_data[i];
				found = 1;
				break;
			} else if (pm8917_regulator_data[i].rdesc_pc.name
			      && strncmp(pm8917_regulator_data[i].rdesc_pc.name,
					name, MAX_NAME_COMPARISON_LEN) == 0) {
				core_data->is_pin_controlled = true;
				core_data->vreg = &pm8917_regulator_data[i];
				found = 1;
				break;
			}
		}
	}

	if (!found)
		pr_err("could not find a match for regulator: %s\n", name);

	return found;
}

static int pm8921_add_regulators(const struct pm8921_platform_data *pdata,
		      struct pm8921 *pmic, int irq_base)
{
	int ret = 0;
	struct mfd_cell *mfd_regulators;
	struct pm8xxx_regulator_core_platform_data *cdata;
	enum pm8xxx_version version;
	int i;

	version = pm8xxx_get_version(pmic->dev);

	/* Add one device for each regulator used by the board. */
	mfd_regulators = kzalloc(sizeof(struct mfd_cell)
				 * (pdata->num_regulators), GFP_KERNEL);
	if (!mfd_regulators) {
		pr_err("Cannot allocate %d bytes for pm8921 regulator "
			"mfd cells\n", sizeof(struct mfd_cell)
					* (pdata->num_regulators));
		return -ENOMEM;
	}
	cdata = kzalloc(sizeof(struct pm8xxx_regulator_core_platform_data)
			* pdata->num_regulators, GFP_KERNEL);
	if (!cdata) {
		pr_err("Cannot allocate %d bytes for pm8921 regulator "
			"core data\n", pdata->num_regulators
			  * sizeof(struct pm8xxx_regulator_core_platform_data));
		kfree(mfd_regulators);
		return -ENOMEM;
	}
	for (i = 0; i < ARRAY_SIZE(regulator_data); i++)
		mutex_init(&regulator_data[i].pc_lock);
	for (i = 0; i < ARRAY_SIZE(pm8917_regulator_data); i++)
		mutex_init(&pm8917_regulator_data[i].pc_lock);

	for (i = 0; i < pdata->num_regulators; i++) {
		if (!pdata->regulator_pdatas[i].init_data.constraints.name) {
			pr_err("name missing for regulator %d\n", i);
			ret = -EINVAL;
			goto bail;
		}
		if (!match_regulator(version, &cdata[i],
		      pdata->regulator_pdatas[i].init_data.constraints.name)) {
			ret = -ENODEV;
			goto bail;
		}
		cdata[i].pdata = &(pdata->regulator_pdatas[i]);
		mfd_regulators[i].name = PM8XXX_REGULATOR_DEV_NAME;
		mfd_regulators[i].id = cdata[i].pdata->id;
		mfd_regulators[i].platform_data = &cdata[i];
		mfd_regulators[i].pdata_size =
			sizeof(struct pm8xxx_regulator_core_platform_data);
	}
	ret = mfd_add_devices(pmic->dev, 0, mfd_regulators,
			pdata->num_regulators, NULL, irq_base);
	if (ret)
		goto bail;

	pmic->mfd_regulators = mfd_regulators;
	pmic->regulator_cdata = cdata;
	return ret;

bail:
	for (i = 0; i < ARRAY_SIZE(regulator_data); i++)
		mutex_destroy(&regulator_data[i].pc_lock);
	for (i = 0; i < ARRAY_SIZE(pm8917_regulator_data); i++)
		mutex_destroy(&pm8917_regulator_data[i].pc_lock);
	kfree(mfd_regulators);
	kfree(cdata);
	return ret;
}

static int pm8921_add_subdevices(const struct pm8921_platform_data *pdata,
		      struct pm8921 *pmic)
{
	int ret = 0, irq_base = 0;
	struct pm_irq_chip *irq_chip;
	enum pm8xxx_version version;

	version = pm8xxx_get_version(pmic->dev);

	if (pdata->irq_pdata) {
		pdata->irq_pdata->irq_cdata.nirqs = PM8921_NR_IRQS;
		pdata->irq_pdata->irq_cdata.base_addr = REG_IRQ_BASE;
		irq_base = pdata->irq_pdata->irq_base;
		irq_chip = pm8xxx_irq_init(pmic->dev, pdata->irq_pdata);

		if (IS_ERR(irq_chip)) {
			pr_err("Failed to init interrupts ret=%ld\n",
					PTR_ERR(irq_chip));
			return PTR_ERR(irq_chip);
		}
		pmic->irq_chip = irq_chip;
	}

	if (pdata->gpio_pdata) {
		if (version == PM8XXX_VERSION_8917) {
			gpio_cell_resources[0].end = gpio_cell_resources[0].end
							+ PM8917_NR_GPIOS
							- PM8921_NR_GPIOS;
			pdata->gpio_pdata->gpio_cdata.ngpios = PM8917_NR_GPIOS;
		} else {
			pdata->gpio_pdata->gpio_cdata.ngpios = PM8921_NR_GPIOS;
		}
		gpio_cell.platform_data = pdata->gpio_pdata;
		gpio_cell.pdata_size = sizeof(struct pm8xxx_gpio_platform_data);
		ret = mfd_add_devices(pmic->dev, 0, &gpio_cell, 1,
					NULL, irq_base);
		if (ret) {
			pr_err("Failed to add  gpio subdevice ret=%d\n", ret);
			goto bail;
		}
	}

	if (pdata->mpp_pdata) {
		if (version == PM8XXX_VERSION_8917) {
			mpp_cell_resources[0].end = mpp_cell_resources[0].end
							+ PM8917_NR_MPPS
							- PM8921_NR_MPPS;
			pdata->mpp_pdata->core_data.nmpps = PM8917_NR_MPPS;
		} else {
			pdata->mpp_pdata->core_data.nmpps = PM8921_NR_MPPS;
		}
		pdata->mpp_pdata->core_data.base_addr = REG_MPP_BASE;
		mpp_cell.platform_data = pdata->mpp_pdata;
		mpp_cell.pdata_size = sizeof(struct pm8xxx_mpp_platform_data);
		ret = mfd_add_devices(pmic->dev, 0, &mpp_cell, 1, NULL,
					irq_base);
		if (ret) {
			pr_err("Failed to add mpp subdevice ret=%d\n", ret);
			goto bail;
		}
	}

	if (pdata->rtc_pdata) {
		rtc_cell.platform_data = pdata->rtc_pdata;
		rtc_cell.pdata_size = sizeof(struct pm8xxx_rtc_platform_data);
		ret = mfd_add_devices(pmic->dev, 0, &rtc_cell, 1, NULL,
				irq_base);
		if (ret) {
			pr_err("Failed to add rtc subdevice ret=%d\n", ret);
			goto bail;
		}
	}

	if (pdata->pwrkey_pdata) {
		pwrkey_cell.platform_data = pdata->pwrkey_pdata;
		pwrkey_cell.pdata_size =
			sizeof(struct pm8xxx_pwrkey_platform_data);
		ret = mfd_add_devices(pmic->dev, 0, &pwrkey_cell, 1, NULL,
					irq_base);
		if (ret) {
			pr_err("Failed to add pwrkey subdevice ret=%d\n", ret);
			goto bail;
		}
	}

	if (pdata->keypad_pdata) {
		keypad_cell.platform_data = pdata->keypad_pdata;
		keypad_cell.pdata_size =
			sizeof(struct pm8xxx_keypad_platform_data);
		ret = mfd_add_devices(pmic->dev, 0, &keypad_cell, 1, NULL,
					irq_base);
		if (ret) {
			pr_err("Failed to add keypad subdevice ret=%d\n", ret);
			goto bail;
		}
	}

	if (pdata->charger_pdata) {
		pdata->charger_pdata->charger_cdata.vbat_channel = CHANNEL_VBAT;
		pdata->charger_pdata->charger_cdata.batt_temp_channel
						= CHANNEL_BATT_THERM;
		pdata->charger_pdata->charger_cdata.batt_id_channel
						= CHANNEL_BATT_ID;
		charger_cell.platform_data = pdata->charger_pdata;
		charger_cell.pdata_size =
				sizeof(struct pm8921_charger_platform_data);
		ret = mfd_add_devices(pmic->dev, 0, &charger_cell, 1, NULL,
					irq_base);
		if (ret) {
			pr_err("Failed to add charger subdevice ret=%d\n", ret);
			goto bail;
		}
	}

	if (pdata->adc_pdata) {
		adc_cell.platform_data = pdata->adc_pdata;
		adc_cell.pdata_size =
			sizeof(struct pm8xxx_adc_platform_data);
		ret = mfd_add_devices(pmic->dev, 0, &adc_cell, 1, NULL,
					irq_base);
		if (ret) {
			pr_err("Failed to add regulator subdevices ret=%d\n",
					ret);
		}
	}

	if (pdata->bms_pdata) {
		pdata->bms_pdata->bms_cdata.batt_temp_channel
						= CHANNEL_BATT_THERM;
		pdata->bms_pdata->bms_cdata.vbat_channel = CHANNEL_VBAT;
		pdata->bms_pdata->bms_cdata.ref625mv_channel = CHANNEL_625MV;
		pdata->bms_pdata->bms_cdata.ref1p25v_channel = CHANNEL_125V;
		pdata->bms_pdata->bms_cdata.batt_id_channel = CHANNEL_BATT_ID;
		bms_cell.platform_data = pdata->bms_pdata;
		bms_cell.pdata_size = sizeof(struct pm8921_bms_platform_data);
		ret = mfd_add_devices(pmic->dev, 0, &bms_cell, 1, NULL,
					irq_base);
		if (ret) {
			pr_err("Failed to add bms subdevice ret=%d\n", ret);
			goto bail;
		}
	}

	if (pdata->num_regulators > 0 && pdata->regulator_pdatas) {
		ret = pm8921_add_regulators(pdata, pmic, irq_base);
		if (ret) {
			pr_err("Failed to add regulator subdevices ret=%d\n",
				ret);
			goto bail;
		}
	}

	ret = mfd_add_devices(pmic->dev, 0, &debugfs_cell, 1, NULL, irq_base);
	if (ret) {
		pr_err("Failed to add debugfs subdevice ret=%d\n", ret);
		goto bail;
	}

	if (pdata->misc_pdata) {
		misc_cell.platform_data = pdata->misc_pdata;
		misc_cell.pdata_size = sizeof(struct pm8xxx_misc_platform_data);
		ret = mfd_add_devices(pmic->dev, 0, &misc_cell, 1, NULL,
				      irq_base);
		if (ret) {
			pr_err("Failed to add  misc subdevice ret=%d\n", ret);
			goto bail;
		}
	}

	ret = mfd_add_devices(pmic->dev, 0, &thermal_alarm_cell, 1, NULL,
				irq_base);
	if (ret) {
		pr_err("Failed to add thermal alarm subdevice ret=%d\n",
			ret);
		goto bail;
	}

	ret = mfd_add_devices(pmic->dev, 0, &batt_alarm_cell, 1, NULL,
				irq_base);
	if (ret) {
		pr_err("Failed to add battery alarm subdevice ret=%d\n",
			ret);
		goto bail;
	}

	if (version != PM8XXX_VERSION_8917) {
		if (pdata->pwm_pdata) {
			pwm_cell.platform_data = pdata->pwm_pdata;
			pwm_cell.pdata_size =
				sizeof(struct pm8xxx_pwm_platform_data);
		}
		ret = mfd_add_devices(pmic->dev, 0, &pwm_cell, 1, NULL, 0);
		if (ret) {
			pr_err("Failed to add pwm subdevice ret=%d\n", ret);
			goto bail;
		}

		if (pdata->leds_pdata) {
			leds_cell.platform_data = pdata->leds_pdata;
			leds_cell.pdata_size =
				sizeof(struct pm8xxx_led_platform_data);
			ret = mfd_add_devices(pmic->dev, 0, &leds_cell,
					      1, NULL, 0);
			if (ret) {
				pr_err("Failed to add leds subdevice ret=%d\n",
						ret);
				goto bail;
			}
		}

		if (pdata->vibrator_pdata) {
			vibrator_cell.platform_data = pdata->vibrator_pdata;
			vibrator_cell.pdata_size =
				sizeof(struct pm8xxx_vibrator_platform_data);
			ret = mfd_add_devices(pmic->dev, 0, &vibrator_cell,
					      1, NULL, 0);
			if (ret) {
				pr_err("Failed to add vibrator ret=%d\n", ret);
				goto bail;
			}
		}
	}

	if (pdata->ccadc_pdata) {
		pdata->ccadc_pdata->ccadc_cdata.batt_temp_channel
						= CHANNEL_BATT_THERM;
		ccadc_cell.platform_data = pdata->ccadc_pdata;
		ccadc_cell.pdata_size =
				sizeof(struct pm8xxx_ccadc_platform_data);

		ret = mfd_add_devices(pmic->dev, 0, &ccadc_cell, 1, NULL,
					irq_base);
		if (ret) {
			pr_err("Failed to add ccadc subdevice ret=%d\n", ret);
			goto bail;
		}
	}

	return 0;
bail:
	if (pmic->irq_chip) {
		pm8xxx_irq_exit(pmic->irq_chip);
		pmic->irq_chip = NULL;
	}
	return ret;
}

static const char * const pm8921_rev_names[] = {
	[PM8XXX_REVISION_8921_TEST]	= "test",
	[PM8XXX_REVISION_8921_1p0]	= "1.0",
	[PM8XXX_REVISION_8921_1p1]	= "1.1",
	[PM8XXX_REVISION_8921_2p0]	= "2.0",
	[PM8XXX_REVISION_8921_3p0]	= "3.0",
	[PM8XXX_REVISION_8921_3p1]	= "3.1",
};

static const char * const pm8922_rev_names[] = {
	[PM8XXX_REVISION_8922_TEST]	= "test",
	[PM8XXX_REVISION_8922_1p0]	= "1.0",
	[PM8XXX_REVISION_8922_1p1]	= "1.1",
	[PM8XXX_REVISION_8922_2p0]	= "2.0",
};

static const char * const pm8917_rev_names[] = {
	[PM8XXX_REVISION_8917_TEST]	= "test",
	[PM8XXX_REVISION_8917_1p0]	= "1.0",
};

static int pm8921_probe(struct platform_device *pdev)
{
	const struct pm8921_platform_data *pdata = pdev->dev.platform_data;
	const char *revision_name = "unknown";
	struct pm8921 *pmic;
	enum pm8xxx_version version;
	int revision;
	int rc;
	u8 val;
>>>>>>> p9x

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	regmap = devm_regmap_init(&pdev->dev, NULL, pdev->dev.parent,
				  &ssbi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Read PMIC chip revision */
	rc = regmap_read(regmap, REG_HWREV, &val);
	if (rc) {
		pr_err("Failed to read hw rev reg %d:rc=%d\n", REG_HWREV, rc);
		return rc;
	}
	pr_info("PMIC revision 1: %02X\n", val);
	pmic->rev_registers = val;

	/* Read PMIC chip revision 2 */
	rc = regmap_read(regmap, REG_HWREV_2, &val);
	if (rc) {
		pr_err("Failed to read hw rev 2 reg %d:rc=%d\n",
			REG_HWREV_2, rc);
		return rc;
	}
	pr_info("PMIC revision 2: %02X\n", val);
	pmic->rev_registers |= val << BITS_PER_BYTE;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip) +
					sizeof(chip->config[0]) * nirqs,
					GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

<<<<<<< HEAD
	platform_set_drvdata(pdev, chip);
	chip->regmap = regmap;
	chip->num_irqs = nirqs;
	chip->num_blocks = DIV_ROUND_UP(chip->num_irqs, 8);
	chip->num_masters = DIV_ROUND_UP(chip->num_blocks, 8);
	spin_lock_init(&chip->pm_irq_lock);

	chip->irqdomain = irq_domain_add_linear(pdev->dev.of_node, nirqs,
						&pm8xxx_irq_domain_ops,
						chip);
	if (!chip->irqdomain)
		return -ENODEV;

	irq_set_handler_data(irq, chip);
	irq_set_chained_handler(irq, pm8xxx_irq_handler);
	irq_set_irq_wake(irq, 1);

	rc = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
=======
	/* Print out human readable version and revision names. */
	version = pm8xxx_get_version(pmic->dev);
	revision = pm8xxx_get_revision(pmic->dev);
	if (version == PM8XXX_VERSION_8921) {
		if (revision >= 0 && revision < ARRAY_SIZE(pm8921_rev_names))
			revision_name = pm8921_rev_names[revision];
		pr_info("PMIC version: PM8921 rev %s\n", revision_name);
	} else if (version == PM8XXX_VERSION_8922) {
		if (revision >= 0 && revision < ARRAY_SIZE(pm8922_rev_names))
			revision_name = pm8922_rev_names[revision];
		pr_info("PMIC version: PM8922 rev %s\n", revision_name);
	} else if (version == PM8XXX_VERSION_8917) {
		if (revision >= 0 && revision < ARRAY_SIZE(pm8917_rev_names))
			revision_name = pm8917_rev_names[revision];
		pr_info("PMIC version: PM8917 rev %s\n", revision_name);
	} else {
		WARN_ON(version != PM8XXX_VERSION_8921
			&& version != PM8XXX_VERSION_8922
			&& version != PM8XXX_VERSION_8917);
	}

	/* Log human readable restart reason */
	rc = msm_ssbi_read(pdev->dev.parent, REG_PM8921_PON_CNTRL_3, &val, 1);
	if (rc) {
		pr_err("Cannot read restart reason rc=%d\n", rc);
		goto err_read_rev;
	}
	val &= PM8XXX_RESTART_REASON_MASK;
	pr_info("PMIC Restart Reason: %s\n", pm8xxx_restart_reason_str[val]);
	pmic->restart_reason = val;

	rc = pm8921_add_subdevices(pdata, pmic);
>>>>>>> p9x
	if (rc) {
		irq_set_chained_handler(irq, NULL);
		irq_set_handler_data(irq, NULL);
		irq_domain_remove(chip->irqdomain);
	}

<<<<<<< HEAD
=======
	/* gpio might not work if no irq device is found */
	WARN_ON(pmic->irq_chip == NULL);

	return 0;

err:
	mfd_remove_devices(pmic->dev);
	platform_set_drvdata(pdev, NULL);
	kfree(pmic->mfd_regulators);
	kfree(pmic->regulator_cdata);
err_read_rev:
	kfree(pmic);
>>>>>>> p9x
	return rc;
}

static int pm8921_remove_child(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int pm8921_remove(struct platform_device *pdev)
{
<<<<<<< HEAD
	int irq = platform_get_irq(pdev, 0);
	struct pm_irq_chip *chip = platform_get_drvdata(pdev);

	device_for_each_child(&pdev->dev, NULL, pm8921_remove_child);
	irq_set_chained_handler(irq, NULL);
	irq_set_handler_data(irq, NULL);
	irq_domain_remove(chip->irqdomain);
=======
	struct pm8xxx_drvdata *drvdata;
	struct pm8921 *pmic = NULL;
	int i;

	drvdata = platform_get_drvdata(pdev);
	if (drvdata)
		pmic = drvdata->pm_chip_data;
	if (pmic) {
		if (pmic->dev)
			mfd_remove_devices(pmic->dev);
		if (pmic->irq_chip)
			pm8xxx_irq_exit(pmic->irq_chip);
		if (pmic->mfd_regulators) {
			for (i = 0; i < ARRAY_SIZE(regulator_data); i++)
				mutex_destroy(&regulator_data[i].pc_lock);
			for (i = 0; i < ARRAY_SIZE(pm8917_regulator_data); i++)
				mutex_destroy(
					&pm8917_regulator_data[i].pc_lock);
		}
		kfree(pmic->mfd_regulators);
		kfree(pmic->regulator_cdata);
		kfree(pmic);
	}
	platform_set_drvdata(pdev, NULL);
>>>>>>> p9x

	return 0;
}

static struct platform_driver pm8921_driver = {
	.probe		= pm8921_probe,
	.remove		= pm8921_remove,
	.driver		= {
		.name	= "pm8921-core",
		.owner	= THIS_MODULE,
		.of_match_table = pm8921_id_table,
	},
};

static int __init pm8921_init(void)
{
	return platform_driver_register(&pm8921_driver);
}
postcore_initcall(pm8921_init);

static void __exit pm8921_exit(void)
{
	platform_driver_unregister(&pm8921_driver);
}
module_exit(pm8921_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC 8921 core driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pm8921-core");
