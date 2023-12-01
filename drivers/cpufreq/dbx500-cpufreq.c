/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010-2012
 *
 * License Terms: GNU General Public License v2
 * Author: Sundar Iyer <sundar.iyer@stericsson.com>
 * Author: Martin Persson <martin.persson@stericsson.com>
 * Author: Jonas Aaberg <jonas.aberg@stericsson.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

static struct cpufreq_frequency_table *freq_table;
static struct clk *armss_clk;

static int dbx500_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int index)
{
	/* update armss clk frequency */
	return clk_set_rate(armss_clk, freq_table[index].frequency * 1000);
}

static int dbx500_cpufreq_init(struct cpufreq_policy *policy)
{
<<<<<<< HEAD
	policy->clk = armss_clk;
	return cpufreq_generic_init(policy, freq_table, 20 * 1000);
=======
	int i = 0;
	unsigned long freq = clk_get_rate(armss_clk) / 1000;

	/* The value is rounded to closest frequency in the defined table. */
	while (freq_table[i + 1].frequency != CPUFREQ_TABLE_END) {
		if (freq < freq_table[i].frequency +
		   (freq_table[i + 1].frequency - freq_table[i].frequency) / 2)
			return freq_table[i].frequency;
		i++;
	}

	return freq_table[i].frequency;
}

static int dbx500_cpufreq_init(struct cpufreq_policy *policy)
{
	int res;

	/* get policy fields based on the table */
	res = cpufreq_frequency_table_cpuinfo(policy, freq_table);
	if (!res)
		cpufreq_frequency_table_get_attr(freq_table, policy->cpu);
	else {
		pr_err("dbx500-cpufreq: Failed to read policy table\n");
		return res;
	}

	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;
	policy->cur = dbx500_cpufreq_getspeed(policy->cpu);
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;

	/*
	 * FIXME : Need to take time measurement across the target()
	 *	   function with no/some/all drivers in the notification
	 *	   list.
	 */
	policy->cpuinfo.transition_latency = 20 * 1000; /* in ns */

	/* policy sharing between dual CPUs */
	cpumask_setall(policy->cpus);

	return 0;
>>>>>>> p9x
}

static struct cpufreq_driver dbx500_cpufreq_driver = {
	.flags  = CPUFREQ_STICKY | CPUFREQ_CONST_LOOPS |
			CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = dbx500_cpufreq_target,
	.get    = cpufreq_generic_get,
	.init   = dbx500_cpufreq_init,
	.name   = "DBX500",
	.attr   = cpufreq_generic_attr,
};

static int dbx500_cpufreq_probe(struct platform_device *pdev)
{
	struct cpufreq_frequency_table *pos;

	freq_table = dev_get_platdata(&pdev->dev);
	if (!freq_table) {
		pr_err("dbx500-cpufreq: Failed to fetch cpufreq table\n");
		return -ENODEV;
	}

	armss_clk = clk_get(&pdev->dev, "armss");
	if (IS_ERR(armss_clk)) {
		pr_err("dbx500-cpufreq: Failed to get armss clk\n");
		return PTR_ERR(armss_clk);
	}

	pr_info("dbx500-cpufreq: Available frequencies:\n");
	cpufreq_for_each_entry(pos, freq_table)
		pr_info("  %d Mhz\n", pos->frequency / 1000);

	return cpufreq_register_driver(&dbx500_cpufreq_driver);
}

static struct platform_driver dbx500_cpufreq_plat_driver = {
	.driver = {
		.name = "cpufreq-ux500",
	},
	.probe = dbx500_cpufreq_probe,
};

static int __init dbx500_cpufreq_register(void)
{
	return platform_driver_register(&dbx500_cpufreq_plat_driver);
}
device_initcall(dbx500_cpufreq_register);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("cpufreq driver for DBX500");
