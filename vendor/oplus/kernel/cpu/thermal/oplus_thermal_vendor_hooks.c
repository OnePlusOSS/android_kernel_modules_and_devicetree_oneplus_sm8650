// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Oplus. All rights reserved.
 */

#include <linux/thermal.h>
#include <trace/hooks/thermal.h>
#include <linux/moduleparam.h>
#include <linux/module.h>


static int irq_wakeable_value = 0;
module_param_named(irq_wakeable, irq_wakeable_value, int, 0644);

static void oplus_thermal_pm_notify(void *unused,
		struct thermal_zone_device *tz, int *irq_wakeable)
{
	*irq_wakeable = irq_wakeable_value;
}

static int __init oplus_thermal_vendor_hook_driver_init(void)
{
	int ret;

	ret = register_trace_android_vh_thermal_pm_notify_suspend(
			oplus_thermal_pm_notify, NULL);
	if (ret)
		pr_err("Failed to register thermal_pm_notify hook, err:%d\n",
			ret);

	return 0;
}

static void __exit oplus_thermal_vendor_hook_driver_exit(void)
{
	unregister_trace_android_vh_thermal_pm_notify_suspend(
			oplus_thermal_pm_notify, NULL);
}

#if IS_MODULE(CONFIG_OPLUS_THERMAL_VENDOR_HOOK)
module_init(oplus_thermal_vendor_hook_driver_init);
#else
subsys_initcall(oplus_thermal_vendor_hook_driver_init);
#endif
module_exit(oplus_thermal_vendor_hook_driver_exit);

MODULE_DESCRIPTION("Oplus Thermal Vendor Hooks Driver");
MODULE_LICENSE("GPL");
