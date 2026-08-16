// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[RECOVERY]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <recovery/oplus_chg_recovery.h>
#include <recovery/state_keep.h>

static struct dentry *debug_root;

struct dentry *oplus_recovery_get_debug_root(void)
{
	return debug_root;
}

static int oplus_recovery_probe(struct platform_device *pdev)
{
	struct device_node *parent_node = pdev->dev.of_node;
	struct device_node *node;
	int rc;

	debug_root = debugfs_create_dir("oplus_chg_recovery", NULL);
	if (debug_root == NULL)
		chg_err("debugfs create failed\n");

	node = of_find_node_by_name(parent_node, "chg_state_keep");
	if (node != NULL) {
		rc = state_keep_init(&pdev->dev, node);
		if (rc < 0)
			chg_err("state_keep init failed, rc=%d\n", rc);
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_recovery_remove(struct platform_device *pdev)
#else
static int oplus_recovery_remove(struct platform_device *pdev)
#endif
{
	struct device_node *parent_node = pdev->dev.of_node;

	if (of_find_node_by_name(parent_node, "chg_state_keep") != NULL)
		state_keep_exit(&pdev->dev);

	if (debug_root != NULL)
		debugfs_remove_recursive(debug_root);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_recovery_match[] = {
	{ .compatible = "oplus,charge-recovery" },
	{},
};

static struct platform_driver oplus_recovery_driver = {
	.driver		= {
		.name = "oplus-charge-recovery",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_recovery_match),
	},
	.probe		= oplus_recovery_probe,
	.remove		= oplus_recovery_remove,
};

/* never return an error value */
static __init int oplus_recovery_init(void)
{
#if __and(IS_BUILTIN(CONFIG_OPLUS_CHG), IS_BUILTIN(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return 0;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return 0;
#endif /* CONFIG_OPLUS_CHG_V2 */
	return platform_driver_register(&oplus_recovery_driver);
}

static __exit void oplus_recovery_exit(void)
{
#if __and(IS_BUILTIN(CONFIG_OPLUS_CHG), IS_BUILTIN(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return;
#endif /* CONFIG_OPLUS_CHG_V2 */
	platform_driver_unregister(&oplus_recovery_driver);
}
oplus_chg_module_register(oplus_recovery);
