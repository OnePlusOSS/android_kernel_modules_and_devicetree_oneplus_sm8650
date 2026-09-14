// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2024 Oplus. All rights reserved.
 */
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "internal.h"
#include "sys-memstat.h"
#include "mm-config.h"

#define MAX_CMDLINE_PARAM_LEN 128
#define MAX_UXMEM_POOL_SIZE 500
char feature_disable1[MAX_CMDLINE_PARAM_LEN];

#define MEM_1GB (1 << (30 - PAGE_SHIFT))
#define MEM_2GB (2 << (30 - PAGE_SHIFT))
#define MEM_3GB (3 << (30 - PAGE_SHIFT))
#define MEM_4GB (4 << (30 - PAGE_SHIFT))
#define MEM_6GB (6 << (30 - PAGE_SHIFT))
#define MEM_8GB (8 << (30 - PAGE_SHIFT))
#define MEM_12GB (12 << (30 - PAGE_SHIFT))
#define MEM_16GB (16 << (30 - PAGE_SHIFT))
#define MEM_24GB (24 << (30 - PAGE_SHIFT))

enum ram_val {
	RAM_1GB = 1,
	RAM_2GB,
	RAM_3GB,
	RAM_4GB,
	RAM_6GB = 6,
	RAM_8GB = 8,
	RAM_12GB = 12,
	RAM_16GB = 16,
	RAM_24GB = 24,
};

struct config_data {
	const char *module_name;
	void *private;
	struct list_head list;
	void (*seq_show)(struct seq_file *m, struct config_data *cd);
};
unsigned long cmdline_mm_reserved_1;

static LIST_HEAD(config_list);
static int read_ram_gb(void)
{
	int ram = -1;
	unsigned long total = sys_totalram();

	if (kasan_enabled()) {
		osvelte_logi("kasan enabled, add 1/8 to total\n");
		total = total / 7 * 8;
	}

	if (total < MEM_1GB)
		ram = RAM_1GB;
	else if (total < MEM_2GB)
		ram = RAM_2GB;
	else if (total < MEM_3GB)
		ram = RAM_3GB;
	else if (total < MEM_4GB)
		ram = RAM_4GB;
	else if (total < MEM_6GB)
		ram = RAM_6GB;
	else if (total < MEM_8GB)
		ram = RAM_8GB;
	else if (total < MEM_12GB)
		ram = RAM_12GB;
	else if (total < MEM_16GB)
		ram = RAM_16GB;
	else if (total < MEM_24GB)
		ram = RAM_24GB;
	return ram;
}

static void config_boost_pool_show(struct seq_file *m, struct config_data *cd)
{
	struct config_oplus_boost_pool *config = (struct config_oplus_boost_pool *)cd->private;

	seq_printf(m, "[%s]\n", cd->module_name);
	seq_printf(m, "  enable: %d\n", config->enable);
}

static void parse_boost_pool_dt(struct device_node *root)
{
	struct config_oplus_boost_pool *config;
	struct config_data *data;
	struct device_node *node;
	const char *name = module_name_boost_pool;

	node = of_get_child_by_name(root, name);
	if (!node)
		return;

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config) {
		osvelte_loge("failed to allocate\n");
		goto put_node;
	}

	config->enable = !of_property_read_bool(node, "feature-disable");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		osvelte_loge("failed to allocate config data\n");
		kfree(config);
		goto put_node;
	}

	data->module_name = name;
	INIT_LIST_HEAD(&data->list);
	data->private = config;
	data->seq_show = config_boost_pool_show;
	list_add_tail(&data->list, &config_list);
put_node:
	of_node_put(node);
}

static void config_uxmem_opt_show(struct seq_file *m, struct config_data *cd)
{
	struct config_oplus_bsp_uxmem_opt *config = (struct config_oplus_bsp_uxmem_opt *)cd->private;

	seq_printf(m, "[%s]\n", cd->module_name);
	seq_printf(m, " enable: %d\n", config->enable);
	seq_printf(m, " order 0 pool size(MB): %d\n", config->page_pool_order0_mb);
	seq_printf(m, " order 1 pool size(MB): %d\n", config->page_pool_order1_mb);
}

static void parse_uxmem_opt_dt(const struct device_node *root)
{
	struct config_oplus_bsp_uxmem_opt *config;
	struct config_data *data;
	struct device_node *node;
	const char *name = module_name_uxmem_opt;
	int err;

	node = of_get_child_by_name(root, name);
	if (!node)
		return;

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config) {
		osvelte_loge("failed to allocate\n");
		goto put_node;
	}

	config->enable = !of_property_read_bool(node, "feature-disable");

	err = of_property_read_u32(node, "page_pool_order0_mb", &config->page_pool_order0_mb);
	if (err) {
		osvelte_logi("read page_pool_order0_mb fail(%d)\n", err);
	} else if (config->page_pool_order0_mb > MAX_UXMEM_POOL_SIZE) {
		osvelte_loge("page_pool_order0_mb %d too large, set to %d\n", config->page_pool_order0_mb, MAX_UXMEM_POOL_SIZE);
		config->page_pool_order0_mb = MAX_UXMEM_POOL_SIZE;
	}

	err = of_property_read_u32(node, "page_pool_order1_mb", &config->page_pool_order1_mb);
	if (err) {
		osvelte_logi("read page_pool_order1_mb fail(%d)\n", err);
	} else if (config->page_pool_order1_mb > MAX_UXMEM_POOL_SIZE) {
		osvelte_loge("page_pool_order1_mb %d too large, set to %d\n", config->page_pool_order1_mb, MAX_UXMEM_POOL_SIZE);
		config->page_pool_order1_mb = MAX_UXMEM_POOL_SIZE;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		osvelte_loge("failed to allocate config data\n");
		kfree(config);
		goto put_node;
	}

	data->module_name = name;
	INIT_LIST_HEAD(&data->list);
	data->private = config;
	data->seq_show = config_uxmem_opt_show;
	list_add_tail(&data->list, &config_list);
put_node:
	of_node_put(node);
}

static void config_zram_opt_show(struct seq_file *m, struct config_data *cd)
{
	struct config_oplus_bsp_zram_opt *config = (struct config_oplus_bsp_zram_opt *)cd->private;

	seq_printf(m, "[%s]\n", cd->module_name);
	seq_printf(m, "  balance_anon_file_reclaim_always_true: %d\n",
		   config->balance_anon_file_reclaim_always_true);
}

static void parse_zram_opt_dt(const struct device_node *root)
{
	struct config_oplus_bsp_zram_opt *config;
	struct config_data *data;
	struct device_node *node;
	const char *name = module_name_zram_opt;

	node = of_get_child_by_name(root, name);
	if (!node)
		return;

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config) {
		osvelte_loge("failed to allocate\n");
		goto put_node;
	}

	config->balance_anon_file_reclaim_always_true = of_property_read_bool(node, "balance_anon_file_reclaim_always_true");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		osvelte_loge("failed to allocate config data\n");
		kfree(config);
		goto put_node;
	}

	data->module_name = name;
	INIT_LIST_HEAD(&data->list);
	data->private = config;
	data->seq_show = config_zram_opt_show;
	list_add_tail(&data->list, &config_list);
put_node:
	of_node_put(node);
}


static void config_ezreclaimd_show(struct seq_file *m, struct config_data *cd)
{
	struct config_ezreclaimd *config = (struct config_ezreclaimd *)cd->private;

	seq_printf(m, "[%s]\n", cd->module_name);
	seq_printf(m, "  enable: %d\n",
		   config->enable);
}

static void parse_ezreclaimd_dt(const struct device_node *root)
{
	struct config_ezreclaimd *config;
	struct config_data *data;
	struct device_node *node;
	const char *name = module_name_ezreclaimd;

	node = of_get_child_by_name(root, name);
	if (!node)
		return;

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config) {
		osvelte_loge("failed to allocate\n");
		goto put_node;
	}

	if (oplus_test_mm_feature_disable(COMFD1_EZRECLAIMD))
		osvelte_logi("ezreclaimd disabled by cmdline\n");
	else
		config->enable = of_property_read_bool(node, "feature-enable");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		osvelte_loge("failed to allocate config data\n");
		kfree(config);
		goto put_node;
	}

	data->module_name = name;
	INIT_LIST_HEAD(&data->list);
	data->private = config;
	data->seq_show = config_ezreclaimd_show;
	list_add_tail(&data->list, &config_list);
put_node:
	of_node_put(node);
}

static void config_kcompressed_show(struct seq_file *m, struct config_data *cd)
{
	struct config_kcompressed *config = (struct config_kcompressed *)cd->private;

	seq_printf(m, "[%s]\n", cd->module_name);
	seq_printf(m, "  enable: %d\n",
		   config->enable);
}

static void parse_kcompressed_dt(const struct device_node *root)
{
	struct config_kcompressed *config;
	struct config_data *data;
	struct device_node *node;
	const char *name = module_name_kcompressed;

	node = of_get_child_by_name(root, name);
	if (!node)
		return;

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config) {
		osvelte_loge("failed to allocate\n");
		goto put_node;
	}
	config->enable = of_property_read_bool(node, "feature-enable");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		osvelte_loge("failed to allocate config data\n");
		kfree(config);
		goto put_node;
	}

	data->module_name = name;
	INIT_LIST_HEAD(&data->list);
	data->private = config;
	data->seq_show = config_kcompressed_show;
	list_add_tail(&data->list, &config_list);
put_node:
	of_node_put(node);
}

static void config_mglru_opt_show(struct seq_file *m, struct config_data *cd)
{
	struct config_oplus_bsp_mglru_opt *config = (struct config_oplus_bsp_mglru_opt *)cd->private;

	seq_printf(m, "[%s]\n", cd->module_name);
	seq_printf(m, "  enable: %d\n", config->enable);
}

static void parse_mglru_opt_dt(const struct device_node *root)
{
	struct config_oplus_bsp_mglru_opt *config;
	struct config_data *data;
	struct device_node *node;
	const char *name = module_name_mglru_opt;

	node = of_get_child_by_name(root, name);
	if (!node)
		return;

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config) {
		osvelte_loge("failed to allocate\n");
		goto put_node;
	}

	config->enable = of_property_read_bool(node, "feature-enable");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		osvelte_loge("failed to allocate config data\n");
		kfree(config);
		goto put_node;
	}

	data->module_name = name;
	INIT_LIST_HEAD(&data->list);
	data->private = config;
	data->seq_show = config_mglru_opt_show;
	list_add_tail(&data->list, &config_list);
put_node:
	of_node_put(node);
}

/******************************************************************************
 *                          File Read Record
 ******************************************************************************/
static void config_frr_show(struct seq_file *m, struct config_data *cd)
{
	struct config_frr *config = (struct config_frr *)cd->private;

	seq_printf(m, "[%s]\n", cd->module_name);
	seq_printf(m, "  enable: %d\n",
		   config->enable);
}

static void parse_frr_dt(const struct device_node *root)
{
	struct config_frr *config;
	struct config_data *data;
	struct device_node *node;
	const char *name = module_name_frr;

	node = of_get_child_by_name(root, name);
	if (!node)
		return;

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config) {
		osvelte_loge("failed to allocate\n");
		goto put_node;
	}
	config->enable = of_property_read_bool(node, "feature-enable");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		osvelte_loge("failed to allocate config data\n");
		kfree(config);
		goto put_node;
	}

	data->module_name = name;
	INIT_LIST_HEAD(&data->list);
	data->private = config;
	data->seq_show = config_frr_show;
	list_add_tail(&data->list, &config_list);
put_node:
	of_node_put(node);
}
static int config_list_show(struct seq_file *m, void *data)
{
	/* module is initialized at boot stage, so no need lock to protect. */
	struct config_data *cd;
	seq_printf(m, "cmdline: %lx\n", cmdline_mm_reserved_1);

	seq_puts(m, "module config list\n");
	list_for_each_entry(cd, &config_list, list)
		cd->seq_show(m, cd);
	return 0;
}
DEFINE_PROC_SHOW_ATTRIBUTE(config_list);

static int parse_mm_config_dt(const struct platform_device *pdev)
{
	const struct device_node *dt_node = pdev->dev.of_node;
	struct device_node *child;
	int ram;
	char buf[256];

	ram = read_ram_gb();
	if (ram < 0) {
		osvelte_loge("unable read total ram\n");
		return -1;
	}
	osvelte_logi("ram: %dG\n", ram);

	/* no need check, always right */
	snprintf(buf, sizeof(buf), "ram-%dg", ram);
	child = of_get_child_by_name(dt_node, buf);
	if (!child) {
		osvelte_logi("failed to found %s node\n", buf);
		return 0;
	}

	/* add a function pointer */
	parse_boost_pool_dt(child);
	parse_uxmem_opt_dt(child);
	parse_zram_opt_dt(child);
	parse_ezreclaimd_dt(child);
	parse_kcompressed_dt(child);
	parse_mglru_opt_dt(child);
	parse_frr_dt(child);
	of_node_put(child);
	return 0;
}

void *oplus_read_mm_config(const char *module_name)
{
	struct config_data *cd;

	if (!module_name)
		return NULL;

	list_for_each_entry(cd, &config_list, list) {
		if (strcmp(module_name, cd->module_name) == 0)
			return cd->private;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(oplus_read_mm_config);

bool oplus_test_mm_feature_disable(unsigned long nr)
{
	return (1ul << nr) & cmdline_mm_reserved_1;
}
EXPORT_SYMBOL_GPL(oplus_test_mm_feature_disable);

static int mm_config_probe(struct platform_device *pdev)
{
	osvelte_logi("parse DT\n");
	parse_mm_config_dt(pdev);
	return 0;
}

static const struct of_device_id mm_config_match_table[] = {
	{.compatible = "oplus,mm_osvelte-config", },
	{},
};
MODULE_DEVICE_TABLE(of, mm_config_match_table);

static struct platform_driver mm_config_driver = {
	.probe = mm_config_probe,
	.driver = {
		.name = "oplus_bsp_mm_osvelte_config",
		.of_match_table = mm_config_match_table,
	},
};

int mm_config_init(struct proc_dir_entry *root)
{
	int ret;

	osvelte_logi("cmdline %s\n", feature_disable1);
	/* parse cmdline result */
	if (feature_disable1[0] != 0 &&
	    !kstrtoul(feature_disable1, 16, &cmdline_mm_reserved_1)) {
		osvelte_logi("mm_reserved_1: 0x%lx\n", cmdline_mm_reserved_1);
	}
	ret = platform_driver_register(&mm_config_driver);
	if (ret < 0) {
		osvelte_loge("failed to register\n");
		return ret;
	}

	proc_create("config", 0444, root, &config_list_proc_ops);
	return 0;
}

int mm_config_exit(void)
{
	/* technically should free module config data. */
	platform_driver_unregister(&mm_config_driver);
	return 0;
}
module_param_string(feature_disable1, feature_disable1, MAX_CMDLINE_PARAM_LEN, 0600);
