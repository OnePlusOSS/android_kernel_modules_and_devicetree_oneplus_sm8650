// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[VIRTUAL_CP]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/iio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>

#define CP_REG_TIMEOUT_MS	120000
#define PROC_DATA_BUF_SIZE	256
#define CP_NAME_BUF_MAX		128
#define MONITOR_WORK_DELAY_MS	500

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
#define pde_data(inode) PDE_DATA(inode)
#endif

struct oplus_virtual_cp_child {
	struct oplus_chg_ic_dev *parent;
	struct oplus_chg_ic_dev *ic_dev;
	int index;
	int max_curr_ma;
	int sw_ocp_count;
	struct work_struct online_work;
	struct work_struct offline_work;
};

struct oplus_cp_strategy;
struct oplus_virtual_cp_ic;

struct oplus_cp_strategy_desc {
	enum oplus_cp_strategy_type type;
	struct oplus_cp_strategy *(*strategy_alloc)(struct oplus_virtual_cp_ic *cp, struct device_node *node);
	int (*strategy_release)(struct oplus_cp_strategy *strategy);
	int (*strategy_init)(struct oplus_cp_strategy *strategy);
	int (*strategy_get_open_data)(struct oplus_cp_strategy *strategy, enum oplus_cp_work_mode mode);
	int (*strategy_get_data)(struct oplus_cp_strategy *strategy, enum oplus_cp_work_mode mode, int curr_ma);
};

struct oplus_cp_strategy {
	struct oplus_virtual_cp_ic *cp;
	struct oplus_cp_strategy_desc *desc;

	bool initialized;
};

struct oplus_virtual_cp_ic {
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	struct device_node *active_node;
	bool online;
	enum oplus_chg_ic_connect_type connect_type;
	int child_num;

	struct oplus_virtual_cp_child *child_list;
	struct oplus_cp_strategy *strategy;
	struct delayed_work monitor_work;
	struct mutex online_lock;

	enum oplus_cp_work_mode work_mode;
	int work_status_change_count;
	bool cp_enable; /* TODO: wireless charging redefines cp_enable */

	/* parallel charge */
	int main_cp;
	int target_iin;
	unsigned long open_flag;
	unsigned long pre_open_flag;
	unsigned int open_flag_change_count;
	bool enable;

	bool reg_proc_node;
};

/* CP_STRAT_OPEN_ALL start */
static struct oplus_cp_strategy *open_all_strategy_alloc(
	struct oplus_virtual_cp_ic *cp, struct device_node *node)
{
	struct oplus_cp_strategy *strategy;

	strategy = kzalloc(sizeof(struct oplus_cp_strategy), GFP_KERNEL);
	if (strategy == NULL) {
		chg_err("alloc %s strategy buf error\n",
			oplus_cp_strategy_type_str(CP_STRAT_OPEN_ALL));
		return NULL;
	}

	return strategy;
}

static int open_all_strategy_release(struct oplus_cp_strategy *strategy)
{
	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}

	kfree(strategy);
	return 0;
}

static int open_all_strategy_init(struct oplus_cp_strategy *strategy)
{
	return 0;
}

static int open_all_strategy_get_open_data(
	struct oplus_cp_strategy *strategy, enum oplus_cp_work_mode mode)
{
	int i;
	int rc;
	int open = 0;
	struct oplus_chg_ic_dev *ic_dev;

	for (i = 0; i < strategy->cp->child_num; i++) {
		ic_dev = strategy->cp->child_list[i].ic_dev;
		if (ic_dev == NULL)
			continue;
		if (!ic_dev->online)
			continue;
		rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT, mode);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] check work mode support error, rc=%d\n", i, rc);
			continue;
		}
		if (rc == -ENOTSUPP)
			continue;
		open |= BIT(i);
	}

	return open;
}

/* CP_STRAT_OPEN_ALL end */

/* CP_STRAT_OPEN_BY_CURR start */
struct oplus_cp_obc_strategy {
	struct oplus_cp_strategy strategy;
	u32 *open_thr_ma;
	u32 *close_thr_ma;
	int data_num;
	u32 data[];
};

static struct oplus_cp_strategy *obc_strategy_alloc(
	struct oplus_virtual_cp_ic *cp, struct device_node *node)
{
	struct oplus_cp_obc_strategy *strategy;
	int num;
	int rc;
	int i;

	if (node == NULL) {
		chg_err("device node is NULL\n");
		return NULL;
	}

	rc = of_property_count_elems_of_size(node,
		"oplus,cp_open_curr_thr_ma", sizeof(u32));
	if (rc < 0) {
		chg_err("can't get \"oplus,cp_open_curr_thr_ma\" number, rc=%d\n", rc);
		return NULL;
	}
	num = rc;
	rc = of_property_count_elems_of_size(node,
		"oplus,cp_close_curr_thr_ma", sizeof(u32));
	if (rc < 0) {
		chg_err("can't get \"oplus,cp_close_curr_thr_ma\" number, rc=%d\n", rc);
		return NULL;
	}
	if (num != rc) {
		chg_err("open_curr_thr and close_curr_thr data quantity does not match\n");
		return NULL;
	}

	strategy = kzalloc(sizeof(struct oplus_cp_obc_strategy) + sizeof(u32) * num * 2, GFP_KERNEL);
	if (strategy == NULL) {
		chg_err("alloc %s strategy buf error\n",
			oplus_cp_strategy_type_str(CP_STRAT_OPEN_BY_CURR));
		return NULL;
	}

	strategy->data_num = num;
	strategy->open_thr_ma = strategy->data;
	strategy->close_thr_ma = &strategy->data[num];

	rc = of_property_read_u32_array(node, "oplus,cp_open_curr_thr_ma",
		strategy->open_thr_ma, num);
	if (rc < 0) {
		chg_err("can't read \"oplus,cp_open_curr_thr_ma\", rc=%d\n", rc);
		goto err;
	}
	for (i = 0; i < num; i++)
		chg_info("open_thr_ma[%d]: %u\n", i, strategy->open_thr_ma[i]);
	rc = of_property_read_u32_array(node, "oplus,cp_close_curr_thr_ma",
		strategy->close_thr_ma, num);
	if (rc < 0) {
		chg_err("can't read \"oplus,cp_close_curr_thr_ma\", rc=%d\n", rc);
		goto err;
	}
	for (i = 0; i < num; i++)
		chg_info("close_thr_ma[%d]: %u\n", i, strategy->close_thr_ma[i]);

	return &strategy->strategy;

err:
	kfree(strategy);
	return NULL;
}

static int obc_strategy_release(struct oplus_cp_strategy *strategy)
{
	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}

	kfree(strategy);
	return 0;
}

static int obc_strategy_init(struct oplus_cp_strategy *strategy)
{
	return 0;
}

static int obc_strategy_get_open_data(
	struct oplus_cp_strategy *strategy, enum oplus_cp_work_mode mode)
{
	int i;
	int rc;
	int open = 0;
	struct oplus_chg_ic_dev *ic_dev;

	for (i = 0; i < strategy->cp->child_num; i++) {
		ic_dev = strategy->cp->child_list[i].ic_dev;
		if (ic_dev == NULL)
			continue;
		if (!ic_dev->online)
			continue;
		rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT, mode);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] check work mode support error, rc=%d\n", i, rc);
			continue;
		}
		if ((rc == -ENOTSUPP) || (rc == 0))
			continue;
		open = BIT(i);
		break;
	}
	chg_debug("open_flag=0x%x\n", open);

	return open;
}

static int obc_strategy_get_data(struct oplus_cp_strategy *strategy,
	enum oplus_cp_work_mode mode, int curr_ma)
{
	int i;
	int rc;
	bool work_start;
	int open_flag, open_num, close_flag;
	struct oplus_chg_ic_dev *ic_dev;
	struct oplus_cp_obc_strategy *obc = (struct oplus_cp_obc_strategy *)strategy;

	open_num = 0;
	for (i = 0; i < obc->data_num; i++) {
		if (curr_ma >= obc->open_thr_ma[i])
			open_num++;
	}
	open_flag = 0;
	for (i = 0; i < strategy->cp->child_num; i++) {
		ic_dev = strategy->cp->child_list[i].ic_dev;
		if (ic_dev == NULL)
			continue;
		if (!ic_dev->online)
			continue;
		rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT, mode);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] check work mode support error, rc=%d\n", i, rc);
			continue;
		}
		if ((rc == -ENOTSUPP) || (rc == 0))
			continue;
		if (open_num > 0) {
			open_flag |= BIT(i);
			open_num--;
			continue;
		}
		rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &work_start);
		if (rc < 0) {
			chg_err("child ic[%d] can't get cp work status, rc=%d\n", i, rc);
			continue;
		}
		if (!work_start)
			continue;
		open_flag |= BIT(i);
	}

	open_num = obc->data_num;
	for (i = obc->data_num - 1; i >= 0; i--) {
		chg_debug("curr_ma: %d, close_thr_ma[%d]: %u\n",
			  curr_ma, i, obc->close_thr_ma[i]);
		if (curr_ma <= obc->close_thr_ma[i])
			open_num = i;
		else
			break;
	}
	chg_debug("open_num: %d\n", open_num);
	close_flag = 0;
	for (i = 0; i < strategy->cp->child_num; i++) {
		if (!(open_flag & BIT(i)))
			continue;
		if (open_num > 0) {
			close_flag |= BIT(i);
			open_num--;
			continue;
		} else {
			break;
		}
	}

	chg_debug("open_flag=0x%x, close_flag=0x%x, curr_ma=%d\n",
		  open_flag, close_flag, curr_ma);

	return (open_flag & close_flag);
}
/* CP_STRAT_OPEN_BY_CURR end */

static struct oplus_cp_strategy_desc g_strategy_desc[] = {
	{
		.type = CP_STRAT_OPEN_ALL,
		.strategy_alloc = open_all_strategy_alloc,
		.strategy_release = open_all_strategy_release,
		.strategy_init = open_all_strategy_init,
		.strategy_get_open_data = open_all_strategy_get_open_data,
		.strategy_get_data = NULL,
	}, {
		.type = CP_STRAT_OPEN_BY_CURR,
		.strategy_alloc = obc_strategy_alloc,
		.strategy_release = obc_strategy_release,
		.strategy_init = obc_strategy_init,
		.strategy_get_open_data = obc_strategy_get_open_data,
		.strategy_get_data = obc_strategy_get_data,
	}
};

static struct oplus_cp_strategy *oplus_vc_strategy_alloc(struct oplus_virtual_cp_ic *cp)
{
	struct device_node *node = cp->dev->of_node;
	struct device_node *strategy_node;
	enum oplus_cp_strategy_type type;
	struct oplus_cp_strategy_desc *desc = NULL;
	struct oplus_cp_strategy *strategy;
	bool strategy_node_find;
	int i;
	int rc;

	rc = of_property_read_u32(node, "oplus,cp_strategy", &type);
	if (rc < 0) {
		chg_err("can't get cp strategy type, rc=%d\n", rc);
		return NULL;
	}

	for (i = 0; i < ARRAY_SIZE(g_strategy_desc); i++) {
		desc = &g_strategy_desc[i];
		if (desc->type == type)
			break;
	}
	if (desc->type != type) {
		chg_err("strategy[%d] not found\n", type);
		return NULL;
	}

	strategy_node_find = false;
	for_each_child_of_node(node, strategy_node) {
		if (strcmp(strategy_node->name, "oplus,cp_strategy_data") == 0) {
			strategy_node_find = true;
			break;
		}
	}
	if (!strategy_node_find)
		strategy_node = NULL;

	if (desc->strategy_alloc == NULL) {
		chg_err("%s strategy strategy_alloc func is NULL\n",
			oplus_cp_strategy_type_str(desc->type));
		return NULL;
	}

	strategy = desc->strategy_alloc(cp, strategy_node);
	if (strategy == NULL)
		return NULL;
	strategy->cp = cp;
	strategy->desc = desc;

	return strategy;
}

static int oplus_vc_strategy_release(struct oplus_cp_strategy *strategy)
{
	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (strategy->desc->strategy_release == NULL)
		return -ENOTSUPP;

	return strategy->desc->strategy_release(strategy);
}

static int oplus_vc_strategy_init(struct oplus_cp_strategy *strategy)
{
	int rc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (strategy->desc->strategy_init == NULL)
		return -ENOTSUPP;

	rc = strategy->desc->strategy_init(strategy);
	if (rc < 0)
		return rc;
	strategy->initialized = true;

	return 0;
}

static int oplus_vc_strategy_get_open_data(
	struct oplus_cp_strategy *strategy, enum oplus_cp_work_mode mode)
{
	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (!strategy->initialized) {
		chg_err("%s strategy not init\n", oplus_cp_strategy_type_str(strategy->desc->type));
		return -EFAULT;
	}
	if (mode == CP_WORK_MODE_UNKNOWN) {
		chg_err("work mode is unknown\n");
		return -EFAULT;
	}
	if (strategy->desc->strategy_get_open_data == NULL)
		return -ENOTSUPP;

	return strategy->desc->strategy_get_open_data(strategy, mode);
}

static int oplus_vc_strategy_get_data(
	struct oplus_cp_strategy *strategy, enum oplus_cp_work_mode mode, int curr_ma)
{
	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (!strategy->initialized) {
		chg_err("%s strategy not init\n", oplus_cp_strategy_type_str(strategy->desc->type));
		return -EFAULT;
	}
	if (mode == CP_WORK_MODE_UNKNOWN) {
		chg_err("work mode is unknown\n");
		return -EFAULT;
	}
	if (strategy->desc->strategy_get_data == NULL)
		return -ENOTSUPP;

	return strategy->desc->strategy_get_data(strategy, mode, curr_ma);
}

static struct device_node *oplus_vc_find_ic_root_node(struct device_node *root, const char *name)
{
	const char *tmp;
	struct device_node *child;
	int rc;
	int i;

	rc = of_property_count_elems_of_size(root, "oplus,cp_ic", sizeof(u32));
	if (rc < 0) {
		chg_err("can't get cp ic number, rc=%d\n", rc);
		return NULL;
	}

	for (i = 0; i < rc; i++) {
		tmp = of_get_oplus_chg_ic_name(root, "oplus,cp_ic", i);
		if ((NULL != tmp) && (NULL != name) && (strcmp(tmp, name) == 0))
			return root;
	}

	for_each_child_of_node(root, child) {
		if (!of_property_read_bool(child, "oplus,cp_ic"))
			continue;
		rc = of_property_count_elems_of_size(child, "oplus,cp_ic", sizeof(u32));
		if (rc < 0) {
			chg_err("can't get cp ic number, rc=%d\n", rc);
			continue;
		}
		for (i = 0; i < rc; i++) {
			tmp = of_get_oplus_chg_ic_name(child, "oplus,cp_ic", i);
			if ((NULL != tmp) && (NULL != name) && (strcmp(tmp, name) == 0))
				return child;
		}
	}

	return NULL;
}

static void oplus_vc_online_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_cp_child *child = virq_data;
	schedule_work(&child->online_work);
}

static void oplus_vc_offline_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_cp_child *child = virq_data;
	schedule_work(&child->offline_work);
}

static void oplus_vc_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_cp_ic *chip = virq_data;

	oplus_chg_ic_copy_err_msg(chip->ic_dev, ic_dev);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
}

static int oplus_vc_base_virq_register(struct oplus_virtual_cp_ic *chip, int index)
{
	int rc = 0;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	void *virq_data = NULL;

	ic_dev = chip->child_list[index].ic_dev;
	virq_data = &chip->child_list[index];
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	chg_info(" index %d virq register %s start!\n", index, ic_dev->name);

	rc = oplus_chg_ic_virq_register(ic_dev,
			OPLUS_IC_VIRQ_ONLINE, oplus_vc_online_handler, virq_data);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(ic_dev,
			OPLUS_IC_VIRQ_OFFLINE, oplus_vc_offline_handler, virq_data);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(ic_dev,
		OPLUS_IC_VIRQ_ERR, oplus_vc_err_handler, chip);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);

	if (ic_dev->name)
		chg_info("%s virq register success\n", ic_dev->name);

	return 0;
}

static void oplus_vc_online_work(struct work_struct *work)
{
	struct oplus_virtual_cp_child *child =
		container_of(work, struct oplus_virtual_cp_child, online_work);
	struct oplus_virtual_cp_ic *chip;
	bool online = false;
	int i;

	chg_info("%s: flush offline_work\n", child->ic_dev->manu_name);
	flush_work(&child->offline_work);

	chg_info("%s online\n", child->ic_dev->manu_name);
	chip = oplus_chg_ic_get_drvdata(child->parent);

	for (i = 0; i < chip->child_num; i++) {
		if (chip->child_list[i].ic_dev == NULL)
			continue;
		if (!chip->child_list[i].ic_dev->online)
			continue;
		if (chip->main_cp != i)
			continue;
		online = true;
		break;
	}

	if (!child->parent->online && online) {
		oplus_chg_ic_func(child->parent, OPLUS_IC_FUNC_INIT);
	}
}

static void oplus_vc_offline_work(struct work_struct *work)
{
	struct oplus_virtual_cp_child *child =
		container_of(work, struct oplus_virtual_cp_child, offline_work);
	struct oplus_virtual_cp_ic *chip;
	bool online = true;
	int i;

	chg_info("%s: flush online_work\n", child->ic_dev->manu_name);
	flush_work(&child->online_work);

	chg_info("%s offline\n", child->ic_dev->manu_name);
	chip = oplus_chg_ic_get_drvdata(child->parent);

	for (i = 0; i < chip->child_num; i++) {
		if (chip->child_list[i].ic_dev == NULL)
			continue;
		if (!chip->child_list[i].ic_dev->online) {
			if (chip->main_cp == i)
				online = false;
			clear_bit(i, &chip->open_flag);
			continue;
		}
	}

	if (child->parent->online && !online) {
		cancel_delayed_work_sync(&chip->monitor_work);
		oplus_chg_ic_func(child->parent, OPLUS_IC_FUNC_EXIT);
	}
}

static void oplus_vc_child_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_virtual_cp_child *child;
	struct oplus_chg_ic_dev *parent;
	struct oplus_virtual_cp_ic *chip;
	struct device_node *node;
	int rc;
	int i;

	if (ic == NULL) {
		chg_err("ic is NULL\n");
		return;
	}
	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	child = data;
	parent = child->parent;
	chip = oplus_chg_ic_get_drvdata(parent);

	if (timeout) {
		chg_info("timeout");
		if (chip->active_node != NULL)
			return;

		if (chip->main_cp == child->index) {
			/* TODO: Add the main cp switching function after IC registration timeout */
		}
		return;
	}

	node = oplus_vc_find_ic_root_node(chip->dev->of_node, ic->name);
	WARN_ON(node == NULL);
	chip->active_node = node;

	for (i = 0; i < chip->child_num; i++) {
		if (&chip->child_list[i] != child)
			continue;
		rc = of_property_read_u32_index(
			node, "oplus,input_curr_max_ma", i,
			&child->max_curr_ma);
		if (rc < 0) {
			chg_err("can't read ic[%d] oplus,input_curr_max_ma, rc=%d\n", i, rc);
			return;
		}
	}

	child->ic_dev = ic;
	oplus_chg_ic_set_parent(ic, parent);

	rc = oplus_vc_base_virq_register(chip, child->index);
	if (rc < 0) {
		chg_err("%s virq register error, rc=%d\n", ic->name, rc);
		return;
	}

	chg_info("ic->name %s online = %d main_cp = %d, index = %d",
		  ic->name, parent->online, chip->main_cp, child->index);
	if (!parent->online && child->index == chip->main_cp) {
		rc = oplus_chg_ic_func(child->parent, OPLUS_IC_FUNC_INIT);
		if (rc < 0) {
			parent->online = false;
			chg_err("ic->name %s, main_cp = %d, index = %d rc = %d init failed, set online as false",
				 ic->name, chip->main_cp, child->index, rc);
		} else {
			chg_info("ic->name %s online = %d main_cp = %d, index = %d init success, set online.",
				  ic->name, parent->online, chip->main_cp, child->index);
			parent->online = true;
		}
	}
}

static int oplus_vc_child_init(struct oplus_virtual_cp_ic *chip)
{
	struct device_node *node = chip->dev->of_node;
	int i = 0;
	int rc = 0;
	const char *name;
	struct device_node *child;

	chip->active_node = NULL;
	rc = of_property_read_u32(node, "oplus,cp_ic_connect",
				  &chip->connect_type);
	if (rc < 0) {
		chg_err("can't get cp ic connect type, rc=%d\n", rc);
		return rc;
	}

	chip->main_cp = 0;
	rc = of_property_count_elems_of_size(node, "oplus,cp_ic",
					     sizeof(u32));
	if (rc < 0) {
		chg_err("can't get cp ic number, rc=%d\n", rc);
		return rc;
	}
	chip->child_num = rc;
	chip->child_list = devm_kzalloc(
		chip->dev,
		sizeof(struct oplus_virtual_cp_child) * chip->child_num,
		GFP_KERNEL);
	if (chip->child_list == NULL) {
		rc = -ENOMEM;
		chg_err("alloc child ic memory error\n");
		return rc;
	}

	for (i = 0; i < chip->child_num; i++) {
		rc = of_property_read_u32_index(node,
			"oplus,input_curr_max_ma", i,
			&chip->child_list[i].max_curr_ma);
		if (rc < 0) {
			chg_err("can't read ic[%d] oplus,input_curr_max_ma, rc=%d\n", i, rc);
			goto read_property_err;
		}
		chip->child_list[i].index = i;
		chip->child_list[i].parent = chip->ic_dev;
		INIT_WORK(&chip->child_list[i].online_work, oplus_vc_online_work);
		INIT_WORK(&chip->child_list[i].offline_work, oplus_vc_offline_work);
		name = of_get_oplus_chg_ic_name(node, "oplus,cp_ic", i);
		chg_info("name is %s, i = %d", name, i);
		rc = oplus_chg_ic_wait_ic_timeout(name, oplus_vc_child_reg_callback,
						  &chip->child_list[i],
						  msecs_to_jiffies(CP_REG_TIMEOUT_MS));
		if (rc < 0) {
			chg_err("can't wait ic[%d](%s), rc=%d\n", i, name, rc);
			goto read_property_err;
		}
	}

	for_each_child_of_node(node, child) {
		if (!of_property_read_bool(child, "oplus,cp_ic"))
			continue;
		for (i = 0; i < chip->child_num; i++) {
			name = of_get_oplus_chg_ic_name(child, "oplus,cp_ic", i);
			chg_info("name is %s, i = %d", name, i);
			rc = oplus_chg_ic_wait_ic_timeout(name, oplus_vc_child_reg_callback, &chip->child_list[i],
							  msecs_to_jiffies(CP_REG_TIMEOUT_MS));
			if (rc < 0) {
				chg_err("can't wait ic[%d](%s), rc=%d\n", i, name, rc);
				continue;
			}
		}
	}
	return 0;

read_property_err:
	for (; i >=0; i--)
		chip->child_list[i].ic_dev = NULL;
	devm_kfree(chip->dev, chip->child_list);
	return rc;
}

static int oplus_chg_vc_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_cp_ic *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	mutex_lock(&chip->online_lock);
	if (ic_dev->online) {
		mutex_unlock(&chip->online_lock);
		return 0;
	}
	chip->strategy = oplus_vc_strategy_alloc(chip);
	if (chip->strategy != NULL) {
		rc = oplus_vc_strategy_init(chip->strategy);
		if (rc < 0) {
			chg_err("cp strategy init error, rc=%d", rc);
			oplus_vc_strategy_release(chip->strategy);
			chip->strategy = NULL;
		}
	}
	chip->work_mode = CP_WORK_MODE_UNKNOWN;
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);
	mutex_unlock(&chip->online_lock);

	return 0;
}

static int oplus_chg_vc_exit(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_cp_ic *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!ic_dev->online)
		return 0;

	mutex_lock(&chip->online_lock);
	ic_dev->online = false;
	oplus_vc_strategy_release(chip->strategy);
	chip->strategy = NULL;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);
	chg_info("unregister success\n");
	mutex_unlock(&chip->online_lock);

	return 0;
}

static int oplus_chg_vc_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vc->child_num; i++) {
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev, OPLUS_IC_FUNC_REG_DUMP);
		if (rc < 0)
			chg_err("child ic[%d] reg dump error, rc=%d\n", i, rc);
	}

	return 0;
}

static int oplus_chg_vc_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	struct oplus_virtual_cp_ic *vc;
	int i, index;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	index = 0;
	for (i = 0; i < vc->child_num; i++) {
		if (index >= len)
			return len;
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_SMT_TEST, buf + index,
				       len - index);
		if (rc < 0) {
			if (rc != -ENOTSUPP) {
				chg_err("child ic[%d] smt test error, rc=%d\n",
					i, rc);
				rc = snprintf(buf + index, len - index,
					"[%s]-[%s]:%d\n",
					vc->child_list[i].ic_dev->manu_name,
					"FUNC_ERR", rc);
			} else {
				rc = 0;
			}
		} else {
			if ((rc > 0) && buf[index + rc - 1] != '\n') {
				buf[index + rc] = '\n';
				index++;
			}
		}
		index += rc;
	}

	return index;
}

static int oplus_chg_vc_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;
	bool enable;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	vc->cp_enable = en;
	/* When using a policy, the CP is enabled by the policy */
	if (en && vc->strategy)
		return 0;

	for (i = 0; i < vc->child_num; i++) {
		enable = en;
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev, OPLUS_IC_FUNC_CP_ENABLE, enable);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] %s error, rc=%d\n", i, enable ? "enable" : "disable", rc);
			/* TODO: need push ic error msg, and offline child ic */
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vc_wd_enable(struct oplus_chg_ic_dev *ic_dev, int timeout_ms)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vc->child_num; i++) {
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev, OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE, timeout_ms);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] set timeout to %dms error, rc=%d\n", i, timeout_ms, rc);
			/* TODO: need push ic error msg, and offline child ic */
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vc_hw_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vc->child_num; i++) {
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev, OPLUS_IC_FUNC_CP_HW_INTI);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] hw init error, rc=%d\n", i, rc);
			/* TODO: need push ic error msg, and offline child ic */
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vc_set_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	struct oplus_virtual_cp_ic *vc;
	int open_flag;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT, mode);
	if (rc < 0 && rc != -ENOTSUPP) {
		chg_err("check work mode support error, rc=%d\n", rc);
		return rc;
	}
	if ((rc == -ENOTSUPP) || (rc == 0)) {
		chg_err("cp[%s] not support %s mode\n", ic_dev->manu_name,
			oplus_cp_work_mode_str(mode));
		return -EINVAL;
	}

	if (vc->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
		vc->work_mode = mode;
		if (vc->strategy != NULL) {
			open_flag = oplus_vc_strategy_get_open_data(vc->strategy, vc->work_mode);
			if (open_flag < 0) {
				chg_err("cp strategy get open data error, rc=%d\n", open_flag);
				return open_flag;
			}
		}
		for (i = 0; i < vc->child_num; i++) {
			if (vc->strategy && !(open_flag & BIT(i)))
				continue;
			chg_info("%s: set work mode to %s",
				 vc->child_list[i].ic_dev->manu_name,
				 oplus_cp_work_mode_str(mode));
			rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
				OPLUS_IC_FUNC_CP_SET_WORK_MODE, mode);
			if (rc < 0 && rc != -ENOTSUPP) {
				chg_err("cp[%s] set work mode to %s error, rc=%d\n",
					vc->child_list[i].ic_dev->manu_name,
					oplus_cp_work_mode_str(mode), rc);
				return rc;
			}
		}
	} else if (vc->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
		/* TODO */
		return -ENOTSUPP;
	} else {
		chg_err("Unknown connect type\n");
		return -EINVAL;
	}

	return 0;
}

static int oplus_chg_vc_get_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode *mode)
{
	struct oplus_virtual_cp_ic *vc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	if (vc->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
		*mode = vc->work_mode;
	} else if (vc->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
		/* TODO */
		return -ENOTSUPP;
	} else {
		chg_err("Unknown connect type\n");
		return -EINVAL;
	}

	return 0;
}

static int oplus_chg_vc_check_work_mode_support(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;
	bool support = false;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vc->child_num; i++) {
		if (vc->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			if (vc->child_list[i].ic_dev == NULL)
				continue;
			if (!vc->child_list[i].ic_dev->online)
				continue;
			rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
				OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT, mode);
			if (rc < 0 && rc != -ENOTSUPP) {
				chg_err("child ic[%d] check work mode support error, rc=%d\n", i, rc);
				return rc;
			}
			if (rc == 0) {
				return 0;
			} else if (rc > 0) {
				support = true;
				err = rc;
			}
		} else if (vc->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			/* TODO */
			chg_err("child ic[%d] check work mode not support because it is serial connect.\n", i);
			return -ENOTSUPP;
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return err < 0 ? err : support;
}

static int oplus_chg_vc_set_iin_no_strategy(struct oplus_virtual_cp_ic *vc, int iin)
{
	bool work_start;
	int curr_sum;
	int curr_min;
	int start_num;
	int i;
	int rc;

	curr_min = vc->child_list[vc->main_cp].max_curr_ma;
	start_num = 1;

	for (i = 0; i < vc->child_num; i++) {
		if (i == vc->main_cp)
			continue;
		if (vc->child_list[i].ic_dev == NULL) {
			chg_err("vc->child_list[%d].ic_dev is NULL\n", i);
			continue;
		}
		if (!vc->child_list[i].ic_dev->online) {
			chg_err("%s offline\n", vc->child_list[i].ic_dev->name);
			continue;
		}
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &work_start);
		if (rc < 0) {
			chg_err("can't get cp[%d] work status, rc=%d\n", i, rc);
			continue;
		}
		if (work_start) {
			if (vc->child_list[i].max_curr_ma < curr_min)
				curr_min = vc->child_list[i].max_curr_ma;
			start_num++;
		}
	}

	curr_sum = curr_min * start_num;
	if (curr_sum >= iin) {
		vc->target_iin = iin;
		return 0;
	}

	for (i = 0; i < vc->child_num; i++) {
		if (i == vc->main_cp)
			continue;
		if (vc->child_list[i].ic_dev == NULL) {
			chg_err("vc->child_list[%d].ic_dev is NULL\n", i);
			continue;
		}
		if (!vc->child_list[i].ic_dev->online) {
			chg_err("%s offline\n", vc->child_list[i].ic_dev->name);
			continue;
		}
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &work_start);
		if (rc < 0) {
			chg_err("can't get cp[%d] work status, rc=%d\n", i, rc);
			continue;
		}
		if (work_start)
			continue;

		oplus_chg_ic_func(vc->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, true);

		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_SET_WORK_START, true);

		if (rc < 0) {
			chg_err("can't set cp[%d] work start, rc=%d\n", i, rc);
			continue;
		}
		if (vc->child_list[i].max_curr_ma < curr_min)
			curr_min = vc->child_list[i].max_curr_ma;
		start_num++;
		curr_sum = curr_min * start_num;
		if (curr_sum >= iin) {
			vc->target_iin = iin;
			return 0;
		}
	}

	chg_err("input current is too large, iin=%d\n", iin);

	return -EINVAL;
}

static int oplus_chg_vc_open_step(struct oplus_virtual_cp_ic *vc, struct oplus_chg_ic_dev *ic_dev)
{
	int rc;

	if (ic_dev == NULL)
		return -ENODEV;

	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, true);
	if (rc < 0) {
		chg_err("can't enable cp[%s] adc, rc=%d\n", ic_dev->manu_name, rc);
		return rc;
	}

	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_SET_WORK_MODE, vc->work_mode);
	if (rc < 0) {
		chg_err("can't set cp[%s] work mode to %s, rc=%d\n", ic_dev->manu_name,
			oplus_cp_work_mode_str(vc->work_mode), rc);
		goto work_mode_err;
	}

	if (vc->cp_enable) {
		rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_ENABLE, true);
		if (rc < 0) {
			chg_err("can't enable cp[%s], rc=%d\n", ic_dev->manu_name, rc);
			goto work_mode_err;
		}
	}

	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_SET_WORK_START, true);
	if (rc < 0) {
		chg_err("can't set cp[%s] work start, rc=%d\n", ic_dev->manu_name, rc);
		goto work_start_err;
	}

	return 0;

work_start_err:
	oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_ENABLE, false);
work_mode_err:
	oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, false);
	return rc;
}

static void oplus_chg_vc_close_step(struct oplus_virtual_cp_ic *vc, struct oplus_chg_ic_dev *ic_dev)
{
	int rc;

	if (ic_dev == NULL)
		return;

	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_SET_WORK_START, false);
	if (rc < 0)
		chg_err("can't set cp[%s] work stop, rc=%d\n", ic_dev->manu_name, rc);
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_ENABLE, false);
	if (rc < 0)
		chg_err("can't disable cp[%s], rc=%d\n", ic_dev->manu_name, rc);
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, false);
	if (rc < 0)
		chg_err("can't disable cp[%s] adc, rc=%d\n", ic_dev->manu_name, rc);
}

static int oplus_chg_vc_set_iin_strategy(struct oplus_virtual_cp_ic *vc, int iin)
{
	vc->target_iin = iin;

	return 0;
}

static int oplus_chg_vc_set_iin(struct oplus_chg_ic_dev *ic_dev, int iin)
{
	struct oplus_virtual_cp_ic *vc;
	bool work_start;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	if (vc->connect_type != OPLUS_CHG_IC_CONNECT_PARALLEL)
		return -ENOTSUPP;

	if (vc->target_iin == iin)
		return 0;

	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &work_start);
	if (rc < 0) {
		chg_err("can't get cp work status, rc=%d\n", rc);
		return rc;
	}
	if (!work_start) {
		vc->target_iin = 0;
		return 0;
	}

	if (vc->strategy == NULL)
		return oplus_chg_vc_set_iin_no_strategy(vc, iin);
	else
		return oplus_chg_vc_set_iin_strategy(vc, iin);
}

static int oplus_chg_vc_get_vin(struct oplus_chg_ic_dev *ic_dev, int *vin)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vc->child_num; i++) {
		if (vc->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
				OPLUS_IC_FUNC_CP_GET_VIN, vin);
			if (rc < 0 && rc != -ENOTSUPP) {
				chg_err("child ic[%d] get vin error, rc=%d\n", i, rc);
				err = rc;
			} else if (rc >= 0) {
				return 0;
			}
		} else if (vc->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			/* TODO */
			return -ENOTSUPP;
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return err;
}

static int oplus_chg_vc_get_iin(struct oplus_chg_ic_dev *ic_dev, int *iin)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;
	int curr;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	*iin = 0;
	for (i = 0; i < vc->child_num; i++) {
		if (vc->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
				OPLUS_IC_FUNC_CP_GET_IIN, &curr);
			if (rc < 0 && rc != -ENOTSUPP) {
				chg_err("child ic[%d] get iin error, rc=%d\n", i, rc);
				err = rc;
			} else if (rc >= 0) {
				*iin += curr;
				if (err == -ENOTSUPP)
					err = rc;
			}
		} else if (vc->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			/* TODO */
			return -ENOTSUPP;
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return err;
}

static int oplus_chg_vc_get_vout(struct oplus_chg_ic_dev *ic_dev, int *vout)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vc->child_num; i++) {
		if (vc->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
				OPLUS_IC_FUNC_CP_GET_VOUT, vout);
			if (rc < 0 && rc != -ENOTSUPP) {
				chg_err("child ic[%d] get vout error, rc=%d\n", i, rc);
				err = rc;
			} else if (rc >= 0) {
				return 0;
			}
		} else if (vc->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			/* TODO */
			return -ENOTSUPP;
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return err;
}

static int oplus_chg_vc_get_iout(struct oplus_chg_ic_dev *ic_dev, int *iout)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;
	int curr;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	*iout = 0;
	for (i = 0; i < vc->child_num; i++) {
		if (vc->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
			rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
				OPLUS_IC_FUNC_CP_GET_IOUT, &curr);
			if (rc < 0 && rc != -ENOTSUPP) {
				chg_err("child ic[%d] get iout error, rc=%d\n", i, rc);
				err = rc;
			} else if (rc >= 0) {
				*iout += curr;
				if (err == -ENOTSUPP)
					err = rc;
			}
		} else if (vc->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
			/* TODO */
			return -ENOTSUPP;
		} else {
			chg_err("Unknown connect type\n");
			return -EINVAL;
		}
	}

	return err;
}

static int oplus_chg_vc_get_vac(struct oplus_chg_ic_dev *ic_dev, int *vac)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc;
	int err = -ENOTSUPP;
	int vol;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vc->child_num; i++) {
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_GET_VAC, &vol);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] get iout error, rc=%d\n", i, rc);
			err = rc;
		} else if (rc >= 0) {
			*vac = vol;
			return 0;
		}
	}

	return err;
}

static int oplus_chg_vc_set_work_start_strategy(struct oplus_virtual_cp_ic *vc)
{
	int open_flag;
	int i;
	int rc;

	open_flag = oplus_vc_strategy_get_open_data(vc->strategy, vc->work_mode);
	if (open_flag < 0) {
		chg_err("cp strategy get open data error, rc=%d\n", open_flag);
		return open_flag;
	}
	for (i = 0; i < vc->child_num; i++) {
		vc->child_list[i].sw_ocp_count = 0;
		if (vc->child_list[i].ic_dev == NULL)
			continue;
		if (!(open_flag & BIT(i)))
			continue;
		rc = oplus_chg_vc_open_step(vc, vc->child_list[i].ic_dev);
		if (rc < 0) {
			chg_err("cp[%s] open error, rc=%d\n",
				vc->child_list[i].ic_dev->manu_name, rc);
			goto err;
		}
		set_bit(i, &vc->open_flag);
		open_flag &= ~BIT(i);
	}

	return 0;
err:
	for (; i >= 0; i--) {
		clear_bit(i, &vc->open_flag);
		oplus_chg_vc_close_step(vc, vc->child_list[i].ic_dev);
	}
	return rc;
}

/* Returns true if the CP is offline */
static bool oplus_chg_vc_check_offline_cp(struct oplus_virtual_cp_ic *chip, int index)
{
	if (chip->child_list[index].ic_dev == NULL)
		return true;
	if (chip->child_list[index].ic_dev->online)
		return false;
	/*
	 * When the IC's software protection is triggered,
	 * we will try to restore it after charging is completed.
	 */
	oplus_chg_ic_func(chip->child_list[index].ic_dev, OPLUS_IC_FUNC_INIT);

	return true;
}

static int oplus_chg_vc_set_work_start(struct oplus_chg_ic_dev *ic_dev, bool start)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online) {
		if (start) {
			chg_err("%s: offline, can't start\n", ic_dev->manu_name);
			return -EFAULT;
		} else {
			chg_err("%s: offline, try restore\n", ic_dev->manu_name);
		}
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	vc->target_iin = 0;
	vc->work_status_change_count = 0;
	vc->open_flag = 0;
	vc->pre_open_flag = 0;
	vc->open_flag_change_count = 0;

	if (vc->strategy == NULL && ic_dev->online) {
		mutex_lock(&vc->online_lock);
		vc->strategy = oplus_vc_strategy_alloc(vc);
		if (vc->strategy != NULL) {
			rc = oplus_vc_strategy_init(vc->strategy);
			if (rc < 0) {
				chg_err("cp strategy init error, rc=%d", rc);
				oplus_vc_strategy_release(vc->strategy);
				vc->strategy = NULL;
			}
		}
		mutex_unlock(&vc->online_lock);
	}

	if (vc->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
		if (start) {
			if (vc->strategy) {
				chg_info("%s: use %s strategy\n", ic_dev->manu_name,
					 oplus_cp_strategy_type_str(vc->strategy->desc->type));
				cancel_delayed_work_sync(&vc->monitor_work);
				rc = oplus_chg_vc_set_work_start_strategy(vc);
				/*
				 * Do not start immediately, there may
				 * be a delay in hardware status updates
				 */
				schedule_delayed_work(&vc->monitor_work,
					msecs_to_jiffies(MONITOR_WORK_DELAY_MS));
			} else {
				chg_info("%s: no strategy\n", ic_dev->manu_name);
				/* Only the main CP is allowed to be turned on here */
				if (vc->main_cp < 0 || vc->main_cp >= vc->child_num)
					return -EINVAL;
				rc = oplus_chg_ic_func(vc->child_list[vc->main_cp].ic_dev,
						OPLUS_IC_FUNC_CP_SET_WORK_START, true);
				if (rc < 0 && rc != -ENOTSUPP)
					chg_err("main cp set start work error, rc=%d\n", rc);
			}
			return rc;
		} else {
			cancel_delayed_work_sync(&vc->monitor_work);
			for (i = vc->child_num - 1; i >= 0; i--) {
				/* If the CP is already offline, there is no need to shut it close again. */
				if (oplus_chg_vc_check_offline_cp(vc, i))
					continue;
				if (vc->strategy == NULL) {
					rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
						OPLUS_IC_FUNC_CP_SET_WORK_START, false);
				} else {
					clear_bit(i, &vc->open_flag);
					oplus_chg_vc_close_step(vc, vc->child_list[i].ic_dev);
					rc = 0;
				}
				if (rc < 0 && rc != -ENOTSUPP) {
					chg_err("cp[%s] set stop work error, rc=%d\n",
						vc->child_list[i].ic_dev->manu_name, rc);
					err = rc;
				} else if (rc >= 0) {
					err = rc;
				}
			}
		}
	} else if (vc->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
		/* TODO */
		return -ENOTSUPP;
	} else {
		chg_err("Unknown connect type\n");
		return -EINVAL;
	}

	return err;
}

static int oplus_chg_vc_set_ucp_disable(struct oplus_chg_ic_dev *ic_dev, bool disable)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);

	/* TODO */
	for (i = 0; i < vc->child_num; i++) {
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_SET_UCP_DISABLE, disable);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] %s error, rc=%d\n", i, disable ? "disable" : "enable", rc);
			err = rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vc_set_sstimeout_ucp_enable(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	struct oplus_virtual_cp_ic *vc;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);

	if (vc->main_cp < 0 || vc->main_cp >= vc->child_num)
		return -EINVAL;
	rc = oplus_chg_ic_func(vc->child_list[vc->main_cp].ic_dev,
			OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE, enable);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_err("main cp set sstimeout ucp err, enable = %d, rc=%d\n", enable, rc);
	return rc;
}

static int oplus_chg_vc_get_work_status(struct oplus_chg_ic_dev *ic_dev, bool *start)
{
	struct oplus_virtual_cp_ic *vc;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	if (vc->connect_type == OPLUS_CHG_IC_CONNECT_PARALLEL) {
		/* After the main CP is turned on, it is considered to be turned on */
		if (vc->main_cp < 0 || vc->main_cp >= vc->child_num)
			return -EINVAL;
		rc = oplus_chg_ic_func(vc->child_list[vc->main_cp].ic_dev,
				OPLUS_IC_FUNC_CP_GET_WORK_STATUS, start);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("main cp get work status error, rc=%d\n", rc);
		return rc;
	} else if (vc->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL) {
		/* TODO */
		return -ENOTSUPP;
	} else {
		chg_err("Unknown connect type\n");
		return -EINVAL;
	}

	return err;
}

static int oplus_chg_vc_adc_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	/* When using a policy, the CP adc is enabled by the policy */
	if (en && vc->strategy)
		return 0;

	for (i = 0; i < vc->child_num; i++) {
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, en);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] %s error, rc=%d\n", i, en ? "enable" : "disable", rc);
			err = rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vc_get_cp_temp(struct oplus_chg_ic_dev *ic_dev,
					  int *cp_temp)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int temp = 0;
	int temp_max = 0;
	int rc = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vc->child_num; i++) {
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
				OPLUS_IC_FUNC_CP_GET_TEMP, &temp);
		if (rc < 0) {
			if (rc != -ENOTSUPP)
				chg_err("child ic[%d] can't get cp btb temp, rc=%d\n", i, rc);
			continue;
		}
		temp_max = temp_max > temp ? temp_max : temp;
	}

	*cp_temp = temp_max;

	return rc;
}

static int oplus_chg_vc_get_iin_max(struct oplus_chg_ic_dev *ic_dev,
				    enum oplus_cp_work_mode mode, int *iin_max)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int curr_ma = 0;
	int rc = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vc->child_num; i++) {
		if (vc->child_list[i].ic_dev == NULL)
			continue;
		if (!vc->child_list[i].ic_dev->online)
			continue;
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
				OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT, mode);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("cp[%s] check work mode support error, rc=%d\n",
				vc->child_list[i].ic_dev->manu_name, rc);
			continue;
		}
		if ((rc == 0) || (rc == -ENOTSUPP))
			continue;
		curr_ma += vc->child_list[i].max_curr_ma;
	}

	*iin_max = curr_ma;

	return 0;
}

static int oplus_chg_vc_watchdog_reset(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_cp_ic *vc;
	int i;
	int rc = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vc->child_num; i++) {
		if (vc->child_list[i].ic_dev == NULL)
			continue;
		rc = oplus_chg_ic_func(vc->child_list[i].ic_dev,
				OPLUS_IC_FUNC_CP_WATCHDOG_RESET);
		if (rc < 0 && rc != -ENOTSUPP)
			chg_err("cp[%s] watchdog reset error, rc=%d\n",
				vc->child_list[i].ic_dev->manu_name, rc);
	}

	return rc;
}

static void oplus_vc_work_status_monitor(struct oplus_virtual_cp_ic *vc)
{
	int i;
	unsigned long open_flag = 0;
	bool work_start;
	struct oplus_chg_ic_dev *ic_dev;
	int rc;

#define OPEN_FLAG_CHANGE_MAX	3

	chg_debug("open_flag: 0x%lx, pre_open_flag:0x%lx\n",
		vc->open_flag, vc->pre_open_flag);
	for (i = 0; i < vc->child_num; i++) {
		ic_dev = vc->child_list[i].ic_dev;
		if (ic_dev == NULL)
			continue;
		if (!ic_dev->online)
			continue;
		rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &work_start);
		if (rc < 0) {
			chg_err("child ic[%d] can't get cp work status, rc=%d\n", i, rc);
			continue;
		}
		if (work_start) {
			open_flag |= BIT(i);
			set_bit(i, &vc->pre_open_flag);
			continue;
		}
		if (!test_bit(i, &vc->open_flag)) {
			clear_bit(i, &vc->pre_open_flag);
			continue;
		}
		if (vc->open_flag_change_count < OPEN_FLAG_CHANGE_MAX &&
		    !test_bit(i, &vc->pre_open_flag)) {
			vc->open_flag_change_count++;
			continue;
		}
		set_bit(i, &vc->pre_open_flag);
		vc->open_flag_change_count = 0;
		chg_err("[%s]: abnormal close\n", ic_dev->manu_name);
		oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_EXIT);
	}

	if (vc->open_flag == open_flag)
		return;
	if (vc->open_flag_change_count == 0)
		return;
	chg_err("expected open_flag: 0x%lx, practical open_flag: 0x%lx\n",
		vc->open_flag, open_flag);
}

static void oplus_vc_strategy_monitor(struct oplus_virtual_cp_ic *vc)
{
	int input_curr_ma;
	int open_flag, target_open_flag;
	int i;
	bool changed = false;
	bool work_start;
	struct oplus_chg_ic_dev *ic_dev;
	int rc;

#define CP_WORK_STATUS_CHANGE_COUNT_THR		3
#define CP_CURRENT_ABNORMAL_REPORT_THR_MA	500

	rc = oplus_chg_ic_func(vc->ic_dev, OPLUS_IC_FUNC_CP_GET_IIN, &input_curr_ma);
	if (rc < 0 && rc != -ENOTSUPP) {
		chg_err("can't get cp input current, rc=%d\n", rc);
		return;
	}
	if (rc == -ENOTSUPP)
		return;

	open_flag = oplus_vc_strategy_get_data(vc->strategy, vc->work_mode, input_curr_ma);
	if (open_flag == -ENOTSUPP) {
		return;
	} else if (open_flag < 0) {
		chg_err("cp strategy get data error, rc=%d\n", open_flag);
		return;
	}
	target_open_flag = oplus_vc_strategy_get_data(vc->strategy, vc->work_mode, vc->target_iin);
	if (target_open_flag == -ENOTSUPP) {
		return;
	} else if (target_open_flag < 0) {
		chg_err("cp strategy get data error, rc=%d\n", target_open_flag);
		return;
	}
	chg_debug("input_curr_ma:%d, open_flag=0x%x, target_open_flag=0x%x\n",
		  input_curr_ma, open_flag, target_open_flag);
	for (i = 0; i < vc->child_num; i++) {
		ic_dev = vc->child_list[i].ic_dev;
		if (ic_dev == NULL)
			continue;
		if (!ic_dev->online)
			continue;
		rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &work_start);
		if (rc < 0) {
			chg_err("child ic[%d] can't get cp work status, rc=%d\n", i, rc);
			continue;
		}
		if (work_start == (!!(open_flag & BIT(i))))
			continue;
		changed = true;
		break;
	}
	chg_debug("changed:%d, work_status_change_count:%d\n", changed, vc->work_status_change_count);
	if (!changed) {
		vc->work_status_change_count = 0;
		return;
	}
	if ((open_flag != target_open_flag) &&
	    (vc->work_status_change_count < CP_WORK_STATUS_CHANGE_COUNT_THR)) {
		vc->work_status_change_count++;
		return;
	}

	for (i = 0; i < vc->child_num; i++) {
		ic_dev = vc->child_list[i].ic_dev;
		if (ic_dev == NULL)
			continue;
		if (!ic_dev->online)
			continue;
		rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &work_start);
		if (rc < 0) {
			chg_err("child ic[%d] can't get cp work status, rc=%d\n", i, rc);
			continue;
		}
		if (work_start == (!!(open_flag & BIT(i))))
			continue;
		if (open_flag & BIT(i)) {
			rc = oplus_chg_vc_open_step(vc, vc->child_list[i].ic_dev);
			if (rc < 0) {
				chg_err("cp[%d] open step error, rc=%d\n", i, rc);
				continue;
			}
			set_bit(i, &vc->open_flag);
		} else {
			clear_bit(i, &vc->open_flag);
			oplus_chg_vc_close_step(vc, vc->child_list[i].ic_dev);
		}
	}
	vc->work_status_change_count = 0;
}

static void oplus_vc_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_virtual_cp_ic *vc =
		container_of(dwork, struct oplus_virtual_cp_ic, monitor_work);
	struct oplus_chg_ic_dev *ic_dev = vc->ic_dev;

	if (ic_dev == NULL || !ic_dev->online) {
		chg_err("vc is offline\n");
		return;
	}

	if (vc->open_flag == 0) {
		chg_info("all CPs are closed, exit\n");
		return;
	}

	oplus_vc_work_status_monitor(vc);
	oplus_vc_strategy_monitor(vc);

	schedule_delayed_work(&vc->monitor_work, msecs_to_jiffies(MONITOR_WORK_DELAY_MS));
}

static void *oplus_chg_vc_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT) &&
	    (func_id != OPLUS_IC_FUNC_CP_SET_WORK_START)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}
	if (!oplus_chg_ic_func_is_support(ic_dev, func_id)) {
		chg_info("%s: this func(=%d) is not supported\n",  ic_dev->name, func_id);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_vc_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_vc_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, oplus_chg_vc_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, oplus_chg_vc_smt_test);
		break;
	case OPLUS_IC_FUNC_CP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_ENABLE, oplus_chg_vc_enable);
		break;
	case OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE, oplus_chg_vc_wd_enable);
		break;
	case OPLUS_IC_FUNC_CP_HW_INTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_HW_INTI, oplus_chg_vc_hw_init);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_MODE, oplus_chg_vc_set_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_MODE, oplus_chg_vc_get_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT,
					       oplus_chg_vc_check_work_mode_support);
		break;
	case OPLUS_IC_FUNC_CP_SET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_IIN, oplus_chg_vc_set_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VIN, oplus_chg_vc_get_vin);
		break;
	case OPLUS_IC_FUNC_CP_GET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IIN, oplus_chg_vc_get_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VOUT, oplus_chg_vc_get_vout);
		break;
	case OPLUS_IC_FUNC_CP_GET_IOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IOUT, oplus_chg_vc_get_iout);
		break;
	case OPLUS_IC_FUNC_CP_GET_VAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VAC, oplus_chg_vc_get_vac);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START, oplus_chg_vc_set_work_start);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_STATUS, oplus_chg_vc_get_work_status);
		break;
	case OPLUS_IC_FUNC_CP_SET_ADC_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, oplus_chg_vc_adc_enable);
		break;
	case OPLUS_IC_FUNC_CP_GET_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_TEMP, oplus_chg_vc_get_cp_temp);
		break;
	case OPLUS_IC_FUNC_CP_SET_UCP_DISABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_UCP_DISABLE, oplus_chg_vc_set_ucp_disable);
		break;
	case OPLUS_IC_FUNC_CP_GET_IIN_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IIN_MAX, oplus_chg_vc_get_iin_max);
		break;
	case OPLUS_IC_FUNC_CP_WATCHDOG_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_WATCHDOG_RESET, oplus_chg_vc_watchdog_reset);
		break;
	case OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE,
			oplus_chg_vc_set_sstimeout_ucp_enable);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_vc_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static ssize_t oplus_vc_iin_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_virtual_cp_ic *chip = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE];
	int curr;
	int i;
	int len = 0;
	int rc;

	for (i = 0; i < chip->child_num; i++) {
		if (i > 0)
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, ",");
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_GET_IIN, &curr);
		if (rc == -ENOTSUPP) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "NULL");
			continue;
		} else if (rc < 0) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "Error(%d)", rc);
			continue;
		}
		len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "%d", curr);
	}
	len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "\n");

	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_vc_iin_ops =
{
	.read = oplus_vc_iin_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_vc_iin_ops =
{
	.proc_read  = oplus_vc_iin_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_vc_vin_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_virtual_cp_ic *chip = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE];
	int vol;
	int i;
	int len = 0;
	int rc;

	for (i = 0; i < chip->child_num; i++) {
		if (i > 0)
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, ",");
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_GET_VIN, &vol);
		if (rc == -ENOTSUPP) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "NULL");
			continue;
		} else if (rc < 0) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "Error(%d)", rc);
			continue;
		}
		len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "%d", vol);
	}
	len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "\n");

	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_vc_vin_ops =
{
	.read = oplus_vc_vin_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_vc_vin_ops =
{
	.proc_read  = oplus_vc_vin_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_vc_iout_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_virtual_cp_ic *chip = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE];
	int curr;
	int i;
	int len = 0;
	int rc;

	for (i = 0; i < chip->child_num; i++) {
		if (i > 0)
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, ",");
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_GET_IOUT, &curr);
		if (rc == -ENOTSUPP) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "NULL");
			continue;
		} else if (rc < 0) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "Error(%d)", rc);
			continue;
		}
		len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "%d", curr);
	}
	len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "\n");

	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_vc_iout_ops =
{
	.read = oplus_vc_iout_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_vc_iout_ops =
{
	.proc_read  = oplus_vc_iout_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_vc_vout_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_virtual_cp_ic *chip = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE];
	int vol;
	int i;
	int len = 0;
	int rc;

	for (i = 0; i < chip->child_num; i++) {
		if (i > 0)
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, ",");
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_GET_VOUT, &vol);
		if (rc == -ENOTSUPP) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "NULL");
			continue;
		} else if (rc < 0) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "Error(%d)", rc);
			continue;
		}
		len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "%d", vol);
	}
	len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "\n");

	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_vc_vout_ops =
{
	.read = oplus_vc_vout_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_vc_vout_ops =
{
	.proc_read  = oplus_vc_vout_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_vc_number_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_virtual_cp_ic *chip = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE];
	int len;

	len = snprintf(buf, ARRAY_SIZE(buf), "%d\n", chip->child_num);
	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_vc_number_ops =
{
	.read = oplus_vc_number_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_vc_number_ops =
{
	.proc_read  = oplus_vc_number_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_vc_work_mode_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_virtual_cp_ic *chip = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE];
	enum oplus_cp_work_mode mode;
	int i;
	int len = 0;
	int rc;

	for (i = 0; i < chip->child_num; i++) {
		if (i > 0)
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, ",");
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_GET_WORK_MODE, &mode);
		if (rc == -ENOTSUPP) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "Unknown");
			continue;
		} else if (rc < 0) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "Error(%d)", rc);
			continue;
		}
		switch (mode) {
		case CP_WORK_MODE_AUTO:
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "Auto");
			break;
		case CP_WORK_MODE_BYPASS:
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "1:1");
			break;
		case CP_WORK_MODE_2_TO_1:
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "2:1");
			break;
		case CP_WORK_MODE_3_TO_1:
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "3:1");
			break;
		case CP_WORK_MODE_4_TO_1:
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "4:1");
			break;
		case CP_WORK_MODE_1_TO_2:
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "1:2");
			break;
		case CP_WORK_MODE_1_TO_3:
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "1:3");
			break;
		case CP_WORK_MODE_1_TO_4:
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "1:4");
			break;
		default:
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "Unknown");
			break;
		}
	}
	len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "\n");

	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_vc_work_mode_ops =
{
	.read = oplus_vc_work_mode_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_vc_work_mode_ops =
{
	.proc_read  = oplus_vc_work_mode_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_vc_support_mode_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	return -ENOTSUPP;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_vc_support_mode_ops =
{
	.read = oplus_vc_support_mode_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_vc_support_mode_ops =
{
	.proc_read  = oplus_vc_support_mode_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_vc_work_status_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_virtual_cp_ic *chip = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE];
	bool work_start;
	int i;
	int len = 0;
	int rc;

	for (i = 0; i < chip->child_num; i++) {
		if (i > 0)
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, ",");
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev,
			OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &work_start);
		if (rc == -ENOTSUPP) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "Unknown");
			continue;
		} else if (rc < 0) {
			len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "Error(%d)", rc);
			continue;
		}
		len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "%s", work_start ? "Open" : "Close");
	}
	len += snprintf(buf + len, ARRAY_SIZE(buf) - len, "\n");

	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_vc_work_status_ops =
{
	.read = oplus_vc_work_status_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_vc_work_status_ops =
{
	.proc_read  = oplus_vc_work_status_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_vc_connect_type_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_virtual_cp_ic *chip = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE];
	int len;

	switch(chip->connect_type) {
	case OPLUS_CHG_IC_CONNECT_PARALLEL:
		len = snprintf(buf, ARRAY_SIZE(buf), "Parallel\n");
		break;
	case OPLUS_CHG_IC_CONNECT_SERIAL:
		len = snprintf(buf, ARRAY_SIZE(buf), "Serial\n");
		break;
	default:
		len = snprintf(buf, ARRAY_SIZE(buf), "Unknown\n");
		break;
	}

	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_vc_connect_type_ops =
{
	.read = oplus_vc_connect_type_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_vc_connect_type_ops =
{
	.proc_read  = oplus_vc_connect_type_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static struct proc_dir_entry *charger_dir = NULL;
#endif
static int oplus_virtual_cp_proc_init(struct oplus_virtual_cp_ic *chip)
{
	struct proc_dir_entry *cp_entry;
	struct proc_dir_entry *pr_entry_tmp;
	char name_buf[CP_NAME_BUF_MAX] = { 0 };

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	charger_dir = proc_mkdir("charger_cp", NULL);
	if (!charger_dir) {
		chg_err("Couldn't create charger proc entry\n");
		return -EFAULT;
	}

	snprintf(name_buf, CP_NAME_BUF_MAX - 1, "cp_%d", chip->ic_dev->index);
	chg_err("oplus_virtual_cp_proc_init index cp_%d\n", chip->ic_dev->index);
	cp_entry = proc_mkdir(name_buf, charger_dir);
#else
	snprintf(name_buf, CP_NAME_BUF_MAX - 1, "charger/cp:%d", chip->ic_dev->index);
	cp_entry = proc_mkdir(name_buf, NULL);
#endif
	if (cp_entry == NULL) {
		chg_err("Couldn't create charger/cp proc entry\n");
		return -EFAULT;
	}

	pr_entry_tmp = proc_create_data("iin", 0644, cp_entry, &oplus_vc_iin_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create iin proc entry\n");
	pr_entry_tmp = proc_create_data("vin", 0644, cp_entry, &oplus_vc_vin_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create vin proc entry\n");
	pr_entry_tmp = proc_create_data("iout", 0644, cp_entry, &oplus_vc_iout_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create iout proc entry\n");
	pr_entry_tmp = proc_create_data("vout", 0644, cp_entry, &oplus_vc_vout_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create vout proc entry\n");
	pr_entry_tmp = proc_create_data("number", 0644, cp_entry, &oplus_vc_number_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create number proc entry\n");
	pr_entry_tmp = proc_create_data("work_mode", 0644, cp_entry, &oplus_vc_work_mode_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create work_mode proc entry\n");
	pr_entry_tmp = proc_create_data("support_mode", 0644, cp_entry, &oplus_vc_support_mode_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create support_mode proc entry\n");
	pr_entry_tmp = proc_create_data("work_status", 0644, cp_entry, &oplus_vc_work_status_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create work_status proc entry\n");
	pr_entry_tmp = proc_create_data("connect_type", 0644, cp_entry, &oplus_vc_connect_type_ops, chip);
	if (pr_entry_tmp == NULL)
		chg_err("Couldn't create connect_type proc entry\n");

	return 0;
}

static int oplus_virtual_cp_probe(struct platform_device *pdev)
{
	struct oplus_virtual_cp_ic *chip;
	struct device_node *node = pdev->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	char name_buf[CP_NAME_BUF_MAX] = { 0 };
	static int retry_count = 0;
	int ic_index;
	int rc;

#define PROBE_RETRY_MAX	300

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_virtual_cp_ic),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	mutex_init(&chip->online_lock);
	INIT_DELAYED_WORK(&chip->monitor_work, oplus_vc_monitor_work);

	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto reg_ic_err;
	}
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-virtual:%d", ic_index);
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.get_func = oplus_chg_vc_get_func;
	ic_cfg.virq_data = oplus_vc_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(oplus_vc_virq_table);
	ic_cfg.of_node = node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto reg_ic_err;
	}

	chip->reg_proc_node = of_property_read_bool(node, "oplus,reg_proc_node");
	if (chip->reg_proc_node) {
		rc = oplus_virtual_cp_proc_init(chip);
		if (rc < 0) {
			rc = -EPROBE_DEFER;
			goto proc_init_err;
		}
	}

	rc = oplus_vc_child_init(chip);
	if (rc < 0) {
		chg_err("child ic init error, rc=%d\n", rc);
		goto child_init_err;
	}

	chg_info("probe success\n");
	return 0;

child_init_err:
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	if (chip->reg_proc_node && charger_dir != NULL) {
		snprintf(name_buf, CP_NAME_BUF_MAX - 1, "cp_%d", chip->ic_dev->index);
		chg_err("remove virtual_cp index cp_%d\n", chip->ic_dev->index);
		remove_proc_entry(name_buf, charger_dir);
	}
#else
	if (chip->reg_proc_node) {
		snprintf(name_buf, CP_NAME_BUF_MAX - 1, "charger/cp:%d", chip->ic_dev->index);
		remove_proc_entry(name_buf, NULL);
	}
#endif
proc_init_err:
	devm_oplus_chg_ic_unregister(&pdev->dev, chip->ic_dev);
reg_ic_err:
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

	if (rc == -EPROBE_DEFER && retry_count < PROBE_RETRY_MAX) {
		retry_count++;
		return rc;
	}
	chg_err("probe error, retry=%d, rc=%d\n", retry_count, rc);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_virtual_cp_remove(struct platform_device *pdev)
#else
static int oplus_virtual_cp_remove(struct platform_device *pdev)
#endif
{
	struct oplus_virtual_cp_ic *chip = platform_get_drvdata(pdev);
	char name_buf[CP_NAME_BUF_MAX] = { 0 };

	if (chip == NULL) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
		return -ENODEV;
#else
		return;
#endif
	}

	if (chip->ic_dev->online)
		oplus_chg_vc_exit(chip->ic_dev);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	if (chip->reg_proc_node && charger_dir != NULL) {
		snprintf(name_buf, CP_NAME_BUF_MAX - 1, "cp_%d", chip->ic_dev->index);
		chg_err("oplus_virtual_cp_remove index cp_%d\n", chip->ic_dev->index);
		remove_proc_entry(name_buf, charger_dir);
	}
#else
	if (chip->reg_proc_node) {
		snprintf(name_buf, CP_NAME_BUF_MAX - 1, "charger/cp:%d", chip->ic_dev->index);
		remove_proc_entry(name_buf, NULL);
	}
#endif
	devm_oplus_chg_ic_unregister(&pdev->dev, chip->ic_dev);
	devm_kfree(&pdev->dev, chip->child_list);
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_virtual_cp_match[] = {
	{ .compatible = "oplus,virtual_cp" },
	{},
};

static struct platform_driver oplus_virtual_cp_driver = {
	.driver		= {
		.name = "oplus-virtual_cp",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_virtual_cp_match),
	},
	.probe		= oplus_virtual_cp_probe,
	.remove		= oplus_virtual_cp_remove,
};

static __init int oplus_virtual_cp_init(void)
{
	return platform_driver_register(&oplus_virtual_cp_driver);
}

static __exit void oplus_virtual_cp_exit(void)
{
	platform_driver_unregister(&oplus_virtual_cp_driver);
}

oplus_chg_module_register(oplus_virtual_cp);
