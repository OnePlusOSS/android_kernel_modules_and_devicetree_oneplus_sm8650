/***********************************************************
** Copyright (C), 2025-2025 Oplus. All rights reserved.
** File: oplus_virtual_reverse_chg.c
** Description: virtual reverse ic
** Date: 2025-11-14
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#define pr_fmt(fmt) "[VIRTUAL_REVERSE_CHG]([%s][%d]): " fmt, __func__, __LINE__

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
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_reverse_chg.h>

#define REVERSE_CHG_REG_TIMEOUT_MS	120000
#define REVERSE_CHG_NAME_BUF_MAX	128

struct oplus_virtual_reverse_chg_child {
	struct oplus_chg_ic_dev *parent;
	struct oplus_chg_ic_dev *ic_dev;
	int index;
	struct work_struct online_work;
	struct work_struct offline_work;
};

struct oplus_virtual_reverse_chg_ic;

struct oplus_virtual_reverse_chg_ic {
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	bool online;
	int child_num;

	struct oplus_virtual_reverse_chg_child *child_list;
	struct mutex online_lock;
};

static void oplus_vrc_online_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_reverse_chg_child *child = virq_data;
	schedule_work(&child->online_work);
}

static void oplus_vrc_offline_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_reverse_chg_child *child = virq_data;
	schedule_work(&child->offline_work);
}

static void oplus_vrc_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_reverse_chg_ic *chip = virq_data;

	oplus_chg_ic_copy_err_msg(chip->ic_dev, ic_dev);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
}

static void oplus_vrc_reverse_enable_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_reverse_chg_ic *chip = virq_data;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_REVERSE_ENABLE);
}

static void oplus_vrc_high_reverse_enable_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_reverse_chg_ic *chip = virq_data;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE);
}

static void oplus_vrc_hard_reset_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_reverse_chg_ic *chip = virq_data;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_HARD_RESET);
}

static void oplus_vrc_sink_req_msg_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_reverse_chg_ic *chip = virq_data;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_SINK_REQ_MSG);
}

static int oplus_vrc_base_virq_register(struct oplus_virtual_reverse_chg_ic *chip, int index)
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
			OPLUS_IC_VIRQ_ONLINE, oplus_vrc_online_handler, virq_data);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(ic_dev,
			OPLUS_IC_VIRQ_OFFLINE, oplus_vrc_offline_handler, virq_data);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_OFFLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(ic_dev,
		OPLUS_IC_VIRQ_ERR, oplus_vrc_err_handler, chip);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(ic_dev,
		OPLUS_IC_VIRQ_REVERSE_ENABLE, oplus_vrc_reverse_enable_handler, chip);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_info("register OPLUS_IC_VIRQ_REVERSE_ENABLE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(ic_dev,
		OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE, oplus_vrc_high_reverse_enable_handler, chip);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_info("register OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(ic_dev,
		OPLUS_IC_VIRQ_HARD_RESET, oplus_vrc_hard_reset_handler, chip);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_info("register OPLUS_IC_VIRQ_HARD_RESET error, rc=%d", rc);

	rc = oplus_chg_ic_virq_register(ic_dev,
		OPLUS_IC_VIRQ_SINK_REQ_MSG, oplus_vrc_sink_req_msg_handler, chip);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_info("register OPLUS_IC_VIRQ_SINK_REQ_MSG error, rc=%d", rc);

	if (ic_dev->name)
		chg_info("%s virq register success\n", ic_dev->name);

	return 0;
}

static void oplus_vrc_online_work(struct work_struct *work)
{
	struct oplus_virtual_reverse_chg_child *child =
		container_of(work, struct oplus_virtual_reverse_chg_child, online_work);
	struct oplus_virtual_reverse_chg_ic *chip;
	bool online = false;
	int i;

	chg_info("%s online\n", child->ic_dev->manu_name);
	chip = oplus_chg_ic_get_drvdata(child->parent);

	for (i = 0; i < chip->child_num; i++) {
		if (chip->child_list[i].ic_dev == NULL)
			continue;
		if (!chip->child_list[i].ic_dev->online)
			continue;
		online = true;
		break;
	}

	if (!child->parent->online && online) {
		oplus_chg_ic_func(child->parent, OPLUS_IC_FUNC_INIT);
	}
}

static void oplus_vrc_offline_work(struct work_struct *work)
{
	struct oplus_virtual_reverse_chg_child *child =
		container_of(work, struct oplus_virtual_reverse_chg_child, offline_work);
	struct oplus_virtual_reverse_chg_ic *chip;
	bool online = true;
	int i;

	chg_info("%s offline\n", child->ic_dev->manu_name);
	chip = oplus_chg_ic_get_drvdata(child->parent);

	for (i = 0; i < chip->child_num; i++) {
		if (chip->child_list[i].ic_dev == NULL)
			continue;
		if (!chip->child_list[i].ic_dev->online) {
				online = false;
			continue;
		}
	}

	if (child->parent->online && !online) {
		oplus_chg_ic_func(child->parent, OPLUS_IC_FUNC_EXIT);
	}
}

static void oplus_vrc_child_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_virtual_reverse_chg_child *child;
	struct oplus_chg_ic_dev *parent;
	struct oplus_virtual_reverse_chg_ic *chip;
	int rc;

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
		return;
	}

	child->ic_dev = ic;
	oplus_chg_ic_set_parent(ic, parent);

	rc = oplus_vrc_base_virq_register(chip, child->index);
	if (rc < 0) {
		chg_err("%s virq register error, rc=%d\n", ic->name, rc);
		return;
	}

	chg_info("ic->name %s online = %d, index = %d",
		  ic->name, parent->online, child->index);
	if (!parent->online) {
		rc = oplus_chg_ic_func(child->parent, OPLUS_IC_FUNC_INIT);
		if (rc < 0) {
			parent->online = false;
			chg_err("ic->name %s,index = %d rc = %d init failed, set online as false",
				 ic->name, child->index, rc);
		} else {
			chg_info("ic->name %s online = %d index = %d init success, set online.",
				  ic->name, parent->online, child->index);
			parent->online = true;
		}
	}
}

static int oplus_vrc_child_init(struct oplus_virtual_reverse_chg_ic *chip)
{
	struct device_node *node = chip->dev->of_node;
	int i = 0;
	int rc = 0;
	const char *name;
	struct device_node *child;

	rc = of_property_count_elems_of_size(node, "oplus,reverse_chg_ic",
					     sizeof(u32));
	if (rc < 0) {
		chg_err("can't get reverse_chg ic number, rc=%d\n", rc);
		return rc;
	}
	chip->child_num = rc;
	chip->child_list = devm_kzalloc(
		chip->dev,
		sizeof(struct oplus_virtual_reverse_chg_child) * chip->child_num,
		GFP_KERNEL);
	if (chip->child_list == NULL) {
		rc = -ENOMEM;
		chg_err("alloc child ic memory error\n");
		return rc;
	}

	for (i = 0; i < chip->child_num; i++) {
		chip->child_list[i].index = i;
		chip->child_list[i].parent = chip->ic_dev;
		INIT_WORK(&chip->child_list[i].online_work, oplus_vrc_online_work);
		INIT_WORK(&chip->child_list[i].offline_work, oplus_vrc_offline_work);
		name = of_get_oplus_chg_ic_name(node, "oplus,reverse_chg_ic", i);
		chg_info("name is %s, i = %d", name, i);
		rc = oplus_chg_ic_wait_ic_timeout(name, oplus_vrc_child_reg_callback,
						  &chip->child_list[i],
						  msecs_to_jiffies(REVERSE_CHG_REG_TIMEOUT_MS));
		if (rc < 0) {
			chg_err("can't wait ic[%d](%s), rc=%d\n", i, name, rc);
			goto read_property_err;
		}
	}

	for_each_child_of_node(node, child) {
		if (!of_property_read_bool(child, "oplus,reverse_chg_ic"))
			continue;
		for (i = 0; i < chip->child_num; i++) {
			name = of_get_oplus_chg_ic_name(child, "oplus,reverse_chg_ic", i);
			chg_info("name is %s, i = %d", name, i);
			rc = oplus_chg_ic_wait_ic_timeout(name, oplus_vrc_child_reg_callback, &chip->child_list[i],
							  msecs_to_jiffies(REVERSE_CHG_REG_TIMEOUT_MS));
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

static int oplus_chg_vrc_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_reverse_chg_ic *chip;

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

	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);
	mutex_unlock(&chip->online_lock);

	return 0;
}

static int oplus_chg_vrc_exit(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_reverse_chg_ic *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!ic_dev->online)
		return 0;

	mutex_lock(&chip->online_lock);
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);
	chg_info("unregister success\n");
	mutex_unlock(&chip->online_lock);

	return 0;
}

static int oplus_chg_vrc_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_REG_DUMP);
		if (rc < 0)
			chg_err("child ic[%d] reg dump error, rc=%d\n", i, rc);
	}

	return 0;
}

static int oplus_chg_vrc_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int index;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);
	index = 0;
	for (i = 0; i < vrc->child_num; i++) {
		if (index >= len)
			return len;
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev,
				       OPLUS_IC_FUNC_SMT_TEST, buf + index,
				       len - index);
		if (rc < 0) {
			if (rc != -ENOTSUPP) {
				chg_err("child ic[%d] smt test error, rc=%d\n",
					i, rc);
				rc = snprintf(buf + index, len - index,
					"[%s]-[%s]:%d\n",
					vrc->child_list[i].ic_dev->manu_name,
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

static int oplus_chg_vrc_get_reverse_enable(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_GET_REVERSE_ENABLE, enable);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] %s error, rc=%d\n", i, enable ? "enable" : "disable", rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vrc_get_high_reverse_enable(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_GET_HIGH_REVERSE_ENABLE, enable);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] %s error, rc=%d\n", i, enable ? "enable" : "disable", rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vrc_set_rvs_high_en(struct oplus_chg_ic_dev *ic_dev, int enable)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN, enable);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] %s error, rc=%d\n", i, enable ? "enable" : "disable", rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vrc_set_reverse_src_pdo
	(struct oplus_chg_ic_dev *ic_dev, int pdo0_vol_mv, int pdo0_curr_ma, int pdo1_vol_mv, int pdo1_curr_ma)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO,
			pdo0_vol_mv, pdo0_curr_ma, pdo1_vol_mv, pdo1_curr_ma);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] pdo_set error, rc=%d\n", i, rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vrc_get_reverse_chg_svid(struct oplus_chg_ic_dev *ic_dev, u32 *svid)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_GET_REVERSE_CHG_SVID, svid);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] get svid error, rc=%d\n", i, rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vrc_wdt_enable(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_REVERSE_CHG_WDT_ENABLE, enable);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] set wdt enable error, rc=%d\n", i, rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vrc_kick_wdt(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_REVERSE_CHG_KICK_WDT);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] set kick wdt error, rc=%d\n", i, rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int  oplus_chg_vrc_get_rvs_msg_type(struct oplus_chg_ic_dev *ic_dev, int *msg_type)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_GET_RVS_CHG_MSG, msg_type);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] get sink req msg error, rc=%d\n", i, rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int  oplus_chg_vrc_get_sink_req_pdo(struct oplus_chg_ic_dev *ic_dev,
	int *req_voltage, int *req_current)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_GET_SINK_REQ_PDO, req_voltage, req_current);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] get sink req pdo error, rc=%d\n", i, rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vrc_get_reverse_chg_type(struct oplus_chg_ic_dev *ic_dev, int *reverse_chg_type)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev, OPLUS_IC_FUNC_GET_REVERSE_CHG_TYPE, reverse_chg_type);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] get reverse chg type error, rc=%d\n", i, rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vrc_get_source_plug_out(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);

	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev,
			OPLUS_IC_FUNC_GET_SOURCE_PLUG_OUT, enable);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] %s error, rc=%d\n",
				i, enable ? "enable" : "disable", rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static int oplus_chg_vrc_set_normal_reverse_hw_ocp(struct oplus_chg_ic_dev *ic_dev, int curr_ma)
{
	struct oplus_virtual_reverse_chg_ic *vrc;
	int i;
	int rc = 0;
	int err = -ENOTSUPP;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	vrc = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vrc->child_num; i++) {
		rc = oplus_chg_ic_func(vrc->child_list[i].ic_dev,
			OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP, curr_ma);
		if (rc < 0 && rc != -ENOTSUPP) {
			chg_err("child ic[%d] hw_ocp_set error, rc=%d\n", i, rc);
			return rc;
		} else if (rc >= 0) {
			err = rc;
		}
	}

	return err;
}

static void *oplus_chg_vrc_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}
	if (!oplus_chg_ic_func_is_support(ic_dev, func_id)) {
		chg_info("%s: this func(=%d) is not supported\n",  ic_dev->name, func_id);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_vrc_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_vrc_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, oplus_chg_vrc_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, oplus_chg_vrc_smt_test);
		break;
	case OPLUS_IC_FUNC_GET_REVERSE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_ENABLE, oplus_chg_vrc_get_reverse_enable);
		break;
	case OPLUS_IC_FUNC_GET_HIGH_REVERSE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_HIGH_REVERSE_ENABLE,
			oplus_chg_vrc_get_high_reverse_enable);
		break;
	case OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN,
			oplus_chg_vrc_set_rvs_high_en);
		break;
	case OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO, oplus_chg_vrc_set_reverse_src_pdo);
		break;
	case OPLUS_IC_FUNC_GET_REVERSE_CHG_SVID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_CHG_SVID, oplus_chg_vrc_get_reverse_chg_svid);
		break;
	case OPLUS_IC_FUNC_REVERSE_CHG_WDT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REVERSE_CHG_WDT_ENABLE, oplus_chg_vrc_wdt_enable);
		break;
	case OPLUS_IC_FUNC_REVERSE_CHG_KICK_WDT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REVERSE_CHG_KICK_WDT, oplus_chg_vrc_kick_wdt);
		break;
	case OPLUS_IC_FUNC_GET_RVS_CHG_MSG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_RVS_CHG_MSG,
					       oplus_chg_vrc_get_rvs_msg_type);
		break;
	case OPLUS_IC_FUNC_GET_SINK_REQ_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SINK_REQ_PDO,
					       oplus_chg_vrc_get_sink_req_pdo);
		break;
	case OPLUS_IC_FUNC_GET_REVERSE_CHG_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_CHG_TYPE,
					       oplus_chg_vrc_get_reverse_chg_type);
		break;
	case OPLUS_IC_FUNC_GET_SOURCE_PLUG_OUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SOURCE_PLUG_OUT,
					       oplus_chg_vrc_get_source_plug_out);
		break;
	case OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP,
					       oplus_chg_vrc_set_normal_reverse_hw_ocp);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_vrc_virq_table[] = {
	{.virq_id = OPLUS_IC_VIRQ_ERR},
	{.virq_id = OPLUS_IC_VIRQ_ONLINE},
	{.virq_id = OPLUS_IC_VIRQ_OFFLINE},
	{.virq_id = OPLUS_IC_VIRQ_REVERSE_ENABLE},
	{.virq_id = OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE},
	{.virq_id = OPLUS_IC_VIRQ_HARD_RESET},
	{.virq_id = OPLUS_IC_VIRQ_SINK_REQ_MSG},
};

static int oplus_virtual_reverse_chg_probe(struct platform_device *pdev)
{
	struct oplus_virtual_reverse_chg_ic *chip;
	struct device_node *node = pdev->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	static int retry_count = 0;
	int ic_index;
	int rc;

#define PROBE_RETRY_MAX	300

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_virtual_reverse_chg_ic),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	mutex_init(&chip->online_lock);

	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto reg_ic_err;
	}
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "reverse-virtual:%d", ic_index);
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.get_func = oplus_chg_vrc_get_func;
	ic_cfg.virq_data = oplus_vrc_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(oplus_vrc_virq_table);
	ic_cfg.of_node = node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto reg_ic_err;
	}

	rc = oplus_vrc_child_init(chip);
	if (rc < 0) {
		chg_err("child ic init error, rc=%d\n", rc);
		goto child_init_err;
	}

	chg_info("probe success\n");
	return 0;

child_init_err:
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
static void oplus_virtual_reverse_chg_remove(struct platform_device *pdev)
#else
static int oplus_virtual_reverse_chg_remove(struct platform_device *pdev)
#endif
{
	struct oplus_virtual_reverse_chg_ic *chip = platform_get_drvdata(pdev);

	if (chip == NULL) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
		return -ENODEV;
#else
		return;
#endif
	}

	if (chip->ic_dev->online)
		oplus_chg_vrc_exit(chip->ic_dev);

	devm_oplus_chg_ic_unregister(&pdev->dev, chip->ic_dev);
	devm_kfree(&pdev->dev, chip->child_list);
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_virtual_reverse_chg_match[] = {
	{.compatible = "oplus,virtual_reverse_chg"},
	{},
};

static struct platform_driver oplus_virtual_reverse_chg_driver = {
	.driver		= {
		.name = "oplus-virtual-reverse-chg",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_virtual_reverse_chg_match),
	},
	.probe		= oplus_virtual_reverse_chg_probe,
	.remove		= oplus_virtual_reverse_chg_remove,
};

static __init int oplus_virtual_reverse_chg_init(void)
{
	return platform_driver_register(&oplus_virtual_reverse_chg_driver);
}

static __exit void oplus_virtual_reverse_chg_exit(void)
{
	platform_driver_unregister(&oplus_virtual_reverse_chg_driver);
}

oplus_chg_module_register(oplus_virtual_reverse_chg);
