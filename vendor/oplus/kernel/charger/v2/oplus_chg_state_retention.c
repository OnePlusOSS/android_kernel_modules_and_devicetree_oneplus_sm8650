// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2024 . Oplus All rights reserved.
 */

#define pr_fmt(fmt) "[RETENTION_CHG]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/gfp.h>
#include <linux/power_supply.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/system/oplus_project.h>
#endif

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_quirks.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_cpa.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_state_retention.h>

struct oplus_retention_charge {
	struct device *dev;
	struct oplus_mms *comm_topic;
	struct oplus_mms *wired_topic;
	struct oplus_mms *retention_topic;
	struct oplus_mms *cpa_topic;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *cpa_subs;
	bool wired_online;

	int wired_type;
	int irq_plugin;
	int pre_irq_plugin;
	int connect_status;
	int first_present_out;
	int pre_connect_status;
	int connect_status_flag;
	int disconnect_count;
	int total_disconnect_count;
	int connect_error_count_level;
	int cc_detect;
	int detect_flag;
	int irq_plug_flag;
	int clear_disconnect_count_flag;
	enum oplus_chg_protocol_type cpa_current_type;
	enum oplus_chg_protocol_type pre_cpa_current_type;

	struct delayed_work update_work;
	struct delayed_work online_check_work;
	struct delayed_work present_check_work;
	struct delayed_work wired_type_change_work;
};

#define CONNECT_ERROR_COUNT_LEVEL 3
#define VOOC_CONNECT_ERROR_COUNT_LEVEL 12
#define CABLE_PLUGOUT_FROM_ADAPTER	1500

static struct oplus_chg_retention *retention;

static DEFINE_MUTEX(list_lock);
static LIST_HEAD(retention_list);

static struct oplus_chg_retention_desc *retention_desc_find_by_name(const char *name)
{
	struct oplus_chg_retention_desc *desc;

	if (name == NULL) {
		chg_err("name is NULL\n");
		return NULL;
	}

	mutex_lock(&list_lock);
	list_for_each_entry(desc, &retention_list, list) {
		if (desc->name == NULL)
			continue;
		if (strcmp(desc->name, name) == 0) {
			mutex_unlock(&list_lock);
			return desc;
		}
	}
	mutex_unlock(&list_lock);
	chg_err("%s get desc->name, name=%s\n", desc->name, name);
	return NULL;
}

struct oplus_chg_retention *
oplus_chg_retention_alloc(const char *name)
{
	struct oplus_chg_retention_desc *desc;

	if (name == NULL) {
		chg_err("name is NULL\n");
		return NULL;
	}

	desc = retention_desc_find_by_name(name);
	if (desc == NULL) {
		chg_err("No retention with name %s was found\n", name);
		return NULL;
	}

	retention = kzalloc(sizeof(struct oplus_chg_retention), GFP_KERNEL);

	if (IS_ERR_OR_NULL(retention)) {
		chg_err("%s retention alloc error, rc=%ld\n", name, PTR_ERR(retention));
		return NULL;
	}

	retention->desc = desc;

	return retention;
}

static int oplus_chg_state_retention(struct oplus_chg_retention *retention)
{
	int rc = 0;
	struct oplus_chg_retention_desc *desc;

	if (retention == NULL) {
		chg_err("retention is NULL\n");
		return -EINVAL;
	}
	if (retention->desc == NULL) {
		chg_err("retention desc is NULL\n");
		return -EINVAL;
	}
	list_for_each_entry(desc, &retention_list, list)
		rc |= retention->desc->state_retention();

	if (rc < 0) {
		chg_err("%s get state retention error, rc=%d\n", retention->desc->name, rc);
		return rc;
	}

	return rc;
}

static int oplus_chg_state_retention_notify(struct oplus_chg_retention *retention, bool irq_plugin)
{
	int rc = 0;
	struct oplus_chg_retention_desc *desc;

	if (retention == NULL) {
		chg_err("retention is NULL\n");
		return -EINVAL;
	}
	if (retention->desc == NULL) {
		chg_err("retention desc is NULL\n");
		return -EINVAL;
	}
	list_for_each_entry(desc, &retention_list, list)
		rc = retention->desc->state_retention_notify(irq_plugin);

	if (rc < 0) {
		chg_err("%s get state retention error, rc=%d\n", retention->desc->name, rc);
		return rc;
	}

	return rc;
}

static int oplus_chg_retention_disconnect_count(struct oplus_chg_retention *retention)
{
	int rc;

	if (retention == NULL) {
		chg_err("retention is NULL\n");
		return -EINVAL;
	}
	if (retention->desc == NULL) {
		chg_err("retention desc is NULL\n");
		return -EINVAL;
	}

	rc = retention->desc->disconnect_count();
	if (rc < 0) {
		chg_err("%s get diconnect count error, rc=%d\n", retention->desc->name, rc);
		return rc;
	}

	return rc;
}

static int oplus_chg_retention_clear_disconnect_count(struct oplus_chg_retention *retention)
{
	int rc;

	if (retention == NULL) {
		chg_err("retention is NULL\n");
		return -EINVAL;
	}
	if (retention->desc == NULL) {
		chg_err("retention desc is NULL\n");
		return -EINVAL;
	}

	rc = retention->desc->clear_disconnect_count();
	if (rc < 0) {
		chg_err("%s set clear connect status error, rc=%d\n", retention->desc->name, rc);
		return rc;
	}

	return rc;
}

int oplus_chg_retention_register(struct oplus_chg_retention_desc *desc)
{
	struct oplus_chg_retention_desc *desc_temp;

	if (desc == NULL) {
		chg_err("retention desc is NULL");
		return -EINVAL;
	}

	mutex_lock(&list_lock);
	list_for_each_entry(desc_temp, &retention_list, list) {
		if (desc_temp->name == NULL)
			continue;
		if (strcmp(desc_temp->name, desc->name) == 0) {
			chg_err("the same retention name already exists\n");
			mutex_unlock(&list_lock);
			return -EINVAL;
		}
	}
	list_add(&desc->list, &retention_list);
	mutex_unlock(&list_lock);
	chg_debug("%s retention_register desc->name\n", desc->name);
	return 0;
}

static bool oplus_state_retention(struct oplus_mms *topic)
{
	struct oplus_retention_charge *chip;
	bool state_retention = false;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return 0;
	}
	if (retention == NULL) {
		chg_err("retention_chip is NULL\n");
		return 0;
	}

	chip = oplus_mms_get_drvdata(topic);

	if (oplus_chg_state_retention(retention)) {
		state_retention = true;
	} else {
		state_retention = false;
		chip->disconnect_count = 0;
		chip->total_disconnect_count = 0;
		chip->clear_disconnect_count_flag = 0;
	}
	return state_retention;
}

static int oplus_state_retention_notify(struct oplus_mms *topic, bool irq_plugin)
{
	struct oplus_retention_charge *chip;
	int rc;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return 0;
	}
	if (retention == NULL) {
		chg_err("retention_chip is NULL\n");
		return 0;
	}

	chip = oplus_mms_get_drvdata(topic);

	rc = oplus_chg_state_retention_notify(retention, irq_plugin);
	return rc;
}

static int oplus_retention_disconnect_count(struct oplus_mms *topic)
{
	int val = 0;
	struct oplus_retention_charge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return 0;
	}
	if (retention == NULL) {
		chg_err("retention_chip is NULL\n");
		return 0;
	}

	chip = oplus_mms_get_drvdata(topic);

	val = oplus_chg_retention_disconnect_count(retention);
	return val;
}

static void oplus_retention_clear_disconnect_count(struct oplus_mms *topic)
{
	struct oplus_retention_charge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	if (retention == NULL) {
		chg_err("retention_chip is NULL\n");
		return;
	}

	chip = oplus_mms_get_drvdata(topic);
	oplus_chg_retention_clear_disconnect_count(retention);
	return;
}

static void oplus_retention_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_retention_charge *chip = container_of(dwork, struct oplus_retention_charge,
						   update_work);
	struct mms_msg *msg;
	int rc;

	if (chip->cc_detect <= CC_DETECT_NULL) {
		chg_err("can't support cc_detect, state_retention will not be available\n");
		return;
	}

	chip->disconnect_count = oplus_retention_disconnect_count(chip->retention_topic);
	chip->connect_status = oplus_state_retention(chip->retention_topic);
	if (chip->cc_detect == CC_DETECT_NOTPLUG || chip->detect_flag == CC_DETECT_NOTPLUG) {
		chip->connect_status = 0;
		chip->total_disconnect_count = 0;
		chip->clear_disconnect_count_flag = 0;
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, RETENTION_ITEM_CONNECT_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
	} else {
		rc = oplus_mms_publish_msg_sync(chip->retention_topic, msg);
		if (rc < 0) {
			chg_err("publish retention connect status msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, RETENTION_ITEM_DISCONNECT_COUNT);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
	} else {
		rc = oplus_mms_publish_msg_sync(chip->retention_topic, msg);
		if (rc < 0) {
			chg_err("publish retention diconnect count msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}

	if (chip->irq_plugin) {
		msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, RETENTION_ITEM_TOTAL_DISCONNECT_COUNT);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
		} else {
			rc = oplus_mms_publish_msg_sync(chip->retention_topic, msg);
			if (rc < 0) {
				chg_err("publish retention diconnect count msg error, rc=%d\n", rc);
				kfree(msg);
			}
		}
	}
	if (!chip->irq_plugin && chip->pre_irq_plugin && !chip->pre_connect_status) {
		chip->first_present_out = true;
		msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, RETENTION_ITEM_STATE_READY);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
		} else {
			rc = oplus_mms_publish_msg_sync(chip->retention_topic, msg);
			if (rc < 0) {
				chg_err("publish retention diconnect count msg error, rc=%d\n", rc);
				kfree(msg);
			}
		}
	}
	chip->pre_irq_plugin = chip->irq_plugin;
	chip->pre_connect_status = chip->connect_status;
	chip->irq_plug_flag = 0;

	chg_info("irq_plugin=%d, state_retention=%d, disconnect_count=%d, total_disconnect_count=%d\n",
		chip->irq_plugin, chip->connect_status, chip->disconnect_count, chip->total_disconnect_count);

	return;
}

static void oplus_retention_online_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_retention_charge *chip = container_of(dwork, struct oplus_retention_charge,
						   online_check_work);
	struct mms_msg *msg;
	bool connect_status;
	int rc;

	connect_status = oplus_state_retention(chip->retention_topic);
	if (chip->cc_detect == CC_DETECT_NOTPLUG || chip->detect_flag == CC_DETECT_NOTPLUG) {
		connect_status = 0;

		if (chip->connect_status == connect_status)
			return;
		chip->connect_status = connect_status;
		msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, RETENTION_ITEM_CONNECT_STATUS);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			return;
		}
		rc = oplus_mms_publish_msg_sync(chip->retention_topic, msg);
		if (rc < 0) {
			chg_err("publish retention connect status msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}
}

static void oplus_retention_wired_type_change_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_retention_charge *chip = container_of(dwork, struct oplus_retention_charge,
						   wired_type_change_work);
	struct mms_msg *msg;
	int rc;

	if (chip->wired_type == OPLUS_CHG_USB_TYPE_PD_SDP || chip->wired_type == OPLUS_CHG_USB_TYPE_SDP ||
		chip->wired_type == OPLUS_CHG_USB_TYPE_CDP) {
		chip->connect_status = 0;
		msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, RETENTION_ITEM_CONNECT_STATUS);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
		} else {
			rc = oplus_mms_publish_msg_sync(chip->retention_topic, msg);
			if (rc < 0) {
				chg_err("publish retention connect status msg error, rc=%d\n", rc);
				kfree(msg);
			}
		}
	}

	if (!chip->connect_status)
		chip->pre_cpa_current_type = chip->cpa_current_type;

	if (chip->pre_cpa_current_type != chip->cpa_current_type &&
	    chip->connect_status) {
		chip->clear_disconnect_count_flag += chip->disconnect_count;
		oplus_retention_clear_disconnect_count(chip->retention_topic);
		chip->pre_cpa_current_type = chip->cpa_current_type;
		chip->disconnect_count = 0;
	}

	if (chip->connect_status) {
		chip->disconnect_count = oplus_retention_disconnect_count(chip->retention_topic);
		chip->total_disconnect_count = chip->disconnect_count + chip->clear_disconnect_count_flag;
	}
}

static void oplus_retention_present_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_retention_charge *chip = container_of(dwork, struct oplus_retention_charge,
						   present_check_work);
	int ret = 0;

	oplus_state_retention_notify(chip->retention_topic, chip->irq_plugin);
	schedule_delayed_work(&chip->update_work, 0);
	chip->connect_status_flag = oplus_state_retention(chip->retention_topic);
	chg_debug("plugin = %d, connect_status_flag =%d\n", chip->irq_plugin, chip->connect_status_flag);
	if (!chip->irq_plugin && chip->connect_status_flag) {
		ret = schedule_delayed_work(&chip->present_check_work,
			msecs_to_jiffies(CABLE_PLUGOUT_FROM_ADAPTER));
		if (ret == 0) {
			cancel_delayed_work(&chip->present_check_work);
			ret = schedule_delayed_work(&chip->present_check_work,
				msecs_to_jiffies(CABLE_PLUGOUT_FROM_ADAPTER));
			chg_info("ret:%d\n", ret);
		}
	}
}

static void oplus_retention_chg_wired_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_retention_charge *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chip->wired_online = !!data.intval;
			if (!chip->wired_online)
				schedule_delayed_work(&chip->online_check_work, 0);
			if (chip->irq_plug_flag == 0 && !chip->wired_online)
				schedule_delayed_work(&chip->present_check_work, 0);
			break;
		case WIRED_ITEM_CHG_TYPE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chip->wired_type = data.intval;
			schedule_delayed_work(&chip->wired_type_change_work, 0);
			break;
		case WIRED_ITEM_PRESENT:
			chip->irq_plug_flag = 1;
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chip->irq_plugin = !!data.intval;
			cancel_delayed_work(&chip->present_check_work);
			schedule_delayed_work(&chip->present_check_work, 0);
			break;
		case WIRED_ITEM_CC_DETECT:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chip->cc_detect = data.intval;
			chip->detect_flag = chip->cc_detect;
			if (chip->cc_detect == CC_DETECT_NOTPLUG)
				schedule_delayed_work(&chip->update_work, 0);
			break;
		case WIRED_ITEM_ONLINE_STATUS_ERR:
			cancel_delayed_work(&chip->online_check_work);
			if (chip->connect_status) {
				chip->detect_flag = CC_DETECT_NOTPLUG;
				schedule_delayed_work(&chip->present_check_work, 0);
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_retention_chg_subscribe_wired_topic(struct oplus_mms *topic,
						  void *prv_data)
{
	struct oplus_retention_charge *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_retention_chg_wired_subs_callback, "retention");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data, true);
	chip->wired_online = !!data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE, &data, true);
	chip->wired_type = data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CC_DETECT, &data, true);
	chip->cc_detect = data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_PRESENT, &data, true);
	chip->irq_plugin = data.intval;
	if (chip->irq_plugin)
		schedule_delayed_work(&chip->present_check_work, 0);
}

static void oplus_retention_chg_cpa_subs_callback(struct mms_subscribe *subs,
						enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_retention_charge *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case CPA_ITEM_ALLOW:
			oplus_mms_get_item_data(chip->cpa_topic, id, &data, false);
			chip->cpa_current_type = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_retention_chg_subscribe_cpa_topic(struct oplus_mms *topic,
						  void *prv_data)
{
	struct oplus_retention_charge *chip = prv_data;

	chip->cpa_topic = topic;
	chip->cpa_subs =
		oplus_mms_subscribe(chip->cpa_topic, chip,
				    oplus_retention_chg_cpa_subs_callback, "retention");
	if (IS_ERR_OR_NULL(chip->cpa_subs)) {
		chg_err("subscribe cpa topic error, rc=%ld\n",
			PTR_ERR(chip->cpa_subs));
		return;
	}
}

static int retention_connect_status(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_retention_charge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->connect_status;

	return 0;
}

static int retention_disconnect_count(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_retention_charge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->disconnect_count;

	return 0;
}

static int retention_total_disconnect_count(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_retention_charge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->total_disconnect_count;

	return 0;
}

static int retention_first_present_out(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_retention_charge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->first_present_out;

	return 0;
}

static void oplus_retention_topic_update(struct oplus_mms *mms, bool publish)
{
}

static struct mms_item oplus_retention_item[] = {
	{
		.desc = {
			.item_id = RETENTION_ITEM_CONNECT_STATUS,
			.update = retention_connect_status,
		}
	},
	{
		.desc = {
			.item_id = RETENTION_ITEM_DISCONNECT_COUNT,
			.update = retention_disconnect_count,
		}
	},
	{
		.desc = {
			.item_id = RETENTION_ITEM_TOTAL_DISCONNECT_COUNT,
			.update = retention_total_disconnect_count,
		}
	},
	{
		.desc = {
			.item_id = RETENTION_ITEM_STATE_READY,
			.update = retention_first_present_out,
		}
	},
};

static const struct oplus_mms_desc oplus_retention_desc = {
	.name = "retention",
	.type = OPLUS_MMS_TYPE_RETENTION,
	.item_table = oplus_retention_item,
	.item_num = ARRAY_SIZE(oplus_retention_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_retention_topic_update,
};

static int oplus_retention_topic_init(struct oplus_retention_charge *chip)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;

	mms_cfg.drv_data = chip;
	mms_cfg.of_node = chip->dev->of_node;

	if (of_property_read_bool(mms_cfg.of_node,
				  "oplus,topic-update-interval")) {
		rc = of_property_read_u32(mms_cfg.of_node,
					  "oplus,topic-update-interval",
					  &mms_cfg.update_interval);
		if (rc < 0)
			mms_cfg.update_interval = 0;
	}

	chip->retention_topic = devm_oplus_mms_register(chip->dev, &oplus_retention_desc, &mms_cfg);
	if (IS_ERR(chip->retention_topic)) {
		chg_err("Couldn't register retention topic\n");
		rc = PTR_ERR(chip->retention_topic);
		return rc;
	}

	return 0;
}

static void oplus_retention_chg_common_topic_ready(struct oplus_mms *topic,
						void *prv_data)
{
	struct oplus_retention_charge *chip = prv_data;
	int rc;

	chip->comm_topic = topic;

	rc = oplus_retention_topic_init(chip);
	if (rc < 0)
		goto topic_init_err;

	oplus_mms_wait_topic("wired", oplus_retention_chg_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("cpa", oplus_retention_chg_subscribe_cpa_topic, chip);

topic_init_err:
	return;
}

static int oplus_retention_chip_init(struct oplus_retention_charge *chip)
{
	const char *support_retention_name;
	struct oplus_chg_retention *retention_chip;
	struct device_node *node = chip->dev->of_node;
	int rc;

	rc = of_property_read_string(node, "oplus,support_retention_name", &support_retention_name);
	if (rc < 0)
		return -ENODEV;

	retention_chip = oplus_chg_retention_alloc(support_retention_name);
	if (retention_chip == NULL)
		chg_err("retention chip alloc error");

	return 0;
}

static int oplus_chg_state_retention_probe(struct platform_device *pdev)
{
	struct oplus_retention_charge *chip;

	if (pdev == NULL) {
		chg_err("oplus_chg_state_retention_probe input pdev error\n");
		return -ENODEV;
	}
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == FACTORY) {
		chg_info("FACTORY MODE not support retention_chg\n");
		return 0;
	}
#endif
	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_retention_charge), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc retention_chg buffer error\n");
		return 0;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	oplus_retention_chip_init(chip);

	oplus_mms_wait_topic("common", oplus_retention_chg_common_topic_ready, chip);
	INIT_DELAYED_WORK(&chip->update_work, oplus_retention_update_work);
	INIT_DELAYED_WORK(&chip->online_check_work, oplus_retention_online_check_work);
	INIT_DELAYED_WORK(&chip->present_check_work, oplus_retention_present_check_work);
	INIT_DELAYED_WORK(&chip->wired_type_change_work, oplus_retention_wired_type_change_work);

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_retention_charge_remove(struct platform_device *pdev)
#else
static int oplus_retention_charge_remove(struct platform_device *pdev)
#endif
{
	struct oplus_retention_charge *chip = platform_get_drvdata(pdev);

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == FACTORY) {
		chg_info("FACTORY MODE not support retention_chg.\n");
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
		return 0;
#else
		return;
#endif
	}
#endif
	if (!IS_ERR_OR_NULL(chip->wired_subs))
		oplus_mms_unsubscribe(chip->wired_subs);
	devm_kfree(&pdev->dev, chip);
	kfree(retention);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_retention_charge_match[] = {
	{ .compatible = "oplus,retention_charge" },
	{},
};

static struct platform_driver oplus_retention_charge_driver = {
	.driver		= {
		.name = "oplus-retention_charge",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_retention_charge_match),
	},
	.probe		= oplus_chg_state_retention_probe,
	.remove		= oplus_retention_charge_remove,
};


static __init int oplus_retention_charge_init(void)
{
	return platform_driver_register(&oplus_retention_charge_driver);
}

static __exit void oplus_retention_charge_exit(void)
{
	struct oplus_chg_retention_desc *desc, *tmp;

	mutex_lock(&list_lock);
	list_for_each_entry_safe(desc, tmp, &retention_list, list) {
		list_del_init(&desc->list);
	}
	mutex_unlock(&list_lock);

	platform_driver_unregister(&oplus_retention_charge_driver);
}

oplus_chg_module_register(oplus_retention_charge);

