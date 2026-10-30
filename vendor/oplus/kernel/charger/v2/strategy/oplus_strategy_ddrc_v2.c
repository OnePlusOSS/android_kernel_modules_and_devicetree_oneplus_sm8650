// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[STRATEGY_DDRC_V2]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/of.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_strategy.h>

enum ddrc_ratio_range {
	DDRC_RATIO_RANGE_MIN = 0,
	DDRC_RATIO_RANGE_LOW,
	DDRC_RATIO_RANGE_MID_LOW,
	DDRC_RATIO_RANGE_MID,
	DDRC_RATIO_RANGE_MID_HIGH,
	DDRC_RATIO_RANGE_HIGH,
	DDRC_RATIO_RANGE_MAX = DDRC_RATIO_RANGE_HIGH,
};

struct ddrc_ratio_curves {
	struct ddrc_temp_curves *temp_curves;
};

#define NAME_LENGTH	16
struct ddrc_strategy {
	struct oplus_chg_strategy strategy;
	struct ddrc_temp_curves *curve;
	struct ddrc_ratio_curves ratio_curves[DDRC_RATIO_RANGE_MAX + 1];
	uint32_t ratio_range_data[DDRC_RATIO_RANGE_MAX];
	int32_t *temp_range_data;
	uint32_t temp_range_num;
	uint32_t temp_type;
	int curr_level;
	int table_max_vterm;
	int table_max_vshut;
	char topic_name[NAME_LENGTH];
	struct oplus_mms *topic;
};

#define DDRC_DATA_SIZE	sizeof(struct ddrc_strategy_data)

static const char * const ddrc_strategy_ratio[] = {
	[DDRC_RATIO_RANGE_MIN]		= "strategy_ratio_range_min",
	[DDRC_RATIO_RANGE_LOW]		= "strategy_ratio_range_low",
	[DDRC_RATIO_RANGE_MID_LOW]	= "strategy_ratio_range_mid_low",
	[DDRC_RATIO_RANGE_MID]		= "strategy_ratio_range_mid",
	[DDRC_RATIO_RANGE_MID_HIGH]	= "strategy_ratio_range_mid_high",
	[DDRC_RATIO_RANGE_HIGH]		= "strategy_ratio_range_high",
};

static struct oplus_mms *comm_topic;
static struct oplus_mms *gauge_topic;

__maybe_unused static bool is_comm_topic_available(void)
{
	if (!comm_topic)
		comm_topic = oplus_mms_get_by_name("common");
	return !!comm_topic;
}

__maybe_unused static bool is_gauge_topic_available(void)
{
	if (!gauge_topic)
		gauge_topic = oplus_mms_get_by_name("gauge");
	return !!gauge_topic;
}

__maybe_unused static bool is_gauge_topic_available_by_ddrc(struct ddrc_strategy *ddrc)
{
	if (!ddrc->topic)
		ddrc->topic = oplus_mms_get_by_name(ddrc->topic_name);
	return !!ddrc->topic;
}

static int __read_signed_data_from_node(struct device_node *node,
					const char *prop_str,
					s32 **addr, u32 *len)
{
	int rc = 0, length;

	if (!node || !prop_str || !addr || !len) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(s32));
	if (rc < 0) {
		chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;
	*addr = kzalloc(sizeof(int32_t) * length, GFP_KERNEL);
	if (!*addr) {
		return -ENOMEM;
	}
	rc = of_property_read_u32_array(node, prop_str, (u32 *)*addr, length);
	if (rc) {
		chg_err("Read %s failed, rc=%d\n", prop_str, rc);
		goto error;
	}
	*len = length;

	return rc;

error:
	kfree(*addr);
	*addr = NULL;
	return rc;
}

static int __read_unsigned_data_from_node(struct device_node *node,
					  const char *prop_str, u32 *addr,
					  int len_max)
{
	int rc = 0, length;

	if (!node || !prop_str || !addr) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u32));
	if (rc < 0) {
		chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;

	if (length != len_max) {
		chg_err("entries(%d) num error, only %d allowed\n", length,
			len_max);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(node, prop_str, (u32 *)addr, length);
	if (rc < 0) {
		chg_err("Read %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	return length;
}

static int ddrc_strategy_get_ratio(struct ddrc_strategy *ddrc, int *ratio)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!is_gauge_topic_available_by_ddrc(ddrc)) {
		chg_err("gauge topic not found\n");
		return -ENODEV;
	}
	rc = oplus_mms_get_item_data(ddrc->topic, GAUGE_ITEM_RATIO_VALUE,
				     &data, true);
	if (rc < 0) {
		chg_err("can't get ratio, rc=%d\n", rc);
		return rc;
	}
	*ratio = data.intval;

	return 0;
}

static int ddrc_strategy_get_temp(struct ddrc_strategy *ddrc, int *temp)
{
	union mms_msg_data data = { 0 };
	int rc;

	switch (ddrc->temp_type) {
	case STRATEGY_USE_BATT_TEMP:
		if (!is_gauge_topic_available()) {
			chg_err("gauge topic not found\n");
			return -ENODEV;
		}
		rc = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_TEMP,
					     &data, true);
		if (rc < 0) {
			chg_err("can't get battery temp, rc=%d\n", rc);
			return rc;
		}

		*temp = data.intval;
		break;
	case STRATEGY_USE_SHELL_TEMP:
		if (!is_comm_topic_available()) {
			chg_err("common topic not found\n");
			return -ENODEV;
		}
		rc = oplus_mms_get_item_data(comm_topic, COMM_ITEM_SHELL_TEMP,
					     &data, false);
		if (rc < 0) {
			chg_err("can't get shell temp, rc=%d\n", rc);
			return rc;
		}

		*temp = data.intval;
		break;
	default:
		chg_err("not support temp type, type=%d\n", ddrc->temp_type);
		return -EINVAL;
	}

	return 0;
}

static enum ddrc_ratio_range
ddrc_get_ratio_region(struct ddrc_strategy *ddrc)
{
	int ratio;
	enum ddrc_ratio_range ratio_region = DDRC_RATIO_RANGE_MAX;
	int i;
	int rc;

	rc = ddrc_strategy_get_ratio(ddrc, &ratio);
	if (rc < 0) {
		chg_err("can't get ratio, rc=%d\n", rc);
		return DDRC_RATIO_RANGE_MAX;
	}

	for (i = 0; i < DDRC_RATIO_RANGE_MAX; i++) {
		if (ratio < ddrc->ratio_range_data[i]) {
			ratio_region = i;
			break;
		}
	}
	return ratio_region;
}

static int ddrc_get_temp_region(struct ddrc_strategy *ddrc)
{
	int temp, i, rc;
	int temp_region = 0;
	union mms_msg_data data = { 0 };

	if (!ddrc->temp_range_num)
		return 0;

	temp_region = ddrc->temp_range_num;
	if (!is_gauge_topic_available()) {
		chg_err("gauge topic not found\n");
		return temp_region;
	}
	rc = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_RATIO_TRANGE,
				     &data, true);

	chg_err("region=%d\n", data.intval);
	if (rc < 0 || data.intval < 0 || data.intval > ddrc->temp_range_num) {
		rc = ddrc_strategy_get_temp(ddrc, &temp);
		if (rc < 0) {
			chg_err("can't get temp, rc=%d\n", rc);
			return ddrc->temp_range_num;
		}

		for (i = 0; i < ddrc->temp_range_num; i++) {
			if (temp <= ddrc->temp_range_data[i]) {
				temp_region = i;
				break;
			}
		}
		return temp_region;
	}

	temp_region = data.intval;

	return temp_region;
}

static struct oplus_chg_strategy *
ddrc_strategy_alloc(unsigned char *buf, size_t size)
{
	return ERR_PTR(-ENOTSUPP);
}

static void ddrc_strategy_get_topic_name(struct ddrc_strategy *ddrc, struct device_node *node)
{
	int rc;
	const char *topic_name;

	rc = of_property_read_string(node, "oplus,gauge_topic_name", &topic_name);
	if (rc < 0) {
		snprintf(ddrc->topic_name, sizeof(ddrc->topic_name), "gauge");
	} else {
		snprintf(ddrc->topic_name, sizeof(ddrc->topic_name), "%s", topic_name);
		chg_info("ddrc gauge topic name: %s\n", ddrc->topic_name);
	}
}

static void ddrc_strategy_get_temp_type(struct ddrc_strategy *ddrc, struct device_node *node)
{
	int rc;
	u32 data;

	rc = of_property_read_u32(node, "oplus,temp_type", &data);
	if (rc < 0) {
		chg_err("oplus,temp_type reading failed, rc=%d\n", rc);
		ddrc->temp_type = STRATEGY_USE_SHELL_TEMP;
	} else {
		ddrc->temp_type = (uint32_t)data;
	}
}

static int ddrc_strategy_read_data(struct ddrc_strategy *ddrc, struct device_node *node)
{
	int rc;

	rc = __read_unsigned_data_from_node(node, "oplus,ratio_range",
						    (u32 *)ddrc->ratio_range_data,
						    DDRC_RATIO_RANGE_MAX);
	if (rc < 0) {
		chg_err("get oplus,ratio_range property error, rc=%d\n", rc);
		return rc;
	}
	rc = __read_signed_data_from_node(node, "oplus,temp_range",
					  &ddrc->temp_range_data,
					  &ddrc->temp_range_num);
	if (rc < 0) {
		chg_err("get oplus,temp_range property error, rc=%d\n", rc);
		return rc;
	}
	return rc;
}

#define STN_NAME_LENGTH 32
static int __ddrc_strategy_get_temp_curves(
	struct ddrc_strategy *ddrc, struct device_node *soc_node, int i, int j)
{
	int rc;
	char strategy_temp_name[STN_NAME_LENGTH] = {0};
	int length;

	snprintf(strategy_temp_name, sizeof(strategy_temp_name), "strategy_temp_%d", j);
	length = of_property_count_elems_of_size(
		soc_node, strategy_temp_name, sizeof(u32));
	if (length < 0) {
		chg_err("can't find %s property, rc=%d\n",
			strategy_temp_name, length);
		rc = -EINVAL;
		goto err;
	}
	rc = length * sizeof(u32);
	if (rc % DDRC_DATA_SIZE != 0) {
		chg_err("buf size does not meet the requirements, size=%d\n", rc);
		rc = -EINVAL;
		goto err;
	}

	ddrc->ratio_curves[i].temp_curves[j].num = rc / DDRC_DATA_SIZE;
	ddrc->ratio_curves[i].temp_curves[j].data = kzalloc(rc , GFP_KERNEL);
	if (ddrc->ratio_curves[i].temp_curves[j].data == NULL) {
		chg_err("alloc strategy data memory error\n");
		rc = -ENOMEM;
		goto err;
	}

	rc = of_property_read_u32_array(
			soc_node, strategy_temp_name,
			(u32 *)ddrc->ratio_curves[i].temp_curves[j].data,
			length);
	if (rc < 0) {
		chg_err("read %s property error, rc=%d\n",
			strategy_temp_name, rc);
		goto err;
	}
err:
	return rc;
}
static int ddrc_strategy_get_temp_curves(struct ddrc_strategy *ddrc, struct device_node *node)
{
	int rc;
	struct device_node *soc_node;
	int i, j;
	int temp_num;
	struct property *prop;

	for (i = 0; i <= DDRC_RATIO_RANGE_MAX; i++) {
		soc_node = of_get_child_by_name(node, ddrc_strategy_ratio[i]);
		if (!soc_node) {
			chg_err("can't find %s node\n", ddrc_strategy_ratio[i]);
			rc = -ENODEV;
			goto get_temp_curves_err;
		}
		ddrc->ratio_curves[i].temp_curves =
			kzalloc(sizeof(struct ddrc_temp_curves) * (ddrc->temp_range_num + 1), GFP_KERNEL);
		temp_num = 0;
		for_each_property_of_node(soc_node, prop) {
			if (strncmp(prop->name, "strategy_temp_", strlen("strategy_temp_")) == 0)
				temp_num++;
		}
		if (temp_num != ddrc->temp_range_num + 1) {
			chg_err("err strategy_temp num %d %d\n", temp_num, ddrc->temp_range_num);
			rc = -EINVAL;
			goto get_temp_curves_err;
		}
		for (j = 0; j < temp_num; j++) {
			rc = __ddrc_strategy_get_temp_curves(ddrc, soc_node, i, j);
			if (rc < 0)
				goto get_temp_curves_err;
		}
	}
get_temp_curves_err:
	return rc;
}

static void ddrc_strategy_free_temp_curves(struct ddrc_strategy *ddrc)
{
	int i, j;

	for (i = 0; i <= DDRC_RATIO_RANGE_MAX; i++) {
		for (j = 0; j <= ddrc->temp_range_num; j++) {
			if (ddrc->ratio_curves[i].temp_curves && ddrc->ratio_curves[i].temp_curves[j].data != NULL) {
				kfree(ddrc->ratio_curves[i].temp_curves[j].data);
				ddrc->ratio_curves[i].temp_curves[j].data = NULL;
			}
		}
		if (ddrc->ratio_curves[i].temp_curves)
			kfree(ddrc->ratio_curves[i].temp_curves);
	}
}

static void ddrc_strategy_fill_table_max_vterm(struct ddrc_strategy *ddrc)
{
	int i, j, k;
	struct ddrc_temp_curves *tc;
	int max_vterm = -ENOENT;
	int max_vshut = -ENOENT;

	for (i = 0; i <= DDRC_RATIO_RANGE_MAX; i++) {
		if (!ddrc->ratio_curves[i].temp_curves)
			continue;
		for (j = 0; j <= ddrc->temp_range_num; j++) {
			tc = &ddrc->ratio_curves[i].temp_curves[j];

			if (!tc->data || tc->num <= 0)
				continue;
			for (k = 0; k < tc->num; k++) {
				max_vterm = max(max_vterm, tc->data[k].vbat1);
				max_vshut = max(max_vshut, tc->data[k].vbat0);
			}
		}
	}

	ddrc->table_max_vterm = max_vterm;
	ddrc->table_max_vshut = max_vshut;
	chg_info("ddrc table max vterm %d mV, max vshut %d mV\n",
		 ddrc->table_max_vterm, ddrc->table_max_vshut);
}

static struct oplus_chg_strategy *
ddrc_strategy_alloc_by_node(struct device_node *node)
{
	struct ddrc_strategy *ddrc;
	int rc;

	if (node == NULL) {
		chg_err("node is NULL\n");
		return ERR_PTR(-EINVAL);
	}

	ddrc = kzalloc(sizeof(struct ddrc_strategy), GFP_KERNEL);
	if (ddrc == NULL) {
		chg_err("alloc strategy memory error\n");
		return ERR_PTR(-ENOMEM);
	}

	memset(ddrc->topic_name, 0, sizeof(ddrc->topic_name));
	ddrc->temp_range_num = 0;
	ddrc_strategy_get_topic_name(ddrc, node);
	ddrc_strategy_get_temp_type(ddrc, node);

	rc = ddrc_strategy_read_data(ddrc, node);
	if (rc < 0) {
		goto base_info_err;
	}

	chg_err("temp_range_num=%d\n", ddrc->temp_range_num);
	rc = ddrc_strategy_get_temp_curves(ddrc, node);
	if (rc == -ENODEV)
		goto base_info_err;
	else if (rc < 0)
		goto data_err;
	ddrc_strategy_fill_table_max_vterm(ddrc);
	return (struct oplus_chg_strategy *)ddrc;

data_err:
	ddrc_strategy_free_temp_curves(ddrc);
base_info_err:
	if (ddrc->temp_range_data)
		kfree(ddrc->temp_range_data);
	kfree(ddrc);
	return ERR_PTR(rc);
}

static int ddrc_strategy_release(struct oplus_chg_strategy *strategy)
{
	struct ddrc_strategy *ddrc;
	int i, j;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	ddrc = (struct ddrc_strategy *)strategy;

	for (i = 0; i <= DDRC_RATIO_RANGE_MAX; i++) {
		for (j = 0; j <= ddrc->temp_range_num; j++) {
			if (ddrc->ratio_curves[i].temp_curves[j].data != NULL) {
				kfree(ddrc->ratio_curves[i].temp_curves[j].data);
				ddrc->ratio_curves[i].temp_curves[j].data = NULL;
			}
		}
		if (ddrc->ratio_curves[i].temp_curves)
			kfree(ddrc->ratio_curves[i].temp_curves);
	}

	if (ddrc->temp_range_data)
		kfree(ddrc->temp_range_data);
	kfree(ddrc);

	return 0;
}

static int ddrc_strategy_init(struct oplus_chg_strategy *strategy)
{
	struct ddrc_strategy *ddrc;
	int temp_range;
	enum ddrc_ratio_range ratio_range;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	ddrc = (struct ddrc_strategy *)strategy;

	ratio_range = ddrc_get_ratio_region(ddrc);
	temp_range = ddrc_get_temp_region(ddrc);
	ddrc->curve = &ddrc->ratio_curves[ratio_range].temp_curves[temp_range];
	ddrc->curve->index_r = ratio_range;
	ddrc->curve->index_t = temp_range;

	chg_info("use %s:%d curve\n", ddrc_strategy_ratio[ratio_range], temp_range);

	return 0;
}

static int ddrc_strategy_get_data(struct oplus_chg_strategy *strategy, void *ret)
{
	struct ddrc_strategy *ddrc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (ret == NULL) {
		chg_err("ret is NULL\n");
		return -EINVAL;
	}

	ddrc = (struct ddrc_strategy *)strategy;

	memcpy(ret, ddrc->curve, sizeof(*ddrc->curve));
	return 0;
}

static int ddrc_strategy_get_metadata(struct oplus_chg_strategy *strategy, void *ret)
{
	struct ddrc_strategy *ddrc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (ret == NULL) {
		chg_err("ret is NULL\n");
		return -EINVAL;
	}

	ddrc = (struct ddrc_strategy *)strategy;

	memcpy(ret, ddrc->curve, sizeof(*ddrc->curve));
	return 0;
}

static int ddrc_strategy_get_custom_data(struct oplus_chg_strategy *strategy,
					  const char *type, void *ret)
{
	struct ddrc_strategy *ddrc;

	if (strategy == NULL || type == NULL || ret == NULL) {
		chg_err("invalid parameters\n");
		return -EINVAL;
	}

	ddrc = (struct ddrc_strategy *)strategy;

	if (sysfs_streq(type, OPLUS_DDRC_STRATEGY_CUSTOM_MAX_VTERM)) {
		*(int *)ret = ddrc->table_max_vterm;
		return 0;
	}
	if (sysfs_streq(type, OPLUS_DDRC_STRATEGY_CUSTOM_MAX_VSHUT)) {
		*(int *)ret = ddrc->table_max_vshut;
		return 0;
	}

	return -ENOTSUPP;
}

static struct oplus_chg_strategy_desc ddrc_strategy_desc = {
	.name = "ddrc_curve_v2",
	.strategy_init = ddrc_strategy_init,
	.strategy_release = ddrc_strategy_release,
	.strategy_alloc = ddrc_strategy_alloc,
	.strategy_alloc_by_node = ddrc_strategy_alloc_by_node,
	.strategy_get_data = ddrc_strategy_get_data,
	.strategy_get_metadata = ddrc_strategy_get_metadata,
	.strategy_get_custom_data = ddrc_strategy_get_custom_data,
};

int ddrc_v2_strategy_register(void)
{
	return oplus_chg_strategy_register(&ddrc_strategy_desc);
}

