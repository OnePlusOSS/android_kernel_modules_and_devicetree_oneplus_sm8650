// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2021 Oplus. All rights reserved.
 */

#ifndef __OPLUS_STRATEGY_H__
#define __OPLUS_STRATEGY_H__

#include <linux/list.h>
#include <linux/device.h>

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include <oplus_cfg.h>
#endif

struct oplus_chg_strategy;

enum {
	STRATEGY_USE_BATT_TEMP = 0,
	STRATEGY_USE_SHELL_TEMP,
	STRATEGY_USE_MAIN_BATT_TEMP,
	STRATEGY_USE_SUB_BATT_TEMP,
};

struct oplus_chg_strategy_desc {
	const char *name;
	struct list_head list;

	struct oplus_chg_strategy *(*strategy_alloc)(unsigned char *buf,
						     size_t size);
	struct oplus_chg_strategy *(*strategy_alloc_by_node)(struct device_node *node);
#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	struct oplus_chg_strategy *(*strategy_alloc_by_param_head)(const char *node_name, struct oplus_param_head *head);
#endif
	int (*strategy_release)(struct oplus_chg_strategy *strategy);
	int (*strategy_init)(struct oplus_chg_strategy *strategy);
	int (*strategy_get_data)(struct oplus_chg_strategy *strategy, void *ret);
	int (*strategy_set_process_data)(struct oplus_chg_strategy *strategy, const char *type, unsigned long arg);
	int (*strategy_get_metadata)(struct oplus_chg_strategy *strategy, void *ret);
	int (*strategy_get_custom_data)(struct oplus_chg_strategy *strategy, const char *type, void *ret);
};

struct oplus_chg_strategy {
	struct oplus_chg_strategy_desc *desc;
	bool initialized;
};

struct oplus_chg_strategy *
oplus_chg_strategy_alloc(const char *name, unsigned char *buf, size_t size);
struct oplus_chg_strategy *
oplus_chg_strategy_alloc_by_node(const char *name, struct device_node *node);
#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
struct oplus_chg_strategy *
oplus_chg_strategy_alloc_by_param_head(const char *name, const char *node_name, struct oplus_param_head *head);
#endif
int oplus_chg_strategy_init(struct oplus_chg_strategy *strategy);
int oplus_chg_strategy_release(struct oplus_chg_strategy *strategy);
int oplus_chg_strategy_get_data(struct oplus_chg_strategy *strategy, void *ret);
int oplus_chg_strategy_set_process_data(struct oplus_chg_strategy *strategy, const char *type, unsigned long arg);
int oplus_chg_strategy_get_custom_data(struct oplus_chg_strategy *strategy, const char *type, void *ret);
int oplus_chg_strategy_get_metadata(struct oplus_chg_strategy *strategy, void *ret);
int oplus_chg_strategy_register(struct oplus_chg_strategy_desc *desc);
int oplus_chg_strategy_read_data(struct device *dev,
				 const char *prop_str, uint8_t **buf);

/* pps ufcs curve strategy*/

struct puc_strategy_data {
	int32_t target_vbus;
	int32_t target_vbat;
	int32_t target_ibus;
	union {
		uint32_t flags;
		struct {
			uint32_t exit          : 1;
			uint32_t support_cv    : 1;
			uint32_t reserve_flags : 30;
		};
	};
	int32_t target_time;
} __attribute__((packed));

struct puc_strategy_temp_curves {
	struct puc_strategy_data *data;
	int num;
};

struct puc_strategy_ret_data {
	int32_t target_vbus;
	int32_t target_vbat;
	int32_t target_ibus;
	int index;
	bool last_gear;
	bool exit;
	bool support_cv;
	int32_t iterm;
};

struct ddrc_strategy_data {
	int32_t count;
	int32_t vbat0;
	int32_t vbat1;
	int32_t index;
} __attribute__((packed));

struct ddrc_temp_curves {
	struct ddrc_strategy_data *data;
	int index_r;
	int index_t;
	int num;
};
#endif /* __OPLUS_STRATEGY_H__ */
