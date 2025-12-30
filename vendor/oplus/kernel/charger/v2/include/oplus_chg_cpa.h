// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */

#ifndef __OPLUS_CHG_CPA_H__
#define __OPLUS_CHG_CPA_H__

enum oplus_chg_protocol_type {
	CHG_PROTOCOL_INVALID = -1,
	CHG_PROTOCOL_BC12 = 0,
	CHG_PROTOCOL_PD,
	CHG_PROTOCOL_PPS,
	CHG_PROTOCOL_VOOC,
	CHG_PROTOCOL_UFCS,
	CHG_PROTOCOL_QC,
	CHG_PROTOCOL_MAX,
};

enum cpa_topic_item {
	CPA_ITEM_CHG_TYPE,
	CPA_ITEM_ALLOW,
	CPA_ITEM_TIMEOUT,
	CPA_ITEM_POWER,
};

struct protocol_map {
	uint32_t protocol;
	int type[CHG_PROTOCOL_MAX];
};

const char *get_protocol_name_str(enum oplus_chg_protocol_type type);
bool oplus_cpa_support(void);
int oplus_cpa_request(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
int oplus_cpa_protocol_ready(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
int oplus_cpa_switch_start(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
int oplus_cpa_switch_end(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
int oplus_cpa_get_high_prio_wired_type(struct oplus_mms *topic, struct protocol_map *map);
enum oplus_chg_protocol_type oplus_cpa_curr_high_prio_protocol_type(struct oplus_mms *topic);
void oplus_cpa_protocol_add_type(struct protocol_map *map, int type);
int oplus_cpa_protocol_set_max_power(struct oplus_mms *topic, enum oplus_chg_protocol_type type, int power_mw);
int oplus_cpa_protocol_restore_max_power(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
int oplus_cpa_protocol_restore_max_power_all(struct oplus_mms *topic);
int oplus_cpa_protocol_set_power(struct oplus_mms *topic, enum oplus_chg_protocol_type type, int power_mw);
int oplus_cpa_protocol_clear_power(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
int oplus_cpa_protocol_clear_power_all(struct oplus_mms *topic);
int oplus_cpa_protocol_get_power(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
int oplus_cpa_protocol_get_max_power(struct oplus_mms *topic);
int oplus_cpa_protocol_get_max_power_by_type(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
int oplus_cpa_request_lock(struct oplus_mms *topic, const char *name);
int oplus_cpa_request_unlock(struct oplus_mms *topic, const char *name);
int oplus_cpa_protocol_disable(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
int oplus_cpa_protocol_enable(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
int oplus_cpa_protocol_enable_all(struct oplus_mms *topic);
int oplus_cpa_get_actual_used_power(struct oplus_mms *topic);
bool oplus_cpa_protocol_check_enable(struct oplus_mms *topic, enum oplus_chg_protocol_type type);
#endif /* __OPLUS_CHG_CPA_H__ */
