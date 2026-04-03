/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#ifndef __OPLUS_CHG_DUAL_CELLS_PROTECTION_H__
#define __OPLUS_CHG_DUAL_CELLS_PROTECTION_H__
#include <oplus_mms.h>

#define DUAL_CELLS_PROTECTION_CUR_LIMIT_DEFAULT 2500

enum protection_topic_item {
	DUAL_CELLS_BATT_STATUS,
};

struct dual_cells_protect_track_info {
	int reason;
	int batt_status;
	int batt_cc;
	int batt_fcc;
	int soc;
	int pre_soc;
	int plugin_soc;
	int batt_intfcc;
	int xvdd_occur;
	int sn_change;
};

enum dual_cells_protect_reason {
	PROTECT_UNKNOWN,
	FCC_SMALL,
	INTFCC_SMALL,
	SOC_JUMP,
	FCC_RECOVER,
	FCC_RECOVERING,
	INTFCC_RECOVER,
	SN_CHANGE,
	XVDD_OCCUR,
	REASON_MAX,
};

int oplus_chg_get_dual_cells_batt_health(struct oplus_mms *topic, int *status, int *reason);
void oplus_chg_set_dual_cells_batt_health(struct oplus_mms *topic, int val);

void oplus_chg_set_dual_cells_protect_track_debug(struct oplus_mms *topic, int val);
int oplus_chg_get_dual_cells_protect_track_debug(struct oplus_mms *topic);

const char *get_protect_reason_str(enum dual_cells_protect_reason type);
#endif /* __OPLUS_CHG_DUAL_CELLS_PROTECTION_H__ */
