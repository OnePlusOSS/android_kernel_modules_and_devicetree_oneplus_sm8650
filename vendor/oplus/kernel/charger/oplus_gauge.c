// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include "oplus_gauge.h"
#include "oplus_charger.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/oplus_project.h>
#endif
#include "charger_ic/oplus_switching.h"
#if __and(IS_MODULE(CONFIG_OPLUS_CHG), IS_MODULE(CONFIG_OPLUS_CHG_V2))
#include "oplus_chg_symbol.h"
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
extern struct oplus_gauge_chip* oplus_mtk_gauge_init(void);
#endif
static struct oplus_gauge_chip *g_gauge_chip = NULL;
static struct oplus_gauge_chip *g_sub_gauge_chip = NULL;
static struct oplus_plat_gauge_operations *g_plat_gauge_ops = NULL;
static struct oplus_external_auth_chip *g_external_auth_chip = NULL;

int gauge_dbg_tbat = 0;
module_param(gauge_dbg_tbat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_tbat, "debug battery temperature");

static int gauge_dbg_vbat = 0;
module_param(gauge_dbg_vbat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_vbat, "debug battery voltage");

static int gauge_dbg_ibat = 0;
module_param(gauge_dbg_ibat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_ibat, "debug battery current");

int sub_gauge_dbg_tbat = 0;
module_param(sub_gauge_dbg_tbat, int, 0644);
MODULE_PARM_DESC(sub_gauge_dbg_tbat, "debug sub_battery temperature");

static int sub_gauge_dbg_vbat = 0;
module_param(sub_gauge_dbg_vbat, int, 0644);
MODULE_PARM_DESC(sub_gauge_dbg_vbat, "debug sub_battery voltage");

static int sub_gauge_dbg_ibat = 0;
module_param(sub_gauge_dbg_ibat, int, 0644);
MODULE_PARM_DESC(sub_gauge_dbg_ibat, "debug sub_battery current");

static int gauge_dbg_soc = 0;
module_param(gauge_dbg_soc, int, 0644);
MODULE_PARM_DESC(gauge_dbg_soc, "debug battery soc");

int oplus_plat_gauge_is_support(void)
{
	if (!g_plat_gauge_ops) {
		return 0;
	} else {
		return 1;
	}
}

int oplus_gauge_get_plat_batt_mvolts(void)
{
	if (!g_plat_gauge_ops) {
		return 0;
	} else {
		if (gauge_dbg_vbat != 0) {
			chg_err("[OPLUS_CHG]%s:debug enabled,voltage gauge_dbg_vbat[%d]\n", __func__, gauge_dbg_vbat);
			return gauge_dbg_vbat;
		}
		return g_plat_gauge_ops->get_plat_battery_mvolts();
	}
	return 0;
}

int oplus_gauge_get_plat_batt_current(void)
{
	if (!g_plat_gauge_ops) {
		return 0;
	} else {
		return g_plat_gauge_ops->get_plat_battery_current();
	}
	return 0;
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_batt_mvolts(void)
{
	if (!g_gauge_chip) {
		return 3800;
	} else {
		if (gauge_dbg_vbat != 0) {
			chg_err("[OPLUS_CHG]%s:debug enabled,voltage gauge_dbg_vbat[%d]\n", __func__, gauge_dbg_vbat);
			return gauge_dbg_vbat;
		}
		return g_gauge_chip->gauge_ops->get_battery_mvolts();
	}
}

int oplus_gauge_get_batt_fc(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_fc) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_fc();
	}
}

int oplus_gauge_get_batt_qm(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_qm) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_qm();
	}
}

int oplus_gauge_get_batt_pd(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_pd) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_pd();
	}
}

int oplus_gauge_get_batt_rcu(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_rcu) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_rcu();
	}
}

int oplus_gauge_get_batt_rcf(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_rcf) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_rcf();
	}
}

int oplus_gauge_get_batt_fcu(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_fcu) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_fcu();
	}
}

int oplus_gauge_get_batt_fcf(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_fcf) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_fcf();
	}
}

int oplus_gauge_get_batt_sou(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_sou) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_sou();
	}
}

int oplus_gauge_get_batt_do0(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_do0) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_do0();
	}
}

int oplus_gauge_get_batt_doe(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_doe) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_doe();
	}
}

int oplus_gauge_get_batt_trm(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_trm) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_trm();
	}
}

int oplus_gauge_get_batt_pc(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_pc) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_pc();
	}
}

int oplus_gauge_get_batt_qs(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_battery_qs) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_qs();
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_batt_mvolts_2cell_max(void)
{
	if (!g_gauge_chip) {
		return 3800;
	} else {
		if (gauge_dbg_vbat != 0) {
			chg_err("[OPLUS_CHG]%s:debug enabled,max voltage gauge_dbg_vbat[%d]\n", __func__, gauge_dbg_vbat);
			return gauge_dbg_vbat;
		}
		return g_gauge_chip->gauge_ops->get_battery_mvolts_2cell_max();
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_batt_mvolts_2cell_min(void)
{
	if (!g_gauge_chip) {
		return 3800;
	} else {
		if (gauge_dbg_vbat != 0) {
			chg_err("[OPLUS_CHG]%s:debug enabled,min voltage gauge_dbg_vbat[%d]\n", __func__, gauge_dbg_vbat);
			return gauge_dbg_vbat;
		}
		return g_gauge_chip->gauge_ops->get_battery_mvolts_2cell_min();
	}
}

int oplus_gauge_get_batt_temperature(void)
{
	int batt_temp = 0;
	if (!g_gauge_chip) {
		return 250;
	} else {
		if (gauge_dbg_tbat != 0 || sub_gauge_dbg_tbat != 0) {
			chg_err("[OPLUS_CHG]debug enabled, gauge_dbg_tbat[%d], "
				"sub_gauge_dbg_ibat[%d]\n",
				gauge_dbg_tbat, sub_gauge_dbg_tbat);
			return gauge_dbg_tbat != 0 ? gauge_dbg_tbat : sub_gauge_dbg_tbat;
		}
		batt_temp = g_gauge_chip->gauge_ops->get_battery_temperature();

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
		if (get_eng_version() == HIGH_TEMP_AGING || oplus_is_ptcrb_version()) {
			if (batt_temp > 690) {
                        	chg_err("[OPLUS_CHG]CONFIG_HIGH_TEMP_VERSION enable here,"
                                	"disable high tbat shutdown\n");
				batt_temp = 690;
			}
		}
#endif
		return batt_temp;
	}
}

int oplus_gauge_get_dec_fg_type(void)
{
	int type = 0;
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_dec_fg_type) {
		return DEC_CV_FG_UNKNOWN;
	} else {
		type = g_gauge_chip->gauge_ops->get_dec_fg_type();
		return type;
	}
}

int oplus_gauge_get_vct(void)
{
	int vct = 0;
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_fg_vct) {
		return 0;
	} else {
		vct = g_gauge_chip->gauge_ops->get_fg_vct();
		return vct;
	}
}

bool oplus_gauge_set_vct(int vct)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->set_fg_vct) {
		return true;
	} else {
		return g_gauge_chip->gauge_ops->set_fg_vct(vct);
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_batt_soc(void)
{
	int batt_soc;
	int sub_batt_soc;
	int soc;
	int soc_remainder;
	struct oplus_chg_chip *chip = oplus_chg_get_chg_struct();

	if (!g_gauge_chip) {
		return -1;
	} else {
		if (gauge_dbg_soc != 0) {
			chg_err("debug enabled, soc[%d]\n", gauge_dbg_soc);
			return gauge_dbg_soc;
		} else if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops ||
			   !g_sub_gauge_chip->gauge_ops->get_battery_soc) {
			return g_gauge_chip->gauge_ops->get_battery_soc();
		} else if (oplus_switching_support_parallel_chg() == PARALLEL_MOS_CTRL && chip && !chip->authenticate) {
			return g_gauge_chip->gauge_ops->get_battery_soc();
		} else {
			batt_soc = g_gauge_chip->gauge_ops->get_battery_soc();
			sub_batt_soc = oplus_gauge_get_sub_batt_soc();
			soc_remainder = (batt_soc * g_gauge_chip->capacity_pct +
					 sub_batt_soc * g_sub_gauge_chip->capacity_pct) %
					100;
			soc = (batt_soc * g_gauge_chip->capacity_pct + sub_batt_soc * g_sub_gauge_chip->capacity_pct) /
			      100;
			if (soc_remainder != 0) {
				soc = soc + 1;
			}
			chg_err("batt_soc:%d, sub_batt_soc:%d, soc_remainder:%d, soc:%d\n", batt_soc, sub_batt_soc,
				soc_remainder, soc);
			return soc;
		}
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_batt_current(void)
{
	if (!g_gauge_chip) {
		return 100;
	} else {
		if (gauge_dbg_ibat != 0) {
			chg_err("[OPLUS_CHG]debug enabled,current gauge_dbg_ibat[%d]\n", gauge_dbg_ibat);
			return gauge_dbg_ibat;
		}
		return g_gauge_chip->gauge_ops->get_average_current();
	}
}
EXPORT_SYMBOL(oplus_gauge_get_batt_current);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_sub_current(void)
{
	if (!g_gauge_chip) {
		return 100;
	} else {
		if (!g_gauge_chip->gauge_ops->get_sub_current) {
			return -1;
		} else {
			return g_gauge_chip->gauge_ops->get_sub_current();
		}
	}
}

int oplus_gauge_get_remaining_capacity(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops ||
		    !g_sub_gauge_chip->gauge_ops->get_batt_remaining_capacity) {
			return g_gauge_chip->gauge_ops->get_batt_remaining_capacity();
		} else {
			return g_gauge_chip->gauge_ops->get_batt_remaining_capacity() +
			       g_sub_gauge_chip->gauge_ops->get_batt_remaining_capacity();
		}
	}
}

int oplus_gauge_get_device_type(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		return g_gauge_chip->device_type;
	}
}

int oplus_gauge_get_device_type_for_vooc(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		return g_gauge_chip->device_type_for_vooc;
	}
}

int oplus_gauge_get_batt_fcc(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops ||
		    !g_sub_gauge_chip->gauge_ops->get_battery_fcc) {
			return g_gauge_chip->gauge_ops->get_battery_fcc();
		} else {
			return g_gauge_chip->gauge_ops->get_battery_fcc() +
			       g_sub_gauge_chip->gauge_ops->get_battery_fcc();
		}
	}
}

int oplus_gauge_get_batt_cc(void)
{
	int batt_cc;
	int sub_batt_cc;

	if (!g_gauge_chip) {
		return 0;
	} else {
		if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops || !g_sub_gauge_chip->gauge_ops->get_battery_cc) {
			return g_gauge_chip->gauge_ops->get_battery_cc();
		} else {
			batt_cc = g_gauge_chip->gauge_ops->get_battery_cc();
			sub_batt_cc = g_sub_gauge_chip->gauge_ops->get_battery_cc();
			if ((batt_cc > 0) && (sub_batt_cc > 0)) {
				return (((batt_cc * g_gauge_chip->capacity_pct) +
					 (sub_batt_cc * g_sub_gauge_chip->capacity_pct)) /
					100);
			} else if (batt_cc > 0) {
				return batt_cc;
			} else if (sub_batt_cc > 0) {
				return sub_batt_cc;
			} else {
				return 0;
			}
		}
	}
}

int oplus_gauge_get_batt_soh(void)
{
	int batt_soh = 0;
	int sub_batt_soh = 0;

	if (!g_gauge_chip) {
		return 0;
	} else {
		if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops ||
		    !g_sub_gauge_chip->gauge_ops->get_battery_soh) {
			return g_gauge_chip->gauge_ops->get_battery_soh();
		} else {
			batt_soh = g_gauge_chip->gauge_ops->get_battery_soh();
			sub_batt_soh = g_sub_gauge_chip->gauge_ops->get_battery_soh();
			if ((batt_soh > 0) && (sub_batt_soh > 0)) {
				return (((batt_soh * g_gauge_chip->capacity_pct) +
					 (sub_batt_soh * g_sub_gauge_chip->capacity_pct)) /
					100);
			} else if (batt_soh > 0) {
				return batt_soh;
			} else if (sub_batt_soh > 0) {
				return sub_batt_soh;
			} else {
				return 0;
			}
		}
	}
}

bool oplus_gauge_get_batt_hmac(void)
{
	if (!g_gauge_chip) {
		return false;
	} else if (!g_gauge_chip->gauge_ops->get_battery_hmac) {
		return true;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_hmac();
	}
}

bool oplus_gauge_get_batt_external_hmac(void)
{
	chg_err("[OPLUS_CHG]%s:oplus_gauge_get_batt_external_hmac \n", __func__);
	if (!g_external_auth_chip) {
		return false;
	}
	if (!g_external_auth_chip->get_external_auth_hmac) {
		return false;
	} else {
		return g_external_auth_chip->get_external_auth_hmac();
	}
}

int oplus_gauge_start_test_external_hmac(int count)
{
	chg_err("[OPLUS_CHG]%s:oplus_gauge_start_test_external_hmac \n", __func__);
	if (!g_external_auth_chip) {
		return false;
	} else {
		return g_external_auth_chip->start_test_external_hmac(count);
	}
}

int oplus_gauge_get_external_hmac_test_result(int *count_total, int *count_now, int *fail_count)
{
	chg_err("[OPLUS_CHG]%s:oplus_gauge_get_hmac_test_result \n", __func__);
	if (!g_external_auth_chip) {
		return false;
	} else {
		return g_external_auth_chip->get_hmac_test_result(count_total, count_now, fail_count);
	}
}

int oplus_gauge_get_external_hmac_status(int *status, int *fail_count, int *total_count, int *real_fail_count,
					 int *real_total_count)
{
	chg_err("[OPLUS_CHG]%s:oplus_gauge_get_hmac_status \n", __func__);
	if (!g_external_auth_chip) {
		return false;
	} else {
		return g_external_auth_chip->get_hmac_status(status, fail_count, total_count, real_fail_count,
							     real_total_count);
	}
}

bool oplus_gauge_get_batt_authenticate(void)
{
	if (!g_gauge_chip) {
		return false;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_authenticate();
	}
}

bool oplus_gauge_set_power_sel(int sel)
{
	if (NULL == g_gauge_chip || NULL == g_gauge_chip->gauge_ops ||
	    NULL == g_gauge_chip->gauge_ops->set_gauge_power_sel) {
		return false;
	} else {
		chg_err("set_gauge_power_sel: %d \n", sel);
		return g_gauge_chip->gauge_ops->set_gauge_power_sel(sel);
	}
}

void oplus_gauge_set_float_uv_ma(int iterm_ma, int float_volt_uv)
{
	if (g_gauge_chip) {
		g_gauge_chip->gauge_ops->set_float_uv_ma(iterm_ma, float_volt_uv);
	}
}

void oplus_gauge_set_batt_full(bool full)
{
	if (g_gauge_chip) {
		g_gauge_chip->gauge_ops->set_battery_full(full);
	}
}

bool oplus_gauge_check_chip_is_null(void)
{
	if (!g_gauge_chip) {
		return true;
	} else {
		return false;
	}
}

void oplus_gauge_init(struct oplus_gauge_chip *chip)
{
	g_gauge_chip = chip;
}
EXPORT_SYMBOL(oplus_gauge_init);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
void oplus_mtk_plat_gauge_init(void)
{
	g_gauge_chip = oplus_mtk_gauge_init();
}
#endif

void oplus_plat_gauge_init(struct oplus_plat_gauge_operations *ops)
{
	g_plat_gauge_ops = ops;
}

void oplus_external_auth_init(struct oplus_external_auth_chip *chip)
{
	g_external_auth_chip = chip;
}

void oplus_sub_gauge_init(struct oplus_gauge_chip *chip)
{
	g_sub_gauge_chip = chip;
}

int oplus_gauge_get_prev_batt_mvolts(void)
{
	if (!g_gauge_chip)
		return 3800;
	else {
		if (gauge_dbg_vbat != 0) {
			chg_err("[OPLUS_CHG]%s:debug enabled,voltage gauge_dbg_vbat[%d] \n", __func__, gauge_dbg_vbat);
			return gauge_dbg_vbat;
		}
		return g_gauge_chip->gauge_ops->get_prev_battery_mvolts();
	}
}

int oplus_gauge_get_sub_prev_batt_mvolts(void)
{
	if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops || !g_sub_gauge_chip->gauge_ops->get_prev_battery_mvolts) {
		return 3800;
	} else {
		if (sub_gauge_dbg_vbat != 0) {
			chg_err("[OPLUS_CHG]%s:debug enabled,voltage sub_gauge_dbg_vbat[%d] \n", __func__,
				sub_gauge_dbg_vbat);
			return sub_gauge_dbg_vbat;
		}
		return g_sub_gauge_chip->gauge_ops->get_prev_battery_mvolts();
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_prev_batt_mvolts_2cell_max(void)
{
	if (!g_gauge_chip)
		return 3800;
	else {
		if (gauge_dbg_vbat != 0) {
			chg_err("[OPLUS_CHG]%s: debug enabled,voltage gauge_dbg_vbat[%d] \n", __func__, gauge_dbg_vbat);
			return gauge_dbg_vbat;
		}
		return g_gauge_chip->gauge_ops->get_prev_battery_mvolts_2cell_max();
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_prev_batt_mvolts_2cell_min(void)
{
	if (!g_gauge_chip)
		return 3800;
	else {
		if (gauge_dbg_vbat != 0) {
			chg_err("[OPLUS_CHG]%s:debug enabled,voltage gauge_dbg_vbat[%d] \n", __func__, gauge_dbg_vbat);
			return gauge_dbg_vbat;
		}
		return g_gauge_chip->gauge_ops->get_prev_battery_mvolts_2cell_min();
	}
}

int oplus_gauge_get_prev_batt_temperature(void)
{
	int batt_temp = 0;
	if (!g_gauge_chip)
		return 250;
	else {
		if (gauge_dbg_tbat != 0) {
			chg_err("[OPLUS_CHG]%s: debug enabled, gauge_dbg_tbat[%d] \n", __func__, gauge_dbg_tbat);
			return gauge_dbg_tbat;
		}
		batt_temp = g_gauge_chip->gauge_ops->get_prev_battery_temperature();

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
		if (get_eng_version() == HIGH_TEMP_AGING || oplus_is_ptcrb_version()) {
			if (batt_temp > 690) {
				chg_err("[OPLUS_CHG]CONFIG_HIGH_TEMP_VERSION enable here,"
					"disable high tbat shutdown \n");
				batt_temp = 690;
			}
		}
#endif
		return batt_temp;
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_prev_batt_soc(void)
{
	if (!g_gauge_chip)
		return 50;
	else
		return g_gauge_chip->gauge_ops->get_prev_battery_soc();
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_prev_batt_current(void)
{
	if (!g_gauge_chip)
		return 100;
	else {
		if (gauge_dbg_ibat != 0) {
			chg_err("[OPLUS_CHG]%s:debug enabled,current gauge_dbg_ibat[%d] \n", __func__, gauge_dbg_ibat);
			return gauge_dbg_ibat;
		}
		return g_gauge_chip->gauge_ops->get_prev_average_current();
	}
}

int oplus_gauge_get_prev_remaining_capacity(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops ||
		    !g_sub_gauge_chip->gauge_ops->get_prev_batt_remaining_capacity) {
			return g_gauge_chip->gauge_ops->get_prev_batt_remaining_capacity();
		} else {
			chg_err("gauge pre rm = %d sub pre rm = %d\n",
				g_gauge_chip->gauge_ops->get_prev_batt_remaining_capacity(),
				g_sub_gauge_chip->gauge_ops->get_prev_batt_remaining_capacity());
			return g_gauge_chip->gauge_ops->get_prev_batt_remaining_capacity() +
			       g_sub_gauge_chip->gauge_ops->get_prev_batt_remaining_capacity();
		}
	}
}

int oplus_gauge_update_battery_dod0(void)
{
	if (!g_gauge_chip)
		return 0;
	else
		return g_gauge_chip->gauge_ops->update_battery_dod0();
}

int oplus_gauge_update_soc_smooth_parameter(void)
{
	if (!g_gauge_chip)
		return 0;
	else {
		if (oplus_switching_support_parallel_chg()) {
			if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops ||
			    !g_sub_gauge_chip->gauge_ops->update_soc_smooth_parameter) {
				g_sub_gauge_chip->gauge_ops->update_soc_smooth_parameter();
			}
		}
		return g_gauge_chip->gauge_ops->update_soc_smooth_parameter();
	}
}

int oplus_gauge_get_battery_cb_status(void)
{
	if (!g_gauge_chip)
		return 0;
	else
		return g_gauge_chip->gauge_ops->get_battery_cb_status();
}

int oplus_gauge_get_i2c_err(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_gauge_i2c_err) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_gauge_i2c_err();
	}
}

void oplus_gauge_clear_i2c_err(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->clear_gauge_i2c_err) {
		return;
	} else {
		return g_gauge_chip->gauge_ops->clear_gauge_i2c_err();
	}
}

int oplus_gauge_get_prev_batt_fcc(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_prev_batt_fcc) {
		return 0;
	} else {
		if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops ||
		    !g_sub_gauge_chip->gauge_ops->get_prev_batt_fcc) {
			return g_gauge_chip->gauge_ops->get_prev_batt_fcc();
		} else {
			chg_err("gauge pre fcc = %d sub pre fcc = %d\n", g_gauge_chip->gauge_ops->get_prev_batt_fcc(),
				g_sub_gauge_chip->gauge_ops->get_prev_batt_fcc());
			return g_gauge_chip->gauge_ops->get_prev_batt_fcc() +
			       g_sub_gauge_chip->gauge_ops->get_prev_batt_fcc();
		}
	}
}

int oplus_gauge_protect_check(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->protect_check) {
			return g_gauge_chip->gauge_ops->protect_check();
		}
		return 0;
	}
}

bool oplus_gauge_afi_update_done(void)
{
	if (!g_gauge_chip) {
		return false;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->afi_update_done) {
			return g_gauge_chip->gauge_ops->afi_update_done();
		}
		return true;
	}
}

int oplus_gauge_get_passedchg(int *val)
{
	int ret;
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_passdchg) {
		return 0;
	} else {
		ret = g_gauge_chip->gauge_ops->get_passdchg(val);
		if (ret) {
			pr_err("%s: get passedchg error %d\n", __FUNCTION__, ret);
		}
		return ret;
	}
}

int oplus_gauge_dump_register(void)
{
	if (!g_gauge_chip)
		return 0;
	else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->dump_register) {
			g_gauge_chip->gauge_ops->dump_register();
		}
		return 0;
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_sub_batt_mvolts(void)
{
	if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops || !g_sub_gauge_chip->gauge_ops->get_battery_mvolts) {
		return 3800;
	} else {
		if (sub_gauge_dbg_vbat != 0) {
			chg_err("[OPLUS_CHG]%s:debug enabled,voltage sub_gauge_dbg_vbat[%d] \n", __func__,
				sub_gauge_dbg_vbat);
			return sub_gauge_dbg_vbat;
		}
		return g_sub_gauge_chip->gauge_ops->get_battery_mvolts();
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_sub_batt_current(void)
{
	if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops || !g_sub_gauge_chip->gauge_ops->get_average_current) {
		return 100;
	} else {
		if (sub_gauge_dbg_ibat != 0) {
			chg_err("[OPLUS_CHG]debug enabled,current sub_gauge_dbg_ibat[%d] \n", sub_gauge_dbg_ibat);
			return sub_gauge_dbg_ibat;
		}
		return g_sub_gauge_chip->gauge_ops->get_average_current();
	}
}

int oplus_gauge_get_main_batt_soc(void)
{
	if (!g_gauge_chip) {
		return -1;
	} else {
		if (gauge_dbg_soc != 0) {
			chg_err("debug enabled, main soc[%d]\n", gauge_dbg_soc);
			return gauge_dbg_soc;
		} else {
			return g_gauge_chip->gauge_ops->get_battery_soc();
		}
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_sub_batt_soc(void)
{
	if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops || !g_sub_gauge_chip->gauge_ops->get_battery_soc) {
		return -1;
	} else {
		return g_sub_gauge_chip->gauge_ops->get_battery_soc();
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
int oplus_gauge_get_sub_batt_temperature(void)
{
	int batt_temp = 0;
	if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops ||
	    !g_sub_gauge_chip->gauge_ops->get_battery_temperature) {
		return 250;
	} else {
		if (sub_gauge_dbg_tbat != 0 || gauge_dbg_tbat != 0) {
			chg_err("[OPLUS_CHG]debug enabled, gauge_dbg_tbat[%d],  sub_gauge_dbg_ibat[%d]\n",
				gauge_dbg_tbat, sub_gauge_dbg_tbat);
			return sub_gauge_dbg_tbat != 0 ? sub_gauge_dbg_tbat : gauge_dbg_tbat;
		}
		batt_temp = g_sub_gauge_chip->gauge_ops->get_battery_temperature();

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
		if (get_eng_version() == HIGH_TEMP_AGING || oplus_is_ptcrb_version()) {
			if (batt_temp > 690) {
				chg_err("[OPLUS_CHG]CONFIG_HIGH_TEMP_VERSION enable here,"
					"disable high tbat shutdown\n");
				batt_temp = 690;
			}
		}
#endif
		return batt_temp;
	}
}

bool oplus_gauge_get_sub_batt_authenticate(void)
{
	if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops ||
	    !g_sub_gauge_chip->gauge_ops->get_battery_authenticate) {
		return false;
	} else {
		return g_sub_gauge_chip->gauge_ops->get_battery_authenticate();
	}
}

int oplus_gauge_get_bcc_parameters(char *buf)
{
	if (!g_gauge_chip)
		return 0;
	else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_bcc_parameters) {
			g_gauge_chip->gauge_ops->get_bcc_parameters(buf);
		}
		return 0;
	}
}

int oplus_gauge_fastchg_update_bcc_parameters(char *buf)
{
	if (!g_gauge_chip)
		return 0;
	else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_update_bcc_parameters) {
			g_gauge_chip->gauge_ops->get_update_bcc_parameters(buf);
		}
		return 0;
	}
}

int oplus_gauge_get_prev_bcc_parameters(char *buf)
{
	if (!g_gauge_chip)
		return 0;
	else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_prev_bcc_parameters) {
			g_gauge_chip->gauge_ops->get_prev_bcc_parameters(buf);
		}
		return 0;
	}
}

int oplus_gauge_set_bcc_parameters(const char *buf)
{
	if (!g_gauge_chip)
		return 0;
	else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->set_bcc_parameters) {
			g_gauge_chip->gauge_ops->set_bcc_parameters(buf);
		}
		return 0;
	}
}

bool oplus_gauge_check_rc_sfr(void)
{
	if (!g_gauge_chip)
		return false;
	else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->check_rc_sfr) {
			return g_gauge_chip->gauge_ops->check_rc_sfr();
		}
		return false;
	}
}

int oplus_gauge_soft_reset_rc_sfr(void)
{
	if (!g_gauge_chip)
		return -1;
	else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->soft_reset_rc_sfr) {
			return g_gauge_chip->gauge_ops->soft_reset_rc_sfr();
		}
		return -1;
	}
}

bool oplus_gauge_get_bqfs_status(void)
{
	if (!g_gauge_chip)
		return false;
	else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_bqfs_status) {
			return g_gauge_chip->gauge_ops->get_bqfs_status();
		}
		return true;
	}
}

int oplus_gauge_check_bqfs_fw(void)
{
	int rc = 0;
	if (!g_gauge_chip)
		return rc;

	if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->bqfs_fw_check)
		rc = g_gauge_chip->gauge_ops->bqfs_fw_check();

	return rc;
}

int oplus_gauge_bqfs_data_check(void)
{
	int rc = 0;
	if (!g_gauge_chip)
		return rc;

	if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->bqfs_data_check)
		rc = g_gauge_chip->gauge_ops->bqfs_data_check();

	return rc;
}

void oplus_gauge_get_device_name(u8 *name, int len)
{
	if (!g_gauge_chip || !name)
		return;
	else if (!g_gauge_chip->device_name) {
		if (!g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_device_name)
			return;
		g_gauge_chip->device_name = kmalloc(len, GFP_KERNEL);
		if (!g_gauge_chip->device_name)
			return;
		else
			memset(g_gauge_chip->device_name, 0, len);

		g_gauge_chip->gauge_ops->get_device_name(g_gauge_chip->device_name, len);
		strncpy(name, g_gauge_chip->device_name, len);
	} else
		strncpy(name, g_gauge_chip->device_name, len);
}

int oplus_gauge_get_info(u8 *info, int len)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_gauge_info)
		return -1;

	return g_gauge_chip->gauge_ops->get_gauge_info(info, len);
}

int oplus_sub_gauge_get_info(u8 *info, int len)
{
	if (!g_sub_gauge_chip || !g_sub_gauge_chip->gauge_ops || !g_sub_gauge_chip->gauge_ops->get_gauge_info)
		return -1;
	return g_sub_gauge_chip->gauge_ops->get_gauge_info(info, len);
}

int oplus_gauge_get_qmax_v1(int *qmax1, int *qmax2)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_batt_qmax)
		return -1;
	return g_gauge_chip->gauge_ops->get_batt_qmax(qmax1, qmax2);
}

int oplus_gauge_get_fcc(int *fcc1, int *fcc2)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_batt_fcc)
		return -1;
	return g_gauge_chip->gauge_ops->get_batt_fcc(fcc1, fcc2);
}

int oplus_gauge_get_cc(int *cc1, int *cc2)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_batt_cc)
		return -1;
	return g_gauge_chip->gauge_ops->get_batt_cc(cc1, cc2);
}

int oplus_gauge_get_soh(int *soh1, int *soh2)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_batt_soh)
		return -1;
	return g_gauge_chip->gauge_ops->get_batt_soh(soh1, soh2);
}

int oplus_gauge_get_calib_time(int *dod_calib_time, int *qmax_calib_time, int gauge_index)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops || !g_gauge_chip->gauge_ops->get_calib_time)
		return -1;
	return g_gauge_chip->gauge_ops->get_calib_time(dod_calib_time, qmax_calib_time, gauge_index);
}

void oplus_gauge_cal_model_check(bool ffc_state)
{
	if (!g_gauge_chip)
		return;
	else {
		if (oplus_switching_support_parallel_chg()) {
			if (g_sub_gauge_chip && g_sub_gauge_chip->gauge_ops &&
					g_sub_gauge_chip->gauge_ops->cal_model_check) {
				g_sub_gauge_chip->gauge_ops->cal_model_check(ffc_state);
			}
		}
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->cal_model_check) {
			return g_gauge_chip->gauge_ops->cal_model_check(ffc_state);
		}
		return;
	}
}

int oplus_gauge_get_bat_info_manu_date(char *info, int len)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_batt_manu_date) {
			return g_gauge_chip->gauge_ops->get_batt_manu_date(info, len);
		}
		return 0;
	}
}

int oplus_gauge_get_bat_info_first_usage_date(char *info, int len)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_batt_first_usage_date) {
			return g_gauge_chip->gauge_ops->get_batt_first_usage_date(info, len);
		}
		return 0;
	}
}

int oplus_gauge_set_bat_info_first_usage_date(const char *info)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->set_batt_first_usage_date) {
			return g_gauge_chip->gauge_ops->set_batt_first_usage_date(info);
		}
		return 0;
	}
}

int oplus_pack_gauge_get_seal_flag(void)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_seal_flag) {
			return g_gauge_chip->gauge_ops->get_seal_flag();
		}
		return 0;
	}
}

int oplus_pack_gauge_set_seal_flag(int seal_flag)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->set_seal_flag) {
			return g_gauge_chip->gauge_ops->set_seal_flag(seal_flag);
		}
		return 0;
	}
}

int oplus_gauge_get_battinfo_ui_cc(void)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_batt_ui_cc) {
			return g_gauge_chip->gauge_ops->get_batt_ui_cc();
		}
		return 0;
	}
}

int oplus_gauge_set_battinfo_ui_cc(int ui_cc)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->set_batt_ui_cc) {
			return g_gauge_chip->gauge_ops->set_batt_ui_cc(ui_cc);
		}
		return 0;
	}
}

int oplus_gauge_get_battinfo_ui_soh(void)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_batt_ui_soh) {
			return g_gauge_chip->gauge_ops->get_batt_ui_soh();
		}
		return 0;
	}
}

int oplus_gauge_set_battinfo_ui_soh(int ui_soh)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->set_batt_ui_soh) {
			return g_gauge_chip->gauge_ops->set_batt_ui_soh(ui_soh);
		}
		return 0;
	}
}

int oplus_gauge_get_battinfo_used_flag(void)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_batt_used_flag) {
			return g_gauge_chip->gauge_ops->get_batt_used_flag();
		}
		return 0;
	}
}

int oplus_gauge_set_battinfo_used_flag(int used_flag)
{
	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->set_batt_used_flag) {
			return g_gauge_chip->gauge_ops->set_batt_used_flag(used_flag);
		}
		return 0;
	}
}

int oplus_gauge_get_bat_info_sn(char *sn_buff, int size_buffer)
{
	int rc = 0;
	int len = 0;

	if (!g_gauge_chip) {
		return -EINVAL;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->get_battinfo_sn) {
			rc = g_gauge_chip->gauge_ops->get_battinfo_sn(sn_buff, size_buffer);
			if (rc < 0) {
				return -EINVAL;
			}
			len = rc;
			return len;
		}
	}
	return 0;
}
