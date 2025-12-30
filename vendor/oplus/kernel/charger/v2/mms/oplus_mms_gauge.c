#define pr_fmt(fmt) "[MMS_GAUGE]([%s][%d]): " fmt, __func__, __LINE__

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
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/power_supply.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_voter.h>
#include <oplus_mms.h>
#include <oplus_chg_monitor.h>
#include <oplus_mms_wired.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_vooc.h>
#include <oplus_batt_bal.h>
#include <oplus_parallel.h>
#include <oplus_chg_wls.h>
#include <oplus_chg_mutual.h>
#include <linux/ktime.h>
#include <linux/sched/clock.h>
#include "gauge/oplus_gauge_common.h"
#include <oplus_sec.h>
#include <oplus_chg_cpa.h>

#ifndef CONFIG_OPLUS_CHARGER_MTK
#include <linux/soc/qcom/smem.h>
#endif

#define GAUGE_IC_NUM_MAX 2
#define ERR_COUNT_MAX 3
#define GAUGE_PARALLEL_IC_NUM_MIN 2
#define GAUGE_DEFAULT_VOLT_MV		3800
#define DEFAULT_SOC 50

enum fcl_table_type {
	FCL_EXTERN_GAUGE,
	FCL_PLATFORM_GAUGE,
	FCL_GAUGE_MAX,
};

static struct oplus_mms_gauge *g_mms_gauge;
static void oplus_mms_gauge_get_reserve_calib_info(struct oplus_mms_gauge *chip);
static int oplus_mms_gauge_push_auth(struct oplus_mms_gauge *chip);
static int oplus_mms_gauge_push_hmac(struct oplus_mms_gauge *chip);

static int gauge_dbg_tbat = 0;
module_param(gauge_dbg_tbat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_tbat, "debug battery temperature");

static int gauge_dbg_vbat = 0;
module_param(gauge_dbg_vbat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_vbat, "debug battery voltage");

static int gauge_dbg_ibat = 0;
module_param(gauge_dbg_ibat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_ibat, "debug battery current");

static struct fcl_table gauge_fcl_table[FCL_TABLE_MAX][FCL_CURVE_MAX] = {
	{
		{0, 500, 800},
		{20, 200, 800},
		{60, 0, 800}
	},
	{
		{0, 500, 800},
		{20, 200, 800},
		{40, 0, 800}
	}
};

__maybe_unused static bool
is_err_topic_available(struct oplus_mms_gauge *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

static bool is_support_parallel(struct oplus_mms_gauge *chip)
{
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return false;
	}

	if (chip->child_num >= GAUGE_PARALLEL_IC_NUM_MIN)
		return true;
	else
		return false;
}

int is_support_parallel_battery(struct oplus_mms *topic)
{
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (!is_support_parallel(chip))
		return DEFAULT_CONNECT_TYPE;

	if (chip->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL)
		return SERIAL_CONNECT_TYPE;
	else
		return PARALLEL_CONNECT_TYPE;
}

int oplus_gauge_get_batt_mvolts(void)
{
	int rc;
	int vol_mv;

	if (!g_mms_gauge)
		return 3800;

	if (gauge_dbg_vbat != 0) {
		chg_info("debug enabled, voltage gauge_dbg_vbat[%d]\n", gauge_dbg_vbat);
		return gauge_dbg_vbat;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage error, rc=%d\n", rc);
		return 3800;
	}

	return vol_mv;
}

int oplus_gauge_get_batt_fc(void)
{
	int rc;
	int fc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FC, &fc);
	if (rc < 0) {
		chg_err("get battery fc error, rc=%d\n", rc);
		return 0;
	}

	return fc;
}

int oplus_gauge_get_batt_qm(void)
{
	int rc;
	int qm;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_QM, &qm);
	if (rc < 0) {
		chg_err("get battery qm error, rc=%d\n", rc);
		return 0;
	}

	return qm;
}

int oplus_gauge_get_batt_pd(void)
{
	int rc;
	int pd;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_PD, &pd);
	if (rc < 0) {
		chg_err("get battery pd error, rc=%d\n", rc);
		return 0;
	}

	return pd;
}

int oplus_gauge_get_batt_rcu(void)
{
	int rc;
	int rcu;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RCU, &rcu);
	if (rc < 0) {
		chg_err("get battery rcu error, rc=%d\n", rc);
		return 0;
	}

	return rcu;
}

int oplus_gauge_get_batt_rcf(void)
{
	int rc;
	int rcf;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RCF, &rcf);
	if (rc < 0) {
		chg_err("get battery rcf error, rc=%d\n", rc);
		return 0;
	}

	return rcf;
}

int oplus_gauge_get_batt_fcu(void)
{
	int rc;
	int fcu;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCU, &fcu);
	if (rc < 0) {
		chg_err("get battery fcu error, rc=%d\n", rc);
		return 0;
	}

	return fcu;
}

int oplus_gauge_get_batt_fcf(void)
{
	int rc;
	int fcf;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCF, &fcf);
	if (rc < 0) {
		chg_err("get battery fcf error, rc=%d\n", rc);
		return 0;
	}

	return fcf;
}

int oplus_gauge_get_batt_sou(void)
{
	int rc;
	int sou;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOU, &sou);
	if (rc < 0) {
		chg_err("get battery sou error, rc=%d\n", rc);
		return 0;
	}

	return sou;
}

int oplus_gauge_get_batt_do0(void)
{
	int rc;
	int do0;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_DO0, &do0);
	if (rc < 0) {
		chg_err("get battery do0 error, rc=%d\n", rc);
		return 0;
	}

	return do0;
}

int oplus_gauge_get_batt_doe(void)
{
	int rc;
	int doe;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_DOE, &doe);
	if (rc < 0) {
		chg_err("get battery doe error, rc=%d\n", rc);
		return 0;
	}

	return doe;
}

int oplus_gauge_get_batt_trm(void)
{
	int rc;
	int trm;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_TRM, &trm);
	if (rc < 0) {
		chg_err("get battery trm error, rc=%d\n", rc);
		return 0;
	}

	return trm;
}

int oplus_gauge_get_batt_pc(void)
{
	int rc;
	int pc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_PC, &pc);
	if (rc < 0) {
		chg_err("get battery pc error, rc=%d\n", rc);
		return 0;
	}

	return pc;
}

int oplus_gauge_get_batt_qs(void)
{
	int rc;
	int qs;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_QS, &qs);
	if (rc < 0) {
		chg_err("get battery qs error, rc=%d\n", rc);
		return 0;
	}

	return qs;
}

int oplus_gauge_get_batt_mvolts_2cell_max(void)
{
	int rc;
	int vol_mv;

	if (!g_mms_gauge)
		return 0;

	if (gauge_dbg_vbat != 0) {
		chg_info("debug enabled, voltage gauge_dbg_vbat[%d]\n", gauge_dbg_vbat);
		return gauge_dbg_vbat;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage max error, rc=%d\n", rc);
		return 0;
	}

	return vol_mv;
}

int oplus_gauge_get_batt_mvolts_2cell_min(void)
{
	int rc;
	int vol_mv;

	if (!g_mms_gauge)
		return 0;

	if (gauge_dbg_vbat != 0) {
		chg_info("debug enabled, voltage gauge_dbg_vbat[%d]\n", gauge_dbg_vbat);
		return gauge_dbg_vbat;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN, &vol_mv);
	if (rc < 0) {
		chg_err("get battery voltage min error, rc=%d\n", rc);
		return 0;
	}

	return vol_mv;
}

static int oplus_gauge_get_subboard_temp(struct oplus_mms_gauge *chip)
{
	int rc;
	int temp;

	if (gauge_dbg_tbat != 0) {
		chg_err("debug enabled, gauge_dbg_tbat[%d]\n", gauge_dbg_tbat);
		return gauge_dbg_tbat;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_SUBBOARD_TEMP, &temp);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("get sub board temp error, rc=%d\n", rc);
		return GAUGE_INVALID_TEMP;
	}
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == HIGH_TEMP_AGING || oplus_is_ptcrb_version()) {
		chg_info("HIGH_TEMP_AGING, disable high tbat shutdown,temp %d -> 690\n", temp);
		if (temp > 690)
			temp = 690;
	}
#endif
	return temp;
}

int oplus_gauge_show_batt_chem_id(struct oplus_mms *topic, char *buf, int len)
{
	struct oplus_mms_gauge *chip;
	u8 chemid[CHEMID_MAX_LENGTH] = {0};
	int rc;
	struct oplus_chg_ic_dev *ic;
	int i, j;
	int index = 0;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return 0;
	}
	if (sizeof(chemid) < CHEM_ID_LENGTH * chip->child_num)
		return 0;

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_CHEM_ID, chemid + i * CHEM_ID_LENGTH,
					       sizeof(chemid));
			if (rc < 0)
				continue;
			index += snprintf(buf + index, len - index, "[%d][%s]", i, ic->manu_name);
			for (j = i * CHEM_ID_LENGTH; j < CHEM_ID_LENGTH * (i + 1); j++)
				index += snprintf(buf + index, len - index, "%x ", chemid[j]);
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_CHEM_ID, chemid, sizeof(chemid));
			if (rc >= 0) {
				index += snprintf(buf + index, len - index, "[%d][%s]", i, ic->manu_name);
				for (j = 0; j < CHEM_ID_LENGTH; j++)
					index += snprintf(buf + index, len - index, "%x ", chemid[j]);
			}
		}
	}

	return index;
}

static void oplus_mms_gauge_read_mode_set(struct oplus_mms *mms, bool mode)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}

	chip = oplus_mms_get_drvdata(mms);

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_READ_MODE, mode);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't set gauge read mode, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_READ_MODE, mode);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't set gauge read mode, rc=%d\n", i, ic->manu_name, rc);
			return;
		}
	}
}

#define TEMP_SELECT_POINT 320
static int oplus_gauge_get_batt_temperature(struct oplus_mms_gauge *chip)
{
	int rc;
	int temp;
	int main_temp, sub_temp;

	if (gauge_dbg_tbat != 0) {
		chg_err("debug enabled, gauge_dbg_tbat[%d]\n", gauge_dbg_tbat);
		return gauge_dbg_tbat;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic_comb[chip->main_gauge],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP, &main_temp);
	if (rc < 0) {
		chg_err("get battery temp error, rc=%d\n", rc);
		main_temp = GAUGE_INVALID_TEMP;
	}
	if (chip->sub_gauge) {
		rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic_comb[__ffs(chip->sub_gauge)],
			OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP, &sub_temp);
		if (rc < 0) {
			chg_err("get sub battery temp error, rc=%d\n", rc);
			sub_temp = GAUGE_INVALID_TEMP;
		}
		if (sub_temp == GAUGE_INVALID_TEMP || main_temp == GAUGE_INVALID_TEMP) {
			if (chip->connect_type != OPLUS_CHG_IC_CONNECT_SERIAL)
				temp = main_temp;
			else
				temp = GAUGE_INVALID_TEMP;
		} else if (main_temp >= TEMP_SELECT_POINT || sub_temp >= TEMP_SELECT_POINT) {
			temp = main_temp > sub_temp ? main_temp : sub_temp;
		} else {
			temp = main_temp < sub_temp ? main_temp : sub_temp;
		}
	} else {
		temp = main_temp;
	}

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == HIGH_TEMP_AGING || oplus_is_ptcrb_version()) {
		chg_info("HIGH_TEMP_AGING, disable high tbat shutdown,temp %d -> 690\n", temp);
		if (temp > 690)
			temp = 690;
	}
#endif
	return temp;
}

int oplus_gauge_get_batt_soc(void)
{
	int rc;
	int soc;

	if (!g_mms_gauge)
		return DEFAULT_SOC;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC, &soc);
	if (rc < 0) {
		chg_err("get battery soc error, rc=%d\n", rc);
		return DEFAULT_SOC;
	}

	return soc;
}

int oplus_gauge_get_bcc_parameters(char *buf)
{
	int rc;
	bool gauge_locked;

	if (g_mms_gauge == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	gauge_locked = oplus_gauge_is_locked();

	if (!gauge_locked) {
		rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BCC_PARMS, buf);
		if (rc < 0) {
			chg_err("update cc parms, rc=%d\n", rc);
			return 0;
		}
	}

	return 0;
}

int oplus_gauge_fastchg_update_bcc_parameters(char *buf)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_FASTCHG_UPDATE_BCC_PARMS, buf);
	if (rc < 0) {
		chg_err("update cc parms, rc=%d\n", rc);
		return 0;
	}
	return 0;
}

int oplus_gauge_get_prev_bcc_parameters(char *buf)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_PREV_BCC_PARMS, buf);
	if (rc < 0) {
		chg_err("update cc parms, rc=%d\n", rc);
		return 0;
	}
	return 0;
}

int oplus_gauge_set_bcc_parameters(const char *buf)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_BCC_PARMS, buf);
	if (rc < 0) {
		chg_err("update cc parms, rc=%d\n", rc);
		return 0;
	}
	return 0;
}

int oplus_gauge_protect_check(void)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return 0;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_PROTECT_CHECK);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("protect_check, rc=%d\n", rc);
		return 0;
	}
	return 0;
}

bool oplus_gauge_afi_update_done(void)
{
	int rc;
	bool status = true;

	if (g_mms_gauge == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return true;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_AFI_UPDATE_DONE, &status);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("afi_update_done, rc=%d\n", rc);
		return true;
	}
	return status;
}

bool oplus_gauge_check_reset_condition(void)
{
	int rc;
	bool need_reset = false;

	if (g_mms_gauge == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return true;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_CHECK_RESET, &need_reset);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("need_reset, rc=%d\n", rc);
		return false;
	}
	return need_reset;
}

bool oplus_gauge_reset(void)
{
	int rc;
	bool reset_done = false;

	if (g_mms_gauge == NULL) {
		chg_err("g_gauge_ic is NULL\n");
		return true;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_RESET, &reset_done);
	if (rc < 0) {
		chg_err("reset_done, rc=%d\n", rc);
		return false;
	}
	chg_err("oplus_gauge_reset, reset_done=%d\n", reset_done);
	return reset_done;
}

int oplus_gauge_get_real_time_current(void)
{
	int rc;
	int curr_ma;
	int main_curr = 0, sub_curr = 0;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic_comb[g_mms_gauge->main_gauge],
		OPLUS_IC_FUNC_GAUGE_GET_REAL_TIME_CURR, &main_curr);
	if (rc < 0) {
		rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic_comb[g_mms_gauge->main_gauge],
			OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &main_curr);
		if (rc < 0) {
			chg_err("get main battery current error, rc=%d\n", rc);
			main_curr = 0;
		}
	}
	if (g_mms_gauge->sub_gauge) {
		rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic_comb[__ffs(g_mms_gauge->sub_gauge)],
			OPLUS_IC_FUNC_GAUGE_GET_REAL_TIME_CURR, &sub_curr);
		if (rc < 0) {
			rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic_comb[__ffs(g_mms_gauge->sub_gauge)],
				OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &sub_curr);
			if (rc < 0) {
				chg_err("get sub battery current error, rc=%d\n", rc);
				sub_curr = 0;
			}
		}
	}
	if (g_mms_gauge->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL)
		curr_ma = (main_curr + sub_curr) / 2;
	else
		curr_ma = main_curr + sub_curr;

	return curr_ma;
}

int oplus_gauge_get_batt_current(void)
{
	int rc;
	int curr_ma;
	int main_curr = 0, sub_curr = 0;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic_comb[g_mms_gauge->main_gauge],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &main_curr);
	if (rc < 0) {
		chg_err("get main battery current error, rc=%d\n", rc);
		main_curr = 0;
	}
	if (g_mms_gauge->sub_gauge) {
		rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic_comb[__ffs(g_mms_gauge->sub_gauge)],
			OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &sub_curr);
		if (rc < 0) {
			chg_err("get sub battery current error, rc=%d\n", rc);
			sub_curr = 0;
		}
	}
	if (g_mms_gauge->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL)
		curr_ma = (main_curr + sub_curr) / 2;
	else
		curr_ma = main_curr + sub_curr;

	return curr_ma;
}

int oplus_gauge_get_remaining_capacity(void)
{
	int rc;
	int rm;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_RM, &rm);
	if (rc < 0) {
		chg_err("get battery remaining capacity error, rc=%d\n", rc);
		return 0;
	}

	return rm;
}

void oplus_gauge_set_plugin_status(void)
{
	int rc;
	if (!g_mms_gauge)
		return;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SYNC_PLUGIN);
	if (rc < 0)
		chg_err("set gauge plugin status err, rc=%d\n", rc);

	return;
}

int oplus_gauge_get_device_type(void)
{
	if (!g_mms_gauge)
		return 0;
	return g_mms_gauge->device_type;
}

int oplus_gauge_get_device_type_for_vooc(void)
{
	if (!g_mms_gauge)
		return 0;
	return g_mms_gauge->device_type_for_vooc;
}

int oplus_gauge_get_batt_fcc(void)
{
	int rc;
	int fcc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC, &fcc);
	if (rc < 0) {
		chg_err("get battery fcc error, rc=%d\n", rc);
		return 0;
	}

	return fcc;
}

int oplus_gauge_get_batt_cc(void)
{
	int rc;
	int cc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CC, &cc);
	if (rc < 0) {
		chg_err("get battery cc error, rc=%d\n", rc);
		return 0;
	}

	return cc;
}

int oplus_gauge_get_batt_soh(void)
{
	int rc;
	int soh;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH, &soh);
	if (rc < 0) {
		chg_err("get battery soh error, rc=%d\n", rc);
		return 0;
	}

	return soh;
}

bool oplus_gauge_get_batt_hmac(void)
{
	int rc;
	bool pass;

	if (!g_mms_gauge)
		return false;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC, &pass);
	if (rc < 0) {
		chg_err("get battery hmac status error, rc=%d\n", rc);
		return false;
	}

	return pass;
}

bool oplus_gauge_get_batt_authenticate(void)
{
	int rc;
	bool pass;

	if (!g_mms_gauge)
		return false;
	if (g_mms_gauge->bat_volt_different)
		return false;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic,
			       OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH, &pass);
	if (rc < 0) {
		chg_err("get battery authenticate status error, rc=%d\n", rc);
		return false;
	}

	return pass;
}

bool oplus_gauge_is_exist(struct oplus_mms *topic)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;
	bool exist = true;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return false;
	}

	chip = oplus_mms_get_drvdata(topic);

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST, &exist);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get batt exist, rc=%d\n", i, ic->manu_name, rc);
				exist = false;
				break;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST, &exist);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get batt exist, rc=%d\n", i, ic->manu_name, rc);
				exist = false;
			}
			break;
		}
	}

	return exist;
}

int oplus_gauge_get_batt_capacity_mah(struct oplus_mms *topic)
{
	struct oplus_mms_gauge *chip;
	int rc;
	int cap_mah = 0;
	int i;
	int cap_temp;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 2000;
	}
	chip = oplus_mms_get_drvdata(topic);

	for (i = 0; i < chip->child_num; i++) {
		rc = oplus_chg_ic_func(chip->gauge_ic_comb[i],
			OPLUS_IC_FUNC_GAUGE_GET_BATT_CAP, &cap_temp);
		if (rc < 0) {
			chg_err("get battery capacity_mah error, rc=%d\n", rc);
			cap_temp = 2000;
		}
		cap_mah += cap_temp;
	}

	return cap_mah;
}

int oplus_gauge_get_battinfo_sn(struct oplus_mms *topic, char *sn_buff, int size_buffer)
{
	int rc = 0;
	int len = 0;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip || !sn_buff || size_buffer < OPLUS_BATT_SERIAL_NUM_SIZE)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_SN, sn_buff, size_buffer);
	len = rc;
	if (rc < 0) {
		chg_err("get main battery serial number error, rc=%d\n", rc);
	} else {
		if (chip->sub_gauge
		    && ((size_buffer - len) > OPLUS_BATT_SERIAL_NUM_SIZE)) {
			/* change the end of main_gauge '\0' to '\n', and add 1 to the length */
			sn_buff[len] = '\n';
			len += 1;
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
				OPLUS_IC_FUNC_GAUGE_GET_BATT_SN, &sn_buff[len], (size_buffer - len));
			if (rc < 0)
				chg_err("get sub battery serial number error, rc=%d\n", rc);
			else
				len += rc;
		}
	}
	return len;
}

int oplus_gauge_get_battinfo_manu_date(struct oplus_mms *topic, char *buff, int size_buffer)
{
	int rc = 0;
	int len = 0;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip || !buff || size_buffer < OPLUS_BATTINFO_DATE_SIZE)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
		OPLUS_IC_FUNC_GAUGE_GET_MANU_DATE, buff, size_buffer);
	len = rc;
	if (rc < 0) {
		chg_err("get main battery manu date error, rc=%d\n", rc);
	} else {
		if (chip->sub_gauge
		    && ((size_buffer - len) > OPLUS_BATTINFO_DATE_SIZE)) {
			/* change the end of main_gauge '\0' to '\n', and add 1 to the length */
			buff[len] = '\n';
			len += 1;
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
				OPLUS_IC_FUNC_GAUGE_GET_MANU_DATE, &buff[len], (size_buffer - len));
			if (rc < 0)
				chg_err("get sub battery manu date error, rc=%d\n", rc);
			else
				len += rc;
		}
	}
	return len;
}

/* first usage date store in main gauge */
int oplus_gauge_get_battinfo_first_usage_date(struct oplus_mms *topic, char *buff, int size_buffer)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip || !buff || size_buffer < OPLUS_BATTINFO_DATE_SIZE)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->gauge_ic,
		OPLUS_IC_FUNC_GAUGE_GET_FIRST_USAGE_DATE, buff, size_buffer);

	if (rc < 0)
		chg_err("get main battery first usage date error, rc=%d\n", rc);

	return rc;
}

#define OPLUS_BATTSOH_INFO_DATE_SIZE 128
int oplus_gauge_get_batt_sn(struct oplus_mms *topic, char *buff, int size_buffer)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;
	char buff_get[OPLUS_BATTSOH_INFO_DATE_SIZE] = {"\0"};
	int i;
	int len = 0;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip || !buff)
		return -EINVAL;

	if (strcmp("gauge", topic->desc->name) == 0) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_GET_BATT_IC_SN, buff_get, size_buffer);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get batt BATT_IC_SN, rc=%d\n",
					i, ic->manu_name, rc);
				continue;
			}
			len = sprintf(buff, "%s", buff_get);
		}
	}
	return rc;
}

int oplus_gauge_get_historic_soh_date(struct oplus_mms *topic, int *buff, int size_buffer)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;
	int buff_get[OPLUS_BATTSOH_INFO_DATE_SIZE] = {0};
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip || !buff)
		return -EINVAL;

	if (strcmp("gauge", topic->desc->name) == 0) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_GET_BATT_HISTSOH_DATA, buff_get, size_buffer);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get batt BATT_IC_SN, rc=%d\n",
					i, ic->manu_name, rc);
				continue;
			}
			memcpy(buff, buff_get, size_buffer);
		}
	}
	return rc;
}

int oplus_set_histrioc_soh_date(struct oplus_mms *topic, const int *buff)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip || !buff)
		return -EINVAL;

	if (strcmp("gauge", topic->desc->name) == 0) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_BATT_HISTSOH_DATA, buff);
			if (rc < 0 && rc != -ENOTSUPP) {
				chg_err("gauge[%d](%s): can't set BATT_HISTSOH_DATA, rc=%d\n", i, ic->manu_name, rc);
				break;
			}
		}
	}
	return rc;
}

int oplus_set_reset_gauge_parms(struct oplus_mms *topic, const int *buff)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip || !buff)
		return -EINVAL;

	if (strcmp("gauge", topic->desc->name) == 0) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_RESET_GAUGE_DATE, buff);
			if (rc < 0 && rc != -ENOTSUPP) {
				chg_err("gauge[%d](%s): can't set RESET_GAUGE_DATE, rc=%d\n", i, ic->manu_name, rc);
				break;
			}
		}
	}
	return rc;
}

int oplus_gauge_set_battinfo_first_usage_date(struct oplus_mms *topic, const char *buff)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip || !buff)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->gauge_ic,
		OPLUS_IC_FUNC_GAUGE_SET_FIRST_USAGE_DATE, buff);

	if (rc < 0)
		chg_err("set battery first usage date error, rc=%d\n", rc);

	return rc;
}

int oplus_gauge_get_ui_cc(struct oplus_mms *topic)
{
	int rc;
	u16 cyc_cnt;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_UI_CC, &cyc_cnt);
	if (rc < 0) {
		chg_err("get battery ui cycle count error, rc=%d\n", rc);
		return 0;
	}

	return cyc_cnt;
}

int oplus_gauge_set_ui_cc(struct oplus_mms *topic, int count)
{
	int rc = 0;
	u16 cyc_cnt = count & 0xFFFF;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_UI_CC, cyc_cnt);
	if (rc < 0) {
		chg_err("set battery ui cycle count error, rc=%d\n", rc);
	}

	return rc;
}

int oplus_gauge_get_ui_soh(struct oplus_mms *topic)
{
	int rc;
	u8 soh;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_UI_SOH, &soh);
	if (rc < 0) {
		chg_err("get battery ui soh error, rc=%d\n", rc);
		return 0;
	}

	return soh;
}

int oplus_gauge_set_ui_soh(struct oplus_mms *topic, int ui_soh)
{
	int rc = 0;
	u8 soh = ui_soh & 0xFF;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_UI_SOH, soh);
	if (rc < 0) {
		chg_err("set battery ui cycle count error, rc=%d\n", rc);
	}

	return rc;
}

int oplus_gauge_get_used_flag(struct oplus_mms *topic)
{
	int rc;
	u8 flag;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_USED_FLAG, &flag);
	if (rc < 0) {
		chg_err("get battery used flag error, rc=%d\n", rc);
		return 0;
	}

	return flag;
}

int oplus_gauge_set_used_flag(struct oplus_mms *topic, int used_flag)
{
	int rc = 0;
	u8 flag = used_flag & 0xFF;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_USED_FLAG, flag);
	if (rc < 0) {
		chg_err("set battery used flag error, rc=%d\n", rc);
	}

	return rc;
}

int oplus_gauge_get_dec_pack_type(struct oplus_mms *topic)
{
	int rc;
	int flag;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return 0;

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_DEC_FG_TYPE, &flag);
	if (rc < 0) {
		chg_err("get battery used flag error, rc=%d\n", rc);
		return 0;
	}

	return flag;
}

int oplus_gauge_get_dec_cv_soh(struct oplus_mms *topic)
{
	int rc;
	int soh;
	struct oplus_mms_gauge *chip;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return 0;

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH, &soh);
	if (rc < 0) {
		chg_err("get battery used flag error, rc=%d\n", rc);
		return 0;
	}

	return soh;
}

bool oplus_gauge_set_fg_vct(struct oplus_mms *mms, int vct)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return false;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip  || vct < 0)
		return false;

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_VCT, vct);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't set vct, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_VCT, vct);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't  set vct, rc=%d\n", i, ic->manu_name, rc);
			return true;
		}
	}
	return true;
}

int oplus_gauge_get_fg_vct(struct oplus_mms *mms)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;
	int temp = -EINVAL;
	int i;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip)
		return rc;

	if (mms == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_VCT, &temp);
			if (rc < 0) {
				chg_err("gauge[%d](%s): get batt vct error, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_VCT, &temp);
			if (rc < 0)
				chg_err("gauge[%d](%s): get batt vct error, rc=%d\n", i, ic->manu_name, rc);
			return temp;
		}
	}
	chg_info(" temp=%d\n", temp);

	return temp;
}

static const char *physical_gauge_name_array[] = {
	"bq27541", "bq27411", "bq27426", "bq27z561", "bq28z610",
	"zy0602", "zy0603",
	"nfg1000a", "nfg8011b", "sn28z729", "mpc7022",
};

int oplus_gauge_get_physical_name(struct oplus_mms *mms, char *name, int len)
{
	int i, j;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;
	struct device_node *node;
	char extracted_name[OPLUS_CHG_IC_MANU_NAME_MAX] = { 0 };
	int child_num = 0;
	int rc = 0;

	if (!mms) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}

	if (len <= 0) {
		chg_err("len %d invalid\n", len);
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip) {
		chg_err("chip is null\n");
		return -EINVAL;
	}

	node = chip->child_list[0].ic_dev->dev->of_node;
	child_num = of_property_count_elems_of_size(node, "oplus,gauge_ic", sizeof(u32));
	if (child_num <= 0) {
		chg_err("oplus,gauge_ic elems %d invalid\n", child_num);
		return -ENODATA;
	}

	for (i = 0; i < child_num; i++) {
		ic = of_get_oplus_chg_ic(node, "oplus,gauge_ic", i);
		if (!ic) {
			chg_err("ic is NULL for index %d\n", i);
			continue;
		}
		memset(extracted_name, 0, sizeof(extracted_name));
		rc = sscanf(ic->manu_name, "gauge-%127[^:]", extracted_name);
		chg_debug("%s i=%d %s extracted_name=%s rc=%d\n", chip->child_list[0].ic_dev->manu_name, i,
			 ic->manu_name, extracted_name, rc);
		if (rc != 1)
			continue;
		for (j = 0; j < ARRAY_SIZE(physical_gauge_name_array); j++) {
			if (!strcmp(extracted_name, physical_gauge_name_array[j])) {
				snprintf(name, len, "%s", extracted_name);
				chg_debug("found %s\n", name);
				return 0;
			}
		}
	}

	return -ENODATA;
}

void oplus_gauge_set_batt_full(bool full)
{
	int rc;

	if (!g_mms_gauge)
		return;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL, full);
	if (rc < 0)
		chg_err("set battery full error, rc=%d\n", rc);
}

static void oplus_mms_gauge_set_batt_full_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip =
		container_of(work, struct oplus_mms_gauge, set_gauge_batt_full_work);
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_CHG_FULL,
				&data, false);
	chip->batt_full = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_TEMP_REGION,
				&data, false);
	chip->batt_temp_region = data.intval;

	if (chip->batt_temp_region >= TEMP_REGION_COLD &&
	    chip->batt_temp_region <= TEMP_REGION_NORMAL_HIGH &&
	    chip->batt_full) {
		oplus_gauge_set_batt_full(true);
	} else {
		oplus_gauge_set_batt_full(false);
	}
}

bool oplus_gauge_check_chip_is_null(void)
{
	if (!g_mms_gauge) {
		return true;
	} else {
		return false;
	}
}

int oplus_gauge_update_battery_dod0(void)
{
	int rc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_UPDATE_DOD0);
	if (rc < 0)
		chg_err("update battery dod0 error, rc=%d\n", rc);

	return 0;
}

int oplus_gauge_update_soc_smooth_parameter(void)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_mms_gauge is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH);
	if (rc < 0)
		chg_err("update soc smooth parameter, rc=%d\n", rc);

	return rc;
}

int oplus_gauge_get_battery_cb_status(void)
{
	int rc;
	int cb_status;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_CB_STATUS, &cb_status);
	if (rc < 0) {
		chg_err("update soc smooth parameter, rc=%d\n", rc);
		return 0;
	}

	return cb_status;
}

int oplus_gauge_get_i2c_err(void)
{
	return 0; /* nick.hu TODO */
}

void oplus_gauge_clear_i2c_err(void)
{
	/* nick.hu TODO */
}

int oplus_gauge_get_passedchg(int *val)
{
	int rc;

	if (!g_mms_gauge)
		return 0;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_PASSEDCHG, val);
	if (rc < 0) {
		chg_err("get passedchg error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int oplus_gauge_get_dod0(struct oplus_mms *mms, int index, int *val)
{
	struct oplus_mms_gauge *chip;
	int rc;

	if ((val == NULL) || (mms == NULL))
		return 0;

	chip = oplus_mms_get_drvdata(mms);
	if (!chip)
		return 0;

	if (is_support_parallel(chip)) {
		switch (index) {
		case 0:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
				OPLUS_IC_FUNC_GAUGE_GET_DOD0, 0, val);
			if (rc < 0) {
				chg_err("get main battery dod0 error, rc=%d\n", rc);
				return rc;
			}
			break;
		case 1:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
				OPLUS_IC_FUNC_GAUGE_GET_DOD0, 0, val);
			if (rc < 0) {
				chg_err("get sub battery dod0 error, rc=%d\n", rc);
				return rc;
			}
			break;
		default:
			break;
		}
	} else {
		rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_DOD0, index, val);
		if (rc < 0) {
			chg_err("get dod0 error, rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

int oplus_gauge_get_dod0_passed_q(struct oplus_mms *mms, int index, int *val)
{
	struct oplus_mms_gauge *chip;
	int rc;

	if ((val == NULL) || (mms == NULL))
		return 0;

	chip = oplus_mms_get_drvdata(mms);
	if (!chip)
		return 0;

	if (is_support_parallel(chip)) {
		switch (index) {
		case 0:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
				OPLUS_IC_FUNC_GAUGE_GET_DOD0_PASSED_Q, val);
			if (rc < 0) {
				chg_err("get main battery _dod0_passed_q error, rc=%d\n", rc);
				return rc;
			}
			break;
		case 1:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
				OPLUS_IC_FUNC_GAUGE_GET_DOD0_PASSED_Q, val);
			if (rc < 0) {
				chg_err("get sub battery _dod0_passed_q error, rc=%d\n", rc);
				return rc;
			}
			break;
		default:
			break;
		}
	} else {
		rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_DOD0_PASSED_Q, val);
		if (rc < 0) {
			chg_err("get _dod0_passed_q error, rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

int oplus_gauge_get_qmax(struct oplus_mms *mms, int index, int *val)
{
	struct oplus_mms_gauge *chip;
	int rc;

	if ((val == NULL) || (mms == NULL))
		return 0;

	chip = oplus_mms_get_drvdata(mms);
	if (!chip)
		return 0;

	if (is_support_parallel(chip)) {
		switch (index) {
		case 0:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
				OPLUS_IC_FUNC_GAUGE_GET_QMAX, 0, val);
			if (rc < 0) {
				chg_err("get main battery qmax error, rc=%d\n", rc);
				return rc;
			}
			break;
		case 1:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
				OPLUS_IC_FUNC_GAUGE_GET_QMAX, 0, val);
			if (rc < 0) {
				chg_err("get sub battery qmax error, rc=%d\n", rc);
				return rc;
			}
			break;
		default:
			break;
		}
	} else {
		rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_QMAX, index, val);
		if (rc < 0) {
			chg_err("get qmax error, rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

int oplus_gauge_get_qmax_passed_q(struct oplus_mms *mms, int index, int *val)
{
	struct oplus_mms_gauge *chip;
	int rc;

	if ((val == NULL) || (mms == NULL))
		return 0;

	chip = oplus_mms_get_drvdata(mms);
	if (!chip)
		return 0;

	if (is_support_parallel(chip)) {
		switch (index) {
		case 0:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
				OPLUS_IC_FUNC_GAUGE_GET_QMAX_PASSED_Q, val);
			if (rc < 0) {
				chg_err("get main battery qmax_passed_q error, rc=%d\n", rc);
				return rc;
			}
			break;
		case 1:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
				OPLUS_IC_FUNC_GAUGE_GET_QMAX_PASSED_Q, val);
			if (rc < 0) {
				chg_err("get sub battery qmax_passed_q error, rc=%d\n", rc);
				return rc;
			}
			break;
		default:
			break;
		}
	} else {
		rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_QMAX_PASSED_Q, val);
		if (rc < 0) {
			chg_err("get qmax_passed_q error, rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

int oplus_gauge_get_volt(struct oplus_mms *mms, int index, int *val)
{
	struct oplus_mms_gauge *chip;
	int rc;

	if ((val == NULL) || (mms == NULL))
		return 0;

	chip = oplus_mms_get_drvdata(mms);
	if (!chip)
		return 0;

	if (is_support_parallel(chip)) {
		switch (index) {
		case 0:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
				OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL, 0, val);
			if (rc < 0) {
				chg_err("get main battery volt error, rc=%d\n", rc);
				return rc;
			}
			break;
		case 1:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
				OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL, 0, val);
			if (rc < 0) {
				chg_err("get sub battery volt error, rc=%d\n", rc);
				return rc;
			}
			break;
		default:
			break;
		}
	} else {
		rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL, index, val);
		if (rc < 0) {
			chg_err("get volt error, rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

int oplus_gauge_get_gauge_type(struct oplus_mms *mms, int index, int *val)
{
	struct oplus_mms_gauge *chip;
	int rc;

	if ((val == NULL) || (mms == NULL))
		return 0;

	chip = oplus_mms_get_drvdata(mms);
	if (!chip)
		return 0;

	if (is_support_parallel(chip)) {
		switch (index) {
		case 0:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
				OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC, val);
			if (rc < 0) {
				chg_err("get main battery gauge_type error, rc=%d\n", rc);
				return rc;
			}
			break;
		case 1:
			rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
				OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC, val);
			if (rc < 0) {
				chg_err("get sub battery gauge_type error, rc=%d\n", rc);
				return rc;
			}
			break;
		default:
			break;
		}
	} else {
		rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC, val);
		if (rc < 0) {
			chg_err("get gauge_type error, rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

int oplus_gauge_dump_register(void)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_mms_gauge is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_REG_DUMP);
	if (rc == -ENOTSUPP)
		rc = 0;

	return rc;
}

int oplus_gauge_lock(void)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_mms_gauge is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_LOCK, true);
	if (rc == -ENOTSUPP)
		rc = 0;

	return rc;
}

int oplus_gauge_unlock(void)
{
	int rc;

	if (g_mms_gauge == NULL) {
		chg_err("g_mms_gauge is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_LOCK, false);
	if (rc == -ENOTSUPP)
		rc = 0;

	return rc;
}

bool oplus_gauge_is_locked(void)
{
	int rc;
	bool locked;

	if (g_mms_gauge == NULL) {
		chg_err("g_mms_gauge is NULL\n");
		return true;
	}

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_IS_LOCKED, &locked);
	if (rc == -ENOTSUPP)
		return false;
	else if (rc < 0)
		return true;

	return locked;
}

int oplus_gauge_get_batt_num(void)
{
	int rc;
	int num;

	if (!g_mms_gauge)
		return 1;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM, &num);
	if (rc < 0) {
		chg_err("can't get battery num, rc=%d\n", rc);
		return 1;
	}

	return num;
}

int oplus_get_gauge_type(void)
{
	int rc;
	int gauge_type = GAUGE_TYPE_UNKNOW;

	if (!g_mms_gauge)
		return GAUGE_TYPE_UNKNOW;

	rc = oplus_chg_ic_func(g_mms_gauge->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_GAUGE_TYPE, &gauge_type);
	if (rc < 0) {
		chg_err("can't get gauge type, rc=%d\n", rc);
		return GAUGE_TYPE_UNKNOW;
	}

	return gauge_type;
}

int oplus_gauge_set_seal_flag(struct oplus_mms *topic, int seal_flag)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_SEAL_FLAG, seal_flag);
			if (rc < 0 && rc != -ENOTSUPP) {
				chg_err("gauge[%d](%s): can't set seal flag, rc=%d\n", i, ic->manu_name, rc);
				break;
			}
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_SEAL_FLAG, seal_flag);
			if (rc < 0 && rc != -ENOTSUPP) {
				chg_err("gauge[%d](%s): can't set seal flag, rc=%d\n", i, ic->manu_name, rc);
			}
			break;
		}
	}

	return rc;
}

int oplus_gauge_sec_get_romid(struct oplus_mms *topic, uint8_t *romid, int *len)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;
	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SEC_GET_ROMID, romid, len);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec can't get romid, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SEC_GET_ROMID, romid, len);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec can't get romid, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}

	return rc;
}

int oplus_gauge_sec_write_page(struct oplus_mms *topic, int page_id, uint8_t *data, int len)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_WRITE_PAGE, page_id, data, len);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec can't write page, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_WRITE_PAGE, page_id, data, len);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec can't write page, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}

	return rc;
}

int oplus_gauge_sec_read_page(struct oplus_mms *topic, int page_id, uint8_t *data, int *len)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_READ_PAGE, page_id, data, len);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec can't read page, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_READ_PAGE, page_id, data, len);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec can't read page, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}

	return rc;
}

int oplus_gauge_sec_ecdsa(struct oplus_mms *topic, bool *val)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_ECDSA, val);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0 && rc != -ENOTSUPP)
				chg_err("gauge[%d](%s): sec ecdsa fail, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_ECDSA, val);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0 && rc != -ENOTSUPP)
				chg_err("gauge[%d](%s): sec ecdsa fail, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}

	return rc;
}

int oplus_gauge_sec_ecw(struct oplus_mms *topic, bool *val)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_ECW, val);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec ecw fail, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_ECW, val);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec ecw fail, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}

	return rc;
}

int oplus_gauge_sec_shutdown(struct oplus_mms *topic, bool *val)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_SHUTDOWN, val);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec shutdown fail, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_SHUTDOWN, val);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec shutdown fail, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}

	return rc;
}

int oplus_gauge_sec_set_prikey(struct oplus_mms *topic, int index, uint8_t *prikey, int len)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_SET_PRIKEY, index, prikey, len);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec set prikey fail, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_SET_PRIKEY, index, prikey, len);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec set prikey fail, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}

	return rc;
}

int oplus_gauge_sec_get_prikey_index(struct oplus_mms *topic, int *index)
{
	int rc = 0;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_GET_PRIKEY_INDEX, index);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec get prikey index fail, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic,
				OPLUS_IC_FUNC_GAUGE_SEC_GET_PRIKEY_INDEX, index);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): sec get prikey index fail, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}

	return rc;
}

static int oplus_mms_gauge_set_err_code(struct oplus_mms_gauge *chip,
					unsigned int err_code)
{
	struct mms_msg *msg;
	int rc;

	if (chip->err_code == err_code)
		return 0;

	chip->err_code = err_code;
	chg_info("set err_code=%08x\n", err_code);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  GAUGE_ITEM_ERR_CODE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish notify code msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static bool oplus_mms_gauge_get_batt_hmac(struct oplus_mms_gauge *chip)
{
	int rc = 0;
	int i;
	struct oplus_chg_ic_dev *ic;
	bool pass = false;
	bool result = true;

	if (!chip)
		return false;

	for (i = 0; i < chip->child_num; i++) {
		ic = chip->child_list[i].ic_dev;
		rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC, &pass);
		if (rc < 0) {
			result = false;
			chg_err("gauge[%d](%s): can't get battery hmac status, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
		result &= pass;
	}

	return result;
}

static bool oplus_mms_gauge_get_sn_match(struct oplus_mms_gauge *chip)
{
	int rc = 0;
	int i;
	struct oplus_chg_ic_dev *ic;
	bool match = false;
	bool result = true;

	if (!chip)
		return true;

	for (i = 0; i < chip->child_num; i++) {
		ic = chip->child_list[i].ic_dev;
		rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_SN_MATCH, &match);
		if (rc == -ENOTSUPP)
			return true;
		if (rc < 0) {
			result = false;
			chg_err("gauge[%d](%s): can't get sm match, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
		result &= match;
	}

	return result;
}

int oplus_gauge_get_fcl_support(struct oplus_mms *mms, bool *fcl_support)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}

	if (fcl_support == NULL) {
		chg_err("fcl_support is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip) {
		chg_err("chip is NULL.\n");
		return -EINVAL;
	}

	*fcl_support = chip->fcl.support;

	return rc;
}

int oplus_gauge_get_vb_offset(struct oplus_mms *mms, int *vb_offset)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}

	if (vb_offset == NULL) {
		chg_err("vb_offset is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip) {
		chg_err("chip is NULL.\n");
		return -EINVAL;
	}

	*vb_offset = chip->fcl_offset;

	return rc;
}

bool oplus_gauge_get_fcl_curr(int hw_vth, int sw_vth, int vbat, int *curr_dec, int *min_curr, bool *hw)
{
	int i;
	int volt_diff;
	struct oplus_mms_gauge *chip = g_mms_gauge;

	if (!chip || !curr_dec || !min_curr || !hw)
		return false;

	for (i = 0; i < chip->fcl.nums; i++) {
		if (i == 0)
			volt_diff = hw_vth - vbat;
		else
			volt_diff = sw_vth - vbat;

		if (chip->fcl.limits[i].volt_diff > volt_diff) {
			*curr_dec = chip->fcl.limits[i].curr_dec;
			*min_curr = chip->fcl.limits[i].min_curr;
			chg_info("i=%d[%d, %d, %d][%d, %d]\n", i, hw_vth, sw_vth, vbat, *curr_dec, *min_curr);
			if (i == 0)
				*hw = true;
			return true;
		}
	}

	return false;
}

static int oplus_gauge_fcl_curves_init(struct oplus_mms_gauge *chip)
{
	int rc;
	struct device_node *node;

	if (!chip)
		return -ENODEV;
	node = oplus_get_node_by_type(chip->dev->of_node);
	chip->fcl.support = of_property_read_bool(node, "oplus,fcl_support");
	rc = of_property_read_u32(node, "oplus,vb_offset", &chip->fcl_offset);
	rc = of_property_read_u32(node, "oplus,fcl_index", &chip->fcl.index);
	if (rc)
		chip->fcl.index = FCL_EXTERN_GAUGE;

	if (chip->fcl.index >= FCL_GAUGE_MAX)
		chip->fcl.index = FCL_EXTERN_GAUGE;

	memmove(chip->fcl.limits, gauge_fcl_table[chip->fcl.index], sizeof(gauge_fcl_table[FCL_EXTERN_GAUGE]));
	chip->fcl.nums = ARRAY_SIZE(gauge_fcl_table[FCL_EXTERN_GAUGE]);

	chg_info("[%d, %d, %d]", chip->fcl.support, chip->fcl.index, chip->fcl_offset);
	return 0;
}

static int oplus_mms_gauge_virq_register(struct oplus_mms_gauge *chip);
static int oplus_mms_gauge_topic_init(struct oplus_mms_gauge *chip);
static void oplus_mms_gauge_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip = container_of(dwork,
		struct oplus_mms_gauge, hal_gauge_init_work);
	struct device_node *node = chip->dev->of_node;
	struct device_node *level_shift_node = NULL;
	static int retry = OPLUS_CHG_IC_INIT_RETRY_MAX;
	int rc;
	int i = 0;
	int j;

	rc = of_property_read_u32(node, "oplus,gauge_ic_connect", &chip->connect_type);
	if (rc < 0)
		chip->connect_type = 0;
	/* only for project which define oplus,level_shift_ic in dtsi */
	level_shift_node = of_parse_phandle(node, "oplus,level_shift_ic", 0);
	if (level_shift_node) {
		chip->level_shift_ic = of_get_oplus_chg_ic(node, "oplus,level_shift_ic", 0);
		if (chip->level_shift_ic == NULL) {
			chg_err("not find level shift ic\n");
			goto init_try_again;
		}
		rc = oplus_chg_ic_func(chip->level_shift_ic, OPLUS_IC_FUNC_INIT);
		if (rc == -EAGAIN) {
			chg_err("level_shift_ic init timeout\n");
			goto init_try_again;
		} else if (rc < 0) {
			chg_err("level_shift_ic init error, rc=%d\n", rc);
			retry = 0;
		}
	} else {
		chip->level_shift_ic = NULL;
	}

	rc = of_property_count_elems_of_size(node, "oplus,gauge_ic",
					     sizeof(u32));
	if (rc < 0) {
		chg_err("can't get gauge ic number, rc=%d\n", rc);
		goto init_try_again;
	}
	chip->child_num = rc;
	chip->child_list = devm_kzalloc(
		chip->dev,
		sizeof(struct oplus_virtual_gauge_child) * chip->child_num,
		GFP_KERNEL);
	if (chip->child_list == NULL) {
		rc = -ENOMEM;
		chg_err("alloc child ic memory error\n");
		goto init_try_again;
	}
	for (i = 0; i < chip->child_num; i++) {
		chip->child_list[i].ic_dev = of_get_oplus_chg_ic(node, "oplus,gauge_ic", i);
		if (chip->child_list[i].ic_dev == NULL) {
			chg_err("not find gauge ic %d\n", i);
			goto init_try_again;
		}
		if (chip->child_num >= GAUGE_PARALLEL_IC_NUM_MIN) {
			rc = of_property_read_u32_index(
				node, "oplus,gauge_ic_capacity_ratio", i,
				&chip->child_list[i].capacity_ratio);
			if (rc < 0) {
				chg_err("can't read ic[%d] current ratio, rc=%d\n", i,
				       rc);
				chip->child_list[i].capacity_ratio = 50;
			}
		}

		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev, OPLUS_IC_FUNC_INIT);
		if (rc == -EAGAIN) {
			chg_err("gauge_ic[%s] %d init timeout\n",
				 chip->child_list[i].ic_dev->name, i);
			goto init_try_again;
		} else if (rc < 0) {
			chg_err("gauge_ic[%s] %d init error, rc=%d\n",
				 chip->child_list[i].ic_dev->name, i, rc);
			retry = 0;
			goto init_error;
		}
	}
	retry = 0;
	rc = of_property_read_u32(node, "oplus,main_gauge", &chip->main_gauge);
	if (rc < 0) {
		chg_err("can't get main charger index, rc=%d\n", rc);
		chip->main_gauge = 0;
	}
	chip->gauge_ic = chip->child_list[chip->main_gauge].ic_dev;
	chip->gauge_ic_comb[chip->main_gauge] = chip->child_list[chip->main_gauge].ic_dev;
	if (chip->child_num >= GAUGE_PARALLEL_IC_NUM_MIN) {
		for (j = 0;j < chip->child_num; j++){
			if (j != chip->main_gauge) {
				chip->sub_gauge |= BIT(j);
			}
		}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
		chg_err("sub_gauge: %u\n", __ffs(chip->sub_gauge));
#else
		chg_err("sub_gauge: %lu\n", __ffs(chip->sub_gauge));
#endif
		chip->gauge_ic_comb[__ffs(chip->sub_gauge)] = chip->child_list[__ffs(chip->sub_gauge)].ic_dev;
	} else {
		chip->sub_gauge = 0;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic,
			       OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE,
			       &chip->device_type);
	if (rc < 0) {
		chg_err("can't get device type, rc=%d\n", rc);
		chip->device_type = 0;
	}
	rc = oplus_chg_ic_func(chip->gauge_ic,
			       OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_VOOC,
			       &chip->device_type_for_vooc);
	if (rc < 0) {
		chg_err("can't get device type for vooc, rc=%d\n", rc);
		chip->device_type_for_vooc = 0;
	}
	chip->hmac = oplus_mms_gauge_get_batt_hmac(chip);
	chip->sn_match = oplus_mms_gauge_get_sn_match(chip);
	chip->parallel_hamc = true;

	rc = of_property_read_u32(node, "oplus,sub_btb_curr_limit", &chip->sub_btb_curr_limit);
	if (rc < 0)
		chip->sub_btb_curr_limit = BATT_SUB_BTB_ABNORMAL_MAX_CURR;

	chip->support_subboard_ntc = of_property_read_bool(node, "oplus,support_subboard_ntc");
	chg_info("hmac=%d, support_subboard_ntc=%d, sub_btb_curr_limit=%d, sn_match=%d\n",
		  chip->hmac, chip->support_subboard_ntc, chip->sub_btb_curr_limit, chip->sn_match);

	chip->check_subboard_ntc_err = false;

	oplus_gauge_parse_deep_spec(chip);

	oplus_gauge_fcl_curves_init(chip);

	oplus_mms_gauge_virq_register(chip);
	g_mms_gauge = chip;

	(void)oplus_mms_gauge_topic_init(chip);

	return;
init_try_again:
	if (retry > 0) {
		retry--;
		schedule_delayed_work(&chip->hal_gauge_init_work,
			msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
	} else {
		chg_err("oplus,gauge_ic not found\n");
	}
init_error:
	if (chip->child_list) {
		for (; i >= 0; i--)
			chip->child_list[i].ic_dev = NULL;
		devm_kfree(chip->dev, chip->child_list);
	}
	return;
}

static void oplus_mms_gauge_err_analyze(struct oplus_mms_gauge *chip,
					struct oplus_chg_ic_err_msg *msg)
{
	oplus_mms_gauge_set_err_code(chip, chip->err_code | BIT(msg->type));
}

static void oplus_mms_gauge_err_handler_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip = container_of(work, struct oplus_mms_gauge,
						err_handler_work);
	struct oplus_chg_ic_err_msg *msg, *tmp;
	struct list_head msg_list;
	int i;

	INIT_LIST_HEAD(&msg_list);
	for (i = 0; i < chip->child_num; i++) {
		spin_lock(&chip->gauge_ic_comb[i]->err_list_lock);
		if (!list_empty(&chip->gauge_ic_comb[i]->err_list)) {
			list_replace_init(&chip->gauge_ic_comb[i]->err_list, &msg_list);
		} else {
			spin_unlock(&chip->gauge_ic_comb[i]->err_list_lock);
			continue;
		}
		spin_unlock(&chip->gauge_ic_comb[i]->err_list_lock);
		list_for_each_entry_safe(msg, tmp, &msg_list, list) {
			if (is_err_topic_available(chip)) {
				oplus_mms_publish_ic_err_msg(chip->err_topic,
							     ERR_ITEM_IC, msg);
			}
			if (msg->type == OPLUS_IC_ERR_I2C && chip->level_shift_ic)
				oplus_chg_ic_func(chip->level_shift_ic, OPLUS_IC_FUNC_REG_DUMP);
			oplus_mms_gauge_err_analyze(chip, msg);
			oplus_print_ic_err(msg);
			list_del(&msg->list);
			kfree(msg);
		}
	}
}

static void oplus_mms_level_shift_err_handler_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip = container_of(work, struct oplus_mms_gauge,
						ls_err_handler_work);
	struct oplus_chg_ic_err_msg *msg, *tmp;
	struct list_head msg_list;

	if (!chip->level_shift_ic)
		return;

	INIT_LIST_HEAD(&msg_list);
	spin_lock(&chip->level_shift_ic->err_list_lock);
	if (!list_empty(&chip->level_shift_ic->err_list))
		list_replace_init(&chip->level_shift_ic->err_list, &msg_list);
	spin_unlock(&chip->level_shift_ic->err_list_lock);

	list_for_each_entry_safe(msg, tmp, &msg_list, list) {
		if (is_err_topic_available(chip))
			oplus_mms_publish_ic_err_msg(chip->err_topic,
						     ERR_ITEM_IC, msg);
		oplus_chg_ic_func(chip->level_shift_ic, OPLUS_IC_FUNC_REG_DUMP);
		oplus_print_ic_err(msg);
		list_del(&msg->list);
		kfree(msg);
	}
}

static void oplus_mms_level_shift_fpga_rst_handler_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip = container_of(work, struct oplus_mms_gauge,
						ls_fpga_rst_handler_work);
	union mms_msg_data data = { 0 };
	int rc;

	if (!chip->level_shift_ic)
		return;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_HMAC, &data, false);
	if (!rc && !!data.intval == false) {
		chip->hmac = oplus_mms_gauge_get_batt_hmac(chip);
		chip->sn_match = oplus_mms_gauge_get_sn_match(chip);
		chg_info("retry hmac=%d sn_match=%d\n", chip->hmac, chip->sn_match);
		if (!!data.intval != (chip->hmac && chip->sn_match))
			oplus_mms_gauge_push_hmac(chip);
	}
}

static void oplus_mms_gauge_online_handler_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip = container_of(
		work, struct oplus_mms_gauge, online_handler_work);
	struct mms_msg *msg;
	unsigned int err_code;
	int rc;

	err_code = chip->err_code;
	if (err_code & BIT(OPLUS_ERR_CODE_I2C))
		err_code &= ~BIT(OPLUS_ERR_CODE_I2C);
	oplus_mms_gauge_set_err_code(chip, err_code);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  GAUGE_ITEM_BATT_EXIST);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish batt exist msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("batt_exist=%d\n", oplus_gauge_is_exist(chip->gauge_topic));
}

static void oplus_mms_gauge_offline_handler_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip = container_of(
		work, struct oplus_mms_gauge, offline_handler_work);
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  GAUGE_ITEM_BATT_EXIST);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish batt exist msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("batt_exist=%d\n", oplus_gauge_is_exist(chip->gauge_topic));
}

static void oplus_mms_gauge_resume_handler_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip = container_of(
		work, struct oplus_mms_gauge, resume_handler_work);
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  GAUGE_ITEM_RESUME, 1);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish gauge remuse msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_mms_gauge_err_handler(struct oplus_chg_ic_dev *ic_dev,
					void *virq_data)
{
	struct oplus_mms_gauge *chip = virq_data;
	schedule_work(&chip->err_handler_work);
}

static void oplus_mms_level_shift_err_handler(struct oplus_chg_ic_dev *ic_dev,
					void *virq_data)
{
	struct oplus_mms_gauge *chip = virq_data;
	schedule_work(&chip->ls_err_handler_work);
}

static void oplus_mms_level_shift_fpga_rst_handler(struct oplus_chg_ic_dev *ic_dev,
					void *virq_data)
{
	struct oplus_mms_gauge *chip = virq_data;
	schedule_work(&chip->ls_fpga_rst_handler_work);
}

static void oplus_mms_gauge_online_handler(struct oplus_chg_ic_dev *ic_dev,
					    void *virq_data)
{
	struct oplus_mms_gauge *chip = virq_data;
	schedule_work(&chip->online_handler_work);
}

static void oplus_mms_gauge_offline_handler(struct oplus_chg_ic_dev *ic_dev,
					    void *virq_data)
{
	struct oplus_mms_gauge *chip = virq_data;
	schedule_work(&chip->offline_handler_work);
}

static void oplus_mms_gauge_resume_handler(struct oplus_chg_ic_dev *ic_dev,
					    void *virq_data)
{
	struct oplus_mms_gauge *chip = virq_data;
	schedule_work(&chip->resume_handler_work);
}

static int oplus_gauge_get_batt_sub_btb_state(struct oplus_mms_gauge *chip)
{
	int rc;
	enum oplus_sub_btb_state state = BATT_BTB_STATE_NOT_CONNECT;

	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->gauge_ic,
			       OPLUS_IC_FUNC_GAUGE_GET_SUB_BTB_CONNECT_STATE, &state);
	if (rc < 0) {
		if (rc != -ENOTSUPP)
			chg_err("get battery sub_btb state error, rc=%d\n", rc);
		return rc;
	}

	return state;
}

static void oplus_mms_gauge_publish_sub_btb_state(struct oplus_mms_gauge *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, GAUGE_ITEM_SUB_BTB_STATE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish gauge btb state change msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static int oplus_mms_gauge_update_sub_btb_state(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip = NULL;
	int state;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip)
		return -EINVAL;

	state = oplus_gauge_get_batt_sub_btb_state(chip);
	if (state < 0)
		return -EINVAL;

	data->intval = state;

	return 0;
}

static void oplus_mms_gauge_sub_btb_state_change_handler_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip = container_of(
		work, struct oplus_mms_gauge, sub_btb_state_change_handler_work);
	int state = BATT_BTB_STATE_NOT_CONNECT;

	state = oplus_gauge_get_batt_sub_btb_state(chip);
	chg_info("state = %d", state);

	oplus_mms_gauge_publish_sub_btb_state(chip);
}


static void oplus_mms_gauge_sub_btb_state_change_handler(struct oplus_chg_ic_dev *ic_dev,
					    void *virq_data)
{
	struct oplus_mms_gauge *chip = virq_data;
	schedule_work(&chip->sub_btb_state_change_handler_work);
}

static void oplus_mms_gauge_hmac_update_handler_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip = container_of(
	    work, struct oplus_mms_gauge, hmac_update_handler_work);
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_HMAC, &data, false);
	if (!rc) {
		chip->hmac = oplus_mms_gauge_get_batt_hmac(chip);
			chip->sn_match = oplus_mms_gauge_get_sn_match(chip);
			chg_info("update hmac=%d sn_match=%d\n", chip->hmac, chip->sn_match);
			if (!!data.intval != (chip->hmac && chip->sn_match))
				oplus_mms_gauge_push_hmac(chip);
	}
}

static void oplus_mms_gauge_hmac_update_handler(struct oplus_chg_ic_dev *ic_dev,
					void *virq_data)
{
	struct oplus_mms_gauge *chip = virq_data;
	schedule_work(&chip->hmac_update_handler_work);
}

static void oplus_gauge_cuv_state_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, gauge_cuv_state_work);
	int state = 0;
	int rc;
	int try = 0;

	/* check the cuv state when bootup complete. */
	rc = oplus_gauge_get_cuv_state(chip->gauge_topic, &state);
	if (rc == -ENOTSUPP)
		return;

	chg_info("state = %d", state);
	if (state > 0)
		chg_info("the cuv_state = %d writecuv failed when shutdown.\n", state);

	while (try < OPLUS_GAUGE_CUV_STATE_CHECK_TRY_MAX) {
		msleep(1000);
		/* set the cuv state to CUV_2 when boot up. */
		rc = oplus_gauge_set_cuv_state(chip->gauge_topic, OPLUS_GAUGE_CUV_STATE_2);
		if (rc == -ENOTSUPP)
			return;

		msleep(1000);  /* wait the cuv statue is set success. */
		rc = oplus_gauge_get_cuv_state(chip->gauge_topic, &state);
		if (state != OPLUS_GAUGE_CUV_STATE_2 || rc != 0)
			chg_info("set cuv_state_2 %d failed rc = 0x%x \n", state, rc);
		else {
			chg_info("set cuv_state_2 %d success \n", state);
			break;
		}
		try++;
	}

	if ((state != OPLUS_GAUGE_CUV_STATE_2) &&
		(try >= OPLUS_GAUGE_CUV_STATE_CHECK_TRY_MAX)) {
		chg_info("send cuv_state config err msg!\n");
		oplus_chg_ic_creat_err_msg(chip->child_list[chip->main_gauge].ic_dev,
				OPLUS_IC_ERR_GAUGE, 0, "boot_up:cuv_state[%d]", state);
		oplus_chg_ic_virq_trigger(chip->child_list[chip->main_gauge].ic_dev,
				OPLUS_IC_VIRQ_ERR);
	}
}

static int oplus_mms_gauge_virq_register(struct oplus_mms_gauge *chip)
{
	int i, rc;
	for (i = 0; i < chip->child_num; i++) {
		if (chip->child_list[i].ic_dev)
			chg_info("child[%d] ic_dev->name %s",
				 i, chip->child_list[i].ic_dev->name);

		rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
			OPLUS_IC_VIRQ_ERR,
			oplus_mms_gauge_err_handler, chip);
		if (rc < 0)
			chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);

		rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
			OPLUS_IC_VIRQ_ONLINE,
			oplus_mms_gauge_online_handler, chip);
		if (rc < 0)
			chg_err("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);

		rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
			OPLUS_IC_VIRQ_OFFLINE,
			oplus_mms_gauge_offline_handler, chip);
		if (rc < 0)
			chg_err("register OPLUS_IC_VIRQ_OFFLINE error, rc=%d", rc);

		rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
			OPLUS_IC_VIRQ_RESUME,
			oplus_mms_gauge_resume_handler, chip);
		if (rc < 0)
			chg_err("register OPLUS_IC_VIRQ_RESUME error, rc=%d", rc);
		rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
			OPLUS_IC_VIRQ_BTB_STATE_CHANGE,
			oplus_mms_gauge_sub_btb_state_change_handler, chip);
		if (rc < 0)
			chg_err("register OPLUS_IC_VIRQ_BTB_STATE_CHANGE error, rc=%d", rc);
		rc = oplus_chg_ic_virq_register(chip->child_list[i].ic_dev,
			OPLUS_IC_VIRQ_HMAC_UPDATE,
			oplus_mms_gauge_hmac_update_handler, chip);
		if (rc < 0)
			chg_err("register OPLUS_IC_VIRQ_HMAC_UPDATE error, rc=%d", rc);
	}

	if (chip->level_shift_ic) {
		rc = oplus_chg_ic_virq_register(chip->level_shift_ic, OPLUS_IC_VIRQ_ERR,
			oplus_mms_level_shift_err_handler, chip);
		if (rc < 0)
			chg_err("register level shift OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
		rc = oplus_chg_ic_virq_register(chip->level_shift_ic, OPLUS_IC_VIRQ_FPGA_RST,
			oplus_mms_level_shift_fpga_rst_handler, chip);
		if (rc < 0)
			chg_err("register level shift OPLUS_IC_VIRQ_FPGA_RST error, rc=%d", rc);
	}

	return 0;
}

static int oplus_mms_gauge_update_soc_centi(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int soc = -EINVAL, rc = -EINVAL, main_soc, sub_soc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC_CENTI, &main_soc);
	if (rc < 0)
		goto rc_err;

	if (main_soc < 0)
		goto value_err;

	if (chip->sub_gauge) {
		rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
			OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC_CENTI, &sub_soc);
		if (rc < 0)
			goto rc_err;

		if (sub_soc < 0)
			goto value_err;

		soc = (main_soc * chip->child_list[chip->main_gauge].capacity_ratio +
		    sub_soc * chip->child_list[__ffs(chip->sub_gauge)].capacity_ratio) / 100;
	} else {
		soc = main_soc;
	}

value_err:
	data->intval = soc;
	return 0;

rc_err:
	data->intval = rc;
	return 0;
}

static int oplus_mms_gauge_update_soc(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int soc;
	int rc;
	int main_soc, sub_soc, soc_remainder;

	if (mms == NULL) {
		chg_err("mms is NULL");
		soc = DEFAULT_SOC;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC, &main_soc);
	if (rc < 0) {
		chg_err("get battery soc error, rc=%d\n", rc);
		soc = DEFAULT_SOC;
		goto end;
	}
	if (chip->sub_gauge) {
		rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
			OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC, &sub_soc);
		if (rc < 0) {
			chg_err("get battery sub soc error, rc=%d\n", rc);
			soc = DEFAULT_SOC;
			goto end;
		}
		soc_remainder = (main_soc * chip->child_list[chip->main_gauge].capacity_ratio +
			sub_soc * chip->child_list[__ffs(chip->sub_gauge)].capacity_ratio) % 100;
		soc = (main_soc * chip->child_list[chip->main_gauge].capacity_ratio +
			sub_soc * chip->child_list[__ffs(chip->sub_gauge)].capacity_ratio) / 100;
		chg_info(" main_soc:%d, sub_soc:%d, main_ratio:%d, sub_ratio:%d, soc:%d, remainder:%d",
			main_soc, sub_soc, chip->child_list[chip->main_gauge].capacity_ratio,
			chip->child_list[__ffs(chip->sub_gauge)].capacity_ratio, soc, soc_remainder);
		if (soc_remainder != 0) {
			soc = soc + 1;
		}
	} else {
		soc = main_soc;
	}

end:
	data->intval = soc;
	return 0;
}

static int oplus_mms_sub_gauge_update_soc(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int soc;
	int rc;
	unsigned long gauge_index;

	if (mms == NULL) {
		chg_err("mms is NULL");
		soc = DEFAULT_SOC;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (strcmp("gauge:0", mms->desc->name) == 0)
		gauge_index = chip->main_gauge;
	else
		gauge_index = __ffs(chip->sub_gauge);
	if (!chip->gauge_ic_comb[gauge_index]) {
		chg_err("sub_gauge_ic is NULL");
		soc = 0;
		goto end;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[gauge_index],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC, &soc);
	if (rc < 0) {
		chg_err("get battery soc error, rc=%d\n", rc);
		soc = DEFAULT_SOC;
	}

end:
	data->intval = soc;
	return 0;
}

static int oplus_mms_sub_gauge_update_cc(
			struct oplus_mms *mms, union mms_msg_data *data)
{
	int rc = -1;
	int i;
	int cc = 0;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);

	for (i = 0; i < chip->child_num; i++) {
		if (mms != chip->gauge_topic_parallel[i])
			continue;
		ic = chip->gauge_ic_comb[i];
		rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_CC, &cc);
		if (rc < 0)
			chg_err("gauge[%d](%s): can't get gauge cc, rc=%d\n", i, ic->manu_name, rc);
		break;
	}

	if (rc == 0)
		data->intval = cc;

	return rc;
}

static bool is_voocphy_ic_available(struct oplus_mms_gauge *chip)
{
	if (!chip->voocphy_ic)
		chip->voocphy_ic = of_get_oplus_chg_ic(chip->dev->of_node,
						       "oplus,voocphy_ic", 0);

	return !!chip->voocphy_ic;
}

#define VBATT_OVER_THR_MV 4600
#define MAX_CP_GAUGE_VBATT_DIFF 800
static int oplus_mms_gauge_choice_fit_vol(struct oplus_mms_gauge *chip, int gauge_vol)
{
	int ret;
	int cp_vbat;
	int vbatt_ov_thr_mv;

	if (chip->comm_topic)
		vbatt_ov_thr_mv = oplus_comm_get_vbatt_over_threshold(chip->comm_topic);
	else
		vbatt_ov_thr_mv = VBATT_OVER_THR_MV;

	ret = oplus_chg_ic_func(chip->voocphy_ic,
				OPLUS_IC_FUNC_VOOCPHY_GET_CP_VBAT,
				&cp_vbat);
	if (ret < 0) {
		chg_err("can't get cp voltage, rc=%d\n", ret);
		return gauge_vol;
	}

	if (cp_vbat >= vbatt_ov_thr_mv || cp_vbat <= 0 ||
	    abs(cp_vbat - gauge_vol) > MAX_CP_GAUGE_VBATT_DIFF) {
		chg_info("choice gauge voltage as vbat [%d, %d]\n",
			 cp_vbat, gauge_vol);
		return gauge_vol;
	} else {
		chg_info("choice cp voltage as vbat [%d, %d]\n",
			 cp_vbat, gauge_vol);
		return cp_vbat;
	}
}

static int oplus_mms_gauge_update_vol_max(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int vol;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (gauge_dbg_vbat != 0) {
		chg_info("debug enabled, voltage gauge_dbg_vbat[%d]\n", gauge_dbg_vbat);
		data->intval = gauge_dbg_vbat;
		return 0;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol);
	if (rc < 0) {
		chg_err("get battery voltage max error, rc=%d\n", rc);
		vol = GAUGE_DEFAULT_VOLT_MV;
	}

	if (chip->wired_online && is_voocphy_ic_available(chip) &&
	    !is_support_parallel(chip))	/* parallel project don't use cp vol */
		vol = oplus_mms_gauge_choice_fit_vol(chip, vol);

	data->intval = vol;
	return 0;
}

static int oplus_mms_gauge_update_vol_fcl(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int vol = 0;
	int rc = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	if (!chip || !data)
		return -EINVAL;

	if (strcmp("gauge", mms->desc->name) != 0)
		return -ENOTSUPP;

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_FCL_VOLT, &vol);
	if (rc < 0)
		vol = 0;

	data->intval = vol;
	return 0;
}

static int oplus_mms_sub_gauge_update_vol_max(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int vol;
	int rc;
	unsigned long gauge_index;

	if (mms == NULL) {
		chg_err("mms is NULL");
		vol = 0;
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	if (strcmp("gauge:0", mms->desc->name) == 0)
		gauge_index = chip->main_gauge;
	else
		gauge_index = __ffs(chip->sub_gauge);
	if (!chip->gauge_ic_comb[gauge_index]) {
		chg_err("sub_gauge_ic is NULL");
		vol = 0;
		goto end;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[gauge_index],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol);
	if (rc < 0) {
		chg_err("get battery voltage max error, rc=%d\n", rc);
		vol = 0;
	}

end:
	data->intval = vol;
	return 0;
}

static int oplus_mms_gauge_update_vol_min(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int vol;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (gauge_dbg_vbat != 0) {
		chg_info("debug enabled, voltage gauge_dbg_vbat[%d]\n", gauge_dbg_vbat);
		data->intval = gauge_dbg_vbat;
		return 0;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN, &vol);
	if (rc < 0) {
		chg_err("get battery voltage min error, rc=%d\n", rc);
		vol = GAUGE_DEFAULT_VOLT_MV;
	}

	if (chip->wired_online && is_voocphy_ic_available(chip))
		vol = oplus_mms_gauge_choice_fit_vol(chip, vol);

	data->intval = vol;
	return 0;
}

static int oplus_mms_gauge_update_gauge_vbat(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int vol;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (gauge_dbg_vbat != 0) {
		chg_info("debug enabled, voltage gauge_dbg_vbat[%d]\n", gauge_dbg_vbat);
		data->intval = gauge_dbg_vbat;
		return 0;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX, &vol);
	if (rc < 0) {
		chg_err("get battery voltage min error, rc=%d\n", rc);
		vol = GAUGE_DEFAULT_VOLT_MV;
	}

	data->intval = vol;
	return 0;
}

static int oplus_mms_gauge_update_curr(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int curr = 0;
	int rc;
	int main_curr = 0, sub_curr = 0;

	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	if (mms == NULL) {
		chg_err("mms is NULL");
		curr = 0;
		goto end;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &main_curr);
	if (rc < 0)
		main_curr = 0;
	if (chip->sub_gauge) {
		rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
			OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &sub_curr);
		if (rc < 0)
			sub_curr = 0;
	}
	if (chip->connect_type == OPLUS_CHG_IC_CONNECT_SERIAL)
		curr = (main_curr + sub_curr) / 2;
	else
		curr = main_curr + sub_curr;
end:
	data->intval = curr;
	return 0;
}

static int oplus_mms_sub_gauge_update_curr(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int curr;
	int rc;
	unsigned long gauge_index;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	if (strcmp("gauge:0", mms->desc->name) == 0)
		gauge_index = chip->main_gauge;
	else
		gauge_index = __ffs(chip->sub_gauge);
	if (!chip->gauge_ic_comb[gauge_index]) {
		chg_err("sub_gauge_ic is NULL");
		curr = 0;
		goto end;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[gauge_index],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR, &curr);
	if (rc < 0) {
		chg_err("get battery current error, rc=%d\n", rc);
		curr = 0;
	}
end:
	data->intval = curr;
	return 0;
}

static int oplus_mms_gauge_push_subboard_temp_err(struct oplus_mms_gauge *chip, bool err)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, GAUGE_ITEM_SUBBOARD_TEMP_ERR, err);
	if (msg == NULL) {
		chg_err("alloc battery subboard msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish battery subboard msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

#define SUBBOARD_LOW_ABNORMAL_TEMP (-300)
#define SUBBOARD_HIGH_ABNORMAL_TEMP 1000
#define GAUGE_LOW_ABNORMAL_TEMP (-200)
static void oplus_mms_subboard_ntc_err_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip =
		container_of(dwork, struct oplus_mms_gauge, subboard_ntc_err_work);
	int subboard_temp, gauge_temp;
	int err_count = 0;

	do {
		subboard_temp = oplus_gauge_get_subboard_temp(chip);
		gauge_temp = oplus_gauge_get_batt_temperature(chip);
		chg_info("subboard_temp:%d, gauge_temp:%d, err_count:%d\n",
			 subboard_temp, gauge_temp, err_count);
		usleep_range(10000, 11000);
		if (((subboard_temp <= SUBBOARD_LOW_ABNORMAL_TEMP) &&
		     (gauge_temp >= GAUGE_LOW_ABNORMAL_TEMP)) ||
		    (subboard_temp >= SUBBOARD_HIGH_ABNORMAL_TEMP))
			err_count++;
		else
			break;
	} while (err_count <= ERR_COUNT_MAX);

	if (err_count >= ERR_COUNT_MAX) {
		chip->check_subboard_ntc_err = true;
		oplus_mms_gauge_push_subboard_temp_err(chip, true);
		chg_err("send subboard temp err msg!\n");
		oplus_chg_ic_creat_err_msg(
		chip->child_list[chip->main_gauge].ic_dev, OPLUS_IC_ERR_GAUGE, 0, "bad_subboard[%d]", subboard_temp);
		oplus_chg_ic_virq_trigger(chip->child_list[chip->main_gauge].ic_dev, OPLUS_IC_VIRQ_ERR);
	}
}

#define SUBBOARD_NTC_ERR_CHECK 100
static int oplus_mms_gauge_update_temp(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int temp, gauge_temp;
	static int last_temp = GAUGE_INVALID_TEMP;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == FACTORY && chip->support_subboard_ntc) {
		temp = oplus_gauge_get_subboard_temp(chip);
		chg_info("is factory mode, use subboard temp directly! temp = %d\n", temp);
		data->intval = temp;
		return 0;
	}
#endif

	if (chip->support_subboard_ntc && !chip->check_subboard_ntc_err) {
		temp = oplus_gauge_get_subboard_temp(chip);
		gauge_temp = oplus_gauge_get_batt_temperature(chip);
		if (gauge_temp <= GAUGE_INVALID_TEMP) {
			temp = gauge_temp;
		} else if (((temp <= SUBBOARD_LOW_ABNORMAL_TEMP) &&
			    (gauge_temp >= GAUGE_LOW_ABNORMAL_TEMP)) ||
			   (temp >= SUBBOARD_HIGH_ABNORMAL_TEMP)) {
			if (!chip->check_subboard_ntc_err) {
				if (!(work_busy(&chip->subboard_ntc_err_work.work) & (WORK_BUSY_RUNNING)))
					schedule_delayed_work(&chip->subboard_ntc_err_work,
						msecs_to_jiffies(SUBBOARD_NTC_ERR_CHECK));
				if (last_temp <= GAUGE_INVALID_TEMP)
					temp = gauge_temp;
				else
					temp = last_temp;
			}
		} else {
			last_temp = temp;
		}
	} else {
		temp = oplus_gauge_get_batt_temperature(chip);
	}

	data->intval = temp;

	chg_debug("support_subboard_ntc = %d, temp = %d\n", chip->support_subboard_ntc, temp);
	return 0;
}

static int oplus_mms_subboard_temp_err(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (chip->support_subboard_ntc)
		data->intval = chip->check_subboard_ntc_err;
	else
		data->intval = 0;
	chg_debug("error_info = %d \n", data->intval);
	return 0;
}

static int oplus_mms_sub_gauge_update_temp(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int temp;
	int rc;
	unsigned long gauge_index;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	if (strcmp("gauge:0", mms->desc->name) == 0)
		gauge_index = chip->main_gauge;
	else
		gauge_index = __ffs(chip->sub_gauge);
	if (!chip->gauge_ic_comb[gauge_index]) {
		chg_err("sub_gauge_ic is NULL");
		temp = GAUGE_INVALID_TEMP;
		goto end;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[gauge_index],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP, &temp);
	if (rc < 0) {
		chg_err("get battery current error, rc=%d\n", rc);
		temp = GAUGE_INVALID_TEMP;
	}
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == HIGH_TEMP_AGING || oplus_is_ptcrb_version()) {
		chg_info("HIGH_TEMP_AGING, disable high tbat shutdown,temp %d -> 690\n", temp);
		if (temp > 690)
			temp = 690;
	}
#endif

end:
	data->intval = temp;
	return 0;
}

static int oplus_mms_gauge_get_reg_info(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int rc = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip->gauge_reg_info[0]) {
		chg_err("gauge_reg_info[0] is NULL");
		return -EINVAL;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_REG_INFO,
				chip->gauge_reg_info[0], GAUGE_REG_INFO_SIZE);
	if (rc == -ENOTSUPP)
		rc = 0;

	if (rc >= GAUGE_REG_INFO_SIZE)
		chip->gauge_reg_info[0][GAUGE_REG_INFO_SIZE - 1] = '\0';
	else if (rc > 0 && rc < GAUGE_REG_INFO_SIZE)
		chip->gauge_reg_info[0][rc] = '\0';

	data->strval = chip->gauge_reg_info[0];
	return rc;
}

static int oplus_mms_gauge_get_r_info(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int rc = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip || !chip->gauge_r_info[0]) {
		chg_err("gauge_r_info[0] is NULL");
		return -EINVAL;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_GAUGE_R_INFO,
				chip->gauge_r_info[0], GAUGE_REG_INFO_SIZE);
	if (rc == -ENOTSUPP)
		rc = 0;

	if (rc >= GAUGE_REG_INFO_SIZE)
		chip->gauge_r_info[0][GAUGE_REG_INFO_SIZE - 1] = '\0';
	else if (rc > 0 && rc < GAUGE_REG_INFO_SIZE)
		chip->gauge_r_info[0][rc] = '\0';

	data->strval = chip->gauge_r_info[0];
	return rc;
}

static int oplus_mms_sub_gauge_get_reg_info(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	unsigned long gauge_index;
	int rc = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (strcmp("gauge:0", mms->desc->name) == 0)
		gauge_index = chip->main_gauge;
	else
		gauge_index = __ffs(chip->sub_gauge);

	if (!chip->gauge_reg_info[gauge_index]) {
		chg_err("gauge_reg_info[%ld] is NULL", gauge_index);
		return -EINVAL;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[gauge_index], OPLUS_IC_FUNC_GAUGE_GET_REG_INFO,
				chip->gauge_reg_info[gauge_index], GAUGE_REG_INFO_SIZE);
	if (rc == -ENOTSUPP)
		rc = 0;

	if (rc >= GAUGE_REG_INFO_SIZE)
		chip->gauge_reg_info[gauge_index][GAUGE_REG_INFO_SIZE - 1] = '\0';
	else if (rc > 0 && rc < GAUGE_REG_INFO_SIZE)
		chip->gauge_reg_info[gauge_index][rc] = '\0';

	data->strval = chip->gauge_reg_info[gauge_index];
	return rc;
}

static int oplus_mms_gauge_get_lifetime_status(
				struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int rc = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	rc = oplus_chg_ic_func(chip->gauge_ic,
		OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_STATUS, &chip->lifetime[0]);

	data->strval = (char *)&chip->lifetime[0];

	return rc;
}

static int oplus_mms_sub_gauge_get_lifetime_status(
				struct oplus_mms *mms, union mms_msg_data *data)
{
	int gauge_index = 0;
	int rc = -1;
	int i;
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);

	for (i = 0; i < chip->child_num; i++) {
		if (mms != chip->gauge_topic_parallel[i])
			continue;
		gauge_index = i;
		ic = chip->gauge_ic_comb[i];
		rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_STATUS, &chip->lifetime[gauge_index]);
		if (rc < 0)
			chg_err("gauge[%d](%s): can't get gauge lifetime status, rc=%d\n", i, ic->manu_name, rc);
		break;
	}

	data->strval = (char *)&chip->lifetime[gauge_index];

	return rc;
}

static void oplus_mms_gauge_check_calib_time_update(struct oplus_mms *mms,
			int dod_calib_time, int qmax_calib_time, struct gauge_calib_info *calib_info)
{
	int i;
	bool update = false;
	bool calib_info_init = false;
	struct oplus_mms_gauge *chip;

	if (calib_info == NULL)
		return;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (chip->child_num > GAUGE_IC_NUM_MAX)
		return;

	if (mms == chip->gauge_topic) {
		calib_info_init = chip->calib_info_init[0];
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			calib_info_init = chip->calib_info_init[i];
			break;
		}
	}

	if (calib_info_init && (dod_calib_time != -1 && calib_info->dod_time != dod_calib_time)) {
		calib_info->dod_time = dod_calib_time;
		update = true;
	}

	if (calib_info_init && (qmax_calib_time != -1 && calib_info->qmax_time != qmax_calib_time)) {
		calib_info->qmax_time = qmax_calib_time;
		update = true;
	}

	if (update)
		queue_work(system_highpri_wq, &chip->set_reserve_calib_info_work);
}

static int oplus_mms_gauge_get_calib_time(struct oplus_mms *mms, union mms_msg_data *data)
{
	int rc = 0;
	struct oplus_mms_gauge *chip;
	struct gauge_calib_info *calib_info;
	int qmax_calib_time = -1, dod_calib_time = -1;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	calib_info = chip->calib_info_load.calib_info;
	rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_CALIB_TIME,
				&dod_calib_time, &qmax_calib_time, calib_info->calib_args, GAUGE_CALIB_ARGS_LEN);
	if (rc == -ENOTSUPP)
		rc = 0;

	oplus_mms_gauge_check_calib_time_update(mms, dod_calib_time, qmax_calib_time, calib_info);
	snprintf((char *)&(chip->calib_time_str[0]), CALIB_TIME_STR_LEN, "%d,%d", dod_calib_time, qmax_calib_time);
	data->strval = (char *)&(chip->calib_time_str[0]);

	return rc;
}

static int oplus_mms_sub_gauge_get_calib_time(struct oplus_mms *mms, union mms_msg_data *data)
{
	int i;
	int rc = 0;
	struct oplus_mms_gauge *chip;
	struct gauge_calib_info *calib_info;
	struct oplus_chg_ic_dev *ic;
	int qmax_calib_time = -1, dod_calib_time = -1;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (chip->child_num > GAUGE_IC_NUM_MAX)
		return rc;

	for (i = 0; i < chip->child_num; i++) {
		if (mms != chip->gauge_topic_parallel[i])
			continue;
		ic = chip->gauge_ic_comb[i];
		calib_info = &(chip->calib_info_load.calib_info[i]);
		rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_CALIB_TIME,
				&dod_calib_time, &qmax_calib_time, calib_info->calib_args, GAUGE_CALIB_ARGS_LEN);
		if (rc < 0)
			chg_err("gauge[%d](%s): can't get gauge calib time, rc=%d\n", i, ic->manu_name, rc);
		oplus_mms_gauge_check_calib_time_update(mms, dod_calib_time, qmax_calib_time, calib_info);
		snprintf((char *)&(chip->calib_time_str[i]), CALIB_TIME_STR_LEN, "%d,%d",  dod_calib_time, qmax_calib_time);
		data->strval = (char *)&(chip->calib_time_str[i]);
		break;
	}

	return rc;
}

static int oplus_mms_gauge_update_fcc(struct oplus_mms *mms, union mms_msg_data *data)
{
	int i;
	int rc;
	struct oplus_chg_ic_dev *ic;
	struct oplus_mms_gauge *chip;
	int fcc = 0, temp_fcc = 0;
	int batt_num = oplus_gauge_get_batt_num();

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (strcmp("gauge", mms->desc->name) == 0) {
		for (i = 0; i < chip->child_num; i++) {
			temp_fcc = 0;
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC, &temp_fcc);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get batt fcc, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}
			fcc += temp_fcc;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC, &fcc);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't get batt fcc, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}
	if (is_support_parallel(chip))
		data->intval = fcc;
	else
		data->intval = fcc * batt_num;

	return 0;
}

static int oplus_mms_gauge_update_rm(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int rm = 0;
	int main_rm = 0;
	int sub_rm = 0;
	int rc;
	int batt_num = oplus_gauge_get_batt_num();

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_RM, &main_rm);
	if (rc < 0) {
		chg_err("get battery remaining capacity error, rc=%d\n", rc);
		main_rm = 0;
	}
	if (chip->sub_gauge) {
		rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
			OPLUS_IC_FUNC_GAUGE_GET_BATT_RM, &sub_rm);
		if (rc < 0) {
			chg_err("get sub battery remaining capacity error, rc=%d\n", rc);
			sub_rm = 0;
		}
	}
	rm = main_rm + sub_rm;
	if (is_support_parallel(chip))
		data->intval = rm;
	else
		data->intval = rm * batt_num;

	return 0;
}

static int oplus_mms_gauge_update_cc(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int cc = 0;
	int main_cc = 0;
	int sub_cc = 0;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	rc = oplus_chg_ic_func(chip->gauge_ic_comb[chip->main_gauge],
		OPLUS_IC_FUNC_GAUGE_GET_BATT_CC, &main_cc);
	if ((rc < 0) || (main_cc <= 0)) {
		chg_err("get battery cc error, rc=%d\n", rc);
		main_cc = 0;
	}
	if (chip->sub_gauge) {
		rc = oplus_chg_ic_func(chip->gauge_ic_comb[__ffs(chip->sub_gauge)],
			OPLUS_IC_FUNC_GAUGE_GET_BATT_CC, &sub_cc);
		if ((rc < 0) || (sub_cc <= 0)) {
			chg_err("get sub battery cc error, rc=%d\n", rc);
			cc = main_cc;
		} else if (main_cc > 0 && sub_cc > 0) {
			cc = (main_cc * chip->child_list[chip->main_gauge].capacity_ratio +
				sub_cc * chip->child_list[__ffs(chip->sub_gauge)].capacity_ratio) / 100;
		} else {
			cc = sub_cc;
		}
		chg_info("main_cc:%d, sub_cc:%d, main_ratio:%d, sub_ratio:%d, cc:%d\n",
			 main_cc, sub_cc, chip->child_list[chip->main_gauge].capacity_ratio,
			 chip->child_list[__ffs(chip->sub_gauge)].capacity_ratio, cc);
	} else {
		cc = main_cc;
	}

	data->intval = cc;
	return 0;
}

static int oplus_mms_gauge_get_zy_gauge_soh(struct oplus_mms *mms, int gauge_type)
{
	int batt_soh = 0;
	int batt_qmax = 0;
	int batt_qmax_1 = 0;
	int batt_qmax_2 = 0;
	int batt_capacity_mah = 0;
	struct oplus_mms_gauge *chip;

	if (mms == NULL)
		return 0;

	chip = oplus_mms_get_drvdata(mms);
	if (chip == NULL)
		return 0;

	batt_capacity_mah = oplus_gauge_get_batt_capacity_mah(mms);
	if (batt_capacity_mah <= 0)
		return 0;

	if (DEVICE_ZY0603 == gauge_type) {
		oplus_gauge_get_qmax(mms, 0, &batt_qmax_1);
		oplus_gauge_get_qmax(mms, 1, &batt_qmax_2);

		batt_qmax = batt_qmax_1 + batt_qmax_2;
		batt_soh = batt_qmax * 100 / batt_capacity_mah;
		chg_info("series qmax_1:%d, qmax_2:%d, capacity_mah:%d, soh:%d, gauge_type:%d\n",
			batt_qmax_1, batt_qmax_2, batt_capacity_mah, batt_soh, gauge_type);
	} else if (DEVICE_ZY0602 == gauge_type) {
		if (is_support_parallel(chip)) {
			oplus_gauge_get_qmax(mms, 0, &batt_qmax_1);
			oplus_gauge_get_qmax(mms, 1, &batt_qmax_2);
			if (batt_qmax_1 > 0 && batt_qmax_2 >= 0) {
				batt_qmax = batt_qmax_1 + batt_qmax_2;
				batt_soh = batt_qmax * 100 / batt_capacity_mah;
			} else if (batt_qmax_1 > 0 && batt_qmax_2 <= 0 &&
			    chip->child_list[chip->main_gauge].capacity_ratio > 0) {
				batt_capacity_mah = batt_capacity_mah *
					chip->child_list[chip->main_gauge].capacity_ratio / 100;
				batt_soh = batt_qmax_1  / batt_capacity_mah;
			} else if (batt_qmax_1 <= 0 && batt_qmax_2 > 0 &&
			    chip->child_list[__ffs(chip->sub_gauge)].capacity_ratio > 0) {
				batt_capacity_mah = batt_capacity_mah *
					chip->child_list[__ffs(chip->sub_gauge)].capacity_ratio / 100;
				batt_soh = batt_qmax_2 / batt_capacity_mah;
			} else if (batt_qmax_1 <= 0 && batt_qmax_2 <= 0) {
				batt_soh = 0;
			}

			chg_info("parallel qmax_1:%d, qmax_2:%d, capacity_mah:%d, soh:%d, gauge_type:%d\n",
				batt_qmax_1, batt_qmax_2, batt_capacity_mah, batt_soh, gauge_type);
		} else {
			oplus_gauge_get_qmax(mms, 0, &batt_qmax);
			batt_soh = batt_qmax * 100 / batt_capacity_mah;
			chg_info("singal qmax:%d, capacity_mah:%d, soh:%d, gauge_type:%d\n",
				batt_qmax, batt_capacity_mah, batt_soh, gauge_type);
		}
	}

	if (batt_soh > 100)
		batt_soh = 100;

	return batt_soh;
}

#define SOH_LIST_NUMBER		5
static int oplus_mms_gauge_update_soh(struct oplus_mms *mms, union mms_msg_data *data)
{
	int i;
	int rc;
	int gauge_type = 0;
	struct oplus_chg_ic_dev *ic;
	struct oplus_mms_gauge *chip;
	int soh = 0, temp_soh;
	int pre_soh = 0;
	int soh_list[SOH_LIST_NUMBER] = {0, 0, 0, 0, 0};

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (chip == NULL)
		return  -EINVAL;

	oplus_gauge_get_gauge_type(mms, 0, &gauge_type);
	if (gauge_type == DEVICE_ZY0602 || gauge_type == DEVICE_ZY0603) {
		soh = oplus_mms_gauge_get_zy_gauge_soh(mms, gauge_type);
		data->intval = soh;
		return 0;
	}

	if (strcmp("gauge", mms->desc->name) == 0) {
		for (i = 0; i < chip->child_num; i++) {
			temp_soh = 0;
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH, &temp_soh);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get batt soh, rc=%d\n", i, ic->manu_name, rc);
				continue;
			}

			if (temp_soh > 0) {
				soh_list[i] = temp_soh;
				pre_soh = temp_soh;
			}
		}

		for (i = 0; i < chip->child_num; i++) {
			if (soh_list[i] <= 0) {
				soh_list[i] = pre_soh;
			}

			if (chip->child_list[i].capacity_ratio)
				soh += soh_list[i] * chip->child_list[i].capacity_ratio;
			else
				soh += soh_list[i] * 100;
		}
		soh /= 100;
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH, &soh);
			if (rc < 0)
				chg_err("gauge[%d](%s): can't get batt soh, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	}
	data->intval = soh;

	return 0;
}

static int oplus_mms_gauge_update_exist(struct oplus_mms *mms,
					union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	bool exist;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	rc = oplus_chg_ic_func(chip->gauge_ic,
			       OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST, &exist);
	if (rc < 0) {
		chg_err("get battery exist status error, rc=%d\n", rc);
		exist = false;
	}

	data->intval = exist;
	return 0;
}

static int oplus_mms_gauge_update_err_code(struct oplus_mms *mms,
					union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	unsigned int err_code;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	err_code = chip->err_code;

	data->intval = err_code;
	return 0;
}

static int oplus_mms_gauge_update_hmac(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	bool hmac = true;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	hmac = chip->hmac;

	if (is_support_parallel(chip) && chip->connect_type != OPLUS_CHG_IC_CONNECT_SERIAL)
		hmac = chip->parallel_hamc;

	data->intval = hmac && chip->sn_match;
	return 0;
}

static int oplus_mms_gauge_update_auth(struct oplus_mms *mms,
				       union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	bool auth;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	if (chip->bat_volt_different) {
		auth = false;
		goto end;
	}

	if (chip->deep_spec.sili_err) {
		auth = false;
		goto end;
	}

	if (is_support_parallel(chip)) {
		auth = true;
		goto end;
	}

	rc = oplus_chg_ic_func(chip->gauge_ic,
			       OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH, &auth);
	if (rc < 0) {
		chg_err("get battery authenticate status error, rc=%d\n", rc);
		auth = false;
	}

end:
	data->intval = auth;
	return 0;
}

static int oplus_mms_gauge_real_temp(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	int temp;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	temp = oplus_gauge_get_batt_temperature(chip);

	data->intval = temp;
	return 0;
}

static void oplus_mms_gauge_update(struct oplus_mms *mms, bool publish)
{
	struct oplus_mms_gauge *chip;
	struct mms_msg *msg;
	int i, rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}
	chip = oplus_mms_get_drvdata(mms);

	(void)oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_UPDATE);

	for (i = 0; i < mms->desc->update_items_num; i++)
		oplus_mms_item_update(mms, mms->desc->update_items[i], true);
	if (publish) {
		msg = oplus_mms_alloc_msg(MSG_TYPE_TIMER, MSG_PRIO_MEDIUM, 0);
		if (msg == NULL) {
			chg_err("alloc msg buf error\n");
			return;
		}
		rc = oplus_mms_publish_msg(mms, msg);
		if (rc < 0) {
			chg_err("publish msg error, rc=%d\n", rc);
			kfree(msg);
			return;
		}
	}
}

static int oplus_gauge_ic_get_qmax(struct oplus_chg_ic_dev *ic_dev, int *qmax)
{
	int rc;
	int batt_num;
	int temp_qmax;
	int i;

	if (ic_dev == NULL || qmax == NULL) {
		chg_err("ic_dev or qmax is NULL\n");
		return -EINVAL;
	}

	*qmax = 0;
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM, &batt_num);
	if (rc < 0) {
		chg_err("can't get batt_num from ic:%s, set default to 1\n", ic_dev->name);
		batt_num = 1;
	}
	for (i = 0; i < batt_num; i++) {
		rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_GAUGE_GET_QMAX, i, &temp_qmax);
		if (rc < 0) {
			if (rc != -ENOTSUPP)
				chg_err("can't get qmax from ic:%s, set default to 0\n", ic_dev->name);
			temp_qmax = 0;
		}
		*qmax += temp_qmax;
	}
	return 0;
}

static int oplus_mms_gauge_update_qmax(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;
	int batt_qmax = 0;
	int temp_qmax = 0;
	int rc = 0;
	int i;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (strcmp("gauge", mms->desc->name) == 0) {
		for (i = 0; i < chip->child_num; i++) {
			temp_qmax = 0;
			ic = chip->child_list[i].ic_dev;
			rc = oplus_gauge_ic_get_qmax(ic, &temp_qmax);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get batt qmax, rc=%d\n",
					i, ic->manu_name, rc);
				continue;
			}
			batt_qmax += temp_qmax;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (mms != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_gauge_ic_get_qmax(ic, &batt_qmax);
			if (rc < 0) {
				chg_err("gauge[%d](%s): can't get batt qmax, rc=%d\n", i, ic->manu_name, rc);
				batt_qmax = 0;
				return -EINVAL;
			}
			break;
		}
	}
	data->intval = batt_qmax;
	return 0;
}

static int oplus_mms_gauge_update_car_c(struct oplus_mms *topic, union mms_msg_data *data)
{
	struct oplus_mms_gauge *chip;
	struct oplus_chg_ic_dev *ic;
	int rc = 0;
	int i;
	int car_c = 0;
	int temp_car_c = 0;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -EINVAL;
	}

	if (data == NULL) {
		chg_err("data is NULL\n");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return -EINVAL;

	if (topic == chip->gauge_topic) {
		for (i = 0; i < chip->child_num; i++) {
			ic = chip->child_list[i].ic_dev;
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_GAUGE_CAR_C, &car_c);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): can't get gauge_car_c, rc=%d\n", i, ic->manu_name, rc);
			break;
		}
	} else {
		for (i = 0; i < chip->child_num; i++) {
			if (topic != chip->gauge_topic_parallel[i])
				continue;
			ic = chip->gauge_ic_comb[i];
			rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_GET_GAUGE_CAR_C, &temp_car_c);
			if (rc == -ENOTSUPP)
				continue;
			if (rc < 0)
				chg_err("gauge[%d](%s): can't get gauge_car_c, rc=%d\n", i, ic->manu_name, rc);
			car_c += temp_car_c;
			temp_car_c = 0;
			break;
		}
	}
	data->intval = car_c;
	return 0;
}

static void oplus_mms_sub_gauge_update(struct oplus_mms *mms, bool publish)
{
	struct mms_msg *msg;
	int i, rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}

	for (i = 0; i < mms->desc->update_items_num; i++)
		oplus_mms_item_update(mms, mms->desc->update_items[i], true);
	if (publish) {
		msg = oplus_mms_alloc_msg(MSG_TYPE_TIMER, MSG_PRIO_MEDIUM, 0);
		if (msg == NULL) {
			chg_err("alloc msg buf error\n");
			return;
		}
		rc = oplus_mms_publish_msg(mms, msg);
		if (rc < 0) {
			chg_err("publish msg error, rc=%d\n", rc);
			kfree(msg);
			return;
		}
	}
}

static struct mms_item oplus_mms_gauge_item[] = {
	{
		.desc = {
			.item_id = GAUGE_ITEM_SOC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_soc,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_VOL_MAX,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_vol_max,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_VOL_MIN,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_vol_min,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_GAUGE_VBAT,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_gauge_vbat,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_CURR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_curr,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_TEMP,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_temp,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_FCC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_fcc,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_RM,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_rm,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_CC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_cc,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SOH,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_soh,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_BATT_EXIST,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_exist,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_ERR_CODE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_err_code,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_RESUME,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_HMAC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_hmac,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_AUTH,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_auth,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_REAL_TEMP,
			.update = oplus_mms_gauge_real_temp,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SUBBOARD_TEMP_ERR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_subboard_temp_err,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_VBAT_UV,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_vbat_uv,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_DEEP_SUPPORT,
			.update = oplus_mms_gauge_get_si_prop,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_REG_INFO,
			.str_data = true,
			.update = oplus_mms_gauge_get_reg_info,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_CALIB_TIME,
			.str_data = true,
			.update = oplus_mms_gauge_get_calib_time,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_FCC_COEFF,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SOH_COEFF,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SPARE_POWER_ENABLE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_spare_power_enable,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SILI_IC_ALG_CFG,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SILI_IC_ALG_DSG_ENABLE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_sili_ic_alg_dsg_enable,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SILI_IC_ALG_TERM_VOLT,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_sili_ic_alg_term_volt_data,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_LIFETIME_STATUS,
			.str_data = true,
			.update = oplus_mms_gauge_get_lifetime_status,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_RATIO_VALUE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_deep_ratio,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_RATIO_TRANGE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_ratio_trange,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_QMAX,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_qmax,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_CAR_C,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_car_c,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_RATIO_LIMIT_CURR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_ratio_limit_curr,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SUB_BTB_STATE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_sub_btb_state,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_GAUGE_R_INFO,
			.str_data = true,
			.update = oplus_mms_gauge_get_r_info,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_VOL_FCL,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_vol_fcl,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SOC_CENTI,
			.update = oplus_mms_gauge_update_soc_centi,
		}
	}
};

static struct mms_item oplus_mms_sub_gauge_item[] = {
	{
		.desc = {
			.item_id = GAUGE_ITEM_SOC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_sub_gauge_update_soc,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_VOL_MAX,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_sub_gauge_update_vol_max,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_VOL_MIN,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_sub_gauge_update_vol_max,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_CURR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_sub_gauge_update_curr,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_TEMP,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_sub_gauge_update_temp,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_REG_INFO,
			.str_data = true,
			.update = oplus_mms_sub_gauge_get_reg_info,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_CALIB_TIME,
			.str_data = true,
			.update = oplus_mms_sub_gauge_get_calib_time,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SILI_IC_ALG_DSG_ENABLE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_sili_ic_alg_dsg_enable,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_CC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_sub_gauge_update_cc, /* todo not read when xxx_subscribe_main(sub)_gauge_topic  */
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SILI_IC_ALG_TERM_VOLT,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_sub_gauge_update_sili_ic_alg_term_volt,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_LIFETIME_STATUS,
			.str_data = true,
			.update = oplus_mms_sub_gauge_get_lifetime_status,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_SOH,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_soh, /* todo not read when xxx_subscribe_main(sub)_gauge_topic  */
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_FCC,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_fcc, /* todo not read when xxx_subscribe_main(sub)_gauge_topic  */
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_QMAX,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_qmax,
		}
	}, {
		.desc = {
			.item_id = GAUGE_ITEM_RATIO_VALUE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_mms_gauge_update_deep_ratio,
		}
	}
};

static const u32 oplus_mms_gauge_update_item[] = {
	GAUGE_ITEM_SOC,
	GAUGE_ITEM_VOL_MAX,
	GAUGE_ITEM_VOL_MIN,
	GAUGE_ITEM_GAUGE_VBAT,
	GAUGE_ITEM_CURR,
	GAUGE_ITEM_TEMP,
	GAUGE_ITEM_FCC,
	GAUGE_ITEM_CC,
	GAUGE_ITEM_SOH,
	GAUGE_ITEM_RM,
	GAUGE_ITEM_REAL_TEMP,
	GAUGE_ITEM_QMAX,
	GAUGE_ITEM_CAR_C,
};

static const u32 oplus_mms_main_sub_gauge_update_item[] = {
	GAUGE_ITEM_SOC,
	GAUGE_ITEM_VOL_MAX,
	GAUGE_ITEM_CURR,
	GAUGE_ITEM_TEMP,
};

static const struct oplus_mms_desc oplus_mms_gauge_desc = {
	.name = "gauge",
	.type = OPLUS_MMS_TYPE_GAUGE,
	.item_table = oplus_mms_gauge_item,
	.item_num = ARRAY_SIZE(oplus_mms_gauge_item),
	.update_items = oplus_mms_gauge_update_item,
	.update_items_num = ARRAY_SIZE(oplus_mms_gauge_update_item),
	.update_interval = 10000, /* ms */
	.update = oplus_mms_gauge_update,
	.set_update_mode = oplus_mms_gauge_read_mode_set,
};

#define CHARGE_UPDATE_INTERVAL		5000
static void oplus_mms_gauge_update_change_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip =
		container_of(work, struct oplus_mms_gauge, update_change_work);
	union mms_msg_data data = { 0 };
	int rc;

/* TODO: factory mode app adaptation
	if (chip->factory_test_mode)
		vote(chip->gauge_update_votable, FACTORY_TEST_VOTER, true, 500, false);
	else
		vote(chip->gauge_update_votable, FACTORY_TEST_VOTER, false, 0, false);
*/

	if (chip->wired_online || chip->wls_online) {
		chip->check_subboard_ntc_err = false;
		oplus_mms_gauge_push_subboard_temp_err(chip, false);
		vote(chip->gauge_update_votable, USER_VOTER, true, CHARGE_UPDATE_INTERVAL, false);

		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_AUTH, &data, false);
		if (!rc && !!data.intval == false) {
			chg_info("retry auth\n");
			oplus_mms_gauge_push_auth(chip);
		}

		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_HMAC, &data, false);
		if (!rc && !!data.intval == false) {
			chg_info("retry hmac\n");
			chip->hmac = oplus_mms_gauge_get_batt_hmac(chip);
			chip->sn_match = oplus_mms_gauge_get_sn_match(chip);
			chg_info("hmac=%d sn_match=%d\n", chip->hmac, chip->sn_match);
			if (!!data.intval != (chip->hmac && chip->sn_match))
				oplus_mms_gauge_push_hmac(chip);
		}
	} else {
		vote(chip->gauge_update_votable, USER_VOTER, false, 0, false);
	}
}

static void oplus_mms_gauge_comm_subs_callback(struct mms_subscribe *subs,
					       enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_gauge *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_FACTORY_TEST:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->factory_test_mode = data.intval;
			schedule_work(&chip->update_change_work);
			break;
		case COMM_ITEM_CHG_FULL:
			schedule_work(&chip->set_gauge_batt_full_work);
			break;
		case COMM_ITEM_UI_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->ui_soc = data.intval;
			break;
		case COMM_ITEM_SUPER_ENDURANCE_STATUS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->super_endurance_mode_status = !!data.intval;
			schedule_work(&chip->update_super_endurance_mode_status_work);
			break;
		case COMM_ITEM_SUPER_ENDURANCE_COUNT:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->super_endurance_mode_count = data.intval;
			break;
		case COMM_ITEM_BOOT_COMPLETED:
			oplus_mms_gauge_get_reserve_calib_info(chip);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_mms_gauge_subscribe_comm_topic(struct oplus_mms *topic,
						 void *prv_data)
{
	struct oplus_mms_gauge *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->comm_topic = topic;
	chip->comm_subs =
		oplus_mms_subscribe(chip->comm_topic, chip,
				    oplus_mms_gauge_comm_subs_callback,
				    "mms_gauge");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe common topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_FACTORY_TEST, &data,
				true);
	chip->factory_test_mode = data.intval;

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_BOOT_COMPLETED, &data,
				true);
	if (data.intval)
		oplus_mms_gauge_get_reserve_calib_info(chip);

	schedule_work(&chip->update_change_work);

	/* when bootup, check the btb state. */
	schedule_work(&chip->sub_btb_state_change_handler_work);
	schedule_delayed_work(&chip->gauge_cuv_state_work, msecs_to_jiffies(20000));
	schedule_delayed_work(&chip->gauge_update_three_level_term_volt_work, msecs_to_jiffies(22000));
}

static void oplus_mms_gauge_set_curve_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip =
		container_of(work, struct oplus_mms_gauge, gauge_set_curve_work);
	int type;
	unsigned int adapter_id;
	union mms_msg_data data = { 0 };
	int rc;
	int batt_temp_region;

	if (chip->wired_online) {
		type = oplus_wired_get_chg_type();
		if ((type != OPLUS_CHG_USB_TYPE_VOOC) &&
			(type != OPLUS_CHG_USB_TYPE_SVOOC)) {
			if (sid_to_adapter_chg_type(chip->vooc_sid) == CHARGER_TYPE_VOOC)
				type = OPLUS_CHG_USB_TYPE_VOOC;
			else if (sid_to_adapter_chg_type(chip->vooc_sid) == CHARGER_TYPE_SVOOC)
				type = OPLUS_CHG_USB_TYPE_SVOOC;
		}

		switch (type) {
		case OPLUS_CHG_USB_TYPE_UNKNOWN:
			type = CHARGER_SUBTYPE_DEFAULT;
			break;
		case OPLUS_CHG_USB_TYPE_QC2:
		case OPLUS_CHG_USB_TYPE_QC3:
			type = CHARGER_SUBTYPE_QC;
			break;
		case OPLUS_CHG_USB_TYPE_PD:
		case OPLUS_CHG_USB_TYPE_PD_DRP:
			type = CHARGER_SUBTYPE_PD;
			break;
		case OPLUS_CHG_USB_TYPE_PD_PPS:
			type = CHARGER_SUBTYPE_PPS;
			break;
		case OPLUS_CHG_USB_TYPE_VOOC:
			type = CHARGER_SUBTYPE_FASTCHG_VOOC;
			break;
		case OPLUS_CHG_USB_TYPE_SVOOC:
			type = CHARGER_SUBTYPE_FASTCHG_SVOOC;
			break;
		case OPLUS_CHG_USB_TYPE_UFCS:
			type = CHARGER_SUBTYPE_UFCS;
			break;
		default:
			type = CHARGER_SUBTYPE_DEFAULT;
			break;
		}

		adapter_id = sid_to_adapter_id(chip->vooc_sid);
	} else if (chip->wls_online) {
		rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_WLS_TYPE, &data, false);
		if (rc < 0) {
			type = OPLUS_CHG_WLS_UNKNOWN;
			chg_err("get wls type err\n");
		}
		adapter_id = 0;
		type = data.intval;

		oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_TEMP_REGION, &data, false);
		batt_temp_region = data.intval;
		if ((batt_temp_region <= TEMP_REGION_LITTLE_COLD || batt_temp_region >= TEMP_REGION_WARM) &&
		    (type == OPLUS_CHG_WLS_SVOOC || type == OPLUS_CHG_WLS_PD_65W)) {
			type = OPLUS_CHG_WLS_BPP;
		}

		chg_info("oplus_mms_gauge_set_curve_work, wls_online:%d, type:%d, batt_temp_region:%d\n",
			  chip->wls_online, type, batt_temp_region);

		switch (type) {
		case OPLUS_CHG_WLS_UNKNOWN:
			type = CHARGER_SUBTYPE_DEFAULT;
			break;
		case OPLUS_CHG_WLS_BPP:
		case OPLUS_CHG_WLS_EPP:
		case OPLUS_CHG_WLS_EPP_PLUS:
		case OPLUS_CHG_WLS_VOOC:
			type = CHARGER_SUBTYPE_DEFAULT;
			break;
		case OPLUS_CHG_WLS_SVOOC:
		case OPLUS_CHG_WLS_PD_65W:
			type = CHARGER_SUBTYPE_FASTCHG_VOOC;
			break;
		default:
			type = CHARGER_SUBTYPE_DEFAULT;
			break;
		}
	}
	(void)oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_BATTERY_CURVE,
				type, adapter_id, chip->pd_svooc);
}

static void oplus_mms_gauge_wired_subs_callback(struct mms_subscribe *subs,
						enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_gauge *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->wired_online = data.intval;
			schedule_work(&chip->update_change_work);
			oplus_gauge_deep_dischg_check(chip);
			break;
		case WIRED_ITEM_CHG_TYPE:
			if (chip->wired_online && is_voocphy_ic_available(chip))
				schedule_work(&chip->gauge_set_curve_work);
			break;
		case WIRED_TIME_ABNORMAL_ADAPTER:
			oplus_mms_get_item_data(chip->wired_topic,
						WIRED_TIME_ABNORMAL_ADAPTER,
						&data, false);
			chip->pd_svooc = data.intval;
			if (chip->wired_online && is_voocphy_ic_available(chip))
				schedule_work(&chip->gauge_set_curve_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

int oplus_gauge_get_sub_btb_curr_limit(struct oplus_mms *topic)
{
	struct oplus_mms_gauge *chip = NULL;
	if (NULL == topic) {
		chg_err("topic is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (NULL == chip) {
		chg_err("topic is NULL");
		return -EINVAL;
	}

	chg_debug("sub_btb_curr_limit = %d \n", chip->sub_btb_curr_limit);

	return chip->sub_btb_curr_limit;
}

static void oplus_mms_gauge_subscribe_wired_topic(struct oplus_mms *topic,
						  void *prv_data)
{
	struct oplus_mms_gauge *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_mms_gauge_wired_subs_callback,
				    "mms_gauge");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				true);
	chip->wired_online = !!data.intval;
	schedule_work(&chip->update_change_work);
	oplus_mms_gauge_deep_dischg_init(chip);
	if (chip->deep_spec.sili_err)
		oplus_mms_gauge_push_auth(chip);

	if (chip->wired_online && is_voocphy_ic_available(chip)) {
		oplus_mms_get_item_data(chip->wired_topic,
					WIRED_TIME_ABNORMAL_ADAPTER,
					&data, false);
		chip->pd_svooc = data.intval;
		schedule_work(&chip->gauge_set_curve_work);
	}
}

static int oplus_mms_gauge_push_hmac(struct oplus_mms_gauge *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  GAUGE_ITEM_HMAC);
	if (msg == NULL) {
		chg_err("alloc battery hmac msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish battery hmac msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

static int oplus_mms_gauge_push_auth(struct oplus_mms_gauge *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  GAUGE_ITEM_AUTH);
	if (msg == NULL) {
		chg_err("alloc battery auth msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->gauge_topic, msg);
	if (rc < 0) {
		chg_err("publish battery auth msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return rc;
}

#define ALLOW_DIFF_VALUE_MV	1000
#define BATT_VOL_ERR_COUNT_MAX	5
static int oplus_mms_gauge_check_batt_vol_diff(struct oplus_mms_gauge *chip)
{
	int vbat_max;
	int vbat_min;
	int batt_num = oplus_gauge_get_batt_num();
	struct votable *chg_disable_votable;
	union mms_msg_data data = { 0 };
	int rc;

	/* just support 2S battery */
	if (batt_num != 2)
		return 0;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data,
				     false);
	if (rc < 0) {
		chg_err("can't get vbat_max, rc=%d\n", rc);
		vbat_max = 0;
	} else {
		vbat_max = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data,
				     false);
	if (rc < 0) {
		chg_err("can't get vbat_min, rc=%d\n", rc);
		vbat_min = 0;
	} else {
		vbat_min = data.intval;
	}

	if (abs(vbat_max - vbat_min) > ALLOW_DIFF_VALUE_MV) {
		chip->check_batt_vol_count++;
		if (chip->check_batt_vol_count > BATT_VOL_ERR_COUNT_MAX) {
			chg_disable_votable = find_votable("CHG_DISABLE");
			if (chg_disable_votable)
				vote(chg_disable_votable, VOL_DIFF_VOTER,
				     true, 1, false);
			if (!chip->bat_volt_different)
				oplus_mms_gauge_push_auth(chip);
			chip->bat_volt_different = true;
			chip->check_batt_vol_count = 0;
			chg_info("BATTERY_SOFT_DIFF_VOLTAGE disable chg\n");
		}
	} else {
		if (chip->bat_volt_different) {
			chip->check_batt_vol_count++;
			if (chip->check_batt_vol_count >
			    BATT_VOL_ERR_COUNT_MAX) {
				if (chip->bat_volt_different) {
					chip->bat_volt_different = false;
					oplus_mms_gauge_push_auth(chip);
				}
				chip->check_batt_vol_count = 0;
				chg_disable_votable = find_votable("CHG_DISABLE");
				if (chg_disable_votable)
					vote(chg_disable_votable,
					     VOL_DIFF_VOTER, false, 0, false);
				chg_info(
					"Recovery BATTERY_SOFT_DIFF_VOLTAGE\n");
			}
		} else {
			chip->check_batt_vol_count = 0;
		}
	}

	return 0;
}

static void oplus_mms_gauge_gauge_update_work(struct work_struct *work)
{
	struct oplus_mms_gauge *chip =
		container_of(work, struct oplus_mms_gauge, gauge_update_work);

	oplus_mms_gauge_check_batt_vol_diff(chip);
	oplus_mms_gauge_update_sili_ic_alg_term_volt(chip, false);
}

static void oplus_mms_gauge_gauge_subs_callback(struct mms_subscribe *subs,
						enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_gauge *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_TIMER:
		schedule_work(&chip->gauge_update_work);
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case GAUGE_ITEM_SPARE_POWER_ENABLE:
			schedule_work(&chip->update_sili_spare_power_enable_work);
			break;
		case GAUGE_ITEM_SILI_IC_ALG_CFG:
			schedule_work(&chip->update_sili_ic_alg_cfg_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static int oplus_mms_gauge_subscribe_gauge_topic(struct oplus_mms_gauge *chip)
{
	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
				    oplus_mms_gauge_gauge_subs_callback,
				    "mms_gauge");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return PTR_ERR(chip->gauge_subs);
	}

	return 0;
}

static void oplus_mms_gauge_vooc_subs_callback(struct mms_subscribe *subs,
					       enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_gauge *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case VOOC_ITEM_SID:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_sid = (unsigned int)data.intval;
			if (chip->vooc_sid && chip->wired_online &&
			    is_voocphy_ic_available(chip))
				schedule_work(&chip->gauge_set_curve_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}
static void oplus_mms_gauge_subscribe_vooc_topic(struct oplus_mms *topic,
						 void *prv_data)
{
	struct oplus_mms_gauge *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->vooc_topic = topic;
	chip->vooc_subs =
		oplus_mms_subscribe(chip->vooc_topic, chip,
				    oplus_mms_gauge_vooc_subs_callback,
				    "mms_gauge");
	if (IS_ERR_OR_NULL(chip->vooc_subs)) {
		chg_err("subscribe vooc topic error, rc=%ld\n",
			PTR_ERR(chip->vooc_subs));
		return;
	}

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_SID, &data,
				true);
	chip->vooc_sid = (unsigned int)data.intval;
	if (chip->wired_online && is_voocphy_ic_available(chip))
		schedule_work(&chip->gauge_set_curve_work);
}

static void oplus_mms_gauge_parallel_subs_callback(struct mms_subscribe *subs,
						   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_gauge *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case SWITCH_ITEM_STATUS:
			oplus_mms_get_item_data(chip->parallel_topic, id, &data,
						false);
			if (data.intval == PARALLEL_BAT_BALANCE_ERROR_STATUS8) {
				chip->parallel_hamc = false;
				chg_err(" ERROR_STATUS8, hmac set false\n");
				oplus_mms_gauge_push_hmac(chip);
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

static void oplus_mms_gauge_subscribe_parallel_topic(struct oplus_mms *topic,
						  void *prv_data)
{
	struct oplus_mms_gauge *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->parallel_topic = topic;
	chip->parallel_subs =
		oplus_mms_subscribe(chip->parallel_topic, chip,
				    oplus_mms_gauge_parallel_subs_callback,
				    "mms_gauge");
	if (IS_ERR_OR_NULL(chip->parallel_subs)) {
		chg_err("subscribe switch topic error, rc=%ld\n",
			PTR_ERR(chip->parallel_subs));
		return;
	}

	oplus_mms_get_item_data(chip->parallel_topic, SWITCH_ITEM_STATUS, &data,
				 true);
	if (data.intval == PARALLEL_BAT_BALANCE_ERROR_STATUS8) {
		chip->parallel_hamc = false;
		chg_err(" ERROR_STATUS8, hmac set false\n");
		oplus_mms_gauge_push_hmac(chip);
	}
}

static void oplus_mms_gauge_batt_bal_subs_callback(struct mms_subscribe *subs,
						   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_gauge *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case BATT_BAL_ITEM_ABNORMAL_STATE:
			oplus_mms_get_item_data(chip->batt_bal_topic, id, &data, false);
			if (data.intval) {
				chg_err("batt bal abnormal state\n");
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

static void oplus_mms_gauge_subscribe_batt_bal_topic(
	struct oplus_mms *topic, void *prv_data)
{
	struct oplus_mms_gauge *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->batt_bal_topic = topic;
	chip->batt_bal_subs =
		oplus_mms_subscribe(chip->batt_bal_topic, chip,
				    oplus_mms_gauge_batt_bal_subs_callback,
				    "mms_gauge");
	if (IS_ERR_OR_NULL(chip->batt_bal_subs)) {
		chg_err("subscribe batt bal topic error, rc=%ld\n",
			PTR_ERR(chip->batt_bal_subs));
		return;
	}

	oplus_mms_get_item_data(chip->batt_bal_topic,
		BATT_BAL_ITEM_ABNORMAL_STATE, &data, true);
	chg_info("abnormal state=%d\n", data.intval);
	if (data.intval) {
		chg_err("batt bal abnormal state\n");
	}
}

static void oplus_mms_gauge_wls_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_gauge *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WLS_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wls_topic, id, &data, false);
			chip->wls_online = !!data.intval;
			schedule_work(&chip->update_change_work);
			oplus_gauge_deep_dischg_check(chip);
			break;
		case WLS_ITEM_WLS_TYPE:
			if (chip->wls_online && is_voocphy_ic_available(chip))
				schedule_work(&chip->gauge_set_curve_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_mms_gauge_subscribe_wls_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_mms_gauge *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wls_topic = topic;
	chip->wls_subs = oplus_mms_subscribe(chip->wls_topic, chip, oplus_mms_gauge_wls_subs_callback, "mms_gauge");
	if (IS_ERR_OR_NULL(chip->wls_subs)) {
		chg_err("subscribe wls topic error, rc=%ld\n", PTR_ERR(chip->wls_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_PRESENT, &data, true);
	chip->wls_online = !!data.intval;
	if (chip->wls_online)
		schedule_work(&chip->update_change_work);
}

static void oplus_mms_gauge_cpa_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms_gauge *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case CPA_ITEM_POWER:
			chg_info("cpa set power\n");
			schedule_work(&chip->gauge_set_curve_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_mms_gauge_subscribe_cpa_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_mms_gauge *chip = prv_data;

	chip->cpa_topic = topic;
	chip->cpa_subs = oplus_mms_subscribe(chip->cpa_topic, chip, oplus_mms_gauge_cpa_subs_callback, "mms_gauge");
	if (IS_ERR_OR_NULL(chip->cpa_subs))
		chg_err("subscribe cpa topic error, rc=%ld\n", PTR_ERR(chip->cpa_subs));
}

#define GAUGE_NAME_LENGTH 10
static int oplus_mms_gauge_topic_init(struct oplus_mms_gauge *chip)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;
	int i;
	struct oplus_mms_desc *mms_desc;
	char *name;
	struct mms_item *item;

	mms_cfg.drv_data = chip;
	mms_cfg.of_node = chip->dev->of_node;

	if (of_property_read_bool(mms_cfg.of_node,
				  "oplus,topic-update-interval")) {
		rc = of_property_read_u32(mms_cfg.of_node,
					  "oplus,topic-update-interval",
					  &mms_cfg.update_interval);
		if (rc < 0) {
			chg_err("can't read oplus,topic-update-interval, rc=%d\n",
				rc);
			mms_cfg.update_interval = 0;
		}
	}

	chip->gauge_topic = devm_oplus_mms_register(chip->dev, &oplus_mms_gauge_desc, &mms_cfg);
	if (IS_ERR(chip->gauge_topic)) {
		chg_err("Couldn't register gauge topic\n");
		rc = PTR_ERR(chip->gauge_topic);
		return rc;
	}
	vote(chip->gauge_update_votable, DEF_VOTER, true, oplus_mms_gauge_desc.update_interval, false);
	for (i = 0; i < chip->child_num; i++) {
		chip->gauge_reg_info[i] = devm_kzalloc(chip->dev,
					sizeof(unsigned char) * GAUGE_REG_INFO_SIZE, GFP_KERNEL);
		chip->gauge_r_info[i] = devm_kzalloc(chip->dev,
					sizeof(unsigned char) * GAUGE_REG_INFO_SIZE, GFP_KERNEL);
	}
	if (is_support_parallel(chip)) {
		mms_cfg.update_interval = 0;
		mms_desc = devm_kzalloc(chip->dev, sizeof(struct oplus_mms_desc) * chip->child_num,
					GFP_KERNEL);
		for (i = 0; i < chip->child_num; i++) {
			name = devm_kzalloc(chip->dev, sizeof(char) * GAUGE_NAME_LENGTH,
					GFP_KERNEL);
			if (i != 0) {
				item = devm_kzalloc(chip->dev,
					sizeof(struct mms_item) * ARRAY_SIZE(oplus_mms_sub_gauge_item), GFP_KERNEL);
				memcpy(item, oplus_mms_sub_gauge_item, sizeof(oplus_mms_sub_gauge_item));
			}
			snprintf(name, GAUGE_NAME_LENGTH, "gauge:%d", i);
			mms_desc[i].name = name;
			mms_desc[i].type = OPLUS_MMS_TYPE_GAUGE;
			if (i == 0)
				mms_desc[i].item_table = oplus_mms_sub_gauge_item;
			else
				mms_desc[i].item_table = item;
			mms_desc[i].item_num = ARRAY_SIZE(oplus_mms_sub_gauge_item);
			mms_desc[i].update_items = oplus_mms_main_sub_gauge_update_item;
			mms_desc[i].update_items_num = ARRAY_SIZE(oplus_mms_main_sub_gauge_update_item);
			mms_desc[i].update_interval = 0;
			mms_desc[i].update = oplus_mms_sub_gauge_update;
			mms_desc[i].set_update_mode = oplus_mms_gauge_read_mode_set;
			if (strcmp("gauge:0", mms_desc[i].name) == 0) {
				chip->gauge_topic_parallel[chip->main_gauge] =
					devm_oplus_mms_register(chip->dev, &mms_desc[i], &mms_cfg);
				if (IS_ERR(chip->gauge_topic_parallel[chip->main_gauge]))
					chg_err(" error to register main_gauge topic\n");
				else
					chg_info(" register main_gauge topic success %s\n", mms_desc[i].name);
			} else {
				chip->gauge_topic_parallel[__ffs(chip->sub_gauge)] =
					devm_oplus_mms_register(chip->dev, &mms_desc[i], &mms_cfg);
				if (IS_ERR(chip->gauge_topic_parallel[__ffs(chip->sub_gauge)]))
					chg_err(" error to register sub_gauge topic\n");
				else
					chg_info(" register sub_gauge topic success %s\n", mms_desc[i].name);
			}
		}
	}

	oplus_mms_gauge_subscribe_gauge_topic(chip);
	oplus_mms_gauge_push_hmac(chip);
	oplus_mms_wait_topic("common", oplus_mms_gauge_subscribe_comm_topic, chip);

	oplus_mms_gauge_sili_init(chip);
	oplus_mms_gauge_sili_ic_alg_cfg_init(chip);

	oplus_mms_wait_topic("wired", oplus_mms_gauge_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("vooc", oplus_mms_gauge_subscribe_vooc_topic, chip);
	oplus_mms_wait_topic("parallel", oplus_mms_gauge_subscribe_parallel_topic, chip);
	oplus_mms_wait_topic("wireless", oplus_mms_gauge_subscribe_wls_topic, chip);
	oplus_mms_wait_topic("batt_bal", oplus_mms_gauge_subscribe_batt_bal_topic, chip);
	oplus_mms_wait_topic("cpa", oplus_mms_gauge_subscribe_cpa_topic, chip);

	if (chip->deep_spec.limit_curr_curves.nums <= 0) {
		rc = oplus_mms_set_item_disable(chip->gauge_topic, GAUGE_ITEM_RATIO_LIMIT_CURR);
		if (rc < 0)
			chg_err("disabe GAUGE_ITEM_RATIO_LIMIT_CURR fail rc=%d\n", rc);
	}
	return 0;
}

static int oplus_gauge_update_vote_callback(struct votable *votable, void *data,
					    int time_ms, const char *client,
					    bool step)
{
	struct oplus_mms_gauge *chip = data;
	int rc;

	if (time_ms < 0) {
		chg_err("time_ms=%d, restore default publish interval\n", time_ms);
		oplus_mms_restore_publish(chip->gauge_topic);
		return 0;
	}

	rc = oplus_mms_set_publish_interval(chip->gauge_topic, time_ms);
	if (rc < 0)
		chg_err("can't set gauge publish interval to %d\n", time_ms);
	else
		chg_err("set gauge publish interval to %d\n", time_ms);

	return rc;
}

static void oplus_mms_gauge_get_reserve_calib_info(
			struct oplus_mms_gauge *chip)
{
	static bool update = false;

	if (!update) {
		update = true;
		schedule_delayed_work(&chip->get_reserve_calib_info_work, msecs_to_jiffies(2000));
	}
}

static void oplus_mms_gauge_set_reserve_calib_info_work(struct work_struct *work)
{
	int rc;
	struct oplus_mms_gauge *chip = container_of(work, struct oplus_mms_gauge,
		set_reserve_calib_info_work);

	rc = oplus_chg_set_mutual_cmd(CMD_GAUGE_CALIB_UPDATE,
		sizeof(chip->calib_info_load), &(chip->calib_info_load));
	if (rc == CMD_ACK_OK)
		chg_info("success\n");
	else
		chg_info("fail\n");
}

static void oplus_mms_gauge_get_reserve_calib_info_work(
				struct work_struct *work)
{
	int i;
	int mutual_rc;
	int func_rc;
	struct oplus_chg_ic_dev *ic;
	struct gauge_calib_info *calib_info;
	static int try_count = GAUGE_CALIB_OBTAIN_COUNTS;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip = container_of(dwork, struct oplus_mms_gauge, get_reserve_calib_info_work);

	if (chip->child_num > GAUGE_IC_NUM_MAX)
		return;

	mutual_rc = oplus_chg_set_mutual_cmd(CMD_GAUGE_CALIB_OBTAIN, 0, NULL);
	if (mutual_rc != CMD_ACK_OK && try_count--) {
		schedule_delayed_work(&chip->get_reserve_calib_info_work, msecs_to_jiffies(2000));
		return;
	}

	for (i = 0; i < chip->child_num; i++) {
		calib_info = &chip->calib_info_load.calib_info[i];
		ic = chip->child_list[i].ic_dev;
		func_rc = oplus_chg_ic_func(ic, OPLUS_IC_FUNC_GAUGE_SET_CALIB_TIME,
			calib_info->dod_time, calib_info->qmax_time,
			calib_info->calib_args, sizeof(calib_info->calib_args));
		if (func_rc == 0 && mutual_rc == CMD_ACK_OK)
			chip->calib_info_init[i] = true;
	}
}

static void oplus_gauge_update_three_level_term_volt_work(struct work_struct *work)
{
	int func_rc = -ENOTSUPP;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_mms_gauge *chip = container_of(dwork, struct oplus_mms_gauge,
			gauge_update_three_level_term_volt_work);
	unsigned char data[32] = {0};
	struct gauge_three_level_term_volt_cfg *volt_cfg = NULL;
	int current_volt = INVALID_MIN_VOLTAGE;
	int reg_term_volt = 0;

	func_rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_THREE_LEVEL_TERM_VOLT,
			data, OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN);
	if (func_rc != 0)
		return;

	chg_info("term_vol:[%x,%x,%x,%x,%x,%x],holdtime[%x,%x,%x],time_to_drop[%x,%x,%x]," \
		 "recover_voltage[%x,%x,%x,%x],recover holdtime[%x,%x]\n",
		 data[0], data[1], data[2], data[3], data[4], data[5],
		 data[6], data[7], data[8],
		 data[9], data[10], data[11],
		 data[12], data[13], data[14], data[15],
		 data[16], data[17]);

	/* update the param based on the DTS config.*/
	volt_cfg = &chip->three_level_term_volt_cfg;

	/*
	 * The 18 bytes three leve term volt param:
	 * byte[0]: the low 8 bit of Term Voltage
	 * byte[1]: the high 8 bit of Term voltage
	 * byte[2]: the low 8 bit of Term Voltage_2
	 * byte[3]: the high 8 bit of Term voltage_2
	 * Byte[4]: the low 8 bit of Term Voltage_3
	 * byte[5]: the high 8 bit of Term voltage_3
	 * byte[6]: the hold time of Term Voltage
	 * byte[7]: the hold time of Term voltage_2
	 * byte[8]: the hold time of Term Voltage_3
	 * byte[9]: the time_to_drop_per1%
	 * byte[10]: the time_to_drop_per1%_2
	 * byte[11]: the time_to_drop_per1%_3
	 * byte[12]: the recover low 8 bit of Term Voltage
	 * byte[13]: the recover high 8 bit of Term voltage
	 * byte[14]: the recover low 8 bit of Term Voltage_2
	 * byte[15]: the recover high 8 bit of Term voltage_2
	 * byte[16]: the recover hold time of Term Voltage
	 * byte[17]: the recover hold time of Term voltage_2
	 */
	func_rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT, &current_volt);
	if (func_rc < 0)
		chg_err("get batt deep term volt error, rc=%d\n", func_rc);

	reg_term_volt = (data[1] << 8) + data[0];
	chg_info("get term_volt = [%d, %d, %d], reg_term_volt[%d]\n", current_volt,
		 chip->deep_spec.config.term_voltage,
		 get_effective_result(chip->gauge_term_voltage_votable),
		 reg_term_volt);
	if ((volt_cfg->term_volt > current_volt) &&
	    (reg_term_volt != volt_cfg->term_volt)) {
		chg_info("set term_volt = %d\n", volt_cfg->term_volt);
		vote(chip->gauge_term_voltage_votable, READY_VOTER, true, volt_cfg->term_volt, false);
		return;
	}

	data[6] = (volt_cfg->hold_time > 0) ? volt_cfg->hold_time : 0;
	data[7] = (volt_cfg->hold_time_2 > 0) ? volt_cfg->hold_time_2 : 0;
	data[8] = (volt_cfg->hold_time_3 > 0) ? volt_cfg->hold_time_3 : 0;
	data[9] = (volt_cfg->time_to_drop_per1 > 0) ? volt_cfg->time_to_drop_per1 : 0;
	data[10] = (volt_cfg->time_to_drop_per1_2 > 0) ? volt_cfg->time_to_drop_per1_2 : 0;
	data[11] = (volt_cfg->time_to_drop_per1_3 > 0) ? volt_cfg->time_to_drop_per1_3 : 0;

	if (volt_cfg->recover_term_volt) {
		data[12] = volt_cfg->recover_term_volt & 0xff;
		data[13] = ((volt_cfg->recover_term_volt & 0xff00) >> 8);
	}
	if (volt_cfg->recover_term_volt_2) {
		data[14] = volt_cfg->recover_term_volt_2 & 0xff;
		data[15] = ((volt_cfg->recover_term_volt_2 & 0xff00) >> 8);
	}
	data[16] = (volt_cfg->recover_hold_time_of_term_voltage > 0) ?
			volt_cfg->recover_hold_time_of_term_voltage : 0;
	data[17] = (volt_cfg->recover_hold_time_of_term_voltage_2 > 0) ?
			volt_cfg->recover_hold_time_of_term_voltage_2 : 0;

	chg_info("update term_volt[%x,%x,%x,%x,%x,%x], holdtime[%x,%x,%x],time_to_drop[%x,%x,%x]," \
		 "recover_voltage[%x,%x,%x,%x],recover holdtime[%x,%x]\n",
		 data[0], data[1], data[2], data[3], data[4], data[5],
		 data[6], data[7], data[8],
		 data[9], data[10], data[11],
		 data[12], data[13], data[14], data[15], data[16], data[17]);

	if (chip->three_level_term_volt_cfg.term_volt) {
		func_rc = oplus_chg_ic_func(chip->gauge_ic, OPLUS_IC_FUNC_GAUGE_SET_THREE_LEVEL_TERM_VOLT,
					data, OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN);
		chg_info(" func_rc = %d \n", func_rc);
	}
}

static int oplus_mms_gauge_calib_obtain_mutual_notifier_call(
		struct notifier_block *nb, unsigned long param, void *v)
{
	int i;
	struct oplus_mms_gauge *chip;
	struct gauge_calib_info_load *calib_info_load;
	struct oplus_chg_mutual_notifier *notifier;

	notifier = container_of(nb, struct oplus_chg_mutual_notifier, nb);
	chip = container_of(notifier, struct oplus_mms_gauge, calib_obtain_mutual);

	if (mutual_info_to_cmd(param) != CMD_GAUGE_CALIB_OBTAIN) {
		chg_err("cmd is not matching, should return\n");
		return NOTIFY_OK;
	}

	if (mutual_info_to_data_size(param) != sizeof(struct gauge_calib_info_load)) {
		chg_err("data_len is not ok, datas is invalid\n");
		return NOTIFY_DONE;
	}

	calib_info_load = (struct gauge_calib_info_load *)v;
	if (calib_info_load)
		memcpy(&chip->calib_info_load, calib_info_load, sizeof(struct gauge_calib_info_load));

	for (i = 0; i < GAUGE_IC_NUM_MAX; i++)
		chg_info("index:%d, tag:%s, dod_time:%d, qmax_time:%d, calib_args:[%*ph]", i,
			chip->calib_info_load.tag_info, chip->calib_info_load.calib_info[i].dod_time,
			chip->calib_info_load.calib_info[i].qmax_time, GAUGE_CALIB_ARGS_LEN,
			chip->calib_info_load.calib_info[i].calib_args);

	return NOTIFY_OK;
}

static int oplus_mms_gauge_calib_update_mutual_notifier_call(
		struct notifier_block *nb, unsigned long param, void *v)
{
	struct oplus_mms_gauge *chip;
	struct oplus_chg_mutual_notifier *notifier;

	notifier = container_of(nb, struct oplus_chg_mutual_notifier, nb);
	chip = container_of(notifier, struct oplus_mms_gauge, calib_update_mutual);

	if (mutual_info_to_cmd(param) != CMD_GAUGE_CALIB_UPDATE) {
		chg_err("cmd is not matching, should return\n");
		return NOTIFY_OK;
	}
	chg_info("success\n");

	return NOTIFY_OK;
}

static int oplus_mms_gauge_calib_obtain_mutual_notify_reg(struct oplus_mms_gauge *chip)
{
	int rc = 0;

	chip->calib_obtain_mutual.name = "gauge_calib_obtain";
	chip->calib_obtain_mutual.cmd = CMD_GAUGE_CALIB_OBTAIN;
	chip->calib_obtain_mutual.nb.notifier_call = oplus_mms_gauge_calib_obtain_mutual_notifier_call;
	rc = oplus_chg_reg_mutual_notifier(&chip->calib_obtain_mutual);
	if (rc < 0) {
		chg_err("register gauge calib obtain mutual event notifier error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int oplus_mms_gauge_calib_update_mutual_notify_reg(struct oplus_mms_gauge *chip)
{
	int rc = 0;

	chip->calib_update_mutual.name = "gauge_calib_update";
	chip->calib_update_mutual.cmd = CMD_GAUGE_CALIB_UPDATE;
	chip->calib_update_mutual.nb.notifier_call = oplus_mms_gauge_calib_update_mutual_notifier_call;
	rc = oplus_chg_reg_mutual_notifier(&chip->calib_update_mutual);
	if (rc < 0) {
		chg_err("register gauge calib update mutual event notifier error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int oplus_mms_gauge_probe(struct platform_device *pdev)
{
	struct oplus_mms_gauge *chip;
	int rc;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_mms_gauge),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	of_platform_populate(chip->dev->of_node, NULL, NULL, chip->dev);

	mutex_init(&chip->deep_spec.lock);

	chip->gauge_update_votable =
		create_votable("GAUGE_UPDATE", VOTE_MIN,
			       oplus_gauge_update_vote_callback, chip);
	if (IS_ERR(chip->gauge_update_votable)) {
		rc = PTR_ERR(chip->gauge_update_votable);
		chip->gauge_update_votable = NULL;
		goto create_vote_err;
	}

	chip->gauge_shutdown_voltage_votable =
		create_votable("GAUGE_SHUTDOWN_VOLTAGE", VOTE_MAX, oplus_gauge_shutdown_voltage_vote_callback, chip);
	if (IS_ERR(chip->gauge_shutdown_voltage_votable)) {
		rc = PTR_ERR(chip->gauge_shutdown_voltage_votable);
		chip->gauge_shutdown_voltage_votable = NULL;
		goto creat_gauge_shutdown_voltage_votable_err;
	}

	chip->gauge_term_voltage_votable =
		create_votable("GAUGE_TERM_VOLTAGE", VOTE_MAX, oplus_gauge_term_voltage_vote_callback, chip);
	if (IS_ERR(chip->gauge_term_voltage_votable)) {
		rc = PTR_ERR(chip->gauge_term_voltage_votable);
		chip->gauge_term_voltage_votable = NULL;
		goto creat_gauge_term_voltage_votable_err;
	}
	chip->target_term_voltage_votable =
		create_votable("TARGET_TERM_VOLTAGE", VOTE_MAX, oplus_target_term_voltage_vote_callback, chip);
	if (IS_ERR(chip->target_term_voltage_votable)) {
		rc = PTR_ERR(chip->target_term_voltage_votable);
		chip->target_term_voltage_votable = NULL;
		goto creat_target_term_voltage_votable_err;
	}
	chip->target_shutdown_voltage_votable =
		create_votable("TARGET_SHUTDOWN_VOLTAGE", VOTE_MAX, oplus_target_shutdown_voltage_vote_callback, chip);
	if (IS_ERR(chip->target_shutdown_voltage_votable)) {
		rc = PTR_ERR(chip->target_shutdown_voltage_votable);
		chip->target_shutdown_voltage_votable = NULL;
		goto creat_target_shutdown_voltage_votable_err;
	}

	rc = oplus_mms_gauge_calib_obtain_mutual_notify_reg(chip);
	if (rc < 0)
		goto calib_obtain_mutual_nb_reg_err;

	rc = oplus_mms_gauge_calib_update_mutual_notify_reg(chip);
	if (rc < 0)
		goto calib_update_mutual_nb_reg_err;

	INIT_DELAYED_WORK(&chip->hal_gauge_init_work, oplus_mms_gauge_init_work);
	INIT_DELAYED_WORK(&chip->get_reserve_calib_info_work, oplus_mms_gauge_get_reserve_calib_info_work);
	INIT_WORK(&chip->set_reserve_calib_info_work, oplus_mms_gauge_set_reserve_calib_info_work);
	INIT_WORK(&chip->err_handler_work, oplus_mms_gauge_err_handler_work);
	INIT_WORK(&chip->ls_err_handler_work, oplus_mms_level_shift_err_handler_work);
	INIT_WORK(&chip->ls_fpga_rst_handler_work, oplus_mms_level_shift_fpga_rst_handler_work);
	INIT_WORK(&chip->online_handler_work, oplus_mms_gauge_online_handler_work);
	INIT_WORK(&chip->offline_handler_work, oplus_mms_gauge_offline_handler_work);
	INIT_WORK(&chip->resume_handler_work, oplus_mms_gauge_resume_handler_work);
	INIT_WORK(&chip->update_change_work, oplus_mms_gauge_update_change_work);
	INIT_WORK(&chip->gauge_update_work, oplus_mms_gauge_gauge_update_work);
	INIT_WORK(&chip->gauge_set_curve_work, oplus_mms_gauge_set_curve_work);
	INIT_WORK(&chip->set_gauge_batt_full_work, oplus_mms_gauge_set_batt_full_work);
	INIT_WORK(&chip->update_super_endurance_mode_status_work,
		  oplus_mms_gauge_update_super_endurance_mode_status_work);
	INIT_WORK(&chip->sub_btb_state_change_handler_work,
		  oplus_mms_gauge_sub_btb_state_change_handler_work);
	INIT_WORK(&chip->hmac_update_handler_work,
		  oplus_mms_gauge_hmac_update_handler_work);

	INIT_DELAYED_WORK(&chip->subboard_ntc_err_work, oplus_mms_subboard_ntc_err_work);
	INIT_DELAYED_WORK(&chip->deep_dischg_work, oplus_gauge_deep_dischg_work);
	INIT_DELAYED_WORK(&chip->sub_deep_dischg_work, oplus_gauge_sub_deep_dischg_work);
	INIT_DELAYED_WORK(&chip->deep_id_work, oplus_gauge_deep_id_work);
	INIT_DELAYED_WORK(&chip->deep_track_work, oplus_gauge_deep_track_work);
	INIT_DELAYED_WORK(&chip->sub_deep_track_work, oplus_gauge_sub_deep_track_work);
	INIT_DELAYED_WORK(&chip->deep_ratio_work, oplus_gauge_deep_ratio_work);
	INIT_WORK(&chip->update_sili_ic_alg_cfg_work,
		  oplus_mms_gauge_update_sili_ic_alg_cfg_work);
	INIT_WORK(&chip->update_sili_spare_power_enable_work,
		  oplus_mms_gauge_update_sili_spare_power_enable_work);
	INIT_DELAYED_WORK(&chip->sili_spare_power_effect_check_work,
		  oplus_mms_gauge_sili_spare_power_effect_check_work);
	INIT_DELAYED_WORK(&chip->sili_term_volt_effect_check_work,
		  oplus_mms_gauge_sili_term_volt_effect_check_work);
	INIT_DELAYED_WORK(&chip->deep_temp_work, oplus_gauge_deep_temp_work);
	INIT_DELAYED_WORK(&chip->gauge_cuv_state_work,
		  oplus_gauge_cuv_state_work);
	INIT_DELAYED_WORK(&chip->gauge_update_three_level_term_volt_work,
		oplus_gauge_update_three_level_term_volt_work);
	INIT_DELAYED_WORK(&chip->gauge_nvram_stress_test_work,
		oplus_gauge_nvram_stress_test_work);
	INIT_DELAYED_WORK(&chip->gauge_stress_read_test_work,
		oplus_gauge_read_stress_test_work);
	INIT_DELAYED_WORK(&chip->gauge_term_volt_stress_test_work,
		oplus_gauge_term_volt_stress_test_work);

	chip->sec_chip = oplus_sec_init();
	if (chip->sec_chip == NULL)
		chg_err("oplus_sec init error");

	schedule_delayed_work(&chip->hal_gauge_init_work, 0);

	chg_info("probe success\n");
	return 0;

calib_update_mutual_nb_reg_err:
	oplus_chg_unreg_mutual_notifier(&chip->calib_obtain_mutual);
calib_obtain_mutual_nb_reg_err:
	destroy_votable(chip->target_shutdown_voltage_votable);
creat_target_shutdown_voltage_votable_err:
	destroy_votable(chip->target_term_voltage_votable);
creat_target_term_voltage_votable_err:
	destroy_votable(chip->gauge_term_voltage_votable);
creat_gauge_term_voltage_votable_err:
	destroy_votable(chip->gauge_shutdown_voltage_votable);
creat_gauge_shutdown_voltage_votable_err:
	destroy_votable(chip->gauge_update_votable);
create_vote_err:
	platform_set_drvdata(pdev, NULL);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_mms_gauge_remove(struct platform_device *pdev)
#else
static int oplus_mms_gauge_remove(struct platform_device *pdev)
#endif
{
	struct oplus_mms_gauge *chip = platform_get_drvdata(pdev);
	int i;

	if (!IS_ERR_OR_NULL(chip->comm_subs))
		oplus_mms_unsubscribe(chip->comm_subs);
	if (!IS_ERR_OR_NULL(chip->wired_subs))
		oplus_mms_unsubscribe(chip->wired_subs);
	if (!IS_ERR_OR_NULL(chip->gauge_subs))
		oplus_mms_unsubscribe(chip->gauge_subs);
	if (!IS_ERR_OR_NULL(chip->cpa_subs))
		oplus_mms_unsubscribe(chip->cpa_subs);
	for (i = 0; i < chip->ddrc_num; i++) {
		if (chip->ddrc_strategy[i] != NULL)
			oplus_chg_strategy_release(chip->ddrc_strategy[i]);
	}

	oplus_sec_release(chip->sec_chip);

	oplus_chg_unreg_mutual_notifier(&chip->calib_update_mutual);
	oplus_chg_unreg_mutual_notifier(&chip->calib_obtain_mutual);

	destroy_votable(chip->gauge_term_voltage_votable);
	destroy_votable(chip->gauge_shutdown_voltage_votable);
	destroy_votable(chip->gauge_update_votable);
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_mms_gauge_match[] = {
	{ .compatible = "oplus,mms_gauge" },
	{},
};

static struct platform_driver oplus_mms_gauge_driver = {
	.driver		= {
		.name = "oplus-mms_gauge",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_mms_gauge_match),
	},
	.probe		= oplus_mms_gauge_probe,
	.remove		= oplus_mms_gauge_remove,
};

static __init int oplus_mms_gauge_init(void)
{
	return platform_driver_register(&oplus_mms_gauge_driver);
}

static __exit void oplus_mms_gauge_exit(void)
{
	platform_driver_unregister(&oplus_mms_gauge_driver);
}

oplus_chg_module_register(oplus_mms_gauge);
