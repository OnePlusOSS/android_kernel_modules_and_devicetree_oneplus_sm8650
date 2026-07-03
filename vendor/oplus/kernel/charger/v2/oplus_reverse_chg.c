/***********************************************************
** Copyright (C), 2025-2025 Oplus. All rights reserved.
** File: oplus_reverse_chg.c
** Description: cpreverse ic
** Date: 2025-11-14
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#define pr_fmt(fmt) "[REVERSE]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/gfp.h>
#include <linux/kthread.h>
#include <linux/sched/clock.h>
#include <linux/proc_fs.h>

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_ic.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_cpa.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_pps.h>
#include <oplus_reverse_chg.h>

#define DEC_REVERSE_UISOC_LINIT_COUNT	4
#define DEC_REVERSE_TEMP_LINIT_COUNT	4
#define DEC_REVERSE_VBUS_LINIT_COUNT	5
#define DEC_REVERSE_VBAT_LINIT_COUNT	5
#define DEC_REVERSE_PDO_LINIT_COUNT	6
#define DEC_LAST_LINIT_COUNT		2
#define MAX_NORMAL_REVERSE_COUNT	5
#define MAX_HARDRESET_LIMIT_COUNT	6
#define HIGH_VBUS_START			6000

struct reverse_limit_data {
	int reverse_uisoc_limit[DEC_REVERSE_UISOC_LINIT_COUNT];
	int reverse_temp_limit[DEC_REVERSE_TEMP_LINIT_COUNT];
	int reverse_vbus_limit[DEC_REVERSE_VBUS_LINIT_COUNT];
	int reverse_vbat_limit[DEC_REVERSE_VBAT_LINIT_COUNT];
	int last_uisoc_limit[DEC_LAST_LINIT_COUNT];
	int last_current[DEC_LAST_LINIT_COUNT];
	int reverse_led_on_limit[DEC_LAST_LINIT_COUNT];
	int reverse_authenticate_limit[DEC_LAST_LINIT_COUNT];
	int reverse_over_ibus_limit[DEC_LAST_LINIT_COUNT];
	int normal_reverse_current[MAX_NORMAL_REVERSE_COUNT];
	int normal_reverse_hw_ocp[MAX_NORMAL_REVERSE_COUNT];
};

struct oplus_reverse_pdo_limit_t {
	int reverse_pdo_voltage;
	int reverse_pdo_current;
	u32 reverse_pdo;
};

struct oplus_reverse_temp_pdo_limit {
	struct oplus_reverse_pdo_limit_t comm_pdo_limit;
	struct oplus_reverse_pdo_limit_t oplus_pdo_limit;
};

struct oplus_reverse_uisoc_pdo_limit {
	struct oplus_reverse_temp_pdo_limit reverse_temp_pdo_limit[DEC_REVERSE_TEMP_LINIT_COUNT + 1];
};

struct oplus_chg_reverse {
	struct device *dev;
	struct proc_dir_entry *reverse_entry;
	struct oplus_mms *reverse_topic;
	struct oplus_mms *comm_topic;
	struct oplus_mms *wired_topic;
	struct oplus_mms *cpa_topic;
	struct mms_subscribe *comm_subs;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *reverse_subs;
	struct mms_subscribe *gauge_subs;
	struct oplus_chg_ic_dev *cp_ic;
	struct oplus_chg_ic_dev *buck_ic;
	struct oplus_chg_ic_dev *reverse_ic;
	int shell_temp;

	struct oplus_chg_strategy *temperature_strategy;
	struct delayed_work reverse_monitor_work;
	struct delayed_work usbtemp_high_work;
	struct delayed_work reverse_pdo_update_work;
	struct delayed_work reverse_chg_init_work;
	struct delayed_work reverse_chg_type_work;
	struct delayed_work reverse_error_flag_work;
	struct delayed_work normal_reverse_set_hw_ocp_work;
	struct delayed_work reverse_vbus_retention_check_work;
	struct delayed_work reverse_chg_test_mode_work;
	struct delayed_work reverse_clear_keep_status_work;
	struct delayed_work reverse_hard_reset_work;
	struct work_struct reverse_online_work;
	struct work_struct reverse_vbus_check_work;

	struct mutex set_pdo_lock;
	bool reverse_chg_switch;
	bool reverse_chg_testing;
	int reverse_chg_test_mode;
	int test_high_pwr_vbus_mv;
	int test_result;
	bool wired_online;
	int vbus_mv;
	int vbus_level;
	int vbus_limit_count;
	int vbat_level;
	int vbat_limit_count;
	int hardreset_limit_count;
	int hw_detect;
	int reverse_status;
	int reverse_enable;
	int high_reverse_enable;
	int high_reverse_count;
	int enable_err_count;
	pd_msg_data pdo[PPS_PDO_MAX];

	int reverse_temp_cur_range;
	int reverse_uisoc_cur_range;
	bool high_reverse_charging;
	int reverse_max_curr;
	bool start_check;
	int ibus_ma;
	int reverse_strategy_change_count;
	int reverse_ibus_lower_oplus;
	struct oplus_chg_strategy *strategy;
	struct votable *reverse_pdo_votable;
	struct votable *cap_pdo0_votable;
	struct votable *high_reverse_disable_votable;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *sub_gauge_topic;
	int ibus_over;
	int ibus_lower;
	bool ibus_ov;
	int thired_oplus_svid_limit;
	bool use_high_reverse_awake;
	bool kthread_reverse_enable;
	bool reverse_vote;
	bool use_cp_reverse;
	bool authenticate;
	bool hmac;
	bool sink_svid_support;
	bool is_support_high_reverse;
	int ui_soc;
	int led_on;
	int vbat_min_mv;
	int vbat_mv;
	int ibat_ma;
	int batt_temp;
	int batt_soc;
	int batt_realy_temp;
	int hard_reset_count;
	int drop_down_count;
	int reset_watchdog_time;
	int pre_pdo_voltage;
	int pre_pdo_current;
	int high_reverse_err_flag;
	int normal_reverse_count;
	int req_volt;
	int req_current;
	int reverse_chg_type;
	int oplus_svid;
	u32 target_pdo;
	u32 target_cap_pdo0;
	u32 pre_target_cap_pdo0;
	u32 led_on_pdo_limit;
	u32 authenticate_pdo_limit;
	u32 over_ibus_pdo_limit;
	u32 reverse_chg_svid;
	u32 last_high_reverse_pdo;
	int reverse_vbus_pdo_limit_num;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	struct wake_lock reverse_wake_lock;
#else
	struct wakeup_source *reverse_ws;
#endif
	struct task_struct *reverse_watchdog;
	wait_queue_head_t reverse_watchdog_wq;
	struct reverse_limit_data reverse_limit;
	struct oplus_reverse_uisoc_pdo_limit
		reverse_soc_pdo_limit[DEC_REVERSE_PDO_LINIT_COUNT];
	struct oplus_reverse_pdo_limit_t reverse_vbus_pdo_limit[DEC_REVERSE_PDO_LINIT_COUNT];
	struct oplus_reverse_pdo_limit_t reverse_hardreset_pdo_limit[DEC_REVERSE_PDO_LINIT_COUNT];
};
static struct oplus_chg_reverse *g_rvs_chg_chip = NULL;

enum oplus_last_reason {
	LAST_LEVEL_1,
	LAST_LEVEL_2
};

enum oplus_uisoc_reason {
	UISOC_LEVEL_MIN, /* 20-30% */
	UISOC_LEVEL_MID, /* 30-50% */
	UISOC_LEVEL_NORMAL, /*50- 70% */
	UISOC_LEVEL_MID_HIGH, /*70- 80% */
	UISOC_LEVEL_HIGH, /*80- 100% */
	UISOC_LEVEL_INVALID
};

enum oplus_temp_reason {
	TEMP_LEVLE_RANGE_COLD = 0,
	TEMP_LEVLE_RANGE_COOL, /* 0 ~ 16 */
	TEMP_LEVLE_RANGE_NORMAL, /* 16 ~ 45 */
	TEMP_LEVLE_RANGE_WARM, /* 45 ~ 52 */
	TEMP_LEVLE_RANGE_HIGH,
};

enum oplus_reverse_pdo_reason {
	REVERSE_PDO_FIRST_LEVEL, /* 5V1.5A */
	REVERSE_PDO_SECOND_LEVEL, /* 9V1A */
	REVERSE_PDO_THIRD_LEVEL, /* 9V1.5A */
	REVERSE_PDO_FOURTH_LEVEL, /* 9V2A */
	REVERSE_PDO_FIFTH_LEVEL, /* 9V2.5A */
	REVERSE_PDO_SIXTH_LEVEL, /* 9V3A */
	REVERSE_PDO_MAX,
};

static const char *disable_vote_level_names[] = {
	"USER_VOTER",
	"BATT_TEMP_VOTER",
	"BATT_SOC_VOTER",
	"CURR_ERR_VOTER",
	"CURR_LIMIT_VOTER",
	"USB_VOTER",
	"HW_ERR_VOTER",
	"BATT_VOL_VOTER",
	"AUTH_VOTER",
	"SVID_VOTER",
	"NO_DATA_VOTER"
};

static void oplus_reverse_set_awake(struct oplus_chg_reverse *chip, bool awake);

#define CANNOT_OPEN_HIGH_PWR_PATH_VOL_MV 2000
#define LOWER_IBUS 500
#define NORMAL_VBUS 5000
#define PD_VBUS 9000
#define NORMAL_IBUS 1000
#define REVERSE_IBUS 1500
#define WAIT_HIGH_REVERSE_COLSE 1000

__maybe_unused static bool
is_gauge_topic_available(struct oplus_chg_reverse *chip)
{
	if (!chip->gauge_topic)
		chip->gauge_topic = oplus_mms_get_by_name("gauge");
	return !!chip->gauge_topic;
}


__maybe_unused static bool
is_comm_topic_available(struct oplus_chg_reverse *chip)
{
	if (!chip->comm_topic)
		chip->comm_topic = oplus_mms_get_by_name("common");
	return !!chip->comm_topic;
}
#define PDO_LENGTH_LIMIT 2
static int oplus_reverse_uisoc_pdo_limit_parse_dt(struct oplus_chg_reverse *chip)
{
	int i = 0;
	int j = 0;
	int rc = 0;
	int num_elems = 0;
	uint32_t comm_data[2];
	uint32_t data[2 * PDO_LENGTH_LIMIT];
	u32 pdo;
	u32 comm_pdo;
	u32 oplus_pdo;
	struct device_node *node = chip->dev->of_node;
	struct device_node *uisoc_list_node, *temp_node;
	const char *temp_level_names[] = {
		"temp_level_range_cold",
		"temp_level_range_cool",
		"temp_level_range_normal",
		"temp_level_range_warm",
		"temp_level_range_high"
	};
	const char *soc_node_names[] = {
		"reverse_uisoc_range_min_pdo_limit_list",
		"reverse_uisoc_range_mid_pdo_limit_list",
		"reverse_uisoc_range_normal_pdo_limit_list",
		"reverse_uisoc_range_mid_high_pdo_limit_list",
		"reverse_uisoc_range_high_pdo_limit_list"
	};

	uisoc_list_node = of_get_child_by_name(node,
		"oplus,reverse_uisoc_pdo_limit_list");
	if (!uisoc_list_node) {
		chg_err("Failed get reverse_uisoc_pdo_limit_list node\n");
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(soc_node_names); i++) {
		temp_node = of_get_child_by_name(uisoc_list_node, soc_node_names[i]);
		if (!temp_node) {
			chg_info("SOC node %s not found\n", soc_node_names[i]);
			of_node_put(temp_node);
			break;
		}

		for (j = 0; j < ARRAY_SIZE(temp_level_names); j++) {
			num_elems = of_property_count_elems_of_size(temp_node,
				temp_level_names[j], sizeof(u32));
			if (num_elems < 0) {
				chg_info("Temp property %s missing in %s, stop temp parsing for this SOC\n",
					temp_level_names[j], soc_node_names[i]);
				break;
			}
			if (num_elems == PDO_LENGTH_LIMIT) {
				rc = of_property_read_u32_array(temp_node,
					temp_level_names[j], comm_data, 2);
				if (rc < 0) {
					chg_err("Failed  for %s, rc=%d\n", temp_level_names[j], rc);
					of_node_put(temp_node);
					goto out;
				}

				/* oplus use comm */
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].comm_pdo_limit.
					reverse_pdo_voltage = comm_data[0];
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].comm_pdo_limit.
					reverse_pdo_current = comm_data[1];
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].oplus_pdo_limit.
					reverse_pdo_voltage = comm_data[0];
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].oplus_pdo_limit.
					reverse_pdo_current = comm_data[1];

				pdo = (comm_data[0] << 16) | (comm_data[1] & 0xFFFF);
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].
					comm_pdo_limit.reverse_pdo = pdo;
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].
					oplus_pdo_limit.reverse_pdo = pdo;
				chg_info("comm_pdo_limit: voltage:[%d], current:[%d]\n",
					comm_data[0], comm_data[1]);
			} else if (num_elems == 2 * PDO_LENGTH_LIMIT) {
				rc = of_property_read_u32_array(temp_node, temp_level_names[j], data, 4);
				if (rc < 0) {
					chg_err("Failed to read 4 values for %s, rc=%d\n",
						temp_level_names[j], rc);
					of_node_put(temp_node);
					goto out;
				}

				/* comm */
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].comm_pdo_limit.
					reverse_pdo_voltage = data[0];
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].comm_pdo_limit.
					reverse_pdo_current = data[1];

				/* oplus  */
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].oplus_pdo_limit.
					reverse_pdo_voltage = data[2];
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].oplus_pdo_limit.
					reverse_pdo_current = data[3];

				comm_pdo = (data[0] << 16) | (data[1] & 0xFFFF);
				oplus_pdo = (data[2] << 16) | (data[3] & 0xFFFF);
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].
					comm_pdo_limit.reverse_pdo = comm_pdo;
				chip->reverse_soc_pdo_limit[i].reverse_temp_pdo_limit[j].
					oplus_pdo_limit.reverse_pdo = oplus_pdo;
				chg_info("comm_pdo_limit: voltage:[%d], current:[%d]\n", data[0], data[1]);
				chg_info("oplus_pdo_limit: voltage:[%d], current:[%d]\n", data[2], data[3]);
			} else {
				chg_err("Invalid number of values (%d) for %s in %s, expected 2 or 4\n",
					num_elems, temp_level_names[j], soc_node_names[i]);
				rc = -EINVAL;
				goto out;
			}
		}
		of_node_put(temp_node);
	}

	rc = 0;
out:
	of_node_put(uisoc_list_node);
	return rc;
}

static int oplus_reverse_vbus_pdo_limit_parse_dt(struct oplus_chg_reverse *chip)
{
	int i;
	int num;
	int rc = 0;
	uint32_t data;
	struct device_node *node = chip->dev->of_node;

	num = of_property_count_elems_of_size(node, "oplus,reverse_vbus_pdo_limit_list", sizeof(u32));

	chip->is_support_high_reverse = false;
	if (num < 2) {
		chg_err("read oplus,reverse_vbus_pdo_limit_list failed, rc=%d\n", num);
		return num;
	} else if ((num >> 1) > DEC_REVERSE_PDO_LINIT_COUNT) {
		chg_err("too many items in \"oplus,reverse_vbus_pdo_limit_list\"\n");
		num = DEC_REVERSE_PDO_LINIT_COUNT;
		return num;
	} else {
		num /= 2;
		chip->reverse_vbus_pdo_limit_num = num;
		chg_info("reverse_vbus_pdo_limit_num[%d]\n", chip->reverse_vbus_pdo_limit_num);
	}

	for (i = 0; i < num; i++) {
		rc = of_property_read_u32_index(node, "oplus,reverse_vbus_pdo_limit_list", i * 2, &data);
		if (rc < 0) {
			chg_err("read oplus,reverse_vbus_pdo_limit_list index %d failed, rc=%d\n", i * 2, rc);
			continue;
		} else {
			if (data > 10000) {
				chg_err("oplus,reverse_vbus_pdo_limit_list index %d data error, data=%u\n", i, data);
				continue;
			} else {
				chip->reverse_vbus_pdo_limit[i].reverse_pdo_voltage = data;
				if (chip->reverse_vbus_pdo_limit[i].reverse_pdo_voltage >= PD_VBUS)
					chip->is_support_high_reverse = true;
			}
		}
		rc = of_property_read_u32_index(node, "oplus,reverse_vbus_pdo_limit_list", i * 2 + 1, &data);
		if (rc < 0) {
			chg_err("read oplus,reverse_vbus_pdo_limit_list index %d failed, rc=%d\n", i * 2 + 1, rc);
			continue;
		} else {
			chip->reverse_vbus_pdo_limit[i].reverse_pdo_current = data;
		}
		chip->reverse_vbus_pdo_limit[i].reverse_pdo =
		chip->reverse_vbus_pdo_limit[i].reverse_pdo_voltage << 16 | chip->reverse_vbus_pdo_limit[i].reverse_pdo_current;
		chg_info("reverse_vbus_pdo_limit: voltage:[%d], current:[%d]\n",
				chip->reverse_vbus_pdo_limit[i].reverse_pdo_voltage, chip->reverse_vbus_pdo_limit[i].reverse_pdo_current);
	}

	return 0;
}

static int oplus_normal_reverse_hw_ocp_parse_dt(struct oplus_chg_reverse *chip)
{
	int i;
	int num;
	int rc = 0;
	uint32_t data;
	struct device_node *node = chip->dev->of_node;

	num = of_property_count_elems_of_size(node,
		"oplus,normal_reverse_hw_ocp", sizeof(u32));
	if (num < 2) {
		chg_err("read oplus,normal_reverse_hw_ocp failed, rc=%d\n", num);
		return num;
	} else if ((num >> 1) > MAX_NORMAL_REVERSE_COUNT) {
		chg_err("too many items in \"oplus,normal_reverse_hw_ocp\"\n");
		num = MAX_NORMAL_REVERSE_COUNT;
		return num;
	} else {
		num /= 2;
		chip->normal_reverse_count = num;
		chg_info("normal_reverse_count[%d]\n", chip->normal_reverse_count);
	}
	for (i = 0; i < num; i++) {
		rc = of_property_read_u32_index(node,
			"oplus,normal_reverse_hw_ocp", i * 2, &data);
		if (rc < 0) {
			chg_err("read oplus,normal_reverse_hw_ocp index %d failed, rc=%d\n",
				i * 2, rc);
			continue;
		} else {
			if (data > 10000) {
				chg_err("oplus,normal_reverse_hw_ocp index %d data error, data=%u\n",
					i, data);
				continue;
			} else {
				chip->reverse_limit.normal_reverse_current[i] = data;
			}
		}
		rc = of_property_read_u32_index(node,
			"oplus,normal_reverse_hw_ocp", i * 2 + 1, &data);
		if (rc < 0) {
			chg_err("read oplus,normal_reverse_hw_ocp index %d failed, rc=%d\n",
				i * 2 + 1, rc);
			continue;
		} else {
			chip->reverse_limit.normal_reverse_hw_ocp[i] = data;
		}
		chg_info("normal_reverse: current:[%d], hw_ocp:[%d]\n",
			chip->reverse_limit.normal_reverse_current[i], chip->reverse_limit.normal_reverse_hw_ocp[i]);
	}
	return 0;
}

static int oplus_reverse_hardreset_limit_parse_dt(struct oplus_chg_reverse *chip)
{
	int i;
	int num;
	int rc = 0;
	uint32_t data;
	struct device_node *node = chip->dev->of_node;

	num = of_property_count_elems_of_size(node,
		"oplus,reverse_hardreset_limit", sizeof(u32));
	if (num < 2) {
		chg_err("read oplus,reverse_hardreset_limit failed, rc=%d\n", num);
		return num;
	} else if ((num >> 1) > MAX_HARDRESET_LIMIT_COUNT) {
		chg_err("too many items in \"oplus,reverse_hardreset_limit\"\n");
		num = MAX_HARDRESET_LIMIT_COUNT;
		return num;
	} else {
		num /= 2;
		chip->hardreset_limit_count = num;
		chg_info("hardreset_limit_count[%d]\n", chip->hardreset_limit_count);
	}
	for (i = 0; i < num; i++) {
		rc = of_property_read_u32_index(node,
			"oplus,reverse_hardreset_limit", i * 2, &data);
		if (rc < 0) {
			chg_err("read oplus,reverse_hardreset_limit index %d failed, rc=%d\n",
				i * 2, rc);
			continue;
		} else {
			if (data > 10000) {
				chg_err("oplus,reverse_hardreset_limit index %d data error, data=%u\n",
					i, data);
				continue;
			} else {
				chip->reverse_hardreset_pdo_limit[i].reverse_pdo_voltage = data;
			}
		}
		rc = of_property_read_u32_index(node,
			"oplus,reverse_hardreset_limit", i * 2 + 1, &data);
		if (rc < 0) {
			chg_err("read oplus,reverse_hardreset_limit index %d failed, rc=%d\n",
				i * 2 + 1, rc);
			continue;
		} else {
			chip->reverse_hardreset_pdo_limit[i].reverse_pdo_current = data;
		}
		chip->reverse_hardreset_pdo_limit[i].reverse_pdo =
		chip->reverse_hardreset_pdo_limit[i].reverse_pdo_voltage << 16 | chip->reverse_hardreset_pdo_limit[i].reverse_pdo_current;
		chg_info("reverse_hardreset_pdo_limit: voltage:[%d], current:[%d]\n",
				chip->reverse_hardreset_pdo_limit[i].reverse_pdo_voltage, chip->reverse_hardreset_pdo_limit[i].reverse_pdo_current);
	}
	return 0;
}

static int oplus_reverse_parse_dt(struct oplus_chg_reverse *chip)
{
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);
	int rc;
	int i;

	rc = of_property_count_elems_of_size(node, "oplus_spec,reverse-uisoc-limit", sizeof(u32));
	if (rc > 0 && rc <= DEC_REVERSE_UISOC_LINIT_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,reverse-uisoc-limit", (u32 *)chip->reverse_limit.reverse_uisoc_limit, rc);
	}
	for (i = 0; i < DEC_REVERSE_UISOC_LINIT_COUNT; i++) {
		chg_info("reverse_uisoc_limit[%d]=%d \n", i, chip->reverse_limit.reverse_uisoc_limit[i]);
	}
	rc = of_property_count_elems_of_size(node, "oplus_spec,reverse-temp-limit", sizeof(u32));
	if (rc > 0 && rc <= DEC_REVERSE_TEMP_LINIT_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,reverse-temp-limit", (u32 *)chip->reverse_limit.reverse_temp_limit, rc);
	}
	for (i = 0; i < DEC_REVERSE_TEMP_LINIT_COUNT; i++) {
		chg_info("reverse-temp-limit[%d]=%d \n", i, chip->reverse_limit.reverse_temp_limit[i]);
	}
	rc = of_property_count_elems_of_size(node, "oplus_spec,reverse-vbus-limit", sizeof(u32));
	chip->vbus_limit_count = rc;
	if (rc > 0 && rc <= DEC_REVERSE_VBUS_LINIT_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,reverse-vbus-limit", (u32 *)chip->reverse_limit.reverse_vbus_limit, rc);
	}
	for (i = 0; i < chip->vbus_limit_count; i++) {
		chg_info("reverse-vbus-limit[%d]=%d \n", i, chip->reverse_limit.reverse_vbus_limit[i]);
	}
	rc = of_property_count_elems_of_size(node, "oplus_spec,reverse-vbat-limit", sizeof(u32));
	chip->vbat_limit_count = rc;
	if (rc > 0 && rc <= DEC_REVERSE_VBAT_LINIT_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,reverse-vbat-limit", (u32 *)chip->reverse_limit.reverse_vbat_limit, rc);
	}
	for (i = 0; i < chip->vbat_limit_count; i++) {
		chg_info("reverse-vbat-limit[%d]=%d \n", i, chip->reverse_limit.reverse_vbat_limit[i]);
	}
	rc = of_property_count_elems_of_size(node, "oplus_spec,last-uisoc-limit", sizeof(u32));
	if (rc > 0 && rc <= DEC_LAST_LINIT_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,last-uisoc-limit", (u32 *)chip->reverse_limit.last_uisoc_limit, rc);
	}
	for (i = 0; i < DEC_LAST_LINIT_COUNT; i++) {
		chg_info("last-uisoc-limit[%d]=%d \n", i, chip->reverse_limit.last_uisoc_limit[i]);
	}
	rc = of_property_count_elems_of_size(node, "oplus_spec,last-current", sizeof(u32));
	if (rc > 0 && rc <= DEC_LAST_LINIT_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,last-current", (u32 *)chip->reverse_limit.last_current, rc);
	}
	for (i = 0; i < DEC_LAST_LINIT_COUNT; i++) {
		chg_info("oplus_spec,last-current[%d]=%d \n", i, chip->reverse_limit.last_current[i]);
	}
	rc = of_property_count_elems_of_size(node, "oplus_spec,reverse-led-on-limit", sizeof(u32));
	if (rc > 0 && rc <= DEC_LAST_LINIT_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,reverse-led-on-limit", (u32 *)chip->reverse_limit.reverse_led_on_limit, rc);
	}
	for (i = 0; i < DEC_LAST_LINIT_COUNT; i++) {
		chg_info("oplus_spec,reverse_led_on_limit[%d]=%d \n", i, chip->reverse_limit.reverse_led_on_limit[i]);
	}
	chip->led_on_pdo_limit =
	chip->reverse_limit.reverse_led_on_limit[LAST_LEVEL_1] << 16 | chip->reverse_limit.reverse_led_on_limit[LAST_LEVEL_2];

	rc = of_property_count_elems_of_size(node,
		"oplus_spec,reverse-authenticate-limit", sizeof(u32));
	if (rc > 0 && rc <= DEC_LAST_LINIT_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,reverse-authenticate-limit",
			(u32 *)chip->reverse_limit.reverse_authenticate_limit, rc);
	}
	for (i = 0; i < DEC_LAST_LINIT_COUNT; i++) {
		chg_info("oplus_spec,reverse-authenticate-limit[%d] = %d\n",
			i, chip->reverse_limit.reverse_authenticate_limit[i]);
	}
	chip->authenticate_pdo_limit =
		(chip->reverse_limit.reverse_authenticate_limit[LAST_LEVEL_1] << 16) |
		(chip->reverse_limit.reverse_authenticate_limit[LAST_LEVEL_2]);

	rc = of_property_count_elems_of_size(node,
		"oplus_spec,reverse-over-ibus-limit", sizeof(u32));
	if (rc > 0 && rc <= DEC_LAST_LINIT_COUNT) {
		rc = of_property_read_u32_array(node, "oplus_spec,reverse-over-ibus-limit",
			(u32 *)chip->reverse_limit.reverse_over_ibus_limit, rc);
	}
	for (i = 0; i < DEC_LAST_LINIT_COUNT; i++) {
		chg_info("oplus_spec,reverse-over-ibus-limit[%d] = %d\n",
			i, chip->reverse_limit.reverse_over_ibus_limit[i]);
	}
	chip->over_ibus_pdo_limit =
		(chip->reverse_limit.reverse_over_ibus_limit[LAST_LEVEL_1] << 16) |
		(chip->reverse_limit.reverse_over_ibus_limit[LAST_LEVEL_2]);

	rc = of_property_read_u32(node, "oplus,reverse_ibus_lower_oplus", &chip->reverse_ibus_lower_oplus);
	if (rc) {
		chg_err("get oplus,reverse_normal_current error, rc=%d\n", rc);
		chip->reverse_ibus_lower_oplus = 100;
	}
	rc = of_property_read_u32(node, "oplus,reset_watchdog_time", &chip->reset_watchdog_time);
	if (rc) {
		chg_err("get oplus,reverse_normal_current error, rc=%d\n", rc);
		chip->reset_watchdog_time = 1000;
	}
	chip->use_high_reverse_awake = of_property_read_bool(node, "oplus,use_high_reverse_awake");
	chg_info("use_high_reverse_awake is %d", chip->use_high_reverse_awake);
	chip->use_cp_reverse = of_property_read_bool(node, "oplus,use_cp_reverse");
	chg_info("use_cp_reverse is %d", chip->use_cp_reverse);
	chip->sink_svid_support = of_property_read_bool(node, "oplus,sink_svid_support");
	chg_info("sink_svid_support is %d", chip->sink_svid_support);
	oplus_reverse_uisoc_pdo_limit_parse_dt(chip);
	oplus_reverse_vbus_pdo_limit_parse_dt(chip);
	oplus_reverse_hardreset_limit_parse_dt(chip);
	rc = of_property_read_u32(node, "oplus,thired_oplus_svid_limit", &chip->thired_oplus_svid_limit);
	if (rc) {
		chg_err("get thired_oplus_svid_limit error, use REVERSE_PDO_FIRST_LEVEL rc=%d\n", rc);
		chip->thired_oplus_svid_limit =
			chip->reverse_soc_pdo_limit[UISOC_LEVEL_MIN].
			reverse_temp_pdo_limit[TEMP_LEVLE_RANGE_COLD].
			comm_pdo_limit.reverse_pdo_current;
	}
	oplus_normal_reverse_hw_ocp_parse_dt(chip);
	return 0;
}

#define ENABLE_ERROR_COUNT_LEVEL 3
#define CHECK_BOOST_VOL_DELAY 1000
static int oplus_reverse_get_boost_vol(
	struct oplus_chg_reverse *chip, int *vout);
static void oplus_reverse_error_flag_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip = container_of(dwork,
		struct oplus_chg_reverse, reverse_error_flag_work);
	static int pre_high_reverse_err_flag;
	static int pre_hard_reset_count;
	struct mms_msg *msg;
	int cp_vbus_mv;
	int rc;
	int i;

	if (get_client_vote(chip->high_reverse_disable_votable, USER_VOTER)) {
		chip->high_reverse_err_flag = HPR_USER_VOTER_DISABLE;
	} else if (get_client_vote(
		chip->high_reverse_disable_votable, BATT_TEMP_VOTER)) {
		chip->high_reverse_err_flag = HPR_BATT_TEMP_VOTER_DISABLE;
	} else if (get_client_vote(
		chip->high_reverse_disable_votable, BATT_SOC_VOTER)) {
		chip->high_reverse_err_flag = HPR_BATT_SOC_VOTER_DISABLE;
	} else if (get_client_vote(
		chip->high_reverse_disable_votable, CURR_ERR_VOTER)) {
		chip->high_reverse_err_flag = HPR_CURR_ERR_VOTER_DISABLE;
	} else if (get_client_vote(
		chip->high_reverse_disable_votable, AUTH_VOTER)) {
		chip->high_reverse_err_flag = HPR_AUTH_VOTER_DISABLE;
	} else if (get_client_vote(
		chip->high_reverse_disable_votable, SVID_VOTER)) {
		chip->high_reverse_err_flag = HPR_SVID_VOTER_DISABLE;
	} else if (get_client_vote(
		chip->high_reverse_disable_votable, USB_VOTER)) {
		chip->high_reverse_err_flag = HPR_USB_VOTER_DISABLE;
	} else if (get_client_vote(
		chip->high_reverse_disable_votable, HW_ERR_VOTER) ||
		(chip->hard_reset_count > 0 &&
		chip->hard_reset_count != pre_hard_reset_count)) {
		pre_hard_reset_count = chip->hard_reset_count;
		chip->high_reverse_err_flag = HPR_HW_ERR_VOTER_DISABLE;
	} else if (chip->pre_pdo_voltage > NORMAL_VBUS &&
		chip->req_volt > NORMAL_VBUS) {
		rc = oplus_reverse_get_boost_vol(chip, &cp_vbus_mv);
		if (rc < 0) {
			chg_err("oplus_reverse_check_vbus_error \r\n");
			return;
		}
		if (cp_vbus_mv <= HIGH_VBUS_START) {
			for (i = 0; i <= ENABLE_ERROR_COUNT_LEVEL; i++) {
				msleep(CHECK_BOOST_VOL_DELAY);
				oplus_reverse_get_boost_vol(chip, &cp_vbus_mv);
				chg_info("reverse check device err %d boost vol =%d \n", i, cp_vbus_mv);
				if (!chip->reverse_enable)
					return;
				if (cp_vbus_mv <= HIGH_VBUS_START)
					continue;
				else
					break;
			}
		}
		if (cp_vbus_mv <= HIGH_VBUS_START)
			chip->high_reverse_err_flag = HPR_DEVICE_ERROR_DISABLE;
		else
			chip->high_reverse_err_flag = HPR_NO_VOTER_DISABLE;
	} else {
		chip->high_reverse_err_flag = HPR_NO_VOTER_DISABLE;
	}

	if (chip->high_reverse_err_flag > 0 && chip->high_reverse_err_flag !=
		pre_high_reverse_err_flag) {
		chg_info("high_reverse_err_flag =%d \n", chip->high_reverse_err_flag);
		msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
						REVERSE_ITEM_HIGH_REVERSE_ERR);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			return;
		}
		rc = oplus_mms_publish_msg(chip->reverse_topic, msg);
		if (rc < 0) {
			chg_err("publish reverse err flag msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}
	pre_high_reverse_err_flag = chip->high_reverse_err_flag;

	return;
}

static int oplus_reverse_set_source_pdo
	(struct oplus_chg_reverse *chip, int pdo0_volt, int pdo0_curr, int pdo1_volt, int pdo1_curr)
{
	int rc;

	rc = oplus_chg_ic_func(chip->reverse_ic,
			OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO,
			pdo0_volt, pdo0_curr, pdo1_volt, pdo1_curr);
	if (rc < 0) {
		chg_err("can't pdo0 voltage: %d, current: %d, pdo1 voltage: %d, current: %d, rc=%d\n",
			pdo0_volt, pdo0_curr, pdo1_volt, pdo1_curr, rc);
		return -1;
	}
	return 0;
}

static int oplus_set_reverse_chg_type_ufcs(struct oplus_chg_reverse *chip, bool enable)
{
	return 0;
}

static int oplus_set_reverse_chg_type_pps(struct oplus_chg_reverse *chip, bool enable)
{
	return 0;
}

static int oplus_set_reverse_chg_type_pd(struct oplus_chg_reverse *chip, bool enable)
{
	int rc = -1;
	if (!chip->reverse_ic) {
		chg_err("reverse_ic is NULL");
		return -1;
	}

	if (enable) {
		vote(chip->high_reverse_disable_votable, USER_VOTER, false, 0, false);
		oplus_reverse_set_source_pdo(chip, NORMAL_VBUS, NORMAL_IBUS, PD_VBUS, NORMAL_IBUS);
	} else {
		vote(chip->high_reverse_disable_votable, USER_VOTER, true, 1, false);
		oplus_reverse_set_source_pdo(chip, NORMAL_VBUS, NORMAL_IBUS, NORMAL_VBUS, NORMAL_IBUS);
	}
	return rc;
}
static bool oplus_check_reverse_chg_is_ufcs_type(struct oplus_chg_reverse *chip)
{
	return false;
}

static bool oplus_check_reverse_chg_is_pps_type(struct oplus_chg_reverse *chip)
{
	int type = 0;
	int rc;

	if (chip->reverse_ic) {
		rc = oplus_chg_ic_func(chip->reverse_ic,
			OPLUS_IC_FUNC_GET_REVERSE_CHG_TYPE, &type);
		if (rc < 0)
				chg_err("can't get reverse sink rdo, rc=%d\n", rc);
	}
	if (type == BIT(1))
		return true;
	return false;
}
static bool oplus_check_reverse_chg_is_pd_type(struct oplus_chg_reverse *chip)
{
	int type = 0;
	int rc;

	if (chip->reverse_ic) {
		rc = oplus_chg_ic_func(chip->reverse_ic,
			OPLUS_IC_FUNC_GET_REVERSE_CHG_TYPE, &type);
		if (rc < 0)
				chg_err("can't get reverse sink rdo, rc=%d\n", rc);
	}
	if (type == BIT(0))
		return true;

	return false;
}
static void oplus_reverse_chg_type_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip = container_of(dwork,
		struct oplus_chg_reverse, reverse_chg_type_work);
	struct mms_msg *msg;
	static int pre_reverse_chg_type = 0;
	int rc;

	if (oplus_check_reverse_chg_is_ufcs_type(chip))
		chip->reverse_chg_type = REVERSE_CHG_TYPE_UFCS;
	else if (oplus_check_reverse_chg_is_pps_type(chip))
		chip->reverse_chg_type = REVERSE_CHG_TYPE_PPS;
	else if (oplus_check_reverse_chg_is_pd_type(chip))
		chip->reverse_chg_type = REVERSE_CHG_TYPE_PD;
	else if (chip->reverse_enable)
		chip->reverse_chg_type = REVERSE_CHG_TYPE_NORMAL;
	else
		chip->reverse_chg_type = REVERSE_CHG_TYPE_UNKNOWN;

	if (chip->reverse_chg_type != pre_reverse_chg_type) {
		msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
						REVERSE_ITEM_REVERSE_CHG_TYPE);
		if (msg == NULL) {
			chg_err("alloc msg error\n");
			return;
		}
		rc = oplus_mms_publish_msg(chip->reverse_topic, msg);
		if (rc < 0) {
			chg_err("publish reverse chg type msg error, rc=%d\n", rc);
			kfree(msg);
		}
		pre_reverse_chg_type = chip->reverse_chg_type;
	}
}

#define HARD_RESET_COUNT_LEVEL 5
static void oplus_reverse_clear_keep_status_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip = container_of(dwork,
		struct oplus_chg_reverse, reverse_clear_keep_status_work);

	if (!chip->reverse_enable) {
		chip->req_volt = 0;
		chip->req_current = 0;
		chip->hard_reset_count = 0;
		chip->pre_pdo_voltage = 0;
		chip->pre_pdo_current = 0;
		chip->high_reverse_count = 0;
		chip->drop_down_count = 0;
		vote(chip->reverse_pdo_votable, HW_ERR_VOTER, false, 0, false);
		vote(chip->high_reverse_disable_votable, HW_ERR_VOTER, false, 0, false);
		vote(chip->high_reverse_disable_votable, USB_VOTER, false, 0, false);
		vote(chip->reverse_pdo_votable, CURR_ERR_VOTER, false, 0, false);
		vote(chip->reverse_pdo_votable, USB_VOTER, false, 0, false);
		vote(chip->high_reverse_disable_votable, CURR_ERR_VOTER, false, 0, false);
		vote(chip->reverse_pdo_votable, VOL_DIFF_VOTER, false, 0, false);
		vote(chip->reverse_pdo_votable, VBUS_MV_VOTER, false, 0, false);
		chg_info("clear keep status \n");
	}
}

static void oplus_reverse_chg_test_mode_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip = container_of(dwork,
		struct oplus_chg_reverse, reverse_chg_test_mode_work);
	int rc;
	int pass_retry_cnt = 10;
	int vbus_mv = 0;

	/* system shoud stay awake when testing */
	oplus_reverse_set_awake(chip, true);

	chip->test_high_pwr_vbus_mv = 0;
	chip->test_result = REVERSE_TEST_MODE_RESULT_FAIL;
	rc = oplus_chg_ic_func(g_rvs_chg_chip->reverse_ic,
					OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN, 1);
	do {
		msleep(100);
		vbus_mv = oplus_wired_get_vbus();
		pass_retry_cnt--;
		chg_info("vbus_mv = %d, open pass_retry_cnt = %d\n", vbus_mv, pass_retry_cnt);
	} while (vbus_mv < 4500 && pass_retry_cnt > 0);
	chip->test_high_pwr_vbus_mv = vbus_mv;
	if (vbus_mv < 4500) {
		rc = oplus_chg_ic_func(g_rvs_chg_chip->reverse_ic,
						OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN, 0);
		chip->reverse_chg_testing = false;
		chip->test_result = REVERSE_TEST_MODE_OPEN_HIGH_PWR_MODE_FAIL;
		/* set awake false after testing complete*/
		oplus_reverse_set_awake(chip, false);
		chg_err("vbus_mv = %d, test result REVERSE_TEST_MODE_OPEN_HIGH_PWR_MODE_FAIL\n", vbus_mv);
		return;
	}

	pass_retry_cnt = 20;
	do {
		rc = oplus_chg_ic_func(g_rvs_chg_chip->reverse_ic,
						OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN, 0);
		msleep(100);
		pass_retry_cnt--;
		vbus_mv = oplus_wired_get_vbus();
		chg_info("vbus_mv = %d, close pass_retry_cnt = %d\n", vbus_mv, pass_retry_cnt);
	} while (vbus_mv > CANNOT_OPEN_HIGH_PWR_PATH_VOL_MV && pass_retry_cnt > 0);
	if (vbus_mv < CANNOT_OPEN_HIGH_PWR_PATH_VOL_MV) {
		chip->test_result = REVERSE_TEST_MODE_RESULT_SUCCESS;
		chg_info("vbus_mv = %d, test result REVERSE_TEST_MODE_RESULT_SUCCESS\n", vbus_mv);
	} else {
		chip->test_high_pwr_vbus_mv = vbus_mv;
		chip->test_result = REVERSE_TEST_MODE_CLOSE_HIGH_PWR_MODE_FAIL;
		chg_err("vbus_mv = %d, test result REVERSE_TEST_MODE_CLOSE_HIGH_PWR_MODE_FAIL\n", vbus_mv);
	}

	chip->reverse_chg_testing = false;
	/* set awake false after testing complete*/
	oplus_reverse_set_awake(chip, false);
	return;
}

static int oplus_reverse_temp_cur_range_init(struct oplus_chg_reverse *chip)
{
	int vbat_temp_cur;
	int rc;
	union mms_msg_data data = { 0 };

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, false);
	if (unlikely(rc < 0)) {
		chg_err("can't get comm_item_shell_temp, rc=%d\n", rc);
		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
		if (unlikely(rc < 0)) {
			chg_err("can't get gauge_item_temp, rc=%d\n", rc);
			chip->shell_temp = 250;
		} else {
			chip->shell_temp = data.intval;
		}
	} else {
		chip->shell_temp = data.intval;
	}

	vbat_temp_cur = chip->shell_temp;
	chip->ibus_ov = false;
	if (vbat_temp_cur < chip->reverse_limit.reverse_temp_limit[TEMP_LEVLE_RANGE_COLD]) /*0C*/
		chip->reverse_temp_cur_range = TEMP_LEVLE_RANGE_COLD;
	else if (vbat_temp_cur < chip->reverse_limit.reverse_temp_limit[TEMP_LEVLE_RANGE_COOL]) /*0-16C*/
		chip->reverse_temp_cur_range = TEMP_LEVLE_RANGE_COOL;
	else if (vbat_temp_cur < chip->reverse_limit.reverse_temp_limit[TEMP_LEVLE_RANGE_NORMAL]) /*16-45C*/
		chip->reverse_temp_cur_range = TEMP_LEVLE_RANGE_NORMAL;
	else if (vbat_temp_cur < chip->reverse_limit.reverse_temp_limit[TEMP_LEVLE_RANGE_WARM]) /*45-52C*/
		chip->reverse_temp_cur_range = TEMP_LEVLE_RANGE_WARM;
	else
		chip->reverse_temp_cur_range = TEMP_LEVLE_RANGE_HIGH;
	chg_info("vbat_temp_cur =%d reverse_temp_cur_range =%d\r\n", vbat_temp_cur, chip->reverse_temp_cur_range);
	return 0;
}

static void oplus_normal_reverse_set_hw_ocp_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip = container_of(dwork,
		struct oplus_chg_reverse, normal_reverse_set_hw_ocp_work);
	int ocp_count = 0;
	int ocp_level = 0;
	int rc;
	int i;

	if (chip->normal_reverse_count == 0)
		return;

	if (chip->reverse_enable) {
		for (i = 0; i < chip->normal_reverse_count; i++) {
			if (chip->pre_pdo_current != 0 &&
				chip->pre_pdo_current == chip->reverse_limit.normal_reverse_current[i]) {
				ocp_level = chip->reverse_limit.normal_reverse_hw_ocp[i];
				break;
			}
		}
		if (ocp_level != 0) {
			rc = oplus_chg_ic_func(chip->reverse_ic,
					OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP,
					ocp_level);
			if (rc < 0)
				chg_err("can't set normal_reverse_hw_ocp =%d, rc=%d\n", ocp_level, rc);
		}
	} else {
		ocp_count = chip->normal_reverse_count - 1;
		rc = oplus_chg_ic_func(chip->reverse_ic,
				OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP,
				chip->reverse_limit.normal_reverse_hw_ocp[ocp_count]);
		if (rc < 0)
			chg_err("can't set normal_reverse_hw_ocp =%d, rc=%d\n",
			chip->reverse_limit.normal_reverse_hw_ocp[ocp_count], rc);
	}
}

static void oplus_get_last_high_reverse_pdo(struct oplus_chg_reverse *chip, int pdo_voltage)
{
	static u32 last_high_reverse_pdo;
	struct mms_msg *msg;
	int rc;

	if (pdo_voltage == NORMAL_VBUS && chip->pre_pdo_voltage > NORMAL_VBUS) {
		last_high_reverse_pdo = chip->pre_pdo_voltage << 16 | chip->pre_pdo_current;
		if (chip->last_high_reverse_pdo != last_high_reverse_pdo) {
			chip->last_high_reverse_pdo = last_high_reverse_pdo;
			msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
							REVERSE_ITEM_LAST_HIGH_REVERSE_PDO);
			if (msg == NULL) {
				chg_err("alloc msg error\n");
				return;
			}
			rc = oplus_mms_publish_msg(chip->reverse_topic, msg);
			if (rc < 0) {
				chg_err("publish reverse chg last high reverse pdo error, rc=%d\n", rc);
				kfree(msg);
			}
		}
	}
}

static int oplus_reverse_set_pdo(struct oplus_chg_reverse *chip, u32 target_cap_pdo0, u32 target_pdo)
{
	int rc;
	int i;
	int pdo0_volt;
	int pdo0_curr;
	int pdo_voltage;
	int pdo_current;

	pdo0_volt = (target_cap_pdo0 & 0xFFFF0000) >> 16;
	pdo0_curr = (target_cap_pdo0 & 0xFFFF);
	pdo_voltage = (target_pdo & 0xFFFF0000) >> 16;
	pdo_current = (target_pdo & 0xFFFF);

	if (pdo_voltage > NORMAL_VBUS && (pdo0_volt < NORMAL_VBUS || pdo0_curr < LOWER_IBUS)) {
		pdo0_volt = NORMAL_VBUS;
		pdo0_curr = LOWER_IBUS;
	}

	oplus_get_last_high_reverse_pdo(chip, pdo_voltage);

	if (get_client_vote(chip->reverse_pdo_votable, USER_VOTER) > 0) {
		chg_info("user_voter control, plase remove typec retry or wait 30s\n");
		return 0;
	}

	if (get_effective_result(chip->high_reverse_disable_votable) > 0 && pdo_voltage > NORMAL_VBUS) {
		for (i = 0; i < ARRAY_SIZE(disable_vote_level_names); i++) {
			if (get_client_vote(chip->high_reverse_disable_votable, disable_vote_level_names[i]) > 0)
				chg_info("reverse vote[%d][%s] disable high reverse\n", i, disable_vote_level_names[i]);
		}
		chg_info("reverse vote disable set %d mv \n", NORMAL_VBUS);
		pdo_voltage = NORMAL_VBUS;
		pdo_current = chip->reverse_soc_pdo_limit[UISOC_LEVEL_MIN].
				reverse_temp_pdo_limit[TEMP_LEVLE_RANGE_COLD].comm_pdo_limit.reverse_pdo_current;
	}
	if (pdo_voltage == chip->pre_pdo_voltage && pdo_current == chip->pre_pdo_current &&
		chip->target_cap_pdo0 == chip->pre_target_cap_pdo0) {
		chg_info("set the same pdo0 voltage: %d, current: %d, pdo1 voltage: %d, current: %d return\n",
			pdo0_volt, pdo0_curr, pdo_voltage, pdo_current);
		return 0;
	}

	if (pdo_voltage == NORMAL_VBUS && pdo_current > chip->pre_pdo_current) {
		cancel_delayed_work_sync(&chip->normal_reverse_set_hw_ocp_work);
		chip->pre_pdo_current = pdo_current;
		schedule_delayed_work(&chip->normal_reverse_set_hw_ocp_work, 0);
	}

	if (chip->reverse_ic) {
		rc = oplus_chg_ic_func(chip->reverse_ic,
				OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO,
				pdo0_volt, pdo0_curr, pdo_voltage, pdo_current);
		if (rc < 0)
			chg_err("can't pdo0 voltage: %d, current: %d, pdo1 voltage: %d, current: %d, rc=%d\n",
				pdo0_volt, pdo0_curr, pdo_voltage, pdo_current, rc);
	}

	if (pdo_voltage == NORMAL_VBUS && pdo_current < chip->pre_pdo_current) {
		cancel_delayed_work_sync(&chip->normal_reverse_set_hw_ocp_work);
		schedule_delayed_work(&chip->normal_reverse_set_hw_ocp_work,
			msecs_to_jiffies(REVERSE_ENABLE_DELAY_MS));
	}
	chip->pre_pdo_voltage = pdo_voltage;
	chip->pre_pdo_current = pdo_current;
	chip->pre_target_cap_pdo0 = chip->target_cap_pdo0;
	return 0;
}

PARSE_RESULT parse_numbers(const char *str, int *num1, int *num2, int *num3)
{
	char buffer[32] = {0};
	int parsed  = 0;
	char verify[8] = {0};
	strncpy(buffer, str, sizeof(buffer) - 1);

	parsed = sscanf(buffer, "%7d,%7d,%7d", num1, num2, num3);
	if (parsed != MAX_DIGITS_NUM) {
		return PARSE_FORMAT_ERROR;
	}

	if (snprintf(verify, sizeof(verify), "%d", *num1) > MAX_DIGITS ||
		snprintf(verify, sizeof(verify), "%d", *num2) > MAX_DIGITS ||
		snprintf(verify, sizeof(verify), "%d", *num3) > MAX_DIGITS)
		return PARSE_OVERFLOW_ERROR;

	if (*num1 > MAX_VOLT || *num1 < MIN_VOLT) {
		chg_err("volt parse error %d\n", *num1);
		return PARSE_RANGE_ERROR;
	}

	if (*num2 > MAX_CURRENT || *num2 < MIN_CURRENT) {
		chg_err("current parse error %d\n", *num2);
		return PARSE_RANGE_ERROR;
	}

	if (*num3 < 0) {
		chg_err("force parse error %d\n", *num3);
		return PARSE_RANGE_ERROR;
	}

	return PARSE_SUCCESS;
}

int oplus_reverse_high_pwr_test(void)
{
	struct oplus_chg_reverse *chip = g_rvs_chg_chip;
	int rc = 0;
	int hw_detect = 0;
	int vbus_mv = 0;

	if (chip == NULL)
		return -EINVAL;

	vbus_mv = oplus_wired_get_vbus();
	hw_detect = oplus_wired_get_hw_detect();
	if (hw_detect == CC_DETECT_PLUGIN || vbus_mv > CANNOT_OPEN_HIGH_PWR_PATH_VOL_MV) {
		chg_info("can not open high power path[%d,%d]\n", hw_detect, vbus_mv);
		return -EINVAL;
	} else if (hw_detect == CC_DETECT_NULL && vbus_mv > CANNOT_OPEN_HIGH_PWR_PATH_VOL_MV) {
		chg_info("can not open high power path[%d,%d]\n", hw_detect, vbus_mv);
		return -EINVAL;
	} else {
		chg_info("can open high power path[%d,%d]\n", hw_detect, vbus_mv);
		chip->reverse_chg_testing = true;
		schedule_delayed_work(&chip->reverse_chg_test_mode_work, 0);
	}
	return rc;
}

int oplus_reverse_chg_set_level(const char *buf, int len)
{
	struct oplus_chg_reverse *chip = g_rvs_chg_chip;
	int volt = 0;
	int curr = 0;
	int user_cmd = 0;
	PARSE_RESULT  parse_ret = 0;
	int rc = 0;

	if (chip == NULL)
		return -EINVAL;
	parse_ret = parse_numbers(buf, &volt, &curr, &user_cmd);
	if (parse_ret != PARSE_SUCCESS) {
		chg_err("buf data error %d\n", parse_ret);
		return -1;
	}
	chg_info("buf data error %d %d %d\n", volt, curr, user_cmd);

	switch (user_cmd) {
	case REVERSE_USER_CMD_SET_LEVEL_DEFAULT:
		chg_info("set level default\n");
		vote(chip->reverse_pdo_votable, COOL_DOWN_VOTER, true, (volt << 16 | curr), false);
		break;
	case REVERSE_USER_CMD_SET_LEVEL_FORCE:
		chg_info("set level force\n");
		if (chip->reverse_pdo_votable)
			vote(chip->reverse_pdo_votable, USER_VOTER, true,
					(volt << 16 | curr), false);
		rc = oplus_reverse_set_source_pdo(chip, NORMAL_VBUS, NORMAL_IBUS, volt, curr);
		if (rc >= 0) {
			chip->pre_pdo_voltage = volt;
			chip->pre_pdo_current = curr;
		}
		break;
	case REVERSE_USER_CMD_SET_HIGH_PWR_PATH_TEST_MODE:
		chg_info("set high power path test mode\n");
		chip->reverse_chg_test_mode	= 1; /* start test mode*/
		oplus_reverse_high_pwr_test();
		break;
	case REVERSE_USER_CMD_GET_HIGH_PWR_PATH_TEST_END:
		chip->reverse_chg_test_mode = 0; /* end test mode*/
		break;
	default:
		chg_err("unknow user cmd %d\n", user_cmd);
		break;
	}
	return 0;
}

#define STR_BUFFER_SIZE 32
int oplus_reverse_chg_info_show(char *buf)
{
	int src_volt = 0;
	int src_current = 0;
	int req_volt = 0;
	int req_current = 0;
	char str_buf[STR_BUFFER_SIZE] = {0};
	int str_len = 0;

	if (g_rvs_chg_chip == NULL)
		return -1;

	src_volt = g_rvs_chg_chip->pre_pdo_voltage;
	src_current = g_rvs_chg_chip->pre_pdo_current;
	req_volt = g_rvs_chg_chip->req_volt;
	req_current = g_rvs_chg_chip->req_current;

	chg_info("oplus_reverse_chg_info_show start [%d]\n", g_rvs_chg_chip->reverse_chg_test_mode);
	if (g_rvs_chg_chip->reverse_chg_test_mode) {
		req_volt = g_rvs_chg_chip->test_high_pwr_vbus_mv;
		req_current = g_rvs_chg_chip->test_result;
	}
	str_len = snprintf(str_buf, STR_BUFFER_SIZE, "%d+%d+%d+%d",
		src_volt, src_current, req_volt, req_current);

	if (str_len <= 0 || str_len >= (STR_BUFFER_SIZE - 1)) {
		chg_err("buf data error %d\n", str_len);
		return -1;
	}

	strncpy(buf, str_buf, str_len);
	buf[str_len] = '\0';

	chg_info("buf data  %s\n", buf);
	return str_len;
}

#define POWER_ROLE_BUFF_LEN 16
int oplus_reverse_chg_get_power_role_status(char *buf)
{
	struct oplus_chg_reverse *chip = g_rvs_chg_chip;

	if (chip == NULL || buf == NULL)
		return 0;

	return scnprintf(buf, POWER_ROLE_BUFF_LEN, "%d\n", chip->reverse_enable);
}

static int oplus_reverse_set_charging(struct oplus_chg_reverse *chip, bool charging)
{
	struct mms_msg *msg;
	int rc = 0;

	if (chip->high_reverse_charging == charging)
		return 0;

	chip->high_reverse_charging = charging;
	chg_info("set high_reverse_charging=%s\n", charging ? "true" : "false");

	if (!chip->high_reverse_charging)
			chip->reverse_max_curr = 0;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
					HIGH_REVERSE_ITEM_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->reverse_topic, msg);
	if (rc < 0) {
		chg_err("publish high reverse charging msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}


	return 0;
}

static void oplus_reverse_force_exit(struct oplus_chg_reverse *chip)
{
	chg_info("reverse force exit!");
	oplus_reverse_set_charging(chip, false);
}

static bool oplus_check_reverse_chg_source_plug_out(
	struct oplus_chg_reverse *chip)
{
	bool enable = 0;
	int rc;

	if (chip->reverse_ic) {
		rc = oplus_chg_ic_func(chip->reverse_ic,
			OPLUS_IC_FUNC_GET_SOURCE_PLUG_OUT, &enable);
		if (rc < 0)
			chg_err("can't get reverse sink rdo, rc=%d\n", rc);
	}

	return enable;
}

static void oplus_reverse_chg_disbale_clear_flags(
	struct oplus_chg_reverse *chip)
{
	vote(chip->high_reverse_disable_votable, BATT_TEMP_VOTER, false, 0, false);
	vote(chip->high_reverse_disable_votable, BATT_SOC_VOTER, false, 0, false);
	vote(chip->high_reverse_disable_votable, CURR_LIMIT_VOTER, false, 0, false);
	vote(chip->high_reverse_disable_votable, USER_VOTER, false, 0, false);
	vote(chip->high_reverse_disable_votable, BATT_VOL_VOTER, false, 0, false);
	vote(chip->reverse_pdo_votable, BATT_TEMP_VOTER, false, 0, false);
	vote(chip->reverse_pdo_votable, BATT_SOC_VOTER, false, 0, false);
	vote(chip->reverse_pdo_votable, CURR_LIMIT_VOTER, false, 0, false);
	vote(chip->reverse_pdo_votable, USER_VOTER, false, 0, false);
	vote(chip->reverse_pdo_votable, BATT_VOL_VOTER, false, 0, false);
	vote(chip->cap_pdo0_votable, BATT_VOL_VOTER, false, 0, false);
	vote(chip->cap_pdo0_votable, BATT_CURR_VOTER, false, 0, false);
	cancel_delayed_work_sync(&chip->reverse_monitor_work);
	oplus_reverse_force_exit(chip);
	schedule_delayed_work(&chip->reverse_chg_type_work, 0);
	schedule_delayed_work(&chip->normal_reverse_set_hw_ocp_work, 0);
	chip->vbus_level = chip->vbus_limit_count;
	chip->vbat_level = chip->vbat_limit_count;
	chip->pre_target_cap_pdo0 = 0;
	chip->kthread_reverse_enable = false;
	chip->high_reverse_enable = false;
}

static void oplus_reverse_online_init(struct oplus_chg_reverse *chip)
{
	chip->last_high_reverse_pdo = 0;
}

static void oplus_reverse_handle_level_for_drop_down(void)
{
	struct oplus_chg_reverse *chip = g_rvs_chg_chip;
	static int pre_drop_down_count = 0;

	if (chip->drop_down_count == pre_drop_down_count
		|| chip->drop_down_count <= 0
		|| chip->drop_down_count <= chip->hard_reset_count)
		return;

	pre_drop_down_count = chip->drop_down_count;

	chg_info("drop_down_count=%d\n", chip->drop_down_count);
	if (chip->drop_down_count <= HARD_RESET_COUNT_LEVEL) {
		vote(chip->reverse_pdo_votable, HW_ERR_VOTER, true,
			chip->reverse_vbus_pdo_limit[REVERSE_PDO_SIXTH_LEVEL -
			chip->drop_down_count].reverse_pdo, false);
	} else {
		vote(chip->reverse_pdo_votable, HW_ERR_VOTER, true,
			(NORMAL_VBUS << 16 | NORMAL_IBUS), false);/* 5V1A */
		vote(chip->high_reverse_disable_votable, HW_ERR_VOTER, true, 1, false);
	}
}

static void oplus_reverse_online_work(struct work_struct *work)
{
	struct oplus_chg_reverse *chip =
		container_of(work, struct oplus_chg_reverse, reverse_online_work);

	if (chip->reverse_enable) {
		oplus_reverse_online_init(chip);
		schedule_delayed_work(&chip->reverse_monitor_work, msecs_to_jiffies(2000));
	} else {
		chg_info("reverse disbale clear flags,source plug out =%d \n",
			oplus_check_reverse_chg_source_plug_out(chip));
		oplus_reverse_chg_disbale_clear_flags(chip);
		if (chip->reverse_ic && chip->use_cp_reverse)
			oplus_reverse_set_source_pdo(chip, NORMAL_VBUS, LOWER_IBUS, NORMAL_VBUS, LOWER_IBUS);
		cancel_delayed_work(&chip->reverse_vbus_retention_check_work);
		schedule_delayed_work(&chip->reverse_vbus_retention_check_work, 0);
		chip->drop_down_count += 1;
		cancel_delayed_work(&chip->reverse_clear_keep_status_work);
		schedule_delayed_work(&chip->reverse_clear_keep_status_work, msecs_to_jiffies(5000));
	}

	if (!chip->use_high_reverse_awake) {
		chip->kthread_reverse_enable = chip->reverse_enable;
		if (chip->reverse_enable)
			wake_up_interruptible(&chip->reverse_watchdog_wq);
	}
}

static int oplus_reverse_charge_start(struct oplus_chg_reverse *chip)
{
	int rc;

	if (chip->reverse_enable) {
		chg_info("oplus reverse charge start\n");
		rc = oplus_reverse_temp_cur_range_init(chip);
		if (rc < 0)
			return rc;

		oplus_reverse_set_charging(chip, true);
		return 0;
	}

	chg_err("reverse not work \n");
	return -EINVAL;
}

#define HYST 10
static int oplus_reverse_update_batt_temp_curr(struct oplus_chg_reverse *chip)
{
	int ret;
	int vbat_temp_cur;
	int diff;
	int upper_bound;
	int lower_bound;
	union mms_msg_data data = { 0 };
	enum oplus_temp_reason target_temp_range;
	enum oplus_temp_reason current_temp_range;
	enum oplus_temp_reason new_temp_range;

	if (!is_comm_topic_available(chip)) {
		chg_err("common topic not found\n");
		vote(chip->high_reverse_disable_votable, NO_DATA_VOTER, true, 1, false);
		return -ENODEV;
	}

	ret = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, false);
	if (unlikely(ret < 0)) {
		chg_err("can't get comm_item_shell_temp, ret=%d\n", ret);
		ret = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
		if (unlikely(ret < 0)) {
			chg_err("can't get gauge_item_temp, ret=%d\n", ret);
			chip->shell_temp = 250;
		} else {
			chip->shell_temp = data.intval;
		}
	} else {
		chip->shell_temp = data.intval;
	}

	vbat_temp_cur = chip->shell_temp;

	if (vbat_temp_cur <
		chip->reverse_limit.reverse_temp_limit[TEMP_LEVLE_RANGE_COLD])
		target_temp_range = TEMP_LEVLE_RANGE_COLD;
	else if (vbat_temp_cur <
		chip->reverse_limit.reverse_temp_limit[TEMP_LEVLE_RANGE_COOL])
		target_temp_range = TEMP_LEVLE_RANGE_COOL;
	else if (vbat_temp_cur <
		chip->reverse_limit.reverse_temp_limit[TEMP_LEVLE_RANGE_NORMAL])
		target_temp_range = TEMP_LEVLE_RANGE_NORMAL;
	else if (vbat_temp_cur <
		chip->reverse_limit.reverse_temp_limit[TEMP_LEVLE_RANGE_WARM])
		target_temp_range = TEMP_LEVLE_RANGE_WARM;
	else
		target_temp_range = TEMP_LEVLE_RANGE_HIGH;

	current_temp_range = chip->reverse_temp_cur_range;

	if (current_temp_range < TEMP_LEVLE_RANGE_COLD ||
	    current_temp_range > TEMP_LEVLE_RANGE_HIGH) {
		chg_err("reverse_temp_cur_range invalid(%d), reset to COLD\n",
			current_temp_range);
		current_temp_range = TEMP_LEVLE_RANGE_COLD;
		chip->reverse_temp_cur_range = current_temp_range;
	}
	if (target_temp_range < TEMP_LEVLE_RANGE_COLD ||
	    target_temp_range > TEMP_LEVLE_RANGE_HIGH) {
		chg_err("target_temp_range invalid(%d), reset to COLD\n",
			target_temp_range);
		target_temp_range = TEMP_LEVLE_RANGE_COLD;
	}

	new_temp_range = current_temp_range;
	diff = (int)target_temp_range - (int)current_temp_range;

	if (diff == 1) {
		upper_bound = chip->reverse_limit.reverse_temp_limit[current_temp_range];
		if (vbat_temp_cur >= upper_bound + HYST)
			new_temp_range = target_temp_range;
	}
	else if (diff == -1) {
		lower_bound = chip->reverse_limit.reverse_temp_limit[target_temp_range];
		if (vbat_temp_cur <= lower_bound - HYST)
			new_temp_range = target_temp_range;
	}
	else if (diff > 1 || diff < -1) {
		chg_info("Direct switching across temperature ranges\n");
		new_temp_range = target_temp_range;
	}

	if (new_temp_range != current_temp_range)
		chip->reverse_temp_cur_range = new_temp_range;

	chg_info("vbat_temp_cur =%d reverse_temp_cur_range =%d \r\n", vbat_temp_cur, chip->reverse_temp_cur_range);

	return ret;
}

static int oplus_reverse_get_uisoc_curr(struct oplus_chg_reverse *chip)
{
	int ret;
	int ui_soc_cur;
	union mms_msg_data data = { 0 };

	if (!is_comm_topic_available(chip)) {
		chg_err("common topic not found\n");
		vote(chip->high_reverse_disable_votable, NO_DATA_VOTER, true, 1, false);
		return -ENODEV;
	}

	ret = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data, false);
	if (ret < 0)
		chg_err("can't get ui soc data, rc= %d", ret);
	else
		chip->ui_soc = data.intval;

	ui_soc_cur = chip->ui_soc;

	if (ui_soc_cur < chip->reverse_limit.reverse_uisoc_limit[UISOC_LEVEL_MIN] &&
		ui_soc_cur >= chip->reverse_limit.last_uisoc_limit[LAST_LEVEL_2]) {				/* 20-30%*/
		chip->reverse_uisoc_cur_range = UISOC_LEVEL_MIN;
	} else if (ui_soc_cur <
		chip->reverse_limit.reverse_uisoc_limit[UISOC_LEVEL_MID]) {
		chip->reverse_uisoc_cur_range = UISOC_LEVEL_MID;
	} else if (ui_soc_cur <
		chip->reverse_limit.reverse_uisoc_limit[UISOC_LEVEL_NORMAL]) {
		chip->reverse_uisoc_cur_range = UISOC_LEVEL_NORMAL;
	} else if (ui_soc_cur <
		chip->reverse_limit.reverse_uisoc_limit[UISOC_LEVEL_MID_HIGH]) {
		chip->reverse_uisoc_cur_range = UISOC_LEVEL_MID_HIGH;
	} else if (ui_soc_cur >=
		chip->reverse_limit.reverse_uisoc_limit[UISOC_LEVEL_MID_HIGH]) {
		chip->reverse_uisoc_cur_range = UISOC_LEVEL_HIGH;
	} else {
		chip->reverse_uisoc_cur_range = UISOC_LEVEL_INVALID;
	}

	chg_info("ui_soc_cur =%d\r\n", ui_soc_cur);

	return ret;
}

#define OPLUS_SVID 		    0x22D9
static void oplus_update_pdo_limit(struct oplus_chg_reverse *chip)
{
	u32 pdo_limit;
	int soc_idx = chip->reverse_uisoc_cur_range;
	int temp_idx = chip->reverse_temp_cur_range;

	if (soc_idx < 0 || soc_idx >= ARRAY_SIZE(chip->reverse_soc_pdo_limit)) {
		chg_err("Invalid soc_idx=%d\n", soc_idx);
		return;
	}
	if (temp_idx < 0 ||
		temp_idx >= ARRAY_SIZE(chip->reverse_soc_pdo_limit[UISOC_LEVEL_MIN].reverse_temp_pdo_limit)) {
		chg_err("Invalid temp_idx=%d\n", temp_idx);
		return;
	}

	if (chip->sink_svid_support && chip->reverse_chg_svid == OPLUS_SVID)
		pdo_limit = chip->reverse_soc_pdo_limit[soc_idx].
				reverse_temp_pdo_limit[temp_idx].oplus_pdo_limit.reverse_pdo;
	else
		pdo_limit = chip->reverse_soc_pdo_limit[soc_idx].
				reverse_temp_pdo_limit[temp_idx].comm_pdo_limit.reverse_pdo;
	vote(chip->reverse_pdo_votable, BATT_SOC_VOTER, true, pdo_limit, false);

	if ((pdo_limit & 0xFFFF0000) >> 16 == NORMAL_VBUS) {
		if (temp_idx <= TEMP_LEVLE_RANGE_COLD ||
		    temp_idx >= TEMP_LEVLE_RANGE_WARM)
			vote(chip->high_reverse_disable_votable, BATT_TEMP_VOTER, true, 1, false);
		if (soc_idx == UISOC_LEVEL_MIN || soc_idx == UISOC_LEVEL_INVALID)
			vote(chip->high_reverse_disable_votable, BATT_SOC_VOTER, true, 1, false);
	} else {
		vote(chip->high_reverse_disable_votable, BATT_TEMP_VOTER, false, 0, false);
		vote(chip->high_reverse_disable_votable, BATT_SOC_VOTER, false, 0, false);
	}

	chg_info("Apply PDO: soc_idx = %d, temp_idx = %d, pdo_limit = %d\n",
		 soc_idx, temp_idx, pdo_limit);
}

static int oplus_reverse_charge_allow_check(struct oplus_chg_reverse *chip)
{
	union mms_msg_data data = { 0 };
	int chg_temp = 0;
	int rc = 0;

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, true);
	if (unlikely(rc < 0)) {
		chg_err("can't get comm_item_shell_temp, rc=%d\n", rc);
		rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
		if (unlikely(rc < 0)) {
			chg_err("can't get gauge_item_temp, rc=%d\n", rc);
			chg_temp = 250;
		} else {
			chg_temp = data.intval;
		}
	} else {
		chg_temp = data.intval;
	}
	chg_info("chg_temp =%d \r\n", chg_temp);
	/*chg_temp over_low or over_high vote*/
	if (chg_temp < chip->reverse_limit.reverse_temp_limit[TEMP_LEVLE_RANGE_COLD]) {
		vote(chip->high_reverse_disable_votable, BATT_TEMP_VOTER, true, 1, false);
	} else if (chg_temp >= chip->reverse_limit.reverse_temp_limit[TEMP_LEVLE_RANGE_WARM]) {
		vote(chip->high_reverse_disable_votable, BATT_TEMP_VOTER, true, 1, false);
	} else {
		vote(chip->high_reverse_disable_votable, BATT_TEMP_VOTER, false, 0, false);
	}

	return 0;
}

static int oplus_reverse_get_boost_curr(struct oplus_chg_reverse *chip, int *iout)
{
	int rc = -1;
	if (iout == NULL)
		return rc;

	if (chip->use_cp_reverse && chip->cp_ic) {
		rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_IOUT, iout);
	} else {
		*iout = oplus_gauge_get_batt_current();
		chg_info("reverse chg: iout = %d\n", *iout);
		rc = *iout < 0 ? -1 : 0;
	}

	return rc;
}

static int oplus_reverse_get_boost_vol(
	struct oplus_chg_reverse *chip, int *vout)
{
	int rc = -1;
	if (vout == NULL)
		return rc;

	if (chip->use_cp_reverse && chip->cp_ic) {
		rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_VOUT, vout);
	} else {
		*vout = oplus_wired_get_vbus();
		chg_info("reverse chg: vout = %d\n", *vout);
		rc = *vout < 0 ? -1 : 0;
	}

	return rc;
}

static int  oplus_reverse_check_ibus_curr(struct oplus_chg_reverse *chip)
{
#define REVERSE_IBUS_OVER_COUNTS 3
#define REVERSE_TBATT_OV_CNT	1
#define OCP_ADD_LEVEL	300
#define OCBAT_ADD_LEVEL		3000

	int ret = 0;
	int cp_ibus_ma;
	int cp_vbus_mv;
	int over_curr_offset = 0;
	int batt_num = oplus_gauge_get_batt_num();

	if (!chip->reverse_enable) {
		chip->ibus_over = 0;
		chip->ibus_lower = 0;
		return -EINVAL;
	}

	if (batt_num <= 0 || batt_num > 2)
		batt_num = 1;

	ret = oplus_reverse_get_boost_curr(chip, &cp_ibus_ma);
	if (ret < 0)
		chg_info("oplus_reverse_check_ibus_error \r\n");
	ret = oplus_reverse_get_boost_vol(chip, &cp_vbus_mv);
	if (ret < 0)
		chg_info("oplus_reverse_check_vbus_error \r\n");

	chg_info("cp_ibus_ma =%d, cp_vbus_mv =%d\r\n", (cp_ibus_ma > 0) ? -cp_ibus_ma : 0, cp_vbus_mv);

	if (cp_vbus_mv > HIGH_VBUS_START) {
		if (cp_ibus_ma < chip->reverse_ibus_lower_oplus) {
			chip->ibus_lower++;
			if (chip->ibus_lower > REVERSE_IBUS_OVER_COUNTS) {
				chg_err("ibus lower than %d, ", chip->reverse_ibus_lower_oplus);
				vote(chip->reverse_pdo_votable, CURR_ERR_VOTER,
					true, chip->over_ibus_pdo_limit, false);/* 5V1.5A */
				vote(chip->high_reverse_disable_votable, CURR_ERR_VOTER, true, 1, false);
				chip->ibus_lower = 0;
			}
		} else {
			chip->ibus_lower = 0;
		}

		if (chip->use_cp_reverse)
			over_curr_offset = OCP_ADD_LEVEL;
		else
			over_curr_offset = OCBAT_ADD_LEVEL;

		if (cp_ibus_ma > over_curr_offset + (chip->pre_pdo_current * (2 / batt_num))) {
			chip->ibus_over++;
			if (chip->ibus_over > REVERSE_IBUS_OVER_COUNTS) {
				chg_err("ibus over than %d, ", chip->pre_pdo_current);
				vote(chip->reverse_pdo_votable, CURR_ERR_VOTER,
					true, chip->over_ibus_pdo_limit, false);/* 5V1.5A */
				vote(chip->high_reverse_disable_votable, CURR_ERR_VOTER, true, 1, false);
				chip->ibus_over = 0;
			}
		} else {
			chip->ibus_over = 0;
		}
	}

	return ret;
}

#define FORCE_EXIT_REVERSE_CHG_VBUS_THR  2000
static void oplus_reverse_vbus_hw_detect_check(struct oplus_chg_reverse *chip)
{
	int vbus_check = 0;
	int hw_detect_check = 0;
	int retry_check = 5; /*retry 5 count*/

	do {
		vbus_check = oplus_wired_get_vbus();
		hw_detect_check = oplus_wired_get_hw_detect();
		if (vbus_check > FORCE_EXIT_REVERSE_CHG_VBUS_THR
			|| hw_detect_check == CC_DETECT_PLUGIN) {
			break;
		}
		msleep(500);
		retry_check--;
		chg_debug("vbus_check[%d] hw_detect_check[%d]\n",
			vbus_check, hw_detect_check);
	} while (retry_check > 0);

	if (retry_check <= 0
		&& vbus_check <= FORCE_EXIT_REVERSE_CHG_VBUS_THR
		&& hw_detect_check != CC_DETECT_PLUGIN) {
		chip->reverse_enable = false;
		schedule_work(&chip->reverse_online_work);
		chg_err("force exit: vbus_check[%d] hw_detect_check[%d]\n",
			vbus_check, hw_detect_check);
	}

	return;
}

static void oplus_reverse_vbus_retention_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip = container_of(dwork,
		struct oplus_chg_reverse, reverse_vbus_retention_check_work);
	int i = 0;
	int retry_check = 5;
	static int vbus_retention_pdo_level;
	static int vbus_down_count = 0;

	for (i = 0; i < retry_check; i++) {
		msleep(500);
		if (chip->reverse_enable)
			break;
	}
	if (chip->reverse_enable && chip->req_volt == PD_VBUS) {
		vbus_down_count++;
		vbus_retention_pdo_level = chip->reverse_vbus_pdo_limit_num - vbus_down_count;
		chg_info("pre_pdo_current[%d], pre_pdo_current[%d], reverse_vbus_pdo_limit[%d]]\n",
			chip->pre_pdo_voltage, chip->pre_pdo_current, vbus_retention_pdo_level);
		if (vbus_down_count < chip->reverse_vbus_pdo_limit_num) {
			for (i = vbus_retention_pdo_level; i > 1; i--) {
				if (chip->reverse_vbus_pdo_limit[vbus_retention_pdo_level].reverse_pdo
					> ((chip->pre_pdo_voltage << 16) | chip->pre_pdo_current)) {
					vbus_retention_pdo_level--;
				} else {
					break;
				}
			}
			if (vbus_retention_pdo_level >= 1)
				vote(chip->reverse_pdo_votable, VBUS_MV_VOTER, true,
					chip->reverse_vbus_pdo_limit[vbus_retention_pdo_level - 1].reverse_pdo, false);
			else
				vote(chip->reverse_pdo_votable, VBUS_MV_VOTER, true,
					chip->reverse_vbus_pdo_limit[0].reverse_pdo, false);
		}
	} else {
		vbus_down_count = 0;
		vote(chip->reverse_pdo_votable, VBUS_MV_VOTER, false, 0, false);
	}
	chg_info("2.5s reverse_enable[%d]\n", chip->reverse_enable);
	return;
}

#define OPLUS_SVID 		    0x22D9
static void oplus_reverse_protection_check(struct oplus_chg_reverse *chip, struct puc_strategy_ret_data *data)
{
	int rc;
	struct mms_msg *msg;
	/* authenticate or hmac vote */
	if (!chip->authenticate || !chip->hmac) {
		vote(chip->reverse_pdo_votable, AUTH_VOTER,
			true, chip->authenticate_pdo_limit, false);/* 5V1.5A */
		vote(chip->high_reverse_disable_votable, AUTH_VOTER, true, 1, false);
	} else {
		vote(chip->high_reverse_disable_votable, AUTH_VOTER, false, 0, false);
		vote(chip->reverse_pdo_votable, AUTH_VOTER, false, 0, false);
	}

	/* Power Saving Mode and Ultra Power Saving Mode vote */
	if (chip->ui_soc < chip->reverse_limit.last_uisoc_limit[LAST_LEVEL_2]) {
		if (chip->ui_soc <= chip->reverse_limit.last_uisoc_limit[LAST_LEVEL_1]) {
			vote(chip->reverse_pdo_votable, CURR_LIMIT_VOTER, true,
				(NORMAL_VBUS << 16 | chip->reverse_limit.last_current[LAST_LEVEL_1]), false);/* 5V 0.5A */
			vote(chip->high_reverse_disable_votable, CURR_LIMIT_VOTER, true, 0, false);
		} else {
			vote(chip->reverse_pdo_votable, CURR_LIMIT_VOTER, true,
				(NORMAL_VBUS << 16 | chip->reverse_limit.last_current[LAST_LEVEL_2]), false);/* 5V1.5A */
			vote(chip->high_reverse_disable_votable, CURR_LIMIT_VOTER, true, 0, false);
		}
	}

	/* led_on vote */
	if (chip->led_on && !get_effective_result(chip->high_reverse_disable_votable)) {
		vote(chip->reverse_pdo_votable, LED_ON_VOTER, true, chip->led_on_pdo_limit, false);/* 9V1A */
	} else {
		vote(chip->reverse_pdo_votable, LED_ON_VOTER, false, 0, false);
	}

	/* SVID vote */
	rc = oplus_chg_ic_func(chip->reverse_ic,
		OPLUS_IC_FUNC_GET_REVERSE_CHG_SVID, &chip->reverse_chg_svid);
	if (rc >= 0) {
		chg_info("get reverse chg svid %04X ,ui_soc %d\n",
			chip->reverse_chg_svid, chip->ui_soc);
		if (chip->sink_svid_support && chip->reverse_chg_svid != OPLUS_SVID &&
			chip->ui_soc < chip->reverse_limit.reverse_uisoc_limit[UISOC_LEVEL_NORMAL]) {
			vote(chip->reverse_pdo_votable, SVID_VOTER, true,
				NORMAL_VBUS << 16 | chip->thired_oplus_svid_limit, false);/* 5V1.5A */
			vote(chip->high_reverse_disable_votable, SVID_VOTER, true, 1, false);
		} else {
			vote(chip->high_reverse_disable_votable, SVID_VOTER, false, 0, false);
			vote(chip->reverse_pdo_votable, SVID_VOTER, false, 0, false);
		}
		if (chip->reverse_chg_svid != chip->oplus_svid) {
			chip->oplus_svid = (int)chip->reverse_chg_svid;
			msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
							REVERSE_ITEM_SINK_OPLUS_SVID);
			if (msg == NULL) {
				chg_err("alloc msg error\n");
				return;
			}
			rc = oplus_mms_publish_msg(chip->reverse_topic, msg);
			if (rc < 0) {
				chg_err("publish reverse chg type msg error, rc=%d\n", rc);
				kfree(msg);
			}
		}
	} else
		chg_err("can't get reverse_chg_svid %d, rc=%d\n", chip->reverse_chg_svid, rc);

	oplus_reverse_check_ibus_curr(chip);
	oplus_reverse_vbus_hw_detect_check(chip);
	schedule_delayed_work(&chip->usbtemp_high_work, 0);
	schedule_work(&chip->reverse_vbus_check_work);
}

#define WAIT_VBUS_READY_TIME 1000
static void oplus_reverse_vbus_check_work(struct work_struct *work)
{
	struct oplus_chg_reverse *chip =
		container_of(work, struct oplus_chg_reverse, reverse_vbus_check_work);
	int ret;
	int reverse_vbus_mv;

	if (chip->vbus_level == 0 ||
		chip->reverse_vbus_pdo_limit_num != chip->vbus_limit_count + 1) {
		chg_err("not support reverse vbus check \r\n");
		return;
	}

	ret = oplus_reverse_get_boost_vol(chip, &reverse_vbus_mv);
	if (ret < 0) {
		chg_err("oplus_reverse_check_vbus_error \r\n");
		return;
	}
	if (chip->vbus_level > 0 &&
		chip->pre_pdo_voltage > NORMAL_VBUS && reverse_vbus_mv > HIGH_VBUS_START &&
		reverse_vbus_mv <
		chip->reverse_limit.reverse_vbus_limit[chip->vbus_level - 1]) {
		do {
			chg_info("reverse_vbus_mv =%d, reverse_vbus_limit=%d\r\n", reverse_vbus_mv,
			chip->reverse_limit.reverse_vbus_limit[chip->vbus_level - 1]);
			if (reverse_vbus_mv <
				chip->reverse_limit.reverse_vbus_limit[chip->vbus_level - 1]) {
				vote(chip->reverse_pdo_votable, VOL_DIFF_VOTER, true,
					chip->reverse_vbus_pdo_limit[chip->vbus_level - 1].reverse_pdo, false);
				mutex_lock(&chip->set_pdo_lock);
				oplus_reverse_set_pdo(chip, chip->target_cap_pdo0, chip->target_pdo);
				mutex_unlock(&chip->set_pdo_lock);
			} else {
				break;
			}
			msleep(WAIT_VBUS_READY_TIME);
			oplus_reverse_get_boost_vol(chip, &reverse_vbus_mv);
			chip->vbus_level--;
		} while (chip->vbus_level >= 1);
	}
	return;
}

#define WAIT_VBAT_READY_TIME 1000
static void oplus_reverse_vbat_check(struct oplus_chg_reverse *chip)
{
	union mms_msg_data data = { 0 };

	if (chip->vbat_level == 0 ||
		chip->normal_reverse_count != (chip->vbat_limit_count + 1)) {
		chg_info("not support reverse vbat check\r\n");
		return;
	}
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	chip->vbat_mv = data.intval;

	chg_info("vbat_level =%d, is_support_high_reverse=%d, vbat_mv=%d\r\n",
		chip->vbat_level, chip->is_support_high_reverse, chip->vbat_mv);
	if (chip->vbat_level > 0 &&
		chip->vbat_mv < chip->reverse_limit.reverse_vbat_limit[chip->vbat_level - 1]) {
		do {
			chg_info("reverse_vbat_mv =%d, reverse_vbat_limit=%d, pre_pdo_voltage=%d\r\n", chip->vbat_mv,
			chip->reverse_limit.reverse_vbat_limit[chip->vbat_level - 1], chip->pre_pdo_voltage);
			if (chip->vbat_mv < chip->reverse_limit.reverse_vbat_limit[chip->vbat_level - 1]) {
				if (chip->pre_pdo_voltage == NORMAL_VBUS || !chip->is_support_high_reverse) {
					vote(chip->reverse_pdo_votable, BATT_VOL_VOTER, true,
					(NORMAL_VBUS << 16 |
					chip->reverse_limit.normal_reverse_current[chip->vbat_level - 1]),
					false);
				} else if (chip->pre_pdo_voltage > NORMAL_VBUS || chip->pre_pdo_voltage == 0) {
					vote(chip->cap_pdo0_votable, BATT_VOL_VOTER, true,
					(NORMAL_VBUS << 16 |
					chip->reverse_limit.normal_reverse_current[chip->vbat_level - 1]),
					false);
				}
				mutex_lock(&chip->set_pdo_lock);
				oplus_reverse_set_pdo(chip, chip->target_cap_pdo0, chip->target_pdo);
				mutex_unlock(&chip->set_pdo_lock);
			} else {
				break;
			}
			msleep(WAIT_VBAT_READY_TIME);
			oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
			chip->vbat_mv = data.intval;
			chip->vbat_level--;
		} while (chip->vbat_level >= 1);
	} else if (chip->vbat_level > 0 && chip->pre_pdo_voltage > NORMAL_VBUS &&
			chip->vbat_mv >= chip->reverse_limit.reverse_vbat_limit[chip->vbat_limit_count - 1]) {
			vote(chip->cap_pdo0_votable, BATT_CURR_VOTER, true,
				(NORMAL_VBUS << 16 |
				chip->reverse_limit.normal_reverse_current[chip->normal_reverse_count - 1]),
				false);
			mutex_lock(&chip->set_pdo_lock);
			oplus_reverse_set_pdo(chip, chip->target_cap_pdo0, chip->target_pdo);
			mutex_unlock(&chip->set_pdo_lock);
	} else {
		mutex_lock(&chip->set_pdo_lock);
		oplus_reverse_set_pdo(chip, chip->target_cap_pdo0, chip->target_pdo);
		mutex_unlock(&chip->set_pdo_lock);
	}
	return;
}

#define USER_CONTROL_MAX_LEVEL 6 /* 30S */
static void oplus_reverse_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip =
		container_of(dwork, struct oplus_chg_reverse, reverse_monitor_work);
	struct puc_strategy_ret_data data;
	static int user_voter_control;
	int rc;
	int range_switch_dealy = HRP_TEMP_SWITCH_DELAY;

	if (!chip->high_reverse_charging) {
		rc = oplus_reverse_charge_start(chip);
		if (rc < 0)
			return;
	}

	oplus_reverse_handle_level_for_drop_down();
	oplus_reverse_protection_check(chip, &data);

	if (!chip->reverse_enable) {
		chg_info(" exit reverse");
		oplus_reverse_force_exit(chip);
	}
	oplus_reverse_charge_allow_check(chip);
	/* ui_soc vote*/
	oplus_reverse_get_uisoc_curr(chip);
	/* temp vote*/
	rc = oplus_reverse_update_batt_temp_curr(chip);
	if (rc < 0)
		goto exit;
	oplus_update_pdo_limit(chip);

	if (chip->req_volt == PD_VBUS)
		schedule_delayed_work(&chip->reverse_error_flag_work, 0);

	schedule_delayed_work(&chip->reverse_chg_type_work, 0);

	/* AT vote*/
	if (get_client_vote(chip->reverse_pdo_votable, USER_VOTER) > 0) {
		user_voter_control++;
		if (user_voter_control >= USER_CONTROL_MAX_LEVEL) {
			user_voter_control = 0;
			vote(chip->reverse_pdo_votable, USER_VOTER, false, 0, false);
		}
	}

	if (chip->vbat_level != 0 &&
		chip->normal_reverse_count == (chip->vbat_limit_count + 1)) {
		oplus_reverse_vbat_check(chip);
	} else {
		mutex_lock(&chip->set_pdo_lock);
		oplus_reverse_set_pdo(chip, chip->target_cap_pdo0, chip->target_pdo);
		mutex_unlock(&chip->set_pdo_lock);
	}

	chg_info("oplus reverse monitor work\n");
	schedule_delayed_work(&chip->reverse_monitor_work, msecs_to_jiffies(REVERSE_ENABLE_DELAY_MS));
	return;
exit:
	schedule_delayed_work(&chip->reverse_monitor_work, msecs_to_jiffies(range_switch_dealy));
	return;
}
static int oplus_high_reverse_charging_status(struct oplus_mms *mms,
					    union mms_msg_data *data)
{
	struct oplus_chg_reverse *chip;

	if (mms == NULL) {
		chg_err("topic is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	if (!chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	data->intval = chip->high_reverse_charging;
	return 0;
}

static void oplus_reverse_awake_init(struct oplus_chg_reverse *chip)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	wake_lock_init(&chip->reverse_wake_lock, WAKE_LOCK_SUSPEND,
			"reverse_wake_lock");
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 102) &&                      \
		LINUX_VERSION_CODE > KERNEL_VERSION(4, 14, 999))
	chip->reverse_ws = wakeup_source_register("reverse_wake_lock");
#else
	chip->reverse_ws = wakeup_source_register(NULL, "reverse_wake_lock");
#endif
}

static void oplus_reverse_awake_exit(struct oplus_chg_reverse *chip)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	wake_lock_destroy(&chip->reverse_wake_lock);
#else
	wakeup_source_unregister(chip->reverse_ws);
#endif
}

static void oplus_reverse_set_awake(struct oplus_chg_reverse *chip, bool awake)
{
	static bool pm_flag = false;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	if (awake && !pm_flag) {
		pm_flag = true;
		wake_lock(&chip->reverse_wake_lock);
	} else if (!awake && pm_flag) {
		wake_unlock(&chip->reverse_wake_lock);
		pm_flag = false;
	}
#else
	if (!chip || !chip->reverse_ws) {
		return;
	}
	if (awake && !pm_flag) {
		pm_flag = true;
		__pm_stay_awake(chip->reverse_ws);
	} else if (!awake && pm_flag) {
		__pm_relax(chip->reverse_ws);
		pm_flag = false;
	}
#endif
}

static int oplus_chg_reverse_watchdog_kthread(void *arg)
{
	struct oplus_chg_reverse *chip = (struct oplus_chg_reverse *)arg;
	int rc;

	while (!kthread_should_stop()) {
		wait_event_interruptible(chip->reverse_watchdog_wq, chip->kthread_reverse_enable);
		oplus_reverse_set_awake(chip, true);
		if (chip->use_cp_reverse && chip->cp_ic && chip->high_reverse_enable) {
			rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_WATCHDOG_RESET);
			if (rc < 0 && rc != -ENOTSUPP) {
				oplus_reverse_set_awake(chip, false);
				chg_err("WATCHDOG_RESET error");
				return rc;
			}
		}
		schedule_timeout_interruptible(round_jiffies_relative(msecs_to_jiffies(chip->reset_watchdog_time)));
		oplus_reverse_set_awake(chip, false);
	}
	return 0;
}

static void oplus_chg_reverse_watchdog_init(struct oplus_chg_reverse *chip)
{
	int rc;

	init_waitqueue_head(&chip->reverse_watchdog_wq);

	chip->reverse_watchdog = kthread_run(
		oplus_chg_reverse_watchdog_kthread, chip, "reverse_watchdog");
	if (!IS_ERR(chip->reverse_watchdog)) {
		chg_info("reverse_watchdog kthread creat success\n");
	} else {
		rc = PTR_ERR(chip->reverse_watchdog);
		chg_err("reverse_watchdog creat fail, rc=%d\n", rc);
	}
	return;
}

static int oplus_high_reverse_chg_enable_status(struct oplus_mms *mms,
						    union mms_msg_data *data)
{
	struct oplus_chg_reverse *chip;
	bool enable = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	enable = chip->high_reverse_enable;
	if (chip->use_high_reverse_awake) {
		chip->kthread_reverse_enable = chip->high_reverse_enable;
		if (enable)
			wake_up_interruptible(&chip->reverse_watchdog_wq);
	}
	data->intval = enable;
	return 0;
}

static int oplus_reverse_chg_type_update(struct oplus_mms *mms,
						    union mms_msg_data *data)
{
	struct oplus_chg_reverse *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->reverse_chg_type;
	return 0;
}

static int oplus_reverse_oplus_svid_update(struct oplus_mms *mms,
						    union mms_msg_data *data)
{
	struct oplus_chg_reverse *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->oplus_svid;
	return 0;
}

static int oplus_reverse_high_reverse_err(struct oplus_mms *mms,
						    union mms_msg_data *data)
{
	struct oplus_chg_reverse *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->high_reverse_err_flag;
	return 0;
}

static int oplus_high_reverse_chg_count(struct oplus_mms *mms,
						    union mms_msg_data *data)
{
	struct oplus_chg_reverse *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->high_reverse_count;
	return 0;
}
static int oplus_last_high_reverse_pdo(struct oplus_mms *mms,
						    union mms_msg_data *data)
{
	struct oplus_chg_reverse *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	data->intval = chip->last_high_reverse_pdo;
	return 0;
}

static void oplus_reverse_chg_online_handler(struct oplus_chg_ic_dev *ic_dev,
					   void *virq_data)
{
	chg_info("%s online\n", ic_dev->manu_name);
}

static void oplus_high_reverse_chg_offline_handler(struct oplus_chg_ic_dev *ic_dev,
					   void *virq_data)
{
	chg_info("%s offline\n", ic_dev->manu_name);
}

static void oplus_high_reverse_chg_err_handler(struct oplus_chg_ic_dev *ic_dev,
					   void *virq_data)
{
	chg_info("%s err\n", ic_dev->manu_name);
}

static void oplus_reverse_chg_enable_handler(struct oplus_chg_ic_dev *ic_dev,
					   void *virq_data)
{
	struct oplus_chg_reverse *chip = virq_data;
	int rc;
	bool enable;

	rc = oplus_chg_ic_func(chip->reverse_ic,
			       OPLUS_IC_FUNC_GET_REVERSE_ENABLE, &enable);
	if (rc < 0)
		enable = false;
	chip->reverse_enable = enable;
	chg_info("reverse_enable = %d\n", enable);
	schedule_work(&chip->reverse_online_work);
}

static void oplus_high_reverse_chg_enable_handler(struct oplus_chg_ic_dev *ic_dev,
					   void *virq_data)
{
	struct oplus_chg_reverse *chip = virq_data;
	int rc;
	struct mms_msg *msg;
	bool enable;

	rc = oplus_chg_ic_func(chip->reverse_ic,
			       OPLUS_IC_FUNC_GET_HIGH_REVERSE_ENABLE, &enable);
	if (rc < 0)
		enable = false;
	chip->high_reverse_enable = enable;
	if (enable)
		chip->high_reverse_count++;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
					REVERSE_ITEM_HIGH_REVERSE_CHG_ENABLE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->reverse_topic, msg);
	if (rc < 0) {
		chg_err("publish high reverse enable msg error, rc=%d\n", rc);
		kfree(msg);
	}
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
					REVERSE_ITEM_HIGH_REVERSE_CHG_COUNT);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->reverse_topic, msg);
	if (rc < 0) {
		chg_err("publish high reverse count msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_reverse_hard_reset_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip = container_of(dwork,
		struct oplus_chg_reverse, reverse_hard_reset_work);
	static int hard_reset_pdo_level;
	int i;

	if (chip->hardreset_limit_count == 0) {
		chg_err("not support hardreset limit\n");
		return;
	}
	if (chip->req_volt != PD_VBUS) {
		chg_err("sink request not 9v unsupport hardreset limit\n");
		return;
	}
	if (!chip->reverse_enable)
		msleep(1000); /*wait reverse enable*/
	if (chip->reverse_enable) {
		chip->hard_reset_count++;
		hard_reset_pdo_level = chip->hardreset_limit_count - chip->hard_reset_count;
		chg_info("hard_reset_count = %d, pre_pdo_voltage = %d, pre_pdo_current = %d\n",
			chip->hard_reset_count, chip->pre_pdo_voltage, chip->pre_pdo_current);
		if (chip->hard_reset_count < chip->hardreset_limit_count && chip->pre_pdo_voltage == PD_VBUS) {
			for (i = hard_reset_pdo_level; i > 1; i--) {
				if (chip->reverse_hardreset_pdo_limit[hard_reset_pdo_level].reverse_pdo
					> ((chip->pre_pdo_voltage << 16) | chip->pre_pdo_current))
					hard_reset_pdo_level--;
				else
					break;
			}
			if (hard_reset_pdo_level >= 1)
				vote(chip->reverse_pdo_votable, HW_ERR_VOTER, true,
					chip->reverse_hardreset_pdo_limit[hard_reset_pdo_level - 1].reverse_pdo, false);
			else
				vote(chip->reverse_pdo_votable, HW_ERR_VOTER, true,
					chip->reverse_hardreset_pdo_limit[0].reverse_pdo, false);
		}
	}
	if (chip->hard_reset_count >= chip->hardreset_limit_count) {
		vote(chip->reverse_pdo_votable, HW_ERR_VOTER, true,
			(NORMAL_VBUS << 16 | NORMAL_IBUS), false);/* 5V1A */
		vote(chip->high_reverse_disable_votable, HW_ERR_VOTER, true, 1, false);
		chip->hard_reset_count = 0;
	}
}

static void oplus_reverse_hard_reset_handler(struct oplus_chg_ic_dev *ic_dev,
					   void *virq_data)
{
	struct oplus_chg_reverse *chip = virq_data;
	schedule_delayed_work(&chip->reverse_hard_reset_work, 0);
}

static void oplus_reverse_sink_req_msg_handler(struct oplus_chg_ic_dev *ic_dev,
					   void *virq_data)
{
	struct oplus_chg_reverse *chip = virq_data;
	struct mms_msg *msg;
	int msg_type = REVERSE_CHG_MSG_TYPE_UNKNOWN;
	int req_voltage = 0;
	int req_current = 0;
	int rc;

	rc = oplus_chg_ic_func(chip->reverse_ic,
			       OPLUS_IC_FUNC_GET_RVS_CHG_MSG, &msg_type);
	if (rc < 0) {
		msg_type = REVERSE_CHG_MSG_TYPE_UNKNOWN;
		return;
	}
	chg_info("sink req msg handle, msg_type[%d]\n", msg_type);

	switch (msg_type) {
	case REVERSE_CHG_MSG_TYPE_SINK_REQ_PDO:
		rc = oplus_chg_ic_func(chip->reverse_ic,
			       OPLUS_IC_FUNC_GET_SINK_REQ_PDO, &req_voltage, &req_current);
		if (rc >= 0) {
			chip->req_volt = req_voltage;
			chip->req_current = req_current;
		}

		if (req_voltage >= 5000 && req_current > 0) {
			msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
						REVERSE_ITEM_SINK_REQUEST_VOLT, chip->req_volt);
			if (msg == NULL) {
				chg_err("alloc msg error\n");
				return;
			}
			rc = oplus_mms_publish_msg(chip->reverse_topic, msg);
			if (rc < 0) {
				chg_err("publish high reverse charging msg error, rc=%d\n", rc);
				kfree(msg);
				return;
			}
		}
		break;
	default:
		chg_info("sink req unknown\n");
		break;
	}

	return;
}

static void oplus_reverse_topic_update(struct oplus_mms *mms, bool publish)
{
}

static struct mms_item oplus_reverse_item[] = {
	{
		.desc = {
			.item_id = HIGH_REVERSE_ITEM_STATUS,
			.update = oplus_high_reverse_charging_status,
		}
	},
	{
		.desc = {
			.item_id = REVERSE_ITEM_HIGH_REVERSE_CHG_ENABLE,
			.update = oplus_high_reverse_chg_enable_status,
		}
	},
	{
		.desc = {
			.item_id = REVERSE_ITEM_REVERSE_CHG_TYPE,
			.update = oplus_reverse_chg_type_update,
		}
	},
	{
		.desc = {
			.item_id = REVERSE_ITEM_SINK_OPLUS_SVID,
			.update = oplus_reverse_oplus_svid_update,
		}
	},
	{
		.desc = {
			.item_id = REVERSE_ITEM_HIGH_REVERSE_ERR,
			.update = oplus_reverse_high_reverse_err,
		}
	},
	{
		.desc = {
			.item_id = REVERSE_ITEM_HIGH_REVERSE_CHG_COUNT,
			.update = oplus_high_reverse_chg_count,
		}
	},
	{
		.desc = {
			.item_id = REVERSE_ITEM_LAST_HIGH_REVERSE_PDO,
			.update = oplus_last_high_reverse_pdo,
		}
	},
	{
		.desc = {
			.item_id = REVERSE_ITEM_SINK_REQUEST_VOLT,
			.update = NULL,
		}
	}
};

static const struct oplus_mms_desc oplus_reverse_desc = {
	.name = "reverse",
	.type = OPLUS_MMS_TYPE_REVERSE,
	.item_table = oplus_reverse_item,
	.item_num = ARRAY_SIZE(oplus_reverse_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_reverse_topic_update,
};

static void oplus_reverse_pdo_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip =
		container_of(dwork, struct oplus_chg_reverse, reverse_pdo_update_work);

	if (chip->led_on && !get_effective_result(chip->high_reverse_disable_votable))
		vote(chip->reverse_pdo_votable, LED_ON_VOTER, true, chip->led_on_pdo_limit, false);/* 9V1A */
	else
		vote(chip->reverse_pdo_votable, LED_ON_VOTER, false, 0, false);

	mutex_lock(&chip->set_pdo_lock);
	oplus_reverse_set_pdo(chip, chip->target_cap_pdo0, chip->target_pdo);
	mutex_unlock(&chip->set_pdo_lock);
}

static void oplus_reverse_comm_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_reverse *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_UI_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->ui_soc = data.intval;
			chg_info("ui_soc =%d \r\n", chip->ui_soc);
			break;
		case COMM_ITEM_SHELL_TEMP:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->shell_temp = data.intval;
			chg_info("shell_temp =%d \r\n", chip->shell_temp);
			break;
		case COMM_ITEM_LED_ON:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->led_on = data.intval;
			if (chip->reverse_enable)
				schedule_delayed_work(&chip->reverse_pdo_update_work, 0);
			chg_info("led_on =%d \r\n", chip->led_on);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_reverse_subscribe_comm_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_chg_reverse *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->comm_topic = topic;
	chip->comm_subs =
		oplus_mms_subscribe(topic, chip,
				    oplus_reverse_comm_subs_callback, "reverse");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe comm topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data, false);
	chip->ui_soc = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data, false);
	chip->shell_temp = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_LED_ON, &data, false);
	chip->led_on = data.intval;

	chg_info("ui_soc =%d shell_temp =%d led_on =%d \r\n", chip->ui_soc, chip->shell_temp, chip->led_on);
}

static bool is_sub_gauge_topic_available(struct oplus_chg_reverse *chip)
{
	if (!chip->sub_gauge_topic)
		chip->sub_gauge_topic = oplus_mms_get_by_name("gauge:1");
	return !!chip->sub_gauge_topic;
}

#define USB_54C 54
#define USB_57C 57
#define USB_48C 48
#define USB_12C 12
#define USB_6C 6
#define USB_MAX 100
static void oplus_usbtemp_high_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip =
		container_of(dwork, struct oplus_chg_reverse, usbtemp_high_work);
	int rc;
	union mms_msg_data data = { 0 };
	int usb_temp_l;
	int usb_temp_r;
	int usbtemp_volt_l;
	int usbtemp_volt_r;

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_USB_STATUS, &data, false);

	oplus_wired_get_usb_temp_volt(&usbtemp_volt_l, &usbtemp_volt_r);
	oplus_wired_get_usb_temp(&usb_temp_l, &usb_temp_r);
	chg_info("temp_l =%d temp_r =%d batt_temp =%d volt_l[%d] volt_r[%d] [%d]\r\n",
		usb_temp_l, usb_temp_r, chip->batt_realy_temp / 10,
		usbtemp_volt_l, usbtemp_volt_r, data.intval);
	if (usb_temp_l >= USB_54C || usb_temp_r >= USB_54C
		|| usb_temp_l - chip->batt_realy_temp / 10 >= USB_12C
		|| usb_temp_r - chip->batt_realy_temp / 10 >= USB_12C) {
		vote(chip->reverse_pdo_votable, USB_VOTER, true,
				(PD_VBUS << 16 | NORMAL_IBUS), false);
	} else if (usb_temp_l <= USB_48C && usb_temp_r <= USB_48C
		&& usb_temp_l - chip->batt_realy_temp / 10 <= USB_6C
		&& usb_temp_r - chip->batt_realy_temp / 10 <= USB_6C) {
		vote(chip->reverse_pdo_votable, USB_VOTER, false, 0, false);
	}
	if ((data.intval & USB_TEMP_HIGH) == USB_TEMP_HIGH) {
		chg_err("usbtemp protect triged true\n");
		rc = oplus_chg_ic_func(g_rvs_chg_chip->reverse_ic,
					OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN, 0);
		vote(chip->reverse_pdo_votable, USB_VOTER, true,
				(NORMAL_VBUS << 16 | NORMAL_IBUS), false);
		vote(chip->high_reverse_disable_votable, USB_VOTER, true, 1, false);
		msleep(WAIT_HIGH_REVERSE_COLSE);
		oplus_wired_set_dischg_status(true);
	}
}

static void oplus_reverse_wired_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_reverse *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_USB_STATUS:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			if (((data.intval & USB_TEMP_HIGH) == USB_TEMP_HIGH) &&
				chip->reverse_enable) {
				schedule_delayed_work(&chip->usbtemp_high_work, 0);
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

static void oplus_reverse_subscribe_wired_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_chg_reverse *chip = prv_data;

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_reverse_wired_subs_callback, "reverse");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}
}

static void oplus_reverse_gauge_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_reverse *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_TIMER:
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX,
					&data, false);
		chip->vbat_mv = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP,
					&data, false);
		chip->batt_temp = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR,
					&data, false);
		chip->ibat_ma = data.intval;
		if (is_sub_gauge_topic_available(chip)) {
			oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_VOL_MAX,
						&data, false);
			chip->vbat_min_mv = data.intval;
		} else {
			oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN,
						&data, false);
			chip->vbat_min_mv = data.intval;
		}
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC,
					&data, false);
		chip->batt_soc = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REAL_TEMP,
				&data, false);
		chip->batt_realy_temp = data.intval;
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case GAUGE_ITEM_AUTH:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
				&data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_AUTH data, rc=%d\n", rc);
			} else {
				chip->authenticate = !!data.intval;
			}
			break;
		case GAUGE_ITEM_HMAC:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
								&data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_HMAC data, rc=%d\n",
					rc);
			} else {
				chip->hmac = !!data.intval;
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

static void oplus_reverse_subscribe_gauge_topic(struct oplus_mms *topic,
						  void *prv_data)
{
	struct oplus_chg_reverse *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->gauge_topic = topic;
	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
					oplus_reverse_gauge_subs_callback, "reverse");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data,
				true);
	chip->vbat_mv = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data,
				true);
	chip->batt_temp = data.intval;
	if (is_sub_gauge_topic_available(chip)) {
		oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_VOL_MAX,
					&data, false);
		chip->vbat_min_mv = data.intval;
	} else {
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data,
					true);
		chip->vbat_min_mv = data.intval;
	}
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data,
				true);
	chip->batt_soc = data.intval;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_HMAC, &data,
				     true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_HMAC data, rc=%d\n", rc);
		chip->hmac = false;
	} else {
		chip->hmac = !!data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_AUTH, &data,
				true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_AUTH data, rc=%d\n", rc);
		chip->authenticate = false;
	} else {
		chip->authenticate = !!data.intval;
	}
	chg_info("hmac=%d, authenticate=%d\n", chip->hmac, chip->authenticate);

	chg_info(" vbat_mv =%d vbat_min_mv =%d batt_temp =%d batt_soc =%d \r\n",
	chip->vbat_mv, chip->vbat_min_mv, chip->batt_temp, chip->batt_soc);
}

static int oplus_reverse_topic_init(struct oplus_chg_reverse *chip)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;

	mms_cfg.drv_data = chip;
	mms_cfg.of_node = chip->dev->of_node;

	chip->reverse_topic =
		devm_oplus_mms_register(chip->dev, &oplus_reverse_desc, &mms_cfg);
	if (IS_ERR(chip->reverse_topic)) {
		chg_info("Couldn't register reverse topic\n");
		rc = PTR_ERR(chip->reverse_topic);
		return rc;
	}

	oplus_mms_wait_topic("wired", oplus_reverse_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("common", oplus_reverse_subscribe_comm_topic, chip);
	oplus_mms_wait_topic("gauge", oplus_reverse_subscribe_gauge_topic, chip);
	return 0;
}

static int oplus_high_reverse_disable_vote_callback(struct votable *votable, void *data,
					    int disable, const char *client,
					    bool step)
{
	struct oplus_chg_reverse *chip = data;
	static bool high_reverse_disable;

	if (disable < 0)
		high_reverse_disable = false;
	else
		high_reverse_disable = !!disable;
	chg_info("%s set high reverse disable to %s\n", client, high_reverse_disable ? "true" : "false");
	if (high_reverse_disable && chip->req_volt == PD_VBUS)
		schedule_delayed_work(&chip->reverse_error_flag_work, 0);

	return 0;
}

static int oplus_reverse_pdo_vote_callback(struct votable *votable, void *data,
					 int pdo, const char *client,
					 bool step)
{
	struct oplus_chg_reverse *chip = data;

	if (pdo < 0)
		return -EINVAL;

	chip->target_pdo = pdo;
	chg_info("%s set reverse_pdo volt:%d  curr:%d mA\n",
		client, (pdo & 0xFFFF0000) >> 16, (pdo & 0xFFFF));

	return 0;
}

static int oplus_reverse_cap_pdo0_vote_callback(struct votable *votable, void *data,
					 int pdo, const char *client,
					 bool step)
{
	struct oplus_chg_reverse *chip = data;

	if (pdo < 0)
		return -EINVAL;

	chip->target_cap_pdo0 = pdo;
	chg_info("%s set reverse_cap_pdo0 volt:%d  curr:%d mA\n",
		client, (pdo & 0xFFFF0000) >> 16, (pdo & 0xFFFF));

	return 0;
}

static int oplus_reverse_vote_init(struct oplus_chg_reverse *chip)
{
	int rc;

	chip->reverse_pdo_votable = create_votable("REVERSE_PDO", VOTE_MIN,
					   oplus_reverse_pdo_vote_callback, chip);
	if (IS_ERR(chip->reverse_pdo_votable)) {
		rc = PTR_ERR(chip->reverse_pdo_votable);
		chg_err("creat reverse_pdo_votable error, rc=%d\n", rc);
		chip->reverse_pdo_votable = NULL;
		return rc;
	}

	chip->high_reverse_disable_votable =
		create_votable("HIGH_REVERSE_DISABLE", VOTE_SET_ANY, oplus_high_reverse_disable_vote_callback, chip);
	if (IS_ERR(chip->high_reverse_disable_votable)) {
		rc = PTR_ERR(chip->high_reverse_disable_votable);
		chg_err("creat high_reverse_disable_votable error, rc=%d\n", rc);
		chip->high_reverse_disable_votable = NULL;
		goto creat_disable_votable_err;
	}

	chip->cap_pdo0_votable = create_votable("REVERSE_CAP_PDO0", VOTE_MIN,
					   oplus_reverse_cap_pdo0_vote_callback, chip);
	if (IS_ERR(chip->cap_pdo0_votable)) {
		rc = PTR_ERR(chip->cap_pdo0_votable);
		chg_err("creat cap_pdo0_votable error, rc=%d\n", rc);
		chip->cap_pdo0_votable = NULL;
		goto creat_cap_pdo0_votable_err;
	}

	vote(chip->high_reverse_disable_votable, BATT_TEMP_VOTER, false, 0, false);
	vote(chip->high_reverse_disable_votable, BATT_SOC_VOTER, false, 0, false);
	return 0;
creat_cap_pdo0_votable_err:
	destroy_votable(chip->high_reverse_disable_votable);
creat_disable_votable_err:
	destroy_votable(chip->reverse_pdo_votable);
	return rc;
}

static void oplus_reverse_buck_ic_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_chg_reverse *chip;

	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	chip = data;

	chip->buck_ic = ic;
	if (!chip->buck_ic)
		chg_err("get buck_ic fail\n");
}

static void oplus_reverse_cp_ic_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_chg_reverse *chip;
	const char *name;

	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	chip = data;

	chip->cp_ic = ic;
	if (!chip->cp_ic)
		chg_err("get reverse_ic fail\n");

	name = of_get_oplus_chg_ic_name(chip->dev->of_node, "oplus,buck_ic", 0);
	oplus_chg_ic_wait_ic(name, oplus_reverse_buck_ic_reg_callback, chip);
}

static int oplus_reverse_chg_virq_register(struct oplus_chg_reverse *chip)
{
	int rc;

	rc = oplus_chg_ic_virq_register(chip->reverse_ic, OPLUS_IC_VIRQ_ONLINE,
		oplus_reverse_chg_online_handler, chip);
	if (rc < 0)
		chg_info("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->reverse_ic, OPLUS_IC_VIRQ_OFFLINE,
		oplus_high_reverse_chg_offline_handler, chip);
	if (rc < 0)
		chg_info("register OPLUS_IC_VIRQ_OFFLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->reverse_ic, OPLUS_IC_VIRQ_ERR,
		oplus_high_reverse_chg_err_handler, chip);
	if (rc < 0)
		chg_info("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->reverse_ic, OPLUS_IC_VIRQ_REVERSE_ENABLE,
		oplus_reverse_chg_enable_handler, chip);
	if (rc < 0)
		chg_info("register OPLUS_IC_VIRQ_REVERSE_ENABLE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->reverse_ic, OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE,
		oplus_high_reverse_chg_enable_handler, chip);
	if (rc < 0)
		chg_info("register OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->reverse_ic, OPLUS_IC_VIRQ_HARD_RESET,
		oplus_reverse_hard_reset_handler, chip);
	if (rc < 0)
		chg_info("register OPLUS_IC_VIRQ_HARD_RESET error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->reverse_ic, OPLUS_IC_VIRQ_SINK_REQ_MSG,
		oplus_reverse_sink_req_msg_handler, chip);
	if (rc < 0)
		chg_info("register OPLUS_IC_VIRQ_SINK_REQ_MSG error, rc=%d", rc);
	return rc;
}

static void oplus_reverse_reverse_ic_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_chg_reverse *chip;
	const char *name;
	int rc;

	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	chip = data;

	chip->reverse_ic = ic;
	if (!chip->reverse_ic)
		chg_err("get reverse_ic fail\n");

	rc = oplus_reverse_chg_virq_register(chip);
	if (rc < 0) {
		chg_err("oplus_reverse_chg_virq_register error, rc=%d\n", rc);
		return;
	}

	name = of_get_oplus_chg_ic_name(chip->dev->of_node, "oplus,cp_ic", 0);
	oplus_chg_ic_wait_ic(name, oplus_reverse_cp_ic_reg_callback, chip);
}

static void oplus_reverse_chg_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_reverse *chip = container_of(dwork,
		struct oplus_chg_reverse, reverse_chg_init_work);
	struct device_node *node = chip->dev->of_node;
	const char *name;

	name = of_get_oplus_chg_ic_name(node, "oplus,reverse_ic", 0);
	oplus_chg_ic_wait_ic(name, oplus_reverse_reverse_ic_reg_callback, chip);
}

static int oplus_chg_reverse_probe(struct platform_device *pdev)
{
	struct oplus_chg_reverse *chip;
	int rc;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_chg_reverse), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc oplus_chg_reverse struct buffer error\n");
		return -ENOMEM;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	rc = oplus_reverse_topic_init(chip);
	if (rc < 0)
		goto topic_reg_err;
	g_rvs_chg_chip = chip;

	rc = oplus_reverse_parse_dt(chip);
	if (rc < 0)
		goto parse_dt_err;
	oplus_reverse_awake_init(chip);
	chip->vbus_level = chip->vbus_limit_count;
	chip->vbat_level = chip->vbat_limit_count;
	INIT_DELAYED_WORK(&chip->reverse_chg_init_work,
			  oplus_reverse_chg_init_work);
	INIT_DELAYED_WORK(&chip->reverse_monitor_work, oplus_reverse_monitor_work);
	INIT_DELAYED_WORK(&chip->usbtemp_high_work, oplus_usbtemp_high_work);
	INIT_DELAYED_WORK(&chip->reverse_pdo_update_work, oplus_reverse_pdo_update_work);
	INIT_DELAYED_WORK(&chip->reverse_chg_type_work,
			  oplus_reverse_chg_type_work);
	INIT_DELAYED_WORK(&chip->reverse_error_flag_work,
	oplus_reverse_error_flag_work);
	INIT_DELAYED_WORK(&chip->normal_reverse_set_hw_ocp_work,
	oplus_normal_reverse_set_hw_ocp_work);
	INIT_DELAYED_WORK(&chip->reverse_vbus_retention_check_work,
	oplus_reverse_vbus_retention_check_work);
	INIT_DELAYED_WORK(&chip->reverse_chg_test_mode_work,
			  oplus_reverse_chg_test_mode_work);
	INIT_DELAYED_WORK(&chip->reverse_clear_keep_status_work,
			  oplus_reverse_clear_keep_status_work);
	INIT_DELAYED_WORK(&chip->reverse_hard_reset_work,
			  oplus_reverse_hard_reset_work);
	INIT_WORK(&chip->reverse_online_work, oplus_reverse_online_work);
	INIT_WORK(&chip->reverse_vbus_check_work,
			  oplus_reverse_vbus_check_work);
	mutex_init(&chip->set_pdo_lock);
	oplus_reverse_vote_init(chip);
	schedule_delayed_work(&chip->reverse_chg_init_work, 0);
	oplus_chg_reverse_watchdog_init(chip);
	chg_info("probe success\n");
	return 0;

topic_reg_err:
parse_dt_err:
	devm_kfree(&pdev->dev, chip);
	g_rvs_chg_chip = NULL;
	chg_err("probe fail\n");
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_chg_reverse_remove(struct platform_device *pdev)
#else
static int oplus_chg_reverse_remove(struct platform_device *pdev)
#endif
{
	struct oplus_chg_reverse *chip = platform_get_drvdata(pdev);

	if (!IS_ERR_OR_NULL(chip->comm_subs))
		oplus_mms_unsubscribe(chip->comm_subs);
	if (!IS_ERR_OR_NULL(chip->wired_subs))
		oplus_mms_unsubscribe(chip->wired_subs);
	if (!IS_ERR_OR_NULL(chip->reverse_subs))
		oplus_mms_unsubscribe(chip->reverse_subs);
	if (!IS_ERR_OR_NULL(chip->gauge_subs))
		oplus_mms_unsubscribe(chip->gauge_subs);
	if (chip->reverse_pdo_votable != NULL)
		destroy_votable(chip->reverse_pdo_votable);
	if (chip->high_reverse_disable_votable != NULL)
		destroy_votable(chip->high_reverse_disable_votable);
	oplus_reverse_awake_exit(chip);
	kthread_stop(chip->reverse_watchdog);
	devm_kfree(&pdev->dev, chip);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_chg_reverse_match[] = {
	{.compatible = "oplus,reverse_charge"},
	{},
};

static struct platform_driver oplus_chg_reverse_driver = {
	.driver = {
		.name = "oplus-reverse_charge",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_chg_reverse_match),
	},
	.probe = oplus_chg_reverse_probe,
	.remove = oplus_chg_reverse_remove,
};

static __init int oplus_chg_reverse_init(void)
{
	return platform_driver_register(&oplus_chg_reverse_driver);
}

static __exit void oplus_chg_reverse_exit(void)
{
	platform_driver_unregister(&oplus_chg_reverse_driver);
}

oplus_chg_module_register(oplus_chg_reverse);

int oplus_set_reverse_chg_type(struct oplus_mms *topic, int type)
{
	struct oplus_chg_reverse *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -1;
	}

	chip = oplus_mms_get_drvdata(topic);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -1;
	}

	switch (type) {
	case REVERSE_CHG_TYPE_UFCS:
		oplus_set_reverse_chg_type_pd(chip, false);
		oplus_set_reverse_chg_type_pps(chip, false);
		oplus_set_reverse_chg_type_ufcs(chip, true);
		break;
	case REVERSE_CHG_TYPE_PPS:
		oplus_set_reverse_chg_type_pd(chip, false);
		oplus_set_reverse_chg_type_ufcs(chip, false);
		oplus_set_reverse_chg_type_pps(chip, true);
		break;
	case REVERSE_CHG_TYPE_PD:
		oplus_set_reverse_chg_type_ufcs(chip, false);
		oplus_set_reverse_chg_type_pps(chip, false);
		oplus_set_reverse_chg_type_pd(chip, true);
		break;
	case REVERSE_CHG_TYPE_NORMAL:
		oplus_set_reverse_chg_type_ufcs(chip, false);
		oplus_set_reverse_chg_type_pps(chip, false);
		oplus_set_reverse_chg_type_pd(chip, false);
		break;
	default:
		chg_err("this type = %d is not supported\n", type);
		break;
	}
	return 0;
}
