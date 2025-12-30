// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2020 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[CHG_WLS]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/power_supply.h>
#include <linux/miscdevice.h>
#include <linux/sched/clock.h>
#include <linux/pinctrl/consumer.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mtk_boot_common.h>
#endif
#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_monitor.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_batt_bal.h>
#include <oplus_chg_wired.h>
#include <oplus_chg_wls.h>
#include <oplus_hal_wls.h>
#include <oplus_chg_mutual.h>
#include "monitor/oplus_chg_track.h"

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "oplus_cfg.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_MAGCVR_NOTIFY)
#include <soc/oplus/magcvr_notify.h>
#else
static inline int magcvr_event_register_notifier(struct notifier_block *nb)
{
	return 0;
}
static inline int magcvr_event_unregister_notifier(struct notifier_block *nb)
{
	return 0;
}
static inline int magcvr_get_current_pos(void)
{
	return MAGCVR_STATUS_FAR;
}
#endif

#define OPLUS_CHG_WLS_BREAK_DETECT_DELAY	6000
#define OPLUS_CHG_WLS_START_DETECT_DELAY	3000
#define OPLUS_CHG_WLS_MONITOR_DELAY		5000
#define OPLUS_CHG_WLS_RX_MODE_CHECK_DELAY	(30 * 1000)

#define BCC_TO_ICL 			100
#define OPLUS_WLS_BCC_UPDATE_TIME	500
#define OPLUS_WLS_BCC_UPDATE_INTERVAL	round_jiffies_relative(msecs_to_jiffies(OPLUS_WLS_BCC_UPDATE_TIME))
#define HIGH_TEMP_INFO_LEN		(128)
#define HIGH_TEMP_CHECK_LEN		(6)
#define BT_NAME_LEN_MAX		32

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
#define pde_data(inode) PDE_DATA(inode)
#endif

#ifdef WLS_QI_DEBUG
static int wls_dbg_fcc_ma = 0;
module_param(wls_dbg_fcc_ma, int, 0644);
MODULE_PARM_DESC(wls_dbg_fcc_ma, "debug wls fcc ma");

static int wls_dbg_vout_mv = 0;
module_param(wls_dbg_vout_mv, int, 0644);
MODULE_PARM_DESC(wls_dbg_vout_mv, "debug wls vout mv");
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
#ifdef MODULE
#undef printk_ratelimit
#define printk_ratelimit() 0
#endif
#endif

enum wls_err_reason {
	WLS_ERR_NONE,
	WLS_ERR_DES_VERITY,
	WLS_ERR_AES_VERITY,
	WLS_ERR_VOUT_ABNORMAL,
	WLS_ERR_PLOSS_FOD,
	WLS_ERR_PLOSS_FOD_WARN,
	WLS_ERR_ASK_WARN,
	WLS_ERR_ASK_TIMEOUT,
	WLS_ERR_TA_UV,
	WLS_ERR_CEP_TIMEOUT,
	WLS_ERR_RPP_TIMEOUT,
	WLS_ERR_TX_OTP,
	WLS_ERR_Q_VALUE_TIMEOUT,
	WLS_ERR_Q_VALUE_DISCONNECT,
	WLS_ERR_TX_Q_VALUE_ERROR,
	WLS_ERR_SET_TEC_TIMEOUT,
	WLS_ERR_HIGH_TEMP,
	WLS_ERR_FW_UPDATE_FORCE,
	WLS_ERR_FW_UPDATE_FORCE_TX,
	WLS_ERR_FW_UPDATE_BOOT,
	WLS_ERR_FW_UPDATE_FAIL,
	WLS_ERR_EPP_FORCE_TO_BPP,
	WLS_ERR_EPP_QUICK_DISCONNECT_TO_BPP,
	WLS_ERR_FASTCHG_ENTER_TIMEOUT_TO_BPP,
	WLS_ERR_FULL_BRIDGE_ENTER_TIMEOUT_TO_BPP,
	WLS_ERR_EPP_HIGH_TEMP_TO_BPP,
	WLS_ERR_OTHER,
};

struct oplus_chg_wls_fod_cal_data {
	int fod_bpp;
	int fod_epp;
	int fod_fast;
};

struct wls_base_type {
	u8 id;
	int power_max_mw;
};

struct wls_pwr_table {
	u8 f2_id;
	int r_power;/*watt*/
	int t_power;/*watt*/
};

struct battery_info {
	int soc;
	int ui_soc;
	int batt_temp;
	int vbat_mv;
	int ibat_ma;
};

struct wls_mms_info {
	bool rx_present;
	bool rx_online;
	enum oplus_chg_wls_event_code event_code;
	enum wls_status_keep_type wls_status_keep;
};

struct oplus_chg_rx_msg {
	u8 msg_head;
	u8 msg_type;
	u8 data;
	u8 buf[3];
	u8 respone_type;
	bool pending;
	bool long_data;
	bool raw_data;
};

struct wls_dev_cmd {
	unsigned int cmd;
	unsigned int data_size;
	unsigned char data_buf[128];
};

struct wls_auth_result {
	u8 random_num[WLS_AUTH_RANDOM_LEN];
	u8 encode_num[WLS_AUTH_ENCODE_LEN];
};

struct wls_third_part_auth_result {
	int effc_key_index;
	u8 aes_random_num[WLS_AUTH_AES_RANDOM_LEN];
	u8 aes_encode_num[WLS_AUTH_AES_ENCODE_LEN];
};

struct oplus_chg_wls_range_data {
	int32_t low_threshold;
	int32_t high_threshold;
	uint32_t curr_ma;
	uint32_t vol_max_mv;
	int32_t need_wait;
} __attribute__((packed));

struct oplus_chg_wls_bcc_data {
	uint32_t max_batt_volt;
	uint32_t max_curr;
	uint32_t min_curr;
	int32_t exit;
} __attribute__((packed));

struct oplus_chg_wls_skin_range_data {
	int32_t low_threshold;
	int32_t high_threshold;
	uint32_t curr_ma;
} __attribute__((packed));

struct oplus_chg_wls_skewing_data {
	uint32_t curr_ma;
	int32_t fallback_step;
	int32_t switch_to_bpp;
} __attribute__((packed));

struct oplus_chg_wls_fcc_step {
	int max_step;
	struct oplus_chg_wls_range_data fcc_step[WLS_MAX_STEP_CHG_ENTRIES];
	bool allow_fallback[WLS_MAX_STEP_CHG_ENTRIES];
	unsigned long fcc_wait_timeout;
} __attribute__((packed));

struct oplus_chg_wls_fcc_steps {
        struct oplus_chg_wls_fcc_step fcc_step[WLS_FAST_TEMP_MAX];
} __attribute__((packed));

struct oplus_chg_wls_norchg_step {
	int max_step;
	struct oplus_chg_wls_range_data norchg_step[WLS_MAX_STEP_CHG_ENTRIES];
} __attribute__((packed));

#define WLS_NORCHAG_MODE_MAX 5
struct oplus_chg_wls_norchg_steps {
	enum oplus_chg_wls_batt_cl batt_cl;
	int norchg_vol_level;
	enum oplus_temp_region pre_temp_region;
	struct oplus_chg_wls_norchg_step norchg_step[WLS_NORCHAG_MODE_MAX][OPLUS_WLS_CHG_BATT_CL_MAX][TEMP_REGION_MAX];
} __attribute__((packed));

struct oplus_chg_wls_bcc_step {
	int max_step;
	struct oplus_chg_wls_bcc_data bcc_step[BCC_MAX_STEP_ENTRIES];
} __attribute__((packed));

struct oplus_chg_wls_bcc_steps {
	struct oplus_chg_wls_bcc_step bcc_step[WLS_BCC_TEMP_MAX];
} __attribute__((packed));

struct oplus_chg_wls_non_ffc_step {
	int max_step;
	struct oplus_chg_wls_range_data *non_ffc_step;
};

struct oplus_chg_wls_cv_step {
	int max_step;
	struct oplus_chg_wls_range_data *cv_step;
};

struct oplus_chg_wls_cool_down_step {
	int max_step;
	int *cool_down_step;
};

struct oplus_chg_wls_skewing_step {
	int max_step;
	struct oplus_chg_wls_skewing_data *skewing_step;
};

struct oplus_chg_wls_skin_step {
	int max_step;
	struct oplus_chg_wls_skin_range_data *skin_step;
};

struct wls_match_q_type {
	u8 id;
	u8 q_value;
	u8 f_value;
};

struct wls_fod_parm_type {
	u8 id;
	u8 fod_parm[WLS_FOD_PARM_LEN_MAX];
};

struct wls_third_part_qf_parm {
	u8 idh;
	u8 idl;
	u8 q_value;
	u8 f_value;
};

struct wls_third_part_fod_parm {
	u8 idh;
	u8 idl;
	u8 fod_parm[WLS_FOD_PARM_LEN_MAX];
};

struct oplus_chg_wls_static_config {
	bool fastchg_fod_enable;
	bool fastchg_12v_fod_enable;
	int fod_parm_len;
	int fod_parm_array_row_len;
	int bpp_fod_parm_sets;
	int epp_fod_parm_sets;
	int fastchg_fod_parm_sets;
	int third_part_fastchg_qf_parm_sets;
	int third_part_fastchg_qf_parm_magcvr_sets;
	int third_part_epp_qf_parm_sets;
	int third_part_epp_qf_parm_magcvr_sets;
	int third_part_fastchg_fod_parm_sets;
	int third_part_fastchg_fod_parm_magcvr_sets;
	int third_part_epp_fod_parm_sets;
	int third_part_epp_fod_parm_magcvr_sets;
	u8 disable_fod_parm[WLS_FOD_PARM_LEN_MAX];
	struct wls_fod_parm_type bpp_fod_parm[WLS_BASE_NUM_MAX];
	struct wls_fod_parm_type bpp_fod_parm_magcvr[WLS_BASE_NUM_MAX];
	struct wls_fod_parm_type epp_fod_parm[WLS_BASE_NUM_MAX];
	struct wls_fod_parm_type epp_fod_parm_magcvr[WLS_BASE_NUM_MAX];
	struct wls_fod_parm_type fastchg_fod_parm[WLS_BASE_NUM_MAX];
	struct wls_fod_parm_type fastchg_fod_parm_magcvr[WLS_BASE_NUM_MAX];
	struct wls_fod_parm_type fastchg_fod_parm_12v[WLS_BASE_NUM_MAX];
	struct wls_match_q_type fastchg_match_q[WLS_BASE_NUM_MAX];
	struct wls_match_q_type fastchg_match_q_magcvr[WLS_BASE_NUM_MAX];
	struct wls_third_part_qf_parm third_part_fastchg_tx_qf_parm[WLS_THIRD_PART_TX_CONFIG_NUM_MAX];
	struct wls_third_part_qf_parm third_part_fastchg_tx_qf_parm_magcvr[WLS_THIRD_PART_TX_CONFIG_NUM_MAX];
	struct wls_third_part_qf_parm third_part_epp_tx_qf_parm[WLS_THIRD_PART_TX_CONFIG_NUM_MAX];
	struct wls_third_part_qf_parm third_part_epp_tx_qf_parm_magcvr[WLS_THIRD_PART_TX_CONFIG_NUM_MAX];
	struct wls_third_part_fod_parm third_part_fastchg_tx_fod_parm[WLS_THIRD_PART_TX_CONFIG_NUM_MAX];
	struct wls_third_part_fod_parm third_part_fastchg_tx_fod_parm_magcvr[WLS_THIRD_PART_TX_CONFIG_NUM_MAX];
	struct wls_third_part_fod_parm third_part_epp_tx_fod_parm[WLS_THIRD_PART_TX_CONFIG_NUM_MAX];
	struct wls_third_part_fod_parm third_part_epp_tx_fod_parm_magcvr[WLS_THIRD_PART_TX_CONFIG_NUM_MAX];
};

struct oplus_chg_wls_bcc_stop_curr {
	uint32_t bcc_soc_stop_curr[WLS_BCC_TEMP_MAX];
};

struct oplus_chg_wls_dynamic_config {
	int32_t batt_vol_max_mv;
	int32_t iclmax_ma[OPLUS_WLS_CHG_BATT_CL_MAX][OPLUS_WLS_CHG_MODE_MAX][BATT_TEMP_MAX];
	struct oplus_chg_wls_range_data fcc_step[WLS_MAX_STEP_CHG_ENTRIES];
	struct oplus_chg_wls_fcc_steps fcc_steps[WLS_FAST_SOC_MAX];
	struct oplus_chg_wls_range_data non_ffc_step[OPLUS_WLS_CHG_MODE_MAX][WLS_MAX_STEP_CHG_ENTRIES];
	struct oplus_chg_wls_range_data cv_step[OPLUS_WLS_CHG_MODE_MAX][WLS_MAX_STEP_CHG_ENTRIES];
	struct oplus_chg_wls_range_data cv_step_high_batt[OPLUS_WLS_CHG_MODE_MAX][WLS_MAX_STEP_CHG_ENTRIES];
	int32_t cool_down_step[OPLUS_WLS_CHG_MODE_MAX][WLS_COOL_DOWN_LEVEL_MAX];
	struct oplus_chg_wls_skewing_data skewing_step[OPLUS_WLS_SKEWING_MAX][WLS_MAX_STEP_CHG_ENTRIES];
	struct oplus_chg_wls_skin_range_data epp_plus_skin_step[WLS_MAX_STEP_CHG_ENTRIES];
	struct oplus_chg_wls_skin_range_data epp_skin_step[WLS_MAX_STEP_CHG_ENTRIES];
	struct oplus_chg_wls_skin_range_data bpp_skin_step[WLS_MAX_STEP_CHG_ENTRIES];
	int32_t fastchg_curr_max_ma;
	struct oplus_chg_wls_skin_range_data epp_plus_led_on_skin_step[WLS_MAX_STEP_CHG_ENTRIES];
	struct oplus_chg_wls_skin_range_data epp_led_on_skin_step[WLS_MAX_STEP_CHG_ENTRIES];
	int32_t wls_fast_chg_call_on_curr_ma;
	int32_t wls_fast_chg_camera_on_curr_ma;
	int32_t bpp_vol_mv;
	int32_t epp_vol_mv;
	int32_t epp_plus_vol_mv;
	int32_t vooc_vol_mv;
	int32_t svooc_vol_mv;
	int32_t full_bridge_vout_mv;
	int32_t fastchg_init_vout_mv;
	int32_t fastchg_max_soc;
	int32_t fastchg_max_temp;
	int32_t cool_down_12v_thr;
	int32_t verity_curr_max_ma;
	int32_t cp_open_offset_mv;
	int32_t cp_open_offset_min_mv;
	int32_t fastch_wait_thr_mv;
	struct oplus_chg_wls_bcc_stop_curr bcc_stop_curr[WLS_BCC_STOP_SOC_MAX];
	int32_t wls_strategy_soc[WLS_SOC_NUM_MAX];
	struct oplus_chg_wls_fcc_step epp_plus_steps;
	struct oplus_chg_wls_fcc_step epp_steps;
	struct oplus_chg_wls_fcc_step bpp_steps;
	struct oplus_chg_wls_norchg_steps norchg_steps;
	int32_t fastch_max_vbat_mv[BATT_TEMP_MAX];
	int32_t epp_high_temp_thr;
	uint32_t boot_quiet_t;
	uint32_t camera_icl_limit;
	uint32_t camera_handle_icl_limit;
} __attribute__((packed));

struct oplus_wls_chg_rx {
	struct oplus_chg_wls *wls_dev;
	struct device *dev;
	struct oplus_chg_ic_dev *rx_ic;
	struct delayed_work cep_check_work;
	struct completion cep_ok_ack;
	struct wakeup_source *update_fw_wake_lock;

	bool cep_is_ok;
	bool clean_source;

	int vol_set_mv;
};

struct oplus_wls_chg_fast {
	struct oplus_chg_wls *wls_dev;
	struct device *dev;
	struct oplus_chg_ic_dev *fast_ic;
};

struct oplus_wls_chg_normal {
	struct oplus_chg_wls *wls_dev;
	struct device *dev;
	struct oplus_chg_ic_dev *nor_ic;
	struct delayed_work icl_set_work;
	struct delayed_work fcc_set_work;

	struct mutex icl_lock;
	struct mutex fcc_lock;

	int icl_target_ma;
	int icl_step_ma;
	int icl_set_ma;
	int fcc_target_ma;
	int fcc_step_ma;
	int fcc_set_ma;

	bool clean_source;
};


enum high_temp_voter {
	HIGH_TEMP_VOTER_UNKNOW = 0,
	HIGH_TEMP_VOTER_COOLDOWN,
	HIGH_TEMP_VOTER_CURVE,
};

struct oplus_chg_wls_high_temp_track {
	unsigned long wls_start_time;
	unsigned long high_temp_time_diff;
	enum high_temp_voter voter;
	int high_temp_flag;
	int track_wls_type;
	int count;
	int temp[HIGH_TEMP_CHECK_LEN];
};

struct wls_fan_pwm_pulse {
	u32 wls_model_id;
	u32 fan_pwm_pulse_fastchg;
	u32 fan_pwm_pulse_silent;
	u32 fan_pwm_pulse_fastchg_exit;
	u32 fan_pwm_pulse_ui_100;
	u32 fan_pwm_pulse_cool_mode;
};

struct oplus_chg_wls_bt_info {
	char name[BT_NAME_LEN_MAX];
	int connect;
	int incar;
	int carlink;
	int bttype;
};

struct wls_track_record {
	bool ldo_on;
	int max_vrect;
	int wired_charge_type;
	enum oplus_chg_wls_rx_mode rx_mode;
	bool is_fastchg;
	int pmw_scenarios;
	bool break_manu;
};

struct oplus_chg_wls_status {
	u8 adapter_type;
	u8 adapter_id;
	int vendor_id;
	bool tx_extern_cmd_done;
	u32 product_id;
	u8 aes_key_num;
	bool verify_by_aes;
	u8 adapter_power;
	u8 charge_type;
	bool tx_product_id_done;
	int tx_manu_id;
	int last_cep;
	enum oplus_chg_wls_rx_state current_rx_state;
	enum oplus_chg_wls_rx_state next_rx_state;
	enum oplus_chg_wls_rx_state target_rx_state;
	enum oplus_chg_wls_rx_state target_rx_state_backup;
	enum oplus_chg_wls_trx_state trx_state;
	enum oplus_chg_wls_type wls_type;

	enum oplus_temp_region comm_temp_region;
	enum oplus_fcc_gear fcc_gear;
	enum oplus_chg_temp_region temp_region;
	enum oplus_ffc_temp_region ffc_temp_region;

	int state_sub_step;
	int iout_ma;
	int iout_ma_conunt;
	int vout_mv;
	int vrect_mv;
	int trx_curr_ma;
	int trx_vol_mv;
	int fastchg_target_curr_ma;
	int fastchg_target_vol_mv;
	int fastchg_curr_max_ma;
	int fastchg_ibat_max_ma;
	int fastchg_level;
	/*Record the initial temperature when switching to the next gear.*/
	int fastchg_level_init_temp;
	int tx_pwr_mw;
	int rx_pwr_mw;
	int ploos_mw;
	int epp_plus_skin_step;
	int epp_skin_step;
	int bpp_skin_step;
	int epp_plus_led_on_skin_step;
	int epp_led_on_skin_step;
	int pwr_max_mw;
	int tx_pwr_max_mw;
	int tx_mag;
	int ta_typ;
	int tx_version;
	int quiet_mode_need;
	int adapter_info_cmd_count;
	int fastchg_retry_count;
	int fastchg_err_count;
	int non_fcc_level;
	int skewing_level;
	int fast_cep_check;
	int cool_down;
	bool trx_close_delay;
	int bcc_current;
	int wls_bcc_max_curr;
	int wls_bcc_min_curr;
	int wls_bcc_stop_curr;
	int bcc_curve_idx;
	int bcc_true_idx;
	int bcc_temp_range;

	unsigned long cep_ok_wait_timeout;
	unsigned long fastchg_retry_timer;
	unsigned long fastchg_err_timer;
	unsigned long fastchg_enter_timer;
	bool rx_online;
	bool rx_present;
	bool trx_online;
	bool trx_usb_present_once;
	int trx_transfer_start_time;
	int trx_transfer_end_time;
	bool trx_present;
	bool trx_rxac;
	bool trx_present_keep;
	bool is_op_trx;
	bool epp_working;
	bool epp_5w;
	bool quiet_mode;
	bool switch_quiet_mode;
	bool quiet_mode_init;
	bool cep_timeout_adjusted;
	bool upgrade_fw_pending;
	bool fw_upgrading;
	bool fastchg_started;
	bool fastchg_disable;
	bool fastchg_vol_set_ok;
	bool fastchg_curr_set_ok;
	bool fastchg_curr_need_dec;
	bool fastchg_exit_fan_speed_set;
	bool fastchg_ui_100_fan_speed_set;
	bool fastchg_timeout_to_bpp;
	bool fastchg_to_bpp_state_keep;
	bool normal_chg_disabled;
	bool ffc_check;
	bool wait_cep_stable;
	bool ffc_done;
	bool online_keep;
	bool boot_online_keep; /*The driver loading phase of the shutdown charging is set to true*/
	bool led_on;
	bool rx_adc_test_enable;
	bool rx_adc_test_pass;
	bool fod_parm_for_fastchg;
	bool chg_done;
	bool chg_done_quiet_mode;

	bool verity_done;
	bool verity_pass;
	bool verity_data_ok;
	bool aes_verity_data_ok;
	bool verity_started;
	bool verity_state_keep;
	int verity_count;
	int break_count;
	u32 trx_err;
	u8 encrypt_data[WLS_AUTH_RANDOM_LEN];
	u8 aes_encrypt_data[WLS_AUTH_AES_RANDOM_LEN];
	struct wls_auth_result verfity_data;
	unsigned long verity_wait_timeout;
	struct wls_third_part_auth_result aes_verfity_data;

	int epp_chg_level;
	unsigned long epp_check_timeout;
	bool epp_force_to_bpp_flag;
	int epp_to_bpp_reason_record;
	unsigned long epp_disconnect_time;
	unsigned long epp_to_bpp_connect_time;

	struct hrtimer wls_adapter_curve_timer;
	int wls_adapter_curve_gear_cnt;
	int tx_coil;
	int tx_q_val;
	int tx_q_val_frm_rx;
	int q_val_done;
	int ploss_fod_warn_count;
	int ta_uv_count;
	u32 track_error_flag;
	int adapter_curve_pwr;
	struct wls_track_record track_record;
};

struct oplus_chg_wls {
	struct device *dev;
	wait_queue_head_t read_wq;
	struct miscdevice misc_dev;
	struct oplus_mms *wls_topic;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *wired_topic;
	struct oplus_mms *comm_topic;
	struct oplus_mms *err_topic;
	struct oplus_mms *batt_bal_topic;
	struct mms_subscribe *wls_subs;
	struct mms_subscribe *gauge_subs;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *comm_subs;
	struct mms_subscribe *batt_bal_subs;
	struct battery_info batt_info;
	struct wls_mms_info mms_info;

	struct power_supply *wls_psy;
	struct power_supply *batt_psy;
	struct workqueue_struct *wls_wq;
	struct delayed_work wls_rx_sm_work;
	struct delayed_work wls_trx_sm_work;
	struct delayed_work wls_connect_work;
	struct delayed_work wls_send_msg_work;
	struct delayed_work wls_upgrade_fw_work;
	struct delayed_work wls_data_update_work;
	struct delayed_work usb_int_work;
	struct delayed_work vac_int_work;
	struct delayed_work wls_trx_disconnect_work;
	struct delayed_work usb_connect_work;
	struct delayed_work wls_start_work;
	struct delayed_work wls_break_work;
	struct delayed_work fod_cal_work;
	struct delayed_work rx_restore_work;
	struct delayed_work fast_fault_check_work;
	struct delayed_work rx_iic_restore_work;
	struct delayed_work rx_restart_work;
	struct delayed_work rx_verity_restore_work;
	struct delayed_work online_keep_remove_work;
	struct delayed_work verity_state_remove_work;
	struct delayed_work wls_verity_work;
	struct delayed_work wls_get_third_part_verity_data_work;
	struct delayed_work wls_clear_trx_work;
	struct delayed_work wls_skewing_work;
	struct delayed_work wls_bcc_curr_update_work;
	struct delayed_work wls_vout_err_work;
	struct delayed_work wls_mms_init_work;
	struct delayed_work wls_otg_enable_work;
	struct delayed_work wls_monitor_work;
	struct delayed_work wls_set_tec_work;
	struct delayed_work wls_init_complete_work;
	struct delayed_work rx_mode_check_work;
	struct delayed_work wls_match_q_work;
	struct delayed_work wls_ploss_warn_work;
	struct delayed_work wls_epp_check_flag_clear_work;
	struct delayed_work wls_fastchg_to_bpp_keep_state_remove_work;
	struct delayed_work wls_epp_iout_too_small_check_work;
	struct delayed_work wls_boot_quiet_clear_work;
	struct work_struct wls_err_handler_work;
	struct work_struct wls_present_handler_work;
	struct work_struct wls_online_handler_work;
	struct work_struct wls_offline_handler_work;
	struct work_struct wls_event_changed_handler_work;
	struct work_struct temp_region_update_work;
	struct work_struct fcc_gear_update_work;
	struct work_struct cool_down_update_work;
	struct work_struct batt_bal_curr_limit_work;
	struct work_struct adapter_curve_vote_work;
	struct wakeup_source *rx_wake_lock;
	struct wakeup_source *trx_wake_lock;
	struct mutex connect_lock;
	struct mutex read_lock;
	struct mutex cmd_data_lock;
	struct mutex send_msg_lock;
	struct mutex update_data_lock;
	struct mutex tx_switch_lock;
	struct mutex trx_enable_lock;

	struct votable *fcc_votable;
	struct votable *fastchg_disable_votable;
	struct votable *rx_disable_votable;
	struct votable *wrx_en_votable;
	struct votable *nor_icl_votable;
	struct votable *nor_fcc_votable;
	struct votable *nor_fv_votable;
	struct votable *nor_out_disable_votable;
	struct votable *nor_input_disable_votable;
	struct votable *insert_disable_votable;
	struct votable *quiet_mode_votable;
	struct votable *rx_comu_votable;
	struct votable *force_bpp_mode_votable;

	struct oplus_wls_chg_rx *wls_rx;
	struct oplus_wls_chg_fast *wls_fast;
	struct oplus_wls_chg_normal *wls_nor;

	struct oplus_chg_wls_status wls_status;
	struct oplus_chg_wls_static_config static_config;
	struct oplus_chg_wls_dynamic_config dynamic_config;
	struct oplus_chg_wls_bcc_step wls_bcc_step;
	struct oplus_chg_wls_bcc_steps bcc_steps[WLS_BCC_SOC_MAX];
	struct oplus_chg_wls_fcc_step wls_fcc_step;
	struct oplus_chg_wls_fcc_steps fcc_steps[WLS_FAST_SOC_MAX];
	struct oplus_chg_wls_fcc_steps fcc_steps_magcvr[WLS_FAST_SOC_MAX];
	struct oplus_chg_wls_fcc_steps fcc_third_part_steps[WLS_FAST_SOC_MAX];
	struct oplus_chg_wls_non_ffc_step non_ffc_step[OPLUS_WLS_CHG_MODE_MAX];
	struct oplus_chg_wls_cv_step cv_step[OPLUS_WLS_CHG_MODE_MAX];
	struct oplus_chg_wls_cv_step cv_step_high_batt[OPLUS_WLS_CHG_MODE_MAX];
	struct oplus_chg_wls_cool_down_step cool_down_step[OPLUS_WLS_CHG_MODE_MAX];
	struct oplus_chg_wls_skewing_step skewing_step[OPLUS_WLS_SKEWING_MAX];
	struct oplus_chg_wls_skin_step epp_plus_skin_step;
	struct oplus_chg_wls_skin_step epp_skin_step;
	struct oplus_chg_wls_skin_step bpp_skin_step;
	struct oplus_chg_wls_skin_step epp_plus_led_on_skin_step;
	struct oplus_chg_wls_skin_step epp_led_on_skin_step;

	struct oplus_chg_rx_msg rx_msg;
	struct completion msg_ack;
	struct completion nor_ic_ack;
	struct wls_dev_cmd cmd;

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	struct oplus_cfg debug_cfg;
#endif

	u8 *fw_buf;
	int fw_size;
	bool fw_upgrade_by_buf;

	int wrx_en_gpio;
	int ext_pwr_en_gpio;
	int usb_int_gpio;
	int usb_int_irq;
	struct pinctrl *pinctrl;
	struct pinctrl_state *wrx_en_active;
	struct pinctrl_state *wrx_en_sleep;
	struct pinctrl_state *ext_pwr_en_active;
	struct pinctrl_state *ext_pwr_en_sleep;

	bool support_fastchg;
	bool support_get_tx_pwr;
	bool support_epp_plus;
	bool support_tx_boost;
	bool support_wls_and_tx_boost;
	bool support_wls_chg_bcc;
	bool support_wls_use_usb_drv;
	bool support_acdrv_no_ext_boost;
	bool support_no_hw_interlock;
	bool rx_wake_lock_on;
	bool trx_wake_lock_on;
	bool usb_present;
	bool vac_present;
	bool pre_otg_enable;
	bool charge_enable;
	bool charging_disable;
	bool batt_charge_enable;
	bool ftm_mode;
	bool debug_mode;
	bool factory_mode;
	bool fod_is_cal; /*fod is calibrated*/
	bool fod_cal_data_ok;
	bool msg_callback_ok;
	bool cmd_data_ok;
	bool wlsotg_non_coexistence;
	bool wls_init_completed;
	bool support_non_fcc_with_temp;
	int force_rx_mode;
	int force_upgrade;

	struct oplus_chg_wls_fod_cal_data cal_data;
	enum oplus_chg_wls_force_type force_type;
	enum oplus_chg_wls_path_ctrl_type path_ctrl_type;

	int icl_max_ma[OPLUS_WLS_CHG_MODE_MAX];

	int32_t wls_power_mw;
	u32 wls_phone_id;
	u32 rx_protocol_version;
	u32 rx_coil;
	u32 vbridge_ratio;
	u32 magcvr_vbridge_ratio;
	u32 tx_vbridge;
	u32 tx_ibridge;
	u32 tx_ibridge_freq;
	u32 tx_ploss;
	u32 tx_ploss_voted;
	u32 tx_ploss_warning;
	u32 ta_uv;
	u32 tec_power;
	u32 tx_tec_power;
	u32 tec_init_power;
	u32 non_mag_power_mw;
	unsigned long ploss_interval_jiffies;
	unsigned int wls_bcc_fcc_to_icl_factor;
	int little_warm_batt_temp;
	int normal_high_batt_temp;
	u32 wls_boost_curr_limit_ma;
	int monitor_count;

	struct oplus_chg_mutual_notifier aes_mutual;

	struct notifier_block magcvr_notify;
	int magcvr_status;
	int magcvr_update_complete;

	int wls_notify;
	int callname;
	struct oplus_chg_wls_high_temp_track high_temp_track;
	struct delayed_work wls_high_temp_track_work;
	struct oplus_chg_wls_bt_info bt_info;
};

struct wls_adapter_curve_step {
	int adapter_new_power_r;
	int adapter_old_power_r;
	int tec_power;
	int time_s;  /* -1 is end, then use the last pws to charge*/
};

struct wls_adapter_curve_table {
	int f2_id;
	struct wls_adapter_curve_step curves[ADAPTER_CURVE_GEAR_MAX];
};

static struct wls_adapter_curve_table adapter_curves_table[] = {
	/* 33W */
	{0x1A, {{12, 12, 82, -1}}}, {0x1B, {{12, 12, 82, -1}}}, {0x49, {{12, 12, 82, -1}}},
	{0x4A, {{12, 12, 82, -1}}}, {0x61, {{12, 12, 82, -1}}},
	/* 44W */
	{0x1D, {{12, 12, 82, -1}}}, {0x1E, {{12, 12, 82, -1}}}, {0x22, {{12, 12, 82, -1}}},
	/* 45W  12W - */
	{0x1C, {{12, 12, 82, 1100}, {12, 12, 100, -1}}},
	/* 50W */
	{0x11, {{25, 30, 64, 290}, {20, 20, 64, -1}}}, {0x12, {{25, 30, 64, 290}, {20, 20, 64, -1}}}, {0x21, {{25, 30, 64, 290}, {20, 20, 64, -1}}},
	{0x23, {{25, 30, 64, 290}, {20, 20, 64, -1}}}, {0x31, {{25, 30, 64, 290}, {20, 20, 64, -1}}}, {0x33, {{25, 30, 64, 290}, {20, 20, 64, -1}}},
	{0x62, {{25, 30, 64, 290}, {20, 20, 64, -1}}}, {0x03, {{25, 30, 64, 290}, {20, 20, 64, -1}}},
	/* 55W */
	{0x24, {{25, 30, 64, 290}, {20, 25, 64, -1}}}, {0x25, {{25, 30, 64, 290}, {20, 25, 64, -1}}},
	{0x26, {{25, 30, 64, 290}, {20, 25, 64, -1}}}, {0x27, {{25, 30, 64, 290}, {20, 25, 64, -1}}},
	/* 65W */
	{0x14, {{35, 35, 64, 290}, {25, 30, 64, 600}, {20, 25, 64, -1}}}, {0x28, {{35, 35, 64, 290}, {25, 30, 64, 600}, {20, 25, 64, -1}}},
	{0x2A, {{35, 35, 64, 290}, {25, 30, 64, 600}, {20, 25, 64, -1}}}, {0x35, {{35, 35, 64, 290}, {25, 30, 64, 600}, {20, 25, 64, -1}}},
	{0x63, {{35, 35, 64, 290}, {25, 30, 64, 600}, {20, 25, 64, -1}}}, {0x66, {{35, 35, 64, 290}, {25, 30, 64, 600}, {20, 25, 64, -1}}},
	{0x6E, {{35, 35, 64, 290}, {25, 30, 64, 600}, {20, 25, 64, -1}}}, {0x04, {{35, 35, 64, 290}, {25, 30, 64, 600}, {20, 25, 64, -1}}},

	/* 66W */
	{0x2B, {{35, 35, 64, 170}, {25, 30, 64, 180}, {25, 25, 64, -1}}}, {0x36, {{35, 35, 64, 170}, {25, 30, 64, 180}, {25, 25, 64, -1}}},
	{0x64, {{35, 35, 64, 170}, {25, 30, 64, 180}, {25, 25, 64, -1}}},
	/* 67W */
	{0x6C, {{35, 35, 64, 410}, {25, 30, 64, 720}, {25, 25, 64, -1}}}, {0x2C, {{35, 35, 64, 410}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	{0x2D, {{35, 35, 64, 410}, {25, 30, 64, 720}, {25, 25, 64, -1}}}, {0x2E, {{35, 35, 64, 410}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	{0x6D, {{35, 35, 64, 410}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	/* 80W */
	{0x65, {{50, 50, 100, 20}, {35, 35, 64, 240}, {25, 30, 64, 480}, {25, 25, 64, -1}}},
	/* 80W */
	{0x4B, {{50, 50, 100, 50}, {35, 35, 64, 420}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	{0x4C, {{50, 50, 100, 50}, {35, 35, 64, 420}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	{0x4D, {{50, 50, 100, 50}, {35, 35, 64, 420}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	{0x4E, {{50, 50, 100, 50}, {35, 35, 64, 420}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	/* 88W */
	{0x37, {{50, 50, 64, 50}, {35, 35, 64, 420}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	{0x38, {{50, 50, 64, 50}, {35, 35, 64, 420}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	{0x39, {{50, 50, 64, 50}, {35, 35, 64, 420}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	{0x3A, {{50, 50, 64, 50}, {35, 35, 64, 420}, {25, 30, 64, 720}, {25, 25, 64, -1}}},
	/* 100W */
	{0x69, {{50, 50, 64, 170}, {40, 50, 64, 300}, {35, 35, 64, 600}, {25, 30, 64, 300}, {25, 25, 64, -1}}},
	{0x6A, {{50, 50, 64, 170}, {40, 50, 64, 300}, {35, 35, 64, 600}, {25, 30, 64, 300}, {25, 25, 64, -1}}},
	{0x3B, {{50, 50, 64, 170}, {40, 50, 64, 300}, {35, 35, 64, 600}, {25, 30, 64, 300}, {25, 25, 64, -1}}},
	{0x3C, {{50, 50, 64, 170}, {40, 50, 64, 300}, {35, 35, 64, 600}, {25, 30, 64, 300}, {25, 25, 64, -1}}},
	{0x3D, {{50, 50, 64, 170}, {40, 50, 64, 300}, {35, 35, 64, 600}, {25, 30, 64, 300}, {25, 25, 64, -1}}},
	{0x3E, {{50, 50, 64, 170}, {40, 50, 64, 300}, {35, 35, 64, 600}, {25, 30, 64, 300}, {25, 25, 64, -1}}},
	/* 120W */
	{0x47, {{80, 80, 64, 20}, {50, 50, 64, 150}, {40, 50, 64, 300}, {35, 35, 64, 600}, {25, 30, 64, 300}, {25, 25, 64, -1}}},
	{0x48, {{80, 80, 64, 20}, {50, 50, 64, 150}, {40, 50, 64, 300}, {35, 35, 64, 600}, {25, 30, 64, 300}, {25, 25, 64, -1}}},
	{0x32, {{80, 80, 64, 20}, {50, 50, 64, 150}, {40, 50, 64, 300}, {35, 35, 64, 600}, {25, 30, 64, 300}, {25, 25, 64, -1}}},
	{0x6B, {{80, 80, 64, 20}, {50, 50, 64, 150}, {40, 50, 64, 300}, {35, 35, 64, 600}, {25, 30, 64, 300}, {25, 25, 64, -1}}},
};

struct oplus_chg_wls_state_handler {
	int (*enter_state)(struct oplus_chg_wls *wls_dev);
	int (*handle_state)(struct oplus_chg_wls *wls_dev);
	int (*exit_state)(struct oplus_chg_wls *wls_dev);
};

static struct wls_base_type wls_base_table[] = {
	{ 0x00, 30000 }, { 0x01, 40000 }, { 0x02, 50000 }, { 0x03, 50000 }, { 0x04, 50000 },
	{ 0x05, 50000 }, { 0x06, 50000 }, { 0x07, 50000 }, { 0x08, 50000 }, { 0x09, 50000 },
	{ 0x0a, 100000 }, { 0x0b, 100000 }, { 0x10, 100000 }, { 0x11, 100000 }, { 0x12, 100000 },
	{ 0x13, 100000 }, { 0x1f, 50000 },
};

static u8 oplus_trx_id_table[] = {
	0x02, 0x03, 0x04, 0x05, 0x06
};

/*static int oplus_chg_wls_pwr_table[] = {0, 12000, 12000, 35000, 50000};*/
static struct wls_pwr_table oplus_chg_wls_pwr_table[] = {/*(f2_id, r_power, t_power)*/
	{ 0x00, 12, 15 }, { 0x01, 12, 20 }, { 0x02, 12, 30 }, { 0x03, 35, 50 }, { 0x04, 45, 65 },
	{ 0x05, 50, 75 }, { 0x06, 60, 85 }, { 0x07, 65, 95 }, { 0x08, 75, 105 }, { 0x09, 80, 115 },
	{ 0x0A, 90, 125 }, { 0x0B, 20, 20 }, { 0x0C, 100, 140 }, { 0x0D, 115, 160 }, { 0x0E, 130, 180 },
	{ 0x0F, 145, 200 },
	{ 0x11, 35, 50 }, { 0x12, 35, 50 }, { 0x13, 12, 20 }, { 0x14, 45, 65 }, { 0x15, 12, 20 },
	{ 0x16, 12, 20 }, { 0x17, 12, 30 }, { 0x18, 12, 30 }, { 0x19, 12, 30 }, { 0x1A, 12, 33 },
	{ 0x1B, 12, 33 }, { 0x1C, 12, 44 }, { 0x1D, 12, 44 }, { 0x1E, 12, 44 },
	{ 0x21, 35, 50 }, { 0x22, 12, 44 }, { 0x23, 35, 50 }, { 0x24, 35, 55 }, { 0x25, 35, 55 },
	{ 0x26, 35, 55 }, { 0x27, 35, 55 }, { 0x28, 45, 65 }, { 0x29, 12, 30 }, { 0x2A, 45, 65 },
	{ 0x2B, 45, 66 }, { 0x2C, 45, 67 }, { 0x2D, 45, 67 }, { 0x2E, 45, 67 },
	{ 0x31, 35, 50 }, { 0x32, 90, 120 }, { 0x33, 35, 50 }, { 0x34, 12, 20 }, { 0x35, 45, 65 },
	{ 0x36, 45, 66 }, { 0x37, 50, 88 }, { 0x38, 50, 88 }, { 0x39, 50, 88 }, { 0x3A, 50, 88 },
	{ 0x3B, 75, 100 }, { 0x3C, 75, 100 }, { 0x3D, 75, 100 }, { 0x3E, 75, 100 },
	{ 0x41, 12, 30 }, { 0x42, 12, 30 }, { 0x43, 12, 30 }, { 0x44, 12, 30 }, { 0x45, 12, 30 },
	{ 0x46, 12, 30 }, { 0x47, 90, 120 }, { 0x48, 90, 120 }, { 0x49, 12, 33 }, { 0x4A, 12, 33 },
	{ 0x4B, 50, 80 }, { 0x4C, 50, 80 }, { 0x4D, 50, 80 }, { 0x4E, 50, 80 },
	{ 0x51, 90, 125 },
	{ 0x61, 12, 33 }, { 0x62, 35, 50 }, { 0x63, 45, 65 }, { 0x64, 45, 66 }, { 0x65, 50, 80 },
	{ 0x66, 45, 65 }, { 0x67, 90, 125 }, { 0x68, 90, 125 }, { 0x69, 75, 100 }, { 0x6A, 75, 100 },
	{ 0x6B, 90, 120 }, { 0x6C, 45, 67 }, { 0x6D, 45, 67 }, { 0x6E, 45, 65 },
	{ 0x7F, 30, 0 },
};


static struct wls_pwr_table oplus_chg_wls_tripartite_pwr_table[] = {
	{0x01, 20, 20}, {0x02, 30, 30}, {0x03, 40, 40},  {0x04, 50, 50},
};

static const char * const oplus_chg_wls_rx_state_text[] = {
	[OPLUS_CHG_WLS_RX_STATE_DEFAULT] = "default",
	[OPLUS_CHG_WLS_RX_STATE_BPP] = "bpp",
	[OPLUS_CHG_WLS_RX_STATE_EPP] = "epp",
	[OPLUS_CHG_WLS_RX_STATE_EPP_PLUS] = "epp-plus",
	[OPLUS_CHG_WLS_RX_STATE_FAST] = "fast",
	[OPLUS_CHG_WLS_RX_STATE_FFC] = "ffc",
	[OPLUS_CHG_WLS_RX_STATE_DONE] = "done",
	[OPLUS_CHG_WLS_RX_STATE_QUIET] = "quiet",
	[OPLUS_CHG_WLS_RX_STATE_STOP] = "stop",
	[OPLUS_CHG_WLS_RX_STATE_DEBUG] = "debug",
	[OPLUS_CHG_WLS_RX_STATE_FTM] = "ftm",
	[OPLUS_CHG_WLS_RX_STATE_ERROR] = "error",
};

static const char * const oplus_chg_wls_trx_state_text[] = {
	[OPLUS_CHG_WLS_TRX_STATE_DEFAULT] = "default",
	[OPLUS_CHG_WLS_TRX_STATE_READY] = "ready",
	[OPLUS_CHG_WLS_TRX_STATE_WAIT_PING] = "wait_ping",
	[OPLUS_CHG_WLS_TRX_STATE_TRANSFER] = "transfer",
	[OPLUS_CHG_WLS_TRX_STATE_OFF] = "off",
};

static const char * const wls_err_reason_text[] = {
	[WLS_ERR_NONE] = "none",
	[WLS_ERR_DES_VERITY] = "des_verity",
	[WLS_ERR_AES_VERITY] = "aes_verity",
	[WLS_ERR_VOUT_ABNORMAL] = "vout_abnormal",
	[WLS_ERR_PLOSS_FOD] = "ploss_fod",
	[WLS_ERR_PLOSS_FOD_WARN] = "ploss_fod_warn",
	[WLS_ERR_ASK_WARN] = "ask_warn",
	[WLS_ERR_ASK_TIMEOUT] = "ask_timeout",
	[WLS_ERR_TA_UV] = "ta_uv",
	[WLS_ERR_CEP_TIMEOUT] = "cep_timeout",
	[WLS_ERR_RPP_TIMEOUT] = "rpp_timeout",
	[WLS_ERR_TX_OTP] = "tx_otp",
	[WLS_ERR_Q_VALUE_TIMEOUT] = "q_value_timeout",
	[WLS_ERR_Q_VALUE_DISCONNECT] = "q_value_disconnect",
	[WLS_ERR_TX_Q_VALUE_ERROR] = "q_value_error",
	[WLS_ERR_SET_TEC_TIMEOUT] = "set_tec_timeout",
	[WLS_ERR_HIGH_TEMP] = "high_temp",
	[WLS_ERR_FW_UPDATE_FORCE] = "fw_update_force",
	[WLS_ERR_FW_UPDATE_FORCE_TX] = "fw_update_force_tx",
	[WLS_ERR_FW_UPDATE_BOOT] = "fw_update_boot",
	[WLS_ERR_FW_UPDATE_FAIL] = "fw_update_fail",
	[WLS_ERR_EPP_FORCE_TO_BPP] = "epp_force_to_bpp",
	[WLS_ERR_EPP_QUICK_DISCONNECT_TO_BPP] = "epp_quick_disconnect_to_bpp",
	[WLS_ERR_FASTCHG_ENTER_TIMEOUT_TO_BPP] = "fast_enter_timeout_to_bpp",
	[WLS_ERR_FULL_BRIDGE_ENTER_TIMEOUT_TO_BPP] = "full_bridge_enter_timeout_to_bpp",
	[WLS_ERR_EPP_HIGH_TEMP_TO_BPP] = "err_high_temp_to_bpp",
	[WLS_ERR_OTHER] = "other",
};

static u8 oplus_chg_wls_disable_fod_parm[] = {
	0xff, 0x7f, 0xff, 0x7f, 0xff, 0x7f,
	0xff, 0x7f, 0xff, 0x7f, 0xff, 0x7f,
	0xff, 0x7f,
};

static struct oplus_chg_wls_dynamic_config default_config = {
	.fcc_step = {
		{},
		{},
		{},
		{},
	},
};

const int oplus_strategy_soc_default_para_old_ver[WLS_SOC_NUM_MAX_OLD_VER] = {30, 70, 90};
const int oplus_strategy_soc_default_para[WLS_SOC_NUM_MAX] = {20, 35, 55, 75, 85, 90};

const int32_t default_wls_epp_strategy[] = {
	0, 375, 900, 1100, 1,
	350, 430, 800, 1000, 1,
	370, 450, 600, 800, 1,
	400, 530, 400, 500, 1
};

const int32_t default_wls_epp_plus_strategy[] = {
	0, 375, 1200, 1500, 1,
	350, 430, 800, 1000, 1,
	370, 450, 600, 800, 1,
	400, 530, 400, 500, 1
};

const int32_t default_wls_bpp_strategy[] = {
	0, 460, 750, 1000, 1,
	440, 530, 400, 1000, 1
};

static int oplus_chg_wls_track_upload_wls_err_info(struct oplus_chg_wls *wls_dev,
	enum wls_err_scene scene_type, enum wls_err_reason reason_type);
static void oplus_chg_wls_rx_msg_callback(void *dev_data, u8 data[]);
static void oplus_chg_wls_ploss_warn_ta_uv(struct oplus_chg_wls *wls_dev, u8 err_type);
static int oplus_chg_wls_set_tec_power(struct oplus_chg_wls *wls_dev, int power);
static int oplus_chg_wls_entry_rx_mode(struct oplus_chg_wls *wls_dev, enum oplus_chg_wls_rx_mode rx_mode, const char *client_str);
static int oplus_chg_wls_rx_set_rx_mode_safety(struct oplus_chg_wls *wls_dev, enum oplus_chg_wls_rx_mode rx_mode);
static int oplus_chg_wls_send_raw_data(struct oplus_chg_wls *wls_dev, u8 msg[2], u8 data[3], int wait_time_s);
static int oplus_chg_wls_send_match_q(struct oplus_chg_wls *wls_dev);
static int oplus_chg_wls_choose_bpp_epp_curve(struct oplus_chg_wls *wls_dev);
static void oplus_chg_wls_high_temp_check(enum high_temp_voter voter, struct oplus_chg_wls *wls_dev, int icl_ma);
static void oplus_chg_wls_high_temp_update_track_info(struct oplus_chg_wls *wls_dev, char *buff);
static int oplus_chg_wls_epp_force_to_bpp_loop_check(struct oplus_chg_wls *wls_dev);


__maybe_unused static bool is_nor_fv_votable_available(struct oplus_chg_wls *wls_dev)
{
	if (!wls_dev->nor_fv_votable)
		wls_dev->nor_fv_votable = find_votable("FV_MAX");
	return !!wls_dev->nor_fv_votable;
}

__maybe_unused static bool is_wls_psy_available(struct oplus_chg_wls *wls_dev)
{
	if (!wls_dev->wls_psy)
		wls_dev->wls_psy = power_supply_get_by_name("wireless");
	return !!wls_dev->wls_psy;
}

__maybe_unused static bool is_batt_psy_available(struct oplus_chg_wls *wls_dev)
{
	if (!wls_dev->batt_psy)
		wls_dev->batt_psy = power_supply_get_by_name("battery");
	return !!wls_dev->batt_psy;
}

__maybe_unused static bool is_err_topic_available(struct oplus_chg_wls *wls_dev)
{
	if (!wls_dev->err_topic)
		wls_dev->err_topic = oplus_mms_get_by_name("error");
	return !!wls_dev->err_topic;
}

__maybe_unused static bool is_batt_bal_topic_available(struct oplus_chg_wls *wls_dev)
{
	if (!wls_dev->batt_bal_topic)
		wls_dev->batt_bal_topic = oplus_mms_get_by_name("batt_bal");
	return !!wls_dev->batt_bal_topic;
}

__maybe_unused static bool is_rx_ic_available(struct oplus_wls_chg_rx *wls_rx)
{
	struct device_node *node = oplus_get_node_by_type(wls_rx->dev->of_node);

	if (wls_rx->rx_ic == NULL)
		wls_rx->rx_ic = of_get_oplus_chg_ic(node, "oplus,rx_ic", 0);
	return !!wls_rx->rx_ic;
}

__maybe_unused static bool oplus_chg_wls_is_usb_present(struct oplus_chg_wls *wls_dev)
{
	union mms_msg_data data = { 0 };

	if (!wls_dev || !wls_dev->wired_topic)
		return false;
	oplus_mms_get_item_data(wls_dev->wired_topic, WIRED_ITEM_PRESENT, &data, false);
	return !!data.intval;
}

__maybe_unused static bool oplus_chg_wls_is_usb_connected(struct oplus_chg_wls *wls_dev)
{
	if (!gpio_is_valid(wls_dev->usb_int_gpio)) {
		chg_err("usb_int_gpio invalid\n");
		return false;
	}

	return !!gpio_get_value(wls_dev->usb_int_gpio);
}


static void oplus_chg_wls_publish_wls_type(struct oplus_chg_wls *wls_dev, bool set_any, bool reset)
{
	struct mms_msg *msg;
	int rc;
	static enum oplus_chg_wls_type pre_wls_type = -1;

	if (reset == true) {
		pre_wls_type = -1;
		chg_info("pre_wls_type=%d, reset:%d\n", pre_wls_type, reset);
		return;
	}

	if (wls_dev == NULL || wls_dev->wls_topic == NULL) {
		chg_err("wls_topic not ready\n");
		return;
	}

	if ((set_any == false) && (wls_dev->wls_status.wls_type == pre_wls_type))
		 return;

	chg_info("wls_type=%d, pre_wls_type:%d\n", wls_dev->wls_status.wls_type, pre_wls_type);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, WLS_ITEM_WLS_TYPE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(wls_dev->wls_topic, msg);
	if (rc < 0) {
		chg_err("publish wls wls_type msg error, rc=%d\n", rc);
		kfree(msg);
	}

	pre_wls_type = wls_dev->wls_status.wls_type;
}

static int oplus_chg_wls_get_base_power_max(u8 id)
{
	int i;
	int pwr = WLS_VOOC_PWR_MAX_MW;

	for (i = 0; i < ARRAY_SIZE(wls_base_table); i++) {
		if (wls_base_table[i].id == id) {
			pwr = wls_base_table[i].power_max_mw;
			return pwr;
		}
	}

	return pwr;
}

static void oplus_chg_wls_cam_scene_icl_limit(struct oplus_chg_wls *wls_dev)
{
	if (!wls_dev) {
		chg_err("wls_dev is null\n");
		return;
	}

	if (wls_dev->mms_info.rx_present != 1)
		return;

	if (wls_dev->dynamic_config.camera_handle_icl_limit > 0 &&
	    wls_dev->bt_info.connect == 1 && wls_dev->bt_info.bttype == BT_MAG_HANDLE) {
		cancel_delayed_work_sync(&wls_dev->wls_nor->icl_set_work);
		vote(wls_dev->nor_icl_votable, WLS_CAMERA_MODE_VOTER, true,
		     wls_dev->dynamic_config.camera_handle_icl_limit, false);
		return;
	}

	if (wls_dev->dynamic_config.camera_icl_limit > 0) {
		cancel_delayed_work_sync(&wls_dev->wls_nor->icl_set_work);
		vote(wls_dev->nor_icl_votable, WLS_CAMERA_MODE_VOTER, true,
		     wls_dev->dynamic_config.camera_icl_limit, false);
	}

	return;
}

static void oplus_chg_wls_adapter_curve_vote_work(struct work_struct *work)
{
	struct oplus_chg_wls *wls_dev =
		container_of(work, struct oplus_chg_wls, adapter_curve_vote_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;

	chg_info("wls_status adapter_curve_pwr:%d\n", wls_status->adapter_curve_pwr);
	vote(wls_dev->fcc_votable, RX_ADAPTER_CURVE_VOTER, true,
	     (wls_status->adapter_curve_pwr * 1000 * 1000 / dynamic_cfg->svooc_vol_mv), false);
}

static int oplus_wls_adapter_curve_timer_start(struct oplus_chg_wls *wls_dev)
{
	if (!wls_dev) {
		chg_err("wls_dev is null\n");
		return -ENODEV;
	}

	if (!hrtimer_active(&wls_dev->wls_status.wls_adapter_curve_timer)) {
		hrtimer_start(&wls_dev->wls_status.wls_adapter_curve_timer, 0, HRTIMER_MODE_REL);
		chg_info("wls wls_adapter_curve_timer start\n");
		return 0;
	}

	chg_info("wls wls_adapter_curve_timer is active\n");
	return -EBUSY;
}

static void oplus_wls_adapter_curve_timer_stop(struct oplus_chg_wls *wls_dev)
{
	if (!wls_dev) {
		chg_err("wls_dev is null\n");
		return;
	}

	hrtimer_cancel(&wls_dev->wls_status.wls_adapter_curve_timer);
	wls_dev->wls_status.wls_adapter_curve_gear_cnt = 0;
	chg_info("wls wls_adapter_curve_timer is stop\n");
}


enum hrtimer_restart wls_adapter_curve_timer_callback(struct hrtimer *timer) {
	int pwr = 0;
	int time = 0;
	int i;
	struct oplus_chg_wls_status *wls_status = container_of(timer, struct oplus_chg_wls_status, wls_adapter_curve_timer);
	struct oplus_chg_wls *wls_dev = container_of(wls_status, struct oplus_chg_wls, wls_status);
	u8 f2_id = wls_status->adapter_power & 0x7F;
	int count = wls_status->wls_adapter_curve_gear_cnt;

	if (count >= ADAPTER_CURVE_GEAR_MAX) {
		chg_err("wls cnt is out of limit\n");
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(adapter_curves_table); i++) {
		if (adapter_curves_table[i].f2_id == f2_id) {
			pwr = wls_status->adapter_id < WLS_ADAPTER_MODEL_4 ?
				adapter_curves_table[i].curves[count].adapter_old_power_r : adapter_curves_table[i].curves[count].adapter_new_power_r;
			time = adapter_curves_table[i].curves[count].time_s;
			wls_dev->tec_power = adapter_curves_table[i].curves[count].tec_power;
			wls_status->wls_adapter_curve_gear_cnt++;
			break;
		}
	}

	chg_info("pwr:%d, time:%d, cnt:%d, tec_power:%d\n", pwr, time, count, wls_dev->tec_power);
	if (pwr <= 0)
		goto out;

	if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30 &&
		wls_status->adapter_id >= WLS_ADAPTER_MODEL_4 &&
		wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY &&
		(wls_status->tx_mag == TX_MAG_TEC || wls_status->tx_mag == TX_NON_MAG_TEC)) {
		if (wls_dev->tec_init_power == 0)
			schedule_delayed_work(&wls_dev->wls_set_tec_work, 0);
		else
			schedule_delayed_work(&wls_dev->wls_set_tec_work, msecs_to_jiffies(10 * 1000));
		wls_dev->tec_init_power = 1;
	}

	wls_status->adapter_curve_pwr = pwr;
	schedule_work(&wls_dev->adapter_curve_vote_work);

	if (time < 0)
		goto out;

	hrtimer_forward_now(timer, ktime_set(time, 0));
	return HRTIMER_RESTART;
out:
	chg_info("wls_adapter_curve_timer no start!\n");
	return HRTIMER_NORESTART;  /* do not restart timer */
}

static bool oplus_wls_adapter_curve_runing(struct oplus_chg_wls *wls_dev)
{
	u8 f2_id = 0;
	struct oplus_chg_wls_status *wls_status = NULL;
	int ret = 0;
	int i;

	if (!wls_dev) {
		chg_err("wls_dev is null\n");
		return false;
	}

	wls_status = &wls_dev->wls_status;
	f2_id = wls_status->adapter_power & 0x7F;

	for (i = 0; i < ARRAY_SIZE(adapter_curves_table); i++) {
		if (adapter_curves_table[i].f2_id == f2_id) {
			ret = oplus_wls_adapter_curve_timer_start(wls_dev);
			if (!ret) {
				chg_err("wls chg wls curve runing\n");
				return true;
			}
			break;
		}
	}

	chg_info("adapter is not on the adapter_curves_table\n");

	return false;
}

static int oplus_chg_wls_get_r_power(struct oplus_chg_wls *wls_dev, u8 f2_data)
{
	int i = 0;
	int r_pwr = WLS_RECEIVE_POWER_DEFAULT;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	for (i = 0; i < ARRAY_SIZE(oplus_chg_wls_pwr_table); i++) {
		if (oplus_chg_wls_pwr_table[i].f2_id == (f2_data & 0x7F)) {
			r_pwr = oplus_chg_wls_pwr_table[i].r_power * 1000;
			break;
		}
	}
	if (wls_status->wls_type == OPLUS_CHG_WLS_PD_65W)
		return WLS_RECEIVE_POWER_PD65W;
	return r_pwr;
}

static int oplus_chg_wls_get_tripartite_r_power(u8 f2_data)
{
	int i = 0;
	int r_pwr = WLS_RECEIVE_POWER_DEFAULT;

	for (i = 0; i < ARRAY_SIZE(oplus_chg_wls_tripartite_pwr_table); i++) {
		if (oplus_chg_wls_tripartite_pwr_table[i].f2_id == (f2_data & 0x7F)) {
			r_pwr = oplus_chg_wls_tripartite_pwr_table[i].r_power * 1000;
			break;
		}
	}
	return r_pwr;
}

#define DEFAULT_FACTOR	1
static int vout_factor(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int factor = DEFAULT_FACTOR;

	if (dynamic_cfg->svooc_vol_mv > 0)
		factor = WLS_RX_VOL_MAX_MV / dynamic_cfg->svooc_vol_mv;
	return factor;
}


static bool oplus_chg_wls_third_part_fod_parm_config(struct oplus_chg_wls *wls_dev, u8 *fod_parm,
				struct wls_third_part_fod_parm *tx_fod, int tx_fod_set, int tx_id)
{
	int tx_id_config = 0;
	int i = 0;

	if (!tx_fod || tx_fod_set > WLS_THIRD_PART_TX_CONFIG_NUM_MAX || tx_fod_set <= 0)
		return false;

	for (i = 0; i < tx_fod_set; i++, tx_fod++) {
		tx_id_config = (tx_fod->idh << 8) | tx_fod->idl;
		if (tx_id_config == tx_id) {
			chg_info("tx_id_config: %x\n", tx_id_config);
			memmove(fod_parm, tx_fod->fod_parm, wls_dev->static_config.fod_parm_len);
			return true;
		}
	}

	return false;
}

static int oplus_chg_wls_third_part_qf_parm_config(struct oplus_chg_wls *wls_dev, int qf_type,
				struct wls_third_part_qf_parm *tx_qf, int tx_qf_set, int tx_id)
{
	int tx_id_config = 0;
	int i = 0;

	if (!tx_qf || tx_qf_set > WLS_THIRD_PART_TX_CONFIG_NUM_MAX || tx_qf_set <= 0)
		return 0;

	for (i = 0; i < tx_qf_set; i++, tx_qf++) {
		tx_id_config = (tx_qf->idh << 8) | tx_qf->idl;
		if (tx_id_config == tx_id) {
			chg_info("tx_id_config:%x, q_value:%x, f_value:%x\n", tx_id_config,
				tx_qf->q_value, tx_qf->f_value);
			return qf_type == Q_VALUE ? tx_qf->q_value : tx_qf->f_value;
		}
	}

	return 0;
}

#define WLS_BASE_ID_DEFAULT	0x02
static u8 oplus_chg_wls_get_fastchg_qf_value(struct oplus_chg_wls *wls_dev, u8 id, int qf_type)
{
	int i;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_static_config *static_cfg = &wls_dev->static_config;
	int qf_val = 0;

	if (wls_status->adapter_id == WLS_ADAPTER_THIRD_PARTY && wls_status->tx_product_id_done) {
		if (wls_dev->magcvr_status == MAGCVR_STATUS_NEAR)
			qf_val = oplus_chg_wls_third_part_qf_parm_config(wls_dev, qf_type,
				     static_cfg->third_part_fastchg_tx_qf_parm_magcvr,
				     static_cfg->third_part_fastchg_qf_parm_magcvr_sets,
				     wls_status->product_id);
		else
			qf_val = oplus_chg_wls_third_part_qf_parm_config(wls_dev, qf_type,
				     static_cfg->third_part_fastchg_tx_qf_parm,
				     static_cfg->third_part_fastchg_qf_parm_sets,
				     wls_status->product_id);
		if (qf_val) {
			qf_val = qf_type == F_VALUE ? qf_val & F_VALUE_MASK : qf_val;
			return qf_val;
		}
	}

	for (i = 0; i < WLS_BASE_NUM_MAX; i++) {
		if (wls_dev->magcvr_status == MAGCVR_STATUS_NEAR) {
			if (wls_dev->static_config.fastchg_match_q_magcvr[i].id == id)
				return qf_type == Q_VALUE ?
				       wls_dev->static_config.fastchg_match_q_magcvr[i].q_value :
				       wls_dev->static_config.fastchg_match_q_magcvr[i].f_value & F_VALUE_MASK;
		} else {
			if (wls_dev->static_config.fastchg_match_q[i].id == id)
				return qf_type == Q_VALUE ?
				       wls_dev->static_config.fastchg_match_q[i].q_value :
				       wls_dev->static_config.fastchg_match_q[i].f_value & F_VALUE_MASK;
		}
	}

	for (i = 0; i < WLS_BASE_NUM_MAX; i++) {
		if (wls_dev->static_config.fastchg_match_q[i].id == WLS_BASE_ID_DEFAULT)
			return qf_type == Q_VALUE ?
			       wls_dev->static_config.fastchg_match_q[i].q_value :
			       wls_dev->static_config.fastchg_match_q[i].f_value & F_VALUE_MASK;
	}

	return 0;
}

#define WLS_EPP_QF_PARM_ID_DEFAULT	0xff
static u8 oplus_chg_wls_get_epp_qf_value(struct oplus_chg_wls *wls_dev, u8 id, int qf_type)
{
	int qf_val = 0;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_static_config *static_cfg = &wls_dev->static_config;

	if (wls_status->tx_manu_id != 0 && wls_status->tx_product_id_done == true) {
		if (wls_dev->magcvr_status == MAGCVR_STATUS_NEAR)
			qf_val = oplus_chg_wls_third_part_qf_parm_config(wls_dev, qf_type,
				    static_cfg->third_part_epp_tx_qf_parm_magcvr,
				    static_cfg->third_part_epp_qf_parm_magcvr_sets,
				    wls_status->tx_manu_id);
		else
			qf_val = oplus_chg_wls_third_part_qf_parm_config(wls_dev, qf_type,
				    static_cfg->third_part_epp_tx_qf_parm,
				    static_cfg->third_part_epp_qf_parm_sets,
				    wls_status->tx_manu_id);
		if (qf_val)
			return qf_val;
	}

	return 0;
}

#define WLS_FOD_PARM_ID_DEFAULT	0xff
static u8* oplus_chg_wls_get_fod_parm(struct oplus_chg_wls *wls_dev, u8 id, int mode)
{
	int i;
	int fod_parm_sets = 0;
	static u8 fod_parm[WLS_FOD_PARM_LEN_MAX];
	struct wls_fod_parm_type *fod_parm_buf;
	struct oplus_chg_wls_static_config *static_cfg = &wls_dev->static_config;
	size_t buf_size = sizeof(struct wls_fod_parm_type) * WLS_BASE_NUM_MAX;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	bool rc = false;

	memcpy(fod_parm, static_cfg->disable_fod_parm, static_cfg->fod_parm_len);
	fod_parm_buf = kzalloc(buf_size, GFP_KERNEL);
	if (fod_parm_buf == NULL) {
		chg_err("alloc fod_parm_buf memory error\n");
		return fod_parm;
	}

	switch (mode) {
	case FOD_BPP_MODE:
		fod_parm_sets = wls_dev->static_config.bpp_fod_parm_sets;
		if (fod_parm_sets <= 0)
			break;
		if (wls_dev->magcvr_status == MAGCVR_STATUS_NEAR)
			memcpy(fod_parm_buf, wls_dev->static_config.bpp_fod_parm_magcvr, buf_size);
		else
			memcpy(fod_parm_buf, wls_dev->static_config.bpp_fod_parm, buf_size);
		break;
	case FOD_EPP_MODE:
		fod_parm_sets = wls_dev->static_config.epp_fod_parm_sets;
		if (fod_parm_sets <= 0)
			break;
		if (wls_dev->magcvr_status == MAGCVR_STATUS_NEAR)
			memcpy(fod_parm_buf, wls_dev->static_config.epp_fod_parm_magcvr, buf_size);
		else
			memcpy(fod_parm_buf, wls_dev->static_config.epp_fod_parm, buf_size);
		if (wls_status->tx_manu_id != 0 && wls_status->tx_product_id_done == true) {
			if (wls_dev->magcvr_status == MAGCVR_STATUS_NEAR)
				rc = oplus_chg_wls_third_part_fod_parm_config(wls_dev, fod_parm,
					static_cfg->third_part_epp_tx_fod_parm_magcvr,
					static_cfg->third_part_epp_fod_parm_magcvr_sets,
					wls_status->tx_manu_id);
			else
				rc = oplus_chg_wls_third_part_fod_parm_config(wls_dev, fod_parm,
					static_cfg->third_part_epp_tx_fod_parm,
					static_cfg->third_part_epp_fod_parm_sets,
					wls_status->tx_manu_id);
			if (rc == true) {
				kfree(fod_parm_buf);
				return fod_parm;
			}
		}
		break;
	case FOD_FAST_MODE:
		fod_parm_sets = wls_dev->static_config.fastchg_fod_parm_sets;
		if (fod_parm_sets <= 0)
			break;
		if (wls_dev->magcvr_status == MAGCVR_STATUS_NEAR)
			memcpy(fod_parm_buf, wls_dev->static_config.fastchg_fod_parm_magcvr, buf_size);
		else
			memcpy(fod_parm_buf, wls_dev->static_config.fastchg_fod_parm, buf_size);

		if (wls_status->adapter_id == WLS_ADAPTER_THIRD_PARTY && wls_status->tx_product_id_done) {
			if (wls_dev->magcvr_status == MAGCVR_STATUS_NEAR)
				rc = oplus_chg_wls_third_part_fod_parm_config(wls_dev, fod_parm,
					static_cfg->third_part_fastchg_tx_fod_parm_magcvr,
					static_cfg->third_part_fastchg_fod_parm_magcvr_sets,
					wls_status->product_id);
			else
				rc = oplus_chg_wls_third_part_fod_parm_config(wls_dev, fod_parm,
					static_cfg->third_part_fastchg_tx_fod_parm,
					static_cfg->third_part_fastchg_fod_parm_sets,
					wls_status->product_id);
			if (rc == true) {
				kfree(fod_parm_buf);
				return fod_parm;
			}
		}
		break;
	default:
		break;
	}

	for (i = 0; i < fod_parm_sets; i++) {
		if (fod_parm_buf[i].id == WLS_FOD_PARM_ID_DEFAULT) {
			memcpy(fod_parm, fod_parm_buf[i].fod_parm, wls_dev->static_config.fod_parm_len);
			break;
		}
	}
	for (i = 0; i < fod_parm_sets; i++) {
		if (fod_parm_buf[i].id == id) {
			memcpy(fod_parm, fod_parm_buf[i].fod_parm, wls_dev->static_config.fod_parm_len);
			break;
		}
	}

	kfree(fod_parm_buf);
	return fod_parm;
}

static u8* oplus_chg_wls_get_fod_parm_12v(struct oplus_chg_wls *wls_dev, u8 id)
{
	int i;
	static u8 fod_parm_12v[WLS_FOD_PARM_LEN_MAX];

	memcpy(fod_parm_12v, oplus_chg_wls_disable_fod_parm, ARRAY_SIZE(oplus_chg_wls_disable_fod_parm));

	for (i = 0; i < wls_dev->static_config.fastchg_fod_parm_sets; i++) {
		if (wls_dev->static_config.fastchg_fod_parm_12v[i].id == WLS_FOD_PARM_ID_DEFAULT) {
			memcpy(fod_parm_12v, wls_dev->static_config.fastchg_fod_parm_12v[i].fod_parm,
				wls_dev->static_config.fod_parm_len);
			break;
		}
	}

	for (i = 0; i < wls_dev->static_config.fastchg_fod_parm_sets; i++) {
		if (wls_dev->static_config.fastchg_fod_parm_12v[i].id == id) {
			memcpy(fod_parm_12v, wls_dev->static_config.fastchg_fod_parm_12v[i].fod_parm,
				wls_dev->static_config.fod_parm_len);
			break;
		}
	}

	return fod_parm_12v;
}

static int oplus_chg_wls_get_ibat(struct oplus_chg_wls *wls_dev, int *ibat_ma)
{
	union mms_msg_data data = { 0 };

	if (!wls_dev || !wls_dev->gauge_topic) {
		chg_err("wls_dev or gauge_topic is null\n");
		return -ENODEV;
	}
	oplus_mms_get_item_data(wls_dev->gauge_topic, GAUGE_ITEM_CURR, &data, true);
	*ibat_ma = data.intval;
	return 0;
}

static int oplus_chg_wls_get_vbat(struct oplus_chg_wls *wls_dev, int *vbat_mv)
{
	union mms_msg_data data = { 0 };

	if (!wls_dev || !wls_dev->gauge_topic) {
		chg_err("wls_dev or gauge_topic is null\n");
		return -ENODEV;
	}
	oplus_mms_get_item_data(wls_dev->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	*vbat_mv = data.intval;
	return 0;
}

static int oplus_chg_wls_get_real_soc(struct oplus_chg_wls *wls_dev, int *soc)
{
	if (!wls_dev) {
		chg_err("wls_dev is null\n");
		return -ENODEV;
	}
	*soc = wls_dev->batt_info.soc;

	return 0;
}

static int oplus_chg_wls_get_ui_soc(struct oplus_chg_wls *wls_dev, int *ui_soc)
{
	if (!wls_dev) {
		chg_err("wls_dev is null\n");
		return -ENODEV;
	}
	*ui_soc = min(wls_dev->batt_info.ui_soc, 100);

	return 0;
}

static int oplus_chg_wls_get_batt_temp(struct oplus_chg_wls *wls_dev, int *temp)
{
	if (!wls_dev) {
		chg_err("wls_dev is null\n");
		return -ENODEV;
	}
	*temp = wls_dev->batt_info.batt_temp;

	return 0;
}

static int oplus_chg_wls_get_batt_num(struct oplus_chg_wls *wls_dev, int *num)
{
	*num = oplus_gauge_get_batt_num();
	return 0;
}

static int oplus_chg_wls_get_skin_temp(struct oplus_chg_wls *wls_dev, int *temp)
{
	*temp = 250;

	return 0;
}

static int oplus_chg_wls_set_wrx_enable(struct oplus_chg_wls *wls_dev, bool en)
{
	int rc;

	if (wls_dev->support_wls_use_usb_drv) {
		rc = oplus_chg_wls_nor_set_usb_drv(wls_dev->wls_nor->nor_ic, en);
		if (rc < 0)
			chg_err("can't %s wrx, rc=%d\n", en ? "enable" : "disable", rc);
		else
			chg_info("set wrx %s with usb drv\n", en ? "enable" : "disable");

		return rc;
	}

	if (IS_ERR_OR_NULL(wls_dev->pinctrl) ||
	    IS_ERR_OR_NULL(wls_dev->wrx_en_active) ||
	    IS_ERR_OR_NULL(wls_dev->wrx_en_sleep)) {
		chg_info("wrx_en pinctrl error or null\n");
		return -ENODEV;
	}

	rc = pinctrl_select_state(wls_dev->pinctrl,
		en ? wls_dev->wrx_en_active : wls_dev->wrx_en_sleep);
	if (rc < 0)
		chg_err("can't %s wrx, rc=%d\n", en ? "enable" : "disable", rc);
	else
		chg_info("set wrx %s\n", en ? "enable" : "disable");
	chg_info("wrx: set value:%d, gpio_val:%d\n", en, gpio_get_value(wls_dev->wrx_en_gpio));

	return rc;
}

/**********************************************************************
* wls rx funcs:
**********************************************************************/
static int oplus_chg_wls_rx_set_vout_ms(struct oplus_wls_chg_rx *wls_rx, int vol_mv, int wait_time_ms)
{
	int cep;
	int rc;

	if (wls_rx == NULL) {
		chg_err("wls_rx is NULL\n");
		return -ENODEV;
	}

	chg_info("set vout to %d mV\n", vol_mv);
	rc = oplus_chg_wls_rx_set_vout_raw(wls_rx->rx_ic, vol_mv);
	if (rc < 0) {
		chg_err("can't set vout to %dmV, rc=%d\n", vol_mv, rc);
		return rc;
	}
	wls_rx->vol_set_mv = vol_mv;
	wls_rx->cep_is_ok = false;
	(void)oplus_chg_wls_get_cep_check_update(wls_rx->rx_ic, &cep);
	if (wait_time_ms > 0) {
		reinit_completion(&wls_rx->cep_ok_ack);
		schedule_delayed_work(&wls_rx->cep_check_work, msecs_to_jiffies(100));
		rc = wait_for_completion_timeout(&wls_rx->cep_ok_ack, msecs_to_jiffies(wait_time_ms));
		if (!rc) {
			chg_err("wait cep timeout\n");
			cancel_delayed_work_sync(&wls_rx->cep_check_work);
			return -ETIMEDOUT;
		}
		if (wls_rx->clean_source)
			return -EINVAL;
	} else if (wait_time_ms < 0) {
		reinit_completion(&wls_rx->cep_ok_ack);
		schedule_delayed_work(&wls_rx->cep_check_work, msecs_to_jiffies(100));
		wait_for_completion(&wls_rx->cep_ok_ack);
		if (wls_rx->clean_source)
			return -EINVAL;
	}

	return 0;
}

static int oplus_chg_wls_rx_set_vout(struct oplus_wls_chg_rx *wls_rx, int vol_mv, int wait_time_s)
{
	unsigned long wait_time_ms;
	int rc;

	if (wait_time_s > 0)
		wait_time_ms = wait_time_s * 1000;
	else
		wait_time_ms = wait_time_s;

	rc = oplus_chg_wls_rx_set_vout_ms(wls_rx, vol_mv, wait_time_ms);
	if (rc < 0)
		return rc;

	return 0;
}

static int oplus_chg_wls_rx_set_vout_step(struct oplus_wls_chg_rx *wls_rx,
				int target_vol_mv, int step_mv, int wait_time_s)
{
	int next_step_mv;
	unsigned long remaining_time_ms;
	unsigned long stop_time;
	unsigned long start_time = jiffies;
	bool no_timeout;
	int rc;

	if (wls_rx == NULL) {
		chg_err("wls_rx is NULL\n");
		return -ENODEV;
	}

	if (wait_time_s == 0) {
		chg_err("waiting time must be greater than 0\n");
		return -EINVAL;
	} else if (wait_time_s > 0) {
		no_timeout = false;
		remaining_time_ms = wait_time_s * 1000;
		stop_time = msecs_to_jiffies(wait_time_s * 1000) + start_time;
	} else {
		no_timeout = true;
	}

next_step:
	if (target_vol_mv >= wls_rx->vol_set_mv) {
		next_step_mv = wls_rx->vol_set_mv + step_mv;
		if (next_step_mv > target_vol_mv)
			next_step_mv = target_vol_mv;
	} else {
		next_step_mv = wls_rx->vol_set_mv - step_mv;
		if (next_step_mv < target_vol_mv)
			next_step_mv = target_vol_mv;
	}
	rc = oplus_chg_wls_rx_set_vout_ms(wls_rx, next_step_mv, no_timeout ? -1 : remaining_time_ms);
	if (rc < 0) {
		chg_err("can't set vout to %dmV, rc=%d\n", next_step_mv, rc);
		return rc;
	}
	if (next_step_mv != target_vol_mv) {
		if (!no_timeout) {
			if (time_is_before_jiffies(stop_time)) {
				chg_err("can't set vout to %dmV\n", next_step_mv);
				return -ETIMEDOUT;
			}
			remaining_time_ms = remaining_time_ms - jiffies_to_msecs(jiffies - start_time);
			start_time = jiffies;
		}
		goto next_step;
	}

	return 0;
}

static void oplus_chg_wls_cep_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_wls_chg_rx *wls_rx = container_of(dwork, struct oplus_wls_chg_rx, cep_check_work);
	int rc;
	int cep;

	if (wls_rx == NULL) {
		chg_err("wls_rx is NULL\n");
		return;
	}

	if (!wls_rx->wls_dev->wls_status.rx_online) {
		chg_info("rx is offline, exit\n");
		complete(&wls_rx->cep_ok_ack);
		return;
	}

	rc = oplus_chg_wls_get_cep_check_update(wls_rx->rx_ic, &cep);
	if (rc < 0)
		goto out;
	chg_info("wkcs: cep=%d\n", cep);
	if (abs(cep) < 3) {
		wls_rx->cep_is_ok = true;
		complete(&wls_rx->cep_ok_ack);
		return;
	}

out:
	schedule_delayed_work(&wls_rx->cep_check_work, msecs_to_jiffies(100));
}

static int oplus_chg_wls_rx_send_msg(struct oplus_wls_chg_rx *wls_rx, unsigned char msg, unsigned char data)
{
	unsigned char buf[4] = {msg, ~msg, data, ~data};
	int rc;

	if (wls_rx == NULL) {
		chg_err("wls_rx is NULL\n");
		return -ENODEV;
	}

	if (wls_rx->wls_dev->wls_status.rx_present == false) {
		chg_info("rx_present=%d, should not send msg\n", wls_rx->wls_dev->wls_status.rx_present);
		return 0;
	}

	chg_info("send msg, msg=0x%02x, data=0x%02x\n", msg, data);
	rc = oplus_chg_wls_rx_send_msg_raw(wls_rx->rx_ic, buf, ARRAY_SIZE(buf), 0);
	if (rc < 0)
		chg_err("can't send msg, rc=%d\n", rc);

	return rc;
}

/**
 * oplus_chg_wls_rx_send_data()
 *
 * @wls_rx:	the wls_rx object
 * @msg[2]:	msg:msg[0]:msg_head, msg[1]:msg_type
 * @data[]:	data
 * @data_len:	data length
 * @raw_data:	raw data will send msg_head and msg_type
 *
 * Returns: return > 0 if send data success. return < 0 otherwise.
 */
static int oplus_chg_wls_rx_send_data(struct oplus_wls_chg_rx *wls_rx, unsigned char *msg, unsigned char data[], int data_len, int raw_data)
{
	unsigned char buf[5] = {0};
	int rc;
	int msg_len = 0;

	if (wls_rx == NULL) {
		chg_err("wls_rx is NULL\n");
		return -ENODEV;
	}

	if (raw_data)
		msg_len = 2;
	else
		msg_len = 1;
	/*msg[0]:msg_head, msg[1]:msg_type, raw data should send msg_head and msg_type*/
	if (raw_data) {
		buf[0] = msg[0];
		buf[1] = msg[1];
	} else {
		buf[0] = msg[1];
	}

	memcpy(buf + msg_len, data, data_len);
	chg_info("send msg, msg=0x%02x%02x, data=0x%02x 0x%02x 0x%02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
	rc = oplus_chg_wls_rx_send_msg_raw(wls_rx->rx_ic, buf, data_len + msg_len, raw_data);

	return rc;
}

#define WLS_Q_VALUE_ERROR_MAX_POWER_W	20
#define WLS_Q_VALUE_TIMEOUT_S		10
static int oplus_chg_wls_rx_send_match_q_response(struct oplus_chg_wls *wls_dev, u8 data_buf[])
{
	int rc = 0;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	u8 buf[5] = {0x38, 0x48, data_buf[0], data_buf[1], 0};
	int retry = 0;
	unsigned long q_val_wait = jiffies + msecs_to_jiffies(WLS_Q_VALUE_TIMEOUT_S * 1000);

	if (wls_dev->wls_status.q_val_done == true) {
		chg_info("q value already done\n");
		return 0;
	}

	do {
		rc = oplus_chg_wls_send_raw_data(wls_dev, &buf[0], &buf[2], WLS_Q_VALUE_TIMEOUT_S);
		if (rc == -ETIMEDOUT) {
			chg_err("send match q value timeout\n");
			vote(wls_dev->fcc_votable, WLS_Q_VALUE_ERROR_VOTER, true,
				(WLS_Q_VALUE_ERROR_MAX_POWER_W * 1000 * 1000 / dynamic_cfg->svooc_vol_mv), false);
			break;
		} else if (wls_dev->wls_status.tx_q_val_frm_rx != data_buf[1] && rc >= 0) {
			retry++;
			rc = -1;
			chg_info("q_value:0x%x != tx_q_val_frm_rx:0x%x, retry:%d\n",
				data_buf[1], wls_dev->wls_status.tx_q_val_frm_rx, retry);
		} else if (rc == -EAGAIN) {
			chg_info("msg pending, retry\n");
			msleep(100);
		}

		if (time_after(jiffies, q_val_wait) || retry > 10) {
			chg_err("send match q value error, rc:%d, retry:%d\n", rc, retry);
			vote(wls_dev->fcc_votable, WLS_Q_VALUE_ERROR_VOTER, true,
				(WLS_Q_VALUE_ERROR_MAX_POWER_W * 1000 * 1000 / dynamic_cfg->svooc_vol_mv), false);
			break;
		}
	} while (rc < 0 && wls_dev->wls_status.rx_online);

	wls_dev->wls_status.q_val_done = true;
	return rc;
}

static void oplus_chg_wls_publish_fw_upgrading_msg(struct oplus_chg_wls *wls_dev, bool fw_upgrading)
{
	struct mms_msg *msg;
	int rc;

	if (wls_dev == NULL || wls_dev->wls_topic == NULL) {
		chg_err("wls_topic not ready\n");
		return;
	}

	if (wls_dev->wls_status.fw_upgrading == fw_upgrading)
		return;

	wls_dev->wls_status.fw_upgrading = fw_upgrading;
	chg_info("set fw_upgrading=%s\n", fw_upgrading ? "true" : "false");
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, WLS_ITEM_FW_UPGRADING);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(wls_dev->wls_topic, msg);
	if (rc < 0) {
		chg_err("publish wls fw_upgrading msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static int oplus_chg_wls_rx_upgrade_firmware_by_buf(struct oplus_wls_chg_rx *wls_rx, unsigned char *buf, int len)
{
	int rc;

	if (wls_rx == NULL) {
		chg_err("wls_rx is NULL\n");
		return -ENODEV;
	}

	__pm_stay_awake(wls_rx->update_fw_wake_lock);
	oplus_chg_wls_publish_fw_upgrading_msg(wls_rx->wls_dev, true);
	rc = oplus_chg_wls_rx_upgrade_firmware_by_buf_raw(wls_rx->rx_ic, buf, len);
	if (rc < 0)
		chg_err("can't upgrade firmware by buf, rc=%d\n", rc);
	oplus_chg_wls_publish_fw_upgrading_msg(wls_rx->wls_dev, false);
	__pm_relax(wls_rx->update_fw_wake_lock);

	return rc;
}

static int oplus_chg_wls_rx_upgrade_firmware_by_img(struct oplus_wls_chg_rx *wls_rx)
{
	int rc;

	if (wls_rx == NULL) {
		chg_err("wls_rx is NULL\n");
		return -ENODEV;
	}

	__pm_stay_awake(wls_rx->update_fw_wake_lock);
	oplus_chg_wls_publish_fw_upgrading_msg(wls_rx->wls_dev, true);
	rc = oplus_chg_wls_rx_upgrade_firmware_by_img_raw(wls_rx->rx_ic);
	if (rc < 0)
		chg_err("can't upgrade firmware by img, rc=%d\n", rc);
	oplus_chg_wls_publish_fw_upgrading_msg(wls_rx->wls_dev, false);
	__pm_relax(wls_rx->update_fw_wake_lock);

	return rc;
}

static int oplus_chg_wls_rx_clean_source(struct oplus_wls_chg_rx *wls_rx)
{
	if (wls_rx == NULL) {
		chg_err("wls_rx is NULL\n");
		return -ENODEV;
	}

	wls_rx->clean_source = true;
	cancel_delayed_work_sync(&wls_rx->cep_check_work);
	complete(&wls_rx->cep_ok_ack);
	wls_rx->cep_is_ok = true;
	wls_rx->vol_set_mv = 5000;
	wls_rx->clean_source = false;

	return 0;
}

static int oplus_chg_wls_rx_init(struct oplus_chg_wls *wls_dev)
{
	struct oplus_wls_chg_rx *wls_rx;

	wls_rx = devm_kzalloc(wls_dev->dev, sizeof(struct oplus_wls_chg_rx), GFP_KERNEL);
	if (wls_rx == NULL) {
		chg_err("alloc memory error\n");
		devm_kfree(wls_dev->dev, wls_rx);
		return -ENOMEM;
	}

	wls_dev->wls_rx = wls_rx;
	wls_rx->dev = wls_dev->dev;
	wls_rx->wls_dev = wls_dev;

	INIT_DELAYED_WORK(&wls_rx->cep_check_work, oplus_chg_wls_cep_check_work);
	init_completion(&wls_rx->cep_ok_ack);
	wls_rx->update_fw_wake_lock = wakeup_source_register(wls_rx->dev, "wls_update_lock");

	return 0;
}

static int oplus_chg_wls_rx_remove(struct oplus_chg_wls *wls_dev)
{
	struct oplus_wls_chg_rx *wls_rx = wls_dev->wls_rx;

	devm_kfree(wls_dev->dev, wls_rx);
	wls_dev->wls_rx = NULL;

	return 0;
}

/**********************************************************************
* wls nor funcs:
**********************************************************************/
static int oplus_chg_wls_nor_set_icl(struct oplus_wls_chg_normal *wls_nor, int icl_ma)
{
	int rc;

	if (wls_nor == NULL) {
		chg_err("wls_nor is NULL\n");
		return -ENODEV;
	}

	chg_info("set icl to %d ma\n", icl_ma);

	mutex_lock(&wls_nor->icl_lock);
	rc = oplus_chg_wls_nor_set_icl_raw(wls_nor->nor_ic, icl_ma);
	if (rc < 0) {
		chg_err("set icl to %d mA error, rc=%d\n", icl_ma, rc);
		mutex_unlock(&wls_nor->icl_lock);
		return rc;
	}

	wls_nor->icl_set_ma = icl_ma;
	mutex_unlock(&wls_nor->icl_lock);

	return 0;
}

static void oplus_chg_wls_nor_icl_set_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_wls_chg_normal *wls_nor = container_of(dwork, struct oplus_wls_chg_normal, icl_set_work);
	int icl_next_ma;
	int rc;

	if (!wls_nor->wls_dev->wls_status.rx_present)
		return;
	if (wls_nor->clean_source)
		return;

	mutex_lock(&wls_nor->icl_lock);
	if (wls_nor->icl_target_ma >= wls_nor->icl_set_ma) {
		icl_next_ma = wls_nor->icl_set_ma + wls_nor->icl_step_ma;
		if (icl_next_ma > wls_nor->icl_target_ma)
			icl_next_ma = wls_nor->icl_target_ma;
	} else {
		icl_next_ma = wls_nor->icl_set_ma - wls_nor->icl_step_ma;
		if (icl_next_ma < wls_nor->icl_target_ma)
			icl_next_ma = wls_nor->icl_target_ma;
	}
	mutex_unlock(&wls_nor->icl_lock);
	rc = oplus_chg_wls_nor_set_icl(wls_nor, icl_next_ma);
	if (rc < 0) {
		chg_err("set icl to %d mA error, rc=%d\n", icl_next_ma, rc);
		return;
	}
	if (icl_next_ma != wls_nor->icl_target_ma) {
		if (!wls_nor->wls_dev->wls_status.rx_present)
			return;
		schedule_delayed_work(&wls_nor->icl_set_work, msecs_to_jiffies(100));
		return;
	}
}

static int oplus_chg_wls_nor_set_icl_by_step(struct oplus_wls_chg_normal *wls_nor, int icl_ma, int step_ma, bool block)
{
	int icl_next_ma;
	int rc;

	if (wls_nor == NULL) {
		chg_err("wls_nor is NULL\n");
		return -ENODEV;
	}

	if (block) {
		mutex_lock(&wls_nor->icl_lock);
		wls_nor->icl_target_ma = icl_ma;
		wls_nor->icl_step_ma = step_ma;
		mutex_unlock(&wls_nor->icl_lock);
next_step:
		mutex_lock(&wls_nor->icl_lock);
		if (icl_ma >= wls_nor->icl_set_ma) {
			icl_next_ma = wls_nor->icl_set_ma + step_ma;
			if (icl_next_ma > icl_ma)
				icl_next_ma = icl_ma;
		} else {
			icl_next_ma = wls_nor->icl_set_ma - step_ma;
			if (icl_next_ma < icl_ma)
				icl_next_ma = icl_ma;
		}
		mutex_unlock(&wls_nor->icl_lock);
		rc = oplus_chg_wls_nor_set_icl(wls_nor, icl_next_ma);
		if (rc < 0) {
			chg_err("set icl to %d mA error, rc=%d\n", icl_next_ma, rc);
			return rc;
		}
		if (wls_nor->clean_source)
			return -EINVAL;
		if (icl_next_ma != icl_ma) {
			msleep(100);
			goto next_step;
		}
	} else {
		mutex_lock(&wls_nor->icl_lock);
		wls_nor->icl_target_ma = icl_ma;
		wls_nor->icl_step_ma = step_ma;
		mutex_unlock(&wls_nor->icl_lock);
		schedule_delayed_work(&wls_nor->icl_set_work, 0);
	}

	return 0;
}

static int oplus_chg_wls_nor_set_fcc(struct oplus_wls_chg_normal *wls_nor, int fcc_ma)
{
	int rc;

	if (wls_nor == NULL) {
		chg_err("wls_nor is NULL\n");
		return -ENODEV;
	}
	if (oplus_chg_wls_is_usb_present(wls_nor->wls_dev)) {
		chg_debug("usb present, exit\n");
		return -EPERM;
	}

	mutex_lock(&wls_nor->fcc_lock);
	rc = oplus_chg_wls_nor_set_fcc_raw(wls_nor->nor_ic, fcc_ma);
	if (rc < 0) {
		chg_err("set fcc to %d mA error, rc=%d\n", fcc_ma, rc);
		mutex_unlock(&wls_nor->fcc_lock);
		return rc;
	} else {
		chg_info("set fcc to %d mA\n", fcc_ma);
	}
	wls_nor->fcc_set_ma = fcc_ma;
	mutex_unlock(&wls_nor->fcc_lock);

	return 0;
}

static void oplus_chg_wls_nor_fcc_set_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_wls_chg_normal *wls_nor = container_of(dwork, struct oplus_wls_chg_normal, fcc_set_work);
	int fcc_next_ma;
	int rc;

	if (!wls_nor->wls_dev->wls_status.rx_present)
		return;
	if (wls_nor->clean_source)
		return;

	mutex_lock(&wls_nor->fcc_lock);
	if (wls_nor->fcc_target_ma >= wls_nor->fcc_set_ma) {
		fcc_next_ma = wls_nor->fcc_set_ma + wls_nor->fcc_step_ma;
		if (fcc_next_ma > wls_nor->fcc_target_ma)
			fcc_next_ma = wls_nor->fcc_target_ma;
	} else {
		fcc_next_ma = wls_nor->fcc_set_ma - wls_nor->fcc_step_ma;
		if (fcc_next_ma < wls_nor->fcc_target_ma)
			fcc_next_ma = wls_nor->fcc_target_ma;
	}
	mutex_unlock(&wls_nor->fcc_lock);
	rc = oplus_chg_wls_nor_set_fcc(wls_nor, fcc_next_ma);
	if (rc < 0) {
		chg_err("set fcc to %d mA error, rc=%d\n", fcc_next_ma, rc);
		return;
	}
	if (fcc_next_ma != wls_nor->fcc_target_ma) {
		if (!wls_nor->wls_dev->wls_status.rx_present)
			return;
		schedule_delayed_work(&wls_nor->fcc_set_work, msecs_to_jiffies(100));
		return;
	}
}

static int oplus_chg_wls_nor_set_fcc_by_step(struct oplus_wls_chg_normal *wls_nor, int fcc_ma, int step_ma, bool block)
{
	int fcc_next_ma;
	int rc;

	if (wls_nor == NULL) {
		chg_err("wls_nor is NULL\n");
		return -ENODEV;
	}

	if (block) {
		mutex_lock(&wls_nor->fcc_lock);
		wls_nor->fcc_target_ma = fcc_ma;
		wls_nor->fcc_step_ma = step_ma;
		mutex_unlock(&wls_nor->fcc_lock);
next_step:
		mutex_lock(&wls_nor->fcc_lock);
		if (fcc_ma >= wls_nor->fcc_set_ma) {
			fcc_next_ma = wls_nor->fcc_set_ma + step_ma;
			if (fcc_next_ma > fcc_ma)
				fcc_next_ma = fcc_ma;
		} else {
			fcc_next_ma = wls_nor->fcc_set_ma - step_ma;
			if (fcc_next_ma < fcc_ma)
				fcc_next_ma = fcc_ma;
		}
		mutex_unlock(&wls_nor->fcc_lock);
		rc = oplus_chg_wls_nor_set_fcc(wls_nor, fcc_next_ma);
		if (rc < 0) {
			chg_err("set fcc to %d mA error, rc=%d\n", fcc_next_ma, rc);
			return rc;
		}
		if (wls_nor->clean_source)
			return -EINVAL;
		if (fcc_next_ma != fcc_ma) {
			msleep(100);
			goto next_step;
		}
	} else {
		mutex_lock(&wls_nor->fcc_lock);
		wls_nor->fcc_target_ma = fcc_ma;
		wls_nor->fcc_step_ma = step_ma;
		mutex_unlock(&wls_nor->fcc_lock);
		schedule_delayed_work(&wls_nor->fcc_set_work, 0);
	}

	return 0;
}

static int oplus_chg_wls_nor_clean_source(struct oplus_wls_chg_normal *wls_nor)
{
	if (wls_nor == NULL) {
		chg_err("wls_nor is NULL\n");
		return -ENODEV;
	}

	wls_nor->clean_source = true;
	cancel_delayed_work_sync(&wls_nor->icl_set_work);
	cancel_delayed_work_sync(&wls_nor->fcc_set_work);
	mutex_lock(&wls_nor->icl_lock);
	wls_nor->icl_target_ma = 0;
	wls_nor->icl_step_ma = 0;
	wls_nor->icl_set_ma = 0;
	mutex_unlock(&wls_nor->icl_lock);
	mutex_lock(&wls_nor->fcc_lock);
	wls_nor->fcc_target_ma = 0;
	wls_nor->fcc_step_ma = 0;
	wls_nor->fcc_set_ma = 0;
	mutex_unlock(&wls_nor->fcc_lock);
	wls_nor->clean_source = false;

	return 0;
}

static int oplus_chg_wls_nor_init(struct oplus_chg_wls *wls_dev)
{
	struct oplus_wls_chg_normal *wls_nor;

	wls_nor = devm_kzalloc(wls_dev->dev, sizeof(struct oplus_wls_chg_normal), GFP_KERNEL);
	if (wls_nor == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	wls_dev->wls_nor = wls_nor;
	wls_nor->dev = wls_dev->dev;
	wls_nor->wls_dev = wls_dev;

	INIT_DELAYED_WORK(&wls_nor->icl_set_work, oplus_chg_wls_nor_icl_set_work);
	INIT_DELAYED_WORK(&wls_nor->fcc_set_work, oplus_chg_wls_nor_fcc_set_work);
	mutex_init(&wls_nor->icl_lock);
	mutex_init(&wls_nor->fcc_lock);

	return 0;
}

static int oplus_chg_wls_nor_remove(struct oplus_chg_wls *wls_dev)
{
	struct oplus_wls_chg_normal *wls_nor = wls_dev->wls_nor;

	devm_kfree(wls_dev->dev, wls_nor);
	wls_dev->wls_nor = NULL;

	return 0;
}

/**********************************************************************
* wls fast funcs:
**********************************************************************/
static int oplus_chg_wls_fast_init(struct oplus_chg_wls *wls_dev)
{
	struct oplus_wls_chg_fast *wls_fast;

	wls_fast = devm_kzalloc(wls_dev->dev, sizeof(struct oplus_wls_chg_fast), GFP_KERNEL);
	if (wls_fast == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	wls_dev->wls_fast = wls_fast;
	wls_fast->dev = wls_dev->dev;
	wls_fast->wls_dev = wls_dev;

	return 0;
}

static int oplus_chg_wls_fast_remove(struct oplus_chg_wls *wls_dev)
{
	struct oplus_wls_chg_fast *wls_fast = wls_dev->wls_fast;

	devm_kfree(wls_dev->dev, wls_fast);
	wls_dev->wls_fast = NULL;

	return 0;
}

static int oplus_chg_wls_fcc_vote_callback(struct votable *votable, void *data,
				int fcc_ma, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;

	if (fcc_ma < 0)
		return 0;

	if (fcc_ma > dynamic_cfg->fastchg_curr_max_ma)
		fcc_ma = dynamic_cfg->fastchg_curr_max_ma;

#ifdef WLS_QI_DEBUG
	if (wls_dbg_fcc_ma != 0)
		fcc_ma = wls_dbg_fcc_ma;
#endif

	wls_status->fastchg_target_curr_ma = fcc_ma;
	chg_info("set target current to %d by %s\n", wls_status->fastchg_target_curr_ma, client);

	return 0;
}

static int oplus_chg_wls_fastchg_disable_vote_callback(struct votable *votable, void *data,
				int disable, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	wls_status->fastchg_disable = disable;
	chg_info("%s wireless fast charge by %s\n", disable ? "disable" : "enable", client);

	return 0;
}

static int oplus_chg_wls_wrx_en_vote_callback(struct votable *votable, void *data,
				int en, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;
	int rc;

	rc = oplus_chg_wls_set_wrx_enable(wls_dev, en);
	chg_info("%s wireless wrx_en by %s\n", en ? "enable" : "disable", client);

	return rc;
}

static int oplus_chg_wls_nor_icl_vote_callback(struct votable *votable, void *data,
				int icl_ma, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;
	int rc;

	if (icl_ma < 0)
		return 0;

	chg_info("wireless nor icl[%d] charge by %s\n", icl_ma, client);
	if (step)
		rc = oplus_chg_wls_nor_set_icl_by_step(wls_dev->wls_nor, icl_ma, 200, false);
	else
		rc = oplus_chg_wls_nor_set_icl(wls_dev->wls_nor, icl_ma);

	return rc;
}

static int oplus_chg_wls_nor_fcc_vote_callback(struct votable *votable, void *data,
				int fcc_ma, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;
	int rc;

	if (fcc_ma < 0)
		return 0;

	chg_info("wireless nor fcc[%d] charge by %s\n", fcc_ma, client);
	if (step)
		rc = oplus_chg_wls_nor_set_fcc_by_step(wls_dev->wls_nor, fcc_ma, 100, false);
	else
		rc = oplus_chg_wls_nor_set_fcc(wls_dev->wls_nor, fcc_ma);

	return rc;
}

__maybe_unused static int oplus_chg_wls_nor_fv_vote_callback(struct votable *votable, void *data,
				int fv_mv, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;
	int rc;

	if (fv_mv < 0)
		return 0;

	rc = oplus_chg_wls_nor_set_fv(wls_dev->wls_nor->nor_ic, fv_mv);

	return rc;
}

static int oplus_chg_wls_comu_vote_callback(struct votable *votable, void *data,
				int comu, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;

	if (comu < 0 || comu > WLS_RX_COMU_CAP_AB)
		return -EINVAL;

	oplus_chg_wls_rx_set_comu(wls_dev->wls_rx->rx_ic, comu);
	chg_info("set rx comu %d, by %s\n", comu, client);

	return 0;
}

static int oplus_chg_wls_force_bpp_mode_vote_callback(struct votable *votable, void *data,
				int bpp_mode_set, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;

	if (!!bpp_mode_set)
		wls_dev->force_rx_mode = OPLUS_CHG_WLS_RX_MODE_BPP;
	else
		wls_dev->force_rx_mode = OPLUS_CHG_WLS_RX_MODE_UNKNOWN;

	if (wls_dev->mms_info.rx_present == 1)
		oplus_chg_wls_entry_rx_mode(wls_dev, wls_dev->force_rx_mode, client);

	chg_info("rx_present: %d, bpp_mode_set %d, by %s\n", wls_dev->mms_info.rx_present, bpp_mode_set, client);

	return 0;
}

static void  oplus_chg_wls_set_charging_disable(
		struct oplus_chg_wls *wls_dev, bool disable)
{
	int rc;
	struct mms_msg *msg;

	if (wls_dev->charging_disable == disable)
		return;

	wls_dev->charging_disable = disable;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  WLS_ITEM_CHARGING_DISABLE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(wls_dev->wls_topic, msg);
	if (rc < 0) {
		chg_err("publish charging disable error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("charging_disable=0x%x\n", wls_dev->charging_disable);
}

static int oplus_chg_wls_nor_out_disable_vote_callback(struct votable *votable,
			void *data, int disable, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;
	int rc;

	oplus_chg_wls_set_charging_disable(wls_dev, disable);

	rc = oplus_chg_wls_nor_set_output_enable(wls_dev->wls_nor->nor_ic, !disable);
	if (rc < 0)
		chg_err("can't %s wireless nor out charge\n", disable ? "disable" : "enable");
	else
		chg_info("%s wireless nor out charge by %s\n", disable ? "disable" : "enable", client);

	return rc;
}

static int oplus_chg_wls_nor_input_disable_vote_callback(struct votable *votable,
			void *data, int disable, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;
	int rc;

	rc = oplus_chg_wls_nor_set_input_enable(wls_dev->wls_nor->nor_ic, !disable);
	if (rc < 0)
		chg_err("can't %s wireless nor input charge\n", disable ? "disable" : "enable");
	else
		chg_info("%s wireless nor input charge by %s\n", disable ? "disable" : "enable", client);

	return rc;
}

static int oplus_chg_wls_rx_disable_vote_callback(struct votable *votable, void *data,
				int disable, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;
	int rc;

	rc = oplus_chg_wls_rx_enable(wls_dev->wls_rx->rx_ic, !disable);
	if (rc < 0)
		chg_err("can't %s wireless charge\n", disable ? "disable" : "enable");
	else
		chg_info("%s wireless charge by %s\n", disable ? "disable" : "enable", client);

	return rc;
}

static int oplus_chg_wls_insert_disable_vote_callback(struct votable *votable, void *data,
				int disable, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;
	int rc;

	rc = oplus_chg_wls_rx_set_insert_disable(wls_dev->wls_rx->rx_ic, disable);
	if (rc == -ENOTSUPP)
		chg_info("not support insert disable\n");
	else if (rc < 0)
		chg_err("can't %s insert path\n", disable ? "disable" : "enable");
	else
		chg_info("%s insert path by %s\n", disable ? "disable" : "enable", client);

	return rc;
}

static int oplus_chg_wls_quiet_mode_vote_callback(struct votable *votable, void *data,
						  int enable, const char *client, bool step)
{
	struct oplus_chg_wls *wls_dev = data;

	wls_dev->wls_status.switch_quiet_mode = !!enable;
	if (!wls_dev->wls_status.rx_present ||
	    wls_dev->wls_status.adapter_type == WLS_ADAPTER_TYPE_USB ||
	    wls_dev->wls_status.adapter_type == WLS_ADAPTER_TYPE_NORMAL ||
	    wls_dev->wls_status.adapter_type == WLS_ADAPTER_TYPE_UNKNOWN) {
		chg_err("wls not present or isn't op_tx, can't enter quiet mode!\n");
	} else {
		if (wls_dev->wls_status.switch_quiet_mode != wls_dev->wls_status.quiet_mode ||
		    wls_dev->wls_status.quiet_mode_init == false) {
			cancel_delayed_work(&wls_dev->wls_rx_sm_work);
			queue_delayed_work(wls_dev->wls_wq, &wls_dev->wls_rx_sm_work, 0);
		}
	}
	chg_info("%s quiet_mode by %s\n", enable ? "enable" : "disable", client);
	return 0;
}

static int oplus_chg_wls_set_ext_pwr_enable(struct oplus_chg_wls *wls_dev, bool en)
{
	int rc;

	if (IS_ERR_OR_NULL(wls_dev->pinctrl) ||
	    IS_ERR_OR_NULL(wls_dev->ext_pwr_en_active) ||
	    IS_ERR_OR_NULL(wls_dev->ext_pwr_en_sleep)) {
		chg_err("ext_pwr_en pinctrl error\n");
		return -ENODEV;
	}

	rc = pinctrl_select_state(wls_dev->pinctrl,
		en ? wls_dev->ext_pwr_en_active : wls_dev->ext_pwr_en_sleep);
	if (rc < 0)
		chg_err("can't %s ext_pwr, rc=%d\n", en ? "enable" : "disable", rc);
	else
		chg_info("set ext_pwr %s\n", en ? "enable" : "disable");

	chg_info("ext_pwr: set value:%d, gpio_val:%d\n", en, gpio_get_value(wls_dev->ext_pwr_en_gpio));

	return rc;
}

static void oplus_chg_wls_get_third_part_verity_data_work(
				struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev =
		container_of(dwork, struct oplus_chg_wls, wls_get_third_part_verity_data_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	oplus_chg_set_mutual_cmd(CMD_WLS_THIRD_PART_AUTH,
		sizeof(wls_status->vendor_id), &(wls_status->vendor_id));
}

#define OPLUS_TX_CUSTOM_ID 0x11
static void oplus_wls_tx_aes_verity_init(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (wls_status->verify_by_aes)
		return;

	if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30 &&
	    wls_status->adapter_id >= WLS_ADAPTER_MODEL_4 &&
	    wls_status->adapter_id <= WLS_ADAPTER_MODEL_25) {
		wls_status->vendor_id = OPLUS_TX_CUSTOM_ID;
		wls_status->aes_verity_data_ok = false;
		if (!wls_status->verify_by_aes)
			schedule_delayed_work(&wls_dev->wls_get_third_part_verity_data_work, 0);
		wls_status->verify_by_aes = true;
		chg_info("verify_by_aes=%d, vendor_id=0x%02x\n",
		         wls_status->verify_by_aes, wls_status->vendor_id);
	}

	return;
}

static void oplus_chg_wls_standard_msg_handler(struct oplus_chg_wls *wls_dev,
					       u8 mask, u8 data)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_rx_msg *rx_msg = &wls_dev->rx_msg;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	bool msg_ok = false;
	int pwr_mw;

	chg_info("mask=0x%02x, msg_type=0x%02x, data=0x%02x\n", mask, rx_msg->msg_type, data);
	switch (mask) {
	case WLS_RESPONE_EXTERN_CMD:
		if (rx_msg->msg_type == WLS_CMD_GET_EXTERN_CMD) {
			chg_info("tx extern cmd=0x%02x\n", data);
			wls_status->tx_extern_cmd_done = true;
		}
		break;
	case WLS_RESPONE_VENDOR_ID:
		if (rx_msg->msg_type == WLS_CMD_GET_VENDOR_ID) {
			wls_status->vendor_id = data;
			wls_status->aes_verity_data_ok = false;
			if (!wls_status->verify_by_aes)
				schedule_delayed_work(&wls_dev->wls_get_third_part_verity_data_work, 0);
			wls_status->verify_by_aes = true;
			chg_info("verify_by_aes=%d, vendor_id=0x%02x\n",
				 wls_status->verify_by_aes, wls_status->vendor_id);
		}
		break;
	case WLS_RESPONE_ADAPTER_TYPE:
		if (rx_msg->msg_type == WLS_CMD_INDENTIFY_ADAPTER) {
			wls_status->adapter_type = data & WLS_ADAPTER_TYPE_MASK;
			wls_status->adapter_id = (data & WLS_ADAPTER_ID_MASK) >> 3;
			chg_info("wkcs: adapter_id = %d\n", wls_status->adapter_id);
			chg_info("wkcs: adapter_type = 0x%02x\n", wls_status->adapter_type);
			pwr_mw = oplus_chg_wls_get_base_power_max(wls_status->adapter_id);
			vote(wls_dev->fcc_votable, BASE_MAX_VOTER, true, (pwr_mw * 1000 / dynamic_cfg->svooc_vol_mv), false);
		}
		oplus_wls_tx_aes_verity_init(wls_dev);
		break;
	case WLS_RESPONE_INTO_FASTCHAGE:
		if (rx_msg->msg_type == WLS_CMD_INTO_FASTCHAGE) {
			wls_status->adapter_power = data;
			if (wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY) {
				wls_status->pwr_max_mw = oplus_chg_wls_get_r_power(wls_dev, wls_status->adapter_power);
				(void)oplus_wls_adapter_curve_runing(wls_dev);
			} else {
				wls_status->pwr_max_mw = oplus_chg_wls_get_tripartite_r_power(wls_status->adapter_power);
			}
			if (wls_status->pwr_max_mw > 0) {
				vote(wls_dev->fcc_votable, RX_MAX_VOTER, true,
				     (wls_status->pwr_max_mw * 1000 / dynamic_cfg->svooc_vol_mv),
				     false);
			}
			wls_status->charge_type = WLS_CHARGE_TYPE_FAST;
			if (wls_dev->static_config.fastchg_fod_enable) {
				if (wls_status->fod_parm_for_fastchg)
					(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
						oplus_chg_wls_get_fod_parm(wls_dev, wls_status->adapter_id, FOD_FAST_MODE),
						wls_dev->static_config.fod_parm_len,
						FOD_FAST_MODE, wls_dev->magcvr_status);
				else if (wls_dev->static_config.fastchg_12v_fod_enable)
					(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
						oplus_chg_wls_get_fod_parm_12v(wls_dev, wls_status->adapter_id),
						wls_dev->static_config.fod_parm_len,
						FOD_FAST_MODE, wls_dev->magcvr_status);
			} else {
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
					FOD_FAST_MODE, wls_dev->magcvr_status);
			}
			if (!wls_status->verity_started)
				vote(wls_dev->fcc_votable, VERITY_VOTER, true, dynamic_cfg->verity_curr_max_ma, false);
			oplus_chg_wls_publish_wls_type(wls_dev, true, false);
		}
		break;
	case WLS_RESPONE_INTO_NORMAL_MODE:
		if (rx_msg->msg_type == WLS_CMD_SET_NORMAL_MODE) {
			wls_status->quiet_mode = false;
			wls_status->quiet_mode_init = true;
		}
		break;
	case WLS_RESPONE_INTO_QUIET_MODE:
		if (rx_msg->msg_type == WLS_CMD_SET_QUIET_MODE) {
			wls_status->quiet_mode = true;
			wls_status->quiet_mode_init = true;
		}
		break;
	case WLS_RESPONE_FAN_SPEED:
		if (rx_msg->msg_type == WLS_CMD_SET_FAN_SPEED) {
			if (data <= FAN_PWM_PULSE_IN_SILENT_MODE_THR)
				wls_status->quiet_mode = true;
			else
				wls_status->quiet_mode = false;
			wls_status->quiet_mode_init = true;
		}
		break;
	case WLS_RESPONE_CEP_TIMEOUT:
		if (rx_msg->msg_type == WLS_CMD_SET_CEP_TIMEOUT) {
			wls_status->cep_timeout_adjusted = true;
		}
		break;
	case WLS_RESPONE_READY_FOR_EPP:
		wls_status->adapter_type = WLS_ADAPTER_TYPE_EPP;
		break;
	case WLS_RESPONE_WORKING_IN_EPP:
		wls_status->epp_working = true;
		break;
	default:
		break;
	}

	mutex_lock(&wls_dev->send_msg_lock);
	switch (mask) {
	case WLS_RESPONE_ADAPTER_TYPE:
		if (rx_msg->msg_type == WLS_CMD_INDENTIFY_ADAPTER)
			msg_ok = true;
		break;
	case WLS_RESPONE_INTO_FASTCHAGE:
		if (rx_msg->msg_type == WLS_CMD_INTO_FASTCHAGE)
			msg_ok = true;
		break;
	case WLS_RESPONE_INTO_NORMAL_MODE:
		if (rx_msg->msg_type == WLS_CMD_SET_NORMAL_MODE)
			msg_ok = true;
		break;
	case WLS_RESPONE_INTO_QUIET_MODE:
		if (rx_msg->msg_type == WLS_CMD_SET_QUIET_MODE)
			msg_ok = true;
		break;
	case WLS_RESPONE_CEP_TIMEOUT:
		if (rx_msg->msg_type == WLS_CMD_SET_CEP_TIMEOUT)
			msg_ok = true;
		break;
	case WLS_RESPONE_LED_BRIGHTNESS:
		if (rx_msg->msg_type == WLS_CMD_SET_LED_BRIGHTNESS)
			msg_ok = true;
		break;
	case WLS_RESPONE_FAN_SPEED:
		if (rx_msg->msg_type == WLS_CMD_SET_FAN_SPEED)
			msg_ok = true;
		break;
	case WLS_RESPONE_VENDOR_ID:
		if (rx_msg->msg_type == WLS_CMD_GET_VENDOR_ID)
			msg_ok = true;
		break;
	case WLS_RESPONE_EXTERN_CMD:
		if (rx_msg->msg_type == WLS_CMD_GET_EXTERN_CMD)
			msg_ok = true;
		break;
	default:
		chg_err("unknown msg respone(=%d)\n", mask);
	}
	mutex_unlock(&wls_dev->send_msg_lock);

	if (msg_ok) {
		cancel_delayed_work_sync(&wls_dev->wls_send_msg_work);
		complete(&wls_dev->msg_ack);
	}

	switch (mask) {
	case WLS_RESPONE_ADAPTER_TYPE:
		if (wls_status->adapter_type == WLS_ADAPTER_TYPE_VOOC ||
		    wls_status->adapter_type == WLS_ADAPTER_TYPE_SVOOC ||
		    wls_status->adapter_type == WLS_ADAPTER_TYPE_PD_65W)
			schedule_delayed_work(&wls_dev->wls_match_q_work, 0);
		break;
	default:
		break;
	}
}

static void oplus_chg_wls_data_msg_handler(struct oplus_chg_wls *wls_dev,
					   u8 mask, u8 data[3])
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_rx_msg *rx_msg = &wls_dev->rx_msg;
	bool msg_ok = false;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;

	switch (mask) {
	case WLS_RESPONE_ENCRYPT_DATA4:
		if (rx_msg->msg_type == WLS_CMD_GET_ENCRYPT_DATA4) {
			wls_status->encrypt_data[0] = data[0];
			wls_status->encrypt_data[1] = data[1];
			wls_status->encrypt_data[2] = data[2];
		}
		break;
	case WLS_RESPONE_ENCRYPT_DATA5:
		if (rx_msg->msg_type == WLS_CMD_GET_ENCRYPT_DATA5) {
			wls_status->encrypt_data[3] = data[0];
			wls_status->encrypt_data[4] = data[1];
			wls_status->encrypt_data[5] = data[2];
		}
		break;
	case WLS_RESPONE_ENCRYPT_DATA6:
		if (rx_msg->msg_type == WLS_CMD_GET_ENCRYPT_DATA6) {
			wls_status->encrypt_data[6] = data[0];
			wls_status->encrypt_data[7] = data[1];
		}
		break;
	case WLS_RESPONE_GET_AES_DATA1:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA1) {
			wls_status->aes_encrypt_data[0] = data[0];
			wls_status->aes_encrypt_data[1] = data[1];
			wls_status->aes_encrypt_data[2] = data[2];
		}
		break;
	case WLS_RESPONE_GET_AES_DATA2:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA2) {
			wls_status->aes_encrypt_data[3] = data[0];
			wls_status->aes_encrypt_data[4] = data[1];
			wls_status->aes_encrypt_data[5] = data[2];
		}
		break;
	case WLS_RESPONE_GET_AES_DATA3:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA3) {
			wls_status->aes_encrypt_data[6] = data[0];
			wls_status->aes_encrypt_data[7] = data[1];
			wls_status->aes_encrypt_data[8] = data[2];
		}
		break;
	case WLS_RESPONE_GET_AES_DATA4:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA4) {
			wls_status->aes_encrypt_data[9] = data[0];
			wls_status->aes_encrypt_data[10] = data[1];
			wls_status->aes_encrypt_data[11] = data[2];
		}
		break;
	case WLS_RESPONE_GET_AES_DATA5:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA5) {
			wls_status->aes_encrypt_data[12] = data[0];
			wls_status->aes_encrypt_data[13] = data[1];
			wls_status->aes_encrypt_data[14] = data[2];
		}
		break;
	case WLS_RESPONE_GET_AES_DATA6:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA6) {
			wls_status->aes_encrypt_data[15] = data[0];
		}
		break;
	case WLS_RESPONE_PRODUCT_ID:
		if (rx_msg->msg_type == WLS_CMD_GET_PRODUCT_ID) {
			wls_status->tx_product_id_done = true;
			wls_status->product_id = (data[0] << 8) | data[1];
			chg_info("product_id:0x%x, tx_product_id_done:%d\n",
				wls_status->product_id, wls_status->tx_product_id_done);

			schedule_delayed_work(&wls_dev->wls_match_q_work, 0);
			if (wls_dev->static_config.fastchg_fod_enable && wls_status->fod_parm_for_fastchg)
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
						oplus_chg_wls_get_fod_parm(wls_dev, wls_status->adapter_id,
						FOD_FAST_MODE), wls_dev->static_config.fod_parm_len,
						FOD_FAST_MODE, wls_dev->magcvr_status);
		}
		break;
	case WLS_RESPONE_BATT_TEMP_SOC:
		if (rx_msg->msg_type == WLS_CMD_SEND_BATT_TEMP_SOC) {
			chg_info("temp:%d, soc:%d\n", (data[0] << 8) | data[1], data[2]);
		}
		break;
	case WLS_RESPONE_INTO_FASTCHAGE:
		if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30 &&
			wls_status->adapter_id >= WLS_ADAPTER_MODEL_4 &&
			wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY &&
			rx_msg->msg_type == WLS_CMD_INTO_FASTCHAGE) {
			wls_status->tx_pwr_max_mw = ((data[0] & WLS_TX_POWER_MASK) >> WLS_TX_POWER_OFFSET) * 10 + 10;
			wls_status->tx_mag = (data[0] & WLS_TX_MAG_MASK) >> WLS_TX_MAG_OFFSET;
			wls_status->ta_typ = (data[0] & WLS_TA_TYPE_MASK) >> WLS_TA_TYPE_OFFSET;
			wls_status->adapter_power = data[1];
			wls_status->tx_version = data[2];
			chg_info("tx_pwr_max_mw:%d, tx_mag:%d, ta_typ:%d, adapter_power:%d, tx_version:%d\n",
					wls_status->tx_pwr_max_mw, wls_status->tx_mag, wls_status->ta_typ, wls_status->adapter_power, wls_status->tx_version);
			if (wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY) {
				wls_status->pwr_max_mw = oplus_chg_wls_get_r_power(wls_dev, wls_status->adapter_power);
				(void)oplus_wls_adapter_curve_runing(wls_dev);
			} else {
				wls_status->pwr_max_mw = oplus_chg_wls_get_tripartite_r_power(wls_status->adapter_power);
			}
			if (wls_status->tx_mag == TX_MAG_TEC || wls_status->tx_mag == TX_MAG_NON_TEC) {
				if (wls_dev->magcvr_status == MAGCVR_STATUS_FAR) {
					wls_status->pwr_max_mw = wls_status->pwr_max_mw > wls_dev->non_mag_power_mw ?
											 wls_dev->non_mag_power_mw : wls_status->pwr_max_mw;
				}
			}

			if (wls_status->pwr_max_mw > 0) {
				vote(wls_dev->fcc_votable, RX_MAX_VOTER, true,
				     (wls_status->pwr_max_mw * 1000 / dynamic_cfg->svooc_vol_mv),
				     false);
			}

			wls_status->charge_type = WLS_CHARGE_TYPE_FAST;
			if (wls_dev->static_config.fastchg_fod_enable) {
				if (wls_status->fod_parm_for_fastchg)
					(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
						  oplus_chg_wls_get_fod_parm(wls_dev, wls_status->adapter_id, FOD_FAST_MODE),
						  wls_dev->static_config.fod_parm_len,
						  FOD_FAST_MODE, wls_dev->magcvr_status);
				else if (wls_dev->static_config.fastchg_12v_fod_enable)
					(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
						  oplus_chg_wls_get_fod_parm_12v(wls_dev, wls_status->adapter_id),
						  wls_dev->static_config.fod_parm_len,
						  FOD_FAST_MODE, wls_dev->magcvr_status);
			} else {
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					  wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
					  FOD_FAST_MODE, wls_dev->magcvr_status);
			}
			if (!wls_status->verity_started)
				vote(wls_dev->fcc_votable, VERITY_VOTER, true, dynamic_cfg->verity_curr_max_ma, false);
			oplus_chg_wls_publish_wls_type(wls_dev, true, false);

			if (is_batt_psy_available(wls_dev))
				power_supply_changed(wls_dev->batt_psy);
		}
		break;
	default:
		break;
	}

	mutex_lock(&wls_dev->send_msg_lock);
	switch (mask) {
	case WLS_RESPONE_ENCRYPT_DATA1:
		if (rx_msg->msg_type == WLS_CMD_GET_ENCRYPT_DATA1)
			msg_ok = true;
		break;
	case WLS_RESPONE_ENCRYPT_DATA2:
		if (rx_msg->msg_type == WLS_CMD_GET_ENCRYPT_DATA2)
			msg_ok = true;
		break;
	case WLS_RESPONE_ENCRYPT_DATA3:
		if (rx_msg->msg_type == WLS_CMD_GET_ENCRYPT_DATA3)
			msg_ok = true;
		break;
	case WLS_RESPONE_ENCRYPT_DATA4:
		if (rx_msg->msg_type == WLS_CMD_GET_ENCRYPT_DATA4)
			msg_ok = true;
		break;
	case WLS_RESPONE_ENCRYPT_DATA5:
		if (rx_msg->msg_type == WLS_CMD_GET_ENCRYPT_DATA5)
			msg_ok = true;
		break;
	case WLS_RESPONE_ENCRYPT_DATA6:
		if (rx_msg->msg_type == WLS_CMD_GET_ENCRYPT_DATA6)
			msg_ok = true;
		break;
	case WLS_RESPONE_SET_AES_DATA1:
		if (rx_msg->msg_type == WLS_CMD_SET_AES_DATA1)
			msg_ok = true;
		break;
	case WLS_RESPONE_SET_AES_DATA2:
		if (rx_msg->msg_type == WLS_CMD_SET_AES_DATA2)
			msg_ok = true;
		break;
	case WLS_RESPONE_SET_AES_DATA3:
		if (rx_msg->msg_type == WLS_CMD_SET_AES_DATA3)
			msg_ok = true;
		break;
	case WLS_RESPONE_SET_AES_DATA4:
		if (rx_msg->msg_type == WLS_CMD_SET_AES_DATA4)
			msg_ok = true;
		break;
	case WLS_RESPONE_SET_AES_DATA5:
		if (rx_msg->msg_type == WLS_CMD_SET_AES_DATA5)
			msg_ok = true;
		break;
	case WLS_RESPONE_SET_AES_DATA6:
		if (rx_msg->msg_type == WLS_CMD_SET_AES_DATA6)
			msg_ok = true;
		break;
	case WLS_RESPONE_GET_AES_DATA1:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA1)
			msg_ok = true;
		break;
	case WLS_RESPONE_GET_AES_DATA2:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA2)
			msg_ok = true;
		break;
	case WLS_RESPONE_GET_AES_DATA3:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA3)
			msg_ok = true;
		break;
	case WLS_RESPONE_GET_AES_DATA4:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA4)
			msg_ok = true;
		break;
	case WLS_RESPONE_GET_AES_DATA5:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA5)
			msg_ok = true;
		break;
	case WLS_RESPONE_GET_AES_DATA6:
		if (rx_msg->msg_type == WLS_CMD_GET_AES_DATA6)
			msg_ok = true;
		break;
	case WLS_RESPONE_PRODUCT_ID:
		if (rx_msg->msg_type == WLS_CMD_GET_PRODUCT_ID)
			msg_ok = true;
		break;
	case WLS_RESPONE_BATT_TEMP_SOC:
		if (rx_msg->msg_type == WLS_CMD_SEND_BATT_TEMP_SOC)
			msg_ok = true;
		break;
	case WLS_RESPONE_INTO_FASTCHAGE:
		if (rx_msg->msg_type == WLS_CMD_INTO_FASTCHAGE)
			msg_ok = true;
		break;
	default:
		chg_err("unknown msg respone(=%d)\n", mask);
	}
	mutex_unlock(&wls_dev->send_msg_lock);
	if (msg_ok) {
		cancel_delayed_work_sync(&wls_dev->wls_send_msg_work);
		complete(&wls_dev->msg_ack);
	}
}

static bool oplus_chg_wls_is_oplus_trx(u8 id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(oplus_trx_id_table); i++) {
		if (oplus_trx_id_table[i] == id)
			return true;
	}

	return false;
}

static void oplus_chg_wls_extended_msg_handler(struct oplus_chg_wls *wls_dev,
					       u8 data[])
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_rx_msg *rx_msg = &wls_dev->rx_msg;
	bool msg_ok = false;

	chg_info("msg_type=0x%02x\n", data[0]);

	switch (data[0]) {
	case WLS_CMD_GET_TX_ID:
		if (rx_msg->msg_type != WLS_CMD_GET_TX_ID)
			break;
		if (data[4] == 0x04 && data[2] == 0x05 && oplus_chg_wls_is_oplus_trx(data[3])) {
			wls_status->is_op_trx = true;
			chg_info("is oplus trx\n");
		}
		msg_ok = true;
		break;
	case WLS_CMD_GET_TX_PWR:
		wls_status->tx_pwr_mw = (data[2] << 8) | data[1];
		wls_status->rx_pwr_mw = (data[4] << 8) | data[3];
		wls_status->ploos_mw = wls_status->tx_pwr_mw - wls_status->rx_pwr_mw;
		chg_info("tx_pwr=%d, rx_pwr=%d, ploos=%d\n", wls_status->tx_pwr_mw,
			wls_status->rx_pwr_mw, wls_status->ploos_mw);
		break;
	default:
		chg_err("unknown msg_type(=%d)\n", data[0]);
	}

	if (msg_ok) {
		cancel_delayed_work_sync(&wls_dev->wls_send_msg_work);
		complete(&wls_dev->msg_ack);
	}
}

static void oplus_chg_wls_tx_msg_0x2f_handler(struct oplus_chg_wls *wls_dev, u8 data[])
{
	bool msg_ok = false;

	chg_info("msg_type=0x%02x, data[0]=0x%02x, data[1]=0x%02x, data[2]=0x%02x\n", data[0], data[1], data[2], data[3]);
	switch (data[0]) {
	case WLS_CMD_TX_RESPONE_TEC_POWER:
		chg_err("TX_RESPONE_TEC_POWER:%d\n", data[1]);
		wls_dev->tx_tec_power = data[1];
		msg_ok = true;
		break;
	default:
		chg_err("unknown msg_type(=%d)\n", data[0]);
	}

	if (msg_ok) {
		cancel_delayed_work_sync(&wls_dev->wls_send_msg_work);
		complete(&wls_dev->msg_ack);
	}
}

static int oplus_chg_wls_track_get_track_error_flag(struct oplus_chg_wls *wls_dev, int mask)
{
	if (mask > WLS_TX_ERROR_TA_CAPCHANGE)
		return -EINVAL;
	return wls_dev->wls_status.track_error_flag & (1 << mask);
}

static int oplus_chg_wls_track_set_track_error_flag(struct oplus_chg_wls *wls_dev, int mask)
{
	if (mask > WLS_TX_ERROR_TA_CAPCHANGE)
		return -EINVAL;

	wls_dev->wls_status.track_error_flag |= (1 << mask);
	return wls_dev->wls_status.track_error_flag;
}

static void oplus_chg_wls_tx_error_msg_handler(struct oplus_chg_wls *wls_dev,
					       u8 data[])
{
	bool msg_ok = false;

	chg_info("msg_type=0x%02x, data[0]=0x%02x, data[1]=0x%02x, data[2]=0x%02x\n", data[0], data[1], data[2], data[3]);
	switch (data[0]) {
	case WLS_TX_ERROR_PLOSS_FOD:
		if (oplus_chg_wls_track_get_track_error_flag(wls_dev, WLS_TX_ERROR_PLOSS_FOD) == 0) {
			oplus_chg_wls_track_set_track_error_flag(wls_dev, WLS_TX_ERROR_PLOSS_FOD);
			oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_TX, WLS_ERR_PLOSS_FOD);
		}
		msg_ok = true;
		break;
	case WLS_TX_ERROR_PLOSS_FOD_WARN:
		wls_dev->wls_status.ploss_fod_warn_count++;
		oplus_chg_wls_track_set_track_error_flag(wls_dev, WLS_TX_ERROR_PLOSS_FOD_WARN);
		oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_TX, WLS_ERR_PLOSS_FOD_WARN);
		oplus_chg_wls_ploss_warn_ta_uv(wls_dev, data[0]);
		msg_ok = true;
		break;
	case WLS_TX_ERROR_ASK_WARN:
		msg_ok = true;
		break;
	case WLS_TX_ERROR_ASK_TIMEOUT:
		if (oplus_chg_wls_track_get_track_error_flag(wls_dev, WLS_TX_ERROR_ASK_TIMEOUT) == 0) {
			oplus_chg_wls_track_set_track_error_flag(wls_dev, WLS_TX_ERROR_ASK_TIMEOUT);
			oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_TX, WLS_ERR_ASK_TIMEOUT);
		}
		msg_ok = true;
		break;
	case WLS_TX_ERROR_TA_UV:
		wls_dev->wls_status.ta_uv_count++;
		oplus_chg_wls_track_set_track_error_flag(wls_dev, WLS_TX_ERROR_TA_UV);
		oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_TX, WLS_ERR_TA_UV);
		oplus_chg_wls_ploss_warn_ta_uv(wls_dev, data[0]);
		msg_ok = true;
		break;
	case WLS_TX_ERROR_CEP_TIMEOUT:
		if (oplus_chg_wls_track_get_track_error_flag(wls_dev, WLS_TX_ERROR_CEP_TIMEOUT) == 0) {
			oplus_chg_wls_track_set_track_error_flag(wls_dev, WLS_TX_ERROR_CEP_TIMEOUT);
			oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_TX, WLS_ERR_CEP_TIMEOUT);
		}
		msg_ok = true;
		break;
	case WLS_TX_ERROR_RPP_TIMEOUT:
		if (oplus_chg_wls_track_get_track_error_flag(wls_dev, WLS_TX_ERROR_RPP_TIMEOUT) == 0) {
			oplus_chg_wls_track_set_track_error_flag(wls_dev, WLS_TX_ERROR_RPP_TIMEOUT);
			oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_TX, WLS_ERR_RPP_TIMEOUT);
		}
		msg_ok = true;
		break;
	case WLS_TX_ERROR_OTP:
		if (oplus_chg_wls_track_get_track_error_flag(wls_dev, WLS_TX_ERROR_OTP) == 0) {
			oplus_chg_wls_track_set_track_error_flag(wls_dev, WLS_TX_ERROR_OTP);
			oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_TX, WLS_ERR_TX_OTP);
		}
		msg_ok = true;
		break;
	case WLS_TX_ERROR_TA_CAPCHANGE:
		msg_ok = true;
		break;
	default:
		chg_err("unknown msg_type(=%d)\n", data[0]);
	}

	if (msg_ok) {
		cancel_delayed_work_sync(&wls_dev->wls_send_msg_work);
		complete(&wls_dev->msg_ack);
	}
}

static void oplus_chg_wls_tx_respone_msg_handler(struct oplus_chg_wls *wls_dev,
					       u8 data[])
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	bool msg_ok = false;

	switch (data[0]) {
	case WLS_CMD_TX_GET_PLOSS_MARGIN:
		wls_dev->tx_ploss = ((data[1] << 8) | data[2]) & 0xffff;
		msg_ok = true;
		chg_info("tx_ploss_margin=%d mw\n", wls_dev->tx_ploss);
		break;
	case WLS_CMD_TX_GET_VBRIDGE:
		wls_dev->tx_vbridge = ((data[1] << 8) | data[2]) & 0xffff;
		chg_info("tx_vbridge=%d mv\n", wls_dev->tx_vbridge);
		msg_ok = true;
		break;
	case WLS_CMD_TX_GET_IBRIDGE:
		wls_dev->tx_ibridge = (data[1] & 0xff) * TX_IBRIDGE_STEP;
		wls_dev->tx_ibridge_freq = (data[2] & 0xff) + TX_IBRIDGE_STEP_FREQ_OFFSET;
		chg_info("tx_ibridge=%d ma, tx_ibridge_freq=%d khz\n", wls_dev->tx_ibridge, wls_dev->tx_ibridge_freq);
		msg_ok = true;
		break;
	case WLS_CMD_TX_COIL_QVALUE:
		wls_status->tx_coil = (data[1] & WLS_TX_COIL_MASK) >> WLS_TX_COIL_OFFSET;
		wls_status->tx_q_val = data[1] & WLS_TX_QVALUE_MASK;
		wls_status->tx_q_val_frm_rx = data[2];
		msg_ok = true;
		chg_info("tx_coil=0x%x, tx_q_val=0x%x, tx_q_val_frm_rx:0x%x\n",
			wls_status->tx_coil, wls_status->tx_q_val, wls_status->tx_q_val_frm_rx);
		break;
	default:
		chg_err("unknown msg_type(=%d)\n", data[0]);
	}

	if (msg_ok) {
		cancel_delayed_work_sync(&wls_dev->wls_send_msg_work);
		complete(&wls_dev->msg_ack);
	}
}

static void oplus_chg_wls_rx_msg_callback(void *dev_data, u8 data[])
{
	struct oplus_chg_wls *wls_dev = dev_data;
	u8 temp[2];

	if (wls_dev == NULL) {
		chg_err("wls_dev is NULL\n");
		return;
	}

	temp[0] = ~data[2];
	temp[1] = ~data[4];

	switch (data[0]) {
	case WLS_MSG_TYPE_STANDARD_MSG:
		chg_info("received: standrad msg\n");
		if (!((data[1] >= WLS_RESPONE_ENCRYPT_DATA1 && data[1] <= WLS_RESPONE_ENCRYPT_DATA6) ||
		    (data[1] >= WLS_RESPONE_SET_AES_DATA1 && data[1] <= WLS_RESPONE_GET_AES_DATA6) ||
		    data[1] == WLS_RESPONE_PRODUCT_ID || data[1] == WLS_RESPONE_BATT_TEMP_SOC ||
		    (data[1] == WLS_RESPONE_INTO_FASTCHAGE && wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30 &&
		    wls_dev->wls_status.adapter_id >= WLS_ADAPTER_MODEL_4 &&
		    wls_dev->wls_status.adapter_id != WLS_ADAPTER_THIRD_PARTY))) {
			if ((data[1] == temp[0]) && (data[3] == temp[1])) {
				chg_info("Received TX command: 0x%02X, data: 0x%02X\n",
					 data[1], data[3]);
				oplus_chg_wls_standard_msg_handler(wls_dev, data[1], data[3]);
			} else {
				chg_err("msg data error\n");
			}
		} else {
			chg_info("Received TX data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", data[1], data[2], data[3], data[4]);
			oplus_chg_wls_data_msg_handler(wls_dev, data[1], &data[2]);
		}
		break;
	case WLS_MSG_TYPE_EXTENDED_MSG:
		oplus_chg_wls_extended_msg_handler(wls_dev, &data[1]);
		break;
	case WLS_MSG_TYPE_TX_MSG_0X3F:
		oplus_chg_wls_tx_respone_msg_handler(wls_dev, &data[1]);
		break;
	case WLS_MSG_TYPE_TX_MSG_0X2F:
		oplus_chg_wls_tx_msg_0x2f_handler(wls_dev, &data[1]);
		break;
	case WLS_MSG_TYPE_TX_MSG_0X1F:
		oplus_chg_wls_tx_error_msg_handler(wls_dev, &data[1]);
		break;
	default:
		chg_err("Unknown msg type(=%d)\n", data[0]);
	}
}

static void oplus_chg_wls_send_msg_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev =
		container_of(dwork, struct oplus_chg_wls, wls_send_msg_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_rx_msg *rx_msg = &wls_dev->rx_msg;
	int delay_ms = 1000;
	int cep;
	int rc;
	u8 msg[2] = {rx_msg->msg_head, rx_msg->msg_type};

	if (!wls_status->rx_online) {
		chg_err("wireless charge is not online\n");
		complete(&wls_dev->msg_ack);
		return;
	}

	if (rx_msg->msg_type == WLS_CMD_GET_TX_PWR || rx_msg->long_data) {
		/*need wait cep*/
		rc = oplus_chg_wls_get_cep(wls_dev->wls_rx->rx_ic, &cep);
		if (rc < 0) {
			chg_err("can't read cep, rc=%d\n", rc);
			complete(&wls_dev->msg_ack);
			return;
		}
		chg_info("wkcs: cep = %d\n", cep);
		if (abs(cep) > 3) {
			delay_ms = 3000;
			goto out;
		}
		chg_info("wkcs: get tx pwr\n");
	}
	if (rx_msg->long_data || rx_msg->raw_data)
		rc = oplus_chg_wls_rx_send_data(wls_dev->wls_rx, msg,
			rx_msg->buf, ARRAY_SIZE(rx_msg->buf), rx_msg->raw_data);
	else
		rc = oplus_chg_wls_rx_send_msg(wls_dev->wls_rx, rx_msg->msg_type, rx_msg->data);
	if (rc < 0) {
		chg_err("rx send msg error, rc=%d\n", rc);
		complete(&wls_dev->msg_ack);
		return;
	}

out:
	schedule_delayed_work(&wls_dev->wls_send_msg_work, msecs_to_jiffies(delay_ms));
}

static int oplus_chg_wls_send_msg(struct oplus_chg_wls *wls_dev, u8 msg, u8 data, int wait_time_s)
{
	struct oplus_chg_rx_msg *rx_msg = &wls_dev->rx_msg;
	int cep;
	int rc;
	unsigned long time_left;

	if (!wls_dev->msg_callback_ok) {
		rc = oplus_chg_wls_rx_register_msg_callback(wls_dev->wls_rx->rx_ic, wls_dev,
			oplus_chg_wls_rx_msg_callback);
		if (rc < 0) {
			chg_err("can't reg msg callback, rc=%d\n", rc);
			return rc;
		} else {
			wls_dev->msg_callback_ok = true;
		}
	}

	if (rx_msg->pending) {
		chg_err("msg pending\n");
		return -EAGAIN;
	}

	if (msg == WLS_CMD_GET_TX_PWR) {
		/*need wait cep*/
		rc = oplus_chg_wls_get_cep(wls_dev->wls_rx->rx_ic, &cep);
		if (rc < 0) {
			chg_err("can't read cep, rc=%d\n", rc);
			return rc;
		}
		if (abs(cep) > 3) {
			chg_info("wkcs: cep = %d\n", cep);
			return -EAGAIN;
		}
	}

	rc = oplus_chg_wls_rx_send_msg(wls_dev->wls_rx, msg, data);
	if (rc) {
		chg_err("send msg error, rc=%d\n", rc);
		return rc;
	}

	mutex_lock(&wls_dev->send_msg_lock);
	rx_msg->msg_type = msg;
	rx_msg->data = data;
	rx_msg->long_data = false;
	mutex_unlock(&wls_dev->send_msg_lock);
	if (wait_time_s > 0) {
		rx_msg->pending = true;
		reinit_completion(&wls_dev->msg_ack);
		schedule_delayed_work(&wls_dev->wls_send_msg_work, msecs_to_jiffies(1000));
		time_left = wait_for_completion_timeout(&wls_dev->msg_ack, msecs_to_jiffies(wait_time_s * 1000));
		if (!time_left) {
			chg_err("Error, timed out sending message\n");
			cancel_delayed_work_sync(&wls_dev->wls_send_msg_work);
			rc = -ETIMEDOUT;
		} else {
			rc = 0;
		}
		rx_msg->msg_type = 0;
		rx_msg->data = 0;
		rx_msg->buf[0] = 0;
		rx_msg->buf[1] = 0;
		rx_msg->buf[2] = 0;
		rx_msg->respone_type = 0;
		rx_msg->long_data = false;
		rx_msg->pending = false;
	} else if (wait_time_s < 0) {
		rx_msg->pending = false;
		schedule_delayed_work(&wls_dev->wls_send_msg_work, msecs_to_jiffies(1000));
	}

	return rc;
}

static int oplus_chg_wls_send_data(struct oplus_chg_wls *wls_dev, u8 msg, u8 data[3], int wait_time_s)
{
	struct oplus_chg_rx_msg *rx_msg = &wls_dev->rx_msg;
	int cep;
	int rc;
	u8 msg_buf[2] = {0, msg};

	if (!wls_dev->msg_callback_ok) {
		rc = oplus_chg_wls_rx_register_msg_callback(wls_dev->wls_rx->rx_ic, wls_dev,
			oplus_chg_wls_rx_msg_callback);
		if (rc < 0) {
			chg_err("can't reg msg callback, rc=%d\n", rc);
			return rc;
		} else {
			wls_dev->msg_callback_ok = true;
		}
	}

	if (rx_msg->pending) {
		chg_info("send msg pending, msg=0x%02x%02x, data=0x%02x 0x%02x 0x%02x\n",
			msg_buf[0], msg_buf[1], data[0], data[1], data[2]);
		return -EAGAIN;
	}

	/*need wait cep*/
	rc = oplus_chg_wls_get_cep(wls_dev->wls_rx->rx_ic, &cep);
	if (rc < 0) {
		chg_err("can't read cep, rc=%d\n", rc);
		return rc;
	}
	if (abs(cep) > 3) {
		chg_info("wkcs: cep = %d\n", cep);
		return -EAGAIN;
	}

	rc = oplus_chg_wls_rx_send_data(wls_dev->wls_rx, msg_buf, data, 3, 0);
	if (rc) {
		chg_err("send data error, rc=%d\n", rc);
		return rc;
	}

	mutex_lock(&wls_dev->send_msg_lock);
	rx_msg->msg_type = msg;
	rx_msg->buf[0] = data[0];
	rx_msg->buf[1] = data[1];
	rx_msg->buf[2] = data[2];
	rx_msg->long_data = true;
	mutex_unlock(&wls_dev->send_msg_lock);
	if (wait_time_s > 0) {
		rx_msg->pending = true;
		reinit_completion(&wls_dev->msg_ack);
		schedule_delayed_work(&wls_dev->wls_send_msg_work, msecs_to_jiffies(1000));
		rc = wait_for_completion_timeout(&wls_dev->msg_ack, msecs_to_jiffies(wait_time_s * 1000));
		if (!rc) {
			chg_err("Error, timed out sending message\n");
			cancel_delayed_work_sync(&wls_dev->wls_send_msg_work);
			rc = -ETIMEDOUT;
		}
		rx_msg->msg_type = 0;
		rx_msg->data = 0;
		rx_msg->buf[0] = 0;
		rx_msg->buf[1] = 0;
		rx_msg->buf[2] = 0;
		rx_msg->respone_type = 0;
		rx_msg->long_data = false;
		rx_msg->pending = false;
	} else if (wait_time_s < 0) {
		rx_msg->pending = false;
		schedule_delayed_work(&wls_dev->wls_send_msg_work, msecs_to_jiffies(1000));
	}

	return rc;
}

static int oplus_chg_wls_send_raw_data(struct oplus_chg_wls *wls_dev, u8 msg[2], u8 data[3], int wait_time_s)
{
	struct oplus_chg_rx_msg *rx_msg = &wls_dev->rx_msg;
	int rc = 0;
	unsigned long time_left;

	if (!wls_dev->msg_callback_ok) {
		rc = oplus_chg_wls_rx_register_msg_callback(wls_dev->wls_rx->rx_ic, wls_dev,
			oplus_chg_wls_rx_msg_callback);
		if (rc < 0) {
			chg_err("can't reg msg callback, rc=%d\n", rc);
			return rc;
		} else {
			wls_dev->msg_callback_ok = true;
		}
	}

	if (rx_msg->pending) {
		chg_info("send msg pending, msg=0x%02x%02x, data=0x%02x 0x%02x 0x%02x\n",
			msg[0], msg[1], data[0], data[1], data[2]);
		return -EAGAIN;
	}

	rc = oplus_chg_wls_rx_send_data(wls_dev->wls_rx, msg, data, 3, 1);

	mutex_lock(&wls_dev->send_msg_lock);
	rx_msg->msg_head = msg[0];
	rx_msg->msg_type = msg[1];
	rx_msg->buf[0] = data[0];
	rx_msg->buf[1] = data[1];
	rx_msg->buf[2] = data[2];
	rx_msg->raw_data = true;
	mutex_unlock(&wls_dev->send_msg_lock);
	if (wait_time_s > 0) {
		rx_msg->pending = true;
		reinit_completion(&wls_dev->msg_ack);
		schedule_delayed_work(&wls_dev->wls_send_msg_work, msecs_to_jiffies(1000));
		if (rc) {
			chg_err("send data error, rc=%d\n", rc);
		}
		time_left = wait_for_completion_timeout(&wls_dev->msg_ack, msecs_to_jiffies(wait_time_s * 1000));
		if (!time_left) {
			chg_err("Error, timed out sending message\n");
			cancel_delayed_work_sync(&wls_dev->wls_send_msg_work);
			rc = -ETIMEDOUT;
		} else {
			rc = 0;
		}
		rx_msg->msg_head = 0;
		rx_msg->msg_type = 0;
		rx_msg->data = 0;
		rx_msg->buf[0] = 0;
		rx_msg->buf[1] = 0;
		rx_msg->buf[2] = 0;
		rx_msg->respone_type = 0;
		rx_msg->raw_data = false;
		rx_msg->pending = false;
	} else if (wait_time_s < 0) {
		rx_msg->pending = false;
		schedule_delayed_work(&wls_dev->wls_send_msg_work, msecs_to_jiffies(1000));
	}

	return rc;
}

static void oplus_chg_wls_clean_msg(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_rx_msg *rx_msg = &wls_dev->rx_msg;

	cancel_delayed_work_sync(&wls_dev->wls_send_msg_work);
	if (rx_msg->pending)
		complete(&wls_dev->msg_ack);
	rx_msg->msg_type = 0;
	rx_msg->data = 0;
	rx_msg->buf[0] = 0;
	rx_msg->buf[1] = 0;
	rx_msg->buf[2] = 0;
	rx_msg->respone_type = 0;
	rx_msg->long_data = false;
	rx_msg->pending = false;
	rx_msg->raw_data = false;
}

static int oplus_chg_wls_get_verity_data(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	mutex_lock(&wls_dev->cmd_data_lock);
	memset(&wls_dev->cmd, 0, sizeof(struct wls_dev_cmd));
	wls_dev->cmd.cmd = WLS_DEV_CMD_WLS_AUTH;
	wls_dev->cmd.data_size = 0;
	wls_dev->cmd_data_ok = true;
	wls_status->verity_data_ok = false;
	mutex_unlock(&wls_dev->cmd_data_lock);
	wake_up(&wls_dev->read_wq);

	return 0;
}

#define VERITY_TIMEOUT_MAX	40
static void oplus_chg_wls_des_verity(struct oplus_chg_wls *wls_dev)
{
	u8 buf[3];
	int rc;
	struct oplus_chg_wls_status *wls_status;

	if (!wls_dev)
		return;

	wls_status = &wls_dev->wls_status;
	if (!wls_status->rx_online)
		return;

	wls_status->verity_started = true;
#ifndef OPLUS_CHG_WLS_AUTH_ENABLE
	wls_status->verity_pass = true;
	goto done;
#endif

	wls_status->verity_wait_timeout = jiffies + VERITY_TIMEOUT_MAX * HZ;
retry:
	if (!wls_status->verity_data_ok) {
		chg_err("verity data not ready\n");
		if (wls_status->verity_count < 5) {
			wls_status->verity_count++;
			oplus_chg_wls_get_verity_data(wls_dev);
			schedule_delayed_work(&wls_dev->wls_verity_work,
					      msecs_to_jiffies(500));
			return;
		}
		wls_status->verity_pass = false;
		goto done;
	}

	buf[0] = wls_status->verfity_data.random_num[0];
	buf[1] = wls_status->verfity_data.random_num[1];
	buf[2] = wls_status->verfity_data.random_num[2];
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_data(wls_dev, WLS_CMD_GET_ENCRYPT_DATA1, buf, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_ENCRYPT_DATA1, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	buf[0] = wls_status->verfity_data.random_num[3];
	buf[1] = wls_status->verfity_data.random_num[4];
	buf[2] = wls_status->verfity_data.random_num[5];
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_data(wls_dev, WLS_CMD_GET_ENCRYPT_DATA2, buf, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_ENCRYPT_DATA2, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	buf[0] = wls_status->verfity_data.random_num[6];
	buf[1] = wls_status->verfity_data.random_num[7];
	buf[2] = WLS_ENCODE_MASK;
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_data(wls_dev, WLS_CMD_GET_ENCRYPT_DATA3, buf, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_ENCRYPT_DATA3, rc);
		wls_status->verity_pass = false;
		goto done;
	}

	/* Wait for the base encryption to complete */
	msleep(500);

	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_ENCRYPT_DATA4, 0xff, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_ENCRYPT_DATA4, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_ENCRYPT_DATA5, 0xff, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_ENCRYPT_DATA5, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_ENCRYPT_DATA6, 0xff, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_ENCRYPT_DATA6, rc);
		wls_status->verity_pass = false;
		goto done;
	}

	chg_info("encrypt_data: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
		 wls_status->encrypt_data[0], wls_status->encrypt_data[1],
		 wls_status->encrypt_data[2], wls_status->encrypt_data[3],
		 wls_status->encrypt_data[4], wls_status->encrypt_data[5],
		 wls_status->encrypt_data[6], wls_status->encrypt_data[7]);

	if (memcmp(&wls_status->encrypt_data,
		   &wls_status->verfity_data.encode_num, WLS_AUTH_ENCODE_LEN)) {
		chg_err("verity faile\n");
		wls_status->verity_pass = false;
	} else {
		chg_info("verity pass\n");
		wls_status->verity_pass = true;
	}

done:
	wls_status->verity_count = 0;
	if (!wls_status->verity_pass && wls_status->rx_online &&
	    time_before(jiffies, wls_status->verity_wait_timeout))
		goto retry;
	wls_status->verity_done = true;
	vote(wls_dev->fcc_votable, VERITY_VOTER, false, 0, false);
	if (!wls_status->verity_pass && wls_status->rx_online) {
		wls_status->online_keep = true;
		wls_status->verity_state_keep = true;
		vote(wls_dev->rx_disable_votable, VERITY_VOTER, true, 1, false);
		schedule_delayed_work(&wls_dev->rx_verity_restore_work, msecs_to_jiffies(500));
		oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_RX, WLS_ERR_DES_VERITY);
	}
}

static void oplus_chg_wls_aes_verity(struct oplus_chg_wls *wls_dev)
{
	u8 buf[3];
	int rc;
	int ret = CMD_ERROR_DATA_INVALID;
	struct oplus_chg_wls_status *wls_status;

	if (!wls_dev)
		return;

	wls_status = &wls_dev->wls_status;
	if (!wls_status->rx_online)
		return;

	wls_status->verity_started = true;
#ifndef OPLUS_CHG_WLS_AUTH_ENABLE
	wls_status->verity_pass = true;
	goto done;
#endif

	wls_status->verity_wait_timeout = jiffies + VERITY_TIMEOUT_MAX * HZ;
retry:
	if (!wls_status->aes_verity_data_ok) {
		chg_err("verity data not ready\n");
		ret = oplus_chg_set_mutual_cmd(CMD_WLS_THIRD_PART_AUTH,
				sizeof(wls_status->vendor_id), &(wls_status->vendor_id));
		if (ret == CMD_ERROR_HIDL_NOT_READY) {
			schedule_delayed_work(&wls_dev->wls_verity_work, msecs_to_jiffies(1000));
			return;
		} else if (ret != CMD_ACK_OK) {
			if (wls_status->verity_count < 5) {
				wls_status->verity_count++;
				schedule_delayed_work(&wls_dev->wls_verity_work, msecs_to_jiffies(1500));
				return;
			}
			wls_status->verity_pass = false;
			goto done;
		}
	}

	buf[0] = wls_status->aes_verfity_data.aes_random_num[0];
	buf[1] = wls_status->aes_verfity_data.aes_random_num[1];
	buf[2] = wls_status->aes_verfity_data.aes_random_num[2];
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_data(wls_dev, WLS_CMD_SET_AES_DATA1, buf, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_SET_AES_DATA1, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	msleep(500);
	buf[0] = wls_status->aes_verfity_data.aes_random_num[3];
	buf[1] = wls_status->aes_verfity_data.aes_random_num[4];
	buf[2] = wls_status->aes_verfity_data.aes_random_num[5];
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_data(wls_dev, WLS_CMD_SET_AES_DATA2, buf, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_SET_AES_DATA2, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	msleep(500);
	buf[0] = wls_status->aes_verfity_data.aes_random_num[6];
	buf[1] = wls_status->aes_verfity_data.aes_random_num[7];
	buf[2] = wls_status->aes_verfity_data.aes_random_num[8];
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_data(wls_dev, WLS_CMD_SET_AES_DATA3, buf, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_SET_AES_DATA3, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	msleep(500);
	buf[0] = wls_status->aes_verfity_data.aes_random_num[9];
	buf[1] = wls_status->aes_verfity_data.aes_random_num[10];
	buf[2] = wls_status->aes_verfity_data.aes_random_num[11];
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_data(wls_dev, WLS_CMD_SET_AES_DATA4, buf, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_SET_AES_DATA4, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	msleep(500);
	buf[0] = wls_status->aes_verfity_data.aes_random_num[12];
	buf[1] = wls_status->aes_verfity_data.aes_random_num[13];
	buf[2] = wls_status->aes_verfity_data.aes_random_num[14];
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_data(wls_dev, WLS_CMD_SET_AES_DATA5, buf, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_SET_AES_DATA5, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	msleep(500);
	buf[0] = wls_status->aes_verfity_data.aes_random_num[15];
	buf[1] = wls_status->vendor_id;
	buf[2] = wls_status->aes_key_num;
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_data(wls_dev, WLS_CMD_SET_AES_DATA6, buf, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_SET_AES_DATA6, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	/* Wait for the base encryption to complete */
	msleep(500);

	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_AES_DATA1, 0xff, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_AES_DATA1, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	msleep(500);
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_AES_DATA2, 0xff, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_AES_DATA2, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	msleep(500);
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_AES_DATA3, 0xff, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_AES_DATA3, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	msleep(500);
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_AES_DATA4, 0xff, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_AES_DATA4, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	msleep(500);
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_AES_DATA5, 0xff, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_AES_DATA5, rc);
		wls_status->verity_pass = false;
		goto done;
	}
	msleep(500);
	wls_status->verity_count = 0;
	do {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_AES_DATA6, 0xff, 5);
		if (rc < 0) {
			if (rc != -EAGAIN)
				wls_status->verity_count++;
			msleep(500);
		}
	} while (rc < 0 && wls_status->verity_count < 5 && wls_status->rx_online);
	if (rc < 0 || !wls_status->rx_online) {
		chg_err("send 0x%02x error, rc=%d\n", WLS_CMD_GET_AES_DATA6, rc);
		wls_status->verity_pass = false;
		goto done;
	}

	chg_info("aes encrypt_data: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, "
		 "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
		 wls_status->aes_encrypt_data[0], wls_status->aes_encrypt_data[1],
		 wls_status->aes_encrypt_data[2], wls_status->aes_encrypt_data[3],
		 wls_status->aes_encrypt_data[4], wls_status->aes_encrypt_data[5],
		 wls_status->aes_encrypt_data[6], wls_status->aes_encrypt_data[7],
		 wls_status->aes_encrypt_data[8], wls_status->aes_encrypt_data[9],
		 wls_status->aes_encrypt_data[10], wls_status->aes_encrypt_data[11],
		 wls_status->aes_encrypt_data[12], wls_status->aes_encrypt_data[13],
		 wls_status->aes_encrypt_data[14], wls_status->aes_encrypt_data[15]);

	if (memcmp(&wls_status->aes_encrypt_data,
		   &wls_status->aes_verfity_data.aes_encode_num, WLS_AUTH_AES_ENCODE_LEN)) {
		chg_err("verity faile\n");
		wls_status->verity_pass = false;
	} else {
		chg_info("verity pass\n");
		wls_status->verity_pass = true;
	}

done:
	wls_status->verity_count = 0;
	if (!wls_status->verity_pass && wls_status->rx_online &&
	    time_before(jiffies, wls_status->verity_wait_timeout))
		goto retry;
	wls_status->verity_done = true;
	vote(wls_dev->fcc_votable, VERITY_VOTER, false, 0, false);
	if (!wls_status->verity_pass && wls_status->rx_online) {
		wls_status->online_keep = true;
		wls_status->verity_state_keep = true;
		vote(wls_dev->rx_disable_votable, VERITY_VOTER, true, 1, false);
		schedule_delayed_work(&wls_dev->rx_verity_restore_work, msecs_to_jiffies(500));
		oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_RX, WLS_ERR_AES_VERITY);
	}
}

static void oplus_chg_wls_verity_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev =
		container_of(dwork, struct oplus_chg_wls, wls_verity_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (!wls_status->verify_by_aes)
		oplus_chg_wls_des_verity(wls_dev);
	else
		oplus_chg_wls_aes_verity(wls_dev);
}

static void oplus_chg_wls_exchange_batt_mesg(struct oplus_chg_wls *wls_dev)
{
	int soc = 0, temp = 0;
	u8 buf[3];
	struct oplus_chg_wls_status *wls_status;
	int rc;

	if (!wls_dev)
		return;

	wls_status = &wls_dev->wls_status;
	if (!wls_status->rx_online || !wls_status->verity_done ||
	    wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY)
		return;

	rc = oplus_chg_wls_get_ui_soc(wls_dev, &soc);
	if (rc < 0) {
		chg_err("can't get ui soc, rc=%d\n", rc);
		return;
	}
	rc = oplus_chg_wls_get_batt_temp(wls_dev, &temp);
	if (rc < 0) {
		chg_err("can't get batt temp, rc=%d\n", rc);
		return;
	}

	buf[0] = (temp >> 8) & 0xff;
	buf[1] = temp & 0xff;
	buf[2] = soc & 0xff;
	chg_info("soc:%d, temp:%d\n", soc, temp);

	mutex_lock(&wls_dev->update_data_lock);
	oplus_chg_wls_send_data(wls_dev, WLS_CMD_SEND_BATT_TEMP_SOC, buf, 0);
	msleep(100);
	mutex_unlock(&wls_dev->update_data_lock);
}

static void oplus_chg_wls_early_reset_variables(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	wls_status->cool_down = 0;
	vote(wls_dev->nor_icl_votable, COOL_DOWN_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, COOL_DOWN_VOTER, false, 0, false);
}

static void oplus_chg_wls_reset_variables(struct oplus_chg_wls *wls_dev) {
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;

	wls_status->adapter_type = WLS_ADAPTER_TYPE_UNKNOWN;
	wls_status->adapter_id = 0;
	wls_status->adapter_power = 0;
	wls_status->charge_type = WLS_CHARGE_TYPE_DEFAULT;
	wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_DEFAULT;
	wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_DEFAULT;
	wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DEFAULT;
	wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_DEFAULT;
	wls_status->wls_type = OPLUS_CHG_WLS_UNKNOWN;
	wls_status->temp_region = BATT_TEMP_COLD;

	wls_status->is_op_trx = false;
	wls_status->epp_working = false;
	wls_status->epp_5w = false;
	wls_status->quiet_mode = false;
	wls_status->quiet_mode_init = false;
	wls_status->cep_timeout_adjusted = false;
	wls_status->upgrade_fw_pending = false;
	wls_status->fw_upgrading = false;
	wls_status->trx_present = false;
	wls_status->trx_online = false;
	wls_status->trx_transfer_start_time = 0;
	wls_status->trx_transfer_end_time = 0;
	wls_status->trx_usb_present_once = false;
	wls_status->fastchg_started = false;
	wls_status->fastchg_disable = false;
	wls_status->fastchg_vol_set_ok = false;
	wls_status->fastchg_curr_set_ok = false;
	wls_status->fastchg_curr_need_dec = false;
	wls_status->ffc_check = false;
	wls_status->wait_cep_stable = false;
	wls_status->rx_adc_test_enable = false;
	wls_status->rx_adc_test_pass = false;
	wls_status->boot_online_keep = false;
	wls_status->chg_done = false;
	wls_status->chg_done_quiet_mode = false;

	wls_status->state_sub_step = 0;
	wls_status->iout_ma = 0;
	wls_status->iout_ma_conunt = 0;
	wls_status->vout_mv = 0;
	wls_status->vrect_mv = 0;
	wls_status->trx_curr_ma = 0;
	wls_status->trx_vol_mv = 0;
	wls_status->fastchg_target_vol_mv = 0;
	wls_status->fastchg_ibat_max_ma = 0;
	wls_status->tx_pwr_mw = 0;
	wls_status->rx_pwr_mw = 0;
	wls_status->ploos_mw = 0;
	wls_status->epp_plus_skin_step = 0;
	wls_status->epp_skin_step = 0;
	wls_status->bpp_skin_step = 0;
	wls_status->epp_plus_led_on_skin_step = 0;
	wls_status->epp_led_on_skin_step = 0;
	wls_status->pwr_max_mw = 0;
	wls_status->quiet_mode_need = WLS_QUIET_MODE_UNKOWN;
	wls_status->adapter_info_cmd_count = 0;
	wls_status->fastchg_retry_count = 0;
	wls_status->fastchg_err_count = 0;
	wls_status->non_fcc_level = 0;
	wls_status->skewing_level = 0;
	wls_status->fast_cep_check = 0;
	wls_status->fastchg_exit_fan_speed_set = false;
	wls_status->fastchg_ui_100_fan_speed_set = false;

	if (!wls_status->fastchg_to_bpp_state_keep)
		wls_status->fastchg_timeout_to_bpp = false;

	wls_status->cep_ok_wait_timeout = jiffies;
	wls_status->fastchg_retry_timer = jiffies;
	wls_status->fastchg_err_timer = jiffies;
	wls_status->fastchg_enter_timer = jiffies;
	wls_dev->batt_charge_enable = true;
	wls_dev->tx_vbridge = 0;
	wls_dev->tx_ibridge = 0;
	wls_dev->tx_ibridge_freq = 0;
	wls_dev->tx_ploss = 0;
	wls_status->tx_coil = 0;
	wls_status->tx_q_val = 0;
	wls_status->tx_q_val_frm_rx = 0;
	wls_status->q_val_done = 0;
	wls_dev->tx_tec_power = 0;
	wls_status->ploss_fod_warn_count = 0;
	wls_status->ta_uv_count = 0;
	wls_dev->wls_status.track_error_flag = 0;
	wls_dev->tx_ploss_voted = 0;
	wls_dev->tec_init_power = 0;
	wls_dev->monitor_count = 0;

	wls_status->wls_bcc_max_curr = 0;
	wls_status->wls_bcc_min_curr = 0;
	wls_status->wls_bcc_stop_curr = 0;
	wls_status->bcc_curve_idx = 0;
	wls_status->bcc_true_idx = 0;
	wls_status->bcc_temp_range = 0;

	wls_status->epp_chg_level = -1;
	dynamic_cfg->epp_steps.fcc_wait_timeout = jiffies;
	dynamic_cfg->norchg_steps.norchg_vol_level = 0;
	dynamic_cfg->norchg_steps.pre_temp_region = -1;

	/*wls encrypt*/
	if (!wls_status->verity_state_keep) {
		/*
		 * After the AP is actively disconnected, the verification
		 * status flag does not need to change.
		 */
		wls_status->verity_pass = true;
	}
	wls_status->verity_started = false;
	wls_status->verity_data_ok = false;
	wls_status->verity_count = 0;
	wls_status->verity_done = false;
	wls_status->verify_by_aes = false;
	wls_status->tx_extern_cmd_done = false;
	wls_status->tx_product_id_done = false;
	if (wls_status->rx_present) {
		wls_status->vendor_id = 0;
		wls_status->product_id = 0;
		wls_status->tx_manu_id = 0;
	}
	wls_dev->mms_info.event_code = WLS_EVENT_RX_UNKNOWN;
	memset(&wls_status->encrypt_data, 0, ARRAY_SIZE(wls_status->encrypt_data));
	memset(&wls_status->verfity_data, 0, sizeof(struct wls_auth_result));
	wls_status->adapter_curve_pwr = 0;

	if (!wls_dev->usb_present)
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, wls_status->rx_present ? 0 : 100, false);
	if (!wls_status->rx_present)
		vote(wls_dev->nor_icl_votable, WLS_CAMERA_MODE_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, MAX_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, SKIN_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, RX_MAX_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, UOVP_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, CHG_DONE_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, NON_FFC_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, CV_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, CEP_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, BCC_CURRENT_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, EPP_CURVE_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, CHG_LIMIT_CHG_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, CHG_FULL_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, FFC_VOTER, false, 0, false);
	vote(wls_dev->nor_fcc_votable, USER_VOTER, false, 0, false);
	vote(wls_dev->nor_fcc_votable, MAX_VOTER, false, 0, false);
	vote(wls_dev->nor_fcc_votable, FFC_VOTER, false, 0, false);
	if (is_nor_fv_votable_available(wls_dev))
		vote(wls_dev->nor_fv_votable, USER_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, WLS_PLOSS_WARN_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, WLS_TA_UV_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, MAX_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, DEF_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, STEP_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, EXIT_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, FCC_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, JEITA_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, CEP_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, SKIN_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, DEBUG_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, BASE_MAX_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, RX_MAX_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, VERITY_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, BCC_CURRENT_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, BATT_BAL_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, RX_ADAPTER_CURVE_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, WLS_Q_VALUE_ERROR_VOTER, false, 0, false);
	vote(wls_dev->fcc_votable, BAD_SUB_BTB_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, QUIET_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, CEP_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, FCC_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, SKIN_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, BATT_VOL_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, BATT_CURR_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, RX_FAST_ERR_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, IOUT_CURR_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, STARTUP_CEP_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, HW_ERR_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, CURR_ERR_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, COOL_DOWN_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, CHG_LIMIT_CHG_VOTER, false, 0, false);
	vote(wls_dev->fastchg_disable_votable, CHG_FULL_VOTER, false, 0, false);
	vote(wls_dev->nor_out_disable_votable, FFC_VOTER, false, 0, false);
	vote(wls_dev->nor_out_disable_votable, USER_VOTER, false, 0, false);
	vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
	vote(wls_dev->nor_out_disable_votable, UOVP_VOTER, false, 0, false);
	vote(wls_dev->nor_out_disable_votable, CHG_LIMIT_CHG_VOTER, false, 0, false);
	vote(wls_dev->nor_out_disable_votable, CHG_FULL_VOTER, false, 0, false);
	vote(wls_dev->nor_input_disable_votable, UOVP_VOTER, false, 0, false);
	vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
	vote(wls_dev->rx_comu_votable, CHG_LIMIT_CHG_VOTER, false, 0, false);
	vote(wls_dev->rx_comu_votable, CHG_FULL_VOTER, false, 0, false);
	vote(wls_dev->quiet_mode_votable, WLS_QUIET_MODE_VOTER, false, 0, false);
	if (!wls_dev->usb_present)
		rerun_election(wls_dev->nor_icl_votable, false);
	rerun_election(wls_dev->nor_fcc_votable, false);
	if (is_nor_fv_votable_available(wls_dev))
		rerun_election(wls_dev->nor_fv_votable, false);
	rerun_election(wls_dev->fcc_votable, false);
	rerun_election(wls_dev->fastchg_disable_votable, false);
	rerun_election(wls_dev->nor_out_disable_votable, false);
	rerun_election(wls_dev->nor_input_disable_votable, false);
	rerun_election(wls_dev->rx_comu_votable, false);
	oplus_chg_wls_publish_wls_type(wls_dev, false, true);


	/*if (is_comm_ocm_available(wls_dev))
		oplus_chg_comm_status_init(wls_dev->comm_ocm);*/
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAULT_INJECT_CHG)
noinline
#endif
#define OPLUS_CHG_TRACK_V30_INFO_LEN 256
static void oplus_chg_wls_update_track_info(struct oplus_chg_wls *wls_dev,
					    char *crux_info, bool clear)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int batt_temp = 0;
	u32 rx_version = 0;
	u32 trx_version = 0;
	int cep_curr_ma = 0;
	static int highest_temp = 0;
	static int max_iout = 0;
	static int max_vout = 0;
	static int min_cool_down = 0;
	static int min_skewing_current = 0;
	static int adapter_type = 0;
	static int adapter_id = 0;
	char v30_info_buf[OPLUS_CHG_TRACK_V30_INFO_LEN] = {0};
	union mms_msg_data data = { 0 };

	if (clear) {
		highest_temp = 0;
		max_iout = 0;
		max_vout = 0;
		min_cool_down = 0;
		min_skewing_current = 0;
		adapter_type = 0;
		adapter_id = 0;
		return;
	}

	oplus_chg_wls_get_batt_temp(wls_dev, &batt_temp);
	highest_temp = max(highest_temp, batt_temp);
	max_iout = max(max_iout, wls_status->iout_ma);
	max_vout = max(max_vout, wls_status->vout_mv);
	wls_status->track_record.max_vrect = max(wls_status->track_record.max_vrect, wls_status->vrect_mv);
	if (wls_status->cool_down) {
		if (!min_cool_down)
			min_cool_down = wls_status->cool_down;
		else
			min_cool_down = min(min_cool_down, wls_status->cool_down);
	}

	if (is_client_vote_enabled(wls_dev->fcc_votable, CEP_VOTER))
		cep_curr_ma = get_client_vote(wls_dev->fcc_votable, CEP_VOTER);
	else if (is_client_vote_enabled(wls_dev->nor_icl_votable, CEP_VOTER))
		cep_curr_ma = get_client_vote(wls_dev->nor_icl_votable, CEP_VOTER);

	if (cep_curr_ma) {
		if (!min_skewing_current)
			min_skewing_current = cep_curr_ma;
		else
			min_skewing_current = min(min_skewing_current, cep_curr_ma);
	}

	oplus_chg_wls_rx_get_trx_version(wls_dev->wls_rx->rx_ic, &trx_version);
	oplus_chg_wls_rx_get_rx_version(wls_dev->wls_rx->rx_ic, &rx_version);

	if (wls_status->rx_online) {
		oplus_mms_get_item_data(wls_dev->wls_topic, WLS_ITEM_TRACK_RECORD_WLS_TYPE, &data, true);
		adapter_type = data.intval;
		adapter_id = wls_status->adapter_id;
	}

	if (crux_info) {
		if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30 &&
		    wls_status->adapter_id >= WLS_ADAPTER_MODEL_4 &&
		    wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY) {
			scnprintf(v30_info_buf, OPLUS_CHG_TRACK_V30_INFO_LEN,
				",coil:%d,tx_q=%d,tx_power=%d,"
				"tx_mag=%d,ta_type=%d,ploss_fod_warn_count=%d,"
				"ta_uv_count=%d,rx_proto_ver=%d",
				wls_status->tx_coil, wls_status->tx_q_val, wls_status->tx_pwr_max_mw,
				wls_status->tx_mag, wls_status->ta_typ, wls_status->ploss_fod_warn_count,
				wls_status->ta_uv_count, wls_dev->rx_protocol_version);
		}
		scnprintf(crux_info,
			OPLUS_CHG_TRACK_CURX_INFO_LEN,
			"$$wls_general_info@@tx_version=0x%x,rx_version=0x%x,adapter_type=%d,"
			"dock_version=0x%x,fastchg_ing=%d,vout=%d,"
			"iout=%d,break_count=%d,wpc_chg_err=%d,"
			"highest_temp=%d,max_iout=%d,max_vout=%d,min_cool_down=%d,"
			"min_skewing_current=%d,wls_auth_fail=%d,work_silent_mode=%d,"
			"magcvr=%d,verify_by_aes=%d,tx_manu_id=%d,"
			"vendor_id=0x%x,product_id=0x%x,last_cep=%d,"
			"epp_to_bpp_connect_time=%lums,bt_connect=%d,bt_incar=%d,"
			"bt_car+=%d,bt_type=%d,bt_name=%s,ldo_on=%d,max_vrect=%d" "%s",
			trx_version, rx_version, wls_status->adapter_type,
			wls_status->adapter_id, wls_status->fastchg_started, wls_status->vout_mv,
			wls_status->iout_ma, wls_status->break_count, wls_status->trx_err,
			highest_temp, max_iout, max_vout, min_cool_down,
			min_skewing_current, !wls_status->verity_pass, wls_status->switch_quiet_mode,
			wls_dev->magcvr_status, wls_status->verify_by_aes, wls_status->tx_manu_id,
			wls_status->vendor_id, wls_status->product_id, wls_status->last_cep,
			wls_status->epp_to_bpp_connect_time, wls_dev->bt_info.connect, wls_dev->bt_info.incar,
			wls_dev->bt_info.carlink, wls_dev->bt_info.bttype, wls_dev->bt_info.name,
			wls_status->track_record.ldo_on, wls_status->track_record.max_vrect, v30_info_buf);
		chg_info("%s\n", crux_info);
	}
}

#define WLS_TRX_INFO_UPLOAD_THD_2MINS		(2 * 60)
#define WLS_LOCAL_T_NS_TO_S_THD			1000000000
#define WLS_TRX_INFO_THD_1MIN			60
static int oplus_chg_wls_get_local_time_s(void)
{
	int local_time_s;

	local_time_s = local_clock() / WLS_LOCAL_T_NS_TO_S_THD;
	chg_info("local_time_s:%d\n", local_time_s);

	return local_time_s;
}

static int oplus_chg_wls_track_upload_general_info(struct oplus_chg_wls *wls_dev, const char *chg_mode)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct mms_msg *msg;
	char *tx_crux_info;
	int rc;
	union mms_msg_data data = { 0 };
	int otg_online;
	struct wls_track_record *track_record = &wls_dev->wls_status.track_record;

	if (!is_err_topic_available(wls_dev)) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	tx_crux_info = kzalloc(OPLUS_CHG_TRACK_CURX_INFO_LEN, GFP_KERNEL);
	if (!tx_crux_info) {
		chg_err("tx_crux_info memery alloc fail\n");
		return -ENOMEM;
	}

	oplus_mms_get_item_data(wls_dev->wired_topic, WIRED_ITEM_OTG_ENABLE, &data, false);
	otg_online = data.intval;

	oplus_chg_wls_update_track_info(wls_dev, tx_crux_info, false);
	tx_crux_info[OPLUS_CHG_TRACK_CURX_INFO_LEN - 1] = '\0';
	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_WLS_INFO,
		"$$power_mode@@wireless"
		"$$wls_chg_mode@@%s"
		"$$total_time@@%d"
		"$$wired_online@@%d"
		"$$wired_type@@%d"
		"$$otg_online@@%d"
		"%s",
		(strcmp(chg_mode, "trx_mode") == 0 || strcmp(chg_mode, "rx_mode") == 0) ? chg_mode : "unknown",
		(wls_status->trx_transfer_end_time - wls_status->trx_transfer_start_time) / WLS_TRX_INFO_THD_1MIN,
		wls_status->trx_usb_present_once, track_record->wired_charge_type, otg_online,
		tx_crux_info);
	if (msg == NULL) {
		chg_err("alloc wls info msg error\n");
		kfree(tx_crux_info);
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(wls_dev->err_topic, msg);
	if (rc < 0) {
		chg_err("publish wls info msg error, rc=%d\n", rc);
		kfree(msg);
	}
	kfree(tx_crux_info);

	return rc;
}

static int oplus_chg_wls_track_upload_wls_err_info(struct oplus_chg_wls *wls_dev,
	enum wls_err_scene scene_type, enum wls_err_reason reason_type)
{
	char *rx_crux_info;
	char *high_temp_info = NULL;

	if (!is_err_topic_available(wls_dev)) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}
	if (scene_type >= ARRAY_SIZE(wls_err_scene_text) || scene_type < 0) {
		chg_err("wls err scene inval\n");
		return -EINVAL;
	}
	if (reason_type >= ARRAY_SIZE(wls_err_reason_text) || reason_type < 0) {
		chg_err("wls err reason inval\n");
		return -EINVAL;
	}

	rx_crux_info = kzalloc(OPLUS_CHG_TRACK_CURX_INFO_LEN, GFP_KERNEL);
	if (!rx_crux_info) {
		chg_err("rx_crux_info memery alloc fail\n");
		return -ENOMEM;
	}

	if (reason_type == WLS_ERR_HIGH_TEMP) {
		high_temp_info = kzalloc(HIGH_TEMP_INFO_LEN, GFP_KERNEL);
		if (!high_temp_info) {
			chg_err("high_temp_info memery alloc fail\n");
			kfree(rx_crux_info);
			return -ENOMEM;
		}
		memset(high_temp_info, 0, HIGH_TEMP_INFO_LEN);
		oplus_chg_wls_high_temp_update_track_info(wls_dev, high_temp_info);
		high_temp_info[HIGH_TEMP_INFO_LEN - 1] = '\0';
	}
	oplus_chg_wls_update_track_info(wls_dev, rx_crux_info, false);
	rx_crux_info[OPLUS_CHG_TRACK_CURX_INFO_LEN - 1] = '\0';
	oplus_chg_ic_creat_err_msg(wls_dev->wls_rx->rx_ic, OPLUS_IC_ERR_WLS_RX, 0,
		"$$err_scene@@%s$$err_reason@@%s"
		"%s""%s",
		wls_err_scene_text[scene_type], wls_err_reason_text[reason_type],
		rx_crux_info, high_temp_info == NULL ? "" : high_temp_info);
	oplus_chg_ic_virq_trigger(wls_dev->wls_rx->rx_ic, OPLUS_IC_VIRQ_ERR);
	kfree(rx_crux_info);
	if (!IS_ERR_OR_NULL(high_temp_info))
		kfree(high_temp_info);

	return 0;
}

static int oplus_chg_wls_set_trx_enable(struct oplus_chg_wls *wls_dev, bool en)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc = 0;
	int pre_trx_online = 0;

	if (en && wls_status->fw_upgrading) {
		chg_err("FW is upgrading, reverse charging cannot be used\n");
		return -EFAULT;
	}
	if (en && wls_dev->usb_present && !wls_dev->support_tx_boost &&
	    !wls_dev->support_wls_and_tx_boost && !wls_dev->support_acdrv_no_ext_boost) {
		chg_err("during USB charging, reverse charging cannot be used\n");
		return -EFAULT;
	}
	if (en && wls_status->rx_present) {
		chg_err("wireless charging, reverse charging cannot be used\n");
		return -EFAULT;
	}

	mutex_lock(&wls_dev->connect_lock);
	if (en) {
		oplus_chg_wls_rx_set_rx_mode_safety(wls_dev, OPLUS_CHG_WLS_RX_MODE_UNKNOWN);
		if (wls_status->wls_type == OPLUS_CHG_WLS_TRX)
			goto out;
		cancel_delayed_work_sync(&wls_dev->wls_connect_work);
		if (!wls_dev->usb_present && !wls_dev->vac_present &&
		    !(wls_dev->pre_otg_enable && wls_dev->support_acdrv_no_ext_boost)) {
			vote(wls_dev->insert_disable_votable, TRX_EN_VOTER, true, 1, false);
			/*for HW spec,need 20ms delay*/
			msleep(20);
		}
		if (!wls_dev->vac_present && !(wls_dev->pre_otg_enable && wls_dev->support_acdrv_no_ext_boost) &&
		    !(wls_dev->usb_present && wls_dev->support_acdrv_no_ext_boost)) {
			rc = oplus_chg_wls_nor_set_boost_vol(wls_dev->wls_nor->nor_ic, WLS_TRX_MODE_VOL_MV);
			if (rc < 0) {
				chg_err("can't set trx boost vol, rc=%d\n", rc);
				goto out;
			}
			if (wls_dev->wls_boost_curr_limit_ma > 0)
				oplus_chg_wls_nor_set_boost_curr_limit(wls_dev->wls_nor->nor_ic,
					wls_dev->wls_boost_curr_limit_ma);
			oplus_chg_wls_nor_set_boost_en(wls_dev->wls_nor->nor_ic, true);
			if (rc < 0) {
				chg_err("can't enable trx boost, rc=%d\n", rc);
				goto out;
			}
			msleep(500);
		}
		oplus_chg_wls_reset_variables(wls_dev);
		wls_status->wls_type = OPLUS_CHG_WLS_TRX;
		/*set trx_present true after power up for factory mode*/
		wls_status->trx_present = true;
		wls_status->trx_present_keep = false;
		wls_status->trx_rxac = false;
		oplus_chg_wls_rx_set_trx_start(wls_dev->wls_rx->rx_ic, true);

		if (!wls_dev->trx_wake_lock_on) {
			chg_info("acquire trx_wake_lock\n");
			__pm_stay_awake(wls_dev->trx_wake_lock);
			wls_dev->trx_wake_lock_on = true;
		} else {
			chg_err("trx_wake_lock is already stay awake\n");
		}
		cancel_delayed_work_sync(&wls_dev->wls_trx_sm_work);
		queue_delayed_work(wls_dev->wls_wq, &wls_dev->wls_trx_sm_work, 0);
	} else {
		if (wls_status->wls_type != OPLUS_CHG_WLS_TRX)
			goto out;
		if (wls_status->trx_rxac == true)
			vote(wls_dev->rx_disable_votable, RXAC_VOTER, true, 1, false);
		cancel_delayed_work_sync(&wls_dev->wls_trx_sm_work);
		oplus_chg_wls_rx_set_trx_start(wls_dev->wls_rx->rx_ic, false);
		if (!(wls_dev->pre_otg_enable && wls_dev->support_acdrv_no_ext_boost))
			oplus_chg_wls_nor_set_boost_en(wls_dev->wls_nor->nor_ic, false);
		wls_status->wls_type = OPLUS_CHG_WLS_UNKNOWN;
		wls_status->trx_present = false;
		pre_trx_online = wls_status->trx_online;
		wls_status->trx_online = false;
		msleep(100);
		if (wls_status->trx_rxac == true)
			vote(wls_dev->rx_disable_votable, RXAC_VOTER, false, 0, false);
		vote(wls_dev->insert_disable_votable, TRX_EN_VOTER, false, 0, false);
		wls_status->trx_transfer_end_time = oplus_chg_wls_get_local_time_s();
		chg_info("trx_online=%d, usb_present_once=%d,start_time=%d, end_time=%d\n",
			wls_status->trx_online, wls_status->trx_usb_present_once,
			wls_status->trx_transfer_start_time,
			wls_status->trx_transfer_end_time);
		if (pre_trx_online && (wls_status->trx_transfer_start_time &&
		   (wls_status->trx_transfer_end_time - wls_status->trx_transfer_start_time >
		    WLS_TRX_INFO_UPLOAD_THD_2MINS))) {
			oplus_chg_wls_track_upload_general_info(
				wls_dev, "trx_mode");
		}
		wls_status->track_record.wired_charge_type = 0;
		oplus_chg_wls_reset_variables(wls_dev);
		if (is_batt_psy_available(wls_dev))
			power_supply_changed(wls_dev->batt_psy);
		if (wls_dev->trx_wake_lock_on) {
			chg_info("release trx_wake_lock\n");
			__pm_relax(wls_dev->trx_wake_lock);
			wls_dev->trx_wake_lock_on = false;
		} else {
			chg_err("trx_wake_lock is already relax\n");
		}
	}

out:
	mutex_unlock(&wls_dev->connect_lock);
	return rc;
}

static void oplus_chg_wls_set_cool_down_iout(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int cool_down = wls_status->cool_down;
	int icl_ma = 0;
	int cool_down_max_step = 0;
	int index = 0;

	switch (wls_status->wls_type) {
	case OPLUS_CHG_WLS_BPP:
		index = OPLUS_WLS_CHG_MODE_BPP;
		break;
	case OPLUS_CHG_WLS_EPP:
		index = OPLUS_WLS_CHG_MODE_EPP;
		break;
	case OPLUS_CHG_WLS_EPP_PLUS:
		index = OPLUS_WLS_CHG_MODE_EPP_PLUS;
		break;
	case OPLUS_CHG_WLS_VOOC:
		index = OPLUS_WLS_CHG_MODE_AIRVOOC;
		break;
	case OPLUS_CHG_WLS_SVOOC:
	case OPLUS_CHG_WLS_PD_65W:
		index = OPLUS_WLS_CHG_MODE_AIRSVOOC;
		break;
	default:
		index = OPLUS_WLS_CHG_MODE_BPP;
		break;
	}

	cool_down_max_step = wls_dev->cool_down_step[index].max_step;
	if (cool_down >= cool_down_max_step && cool_down > 1)
		cool_down = cool_down_max_step - 1;
	icl_ma = dynamic_cfg->cool_down_step[index][cool_down];

	if (cool_down == 0 || wls_dev->factory_mode) {
		vote(wls_dev->nor_icl_votable, COOL_DOWN_VOTER, false, 0, false);
		vote(wls_dev->fcc_votable, COOL_DOWN_VOTER, false, 0, false);
		icl_ma = 0;
	} else {
		if (icl_ma > 0) {
			vote(wls_dev->nor_icl_votable, COOL_DOWN_VOTER, true, icl_ma, false);
			vote(wls_dev->fcc_votable, COOL_DOWN_VOTER, true, icl_ma, false);
		}
	}
	oplus_chg_wls_high_temp_check(HIGH_TEMP_VOTER_COOLDOWN, wls_dev, icl_ma);

	chg_info("icl_cool_down_ma=%d, cool_down_level=%d\n", icl_ma, wls_status->cool_down);
}

static enum oplus_chg_temp_region oplus_chg_wls_get_temp_region(struct oplus_chg_wls *wls_dev)
{
	enum oplus_chg_temp_region temp_region;
	int batt_temp;
	int rc;

	switch (wls_dev->wls_status.comm_temp_region) {
	case TEMP_REGION_COLD:
		temp_region = BATT_TEMP_COLD;
		break;
	case TEMP_REGION_LITTLE_COLD:
		temp_region = BATT_TEMP_LITTLE_COLD;
		break;
	case TEMP_REGION_COOL:
		temp_region = BATT_TEMP_COOL;
		break;
	case TEMP_REGION_LITTLE_COOL:
		temp_region = BATT_TEMP_LITTLE_COOL;
		break;
	case TEMP_REGION_PRE_NORMAL:
		temp_region = BATT_TEMP_PRE_NORMAL;
		break;
	case TEMP_REGION_NORMAL:
	case TEMP_REGION_NORMAL_HIGH:
		rc = oplus_chg_wls_get_batt_temp(wls_dev, &batt_temp);
		if (rc < 0 || batt_temp < wls_dev->normal_high_batt_temp)
			temp_region = BATT_TEMP_NORMAL;
		else if (batt_temp < wls_dev->little_warm_batt_temp)
			temp_region = BATT_TEMP_NORMAL_HIGH;
		else
			temp_region = BATT_TEMP_LITTLE_WARM;
		break;
	case TEMP_REGION_WARM:
		temp_region = BATT_TEMP_WARM;
		break;
	case TEMP_REGION_HOT:
		temp_region = BATT_TEMP_HOT;
		break;
	case TEMP_REGION_MAX:
		temp_region = BATT_TEMP_MAX - 1;
		break;
	default:
		temp_region = BATT_TEMP_NORMAL;
		break;
	}
	return temp_region;
}

static int oplus_chg_wls_choose_fastchg_curve(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	enum oplus_chg_temp_region temp_region;
	int batt_soc_plugin, batt_temp_plugin;
	int real_soc = 100;
	int rc = 0;
	int i = 0;

	temp_region = oplus_chg_wls_get_temp_region(wls_dev);

	if (temp_region <= BATT_TEMP_COOL) {
		batt_temp_plugin = WLS_FAST_TEMP_0_TO_50;
	} else if (temp_region == BATT_TEMP_LITTLE_COOL) {
		batt_temp_plugin = WLS_FAST_TEMP_50_TO_120;
	} else if (temp_region == BATT_TEMP_PRE_NORMAL) {
		batt_temp_plugin = WLS_FAST_TEMP_120_TO_160;
	} else if (temp_region == BATT_TEMP_NORMAL) {
		batt_temp_plugin = WLS_FAST_TEMP_160_TO_400;
	} else if (temp_region == BATT_TEMP_NORMAL_HIGH) {
		batt_temp_plugin = WLS_FAST_TEMP_160_TO_400_HIGH;
	} else if (temp_region == BATT_TEMP_LITTLE_WARM) {
		batt_temp_plugin = WLS_FAST_TEMP_400_TO_440;
	} else {
		batt_temp_plugin = WLS_FAST_TEMP_400_TO_440;
	}

	rc = oplus_chg_wls_get_real_soc(wls_dev, &real_soc);
	if ((rc < 0) || (real_soc >= dynamic_cfg->fastchg_max_soc)) {
		chg_err("can't get real soc or soc is too high, rc=%d\n", rc);
		return -EPERM;
	}

	for (i = 0; i < WLS_FAST_SOC_MAX; i++) {
		if (real_soc < dynamic_cfg->wls_strategy_soc[i]) {
			batt_soc_plugin = i;
			break;
		}
	}
	if (i == WLS_FAST_SOC_MAX)
		batt_soc_plugin = WLS_FAST_SOC_70_TO_90_HIGH;

	if ((batt_temp_plugin != WLS_FAST_TEMP_MAX)
			&& (batt_soc_plugin != WLS_FAST_SOC_MAX)) {
		if (wls_dev->wls_status.adapter_id != WLS_ADAPTER_THIRD_PARTY) {
			if (wls_dev->magcvr_status == MAGCVR_STATUS_NEAR) {
				wls_dev->wls_fcc_step.max_step =
				    wls_dev->fcc_steps_magcvr[batt_soc_plugin].fcc_step[batt_temp_plugin].max_step;
				memcpy(&wls_dev->wls_fcc_step.fcc_step,
				    wls_dev->fcc_steps_magcvr[batt_soc_plugin].fcc_step[batt_temp_plugin].fcc_step,
				    sizeof(struct oplus_chg_wls_range_data) * wls_dev->wls_fcc_step.max_step);
			} else {
				wls_dev->wls_fcc_step.max_step =
				    wls_dev->fcc_steps[batt_soc_plugin].fcc_step[batt_temp_plugin].max_step;
				memcpy(&wls_dev->wls_fcc_step.fcc_step,
				    wls_dev->fcc_steps[batt_soc_plugin].fcc_step[batt_temp_plugin].fcc_step,
				    sizeof(struct oplus_chg_wls_range_data) * wls_dev->wls_fcc_step.max_step);
			}
		} else {
			wls_dev->wls_fcc_step.max_step =
			    wls_dev->fcc_third_part_steps[batt_soc_plugin].fcc_step[batt_temp_plugin].max_step;
			memcpy(&wls_dev->wls_fcc_step.fcc_step,
			    wls_dev->fcc_third_part_steps[batt_soc_plugin].fcc_step[batt_temp_plugin].fcc_step,
			    sizeof(struct oplus_chg_wls_range_data) * wls_dev->wls_fcc_step.max_step);
		}
		for (i=0; i < wls_dev->wls_fcc_step.max_step; i++) {
			chg_info(" %d %d %d %d %d\n",
				wls_dev->wls_fcc_step.fcc_step[i].low_threshold,
				wls_dev->wls_fcc_step.fcc_step[i].high_threshold,
				wls_dev->wls_fcc_step.fcc_step[i].curr_ma,
				wls_dev->wls_fcc_step.fcc_step[i].vol_max_mv,
				wls_dev->wls_fcc_step.fcc_step[i].need_wait);
		}
	} else {
		return -EPERM;
	}

	return 0;
}

static int oplus_chg_wls_bcc_choose_curve(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	enum oplus_chg_temp_region temp_region;
	int batt_soc_plugin, batt_temp_plugin;
	int real_soc = 100;
	int rc = 0;
	int i = 0;

	temp_region = oplus_chg_wls_get_temp_region(wls_dev);

	if (temp_region <= BATT_TEMP_COOL)
		batt_temp_plugin = WLS_BCC_TEMP_0_TO_50;
	else if (temp_region == BATT_TEMP_LITTLE_COOL)
		batt_temp_plugin = WLS_BCC_TEMP_50_TO_120;
	else if (temp_region == BATT_TEMP_PRE_NORMAL)
		batt_temp_plugin = WLS_BCC_TEMP_120_TO_160;
	else if (temp_region == BATT_TEMP_NORMAL)
		batt_temp_plugin = WLS_BCC_TEMP_160_TO_400;
	else if (temp_region == BATT_TEMP_NORMAL_HIGH)
		batt_temp_plugin = WLS_BCC_TEMP_160_TO_400_HIGH;
	else if (temp_region == BATT_TEMP_LITTLE_WARM)
		batt_temp_plugin = WLS_BCC_TEMP_400_TO_440;
	else
		batt_temp_plugin = WLS_BCC_TEMP_400_TO_440;

	wls_status->bcc_temp_range = batt_temp_plugin;

	rc = oplus_chg_wls_get_real_soc(wls_dev, &real_soc);
	if ((rc < 0) || (real_soc >= dynamic_cfg->fastchg_max_soc)) {
		chg_err("can't get real soc or soc is too high, rc=%d\n", rc);
		return -EPERM;
	}

	for (i = 0; i < WLS_BCC_SOC_MAX; i++) {
		if (real_soc < dynamic_cfg->wls_strategy_soc[i]) {
			batt_soc_plugin = i;
			break;
		}
	}
	if (i == WLS_BCC_SOC_MAX)
		batt_soc_plugin = WLS_BCC_SOC_70_TO_90_HIGH;

	if ((batt_temp_plugin != WLS_BCC_TEMP_MAX) &&
	    (batt_soc_plugin != WLS_BCC_SOC_MAX)) {
		wls_dev->wls_bcc_step.max_step =
			wls_dev->bcc_steps[batt_soc_plugin].bcc_step[batt_temp_plugin].max_step;
		memcpy(&wls_dev->wls_bcc_step.bcc_step,
			wls_dev->bcc_steps[batt_soc_plugin].bcc_step[batt_temp_plugin].bcc_step,
			sizeof(struct oplus_chg_wls_bcc_data) * wls_dev->wls_bcc_step.max_step);
		for (i = 0; i < wls_dev->wls_bcc_step.max_step; i++) {
			chg_info(" %d %d %d %d\n",
				wls_dev->wls_bcc_step.bcc_step[i].max_batt_volt,
				wls_dev->wls_bcc_step.bcc_step[i].max_curr,
				wls_dev->wls_bcc_step.bcc_step[i].min_curr,
				wls_dev->wls_bcc_step.bcc_step[i].exit);
		}
	}

	return 0;
}

static void oplus_chg_wls_bcc_get_curve(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_bcc_step *bcc_chg = &wls_dev->wls_bcc_step;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int bcc_batt_volt_now;
	int i, rc;
	int curve_idx = 0;

	rc = oplus_chg_wls_bcc_choose_curve(wls_dev);
	if (rc < 0) {
		chg_err("choose bcc curve failed, rc = %d\n", rc);
		return;
	}

	rc = oplus_chg_wls_get_vbat(wls_dev, &bcc_batt_volt_now);
	if (rc < 0) {
		chg_err("can't get batt vol, rc=%d\n", rc);
		return;
	}

	for (i = 0; i < wls_dev->wls_bcc_step.max_step; i++) {
		chg_debug("bcc_batt is %d target_volt is %d\n", bcc_batt_volt_now,
			bcc_chg->bcc_step[i].max_batt_volt);
		if (bcc_batt_volt_now < bcc_chg->bcc_step[i].max_batt_volt) {
			curve_idx = i;
			chg_info("curve idx = %d\n", curve_idx);
			break;
		}
	}

	wls_status->wls_bcc_max_curr = bcc_chg->bcc_step[curve_idx].max_curr;
	wls_status->wls_bcc_min_curr = bcc_chg->bcc_step[curve_idx].min_curr;
	chg_info("choose max curr is %d, min curr is %d\n", wls_status->wls_bcc_max_curr, wls_status->wls_bcc_min_curr);
}

#define CURVE_CHANGE_COUNT 20
static void oplus_chg_wls_bcc_curr_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev =
		container_of(dwork, struct oplus_chg_wls, wls_bcc_curr_update_work);
	struct oplus_chg_wls_bcc_step *bcc_chg = &wls_dev->wls_bcc_step;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int bcc_batt_volt_now;
	int i, rc;
	static int pre_curve_idx = 0;
	static int idx_cnt = 0;

	rc = oplus_chg_wls_get_vbat(wls_dev, &bcc_batt_volt_now);
	if (rc < 0) {
		chg_err("can't get batt vol, rc=%d\n", rc);
		return;
	}

	for (i = wls_status->bcc_true_idx; i < wls_dev->wls_bcc_step.max_step; i++) {
		chg_debug("bcc_batt is %d target_volt now is %d\n", bcc_batt_volt_now,
			bcc_chg->bcc_step[i].max_batt_volt);
		if (bcc_batt_volt_now < bcc_chg->bcc_step[i].max_batt_volt) {
			wls_status->bcc_curve_idx = i;
			chg_info("curve idx = %d\n", wls_status->bcc_curve_idx);
			break;
		}
	}

	if (pre_curve_idx == wls_status->bcc_curve_idx) {
		idx_cnt = idx_cnt + 1;
		if (idx_cnt >= CURVE_CHANGE_COUNT) {
			wls_status->wls_bcc_max_curr = bcc_chg->bcc_step[wls_status->bcc_curve_idx].max_curr;
			wls_status->wls_bcc_min_curr = bcc_chg->bcc_step[wls_status->bcc_curve_idx].min_curr;
			wls_status->bcc_true_idx = wls_status->bcc_curve_idx;
			chg_info("choose max curr is %d, min curr is %d true idx is %d\n",
				wls_status->wls_bcc_max_curr, wls_status->wls_bcc_min_curr, wls_status->bcc_true_idx);
		}
	} else {
		idx_cnt = 0;
	}
	pre_curve_idx = wls_status->bcc_curve_idx;

	schedule_delayed_work(&wls_dev->wls_bcc_curr_update_work, OPLUS_WLS_BCC_UPDATE_INTERVAL);
}

static bool oplus_chg_wls_wake_bcc_update_work(struct oplus_chg_wls *wls_dev)
{
	if (!wls_dev)
		return true;

	schedule_delayed_work(&wls_dev->wls_bcc_curr_update_work, OPLUS_WLS_BCC_UPDATE_INTERVAL);

	return true;
}

static void oplus_chg_wls_cancel_bcc_update_work(struct oplus_chg_wls *wls_dev)
{
	if (!wls_dev)
		return;

	cancel_delayed_work_sync(&wls_dev->wls_bcc_curr_update_work);
}

#define OPLUS_WLS_BCC_MAX_CURR_INIT 5000
#define OPLUS_WLS_BCC_MIN_CURR_INIT 4000
#define OPLUS_WLS_BCC_STOP_CURR_INIT 1000
static void oplus_chg_wls_bcc_parms_init(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (!wls_dev)
		return;

	wls_status->wls_bcc_max_curr = OPLUS_WLS_BCC_MAX_CURR_INIT;
	wls_status->wls_bcc_min_curr = OPLUS_WLS_BCC_MIN_CURR_INIT;
	wls_status->wls_bcc_stop_curr = OPLUS_WLS_BCC_STOP_CURR_INIT;
	wls_status->bcc_curve_idx = 0;
	wls_status->bcc_true_idx = 0;
	wls_status->bcc_temp_range = 0;
}

static int oplus_chg_wls_get_ffc_status(struct oplus_chg_wls *wls_dev)
{
	int ffc_status = FFC_DEFAULT;
	union mms_msg_data data = { 0 };
	int rc;

	if (!wls_dev || !wls_dev->comm_topic)
		return FFC_DEFAULT;

	rc = oplus_mms_get_item_data(wls_dev->comm_topic, COMM_ITEM_FFC_STATUS, &data, true);
	if (!rc)
		ffc_status = data.intval;

	return ffc_status;
}

static void oplus_chg_wls_config(struct oplus_chg_wls *wls_dev)
{
	enum oplus_chg_temp_region temp_region;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	enum oplus_chg_ffc_status ffc_status;
	int icl_max_ma;
	int fcc_max_ma;
	int icl_index;
	static bool pre_temp_abnormal;

	ffc_status = oplus_chg_wls_get_ffc_status(wls_dev);
	if (ffc_status != FFC_DEFAULT) {
		chg_err("ffc charging, exit\n");
		return;
	}

	if (!wls_status->rx_present && !wls_status->online_keep) {
		chg_err("rx is offline\n");
		return;
	}

	/*in comm module, if gauge_topic is null in bootup, batt_temp is GAUGE_INVALID_TEMP, that is TEMP_REGION_COLD*/
	if (!wls_dev->gauge_topic) {
		chg_err("gauge_topic is null\n");
		return;
	}

	if (wls_status->fcc_gear == FCC_GEAR_HIGH)
		icl_index = OPLUS_WLS_CHG_BATT_CL_HIGH;
	else
		icl_index = OPLUS_WLS_CHG_BATT_CL_LOW;

	temp_region = oplus_chg_wls_get_temp_region(wls_dev);
	switch (temp_region) {
	case BATT_TEMP_COLD:
	case BATT_TEMP_HOT:
		if (!pre_temp_abnormal) {
			pre_temp_abnormal = true;
			wls_status->online_keep = true;
			vote(wls_dev->rx_disable_votable, JEITA_VOTER, true, 1, false);
			vote(wls_dev->nor_icl_votable, JEITA_VOTER, true, WLS_CURR_JEITA_CHG_MA, true);
			vote(wls_dev->nor_out_disable_votable, JEITA_VOTER, true, 1, false);
			schedule_delayed_work(&wls_dev->rx_restore_work, msecs_to_jiffies(500));
		} else {
			vote(wls_dev->rx_disable_votable, JEITA_VOTER, false, 0, false);
			vote(wls_dev->nor_icl_votable, JEITA_VOTER, true, WLS_CURR_JEITA_CHG_MA, true);
		}
		break;
	default:
		pre_temp_abnormal = false;
		vote(wls_dev->rx_disable_votable, JEITA_VOTER, false, 0, false);
		vote(wls_dev->nor_icl_votable, JEITA_VOTER, false, 0, true);
		vote(wls_dev->nor_out_disable_votable, JEITA_VOTER, false, 0, false);
		break;
	}

	if (wls_status->wls_type == OPLUS_CHG_WLS_VOOC) {
		if (wls_status->temp_region == BATT_TEMP_COOL) {
			if (temp_region == BATT_TEMP_LITTLE_COLD) {
				wls_status->online_keep = true;
				schedule_delayed_work(&wls_dev->rx_restart_work, 0);
			}
		}

		if (wls_status->temp_region == BATT_TEMP_LITTLE_WARM) {
			if (temp_region == BATT_TEMP_WARM) {
				wls_status->online_keep = true;
				schedule_delayed_work(&wls_dev->rx_restart_work, 0);
			}
		}
	}

	wls_status->temp_region = temp_region;

	switch (wls_status->wls_type) {
	case OPLUS_CHG_WLS_BPP:
		icl_max_ma = dynamic_cfg->iclmax_ma[icl_index][OPLUS_WLS_CHG_MODE_BPP][temp_region];
		fcc_max_ma = 1200;
		break;
	case OPLUS_CHG_WLS_EPP:
		icl_max_ma = dynamic_cfg->iclmax_ma[icl_index][OPLUS_WLS_CHG_MODE_EPP][temp_region];
		fcc_max_ma = 2800;
		break;
	case OPLUS_CHG_WLS_EPP_PLUS:
		icl_max_ma = dynamic_cfg->iclmax_ma[icl_index][OPLUS_WLS_CHG_MODE_EPP_PLUS][temp_region];
		fcc_max_ma = 4000;
		break;
	case OPLUS_CHG_WLS_VOOC:
		icl_max_ma = dynamic_cfg->iclmax_ma[icl_index][OPLUS_WLS_CHG_MODE_AIRVOOC][temp_region];
		fcc_max_ma = 3600;
		break;
	case OPLUS_CHG_WLS_SVOOC:
	case OPLUS_CHG_WLS_PD_65W:
		icl_max_ma = dynamic_cfg->iclmax_ma[icl_index][OPLUS_WLS_CHG_MODE_AIRSVOOC][temp_region];
		fcc_max_ma = 3600;
		break;
	default:
		chg_err("Unsupported charging mode(=%d)\n", wls_status->wls_type);
		return;
	}
	oplus_chg_wls_set_cool_down_iout(wls_dev);

	vote(wls_dev->nor_icl_votable, MAX_VOTER, true, icl_max_ma, true);
	vote(wls_dev->nor_fcc_votable, MAX_VOTER, true, fcc_max_ma, false);
	chg_info("chg_type=%d, temp_region=%d, fcc=%d, icl_max=%d\n",
		 wls_status->wls_type, temp_region, fcc_max_ma, icl_max_ma);
}

#ifdef OPLUS_CHG_DEBUG
#define UPGRADE_START 0
#define UPGRADE_FW    1
#define UPGRADE_END   2
struct oplus_chg_fw_head {
	u8 magic[4];
	int size;
};

ssize_t oplus_chg_wls_upgrade_fw_show(struct oplus_mms *mms, char *buf)
{
	int rc = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}

	rc = sprintf(buf, "wireless\n");
	return rc;
}

ssize_t oplus_chg_wls_upgrade_fw_store(struct oplus_mms *mms, const char *buf, size_t count)
{
	struct oplus_chg_wls *wls_dev = NULL;
	u8 temp_buf[sizeof(struct oplus_chg_fw_head)];
	static u8 *fw_buf;
	static int upgrade_step = UPGRADE_START;
	static int fw_index;
	static int fw_size;
	struct oplus_chg_fw_head *fw_head;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}
	wls_dev = oplus_mms_get_drvdata(mms);
start:
	switch (upgrade_step) {
	case UPGRADE_START:
		if (count < sizeof(struct oplus_chg_fw_head)) {
			chg_err("<FW UPDATE>image format error\n");
			return -EINVAL;
		}
		memset(temp_buf, 0, sizeof(struct oplus_chg_fw_head));
		memcpy(temp_buf, buf, sizeof(struct oplus_chg_fw_head));
		fw_head = (struct oplus_chg_fw_head *)temp_buf;
		if (fw_head->magic[0] == 0x02 && fw_head->magic[1] == 0x00 &&
		    fw_head->magic[2] == 0x03 && fw_head->magic[3] == 0x00) {
			fw_size = fw_head->size;
			fw_buf = kzalloc(fw_size, GFP_KERNEL);
			if (fw_buf == NULL) {
				chg_err("<FW UPDATE>alloc fw_buf err\n");
				return -ENOMEM;
			}
			wls_dev->fw_buf = fw_buf;
			wls_dev->fw_size = fw_size;
			chg_info("<FW UPDATE>image header verification succeeded, fw_size=%d\n", fw_size);
			memcpy(fw_buf, buf + sizeof(struct oplus_chg_fw_head), count - sizeof(struct oplus_chg_fw_head));
			fw_index = count - sizeof(struct oplus_chg_fw_head);
			chg_info("<FW UPDATE>Receiving image, fw_size=%d, fw_index=%d\n", fw_size, fw_index);
			if (fw_index >= fw_size) {
				upgrade_step = UPGRADE_END;
				goto start;
			} else {
				upgrade_step = UPGRADE_FW;
			}
		} else {
			chg_err("<FW UPDATE>image format error\n");
			return -EINVAL;
		}
		break;
	case UPGRADE_FW:
		memcpy(fw_buf + fw_index, buf, count);
		fw_index += count;
		chg_info("<FW UPDATE>Receiving image, fw_size=%d, fw_index=%d\n", fw_size, fw_index);
		if (fw_index >= fw_size) {
			upgrade_step = UPGRADE_END;
			goto start;
		}
		break;
	case UPGRADE_END:
		wls_dev->fw_upgrade_by_buf = true;
		schedule_delayed_work(&wls_dev->wls_upgrade_fw_work, 0);
		fw_buf = NULL;
		upgrade_step = UPGRADE_START;
		break;
	default:
		upgrade_step = UPGRADE_START;
		chg_err("<FW UPDATE>status error\n");
		if (fw_buf != NULL) {
			kfree(fw_buf);
			fw_buf = NULL;
			wls_dev->fw_buf = NULL;
			wls_dev->fw_size = 0;
			wls_dev->fw_upgrade_by_buf = false;
		}
		break;
	}

	return count;
}

int oplus_chg_wls_get_break_sub_crux_info(struct oplus_mms *mms, char *crux_info)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}
	if (crux_info == NULL) {
		chg_err("crux_info is NULL");
		return -ENODEV;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	chg_info("start\n");
	oplus_chg_wls_update_track_info(wls_dev, crux_info, false);
	return 0;
}

static int oplus_chg_wls_get_max_wireless_power(struct oplus_chg_wls *wls_dev)
{
	int max_wls_power = 0;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int max_wls_base_power = 0;
	int max_wls_r_power = 0;

	if (!wls_status || !wls_status->rx_online)
		return 0;

	switch (wls_status->wls_type) {
	case OPLUS_CHG_WLS_BPP:
		max_wls_power = WLS_BPP_PWR_MAX_MW;
		break;
	case OPLUS_CHG_WLS_EPP:
		max_wls_power = WLS_EPP_PWR_MAX_MW;
		break;
	case OPLUS_CHG_WLS_EPP_PLUS:
		max_wls_power = WLS_EPP_PLUS_PWR_MAX_MW;
		break;
	case OPLUS_CHG_WLS_VOOC:
		max_wls_power = WLS_VOOC_PWR_MAX_MW;
		break;
	case OPLUS_CHG_WLS_SVOOC:
	case OPLUS_CHG_WLS_PD_65W:
		max_wls_base_power = oplus_chg_wls_get_base_power_max(wls_status->adapter_id);
		if (wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY)
			max_wls_r_power = oplus_chg_wls_get_r_power(wls_dev, wls_status->adapter_power);
		else
			max_wls_r_power = oplus_chg_wls_get_tripartite_r_power(wls_status->adapter_power);

		max_wls_power = max_wls_base_power > max_wls_r_power ? max_wls_r_power : max_wls_base_power;
		break;
	default:
		max_wls_power = WLS_BPP_PWR_MAX_MW;
		break;
	}

	return max_wls_power;
}
#endif /* OPLUS_CHG_DEBUG */

#define CLIENT_STR_LEN		32
#define RX_DISABLE_PARAM_NUM	2
#define RX_DISABLE		1
#define RX_ENABLE		0
#define CALL_NAME_BATTERY	2
#define CALL_NAME_TEST		5
ssize_t oplus_chg_wls_rx_disable_show(struct oplus_mms *mms, char *buf)
{
	struct oplus_chg_wls *wls_dev = NULL;
	char client_name[CLIENT_STR_LEN] = { '\0' };
	int rx_status = 0;

	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}
	if (!mms) {
		chg_err("mms is NULL\n");
		return -ENODEV;
	}

	wls_dev = oplus_mms_get_drvdata(mms);
	snprintf(client_name, CLIENT_STR_LEN - 1, "CALL_NAME_%d", wls_dev->callname);
	rx_status = is_client_vote_enabled(wls_dev->rx_disable_votable, client_name);

	return sprintf(buf, "callname=%d:%d\n", wls_dev->callname, rx_status);
}

ssize_t oplus_chg_wls_rx_disable_store(struct oplus_mms *mms, const char *buf, size_t count)
{
	struct oplus_chg_wls *wls_dev = NULL;
	char client_name[CLIENT_STR_LEN] = { '\0' };
	int config = -1;
	int callname = 0;

	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}
	if (!mms) {
		chg_err("mms is NULL\n");
		return -ENODEV;
	}

	wls_dev = oplus_mms_get_drvdata(mms);
	if (sscanf(buf, "disable=%dcallname=%d", &config, &callname) != RX_DISABLE_PARAM_NUM) {
		chg_err("buf format error\n");
		return -EINVAL;
	}

	if (config != RX_ENABLE && config != RX_DISABLE) {
		chg_err("[disable=%d] is not support!\n", config);
		return -EINVAL;
	}

	switch (callname) {
	case CALL_NAME_BATTERY:
	case CALL_NAME_TEST:
		wls_dev->callname = callname;
		snprintf(client_name, CLIENT_STR_LEN - 1, "CALL_NAME_%d", wls_dev->callname);
		break;
	default:
		chg_info("[callname=%d] is not support!\n", callname);
		return -EINVAL;
	}

	if (config == RX_DISABLE)
		oplus_chg_wls_rx_set_rx_mode_safety(wls_dev, OPLUS_CHG_WLS_RX_MODE_UNKNOWN);
	chg_info("callname=%d:disable=%d\n", callname, config);
	vote(wls_dev->rx_disable_votable, client_name, (config == RX_DISABLE) ? true : false, config, false);

	return count;
}

ssize_t oplus_chg_wls_bt_info_store(struct oplus_mms *mms, const char *buf, size_t count)
{
	struct oplus_chg_wls *wls_dev = NULL;
	char test_buf[4] = { '\0' };
	char *name_str = NULL;
	int parsed_params = 0;

	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}
	if (!mms) {
		chg_err("mms is NULL\n");
		return -ENODEV;
	}

	wls_dev = oplus_mms_get_drvdata(mms);
	parsed_params = sscanf(buf, "btconnect=%dbtincar=%dcarlink=%dbttype=%dbtname=%1s",
	                      &wls_dev->bt_info.connect,
	                      &wls_dev->bt_info.incar,
	                      &wls_dev->bt_info.carlink,
	                      &wls_dev->bt_info.bttype,
	                      test_buf);

	if (parsed_params != 5) {
		parsed_params = sscanf(buf, "btconnect=%dbtincar=%dcarlink=%dbtname=%1s",
		                       &wls_dev->bt_info.connect,
		                       &wls_dev->bt_info.incar,
		                       &wls_dev->bt_info.carlink,
		                       test_buf);
		if (parsed_params == 4) {
			wls_dev->bt_info.bttype = 0;
		} else {
			chg_err("buf format error\n");
			return -EINVAL;
		}
	}
	name_str = strstr(buf, "btname=");
	if (name_str) {
		name_str = name_str + strlen("btname=");
		memset(wls_dev->bt_info.name, '\0', sizeof(wls_dev->bt_info.name));
		strncpy(wls_dev->bt_info.name, name_str, sizeof(wls_dev->bt_info.name) - 1);
	}
	chg_info("connect[%d], incar[%d], car+[%d], bttype[%d], name[%s]\n", wls_dev->bt_info.connect, wls_dev->bt_info.incar,
		wls_dev->bt_info.carlink, wls_dev->bt_info.bttype, wls_dev->bt_info.name);

	return count;
}

static int oplus_chg_wls_mms_update_project_power(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = (int)wls_dev->wls_power_mw;

	return 0;
}

static int oplus_chg_wls_ui_power(struct oplus_mms *mms, union mms_msg_data *data)
{
	int ui_power = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	/* wls no need to show ui power, ui_power allways return 0
	wls_dev = oplus_mms_get_drvdata(mms);
	if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30 &&
		wls_dev->wls_status.adapter_id >= WLS_ADAPTER_MODEL_4 &&
		wls_dev->wls_status.adapter_id != WLS_ADAPTER_THIRD_PARTY) {
		ui_power = oplus_chg_wls_get_max_wireless_power(wls_dev);
		if (wls_dev->wls_power_mw > ui_power)
			ui_power = 0;
	} else {
		ui_power = 0;
	}*/
	data->intval = ui_power;

	return 0;
}

static void oplus_chg_wls_track_record_reset(struct oplus_chg_wls *wls_dev)
{
	struct wls_track_record *track_record = &wls_dev->wls_status.track_record;

	track_record->ldo_on = false;
	track_record->wired_charge_type = 0;
	track_record->rx_mode = OPLUS_CHG_WLS_RX_MODE_UNKNOWN;
	track_record->is_fastchg = false;

	return;
}

static int oplus_chg_wls_wireless_notifier_call(struct oplus_chg_wls *wls_dev, enum wls_topic_item id)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	union mms_msg_data data = { 0 };
	enum oplus_chg_wls_rx_mode rx_mode = OPLUS_CHG_WLS_RX_MODE_UNKNOWN;

	switch (id) {
	case WLS_ITEM_PRESENT:
		oplus_mms_get_item_data(wls_dev->wls_topic, WLS_ITEM_PRESENT, &data, true);
		if (data.intval == 0) {
			oplus_mms_get_item_data(wls_dev->wls_topic, WLS_ITEM_TRACK_RECORD_WLS_TYPE, &data, true);
			return 0;
		}
		chg_info("wls present\n");
		wls_status->boot_online_keep = false;
		if (wls_status->rx_present)
			break;
		wls_status->rx_present = true;
		wls_status->online_keep = false;
		oplus_chg_wls_nor_set_vindpm(wls_dev->wls_nor->nor_ic, WLS_VINDPM_BPP);
		oplus_chg_wls_rx_get_rx_mode(wls_dev->wls_rx->rx_ic, &rx_mode);
		if (rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_5W || rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP ||
		    rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_PLUS)
			(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
				oplus_chg_wls_get_fod_parm(wls_dev, wls_status->adapter_id, FOD_EPP_MODE),
				wls_dev->static_config.fod_parm_len,
				FOD_EPP_MODE, wls_dev->magcvr_status);
		else
			(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
				oplus_chg_wls_get_fod_parm(wls_dev, wls_status->adapter_id, FOD_BPP_MODE),
				wls_dev->static_config.fod_parm_len,
				FOD_BPP_MODE, wls_dev->magcvr_status);
		if (is_batt_psy_available(wls_dev))
			power_supply_changed(wls_dev->batt_psy);

		oplus_chg_wls_track_record_reset(wls_dev);
		wls_status->track_record.rx_mode = rx_mode;
		break;
	case WLS_ITEM_ONLINE:
		oplus_mms_get_item_data(wls_dev->wls_topic, WLS_ITEM_ONLINE, &data, true);
		if (data.intval) {
			chg_info("wls online\n");
			wls_status->boot_online_keep = false;
			if (wls_status->rx_online)
				break;
			wls_status->rx_online = true;
			wls_status->online_keep = false;
			wls_status->track_record.ldo_on = true;
			wls_status->track_record.break_manu = false;
			/*if (adsp_started)*/
				schedule_delayed_work(&wls_dev->wls_connect_work, 0);
			/*else
				online_pending = true;*/
			if (is_batt_psy_available(wls_dev))
				power_supply_changed(wls_dev->batt_psy);
			schedule_delayed_work(&wls_dev->wls_start_work, 0);
		} else {
			chg_info("wls offline\n");
			wls_status->boot_online_keep = false;
			if (!wls_status->rx_online && !wls_status->rx_present)
				break;
			wls_status->rx_present = false;
			wls_status->rx_online = false;

			oplus_mms_get_item_data(wls_dev->wls_topic, WLS_ITEM_STATUS_KEEP, &data, true);
			if (data.intval != WLS_SK_BY_FORCE_BPP)
				oplus_chg_wls_set_status_keep(wls_dev->wls_topic, WLS_SK_NULL);
			schedule_delayed_work(&wls_dev->wls_connect_work, 0);
			cancel_delayed_work(&wls_dev->wls_start_work);
			schedule_delayed_work(&wls_dev->wls_break_work,
				msecs_to_jiffies(OPLUS_CHG_WLS_BREAK_DETECT_DELAY));
			if (is_batt_psy_available(wls_dev))
				power_supply_changed(wls_dev->batt_psy);
		}
		break;
	default:
		chg_info("unknown wls_topic_item[%d]\n", id);
		break;
	}
	return 0;
}

static void oplus_chg_wls_usb_int_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, usb_int_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (wls_dev->usb_present) {
		oplus_chg_wls_rx_set_rx_mode_safety(wls_dev, OPLUS_CHG_WLS_RX_MODE_UNKNOWN);
		vote(wls_dev->rx_disable_votable, USB_VOTER, true, 1, false);
		rerun_election(wls_dev->rx_disable_votable, true);
		if (!wls_dev->support_tx_boost && !wls_dev->support_wls_and_tx_boost && !wls_dev->support_no_hw_interlock) {
			mutex_lock(&wls_dev->trx_enable_lock);
			(void)oplus_chg_wls_set_trx_enable(wls_dev, false);
			mutex_unlock(&wls_dev->trx_enable_lock);
		}
		/*oplus_chg_anon_mod_event(wls_dev->wls_ocm, OPLUS_CHG_EVENT_OFFLINE);*/
		if (wls_status->trx_present && (wls_dev->support_wls_and_tx_boost || wls_dev->support_no_hw_interlock)) {
			if (wls_status->wls_type == OPLUS_CHG_WLS_TRX)
				wls_status->trx_present_keep = true;
			mutex_lock(&wls_dev->trx_enable_lock);
			(void)oplus_chg_wls_set_trx_enable(wls_dev, false);
			/*for HW spec,need 100ms delay*/
			msleep(100);
			(void)oplus_chg_wls_set_trx_enable(wls_dev, true);
			mutex_unlock(&wls_dev->trx_enable_lock);
		}
	} else {
		vote(wls_dev->rx_disable_votable, USB_VOTER, false, 0, false);
		if (wls_status->trx_present && wls_dev->support_acdrv_no_ext_boost) {
			if (wls_dev->support_no_hw_interlock)
				vote(wls_dev->insert_disable_votable, USB_VOTER, true, 1, false);
			if (wls_status->wls_type == OPLUS_CHG_WLS_TRX)
				wls_status->trx_present_keep = true;
			mutex_lock(&wls_dev->trx_enable_lock);
			(void)oplus_chg_wls_set_trx_enable(wls_dev, false);
			/*for HW spec,need 100ms delay*/
			msleep(100);
			(void)oplus_chg_wls_set_trx_enable(wls_dev, true);
			mutex_unlock(&wls_dev->trx_enable_lock);
			if (wls_dev->support_no_hw_interlock)
				vote(wls_dev->insert_disable_votable, USB_VOTER, false, 0, false);
		}
	}
}

static void oplus_chg_wls_vac_int_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, vac_int_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (wls_dev->vac_present) {
		vote(wls_dev->rx_disable_votable, USB_VOTER, true, 1, false);
		if (wls_status->wls_type == OPLUS_CHG_WLS_TRX)
			wls_status->trx_present_keep = true;
		mutex_lock(&wls_dev->trx_enable_lock);
		(void)oplus_chg_wls_set_trx_enable(wls_dev, false);
		/*for HW spec,need 100ms delay*/
		msleep(100);
		(void)oplus_chg_wls_set_trx_enable(wls_dev, true);
		mutex_unlock(&wls_dev->trx_enable_lock);
	} else {
		vote(wls_dev->rx_disable_votable, USB_VOTER, false, 0, false);
		if (wls_status->trx_present) {
			if (wls_status->wls_type == OPLUS_CHG_WLS_TRX)
				wls_status->trx_present_keep = true;
			vote(wls_dev->insert_disable_votable, USB_VOTER, true, 1, false);
			mutex_lock(&wls_dev->trx_enable_lock);
			(void)oplus_chg_wls_set_trx_enable(wls_dev, false);
			/*for HW spec,need 100ms delay*/
			msleep(100);
			(void)oplus_chg_wls_set_trx_enable(wls_dev, true);
			mutex_unlock(&wls_dev->trx_enable_lock);
			vote(wls_dev->insert_disable_votable, USB_VOTER, false, 0, false);
		}
	}
}

#define OPLUS_CHG_WLS_START_DETECT_CNT 10
static void oplus_chg_wls_start_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_start_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	static int wpc_online_cnt;

	if (wls_status->rx_online) {
		wpc_online_cnt++;
		if (wpc_online_cnt >= OPLUS_CHG_WLS_START_DETECT_CNT) {
			wpc_online_cnt = 0;
			oplus_chg_wls_update_track_info(wls_dev, NULL, false);
			chg_info("wireless chg start after connect 30s\n");
		} else {
			schedule_delayed_work(&wls_dev->wls_start_work,
				round_jiffies_relative(msecs_to_jiffies(OPLUS_CHG_WLS_START_DETECT_DELAY)));
		}
	} else {
		chg_info("wireless chg not online within connect 30s, cnt=%d\n", wpc_online_cnt);
		wpc_online_cnt = 0;
	}
}

static void oplus_chg_wls_break_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_break_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (wls_status->rx_online) {
		wls_status->break_count++;
		chg_info("wireless disconnect less than 6s, count=%d\n", wls_status->break_count);
	} else {
		chg_info("wireless disconnect more than 6s, charging stop\n");
		wls_status->break_count = 0;
		oplus_chg_wls_update_track_info(wls_dev, NULL, true);
	}
}

static void oplus_chg_wls_connect_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_connect_work);
	/*struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;*/
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_wls_chg_rx *wls_rx = wls_dev->wls_rx;
	struct oplus_wls_chg_normal *wls_nor = wls_dev->wls_nor;
	unsigned long delay_time = jiffies + msecs_to_jiffies(500);
	int skin_temp;
	int rc;

	if (wls_status->rx_online) {
		chg_err("!!!!!wls connect >>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
		if (!wls_dev->rx_wake_lock_on) {
			chg_info("acquire rx_wake_lock\n");
			__pm_stay_awake(wls_dev->rx_wake_lock);
			wls_dev->rx_wake_lock_on = true;
		} else {
			chg_err("rx_wake_lock is already stay awake\n");
		}

		cancel_delayed_work_sync(&wls_dev->wls_clear_trx_work);
		oplus_chg_wls_reset_variables(wls_dev);
		/*
		 * The verity_state_keep flag needs to be cleared immediately
		 * after reconnection to ensure that the subsequent verification
		 * function is normal.
		 */
		wls_status->verity_state_keep = false;
		wls_status->fastchg_to_bpp_state_keep = false;
		chg_info("nor_fcc_votable: client:%s, result=%d\n",
			get_effective_client(wls_dev->nor_fcc_votable),
			get_effective_result(wls_dev->nor_fcc_votable));
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 1500, false);
		rerun_election(wls_dev->nor_fcc_votable, false);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 100, false);
		rerun_election(wls_dev->nor_icl_votable, false);
		vote(wls_dev->rx_comu_votable, USER_VOTER, true, 0, false);
		rerun_election(wls_dev->rx_comu_votable, false);
		/* reset charger status */
		vote(wls_dev->nor_out_disable_votable, USER_VOTER, true, 1, false);
		vote(wls_dev->nor_out_disable_votable, USER_VOTER, false, 0, false);
		schedule_delayed_work(&wls_dev->wls_data_update_work, 0);
		schedule_delayed_work(&wls_dev->wls_vout_err_work, 0);
		queue_delayed_work(wls_dev->wls_wq, &wls_dev->wls_rx_sm_work, 0);
		rc = oplus_chg_wls_get_skin_temp(wls_dev, &skin_temp);
		if (rc < 0) {
			chg_err("can't get skin temp, rc=%d\n", rc);
			skin_temp = 250;
		}
		wls_dev->high_temp_track.wls_start_time = jiffies;
		schedule_delayed_work(&wls_dev->wls_monitor_work, msecs_to_jiffies(OPLUS_CHG_WLS_MONITOR_DELAY));
	} else {
		chg_err("!!!!!wls disconnect <<<<<<<<<<<<<<<<<<<<<<<<<<\n");
		vote(wls_dev->rx_disable_votable, CONNECT_VOTER, true, 1, false);
		if (wls_dev->support_fastchg) {
			(void)oplus_chg_wls_rx_set_dcdc_enable(wls_dev->wls_rx->rx_ic, false);
			msleep(10);
			oplus_chg_wls_set_ext_pwr_enable(wls_dev, false);
			oplus_chg_wls_fast_set_enable(wls_dev->wls_fast->fast_ic, false);
		}

		/* should clean all resource first after disconnect */
		(void)oplus_chg_wls_clean_msg(wls_dev);
		(void)oplus_chg_wls_rx_clean_source(wls_rx);
		(void)oplus_chg_wls_nor_clean_source(wls_nor);
		cancel_delayed_work_sync(&wls_dev->wls_data_update_work);
		cancel_delayed_work_sync(&wls_dev->wls_rx_sm_work);
		cancel_delayed_work_sync(&wls_dev->wls_trx_sm_work);
		cancel_delayed_work_sync(&wls_dev->wls_verity_work);
		cancel_delayed_work_sync(&wls_dev->wls_skewing_work);
		cancel_delayed_work_sync(&wls_dev->wls_vout_err_work);
		cancel_delayed_work(&wls_dev->wls_monitor_work);
		cancel_delayed_work(&wls_dev->wls_ploss_warn_work);
		cancel_delayed_work(&wls_dev->wls_set_tec_work);
		cancel_delayed_work(&wls_dev->wls_match_q_work);
		oplus_chg_wls_cancel_bcc_update_work(wls_dev);
		oplus_chg_wls_nor_set_vindpm(wls_dev->wls_nor->nor_ic, WLS_VINDPM_DEFAULT);
		oplus_chg_wls_reset_variables(wls_dev);
		if (time_is_after_jiffies(delay_time)) {
			delay_time = delay_time - jiffies;
			msleep(jiffies_to_msecs(delay_time));
		}
		vote(wls_dev->rx_disable_votable, CONNECT_VOTER, false, 0, false);
		oplus_chg_wls_reset_variables(wls_dev);
		if (wls_status->online_keep) {
			schedule_delayed_work(&wls_dev->online_keep_remove_work, msecs_to_jiffies(2000));
		} else {
			if (wls_dev->rx_wake_lock_on) {
				chg_info("release rx_wake_lock\n");
				__pm_relax(wls_dev->rx_wake_lock);
				wls_dev->rx_wake_lock_on = false;
			} else {
				chg_err("rx_wake_lock is already relax\n");
			}
		}
		oplus_wls_adapter_curve_timer_stop(wls_dev);
	}
}

static int oplus_chg_wls_cmd_into_fastchage(struct oplus_chg_wls *wls_dev, u8 data, int wait_time_s)
{
	u8 msg[2] = {0x48, WLS_CMD_INTO_FASTCHAGE};
	u8 buf[3] = {0};
	u32 wls_rx_vout;
	int rc = 0;
	int i = 0;

	if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30 &&
	    wls_dev->wls_status.adapter_id >= WLS_ADAPTER_MODEL_4 &&
	    wls_dev->wls_status.adapter_id != WLS_ADAPTER_THIRD_PARTY) {
		while (wls_dev->wls_status.q_val_done == false) {
			msleep(100);
			i++;
			if (i > ((WLS_Q_VALUE_TIMEOUT_S + 1) * 1000) / 100) {
				chg_err("q_val_done is false, retry\n");
				break;
			}
		}
		wls_rx_vout = wls_dev->dynamic_config.svooc_vol_mv == 10000 ? 0 : 1;
		buf[0] = (wls_rx_vout << WLS_RX_VOUT_OFFSET) | (wls_dev->rx_coil & WLS_RX_COIL_MASK);
		buf[1] = wls_dev->magcvr_status == MAGCVR_STATUS_NEAR ?
			wls_dev->magcvr_vbridge_ratio : wls_dev->vbridge_ratio;
		buf[2] = wls_dev->rx_protocol_version;
		rc = oplus_chg_wls_send_raw_data(wls_dev, msg, buf, wait_time_s);
		if (rc < 0)
			chg_err("WLS_CMD_INTO_FASTCHAGE send error rc:%d\n", rc);
	} else {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_INTO_FASTCHAGE, data, wait_time_s);
	}

	return rc;
}

#define PLOSS_WARN_DELAY_MS	(1000 * 60)
static void oplus_chg_wls_ploss_warn_ta_uv(struct oplus_chg_wls *wls_dev, u8 err_type)
{
	int fcc_ma = 0;
	int vout_mv = 0;
	int rc;

	fcc_ma = get_effective_result(wls_dev->fcc_votable);
	if (err_type == WLS_TX_ERROR_PLOSS_FOD_WARN) {
		fcc_ma = fcc_ma - WLS_TX_PLOSS_CURRENT_STEP_MA < WLS_TX_PLOSS_CURRENT_LIMIT_LOW_MA ?
			WLS_TX_PLOSS_CURRENT_LIMIT_LOW_MA : fcc_ma - WLS_TX_PLOSS_CURRENT_STEP_MA;
		vote(wls_dev->fcc_votable, WLS_PLOSS_WARN_VOTER, true, fcc_ma, false);
		cancel_delayed_work(&wls_dev->wls_ploss_warn_work);
		schedule_delayed_work(&wls_dev->wls_ploss_warn_work, msecs_to_jiffies(PLOSS_WARN_DELAY_MS));
	}
	if (err_type == WLS_TX_ERROR_TA_UV) {
		fcc_ma = fcc_ma - WLS_TX_TAUV_CURRENT_STEP_MA < WLS_TX_TAUV_CURRENT_LIMIT_LOW_MA ?
			WLS_TX_TAUV_CURRENT_LIMIT_LOW_MA : fcc_ma - WLS_TX_TAUV_CURRENT_STEP_MA;
		rc = oplus_chg_wls_rx_get_vout(wls_dev->wls_rx->rx_ic, &vout_mv);
		if (rc < 0)
			chg_err("can't get vout, rc=%d\n", rc);
		else
			(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, vout_mv, 0);

		vote(wls_dev->fcc_votable, WLS_TA_UV_VOTER, true, fcc_ma, false);
	}
	chg_info("ploss_warn or ta_uv set fcc_ma:%d\n", fcc_ma);
}

static int oplus_chg_wls_set_tec_power(struct oplus_chg_wls *wls_dev, int power)
{
	u8 msg[2] = {WLS_CMD_HEAD_0X28, WLS_CMD_SET_TEC_POWER};
	u8 buf[3] = {0};
	int rc = 0;

	if (wls_dev->wls_status.rx_present == false)
		return rc;
	buf[0] = power;
	rc = oplus_chg_wls_send_raw_data(wls_dev, msg, buf, 2);
	return rc;
}

static void oplus_chg_wls_monitor_ploss(struct oplus_chg_wls *wls_dev)
{
	u8 msg[2] = {WLS_CMD_HEAD_0X19, WLS_CMD_GET_PLOSS_MARGIN};
	u8 buf[3] = {0};
	if (wls_dev->wls_status.rx_present == false ||
	    (wls_dev->wls_status.verity_done == false && wls_dev->wls_status.verity_started == true))
		return;
	oplus_chg_wls_send_raw_data(wls_dev, msg, buf, 2);
}

static void oplus_chg_wls_monitor_vbridge(struct oplus_chg_wls *wls_dev)
{
	u8 msg[2] = {WLS_CMD_HEAD_0X19, WLS_CMD_GET_VBRIDGE};
	u8 buf[3] = {0};
	if (wls_dev->wls_status.rx_present == false ||
	    (wls_dev->wls_status.verity_done == false && wls_dev->wls_status.verity_started == true))
		return;
	oplus_chg_wls_send_raw_data(wls_dev, msg, buf, 2);
}

static void oplus_chg_wls_monitor_ibridge(struct oplus_chg_wls *wls_dev)
{
	u8 msg[2] = {WLS_CMD_HEAD_0X19, WLS_CMD_GET_IBRIDGE};
	u8 buf[3] = {0};
	if (wls_dev->wls_status.rx_present == false ||
	    (wls_dev->wls_status.verity_done == false && wls_dev->wls_status.verity_started == true))
		return;
	oplus_chg_wls_send_raw_data(wls_dev, msg, buf, 2);
}

#define PENDING_ERROR_COUNT	60
#define OTHER_ERROR_COUNT	3
static void oplus_chg_wls_set_tec_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_set_tec_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int pending_err_retry_count = 0;
	int other_err_retry_count = 0;
	int rc = 0;

	if (wls_dev->rx_protocol_version < WLS_RX_PROTOCOL_VERSION_30 ||
	    wls_status->adapter_id < WLS_ADAPTER_MODEL_4 ||
	    wls_status->adapter_id == WLS_ADAPTER_THIRD_PARTY ||
	    wls_dev->wls_status.rx_present == false) {
		chg_info("rx_protocol_version:%d, adapter_id:%d\n", wls_dev->rx_protocol_version, wls_status->adapter_id);
		return;
	}

	do {
		rc = oplus_chg_wls_set_tec_power(wls_dev, wls_dev->tec_power);
		if (rc == -EAGAIN) {
			pending_err_retry_count++;
			msleep(50);
			if (pending_err_retry_count > PENDING_ERROR_COUNT)
				break;
		} else if (rc < 0) {
			other_err_retry_count++;
			if (other_err_retry_count > OTHER_ERROR_COUNT)
				break;
		}
	} while (rc < 0 && wls_status->rx_online);

	chg_info("set tec power:%d, pending_err_retry_count:%d, other_err_retry_count:%d, rc:%d\n",
		wls_dev->tec_power, pending_err_retry_count, other_err_retry_count, rc);
}

#define QUIET_MODE_MONITOR_COUNT	12
static void oplus_chg_wls_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_monitor_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (wls_dev->rx_protocol_version < WLS_RX_PROTOCOL_VERSION_30 ||
	    wls_status->adapter_id < WLS_ADAPTER_MODEL_4 ||
	    wls_status->adapter_id == WLS_ADAPTER_THIRD_PARTY ||
	    wls_dev->wls_status.rx_present == false) {
		chg_info("rx_protocol_version:%d, adapter_id:%d\n", wls_dev->rx_protocol_version, wls_status->adapter_id);
		return;
	}

	if (wls_status->verity_done == true) {
		oplus_chg_wls_monitor_ploss(wls_dev);
		/*need to sleep 1.5s,to avoid cep timeout*/
		msleep(1500);
		oplus_chg_wls_monitor_vbridge(wls_dev);
		/*need to sleep 1.5s,to avoid cep timeout*/
		msleep(1500);
		oplus_chg_wls_monitor_ibridge(wls_dev);
	} else if (wls_status->switch_quiet_mode == true) {
		if (wls_dev->monitor_count < QUIET_MODE_MONITOR_COUNT ||
		    wls_status->verity_started == true) {
			wls_dev->monitor_count++;
			chg_info("monitor_count:%d", wls_dev->monitor_count);
		} else {
			oplus_chg_wls_monitor_ploss(wls_dev);
			/*need to sleep 1.5s,to avoid cep timeout*/
			msleep(1500);
			oplus_chg_wls_monitor_vbridge(wls_dev);
			/*need to sleep 1.5s,to avoid cep timeout*/
			msleep(1500);
			oplus_chg_wls_monitor_ibridge(wls_dev);
		}
	}
	chg_info("switch_quiet_mode:%d, monitor_count:%d, verity_done:%d, verity_started:%d",
		wls_dev->wls_status.switch_quiet_mode, wls_dev->monitor_count, wls_status->verity_done, wls_dev->wls_status.verity_started);

	schedule_delayed_work(&wls_dev->wls_monitor_work, msecs_to_jiffies(OPLUS_CHG_WLS_MONITOR_DELAY));
}

static void oplus_chg_rx_mode_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, rx_mode_check_work);

	chg_info("time out exit rx_mode_bpp\n");
	vote(wls_dev->force_bpp_mode_votable, WLS_CAMERA_MODE_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, WLS_CAMERA_MODE_VOTER, false, 0, false);
}

static void oplus_chg_wls_ploss_warn_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_ploss_warn_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (!wls_status->rx_online)
		return;

	chg_info("ploss_warn: recovery charging current\n");
	vote(wls_dev->fcc_votable, WLS_PLOSS_WARN_VOTER, false, 0, false);
}

static void oplus_chg_wls_high_temp_check(enum high_temp_voter voter, struct oplus_chg_wls *wls_dev, int icl_ma)
{
	unsigned long diff_time = 0;
	unsigned long wls_start_time;
	int curr_limit, index = 0;
	struct oplus_chg_wls_fcc_step *fcc_cfg;

	if (wls_dev == NULL || wls_dev->high_temp_track.high_temp_flag || !wls_dev->wls_status.rx_online)
		return;

	switch (wls_dev->wls_status.wls_type) {
	case OPLUS_CHG_WLS_EPP:
		index = OPLUS_WLS_CHG_MODE_EPP;
		fcc_cfg = &wls_dev->dynamic_config.epp_steps;
		break;
	case OPLUS_CHG_WLS_EPP_PLUS:
		index = OPLUS_WLS_CHG_MODE_EPP_PLUS;
		fcc_cfg = &wls_dev->dynamic_config.epp_plus_steps;
		break;
	case OPLUS_CHG_WLS_BPP:
		index = OPLUS_WLS_CHG_MODE_BPP;
		fcc_cfg = &wls_dev->dynamic_config.bpp_steps;
		break;
	default:
		return;
	}

	curr_limit = (voter == HIGH_TEMP_VOTER_CURVE ? fcc_cfg->fcc_step[fcc_cfg->max_step - 1].curr_ma :
		wls_dev->dynamic_config.cool_down_step[index][1]);
	if (icl_ma > curr_limit || icl_ma <= 0)
		return;
	wls_start_time = wls_dev->high_temp_track.wls_start_time;
	wls_dev->high_temp_track.high_temp_flag = 1;
	wls_dev->high_temp_track.voter = voter;
	wls_dev->high_temp_track.track_wls_type = wls_dev->wls_status.wls_type;
	diff_time = jiffies > wls_start_time ? jiffies - wls_start_time : 0;
	wls_dev->high_temp_track.high_temp_time_diff = jiffies_to_msecs(diff_time) / 1000;
	chg_info("icl_ma:%d, wls_start_time:%lu, jiffies:%lu, diff_time:%lu\n",
		icl_ma, wls_start_time, jiffies, diff_time);
	schedule_delayed_work(&wls_dev->wls_high_temp_track_work, 0);
}

static void oplus_chg_wls_high_temp_update_track_info(struct oplus_chg_wls *wls_dev, char *buff)
{
	int i;
	int *temp;

	if (!is_err_topic_available(wls_dev)) {
		chg_err("error topic not found\n");
		return;
	}
	if (!buff) {
		chg_err("buff NULL!!\n");
		return;
	}
	if (wls_dev->high_temp_track.high_temp_flag == 0)
		return;

	temp = wls_dev->high_temp_track.temp;
	scnprintf(buff, HIGH_TEMP_INFO_LEN,
		"$$wls_h_tmp_typ@@%d$$h_tmp_t@@%lu"
		"$$wls_h_tmp_v@@%d$$h_tmp_r@@%d,%d,%d,%d,%d,%d",
		wls_dev->high_temp_track.track_wls_type, wls_dev->high_temp_track.high_temp_time_diff,
		wls_dev->high_temp_track.voter, temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]);
	chg_info("track_info:%s\n", buff);
	wls_dev->high_temp_track.high_temp_flag = 0;
	wls_dev->high_temp_track.track_wls_type = 0;
	wls_dev->high_temp_track.high_temp_time_diff = 0;
	wls_dev->high_temp_track.count = 0;
	wls_dev->high_temp_track.voter = HIGH_TEMP_VOTER_UNKNOW;
	for (i = 0; i < HIGH_TEMP_CHECK_LEN; i++)
		wls_dev->high_temp_track.temp[i] = GAUGE_INVALID_TEMP;
	return;
}

#define HIGH_TEMP_CHECK_DELAY_MS	(60 * 2 * 1000)
static void oplus_chg_wls_high_temp_track_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_high_temp_track_work);
	int temp = 0;
	int rc = 0;

	if (!wls_dev->wls_status.rx_online || !wls_dev->high_temp_track.high_temp_flag ||
	    wls_dev->high_temp_track.count >= HIGH_TEMP_CHECK_LEN)
		return;

	rc = oplus_chg_wls_get_batt_temp(wls_dev, &temp);
	if (rc < 0) {
		chg_err("can't get batt temp, rc=%d\n", rc);
	}
	wls_dev->high_temp_track.temp[wls_dev->high_temp_track.count] = temp;
	wls_dev->high_temp_track.count++;
	chg_info("count:%d, temp:%d\n", wls_dev->high_temp_track.count, temp);

	if (wls_dev->high_temp_track.count < HIGH_TEMP_CHECK_LEN)
		schedule_delayed_work(&wls_dev->wls_high_temp_track_work, msecs_to_jiffies(HIGH_TEMP_CHECK_DELAY_MS));
}

static int oplus_chg_wls_nor_skin_check(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_skin_step *skin_step;
	int *skin_step_count;
	int skin_temp;
	/*int curr_ma;*/
	int rc;

	rc = oplus_chg_wls_get_skin_temp(wls_dev, &skin_temp);
	if (rc < 0) {
		chg_err("can't get skin temp, rc=%d\n", rc);
		return rc;
	}

	switch (wls_status->wls_type) {
	case OPLUS_CHG_WLS_BPP:
		skin_step = &wls_dev->bpp_skin_step;
		skin_step_count = &wls_status->bpp_skin_step;
		break;
	case OPLUS_CHG_WLS_EPP:
		skin_step = &wls_dev->epp_skin_step;
		skin_step_count = &wls_status->epp_skin_step;
		break;
	case OPLUS_CHG_WLS_EPP_PLUS:
		skin_step = &wls_dev->epp_plus_skin_step;
		skin_step_count = &wls_status->epp_plus_skin_step;
		break;
	case OPLUS_CHG_WLS_VOOC:
	case OPLUS_CHG_WLS_SVOOC:
	case OPLUS_CHG_WLS_PD_65W:
		skin_step = &wls_dev->epp_plus_skin_step;
		skin_step_count = &wls_status->epp_plus_skin_step;
		break;
	default:
		chg_err("Unsupported charging mode(=%d)\n", wls_status->wls_type);
		return -EINVAL;
	}

	if (skin_step->max_step == 0)
		return 0;

	if ((skin_temp < skin_step->skin_step[*skin_step_count].low_threshold) &&
	    (*skin_step_count > 0)) {
		chg_info("skin_temp=%d, switch to the previous gear\n", skin_temp);
		*skin_step_count = *skin_step_count - 1;
	} else if ((skin_temp > skin_step->skin_step[*skin_step_count].high_threshold) &&
		   (*skin_step_count < (skin_step->max_step - 1))) {
		chg_info("skin_temp=%d, switch to the next gear\n", skin_temp);
		*skin_step_count = *skin_step_count + 1;
	}
	/*curr_ma = skin_step->skin_step[*skin_step_count].curr_ma;*/
	/*vote(wls_dev->nor_icl_votable, SKIN_VOTER, true, curr_ma, true);*/

	return 0;
}

static void oplus_chg_wls_fast_fcc_param_init(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_fcc_step *fcc_chg = &wls_dev->wls_fcc_step;
	int i;

	for(i = 0; i < fcc_chg->max_step; i++) {
		if (fcc_chg->fcc_step[i].low_threshold > 0)
			fcc_chg->allow_fallback[i] = true;
		else
			fcc_chg->allow_fallback[i] = false;
	}
}

static bool oplus_chg_wls_fast_step_is_timeout(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_fcc_step *fcc_chg = &wls_dev->wls_fcc_step;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int running_time;

	if (wls_status->fastchg_level >= 0 && wls_status->fastchg_level < fcc_chg->max_step) {
		running_time = fcc_chg->fcc_step[wls_status->fastchg_level].need_wait;
		if (running_time > 1 && time_after(jiffies, fcc_chg->fcc_wait_timeout + HZ * running_time)) {
			chg_info("step=%d running time > %d seconds\n", wls_status->fastchg_level, running_time);
			return true;
		}
	}
	return false;
}

static void oplus_chg_wls_fast_switch_next_step(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_fcc_step *fcc_chg = &wls_dev->wls_fcc_step;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	u32 batt_vol_max = fcc_chg->fcc_step[wls_status->fastchg_level].vol_max_mv;
	int batt_vol_mv;
	int batt_temp;
	int rc;

	rc = oplus_chg_wls_get_vbat(wls_dev, &batt_vol_mv);
	if (rc < 0) {
		chg_err("can't get batt vol, rc=%d\n", rc);
		return;
	}
	rc = oplus_chg_wls_get_batt_temp(wls_dev, &batt_temp);
	if (rc < 0) {
		chg_err("can't get batt temp, rc=%d\n", rc);
		return;
	}

	if (fcc_chg->fcc_step[wls_status->fastchg_level].need_wait == 0) {
		if (batt_vol_mv >= batt_vol_max) {
			/* Must delay 1 sec and wait for the batt voltage to drop */
			fcc_chg->fcc_wait_timeout = jiffies + HZ * 5;
		} else {
			fcc_chg->fcc_wait_timeout = jiffies;
		}
	} else if (fcc_chg->fcc_step[wls_status->fastchg_level].need_wait > 1) {
		/*init fcc_wait_timeout to check step timeout*/
		fcc_chg->fcc_wait_timeout = jiffies;
	} else {
		/* Delay 1 minute and wait for the temperature to drop */
		fcc_chg->fcc_wait_timeout = jiffies + HZ * 60;
	}

	wls_status->fastchg_level++;
	chg_info("switch to next level=%d\n", wls_status->fastchg_level);
	if (wls_status->fastchg_level >= fcc_chg->max_step) {
		if (batt_vol_mv >= batt_vol_max) {
			chg_info("run normal charge ffc\n");
			wls_status->ffc_check = true;
			/*oplus_chg_wls_track_set_prop(
				wls_dev, OPLUS_CHG_PROP_STATUS,
				TRACK_WLS_FASTCHG_FULL);*/
		}
	} else {
		wls_status->wait_cep_stable = true;
		vote(wls_dev->fcc_votable, FCC_VOTER, true,
		     fcc_chg->fcc_step[wls_status->fastchg_level].curr_ma, false);
	}
	wls_status->fastchg_level_init_temp = batt_temp;
	if (batt_vol_mv >= batt_vol_max) {
		if (wls_status->fastchg_level < fcc_chg->max_step)
			fcc_chg->allow_fallback[wls_status->fastchg_level] = false;
	}
}

static void oplus_chg_wls_fast_switch_prev_step(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_fcc_step *fcc_chg = &wls_dev->wls_fcc_step;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	wls_status->fastchg_level--;
	chg_info("switch to prev level=%d\n", wls_status->fastchg_level);
	vote(wls_dev->fcc_votable, FCC_VOTER, true,
	     fcc_chg->fcc_step[wls_status->fastchg_level].curr_ma, false);
	wls_status->fastchg_level_init_temp = 0;
	fcc_chg->fcc_wait_timeout = jiffies;
}

static int oplus_chg_wls_fast_temp_check(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_fcc_step *fcc_chg = &wls_dev->wls_fcc_step;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	enum oplus_chg_temp_region temp_region;
	int batt_temp;
	int batt_vol_mv;
	int def_curr_ma;
	int fcc_curr_ma;
	int next_curr_ma;
	/*
	 * We want the temperature to drop when switching to a lower current range.
	 * If the temperature rises by 2 degrees before the next gear begins to
	 * detect temperature, then you should immediately switch to a lower gear.
	 */
	int temp_diff;
	int rc;

	temp_region = oplus_chg_wls_get_temp_region(wls_dev);

	if ((temp_region < BATT_TEMP_COOL) || (temp_region > BATT_TEMP_LITTLE_WARM)) {
		chg_info("Abnormal battery temperature, exit fast charge\n");
		vote(wls_dev->fastchg_disable_votable, FCC_VOTER, true, 1, false);
		return -EPERM;
	}
	rc = oplus_chg_wls_get_vbat(wls_dev, &batt_vol_mv);
	if (rc < 0) {
		chg_err("can't get batt vol, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_chg_wls_get_batt_temp(wls_dev, &batt_temp);
	if (rc < 0) {
		chg_err("can't get batt temp, rc=%d\n", rc);
		return rc;
	}
	def_curr_ma = get_client_vote(wls_dev->fcc_votable, JEITA_VOTER);
	if (def_curr_ma <= 0)
		def_curr_ma = get_client_vote(wls_dev->fcc_votable, MAX_VOTER);
	else
		def_curr_ma = min(get_client_vote(wls_dev->fcc_votable, MAX_VOTER), def_curr_ma);
	chg_info("wkcs: def_curr_ma=%d, max_step=%d\n", def_curr_ma, fcc_chg->max_step);
	if (wls_status->fastchg_level + 1 < fcc_chg->max_step)
		next_curr_ma = fcc_chg->fcc_step[wls_status->fastchg_level + 1].curr_ma;
	else
		next_curr_ma = 0;
	if (wls_status->fastchg_level_init_temp != 0)
		temp_diff = batt_temp - wls_status->fastchg_level_init_temp;
	else
		temp_diff = 0;

	if (wls_status->fastchg_level >= 0 && wls_status->fastchg_level < fcc_chg->max_step)
		fcc_curr_ma = fcc_chg->fcc_step[wls_status->fastchg_level].curr_ma;
	else
		fcc_curr_ma = 0;
	chg_info("battery temp = %d, vol = %d, level = %d, temp_diff = %d, fcc_curr_ma=%d\n",
		batt_temp, batt_vol_mv, wls_status->fastchg_level, temp_diff, fcc_curr_ma);

	if (wls_status->fastchg_level == 0) {
		if ((batt_temp > fcc_chg->fcc_step[wls_status->fastchg_level].high_threshold) ||
		    (batt_vol_mv >= fcc_chg->fcc_step[wls_status->fastchg_level].vol_max_mv) ||
		    oplus_chg_wls_fast_step_is_timeout(wls_dev)) {
			oplus_chg_wls_fast_switch_next_step(wls_dev);
			return 0;
		}
		vote(wls_dev->fcc_votable, FCC_VOTER, true,
			fcc_chg->fcc_step[wls_status->fastchg_level].curr_ma, false);
	} else if (wls_status->fastchg_level >= fcc_chg->max_step) {  /*switch to pmic*/
		vote(wls_dev->fastchg_disable_votable, FCC_VOTER, true, 1, false);
		return -EPERM;
	} else {
		if (batt_vol_mv >= dynamic_cfg->batt_vol_max_mv || oplus_chg_wls_fast_step_is_timeout(wls_dev)) {
			chg_info("batt voltage too high or timeout, switch next step\n");
			oplus_chg_wls_fast_switch_next_step(wls_dev);
			return 0;
		}
		if ((batt_temp < fcc_chg->fcc_step[wls_status->fastchg_level].low_threshold) &&
		    fcc_chg->allow_fallback[wls_status->fastchg_level] &&
		    (def_curr_ma > fcc_chg->fcc_step[wls_status->fastchg_level].curr_ma)) {
			chg_info("target current too low, switch prev step\n");
			oplus_chg_wls_fast_switch_prev_step(wls_dev);
			return 0;
		}
		chg_info("jiffies=%lu, timeout=%lu, high_threshold=%d, batt_vol_max=%d\n",
			 jiffies, fcc_chg->fcc_wait_timeout,
			 fcc_chg->fcc_step[wls_status->fastchg_level].high_threshold,
			 fcc_chg->fcc_step[wls_status->fastchg_level].vol_max_mv);
		if (time_after(jiffies, fcc_chg->fcc_wait_timeout) || (temp_diff > 200) ||
		    (batt_vol_mv > dynamic_cfg->fastch_wait_thr_mv) ||
		    (get_effective_result(wls_dev->fcc_votable) <= next_curr_ma)) {
			if ((batt_temp > fcc_chg->fcc_step[wls_status->fastchg_level].high_threshold) ||
			    (batt_vol_mv >= fcc_chg->fcc_step[wls_status->fastchg_level].vol_max_mv) ||
			    oplus_chg_wls_fast_step_is_timeout(wls_dev)) {
				oplus_chg_wls_fast_switch_next_step(wls_dev);
			}
		}
	}

	return 0;
}

#define CEP_ERR_MAX 12/*(3*2s=12*500ms)*/
#define CEP_OK_MAX 10
#define CEP_WAIT_MAX 40
#define CEP_OK_TIMEOUT_MAX 30
static void oplus_chg_wls_fast_cep_check(struct oplus_chg_wls *wls_dev, int cep)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int curr_ma, cep_curr_ma;
	static int wait_cep_count;
	static int cep_err_count;
	static int cep_ok_count;

	if (!wls_status->wait_cep_stable) {
		/* Insufficient energy only when CEP is positive */
		if (cep < 3) {
			cep_ok_count++;
			cep_err_count = 0;
			if ((cep_ok_count >= CEP_OK_MAX) &&
			    time_after(jiffies, wls_status->cep_ok_wait_timeout) &&
			    is_client_vote_enabled(wls_dev->fcc_votable, CEP_VOTER)) {
				chg_info("skewing: recovery charging current\n");
				cep_ok_count = 0;
				wls_status->wait_cep_stable = true;
				wls_status->cep_ok_wait_timeout = jiffies + CEP_OK_TIMEOUT_MAX * HZ;
				wait_cep_count = 0;
				vote(wls_dev->fcc_votable, CEP_VOTER, false, 0, false);
			}
		} else {
			cep_ok_count = 0;
			cep_err_count++;
			if (cep_err_count >= CEP_ERR_MAX) {
				chg_info("skewing: reduce charging current\n");
				cep_err_count = 0;
				wls_status->wait_cep_stable = true;
				wait_cep_count = 0;
				if (is_client_vote_enabled(wls_dev->fcc_votable, CEP_VOTER))
					cep_curr_ma = get_client_vote(wls_dev->fcc_votable, CEP_VOTER);
				else
					cep_curr_ma = 0;
				if ((cep_curr_ma > 0) && (cep_curr_ma <= WLS_FASTCHG_CURR_MIN_MA)) {
					chg_info("skewing: Energy is too low, exit fast charge\n");
					vote(wls_dev->fastchg_disable_votable, CEP_VOTER, true, 1, false);
					wls_status->cep_ok_wait_timeout = jiffies + CEP_OK_TIMEOUT_MAX * HZ;
					wls_status->fast_cep_check = -1;
					return;
				} else {
					curr_ma = wls_status->iout_ma;
					/* Target current is adjusted in 50ma steps*/
					curr_ma = curr_ma - (curr_ma % WLS_FASTCHG_CURR_ERR_MA) - WLS_FASTCHG_CURR_ERR_MA;
					if (curr_ma < WLS_FASTCHG_CURR_MIN_MA)
						curr_ma = WLS_FASTCHG_CURR_MIN_MA;
					vote(wls_dev->fcc_votable, CEP_VOTER, true, curr_ma, false);
				}
				wls_status->cep_ok_wait_timeout = jiffies + CEP_OK_TIMEOUT_MAX * HZ;
			}
		}
	} else {
		if (wait_cep_count < CEP_WAIT_MAX) {
			wait_cep_count++;
		} else {
			wls_status->wait_cep_stable = false;
			wait_cep_count = 0;
			cep_err_count = 0;
			cep_ok_count = 0;
		}
	}

	return;
}

static int oplus_chg_wls_fast_ibat_check(struct oplus_chg_wls *wls_dev)
{
	int ibat_ma = 0;
	int rc;

	rc = oplus_chg_wls_get_ibat(wls_dev, &ibat_ma);
	if (rc < 0) {
		chg_err("can't get ibat, rc=%d\n", rc);
		return rc;
	}

	if (ibat_ma >= WLS_FASTCHG_CURR_DISCHG_MAX_MA) {
		chg_err("discharge current is too large, exit fast charge\n");
		vote(wls_dev->fastchg_disable_votable, BATT_CURR_VOTER, true, 1, false);
		return -1;
	}

	return 0;
}

#define IOUT_SMALL_COUNT 30
static int oplus_chg_wls_fast_iout_check(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int iout_ma = 0;

	iout_ma = wls_status->iout_ma;

	if (iout_ma <= WLS_FASTCHG_IOUT_CURR_MIN_MA) {
		wls_status->iout_ma_conunt++;
		if (wls_status->iout_ma_conunt >= IOUT_SMALL_COUNT) {
			chg_err("iout current is too small, exit fast charge\n");
			vote(wls_dev->fastchg_disable_votable, IOUT_CURR_VOTER, true, 1, false);
			return -1;
		}
	} else {
		wls_status->iout_ma_conunt = 0;
	}
	return 0;
}

static void oplus_chg_wls_fast_skin_temp_check(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int skin_temp;
	/*int curr_ma;*/
	int rc;

	if (wls_dev->factory_mode) {
		vote(wls_dev->fcc_votable, SKIN_VOTER, false, 0, false);
		return;
	}

	rc = oplus_chg_wls_get_skin_temp(wls_dev, &skin_temp);
	if (rc < 0) {
		chg_err("can't get skin temp, rc=%d\n", rc);
		skin_temp = 250;
	}

	if (wls_status->led_on) {
		vote(wls_dev->fcc_votable, SKIN_VOTER, false, 0, false);
	} else {
		/*if (wls_dev->wls_fast_chg_led_off_strategy.initialized) {
			curr_ma = oplus_chg_strategy_get_data(&wls_dev->wls_fast_chg_led_off_strategy,
				&wls_dev->wls_fast_chg_led_off_strategy.temp_region, skin_temp);
			chg_info("led is off, curr = %d\n", curr_ma);
			vote(wls_dev->fcc_votable, SKIN_VOTER, true, curr_ma, false);
		}*/
	}
}

static int oplus_chg_wls_fast_cool_down_check(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int cool_down = wls_status->cool_down;

	if (!wls_dev->factory_mode) {
		if (cool_down > 0 && cool_down <= dynamic_cfg->cool_down_12v_thr) {
			chg_err("cool_down level < %d, exit fast charge\n", dynamic_cfg->cool_down_12v_thr);
			vote(wls_dev->fastchg_disable_votable, COOL_DOWN_VOTER, true, 1, false);
			return -1;
		}
	}

	return 0;
}

struct wls_fan_pwm_pulse fan_pwm_pulse[WLS_ADAPTER_MODEL_MAX] = {
	{0xff, 80, 52, 80, 80, 80},
	{0x01, 93, 60, 93, 93, 93},
	{0x02, 80, 52, 80, 80, 80},
	{0x03, 80, 52, 80, 80, 80},
	{0x04, 80, 52, 91, 91, 91},
	{0x05, 80, 52, 91, 91, 91},
	{0x06, 80, 52, 91, 91, 91},
	{0x07, 80, 52, 91, 91, 91},
	{0x08, 70, 40, 70, 70, 70},
	{0x09, 70, 40, 70, 70, 70},
	{0x0a, 70, 40, 70, 70, 70},
	{0x0b, 70, 40, 70, 70, 70},
	{0x0c, 70, 40, 70, 70, 70},
	{0x0d, 70, 40, 70, 70, 70},
	{0x0e, 70, 40, 70, 70, 70},
	{0x0f, 70, 40, 70, 70, 70},
};

static void oplus_chg_wls_set_quiet_mode(struct oplus_chg_wls *wls_dev, bool quiet_mode)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	u32 fan_pwm_pulse_fastchg = 0;
	u32 fan_pwm_pulse_silent = 0;
	u32 fan_pwm_pulse_fastchg_exit = 0;
	u32 fan_pwm_pulse_ui_100 = 0;
	u32 wls_model_id = 0;
	int i = 0;

	if (wls_status->adapter_id >= WLS_ADAPTER_MODEL_1 &&
	    wls_status->adapter_id <= WLS_ADAPTER_MODEL_15)
		wls_model_id = wls_status->adapter_id;
	else
		wls_model_id = 0xff;


	for (i = 0; i < ARRAY_SIZE(fan_pwm_pulse); i++) {
		if (fan_pwm_pulse[i].wls_model_id == wls_model_id) {
			fan_pwm_pulse_fastchg = fan_pwm_pulse[i].fan_pwm_pulse_fastchg;
			fan_pwm_pulse_silent = fan_pwm_pulse[i].fan_pwm_pulse_silent;
			fan_pwm_pulse_fastchg_exit = fan_pwm_pulse[i].fan_pwm_pulse_fastchg_exit;
			fan_pwm_pulse_ui_100 = fan_pwm_pulse[i].fan_pwm_pulse_ui_100;

			chg_info("%d, %d, %d, %d, %d\n", wls_model_id, fan_pwm_pulse_fastchg, fan_pwm_pulse_silent,
				fan_pwm_pulse_fastchg_exit, fan_pwm_pulse_ui_100);
			break;
		}
	}

	if (wls_status->adapter_id == WLS_ADAPTER_MODEL_0)
		(void)oplus_chg_wls_send_msg(wls_dev, quiet_mode ?
			WLS_CMD_SET_QUIET_MODE : WLS_CMD_SET_NORMAL_MODE, 0xff, 2);
	else if (wls_status->fastchg_ui_100_fan_speed_set == true)
		(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_SET_FAN_SPEED,
			quiet_mode ? fan_pwm_pulse_silent : fan_pwm_pulse_ui_100, 2);
	else if (wls_status->fastchg_exit_fan_speed_set == true)
		(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_SET_FAN_SPEED,
			quiet_mode ? fan_pwm_pulse_silent : fan_pwm_pulse_fastchg_exit, 2);
	else
		(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_SET_FAN_SPEED,
			quiet_mode ? fan_pwm_pulse_silent : fan_pwm_pulse_fastchg, 2);

	(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_SET_LED_BRIGHTNESS,
		quiet_mode ? QUIET_MODE_LED_BRIGHTNESS : 100, 2);
}

static void oplus_chg_wls_check_quiet_mode(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (wls_status->charge_type != WLS_CHARGE_TYPE_FAST)
		return;

	if (wls_dev->batt_info.ui_soc == 100)
		wls_status->fastchg_ui_100_fan_speed_set = true;

	if (wls_status->switch_quiet_mode) {
		if ((!wls_status->quiet_mode || wls_status->quiet_mode_init == false)
				&& !wls_status->chg_done_quiet_mode)
			(void)oplus_chg_wls_set_quiet_mode(wls_dev, wls_status->switch_quiet_mode);
	} else {
		if (wls_status->quiet_mode || wls_status->quiet_mode_init == false)
			(void)oplus_chg_wls_set_quiet_mode(wls_dev, wls_status->switch_quiet_mode);
	}
}

static void oplus_chg_wls_fastchg_to_bpp_keep_state_remove_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls,
					wls_fastchg_to_bpp_keep_state_remove_work);

	wls_dev->wls_status.fastchg_to_bpp_state_keep = false;
}

static void oplus_chg_wls_fastchg_enforce_to_bpp(struct oplus_chg_wls *wls_dev, enum wls_err_reason reason_type)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (wls_dev->rx_protocol_version < WLS_RX_PROTOCOL_VERSION_30)
		return;

	wls_status->online_keep = true;
	wls_status->fastchg_timeout_to_bpp = true;
	wls_status->fastchg_to_bpp_state_keep = true;

	oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_RX, reason_type);
	oplus_chg_wls_set_status_keep(wls_dev->wls_topic, WLS_SK_BY_FORCE_BPP);
	if (oplus_chg_wls_rx_set_silent(wls_dev->wls_rx->rx_ic) == -ENOTSUPP) {
		vote(wls_dev->rx_disable_votable, WLS_TIMEOUT_VOTER, true, 1, false);
		msleep(200);
		vote(wls_dev->rx_disable_votable, WLS_TIMEOUT_VOTER, false, 1, false);
	}

	schedule_delayed_work(&wls_dev->wls_fastchg_to_bpp_keep_state_remove_work, msecs_to_jiffies(8000));
}

static void oplus_chg_wls_check_term_charge(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int curr_ma;
	int skin_temp;
	int rc;

	if (!wls_status->cep_timeout_adjusted &&
	    !wls_status->fastchg_started &&
	    (wls_status->charge_type == WLS_CHARGE_TYPE_FAST) &&
	    ((wls_status->current_rx_state == OPLUS_CHG_WLS_RX_STATE_EPP) ||
	     (wls_status->current_rx_state == OPLUS_CHG_WLS_RX_STATE_EPP_PLUS) ||
	     (wls_status->current_rx_state == OPLUS_CHG_WLS_RX_STATE_FFC) ||
	     (wls_status->current_rx_state == OPLUS_CHG_WLS_RX_STATE_DONE))) {
		(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_SET_CEP_TIMEOUT, 0xff, 2);
	}
	curr_ma = get_client_vote(wls_dev->nor_icl_votable, CHG_DONE_VOTER);
	rc = oplus_chg_wls_get_skin_temp(wls_dev, &skin_temp);
	if (rc < 0) {
		chg_err("can't get skin temp, rc=%d\n", rc);
		skin_temp = 250;
	}

	if (wls_status->chg_done) {
		if (curr_ma <= 0) {
			chg_info("chg done, set icl to %dma\n", WLS_CURR_STOP_CHG_MA);
			vote(wls_dev->nor_icl_votable, CHG_DONE_VOTER, true, WLS_CURR_STOP_CHG_MA, true);
		}
		if (skin_temp < CHARGE_FULL_FAN_THREOD_LO)
			wls_status->chg_done_quiet_mode = true;
		else if (skin_temp > CHARGE_FULL_FAN_THREOD_HI)
			wls_status->chg_done_quiet_mode = false;
	} else {
		if (curr_ma > 0) {
			vote(wls_dev->nor_icl_votable, CHG_DONE_VOTER, false, 0, true);
		}
		wls_status->chg_done_quiet_mode = false;
	}
}

static int oplus_chg_wls_fastchg_restart_check(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	enum oplus_chg_temp_region temp_region;
	int batt_temp;
	int real_soc = 100;
	int ibat_ma = 0;
	int vbat_mv = 0;
	int rc;
	static int cnt = 0;

	if (wls_status->switch_quiet_mode || !wls_dev->batt_charge_enable)
		return -EPERM;

	rc = oplus_chg_wls_get_real_soc(wls_dev, &real_soc);
	if ((rc < 0) || (real_soc >= dynamic_cfg->fastchg_max_soc)) {
		chg_err("can't get real soc or soc is too high, rc=%d\n", rc);
		return -EPERM;
	}

	rc = oplus_chg_wls_get_batt_temp(wls_dev, &batt_temp);
	if (rc < 0 || batt_temp >= dynamic_cfg->fastchg_max_temp) {
		chg_err("can't get batt temp, or batt_temp is too high rc=%d\n", rc);
		return -EPERM;
	}

	temp_region = oplus_chg_wls_get_temp_region(wls_dev);
	if ((temp_region < BATT_TEMP_COOL) || (temp_region > BATT_TEMP_LITTLE_WARM)) {
		chg_info("Abnormal battery temperature, can not restart fast charge\n");
		return -EPERM;
	}

	rc = oplus_chg_wls_get_vbat(wls_dev, &vbat_mv);
	if ((rc < 0) || vbat_mv >= dynamic_cfg->fastch_max_vbat_mv[temp_region]) {
		chg_err("can't get vbat, or vbat is too high rc=%d\n", rc);
		cnt = 0;
		return -EPERM;
	} else if (vbat_mv < dynamic_cfg->fastch_max_vbat_mv[temp_region] && cnt < 1) {
		cnt++;
		chg_err("vbat is lower than fastch_max_vbat_mv, vbat_mv=%d, cnt=%d,"
			"fastch_max_vbat_mv=%d\n", vbat_mv, cnt,
			dynamic_cfg->fastch_max_vbat_mv[temp_region]);
		return -EPERM;
	} else {
		cnt = 0;
	}

	rc = oplus_chg_wls_get_ibat(wls_dev, &ibat_ma);
	if (rc < 0) {
		chg_err("can't get ibat, rc=%d\n", rc);
		return rc;
	}

	if (!wls_status->fastchg_disable)
		return 0;

	if (is_client_vote_enabled(wls_dev->fastchg_disable_votable, RX_FAST_ERR_VOTER) &&
		(wls_status->fastchg_err_count < 2) &&
		time_is_before_jiffies(wls_status->fastchg_err_timer))
		vote(wls_dev->fastchg_disable_votable, RX_FAST_ERR_VOTER, false, 0, false);

	if (is_client_vote_enabled(wls_dev->fastchg_disable_votable, FCC_VOTER)) {
		vote(wls_dev->fastchg_disable_votable, FCC_VOTER, false, 0, false);
	}

	if (is_client_vote_enabled(wls_dev->fastchg_disable_votable, BATT_CURR_VOTER) && (ibat_ma < 0))
		vote(wls_dev->fastchg_disable_votable, BATT_CURR_VOTER, false, 0, false);


	if (is_client_vote_enabled(wls_dev->fastchg_disable_votable, STARTUP_CEP_VOTER) &&
	    (wls_status->fastchg_retry_count < 10) &&
	    time_is_before_jiffies(wls_status->fastchg_retry_timer))
		vote(wls_dev->fastchg_disable_votable, STARTUP_CEP_VOTER, false, 0, false);

	if (is_client_vote_enabled(wls_dev->fastchg_disable_votable, QUIET_VOTER))
                vote(wls_dev->fastchg_disable_votable, QUIET_VOTER, false, 0, false);

	if (is_client_vote_enabled(wls_dev->fastchg_disable_votable, COOL_DOWN_VOTER)) {
		if ((dynamic_cfg->cool_down_12v_thr > 0 && wls_status->cool_down > dynamic_cfg->cool_down_12v_thr) ||
		    wls_status->cool_down == 0)
			vote(wls_dev->fastchg_disable_votable, COOL_DOWN_VOTER, false, 0, false);
	}

	return 0;
}

static void oplus_chg_wls_non_ffc_current_batt_cl_init(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;

	if (wls_status->fcc_gear == FCC_GEAR_HIGH)
		dynamic_cfg->norchg_steps.batt_cl = OPLUS_WLS_CHG_BATT_CL_HIGH;
	else
		dynamic_cfg->norchg_steps.batt_cl = OPLUS_WLS_CHG_BATT_CL_LOW;

	dynamic_cfg->norchg_steps.pre_temp_region = -1;
	chg_info("oplus_chg_wls_non_ffc_current_batt_cl_init, batt_cl: %d\n", dynamic_cfg->norchg_steps.batt_cl);
	return;
}

#define FALLBACK_COUNT_MAX	3
static int oplus_chg_wls_set_non_ffc_current_cc_with_temp(struct oplus_chg_wls *wls_dev, int chg_mode, int batt_vol_mv)
{
	int norchg_max_step = 0;
	static u32 fallback_count = 0;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int *level = &dynamic_cfg->norchg_steps.norchg_vol_level;
	struct oplus_chg_wls_norchg_step norchg_step_cfg;
	int j = 0;
	enum oplus_temp_region temp_region;

	temp_region = wls_dev->wls_status.comm_temp_region;

	if ((temp_region == TEMP_REGION_COLD) || (temp_region == TEMP_REGION_HOT) || (temp_region == TEMP_REGION_MAX)) {
		chg_err("temp_region: %d\n", temp_region);
		return 0;
	}

	norchg_step_cfg = dynamic_cfg->norchg_steps.norchg_step[chg_mode][dynamic_cfg->norchg_steps.batt_cl][temp_region];
	norchg_max_step = norchg_step_cfg.max_step;

	if (dynamic_cfg->norchg_steps.pre_temp_region != temp_region) {
		chg_err("pre_temp_region: %d, temp_region: %d\n", dynamic_cfg->norchg_steps.pre_temp_region, temp_region);
		*level = 0;
	}

	for (j = 0; j < norchg_max_step; j++) {
		if (batt_vol_mv < norchg_step_cfg.norchg_step[j].vol_max_mv) {
			if (*level < j)
				*level = j;
			break;
		}
	}

	dynamic_cfg->norchg_steps.pre_temp_region = temp_region;
	if (norchg_step_cfg.norchg_step[*level].curr_ma > 0) {
		if ((*level == norchg_max_step - 1) && (*level > 0)) {
			if (batt_vol_mv <= norchg_step_cfg.norchg_step[*level - 1].vol_max_mv - WLS_NON_FFC_FLOAT_VOL)
				fallback_count++;
			else
				fallback_count = 0;

			if (fallback_count >= FALLBACK_COUNT_MAX) {
				*level = norchg_max_step - 2;
				vote(wls_dev->nor_icl_votable, NON_FFC_VOTER, true,
					norchg_step_cfg.norchg_step[*level].curr_ma, true);
				chg_info("fallback: non_ffc_icl=%d, j=%d, level=%d\n",
					norchg_step_cfg.norchg_step[*level].curr_ma,
					j, *level);
			} else {
				vote(wls_dev->nor_icl_votable, NON_FFC_VOTER, true,
					norchg_step_cfg.norchg_step[*level].curr_ma, true);
				chg_info("non_ffc_icl=%d, j=%d, level=%d\n",
					norchg_step_cfg.norchg_step[*level].curr_ma,
					j, *level);
			}
		} else {
			fallback_count = 0;
			vote(wls_dev->nor_icl_votable, NON_FFC_VOTER, true,
				norchg_step_cfg.norchg_step[*level].curr_ma, true);
			chg_info("non_ffc_icl=%d, j=%d, level=%d\n",
				norchg_step_cfg.norchg_step[*level].curr_ma,
				j, *level);
		}
	}
	return 0;
}


static int oplus_chg_wls_set_non_ffc_current_cc(struct oplus_chg_wls *wls_dev, int chg_mode, int batt_vol_mv)
{
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	static u32 fallback_count = 0;
	int non_ffc_max_step = 0;
	int j = 0;

	non_ffc_max_step = wls_dev->non_ffc_step[chg_mode].max_step;

	for (j = 0; j < non_ffc_max_step; j++) {
		if (batt_vol_mv < dynamic_cfg->non_ffc_step[chg_mode][j].vol_max_mv) {
			if (wls_status->non_fcc_level < j)
				wls_status->non_fcc_level = j;
			break;
		}
	}
	if (dynamic_cfg->non_ffc_step[chg_mode][wls_status->non_fcc_level].curr_ma > 0) {
		if ((wls_status->non_fcc_level == non_ffc_max_step - 1) && (wls_status->non_fcc_level > 0)) {
			if (batt_vol_mv <= dynamic_cfg->non_ffc_step[chg_mode][wls_status->non_fcc_level - 1].vol_max_mv - 50)
				fallback_count++;
			else
				fallback_count = 0;
			if (fallback_count >= FALLBACK_COUNT_MAX) {
				wls_status->non_fcc_level = non_ffc_max_step - 2;
				vote(wls_dev->nor_icl_votable, NON_FFC_VOTER, true,
					dynamic_cfg->non_ffc_step[chg_mode][wls_status->non_fcc_level].curr_ma, true);
				chg_info("fallback: non_ffc_icl=%d, j=%d, level=%d\n",
					dynamic_cfg->non_ffc_step[chg_mode][wls_status->non_fcc_level].curr_ma,
					j, wls_status->non_fcc_level);
			} else {
				vote(wls_dev->nor_icl_votable, NON_FFC_VOTER, true,
					dynamic_cfg->non_ffc_step[chg_mode][wls_status->non_fcc_level].curr_ma, true);
				chg_info("non_ffc_icl=%d, j=%d, level=%d\n",
					dynamic_cfg->non_ffc_step[chg_mode][wls_status->non_fcc_level].curr_ma,
					j, wls_status->non_fcc_level);
			}
		} else {
			fallback_count = 0;
			vote(wls_dev->nor_icl_votable, NON_FFC_VOTER, true,
				dynamic_cfg->non_ffc_step[chg_mode][wls_status->non_fcc_level].curr_ma, true);
			chg_info("non_ffc_icl=%d, j=%d, level=%d\n",
				dynamic_cfg->non_ffc_step[chg_mode][wls_status->non_fcc_level].curr_ma,
				j, wls_status->non_fcc_level);
		}
	}

	return 0;
}

static int oplus_chg_wls_set_non_ffc_current_cv(struct oplus_chg_wls *wls_dev, int chg_mode, int batt_vol_mv)
{
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int cv_icl = 0;
	int j = 0;
	int batt_temp = 0;
	int rc = 0;
	struct oplus_chg_wls_cv_step (*cv_step)[OPLUS_WLS_CHG_MODE_MAX];
	struct oplus_chg_wls_range_data (*cv_step_range_data)[OPLUS_WLS_CHG_MODE_MAX][WLS_MAX_STEP_CHG_ENTRIES];

	if ((wls_dev->support_non_fcc_with_temp == true) && (dynamic_cfg->norchg_steps.batt_cl == OPLUS_WLS_CHG_BATT_CL_HIGH)) {
		cv_step = &wls_dev->cv_step_high_batt;
		cv_step_range_data = &dynamic_cfg->cv_step_high_batt;
	} else {
		cv_step = &wls_dev->cv_step;
		cv_step_range_data = &dynamic_cfg->cv_step;
	}

	rc = oplus_chg_wls_get_batt_temp(wls_dev, &batt_temp);
	if (rc < 0) {
		chg_err("can't get batt temp, rc=%d\n", rc);
		return -EPERM;
	}

	for (j = 0; j < (*cv_step)[chg_mode].max_step; j++) {
		if (batt_temp <= (*cv_step_range_data)[chg_mode][j].high_threshold) {
			cv_icl = (*cv_step_range_data)[chg_mode][j].curr_ma;
			chg_info("cv:j=%d, cv_icl=%d, batt_vol_mv:%d, cv_step_vol_max_mv:%d\n", j, cv_icl, batt_vol_mv,
				(*cv_step_range_data)[chg_mode][j].vol_max_mv);
			break;
		}
	}

	if (cv_icl <= 0) {
		chg_err("cv_icl: %d\n", cv_icl);
		return 0;
	}

	if (j > (*cv_step)[chg_mode].max_step - 1)
		j = (*cv_step)[chg_mode].max_step - 1;

	if (batt_vol_mv >= (*cv_step_range_data)[chg_mode][j].vol_max_mv && cv_icl > 0)
		vote(wls_dev->nor_icl_votable, CV_VOTER, true, cv_icl, true);

	if (batt_vol_mv <= (*cv_step_range_data)[chg_mode][j].vol_max_mv - 50)
		vote(wls_dev->nor_icl_votable, CV_VOTER, false, 0, false);

	return 0;
}

static int oplus_chg_wls_set_non_ffc_current(struct oplus_chg_wls *wls_dev) {
	int i = 0;
	int rc = 0;
	int batt_vol_mv = 0;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	switch (wls_status->wls_type) {
	case OPLUS_CHG_WLS_BPP:
		i = OPLUS_WLS_CHG_MODE_BPP;
		break;
	case OPLUS_CHG_WLS_EPP:
		i = OPLUS_WLS_CHG_MODE_EPP;
		break;
	case OPLUS_CHG_WLS_EPP_PLUS:
		i = OPLUS_WLS_CHG_MODE_EPP_PLUS;
		break;
	case OPLUS_CHG_WLS_VOOC:
		i = OPLUS_WLS_CHG_MODE_AIRVOOC;
		break;
	case OPLUS_CHG_WLS_SVOOC:
	case OPLUS_CHG_WLS_PD_65W:
		i = OPLUS_WLS_CHG_MODE_AIRSVOOC;
		break;
	default:
		i = OPLUS_WLS_CHG_MODE_BPP;
		break;
	}

	rc = oplus_chg_wls_get_vbat(wls_dev, &batt_vol_mv);
	if (rc < 0) {
		chg_err("can't get batt vol, rc=%d\n", rc);
		if (i == OPLUS_WLS_CHG_MODE_AIRSVOOC)
			vote(wls_dev->nor_icl_votable, NON_FFC_VOTER, true,
				12000 * 1000 / wls_dev->dynamic_config.fastchg_init_vout_mv, true);
		return rc;
	}

	if (wls_dev->support_non_fcc_with_temp == true)
		oplus_chg_wls_set_non_ffc_current_cc_with_temp(wls_dev, i, batt_vol_mv);
	else
		oplus_chg_wls_set_non_ffc_current_cc(wls_dev, i, batt_vol_mv);

	oplus_chg_wls_set_non_ffc_current_cv(wls_dev, i, batt_vol_mv);
	return 0;
}

static int oplus_chg_wls_get_third_adapter_ext_cmd_p_id(struct oplus_chg_wls *wls_dev)
{
	int rc = 0;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int try_count = 0;
	char buf[3] = {0};
	int soc = 0, temp = 0;
	int max_wls_power = 0;

	if (wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY)
		return rc;

	if (!wls_status->tx_extern_cmd_done) {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_EXTERN_CMD, 0xff, 5);
		if (rc < 0) {
			chg_err("can't extern cmd, rc=%d\n", rc);
			wls_status->tx_extern_cmd_done = false;
			return rc;
		}
		msleep(200);
	}

	if (!wls_status->tx_product_id_done) {
		buf[0] = (wls_dev->wls_phone_id >> 8) & 0xff;
		buf[1] = wls_dev->wls_phone_id & 0xff;
		chg_info("wls_phone_id=0x%x\n", wls_dev->wls_phone_id);
		max_wls_power = wls_status->pwr_max_mw > wls_dev->wls_power_mw ? wls_dev->wls_power_mw : wls_status->pwr_max_mw;
		max_wls_power = max_wls_power / 1000;
		/*buf[2] is Pld, power = 10 + pld[bit0:bit3] * 10 + pld[bit4:bit7]*/
		buf[2] = ((((max_wls_power - 10) / 10) % 10) & 0xf) | ((((max_wls_power - 10) % 10) & 0xf) << 4);
		chg_info("max_wls_power=%d, pwr_max_mw:%d, buf[2]:%d\n", max_wls_power, wls_status->pwr_max_mw, buf[2]);
		do {
			rc = oplus_chg_wls_send_data(wls_dev, WLS_CMD_GET_PRODUCT_ID, buf, 5);
			if (rc < 0) {
				if (rc != -EAGAIN)
					try_count++;
				msleep(200);
			}
		} while (rc < 0 && try_count < 2 && wls_status->rx_online);
		if (rc < 0 || !wls_status->rx_online) {
			chg_err("can't get product id, rc=%d\n", rc);
			wls_status->tx_product_id_done = false;
			return rc;
		}
		msleep(500);
	}

	rc = oplus_chg_wls_get_ui_soc(wls_dev, &soc);
	if (rc < 0) {
		chg_err("can't get ui soc, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_chg_wls_get_batt_temp(wls_dev, &temp);
	if (rc < 0) {
		chg_err("can't get batt temp, rc=%d\n", rc);
		return rc;
	}

	buf[0] = (temp >> 8) & 0xff;
	buf[1] = temp & 0xff;
	buf[2] = soc & 0xff;
	chg_info("soc:%d, temp:%d\n", soc, temp);
	oplus_chg_wls_send_data(wls_dev, WLS_CMD_SEND_BATT_TEMP_SOC, buf, 0);

	msleep(1000);
	return rc;
}

static int oplus_chg_wls_get_third_adapter_v_id(struct oplus_chg_wls *wls_dev)
{
	int rc = 0;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY)
		return rc;

	if (!wls_status->verify_by_aes) {
		rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_VENDOR_ID, 0xff, 5);
		if (rc < 0) {
			chg_err("can't vendor id, rc=%d\n", rc);
			wls_status->verify_by_aes = false;
			wls_status->adapter_type = WLS_ADAPTER_TYPE_UNKNOWN;
			return rc;
		}
	}

	return rc;
}

static int oplus_chg_wls_send_match_q(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	u8 q_value, f_value;
	u8 data_buf[2] = {0};
	enum oplus_chg_wls_rx_mode rx_mode;

	if ((wls_status->adapter_type == WLS_ADAPTER_TYPE_VOOC) ||
	    (wls_status->adapter_type == WLS_ADAPTER_TYPE_SVOOC) ||
	    (wls_status->adapter_type == WLS_ADAPTER_TYPE_PD_65W)) {
		q_value = oplus_chg_wls_get_fastchg_qf_value(wls_dev, wls_status->adapter_id, Q_VALUE);
		f_value = oplus_chg_wls_get_fastchg_qf_value(wls_dev, wls_status->adapter_id, F_VALUE);
		data_buf[1] = q_value;
		if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30 &&
			wls_status->adapter_id >= WLS_ADAPTER_MODEL_4 &&
			wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY) {
			msleep(10);
			if (!wls_dev->magcvr_update_complete)
				data_buf[0] = AP_HALL_NON_MAG;
			else if (wls_dev->magcvr_status)
				data_buf[0] = AP_HALL_MAG;
			else
				data_buf[0] = AP_HALL_NON_MAG;

			data_buf[0] = data_buf[0] | f_value;
			(void)oplus_chg_wls_rx_send_match_q_response(wls_dev, data_buf);
		} else {
			(void)oplus_chg_wls_rx_send_match_q(wls_dev->wls_rx->rx_ic, data_buf);
		}
		return 0;
	}

	oplus_chg_wls_rx_get_rx_mode(wls_dev->wls_rx->rx_ic, &rx_mode);
	if (rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_5W || rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP ||
	    rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_PLUS) {
		data_buf[0] = oplus_chg_wls_get_epp_qf_value(wls_dev, WLS_EPP_QF_PARM_ID_DEFAULT, F_VALUE);
		data_buf[1] = oplus_chg_wls_get_epp_qf_value(wls_dev, WLS_EPP_QF_PARM_ID_DEFAULT, Q_VALUE);
		if (data_buf[0] != 0 && data_buf[1] != 0)
			(void)oplus_chg_wls_rx_send_epp_match_q(wls_dev->wls_rx->rx_ic, data_buf);
	}

	return 0;
}

static void oplus_chg_send_match_q_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_match_q_work);

	oplus_chg_wls_send_match_q(wls_dev);
}

#define WAIT_FOR_2CEP_INTERVAL_MS	500
static int oplus_chg_wls_rx_handle_state_default(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
#ifdef WLS_SUPPORT_OPLUS_CHG
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int real_soc = 100;
#endif
	enum oplus_chg_temp_region temp_region;
	enum oplus_chg_wls_rx_mode rx_mode;
	int rc;
	bool psy_changed = false;

#ifdef WLS_SUPPORT_OPLUS_CHG
	vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
	oplus_chg_wls_rx_get_rx_mode(wls_dev->wls_rx->rx_ic, &rx_mode);
	switch (rx_mode) {
	case OPLUS_CHG_WLS_RX_MODE_EPP_5W:
		wls_status->epp_5w = true;
		fallthrough;
	case OPLUS_CHG_WLS_RX_MODE_EPP:
		wls_status->epp_working = true;
		wls_status->wls_type = OPLUS_CHG_WLS_EPP;
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP;
		(void)oplus_chg_wls_rx_get_tx_id(wls_dev->wls_rx->rx_ic, &wls_status->tx_manu_id);
		if (wls_status->tx_manu_id) {
			wls_status->tx_product_id_done = true;
			(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					oplus_chg_wls_get_fod_parm(wls_dev, wls_status->tx_manu_id, FOD_EPP_MODE),
					wls_dev->static_config.fod_parm_len,
					FOD_EPP_MODE, wls_dev->magcvr_status);
		}
		goto out;
	case OPLUS_CHG_WLS_RX_MODE_EPP_PLUS:
		wls_status->epp_working = true;
		wls_status->wls_type = OPLUS_CHG_WLS_EPP_PLUS;
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
		(void)oplus_chg_wls_rx_get_tx_id(wls_dev->wls_rx->rx_ic, &wls_status->tx_manu_id);
		if (wls_status->tx_manu_id) {
			wls_status->tx_product_id_done = true;
			(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					oplus_chg_wls_get_fod_parm(wls_dev, wls_status->tx_manu_id, FOD_EPP_MODE),
					wls_dev->static_config.fod_parm_len,
					FOD_EPP_MODE, wls_dev->magcvr_status);
		}
		goto out;
	default:
		break;
	}

	temp_region = oplus_chg_wls_get_temp_region(wls_dev);

	vote(wls_dev->nor_input_disable_votable, USER_VOTER, true, 0, false);

	if (wls_status->verity_pass && wls_dev->force_rx_mode != OPLUS_CHG_WLS_RX_MODE_BPP &&
	    !wls_status->fastchg_timeout_to_bpp) {
		if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30)
			(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_INDENTIFY_ADAPTER, wls_dev->wls_phone_id, 0);
		else
			(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_INDENTIFY_ADAPTER, 0xff, 0);
		msleep(WAIT_FOR_2CEP_INTERVAL_MS);
	}

	vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, USER_VOTER, true, 750, true);

	if (wls_dev->force_rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
		wls_status->wls_type = OPLUS_CHG_WLS_BPP;
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
		goto out;
	}
	if (wls_status->verity_pass && wls_status->adapter_type == WLS_ADAPTER_TYPE_UNKNOWN &&
	    !wls_status->fastchg_timeout_to_bpp) {
		if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30)
			rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_INDENTIFY_ADAPTER, wls_dev->wls_phone_id, 5);
		else
			rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_INDENTIFY_ADAPTER, 0xff, 5);
		msleep(200);
	} else {
		rc = -EINVAL;
	}
	if (rc < 0 && wls_status->adapter_type == WLS_ADAPTER_TYPE_UNKNOWN) {
		chg_info("can't get adapter type, rc=%d\n", rc);
		wls_status->wls_type = OPLUS_CHG_WLS_UNKNOWN;
	} else {
		switch (wls_status->adapter_type) {
		case WLS_ADAPTER_TYPE_USB:
		case WLS_ADAPTER_TYPE_NORMAL:
			wls_status->wls_type = OPLUS_CHG_WLS_BPP;
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
			goto out;
		/*case WLS_ADAPTER_TYPE_EPP:
			rc = oplus_chg_wls_rx_get_rx_mode(wls_dev->wls_rx, &rx_mode);
			if (rc < 0) {
				chg_err("get rx mode error, rc=%d\n", rc);
				wls_status->wls_type = OPLUS_CHG_WLS_EPP;
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP;
				goto out;
			}
			if (rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_PLUS) {
				wls_status->wls_type = OPLUS_CHG_WLS_EPP_PLUS;
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
			} else if (rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP){
				wls_status->wls_type = OPLUS_CHG_WLS_EPP;
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP;
			} else {
				wls_status->wls_type = OPLUS_CHG_WLS_BPP;
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
			}
			goto out;*/
		case WLS_ADAPTER_TYPE_VOOC:
			rc = oplus_chg_wls_get_third_adapter_v_id(wls_dev);
			if (rc < 0) {
				wls_status->wls_type = OPLUS_CHG_WLS_BPP;
				goto out;
			}

			if (wls_dev->support_fastchg) {
				wls_status->wls_type = OPLUS_CHG_WLS_VOOC;
				psy_changed = true;
			} else {
				wls_status->epp_working = true;
				wls_status->wls_type = OPLUS_CHG_WLS_EPP_PLUS;
			}
			if (temp_region > BATT_TEMP_LITTLE_COLD && temp_region < BATT_TEMP_WARM) {
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
			} else {
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
			}
			goto out;
		case WLS_ADAPTER_TYPE_SVOOC:
		case WLS_ADAPTER_TYPE_PD_65W:
			rc = oplus_chg_wls_get_third_adapter_v_id(wls_dev);
			if (rc < 0) {
				wls_status->wls_type = OPLUS_CHG_WLS_BPP;
				goto out;
			}

			if (wls_dev->support_fastchg) {
				if (wls_status->adapter_type == WLS_ADAPTER_TYPE_SVOOC)
					wls_status->wls_type = OPLUS_CHG_WLS_SVOOC;
				else
					wls_status->wls_type = OPLUS_CHG_WLS_PD_65W;
				psy_changed = true;
				if (!wls_dev->batt_charge_enable)
					wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
				else
					wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
				/*
		 		* The fan of the 30w wireless charger cannot be reset automatically.
		 		* Actively turn on the fan once when wireless charging is connected.
		 		*/
				if (wls_status->adapter_id == WLS_ADAPTER_MODEL_0 &&
				    !wls_status->quiet_mode_init &&
				    !wls_status->switch_quiet_mode)
					(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_SET_NORMAL_MODE, 0xff, 2);

				rc = oplus_chg_wls_get_real_soc(wls_dev, &real_soc);
				if ((rc < 0) || (real_soc >= dynamic_cfg->fastchg_max_soc)) {
					chg_err("can't get real soc or soc is too high, rc=%d\n", rc);
					wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
				}
				if ((temp_region < BATT_TEMP_COOL) || (temp_region > BATT_TEMP_LITTLE_WARM)) {
					chg_err("Abnormal battery temperature, temp_region=%d\n", temp_region);
					vote(wls_dev->fastchg_disable_votable, QUIET_VOTER, true, 0, false);
					wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
				}
			} else {
				wls_status->epp_working = true;
				wls_status->wls_type = OPLUS_CHG_WLS_EPP_PLUS;
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
			}
			if (wls_dev->debug_mode)
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DEBUG;
			goto out;
		default:
			wls_status->wls_type = OPLUS_CHG_WLS_UNKNOWN;
		}
	}
#endif
	rc = oplus_chg_wls_rx_get_rx_mode(wls_dev->wls_rx->rx_ic, &rx_mode);
	if (rc < 0) {
		chg_err("get rx mode error, rc=%d\n", rc);
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
		goto out;
	}

	switch (rx_mode) {
	case OPLUS_CHG_WLS_RX_MODE_BPP:
		wls_status->wls_type = OPLUS_CHG_WLS_BPP;
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
		break;
	case OPLUS_CHG_WLS_RX_MODE_EPP_5W:
		wls_status->epp_5w = true;
		fallthrough;
	case OPLUS_CHG_WLS_RX_MODE_EPP:
		wls_status->epp_working = true;
		wls_status->wls_type = OPLUS_CHG_WLS_EPP;
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP;
		break;
	case OPLUS_CHG_WLS_RX_MODE_EPP_PLUS:
		wls_status->epp_working = true;
		wls_status->wls_type = OPLUS_CHG_WLS_EPP_PLUS;
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
		break;
	default:
		wls_status->wls_type = OPLUS_CHG_WLS_BPP;
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
		break;
	}

out:
	/*if (is_comm_ocm_available(wls_dev))
		oplus_chg_comm_update_config(wls_dev->comm_ocm);*/
	if (psy_changed && is_wls_psy_available(wls_dev))
		power_supply_changed(wls_dev->wls_psy);
	oplus_chg_wls_config(wls_dev);
	oplus_chg_wls_non_ffc_current_batt_cl_init(wls_dev);
	oplus_chg_wls_publish_wls_type(wls_dev, false, false);
	return 0;
}

static int oplus_chg_wls_rx_exit_state_default(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	switch (wls_status->target_rx_state) {
	case OPLUS_CHG_WLS_RX_STATE_BPP:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
		oplus_chg_wls_nor_set_aicl_enable(wls_dev->wls_nor->nor_ic, true);
		break;
	case OPLUS_CHG_WLS_RX_STATE_EPP:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP;
		oplus_chg_wls_nor_set_aicl_enable(wls_dev->wls_nor->nor_ic, true);
		break;
	case OPLUS_CHG_WLS_RX_STATE_EPP_PLUS:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
		oplus_chg_wls_nor_set_aicl_enable(wls_dev->wls_nor->nor_ic, true);
		break;
	case OPLUS_CHG_WLS_RX_STATE_FAST:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_QUIET:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_QUIET;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_STOP:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 0, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_DEBUG:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_DEBUG;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_DONE:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		break;
	default:
		chg_err("unsupported target status(=%s)\n", oplus_chg_wls_rx_state_text[wls_status->target_rx_state]);
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		break;
	}

	return 0;
}

#define WLS_CURR_OP_TRX_CHG_MA	600
static int oplus_chg_wls_rx_enter_state_bpp(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int icl_max_ma = wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_BPP];
	int wait_time_ms = 0;
#ifdef WLS_SUPPORT_OPLUS_CHG
	int rc = 0;

	if (wls_dev->force_rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
		if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER) > 0) {
			wls_status->epp_force_to_bpp_flag = false;
			cancel_delayed_work(&wls_dev->wls_epp_check_flag_clear_work);
		}
	}

	switch(wls_status->state_sub_step) {
	case 0:
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 1200, false);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 750, true);
		if (wls_status->adapter_type != WLS_ADAPTER_TYPE_UNKNOWN) {
			/* (void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, 5000, 0); */
		} else {
			rc = oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_TX_ID, 0xff, 5);
			if (rc < 0)
				chg_info("can't get tx id, it's not op tx\n");
		}
		wait_time_ms = 100;
		wls_status->state_sub_step = 1;
		break;
	case 1:
		if (wls_status->is_op_trx) {
			vote(wls_dev->nor_icl_votable, USER_VOTER, true, WLS_CURR_OP_TRX_CHG_MA, false);
			oplus_chg_wls_nor_set_aicl_enable(wls_dev->wls_nor->nor_ic, false);
			oplus_chg_wls_nor_set_aicl_enable(wls_dev->wls_nor->nor_ic, true);
			wls_status->state_sub_step = 0;
			wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
		} else {
			wls_status->state_sub_step = 2;
			wait_time_ms = 100;
		}
		wls_status->adapter_info_cmd_count = 0;
		break;
	case 2:
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, icl_max_ma, true);
		wls_status->state_sub_step = 0;
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
		break;
	default:
		wls_status->state_sub_step = 0;
		break;
	}
#else
	switch (wls_status->state_sub_step) {
	case 0:
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 2000, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 1;
		break;
	case 1:
		oplus_chg_wls_nor_set_aicl_enable(wls_dev->wls_nor, true);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 200, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 2;
		break;
	case 2:
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 400, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 3;
		break;
	case 3:
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 700, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 4;
		break;
	case 4:
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 1000, true);
		wls_status->state_sub_step = 0;
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
		break;
	default:
		wls_status->state_sub_step = 0;
		break;
	}
#endif
	return wait_time_ms;
}

#define A1_COUNT_MAX	30
static int oplus_chg_wls_rx_handle_state_bpp(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int iout_ma, vout_mv, vrect_mv;
	int wait_time_ms = 4000;
#ifdef WLS_SUPPORT_OPLUS_CHG
	int rc;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int real_soc = 100;
	enum oplus_chg_temp_region temp_region;
	enum oplus_chg_wls_rx_mode rx_mode;
	bool psy_changed = false;

	temp_region = oplus_chg_wls_get_temp_region(wls_dev);

	if (wls_dev->factory_mode && wls_dev->support_get_tx_pwr) {
		(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_TX_PWR, 0xff, 0);
		wait_time_ms = 3000;
	} else {
		if (wls_status->adapter_info_cmd_count < A1_COUNT_MAX && wls_status->verity_pass &&
		    wls_dev->force_rx_mode != OPLUS_CHG_WLS_RX_MODE_BPP && !wls_status->fastchg_timeout_to_bpp) {
			if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30)
				(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_INDENTIFY_ADAPTER, wls_dev->wls_phone_id, 0);
			else
				(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_INDENTIFY_ADAPTER, 0xff, 0);
			msleep(200);
			wait_time_ms = 1000;
			wls_status->adapter_info_cmd_count++;
		}
	}
#endif

	if (wls_dev->batt_charge_enable) {
		if (!!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
		}
	} else {
		if (!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, true, WLS_CURR_STOP_CHG_MA, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, true, 0, false);
		}
	}

#ifdef WLS_SUPPORT_OPLUS_CHG
	if (wls_dev->force_rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
		wls_status->wls_type = OPLUS_CHG_WLS_BPP;
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
		goto out;
	}
	switch (wls_status->adapter_type) {
	case WLS_ADAPTER_TYPE_EPP:
		rc = oplus_chg_wls_rx_get_rx_mode(wls_dev->wls_rx->rx_ic, &rx_mode);
		if (rc < 0) {
			chg_err("get rx mode error, rc=%d\n", rc);
			wls_status->wls_type = OPLUS_CHG_WLS_EPP_PLUS;
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
			break;
		}
		if (rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_PLUS) {
			wls_status->wls_type = OPLUS_CHG_WLS_EPP_PLUS;
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
		} else {
			wls_status->wls_type = OPLUS_CHG_WLS_EPP;
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP;
		}
		break;
	case WLS_ADAPTER_TYPE_VOOC:
		rc = oplus_chg_wls_get_third_adapter_v_id(wls_dev);
		if (rc < 0) {
			wls_status->wls_type = OPLUS_CHG_WLS_BPP;
			goto out;
		}

		if (wls_dev->support_fastchg) {
			wls_status->wls_type = OPLUS_CHG_WLS_VOOC;
			psy_changed = true;
		} else {
			wls_status->epp_working = true;
			wls_status->wls_type = OPLUS_CHG_WLS_EPP_PLUS;
		}
		if (temp_region > BATT_TEMP_LITTLE_COLD && temp_region < BATT_TEMP_WARM) {
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DEFAULT;
		} else {
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_BPP;
		}
		break;
	case WLS_ADAPTER_TYPE_SVOOC:
	case WLS_ADAPTER_TYPE_PD_65W:
		rc = oplus_chg_wls_get_third_adapter_v_id(wls_dev);
		if (rc < 0) {
			wls_status->wls_type = OPLUS_CHG_WLS_BPP;
			goto out;
		}

		if (wls_dev->support_fastchg) {
			if (wls_status->adapter_type == WLS_ADAPTER_TYPE_SVOOC)
				wls_status->wls_type = OPLUS_CHG_WLS_SVOOC;
			else
				wls_status->wls_type = OPLUS_CHG_WLS_PD_65W;
			psy_changed = true;
			if (!wls_dev->batt_charge_enable)
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
			else
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
			rc = oplus_chg_wls_get_real_soc(wls_dev, &real_soc);
			if ((rc < 0) || (real_soc >= dynamic_cfg->fastchg_max_soc)) {
				chg_err("can't get real soc or soc is too high, rc=%d\n", rc);
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
			}
			if ((temp_region < BATT_TEMP_LITTLE_COLD) || (temp_region > BATT_TEMP_WARM)) {
				chg_err("Abnormal battery temperature, temp_region=%d\n", temp_region);
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
			}
		} else {
			wls_status->epp_working = true;
			wls_status->wls_type = OPLUS_CHG_WLS_EPP_PLUS;
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
		}
		if (wls_dev->debug_mode)
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DEBUG;
		break;
	default:
		goto out;
	}
	wait_time_ms = 100;
	if (psy_changed && is_wls_psy_available(wls_dev))
		power_supply_changed(wls_dev->wls_psy);
	oplus_chg_wls_config(wls_dev);
out:
#endif
	(void)oplus_chg_wls_nor_skin_check(wls_dev);
	oplus_chg_wls_check_term_charge(wls_dev);
	oplus_chg_wls_choose_bpp_epp_curve(wls_dev);
	oplus_chg_wls_set_non_ffc_current(wls_dev);
	oplus_chg_wls_publish_wls_type(wls_dev, false, false);

	iout_ma = wls_status->iout_ma;
	vout_mv = wls_status->vout_mv;
	vrect_mv = wls_status->vrect_mv;
	chg_info("wkcs: iout=%d, vout=%d, vrect=%d\n", iout_ma, vout_mv, vrect_mv);

	return wait_time_ms;
}

static int oplus_chg_wls_rx_exit_state_bpp(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	switch (wls_status->target_rx_state) {
	case OPLUS_CHG_WLS_RX_STATE_EPP:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP;
		oplus_chg_wls_nor_set_aicl_enable(wls_dev->wls_nor->nor_ic, true);
		break;
	case OPLUS_CHG_WLS_RX_STATE_EPP_PLUS:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
		oplus_chg_wls_nor_set_aicl_enable(wls_dev->wls_nor->nor_ic, true);
		break;
	case OPLUS_CHG_WLS_RX_STATE_FAST:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_QUIET:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_QUIET;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_STOP:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 0, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_DEBUG:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_DEBUG;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_DONE:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_DEFAULT:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_DEFAULT;
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_DEFAULT;
		break;
	default:
		chg_err("unsupported target status(=%s)\n", oplus_chg_wls_rx_state_text[wls_status->target_rx_state]);
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		break;
	}

	return 0;
}

#define SKEWING_WORK_DELAY	3000
#define VOUT_OK_OFFSET		500
#define WAIT_FOR_FAST_MS	500
static int oplus_chg_wls_rx_enter_state_epp(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int icl_max_ma = wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_EPP];
	int wait_time_ms = 0;
	int iout_ma = 0;
	int factor = vout_factor(wls_dev);

#ifdef WLS_SUPPORT_OPLUS_CHG
	switch(wls_status->state_sub_step) {
	case 0:
		if (wls_dev->force_rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
			if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_CAMERA_MODE_VOTER) > 0)
				oplus_chg_wls_entry_rx_mode(wls_dev, OPLUS_CHG_WLS_RX_MODE_BPP, WLS_CAMERA_MODE_VOTER);
			return 200;
		}
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 2800, false);
		if (wls_status->adapter_type == WLS_ADAPTER_TYPE_VOOC ||
		    wls_status->adapter_type == WLS_ADAPTER_TYPE_SVOOC ||
		    wls_status->adapter_type == WLS_ADAPTER_TYPE_PD_65W) {
			cancel_delayed_work_sync(&wls_dev->wls_nor->icl_set_work);
			vote(wls_dev->nor_icl_votable, USER_VOTER, true, 100, false);
			vote(wls_dev->nor_input_disable_votable, USER_VOTER, true, 0, false);
			oplus_chg_wls_rx_get_iout(wls_dev->wls_rx->rx_ic, &iout_ma);
			if (iout_ma > 200) {
				wait_time_ms = 100;
				break;
			}
			(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->vooc_vol_mv, 0);
			if (wls_dev->support_fastchg)
				oplus_chg_wls_cmd_into_fastchage(wls_dev, 0xff, -1);
			else
				oplus_chg_wls_cmd_into_fastchage(wls_dev, 0x01, -1);
			wls_status->fod_parm_for_fastchg = true;
			(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->vooc_vol_mv, 15);

			if (wls_status->pwr_max_mw > 0) {
				vote(wls_dev->nor_icl_votable, RX_MAX_VOTER, true,
				     wls_status->pwr_max_mw * 1000 /dynamic_cfg->vooc_vol_mv, false);
			}
		}
		wait_time_ms = 100;
		wls_status->state_sub_step = 1;
		break;
	case 1:
		if (wls_status->adapter_type == WLS_ADAPTER_TYPE_VOOC && wls_status->charge_type != WLS_CHARGE_TYPE_FAST)
			return WAIT_FOR_FAST_MS;
		if (wls_status->adapter_type == WLS_ADAPTER_TYPE_VOOC
				|| wls_status->adapter_type == WLS_ADAPTER_TYPE_SVOOC
				|| wls_status->adapter_type == WLS_ADAPTER_TYPE_PD_65W) {
			if (wls_status->vout_mv > dynamic_cfg->epp_vol_mv - VOUT_OK_OFFSET) {
				vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
				oplus_chg_wls_nor_set_vindpm(wls_dev->wls_nor->nor_ic, WLS_VINDPM_AIRVOOC);
				vote(wls_dev->nor_icl_votable, USER_VOTER, true, icl_max_ma, true);
				wls_status->state_sub_step = 0;
				wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP;
			} else {
				vote(wls_dev->nor_icl_votable, USER_VOTER, true, 300, true);
			}
		} else {
			if (wls_status->epp_5w || wls_status->vout_mv < 8000 / factor)
				vote(wls_dev->nor_icl_votable, USER_VOTER, true, 450, true);
			else
				vote(wls_dev->nor_icl_votable, USER_VOTER, true, icl_max_ma, true);

			wls_status->state_sub_step = 0;
			wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP;
		}
		wait_time_ms = 100;
		break;
	default:
		wls_status->state_sub_step = 0;
		break;
	}
#else
	switch(wls_status->state_sub_step) {
	case 0:
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 2000, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 1;
		break;
	case 1:
		oplus_chg_wls_nor_set_aicl_enable(wls_dev->wls_nor, true);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 200, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 2;
		break;
	case 2:
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 3;
		break;
	case 3:
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 900, false);
		wls_status->state_sub_step = 0;
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP;
		break;
	default:
		wls_status->state_sub_step = 0;
		break;
	}
#endif
	if (wls_status->current_rx_state == OPLUS_CHG_WLS_RX_STATE_EPP)
		schedule_delayed_work(&wls_dev->wls_skewing_work, msecs_to_jiffies(SKEWING_WORK_DELAY));

	return wait_time_ms;
}

#define EPP_STRAGE_CHANGE_WAIT 60
static int oplus_chg_wls_choose_bpp_epp_curve(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_fcc_step *fcc_cfg;
	int batt_temp = 0;
	int rc, i;
	int last_epp_chg_level;
	static int last_wls_type = -1;

	rc = oplus_chg_wls_get_batt_temp(wls_dev, &batt_temp);
	if (rc < 0) {
		chg_err("can't get batt temp, rc=%d\n", rc);
		return -EPERM;
	}

	if (last_wls_type != wls_status->wls_type) {
		vote(wls_dev->nor_icl_votable, EPP_CURVE_VOTER, false, 0, false);
		wls_status->epp_chg_level = -1;
		last_wls_type = wls_status->wls_type;
	}

	switch (wls_status->wls_type) {
	case OPLUS_CHG_WLS_EPP:
		fcc_cfg = &dynamic_cfg->epp_steps;
		break;
	case OPLUS_CHG_WLS_EPP_PLUS:
		fcc_cfg = &dynamic_cfg->epp_plus_steps;
		break;
	case OPLUS_CHG_WLS_SVOOC:
	case OPLUS_CHG_WLS_PD_65W:
		if (wls_status->switch_quiet_mode)
			fcc_cfg = &dynamic_cfg->epp_plus_steps;
		else
			return -EPERM;
		break;
	case OPLUS_CHG_WLS_BPP:
		fcc_cfg = &dynamic_cfg->bpp_steps;
		break;
	default:
		return -EPERM;
	}

	if (wls_status->epp_chg_level == -1) {
		for (i = 0; i < fcc_cfg->max_step; i++) {
			if (batt_temp < fcc_cfg->fcc_step[i].high_threshold)
				break;
		}
		wls_status->epp_chg_level = (i >= fcc_cfg->max_step) ? (fcc_cfg->max_step - 1) : i;
		dynamic_cfg->epp_steps.fcc_wait_timeout = jiffies + HZ * EPP_STRAGE_CHANGE_WAIT;
		vote(wls_dev->nor_icl_votable, EPP_CURVE_VOTER, true,
			fcc_cfg->fcc_step[wls_status->epp_chg_level].curr_ma, true);
		oplus_chg_wls_high_temp_check(HIGH_TEMP_VOTER_CURVE, wls_dev,
			fcc_cfg->fcc_step[wls_status->epp_chg_level].curr_ma);
		return 0;
	}

	if (!time_after(jiffies, dynamic_cfg->epp_steps.fcc_wait_timeout))
		return 0;

	last_epp_chg_level = wls_status->epp_chg_level;
	if (batt_temp > fcc_cfg->fcc_step[wls_status->epp_chg_level].high_threshold &&
	    (wls_status->epp_chg_level + 1 < fcc_cfg->max_step)) {
		wls_status->epp_chg_level += 1;
		dynamic_cfg->epp_steps.fcc_wait_timeout = jiffies + HZ * EPP_STRAGE_CHANGE_WAIT;
	} else if (batt_temp < fcc_cfg->fcc_step[wls_status->epp_chg_level].low_threshold &&
		   wls_status->epp_chg_level - 1 >= 0) {
			wls_status->epp_chg_level -= 1;
			dynamic_cfg->epp_steps.fcc_wait_timeout = jiffies + HZ * EPP_STRAGE_CHANGE_WAIT;
	}
	vote(wls_dev->nor_icl_votable, EPP_CURVE_VOTER, true, fcc_cfg->fcc_step[wls_status->epp_chg_level].curr_ma, true);

	oplus_chg_wls_high_temp_check(HIGH_TEMP_VOTER_CURVE, wls_dev,
		fcc_cfg->fcc_step[wls_status->epp_chg_level].curr_ma);

	if (last_epp_chg_level != wls_status->epp_chg_level)
		chg_info("[%s %d %d %d]\n", oplus_chg_wls_rx_state_text[wls_status->wls_type],
			batt_temp, wls_status->epp_chg_level, fcc_cfg->max_step);

	return 0;
}

static int oplus_chg_wls_rx_handle_state_epp(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int icl_max_ma = wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_EPP];
	int iout_ma, icl_ma, vout_mv, vrect_mv;
	int nor_input_curr_ma;
	int wait_time_ms = 4000;
	int rc = 0;
	int factor = vout_factor(wls_dev);

#ifdef WLS_SUPPORT_OPLUS_CHG
	if (wls_dev->factory_mode && wls_dev->support_get_tx_pwr) {
		(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_TX_PWR, 0xff, 0);
		wait_time_ms = 3000;
	} else {
		oplus_chg_wls_check_quiet_mode(wls_dev);
	}
#endif

	if (wls_dev->batt_charge_enable) {
		if (!!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
		}
	} else {
		if (!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, true, WLS_CURR_STOP_CHG_MA, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, true, 0, false);
		}
	}

	if (wls_status->epp_5w || wls_status->vout_mv < 8000 / factor) {
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 450, true);
		if (!wls_status->epp_5w)
			wait_time_ms = 500;
	} else {
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, icl_max_ma, true);
	}

	(void)oplus_chg_wls_nor_skin_check(wls_dev);
	oplus_chg_wls_check_term_charge(wls_dev);
	oplus_chg_wls_choose_bpp_epp_curve(wls_dev);
	oplus_chg_wls_set_non_ffc_current(wls_dev);
	oplus_chg_wls_epp_force_to_bpp_loop_check(wls_dev);

	if ((wls_status->adapter_type != WLS_ADAPTER_TYPE_UNKNOWN) &&
	     !wls_status->verity_started) {
		icl_ma = get_effective_result(wls_dev->nor_icl_votable);
		rc = oplus_chg_wls_nor_get_input_curr(wls_dev->wls_nor->nor_ic, &nor_input_curr_ma);
		if (rc < 0)
			nor_input_curr_ma = wls_status->iout_ma;
		if (icl_ma - nor_input_curr_ma < 300)
			schedule_delayed_work(&wls_dev->wls_verity_work, msecs_to_jiffies(500));
	}

	iout_ma = wls_status->iout_ma;
	vout_mv = wls_status->vout_mv;
	vrect_mv = wls_status->vrect_mv;
	chg_info("wkcs: iout=%d, vout=%d, vrect=%d\n", iout_ma, vout_mv, vrect_mv);
	return wait_time_ms;
}

static int oplus_chg_wls_rx_exit_state_epp(struct oplus_chg_wls *wls_dev)
{
	/*struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc;*/

	return 0;
}

static int oplus_chg_wls_rx_enter_state_epp_plus(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int icl_max_ma = wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_EPP_PLUS];
	int wait_time_ms = 0;
	int iout_ma = 0;
	int factor = vout_factor(wls_dev);

#ifdef WLS_SUPPORT_OPLUS_CHG
	switch(wls_status->state_sub_step) {
	case 0:
		if (wls_dev->force_rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
			if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_CAMERA_MODE_VOTER) > 0)
				oplus_chg_wls_entry_rx_mode(wls_dev, OPLUS_CHG_WLS_RX_MODE_BPP, WLS_CAMERA_MODE_VOTER);
			wait_time_ms = 200;
			return wait_time_ms;
		}
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 4000, false);
		if (wls_status->adapter_type == WLS_ADAPTER_TYPE_VOOC ||
		    wls_status->adapter_type == WLS_ADAPTER_TYPE_SVOOC ||
		    wls_status->adapter_type == WLS_ADAPTER_TYPE_PD_65W) {
			icl_max_ma = wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_AIRVOOC];
			cancel_delayed_work_sync(&wls_dev->wls_nor->icl_set_work);
			vote(wls_dev->nor_icl_votable, USER_VOTER, true, 100, false);
			rerun_election(wls_dev->nor_icl_votable, false);
			vote(wls_dev->nor_input_disable_votable, USER_VOTER, true, 0, true);
			oplus_chg_wls_rx_get_iout(wls_dev->wls_rx->rx_ic, &iout_ma);
			if (iout_ma > 200) {
				wait_time_ms = 100;
				break;
			}
			(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->vooc_vol_mv, 0);
			if (wls_dev->support_fastchg)
				oplus_chg_wls_cmd_into_fastchage(wls_dev, 0xff, -1);
			else
				oplus_chg_wls_cmd_into_fastchage(wls_dev, 0x01, -1);
			wls_status->fod_parm_for_fastchg = true;
			(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->vooc_vol_mv, 15);
			if (wls_status->pwr_max_mw > 0) {
				vote(wls_dev->nor_icl_votable, RX_MAX_VOTER, true,
				     wls_status->pwr_max_mw * 1000 / dynamic_cfg->vooc_vol_mv, false);
			} else {
				/*vote a default max icl for vooc*/
				vote(wls_dev->nor_icl_votable, RX_MAX_VOTER, true,
					12000 * 1000 / dynamic_cfg->vooc_vol_mv, false);
			}
		}
		wait_time_ms = 100;
		wls_status->state_sub_step = 1;
		break;
	case 1:
		if (wls_status->adapter_type == WLS_ADAPTER_TYPE_VOOC && wls_status->charge_type != WLS_CHARGE_TYPE_FAST)
			return WAIT_FOR_FAST_MS;
		if (wls_status->adapter_type == WLS_ADAPTER_TYPE_VOOC
				|| wls_status->adapter_type == WLS_ADAPTER_TYPE_SVOOC
				|| wls_status->adapter_type == WLS_ADAPTER_TYPE_PD_65W) {
			icl_max_ma = wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_AIRVOOC];
			if (wls_status->vout_mv > dynamic_cfg->epp_plus_vol_mv - VOUT_OK_OFFSET) {
				vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
				oplus_chg_wls_nor_set_vindpm(wls_dev->wls_nor->nor_ic, WLS_VINDPM_AIRVOOC);
				vote(wls_dev->nor_icl_votable, USER_VOTER, true, icl_max_ma, true);
				wls_status->state_sub_step = 0;
				wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
			} else {
				vote(wls_dev->nor_icl_votable, USER_VOTER, true, 300, true);
			}
		} else {
			if (wls_status->vout_mv < 8000 / factor)
				vote(wls_dev->nor_icl_votable, USER_VOTER, true, 400, true);
			else
				vote(wls_dev->nor_icl_votable, USER_VOTER, true, icl_max_ma, true);
			wls_status->state_sub_step = 0;
			wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
		}

		wait_time_ms = 100;
		break;
	default:
		wls_status->state_sub_step = 0;
		break;
	}
#else
	switch(wls_status->state_sub_step) {
	case 0:
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 3000, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 1;
		break;
	case 1:
		oplus_chg_wls_nor_set_aicl_enable(wls_dev->wls_nor, true);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 200, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 2;
		break;
	case 2:
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 3;
		break;
	case 3:
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 900, false);
		wait_time_ms = 300;
		wls_status->state_sub_step = 4;
		break;
	case 4:
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 1250, false);
		wls_status->state_sub_step = 0;
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_EPP_PLUS;
		break;
	default:
		wls_status->state_sub_step = 0;
		break;
	}
#endif
	if (wls_status->current_rx_state == OPLUS_CHG_WLS_RX_STATE_EPP_PLUS)
		schedule_delayed_work(&wls_dev->wls_skewing_work, msecs_to_jiffies(SKEWING_WORK_DELAY));
	return wait_time_ms;
}

static int oplus_chg_wls_rx_handle_state_epp_plus(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int icl_max_ma = wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_EPP_PLUS];
	int iout_ma, icl_ma, vout_mv, vrect_mv;
	int nor_input_curr_ma;
	int wait_time_ms = 4000;
	int rc = 0;
	int factor = vout_factor(wls_dev);

#ifdef WLS_SUPPORT_OPLUS_CHG
	if (wls_dev->factory_mode && wls_dev->support_get_tx_pwr) {
		(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_TX_PWR, 0xff, 0);
		wait_time_ms = 3000;
	} else {
		oplus_chg_wls_check_quiet_mode(wls_dev);
	}
#endif

	if (wls_status->adapter_type == WLS_ADAPTER_TYPE_VOOC
			|| wls_status->adapter_type == WLS_ADAPTER_TYPE_SVOOC
			|| wls_status->adapter_type == WLS_ADAPTER_TYPE_PD_65W) {
		icl_max_ma = wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_AIRVOOC];
		if (wls_status->pwr_max_mw > 0) {
			vote(wls_dev->nor_icl_votable, RX_MAX_VOTER, true,
				wls_status->pwr_max_mw * 1000 / dynamic_cfg->vooc_vol_mv, false);
		}
	}

	if (wls_dev->batt_charge_enable) {
		if (!!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
		}
	} else {
		if (!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, true, WLS_CURR_STOP_CHG_MA, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, true, 0, false);
		}
	}

	if (wls_status->vout_mv < 8000 / factor) {
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 400, true);
		wait_time_ms = 500;
	} else {
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, icl_max_ma, true);
	}

	(void)oplus_chg_wls_nor_skin_check(wls_dev);
	oplus_chg_wls_check_term_charge(wls_dev);
	oplus_chg_wls_choose_bpp_epp_curve(wls_dev);
	oplus_chg_wls_set_non_ffc_current(wls_dev);
	oplus_chg_wls_epp_force_to_bpp_loop_check(wls_dev);

	if ((wls_status->adapter_type != WLS_ADAPTER_TYPE_UNKNOWN) &&
	     !wls_status->verity_started) {
		icl_ma = get_effective_result(wls_dev->nor_icl_votable);
		rc = oplus_chg_wls_nor_get_input_curr(wls_dev->wls_nor->nor_ic, &nor_input_curr_ma);
		if (rc < 0)
			nor_input_curr_ma = wls_status->iout_ma;
		if (icl_ma - nor_input_curr_ma < 300)
			schedule_delayed_work(&wls_dev->wls_verity_work, msecs_to_jiffies(500));
	}

	iout_ma = wls_status->iout_ma;
	vout_mv = wls_status->vout_mv;
	vrect_mv = wls_status->vrect_mv;
	chg_info("wkcs: iout=%d, vout=%d, vrect=%d\n", iout_ma, vout_mv, vrect_mv);
	return wait_time_ms;
}

static int oplus_chg_wls_rx_exit_state_epp_plus(struct oplus_chg_wls *wls_dev)
{
	/*struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc;*/

	return 0;
}

#define CP_OPEN_OFFSET		100
#define CURR_ERR_COUNT_MAX	200
static int oplus_chg_wls_rx_enter_state_fast(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_fcc_step *fcc_chg = &wls_dev->wls_fcc_step;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int vbat_mv, vout_mv, batt_num, batt_temp;
	static int curr_err_count;
	int delay_ms = 0;
	int real_soc;
	int iout_max_ma;
	int scale_factor;
	int rc;
	int iout_ma = 0;
	int i = 0;
	int factor = vout_factor(wls_dev);
	int vout_temp_mv;
	int bridge_mode = WLS_HALF_BRIDGE_MODE;

	if (wls_status->switch_quiet_mode &&
	    wls_status->state_sub_step > OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_FAST) {
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_QUIET;
		return 0;
	}

	if (!wls_dev->batt_charge_enable) {
		if (wls_status->state_sub_step > OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_FAST) {
			wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
			return 0;
		}
	} else {
		if (!!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
		}
	}

	rc = oplus_chg_wls_get_real_soc(wls_dev, &real_soc);
	if ((rc < 0) || (real_soc >= dynamic_cfg->fastchg_max_soc)) {
		chg_err("can't get real soc or soc is too high, rc=%d\n", rc);
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		return 0;
	}

	rc = oplus_chg_wls_get_vbat(wls_dev, &vbat_mv);
	if (rc < 0) {
		if (printk_ratelimit())
			chg_err("can't get vbat, rc=%d\n", rc);
		delay_ms = 100;
		return delay_ms;
	}

	rc = oplus_chg_wls_get_batt_temp(wls_dev, &batt_temp);
	if (rc < 0) {
		chg_err("can't get batt temp, rc=%d\n", rc);
		delay_ms = 100;
		return delay_ms;
	}

	chg_info("state_sub_step=%d\n", wls_status->state_sub_step);

	switch(wls_status->state_sub_step) {
	case OPLUS_CHG_WLS_FAST_SUB_STATE_INIT:
		rc = oplus_chg_wls_choose_fastchg_curve(wls_dev);
		if (rc < 0) {
			chg_err("choose fastchg curve failed, rc = %d\n", rc);
			return 0;
		}
		oplus_chg_wls_bcc_parms_init(wls_dev);
		oplus_chg_wls_bcc_get_curve(wls_dev);
		cancel_delayed_work_sync(&wls_dev->wls_nor->icl_set_work);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 100, false);
		rerun_election(wls_dev->nor_icl_votable, false);
		if (wls_status->vout_mv < 9000 / factor)
			vote(wls_dev->nor_input_disable_votable, USER_VOTER, true, 0, false);
		oplus_chg_wls_rx_get_iout(wls_dev->wls_rx->rx_ic, &iout_ma);
		if (iout_ma > 200) {
			delay_ms = 100;
			break;
		}
		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->fastchg_init_vout_mv, 0);
		oplus_chg_wls_cmd_into_fastchage(wls_dev, 0xff, -1);
		wls_status->fod_parm_for_fastchg = true;
		wls_status->state_sub_step = OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_FAST;
		oplus_chg_wls_get_verity_data(wls_dev);
		delay_ms = 100;
		wls_status->fastchg_enter_timer = jiffies + (unsigned long)(WLS_FASTCHG_ENTER_TIMEOUT * HZ);
		break;
	case OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_FAST:
		if (wls_status->charge_type != WLS_CHARGE_TYPE_FAST)
			return 500;
		/* Offset exit fast charge and restore, packet F2 is not returned due to the encrypted packet */
		if (wls_dev->static_config.fastchg_fod_enable) {
			if (wls_status->fod_parm_for_fastchg)
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					oplus_chg_wls_get_fod_parm(wls_dev, wls_status->adapter_id, FOD_FAST_MODE),
					wls_dev->static_config.fod_parm_len,
					FOD_FAST_MODE, wls_dev->magcvr_status);
			else if (wls_dev->static_config.fastchg_12v_fod_enable)
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					oplus_chg_wls_get_fod_parm_12v(wls_dev, wls_status->adapter_id),
					wls_dev->static_config.fod_parm_len,
					FOD_FAST_MODE, wls_dev->magcvr_status);
		} else {
			(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
				wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
				FOD_FAST_MODE, wls_dev->magcvr_status);
		}
		if (dynamic_cfg->full_bridge_vout_mv > 0) {
			(void)oplus_chg_wls_rx_get_bridge_mode(wls_dev->wls_rx->rx_ic, &bridge_mode);
			if (bridge_mode != WLS_FULL_BRIDGE_MODE) {
				(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->full_bridge_vout_mv, 0);
				if (time_after(jiffies, wls_status->fastchg_enter_timer) &&
				    !wls_status->fastchg_to_bpp_state_keep)
					oplus_chg_wls_fastchg_enforce_to_bpp(wls_dev, WLS_ERR_FULL_BRIDGE_ENTER_TIMEOUT_TO_BPP);
				return 500;
			}
		}
		rc = oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->fastchg_init_vout_mv,
						WLS_FASTCHG_ENTER_TIMEOUT);
		if (rc < 0) {
			if (time_after(jiffies, wls_status->fastchg_enter_timer) &&
			    !wls_status->fastchg_to_bpp_state_keep)
				oplus_chg_wls_fastchg_enforce_to_bpp(wls_dev, WLS_ERR_FASTCHG_ENTER_TIMEOUT_TO_BPP);
			return 500;
		}
		oplus_chg_wls_nor_set_vindpm(wls_dev->wls_nor->nor_ic, WLS_VINDPM_AIRSVOOC);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, WLS_CURR_WAIT_FAST_MA, false);
		vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
		if (wls_status->rx_online)
			(void)oplus_chg_wls_set_ext_pwr_enable(wls_dev, true);
		/*need about 600ms for 9415*/
		msleep(600);
		if (wls_status->rx_online)
			(void)oplus_chg_wls_rx_set_dcdc_enable(wls_dev->wls_rx->rx_ic, true);
		curr_err_count = 0;
		if (wls_status->rx_online && wls_status->adapter_id == WLS_ADAPTER_THIRD_PARTY) {
			rc = oplus_chg_wls_get_third_adapter_ext_cmd_p_id(wls_dev);
			if (rc < 0) {
				if (!wls_status->rx_online)
					return 0;
				chg_err("get product id fail\n");
				wls_status->online_keep = true;
				vote(wls_dev->rx_disable_votable, VERITY_VOTER, true, 1, false);
				schedule_delayed_work(&wls_dev->rx_verity_restore_work, msecs_to_jiffies(500));
				return 500;
			}
		}
		if (wls_status->iout_ma < 100) {
			wls_status->state_sub_step = OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_IOUT;
			curr_err_count++;
			delay_ms = 100;
		} else {
			wls_status->state_sub_step = OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_VOUT;
			curr_err_count = 0;
			delay_ms = 0;
		}
		break;
	case OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_IOUT:
		if (wls_status->iout_ma < 100) {
			if (curr_err_count == CURR_ERR_COUNT_MAX / 2)
				oplus_chg_wls_nor_set_aicl_rerun(wls_dev->wls_nor->nor_ic);
			curr_err_count++;
			delay_ms = 100;
		} else {
			curr_err_count = 0;
			delay_ms = 0;
			wls_status->state_sub_step = OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_VOUT;
		}
		if (curr_err_count > CURR_ERR_COUNT_MAX) {
			curr_err_count = 0;
			wls_status->state_sub_step = 0;
			vote(wls_dev->fastchg_disable_votable, CURR_ERR_VOTER, true, 1, false);
			wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
			delay_ms = 0;
		}
		break;
	case OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_VOUT:
		vout_temp_mv = 4 * vbat_mv / factor + dynamic_cfg->cp_open_offset_mv;
		rc = oplus_chg_wls_rx_set_vout_step(wls_dev->wls_rx, vout_temp_mv, 1000, 50);
		if (rc < 0) {
			chg_err("can't set vout to %d, rc=%d\n", vout_temp_mv, rc);
			wls_status->state_sub_step = 0;
			wls_status->fastchg_retry_count++;
			wls_status->fastchg_retry_timer = jiffies + (unsigned long)(300 * HZ); /*5 min*/
			vote(wls_dev->fastchg_disable_votable, STARTUP_CEP_VOTER, true, 1, false);
			wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
			delay_ms = 0;
			break;
		}
		wls_status->state_sub_step = OPLUS_CHG_WLS_FAST_SUB_STATE_CHECK_IOUT;
		delay_ms = 0;
		break;
	case OPLUS_CHG_WLS_FAST_SUB_STATE_CHECK_IOUT:
		rc = oplus_chg_wls_rx_get_vout(wls_dev->wls_rx->rx_ic, &vout_mv);
		if (rc < 0) {
			chg_err("can't get vout, rc=%d\n", rc);
			delay_ms = 100;
			break;
		}
		vout_temp_mv = vbat_mv * 4 / factor + dynamic_cfg->cp_open_offset_min_mv;
		if (vout_mv < vout_temp_mv) {
			chg_info("rx vout(=%d) < %d, retry\n", vout_mv, vout_temp_mv);
			vout_temp_mv = 4 * vbat_mv / factor + dynamic_cfg->cp_open_offset_mv + CP_OPEN_OFFSET;
			(void)oplus_chg_wls_rx_set_vout_step(wls_dev->wls_rx, vout_temp_mv, 1000, 10);
			delay_ms = 0;
			break;
		}
		wls_status->state_sub_step = OPLUS_CHG_WLS_FAST_SUB_STATE_START;
		delay_ms = 0;
		break;
	case OPLUS_CHG_WLS_FAST_SUB_STATE_START:
		if (!wls_status->switch_quiet_mode) {
			wls_status->quiet_mode_init = false;
			wls_status->fastchg_exit_fan_speed_set = false;
		}
		rc = oplus_chg_wls_fast_set_enable(wls_dev->wls_fast->fast_ic, true);
		if (rc < 0) {
			wls_status->state_sub_step = OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_VOUT;
			delay_ms = 100;
			break;
		}
		rc = oplus_chg_wls_fast_start(wls_dev->wls_fast->fast_ic);
		if (rc < 0) {
			wls_status->state_sub_step = OPLUS_CHG_WLS_FAST_SUB_STATE_WAIT_VOUT;
			delay_ms = 100;
			break;
		}
		rc = oplus_chg_wls_get_real_soc(wls_dev, &real_soc);
		if (rc < 0) {
			chg_err("can't get real soc, rc=%d\n", rc);
			goto err;
		}
		rc = oplus_chg_wls_get_batt_num(wls_dev, &batt_num);
		if (rc == 0) {
			scale_factor = dynamic_cfg->svooc_vol_mv / 5000 / batt_num;
		} else {
			scale_factor = 1;
		}
		if ((wls_status->wls_type == OPLUS_CHG_WLS_SVOOC) ||
		    (wls_status->wls_type == OPLUS_CHG_WLS_PD_65W)) {
			wls_status->fastchg_ibat_max_ma = dynamic_cfg->fastchg_curr_max_ma * scale_factor;
			iout_max_ma = WLS_FASTCHG_CURR_50W_MAX_MA * factor;
			if (wls_status->wls_type == OPLUS_CHG_WLS_SVOOC &&
			    iout_max_ma > WLS_FASTCHG_CURR_45W_MAX_MA * factor &&
			    wls_status->adapter_power <= WLS_ADAPTER_POWER_45W_MAX_ID)
				iout_max_ma = WLS_FASTCHG_CURR_45W_MAX_MA * factor;
			vote(wls_dev->fcc_votable, MAX_VOTER, true, iout_max_ma, false);
		} else {
			wls_status->fastchg_ibat_max_ma = WLS_FASTCHG_CURR_15W_MAX_MA * scale_factor;
			vote(wls_dev->fcc_votable, MAX_VOTER, true, WLS_FASTCHG_CURR_15W_MAX_MA * factor, false);
		}
		wls_status->state_sub_step = 0;
		wls_status->fastchg_started = true;
		wls_status->fastchg_level_init_temp = 0;
		wls_status->wait_cep_stable = true;
		wls_status->fastchg_retry_count = 0;
		wls_status->iout_ma_conunt = 0;
		fcc_chg->fcc_wait_timeout = jiffies;

		oplus_chg_wls_wake_bcc_update_work(wls_dev);

		for (i = 0; i < fcc_chg->max_step; i++) {
			if ((batt_temp < fcc_chg->fcc_step[i].high_threshold) &&
				(vbat_mv < fcc_chg->fcc_step[i].vol_max_mv))
				break;
		}
		wls_status->fastchg_level = (i >= fcc_chg->max_step) ? (fcc_chg->max_step - 1) : i;
		if (wls_status->fastchg_level < 0)
			wls_status->fastchg_level = 0;
		vote(wls_dev->fcc_votable, FCC_VOTER, true,
			fcc_chg->fcc_step[wls_status->fastchg_level].curr_ma, false);
		oplus_chg_wls_fast_fcc_param_init(wls_dev);
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
		if (is_nor_fv_votable_available(wls_dev))
			vote(wls_dev->nor_fv_votable, USER_VOTER, true,
				dynamic_cfg->batt_vol_max_mv + 100, false);
		delay_ms = 0;
		break;
	}

	return delay_ms;

err:
	wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
	wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
	return 0;
}

static int oplus_chg_wls_rx_handle_state_fast(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int wait_time_ms = 4000;
	int rc;
	int factor = vout_factor(wls_dev);

	oplus_chg_wls_check_quiet_mode(wls_dev);

	if ((wls_status->iout_ma > 500 * factor) && (wls_dev->wls_nor->icl_set_ma != 0)) {
		vote(wls_dev->nor_input_disable_votable, USER_VOTER, true, 0, false);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 0, false);
	}

	if (wls_dev->factory_mode&& wls_dev->support_get_tx_pwr) {
		(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_TX_PWR, 0xff, 0);
		wait_time_ms = 3000;
	}

	if (wls_dev->force_type == OPLUS_CHG_WLS_FORCE_TYPE_AUTO)
		return wait_time_ms;

	if (wls_status->switch_quiet_mode) {
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_QUIET;
		return 0;
	}

	if (!wls_dev->batt_charge_enable) {
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
		return 0;
	}

	if (is_client_vote_enabled(wls_dev->fastchg_disable_votable, RX_FAST_ERR_VOTER)) {
		chg_info("exit wls fast charge by fast_chg err\n");
		wls_status->fastchg_err_count++;
		wls_status->fastchg_err_timer = jiffies + (unsigned long)(300 * HZ); /*5 min*/
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		return 0;
	}

	if (is_client_vote_enabled(wls_dev->fastchg_disable_votable, CHG_FULL_VOTER) ||
	    is_client_vote_enabled(wls_dev->fastchg_disable_votable, CHG_LIMIT_CHG_VOTER)) {
		chg_info("exit wls fast charge by eco design\n");
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		return 0;
	}

	if (!wls_status->verity_started &&
	    (wls_status->fastchg_target_curr_ma - wls_status->iout_ma <
	     WLS_CURR_ERR_MIN_MA * factor)) {
		schedule_delayed_work(&wls_dev->wls_verity_work, msecs_to_jiffies(500));
	}

	rc = oplus_chg_wls_fast_temp_check(wls_dev);
	if (rc < 0) {
		chg_info("exit wls fast charge, exit_code=%d\n", rc);
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		return 0;
	}

	if (wls_status->fast_cep_check < 0) {
		chg_info("exit wls fast charge by cep check\n");
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		return 0;
	}

	rc = oplus_chg_wls_fast_ibat_check(wls_dev);
	if (rc < 0) {
		chg_info("exit wls fast charge by ibat check\n");
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		return 0;
	}

	rc = oplus_chg_wls_fast_iout_check(wls_dev);
	if (rc < 0) {
		chg_info("exit wls fast charge by iout check\n");
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		return 0;
	}

	oplus_chg_wls_fast_skin_temp_check(wls_dev);
	rc = oplus_chg_wls_fast_cool_down_check(wls_dev);
	if (rc < 0) {
		chg_info("exit wls fast charge by cool down\n");
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		return 0;
	}

	if (wls_status->ffc_check) {
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_FFC;
		wls_status->ffc_check = false;
		return 0;
	}
	oplus_chg_wls_exchange_batt_mesg(wls_dev);
	wls_status->track_record.is_fastchg = true;

	return wait_time_ms;
}

static int oplus_chg_wls_rx_exit_state_fast(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;

	vote(wls_dev->fcc_votable, EXIT_VOTER, true, WLS_FASTCHG_CURR_EXIT_MA, false);
	while (wls_status->iout_ma > (WLS_FASTCHG_CURR_EXIT_MA + WLS_FASTCHG_CURR_ERR_MA)) {
		if (!wls_status->rx_online) {
			chg_info("wireless charge is offline\n");
			return 0;
		}
		msleep(500);
	}
	wls_status->fastchg_started = false;
	if (is_nor_fv_votable_available(wls_dev))
		vote(wls_dev->nor_fv_votable, USER_VOTER, false, 0, false);
	oplus_chg_wls_cancel_bcc_update_work(wls_dev);

	/*if (is_comm_ocm_available(wls_dev))
		oplus_chg_comm_update_config(wls_dev->comm_ocm);*/

	oplus_chg_wls_config(wls_dev);

	(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
		wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
		FOD_DISABLE_MODE, wls_dev->magcvr_status);

	switch (wls_status->target_rx_state) {
	case OPLUS_CHG_WLS_RX_STATE_DONE:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 300, false);
		vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
		msleep(100);
		(void)oplus_chg_wls_fast_set_enable(wls_dev->wls_fast->fast_ic, false);
		(void)oplus_chg_wls_rx_set_vout_step(wls_dev->wls_rx,
			dynamic_cfg->fastchg_init_vout_mv, 1000, -1);
		vote(wls_dev->fcc_votable, EXIT_VOTER, false, 0, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_QUIET:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_QUIET;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 300, false);
		vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
		msleep(100);
		(void)oplus_chg_wls_fast_set_enable(wls_dev->wls_fast->fast_ic, false);
		(void)oplus_chg_wls_rx_set_vout_step(wls_dev->wls_rx,
			dynamic_cfg->fastchg_init_vout_mv, 1000, -1);
		vote(wls_dev->fcc_votable, EXIT_VOTER, false, 0, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_STOP:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 300, false);
		vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
		msleep(100);
		(void)oplus_chg_wls_fast_set_enable(wls_dev->wls_fast->fast_ic, false);
		(void)oplus_chg_wls_rx_set_vout_step(wls_dev->wls_rx,
			dynamic_cfg->fastchg_init_vout_mv, 1000, -1);
		vote(wls_dev->fcc_votable, EXIT_VOTER, false, 0, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_DEBUG:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_DEBUG;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 300, false);
		vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
		msleep(100);
		(void)oplus_chg_wls_fast_set_enable(wls_dev->wls_fast->fast_ic, false);
		(void)oplus_chg_wls_rx_set_vout_step(wls_dev->wls_rx,
			dynamic_cfg->fastchg_init_vout_mv, 1000, -1);
		vote(wls_dev->fcc_votable, EXIT_VOTER, false, 0, false);
		break;
	case OPLUS_CHG_WLS_RX_STATE_FFC:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_FFC;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 300, false);
		vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
		msleep(100);
		(void)oplus_chg_wls_fast_set_enable(wls_dev->wls_fast->fast_ic, false);
		(void)oplus_chg_wls_rx_set_vout_step(wls_dev->wls_rx,
			dynamic_cfg->fastchg_init_vout_mv, 1000, -1);
		vote(wls_dev->fcc_votable, EXIT_VOTER, false, 0, false);
		break;
	default:
		chg_err("unsupported target status(=%s)\n",
			oplus_chg_wls_rx_state_text[wls_status->target_rx_state]);
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		break;
	}

	if (!wls_status->switch_quiet_mode) {
		wls_status->quiet_mode_init = false;
		wls_status->fastchg_exit_fan_speed_set = true;
		oplus_chg_wls_check_quiet_mode(wls_dev);
	}
	return 0;
}

static int oplus_chg_wls_rx_enter_state_ffc(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	static unsigned long stop_time;
	int wait_time_ms = 0;
	int rc;

	switch(wls_status->state_sub_step) {
	case 0:
		vote(wls_dev->nor_out_disable_votable, FFC_VOTER, true, 1, false);
		stop_time = jiffies + msecs_to_jiffies(30000);
		wait_time_ms = (int)jiffies_to_msecs(stop_time - jiffies);
		wls_status->fod_parm_for_fastchg = false;

		if (wls_dev->static_config.fastchg_12v_fod_enable)
			(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
				oplus_chg_wls_get_fod_parm_12v(wls_dev, wls_status->adapter_id),
				wls_dev->static_config.fod_parm_len,
				FOD_FAST_MODE, wls_dev->magcvr_status);
		else
			(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
				wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
				FOD_DISABLE_MODE, wls_dev->magcvr_status);

		oplus_chg_wls_get_verity_data(wls_dev);
		wls_status->state_sub_step = 1;
		break;
	case 1:
		if (!time_is_before_jiffies(stop_time)) {
			wait_time_ms = (int)jiffies_to_msecs(stop_time - jiffies);
			break;
		}
		vote(wls_dev->nor_out_disable_votable, FFC_VOTER, false, 0, false);
		if (wls_dev->comm_topic) {
			rc = oplus_comm_switch_ffc(wls_dev->comm_topic);
			if (rc < 0) {
				chg_err("can't switch to ffc charge, rc=%d\n", rc);
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
			} else {
				vote(wls_dev->nor_icl_votable, FFC_VOTER, true, 300, false);
				vote(wls_dev->nor_icl_votable, USER_VOTER, false, 0, false);
				vote(wls_dev->nor_fcc_votable, USER_VOTER, false, 0, false);
			}
		} else {
			chg_err("comm_topic is null, can't switch to ffc charge\n");
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		}
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_FFC;
		wls_status->state_sub_step = 0;
		break;
	default:
		wls_status->state_sub_step = 0;
		break;
	}

	return wait_time_ms;
}

static int oplus_chg_wls_rx_handle_state_ffc(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int iout_ma, vout_mv, vrect_mv;
	/*int rc;*/

	oplus_chg_wls_check_quiet_mode(wls_dev);

	if (wls_dev->batt_charge_enable) {
		if (!!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
		}
	} else {
		if (!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, true, WLS_CURR_STOP_CHG_MA, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, true, 0, false);
		}
	}

	/*if (is_comm_ocm_available(wls_dev)) {
		rc = oplus_chg_comm_check_ffc(wls_dev->comm_ocm);
		if (rc < 0) {
			chg_err("ffc check error, exit ffc charge, rc=%d\n", rc);
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
			return 0;
		} else if (rc > 0) {
			chg_info("ffc done\n");
			wls_status->ffc_done = true;
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
			return 0;
		}
	} else {
		chg_err("comm mod not found, exit ffc charge\n");
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		return 0;
	}*/
	if (oplus_chg_wls_get_ffc_status(wls_dev) == FFC_DEFAULT) {
		chg_info("ffc charging, exit\n");
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 300, false);
		vote(wls_dev->nor_icl_votable, FFC_VOTER, false, 0, false);
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		return 0;
	}

	iout_ma = wls_status->iout_ma;
	vout_mv = wls_status->vout_mv;
	vrect_mv = wls_status->vrect_mv;
	chg_info("wkcs: iout=%d, vout=%d, vrect=%d\n", iout_ma, vout_mv, vrect_mv);

	if (!wls_status->verity_started)
		schedule_delayed_work(&wls_dev->wls_verity_work, msecs_to_jiffies(500));

	return 4000;
}

static int oplus_chg_wls_rx_exit_state_ffc(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	switch (wls_status->target_rx_state) {
	case OPLUS_CHG_WLS_RX_STATE_DONE:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		break;
	case OPLUS_CHG_WLS_RX_STATE_QUIET:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_QUIET;
		break;
	case OPLUS_CHG_WLS_RX_STATE_STOP:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
		break;
	case OPLUS_CHG_WLS_RX_STATE_DEBUG:
		break;
	default:
		chg_err("unsupported target status(=%s)\n", oplus_chg_wls_rx_state_text[wls_status->target_rx_state]);
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		break;
	}

	return 0;
}

static int oplus_chg_wls_rx_enter_state_done(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int index = 0;
	int rc = 0;
	int bridge_mode = WLS_HALF_BRIDGE_MODE;

	switch (wls_status->wls_type) {
	case OPLUS_CHG_WLS_BPP:
		index = OPLUS_WLS_CHG_MODE_BPP;
		break;
	case OPLUS_CHG_WLS_EPP:
		index = OPLUS_WLS_CHG_MODE_EPP;
		break;
	case OPLUS_CHG_WLS_EPP_PLUS:
		index = OPLUS_WLS_CHG_MODE_EPP_PLUS;
		break;
	case OPLUS_CHG_WLS_VOOC:
		index = OPLUS_WLS_CHG_MODE_AIRVOOC;
		break;
	case OPLUS_CHG_WLS_SVOOC:
	case OPLUS_CHG_WLS_PD_65W:
		index = OPLUS_WLS_CHG_MODE_AIRSVOOC;
		break;
	default:
		index = OPLUS_WLS_CHG_MODE_BPP;
		break;
	}

	switch (wls_status->state_sub_step) {
	case 0:
		oplus_chg_wls_get_verity_data(wls_dev);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 100, true);
		vote(wls_dev->nor_input_disable_votable, USER_VOTER, true, 0, false);
		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx,
			dynamic_cfg->fastchg_init_vout_mv, 0);
		wls_status->state_sub_step = 1;
		fallthrough;
	case 1:
		if (wls_status->charge_type != WLS_CHARGE_TYPE_FAST) {
			rc = oplus_chg_wls_cmd_into_fastchage(wls_dev, 0xff, 0);
			if (rc < 0) {
				chg_err("send WLS_CMD_INTO_FASTCHAGE err, rc=%d\n", rc);
				return 100;
			}
			wls_status->fod_parm_for_fastchg = false;
			return 1000;
		} else {
			if (dynamic_cfg->full_bridge_vout_mv > 0) {
				(void)oplus_chg_wls_rx_get_bridge_mode(wls_dev->wls_rx->rx_ic, &bridge_mode);
				if (bridge_mode != WLS_FULL_BRIDGE_MODE) {
					(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->full_bridge_vout_mv, 0);
					return 500;
				}
				(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->fastchg_init_vout_mv, -1);
			}
			if (wls_status->adapter_id == WLS_ADAPTER_THIRD_PARTY) {
				rc = oplus_chg_wls_get_third_adapter_ext_cmd_p_id(wls_dev);
				if (rc < 0) {
					if (!wls_status->rx_online)
						return 0;
					chg_err("get product id fail\n");
					wls_status->online_keep = true;
					vote(wls_dev->rx_disable_votable, VERITY_VOTER, true, 1, false);
					schedule_delayed_work(&wls_dev->rx_verity_restore_work, msecs_to_jiffies(500));
					return 100;
				}
			}

			if (wls_dev->static_config.fastchg_12v_fod_enable)
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					oplus_chg_wls_get_fod_parm_12v(wls_dev, wls_status->adapter_id),
					wls_dev->static_config.fod_parm_len,
					FOD_FAST_MODE, wls_dev->magcvr_status);
			else
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
					FOD_DISABLE_MODE, wls_dev->magcvr_status);
		}
		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->fastchg_init_vout_mv, -1);
		wls_status->state_sub_step = 2;
		break;
	case 2:
		if (wls_status->vout_mv > dynamic_cfg->vooc_vol_mv - VOUT_OK_OFFSET) {
			vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 3600, false);
			vote(wls_dev->nor_icl_votable, USER_VOTER, true, wls_dev->icl_max_ma[index], true);
			oplus_chg_wls_set_non_ffc_current(wls_dev);
			vote(wls_dev->nor_input_disable_votable, USER_VOTER, false, 0, false);
			oplus_chg_wls_nor_set_vindpm(wls_dev->wls_nor->nor_ic, WLS_VINDPM_AIRSVOOC);
			rc = oplus_chg_wls_choose_fastchg_curve(wls_dev);
			if (rc < 0)
				chg_err("choose fastchg curve failed, rc = %d\n", rc);
			wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
			wls_status->state_sub_step = 0;
		} else {
			return 100;
		}
		break;
	default:
		wls_status->state_sub_step = 0;
		break;
	}

	return 0;
}

static int oplus_chg_wls_rx_handle_state_done(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int wait_time_ms = 4000;
	int iout_ma, vout_mv, vrect_mv;
	int rc;

	wls_status->fastchg_exit_fan_speed_set = true;
	oplus_chg_wls_check_quiet_mode(wls_dev);

	if (wls_dev->batt_charge_enable) {
		if (!!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
		}
	} else {
		if (!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, true, WLS_CURR_STOP_CHG_MA, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, true, 0, false);
		}
	}

	rc = oplus_chg_wls_fastchg_restart_check(wls_dev);
	if ((rc >= 0) && (!wls_status->fastchg_disable))
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;

	(void)oplus_chg_wls_nor_skin_check(wls_dev);
	oplus_chg_wls_check_term_charge(wls_dev);
	oplus_chg_wls_set_non_ffc_current(wls_dev);
	oplus_chg_wls_exchange_batt_mesg(wls_dev);

	iout_ma = wls_status->iout_ma;
	vout_mv = wls_status->vout_mv;
	vrect_mv = wls_status->vrect_mv;
	chg_info("wkcs: iout=%d, vout=%d, vrect=%d\n", iout_ma, vout_mv, vrect_mv);

	if (!wls_status->verity_started)
		schedule_delayed_work(&wls_dev->wls_verity_work, msecs_to_jiffies(500));

	return wait_time_ms;
}

static int oplus_chg_wls_rx_exit_state_done(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	switch (wls_status->target_rx_state) {
	case OPLUS_CHG_WLS_RX_STATE_FAST:
		if (!!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
		}
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, true);
		(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
			wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
			FOD_DISABLE_MODE, wls_dev->magcvr_status);
		break;
	default:
		chg_err("unsupported target status(=%s)\n", oplus_chg_wls_rx_state_text[wls_status->target_rx_state]);
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		break;
	}

	return 0;
}

static int oplus_chg_wls_rx_enter_state_quiet(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int icl_vooc_max_ma;
	int fcc_vooc_max_ma;
	int rc = 0;

	switch (wls_status->state_sub_step) {
	case 0:
		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->fastchg_init_vout_mv, 0);
		wls_status->state_sub_step = 1;
		fallthrough;
	case 1:
		if (wls_status->charge_type != WLS_CHARGE_TYPE_FAST) {
			rc = oplus_chg_wls_cmd_into_fastchage(wls_dev, 0xff, 0);

			if (rc < 0) {
				chg_err("send WLS_CMD_INTO_FASTCHAGE err, rc=%d\n", rc);
				return 100;
			}
			wls_status->fod_parm_for_fastchg = false;
			return 1000;
		} else {
			if (wls_status->adapter_id == WLS_ADAPTER_THIRD_PARTY) {
				rc = oplus_chg_wls_get_third_adapter_ext_cmd_p_id(wls_dev);
				if (rc < 0) {
					if (!wls_status->rx_online)
						return 0;
					chg_err("get product id fail\n");
					wls_status->online_keep = true;
					vote(wls_dev->rx_disable_votable, VERITY_VOTER, true, 1, false);
					schedule_delayed_work(&wls_dev->rx_verity_restore_work, msecs_to_jiffies(500));
					return 100;
				}
			}

			if (wls_dev->static_config.fastchg_12v_fod_enable)
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					oplus_chg_wls_get_fod_parm_12v(wls_dev, wls_status->adapter_id),
					wls_dev->static_config.fod_parm_len,
					FOD_FAST_MODE, wls_dev->magcvr_status);
			else
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
					FOD_DISABLE_MODE, wls_dev->magcvr_status);
		}

		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->fastchg_init_vout_mv, -1);
		/*set normal temp AIRVOOC current to quiet mode, Not more than 12W*/
		icl_vooc_max_ma = wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_AIRVOOC];
		icl_vooc_max_ma = min(icl_vooc_max_ma, 12000 * 1000 / dynamic_cfg->fastchg_init_vout_mv);
		fcc_vooc_max_ma = 3600;
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, fcc_vooc_max_ma, false);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, icl_vooc_max_ma, true);
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_QUIET;
		wls_status->state_sub_step = 0;
		break;
	default:
		wls_status->state_sub_step = 0;
		break;
	}

	return 0;
}

static int oplus_chg_wls_rx_handle_state_quiet(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int icl_ma, nor_input_curr_ma;
	int rc;

	oplus_chg_wls_check_quiet_mode(wls_dev);

	if (!wls_status->switch_quiet_mode || wls_status->chg_done_quiet_mode) {
		if (wls_status->quiet_mode && !wls_status->chg_done_quiet_mode) {
			goto out;
		} else {
			if (wls_dev->batt_charge_enable) {
				if (!wls_status->chg_done_quiet_mode)
					wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
				else
					wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
			} else {
				wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
			}
			return 0;
		}
	}

	if (wls_dev->batt_charge_enable) {
		if (!!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
		}
	} else {
		if (!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, true, WLS_CURR_STOP_CHG_MA, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, true, 0, false);
		}
	}

out:
	(void)oplus_chg_wls_nor_skin_check(wls_dev);
	oplus_chg_wls_check_term_charge(wls_dev);
	oplus_chg_wls_choose_bpp_epp_curve(wls_dev);
	oplus_chg_wls_set_non_ffc_current(wls_dev);
	oplus_chg_wls_exchange_batt_mesg(wls_dev);

	if ((wls_status->adapter_type != WLS_ADAPTER_TYPE_UNKNOWN) &&
	     !wls_status->verity_started) {
		icl_ma = get_effective_result(wls_dev->nor_icl_votable);
		rc = oplus_chg_wls_nor_get_input_curr(wls_dev->wls_nor->nor_ic, &nor_input_curr_ma);
		if (rc < 0)
			nor_input_curr_ma = wls_status->iout_ma;
		if (icl_ma - nor_input_curr_ma < 300)
			schedule_delayed_work(&wls_dev->wls_verity_work, msecs_to_jiffies(500));
	}

	return 4000;
}

static int oplus_chg_wls_rx_exit_state_quiet(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	switch (wls_status->target_rx_state) {
	case OPLUS_CHG_WLS_RX_STATE_FAST:
		if (!!get_client_vote(wls_dev->nor_out_disable_votable, STOP_VOTER)) {
			vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, true);
			vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
		}
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, true);
		(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
			wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
			FOD_DISABLE_MODE, wls_dev->magcvr_status);
		break;
	case OPLUS_CHG_WLS_RX_STATE_STOP:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
		break;
	case OPLUS_CHG_WLS_RX_STATE_DONE:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_DONE;
		break;
	default:
		chg_err("unsupported target status(=%s)\n", oplus_chg_wls_rx_state_text[wls_status->target_rx_state]);
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		break;
	}

	return 0;
}

static int oplus_chg_wls_rx_enter_state_stop(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int rc = 0;

	switch (wls_status->state_sub_step) {
	case 0:
		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx,
			dynamic_cfg->fastchg_init_vout_mv, 0);
		wls_status->state_sub_step = 1;
		fallthrough;
	case 1:
		if (wls_status->charge_type != WLS_CHARGE_TYPE_FAST) {
			rc = oplus_chg_wls_cmd_into_fastchage(wls_dev, 0xff, 0);

			if (rc < 0) {
				chg_err("send WLS_CMD_INTO_FASTCHAGE err, rc=%d\n", rc);
				return 100;
			}
			wls_status->fod_parm_for_fastchg = false;
			return 1000;
		} else {
			if (wls_dev->static_config.fastchg_12v_fod_enable)
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					oplus_chg_wls_get_fod_parm_12v(wls_dev, wls_status->adapter_id),
					wls_dev->static_config.fod_parm_len,
					FOD_FAST_MODE, wls_dev->magcvr_status);
			else
				(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
					wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
					FOD_DISABLE_MODE, wls_dev->magcvr_status);
		}

		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->fastchg_init_vout_mv, -1);
		vote(wls_dev->nor_icl_votable, STOP_VOTER, true, 300, true);
		vote(wls_dev->nor_out_disable_votable, STOP_VOTER, true, 1, false);
		wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_STOP;
		wls_status->state_sub_step = 0;
		WARN_ON(wls_dev->batt_charge_enable);
		break;
	default:
		wls_status->state_sub_step = 0;
		break;
	}

	return 0;
}

static int oplus_chg_wls_rx_handle_state_stop(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int wait_time_ms = 4000;

	if (wls_dev->batt_charge_enable) {
		vote(wls_dev->nor_icl_votable, STOP_VOTER, false, 0, true);
		vote(wls_dev->nor_out_disable_votable, STOP_VOTER, false, 0, false);
		if (wls_status->switch_quiet_mode)
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_QUIET;
		else
			wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
		return 0;
	}

	oplus_chg_wls_check_quiet_mode(wls_dev);

	(void)oplus_chg_wls_nor_skin_check(wls_dev);
	oplus_chg_wls_check_term_charge(wls_dev);
	oplus_chg_wls_set_non_ffc_current(wls_dev);
	return wait_time_ms;
}

static int oplus_chg_wls_rx_exit_state_stop(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	switch (wls_status->target_rx_state) {
	case OPLUS_CHG_WLS_RX_STATE_FAST:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_FAST;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, true);
		(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
			wls_dev->static_config.disable_fod_parm, wls_dev->static_config.fod_parm_len,
			FOD_DISABLE_MODE, wls_dev->magcvr_status);
		break;
	case OPLUS_CHG_WLS_RX_STATE_QUIET:
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_QUIET;
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, true);
		break;
	default:
		chg_err("unsupported target status(=%s)\n", oplus_chg_wls_rx_state_text[wls_status->target_rx_state]);
		wls_status->target_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		wls_status->next_rx_state = OPLUS_CHG_WLS_RX_STATE_ERROR;
		break;
	}

	return 0;
}

static int oplus_chg_wls_rx_enter_state_debug(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int scale_factor;
	int batt_num;
	int rc;
	int factor = vout_factor(wls_dev);

	switch (wls_dev->force_type) {
	case OPLUS_CHG_WLS_FORCE_TYPE_BPP:
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 1500, false);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, 5000, 0);
		msleep(500);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 1000, false);
		if (wls_dev->factory_mode)
			(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_TX_PWR, 0xff, -1);
		break;
	case OPLUS_CHG_WLS_FORCE_TYPE_EPP:
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 1500, false);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->epp_vol_mv, 0);
		(void)oplus_chg_wls_cmd_into_fastchage(wls_dev, 0xff, -1);

		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->epp_vol_mv, -1);
		msleep(500);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 850, false);
		if (wls_dev->factory_mode)
			(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_TX_PWR, 0xff, -1);
		break;
	case OPLUS_CHG_WLS_FORCE_TYPE_EPP_PLUS:
		vote(wls_dev->nor_fcc_votable, USER_VOTER, true, 2000, false);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->epp_plus_vol_mv, 0);
		(void)oplus_chg_wls_cmd_into_fastchage(wls_dev, 0xff, -1);

		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->epp_plus_vol_mv, -1);
		msleep(500);
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 1250, false);
		if (wls_dev->factory_mode)
			(void)oplus_chg_wls_send_msg(wls_dev, WLS_CMD_GET_TX_PWR, 0xff, -1);
		break;
	case OPLUS_CHG_WLS_FORCE_TYPE_FAST:
		vote(wls_dev->nor_icl_votable, USER_VOTER, true, 500, false);
		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->fastchg_init_vout_mv, 0);
		(void)oplus_chg_wls_cmd_into_fastchage(wls_dev, 0xff, -1);

		(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, dynamic_cfg->fastchg_init_vout_mv, -1);
		(void)oplus_chg_wls_rx_set_vout_step(wls_dev->wls_rx, 15000, 1000, -1);
		rc = oplus_chg_wls_get_batt_num(wls_dev, &batt_num);
		if (rc == 0) {
			scale_factor = dynamic_cfg->svooc_vol_mv / 5000 / batt_num;
		} else {
			scale_factor = 1;
		}
		wls_status->fastchg_ibat_max_ma = dynamic_cfg->fastchg_curr_max_ma * scale_factor;
		vote(wls_dev->fcc_votable, MAX_VOTER, true, WLS_FASTCHG_CURR_45W_MAX_MA * factor, false);
		break;
	default:
		break;
	}

	wls_status->current_rx_state = OPLUS_CHG_WLS_RX_STATE_DEBUG;

	return 0;
}

static int oplus_chg_wls_rx_handle_state_debug(struct oplus_chg_wls *wls_dev)
{
	/*struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc;*/

	return 4000;
}

static int oplus_chg_wls_rx_exit_state_debug(struct oplus_chg_wls *wls_dev)
{
	/*struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc;*/

	return 0;
}

static int oplus_chg_wls_rx_enter_state_ftm(struct oplus_chg_wls *wls_dev)
{
	/*struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc;*/

	return 0;
}

static int oplus_chg_wls_rx_handle_state_ftm(struct oplus_chg_wls *wls_dev)
{
	/*struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc;*/

	return 4000;
}

static int oplus_chg_wls_rx_exit_state_ftm(struct oplus_chg_wls *wls_dev)
{
	/*struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc;*/

	return 0;
}

static int oplus_chg_wls_rx_enter_state_error(struct oplus_chg_wls *wls_dev)
{
	/*struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc;*/

	return 0;
}

static int oplus_chg_wls_rx_handle_state_error(struct oplus_chg_wls *wls_dev)
{
	/*struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc;*/

	return 4000;
}

static int oplus_chg_wls_rx_exit_state_error(struct oplus_chg_wls *wls_dev)
{
	/*struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc;*/

	return 0;
}

struct oplus_chg_wls_state_handler oplus_chg_wls_rx_state_handler[] = {
	[OPLUS_CHG_WLS_RX_STATE_DEFAULT] = {
		NULL,
		oplus_chg_wls_rx_handle_state_default,
		oplus_chg_wls_rx_exit_state_default
	},
	[OPLUS_CHG_WLS_RX_STATE_BPP] = {
		oplus_chg_wls_rx_enter_state_bpp,
		oplus_chg_wls_rx_handle_state_bpp,
		oplus_chg_wls_rx_exit_state_bpp
	},
	[OPLUS_CHG_WLS_RX_STATE_EPP] = {
		oplus_chg_wls_rx_enter_state_epp,
		oplus_chg_wls_rx_handle_state_epp,
		oplus_chg_wls_rx_exit_state_epp
	},
	[OPLUS_CHG_WLS_RX_STATE_EPP_PLUS] = {
		oplus_chg_wls_rx_enter_state_epp_plus,
		oplus_chg_wls_rx_handle_state_epp_plus,
		oplus_chg_wls_rx_exit_state_epp_plus,
	},
	[OPLUS_CHG_WLS_RX_STATE_FAST] = {
		oplus_chg_wls_rx_enter_state_fast,
		oplus_chg_wls_rx_handle_state_fast,
		oplus_chg_wls_rx_exit_state_fast,
	},
	[OPLUS_CHG_WLS_RX_STATE_FFC] = {
		oplus_chg_wls_rx_enter_state_ffc,
		oplus_chg_wls_rx_handle_state_ffc,
		oplus_chg_wls_rx_exit_state_ffc,
	},
	[OPLUS_CHG_WLS_RX_STATE_DONE] = {
		oplus_chg_wls_rx_enter_state_done,
		oplus_chg_wls_rx_handle_state_done,
		oplus_chg_wls_rx_exit_state_done,
	},
	[OPLUS_CHG_WLS_RX_STATE_QUIET] = {
		oplus_chg_wls_rx_enter_state_quiet,
		oplus_chg_wls_rx_handle_state_quiet,
		oplus_chg_wls_rx_exit_state_quiet,
	},
	[OPLUS_CHG_WLS_RX_STATE_STOP] = {
		oplus_chg_wls_rx_enter_state_stop,
		oplus_chg_wls_rx_handle_state_stop,
		oplus_chg_wls_rx_exit_state_stop,
	},
	[OPLUS_CHG_WLS_RX_STATE_DEBUG] = {
		oplus_chg_wls_rx_enter_state_debug,
		oplus_chg_wls_rx_handle_state_debug,
		oplus_chg_wls_rx_exit_state_debug,
	},
	[OPLUS_CHG_WLS_RX_STATE_FTM] = {
		oplus_chg_wls_rx_enter_state_ftm,
		oplus_chg_wls_rx_handle_state_ftm,
		oplus_chg_wls_rx_exit_state_ftm,
	},
	[OPLUS_CHG_WLS_RX_STATE_ERROR] = {
		oplus_chg_wls_rx_enter_state_error,
		oplus_chg_wls_rx_handle_state_error,
		oplus_chg_wls_rx_exit_state_error,
	},
};

static void oplus_chg_wls_rx_sm(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_rx_sm_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int delay_ms = 4000;

	if (!wls_status->rx_online) {
		chg_info("wireless charge is offline\n");
		return;
	}

	chg_info("curr_state=%s, next_state=%s, target_state=%s\n",
		oplus_chg_wls_rx_state_text[wls_status->current_rx_state],
		oplus_chg_wls_rx_state_text[wls_status->next_rx_state],
		oplus_chg_wls_rx_state_text[wls_status->target_rx_state]);

	if (wls_status->current_rx_state != wls_status->target_rx_state) {
		if (wls_status->current_rx_state != wls_status->next_rx_state) {
			if (oplus_chg_wls_rx_state_handler[wls_status->next_rx_state].enter_state != NULL) {
				delay_ms = oplus_chg_wls_rx_state_handler[wls_status->next_rx_state].enter_state(wls_dev);
			} else {
				delay_ms = 0;
			}
		} else {
			if (oplus_chg_wls_rx_state_handler[wls_status->current_rx_state].exit_state != NULL) {
				delay_ms = oplus_chg_wls_rx_state_handler[wls_status->current_rx_state].exit_state(wls_dev);
			} else {
				delay_ms = 0;
			}
		}
	} else {
		if (oplus_chg_wls_rx_state_handler[wls_status->current_rx_state].handle_state != NULL) {
			delay_ms = oplus_chg_wls_rx_state_handler[wls_status->current_rx_state].handle_state(wls_dev);
		}
	}

	queue_delayed_work(wls_dev->wls_wq, &wls_dev->wls_rx_sm_work, msecs_to_jiffies(delay_ms));
}

static void oplus_chg_wls_trx_disconnect_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_trx_disconnect_work);
	static int retry_num;
	int rc;

	mutex_lock(&wls_dev->trx_enable_lock);
	rc = oplus_chg_wls_set_trx_enable(wls_dev, false);
	mutex_unlock(&wls_dev->trx_enable_lock);
	if (rc < 0) {
		retry_num++;
		chg_err("can't disable trx, retry_num=%d, rc=%d\n", retry_num, rc);
		if (retry_num < 5)
			schedule_delayed_work(&wls_dev->wls_trx_disconnect_work, msecs_to_jiffies(100));
		else
			retry_num = 0;
	}
	retry_num = 0;
}

static void oplus_chg_wls_clear_trx_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_clear_trx_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (!wls_status->rx_present && !wls_status->rx_online) {
		wls_status->trx_close_delay = false;
		if (is_batt_psy_available(wls_dev))
			power_supply_changed(wls_dev->batt_psy);
	}
}

static void oplus_chg_wls_trx_sm(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_trx_sm_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	u8 trx_status;
	u32 trx_err;
	static int err_count;
	static bool pre_trx_online = false;
	int delay_ms = 5000;
	int rc;
	struct wls_track_record *track_record = &wls_dev->wls_status.track_record;
	union mms_msg_data data = { 0 };

	rc = oplus_chg_wls_rx_get_trx_err(wls_dev->wls_rx->rx_ic, &trx_err);
	if (rc < 0) {
		chg_err("can't get trx err code, rc=%d\n", rc);
		goto out;
	}
	chg_info("wkcs: trx_err=0x%02x\n", trx_err);
	wls_status->trx_err = trx_err;

	if (trx_err & WLS_TRX_ERR_RXAC) {
		chg_info("trx err: RXAC\n");
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_OFF;
		wls_status->trx_rxac = true;
		wls_status->trx_close_delay = true;
	}
	if (trx_err & WLS_TRX_ERR_OCP) {
		chg_info("trx err: OCP\n");
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_OFF;
		wls_status->trx_close_delay = true;
	}
	if (trx_err & WLS_TRX_ERR_OVP) {
		chg_info("trx err: OVP\n");
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_OFF;
	}
	if (trx_err & WLS_TRX_ERR_LVP) {
		chg_info("trx err: LVP\n");
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_OFF;
	}
	if (trx_err & WLS_TRX_ERR_FOD) {
		chg_info("trx err: FOD\n");
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_OFF;
	}
	if (trx_err & WLS_TRX_ERR_OTP) {
		chg_info("trx err: OTP\n");
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_OFF;
	}
	if (trx_err & WLS_TRX_ERR_CEPTIMEOUT) {
		chg_info("trx err: CEPTIMEOUT\n");
	}
	if (trx_err & WLS_TRX_ERR_RXEPT) {
		chg_info("trx err: RXEPT\n");
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_OFF;
	}
	if (trx_err & WLS_TRX_ERR_VRECTOVP) {
		chg_info("trx err: VRECTOVP\n");
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_OFF;
	}
	if (wls_status->trx_state == OPLUS_CHG_WLS_TRX_STATE_OFF)
		goto err;

	rc = oplus_chg_wls_rx_get_trx_status(wls_dev->wls_rx->rx_ic, &trx_status);
	if (rc < 0) {
		chg_err("can't get trx err code, rc=%d\n", rc);
		goto out;
	}
	chg_info("wkcs: trx_status=0x%02x\n", trx_status);
	if (wls_status->wls_type != OPLUS_CHG_WLS_TRX)
		goto out;
	if (trx_status & WLS_TRX_STATUS_READY) {
		wls_status->trx_present = true;
		wls_status->trx_online = false;
		pre_trx_online = false;
		if (is_batt_psy_available(wls_dev))
			power_supply_changed(wls_dev->batt_psy);
		rc = oplus_chg_wls_rx_set_trx_enable(wls_dev->wls_rx->rx_ic, true);
		if (rc < 0) {
			chg_err("can't enable trx, rc=%d\n", rc);
			goto out;
		}
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_READY;
		delay_ms = 200;
	} else if ((trx_status & WLS_TRX_STATUS_DIGITALPING)
			|| (trx_status & WLS_TRX_STATUS_ANALOGPING)) {
		wls_status->trx_present = true;
		wls_status->trx_online = false;
		if (is_batt_psy_available(wls_dev))
			power_supply_changed(wls_dev->batt_psy);
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_WAIT_PING;
		delay_ms = 2000;
	} else if (trx_status & WLS_TRX_STATUS_TRANSFER) {
		wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_TRANSFER;
		wls_status->trx_online = true;
		if (!wls_status->trx_usb_present_once)
			wls_status->trx_usb_present_once = oplus_chg_wls_is_usb_present(wls_dev);

		if (wls_status->trx_usb_present_once) {
			oplus_mms_get_item_data(wls_dev->wired_topic, WIRED_ITEM_CHG_TYPE, &data, false);
			track_record->wired_charge_type = data.intval != 0 ? data.intval : 0;
		}

		if (is_batt_psy_available(wls_dev))
			power_supply_changed(wls_dev->batt_psy);
		rc = oplus_chg_wls_rx_get_trx_curr(wls_dev->wls_rx->rx_ic, &wls_status->trx_curr_ma);
		if (rc < 0)
			chg_err("can't get trx curr, rc=%d\n", rc);
		rc = oplus_chg_wls_rx_get_trx_vol(wls_dev->wls_rx->rx_ic, &wls_status->trx_vol_mv);
		if (rc < 0)
			chg_err("can't get trx vol, rc=%d\n", rc);
		chg_info("trx_vol=%d, trx_curr=%d\n", wls_status->trx_vol_mv, wls_status->trx_curr_ma);
		delay_ms = 5000;
	} else if (trx_status == 0) {
		goto out;
	}

	if (pre_trx_online && !wls_status->trx_online) {
		wls_status->trx_transfer_end_time = oplus_chg_wls_get_local_time_s();
		chg_info("trx_online:%d, trx_start_time:%d, trx_end_time:%d,"
			"trx_usb_present_once:%d\n",
			wls_status->trx_online, wls_status->trx_transfer_start_time,
			wls_status->trx_transfer_end_time,
			wls_status->trx_usb_present_once);
		if (wls_status->trx_transfer_end_time - wls_status->trx_transfer_start_time >
		    WLS_TRX_INFO_UPLOAD_THD_2MINS) {
			oplus_chg_wls_track_upload_general_info(
				wls_dev, "trx_mode");
		}
		wls_status->trx_usb_present_once = false;
		track_record->wired_charge_type = 0;
	} else if (!pre_trx_online && wls_status->trx_online) {
		wls_status->trx_usb_present_once = false;
		track_record->wired_charge_type = 0;
		wls_status->trx_transfer_start_time =
			oplus_chg_wls_get_local_time_s();
		chg_info("trx_online=%d, trx_start_time=%d, trx_end_time=%d\n",
			wls_status->trx_online, wls_status->trx_transfer_start_time,
			wls_status->trx_transfer_end_time);
	}
	pre_trx_online = wls_status->trx_online;

	chg_info("trx_state=%s\n", oplus_chg_wls_trx_state_text[wls_status->trx_state]);
	err_count = 0;

schedule:
	queue_delayed_work(wls_dev->wls_wq, &wls_dev->wls_trx_sm_work, msecs_to_jiffies(delay_ms));
	return;
out:
	err_count++;
	delay_ms = 200;
	if (err_count > (2 * 60 * 5) || wls_status->wls_type != OPLUS_CHG_WLS_TRX) {
		chg_err("trx status err, exit\n");
		goto err;
	}
	goto schedule;
err:
	wls_status->trx_state = OPLUS_CHG_WLS_TRX_STATE_OFF;
	err_count = 0;
	schedule_delayed_work(&wls_dev->wls_trx_disconnect_work, 0);
}

static void oplus_chg_wls_upgrade_fw_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_upgrade_fw_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	bool usb_present;
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int boot_mode;
#endif
	static int retry_num;
	int rc;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	boot_mode = get_boot_mode();
#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (boot_mode == MSM_BOOT_MODE__FACTORY || boot_mode == MSM_BOOT_MODE__RF) {
#else
	if (boot_mode == META_BOOT || boot_mode == FACTORY_BOOT ||
	    boot_mode == ADVMETA_BOOT || boot_mode == ATE_FACTORY_BOOT) {
#endif
		chg_err("is factory mode, can't upgrade fw\n");
		vote(wls_dev->rx_disable_votable, UPGRADE_VOTER, true, 1, false);
		msleep(1000);
		vote(wls_dev->rx_disable_votable, UPGRADE_VOTER, false, 0, false);
		retry_num = 0;
		return;
	}
#endif

	usb_present = oplus_chg_wls_is_usb_present(wls_dev);
	if (usb_present && !wls_dev->support_tx_boost) {
		chg_info("usb online, wls fw upgrade pending\n");
		wls_status->upgrade_fw_pending = true;
		return;
	}
	if (wls_dev->pre_otg_enable && wls_dev->support_acdrv_no_ext_boost) {
		chg_info("otg online, wls fw upgrade pending\n");
		wls_status->upgrade_fw_pending = true;
		return;
	}

	if (wls_status->rx_present && wls_dev->force_upgrade == FORCE_UPGRADE_NONE) {
		chg_err("rx present, exit upgrade\n");
		retry_num = 0;
		return;
	}

	if (wls_dev->fw_upgrade_by_buf) {
		if (wls_dev->fw_buf == NULL || wls_dev->fw_size == 0) {
			chg_err("fw buf is NULL or fw size is 0, can't upgrade\n");
			wls_dev->fw_upgrade_by_buf = false;
			wls_dev->fw_size = 0;
			return;
		}
		vote(wls_dev->rx_disable_votable, UPGRADE_VOTER, true, 1, false);
		vote(wls_dev->wrx_en_votable, UPGRADE_VOTER, true, 1, false);
		vote(wls_dev->insert_disable_votable, UPGRADE_VOTER, true, 1, false);
		rc = oplus_chg_wls_rx_upgrade_firmware_by_buf(wls_dev->wls_rx, wls_dev->fw_buf, wls_dev->fw_size);
		vote(wls_dev->insert_disable_votable, UPGRADE_VOTER, false, 0, false);
		vote(wls_dev->wrx_en_votable, UPGRADE_VOTER, false, 0, false);
		vote(wls_dev->rx_disable_votable, UPGRADE_VOTER, false, 0, false);
		if (rc < 0)
			chg_err("upgrade error, rc=%d\n", rc);
		kfree(wls_dev->fw_buf);
		wls_dev->fw_buf = NULL;
		wls_dev->fw_upgrade_by_buf = false;
		wls_dev->fw_size = 0;
	} else {
		vote(wls_dev->rx_disable_votable, UPGRADE_VOTER, true, 1, false);
		vote(wls_dev->wrx_en_votable, UPGRADE_VOTER, true, 1, false);
		vote(wls_dev->insert_disable_votable, UPGRADE_VOTER, true, 1, false);
		rc = oplus_chg_wls_rx_upgrade_firmware_by_img(wls_dev->wls_rx);
		vote(wls_dev->insert_disable_votable, UPGRADE_VOTER, false, 0, false);
		vote(wls_dev->wrx_en_votable, UPGRADE_VOTER, false, 0, false);
		vote(wls_dev->rx_disable_votable, UPGRADE_VOTER, false, 0, false);
		if (rc < 0) {
			retry_num++;
			chg_err("upgrade error, retry_num=%d, rc=%d\n", retry_num, rc);
			goto out;
		}
		if (wls_dev->force_upgrade == FORCE_UPGRADE_RX)
			oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_UPDATE, WLS_ERR_FW_UPDATE_FORCE);
		else if (wls_dev->force_upgrade == FORCE_UPGRADE_TX)
			oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_UPDATE, WLS_ERR_FW_UPDATE_FORCE_TX);
		else
			oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_UPDATE, WLS_ERR_FW_UPDATE_BOOT);
		wls_dev->force_upgrade = FORCE_UPGRADE_NONE;
	}
	chg_info("update successed\n");

	return;

out:
	if (retry_num >= 5) {
		retry_num = 0;
		wls_dev->force_upgrade = FORCE_UPGRADE_NONE;
		oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_UPDATE, WLS_ERR_FW_UPDATE_FAIL);
		return;
	}
	schedule_delayed_work(&wls_dev->wls_upgrade_fw_work, msecs_to_jiffies(1000));
}

static void oplus_chg_wls_init_complete_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_init_complete_work);

	wls_dev->wls_init_completed = true;
}

static int oplus_chg_wls_rx_set_rx_mode_safety(struct oplus_chg_wls *wls_dev, enum oplus_chg_wls_rx_mode rx_mode)
{
	if ((rx_mode != OPLUS_CHG_WLS_RX_MODE_UNKNOWN) && (wls_dev->wls_status.fw_upgrading ||
	     wls_dev->usb_present || wls_dev->wls_status.trx_present)) {
		chg_info("should not send pwm, fw_upgrading:%d, usb_present%d, trx_present%d, pre_otg_enable%d\n",
			 wls_dev->wls_status.fw_upgrading, wls_dev->usb_present,
			 wls_dev->wls_status.trx_present, wls_dev->pre_otg_enable);
		return 0;
	}

	return oplus_chg_wls_rx_set_rx_mode(wls_dev->wls_rx->rx_ic, rx_mode);
}

static int oplus_chg_wls_entry_rx_mode(struct oplus_chg_wls *wls_dev, enum oplus_chg_wls_rx_mode rx_mode,
				       const char *client_str)
{
	if (wls_dev == NULL) {
		chg_err("wls_dev is NULL\n");
		return -ENODEV;
	}
	if (wls_dev->rx_protocol_version < WLS_RX_PROTOCOL_VERSION_30)
		return 0;

	chg_info("rx_mode:%d\n", rx_mode);

	if ((rx_mode != OPLUS_CHG_WLS_RX_MODE_UNKNOWN) && (wls_dev->wls_status.fw_upgrading ||
	     wls_dev->usb_present || wls_dev->wls_status.trx_present)) {
		chg_info("should not send pwm, fw_upgrading:%d, usb_present%d, trx_present%d, pre_otg_enable%d\n",
			 wls_dev->wls_status.fw_upgrading, wls_dev->usb_present,
			 wls_dev->wls_status.trx_present, wls_dev->pre_otg_enable);
		return 0;
	}

	if (rx_mode == OPLUS_CHG_WLS_RX_MODE_UNKNOWN) {
		cancel_delayed_work(&wls_dev->rx_mode_check_work);
		oplus_chg_wls_rx_set_rx_mode_safety(wls_dev, OPLUS_CHG_WLS_RX_MODE_UNKNOWN);
		if (wls_dev->wls_status.rx_present &&
		    (wls_dev->wls_status.wls_type == OPLUS_CHG_WLS_BPP ||
		    wls_dev->wls_status.wls_type == OPLUS_CHG_WLS_UNKNOWN)) {
			wls_dev->wls_status.online_keep = true;
			oplus_chg_wls_set_status_keep(wls_dev->wls_topic, WLS_SK_BY_FORCE_BPP);
			if (oplus_chg_wls_rx_set_silent(wls_dev->wls_rx->rx_ic) == -ENOTSUPP) {
				vote(wls_dev->rx_disable_votable, client_str, true, 1, false);
				msleep(200);
				vote(wls_dev->rx_disable_votable, client_str, false, 1, false);
			}
		}
	} else if (rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
		if (wls_dev->wls_status.rx_present &&
		    wls_dev->wls_status.wls_type != OPLUS_CHG_WLS_BPP) {
			wls_dev->wls_status.online_keep = true;
			oplus_chg_wls_set_status_keep(wls_dev->wls_topic, WLS_SK_BY_FORCE_BPP);
			if (oplus_chg_wls_rx_set_silent(wls_dev->wls_rx->rx_ic) == -ENOTSUPP) {
				vote(wls_dev->rx_disable_votable, client_str, true, 1, false);
				msleep(200);
				vote(wls_dev->rx_disable_votable, client_str, false, 1, false);
			}
			chg_info("force bpp after wls disconnect\n");
		}

		if (strcmp(client_str, WLS_CAMERA_MODE_VOTER) == 0)
			schedule_delayed_work(&wls_dev->rx_mode_check_work, msecs_to_jiffies(OPLUS_CHG_WLS_RX_MODE_CHECK_DELAY));
	}

	return 0;
}

#define DELTA_IOUT_500MA	500
#define DELTA_IOUT_300MA	300
#define DELTA_IOUT_100MA	100
#define VOUT_200MV		200
#define VOUT_100MV		100
#define VOUT_50MV		50
#define VOUT_20MV		20
static void oplus_chg_wls_data_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_data_update_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int ibat_ma, tmp_val, ibat_err, cep, vol_set_mv;
	int iout_ma, vout_mv, vrect_mv;
	static int cep_err_count;
	bool skip_cep_check = false;
	bool cep_ok = false;
	int rc;
	int factor = vout_factor(wls_dev);

	if (!wls_status->rx_online) {
		chg_err("wireless charge is not online\n");
		return;
	}

	oplus_chg_wls_update_track_info(wls_dev, NULL, false);

	rc = oplus_chg_wls_rx_get_iout(wls_dev->wls_rx->rx_ic, &iout_ma);
	if (rc < 0)
		chg_err("can't get rx iout, rc=%d\n", rc);
	rc = oplus_chg_wls_rx_get_vout(wls_dev->wls_rx->rx_ic, &vout_mv);
	if (rc < 0)
		chg_err("can't get rx vout, rc=%d\n", rc);
	rc = oplus_chg_wls_rx_get_vrect(wls_dev->wls_rx->rx_ic, &vrect_mv);
	if (rc < 0)
		chg_err("can't get rx vrect, rc=%d\n", rc);
	WRITE_ONCE(wls_status->iout_ma, iout_ma);
	WRITE_ONCE(wls_status->vout_mv, vout_mv);
	WRITE_ONCE(wls_status->vrect_mv, vrect_mv);

	if (!wls_status->fastchg_started)
		goto out;

	rc = oplus_chg_wls_get_ibat(wls_dev, &ibat_ma);
	if (rc < 0) {
		chg_err("can't get ibat, rc=%d\n", rc);
		goto out;
	}

	tmp_val = wls_status->fastchg_target_curr_ma - wls_status->iout_ma;
	/* (1) a = ibat_max_ma - ibat_ma, (2) convert into ibus: b = a / 2, (3) ibat_err = (b - err_min) / 2 */
	ibat_err = ((wls_status->fastchg_ibat_max_ma - abs(ibat_ma)) / 2 * factor - WLS_CURR_ERR_MIN_MA) / 2;
	/* Prevent the voltage from increasing too much, ibat exceeds expectations */
	if ((ibat_err > -(WLS_CURR_ERR_MIN_MA / 2)) && (ibat_err < 0) && (tmp_val > 0)) {
		/*
		 * When ibat is greater than 5800mA, the current is not
		 * allowed to continue to increase, preventing fluctuations.
		 */
		tmp_val = 0;
	} else {
		tmp_val = tmp_val > ibat_err ? ibat_err : tmp_val;
	}
	rc = oplus_chg_wls_get_cep_check_update(wls_dev->wls_rx->rx_ic, &cep);
	if (rc < 0) {
		chg_err("can't get cep, rc=%d\n", rc);
		cep_ok = false;
	} else {
		if (abs(cep) < 3)
			cep_ok = true;
		else
			cep_ok = false;
	}
	if (tmp_val < 0) {
		if (!cep_ok)
			cep_err_count++;
		else
			cep_err_count = 0;
		if (!wls_status->fastchg_curr_need_dec || cep_err_count >= WLS_CEP_ERR_MAX) {
			skip_cep_check = true;
			wls_status->fastchg_curr_need_dec = true;
			cep_err_count = 0;
		}
	} else {
		cep_err_count = 0;
		wls_status->fastchg_curr_need_dec = false;
	}
	vol_set_mv = wls_dev->wls_rx->vol_set_mv;
	if (cep_ok || skip_cep_check) {
		if (tmp_val > WLS_CURR_ERR_MIN_MA * factor || tmp_val < -WLS_CURR_ERR_MIN_MA * factor) {
			if (tmp_val > WLS_CURR_ERR_MIN_MA * factor) {
				if (tmp_val > DELTA_IOUT_500MA * factor)
					vol_set_mv += VOUT_200MV / factor;
				else if (tmp_val > DELTA_IOUT_300MA * factor)
					vol_set_mv += VOUT_100MV / factor;
				else if (tmp_val > DELTA_IOUT_100MA * factor)
					vol_set_mv += VOUT_50MV / factor;
				else
					vol_set_mv += VOUT_20MV / factor;
			} else {
				if (tmp_val < -DELTA_IOUT_500MA * factor)
					vol_set_mv -= VOUT_200MV / factor;
				else if (tmp_val < -DELTA_IOUT_300MA * factor)
					vol_set_mv -= VOUT_100MV / factor;
				else if (tmp_val < -DELTA_IOUT_100MA * factor)
					vol_set_mv -= VOUT_50MV / factor;
				else
					vol_set_mv -= VOUT_20MV / factor;
			}
			if (vol_set_mv > dynamic_cfg->svooc_vol_mv)
				vol_set_mv = dynamic_cfg->svooc_vol_mv;
			if (vol_set_mv < WLS_FASTCHG_MODE_VOL_MIN_MV / factor)
				vol_set_mv = WLS_FASTCHG_MODE_VOL_MIN_MV / factor;
			mutex_lock(&wls_dev->update_data_lock);
			(void)oplus_chg_wls_rx_set_vout(wls_dev->wls_rx, vol_set_mv, 0);
			mutex_unlock(&wls_dev->update_data_lock);
		}
	}

	chg_info("Iout: target=%d, out=%d, ibat_max=%d, ibat=%d; vout=%d; vrect=%d; cep=%d\n",
		wls_status->fastchg_target_curr_ma, wls_status->iout_ma,
		wls_status->fastchg_ibat_max_ma, ibat_ma,
		wls_status->vout_mv, wls_status->vrect_mv,
		cep);
	wls_status->last_cep = cep;
	oplus_chg_wls_fast_cep_check(wls_dev, cep);

out:
	schedule_delayed_work(&wls_dev->wls_data_update_work, msecs_to_jiffies(500));
}

#define CEP_SKEWING_VALUE	3
#define SKEWING_INTERVAL	500
static void oplus_chg_wls_skewing_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_skewing_work);
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int cep = 0;
	static int cep_ok_count = 0;
	static int cep_err_count = 0;
	int skewing_icl = 0;
	int skewing_max_step = 0;
	int pre_level = 0;
	int rc = 0;
	int i = 0;

	if (!wls_status->rx_online) {
		cep_ok_count = 0;
		cep_err_count = 0;
		return;
	}

	rc = oplus_chg_wls_get_cep(wls_dev->wls_rx->rx_ic, &cep);
	if (rc < 0) {
		chg_err("can't get cep, rc=%d\n", rc);
		goto out;
	}
	wls_status->last_cep = cep;

	switch (wls_status->wls_type) {
	case OPLUS_CHG_WLS_EPP:
		i = OPLUS_WLS_SKEWING_EPP;
		break;
	case OPLUS_CHG_WLS_EPP_PLUS:
		i = OPLUS_WLS_SKEWING_EPP_PLUS;
		break;
	case OPLUS_CHG_WLS_VOOC:
		i = OPLUS_WLS_SKEWING_AIRVOOC;
		break;
	default:
		i = OPLUS_WLS_SKEWING_MAX;
		break;
	}
	if (i >= OPLUS_WLS_SKEWING_MAX)
		goto out;

	skewing_max_step = wls_dev->skewing_step[i].max_step;
	pre_level = wls_status->skewing_level;

	if (cep < CEP_SKEWING_VALUE) {
		cep_ok_count++;
		cep_err_count = 0;
		if (cep_ok_count >= CEP_OK_MAX &&
		    dynamic_cfg->skewing_step[i][wls_status->skewing_level].fallback_step &&
		    time_after(jiffies, wls_status->cep_ok_wait_timeout)) {
			cep_ok_count = 0;
			wls_status->skewing_level =
				dynamic_cfg->skewing_step[i][wls_status->skewing_level].fallback_step - 1;
			wls_status->cep_ok_wait_timeout = jiffies + HZ;
			chg_info("skewing: recovery charging current, level[%d]\n",
				wls_status->skewing_level);
		}
	} else {
		cep_ok_count = 0;
		cep_err_count++;
		if (cep_err_count >= CEP_ERR_MAX) {
			if (wls_status->skewing_level < (skewing_max_step - 1)) {
				cep_err_count = 0;
				wls_status->skewing_level++;
				wls_status->cep_ok_wait_timeout = jiffies + HZ;
				chg_info("skewing: reduce charging current, level[%d]\n",
					wls_status->skewing_level);
			} else {
				if (dynamic_cfg->skewing_step[i][wls_status->skewing_level].switch_to_bpp) {
					chg_info("skewing: switch to bpp, level[%d]\n", wls_status->skewing_level);
					oplus_chg_wls_rx_set_rx_mode_safety(wls_dev, OPLUS_CHG_WLS_RX_MODE_BPP);
				}
			}
		}
	}

	skewing_icl = dynamic_cfg->skewing_step[i][wls_status->skewing_level].curr_ma;
	if (skewing_icl > 0 && pre_level != wls_status->skewing_level)
		vote(wls_dev->nor_icl_votable, CEP_VOTER, true, skewing_icl, true);

	chg_info("level=%d, icl=%d, cep=%d\n", wls_status->skewing_level, skewing_icl, cep);
out:
	schedule_delayed_work(&wls_dev->wls_skewing_work, msecs_to_jiffies(SKEWING_INTERVAL));
}

static void oplus_chg_wls_usb_connect_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, usb_connect_work);
	bool connected, pre_connected;

	pre_connected = oplus_chg_wls_is_usb_connected(wls_dev);
retry:
	msleep(10);
	connected = oplus_chg_wls_is_usb_connected(wls_dev);
	if (connected != pre_connected) {
		pre_connected = connected;
		goto retry;
	}
}

static void oplus_chg_wls_fod_cal_work(struct work_struct *work)
{
}

static void oplus_chg_wls_rx_restore_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, rx_restore_work);

	vote(wls_dev->rx_disable_votable, JEITA_VOTER, false, 0, false);
	vote(wls_dev->nor_icl_votable, JEITA_VOTER, true, WLS_CURR_JEITA_CHG_MA, true);
}

static void oplus_chg_wls_rx_iic_restore_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, rx_iic_restore_work);

	vote(wls_dev->rx_disable_votable, RX_IIC_VOTER, false, 0, false);
}

static void oplus_chg_wls_rx_verity_restore_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, rx_verity_restore_work);

	vote(wls_dev->rx_disable_votable, VERITY_VOTER, false, 0, false);
	schedule_delayed_work(&wls_dev->verity_state_remove_work, msecs_to_jiffies(10000));
}

#define FAST_FAULT_SIZE 80
static void oplus_chg_wls_fast_fault_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, fast_fault_check_work);
	char *fast_fault;
	int rc;

	fast_fault = kzalloc(FAST_FAULT_SIZE, GFP_KERNEL);
	if (fast_fault == NULL) {
		chg_err("<FAST_FAULT>alloc fast_fault err\n");
		goto alloc_error;
	}

	rc = oplus_chg_wls_fast_get_fault(wls_dev->wls_fast->fast_ic, fast_fault);
	if (rc < 0)
		chg_err("can't get fast fault code, rc=%d\n", rc);
	else
		chg_info("fast fault =%s \n", fast_fault);

	kfree(fast_fault);
alloc_error:
	vote(wls_dev->fastchg_disable_votable, RX_FAST_ERR_VOTER, true, 1, false);
}

static void oplus_chg_wls_rx_restart_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, rx_restart_work);
	static int retry_count;

	if (!is_rx_ic_available(wls_dev->wls_rx)) {
		if (retry_count > 5) {
			chg_err("can't found wls rx ic\n");
			retry_count = 0;
			return;
		}
		retry_count++;
		schedule_delayed_work(&wls_dev->rx_restart_work, msecs_to_jiffies(500));
	}
	if (!oplus_chg_wls_rx_is_connected(wls_dev->wls_rx->rx_ic)) {
		chg_info("wireless charging is not connected\n");
		return;
	}

	retry_count = 0;
	wls_dev->wls_status.online_keep = true;
	vote(wls_dev->rx_disable_votable, USER_VOTER, true, 1, false);
	msleep(1000);
	vote(wls_dev->rx_disable_votable, USER_VOTER, false, 0, false);
	if (READ_ONCE(wls_dev->wls_status.online_keep))
		schedule_delayed_work(&wls_dev->online_keep_remove_work, msecs_to_jiffies(4000));
}

static void oplus_chg_wls_online_keep_remove_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, online_keep_remove_work);
	union mms_msg_data pre_present = { 0 };
	union mms_msg_data cur_present = { 0 };

	oplus_mms_get_item_data(wls_dev->wls_topic, WLS_ITEM_PRESENT, &pre_present, true);
	wls_dev->wls_status.online_keep = false;
	oplus_mms_get_item_data(wls_dev->wls_topic, WLS_ITEM_PRESENT, &cur_present, true);
	if (pre_present.intval != cur_present.intval)
		schedule_work(&wls_dev->wls_present_handler_work);

	if (!wls_dev->wls_status.rx_online) {
		if (wls_dev->rx_wake_lock_on) {
			chg_info("release rx_wake_lock\n");
			__pm_relax(wls_dev->rx_wake_lock);
			wls_dev->rx_wake_lock_on = false;
		} else {
			chg_err("rx_wake_lock is already relax\n");
		}
	}
}

static void oplus_chg_wls_verity_state_remove_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, verity_state_remove_work);

	wls_dev->wls_status.verity_state_keep = false;
}

#define VOUT_ERR_CHECK_TIME	500
#define VOUT_ERR_CHECK_COUNT	5
#define VOUT_ERR_THRESHOLD_MV	3000
static void oplus_chg_wls_vout_err_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_vout_err_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int vout_mv;
	static int vout_err_count = 0;

	if (!wls_dev->support_fastchg || !wls_status->rx_online) {
		vout_err_count = 0;
		chg_info("wireless charge is offline or not support fastchg\n");
		return;
	}

	vout_mv = wls_status->vout_mv;

#ifdef WLS_QI_DEBUG
	if (wls_dbg_vout_mv != 0)
		vout_mv = wls_dbg_vout_mv;
#endif

	if (vout_mv < VOUT_ERR_THRESHOLD_MV)
		vout_err_count++;
	else
		vout_err_count = 0;
	if (vout_err_count > VOUT_ERR_CHECK_COUNT) {
		vout_err_count = 0;
		chg_err("vout is err, disable ext pwr!\n");
		oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_RX, WLS_ERR_VOUT_ABNORMAL);
		(void)oplus_chg_wls_rx_set_dcdc_enable(wls_dev->wls_rx->rx_ic, false);
		msleep(10);
		(void)oplus_chg_wls_set_ext_pwr_enable(wls_dev, false);
		return;
	}

	schedule_delayed_work(&wls_dev->wls_vout_err_work, msecs_to_jiffies(VOUT_ERR_CHECK_TIME));
}

static int oplus_chg_wls_dev_open(struct inode *inode, struct file *filp)
{
	struct oplus_chg_wls *wls_dev = container_of(filp->private_data,
		struct oplus_chg_wls, misc_dev);

	filp->private_data = wls_dev;
	chg_debug("%d,%d\n", imajor(inode), iminor(inode));
	return 0;
}

static ssize_t oplus_chg_wls_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct oplus_chg_wls *wls_dev = filp->private_data;
	struct wls_dev_cmd cmd;
	int rc = 0;

	mutex_lock(&wls_dev->read_lock);
	rc = wait_event_interruptible(wls_dev->read_wq, wls_dev->cmd_data_ok);
	mutex_unlock(&wls_dev->read_lock);
	if (rc)
		return rc;
	if (!wls_dev->cmd_data_ok)
		chg_err("wlchg false wakeup, rc=%d\n", rc);
	mutex_lock(&wls_dev->cmd_data_lock);
	wls_dev->cmd_data_ok = false;
	memcpy(&cmd, &wls_dev->cmd, sizeof(struct wls_dev_cmd));
	mutex_unlock(&wls_dev->cmd_data_lock);
	if (copy_to_user(buf, &cmd, sizeof(struct wls_dev_cmd))) {
		chg_err("failed to copy to user space\n");
		return -EFAULT;
	}

	return sizeof(struct wls_dev_cmd);
}

#define WLS_IOC_MAGIC			0xf1
#define WLS_NOTIFY_WLS_AUTH		_IOW(WLS_IOC_MAGIC, 1, struct wls_auth_result)

static long oplus_chg_wls_dev_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct oplus_chg_wls *wls_dev = filp->private_data;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	void __user *argp = (void __user *)arg;
	int rc;

	switch (cmd) {
	case WLS_NOTIFY_WLS_AUTH:
		rc = copy_from_user(&wls_status->verfity_data, argp, sizeof(struct wls_auth_result));
		if (rc) {
			chg_err("failed copy to user space\n");
			return rc;
		}
		wls_status->verity_data_ok = true;
		break;
	default:
		chg_err("bad ioctl %u\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static ssize_t oplus_chg_wls_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	return count;
}

static const struct file_operations oplus_chg_wls_dev_fops = {
	.owner			= THIS_MODULE,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	.llseek			= noop_llseek,
#else
	.llseek			= no_llseek,
#endif
	.write			= oplus_chg_wls_dev_write,
	.read			= oplus_chg_wls_dev_read,
	.open			= oplus_chg_wls_dev_open,
	.unlocked_ioctl	= oplus_chg_wls_dev_ioctl,
};

static irqreturn_t oplus_chg_wls_usb_int_handler(int irq, void *dev_id)
{
	struct oplus_chg_wls *wls_dev = dev_id;

	schedule_delayed_work(&wls_dev->usb_connect_work, 0);
	return IRQ_HANDLED;
}

#define OLD_Q_VALUE_LENGTH 2
#define NEW_Q_VALUE_LENGTH 3
static int read_q_value_from_node(struct device_node *node,
		const char *prop_str, struct wls_match_q_type *fastchg_match_q,
		int length, u32 rx_protocol_version)
{
	int i, q_value_length, index, rc;
	u8 buf[WLS_BASE_NUM_MAX * NEW_Q_VALUE_LENGTH];
	q_value_length = rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30 ? NEW_Q_VALUE_LENGTH : OLD_Q_VALUE_LENGTH;


	rc = of_property_read_u8_array(node, prop_str, (u8 *)buf, length);
	if (rc < 0)
		chg_err("%s reading failed, rc=%d\n", prop_str, rc);

	for (i = 0; i < length / q_value_length; i++) {
		index = i * q_value_length;
		fastchg_match_q[i].id = buf[index];
		fastchg_match_q[i].q_value = buf[index + 1];
		if (q_value_length >= NEW_Q_VALUE_LENGTH)
			fastchg_match_q[i].f_value = buf[index + 2];

		chg_info("%s: 0x%x 0x%x 0x%x\n", prop_str, fastchg_match_q[i].id,
			 fastchg_match_q[i].q_value, fastchg_match_q[i].f_value);
	}

	return rc;
}

static void read_third_part_qf_value_from_node(struct device_node *node,
		const char *prop_str, struct wls_third_part_qf_parm *fastchg_match_qf, int *set)
{
	int i, length, rc;

	if (!node || !prop_str || !fastchg_match_qf) {
		*set = 0;
		chg_err("Invalid parameters passed\n");
		return;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u8));
	if (rc <= 0 || rc > WLS_THIRD_PART_TX_CONFIG_NUM_MAX * sizeof(struct wls_third_part_qf_parm)) {
		chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		*set = 0;
		return;
	}
	length = rc;

	rc = of_property_read_u8_array(node, prop_str, (u8 *)fastchg_match_qf, length);
	if (rc < 0) {
		chg_err("%s reading failed, rc=%d\n", prop_str, rc);
		*set = 0;
		return;
	}

	*set = length / sizeof(struct wls_third_part_qf_parm);
	for (i = 0; i < *set; i++) {
		chg_info("%s: 0x%x 0x%x : 0x%x 0x%x\n",
			 prop_str, fastchg_match_qf[i].idh, fastchg_match_qf[i].idl,
			 fastchg_match_qf[i].q_value, fastchg_match_qf[i].f_value);
	}

	return;
}

static int read_skin_range_data_from_node(struct device_node *node,
		const char *prop_str, struct oplus_chg_wls_skin_range_data *ranges,
		int max_threshold, u32 max_value)
{
	int rc = 0, i, length, per_tuple_length, tuples;

	if (!node || !prop_str || !ranges) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u32));
	if (rc < 0) {
		chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;
	per_tuple_length = sizeof(struct oplus_chg_wls_skin_range_data) / sizeof(u32);
	if (length % per_tuple_length) {
		chg_err("%s length (%d) should be multiple of %d\n", prop_str, length, per_tuple_length);
		return -EINVAL;
	}
	tuples = length / per_tuple_length;

	if (tuples > WLS_MAX_STEP_CHG_ENTRIES) {
		chg_err("too many entries(%d), only %d allowed\n", tuples, WLS_MAX_STEP_CHG_ENTRIES);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(node, prop_str, (u32 *)ranges, length);
	if (rc) {
		chg_err("Read %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	for (i = 0; i < tuples; i++) {
		if (ranges[i].low_threshold > ranges[i].high_threshold) {
			chg_err("%s thresholds should be in ascendant ranges\n", prop_str);
			rc = -EINVAL;
			goto clean;
		}

		if (ranges[i].low_threshold > max_threshold)
			ranges[i].low_threshold = max_threshold;
		if (ranges[i].high_threshold > max_threshold)
			ranges[i].high_threshold = max_threshold;
		if (ranges[i].curr_ma > max_value)
			ranges[i].curr_ma = max_value;
	}

	return tuples;
clean:
	memset(ranges, 0, tuples * sizeof(struct oplus_chg_wls_skin_range_data));
	return rc;
}

static bool check_wls_range_data_is_valid(const char *prop_str, int length)
{
	int tuples;
	int per_tuple_length;

	per_tuple_length = sizeof(struct oplus_chg_wls_range_data) / sizeof(u32);
	if (length % per_tuple_length) {
		chg_err("%s length (%d) should be multiple of %d\n", prop_str, length, per_tuple_length);
		return false;
	}
	tuples = length / per_tuple_length;
	if (tuples > WLS_MAX_STEP_CHG_ENTRIES) {
		chg_err("too many entries(%d), only %d allowed\n", tuples, WLS_MAX_STEP_CHG_ENTRIES);
		return false;
	}

	return true;
}

static const char * const strategy_soc[] = {
	[WLS_FAST_SOC_0_TO_30]	= "strategy_soc_0_to_30",
	[WLS_FAST_SOC_0_TO_30_HIGH]	= "strategy_soc_0_to_30_high",
	[WLS_FAST_SOC_30_TO_70]	= "strategy_soc_30_to_70",
	[WLS_FAST_SOC_30_TO_70_HIGH]	= "strategy_soc_30_to_70_high",
	[WLS_FAST_SOC_70_TO_90]	= "strategy_soc_70_to_90",
	[WLS_FAST_SOC_70_TO_90_HIGH]	= "strategy_soc_70_to_90_high",
};

static const char * const strategy_temp[] = {
	[WLS_FAST_TEMP_0_TO_50]		= "strategy_temp_0_to_50",
	[WLS_FAST_TEMP_50_TO_120]	= "strategy_temp_50_to_120",
	[WLS_FAST_TEMP_120_TO_160]	= "strategy_temp_120_to_160",
	[WLS_FAST_TEMP_160_TO_400]	= "strategy_temp_160_to_400",
	[WLS_FAST_TEMP_160_TO_400_HIGH]	= "strategy_temp_160_to_400_high",
	[WLS_FAST_TEMP_400_TO_440]	= "strategy_temp_400_to_440",
};

static const char * const bcc_soc[] = {
	[WLS_BCC_SOC_0_TO_30]	= "bcc_soc_0_to_30",
	[WLS_BCC_SOC_0_TO_30_HIGH]	= "bcc_soc_0_to_30_high",
	[WLS_BCC_SOC_30_TO_70]	= "bcc_soc_30_to_70",
	[WLS_BCC_SOC_30_TO_70_HIGH]	= "bcc_soc_30_to_70_high",
	[WLS_BCC_SOC_70_TO_90]	= "bcc_soc_70_to_90",
	[WLS_BCC_SOC_70_TO_90_HIGH]	= "bcc_soc_70_to_90_high",
};

static const char * const bcc_temp[] = {
	[WLS_BCC_TEMP_0_TO_50]		= "bcc_temp_0_to_50",
	[WLS_BCC_TEMP_50_TO_120]	= "bcc_temp_50_to_120",
	[WLS_BCC_TEMP_120_TO_160]	= "bcc_temp_120_to_160",
	[WLS_BCC_TEMP_160_TO_400]	= "bcc_temp_160_to_400",
	[WLS_BCC_TEMP_160_TO_400_HIGH]	= "bcc_temp_160_to_400_high",
	[WLS_BCC_TEMP_400_TO_440]	= "bcc_temp_400_to_440",
};

static const char * const bcc_stop_curr[] = {
	[WLS_BCC_STOP_0_TO_30]	= "bcc_stop_curr_0_to_30",
	[WLS_BCC_STOP_0_TO_30_HIGH]	= "bcc_stop_curr_0_to_30_high",
	[WLS_BCC_STOP_30_TO_70]	= "bcc_stop_curr_30_to_70",
	[WLS_BCC_STOP_30_TO_70_HIGH]	= "bcc_stop_curr_30_to_70_high",
	[WLS_BCC_STOP_70_TO_90]	= "bcc_stop_curr_70_to_90",
	[WLS_BCC_STOP_70_TO_90_HIGH]	= "bcc_stop_curr_70_to_90_high",
};

static const char * const strategy_non_ffc[] = {
	[OPLUS_WLS_CHG_MODE_BPP]	= "non-ffc-bpp",
	[OPLUS_WLS_CHG_MODE_EPP]	= "non-ffc-epp",
	[OPLUS_WLS_CHG_MODE_EPP_PLUS]	= "non-ffc-epp-plus",
	[OPLUS_WLS_CHG_MODE_AIRVOOC]	= "non-ffc-airvooc",
	[OPLUS_WLS_CHG_MODE_AIRSVOOC]	= "non-ffc-airsvooc",
};

static const char * const strategy_cv[] = {
	[OPLUS_WLS_CHG_MODE_BPP]	= "cv-bpp",
	[OPLUS_WLS_CHG_MODE_EPP]	= "cv-epp",
	[OPLUS_WLS_CHG_MODE_EPP_PLUS]	= "cv-epp-plus",
	[OPLUS_WLS_CHG_MODE_AIRVOOC]	= "cv-airvooc",
	[OPLUS_WLS_CHG_MODE_AIRSVOOC]	= "cv-airsvooc",
};

static const char * const strategy_cv_high_batt[] = {
	[OPLUS_WLS_CHG_MODE_BPP]	= "cv-bpp-high-batt",
	[OPLUS_WLS_CHG_MODE_EPP]	= "cv-epp-high-batt",
	[OPLUS_WLS_CHG_MODE_EPP_PLUS]	= "cv-epp-plus-high-batt",
	[OPLUS_WLS_CHG_MODE_AIRVOOC]	= "cv-airvooc-high-batt",
	[OPLUS_WLS_CHG_MODE_AIRSVOOC]	= "cv-airsvooc-high-batt",
};

static const char * const strategy_cool_down[] = {
	[OPLUS_WLS_CHG_MODE_BPP]	= "cool-down-bpp",
	[OPLUS_WLS_CHG_MODE_EPP]	= "cool-down-epp",
	[OPLUS_WLS_CHG_MODE_EPP_PLUS]	= "cool-down-epp-plus",
	[OPLUS_WLS_CHG_MODE_AIRVOOC]	= "cool-down-airvooc",
	[OPLUS_WLS_CHG_MODE_AIRSVOOC]	= "cool-down-airsvooc",
};

static const char * const strategy_skewing[] = {
	[OPLUS_WLS_SKEWING_EPP]		= "skewing-epp",
	[OPLUS_WLS_SKEWING_EPP_PLUS]	= "skewing-epp-plus",
	[OPLUS_WLS_SKEWING_AIRVOOC]	= "skewing-airvooc",
};

static const char * const norchg_strategy_mode[] = {
	[OPLUS_WLS_CHG_MODE_BPP]	= "wls_bpp_strategy",
	[OPLUS_WLS_CHG_MODE_EPP]	= "wls_epp_strategy",
	[OPLUS_WLS_CHG_MODE_EPP_PLUS]	= "wls_epp_plus_strategy",
	[OPLUS_WLS_CHG_MODE_AIRVOOC]	= "wls_airvooc_strategy",
	[OPLUS_WLS_CHG_MODE_AIRSVOOC]	= "wls_airsvooc_strategy",
};

static const char * const norchg_strategy_temp[] = {
	[TEMP_REGION_COLD]	= "wls_temp_cold",
	[TEMP_REGION_LITTLE_COLD]	= "wls_temp_little_cold",
	[TEMP_REGION_COOL]	= "wls_temp_cool",
	[TEMP_REGION_LITTLE_COOL]	= "wls_temp_little_cool",
	[TEMP_REGION_PRE_NORMAL]	= "wls_temp_pre_normal",
	[TEMP_REGION_NORMAL]	= "wls_temp_normal",
	[TEMP_REGION_NORMAL_HIGH]	= "wls_temp_normal_high",
	[TEMP_REGION_WARM]	= "wls_temp_warm",
	[TEMP_REGION_HOT]	= "wls_temp_hot",
};

static const char * const norchg_strategy_vol[] = {
	[OPLUS_WLS_CHG_BATT_CL_LOW]	= "strategy_low_battery",
	[OPLUS_WLS_CHG_BATT_CL_HIGH]	= "strategy_high_battery",
};

static int read_fod_parm_data_from_node(struct device_node *node, const char *prop_str,
		struct wls_fod_parm_type ranges[], u32 total_len, u32 per_tuple_len)
{
	int rc = 0;
	int i;
	int j;
	int tuples;
	u8 *buf_temp = NULL;

	if (!node || !prop_str || !ranges) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	if (per_tuple_len > WLS_FOD_PARM_LEN_MAX) {
		chg_err("per_tuple_len is too large, only %d allowed\n", WLS_FOD_PARM_LEN_MAX);
		return -EINVAL;
	}
	if (total_len > sizeof(struct wls_fod_parm_type) * WLS_BASE_NUM_MAX) {
		chg_err("total_len is too large, only %lu allowed\n", sizeof(struct wls_fod_parm_type) * WLS_BASE_NUM_MAX);
		return -EINVAL;
	}

	if (total_len % per_tuple_len) {
		chg_err("%s length (%d) should be multiple of %d\n", prop_str, total_len, per_tuple_len);
		return -EINVAL;
	}
	tuples = total_len / per_tuple_len;

	if (tuples > WLS_BASE_NUM_MAX) {
		chg_err("too many entries(%d), only %d allowed\n", tuples, WLS_BASE_NUM_MAX);
		return -EINVAL;
	}

	buf_temp = kzalloc(total_len, GFP_KERNEL);
	if (!buf_temp) {
		chg_err("alloc memary error\n");
		return -ENOMEM;
	}
	rc = of_property_read_u8_array(node, prop_str, (u8*)buf_temp, total_len);
	if (rc) {
		chg_err("Read %s failed, rc=%d\n", prop_str, rc);
		kfree(buf_temp);
		return rc;
	}

	for (i = 0; i < tuples; i++)
		memcpy(&ranges[i], &buf_temp[i * per_tuple_len], per_tuple_len);

	for (i = 0; i < tuples; i++) {
		printk(KERN_CONT "%s: 0x%x:", prop_str, ranges[i].id);
		for (j = 0; j < per_tuple_len - 1; j++)
			printk(KERN_CONT " 0x%x", ranges[i].fod_parm[j]);
		printk(KERN_CONT "\n");
	}

	kfree(buf_temp);
	return tuples;
}

static void print_third_part_fod_parm_data(struct oplus_chg_wls *wls_dev, const char *prop_str,
				struct wls_third_part_fod_parm ranges[], int *set) {
	struct oplus_chg_wls_static_config *static_cfg = &wls_dev->static_config;
	int i = 0;
	int j = 0;

	if (!prop_str || !ranges) {
		chg_err("Invalid parameters passed\n");
		return;
	}

	for (i = 0; i < *set; i++) {
		printk(KERN_CONT "%s: 0x%x, 0x%x:", prop_str, ranges[i].idh, ranges[i].idl);
		for (j = 0; j < static_cfg->fod_parm_len; j++)
			printk(KERN_CONT " 0x%x", ranges[i].fod_parm[j]);
		printk(KERN_CONT "\n");
	}
}

static void read_third_part_fod_parm_data_from_node(struct oplus_chg_wls *wls_dev, struct device_node *node, const char *prop_str,
				struct wls_third_part_fod_parm ranges[], int *set)
{
	int rc = 0;
	int length = 0;
	u8 *buf_temp = NULL;
	int per_tuple_len = 0;
	struct oplus_chg_wls_static_config *static_cfg = &wls_dev->static_config;
	int i = 0;

	if (!node || !ranges) {
		*set = 0;
		chg_err("Invalid parameters passed\n");
		return;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u8));
	per_tuple_len = offsetof(struct wls_third_part_fod_parm, fod_parm) + static_cfg->fod_parm_len;
	if (rc <= 0 || per_tuple_len <= 2 || per_tuple_len > sizeof(struct wls_third_part_fod_parm)) {
		*set = 0;
		chg_err("Read %s failed, rc=%d, per_tuple_len=%d\n", prop_str, rc, per_tuple_len);
		return;
	}

	length = rc;
	*set = length / per_tuple_len;
	if (*set > WLS_THIRD_PART_TX_CONFIG_NUM_MAX) {
		chg_err("%s is out of range, rc=%d\n", prop_str, *set);
		*set = 0;
		return;
	}

	buf_temp = kzalloc(length, GFP_KERNEL);
	if (!buf_temp) {
		*set = 0;
		chg_err("alloc memary error\n");
		return;
	}
	rc = of_property_read_u8_array(node, prop_str, (u8*)buf_temp, length);
	if (rc) {
		*set = 0;
		chg_err("Read %s failed, rc=%d\n", prop_str, rc);
		kfree(buf_temp);
		return;
	}

	for (i = 0; i < *set; i++)
		memmove(&ranges[i], &buf_temp[i * per_tuple_len], per_tuple_len);

	print_third_part_fod_parm_data(wls_dev, prop_str, ranges, set);

	kfree(buf_temp);
}

static int read_fastchg_strategy_from_node(struct device_node *node, const char *prop_str,
		struct oplus_chg_wls_fcc_steps ranges[])
{
	struct device_node *fastchg_strategy_node, *soc_strategy_node;
	int i, j, k;
	int length;
	int rc;

	if (!node || !prop_str || !ranges) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	fastchg_strategy_node = of_get_child_by_name(node, prop_str);
	if (!fastchg_strategy_node) {
		chg_err("Can not find %s node\n", prop_str);
		return -EINVAL;
	}

	for (i = 0; i < WLS_FAST_SOC_MAX; i++) {
		soc_strategy_node = of_get_child_by_name(fastchg_strategy_node, strategy_soc[i]);
		if (!soc_strategy_node && (i == WLS_FAST_SOC_0_TO_30_HIGH ||
		    i == WLS_FAST_SOC_30_TO_70_HIGH || i == WLS_FAST_SOC_70_TO_90_HIGH)) {
			memmove(&ranges[i], &ranges[i - 1], sizeof(struct oplus_chg_wls_fcc_steps));
			continue;
		}
		if (!soc_strategy_node) {
			chg_err("Can not find %s node\n", strategy_soc[i]);
			return -EINVAL;
		}
		for (j = 0; j < WLS_FAST_TEMP_MAX; j++) {
			rc = of_property_count_elems_of_size(soc_strategy_node, strategy_temp[j], sizeof(u32));
			if (rc < 0 && j == WLS_FAST_TEMP_160_TO_400_HIGH) {
				memmove(&ranges[i].fcc_step[j], &ranges[i].fcc_step[j-1],
					sizeof(struct oplus_chg_wls_fcc_step));
				continue;
			}
			if (rc < 0) {
				chg_err("Count %s failed, rc=%d\n", strategy_temp[j], rc);
				return rc;
			}
			length = rc;
			if (!check_wls_range_data_is_valid(prop_str, length))
				return -EINVAL;
			rc = of_property_read_u32_array(soc_strategy_node, strategy_temp[j],
				(u32 *)ranges[i].fcc_step[j].fcc_step, length);

			if (rc < 0) {
				chg_err("read %s array failed, rc=%d\n", prop_str, rc);
				return rc;
			}
			ranges[i].fcc_step[j].max_step = length / (sizeof(struct oplus_chg_wls_range_data) / sizeof(u32));
		}
	}
	for (i = 0; i < WLS_FAST_SOC_MAX; i++) {
		for (j = 0; j < WLS_FAST_TEMP_MAX; j++) {
			for (k = 0; k < ranges[i].fcc_step[j].max_step; k++) {
				chg_debug("%s, %s, %s: %d %d %d %d %d\n",
					prop_str, strategy_soc[i], strategy_temp[j],
					ranges[i].fcc_step[j].fcc_step[k].low_threshold,
					ranges[i].fcc_step[j].fcc_step[k].high_threshold,
					ranges[i].fcc_step[j].fcc_step[k].curr_ma,
					ranges[i].fcc_step[j].fcc_step[k].vol_max_mv,
					ranges[i].fcc_step[j].fcc_step[k].need_wait);
			}
		}
	}

	return 0;
}

static int read_norchg_strategy_from_node(struct device_node *node, const char *prop_str,
		struct oplus_chg_wls_norchg_steps* ranges) {
	struct device_node *nor_strategy_node, *norchg_mode_node, *norchg_vol_node;
	int i, j, k;
	int length;
	int rc;

	if (!node || !prop_str || !ranges) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	nor_strategy_node = of_get_child_by_name(node, prop_str);
	if (!nor_strategy_node) {
		chg_err("Can not find %s node\n", prop_str);
		return -EINVAL;
	}

	for (i = 0; i < WLS_NORCHAG_MODE_MAX; i++) {
		norchg_mode_node = of_get_child_by_name(nor_strategy_node, norchg_strategy_mode[i]);
		if (!norchg_mode_node) {
			chg_err("Can not find %s node\n", norchg_strategy_mode[i]);
			return -EINVAL;
		}
		for (j = 0; j < OPLUS_WLS_CHG_BATT_CL_MAX; j++) {
			norchg_vol_node = of_get_child_by_name(norchg_mode_node, norchg_strategy_vol[j]);
			if (!norchg_vol_node) {
				chg_err("Can not find %s node\n", norchg_strategy_vol[j]);
				return -EINVAL;
			}

			for (k = TEMP_REGION_LITTLE_COLD; k < TEMP_REGION_HOT; k++) {
				rc = of_property_count_elems_of_size(norchg_vol_node, norchg_strategy_temp[k], sizeof(u32));
				if (rc < 0) {
					chg_err("Count %s failed, rc=%d\n", norchg_strategy_temp[k], rc);
					return rc;
				}
				length = rc;

				if (!check_wls_range_data_is_valid(norchg_strategy_temp[k], length))
					return -EINVAL;

				rc = of_property_read_u32_array(norchg_vol_node, norchg_strategy_temp[k],
					(u32 *)ranges->norchg_step[i][j][k].norchg_step, length);
				if (rc < 0) {
					chg_err("read %s array failed, rc=%d\n", norchg_strategy_temp[k], rc);
					return rc;
				}
				ranges->norchg_step[i][j][k].max_step = length / (sizeof(struct oplus_chg_wls_range_data) / sizeof(u32));
			}
		}
	}

	return 0;
}

static void oplus_chg_wls_fan_pulse_parse_dt(struct oplus_chg_wls *wls_dev, struct device_node *node)
{
	int length;
	int rc;
	int fan_pwm_pulse_len;
	int fan_config_sets;
	unsigned int *pdata = NULL;
	int i = 0;
	int j = 0;

	length = of_property_count_elems_of_size(node, "oplus,fan-pwm-pulse", sizeof(u32));
	if (length <= 0) {
		chg_info("Read oplus,fan-pwm-pulse size failed, length=%d, use default fan_pwm_pulse\n", length);
		return;
	}

	rc = of_property_read_u32(node, "oplus,fan-pwm-pulse-len", &fan_pwm_pulse_len);
	if ((rc < 0) || (fan_pwm_pulse_len <= 0) ||
	    (fan_pwm_pulse_len > sizeof(struct wls_fan_pwm_pulse) / sizeof(u32))) {
		chg_err("Read oplus,fan-pwm-pulse-len failed, rc=%d\n", rc);
		return;
	}

	fan_config_sets = length / fan_pwm_pulse_len;
	if (fan_config_sets > ARRAY_SIZE(fan_pwm_pulse)) {
		chg_err("oplus,fan-pwm-pulse size err, use default fan_pwm_pulse\n");
		return;
	}

	pdata = (unsigned int*)kzalloc(length * sizeof(u32), GFP_KERNEL);
	if (pdata == NULL) {
		chg_err("alloc memory error\n");
		return;
	}

	rc = of_property_read_u32_array(node, "oplus,fan-pwm-pulse", pdata, length);
	if (rc < 0) {
		chg_err("Read oplus,fan-pwm-pulse failed, rc=%d, use default fan_pwm_pulse\n", rc);
		kfree(pdata);
		return;
	} else {
		for (i = 0; i < fan_config_sets; i++) {
			for (j = 0; j < ARRAY_SIZE(fan_pwm_pulse); j++) {
				if (*(pdata + i * fan_pwm_pulse_len) == fan_pwm_pulse[j].wls_model_id) {
					memcpy(&fan_pwm_pulse[j], pdata + i * fan_pwm_pulse_len, fan_pwm_pulse_len * sizeof(u32));
					break;
				}
			}
		}
	}
	kfree(pdata);

	for (j = 0; j < ARRAY_SIZE(fan_pwm_pulse); j++) {
		chg_info("%u %u %u %u %u %u\n",
			fan_pwm_pulse[j].wls_model_id,
			fan_pwm_pulse[j].fan_pwm_pulse_fastchg,
			fan_pwm_pulse[j].fan_pwm_pulse_silent,
			fan_pwm_pulse[j].fan_pwm_pulse_fastchg_exit,
			fan_pwm_pulse[j].fan_pwm_pulse_ui_100,
			fan_pwm_pulse[j].fan_pwm_pulse_cool_mode);
	}
}

static void oplus_chg_wls_flag_parse_dt(struct oplus_chg_wls *wls_dev)
{
	struct device_node *node = oplus_get_node_by_type(wls_dev->dev->of_node);

	wls_dev->support_epp_plus = of_property_read_bool(node, "oplus,support_epp_plus");
	wls_dev->support_fastchg = of_property_read_bool(node, "oplus,support_fastchg");
	wls_dev->support_get_tx_pwr = of_property_read_bool(node, "oplus,support_get_tx_pwr");
	wls_dev->support_tx_boost = of_property_read_bool(node, "oplus,support_tx_boost");
	wls_dev->support_wls_and_tx_boost = of_property_read_bool(node, "oplus,support_wls_and_tx_boost");
	wls_dev->support_wls_chg_bcc = of_property_read_bool(node, "oplus,support_wls_chg_bcc");
	wls_dev->support_wls_use_usb_drv = of_property_read_bool(node, "oplus,support_wls_use_usb_drv");
	wls_dev->support_acdrv_no_ext_boost = of_property_read_bool(node, "oplus,support_acdrv_no_ext_boost");
	wls_dev->support_no_hw_interlock = of_property_read_bool(node, "oplus,support_no_hw_interlock");
}

#define IS_VALID_ICLMAX_COUNT(cnt) \
	((cnt) == ((BATT_TEMP_MAX - 1) * OPLUS_WLS_CHG_MODE_MAX) || \
	(cnt) == (BATT_TEMP_MAX * OPLUS_WLS_CHG_MODE_MAX))

static void oplus_chg_wls_iclmax_parse_dt(struct oplus_chg_wls *wls_dev, struct device_node *node,
					   const char *prop_str, int cl_idx)
{
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int arr[BATT_TEMP_MAX * OPLUS_WLS_CHG_MODE_MAX] = {0};
	int t = 0;
	int i, m, cnt, rc;

	cnt = of_property_count_elems_of_size(node, prop_str, sizeof(u32));
	if (IS_VALID_ICLMAX_COUNT(cnt)) {
		rc = read_unsigned_data_from_node(node, prop_str, (u32 *)arr, cnt);
		if (rc < 0) {
			chg_err("get oplus,iclmax-ma property error, rc=%d\n", rc);
			memmove(dynamic_cfg->iclmax_ma[cl_idx],
				default_config.iclmax_ma[cl_idx],
				OPLUS_WLS_CHG_MODE_MAX * BATT_TEMP_MAX * sizeof(u32));
			return;
		}
	}

	for (i = 0; i < OPLUS_WLS_CHG_MODE_MAX; i++) {
		wls_dev->icl_max_ma[i] = 0;
		for (m = 0; m < BATT_TEMP_MAX; m++) {
			if ((cnt == (BATT_TEMP_MAX - 1) * OPLUS_WLS_CHG_MODE_MAX) && (m == BATT_TEMP_NORMAL_HIGH)) {
				dynamic_cfg->iclmax_ma[cl_idx][i][m] = arr[t - 1];
			} else {
				dynamic_cfg->iclmax_ma[cl_idx][i][m] = arr[t];
				t++;
			}
			wls_dev->icl_max_ma[i] = dynamic_cfg->iclmax_ma[cl_idx][i][m] >
						 wls_dev->icl_max_ma[i] ?
						 dynamic_cfg->iclmax_ma[cl_idx][i][m] :
						 wls_dev->icl_max_ma[i];
		}
	}

	for (i = 0; i < OPLUS_WLS_CHG_MODE_MAX; i++) {
		chg_debug("CHG_BATT_CL_IDX:%d: %d %d %d %d %d %d %d %d %d %d, iclmax_ma:%d\n",
			cl_idx,
			dynamic_cfg->iclmax_ma[cl_idx][i][0],
			dynamic_cfg->iclmax_ma[cl_idx][i][1],
			dynamic_cfg->iclmax_ma[cl_idx][i][2],
			dynamic_cfg->iclmax_ma[cl_idx][i][3],
			dynamic_cfg->iclmax_ma[cl_idx][i][4],
			dynamic_cfg->iclmax_ma[cl_idx][i][5],
			dynamic_cfg->iclmax_ma[cl_idx][i][6],
			dynamic_cfg->iclmax_ma[cl_idx][i][7],
			dynamic_cfg->iclmax_ma[cl_idx][i][8],
			dynamic_cfg->iclmax_ma[cl_idx][i][9],
			wls_dev->icl_max_ma[i]);
	}
}

#define IS_BCC_STOP_SOC_NEW_VER(i)  \
	((i) == WLS_BCC_STOP_0_TO_30_HIGH ||  \
	 (i) == WLS_BCC_STOP_30_TO_70_HIGH ||  \
	 (i) == WLS_BCC_STOP_70_TO_90_HIGH)

#define IS_VALID_BCC_TEMP_COUNT(cnt) \
	(((cnt) == WLS_BCC_TEMP_MAX - 1) || (cnt) == WLS_BCC_TEMP_MAX)

static void oplus_chg_wls_bcc_stop_cur_parse_dt(struct oplus_chg_wls *wls_dev, struct device_node *node)
{
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int i, j, k, rc, cnt;
	int idx;

	for (i = 0; i < WLS_BCC_STOP_SOC_MAX; i++) {
		cnt = of_property_count_elems_of_size(node, bcc_stop_curr[i], sizeof(u32));
		if ((cnt < 0) && IS_BCC_STOP_SOC_NEW_VER(i)) {
			memmove(&dynamic_cfg->bcc_stop_curr[i], &dynamic_cfg->bcc_stop_curr[i - 1],
				sizeof(struct oplus_chg_wls_bcc_stop_curr));
		} else if (IS_VALID_BCC_TEMP_COUNT(cnt)) {
			for (j = 0; j < WLS_BCC_TEMP_MAX; j++) {
				idx = (cnt == WLS_BCC_TEMP_MAX - 1 && j >= WLS_BCC_TEMP_160_TO_400_HIGH) ? j - 1 : j;
				rc = of_property_read_u32_index(node, bcc_stop_curr[i], idx,
					&dynamic_cfg->bcc_stop_curr[i].bcc_soc_stop_curr[j]);
				if (rc < 0) {
					for (k = 0; k < WLS_BCC_TEMP_MAX; k++)
						dynamic_cfg->bcc_stop_curr[i].bcc_soc_stop_curr[k] = 1000;
					break;
				}
			}
		} else {
			for (j = 0; j < WLS_BCC_TEMP_MAX; j++)
				dynamic_cfg->bcc_stop_curr[i].bcc_soc_stop_curr[j] = 1000;
		}

		for (j = 0; j < WLS_BCC_TEMP_MAX; j++)
			chg_debug("%s: %d", bcc_stop_curr[i], dynamic_cfg->bcc_stop_curr[i].bcc_soc_stop_curr[j]);
	}
}

static void oplus_chg_wls_strategy_soc_parse_dt(struct oplus_chg_wls *wls_dev, struct device_node *node)
{
	int index = 0;
	int i, rc;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	int strategy_soc_len;

	strategy_soc_len = of_property_count_elems_of_size(node, "oplus,strategy_soc", sizeof(u32));
	if (strategy_soc_len == WLS_SOC_NUM_MAX_OLD_VER || strategy_soc_len == WLS_SOC_NUM_MAX) {
		for (i = 0; i < WLS_SOC_NUM_MAX; i++) {
			index = (strategy_soc_len == WLS_SOC_NUM_MAX_OLD_VER) ? i / 2 : i;
			rc = of_property_read_u32_index(node, "oplus,strategy_soc", index,
							&dynamic_cfg->wls_strategy_soc[i]);
			if (rc < 0) {
				chg_err("get oplus,strategy_soc property error, rc=%d\n", rc);
				goto default_set;
			}
		}
	}

	for (i = 0; i < WLS_SOC_NUM_MAX; i++)
		chg_info(" strategy_soc: %d", dynamic_cfg->wls_strategy_soc[i]);

	goto end;

default_set:
	if (strategy_soc_len == WLS_SOC_NUM_MAX) {
		memmove(dynamic_cfg->wls_strategy_soc, &oplus_strategy_soc_default_para, sizeof(int32_t) * WLS_SOC_NUM_MAX);
	} else {
		for (i = 0; i < WLS_SOC_NUM_MAX; i++)
			dynamic_cfg->wls_strategy_soc[i] = oplus_strategy_soc_default_para_old_ver[i / 2];
	}

end:
	return;
}

static int oplus_chg_wls_parse_dt(struct oplus_chg_wls *wls_dev)
{
	struct device_node *node = oplus_get_node_by_type(wls_dev->dev->of_node);
	struct device_node *soc_strategy_node;
	struct device_node *wls_third_part_strategy_node;
	struct device_node *wls_bcc_table_node, *soc_bcc_node;
	struct oplus_chg_wls_static_config *static_cfg = &wls_dev->static_config;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	struct oplus_chg_wls_skin_step *skin_step;
	int i, j, k, length;
	int rc, cnt;

	oplus_chg_wls_flag_parse_dt(wls_dev);
	chg_info("node=%s\n", node->full_name);

	rc = of_property_read_u32(node, "oplus,normal_high_batt_temp", &wls_dev->normal_high_batt_temp);
	if (rc < 0) {
		chg_info("oplus,oplus,normal_high_batt_temp reading failed, rc=%d\n", rc);
		wls_dev->normal_high_batt_temp = 210;
	}
	rc = of_property_read_u32(node, "oplus,little_warm_batt_temp", &wls_dev->little_warm_batt_temp);
	if (rc < 0) {
		chg_info("oplus,oplus,little_warm_batt_temp reading failed, rc=%d\n", rc);
		wls_dev->little_warm_batt_temp = 375;
	}
	rc = of_property_read_u32(node, "oplus,wls_phone_id", &wls_dev->wls_phone_id);
	if (rc < 0) {
		chg_err("oplus,wls_phone_id reading failed, rc=%d\n", rc);
		wls_dev->wls_phone_id = 0x000A;
	}
	rc = of_property_read_u32(node, "oplus,rx_coil", &wls_dev->rx_coil);
	if (rc < 0) {
		chg_err("oplus,rx_coil reading failed, rc=%d\n", rc);
		wls_dev->rx_coil = 48;
	}
	rc = of_property_read_u32(node, "oplus,vbridge_ratio", &wls_dev->vbridge_ratio);
	if (rc < 0) {
		chg_err("oplus,vbridge_ratio reading failed, rc=%d\n", rc);
		wls_dev->vbridge_ratio = 14;
	}
	rc = of_property_read_u32(node, "oplus,magcvr_vbridge_ratio", &wls_dev->magcvr_vbridge_ratio);
	if (rc < 0) {
		chg_err("oplus,magcvr_vbridge_ratio reading failed, rc=%d\n", rc);
		wls_dev->magcvr_vbridge_ratio = 28;
	}
	rc = of_property_read_u32(node, "oplus,rx_protocol_version", &wls_dev->rx_protocol_version);
	if (rc < 0) {
		chg_err("oplus,rx_protocol_version reading failed, rc=%d\n", rc);
		wls_dev->rx_protocol_version = 28;
	}
	rc = of_property_read_u32(node, "oplus,non_mag_power_mw", &wls_dev->non_mag_power_mw);
	if (rc < 0) {
		chg_err("oplus,non_mag_power_mw reading failed, rc=%d\n", rc);
		wls_dev->non_mag_power_mw = 20000;
	}

	rc = of_property_read_u32(node, "oplus,wls_power_mw", &wls_dev->wls_power_mw);
	if (rc < 0) {
		chg_err("oplus,oplus,wls_power_mw reading failed, rc=%d\n", rc);
		wls_dev->wls_power_mw = 30000;
	}
	rc = of_property_read_u32(node, "oplus,wls_boost_curr_limit_ma", &wls_dev->wls_boost_curr_limit_ma);
	if (rc < 0) {
		chg_info("oplus,oplus,wls_boost_curr_limit_ma reading failed, rc=%d\n", rc);
		wls_dev->wls_boost_curr_limit_ma = 0;
	}
	rc = of_property_count_elems_of_size(node, "oplus,fastchg-match-q", sizeof(u8));
	if (rc < 0) {
		chg_err("Count oplus,fastchg-match-q failed, rc=%d\n", rc);
	} else {
		length = rc;
		read_q_value_from_node(node, "oplus,fastchg-match-q", static_cfg->fastchg_match_q,
				       length, wls_dev->rx_protocol_version);

		rc = of_property_count_elems_of_size(node, "oplus,fastchg-match-q-magcvr", sizeof(u8));
		if (rc < 0) {
			memcpy(&static_cfg->fastchg_match_q_magcvr, &static_cfg->fastchg_match_q, length);
		} else {
			length = rc;
			read_q_value_from_node(node, "oplus,fastchg-match-q-magcvr", static_cfg->fastchg_match_q_magcvr,
					       length, wls_dev->rx_protocol_version);
		}
	}

	(void)read_third_part_qf_value_from_node(node, "oplus,wls-third-part-fastchg-qf-parm",
		static_cfg->third_part_fastchg_tx_qf_parm, &static_cfg->third_part_fastchg_qf_parm_sets);
	(void)read_third_part_qf_value_from_node(node, "oplus,wls-third-part-fastchg-qf-parm-magcvr",
		static_cfg->third_part_fastchg_tx_qf_parm_magcvr, &static_cfg->third_part_fastchg_qf_parm_magcvr_sets);
	(void)read_third_part_qf_value_from_node(node, "oplus,wls-third-part-epp-qf-parm",
		static_cfg->third_part_epp_tx_qf_parm, &static_cfg->third_part_epp_qf_parm_sets);
	(void)read_third_part_qf_value_from_node(node, "oplus,wls-third-part-epp-qf-parm-magcvr",
		static_cfg->third_part_epp_tx_qf_parm_magcvr, &static_cfg->third_part_epp_qf_parm_magcvr_sets);

	rc = of_property_count_elems_of_size(node, "oplus,disable-fod-parm", sizeof(u8));
	if (rc < 0) {
		static_cfg->fod_parm_len = sizeof(oplus_chg_wls_disable_fod_parm);
		chg_err("Read oplus,disable-fod-parm len failed, rc=%d\n", rc);
	} else {
		static_cfg->fod_parm_len = rc;
		chg_info("oplus,disable-fod-parm length:%d\n", static_cfg->fod_parm_len);
	}
	rc = of_property_read_u8_array(node, "oplus,disable-fod-parm",
			(u8 *)&static_cfg->disable_fod_parm, static_cfg->fod_parm_len);
	if (rc < 0) {
		chg_err("Read oplus,disable-fod-parm failed, rc=%d, use default parm.\n", rc);
		memcpy(&static_cfg->disable_fod_parm, oplus_chg_wls_disable_fod_parm,
			static_cfg->fod_parm_len * sizeof(u8));
	}

	static_cfg->fastchg_fod_enable = of_property_read_bool(node, "oplus,fastchg-fod-enable");
	if (static_cfg->fastchg_fod_enable) {
		rc = of_property_read_u32(node, "oplus,fastchg-fod-parm-len", &static_cfg->fod_parm_len);
		if (rc < 0) {
			static_cfg->fod_parm_len = sizeof(oplus_chg_wls_disable_fod_parm);
			chg_err("Read oplus,fastchg-fod-parm-len failed, rc=%d\n", rc);
		}
		if (static_cfg->fod_parm_len > WLS_FOD_PARM_LEN_MAX)
			static_cfg->fod_parm_len = WLS_FOD_PARM_LEN_MAX;
		rc = of_property_read_u32(node, "oplus,fod-parm-array-row-len", &static_cfg->fod_parm_array_row_len);
		if (rc < 0) {
			/*old projects: WLS_FOD_PARM_LEN_MAX is 32 + dock id len*/
			static_cfg->fod_parm_array_row_len = 33;
			chg_info("Read oplus,fod-parm-array-row-len failed, default len=%d\n", static_cfg->fod_parm_array_row_len);
		}
		if (static_cfg->fod_parm_array_row_len > sizeof(struct wls_fod_parm_type))
			static_cfg->fod_parm_array_row_len = sizeof(struct wls_fod_parm_type);

		rc = of_property_count_elems_of_size(node, "oplus,fastchg-fod-parm", sizeof(u8));
		if (rc < 0) {
			static_cfg->fod_parm_len = sizeof(oplus_chg_wls_disable_fod_parm);
			static_cfg->fastchg_fod_enable = false;
			chg_err("Read oplus,fastchg-fod-parm size failed, rc=%d\n", rc);
		} else {
			length = rc;
			static_cfg->fastchg_fod_parm_sets = length / static_cfg->fod_parm_array_row_len;
			rc = read_fod_parm_data_from_node(node, "oplus,fastchg-fod-parm",
				static_cfg->fastchg_fod_parm, length, static_cfg->fod_parm_array_row_len);
			if (rc < 0) {
				static_cfg->fastchg_fod_enable = false;
				chg_err("Read oplus,fastchg-fod-parm failed, rc=%d\n", rc);
			} else {
				rc = of_property_count_elems_of_size(node, "oplus,fastchg-fod-parm-magcvr", sizeof(u8));
				if (rc < 0) {
					memcpy(&static_cfg->fastchg_fod_parm_magcvr, &static_cfg->fastchg_fod_parm,
						length);
					chg_info("Read oplus,fastchg-fod-parm-magcvr size failed, rc=%d\n", rc);
				} else {
					rc = read_fod_parm_data_from_node(node, "oplus,fastchg-fod-parm-magcvr",
						static_cfg->fastchg_fod_parm_magcvr, length,
						static_cfg->fod_parm_array_row_len);
					if (rc < 0) {
						static_cfg->fastchg_fod_enable = false;
						chg_err("Read oplus,fastchg-fod-parm-magcvr failed, rc=%d\n", rc);
					}
				}
			}
		}

		rc = of_property_count_elems_of_size(node, "oplus,bpp-fod-parm", sizeof(u8));
		if (rc < 0) {
			static_cfg->bpp_fod_parm_sets = 0;
			chg_info("Read oplus,bpp-fod-parm size failed, rc=%d\n", rc);
		} else {
			length = rc;
			static_cfg->bpp_fod_parm_sets = length / static_cfg->fod_parm_array_row_len;
			rc = read_fod_parm_data_from_node(node, "oplus,bpp-fod-parm",
				static_cfg->bpp_fod_parm, length, static_cfg->fod_parm_array_row_len);
			if (rc < 0) {
				static_cfg->bpp_fod_parm_sets = 0;
				chg_err("Read oplus,bpp-fod-parm failed, rc=%d\n", rc);
			} else {
				rc = of_property_count_elems_of_size(node, "oplus,bpp-fod-parm-magcvr", sizeof(u8));
				if (rc < 0) {
					static_cfg->bpp_fod_parm_sets = 0;
					chg_info("Read oplus,bpp-fod-parm-magcvr size failed, rc=%d\n", rc);
				} else {
					rc = read_fod_parm_data_from_node(node, "oplus,bpp-fod-parm-magcvr",
						static_cfg->bpp_fod_parm_magcvr, length, static_cfg->fod_parm_array_row_len);
					if (rc < 0) {
						static_cfg->bpp_fod_parm_sets = 0;
						chg_err("Read oplus,bpp-fod-parm-magcvr failed, rc=%d\n", rc);
					}
				}
			}
		}

		rc = of_property_count_elems_of_size(node, "oplus,epp-fod-parm", sizeof(u8));
		if (rc < 0) {
			static_cfg->epp_fod_parm_sets = 0;
			chg_info("Read oplus,epp-fod-parm size failed, rc=%d\n", rc);
		} else {
			length = rc;
			static_cfg->epp_fod_parm_sets = length / static_cfg->fod_parm_array_row_len;
			rc = read_fod_parm_data_from_node(node, "oplus,epp-fod-parm",
				static_cfg->epp_fod_parm, length, static_cfg->fod_parm_array_row_len);
			if (rc < 0) {
				static_cfg->epp_fod_parm_sets = 0;
				chg_err("Read oplus,epp-fod-parm failed, rc=%d\n", rc);
			} else {
				rc = of_property_count_elems_of_size(node, "oplus,epp-fod-parm-magcvr", sizeof(u8));
				if (rc < 0) {
					static_cfg->epp_fod_parm_sets = 0;
					chg_info("Read oplus,epp-fod-parm-magcvr size failed, rc=%d\n", rc);
				} else {
					rc = read_fod_parm_data_from_node(node, "oplus,epp-fod-parm-magcvr",
						static_cfg->epp_fod_parm_magcvr, length, static_cfg->fod_parm_array_row_len);
					if (rc < 0) {
						static_cfg->epp_fod_parm_sets = 0;
						chg_err("Read oplus,epp-fod-parm-magcvr failed, rc=%d\n", rc);
					}
				}
			}
		}

		static_cfg->fastchg_12v_fod_enable = true;
		rc = of_property_count_elems_of_size(node, "oplus,fastchg-fod-parm-12V", sizeof(u8));
		if (rc < 0) {
			static_cfg->fastchg_12v_fod_enable = false;
			chg_err("Read oplus,fastchg-fod-parm-12V size failed, rc=%d\n", rc);
		} else {
			length = rc;
			static_cfg->fastchg_fod_parm_sets = length / static_cfg->fod_parm_array_row_len;
			rc = read_fod_parm_data_from_node(node, "oplus,fastchg-fod-parm-12V",
				static_cfg->fastchg_fod_parm_12v, length, static_cfg->fod_parm_array_row_len);
			if (rc < 0) {
				static_cfg->fastchg_12v_fod_enable = false;
				chg_err("Read oplus,oplus,fastchg-fod-parm-12V failed, rc=%d\n", rc);
			}
		}

		(void)read_third_part_fod_parm_data_from_node(wls_dev, node, "oplus,wls-third-part-fastchg-fod-parm",
			static_cfg->third_part_fastchg_tx_fod_parm, &static_cfg->third_part_fastchg_fod_parm_sets);

		(void)read_third_part_fod_parm_data_from_node(wls_dev, node, "oplus,wls-third-part-fastchg-fod-parm-magcvr",
			static_cfg->third_part_fastchg_tx_fod_parm_magcvr, &static_cfg->third_part_fastchg_fod_parm_magcvr_sets);

		(void)read_third_part_fod_parm_data_from_node(wls_dev, node, "oplus,wls-third-part-epp-fod-parm",
			static_cfg->third_part_epp_tx_fod_parm, &static_cfg->third_part_epp_fod_parm_sets);

		(void)read_third_part_fod_parm_data_from_node(wls_dev, node, "oplus,wls-third-part-epp-fod-parm-magcvr",
			static_cfg->third_part_epp_tx_fod_parm_magcvr, &static_cfg->third_part_epp_fod_parm_magcvr_sets);
	}

	rc = of_property_read_u32(node, "oplus,max-voltage-mv", &dynamic_cfg->batt_vol_max_mv);
	if (rc < 0) {
		chg_err("oplus,max-voltage-mv reading failed, rc=%d\n", rc);
		dynamic_cfg->batt_vol_max_mv = 4550;
	}

	rc = of_property_read_u32(node, "oplus,fastchg_curr_max_ma", &dynamic_cfg->fastchg_curr_max_ma);
	if (rc < 0) {
		chg_err("oplus,fastchg_curr_max_ma reading failed, rc=%d\n", rc);
		dynamic_cfg->fastchg_curr_max_ma = 1500;
	}

	rc = of_property_read_u32(node, "oplus,verity_curr_max_ma", &dynamic_cfg->verity_curr_max_ma);
	if (rc < 0) {
		chg_err("oplus,verity_curr_max_ma reading failed, rc=%d\n", rc);
		dynamic_cfg->verity_curr_max_ma = dynamic_cfg->fastchg_curr_max_ma;
	}

	rc = read_fastchg_strategy_from_node(node, "wireless_fastchg_strategy", wls_dev->fcc_steps);
	if (rc < 0) {
		chg_err("Can not read wireless_fastchg_strategy data\n");
		return -EINVAL;
	}
	rc = read_fastchg_strategy_from_node(node, "wireless_fastchg_strategy_magcvr", wls_dev->fcc_steps_magcvr);
	if (rc < 0) {
		chg_info("Can not read wireless_fastchg_strategy_magcvr data\n");
		memcpy(wls_dev->fcc_steps_magcvr, wls_dev->fcc_steps, sizeof(wls_dev->fcc_steps));
	}

	rc = of_property_count_elems_of_size(node, "oplus,wls_epp_strategy", sizeof(u32));
	if (rc < 0) {
		chg_err("Count oplus,wls_epp_strategy, rc=%d\n", rc);
		length = sizeof(default_wls_epp_strategy) / sizeof(int32_t);
		memcpy(&dynamic_cfg->epp_steps.fcc_step, default_wls_epp_strategy, length * sizeof(int32_t));
	} else {
		length = rc;
		if (!check_wls_range_data_is_valid("oplus,wls_epp_strategy", length))
			return -EINVAL;
		rc = of_property_read_u32_array(node, "oplus,wls_epp_strategy",
			(u32 *)&dynamic_cfg->epp_steps.fcc_step, length);
		if (rc < 0) {
			chg_err("read oplus,wls_epp_strategy failed, rc=%d\n", rc);
			return rc;
		}
	}
	dynamic_cfg->epp_steps.max_step = length / 5;
	for (k = 0; k < dynamic_cfg->epp_steps.max_step; k++) {
		chg_debug("wls_epp_strategy[%d]%d %d %d %d %d\n", k,
			dynamic_cfg->epp_steps.fcc_step[k].low_threshold,
			dynamic_cfg->epp_steps.fcc_step[k].high_threshold,
			dynamic_cfg->epp_steps.fcc_step[k].curr_ma,
			dynamic_cfg->epp_steps.fcc_step[k].vol_max_mv,
			dynamic_cfg->epp_steps.fcc_step[k].need_wait);
	}

	rc = of_property_count_elems_of_size(node, "oplus,wls_epp_plus_strategy", sizeof(u32));
	if (rc < 0) {
		chg_err("Count oplus,wls_epp_plus_strategy, rc=%d\n", rc);
		length = sizeof(default_wls_epp_plus_strategy) / sizeof(int32_t);
		memcpy(&dynamic_cfg->epp_plus_steps.fcc_step, default_wls_epp_plus_strategy, length * (sizeof(int32_t)));
	} else {
		length = rc;
		if (!check_wls_range_data_is_valid("oplus,wls_epp_plus_strategy", length))
			return -EINVAL;
		rc = of_property_read_u32_array(node, "oplus,wls_epp_plus_strategy",
			(u32 *)&dynamic_cfg->epp_plus_steps.fcc_step, length);
		if (rc < 0) {
			chg_err("read oplus,wls_epp_plus_strategy failed, rc=%d\n", rc);
			return rc;
		}
	}
	dynamic_cfg->epp_plus_steps.max_step = length / 5;
	for (k = 0; k < dynamic_cfg->epp_plus_steps.max_step; k++) {
		chg_debug("wls_epp_plus_strategy[%d]%d %d %d %d %d\n", k,
			dynamic_cfg->epp_plus_steps.fcc_step[k].low_threshold,
			dynamic_cfg->epp_plus_steps.fcc_step[k].high_threshold,
			dynamic_cfg->epp_plus_steps.fcc_step[k].curr_ma,
			dynamic_cfg->epp_plus_steps.fcc_step[k].vol_max_mv,
			dynamic_cfg->epp_plus_steps.fcc_step[k].need_wait);
	}

	rc = of_property_count_elems_of_size(node, "oplus,wls_bpp_strategy", sizeof(u32));
	if (rc < 0) {
		chg_info("Count oplus,wls_bpp_strategy, rc=%d\n", rc);
		length = sizeof(default_wls_bpp_strategy) / sizeof(int32_t);
		memcpy(&dynamic_cfg->bpp_steps.fcc_step, default_wls_bpp_strategy, length * sizeof(int32_t));
	} else {
		length = rc;
		if (!check_wls_range_data_is_valid("oplus,wls_bpp_strategy", length))
			return -EINVAL;
		rc = of_property_read_u32_array(node, "oplus,wls_bpp_strategy",
			(u32 *)&dynamic_cfg->bpp_steps.fcc_step, length);
		if (rc < 0) {
			chg_err("read oplus,wls_bpp_strategy failed, rc=%d\n", rc);
			return rc;
		}
	}
	dynamic_cfg->bpp_steps.max_step = length / 5;
	for (k = 0; k < dynamic_cfg->bpp_steps.max_step; k++) {
		chg_debug("wls_bpp_strategy[%d]%d %d %d %d %d\n", k,
			dynamic_cfg->bpp_steps.fcc_step[k].low_threshold,
			dynamic_cfg->bpp_steps.fcc_step[k].high_threshold,
			dynamic_cfg->bpp_steps.fcc_step[k].curr_ma,
			dynamic_cfg->bpp_steps.fcc_step[k].vol_max_mv,
			dynamic_cfg->bpp_steps.fcc_step[k].need_wait);
	}

	wls_bcc_table_node = of_get_child_by_name(node, "wireless_bcc_table");
	if (!wls_bcc_table_node) {
		chg_err("Can not find wireless_bcc_table node\n");
	} else {
		rc = of_property_read_u32(node, "oplus,wls-bcc-fcc-to-icl-factor", &wls_dev->wls_bcc_fcc_to_icl_factor);
		if (rc < 0) {
			chg_err("oplus,wls-bcc-fcc-to-icl-factor reading failed, rc=%d\n", rc);
			wls_dev->wls_bcc_fcc_to_icl_factor = 2;
		}
		for (i = 0; i < WLS_BCC_SOC_MAX; i++) {
			soc_bcc_node = of_get_child_by_name(wls_bcc_table_node, bcc_soc[i]);
			if (!soc_bcc_node && ((i == WLS_BCC_SOC_0_TO_30_HIGH) || (i == WLS_BCC_SOC_30_TO_70_HIGH) ||
			    (i == WLS_BCC_SOC_70_TO_90_HIGH))) {
				memmove(&wls_dev->bcc_steps[i], &wls_dev->bcc_steps[i - 1], sizeof(struct oplus_chg_wls_bcc_steps));
				continue;
			}
			if (!soc_bcc_node)
				chg_err("Can not find %s node\n", bcc_soc[i]);
			for (j = 0; j < WLS_BCC_TEMP_MAX; j++) {
				rc = of_property_count_elems_of_size(soc_bcc_node, bcc_temp[j], sizeof(u32));
				if (rc < 0 && j == WLS_BCC_TEMP_160_TO_400_HIGH) {
					memmove(&wls_dev->bcc_steps[i].bcc_step[j], &wls_dev->bcc_steps[i].bcc_step[j - 1],
						sizeof(struct oplus_chg_wls_bcc_step));
					continue;
				}
				if (rc < 0) {
					chg_err("Count %s failed, rc=%d\n", bcc_temp[j], rc);
					return rc;
				}
				length = rc;

				if ((length / 4) > BCC_MAX_STEP_ENTRIES) {
					chg_err("too many entries(%d), only %d allowed\n", length / 4, BCC_MAX_STEP_ENTRIES);
					return -EINVAL;
				}

				rc = of_property_read_u32_array(soc_bcc_node, bcc_temp[j],
					(u32 *)wls_dev->bcc_steps[i].bcc_step[j].bcc_step, length);
				wls_dev->bcc_steps[i].bcc_step[j].max_step = length/4;
			}
		}

		for (i = 0; i < WLS_BCC_SOC_MAX; i++) {
			for (j = 0; j < WLS_BCC_TEMP_MAX; j++) {
				for (k = 0; k < wls_dev->bcc_steps[i].bcc_step[j].max_step; k++) {
					chg_debug("bcc_soc:%d, bcc_tmp:%d, bcc_step: %d %d %d %d\n",
						i, j,
						wls_dev->bcc_steps[i].bcc_step[j].bcc_step[k].max_batt_volt,
						wls_dev->bcc_steps[i].bcc_step[j].bcc_step[k].max_curr,
						wls_dev->bcc_steps[i].bcc_step[j].bcc_step[k].min_curr,
						wls_dev->bcc_steps[i].bcc_step[j].bcc_step[k].exit);
				}
			}
		}
	}

	oplus_chg_wls_bcc_stop_cur_parse_dt(wls_dev, node);

	wls_third_part_strategy_node = of_get_child_by_name(node, "wireless_fastchg_third_part_strategy");
	if (!wls_third_part_strategy_node) {
		chg_err("Can not find wls third_part_strategy node\n");
		memcpy(wls_dev->fcc_third_part_steps, wls_dev->fcc_steps, sizeof(wls_dev->fcc_steps));
	} else {
		for (i = 0; i < WLS_FAST_SOC_MAX; i++) {
			soc_strategy_node = of_get_child_by_name(wls_third_part_strategy_node, strategy_soc[i]);
			if (!soc_strategy_node) {
				chg_err("Can not find %s node\n", strategy_soc[i]);
				return -EINVAL;
			}
			for (j = 0; j < WLS_FAST_TEMP_MAX; j++) {
				rc = of_property_count_elems_of_size(soc_strategy_node, strategy_temp[j], sizeof(u32));
				if (rc < 0) {
					chg_err("Count %s failed, rc=%d\n", strategy_temp[j], rc);
					return rc;
				}
				length = rc;
				if (!check_wls_range_data_is_valid("wireless_fastchg_third_part_strategy", length))
					return -EINVAL;
				rc = of_property_read_u32_array(soc_strategy_node, strategy_temp[j],
					(u32 *)wls_dev->fcc_third_part_steps[i].fcc_step[j].fcc_step, length);
				wls_dev->fcc_third_part_steps[i].fcc_step[j].max_step = length/5;
			}
		}
	}

	/* non-ffc */
	for (i = 0; i < OPLUS_WLS_CHG_MODE_MAX; i++) {
		rc = of_property_count_elems_of_size(node, strategy_non_ffc[i], sizeof(u32));
		if (rc < 0) {
			chg_err("Count %s failed, rc=%d\n", strategy_non_ffc[i], rc);
			return rc;
		}
		length = rc;
		rc = of_property_read_u32_array(node, strategy_non_ffc[i],
			(u32 *)dynamic_cfg->non_ffc_step[i], length);
		wls_dev->non_ffc_step[i].max_step = length / 5;
	}
	for (i = 0; i < OPLUS_WLS_CHG_MODE_MAX; i++) {
		for (j = 0; j < wls_dev->non_ffc_step[i].max_step; j++) {
			chg_debug("%d %d %d %d %d\n",
				dynamic_cfg->non_ffc_step[i][j].low_threshold,
				dynamic_cfg->non_ffc_step[i][j].high_threshold,
				dynamic_cfg->non_ffc_step[i][j].curr_ma,
				dynamic_cfg->non_ffc_step[i][j].vol_max_mv,
				dynamic_cfg->non_ffc_step[i][j].need_wait);
		}
	}

	rc = read_norchg_strategy_from_node(node, "non_ffc_strategy", &dynamic_cfg->norchg_steps);
	if (rc < 0) {
		chg_err("Can not read non_ffc_strategy data\n");
		wls_dev->support_non_fcc_with_temp = false;
	} else {
		wls_dev->support_non_fcc_with_temp = true;
	}
	chg_info("support_non_fcc_with_temp = %d\n", wls_dev->support_non_fcc_with_temp);

	/* cv */
	for (i = 0; i < OPLUS_WLS_CHG_MODE_MAX; i++) {
		rc = of_property_count_elems_of_size(node, strategy_cv[i], sizeof(u32));
		if (rc < 0) {
			chg_err("Count %s failed, rc=%d\n", strategy_cv[i], rc);
			return rc;
		}
		length = rc;
		rc = of_property_read_u32_array(node, strategy_cv[i],
			(u32 *)dynamic_cfg->cv_step[i], length);
		wls_dev->cv_step[i].max_step = length / 5;
	}
	for (i = 0; i < OPLUS_WLS_CHG_MODE_MAX; i++) {
		for (j = 0; j < wls_dev->cv_step[i].max_step; j++) {
			chg_debug("%d %d %d %d %d\n",
				dynamic_cfg->cv_step[i][j].low_threshold,
				dynamic_cfg->cv_step[i][j].high_threshold,
				dynamic_cfg->cv_step[i][j].curr_ma,
				dynamic_cfg->cv_step[i][j].vol_max_mv,
				dynamic_cfg->cv_step[i][j].need_wait);
		}
	}

	for (i = 0; i < OPLUS_WLS_CHG_MODE_MAX; i++) {
		rc = of_property_count_elems_of_size(node, strategy_cv_high_batt[i], sizeof(u32));
		if (rc < 0)
			chg_err("Count %s failed, rc=%d\n", strategy_cv_high_batt[i], rc);

		length = rc;
		rc = of_property_read_u32_array(node, strategy_cv_high_batt[i],
			(u32 *)dynamic_cfg->cv_step_high_batt[i], length);
		wls_dev->cv_step_high_batt[i].max_step = length / 5;
	}

	/* cool down */
	rc = of_property_read_u32(node, "oplus,cool-down-12v-thr", &dynamic_cfg->cool_down_12v_thr);
	if (rc < 0) {
		chg_err("cool-down-12v-thr reading failed, rc=%d\n", rc);
		dynamic_cfg->cool_down_12v_thr = -EINVAL;
	}
	for (i = 0; i < OPLUS_WLS_CHG_MODE_MAX; i++) {
		rc = of_property_count_elems_of_size(node, strategy_cool_down[i], sizeof(u32));
		if (rc < 0) {
			chg_err("Count %s failed, rc=%d\n", strategy_cool_down[i], rc);
			return rc;
		}
		length = rc;
		rc = read_unsigned_data_from_node(node, strategy_cool_down[i],
			(u32 *)dynamic_cfg->cool_down_step[i], length);
		wls_dev->cool_down_step[i].max_step = length;
	}
	for (i = 0; i < OPLUS_WLS_CHG_MODE_MAX; i++) {
		for (j = 0; j < wls_dev->cool_down_step[i].max_step; j++)
			chg_debug(" %d\n", dynamic_cfg->cool_down_step[i][j]);
	}

	/* skewing */
	for (i = 0; i < OPLUS_WLS_SKEWING_MAX; i++) {
		rc = of_property_count_elems_of_size(node, strategy_skewing[i], sizeof(u32));
		if (rc < 0) {
			chg_err("Count %s failed, rc=%d\n", strategy_skewing[i], rc);
			return rc;
		}
		length = rc;
		rc = read_unsigned_data_from_node(node, strategy_skewing[i],
			(u32 *)dynamic_cfg->skewing_step[i], length);
		wls_dev->skewing_step[i].max_step = length / 3;
	}
	for (i = 0; i < OPLUS_WLS_SKEWING_MAX; i++) {
		for (j = 0; j < wls_dev->skewing_step[i].max_step; j++) {
			chg_debug("%d %d %d\n",
				dynamic_cfg->skewing_step[i][j].curr_ma,
				dynamic_cfg->skewing_step[i][j].fallback_step,
				dynamic_cfg->skewing_step[i][j].switch_to_bpp);
		}
	}

	rc = of_property_read_u32(node, "oplus,bpp-vol-mv", &dynamic_cfg->bpp_vol_mv);
	if (rc < 0) {
		chg_err("oplus,bpp-vol-mv reading failed, rc=%d\n", rc);
		dynamic_cfg->bpp_vol_mv = 5000;
	}
	rc = of_property_read_u32(node, "oplus,epp-vol-mv", &dynamic_cfg->epp_vol_mv);
	if (rc < 0) {
		chg_err("oplus,epp-vol-mv reading failed, rc=%d\n", rc);
		dynamic_cfg->epp_vol_mv = 10000;
	}
	rc = of_property_read_u32(node, "oplus,epp_plus-vol-mv", &dynamic_cfg->epp_plus_vol_mv);
	if (rc < 0) {
		chg_err("oplus,epp_plus-vol-mv reading failed, rc=%d\n", rc);
		dynamic_cfg->epp_plus_vol_mv = 10000;
	}
	rc = of_property_read_u32(node, "oplus,vooc-vol-mv", &dynamic_cfg->vooc_vol_mv);
	if (rc < 0) {
		chg_err("oplus,vooc-vol-mv reading failed, rc=%d\n", rc);
		dynamic_cfg->vooc_vol_mv = 10000;
	}
	rc = of_property_read_u32(node, "oplus,svooc-vol-mv", &dynamic_cfg->svooc_vol_mv);
	if (rc < 0) {
		chg_err("oplus,svooc-vol-mv reading failed, rc=%d\n", rc);
		dynamic_cfg->svooc_vol_mv = WLS_RX_VOL_MAX_MV;
	}
	rc = of_property_read_u32(node, "oplus,fastchg-init-vout-mv", &dynamic_cfg->fastchg_init_vout_mv);
	if (rc < 0) {
		chg_info("fastchg_init_vout_mv: use default value\n");
		dynamic_cfg->fastchg_init_vout_mv = WLS_VOUT_FASTCHG_INIT_MV;
	}
	rc = of_property_read_u32(node, "oplus,full-bridge-vout-mv", &dynamic_cfg->full_bridge_vout_mv);
	if (rc < 0) {
		chg_info("full_bridge_vout_mv: not support\n");
		dynamic_cfg->full_bridge_vout_mv = -EINVAL;
	}
	rc = of_property_read_u32(node, "oplus,cp-open-offset-mv", &dynamic_cfg->cp_open_offset_mv);
	if (rc < 0) {
		chg_info("cp_open_offset_mv: use default value\n");
		dynamic_cfg->cp_open_offset_mv = 300;
	}
	rc = of_property_read_u32(node, "oplus,cp-open-offset-min-mv", &dynamic_cfg->cp_open_offset_min_mv);
	if (rc < 0) {
		chg_info("cp_open_offset_min_mv: use default value\n");
		dynamic_cfg->cp_open_offset_min_mv = 100;
	}
	rc = of_property_read_u32(node, "oplus,fastch-wait-thr-mv", &dynamic_cfg->fastch_wait_thr_mv);
	if (rc < 0) {
		chg_info("fastch_wait_thr_mv: use default value\n");
		dynamic_cfg->fastch_wait_thr_mv = 5000;
	}

	oplus_chg_wls_iclmax_parse_dt(wls_dev, node, "oplus,iclmax-ma", OPLUS_WLS_CHG_BATT_CL_LOW);
	oplus_chg_wls_iclmax_parse_dt(wls_dev, node, "oplus,iclmax-batt-high-ma", OPLUS_WLS_CHG_BATT_CL_HIGH);

	skin_step = &wls_dev->epp_plus_skin_step;
	skin_step->skin_step = dynamic_cfg->epp_plus_skin_step;
	rc = read_skin_range_data_from_node(node, "oplus,epp_plus-skin-step", skin_step->skin_step,
		WLS_SKIN_TEMP_MAX, wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_EPP_PLUS]);
	if (rc < 0) {
		for (i = 0; i < WLS_MAX_STEP_CHG_ENTRIES; i++) {
			skin_step->skin_step[i].low_threshold = default_config.epp_plus_skin_step[i].low_threshold;
			skin_step->skin_step[i].high_threshold = default_config.epp_plus_skin_step[i].high_threshold;
			skin_step->skin_step[i].curr_ma = default_config.epp_plus_skin_step[i].curr_ma;
			if ((skin_step->skin_step[i].low_threshold == 0) &&
			    (skin_step->skin_step[i].high_threshold == 0) &&
			    (skin_step->skin_step[i].curr_ma == 0)) {
				skin_step->max_step = i;
				break;
			}
		}
	} else {
		skin_step->max_step = rc;
	}
	for (i = 0; i < skin_step->max_step; i++) {
		chg_info("epp_plus-skin-step: %d %d %d\n", skin_step->skin_step[i].low_threshold,
			skin_step->skin_step[i].high_threshold, skin_step->skin_step[i].curr_ma);
	}

	skin_step = &wls_dev->epp_skin_step;
	skin_step->skin_step = dynamic_cfg->epp_skin_step;
	rc = read_skin_range_data_from_node(node, "oplus,epp-skin-step", skin_step->skin_step,
		WLS_SKIN_TEMP_MAX, wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_EPP]);
	if (rc < 0) {
		for (i = 0; i < WLS_MAX_STEP_CHG_ENTRIES; i++) {
			skin_step->skin_step[i].low_threshold = default_config.epp_skin_step[i].low_threshold;
			skin_step->skin_step[i].high_threshold = default_config.epp_skin_step[i].high_threshold;
			skin_step->skin_step[i].curr_ma = default_config.epp_skin_step[i].curr_ma;
			if ((skin_step->skin_step[i].low_threshold == 0) &&
			    (skin_step->skin_step[i].high_threshold == 0) &&
			    (skin_step->skin_step[i].curr_ma == 0)) {
				skin_step->max_step = i;
				break;
			}
		}
	} else {
		skin_step->max_step = rc;
	}
	for (i = 0; i < skin_step->max_step; i++) {
		chg_debug("epp-skin-step: %d %d %d\n", skin_step->skin_step[i].low_threshold,
			skin_step->skin_step[i].high_threshold, skin_step->skin_step[i].curr_ma);
	}

	skin_step = &wls_dev->bpp_skin_step;
	skin_step->skin_step = dynamic_cfg->bpp_skin_step;
	rc = read_skin_range_data_from_node(node, "oplus,bpp-skin-step", skin_step->skin_step,
		WLS_SKIN_TEMP_MAX, wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_BPP]);
	if (rc < 0) {
		for (i = 0; i < WLS_MAX_STEP_CHG_ENTRIES; i++) {
			skin_step->skin_step[i].low_threshold =
				default_config.bpp_skin_step[i].low_threshold;
			skin_step->skin_step[i].high_threshold =
				default_config.bpp_skin_step[i].high_threshold;
			skin_step->skin_step[i].curr_ma =
				default_config.bpp_skin_step[i].curr_ma;
			if ((skin_step->skin_step[i].low_threshold == 0) &&
			    (skin_step->skin_step[i].high_threshold == 0) &&
			    (skin_step->skin_step[i].curr_ma == 0)) {
				skin_step->max_step = i;
				break;
			}
		}
	} else {
		skin_step->max_step = rc;
	}
	for (i = 0; i < skin_step->max_step; i++) {
		chg_info("bpp-skin-step: %d %d %d\n", skin_step->skin_step[i].low_threshold,
			skin_step->skin_step[i].high_threshold, skin_step->skin_step[i].curr_ma);
	}

	skin_step = &wls_dev->epp_plus_led_on_skin_step;
	skin_step->skin_step = dynamic_cfg->epp_plus_led_on_skin_step;
	rc = read_skin_range_data_from_node(node, "oplus,epp_plus-led-on-skin-step", skin_step->skin_step,
		WLS_SKIN_TEMP_MAX, wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_EPP_PLUS]);
	if (rc < 0) {
		for (i = 0; i < WLS_MAX_STEP_CHG_ENTRIES; i++) {
			skin_step->skin_step[i].low_threshold = default_config.epp_plus_led_on_skin_step[i].low_threshold;
			skin_step->skin_step[i].high_threshold = default_config.epp_plus_led_on_skin_step[i].high_threshold;
			skin_step->skin_step[i].curr_ma = default_config.epp_plus_led_on_skin_step[i].curr_ma;
			if ((skin_step->skin_step[i].low_threshold == 0) &&
			    (skin_step->skin_step[i].high_threshold == 0) &&
			    (skin_step->skin_step[i].curr_ma == 0)) {
				skin_step->max_step = i;
				break;
			}
		}
	} else {
		skin_step->max_step = rc;
	}
	for (i = 0; i < skin_step->max_step; i++) {
		chg_info("epp_plus-led-on-skin-step: %d %d %d\n", skin_step->skin_step[i].low_threshold,
			skin_step->skin_step[i].high_threshold, skin_step->skin_step[i].curr_ma);
	}

	skin_step = &wls_dev->epp_led_on_skin_step;
	skin_step->skin_step = dynamic_cfg->epp_led_on_skin_step;
	rc = read_skin_range_data_from_node(node, "oplus,epp-led-on-skin-step", skin_step->skin_step,
		WLS_SKIN_TEMP_MAX, wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_EPP]);
	if (rc < 0) {
		for (i = 0; i < WLS_MAX_STEP_CHG_ENTRIES; i++) {
			skin_step->skin_step[i].low_threshold = default_config.epp_led_on_skin_step[i].low_threshold;
			skin_step->skin_step[i].high_threshold = default_config.epp_led_on_skin_step[i].high_threshold;
			skin_step->skin_step[i].curr_ma = default_config.epp_led_on_skin_step[i].curr_ma;
			if ((skin_step->skin_step[i].low_threshold == 0) &&
			    (skin_step->skin_step[i].high_threshold == 0) &&
			    (skin_step->skin_step[i].curr_ma == 0)) {
				skin_step->max_step = i;
				break;
			}
		}
	} else {
		skin_step->max_step = rc;
	}
	for (i = 0; i < skin_step->max_step; i++) {
		chg_info("epp-led-on-skin-step: %d %d %d\n", skin_step->skin_step[i].low_threshold,
			skin_step->skin_step[i].high_threshold, skin_step->skin_step[i].curr_ma);
	}

	chg_info("icl_max_ma BPP EPP EPP_PLUS AIRVOOC AIRSVOOC: %d %d %d %d %d\n",
		wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_BPP],
		wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_EPP],
		wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_EPP_PLUS],
		wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_AIRVOOC],
		wls_dev->icl_max_ma[OPLUS_WLS_CHG_MODE_AIRSVOOC]);

	rc = of_property_read_u32(node, "oplus,wls-fast-chg-call-on-curr-ma",
		&dynamic_cfg->wls_fast_chg_call_on_curr_ma);
	if (rc < 0) {
		chg_err("oplus,oplus,wls-fast-chg-call-on-curr-ma reading failed, rc=%d\n", rc);
		dynamic_cfg->wls_fast_chg_call_on_curr_ma = 600;
	}
	rc = of_property_read_u32(node, "oplus,wls-fast-chg-camera-on-curr-ma",
		&dynamic_cfg->wls_fast_chg_camera_on_curr_ma);
	if (rc < 0) {
		chg_err("oplus,oplus,wls-fast-chg-camera-on-curr-ma reading failed, rc=%d\n", rc);
		dynamic_cfg->wls_fast_chg_camera_on_curr_ma = 600;
	}

	rc = of_property_read_u32(node, "oplus,fastchg-max-soc", &dynamic_cfg->fastchg_max_soc);
	if (rc < 0) {
		chg_err("oplus,fastchg-max-soc reading failed, rc=%d\n", rc);
		dynamic_cfg->fastchg_max_soc = 90;
	}

	rc = of_property_read_u32(node, "oplus,fastchg-max-temp", &dynamic_cfg->fastchg_max_temp);
	if (rc < 0) {
		chg_err("oplus,fastchg-max-temp reading failed, rc=%d\n", rc);
		dynamic_cfg->fastchg_max_temp = 440;
	}

	oplus_chg_wls_strategy_soc_parse_dt(wls_dev, node);

	cnt = of_property_count_elems_of_size(node, "oplus,fastchg-max-vbat", sizeof(u32));
	if (cnt == BATT_TEMP_MAX || cnt == BATT_TEMP_MAX - 1) {
		for (i = 0; i < BATT_TEMP_MAX; i++) {
			j = (cnt == BATT_TEMP_MAX - 1 && i >= WLS_BCC_TEMP_160_TO_400_HIGH) ? i - 1 : i;
			rc = of_property_read_u32_index(node, "oplus,fastchg-max-vbat", j,
			     &dynamic_cfg->fastch_max_vbat_mv[i]);
			if (rc < 0)
				break;
		}
	}
	if (cnt < 0 || rc < 0) {
		chg_info("use default oplus,fastchg-max-vbat value\n");
		for (i = 0; i < BATT_TEMP_MAX; i++)
			dynamic_cfg->fastch_max_vbat_mv[i] = 5000;
	} else {
		for (i = 0; i < BATT_TEMP_MAX; i++)
			chg_info("fastchg-max-vbat: %d", dynamic_cfg->fastch_max_vbat_mv[i]);
	}

	rc = of_property_read_u32(node, "oplus,epp-high-temp-force-bpp-thr", &dynamic_cfg->epp_high_temp_thr);
	if (rc < 0) {
		chg_err("oplus,epp-high-temp-force-bpp-thr reading failed, rc=%d\n", rc);
		dynamic_cfg->epp_high_temp_thr = -EINVAL;
	}

	oplus_chg_wls_fan_pulse_parse_dt(wls_dev, node);

	rc = of_property_read_u32(node, "oplus,boot_quiet_time", &dynamic_cfg->boot_quiet_t);
	if (rc < 0) {
		chg_info("oplus,boot_quiet_time reading failed, rc=%d\n", rc);
		dynamic_cfg->boot_quiet_t = 0;
	} else {
		chg_info("boot_quiet_time: %d\n", dynamic_cfg->boot_quiet_t);
	}

	rc = of_property_read_u32(node, "oplus,camera_icl_limit", &dynamic_cfg->camera_icl_limit);
	if (rc < 0) {
		chg_info("oplus,camera_icl_limit reading failed, rc=%d\n", rc);
		dynamic_cfg->camera_icl_limit = 0;
	} else {
		chg_info("camera_icl_limit: %d\n", dynamic_cfg->camera_icl_limit);
	}

	rc = of_property_read_u32(node, "oplus,camera_handle_icl_limit", &dynamic_cfg->camera_handle_icl_limit);
	if (rc < 0) {
		chg_info("oplus,camera_handle_icl_limit reading failed, rc=%d\n", rc);
		dynamic_cfg->camera_handle_icl_limit = 0;
	} else {
		chg_info("camera_handle_icl_limit: %d\n", dynamic_cfg->camera_handle_icl_limit);
	}

	return 0;
}

static int oplus_chg_wls_gpio_init(struct oplus_chg_wls *wls_dev)
{
	int rc = 0;
	struct device_node *node = oplus_get_node_by_type(wls_dev->dev->of_node);

	wls_dev->pinctrl = devm_pinctrl_get(wls_dev->dev);
	if (IS_ERR_OR_NULL(wls_dev->pinctrl)) {
		chg_info("not support pinctrl\n");
		return 0;
	}

	wls_dev->wrx_en_gpio = of_get_named_gpio(node, "oplus,wrx_en-gpio", 0);
	if (gpio_is_valid(wls_dev->wrx_en_gpio)) {
		rc = gpio_request(wls_dev->wrx_en_gpio, "wrx_en-gpio");
		if (rc < 0) {
			chg_err("wrx_en_gpio request error, rc=%d\n", rc);
			return rc;
		}
		wls_dev->wrx_en_active = pinctrl_lookup_state(wls_dev->pinctrl, "wrx_en_active");
		if (IS_ERR_OR_NULL(wls_dev->wrx_en_active)) {
			chg_err("get wrx_en_active fail\n");
			goto free_wrx_en_gpio;
		}
		wls_dev->wrx_en_sleep = pinctrl_lookup_state(wls_dev->pinctrl, "wrx_en_sleep");
		if (IS_ERR_OR_NULL(wls_dev->wrx_en_sleep)) {
			chg_err("get wrx_en_sleep fail\n");
			goto free_wrx_en_gpio;
		}
		gpio_direction_output(wls_dev->wrx_en_gpio, 0);
		pinctrl_select_state(wls_dev->pinctrl, wls_dev->wrx_en_sleep);
	}

	if (!wls_dev->support_fastchg)
		return 0;

	wls_dev->ext_pwr_en_gpio = of_get_named_gpio(node, "oplus,ext_pwr_en-gpio", 0);
	if (gpio_is_valid(wls_dev->ext_pwr_en_gpio)) {
		rc = gpio_request(wls_dev->ext_pwr_en_gpio, "ext_pwr_en-gpio");
		if (rc < 0) {
			chg_err("ext_pwr_en_gpio request error, rc=%d\n", rc);
			goto free_wrx_en_gpio;
		}
		wls_dev->ext_pwr_en_active = pinctrl_lookup_state(wls_dev->pinctrl, "ext_pwr_en_active");
		if (IS_ERR_OR_NULL(wls_dev->ext_pwr_en_active)) {
			chg_err("get ext_pwr_en_active fail\n");
			goto free_ext_pwr_en_gpio;
		}
		wls_dev->ext_pwr_en_sleep = pinctrl_lookup_state(wls_dev->pinctrl, "ext_pwr_en_sleep");
		if (IS_ERR_OR_NULL(wls_dev->ext_pwr_en_sleep)) {
			chg_err("get ext_pwr_en_sleep fail\n");
			goto free_ext_pwr_en_gpio;
		}
		gpio_direction_output(wls_dev->ext_pwr_en_gpio, 0);
		pinctrl_select_state(wls_dev->pinctrl, wls_dev->ext_pwr_en_sleep);
	}

	wls_dev->usb_int_gpio = of_get_named_gpio(node, "oplus,usb_int-gpio", 0);
	if (!gpio_is_valid(wls_dev->usb_int_gpio)) {
		chg_err("usb_int_gpio not specified\n");
		goto out;
	}
	rc = gpio_request(wls_dev->usb_int_gpio, "usb_int-gpio");
	if (rc < 0) {
		chg_err("usb_int_gpio request error, rc=%d\n", rc);
		goto free_ext_pwr_en_gpio;
	}
	gpio_direction_input(wls_dev->usb_int_gpio);
	wls_dev->usb_int_irq = gpio_to_irq(wls_dev->usb_int_gpio);
	rc = devm_request_irq(wls_dev->dev, wls_dev->usb_int_irq,
			      oplus_chg_wls_usb_int_handler,
			      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			      "usb_int_irq", wls_dev);
	if (rc < 0) {
		chg_err("usb_int_irq request error, rc=%d\n", rc);
		goto free_int_gpio;
	}
	enable_irq_wake(wls_dev->usb_int_irq);

out:
	return 0;

free_int_gpio:
	if (gpio_is_valid(wls_dev->usb_int_gpio))
		gpio_free(wls_dev->usb_int_gpio);
free_ext_pwr_en_gpio:
	if (gpio_is_valid(wls_dev->ext_pwr_en_gpio))
		gpio_free(wls_dev->ext_pwr_en_gpio);
free_wrx_en_gpio:
	if (gpio_is_valid(wls_dev->wrx_en_gpio))
		gpio_free(wls_dev->wrx_en_gpio);
	return rc;
}

static ssize_t oplus_chg_wls_proc_deviated_read(struct file *file,
						char __user *buf, size_t count,
						loff_t *ppos)
{
	uint8_t ret = 0;
	char page[7];
	size_t len = 7;

	memset(page, 0, 7);
	len = snprintf(page, len, "%s\n", "false");
	ret = simple_read_from_buffer(buf, count, ppos, page, len);

	return ret;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_chg_wls_proc_deviated_ops =
{
	.read = oplus_chg_wls_proc_deviated_read,
	.open  = simple_open,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_chg_wls_proc_deviated_ops = {
	.proc_read = oplus_chg_wls_proc_deviated_read,
	.proc_write = NULL,
	.proc_open = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_chg_wls_proc_tx_read(struct file *file, char __user *buf,
					  size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[10] = { 0 };
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));
	union mms_msg_data data = { 0 };

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	if (wls_dev->wls_status.trx_online)
		data.intval = OPLUS_CHG_WLS_TRX_STATUS_CHARGING;
	else if (wls_dev->wls_status.trx_present || wls_dev->wls_status.trx_present_keep)
		data.intval = OPLUS_CHG_WLS_TRX_STATUS_ENABLE;
	else
		data.intval = OPLUS_CHG_WLS_TRX_STATUS_DISENABLE;

	/*
	*When the wireless reverse charging error occurs,
	*the upper layer turns off the reverse charging button.
	*/
	if (wls_dev->wls_status.trx_close_delay) {
		data.intval = OPLUS_CHG_WLS_TRX_STATUS_ENABLE;
		schedule_delayed_work(&wls_dev->wls_clear_trx_work, msecs_to_jiffies(3000));
	}

	switch (data.intval) {
	case OPLUS_CHG_WLS_TRX_STATUS_ENABLE:
		snprintf(page, 10, "%s\n", "enable");
		break;
	case OPLUS_CHG_WLS_TRX_STATUS_CHARGING:
		snprintf(page, 10, "%s\n", "charging");
		break;
	case OPLUS_CHG_WLS_TRX_STATUS_DISENABLE:
	default:
		snprintf(page, 10, "%s\n", "disable");
		break;
	}

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t oplus_chg_wls_proc_tx_write(struct file *file,
					   const char __user *buf, size_t count,
					   loff_t *lo)
{
	char buffer[5] = { 0 };
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));
	int val;
	int rc;

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	if (count > sizeof(buffer) - 1)
		return -EFAULT;

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: error.\n", __func__);
		return -EFAULT;
	}
	buffer[count] = '\0';
	chg_info("buffer=%s\n", buffer);
	rc = kstrtoint(strstrip(buffer), 0, &val);
	if (rc != 0)
		return -EINVAL;
	chg_info("val = %d\n", val);

	/*
	*When the wireless reverse charging error occurs,
	*the upper layer turns off the reverse charging button.
	*/
	wls_dev->wls_status.trx_close_delay = false;

	mutex_lock(&wls_dev->trx_enable_lock);
	rc = oplus_chg_wls_set_trx_enable(wls_dev, !!val);
	mutex_unlock(&wls_dev->trx_enable_lock);
	if (rc < 0) {
		chg_err("can't enable trx, rc=%d\n", rc);
		return (ssize_t)rc;
	}
	return count;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_chg_wls_proc_tx_ops =
{
	.read = oplus_chg_wls_proc_tx_read,
	.write = oplus_chg_wls_proc_tx_write,
	.open  = simple_open,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_chg_wls_proc_tx_ops = {
	.proc_read = oplus_chg_wls_proc_tx_read,
	.proc_write = oplus_chg_wls_proc_tx_write,
	.proc_open = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_chg_wls_proc_rx_read(struct file *file, char __user *buf,
					  size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[3];
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	memset(page, 0, 3);
	snprintf(page, 3, "%c\n", wls_dev->charge_enable ? '1' : '0');
	ret = simple_read_from_buffer(buf, count, ppos, page, 3);
	return ret;
}

static ssize_t oplus_chg_wls_proc_rx_write(struct file *file,
					   const char __user *buf, size_t count,
					   loff_t *lo)
{
	char buffer[5] = { 0 };
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));
	int val;
	int rc;

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	if (count > sizeof(buffer) - 1)
		return -EFAULT;

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: error.\n", __func__);
		return -EFAULT;
	}
	buffer[count] = '\0';
	chg_info("buffer=%s", buffer);
	rc = kstrtoint(strstrip(buffer), 0, &val);
	if (rc != 0)
		return -EINVAL;
	chg_info("val = %d", val);

	wls_dev->charge_enable = !!val;
	vote(wls_dev->rx_disable_votable, DEBUG_VOTER, !wls_dev->charge_enable,
	     1, false);
	chg_info("%s wls rx\n", wls_dev->charge_enable ? "enable" : "disable");
	return count;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_chg_wls_proc_rx_ops =
{
	.read = oplus_chg_wls_proc_rx_read,
	.write = oplus_chg_wls_proc_rx_write,
	.open  = simple_open,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_chg_wls_proc_rx_ops = {
	.proc_read = oplus_chg_wls_proc_rx_read,
	.proc_write = oplus_chg_wls_proc_rx_write,
	.proc_open = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_chg_wls_proc_user_sleep_mode_read(struct file *file,
						       char __user *buf,
						       size_t count,
						       loff_t *ppos)
{
	uint8_t ret = 0;
	char page[10];
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	chg_info("quiet_mode_need = %d.\n", wls_dev->wls_status.quiet_mode_need);
	sprintf(page, "%d", wls_dev->wls_status.quiet_mode_need);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t oplus_chg_wls_proc_user_sleep_mode_write(struct file *file,
							const char __user *buf,
							size_t len, loff_t *lo)
{
	char buffer[5] = { 0 };
	int pmw_pulse = 0;
	int rc = -1;
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));
	/*union oplus_chg_mod_propval pval;
	union mms_msg_data data = { 0 };*/

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	if (len > sizeof(buffer) - 1) {
		chg_err("len[%lu] -EFAULT\n", len);
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, len)) {
		chg_err("copy from user error\n");
		return -EFAULT;
	}
	buffer[len] = '\0';
	chg_info("user mode: buffer=%s\n", buffer);
	rc = kstrtoint(strstrip(buffer), 0, &pmw_pulse);
	if (rc != 0)
		return -EINVAL;
	if (pmw_pulse == WLS_FASTCHG_MODE) {
		if (!wls_dev->wls_status.rx_present ||
		    wls_dev->wls_status.adapter_type == WLS_ADAPTER_TYPE_USB ||
		    wls_dev->wls_status.adapter_type == WLS_ADAPTER_TYPE_NORMAL ||
		    wls_dev->wls_status.adapter_type == WLS_ADAPTER_TYPE_UNKNOWN) {
			chg_err("wls not present or isn't op_tx, can't enter quiet mode!\n");
		} else {
			vote(wls_dev->quiet_mode_votable, WLS_QUIET_MODE_VOTER, false, 0, false);
			wls_dev->wls_status.quiet_mode_need = WLS_FASTCHG_MODE;
		}

		chg_info("set user mode: %d, switch_quiet_mode: %d fastchg mode, rc: %d\n", pmw_pulse,
		       wls_dev->wls_status.switch_quiet_mode, rc);
	} else if (pmw_pulse == WLS_SILENT_MODE) {
		if (!wls_dev->wls_status.rx_present ||
		    wls_dev->wls_status.adapter_type == WLS_ADAPTER_TYPE_USB ||
		    wls_dev->wls_status.adapter_type == WLS_ADAPTER_TYPE_NORMAL ||
		    wls_dev->wls_status.adapter_type == WLS_ADAPTER_TYPE_UNKNOWN) {
			rc = -EINVAL;
			chg_err("wls not present or isn't op_tx, can't enter quiet mode!\n");
		} else {
			vote(wls_dev->quiet_mode_votable, WLS_QUIET_MODE_VOTER, true, 0, false);
			wls_dev->wls_status.quiet_mode_need = WLS_SILENT_MODE;
		}

		chg_info("set user mode: %d, switch_quiet_mode: %d silent mode, rc: %d\n", pmw_pulse,
		       wls_dev->wls_status.switch_quiet_mode, rc);
		/*nu1619_set_dock_led_pwm_pulse(3);*/
	} else if (pmw_pulse == WLS_BATTERY_FULL_MODE) {
		chg_info("set user mode: %d, battery full mode\n", pmw_pulse);
		wls_dev->wls_status.quiet_mode_need = WLS_BATTERY_FULL_MODE;
		/*nu1619_set_dock_fan_pwm_pulse(60);*/
	} else if (pmw_pulse == WLS_CALL_MODE) {
		chg_info("set user mode: %d, call mode\n", pmw_pulse);
		/*chip->nu1619_chg_status.call_mode = true;*/
	} else if (pmw_pulse == WLS_EXIT_CALL_MODE) {
		/*chip->nu1619_chg_status.call_mode = false;*/
		chg_info("set user mode: %d, exit call mode\n", pmw_pulse);
	} else if (pmw_pulse == WLS_EXIT_AUDIO_MODE && wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30) {
		wls_dev->wls_status.track_record.pmw_scenarios = pmw_pulse;
		vote(wls_dev->quiet_mode_votable, WLS_AUDIO_MODE_VOTER, false, 0, false);
		chg_info("set user mode: %d, exit audio mode\n", pmw_pulse);
	} else if (pmw_pulse == WLS_AUDIO_MODE && wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30) {
		wls_dev->wls_status.track_record.pmw_scenarios = pmw_pulse;
		vote(wls_dev->quiet_mode_votable, WLS_AUDIO_MODE_VOTER, true, 0, false);
		chg_info("set user mode: %d, call audio mode\n", pmw_pulse);
	} else if (pmw_pulse == WLS_EXIT_CAMERA_MODE && wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30) {
		wls_dev->wls_status.track_record.pmw_scenarios = pmw_pulse;
		if (wls_dev->force_rx_mode != OPLUS_CHG_WLS_RX_MODE_UNKNOWN) {
			wls_dev->wls_status.track_record.break_manu = true;
			vote(wls_dev->force_bpp_mode_votable, WLS_CAMERA_MODE_VOTER, false, 0, false);
			cancel_delayed_work(&wls_dev->rx_mode_check_work);
			chg_info("set user mode: %d, exit camera mode\n", pmw_pulse);
		}
		vote(wls_dev->nor_icl_votable, WLS_CAMERA_MODE_VOTER, false, 0, false);
	} else if (pmw_pulse == WLS_CAMERA_MODE && wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30) {
		wls_dev->wls_status.track_record.pmw_scenarios = pmw_pulse;
		if (wls_dev->mms_info.rx_present == 1 &&
		    wls_dev->wls_status.wls_type == OPLUS_CHG_WLS_BPP)
			oplus_chg_wls_cam_scene_icl_limit(wls_dev);

		if (wls_dev->force_rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
			cancel_delayed_work(&wls_dev->rx_mode_check_work);
			schedule_delayed_work(&wls_dev->rx_mode_check_work,
					      msecs_to_jiffies(OPLUS_CHG_WLS_RX_MODE_CHECK_DELAY));
		} else {
			wls_dev->wls_status.track_record.break_manu = true;
			vote(wls_dev->force_bpp_mode_votable, WLS_CAMERA_MODE_VOTER, true, 1, false);
			schedule_delayed_work(&wls_dev->rx_mode_check_work,
					      msecs_to_jiffies(OPLUS_CHG_WLS_RX_MODE_CHECK_DELAY));
		}
		chg_info("set user mode: %d, call camera mode\n", pmw_pulse);
	} else {
		chg_info("user sleep mode: pmw_pulse: %d\n", pmw_pulse);
		wls_dev->wls_status.quiet_mode_need = pmw_pulse;
		/*nu1619_set_dock_fan_pwm_pulse(pmw_pulse);*/
	}

	return len;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_chg_wls_proc_user_sleep_mode_ops =
{
	.read = oplus_chg_wls_proc_user_sleep_mode_read,
	.write = oplus_chg_wls_proc_user_sleep_mode_write,
	.open  = simple_open,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_chg_wls_proc_user_sleep_mode_ops = {
	.proc_read = oplus_chg_wls_proc_user_sleep_mode_read,
	.proc_write = oplus_chg_wls_proc_user_sleep_mode_write,
	.proc_open = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_chg_wls_proc_idt_adc_test_read(struct file *file,
						    char __user *buf,
						    size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[10];
	int rx_adc_result = 0;
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	if (wls_dev->wls_status.rx_adc_test_pass == true) {
		rx_adc_result = 1;
	} else {
		rx_adc_result = 0;
	}
	rx_adc_result = 1; /*needn't test*/
	/*chg_info("rx_adc_test: result %d.\n", rx_adc_result);*/
	sprintf(page, "%d", rx_adc_result);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t oplus_chg_wls_proc_idt_adc_test_write(struct file *file,
						     const char __user *buf,
						     size_t len, loff_t *lo)
{
	char buffer[5] = { 0 };
	int rx_adc_cmd = 0;
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));
	int rc;

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	if (len > sizeof(buffer) - 1) {
		chg_err("%s: len[%lu] -EFAULT.\n", __func__, len);
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, len)) {
		chg_err("%s:  error.\n", __func__);
		return -EFAULT;
	}
	buffer[len] = '\0';
	rc = kstrtoint(strstrip(buffer), 0, &rx_adc_cmd);
	if (rc != 0)
		return -EINVAL;
	if (rx_adc_cmd == 0) {
		chg_info("rx_adc_test: set 0.\n");
		wls_dev->wls_status.rx_adc_test_enable = false;
	} else if (rx_adc_cmd == 1) {
		chg_info("rx_adc_test: set 1.\n");
		wls_dev->wls_status.rx_adc_test_enable = true;
	} else {
		chg_info("rx_adc_test: set %d, invalid.\n", rx_adc_cmd);
	}

	return len;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_chg_wls_proc_idt_adc_test_ops =
{
	.read = oplus_chg_wls_proc_idt_adc_test_read,
	.write = oplus_chg_wls_proc_idt_adc_test_write,
	.open  = simple_open,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_chg_wls_proc_idt_adc_test_ops = {
	.proc_read = oplus_chg_wls_proc_idt_adc_test_read,
	.proc_write = oplus_chg_wls_proc_idt_adc_test_write,
	.proc_open = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_chg_wls_proc_rx_power_read(struct file *file,
						char __user *buf, size_t count,
						loff_t *ppos)
{
	uint8_t ret = 0;
	char page[16] = { 0 };
	int rx_power = 0;
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	rx_power = wls_dev->wls_status.rx_pwr_mw;
	if (wls_dev->wls_status.trx_online)
		rx_power = 1563;

	/*chg_info("rx_power = %d\n", rx_power);*/
	sprintf(page, "%d\n", rx_power);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t oplus_chg_wls_proc_rx_power_write(struct file *file,
						 const char __user *buf,
						 size_t count, loff_t *lo)
{
	return count;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_chg_wls_proc_rx_power_ops =
{
	.read = oplus_chg_wls_proc_rx_power_read,
	.write = oplus_chg_wls_proc_rx_power_write,
	.open  = simple_open,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_chg_wls_proc_rx_power_ops = {
	.proc_read = oplus_chg_wls_proc_rx_power_read,
	.proc_write = oplus_chg_wls_proc_rx_power_write,
	.proc_open = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_chg_wls_proc_tx_power_read(struct file *file,
						char __user *buf, size_t count,
						loff_t *ppos)
{
	uint8_t ret = 0;
	char page[16] = { 0 };
	int tx_power = 0;
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	tx_power = wls_dev->wls_status.tx_pwr_mw;
	if (wls_dev->wls_status.trx_online)
		tx_power = wls_dev->wls_status.trx_vol_mv * wls_dev->wls_status.trx_curr_ma / 1000;

	/*chg_info("tx_power = %d\n", tx_power);*/
	sprintf(page, "%d\n", tx_power);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t oplus_chg_wls_proc_tx_power_write(struct file *file,
						 const char __user *buf,
						 size_t count, loff_t *lo)
{
	return count;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_chg_wls_proc_tx_power_ops =
{
	.read = oplus_chg_wls_proc_tx_power_read,
	.write = oplus_chg_wls_proc_tx_power_write,
	.open  = simple_open,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_chg_wls_proc_tx_power_ops = {
	.proc_read = oplus_chg_wls_proc_tx_power_read,
	.proc_write = oplus_chg_wls_proc_tx_power_write,
	.proc_open = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_chg_wls_proc_rx_version_read(struct file *file,
						  char __user *buf,
						  size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[16] = { 0 };
	u32 rx_version = 0;
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));
	int rc;

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	rc = oplus_chg_wls_rx_get_rx_version(wls_dev->wls_rx->rx_ic, &rx_version);
	if (rc < 0) {
		chg_err("can't get rx version, rc=%d\n", rc);
		return rc;
	}

	/*chg_info("rx_version = 0x%x\n", rx_version);*/
	sprintf(page, "0x%x\n", rx_version);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t oplus_chg_wls_proc_rx_version_write(struct file *file,
						   const char __user *buf,
						   size_t count, loff_t *lo)
{
	return count;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_chg_wls_proc_rx_version_ops =
{
	.read = oplus_chg_wls_proc_rx_version_read,
	.write = oplus_chg_wls_proc_rx_version_write,
	.open  = simple_open,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_chg_wls_proc_rx_version_ops = {
	.proc_read = oplus_chg_wls_proc_rx_version_read,
	.proc_write = oplus_chg_wls_proc_rx_version_write,
	.proc_open = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_chg_wls_proc_tx_version_read(struct file *file,
						  char __user *buf,
						  size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[16] = { 0 };
	u32 tx_version = 0;
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));
	int rc;

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	rc = oplus_chg_wls_rx_get_trx_version(wls_dev->wls_rx->rx_ic, &tx_version);
	if (rc < 0) {
		chg_err("can't get tx version, rc=%d\n", rc);
		return rc;
	}

	/*chg_info("tx_version = 0x%x\n", tx_version);*/
	sprintf(page, "0x%x\n", tx_version);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t oplus_chg_wls_proc_tx_version_write(struct file *file,
						   const char __user *buf,
						   size_t count, loff_t *lo)
{
	return count;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_chg_wls_proc_tx_version_ops =
{
	.read = oplus_chg_wls_proc_tx_version_read,
	.write = oplus_chg_wls_proc_tx_version_write,
	.open  = simple_open,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_chg_wls_proc_tx_version_ops = {
	.proc_read = oplus_chg_wls_proc_tx_version_read,
	.proc_write = oplus_chg_wls_proc_tx_version_write,
	.proc_open = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t oplus_chg_wls_proc_ftm_mode_read(struct file *file,
						char __user *buf, size_t count,
						loff_t *ppos)
{
	uint8_t ret = 0;
	char page[256];
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	sprintf(page, "ftm_mode[%d], engineering_mode[%d]\n", wls_dev->ftm_mode,
		wls_dev->factory_mode);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

#define FTM_MODE_DISABLE 0
#define FTM_MODE_ENABLE 1
#define ENGINEERING_MODE_ENABLE 2
#define ENGINEERING_MODE_DISABLE 3
static ssize_t oplus_chg_wls_proc_ftm_mode_write(struct file *file,
						 const char __user *buf,
						 size_t len, loff_t *lo)
{
	char buffer[4] = { 0 };
	int ftm_mode = 0;
	struct oplus_chg_wls *wls_dev = pde_data(file_inode(file));
	int rc;

	if (wls_dev == NULL) {
		chg_err("wls dev is not fount\n");
		return -ENODEV;
	}

	if (len > sizeof(buffer) - 1) {
		chg_err("len[%lu] -EFAULT\n", len);
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, len)) {
		chg_err("copy from user error\n");
		return -EFAULT;
	}
	buffer[len] = '\0';
	chg_info("ftm mode: buffer=%s\n", buffer);
	rc = kstrtoint(strstrip(buffer), 0, &ftm_mode);
	if (rc != 0)
		return -EINVAL;
	if (ftm_mode == FTM_MODE_DISABLE) {
		wls_dev->ftm_mode = false;
	} else if (ftm_mode == FTM_MODE_ENABLE) {
		wls_dev->ftm_mode = true;
	} else if (ftm_mode == ENGINEERING_MODE_ENABLE) {
		wls_dev->factory_mode = true;
	} else if (ftm_mode == ENGINEERING_MODE_DISABLE) {
		wls_dev->factory_mode = false;
	}

	return len;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations oplus_chg_wls_proc_ftm_mode_ops =
{
	.read = oplus_chg_wls_proc_ftm_mode_read,
	.write = oplus_chg_wls_proc_ftm_mode_write,
	.open  = simple_open,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops oplus_chg_wls_proc_ftm_mode_ops = {
	.proc_read = oplus_chg_wls_proc_ftm_mode_read,
	.proc_write = oplus_chg_wls_proc_ftm_mode_write,
	.proc_open = simple_open,
	.proc_lseek = noop_llseek,
};
#endif

static int oplus_chg_wls_init_charge_proc(struct oplus_chg_wls *wls_dev)
{
	int ret = 0;
	struct proc_dir_entry *pr_entry_da = NULL;
	struct proc_dir_entry *pr_entry_tmp = NULL;

	pr_entry_da = proc_mkdir("wireless", NULL);
	if (pr_entry_da == NULL) {
		chg_err("Couldn't create wireless proc entry\n");
		return -ENOMEM;
	}

	pr_entry_tmp = proc_create_data("enable_tx", 0664, pr_entry_da,
					&oplus_chg_wls_proc_tx_ops, wls_dev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_err("Couldn't create enable_tx proc entry\n");
		goto fail;
	}

	pr_entry_tmp = proc_create_data("deviated", 0664, pr_entry_da,
					&oplus_chg_wls_proc_deviated_ops, wls_dev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_err("Couldn't create deviated proc entry\n");
		goto fail;
	}

	pr_entry_tmp = proc_create_data("user_sleep_mode", 0664, pr_entry_da,
					&oplus_chg_wls_proc_user_sleep_mode_ops, wls_dev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_err("Couldn't create user_sleep_mode proc entry\n");
		goto fail;
	}

	pr_entry_tmp = proc_create_data("idt_adc_test", 0664, pr_entry_da,
					&oplus_chg_wls_proc_idt_adc_test_ops, wls_dev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_err("Couldn't create idt_adc_test proc entry\n");
		goto fail;
	}

	pr_entry_tmp = proc_create_data("enable_rx", 0664, pr_entry_da,
					&oplus_chg_wls_proc_rx_ops, wls_dev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_err("Couldn't create enable_rx proc entry\n");
		goto fail;
	}

	pr_entry_tmp = proc_create_data("rx_power", 0664, pr_entry_da,
					&oplus_chg_wls_proc_rx_power_ops, wls_dev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_err("Couldn't create rx_power proc entry\n");
		goto fail;
	}

	pr_entry_tmp = proc_create_data("tx_power", 0664, pr_entry_da,
					&oplus_chg_wls_proc_tx_power_ops, wls_dev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_err("Couldn't create tx_power proc entry\n");
		goto fail;
	}

	pr_entry_tmp = proc_create_data("rx_version", 0664, pr_entry_da,
					&oplus_chg_wls_proc_rx_version_ops, wls_dev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_err("Couldn't create rx_version proc entry\n");
		goto fail;
	}

	pr_entry_tmp = proc_create_data("tx_version", 0664, pr_entry_da,
					&oplus_chg_wls_proc_tx_version_ops, wls_dev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_err("Couldn't create tx_version proc entry\n");
		goto fail;
	}

	pr_entry_tmp = proc_create_data("ftm_mode", 0664, pr_entry_da,
					&oplus_chg_wls_proc_ftm_mode_ops, wls_dev);
	if (pr_entry_tmp == NULL) {
		ret = -ENOMEM;
		chg_err("Couldn't create ftm_mode proc entry\n");
		goto fail;
	}

	return 0;

fail:
	remove_proc_entry("wireless", NULL);
	return ret;
}

static void oplus_chg_wls_notify_check(struct oplus_chg_wls *wls_dev, int notify, bool is_set)
{
	int wls_notify;
	struct mms_msg *msg;
	int rc;

	if (is_set)
		wls_notify = wls_dev->wls_notify | notify;
	else
		wls_notify = wls_dev->wls_notify & (~notify);

	if (wls_dev->wls_notify == wls_notify)
		return;
	wls_dev->wls_notify = wls_notify;
	chg_info("wls_notify=0x%08x\n", wls_notify);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, WLS_ITEM_NOTIFY);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(wls_dev->wls_topic, msg);
	if (rc < 0) {
		chg_err("publish wls notify msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_chg_wls_gauge_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_wls *wls_dev = subs->priv_data;
	union mms_msg_data data = { 0 };
	int btb_abnormal_current;
	int rc;

	switch (type) {
	case MSG_TYPE_TIMER:
		oplus_mms_get_item_data(wls_dev->gauge_topic, GAUGE_ITEM_SOC, &data, false);
		wls_dev->batt_info.soc = data.intval;
		oplus_mms_get_item_data(wls_dev->gauge_topic, GAUGE_ITEM_CURR, &data, false);
		wls_dev->batt_info.ibat_ma = data.intval;
		oplus_mms_get_item_data(wls_dev->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, false);
		wls_dev->batt_info.vbat_mv = data.intval;
		oplus_mms_get_item_data(wls_dev->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
		wls_dev->batt_info.batt_temp = data.intval;
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case GAUGE_ITEM_SUB_BTB_STATE:
			rc = oplus_mms_get_item_data(wls_dev->gauge_topic, id,
						     &data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_SUB_BTB_STATE data, rc=%d\n",
					 rc);
			} else {
				if (data.intval == BATT_BTB_STATE_NOT_CONNECT) {
					btb_abnormal_current = oplus_gauge_get_sub_btb_curr_limit(wls_dev->gauge_topic);
					chg_debug("sub_btb abnormal, max curr set %d\n", btb_abnormal_current);
					if (btb_abnormal_current > 0)
						vote(wls_dev->fcc_votable, BAD_SUB_BTB_VOTER, true,
						     btb_abnormal_current, false);
				} else if (data.intval == BATT_BTB_STATE_CONNECT) {
					vote(wls_dev->fcc_votable, BAD_SUB_BTB_VOTER, false,
					     0, false);
				}
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

static void oplus_chg_wls_subscribe_gauge_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_wls *wls_dev = prv_data;
	union mms_msg_data data = { 0 };

	wls_dev->gauge_topic = topic;
	wls_dev->gauge_subs = oplus_mms_subscribe(wls_dev->gauge_topic, wls_dev,
						   oplus_chg_wls_gauge_subs_callback,
						   "chg_wls");
	if (IS_ERR_OR_NULL(wls_dev->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n", PTR_ERR(wls_dev->gauge_subs));
		return;
	}

	oplus_mms_get_item_data(wls_dev->gauge_topic, GAUGE_ITEM_SOC, &data, true);
	wls_dev->batt_info.soc = data.intval;
	oplus_mms_get_item_data(wls_dev->gauge_topic, GAUGE_ITEM_CURR, &data, true);
	wls_dev->batt_info.ibat_ma = data.intval;
	oplus_mms_get_item_data(wls_dev->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	wls_dev->batt_info.vbat_mv = data.intval;
	oplus_mms_get_item_data(wls_dev->gauge_topic, GAUGE_ITEM_TEMP, &data, true);
	wls_dev->batt_info.batt_temp = data.intval;
}

static void oplus_chg_wls_otg_enable_item_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_otg_enable_work);
	union mms_msg_data data = { 0 };

	mutex_lock(&wls_dev->tx_switch_lock);
	oplus_mms_get_item_data(wls_dev->wired_topic, WIRED_ITEM_OTG_ENABLE, &data, false);
	if (!data.intval) {
		wls_dev->pre_otg_enable = false;
		if (wls_dev->support_no_hw_interlock)
			vote(wls_dev->rx_disable_votable, OTG_EN_VOTER, false, 0, false);
		if (!wls_dev->support_acdrv_no_ext_boost)
			vote(wls_dev->insert_disable_votable, OTG_EN_VOTER, false, 0, false);
		if (wls_dev->support_acdrv_no_ext_boost &&
		    (wls_dev->wls_status.wls_type == OPLUS_CHG_WLS_TRX || wls_dev->wls_status.trx_present_keep == true)) {
			wls_dev->wls_status.trx_present_keep = true;
			mutex_lock(&wls_dev->trx_enable_lock);
			(void)oplus_chg_wls_set_trx_enable(wls_dev, false);
			/*for HW spec,need 100ms delay*/
			msleep(100);
			(void)oplus_chg_wls_set_trx_enable(wls_dev, true);
			mutex_unlock(&wls_dev->trx_enable_lock);
		}
		vote(wls_dev->wrx_en_votable, OTG_EN_VOTER, false, 0, false);

		if (wls_dev->wls_status.upgrade_fw_pending) {
			wls_dev->wls_status.upgrade_fw_pending = false;
			schedule_delayed_work(&wls_dev->wls_upgrade_fw_work, 0);
		}
		if (wls_dev->wlsotg_non_coexistence) {
			(void)oplus_chg_wls_rx_standby_config(wls_dev->wls_rx->rx_ic, false);
			/*cancel wls connect pending*/
			oplus_chg_wls_notify_check(wls_dev, WLS_CONNECT_PENDING, false);
		}
	} else {
		if (wls_dev->support_acdrv_no_ext_boost && wls_dev->wls_status.trx_present_keep == true) {
			mutex_lock(&wls_dev->trx_enable_lock);
			(void)oplus_chg_wls_set_trx_enable(wls_dev, true);
			mutex_unlock(&wls_dev->trx_enable_lock);
		}
		if (wls_dev->wlsotg_non_coexistence)
			(void)oplus_chg_wls_rx_standby_config(wls_dev->wls_rx->rx_ic, true);
	}
	mutex_unlock(&wls_dev->tx_switch_lock);
}

static void oplus_chg_wls_wired_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_wls *wls_dev = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_PRESENT:
			oplus_mms_get_item_data(wls_dev->wired_topic, id, &data, false);
			if (data.intval) {
				chg_info("usb present\n");
				oplus_chg_wls_rx_set_rx_mode_safety(wls_dev, OPLUS_CHG_WLS_RX_MODE_UNKNOWN);
				wls_dev->usb_present = true;
				if (!wls_dev->vac_present)
					schedule_delayed_work(&wls_dev->usb_int_work, 0);
				if (is_batt_psy_available(wls_dev))
					power_supply_changed(wls_dev->batt_psy);
			}
			break;
		case WIRED_ITEM_ONLINE:
			oplus_mms_get_item_data(wls_dev->wired_topic, id, &data, false);
			if (data.intval) {
				chg_info("usb online\n");
				wls_dev->usb_present = true;
				if (!wls_dev->vac_present)
					schedule_delayed_work(&wls_dev->usb_int_work, 0);
				if (is_batt_psy_available(wls_dev))
					power_supply_changed(wls_dev->batt_psy);
			} else {
				chg_info("usb offline\n");
				wls_dev->usb_present = false;
				if (wls_dev->vac_present) {
					wls_dev->vac_present = false;
					schedule_delayed_work(&wls_dev->vac_int_work, 0);
				} else {
					schedule_delayed_work(&wls_dev->usb_int_work, 0);
				}
				if (wls_dev->wls_status.upgrade_fw_pending) {
					wls_dev->wls_status.upgrade_fw_pending = false;
					schedule_delayed_work(&wls_dev->wls_upgrade_fw_work, 0);
				}
			}
			break;
		case WIRED_ITEM_PRE_OTG_ENABLE:
			if (wls_dev->pre_otg_enable)
				break;
			chg_info("pre otg enable\n");
			mutex_lock(&wls_dev->tx_switch_lock);
			if (wls_dev->support_acdrv_no_ext_boost && wls_dev->wls_status.wls_type == OPLUS_CHG_WLS_TRX) {
				wls_dev->wls_status.trx_present_keep = true;
				mutex_lock(&wls_dev->trx_enable_lock);
				(void)oplus_chg_wls_set_trx_enable(wls_dev, false);
				mutex_unlock(&wls_dev->trx_enable_lock);
			}
			wls_dev->pre_otg_enable = true;
			vote(wls_dev->wrx_en_votable, OTG_EN_VOTER, true, 1, false);
			if (!wls_dev->support_acdrv_no_ext_boost)
				vote(wls_dev->insert_disable_votable, OTG_EN_VOTER, true, 1, false);
			if (wls_dev->support_no_hw_interlock)
				vote(wls_dev->rx_disable_votable, OTG_EN_VOTER, true, 1, false);
			mutex_unlock(&wls_dev->tx_switch_lock);
			/*for HW spec,need 100ms delay*/
			msleep(100);
			break;
		case WIRED_ITEM_OTG_ENABLE:
			/*for HW spec,need 100ms delay*/
			cancel_delayed_work(&wls_dev->wls_otg_enable_work);
			schedule_delayed_work(&wls_dev->wls_otg_enable_work, msecs_to_jiffies(100));
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_chg_wls_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_wls *wls_dev = prv_data;
	union mms_msg_data data = { 0 };

	wls_dev->wired_topic = topic;
	wls_dev->wired_subs = oplus_mms_subscribe(wls_dev->wired_topic, wls_dev,
						  oplus_chg_wls_wired_subs_callback,
						  "chg_wls");
	if (IS_ERR_OR_NULL(wls_dev->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n", PTR_ERR(wls_dev->wired_subs));
		return;
	}

	oplus_mms_get_item_data(wls_dev->wired_topic, WIRED_ITEM_COEXISTENCE, &data, true);
	wls_dev->wlsotg_non_coexistence = !!data.intval;
	complete_all(&wls_dev->nor_ic_ack);
	oplus_mms_get_item_data(wls_dev->wired_topic, WIRED_ITEM_PRESENT, &data, true);
	oplus_mms_get_item_data(wls_dev->wired_topic, WIRED_ITEM_ONLINE, &data, true);
	wls_dev->usb_present = !!data.intval;
	oplus_mms_get_item_data(wls_dev->wired_topic, WIRED_ITEM_OTG_ENABLE, &data, true);
}

static void oplus_chg_wls_batt_bal_subs_callback(struct mms_subscribe *subs,
						 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_wls *wls_dev = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case BATT_BAL_ITEM_CURR_LIMIT:
			schedule_work(&wls_dev->batt_bal_curr_limit_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_chg_wls_subscribe_batt_bal_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_chg_wls *wls_dev = prv_data;
	union mms_msg_data data = { 0 };

	wls_dev->batt_bal_topic = topic;
	wls_dev->batt_bal_subs =
		oplus_mms_subscribe(wls_dev->batt_bal_topic, wls_dev,
				    oplus_chg_wls_batt_bal_subs_callback, "chg_wls");
	if (IS_ERR_OR_NULL(wls_dev->batt_bal_subs)) {
		chg_err("subscribe batt bal topic error, rc=%ld\n",
			PTR_ERR(wls_dev->batt_bal_subs));
		return;
	}

	oplus_mms_get_item_data(wls_dev->batt_bal_topic, BATT_BAL_ITEM_CURR_LIMIT, &data, true);
	if (data.intval)
		schedule_work(&wls_dev->batt_bal_curr_limit_work);
	chg_info("batt_bal_curr_limit=%d\n", data.intval);
}


static void oplus_chg_wls_ui_soc_tx_fan_change_handler(struct oplus_chg_wls *wls_dev)
{
	static bool ui_soc_100_set = false;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if ((!wls_status->rx_online) ||
	    (wls_status->charge_type != WLS_CHARGE_TYPE_FAST)) {
		ui_soc_100_set = false;
		wls_status->fastchg_ui_100_fan_speed_set = false;
		return;
	}

	if (wls_dev->batt_info.ui_soc == 100) {
		wls_status->fastchg_ui_100_fan_speed_set = true;
		wls_status->quiet_mode_init = false;
		oplus_chg_wls_check_quiet_mode(wls_dev);
		ui_soc_100_set = true;
	}

	if ((wls_dev->batt_info.ui_soc == 98) && (ui_soc_100_set == true)) {
		wls_status->fastchg_ui_100_fan_speed_set = false;
		wls_status->quiet_mode_init = false;
		oplus_chg_wls_check_quiet_mode(wls_dev);
		ui_soc_100_set = false;
	}

	return;
}

static void oplus_chg_wls_comm_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_wls *wls_dev = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_TEMP_REGION:
			schedule_work(&wls_dev->temp_region_update_work);
			break;
		case COMM_ITEM_FCC_GEAR:
			schedule_work(&wls_dev->fcc_gear_update_work);
			break;
		case COMM_ITEM_UI_SOC:
			oplus_mms_get_item_data(wls_dev->comm_topic, id, &data, false);
			wls_dev->batt_info.ui_soc = data.intval;
			oplus_chg_wls_ui_soc_tx_fan_change_handler(wls_dev);
			break;
		case COMM_ITEM_COOL_DOWN:
			schedule_work(&wls_dev->cool_down_update_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_chg_wls_subscribe_comm_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_wls *wls_dev = prv_data;
	union mms_msg_data data = { 0 };

	wls_dev->comm_topic = topic;
	wls_dev->comm_subs = oplus_mms_subscribe(wls_dev->comm_topic, wls_dev,
						   oplus_chg_wls_comm_subs_callback,
						   "chg_wls");
	if (IS_ERR_OR_NULL(wls_dev->comm_subs)) {
		chg_err("subscribe comm topic error, rc=%ld\n", PTR_ERR(wls_dev->comm_subs));
		return;
	}

	oplus_mms_get_item_data(wls_dev->comm_topic, COMM_ITEM_TEMP_REGION, &data, true);
	wls_dev->wls_status.comm_temp_region = data.intval;
	oplus_mms_get_item_data(wls_dev->comm_topic, COMM_ITEM_FCC_GEAR, &data, true);
	wls_dev->wls_status.fcc_gear = data.intval;
	oplus_mms_get_item_data(wls_dev->comm_topic, COMM_ITEM_UI_SOC, &data, true);
	wls_dev->batt_info.ui_soc = data.intval;
	oplus_mms_get_item_data(wls_dev->comm_topic, COMM_ITEM_COOL_DOWN, &data, true);
	wls_dev->wls_status.cool_down = data.intval < 0 ? 0 : data.intval;
}

static void oplus_chg_wls_temp_region_update_work(struct work_struct *work)
{
	struct oplus_chg_wls *wls_dev =
		container_of(work, struct oplus_chg_wls, temp_region_update_work);
	enum oplus_chg_temp_region temp_region;
	enum oplus_chg_temp_region pre_temp_region;
	union mms_msg_data data = { 0 };
	int rc;

	if (wls_dev->comm_topic == NULL) {
		chg_err("comm_topic not ready\n");
		return;
	}

	pre_temp_region = oplus_chg_wls_get_temp_region(wls_dev);
	oplus_mms_get_item_data(wls_dev->comm_topic, COMM_ITEM_TEMP_REGION, &data, false);
	wls_dev->wls_status.comm_temp_region = data.intval;
	temp_region = oplus_chg_wls_get_temp_region(wls_dev);
	if (wls_dev->wls_status.fastchg_started) {
		if (!((pre_temp_region == BATT_TEMP_LITTLE_COOL && temp_region == BATT_TEMP_PRE_NORMAL) ||
		    (pre_temp_region == BATT_TEMP_PRE_NORMAL && temp_region == BATT_TEMP_LITTLE_COOL)))
			return;
	}
	if ((BATT_TEMP_COOL <= temp_region) && (temp_region <= BATT_TEMP_LITTLE_WARM)) {
		rc = oplus_chg_wls_choose_fastchg_curve(wls_dev);
		if (rc < 0)
			chg_err("choose fastchg curve failed, rc = %d\n", rc);
		else
			wls_dev->wls_status.fastchg_level = 0;
	}
	oplus_chg_wls_config(wls_dev);
}

static void oplus_chg_wls_fcc_gear_update_work(struct work_struct *work)
{
	struct oplus_chg_wls *wls_dev =
		container_of(work, struct oplus_chg_wls, fcc_gear_update_work);
	union mms_msg_data data = { 0 };

	if (wls_dev->comm_topic == NULL) {
		chg_err("comm_topic not ready\n");
		return;
	}
	oplus_mms_get_item_data(wls_dev->comm_topic, COMM_ITEM_FCC_GEAR, &data, false);
	wls_dev->wls_status.fcc_gear = data.intval;
	chg_info("set fcc gear to %d\n", wls_dev->wls_status.fcc_gear);
	oplus_chg_wls_config(wls_dev);
}

static void oplus_chg_wls_cool_down_update_work(struct work_struct *work)
{
	struct oplus_chg_wls *wls_dev =
		container_of(work, struct oplus_chg_wls, cool_down_update_work);
	union mms_msg_data data = { 0 };

	if (wls_dev->comm_topic == NULL) {
		chg_err("comm_topic not ready\n");
		return;
	}
	oplus_mms_get_item_data(wls_dev->comm_topic, COMM_ITEM_COOL_DOWN, &data, false);
	wls_dev->wls_status.cool_down = data.intval < 0 ? 0 : data.intval;
	chg_info("set cool down level to %d\n", wls_dev->wls_status.cool_down);
	oplus_chg_wls_config(wls_dev);
}

static void oplus_chg_wls_batt_bal_curr_limit_work(struct work_struct *work)
{
	/*int curr_limit = 0;*/
	struct oplus_chg_wls *wls_dev =
		container_of(work, struct oplus_chg_wls, batt_bal_curr_limit_work);
	union mms_msg_data data = { 0 };

	if (wls_dev->batt_bal_topic == NULL) {
		chg_err("batt_bal_topic not ready\n");
		return;
	}
	oplus_mms_get_item_data(wls_dev->batt_bal_topic, BATT_BAL_ITEM_CURR_LIMIT, &data, false);
	/*curr_limit = data.intval;*/

/* The current needs to be set according to whether cp is 2: 1 or 1: 1
	if (curr_limit)
		vote(wls_dev->fcc_votable, BATT_BAL_VOTER, true, curr_limit, false);
	else
		vote(wls_dev->fcc_votable, BATT_BAL_VOTER, false, 0, false);
	chg_info("set curr_limit = %d\n", curr_limit);
 */
}

static void oplus_chg_wls_err_handler_work(struct work_struct *work)
{
	struct oplus_chg_wls *wls_dev =
		container_of(work, struct oplus_chg_wls, wls_err_handler_work);
	struct oplus_chg_ic_err_msg *msg, *tmp;
	struct list_head msg_list;

	INIT_LIST_HEAD(&msg_list);
	spin_lock(&wls_dev->wls_rx->rx_ic->err_list_lock);
	if (!list_empty(&wls_dev->wls_rx->rx_ic->err_list))
		list_replace_init(&wls_dev->wls_rx->rx_ic->err_list, &msg_list);
	spin_unlock(&wls_dev->wls_rx->rx_ic->err_list_lock);

	list_for_each_entry_safe(msg, tmp, &msg_list, list) {
		if (is_err_topic_available(wls_dev))
			oplus_mms_publish_ic_err_msg(wls_dev->err_topic, ERR_ITEM_IC, msg);
		oplus_print_ic_err(msg);
		list_del(&msg->list);
		kfree(msg);
	}
}

static void oplus_chg_wls_epp_force_to_bpp_flag_clear_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_epp_check_flag_clear_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER) > 0) {
		vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER, false, 0, false);
		oplus_chg_wls_rx_set_rx_mode_safety(wls_dev, OPLUS_CHG_WLS_RX_MODE_UNKNOWN);
	}
	wls_status->epp_check_timeout = 0;
	wls_status->epp_to_bpp_connect_time = 0;
	wls_status->epp_disconnect_time = 0;
	wls_status->epp_to_bpp_reason_record = 0;
	wls_status->epp_force_to_bpp_flag = false;
	chg_info("oplus_chg_wls_epp_force_to_bpp_flag_clear_work\n");

	return;
}

static int oplus_wls_get_batt_full_status(struct oplus_chg_wls *wls_dev)
{
	bool batt_full = 0;
	union mms_msg_data data = { 0 };
	int rc;

	if (!wls_dev || !wls_dev->comm_topic) {
		chg_err("comm_topic not ready\n");
		return -1;
	}

	rc = oplus_mms_get_item_data(wls_dev->comm_topic, COMM_ITEM_CHG_FULL, &data,
				     true);
	if (!rc)
		batt_full = data.intval;

	chg_info("get batt_full status = %d\n", batt_full);
	return batt_full;
}

static void oplus_wls_epp_force_to_bpp_action(struct oplus_chg_wls *wls_dev, unsigned long force_bpp_timeout)
{
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;

	if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER) > 0) {
		wls_status->epp_check_timeout = 0;
		wls_status->epp_disconnect_time = jiffies;
		wls_status->epp_force_to_bpp_flag = true;
		cancel_delayed_work(&wls_dev->wls_epp_check_flag_clear_work);
		schedule_delayed_work(&wls_dev->wls_epp_check_flag_clear_work, msecs_to_jiffies(force_bpp_timeout));
	}
}


#define EPP_CHECK_WORK_DELAY_SEC	70
#define EPP_FORCE_TO_BPP_TOGGLE_MS	3500
#define EPP_FORCE_TO_BPP_MODE_TIMEOUT_MS	6200
#define EPP_FORCE_TO_BPP_IOUT_THRESHTOLD	200

static void oplus_chg_wls_epp_iout_too_small_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_epp_iout_too_small_check_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	bool batt_full = false;
	int iout_ma = 0;

	if (wls_dev->rx_protocol_version < WLS_RX_PROTOCOL_VERSION_30)
		return;

	if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_CAMERA_MODE_VOTER) > 0)
		goto out;

	if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER) > 0)
		return;

	oplus_chg_wls_rx_get_iout(wls_dev->wls_rx->rx_ic, &iout_ma);
	batt_full = oplus_wls_get_batt_full_status(wls_dev);

	chg_info("oplus_chg_wls_epp_iout_too_small_check_work\n");
	if (wls_dev->mms_info.rx_present && iout_ma < EPP_FORCE_TO_BPP_IOUT_THRESHTOLD && !batt_full) {
		vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER, true, 1, false);
		oplus_wls_epp_force_to_bpp_action(wls_dev, EPP_FORCE_TO_BPP_MODE_TIMEOUT_MS);
		wls_status->epp_to_bpp_reason_record = WLS_ERR_EPP_FORCE_TO_BPP;
		return;
	}

out:
	cancel_delayed_work(&wls_dev->wls_epp_check_flag_clear_work);
	schedule_delayed_work(&wls_dev->wls_epp_check_flag_clear_work, 0);

	return;
}

static void oplus_chg_wls_boot_quiet_clear_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork, struct oplus_chg_wls, wls_boot_quiet_clear_work);

	vote(wls_dev->quiet_mode_votable, WLS_BOOT_QUIET_VOTER, false, 0, false);
}

static int oplus_chg_wls_epp_force_to_bpp_loop_check(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_dynamic_config *dynamic_cfg = &wls_dev->dynamic_config;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int batt_temp;
	int rc = 0;
	enum oplus_chg_wls_rx_mode rx_mode;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == HIGH_TEMP_AGING || get_eng_version() == FACTORY) {
		chg_err("factory/high temp version, skip epp force bpp\n");
		return rc;
	}
#endif

	oplus_chg_wls_rx_get_rx_mode(wls_dev->wls_rx->rx_ic, &rx_mode);
	if (rc < 0) {
		chg_err("get rx mode error, rc=%d\n", rc);
		return rc;
	}

	if ((wls_dev->rx_protocol_version < WLS_RX_PROTOCOL_VERSION_30) ||
	    (rx_mode != OPLUS_CHG_WLS_RX_MODE_EPP_5W && rx_mode != OPLUS_CHG_WLS_RX_MODE_EPP &&
		 rx_mode != OPLUS_CHG_WLS_RX_MODE_EPP_PLUS))
		return -ENOTSUPP;

	if ((get_client_vote(wls_dev->force_bpp_mode_votable, WLS_CAMERA_MODE_VOTER) > 0)) {
		cancel_delayed_work(&wls_dev->wls_epp_check_flag_clear_work);
		schedule_delayed_work(&wls_dev->wls_epp_check_flag_clear_work, 0);
		return -EBUSY;
	}

	if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER) > 0)
		return 0;
	rc = oplus_chg_wls_get_batt_temp(wls_dev, &batt_temp);
	if (rc < 0) {
		chg_err("can't get batt temp, rc=%d\n", rc);
		return rc;
	}

	if (wls_dev->mms_info.rx_present) {
		if ((dynamic_cfg->epp_high_temp_thr != -EINVAL) && (batt_temp > dynamic_cfg->epp_high_temp_thr)) {
			vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER, true, 1, false);
			oplus_wls_epp_force_to_bpp_action(wls_dev, EPP_FORCE_TO_BPP_MODE_TIMEOUT_MS);
			wls_status->epp_to_bpp_reason_record = WLS_ERR_EPP_HIGH_TEMP_TO_BPP;
		}
	}

	return 0;
}

static void oplus_chg_wls_epp_force_to_bpp_check_handler(struct oplus_chg_wls *wls_dev)
{
	enum oplus_chg_wls_rx_mode rx_mode;
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	int rc = 0;

	if (wls_dev->rx_protocol_version < WLS_RX_PROTOCOL_VERSION_30)
		return;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == HIGH_TEMP_AGING || get_eng_version() == FACTORY) {
		chg_err("factory/high temp version, skip epp force bpp\n");
		return;
	}
#endif

	if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_CAMERA_MODE_VOTER) > 0)
		goto out;

	if (wls_dev->mms_info.rx_present == 1) {
		wls_status->epp_check_timeout = 0;
		wls_status->epp_force_to_bpp_flag = false;
		oplus_chg_wls_rx_get_rx_mode(wls_dev->wls_rx->rx_ic, &rx_mode);
		if (rc < 0) {
			chg_err("get rx mode error, rc=%d\n", rc);
			goto out;
		}
		chg_info("rx_mode:%d, jiffies:%lu, epp_check_timeout=%lu n",
			rx_mode, jiffies, wls_status->epp_check_timeout);

		if ((rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_5W || rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP ||
		    rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_PLUS) && !get_client_vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER)) {
			wls_status->epp_check_timeout = jiffies + EPP_CHECK_WORK_DELAY_SEC * HZ;
			wls_status->epp_to_bpp_connect_time = 0;
			wls_status->epp_disconnect_time = 0;
			cancel_delayed_work(&wls_dev->wls_epp_iout_too_small_check_work);
			schedule_delayed_work(&wls_dev->wls_epp_iout_too_small_check_work, msecs_to_jiffies(EPP_CHECK_WORK_DELAY_SEC * 1000));
			return;
		}

		if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER) && rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
			cancel_delayed_work(&wls_dev->wls_epp_check_flag_clear_work);
			wls_status->epp_to_bpp_connect_time = jiffies_to_msecs(jiffies - wls_status->epp_disconnect_time);
			oplus_chg_wls_track_upload_wls_err_info(wls_dev, WLS_ERR_SCENE_RX, wls_status->epp_to_bpp_reason_record);
			return;
		}

	} else {
		cancel_delayed_work(&wls_dev->wls_epp_iout_too_small_check_work);
		if (time_before(jiffies, wls_status->epp_check_timeout) &&
		    !get_client_vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER) &&
		    !get_effective_result(wls_dev->rx_disable_votable)) {
			chg_info("epp disconnect: jiffies:%lu, epp_check_timeout:%lu\n", jiffies, wls_status->epp_check_timeout);
			cancel_delayed_work(&wls_dev->wls_epp_iout_too_small_check_work);
			vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER, true, 1, false);
			oplus_wls_epp_force_to_bpp_action(wls_dev, EPP_FORCE_TO_BPP_TOGGLE_MS);
			wls_status->epp_to_bpp_reason_record = WLS_ERR_EPP_QUICK_DISCONNECT_TO_BPP;
			return;
		} else if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_FORCE_EPP_TO_BPP_VOTER)) {
			if (wls_status->wls_type == OPLUS_CHG_WLS_EPP || wls_status->wls_type == OPLUS_CHG_WLS_EPP_PLUS) {
				oplus_wls_epp_force_to_bpp_action(wls_dev, EPP_FORCE_TO_BPP_MODE_TIMEOUT_MS);
				return;
		    }
		}

		if (wls_status->epp_force_to_bpp_flag)
			return;
	}

out:
	cancel_delayed_work(&wls_dev->wls_epp_check_flag_clear_work);
	schedule_delayed_work(&wls_dev->wls_epp_check_flag_clear_work, 0);
	return;
}


static void oplus_chg_wls_present_handler_work(struct work_struct *work)
{
	struct oplus_chg_wls *wls_dev =
		container_of(work, struct oplus_chg_wls, wls_present_handler_work);
	struct oplus_chg_wls_status *wls_status = &wls_dev->wls_status;
	struct mms_msg *msg;
	int rc;

	if (wls_dev->wls_topic == NULL) {
		chg_err("wls_topic not ready\n");
		return;
	}

	wls_dev->mms_info.rx_present = oplus_chg_wls_rx_is_connected(wls_dev->wls_rx->rx_ic);
	chg_info("wls present=%d\n", wls_dev->mms_info.rx_present);
	oplus_chg_wls_epp_force_to_bpp_check_handler(wls_dev);
	if (wls_dev->mms_info.rx_present == 1) {
		if (wls_dev->force_rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
			vote(wls_dev->rx_disable_votable, CONNECT_VOTER, false, 0, false);
			if (get_client_vote(wls_dev->force_bpp_mode_votable, WLS_CAMERA_MODE_VOTER) > 0)
				oplus_chg_wls_cam_scene_icl_limit(wls_dev);
		}
		oplus_chg_wls_rx_set_rx_mode_safety(wls_dev, OPLUS_CHG_WLS_RX_MODE_UNKNOWN);
		oplus_chg_wls_rx_get_vrect(wls_dev->wls_rx->rx_ic, &wls_dev->wls_status.track_record.max_vrect);
	} else if ((wls_dev->force_rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP &&
	         wls_status->wls_type != OPLUS_CHG_WLS_BPP && wls_status->wls_type != OPLUS_CHG_WLS_TRX &&
	         wls_status->wls_type != OPLUS_CHG_WLS_UNKNOWN) || (wls_status->epp_force_to_bpp_flag))
		oplus_chg_wls_rx_set_rx_mode_safety(wls_dev, OPLUS_CHG_WLS_RX_MODE_BPP);
	oplus_chg_wls_early_reset_variables(wls_dev);
	oplus_chg_wls_wireless_notifier_call(wls_dev, WLS_ITEM_PRESENT);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, WLS_ITEM_PRESENT);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(wls_dev->wls_topic, msg);
	if (rc < 0) {
		chg_err("publish wls present msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_chg_wls_online_handler_work(struct work_struct *work)
{
	struct oplus_chg_wls *wls_dev =
		container_of(work, struct oplus_chg_wls, wls_online_handler_work);
	struct mms_msg *msg;
	int rc;

	if (wls_dev->wls_topic == NULL) {
		chg_err("wls_topic not ready\n");
		return;
	}

	if (!oplus_chg_wls_rx_is_connected(wls_dev->wls_rx->rx_ic)) {
		chg_err("wireless charging is not connected\n");
		return;
	}

	chg_info("wls online\n");
	wls_dev->mms_info.rx_online = true;
	oplus_chg_wls_wireless_notifier_call(wls_dev, WLS_ITEM_ONLINE);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, WLS_ITEM_ONLINE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(wls_dev->wls_topic, msg);
	if (rc < 0) {
		chg_err("publish wls present msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_chg_wls_offline_handler_work(struct work_struct *work)
{
	struct oplus_chg_wls *wls_dev =
		container_of(work, struct oplus_chg_wls, wls_offline_handler_work);
	struct mms_msg *msg;
	int rc;

	if (wls_dev->wls_topic == NULL) {
		chg_err("wls_topic not ready\n");
		return;
	}

	chg_info("wls offline\n");
	wls_dev->mms_info.rx_online = false;
	oplus_chg_wls_wireless_notifier_call(wls_dev, WLS_ITEM_ONLINE);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, WLS_ITEM_ONLINE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(wls_dev->wls_topic, msg);
	if (rc < 0) {
		chg_err("publish wls offline msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_chg_wls_event_changed_handler_work(struct work_struct *work)
{
	struct oplus_chg_wls *wls_dev =
		container_of(work, struct oplus_chg_wls, wls_event_changed_handler_work);
	enum oplus_chg_wls_event_code event_code;
	enum oplus_chg_wls_rx_mode rx_mode;
	int rc;

	if (wls_dev->wls_topic == NULL) {
		chg_err("wls_topic not ready\n");
		return;
	}

	rc = oplus_chg_wls_rx_get_event_code(wls_dev->wls_rx->rx_ic, &event_code);
	if (rc < 0) {
		chg_err("get event code err\n");
		return;
	}
	wls_dev->mms_info.event_code = event_code;
	switch (event_code) {
	case WLS_EVENT_RX_EPP_CAP:
		oplus_chg_wls_rx_get_rx_mode(wls_dev->wls_rx->rx_ic, &rx_mode);
		if (rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_5W ||
		    rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP ||
		    rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_PLUS) {
			oplus_chg_wls_nor_set_vindpm(wls_dev->wls_nor->nor_ic, WLS_VINDPM_EPP);
			(void)oplus_chg_wls_rx_get_tx_id(wls_dev->wls_rx->rx_ic, &wls_dev->wls_status.tx_manu_id);
		}
		break;
	case WLS_EVENT_RX_UVP_ALARM:
		vote(wls_dev->nor_input_disable_votable, UOVP_VOTER, true, 0, false);
		break;
	case WLS_EVENT_RX_UVP_CLEAR:
		vote(wls_dev->nor_input_disable_votable, UOVP_VOTER, false, 0, false);
		break;
	case WLS_EVENT_TRX_CHECK:
		if (wls_dev->wls_status.trx_present) {
			cancel_delayed_work(&wls_dev->wls_trx_sm_work);
			queue_delayed_work(wls_dev->wls_wq, &wls_dev->wls_trx_sm_work, 0);
		} else {
			chg_err("trx not present\n");
		}
		break;
	case WLS_EVENT_VAC_PRESENT:
		if (wls_dev->vac_present || wls_dev->pre_otg_enable)
			break;
		chg_info("vac present\n");
		wls_dev->vac_present = true;
		schedule_delayed_work(&wls_dev->vac_int_work, 0);
		break;
	case WLS_EVENT_FORCE_UPGRADE:
		if (wls_dev->force_upgrade != FORCE_UPGRADE_NONE)
			break;
		if (wls_dev->wls_status.wls_type == OPLUS_CHG_WLS_TRX) {
			if (!wls_dev->pre_otg_enable && !oplus_chg_wls_is_usb_present(wls_dev)) {
				chg_info("force upgrade: disable tx\n");
				mutex_lock(&wls_dev->trx_enable_lock);
				(void)oplus_chg_wls_set_trx_enable(wls_dev, false);
				mutex_unlock(&wls_dev->trx_enable_lock);
				wls_dev->force_upgrade = FORCE_UPGRADE_TX;
			} else {
				break;
			}
		} else {
			wls_dev->force_upgrade = FORCE_UPGRADE_RX;
		}
		chg_info("force upgrade[%d]...\n", wls_dev->force_upgrade);
		schedule_delayed_work(&wls_dev->wls_upgrade_fw_work, 0);
		break;
	case WLS_EVENT_RXAC_ATTACH:
		chg_info("wls connect pending...\n");
		oplus_chg_wls_notify_check(wls_dev, WLS_CONNECT_PENDING, true);
		if (is_batt_psy_available(wls_dev))
			power_supply_changed(wls_dev->batt_psy);
		break;
	case WLS_EVENT_RXAC_DETACH:
		chg_info("cancel wls connect pending...\n");
		oplus_chg_wls_notify_check(wls_dev, WLS_CONNECT_PENDING, false);
		if (is_batt_psy_available(wls_dev))
			power_supply_changed(wls_dev->batt_psy);
		break;
	case WLS_EVENT_EPP_NEGO_FAIL:
		chg_info("EPP nego fail\n");
		(void)oplus_chg_wls_rx_set_fod_parm(wls_dev->wls_rx->rx_ic,
			oplus_chg_wls_get_fod_parm(wls_dev, wls_dev->wls_status.adapter_id, FOD_BPP_MODE),
			wls_dev->static_config.fod_parm_len, FOD_BPP_MODE, wls_dev->magcvr_status);
		break;
	case WLS_EVENT_EPP_TX_MANU_ID:
		chg_info("EPP tx_manu_id get event\n");
		if (wls_dev->static_config.third_part_epp_qf_parm_sets == 0 &&
		    wls_dev->static_config.third_part_epp_qf_parm_magcvr_sets == 0)
			break;
		(void)oplus_chg_wls_rx_get_tx_id(wls_dev->wls_rx->rx_ic, &wls_dev->wls_status.tx_manu_id);
		if (wls_dev->wls_status.tx_manu_id) {
			wls_dev->wls_status.tx_product_id_done = true;
			schedule_delayed_work(&wls_dev->wls_match_q_work, 0);
		}
		break;
	default:
		break;
	}
}

static void oplus_chg_wls_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_chg_wls *wls_dev = virq_data;

	schedule_work(&wls_dev->wls_err_handler_work);
}

static void oplus_chg_wls_present_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_chg_wls *wls_dev = virq_data;

	schedule_work(&wls_dev->wls_present_handler_work);
}

static void oplus_chg_wls_online_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_chg_wls *wls_dev = virq_data;

	schedule_work(&wls_dev->wls_online_handler_work);
}

static void oplus_chg_wls_offline_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_chg_wls *wls_dev = virq_data;

	schedule_work(&wls_dev->wls_offline_handler_work);
}

static void oplus_chg_wls_event_changed_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_chg_wls *wls_dev = virq_data;

	schedule_work(&wls_dev->wls_event_changed_handler_work);
}

static int oplus_chg_wls_virq_register(struct oplus_chg_wls *wls_dev)
{
	int rc;

	rc = oplus_chg_ic_virq_register(wls_dev->wls_rx->rx_ic, OPLUS_IC_VIRQ_ERR,
		oplus_chg_wls_err_handler, wls_dev);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(wls_dev->wls_rx->rx_ic, OPLUS_IC_VIRQ_PRESENT,
		oplus_chg_wls_present_handler, wls_dev);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_PRESENT error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(wls_dev->wls_rx->rx_ic, OPLUS_IC_VIRQ_ONLINE,
		oplus_chg_wls_online_handler, wls_dev);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(wls_dev->wls_rx->rx_ic, OPLUS_IC_VIRQ_OFFLINE,
		oplus_chg_wls_offline_handler, wls_dev);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_OFFLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(wls_dev->wls_rx->rx_ic, OPLUS_IC_VIRQ_EVENT_CHANGED,
		oplus_chg_wls_event_changed_handler, wls_dev);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_EVENT_CHANGED error, rc=%d", rc);

	return 0;
}

static int oplus_chg_wls_mms_update_present(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	bool present = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	present = wls_dev->wls_status.online_keep | wls_dev->wls_status.boot_online_keep;
	present = present | wls_dev->mms_info.rx_present;

	data->intval = present;
	return 0;
}

static int oplus_chg_wls_mms_update_online(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	bool online = false;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	online = wls_dev->mms_info.rx_online;

	data->intval = online;

	return 0;
}

static int oplus_chg_wls_mms_update_vout(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	int vout = 0;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	vout = wls_dev->wls_status.vout_mv;
	if (wls_dev->wls_status.trx_present) {
		rc = oplus_chg_wls_rx_get_trx_vol(wls_dev->wls_rx->rx_ic, &wls_dev->wls_status.trx_vol_mv);
		if (rc < 0)
			chg_err("can't get trx vol, rc=%d\n", rc);
		else
			vout = wls_dev->wls_status.trx_vol_mv;
	}

	data->intval = vout;

	return 0;
}

static int oplus_chg_wls_mms_update_iout(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	int iout = 0;
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	iout = wls_dev->wls_status.iout_ma;
	if (wls_dev->wls_status.trx_present) {
		rc = oplus_chg_wls_rx_get_trx_curr(wls_dev->wls_rx->rx_ic, &wls_dev->wls_status.trx_curr_ma);
		if (rc < 0)
			chg_err("can't get trx curr, rc=%d\n", rc);
		else
			iout = wls_dev->wls_status.trx_curr_ma;
	}

	data->intval = iout;

	return 0;
}

static int oplus_chg_wls_mms_update_wls_type(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	enum oplus_chg_wls_type type = OPLUS_CHG_WLS_UNKNOWN;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	type = wls_dev->wls_status.wls_type;

	data->intval = type;

	return 0;
}

static int oplus_chg_wls_mms_update_track_record_wls_type(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	static enum oplus_chg_wls_type type = OPLUS_CHG_WLS_UNKNOWN;
	enum oplus_chg_wls_rx_mode rx_mode;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	if (wls_dev->wls_status.wls_type != OPLUS_CHG_WLS_UNKNOWN || wls_dev->mms_info.rx_present)
		type = wls_dev->wls_status.wls_type;

	if (wls_dev->wls_status.wls_type == OPLUS_CHG_WLS_UNKNOWN && wls_dev->wls_status.track_record.rx_mode) {
		rx_mode = wls_dev->wls_status.track_record.rx_mode;
		if (rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_5W || rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP)
			type = OPLUS_CHG_WLS_EPP;
		else if (rx_mode == OPLUS_CHG_WLS_RX_MODE_EPP_PLUS)
			type = OPLUS_CHG_WLS_EPP_PLUS;
		else if (rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP)
			type = OPLUS_CHG_WLS_BPP;
		else
			type = OPLUS_CHG_WLS_UNKNOWN;
	}
	data->intval = type;

	return 0;
}

static int oplus_chg_wls_mms_update_icon_type(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	enum oplus_chg_wls_type icon_type = OPLUS_CHG_WLS_UNKNOWN;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	icon_type = wls_dev->wls_status.wls_type;

	if (wls_dev->wls_status.fastchg_timeout_to_bpp &&
	    wls_dev->wls_status.wls_type == OPLUS_CHG_WLS_BPP)
		icon_type = OPLUS_CHG_WLS_SVOOC;

	data->intval = icon_type;

	return 0;
}

static int oplus_chg_wls_mms_update_audio_cam_scenes(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = wls_dev->wls_status.track_record.pmw_scenarios;

	return 0;
}

static int oplus_chg_wls_mms_update_quiet_mode(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = (wls_dev->wls_status.current_rx_state == OPLUS_CHG_WLS_RX_STATE_QUIET) ?
					true : false;

	return 0;
}

static int oplus_chg_wls_mms_update_tx_manu_id(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = wls_dev->wls_status.tx_manu_id;

	return 0;
}

static int oplus_chg_wls_mms_update_vendor_id(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = wls_dev->wls_status.vendor_id;

	return 0;
}

static int oplus_chg_wls_mms_update_product_id(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = wls_dev->wls_status.product_id;

	return 0;
}

static int oplus_chg_wls_mms_update_track_is_fastchg(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = wls_dev->wls_status.track_record.is_fastchg;

	return 0;
}

static int oplus_chg_wls_mms_update_track_break_manu(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = wls_dev->wls_status.track_record.break_manu;

	return 0;
}

static int oplus_chg_wls_mms_update_real_type(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	enum power_supply_type real_type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	switch (wls_dev->wls_status.wls_type) {
	case OPLUS_CHG_WLS_BPP:
		real_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case OPLUS_CHG_WLS_EPP:
		real_type = POWER_SUPPLY_TYPE_USB_PD;
		break;
	case OPLUS_CHG_WLS_EPP_PLUS:
		real_type = POWER_SUPPLY_TYPE_USB_PD;
		break;
	case OPLUS_CHG_WLS_VOOC:
	case OPLUS_CHG_WLS_SVOOC:
	case OPLUS_CHG_WLS_PD_65W:
		real_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		chg_debug("Unsupported charging mode(=%d)\n", wls_dev->wls_status.wls_type);
		real_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	}

	data->intval = real_type;

	return 0;
}

static int oplus_chg_wls_mms_update_status_keep(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	enum wls_status_keep_type status_keep = WLS_SK_NULL;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	status_keep = wls_dev->mms_info.wls_status_keep;

	data->intval = status_keep;

	return 0;
}

int oplus_chg_wls_set_status_keep(struct oplus_mms *mms, enum wls_status_keep_type status_keep)
{
	struct oplus_chg_wls *wls_dev;
	static int pre_status_keep = WLS_SK_NULL;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	wls_dev->mms_info.wls_status_keep = status_keep;
	if (pre_status_keep != status_keep)
		chg_info("wls_status_keep=%d\n", status_keep);
	pre_status_keep = status_keep;

	return 0;
}

static int oplus_chg_wls_mms_update_dock_type(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = wls_dev->wls_status.adapter_id;

	return 0;
}

static int oplus_chg_wls_mms_update_max_power(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = oplus_chg_wls_get_max_wireless_power(wls_dev);

	return 0;
}

static int oplus_chg_wls_mms_update_fastchg_status(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = wls_dev->wls_status.fastchg_started;

	return 0;
}

static int oplus_chg_wls_mms_update_bcc_max_curr(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	struct oplus_chg_wls_status *wls_status;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);
	wls_status = &wls_dev->wls_status;

	data->intval = wls_status->wls_bcc_max_curr / BCC_TO_ICL;

	return 0;
}

static int oplus_chg_wls_mms_update_bcc_min_curr(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	struct oplus_chg_wls_status *wls_status;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);
	wls_status = &wls_dev->wls_status;

	data->intval = wls_status->wls_bcc_min_curr / BCC_TO_ICL;

	return 0;
}

static int oplus_chg_wls_mms_update_charging_disable(
	struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	wls_dev = oplus_mms_get_drvdata(mms);
	data->intval = wls_dev->charging_disable;

	return 0;
}

static int oplus_chg_wls_mms_update_bcc_exit_curr(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg;
	enum oplus_chg_temp_region temp_region;
	int batt_soc_plugin, batt_temp_plugin;
	int real_soc = 100;
	int rc = 0;
	int wls_stop_curr = 1000;
	int i;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);
	dynamic_cfg = &wls_dev->dynamic_config;

	rc = oplus_chg_wls_get_real_soc(wls_dev, &real_soc);
	if ((rc < 0) || (real_soc >= dynamic_cfg->fastchg_max_soc)) {
		chg_err("can't get real soc or soc is too high, rc=%d\n", rc);
		return -EPERM;
	}

	for (i = 0; i < WLS_FAST_SOC_MAX; i++) {
		if (real_soc < dynamic_cfg->wls_strategy_soc[i]) {
			batt_soc_plugin = i;
			break;
		}
	}
	if (i == WLS_FAST_SOC_MAX)
		batt_soc_plugin = WLS_BCC_SOC_70_TO_90_HIGH;
	temp_region = oplus_chg_wls_get_temp_region(wls_dev);


	if (temp_region <= BATT_TEMP_COOL)
		batt_temp_plugin = WLS_BCC_TEMP_0_TO_50;
	else if (temp_region == BATT_TEMP_LITTLE_COOL)
		batt_temp_plugin = WLS_BCC_TEMP_50_TO_120;
	else if (temp_region == BATT_TEMP_PRE_NORMAL)
		batt_temp_plugin = WLS_BCC_TEMP_120_TO_160;
	else if (temp_region == BATT_TEMP_NORMAL)
		batt_temp_plugin = WLS_BCC_TEMP_160_TO_400;
	else if (temp_region == BATT_TEMP_NORMAL_HIGH)
		batt_temp_plugin = WLS_BCC_TEMP_160_TO_400_HIGH;
	else if (temp_region == BATT_TEMP_LITTLE_WARM)
		batt_temp_plugin = WLS_BCC_TEMP_400_TO_440;
	else
		batt_temp_plugin = WLS_BCC_TEMP_400_TO_440;

	wls_stop_curr = dynamic_cfg->bcc_stop_curr[batt_soc_plugin].bcc_soc_stop_curr[batt_temp_plugin];

	chg_info("get stop curr, enter soc is [%d], batt_temp_plugin[%d], stop curr is %d\n",
	         batt_soc_plugin, batt_temp_plugin, wls_stop_curr);
	data->intval = wls_stop_curr  / BCC_TO_ICL;

	return 0;
}

static int oplus_chg_wls_update_bcc_temp_range(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	struct oplus_chg_wls_status *wls_status;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);
	wls_status = &wls_dev->wls_status;

	if (wls_dev->support_wls_chg_bcc &&
	    (wls_status->bcc_temp_range == WLS_BCC_TEMP_160_TO_400 ||
	     wls_status->bcc_temp_range == WLS_BCC_TEMP_160_TO_400_HIGH ||
	     wls_status->bcc_temp_range == WLS_BCC_TEMP_400_TO_440))
	     data->intval = true;

	return 0;
}

static int oplus_chg_wls_mms_fcc_to_icl(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;
	int fcc_to_icl;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	fcc_to_icl = wls_dev->wls_bcc_fcc_to_icl_factor;

	data->intval = fcc_to_icl;
	return 0;
}

static int oplus_chg_wls_mms_update_magcvr(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = wls_dev->magcvr_status;

	return 0;
}

static int oplus_chg_wls_mms_update_notify(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = wls_dev->wls_notify;

	return 0;
}

static int oplus_chg_wls_mms_update_init_completed(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = (int)wls_dev->wls_init_completed;

	return 0;
}

static int oplus_chg_wls_mms_update_fw_upgrading(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_chg_wls *wls_dev;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	wls_dev = oplus_mms_get_drvdata(mms);

	data->intval = (int)wls_dev->wls_status.fw_upgrading;

	return 0;
}

static void oplus_chg_wls_mms_update(struct oplus_mms *mms, bool publish)
{
	/*only for auto update*/
	return;
}

static struct mms_item oplus_chg_wls_mms_item[] = {
	{
		.desc = {
			.item_id = WLS_ITEM_PRESENT,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_present,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_ONLINE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_online,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_VOUT,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_vout,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_IOUT,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_iout,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_WLS_TYPE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_wls_type,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_REAL_TYPE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_real_type,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_STATUS_KEEP,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_status_keep,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_DOCK_TYPE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_dock_type,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_MAX_POWER,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_max_power,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_FASTCHG_STATUS,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_fastchg_status,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_BCC_MAX_CURR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_bcc_max_curr,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_BCC_MIN_CURR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_bcc_min_curr,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_BCC_EXIT_CURR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_bcc_exit_curr,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_BCC_TEMP_RANGE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_update_bcc_temp_range,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_FCC_TO_ICL,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_fcc_to_icl,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_CHARGING_DISABLE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_charging_disable,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_MAGCVR,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_magcvr,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_NOTIFY,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_notify,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_INIT_COMPLETED,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_init_completed,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_MAX_PROJECT_POWER,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_project_power,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_UI_POWER,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_ui_power,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_FW_UPGRADING,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_fw_upgrading,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_ICON_TYPE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_icon_type,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_TRACK_RECORD_WLS_TYPE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_track_record_wls_type,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_AUDIO_CAM_PMW,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_audio_cam_scenes,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_QUIET_MODE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_quiet_mode,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_TX_MANU_ID,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_tx_manu_id,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_VENDOR_ID,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_vendor_id,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_PRODUCT_ID,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_product_id,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_TRACK_RECORD_FASTCHG,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_track_is_fastchg,
		}
	},
	{
		.desc = {
			.item_id = WLS_ITEM_TRACK_BREAK_MANU,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_chg_wls_mms_update_track_break_manu,
		}
	}
};

static const struct oplus_mms_desc oplus_chg_wls_mms_desc = {
	.name = "wireless",
	.type = OPLUS_MMS_TYPE_WLS,
	.item_table = oplus_chg_wls_mms_item,
	.item_num = ARRAY_SIZE(oplus_chg_wls_mms_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_chg_wls_mms_update,
};

static int oplus_chg_wls_topic_init(struct oplus_chg_wls *wls_dev)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;

	mms_cfg.drv_data = wls_dev;
	mms_cfg.of_node = oplus_get_node_by_type(wls_dev->dev->of_node);

	if (of_property_read_bool(mms_cfg.of_node, "oplus,topic-update-interval")) {
		rc = of_property_read_u32(mms_cfg.of_node, "oplus,topic-update-interval", &mms_cfg.update_interval);
		if (rc < 0)
			mms_cfg.update_interval = 0;
	}

	wls_dev->wls_topic = devm_oplus_mms_register(wls_dev->dev, &oplus_chg_wls_mms_desc, &mms_cfg);
	if (IS_ERR(wls_dev->wls_topic)) {
		rc = PTR_ERR(wls_dev->wls_topic);
		wls_dev->wls_topic = NULL;
		chg_err("Couldn't register wls topic\n");
		return rc;
	}

	return 0;
}

#define NOR_IC_INIT_TIMEOUT_MS	((OPLUS_CHG_IC_INIT_RETRY_DELAY) * (OPLUS_CHG_IC_INIT_RETRY_MAX))
#define WLS_INIT_COMPLETE_DELAY_MS	500
static void oplus_chg_wls_mms_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_wls *wls_dev = container_of(dwork,
		struct oplus_chg_wls, wls_mms_init_work);
	struct device_node *node = oplus_get_node_by_type(wls_dev->dev->of_node);
	static int retry = OPLUS_CHG_IC_INIT_RETRY_MAX;
	int rc;
	bool usb_present;

	wls_dev->wls_rx->rx_ic = of_get_oplus_chg_ic(node, "oplus,rx_ic", 0);
	if (wls_dev->wls_rx->rx_ic == NULL) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&wls_dev->wls_mms_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("oplus,rx_ic not found\n");
		}
		retry = 0;
		return;
	}

	/*wls topic init after rx ic and before nor ic, becase nor ic init very, very late in adsp project*/
	if (wls_dev->wls_topic == NULL)
		(void)oplus_chg_wls_topic_init(wls_dev);

	wls_dev->wls_nor->nor_ic = of_get_oplus_chg_ic(node, "oplus,nor_ic", 0);
	if (wls_dev->wls_nor->nor_ic == NULL) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&wls_dev->wls_mms_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("oplus,nor_ic not found\n");
		}
		retry = 0;
		return;
	}

	if (wls_dev->support_fastchg) {
		wls_dev->wls_fast->fast_ic = of_get_oplus_chg_ic(node, "oplus,fast_ic", 0);
		if (wls_dev->wls_fast->fast_ic == NULL) {
			if (retry > 0) {
				retry--;
				schedule_delayed_work(&wls_dev->wls_mms_init_work,
					msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
				return;
			} else {
				chg_err("oplus,fast_ic not found\n");
			}
			retry = 0;
			return;
		}
	}

	/*virq_register need before OPLUS_IC_FUNC_INIT*/
	(void)oplus_chg_wls_virq_register(wls_dev);

	/*need nor_ic init done before rx_ic init*/
	rc = wait_for_completion_timeout(&wls_dev->nor_ic_ack, msecs_to_jiffies(NOR_IC_INIT_TIMEOUT_MS));
	if (!rc) {
		chg_err("nor_ic init timedout\n");
		return;
	}

	rc = oplus_chg_ic_func(wls_dev->wls_rx->rx_ic, OPLUS_IC_FUNC_INIT);
	if (rc == -EAGAIN) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&wls_dev->wls_mms_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("rx_ic init timeout\n");
		}
		retry = 0;
		return;
	} else if (rc < 0) {
		chg_err("rx_ic init error, rc=%d\n", rc);
		retry = 0;
		return;
	}
	retry = 0;
	chg_info("wls topic OPLUS_IC_FUNC_INIT success\n");

	usb_present = oplus_chg_wls_is_usb_present(wls_dev);
	if (usb_present) {
		wls_dev->usb_present = true;
		schedule_delayed_work(&wls_dev->usb_int_work, 0);
	} else {
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
		if (oplus_is_power_off_charging())
			wls_dev->wls_status.boot_online_keep = true;
#endif
	}

	schedule_delayed_work(&wls_dev->wls_upgrade_fw_work, msecs_to_jiffies(1000));
	schedule_delayed_work(&wls_dev->wls_init_complete_work, msecs_to_jiffies(WLS_INIT_COMPLETE_DELAY_MS));
	return;
}

#if IS_ENABLED(CONFIG_OPLUS_MAGCVR_NOTIFY)
static void magcvr_notifier_handler(struct oplus_chg_wls *wls_dev)
{
	struct oplus_chg_wls_status *wls_status;
	struct oplus_chg_wls_dynamic_config *dynamic_cfg;

	if (wls_dev == NULL)
		return;

	wls_status = &wls_dev->wls_status;
	dynamic_cfg = &wls_dev->dynamic_config;

	if (!wls_status->rx_online)
		return;

	if (wls_dev->rx_protocol_version >= WLS_RX_PROTOCOL_VERSION_30 &&
	    wls_status->adapter_id >= WLS_ADAPTER_MODEL_4 &&
	    wls_status->adapter_id != WLS_ADAPTER_THIRD_PARTY) {
		chg_info("pwr_max_mw=%d, non_mag_power_mw=%d\n", wls_status->pwr_max_mw, wls_dev->non_mag_power_mw);
		if (wls_status->tx_mag == TX_MAG_TEC || wls_status->tx_mag == TX_MAG_NON_TEC) {
			if (wls_dev->magcvr_status == MAGCVR_STATUS_FAR) {
				wls_status->pwr_max_mw = wls_status->pwr_max_mw > wls_dev->non_mag_power_mw ?
							wls_dev->non_mag_power_mw : wls_status->pwr_max_mw;

				if (wls_status->pwr_max_mw > 0)
					vote(wls_dev->fcc_votable, RX_MAX_VOTER, true,
						(wls_status->pwr_max_mw * 1000 / dynamic_cfg->svooc_vol_mv), false);
			}
		}
	}

	return;
}
#endif

static int magcvr_notifier_call(struct notifier_block *nb, unsigned long action, void *data)
{
#if IS_ENABLED(CONFIG_OPLUS_MAGCVR_NOTIFY)
	struct oplus_chg_wls *wls_dev = container_of(nb, struct oplus_chg_wls, magcvr_notify);
	struct magcvr_notify_event *event_data = data;
	int magcvr_status;

	if (action != EVENT_ACTION_FOR_MAGCVR || !event_data)
		return NOTIFY_DONE;

	magcvr_status = event_data->type;
	chg_info("magcvr notifier event: magcvr_status[%d]", magcvr_status);
	wls_dev->magcvr_update_complete = true;
	if (magcvr_status == wls_dev->magcvr_status)
		return NOTIFY_OK;
	switch (magcvr_status) {
	case MAGCVR_CALL_NEAR:
	case MAGCVR_CALL_IIC_FAIL:
	case MAGCVR_CALL_IC_FAIL:
		wls_dev->magcvr_status = MAGCVR_STATUS_NEAR;
		break;
	case  MAGCVR_CALL_FAR:
		wls_dev->magcvr_status = MAGCVR_STATUS_FAR;
		break;
	default:
		break;
	}

	magcvr_notifier_handler(wls_dev);

	return NOTIFY_OK;
#else
	return NOTIFY_DONE;
#endif
}

static int get_init_magcvr_status(void)
{
	int magcvr_status;

	magcvr_status = magcvr_get_current_pos();
	/* Hall driver not init || NEAR or FAIL */
	if (magcvr_status < 0 || magcvr_status != MAGCVR_STATUS_FAR)
		return MAGCVR_STATUS_NEAR;
	return MAGCVR_STATUS_FAR;
}

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "config/dynamic_cfg/oplus_wls_cfg.h"
#endif

static int oplus_chg_wls_aes_mutual_notifier_call(
		struct notifier_block *nb, unsigned long param, void *v)
{
	int i;
	struct oplus_chg_wls *wls_dev;
	struct oplus_chg_wls_status *wls_status;
	struct wls_third_part_auth_result *aes_auth_result;
	struct oplus_chg_mutual_notifier *notifier;

	notifier = container_of(nb, struct oplus_chg_mutual_notifier, nb);
	wls_dev = container_of(notifier, struct oplus_chg_wls, aes_mutual);
	wls_status = &wls_dev->wls_status;

	if (mutual_info_to_cmd(param) != CMD_WLS_THIRD_PART_AUTH) {
		chg_err("cmd is not matching, should return\n");
		return NOTIFY_OK;
	}

	if (mutual_info_to_data_size(param) != sizeof(struct wls_third_part_auth_result)) {
		chg_err("data_len is not ok, datas is invalid\n");
		return NOTIFY_DONE;
	}

	aes_auth_result = (struct wls_third_part_auth_result *)v;
	if (aes_auth_result) {
		wls_status->aes_key_num = aes_auth_result->effc_key_index;
		chg_info("aes_key_num:%d\n", wls_status->aes_key_num);

		memcpy(wls_status->aes_verfity_data.aes_random_num, aes_auth_result->aes_random_num,
			sizeof(aes_auth_result->aes_random_num));
		for (i = 0; i < WLS_AUTH_AES_ENCODE_LEN; i++)
			chg_info("aes_random_num[%d]:0x%02x\n", i, (wls_status->aes_verfity_data.aes_random_num)[i]);

		memcpy(wls_status->aes_verfity_data.aes_encode_num, aes_auth_result->aes_encode_num,
			sizeof(aes_auth_result->aes_encode_num));
		wls_status->aes_verity_data_ok = true;
		for (i = 0; i < WLS_AUTH_AES_ENCODE_LEN; i++)
			chg_info("aes_encode_num[%d]:0x%02x ", i, (wls_status->aes_verfity_data.aes_encode_num)[i]);
	}

	return NOTIFY_OK;
}

static int oplus_chg_wls_aes_mutual_notify_reg(struct oplus_chg_wls *wls_dev)
{
	int rc = 0;

	wls_dev->aes_mutual.name = "wls_aes";
	wls_dev->aes_mutual.cmd = CMD_WLS_THIRD_PART_AUTH;
	wls_dev->aes_mutual.nb.notifier_call = oplus_chg_wls_aes_mutual_notifier_call;

	rc = oplus_chg_reg_mutual_notifier(&wls_dev->aes_mutual);
	if (rc < 0) {
		chg_err("register aes mutual event notifier error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int oplus_chg_wls_driver_probe(struct platform_device *pdev)
{
	struct oplus_chg_wls *wls_dev;
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int boot_mode;
#endif
	int rc;

	wls_dev = devm_kzalloc(&pdev->dev, sizeof(struct oplus_chg_wls), GFP_KERNEL);
	if (wls_dev == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}

	wls_dev->dev = &pdev->dev;
	platform_set_drvdata(pdev, wls_dev);

	rc = oplus_chg_wls_parse_dt(wls_dev);
	if (rc < 0) {
		chg_err("oplus chg wls parse dts error, rc=%d\n", rc);
		goto parse_dt_err;
	}

	wls_dev->wls_wq = alloc_ordered_workqueue("wls_wq", WQ_FREEZABLE | WQ_HIGHPRI);
	if (!wls_dev->wls_wq) {
		chg_err("alloc wls work error\n");
		rc = -ENOMEM;
		goto alloc_work_err;
	}

	rc = oplus_chg_wls_rx_init(wls_dev);
	if (rc < 0) {
		goto rx_init_err;
	}
	rc = oplus_chg_wls_rx_register_msg_callback(wls_dev->wls_rx->rx_ic, wls_dev,
		oplus_chg_wls_rx_msg_callback);
	if (!rc)
		wls_dev->msg_callback_ok = true;
	rc = oplus_chg_wls_nor_init(wls_dev);
	if (rc < 0) {
		goto nor_init_err;
	}
	if (wls_dev->support_fastchg) {
		rc = oplus_chg_wls_fast_init(wls_dev);
		if (rc < 0) {
			goto fast_init_err;
		}
	}

	rc = oplus_chg_wls_gpio_init(wls_dev);
	if (rc < 0)
		goto gpio_init_err;

	wls_dev->fcc_votable = create_votable("WLS_FCC", VOTE_MIN,
				oplus_chg_wls_fcc_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->fcc_votable)) {
		rc = PTR_ERR(wls_dev->fcc_votable);
		wls_dev->fcc_votable = NULL;
		goto create_fcc_votable_err;
	}

	wls_dev->fastchg_disable_votable = create_votable("WLS_FASTCHG_DISABLE",
				VOTE_SET_ANY,
				oplus_chg_wls_fastchg_disable_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->fastchg_disable_votable)) {
		rc = PTR_ERR(wls_dev->fastchg_disable_votable);
		wls_dev->fastchg_disable_votable = NULL;
		goto create_disable_votable_err;
	}
	wls_dev->wrx_en_votable = create_votable("WLS_WRX_EN",
				VOTE_SET_ANY,
				oplus_chg_wls_wrx_en_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->wrx_en_votable)) {
		rc = PTR_ERR(wls_dev->wrx_en_votable);
		wls_dev->wrx_en_votable = NULL;
		goto create_wrx_en_votable_err;
	}
	wls_dev->nor_icl_votable = create_votable("WLS_NOR_ICL",
				VOTE_MIN,
				oplus_chg_wls_nor_icl_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->nor_icl_votable)) {
		rc = PTR_ERR(wls_dev->nor_icl_votable);
		wls_dev->nor_icl_votable = NULL;
		goto create_nor_icl_votable_err;
	}
	wls_dev->nor_fcc_votable = create_votable("WLS_NOR_FCC",
				VOTE_MIN,
				oplus_chg_wls_nor_fcc_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->nor_fcc_votable)) {
		rc = PTR_ERR(wls_dev->nor_fcc_votable);
		wls_dev->nor_fcc_votable = NULL;
		goto create_nor_fcc_votable_err;
	}
	/*wls_dev->nor_fv_votable = create_votable("WLS_NOR_FV",
				VOTE_MIN,
				oplus_chg_wls_nor_fv_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->nor_fv_votable)) {
		rc = PTR_ERR(wls_dev->nor_fv_votable);
		wls_dev->nor_fv_votable = NULL;
		goto create_nor_fv_votable_err;
	}*/
	wls_dev->nor_out_disable_votable = create_votable("WLS_NOR_OUT_DISABLE",
				VOTE_SET_ANY,
				oplus_chg_wls_nor_out_disable_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->nor_out_disable_votable)) {
		rc = PTR_ERR(wls_dev->nor_out_disable_votable);
		wls_dev->nor_out_disable_votable = NULL;
		goto create_nor_out_disable_votable_err;
	}
	wls_dev->nor_input_disable_votable = create_votable("WLS_NOR_INPUT_DISABLE",
				VOTE_SET_ANY,
				oplus_chg_wls_nor_input_disable_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->nor_input_disable_votable)) {
		rc = PTR_ERR(wls_dev->nor_input_disable_votable);
		wls_dev->nor_input_disable_votable = NULL;
		goto create_nor_input_disable_votable_err;
	}
	wls_dev->rx_disable_votable = create_votable("WLS_RX_DISABLE",
				VOTE_SET_ANY,
				oplus_chg_wls_rx_disable_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->rx_disable_votable)) {
		rc = PTR_ERR(wls_dev->rx_disable_votable);
		wls_dev->rx_disable_votable = NULL;
		goto create_rx_disable_votable_err;
	}
	wls_dev->insert_disable_votable = create_votable("WLS_INSERT_DISABLE",
				VOTE_SET_ANY,
				oplus_chg_wls_insert_disable_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->insert_disable_votable)) {
		rc = PTR_ERR(wls_dev->insert_disable_votable);
		wls_dev->insert_disable_votable = NULL;
		goto create_insert_disable_votable_err;
	}

	wls_dev->quiet_mode_votable = create_votable("WLS_QUIET_MODE",
				VOTE_SET_ANY,
				oplus_chg_wls_quiet_mode_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->quiet_mode_votable)) {
		rc = PTR_ERR(wls_dev->quiet_mode_votable);
		wls_dev->quiet_mode_votable = NULL;
		goto create_quiet_mode_votable_err;
	}

	wls_dev->rx_comu_votable = create_votable("WLS_COMU",
				VOTE_MAX,
				oplus_chg_wls_comu_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->rx_comu_votable)) {
		rc = PTR_ERR(wls_dev->rx_comu_votable);
		wls_dev->rx_comu_votable = NULL;
		goto create_rx_comu_votable_err;
	}

	wls_dev->force_bpp_mode_votable = create_votable("WLS_FORCE_BPP_MODE",
				VOTE_SET_ANY,
				oplus_chg_wls_force_bpp_mode_vote_callback,
				wls_dev);
	if (IS_ERR(wls_dev->force_bpp_mode_votable)) {
		rc = PTR_ERR(wls_dev->force_bpp_mode_votable);
		wls_dev->force_bpp_mode_votable = NULL;
		goto create_force_bpp_mode_votable_err;
	}

	wls_dev->misc_dev.minor = MISC_DYNAMIC_MINOR;
	wls_dev->misc_dev.name = "wls_dev";
	wls_dev->misc_dev.fops = &oplus_chg_wls_dev_fops;
	rc = misc_register(&wls_dev->misc_dev);
	if (rc) {
		chg_err("misc_register failed, rc=%d\n", rc);
		goto misc_reg_err;
	}

	rc = oplus_chg_wls_init_charge_proc(wls_dev);
	if (rc < 0) {
		chg_err("can't init charge proc, rc=%d\n", rc);
		goto init_charge_proc_err;
	}

	rc = oplus_chg_wls_aes_mutual_notify_reg(wls_dev);
	if (rc < 0)
		goto aes_mutual_nb_reg_err;

	wls_dev->magcvr_status = get_init_magcvr_status();
	wls_dev->magcvr_update_complete = false;
	wls_dev->magcvr_notify.notifier_call = magcvr_notifier_call;
	magcvr_event_register_notifier(&wls_dev->magcvr_notify);

	wls_dev->force_rx_mode = OPLUS_CHG_WLS_RX_MODE_UNKNOWN;

	INIT_DELAYED_WORK(&wls_dev->wls_rx_sm_work, oplus_chg_wls_rx_sm);
	INIT_DELAYED_WORK(&wls_dev->wls_trx_sm_work, oplus_chg_wls_trx_sm);
	INIT_DELAYED_WORK(&wls_dev->wls_connect_work, oplus_chg_wls_connect_work);
	INIT_DELAYED_WORK(&wls_dev->wls_monitor_work, oplus_chg_wls_monitor_work);
	INIT_DELAYED_WORK(&wls_dev->rx_mode_check_work, oplus_chg_rx_mode_check_work);
	INIT_DELAYED_WORK(&wls_dev->wls_match_q_work, oplus_chg_send_match_q_work);
	INIT_DELAYED_WORK(&wls_dev->wls_set_tec_work, oplus_chg_wls_set_tec_work);
	INIT_DELAYED_WORK(&wls_dev->wls_send_msg_work, oplus_chg_wls_send_msg_work);
	INIT_DELAYED_WORK(&wls_dev->wls_upgrade_fw_work, oplus_chg_wls_upgrade_fw_work);
	INIT_DELAYED_WORK(&wls_dev->wls_init_complete_work, oplus_chg_wls_init_complete_work);
	INIT_DELAYED_WORK(&wls_dev->usb_int_work, oplus_chg_wls_usb_int_work);
	INIT_DELAYED_WORK(&wls_dev->vac_int_work, oplus_chg_wls_vac_int_work);
	INIT_DELAYED_WORK(&wls_dev->wls_data_update_work, oplus_chg_wls_data_update_work);
	INIT_DELAYED_WORK(&wls_dev->wls_trx_disconnect_work, oplus_chg_wls_trx_disconnect_work);
	INIT_DELAYED_WORK(&wls_dev->usb_connect_work, oplus_chg_wls_usb_connect_work);
	INIT_DELAYED_WORK(&wls_dev->wls_start_work, oplus_chg_wls_start_work);
	INIT_DELAYED_WORK(&wls_dev->wls_break_work, oplus_chg_wls_break_work);
	INIT_DELAYED_WORK(&wls_dev->fod_cal_work, oplus_chg_wls_fod_cal_work);
	INIT_DELAYED_WORK(&wls_dev->rx_restore_work, oplus_chg_wls_rx_restore_work);
	INIT_DELAYED_WORK(&wls_dev->rx_iic_restore_work, oplus_chg_wls_rx_iic_restore_work);
	INIT_DELAYED_WORK(&wls_dev->rx_verity_restore_work, oplus_chg_wls_rx_verity_restore_work);
	INIT_DELAYED_WORK(&wls_dev->fast_fault_check_work, oplus_chg_wls_fast_fault_check_work);
	INIT_DELAYED_WORK(&wls_dev->rx_restart_work, oplus_chg_wls_rx_restart_work);
	INIT_DELAYED_WORK(&wls_dev->online_keep_remove_work, oplus_chg_wls_online_keep_remove_work);
	INIT_DELAYED_WORK(&wls_dev->verity_state_remove_work, oplus_chg_wls_verity_state_remove_work);
	INIT_DELAYED_WORK(&wls_dev->wls_verity_work, oplus_chg_wls_verity_work);
	INIT_DELAYED_WORK(&wls_dev->wls_get_third_part_verity_data_work,
		oplus_chg_wls_get_third_part_verity_data_work);
	INIT_DELAYED_WORK(&wls_dev->wls_clear_trx_work, oplus_chg_wls_clear_trx_work);
	INIT_DELAYED_WORK(&wls_dev->wls_skewing_work, oplus_chg_wls_skewing_work);
	INIT_DELAYED_WORK(&wls_dev->wls_bcc_curr_update_work, oplus_chg_wls_bcc_curr_update_work);
	INIT_DELAYED_WORK(&wls_dev->wls_vout_err_work, oplus_chg_wls_vout_err_work);
	INIT_DELAYED_WORK(&wls_dev->wls_mms_init_work, oplus_chg_wls_mms_init_work);
	INIT_DELAYED_WORK(&wls_dev->wls_otg_enable_work, oplus_chg_wls_otg_enable_item_work);
	INIT_DELAYED_WORK(&wls_dev->wls_ploss_warn_work, oplus_chg_wls_ploss_warn_work);
	INIT_DELAYED_WORK(&wls_dev->wls_high_temp_track_work, oplus_chg_wls_high_temp_track_work);
	INIT_DELAYED_WORK(&wls_dev->wls_epp_check_flag_clear_work, oplus_chg_wls_epp_force_to_bpp_flag_clear_work);
	INIT_DELAYED_WORK(&wls_dev->wls_fastchg_to_bpp_keep_state_remove_work,
			  oplus_chg_wls_fastchg_to_bpp_keep_state_remove_work);
	INIT_DELAYED_WORK(&wls_dev->wls_epp_iout_too_small_check_work, oplus_chg_wls_epp_iout_too_small_check_work);
	INIT_DELAYED_WORK(&wls_dev->wls_boot_quiet_clear_work, oplus_chg_wls_boot_quiet_clear_work);
	INIT_WORK(&wls_dev->wls_err_handler_work, oplus_chg_wls_err_handler_work);
	INIT_WORK(&wls_dev->wls_present_handler_work, oplus_chg_wls_present_handler_work);
	INIT_WORK(&wls_dev->wls_online_handler_work, oplus_chg_wls_online_handler_work);
	INIT_WORK(&wls_dev->wls_offline_handler_work, oplus_chg_wls_offline_handler_work);
	INIT_WORK(&wls_dev->wls_event_changed_handler_work, oplus_chg_wls_event_changed_handler_work);
	INIT_WORK(&wls_dev->temp_region_update_work, oplus_chg_wls_temp_region_update_work);
	INIT_WORK(&wls_dev->fcc_gear_update_work, oplus_chg_wls_fcc_gear_update_work);
	INIT_WORK(&wls_dev->cool_down_update_work, oplus_chg_wls_cool_down_update_work);
	INIT_WORK(&wls_dev->batt_bal_curr_limit_work, oplus_chg_wls_batt_bal_curr_limit_work);
	INIT_WORK(&wls_dev->adapter_curve_vote_work, oplus_chg_wls_adapter_curve_vote_work);
	init_completion(&wls_dev->msg_ack);
	init_completion(&wls_dev->nor_ic_ack);
	mutex_init(&wls_dev->connect_lock);
	mutex_init(&wls_dev->read_lock);
	mutex_init(&wls_dev->cmd_data_lock);
	mutex_init(&wls_dev->send_msg_lock);
	mutex_init(&wls_dev->update_data_lock);
	mutex_init(&wls_dev->tx_switch_lock);
	mutex_init(&wls_dev->trx_enable_lock);
	init_waitqueue_head(&wls_dev->read_wq);

	wls_dev->rx_wake_lock = wakeup_source_register(wls_dev->dev, "rx_wake_lock");
	wls_dev->trx_wake_lock = wakeup_source_register(wls_dev->dev, "trx_wake_lock");

	wls_dev->charge_enable = true;
	wls_dev->batt_charge_enable = true;
	wls_dev->ploss_interval_jiffies = jiffies;

	hrtimer_init(&wls_dev->wls_status.wls_adapter_curve_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	wls_dev->wls_status.wls_adapter_curve_timer.function = &wls_adapter_curve_timer_callback;
	wls_dev->wls_status.wls_adapter_curve_gear_cnt = 0;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	boot_mode = get_boot_mode();
#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (boot_mode == MSM_BOOT_MODE__FACTORY)
#else
	if (boot_mode == FACTORY_BOOT || boot_mode == ATE_FACTORY_BOOT)
#endif
		wls_dev->ftm_mode = true;
#endif

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	(void)oplus_wls_reg_debug_config(wls_dev);
#endif

	oplus_mms_wait_topic("gauge", oplus_chg_wls_subscribe_gauge_topic, wls_dev);
	oplus_mms_wait_topic("wired", oplus_chg_wls_subscribe_wired_topic, wls_dev);
	oplus_mms_wait_topic("common", oplus_chg_wls_subscribe_comm_topic, wls_dev);
	oplus_mms_wait_topic("batt_bal", oplus_chg_wls_subscribe_batt_bal_topic, wls_dev);
	schedule_delayed_work(&wls_dev->wls_mms_init_work, 0);

	if (wls_dev->dynamic_config.boot_quiet_t > 0) {
		vote(wls_dev->quiet_mode_votable, WLS_BOOT_QUIET_VOTER, true, 0, false);
		schedule_delayed_work(&wls_dev->wls_boot_quiet_clear_work, msecs_to_jiffies(wls_dev->dynamic_config.boot_quiet_t * 1000));
	}

	chg_info("probe done\n");

	return 0;

aes_mutual_nb_reg_err:
	remove_proc_entry("wireless", NULL);
init_charge_proc_err:
	misc_deregister(&wls_dev->misc_dev);

misc_reg_err:
	destroy_votable(wls_dev->quiet_mode_votable);
create_quiet_mode_votable_err :
	destroy_votable(wls_dev->insert_disable_votable);
create_insert_disable_votable_err:
	destroy_votable(wls_dev->rx_disable_votable);
create_rx_disable_votable_err:
	destroy_votable(wls_dev->nor_input_disable_votable);
create_nor_input_disable_votable_err:
	destroy_votable(wls_dev->nor_out_disable_votable);
create_nor_out_disable_votable_err:
	/*destroy_votable(wls_dev->nor_fv_votable);
create_nor_fv_votable_err:*/
	destroy_votable(wls_dev->nor_fcc_votable);
create_nor_fcc_votable_err:
	destroy_votable(wls_dev->nor_icl_votable);
create_nor_icl_votable_err:
	destroy_votable(wls_dev->wrx_en_votable);
create_wrx_en_votable_err:
	destroy_votable(wls_dev->fastchg_disable_votable);
create_disable_votable_err:
	destroy_votable(wls_dev->fcc_votable);
create_fcc_votable_err:
	disable_irq(wls_dev->usb_int_irq);
	if (gpio_is_valid(wls_dev->usb_int_gpio))
		gpio_free(wls_dev->usb_int_gpio);
	if (gpio_is_valid(wls_dev->ext_pwr_en_gpio))
		gpio_free(wls_dev->ext_pwr_en_gpio);
	if (gpio_is_valid(wls_dev->wrx_en_gpio))
		gpio_free(wls_dev->wrx_en_gpio);
create_rx_comu_votable_err:
	destroy_votable(wls_dev->rx_comu_votable);
create_force_bpp_mode_votable_err:
	destroy_votable(wls_dev->force_bpp_mode_votable);
gpio_init_err:
	if (wls_dev->support_fastchg)
		oplus_chg_wls_fast_remove(wls_dev);
fast_init_err:
	oplus_chg_wls_nor_remove(wls_dev);
nor_init_err:
	oplus_chg_wls_rx_remove(wls_dev);
rx_init_err:
	destroy_workqueue(wls_dev->wls_wq);
alloc_work_err:
parse_dt_err:
	devm_kfree(&pdev->dev, wls_dev);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_chg_wls_driver_remove(struct platform_device *pdev)
#else
static int oplus_chg_wls_driver_remove(struct platform_device *pdev)
#endif
{
	struct oplus_chg_wls *wls_dev = platform_get_drvdata(pdev);

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	oplus_wls_unreg_debug_config(wls_dev);
#endif
	oplus_chg_unreg_mutual_notifier(&wls_dev->aes_mutual);

	remove_proc_entry("wireless", NULL);

	if (!IS_ERR_OR_NULL(wls_dev->comm_subs))
		oplus_mms_unsubscribe(wls_dev->comm_subs);
	if (!IS_ERR_OR_NULL(wls_dev->wired_subs))
		oplus_mms_unsubscribe(wls_dev->wired_subs);
	if (!IS_ERR_OR_NULL(wls_dev->gauge_subs))
		oplus_mms_unsubscribe(wls_dev->gauge_subs);
	if (!IS_ERR_OR_NULL(wls_dev->wls_subs))
		oplus_mms_unsubscribe(wls_dev->wls_subs);

	magcvr_event_unregister_notifier(&wls_dev->magcvr_notify);
	misc_deregister(&wls_dev->misc_dev);
	destroy_votable(wls_dev->insert_disable_votable);
	destroy_votable(wls_dev->quiet_mode_votable);
	destroy_votable(wls_dev->rx_disable_votable);
	destroy_votable(wls_dev->nor_out_disable_votable);
	destroy_votable(wls_dev->nor_input_disable_votable);
	/*destroy_votable(wls_dev->nor_fv_votable);*/
	destroy_votable(wls_dev->nor_fcc_votable);
	destroy_votable(wls_dev->nor_icl_votable);
	destroy_votable(wls_dev->wrx_en_votable);
	destroy_votable(wls_dev->fastchg_disable_votable);
	destroy_votable(wls_dev->fcc_votable);
	disable_irq(wls_dev->usb_int_irq);
	if (gpio_is_valid(wls_dev->usb_int_gpio))
		gpio_free(wls_dev->usb_int_gpio);
	if (gpio_is_valid(wls_dev->ext_pwr_en_gpio))
		gpio_free(wls_dev->ext_pwr_en_gpio);
	if (gpio_is_valid(wls_dev->wrx_en_gpio))
		gpio_free(wls_dev->wrx_en_gpio);
	oplus_wls_adapter_curve_timer_stop(wls_dev);
	if (wls_dev->support_fastchg)
		oplus_chg_wls_fast_remove(wls_dev);
	oplus_chg_wls_nor_remove(wls_dev);
	oplus_chg_wls_rx_remove(wls_dev);
	destroy_workqueue(wls_dev->wls_wq);
	devm_kfree(&pdev->dev, wls_dev);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static void oplus_chg_wls_driver_shutdown(struct platform_device *pdev)
{
	struct oplus_chg_wls *wls_dev = platform_get_drvdata(pdev);

	if (wls_dev && wls_dev->wls_status.wls_type == OPLUS_CHG_WLS_TRX)
		(void)oplus_chg_wls_set_trx_enable(wls_dev, false);
}

static const struct of_device_id oplus_chg_wls_match[] = {
	{ .compatible = "oplus,chg_wls" },
	{},
};

static struct platform_driver oplus_chg_wls_driver = {
	.driver = {
		.name = "oplus-chg_wls",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_chg_wls_match),
	},
	.probe		= oplus_chg_wls_driver_probe,
	.remove		= oplus_chg_wls_driver_remove,
	.shutdown	= oplus_chg_wls_driver_shutdown,
};

static __init int oplus_chg_wls_driver_init(void)
{
	return platform_driver_register(&oplus_chg_wls_driver);
}

static __exit void oplus_chg_wls_driver_exit(void)
{
	platform_driver_unregister(&oplus_chg_wls_driver);
}

oplus_chg_module_register(oplus_chg_wls_driver);
