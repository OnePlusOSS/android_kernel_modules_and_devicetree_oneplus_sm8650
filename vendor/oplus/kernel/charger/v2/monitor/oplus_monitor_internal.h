#ifndef __OPLUS_MONITOR_INTERNAL_H__
#define __OPLUS_MONITOR_INTERNAL_H__

#include <linux/device.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_monitor.h>
#include "oplus_chg_track.h"


struct dischg_avg {
	int tbat;
	int tsub;
	int tshell;
	int ibat;
	int vbat;
};
#define DEEP_DISCHG_AVG_PROFILE_SIZE		36

struct deep_dischg_profile {
	int32_t vbat_10;
	int32_t vbat_5;
	int32_t vbat_avg;

	int32_t tbat_max;
	int32_t tbat_min;
	int32_t tbat_avg;

	int32_t ibat_max;
	int32_t ibat_min;
	int32_t ibat_avg;

	int32_t tsub_max;
	int32_t tsub_min;
	int32_t tsub_avg;

	int32_t tshell_max;
	int32_t tshell_min;
	int32_t tshell_avg;

	int32_t tbat_now;
	int32_t tsub_now;
	int32_t tshell_now;
	int32_t vmax0;
	int32_t vmin0;
	int32_t tsoc0;
	int32_t sicc;

	int32_t vbat_max;
	int32_t vbat_min;
	int32_t ui_soc;
	int32_t soc;
	struct dischg_avg profile_avg[DEEP_DISCHG_AVG_PROFILE_SIZE];
	int32_t index;
	int32_t upload;

	int32_t vbat_term;
	int32_t dod1;
	int32_t dod2;
	int32_t cc;
	int32_t counts;
	int32_t ratio;
	int32_t time;
	unsigned long init_jiffies;
};

struct endurance_track_info {
	int time;
	int batt_temp;
	int batt_rm;
	int soc;
	int ui_soc;
	int vol_max;
	int vol_min;
	int batt_fcc;
};

struct super_endurance_mode_info {
	struct endurance_track_info start_info;
	struct endurance_track_info end_info;

	int duration_time;
	int exit_reason;
	int count;
	bool uisoc_0;
	bool pre_status;
	bool status;
};

struct oplus_monitor {
	struct device *dev;
	struct oplus_mms *err_topic;
	struct oplus_mms *wired_topic;
	struct oplus_mms *wls_topic;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *vooc_topic;
	struct oplus_mms *comm_topic;
	struct oplus_mms *main_gauge_topic;
	struct oplus_mms *sub_gauge_topic;
	struct oplus_mms *dual_chan_topic;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *wls_subs;
	struct mms_subscribe *gauge_subs;
	struct mms_subscribe *vooc_subs;
	struct mms_subscribe *dual_chan_subs;
	struct mms_subscribe *comm_subs;
	struct oplus_mms *ufcs_topic;
	struct oplus_mms *retention_topic;
	struct mms_subscribe *ufcs_subs;
	struct mms_subscribe *retention_subs;
	struct oplus_mms *plc_topic;
	struct mms_subscribe *plc_subs;
	struct oplus_mms *pps_topic;
	struct mms_subscribe *keep_subs;
	struct oplus_mms *keep_topic;

	struct oplus_chg_track *track;

	struct work_struct charge_info_update_work;
	struct work_struct wired_plugin_work;
	struct work_struct wired_present_work;
	struct work_struct ffc_step_change_work;
	struct work_struct ffc_end_work;
#if IS_ENABLED(CONFIG_OPLUS_FPGA_NOTIFY)
	struct work_struct fgpa_reset_start_work;
	struct work_struct fgpa_reset_end_work;
	struct work_struct gauge_i2c_reset_work;
	struct notifier_block nb;
	struct notifier_block i2c_nb;
#endif
	int fpga_reset_status;
	int i2c_reset_status;
	struct votable *fv_votable;
	struct delayed_work water_inlet_detect_work;
	struct delayed_work water_inlet_clear_work;
	struct delayed_work chg_into_liquid_trigger_work_timeout;
	struct delayed_work dischg_profile_update_work;
	struct delayed_work dischg_profile_check_work;

	struct votable *wired_icl_votable;
	struct votable *wired_fcc_votable;
	struct votable *wired_charge_suspend_votable;
	struct votable *wired_charging_disable_votable;
	struct votable *wls_icl_votable;
	struct votable *wls_fcc_votable;
	struct votable *wls_charge_suspend_votable;
	struct votable *wls_charging_disable_votable;
	struct votable *wls_rx_disable_votable;
	struct votable *chg_disable_votable;
	struct votable *vooc_curr_votable;

	struct deep_dischg_profile dischg_profile;

	/* battery */
	int vbat_mv;
	int vbat_min_mv;
	int ibat_ma;
	int batt_temp;
	int shell_temp;
	int batt_fcc;
	int batt_rm;
	int batt_cc;
	int batt_soh;
	int batt_soc;
	int ui_soc;
	int smooth_soc;
	int delta_soc;
	int soc_load;
	int batt_status;
	bool batt_exist;
	bool batt_full;
	unsigned int batt_err_code;
	int batt_fcc_coeff;
	int batt_soh_coeff;
	int batt_fcc_comp;
	int batt_soh_comp;
	int uisoc_keep_2_err;
	int batt_qmax;
	int gauge_car_c;
	struct super_endurance_mode_info sem_info;
	bool gauge_inited;
	int sub_vbat;
	int sub_soc;
	int sub_temp;
	int sub_ibat;
	int main_vbat;
	int main_soc;
	int main_temp;
	int main_ibat;

	/* charge */
	int fcc_ma;
	int fv_mv;
	int total_time;
	bool chg_disable;
	bool chg_user_disable;
	bool sw_full;
	bool hw_full_by_sw;
	bool hw_full;

	/* wired */
	int wired_ibus_ma;
	int wired_vbus_mv;
	int wired_icl_ma;
	int wired_charge_type;
	int cc_mode;
	unsigned int wired_err_code;
	bool wired_online;
	bool wired_suspend;
	bool wired_user_suspend;
	int cc_detect;
	bool otg_enable;
	bool pd_svooc;
	int vbus_vol_type;

	/* wireless */
	int wls_iout_ma;
	int wls_vout_mv;
	int wls_icl_ma;
	int wls_charge_type;
	int wls_pre_type;
	int wls_dock_type;
	int wls_magcvr_status;
	unsigned int wls_err_code;
	bool wls_online;
	bool wls_suspend;
	bool wls_user_suspend;
	int tx_manu_id;
	int vendor_id;
	int product_id;

	/* common */
	enum oplus_temp_region temp_region;
	enum oplus_chg_ffc_status ffc_status;
	int cool_down;
	int normal_cool_down;
	unsigned int notify_code;
	unsigned int notify_flag;
	bool led_on;
	bool rechging;
	bool ui_soc_ready;
	int chg_cycle_status;
	bool slow_chg_enable;
	int slow_chg_pct;
	int slow_chg_watt;
	int bcc_current;
	int mmi_chg;
	int usb_status;
	bool otg_switch_status;
	bool deep_support;

	/* vooc */
	bool vooc_online;
	bool vooc_started;
	bool vooc_charging;
	bool vooc_online_keep;
	unsigned vooc_sid;
	unsigned pre_vooc_sid;
	bool chg_ctrl_by_vooc;
	int vooc_normal_connect_count_level;

	/* ufcs */
	bool ufcs_online;
	bool ufcs_charging;
	u32 ufcs_adapter_id;
	bool ufcs_oplus_adapter;
	int ufcs_emark_power;
	int ufcs_adapter_power;

	u32 pre_ufcs_adapter_id;
	bool pre_ufcs_oplus_adapter;
	int pre_ufcs_emark_power;
	int pre_ufcs_adapter_power;

	/* plc */
	int plc_status;
	bool plc_support;
	int enable_count;
	int plc_init_sm_soc;
	int plc_init_ui_soc;
	int plc_init_temp;

	/* chg into liqued*/
	int cc_state;
	bool liquid_inlet_detection_switch;
	int water_inlet_plugin_count;
	bool oplus_liquid_intake_enable;
	bool oplus_liquid_intake_led_status;
	bool oplus_liquid_intake_check_led_status;
	int chg_into_liquid_cc_disconnect;
	int chg_into_liquid_total_time;
	int chg_into_liquid_max_interval_time;

	int rechg_soc_en;
	int rechg_soc_threshold;

	/* retention */
	bool retention_state;
	bool pre_retention_state;
	int total_disconnect_count;

	/* state keep */
	bool switch_protocol;
	bool keep_wired_online;
};

struct oplus_chg_into_l{
	unsigned long long time;
	int cid_status;
};

#endif /* __OPLUS_MONITOR_INTERNAL_H__ */
