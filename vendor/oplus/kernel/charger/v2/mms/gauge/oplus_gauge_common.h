#ifndef __OPLUS_GAUGE_COMMON_H__
#define __OPLUS_GAUGE_COMMON_H__

#include <oplus_mms.h>
#include "oplus_sili.h"
#include <oplus_sec.h>

#define GAUGE_IC_NUM_MAX 2
#define CALIB_TIME_STR_LEN 32

#define GAUGE_CALIB_TAG_LEN 12
#define GAUGE_CALIB_OBTAIN_COUNTS 3
#define FCL_TABLE_MAX 2
#define FCL_CURVE_MAX 3

struct gauge_calib_info_load {
    char tag_info[GAUGE_CALIB_TAG_LEN];
    struct gauge_calib_info calib_info[GAUGE_IC_NUM_MAX];
}__attribute__((aligned(4)));

struct gauge_three_level_term_volt_cfg {
	unsigned short term_volt;
	unsigned short term_volt_2;
	unsigned short term_volt_3;
	unsigned short recover_term_volt;
	unsigned short recover_term_volt_2;
	unsigned char hold_time;
	unsigned char hold_time_2;
	unsigned char hold_time_3;
	unsigned char time_to_drop_per1;
	unsigned char time_to_drop_per1_2;
	unsigned char time_to_drop_per1_3;
	unsigned char recover_hold_time_of_term_voltage;
	unsigned char recover_hold_time_of_term_voltage_2;
	int term_volt_size;
};

struct fcl_table {
	int volt_diff;
	int curr_dec;
	int min_curr;
} __attribute__ ((packed));

struct fcl_curves {
	struct fcl_table limits[FCL_CURVE_MAX];
	bool support;
	int nums;
	int index;
};
#define GAUGE_NVRAM_TEST_MAX_COUNT     40000
#define GAUGE_TERM_VOLT_TEST_MAX_COUNT 20000
#define GAUGE_NVRAM_TEST_DEFAULT_INTERVAL_MS   2000

struct oplus_mms_gauge {
	struct device *dev;
	struct oplus_chg_ic_dev *gauge_ic;
	struct oplus_chg_ic_dev *level_shift_ic;
	struct oplus_chg_ic_dev *gauge_ic_comb[GAUGE_IC_NUM_MAX];
	struct oplus_chg_ic_dev *voocphy_ic;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *gauge_topic_parallel[GAUGE_IC_NUM_MAX];
	struct oplus_mms *comm_topic;
	struct oplus_mms *wired_topic;
	struct oplus_mms *vooc_topic;
	struct oplus_mms *err_topic;
	struct oplus_mms *parallel_topic;
	struct oplus_mms *batt_bal_topic;
	struct oplus_mms *wls_topic;
	struct oplus_mms *cpa_topic;
	struct mms_subscribe *comm_subs;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *gauge_subs;
	struct mms_subscribe *vooc_subs;
	struct mms_subscribe *parallel_subs;
	struct mms_subscribe *wls_subs;
	struct mms_subscribe *batt_bal_subs;
	struct mms_subscribe *cpa_subs;

	struct delayed_work hal_gauge_init_work;
	struct delayed_work get_reserve_calib_info_work;
	struct work_struct set_reserve_calib_info_work;
	struct work_struct err_handler_work;
	struct work_struct ls_err_handler_work;
	struct work_struct ls_fpga_rst_handler_work;
	struct work_struct online_handler_work;
	struct work_struct offline_handler_work;
	struct work_struct resume_handler_work;
	struct work_struct update_change_work;
	struct work_struct gauge_update_work;
	struct work_struct gauge_set_curve_work;
	struct work_struct set_gauge_batt_full_work;
	struct work_struct update_super_endurance_mode_status_work;
	struct work_struct update_sili_spare_power_enable_work;
	struct work_struct update_sili_ic_alg_cfg_work;
	struct work_struct sub_btb_state_change_handler_work;
	struct work_struct hmac_update_handler_work;

	struct delayed_work sili_spare_power_effect_check_work;
	struct delayed_work sili_term_volt_effect_check_work;
	struct delayed_work subboard_ntc_err_work;
	struct delayed_work deep_dischg_work;
	struct delayed_work sub_deep_dischg_work;
	struct delayed_work deep_id_work;
	struct delayed_work deep_track_work;
	struct delayed_work sub_deep_track_work;
	struct delayed_work deep_ratio_work;
	struct delayed_work deep_temp_work;
	struct delayed_work gauge_cuv_state_work;
	struct delayed_work gauge_update_three_level_term_volt_work;
	struct delayed_work gauge_nvram_stress_test_work;
	struct delayed_work gauge_stress_read_test_work;
	struct delayed_work gauge_term_volt_stress_test_work;

	struct votable *gauge_update_votable;
	struct deep_dischg_spec deep_spec;
	struct oplus_chg_strategy **ddrc_strategy;
	struct ddrc_temp_curves ddrc_curve;
	struct ddrc_temp_curves ddrc_curve_sub;
	int ddrc_num;

	struct oplus_sec *sec_chip;

	int device_type;
	int device_type_for_vooc;
	unsigned int vooc_sid;
	unsigned int err_code;
	int check_batt_vol_count;
	bool pd_svooc;
	bool bat_volt_different;

	bool factory_test_mode;
	bool wired_online;
	bool wls_online;
	bool hmac;
	bool sn_match;
	bool parallel_hamc;
	bool support_subboard_ntc;
	bool check_subboard_ntc_err;
	bool batt_full;
	int batt_temp_region;
	int child_num;
	struct oplus_virtual_gauge_child *child_list;
	int main_gauge;
	int sub_gauge;
	int ui_soc;
	enum oplus_chg_ic_connect_type connect_type;

	struct oplus_chg_mutual_notifier calib_obtain_mutual;
	struct oplus_chg_mutual_notifier calib_update_mutual;
	bool super_endurance_mode_status;
	int super_endurance_mode_count;
	struct votable *gauge_term_voltage_votable;
	struct votable *gauge_shutdown_voltage_votable;
	struct votable *target_shutdown_voltage_votable;
	struct votable *target_term_voltage_votable;
	unsigned char *gauge_reg_info[GAUGE_IC_NUM_MAX];
	unsigned char *gauge_r_info[GAUGE_IC_NUM_MAX];
	unsigned char calib_time_str[GAUGE_IC_NUM_MAX][CALIB_TIME_STR_LEN];
	struct oplus_gauge_lifetime lifetime[GAUGE_IC_NUM_MAX];
	struct gauge_calib_info_load calib_info_load;
	bool calib_info_init[GAUGE_IC_NUM_MAX];
	struct deep_track_info deep_info;
	struct deep_track_info sub_deep_info;
	struct gauge_three_level_term_volt_cfg three_level_term_volt_cfg;
	int sub_btb_curr_limit;
	struct fcl_curves fcl;
	int fcl_offset;
	struct oplus_gauge_nvram_stress_test nvram_test;
};

#endif /* __OPLUS_GAUGE_COMMON_H__ */
