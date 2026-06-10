#ifndef __OPLUS_SILI_H__
#define __OPLUS_SILI_H__

#include <oplus_mms.h>
#include <oplus_chg_mutual.h>



#define GAUGE_IC_NUM_MAX 2
#define CALIB_TIME_STR_LEN 32

#define PUSH_DELAY_MS 2000
#define DUMP_INFO_LEN 128
static char deep_id_info[DUMP_INFO_LEN] = { 0 };

#define INVALID_MAX_VOLTAGE 3800
#define INVALID_MIN_VOLTAGE 2000
#define INVALID_CC_VALUE 5000

#define GAUGE_REG_INFO_SIZE 512

#define GAUGE_TERM_VOLT_EFFECT_GAP_MV(x) x


#define DEEP_INFO_LEN 1023
#define DDB_RANGE_MAX 20
#define DDC_CURVE_MAX 20
#define DDI_CURVE_MAX 20
#define DDT_COEFF_SIZE (sizeof(struct ddt_coeff) / sizeof(u32))
struct oplus_mms_gauge;

enum ddb_temp_region{
	DDB_CURVE_TEMP_COLD,
	DDB_CURVE_TEMP_COOL,
	DDB_CURVE_TEMP_NORMAL,
	DDB_CURVE_TEMP_WARM,
	DDB_CURVE_TEMP_INVALID = DDB_CURVE_TEMP_WARM
};

struct ddb_temp_range {
	int *range;
	int index_n;
	int index_p;
	int temp_type;
};

struct ddb_tcnt {
	int ratio;
	int dischg;
};

static const char *const ddbc_curve_range_name[] = {
	[DDB_CURVE_TEMP_COLD] 	= "deep_spec,ddbc_temp_cold",
	[DDB_CURVE_TEMP_COOL] 	= "deep_spec,ddbc_temp_cool",
	[DDB_CURVE_TEMP_NORMAL] = "deep_spec,ddbc_temp_normal",
	[DDB_CURVE_TEMP_WARM] 	= "deep_spec,ddbc_temp_warm",
};

struct ddb_curve {
	unsigned int iterm;
	unsigned int vterm;
	unsigned int ctime;
};

#define DDB_CURVE_MAX		6
struct ddb_curves {
	struct ddb_curve limits[DDB_CURVE_MAX];
	int nums;
};

struct dds_curve {
	int temp;
	int step;
	int index;
};

struct dds_curves {
	struct dds_curve limits[DDC_CURVE_MAX];
	int nums;
};

struct ddi_curve {
	int ratio;
	int cc;
	int index;
};

struct ddi_curves {
	struct ddi_curve limits[DDI_CURVE_MAX];
	int nums;
};

struct deep_track_info {
	unsigned char msg[DEEP_INFO_LEN];
	int index;
};

struct deep_dischg_sili_ic_alg_cfg {
	unsigned int list[SILI_CFG_TYPE_MAX];
	int nums;
};

struct deep_dischg_limits {
	int32_t uv_thr;
	int32_t target_uv_thr;
	int32_t count_thr;
	int32_t count_cali;
	int32_t soc;
	int32_t term_voltage;
	int32_t target_term_voltage;
	int32_t ratio_shake;
	int32_t sub_ratio_shake;
	int32_t ratio_default;
	int32_t ratio_status;
	int32_t sub_ratio_status;
	int32_t current_fcc_coeff;
	int32_t current_soh_coeff;
	int32_t spare_power_term_voltage;
	int32_t volt_step;
	int32_t index_r;
	int32_t index_t;
	bool step_status;
	uint8_t *ddrc_strategy_name;

	int32_t curr_max_ma;
	int32_t curr_limit_ma;
};

struct ddt_coeff {
	int32_t term_voltage;
	int32_t fcc_coeff;
	int32_t soh_coeff;
} __attribute__ ((packed));

struct deep_dischg_spec {
	bool support;
	int counts;
	int sub_counts;
	int cc;
	int sub_cc;
	int ratio;
	int sub_ratio;
	bool sili_err;
	struct ddb_temp_range ddbc_tbatt;
	struct ddb_temp_range ddbc_tdefault;
	struct ddb_temp_range ddrc_tbatt;
	struct ddb_temp_range ddrc_tdefault;
	struct ddb_curves batt_curves[DDB_RANGE_MAX];
	struct dds_curves step_curves;
	struct ddi_curves limit_curr_curves;
	struct ddt_coeff term_coeff[DDC_CURVE_MAX];
	struct deep_dischg_limits config;
	struct ddb_tcnt cnts;
	struct deep_dischg_sili_ic_alg_cfg sili_ic_alg_cfg;
	struct mutex lock;
	int term_coeff_size;
	int sili_ic_alg_term_volt;
	bool sili_ic_alg_dsg_enable;
	bool sili_ic_alg_support;
	bool spare_power_enable;
	bool spare_power_support;
	bool ddrc_strategy_v2;
	int ddrc_temp_num;
	int ddrc_temp_switch;
};

struct oplus_virtual_gauge_child {
	struct oplus_chg_ic_dev *ic_dev;
	int index;
	int capacity_ratio;
	enum oplus_chg_ic_func *funcs;
	int func_num;
	enum oplus_chg_ic_virq_id *virqs;
	int virq_num;
};

#define SMEM_OPLUS_CHG 127
typedef struct {
    uint32_t size;
    uint8_t support_external_gauge;
    uint8_t support_adsp_voocphy;
    uint8_t support_150w_pps;
    uint8_t btbover_std_version;
    uint32_t abnormal_adapter_break_interval;
    uint8_t support_2s_battery_with_1s_pmic;
    uint8_t support_get_temp_by_subboard_ntc;
    uint8_t support_get_temp_by_subboard_ntc_adc_channel;
    uint8_t support_pmic_detect_bat;
    int8_t battery_type_str[OPLUS_BATTERY_TYPE_LEN];
    uint8_t support_identify_battery_by_adc;
} oplus_ap_feature_data;

int oplus_gauge_parse_deep_spec(struct oplus_mms_gauge *chip);
int oplus_mms_gauge_update_vbat_uv(struct oplus_mms *mms, union mms_msg_data *data);
int oplus_mms_gauge_get_si_prop(struct oplus_mms *mms, union mms_msg_data *data);
int oplus_mms_gauge_update_spare_power_enable(struct oplus_mms *mms, union mms_msg_data *data);
int oplus_mms_gauge_update_sili_ic_alg_dsg_enable(struct oplus_mms *mms, union mms_msg_data *data);
void oplus_mms_gauge_update_sili_ic_alg_term_volt(struct oplus_mms_gauge *chip, bool force);
int oplus_mms_gauge_update_sili_ic_alg_term_volt_data(struct oplus_mms *mms, union mms_msg_data *data);
int oplus_mms_sub_gauge_update_sili_ic_alg_term_volt(struct oplus_mms *mms, union mms_msg_data *data);
int oplus_gauge_shutdown_voltage_vote_callback(struct votable *votable, void *data, int volt, const char *client, bool step);
int oplus_target_shutdown_voltage_vote_callback(struct votable *votable, void *data, int volt, const char *client, bool step);
int oplus_gauge_term_voltage_vote_callback(struct votable *votable, void *data, int volt, const char *client, bool step);
int oplus_target_term_voltage_vote_callback(struct votable *votable, void *data, int volt, const char *client, bool step);
void oplus_mms_gauge_update_super_endurance_mode_status_work(struct work_struct *work);
void oplus_gauge_deep_dischg_work(struct work_struct *work);
void oplus_gauge_sub_deep_dischg_work(struct work_struct *work);
void oplus_gauge_deep_id_work(struct work_struct *work);
void oplus_gauge_deep_track_work(struct work_struct *work);
void oplus_gauge_sub_deep_track_work(struct work_struct *work);
void oplus_gauge_deep_ratio_work(struct work_struct *work);
void oplus_gauge_deep_temp_work(struct work_struct *work);
void oplus_mms_gauge_update_sili_ic_alg_cfg_work(struct work_struct *work);
void oplus_mms_gauge_update_sili_spare_power_enable_work(struct work_struct *work);
void oplus_mms_gauge_sili_spare_power_effect_check_work(struct work_struct *work);
void oplus_mms_gauge_sili_term_volt_effect_check_work(struct work_struct *work);
void oplus_mms_gauge_set_deep_term_volt_work(struct work_struct *work);
void oplus_gauge_nvram_stress_test_work(struct work_struct *work);
void oplus_gauge_term_volt_stress_test_work(struct work_struct *work);
void oplus_gauge_read_stress_test_work(struct work_struct *work);

void oplus_mms_gauge_sili_init(struct oplus_mms_gauge *chip);
int oplus_mms_gauge_sili_ic_alg_cfg_init(struct oplus_mms_gauge *chip);
void oplus_mms_gauge_deep_dischg_init(struct oplus_mms_gauge *chip);
void oplus_gauge_deep_dischg_check(struct oplus_mms_gauge *chip);
int oplus_mms_gauge_update_deep_ratio(struct oplus_mms *mms, union mms_msg_data *data);
int oplus_mms_gauge_update_ratio_trange(struct oplus_mms *mms, union mms_msg_data *data);
int oplus_mms_gauge_update_ratio_limit_curr(struct oplus_mms *mms, union mms_msg_data *data);
int oplus_mms_gauge_set_deep_term_volt(struct oplus_mms *mms, int volt_mv);
#endif /* __OPLUS_SILI_H__ */
