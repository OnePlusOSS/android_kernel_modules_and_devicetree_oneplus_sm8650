#ifndef __OPLUS_CHG_COMM_H__
#define __OPLUS_CHG_COMM_H__

#include <oplus_mms.h>

#define FFC_CHG_STEP_MAX			5
#define AGAIN_FFC_CYCLY_THR_COUNT		2
#define BATT_TEMP_HYST				20
#define BATT_VOL_HYST				50
#define OPCHG_PWROFF_FORCE_UPDATE_BATT_TEMP	500
#define OPCHG_PWROFF_HIGH_BATT_TEMP		770
#define OPCHG_PWROFF_EMERGENCY_BATT_TEMP	850
#define FACTORY_PWROFF_HIGH_BATT_TEMP		650
#define FACTORY_PWROFF_EMERGENCY_BATT_TEMP	700
#define FFC_VOLT_COUNTS				4
#define FFC_CURRENT_COUNTS			2
#define SOFT_REST_VOL_THRESHOLD			4300
#define SOFT_REST_SOC_THRESHOLD			95
#define SOFT_REST_CHECK_DISCHG_MAX_CUR		200
#define SOFT_REST_RETRY_MAX_CNT			2
#define OPLUS_BMS_HEAT_THRE			250
#define SALE_MODE_COOL_DOWN_VAL		1
#define SALE_MODE_COOL_DOWN		501
#define SALE_MODE_COOL_DOWN_TWO		502

enum oplus_temp_region {
	TEMP_REGION_COLD = 0,
	TEMP_REGION_LITTLE_COLD,
	TEMP_REGION_LITTLE_COLD_HIGH,
	TEMP_REGION_COOL,
	TEMP_REGION_LITTLE_COOL,
	TEMP_REGION_PRE_NORMAL,
	TEMP_REGION_NORMAL,
	TEMP_REGION_NORMAL_HIGH,
	TEMP_REGION_WARM,
	TEMP_REGION_HOT,
	TEMP_REGION_MAX,
};

#define V3P6_TEMP_REGION_MAX 8
#define V3P7_TEMP_REGION_MAX 9

enum power_role_type {
	POWER_ROLE_UNKNOWN = -1,
	POWER_ROLE_SINK = 0,
	POWER_ROLE_SOURCE = 1,
};

enum oplus_ffc_temp_region {
	FFC_TEMP_REGION_COOL,
	FFC_TEMP_REGION_PRE_NORMAL,
	FFC_TEMP_REGION_PRE_NORMAL_HIGH,
	FFC_TEMP_REGION_NORMAL,
	FFC_TEMP_REGION_WARM,
	FFC_TEMP_REGION_MAX,
};

#define V3P6_FFC_TEMP_REGION_MAX 4

enum oplus_fcc_gear {
	FCC_GEAR_LOW = 0,
	FCC_GEAR_LV1,
	FCC_GEAR_LV2,
	FCC_GEAR_LV3,
	FCC_GEAR_HIGH,
	FCC_GEAR_MAX,
};

enum oplus_chg_full_type {
	CHG_FULL_SW,
	CHG_FULL_HW_BY_SW,
};

enum oplus_eis_status_type {
	EIS_STATUS_DISABLE,
	EIS_STATUS_PREPARE,
	EIS_STATUS_HIGH_CURRENT,
	EIS_STATUS_LOW_CURRENT,
};

enum comm_topic_item {
	COMM_ITEM_TEMP_REGION,
	COMM_ITEM_FCC_GEAR,
	COMM_ITEM_CHG_FULL,
	COMM_ITEM_CHG_SUB_BATT_FULL,
	COMM_ITEM_FFC_STATUS,
	COMM_ITEM_CHARGING_DISABLE,
	COMM_ITEM_CHARGE_SUSPEND,
	COMM_ITEM_COOL_DOWN,
	COMM_ITEM_BATT_STATUS,
	COMM_ITEM_BATT_HEALTH,
	COMM_ITEM_BATT_CHG_TYPE,
	COMM_ITEM_UI_SOC,
	COMM_ITEM_NOTIFY_CODE,
	COMM_ITEM_NOTIFY_FLAG,
	COMM_ITEM_SHELL_TEMP,
	COMM_ITEM_FACTORY_TEST,
	COMM_ITEM_LED_ON,
	COMM_ITEM_UNWAKELOCK,
	COMM_ITEM_POWER_SAVE,
	COMM_ITEM_RECHGING,
	COMM_ITEM_FFC_STEP,
	COMM_ITEM_SMOOTH_SOC,
	COMM_ITEM_CHG_CYCLE_STATUS,
	COMM_ITEM_SLOW_CHG,
	COMM_ITEM_SHUTDOWN_SOC,
	COMM_ITEM_VBAT_UV_THR,
	COMM_ITEM_DELTA_SOC,
	COMM_ITEM_CV_CUTOFF_VOLT_CURR,
	COMM_ITEM_FFC_CUTOFF_CURR,
	COMM_ITEM_SUPER_ENDURANCE_STATUS,
	COMM_ITEM_SUPER_ENDURANCE_COUNT,
	COMM_ITEM_UISOC_KEEP_2_ERROR,
	COMM_ITEM_UISOC_KEEP_3_ERROR,
	COMM_ITEM_RECHG_SOC_EN_STATUS,
	COMM_ITEM_SALE_MODE,
	COMM_ITEM_NVID_SUPPORT_FLAGS,
	COMM_ITEM_BOOT_COMPLETED,
	COMM_ITEM_EIS_STATUS,
	COMM_ITEM_FLASH_MODE,
	COMM_ITEM_BATT_CV_FULL,
};

/*
 * +--------+--------+--------+
 * | 31  24 | 23   8 | 7    0 |
 * +--------+--------+--------+
 *     |         |       |
 *     |         |       +------------> enable
 *     |         +--------------------> watt
 *     +------------------------------> pct
 */
#define SLOW_CHG_TO_PARAM(pct, watt, en) ((((pct) & 0xff) << 24) | (((watt) & 0xffff) << 8) | ((en) & 0xff))
#define SLOW_CHG_TO_PCT(param) (((param) >> 24) & 0xff)
#define SLOW_CHG_TO_WATT(param) (((param) >> 8) & 0xffff)
#define SLOW_CHG_TO_ENABLE(param) ((param) & 0xff)

#define RECHG_SOC_TO_PARAM(rechg_soc, en) ((((rechg_soc) & 0xff) << 8) | ((en) & 0xff))
#define RECHG_SOC_TO_SOC(param) (((param) >> 8) & 0xff)
#define RECHG_SOC_TO_ENABLE(param) ((param) & 0xff)

#define CUTOFF_DATA_SHIFT 16
#define CUTOFF_ITERM_SHIFT 8
#define CUTOFF_DATA_MASK 0xffff
#define CUTOFF_ITERM_MASK 0xff

enum oplus_chg_ffc_status {
	FFC_DEFAULT = 0,
	FFC_WAIT,
	FFC_FAST,
	FFC_IDLE,
};

enum aging_ffc_version {
	AGING_FFC_NOT_SUPPORT,
	AGING_FFC_V1,
	AGING_FFC_VERSION_MAX
};

typedef enum {
	CHG_CYCLE_VOTER__NONE		= 0,
	CHG_CYCLE_VOTER__ENGINEER	= (1 << 0),
	CHG_CYCLE_VOTER__USER		= (1 << 1),
}OPLUS_CHG_CYCLE_VOTER;

int oplus_comm_get_vbatt_over_threshold(struct oplus_mms *topic);
int oplus_comm_switch_ffc(struct oplus_mms *topic);
const char *oplus_comm_get_temp_region_str(enum oplus_temp_region temp_region);
int read_signed_data_from_node(struct device_node *node,
			       const char *prop_str,
			       s32 *addr, int len_max);
int read_unsigned_data_from_node(struct device_node *node,
				 const char *prop_str, u32 *addr,
				 int len_max);

int oplus_comm_get_wired_aging_ffc_offset(struct oplus_mms *topic, int step);
int oplus_comm_get_current_wired_ffc_cutoff_fv(struct oplus_mms *topic, int step);
int oplus_comm_get_wired_ffc_cutoff_fv(struct oplus_mms *topic, int step,
	enum oplus_ffc_temp_region temp_region);
int oplus_comm_get_wired_ffc_step_max(struct oplus_mms *topic);
int oplus_comm_get_wired_aging_ffc_version(struct oplus_mms *topic);
int oplus_comm_get_dec_vol(struct oplus_mms *topic, int *fv_dec, int *wired_ffc_dec, int *wls_ffc_dec, int *vct);
bool oplus_comm_get_boot_completed(void);
int oplus_comm_get_bms_heat_temp_compensation(struct oplus_mms *topic);
void oplus_comm_set_bms_heat_temp_compensation(struct oplus_mms *topic, int bms_heat_temp_compensation);
void oplus_comm_set_slow_chg(struct oplus_mms *topic, int pct, int watt, bool en);
void oplus_comm_set_sale_mode(struct oplus_mms *topic, int sale_mode);
int oplus_comm_temp_region_map(int index);
int oplus_comm_get_temp_region_max(void);
int read_unsigned_temp_region_data(struct device_node *node, const char *prop_str, u32 *addr,
					    int col, int col_max, int row, int (*col_map)(int));
int read_signed_temp_region_data(struct device_node *node, const char *prop_str, s32 *addr,
					    int col, int col_max, int row, int (*col_map)(int));
void oplus_comm_set_rechg_soc_limit(struct oplus_mms *topic, int rechg_soc, bool en);
void oplus_comm_get_rechg_soc_limit(struct oplus_mms *topic, int *rechg_soc, bool *en);
int oplus_set_chg_up_limit(struct oplus_mms *topic, int charge_limit_enable, int charge_limit_value,
    int is_force_set_charge_limit, int charge_limit_recharge_value, int callname);
void oplus_comm_set_anti_expansion_status(struct oplus_mms *topic,int val);
int oplus_comm_get_dis_ui_power_state(struct oplus_mms *topic);
int oplus_comm_get_removed_bat_decidegc(struct oplus_mms *topic);
void oplus_comm_set_dec_delta(struct oplus_mms *topic, int val);
void oplus_comm_get_dec_delta(struct oplus_mms *topic, int *val);
bool oplus_comm_get_hmac_not_pop_up(struct oplus_mms *topic);
int oplus_comm_get_bdd_voltdiff_trend(struct oplus_mms *topic);
#endif /* __OPLUS_CHG_COMM_H__ */
