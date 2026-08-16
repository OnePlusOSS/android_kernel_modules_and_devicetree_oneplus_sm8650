// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2026 Oplus. All rights reserved.
 */
#ifndef __OPLUS_NFG8011B_H__
#define __OPLUS_NFG8011B_H__

#include <oplus_chg.h>

#define DEVICE_TYPE_NFG8011B		0x4231

#define NFG8011B_SUBCMD_DA_STATUS1	0X0071
#define NFG8011B_SUBCMD_DA_STATUS2	0X0072
#define NFG8011B_SUBCMD_IT_STATUS1	0X0073
#define NFG8011B_DATAFLASHBLOCK		0x3E
#define NFG8011B_FORCE_TO_0		0xC3
#define NFG8011B_FW_VERSION		0x02
#define NFG8011B_REG_CIS_ALERT_LEVEL	0x72
#define NFG8011B_SUBCMD_CHEMID		0X0006
#define NFG8011B_SUBCMD_GAUGE_STATUS	0X0056
#define NFG8011B_SUBCMD_DEEP_INFO_ADDR 0x007B
#define NFG8011B_SUBCMD_DEEP_TERM_VOLT_ADDR 0x0050
#define NFG8011B_SUBCMD_SI_ALG_CTRL_ADDR 0x4410
#define NFG8011B_SUBCMD_FASTOCV_ADDR 0x0076
#define NFG8011B_SUBCMD_ALG_ADDR_0 0x00D0
#define NFG8011B_SUBCMD_LIFETIME_1_ADDR 0x0060
#define NFG8011B_SUBCMD_LIFETIME_2_ADDR 0x0062
#define NFG8011B_SUBCMD_ISC_INFO_ADDR	0x0093
#define NFG8011B_SUBCMD_IPM_ADDR	    0x007C
#define NFG8011B_SUBCMD_ALG_ADDR_1 0x007D
#define NFG8011B_SUBCMD_ISC_LAST_TEN_ADDR 0x0090
#define NFG8011B_SUBCMD_SOH2_ADDR	0x009D

#define NFG8011B_SUBCMD_MANU_INFO_ADDR		0x0070
#define NFG8011B_SUBCMD_MANU_DATE_ADDR		0x004D
#define NFG8011B_ECO_FIRST_USAGE_DATE_OFFSET	20
#define NFG8011B_ECO_UI_SOH_OFFSET		23
#define NFG8011B_ECO_UI_CC_OFFSET		25
#define NFG8011B_ECO_USED_FLAG_OFFSET		28

#define NFG8011B_SUBCMD_TRY_COUNT	3

#define NFG8011B_AUTHENDATA_1ST		0x40
#define NFG8011B_AUTHENDATA_2ND		0x50
#define NFG8011B_AUTHENCHECKSUM		0x60
#define NFG8011B_AUTHENLEN		0x61
#define NFG8011B_OPERATION_STATUS	0x0054
#define NFG8011B_I2C_TRY_COUNT		7

#define NFG8011B_DEEP_DISCHG_CHECK 0xFF
#define NFG8011B_DEEP_SHIFT 0x08

#define NFG8011B_UNSEAL_SUBCMD1 0x7236
#define NFG8011B_UNSEAL_SUBCMD2 0x1404
#define NFG8011B_SEAL_STATUS 0x0054
#define NFG8011B_SEAL_BIT (BIT(0) | BIT(1))
#define NFG8011B_SEAL_VALUE 3
#define NFG8011B_SEAL_SUBCMD 0x0030
#define NFG8011B_SEAL_POLLING_RETRY_LIMIT 10
#define NFG8011B_SILI_ALG_CTRL_MASK 0x3F
#define NFG8011B_SI_ALG_DSG_ENABLE_MASK 0x0C
#define NFG8011B_SILI_OCV_HYSTERESIS_MASK 		BIT(0)
#define NFG8011B_SILI_OCV_AGING_OFFSET_MASK		BIT(1)
#define NFG8011B_SILI_DYNAMIC_DSG_CTRL_MASK		BIT(2)
#define NFG8011B_SILI_STATIC_DSG_CTRL_MASK		BIT(3)
#define NFG8011B_SILI_MONITOR_MODE_MASK			BIT(5)

#define NFG8011B_CHEMISTRY_ID_EN_ADDR		0x3E
#define NFG8011B_CHEMISTRY_ID_CMD		0x06
#define NFG8011B_CHEMISTRY_ID_ADDR		0x40
#define NFG8011B_CHEMISTRY_ID_SIZE		8

#define NFG801B_REG_RSOC_CENTI			0x1E

#define NFG8011B_REG_ENABLE_FAST_SAMP		0x005d
#define NFG8011B_REG_DISABLE_FAST_SAMP		0x005e
#define NFG8011B_REG_ENABLE_STATE		0x005f
#define NFG8011B_ENABLE_OK			0xa5
#define NFG8011B_REG_REAL_CURR			0x1c
#define NFG8011B_REG_REAL_VOLT			0x1a

#ifdef CONFIG_OPLUS_GAUGE_NFG8011B
bool nfg8011b_sha256_hmac_authenticate(struct chip_bq27541 *chip);
int nfg8011b_get_car_c_parameters(struct chip_bq27541 *chip, int *car_c_ptr);
int nfg8011b_get_qmax_parameters(struct chip_bq27541 *chip, int *cell_qmax);
int nfg8011b_get_dod0_parameters(struct chip_bq27541 *chip, int *cell_dod);
int nfg8011b_get_calib_time(struct chip_bq27541 *chip, int *dod_time, int *qmax_time);
int nfg8011b_get_info(struct chip_bq27541 *chip, u8 *info, int len);
int nfg8011b_set_spare_power_enable(struct chip_bq27541 *chip);
int nfg8011b_get_sili_ic_alg_term_volt(struct chip_bq27541 *chip, int *volt);
int nfg8011b_get_sili_simulate_term_volt(struct chip_bq27541 *chip, int *volt);
int nfg8011b_get_deep_dischg_counts(struct chip_bq27541 *chip, int *count);
int nfg8011b_set_deep_dischg_counts(struct chip_bq27541 *chip, int count);
int nfg8011b_set_term_volt(struct chip_bq27541 *chip, int volt_mv);
int nfg8011b_get_define_term_volt(struct chip_bq27541 *chip, int *volt);
int nfg8011b_set_sili_ic_alg_cfg(struct chip_bq27541 *chip, int config);
int nfg8011b_set_sili_ic_alg_term_volt(struct chip_bq27541 *chip, int sys_term_vol);
int nfg8011b_get_sili_lifetime_status(struct chip_bq27541 *chip, struct oplus_gauge_lifetime *lifetime_status);
int nfg8011b_get_sili_lifetime_info(struct chip_bq27541 *chip, u8 *buf, int len);
int nfg8011b_set_battery_full(struct chip_bq27541 *chip, bool full);
int nfg8011b_set_soc_froce_to_0_threshold(struct chip_bq27541 *chip, u16 voltage_mv, u16 time_sec);
int nfg8011b_get_sili_alg_application_info(struct chip_bq27541 *chip, u8 *buf, int len);
int nfg8011b_get_sili_chemistry_info(struct chip_bq27541 *chip, u8 *info, int len);
int nfg8011b_get_last_cc(struct chip_bq27541 *chip, int *cc);
int nfg8011b_set_last_cc(struct chip_bq27541 *chip, int cc);
int nfg8011b_read_block(struct chip_bq27541 *chip, int addr, u8 *buf, int len, int offset, bool do_checksum);
int nfg8011b_write_block(struct chip_bq27541 *chip,
	int addr, u8 *buf, int len, int offset, bool do_checksum, bool read_back);
void nfg8011b_effect_term_volt_init(struct chip_bq27541 *chip);
int nfg8011b_soc_centi_init(struct chip_bq27541 *chip);
int nfg8011b_set_fast_sampling(struct chip_bq27541 *chip, bool enable);
#else
bool nfg8011b_sha256_hmac_authenticate(struct chip_bq27541 *chip)
{
	return false;
}

int nfg8011b_get_car_c_parameters(struct chip_bq27541 *chip, int *car_c_ptr)
{
	return 0;
}

int nfg8011b_get_qmax_parameters(struct chip_bq27541 *chip, int *cell_qmax)
{
	return 0;
}

int nfg8011b_get_dod0_parameters(struct chip_bq27541 *chip, int *cell_dod)
{
	return 0;
}

int nfg8011b_get_calib_time(struct chip_bq27541 *chip, int *dod_time, int *qmax_time)
{
	return 0;
}

int nfg8011b_get_info(struct chip_bq27541 *chip, u8 *info, int len)
{
	return 0;
}

int nfg8011b_set_spare_power_enable(struct chip_bq27541 *chip)
{
	return 0;
}

int nfg8011b_get_sili_ic_alg_term_volt(struct chip_bq27541 *chip, int *volt)
{
	return 0;
}

int nfg8011b_get_sili_simulate_term_volt(struct chip_bq27541 *chip, int *volt)
{
	return 0;
}

int nfg8011b_get_deep_dischg_counts(struct chip_bq27541 *chip, int *count)
{
	return 0;
}

int nfg8011b_set_deep_dischg_counts(struct chip_bq27541 *chip, int count)
{
	return 0;
}

int nfg8011b_set_term_volt(struct chip_bq27541 *chip, int volt_mv)
{
	return 0;
}

int nfg8011b_get_define_term_volt(struct chip_bq27541 *chip, int *volt)
{
	return 0;
}

int nfg8011b_set_sili_ic_alg_cfg(struct chip_bq27541 *chip, int config)
{
	return 0;
}

int nfg8011b_set_sili_ic_alg_term_volt(struct chip_bq27541 *chip, int sys_term_vol)
{
	return 0;
}

int nfg8011b_get_sili_lifetime_status(struct chip_bq27541 *chip, struct oplus_gauge_lifetime *lifetime_status)
{
	return 0;
}

int nfg8011b_get_sili_lifetime_info(struct chip_bq27541 *chip, u8 *buf, int len)
{
	return 0;
}

int nfg8011b_set_battery_full(struct chip_bq27541 *chip, bool full)
{
	return 0;
}

int nfg8011b_set_soc_froce_to_0_threshold(struct chip_bq27541 *chip, u16 voltage_mv, u16 time_sec)
{
	return 0;
}

int nfg8011b_get_sili_alg_application_info(struct chip_bq27541 *chip, u8 *buf, int len)
{
	return 0;
}

int nfg8011b_get_sili_chemistry_info(struct chip_bq27541 *chip, u8 *info, int len)
{
	return 0;
}

int nfg8011b_get_last_cc(struct chip_bq27541 *chip, int *cc)
{
	return 0;
}

int nfg8011b_set_last_cc(struct chip_bq27541 *chip, int cc)
{
	return 0;
}

int nfg8011b_read_block(struct chip_bq27541 *chip, int addr, u8 *buf, int len, int offset, bool do_checksum)
{
	return 0;
}

int nfg8011b_write_block(struct chip_bq27541 *chip,
	int addr, u8 *buf, int len, int offset, bool do_checksum, bool read_back)
{
	return 0;
}

void nfg8011b_effect_term_volt_init(struct chip_bq27541 *chip)
{
}

int nfg8011b_soc_centi_init(struct chip_bq27541 *chip)
{
	return 0;
}

int nfg8011b_set_fast_sampling(struct chip_bq27541 *chip, bool enable)
{
	return 0;
}
#endif

#endif  /* __OPLUS_NFG8011B_H__ */
