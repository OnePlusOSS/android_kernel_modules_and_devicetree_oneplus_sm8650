// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2026 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "[NFG8011B]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <oplus_mms_gauge.h>
#include "oplus_hal_bq27541.h"
#include "oplus_hal_nfg8011b.h"

static struct gauge_sili_ic_alg_cfg_map cfg_mapping_table[] = {
	{ BIT(SILI_OCV_HYSTERESIS), NFG8011B_SILI_OCV_HYSTERESIS_MASK },
	{ BIT(SILI_OCV_AGING_OFFSET), NFG8011B_SILI_OCV_AGING_OFFSET_MASK },
	{ BIT(SILI_DYNAMIC_DSG_CTRL), NFG8011B_SILI_DYNAMIC_DSG_CTRL_MASK },
	{ BIT(SILI_STATIC_DSG_CTRL), NFG8011B_SILI_STATIC_DSG_CTRL_MASK },
	{ BIT(SILI_MONITOR_MODE), NFG8011B_SILI_MONITOR_MODE_MASK},
};

static bool nfg8011b_firmware_support_fast_sampling(struct chip_bq27541 *chip);

bool nfg8011b_sha256_hmac_authenticate(struct chip_bq27541 *chip)
{
	int i;
	int ret;
	int count;
	u8 checksum = 0;
	u8 data_buf[2] = {0};
	u8 *p_temp = chip->sha256_authenticate_data->random;
	int len = ARRAY_SIZE(chip->sha256_authenticate_data->random);
	int half_len = len / 2;

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	for (i = 0; i < len; i++)
		checksum = checksum + chip->sha256_authenticate_data->random[i];
	checksum = 0xff - (checksum & 0xff);

	ret = bq27541_write_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, sizeof(data_buf), data_buf);
	if (ret < 0)
		goto error;

	ret = bq27541_write_i2c_block(chip, NFG8011B_AUTHENDATA_1ST, half_len, p_temp);
	if (ret < 0)
		goto error;

	ret = bq27541_write_i2c_block(chip, NFG8011B_AUTHENDATA_2ND, half_len, p_temp + half_len);
	if (ret < 0)
		goto error;

	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;

	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x24);
	if (ret < 0)
		goto error;

	p_temp = chip->sha256_authenticate_data->gauge_encode;

	for (count = 0; count< NFG8011B_I2C_TRY_COUNT; count++) {
		msleep(20);
		ret = bq27541_read_i2c_block(chip, NFG8011B_AUTHENDATA_1ST, half_len, p_temp);
		if (ret < 0) {
			chg_info("ret=%d, count=%d\n", ret, count);
			msleep(20);
		} else {
			break;
		}
	}

	ret = bq27541_read_i2c_block(chip, NFG8011B_AUTHENDATA_2ND, half_len, p_temp + half_len);
	if (ret < 0)
		goto error;

	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return true;

error:
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return false;
}

int nfg8011b_get_car_c_parameters(struct chip_bq27541 *chip, int *car_c_ptr)
{
	int ret;
	int data_check;
	int value;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[14] = {0};
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_ALG_ADDR_1 , 12 };

	if (!chip || !car_c_ptr)
		return -EINVAL;

	for (; try_count > 0; try_count--) {
		mutex_lock(&chip->bq28z610_alt_manufacturer_access);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
		if (ret < 0)
			goto error;
		data_check = (extend_data[1] << 0x8) | extend_data[0];
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		if (data_check != extend.addr) {
			chg_info("not match. add=0x%4x, count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
				extend.addr, try_count, extend_data[0], extend_data[1]);
			usleep_range(2000, 2000);
		} else {
			break;
		}
	}
	if (!try_count)
		return -EINVAL;

	/* data10 and data11 represent car values */
	value = (extend_data[11] << 0x08) | extend_data[10];
	if (value & 0x8000)
		value = -((~(value - 1)) & 0xFFFF);
	*car_c_ptr = value;
	return 0;

error:
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return ret;
}

int nfg8011b_get_qmax_parameters(struct chip_bq27541 *chip, int *cell_qmax)
{
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[12] = {0};
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DA_STATUS1 , 10 };

	if (!chip || !cell_qmax)
		return -EINVAL;

	for (; try_count > 0; try_count--) {
		mutex_lock(&chip->bq28z610_alt_manufacturer_access);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
		if (ret < 0)
			goto error;
		data_check = (extend_data[1] << 0x8) | extend_data[0];
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		if (data_check != extend.addr) {
			chg_info("not match. add=0x%4x, count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
				extend.addr, try_count, extend_data[0], extend_data[1]);
			usleep_range(2000, 2000);
		} else {
			break;
		}
	}
	if (!try_count)
		return -EINVAL;

	*cell_qmax = (extend_data[11] << 0x08) | extend_data[10];
	chg_debug("cell_qmax:%d\n", *cell_qmax);
	return 0;

error:
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return ret;
}

int nfg8011b_get_dod0_parameters(struct chip_bq27541 *chip, int *dod)
{
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[6] = {0};
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DA_STATUS1 , 4 };

	if (!chip || !dod)
		return -EINVAL;

	for (; try_count > 0; try_count--) {
		mutex_lock(&chip->bq28z610_alt_manufacturer_access);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
		if (ret < 0)
			goto error;
		data_check = (extend_data[1] << 0x8) | extend_data[0];
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		if (data_check != extend.addr) {
			chg_info("not match. add=0x%4x, count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
			extend.addr, try_count, extend_data[0], extend_data[1]);
			usleep_range(2000, 2000);
		} else {
			break;
		}
	}

	if (!try_count)
		return -EINVAL;

	*dod = (extend_data[5] << 0x08) | extend_data[4];
	chg_debug("cell_dod0:%d\n", *dod);
	return 0;

error:
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return ret;
}

int nfg8011b_get_calib_time(struct chip_bq27541 *chip, int *dod_time, int *qmax_time)
{
	int ret;
	int check_args[CALIB_TIME_CHECK_ARGS] = {0};
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	int data_check;
	u8 extend_data[22] = {0};
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DA_STATUS1 , 20};

	if (!chip || !dod_time || !qmax_time)
		return -EINVAL;

	for (; try_count > 0; try_count--) {
		mutex_lock(&chip->bq28z610_alt_manufacturer_access);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
		if (ret < 0)
			goto error;
		data_check = (extend_data[1] << 0x8) | extend_data[0];
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		if (data_check != extend.addr) {
			chg_info("not match. add=0x%4x, count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
			extend.addr, try_count, extend_data[0], extend_data[1]);
			usleep_range(2000, 2000);
		} else {
			break;
		}
	}
	if (!try_count)
		return -EINVAL;

	check_args[0] = (extend_data[21] << 0x08) | extend_data[20];
	check_args[1] = (extend_data[13] << 0x08) | extend_data[12];
	check_args[2] = (extend_data[7] << 0x08) | extend_data[6];
	check_args[3] = (extend_data[15] << 0x08) | extend_data[14];
	check_args[4] = (extend_data[9] << 0x08) | extend_data[8];
	if (check_args[0] != chip->calib_check_args_pre[0])
		chip->dod_time = 1;
	else
		chip->dod_time++;

	if (check_args[2] != chip->calib_check_args_pre[2] ||
		check_args[4] != chip->calib_check_args_pre[4] ||
		check_args[1] != chip->calib_check_args_pre[1] || check_args[3] != chip->calib_check_args_pre[3])
		chip->qmax_time = 1;
	else
		chip->qmax_time++;

	memcpy(chip->calib_check_args_pre, check_args, sizeof(check_args));
	*dod_time = chip->dod_time;
	*qmax_time = chip->qmax_time;
	return ret;

error:
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

int nfg8011b_get_info(struct chip_bq27541 *chip, u8 *info, int len)
{
	int i;
	int j;
	int ret;
	int size;
	int data = 0;
	int index = 0;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[34] = {0};
	struct gauge_track_info_reg *extend;
	struct gauge_track_info_reg standard[] = {
		{ chip->cmd_addr.reg_temp, 2 },
		{ chip->cmd_addr.reg_volt , 2 },
		{ chip->cmd_addr.reg_flags  , 2 },
		{ chip->cmd_addr.reg_nac , 2 },
		{ chip->cmd_addr.reg_rm , 2 },
		{ chip->cmd_addr.reg_fcc , 2 },
		{ chip->cmd_addr.reg_cc , 2 },
		{ chip->cmd_addr.reg_soc , 2 },
		{ chip->cmd_addr.reg_soh , 2 },
	};
	struct gauge_track_info_reg nfg8011b_extend[] = {
		{ NFG8011B_SUBCMD_CHEMID, 2, 0, 1 },
		{ NFG8011B_SUBCMD_GAUGE_STATUS  , 3, 0, 2 },
		{ NFG8011B_SUBCMD_DA_STATUS1 , 32, 0, 31 },
		{ NFG8011B_SUBCMD_IT_STATUS1 , 18, 0, 17 },
		{ NFG8011B_SUBCMD_DEEP_INFO_ADDR , 6, 0, 5 },
		{ NFG8011B_SUBCMD_FASTOCV_ADDR, 2, 0, 1 },
		{ NFG8011B_SUBCMD_ALG_ADDR_0, 10, 0, 9 },
	};

	extend = nfg8011b_extend;
	size = ARRAY_SIZE(nfg8011b_extend);

	for (i = 0; i < ARRAY_SIZE(standard); i++) {
		if (chip->fpga_support && is_return_pre_value(chip))
			return index;
		ret = bq27541_read_i2c(chip, standard[i].addr, &data);
		if (ret < 0)
			continue;
		index += snprintf(info + index, len - index,
			  "0x%02x=%02x,%02x|", standard[i].addr, (data & 0xff), ((data >> 8) & 0xff));
	}

	for (i = 0; i < size; i++) {
		if (chip->fpga_support && is_return_pre_value(chip))
			return index;
		try_count = NFG8011B_SUBCMD_TRY_COUNT;
try:
		mutex_lock(&chip->bq28z610_alt_manufacturer_access);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend[i].addr);
		if (ret < 0) {
			mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
			continue;
		}

		if (sizeof(extend_data) >= extend[i].len + 2) {
			usleep_range(1000, 1000);
			ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend[i].len + 2), extend_data);
			mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
			if (ret < 0)
				continue;
			data_check = (extend_data[1] << 0x8) | extend_data[0];
			if (try_count-- > 0 && data_check != extend[i].addr) {
				chg_info("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
					extend[i].addr, try_count, extend_data[0], extend_data[1]);
				usleep_range(2000, 2000);
				goto try;
			}
			if (try_count < 0)
				continue;
			index += snprintf(info + index, len - index, "0x%04x=", extend[i].addr);
			for (j = extend[i].start_index; j < extend[i].end_index; j++)
				index += snprintf(info + index, len - index, "%02x,", extend_data[j + 2]);
			index += snprintf(info + index, len - index, "%02x", extend_data[j + 2]);
			if (i < size - 1)
				usleep_range(2000, 2000);
		} else {
			mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		}
		if (i  <  size - 1)
			index += snprintf(info + index, len - index, "|");
	}

	return index;
}

int nfg8011b_set_spare_power_enable(struct chip_bq27541 *chip)
{
	int i;
	int ret;
	u8 extend_data[4] = {0xC0, 0x00, 0x01, 0x00};
	u8 checksum = 0;

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_write_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, ARRAY_SIZE(extend_data), extend_data);
	if (ret < 0)
		goto error;

	for (i = 0; i < ARRAY_SIZE(extend_data); i++)
		checksum = checksum + extend_data[i];
	checksum = 0xff - (checksum & 0xff);
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x06);
	if (ret < 0)
		goto error;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	chg_info("success\n");
	return 0;

error:
	chg_err("fail\n");
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}


int nfg8011b_get_sili_ic_alg_term_volt(struct chip_bq27541 *chip, int *volt)
{
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[4] = {0};
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_ALG_ADDR_0 , 2 };

	if (!chip || !volt)
		return -EINVAL;

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	*volt = (extend_data[3] << 0x08) | extend_data[2];
	chg_info("sys_term_volt:%d\n", *volt);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return 0;

error:
	*volt = 0;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

int nfg8011b_get_sili_simulate_term_volt(struct chip_bq27541 *chip, int *volt)
{
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[16] = {0};
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_IT_STATUS1 , 14 };

	if (!chip || !volt)
		return -EINVAL;

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	*volt = (extend_data[15] << 0x08) | extend_data[14];
	chg_info("simulate_term_volt:%d\n", *volt);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return 0;

error:
	*volt = 0;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

int nfg8011b_get_deep_dischg_counts(struct chip_bq27541 *chip, int *count)
{
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[14] = {0};
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DEEP_INFO_ADDR , 12 };

	if (!chip || !count)
		return -EINVAL;

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return chip->deep_dischg_count_pre;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	if (extend_data[4] != (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[2] + extend_data[3]) & NFG8011B_DEEP_DISCHG_CHECK)) {
		chg_err("check sum not match. extend_data[0x%2x, 0x%2x, 0x%2x]\n", extend_data[2], extend_data[3], extend_data[4]);
		goto error;
	}

	*count = (extend_data[3] << 0x08) | extend_data[2];
	chg_info("count:%d\n", *count);
	chip->deep_dischg_count_pre = *count;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return 0;

error:
	*count = 0;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

int nfg8011b_get_define_term_volt(struct chip_bq27541 *chip, int *volt)
{
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[14] = {0};
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DEEP_INFO_ADDR , 12 };

	if (!chip || !volt)
		return -EINVAL;

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return chip->deep_term_volt_pre;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	if (extend_data[7] != (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[5] + extend_data[6]) & NFG8011B_DEEP_DISCHG_CHECK)) {
		chg_err("check sum not match. extend_data[0x%2x, 0x%2x, 0x%2x]\n", extend_data[5], extend_data[6], extend_data[7]);
		goto error;
	}

	*volt = (extend_data[6] << 0x08) | extend_data[5];
	chg_info("get term_volt:%d\n", *volt);
	chip->deep_term_volt_pre = *volt;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return 0;

error:
	*volt = 0;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

int nfg8011b_set_deep_dischg_counts(struct chip_bq27541 *chip, int count)
{
	int i;
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[36] = {0};
	u8 checksum = 0;
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DEEP_INFO_ADDR , 32 };

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;
	extend_data[2] = count & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[3] = (count  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[4] = (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[2] + extend_data[3]) & NFG8011B_DEEP_DISCHG_CHECK);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	ret = bq27541_write_i2c_block(chip, NFG8011B_AUTHENDATA_1ST, extend.len, extend_data + 2);
	if (ret < 0)
		goto error;

	extend_data[0] = extend.addr & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[1] =  (extend.addr  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;
	for (i = 0; i < extend.len + 2; i++)
		checksum = checksum + extend_data[i];
	checksum = 0xff - (checksum & 0xff);
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x24);
	if (ret < 0)
		goto error;

	try_count = NFG8011B_SUBCMD_TRY_COUNT;
	do {
		data_check = true;
		usleep_range(15000, 15000);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, 5, extend_data);
		if (extend_data[4] != (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[2] + extend_data[3]) & NFG8011B_DEEP_DISCHG_CHECK) ||
			count != ((extend_data[3] << 0x08) | extend_data[2])) {
			chg_err("count not match. extend_data[0x%2x, 0x%2x, 0x%2x]\n", extend_data[2], extend_data[3], extend_data[4]);
			data_check = false;
		}
	} while (!data_check && try_count--);
	if (!data_check)
		goto error;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	chg_info("set deep count %d success\n", count);
	return 0;

error:
	chg_info("set deep count fail\n");
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

static int nfg8011b_set_define_term_volt(struct chip_bq27541 *chip, int volt_mv)
{
	int i;
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[36] = {0};
	u8 checksum = 0;
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DEEP_INFO_ADDR , 32 };

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;
	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	extend_data[5] = volt_mv & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[6] = (volt_mv  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[7] = (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[5] + extend_data[6]) & NFG8011B_DEEP_DISCHG_CHECK);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	ret = bq27541_write_i2c_block(chip, NFG8011B_AUTHENDATA_1ST, extend.len, extend_data + 2);
	if (ret < 0)
		goto error;
	extend_data[0] = extend.addr & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[1] =  (extend.addr  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;
	for (i = 0; i < extend.len + 2; i++)
		checksum = checksum + extend_data[i];
	checksum = 0xff - (checksum & 0xff);
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x24);
	if (ret < 0)
		goto error;

	memset(extend_data, 0, sizeof(extend_data));
	try_count = NFG8011B_SUBCMD_TRY_COUNT;
	do {
		usleep_range(15000, 15000);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		data_check = true;
		ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
		if (extend_data[7] != (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[5] + extend_data[6]) & NFG8011B_DEEP_DISCHG_CHECK) ||
			volt_mv != ((extend_data[6] << 0x08) | extend_data[5])) {
			chg_err("volt not match. extend_data[0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x]\n",
				extend_data[0], extend_data[1], extend_data[5], extend_data[6], extend_data[7]);
			data_check = false;
		}
	} while (!data_check && try_count--);
	if (!data_check)
		goto error;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	chg_info("set define term volt %d success\n", volt_mv);
	return 0;

error:
	chg_err("set define term volt fail\n");
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

static int nfg8011b_set_effect_term_volt(struct chip_bq27541 *chip, int volt_mv)
{
	int i;
	int ret;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[4] = {0};
	u8 checksum = 0;
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DEEP_TERM_VOLT_ADDR , 2 };

	if (!chip)
		return -EINVAL;

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

	extend_data[0] = extend.addr & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[1] =  (extend.addr  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[2] = volt_mv & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[3] = (volt_mv  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	ret = bq27541_write_i2c_block(chip, NFG8011B_AUTHENDATA_1ST, extend.len, extend_data+2);
	if (ret < 0)
		goto error;
	for (i = 0; i < extend.len + 2; i++)
		checksum = checksum + extend_data[i];
	checksum = 0xff - (checksum & 0xff);
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x6);
	if (ret < 0)
		goto error;

try:
	usleep_range(5000, 5000);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, 4, extend_data);
	if (ret < 0)
		goto error;
	if (((extend_data[3] << 0x8) + extend_data[2]) == volt_mv) {
		chg_info("deep term volt %d set success\n", volt_mv);
	} else {
		if (try_count--) {
			chg_info("%s, count=%d\n", __func__, try_count);
			goto try;
		} else {
			chg_err("deep term volt set fail\n");
		}
	}
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return 0;

error:
	chg_err("deep term volt set error\n");
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

void nfg8011b_effect_term_volt_init(struct chip_bq27541 *chip)
{
	int rc;
	int deep_term_volt;

	rc = nfg8011b_get_define_term_volt(chip, &deep_term_volt);
	if (!rc)
		nfg8011b_set_effect_term_volt(chip, deep_term_volt);
}

int nfg8011b_set_term_volt(struct chip_bq27541 *chip, int volt_mv)
{
	int ret;

	if (!chip)
		return -EINVAL;

	ret = nfg8011b_set_define_term_volt(chip, volt_mv);
	ret |= nfg8011b_set_effect_term_volt(chip, volt_mv);

	return ret;
}

static int nfg8011b_i2c_deep_int(struct chip_bq27541 *chip)
{
	int rc = 0;

	if (!chip->client)
		return 0;

	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	rc = i2c_smbus_write_word_data(chip->client, NFG8011B_DATAFLASHBLOCK, NFG8011B_UNSEAL_SUBCMD1);
	usleep_range(5000, 5000);
	rc = i2c_smbus_write_word_data(chip->client, NFG8011B_DATAFLASHBLOCK, NFG8011B_UNSEAL_SUBCMD2);
	if (rc < 0)
		chg_err("write err, rc = %d\n", rc);
	mutex_unlock(&chip->chip_mutex);
	return 0;
}

static int nfg8011b_sealed(struct chip_bq27541 *chip)
{
	int value = 0;
	u8 data[4] = {0};

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, NFG8011B_SEAL_STATUS);
	usleep_range(2000, 2000);
	bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, 4, data);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	chg_info("data:0x%x, 0x%x, 0x%x, 0x%x\n", data[0], data[1], data[2], data[3]);
	value = (data[3] & NFG8011B_SEAL_BIT);
	if (value == NFG8011B_SEAL_VALUE) {
		chg_info("sealed, value = %x return 1\n", value);
		return 1;
	} else {
		chg_info("unsealed, value = %x return 0\n", value);
		return 0;
	}
}

static int nfg8011b_deep_init(struct chip_bq27541 *chip)
{
	if (!nfg8011b_sealed(chip)) {
		chg_err("already unsealed\n");
		return 1;
	}
	nfg8011b_i2c_deep_int(chip);

	usleep_range(50000, 50000);
	if (!nfg8011b_sealed(chip)) {
		return 1;
	} else {
		chg_err("unseal failed\n");
		return 0;
	}
}

static void nfg8011b_deep_deinit(struct chip_bq27541 *chip)
{
	int i = 0;

	if (!nfg8011b_sealed(chip)) {
		usleep_range(1000, 1000);
		bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, NFG8011B_SEAL_SUBCMD);
		usleep_range(50000, 50000);
		for (i = 0; i < NFG8011B_SEAL_POLLING_RETRY_LIMIT; i++) {
			if (nfg8011b_sealed(chip)) {
				chg_info("sealed,used %d x10ms\n", i);
				return;
			}
			usleep_range(10000, 10000);
		}
	}
}

static int nfg8011b_mapping_alg_cfg(int src_config)
{
	int i;
	int des_config = 0;

	for (i = 0; i < ARRAY_SIZE(cfg_mapping_table); i++) {
		if (cfg_mapping_table[i].map_src & src_config)
			des_config |= cfg_mapping_table[i].map_des;
	}
	des_config &= NFG8011B_SILI_ALG_CTRL_MASK;

	return des_config;
}

int nfg8011b_set_sili_ic_alg_cfg(struct chip_bq27541 *chip, int config)
{
	int i;
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[36] = {0};
	u8 checksum = 0;
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_SI_ALG_CTRL_ADDR , 32 };

	if (!chip)
		return -EINVAL;

	if (!nfg8011b_deep_init(chip))
		return -EINVAL;

	config = nfg8011b_mapping_alg_cfg(config);
try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	extend_data[21] &= ~NFG8011B_SILI_ALG_CTRL_MASK;
	extend_data[21] |= config;
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	ret = bq27541_write_i2c_block(chip, NFG8011B_AUTHENDATA_1ST, extend.len, extend_data+2);
	if (ret < 0)
		goto error;
	extend_data[0] = extend.addr & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[1] =  (extend.addr  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;
	for (i = 0; i < extend.len + 2; i++)
		checksum = checksum + extend_data[i];
	checksum = 0xff - (checksum & 0xff);
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x24);
	if (ret < 0)
		goto error;

	usleep_range(1000, 1000);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, 4, extend_data);
	if (ret < 0)
		goto error;
	if ((extend_data[21] & NFG8011B_SILI_ALG_CTRL_MASK) == config)
		chg_info("set config=0x%x success\n", config);
	else
		chg_err("set config=0x%x fail\n", config);
	if (config & NFG8011B_SI_ALG_DSG_ENABLE_MASK)
		chip->dsg_enable = true;
	else
		chip->dsg_enable = false;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	nfg8011b_deep_deinit(chip);
	return 0;

error:
	chg_err("set config=0x%x error\n", config);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	nfg8011b_deep_deinit(chip);
	return -EINVAL;
}

int nfg8011b_set_sili_ic_alg_term_volt(struct chip_bq27541 *chip, int sys_term_vol)
{
	int i;
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[36] = {0};
	u8 checksum = 0;
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DEEP_INFO_ADDR , 32 };

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	extend_data[8] = sys_term_vol & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[9] = (sys_term_vol  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[10] = (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[8] + extend_data[9]) & NFG8011B_DEEP_DISCHG_CHECK);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	ret = bq27541_write_i2c_block(chip, NFG8011B_AUTHENDATA_1ST, extend.len, extend_data + 2);
	if (ret < 0)
		goto error;

	extend_data[0] = extend.addr & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[1] =  (extend.addr  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;
	for (i = 0; i < extend.len + 2; i++)
		checksum = checksum + extend_data[i];
	checksum = 0xff - (checksum & 0xff);
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x24);
	if (ret < 0)
		goto error;

	try_count = NFG8011B_SUBCMD_TRY_COUNT;
	do {
		data_check = true;
		usleep_range(15000, 15000);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, 5, extend_data);
		if (extend_data[10] != (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[8] + extend_data[9]) & NFG8011B_DEEP_DISCHG_CHECK) ||
			sys_term_vol != ((extend_data[9] << 0x08) | extend_data[8])) {
			chg_err("count not match. extend_data[0x%2x, 0x%2x, 0x%2x]\n", extend_data[8], extend_data[9], extend_data[10]);
			data_check = false;
		}
	} while (!data_check && try_count--);
	if (!data_check)
		goto error;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	chg_info("set sys_term_voltage %d success\n", sys_term_vol);
	return 0;

error:
	chg_info("set sys_term_voltage fail\n");
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

int nfg8011b_get_sili_lifetime_status(struct chip_bq27541 *chip, struct oplus_gauge_lifetime *lifetime)
{
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[10] = {0};
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_LIFETIME_1_ADDR , 8};

	if (!chip || !lifetime)
		return -EINVAL;

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	lifetime->max_cell_vol = (extend_data[3] << 0x08) | extend_data[2];
	lifetime->max_charge_curr = (extend_data[5] << 0x08) | extend_data[4];
	lifetime->max_dischg_curr = (extend_data[7] << 0x08) | extend_data[6];
	lifetime->max_cell_temp = extend_data[8];
	lifetime->min_cell_temp = extend_data[9];
	chg_info("lifetime status:%d %d %d %d %d\n", lifetime->max_cell_vol, lifetime->max_charge_curr,
		lifetime->max_dischg_curr, lifetime->max_cell_temp, lifetime->min_cell_temp);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return 0;

error:
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

/* Check if firmware version >= min_version */
static bool bq27541_check_fw_version(struct chip_bq27541 *chip, const char *min_version)
{
	u8 version_cmd[2] = {NFG8011B_FW_VERSION, 0x00};
	u8 version_data[14] = {0};
	int ret;

	if (min_version == NULL) {
		chg_info("min_version is NULL\n");
		return false;
	}

	/* Read firmware version information */
	ret = bq27541_write_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, sizeof(version_cmd), version_cmd);
	if (ret < 0)
		return false;

	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, sizeof(version_data), version_data);
	if (ret < 0 || version_data[0] != NFG8011B_FW_VERSION || version_data[1] != 0x00) {
		chg_info("version read failed\n");
		return false;
	}

	/* version string comparison */
	return (memcmp(&(version_data[2]), min_version, 7) >= 0);
}

#define FW_VERSION_1_3_0_P1T4 (u8[]) {0x01, 0x03, 0x00, 'P', '1', 't', '4'}
int nfg8011b_set_soc_froce_to_0_threshold(struct chip_bq27541 *chip, u16 voltage_mv, u16 time_sec)
{
	u8 read_cmd[2] = {NFG8011B_FORCE_TO_0, 0x00};
	u8 write_data[6] = {NFG8011B_FORCE_TO_0, 0x00,
				(u8)voltage_mv, (u8)(voltage_mv >> 8),
				(u8)time_sec, (u8)(time_sec >> 8)};
	u8 response[6];
	u8 checksum = 0;
	int ret;
	int i;

	if (chip == NULL) {
		chg_info("chip is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	/* Version check */
	if (bq27541_check_fw_version(chip, FW_VERSION_1_3_0_P1T4) == false) {
		chg_info("firmware version too old, need >=1.3.0P1T4\n");
		ret = -EOPNOTSUPP;
		goto error;
	}

	/* 1. Read current value for verification */
	ret = bq27541_write_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, sizeof(read_cmd), read_cmd);
	if (ret < 0) goto error;
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, sizeof(response), response);
	if (ret < 0 || response[0] != 0xC3 || response[1] != 0x00) {
		chg_info("read verification failed\n");
		goto error;
	}

	/* 2. Write new threshold to RAM (with checksum) */
	ret = bq27541_write_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, sizeof(write_data), write_data);
	if (ret < 0)
		goto error;

	/* Calculate checksum */
	for (i = 0; i < sizeof(write_data); i++)
		checksum += write_data[i];
	checksum = 0xFF - checksum;

	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x08);
	if (ret < 0)
		goto error;

	/* 3. Read back for verification */
	ret = bq27541_write_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, sizeof(read_cmd), read_cmd);
	if (ret < 0) goto error;
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, sizeof(response), response);
	if (ret < 0 || memcmp(write_data, response, sizeof(write_data)) != 0) {
		chg_info("write verification failed\n");
		goto error;
	}

	chg_info("RAM froce_to_0 threshold updated success: %dmV %ds\n", voltage_mv, time_sec);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return 0;

error:
	chg_info("RAM froce_to_0 threshold update failed\n");
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return (ret < 0) ? ret : -EINVAL;
}

int nfg8011b_set_battery_full(struct chip_bq27541 *chip, bool full)
{
	u8 mac_data[4] = {0x31, 0x00, 0xAA, 0x55};
	u8 checksum = 0;
	int ret;
	int i;

	if (chip == NULL) {
		chg_info("chip is NULL\n");
		return -EINVAL;
	}

	if (full == false) {
		return 0;
	}

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	/* Version check */
	if (bq27541_check_fw_version(chip, FW_VERSION_1_3_0_P1T4) == false) {
		chg_info("firmware version too old, need >=1.3.0P1T4\n");
		ret = -EOPNOTSUPP;
		goto error;
	}

	/* Execute battery full report command */
	ret = bq27541_write_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, ARRAY_SIZE(mac_data), mac_data);
	if (ret < 0)
		goto error;

	for (i = 0; i < ARRAY_SIZE(mac_data); i++)
		checksum = checksum + mac_data[i];
	checksum = 0xff - (checksum & 0xff);
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x06);
	if (ret < 0)
		goto error;
	chg_info("force set battery full success\n");
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return 0;

error:
	chg_info("force set battery full fail\n");
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return (ret < 0) ? ret : -EINVAL;
}

int nfg8011b_get_sili_lifetime_info(struct chip_bq27541 *chip, u8 *info, int len)
{
	int i;
	int j;
	int ret;
	int size;
	int data = 0;
	int index = 0;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[34] = {0};
	bool support_fast_sampling;
	struct gauge_track_info_reg standard[] = {
		{ NFG8011B_REG_CIS_ALERT_LEVEL, 2},
	};
	struct gauge_track_info_reg *extend;
	struct gauge_track_info_reg nfg8011b_extend[] = {
		{ NFG8011B_SUBCMD_IPM_ADDR, 32, 0, 31},
		{ NFG8011B_SUBCMD_ALG_ADDR_1, 12, 0, 11},
		{ NFG8011B_SUBCMD_LIFETIME_1_ADDR, 8, 0, 7},
		{ NFG8011B_SUBCMD_LIFETIME_2_ADDR, 24, 0, 23},
		{ NFG8011B_SUBCMD_ISC_INFO_ADDR, 32, 22, 31},
		{ NFG8011B_SUBCMD_ISC_LAST_TEN_ADDR, 20, 0, 19},
		{ NFG8011B_REG_ENABLE_STATE, 1, 0, 0},
		{ NFG8011B_SUBCMD_SOH2_ADDR, 6, 0, 5},
	};

	extend = nfg8011b_extend;
	size = ARRAY_SIZE(nfg8011b_extend);

	for (i = 0; i < ARRAY_SIZE(standard); i++) {
		if (chip->fpga_support && is_return_pre_value(chip))
			return index;
		ret = bq27541_read_i2c(chip, standard[i].addr, &data);
		if (ret < 0)
			continue;
		index += snprintf(info + index, len - index,
			"0x%02x=%02x,%02x|", standard[i].addr, (data & 0xff), ((data >> 8) & 0xff));
	}

	support_fast_sampling = nfg8011b_firmware_support_fast_sampling(chip);
	for (i = 0; i < size; i++) {
		if (!support_fast_sampling && extend[i].addr == NFG8011B_REG_ENABLE_STATE)
			continue;

		try_count = NFG8011B_SUBCMD_TRY_COUNT;
try:
		mutex_lock(&chip->bq28z610_alt_manufacturer_access);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend[i].addr);
		if (ret < 0) {
			mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
			continue;
		}

		if (sizeof(extend_data) >= extend[i].len + 2) {
			usleep_range(1000, 1000);
			ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend[i].len + 2), extend_data);
			mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
			if (ret < 0)
				continue;
			data_check = (extend_data[1] << 0x8) | extend_data[0];
			if (try_count-- > 0 && data_check != extend[i].addr) {
				chg_info("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
					extend[i].addr, try_count, extend_data[0], extend_data[1]);
				usleep_range(2000, 2000);
				goto try;
			}
			if (try_count < 0)
				continue;
			index += snprintf(info + index, len - index, "0x%04x=", extend[i].addr);
			for (j = extend[i].start_index; j < extend[i].end_index; j++)
				index += snprintf(info + index, len - index, "%02x,", extend_data[j + 2]);
			index += snprintf(info + index, len - index, "%02x", extend_data[j + 2]);
			if (i < size - 1)
				usleep_range(2000, 2000);
		} else {
			mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		}
		if (i  <  size - 1)
			index += snprintf(info + index, len - index, "|");
	}

	return index;
}

int nfg8011b_get_sili_alg_application_info(struct chip_bq27541 *chip, u8 *info, int len)
{
	int i;
	int j;
	int ret;
	int size;
	int index = 0;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[34] = {0};
	struct gauge_track_info_reg *extend;
	struct gauge_track_info_reg nfg8011b_extend[] = {
		{ 0x00F0, 32, 0, 31 },
		{ 0x00F1, 32, 0, 31 },
		{ 0x00F2, 32, 0, 31 },
		{ 0x00F3, 32, 0, 31 },
		{ 0x00F4, 32, 0, 31 },
		{ 0x00F5, 32, 0, 31 },
		{ 0x00F6, 32, 0, 31 },
	};

	extend = nfg8011b_extend;
	size = ARRAY_SIZE(nfg8011b_extend);

	for (i = 0; i < size; i++) {
		try_count = NFG8011B_SUBCMD_TRY_COUNT;
try:
		mutex_lock(&chip->bq28z610_alt_manufacturer_access);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend[i].addr);
		if (ret < 0) {
			mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
			continue;
		}

		if (sizeof(extend_data) >= extend[i].len + 2) {
			usleep_range(1000, 1000);
			ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend[i].len + 2), extend_data);
			mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
			if (ret < 0)
				continue;
			data_check = (extend_data[1] << 0x8) | extend_data[0];
			if (try_count-- > 0 && data_check != extend[i].addr) {
				chg_info("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
					extend[i].addr, try_count, extend_data[0], extend_data[1]);
				usleep_range(2000, 2000);
				goto try;
			}
			if (try_count < 0)
				continue;
			for (j = extend[i].start_index; j <= extend[i].end_index; j++)
				index += snprintf(info + index, len - index, "%02x", extend_data[j + 2]);
			if (i < size - 1)
				usleep_range(2000, 2000);
		} else {
			mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		}
	}

	return index;
}

int nfg8011b_get_sili_chemistry_info(struct chip_bq27541 *chip, u8 *info, int len)
{
	u8 data[CHEM_ID_LENGTH] = {0};
	int ret = 0;

	if (!chip)
		return -EINVAL;

	if (is_return_pre_value(chip) == 1)
		return -EINVAL;

	if (len < CHEM_ID_LENGTH)
		return -EINVAL;

	if (atomic_read(&chip->locked) == 0) {
		mutex_lock(&chip->bq28z610_alt_manufacturer_access);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_CHEMISTRY_ID_EN_ADDR,
					   NFG8011B_CHEMISTRY_ID_CMD);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		ret = bq27541_read_i2c_block(chip,
			NFG8011B_CHEMISTRY_ID_ADDR,
			NFG8011B_CHEMISTRY_ID_SIZE, data);
		if (ret)
			goto error;
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		chg_info("chemistry id: [0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]\n",
			 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
		memcpy(info, data, CHEM_ID_LENGTH);
	}

	return ret;
error:
	chg_err("fail to get chemistry info\n");
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

int nfg8011b_get_last_cc(struct chip_bq27541 *chip, int *cc)
{
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[14] = {0};
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DEEP_INFO_ADDR , 12 };

	if (!chip || !cc)
		return -EINVAL;

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return chip->last_cc_pre;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	if (extend_data[13] != (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[11] + extend_data[12]) & NFG8011B_DEEP_DISCHG_CHECK)) {
		chg_err("check sum not match. extend_data[0x%2x, 0x%2x, 0x%2x]\n", extend_data[11], extend_data[12], extend_data[13]);
		goto error;
	}

	*cc = (extend_data[12] << 0x08) | extend_data[11];
	chg_info("cc:%d\n", *cc);
	chip->last_cc_pre = *cc;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return 0;

error:
	*cc = 0;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

int nfg8011b_set_last_cc(struct chip_bq27541 *chip, int cc)
{
	int i;
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[36] = {0};
	u8 checksum = 0;
	struct gauge_track_info_reg extend = { NFG8011B_SUBCMD_DEEP_INFO_ADDR , 32 };

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (extend.len + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != extend.addr) {
		chg_err("0x%4x not match. try_count=%d, extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
		extend.addr, try_count, extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	extend_data[11] = cc & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[12] = (cc  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[13] = (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[11] + extend_data[12]) & NFG8011B_DEEP_DISCHG_CHECK);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
	if (ret < 0)
		goto error;
	ret = bq27541_write_i2c_block(chip, NFG8011B_AUTHENDATA_1ST, extend.len, extend_data + 2);
	if (ret < 0)
		goto error;

	extend_data[0] = extend.addr & NFG8011B_DEEP_DISCHG_CHECK;
	extend_data[1] =  (extend.addr  >> NFG8011B_DEEP_SHIFT) & NFG8011B_DEEP_DISCHG_CHECK;
	for (i = 0; i < extend.len + 2; i++)
		checksum = checksum + extend_data[i];
	checksum = 0xff - (checksum & 0xff);
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x24);
	if (ret < 0)
		goto error;

	try_count = NFG8011B_SUBCMD_TRY_COUNT;
	do {
		data_check = true;
		usleep_range(15000, 15000);
		ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, extend.addr);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, 5, extend_data);
		if (extend_data[13] != (NFG8011B_DEEP_DISCHG_CHECK - (extend_data[11] + extend_data[12]) & NFG8011B_DEEP_DISCHG_CHECK) ||
			cc != ((extend_data[12] << 0x08) | extend_data[11])) {
			chg_err("count not match. extend_data[0x%2x, 0x%2x, 0x%2x]\n", extend_data[11], extend_data[12], extend_data[13]);
			data_check = false;
		}
	} while (!data_check && try_count--);
	if (!data_check)
		goto error;
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	chg_info("set last cc %d success\n", cc);
	return 0;

error:
	chg_info("set last cc fail\n");
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

#define NFG8011B_BLOCK_SIZE 32
static u8 nfg8011b_calc_checksum(u8 *buf, int len)
{
	u8 checksum = 0;
	while (len--)
		checksum += buf[len];
	return 0xff - checksum;
}

static int nfg8011b_block_check_conditions(struct chip_bq27541 *chip, u8 *buf, int len, int offset, bool do_checksum)
{
	if (!chip || !buf || offset < 0 || offset >= NFG8011B_BLOCK_SIZE || len <= 0 ||
	    (len + do_checksum > NFG8011B_BLOCK_SIZE) || (offset + len + do_checksum > NFG8011B_BLOCK_SIZE)) {
		chg_err("%soffset %d or len %d invalid\n", buf ? "buf is null or " : "", offset, len);
		return -EINVAL;
	}

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

	return 0;
}

int nfg8011b_read_block(struct chip_bq27541 *chip, int addr, u8 *buf, int len, int offset, bool do_checksum)
{
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_data[NFG8011B_BLOCK_SIZE + 2] = { 0 };
	u8 checksum = 0;

	ret = nfg8011b_block_check_conditions(chip, buf, len, offset, do_checksum);
	if (ret < 0)
		return ret;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (offset + len + do_checksum + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != addr) {
		chg_err("0x%04x not match. try_count=%d extend_data[0]=0x%2x, extend_data[1]=0x%2x\n", addr, try_count,
			extend_data[0], extend_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	if (do_checksum) {
		checksum = nfg8011b_calc_checksum(&extend_data[offset + 2], len);
		if (checksum != extend_data[offset + len + 2]) {
			chg_err("[%*ph]checksum not match. expect=0x%02x actual=0x%02x\n",
				offset + len + do_checksum + 2, extend_data, checksum, extend_data[offset + len + 2]);
			goto error;
		}
	}

	memcpy(buf, &extend_data[offset + 2], len);
	chg_info("addr=0x%04x offset=%d buf=[%*ph] do_checksum=%d read success\n", addr, offset, len, buf, do_checksum);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return 0;

error:
	chg_info("addr=0x%04x offset=%d buf=[%*ph] do_checksum=%d read fail\n", addr, offset, len, buf, do_checksum);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

int nfg8011b_write_block(struct chip_bq27541 *chip,
	int addr, u8 *buf, int len, int offset, bool do_checksum, bool read_back)
{
	int ret;
	int data_check;
	int try_count = NFG8011B_SUBCMD_TRY_COUNT;
	u8 extend_read_data[NFG8011B_BLOCK_SIZE + 2] = { 0 };
	u8 extend_write_data[NFG8011B_BLOCK_SIZE + 2] = { 0 };
	u8 checksum = 0;

	ret = nfg8011b_block_check_conditions(chip, buf, len, offset, do_checksum);
	if (ret < 0)
		return ret;

try:
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, NFG8011B_DATAFLASHBLOCK, (NFG8011B_BLOCK_SIZE + 2), extend_read_data);
	if (ret < 0)
		goto error;

	data_check = (extend_read_data[1] << 0x8) | extend_read_data[0];
	if (try_count-- > 0 && data_check != addr) {
		chg_err("0x%04x not match. try_count=%d offset=%d extend_data[0]=0x%2x, extend_data[1]=0x%2x\n", addr,
			try_count, offset, extend_read_data[0], extend_read_data[1]);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	memcpy(extend_write_data, extend_read_data, NFG8011B_BLOCK_SIZE + 2);
	memcpy(&extend_write_data[offset + 2], buf, len);
	if (do_checksum)
		extend_write_data[offset + len + 2] = nfg8011b_calc_checksum(buf, len);

	ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, addr);
	if (ret < 0)
		goto error;
	ret = bq27541_write_i2c_block(chip, NFG8011B_AUTHENDATA_1ST, NFG8011B_BLOCK_SIZE, extend_write_data + 2);
	if (ret < 0)
		goto error;
	checksum = nfg8011b_calc_checksum(extend_write_data, NFG8011B_BLOCK_SIZE + 2);
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENCHECKSUM, checksum);
	if (ret < 0)
		goto error;
	ret = bq27541_i2c_txsubcmd_onebyte(chip, NFG8011B_AUTHENLEN, 0x24);
	if (ret < 0)
		goto error;

	if (read_back) {
		try_count = NFG8011B_SUBCMD_TRY_COUNT;
		do {
			data_check = true;
			memset(extend_read_data, 0, NFG8011B_BLOCK_SIZE + 2);
			usleep_range(15000, 15000);
			ret = bq27541_i2c_txsubcmd(chip, NFG8011B_DATAFLASHBLOCK, addr);
			if (ret < 0)
				goto error;
			usleep_range(1000, 1000);
			ret = bq27541_read_i2c_block(chip,
				NFG8011B_DATAFLASHBLOCK, NFG8011B_BLOCK_SIZE + 2, extend_read_data);
			if (memcmp(extend_read_data, extend_write_data, NFG8011B_BLOCK_SIZE + 2)) {
				chg_err("reg not match.extend_read_data =[%*ph]\n",
					NFG8011B_BLOCK_SIZE + 2, extend_read_data);
				chg_err("reg not match.extend_write_data=[%*ph]\n",
					NFG8011B_BLOCK_SIZE + 2, extend_write_data);
				data_check = false;
			}
		} while (!data_check && try_count--);
		if (!data_check)
			goto error;
	}

	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	chg_info("addr=0x%04x offset=%d buf=[%*ph] write success\n", addr, offset, len, buf);
	return 0;

error:
	chg_info("addr=0x%04x offset=%d buf=[%*ph] write fail\n", addr, offset, len, buf);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return -EINVAL;
}

#define FW_VERSION_1_3_0_P1T2 (u8[]) {0x01, 0x03, 0x00, 'P', '1', 't', '2'}
int nfg8011b_soc_centi_init(struct chip_bq27541 *chip)
{
	int rc = 0;

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	if (bq27541_check_fw_version(chip, FW_VERSION_1_3_0_P1T2) == false) {
		chg_info("firmware version too old, need >= 1.3.0P1T2\n");
		rc = -ENOTSUPP;
		goto error;
	}
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	chip->cmd_addr.reg_soc_centi = NFG801B_REG_RSOC_CENTI;

	return 0;
error:
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return rc;
}

static bool nfg8011b_firmware_support_fast_sampling(struct chip_bq27541 *chip)
{
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	if (bq27541_check_fw_version(chip, FW_VERSION_1_3_0_P1T4) == false) {
		chg_info("firmware version too old, need >= 1.3.0P1T4\n");
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		return false;
	}
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);

	return true;
}

int nfg8011b_set_fast_sampling(struct chip_bq27541 *chip, bool enable)
{
	int ret;
	bool firmware_support;
	u8 enable_buf[] = {0xa1, 0xa2, 0xa3, 0xa4};
	u8 disable_buf[] = {0x01, 0x02, 0x03, 0x04};

	firmware_support = nfg8011b_firmware_support_fast_sampling(chip);
	if (!firmware_support)
		return -ENOTSUPP;

	if (enable)
		ret = nfg8011b_write_block(chip, NFG8011B_REG_ENABLE_FAST_SAMP,
			enable_buf, sizeof(enable_buf), 0, false, false);
	else
		ret = nfg8011b_write_block(chip, NFG8011B_REG_DISABLE_FAST_SAMP,
			disable_buf, sizeof(disable_buf), 0, false, false);

	if (!ret)
		chip->fast_sampling_enable = enable;

	return ret;
}

