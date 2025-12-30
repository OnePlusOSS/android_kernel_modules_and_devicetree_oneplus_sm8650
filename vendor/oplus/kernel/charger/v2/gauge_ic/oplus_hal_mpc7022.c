// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[MPC7022]([%s][%d]): " fmt, __func__, __LINE__

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <asm/unaligned.h>
#include <linux/init.h>
#ifdef MODULE
#include <asm/setup.h>
#endif
#else
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/soc/qcom/smem.h>
#endif

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/device_info.h>
#endif

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/regmap.h>
#include <oplus_chg_ic.h>
#include <linux/version.h>
#include<linux/gfp.h>
#include <linux/pinctrl/consumer.h>
#include <oplus_chg_module.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_vooc.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_smart_chg.h>
#include <oplus_mms_wired.h>
#include "test-kit.h"
#include "oplus_hal_mpc7022.h"
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
#include <debug-kit.h>
#endif

#define GAUGE_ERROR				(-1)
#define GAUGE_OK				0
#define BATT_FULL_ERROR				2
#define VOLT_MIN				1000
#define VOLT_MAX				5000
#define CURR_MAX				20000
#define CURR_MIN				-25000
#define TEMP_MAX				(1000 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN)
#define TEMP_MIN				(-400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN)
#define SOH_MIN					0
#define SOH_MAX					100
#define FCC_MIN					10
#define FCC_MAX					12000
#define CC_MIN					0
#define CC_MAX					5000
#define QMAX_MIN				10
#define QMAX_MAX				12000
#define SOC_MIN					0
#define SOC_MAX					100
#define SOC_CENTI_MIN				0
#define SOC_CENTI_MAX				10000
#define RETRY_CNT				3
#define I2C_ERR_MAX				2
#define CALIB_TIME_CHECK_ARGS			6
#define CHECK_IIC_RECOVER_TIME			5000
#define GAUGE_SHA256_AUTH_MSG_LEN		32
#define SMEM_RESERVED_BOOT_INFO_FOR_APPS	418
#define EXTEND_CMD_TRY_COUNT			3
#define SEAL_POLLING_RETRY_LIMIT		100

#define GAUGE_AUTH_MSG_LEN			20
#define WLS_AUTH_RANDOM_LEN			8
#define WLS_AUTH_ENCODE_LEN			8
#define GAUGE_SHA256_AUTH_MSG_LEN		32
#define UFCS_AUTH_MSG_LEN			16
#define GET_BATTERY_AUTH_RETRY_COUNT 		5

enum mpc7022_volt_type {
	MPC7022_CELL_MAX_VOLT,
	MPC7022_CELL_MIN_VOLT,
	MPC7022_CELL_1_VOLT,
	MPC7022_CELL_2_VOLT,
};

enum mpc7022_dod_parameter_type {
	MPC7022_CELL_DOD_PASSED_Q,
	MPC7022_CELL_1_DOD0,
	MPC7022_CELL_2_DOD0,
};

enum mpc7022_qmax_parameter_type {
	MPC7022_CELL_QMAX_PASSED_Q,
	MPC7022_CELL_1_QMAX,
	MPC7022_CELL_2_QMAX,
};

enum mpc7022_security_mode {
	MPC7022_MODE_RESERVED,
	MPC7022_MODE_FULL_ACCESS,
	MPC7022_MODE_UNSEALED,
	MPC7022_MODE_SEALED
};

struct oplus_gauge_auth_result {
	int result;
	unsigned char msg[GAUGE_AUTH_MSG_LEN];
	unsigned char rcv_msg[GAUGE_AUTH_MSG_LEN];
};

struct wls_chg_auth_result {
	unsigned char random_num[WLS_AUTH_RANDOM_LEN];
	unsigned char encode_num[WLS_AUTH_ENCODE_LEN];
};

struct oplus_ufcs_auth_result {
	unsigned char msg[UFCS_AUTH_MSG_LEN];
};

struct mpc7022_hmac_result{
	u8 msg[GAUGE_SHA256_AUTH_MSG_LEN];
	u8 rcv_msg[GAUGE_SHA256_AUTH_MSG_LEN];
};

struct mpc7022_hmac_mapping{
	struct oplus_gauge_auth_result rst_k0;
	struct oplus_gauge_auth_result rst_k1;
	struct wls_chg_auth_result wls_auth_data;
	struct oplus_gauge_auth_result rst_k2;
	struct oplus_ufcs_auth_result ufcs_k0;
	struct mpc7022_hmac_result sha256_rst_k0;
};

struct mpc7022_block_access {
	int addr;
	int len;
	int start_index;
	int end_index;
};

struct mpc7022_cmd_address {
	u8 reg_temp;
	u8 reg_volt;
	u8 reg_flags;
	u8 reg_ti;
	u8 reg_rm;
	u8 reg_fcc;
	u8 reg_ai;
	u8 reg_soc;
	u8 reg_soh;
	u8 reg_cc;
	u8 reg_soc_centi;
};

struct chip_mpc7022 {
	struct i2c_client *client;
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	struct regmap *regmap;
	struct oplus_device_bus *odb;
#endif

	atomic_t locked;
	atomic_t suspended;
	atomic_t i2c_err_count;

	struct mutex chip_mutex;
	struct mutex calib_time_mutex;
	struct mutex extended_cmd_access;
	struct mutex block_access;

	int batt_num;
	int gauge_num;

	bool i2c_err;

	int soc_pre;
	int soc_centi_pre;
	int temp_pre;
	int current_pre;
	int cc_pre;
	int soh_pre;
	int fcc_pre;
	int rm_pre;
	int batt_max_volt_pre;
	int batt_min_volt_pre;
	int batt_cell_1_volt_pre;
	int batt_cell_2_volt_pre;

	int batt_cell_1_dod0_pre;
	int batt_cell_2_dod0_pre;
	int batt_cell_dod_passed_q_pre;

	int batt_cell_1_qmax_pre;
	int batt_cell_2_qmax_pre;
	int batt_cell_qmax_passed_q_pre;

	int deep_count_pre;
	int deep_term_volt_pre;
	int deep_last_cc_pre;

	int car_c_pre;

	struct mpc7022_cmd_address cmd_addr;

	bool calib_info_init;
	int dod_time;
	int qmax_time;
	int dod_time_pre;
	int qmax_time_pre;
	int calib_check_args[CALIB_TIME_CHECK_ARGS];
	int calib_check_args_pre[CALIB_TIME_CHECK_ARGS];

	struct delayed_work check_iic_recover;
	/* workaround for I2C pull SDA can't trigger error issue 230504153935012779 */
	bool i2c_rst_ext;
	bool err_status;

	struct battery_manufacture_info battinfo;
	struct delayed_work get_manu_battinfo_work;
	int get_temp;
	int temp_err_count;
};

static int oplus_mpc7022_init(struct oplus_chg_ic_dev *ic_dev);
static int oplus_mpc7022_exit(struct oplus_chg_ic_dev *ic_dev);
static int mpc7022_shutdown_set_cuv_state(struct chip_mpc7022 *chip);

#ifdef CONFIG_OPLUS_CHARGER_MTK
/* workaround for I2C pull SDA can't trigger error issue 230504153935012779 */
#ifdef CONFIG_OPLUS_FG_ERROR_RESET_I2C
/* this workaround only for flamingo, for scanning tool issue */
void __attribute__((weak)) oplus_set_fg_err_flag(struct i2c_adapter *adap, bool flag);
#endif
/* end workaround 230504153935012779 */
#endif

static __inline__ void mpc7022_push_i2c_err(struct chip_mpc7022 *chip, bool read)
{
	if (unlikely(!chip->ic_dev))
		return;

	if (unlikely(!chip->ic_dev->online))
		return;

	chip->i2c_err = true;

	if (atomic_read(&chip->i2c_err_count) > I2C_ERR_MAX)
		return;

	atomic_inc(&chip->i2c_err_count);
	if (atomic_read(&chip->i2c_err_count) > I2C_ERR_MAX) {
		oplus_mpc7022_exit(chip->ic_dev);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_OFFLINE);
		schedule_delayed_work(&chip->check_iic_recover, msecs_to_jiffies(CHECK_IIC_RECOVER_TIME));
	} else {
		oplus_chg_ic_creat_err_msg(chip->ic_dev, OPLUS_IC_ERR_I2C, 0,
					   read ? "read error" : "write error");
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
	}
}

static __inline__ void mpc7022_i2c_err_clr(struct chip_mpc7022 *chip)
{
	if (unlikely(chip->i2c_err)) {
		/* workaround for I2C pull SDA can't trigger error issue 230504153935012779 */
		if (chip->i2c_rst_ext && chip->err_status)
			return;
		/* end workaround 230504153935012779 */

		chip->i2c_err = false;
		atomic_set(&chip->i2c_err_count, 0);
		oplus_mpc7022_init(chip->ic_dev);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ONLINE);
	}
}

__maybe_unused static int mpc7022_read_i2c(
	struct chip_mpc7022 *chip, int cmd, int *returnData)
{
	int retry = RETRY_CNT;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	int rc;
#endif

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return -EINVAL;
	}
	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	do {
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
		rc = oplus_dev_bus_bulk_read(chip->odb, cmd, returnData, 2);
		if (rc < 0)
			*returnData = rc;
#else
		*returnData = i2c_smbus_read_word_data(chip->client, cmd);
#endif
		if (*returnData < 0 && retry > 0)
			usleep_range(5000, 5000);
	} while (*returnData < 0 && retry-- > 0);
	mutex_unlock(&chip->chip_mutex);

	if (*returnData < 0) {
		chg_err("read err, rc = %d\n", *returnData);
#ifdef CONFIG_OPLUS_CHARGER_MTK
		if (chip->i2c_rst_ext) {
			chip->err_status = false;
#ifdef CONFIG_OPLUS_FG_ERROR_RESET_I2C
/* this workaround only for flamingo, for scanning tool issue */
			oplus_set_fg_err_flag(chip->client->adapter, false);
#endif
		}
#endif
		mpc7022_push_i2c_err(chip, true);
		return 1;
	}

	mpc7022_i2c_err_clr(chip);
	return 0;
}

__maybe_unused static int mpc7022_i2c_txsubcmd(
	struct chip_mpc7022 *chip, int cmd, int writeData)
{
	int rc;
	int retry = RETRY_CNT;

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return -EINVAL;
	}

	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	do {
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
		rc = oplus_dev_bus_bulk_write(chip->odb, cmd, &writeData, 2);
#else
		rc = i2c_smbus_write_word_data(chip->client, cmd, writeData);
#endif
		if (rc < 0 && retry > 0)
			usleep_range(5000, 5000);
	} while (rc < 0 && retry-- > 0);
	mutex_unlock(&chip->chip_mutex);

	if (rc < 0) {
		chg_err("write err, rc = %d\n", rc);
		mpc7022_push_i2c_err(chip, false);
		return -EINVAL;
	}

	mpc7022_i2c_err_clr(chip);

	return 0;
}

__maybe_unused static int mpc7022_write_i2c_block(
	struct chip_mpc7022 *chip, u8 cmd, u8 length, u8 *writeData)
{
	int rc;
	int retry = RETRY_CNT;

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return -EINVAL;
	}

	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	do {
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
		rc = oplus_dev_bus_bulk_write(chip->odb, cmd, writeData, length);
#else
		rc = i2c_smbus_write_i2c_block_data(chip->client, cmd, length, writeData);
#endif
		if (rc < 0 && retry > 0)
			usleep_range(5000, 5000);
	} while (rc < 0 && retry-- > 0);
	mutex_unlock(&chip->chip_mutex);

	if (rc < 0) {
		chg_err("write err, rc = %d\n", rc);
		mpc7022_push_i2c_err(chip, false);
		return -EINVAL;
	}

	mpc7022_i2c_err_clr(chip);

	return 0;
}

__maybe_unused static int mpc7022_read_i2c_block(
	struct chip_mpc7022 *chip, u8 cmd, u8 length, u8 *returnData)
{
	int rc;
	int retry = RETRY_CNT;

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return -EINVAL;
	}

	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	do {
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
		rc = oplus_dev_bus_bulk_read(chip->odb, cmd, returnData, length);
#else
		rc = i2c_smbus_read_i2c_block_data(chip->client, cmd, length, returnData);
#endif
		if (rc < 0 && retry > 0)
			usleep_range(5000, 5000);
	} while (rc < 0 && retry-- > 0);
	mutex_unlock(&chip->chip_mutex);

	if (rc < 0) {
		chg_err("read err, rc = %d\n", rc);
		mpc7022_push_i2c_err(chip, true);
		return -EINVAL;
	}

	mpc7022_i2c_err_clr(chip);

	return 0;
}

__maybe_unused static int mpc7022_read_i2c_onebyte(
	struct chip_mpc7022 *chip, u8 cmd, u8 *returnData)
{
	int rc;
	int retry = RETRY_CNT;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	unsigned int buf;
#endif

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return -EINVAL;
	}

	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	do {
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
		rc = oplus_dev_bus_read(chip->odb, cmd, &buf);
		if (rc >= 0)
			rc = buf;
#else
		rc = i2c_smbus_read_byte_data(chip->client, cmd);
#endif
		if (rc < 0 && retry > 0)
			usleep_range(5000, 5000);
	} while (rc < 0 && retry-- > 0);

	if (rc >= 0)
		*returnData = (u8)rc;
	mutex_unlock(&chip->chip_mutex);

	if (rc < 0) {
		chg_err("read err, rc = %d\n", rc);
		mpc7022_push_i2c_err(chip, true);
		return -EINVAL;
	}

	mpc7022_i2c_err_clr(chip);

	return 0;
}

__maybe_unused static int mpc7022_i2c_txsubcmd_onebyte(
	struct chip_mpc7022 *chip, u8 cmd, u8 writeData)
{
	int rc;
	int retry = RETRY_CNT;

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return -EINVAL;
	}

	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	do {
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	rc = oplus_dev_bus_write(chip->odb, cmd, writeData);
#else
	rc = i2c_smbus_write_byte_data(chip->client, cmd, writeData);
#endif
	if (rc < 0 && retry > 0)
			usleep_range(5000, 5000);
	} while (rc < 0 && retry-- > 0);
	mutex_unlock(&chip->chip_mutex);

	if (rc < 0) {
		chg_err("write err, rc = %d\n", rc);
		mpc7022_push_i2c_err(chip, true);
		return -EINVAL;
	}

	mpc7022_i2c_err_clr(chip);

	return 0;
}

static inline bool is_chip_suspended_or_locked(struct chip_mpc7022 *chip)
{
	return atomic_read(&chip->suspended) || atomic_read(&chip->locked);
}

static u8 mpc7022_calc_checksum(u8 *buf, int len)
{
	u8 checksum = 0;

	while (len--)
		checksum += buf[len];

	return 0xff - checksum;
}

static int mpc7022_block_check_conditions(
	struct chip_mpc7022 *chip, u8 *buf, int len, int offset, bool do_checksum, int block_size)
{
	if (!chip || !buf || block_size > MPC7022_BLOCK_SIZE || offset < 0 || offset >= block_size || len <= 0 ||
	    (len + do_checksum > block_size) || (offset + len + do_checksum > block_size)) {
		chg_err("%soffset %d or len %d invalid\n", buf ? "buf is null or " : "", offset, len);
		return -EINVAL;
	}

	return 0;
}

static int __mpc7022_read_block(
	struct chip_mpc7022 *chip, int addr, u8 *extend_data, int len, bool access_lock)
{
	int ret;
	int data_check;
	int try_count = EXTEND_CMD_TRY_COUNT;

	do {
		if (access_lock)
			mutex_lock(&chip->extended_cmd_access);
		ret = mpc7022_i2c_txsubcmd(chip, MPC7022_DATA_FLASH_BLOCK, addr);
		usleep_range(1000, 1000);
		ret |= mpc7022_read_i2c_block(chip, MPC7022_DATA_FLASH_BLOCK, 2, extend_data);
		ret |= mpc7022_read_i2c_block(chip, MPC7022_DATA_FLASH_START, len, &extend_data[2]);
		if (access_lock)
			mutex_unlock(&chip->extended_cmd_access);

		data_check = (extend_data[1] << 0x8) | extend_data[0];
		if (data_check == addr && !ret)
			break;
		usleep_range(2000, 2000);
		chg_err("0x%04x not match. try_count=%d extend_data[0]=0x%2x, extend_data[1]=0x%2x\n", addr, try_count,
			extend_data[0], extend_data[1]);
	} while (try_count-- > 0);

	if (data_check == addr && !ret)
		return 0;

	return -EINVAL;
}

static int mpc7022_read_block(
	struct chip_mpc7022 *chip, int addr, u8 *buf, int len, int offset, bool do_checksum, bool access_lock)
{
	int ret;
	u8 checksum;
	u8 extend_data[MPC7022_BLOCK_SIZE + 2] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	ret = mpc7022_block_check_conditions(chip, buf, len, offset, do_checksum, MPC7022_BLOCK_SIZE);
	if (ret < 0)
		return ret;

	mutex_lock(&chip->block_access);
	ret = __mpc7022_read_block(chip, addr, extend_data, (offset + len + do_checksum), access_lock);
	if (ret < 0)
		goto error;

	if (do_checksum) {
		checksum = mpc7022_calc_checksum(&extend_data[offset + 2], len);
		if (checksum != extend_data[offset + len + 2]) {
			chg_err("[%*ph]checksum not match. expect=0x%02x actual=0x%02x\n",
				offset + len + do_checksum + 2, extend_data, checksum, extend_data[offset + len + 2]);
			goto error;
		}
	}

	memmove(buf, &extend_data[offset + 2], len);
	chg_info("addr=0x%04x offset=%d buf=[%*ph] do_checksum=%d read success\n", addr, offset, len, buf, do_checksum);
	mutex_unlock(&chip->block_access);
	return 0;

error:
	chg_info("addr=0x%04x offset=%d buf=[%*ph] do_checksum=%d read fail\n", addr, offset, len, buf, do_checksum);
	mutex_unlock(&chip->block_access);
	return -EINVAL;
}

static int __mpc7022_write_block(
	struct chip_mpc7022 *chip, int addr, u8 *extend_write_data, int block_size, bool access_lock)
{
	int ret;
	u8 checksum;

	if (access_lock)
		mutex_lock(&chip->extended_cmd_access);
	ret = mpc7022_i2c_txsubcmd(chip, MPC7022_DATA_FLASH_BLOCK, addr);
	ret |= mpc7022_write_i2c_block(chip, MPC7022_EXTEND_DATA_ADDR, block_size, extend_write_data + 2);
	if (ret < 0)
		goto error;

	checksum = mpc7022_calc_checksum(extend_write_data, block_size + 2);
	ret |= mpc7022_i2c_txsubcmd_onebyte(chip, MPC7022_EXTEND_DATA_CHECKSUM_ADDR, checksum);
	ret |= mpc7022_i2c_txsubcmd_onebyte(chip, MPC7022_EXTEND_DATA_LEN_ADDR, block_size + 2 + 2);
	if (ret < 0)
		goto error;

	if (access_lock)
		mutex_unlock(&chip->extended_cmd_access);
	return 0;

error:
	if (access_lock)
		mutex_unlock(&chip->extended_cmd_access);
	return -EINVAL;
}

static int __mpc7022_read_back_check(
	struct chip_mpc7022 *chip, int addr, u8 *extend_write_data,
	u8 *extend_read_data, int block_size, bool access_lock, bool read_back)
{
	bool data_check;
	int try_count = EXTEND_CMD_TRY_COUNT;

	if (!read_back)
		return 0;

	do {
		data_check = true;
		memset(extend_read_data, 0, block_size + 2);
		usleep_range(1000, 1000);
		if (access_lock)
			mutex_lock(&chip->extended_cmd_access);
		mpc7022_i2c_txsubcmd(chip, MPC7022_DATA_FLASH_BLOCK, addr);
		usleep_range(1000, 1000);
		mpc7022_read_i2c_block(chip, MPC7022_DATA_FLASH_BLOCK, 2, extend_read_data);
		mpc7022_read_i2c_block(chip, MPC7022_DATA_FLASH_START, block_size, &extend_read_data[2]);
		if (access_lock)
			mutex_unlock(&chip->extended_cmd_access);
		if (memcmp(extend_read_data, extend_write_data, block_size + 2)) {
			chg_err("reg not match.extend_read_data =[%*ph]\n", block_size + 2, extend_read_data);
			chg_err("reg not match.extend_write_data=[%*ph]\n", block_size + 2, extend_write_data);
			data_check = false;
		}
	} while (!data_check && try_count-- > 0);

	if (!data_check)
		return -EINVAL;

	return 0;
}

static int mpc7022_write_block(
	struct chip_mpc7022 *chip, int addr, u8 *buf, int len, int offset,
	bool do_checksum, int block_size, bool access_lock, bool read_back)
{
	int ret;
	u8 extend_read_data[MPC7022_BLOCK_SIZE + 2] = { 0 };
	u8 extend_write_data[MPC7022_BLOCK_SIZE + 2] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	ret = mpc7022_block_check_conditions(chip, buf, len, offset, do_checksum, block_size);
	if (ret < 0)
		return ret;

	mutex_lock(&chip->block_access);
	ret = __mpc7022_read_block(chip, addr, extend_read_data, block_size, access_lock);
	if (ret < 0)
		goto error;

	memmove(extend_write_data, extend_read_data, block_size + 2);
	memmove(&extend_write_data[offset + 2], buf, len);
	if (do_checksum)
		extend_write_data[offset + len + 2] = mpc7022_calc_checksum(buf, len);

	ret = __mpc7022_write_block(chip, addr, extend_write_data, block_size, access_lock);
	if (ret < 0)
		goto error;

	usleep_range(15000, 15000);
	ret =  __mpc7022_read_back_check(chip, addr, extend_write_data,
		extend_read_data, block_size, access_lock, read_back);
	if (ret < 0)
		goto error;

	mutex_unlock(&chip->block_access);
	chg_info("addr=0x%04x offset=%d buf=[%*ph] write success\n", addr, offset, len, buf);
	return 0;

error:
	chg_info("addr=0x%04x offset=%d buf=[%*ph] write fail\n", addr, offset, len, buf);
	mutex_unlock(&chip->block_access);
	return -EINVAL;
}

static bool normal_range_judge(int max, int min, int data)
{
	if (data > max || data < min)
		return false;

	return true;
}

static int mpc7022_get_battery_cc(struct chip_mpc7022 *chip)
{
	int ret;
	int cc = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return 0;

	if (is_chip_suspended_or_locked(chip))
		return chip->cc_pre;

	do {
		ret = mpc7022_read_i2c(chip, chip->cmd_addr.reg_cc, &cc);
		if (ret) {
			dev_err(chip->dev, "error reading cc.\n");
			return chip->cc_pre;
		}
		if (normal_range_judge(CC_MAX, CC_MIN, cc))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("cc abnormal, retry:%d, cc:%d\n", retry, cc);
	} while (retry-- > 0);

	if (retry < 0)
		return chip->cc_pre;

	chip->cc_pre = cc;
	return cc;
}

static int mpc7022_get_battery_fcc(struct chip_mpc7022 *chip)
{
	int ret;
	int fcc = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return 0;

	if (is_chip_suspended_or_locked(chip))
		return chip->fcc_pre;

	do {
		ret = mpc7022_read_i2c(chip, chip->cmd_addr.reg_fcc, &fcc);
		if (ret) {
			dev_err(chip->dev, "error reading fcc.\n");
			return chip->fcc_pre;
		}
		if (normal_range_judge(FCC_MAX, FCC_MIN, fcc))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("fcc abnormal, retry:%d, fcc:%d\n", retry, fcc);
	} while (retry--> 0);

	if (retry < 0)
		return chip->fcc_pre;

	chip->fcc_pre = fcc;
	return fcc;
}

static int mpc7022_get_battery_soh(struct chip_mpc7022 *chip)
{
	int ret;
	int soh = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return 0;

	if (is_chip_suspended_or_locked(chip))
		return chip->soh_pre;

	do {
		ret = mpc7022_read_i2c(chip, chip->cmd_addr.reg_soh, &soh);
		if (ret) {
			dev_err(chip->dev, "error reading soh.\n");
			return chip->soh_pre;
		}
		if (normal_range_judge(SOH_MAX, SOH_MIN, soh))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("soh abnormal, retry:%d, soh:%d\n", retry, soh);
	} while (retry-- > 0);

	if (retry < 0)
		return chip->soh_pre;

	chip->soh_pre = soh;
	return soh;
}

static int mpc7022_get_pre_batt_volt(
	struct chip_mpc7022 *chip, enum mpc7022_volt_type type)
{
	int volt = 0;

	switch (type) {
	case MPC7022_CELL_MAX_VOLT:
		volt = chip->batt_max_volt_pre;
		break;
	case MPC7022_CELL_MIN_VOLT:
		volt = chip->batt_min_volt_pre;
		break;
	case MPC7022_CELL_1_VOLT:
		volt = chip->batt_cell_1_volt_pre;
		break;
	case MPC7022_CELL_2_VOLT:
		volt = chip->batt_cell_2_volt_pre;
		break;
	default:
		break;
	}

	return volt;
}

static int mpc7022_update_pre_batt_volt(
	struct chip_mpc7022 *chip, enum mpc7022_volt_type type, int batt_cell_1_volt, int batt_cell_2_volt)
{
	int volt = 0;
	int batt_max_volt;
	int batt_min_volt;

	batt_max_volt = batt_cell_1_volt > batt_cell_2_volt ? batt_cell_1_volt : batt_cell_2_volt;
	batt_min_volt = batt_cell_1_volt > batt_cell_2_volt ? batt_cell_2_volt : batt_cell_1_volt;

	switch (type) {
	case MPC7022_CELL_MAX_VOLT:
		chip->batt_max_volt_pre = batt_max_volt;
		volt = batt_max_volt;
		break;
	case MPC7022_CELL_MIN_VOLT:
		chip->batt_min_volt_pre = batt_min_volt;
		volt = batt_min_volt;
		break;
	case MPC7022_CELL_1_VOLT:
		chip->batt_cell_1_volt_pre = batt_cell_1_volt;
		volt = batt_cell_1_volt;
		break;
	case MPC7022_CELL_2_VOLT:
		chip->batt_cell_2_volt_pre = batt_cell_2_volt;
		volt = batt_cell_2_volt;
		break;
	default:
		chg_err("volt type err\n");
		break;
	}

	return volt;
}

static int mpc7022_get_batt_volt(struct chip_mpc7022 *chip, enum mpc7022_volt_type type)
{
	int ret;
	int volt = 0;
	int batt_cell_1_volt = 0;
	int batt_cell_2_volt = 0;
	int retry = RETRY_CNT;
	u8 buf[MPC7022_VOLT_NUM_SIZE] = {0};

	if (!chip)
		return 0;

	if (is_chip_suspended_or_locked(chip)) {
		volt = mpc7022_get_pre_batt_volt(chip, type);
		return volt;
	}

	do {
		ret = mpc7022_read_block(chip, MPC7022_REG_DA_STATUS1,
			buf, MPC7022_VOLT_NUM_SIZE, 0, false, true);
		if (ret) {
			dev_err(chip->dev, "error reading volt.\n");
			return mpc7022_get_pre_batt_volt(chip, type);
		}
		batt_cell_1_volt = (buf[1] << 8) | buf[0];
		batt_cell_2_volt = (buf[3] << 8) | buf[2];
		if (normal_range_judge(VOLT_MAX, VOLT_MIN, batt_cell_1_volt) &&
		    normal_range_judge(VOLT_MAX, VOLT_MIN, batt_cell_2_volt))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("volt abnormal, retry:%d, volt:%d\n", retry, volt);
	} while (retry--);

	if (retry < 0)
		return mpc7022_get_pre_batt_volt(chip, type);

	volt = mpc7022_update_pre_batt_volt(chip, type, batt_cell_1_volt, batt_cell_2_volt);

	return volt;
}

static int mpc7022_handle_batt_temp_err(struct chip_mpc7022 *chip)
{
	chip->temp_err_count++;

	if (chip->temp_err_count > 1) { /* the second time still failed */
		chip->temp_err_count = 0;
		chip->temp_pre = -400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
		return -400; /* default -40 c */
	}

	return chip->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

static int mpc7022_get_battery_temperature(struct chip_mpc7022 *chip)
{
	int ret;
	int temp = 0;
	int retry = RETRY_CNT;

	if (is_chip_suspended_or_locked(chip))
		return chip->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;

	do {
		ret = mpc7022_read_i2c(chip, chip->cmd_addr.reg_temp, &temp);
		if (ret) {
			dev_err(chip->dev, "error reading temp.\n");
			return mpc7022_handle_batt_temp_err(chip);
		}
		chip->temp_err_count = 0;
		if (normal_range_judge(TEMP_MAX, TEMP_MIN, temp))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("temp abnormal, retry:%d, temp:%d\n", retry, (temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN));
	} while (retry--);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (chip->i2c_rst_ext) {
		if (!temp) {
			chg_err("fg read temperature i2c error, set err flag\n");
			chip->err_status = true;
#ifdef CONFIG_OPLUS_FG_ERROR_RESET_I2C
			oplus_set_fg_err_flag(chip->client->adapter, true);
#endif
			mpc7022_push_i2c_err(chip, true);
			temp = -400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
		} else {
			chip->err_status = false;
#ifdef CONFIG_OPLUS_FG_ERROR_RESET_I2C
			oplus_set_fg_err_flag(chip->client->adapter, false);
#endif
			mpc7022_i2c_err_clr(chip);
		}
	}
#endif

	if (temp > TEMP_MAX)
		temp = chip->temp_pre;
	chip->temp_pre = temp;

	return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

static int mpc7022_get_batt_remaining_capacity(struct chip_mpc7022 *chip)
{
	int ret;
	int cap = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return -1;

	if (is_chip_suspended_or_locked(chip))
		return chip->rm_pre;

	do {
		ret = mpc7022_read_i2c(chip, chip->cmd_addr.reg_rm, &cap);
		if (ret) {
			dev_err(chip->dev, "error reading cap.\n");
			return chip->rm_pre;
		}
		if (normal_range_judge(FCC_MAX, 0, cap))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("rm abnormal, retry:%d, cap:%d\n", retry, cap);
	} while (retry--);

	if (retry < 0)
		return chip->rm_pre;

	chip->rm_pre = cap;

	return cap;
}

static int mpc7022_get_pre_dod0(
	struct chip_mpc7022 *chip, enum mpc7022_dod_parameter_type type)
{
	int dod_parameters = 0;

	switch (type) {
	case MPC7022_CELL_1_DOD0:
		dod_parameters = chip->batt_cell_1_dod0_pre;
		break;
	case MPC7022_CELL_2_DOD0:
		dod_parameters = chip->batt_cell_2_dod0_pre;
		break;
	default:
		break;
	}

	return dod_parameters;
}

static int mpc7022_update_pre_dod0(
	struct chip_mpc7022 *chip, enum mpc7022_dod_parameter_type type,
	int batt_cell_1_dod0, int batt_cell_2_dod0)
{
	int dod_parameters = 0;

	switch (type) {
	case MPC7022_CELL_1_DOD0:
		chip->batt_cell_1_dod0_pre = batt_cell_1_dod0;
		dod_parameters = batt_cell_1_dod0;
		break;
	case MPC7022_CELL_2_DOD0:
		chip->batt_cell_2_dod0_pre = batt_cell_2_dod0;
		dod_parameters = batt_cell_2_dod0;
		break;
	default:
		chg_err("dod_parameters type err\n");
		break;
	}

	return dod_parameters;
}

static int mpc7022_get_dod0(
	struct chip_mpc7022 *chip, enum mpc7022_dod_parameter_type type)
{
	int ret;
	int dod_parameters;
	int batt_cell_1_dod0;
	int batt_cell_2_dod0;
	u8 buf[MPC7022_DOD_NUM_SIZE] = {0};

	if (!chip)
		return 0;

	if (is_chip_suspended_or_locked(chip)) {
		dod_parameters = mpc7022_get_pre_dod0(chip, type);
		return dod_parameters;
	}

	ret = mpc7022_read_block(
		chip, MPC7022_REG_DOD0, buf, MPC7022_DOD_NUM_SIZE, MPC7022_DOD_OFFSET, false, true);
	if (ret) {
		chg_err("error reading dod0\n");
		return mpc7022_get_pre_dod0(chip, type);
	}
	batt_cell_1_dod0 = (buf[1] << 8) | buf[0];
	batt_cell_2_dod0 = (buf[3] << 8) | buf[2];

	dod_parameters = mpc7022_update_pre_dod0(chip, type, batt_cell_1_dod0, batt_cell_2_dod0);

	return dod_parameters;
}

static int mpc7022_get_dod_passed_q(struct chip_mpc7022 *chip)
{
	int ret;
	int batt_cell_dod_passed_q;
	u8 buf[MPC7022_DOD_PASSED_Q_NUM_SIZE] = {0};

	if (!chip)
		return 0;

	if (is_chip_suspended_or_locked(chip)) {
		return chip->batt_cell_dod_passed_q_pre;
	}

	ret = mpc7022_read_block(
		chip, MPC7022_REG_IT_STATUS2, buf, MPC7022_DOD_PASSED_Q_NUM_SIZE, MPC7022_DOD_PASSED_Q_OFFSET, false, true);
	if (ret) {
		chg_err("error reading dod_passed_q\n");
		return chip->batt_cell_dod_passed_q_pre;
	}
	batt_cell_dod_passed_q = (buf[5] << 8) | buf[4];
	if (batt_cell_dod_passed_q & 0x8000)
		batt_cell_dod_passed_q = -((~(batt_cell_dod_passed_q - 1)) & 0xFFFF);

	chip->batt_cell_dod_passed_q_pre = batt_cell_dod_passed_q;

	return batt_cell_dod_passed_q;
}

static int mpc7022_get_pre_qmax_parameters(
	struct chip_mpc7022 *chip, enum mpc7022_qmax_parameter_type type)
{
	int qmax_parameters = 0;

	switch (type) {
	case MPC7022_CELL_1_QMAX:
		qmax_parameters = chip->batt_cell_1_qmax_pre;
		break;
	case MPC7022_CELL_2_QMAX:
		qmax_parameters = chip->batt_cell_2_qmax_pre;
		break;
	case MPC7022_CELL_QMAX_PASSED_Q:
		qmax_parameters = chip->batt_cell_qmax_passed_q_pre;
		break;
	default:
		break;
	}

	return qmax_parameters;
}

static int mpc7022_update_pre_qmax_parameters(
	struct chip_mpc7022 *chip, enum mpc7022_qmax_parameter_type type,
	int batt_cell_1_qmax, int batt_cell_2_qmax, int batt_cell_qmax_passed_q)
{
	int qmax_parameters = 0;

	switch (type) {
	case MPC7022_CELL_1_QMAX:
		chip->batt_cell_1_qmax_pre = batt_cell_1_qmax;
		qmax_parameters = batt_cell_1_qmax;
		break;
	case MPC7022_CELL_2_QMAX:
		chip->batt_cell_2_qmax_pre = batt_cell_2_qmax;
		qmax_parameters = batt_cell_2_qmax;
		break;
	case MPC7022_CELL_QMAX_PASSED_Q:
		chip->batt_cell_qmax_passed_q_pre = batt_cell_qmax_passed_q;
		qmax_parameters = batt_cell_qmax_passed_q;
		break;
	default:
		chg_err("qmax_parameters type err\n");
		break;
	}

	return qmax_parameters;
}

static int mpc7022_get_qmax_parameters(
	struct chip_mpc7022 *chip, enum mpc7022_qmax_parameter_type type)
{
	int ret;
	int qmax_parameters;
	int batt_cell_1_qmax;
	int batt_cell_2_qmax;
	int batt_cell_qmax_passed_q;
	int retry = RETRY_CNT;
	u8 buf[MPC7022_QMAX_NUM_SIZE] = {0};

	if (!chip)
		return 0;

	if (is_chip_suspended_or_locked(chip)) {
		qmax_parameters = mpc7022_get_pre_qmax_parameters(chip, type);
		return qmax_parameters;
	}

	do {
		ret = mpc7022_read_block(
			chip, MPC7022_REG_IT_STATUS3, buf, MPC7022_QMAX_NUM_SIZE, 0, false, true);
		if (ret) {
			dev_err(chip->dev, "error reading qmax parameters\n");
			return mpc7022_get_pre_qmax_parameters(chip, type);
		}

		batt_cell_1_qmax = (buf[1] << 8) | buf[0];
		batt_cell_2_qmax = (buf[3] << 8) | buf[2];
		batt_cell_qmax_passed_q = (buf[MPC7022_QMAX_NUM_SIZE - 1] << 8) | buf[MPC7022_QMAX_NUM_SIZE - 2];
		if (batt_cell_qmax_passed_q & 0x8000)
			batt_cell_qmax_passed_q = -((~(batt_cell_qmax_passed_q-1)) & 0xFFFF);

		if (normal_range_judge(QMAX_MAX, QMAX_MIN, batt_cell_1_qmax) &&
		    normal_range_judge(QMAX_MAX, QMAX_MIN, batt_cell_2_qmax))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("qmax abnormal, retry:%d, qmax:%d, %d\n", retry, batt_cell_1_qmax, batt_cell_2_qmax);
	} while (retry--);

	if (retry < 0)
		return mpc7022_get_pre_qmax_parameters(chip, type);

	qmax_parameters = mpc7022_update_pre_qmax_parameters(
		chip, type, batt_cell_1_qmax, batt_cell_2_qmax, batt_cell_qmax_passed_q);

	return qmax_parameters;
}

static int mpc7022_get_battery_soc(struct chip_mpc7022 *chip)
{
	int ret;
	int soc = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return 50;

	if (is_chip_suspended_or_locked(chip))
		return chip->soc_pre;

	do {
		ret = mpc7022_read_i2c(chip, chip->cmd_addr.reg_soc, &soc);
		if (ret) {
			dev_err(chip->dev, "error reading soc.\n");
			return chip->soc_pre;
		}
		if (normal_range_judge(SOC_MAX, SOC_MIN, soc))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("soc abnormal, retry:%d, soc:%d\n", retry, soc);
	} while (retry-- > 0);

	if (retry < 0)
		return chip->soc_pre;

	chip->soc_pre = soc;
	return soc;
}

static int mpc7022_get_battery_soc_centi(struct chip_mpc7022 *chip)
{
	int ret;
	int soc_centi = 0;
	int retry = RETRY_CNT;

	if (is_chip_suspended_or_locked(chip))
		return chip->soc_centi_pre;

	do {
		ret = mpc7022_read_i2c(chip, chip->cmd_addr.reg_soc_centi, &soc_centi);
		if (ret) {
			dev_err(chip->dev, "error reading soc_centi.\n");
			return chip->soc_centi_pre;
		}
		if (normal_range_judge(SOC_CENTI_MAX, SOC_CENTI_MIN, soc_centi))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("soc_centi abnormal, retry:%d, soc_centi:%d\n", retry, soc_centi);
	} while (retry-- > 0);

	if (retry < 0)
		return chip->soc_centi_pre;

	chip->soc_centi_pre = soc_centi;
	return soc_centi;
}

static int mpc7022_get_real_current(struct chip_mpc7022 *chip)
{
	int ret;
	int curr = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return 0;

	if (is_chip_suspended_or_locked(chip))
		return chip->current_pre;

	do {
		ret = mpc7022_read_i2c(chip, chip->cmd_addr.reg_ti, &curr);
		if (ret) {
			dev_err(chip->dev, "error reading current.\n");
			return chip->current_pre;
		}

		if (curr & 0x8000)
			curr = ((~(curr - 1)) & 0xFFFF);
		else
			curr = -curr;

		if (normal_range_judge(CURR_MAX, CURR_MIN, curr))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("current abnormal, retry:%d, current:%d\n", retry, curr);
	} while (retry-- > 0);

	if (retry < 0)
		return chip->current_pre;

	chip->current_pre = curr;
	return curr;
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
static bool mpc7022_get_lk_hmac_info(struct mpc7022_hmac_result *ap_result)
{
	return false;
}
#else
static bool mpc7022_get_uefi_hmac_info(struct mpc7022_hmac_result *ap_result)
{
	size_t smem_size;
	void *smem_addr;
	struct mpc7022_hmac_mapping *smem_data;
	int len = ARRAY_SIZE(ap_result->msg);
	u8 *ap_temp = ap_result->msg;

	smem_addr = qcom_smem_get(QCOM_SMEM_HOST_ANY,
				SMEM_RESERVED_BOOT_INFO_FOR_APPS, &smem_size);
	if (IS_ERR(smem_addr)) {
		chg_err("unable to acquire smem SMEM_RESERVED_BOOT_INFO_FOR_APPS entry\n");
		return false;
	}

	smem_data = (struct mpc7022_hmac_mapping *)smem_addr;
	if (smem_data == ERR_PTR(-EPROBE_DEFER)) {
		chg_err("fail to get smem_data\n");
		return false;
	}

	memmove(ap_result, &smem_data->sha256_rst_k0, sizeof(struct mpc7022_hmac_result));

	chg_info("ap_random =[%*ph]\n", len, ap_temp);

	return true;
}
#endif

static bool mpc7022_get_bootloader_hmac_info(
	struct mpc7022_hmac_result *ap_result)
{
	int ret;

#ifdef CONFIG_OPLUS_CHARGER_MTK
	ret = mpc7022_get_lk_hmac_info(ap_result);
#else
	ret = mpc7022_get_uefi_hmac_info(ap_result);
#endif

	return ret;
}

static bool mpc7022_get_ic_hmac_info(struct chip_mpc7022 *chip,
	struct mpc7022_hmac_result *ap_result, struct mpc7022_hmac_result *ic_result)
{
	int count = MPC7022_I2C_TRY_COUNT;
	int ret;
	u8 checksum;
	u8 data_buf[2] = {0};
	int len = ARRAY_SIZE(ic_result->rcv_msg);
	int half_len = len / 2;

	checksum = mpc7022_calc_checksum(ap_result->msg, ARRAY_SIZE(ap_result->msg));

	mutex_lock(&chip->extended_cmd_access);
	ret = mpc7022_write_i2c_block(chip, MPC7022_DATA_FLASH_BLOCK, sizeof(data_buf), data_buf);
	if (ret < 0)
		goto err;

	ret = mpc7022_write_i2c_block(chip, MPC7022_REG_HMAC_DATA_1ST, half_len, ap_result->msg);
	if (ret < 0)
		goto err;

	ret = mpc7022_write_i2c_block(chip, MPC7022_REG_HMAC_DATA_2ND, half_len, &ap_result->msg[half_len]);
	if (ret < 0)
		goto err;

	msleep(5);
	ret = mpc7022_i2c_txsubcmd_onebyte(chip, MPC7022_REG_HMAC_CHECKSUM, checksum);
	if (ret < 0)
		goto err;

	ret = mpc7022_i2c_txsubcmd_onebyte(chip, MPC7022_REG_HMAC_LEN, 0x24);
	if (ret < 0)
		goto err;

	do {
		msleep(20);
		ret = mpc7022_read_i2c_block(chip, MPC7022_REG_HMAC_DATA_1ST, half_len, ic_result->rcv_msg);
	} while (ret < 0 && count-- > 0);

	if (ret < 0)
		goto err;

	ret = mpc7022_read_i2c_block(chip, MPC7022_REG_HMAC_DATA_2ND, half_len, &ic_result->rcv_msg[half_len]);
	if (ret < 0)
		goto err;

	mutex_unlock(&chip->extended_cmd_access);
	return true;

err:
	chg_err("fail\n");
	mutex_unlock(&chip->extended_cmd_access);
	return false;
}

static bool mpc7022_hmac_result_check(
	struct mpc7022_hmac_result *ap_result, struct mpc7022_hmac_result *ic_result)
{
	int len = ARRAY_SIZE(ap_result->rcv_msg);
	u8 *ap_temp = ap_result->rcv_msg;
	u8 *ic_temp = ic_result->rcv_msg;

	chg_info("ap_encode   =[%*ph]\n", len, ap_temp);
	chg_info("gauge_encode=[%*ph]\n", len, ic_temp);

	if (memcmp(ap_temp, ic_temp, len)) {
		chg_err("gauge hmac compare failed\n");
		return false;
	}

	chg_info("gauge hmac succeed\n");

	return true;
}

static bool mpc7022_hmac_interaction(struct chip_mpc7022 *chip)
{
	bool ret;
	struct mpc7022_hmac_result hmac_ap_result = {0};
	struct mpc7022_hmac_result hmac_ic_result = {0};

	ret = mpc7022_get_bootloader_hmac_info(&hmac_ap_result);
	if (!ret) {
		chg_err("get bootloader hmac info failed\n");
		return ret;
	}

	ret = mpc7022_get_ic_hmac_info(chip, &hmac_ap_result, &hmac_ic_result);
	if (!ret) {
		chg_err("get ic hmac info failed\n");
		return ret;
	}

	ret = mpc7022_hmac_result_check(&hmac_ap_result, &hmac_ic_result);

	return ret;
}

static bool mpc7022_get_battery_hmac(struct chip_mpc7022 *chip)
{
	int ret = false;

	if (!chip)
		return true;

	ret = mpc7022_hmac_interaction(chip);

	return ret;
}

static bool mpc7022_get_battery_authenticate(struct chip_mpc7022 *chip)
{
	if (!chip)
		return true;

	if (!chip->temp_pre || chip->get_temp < GET_BATTERY_AUTH_RETRY_COUNT) {
		chip->get_temp++;
		mpc7022_get_battery_temperature(chip);
		msleep(10);
		mpc7022_get_battery_temperature(chip);
	}

	if (chip->temp_pre ==TEMP_MIN)
		return false;
	else
		return true;
}

static int mpc7022_get_sealed_status(struct chip_mpc7022 *chip, bool *sealed)
{
	int ret = 0;
	u8 mode;
	u8 buf[MPC7022_SEAL_NUM_SIZE] = { 0 };

	ret = mpc7022_read_block(
		chip, MPC7022_REG_OPERATION_STATUS, buf, MPC7022_SEAL_NUM_SIZE, 0, false, false);
	if (ret < 0)
		return ret;

	chg_info("operation stauts=[%*ph]\n", MPC7022_SEAL_NUM_SIZE, buf);
	mode = (buf[MPC7022_SEAL_NUM_SIZE - 1] & MPC7022_SEAL_MASK_BIT);
	if (mode == MPC7022_MODE_SEALED) {
		chg_info("is sealed\n");
		*sealed = true;
	} else {
		chg_info("is unsealed\n");
		*sealed = false;
	}

	return 0;
}

static bool mpc7022_set_sealed_status(struct chip_mpc7022 *chip, bool sealed)
{
	int i;
	int ret;
	bool ic_sealed = false;

	ret = mpc7022_get_sealed_status(chip, &ic_sealed);

	if (!ret && (ic_sealed == sealed)) {
		chg_info("sealed status not need set, return\n");
		return true;
	}

	if (sealed) {
		ret = mpc7022_i2c_txsubcmd(chip, MPC7022_DATA_FLASH_BLOCK, MPC7022_SEALED_SUBCMD);
		usleep_range(100000, 100000);
	} else {
		ret = mpc7022_i2c_txsubcmd(chip, MPC7022_DATA_FLASH_BLOCK, MPC7022_UNSEALED_SUBCMD1);
		usleep_range(10000, 10000);
		ret |= mpc7022_i2c_txsubcmd(chip, MPC7022_DATA_FLASH_BLOCK, MPC7022_UNSEALED_SUBCMD2);
		usleep_range(100000, 100000);
	}

	if (ret) {
		chg_err("sealed status set err, return\n");
		return false;
	}

	for (i = 0; i < SEAL_POLLING_RETRY_LIMIT; i++) {
		ret = mpc7022_get_sealed_status(chip, &ic_sealed);
		if (!ret && (ic_sealed == sealed)) {
			chg_info("sealed status set success\n");
			return true;
		}
		usleep_range(10000, 10000);
	}

	chg_err("sealed status set timeout\n");
	return false;
}

static int mpc7022_set_sleep_mode_status(struct chip_mpc7022 *chip, bool enable)
{
	int ret;
	bool ic_enable = 0;
	u8 buf[MPC7022_SLEEP_MODE_NUM_SIZE] = { 0 };

	ret = mpc7022_read_block(chip, MPC7022_REG_DA_CFG, buf,
		MPC7022_SLEEP_MODE_NUM_SIZE, MPC7022_SLEEP_MODE_OFFSET, false, false);
	if (ret < 0)
		return ret;

	if (buf[0] & MPC7022_SLEEP_MODE_MASK_BIT)
		ic_enable = true;
	else
		ic_enable = false;

	if (ic_enable == enable) {
		chg_info("sleep mode status not need set, return\n");
		return 0;
	}

	chg_info("set sleep_mode:%d\n", enable);

	if (enable)
		buf[0] |= MPC7022_SLEEP_MODE_MASK_BIT;
	else
		buf[0] &= ~MPC7022_SLEEP_MODE_MASK_BIT;

	ret = mpc7022_write_block(
		chip, MPC7022_REG_DA_CFG, buf, MPC7022_SLEEP_MODE_NUM_SIZE,
		MPC7022_SLEEP_MODE_OFFSET, false, MPC7022_BLOCK_SIZE, false, true);
	msleep(10);

	return ret;
}

static int mpc7022_update_sleep_mode_status(
	struct chip_mpc7022 *chip, bool enable)
{
	bool rc;
	int ret;
	int try_count = 1;

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	mutex_lock(&chip->extended_cmd_access);
	rc = mpc7022_set_sealed_status(chip, false);
	if (!rc) {
		mutex_unlock(&chip->extended_cmd_access);
		return -EINVAL;
	}

	do {
		ret = mpc7022_set_sleep_mode_status(chip, enable);
	} while (ret < 0 && try_count-- > 0);

	usleep_range(1000, 1000);
	rc = mpc7022_set_sealed_status(chip, true);
	mutex_unlock(&chip->extended_cmd_access);

	if (!rc || ret < 0) {
		chg_err("fail\n");
		return -EINVAL;
	}

	chg_info("success\n");
	return 0;
}

static void mpc7022_set_cmd_addr(struct chip_mpc7022 *chip)
{
	chip->cmd_addr.reg_temp = MPC7022_REG_TEMP;
	chip->cmd_addr.reg_volt = MPC7022_REG_VOLT;
	chip->cmd_addr.reg_flags = MPC7022_REG_FLAGS;
	chip->cmd_addr.reg_ti = MPC7022_REG_TI;
	chip->cmd_addr.reg_rm = MPC7022_REG_RM;
	chip->cmd_addr.reg_fcc = MPC7022_REG_FCC;
	chip->cmd_addr.reg_ai = MPC7022_REG_AI;
	chip->cmd_addr.reg_soc = MPC7022_REG_SOC;
	chip->cmd_addr.reg_soh = MPC7022_REG_SOH;
	chip->cmd_addr.reg_cc = MPC7022_REG_CC;
	chip->cmd_addr.reg_soc_centi = MPC7022_REG_SOC_CENTI;
}

static int mpc7022_get_device_type(struct chip_mpc7022 *chip, int *device_type)
{
	int ret;
	u8 buf[MPC7022_DEVICE_TYPE_NUM_SIZE] = { 0 };

	ret = mpc7022_read_block(
		chip, MPC7022_REG_DEVICE_TYPE, buf, MPC7022_DEVICE_TYPE_NUM_SIZE, 0, false, true);
	if (ret < 0)
		return ret;

	*device_type = (buf[1] << 0x8) | buf[0];

	return 0;
}

static void mpc7022_hw_config(struct chip_mpc7022 *chip)
{
	int device_type = 0;

	mpc7022_set_cmd_addr(chip);
	mpc7022_get_device_type(chip, &device_type);

	chg_info("device type is 0x%02x\n", device_type);
}

static void mpc7022_parse_dt(struct chip_mpc7022 *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;

	rc = of_property_read_u32(node, "oplus,batt_num", &chip->batt_num);
	if (rc < 0) {
		chg_err("can't get oplus,batt_num, rc=%d\n", rc);
		chip->batt_num = 2;
	}

	rc = of_property_read_u32(node, "qcom,gauge_num", &chip->gauge_num);
	if (rc) {
		chip->gauge_num = 0;
	}

	chip->i2c_rst_ext = of_property_read_bool(node, "oplus,i2c_rst_ext");
}

static int mpc7022_set_term_volt(struct chip_mpc7022 *chip, int volt_mv)
{
	int ret;
	u8 effect_buf[MPC7022_TERM_VOLT_NUM_SIZE] = { 0 };
	u8 save_buf[MPC7022_TERM_VOLT_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	effect_buf[0] = volt_mv & 0xff;
	effect_buf[1] = (volt_mv >> 8) & 0xff;
	save_buf[0] = volt_mv & 0xff;
	save_buf[1] = (volt_mv >> 8) & 0xff;

	ret = mpc7022_write_block(
		chip, MPC7022_REG_TERM_VOLT, effect_buf, MPC7022_TERM_VOLT_NUM_SIZE,
		MPC7022_TERM_VOLT_OFFSET, false, MPC7022_TERM_VOLT_BLOCK_SIZE, true, true);
	if (ret < 0)
		return ret;

	ret = mpc7022_write_block(
		  chip, MPC7022_REG_OPLUS_DATA, save_buf, MPC7022_DEEP_VOLT_NUM_SIZE,
		  MPC7022_DEEP_VOLT_OFFSET, true, MPC7022_BLOCK_SIZE, true, true);
	if (ret < 0)
		return ret;

	return 0;
}

static int mpc7022_get_deep_term_volt(struct chip_mpc7022 *chip, int *volt)
{
	int ret = 	-EINVAL;
	u8 buf[MPC7022_DEEP_VOLT_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip)) {
		*volt = chip->deep_term_volt_pre;
		return ret;
	}

	ret = mpc7022_read_block(
		chip, MPC7022_REG_OPLUS_DATA, buf, MPC7022_DEEP_VOLT_NUM_SIZE,
		MPC7022_DEEP_VOLT_OFFSET, true, true);
	if (ret < 0) {
		*volt = chip->deep_term_volt_pre;
		return ret;
	}

	*volt = (buf[1] << 0x8) + buf[0];
	chip->deep_term_volt_pre = *volt;

	return ret;
}

static int mpc7022_get_deep_count(struct chip_mpc7022 *chip)
{
	int ret;
	int count;
	u8 buf[MPC7022_DEEP_COUNT_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return chip->deep_count_pre;

	ret = mpc7022_read_block(chip, MPC7022_REG_OPLUS_DATA, buf,
		MPC7022_DEEP_COUNT_NUM_SIZE, MPC7022_DEEP_COUNT_OFFSET, true, true);
	if (ret < 0)
		return chip->deep_count_pre;

	count = (buf[1] << 0x8) + buf[0];
	chip->deep_count_pre = count;

	return count;
}

static int mpc7022_set_deep_count(struct chip_mpc7022 *chip, int count)
{
	int ret;
	u8 buf[MPC7022_DEEP_COUNT_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	buf[0] = count & 0xff;
	buf[1] = (count >> 8) & 0xff;

	ret = mpc7022_write_block(
		chip, MPC7022_REG_OPLUS_DATA, buf, MPC7022_DEEP_COUNT_NUM_SIZE,
		MPC7022_DEEP_COUNT_OFFSET, true, MPC7022_BLOCK_SIZE, true, true);
	if (ret < 0)
		return ret;

	return 0;
}

static int mpc7022_set_deep_last_cc(struct chip_mpc7022 *chip, int cc)
{
	int ret;
	u8 buf[MPC7022_DEEP_LAST_CC_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	buf[0] = cc & 0xff;
	buf[1] = (cc >> 8) & 0xff;

	ret = mpc7022_write_block(
		chip, MPC7022_REG_OPLUS_DATA, buf, MPC7022_DEEP_LAST_CC_NUM_SIZE,
		MPC7022_DEEP_LAST_CC_OFFSET, true, MPC7022_BLOCK_SIZE, true, true);
	if (ret < 0)
		return ret;

	return 0;
}

static int mpc7022_get_deep_last_cc(struct chip_mpc7022 *chip)
{
	int ret;
	int cc;
	u8 buf[MPC7022_DEEP_LAST_CC_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return chip->deep_last_cc_pre;

	ret = mpc7022_read_block(chip, MPC7022_REG_OPLUS_DATA, buf,
		MPC7022_DEEP_LAST_CC_NUM_SIZE, MPC7022_DEEP_LAST_CC_OFFSET, true, true);
	if (ret < 0)
		return chip->deep_last_cc_pre;

	cc = (buf[1] << 0x8) + buf[0];
	chip->deep_last_cc_pre = cc;

	return cc;
}

static int oplus_mpc7022_get_deep_count(
	struct oplus_chg_ic_dev *ic_dev, int *count)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || count == NULL) {
		chg_err("oplus_chg_ic_dev or count is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*count = mpc7022_get_deep_count(chip);

	return 0;
}

static int oplus_mpc7022_set_deep_count(
	struct oplus_chg_ic_dev *ic_dev, int count)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	mpc7022_set_deep_count(chip, count);

	return 0;
}

static int oplus_mpc7022_set_deep_term_volt(
	struct oplus_chg_ic_dev *ic_dev, int volt_mv)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	mpc7022_set_term_volt(chip, volt_mv);

	return 0;
}

static int oplus_mpc7022_get_deep_term_volt(
	struct oplus_chg_ic_dev *ic_dev, int *volt)
{
	int ret;
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || volt == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ret = mpc7022_get_deep_term_volt(chip, volt);

	return ret;
}

static int oplus_mpc7022_set_deep_last_cc(
	struct oplus_chg_ic_dev *ic_dev, int cc)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	mpc7022_set_deep_last_cc(chip, cc);

	return 0;
}

static int oplus_mpc7022_get_deep_last_cc(
	struct oplus_chg_ic_dev *ic_dev, int *cc)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || cc == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*cc = mpc7022_get_deep_last_cc(chip);

	return 0;
}

static int mpc7022_get_info(struct chip_mpc7022 *chip, u8 *info, int len)
{
	int i;
	int j;
	int ret;
	int data;
	int index = 0;
	u8 buf[MPC7022_BLOCK_SIZE] = {0};
	struct mpc7022_block_access standard[] = {
		{ chip->cmd_addr.reg_temp, 2 }, { chip->cmd_addr.reg_volt, 2 },
		{ chip->cmd_addr.reg_flags, 2 }, { chip->cmd_addr.reg_ti, 2 },
		{ chip->cmd_addr.reg_rm, 2 }, { chip->cmd_addr.reg_fcc, 2 },
		{ chip->cmd_addr.reg_ai, 2 }, { chip->cmd_addr.reg_cc, 2 },
		{ chip->cmd_addr.reg_soc, 2 }, { chip->cmd_addr.reg_soh, 2 },
	};

	struct mpc7022_block_access extend[] = {
		{ MPC7022_REG_CHEMID, 2, 0, 1},
		{ MPC7022_REG_OPERATION_STATUS, 4, 0, 3},
		{ MPC7022_REG_DA_STATUS1, 4, 0, 3 },
		{ MPC7022_REG_IT_STATUS1, 14, 0, 13 },
		{ MPC7022_REG_IT_STATUS2, 24, 10, 21 },
		{ MPC7022_REG_IT_STATUS3, 12, 0, 11 },
		{ MPC7022_REG_CB_STATUS, 8, 0, 7 },
		{ MPC7022_REG_DOD0, 20, 8, 19 },
		{ MPC7022_REG_SIMULATE_BLOCK, 24, 20, 23 },
		{ MPC7022_REG_SIMULATE_CURR, 4, 2, 3 },
	};

	/*standard register packaging*/
	for (i = 0; i < ARRAY_SIZE(standard); i++) {
		ret = mpc7022_read_i2c(chip, standard[i].addr, &data);
		if (ret < 0)
			continue;
		index += scnprintf(info + index, len - index,
			  "0x%02x=%02x,%02x|", standard[i].addr, (data & 0xff), ((data >> 8) & 0xff));
	}

	/*extended register packaging*/
	for (i = 0; i < ARRAY_SIZE(extend); i++) {
		ret = mpc7022_read_block(chip, extend[i].addr, buf, extend[i].len, 0, false, true);
		if (ret < 0)
			continue;

		index += scnprintf(info + index, len - index, "0x%04x=", extend[i].addr);
		for (j = extend[i].start_index; j < extend[i].end_index; j++)
			index += scnprintf(info + index, len - index, "%02x,", buf[j]);
		index += scnprintf(info + index, len - index, "%02x", buf[j]);

		if (i < ARRAY_SIZE(extend) - 1) {
			index += scnprintf(info + index, len - index, "|");
			usleep_range(500, 500);
		}
	}

	return index;
}

static int mpc7022_get_lifetime_info(struct chip_mpc7022 *chip, u8 *info, int len)
{
	int i;
	int j;
	int ret;
	int index = 0;
	u8 buf[MPC7022_BLOCK_SIZE] = {0};
	struct mpc7022_block_access extend[] = {
		{ MPC7022_REG_STATIC_DF_SIG, 2, 0, 1},
		{ MPC7022_REG_STATIC_CHEM_DF_SIG, 2, 0, 1 },
		{ MPC7022_REG_SILI_LOSS_EXPANSION, 10, 0, 9 },
		{ MPC7022_REG_LIFETIME_ADDR, 10, 0, 9 },
		{ MPC7022_REG_OPLUS_DATA, 9, 0, 8 },
 		{ MPC7022_REG_CALIB_REASON_ADDR, 12, 0, 11 },
		{ MPC7022_REG_CIS_1, 20, 0, 19 },
		{ MPC7022_REG_CIS_2, 32, 24, 31 },
		{ MPC7022_REG_EIS_SOH, 8, 0, 7 },
		{ MPC7022_REG_IMP_VOLT_TABLT0, 32, 0, 31 },
		{ MPC7022_REG_IMP_VOLT_TABLT1, 32, 0, 31 },
		{ MPC7022_REG_IMP_VOLT_TABLT2, 32, 0, 31 },
		{ MPC7022_REG_IMP_VOLT_TABLT3, 32, 0, 31 },
	};

	for (i = 0; i < ARRAY_SIZE(extend); i++) {
		ret = mpc7022_read_block(chip, extend[i].addr, buf, extend[i].len, 0, false, true);
		if (ret < 0)
			continue;

		index += scnprintf(info + index, len - index, "0x%04x=", extend[i].addr);
		for (j = extend[i].start_index; j < extend[i].end_index; j++)
			index += scnprintf(info + index, len - index, "%02x,", buf[j]);
		index += scnprintf(info + index, len - index, "%02x", buf[j]);

		if (i < ARRAY_SIZE(extend) - 1) {
			index += scnprintf(info + index, len - index, "|");
			usleep_range(500, 500);
		}
	}

	return index;
}

static int oplus_mpc7022_get_reg_info(
	struct oplus_chg_ic_dev *ic_dev, u8 *info, int len)
{
	int index;
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || !info) {
		chg_err("oplus_chg_ic_dev or info is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	index = mpc7022_get_info(chip, info, len);

	return index;
}

static void mpc7022_update_dod_qmax_time(struct chip_mpc7022 *chip, const int *check_args)
{
	if (check_args[0] != chip->calib_check_args_pre[0] || check_args[3] != chip->calib_check_args_pre[3])
		chip->dod_time = 1; /* init as one day */
	else
		chip->dod_time++;

	if (check_args[1] != chip->calib_check_args_pre[1] || check_args[2] != chip->calib_check_args_pre[2] ||
	    check_args[4] != chip->calib_check_args_pre[4] || check_args[5] != chip->calib_check_args_pre[5])
		chip->qmax_time = 1; /* init as one day */
	else
		chip->qmax_time++;
}

static int mpc7022_get_calib_time(
	struct chip_mpc7022 *chip, int *dod_calib_time, int *qmax_calib_time)
{
	int i;
	int ret;
	u8 buf[MPC7022_BLOCK_SIZE] = {0};
	int check_args[CALIB_TIME_CHECK_ARGS] = {0};
	struct mpc7022_block_access extend[] = {
		{ MPC7022_REG_IT_STATUS2, 14 },
		{ MPC7022_REG_IT_STATUS3, 8 },
	};

	for (i = 0; i < ARRAY_SIZE(extend); i++) {
		ret = mpc7022_read_block(chip, extend[i].addr, buf, extend[i].len, 0, false, true);
		if (ret < 0)
			return ret;

		if (extend[i].addr == MPC7022_REG_IT_STATUS2) {
			check_args[0] = (buf[11] << 0x08) | buf[10];
			check_args[3] = (buf[13] << 0x08) | buf[12];
		} else {
			check_args[1] = (buf[1] << 0x08) | buf[0];
			check_args[2] = (buf[5] << 0x08) | buf[4];
			check_args[4] = (buf[3] << 0x08) | buf[2];
			check_args[5] = (buf[7] << 0x08) | buf[6];
		}
		if (i < ARRAY_SIZE(extend) - 1)
			usleep_range(500, 500);
	}

	mpc7022_update_dod_qmax_time(chip, check_args);
	memmove(chip->calib_check_args_pre, check_args, sizeof(check_args));
	*dod_calib_time = chip->dod_time;
	*qmax_calib_time = chip->qmax_time;

	return ret;
}

static void oplus_mpc7022_calib_args_to_check_args(
	struct chip_mpc7022 *chip, char *calib_args, int len)
{
	int i;
	int j;

	if (len != (CALIB_TIME_CHECK_ARGS * 2)) {
		chg_err("len not match\n");
		return;
	}

	for (i = 0, j = 0; i < CALIB_TIME_CHECK_ARGS; i++, j += 2) {
		chip->calib_check_args_pre[i] = (calib_args[j + 1] << 0x8) + calib_args[j];
		chg_debug("calib_check_args_pre[%d]=0x%04x\n", i, chip->calib_check_args_pre[i]);
	}
}

static void oplus_mpc7022_check_args_to_calib_args(
	struct chip_mpc7022 *chip, char *calib_args, int len)
{
	int i;
	int j;

	if (len != (CALIB_TIME_CHECK_ARGS * 2)) {
		chg_err("len not match\n");
		return;
	}

	for (i = 0, j = 0; i < CALIB_TIME_CHECK_ARGS; i++, j += 2) {
		calib_args[j] = chip->calib_check_args_pre[i] & 0xff;
		calib_args[j + 1] = (chip->calib_check_args_pre[i] >> 0x8) & 0xff;
		chg_debug("calib_args[%d]=0x%02x, 0x%02x\n", i, calib_args[j], calib_args[j + 1]);
	}
}

static int oplus_mpc7022_get_calib_time(struct oplus_chg_ic_dev *ic_dev,
	int *dod_calib_time, int *qmax_calib_time, char *calib_args, int len)
{
	int ret;
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	if (calib_args == NULL || dod_calib_time== NULL || qmax_calib_time == NULL || !len)
		return -EINVAL;

	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip->calib_info_init) {
		*dod_calib_time = -1;
		*qmax_calib_time = -1;
		return 0;
	}

	if (is_chip_suspended_or_locked(chip)) {
		*dod_calib_time = chip->dod_time_pre;
		*qmax_calib_time = chip->qmax_time_pre;
		return 0;
	}

	mutex_lock(&chip->calib_time_mutex);
	ret = mpc7022_get_calib_time(chip, dod_calib_time, qmax_calib_time);
	if (ret < 0) {
		*dod_calib_time = chip->dod_time_pre;
		*qmax_calib_time = chip->qmax_time_pre;
		mutex_unlock(&chip->calib_time_mutex);
		return 0;
	}

	oplus_mpc7022_check_args_to_calib_args(chip, calib_args, len);
	chip->dod_time_pre = *dod_calib_time;
	chip->qmax_time_pre = *qmax_calib_time;
	mutex_unlock(&chip->calib_time_mutex);

	return 0;
}

static int oplus_mpc7022_set_calib_time(struct oplus_chg_ic_dev *ic_dev,
	int dod_calib_time, int qmax_calib_time, char *calib_args, int len)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL)
		return -ENODEV;

	if (calib_args == NULL || !len)
		return -EINVAL;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (len != (CALIB_TIME_CHECK_ARGS * 2)) {
		chg_err("len not match\n");
		return -EINVAL;
	}

	if (dod_calib_time) {
		chip->dod_time_pre = dod_calib_time;
		chip->qmax_time_pre = qmax_calib_time;
	} else {
		chip->dod_time_pre = 1;
		chip->qmax_time_pre = 1;
	}

	chip->dod_time = dod_calib_time;
	chip->qmax_time = qmax_calib_time;
	oplus_mpc7022_calib_args_to_check_args(chip, calib_args, len);
	chip->calib_info_init = true;

	return 0;
}

static void mpc7022_reset(struct i2c_client *client)
{
	struct chip_mpc7022 *chip;

	chip = dev_get_drvdata(&client->dev);

	if (chip)
		mpc7022_shutdown_set_cuv_state(chip);
}

static int mpc7022_pm_resume(struct device *dev)
{
	struct chip_mpc7022 *chip;

	chip = dev_get_drvdata(dev);
	if (!chip)
		return 0;

	atomic_set(&chip->suspended, 0);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_RESUME);
	return 0;
}

static int mpc7022_pm_suspend(struct device *dev)
{
	struct chip_mpc7022 *chip;

	chip = dev_get_drvdata(dev);
	if (!chip)
		return 0;

	atomic_set(&chip->suspended, 1);
	return 0;
}

static const struct dev_pm_ops mpc7022_pm_ops = {
	.resume = mpc7022_pm_resume,
	.suspend = mpc7022_pm_suspend,
};

static int oplus_mpc7022_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL\n");
		return -ENODEV;
	}

	ic_dev->online = true;

	return 0;
}

static int oplus_mpc7022_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;

	return 0;
}

static int oplus_mpc7022_get_batt_vol(
	struct oplus_chg_ic_dev *ic_dev, int index, int *volt)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || volt == NULL) {
		chg_err("oplus_chg_ic_dev or volt is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (index) {
	case 0:
		*volt = mpc7022_get_batt_volt(chip, MPC7022_CELL_1_VOLT);
		break;
	case 1:
		*volt = mpc7022_get_batt_volt(chip, MPC7022_CELL_2_VOLT);
		break;
	default:
		chg_err("index(=%d) over size\n", index);
		return -EINVAL;
	}

	return 0;
}

static int oplus_mpc7022_get_batt_max(
	struct oplus_chg_ic_dev *ic_dev, int *volt)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || volt == NULL) {
		chg_err("oplus_chg_ic_dev or volt is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*volt = mpc7022_get_batt_volt(chip, MPC7022_CELL_MAX_VOLT);

	return 0;
}

static int oplus_mpc7022_get_batt_min(
	struct oplus_chg_ic_dev *ic_dev, int *volt)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || volt == NULL) {
		chg_err("oplus_chg_ic_dev or volt is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*volt = mpc7022_get_batt_volt(chip, MPC7022_CELL_MIN_VOLT);

	return 0;
}

static int oplus_mpc7022_get_batt_curr(
	struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || curr_ma == NULL) {
		chg_err("oplus_chg_ic_dev or curr_ma is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*curr_ma = mpc7022_get_real_current(chip);

	return 0;
}

static int oplus_mpc7022_get_batt_temp(
	struct oplus_chg_ic_dev *ic_dev, int *temp)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || temp == NULL) {
		chg_err("oplus_chg_ic_dev or temp is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip)
		*temp = mpc7022_get_battery_temperature(chip);

	return 0;
}

static int oplus_mpc7022_get_batt_soc(
	struct oplus_chg_ic_dev *ic_dev, int *soc)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || soc == NULL) {
		chg_err("oplus_chg_ic_dev or soc is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*soc = mpc7022_get_battery_soc(chip);

	return 0;
}

static int oplus_mpc7022_get_batt_soc_centi(struct oplus_chg_ic_dev *ic_dev, int *soc_centi)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || soc_centi == NULL) {
		chg_err("oplus_chg_ic_dev or soc_centi is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*soc_centi = mpc7022_get_battery_soc_centi(chip);

	return 0;
}

static int oplus_mpc7022_get_batt_fcc(
	struct oplus_chg_ic_dev *ic_dev, int *fcc)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || fcc == NULL) {
		chg_err("oplus_chg_ic_dev or fcc is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*fcc = mpc7022_get_battery_fcc(chip);

	return 0;
}

static int oplus_mpc7022_get_batt_cc(
	struct oplus_chg_ic_dev *ic_dev, int *cc)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || cc == NULL) {
		chg_err("oplus_chg_ic_dev or cc is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*cc = mpc7022_get_battery_cc(chip);

	return 0;
}

static int oplus_mpc7022_get_batt_rm(
	struct oplus_chg_ic_dev *ic_dev, int *rm)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || rm == NULL) {
		chg_err("oplus_chg_ic_dev or rm is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*rm = mpc7022_get_batt_remaining_capacity(chip);

	return 0;
}

static int oplus_mpc7022_get_batt_soh(
	struct oplus_chg_ic_dev *ic_dev, int *soh)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || soh == NULL) {
		chg_err("oplus_chg_ic_dev or soh is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*soh = mpc7022_get_battery_soh(chip);

	return 0;
}

static int oplus_mpc7022_get_batt_auth(
	struct oplus_chg_ic_dev *ic_dev, bool *pass)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || pass == NULL) {
		chg_err("oplus_chg_ic_dev or pass is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*pass = mpc7022_get_battery_authenticate(chip);

	return 0;
}

static int oplus_mpc7022_get_batt_hmac(
	struct oplus_chg_ic_dev *ic_dev, bool *pass)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || pass == NULL) {
		chg_err("oplus_chg_ic_dev or pass is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*pass = mpc7022_get_battery_hmac(chip);

	return 0;
}

static int oplus_mpc7022_update_soc_smooth_parameter(
	struct oplus_chg_ic_dev *ic_dev)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	return mpc7022_update_sleep_mode_status(chip, true);
}

static int oplus_mpc7022_get_device_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	if (ic_dev == NULL || type == NULL) {
		chg_err("oplus_chg_ic_dev or type is NULL\n");
		return -ENODEV;
	}

	*type = DEVICE_MPC7022;

	return 0;
}

static int oplus_mpc7022_set_lock(
	struct oplus_chg_ic_dev *ic_dev, bool lock)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	atomic_set(&chip->locked, lock ? 1 : 0);

	return 0;
}

static int oplus_mpc7022_is_locked(
	struct oplus_chg_ic_dev *ic_dev, bool *locked)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*locked = !!atomic_read(&chip->locked);

	return 0;
}

static int oplus_mpc7022_get_batt_num(
	struct oplus_chg_ic_dev *ic_dev, int *num)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || num == NULL) {
		chg_err("oplus_chg_ic_dev or num is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*num = chip->batt_num;

	return 0;
}

static int oplus_mpc7022_get_battery_gauge_type_for_bcc(
	struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || type == NULL) {
		chg_err("oplus_chg_ic_dev or type is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is null\n");
		return -ENODEV;
	}

	*type = DEVICE_MPC7022;

	return 0;
}

static int oplus_mpc7022_get_battery_dod0(
	struct oplus_chg_ic_dev *ic_dev, int index, int *dod0)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || dod0 == NULL) {
		chg_err("oplus_chg_ic_dev or dod0 is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (index) {
	case 0:
		*dod0 = mpc7022_get_dod0(chip, MPC7022_CELL_1_DOD0);
		break;
	case 1:
		*dod0 = mpc7022_get_dod0(chip, MPC7022_CELL_2_DOD0);
		break;
	default:
		chg_err("index(=%d), over size\n", index);
		return -EINVAL;
	}

	return 0;
}

static int oplus_mpc7022_get_battery_dod0_passed_q(
	struct oplus_chg_ic_dev *ic_dev, int *dod_passed_q)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || dod_passed_q == NULL) {
		chg_err("oplus_chg_ic_dev or dod_passed_q is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*dod_passed_q = mpc7022_get_dod_passed_q(chip);

	return 0;
}

static int oplus_mpc7022_get_battery_qmax(
	struct oplus_chg_ic_dev *ic_dev, int index, int *qmax)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || qmax == NULL) {
		chg_err("oplus_chg_ic_dev or qmax is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (index) {
	case 0:
		*qmax = mpc7022_get_qmax_parameters(chip, MPC7022_CELL_1_QMAX);
		break;
	case 1:
		*qmax = mpc7022_get_qmax_parameters(chip, MPC7022_CELL_2_QMAX);
		break;
	default:
		chg_err("index(=%d), over size\n", index);
		return -EINVAL;
	}

	return 0;
}

static int oplus_mpc7022_get_battery_qmax_passed_q(
	struct oplus_chg_ic_dev *ic_dev, int *qmax_passed_q)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || qmax_passed_q == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*qmax_passed_q = mpc7022_get_qmax_parameters(chip, MPC7022_CELL_QMAX_PASSED_Q);

	return 0;
}

static int oplus_mpc7022_is_suspend(
	struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || suspend == NULL) {
		chg_err("oplus_chg_ic_dev or suspend is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*suspend = atomic_read(&chip->suspended);

	return 0;
}

static int oplus_mpc7022_get_batt_exist(
	struct oplus_chg_ic_dev *ic_dev, bool *exist)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || exist == NULL) {
		chg_err("oplus_chg_ic_dev or exist is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (atomic_read(&chip->i2c_err_count) > I2C_ERR_MAX)
		*exist = false;
	else
		*exist = true;

	return 0;
}

static int mpc7022_get_batt_sn(struct chip_mpc7022 *chip)
{
	int ret;
	u8 buf[MPC7022_BATT_SERIAL_NUM_SIZE] = {0};

	if (is_chip_suspended_or_locked(chip))
		return 0;

	if (OPLUS_BATT_SERIAL_NUM_SIZE <= MPC7022_BATT_SERIAL_NUM_SIZE) {
		chg_err("sn insufficient container storage length\n");
		return ret;
	}

	ret = mpc7022_read_block(
		chip, MPC7022_REG_MANUFACTURER_NAME, buf, MPC7022_BATT_SERIAL_NUM_SIZE, 0, false, true);
	if (ret < 0) {
		chg_err("get sn failed\n");
		return ret;
	}

	memmove(chip->battinfo.batt_serial_num, buf, MPC7022_BATT_SERIAL_NUM_SIZE);

	return 0;
}

static int oplus_mpc7022_get_batt_sn(
	struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	int bsnlen;
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !buf || len < OPLUS_BATT_SERIAL_NUM_SIZE)
		return -EINVAL;

	if (!strlen(chip->battinfo.batt_serial_num))
		mpc7022_get_batt_sn(chip);

	chg_info("batt_sn(%s):%s\n", ic_dev->name, chip->battinfo.batt_serial_num);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	strscpy = strlcpy(buf, chip->battinfo.batt_serial_num, OPLUS_BATT_SERIAL_NUM_SIZE);
#else
	bsnlen = strlcpy(buf, chip->battinfo.batt_serial_num, OPLUS_BATT_SERIAL_NUM_SIZE);
#endif

	return bsnlen;
}

static int mpc7022_set_first_usage_date(struct chip_mpc7022 *chip, u32 data)
{
	int ret;
	u16 first_usage_date = (data >> 8) & 0xffff;
	u8 check_sum = data & 0xff;
	u8 calc_check_sum;
	u8 buf[MPC7022_FIRST_USAGE_DATA_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	buf[0] = first_usage_date & 0xff;
	buf[1] = (first_usage_date >> 8) & 0xff;
	calc_check_sum = (0xff - buf[0] - buf[1]) & 0xff;

	if (calc_check_sum != check_sum) {
		chg_err("check_sum err\n");
		return -EINVAL;
	}

	ret = mpc7022_write_block(
		chip, MPC7022_REG_OPLUS_DATA, buf, MPC7022_FIRST_USAGE_DATA_NUM_SIZE,
		MPC7022_FIRST_USAGE_DATA_OFFSET, true, MPC7022_BLOCK_SIZE, true, true);

	return ret;
}

static int oplus_mpc7022_set_first_usage_date(struct oplus_chg_ic_dev *ic_dev, const char *buf)
{
	u32 data;
	u16 date;
	u8 check_sum;
	int year = 0;
	int month = 0;
	int day = 0;
	int rc;
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !buf)
		return -EINVAL;

	sscanf(buf, "%d-%d-%d", &year, &month, &day);
	date = (((year - 1980) & 0x7f) << 9) | ((month & 0xf) << 5) | (day & 0x1f);
	check_sum = 0xff - ((date >> 8) & 0xff) - (date & 0xff);
	data = date << 8 | check_sum;
	chg_info("%d-%d-%d, date=0x%04x, data=0x%08x\n", year, month, day, date, data);

	rc = mpc7022_set_first_usage_date(chip, data);
	if (rc < 0) {
		chg_err("set firset usage date fail rc = %d\n", rc);
		return rc;
	}
	chip->battinfo.first_usage_date = date;

	return 0;
}

static int oplus_mpc7022_get_first_usage_date(struct oplus_chg_ic_dev *ic_dev, char *buf, int len)
{
	struct chip_mpc7022 *chip;
	int date_len;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !buf || len < OPLUS_BATTINFO_DATE_SIZE)
		return -EINVAL;

	date_len = scnprintf(buf, len, "%d-%02d-%02d", (((chip->battinfo.first_usage_date >> 9) & 0x7f) + 1980),
		(chip->battinfo.first_usage_date >> 5) & 0xf, chip->battinfo.first_usage_date & 0x1f);

	return date_len;
}

static int mpc7022_set_ui_cycle_count(struct chip_mpc7022 *chip, u32 data)
{
	int ret;
	u16 cycle_count = (data >> 8) & 0xffff;
	u8 check_sum = data & 0xff;
	u8 calc_check_sum = 0x00;
	u8 buf[MPC7022_UI_CYCLE_COUNT_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	buf[0] = cycle_count & 0xff;
	buf[1] = (cycle_count >> 8) & 0xff;
	calc_check_sum = (0xff - buf[0] - buf[1]) & 0xff;

	if (calc_check_sum != check_sum) {
		chg_err("check_sum err\n");
		return -EINVAL;
	}

	ret = mpc7022_write_block(
		chip, MPC7022_REG_OPLUS_DATA, buf, MPC7022_UI_CYCLE_COUNT_NUM_SIZE,
		MPC7022_UI_CYCLE_COUNT_OFFSET, true, MPC7022_BLOCK_SIZE, true, true);

	return ret;
}

static int oplus_mpc7022_set_ui_cycle_count(struct oplus_chg_ic_dev *ic_dev, u16 ui_cycle_count)
{
	struct chip_mpc7022 *chip;
	u8 check_sum;
	u32 data;
	int ret;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip)
		return -EINVAL;

	check_sum = 0xff - ((ui_cycle_count >> 8) & 0xff) - (ui_cycle_count & 0xff);
	data = ui_cycle_count << 8 | check_sum;
	ret = mpc7022_set_ui_cycle_count(chip, data);
	if (ret < 0) {
		chg_err("set ui cycle count %u failed\n", ui_cycle_count);
		return ret;
	}
	chip->battinfo.ui_cycle_count = ui_cycle_count;
	chg_info("set ui cycle count %u success\n", ui_cycle_count);

	return 0;
}

static int oplus_mpc7022_get_ui_cycle_count(
	struct oplus_chg_ic_dev *ic_dev, u16 *ui_cycle_count)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !ui_cycle_count)
		return -EINVAL;

	chg_info("batt_ui_cycle_count:%d\n", chip->battinfo.ui_cycle_count);
	*ui_cycle_count = chip->battinfo.ui_cycle_count;

	return 0;
}

static int mpc7022_set_ui_soh(struct chip_mpc7022 *chip, u32 data)
{
	int ret;
	u8 soh = (data >> 8) & 0xff;
	u8 check_sum = data & 0xff;
	u8 calc_check_sum;
	u8 buf[MPC7022_UISOH_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	buf[0] = soh;
	calc_check_sum = (0xff - buf[0]) & 0xff;

	if (calc_check_sum != check_sum) {
		chg_err("check_sum err\n");
		return -EINVAL;
	}

	ret = mpc7022_write_block(
		chip, MPC7022_REG_OPLUS_DATA, buf, MPC7022_UISOH_NUM_SIZE,
		MPC7022_UISOH_OFFSET, true, MPC7022_BLOCK_SIZE, true, true);

	return ret;
}

static int oplus_mpc7022_set_ui_soh(struct oplus_chg_ic_dev *ic_dev, u8 ui_soh)
{
	struct chip_mpc7022 *chip;
	u8 check_sum;
	u32 data;
	int ret;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip)
		return -EINVAL;

	check_sum = 0xff - (ui_soh & 0xff);
	data = ui_soh << 8 | check_sum;
	ret = mpc7022_set_ui_soh(chip, data);
	if (ret < 0) {
		chg_err("set batt ui soh %u failed\n", ui_soh);
		return ret;
	}
	chip->battinfo.ui_soh = data;
	chg_info("set batt ui soh %u success\n", ui_soh);

	return 0;
}

static int oplus_mpc7022_get_ui_soh(struct oplus_chg_ic_dev *ic_dev, u8 *ui_soh)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !ui_soh)
		return -EINVAL;

	chg_info("batt_ui_soh:%d\n", chip->battinfo.ui_soh);
	*ui_soh = chip->battinfo.ui_soh;

	return 0;
}

static int oplus_mpc7022_get_used_flag(struct oplus_chg_ic_dev *ic_dev, u8 *used_flag)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !used_flag)
		return -EINVAL;

	*used_flag = chip->battinfo.used_flag;
	chg_info("get used_flag %u\n", *used_flag);

	return 0;
}

static int mpc7022_set_used_flag(struct chip_mpc7022 *chip, u32 data)
{
	int ret;
	u8 flag = (data >> 8) & 0xff;
	u8 check_sum = data & 0xff;
	u8 calc_check_sum;
	u8 buf[MPC7022_USED_FLAG_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	buf[0] = flag;
	calc_check_sum = (0xff - buf[0]) & 0xff;

	if (calc_check_sum != check_sum) {
		chg_err("check_sum err\n");
		return -EINVAL;
	}

	ret = mpc7022_write_block(
		chip, MPC7022_REG_OPLUS_DATA, buf, MPC7022_USED_FLAG_NUM_SIZE,
		MPC7022_USED_FLAG_OFFSET, true, MPC7022_BLOCK_SIZE, true, true);

	return ret;
}

static int oplus_mpc7022_set_used_flag(struct oplus_chg_ic_dev *ic_dev, u8 used_flag)
{
	struct chip_mpc7022 *chip;
	u8 check_sum;
	u32 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip)
		return -EINVAL;

	check_sum = 0xff - (used_flag & 0xff);
	data = used_flag << 8 | check_sum;
	rc = mpc7022_set_used_flag(chip, data);
	if (rc < 0) {
		chg_err("set batt used flag failed\n");
		return rc;
	}
	chip->battinfo.used_flag = used_flag;
	chg_info("set batt used flag %u success\n", used_flag);

	return 0;
}

static int mpc7022_get_manu_date(struct chip_mpc7022 *chip)
{
	int ret;
	u8 buf[MPC7022_BATT_MANUDATE_NUM_SIZE] = {0};

	ret = mpc7022_read_block(
		chip, MPC7022_REG_BATT_MANUDATE, buf, MPC7022_BATT_MANUDATE_NUM_SIZE, 0, false, true);
	if (ret < 0) {
		chg_err("get manu date failed\n");
		return ret;
	}

	chip->battinfo.manu_date = buf[0] | (buf[1] << 8);
	return ret;
}

static int oplus_mpc7022_get_manu_date(struct oplus_chg_ic_dev *ic_dev, char *buf, int len)
{
	struct chip_mpc7022 *chip;
	int date_len;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip || !buf || len <  OPLUS_BATTINFO_DATE_SIZE)
		return -EINVAL;

	chg_info("batt_manu_date:0x%04x\n", chip->battinfo.manu_date);
	date_len = scnprintf(buf, len, "%d-%02d-%02d", (((chip->battinfo.manu_date >> 9) & 0x7F) + 1980),
		(chip->battinfo.manu_date >> 5) & 0xF, chip->battinfo.manu_date & 0x1F);

	return date_len;
}

static int mpc7022_get_vdm_data(struct chip_mpc7022 *chip)
{
	int ret;
	u8 first_usage_data_buf[MPC7022_FIRST_USAGE_DATA_NUM_SIZE] = {0};
	u8 uisoh_buf[MPC7022_UISOH_NUM_SIZE] = {0};
	u8 ui_cycle_count_buf[MPC7022_UI_CYCLE_COUNT_NUM_SIZE] = {0};
	u8 used_flag_buf[MPC7022_USED_FLAG_NUM_SIZE] = {0};

	if (!chip)
		return -EINVAL;

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	ret = mpc7022_read_block(
		chip, MPC7022_REG_OPLUS_DATA, first_usage_data_buf,
		MPC7022_FIRST_USAGE_DATA_NUM_SIZE, MPC7022_FIRST_USAGE_DATA_OFFSET, true, true);
	if (!ret)
		chip->battinfo.first_usage_date = first_usage_data_buf[0] | first_usage_data_buf[1] << 8;

	ret = mpc7022_read_block(
		chip, MPC7022_REG_OPLUS_DATA, uisoh_buf, MPC7022_UISOH_NUM_SIZE, MPC7022_UISOH_OFFSET, true, true);
	if (!ret)
		chip->battinfo.ui_soh = uisoh_buf[0];

	ret = mpc7022_read_block(
		chip, MPC7022_REG_OPLUS_DATA, ui_cycle_count_buf,
		MPC7022_UI_CYCLE_COUNT_NUM_SIZE, MPC7022_UI_CYCLE_COUNT_OFFSET, true, true);
	if (!ret)
		chip->battinfo.ui_cycle_count = ui_cycle_count_buf[0] | ui_cycle_count_buf[1] << 8;

	ret = mpc7022_read_block(
		chip, MPC7022_REG_OPLUS_DATA, used_flag_buf, MPC7022_USED_FLAG_NUM_SIZE, MPC7022_USED_FLAG_OFFSET, true, true);
	if (!ret)
		chip->battinfo.used_flag = used_flag_buf[0];

	return ret;
}

static void mpc7022_get_manu_battinfo_work(struct work_struct *work)
{
	struct chip_mpc7022 *chip =
		container_of(work, struct chip_mpc7022, get_manu_battinfo_work.work);

	mpc7022_get_batt_sn(chip);
	mpc7022_get_manu_date(chip);
	mpc7022_get_vdm_data(chip);
}

static int mpc7022_get_lifetime_status(
	struct chip_mpc7022 *chip, struct oplus_gauge_lifetime *lifetime)
{
	int ret;
	int cell_0_volt;
	int cell_1_volt;
	u8 buf[MPC7022_LIFETIME_NUM_SIZE] = {0};

	if (!chip || !lifetime)
		return -EINVAL;

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	ret = mpc7022_read_block(chip, MPC7022_REG_LIFETIME_ADDR,
			buf, MPC7022_LIFETIME_NUM_SIZE, 0, false, true);
	if (ret) {
		chg_err("error reading lifetime.\n");
		return -EINVAL;
	}
	cell_0_volt = (buf[1] << 0x08) | buf[0];
	cell_1_volt = (buf[3] << 0x08) | buf[2];
	lifetime->max_cell_vol = cell_0_volt > cell_1_volt ? cell_0_volt : cell_1_volt;
	lifetime->max_charge_curr = (buf[5] << 0x08) | buf[4];
	lifetime->max_dischg_curr = (buf[7] << 0x08) | buf[6];
	lifetime->max_cell_temp = buf[8];
	lifetime->min_cell_temp = buf[9];

	return 0;
}

static int oplus_mpc7022_get_lifetime_status(
	struct oplus_chg_ic_dev *ic_dev, struct oplus_gauge_lifetime *lifetime_status)
{
	struct chip_mpc7022 *chip;
	int ret = -EINVAL;

	if (ic_dev == NULL || lifetime_status == NULL) {
		chg_err("oplus_chg_ic_dev or lifetime_status is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ret = mpc7022_get_lifetime_status(chip, lifetime_status);

	return ret;
}

static int oplus_mpc7022_get_lifetime_info(
	struct oplus_chg_ic_dev *ic_dev, u8 *buf, int len)
{
	struct chip_mpc7022 *chip;
	int ret = 0;

	if (ic_dev == NULL || buf == NULL) {
		chg_err("oplus_chg_ic_dev or buf is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ret = mpc7022_get_lifetime_info(chip, buf, len);

	return ret;
}

static int mpc7022_get_battery_car_c(struct chip_mpc7022 *chip)
{
	int ret;
	int car_c_ptr;
	u8 buf[MPC7022_CAR_C_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return chip->car_c_pre;

	ret = mpc7022_read_block(chip, MPC7022_REG_CALIB_REASON_ADDR, buf,
		MPC7022_CAR_C_NUM_SIZE, MPC7022_CAR_C_OFFSET, false, true);
	if (ret < 0)
		return chip->car_c_pre;

	car_c_ptr = (buf[1] << 0x8) + buf[0];
	if (car_c_ptr & 0x8000)
		car_c_ptr = -((~(car_c_ptr - 1)) & 0xffff);

	chip->car_c_pre = car_c_ptr;

	return car_c_ptr;
}

static int oplus_mpc7022_get_battery_car_c(
	struct oplus_chg_ic_dev *ic_dev, int *car_c_ptr)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (car_c_ptr)
		*car_c_ptr = mpc7022_get_battery_car_c(chip);

	return 0;
}

static int mpc7022_set_batt_full(struct chip_mpc7022 *chip)
{
	int ret;
	u8 buf[MPC7022_SET_FULL_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	buf[0] = MPC7022_SET_FULL_DATA & 0xff;
	buf[1] = (MPC7022_SET_FULL_DATA >> 8) & 0xff;

	ret = mpc7022_write_block(
		chip, MPC7022_REG_SET_FULL, buf, MPC7022_SET_FULL_NUM_SIZE,
		MPC7022_SET_FULL_OFFSET, false, MPC7022_SET_FULL_NUM_SIZE, true, false);
	if (ret < 0)
		return ret;

	return 0;
}

static int oplus_mpc7022_set_batt_full(struct oplus_chg_ic_dev *ic_dev, bool full)
{
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (full)
		mpc7022_set_batt_full(chip);

	return 0;
}

static int mpc7022_set_cuv_state(struct chip_mpc7022 *chip, int cuv_state)
{
	int ret;
	u8 state_2_buf[] = {0x21, 0x43, 0xba, 0xdc, 0x34, 0x12, 0xcd, 0xab};
	u8 state_1_buf[] = {0x21, 0x43, 0xef, 0xbe, 0x34, 0x12, 0xed, 0xfe};

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	switch (cuv_state) {
	case OPLUS_GAUGE_CUV_STATE_1:
		ret = mpc7022_write_block(chip, MPC7022_REG_SET_CUV_STATE_1,
			state_1_buf, sizeof(state_1_buf), 0, false, sizeof(state_1_buf), true, false);
		break;
	case OPLUS_GAUGE_CUV_STATE_2:
		ret = mpc7022_write_block(chip, MPC7022_REG_SET_CUV_STATE_2,
			state_2_buf, sizeof(state_2_buf), 0, false, sizeof(state_2_buf), true, false);
		break;
	default:
		chg_err("state set error\n");
		ret = -EINVAL;
		break;
	}

	if (ret < 0)
		return ret;

	return 0;
}

static int mpc7022_shutdown_set_cuv_state(struct chip_mpc7022 *chip)
{
	int ret;
	int try_count = 1; /* try one time when i2c fail */

	do {
		ret = mpc7022_set_cuv_state(chip, OPLUS_GAUGE_CUV_STATE_1);
		if (!ret)
			break;
		msleep(600);
	} while (ret < 0 && try_count-- > 0);

	return ret;
}

static int oplus_mpc7022_set_cuv_state(struct oplus_chg_ic_dev *ic_dev, int cuv_state)
{
	int ret;
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ret = mpc7022_set_cuv_state(chip, cuv_state);

	return ret;
}

static int mpc7022_get_cuv_state(struct chip_mpc7022 *chip, int *cuv_state)
{
	int ret = 	-EINVAL;
	u8 buf[MPC7022_CUV_STATE_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip)) {
		*cuv_state = -EINVAL;
		return ret;
	}

	ret = mpc7022_read_block(chip, MPC7022_REG_CUV_STATE,
		buf, MPC7022_CUV_STATE_NUM_SIZE, 0, false, true);
	if (ret < 0) {
		*cuv_state = -EINVAL;
		return ret;
	}

	if (buf[0] & MPC7022_REG_CUV_STATE_MASK)
		*cuv_state = OPLUS_GAUGE_CUV_STATE_2;
	else
		*cuv_state = OPLUS_GAUGE_CUV_STATE_1;
	chg_info("cuv_state=%d\n", *cuv_state);

	return 0;
}

static int oplus_mpc7022_get_cuv_state(
	struct oplus_chg_ic_dev *ic_dev, int *cuv_state)
{
	int ret;
	struct chip_mpc7022 *chip;

	if (ic_dev == NULL || cuv_state == NULL) {
		chg_err("oplus_chg_ic_dev or cuv_state is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ret = mpc7022_get_cuv_state(chip, cuv_state);

	return ret;
}

static void *oplus_chg_get_func(
	struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
			oplus_mpc7022_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
			oplus_mpc7022_exit);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL,
			oplus_mpc7022_get_batt_vol);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX,
			oplus_mpc7022_get_batt_max);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN,
			oplus_mpc7022_get_batt_min);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR,
			oplus_mpc7022_get_batt_curr);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP,
			oplus_mpc7022_get_batt_temp);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC,
			oplus_mpc7022_get_batt_soc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC_CENTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC_CENTI,
			oplus_mpc7022_get_batt_soc_centi);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC,
			oplus_mpc7022_get_batt_fcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_CC,
			oplus_mpc7022_get_batt_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_RM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_RM,
			oplus_mpc7022_get_batt_rm);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH,
			oplus_mpc7022_get_batt_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH,
			oplus_mpc7022_get_batt_auth);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC,
			oplus_mpc7022_get_batt_hmac);
		break;
	case OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH,
			oplus_mpc7022_update_soc_smooth_parameter);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE,
			oplus_mpc7022_get_device_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_LOCK:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_LOCK,
			oplus_mpc7022_set_lock);
		break;
	case OPLUS_IC_FUNC_GAUGE_IS_LOCKED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_IS_LOCKED,
			oplus_mpc7022_is_locked);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM,
			oplus_mpc7022_get_batt_num);
		break;
	case OPLUS_IC_FUNC_GAUGE_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_IS_SUSPEND,
			oplus_mpc7022_is_suspend);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DOD0:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DOD0,
			oplus_mpc7022_get_battery_dod0);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DOD0_PASSED_Q:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DOD0_PASSED_Q,
			oplus_mpc7022_get_battery_dod0_passed_q);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_QMAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_QMAX,
			oplus_mpc7022_get_battery_qmax);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_QMAX_PASSED_Q:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_QMAX_PASSED_Q,
			oplus_mpc7022_get_battery_qmax_passed_q);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC,
			oplus_mpc7022_get_battery_gauge_type_for_bcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_STATUS,
			oplus_mpc7022_get_lifetime_status);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_INFO,
			oplus_mpc7022_get_lifetime_info);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST,
			oplus_mpc7022_get_batt_exist);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEEP_DISCHG_COUNT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEEP_DISCHG_COUNT,
			oplus_mpc7022_get_deep_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT,
			oplus_mpc7022_set_deep_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT,
			oplus_mpc7022_set_deep_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_LAST_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_LAST_CC,
			oplus_mpc7022_set_deep_last_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_REG_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_REG_INFO,
			oplus_mpc7022_get_reg_info);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_CALIB_TIME:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_CALIB_TIME,
			oplus_mpc7022_get_calib_time);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_CALIB_TIME:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_CALIB_TIME,
			oplus_mpc7022_set_calib_time);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SN,
			oplus_mpc7022_get_batt_sn);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_FIRST_USAGE_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_FIRST_USAGE_DATE,
			oplus_mpc7022_get_first_usage_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_FIRST_USAGE_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_FIRST_USAGE_DATE,
			oplus_mpc7022_set_first_usage_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_UI_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_UI_CC,
			oplus_mpc7022_get_ui_cycle_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_UI_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_UI_CC,
			oplus_mpc7022_set_ui_cycle_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_UI_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_UI_SOH,
			oplus_mpc7022_get_ui_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_UI_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_UI_SOH,
			oplus_mpc7022_set_ui_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_USED_FLAG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_USED_FLAG,
			oplus_mpc7022_get_used_flag);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_USED_FLAG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_USED_FLAG,
			oplus_mpc7022_set_used_flag);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_MANU_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_MANU_DATE,
			oplus_mpc7022_get_manu_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT,
			oplus_mpc7022_get_deep_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_LAST_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_LAST_CC,
			oplus_mpc7022_get_deep_last_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_GAUGE_CAR_C:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_GAUGE_CAR_C,
			oplus_mpc7022_get_battery_car_c);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL,
			oplus_mpc7022_set_batt_full);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_CUV_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_CUV_STATE,
			oplus_mpc7022_set_cuv_state);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_CUV_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_CUV_STATE,
			oplus_mpc7022_get_cuv_state);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq mpc7022_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{ .virq_id = OPLUS_IC_VIRQ_RESUME },
};

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
static struct regmap_config mpc7022_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
};
#endif

static void mpc7022_check_iic_recover(struct work_struct *work)
{
	struct chip_mpc7022 *chip = container_of(
		work, struct chip_mpc7022, check_iic_recover.work);

	/* workaround for I2C pull SDA can't trigger error issue 230504153935012779 */
	if (chip->i2c_rst_ext)
		mpc7022_get_battery_temperature(chip);
	else
		mpc7022_get_battery_soc(chip);

	chg_info("gauge online state:%d\n", chip->ic_dev->online);
	if (!chip->ic_dev->online) {
		schedule_delayed_work(&chip->check_iic_recover, msecs_to_jiffies(CHECK_IIC_RECOVER_TIME));
	} else {
	 	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ONLINE);
	}
}

static int mpc7022_vars_init(struct chip_mpc7022 *chip)
{
	atomic_set(&chip->suspended, 0);
	atomic_set(&chip->locked, 0);
	mutex_init(&chip->chip_mutex);
	mutex_init(&chip->calib_time_mutex);
	mutex_init(&chip->extended_cmd_access);
	mutex_init(&chip->block_access);

	/* workaround for I2C pull SDA can't trigger error issue 230504153935012779 */
	chip->err_status = false;

	 /* soc default set 50% */
	chip->soc_pre = 50;
	chip->soc_centi_pre = -EINVAL;
	/* current default set 999ma */
	chip->current_pre = 999;

	 /* volt default set 3800mv */
	chip->batt_max_volt_pre = 3800;
	chip->batt_min_volt_pre = 3800;
	chip->batt_cell_1_volt_pre = 3800;
	chip->batt_cell_2_volt_pre = 3800;

	INIT_DELAYED_WORK(&chip->check_iic_recover, mpc7022_check_iic_recover);
	INIT_DELAYED_WORK(&chip->get_manu_battinfo_work, mpc7022_get_manu_battinfo_work);

	return 0;
}

static void mpc7022_effect_term_volt_init(struct chip_mpc7022 *chip)
{
	int rc;
	int deep_term_volt;

	rc = mpc7022_get_deep_term_volt(chip, &deep_term_volt);
	if (!rc)
		mpc7022_set_term_volt(chip, deep_term_volt);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int mpc7022_driver_probe(struct i2c_client *client)
#else
static int mpc7022_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	int rc = 0;
	int ic_index;
	struct chip_mpc7022 *chip;
	enum oplus_chg_ic_type ic_type;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };

	chip = devm_kzalloc(&client->dev, sizeof(struct chip_mpc7022), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, chip);
	chip->dev = &client->dev;
	chip->client = client;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	chip->regmap = devm_regmap_init_i2c(client, &mpc7022_regmap_config);
	if (!chip->regmap) {
		rc = -ENODEV;
		goto regmap_init_err;
	}
#endif /* CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT */

	mpc7022_vars_init(chip);
	mpc7022_parse_dt(chip);
	mpc7022_hw_config(chip);

	rc = of_property_read_u32(chip->dev->of_node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		goto error;
	}
	rc = of_property_read_u32(chip->dev->of_node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto error;
	}
	ic_cfg.name = chip->dev->of_node->name;
	ic_cfg.index = ic_index;
	scnprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "gauge-mpc7022:%d", ic_index);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	chip->odb = devm_oplus_device_bus_register(chip->dev, &mpc7022_regmap_config, ic_cfg.manu_name);
	if (IS_ERR_OR_NULL(chip->odb)) {
		chg_err("register odb error\n");
		rc = -EFAULT;
		goto error;
	}
#endif /* CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT */
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = mpc7022_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(mpc7022_virq_table);
	ic_cfg.of_node = chip->dev->of_node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", chip->dev->of_node->name);
		goto ic_reg_error;
	}
	chg_info("register %s\n", chip->dev->of_node->name);

	mpc7022_effect_term_volt_init(chip);
	oplus_mpc7022_init(chip->ic_dev);
	schedule_delayed_work(&chip->get_manu_battinfo_work, 0);

	return 0;

ic_reg_error:
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	devm_oplus_device_bus_unregister(chip->odb);
#endif
error:
	mutex_destroy(&chip->chip_mutex);
	mutex_destroy(&chip->calib_time_mutex);
	mutex_destroy(&chip->extended_cmd_access);
	mutex_destroy(&chip->block_access);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
regmap_init_err:
#endif
	devm_kfree(&client->dev, chip);
	return rc;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
static int mpc7022_driver_remove(struct i2c_client *client)
#else
static void mpc7022_driver_remove(struct i2c_client *client)
#endif
{
	struct chip_mpc7022 *chip = i2c_get_clientdata(client);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
	if (chip == NULL)
		return -ENODEV;
#else
	if (chip == NULL)
		return;
#endif

	if (chip->ic_dev)
		devm_oplus_chg_ic_unregister(chip->dev, chip->ic_dev);
	chip->ic_dev = NULL;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	devm_oplus_device_bus_unregister(chip->odb);
#endif
	mutex_destroy(&chip->chip_mutex);
	mutex_destroy(&chip->calib_time_mutex);
	mutex_destroy(&chip->extended_cmd_access);
	mutex_destroy(&chip->block_access);

	devm_kfree(&client->dev, chip);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
	return 0;
#else
	return;
#endif
}

static const struct of_device_id mpc7022_match[] = {
	{ .compatible = "oplus,mpc7022-battery" },
	{},
};

static const struct i2c_device_id mpc7022_id[] = {
	{ "mpc7022-battery", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mpc7022_id);

static struct i2c_driver mpc7022_i2c_driver = {
	.driver = {
		.name = "mpc7022-battery",
		.owner = THIS_MODULE,
		.of_match_table = mpc7022_match,
		.pm = &mpc7022_pm_ops,
	},
	.probe = mpc7022_driver_probe,
	.remove = mpc7022_driver_remove,
	.shutdown = mpc7022_reset,
	.id_table = mpc7022_id,
};

static __init int mpc7022_driver_init(void)
{
	return i2c_add_driver(&mpc7022_i2c_driver);
}

static __exit void mpc7022_driver_exit(void)
{
	i2c_del_driver(&mpc7022_i2c_driver);
}

oplus_chg_module_register(mpc7022_driver);

MODULE_DESCRIPTION("Driver for mpc7022 charger chip");
MODULE_LICENSE("GPL v2");
