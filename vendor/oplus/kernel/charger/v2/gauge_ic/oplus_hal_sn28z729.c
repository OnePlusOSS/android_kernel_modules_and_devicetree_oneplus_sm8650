// SPDX-License-Identifier: GPL-2.0-only
/*Copyright (C) 2018-2025 Oplus. All rights reserved.*/

#define pr_fmt(fmt) "[SN28Z729]([%s][%d]): " fmt, __func__, __LINE__

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <asm/atomic.h>
#include <asm/unaligned.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#ifdef MODULE
#include <asm/setup.h>
#endif
#else
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/soc/qcom/smem.h>
#endif

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/device_info.h>
#endif

#include "oplus_hal_sn28z729.h"
#include "test-kit.h"
#include <linux/delay.h>
#include <linux/gfp.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg_vooc.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
#include <debug-kit.h>
#endif

#define GAUGE_ERROR (-1)
#define GAUGE_OK 0
#define BATT_FULL_ERROR 2
#define VOLT_MIN 1000
#define VOLT_MAX 5000
#define CURR_MAX 20000
#define CURR_MIN -25000
#define TEMP_MAX (1000 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN)
#define TEMP_MIN (-400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN)
#define SOH_MIN 0
#define SOH_MAX 100
#define FCC_MIN 10
#define FCC_MAX 12000
#define CC_MIN 0
#define CC_MAX 5000
#define QMAX_MIN 10
#define QMAX_MAX 12000
#define SOC_MIN 0
#define SOC_MAX 100
#define RETRY_CNT 3
#define I2C_ERR_MAX 2
#define CALIB_TIME_CHECK_ARGS 6
#define CHECK_IIC_RECOVER_TIME 5000
#define HMAC_MSG_LEN 20
#define SMEM_RESERVED_BOOT_INFO_FOR_APPS 418
#define EXTEND_CMD_TRY_COUNT 3
#define SEAL_POLLING_RETRY_LIMIT 100
#define SOC_CENTI_MIN		0
#define SOC_CENTI_MAX		10000

#define DATAFLASHBLOCK 0x3F
#define AUTHENDATA 0x40
#define AUTHENCHECKSUM 0x54
/* #define XBL_AUTH_DEBUG */

#define SN28Z729_AUTHENTICATE_DATA_COUNT sizeof(struct sn28z729_authenticate_data)
struct sn28z729_block_access {
	int addr;
	int len;
	int start_index;
	int end_index;
};

struct sn28z729_cmd_address {
	u8 reg_temp;
	u8 reg_volt;
	u8 reg_flags;
	u8 reg_nac;
	u8 reg_rm;
	u8 reg_fcc;
	u8 reg_ai;
	u8 reg_soc;
	u8 reg_soh;
	u8 reg_cc;
};

struct chip_sn28z729 {
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
	atomic_t sync_lock;
	struct mutex chip_mutex;
	struct mutex calib_time_mutex;
	struct mutex extended_cmd_access;

	oplus_gauge_auth_result auth_data;
	struct sn28z729_authenticate_data *authenticate_data;
	struct oplus_gauge_sha256_auth *sha256_authenticate_data;

	int batt_num;
	int gauge_num;
	int device_type;
	int firm_ver;

	bool i2c_err;
	bool probe_err;

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

	struct sn28z729_cmd_address cmd_addr;

	bool calib_info_init;
	int dod_time;
	int qmax_time;
	int dod_time_pre;
	int qmax_time_pre;
	int calib_check_args[CALIB_TIME_CHECK_ARGS];
	int calib_check_args_pre[CALIB_TIME_CHECK_ARGS];
	int sn28z729_seal_flag;
	struct delayed_work check_iic_recover;
	/* workaround for I2C pull SDA can't trigger error issue 230504153935012779 */
	bool i2c_rst_ext;
	bool err_status;
	bool support_eco_design;
	struct work_struct fcc_too_small_check_work;
	struct battery_manufacture_info battinfo;
};

static int oplus_sn28z729_init(struct oplus_chg_ic_dev *ic_dev);
static int oplus_sn28z729_exit(struct oplus_chg_ic_dev *ic_dev);
static int sn28z729_get_firm_ver(struct chip_sn28z729 *chip);

static __inline__ void sn28z729_push_i2c_err(struct chip_sn28z729 *chip, bool read)
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
		oplus_sn28z729_exit(chip->ic_dev);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_OFFLINE);
		schedule_delayed_work(&chip->check_iic_recover,
			msecs_to_jiffies(CHECK_IIC_RECOVER_TIME));
	} else {
		oplus_chg_ic_creat_err_msg(chip->ic_dev, OPLUS_IC_ERR_I2C, 0,
						read ? "read error" : "write error");
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
	}
}

static __inline__ void sn28z729_i2c_err_clr(struct chip_sn28z729 *chip)
{
	if (unlikely(chip->i2c_err)) {
		/* workaround for I2C pull SDA can't trigger error issue 230504153935012779 */
		if (chip->i2c_rst_ext && chip->err_status)
			return;
		/* end workaround 230504153935012779 */

		chip->i2c_err = false;
		atomic_set(&chip->i2c_err_count, 0);
		oplus_sn28z729_init(chip->ic_dev);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ONLINE);
	}
}

static inline bool is_chip_suspended_or_locked(struct chip_sn28z729 *chip)
{
	return atomic_read(&chip->suspended) || atomic_read(&chip->locked);
}

static int sn28z729_read_i2c(struct chip_sn28z729 *chip, int cmd, int *returnData)
{
	int retry = RETRY_CNT;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	int rc;
#endif

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return GAUGE_ERROR;
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
	} while (*returnData < 0 && retry--);
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
		sn28z729_push_i2c_err(chip, true);
		return 1;
	}

	sn28z729_i2c_err_clr(chip);
	return 0;
}

static int sn28z729_i2c_txsubcmd(struct chip_sn28z729 *chip, int cmd, int writeData)
{
	int rc;
	int retry = RETRY_CNT;

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return 0;
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
	} while (rc < 0 && retry--);
	mutex_unlock(&chip->chip_mutex);

	if (rc < 0) {
		chg_err("write err, rc = %d\n", rc);
		sn28z729_push_i2c_err(chip, false);
		return GAUGE_ERROR;
	}

	sn28z729_i2c_err_clr(chip);
	return 0;
}

static int sn28z729_write_i2c_block(struct chip_sn28z729 *chip, u8 cmd, u8 length, u8 *writeData)
{
	int rc;
	int retry = RETRY_CNT;

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return GAUGE_ERROR;
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
	} while (rc < 0 && retry--);
	mutex_unlock(&chip->chip_mutex);

	if (rc < 0) {
		chg_err("write err, rc = %d\n", rc);
		sn28z729_push_i2c_err(chip, false);
		return GAUGE_ERROR;
	}

	sn28z729_i2c_err_clr(chip);
	return 0;
}

#define I2C_SMBUS_BLOCK_MAX	32
static int sn28z729_read_i2c_block(struct chip_sn28z729 *chip, u8 cmd, u8 length, u8 *returnData)
{
	int rc;
	int retry = RETRY_CNT;

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return GAUGE_ERROR;
	}
	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	do {
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
		rc = oplus_dev_bus_bulk_read(chip->odb, cmd, returnData, length);
#else
		if (length <= I2C_SMBUS_BLOCK_MAX) {
			rc = i2c_smbus_read_i2c_block_data(chip->client, cmd, length,
				returnData);
			if (rc < 0)
				chg_err("read 0x%-4x error, rc=%d\n", cmd, rc);
		} else {
			rc = i2c_master_send(chip->client, &cmd, 1);
			if (rc < 1) {
				chg_err("write 0x%-4x error, rc=%d\n", cmd, rc);
				rc = rc < 0 ? rc : -EIO;
				goto retry;
			}
			rc = i2c_master_recv(chip->client, returnData, length);
			if (rc < length) {
				chg_err("read 0x%-4x error, rc=%d\n", cmd, rc);
				rc = rc < 0 ? rc : -EIO;
			}
		}
retry:
#endif
		if (rc < 0) {
			retry--;
			usleep_range(5000, 5000);
		}
	} while (rc < 0 && retry);

	if (rc < 0) {
		chg_err("read err, rc = %d\n", rc);
		sn28z729_push_i2c_err(chip, true);
	} else {
		sn28z729_i2c_err_clr(chip);
		rc = 0;
	}
	mutex_unlock(&chip->chip_mutex);
	return rc;
}

__maybe_unused static int sn28z729_read_i2c_onebyte(struct chip_sn28z729 *chip,
				u8 cmd, u8 *returnData)
{
	int rc;
	int retry = RETRY_CNT;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	unsigned int buf;
#endif

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return GAUGE_ERROR;
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
	} while (rc < 0 && retry--);

	if (rc >= 0)
		*returnData = (u8)rc;
	mutex_unlock(&chip->chip_mutex);

	if (rc < 0) {
		chg_err("read err, rc = %d\n", rc);
		sn28z729_push_i2c_err(chip, true);
		return GAUGE_ERROR;
	}

	sn28z729_i2c_err_clr(chip);
	return 0;
}

__maybe_unused static int sn28z729_i2c_txsubcmd_onebyte(struct chip_sn28z729 *chip,
							u8 cmd, u8 writeData)
{
	int rc;
	int retry = RETRY_CNT;

	if (!chip->client) {
		chg_err("gauge client is null\n");
		return 0;
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
	} while (rc < 0 && retry--);
	mutex_unlock(&chip->chip_mutex);

	if (rc < 0) {
		chg_err("write err, rc = %d\n", rc);
		sn28z729_push_i2c_err(chip, true);
		return GAUGE_ERROR;
	}

	sn28z729_i2c_err_clr(chip);
	return 0;
}

static u8 sn28z729_calc_checksum(u8 *buf, int len)
{
	u8 checksum = 0;

	while (len--)
		checksum += buf[len];

	return 0xff - checksum;
}

static int sn28z729_block_check_conditions(u8 *buf, int len, int offset,
						bool do_checksum, int block_size)
{
	if (!buf || (block_size > SN28Z729_BLOCK_SIZE) ||
		(offset < 0) || (offset >= block_size) || (len <= 0) ||
		(len + do_checksum > block_size) ||
		(offset + len + do_checksum > block_size)) {
		chg_err("%s offset[%d] or len[%d] block_size[%d] invalid\n",
			buf == NULL ? "buf is null or " : "", offset, len, block_size);
		return -EINVAL;
	}
	return 0;
}

static int sn28z729_read_block(struct chip_sn28z729 *chip, int addr, u8 *buf, int len,
						int offset, bool do_checksum, bool access_lock)
{
	int ret;
	int data_check;
	int try_count = EXTEND_CMD_TRY_COUNT;
	u8 extend_data[SN28Z729_BLOCK_SIZE + 2] = { 0 };
	u8 checksum = 0;

	ret = sn28z729_block_check_conditions(buf, len, offset, do_checksum, SN28Z729_BLOCK_SIZE);
	if (ret < 0)
		return ret;

try:
	if (access_lock)
		mutex_lock(&chip->extended_cmd_access);
	ret = sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, addr);
	if (ret < 0)
		goto error;

	usleep_range(1000, 1000);
	ret = sn28z729_read_i2c_block(chip, SN28Z729_DATA_FLASH_BLOCK,
			(offset + len + do_checksum + 2), extend_data);
	if (ret < 0)
		goto error;

	data_check = (extend_data[1] << 0x8) | extend_data[0];
	if (try_count-- > 0 && data_check != addr) {
		chg_err("0x%04x not match. try_count=%d extend_data[0]=0x%2x, "
			"extend_data[1]=0x%2x\n",
			addr, try_count, extend_data[0], extend_data[1]);
		if (access_lock)
			mutex_unlock(&chip->extended_cmd_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	if (do_checksum) {
		checksum = sn28z729_calc_checksum(&extend_data[offset + 2], len);
		if (checksum != extend_data[offset + len + 2]) {
			chg_err("[%*ph]checksum not match. expect=0x%02x actual=0x%02x\n",
				offset + len + do_checksum + 2, extend_data, checksum,
						extend_data[offset + len + 2]);
			goto error;
		}
	}

	memmove(buf, &extend_data[offset + 2], len);
	chg_debug("addr=0x%04x offset=%d buf=[%*ph] do_checksum=%d read success\n",
					addr, offset, len, buf, do_checksum);
	if (access_lock)
		mutex_unlock(&chip->extended_cmd_access);
	return 0;

error:
	chg_err("addr=0x%04x offset=%d buf=[%*ph] do_checksum=%d read fail\n", addr,
						offset, len, buf, do_checksum);
	if (access_lock)
		mutex_unlock(&chip->extended_cmd_access);

	return -EINVAL;
}

static int sn28z729_write_block(struct chip_sn28z729 *chip, int addr, u8 *buf, int len,
				int offset, bool do_checksum, int block_size, bool access_lock)
{
	int ret;
	int data_check;
	int try_count = EXTEND_CMD_TRY_COUNT;
	u8 extend_read_data[SN28Z729_BLOCK_SIZE + 2] = { 0 };
	u8 extend_write_data[SN28Z729_BLOCK_SIZE + 2] = { 0 };
	u8 check_data[2] = { 0 };
	u8 checksum = 0;

	ret = sn28z729_block_check_conditions(buf, len, offset, do_checksum, block_size);
	if (ret < 0)
		return ret;

try:
	if (access_lock)
		mutex_lock(&chip->extended_cmd_access);
	ret = sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, addr);
	if (ret < 0)
		goto error;
	usleep_range(1000, 1000);
	ret = sn28z729_read_i2c_block(chip, SN28Z729_DATA_FLASH_BLOCK, (block_size + 2),
						extend_read_data);
	if (ret < 0)
		goto error;

	data_check = (extend_read_data[1] << 0x8) | extend_read_data[0];
	if ((try_count-- > 0) && (data_check != addr)) {
		chg_err("0x%04x not match. try_count=%d offset=%d extend_data[0]=0x%2x, "
				"extend_data[1]=0x%2x\n",
				addr, try_count, offset, extend_read_data[0], extend_read_data[1]);
		if (access_lock)
			mutex_unlock(&chip->extended_cmd_access);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	memmove(extend_write_data, extend_read_data, block_size + 2);
	memmove(&extend_write_data[offset + 2], buf, len);
	if (do_checksum)
		extend_write_data[offset + len + 2] = sn28z729_calc_checksum(buf, len);

	ret = sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, addr);
	if (ret < 0)
		goto error;
	ret = sn28z729_write_i2c_block(chip, SN28Z729_EXTEND_DATA_ADDR, block_size,
							extend_write_data + 2);
	if (ret < 0)
		goto error;
	checksum = sn28z729_calc_checksum(extend_write_data, block_size + 2);
	check_data[0] = checksum;
	check_data[1] = block_size + 2 + 2;
	ret = sn28z729_write_i2c_block(chip, SN28Z729_EXTEND_DATA_CHECKSUM_ADDR, 2,
		check_data);
	if (ret < 0)
		goto error;

	try_count = EXTEND_CMD_TRY_COUNT;
	do {
		data_check = true;
		memset(extend_read_data, 0, block_size + 2);
		usleep_range(15000, 15000);
		ret = sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, addr);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		ret = sn28z729_read_i2c_block(chip, SN28Z729_DATA_FLASH_BLOCK, block_size + 2,
						extend_read_data);
		if (memcmp(extend_read_data, extend_write_data, block_size + 2)) {
			chg_err("reg not match.extend_read_data =[%*ph]\n", block_size + 2,
			extend_read_data);
			chg_err("reg not match.extend_write_data=[%*ph]\n", block_size + 2,
			extend_write_data);
			data_check = false;
		}
	} while (!data_check && try_count--);
	if (!data_check)
		goto error;

	if (access_lock)
		mutex_unlock(&chip->extended_cmd_access);
	chg_debug("addr=0x%04x offset=%d buf=[%*ph] write success\n", addr, offset, len, buf);
	return 0;

error:
	chg_info("addr=0x%04x offset=%d buf=[%*ph] write fail\n", addr, offset, len, buf);
	if (access_lock)
		mutex_unlock(&chip->extended_cmd_access);
	return -EINVAL;
}

static bool normal_range_judge(int max, int min, int data)
{
	if (data > max || data < min)
		return false;

	return true;
}

static int sn28z729_sealed(struct chip_sn28z729 *chip)
{
	int value = 0;
	int ret = 0;
	u8 CNTL1_VAL[SN28Z729_REG_SEAL_SIZE] = { 0 };

	ret = sn28z729_read_block(chip, SN28Z729_SEAL_STATUS, CNTL1_VAL, SN28Z729_REG_SEAL_SIZE, 0,
		false, false);
	if (ret < 0) {
		chg_err("sn28z729 sealed, read ret error");
		return -EINVAL;
	}
	chg_debug("sn28z729_sealed CNTL1_VAL[%x, %x, %x, %x]\n", CNTL1_VAL[0], CNTL1_VAL[1],
		CNTL1_VAL[2], CNTL1_VAL[3]);
	value = (CNTL1_VAL[1] & SN28Z729_SEAL_BIT);
	if (value == SN28Z729_SEAL_VALUE) {
		chg_info("sn28z729 sealed, value = %x return 1\n", value);
		return 1;
	}

	chg_info("sn28z729 sealed, value = %x return 0\n", value);
	return 0;
}

__maybe_unused static int sn28z729_seal(struct chip_sn28z729 *chip)
{
	int i = 0;
	int ret = 0;

	if (sn28z729_sealed(chip)) {
		chg_info("sn28z729 sealed, return\n");
		return 1;
	}
	sn28z729_i2c_txsubcmd(chip, 0, SN28Z729_SEALED_SUBCMD);
	usleep_range(100000, 100000);
	for (i = 0; i < SN28Z729_SEAL_POLLING_RETRY_LIMIT; i++) {
		ret = sn28z729_sealed(chip);
		if (1 == ret) {
			chg_debug("sn28z729 sealed,used %d x100ms\n", i);
			return 1;
		} else if (-1 == ret) {
			chg_err("sn28z729 seal failed by ret error\n");
			return 0;
		}
		usleep_range(10000, 10000);
	}
	return 0;
}

__maybe_unused static int sn28z729_unseal(struct chip_sn28z729 *chip)
{
	int i = 0;
	int ret = 0;

	if (!sn28z729_sealed(chip))
		goto out;

	sn28z729_i2c_txsubcmd(chip, 0, SN28Z729_UNSEALED_SUBCMD1);
	usleep_range(10000, 10000);
	sn28z729_i2c_txsubcmd(chip, 0, SN28Z729_UNSEALED_SUBCMD2);
	usleep_range(100000, 100000);
	while (i < SN28Z729_SEAL_POLLING_RETRY_LIMIT) {
		i++;
		ret = sn28z729_sealed(chip);
		if (0 == ret) {
			chg_debug("sn28z729 unsealed,used %d x100ms\n", i);
			break;
		} else if (GAUGE_ERROR == ret) {
			chg_err("sn28z729 unseal failed by ret error\n");
			return 0;
		}
		usleep_range(10000, 10000);
	}

out:
	chg_debug("sn28z729 : i=%d\n", i);
	if (i == SN28Z729_SEAL_POLLING_RETRY_LIMIT) {
		chg_err("sn28z729 unseal failed\n");
		return 0;
	} else {
		return 1;
	}
}

static int sn28z729_i2c_deep_int(struct chip_sn28z729 *chip)
{
	int rc = 0;

	if (!chip->client) {
		chg_info("gauge_ic->client NULL, return\n");
		return 0;
	}
	if (oplus_is_rf_ftm_mode()) {
		chg_info("oplus_is_rf_ftm_mode err");
		return 0;
	}
	mutex_lock(&chip->chip_mutex);
	rc = i2c_smbus_write_word_data(chip->client, SN28Z729_DATA_FLASH_BLOCK,
		SN28Z729_UNSEALED_SUBCMD1);
	if (rc < 0)
		chg_info("SN28Z729_UNSEALED_SUBCMD1 is err");

	usleep_range(10000, 10000);
	rc = i2c_smbus_write_word_data(chip->client, SN28Z729_DATA_FLASH_BLOCK,
		SN28Z729_UNSEALED_SUBCMD2);
	if (rc < 0)
		chg_info("write err, rc = %d\n", rc);

	mutex_unlock(&chip->chip_mutex);
	return 0;
}

static bool sn28z729_deep_init(struct chip_sn28z729 *chip)
{
	if (!sn28z729_sealed(chip)) {
		chg_info("sn28z729 already unsealed\n");
		return true;
	}
	sn28z729_i2c_deep_int(chip);

	usleep_range(100000, 100000);
	if (!sn28z729_sealed(chip))
		return true;

	chg_info("sn28z729 unseal failed\n");
	return false;
}

static void sn28z729_deep_deinit(struct chip_sn28z729 *chip)
{
	int i = 0;

	if (sn28z729_sealed(chip) == 0) {
		usleep_range(1000, 1000);
		sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, SN28Z729_SEALED_SUBCMD);
		usleep_range(100000, 100000);
		for (i = 0; i < SN28Z729_SEAL_POLLING_RETRY_LIMIT; i++) {
			if (sn28z729_sealed(chip)) {
				chg_info("sn28z729 sealed,used %d x100ms\n", i);
				return;
			}
			usleep_range(10000, 10000);
		}
	}
}

static int sn28z729_get_battery_cc(struct chip_sn28z729 *chip)
{
	int ret;
	int cc = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return 0;

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return chip->cc_pre;

	do {
		ret = sn28z729_read_i2c(chip, chip->cmd_addr.reg_cc, &cc);
		if (ret) {
			dev_err(chip->dev, "error reading cc.\n");
			return chip->cc_pre;
		}
		if (normal_range_judge(CC_MAX, CC_MIN, cc))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("cc abnormal, retry:%d, cc:%d\n", retry, cc);
	} while (retry--);

	if (retry < 0)
		return chip->cc_pre;

	chip->cc_pre = cc;
	return cc;
}

static int sn28z729_get_true_fcc(struct chip_sn28z729 *chip, int *true_fcc)
{
	int ret;
	u8 buf[SN28Z729_TRUE_FCC_NUM_SIZE] = { 0 };

	if (!chip || !true_fcc)
		return -EINVAL;

	ret = sn28z729_read_block(
		chip, SN28Z729_REG_TRUE_FCC, buf, SN28Z729_TRUE_FCC_NUM_SIZE, SN28Z729_TRUE_FCC_OFFSET, false, true);
	if (ret < 0)
		return ret;

	*true_fcc = (buf[1] << 8) | buf[0];
	chg_info("true_fcc=%d\n", *true_fcc);

	return ret;
}

static void sn28z729_set_fcc_sync(struct chip_sn28z729 *chip)
{
	mutex_lock(&chip->extended_cmd_access);
	sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, SN28Z729_FCC_SYNC_CMD);
	mutex_unlock(&chip->extended_cmd_access);
	chg_info("set fcc sync\n");
}

static void sn28z729_fcc_too_small_check_work(struct work_struct *work)
{
	int ret;
	int true_fcc = 0;
	struct chip_sn28z729 *chip = container_of(
		work, struct chip_sn28z729, fcc_too_small_check_work);

	ret = sn28z729_get_true_fcc(chip, &true_fcc);
	if (!ret && (true_fcc > SN28Z729_FCC_SYNC_THD)) /* TODO: true_fcc value is more than q */
		sn28z729_set_fcc_sync(chip);

	atomic_set(&chip->sync_lock, 0);
}

static void sn28z729_fcc_too_small_check(struct chip_sn28z729 *chip, int fcc)
{
	if (!chip)
		return;

	if (atomic_read(&chip->sync_lock)) {
		chg_info("fcc too small checking, ignore this time");
		return;
	}

	/* TODO: fcc value is less than 200 */
	if (fcc < SN28Z729_FCC_SYNC_THD) {
		atomic_set(&chip->sync_lock, 1);
		schedule_work(&chip->fcc_too_small_check_work);
	}
}

static int sn28z729_get_battery_fcc(struct chip_sn28z729 *chip)
{
	int ret;
	int fcc = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return 0;

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return chip->fcc_pre;

	do {
		ret = sn28z729_read_i2c(chip, chip->cmd_addr.reg_fcc, &fcc);
		if (ret) {
			dev_err(chip->dev, "error reading fcc.\n");
			return chip->fcc_pre;
		}
		if (normal_range_judge(FCC_MAX, FCC_MIN, fcc))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("fcc abnormal, retry:%d, fcc:%d\n", retry, fcc);
	} while (retry--);

	if (retry < 0)
		return chip->fcc_pre;

	chip->fcc_pre = fcc;
	sn28z729_fcc_too_small_check(chip, fcc);

	return fcc;
}

static int sn28z729_get_battery_soh(struct chip_sn28z729 *chip)
{
	int ret;
	int soh = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return 0;

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return chip->soh_pre;

	do {
		ret = sn28z729_read_i2c(chip, chip->cmd_addr.reg_soh, &soh);
		if (ret) {
			dev_err(chip->dev, "error reading soh.\n");
			return chip->soh_pre;
		}
		if (normal_range_judge(SOH_MAX, SOH_MIN, soh))
			break;

		usleep_range(10000, 10000);
		chg_err("soh abnormal, retry:%d, soh:%d\n", retry, soh);
	} while (retry--);

	if (retry < 0)
		return chip->soh_pre;

	chip->soh_pre = soh;
	return soh;
}

static int sn28z729_get_pre_batt_volt(struct chip_sn28z729 *chip, enum sn28z729_volt_type type)
{
	int volt = 0;

	switch (type) {
	case SN28Z729_CELL_MAX_VOLT:
		volt = chip->batt_max_volt_pre;
		break;
	case SN28Z729_CELL_MIN_VOLT:
		volt = chip->batt_min_volt_pre;
		break;
	case SN28Z729_CELL_1_VOLT:
		volt = chip->batt_cell_1_volt_pre;
		break;
	case SN28Z729_CELL_2_VOLT:
		volt = chip->batt_cell_2_volt_pre;
		break;
	default:
		break;
	}

	return volt;
}

static int sn28z729_update_pre_batt_volt(struct chip_sn28z729 *chip, enum sn28z729_volt_type type,
	int batt_cell_1_volt, int batt_cell_2_volt)
{
	int volt = 0;
	int batt_max_volt;
	int batt_min_volt;

	batt_max_volt = batt_cell_1_volt > batt_cell_2_volt ? batt_cell_1_volt : batt_cell_2_volt;
	batt_min_volt = batt_cell_1_volt > batt_cell_2_volt ? batt_cell_2_volt : batt_cell_1_volt;

	switch (type) {
	case SN28Z729_CELL_MAX_VOLT:
		chip->batt_max_volt_pre = batt_max_volt;
		volt = batt_max_volt;
		break;
	case SN28Z729_CELL_MIN_VOLT:
		chip->batt_min_volt_pre = batt_min_volt;
		volt = batt_min_volt;
		break;
	case SN28Z729_CELL_1_VOLT:
		chip->batt_cell_1_volt_pre = batt_cell_1_volt;
		volt = batt_cell_1_volt;
		break;
	case SN28Z729_CELL_2_VOLT:
		chip->batt_cell_2_volt_pre = batt_cell_2_volt;
		volt = batt_cell_2_volt;
		break;
	default:
		chg_err("volt type err\n");
		break;
	}

	return volt;
}

static int sn28z729_get_batt_volt(struct chip_sn28z729 *chip, enum sn28z729_volt_type type)
{
	int ret;
	int volt = 0;
	int batt_cell_1_volt = 0;
	int batt_cell_2_volt = 0;
	int retry = RETRY_CNT;
	u8 buf[SN28Z729_VOLT_NUM_SIZE] = { 0 };

	if (!chip)
		return 0;

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked)) {
		volt = sn28z729_get_pre_batt_volt(chip, type);
		return volt;
	}

	do {
		ret = sn28z729_read_block(chip, SN28Z729_REG_DA_STATUS1, buf,
				SN28Z729_VOLT_NUM_SIZE, 0, false, true);
		if (ret) {
			dev_err(chip->dev, "error reading volt.\n");
			return sn28z729_get_pre_batt_volt(chip, type);
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
		return sn28z729_get_pre_batt_volt(chip, type);

	volt = sn28z729_update_pre_batt_volt(chip, type, batt_cell_1_volt, batt_cell_2_volt);
	return volt;
}

static int sn28z729_get_battery_temperature(struct chip_sn28z729 *chip)
{
	int ret;
	int temp = 0;
	static int err_count = 0;
	int retry = RETRY_CNT;

	if (is_chip_suspended_or_locked(chip))
		return chip->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;

	do {
		ret = sn28z729_read_i2c(chip, chip->cmd_addr.reg_temp, &temp);
		if (ret) {
			dev_err(chip->dev, "error reading temp.\n");
			err_count++;
			if (err_count > 1) {
				err_count = 0;
				chip->temp_pre = -400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
				return -400;
			}
			return chip->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
		}
		err_count = 0;
		if (normal_range_judge(TEMP_MAX, TEMP_MIN, temp))
			break;

		usleep_range(10000, 10000);
		chg_err("temp abnormal, retry:%d, temp:%d\n", retry, (temp +
				ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN));
	} while (retry--);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (chip->i2c_rst_ext) {
		if (!temp) {
			chg_err("fg read temperature i2c error, set err flag\n");
			chip->err_status = true;
#ifdef CONFIG_OPLUS_FG_ERROR_RESET_I2C
			oplus_set_fg_err_flag(chip->client->adapter, true);
#endif
			sn28z729_push_i2c_err(chip, true);
			temp = -400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
		} else {
			chip->err_status = false;
#ifdef CONFIG_OPLUS_FG_ERROR_RESET_I2C
			oplus_set_fg_err_flag(chip->client->adapter, false);
#endif
			sn28z729_i2c_err_clr(chip);
		}
	}
#endif
	if (temp > TEMP_MAX)
		temp = chip->temp_pre;
	chip->temp_pre = temp;

	return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

static int sn28z729_get_batt_remaining_capacity(struct chip_sn28z729 *chip)
{
	int ret;
	int cap = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return GAUGE_ERROR;

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return chip->rm_pre;

	do {
		ret = sn28z729_read_i2c(chip, chip->cmd_addr.reg_rm, &cap);
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

static int sn28z729_get_pre_dod_parameters(struct chip_sn28z729 *chip,
					enum sn28z729_dod_parameter_type type)
{
	int dod_parameters = 0;

	switch (type) {
	case SN28Z729_CELL_1_DOD0:
		dod_parameters = chip->batt_cell_1_dod0_pre;
		break;
	case SN28Z729_CELL_2_DOD0:
		dod_parameters = chip->batt_cell_2_dod0_pre;
		break;
	case SN28Z729_CELL_DOD_PASSED_Q:
		dod_parameters = chip->batt_cell_dod_passed_q_pre;
		break;
	default:
		break;
	}

	return dod_parameters;
}

static int sn28z729_update_pre_dod_parameters(struct chip_sn28z729 *chip,
			enum sn28z729_dod_parameter_type type, int batt_cell_1_dod0,
					int batt_cell_2_dod0, int batt_cell_dod_passed_q)
{
	int dod_parameters = 0;

	switch (type) {
	case SN28Z729_CELL_1_DOD0:
		chip->batt_cell_1_dod0_pre = batt_cell_1_dod0;
		dod_parameters = batt_cell_1_dod0;
		break;
	case SN28Z729_CELL_2_DOD0:
		chip->batt_cell_2_dod0_pre = batt_cell_2_dod0;
		dod_parameters = batt_cell_2_dod0;
		break;
	case SN28Z729_CELL_DOD_PASSED_Q:
		chip->batt_cell_dod_passed_q_pre = batt_cell_dod_passed_q;
		dod_parameters = batt_cell_dod_passed_q;
		break;
	default:
		chg_info("dod_parameters type err\n");
		break;
	}

	return dod_parameters;
}

static int sn28z729_get_dod_parameters(struct chip_sn28z729 *chip,
					enum sn28z729_dod_parameter_type type)
{
	int ret;
	int dod_parameters;
	int batt_cell_1_dod0;
	int batt_cell_2_dod0;
	int batt_cell_dod_passed_q;
	u8 buf[SN28Z729_DOD_NUM_SIZE] = { 0 };

	if (!chip)
		return 0;

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked)) {
		dod_parameters = sn28z729_get_pre_dod_parameters(chip, type);
		return dod_parameters;
	}

	ret = sn28z729_read_block(chip, SN28Z729_REG_IT_STATUS2, buf, SN28Z729_DOD_NUM_SIZE,
					SN28Z729_DOD_OFFSET, false, true);
	if (ret)
		return sn28z729_get_pre_dod_parameters(chip, type);

	batt_cell_1_dod0 = (buf[1] << 8) | buf[0];
	batt_cell_2_dod0 = (buf[3] << 8) | buf[2];
	batt_cell_dod_passed_q = (buf[5] << 8) | buf[4];
	if (batt_cell_dod_passed_q & 0x8000)
		batt_cell_dod_passed_q = -((~(batt_cell_dod_passed_q - 1)) & 0xFFFF);

	dod_parameters = sn28z729_update_pre_dod_parameters(chip, type, batt_cell_1_dod0,
							batt_cell_2_dod0, batt_cell_dod_passed_q);

	return dod_parameters;
}

static int sn28z729_get_pre_qmax_parameters(struct chip_sn28z729 *chip,
				enum sn28z729_qmax_parameter_type type)
{
	int qmax_parameters = 0;

	switch (type) {
	case SN28Z729_CELL_1_QMAX:
		qmax_parameters = chip->batt_cell_1_qmax_pre;
		break;
	case SN28Z729_CELL_2_QMAX:
		qmax_parameters = chip->batt_cell_2_qmax_pre;
		break;
	case SN28Z729_CELL_QMAX_PASSED_Q:
		qmax_parameters = chip->batt_cell_qmax_passed_q_pre;
		break;
	default:
		break;
	}

	return qmax_parameters;
}

static int sn28z729_update_pre_qmax_parameters(struct chip_sn28z729 *chip,
		enum sn28z729_qmax_parameter_type type, int batt_cell_1_qmax,
		int batt_cell_2_qmax, int batt_cell_qmax_passed_q)
{
	int qmax_parameters = 0;

	switch (type) {
	case SN28Z729_CELL_1_QMAX:
		chip->batt_cell_1_qmax_pre = batt_cell_1_qmax;
		qmax_parameters = batt_cell_1_qmax;
		break;
	case SN28Z729_CELL_2_QMAX:
		chip->batt_cell_2_qmax_pre = batt_cell_2_qmax;
		qmax_parameters = batt_cell_2_qmax;
		break;
	case SN28Z729_CELL_QMAX_PASSED_Q:
		chip->batt_cell_qmax_passed_q_pre = batt_cell_qmax_passed_q;
		qmax_parameters = batt_cell_qmax_passed_q;
		break;
	default:
		chg_err("qmax_parameters type err\n");
		break;
	}

	return qmax_parameters;
}

static int sn28z729_get_qmax_parameters(struct chip_sn28z729 *chip,
			enum sn28z729_qmax_parameter_type type)
{
	int ret;
	int qmax_parameters;
	int batt_cell_1_qmax;
	int batt_cell_2_qmax;
	int batt_cell_qmax_passed_q;
	int retry = RETRY_CNT;
	u8 buf[SN28Z729_QMAX_NUM_SIZE] = { 0 };

	if (!chip)
		return 0;

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked)) {
		qmax_parameters = sn28z729_get_pre_qmax_parameters(chip, type);
		return qmax_parameters;
	}

	do {
		ret = sn28z729_read_block(chip, SN28Z729_REG_IT_STATUS3, buf,
				SN28Z729_QMAX_NUM_SIZE, 0, false, true);
		if (ret < 0)
			return sn28z729_get_pre_qmax_parameters(chip, type);

		batt_cell_1_qmax = (buf[1] << 8) | buf[0];
		batt_cell_2_qmax = (buf[3] << 8) | buf[2];
		batt_cell_qmax_passed_q = (buf[SN28Z729_QMAX_NUM_SIZE - 1] << 8) |
			buf[SN28Z729_QMAX_NUM_SIZE - 2];
		if (batt_cell_qmax_passed_q & 0x8000)
			batt_cell_qmax_passed_q = -((~(batt_cell_qmax_passed_q - 1)) & 0xFFFF);

		if (normal_range_judge(QMAX_MAX, QMAX_MIN, batt_cell_1_qmax) &&
				normal_range_judge(QMAX_MAX, QMAX_MIN, batt_cell_2_qmax))
			break;
		else
			usleep_range(10000, 10000);
		chg_info("qmax abnormal, retry:%d, qmax:%d, %d\n", retry,
			batt_cell_1_qmax, batt_cell_2_qmax);
	} while (retry--);

	if (retry < 0)
		return sn28z729_get_pre_qmax_parameters(chip, type);

	qmax_parameters = sn28z729_update_pre_qmax_parameters(chip, type, batt_cell_1_qmax,
		batt_cell_2_qmax, batt_cell_qmax_passed_q);

	return qmax_parameters;
}

static int sn28z729_get_battery_soc(struct chip_sn28z729 *chip)
{
	int ret;
	int soc = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return 50;

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return chip->soc_pre;

	do {
		ret = sn28z729_read_i2c(chip, chip->cmd_addr.reg_soc, &soc);
		if (ret) {
			dev_err(chip->dev, "error reading soc.\n");
			return chip->soc_pre;
		}
		if (normal_range_judge(SOC_MAX, SOC_MIN, soc))
			break;
		else
			usleep_range(10000, 10000);
		chg_err("soc abnormal, retry:%d, soc:%d\n", retry, soc);
	} while (retry--);

	if (retry < 0)
		return chip->soc_pre;

	chip->soc_pre = soc;
	return soc;
}

static int sn28z729_get_average_current(struct chip_sn28z729 *chip)
{
	int ret;
	int curr = 0;
	int retry = RETRY_CNT;

	if (!chip)
		return 0;

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return chip->current_pre;

	do {
		ret = sn28z729_read_i2c(chip, chip->cmd_addr.reg_ai, &curr);
		if (ret < 0) {
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
	} while (retry--);

	if (retry < 0)
		return chip->current_pre;

	chip->current_pre = curr;
	return curr;
}

static bool sn28z729_sha1_hmac_authenticate(struct chip_sn28z729 *chip,
					struct sn28z729_authenticate_data *authenticate_data)
{
	int i;
	int ret;
	unsigned char t;
	int len;
	u8 checksum_buf[1] = { 0x0 };
	u8 authen_cmd_buf[1] = { 0x00 };
	u8 recv_buf[AUTHEN_MESSAGE_MAX_COUNT] = { 0x0 };

	if (authenticate_data == NULL) {
		chg_err(" authenticate_data is NULL\n");
		return GAUGE_ERROR;
	}
	for (i = 0; i < authenticate_data->message_len; i++)
		checksum_buf[0] = checksum_buf[0] + authenticate_data->message[i];
	checksum_buf[0] = 0xff - (checksum_buf[0] & 0xff);
	mutex_lock(&chip->extended_cmd_access);
	ret = sn28z729_i2c_txsubcmd_onebyte(chip, DATAFLASHBLOCK, authen_cmd_buf[0]);
	if (ret < 0)
		goto err;

	ret = sn28z729_write_i2c_block(chip, AUTHENDATA, authenticate_data->message_len,
		authenticate_data->message);
	if (ret < 0)
		goto err;
	msleep(5);

	ret = sn28z729_i2c_txsubcmd_onebyte(chip, AUTHENCHECKSUM, checksum_buf[0]);
	if (ret < 0)
		goto err;
	msleep(10);

	ret = sn28z729_read_i2c_block(chip, AUTHENDATA, authenticate_data->message_len, &recv_buf[0]);
	if (ret < 0)
		goto err;
	mutex_unlock(&chip->extended_cmd_access);
	len = authenticate_data->message_len;
	for (i = 0; i < len / 2; i++) {
		t = recv_buf[i];
		recv_buf[i] = recv_buf[len - i - 1];
		recv_buf[len - i - 1] = t;
	}

	memmove(authenticate_data->message, &recv_buf[0], authenticate_data->message_len);

	return true;
err:
	chg_err("fail\n");
	mutex_unlock(&chip->extended_cmd_access);
	return false;
}

static bool sn28z729_get_smem_batt_info(oplus_gauge_auth_result *auth, int kk)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
	int ret = 0;

	ret = get_auth_msg(auth->msg, auth->rcv_msg);
	if (ret == 0)
		return true;

	return false;
#else
	size_t smem_size;
	void *smem_addr;
	oplus_gauge_auth_info_type *smem_data;

	if (NULL == auth) {
		chg_err(" invalid parameters\n");
		return false;
	}

	memset(auth, 0, sizeof(oplus_gauge_auth_result));
	smem_addr = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_RESERVED_BOOT_INFO_FOR_APPS, &smem_size);
	if (IS_ERR(smem_addr)) {
		chg_err("unable to acquire smem SMEM_RESERVED_BOOT_INFO_FOR_APPS entry\n");
		return false;
	}

	smem_data = (oplus_gauge_auth_info_type *)smem_addr;
	if (smem_data == ERR_PTR(-EPROBE_DEFER)) {
		smem_data = NULL;
		chg_info("fail to get smem_data\n");
		return false;
	}
	if (0 == kk)
		memmove(auth, &smem_data->rst_k0, sizeof(oplus_gauge_auth_result));
	else
		memmove(auth, &smem_data->rst_k1, sizeof(oplus_gauge_auth_result));

	return true;
#endif
}

static bool init_gauge_auth(struct chip_sn28z729 *chip, oplus_gauge_auth_result *rst,
				struct sn28z729_authenticate_data *authenticate_data)
{
	int len = GAUGE_AUTH_MSG_LEN < AUTHEN_MESSAGE_MAX_COUNT ? GAUGE_AUTH_MSG_LEN :
		AUTHEN_MESSAGE_MAX_COUNT;
	bool ret = false;

	if (NULL == rst || NULL == authenticate_data) {
		chg_err("Gauge authenticate failed\n");
		return false;
	}

	memset(authenticate_data, 0, sizeof(struct sn28z729_authenticate_data));
	authenticate_data->message_len = len;
	memmove(authenticate_data->message, rst->msg, len);
	ret = sn28z729_sha1_hmac_authenticate(chip, authenticate_data);
	if (!ret) {
		chg_err("get ic hmac info failed\n");
		return false;
	}

	if (memcmp(authenticate_data->message, rst->rcv_msg, len)) {
		chg_err("Gauge authenticate compare failed\n");
		return false;
	}
	chg_info("Gauge authenticate succeed\n");
	authenticate_data->result = 1;
	rst->result = 1;

	return true;
}

static bool sn28z729_get_battery_hmac(struct chip_sn28z729 *chip)
{
	int ret = false;

	if (!chip)
		return true;
	if (oplus_is_rf_ftm_mode())
		return true;

	sn28z729_get_smem_batt_info(&chip->auth_data, 1);
	if (init_gauge_auth(chip, &chip->auth_data, chip->authenticate_data))
		return true;

	sn28z729_get_smem_batt_info(&chip->auth_data, 0);
	chg_info("gauge authenticate failed, try again\n");
	ret = init_gauge_auth(chip, &chip->auth_data, chip->authenticate_data);

	return ret;
}

static bool sn28z729_get_battery_authenticate(struct chip_sn28z729 *chip)
{
	static bool get_temp = false;

	if (!chip)
		return true;

	if (!chip->temp_pre && !get_temp) {
		sn28z729_get_battery_temperature(chip);
		msleep(10);
		sn28z729_get_battery_temperature(chip);
	}

	get_temp = true;
	if (chip->temp_pre == (-400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN))
		return false;
	else
		return true;
}

static int sn28z729_get_sealed_status(struct chip_sn28z729 *chip, bool *sealed)
{
	int ret = 0;
	u8 mode;
	u8 buf[SN28Z729_SEAL_NUM_SIZE] = { 0 };

	ret = sn28z729_read_block(chip, SN28Z729_REG_OPERATION_STATUS, buf,
		SN28Z729_SEAL_NUM_SIZE, 0, false, false);
	if (ret < 0)
		return ret;

	chg_debug("operation stauts=[%*ph]\n", SN28Z729_SEAL_NUM_SIZE, buf);
	mode = (buf[SN28Z729_SEAL_NUM_SIZE - 1] & SN28Z729_SEAL_MASK_BIT);
	if (mode == SN28Z729_MODE_SEALED) {
		chg_debug("is sealed\n");
		*sealed = true;
	} else {
		chg_debug("is unsealed\n");
		*sealed = false;
	}

	return 0;
}

static bool sn28z729_set_sealed_status(struct chip_sn28z729 *chip, bool sealed)
{
	int i;
	int ret;
	bool ic_sealed = false;

	ret = sn28z729_get_sealed_status(chip, &ic_sealed);

	if (!ret && (ic_sealed == sealed)) {
		chg_debug("sealed status not need set, return\n");
		return true;
	}

	if (sealed) {
		ret = sn28z729_i2c_txsubcmd(chip, 0, SN28Z729_SEALED_SUBCMD);
		usleep_range(100000, 100000);
	} else {
		ret = sn28z729_i2c_txsubcmd(chip, 0, SN28Z729_UNSEALED_SUBCMD1);
		usleep_range(10000, 10000);
		ret |= sn28z729_i2c_txsubcmd(chip, 0, SN28Z729_UNSEALED_SUBCMD2);
		usleep_range(100000, 100000);
	}

	if (ret) {
		chg_err("sealed status set err, return\n");
		return false;
	}

	for (i = 0; i < SEAL_POLLING_RETRY_LIMIT; i++) {
		ret = sn28z729_get_sealed_status(chip, &ic_sealed);
		if (!ret && (ic_sealed == sealed)) {
			chg_debug("sealed status set success\n");
			return true;
		}
		usleep_range(10000, 10000);
	}

	chg_err("sealed status set timeout\n");
	return false;
}

static int sn28z729_set_sleep_mode_status(struct chip_sn28z729 *chip, bool enable)
{
	int ret;
	bool ic_enable = 0;
	u8 buf[SN28Z729_SLEEP_MODE_NUM_SIZE] = { 0 };

	if (enable && (sn28z729_get_firm_ver(chip) < SN28Z729_SUPPORT_ENABLE_SLEEP_MODE_VERSION)) {
		chg_err("sn28z729_set_sleep_mode_status: not support enable it!\n");
		return 0;
	}

	ret = sn28z729_read_block(chip, SN28Z729_REG_DA_CFG, buf, SN28Z729_SLEEP_MODE_NUM_SIZE,
		0, false, false);
	if (ret < 0)
		return ret;

	if (buf[0] & SN28Z729_SLEEP_MODE_MASK_BIT)
		ic_enable = true;
	else
		ic_enable = false;

	if (ic_enable == enable) {
		chg_debug("sleep mode status not need set, return\n");
		return 0;
	}

	chg_info("set sleep_mode:%d\n", enable);

	if (enable)
		buf[0] |= SN28Z729_SLEEP_MODE_MASK_BIT;
	else
		buf[0] &= ~SN28Z729_SLEEP_MODE_MASK_BIT;

	ret = sn28z729_write_block(chip, SN28Z729_REG_DA_CFG, buf, SN28Z729_SLEEP_MODE_NUM_SIZE,
		0, false, SN28Z729_SLEEP_MODE_NUM_SIZE, false);
	msleep(10);

	return ret;
}

static int sn28z729_update_sleep_mode_status(struct chip_sn28z729 *chip, bool enable)
{
	bool rc;
	int ret;
	int try_count = 1;

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return -EINVAL;

	mutex_lock(&chip->extended_cmd_access);
	rc = sn28z729_set_sealed_status(chip, false);
	if (!rc) {
		mutex_unlock(&chip->extended_cmd_access);
		return GAUGE_ERROR;
	}

	do {
		ret = sn28z729_set_sleep_mode_status(chip, enable);
	} while (ret < 0 && try_count--);

	usleep_range(1000, 1000);
	rc = sn28z729_set_sealed_status(chip, true);
	mutex_unlock(&chip->extended_cmd_access);

	if (!rc || ret < 0) {
		chg_err("fail\n");
		return GAUGE_ERROR;
	}

	return 0;
}

static void sn28z729_set_cmd_addr(struct chip_sn28z729 *chip)
{
	chip->cmd_addr.reg_temp = SN28Z729_REG_TEMP;
	chip->cmd_addr.reg_volt = SN28Z729_REG_VOLT;
	chip->cmd_addr.reg_flags = SN28Z729_REG_FLAGS;
	chip->cmd_addr.reg_nac = SN28Z729_REG_NAC;
	chip->cmd_addr.reg_rm = SN28Z729_REG_RM;
	chip->cmd_addr.reg_fcc = SN28Z729_REG_FCC;
	chip->cmd_addr.reg_ai = SN28Z729_REG_TI;
	chip->cmd_addr.reg_soc = SN28Z729_REG_SOC;
	chip->cmd_addr.reg_soh = SN28Z729_REG_SOH;
	chip->cmd_addr.reg_cc = SN28Z729_REG_CC;
}

__maybe_unused static int sn28z729_gauge_get_firm_version(struct chip_sn28z729 *chip,
								int *firm_version)
{
	int status = 0;
	u8 cntl1_val[GAUGE_GET_DEVICE_FIRM_VER_LEN] = { 0 };
	unsigned short build_num = 0;

	mutex_lock(&chip->extended_cmd_access);
	sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, GAUGE_GET_DEVICE_FIRM_VER_CMD);
	msleep(10);
	status = sn28z729_read_i2c_block(chip,
			SN28Z729_DATA_FLASH_BLOCK, GAUGE_GET_DEVICE_FIRM_VER_LEN, cntl1_val);
	chg_info("get chip id [0x%x 0x%x 0x%x 0x%x] firm_version[0x%x,0x%x] "
		"build_number[0x%x,0x%x]\n", cntl1_val[0], cntl1_val[1], cntl1_val[2], cntl1_val[3],
		cntl1_val[4], cntl1_val[5], cntl1_val[6], cntl1_val[7]);
	*firm_version = ((cntl1_val[4] & 0xf0) >> 4) * 1000 + (cntl1_val[4] & 0xf) * 100 +
			((cntl1_val[5] & 0xf0) >> 4) * 10 + (cntl1_val[5] & 0xf);
	chip->firm_ver = *firm_version;
	build_num = ((cntl1_val[6] & 0xf0) >> 4) * 1000 + (cntl1_val[6] & 0xf) * 100 +
			((cntl1_val[7] & 0xf0) >> 4) * 10 + (cntl1_val[7] & 0xf);
	mutex_unlock(&chip->extended_cmd_access);

	chg_info("firm_version = %4d, build_num = %4d\n", *firm_version, build_num);

	return status;
}

static int sn28z729_get_firm_ver(struct chip_sn28z729 *chip)
{
	if (chip)
		return chip->firm_ver;
	else
		return 0;
}

static int sn28z729_get_cuv_state(struct chip_sn28z729 *chip, int *value)
{
	int status = 0;
	unsigned char cuv_state[2] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return 0;

	if (!value)
		return -EINVAL;

	status = sn28z729_read_block(chip, SN28Z729_GET_CUV_ADDR, cuv_state, 1,
			SN28Z729_GET_CUV_OFFSET, false, true);
	if (status) {
		chg_err("failed, status = %d\n", status);
		return status;
	}
	*value = ((cuv_state[0] & SN28Z729_GET_CUV_MASK) >> SN28Z729_GET_CUV_BIT);

	return 0;
}

static int sn28z729_set_cuv_state(struct chip_sn28z729 *chip, unsigned short state)
{
	int status = 0;

	if (is_chip_suspended_or_locked(chip))
		return 0;

	status = sn28z729_write_block(chip, SN28Z729_REG_CUV_STATE_ADDR, (u8 *)&state,
		sizeof(state), 0, false, SN28Z729_REG_CUV_STATE_LEN, true);

	if (status < 0)
		chg_err("set to state[%d] failed, status = 0x%x\n", state, status);
	else
		chg_info("set to state[%d] success\n", state);

	return status;
}

static int sn28z729_init_cv_state(struct chip_sn28z729 *chip)
{
	int cuv_state = 0;
	int status = 0;

	status = sn28z729_get_cuv_state(chip, &cuv_state);
	if (status < 0)
		chg_err("failed, status = 0x%x\n", status);
	else
		chg_debug("cuv_state = 0x%x\n", cuv_state);

	if (cuv_state != SN28Z729_CUV_STATE_CUV2) {
		/* change the gauge to CUV2 when bootup.*/
		status = sn28z729_set_cuv_state(chip, SN28Z729_CUV_STATE_CUV2);
		if (status < 0)
			chg_err("set_cuv_state failed, status = 0x%x\n", status);
	}
	return status;
}

static int sn28z729_get_device_type(struct chip_sn28z729 *chip, int *device_type)
{
	int ret;
	u8 buf[SN28Z729_DEVICE_TYPE_NUM_SIZE] = { 0 };

	ret = sn28z729_read_block(chip, SN28Z729_REG_DEVICE_TYPE, buf,
		SN28Z729_DEVICE_TYPE_NUM_SIZE, 0, false, true);
	if (ret < 0)
		return ret;

	*device_type = (buf[1] << 0x8) | buf[0];

	return 0;
}

__maybe_unused static int sn28z729_clear_car_c_param(struct chip_sn28z729 *chip)
{
	int status = 0;

	if (is_chip_suspended_or_locked(chip))
		return 0;

	mutex_lock(&chip->extended_cmd_access);
	status = sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, SN28Z729_CLEAR_CAR_C_CMD);
	mutex_unlock(&chip->extended_cmd_access);

	return status;
}

static void sn28z729_hw_config(struct chip_sn28z729 *chip)
{
	int ret;
	int device_type = 0;
	int firm_ver = 0;

	sn28z729_set_cmd_addr(chip);

	ret = sn28z729_get_device_type(chip, &device_type);
	if (ret)
		chip->probe_err = true;
	ret = sn28z729_gauge_get_firm_version(chip, &firm_ver);

	chip->device_type = DEVICE_SN28Z729;
	sn28z729_get_battery_fcc(chip);
	sn28z729_init_cv_state(chip);
	sn28z729_clear_car_c_param(chip);
	chg_info("device type is 0x%02x, firm_ver = %x\n", device_type, firm_ver);
}

static int oplus_sn28z729_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is null\n");
		return -ENODEV;
	}

	ic_dev->online = true;
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip)
		return -ENODEV;

	if (chip->probe_err) {
		chip->probe_err = false;
		sn28z729_hw_config(chip);
	}

	return 0;
}

static int oplus_sn28z729_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;

	return 0;
}

static void sn28z729_parse_dt(struct chip_sn28z729 *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;

	rc = of_property_read_u32(node, "oplus,batt_num", &chip->batt_num);
	if (rc < 0) {
		chg_err("can't get oplus, batt_num, rc=%d\n", rc);
		chip->batt_num = 1;
	}

	rc = of_property_read_u32(node, "qcom,gauge_num", &chip->gauge_num);
	if (rc)
		chip->gauge_num = 0;
	chip->i2c_rst_ext = of_property_read_bool(node, "oplus,i2c_rst_ext");
	chip->support_eco_design = !!(oplus_chg_get_nvid_support_flags() &
	BIT(ECO_DESIGN_SUPPORT_REGION));
}

static int sn28z729_get_deep_term_volt(struct chip_sn28z729 *chip)
{
	int ret;
	int volt;
	u8 buf[SN28Z729_DEEP_VOLT_NUM_SIZE] = { 0 };

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return chip->deep_term_volt_pre;

	ret = sn28z729_read_block(chip, SN28Z729_REG_OPLUS_DATA, buf, SN28Z729_DEEP_VOLT_NUM_SIZE,
				SN28Z729_DEEP_VOLT_OFFSET, true, true);
	if (ret < 0) {
		chg_err("get the deep term volt failed, ret = %d\n", ret);
		return chip->deep_term_volt_pre;
	}

	volt = (buf[1] << 0x8) | buf[0];
	chip->deep_term_volt_pre = volt;

	return volt;
}

static int sn28z729_get_deep_count(struct chip_sn28z729 *chip)
{
	int ret;
	int count;
	u8 buf[SN28Z729_DEEP_COUNT_NUM_SIZE] = { 0 };

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return chip->deep_count_pre;

	ret = sn28z729_read_block(chip, SN28Z729_REG_OPLUS_DATA, buf, SN28Z729_DEEP_COUNT_NUM_SIZE,
					SN28Z729_DEEP_COUNT_OFFSET, true, true);
	if (ret < 0)
		return chip->deep_count_pre;

	count = (buf[1] << 8) | buf[0];

	chip->deep_count_pre = count;

	return count;
}

static int sn28z729_set_deep_count(struct chip_sn28z729 *chip, int count)
{
	int ret;
	u8 buf[SN28Z729_DEEP_COUNT_NUM_SIZE] = { 0 };

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return -EINVAL;

	buf[0] = count & 0xff;
	buf[1] = (count >> 8) & 0xff;

	ret = sn28z729_write_block(chip, SN28Z729_REG_OPLUS_DATA, buf, SN28Z729_DEEP_COUNT_NUM_SIZE,
				SN28Z729_DEEP_COUNT_OFFSET, true, SN28Z729_BLOCK_SIZE, true);
	if (ret < 0)
		return ret;

	return 0;
}

static int sn28z729_set_deep_last_cc(struct chip_sn28z729 *chip, int cc)
{
	int ret;
	u8 buf[SN28Z729_DEEP_LAST_CC_NUM_SIZE] = { 0 };

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return GAUGE_ERROR;

	buf[0] = cc & 0xff;
	buf[1] = (cc >> 8) & 0xff;

	ret = sn28z729_write_block(chip, SN28Z729_REG_OPLUS_DATA, buf,
		SN28Z729_DEEP_LAST_CC_NUM_SIZE, SN28Z729_DEEP_LAST_CC_OFFSET, true,
		SN28Z729_BLOCK_SIZE, true);
	if (ret < 0)
		return ret;

	return 0;
}

static int sn28z729_get_deep_last_cc(struct chip_sn28z729 *chip)
{
	int ret;
	int cc;
	u8 buf[SN28Z729_DEEP_LAST_CC_NUM_SIZE] = { 0 };

	if (atomic_read(&chip->suspended) || atomic_read(&chip->locked))
		return chip->deep_last_cc_pre;

	ret = sn28z729_read_block(chip, SN28Z729_REG_OPLUS_DATA, buf,
		SN28Z729_DEEP_LAST_CC_NUM_SIZE, SN28Z729_DEEP_LAST_CC_OFFSET, true, true);
	if (ret < 0)
		return chip->deep_last_cc_pre;

	cc = (buf[1] << 0x8) | buf[0];
	chip->deep_last_cc_pre = cc;

	return cc;
}

static int oplus_sn28z729_get_deep_count(struct oplus_chg_ic_dev *ic_dev, int *count)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*count = sn28z729_get_deep_count(chip);

	return 0;
}

static int oplus_sn28z729_set_deep_count(struct oplus_chg_ic_dev *ic_dev, int count)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	sn28z729_set_deep_count(chip, count);

	return 0;
}

static int sn28z729_set_term_volt_in_deep_info(struct chip_sn28z729 *chip, int term_param)
{
	unsigned char level = 0;
	unsigned short volt_mv = 0;
	unsigned char extend_data[SN28Z729_TERM_VLOT_LEN + 2] = { 0 };
	int status = 0;

	/* get the term volt level from input param, bit 24 ~ bit32*/
	level = ((term_param >> 24) & 0xff);

	/* update the term volt */
	extend_data[level * 2] = (2 * volt_mv & 0xff);
	extend_data[level * 2 + 1] = ((2 * volt_mv >> 8) & 0xff);
	extend_data[level * 2 + SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET] = (term_param & 0xff);
	extend_data[level * 2 + SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET + 1] =
		((term_param >> 8) & 0xff);

	/* get the term volt level from input param, bit 0 ~ bit15*/
	volt_mv = (term_param & 0xffff);
	chg_info("level[%d], volt_mv=%d\n", level, volt_mv);
	if (level == 0) {
		status = sn28z729_write_block(chip, SN28Z729_REG_OPLUS_DATA,
						&extend_data[SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET],
						SN28Z729_DEEP_VOLT_NUM_SIZE, SN28Z729_DEEP_VOLT_OFFSET,
						true, SN28Z729_BLOCK_SIZE, true);
		if (status < 0) {
			chg_err("read term param failed, status=%d\n", status);
			return status;
		}
	}

	return status;
}

static int sn28z729_set_term_volt_before_firm_ver_9(struct chip_sn28z729 *chip, int volt_mv)
{
	int status = 0;
	u8 extend_data[SN28Z729_TERM_VLOT_BEFORE_VER_9_LEN + 2] = { 0 };
	u8 temp_data[SN28Z729_TERM_VLOT_BEFORE_VER_9_LEN + 2] = { 0 };
	int term_volt = 0;
	int second_term_volt = 0;
	int term_min_cel = 0;

	/* read back the term param. */
	status = sn28z729_read_block(chip, GAUGE_SET_TERM_PARAM_CMD, extend_data,
		SN28Z729_TERM_VLOT_BEFORE_VER_9_LEN, 0, 0, true);
	if (status < 0) {
		chg_err("read term param failed, status=%d\n", status);
		return GAUGE_ERROR;
	}

	term_volt = ((extend_data[1] << 8) | extend_data[0]);
	second_term_volt = ((extend_data[3] << 8) | extend_data[2]);
	term_min_cel = ((extend_data[5] << 8) | extend_data[4]);
	chg_info("read term param "
		 "buf[0x%x,0x%x,0x%x,0x%x,0x%x,0x%x],term_volt=%d,second_term_volt=%"
		 "d,term_min_cel=%d.",
		extend_data[0], extend_data[1], extend_data[2], extend_data[3],
		extend_data[4], extend_data[5], term_volt, second_term_volt, term_min_cel);

	/* update the term volt */
	extend_data[0] = (2 * volt_mv & 0xff);
	extend_data[1] = ((2 * volt_mv >> 8) & 0xff);
	extend_data[4] = (volt_mv & 0xff);
	extend_data[5] = ((volt_mv >> 8) & 0xff);
	chg_info("update term param "
		"buf[0]=0x%x,buf[1]=0x%x,buf[4]=0x%x,buf[5]=0x%x,volt_mv=%d mV.",
		extend_data[0], extend_data[1], extend_data[4], extend_data[5], volt_mv);

	status = sn28z729_write_block(chip, GAUGE_SET_TERM_PARAM_CMD, (u8 *)&(extend_data[0]),
					SN28Z729_TERM_VLOT_BEFORE_VER_9_LEN, 0, false,
					SN28Z729_TERM_VLOT_BEFORE_VER_9_LEN, true);

	if (status < 0) {
		chg_err("update term param failed, status=%d\n", status);
		return GAUGE_ERROR;
	}

	/* read back the term param to check update success ? */
	status = sn28z729_read_block(chip, GAUGE_SET_TERM_PARAM_CMD, temp_data,
		SN28Z729_TERM_VLOT_BEFORE_VER_9_LEN, 0, false, true);
	if (status < 0) {
		chg_err("read term param failed, status=%d.", status);
		return GAUGE_ERROR;
	}
	if (temp_data[5] == ((volt_mv >> 8) & 0xff) && temp_data[4] == (volt_mv & 0xff))
		chg_debug("check the term param is updated success.");
	else
		chg_err("check the term param is updated failed, [0x%x 0x%x 0x%x 0x%x], "
				"RSOC=0x%x.", temp_data[0], temp_data[1], temp_data[4],
				temp_data[5], temp_data[9]);
	return status;
}

static int sn28z729_set_term_volt(struct chip_sn28z729 *chip, int term_param)
{
	int status = 0;
	unsigned char extend_data[SN28Z729_TERM_VLOT_LEN + 2] = { 0 };
	unsigned char temp_data[SN28Z729_TERM_VLOT_LEN + 2] = { 0 };
	int term_volt = 0;
	int term_min_cel = 0;
	unsigned char level = 0;
	unsigned short volt_mv = 0;

	if (is_chip_suspended_or_locked(chip))
		return 0;
	status = sn28z729_read_block(chip, GAUGE_SET_TERM_PARAM_CMD, extend_data,
		SN28Z729_TERM_VLOT_LEN, 0, 0, true);
	if (status < 0) {
		chg_err("read term param failed, status=%d \n", status);
		return status;
	}
	level = ((term_param >> 24) & 0xff);
	volt_mv = (term_param & 0xffff);
	if (level > SN28Z729_GAUGE_TERM_VOLT_LEVEL_THIRD || level <
		SN28Z729_GAUGE_TERM_VOLT_LEVEL_FIRST) {
		return -EINVAL;
	}
	term_volt = ((extend_data[level * 2 + 1] << 8) | extend_data[level * 2]);
	term_min_cel = ((extend_data[level * 2 + SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET + 1] << 8) |
			extend_data[level * 2 + SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET]);
	extend_data[level * 2] = (2 * volt_mv & 0xff);
	extend_data[level * 2 + 1] = ((2 * volt_mv >> 8) & 0xff);
	extend_data[level * 2 + SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET] = (volt_mv & 0xff);
	extend_data[level * 2 + SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET + 1]
				= ((volt_mv >> 8) & 0xff);
	status = sn28z729_write_block(chip, GAUGE_SET_TERM_PARAM_CMD, (u8 *)&(extend_data[0]),
		SN28Z729_TERM_VLOT_LEN, 0, false, SN28Z729_TERM_VLOT_LEN, true);
	if (status) {
		chg_err("update %d term param failed, status=%d.\n", level, status);
		return status;
	}
	status = sn28z729_read_block(chip, GAUGE_SET_TERM_PARAM_CMD, temp_data,
			SN28Z729_TERM_VLOT_LEN, 0, false, true);
	if (status)
		return status;

	if (temp_data[level * 2 + SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET + 1]
		!= ((volt_mv >> 8) & 0xff) || temp_data[level * 2 +
		SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET] != (volt_mv & 0xff)) {
		chg_err("check the %d term param is updated failed, [%x %x %x %x]\n",
			level, temp_data[level * 2], temp_data[level * 2 + 1],
			temp_data[level * 2 + SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET],
			temp_data[level * 2 + SN28Z729_GAUGE_MIN_CELL_VOLTAGE_OFFSET + 1]);
	}
	status = sn28z729_set_term_volt_in_deep_info(chip, term_param);
	return status;
}

#define REG_DUMP_SIZE 1024
static int dump_reg[] = { 0x08, 0x12, 0x2c };

static void sn28z729_it_status1_dump(struct chip_sn28z729 *chip, int len_sus_pwr, u8 *iv,
		int len_max, char *pos, int *sum, int *l)
{
	int i = 0;
	sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, SN28Z729_REG_IT_STATUS1);
	usleep_range(10000, 10000);
	sn28z729_read_i2c_block(chip, SN28Z729_DATA_FLASH_BLOCK, len_sus_pwr, iv);
	for (i = 2; (i < len_sus_pwr) && (*sum < len_max); i++) {
		if ((i % 2) == 0) {
			*l = snprintf(pos, REG_DUMP_SIZE - 1, "/ %d ", (iv[i + 1] << 8) + iv[i]);
			pos += *l;
			*sum += *l;
		}
	}
}

static void sn28z729_it_status2_dump(struct chip_sn28z729 *chip, int len_max_pwr, u8 *iv,
		int len_max, char *pos, int *sum, int *l)
{
	int i = 0;
	sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, SN28Z729_REG_IT_STATUS2);
	usleep_range(10000, 10000);
	sn28z729_read_i2c_block(chip, SN28Z729_DATA_FLASH_BLOCK, len_max_pwr, iv);
	for (i = 2; (i < len_max_pwr) && (*sum < len_max); i++) {
		if (i != 3 && i != 4 && i != 7 && i != 8 && i != 11 && i != 12) {
			if ((i % 2) == 0) {
				*l = snprintf(pos, REG_DUMP_SIZE - 1, "/ %d ",
					(iv[i + 1] << 8) + iv[i]);
				pos += *l;
				*sum += *l;
			}
		}
	}
}

static void sn28z729_it_status3_dump(struct chip_sn28z729 *chip, int len_sus_curr_h, u8 *iv,
		int len_max, char *pos, int *sum, int *l)
{
	int i = 0;
	sn28z729_i2c_txsubcmd(chip, SN28Z729_DATA_FLASH_BLOCK, SN28Z729_REG_IT_STATUS3);
	usleep_range(10000, 10000);
	sn28z729_read_i2c_block(chip, SN28Z729_DATA_FLASH_BLOCK, len_sus_curr_h, iv);
	for (i = 12; (i < len_sus_curr_h) && (*sum < len_max); i++) {
		if (i != 17 && i != 18) {
			if ((i % 2) == 0) {
				*l = snprintf(pos, REG_DUMP_SIZE - 1, "/ %d ",
					(iv[i + 1] << 8) + iv[i]);
				pos += *l;
				*sum += *l;
			}
		}
	}
}

static int sn28z729_gauge_reg_dump(struct chip_sn28z729 *chip)
{
	int val = 0;
	int i;
	int l = 0;
	char *pos;
	int sum = 0, ret;
	u8 iv[32] = { 0 };
	char buf[REG_DUMP_SIZE] = { 0 };
	int len_max = REG_DUMP_SIZE - 16;
	int len_sus_pwr = 6;
	int len_max_pwr = 16;
	int len_sus_curr_h = 26;
	bool read_done = false;

	if (!chip)
		return 0;

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	pos = buf;

	l = snprintf(pos, REG_DUMP_SIZE - 1, "%d ", sn28z729_get_battery_temperature(chip));
	pos += l;
	sum += l;
	l = snprintf(pos, REG_DUMP_SIZE - 1, "/ %d ", sn28z729_get_average_current(chip));
	pos += l;
	sum += l;

	for (i = 0; !read_done; i++) {
		ret = sn28z729_read_i2c(chip, dump_reg[i], &val);
		if (ret) {
			chg_err("error reading regdump, ret:%d\n", ret);
			return -EINVAL;
		}
		l = snprintf(pos, REG_DUMP_SIZE - 1, "/ %d ", val);
		pos += l;
		sum += l;

		read_done = !(i < sizeof(dump_reg) / sizeof(int));
		read_done &= !(sum < len_max);
	}
	mutex_lock(&chip->extended_cmd_access);
	sn28z729_it_status1_dump(chip, len_sus_pwr, iv, len_max, pos, &sum, &l);
	sn28z729_it_status2_dump(chip, len_max_pwr, iv, len_max, pos, &sum, &l);
	sn28z729_it_status3_dump(chip, len_sus_curr_h, iv, len_max, pos, &sum, &l);
	mutex_unlock(&chip->extended_cmd_access);
	chg_err("gauge regs: %s\n", buf);
	return 0;
}

static int oplus_sn28z729_set_deep_term_volt(struct oplus_chg_ic_dev *ic_dev, int volt_mv)
{
	struct chip_sn28z729 *chip;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ret = sn28z729_set_term_volt_in_deep_info(chip, volt_mv);
	if (sn28z729_get_firm_ver(chip) >= SN28Z729_NEED_TO_SUPPORT_THIRD_TERM_VOLT_FIRM_VER)
		ret = sn28z729_set_term_volt(chip, volt_mv);
	else
		ret = sn28z729_set_term_volt_before_firm_ver_9(chip, volt_mv);

	return ret;
}

static int oplus_sn28z729_get_deep_term_volt(struct oplus_chg_ic_dev *ic_dev, int *volt)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*volt = sn28z729_get_deep_term_volt(chip);

	return 0;
}

static int oplus_sn28z729_set_deep_last_cc(struct oplus_chg_ic_dev *ic_dev, int cc)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	sn28z729_set_deep_last_cc(chip, cc);

	return 0;
}

static int oplus_sn28z729_get_deep_last_cc(struct oplus_chg_ic_dev *ic_dev, int *cc)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*cc = sn28z729_get_deep_last_cc(chip);

	return 0;
}

static int sn28z729_get_info(struct chip_sn28z729 *chip, u8 *info, int len)
{
	int i;
	int j;
	int ret;
	int data;
	int index = 0;
	u8 buf[SN28Z729_BLOCK_SIZE] = { 0 };
	struct sn28z729_block_access standard[] = {
		{ chip->cmd_addr.reg_temp, 2 }, { chip->cmd_addr.reg_volt, 2 },
		{ chip->cmd_addr.reg_flags, 2 }, { chip->cmd_addr.reg_nac, 2 },
		{ chip->cmd_addr.reg_rm, 2 }, { chip->cmd_addr.reg_fcc, 2 },
		{ chip->cmd_addr.reg_ai, 2 }, { chip->cmd_addr.reg_cc, 2 },
		{ chip->cmd_addr.reg_soc, 2 }, { chip->cmd_addr.reg_soh, 2 },
	};

	struct sn28z729_block_access extend[] = {
		{ SN28Z729_REG_CHEMID, 2 }, { SN28Z729_REG_GAUGEING_STATUS, 3 },
		{ SN28Z729_REG_DA_STATUS1, 12 }, { SN28Z729_REG_IT_STATUS1, 16 },
		{ SN28Z729_REG_IT_STATUS2, 24 }, { SN28Z729_REG_IT_STATUS3, 16 },
		{ SN28Z729_REG_CB_STATUS, 8 },
	};

	/*standard register packaging*/
	for (i = 0; i < ARRAY_SIZE(standard); i++) {
		ret = sn28z729_read_i2c(chip, standard[i].addr, &data);
		if (ret < 0)
			continue;
		index += snprintf(info + index, len - index, "0x%02x=%02x,%02x|", standard[i].addr,
			(data & 0xff), ((data >> 8) & 0xff));
	}

	/*extended register packaging*/
	for (i = 0; i < ARRAY_SIZE(extend); i++) {
		ret = sn28z729_read_block(chip, extend[i].addr, buf, extend[i].len, 0, false, true);
		if (ret < 0)
			continue;

		index += snprintf(info + index, len - index, "0x%04x=", extend[i].addr);
		for (j = 0; j < extend[i].len - 1; j++)
			index += snprintf(info + index, len - index, "%02x,", buf[j]);
		index += snprintf(info + index, len - index, "%02x", buf[j]);

		if (i < ARRAY_SIZE(extend) - 1) {
			index += snprintf(info + index, len - index, "|");
			usleep_range(500, 500);
		}
	}

	return index;
}

static int sn28z729_get_lifetime_info(struct chip_sn28z729 *chip, u8 *info, int len)
{
	int i;
	int j;
	int ret;
	int index = 0;
	u8 buf[SN28Z729_BLOCK_SIZE] = { 0 };
	struct sn28z729_block_access extend[] = {
		{ SN28Z729_REG_STATIC_DF_SIG, 2 }, { SN28Z729_REG_STATIC_CHEM_DF_SIG, 2 },
		{ GAUGE_SET_TERM_PARAM_CMD, SN28Z729_TERM_VLOT_LEN },
		{ SN28Z729_REG_LIFETIME_ADDR, 10 }, { SN28Z729_REG_QMAX_UPDATE_CONDTION, 5 },
		{ SN28Z729_REG_IPM_1_ADDR, 32 }, { SN28Z729_REG_IPM_2_ADDR, 32 },
		{ SN28Z729_REG_OPLUS_DATA, SN28729_DEEP_SHIFT + 1 },
	};

	/* extended register packaging */
	for (i = 0; i < ARRAY_SIZE(extend); i++) {
		ret = sn28z729_read_block(chip, extend[i].addr, buf, extend[i].len, 0, false, true);
		if (ret < 0)
			continue;

		index += snprintf(info + index, len - index, "0x%04x=", extend[i].addr);
		for (j = 0; j < extend[i].len - 1; j++) {
			index += snprintf(info + index, len - index, "%02x,", buf[j]);
			if (index >= len - 1) {
				chg_err("index[%d] > len = [%d]-1.\n", index, len);
				break;
			}
		}
		if (index >= len - 1) {
			chg_err("index[%d] > len = [%d]-1.\n", index, len);
			break;
		}
		index += snprintf(info + index, len - index, "%02x", buf[j]);
		if (index >= len - 1) {
			chg_err("index[%d] > len = [%d]-1.\n", index, len);
			break;
		}

		if (i < ARRAY_SIZE(extend) - 1) {
			index += snprintf(info + index, len - index, "|");
			usleep_range(500, 500);
		}
	}

	return index;
}

static int oplus_sn28z729_get_reg_info(struct oplus_chg_ic_dev *ic_dev, u8 *info, int len)
{
	int index;
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || !info) {
		chg_err("oplus_chg_ic_dev or info is null");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (is_chip_suspended_or_locked(chip))
		return GAUGE_ERROR;

	index = sn28z729_get_info(chip, info, len);
	return index;
}

static void update_calib_time(struct chip_sn28z729 *chip, const int *check_args)
{
	if (check_args[0] != chip->calib_check_args_pre[0] || check_args[3] !=
			chip->calib_check_args_pre[3])
		chip->dod_time = 1;
	else
		chip->dod_time++;

	if (check_args[1] != chip->calib_check_args_pre[1] || check_args[2] !=
		chip->calib_check_args_pre[2] || check_args[4] != chip->calib_check_args_pre[4]
			|| check_args[5] != chip->calib_check_args_pre[5])
		chip->qmax_time = 1;
	else
		chip->qmax_time++;
}
static int sn28z729_get_calib_time(struct chip_sn28z729 *chip, int *dod_calib_time,
		int *qmax_calib_time)
{
	int ret;
	u8 buf[SN28Z729_BLOCK_SIZE] = { 0 };
	int check_args[CALIB_TIME_CHECK_ARGS] = { 0 };

	ret = sn28z729_read_block(chip, SN28Z729_REG_IT_STATUS2, buf, 14, 0, false, true);
	if (ret < 0)
		return ret;

	check_args[0] = (buf[11] << 0x08) | buf[10];
	check_args[3] = (buf[13] << 0x08) | buf[12];

	ret = sn28z729_read_block(chip, SN28Z729_REG_IT_STATUS3, buf, 8, 0, false, true);
	if (ret < 0)
		return ret;

	check_args[1] = (buf[1] << 0x08) | buf[0];
	check_args[2] = (buf[5] << 0x08) | buf[4];
	check_args[4] = (buf[3] << 0x08) | buf[2];
	check_args[5] = (buf[7] << 0x08) | buf[6];
	update_calib_time(chip, check_args);
	memmove(chip->calib_check_args_pre, check_args, sizeof(check_args));
	*dod_calib_time = chip->dod_time;
	*qmax_calib_time = chip->qmax_time;

	return ret;
}

static void oplus_sn28z729_calib_args_to_check_args(struct chip_sn28z729 *chip, char *calib_args,
						int len)
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

static void oplus_sn28z729_check_args_to_calib_args(struct chip_sn28z729 *chip, char *calib_args,
				int len)
{
	int i;
	int j;

	if (len != (CALIB_TIME_CHECK_ARGS * 2))
		return;

	for (i = 0, j = 0; i < CALIB_TIME_CHECK_ARGS; i++, j += 2) {
		calib_args[j] = chip->calib_check_args_pre[i] & 0xff;
		calib_args[j + 1] = (chip->calib_check_args_pre[i] >> 0x8) & 0xff;
		chg_debug("calib_args[%d]=0x%02x, 0x%02x\n", i, calib_args[j], calib_args[j + 1]);
	}
}

static int oplus_sn28z729_get_calib_time(struct oplus_chg_ic_dev *ic_dev, int *dod_calib_time,
			int *qmax_calib_time, char *calib_args, int len)
{
	int ret;
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null\n");
		return -ENODEV;
	}

	if (calib_args == NULL || dod_calib_time == NULL || qmax_calib_time == NULL || !len)
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
	ret = sn28z729_get_calib_time(chip, dod_calib_time, qmax_calib_time);
	if (ret < 0) {
		*dod_calib_time = chip->dod_time_pre;
		*qmax_calib_time = chip->qmax_time_pre;

		mutex_unlock(&chip->calib_time_mutex);
		return 0;
	}

	oplus_sn28z729_check_args_to_calib_args(chip, calib_args, len);
	chip->dod_time_pre = *dod_calib_time;
	chip->qmax_time_pre = *qmax_calib_time;
	mutex_unlock(&chip->calib_time_mutex);
	return 0;
}

static int oplus_sn28z729_set_calib_time(struct oplus_chg_ic_dev *ic_dev, int dod_calib_time,
	int qmax_calib_time, char *calib_args, int len)
{
	struct chip_sn28z729 *chip;

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
	oplus_sn28z729_calib_args_to_check_args(chip, calib_args, len);
	chip->calib_info_init = true;

	return 0;
}

/*
 * Every time the power is turned on, send the start command 0x008A to 0x3e,
 * and the integrator will start recording the capacity of current * time from
 * zero
*/

static int sn28z729_get_car_c_param(struct chip_sn28z729 *chip, int *value)
{
	int status = 0;
	unsigned char data[6] = { 0 };
	unsigned short car_temp = 0;

	if (!value)
		return -EINVAL;

	if (is_chip_suspended_or_locked(chip))
		return 0;

	status = sn28z729_read_block(chip, SN28Z729_GET_CAR_C_ADDR, data, 6, 0, false, true);
	if (status) {
		chg_err("status = %d failed\n", status);
		return status;
	}

	car_temp = ((data[1] << 0x08) | data[0]);

	/*
	* change to signed data, unit is mAh.
	* the complement code is converted to the original code,the symbol remains
	* unchanged.
	*/
	if (car_temp & 0x8000)
		*value = -((~((car_temp & 0x7FFF) - 1)) & 0x7FFF);
	else
		*value = car_temp;

	chg_debug("[0x%x,0x%x,0x%x,0x%x,0x%x,0x%x],car_temp:0x%x,value:%d\n", data[0], data[1],
		data[2], data[3], data[4], data[5], car_temp, *value);

	return status;
}

static int oplus_sn28z729_get_car_c(struct oplus_chg_ic_dev *ic_dev, int *value)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL)
		return -ENODEV;

	if (value == NULL)
		return -EINVAL;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	return sn28z729_get_car_c_param(chip, value);
}

static int sn28z729_get_cyclic_decrease_voltage(struct chip_sn28z729 *chip, int *data)
{
	int ret = 0;
	u8 value = 0;

	if (is_chip_suspended_or_locked(chip))
		return 0;

	mutex_lock(&chip->extended_cmd_access);
	if (!sn28z729_deep_init(chip)) {
		mutex_unlock(&chip->extended_cmd_access);
		return GAUGE_ERROR;
	}
	ret = sn28z729_read_block(chip, SN28729_SUBCMD_CYCLIC_DECREASE_VOLTAGE_ADDR, (u8 *)&value,
				SN28729_SUBCMD_CYCLIC_DECREASE_VOLTAGE_LEN, 0, false, false);
	if (ret < 0) {
		mutex_unlock(&chip->extended_cmd_access);
		return GAUGE_ERROR;
	}

	chg_debug("vct = 0x%x, status = 0x%x\n", value, ret);
	*data = value;
	sn28z729_deep_deinit(chip);
	mutex_unlock(&chip->extended_cmd_access);

	return ret;
}

static int oplus_sn28z729_get_vct(struct oplus_chg_ic_dev *ic_dev, int *value)
{
	struct chip_sn28z729 *chip;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_info("ic_dev == NULL");
		return -ENODEV;
	}
	if (value == NULL) {
		chg_info("value == NULL");
		return -EINVAL;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ret = sn28z729_get_cyclic_decrease_voltage(chip, value);

	if (ret != GAUGE_OK) {
		chg_info("get the vct failed, ret = %d\n", *value);
		return -EINVAL;
	}

	return ret;
}

static int sn28z729_set_cyclic_decrease_voltage(struct chip_sn28z729 *chip, int data)
{
	int status = 0;
	u16 value = (unsigned short)(data & 0xffff);

	if (is_chip_suspended_or_locked(chip))
		return 0;

	mutex_lock(&chip->extended_cmd_access);
	if (!sn28z729_deep_init(chip)) {
		mutex_unlock(&chip->extended_cmd_access);
		return GAUGE_ERROR;
	}
	status = sn28z729_write_block(chip, SN28729_SUBCMD_CYCLIC_DECREASE_VOLTAGE_ADDR,
		(u8 *)&value, sizeof(value), 0, false, sizeof(value), false);
	if (status < 0)
		chg_err(" value = 0x%x failed, status = 0x%x\n", value, status);

	sn28z729_deep_deinit(chip);
	mutex_unlock(&chip->extended_cmd_access);

	return status;
}

static int oplus_sn28z729_set_vct(struct oplus_chg_ic_dev *ic_dev, int value)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL)
		return -ENODEV;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	return sn28z729_set_cyclic_decrease_voltage(chip, value);
}

static int oplus_sn28z729_get_cuv_state(struct oplus_chg_ic_dev *ic_dev, int *value)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL)
		return -ENODEV;

	if (value == NULL)
		return -EINVAL;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	return sn28z729_get_cuv_state(chip, value);
}

static int oplus_sn28z729_set_cuv_state(struct oplus_chg_ic_dev *ic_dev, int value)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL)
		return -ENODEV;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	return sn28z729_set_cuv_state(chip, value & 0xffff);
}

static int sn28z729_force_report_full(struct chip_sn28z729 *chip, bool data)
{
	int status = 0;
	unsigned short flag;

	if (is_chip_suspended_or_locked(chip))
		return 0;

	if (data)
		flag = SN28729_SUBCMD_FORCE_REPORT_FULL_CMD;
	else
		flag = !SN28729_SUBCMD_FORCE_REPORT_FULL_CMD;

	status = sn28z729_write_block(chip, SN28729_SUBCMD_FORCE_REPORT_FULL_ADDR, (u8 *)&flag,
		sizeof(flag), 0, false, sizeof(flag), true);
	if (status < 0)
		chg_err("failed, status = 0x%x \n", status);

	return status;
}

static int oplus_sn28z729_set_batt_full(struct oplus_chg_ic_dev *ic_dev, bool value)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL)
		return -ENODEV;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	return sn28z729_force_report_full(chip, value);
}

static void sn28z729_register_devinfo(struct chip_sn28z729 *chip)
{
	int ret = 0;
	char *version = "sn28z729";
	char *manufacture = "TI";

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (chip->gauge_num == 0)
		ret = register_device_proc("gauge", version, manufacture);
	else
		ret = register_device_proc("sub_gauge", version, manufacture);
#endif
	if (ret)
		chg_err("register_devinfo fail\n");
}

static void sn28z729_reset(struct i2c_client *client)
{ /* TODO: */
}

static int sn28z729_pm_resume(struct device *dev)
{
	struct chip_sn28z729 *chip;

	chip = dev_get_drvdata(dev);
	if (!chip)
		return 0;

	atomic_set(&chip->suspended, 0);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_RESUME);
	return 0;
}

static int sn28z729_pm_suspend(struct device *dev)
{
	struct chip_sn28z729 *chip;

	chip = dev_get_drvdata(dev);
	if (!chip)
		return 0;

	atomic_set(&chip->suspended, 1);
	return 0;
}

static const struct dev_pm_ops sn28z729_pm_ops = {
	.resume = sn28z729_pm_resume,
	.suspend = sn28z729_pm_suspend,
};

static int oplus_sn28z729_get_batt_vol(struct oplus_chg_ic_dev *ic_dev, int index, int *volt)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || volt == NULL) {
		chg_err("oplus_chg_ic_dev or volt is null\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (index) {
	case 0:
		*volt = sn28z729_get_batt_volt(chip, SN28Z729_CELL_1_VOLT);
		break;
	case 1:
		*volt = sn28z729_get_batt_volt(chip, SN28Z729_CELL_2_VOLT);
		break;
	default:
		chg_err("index(=%d) over size\n", index);
		return -EINVAL;
	}

	return 0;
}

static int oplus_sn28z729_get_batt_max(struct oplus_chg_ic_dev *ic_dev, int *volt)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || volt == NULL) {
		chg_err("oplus_chg_ic_dev or volt is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*volt = sn28z729_get_batt_volt(chip, SN28Z729_CELL_MAX_VOLT);

	return 0;
}

static int oplus_sn28z729_get_batt_min(struct oplus_chg_ic_dev *ic_dev, int *volt)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || volt == NULL) {
		chg_err("oplus_chg_ic_dev or volt is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*volt = sn28z729_get_batt_volt(chip, SN28Z729_CELL_MIN_VOLT);

	return 0;
}

static int oplus_sn28z729_get_batt_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || curr_ma == NULL) {
		chg_err("oplus_chg_ic_dev or curr_ma is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*curr_ma = sn28z729_get_average_current(chip);

	return 0;
}

static int oplus_sn28z729_get_batt_temp(struct oplus_chg_ic_dev *ic_dev, int *temp)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || temp == NULL) {
		chg_err("oplus_chg_ic_dev or temp is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip)
		return -ENODEV;

	*temp = sn28z729_get_battery_temperature(chip);
	return 0;
}

static int oplus_sn28z729_get_batt_soc(struct oplus_chg_ic_dev *ic_dev, int *soc)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || soc == NULL) {
		chg_err("oplus_chg_ic_dev or soc is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*soc = sn28z729_get_battery_soc(chip);

	return 0;
}

static int oplus_sn28z729_get_batt_fcc(struct oplus_chg_ic_dev *ic_dev, int *fcc)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || fcc == NULL) {
		chg_err("oplus_chg_ic_dev or fcc is null");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*fcc = sn28z729_get_battery_fcc(chip);

	return 0;
}

static int oplus_sn28z729_get_batt_cc(struct oplus_chg_ic_dev *ic_dev, int *cc)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || cc == NULL) {
		chg_err("oplus_chg_ic_dev or cc is null\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*cc = sn28z729_get_battery_cc(chip);

	return 0;
}

static int oplus_sn28z729_get_batt_rm(struct oplus_chg_ic_dev *ic_dev, int *rm)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || rm == NULL) {
		chg_err("oplus_chg_ic_dev or rm is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*rm = sn28z729_get_batt_remaining_capacity(chip);

	return 0;
}

static int oplus_sn28z729_get_batt_soh(struct oplus_chg_ic_dev *ic_dev, int *soh)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || soh == NULL) {
		chg_err("oplus_chg_ic_dev or soh is null");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*soh = sn28z729_get_battery_soh(chip);

	return 0;
}

static int oplus_sn28z729_get_batt_auth(struct oplus_chg_ic_dev *ic_dev, bool *pass)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || pass == NULL) {
		chg_err("oplus_chg_ic_dev or pass is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*pass = sn28z729_get_battery_authenticate(chip);

	return 0;
}

static int oplus_sn28z729_get_batt_hmac(struct oplus_chg_ic_dev *ic_dev, bool *pass)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || pass == NULL) {
		chg_err("oplus_chg_ic_dev or pass is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*pass = sn28z729_get_battery_hmac(chip);

	return 0;
}

static int oplus_sn28z729_update_soc_smooth_parameter(struct oplus_chg_ic_dev *ic_dev)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	return sn28z729_update_sleep_mode_status(chip, true);
}

static int oplus_sn28z729_set_lock(struct oplus_chg_ic_dev *ic_dev, bool lock)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	atomic_set(&chip->locked, lock ? 1 : 0);

	return 0;
}

static int oplus_sn28z729_is_locked(struct oplus_chg_ic_dev *ic_dev, bool *locked)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*locked = !!atomic_read(&chip->locked);

	return 0;
}

static int oplus_sn28z729_get_batt_num(struct oplus_chg_ic_dev *ic_dev, int *num)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*num = chip->batt_num;

	return 0;
}

static int oplus_sn28z729_get_gauge_type(struct oplus_chg_ic_dev *ic_dev, int *gauge_type)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || gauge_type == NULL) {
		chg_err("oplus_chg_ic_dev or gauge_type is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*gauge_type = GAUGE_TYPE_PACK;

	return 0;
}

static int oplus_sn28z729_get_device_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*type = chip->device_type;

	return 0;
}

static int oplus_sn28z729_get_battery_gauge_type_for_bcc(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || type == NULL) {
		chg_err("oplus_chg_ic_dev is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is null\n");
		return -ENODEV;
	}

	*type = DEVICE_SN28Z729;

	return 0;
}

static int oplus_sn28z729_get_battery_dod0(struct oplus_chg_ic_dev *ic_dev, int index, int *dod0)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || dod0 == NULL)
		return -ENODEV;

	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (index) {
	case 0:
		*dod0 = sn28z729_get_dod_parameters(chip, SN28Z729_CELL_1_DOD0);
		break;
	case 1:
		*dod0 = sn28z729_get_dod_parameters(chip, SN28Z729_CELL_2_DOD0);
		break;
	default:
		chg_info("index(=%d), over size\n", index);
		return -EINVAL;
	}
	return 0;
}

static int oplus_sn28z729_get_battery_dod0_passed_q(struct oplus_chg_ic_dev *ic_dev,
	int *dod_passed_q)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL || dod_passed_q == NULL) {
		chg_err("oplus_chg_ic_dev is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*dod_passed_q = sn28z729_get_dod_parameters(chip, SN28Z729_CELL_DOD_PASSED_Q);
	return 0;
}

static int oplus_sn28z729_get_battery_qmax(struct oplus_chg_ic_dev *ic_dev, int index, int *qmax)
{
	struct chip_sn28z729 *chip;
	if (ic_dev == NULL || qmax == NULL) {
		chg_err("oplus_chg_ic_dev or qmax is null");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (index) {
	case 0:
		*qmax = sn28z729_get_qmax_parameters(chip, SN28Z729_CELL_1_QMAX);
		break;
	case 1:
		*qmax = sn28z729_get_qmax_parameters(chip, SN28Z729_CELL_2_QMAX);
		break;
	default:
		chg_info("index(=%d), over size\n", index);
		return -EINVAL;
	}
	return 0;
}

static int oplus_sn28z729_get_battery_qmax_passed_q(struct oplus_chg_ic_dev *ic_dev,
	int *qmax_passed_q)
{
	struct chip_sn28z729 *chip;
	if (ic_dev == NULL || qmax_passed_q == NULL) {
		chg_err("oplus_chg_ic_dev is null\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*qmax_passed_q = sn28z729_get_qmax_parameters(chip, SN28Z729_CELL_QMAX_PASSED_Q);
	return 0;
}

static int oplus_sn28z729_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*suspend = atomic_read(&chip->suspended);

	return 0;
}

static int oplus_sn28z729_get_batt_exist(struct oplus_chg_ic_dev *ic_dev, bool *exist)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (atomic_read(&chip->i2c_err_count) > I2C_ERR_MAX)
		*exist = false;
	else
		*exist = true;

	return 0;
}

static int sn28z729_get_batt_sn(struct chip_sn28z729 *chip)
{
	int ret;
	u8 buf[SN28Z729_BATT_SERIAL_NUM_SIZE] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return 0;

	if (OPLUS_BATT_SERIAL_NUM_SIZE <= SN28Z729_BATT_SERIAL_NUM_SIZE) {
		chg_err("sn insufficient container storage length\n");
		return ret;
	}

	ret = sn28z729_read_block(chip, SN28Z729_REG_MANUFACTURER_NAME, buf,
		SN28Z729_BATT_SERIAL_NUM_SIZE, 0, true, true);
	if (ret < 0) {
		chg_err("get sn failed");
		return ret;
	}

	memmove(chip->battinfo.batt_serial_num, buf, SN28Z729_BATT_SERIAL_NUM_SIZE);

	return 0;
}

static int oplus_sn28z729_get_batt_sn(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	int bsnlen;
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !buf || len < OPLUS_BATT_SERIAL_NUM_SIZE)
		return -EINVAL;

	if (!strlen(chip->battinfo.batt_serial_num))
		sn28z729_get_batt_sn(chip);

	chg_info("batt_sn(%s):%s", ic_dev->name, chip->battinfo.batt_serial_num);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	bsnlen = strscpy(buf, chip->battinfo.batt_serial_num, OPLUS_BATT_SERIAL_NUM_SIZE);
#else
	bsnlen = strlcpy(buf, chip->battinfo.batt_serial_num, OPLUS_BATT_SERIAL_NUM_SIZE);
#endif

	return bsnlen;
}

static int sn28z729_get_lifetime_status(struct chip_sn28z729 *chip,
	struct oplus_gauge_lifetime *lifetime)
{
	int ret;
	int cell_0_volt;
	int cell_1_volt;
	u8 buf[SN28Z729_LIFETIME_NUM_SIZE] = { 0 };

	if (!chip || !lifetime)
		return -EINVAL;

	if (is_chip_suspended_or_locked(chip))
		return -EINVAL;

	ret = sn28z729_read_block(chip, SN28Z729_REG_LIFETIME_ADDR, buf, SN28Z729_LIFETIME_NUM_SIZE,
		0, false, true);
	if (ret) {
		chg_err("error reading lifetime, ret = %d.\n", ret);
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

static int oplus_sn28z729_get_lifetime_status(struct oplus_chg_ic_dev *ic_dev,
						struct oplus_gauge_lifetime *lifetime_status)
{
	struct chip_sn28z729 *chip;
	int ret = -EINVAL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ret = sn28z729_get_lifetime_status(chip, lifetime_status);

	return ret;
}

static int oplus_sn28z729_get_lifetime_info(struct oplus_chg_ic_dev *ic_dev, u8 *buf, int len)
{
	struct chip_sn28z729 *chip;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ret = sn28z729_get_lifetime_info(chip, buf, len);

	return ret;
}

static int oplus_sn28z729_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct chip_sn28z729 *chip;
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	return sn28z729_gauge_reg_dump(chip);
}

static int sn28z729_get_passedchg(struct chip_sn28z729 *chip, int *val)
{
	int ret = -1;
	u8 read_buf[SN28Z729_GET_PASSEDCHG_WLEN] = { 0 };
	if (chip == NULL || is_chip_suspended_or_locked(chip)) {
		return -ENODEV;
	}

	ret = sn28z729_read_block(chip, SN28Z729_GET_PASSEDCHG_ADDR, read_buf,
			SN28Z729_GET_PASSEDCHG_WLEN, SN28Z729_GET_PASSEDCHG_OFFSET, false, true);
	if (ret < 0)
		return ret;


	*val = (read_buf[1] << 8) | read_buf[0];
	if (atomic_read(&chip->suspended) == 1)
		return -EINVAL;

	return 0;
}

static int oplus_sn28z729_get_passedchg(struct oplus_chg_ic_dev *ic_dev, int *val)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	return sn28z729_get_passedchg(chip, val);
}

static int oplus_get_manu_date(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	struct chip_sn28z729 *chip;
	int date_len = 0;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip || !buf || len < OPLUS_BATTINFO_DATE_SIZE)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	chg_info("BattManuDate:0x%04x", chip->battinfo.manu_date);
	date_len = snprintf(buf, len, "%d-%02d-%02d", (((chip->battinfo.manu_date >> 9) & 0x7F)
		+ 1980), (chip->battinfo.manu_date >> 5) & 0xF, chip->battinfo.manu_date & 0x1F);
	return date_len;
}

static int oplus_sn28z729_get_first_usage_date(struct oplus_chg_ic_dev *ic_dev, char *buf, int len)
{
	struct chip_sn28z729 *chip;
	int date_len = 0;
	int ret;
	u8 read_buf[SN28Z729_BATT_FIRST_USAGE_DATE_WLEN] = { 0x00 };
	u16 date = 0;
	u8 check = 0;
	u8 cal_check = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !buf || len < OPLUS_BATTINFO_DATE_SIZE)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	ret = sn28z729_read_block(chip, SN28Z729_REG_OPLUS_DATA, read_buf,
		SN28Z729_BATT_FIRST_USAGE_DATE_WLEN, SN28Z729_BATT_FIRST_USAGE_DATE_OFFSET,
			false, true);
	if (ret < 0) {
		date_len =
			snprintf(buf, len, "%d-%02d-%02d", (((chip->battinfo.first_usage_date >> 9)
				& 0x7F) + 1980), (chip->battinfo.first_usage_date >> 5) & 0xF,
				chip->battinfo.first_usage_date & 0x1F);
		return date_len;
	}
	check = read_buf[2];
	cal_check = (0xFF - read_buf[0] - read_buf[1]) & 0xFF;
	if (check == cal_check) {
		date = (read_buf[1] << 8) | read_buf[0];
		date_len = snprintf(buf, len, "%d-%02d-%02d", (((date >> 9) & 0x7F) + 1980),
			(date >> 5) & 0xF, date & 0x1F);
	} else {
		date_len =
		snprintf(buf, len, "%d-%02d-%02d", (((chip->battinfo.first_usage_date >> 9)
		& 0x7F) + 1980), (chip->battinfo.first_usage_date >> 5) & 0xF,
		chip->battinfo.first_usage_date & 0x1F);
	}

	return date_len;
}

static int sn28z729_set_batt_first_usage_date(struct chip_sn28z729 *chip, u32 data)
{
	int ret = 0;
	u16 first_usage_date = (data >> 8) & 0xFFFF;
	u8 check_sum = data & 0xFF;
	u8 calc_check_sum = 0x00;
	u8 write_data[SN28Z729_BATT_FIRST_USAGE_DATE_WLEN] = { 0x00, 0x00, 0x00 };

	write_data[0] = first_usage_date & 0xFF;
	write_data[1] = (first_usage_date >> 8) & 0xFF;
	write_data[2] = check_sum;
	calc_check_sum = (SN28Z729_BATTINFO_DEFAULT_CHECKSUM - write_data[0] - write_data[1])
			& 0xFF;
	if (check_sum == calc_check_sum) {
		ret = sn28z729_write_block(chip, SN28Z729_REG_OPLUS_DATA, write_data,
			SN28Z729_BATT_FIRST_USAGE_DATE_WLEN,
			SN28Z729_BATT_FIRST_USAGE_DATE_OFFSET, true, SN28Z729_BLOCK_SIZE, true);
		if (ret < 0)
			return ret;
	}
	return ret;
}

static int oplus_sn28z729_set_batt_first_usage_date(struct oplus_chg_ic_dev *ic_dev,
		const char *buf)
{
	int year = 0;
	int month = 0;
	int day = 0;
	u16 date = 0;
	u32 data = 0x00;
	u8 check_sum = 0x00;
	int ret;
	struct chip_sn28z729 *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !buf)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	sscanf(buf, "%d-%d-%d", &year, &month, &day);
	date = (((year - 1980) & 0x7F) << 9) | ((month & 0xF) << 5) | (day & 0x1F);
	check_sum = 0xFF - ((date >> 8) & 0xFF) - (date & 0xFF);
	data = date << 8 | check_sum;
	chg_info("%d-%d-%d, date=0x%04x, data=0x%08x", year, month, day, date, data);
	ret = sn28z729_set_batt_first_usage_date(chip, data);
	if (ret < 0)
		return ret;

	chip->battinfo.first_usage_date = date;

	return 0;
}

static int oplus_get_ui_cycle_count(struct oplus_chg_ic_dev *ic_dev, u16 *ui_cycle_count)
{
	struct chip_sn28z729 *chip;
	int ret;
	u8 check;
	u8 cal_check;
	u8 buf[SN28Z729_ECO_UI_CC_WLEN] = { 0 };
	if (ic_dev == NULL) {
		chg_info("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip || !ui_cycle_count)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	if (is_chip_suspended_or_locked(chip))
		return chip->battinfo.ui_cycle_count;
	ret = sn28z729_read_block(chip, SN28Z729_REG_OPLUS_DATA, buf, SN28Z729_ECO_UI_CC_WLEN,
				SN28Z729_ECO_UI_CC_OFFSET, false, true);
	if (ret < 0) {
		*ui_cycle_count = chip->battinfo.ui_cycle_count;
		return 0;
	}
	check = buf[2];
	cal_check = (0xFF - buf[0] - buf[1]) & 0xFF;
	if (check == cal_check)
		*ui_cycle_count = (buf[1] << 8) | buf[0];
	else
		*ui_cycle_count = chip->battinfo.ui_cycle_count;


	return 0;
}

static int sn28z729_set_batt_ui_cycle_count(struct chip_sn28z729 *chip, u32 data)
{
	int ret = -1;
	u16 cycle_count = (data >> 8) & 0xFFFF;
	u8 check_sum = data & 0xFF;
	u8 calc_check_sum = 0x00;
	u8 write_data[SN28Z729_ECO_UI_CC_WLEN] = { 0x00, 0x00, 0x00 };
	write_data[0] = cycle_count & 0xFF;
	write_data[1] = (cycle_count >> 8) & 0xFF;
	write_data[2] = check_sum;
	calc_check_sum = (0xFF - write_data[0] - write_data[1]) & 0xFF;
	if (check_sum == calc_check_sum) {
		ret = sn28z729_write_block(chip, SN28Z729_REG_OPLUS_DATA, write_data,
			SN28Z729_ECO_UI_CC_WLEN, SN28Z729_ECO_UI_CC_OFFSET, true,
			SN28Z729_BLOCK_SIZE, true);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int oplus_set_ui_cycle_count(struct oplus_chg_ic_dev *ic_dev, u16 ui_cycle_count)
{
	struct chip_sn28z729 *chip;
	u8 check_sum = 0x00;
	u32 data = 0x00;
	int ret;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	check_sum = 0xFF - ((ui_cycle_count >> 8) & 0xFF) - (ui_cycle_count & 0xFF);
	data = ui_cycle_count << 8 | check_sum;
	ret = sn28z729_set_batt_ui_cycle_count(chip, data);
	if (ret < 0) {
		chg_err("set ui cycle  count %u failed", ui_cycle_count);
		return ret;
	}
	chip->battinfo.ui_cycle_count = ui_cycle_count;
	return 0;
}

static int sn28z729_set_batt_ui_soh(struct chip_sn28z729 *chip, u16 data)
{
	int ret = -1;
	u8 soh = (data >> 8) & 0xFF;
	u8 check_sum = data & 0xFF;
	u8 calc_check_sum = 0x00;
	u8 write_data[SN28Z729_BATT_UI_SOH_WLEN] = { 0x00 };
	write_data[0] = soh;
	write_data[1] = check_sum;
	calc_check_sum = (SN28Z729_BATTINFO_DEFAULT_CHECKSUM - write_data[0]) & 0xFF;
	if (check_sum == calc_check_sum) {
		ret = sn28z729_write_block(chip, SN28Z729_REG_OPLUS_DATA, write_data,
		SN28Z729_BATT_UI_SOH_WLEN, SN28Z729_BATT_UI_SOH_OFFSET, true,
		SN28Z729_BLOCK_SIZE, true);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int oplus_set_ui_soh(struct oplus_chg_ic_dev *ic_dev, u8 ui_soh)
{
	struct chip_sn28z729 *chip;
	u8 check_sum = 0x00;
	u16 data = 0x00;
	int ret;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	check_sum = 0xFF - (ui_soh & 0xFF);
	data = ui_soh << 8 | check_sum;
	ret = sn28z729_set_batt_ui_soh(chip, data);
	if (ret < 0) {
		chg_err("set batt ui soh %u failed", ui_soh);
		return ret;
	}
	chip->battinfo.ui_soh = ui_soh;
	return 0;
}

static int sn28z729_get_batt_ui_soh(struct chip_sn28z729 *chip, u8 *ui_soh)
{
	int ret = -1;
	u8 read_data[SN28Z729_BATT_UI_SOH_WLEN] = { 0x00 };
	u8 calc_check_sum = 0;
	ret = sn28z729_read_block(chip, SN28Z729_REG_OPLUS_DATA, read_data,
		SN28Z729_BATT_UI_SOH_WLEN, SN28Z729_BATT_UI_SOH_OFFSET, false, true);
	if (ret < 0) {
		*ui_soh = chip->battinfo.ui_soh;
		return -EINVAL;
	} else {
		calc_check_sum = (SN28Z729_BATTINFO_DEFAULT_CHECKSUM - read_data[0]) & 0xFF;
		if (calc_check_sum == read_data[1]) {
			*ui_soh = read_data[0];
		} else {
			*ui_soh = chip->battinfo.ui_soh;
			return -EINVAL;
		}
	}

	return ret;
}

static int oplus_get_ui_soh(struct oplus_chg_ic_dev *ic_dev, u8 *ui_soh)
{
	struct chip_sn28z729 *chip;
	int ret = -1;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip || !ui_soh)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	ret = sn28z729_get_batt_ui_soh(chip, ui_soh);
	if (ret < 0)
		*ui_soh = chip->battinfo.ui_soh;

	return 0;
}

static int sn28z729_get_used_flag(struct chip_sn28z729 *chip, u8 *used_flag)
{
	int ret = -1;
	u8 read_data[SN28Z729_BATT_USED_FLAG_WLEN] = { 0x00 };
	u8 calc_check_sum = 0;
	ret = sn28z729_read_block(chip, SN28Z729_REG_OPLUS_DATA, read_data,
		SN28Z729_BATT_USED_FLAG_WLEN, SN28Z729_BATT_USED_FLAG_OFFSET, false, true);
	if (ret < 0) {
		*used_flag = chip->battinfo.used_flag;
		return -EINVAL;
	} else {
		calc_check_sum = (SN28Z729_BATTINFO_DEFAULT_CHECKSUM - read_data[0]) & 0xFF;
		if (calc_check_sum == read_data[1]) {
			*used_flag = read_data[0];
		} else {
			*used_flag = chip->battinfo.used_flag;
			return -EINVAL;
		}
	}

	return ret;
}

static int oplus_sn28z729_get_used_flag(struct oplus_chg_ic_dev *ic_dev, u8 *used_flag)
{
	struct chip_sn28z729 *chip;
	int ret;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip || !used_flag)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;
	ret = sn28z729_get_used_flag(chip, used_flag);
	if (ret < 0)
		*used_flag = chip->battinfo.used_flag;

	return 0;
}

static int sn28z729_set_batt_used_flag(struct chip_sn28z729 *chip, int data)
{
	int ret = 0;
	u8 flag = (data >> 8) & 0xFF;
	u8 check_sum = data & 0xFF;
	u8 calc_check_sum = 0x00;
	u8 write_data[SN28Z729_BATT_USED_FLAG_WLEN] = { 0x00 };
	write_data[0] = flag;
	write_data[1] = check_sum;
	calc_check_sum = (SN28Z729_BATTINFO_DEFAULT_CHECKSUM - write_data[0]) & 0xFF;
	if (check_sum == calc_check_sum) {
		ret = sn28z729_write_block(chip, SN28Z729_REG_OPLUS_DATA, write_data,
			SN28Z729_BATT_USED_FLAG_WLEN, SN28Z729_BATT_USED_FLAG_OFFSET, true,
			SN28Z729_BLOCK_SIZE, true);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int oplus_sn28z729_set_used_flag(struct oplus_chg_ic_dev *ic_dev, u8 used_flag)
{
	struct chip_sn28z729 *chip;
	u8 check_sum = 0x00;
	u32 data = 0x00;
	int ret = 0;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip)
		return -EINVAL;

	if (!chip->support_eco_design)
		return -ENOTSUPP;

	check_sum = 0xFF - (used_flag & 0xFF);
	data = used_flag << 8 | check_sum;
	ret = sn28z729_set_batt_used_flag(chip, data);
	if (ret < 0) {
		chg_err("set batt used flag failed");
		return ret;
	}
	chip->battinfo.used_flag = used_flag;
	return 0;
}

static int oplus_get_dec_fg_type(struct oplus_chg_ic_dev *ic_dev, int *fg_type)
{
	struct chip_sn28z729 *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip || !fg_type)
		return -EINVAL;

	*fg_type = DEC_CV_PACK_SINGLE;
	return 0;
}

static int oplus_sn28z729_get_dec_cv_soh(struct oplus_chg_ic_dev *ic_dev, int *dec_soh)
{
	struct chip_sn28z729 *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !dec_soh)
		return -EINVAL;

	*dec_soh = chip->cc_pre;
	return 0;
}

static int sn28z729_set_three_level_term_volt(struct chip_sn28z729 *chip, const char *buf, int len)
{
	int status = 0;
	u8 input_param[SN28Z729_TERM_MAX_VLOT_LEN] = {0};
	u8 term_volt[SN28Z729_TERM_VLOT_LEN] = { 0 };
	u16 term = 0x00;

	if (is_chip_suspended_or_locked(chip))
		return 0;

	memmove(input_param, buf, len);
/* change the param to TI28Z729 register param */
	term = 2 * ((input_param[1] << 8) + input_param[0]);
	term_volt[0] = (term & 0xff);
	term_volt[1] = ((term >> 8) & 0xff);

	term = 2 * ((input_param[3] << 8) + input_param[2]);
	term_volt[2] = (term & 0xff);
	term_volt[3] = ((term >> 8) & 0xff);

	term = 2 * ((input_param[5] << 8) + input_param[4]);
	term_volt[4] = (term & 0xff);
	term_volt[5] = ((term >> 8) & 0xff);

	term_volt[6] = input_param[6];
	term_volt[7] = input_param[7];
	term_volt[8] = input_param[8];
	term_volt[9] = input_param[9];  /* Term V Hold Time4 */
	term_volt[10] = input_param[16]; /* Term V Interval2 */
	term_volt[11] = input_param[0];
	term_volt[12] = input_param[1];
	term_volt[13] = input_param[2];
	term_volt[14] = input_param[3];
	term_volt[15] = input_param[4];
	term_volt[16] = input_param[5];
	chg_info("set term_volt[%x,%x,%x,%x,%x,%x], [%x,%x,%x,%x,%x], [%x,%x,%x,%x,%x,%x]\n",
			term_volt[0], term_volt[1], term_volt[2], term_volt[3], term_volt[4],
			term_volt[5], term_volt[6], term_volt[7], term_volt[8], term_volt[9],
			term_volt[10], term_volt[11], term_volt[12], term_volt[13], term_volt[14],
			term_volt[15], term_volt[16]);

	if (sn28z729_get_firm_ver(chip) >= SN28Z729_NEED_TO_SUPPORT_THIRD_TERM_VOLT_FIRM_VER) {
		status = sn28z729_write_block(chip, GAUGE_SET_TERM_PARAM_CMD, term_volt, len,
			0, false, len, true);
		if (status != 0) {
			chg_info("update term param failed, status=%d.", status);
			return -EINVAL;
		}
	} else {
		/* not support! */
		return -EINVAL;
	}
	return status;
}

static int oplus_sn28z729_set_three_level_term_volt(struct oplus_chg_ic_dev *ic_dev, char *buf,
				int len)
{
	struct chip_sn28z729 *chip;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (len <= 0)
		return -EINVAL;

	if (len > SN28Z729_TERM_VLOT_LEN)
		len = SN28Z729_TERM_VLOT_LEN;

	ret = sn28z729_set_three_level_term_volt(chip, buf, len);
	if (ret < 0) {
		chg_err("set three level term volt failed");
		return ret;
	}
	return 0;
}

static int sn28z729_get_three_level_term_volt(struct chip_sn28z729 *chip, char *buf, int len)
{
	int status = 0;
	u8 term_volt[SN28Z729_TERM_VLOT_LEN] = { 0 };
	u8 output_param[32] = { 0 };

	if (is_chip_suspended_or_locked(chip))
		return 0;

	if (len > SN28Z729_TERM_VLOT_LEN)
		len = SN28Z729_TERM_VLOT_LEN;

	if (sn28z729_get_firm_ver(chip) >= SN28Z729_NEED_TO_SUPPORT_THIRD_TERM_VOLT_FIRM_VER) {
		/* read back the term param. */
		status = sn28z729_read_block(chip, GAUGE_SET_TERM_PARAM_CMD, term_volt, len,
			0, false, true);
		if (status != 0) {
			chg_info("read term param failed, status=%d\n", status);
			return -EINVAL;
		}
		/* change the param to AP interface param.*/
		output_param[0] = term_volt[11];
		output_param[1] = term_volt[12];
		output_param[2] = term_volt[13];
		output_param[3] = term_volt[14];
		output_param[4] = term_volt[15];
		output_param[5] = term_volt[16];
		output_param[6] = term_volt[6];
		output_param[7] = term_volt[7];
		output_param[8] = term_volt[8];
		output_param[9] = term_volt[9]; /* Term V Hold Time4 */
		output_param[16] = term_volt[10];
		memmove(buf, output_param, len);
		chg_info("get term_volt[%x, %x, %x, %x, %x, %x], [%x, %x, %x, %x, %x],"
			"[%x, %x, %x, %x, %x, %x]\n", term_volt[0], term_volt[1], term_volt[2],
			term_volt[3], term_volt[4], term_volt[5], term_volt[6], term_volt[7],
			term_volt[8], term_volt[9], term_volt[10], term_volt[11],
			term_volt[12], term_volt[13], term_volt[14], term_volt[15],
			term_volt[16]);
	} else {
		/* TODO: not support! */
		return -EINVAL;
	}
	return len;
}

static int oplus_sn28z729_get_three_level_term_volt(struct oplus_chg_ic_dev *ic_dev, char *buf,
				int len)
{
	struct chip_sn28z729 *chip;
	int ret = 0;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is null");
		return -ENODEV;
	}

	if (!buf || len <= 0)
		return -EINVAL;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ret = sn28z729_get_three_level_term_volt(chip, buf, len);
	if (ret < 0) {
		chg_err("get three level term volt failed");
		return ret;
	}
	return 0;
}

static int sn28z729_get_battery_soc_centi(struct chip_sn28z729 *chip)
{
	int rc = 0;
	int soc_centi = 0;
	u8 data[SN28Z729_SOC_CENTI_LEN] = {0};

	if (sn28z729_get_firm_ver(chip) < SN28Z729_NEED_TO_SUPPORT_THIRD_TERM_VOLT_FIRM_VER)
		return -ENOTSUPP;

	if (is_chip_suspended_or_locked(chip))
		return chip->soc_centi_pre;

	rc = sn28z729_read_block(chip, SN28Z729_SOC_CENTI_ADDR, data, SN28Z729_SOC_CENTI_LEN,
				0, false, true);
	if (rc) {
		chg_err("error reading soc_centi\n");
		return chip->soc_centi_pre;
	}

	soc_centi = ((data[1] << 0x08) | data[0]);
	chip->soc_centi_pre = soc_centi;

	return soc_centi;
}

static int oplus_sn28z729_get_batt_soc_centi(struct oplus_chg_ic_dev *ic_dev, int *soc_centi)
{
	struct chip_sn28z729 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		*soc_centi = -ENODEV;
		return 0;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*soc_centi = sn28z729_get_battery_soc_centi(chip);

	return 0;
}

static void *oplus_chg_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) && (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_sn28z729_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_sn28z729_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, oplus_sn28z729_reg_dump);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL,
					oplus_sn28z729_get_batt_vol);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX,
					oplus_sn28z729_get_batt_max);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN,
					oplus_sn28z729_get_batt_min);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR,
					oplus_sn28z729_get_batt_curr);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP,
					oplus_sn28z729_get_batt_temp);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC,
					oplus_sn28z729_get_batt_soc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC,
					oplus_sn28z729_get_batt_fcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_CC,
					oplus_sn28z729_get_batt_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_RM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_RM,
					oplus_sn28z729_get_batt_rm);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH,
					oplus_sn28z729_get_batt_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH,
					oplus_sn28z729_get_batt_auth);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC,
					oplus_sn28z729_get_batt_hmac);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL,
					oplus_sn28z729_set_batt_full);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_PASSEDCHG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_PASSEDCHG,
					oplus_sn28z729_get_passedchg);
		break;
	case OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH,
					oplus_sn28z729_update_soc_smooth_parameter);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_LOCK:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_LOCK,
			oplus_sn28z729_set_lock);
		break;
	case OPLUS_IC_FUNC_GAUGE_IS_LOCKED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_IS_LOCKED,
			oplus_sn28z729_is_locked);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM,
			oplus_sn28z729_get_batt_num);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_GAUGE_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_GAUGE_TYPE,
			oplus_sn28z729_get_gauge_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE,
			oplus_sn28z729_get_device_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_IS_SUSPEND,
			oplus_sn28z729_is_suspend);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DOD0:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DOD0,
			oplus_sn28z729_get_battery_dod0);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DOD0_PASSED_Q:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DOD0_PASSED_Q,
						oplus_sn28z729_get_battery_dod0_passed_q);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_QMAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_QMAX,
			oplus_sn28z729_get_battery_qmax);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_QMAX_PASSED_Q:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_QMAX_PASSED_Q,
				oplus_sn28z729_get_battery_qmax_passed_q);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC,
			oplus_sn28z729_get_battery_gauge_type_for_bcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_STATUS,
				oplus_sn28z729_get_lifetime_status);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SILI_LIFETIME_INFO,
				oplus_sn28z729_get_lifetime_info);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST,
			oplus_sn28z729_get_batt_exist);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEEP_DISCHG_COUNT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEEP_DISCHG_COUNT,
						oplus_sn28z729_get_deep_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_DEEP_DISCHG_COUNT,
				oplus_sn28z729_set_deep_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT,
				oplus_sn28z729_set_deep_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_LAST_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_LAST_CC,
			oplus_sn28z729_set_deep_last_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_REG_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_REG_INFO,
			oplus_sn28z729_get_reg_info);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_CALIB_TIME:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_CALIB_TIME,
			oplus_sn28z729_get_calib_time);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_CALIB_TIME:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_CALIB_TIME,
			oplus_sn28z729_set_calib_time);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SN,
			oplus_sn28z729_get_batt_sn);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_MANU_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_MANU_DATE,
			oplus_get_manu_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_FIRST_USAGE_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_FIRST_USAGE_DATE,
			oplus_sn28z729_get_first_usage_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_FIRST_USAGE_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_FIRST_USAGE_DATE,
			oplus_sn28z729_set_batt_first_usage_date);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_UI_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_UI_CC,
			oplus_get_ui_cycle_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_UI_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_UI_CC,
			oplus_set_ui_cycle_count);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_UI_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_UI_SOH, oplus_get_ui_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_UI_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_UI_SOH, oplus_set_ui_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_USED_FLAG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_USED_FLAG,
				oplus_sn28z729_get_used_flag);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_USED_FLAG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_USED_FLAG,
				oplus_sn28z729_set_used_flag);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT,
				oplus_sn28z729_get_deep_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_LAST_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_LAST_CC,
				oplus_sn28z729_get_deep_last_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEC_FG_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEC_FG_TYPE,
				oplus_get_dec_fg_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH,
				oplus_sn28z729_get_dec_cv_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_VCT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_VCT, oplus_sn28z729_set_vct);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_VCT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_VCT, oplus_sn28z729_get_vct);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_GAUGE_CAR_C:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_GAUGE_CAR_C,
				oplus_sn28z729_get_car_c);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_CUV_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_CUV_STATE,
				oplus_sn28z729_set_cuv_state);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_CUV_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_CUV_STATE,
				oplus_sn28z729_get_cuv_state);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_THREE_LEVEL_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_THREE_LEVEL_TERM_VOLT,
			oplus_sn28z729_set_three_level_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_THREE_LEVEL_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_THREE_LEVEL_TERM_VOLT,
			oplus_sn28z729_get_three_level_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC_CENTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC_CENTI,
			oplus_sn28z729_get_batt_soc_centi);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq sn28z729_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{ .virq_id = OPLUS_IC_VIRQ_RESUME },
};

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
static struct regmap_config sn28z729_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
};
#endif

static void sn28z729_check_iic_recover(struct work_struct *work)
{
	struct chip_sn28z729 *chip = container_of(work, struct chip_sn28z729,
		check_iic_recover.work);

	/* workaround for I2C pull SDA can't trigger error issue 230504153935012779 */
	if (chip->i2c_rst_ext)
		sn28z729_get_battery_temperature(chip);
	else
		sn28z729_get_battery_soc(chip);

	chg_info("gauge online state:%d\n", chip->ic_dev->online);
	if (!chip->ic_dev->online) {
		schedule_delayed_work(&chip->check_iic_recover,
		msecs_to_jiffies(CHECK_IIC_RECOVER_TIME));
	} else {
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ONLINE);
	}
}

static int sn28z729_vars_init(struct chip_sn28z729 *chip)
{
	atomic_set(&chip->suspended, 0);
	atomic_set(&chip->locked, 0);
	atomic_set(&chip->sync_lock, 0);
	mutex_init(&chip->chip_mutex);
	mutex_init(&chip->calib_time_mutex);
	mutex_init(&chip->extended_cmd_access);

	/* workaround for I2C pull SDA can't trigger error issue 230504153935012779 */
	chip->err_status = false;

	/* default 50% */
	chip->soc_pre = 50;
	chip->soc_centi_pre = -EINVAL;
	/* default 999mA */
	chip->current_pre = 999;

	/* default 3800mV */
	chip->batt_max_volt_pre = 3800;
	chip->batt_min_volt_pre = 3800;
	chip->batt_cell_1_volt_pre = 3800;
	chip->batt_cell_2_volt_pre = 3800;

	chip->deep_term_volt_pre = 3000;

	INIT_DELAYED_WORK(&chip->check_iic_recover, sn28z729_check_iic_recover);
	INIT_WORK(&chip->fcc_too_small_check_work, sn28z729_fcc_too_small_check_work);
	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sn28z729_driver_probe(struct i2c_client *client)
#else
static int sn28z729_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	int rc = 0;
	int ic_index;
	struct chip_sn28z729 *chip;
	enum oplus_chg_ic_type ic_type;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };

	chip = devm_kzalloc(&client->dev, sizeof(struct chip_sn28z729), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	chip->authenticate_data = devm_kzalloc(&client->dev,
		sizeof(struct sn28z729_authenticate_data), GFP_KERNEL);
	if (chip->authenticate_data == NULL) {
		rc = -ENOMEM;
		dev_err(&client->dev, "failed to allocate authenticate_data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, chip);
	chip->dev = &client->dev;
	chip->client = client;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	chip->regmap = devm_regmap_init_i2c(client, &sn28z729_regmap_config);
	if (!chip->regmap) {
		rc = -ENODEV;
		goto regmap_init_err;
	}
#endif /* CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT */

	sn28z729_vars_init(chip);
	sn28z729_parse_dt(chip);
	sn28z729_hw_config(chip);
	sn28z729_update_sleep_mode_status(chip, false);
	sn28z729_register_devinfo(chip);
	sn28z729_get_batt_sn(chip);

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
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "gauge-sn28z729:%d", ic_index);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	chip->odb = devm_oplus_device_bus_register(chip->dev, &sn28z729_regmap_config,
			ic_cfg.manu_name);
	if (IS_ERR_OR_NULL(chip->odb)) {
		chg_err("register odb error\n");
		rc = -EFAULT;
		goto error;
	}
#endif /* CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT */
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = sn28z729_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(sn28z729_virq_table);
	ic_cfg.of_node = chip->dev->of_node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", chip->dev->of_node->name);
		goto ic_reg_error;
	}
	chg_info("register %s\n", chip->dev->of_node->name);

	oplus_sn28z729_init(chip->ic_dev);

	return 0;

ic_reg_error:
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	devm_oplus_device_bus_unregister(chip->odb);
#endif
error:
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
regmap_init_err:
#endif
	devm_kfree(&client->dev, chip);
	return rc;
}

static const struct of_device_id sn28z729_match[] = {
	{ .compatible = "oplus,sn28z729-battery" },
	{},
};

static const struct i2c_device_id sn28z729_id[] = {
	{ "sn28z729-battery", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sn28z729_id);

static struct i2c_driver sn28z729_i2c_driver = {
	.driver = {
			.name = "sn28z729-battery",
			.owner = THIS_MODULE,
			.of_match_table = sn28z729_match,
			.pm = &sn28z729_pm_ops,
	},
	.probe = sn28z729_driver_probe,
	.shutdown = sn28z729_reset,
	.id_table = sn28z729_id,
};

static __init int sn28z729_driver_init(void)
{
	return i2c_add_driver(&sn28z729_i2c_driver);
}

static __exit void sn28z729_driver_exit(void)
{
	i2c_del_driver(&sn28z729_i2c_driver);
}

oplus_chg_module_register(sn28z729_driver);

MODULE_DESCRIPTION("Driver for sn28z729 charger chip");
MODULE_LICENSE("GPL v2");
