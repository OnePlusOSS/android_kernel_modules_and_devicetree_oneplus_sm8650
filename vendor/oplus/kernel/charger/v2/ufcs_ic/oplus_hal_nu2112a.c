// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[NU2112A]: %s[%d]: " fmt, __func__, __LINE__

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/ktime.h>
#include <trace/events/sched.h>
#include <uapi/linux/sched/types.h>

#include <ufcs_class.h>

#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_impedance_check.h>
#include <oplus_chg_monitor.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
#include <linux/pinctrl/consumer.h>
#endif
#include "../voocphy/oplus_voocphy.h"
#include "oplus_hal_nu2112a.h"

enum nu2112a_osc_status {
	NU2112A_OSC_INIT,
	NU2112A_OSC_ENABLE,
	NU2112A_OSC_DISABLE,
	NU2112A_OSC_INVALID,
};

enum nu2112a_ic_status {
	NU2112A_IC_OK,
	NU2112A_IC_PIN_DIAG_FAIL,
	NU2112A_IC_POWER_NG,
};

static struct oplus_voocphy_manager *oplus_voocphy_mg = NULL;
static struct mutex i2c_rw_lock;

static bool error_reported = false;

#define DEFUALT_VBUS_LOW 100
#define DEFUALT_VBUS_HIGH 200
#define I2C_ERR_NUM 10
#define MAIN_I2C_ERROR (1 << 0)

#define TRACK_REG_ADDR_START	NU2112A_REG_07
#define TRACK_REG_ADDR_END	NU2112A_REG_15
#define TRACK_REG_DUMP_NUM	(TRACK_REG_ADDR_END - TRACK_REG_ADDR_START)
#define NU2112A_FLAG_REG_NUMS	3

static struct ufcs_config nu2112a_ufcs_config = {
	.check_crc = false,
	.reply_ack = false,
	.msg_resend = false,
	.handshake_hard_retry = true,
};

struct nu2112a_device {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct oplus_voocphy_manager *voocphy;
	struct ufcs_dev *ufcs;

	struct oplus_chg_ic_dev *cp_ic;
	struct oplus_impedance_node *input_imp_node;
	struct oplus_impedance_node *output_imp_node;

	struct mutex i2c_rw_lock;
	struct mutex chip_lock;
	atomic_t suspended;
	atomic_t i2c_err_count;
	struct wakeup_source *chip_ws;

	int ovp_reg;
	int ocp_reg;

	bool ufcs_enable;

	bool rested;
	bool error_reported;

	bool use_ufcs_phy;
	bool use_vooc_phy;
	bool vac_support;

	u8 ufcs_reg_dump[NU2112A_FLAG_NUM];
	u8 track_reg_dump[TRACK_REG_DUMP_NUM];

	enum nu2112a_osc_status osc_status;
	struct mutex osc_status_lock;
	struct delayed_work osc_status_daemon_work;
	bool osc_ctrl_support;
	enum oplus_cp_work_mode mode;
	struct work_struct abnormal_upload_info_work;
	enum nu2112a_ic_status ic_status;
};

static enum oplus_cp_work_mode g_cp_support_work_mode[] = {
	CP_WORK_MODE_BYPASS,
	CP_WORK_MODE_2_TO_1,
	CP_WORK_MODE_3_TO_1,
};

static int nu2112a_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data);
static int nu2112a_init_device(struct nu2112a_device *chip);
static int nu2112a_svooc_hw_setting(struct nu2112a_device *chip);
static int nu2112a_vooc_hw_setting(struct nu2112a_device *chip);
static int nu2112a_set_chg_enable(struct oplus_voocphy_manager *chip, bool enable);
static void nu2112a_set_osc_status(
			struct nu2112a_device *chip, enum nu2112a_osc_status osc_status);

const char *nu2112a_adapter_error_info[16] = {
	"adapter output OVP!",
	"adapter outout UVP!",
	"adapter output OCP!",
	"adapter output SCP!",
	"adapter USB OTP!",
	"adapter inside OTP!",
	"adapter CCOVP!",
	"adapter D-OVP!",
	"adapter D+OVP!",
	"adapter input OVP!",
	"adapter input UVP!",
	"adapter drain over current!",
	"adapter input current loss!",
	"adapter CRC error!",
	"adapter watchdog timeout!",
	"invalid msg!",
};

static void nu2112a_i2c_error(bool happen)
{
	int report_flag = 0;
	if (!oplus_voocphy_mg || error_reported)
		return;

	if (happen) {
		oplus_voocphy_mg->voocphy_iic_err = true;
		oplus_voocphy_mg->voocphy_iic_err_num++;
		if (oplus_voocphy_mg->voocphy_iic_err_num >= I2C_ERR_NUM) {
			report_flag |= MAIN_I2C_ERROR;
			error_reported = true;
		}
	} else {
		oplus_voocphy_mg->voocphy_iic_err_num = 0;
	}
}

/************************************************************************/
static int __nu2112a_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	s32 ret;
	struct oplus_voocphy_manager *chip;

	chip = i2c_get_clientdata(client);
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}
	nu2112a_i2c_error(false);
	*data = (u8)ret;

	return 0;
}

static int __nu2112a_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	s32 ret;
	struct oplus_voocphy_manager *chip;

	chip = i2c_get_clientdata(client);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n", val, reg, ret);
		return ret;
	}
	nu2112a_i2c_error(false);
	return 0;
}

static int nu2112a_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	struct nu2112a_device *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}

	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("nu2112a chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = __nu2112a_read_byte(client, reg, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int nu2112a_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;
	struct nu2112a_device *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}

	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("nu2112a chip is NULL\n");
		return -ENODEV;
	}
	mutex_lock(&chip->i2c_rw_lock);
	ret = __nu2112a_write_byte(client, reg, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int nu2112a_update_bits(struct i2c_client *client, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;
	struct nu2112a_device *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}

	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("nu2112a chip is NULL\n");
		return -ENODEV;
	}
	mutex_lock(&chip->i2c_rw_lock);
	ret = __nu2112a_read_byte(client, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __nu2112a_write_byte(client, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
out:
	mutex_unlock(&chip->i2c_rw_lock);
	return ret;
}

static s32 nu2112a_read_word(struct i2c_client *client, u8 reg)
{
	s32 ret;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	struct nu2112a_device *chip;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("nu2112a chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("i2c read word fail: can't read reg:0x%02X \n", reg);
		mutex_unlock(&chip->i2c_rw_lock);
		return ret;
	}
	nu2112a_i2c_error(false);
	mutex_unlock(&chip->i2c_rw_lock);
	return ret;
}

static s32 nu2112a_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	struct nu2112a_device *chip;

	if (voocphy == NULL) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("nu2112a chip is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("i2c write word fail: can't write 0x%02X to reg:0x%02X \n", val, reg);
		mutex_unlock(&chip->i2c_rw_lock);
		return ret;
	}
	nu2112a_i2c_error(false);
	mutex_unlock(&chip->i2c_rw_lock);
	return 0;
}

static int nu2112a_write_bit_mask(struct nu2112a_device *chip, u8 reg,
				  u8 mask, u8 data)
{
	u8 temp = 0;
	int rc = 0;

	rc = nu2112a_read_byte(chip->client, reg, &temp);
	if (rc < 0)
		return rc;

	temp = (data & mask) | (temp & (~mask));

	rc = nu2112a_write_byte(chip->client, reg, temp);
	if (rc < 0)
		return rc;

	return 0;
}

static int nu2112a_set_predata(struct oplus_voocphy_manager *chip, u16 val)
{
	s32 ret;
	if (!chip) {
		pr_err("failed: chip is null\n");
		return -1;
	}

	ret = nu2112a_write_word(chip->client, NU2112A_REG_31, val);
	if (ret < 0) {
		pr_err("failed: write predata\n");
		return -1;
	}
	pr_info("write predata 0x%0x\n", val);
	return ret;
}

static int nu2112a_set_txbuff(struct oplus_voocphy_manager *chip, u16 val)
{
	s32 ret;
	if (!chip) {
		pr_err("failed: chip is null\n");
		return -1;
	}

	ret = nu2112a_write_word(chip->client, NU2112A_REG_2C, val);
	if (ret < 0) {
		pr_err("write txbuff\n");
		return -1;
	}

	return ret;
}

static int nu2112a_get_adapter_info(struct oplus_voocphy_manager *chip)
{
	s32 data;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	data = nu2112a_read_word(chip->client, NU2112A_REG_2E);

	if (data < 0) {
		chg_err("nu2112a_read_word faile\n");
		return -1;
	}

	VOOCPHY_DATA16_SPLIT(data, chip->voocphy_rx_buff, chip->vooc_flag);
	chg_info("data: 0x%0x, vooc_flag: 0x%0x, vooc_rxdata: 0x%0x\n", data, chip->vooc_flag, chip->voocphy_rx_buff);

	return 0;
}

static void nu2112a_update_data(struct oplus_voocphy_manager *chip)
{
	u8 data_block[4] = { 0 };
	int i = 0;
	u8 data = 0;
	s32 ret = 0;

	/*int_flag*/
	nu2112a_read_byte(chip->client, NU2112A_REG_11, &data);
	chip->interrupt_flag = data;

	/*parse data_block for improving time of interrupt*/
	ret = i2c_smbus_read_i2c_block_data(chip->client, NU2112A_REG_20, 4,
					    data_block); /*REG20-21 vsys, REG22-23 vbat*/
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("nu2112a_update_data read vsys vbat error \n");
	} else {
		nu2112a_i2c_error(false);
	}
	for (i = 0; i < 4; i++) {
		pr_info("read vsys vbat data_block[%d] = %u\n", i, data_block[i]);
	}
	chip->cp_vsys = (((data_block[0] & NU2112A_VOUT_POL_H_MASK) << 8) | data_block[1]) * NU2112A_VOUT_ADC_LSB;
	chip->cp_vbat = (((data_block[2] & NU2112A_VOUT_POL_H_MASK) << 8) | data_block[3]) * NU2112A_VOUT_ADC_LSB;

	memset(data_block, 0, sizeof(u8) * 4);

	ret = i2c_smbus_read_i2c_block_data(chip->client, NU2112A_REG_1A, 4,
					    data_block); /*REG1A-1B ibus, REG1C-1D vbus*/
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("nu2112a_update_data read vsys vbat error \n");
	} else {
		nu2112a_i2c_error(false);
	}
	for (i = 0; i < 4; i++) {
		pr_info("read ichg vbus data_block[%d] = %u\n", i, data_block[i]);
	}

	chip->cp_ichg = (((data_block[0] & NU2112A_IBUS_POL_H_MASK) << 8) | data_block[1]) * NU2112A_IBUS_ADC_LSB;
	chip->cp_vbus = (((data_block[2] & NU2112A_VBUS_POL_H_MASK) << 8) | data_block[3]) * NU2112A_VBUS_ADC_LSB;

	memset(data_block, 0, sizeof(u8) * 4);

	ret = i2c_smbus_read_i2c_block_data(chip->client, NU2112A_REG_1E, 2, data_block); /*REG1E-1F vac*/
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("nu2112a_update_data read vac error\n");
	} else {
		nu2112a_i2c_error(false);
	}
	for (i = 0; i < 2; i++) {
		pr_info("read vac data_block[%d] = %u\n", i, data_block[i]);
	}

	chip->cp_vac = (((data_block[0] & NU2112A_VAC_POL_H_MASK) << 8) | data_block[1]) * NU2112A_VAC_ADC_LSB;

	pr_info("cp_ichg = %d cp_vbus = %d, cp_vsys = %d cp_vbat = %d cp_vac = "
		"%d int_flag = %d",
		chip->cp_ichg, chip->cp_vbus, chip->cp_vsys, chip->cp_vbat, chip->cp_vac, chip->interrupt_flag);
}

static int nu2112a_get_cp_ichg(struct oplus_voocphy_manager *chip)
{
	u8 data_block[2] = { 0 };
	int cp_ichg = 0;
	u8 cp_enable = 0;
	s32 ret = 0;

	nu2112a_get_chg_enable(chip, &cp_enable);

	if (cp_enable == 0)
		return 0;
	/*parse data_block for improving time of interrupt*/
	ret = i2c_smbus_read_i2c_block_data(chip->client, NU2112A_REG_1A, 2, data_block);
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("nu2112a read ichg error \n");
	} else {
		nu2112a_i2c_error(false);
	}

	cp_ichg = (((data_block[0] & NU2112A_IBUS_POL_H_MASK) << 8) | data_block[1]) * NU2112A_IBUS_ADC_LSB;

	return cp_ichg;
}

static int nu2112a_get_cp_vbat(struct oplus_voocphy_manager *chip)
{
	u8 data_block[2] = { 0 };
	s32 ret = 0;

	/*parse data_block for improving time of interrupt*/
	ret = i2c_smbus_read_i2c_block_data(chip->client, NU2112A_REG_22, 2, data_block);
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("nu2112a read vbat error \n");
	} else {
		nu2112a_i2c_error(false);
	}

	chip->cp_vbat = (((data_block[0] & NU2112A_VBAT_POL_H_MASK) << 8) | data_block[1]) * NU2112A_VBAT_ADC_LSB;

	return chip->cp_vbat;
}

static int nu2112a_get_cp_vbus(struct oplus_voocphy_manager *chip)
{
	u8 data_block[2] = { 0 };
	s32 ret = 0;

	/* parse data_block for improving time of interrupt */
	ret = i2c_smbus_read_i2c_block_data(chip->client, NU2112A_REG_1C, 2, data_block);
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("nu2112a read vbat error \n");
	} else {
		nu2112a_i2c_error(false);
	}

	chip->cp_vbus = (((data_block[0] & NU2112A_VBUS_POL_H_MASK) << 8) | data_block[1]) * NU2112A_VBUS_ADC_LSB;

	return chip->cp_vbus;
}

/*********************************************************************/
static int nu2112a_reg_reset(struct nu2112a_device *chip, bool enable)
{
	int ret;
	u8 val;
	if (enable)
		val = NU2112A_RESET_REG;
	else
		val = NU2112A_NO_REG_RESET;

	val <<= NU2112A_REG_RESET_SHIFT;

	ret = nu2112a_update_bits(chip->client, NU2112A_REG_07, NU2112A_REG_RESET_MASK, val);

	return ret;
}

static int nu2112a_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	ret = nu2112a_read_byte(chip->client, NU2112A_REG_07, data);
	if (ret < 0) {
		pr_err("NU2112A_REG_07\n");
		return -1;
	}

	*data = *data >> NU2112A_CHG_EN_SHIFT;

	return ret;
}

static int nu2112a_get_voocphy_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	ret = nu2112a_read_byte(chip->client, NU2112A_REG_2B, data);
	if (ret < 0) {
		pr_err("NU2112A_REG_2B\n");
		return -1;
	}

	return ret;
}

static void nu2112a_dump_reg_in_err_issue(struct oplus_voocphy_manager *chip)
{
	int i = 0, p = 0;
	if (!chip) {
		pr_err("!!!!! oplus_voocphy_manager chip NULL");
		return;
	}

	for (i = 0; i < 40; i++) {
		p = p + 1;
		nu2112a_read_byte(chip->client, i, &chip->reg_dump[p]);
	}
	pr_err("p[%d], ", p);

	return;
}

static void nu2112a_track_dump_reg(struct oplus_voocphy_manager *voocphy)
{
	struct nu2112a_device *chip = voocphy->priv_data;

	if (chip == NULL) {
		chg_err("nu2112a chip is NULL\n");
		return;
	}

	i2c_smbus_read_i2c_block_data(chip->client, TRACK_REG_ADDR_START, TRACK_REG_DUMP_NUM, chip->track_reg_dump);
}

#define ERR_MSG_BUF	PAGE_SIZE
__printf(3, 4)
static int nu2112a_publish_ic_err_msg(int type, int sub_type, const char *format, ...)
{
	va_list args;
	char *buf;
	int rc;
	struct mms_msg *topic_msg;
	struct oplus_mms *err_topic = oplus_mms_get_by_name("error");

	if (!err_topic)
		return -ENODEV;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	va_start(args, format);
	vsnprintf(buf, ERR_MSG_BUF, format, args);
	va_end(args);

	topic_msg =
		oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
					"[%s]-[%d]-[%d]:%s", "nu2112a", type, sub_type, buf);
	kfree(buf);
	if (topic_msg == NULL) {
		chg_err("alloc topic msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(err_topic, topic_msg);
	if (rc < 0) {
		chg_err("publish error topic msg error, rc=%d\n", rc);
		kfree(topic_msg);
	}

	return rc;
}

static void nu2112a_track_abnormal_upload_info_work(struct work_struct *work)
{
	struct nu2112a_device *chip =
		container_of(work, struct nu2112a_device, abnormal_upload_info_work);
	char *buf;
	int i;
	size_t index = 0;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return;

	if (chip->ic_status == NU2112A_IC_PIN_DIAG_FAIL)
		index += scnprintf(buf + index, ERR_MSG_BUF, "$$err_reason@@pin_diag_fail$$reg_info@@");
	else
		index += scnprintf(buf + index, ERR_MSG_BUF, "$$err_reason@@power_ng$$reg_info@@");

	for (i = 0; i < TRACK_REG_DUMP_NUM; i++)
		index += scnprintf(buf + index, ERR_MSG_BUF, "0x%04x=%02x,",
			(TRACK_REG_ADDR_START + i), chip->track_reg_dump[i]);
	if (index > 0)
		buf[index - 1] = 0;

	nu2112a_publish_ic_err_msg(OPLUS_IC_ERR_BURN, 0, "%s", buf);
	kfree(buf);
}

static bool nu2112a_ic_is_abnormal(struct oplus_voocphy_manager *chip)
{
	u8 data = 0;
	struct nu2112a_device *device_chip;
	enum nu2112a_ic_status ic_status;

	if (!chip) {
		chg_err("chip is NULL\n");
		return false;
	}

	device_chip = chip->priv_data;
	if (!device_chip) {
		chg_err("device chip is NULL\n");
		return false;
	}

	ic_status = device_chip->ic_status;
	nu2112a_read_byte(chip->client, NU2112A_REG_14, &data);

	if (data & NU2112A_PIN_DIAG_FALL_FLAG_MASK)
		device_chip->ic_status = NU2112A_IC_PIN_DIAG_FAIL;
	else if (data & NU2112A_POWER_NG_FLAG_MASK)
		device_chip->ic_status = NU2112A_IC_POWER_NG;
	else
		device_chip->ic_status = NU2112A_IC_OK;

	chg_info("reg[0x%x] = 0x%x, pre_ic_status:%d, ic_status:%d\n",
		NU2112A_REG_14, data, ic_status, device_chip->ic_status);
	if (device_chip->ic_status != NU2112A_IC_OK) {
		if (ic_status != device_chip->ic_status) {
			nu2112a_track_dump_reg(chip);
			if (NU2112A_REG_14 >= TRACK_REG_ADDR_START && NU2112A_REG_14 < TRACK_REG_ADDR_END)
				device_chip->track_reg_dump[NU2112A_REG_14 - TRACK_REG_ADDR_START] = data;
			schedule_work(&device_chip->abnormal_upload_info_work);
		}
		return true;
	}

	return false;
}

static int nu2112a_cp_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct nu2112a_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	nu2112a_dump_reg_in_err_issue(chip->voocphy);
	return 0;
}

static int nu2112a_cp_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int nu2112a_cp_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static int nu2112a_cp_set_iin(struct oplus_chg_ic_dev *ic_dev, int iin)
{
	return 0;
}

static void nu2112_hardware_init(struct nu2112a_device *chip)
{
	nu2112a_reg_reset(chip, true);
	nu2112a_init_device(chip);
}

static int nu2112a_cp_hw_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct nu2112a_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (chip->rested)
		return 0;

	nu2112_hardware_init(chip);
	return 0;
}

static bool nu2112a_check_work_mode_support(enum oplus_cp_work_mode mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_cp_support_work_mode); i++) {
		if (g_cp_support_work_mode[i] == mode)
			return true;
	}
	return false;
}

static int nu2112a_cp_check_work_mode_support(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	return nu2112a_check_work_mode_support(mode);
}

static int nu2112a_cp_set_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	struct nu2112a_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (!nu2112a_check_work_mode_support(mode)) {
		chg_err("not supported work mode, mode=%d\n", mode);
		return -EINVAL;
	}

	chip->mode = mode;
	chg_info("work mode=%d\n", mode);
	if (mode == CP_WORK_MODE_BYPASS)
		rc = nu2112a_vooc_hw_setting(chip);
	else
		rc = nu2112a_svooc_hw_setting(chip);

	if (rc < 0)
		chg_err("set work mode to %d error\n", mode);

	return rc;
}

static int nu2112a_cp_get_vin(struct oplus_chg_ic_dev *ic_dev, int *vin)
{
	struct nu2112a_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = nu2112a_get_cp_vbus(chip->voocphy);
	if (rc < 0) {
		chg_err("can't get cp vin, rc=%d\n", rc);
		return rc;
	}
	*vin = rc;

	return 0;
}

static int nu2112a_cp_get_iin(struct oplus_chg_ic_dev *ic_dev, int *iin)
{
	struct nu2112a_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = nu2112a_get_cp_ichg(chip->voocphy);
	if (rc < 0) {
		chg_err("can't get cp iin, rc=%d\n", rc);
		return rc;
	}
	*iin = rc;

	return 0;
}

static int nu2112a_cp_set_work_start(struct oplus_chg_ic_dev *ic_dev, bool start)
{
	struct nu2112a_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s work %s\n", chip->dev->of_node->name, start ? "start" : "stop");

	if (start && chip->mode != CP_WORK_MODE_BYPASS)
		nu2112a_set_osc_status(chip, NU2112A_OSC_ENABLE);
	else
		nu2112a_set_osc_status(chip, NU2112A_OSC_DISABLE);

	rc = nu2112a_set_chg_enable(chip->voocphy, start);
	if (rc < 0)
		return rc;
	oplus_imp_node_set_active(chip->input_imp_node, start);
	oplus_imp_node_set_active(chip->output_imp_node, start);

	return 0;
}

static int nu2112a_get_adc_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	ret = nu2112a_read_byte(chip->client, NU2112A_REG_18, data);
	if (ret < 0) {
		pr_err("NU2112A_REG_18\n");
		return -1;
	}

	*data = *data >> NU2112A_ADC_ENABLE_SHIFT;

	return ret;
}

static u8 nu2112a_match_err_value(struct oplus_voocphy_manager *chip, u8 *data_block)
{
	/* TODO */
	return 0;
}

static u8 nu2112a_get_int_value(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 data = 0;
	u8 data_block[NU2112A_FLAG_REG_NUMS] = { 0 };
	int i = 0;

	if (!chip) {
		pr_err("%s: chip null\n", __func__);
		return -1;
	}
	memset(data_block, 0, sizeof(u8) * NU2112A_FLAG_REG_NUMS);

	ret = i2c_smbus_read_i2c_block_data(chip->client, NU2112A_REG_11, NU2112A_FLAG_REG_NUMS, data_block);
	if (ret < 0) {
		nu2112a_i2c_error(true);
		pr_err("nu2112a_get_int_value read vac error\n");
	} else {
		nu2112a_i2c_error(false);
	}
	for (i = 0; i < NU2112A_FLAG_REG_NUMS; i++) {
		pr_info("read int data_block[%d] = %u\n", i, data_block[i]);
	}

	nu2112a_match_err_value(chip, data_block);

	return data;
}

static int nu2112a_set_chg_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	if (enable)
		return nu2112a_write_byte(chip->client, NU2112A_REG_07, 0x82); /*Enable CP, 500KHz*/
	else
		return nu2112a_write_byte(chip->client, NU2112A_REG_07, 0x2); /*Disable CP*/
}

static void nu2112a_set_pd_svooc_config(struct oplus_voocphy_manager *chip, bool enable)
{
	int ret = 0;
	u8 reg_data = 0;
	if (!chip) {
		pr_err("Failed\n");
		return;
	}

	if (enable) {
		reg_data = 0x81;
		nu2112a_write_byte(chip->client, NU2112A_REG_0C, 0x81); /*Enable IBUS_UCP*/
		nu2112a_write_byte(chip->client, NU2112A_REG_17, 0x28); /*IBUS_UCP_RISE_MASK*/
		nu2112a_write_byte(chip->client, NU2112A_REG_08, 0x03); /*WD=1000ms*/
	} else {
		reg_data = 0x00;
		nu2112a_write_byte(chip->client, NU2112A_REG_0C, 0x00);
	}

	ret = nu2112a_read_byte(chip->client, NU2112A_REG_0C, &reg_data);
	if (ret < 0) {
		pr_err("NU2112A_REG_0C\n");
		return;
	}
	pr_err("pd_svooc config NU2112A_REG_0C = %d\n", reg_data);
}

static bool nu2112a_get_pd_svooc_config(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 data = 0;

	if (!chip) {
		pr_err("Failed\n");
		return false;
	}

	ret = nu2112a_read_byte(chip->client, NU2112A_REG_0C, &data);
	if (ret < 0) {
		pr_err("NU2112A_REG_0C\n");
		return false;
	}

	pr_err("NU2112A_REG_0C = 0x%0x\n", data);

	data = data >> NU2112A_IBUS_UCP_DIS_SHIFT;
	if (data == NU2112A_IBUS_UCP_DISABLE)
		return true;
	else
		return false;
}

static int nu2112a_set_chg_pmid2out(struct oplus_voocphy_manager *chip, bool enable, int reason)
{
	if (enable) {
		if (reason == SETTING_REASON_SVOOC)
			return nu2112a_write_byte(chip->client, NU2112A_REG_05, 0x31); /*PMID/2-VOUT < 10%VOUT*/
		else if (reason == SETTING_REASON_VOOC)
			return nu2112a_write_byte(chip->client, NU2112A_REG_05, 0x33);
		else
			chg_err("no type for set_chg_pmid2out\n");
	} else {
		if (reason == SETTING_REASON_SVOOC)
			return nu2112a_write_byte(chip->client, NU2112A_REG_05, 0xB1); /*PMID/2-VOUT < 10%VOUT*/
		else if (reason == SETTING_REASON_VOOC)
			return nu2112a_write_byte(chip->client, NU2112A_REG_05, 0xA3);
		else
			chg_err("no type for set_chg_pmid2out\n");
	}

	return 0;
}

static bool nu2112a_get_chg_pmid2out(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 data = 0;

	ret = nu2112a_read_byte(chip->client, NU2112A_REG_05, &data);
	if (ret < 0) {
		chg_err("read NU2112A_REG_05 error\n");
		return false;
	}

	chg_info("NU2112A_REG_05 = 0x%0x\n", data);

	data = data >> NU2112A_PMID2OUT_OVP_DIS_SHIFT;
	if (data == NU2112A_PMID2OUT_OVP_ENABLE)
		return true;
	else
		return false;
}

static int nu2112a_set_adc_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	if (enable)
		return nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x90); /*Enable ADC*/
	else
		return nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x10); /*Disable ADC*/
}

static void nu2112a_send_handshake(struct oplus_voocphy_manager *chip)
{
	nu2112a_write_byte(chip->client, NU2112A_REG_2B, 0x81);
}

static int nu2112a_reset_voocphy(struct oplus_voocphy_manager *chip)
{
	u8 reg_data;

	/* turn off mos */
	nu2112a_write_byte(chip->client, NU2112A_REG_07, 0x02);
	nu2112a_set_osc_status(chip->priv_data, NU2112A_OSC_DISABLE);

	/* hwic config with plugout */
	reg_data = chip->ovp_reg;
	nu2112a_write_byte(chip->client, NU2112A_REG_00, reg_data); /* vbat ovp=4.65V */
	nu2112a_write_byte(chip->client, NU2112A_REG_02, 0x04); /* vac ovp=12V */
	nu2112a_write_byte(chip->client, NU2112A_REG_03, 0x50); /* VBUS OVP=10V */
	reg_data = chip->ocp_reg & 0x3f;
	nu2112a_write_byte(chip->client, NU2112A_REG_04, reg_data); /* IBUS OCP=3.6A */
	nu2112a_write_byte(chip->client, NU2112A_REG_08, 0x00); /* WD disable, cp 2:1 */
	nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x10); /* ADC Disable */

	/* clear tx data */
	nu2112a_write_byte(chip->client, NU2112A_REG_2C, 0x00); /* WDATA write 0 */
	nu2112a_write_byte(chip->client, NU2112A_REG_2D, 0x00);

	/* disable vooc phy irq */
	nu2112a_write_byte(chip->client, NU2112A_REG_30, 0x7f); /* VOOC MASK */

	/* disable vooc */
	nu2112a_write_byte(chip->client, NU2112A_REG_2B, 0x00);

	/* set predata */
	nu2112a_write_word(chip->client, NU2112A_REG_31, 0x0);

	/* mask insert irq */
	nu2112a_write_byte(chip->client, NU2112A_REG_15, 0x02);
	pr_err("oplus_vooc_reset_voocphy done");

	return VOOCPHY_SUCCESS;
}

static int nu2112a_reactive_voocphy(struct oplus_voocphy_manager *chip)
{
	u8 value;

	/*set predata to avoid cmd of adjust current(0x01)return error, add voocphy
   * bit0 hold time to 800us*/
	nu2112a_write_word(chip->client, NU2112A_REG_31, 0x0);
	nu2112a_read_byte(chip->client, NU2112A_REG_3A, &value);
	value = value | (3 << 5);
	nu2112a_write_byte(chip->client, NU2112A_REG_3A, value);

	/*dpdm*/
	nu2112a_write_byte(chip->client, NU2112A_REG_21, 0x21);
	nu2112a_write_byte(chip->client, NU2112A_REG_22, 0x00);
	nu2112a_write_byte(chip->client, NU2112A_REG_33, 0xD1);

	/*clear tx data*/
	nu2112a_write_byte(chip->client, NU2112A_REG_2C, 0x00);
	nu2112a_write_byte(chip->client, NU2112A_REG_2D, 0x00);

	/*vooc*/
	nu2112a_write_byte(chip->client, NU2112A_REG_30, 0x05);
	nu2112a_send_handshake(chip);

	pr_info("oplus_vooc_reactive_voocphy done");

	return VOOCPHY_SUCCESS;
}

static int nu2112a_read_data(struct nu2112a_device *chip, u8 addr, u8 *buf,
			     int len)
{
	u8 addr_buf = addr & 0xff;
	int rc = 0;

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_master_send(chip->client, &addr_buf, 1);
	if (rc < 1) {
		chg_err("read 0x%04x error, rc=%d\n", addr, rc);
		rc = rc < 0 ? rc : -EIO;
		goto error;
	}

	rc = i2c_master_recv(chip->client, buf, len);
	if (rc < len) {
		chg_err("read 0x%04x error, rc=%d\n", addr, rc);
		rc = rc < 0 ? rc : -EIO;
		goto error;
	}
	mutex_unlock(&chip->i2c_rw_lock);
	return rc;

error:
	mutex_unlock(&chip->i2c_rw_lock);
	return rc;
}

static int nu2112a_write_data(struct nu2112a_device *chip, u8 addr,
			      u16 length, u8 *data)
{
	u8 *buf;
	int rc = 0;

	buf = kzalloc(length + 1, GFP_KERNEL);
	if (!buf) {
		chg_err("alloc memorry for i2c buffer error\n");
		return -ENOMEM;
	}

	buf[0] = addr & 0xff;
	memcpy(&buf[1], data, length);

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_master_send(chip->client, buf, length + 1);
	if (rc < length + 1) {
		chg_err("write 0x%04x error, ret = %d \n", addr, rc);
		mutex_unlock(&chip->i2c_rw_lock);
		kfree(buf);
		rc = rc < 0 ? rc : -EIO;
		return rc;
	}
	mutex_unlock(&chip->i2c_rw_lock);
	kfree(buf);
	return rc;
}

static int nu2112a_retrieve_reg_flags(struct nu2112a_device *chip)
{
	unsigned int err_flag = 0;
	int rc = 0;
	u8 flag_buf[NU2112A_FLAG_NUM] = { 0 };

	rc = nu2112a_read_data(chip, NU2112A_ADDR_GENERAL_INT_FLAG1, flag_buf,
			       NU2112A_FLAG_NUM);
	if (rc < 0) {
		chg_err("failed to read flag register\n");
		return -EBUSY;
	}
	memcpy(chip->ufcs_reg_dump, flag_buf, NU2112A_FLAG_NUM);

	if (flag_buf[1] & NU2112A_FLAG_ACK_RECEIVE_TIMEOUT)
		err_flag |= BIT(UFCS_RECV_ERR_ACK_TIMEOUT);
	if (flag_buf[1] & NU2112A_FLAG_MSG_TRANS_FAIL)
		err_flag |= BIT(UFCS_RECV_ERR_TRANS_FAIL);
	if (flag_buf[1] & NU2112A_FLAG_RX_OVERFLOW)
		err_flag |= BIT(UFCS_COMM_ERR_RX_OVERFLOW);
	if (flag_buf[1] & NU2112A_FLAG_DATA_READY)
		err_flag |= BIT(UFCS_RECV_ERR_DATA_READY);
	if (flag_buf[1] & NU2112A_FLAG_SENT_PACKET_COMPLETE)
		err_flag |= BIT(UFCS_RECV_ERR_SENT_CMP);

	if (flag_buf[2] & NU2112A_FLAG_HARD_RESET)
		err_flag |= BIT(UFCS_HW_ERR_HARD_RESET);
	if (flag_buf[2] & NU2112A_FLAG_CRC_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_CRC_ERR);
	if (flag_buf[2] & NU2112A_FLAG_BAUD_RATE_CHANGE)
		err_flag |= BIT(UFCS_COMM_ERR_BAUD_RATE_CHANGE);
	if (flag_buf[2] & NU2112A_FLAG_LENGTH_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_RX_LEN_ERR);
	if (flag_buf[2] & NU2112A_FLAG_DATA_BYTE_TIMEOUT)
		err_flag |= BIT(UFCS_COMM_ERR_BYTE_TIMEOUT);
	if (flag_buf[2] & NU2112A_FLAG_TRAINING_BYTE_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_TRAINING_ERR);
	if (flag_buf[2] & NU2112A_FLAG_BAUD_RATE_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_BAUD_RATE_ERR);

	chip->ufcs->err_flag_save = err_flag;

	if (chip->ufcs->handshake_state == UFCS_HS_WAIT) {
		if ((flag_buf[1] & NU2112A_FLAG_HANDSHAKE_SUCCESS) &&
		    !(flag_buf[1] & NU2112A_FLAG_HANDSHAKE_FAIL)) {
			chip->ufcs->handshake_state = UFCS_HS_SUCCESS;
			err_flag = 0;
			chip->ufcs->err_flag_save = err_flag;
		} else if (flag_buf[1] & NU2112A_FLAG_HANDSHAKE_FAIL) {
			chip->ufcs->handshake_state = UFCS_HS_FAIL;
		}
	}
	chg_info("[0x%x, 0x%x, 0x%x], err_flag=0x%x\n", flag_buf[0], flag_buf[1], flag_buf[2],
		err_flag);

	return ufcs_set_error_flag(chip->ufcs, err_flag);
}

static void nu2112a_ufcs_event_handler(struct nu2112a_device *chip)
{
	/* set awake */
	nu2112a_retrieve_reg_flags(chip);
	ufcs_msg_handler(chip->ufcs);
}

static irqreturn_t nu2112a_charger_interrupt(int irq, void *dev_id)
{
	struct nu2112a_device *chip = dev_id;
	struct oplus_voocphy_manager *voocphy = chip->voocphy;

	if (chip->use_ufcs_phy && chip->ufcs_enable) {
		nu2112a_ufcs_event_handler(chip);
		return IRQ_HANDLED;
	} else if (chip->use_vooc_phy) {
		return oplus_voocphy_interrupt_handler(voocphy);
	}
	return IRQ_HANDLED;
}

static int nu2112a_init_device(struct nu2112a_device *chip)
{
	u8 reg_data;

	nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x10); /* ADC_CTRL:disable */
	nu2112a_write_byte(chip->client, NU2112A_REG_02, 0x4); /*VAC OVP=12V*/
	nu2112a_write_byte(chip->client, NU2112A_REG_03, 0x50); /* VBUS_OVP:10V */
	reg_data = chip->ovp_reg;
	nu2112a_write_byte(chip->client, NU2112A_REG_00, reg_data); /* VBAT_OVP:4.65V */
	reg_data = chip->ocp_reg & 0x3f;
	nu2112a_write_byte(chip->client, NU2112A_REG_04, reg_data); /* IBUS_OCP_UCP:3.6A */
	nu2112a_write_byte(chip->client, NU2112A_REG_0D, 0x01); /* IBUS UCP Falling =150ms */
	nu2112a_write_byte(chip->client, NU2112A_REG_09, 0x80); /* IBUS_UCP_RISE:1.28S */
	nu2112a_write_byte(chip->client, NU2112A_REG_0C, 0x01); /* IBUS UCP 150ma Falling,300ma Rising */

	nu2112a_write_byte(chip->client, NU2112A_REG_01, 0xa8); /*IBAT OCP Disable*/
	nu2112a_write_byte(chip->client, NU2112A_REG_2B, 0x00); /* VOOC_CTRL:disable */
	nu2112a_write_byte(chip->client, NU2112A_REG_35, 0x20); /*VOOC Option2*/
	nu2112a_write_byte(chip->client, NU2112A_REG_08, 0x0); /*VOOC Option2*/
	nu2112a_write_byte(chip->client, NU2112A_REG_17, 0x28); /*REG_17=0x28, IBUS_UCP_RISE_MASK_MASK*/
	nu2112a_write_byte(chip->client, NU2112A_REG_15, 0x02); /* mask insert irq */

	nu2112a_update_bits(chip->client, NU2112A_REG_0A, NU2112A_CFLY_PRECHG_TIMEOUT_MASK,
		NU2112A_CFLY_PRECHG_20_MS << NU2112A_CFLY_PRECHG_TIMEOUT_SHIFT);

	pr_err("nu2112a_init_device done");

	return 0;
}

static int nu2112a_init_vooc(struct oplus_voocphy_manager *voocphy)
{
	struct nu2112a_device *chip;

	pr_err(" >>>>start init vooc\n");

	chip = voocphy->priv_data;
	nu2112a_reg_reset(chip, true);
	nu2112a_init_device(chip);

	/* to avoid cmd of adjust current(0x01)return error, add voocphy bit0 hold time to 800us */
	/* SET PREDATA */
	nu2112a_write_word(chip->client, NU2112A_REG_31, 0x0);
	/*nu2112a_set_predata(0x0);*/
	nu2112a_write_byte(chip->client, NU2112A_REG_35, 0x20);

	/* dpdm */
	nu2112a_write_byte(chip->client, NU2112A_REG_33, 0xD1);

	/* vooc */
	nu2112a_write_byte(chip->client, NU2112A_REG_30, 0x05);

	return 0;
}

static int nu2112a_osc_enable(struct nu2112a_device *chip)
{
	int rc = 0;
	u8 data = 0;
	int retry_count = 1;

	do {
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x90);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x00);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x78);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x87);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0xAA);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x55);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_E7, 0x80);
		rc |= nu2112a_read_byte(chip->client, NU2112A_REG_E7, &data);
	} while (data != 0x80 && retry_count-- > 0);

	rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x00);
	chg_info("rc =%d, reg:0x%x\n", rc, data);

	if (data == 0x80)
		return 0;
	else
		return -1;
}

static int nu2112a_osc_disable(struct nu2112a_device *chip)
{
	int rc = 0;
	u8 data  = 0x80;
	int retry_count = 1;

	do {
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x90);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x00);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x78);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x87);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0xAA);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x55);
		rc |= nu2112a_write_byte(chip->client, NU2112A_REG_E7, 0x00);
		rc |= nu2112a_read_byte(chip->client, NU2112A_REG_E7, &data);
	} while (data != 0x00 && retry_count-- > 0);

	rc |= nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x00);
	rc |= nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x10);
	chg_info("rc =%d, reg:0x%x\n", rc, data);

	if (data == 0x00)
		return 0;
	else
		return -1;
}

static void nu2112a_force_osc_disable(struct nu2112a_device *chip)
{
	if (!chip || !chip->osc_ctrl_support || !chip->client)
		return;

	nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x90);
	nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x00);
	nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x78);
	nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x87);
	nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0xAA);
	nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x55);
	nu2112a_write_byte(chip->client, NU2112A_REG_E7, 0x00);
	nu2112a_write_byte(chip->client, NU2112A_REG_DE, 0x00);
	nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x10);
}

static void nu2112a_set_osc_status(
			struct nu2112a_device *chip, enum nu2112a_osc_status osc_status)
{
	int rc;

	if (!chip)
		return;

	if (!chip->osc_ctrl_support)
		return;

	mutex_lock(&chip->osc_status_lock);
	chg_info("current status:%d, set status:%d\n",
		chip->osc_status, osc_status);
	if (chip->osc_status == NU2112A_OSC_INVALID) {
		chg_info("osc status invalid, wait work recovery\n");
		mutex_unlock(&chip->osc_status_lock);
		return;
	}

	if (osc_status == chip->osc_status) {
		mutex_unlock(&chip->osc_status_lock);
		return;
	}

	switch (osc_status) {
	case NU2112A_OSC_INIT:
	case NU2112A_OSC_DISABLE:
		rc = nu2112a_osc_disable(chip);
		if (!rc) {
			chip->osc_status = NU2112A_OSC_DISABLE;
			mutex_unlock(&chip->osc_status_lock);
			cancel_delayed_work_sync(&chip->osc_status_daemon_work);
		} else {
			chg_err("disable osc fail, start work rerun\n");
			chip->osc_status = NU2112A_OSC_INVALID;
			mutex_unlock(&chip->osc_status_lock);
			if (osc_status == NU2112A_OSC_INIT)
				schedule_delayed_work(&chip->osc_status_daemon_work, msecs_to_jiffies(1500));
		}
		break;
	case NU2112A_OSC_ENABLE:
		rc = nu2112a_osc_enable(chip);
		chip->osc_status = NU2112A_OSC_ENABLE;
		mutex_unlock(&chip->osc_status_lock);
		cancel_delayed_work_sync(&chip->osc_status_daemon_work);
		schedule_delayed_work(&chip->osc_status_daemon_work, msecs_to_jiffies(1500));
		break;
	default:
		chip->osc_status = NU2112A_OSC_INVALID;
		mutex_unlock(&chip->osc_status_lock);
		chg_err("!!!not goto here\n");
		break;
	}
}

static void nu2112a_osc_status_daemon_work(struct work_struct *work)
{
	int rc;
	int cp_vbus;
	struct delayed_work *dwork = to_delayed_work(work);
	struct nu2112a_device *chip = container_of(dwork,
		struct nu2112a_device, osc_status_daemon_work);

	mutex_lock(&chip->osc_status_lock);
	if (chip->osc_status == NU2112A_OSC_DISABLE) {
		chg_err("osc status has disable, not need handle\n");
		mutex_unlock(&chip->osc_status_lock);
		return;
	}

	cp_vbus = nu2112a_get_cp_vbus(chip->voocphy);
	if (cp_vbus < 2000) { /* cp vbus less than 2000 mv detected */
		usleep_range(5000, 5000);
		cp_vbus = nu2112a_get_cp_vbus(chip->voocphy);
	}
	chg_info("cp_vbus=%d\n", cp_vbus);

	/* osc_status is invalid or cp vbus less than 2000 mv detected */
	if (chip->osc_status == NU2112A_OSC_INVALID || cp_vbus < 2000) {
		rc = nu2112a_osc_disable(chip);
		if (!rc) {
			chip->osc_status = NU2112A_OSC_DISABLE;
			mutex_unlock(&chip->osc_status_lock);
			return;
		}
		chg_err("disable osc fail, start work rerun\n");
	}
	mutex_unlock(&chip->osc_status_lock);
	schedule_delayed_work(&chip->osc_status_daemon_work, msecs_to_jiffies(1500));
}

static void nu2112a_osc_init(struct nu2112a_device *chip)
{
	mutex_init(&chip->osc_status_lock);
	INIT_DELAYED_WORK(&chip->osc_status_daemon_work, nu2112a_osc_status_daemon_work);
	chip->osc_status = NU2112A_OSC_DISABLE;
	nu2112a_set_osc_status(chip, NU2112A_OSC_INIT);
}

static int nu2112a_irq_gpio_init(struct oplus_voocphy_manager *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

		chip->irq_gpio = of_get_named_gpio(node, "oplus,irq_gpio", 0);
	if (!gpio_is_valid(chip->irq_gpio)) {
		chip->irq_gpio = of_get_named_gpio(node, "oplus_spec,irq_gpio", 0);
		if (!gpio_is_valid(chip->irq_gpio)) {
			chg_err("irq_gpio not specified, rc=%d\n", chip->irq_gpio);
			return chip->irq_gpio;
		}
	}
	rc = gpio_request(chip->irq_gpio, "irq_gpio");
	if (rc) {
		chg_err("unable to request gpio[%d]\n", chip->irq_gpio);
		return rc;
	}
	chg_info("irq_gpio = %d\n", chip->irq_gpio);

	chip->irq = gpio_to_irq(chip->irq_gpio);
	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->charging_inter_active =
	    pinctrl_lookup_state(chip->pinctrl, "charging_inter_active");
	if (IS_ERR_OR_NULL(chip->charging_inter_active)) {
		chg_err("failed to get the pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	chip->charging_inter_sleep =
	    pinctrl_lookup_state(chip->pinctrl, "charging_inter_sleep");
	if (IS_ERR_OR_NULL(chip->charging_inter_sleep)) {
		chg_err("Failed to get the pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	gpio_direction_input(chip->irq_gpio);
	pinctrl_select_state(chip->pinctrl, chip->charging_inter_active); /* no_PULL */
	rc = gpio_get_value(chip->irq_gpio);
	chg_info("irq_gpio = %d, irq_gpio_stat = %d\n", chip->irq_gpio, rc);

	return 0;
}

static int nu2112a_irq_register(struct nu2112a_device *chip)
{
	struct oplus_voocphy_manager *voocphy = chip->voocphy;
	struct irq_desc *desc;
	struct cpumask current_mask;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	cpumask_var_t cpu_highcap_mask;
#endif
	int ret;

	ret = nu2112a_irq_gpio_init(voocphy);
	if (ret < 0) {
		chg_err("failed to irq gpio init(%d)\n", ret);
		return ret;
	}
	pr_err(" nu2112a chip->irq = %d\n", voocphy->irq);
	if (voocphy->irq) {
		ret = request_threaded_irq(voocphy->irq, NULL, nu2112a_charger_interrupt,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "nu2112a_charger_irq", chip);
		if (ret < 0) {
			chg_err("request irq for irq=%d failed, ret =%d\n", voocphy->irq, ret);
			return ret;
		}
		enable_irq_wake(voocphy->irq);
	}
	pr_debug("request irq ok\n");
desc = irq_to_desc(voocphy->irq);
	if (desc == NULL) {
		free_irq(voocphy->irq, chip);
		chg_err("%s desc null\n", __func__);
		return ret;
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	update_highcap_mask(cpu_highcap_mask);
	cpumask_and(&current_mask, cpu_online_mask, cpu_highcap_mask);
#else
	cpumask_setall(&current_mask);
	cpumask_and(&current_mask, cpu_online_mask, &current_mask);
#endif
	ret = set_cpus_allowed_ptr(desc->action->thread, &current_mask);

	return 0;
}

int nu2112a_clk_err_clean(void)
{
	if (!oplus_voocphy_mg)
		return 0;

	nu2112a_write_byte(oplus_voocphy_mg->client, NU2112A_REG_2B, 0x02); /*reset vooc*/
	nu2112a_write_word(oplus_voocphy_mg->client, NU2112A_REG_31, 0x0);
	nu2112a_write_byte(oplus_voocphy_mg->client, NU2112A_REG_35, 0x20); /*dpdm */
	nu2112a_write_byte(oplus_voocphy_mg->client, NU2112A_REG_33, 0xD1);
	/*vooc */
	nu2112a_write_byte(oplus_voocphy_mg->client, NU2112A_REG_30, 0x05);
	msleep(15);
	nu2112a_write_byte(oplus_voocphy_mg->client, NU2112A_REG_2B, 0x80); /*enable phy*/

	pr_err("nu2112a_clk_err_clean done");
	return 0;
}

static int nu2112a_svooc_hw_setting(struct nu2112a_device *chip)
{
	u8 reg_data;

	nu2112a_write_byte(chip->client, NU2112A_REG_02, 0x04); /* VAC_OVP:12v */
	nu2112a_write_byte(chip->client, NU2112A_REG_03, 0x50); /* VBUS_OVP:10v */
	reg_data = chip->ocp_reg & 0x3f;
	nu2112a_write_byte(chip->client, NU2112A_REG_04, reg_data); /* IBUS_OCP_UCP:3.6A */
	nu2112a_write_byte(chip->client, NU2112A_REG_17, 0x28); /* Mask IBUS UCP rising */

	nu2112a_write_byte(chip->client, NU2112A_REG_08, 0x03); /* WD:1000ms */
	nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x90); /* ADC_CTRL:ADC_EN */
	nu2112a_write_byte(chip->client, NU2112A_REG_05, 0xB1); /* PMID/2-VOUT < 10%VOUT */

	nu2112a_write_byte(chip->client, NU2112A_REG_33, 0xd1); /* Loose_det=1 */
	nu2112a_write_byte(chip->client, NU2112A_REG_35, 0x20);

	return 0;
}

static int nu2112a_vooc_hw_setting(struct nu2112a_device *chip)
{
	nu2112a_write_byte(chip->client, NU2112A_REG_02, 0x06); /* VAC_OVP:7V */
	nu2112a_write_byte(chip->client, NU2112A_REG_03, 0x50); /* VBUS_OVP:10V */
	nu2112a_write_byte(chip->client, NU2112A_REG_04, 0x2B); /* IBUS_OCP_UCP:4.8A */
	nu2112a_write_byte(chip->client, NU2112A_REG_17, 0x28); /* Mask IBUS UCP rising */

	nu2112a_write_byte(chip->client, NU2112A_REG_08, 0x83); /* WD:1000ms */
	nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x90); /* ADC_CTRL:ADC_EN */
	nu2112a_write_byte(chip->client, NU2112A_REG_05, 0xA3); /* PMID/2-VOUT < 10%VOUT */
	nu2112a_write_byte(chip->client, NU2112A_REG_33, 0xd1); /* Loose_det=1 */
	nu2112a_write_byte(chip->client, NU2112A_REG_35, 0x20); /* VOOCPHY Option2 */

	return 0;
}

static int nu2112a_5v2a_hw_setting(struct nu2112a_device *chip)
{
	nu2112a_write_byte(chip->client, NU2112A_REG_02, 0x06); /* VAC_OVP:7V */
	nu2112a_write_byte(chip->client, NU2112A_REG_03, 0x50); /* VBUS_OVP:10V */

	nu2112a_write_byte(chip->client, NU2112A_REG_17, 0x28); /* Mask IBUS UCP rising */

	nu2112a_write_byte(chip->client, NU2112A_REG_08, 0x00); /* WD */
	nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x90); /* ADC_CTRL:ADC_EN */
	nu2112a_write_byte(chip->client, NU2112A_REG_33, 0xd1); /* Loose_det=1 */
	nu2112a_write_byte(chip->client, NU2112A_REG_35, 0x20); /* VOOCPHY Option2 */

	return 0;
}

static int nu2112a_pdqc_hw_setting(struct nu2112a_device *chip)
{
	nu2112a_write_byte(chip->client, NU2112A_REG_02, 0x04); /* VAC_OVP:12V */
	nu2112a_write_byte(chip->client, NU2112A_REG_03, 0x50); /* VBUS_OVP:10V */

	nu2112a_write_byte(chip->client, NU2112A_REG_08, 0x00); /* WD */
	nu2112a_write_byte(chip->client, NU2112A_REG_18, 0x10); /* ADC_CTRL:ADC_EN */
	nu2112a_write_byte(chip->client, NU2112A_REG_2B, 0x00); /* DISABLE VOOCPHY */

	pr_err("nu2112a_pdqc_hw_setting done");
	return 0;
}

static int nu2112a_hw_setting(struct oplus_voocphy_manager *voocphy, int reason)
{
	struct nu2112a_device *chip;

	if (!voocphy) {
		pr_err("voocphy is null exit\n");
		return -EINVAL;
	}
	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("nu2112a chip is NULL\n");
		return -ENODEV;
	}
	switch (reason) {
	case SETTING_REASON_PROBE:
	case SETTING_REASON_RESET:
		nu2112a_init_device(chip);
		pr_info("SETTING_REASON_RESET OR PROBE\n");
		break;
	case SETTING_REASON_SVOOC:
		nu2112a_svooc_hw_setting(chip);
		nu2112a_set_osc_status(chip, NU2112A_OSC_ENABLE);
		pr_info("SETTING_REASON_SVOOC\n");
		break;
	case SETTING_REASON_VOOC:
		nu2112a_vooc_hw_setting(chip);
		pr_info("SETTING_REASON_VOOC\n");
		break;
	case SETTING_REASON_5V2A:
		nu2112a_5v2a_hw_setting(chip);
		pr_info("SETTING_REASON_5V2A\n");
		break;
	case SETTING_REASON_PDQC:
		nu2112a_pdqc_hw_setting(chip);
		pr_info("SETTING_REASON_PDQC\n");
		break;
	default:
		pr_err("do nothing\n");
		break;
	}
	return 0;
}

static void nu2112a_voocphy_hardware_init(struct oplus_voocphy_manager *voocphy)
{
	struct nu2112a_device *chip;

	chip = voocphy->priv_data;
	if (chip == NULL) {
		chg_err("nu2112a chip is NULL\n");
		return;
	}

	nu2112a_reg_reset(chip, true);
	nu2112a_init_device(chip);
}

static ssize_t nu2112a_show_registers(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "nu2112a");
	for (addr = 0x0; addr <= 0x38; addr++) {
		ret = nu2112a_read_byte(chip->client, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx, "Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t nu2112a_store_register(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x38)
		nu2112a_write_byte(chip->client, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, 0660, nu2112a_show_registers, nu2112a_store_register);

static void nu2112a_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
}

static int nu2112a_parse_dt(struct nu2112a_device *chip)
{
	int rc;
	struct device_node *node = NULL;

	if (!chip) {
		pr_debug("chip null\n");
		return -1;
	}

	/* Parsing gpio switch gpio47*/
	node = chip->dev->of_node;

	chip->osc_ctrl_support = of_property_read_bool(node, "oplus,osc_ctrl_support");
	chg_info("osc_ctrl_support:%d\n", chip->osc_ctrl_support);

	rc = of_property_read_u32(node, "ovp_reg", &chip->ovp_reg);
	if (rc) {
		chip->ovp_reg = 0x5C;
	} else {
		chg_err("ovp_reg is %d\n", chip->ovp_reg);
	}

	rc = of_property_read_u32(node, "ocp_reg", &chip->ocp_reg);
	if (rc) {
		chip->ocp_reg = 0x24;
	} else {
		chg_err("ocp_reg is %d\n", chip->ocp_reg);
	}

	return 0;
}

static int nu2112a_get_chip_id(struct oplus_voocphy_manager *chip)
{
	return CHIP_ID_NU2112A;
}

static struct oplus_voocphy_operations oplus_nu2112a_ops = {
	.hardware_init = nu2112a_voocphy_hardware_init,
	.hw_setting = nu2112a_hw_setting,
	.init_vooc = nu2112a_init_vooc,
	.set_predata = nu2112a_set_predata,
	.set_txbuff = nu2112a_set_txbuff,
	.get_adapter_info = nu2112a_get_adapter_info,
	.update_data = nu2112a_update_data,
	.get_chg_enable = nu2112a_get_chg_enable,
	.set_chg_enable = nu2112a_set_chg_enable,
	.reset_voocphy = nu2112a_reset_voocphy,
	.reactive_voocphy = nu2112a_reactive_voocphy,
	.send_handshake = nu2112a_send_handshake,
	.get_cp_vbat = nu2112a_get_cp_vbat,
	.get_cp_vbus = nu2112a_get_cp_vbus,
	.get_int_value = nu2112a_get_int_value,
	.get_adc_enable = nu2112a_get_adc_enable,
	.set_adc_enable = nu2112a_set_adc_enable,
	.get_ichg = nu2112a_get_cp_ichg,
	.set_pd_svooc_config = nu2112a_set_pd_svooc_config,
	.get_pd_svooc_config = nu2112a_get_pd_svooc_config,
	.get_voocphy_enable = nu2112a_get_voocphy_enable,
	.get_chip_id = nu2112a_get_chip_id,
	.set_chg_pmid2out = nu2112a_set_chg_pmid2out,
	.get_chg_pmid2out = nu2112a_get_chg_pmid2out,
	.dump_voocphy_reg = nu2112a_dump_reg_in_err_issue,
	.ic_is_abnormal = nu2112a_ic_is_abnormal,
};

static int nu2112a_ufcs_init(struct ufcs_dev *ufcs)
{
	return 0;
}

static int nu2112a_ufcs_write_msg(struct ufcs_dev *ufcs, unsigned char *buf, int len)
{
	struct nu2112a_device *chip = ufcs->drv_data;
	int rc;

	rc = nu2112a_write_byte(chip->client, NU2112A_ADDR_TX_LENGTH, len);
	if (rc < 0) {
		chg_err("write tx buf len error, rc=%d\n", rc);
		return rc;
	}
	rc = nu2112a_write_data(chip, NU2112A_ADDR_TX_BUFFER0, len, buf);
	if (rc < 0) {
		chg_err("write tx buf error, rc=%d\n", rc);
		return rc;
	}
	nu2112a_write_bit_mask(chip, NU2112A_ADDR_UFCS_CTRL0,
			       NU2112A_MASK_SND_CMP, NU2112A_CMD_SND_CMP);
	if (rc < 0) {
		chg_err("write tx buf send cmd error, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int nu2112a_ufcs_read_msg(struct ufcs_dev *ufcs, unsigned char *buf, int len)
{
	struct nu2112a_device *chip = ufcs->drv_data;
	u8 rx_buf_len;
	int rc;

	rc = nu2112a_read_byte(chip->client, NU2112A_ADDR_RX_LENGTH, &rx_buf_len);
	if (rc < 0) {
		chg_err("can't read rx buf len, rc=%d\n", rc);
		return rc;
	}
	if (rx_buf_len > len) {
		chg_err("rx_buf_len = %d, limit to %d\n", rx_buf_len, len);
		rx_buf_len = len;
	}
	rc = nu2112a_read_data(chip, NU2112A_ADDR_RX_BUFFER0, buf, rx_buf_len);
	if (rc < 0) {
		chg_err("can't read rx buf, rc=%d\n", rc);
		return rc;
	}

	return (int)rx_buf_len;
}

static int nu2112a_ufcs_handshake(struct ufcs_dev *ufcs)
{
	struct nu2112a_device *chip = ufcs->drv_data;
	int rc;

	chg_info("ufcs handshake\n");
	rc = nu2112a_write_bit_mask(chip, NU2112A_ADDR_UFCS_CTRL0,
				    NU2112A_MASK_EN_HANDSHAKE,
				    NU2112A_CMD_EN_HANDSHAKE);
	if (rc < 0)
		chg_err("send handshake error, rc=%d\n", rc);

	return rc;
}

static int nu2112a_ufcs_source_hard_reset(struct ufcs_dev *ufcs)
{
	struct nu2112a_device *chip = ufcs->drv_data;
	int rc;
	int retry_count = 0;

retry:
	retry_count++;
	if (retry_count > UFCS_HARDRESET_RETRY_CNTS) {
		chg_err("send hard reset, retry count over!\n");
		return -EBUSY;
	}

	rc = nu2112a_write_bit_mask(chip, NU2112A_ADDR_UFCS_CTRL0,
				    SEND_SOURCE_HARDRESET,
				    SEND_SOURCE_HARDRESET);
	if (rc < 0) {
		chg_err("I2c send handshake error\n");
		goto retry;
	}

	msleep(100);
	return 0;
}

static int nu2112a_ufcs_cable_hard_reset(struct ufcs_dev *ufcs)
{
	return 0;
}

static int nu2112a_ufcs_set_baud_rate(struct ufcs_dev *ufcs, enum ufcs_baud_rate baud)
{
	struct nu2112a_device *chip = ufcs->drv_data;
	int rc;

	if (baud == UFCS_BAUD_115200) {
		baud = UFCS_BAUD_57600;
		chg_info("force modify baud 115200 to 57600\n");
	}

	rc = nu2112a_write_bit_mask(chip, NU2112A_ADDR_UFCS_CTRL0,
				FLAG_BAUD_RATE_VALUE,
				(baud << FLAG_BAUD_NUM_SHIFT));
	if (rc < 0)
		chg_err("set baud rate error, rc=%d\n", rc);

	return rc;
}

static int nu2112a_ufcs_enable(struct ufcs_dev *ufcs)
{
	struct nu2112a_device *chip = ufcs->drv_data;
	int i, rc;
	u8 addr_buf[NU2112A_ENABLE_REG_NUM] = { NU2112A_ADDR_UFCS_CTRL0,
						NU2112A_ADDR_GENERAL_INT_FLAG1,
						NU2112A_ADDR_UFCS_INT_MASK0,
						NU2112A_ADDR_UFCS_INT_MASK1,
						NU2112A_ADDR_UFCS_OPTION};
	u8 cmd_buf[NU2112A_ENABLE_REG_NUM] = { NU2112A_CMD_EN_CHIP,
		NU2112A_CMD_CLR_TX_RX, NU2112A_CMD_MASK_ACK_TIMEOUT,
		NU2112A_MASK_TRANING_BYTE_ERROR, NU2112A_BUFFER_OPTION_CONFIG
	};

	chip->rested = true;
	nu2112a_reg_reset(chip, true);
	msleep(10);
	nu2112a_init_device(chip);

	for (i = 0; i < NU2112A_ENABLE_REG_NUM; i++) {
		rc = nu2112a_write_byte(chip->client, addr_buf[i], cmd_buf[i]);
		if (rc < 0) {
			chg_err("write i2c failed!\n");
			return rc;
		}
	}
	chip->ufcs_enable = true;

	rc = nu2112a_write_byte(chip->client, NU2112A_REG_08, NU2112A_WATCHDOG_1S); /* WD:1000ms */
	if (rc < 0) {
		chg_err("failed to set nu2112a_ufcs_enable (%d)\n", rc);
		return rc;
	}

	ufcs_clr_error_flag(chip->ufcs);

	return 0;
}

static int nu2112a_ufcs_disable(struct ufcs_dev *ufcs)
{
	struct nu2112a_device *chip = ufcs->drv_data;
	int rc;

	chip->ufcs_enable = false;
	chip->rested = false;
	rc = nu2112a_write_byte(chip->client, NU2112A_ADDR_UFCS_CTRL0,
				NU2112A_CMD_DIS_CHIP);
	if (rc < 0) {
		chg_err("write i2c failed\n");
		return rc;
	}

	rc = nu2112a_write_byte(chip->client, NU2112A_REG_08, NU2112A_WATCHDOG_DIS); /* dsiable wdt */
	if (rc < 0) {
		chg_err("failed to set nu2112a_ufcs_disable (%d)\n", rc);
		return rc;
	}

	return 0;
}

static int nu2112a_get_wdt_reg_by_time(unsigned int time_ms, u8 *reg)
{
	if (reg == NULL)
		return -EINVAL;

	if (time_ms == 0) {
		*reg = NU2112A_WATCHDOG_DIS;
	} else if (time_ms <= 200) {
		*reg = NU2112A_WATCHDOG_200MS;
	} else if (time_ms <= 500) {
		*reg = NU2112A_WATCHDOG_500MS;
	} else if (time_ms <= 1000) {
		*reg = NU2112A_WATCHDOG_1S;
	} else if (time_ms <= 5000) {
		*reg = NU2112A_WATCHDOG_5S;
	} else if (time_ms <= 30000) {
		*reg = NU2112A_WATCHDOG_30S;
	} else {
		chg_err("nu2112a watchdog not support %dms(>30s)\n", time_ms);
		return -EINVAL;
	}

	return 0;
}

static int nu2112a_ufcs_watchdog_config(struct ufcs_dev *ufcs, unsigned int time_ms)
{
	struct nu2112a_device *chip = ufcs->drv_data;
	u8 reg_val = 0;
	int rc = 0;

	if (chip == NULL) {
		chg_err("nu2112a chip is NULL\n");
		return -ENODEV;
	}

	rc = nu2112a_get_wdt_reg_by_time(time_ms, &reg_val);
	if (rc < 0)
		return rc;

	if (reg_val == NU2112A_WATCHDOG_DIS) {
		rc = nu2112a_update_bits(chip->client, NU2112A_REG_08,
		                         NU2112A_WATCHDOG_MASK, NU2112A_WATCHDOG_DIS); /* dsiable wdt */
		chg_info("watchdog_config disable (%d) ok!\n", reg_val);
	} else {
		rc = nu2112a_update_bits(chip->client, NU2112A_REG_08,
		                         NU2112A_WATCHDOG_MASK, reg_val);
		chg_info("watchdog_config set (%d) ok!\n", reg_val);
	}
	if (rc < 0) {
		chg_err("failed to cp_watchdog_config (%d)\n", rc);
		return rc;
	}

	return 0;
}

static int nu2112a_ufcs_baudrate_end_check_config(struct ufcs_dev *ufcs)
{
	struct nu2112a_device *chip = ufcs->drv_data;
	int rc = 0;

	if (chip == NULL) {
		chg_err("nu2112a chip is NULL\n");
		return -ENODEV;
	}

	rc = nu2112a_update_bits(chip->client, NU2112A_ADDR_UFCS_OPTION,
		NU2112A_BAUDRATE_CHECK_CONFIG | NU2112A_END_CHECK_ENABLE,
		NU2112A_BAUDRATE_CHECK_CONFIG | NU2112A_END_CHECK_ENABLE);

	if (rc < 0) {
		chg_err("failed to config baud_check to +-20Per and enable end_check (%d)\n", rc);
		return rc;
	}

	return 0;
}

static struct ufcs_dev_ops ufcs_ops = {
	.init = nu2112a_ufcs_init,
	.write_msg = nu2112a_ufcs_write_msg,
	.read_msg = nu2112a_ufcs_read_msg,
	.handshake = nu2112a_ufcs_handshake,
	.source_hard_reset = nu2112a_ufcs_source_hard_reset,
	.cable_hard_reset = nu2112a_ufcs_cable_hard_reset,
	.set_baud_rate = nu2112a_ufcs_set_baud_rate,
	.enable = nu2112a_ufcs_enable,
	.disable = nu2112a_ufcs_disable,
	.watchdog_config = nu2112a_ufcs_watchdog_config,
	.baudrate_end_check_config = nu2112a_ufcs_baudrate_end_check_config,
};

static int nu2112a_charger_choose(struct nu2112a_device *chip)
{
	int ret;

	if (!oplus_voocphy_chip_is_null()) {
		pr_err("oplus_voocphy_chip already exists!");
		return 0;
	} else {
		ret = i2c_smbus_read_byte_data(chip->client, 0x07);
		pr_err("0x07 = %d\n", ret);
		if (ret < 0) {
			pr_err("i2c communication fail");
			return -EPROBE_DEFER;
		} else
			return 1;
	}
}

static int nu2112a_cp_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int nu2112a_cp_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static int nu2112a_cp_get_work_status(struct oplus_chg_ic_dev *ic_dev, bool *start)
{
	struct nu2112a_device *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = nu2112a_read_byte(chip->client, NU2112A_REG_07, &data);
	if (rc < 0) {
		chg_err("read NU2112A_REG_07 error, rc=%d\n", rc);
		return rc;
	}

	*start = data & BIT(7);

	return 0;
}

static int nu2112a_cp_adc_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct nu2112a_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	return nu2112a_set_adc_enable(chip->voocphy, en);

	return 0;
}

static int nu2112a_cp_wd_enable(struct oplus_chg_ic_dev *ic_dev, int timeout_ms)
{
	struct nu2112a_device *chip;
	u8 reg_val = 0;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("set watchdog timeout to %dms\n", timeout_ms);

	ret = nu2112a_get_wdt_reg_by_time(timeout_ms, &reg_val);
	if (ret < 0)
		return ret;

	ret = nu2112a_update_bits(chip->client, NU2112A_REG_08,
				NU2112A_WATCHDOG_MASK, reg_val);
	chg_info("set watchdog reg to 0x%02x ok!\n", reg_val);

	if (ret < 0) {
		chg_err("failed to set watchdog reg to 0x%02x, ret=%d\n", reg_val, ret);
		return ret;
	}

	return 0;
}

static int nu2112a_cp_set_ucp_disable(struct oplus_chg_ic_dev *ic_dev, bool disable)
{
	struct nu2112a_device *chip;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s %s\n", chip->dev->of_node->name, disable ? "disable" : "enable");
	if (disable)
		ret = nu2112a_update_bits(chip->client, NU2112A_REG_0C,
				NU2112A_IBUS_UCP_DIS_MASK, NU2112A_IBUS_UCP_DISABLE << NU2112A_IBUS_UCP_DIS_SHIFT);
	else
		ret = nu2112a_update_bits(chip->client, NU2112A_REG_0C,
				NU2112A_IBUS_UCP_DIS_MASK, NU2112A_IBUS_UCP_ENABLE << NU2112A_IBUS_UCP_DIS_SHIFT);

	if (ret < 0) {
		chg_err("failed to set ucp reg disable to 0x%02x, ret=%d\n", disable, ret);
		return ret;
	}

	return 0;
}

static void *nu2112a_cp_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, nu2112a_cp_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, nu2112a_cp_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, nu2112a_cp_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, nu2112a_cp_smt_test);
		break;
	case OPLUS_IC_FUNC_CP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_ENABLE, nu2112a_cp_enable);
		break;
	case OPLUS_IC_FUNC_CP_HW_INTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_HW_INTI, nu2112a_cp_hw_init);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_MODE, nu2112a_cp_set_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_MODE:
		/* func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_MODE, sc8547a_cp_get_work_mode); */
		break;
	case OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT,
				nu2112a_cp_check_work_mode_support);
		break;
	case OPLUS_IC_FUNC_CP_SET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_IIN, nu2112a_cp_set_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VIN, nu2112a_cp_get_vin);
		break;
	case OPLUS_IC_FUNC_CP_GET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IIN, nu2112a_cp_get_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VOUT:
		/* func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VOUT, sc8547a_cp_get_vout); */
		break;
	case OPLUS_IC_FUNC_CP_GET_IOUT:
		/* func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IOUT, sc8547a_cp_get_iout); */
		break;
	case OPLUS_IC_FUNC_CP_GET_VAC:
		/* func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VAC, sc8547a_cp_get_vac); */
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START, nu2112a_cp_set_work_start);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_STATUS, nu2112a_cp_get_work_status);
		break;
	case OPLUS_IC_FUNC_CP_SET_ADC_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, nu2112a_cp_adc_enable);
		break;
	case OPLUS_IC_FUNC_CP_SET_UCP_DISABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_UCP_DISABLE, nu2112a_cp_set_ucp_disable);
		break;
	case OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE, nu2112a_cp_wd_enable);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq nu2112a_cp_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int nu2112a_ic_register(struct nu2112a_device *chip)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *child;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg;
	int rc;

	for_each_child_of_node(chip->dev->of_node, child) {
		rc = of_property_read_u32(child, "oplus,ic_type", &ic_type);
		if (rc < 0)
			continue;
		rc = of_property_read_u32(child, "oplus,ic_index", &ic_index);
		if (rc < 0)
			continue;
		ic_cfg.name = child->name;
		ic_cfg.index = ic_index;
		ic_cfg.type = ic_type;
		ic_cfg.priv_data = chip;
		ic_cfg.of_node = child;
		switch (ic_type) {
		case OPLUS_CHG_IC_CP:
			/* TODO: (void)nu2112a_init_imp_node(chip, child); */
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-nu2112a:%d", ic_index);
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = nu2112a_cp_get_func;
			ic_cfg.virq_data = nu2112a_cp_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(nu2112a_cp_virq_table);
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_type);
			continue;
		}
		ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
		if (!ic_dev) {
			rc = -ENODEV;
			chg_err("register %s error\n", child->name);
			continue;
		}
		chg_info("register %s\n", child->name);

		switch (ic_dev->type) {
		case OPLUS_CHG_IC_CP:
			chip->cp_ic = ic_dev;
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_dev->type);
			continue;
		}

		of_platform_populate(child, NULL, NULL, chip->dev);
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static int nu2112a_driver_probe(struct i2c_client *client)
#else
static int nu2112a_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct nu2112a_device *chip;
	struct oplus_voocphy_manager *voocphy;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(struct nu2112a_device), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc nu2112a device buf error\n");
		return -ENOMEM;
	}

	voocphy = devm_kzalloc(&client->dev, sizeof(struct oplus_voocphy_manager), GFP_KERNEL);
	if (!voocphy) {
		dev_err(&client->dev, "alloc voocphy buf error\n");
		ret = -ENOMEM;
		goto chg_err;
	}

	chip->client = client;
	chip->dev = &client->dev;
	voocphy->client = client;
	voocphy->dev = &client->dev;
	voocphy->priv_data = chip;
	chip->voocphy = voocphy;
	mutex_init(&chip->i2c_rw_lock);
	mutex_init(&chip->chip_lock);
	mutex_init(&i2c_rw_lock);

	i2c_set_clientdata(client, voocphy);

	ret = nu2112a_parse_dt(chip);
	if (ret < 0)
		goto parse_dt_err;

	ret = nu2112a_charger_choose(chip);
	if (ret <= 0) {
		chg_err("choose error, rc=%d\n", ret);
		goto regmap_init_err;
	}

	nu2112a_osc_init(chip);
	nu2112a_create_device_node(&(client->dev));
	INIT_WORK(&chip->abnormal_upload_info_work, nu2112a_track_abnormal_upload_info_work);

	chip->use_vooc_phy = of_property_read_bool(chip->dev->of_node, "oplus,use_vooc_phy");
	chip->use_ufcs_phy = of_property_read_bool(chip->dev->of_node, "oplus,use_ufcs_phy");
	chip->vac_support = of_property_read_bool(chip->dev->of_node, "oplus,vac_support");
	chg_info("use_vooc_phy=%d, use_ufcs_phy=%d, vac_support=%d\n",
		 chip->use_vooc_phy, chip->use_ufcs_phy, chip->vac_support);

	if (chip->use_vooc_phy) {
		voocphy->ops = &oplus_nu2112a_ops;
		ret = oplus_register_voocphy(voocphy);
		if (ret < 0) {
			chg_err("failed to register voocphy, ret = %d", ret);
			goto reg_voocphy_err;
		}
	}

	if (chip->use_ufcs_phy) {
		chip->ufcs = ufcs_device_register(chip->dev, &ufcs_ops, chip, &nu2112a_ufcs_config);
		if (IS_ERR_OR_NULL(chip->ufcs)) {
			chg_err("ufcs device register error\n");
			ret = -ENODEV;
			goto reg_ufcs_err;
		}
	}

	ret = nu2112a_irq_register(chip);
	if (ret < 0) {
		if (chip->use_vooc_phy || chip->use_ufcs_phy) {
			chg_err("irq register error\n");
			goto irq_reg_err;
		}
	}

	ret = nu2112a_ic_register(chip);
	if (ret < 0) {
		chg_err("cp ic register error\n");
		goto cp_reg_err;
	}

	chip->ufcs_enable = false;

	nu2112a_cp_init(chip->cp_ic);
	oplus_voocphy_mg = voocphy;

	chg_info("nu2112a(%s) probe successfully\n", chip->dev->of_node->name);

	return 0;

cp_reg_err:
	if (chip->input_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->input_imp_node);
	if (chip->output_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->output_imp_node);
irq_reg_err:
	if (chip->use_ufcs_phy)
		ufcs_device_unregister(chip->ufcs);
reg_ufcs_err:
reg_voocphy_err:
regmap_init_err:
parse_dt_err:
	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, voocphy);
chg_err:
	devm_kfree(&client->dev, chip);
	return ret;
}

static int nu2112a_pm_resume(struct device *dev_chip)
{
	struct i2c_client *client = container_of(dev_chip, struct i2c_client, dev);
	struct oplus_voocphy_manager *chip = i2c_get_clientdata(client);

	if (chip == NULL)
		return 0;

	return 0;
}

static int nu2112a_pm_suspend(struct device *dev_chip)
{
	struct i2c_client *client = container_of(dev_chip, struct i2c_client, dev);
	struct oplus_voocphy_manager *chip = i2c_get_clientdata(client);

	if (chip == NULL)
		return 0;

	return 0;
}

static const struct dev_pm_ops nu2112a_pm_ops = {
	.resume = nu2112a_pm_resume,
	.suspend = nu2112a_pm_suspend,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void nu2112a_driver_remove(struct i2c_client *client)
{
	struct oplus_voocphy_manager *chip = i2c_get_clientdata(client);

	if (!chip)
		return;

	devm_kfree(&client->dev, chip);
	return;
}
#else
static int nu2112a_driver_remove(struct i2c_client *client)
{
	struct oplus_voocphy_manager *chip = i2c_get_clientdata(client);

	if (chip == NULL)
		return -ENODEV;

	devm_kfree(&client->dev, chip);

	return 0;
}
#endif

static void nu2112a_shutdown(struct i2c_client *client)
{
	struct oplus_voocphy_manager *chip;

	nu2112a_write_byte(client, NU2112A_REG_18, 0x10);

	chip = i2c_get_clientdata(client);
	if (chip)
		nu2112a_force_osc_disable(chip->priv_data);

	return;
}

static const struct of_device_id nu2112a_match[] = {
	{.compatible = "oplus,nu2112a" },
	{},
};

static const struct i2c_device_id nu2112a_id[] = {
	{ "oplus,nu2112a", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, nu2112a_id);

static struct i2c_driver nu2112a_i2c_driver = {
	.driver =
		{
			.name = "nu2112a",
			.owner = THIS_MODULE,
			.of_match_table = nu2112a_match,
			.pm = &nu2112a_pm_ops,
		},
	.probe = nu2112a_driver_probe,
	.remove = nu2112a_driver_remove,
	.id_table = nu2112a_id,
	.shutdown = nu2112a_shutdown,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
static int __init nu2112a_i2c_driver_init(void)
{
	int ret = 0;
	chg_err(" init start\n");

	if (i2c_add_driver(&nu2112a_i2c_driver) != 0) {
		chg_err(" failed to register nu2112a i2c driver.\n");
	} else {
		chg_err(" Success to register nu2112a i2c driver.\n");
	}

	return ret;
}

subsys_initcall(nu2112a_i2c_driver_init);
#else
static __init int nu2112a_i2c_driver_init(void)
{
	return i2c_add_driver(&nu2112a_i2c_driver);
}

static __exit void nu2112a_i2c_driver_exit(void)
{
	i2c_del_driver(&nu2112a_i2c_driver);
}

oplus_chg_module_register(nu2112a_i2c_driver);
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)*/

MODULE_DESCRIPTION("SC NU2112A MASTER VOOCPHY&UFCS Driver");
MODULE_LICENSE("GPL v2");
