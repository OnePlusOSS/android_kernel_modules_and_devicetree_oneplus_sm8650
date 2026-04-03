// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[nu2112a] %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/proc_fs.h>

#include <trace/events/sched.h>
#include <linux/ktime.h>

#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_impedance_check.h>
#include <oplus_chg_monitor.h>
#include "../voocphy/oplus_voocphy.h"
#include "oplus_hal_nu2112a.h"
#include <oplus_mms_wired.h>
#include <oplus_chg_vooc.h>
#define DEFAULT_OVP_REG_CONFIG	0x5C
#define DEFAULT_OCP_REG_CONFIG	0x24
#define TRACK_REG_ADDR_START	NU2112A_REG_07
#define TRACK_REG_ADDR_END	NU2112A_REG_15
#define TRACK_REG_DUMP_NUM	(TRACK_REG_ADDR_END - TRACK_REG_ADDR_START)

enum nu2112a_osc_status {
	NU2112A_OSC_INIT,
	NU2112A_OSC_ENABLE,
	NU2112A_OSC_DISABLE,
	NU2112A_OSC_INVALID,
};

enum nu2112a_slave_ic_status {
	NU2112A_SLAVE_IC_OK,
	NU2112A_SLAVE_IC_PIN_DIAG_FAIL,
	NU2112A_SLAVE_IC_POWER_NG,
};

static struct oplus_voocphy_manager *oplus_voocphy_mg = NULL;
static struct mutex i2c_rw_lock;
static bool error_reported = false;
static int slave_ovp_reg = DEFAULT_OVP_REG_CONFIG;
static int slave_ocp_reg = DEFAULT_OCP_REG_CONFIG;

struct nu2112a_slave_device {
	struct device *slave_dev;
	struct i2c_client *slave_client;
	struct oplus_voocphy_manager *voocphy;
	struct oplus_chg_ic_dev *cp_ic;

	enum oplus_cp_work_mode cp_work_mode;
	enum nu2112a_osc_status osc_status;
	struct mutex osc_status_lock;
	struct delayed_work osc_status_daemon_work;
	bool osc_ctrl_support;
	enum oplus_cp_work_mode mode;

	u8 track_reg_dump[TRACK_REG_DUMP_NUM];
	struct work_struct abnormal_upload_info_work;
	enum nu2112a_slave_ic_status ic_status;
};

static enum oplus_cp_work_mode g_cp_support_work_mode[] = {
	CP_WORK_MODE_BYPASS,
	CP_WORK_MODE_2_TO_1,
};

static struct nu2112a_slave_device *g_device_chip = NULL;
static int nu2112a_slave_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data);
static void nu2112a_slave_set_osc_status(
			struct nu2112a_slave_device *chip, enum nu2112a_osc_status osc_status);

#define I2C_ERR_NUM 10
#define SLAVE_I2C_ERROR (1 << 1)

static void nu2112a_slave_i2c_error(bool happen)
{
	int report_flag = 0;
	if (!oplus_voocphy_mg || error_reported)
		return;

	if (happen) {
		oplus_voocphy_mg->slave_voocphy_iic_err = 1;
		oplus_voocphy_mg->slave_voocphy_iic_err_num++;
		if (oplus_voocphy_mg->slave_voocphy_iic_err_num >= I2C_ERR_NUM) {
			report_flag |= SLAVE_I2C_ERROR;
			error_reported = true;
		}
	} else {
		oplus_voocphy_mg->slave_voocphy_iic_err_num = 0;
	}
}

/************************************************************************/
static int __nu2112a_slave_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		nu2112a_slave_i2c_error(true);
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}
	nu2112a_slave_i2c_error(false);
	*data = (u8)ret;

	return 0;
}

static int __nu2112a_slave_write_byte(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		nu2112a_slave_i2c_error(true);
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n", val, reg, ret);
		return ret;
	}
	nu2112a_slave_i2c_error(false);

	return 0;
}

static int nu2112a_slave_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&i2c_rw_lock);
	ret = __nu2112a_slave_read_byte(client, reg, data);
	mutex_unlock(&i2c_rw_lock);

	return ret;
}

static int nu2112a_slave_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&i2c_rw_lock);
	ret = __nu2112a_slave_write_byte(client, reg, data);
	mutex_unlock(&i2c_rw_lock);

	return ret;
}

static int nu2112a_slave_update_bits(struct i2c_client *client, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&i2c_rw_lock);
	ret = __nu2112a_slave_read_byte(client, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __nu2112a_slave_write_byte(client, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
out:
	mutex_unlock(&i2c_rw_lock);
	return ret;
}

static void nu2112a_slave_update_data(struct oplus_voocphy_manager *chip)
{
	u8 data_block[2] = { 0 };
	int i = 0;
	u8 int_flag = 0;
	s32 ret = 0;

	nu2112a_slave_read_byte(chip->slave_client, NU2112A_REG_11, &int_flag);

	/*parse data_block for improving time of interrupt*/
	ret = i2c_smbus_read_i2c_block_data(chip->slave_client, NU2112A_REG_1A, 2, data_block);
	if (ret < 0) {
		nu2112a_slave_i2c_error(true);
		pr_err("nu2112a_update_data slave read error \n");
	} else {
		nu2112a_slave_i2c_error(false);
	}
	for (i = 0; i < 2; i++) {
		pr_info("data_block[%d] = %u\n", i, data_block[i]);
	}
	chip->slave_cp_ichg = ((data_block[0] << 8) | data_block[1]) * NU2112A_IBUS_ADC_LSB;
	if (oplus_voocphy_mg)
		oplus_voocphy_mg->slave_cp_ichg = chip->slave_cp_ichg;
	pr_info("slave cp_ichg = %d int_flag = %d", chip->slave_cp_ichg, int_flag);
}
/*********************************************************************/
static int oplus_chg_get_vooc_charging(void)
{
	int vooc_charging_status = 0;
	struct oplus_mms *vooc_topic;
	union mms_msg_data data = { 0 };
	int rc;

	vooc_topic = oplus_mms_get_by_name("vooc");
	if (!vooc_topic)
		return 0;

	rc = oplus_mms_get_item_data(vooc_topic, VOOC_ITEM_VOOC_CHARGING, &data, true);
	if (!rc)
		vooc_charging_status = data.intval;

	return vooc_charging_status;
}

int nu2112a_slave_get_ichg(struct oplus_voocphy_manager *chip)
{
	u8 slave_cp_enable;

	if (oplus_chg_get_vooc_charging()) {
		if (oplus_voocphy_mg)
			return oplus_voocphy_mg->slave_cp_ichg;
	}

	nu2112a_slave_update_data(chip);

	nu2112a_slave_get_chg_enable(chip, &slave_cp_enable);
	if (chip->slave_ops) {
		if (slave_cp_enable == 1)
			return chip->slave_cp_ichg;
		else
			return 0;
	} else {
		return 0;
	}
}

static int nu2112a_slave_get_cp_status(struct oplus_voocphy_manager *chip)
{
	u8 data_reg07, data_reg10;
	int ret_reg07, ret_reg10;

	if (!chip) {
		pr_err("Failed\n");
		return 0;
	}

	ret_reg07 = nu2112a_slave_read_byte(chip->slave_client, NU2112A_REG_07, &data_reg07);
	ret_reg10 = nu2112a_slave_read_byte(chip->slave_client, NU2112A_REG_10, &data_reg10);

	if (ret_reg07 < 0 || ret_reg10 < 0) {
		pr_err("NU2112A_REG_07 or NU2112A_REG_10 err\n");
		return 0;
	}
	data_reg07 = data_reg07 >> 7;
	data_reg10 = data_reg10 & NU2112A_CP_SWITCHING_STAT_MASK;

	data_reg10 = data_reg10 >> NU2112A_CP_SWITCHING_STAT_SHIFT;

	pr_err("11 reg07 = %d reg10 = %d\n", data_reg07, data_reg10);

	if (data_reg07 == 1 && data_reg10 == 1) {
		return 1;
	} else {
		return 0;
	}
}

static int nu2112a_slave_reg_reset(struct oplus_voocphy_manager *chip, bool enable)
{
	int ret;
	u8 val;
	if (enable)
		val = NU2112A_RESET_REG;
	else
		val = NU2112A_NO_REG_RESET;

	val <<= NU2112A_REG_RESET_SHIFT;

	ret = nu2112a_slave_update_bits(chip->slave_client, NU2112A_REG_07, NU2112A_REG_RESET_MASK, val);

	return ret;
}

static int nu2112a_slave_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	ret = nu2112a_slave_read_byte(chip->slave_client, NU2112A_REG_07, data);
	if (ret < 0) {
		pr_err("NU2112A_REG_1A\n");
		return -1;
	}
	*data = *data >> NU2112A_CHG_EN_SHIFT;

	return ret;
}

static int nu2112a_slave_set_chg_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	u8 value = 0x8A;
	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	if (enable)
		value = 0x8A; /*Enable CP,550KHz*/
	else
		value = 0x0A; /*Disable CP,550KHz*/

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_07, value);
	pr_err(" enable  = %d, value = 0x%x!\n", enable, value);
	return 0;
}

static int nu2112a_slave_get_voocphy_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	ret = nu2112a_slave_read_byte(chip->slave_client, NU2112A_REG_2B, data);
	if (ret < 0) {
		pr_err("NU2112A_REG_2B\n");
		return -1;
	}

	return ret;
}

static int nu2112a_slave_set_chg_pmid2out(struct oplus_voocphy_manager *chip, bool enable, int reason)
{
	chg_err("nu2112a_slave_set_chg_pmid2out\n");

	if (enable) {
		if (reason == SETTING_REASON_SVOOC)
			return nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_05,
							0x31); /*PMID/2-VOUT < 10%VOUT*/
		else if (reason == SETTING_REASON_VOOC)
			return nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_05,
							0x33);
		else
			chg_err("no type for slave_set_chg_pmid2out\n");
	} else {
		if (reason == SETTING_REASON_SVOOC)
			return nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_05,
							0xB1); /*PMID/2-VOUT < 10%VOUT*/
		else if (reason == SETTING_REASON_VOOC)
			return nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_05,
							0xA3);
		else
			chg_err("no type for slave_set_chg_pmid2out\n");
	}

	return 0;
}

static bool nu2112a_slave_get_chg_pmid2out(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 data = 0;

	ret = nu2112a_slave_read_byte(chip->slave_client, NU2112A_REG_05, &data);
	if (ret < 0) {
		chg_err("read NU2112A_SLAVE_REG_05 error\n");
		return false;
	}

	chg_info("NU2112A_SLAVE_REG_05 = 0x%0x\n", data);

	data = data >> NU2112A_PMID2OUT_OVP_DIS_SHIFT;
	if (data == NU2112A_PMID2OUT_OVP_ENABLE)
		return true;
	else
		return false;
}

static void nu2112a_slave_dump_reg_in_err_issue(struct oplus_voocphy_manager *chip)
{
	int i = 0, p = 0;
	if (!chip) {
		pr_err("!!!!! oplus_voocphy_manager chip NULL");
		return;
	}

	for (i = 0; i < 40; i++) {
		p = p + 1;
		nu2112a_slave_read_byte(chip->slave_client, i, &chip->slave_reg_dump[p]);
	}
	pr_err("p[%d], ", p);
	return;
}

static void nu2112a_slave_track_dump_reg(void)
{
	struct nu2112a_slave_device *chip = g_device_chip;

	if (chip == NULL) {
		chg_err("nu2112a_slave_device chip is NULL\n");
		return;
	}

	i2c_smbus_read_i2c_block_data(chip->slave_client,
		TRACK_REG_ADDR_START, TRACK_REG_DUMP_NUM, chip->track_reg_dump);
}

#define ERR_MSG_BUF	PAGE_SIZE
__printf(3, 4)
static int nu2112a_slave_publish_ic_err_msg(int type, int sub_type, const char *format, ...)
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
					"[%s]-[%d]-[%d]:%s", "nu2112a_slave", type, sub_type, buf);
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

static void nu2112a_slave_track_abnormal_upload_info_work(struct work_struct *work)
{
	struct nu2112a_slave_device *chip =
		container_of(work, struct nu2112a_slave_device, abnormal_upload_info_work);
	char *buf;
	int i;
	size_t index = 0;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return;

	if (chip->ic_status == NU2112A_SLAVE_IC_PIN_DIAG_FAIL)
		index += scnprintf(buf + index, ERR_MSG_BUF, "$$err_reason@@pin_diag_fail$$reg_info@@");
	else
		index += scnprintf(buf + index, ERR_MSG_BUF, "$$err_reason@@power_ng$$reg_info@@");

	for (i = 0; i < TRACK_REG_DUMP_NUM; i++)
		index += scnprintf(buf + index, ERR_MSG_BUF, "0x%04x=%02x,",
			(TRACK_REG_ADDR_START + i), chip->track_reg_dump[i]);
	if (index > 0)
		buf[index - 1] = 0;

	nu2112a_slave_publish_ic_err_msg(OPLUS_IC_ERR_BURN, 0, "%s", buf);
	kfree(buf);
}

static bool nu2112a_slave_ic_is_abnormal(struct oplus_voocphy_manager *chip)
{
	u8 data = 0;
	enum nu2112a_slave_ic_status ic_status;
	struct nu2112a_slave_device *device_chip = g_device_chip;

	if (!chip || !device_chip) {
		chg_err("oplus_voocphy_manager chip or device_chip is NULL\n");
		return false;
	}

	ic_status = device_chip->ic_status;
	nu2112a_slave_read_byte(chip->slave_client, NU2112A_REG_14, &data);

	if (data & NU2112A_PIN_DIAG_FALL_FLAG_MASK)
		device_chip->ic_status = NU2112A_SLAVE_IC_PIN_DIAG_FAIL;
	else if (data & NU2112A_POWER_NG_FLAG_MASK)
		device_chip->ic_status = NU2112A_SLAVE_IC_POWER_NG;
	else
		device_chip->ic_status = NU2112A_SLAVE_IC_OK;

	chg_info("reg[0x%x] = 0x%x, pre_ic_status:%d, ic_status:%d\n",
		NU2112A_REG_14, data, ic_status, device_chip->ic_status);
	if (device_chip->ic_status != NU2112A_SLAVE_IC_OK) {
		if (ic_status != device_chip->ic_status) {
			nu2112a_slave_track_dump_reg();
			if (NU2112A_REG_14 >= TRACK_REG_ADDR_START && NU2112A_REG_14 < TRACK_REG_ADDR_END)
				device_chip->track_reg_dump[NU2112A_REG_14 - TRACK_REG_ADDR_START] = data;
			schedule_work(&device_chip->abnormal_upload_info_work);
		}
		return true;
	}

	return false;
}

static int nu2112a_slave_init_device(struct oplus_voocphy_manager *chip)
{
	u8 reg_data;

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x10); /* ADC_CTRL:disable */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_02, 0x7); /* VAC OVP */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_03, 0x50); /* VBUS_OVP:10V */
	reg_data = slave_ovp_reg;
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_00, reg_data); /* VBAT_OVP:4.65V */
	reg_data = slave_ocp_reg & 0x3f;
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_04, reg_data); /* IBUS_OCP_UCP:3.6A */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_0D, 0x01); /* IBUS UCP Falling =150ms */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_09, 0x80); /* IBUS_UCP_RISE:1.28S */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_0C, 0x41); /* IBUS UCP 250ma Falling,500ma Rising */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_01, 0xa8); /* IBAT OCP Disable */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_2B, 0x00); /* VOOC_CTRL:disable */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_35, 0x20); /* VOOC Option2 */
	/* REG_08=0x00, WD Disable,Charge mode 2:1 */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_08, 0x0); /* VOOC Option2 */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_17, 0x28); /* IBUS_UCP_RISE_MASK_MASK */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_15, 0x02); /* mask insert irq */

	nu2112a_slave_update_bits(chip->slave_client, NU2112A_REG_0A, NU2112A_CFLY_PRECHG_TIMEOUT_MASK,
		NU2112A_CFLY_PRECHG_20_MS << NU2112A_CFLY_PRECHG_TIMEOUT_SHIFT);

	pr_err("nu2112a_slave_init_device done");

	return 0;
}

static int nu2112a_slave_init_vooc(struct oplus_voocphy_manager *chip)
{
	pr_err("nu2112a_slave_init_vooc\n");

	nu2112a_slave_reg_reset(chip, true);
	nu2112a_slave_init_device(chip);

	return 0;
}

static int nu2112a_slave_svooc_hw_setting(struct oplus_voocphy_manager *chip)
{
	u8 reg_data;

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_02, 0x04); /* VAC_OVP */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_03, 0x50); /* VBUS_OVP:10V */
	reg_data = slave_ocp_reg & 0x3f;
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_04, reg_data); /* IBUS_OCP_UCP:3.6A */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_17, 0x28); /* Mask IBUS UCP rising */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_08, 0x03); /* WD:1000ms */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x90); /* ADC_CTRL:ADC_EN */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_05, 0xB1); /* PMID/2-VOUT < 10%VOUT */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_33, 0xd1); /* Loose_det=1 */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_35, 0x20); /* VOOCPHY Option2 */

	return 0;
}

static int nu2112a_slave_vooc_hw_setting(struct oplus_voocphy_manager *chip)
{
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_02, 0x06); /* VAC_OVP */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_03, 0x50); /* VBUS_OVP:10V */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_04, 0x30); /* IBUS_OCP_UCP:4.8A */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_17, 0x28); /* Mask IBUS UCP rising */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_08, 0x83); /* WD:1000ms */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x90); /* ADC_CTRL:ADC_EN */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_05, 0xA3); /* PMID/2-VOUT < 10%VOUT */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_33, 0xd1); /* Loose_det=1 */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_35, 0x20); /* VOOCPHY Option2 */

	return 0;
}

static int nu2112a_slave_5v2a_hw_setting(struct oplus_voocphy_manager *chip)
{
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_02, 0x06); /* VAC_OVP */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_03, 0x00); /* VBUS_OVP */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_17, 0x28); /* Mask IBUS UCP rising */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_08, 0x00); /* WD */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x90); /* ADC_CTRL:ADC_EN */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_33, 0xd1); /* Loose_det=1 */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_35, 0x20); /* VOOCPHY Option2 */

	return 0;
}

static int nu2112a_slave_pdqc_hw_setting(struct oplus_voocphy_manager *chip)
{
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_02, 0x04); /* VAC_OVP */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_03, 0x50); /* VBUS_OVP */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_08, 0x00); /* WD */
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x10); /* ADC_CTRL:ADC_EN */

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_2B, 0x00); /* DISABLE VOOCPHY */

	pr_err("nu2112a_pdqc_hw_setting done");
	return 0;
}

static int nu2112a_slave_hw_setting(struct oplus_voocphy_manager *chip, int reason)
{
	if (!chip) {
		pr_err("chip is null exit\n");
		return -1;
	}
	switch (reason) {
	case SETTING_REASON_PROBE:
	case SETTING_REASON_RESET:
		nu2112a_slave_init_device(chip);
		pr_info("SETTING_REASON_RESET OR PROBE\n");
		break;
	case SETTING_REASON_SVOOC:
		nu2112a_slave_svooc_hw_setting(chip);
		nu2112a_slave_set_osc_status(g_device_chip, NU2112A_OSC_ENABLE);
		pr_info("SETTING_REASON_SVOOC\n");
		break;
	case SETTING_REASON_VOOC:
		nu2112a_slave_vooc_hw_setting(chip);
		pr_info("SETTING_REASON_VOOC\n");
		break;
	case SETTING_REASON_5V2A:
		nu2112a_slave_5v2a_hw_setting(chip);
		pr_info("SETTING_REASON_5V2A\n");
		break;
	case SETTING_REASON_PDQC:
		nu2112a_slave_pdqc_hw_setting(chip);
		pr_info("SETTING_REASON_PDQC\n");
		break;
	default:
		pr_err("do nothing\n");
		break;
	}
	return 0;
}

static int nu2112a_slave_reset_voocphy(struct oplus_voocphy_manager *chip)
{
	nu2112a_slave_set_chg_enable(chip, false);
	nu2112a_slave_set_osc_status(g_device_chip, NU2112A_OSC_DISABLE);
	nu2112a_slave_hw_setting(chip, SETTING_REASON_RESET);

	return VOOCPHY_SUCCESS;
}

static int nu2112a_slave_cp_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int nu2112a_slave_cp_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static int nu2112a_slave_cp_get_iin(struct oplus_chg_ic_dev *ic_dev, int *iin)
{
	struct nu2112a_slave_device *device;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	device = oplus_chg_ic_get_priv_data(ic_dev);

	rc = nu2112a_slave_get_ichg(device->voocphy);
	if (rc < 0) {
		chg_err("can't get cp iin, rc=%d\n", rc);
		return rc;
	}
	*iin = rc;
	return 0;
}

static bool nu2112a_slave_check_work_mode_support(enum oplus_cp_work_mode mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_cp_support_work_mode); i++) {
		if (g_cp_support_work_mode[i] == mode)
			return true;
	}
	return false;
}

static int nu2112a_slave_cp_check_work_mode_support(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	return nu2112a_slave_check_work_mode_support(mode);
}

static int nu2112a_slave_cp_set_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	struct nu2112a_slave_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (!nu2112a_slave_check_work_mode_support(mode)) {
		chg_err("not supported work mode, mode=%d\n", mode);
		return -EINVAL;
	}

	chip->mode = mode;
	chg_info("work mode=%d\n", mode);
	if (mode == CP_WORK_MODE_BYPASS)
		rc = nu2112a_slave_vooc_hw_setting(chip->voocphy);
	else
		rc = nu2112a_slave_svooc_hw_setting(chip->voocphy);

	if (rc < 0)
		chg_err("set work mode to %d error\n", mode);

	return rc;
}

static int nu2112a_slave_get_cp_vbus(struct oplus_voocphy_manager *chip)
{
	u8 data_block[2] = { 0 };
	s32 ret = 0;

	/* parse data_block for improving time of interrupt */
	ret = i2c_smbus_read_i2c_block_data(chip->slave_client, NU2112A_REG_1C, 2, data_block);
	if (ret < 0) {
		nu2112a_slave_i2c_error(true);
		pr_err("nu2112a_slave read vbat error \n");
		return ret;
	} else {
		nu2112a_slave_i2c_error(false);
	}

	return (((data_block[0] & NU2112A_VBUS_POL_H_MASK) << 8) | data_block[1]) * NU2112A_VBUS_ADC_LSB;
}

static int nu2112a_slave_cp_get_vin(struct oplus_chg_ic_dev *ic_dev, int *vin)
{
	struct nu2112a_slave_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = nu2112a_slave_get_cp_vbus(chip->voocphy);
	if (rc < 0) {
		chg_err("can't get cp vin, rc=%d\n", rc);
		return rc;
	}
	*vin = rc;

	return 0;
}

static int nu2112a_slave_cp_set_work_start(struct oplus_chg_ic_dev *ic_dev, bool start)
{
	struct nu2112a_slave_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s work %s\n", chip->slave_dev->of_node->name, start ? "start" : "stop");

	if (start && chip->mode != CP_WORK_MODE_BYPASS)
		nu2112a_slave_set_osc_status(chip, NU2112A_OSC_ENABLE);
	else
		nu2112a_slave_set_osc_status(chip, NU2112A_OSC_DISABLE);

	rc = nu2112a_slave_set_chg_enable(chip->voocphy, start);
	if (rc < 0)
		return rc;

	return 0;
}

static int nu2112a_slave_cp_get_work_status(struct oplus_chg_ic_dev *ic_dev, bool *start)
{
	struct nu2112a_slave_device *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = nu2112a_slave_read_byte(chip->slave_client, NU2112A_REG_07, &data);
	if (rc < 0) {
		chg_err("read NU2112A_REG_07 error, rc=%d\n", rc);
		return rc;
	}

	*start = data & BIT(7);

	return 0;
}

static int nu2112a_slave_set_adc_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!chip) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (enable)
		return nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x90);
	else
		return nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x10);
}

static int nu2112a_slave_cp_adc_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct nu2112a_slave_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	return nu2112a_slave_set_adc_enable(chip->voocphy, en);
}

static void *nu2112a_slave_cp_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, nu2112a_slave_cp_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, nu2112a_slave_cp_exit);
		break;
	case OPLUS_IC_FUNC_CP_GET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IIN, nu2112a_slave_cp_get_iin);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_MODE,
			nu2112a_slave_cp_set_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT,
			nu2112a_slave_cp_check_work_mode_support);
		break;
	case OPLUS_IC_FUNC_CP_GET_VIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VIN,
			nu2112a_slave_cp_get_vin);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START,
			nu2112a_slave_cp_set_work_start);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_STATUS,
			nu2112a_slave_cp_get_work_status);
		break;
	case OPLUS_IC_FUNC_CP_SET_ADC_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_ADC_ENABLE,
			nu2112a_slave_cp_adc_enable);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq nu2112a_slave_cp_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int nu2112a_slave_ic_register(struct nu2112a_slave_device *device)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *child;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg;
	int rc;

	for_each_child_of_node(device->slave_dev->of_node, child) {
		rc = of_property_read_u32(child, "oplus,ic_type", &ic_type);
		if (rc < 0)
			continue;
		rc = of_property_read_u32(child, "oplus,ic_index", &ic_index);
		if (rc < 0)
			continue;
		ic_cfg.name = child->name;
		ic_cfg.index = ic_index;
		ic_cfg.type = ic_type;
		ic_cfg.priv_data = device;
		ic_cfg.of_node = child;
		switch (ic_type) {
		case OPLUS_CHG_IC_CP:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-nu2112a:%d", ic_index);
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = nu2112a_slave_cp_get_func;
			ic_cfg.virq_data = nu2112a_slave_cp_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(nu2112a_slave_cp_virq_table);
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_type);
			continue;
		}

		ic_dev = devm_oplus_chg_ic_register(device->slave_dev, &ic_cfg);
		if (!ic_dev) {
			rc = -ENODEV;
			chg_err("register %s error\n", child->name);
			continue;
		}
		chg_info("register %s\n", child->name);

		switch (ic_dev->type) {
		case OPLUS_CHG_IC_CP:
			device->cp_work_mode = CP_WORK_MODE_UNKNOWN;
			device->cp_ic = ic_dev;
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_dev->type);
			continue;
		}
		of_platform_populate(child, NULL, NULL, device->slave_dev);
	}

	return 0;
}

static ssize_t nu2112a_slave_show_registers(struct device *dev, struct device_attribute *attr, char *buf)
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
		ret = nu2112a_slave_read_byte(chip->slave_client, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx, "Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}
	return idx;
}

static ssize_t nu2112a_slave_store_register(struct device *dev, struct device_attribute *attr, const char *buf,
					    size_t count)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x38)
		nu2112a_slave_write_byte(chip->slave_client, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, 0660, nu2112a_slave_show_registers, nu2112a_slave_store_register);

static void nu2112a_slave_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
}

static struct of_device_id nu2112a_slave_charger_match_table[] = {
	{
		.compatible = "oplus,nu2112a-slave",
	},
	{},
};

static struct oplus_voocphy_operations oplus_nu2112a_slave_ops = {
	.hw_setting = nu2112a_slave_hw_setting,
	.init_vooc = nu2112a_slave_init_vooc,
	.update_data = nu2112a_slave_update_data,
	.get_chg_enable = nu2112a_slave_get_chg_enable,
	.set_chg_enable = nu2112a_slave_set_chg_enable,
	.get_ichg = nu2112a_slave_get_ichg,
	.reset_voocphy = nu2112a_slave_reset_voocphy,
	.get_cp_status = nu2112a_slave_get_cp_status,
	.get_voocphy_enable = nu2112a_slave_get_voocphy_enable,
	.set_chg_pmid2out = nu2112a_slave_set_chg_pmid2out,
	.get_chg_pmid2out = nu2112a_slave_get_chg_pmid2out,
	.dump_voocphy_reg = nu2112a_slave_dump_reg_in_err_issue,
	.ic_is_abnormal = nu2112a_slave_ic_is_abnormal,
};

static int nu2112a_slave_parse_dt(struct oplus_voocphy_manager *chip)
{
	int rc;
	struct device_node *node = NULL;
	struct nu2112a_slave_device *device;

	if (!chip) {
		chg_err("chip null\n");
		return -1;
	}

	node = chip->slave_dev->of_node;
	device = chip->priv_data;
	if (device) {
		device->osc_ctrl_support = of_property_read_bool(node, "oplus,osc_ctrl_support");
		chg_info("osc_ctrl_support:%d\n", device->osc_ctrl_support);
	}

	rc = of_property_read_u32(node, "ovp_reg", &slave_ovp_reg);
	if (rc) {
		slave_ovp_reg = DEFAULT_OVP_REG_CONFIG;
	} else {
		chg_err("slave_ovp_reg is %d\n", slave_ovp_reg);
	}

	rc = of_property_read_u32(node, "ocp_reg", &slave_ocp_reg);
	if (rc) {
		slave_ocp_reg = DEFAULT_OCP_REG_CONFIG;
	} else {
		chg_err("slave_ocp_reg is %d\n", slave_ocp_reg);
	}

	return 0;
}

static int nu2112a_slave_osc_enable(struct nu2112a_slave_device *chip)
{
	int rc = 0;
	u8 data = 0;
	int retry_count = 1;

	do {
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x90);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x00);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x78);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x87);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0xAA);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x55);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_E7, 0x80);
		rc |= nu2112a_slave_read_byte(chip->slave_client, NU2112A_REG_E7, &data);
	} while (data != 0x80 && retry_count-- > 0);

	rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x00);
	chg_info("rc =%d, reg:0x%x\n", rc, data);

	if (data == 0x80)
		return 0;
	else
		return -1;
}

static int nu2112a_slave_osc_disable(struct nu2112a_slave_device *chip)
{
	int rc = 0;
	u8 data  = 0x80;
	int retry_count = 1;

	do {
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x90);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x00);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x78);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x87);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0xAA);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x55);
		rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_E7, 0x00);
		rc |= nu2112a_slave_read_byte(chip->slave_client, NU2112A_REG_E7, &data);
	} while (data != 0x00 && retry_count-- > 0);

	rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x00);
	rc |= nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x10);
	chg_info("rc =%d, reg:0x%x\n", rc, data);

	if (data == 0x00)
		return 0;
	else
		return -1;
}

static void nu2112a_slave_force_osc_disable(struct nu2112a_slave_device *chip)
{
	if (!chip || !chip->osc_ctrl_support || !chip->slave_client)
		return;

	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x90);
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x00);
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x78);
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x87);
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0xAA);
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x55);
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_E7, 0x00);
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_DE, 0x00);
	nu2112a_slave_write_byte(chip->slave_client, NU2112A_REG_18, 0x10);
}

static void nu2112a_slave_set_osc_status(
			struct nu2112a_slave_device *chip, enum nu2112a_osc_status osc_status)
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
		rc = nu2112a_slave_osc_disable(chip);
		if (!rc) {
			chip->osc_status = NU2112A_OSC_DISABLE;
			mutex_unlock(&chip->osc_status_lock);
			cancel_delayed_work_sync(&chip->osc_status_daemon_work);
		} else {
			chg_err("disable fail, start work rerun\n");
			chip->osc_status = NU2112A_OSC_INVALID;
			mutex_unlock(&chip->osc_status_lock);
			if (osc_status == NU2112A_OSC_INIT)
				schedule_delayed_work(&chip->osc_status_daemon_work, msecs_to_jiffies(1500));
		}
		break;
	case NU2112A_OSC_ENABLE:
		rc = nu2112a_slave_osc_enable(chip);
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

static void nu2112a_slave_osc_status_daemon_work(struct work_struct *work)
{
	int rc;
	int cp_vbus;
	struct delayed_work *dwork = to_delayed_work(work);
	struct nu2112a_slave_device *chip = container_of(dwork,
		struct nu2112a_slave_device, osc_status_daemon_work);

	mutex_lock(&chip->osc_status_lock);
	if (chip->osc_status == NU2112A_OSC_DISABLE) {
		chg_err("osc status has disable, not need handle\n");
		mutex_unlock(&chip->osc_status_lock);
		return;
	}

	cp_vbus = nu2112a_slave_get_cp_vbus(chip->voocphy);
	if (cp_vbus < 2000) { /* cp vbus less than 2000 mv detected */
		usleep_range(5000, 5000);
		cp_vbus = nu2112a_slave_get_cp_vbus(chip->voocphy);
	}
	chg_info("cp_vbus=%d\n", cp_vbus);

	/* osc_status is invalid or cp vbus less than 2000 mv detected */
	if (chip->osc_status == NU2112A_OSC_INVALID || cp_vbus < 2000) {
		rc = nu2112a_slave_osc_disable(chip);
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

static void nu2112a_slave_osc_init(struct nu2112a_slave_device *chip)
{
	mutex_init(&chip->osc_status_lock);
	INIT_DELAYED_WORK(&chip->osc_status_daemon_work, nu2112a_slave_osc_status_daemon_work);
	chip->osc_status = NU2112A_OSC_DISABLE;
	nu2112a_slave_set_osc_status(chip, NU2112A_OSC_INIT);
}

static int nu2112a_slave_charger_choose(struct oplus_voocphy_manager *chip)
{
	int ret;
	int max_count = 5;

	if (oplus_voocphy_chip_is_null()) {
		chg_err("oplus_voocphy_chip null, will do after master cp init!");
		return -EPROBE_DEFER;
	} else {
		while (max_count--) {
			ret = i2c_smbus_read_byte_data(chip->slave_client, 0x07);
			chg_info("0x07 = %d\n", ret);
			if (ret < 0) {
				chg_err("i2c communication fail");
				continue;
			} else {
				break;
			}
		}
	}

	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static int nu2112a_slave_charger_probe(struct i2c_client *client)
#else
static int nu2112a_slave_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct nu2112a_slave_device *device;
	struct oplus_voocphy_manager *chip;
	int ret;

	pr_err("nu2112a_slave_slave_charger_probe enter!\n");

	device = devm_kzalloc(&client->dev, sizeof(*device), GFP_KERNEL);
	if (device == NULL) {
		chg_err("alloc nu2112 device buf error\n");
		return -ENOMEM;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		ret = -ENOMEM;
		goto device_err;
	}

	device->slave_client = client;
	device->slave_dev = &client->dev;
	chip->slave_client = client;
	chip->slave_dev = &client->dev;
	chip->priv_data = device;
	device->voocphy = chip;
	mutex_init(&i2c_rw_lock);
	i2c_set_clientdata(client, chip);

	if (oplus_voocphy_chip_is_null()) {
		pr_err("oplus_voocphy_chip null, will do after master cp init.\n");
		ret = -EPROBE_DEFER;
		goto chip_err;
	}

	ret = nu2112a_slave_charger_choose(chip);
	if (ret <= 0) {
		chg_err("slave choose err\n");
		goto chip_err;
	}

	INIT_WORK(&device->abnormal_upload_info_work, nu2112a_slave_track_abnormal_upload_info_work);

	nu2112a_slave_create_device_node(&(client->dev));

	nu2112a_slave_parse_dt(chip);

	nu2112a_slave_osc_init(device);
	g_device_chip = device;

	nu2112a_slave_reg_reset(chip, true);

	nu2112a_slave_init_device(chip);

	chip->slave_ops = &oplus_nu2112a_slave_ops;

	oplus_voocphy_slave_init(chip);

	oplus_voocphy_get_chip(&oplus_voocphy_mg);
	ret = nu2112a_slave_ic_register(device);
	if (ret < 0) {
		chg_err("slave cp ic register error\n");
		ret = -ENOMEM;
		goto chip_err;
	}
	nu2112a_slave_cp_init(device->cp_ic);

	pr_err("probe successfully!\n");

	return 0;

chip_err:
	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, chip);
device_err:
	devm_kfree(&client->dev, device);
	return ret;
}

static void nu2112a_slave_charger_shutdown(struct i2c_client *client)
{
	nu2112a_slave_write_byte(client, NU2112A_REG_18, 0x10);
	nu2112a_slave_force_osc_disable(g_device_chip);

	return;
}

static const struct i2c_device_id nu2112a_slave_charger_id[] = {
	{ "oplus,nu2112a-slave", 0 },
	{},
};

static struct i2c_driver nu2112a_slave_charger_driver = {
	.driver =
		{
			.name = "nu2112a-charger-slave",
			.owner = THIS_MODULE,
			.of_match_table = nu2112a_slave_charger_match_table,
		},
	.id_table = nu2112a_slave_charger_id,

	.probe = nu2112a_slave_charger_probe,
	.shutdown = nu2112a_slave_charger_shutdown,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
static int __init nu2112a_slave_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");

	if (i2c_add_driver(&nu2112a_slave_charger_driver) != 0) {
		chg_err(" failed to register nu2112a i2c driver.\n");
	} else {
		chg_debug(" Success to register nu2112a i2c driver.\n");
	}

	return ret;
}

subsys_initcall(nu2112a_slave_subsys_init);
#else
int nu2112a_slave_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");

	if (i2c_add_driver(&nu2112a_slave_charger_driver) != 0) {
		chg_err(" failed to register nu2112a i2c driver.\n");
	} else {
		chg_debug(" Success to register nu2112a i2c driver.\n");
	}

	return ret;
}

void nu2112a_slave_subsys_exit(void)
{
	i2c_del_driver(&nu2112a_slave_charger_driver);
}
oplus_chg_module_register(nu2112a_slave_subsys);
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)*/

MODULE_DESCRIPTION("SC NU2112A SLAVE VOOCPHY&UFCS Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("JJ Kong");

