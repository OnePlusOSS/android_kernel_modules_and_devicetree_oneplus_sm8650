// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Opus. All rights reserved.
 */
#define pr_fmt(fmt) "OPLUS_CHG[HL7138]: %s[%d]: " fmt, __func__, __LINE__

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/proc_fs.h>

#include <trace/events/sched.h>
#include <linux/ktime.h>
#include <linux/pm_qos.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/sched/clock.h>
#include <linux/mutex.h>
#include <soc/oplus/system/oplus_project.h>
#include <linux/rtc.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>

#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include "../oplus_voocphy.h"
#include "oplus_hl7138.h"

#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_impedance_check.h>
#include <oplus_chg_monitor.h>

#include <oplus_mms_gauge.h>
#include <oplus_impedance_check.h>
#include <oplus_chg_monitor.h>
#include "../voocphy/oplus_voocphy.h"

static struct oplus_voocphy_manager *oplus_voocphy_mg = NULL;

static struct mutex i2c_rw_lock;

struct hl7138_device {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct oplus_voocphy_manager *voocphy;
	struct ufcs_dev *ufcs;

	struct oplus_chg_ic_dev *cp_ic;
	struct oplus_impedance_node *input_imp_node;
	struct oplus_impedance_node *output_imp_node;

	struct oplus_mms *err_topic;

	struct mutex i2c_rw_lock;
	struct mutex chip_lock;
	atomic_t suspended;
	atomic_t i2c_err_count;
	struct wakeup_source *chip_ws;

	int ovp_reg;
	int ocp_reg;

	bool ufcs_enable;

	enum oplus_cp_work_mode cp_work_mode;

	bool rested;
	bool error_reported;
	int high_curr_setting;

	bool use_ufcs_phy;
	bool use_vooc_phy;
	bool vac_support;

	unsigned int cp_vbus;
	unsigned int cp_vsys;
	unsigned int cp_ichg;
	unsigned int master_cp_ichg;
	unsigned int cp_vbat;

};

#define DEFUALT_VBUS_LOW 100
#define DEFUALT_VBUS_HIGH 200

#define I2C_ERR_NUM 10
#define MAIN_I2C_ERROR (1 << 0)

#define VIN_OVP_HL7138_FLAG_MASK	BIT(7)
#define VIN_UVLO_HL7138_FLAG_MASK	BIT(6)
#define TRACK_OV_HL7138_FLAG_MASK	BIT(5)
#define TRACK_UV_HL7138_FLAG_MASK	BIT(4)
#define VBAT_OVP_HL7138_FLAG_MASK	BIT(3)
#define VOUT_OVP_HL7138_FLAG_MASK	BIT(2)
#define PMID_QUAL_HL7138_FLAG_MASK	BIT(1)
#define VBUS_UV_HL7138_FLAG_MASK	BIT(0)

#define IIN_OCP_HL7138_FLAG_MASK	BIT(15)
#define IBAT_OCP_HL7138_FLAG_MASK	BIT(14)
#define IIN_UCP_HL7138_FLAG_MASK	BIT(13)
#define FET_SHROT_HL7138_FLAG_MASK	BIT(12)
#define CFLY_SHORT_HL7138_FLAG_MASK	BIT(11)
#define THSD_HL7138_FLAG_MASK		BIT(8)

static struct irqinfo int_flag_hl7138[IRQ_EVNET_NUM_HL7138] = {
	{VIN_OVP_HL7138_FLAG_MASK, "VIN_OVP", 1},
	{VIN_UVLO_HL7138_FLAG_MASK, "VIN_UVLO", 1},
	{TRACK_OV_HL7138_FLAG_MASK, "TRACK_OV", 1},
	{TRACK_UV_HL7138_FLAG_MASK, "TRACK_UV", 1},
	{VBAT_OVP_HL7138_FLAG_MASK, "VBAT_OVP", 1},
	{VOUT_OVP_HL7138_FLAG_MASK, "VOUT_OVP", 1},
	{PMID_QUAL_HL7138_FLAG_MASK, "PMID_QUAL", 1},
	{VBUS_UV_HL7138_FLAG_MASK,   "VBUS_UV", 1},
	{IIN_OCP_HL7138_FLAG_MASK, "IIN_OCP", 1},
	{IBAT_OCP_FLAG_MASK, "IBAT_OCP", 1},
	{IIN_UCP_HL7138_FLAG_MASK, "IIN_UCP", 1},
	{FET_SHROT_HL7138_FLAG_MASK, "FET_SHROT", 1},
	{CFLY_SHORT_HL7138_FLAG_MASK, "CFLY_SHORT", 1},
	{THSD_HL7138_FLAG_MASK, "THSD", 1},
};

static int __hl7138_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		chg_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __hl7138_write_byte(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		chg_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}

	return 0;
}

static int hl7138_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&i2c_rw_lock);
	ret = __hl7138_read_byte(client, reg, data);
	mutex_unlock(&i2c_rw_lock);

	return ret;
}

static int hl7138_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&i2c_rw_lock);
	ret = __hl7138_write_byte(client, reg, data);
	mutex_unlock(&i2c_rw_lock);

	return ret;
}

static int hl7138_update_bits(struct i2c_client *client, u8 reg,
                              u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&i2c_rw_lock);
	ret = __hl7138_read_byte(client, reg, &tmp);
	if (ret) {
		chg_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __hl7138_write_byte(client, reg, tmp);
	if (ret)
		chg_err("Failed: reg=%02X, ret=%d\n", reg, ret);
out:
	mutex_unlock(&i2c_rw_lock);
	return ret;
}

static s32 hl7138_read_word(struct i2c_client *client, u8 reg)
{
	s32 ret;

	mutex_lock(&i2c_rw_lock);
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		chg_err("i2c read word fail: can't read reg:0x%02X \n", reg);
		mutex_unlock(&i2c_rw_lock);
		return ret;
	}
	mutex_unlock(&i2c_rw_lock);
	return ret;
}

static s32 hl7138_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;

	mutex_lock(&i2c_rw_lock);
	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		chg_err("i2c write word fail: can't write 0x%02X to reg:0x%02X \n", val, reg);
		mutex_unlock(&i2c_rw_lock);
		return ret;
	}
	mutex_unlock(&i2c_rw_lock);
	return 0;
}

static int hl7138_set_predata(struct oplus_voocphy_manager *chip, u16 val)
{
	int ret;
	if (!chip) {
		chg_err("failed: chip is null\n");
		return -1;
	}

	/* predata, pre_wdata,JL: REG_31 change to REG_3D */
	ret = hl7138_write_word(chip->client, HL7138_REG_3D, val);
	if (ret < 0) {
		chg_err("failed: write predata\n");
		return -1;
	}
	chg_debug("write predata 0x%0x\n", val);
	return ret;
}

static int hl7138_set_txbuff(struct oplus_voocphy_manager *chip, u16 val)
{
	int ret;
	if (!chip) {
		chg_err("failed: chip is null\n");
		return -1;
	}

	/* txbuff, tx_wdata, JL: REG_2C change to REG_38 */
	ret = hl7138_write_word(chip->client, HL7138_REG_38, val);
	if (ret < 0) {
		chg_err("failed: write txbuff\n");
		return -1;
	}

	return ret;
}

static int hl7138_get_adapter_info(struct oplus_voocphy_manager *chip)
{
	int data;
	if (!chip) {
		chg_err("gchip is null\n");
		return -1;
	}

	data = hl7138_read_word(chip->client, HL7138_REG_3A);		/* JL: 2E=RX_Rdata,change to 0x3A */

	if (data < 0) {
		chg_err("hl7138_read_word faile\n");
		return -1;
	}

	VOOCPHY_DATA16_SPLIT(data, chip->voocphy_rx_buff, chip->vooc_flag);
	chg_debug("data, vooc_flag, vooc_rxdata: 0x%0x 0x%0x 0x%0x\n", data, chip->vooc_flag, chip->voocphy_rx_buff);

	return 0;
}

static int hl7138_clear_interrupts(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 val = 0;

	if (!chip) {
		chg_err("chip is null\n");
		return -1;
	}

	ret = hl7138_read_byte(chip->client, HL7138_REG_01, &val);
	if (ret) {
		chg_err("clear int fail %d", ret);
		return ret;
	}
	ret = hl7138_read_byte(chip->client, HL7138_REG_3B, &val);
	if (ret) {
		chg_err("clear int fail %d", ret);
		return ret;
	}
	return 0;
}

#define HL7138_SVOOC_IBUS_FACTOR	110/100
#define HL7138_VOOC_IBUS_FACTOR		215/100
#define HL7138_FACTOR_125_100		125/100
#define HL7138_FACTOR_400_100		400/100
#define HL7138_REG_NUM_2 2
#define HL7138_REG_NUM_12 12
#define HL7138_REG_ADC_BIT_OFFSET_4 4
#define HL7138_REG_ADC_H_BIT_VBUS 0
#define HL7138_REG_ADC_L_BIT_VBUS 1
#define HL7138_REG_ADC_H_BIT_ICHG 2
#define HL7138_REG_ADC_L_BIT_ICHG 3
#define HL7138_REG_ADC_H_BIT_VBAT 4
#define HL7138_REG_ADC_L_BIT_VBAT 5
#define HL7138_REG_ADC_H_BIT_VSYS 10
#define HL7138_REG_ADC_L_BIT_VSYS 11
static void hl7138_update_data(struct oplus_voocphy_manager *chip)
{
	u8 data_block[HL7138_REG_NUM_12] = {0};
	int i = 0;
	int data;
	int ret = 0;

	data = hl7138_read_word(chip->client, HL7138_REG_05);
	if (data < 0) {
		chg_err("hl7138_read_word faile\n");
		return;
	}
	chip->interrupt_flag_hl7138 = data;

	/* parse data_block for improving time of interrupt
	 * JL:VIN,IIN,VBAT,IBAT,VTS,VOUT,VDIE,;
	 */
	ret = i2c_smbus_read_i2c_block_data(chip->client, HL7138_REG_42,
						HL7138_REG_NUM_12, data_block);		/* JL:first Reg is 13,need to change to 42; */

	for (i=0; i < HL7138_REG_NUM_12; i++) {
		chg_debug("data_block[%d] = %u\n", i, data_block[i]);
	}

	if (chip->adapter_type == ADAPTER_SVOOC) {
		chip->cp_ichg = ((data_block[HL7138_REG_ADC_H_BIT_ICHG] << HL7138_REG_ADC_BIT_OFFSET_4)
				| data_block[HL7138_REG_ADC_L_BIT_ICHG]) * HL7138_SVOOC_IBUS_FACTOR;	/* Iin_lbs=1.10mA@CP; */
	} else {
		chip->cp_ichg = ((data_block[HL7138_REG_ADC_H_BIT_ICHG] << HL7138_REG_ADC_BIT_OFFSET_4)
				| data_block[HL7138_REG_ADC_L_BIT_ICHG])*HL7138_VOOC_IBUS_FACTOR;	/* Iin_lbs=2.15mA@BP; */
	}
	chip->cp_vbus = ((data_block[HL7138_REG_ADC_H_BIT_VBUS] << HL7138_REG_ADC_BIT_OFFSET_4)
			| data_block[HL7138_REG_ADC_L_BIT_VBUS]) * HL7138_FACTOR_400_100;	/* vbus_lsb=4mV; */
	chip->cp_vsys = ((data_block[HL7138_REG_ADC_H_BIT_VSYS] << HL7138_REG_ADC_BIT_OFFSET_4)
			| data_block[HL7138_REG_ADC_L_BIT_VSYS]) * HL7138_FACTOR_125_100;	/* vout_lsb=1.25mV; */
	chip->cp_vbat = ((data_block[HL7138_REG_ADC_H_BIT_VBAT] << HL7138_REG_ADC_BIT_OFFSET_4)
			| data_block[HL7138_REG_ADC_L_BIT_VBAT]) * HL7138_FACTOR_125_100;	/* vout_lsb=1.25mV; */
	chip->cp_vac = chip->cp_vbus;
	chg_debug("cp_ichg = %d cp_vbus = %d, cp_vsys = %d, cp_vbat = %d, int_flag = %d",
		chip->cp_ichg, chip->cp_vbus, chip->cp_vsys, chip->cp_vbat, chip->interrupt_flag_hl7138);
}

/*********************************************************************/
static int hl7138_reg_reset(struct oplus_voocphy_manager *chip, bool enable)
{
	unsigned char value;
	int ret = 0;
	ret = hl7138_read_byte(chip->client, HL7138_REG_01, &value);	/* clear INTb */

	hl7138_write_byte(chip->client, HL7138_REG_09, 0x00);	/* set default mode; */
	hl7138_write_byte(chip->client, HL7138_REG_0A, 0xAE);	/* set default mode; */
	hl7138_write_byte(chip->client, HL7138_REG_0C, 0x03);	/* set default mode; */
	hl7138_write_byte(chip->client, HL7138_REG_0F, 0x00);	/* set default mode; */
	hl7138_write_byte(chip->client, HL7138_REG_13, 0x40);	/* set 100ms ucp debounce time; */
	hl7138_write_byte(chip->client, HL7138_REG_15, 0x00);	/* set default mode; */
	hl7138_write_byte(chip->client, HL7138_REG_17, 0x00);	/* set default mode; */
	hl7138_read_byte(chip->client, HL7138_REG_05, &value);
	hl7138_read_byte(chip->client, HL7138_REG_06, &value);
	hl7138_write_byte(chip->client, HL7138_REG_37, 0x02);	/* reset VOOC PHY; */
	hl7138_read_byte(chip->client, HL7138_REG_3B, &value);  /* clear flag 2023-jl */
	hl7138_write_byte(chip->client, HL7138_REG_3F, 0xD1);	/* bit7 T6:170us */

	return ret;
}

static u8 hl7138_get_int_value(struct oplus_voocphy_manager *chip)
{
	int int_data;
	u8 data = 0;

	if (!chip) {
		chg_err("%s: chip null\n", __func__);
		return -1;
	}

	int_data = hl7138_read_word(chip->client, HL7138_REG_05);
	if (int_data < 0) {
		chg_err(" read HL7138_REG_05 failed\n");
		return -1;
	}
	chip->interrupt_flag_hl7138 = int_data;
	data = int_data & 0xff;

	return data;
}

static int hl7138_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;
	if (!chip) {
		chg_err("Failed\n");
		return -1;
	}
	ret = hl7138_read_byte(chip->client, HL7138_REG_12, data);	/* JL:RST_REG 12 change to 14; */
	if (ret < 0) {
		chg_err("HL7138_REG_12\n");
		return -1;
	}
	*data = *data >> HL7138_CHG_EN_SHIFT;

	return ret;
}

#define HL7138_CHG_EN (HL7138_CHG_ENABLE << HL7138_CHG_EN_SHIFT)                    /* 1 << 7   0x80 */
#define HL7138_CHG_DIS (HL7138_CHG_DISABLE << HL7138_CHG_EN_SHIFT)                  /* 0 << 7   0x00 */
#define HL7138_IBUS_UCP_EN (HL7138_IBUS_UCP_ENABLE << HL7138_IBUS_UCP_DIS_SHIFT)    /* 1 << 2   0x04 */
#define HL7138_IBUS_UCP_DIS (HL7138_IBUS_UCP_DISABLE << HL7138_IBUS_UCP_DIS_SHIFT)  /* 0 << 2   0x00 */
#define HL7138_IBUS_UCP_DEFAULT  0x01

static int hl7138_set_chg_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!chip) {
		chg_err("Failed\n");
		return -1;
	}

	if (enable) {
		hl7138_write_byte(chip->client, HL7138_REG_13,
				  HL7138_IBUS_UCP_DEB_100ms << HL7138_IBUS_UCP_DEB_SHIFT); /* UCP debounce time 100ms */
		return hl7138_update_bits(chip->client, HL7138_REG_12, HL7138_CHG_EN_MASK, HL7138_CHG_EN);  /* is not pdsvooc adapter: enable ucp */
	} else {
		return hl7138_write_byte(chip->client, HL7138_REG_12, HL7138_CHG_DIS | HL7138_IBUS_UCP_EN | HL7138_IBUS_UCP_DEFAULT);      /* chg disable */
	}
}

static int hl7138_voocphy_set_sstimeout_ucp_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	int ret;
	u8 reg;

	if (!chip) {
		chg_err("chip is null\n");
		return -EINVAL;
	}
	if (!chip->fcl_support)
		return -EINVAL;

	ret = hl7138_read_byte(chip->client, HL7138_REG_12, &reg);
	if ((enable && (reg & HL7138_IBUS_UCP_DIS_MASK)) || (!enable && !(reg & HL7138_IBUS_UCP_DIS_MASK)))
		return -ENODEV;
	if (enable) {
		ret = hl7138_update_bits(chip->client, HL7138_REG_12,
			HL7138_IBUS_UCP_DIS_MASK, HL7138_IBUS_UCP_ENABLE << HL7138_IBUS_UCP_DIS_SHIFT);
	} else {
		ret = hl7138_update_bits(chip->client, HL7138_REG_12,
			HL7138_IBUS_UCP_DIS_MASK, HL7138_IBUS_UCP_DISABLE << HL7138_IBUS_UCP_DIS_SHIFT);
	}

	return ret;
}

static int hl7138_cp_set_sstimeout_ucp_enable(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	struct hl7138_device *chip;
	int ret;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s %s\n", chip->dev->of_node->name, enable ? "enable" : "disable");

	ret = hl7138_voocphy_set_sstimeout_ucp_enable(chip->voocphy, enable);

	return 0;
}

static int hl7138_get_cp_ichg(struct oplus_voocphy_manager *chip)
{
	u8 data_block[HL7138_REG_NUM_2] = {0};
	u8 cp_enable = 0;

	hl7138_get_chg_enable(chip, &cp_enable);

	if(cp_enable == 0)
		return 0;
	/*parse data_block for improving time of interrupt*/
	i2c_smbus_read_i2c_block_data(chip->client, HL7138_REG_44, HL7138_REG_NUM_2, data_block);

	if (chip->adapter_type == ADAPTER_SVOOC) {
		chip->cp_ichg = ((data_block[0] << HL7138_REG_ADC_BIT_OFFSET_4) | data_block[1]) * HL7138_SVOOC_IBUS_FACTOR;	/* Iin_lbs=1.10mA@CP; */
	} else {
		chip->cp_ichg = ((data_block[0] << HL7138_REG_ADC_BIT_OFFSET_4) | data_block[1]) * HL7138_VOOC_IBUS_FACTOR;	/* Iin_lbs=2.15mA@BP; */
	}
	chg_info("chip->cp_ichg = %d\n", chip->cp_ichg);

	return chip->cp_ichg;
}

int hl7138_get_cp_vbat(struct oplus_voocphy_manager *chip)
{
	u8 data_block[HL7138_REG_NUM_2] = {0};

	/*parse data_block for improving time of interrupt*/
	i2c_smbus_read_i2c_block_data(chip->client, HL7138_REG_46, HL7138_REG_NUM_2, data_block);

	chip->cp_vbat = ((data_block[0] << HL7138_REG_ADC_BIT_OFFSET_4) | data_block[1]) * HL7138_FACTOR_125_100;

	return chip->cp_vbat;
}

static void hl7138_set_pd_svooc_config(struct oplus_voocphy_manager *chip, bool enable)
{
	int ret = 0;
	u8 reg = 0;
	if (!chip) {
		chg_err("Failed\n");
		return;
	}

	if (enable)
		hl7138_write_byte(chip->client, HL7138_REG_13,
				  HL7138_IBUS_UCP_DEB_100ms << HL7138_IBUS_UCP_DEB_SHIFT); /* UCP debounce time 100ms */
	else
		hl7138_write_byte(chip->client, HL7138_REG_13,
				  HL7138_IBUS_UCP_DEB_10ms << HL7138_IBUS_UCP_DEB_SHIFT);

	ret = hl7138_read_byte(chip->client, HL7138_REG_13, &reg);
	if (ret < 0) {
		chg_err("HL7138_REG_13\n");
		return;
	}
	chg_debug("pd_svooc config HL7138_REG_13 = %d\n", reg);
}

static bool hl7138_get_pd_svooc_config(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 data = 0;

	if (!chip) {
		chg_err("Failed\n");
		return false;
	}

	ret = hl7138_read_byte(chip->client, HL7138_REG_13, &data);
	if (ret < 0) {
		chg_err("HL7138_REG_13\n");
		return false;
	}

	chg_debug("HL7138_REG_13 = 0x%0x\n", data);

	return (data & HL7138_IBUS_UCP_DEB_MASK);
}

static int hl7138_get_adc_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		chg_err("Failed\n");
		return -1;
	}

	ret = hl7138_read_byte(chip->client, HL7138_REG_40, data);
	if (ret < 0) {
		chg_err("HL7138_REG_40\n");
		return -1;
	}

	*data = (*data & HL7138_ADC_ENABLE);

	return ret;
}

static int hl7138_set_adc_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!chip) {
		chg_err("Failed\n");
		return -1;
	}

	if (enable)
		return hl7138_update_bits(chip->client, HL7138_REG_40,
									HL7138_ADC_EN_MASK, HL7138_ADC_ENABLE);
	else
		return hl7138_update_bits(chip->client, HL7138_REG_40,
									HL7138_ADC_EN_MASK, HL7138_ADC_DISABLE);
}

/*
 * @function	 hl7138_set_adc_forcedly_enable
 * @detailed
 * This function will configure the ADC operating mode
 * @param[in] chip -- oplus_voocphy_manager
 * @param[in] mode -- ADC operation mode
 * HL7138_ADC_AUTO_MODE(00)           : ADC Auto mode. Enabled in Standby state and Active state, disabled in Shutdown state
 * HL7138_ADC_FORCEDLY_ENABLED(01)    : ADC forcedly enabled
 * HL7138_ADC_FORCEDLY_DISABLED_10(10): ADC forcedly disabled
 * HL7138_ADC_FORCEDLY_DISABLED_11(11): ADC forcedly disabled
 * @return 0:success
 */
static int hl7138_set_adc_forcedly_enable(struct oplus_voocphy_manager *chip, int mode)
{
	int ret = 0;
        if (!chip) {
                chg_err("Failed\n");
                return -1;
        }

	switch(mode) {
	case ADC_AUTO_MODE:
		ret = hl7138_update_bits(chip->client, HL7138_REG_40,
					HL7138_ADC_FORCEDLY_EN_MASK, HL7138_ADC_AUTO_MODE);
		break;
	case ADC_FORCEDLY_ENABLED:
		ret = hl7138_update_bits(chip->client, HL7138_REG_40,
					HL7138_ADC_FORCEDLY_EN_MASK, HL7138_ADC_FORCEDLY_ENABLED);
		break;
	case ADC_FORCEDLY_DISABLED_10:
		ret = hl7138_update_bits(chip->client, HL7138_REG_40,
					HL7138_ADC_FORCEDLY_EN_MASK, HL7138_ADC_FORCEDLY_DISABLED_10);
		break;
	case ADC_FORCEDLY_DISABLED_11:
		ret = hl7138_update_bits(chip->client, HL7138_REG_40,
					HL7138_ADC_FORCEDLY_EN_MASK, HL7138_ADC_FORCEDLY_DISABLED_11);
		break;
	default:
		chg_err("[HL7138] Without this mode, no processing is performed!!!\n");
	}

	return ret;
}

void hl7138_send_handshake_seq(struct oplus_voocphy_manager *chip)
{
	unsigned char value;

	hl7138_read_byte(chip->client, HL7138_REG_01, &value);	/* before handshake, clear int 2023-jl */
	hl7138_read_byte(chip->client, HL7138_REG_3B, &value);	/* before handshake, clear int 2023-jl */
	hl7138_write_byte(chip->client, HL7138_REG_37, 0x81);	/* JL:2B->37,EN & Handshake; */

	chg_debug("hl7138_send_handshake_seq done");
}

int hl7138_reset_voocphy(struct oplus_voocphy_manager *chip)
{
	u8 data;
	u8 reg_data;

	/*aviod exit fastchg vbus ovp drop out*/
	hl7138_write_byte(chip->client, HL7138_REG_14, 0x08);

	/* hwic config with plugout */
	reg_data = chip->reg_ctrl_1;
	hl7138_write_byte(chip->client, HL7138_REG_11, reg_data);	/* JL:Dis VBAT,IBAT reg; */
	reg_data = chip->ovp_reg;
	hl7138_write_byte(chip->client, HL7138_REG_08, reg_data);	/* JL:vbat_ovp=4.65V;00->08;(4.65-0.09)/10=54; */
	hl7138_write_byte(chip->client, HL7138_REG_0B, 0x88);	/* JL:VBUS_OVP=12V;4+val*lsb; */
	/* hl7138_write_byte(chip->client, HL7138_REG_0C, 0x0F);		//JL:vbus_ovp=10V;04->0c;10.5/5.25V; */
	reg_data = chip->ocp_reg;
	hl7138_write_byte(chip->client, HL7138_REG_0E, reg_data);	/* JL:UCP_deb=5ms;IBUS_OCP=3.6A;05->0e;3.5A_max; */
	hl7138_write_byte(chip->client, HL7138_REG_40, 0x00);	/* JL:Dis_ADC;11->40; */
	hl7138_write_byte(chip->client, HL7138_REG_02, 0xE0);	/* JL:mask all INT_FLAG */
	hl7138_write_byte(chip->client, HL7138_REG_10, 0xEC);	/* JL:Dis IIN_REG; */

	/* turn off mos */
	hl7138_write_byte(chip->client, HL7138_REG_12, 0x05);	/* JL:Fsw=500KHz;07->12; enable ucp*/
	/* set 100ms ucp debounce time; */
	hl7138_write_byte(chip->client, HL7138_REG_13, 0x40);

	/* clear tx data */
	hl7138_write_byte(chip->client, HL7138_REG_38, 0x00);	/* JL:2C->38; */
	hl7138_write_byte(chip->client, HL7138_REG_39, 0x00);	/* JL:2D->39; */

	/* disable vooc phy irq */
	hl7138_write_byte(chip->client, HL7138_REG_3C, 0xff);	/* JL:30->3C,VOOC_PHY FLAG ALL MASK; */

	/* set D+ HiZ */
	/* hl7138_write_byte(chip->client, HL7138_REG_21, 0xc0);	//JL:No need in HL7138; */

	/* disable vooc */
	hl7138_read_byte(chip->client, HL7138_REG_3B, &data);	/* before disable vooc, 7138 need clear flag */
	hl7138_write_byte(chip->client, HL7138_REG_37, 0x00);	/* JL:2B->37,Dis all; */
	hl7138_read_byte(chip->client, HL7138_REG_3B, &data);	/* after disable vooc, 7138 need clear flag */
	hl7138_set_predata(chip, 0);

	chg_debug("oplus_vooc_reset_voocphy done");

	return VOOCPHY_SUCCESS;
}

int hl7138_reactive_voocphy(struct oplus_voocphy_manager *chip)
{
	/* to avoid cmd of adjust current(0x01)return error, add voocphy bit0 hold time to 800us */
	hl7138_set_predata(chip, 0);

	/* clear tx data */
	hl7138_write_byte(chip->client, HL7138_REG_38, 0x00);	/* JL:2C->38,Dis all; */
	hl7138_write_byte(chip->client, HL7138_REG_39, 0x00);	/* JL:2D->39,Dis all; */

	/* vooc */
	hl7138_write_byte(chip->client, HL7138_REG_3C, 0x85);	/* JL:30->3C,JUST enable RX_START & TX_DONE; */
	hl7138_send_handshake_seq(chip);

	chg_debug ("oplus_vooc_reactive_voocphy done");

	return VOOCPHY_SUCCESS;
}

static int hl7138_init_device(struct oplus_voocphy_manager *chip)
{
	u8 reg_data;

	hl7138_write_byte(chip->client, HL7138_REG_40, 0x00);	/* ADC_CTRL:disable,JL:11-40; */
	hl7138_write_byte(chip->client, HL7138_REG_0B, 0x88);	/* VBUS_OVP=12V,JL:02->0B; */
	/* hl7138_write_byte(chip->client, HL7138_REG_0C, 0x0F);		//VBUS_OVP:10.2 2:1 or 1:1V,JL:04-0C; */
	reg_data = chip->reg_ctrl_1;
	hl7138_write_byte(chip->client, HL7138_REG_11, reg_data);	/* ovp:90mV */
	reg_data = chip->ovp_reg;
	hl7138_write_byte(chip->client, HL7138_REG_08, reg_data);	/* VBAT_OVP:4.56	4.56+0.09*/
	reg_data = chip->ocp_reg;
	hl7138_write_byte(chip->client, HL7138_REG_0E, reg_data);	/* IBUS_OCP:3.5A      ocp:100mA */
	/* hl7138_write_byte(chip->client, HL7138_REG_0A, 0x2E);		//IBAT_OCP:max;JL:01-0A;0X2E=6.6A,MAX; */
	hl7138_write_byte(chip->client, HL7138_REG_37, 0x00);	/* VOOC_CTRL:disable;JL:2B->37; */

	hl7138_write_byte(chip->client, HL7138_REG_3C, 0x85);	/* diff mask inter; */
	hl7138_write_byte(chip->client, HL7138_REG_02, 0xE0);	/* JL:mask all INT_FLAG */
	hl7138_write_byte(chip->client, HL7138_REG_10, 0xEC);	/* JL:Dis IIN_REG; */
	hl7138_write_byte(chip->client, HL7138_REG_12, 0x05);	/* JL:Fsw=500KHz;07->12; */
	hl7138_write_byte(chip->client, HL7138_REG_14, 0x08);	/* JL:dis WDG; */
	hl7138_write_byte(chip->client, HL7138_REG_16, 0x2C);	/* JL:OV=500, UV=250 */

	return 0;
}

static int hl7138_work_mode_lockcheck(struct oplus_voocphy_manager *chip)
{
	unsigned char reg;

	if (!chip) {
		return -1;
	}

	if (!hl7138_read_byte(chip->client, HL7138_REG_A7, &reg) && reg != 0x4) {
		/*test mode unlock & lock avoid burnning out the chip*/
		hl7138_write_byte(chip->client, HL7138_REG_A0, 0xF9);
		hl7138_write_byte(chip->client, HL7138_REG_A0, 0x9F);	/* Unlock test register */
		hl7138_write_byte(chip->client, HL7138_REG_A7, 0x04);
		hl7138_write_byte(chip->client, HL7138_REG_A0, 0x00);	/* Lock test register */
		chg_err("hl7138_work_mode_lockcheck done\n");
	}
	return 0;
}

static bool hl7138_check_hw_ba_version(struct oplus_voocphy_manager *chip)
{
	int ret;
	u8 val;

	ret = hl7138_read_byte(chip->client, HL7138_REG_00, &val);
	if (ret < 0) {
		chg_err("read hl7138 reg0 error\n");
		return false;
	}

	if (val == HL7138_BA_VERSION)
		return true;
	else
		return false;
}

/* turn off system clk for BA version only */
static int hl7138_turnoff_sys_clk(struct oplus_voocphy_manager *chip)
{
	u8 reg_data[2] = {0};
	int retry = 0;

	if (!chip) {
		chg_err("turn off sys clk failed\n");
		return -1;
	}

	do {
		hl7138_write_byte(chip->client, HL7138_REG_A0, 0xF9);
		hl7138_write_byte(chip->client, HL7138_REG_A0, 0x9F);	/* Unlock register */
		hl7138_write_byte(chip->client, HL7138_REG_A3, 0x01);
		hl7138_write_byte(chip->client, HL7138_REG_A0, 0x00);	/* Lock register */

		hl7138_read_byte(chip->client, HL7138_REG_A3, &reg_data[0]);
		hl7138_read_byte(chip->client, HL7138_REG_A0, &reg_data[1]);
		chg_debug("0xA3 = 0x%02x, 0xA0 = 0x%02x\n", reg_data[0], reg_data[1]);

		if ((reg_data[0] == 0x01) && (reg_data[1] == 0x00))	/* Lock register success */
			break;
		mdelay(5);
		retry++;
	} while (retry <= 3);
	chg_debug("hl7138_turnoff_sys_clk done\n");

	return 0;
}

/* turn on system clk for BA version only */
static int hl7138_turnon_sys_clk(struct oplus_voocphy_manager *chip)
{
	u8 reg_data[2] = {0};
	int retry = 0;
	int ret = 0;

	if (!chip) {
		chg_err("turn on sys clk failed\n");
		return -1;
	}

	do {
		hl7138_write_byte(chip->client, HL7138_REG_A0, 0xF9);
		hl7138_write_byte(chip->client, HL7138_REG_A0, 0x9F);	/* Unlock register */
		hl7138_write_byte(chip->client, HL7138_REG_A3, 0x00);
		hl7138_write_byte(chip->client, HL7138_REG_A0, 0x00);	/* Lock register */

		ret = hl7138_read_byte(chip->client, HL7138_REG_A3, &reg_data[0]);
		ret |= hl7138_read_byte(chip->client, HL7138_REG_A0, &reg_data[1]);
		chg_debug("0xA3 = 0x%02x, 0xA0 = 0x%02x\n", reg_data[0], reg_data[1]);

		/* Lock register success */
		if ((reg_data[0] == 0x00) && (reg_data[1] == 0x00) && ret == 0)
			break;
		mdelay(5);
		retry++;
	} while (retry <= 3);

	/* combined operation, let sys_clk return auto mode, current restore to uA level */
	hl7138_write_byte(chip->client, HL7138_REG_02, 0xF0);
	hl7138_write_byte(chip->client, HL7138_REG_40, 0x0D);	/* force enable adc read average with 4 samples data */
	hl7138_write_byte(chip->client, HL7138_REG_14, 0xC8);	/* soft reset register and disable watchdog */
	mdelay(2);
	chg_debug("hl7138_turnon_sys_clk done\n");

	return 0;
}

int hl7138_init_vooc(struct oplus_voocphy_manager *chip)
{
	chg_err(">>>> start init vooc\n");

	hl7138_reg_reset(chip, true);
	hl7138_work_mode_lockcheck(chip);
	hl7138_init_device(chip);

	/* to avoid cmd of adjust current(0x01)return error, add voocphy bit0 hold time to 800us */
	hl7138_set_predata(chip, 0);

	return 0;
}

static int hl7138_svooc_hw_setting(struct oplus_voocphy_manager *chip)
{
	/* hl7138_write_byte(chip->client, HL7138_REG_08, 0x38);	//VBAT_OVP:4.65V,JL:00-08; */
	hl7138_write_byte(chip->client, HL7138_REG_40, 0x05);	/* ADC_CTRL:ADC_EN,JL:11-40; */

	hl7138_write_byte(chip->client, HL7138_REG_0B, 0x88);	/* VBUS_OVP:12V */
	hl7138_write_byte(chip->client, HL7138_REG_0C, 0x00);	/* VIN_OVP:10.2V,,JL:04-0C; */

	if (chip->high_curr_setting)
		hl7138_write_byte(chip->client, HL7138_REG_0E, 0xB2);	/* disable OCP */
	else
		hl7138_write_byte(chip->client, HL7138_REG_0E, 0x32);	/* IBUS_OCP:3.6A,UCP_DEB=5ms;JL:05-0E; */

	hl7138_write_byte(chip->client, HL7138_REG_14, 0x02);	/* WD:1000ms,JL:09-14; */
	hl7138_write_byte(chip->client, HL7138_REG_15, 0x00);	/* enter cp mode */
	hl7138_write_byte(chip->client, HL7138_REG_16, 0x2C);	/* JL:OV=500, UV=250 */

	hl7138_write_byte(chip->client, HL7138_REG_3F, 0x91);	/* Loose_det=1,JL:33-3F; */
	hl7138_voocphy_set_sstimeout_ucp_enable(chip, false);
	return 0;
}

static int hl7138_vooc_hw_setting(struct oplus_voocphy_manager *chip)
{
	/* hl7138_write_byte(chip->client, HL7138_REG_08, 0x38);	//VBAT_OVP:4.65V,JL:00-08; */
	hl7138_write_byte(chip->client, HL7138_REG_40, 0x05);	/* ADC_CTRL:ADC_EN,JL:11-40; */

	hl7138_write_byte(chip->client, HL7138_REG_0B, 0x88);	/* VBUS_OVP=12V */
	hl7138_write_byte(chip->client, HL7138_REG_0C, 0x0F);	/* VIN_OVP:5.85v,,JL:04-0C; */

	hl7138_write_byte(chip->client, HL7138_REG_0E, 0x2F);   /* IBUS_OCP:3.55A, 47*50+1000+200=3550mA */

	hl7138_write_byte(chip->client, HL7138_REG_14, 0x02);	/* WD:1000ms,JL:09-14; */
	hl7138_write_byte(chip->client, HL7138_REG_15, 0x80);	/* JL:bp mode; */
	hl7138_write_byte(chip->client, HL7138_REG_16, 0x2C);	/* JL:OV=500, UV=250 */
	hl7138_write_byte(chip->client, HL7138_REG_3F, 0x91);	/* Loose_det=1,JL:33-3F; */
	hl7138_voocphy_set_sstimeout_ucp_enable(chip, false);
	return 0;
}

static int hl7138_5v2a_hw_setting(struct oplus_voocphy_manager *chip)
{
	/* hl7138_write_byte(chip->client, HL7138_REG_08, 0x38);	//VBAT_OVP:4.65V,JL:00-08; */
	hl7138_write_byte(chip->client, HL7138_REG_0B, 0x88);	/* VBUS_OVP=12V */
	hl7138_write_byte(chip->client, HL7138_REG_0C, 0x0F);	/* VIN_OVP:11.7v,,JL:04-0C; */
	hl7138_write_byte(chip->client, HL7138_REG_0E, 0xAF);	/* IBUS_OCP:3.6A,UCP_DEB=5ms;JL:05-0E; */
	hl7138_write_byte(chip->client, HL7138_REG_14, 0x08);	/* WD:DIS,JL:09-14; */
	hl7138_write_byte(chip->client, HL7138_REG_15, 0x80);	/* JL:bp mode; */

	hl7138_write_byte(chip->client, HL7138_REG_40, 0x00);	/* ADC_CTRL:disable,JL:11-40; */
	hl7138_write_byte(chip->client, HL7138_REG_37, 0x00);	/* VOOC_CTRL:disable;JL:2B->37; */
	/* hl7138_write_byte(chip->client, HL7138_REG_3D, 0x01);	//pre_wdata,JL:31-3D; */
	/* hl7138_write_byte(chip->client, HL7138_REG_3E, 0x80);	//pre_wdata,JL:32-3E; */
	/* hl7138_write_byte(chip->client, HL7138_REG_3F, 0xd1);	//Loose_det=1,JL:33-3F; */
	return 0;
}

static int hl7138_pdqc_hw_setting(struct oplus_voocphy_manager *chip)
{
	hl7138_write_byte(chip->client, HL7138_REG_08, 0x3C);	/* VBAT_OVP:4.6V */
	hl7138_write_byte(chip->client, HL7138_REG_0B, 0x88);	/* VBUS_OVP:12V */
	hl7138_write_byte(chip->client, HL7138_REG_0C, 0x0F);	/* VIN_OVP:11.7V */
	hl7138_write_byte(chip->client, HL7138_REG_0E, 0xB2);	/* IBUS_OCP disable */
	hl7138_write_byte(chip->client, HL7138_REG_14, 0x08);	/* WD:DIS */
	hl7138_write_byte(chip->client, HL7138_REG_15, 0x00);	/* enter cp mode */
	hl7138_write_byte(chip->client, HL7138_REG_40, 0x00);	/* ADC_CTRL:disable */
	hl7138_write_byte(chip->client, HL7138_REG_37, 0x00);	/* VOOC_CTRL:disable */
	return 0;
}

int hl7138_enable_t5t6_check(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!enable) {
		hl7138_write_byte(chip->client, HL7138_REG_3F, 0x91);
	} else {
		hl7138_write_byte(chip->client, HL7138_REG_3F, 0xD1);
	}
	return 0;
}

#define MDELAY_10 10
static int hl7138_hw_reset(struct oplus_voocphy_manager *chip)
{
	hl7138_write_byte(chip->client, HL7138_REG_14, 0xc8);
	mdelay(MDELAY_10);

	return 0;
}

static void hl7138_hardware_init(struct oplus_voocphy_manager *chip)
{
	hl7138_reg_reset(chip, true);
	hl7138_work_mode_lockcheck(chip);
	hl7138_init_device(chip);
}

static bool hl7138_check_cp_int_happened(struct oplus_voocphy_manager *chip,
					 bool *dump_reg, bool *send_info)
{
	int i = 0;

	for (i = 0; i < IRQ_EVNET_NUM_HL7138; i++) {
		if ((int_flag_hl7138[i].mask & chip->interrupt_flag_hl7138) && int_flag_hl7138[i].mark_except) {
			chg_err("cp int happened %s\n", int_flag_hl7138[i].except_info);
		}
	}

	return false;
}


static int hl7138_hw_setting(struct oplus_voocphy_manager *chip, int reason)
{
	if (!chip) {
		chg_err("chip is null exit\n");
		return -1;
	}
	switch (reason) {
		case SETTING_REASON_PROBE:
		case SETTING_REASON_RESET:
			hl7138_init_device(chip);
			/*reset for avoiding PBS01 & PBV01 chg break*/
			hl7138_enable_t5t6_check(chip, true);
			hl7138_hw_reset(chip);
			chg_debug("SETTING_REASON_RESET OR PROBE\n");
			break;
		case SETTING_REASON_SVOOC:
			hl7138_svooc_hw_setting(chip);
			chg_debug("SETTING_REASON_SVOOC\n");
			break;
		case SETTING_REASON_VOOC:
			hl7138_vooc_hw_setting(chip);
			chg_debug("SETTING_REASON_VOOC\n");
			break;
		case SETTING_REASON_5V2A:
			hl7138_5v2a_hw_setting(chip);
			chg_debug("SETTING_REASON_5V2A\n");
			break;
		case SETTING_REASON_PDQC:
			hl7138_pdqc_hw_setting(chip);
			chg_debug("SETTING_REASON_PDQC\n");
			break;
		default:
			chg_err("do nothing\n");
			break;
	}
	return 0;
}

static ssize_t hl7138_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "chip");
	for (addr = 0x0; addr <= 0x4F; addr++) {
		if((addr < 0x18) || (addr > 0x35 && addr < 0x4F)) {
			ret = hl7138_read_byte(chip->client, addr, &val);
			if (ret == 0) {
				len = snprintf(tmpbuf, PAGE_SIZE - idx,
						"Reg[%.2X] = 0x%.2x\n", addr, val);
				memcpy(&buf[idx], tmpbuf, len);
				idx += len;
			}
		}
	}

	return idx;
}

static ssize_t hl7138_store_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x4F)
		hl7138_write_byte(chip->client, (unsigned char)reg, (unsigned char)val);

	return count;
}
static DEVICE_ATTR(registers, 0660, hl7138_show_registers, hl7138_store_register);

static void hl7138_create_device_node(struct device *dev)
{
	int err;

	err = device_create_file(dev, &dev_attr_registers);
	if (err)
		chg_err("hl7138 create device err!\n");
}

static struct of_device_id hl7138_charger_match_table[] = {
	{
		.compatible = "chip,hl7138-standalone",
	},
	{},
};

static int hl7138_get_chip_id(struct oplus_voocphy_manager *chip)
{
	return CHIP_ID_HL7138;
}

struct oplus_voocphy_operations oplus_hl7138_ops = {
	.hardware_init		= hl7138_hardware_init,
	.hw_setting		= hl7138_hw_setting,
	.init_vooc		= hl7138_init_vooc,
	.set_predata		= hl7138_set_predata,
	.set_txbuff		= hl7138_set_txbuff,
	.get_adapter_info	= hl7138_get_adapter_info,
	.update_data		= hl7138_update_data,
	.get_chg_enable		= hl7138_get_chg_enable,
	.set_chg_enable		= hl7138_set_chg_enable,
	.reset_voocphy		= hl7138_reset_voocphy,
	.reactive_voocphy 	= hl7138_reactive_voocphy,
	.send_handshake		= hl7138_send_handshake_seq,
	.get_cp_vbat		= hl7138_get_cp_vbat,
	.get_int_value		= hl7138_get_int_value,
	.get_adc_enable		= hl7138_get_adc_enable,
	.set_adc_enable		= hl7138_set_adc_enable,
	.set_adc_forcedly_enable= hl7138_set_adc_forcedly_enable,
	.get_ichg		= hl7138_get_cp_ichg,
	.set_pd_svooc_config	= hl7138_set_pd_svooc_config,
	.get_pd_svooc_config	= hl7138_get_pd_svooc_config,
	.clear_interrupts	= hl7138_clear_interrupts,
	.get_chip_id		= hl7138_get_chip_id,
	.check_cp_int_happened 	= hl7138_check_cp_int_happened,
	.set_sstimeout_ucp_enable	= hl7138_voocphy_set_sstimeout_ucp_enable,
};

static int hl7138_charger_choose(struct oplus_voocphy_manager *chip)
{
	int ret;

	if (!oplus_voocphy_chip_is_null()) {
		chg_err("oplus_voocphy_chip already exists!");
		return 0;
	} else {
		ret = i2c_smbus_read_byte_data(chip->client, 0x00);
		chg_err("0x00 = %d\n", ret);
		if (ret < 0) {
			chg_err("i2c communication fail");
			return -EPROBE_DEFER;
		}
		else
			return 1;
	}
}

static irqreturn_t hl7138_interrupt_handler(int irq, void *dev_id)
{
	struct oplus_voocphy_manager *voocphy = dev_id;

	return oplus_voocphy_interrupt_handler(voocphy);
}

static int hl7138_irq_gpio_init(struct oplus_voocphy_manager *chip)
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

static int hl7138_irq_register(struct oplus_voocphy_manager *voocphy)
{
	struct irq_desc *desc;
	struct cpumask current_mask;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	cpumask_var_t cpu_highcap_mask;
#endif
	int ret;

	ret = hl7138_irq_gpio_init(voocphy);
	if (ret < 0) {
		chg_err("failed to irq gpio init(%d)\n", ret);
		return ret;
	}

	if (voocphy->irq) {
		ret = request_threaded_irq(voocphy->irq, NULL,
					   hl7138_interrupt_handler,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   "voocphy_irq", voocphy);
		if (ret < 0) {
			chg_err("request irq for irq=%d failed, ret =%d\n",
				voocphy->irq, ret);
			return ret;
		}
		enable_irq_wake(voocphy->irq);
		chg_debug("request irq ok\n");
	}

	desc = irq_to_desc(voocphy->irq);
	if (desc == NULL) {
		free_irq(voocphy->irq, voocphy);
		chg_err("desc null\n");
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

static enum oplus_cp_work_mode g_cp_support_work_mode[] = {
	CP_WORK_MODE_BYPASS,
	CP_WORK_MODE_2_TO_1,
};

static bool hl7138_check_work_mode_support(enum oplus_cp_work_mode mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_cp_support_work_mode); i++) {
		if (g_cp_support_work_mode[i] == mode)
			return true;
	}
	return false;
}

static int hl7138_cp_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int hl7138_cp_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static int hl7138_cp_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct hl7138_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

//	hl7138_voocphy_dump_reg_in_err_issue(chip->voocphy);
	return 0;
}

static int hl7138_cp_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}


static int hl7138_cp_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct hl7138_device *chip;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (en)
		ret = hl7138_write_byte(chip->client, HL7138_REG_14, 0x02); /* WD:1000ms */
	else
		ret = hl7138_write_byte(chip->client, HL7138_REG_14, 0x08); /* dsiable wdt */

	if (ret < 0) {
		chg_err("failed to set hl7138_cp_enable (%d)(%d)\n", en, ret);
		return ret;
	}
	return 0;
}

static int hl7138_cp_hw_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct hl7138_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (chip->rested)
		return 0;

	hl7138_hardware_init(chip->voocphy);
	return 0;
}

static int hl7138_cp_set_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	struct hl7138_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (!hl7138_check_work_mode_support(mode)) {
		chg_err("not supported work mode, mode=%d\n", mode);
		return -EINVAL;
	}

	if (mode == CP_WORK_MODE_BYPASS)
		rc = hl7138_vooc_hw_setting(chip->voocphy);
	else
		rc = hl7138_svooc_hw_setting(chip->voocphy);

	if (rc < 0)
		chg_err("set work mode to %d error\n", mode);

	return rc;
}

static int hl7138_cp_get_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode *mode)
{
	struct oplus_voocphy_manager *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = hl7138_read_byte(chip->client, HL7138_REG_14, &data);
	if (rc < 0) {
		chg_err("read hl7138_REG_14 error, rc=%d\n", rc);
		return rc;
	}

	if (data & BIT(7))
		*mode = CP_WORK_MODE_BYPASS;
	else
		*mode = CP_WORK_MODE_2_TO_1;

	return 0;
}

static int hl7138_cp_check_work_mode_support(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	return hl7138_check_work_mode_support(mode);
}

static int hl7138_cp_set_iin(struct oplus_chg_ic_dev *ic_dev, int iin)
{
	return 0;
}

static int hl7138_get_cp_vbus(struct hl7138_device *chip)
{
	if (chip == NULL || chip->voocphy == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	hl7138_update_data(chip->voocphy);

	return chip->voocphy->cp_vbus;
}

static int hl7138_cp_get_vin(struct oplus_chg_ic_dev *ic_dev, int *vin)
{
	struct hl7138_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = hl7138_get_cp_vbus(chip);
	if (rc < 0) {
		chg_err("can't get cp vin, rc=%d\n", rc);
		return rc;
	}
	*vin = rc;

	return 0;
}

static int hl7138_cp_get_iin(struct oplus_chg_ic_dev *ic_dev, int *iin)
{
	struct hl7138_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	rc = hl7138_get_cp_ichg(chip->voocphy);
	if (rc < 0) {
		chg_err("can't get cp iin, rc=%d\n", rc);
		return rc;
	}
	*iin = rc;

	return 0;
}

static int hl7138_cp_get_vout(struct oplus_chg_ic_dev *ic_dev, int *vout)
{
	struct hl7138_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = hl7138_get_cp_vbat(chip->voocphy);
	if (rc < 0) {
		chg_err("can't get cp vout, rc=%d\n", rc);
		return rc;
	}
	*vout = rc;

	return 0;
}

static int hl7138_cp_get_iout(struct oplus_chg_ic_dev *ic_dev, int *iout)
{
	struct hl7138_device *chip;
	int iin;
	bool working;
	enum oplus_cp_work_mode work_mode;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	/*
	 * There is an exception in the iout adc of sc8537a, which is obtained
	 * indirectly through iin
	 */
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &working);
	if (rc < 0)
		return rc;
	if (!working) {
		*iout = 0;
		return 0;
	}
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_IIN, &iin);
	if (rc < 0)
		return rc;
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_MODE, &work_mode);
	if (rc < 0)
		return rc;
	switch (work_mode) {
	case CP_WORK_MODE_BYPASS:
		*iout = iin;
		break;
	case CP_WORK_MODE_2_TO_1:
		*iout = iin * 2;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int hl7138_cp_get_vac(struct oplus_chg_ic_dev *ic_dev, int *vac)
{
	struct hl7138_device *chip;
	u8 data_block[2] = { 0 };
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (!chip->vac_support)
		return -ENOTSUPP;

	rc = i2c_smbus_read_i2c_block_data(chip->client, HL7138_REG_42, 2, data_block);
	if (rc < 0) {
		//hl7138_i2c_error(chip->voocphy, true, true);
		chg_err("hl7138 read vac error, rc=%d\n", rc);
		return rc;
	} else {
		//hl7138_i2c_error(chip->voocphy, false, true);
	}

//	*vac = (((data_block[0] & hl7138_VAC_POL_H_MASK) << 8) | data_block[1]) * hl7138_VAC_ADC_LSB;

	return 0;
}

static int hl7138_cp_set_work_start(struct oplus_chg_ic_dev *ic_dev, bool start)
{
	struct hl7138_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s work %s\n", chip->dev->of_node->name, start ? "start" : "stop");

	rc = hl7138_set_chg_enable(chip->voocphy, start);
	if (rc < 0)
		return rc;
	oplus_imp_node_set_active(chip->input_imp_node, start);
	oplus_imp_node_set_active(chip->output_imp_node, start);

	return 0;
}

static int hl7138_cp_get_work_status(struct oplus_chg_ic_dev *ic_dev, bool *start)
{
	struct hl7138_device *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = hl7138_read_byte(chip->client, HL7138_REG_12, &data);
	if (rc < 0) {
		chg_err("read hl7138_REG_07 error, rc=%d\n", rc);
		return rc;
	}

	*start = data & BIT(7);

	return 0;
}

static int hl7138_cp_adc_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct hl7138_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	return hl7138_set_adc_enable(chip->voocphy, en);

	return 0;
}

static void *hl7138_cp_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, hl7138_cp_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, hl7138_cp_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, hl7138_cp_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, hl7138_cp_smt_test);
		break;
	case OPLUS_IC_FUNC_CP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_ENABLE, hl7138_cp_enable);
		break;
	case OPLUS_IC_FUNC_CP_HW_INTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_HW_INTI, hl7138_cp_hw_init);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_MODE, hl7138_cp_set_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_MODE, hl7138_cp_get_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT,
			hl7138_cp_check_work_mode_support);
		break;
	case OPLUS_IC_FUNC_CP_SET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_IIN, hl7138_cp_set_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VIN, hl7138_cp_get_vin);
		break;
	case OPLUS_IC_FUNC_CP_GET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IIN, hl7138_cp_get_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VOUT, hl7138_cp_get_vout);
		break;
	case OPLUS_IC_FUNC_CP_GET_IOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IOUT, hl7138_cp_get_iout);
		break;
	case OPLUS_IC_FUNC_CP_GET_VAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VAC, hl7138_cp_get_vac);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START, hl7138_cp_set_work_start);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_STATUS, hl7138_cp_get_work_status);
		break;
	case OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE, hl7138_cp_set_sstimeout_ucp_enable);
		break;
	case OPLUS_IC_FUNC_CP_SET_ADC_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, hl7138_cp_adc_enable);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq hl7138_cp_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int hl7138_get_input_node_impedance(void *data)
{
	struct hl7138_device *chip;
	int vac, vin, iin;
	int r_mohm;
	int rc;

	if (data == NULL)
		return -EINVAL;
	chip = data;

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_VIN, &vin);
	if (rc < 0) {
		chg_err("can't read cp vin, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_IIN, &iin);
	if (rc < 0) {
		chg_err("can't read cp iin, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_VAC, &vac);
	if (rc < 0 && rc != -ENOTSUPP) {
		chg_err("can't read cp vac, rc=%d\n", rc);
		return rc;
	} else if (rc == -ENOTSUPP) {
		/* If the current IC does not support it, try to get it from the parent IC */
		rc = oplus_chg_ic_func(chip->cp_ic->parent, OPLUS_IC_FUNC_CP_GET_VAC, &vac);
		if (rc < 0) {
			chg_err("can't read parent cp vac, rc=%d\n", rc);
			return rc;
		}
	}

	r_mohm = (vac - vin) * 1000 / iin;
	if (r_mohm < 0) {
		chg_err("input_node: r_mohm=%d\n", r_mohm);
		r_mohm = 0;
	}

	return r_mohm;
}

static int hl7138_get_output_node_impedance(void *data)
{
	struct hl7138_device *chip;
	struct oplus_mms *gauge_topic;
	union mms_msg_data mms_data = { 0 };
	int vout, iout, vbat;
	int r_mohm;
	int rc;

	if (data == NULL)
		return -EINVAL;
	chip = data;

	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_VOUT, &vout);
	if (rc < 0) {
		chg_err("can't read cp vout, rc=%d\n", rc);
		return rc;
	}
	rc = oplus_chg_ic_func(chip->cp_ic, OPLUS_IC_FUNC_CP_GET_IOUT, &iout);
	if (rc < 0) {
		chg_err("can't read cp iout, rc=%d\n", rc);
		return rc;
	}

	gauge_topic = oplus_mms_get_by_name("gauge");
	if (gauge_topic == NULL) {
		chg_err("gauge topic not found\n");
		return -ENODEV;
	}
	rc = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_VOL_MAX, &mms_data, false);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		return rc;
	}
	vbat = mms_data.intval;

	r_mohm = (vout - vbat * oplus_gauge_get_batt_num()) * 1000 / iout;
	if (r_mohm < 0) {
		chg_err("output_node: r_mohm=%d\n", r_mohm);
		r_mohm = 0;
	}

	return r_mohm;
}

static int hl7138_init_imp_node(struct hl7138_device *chip, struct device_node *of_node)
{
	struct device_node *imp_node;
	struct device_node *child;
	const char *name;
	int rc;

	imp_node = of_get_child_by_name(of_node, "oplus,impedance_node");
	if (imp_node == NULL)
		return 0;

	for_each_child_of_node(imp_node, child) {
		rc = of_property_read_string(child, "node_name", &name);
		if (rc < 0) {
			chg_err("can't read %s node_name, rc=%d\n", child->name, rc);
			continue;
		}
		if (!strcmp(name, "cp_input")) {
			chip->input_imp_node =
				oplus_imp_node_register(child, chip->dev, chip, hl7138_get_input_node_impedance);
			if (IS_ERR_OR_NULL(chip->input_imp_node)) {
				chg_err("%s register error, rc=%ld\n", child->name, PTR_ERR(chip->input_imp_node));
				chip->input_imp_node = NULL;
				continue;
			}
		} else if (!strcmp(name, "cp_output")) {
			chip->output_imp_node =
				oplus_imp_node_register(child, chip->dev, chip, hl7138_get_output_node_impedance);
			if (IS_ERR_OR_NULL(chip->output_imp_node)) {
				chg_err("%s register error, rc=%ld\n", child->name, PTR_ERR(chip->output_imp_node));
				chip->output_imp_node = NULL;
				continue;
			}
		} else {
			chg_err("unknown node_name: %s\n", name);
		}
	}

	return 0;
}

static int hl7138_ic_register(struct hl7138_device *chip)
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
			(void)hl7138_init_imp_node(chip, child);
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-hl7138:%d", ic_index);
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = hl7138_cp_get_func;
			ic_cfg.virq_data = hl7138_cp_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(hl7138_cp_virq_table);
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
			chip->cp_work_mode = CP_WORK_MODE_UNKNOWN;
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

static int hl7138_parse_dt(struct oplus_voocphy_manager *chip)
{
	int rc;
	struct device_node * node = NULL;

	if (!chip) {
		chg_debug("chip null\n");
		return -1;
	}

	/* Parsing gpio switch gpio47*/
	node = chip->dev->of_node;

	rc = of_property_read_u32(node, "oplus_spec,voocphy_vbus_low",
	                          &chip->voocphy_vbus_low);
	if (rc) {
		chip->voocphy_vbus_low = DEFUALT_VBUS_LOW;
	}
	chg_err("voocphy_vbus_high is %d\n", chip->voocphy_vbus_low);

	rc = of_property_read_u32(node, "oplus_spec,voocphy_vbus_high",
	                          &chip->voocphy_vbus_high);
	if (rc) {
		chip->voocphy_vbus_high = DEFUALT_VBUS_HIGH;
	}
	chg_err("voocphy_vbus_high is %d\n", chip->voocphy_vbus_high);

	chip->high_curr_setting = of_property_read_bool(node, "oplus_spec,high_curr_setting");

	rc = of_property_read_u32(node, "ovp_reg", &chip->ovp_reg);
	if (rc)
		chip->ovp_reg = 0x3C;
	chg_err("ovp_reg=0x%2x\n", chip->ovp_reg);

	rc = of_property_read_u32(node, "reg_ctrl_1", &chip->reg_ctrl_1);
	if (rc)
		chip->reg_ctrl_1 = 0xFC;
	chg_err("reg_ctrl_1=0x%2x\n", chip->reg_ctrl_1);

	rc = of_property_read_u32(node, "ocp_reg", &chip->ocp_reg);
	if (rc)
		chip->ocp_reg = 0x32;
	chg_err("ocp_reg=0x%2x\n", chip->ocp_reg);

	return 0;
}

static int hl7138_charger_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct hl7138_device *chip;
	struct oplus_voocphy_manager *voocphy;
	int ret;

	chg_err("hl7138_charger_probe enter!\n");
	chip = devm_kzalloc(&client->dev, sizeof(struct hl7138_device), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	voocphy = devm_kzalloc(&client->dev, sizeof(struct oplus_voocphy_manager), GFP_KERNEL);
	if (voocphy == NULL) {
		chg_err("alloc voocphy buf error\n");
		ret = -ENOMEM;
		goto hl7138_probe_err;
	}

	chip->dev = &client->dev;
	chip->client = client;
	mutex_init(&i2c_rw_lock);
	voocphy->client = client;
	voocphy->dev = &client->dev;
	voocphy->priv_data = chip;
	chip->voocphy = voocphy;
	mutex_init(&chip->i2c_rw_lock);
	mutex_init(&chip->chip_lock);

	i2c_set_clientdata(client, voocphy);

	ret = hl7138_charger_choose(voocphy);
	if (ret <= 0) {
		chg_err("failed to charger choose, ret = %d", ret);
		goto init_err;
	}

	hl7138_create_device_node(&(client->dev));
	chip->voocphy->ops = &oplus_hl7138_ops;
	ret = oplus_register_voocphy(chip->voocphy);
	if (ret < 0) {
		chg_err("failed to register voocphy, ret = %d", ret);
		goto hl7138_probe_err;
	}
	ret = hl7138_irq_register(voocphy);
	if (ret < 0) {
		chg_err("irq register error, rc=%d\n", ret);
		goto hl7138_probe_err;
	}
	oplus_voocphy_mg = voocphy;
	ret = hl7138_parse_dt(voocphy);
	if (ret < 0)
		goto parse_dt_err;

	/* turn on system clk for BA version only */
	if (hl7138_check_hw_ba_version(voocphy))
		hl7138_turnon_sys_clk(voocphy);

	ret = hl7138_ic_register(chip);
	if (ret < 0) {
		chg_err("cp ic register error\n");
		goto cp_reg_err;
	}

	hl7138_cp_init(chip->cp_ic);
	hl7138_hardware_init(chip->voocphy);

	chg_err("hl7138_charger_probe succesfull\n");
	return 0;

cp_reg_err:
	if (chip->input_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->input_imp_node);
	if (chip->output_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->output_imp_node);

init_err:
parse_dt_err:
	devm_kfree(&client->dev, voocphy);
	devm_kfree(&client->dev, chip);

hl7138_probe_err:
	chg_err("hl7138_charger_probe failed\n");
	return ret;
}

static void hl7138_charger_shutdown(struct i2c_client *client)
{
	if (oplus_voocphy_mg) {
		hl7138_write_byte(oplus_voocphy_mg->client, HL7138_REG_40, 0x00);	/* disable */
		hl7138_reg_reset(oplus_voocphy_mg, true);
		hl7138_hw_reset(oplus_voocphy_mg);

		/* turn off system clk for BA version only */
		if (hl7138_check_hw_ba_version(oplus_voocphy_mg))
			hl7138_turnoff_sys_clk(oplus_voocphy_mg);
	}
	chg_err("hl7138_charger_shutdown end\n");

	return;
}

static const struct i2c_device_id hl7138_charger_id[] = {
	{"hl7138-standalone", 0},
	{},
};

static struct i2c_driver hl7138_charger_driver = {
	.driver		= {
		.name	= "hl7138-charger",
		.owner	= THIS_MODULE,
		.of_match_table = hl7138_charger_match_table,
	},
	.id_table	= hl7138_charger_id,

	.probe		= hl7138_charger_probe,
	.shutdown	= hl7138_charger_shutdown,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
static int __init hl7138_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");

	if (i2c_add_driver(&hl7138_charger_driver) != 0) {
		chg_err(" failed to register hl7138 i2c driver.\n");
	} else {
		chg_debug(" Success to register hl7138 i2c driver.\n");
	}

	return ret;
}

subsys_initcall(hl7138_subsys_init);
#else
int hl7138_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");

	if (i2c_add_driver(&hl7138_charger_driver) != 0) {
		chg_err(" failed to register hl7138 i2c driver.\n");
	} else {
		chg_debug(" Success to register hl7138 i2c driver.\n");
	}

	return ret;
}

void hl7138_subsys_exit(void)
{
	i2c_del_driver(&hl7138_charger_driver);
}
oplus_chg_module_register(hl7138_subsys);
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)*/

MODULE_DESCRIPTION("Hl7138 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("OPLUS");
