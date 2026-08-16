// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "[sc6607]:[%s][%d]: " fmt, __func__, __LINE__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/pm_wakeup.h>
#include <linux/regmap.h>
#include <linux/sched/clock.h>

#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_monitor.h>
#include <oplus_impedance_check.h>
#include <oplus_chg.h>
#include "../oplus_voocphy.h"
#include "oplus_sc6607_cp.h"

#define CP_IC_VOTEABLE_NAME_MAX		128
#define SC6607_CP_STATUS_REG_MAX		2
#define ERR_MSG_BUF		PAGE_SIZE

struct sc6607 {
	s32 chip_id;
	bool error_reported;
	bool vac_support;
	bool shutdown_pending;
	bool work_start;
	bool is_sc6607a;
	enum oplus_cp_work_mode cp_work_mode;
	enum oplus_dpdm_switch_mode dpdm_switch_mode;
	int charge_type;
	int interrupt_flag;
	int ovp_reg;
	int ocp_reg;
	int vac_vbus_ovp_reg;
	struct device *dev;
	struct i2c_client *client;
	struct oplus_voocphy_manager *voocphy;
	struct votable *chg_disable_votable;
	struct oplus_chg_ic_dev *cp_ic;
	struct oplus_chg_ic_dev *ic_dev;
	struct oplus_impedance_node *input_imp_node;
	struct oplus_impedance_node *output_imp_node;
	struct oplus_mms *err_topic;
	struct mms_subscribe *err_subs;
	struct regmap *regmap;
	struct regmap_field *regmap_fields[F_MAX_FIELDS];
	atomic_t driver_suspended;
	atomic_t i2c_err_count;
	struct votable *disable_votable;
	struct work_struct cp_regdump_work;
	struct work_struct ic_offline_work;
	struct mutex i2c_rw_lock;
	struct mutex adc_read_lock;
};

static struct irqinfo sc6607_int_flag[SC6607_IRQ_EVNET_NUM] = {
	{SC6607_VOOCPHY_VBATSNS_OVP_FLAG_MASK, "VBATSNS_OVP", 1},
	{SC6607_VOOCPHY_VBAT_OVP_FLAG_MASK, "VBAT_OVP", 1},
	{SC6607_VOOCPHY_IBUS_OCP_FLAG_MASK, "IBUS_OCP", 1},
	{SC6607_VOOCPHY_IBUS_UCP_FALL_FLAG_MASK , "IBUS_UCP_FALL", 1},
	{SC6607_VOOCPHY_SS_TIMEOUT_FLAG_MASK, "SS_TIMEOUT", 1},
};

static enum oplus_cp_work_mode g_cp_support_work_mode[] = {
	CP_WORK_MODE_BYPASS,
	CP_WORK_MODE_2_TO_1,
};

static const struct regmap_config sc6607_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int sc6607_cp_set_sstimeout_ucp_enable(struct oplus_chg_ic_dev *ic_dev, bool enable);
static int sc6607_voocphy_set_sstimeout_ucp_enable(struct oplus_voocphy_manager *chip, bool enable);

static int sc6607_field_read(struct sc6607 *chip, enum sc6607_fields field_id, u8 *data)
{
	int ret = 0;
	int retry = SC6607_I2C_RETRY_READ_MAX_COUNT;
	int val;

	if (ARRAY_SIZE(sc6607_reg_fields) <= field_id)
		return ret;

	mutex_lock(&chip->i2c_rw_lock);
	ret = regmap_field_read(chip->regmap_fields[field_id], &val);
	mutex_unlock(&chip->i2c_rw_lock);
	if (ret < 0) {
		while (retry > 0 && atomic_read(&chip->driver_suspended) == 0) {
			usleep_range(SC6607_I2C_RETRY_DELAY_US, SC6607_I2C_RETRY_DELAY_US);
			mutex_lock(&chip->i2c_rw_lock);
			ret = regmap_field_read(chip->regmap_fields[field_id], &val);
			mutex_unlock(&chip->i2c_rw_lock);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	if (ret < 0)
		chg_err("i2c read fail: can't read field %d, %d\n", field_id, ret);
	else
		*data = val & 0xff;

	return ret;
}

static int sc6607_field_write(struct sc6607 *chip, enum sc6607_fields field_id, u8 val)
{
	int ret = 0;
	int retry = SC6607_I2C_RETRY_WRITE_MAX_COUNT;

	if (ARRAY_SIZE(sc6607_reg_fields) <= field_id)
		return ret;

	mutex_lock(&chip->i2c_rw_lock);
	ret = regmap_field_write(chip->regmap_fields[field_id], val);
	mutex_unlock(&chip->i2c_rw_lock);
	if (ret < 0) {
		while (retry > 0 && atomic_read(&chip->driver_suspended) == 0) {
			usleep_range(SC6607_I2C_RETRY_DELAY_US, SC6607_I2C_RETRY_DELAY_US);
			mutex_lock(&chip->i2c_rw_lock);
			ret = regmap_field_write(chip->regmap_fields[field_id], val);
			mutex_unlock(&chip->i2c_rw_lock);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	if (ret < 0)
		chg_err("i2c read fail: can't write field %d, %d\n", field_id, ret);

	return ret;
}

static void sc6607_i2c_error(struct oplus_voocphy_manager *voocphy, bool happen, bool read)
{
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return;

	chip = voocphy->priv_data;

	if (happen) {
		if (chip->error_reported)
			return;
		if (atomic_read(&chip->i2c_err_count) < I2C_ERR_NUM) {
			atomic_inc(&chip->i2c_err_count);
			return;
		}
		chip->error_reported = true;
		vote(chip->disable_votable, IIC_VOTER, true, 1, false);
		oplus_chg_ic_creat_err_msg(chip->cp_ic, OPLUS_IC_ERR_CP,
					   CP_ERR_I2C, "%s error",
					   read ? "read" : "write");
		oplus_chg_ic_err_trigger_and_clean(chip->cp_ic);
	} else {
		vote(chip->disable_votable, IIC_VOTER, false, 0, false);
		chip->error_reported = false;
		atomic_set(&chip->i2c_err_count, 0);
	}
}

static int __sc6607_voocphy_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	s32 ret;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (!voocphy) {
		chg_err("voocphy is NULL\n");
		return -EINVAL;
	}

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		sc6607_i2c_error(voocphy, true, true);
		chg_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}
	sc6607_i2c_error(voocphy, false, true);
	*data = (u8)ret;

	return 0;
}

static int __sc6607_voocphy_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	s32 ret;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (!voocphy)
		return -EINVAL;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		sc6607_i2c_error(voocphy, true, false);
		chg_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n", val, reg, ret);
		return ret;
	}
	sc6607_i2c_error(voocphy, false, false);
	return 0;
}

static int sc6607_voocphy_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	struct sc6607 *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (!voocphy || !voocphy->priv_data) {
		return -EINVAL;
	}

	chip = voocphy->priv_data;

	mutex_lock(&chip->i2c_rw_lock);
	ret = __sc6607_voocphy_read_byte(client, reg, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int sc6607_voocphy_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;
	struct sc6607 *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (!voocphy || !voocphy->priv_data) {
		return -EINVAL;
	}

	chip = voocphy->priv_data;

	mutex_lock(&chip->i2c_rw_lock);
	ret = __sc6607_voocphy_write_byte(client, reg, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static s32 sc6607_voocphy_read_word(struct i2c_client *client, u8 reg)
{
	s32 ret;
	struct sc6607 *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (!voocphy || !voocphy->priv_data) {
		return -EINVAL;
	}

	chip = voocphy->priv_data;

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		sc6607_i2c_error(voocphy, true, true);
		chg_err("i2c read word fail: can't read reg:0x%02X \n", reg);
		mutex_unlock(&chip->i2c_rw_lock);
		return ret;
	}
	sc6607_i2c_error(voocphy, false, true);
	mutex_unlock(&chip->i2c_rw_lock);
	return ret;
}

static s32 sc6607_voocphy_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;
	struct sc6607 *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (!voocphy || !voocphy->priv_data) {
		return -EINVAL;
	}

	chip = voocphy->priv_data;

	mutex_lock(&chip->i2c_rw_lock);
	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		sc6607_i2c_error(voocphy, true, false);
		chg_err("i2c write word fail: can't write 0x%02X to reg:0x%02X \n", val, reg);
		mutex_unlock(&chip->i2c_rw_lock);
		return ret;
	}
	sc6607_i2c_error(voocphy, false, false);
	mutex_unlock(&chip->i2c_rw_lock);
	return 0;
}

static int oplus_voocphy_get_fastchg_commu_ing(void)
{
	int fastchg_commu_ing = 0;
	struct oplus_mms *vooc_topic;
	union mms_msg_data data = { 0 };
	int rc;

	vooc_topic = oplus_mms_get_by_name("vooc");
	if (!vooc_topic)
		return 0;

	rc = oplus_mms_get_item_data(vooc_topic, VOOC_ITEM_FASTCHG_COMMU_ING, &data, true);
	if (!rc)
		fastchg_commu_ing = data.intval;

	return fastchg_commu_ing;
}

static const u32 sy6607_adc_step[] = {
	2500, 3750, 5000, 1250, 1250, 1220, 1250, 9766, 9766, 5, 156,
};

static int sc6607_bulk_read(struct sc6607 *chip, u8 reg, u8 *val, size_t count)
{
	int ret;
	int retry = SC6607_I2C_RETRY_READ_MAX_COUNT;

	ret = regmap_bulk_read(chip->regmap, reg, val, count);
	if (ret < 0) {
		while (retry > 0 && atomic_read(&chip->driver_suspended) == 0) {
			usleep_range(SC6607_I2C_RETRY_DELAY_US, SC6607_I2C_RETRY_DELAY_US);
			ret = regmap_bulk_read(chip->regmap, reg, val, count);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	if (ret < 0)
		chg_err("i2c bulk read failed: can't read 0x%0x, ret:%d\n", reg, ret);

	return ret;
}

static int sc6607_hk_get_adc(struct sc6607 *chip, enum SC6607_ADC_MODULE id)
{
	u32 reg = SC6607_REG_HK_IBUS_ADC + id * SC6607_ADC_REG_STEP;
	u8 val[2] = { 0 };
	u64 ret;
	int rc = 0;
	u8 adc_open = 0;

	if (!chip)
		return -EINVAL;

	sc6607_field_read(chip, F_ADC_EN, &adc_open);
	if (!adc_open) {
		chg_err("adc_open false\n");
		return 0;
	}

	mutex_lock(&chip->adc_read_lock);
	sc6607_field_write(chip, F_ADC_FREEZE, 1);
	rc = sc6607_bulk_read(chip, reg, val, sizeof(val));
	sc6607_field_write(chip, F_ADC_FREEZE, 0);
	mutex_unlock(&chip->adc_read_lock);
	if (rc < 0) {
		return -EINVAL;
	}
	ret = val[1] + (val[0] << 8);
	if (id == SC6607_ADC_TDIE) {
		ret = (SC6607_ADC_IDTE_THD - ret) / 2;
	}  else {
		ret *= sy6607_adc_step[id];
	}
	return (int)ret;
}

static int sc6607_adc_read_ibus(struct sc6607 *chip)
{
	int ibus = 0;

	if (!chip || !chip->voocphy)
		return -EINVAL;

	ibus = sc6607_hk_get_adc(chip, SC6607_ADC_IBUS);
	ibus /= SC6607_UA_PER_MA;
	return ibus;
}

static int sc6607_adc_read_vbus_volt(struct sc6607 *chip)
{
	int vbus_vol = 0;

	if (!chip || !chip->voocphy)
		return -EINVAL;

	if (oplus_voocphy_get_fastchg_commu_ing()) {
		chg_info("svooc in communication\n");
		return chip->voocphy ->cp_vbus;
	}
	vbus_vol = sc6607_hk_get_adc(chip, SC6607_ADC_VBUS);
	vbus_vol /= SC6607_UV_PER_MV;

	return vbus_vol;
}

static int sc6607_adc_read_vac(struct sc6607 *chip)
{
	int vac_vol = 0;
	if (!chip)
		return -EINVAL;

	if (chip->voocphy && oplus_voocphy_get_fastchg_commu_ing()) {
		chg_info("svooc in communication\n");
		return chip->voocphy->cp_vac;
	}

	vac_vol = sc6607_hk_get_adc(chip, SC6607_ADC_VAC);
	vac_vol /= SC6607_UV_PER_MV;
	return vac_vol;
}

static int sc6607_voocphy_set_predata(struct oplus_voocphy_manager *voocphy, u16 val)
{
	s32 ret;

	if (!voocphy)
		return -EINVAL;

	ret = sc6607_voocphy_write_word(voocphy->client, SC6607_REG_PREDATA_VALUE, val);
	if (ret < 0) {
		chg_err("failed: write predata\n");
		return -EIO;
	}
	chg_info("write predata 0x%0x\n", val);
	return ret;
}

static int sc6607_voocphy_set_txbuff(struct oplus_voocphy_manager *voocphy, u16 val)
{
	s32 ret;

	if (!voocphy)
		return -EINVAL;

	ret = sc6607_voocphy_write_word(voocphy->client, SC6607_REG_TXBUF_DATA0, val);
	if (ret < 0) {
		chg_err("write txbuff\n");
		return -EIO;
	}

	return ret;
}

static int sc6607_voocphy_get_adapter_info(struct oplus_voocphy_manager *voocphy)
{
	s32 data;

	if (!voocphy)
		return -EINVAL;

	data = sc6607_voocphy_read_word(voocphy->client, SC6607_REG_ADAPTER_INFO);

	if (data < 0) {
		chg_err("\n");
		return -EIO;
	}

	VOOCPHY_DATA16_SPLIT(data, voocphy->voocphy_rx_buff, voocphy->vooc_flag);
	chg_info("data: 0x%0x, vooc_flag: 0x%0x, vooc_rxdata: 0x%0x\n", data, voocphy->vooc_flag, voocphy->voocphy_rx_buff);

	return 0;
}

static void sc6607_voocphy_update_data(struct oplus_voocphy_manager *voocphy)
{
	u8 data_block[18] = { 0 };
	u8 data = 0;
	s32 ret = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return;

	chip = voocphy->priv_data;

	ret = sc6607_field_write(chip, F_WD_TIME_RST, true);
	sc6607_voocphy_read_byte(voocphy->client, SC6607_REG_CP_FLT_FLG, &data);
	chip->interrupt_flag = data;

	/*parse data_block for improving time of interrupt*/
	sc6607_field_write(chip, F_ADC_FREEZE, 1);
	ret = i2c_smbus_read_i2c_block_data(voocphy->client, SC6607_REG_HK_IBUS_ADC, 18, data_block);
	sc6607_field_write(chip, F_ADC_FREEZE, 0);
	if (ret < 0) {
		sc6607_i2c_error(voocphy, true, true);
		chg_err("read vsys vbat error \n");
	} else {
		sc6607_i2c_error(voocphy, false, true);
	}
	voocphy->cp_ichg = (((data_block[0] & SC6607_VOOCPHY_IBUS_POL_H_MASK) << SC6607_VOOCPHY_IBUS_POL_H_SHIFT) |
			data_block[1]) * SC6607_VOOCPHY_IBUS_ADC_LSB;

	voocphy->cp_vbus = (((data_block[2] & SC6607_VOOCPHY_VBUS_POL_H_MASK) << SC6607_VOOCPHY_VBUS_POL_H_SHIFT) |
			data_block[3]) * SC6607_VOOCPHY_VBUS_ADC_LSB;

	voocphy->cp_vac = (((data_block[4] & SC6607_VOOCPHY_VAC_POL_H_MASK) << SC6607_VOOCPHY_VAC_POL_H_SHIFT) |
			data_block[5]) * SC6607_VOOCPHY_VAC_ADC_LSB;

	voocphy->cp_vbat = (((data_block[6] & SC6607_VOOCPHY_VBAT_POL_H_MASK) << SC6607_VOOCPHY_VBAT_POL_H_SHIFT) |
			data_block[7]) * SC6607_VOOCPHY_VBAT_ADC_LSB;

	voocphy->cp_tsbus = (((data_block[14] & SC6607_VOOCPHY_TSBAT_POL_H_MASK) << SC6607_VOOCPHY_TSBAT_POL_H_SHIFT) |
			data_block[15]);
	voocphy->cp_tsbat = (((data_block[16] & SC6607_VOOCPHY_TSBAT_POL_H_MASK) << SC6607_VOOCPHY_TSBUS_POL_H_SHIFT) |
			data_block[17]);
	voocphy->cp_vsys = sc6607_hk_get_adc(chip, SC6607_ADC_VSYS);
	voocphy->cp_vsys /= SC6607_UV_PER_MV;

	voocphy->master_cp_ichg = voocphy->cp_ichg;
	chg_info("ichg=%d vbus=%d vac=%d vbat=%d vsys=%d int_flag=%d tsbus=%d tsbat=%d\n",
			voocphy->cp_ichg, voocphy->cp_vbus, voocphy->cp_vac, voocphy->cp_vbat,
			voocphy->cp_vsys, chip->interrupt_flag, voocphy->cp_tsbus, voocphy->cp_tsbat);
}

static int sc6607_voocphy_get_chg_enable(struct oplus_voocphy_manager *voocphy, u8 *data)
{
	int ret = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	ret = sc6607_field_read(chip, F_CP_EN, data);
	if (ret < 0) {
		chg_err("F_CP_EN err\n");
		return -EIO;
	}

	return ret;
}

static int sc6607_voocphy_get_cp_ichg(struct oplus_voocphy_manager *voocphy)
{
	int cp_ichg = 0;
	u8 cp_enable = 0;
	struct sc6607 *chip;
	bool reset_read_ibus = false;
	int ibus_devation = 0;
	int slave_ibus = 0;

	if (!voocphy || !voocphy->priv_data)
		return 0;

	chip = voocphy->priv_data;

	if (voocphy->voocphy_dual_cp_support) {
		slave_ibus = voocphy->slave_cp_ichg;
		ibus_devation = abs(voocphy->cp_ichg - slave_ibus);
		if (ibus_devation > voocphy->cp_ibus_devation) {
			chg_info("ibus_devation is %d\n" , ibus_devation);
			reset_read_ibus = true;
		}
	}
	if (oplus_voocphy_get_fastchg_commu_ing() && !reset_read_ibus) {
		chg_info("svooc in communication\n");
		return voocphy->master_cp_ichg;
	}

	sc6607_voocphy_get_chg_enable(voocphy, &cp_enable);

	if (cp_enable == 0)
		return 0;

	cp_ichg = sc6607_adc_read_ibus(chip);

	return cp_ichg;
}

static int sc6607_voocphy_reg_reset(struct oplus_voocphy_manager *voocphy, bool enable)
{
	int ret;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	ret = sc6607_field_write(chip, F_PHY_EN, enable);

	return ret;
}

static int sc6607_get_voocphy_enable(struct oplus_voocphy_manager *voocphy, u8 *data)
{
	int ret = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	ret = sc6607_field_read(chip, F_PHY_EN, data);
	if (ret < 0) {
		chg_err("SC6607_REG_PHY_CTRL\n");
		return -EIO;
	}
	chg_info("data = %d\n", *data);

	return ret;
}

static void sc6607_voocphy_dump_reg_in_err_issue(struct oplus_voocphy_manager *voocphy)
{
	int i = 0, p = 0;

	if (!voocphy)
		return;

	for (i = 0x0; i < 0X0F; i++) {
		sc6607_voocphy_read_byte(voocphy->client, i, &voocphy->reg_dump[p]);
		p = p + 1;
	}

	for (i = 0x0; i < 0X0F; i++) {
		p = p + 1;
		sc6607_voocphy_read_byte(voocphy->client, 0x60 + i, &voocphy->reg_dump[p]);
	}

	for (i = 0x0; i < 0X0F; i++) {
		p = p + 1;
		sc6607_voocphy_read_byte(voocphy->client, 0xA0 + i, &voocphy->reg_dump[p]);
	}

	chg_info("p[%d], ", p);

	return;
}

static int sc6607_voopchy_get_adc_enable(struct oplus_voocphy_manager *voocphy, u8 *data)
{
	int ret = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	ret = sc6607_field_read(chip, F_ADC_EN, data);

	if (ret < 0) {
		chg_err("F_ADC_EN\n");
		return -EIO;
	}

	return ret;
}

static int sc6607_adc_read_vbat(struct sc6607 *chip)
{
	int vbat = 0;

	if (!chip || !chip->voocphy)
		return -EINVAL;

	if (oplus_voocphy_get_fastchg_commu_ing()) {
		chg_info("svooc in communication\n");
		return chip->voocphy->cp_vbat;
	}
	vbat = sc6607_hk_get_adc(chip, SC6607_ADC_VBATSNS);
	vbat /= SC6607_UV_PER_MV;

	return vbat;
}

static int oplus_sc6607_get_vbat(struct oplus_voocphy_manager *voocphy)
{
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	return sc6607_adc_read_vbat(chip);
}

static int sc6607_voocphy_get_cp_vbus(struct oplus_voocphy_manager *voocphy)
{
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	return sc6607_adc_read_vbus_volt(chip);
}

static u8 sc6607_voocphy_get_int_value(struct oplus_voocphy_manager *voocphy)
{
	int ret = 0;
	u8 data = 0;
	u8 state = 0;

	if (!voocphy)
		return -EINVAL;

	ret = sc6607_voocphy_read_byte(voocphy->client, SC6607_REG_CP_FLT_FLG, &data); /*ibus ucp register*/
	if (ret < 0) {
		chg_err("read SC6607_REG_CP_FLT_FLG failed\n");
		return -EIO;
	}

	ret = sc6607_voocphy_read_byte(voocphy->client, SC6607_REG_CP_PMID2OUT_FLG, &state); /*pmid2out protection*/
	if (ret < 0) {
		chg_err("read SC6607_REG_CP_PMID2OUT_FLG failed\n");
		return -EIO;
	}
	chg_info("SC6607_REG_CP_FLT_FLG 0x6b=0x%x SC6607_REG_CP_PMID2OUT_FLG(0x6c)=0x%x", data, state);
	return data;
}

static int sc6607_voocphy_set_chg_enable(struct oplus_voocphy_manager *voocphy, bool enable)
{
	u8 data = 0;
	int ret = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	if (enable)
		ret = sc6607_field_write(chip, F_CP_EN, true);
	else
		ret = sc6607_field_write(chip, F_CP_EN, false);
	sc6607_voocphy_read_byte(voocphy->client, SC6607_REG_CP_CTRL, &data); /*performance mode , CP mode*/
	return 0;
}

/* init ucp deglitch 160ms ,which can fix the bug */
static void sc6607_voopchy_set_pd_svooc_config(struct oplus_voocphy_manager *voocphy, bool enable)
{
	chg_info("enter\n");
}

static bool sc6607_voopchy_get_pd_svooc_config(struct oplus_voocphy_manager *voocphy)
{
	int ret = 0;
	u8 data = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	ret = sc6607_field_read(chip, F_IBUS_UCP_DIS, &data);
	if (ret < 0) {
		chg_err("F_IBUS_UCP_DIS\n");
		return false;
	}

	if (data)
		return true;
	else
		return false;
}

static int sc6607a_set_chg_pmid2vout(struct oplus_voocphy_manager *chip, bool enable, int reason)
{
	if (!chip) {
		chg_err("Failed\n");
		return 0;
	}

	if (enable) {
		if (reason == SETTING_REASON_SVOOC || reason == SETTING_REASON_VOOC)
			return sc6607_voocphy_write_byte(chip->client, SC6607_REG_PMID2OUT_OVP,
			    SC6607A_REG_PMID2OUT_OVP_600MV);
		else
			chg_err("no type for set_chg_pmid2out\n");
	}

	return 0;
}

static bool sc6607a_get_chg_pmid2vout(struct oplus_voocphy_manager *chip)
{
	int ret = 0;
	u8 data = 0;

	if (!chip) {
		chg_err("Failed\n");
		return false;
	}

	ret = sc6607_voocphy_read_byte(chip->client, SC6607_REG_PMID2OUT_OVP, &data);
	if (ret < 0) {
		chg_err("read SC6607A REG_PMID2OUT_OVP error\n");
		return false;
	}

	chg_info("SC6607A REG_PMID2OUT_OVP = 0x%0x\n", data);

	if (data == SC6607A_REG_PMID2OUT_OVP_600MV)
		return true;
	else
		return false;
}

static int sc6607_voocphy_set_adc_enable(struct oplus_voocphy_manager *voocphy, bool enable)
{
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	if (enable)
		return sc6607_field_write(chip, F_ADC_EN, true);
	else
		return true;
}

static int sc6607_set_charge_watchdog_timer(struct sc6607 *voocphy, u32 timeout)
{
	u8 val = 0;
	int ret;

	if (!voocphy)
		return -EINVAL;

	if (timeout <= SC6607_WD_TIMEOUT_S(0))
		val = SC6607_WD_DISABLE;
	else if (timeout <= SC6607_WD_TIMEOUT_S(500))
		val = SC6607_WD_0_5_S;
	else if (timeout <= SC6607_WD_TIMEOUT_S(1000))
		val = SC6607_WD_1_S;
	else if (timeout <= SC6607_WD_TIMEOUT_S(2000))
		val = SC6607_WD_2_S;
	else if (timeout <= SC6607_WD_TIMEOUT_S(20000))
		val = SC6607_WD_20_S;
	else if (timeout <= SC6607_WD_TIMEOUT_S(40000))
		val = SC6607_WD_40_S;
	else if (timeout <= SC6607_WD_TIMEOUT_S(80000))
		val = SC6607_WD_80_S;
	else
		val = SC6607_WD_160_S;

	ret = sc6607_field_write(voocphy, F_WD_TIMER, val);
	chg_info("timeout:%d, val=0x%x\n", timeout, val);

	return ret;
}


static void sc6607_voocphy_send_handshake(struct oplus_voocphy_manager *voocphy)
{
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return;

	chip = voocphy->priv_data;
	if (chip->shutdown_pending) {
		chg_info("skip handshake pulse in shutdown state\n");
		return;
	}

	chg_info("\n");
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_PHY_CTRL, 0x81);
}

static int sc6607_voocphy_reset_voocphy(struct oplus_voocphy_manager *voocphy)
{
	u8 reg_data = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	chg_info("reset\n");
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_VOOCPHY_IRQ, 0x7F);
	/* close CP */
	if (chip->is_sc6607a) {
		sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_CP_CTRL, 0x40);
		sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_DPDM_CTRL_2, 0x00);
		sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_DPDM_NONSTD_STAT, 0x00);
	} else {
		sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_CP_CTRL, 0x80);
	}

	/* hwic config with plugout */
	reg_data = 0x20 | (voocphy->ovp_reg & 0x1f);
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_VBATSNS_OVP, reg_data); /*Vbat ovp 4.65V*/
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_VAC_VBUS_OVP, chip->vac_vbus_ovp_reg); /*Vac ovp 12V,Vbus OVP 10V*/
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_IBUS_OCP_UCP, 0x6B); /*ucp deglitch1 160ms,IBUS OCP 3.75A*/

	/* clear tx data */
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_TXBUF_DATA0, 0x00);
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_TXBUF_DATA1, 0x00);

	/* set D+ HiZ */
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_DPDM_CTRL, 0x00);

	/* select big bang mode */

	/* disable vooc */
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_PHY_CTRL, 0x00);

	/* set predata */
	sc6607_voocphy_write_word(voocphy->client, SC6607_REG_PREDATA_VALUE, 0x0);
	sc6607_set_charge_watchdog_timer(chip, 0);
	sc6607_field_write(chip, F_PERFORMANCE_EN, 0);
	sc6607_voocphy_set_sstimeout_ucp_enable(voocphy, true);
	return VOOCPHY_SUCCESS;
}

static int sc6607_voocphy_reactive_voocphy(struct oplus_voocphy_manager *voocphy)
{
	if (!voocphy)
		return -EINVAL;

	sc6607_voocphy_write_word(voocphy->client, SC6607_REG_PHY_CTRL, 0x0);
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_DP_HOLD_TIME, 0x60); /*dp hold time to endtime*/
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_DPDM_CTRL, 0x24); /*dp dm pull down 20k*/
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_T5_T7_SETTING, 0xD1); /*T5~T7 setting*/
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_TXBUF_DATA0, 0x00);
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_TXBUF_DATA1, 0x00);
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_VOOCPHY_IRQ, 0x05); /*mask rx start and txdone flag*/
	sc6607_voocphy_send_handshake(voocphy);

	return VOOCPHY_SUCCESS;
}

static int sc6607_voocphy_init_device(struct oplus_voocphy_manager *voocphy)
{
	u8 reg_data = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_VAC_VBUS_OVP, chip->vac_vbus_ovp_reg); /*Vac ovp 12V	 Vbus_ovp 10V*/
	reg_data = 0x20 | (voocphy->ovp_reg & 0x1f);
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_VBATSNS_OVP, reg_data); /*VBAT_OVP:4.65V */
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_IBUS_OCP_UCP, 0x6B); /* IBUS_OCP_UCP:3.75A */
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_PHY_CTRL, 0x00); /*VOOC_CTRL:disable */
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_DP_HOLD_TIME, 0x60); /*dp hold time to endtime*/
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_CP_INT_MASK, 0x37);
	/*close ucp rising int,change to bit1 bit2 bit4 bit5 1 mask ucp/adc rising int*/
	return 0;
}

static int sc6607_voocphy_init_vooc(struct oplus_voocphy_manager *voocphy)
{
	u8 data = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	chg_info("\n");
	sc6607_voocphy_reg_reset(voocphy, true);
	sc6607_voocphy_init_device(voocphy);

	sc6607_voocphy_write_word(voocphy->client, SC6607_REG_PREDATA_VALUE, 0x0);
	if (chip->is_sc6607a) {
		sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_DPDM_CTRL, 0xB4);
		sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_DPDM_CTRL_2, 0x39);
		sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_DPDM_NONSTD_STAT, 0x08);
	} else {
		sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_DPDM_CTRL, 0x24); /*dp dm pull down 20k*/
	}
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_T5_T7_SETTING, 0xD1); /*T5~T7 setting*/
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_VOOCPHY_IRQ, 0x05); /*mask rx start and txdone flag*/

	sc6607_field_read(chip, F_DPDM_3P3_EN, &data);
	chg_info("data:%d\n", data);
	sc6607_field_write(chip, F_DPDM_3P3_EN, true);


	return 0;
}

static int sc6607_voocphy_svooc_ovp_hw_setting(struct oplus_voocphy_manager *voocphy)
{
	int ret = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	ret = sc6607_field_write(chip, F_VAC_OVP, 0x00);
	ret = sc6607_field_write(chip, F_VBUS_OVP, 0x02);

	return 0;
}

static int sc6607_voocphy_svooc_hw_setting(struct oplus_voocphy_manager *voocphy)
{
	u8 data = 0;
	u8 reg_data = 0;
	int ret = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	ret = sc6607_field_write(chip, F_VAC_OVP, 0x00); /*VAC_OVP:12v VBUS_OVP:10v*/
	ret = sc6607_field_write(chip, F_VBUS_OVP, 0x02);
	reg_data = voocphy->ocp_reg & 0xff;
	ret = sc6607_field_write(chip, F_IBUS_OCP, reg_data); /*IBUS_OCP_UCP:4.25A*/
	ret = sc6607_set_charge_watchdog_timer(chip, 1000);
	ret = sc6607_field_write(chip, F_MODE, 0x0);
	if (chip->is_sc6607a)
		ret = sc6607_voocphy_write_byte(voocphy->client,
		    SC6607_REG_PMID2OUT_OVP, 0x26); /*REG_PMID2OUT_OVP set 2000mv */
	else
		ret = sc6607_field_write(chip, F_PMID2OUT_OVP, 0x07);
	ret = sc6607_field_write(chip, F_CHG_EN, true);
	ret = sc6607_field_write(chip, F_PERFORMANCE_EN, true);
	sc6607_voocphy_read_byte(voocphy->client, SC6607_REG_CP_CTRL, &data);
	sc6607_voocphy_set_sstimeout_ucp_enable(voocphy, false);
	chg_info("data:0x%x\n", data);

	return 0;
}

static int sc6607_voocphy_vooc_hw_setting(struct oplus_voocphy_manager *voocphy)
{
	int ret = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	ret = sc6607_field_write(chip, F_VAC_OVP, 2); /*VAC_OVP:6.5v VBUS_OVP:6v*/
	ret = sc6607_field_write(chip, F_VBUS_OVP, 0);
	ret = sc6607_field_write(chip, F_IBUS_OCP, 0x0F); /*IBUS_OCP_UCP:4.8A,160ms UCP*/
	ret = sc6607_set_charge_watchdog_timer(chip, 1000);
	/*need to set bit1 bp:1 default 0; mode bit1 sc:0 bp:1 default 0  mos bit0, enable:1*/
	ret = sc6607_field_write(chip, F_MODE, 0x1);
	ret = sc6607_field_write(chip, F_CHG_EN, true);
	ret = sc6607_field_write(chip, F_PERFORMANCE_EN, true);
	if (chip->is_sc6607a)
		ret = sc6607_voocphy_write_byte(voocphy->client,
		    SC6607_REG_PMID2OUT_OVP, 0x26); /*REG_PMID2OUT_OVP set 2000mv */
	sc6607_voocphy_set_sstimeout_ucp_enable(voocphy, false);
	return 0;
}

static int sc6607_voocphy_5v2a_hw_setting(struct oplus_voocphy_manager *voocphy)
{
	int ret = 0;
	u8 reg_data = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	ret = sc6607_field_write(chip, F_VAC_OVP, 2); /*VAC_OVP:6.5v VBUS_OVP:6v*/
	ret = sc6607_field_write(chip, F_VBUS_OVP, 0);
	ret = sc6607_field_write(chip, F_MODE, 0x0); /*close CPx*/
	ret = sc6607_field_write(chip, F_CP_EN, 0x0);
	ret = sc6607_set_charge_watchdog_timer(chip, 0);
	ret = sc6607_field_write(chip, F_PHY_EN, 0x0); /*VOOC_disable*/
	ret = sc6607_field_write(chip, F_CHG_EN, true); /*performance mode disable , CP mode*/
	ret = sc6607_field_write(chip, F_PERFORMANCE_EN, false);
	reg_data = 0x20 | (voocphy->ovp_reg & 0x1f);
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_VBATSNS_OVP, reg_data); /* VBAT_OVP:4.65V */
	return 0;
}

static int sc6607_voocphy_pdqc_hw_setting(struct oplus_voocphy_manager *voocphy)
{
	int ret = 0;
	u8 reg_data = 0;
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return -EINVAL;

	chip = voocphy->priv_data;

	reg_data = 0x20 | (voocphy->ovp_reg & 0x1f);
	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_VBATSNS_OVP, reg_data); /* VBAT_OVP:4.65V */
	ret = sc6607_field_write(chip, F_VAC_OVP, 0); /*VAC_OVP:12v VBUS_OVP:10v*/
	ret = sc6607_field_write(chip, F_VBUS_OVP, 1);
	ret = sc6607_field_write(chip, F_PHY_EN, 0x0); /* close CP*/
	ret = sc6607_field_write(chip, F_MODE, 0x0);
	ret = sc6607_field_write(chip, F_CP_EN, 0x0);
	ret = sc6607_set_charge_watchdog_timer(chip, 0);

	sc6607_voocphy_write_byte(voocphy->client, SC6607_REG_PHY_CTRL, 0x00); /*VOOC_disable*/
	ret = sc6607_field_write(chip, F_CHG_EN, true); /*performance mode disable , CP mode*/
	ret = sc6607_field_write(chip, F_PERFORMANCE_EN, false);
	return 0;
}

static void sc6607_voocphy_hardware_init(struct oplus_voocphy_manager *voocphy)
{
	if (!voocphy)
		return;

	sc6607_voocphy_init_device(voocphy);
}

static int sc6607_voocphy_hw_setting(struct oplus_voocphy_manager *voocphy, int reason)
{
	if (!voocphy)
		return -EINVAL;

	switch (reason) {
	case SETTING_REASON_PROBE:
	case SETTING_REASON_RESET:
		sc6607_voocphy_init_device(voocphy);
		chg_info("SETTING_REASON_RESET OR PROBE\n");
		break;
	case SETTING_REASON_COPYCAT_SVOOC:
		sc6607_voocphy_svooc_ovp_hw_setting(voocphy);
		chg_info("SETTING_REASON_COPYCAT_SVOOC\n");
		break;
	case SETTING_REASON_SVOOC:
		sc6607_voocphy_svooc_hw_setting(voocphy);
		chg_info("SETTING_REASON_SVOOC\n");
		break;
	case SETTING_REASON_VOOC:
		sc6607_voocphy_vooc_hw_setting(voocphy);
		chg_info("SETTING_REASON_VOOC\n");
		break;
	case SETTING_REASON_5V2A:
		sc6607_voocphy_5v2a_hw_setting(voocphy);
		chg_info("SETTING_REASON_5V2A\n");
		break;
	case SETTING_REASON_PDQC:
		sc6607_voocphy_pdqc_hw_setting(voocphy);
		chg_info("SETTING_REASON_PDQC\n");
		break;
	default:
		chg_err("do nothing\n");
		break;
	}

	return 0;
}

static int sc6607_voocphy_dump_registers(struct oplus_voocphy_manager *voocphy)
{
	int rc = 0;
	u8 addr;
	u8 val_buf[16] = { 0x0 };

	for (addr = SC6607_REG_VBATSNS_OVP; addr <= SC6607_REG_CP_PMID2OUT_FLG; addr++) {
		rc = sc6607_voocphy_read_byte(voocphy->client, addr, &val_buf[addr - SC6607_REG_VBATSNS_OVP]);
		if (rc < 0) {
			chg_err("Couldn't read 0x%02x, rc = %d\n", addr, rc);
			break;
		}
	}

	chg_info(":[0~5][0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n", val_buf[0], val_buf[1], val_buf[2], val_buf[3],
		val_buf[4], val_buf[5]);
	chg_info(":[6~c][0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n", val_buf[6], val_buf[7], val_buf[8], val_buf[9],
		val_buf[0xa], val_buf[0xb], val_buf[0xc]);

	return 0;
}

static bool sc6607_voocphy_check_cp_int_happened(struct oplus_voocphy_manager *voocphy, bool *dump_reg, bool *send_info)
{
	int i = 0;

	for (i = 0; i < SC6607_IRQ_EVNET_NUM; i++) {
		if ((sc6607_int_flag[i].mask & voocphy->interrupt_flag) && sc6607_int_flag[i].mark_except) {
			chg_err("cp int happened %s\n", sc6607_int_flag[i].except_info);
			if (sc6607_int_flag[i].mask != SC6607_VOOCPHY_VBATSNS_OVP_FLAG_MASK &&
			    sc6607_int_flag[i].mask != SC6607_VOOCPHY_VBAT_OVP_FLAG_MASK &&
			    sc6607_int_flag[i].mask != SC6607_VOOCPHY_SS_TIMEOUT_FLAG_MASK)
				*dump_reg = true;
			return true;
		}
	}

	return false;
}

static int sc6607_voocphy_set_sstimeout_ucp_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	int rc = 0;
	struct sc6607 *dev;
	dev = chip->priv_data;

	if (!chip->fcl_support)
		return -EINVAL;

	if (!dev) {
		chg_err("sc6607 chip is NULL\n");
		return -ENODEV;
	}

	rc = sc6607_cp_set_sstimeout_ucp_enable(dev->cp_ic, enable);

	return rc;
}

static int sc6607a_get_chip_id(struct oplus_voocphy_manager *chip)
{
	struct sc6607 *dev;
	dev = chip->priv_data;

	if (!dev) {
		chg_err("sc6607 chip is NULL\n");
		return CHIP_ID_DEFAULT;
	}

	if (dev->is_sc6607a)
		return CHIP_ID_SC6607A;
	else
		return CHIP_ID_DEFAULT;
}

static struct oplus_voocphy_operations sc6607_voocphy_ops = {
	.hardware_init = sc6607_voocphy_hardware_init,
	.hw_setting = sc6607_voocphy_hw_setting,
	.init_vooc = sc6607_voocphy_init_vooc,
	.set_predata = sc6607_voocphy_set_predata,
	.set_txbuff = sc6607_voocphy_set_txbuff,
	.get_adapter_info = sc6607_voocphy_get_adapter_info,
	.update_data = sc6607_voocphy_update_data,
	.get_chg_enable = sc6607_voocphy_get_chg_enable,
	.set_chg_enable = sc6607_voocphy_set_chg_enable,
	.reset_voocphy = sc6607_voocphy_reset_voocphy,
	.reactive_voocphy = sc6607_voocphy_reactive_voocphy,
	.send_handshake = sc6607_voocphy_send_handshake,
	.get_cp_vbat = oplus_sc6607_get_vbat,
	.get_cp_vbus = sc6607_voocphy_get_cp_vbus,
	.get_int_value = sc6607_voocphy_get_int_value,
	.get_adc_enable = sc6607_voopchy_get_adc_enable,
	.set_adc_enable = sc6607_voocphy_set_adc_enable,
	.get_ichg = sc6607_voocphy_get_cp_ichg,
	.set_pd_svooc_config = sc6607_voopchy_set_pd_svooc_config,
	.get_pd_svooc_config = sc6607_voopchy_get_pd_svooc_config,
	.get_voocphy_enable = sc6607_get_voocphy_enable,
	.get_chip_id = sc6607a_get_chip_id,
	.set_chg_pmid2out = sc6607a_set_chg_pmid2vout,
	.get_chg_pmid2out = sc6607a_get_chg_pmid2vout,
	.dump_voocphy_reg = sc6607_voocphy_dump_reg_in_err_issue,
	.check_cp_int_happened = sc6607_voocphy_check_cp_int_happened,
	.set_sstimeout_ucp_enable = sc6607_voocphy_set_sstimeout_ucp_enable,
};

static int sc6607_voocphy_charger_choose(struct oplus_voocphy_manager *voocphy)
{
	int ret;

	if (!oplus_voocphy_chip_is_null()) {
		chg_err("oplus_voocphy_chip already exists!");
		return 0;
	} else {
		ret = i2c_smbus_read_byte_data(voocphy->client, 0x07);
		chg_info("0x07 = %d\n", ret);
		if (ret < 0) {
			chg_err("i2c communication fail");
			return -EPROBE_DEFER;
		} else {
			return 0;
		}
	}
}

static struct oplus_chg_ic_virq sc6607_cp_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int sc6607_get_input_node_impedance(void *data)
{
	struct sc6607 *chip;
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

static int sc6607_get_output_node_impedance(void *data)
{
	struct sc6607 *chip;
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

static int sc6607_init_imp_node(struct sc6607 *chip, struct device_node *of_node)
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
				oplus_imp_node_register(child, chip->dev, chip, sc6607_get_input_node_impedance);
			if (IS_ERR_OR_NULL(chip->input_imp_node)) {
				chg_err("%s register error, rc=%ld\n", child->name, PTR_ERR(chip->input_imp_node));
				chip->input_imp_node = NULL;
				continue;
			}
		} else if (!strcmp(name, "cp_output")) {
			chip->output_imp_node =
				oplus_imp_node_register(child, chip->dev, chip, sc6607_get_output_node_impedance);
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

static int sc6607_vooc_hw_setting(struct sc6607 *chip)
{
	int ret = 0;

	if (!chip)
		return 0;

	chg_info("\n");
	ret = sc6607_field_write(chip, F_VBATSNS_OVP, chip->ovp_reg); /*VBAT_OVP:4.8V*/
	ret = sc6607_field_write(chip, F_IBUS_OCP, chip->ocp_reg); /*IBUS_OCP_UCP:4.75A*/
	ret = sc6607_field_write(chip, F_VAC_OVP, 2); /*VAC_OVP:6.5v VBUS_OVP:6v*/
	ret = sc6607_field_write(chip, F_VBUS_OVP, 0); /*VBUS_OVP:6v*/
	ret = sc6607_field_write(chip, F_MODE, 0x1); /*bypass*/
	ret = sc6607_field_write(chip, F_CP_EN, 0x0);
	ret = sc6607_field_write(chip, F_PHY_EN, 0x0); /*VOOC_disable*/
	ret = sc6607_field_write(chip, F_CHG_EN, true); /*performance mode disable , CP mode*/
	ret = sc6607_field_write(chip, F_PERFORMANCE_EN, true);
	if (chip->is_sc6607a)
		ret = sc6607_voocphy_write_byte(chip->client,
		    SC6607_REG_PMID2OUT_OVP, 0x26); /*REG_PMID2OUT_OVP set 2000mv */
	return ret;
}

static int sc6607_svooc_hw_setting(struct sc6607 *chip)
{
	int ret = 0;

	if (!chip)
		return 0;

	chg_info("\n");
	ret = sc6607_field_write(chip, F_VBATSNS_OVP, chip->ovp_reg); /*VBAT_OVP:4.8V*/
	ret = sc6607_field_write(chip, F_IBUS_OCP, chip->ocp_reg); /*IBUS_OCP_UCP:4750A*/
	ret = sc6607_field_write(chip, F_VAC_OVP, 0); /*VAC_OVP:12v VBUS_OVP:10v*/
	ret = sc6607_field_write(chip, F_VBUS_OVP, 2);    /*VBUS_OVP:12v*/
	ret = sc6607_field_write(chip, F_PHY_EN, 0x0); /* close CP*/
	ret = sc6607_field_write(chip, F_MODE, 0x0);
	ret = sc6607_field_write(chip, F_CHG_EN, true); /*performance mode disable , CP mode*/
	ret = sc6607_field_write(chip, F_PERFORMANCE_EN, true);
	if (chip->is_sc6607a)
		ret = sc6607_voocphy_write_byte(chip->client,
		    SC6607_REG_PMID2OUT_OVP, 0x26); /*REG_PMID2OUT_OVP set 2000mv */
	else
		ret = sc6607_field_write(chip, F_PMID2OUT_OVP, 0x06); /*F_PMID2OUT_OVP set 550mv */
	return ret;
}

static int sc6607_check_work_mode_support(enum oplus_cp_work_mode mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_cp_support_work_mode); i++) {
		if (g_cp_support_work_mode[i] == mode)
			return true;
	}
	return false;
}

static u8 g_sc6607_cp_status_reg[SC6607_CP_STATUS_REG_MAX] = {
	0x6b, 0x6c
};

static void sc6607_cp_regdump_work(struct work_struct *work)
{
	struct sc6607 *chip = container_of(work, struct sc6607, cp_regdump_work);
	char *buf;
	int i;
	size_t index = 0;
	u8 data;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return;

	for (i = 0; i < SC6607_CP_STATUS_REG_MAX; i++) {
		data = 0;
		sc6607_voocphy_read_byte(chip->client, g_sc6607_cp_status_reg[i], &data);
		index += snprintf(buf + index, ERR_MSG_BUF, "0x%02x=%02x,",
			g_sc6607_cp_status_reg[i], data);
	}
	if (index > 0)
		buf[index - 1] = 0;

	oplus_chg_ic_creat_err_msg(chip->cp_ic, OPLUS_IC_ERR_CP,
		CP_ERR_REG_INFO, "%s", buf);
	oplus_chg_ic_err_trigger_and_clean(chip->cp_ic);
	kfree(buf);
}

static int sc6607_cp_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s init\n", ic_dev->manu_name);

	vote(chip->disable_votable, OCP_VOTER, false, 0, false);
	vote(chip->disable_votable, UCP_VOTER, false, 0, false);
	vote(chip->disable_votable, IIC_VOTER, false, 0, false);
	vote(chip->disable_votable, PARENT_VOTER, false, 0, false);
	chip->error_reported = false;
	atomic_set(&chip->i2c_err_count, 0);

	return 0;
}

static int sc6607_cp_exit(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s exit\n", ic_dev->manu_name);
	vote(chip->disable_votable, PARENT_VOTER, true, 1, false);
	schedule_work(&chip->cp_regdump_work);

	return 0;
}

static int sc6607_cp_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	sc6607_voocphy_dump_reg_in_err_issue(chip->voocphy);
	return 0;
}

static int sc6607_cp_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int sc6607_cp_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static int sc6607_cp_hw_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	sc6607_voocphy_init_device(chip->voocphy);
	chip->work_start = false;
	return 0;
}

static int sc6607_cp_set_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	struct sc6607 *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (!sc6607_check_work_mode_support(mode)) {
		chg_err("not supported work mode, mode=%d\n", mode);
		return -EINVAL;
	}

	if (mode == CP_WORK_MODE_BYPASS)
		rc = sc6607_vooc_hw_setting(chip);
	else
		rc = sc6607_svooc_hw_setting(chip);

	if (rc < 0)
		chg_err("set work mode to %d error\n", mode);

	return rc;
}

static int sc6607_cp_get_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode *mode)
{
	struct sc6607 *chip;
	u8 val;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc6607_field_read(chip, F_MODE, &val);
	if (rc < 0) {
		chg_err("read SC6607_REG_CP_CTRL error, rc=%d\n", rc);
		return rc;
	}

	if (val)
		*mode = CP_WORK_MODE_BYPASS;
	else
		*mode = CP_WORK_MODE_2_TO_1;
	chg_info("mode = %d\n", *mode);
	return 0;
}

static int sc6607_cp_check_work_mode_support(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	return sc6607_check_work_mode_support(mode);
}

static int sc6607_cp_set_iin(struct oplus_chg_ic_dev *ic_dev, int iin)
{
	return 0;
}

static int sc6607_cp_get_vin(struct oplus_chg_ic_dev *ic_dev, int *vin)
{
	struct sc6607 *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc6607_adc_read_vbus_volt(chip);
	if (rc < 0) {
		chg_err("can't get cp vin, rc=%d\n", rc);
		return rc;
	}
	*vin = rc;

	return 0;
}

static int sc6607_cp_get_iin(struct oplus_chg_ic_dev *ic_dev, int *iin)
{
	struct sc6607 *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc6607_voocphy_get_cp_ichg(chip->voocphy);
	if (rc < 0) {
		chg_err("can't get cp iin, rc=%d\n", rc);
		return rc;
	}
	*iin = rc;

	return 0;
}

static int sc6607_cp_get_vout(struct oplus_chg_ic_dev *ic_dev, int *vout)
{
	struct sc6607 *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc6607_adc_read_vbat(chip);
	if (rc < 0) {
		chg_err("can't get cp vout, rc=%d\n", rc);
		return rc;
	}
	*vout = rc;

	return 0;
}

static int sc6607_cp_get_iout(struct oplus_chg_ic_dev *ic_dev, int *iout)
{
	struct sc6607 *chip;
	int iin;
	bool working;
	enum oplus_cp_work_mode work_mode;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

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

static int sc6607_cp_get_vac(struct oplus_chg_ic_dev *ic_dev, int *vac)
{
	struct sc6607 *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (!chip->vac_support)
		return -ENOTSUPP;

	rc = sc6607_adc_read_vac(chip);
	if (rc < 0) {
		chg_err("can't get cp vout, rc=%d\n", rc);
		return rc;
	}
	*vac = rc;

	return 0;
}

static int sc6607_cp_set_work_start(struct oplus_chg_ic_dev *ic_dev, bool start)
{
	struct sc6607 *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s work %s\n", chip->dev->of_node->name, start ? "start" : "stop");
	rc = sc6607_field_write(chip, F_CP_EN, start);
	if (rc < 0)
		return rc;
	chip->work_start = start;
	oplus_imp_node_set_active(chip->input_imp_node, start);
	oplus_imp_node_set_active(chip->output_imp_node, start);

	return 0;
}

static int sc6607_cp_set_sstimeout_ucp_enable(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	struct sc6607 *chip;
	int ret;
	u8 reg_data;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	ret = sc6607_voocphy_read_byte(chip->client, SC6607_REG_CP_CTRL_2, &reg_data);
	if ((enable && !(reg_data & SC6607_VOOCPHY_IBUS_UCP_DIS_MASK)) ||
	     (!enable && (reg_data & SC6607_VOOCPHY_IBUS_UCP_DIS_MASK)))
		return 0;

	if (enable && (reg_data & SC6607_VOOCPHY_IBUS_UCP_DIS_MASK)) {
		ret = sc6607_field_write(chip, F_IBUS_UCP_DIS, false);
		ret = sc6607_field_write(chip, F_SS_TIMEOUT, SC6607_VOOCPHY_SS_TIMEOUT_10S);
	} else {
		ret = sc6607_field_write(chip, F_IBUS_UCP_DIS, true);
		ret = sc6607_field_write(chip, F_SS_TIMEOUT, SC6607_VOOCPHY_SS_TIMEOUT_DISABLE);
	}

	sc6607_voocphy_read_byte(chip->client, SC6607_REG_CP_CTRL_2, &reg_data);

	chg_info("%s %s SC6607_REG_65 = 0x%0x\n", chip->dev->of_node->name, enable ? "enable" : "disable", reg_data);

	return 0;
}

static int sc6607_cp_get_work_status(struct oplus_chg_ic_dev *ic_dev, bool *start)
{
	struct sc6607 *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (!chip) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	rc = sc6607_field_read(chip, F_SWITCHING_STAT, &data);
	if (rc < 0) {
		chg_err("read F_SWITCHING_STAT error, rc=%d\n", rc);
		return rc;
	}

	*start = data & BIT(0);

	return 0;
}

static int sc6607_cp_watchdog_reset(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chg_info("\n");
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	return sc6607_field_write(chip, F_WD_TIME_RST, true);
}

static int sc6607_cp_wd_enable(struct oplus_chg_ic_dev *ic_dev, int timeout_ms)
{
	struct sc6607 *chip;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("timeout_ms %d\n", timeout_ms);

	if (timeout_ms)
		sc6607_set_charge_watchdog_timer(chip, 2000);
	else
		sc6607_set_charge_watchdog_timer(chip, 0);

	if (ret < 0) {
		chg_err("failed to set timeout_ms %d ret = %d)\n", timeout_ms, ret);
		return ret;
	}

	return 0;
}

static int sc6607_cp_set_ucp_disable(struct oplus_chg_ic_dev *ic_dev, bool disable)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s %s\n", chip->dev->of_node->name, disable ? "disable" : "enable");
	sc6607_field_write(chip, F_IBUS_UCP_DIS, disable);
	return 0;
}

static int sc6607_cp_set_pmid2vout_ovp_enable(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	struct sc6607 *chip;
	int rc;
	u8 data = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (!chip->is_sc6607a)
		return 0;

	rc = sc6607_voocphy_read_byte(chip->client, SC6607_REG_PMID2OUT_OVP, &data);
	if (rc < 0) {
		chg_err("read SC6607A REG_PMID2OUT_OVP error\n");
		return -EIO;
	}

	chg_info("SC6607A REG_PMID2OUT_OVP = 0x%0x, enable = %d\n", data, enable);

	if (enable && (data != SC6607A_REG_PMID2OUT_OVP_600MV)) {
		rc = sc6607_voocphy_write_byte(chip->client, SC6607_REG_PMID2OUT_OVP, SC6607A_REG_PMID2OUT_OVP_600MV);
		if (rc < 0) {
			chg_err("write SC6607A REG_PMID2OUT_OVP error\n");
			return -EIO;
		}
	}

	return 0;
}

static void *sc6607_cp_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, sc6607_cp_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, sc6607_cp_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, sc6607_cp_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, sc6607_cp_smt_test);
		break;
	case OPLUS_IC_FUNC_CP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_ENABLE, sc6607_cp_enable);
		break;
	case OPLUS_IC_FUNC_CP_HW_INTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_HW_INTI, sc6607_cp_hw_init);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_MODE, sc6607_cp_set_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_MODE, sc6607_cp_get_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT,
			sc6607_cp_check_work_mode_support);
		break;
	case OPLUS_IC_FUNC_CP_SET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_IIN, sc6607_cp_set_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VIN, sc6607_cp_get_vin);
		break;
	case OPLUS_IC_FUNC_CP_GET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IIN, sc6607_cp_get_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VOUT, sc6607_cp_get_vout);
		break;
	case OPLUS_IC_FUNC_CP_GET_IOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IOUT, sc6607_cp_get_iout);
		break;
	case OPLUS_IC_FUNC_CP_GET_VAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VAC, sc6607_cp_get_vac);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START, sc6607_cp_set_work_start);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_STATUS, sc6607_cp_get_work_status);
		break;
	case OPLUS_IC_FUNC_CP_WATCHDOG_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_WATCHDOG_RESET, sc6607_cp_watchdog_reset);
		break;
	case OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE, sc6607_cp_wd_enable);
		break;
	case OPLUS_IC_FUNC_CP_SET_UCP_DISABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_UCP_DISABLE, sc6607_cp_set_ucp_disable);
		break;
	case OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_SSTIMEOUT_UCP_ENABLE, sc6607_cp_set_sstimeout_ucp_enable);
		break;
	case OPLUS_IC_FUNC_CP_SET_PMID2VOUT_OVP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_PMID2VOUT_OVP_ENABLE, sc6607_cp_set_pmid2vout_ovp_enable);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static int oplus_chg_dpdm_switch_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	if (ic_dev->online)
		return 0;
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int oplus_chg_dpdm_switch_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static int oplus_chg_dpdm_switch_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int oplus_chg_dpdm_switch_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int oplus_chg_dpdm_switch_set_switch_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_dpdm_switch_mode mode)
{
	struct sc6607 *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (mode) {
	case DPDM_SWITCH_TO_AP:
		chg_info("dpdm switch to ap\n");
		break;
	case DPDM_SWITCH_TO_VOOC:
		chg_info("dpdm switch to vooc\n");
		break;
	case DPDM_SWITCH_TO_UFCS:
		chg_info("dpdm switch to ufcs\n");
		break;
	default:
		chg_err("not supported mode, mode=%d\n", mode);
		return -EINVAL;
	}
	chip->dpdm_switch_mode = mode;

	return rc;
}

static int oplus_chg_dpdm_switch_get_switch_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_dpdm_switch_mode *mode)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*mode = chip->dpdm_switch_mode;

	return 0;
}

static void *oplus_chg_dpdm_switch_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}
	if (!oplus_chg_ic_func_is_support(ic_dev, func_id)) {
		chg_info("%s: this func(=%d) is not supported\n",  ic_dev->name, func_id);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_dpdm_switch_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_dpdm_switch_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, oplus_chg_dpdm_switch_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, oplus_chg_dpdm_switch_smt_test);
		break;
	case OPLUS_IC_FUNC_SET_DPDM_SWITCH_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_DPDM_SWITCH_MODE,
			oplus_chg_dpdm_switch_set_switch_mode);
		break;
	case OPLUS_IC_FUNC_GET_DPDM_SWITCH_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_DPDM_SWITCH_MODE,
			oplus_chg_dpdm_switch_get_switch_mode);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_chg_dpdm_switch_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static void sc6607_ic_offline_work(struct work_struct *work)
{
	struct sc6607 *chip = container_of(work, struct sc6607, ic_offline_work);

	if (!chip->work_start)
		return;
	sc6607_cp_set_work_start(chip->cp_ic, false);
	sc6607_cp_enable(chip->cp_ic, false);
}

static int sc6607_disable_vote_callback(struct votable *votable, void *data,
					 int disable, const char *client, bool step)
{
	struct sc6607 *chip = data;
	bool online;

	if (disable < 0)
		online = true;
	else
		online = !disable;
	chg_info("[%s]%s set sc6607 online to %s\n", votable_name(votable),
		 client, online ? "true" : "false");

	if (chip->cp_ic ==NULL)
		return -ENODEV;
	if (chip->cp_ic->online == online)
		return 0;

	chip->cp_ic->online = online;
	if (online) {
		oplus_chg_ic_virq_trigger(chip->cp_ic, OPLUS_IC_VIRQ_ONLINE);
	} else {
		oplus_chg_ic_virq_trigger(chip->cp_ic, OPLUS_IC_VIRQ_OFFLINE);
		schedule_work(&chip->ic_offline_work);
	}

	return 0;
}

static int sc6607_ic_register(struct sc6607 *chip)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *child;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg;
	char votable_name[CP_IC_VOTEABLE_NAME_MAX] = { 0 };
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
			(void)sc6607_init_imp_node(chip, child);
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-sc6607:%d", ic_index);
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = sc6607_cp_get_func;
			ic_cfg.virq_data = sc6607_cp_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(sc6607_cp_virq_table);
			if (chip->disable_votable != NULL)
				break;

			snprintf(votable_name, CP_IC_VOTEABLE_NAME_MAX - 1,
				 "SC6607_DISABLE:%d", ic_index);
			chip->disable_votable = create_votable(votable_name,
				VOTE_SET_ANY, sc6607_disable_vote_callback, chip);
			if (IS_ERR(chip->disable_votable)) {
				rc = PTR_ERR(chip->disable_votable);
				chg_err("creat disable_votable error, rc=%d\n", rc);
				chip->disable_votable = NULL;
				return rc;
			}
			break;
		case OPLUS_CHG_IC_MISC:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "misc-dpdm-switch");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_dpdm_switch_get_func;
			ic_cfg.virq_data = oplus_chg_dpdm_switch_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_dpdm_switch_virq_table);
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
		case OPLUS_CHG_IC_MISC:
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_dev->type);
			continue;
		}

		of_platform_populate(child, NULL, NULL, chip->dev);
	}

	return 0;
}

static void sc6607_err_subs_callback(struct mms_subscribe *subs, enum mms_msg_type type, u32 id, bool sync)
{
	return;
}

static void sc6607_subscribe_error_topic(struct oplus_mms *topic, void *prv_data)
{
	struct sc6607 *chip = prv_data;

	chip->err_topic = topic;
	chip->err_subs =
		oplus_mms_subscribe(chip->err_topic, chip, sc6607_err_subs_callback, "sc6607");
	if (IS_ERR_OR_NULL(chip->err_subs)) {
		chg_err("subscribe error topic error, rc=%ld\n", PTR_ERR(chip->err_subs));
		return;
	}
}

static int sc6607_check_device_id(struct sc6607 *chip)
{
	int ret;
	u8 chip_id;

	if (!chip)
		return -EINVAL;

	chip->is_sc6607a = false;
	ret = sc6607_voocphy_read_byte(chip->client, SC6607_REG_DEVICE_ID, &chip_id);
	if (ret < 0) {
		return ret;
	}

	if (chip_id == SC6607A_CHIP_ID)
		chip->is_sc6607a = true;

	chip->chip_id = chip_id;
	chg_info("chip_id:%d\n", chip->chip_id);

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sc6607_voocphy_probe(struct i2c_client *client)
#else
static int sc6607_voocphy_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct sc6607 *chip;
	struct oplus_voocphy_manager *voocphy;
	int ret = 0;
	int i = 0;
	chg_info("start!\n");
	chip = devm_kzalloc(&client->dev, sizeof(struct sc6607), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	client->addr = SC6607_CP_REAL_ADDR;
	chip->dev = &client->dev;
	chip->client = client;
	mutex_init(&chip->i2c_rw_lock);
	mutex_init(&chip->adc_read_lock);

	chip->regmap = devm_regmap_init_i2c(client, &sc6607_regmap_cfg);
	if (IS_ERR(chip->regmap)) {
		chg_err("failed to allocate register map\n");
		ret = -EINVAL;
		goto err_regmap;
	}
	for (i = 0; i < ARRAY_SIZE(sc6607_reg_fields); i++) {
		chip->regmap_fields[i] = devm_regmap_field_alloc(chip->dev, chip->regmap, sc6607_reg_fields[i]);
		if (IS_ERR(chip->regmap_fields[i])) {
			chg_err("cannot allocate regmap field\n");
			ret = -EINVAL;
			goto err_regmap_fields;
		}
	}

	voocphy = devm_kzalloc(&client->dev, sizeof(struct oplus_voocphy_manager), GFP_KERNEL);
	if (voocphy == NULL) {
		chg_err("alloc voocphy buf error\n");
		ret = -ENOMEM;
		goto chg_err;
	}

	voocphy->client = client;
	voocphy->dev = &client->dev;
	voocphy->priv_data = chip;
	chip->voocphy = voocphy;
	i2c_set_clientdata(client, voocphy);

	ret = sc6607_voocphy_charger_choose(voocphy);
	if (ret < 0)
		goto err_init;
	sc6607_voocphy_dump_registers(voocphy);
	sc6607_check_device_id(chip);

	chip->vac_support = of_property_read_bool(chip->dev->of_node, "oplus,vac_support");
	chg_info("vac_support=%d\n", chip->vac_support);

	ret = of_property_read_u32(chip->dev->of_node, "vac_vbus_ovp_reg", &chip->vac_vbus_ovp_reg);
	if(ret < 0)
		chip->vac_vbus_ovp_reg = 0x01;
	chg_info("vac_vbus_ovp_reg = 0x%2x\n", chip->vac_vbus_ovp_reg);

	voocphy->ops = &sc6607_voocphy_ops;
	ret = oplus_register_voocphy(voocphy);
	if (ret < 0) {
		chg_err("failed to register voocphy, ret = %d", ret);
		goto reg_voocphy_err;
	}
	chip->ocp_reg = voocphy->ocp_reg;
	chip->ovp_reg = voocphy->ovp_reg;

	ret = sc6607_ic_register(chip);
	if (ret < 0) {
		chg_err("cp ic register error\n");
		goto cp_reg_err;
	}
	vote(chip->disable_votable, DEF_VOTER, false, 0, false);

	sc6607_cp_init(chip->cp_ic);
	oplus_mms_wait_topic("error", sc6607_subscribe_error_topic, chip);
	INIT_WORK(&chip->cp_regdump_work, sc6607_cp_regdump_work);
	INIT_WORK(&chip->ic_offline_work, sc6607_ic_offline_work);
	chg_info("end!\n");
	return 0;
cp_reg_err:
	if (chip->input_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->input_imp_node);
	if (chip->output_imp_node != NULL)
		oplus_imp_node_unregister(chip->dev, chip->output_imp_node);
	if (chip->disable_votable != NULL)
		destroy_votable(chip->disable_votable);
reg_voocphy_err:
	devm_kfree(&client->dev, voocphy);
chg_err:
err_init:
err_regmap_fields:
err_regmap:
	mutex_destroy(&chip->i2c_rw_lock);
	mutex_destroy(&chip->adc_read_lock);
	devm_kfree(chip->dev, chip);
	return ret;
}

static int sc6607_pm_resume(struct device *dev)
{
	struct sc6607 *chip;
	struct i2c_client *client = to_i2c_client(dev);
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (!voocphy) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (!chip) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	chg_info("start\n");
	atomic_set(&chip->driver_suspended, 0);

	return 0;
}

static int sc6607_pm_suspend(struct device *dev)
{
	struct sc6607 *chip;
	struct i2c_client *client = to_i2c_client(dev);
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (!voocphy) {
		chg_err("voocphy is NULL\n");
		return -ENODEV;
	}
	chip = voocphy->priv_data;
	if (!chip) {
		chg_err("chip is NULL\n");
		return -ENODEV;
	}

	chg_info("start\n");
	atomic_set(&chip->driver_suspended, 1);

	return 0;
}

static const struct dev_pm_ops sc6607_pm_ops = {
	.resume = sc6607_pm_resume,
	.suspend = sc6607_pm_suspend,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void sc6607_voocphy_remove(struct i2c_client *client)
#else
static int sc6607_voocphy_remove(struct i2c_client *client)
#endif
{
	struct sc6607 *chip;
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);

	if (voocphy && voocphy->priv_data) {
		chip = voocphy->priv_data;
		if (chip->input_imp_node != NULL)
			oplus_imp_node_unregister(chip->dev, chip->input_imp_node);
		if (chip->output_imp_node != NULL)
			oplus_imp_node_unregister(chip->dev, chip->output_imp_node);
		if (chip->disable_votable != NULL)
			destroy_votable(chip->disable_votable);
		devm_kfree(&client->dev, voocphy);
		mutex_destroy(&chip->i2c_rw_lock);
		mutex_destroy(&chip->adc_read_lock);
		devm_kfree(chip->dev, chip);
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	return 0;
#endif
}

static void sc6607_voocphy_shutdown(struct i2c_client *client)
{
	struct oplus_voocphy_manager *voocphy = i2c_get_clientdata(client);
	struct sc6607 *chip;

	if (!voocphy || !voocphy->priv_data)
		return;

	chip = voocphy->priv_data;
	chip->shutdown_pending = true;
	chg_info("set shutdown flag, block handshake pulse\n");
	return;
}

static struct of_device_id sc6607_voocphy_match_table[] = {
	{.compatible = "oplus,sc6607-voocphy", },
	{},
};
MODULE_DEVICE_TABLE(of, sc6607_voocphy_match_table);

static const struct i2c_device_id sc6607_voocphy_device_id[] = {
	{ "sc6607,cp", 0x60 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sc6607_voocphy_device_id);

static struct i2c_driver sc6607_voocphy_driver = {
	.driver =
		{
			.name = "voocphy",
			.owner = THIS_MODULE,
			.of_match_table = sc6607_voocphy_match_table,
			.pm = &sc6607_pm_ops,
		},

	.probe = sc6607_voocphy_probe,
	.remove = sc6607_voocphy_remove,
	.shutdown = sc6607_voocphy_shutdown,
	.id_table = sc6607_voocphy_device_id,
};

int sc6607_voocphy_i2c_driver_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&sc6607_voocphy_driver) != 0)
		chg_err("failed to register sc6607 voocphy driver\n");
	else
		chg_info("success to register sc6607 voocphy driver\n");

	return ret;
}

void sc6607_voocphy_i2c_driver_exit(void)
{
	i2c_del_driver(&sc6607_voocphy_driver);
}
oplus_chg_module_register(sc6607_voocphy_i2c_driver);

MODULE_DESCRIPTION("SC6607 VOOCPHY Driver");
MODULE_LICENSE("GPL v2");
