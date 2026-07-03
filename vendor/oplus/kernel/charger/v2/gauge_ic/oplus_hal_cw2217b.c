// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[CW2217]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/proc_fs.h>
#include <linux/soc/qcom/smem.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/device_info.h>
#endif
#include <linux/version.h>
#include <linux/gfp.h>
#include <linux/pinctrl/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/sizes.h>
#ifdef OPLUS_SHA1_HMAC
#include <linux/random.h>
#endif
#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_vooc.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_monitor.h>
#include "../monitor/oplus_chg_track.h"
#include "test-kit.h"
#include "oplus_hal_cw2217b.h"

struct cw_battery *g_cw_bat = NULL;
struct gauge_track_info_reg {
	int addr;
	int len;
	int start_index;
	int end_index;
};

struct gauge_track_info_reg cw_standard[] = {
	{ REG_CHIP_ID, 2 },
	{ REG_VCELL_H , 2 },
	{ REG_SOC_INT , 2 },
	{ REG_TEMP , 2 },
	{ REG_MODE_CONFIG , 2 },
	{ REG_GPIO_CONFIG , 2 },
	{ REG_TEMP_MAX , 2 },
	{ REG_CURRENT_H , 2 },
	{ REG_T_HOST_H , 2 },
	{ REG_USER_CONF , 2 },
	{ REG_CYCLE_H , 2 },
	{ REG_SOH , 2 },
	{ REG_FW_CHECK , 2 },
	{ REG_BAT_PROFILE , 2 },
};

#ifndef CONFIG_OPLUS_CHARGER_MTK
#if IS_ENABLED(CONFIG_OPLUS_ADSP_CHARGER)
void __attribute__((weak)) oplus_vooc_get_fastchg_started_pfunc(int (*pfunc)(void))
{
	return;
}
void __attribute__((weak)) oplus_vooc_get_fastchg_ing_pfunc(int (*pfunc)(void))
{
	return;
}
#else /*IS_ENABLED(CONFIG_OPLUS_ADSP_CHARGER)*/
void __attribute__((weak)) oplus_get_gauge_chip_is_null_pfunc(bool (*pfunc)(void));
void __attribute__((weak)) oplus_vooc_get_fastchg_started_pfunc(int (*pfunc)(void));
void __attribute__((weak)) oplus_vooc_get_fastchg_ing_pfunc(int (*pfunc)(void));
#endif /*IS_ENABLED(CONFIG_OPLUS_ADSP_CHARGER)*/
#endif

/* CW2217 iic read function */
static int cw_read(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, CW_REG_BYTE, buf);
	if (ret < NUM_0)
		chg_err("IIC error %d\n", ret);

	return ret;
}

/* CW2217 iic write function */
static int cw_write(struct i2c_client *client, unsigned char reg, unsigned char const buf[])
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, CW_REG_BYTE, &buf[NUM_0]);
	if (ret < NUM_0)
		chg_err("IIC error %d\n", ret);

	return ret;
}

/* CW2217 iic read word function */
static int cw_read_word(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret;
	unsigned char reg_val[CW_REG_WORD] = { NUM_0, NUM_0 };
	unsigned int temp_val_buff;
	unsigned int temp_val_second;

	usleep_range(1000, 1000);
	ret = i2c_smbus_read_i2c_block_data(client, reg, CW_REG_WORD, reg_val);
	if (ret < NUM_0)
		chg_err("IIC error %d\n", ret);
	temp_val_buff = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];

	usleep_range(1000, 1000);
	ret = i2c_smbus_read_i2c_block_data(client, reg, CW_REG_WORD, reg_val);
	if (ret < NUM_0)
		chg_err("IIC error %d\n", ret);
	temp_val_second = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];

	if (temp_val_buff != temp_val_second) {
		usleep_range(1000, 1000);
		ret = i2c_smbus_read_i2c_block_data(client, reg, CW_REG_WORD, reg_val);
		if (ret < NUM_0)
			chg_err("IIC error %d\n", ret);
		temp_val_buff = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];
	}

	buf[NUM_0] = reg_val[NUM_0];
	buf[NUM_1] = reg_val[NUM_1];

	return ret;
}

/* CW2217 iic write profile function */
static int cw_write_profile(struct i2c_client *client, unsigned char const buf[])
{
	int ret;
	int i;

	for (i = NUM_0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_write(client, REG_BAT_PROFILE + i, &buf[i]);
		if (ret < NUM_0) {
			chg_err("IIC error %d\n", ret);
			return ret;
		}
	}

	return ret;
}

static int battery_spec_obtaining(struct cw_battery *cw_bat);
static int battery_type_check(struct cw_battery *cw_bat);
static int cw2217_parse_dt(struct cw_battery *cw_bat)
{
	struct device_node *node = oplus_get_node_by_type(cw_bat->dev->of_node);
	int rc = 0;
	int length = 0;
	char config_profile_name[128] = {0};

	cw_bat->cw_switch_config_profile = of_property_read_bool(node, "qcom,cw_switch_config_profile");
	chg_err("cw_switch_config_profile: %d\n", cw_bat->cw_switch_config_profile);

	cw_bat->ignore_battery_authenticate = of_property_read_bool(node, "oplus,ignore_battery_authenticate");
	chg_err("ignore_battery_authenticate: %d\n", cw_bat->ignore_battery_authenticate);

	rc = battery_spec_obtaining(cw_bat);
	if (rc < 0) {
		chg_err("battery_spec_obtaining failed, rc=%d\n", rc);
		return rc;
	}

	rc = battery_type_check(cw_bat);
	if (rc < 0) {
		chg_err("battery_type_check failed, rc=%d\n", rc);
		return rc;
	}

	snprintf(config_profile_name, 128, "qcom,config_profile_data%d", rc);

	rc = of_property_count_elems_of_size(node, config_profile_name, sizeof(u8));
	chg_err("%s rc=%d\n", config_profile_name, rc);
	if (rc < 0) {
		chg_err("get config_profile_data failed, rc=%d, use default profile!\n", rc);
		memmove(cw_bat->cw_config_profile, config_profile_info, SIZE_OF_PROFILE);
		return NUM_0;
	}

	length = rc;
	if (length != SIZE_OF_PROFILE) {
		chg_err("Wrong entry(%d), only %d allowed\n", length, SIZE_OF_PROFILE);
		return NUM_0;
	}

	rc = of_property_read_u8_array(node, config_profile_name,
	                                cw_bat->cw_config_profile,
	                                length);
	if (rc < 0) {
		chg_err("parse config_profile_data failed, rc=%d\n", rc);
		return rc;
	}
	return NUM_0;
}

/*
 * CW2217 Active function
 * The CONFIG register is used for the host MCU to configure the fuel gauge IC. The default value is 0xF0,
 * SLEEP and RESTART bits are set. To power up the IC, the host MCU needs to write 0x30 to exit shutdown
 * mode, and then write 0x00 to restart the gauge to enter active mode. To reset the IC, the host MCU needs
 * to write 0xF0, 0x30 and 0x00 in sequence to this register to complete the restart procedure. The CW2217B
 * will reload relevant parameters and settings and restart SOC calculation. Note that the SOC may be a
 * different value after reset operation since it is a brand-new calculation based on the latest battery status.
 * CONFIG [3:0] is reserved. Don't do any operation with it.
 */
static int cw2217_active(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val = CONFIG_MODE_RESTART;

	chg_err("\n");

	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;
	msleep(CW_SLEEP_20MS);  /* Here delay must >= 20 ms */

	reg_val = CONFIG_MODE_ACTIVE;
	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;
	msleep(CW_SLEEP_10MS);

	return NUM_0;
}

/*
 * CW2217 Sleep function
 * The CONFIG register is used for the host MCU to configure the fuel gauge IC. The default value is 0xF0,
 * SLEEP and RESTART bits are set. To power up the IC, the host MCU needs to write 0x30 to exit shutdown
 * mode, and then write 0x00 to restart the gauge to enter active mode. To reset the IC, the host MCU needs
 * to write 0xF0, 0x30 and 0x00 in sequence to this register to complete the restart procedure. The CW2217B
 * will reload relevant parameters and settings and restart SOC calculation. Note that the SOC may be a
 * different value after reset operation since it is a brand-new calculation based on the latest battery status.
 * CONFIG [3:0] is reserved. Don't do any operation with it.
 */
static int cw2217_sleep(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val = CONFIG_MODE_RESTART;

	chg_err("\n");

	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;
	msleep(CW_SLEEP_20MS);  /* Here delay must >= 20 ms */

	reg_val = CONFIG_MODE_SLEEP;
	ret = cw_write(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;
	msleep(CW_SLEEP_10MS);

	return NUM_0;
}

/*
 * The 0x00 register is an UNSIGNED 8bit read-only register. Its value is fixed to 0xA0 in shutdown
 * mode and active mode.
 */
static int cw_get_chip_id(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int chip_id;

	ret = cw_read(cw_bat->client, REG_CHIP_ID, &reg_val);
	if (ret < NUM_0)
		return ret;

	chip_id = reg_val;  /* This value must be 0xA0! */
	chg_err("chip_id = %d\n", chip_id);
	cw_bat->chip_id = chip_id;

	return NUM_0;
}

/*
 * The VCELL register(0x02 0x03) is an UNSIGNED 14bit read-only register that updates the battery voltage continuously.
 * Battery voltage is measured between the VCELL pin and VSS pin, which is the ground reference. A 14bit
 * sigma-delta A/D converter is used and the voltage resolution is 312.5uV. (0.3125mV is *5/16)
 */
static int cw_get_voltage(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[CW_REG_WORD] = {NUM_0 , NUM_0};
	unsigned int voltage;

	ret = cw_read_word(cw_bat->client, REG_VCELL_H, reg_val);
	if (ret < NUM_0)
		return ret;
	voltage = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];
	voltage = voltage  * CW_VOL_MAGIC_PART1 / CW_VOL_MAGIC_PART2;
	cw_bat->voltage = voltage;

	return NUM_0;
}

/*
 * The SOC register(0x04 0x05) is an UNSIGNED 16bit read-only register that indicates the SOC of the battery. The
 * SOC shows in % format, which means how much percent of the battery's total available capacity is
 * remaining in the battery now. The SOC can intrinsically adjust itself to cater to the change of battery status,
 * including load, temperature and aging etc.
 * The high byte(0x04) contains the SOC in 1% unit which can be directly used if this resolution is good
 * enough for the application. The low byte(0x05) provides more accurate fractional part of the SOC and its
 * LSB is (1/256) %.
 */
static int cw_get_capacity(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[CW_REG_WORD] = { NUM_0, NUM_0 };
	int ui_100 = cw_bat->cw_ui_full;
	int soc_h;
	int soc_l;
	int ui_soc;
	int remainder;

	ret = cw_read_word(cw_bat->client, REG_SOC_INT, reg_val);
	if (ret < NUM_0)
		return ret;
	soc_h = reg_val[NUM_0];
	soc_l = reg_val[NUM_1];
	ui_soc = ((soc_h * CW_SOC_MAGIC_BASE + soc_l) * CW_SOC_MAGIC_100)/ (ui_100 * CW_SOC_MAGIC_BASE);
        remainder = (soc_h * CW_SOC_MAGIC_BASE + soc_l) * CW_SOC_MAGIC_100 * CW_SOC_MAGIC_100;
	remainder = (remainder / (ui_100 * CW_SOC_MAGIC_BASE)) % CW_SOC_MAGIC_100;
	if (ui_soc >= CW_SOC_MAGIC_100) {
		chg_err("CW2015[%d]: UI_SOC = %d larger 100!!!!\n", __LINE__, ui_soc);
		ui_soc = CW_SOC_MAGIC_100;
	}
	cw_bat->ic_soc_h = soc_h;
	cw_bat->ic_soc_l = soc_l;
	cw_bat->ui_soc = ui_soc;

	return NUM_0;
}

/*
 * The TEMP register is an UNSIGNED 8bit read only register.
 * It reports the real-time battery temperature
 * measured at TS pin. The scope is from -40 to 87.5 degrees Celsius,
 * LSB is 0.5 degree Celsius. TEMP(C) = - 40 + Value(0x06 Reg) / 2
 */
static int cw_get_temp(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int temp;

	ret = cw_read(cw_bat->client, REG_TEMP, &reg_val);
	if (ret < NUM_0) {
		cw_bat->temp = -400;
		chg_err("cw_read err!, temp = -400!\n");
		return ret;
	}
	temp = (int)reg_val * CW_TEMP_MAGIC_PART1 / CW_TEMP_MAGIC_PART2 - CW_TEMP_MAGIC_PART3;
	cw_bat->temp = temp;

	return NUM_0;
}

/* get complement code function, unsigned short must be U16 */
static long get_complement_code(unsigned short raw_code)
{
	long complement_code;
	int dir;

	if (NUM_0 != (raw_code & COMPLEMENT_CODE_U16)) {
		dir = ERR_NUM;
		raw_code =  (~raw_code) + NUM_1;
	} else {
		dir = NUM_1;
	}
	complement_code = (long)raw_code * dir;

	return complement_code;
}

/*
 * CURRENT is a SIGNED 16bit register(0x0E 0x0F) that reports current A/D converter result of the voltage across the
 * current sense resistor, 10mohm typical. The result is stored as a two's complement value to show positive
 * and negative current. Voltages outside the minimum and maximum register values are reported as the
 * minimum or maximum value.
 * The register value should be divided by the sense resistance to convert to amperes. The value of the
 * sense resistor determines the resolution and the full-scale range of the current readings. The LSB of 0x0F
 * is (52.4/32768)uV.
 * The default value is 0x0000, stands for 0mA. 0x7FFF stands for the maximum charging current and 0x8001 stands for
 * the maximum discharging current.
 */
static int cw_get_current(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[CW_REG_WORD] = {NUM_0 , NUM_0};
	long cw_current;
	unsigned short current_reg;  /* unsigned short must u16 */

	ret = cw_read_word(cw_bat->client, REG_CURRENT_H, reg_val);
	if (ret < NUM_0)
		return ret;

	current_reg = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];
	cw_current = get_complement_code(current_reg);
	cw_current = cw_current  * CW_CUR_MAGIC_PART1 / cw_bat->cw_user_rsense / CW_CUR_MAGIC_PART2;
	cw_bat->cw_current = cw_current;

	return NUM_0;
}

/*
 * CYCLECNT is an UNSIGNED 16bit register(0xA4 0xA5) that counts cycle life of the battery. The LSB of 0xA5 stands
 * for 1/16 cycle. This register will be clear after enters shutdown mode
 */
static int cw_get_cycle_count(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[CW_REG_WORD] = {NUM_0, NUM_0};
	int cycle;

	ret = cw_read_word(cw_bat->client, REG_CYCLE_H, reg_val);
	if (ret < NUM_0)
		return ret;

	cycle = (reg_val[NUM_0] << CW_REG_BYTE_BITS) + reg_val[NUM_1];
	cw_bat->cycle = cycle / CW_CYCLE_MAGIC;

	return NUM_0;
}

/*
 * SOH (State of Health) is an UNSIGNED 8bit register(0xA6) that represents the level of battery aging by tracking
 * battery internal impedance increment. When the device enters active mode, this register refresh to 0x64
 * by default. Its range is 0x00 to 0x64, indicating 0 to 100%. This register will be clear after enters shutdown
 * mode.
 */
static int cw_get_soh(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int soh;

	ret = cw_read(cw_bat->client, REG_SOH, &reg_val);
	if (ret < NUM_0)
		return ret;

	soh = reg_val;
	cw_bat->soh = soh;
	return NUM_0;
}

/*
 * FW_VERSION register reports the firmware (FW) running in the chip. It is fixed to 0x00 when the chip is
 * in shutdown mode. When in active mode, Bit [7:6] are fixed to '01', which stand for the CW2217B and Bit
 * [5:0] stand for the FW version running in the chip. Note that the FW version is subject to update and contact
 * sales office for confirmation when necessary.
*/
static int cw_get_fw_version(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int fw_version;

	ret = cw_read(cw_bat->client, REG_FW_VERSION, &reg_val);
	if (ret < NUM_0)
		return ret;

	fw_version = reg_val;
	cw_bat->fw_version = fw_version;

	return NUM_0;
}

static int cw_update_data(struct cw_battery *cw_bat)
{
	int ret = NUM_0;

	ret = cw_get_voltage(cw_bat);
	if (ret < 0)
                chg_err("cw_get_voltage error, ret = %d\n", ret);

	ret = cw_get_capacity(cw_bat);
	if (ret < 0)
                chg_err("cw_get_capacity error, ret = %d\n", ret);

	ret = cw_get_temp(cw_bat);
	if (ret < 0)
                chg_err("cw_get_temp error, ret = %d\n", ret);

	ret = cw_get_current(cw_bat);
	if (ret < 0)
                chg_err("cw_get_current error, ret = %d\n", ret);

	ret = cw_get_cycle_count(cw_bat);
	if (ret < 0)
                chg_err("cw_get_cycle_count error, ret = %d\n", ret);

	ret = cw_get_soh(cw_bat);
	if (ret < 0)
                chg_err("cw_get_soh error, ret = %d\n", ret);

	chg_err("vol = %d  current = %ld cap = %d temp = %d\n",
		cw_bat->voltage, cw_bat->cw_current, cw_bat->ui_soc, cw_bat->temp);

	return ret;
}

static int cw_init_data(struct cw_battery *cw_bat)
{
	int ret = NUM_0;

	ret = cw_get_chip_id(cw_bat);
	if (ret < 0)
                chg_err("cw_get_chip_id error, ret = %d\n", ret);

	ret = cw_get_voltage(cw_bat);
	if (ret < 0)
                chg_err("cw_get_voltage error, ret = %d\n", ret);

	ret = cw_get_capacity(cw_bat);
	if (ret < 0)
                chg_err("cw_get_capacity error, ret = %d\n", ret);

	ret = cw_get_temp(cw_bat);
	if (ret < 0)
                chg_err("cw_get_temp error, ret = %d\n", ret);

	ret = cw_get_current(cw_bat);
	if (ret < 0)
                chg_err("cw_get_current error, ret = %d\n", ret);

	ret = cw_get_cycle_count(cw_bat);
	if (ret < 0)
                chg_err("cw_get_cycle_count error, ret = %d\n", ret);

	ret = cw_get_soh(cw_bat);
	if (ret < 0)
                chg_err("cw_get_soh error, ret = %d\n", ret);

	ret = cw_get_fw_version(cw_bat);
	if (ret < 0)
                chg_err("cw_get_fw_version error, ret = %d\n", ret);

	chg_err("chip_id = %d vol = %d  cur = %ld cap = %d temp = %d  fw_version = %d\n",
		cw_bat->chip_id, cw_bat->voltage, cw_bat->cw_current, cw_bat->ui_soc,
                cw_bat->temp, cw_bat->fw_version);

	return ret;
}

static int config_battery_profile_switch(struct cw_battery *cw_bat, unsigned char reg_val, int i)
{
	int ret = NUM_0, reg_index = NUM_0;

	if (cw_bat->cw_switch_config_profile) {
		ret = cw_write_profile(cw_bat->client, cw_bat->cw_config_profile);
	} else {
		ret = cw_write_profile(cw_bat->client, config_profile_info);
	}
	if (ret < NUM_0)
		return ret;

	for (i = NUM_0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_read(cw_bat->client, (REG_BAT_PROFILE + i), &reg_val);
		if (ret < NUM_0)
			return ret;
		reg_index = REG_BAT_PROFILE + i;
		if (cw_bat->cw_switch_config_profile) {
			chg_err("0x%2x, cw_config_profile[i]= 0x%2x, reg_val = 0x%2x\n",
				reg_index, cw_bat->cw_config_profile[i], reg_val);
			if (cw_bat->cw_config_profile[i] != reg_val)
				break;
		} else {
			if (config_profile_info[i] != reg_val)
				break;
		}
	}

	return i;
}

static int update_flag_and_soc_intterrupt_value(struct cw_battery *cw_bat)
{
	int ret = NUM_0, count = NUM_0;
	unsigned char reg_val;

	/* set UPDATE_FLAG AND SOC INTTERRUP VALUE*/
	reg_val = CONFIG_UPDATE_FLG | GPIO_SOC_IRQ_VALUE;
	ret = cw_write(cw_bat->client, REG_SOC_ALERT, &reg_val);
	if (ret < NUM_0)
		return ret;

	/*close all interruptes*/
	reg_val = NUM_0;
	ret = cw_write(cw_bat->client, REG_GPIO_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;

	ret = cw2217_active(cw_bat);
	if (ret < NUM_0)
		return ret;

	while (CW_TRUE) {
		msleep(CW_SLEEP_100MS);
		cw_read(cw_bat->client, REG_IC_STATE, &reg_val);
		if (IC_READY_MARK == (reg_val & IC_READY_MARK))
			break;
		count++;
		if (count >= CW_SLEEP_COUNTS) {
			cw2217_sleep(cw_bat);
			return ERR_NUM;
		}
	}

        return ret;
}

#define CW_INIT_RETRY_MAX 3
#define PUSH_DELAY_MS 12000
/*CW2217 update profile function, often called during initialization*/
static int cw_config_start_ic(struct cw_battery *cw_bat, int init_type)
{
	int ret, reg_index = 0, i = NUM_0, retry_times = 0;
	int index = 0;
	unsigned char reg_val = REG_CHIP_ID;
	unsigned char reg_data[CW_REG_WORD] = {NUM_0 , NUM_0};

	ret = cw2217_sleep(cw_bat);
	if (ret < NUM_0)
		return ret;

CW_EXECUTE_CMD_RETRY:
	/* update new battery info */
        ret = config_battery_profile_switch(cw_bat, reg_val, i);
	if (ret < 0) {
		chg_err("config_battery_profile_switch failed, ret=%d\n", ret);
		return ret;
	}

	if (cw_bat->debug_force_cw_err || ret != SIZE_OF_PROFILE) {
		retry_times++;
		chg_err("Failed at [%d, %d]\n", reg_val, retry_times);
		if (retry_times < CW_INIT_RETRY_MAX) {
			goto CW_EXECUTE_CMD_RETRY;
		}
		cw_bat->debug_force_cw_err = false;
	}
	memset(cw_bat->track_info, 0, sizeof(cw_bat->track_info));
	if (i >= SIZE_OF_PROFILE)
		i = SIZE_OF_PROFILE - 1;
	index = scnprintf(cw_bat->track_info, CW_INFO_LEN, "$$init_type@@%d$$index@@0x%x$$profile@@0x%x$$reg_val@@0x%x"
			"$$retry_times@@%d", init_type, reg_index, cw_bat->cw_config_profile[i], reg_val, retry_times);
	for (i = 0; i < ARRAY_SIZE(cw_standard); i++) {
		ret = cw_read_word(cw_bat->client, cw_standard[i].addr, reg_data);
		if (ret < NUM_0)
			continue;
		index += scnprintf(cw_bat->track_info + index, CW_INFO_LEN - index,
			  "0x%02x=%02x,%02x|", cw_standard[i].addr, reg_data[NUM_0], reg_data[NUM_1]);
	}

	schedule_delayed_work(&cw_bat->cw_track_update_work, msecs_to_jiffies(PUSH_DELAY_MS));

        ret = update_flag_and_soc_intterrupt_value(cw_bat);
	if (ret < 0) {
		chg_err("update_flag_and_soc_intterrupt_value failed, ret=%d\n", ret);
		return ret;
	}

	return NUM_0;
}

static int cw2217_battery_profile_init(struct cw_battery *cw_bat)
{
	int ret = NUM_0, reg_profile = NUM_0;
	unsigned char reg_val;
	int i;

	for (i = NUM_0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_read(cw_bat->client, (REG_BAT_PROFILE + i), &reg_val);
		if (ret < NUM_0)
			return ret;
		reg_profile = REG_BAT_PROFILE + i;
		chg_err("0x%2x = 0x%2x\n", reg_profile, reg_val);
		if (cw_bat->cw_switch_config_profile) {
			if (cw_bat->cw_config_profile[i] != reg_val)
				break;
		} else {
			if (config_profile_info[i] != reg_val)
				break;
		}
	}

	return i;
}

/*
 * Get the cw2217 running state
 * Determine whether the profile needs to be updated
*/
static int cw2217_get_state(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;

	ret = cw_read(cw_bat->client, REG_MODE_CONFIG, &reg_val);
	if (ret < NUM_0)
		return ret;
	if (reg_val != CONFIG_MODE_ACTIVE)
		return CW2217_NOT_ACTIVE;

	ret = cw_read(cw_bat->client, REG_SOC_ALERT, &reg_val);
	if (ret < NUM_0)
		return ret;
	if (NUM_0 == (reg_val & CONFIG_UPDATE_FLG))
		return CW2217_PROFILE_NOT_READY;

	if (cw2217_battery_profile_init(cw_bat) != SIZE_OF_PROFILE)
		return CW2217_PROFILE_NEED_UPDATE;

	return NUM_0;
}

/*CW2217 init function, Often called during initialization*/
static int cw_init(struct cw_battery *cw_bat)
{
	int ret;

	chg_err("\n");
	ret = cw_get_chip_id(cw_bat);
	if (ret < NUM_0) {
		chg_err("iic read write error");
		return ret;
	}
	if (cw_bat->chip_id != IC_VCHIP_ID) {
		chg_err("not cw2217B\n");
		return ERR_NUM;
	}

	ret = cw2217_get_state(cw_bat);
	if (ret < NUM_0) {
		chg_err("iic read write error");
		return ret;
	}

	if (ret != NUM_0) {
		ret = cw_config_start_ic(cw_bat, ret);
		if (ret < NUM_0)
			return ret;
	}
	chg_err("cw2217 init success!\n");
	strncpy(cw_bat->device_name, "cw2217b", CW_NAME_LEN);
	cw_bat->device_type = cw_bat->chip_id;

	return NUM_0;
}

static void cw_bat_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct cw_battery *cw_bat;
	int ret;

	delay_work = container_of(work, struct delayed_work, work);
	cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

	ret = cw_update_data(cw_bat);
	if (ret < NUM_0)
		chg_err("iic read error when update data");

	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(QUEUE_DELAYED_WORK_TIME));
}

#define SOH_INIT_VALUE  100
static int cw2217_get_battery_mvolts(void)
{
        int ret = NUM_0;

        ret = cw_get_voltage(g_cw_bat);
        return g_cw_bat->voltage;
}

static int cw2217_get_battery_fc(void)
{
	cw_get_soh(g_cw_bat);
	g_cw_bat->fcc = (g_cw_bat->soh * g_cw_bat->design_capacity) / SOH_INIT_VALUE;
        return g_cw_bat->fcc;
}

static int cw2217_get_battery_cc(void)
{
	cw_get_cycle_count(g_cw_bat);
	return g_cw_bat->cycle;
}

static int cw2217_get_battery_temperature(void)
{
        int ret = NUM_0;

        ret = cw_get_temp(g_cw_bat);
        return g_cw_bat->temp;
}

static int cw2217_get_batt_remaining_capacity(void)
{
	return g_cw_bat->ui_soc * g_cw_bat->rated_capacity / g_cw_bat->cw_ui_full;
}

static int cw2217_get_battery_soc(void)
{
        int soc = NUM_0;
        cw_get_capacity(g_cw_bat);
		soc = g_cw_bat->ui_soc;
        return soc;
}

static int cw2217_get_average_current(void)
{
        cw_get_current(g_cw_bat);
	return -g_cw_bat->cw_current;
}

static bool cw2217_get_battery_hmac(void)
{
        return true;
}

static void cw2217_void_dumy(bool full)
{
        /* Do nothing */
}

#define NTC_DEFAULT_VOLT_VALUE_MV 950
#define BATID_LOW_MV1 200
#define BATID_HIGH_MV1 320
#define BATID_LOW_MV2 719
#define BATID_HIGH_MV2 820
#define THERMAL_TEMP_UNIT      1000
static int oplus_get_iio_channel(struct cw_battery *cw_bat, const char *propname,
                        struct iio_channel **chan)
{
        int rc = NUM_0;

        rc = of_property_match_string(cw_bat->dev->of_node,
                        "io-channel-names", propname);
        if (rc < NUM_0)
                return rc;

        *chan = iio_channel_get(cw_bat->dev, propname);
        if (IS_ERR(*chan)) {
                rc = PTR_ERR(*chan);
        if (rc != -EPROBE_DEFER)
                chg_err(" %s channel unavailable, %d\n", propname, rc);
        *chan = NULL;
	}

	return rc;
}

static int battery_spec_obtaining(struct cw_battery *cw_bat)
{
	int ret = 0;
	struct device_node *node = oplus_get_node_by_type(cw_bat->dev->of_node);

	ret = of_property_read_u32(node, "oplus,batt_num", &cw_bat->batt_num);
	if (ret < 0) {
		chg_err("can't get oplus,batt_num, ret=%d\n", ret);
		cw_bat->batt_num = 1;
	}

	ret = of_property_read_u32(node, "qcom,cw_ui_full", &cw_bat->cw_ui_full);
	if (ret) {
		cw_bat->cw_ui_full = 100;
		chg_err("cw_ui_full: %d\n", cw_bat->cw_ui_full);
	} else {
		chg_err("cw_ui_full: %d\n", cw_bat->cw_ui_full);
	}

	ret = of_property_read_u32(node, "qcom,cw_user_rsense", &cw_bat->cw_user_rsense);
	if (ret) {
		cw_bat->cw_user_rsense = 2000;
	}
	chg_err("cw_user_rsense: %d\n", cw_bat->cw_user_rsense);

	ret = of_property_read_u32(node, "qcom,design_capacity", &cw_bat->design_capacity);
	if (ret) {
		cw_bat->design_capacity = 5000;
	}
	chg_err("design_capacity: %d\n", cw_bat->design_capacity);

	ret = of_property_read_u32(node, "qcom,rated_capacity", &cw_bat->rated_capacity);
	if (ret) {
		cw_bat->rated_capacity = 4880;
	}
	chg_err("rated_capacity: %d\n", cw_bat->rated_capacity);

        return ret;
}

static int get_batt_id_chan_value(struct cw_battery *cw_bat)
{
        int ret = NUM_0, value = NUM_0;

	if (!cw_bat->batt_id_chan) {
		ret = oplus_get_iio_channel(cw_bat, "batt_id_chan", &cw_bat->batt_id_chan);
		if (ret < 0 && !cw_bat->batt_id_chan) {
		    chg_err(" %s usb_temp1 get failed\n", __func__);
			return ret;
		}
	}

	ret = iio_read_channel_processed(cw_bat->batt_id_chan, &value);
	if (ret < 0) {
		chg_err("fail to read usb_temp1 adc rc = %d\n", ret);
		return ret;
	}

        return value;
}

static int battery_type_check(struct cw_battery *cw_bat)
{
	int battery_type = BAT_TYPE_UNKNOWN;
	int ret = 0;
	int value = 0;
	int length = 0, i = 0;
	struct device_node *node = oplus_get_node_by_type(cw_bat->dev->of_node);

	if (cw_bat->batid_voltage_range[0][0] == 0) {
		length = of_property_count_elems_of_size(node, "batid_voltage_range", sizeof(u32));
		chg_err("batid_voltage_range ret=%d\n", length);
		if (length < 0) {
			chg_err("Count batid_voltage_range failed, rc=%d\n", length);
			return battery_type;
		}
		ret = of_property_read_u32_array(node, "batid_voltage_range",
								&cw_bat->batid_voltage_range[0][0],
								length);
	}

        value = get_batt_id_chan_value(cw_bat);
	if (value <= 0) {
		chg_err("[OPLUS_CHG][%s]: iio_read_channel_processed  get error\n", __func__);
		value = NTC_DEFAULT_VOLT_VALUE_MV;
		return battery_type;
	}
	value = value / THERMAL_TEMP_UNIT;
	for (i = 0; i < BATTID_ARR_LEN; i++) {
		if (value >= cw_bat->batid_voltage_range[i][0] && value <= cw_bat->batid_voltage_range[i][1]) {
			battery_type = i+1;
			break;
		}
	}
	chg_err(" battery_id = %d(%d)\n", battery_type, value);
	return battery_type;
}

bool cw2217_get_battery_authenticate(void)
{
        int bat_type = BAT_TYPE_UNKNOWN;
		int bat_temp = 0;
		if (!g_cw_bat) {
			chg_err("cw2217_get_battery_authenticate g_cw_bat is NULL!");
			return false;
		}
        bat_type = battery_type_check(g_cw_bat);
		bat_temp = cw2217_get_battery_temperature();
		chg_err("bat_type = %d, bat_temp = %d\n", bat_type, bat_temp);
        if ((bat_type <= BAT_TYPE_UNKNOWN || bat_temp <= -400) && !g_cw_bat->ignore_battery_authenticate) {
			return false;
        } else {
			return true;
        }
}

int cw2217_get_battery_soh(void)
{
	cw_get_soh(g_cw_bat);
	return g_cw_bat->soh;
}

int cw2217_get_battery_mvolts_2cell_max(void)
{
	return cw2217_get_battery_mvolts();
}

int cw2217_get_battery_mvolts_2cell_min(void)
{
	return cw2217_get_battery_mvolts();
}

#define PREV_VBAT 3800

int cw2217_prev_battery_mvolts_2cell_max(void)
{
	return PREV_VBAT;
}

int cw2217_prev_battery_mvolts_2cell_min(void)
{
	return PREV_VBAT;
}

static int cw2217_get_qmax(int *qmax1, int *qmax2)
{
	if (!qmax1 || !qmax2 || !g_cw_bat)
		return -1;

	*qmax1 = g_cw_bat->design_capacity;
	*qmax2 = g_cw_bat->design_capacity;
	return 0;
}

static int cw2217_get_info(u8 *info, int len)
{
	int i;
	int ret;
	unsigned char reg_val[CW_REG_WORD] = {NUM_0 , NUM_0};
	int index = 0;

	if (!g_cw_bat || !info || !len)
		return -1;

	for (i = 0; i < ARRAY_SIZE(cw_standard); i++) {
		ret = cw_read_word(g_cw_bat->client, cw_standard[i].addr, reg_val);
		if (ret < NUM_0)
			continue;
		index += snprintf(info + index, len - index,
			  "0x%02x=%02x,%02x|", cw_standard[i].addr, reg_val[NUM_0], reg_val[NUM_1]);
	}
	return 0;
}

static ssize_t
cw2217_show_rfg(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	int idx = snprintf(buf, PAGE_SIZE, "rfg = %d\n", g_cw_bat->cw_user_rsense);
	return idx;
}

static ssize_t
cw2217_store_rfg(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	int ret;
	unsigned int rfg;

	ret = sscanf(buf, "%d", &rfg);
	g_cw_bat->cw_user_rsense = rfg;

	return count;
}


static DEVICE_ATTR(rfg, S_IRUGO | S_IWUSR, cw2217_show_rfg,
		   cw2217_store_rfg);

static struct attribute *cw2217_attributes[] = {
	&dev_attr_rfg.attr,
	NULL,
};

static const struct attribute_group cw2217_attr_group = {
	.attrs = cw2217_attributes,
};

#define TRACK_LOCAL_T_NS_TO_S_THD 1000000000
#define TRACK_UPLOAD_COUNT_MAX 10
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD (24 * 3600)
static int cw2217b_track_get_local_time_s(void)
{
	int local_time_s;

	local_time_s = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	pr_info("local_time_s:%d\n", local_time_s);

	return local_time_s;
}

int cw2217b_track_upload_upgrade_info(struct cw_battery *chip, char *cw_msg)
{
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;

	mutex_lock(&chip->track_upload_lock);
	curr_time = cw2217b_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_UPLOAD_COUNT_MAX) {
		mutex_unlock(&chip->track_upload_lock);
		return 0;
	}
	chg_err(" cw_msg = %s\n", cw_msg);

	mutex_lock(&chip->track_cw_err_lock);
	if (chip->cw_err_uploading) {
		pr_info("cw_err_uploading, should return\n");
		mutex_unlock(&chip->track_cw_err_lock);
		mutex_unlock(&chip->track_upload_lock);
		return 0;
	}
	chip->cw_err_uploading = true;
	upload_count++;
	pre_upload_time = cw2217b_track_get_local_time_s();
	mutex_unlock(&chip->track_cw_err_lock);

	oplus_chg_ic_creat_err_msg(chip->ic_dev, OPLUS_IC_ERR_GAUGE, 0,
		"$$cw_msg@@%s $$err_scene@@%s", cw_msg, "cw_err");
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);

	chip->cw_err_uploading = false;
	mutex_unlock(&chip->track_upload_lock);
	chg_err("success\n");

	return 0;
}

static int cw2217b_track_debugfs_init(struct cw_battery *chip)
{
	int ret = 0;
	struct dentry *debugfs_root;
	struct dentry *debugfs_cw_ic;

	debugfs_root = oplus_chg_track_get_debugfs_root();
	if (!debugfs_root) {
		ret = -ENOENT;
		return ret;
	}

	debugfs_cw_ic = debugfs_create_dir("cw_track", debugfs_root);
	if (!debugfs_cw_ic) {
		ret = -ENOENT;
		return ret;
	}

	chip->debug_force_cw_err = false;
	debugfs_create_u32("debug_force_cw_err", 0644, debugfs_cw_ic, &(chip->debug_force_cw_err));

	return ret;
}

static void oplus_cw2217b_track_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct cw_battery *chip = container_of(dwork, struct cw_battery, cw_track_update_work);

	cw2217b_track_upload_upgrade_info(chip, chip->track_info);
}

static int oplus_cw2217b_track_init(struct cw_battery *chip)
{
	int rc;

	if (!chip)
		return -EINVAL;

	mutex_init(&chip->track_cw_err_lock);
	mutex_init(&chip->track_upload_lock);

	chip->cw_err_uploading = false;

	rc = cw2217b_track_debugfs_init(chip);
	if (rc < 0) {
		chg_err("cw track debugfs init error, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int oplus_cw2217b_init(struct oplus_chg_ic_dev *ic_dev)
{
	ic_dev->online = true;

	return 0;
}

static int oplus_cw2217b_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;

	return 0;
}

static int oplus_cw2217b_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	if (g_cw_bat == NULL) {
		chg_err("g_cw_bat is NULL\n");
		return -ENODEV;
	}

	return cw2217_get_state(g_cw_bat);
}

static int oplus_cw2217b_get_batt_vol(struct oplus_chg_ic_dev *ic_dev, int index, int *vol_mv)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

        *vol_mv = cw2217_get_battery_mvolts();

	return 0;
}

static int oplus_cw2217b_get_batt_max(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	*vol_mv = cw2217_get_battery_mvolts();

	return 0;
}

static int oplus_cw2217b_get_batt_min(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	*vol_mv = cw2217_get_battery_mvolts();

	return 0;
}

static int oplus_cw2217b_get_batt_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	*curr_ma = cw2217_get_average_current();

	return 0;
}

static int oplus_cw2217b_get_batt_temp(struct oplus_chg_ic_dev *ic_dev, int *temp)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

        if (cw2217_get_battery_soc() < NUM_0)
		return ERR_NUM;

	*temp = cw2217_get_battery_temperature();

	return 0;
}

static int oplus_cw2217b_get_batt_soc(struct oplus_chg_ic_dev *ic_dev, int *soc)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	*soc = cw2217_get_battery_soc();
	if (*soc < NUM_0) {
		return ERR_NUM;
	}

	return 0;
}

static int oplus_cw2217b_get_batt_fcc(struct oplus_chg_ic_dev *ic_dev, int *fcc)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	*fcc = cw2217_get_battery_fc();

	return 0;
}

static int oplus_cw2217b_get_batt_cc(struct oplus_chg_ic_dev *ic_dev, int *cc)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	*cc = cw2217_get_battery_cc();

	return 0;
}

static int oplus_cw2217b_get_batt_rm(struct oplus_chg_ic_dev *ic_dev, int *rm)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	*rm = cw2217_get_batt_remaining_capacity();

	return 0;
}

static int oplus_cw2217b_get_batt_soh(struct oplus_chg_ic_dev *ic_dev, int *soh)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	*soh = cw2217_get_battery_soh();

	return 0;
}

static int oplus_cw2217b_get_batt_auth(struct oplus_chg_ic_dev *ic_dev, bool *pass)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	*pass = cw2217_get_battery_authenticate();
	chg_info("*pass = %d\n", *pass);

	return 0;
}

static int oplus_cw2217b_get_batt_hmac(struct oplus_chg_ic_dev *ic_dev, bool *pass)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	*pass = cw2217_get_battery_hmac();
	chg_info("*pass = %d\n", *pass);

	return 0;
}

static int oplus_cw2217b_set_batt_full(struct oplus_chg_ic_dev *ic_dev, bool full)
{
	cw2217_void_dumy(full);
	chg_info("full = %d\n", full);

	return 0;
}

static int oplus_cw2217b_get_batt_num(struct oplus_chg_ic_dev *ic_dev, int *num)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	if (g_cw_bat == NULL) {
		chg_err("g_cw_bat is NULL\n");
		return -ENODEV;
	}

	*num = g_cw_bat->batt_num;

	return 0;
}

static int oplus_cw2217b_get_gauge_type(struct oplus_chg_ic_dev *ic_dev, int *gauge_type)
{
	if (ic_dev == NULL || gauge_type == NULL) {
		chg_err("oplus_chg_ic_dev or gauge_type is NULL");
		return -ENODEV;
	}

	*gauge_type = GAUGE_TYPE_BOARD;

	return 0;
}

static int oplus_cw2217b_get_device_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	if (g_cw_bat == NULL) {
		chg_err("g_cw_bat is NULL\n");
		return -ENODEV;
	}

	rc = cw_get_chip_id(g_cw_bat);
	if (rc < NUM_0) {
		chg_err("cw2217 get chip_id error\n");
		return rc;
	}

	*type = g_cw_bat->chip_id;

	return 0;
}

static int
oplus_cw2217b_get_device_type_for_vooc(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	*type = NUM_0;

	return 0;
}

static int oplus_cw2217b_get_battery_qmax(struct oplus_chg_ic_dev *ic_dev, int batt_id, int *qmax)
{
	int rc;

	if (ic_dev == NULL || qmax == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	/*single cell battery, qmax1 == qmax2*/
	rc = cw2217_get_qmax(qmax, qmax);
	if (rc < 0) {
		chg_err("failed to get qmax from cw2217\n");
		*qmax = 0;
		return -EINVAL;
	}

	return 0;
}

static int oplus_cw2217b_get_battery_gauge_type_for_bcc(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	*type = DEVICE_CW2217B;

	return 0;
}

static int oplus_cw2217b_get_batt_exist(struct oplus_chg_ic_dev *ic_dev, bool *exist)
{
	*exist = true;

	return 0;
}

static int oplus_cw2217_get_afi_update_done(struct oplus_chg_ic_dev *ic_dev, bool *status)
{
	*status = true;

	return 0;
}

static int oplus_cw2217b_get_reg_info(struct oplus_chg_ic_dev *ic_dev, u8 *info, int len)
{
	if (ic_dev == NULL || !info) {
		chg_err("oplus_chg_ic_dev or info is NULL");
		return -ENODEV;
	}

        cw2217_get_info(info, len);

	return 0;
}

static int oplus_cw2217_get_dec_fg_type(struct oplus_chg_ic_dev *ic_dev, int *fg_type)
{
	if (!ic_dev || !fg_type) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*fg_type = MB_CW;

	return 0;
}

static int oplus_cw2217_get_dec_cv_soh(struct oplus_chg_ic_dev *ic_dev, int *dec_soh)
{
	if (!ic_dev || !dec_soh) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*dec_soh = cw2217_get_battery_cc();

	return 0;
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
                                                oplus_cw2217b_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
                                                oplus_cw2217b_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
                                                oplus_cw2217b_reg_dump);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL,
                                                oplus_cw2217b_get_batt_vol);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX,
                                                oplus_cw2217b_get_batt_max);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN,
                                                oplus_cw2217b_get_batt_min);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR,
						oplus_cw2217b_get_batt_curr);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP,
						oplus_cw2217b_get_batt_temp);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC,
                                                oplus_cw2217b_get_batt_soc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC,
                                                oplus_cw2217b_get_batt_fcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_CC,
                                                oplus_cw2217b_get_batt_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_RM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_RM,
                                                oplus_cw2217b_get_batt_rm);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH,
                                                oplus_cw2217b_get_batt_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH,
						oplus_cw2217b_get_batt_auth);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC,
						oplus_cw2217b_get_batt_hmac);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL,
						oplus_cw2217b_set_batt_full);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM,
                                                oplus_cw2217b_get_batt_num);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_GAUGE_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_GAUGE_TYPE,
					       oplus_cw2217b_get_gauge_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE,
                                                oplus_cw2217b_get_device_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_VOOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_VOOC,
                                                oplus_cw2217b_get_device_type_for_vooc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_AFI_UPDATE_DONE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_AFI_UPDATE_DONE,
                                                oplus_cw2217_get_afi_update_done);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEVICE_TYPE_FOR_BCC,
                                                oplus_cw2217b_get_battery_gauge_type_for_bcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_QMAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_QMAX,
                                                oplus_cw2217b_get_battery_qmax);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_REG_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_REG_INFO,
                                                oplus_cw2217b_get_reg_info);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST,
                                                oplus_cw2217b_get_batt_exist);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEC_FG_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEC_FG_TYPE,
		                                oplus_cw2217_get_dec_fg_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH,
		                                oplus_cw2217_get_dec_cv_soh);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq cw2217b_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{ .virq_id = OPLUS_IC_VIRQ_RESUME },
};

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
static struct regmap_config cw2217b_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
};
#endif /* CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT */

static int gauge_ic_cfg_register(struct i2c_client *client, struct cw_battery *cw_bat)
{
	int ret = NUM_0;
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };

	ret = of_property_read_u32(cw_bat->dev->of_node, "oplus,ic_type", &ic_type);
	if (ret < 0) {
		chg_err("can't get ic type, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(cw_bat->dev->of_node, "oplus,ic_index", &ic_index);
	if (ret < 0) {
		chg_err("can't get ic index, ret=%d\n", ret);
		return ret;
	}

	ic_cfg.name = cw_bat->dev->of_node->name;
	ic_cfg.index = ic_index;

	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "gauge-cw2217b:%d", ic_index);
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	cw_bat->odb = devm_oplus_device_bus_register(cw_bat->dev, &cw2217b_regmap_config, ic_cfg.manu_name);
	if (IS_ERR_OR_NULL(cw_bat->odb)) {
		chg_err("register odb error\n");
		return -EFAULT;
	}
#endif /* CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT */

	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = cw2217b_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(cw2217b_virq_table);
	ic_cfg.of_node = cw_bat->dev->of_node;
	cw_bat->ic_dev = devm_oplus_chg_ic_register(cw_bat->dev, &ic_cfg);
	if (!cw_bat->ic_dev) {
		return -ENODEV;
		chg_err("register %s error\n", cw_bat->dev->of_node->name);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
		devm_oplus_device_bus_unregister(cw_bat->odb);
#endif
	}
	chg_info("register %s\n", cw_bat->dev->of_node->name);

	return ret;
}

static int cw2217_device_init(struct cw_battery *cw_bat)
{
	int ret = NUM_0, loop = NUM_0;

	ret = cw2217_parse_dt(cw_bat);
	if (ret) {
		chg_err("%s : cw2217_parse_dt fail!\n", __func__);
		return ret;
	}

	INIT_DELAYED_WORK(&cw_bat->cw_track_update_work, oplus_cw2217b_track_update_work);
	ret = cw_init(cw_bat);
	while ((loop++ < CW_RETRY_COUNT) && (ret != 0)) {
		msleep(CW_SLEEP_200MS);
		ret = cw_init(cw_bat);
	}
	if (ret) {
		chg_err("%s : cw2217 init fail!\n", __func__);
		return ret;
	}

	ret = cw_init_data(cw_bat);
	if (ret) {
		chg_err("%s : cw2217 init data fail!\n", __func__);
		return ret;
	}

	cw_bat->cwfg_workqueue = create_singlethread_workqueue("cwfg_gauge");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work , msecs_to_jiffies(QUEUE_START_WORK_TIME));

	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int cw2217_probe(struct i2c_client *client)
#else
static int cw2217_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	int ret;
	struct cw_battery *cw_bat;

	chg_err("%s start\n", __func__);

	cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
	if (!cw_bat) {
		chg_err("%s : cw_bat create fail!\n", __func__);
		return -ENOMEM;
	}
	cw_bat->dev = &client->dev;
	i2c_set_clientdata(client, cw_bat);
	cw_bat->client = client;
        g_cw_bat = cw_bat;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	cw_bat->regmap = devm_regmap_init_i2c(client, &cw2217b_regmap_config);
	if (!cw_bat->regmap) {
		ret = -ENODEV;
		goto regmap_init_err;
	}
#endif /* CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT */

	ret = cw2217_device_init(cw_bat);
	if (ret) {
		chg_err("%s : cw2217_device_init fail!\n", __func__);
		return ret;
	}
        chg_err("cw2217 driver probe success!\n");

	ret = gauge_ic_cfg_register(client, cw_bat);
	if (ret < 0)
		goto error;

#ifndef CONFIG_OPLUS_CHARGER_MTK
	oplus_get_gauge_chip_is_null_pfunc(&oplus_gauge_check_chip_is_null);
	oplus_vooc_get_fastchg_started_pfunc(&oplus_vooc_get_fastchg_started);
	oplus_vooc_get_fastchg_ing_pfunc(&oplus_vooc_get_fastchg_ing);
#endif

        ret = sysfs_create_group(&cw_bat->dev->kobj, &cw2217_attr_group);
        oplus_cw2217b_track_init(cw_bat);
        oplus_cw2217b_init(cw_bat->ic_dev);
	chg_err("%s end\n", __func__);
	return NUM_0;

error:
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
regmap_init_err:
#endif
	devm_kfree(&client->dev, cw_bat);
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void cw2217_remove(struct i2c_client *client)
#else
static int cw2217_remove(struct i2c_client *client)
#endif
{
	chg_err("\n");
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	return NUM_0;
#endif
}

#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	cancel_delayed_work(&cw_bat->battery_delay_work);
	return NUM_0;
}

static int cw_bat_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(20));
        oplus_chg_ic_virq_trigger(cw_bat->ic_dev, OPLUS_IC_VIRQ_RESUME);
	return NUM_0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
	.suspend  = cw_bat_suspend,
	.resume   = cw_bat_resume,
};
#endif

static const struct i2c_device_id cw2217_id_table[] = {
	{ CWFG_NAME, NUM_0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cw2217_id_table);

static struct of_device_id cw2217_match_table[] = {
	{ .compatible = "cellwise,cw2217", },
	{ },
};

static struct i2c_driver cw2217_driver = {
	.driver   = {
		.name = CWFG_NAME,
#ifdef CONFIG_PM
		.pm = &cw_bat_pm_ops,
#endif
		.owner = THIS_MODULE,
		.of_match_table = cw2217_match_table,
	},
	.probe = cw2217_probe,
	.remove = cw2217_remove,
	.id_table = cw2217_id_table,
};

static __init int oplus_cw2217_driver_init(void)
{
	return i2c_add_driver(&cw2217_driver);
}

static __exit void oplus_cw2217_driver_exit(void)
{
	i2c_del_driver(&cw2217_driver);
}

oplus_chg_module_register(oplus_cw2217_driver);

MODULE_DESCRIPTION("CW2217 FGADC Device Driver V1.2");
MODULE_LICENSE("GPL v2");
