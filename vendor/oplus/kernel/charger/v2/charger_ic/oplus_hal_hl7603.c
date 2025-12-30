// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[HL7603]: %s[%d]: " fmt, __func__, __LINE__

#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/ktime.h>
#include <linux/sched/clock.h>
#include <linux/pinctrl/consumer.h>

#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_chg.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_monitor.h>

#include "test-kit.h"

static int hl7603_debug_track = 0;
module_param(hl7603_debug_track, int, 0644);
MODULE_PARM_DESC(hl7603_debug_track, "debug track");

#define DEV_ID_REV_REG			0x0
#define E2PROMCTRL_REG			0xFF

#define HL7603_DEVICE_ID_REG		0x0
#define HL7603_CONFIG_1_REG		0x01
#define HL7603_FPWM_CFG_MASK		0x1
#define HL7603_FPWM_CFG_FORCE_PWM	0x1
#define HL7603_FPWM_CFG_AUTO_PFM	0x0
#define HL7603_FPWM_CFG_AUTO_PFM_REG   0x68

#define HL7603_VOUT_SEL_REG		0x02
#define HL7603_ILIM_SET_REG		0x03
#define HL7603_CONFIG_2_REG		0x04
#define HL7603_STATUS_REG		0x05
#define HL7603_REG_CNT			6

#define HL7603_ILIM_MA_MIN		4000
#define HL7603_ILIM_MA_MAX		8500
#define HL7603_ILIM_MA_STEP		500
#define HL7603_ILIM_REG_MIN		0x00
#define HL7603_ILIM_REG_MAX		0x09

#define VOUT_MV_MIN			2850
#define VOUT_MV_MAX			4400
#define VOUT_MV_STEP			50
#define VOUT_REG_MIN			0x00
#define VOUT_REG_MAX			0x1F

#define ILIM_MA_MIN			1500
#define ILIM_MA_MAX			5000
#define ILIM_MA_STEP			500
#define ILIM_REG_MIN			0x18
#define ILIM_REG_MAX			0x1F

#define DEFAULT_VOUT_MV			3000
#define HIGH_VOUT_MV			4400

#define DEFAULT_VOUTFLOORSET_VAL	0x03
#define DEFAULT_ILIMSET_VAL		0x1F

#define STATUS_MASK			0x12
#define BOOST_STATUS_NORMAL		0x10
#define BYPASS_STATUS_NORMAL		0x00

#define HL7603X_REG_TEST_MODE               0xA7
#define HL7603X_REG_TEST_MODE_OPEN          0xF9 // open success read 0x01
#define HL7603X_REG_TEST_MODE_CLOSE         0x9F // close success read 0x00
#define HL7603X_REG_TEST_MODE_CLOSE_STATUS  0x00
#define HL7603X_REG_BOOST_CONTROL           0x22
#define HL7603X_REG_BOOST_CONTROL_VAL       0x00
#define HL7603X_BOOST_CONTROL_BIT6_MASK     0x40  /* bit6 mask */

enum {
	BYB_STATUS_FAULT = 0,
	BYB_STATUS_BOOST,
	BYB_STATUS_BYPASS,
};

enum rst_type {
	FPGA_RST = 1,
	I2C_RST,
};

static const char *byb_status_name[] = {
	[BYB_STATUS_FAULT] = "fault",
	[BYB_STATUS_BOOST] = "boost",
	[BYB_STATUS_BYPASS] = "bypass",
};

static const char *gpio_status_name[] = {
	[GPIO_STATUS_NC] = "not connect",
	[GPIO_STATUS_PD] = "pull down",
	[GPIO_STATUS_PU] = "pull up",
	[GPIO_STATUS_NOT_SUPPORT] = "not support",
};

struct chip_hl7603 {
	struct device *dev;
	struct i2c_client *client;
	struct oplus_chg_ic_dev *ic_dev;
	struct regmap *regmap;
	struct mutex pinctrl_lock;
	struct pinctrl *pinctrl;
	struct pinctrl_state *id_not_pull;
	struct pinctrl_state *id_pull_up;
	struct pinctrl_state *id_pull_down;
	int id_gpio;
	int id_match_status;
	int vout_mv;
	bool i2c_success;
	int probe_gpio_status;
	int ilim_ma;
	bool fpga_support;
	struct delayed_work retry_init_work;

	atomic_t suspended;
#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
	struct test_feature *boost_id_gpio_test;
	struct test_feature *fpga_boost_test;
#endif
	unsigned long rst_ing;
	int chip_id;
};

static bool hl7603_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case HL7603_CONFIG_1_REG:
	case HL7603_VOUT_SEL_REG:
	case HL7603_ILIM_SET_REG:
	case HL7603_CONFIG_2_REG:
	case E2PROMCTRL_REG:
	case HL7603X_REG_BOOST_CONTROL:
	case HL7603X_REG_TEST_MODE:
		return true;
	default:
		return false;
	}
}

static bool hl7603_is_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DEV_ID_REV_REG:
	case HL7603_CONFIG_1_REG:
	case HL7603_VOUT_SEL_REG:
	case HL7603_ILIM_SET_REG:
	case HL7603_CONFIG_2_REG:
	case HL7603_STATUS_REG:
	case E2PROMCTRL_REG:
	case HL7603X_REG_BOOST_CONTROL:
	case HL7603X_REG_TEST_MODE:
		return true;
	default:
		return false;
	}
}

static struct regmap_config hl7603_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = hl7603_is_writeable_reg,
	.readable_reg = hl7603_is_readable_reg,
	.max_register = E2PROMCTRL_REG,
};

static int hl7603_read(struct chip_hl7603 *chip, unsigned int reg, unsigned int *val)
{
	int rc = 0;

	if (chip->fpga_support && chip->rst_ing)
		return -1;

	rc = regmap_read(chip->regmap, reg, val);
	if (rc < 0)
		chg_err("read 0x%x fail, rc=%d\n", reg, rc);

	return rc;
}

static int hl7603_write(struct chip_hl7603 *chip, unsigned int reg, unsigned int val)
{
	int rc = 0;

	if (chip->fpga_support && chip->rst_ing)
		return -1;

	rc = regmap_write(chip->regmap, reg, val);
	if (rc < 0)
		chg_err("write 0x%x fail, rc=%d\n", reg, rc);

	return rc;
}

static int vout_mv_to_reg(int mv)
{
	if (mv < VOUT_MV_MIN)
		return VOUT_REG_MIN;
	if (mv > VOUT_MV_MAX)
		return VOUT_REG_MAX;
	return (mv - VOUT_MV_MIN) / VOUT_MV_STEP + VOUT_REG_MIN;
}

static int reg_to_vout_mv(int reg)
{
	int data = reg & VOUT_REG_MAX;

	return (data - VOUT_REG_MIN) * VOUT_MV_STEP + VOUT_MV_MIN;
}

static int hl7603_ilim_ma_to_reg(int ma)
{
	if (ma < HL7603_ILIM_MA_MIN)
		return HL7603_ILIM_REG_MIN;
	if (ma > HL7603_ILIM_MA_MAX)
		return HL7603_ILIM_REG_MAX;

	return (ma - HL7603_ILIM_MA_MIN) / HL7603_ILIM_MA_STEP + HL7603_ILIM_REG_MIN;
}

static int hl7603_parse_dt(struct chip_hl7603 *chip)
{
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);
	int rc;
	struct device_node *parent_node;

	rc = of_property_read_u32(node, "oplus,vout-mv", &chip->vout_mv);
	if (rc < 0) {
		chg_err("oplus,vout-mv read failed, rc=%d\n", rc);
		chip->vout_mv = DEFAULT_VOUT_MV;
	}

	rc = of_property_read_u32(node, "oplus,ilim-ma", &chip->ilim_ma);
	if (rc < 0) {
		chg_err("oplus,ilim-ma read failed, rc=%d\n", rc);
		chip->ilim_ma = ILIM_MA_MAX;
	}

	rc = of_property_read_u32(node, "oplus,id-match-status", &chip->id_match_status);
	if (rc < 0) {
		chg_err("oplus,id-match-status read failed, rc=%d\n", rc);
		chip->id_match_status = GPIO_STATUS_NOT_SUPPORT;
	}
	parent_node = of_get_parent(node);
	if (parent_node) {
		chip->fpga_support = of_property_read_bool(parent_node, "oplus,fpga_support");
		if (chip->fpga_support)
			chg_info("fpga_support=%d\n", chip->fpga_support);
	} else {
		chip->fpga_support = 0;
	}

	chg_info("vout_mv=%d,ilim_ma=%d,id_match_status=%d\n", chip->vout_mv, chip->ilim_ma, chip->id_match_status);
	return 0;
}

static bool hl7603_id_not_support(struct chip_hl7603 *chip)
{
	if (!chip || !gpio_is_valid(chip->id_gpio) ||
	    IS_ERR_OR_NULL(chip->pinctrl) || IS_ERR_OR_NULL(chip->id_not_pull) ||
	    IS_ERR_OR_NULL(chip->id_pull_down) || IS_ERR_OR_NULL(chip->id_pull_up))
		return true;
	else
		return false;
}

static int hl7603_get_id_status(struct chip_hl7603 *chip)
{
	int value_up = 0, value_down = 0;
	int gpio_value = GPIO_STATUS_NOT_SUPPORT;

	if (hl7603_id_not_support(chip)) {
		return GPIO_STATUS_NOT_SUPPORT;
	}

	mutex_lock(&chip->pinctrl_lock);
	gpio_direction_input(chip->id_gpio);
	pinctrl_select_state(chip->pinctrl, chip->id_pull_up);
	usleep_range(10000, 10000);
	value_up = gpio_get_value(chip->id_gpio);

	pinctrl_select_state(chip->pinctrl, chip->id_pull_down);
	usleep_range(10000, 10000);
	value_down = gpio_get_value(chip->id_gpio);

	pinctrl_select_state(chip->pinctrl, chip->id_not_pull);

	if (value_up == 1 && value_down == 0)
		gpio_value = GPIO_STATUS_NC;
	else if (value_up == 0 && value_down == 0)
		gpio_value = GPIO_STATUS_PD;
	else if (value_up == 1 && value_down == 1)
		gpio_value = GPIO_STATUS_PU;
	chg_info("value_up=%d value_down=%d\n", value_up, value_down);
	mutex_unlock(&chip->pinctrl_lock);

	return gpio_value;
}

/*workround: Increase the time Q3 is turned off in boost mode to avoid high current backflow*/
static void set_boost_control(struct chip_hl7603 *chip)
{
	int ret = 0;
	unsigned int reg_val = 0;
	unsigned int new_val = 0;

	ret = hl7603_read(chip, HL7603X_REG_BOOST_CONTROL, &reg_val);
	if (ret) {
		chg_err("hl7603x read HL7603X_REG_BOOST_CONTROL i2c error:%d\n", ret);
		return;
	}
	new_val = (u8)reg_val & ~HL7603X_BOOST_CONTROL_BIT6_MASK;

	ret = hl7603_write(chip, HL7603X_REG_TEST_MODE, HL7603X_REG_TEST_MODE_OPEN);
	if (ret) {
		chg_err("hl7603x HL7603X_REG_TEST_MODE_OPEN i2c error:%d\n", ret);
		return;
	}

	ret = hl7603_write(chip, HL7603X_REG_BOOST_CONTROL, new_val);
	if (ret) {
		chg_err("hl7603x HL7603X_REG_BOOST_CONTROL_VAL i2c error:%d\n", ret);
	}

	ret = hl7603_write(chip, HL7603X_REG_TEST_MODE, HL7603X_REG_TEST_MODE_CLOSE);
	if (ret) {
		chg_err("hl7603x HL7603X_REG_TEST_MODE_CLOSE i2c error:%d\n", ret);
	}
	chg_info("hl7603x: after w 0x%02x=0x%02x\n", HL7603X_REG_BOOST_CONTROL, (u8)new_val);
}

static void check_boost_control_twice(struct chip_hl7603 *chip)
{
	int ret = 0;
	unsigned int reg_val = 0;
	unsigned int reg_test_mode = 0;

	ret = hl7603_read(chip, HL7603X_REG_BOOST_CONTROL, &reg_val);
	chg_info("hl7603x: before r 0x%02x=0x%02x\n", HL7603X_REG_BOOST_CONTROL, (u8)reg_val);

	if (!ret && ((u8)reg_val & HL7603X_BOOST_CONTROL_BIT6_MASK)) {
		set_boost_control(chip);
		ret = hl7603_read(chip, HL7603X_REG_BOOST_CONTROL, &reg_val);
		if (!ret && ((u8)reg_val & HL7603X_BOOST_CONTROL_BIT6_MASK)) {
			set_boost_control(chip);
			hl7603_read(chip, HL7603X_REG_BOOST_CONTROL, &reg_val);
		}

		ret = hl7603_read(chip, HL7603X_REG_TEST_MODE, &reg_test_mode);
		if (!ret && (u8)reg_test_mode != HL7603X_REG_TEST_MODE_CLOSE_STATUS) {
			ret = hl7603_write(chip, HL7603X_REG_TEST_MODE, HL7603X_REG_TEST_MODE_CLOSE);
			if (ret) {
				chg_err("hl7603x HL7603X_REG_TEST_MODE_CLOSE error i2c error:%d\n", ret);
			}
		}
	}

	chg_info("hl7603x: after r 0x%02x=0x%02x\n", HL7603X_REG_BOOST_CONTROL, (u8)reg_val);
}

static int hl7603_gpio_init(struct chip_hl7603 *chip)
{
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);
	int rc = 0;

	chip->id_gpio = of_get_named_gpio(node, "oplus,id-gpio", 0);
	if (!gpio_is_valid(chip->id_gpio)) {
		chg_err("id gpio not specified\n");
		goto error;
	}
	rc = gpio_request(chip->id_gpio, "hl7603-id-gpio");
	if (rc < 0) {
		chg_err("hl7603-id gpio request error, rc=%d\n", rc);
		goto free_id_gpio;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		goto free_id_gpio;
	}

	chip->id_not_pull = pinctrl_lookup_state(chip->pinctrl, "id_not_pull");
	if (IS_ERR_OR_NULL(chip->id_not_pull)) {
		chg_err("get id_not_pull fail\n");
		goto free_id_gpio;
	}

	chip->id_pull_up = pinctrl_lookup_state(chip->pinctrl, "id_pull_up");
	if (IS_ERR_OR_NULL(chip->id_pull_up)) {
		chg_err("get id_pull_up fail\n");
		goto free_id_gpio;
	}
	chip->id_pull_down = pinctrl_lookup_state(chip->pinctrl, "id_pull_down");
	if (IS_ERR_OR_NULL(chip->id_pull_down)) {
		chg_err("get id_pull_down fail\n");
		goto free_id_gpio;
	}

	chip->probe_gpio_status = hl7603_get_id_status(chip);
	chg_info("id gpio value %d %s\n", chip->probe_gpio_status, gpio_status_name[chip->probe_gpio_status]);
	return 0;

free_id_gpio:
	if (!gpio_is_valid(chip->id_gpio))
		gpio_free(chip->id_gpio);
error:
	chip->probe_gpio_status = GPIO_STATUS_NOT_SUPPORT;
	return rc;
}


#define HL7603_CHIP_ID_REV	0xB3	/* the HL7603 default value of address 0x0 */
static int hl7603_hardware_init(struct chip_hl7603 *chip)
{
	int rc = 0;
	u8 buf[HL7603_REG_CNT + 4] = { 0 };

	rc = hl7603_read(chip, DEV_ID_REV_REG, (unsigned int *)&buf[6]);
	if (rc >= 0 && buf[6] != HL7603_CHIP_ID_REV) {
		chip->chip_id = buf[6];
		chg_info("chip_id 0x%x is not HL7603\n", chip->chip_id);
		return 0;
	}

	check_boost_control_twice(chip);
	rc = hl7603_write(chip, HL7603_VOUT_SEL_REG, vout_mv_to_reg(chip->vout_mv));
	rc = hl7603_write(chip, HL7603_ILIM_SET_REG, hl7603_ilim_ma_to_reg(chip->ilim_ma));
	rc = hl7603_read(chip, HL7603_CONFIG_1_REG, (unsigned int *)&buf[0]);

	/* Auto PFM mode. */
	rc = hl7603_write(chip, HL7603_CONFIG_1_REG, HL7603_FPWM_CFG_AUTO_PFM_REG);
	rc = hl7603_read(chip, HL7603_CONFIG_1_REG, (unsigned int *)&buf[0]);
	rc = hl7603_read(chip, HL7603_VOUT_SEL_REG, (unsigned int *)&buf[1]);
	rc = hl7603_read(chip, HL7603_ILIM_SET_REG, (unsigned int *)&buf[2]);
	rc = hl7603_read(chip, HL7603_CONFIG_2_REG, (unsigned int *)&buf[3]);
	rc = hl7603_read(chip, HL7603_STATUS_REG, (unsigned int *)&buf[4]);

	chg_info("buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x, buf[4]=0x%x, buf[6]=0x%x" \
		 "vout_mv_to_reg(%d mV)=0x%x, ilim_ma_to_reg(%d mA) = 0x%x",
		  buf[0], buf[1], buf[2], buf[3], buf[4], buf[6],
		  chip->vout_mv, vout_mv_to_reg(chip->vout_mv),
		  chip->ilim_ma, hl7603_ilim_ma_to_reg(chip->ilim_ma));

	if (rc >= 0 && (buf[1] == vout_mv_to_reg(chip->vout_mv)) &&
	    ((buf[2] & 0xff) == hl7603_ilim_ma_to_reg(chip->ilim_ma)))
		chip->i2c_success = true;
	else
		chip->i2c_success = false;

	chg_info("i2c %s reg=%*ph\n", chip->i2c_success ? "success" : "fail", HL7603_REG_CNT, buf);

	return 0;
}

struct oplus_chg_ic_virq hl7603_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
};

#define TRACK_UPLOAD_COUNT_MAX 10
#define TRACK_LOCAL_T_NS_TO_S_THD 1000000000
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD (24 * 3600)
static int hl7603_push_err(struct oplus_chg_ic_dev *ic_dev,
				   bool i2c_error, int err_code, char *reg, bool tsd)
{
	static int upload_count = 0;
	static int pre_upload_time = 0;
	int curr_time;

	curr_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count >= TRACK_UPLOAD_COUNT_MAX)
		return 0;

	pre_upload_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;

	if (i2c_error)
		oplus_chg_ic_creat_err_msg(ic_dev, OPLUS_IC_ERR_I2C, 0,
			"$$err_scene@@i2c_err$$err_reason@@%d$$byb_id@@%d", err_code, ic_dev->index);
	else
		oplus_chg_ic_creat_err_msg(ic_dev, OPLUS_IC_ERR_BUCK_BOOST, 0,
			"$$err_scene@@byb_work_err$$err_reason@@%s$$reg_info@@%s$$byb_id@@%d",
			tsd ? "TSD" : "normal", reg, ic_dev->index);

	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ERR);
	upload_count++;
	return 0;
}

static int hl7603_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct chip_hl7603 *chip;
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ic_dev->online = true;
	return 0;
}

static int hl7603_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	return 0;
}

#define REG_INFO_LEN 128
static int hl7603_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct chip_hl7603 *chip;
	u8 buf[HL7603_REG_CNT + 3];
	int rc;
	char reg_info[REG_INFO_LEN] = { 0 };

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!ic_dev->online || !chip->i2c_success)
		return 0;

	if(atomic_read(&chip->suspended) == 1) {
		chg_err("in suspended\n");
		return 0;
	}

	rc = hl7603_read(chip, HL7603_CONFIG_1_REG, (unsigned int *)&buf[0]);
	rc = hl7603_read(chip, HL7603_VOUT_SEL_REG, (unsigned int *)&buf[1]);
	rc = hl7603_read(chip, HL7603_ILIM_SET_REG, (unsigned int *)&buf[2]);
	rc = hl7603_read(chip, HL7603_CONFIG_2_REG, (unsigned int *)&buf[3]);
	rc = hl7603_read(chip, HL7603_STATUS_REG, (unsigned int *)&buf[4]);
	rc = hl7603_read(chip, HL7603X_REG_BOOST_CONTROL, (unsigned int *)&buf[5]);

	chg_err("%*ph\n", HL7603_REG_CNT, buf);

	if (rc < 0 || hl7603_debug_track) {
		snprintf(reg_info, REG_INFO_LEN, "reg01~04:[%*ph]", HL7603_REG_CNT, buf);
		hl7603_push_err(ic_dev, rc < 0, rc, reg_info, 0);
	}

	return 0;
}

static int oplus_get_byb_id_info(struct oplus_chg_ic_dev *ic_dev, int *count)
{
	struct chip_hl7603 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*count = chip->probe_gpio_status;

	return 0;
}

static int oplus_get_byb_id_match_info(struct oplus_chg_ic_dev *ic_dev, int *count)
{
	struct chip_hl7603 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (chip->id_match_status == GPIO_STATUS_NOT_SUPPORT)
		return -ENOTSUPP;

	if (chip->probe_gpio_status == chip->id_match_status)
		*count = ID_MATCH_SILI;
	else
		*count = ID_NOT_MATCH;

	return 0;
}

static int oplus_fpga_reset_notify(struct oplus_chg_ic_dev *ic_dev, int status)
{
	struct chip_hl7603 *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->fpga_support)
		return rc;
	chg_err("status = %d", status);
	if (status == FPGA_RESET_START)
		set_bit(FPGA_RST, &chip->rst_ing);
	else if (status == FPGA_RESET_END)
		clear_bit(FPGA_RST, &chip->rst_ing);
	else if (status == GUAGE_I2C_RST_START)
		set_bit(I2C_RST, &chip->rst_ing);
	else if (status == GUAGE_I2C_RST_END)
		clear_bit(I2C_RST, &chip->rst_ing);

	return rc;
}

static bool oplus_chg_fun_call_check(struct oplus_chg_ic_dev *ic_dev,
				enum oplus_chg_ic_func func_id)
{
	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT))
	    return true;

	return false;
}

static int oplus_get_byb_status(struct oplus_chg_ic_dev *ic_dev, char *buf)
{
	struct chip_hl7603 *chip;
	int reg_val = 0;
	int size = 0;
	int rc = 0;
	int status = BYB_STATUS_FAULT;
	int gpio_status = GPIO_STATUS_NOT_SUPPORT;

	if (ic_dev == NULL || buf == NULL) {
		chg_err("ic_dev or buf is NULL\n");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (chip == NULL || (chip->probe_gpio_status != chip->id_match_status && !chip->i2c_success))
		return -ENOTSUPP;

	if(atomic_read(&chip->suspended) == 1) {
		chg_err("in suspended\n");
		size += scnprintf(buf + size, PAGE_SIZE - size, "in_suspended|id_%d:in_suspended|", ic_dev->index);
		return size;
	}

	gpio_status = hl7603_get_id_status(chip);
	if (gpio_status != chip->id_match_status) {
		chg_err("id not match %d %d,", gpio_status, chip->id_match_status);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"id_%d:id_not_match_%d_%d,", ic_dev->index, gpio_status, chip->id_match_status);
	}

	rc = hl7603_read(chip, HL7603_STATUS_REG, &reg_val);
	if (rc < 0) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
				"0x%02x_read_fail_%d|id_%d:0x%02x_read_fail_%d|",
				HL7603_STATUS_REG, rc, ic_dev->index, HL7603_STATUS_REG, rc);
		return size;
	}

	if ((reg_val & STATUS_MASK) == BOOST_STATUS_NORMAL)
		status = BYB_STATUS_BOOST;
	else if ((reg_val & STATUS_MASK) == BYPASS_STATUS_NORMAL)
		status = BYB_STATUS_BYPASS;

	size += scnprintf(buf + size, PAGE_SIZE - size,
			"%s|id_%d:0x%02x:0x%02x", byb_status_name[status], ic_dev->index, HL7603_STATUS_REG, reg_val);

	rc = hl7603_read(chip, HL7603_VOUT_SEL_REG, &reg_val);
	if (rc < 0) {
		size += scnprintf(buf + size, PAGE_SIZE - size, ",0x%02x_read_fail_%d|", HL7603_VOUT_SEL_REG, rc);
		return size;
	}

	size += scnprintf(buf + size, PAGE_SIZE - size, ",vout:%dmv|", reg_to_vout_mv(reg_val));

	chg_info("byb_status_show in id:%d,%s\n", ic_dev->index, buf);
	return size;
}

static int oplus_get_byb_vout(struct oplus_chg_ic_dev *ic_dev, char *buf)
{
	struct chip_hl7603 *chip;
	int reg_val = 0;
	int size = 0;
	int rc = 0;
	int vout = 0;

	if (ic_dev == NULL || buf == NULL) {
		chg_err("ic_dev or buf is NULL\n");
		return -EINVAL;
	}
	if (!ic_dev->online)
		return -ENOTSUPP;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}
	if(atomic_read(&chip->suspended) == 1) {
		chg_err("in suspended\n");
		size += scnprintf(buf + size, PAGE_SIZE - size, "in_suspended|");
		return size;
	}

	rc = hl7603_read(chip, HL7603_VOUT_SEL_REG, &reg_val);
	if (rc < 0) {
		size += scnprintf(buf + size, PAGE_SIZE - size, "0x%02x_read_fail:%d|", HL7603_VOUT_SEL_REG, rc);
		return size;
	}

	vout = reg_to_vout_mv(reg_val);
	size += scnprintf(buf + size, PAGE_SIZE - size, "%d|", vout);
	return size;
}

static int oplus_set_byb_vout(struct oplus_chg_ic_dev *ic_dev, int vout)
{
	struct chip_hl7603 *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL\n");
		return -EINVAL;
	}
	if (!ic_dev->online)
		return -ENOTSUPP;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if(atomic_read(&chip->suspended) == 1) {
		chg_err("in suspended\n");
		return -EAGAIN;
	}

	if (vout == 1)
		rc = hl7603_write(chip, HL7603_VOUT_SEL_REG, vout_mv_to_reg(HIGH_VOUT_MV));
	else if (vout <= 0)
		rc = hl7603_write(chip, HL7603_VOUT_SEL_REG, vout_mv_to_reg(chip->vout_mv));
	else
		rc = hl7603_write(chip, HL7603_VOUT_SEL_REG, vout_mv_to_reg(vout));
	if (rc < 0)
		chg_err("write voutroof fail, rc=%d\n", rc);

	return rc;
}

static void *oplus_chg_get_func(struct oplus_chg_ic_dev *ic_dev,
				enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (oplus_chg_fun_call_check(ic_dev, func_id)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, hl7603_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, hl7603_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, hl7603_reg_dump);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BYB_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BYB_STATUS,
					      oplus_get_byb_status);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BYB_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BYB_VOUT,
					      oplus_get_byb_vout);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_BYB_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_BYB_VOUT,
					      oplus_set_byb_vout);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BYBID_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BYBID_INFO, oplus_get_byb_id_info);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BYBID_MATCH_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BYBID_MATCH_INFO, oplus_get_byb_id_match_info);
		break;
	case OPLUS_IC_FUNC_BUCK_FPGA_RST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_FPGA_RST,
					      oplus_fpga_reset_notify);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
static bool test_kit_boost_id_gpio_test(struct test_feature *feature, char *buf, size_t len)
{
	struct chip_hl7603 *chip;
	int index = 0;
	int gpio_status = GPIO_STATUS_NOT_SUPPORT;

	if (buf == NULL) {
		pr_err("buf is NULL\n");
		return false;
	}
	if (feature == NULL) {
		pr_err("feature is NULL\n");
		index += snprintf(buf + index, len - index, "feature is NULL");
		return false;
	}

	chip = feature->private_data;
	gpio_status = hl7603_get_id_status(chip);
	index += snprintf(buf + index, len - index, "%s\n", gpio_status_name[gpio_status]);
	return true;
}

static bool test_kit_fpga_boost_test(struct test_feature *feature, char *buf, size_t len)
{
	struct chip_hl7603 *chip;
	int index = 0;
	int reg_val = 0;
	int rc = 0;
	int vout = 0;

	if (buf == NULL) {
		pr_err("buf is NULL\n");
		return false;
	}
	if (feature == NULL) {
		pr_err("feature is NULL\n");
		index += snprintf(buf + index, len - index, "feature is NULL");
		return false;
	}

	chip = feature->private_data;

	rc = hl7603_read(chip, HL7603_VOUT_SEL_REG, &reg_val);
	if (rc < 0) {
		return false;
	}
	vout = reg_to_vout_mv(reg_val);
	chg_info("vout=%d", vout);
	if (vout == chip->vout_mv)
		return true;
	else
		return false;
}

static const struct test_feature_cfg boost_id_gpio_test_cfg = {
	.name = "boost_id_gpio_test",
	.test_func = test_kit_boost_id_gpio_test,
};

static const struct test_feature_cfg fpga_boost_test_cfg = {
	.name = "fpga_boost_test",
	.test_func = test_kit_fpga_boost_test,
};
#endif

#ifdef CONFIG_OPLUS_CHG_IC_DEBUG
static ssize_t byb_status_show(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
	struct oplus_chg_ic_dev *ic_dev = dev_get_drvdata(dev);
	struct chip_hl7603 *chip = oplus_chg_ic_get_drvdata(ic_dev);
	int reg_val = 0;
	int size = 0;
	int rc = 0;
	int status = BYB_STATUS_FAULT;
	int gpio_status = GPIO_STATUS_NOT_SUPPORT;

	if (chip->probe_gpio_status != chip->id_match_status && !chip->i2c_success)
		return -ENOTSUPP;

	if(atomic_read(&chip->suspended) == 1) {
		chg_err("in suspended\n");
		size += scnprintf(buf + size, PAGE_SIZE - size, "in_suspended|in_suspended\n");
		return size;
	}

	gpio_status = hl7603_get_id_status(chip);
	if (gpio_status != chip->id_match_status) {
		chg_err("id not match %d %d,", gpio_status, chip->id_match_status);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"id_not_match_%d_%d,", gpio_status, chip->id_match_status);
	}

	rc = hl7603_read(chip, HL7603_STATUS_REG, &reg_val);
	if (rc < 0) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
				"0x%02x_read_fail:%d|0x%02x_read_fail:%d\n",
				HL7603_STATUS_REG, rc, HL7603_STATUS_REG, rc);
		return size;
	}

	if ((reg_val & STATUS_MASK) == BOOST_STATUS_NORMAL)
		status = BYB_STATUS_BOOST;
	else if ((reg_val & STATUS_MASK) == BYPASS_STATUS_NORMAL)
		status = BYB_STATUS_BYPASS;

	size += scnprintf(buf + size, PAGE_SIZE - size,
			"%s|0x%02x:0x%02x", byb_status_name[status], HL7603_STATUS_REG, reg_val);

	rc = hl7603_read(chip, HL7603_VOUT_SEL_REG, &reg_val);
	if (rc < 0) {
		size += scnprintf(buf + size, PAGE_SIZE - size, "0x%02x_read_fail:%d\n", HL7603_VOUT_SEL_REG, rc);
		return size;
	}

	size += scnprintf(buf + size, PAGE_SIZE - size, ",vout:%dmv\n", reg_to_vout_mv(reg_val));

	return size;
}
static DEVICE_ATTR_RO(byb_status);

static ssize_t byb_vout_show(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
	struct oplus_chg_ic_dev *ic_dev = dev_get_drvdata(dev);
	struct chip_hl7603 *chip = oplus_chg_ic_get_drvdata(ic_dev);
	int reg_val = 0;
	int size = 0;
	int rc = 0;
	int vout = 0;

	if (!ic_dev->online)
		return -ENOTSUPP;

	if(atomic_read(&chip->suspended) == 1) {
		chg_err("in suspended\n");
		size += scnprintf(buf + size, PAGE_SIZE - size, "in_suspended\n");
		return size;
	}

	rc = hl7603_read(chip, HL7603_VOUT_SEL_REG, &reg_val);
	if (rc < 0) {
		size += scnprintf(buf + size, PAGE_SIZE - size, "0x%02x_read_fail:%d\n", HL7603_VOUT_SEL_REG, rc);
		return size;
	}

	vout = reg_to_vout_mv(reg_val);
	size += scnprintf(buf + size, PAGE_SIZE - size, "%d\n", vout);
	return size;
}

static ssize_t byb_vout_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct oplus_chg_ic_dev *ic_dev = dev_get_drvdata(dev);
	struct chip_hl7603 *chip = oplus_chg_ic_get_drvdata(ic_dev);
	int val = 0;
	int rc = 0;

	if (!ic_dev->online)
		return -ENOTSUPP;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	if(atomic_read(&chip->suspended) == 1) {
		chg_err("in suspended\n");
		return -EAGAIN;
	}

	if (val == 1)
		rc = hl7603_write(chip, HL7603_VOUT_SEL_REG, vout_mv_to_reg(HIGH_VOUT_MV));
	else if (val <= 0)
		rc = hl7603_write(chip, HL7603_VOUT_SEL_REG, vout_mv_to_reg(chip->vout_mv));
	else
		rc = hl7603_write(chip, HL7603_VOUT_SEL_REG, vout_mv_to_reg(val));
	if (rc < 0)
		chg_err("write voutroof fail, rc=%d\n", rc);

	return count;
}

static struct device_attribute dev_attr_byb_vout = {
	.attr = {
		.name = __stringify(byb_vout),
		.mode = 0666
	},
	.show = byb_vout_show,
	.store = byb_vout_store,
};

static struct device_attribute *hl7603_attributes[] = {
	&dev_attr_byb_status,
	&dev_attr_byb_vout,
	NULL
};
#endif

static void oplus_hl7603_retry_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct chip_hl7603 *chip = container_of(
		dwork, struct chip_hl7603, retry_init_work);

	hl7603_hardware_init(chip);
	if (!chip->i2c_success)
		schedule_delayed_work(&chip->retry_init_work, msecs_to_jiffies(5000));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int hl7603_driver_probe(struct i2c_client *client)
#else
static int hl7603_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct chip_hl7603 *chip;
	struct device_node *node = oplus_get_node_by_type(client->dev.of_node);
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int rc;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	chip = devm_kzalloc(&client->dev, sizeof(struct chip_hl7603), GFP_KERNEL);
	if (!chip) {
		chg_err("failed to allocate chip_hl7603\n");
		return -ENOMEM;
	}

	chip->regmap = devm_regmap_init_i2c(client, &hl7603_regmap_config);
	if (!chip->regmap) {
		rc = -ENODEV;
		goto regmap_init_err;
	}

	chip->dev = &client->dev;
	chip->client = client;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->pinctrl_lock);
	atomic_set(&chip->suspended, 0);

	hl7603_parse_dt(chip);

	rc = hl7603_gpio_init(chip);
	if (rc < 0) {
		chg_err("hl7603 gpio init failed, rc = %d!\n", rc);
	}

	rc = hl7603_hardware_init(chip);
	if (rc < 0) {
		chg_err("hl7603 ic init failed, rc = %d!\n", rc);
		goto gpio_init_err;
	}

	rc = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		goto reg_ic_err;
	}
	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto reg_ic_err;
	}
	ic_cfg.name = client->dev.of_node->name;
	ic_cfg.index = ic_index;
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "buck-BYB/HL7603:%d", ic_index);
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = hl7603_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(hl7603_virq_table);
	ic_cfg.of_node = client->dev.of_node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto reg_ic_err;
	}

#ifdef CONFIG_OPLUS_CHG_IC_DEBUG
	attrs = hl7603_attributes;
	while ((attr = *attrs++)) {
		rc = device_create_file(chip->ic_dev->debug_dev, attr);
		if (rc) {
			chg_err("device_create_file fail!\n");
			goto reg_ic_err;
		}
	}
#endif

#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
	if (gpio_is_valid(chip->id_gpio) && (chip->probe_gpio_status == chip->id_match_status || chip->i2c_success)) {
		chip->boost_id_gpio_test = test_feature_register(&boost_id_gpio_test_cfg, chip);
		if (IS_ERR_OR_NULL(chip->boost_id_gpio_test))
			chg_err("boost_id_gpio_test register error");
		else
			chg_info("boost_id_gpio_test register success");
	}
	if (chip->fpga_support) {
		chip->fpga_boost_test = test_feature_register(&fpga_boost_test_cfg, chip);
		if (IS_ERR_OR_NULL(chip->fpga_boost_test))
			chg_err("fpga_boost_test register error");
		else
			chg_info("fpga_boost_test register success");
	}
	INIT_DELAYED_WORK(&chip->retry_init_work, oplus_hl7603_retry_init_work);
	if (!chip->i2c_success && chip->probe_gpio_status == chip->id_match_status)
		schedule_delayed_work(&chip->retry_init_work, msecs_to_jiffies(5000));
#endif
	chg_info("success!\n");
	return 0;

reg_ic_err:
gpio_init_err:
	if (gpio_is_valid(chip->id_gpio))
		gpio_free(chip->id_gpio);
regmap_init_err:
	devm_kfree(&client->dev, chip);
	return rc;
}

static int hl7603_pm_resume(struct device *dev_chip)
{
	struct i2c_client *client  = container_of(dev_chip, struct i2c_client, dev);
	struct chip_hl7603 *chip = i2c_get_clientdata(client);

	if (chip == NULL)
		return 0;

	atomic_set(&chip->suspended, 0);

	return 0;
}

static int hl7603_pm_suspend(struct device *dev_chip)
{
	struct i2c_client *client  = container_of(dev_chip, struct i2c_client, dev);
	struct chip_hl7603 *chip = i2c_get_clientdata(client);

	if (chip == NULL)
		return 0;

	atomic_set(&chip->suspended, 1);

	return 0;
}

static const struct dev_pm_ops hl7603_pm_ops = {
	.resume = hl7603_pm_resume,
	.suspend = hl7603_pm_suspend,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
static int hl7603_driver_remove(struct i2c_client *client)
{
	struct chip_hl7603 *chip = i2c_get_clientdata(client);

	if(chip == NULL)
		return 0;

#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
	if (!IS_ERR_OR_NULL(chip->boost_id_gpio_test))
		test_feature_unregister(chip->boost_id_gpio_test);
	if (!IS_ERR_OR_NULL(chip->fpga_boost_test))
		test_feature_unregister(chip->fpga_boost_test);
#endif

	if (!gpio_is_valid(chip->id_gpio))
		gpio_free(chip->id_gpio);
	devm_kfree(&client->dev, chip);
	return 0;
}
#else
static void hl7603_driver_remove(struct i2c_client *client)
{
	struct chip_hl7603 *chip = i2c_get_clientdata(client);

	if(chip == NULL)
		return;

	if (!gpio_is_valid(chip->id_gpio))
		gpio_free(chip->id_gpio);
	devm_kfree(&client->dev, chip);
}
#endif

static void hl7603_shutdown(struct i2c_client *chip_client)
{
	struct chip_hl7603 *chip = i2c_get_clientdata(chip_client);

	if(chip == NULL)
		return;

	return;
}

static const struct of_device_id hl7603_match[] = {
	{.compatible = "oplus,byb-hl7603"},
	{},
};

static const struct i2c_device_id hl7603_id[] = {
	{"oplus,byb-hl7603", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, hl7603_id);


static struct i2c_driver hl7603_i2c_driver = {
	.driver		= {
		.name = "byb-hl7603",
		.owner	= THIS_MODULE,
		.of_match_table = hl7603_match,
		.pm = &hl7603_pm_ops,
	},
	.probe		= hl7603_driver_probe,
	.remove		= hl7603_driver_remove,
	.id_table	= hl7603_id,
	.shutdown	= hl7603_shutdown,
};

static __init int hl7603_i2c_driver_init(void)
{
#if __and(IS_BUILTIN(CONFIG_OPLUS_CHG), IS_BUILTIN(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return 0;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return 0;
#endif /* CONFIG_OPLUS_CHG_V2 */
	return i2c_add_driver(&hl7603_i2c_driver);
}

static __exit void hl7603_i2c_driver_exit(void)
{
#if (IS_ENABLED(CONFIG_OPLUS_CHG) && IS_ENABLED(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return;
#endif /* CONFIG_OPLUS_CHG_V2 */
	i2c_del_driver(&hl7603_i2c_driver);
}

oplus_chg_module_register(hl7603_i2c_driver);

MODULE_DESCRIPTION("TI Boost/Bypass Driver");
MODULE_LICENSE("GPL v2");
