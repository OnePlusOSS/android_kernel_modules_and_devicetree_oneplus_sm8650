// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021-2021 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[SY6974B]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <soc/oplus/device_info.h>
#include <linux/iio/consumer.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_cpa.h>
#include "oplus_hal_sy6974b.h"

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mtk_boot_common.h>
#include "charger_class.h"
#else
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#endif
#endif

#ifndef I2C_ERR_MAX
#define I2C_ERR_MAX 2
#endif

#define BC12_TIMEOUT_MS				msecs_to_jiffies(5000)

#define REG_MAX 				0x0b

#define HIZ_MODE_REG				0x00
#define HIZ_MODE_BIT				BIT(7)
#define CHG_CONFIG_REG				0x01
#define CHG_EN_BIT				BIT(4)
#define WATCHGDOG_CONFIG_REG			0x05
#define WATCHGDOG_CONFIG_BIT			(BIT(4) | BIT(5))
#define VINDPM_OVP_CONFIG_REG			0x06
#define VINDPM_CONFIG_BIT			(BIT(0) | BIT(1) | BIT(2) | BIT(3))
#define BC12_REG				0x07
#define BC12_RERUN_BIT				BIT(7)
#define BC12_RESULT_REG				0x08
#define BC12_RESULT_BIT				(BIT(5) | BIT(6) | BIT(7))
#define IRQ_MASK_REG				0x0a
#define VINDPM_IRQ_MASK_BIT			BIT(1)
#define IINDPM_IRQ_MASK_BIT			BIT(0)
#define R_CHARGER_1     			330
#define R_CHARGER_2     			39

#define AICL_POINT_VOL_9V           		7600
#define AICL_POINT_SWITCH_THRE			7500
#define DUAL_BAT_AICL_POINT_VOL_9V  		8500
#define AICL_POINT_VOL_5V           		4140
#define HW_AICL_POINT_VOL_5V_PHASE1 		4400
#define HW_AICL_POINT_VOL_5V_PHASE2 		4520
#define SW_AICL_POINT_VOL_5V_PHASE1 		4500
#define SW_AICL_POINT_VOL_5V_PHASE2 		4535
#define USB_HW_AICL_POINT           		4600
#define USB_SW_AICL_POINT           		4620
#define AICL_DOWN_DELAY_MS			50
#define AICL_DELAY_MIN_US			90000
#define AICL_DELAY_MAX_US			91000
#define SUSPEND_IBUS_MA				100
#define DEFAULT_IBUS_MA				500

#define I2C_RETRY_DELAY_US			5000
#define I2C_RETRY_WRITE_MAX_COUNT		3
#define I2C_RETRY_READ_MAX_COUNT		20
#define OPLUS_BC12_RETRY_CNT 			1
#define OPLUS_BC12_DELAY_CNT 			18
#define INIT_WORK_NORMAL_DELAY 			1500
#define INIT_WORK_OTHER_DELAY 			1000
#define PRE_EVENT_WORK_DELAY_MS			2000
#define PORT_PD_WITH_USB 			2
#define DISCONNECT_FCC_MAX_CURR			800
#define REAL_SUSPEND_CHECK_INTERVAL		500
#define HIGH_VBUS_THRESHOLD			6900
#define DEF_VBUS_ONLINE_TH			3800

static atomic_t i2c_err_count;

struct sy6974b_chip {
	struct i2c_client *client;
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;

	struct mutex pinctrl_lock;
	struct pinctrl *pinctrl;
	struct pinctrl_state *event_default;
	struct pinctrl_state *dis_vbus_active;
	struct pinctrl_state *dis_vbus_sleep;
	struct mutex i2c_lock;
	struct regmap *regmap;

	struct mutex dpdm_lock;
	struct regulator *dpdm_reg;

	struct work_struct otg_enabled_work;
	struct delayed_work event_work;

	struct delayed_work bc12_timeout_work;
	struct oplus_mms *wired_topic;
	struct oplus_mms *cpa_topic;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *cpa_subs;
	enum oplus_chg_protocol_type cpa_current_type;

	int event_gpio;
	int event_irq;
	int dis_vbus_gpio;

	atomic_t driver_suspended;
	atomic_t charger_suspended;

	bool otg_enable;
	bool vbus_present;
	bool bc12_retry;
	bool auto_bc12;
	bool bc12_complete;
	int charge_type;
	bool event_irq_enabled;

	int before_suspend_icl;
	int before_unsuspend_icl;
	unsigned long request_otg;

	enum power_supply_type	oplus_charger_type;
	struct power_supply *chg_psy;
	struct power_supply	*psy;
	struct power_supply_desc psy_desc;
	struct charger_device *chg_dev;
	const char *chg_dev_name;
	int	hw_aicl_point;
	int	sw_aicl_point;
	bool batfet_reset_disable;
	int	normal_init_delay_ms;
	int	other_init_delay_ms;
	int charger_current_pre;
	struct wakeup_source *suspend_ws;
	bool dpdm_enabled;
	bool power_good;
	struct delayed_work	bc12_retry_work;
	struct delayed_work	pre_event_work;
	bool bc12_done;
	char bc12_delay_cnt;
	char bc12_retried;
	struct votable *fcc_votable;
	struct work_struct fcc_vote_work;
};

enum {
	CHARGE_TYPE_NO_INPUT = 0,
	CHARGE_TYPE_SDP,
	CHARGE_TYPE_CDP,
	CHARGE_TYPE_DCP,
	CHARGE_TYPE_VBUS_TYPE_UNKNOWN = 5,
	CHARGE_TYPE_OCP,
	CHARGE_TYPE_OTG,
};

static struct regmap_config sy6974b_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static void sy6974b_get_bc12(struct sy6974b_chip *chip);
static int sy6974b_set_wdt_timer(struct sy6974b_chip *chip, int reg);
static int sy6974b_inform_charger_type(struct sy6974b_chip *chip);
static int sy6974b_reg_dump(struct oplus_chg_ic_dev *ic_dev);
static void oplus_chg_wakelock(struct sy6974b_chip *chip, bool awake);
static int sy6974b_otg_ilim_set(struct sy6974b_chip *chip, int ilim);
static int sy6974b_disable_charger(struct sy6974b_chip *chip);
static int sy6974b_enable_charger(struct sy6974b_chip *chip);
static int sy6974b_set_iindet(struct sy6974b_chip *chip);
static int sy6974b_otg_enable(struct sy6974b_chip *chip);
static int sy6974b_otg_disable(struct sy6974b_chip *chip);
static int get_vbus_voltage(struct sy6974b_chip *chip, int *val);
static int sy6974b_hardware_init(struct sy6974b_chip *chip);
static void sy6974b_really_suspend_charger(struct sy6974b_chip *chip, bool en);
static bool sy6974b_check_really_suspend_charger(struct sy6974b_chip *chip);

static __inline__ void sy6974b_i2c_err_inc(struct sy6974b_chip *chip)
{
	if (atomic_inc_return(&i2c_err_count) > I2C_ERR_MAX) {
		atomic_set(&i2c_err_count, 0);
		oplus_chg_ic_creat_err_msg(chip->ic_dev, OPLUS_IC_ERR_I2C, 0, "continuous error");
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
	}
}

static __inline__ void sy6974b_i2c_err_clr(void)
{
	atomic_set(&i2c_err_count, 0);
}

static void sy6974b_enable_irq(struct sy6974b_chip *chip, bool en)
{
	if (chip->event_irq_enabled && !en) {
		chip->event_irq_enabled = false;
		disable_irq(chip->event_irq);
	} else if (!chip->event_irq_enabled && en) {
		chip->event_irq_enabled = true;
		enable_irq(chip->event_irq);
	} else {
		chg_info("event_irq_enabled:%s, en:%s\n",
			 true_or_false_str(chip->event_irq_enabled),
			 true_or_false_str(chip->event_irq));
	}
}

static int _sy6974b_read_byte(struct sy6974b_chip *chip, int reg, int *data)
{
	s32 ret = 0;
	int retry = I2C_RETRY_READ_MAX_COUNT;

	ret = i2c_smbus_read_byte_data(chip->client, reg);

	if (ret < 0) {
		while(retry > 0 && atomic_read(&chip->driver_suspended) == 0) {
			usleep_range(I2C_RETRY_DELAY_US, I2C_RETRY_DELAY_US);
			ret = i2c_smbus_read_byte_data(chip->client, reg);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	if (ret < 0) {
		chg_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else
		*data = ret;

	return 0;
}

static int sy6974b_read_byte(struct sy6974b_chip *chip, int addr, int *data)
{
	int rc = 0;
	mutex_lock(&chip->i2c_lock);
	rc = _sy6974b_read_byte(chip, addr, data);
	mutex_unlock(&chip->i2c_lock);
	return rc;
}

static int sy6974b_write_byte(struct sy6974b_chip *chip, int reg, int val)
{
	s32 ret = 0;
	int retry = I2C_RETRY_WRITE_MAX_COUNT;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);

	if (ret < 0) {
		while(retry > 0) {
			usleep_range(I2C_RETRY_DELAY_US, I2C_RETRY_DELAY_US);
			ret = i2c_smbus_write_byte_data(chip->client, reg, val);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	if (ret < 0) {
		chg_err("i2c write fail: can't write %02x to %02x: %d\n", val, reg, ret);
		return ret;
	}

	return 0;
}

static int sy6974b_read_byte_mask(struct sy6974b_chip *chip,
		int addr, int mask, int *data)
{
	int temp = 0;
	int rc;

	rc = sy6974b_read_byte(chip, addr, &temp);
	if (rc < 0)
		return rc;

	*data = mask & temp;

	return 0;
}

static int sy6974b_write_byte_mask(struct sy6974b_chip *chip,
		int addr, int mask, int data)
{
	int temp = 0;
	int rc;
	mutex_lock(&chip->i2c_lock);
	rc = _sy6974b_read_byte(chip, addr, &temp);
	if (rc < 0)
		goto ERR;

	temp = (data & mask) | (temp & (~mask));
	rc = sy6974b_write_byte(chip, addr, temp);
ERR:
	mutex_unlock(&chip->i2c_lock);
	return 0;
}

static int sy6974b_request_dpdm(struct sy6974b_chip *chip, bool enable)
{
	int rc = 0;

	/* fetch the DPDM regulator */
	if (!chip->dpdm_reg && of_get_property(chip->dev->of_node, "dpdm-supply", NULL)) {
		chip->dpdm_reg = devm_regulator_get_optional(chip->dev, "dpdm");
		if (IS_ERR(chip->dpdm_reg)) {
			rc = PTR_ERR(chip->dpdm_reg);
			chg_err("Couldn't get dpdm regulator, rc=%d\n", rc);
			chip->dpdm_reg = NULL;
			return rc;
		}
	}

	mutex_lock(&chip->dpdm_lock);
	if (enable) {
		if (chip->dpdm_reg) {
			chg_info("enabling DPDM regulator\n");
			rc = regulator_enable(chip->dpdm_reg);
			if (rc < 0)
				chg_err("Couldn't enable dpdm regulator rc=%d\n", rc);
		}
	} else {
		if (chip->dpdm_reg) {
			chg_err("disabling DPDM regulator\n");
			rc = regulator_disable(chip->dpdm_reg);
			if (rc < 0)
				chg_err("Couldn't disable dpdm regulator rc=%d\n", rc);
		}
	}
	mutex_unlock(&chip->dpdm_lock);

	return rc;
}

int sy6974b_get_iindet(struct sy6974b_chip *chip)
{
	int rc = 0;
	int reg_val = 0;
	bool is_complete = false;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_read_byte(chip, REG07_SY6974B_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG07_SY6974B_ADDRESS rc = %d\n", rc);
		return false;
	}

	is_complete = ((reg_val & REG07_SY6974B_IINDET_EN_MASK) ==
			REG07_SY6974B_IINDET_EN_DET_COMPLETE) ? 1 : 0;
	return is_complete;
}

bool sy6974b_get_bus_gd(struct sy6974b_chip *chip)
{
	int rc = 0;
	int reg_val = 0;
	bool bus_gd = false;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_read_byte(chip, REG0A_SY6974B_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't oplus_chg_is_usb_present rc = %d\n", rc);
		return false;
	}

	bus_gd = ((reg_val & REG0A_SY6974B_BUS_GD_MASK) == REG0A_SY6974B_BUS_GD_YES) ? 1 : 0;
	return bus_gd;
}

static void sy6974b_event_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sy6974b_chip *chip = container_of(dwork, struct sy6974b_chip, event_work);
	int rc = 0;
	union mms_msg_data data = { 0 };
	bool prev_pg = false, curr_pg = false, bus_gd = false;
	int vbus = 0;
	bool hiz = false;

	hiz = sy6974b_check_really_suspend_charger(chip);

	rc = oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, false);
	if (rc < 0) {
		chg_err("cannot get CPA_ITEM_ALLOW data, rc=%d\n", rc);
		return;
	}

	get_vbus_voltage(chip, &vbus);

	chip->cpa_current_type = data.intval;

	if (chip->otg_enable) {
		chg_info("is otg mode\n");
		return;
	}

	prev_pg = chip->power_good;
	curr_pg = bus_gd = sy6974b_get_bus_gd(chip);

	if (sy6974b_get_bus_gd(chip) || vbus > DEF_VBUS_ONLINE_TH) {
		curr_pg = bus_gd = true;
		if (!sy6974b_get_bus_gd(chip)) {
			schedule_delayed_work(&chip->event_work,
				msecs_to_jiffies(REAL_SUSPEND_CHECK_INTERVAL));
			if (vbus > HIGH_VBUS_THRESHOLD)
				return;
		}
	} else {
		curr_pg = bus_gd = false;
	}

	chip->vbus_present = curr_pg;
	chip->power_good = curr_pg;
	chg_info("(%d,%d, %d, %d)\n", prev_pg, chip->power_good, curr_pg, bus_gd);
	if (curr_pg)
		oplus_chg_wakelock(chip, true);

	if (!prev_pg && chip->power_good) {
		oplus_chg_wakelock(chip, true);
#ifdef CONFIG_OPLUS_CHARGER_MTK
		if (get_boot_mode() != META_BOOT)
			Charger_Detect_Init();
#endif
		sy6974b_request_dpdm(chip, true);
		sy6974b_set_wdt_timer(chip, REG05_SY6974B_WATCHDOG_TIMER_40S);
		chip->bc12_done = false;
		chip->bc12_retried = 0;
		chip->bc12_delay_cnt = 0;
		if (hiz)
			sy6974b_really_suspend_charger(chip, false);
		if (chip->oplus_charger_type == POWER_SUPPLY_TYPE_UNKNOWN)
			sy6974b_get_bc12(chip);
	} else if (prev_pg && !chip->power_good) {
		chip->bc12_done = false;
		chip->bc12_retried = 0;
		chip->bc12_delay_cnt = 0;
		chip->oplus_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		chip->charger_current_pre = -1;
		sy6974b_request_dpdm(chip, false);
#ifdef CONFIG_OPLUS_CHARGER_MTK
		Charger_Detect_Release();
		oplus_chg_pullup_dp_set(false);
#endif
		sy6974b_inform_charger_type(chip);
		sy6974b_set_wdt_timer(chip, REG05_SY6974B_WATCHDOG_TIMER_DISABLE);
		oplus_chg_wakelock(chip, false);
	} else if (!prev_pg && !chip->power_good) {
		chg_err("prev_pg & now_pg is false\n");
#ifdef CONFIG_OPLUS_CHARGER_MTK
		Charger_Detect_Release();
#endif
		chip->bc12_done = false;
		chip->bc12_retried = 0;
		chip->bc12_delay_cnt = 0;
	}

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (chip->oplus_charger_type == POWER_SUPPLY_TYPE_USB_CDP)
		oplus_chg_pullup_dp_set(true);
	else
		oplus_chg_pullup_dp_set(false);
#endif
	sy6974b_get_bc12(chip);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PLUGIN);
	return;
}

static void sy6974b_bc12_boot_check(struct sy6974b_chip *chip)
{
	int data;
	int rc;

	/* set vindpm thr to 4V */
	rc = sy6974b_write_byte_mask(chip, VINDPM_OVP_CONFIG_REG, VINDPM_CONFIG_BIT, 0x01);
	if (rc < 0)
		chg_info("set vindpm thr error, rc=%d\n", rc);
	rc = sy6974b_write_byte_mask(
		chip, IRQ_MASK_REG, VINDPM_IRQ_MASK_BIT | IINDPM_IRQ_MASK_BIT,
		VINDPM_IRQ_MASK_BIT | IINDPM_IRQ_MASK_BIT);
	if (rc < 0)
		chg_info("disable vindpm & iindpm interrupt error, rc=%d\n", rc);

	rc = sy6974b_write_byte_mask(chip, WATCHGDOG_CONFIG_REG, WATCHGDOG_CONFIG_BIT, 0x00);
	if (rc < 0)
		chg_info("disable watchdog error, rc=%d\n", rc);

	rc = sy6974b_write_byte_mask(chip, CHG_CONFIG_REG, CHG_EN_BIT, 0x00);
	if (rc < 0)
		chg_info("disable charge error, rc=%d\n", rc);

	rc = sy6974b_read_byte_mask(chip, BC12_RESULT_REG, BC12_RESULT_BIT, &data);
	if (rc < 0) {
		chg_info("can't read charge type, rc=%d\n", rc);
		data = 0;
	}
	sy6974b_enable_irq(chip, true);

	/* BC1.2 result bit is bit5-7*/
	data = data >> 5;
	chg_info("chg_type=%u\n", data);
	if (data == CHARGE_TYPE_CDP) {
		chg_info("bc1.2 result is no input\n");
		chip->charge_type = CHARGE_TYPE_CDP;
		chip->bc12_complete = true;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_BC12_COMPLETED);
		return;
	}

	chip->bc12_retry = false;
	WRITE_ONCE(chip->auto_bc12, false);
	rc = sy6974b_write_byte_mask(chip, BC12_REG, BC12_RERUN_BIT, BC12_RERUN_BIT);
	if (rc < 0)
		chg_info("can't rerun bc1.2, rc=%d", rc);
}

static void sy6974b_bc12_timeout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sy6974b_chip *chip = container_of(dwork, struct sy6974b_chip, bc12_timeout_work);
	int rc;

	chg_info("BC1.2 check timeout\n");
	rc = sy6974b_write_byte_mask(chip, HIZ_MODE_REG, HIZ_MODE_BIT, HIZ_MODE_BIT);
	if (rc < 0)
		chg_err("can't enable hiz mode, rc=%d\n", rc);
}

int sy6974b_kick_wdt(struct sy6974b_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	chg_info("sy6974b_kick_wdt\n");
	rc = sy6974b_write_byte_mask(chip, REG01_SY6974B_ADDRESS,
					REG01_SY6974B_WDT_TIMER_RESET_MASK,
					REG01_SY6974B_WDT_TIMER_RESET);
	if (rc)
		chg_err("Couldn't sy6974b kick wdt rc = %d\n", rc);

	return rc;
}

static int sy6974b_set_wdt_timer(struct sy6974b_chip *chip, int reg)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	sy6974b_kick_wdt(chip);

	rc = sy6974b_write_byte_mask(chip, REG05_SY6974B_ADDRESS,
					REG05_SY6974B_WATCHDOG_TIMER_MASK,
					0);
	if (rc)
		chg_err("Couldn't set recharging threshold rc = %d\n", rc);

	return 0;
}

static irqreturn_t sy6974b_event_handler(int irq, void *dev_id)
{
	struct sy6974b_chip *chip = dev_id;

	chg_info("sy6974b event irq\n");
	schedule_delayed_work(&chip->event_work, 0);
	return IRQ_HANDLED;
}

struct oplus_chg_ic_virq sy6974b_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_PLUGIN },
	{ .virq_id = OPLUS_IC_VIRQ_CHG_TYPE_CHANGE },
	{ .virq_id = OPLUS_IC_VIRQ_BC12_COMPLETED },
};

static int sy6974b_init(struct oplus_chg_ic_dev *ic_dev)
{
	ic_dev->online = true;
	return 0;
}

static int sy6974b_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	return 0;
}

static int sy6974b_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct sy6974b_chip *chip;
	u8 buf[REG_MAX + 1];
	int rc;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	mutex_lock(&chip->i2c_lock);
	rc = regmap_bulk_read(chip->regmap, 0x00, buf, ARRAY_SIZE(buf));
	mutex_unlock(&chip->i2c_lock);
	if (rc < 0) {
		chg_err("can't dump register, rc=%d", rc);
		return rc;
	}
	print_hex_dump(KERN_ERR, "OPLUS_CHG[SY6974B]: ", DUMP_PREFIX_OFFSET,
		       32, 1, buf, ARRAY_SIZE(buf), false);
	return 0;
}

static int sy6974b_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int sy6974b_input_present(struct oplus_chg_ic_dev *ic_dev, bool *present)
{
	struct sy6974b_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*present = chip->vbus_present;

	return 0;
}

static int sy6974b_get_charger_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct sy6974b_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	chg_info("oplus_charger_type = %d\n", chip->oplus_charger_type);

	switch (chip->oplus_charger_type) {
	case POWER_SUPPLY_TYPE_USB:
		*type = OPLUS_CHG_USB_TYPE_SDP;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		*type = OPLUS_CHG_USB_TYPE_CDP;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		*type = OPLUS_CHG_USB_TYPE_DCP;
		break;
	default:
		*type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		break;
	}
	return 0;
}

static int sy6974b_rerun_bc12(struct oplus_chg_ic_dev *ic_dev)
{
	struct sy6974b_chip *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chg_info("rerun bc1.2\n");
	sy6974b_request_dpdm(chip, true);
	/* no need to retry */
	chip->bc12_retry = true;
	chip->auto_bc12 = false;
	chip->bc12_complete = false;
	rc = sy6974b_write_byte_mask(chip, BC12_REG, BC12_RERUN_BIT, BC12_RERUN_BIT);
	if (rc < 0) {
		chg_err("can't rerun bc1.2, rc=%d", rc);
		goto err;
	}

	return 0;

err:
	chip->bc12_complete = true;
	return rc;
}

static int sy6974b_disable_vbus(struct oplus_chg_ic_dev *ic_dev, bool en, bool delay)
{
	struct sy6974b_chip *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chip->otg_enable = en;
	mutex_lock(&chip->pinctrl_lock);
	if (en)
		rc = pinctrl_select_state(chip->pinctrl, chip->dis_vbus_active);
	else {
		/* Wait for VBUS to be completely powered down, usually 20ms */
		if (delay)
			msleep(20);
		rc = pinctrl_select_state(chip->pinctrl, chip->dis_vbus_sleep);
	}
	mutex_unlock(&chip->pinctrl_lock);
	if (rc < 0)
		chg_info("can't set disable vbus gpio to %s, rc=%d\n", en ? "active" : "sleep", rc);
	else
		chg_info("set disable vbus gpio to %s\n", en ? "active" : "sleep");

	return rc;
}

int sy6974b_input_current_limit_without_aicl(struct sy6974b_chip *chip, int current_ma)
{
	int rc = 0;
	int tmp = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1) {
		chg_err("in suspend\n");
		return 0;
	}

	if (current_ma > REG00_SY6974B_INPUT_CURRENT_LIMIT_MAX)
		current_ma = REG00_SY6974B_INPUT_CURRENT_LIMIT_MAX;

	if (current_ma < REG00_SY6974B_INPUT_CURRENT_LIMIT_OFFSET)
		current_ma = REG00_SY6974B_INPUT_CURRENT_LIMIT_OFFSET;

	tmp = (current_ma - REG00_SY6974B_INPUT_CURRENT_LIMIT_OFFSET) /
		REG00_SY6974B_INPUT_CURRENT_LIMIT_STEP;
	chg_info("tmp current [%d]ma\n", current_ma);
	rc = sy6974b_write_byte_mask(chip, REG00_SY6974B_ADDRESS,
			REG00_SY6974B_INPUT_CURRENT_LIMIT_MASK,
			tmp << REG00_SY6974B_INPUT_CURRENT_LIMIT_SHIFT);

	if (rc < 0)
		chg_err("Couldn't set aicl rc = %d\n", rc);

	return rc;
}

static void sy6974b_fcc_vote_work(struct work_struct *work)
{
	struct sy6974b_chip *chip = container_of(work, struct sy6974b_chip, fcc_vote_work);
	union mms_msg_data data = { 0 };
	int max_curr = 0;
	bool chg_online = 0;
	int wire_type = 0;
	int rc = 0;

	if (IS_ERR_OR_NULL(chip->fcc_votable))
		chip->fcc_votable = find_votable("WIRED_FCC");

	if (chip->wired_topic) {
		rc = oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE, &data, false);
		if (rc >= 0)
			wire_type = data.intval;

		rc = oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data, false);
		if (rc >= 0)
			chg_online = !!data.intval;

		if (chg_online == true) {
			if (wire_type == OPLUS_CHG_USB_TYPE_PD_SDP) {
				rc = oplus_mms_get_item_data(chip->wired_topic,
					WIRED_ITEM_CHARGER_CURR_MAX, &data, false);
				if (rc >= 0)
					max_curr = data.intval;

				if (max_curr > 0 && !IS_ERR_OR_NULL(chip->fcc_votable))
					vote(chip->fcc_votable, IC_VOTER, true, max_curr, false);
			} else {
				if (!IS_ERR_OR_NULL(chip->fcc_votable))
					vote(chip->fcc_votable, IC_VOTER, false, 0, false);
			}
		} else {
			if (!IS_ERR_OR_NULL(chip->fcc_votable))
				vote(chip->fcc_votable, IC_VOTER, true, DISCONNECT_FCC_MAX_CURR, false);
		}
	}
}

static void sy6974b_wired_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct sy6974b_chip *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_OTG_ENABLE:
			schedule_work(&chip->otg_enabled_work);
			break;
		case WIRED_ITEM_CHG_TYPE:
		case WIRED_ITEM_ONLINE:
			schedule_work(&chip->fcc_vote_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void sy6974b_cpa_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct sy6974b_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case CPA_ITEM_ALLOW:
			oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, true);
			chip->cpa_current_type = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void sy6974b_subscribe_cpa_topic(struct oplus_mms *topic, void *prv_data)
{
	struct sy6974b_chip *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->cpa_topic = topic;
	chip->cpa_subs = oplus_mms_subscribe(chip->cpa_topic, chip,
		sy6974b_cpa_subs_callback, "sy6974b");
	if (IS_ERR_OR_NULL(chip->cpa_subs)) {
		chg_err("subscribe cpa topic error, rc=%ld\n", PTR_ERR(chip->cpa_subs));
		return;
	}

	oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, true);
	chip->cpa_current_type = data.intval;
}

static void sy6974b_subscribe_wired_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct sy6974b_chip *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs = oplus_mms_subscribe(chip->wired_topic,
				chip, sy6974b_wired_subs_callback, "sy6974b");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n", PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_PRESENT, &data, true);
	chip->vbus_present = !!data.intval;
	if (chip->vbus_present && !chip->otg_enable) {
		sy6974b_request_dpdm(chip, true);
		sy6974b_bc12_boot_check(chip);
	}
}

static int sy6974b_otg_boost_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc;
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (en)
		rc = sy6974b_otg_enable(chip);
	else
		rc = sy6974b_otg_disable(chip);
	if (rc < 0)
		chg_err("can't %s otg boost, rc=%d\n", en ? "enable" : "disable", rc);

	chg_info("otg boost, rc=%s\n", en ? "enable" : "disable");
	return 0;
}

static bool sy6974b_check_really_suspend_charger(struct sy6974b_chip *chip)
{
	int rc = 0;
	int reg_val = 0;
	bool hiz = false;

	if (!chip)
		return false;

	if (atomic_read(&chip->driver_suspended) == 1)
		return false;

	rc = sy6974b_read_byte(chip, REG00_SY6974B_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't oplus_chg_is_usb_present rc = %d\n", rc);
		return false;
	}

	hiz = ((reg_val & REG00_SY6974B_SUSPEND_MODE_ENABLE) == REG00_SY6974B_SUSPEND_MODE_ENABLE) ? 1 : 0;
	return hiz;
}

static void sy6974b_really_suspend_charger(struct sy6974b_chip *chip, bool en)
{
	int rc = 0;

	if (!chip) {
		return;
	}

	if (atomic_read(&chip->driver_suspended) == 1) {
		return;
	}

	rc = sy6974b_write_byte_mask(chip, REG00_SY6974B_ADDRESS,
			REG00_SY6974B_SUSPEND_MODE_MASK,
			en ? REG00_SY6974B_SUSPEND_MODE_ENABLE : REG00_SY6974B_SUSPEND_MODE_DISABLE);
	if (rc < 0) {
		chg_err("fail en=%d rc = %d\n", en, rc);
	}
}

int sy6974b_suspend_charger_input(struct sy6974b_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	atomic_set(&chip->charger_suspended, 1);

	sy6974b_really_suspend_charger(chip, true);
	return rc;
}

int sy6974b_unsuspend_charger_input(struct sy6974b_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	sy6974b_really_suspend_charger(chip, false);

	return rc;
}

static int sy6974b_input_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	int rc;
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (suspend) {
		rc = sy6974b_disable_charger(chip);
		rc = sy6974b_suspend_charger_input(chip);
	} else {
		rc = sy6974b_unsuspend_charger_input(chip);
		rc = sy6974b_enable_charger(chip);
	}

	chg_info("charger input %s, rc = %d\n", suspend ? "suspend" : "unsuspend", rc);

	return rc;
}

int sy6974b_set_otg_voltage(struct sy6974b_chip *chip, int vol_mv)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_write_byte_mask(chip, REG06_SY6974B_ADDRESS,
		REG06_SY6974B_OTG_VLIM_MASK, vol_mv);

	return rc;
}

static int sy6974b_set_otg_boost_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	int rc;
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_set_otg_voltage(chip, vol_mv);
	if (rc < 0)
		chg_err("set otg vol err, rc=%d\n", rc);

	return rc;
}

static int sy6974b_set_otg_boost_curr_limit(struct oplus_chg_ic_dev *ic_dev, int curr_uA)
{
	int rc;
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);
	int curr_ma = 0;
	int val = REG02_SY6974B_OTG_CURRENT_LIMIT_500MA;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}
	curr_ma = curr_uA / 1000;
	if ((curr_ma/1000) >= REG02_SY6974B_BOOSTI_1200)
		val = REG02_SY6974B_OTG_CURRENT_LIMIT_1200MA;

	rc = sy6974b_otg_ilim_set(chip, val);

	if (rc < 0)
		chg_err("set otg cc err, rc=%d\n", rc);
	return rc;
}

static int sy6974b_output_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	int rc = 0;
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (suspend)
		rc = sy6974b_disable_charger(chip);
	else
		rc = sy6974b_enable_charger(chip);

	chg_info("%s: charger out %s, rc = %d", __func__, suspend ? "suspend" : "unsuspend", rc);

	return rc;
}

static int sy6974b_charging_current_write_fast(struct sy6974b_chip *chip, int chg_cur)
{
	int rc = 0;
	int tmp = 0;

	if (!chip)
		return 0;

	if(atomic_read(&chip->driver_suspended) == 1)
		return 0;

	chg_info("set charge current = %d\n", chg_cur);

	if (chg_cur > REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_MAX)
		chg_cur = REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_MAX;

	if (chg_cur < REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_OFFSET)
		chg_cur = REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_OFFSET;

	tmp = chg_cur - REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_OFFSET;
	tmp = tmp / REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_STEP;

	rc = sy6974b_write_byte_mask(chip, REG02_SY6974B_ADDRESS,
			REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_MASK,
			tmp << REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_SHIFT);

	return rc;
}

static int sy6974b_get_charging_current(struct sy6974b_chip *chip, u32 *curr)
{
	int reg_val;
	int ichg;
	int ret;

	ret = sy6974b_read_byte(chip, REG02_SY6974B_ADDRESS, &reg_val);
	if (!ret) {
		ichg = (reg_val & REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_MASK) >>
			REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_SHIFT;
		*curr = ichg * REG02_SY6974B_FAST_CHG_CURRENT_LIMIT_STEP;
	}

	return ret;
}

static int sy6974b_set_fcc(struct oplus_chg_ic_dev *ic_dev, int fcc_ma)
{
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);
	int rc = 0;
	u32 ret_chg_curr = 0;

	rc = sy6974b_charging_current_write_fast(chip, fcc_ma);
	if (rc < 0)
		chg_info("set fast charge current:%d fail\n", fcc_ma);
	else {
		sy6974b_get_charging_current(chip, &ret_chg_curr);
		chg_info("set fast charge current:%d ret_chg_curr = %d\n", fcc_ma, ret_chg_curr);
	}

	return rc;
}

static int sy6974b_set_fv(struct oplus_chg_ic_dev *ic_dev, int fv_mv)
{
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);
	int val;

	if(!chip)
		return -1;

	if (fv_mv < REG04_SY6974B_CHG_VOL_LIMIT_OFFSET)
		fv_mv = REG04_SY6974B_CHG_VOL_LIMIT_OFFSET;

	val = (fv_mv - REG04_SY6974B_CHG_VOL_LIMIT_OFFSET)/REG04_SY6974B_CHG_VOL_LIMIT_STEP;

	return sy6974b_write_byte_mask(chip, REG04_SY6974B_ADDRESS,
				REG04_SY6974B_CHG_VOL_LIMIT_MASK,
				val << REG04_SY6974B_CHG_VOL_LIMIT_SHIFT);
}

static int sy6974b_get_fv(struct oplus_chg_ic_dev *ic_dev, int *fv_mv)
{
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);
	int reg_val;
	int vchg;
	int ret;

	ret = sy6974b_read_byte(chip, REG04_SY6974B_ADDRESS, &reg_val);
	if (!ret) {
		vchg = (reg_val & REG04_SY6974B_CHG_VOL_LIMIT_MASK) >>
			REG04_SY6974B_CHG_VOL_LIMIT_SHIFT;
		*fv_mv = vchg * REG04_SY6974B_CHG_VOL_LIMIT_STEP +
			REG04_SY6974B_CHG_VOL_LIMIT_OFFSET;
	}

	return ret;
}

static int sy6974b_set_iterm(struct oplus_chg_ic_dev *ic_dev, int iterm_ma)
{
	int rc = 0;
	int tmp = 0;
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	chg_info("term_current = %d\n", iterm_ma);
	tmp = iterm_ma - REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_OFFSET;
	tmp = tmp / REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_STEP;

	rc = sy6974b_write_byte_mask(chip, REG03_SY6974B_ADDRESS,
					REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_MASK,
					tmp << REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_SHIFT);
	return 0;
}


static int sy6974b_get_input_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	*curr_ma = 0;
	return 0;
}

static int get_vbus_voltage(struct sy6974b_chip *chip, int *val)
{
	int ret = 0;
	static struct iio_channel		*chan_vbus = NULL;

	if (IS_ERR_OR_NULL(chan_vbus))
		chan_vbus = devm_iio_channel_get(chip->dev, "pmic_vcdt_voltage");

	if (!IS_ERR_OR_NULL(chan_vbus)) {
		ret = iio_read_channel_processed(chan_vbus, val);
		if (ret < 0)
			chg_err("[%s]read fail,ret=%d\n", __func__, ret);
	} else {
		chg_info("[%s]chan error chan_vbus_id\n", __func__);
		ret = -1;
	}

	*val = (((R_CHARGER_1 +
			R_CHARGER_2) * 100 * (*val)) /
			R_CHARGER_2) / 100;
	chg_info("%s get vbus voltage=%d\n", __func__, *val);
	return ret;
}

static int sy6974b_get_input_vol(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);
	get_vbus_voltage(chip, vol_mv);
	chg_debug("sy6974b_get_charger_vol: %d\n", *vol_mv);
	return 0;
}

static int sy6974b_aicl_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;
	return rc;
}

static int sy6974b_aicl_rerun(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;
	return rc;
}

int oplus_sy6974b_enter_shipmode(struct sy6974b_chip *chip, bool en)
{
	int val = 0;
	int rc = 0;

	if(!chip)
		return 0;

	chg_info("enter ship_mode:en:%d\n", en);

	if(en)
		val = SY6974_BATFET_OFF << REG07_SY6974B_BATFET_DIS_SHIFT;
	else
		val = SY6974_BATFET_ON << REG07_SY6974B_BATFET_DIS_SHIFT;
	rc = sy6974b_write_byte_mask(chip, REG07_SY6974B_ADDRESS, REG07_SY6974B_BATFET_DIS_MASK, val);

	chg_info("enter ship_mode:done\n");

	return rc;
}

static int sy6974b_shipmode_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);
	int val = 0;
	int rc = 0;

	if (chip == NULL)
		return -EINVAL;
	if (en) {
		chg_info(" enable ship mode \n");
		val = SY6974_BATFET_OFF << REG07_SY6974B_BATFET_DIS_SHIFT;
	}
	rc = sy6974b_write_byte_mask(chip, REG07_SY6974B_ADDRESS, REG07_SY6974B_BATFET_DIS_MASK, val);
	return 0;
}

static int sy6974b_get_otg_enbale(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct sy6974b_chip *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		return -EINVAL;
	}

	*enable = chip->otg_enable;
	return 0;
}

static int sy6974b_set_qc_config(struct oplus_chg_ic_dev *ic_dev,
	enum oplus_chg_qc_version version, int vol_mv)
{
	return 0;
}

static int sy6974b_set_otg_switch_status(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;
	/* TODO */
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);

	chg_info("[OPLUS_CHG][%s]: otg switch[%d]\n", __func__, en);
	if (en)
		rc = sy6974b_otg_enable(chip);
	else
		rc = sy6974b_otg_disable(chip);

	if (rc < 0)
		chg_err("can't %s otg boost, rc=%d\n", en ? "enable" : "disable", rc);

	chg_info("otg boost, rc=%s\n", en ? "enable" : "disable");

	return 0;
}

static int sy6974b_kick_wdt_func(struct oplus_chg_ic_dev *ic_dev)
{
	struct sy6974b_chip *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	sy6974b_kick_wdt(chip);
	return rc;
}

static int sy6974b_hardware_init_func(struct oplus_chg_ic_dev *ic_dev)
{
	struct sy6974b_chip *chip = oplus_chg_ic_get_drvdata(ic_dev);
	sy6974b_hardware_init(chip);
	return 0;
}

static void *oplus_chg_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
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
					    	sy6974b_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					    	sy6974b_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
					    	sy6974b_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
					    	sy6974b_smt_test);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_PRESENT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_PRESENT,
					    	sy6974b_input_present);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE,
						sy6974b_get_charger_type);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND,
					    	sy6974b_input_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_RERUN_BC12:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_RERUN_BC12,
					    	sy6974b_rerun_bc12);
		break;
	case OPLUS_IC_FUNC_DISABLE_VBUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_DISABLE_VBUS,
					    	sy6974b_disable_vbus);
		break;
	case OPLUS_IC_FUNC_OTG_BOOST_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_OTG_BOOST_ENABLE,
					    	sy6974b_otg_boost_enable);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT,
						sy6974b_set_otg_boost_curr_limit);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND,
						sy6974b_output_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FCC,
					    	sy6974b_set_fcc);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FV,
					    	sy6974b_set_fv);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_FV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_FV,
					    	sy6974b_get_fv);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ITERM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ITERM,
					    	sy6974b_set_iterm);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR,
						sy6974b_get_input_curr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL,
					    	sy6974b_get_input_vol);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_VOL,
					    	sy6974b_set_otg_boost_vol);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_ENABLE,
					    	sy6974b_aicl_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RERUN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RERUN,
					    	sy6974b_aicl_rerun);
		break;
	case OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE,
						sy6974b_shipmode_enable);
		break;
	case OPLUS_IC_FUNC_GET_OTG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_ENABLE,
					    	sy6974b_get_otg_enbale);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG,
					    	sy6974b_set_qc_config);
		break;
	case OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS,
					    	sy6974b_set_otg_switch_status);
		break;
	case OPLUS_IC_FUNC_BUCK_HARDWARE_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_HARDWARE_INIT,
						sy6974b_hardware_init_func);
		break;
	case OPLUS_IC_FUNC_BUCK_KICK_WDT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_KICK_WDT,
						sy6974b_kick_wdt_func);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static int sy6974b_gpio_init(struct sy6974b_chip *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -ENODEV;
	}

	chip->dis_vbus_active = pinctrl_lookup_state(chip->pinctrl, "dis_vbus_active");
	if (IS_ERR_OR_NULL(chip->dis_vbus_active)) {
		chg_err("get dis_vbus_active fail\n");
		goto free_dis_vbus_gpio;
	}
	chip->dis_vbus_sleep = pinctrl_lookup_state(chip->pinctrl, "dis_vbus_sleep");
	if (IS_ERR_OR_NULL(chip->dis_vbus_sleep)) {
		chg_err("get dis_vbus_sleep fail\n");
		goto free_dis_vbus_gpio;
	}
	pinctrl_select_state(chip->pinctrl, chip->dis_vbus_sleep);

	chip->event_gpio = of_get_named_gpio(node, "oplus,event-gpio", 0);
	if (!gpio_is_valid(chip->event_gpio)) {
		chg_err("event_gpio not specified\n");
		rc = -ENODEV;
		goto free_dis_vbus_gpio;
	}
	rc = gpio_request(chip->event_gpio, "sy6974b_event-gpio");
	if (rc < 0) {
		chg_err("event_gpio request error, rc=%d\n", rc);
		goto free_dis_vbus_gpio;
	}

	chip->event_default = pinctrl_lookup_state(chip->pinctrl, "event_default");
	if (IS_ERR_OR_NULL(chip->event_default)) {
		chg_err("get event_default fail\n");
		goto free_event_gpio;
	}
	gpio_direction_input(chip->event_gpio);
	pinctrl_select_state(chip->pinctrl, chip->event_default);

	chip->event_irq = gpio_to_irq(chip->event_gpio);
	rc = devm_request_irq(chip->dev, chip->event_irq,
			      sy6974b_event_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			      "sy6974b_event-irq", chip);
	if (rc < 0) {
		chg_err("event_irq request error, rc=%d\n", rc);
		goto free_event_gpio;
	}
	chip->event_irq_enabled = true;
	sy6974b_enable_irq(chip, false);

	return 0;

free_event_gpio:
	if (gpio_is_valid(chip->event_gpio))
		gpio_free(chip->event_gpio);
free_dis_vbus_gpio:
	if (gpio_is_valid(chip->dis_vbus_gpio))
		gpio_free(chip->dis_vbus_gpio);

	return rc;
}

#ifdef CONFIG_OPLUS_CHARGER_MTK

static int sy6974b_otg_ilim_set(struct sy6974b_chip *chip, int ilim)
{
	int rc;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_write_byte_mask(chip, REG02_SY6974B_ADDRESS,
			REG02_SY6974B_OTG_CURRENT_LIMIT_MASK,
			ilim);
	if (rc < 0)
		chg_err("Couldn't sy6974b_write_byte_mask  rc = %d\n", rc);

	return rc;
}

static int sy6974b_otg_enable(struct sy6974b_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	chg_err("sy6974b_otg_enable\n");

	sy6974b_set_wdt_timer(chip, REG05_SY6974B_WATCHDOG_TIMER_DISABLE);

	rc = sy6974b_otg_ilim_set(chip, REG02_SY6974B_OTG_CURRENT_LIMIT_1200MA);
	if (rc < 0)
		chg_err("Couldn't sy6974b_write_byte_mask rc = %d\n", rc);

	rc = sy6974b_write_byte_mask(chip, REG01_SY6974B_ADDRESS, REG01_SY6974B_OTG_MASK,
			REG01_SY6974B_OTG_ENABLE);
	if (rc < 0)
		chg_err("Couldn't sy6974b_otg_enable  rc = %d\n", rc);

	chip->otg_enable = TRUE;

	if (chip->chg_psy)
		power_supply_changed(chip->chg_psy);
	else
		chg_err("g_oplus_chip->chg_psy is null notify usb failed\n");
	return rc;
}

static int sy6974b_otg_disable(struct sy6974b_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	chg_err("sy6974b_otg_disable\n");

	rc = sy6974b_write_byte_mask(chip, REG01_SY6974B_ADDRESS,
			REG01_SY6974B_OTG_MASK,
			REG01_SY6974B_OTG_DISABLE);
	if (rc < 0)
		chg_err("Couldn't sy6974b_otg_disable rc = %d\n", rc);

	sy6974b_set_wdt_timer(chip, REG05_SY6974B_WATCHDOG_TIMER_DISABLE);
	chip->otg_enable = FALSE;
	if (chip->chg_psy)
		power_supply_changed(chip->chg_psy);
	else
		chg_err("g_oplus_chip->chg_psy is null notify usb failed\n");

	return rc;
}

static const struct charger_properties  sy6974b_chg_props = {
	.alias_name = "sy6974b",
};

int sy6974b_check_charging_enable(struct sy6974b_chip *chip)
{
	int rc = 0;
	int reg_val = 0;
	bool charging_enable = false;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_read_byte(chip, REG01_SY6974B_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG01_SY6974B_ADDRESS rc = %d\n", rc);
		return 0;
	}

	charging_enable = ((reg_val & REG01_SY6974B_CHARGING_MASK) == REG01_SY6974B_CHARGING_ENABLE) ? 1 : 0;

	return charging_enable;
}

static int sy6974b_enable_charger(struct sy6974b_chip *chip)
{
	int ret;

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	if (sy6974b_check_charging_enable(chip))
		return 0;

	chg_info("enable\n");
	if (chip->request_otg) {
		chg_err("suspend or camera, ignore\n");
		return 0;
	}

	ret = sy6974b_write_byte_mask(chip, REG01_SY6974B_ADDRESS,
		REG01_SY6974B_CHARGING_MASK,
		REG01_SY6974B_CHARGING_ENABLE);

	return ret;
}

static int sy6974b_disable_charger(struct sy6974b_chip *chip)
{
	int ret;

	if (!chip)
		return -EINVAL;

	chg_info("disable\n");
	ret = sy6974b_write_byte_mask(chip, REG01_SY6974B_ADDRESS,
		REG01_SY6974B_CHARGING_MASK,
		REG01_SY6974B_CHARGING_DISABLE);
	return ret;
}

static int sy6974b_plug_in(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sy6974b_chip *chip = dev_get_drvdata(&chg_dev->dev);

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	sy6974b_otg_disable(chip);
	ret = sy6974b_enable_charger(chip);
	if (ret < 0)
		chg_err("Couldn't sy6974b_disable_charging ret = %d\n", ret);

	chg_info("sy6974b_disable_charging \n");
	return ret;
}

static int sy6974b_plug_out(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sy6974b_chip *chip = dev_get_drvdata(&chg_dev->dev);

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	sy6974b_otg_disable(chip);
	ret = sy6974b_disable_charger(chip);

	if (ret)
		chg_err("failed to disable charging:%d", ret);
	return ret;
}

static int sy6974b_chgdet_en(struct charger_device *chg_dev, bool en)
{
	struct sy6974b_chip *chip = dev_get_drvdata(&chg_dev->dev);

	if (en)
		return sy6974b_enable_charger(chip);
	else
		return sy6974b_disable_charger(chip);
}


int sy6974b_set_vindpm_vol(struct sy6974b_chip *chip, int vol)
{
	int rc = 0;
	int tmp = 0;
	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	if (vol < REG06_SY6974B_VINDPM_OFFSET)
		vol = REG06_SY6974B_VINDPM_OFFSET;

	tmp = (vol - REG06_SY6974B_VINDPM_OFFSET) / REG06_SY6974B_VINDPM_STEP_MV;
	rc = sy6974b_write_byte_mask(chip, REG06_SY6974B_ADDRESS,
						REG06_SY6974B_VINDPM_MASK,
						tmp << REG06_SY6974B_VINDPM_SHIFT);
	return rc;
}

static int sy6974b_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct sy6974b_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return sy6974b_set_vindpm_vol(chip, volt/1000);
}

static int sy6974b_set_icl(struct charger_device *chg_dev, u32 curr)
{
	struct sy6974b_chip *chip = dev_get_drvdata(&chg_dev->dev);
	chip->charger_current_pre = curr/1000;
	return sy6974b_input_current_limit_without_aicl(chip, curr/1000);
}

int sy6974b_vbus_adc(struct charger_device *dev, u32 *vbus)
{
	int val = 0;
	struct sy6974b_chip *chip = dev_get_drvdata(&dev->dev);
	get_vbus_voltage(chip, &val);
	*vbus = val * 1000;
	return 0;
}

static struct charger_ops sy6974b_charger_ops = {
	.plug_in = sy6974b_plug_in,
	.plug_out = sy6974b_plug_out,
	.enable = sy6974b_chgdet_en,
	.set_mivr = sy6974b_set_ivl,
	.set_input_current = sy6974b_set_icl,
	.get_vbus_adc = sy6974b_vbus_adc,
};

static enum power_supply_usb_type sy6974b_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static enum power_supply_property sy6974b_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static int sy6974b_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct sy6974b_chip *chip = power_supply_get_drvdata(psy);
	int ret = 0;
	int boot_mode = get_boot_mode();

	if (!chip) {
		chg_info("oplus_chip not ready!\n");
		return -ENODATA;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->vbus_present;
		break;
	case POWER_SUPPLY_PROP_TYPE:
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = chip->oplus_charger_type;
		chg_info("sy6974b get power_supply_type = %d boot_mode=%d real_type=%d\n",
			val->intval, boot_mode, chip->oplus_charger_type);
		break;
	default:
		ret = -ENODATA;
	}
	return ret;
}

static char *sy6974b_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static const struct power_supply_desc sy6974b_charger_desc = {
	.type			= POWER_SUPPLY_TYPE_USB,
	.usb_types      	= sy6974b_charger_usb_types,
	.num_usb_types  	= ARRAY_SIZE(sy6974b_charger_usb_types),
	.properties 		= sy6974b_charger_properties,
	.num_properties 	= ARRAY_SIZE(sy6974b_charger_properties),
	.get_property		= sy6974b_charger_get_property,
};

static int sy6974b_chg_init_psy(struct sy6974b_chip *chip)
{
	struct power_supply_config cfg = {
		.drv_data = chip,
		.of_node = chip->dev->of_node,
		.supplied_to = sy6974b_charger_supplied_to,
		.num_supplicants = ARRAY_SIZE(sy6974b_charger_supplied_to),
	};

	memcpy(&chip->psy_desc, &sy6974b_charger_desc, sizeof(chip->psy_desc));
	chip->psy_desc.name = "charger";
	chip->chg_psy = devm_power_supply_register(chip->dev, &chip->psy_desc, &cfg);
	return IS_ERR(chip->chg_psy) ? PTR_ERR(chip->chg_psy) : 0;
}
#endif

static void sy6974b_otg_enabled_work(struct work_struct *work)
{
	struct sy6974b_chip *chip = container_of(work, struct sy6974b_chip, otg_enabled_work);
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_OTG_ENABLE, &data, false);
	chg_info("otg enable_value = %d\n", data.intval);
	if (data.intval)
		sy6974b_otg_enable(chip);
	else
		sy6974b_otg_disable(chip);
}

int sy6974b_reset_charger(struct sy6974b_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_write_byte_mask(chip, REG0B_SY6974B_ADDRESS,
					REG0B_SY6974B_REG_RST_MASK,
					REG0B_SY6974B_REG_RST_RESET);

	if (rc)
		chg_err("Couldn't sy6974b_reset_charger rc = %d\n", rc);

	return rc;
}

int sy6974b_set_stat_dis(struct sy6974b_chip *chip, bool enable)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_write_byte_mask(chip, REG00_SY6974B_ADDRESS,
			REG00_SY6974B_STAT_DIS_MASK,
			enable ? REG00_SY6974B_STAT_DIS_ENABLE : REG00_SY6974B_STAT_DIS_DISABLE);
	if (rc)
		chg_err("Couldn't sy6974b set_stat_dis rc = %d\n", rc);

	return rc;
}

int sy6974b_set_int_mask(struct sy6974b_chip *chip, int val)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_write_byte_mask(chip, REG0A_SY6974B_ADDRESS,
			REG0A_SY6974B_VINDPM_INT_MASK | REG0A_SY6974B_IINDPM_INT_MASK,
			val);
	if (rc)
		chg_err("Couldn't sy6974b set_int_mask rc = %d\n", rc);

	return rc;
}

int sy6974b_set_chg_timer(struct sy6974b_chip *chip, bool enable)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_write_byte_mask(chip, REG05_SY6974B_ADDRESS,
			REG05_SY6974B_CHG_SAFETY_TIMER_MASK,
			enable ? REG05_SY6974B_CHG_SAFETY_TIMER_ENABLE :
			REG05_SY6974B_CHG_SAFETY_TIMER_DISABLE);
	if (rc)
		chg_err("Couldn't sy6974b set_chg_timer rc = %d\n", rc);

	return rc;
}

int sy6974b_set_ovp(struct sy6974b_chip *chip, int val)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_write_byte_mask(chip, REG06_SY6974B_ADDRESS,
					REG06_SY6974B_OVP_MASK,
					val);

	return rc;
}

int sy6974b_set_chging_term_disable(struct sy6974b_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_write_byte_mask(chip, REG05_SY6974B_ADDRESS,
					REG05_SY6974B_TERMINATION_MASK,
					REG05_SY6974B_TERMINATION_DISABLE);
	if (rc)
		chg_err("Couldn't set chging term disable rc = %d\n", rc);

	return rc;
}

int sy6974b_float_voltage_write(struct sy6974b_chip *chip, int vfloat_mv)
{
	int rc = 0;
	int tmp = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	chg_info("vfloat_mv = %d\n", vfloat_mv);

	if (vfloat_mv > REG04_SY6974B_CHG_VOL_LIMIT_MAX)
		vfloat_mv = REG04_SY6974B_CHG_VOL_LIMIT_MAX;
	if (vfloat_mv < REG04_SY6974B_CHG_VOL_LIMIT_OFFSET)
		vfloat_mv = REG04_SY6974B_CHG_VOL_LIMIT_OFFSET;
	tmp = vfloat_mv - REG04_SY6974B_CHG_VOL_LIMIT_OFFSET;

	tmp = tmp / REG04_SY6974B_CHG_VOL_LIMIT_STEP;

	rc = sy6974b_write_byte_mask(chip, REG04_SY6974B_ADDRESS,
					REG04_SY6974B_CHG_VOL_LIMIT_MASK,
					tmp << REG04_SY6974B_CHG_VOL_LIMIT_SHIFT);

	return rc;
}

int sy6974b_set_prechg_voltage_threshold(struct sy6974b_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;
	chg_info("sy6974b_set_prechg_voltage_threshold\n");
	rc = sy6974b_write_byte_mask(chip, REG01_SY6974B_ADDRESS,
					REG01_SY6974B_SYS_VOL_LIMIT_MASK,
					REG01_SY6974B_SYS_VOL_LIMIT_3400MV);

	return rc;
}

int sy6974b_set_prechg_current(struct sy6974b_chip *chip, int ipre_mA)
{
	int tmp = 0;
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	tmp = ipre_mA - REG03_SY6974B_PRE_CHG_CURRENT_LIMIT_OFFSET;
	tmp = tmp / REG03_SY6974B_PRE_CHG_CURRENT_LIMIT_STEP;
	rc = sy6974b_write_byte_mask(chip, REG03_SY6974B_ADDRESS,
			REG03_SY6974B_PRE_CHG_CURRENT_LIMIT_MASK,
			(tmp + 1) << REG03_SY6974B_PRE_CHG_CURRENT_LIMIT_SHIFT);

	return 0;
}

int sy6974b_set_termchg_current(struct sy6974b_chip *chip, int term_curr)
{
	int rc = 0;
	int tmp = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	chg_info("term_current = %d\n", term_curr);
	tmp = term_curr - REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_OFFSET;
	tmp = tmp / REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_STEP;

	rc = sy6974b_write_byte_mask(chip, REG03_SY6974B_ADDRESS,
			REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_MASK,
			tmp << REG03_SY6974B_TERM_CHG_CURRENT_LIMIT_SHIFT);
	return 0;
}

int sy6974b_set_rechg_voltage(struct sy6974b_chip *chip, int recharge_mv)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_write_byte_mask(chip, REG04_SY6974B_ADDRESS,
			REG04_SY6974B_RECHG_THRESHOLD_VOL_MASK,
			recharge_mv);

	if (rc)
		chg_err("Couldn't set recharging threshold rc = %d\n", rc);

	return rc;
}

static int sy6974b_batfet_reset_disable(struct sy6974b_chip *chip, bool enable)
{
	int rc = 0;
	int val = 0;

	if(enable)
		val = SY6974_BATFET_RST_DISABLE << REG07_SY6974B_BATFET_RST_EN_SHIFT;
	else
		val = SY6974_BATFET_RST_ENABLE << REG07_SY6974B_BATFET_RST_EN_SHIFT;

	rc = sy6974b_write_byte_mask(chip, REG07_SY6974B_ADDRESS,
			REG07_SY6974B_BATFET_RST_EN_MASK, val);

	return rc;
}

static int sy6974b_hardware_init(struct sy6974b_chip *chip)
{
	if (!chip)
		return false;

	chg_info("init sy6974b hardware! \n");

	/*(must be before set_vindpm_vol and set_input_current*/
	chip->hw_aicl_point = HW_AICL_POINT_VOL_5V_PHASE1;
	chip->sw_aicl_point = SW_AICL_POINT_VOL_5V_PHASE1;

	sy6974b_set_stat_dis(chip, false);
	sy6974b_set_int_mask(chip, REG0A_SY6974B_VINDPM_INT_NOT_ALLOW |
		REG0A_SY6974B_IINDPM_INT_NOT_ALLOW);

	sy6974b_set_chg_timer(chip, false);

	sy6974b_disable_charger(chip);

	sy6974b_set_ovp(chip, REG06_SY6974B_OVP_14P0V);

	sy6974b_set_chging_term_disable(chip);

	sy6974b_float_voltage_write(chip, WPC_TERMINATION_VOLTAGE);

	sy6974b_otg_ilim_set(chip, REG02_SY6974B_OTG_CURRENT_LIMIT_1200MA);

	sy6974b_set_prechg_voltage_threshold(chip);

	sy6974b_set_prechg_current(chip, WPC_PRECHARGE_CURRENT);

	sy6974b_charging_current_write_fast(chip, WPC_CHARGE_CURRENT_DEFAULT);

	sy6974b_set_termchg_current(chip, WPC_TERMINATION_CURRENT);

	sy6974b_set_rechg_voltage(chip, WPC_RECHARGE_VOLTAGE_OFFSET);

	sy6974b_set_vindpm_vol(chip, chip->hw_aicl_point);

	sy6974b_set_otg_voltage(chip, REG06_SY6974B_OTG_VLIM_5150MV);

	sy6974b_batfet_reset_disable(chip, chip->batfet_reset_disable);
	sy6974b_really_suspend_charger(chip, false);

	if (oplus_is_rf_ftm_mode()) {
		sy6974b_disable_charger(chip);
		sy6974b_suspend_charger_input(chip);
	} else {
		sy6974b_unsuspend_charger_input(chip);
		sy6974b_enable_charger(chip);
	}

	sy6974b_set_wdt_timer(chip, REG05_SY6974B_WATCHDOG_TIMER_40S);

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("suspend,ignore set current=500mA\n");
		return 0;
	} else {
		sy6974b_input_current_limit_without_aicl(chip, DEFAULT_IBUS_MA);
		chip->charger_current_pre = DEFAULT_IBUS_MA;
	}

	return true;
}


int oplus_get_otg_online_status(void)
{
	return 0;
}

bool oplus_chg_is_usb_present(struct sy6974b_chip *chip)
{
	static bool pre_vbus_status = false;

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (oplus_get_otg_online_status()) {
		chg_err("otg,return false");
		pre_vbus_status = false;
		return pre_vbus_status;
	}
#else
	if (oplus_get_otg_online_status_default()) {
		chg_err("otg,return false");
		pre_vbus_status = false;
		return pre_vbus_status;
	}
#endif

	pre_vbus_status = sy6974b_get_bus_gd(chip);
	return pre_vbus_status;
}

static int sy6974b_inform_charger_type(struct sy6974b_chip *chip)
{
	int ret = 0;
	union power_supply_propval propval = {0};

	if (!chip->psy) {
		chip->psy = power_supply_get_by_name("mtk-master-charger");
		if (IS_ERR_OR_NULL(chip->psy)) {
			chg_err("Couldn't get chip->psy");
			return -EINVAL;
		}
	}

	if (chip->power_good)
		propval.intval = 1;
	else
		propval.intval = 0;

	if (chip->psy) {
		chg_info("inform power supply online status:%d", propval.intval);
		ret = power_supply_set_property(chip->psy,
				POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret < 0) {
			chg_err("inform power supply online failed:%d", ret);
		}

		power_supply_changed(chip->psy);
	}

	if (chip->chg_psy)
		power_supply_changed(chip->chg_psy);
	return ret;
}

int sy6974b_get_vbus_stat(struct sy6974b_chip *chip)
{
	int rc = 0;
	int vbus_stat = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_read_byte(chip, REG08_SY6974B_ADDRESS, &vbus_stat);
	if (rc) {
		chg_err("Couldn't read REG08_SY6974B_ADDRESS rc = %d\n", rc);
		return 0;
	}

	vbus_stat = vbus_stat & REG08_SY6974B_VBUS_STAT_MASK;

	return vbus_stat;
}


static int sy6974b_set_iindet(struct sy6974b_chip *chip)
{
	int rc;

	if (!chip)
		return 0;

	if (atomic_read(&chip->driver_suspended) == 1)
		return 0;

	rc = sy6974b_write_byte_mask(chip, REG07_SY6974B_ADDRESS,
					REG07_SY6974B_IINDET_EN_MASK,
					REG07_SY6974B_IINDET_EN_FORCE_DET);
	if (rc < 0)
		chg_err("Couldn't set REG07_SY6974B_IINDET_EN_MASK rc = %d\n", rc);

	msleep(45); /*Modify the delay within 30ms to 50ms*/
	rc = sy6974b_write_byte_mask(chip, REG07_SY6974B_ADDRESS,
			REG07_SY6974B_IINDET_EN_MASK, REG07_SY6974B_IINDET_DIS_FORCE_DET);

	return rc;
}

static void sy6974b_start_bc12_retry(struct sy6974b_chip *chip) {
	if (!chip)
		return;

	sy6974b_set_iindet(chip);
	schedule_delayed_work(&chip->bc12_retry_work, msecs_to_jiffies(100));
}

static void sy6974b_get_bc12(struct sy6974b_chip *chip)
{
	int vbus_stat = 0;

	if (!chip)
		return;

	if (!chip->bc12_done) {
		vbus_stat = sy6974b_get_vbus_stat(chip);
		chg_info("vbus_stat 0x%x\n", vbus_stat);
		switch (vbus_stat) {
		case REG08_SY6974B_VBUS_STAT_SDP:
			if (chip->bc12_retried < OPLUS_BC12_RETRY_CNT) {
				chip->bc12_retried++;
				chg_info("bc1.2 sdp retry cnt=%d\n", chip->bc12_retried);
				sy6974b_start_bc12_retry(chip);
				break;
			}
			chip->bc12_done = true;
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_BC12_COMPLETED);
			chip->oplus_charger_type = POWER_SUPPLY_TYPE_USB;
			#ifdef CONFIG_OPLUS_CHARGER_MTK
			Charger_Detect_Release();
			sy6974b_inform_charger_type(chip);
			#else
			oplus_set_usb_props_type(chip->oplus_charger_type);
			#endif
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			break;
		case REG08_SY6974B_VBUS_STAT_CDP:
			if (chip->bc12_retried < OPLUS_BC12_RETRY_CNT) {
				chip->bc12_retried++;
				chg_info("bc1.2 cdp retry cnt=%d\n", chip->bc12_retried);
				sy6974b_start_bc12_retry(chip);
				break;
			}

			chip->bc12_done = true;
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_BC12_COMPLETED);
			chip->oplus_charger_type = POWER_SUPPLY_TYPE_USB_CDP;
			#ifdef CONFIG_OPLUS_CHARGER_MTK
			Charger_Detect_Release();
			sy6974b_inform_charger_type(chip);
			#else
			oplus_set_usb_props_type(chip->oplus_charger_type);
			#endif
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			break;
		case REG08_SY6974B_VBUS_STAT_DCP:
		case REG08_SY6974B_VBUS_STAT_OCP:
		case REG08_SY6974B_VBUS_STAT_FLOAT:
			chip->bc12_done = true;
			#ifdef CONFIG_OPLUS_CHARGER_MTK
			chip->oplus_charger_type = POWER_SUPPLY_TYPE_USB_DCP;
			chg_info("bc1.2 dcp\n");
			sy6974b_inform_charger_type(chip);
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_BC12_COMPLETED);
			#else
			if (oplus_pd_connected() && oplus_sm8150_get_pd_type() == PD_INACTIVE) {
				chg_info("pd adapter not ready sleep 300ms \n");
				msleep(300);
				if (!oplus_chg_is_usb_present(chip)) {
					sy6974b_request_dpdm(chip, false);
					chip->bc12_done = false;
					chip->bc12_retried = 0;
					chip->bc12_delay_cnt = 0;
					chip->oplus_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
					oplus_set_usb_props_type(chip->oplus_charger_type);
					chg_info("vbus not good,break\n");
					break;
				}
			}
			chip->oplus_charger_type = POWER_SUPPLY_TYPE_USB_DCP;
			oplus_set_usb_props_type(chip->oplus_charger_type);
			#endif
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			break;
		case REG08_SY6974B_VBUS_STAT_OTG_MODE:
		case REG08_SY6974B_VBUS_STAT_UNKNOWN:
		default:
			break;
		}

		chg_info("oplus_charger_type = %d\n", chip->oplus_charger_type);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
	}
}

static void sy6974b_bc12_retry_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sy6974b_chip *chip = container_of(dwork, struct sy6974b_chip, bc12_retry_work);

	do {
		if (!oplus_chg_is_usb_present(chip)) {
			chg_info("plugout during BC1.2,delay_cnt=%d,return\n", chip->bc12_delay_cnt);
			chip->bc12_delay_cnt = 0;
			return;
		}
		if (chip->bc12_delay_cnt >= OPLUS_BC12_DELAY_CNT) {
			chg_info("BC1.2 not complete delay_cnt to max\n");
			return;
		}
		chip->bc12_delay_cnt++;
		chg_info("BC1.2 not complete delay 50ms,delay_cnt=%d\n", chip->bc12_delay_cnt);
		msleep(50);
	} while (!sy6974b_get_iindet(chip));
	chg_info("BC1.2 complete,delay_cnt=%d\n", chip->bc12_delay_cnt);
	sy6974b_get_bc12(chip);
}

static void sy6974b_pre_event_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sy6974b_chip *chip = container_of(dwork, struct sy6974b_chip, pre_event_work);

	chip->vbus_present = true;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
}

static void oplus_chg_wakelock(struct sy6974b_chip *chip, bool awake)
{
	static bool pm_flag = false;

	if (!chip || !chip->suspend_ws)
		return;

	if (awake && !pm_flag) {
		pm_flag = true;
		__pm_stay_awake(chip->suspend_ws);
		chg_info("[%s] true\n", __func__);
	} else if (!awake && pm_flag) {
		__pm_relax(chip->suspend_ws);
		pm_flag = false;
		chg_info("[%s] false\n", __func__);
	}
	return;
}


static void oplus_chg_awake_init(struct sy6974b_chip *chip)
{
	if (!chip) {
		chg_info("[%s]chip is null\n", __func__);
		return;
	}
	chip->suspend_ws = NULL;
	chip->suspend_ws = wakeup_source_register(NULL, "split chg wakelock");
	return;
}

static int sy6974b_driver_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int rc = 0;
	struct sy6974b_chip *chip;
	struct device_node *node = client->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int ret = 0;

	chip = devm_kzalloc(&client->dev, sizeof(struct sy6974b_chip), GFP_KERNEL);
	if (!chip) {
		chg_err("kzalloc failed\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->i2c_lock);
	mutex_init(&chip->dpdm_lock);
	atomic_set(&chip->driver_suspended, 0);
	atomic_set(&chip->charger_suspended, 0);
	mutex_init(&chip->pinctrl_lock);
	INIT_DELAYED_WORK(&chip->event_work, sy6974b_event_work);
	INIT_WORK(&chip->otg_enabled_work, sy6974b_otg_enabled_work);
	INIT_DELAYED_WORK(&chip->bc12_timeout_work, sy6974b_bc12_timeout_work);
	INIT_DELAYED_WORK(&chip->bc12_retry_work, sy6974b_bc12_retry_work);
	INIT_DELAYED_WORK(&chip->pre_event_work, sy6974b_pre_event_work);
	INIT_WORK(&chip->fcc_vote_work, sy6974b_fcc_vote_work);

	chip->dpdm_reg = devm_regulator_get_optional(chip->dev, "dpdm");
	if (IS_ERR(chip->dpdm_reg)) {
		rc = PTR_ERR(chip->dpdm_reg);
		chg_err("Couldn't get dpdm regulator, rc=%d\n", rc);
		chip->dpdm_reg = NULL;
	}

	chip->regmap = devm_regmap_init_i2c(client, &sy6974b_regmap_config);
	if (!chip->regmap) {
		rc = -ENODEV;
		goto regmap_init_err;
	}

	if (of_property_read_string(chip->dev->of_node, "charger_name", &chip->chg_dev_name) < 0) {
		chip->chg_dev_name = "primary_chg";
		chg_err("no charger name\n");
	}

	atomic_set(&chip->driver_suspended, 0);
	rc = sy6974b_gpio_init(chip);
	if (rc < 0) {
		chg_err("gpio init error, rc=%d\n", rc);
		goto gpio_init_err;
	}

	rc = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		goto gpio_init_err;
	}
	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto gpio_init_err;
	}

	chip->batfet_reset_disable = of_property_read_bool(chip->client->dev.of_node,
					"qcom,batfet_reset_disable");
	if (of_property_read_u32(chip->client->dev.of_node, "normal-init-work-delay-ms",
					&chip->normal_init_delay_ms))
		chip->normal_init_delay_ms = INIT_WORK_NORMAL_DELAY;

	if (of_property_read_u32(chip->client->dev.of_node, "other-init-work-delay-ms",
					&chip->other_init_delay_ms))
		chip->other_init_delay_ms = INIT_WORK_OTHER_DELAY;

	chg_info("init work delay [%d %d] name:%s\n", chip->normal_init_delay_ms,
		chip->other_init_delay_ms, node->name);
	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "buck-SY6974B");
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = sy6974b_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(sy6974b_virq_table);
	ic_cfg.of_node = node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto gpio_init_err;
	}

#ifdef CONFIG_OPLUS_CHARGER_MTK
	ret = sy6974b_chg_init_psy(chip);
	if (ret < 0)
		chg_err("failed to init power supply\n");
		/* Register charger device */
	chip->chg_dev = charger_device_register(chip->chg_dev_name,
			&client->dev, chip,
			&sy6974b_charger_ops,
			&sy6974b_chg_props);

	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		chg_info("%s: register charger device  failed\n", __func__);
		ret = PTR_ERR(chip->chg_dev);
		goto reg_ic_err;
	}
#endif

	chip->power_good = false;
	chip->bc12_done = false;
	chip->bc12_retried = 0;
	chip->bc12_delay_cnt = 0;
	sy6974b_reset_charger(chip);
	sy6974b_hardware_init(chip);
	oplus_chg_awake_init(chip);
	sy6974b_set_wdt_timer(chip, REG05_SY6974B_WATCHDOG_TIMER_DISABLE);

	oplus_mms_wait_topic("wired", sy6974b_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("cpa", sy6974b_subscribe_cpa_topic, chip);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (NORMAL_BOOT == get_boot_mode())
#else
	if (MSM_BOOT_MODE__NORMAL == get_boot_mode())
#endif
		schedule_delayed_work(&chip->event_work,
			msecs_to_jiffies(chip->normal_init_delay_ms));
	else
		schedule_delayed_work(&chip->event_work,
			msecs_to_jiffies(chip->other_init_delay_ms));

	if (sy6974b_get_bus_gd(chip))
		schedule_delayed_work(&chip->pre_event_work,
			msecs_to_jiffies(PRE_EVENT_WORK_DELAY_MS));

	sy6974b_enable_irq(chip, true);
	chg_info("success\n");

	return rc;

reg_ic_err:
	devm_oplus_chg_ic_unregister(chip->dev, chip->ic_dev);
	chip->ic_dev = NULL;

	if (gpio_is_valid(chip->event_gpio))
		gpio_free(chip->event_gpio);
gpio_init_err:
regmap_init_err:
	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, chip);
	chg_info("probe error, rc=%d\n", rc);
	return rc;
}

static struct i2c_driver sy6974b_i2c_driver;

static int sy6974b_driver_remove(struct i2c_client *client)
{
	struct sy6974b_chip *chip = i2c_get_clientdata(client);

	if (gpio_is_valid(chip->event_gpio))
		gpio_free(chip->event_gpio);
	if (chip->ic_dev)
		devm_oplus_chg_ic_unregister(chip->dev, chip->ic_dev);
	chip->ic_dev = NULL;
	if (chip->suspend_ws)
		wakeup_source_unregister(chip->suspend_ws);
	chip->suspend_ws = NULL;
	if (chip->chg_dev)
		charger_device_unregister(chip->chg_dev);
	chip->chg_dev = NULL;

	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, chip);
	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int sy6974b_pm_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct sy6974b_chip *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("chip is null\n");
		return 0;
	}

	atomic_set(&chip->driver_suspended, 0);
	return 0;
}

static int sy6974b_pm_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct sy6974b_chip *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("chip is null\n");
		return 0;
	}

	atomic_set(&chip->driver_suspended, 1);
	return 0;
}

static const struct dev_pm_ops sy6974b_pm_ops = {
	.resume = sy6974b_pm_resume,
	.suspend = sy6974b_pm_suspend,
};
#else
static int sy6974b_resume(struct i2c_client *client)
{
	struct sy6974b_chip *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("chip is null\n");
		return 0;
	}

	atomic_set(&chip->driver_suspended, 0);
	return 0;
}

static int sy6974b_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct sy6974b_chip *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("chip is null\n");
		return 0;
	}

	atomic_set(&chip->driver_suspended, 1);
	return 0;
}
#endif

static void sy6974b_shutdown(struct i2c_client *client)
{
	int val = 0;
	struct sy6974b_chip *chip = i2c_get_clientdata(client);

	/*
	 * HIZ mode needs to be disabled on shutdown to ensure activation
	 * signal is available.
	 */
	if (READ_ONCE(chip->vbus_present))
		sy6974b_write_byte_mask(chip, HIZ_MODE_REG, HIZ_MODE_BIT, 0);

	if (oplus_wired_shipmode_is_enabled()) {
		chg_info(" enable ship mode \n");
		val = SY6974_BATFET_OFF << REG07_SY6974B_BATFET_DIS_SHIFT;
		sy6974b_write_byte_mask(chip, REG07_SY6974B_ADDRESS,
			REG07_SY6974B_BATFET_DIS_MASK, val);
	}
	if (chip->event_irq)
		disable_irq(chip->event_irq);
}

static const struct of_device_id sy6974b_match[] = {
	{ .compatible = "oplus,sy6974b-charger" },
	{},
};

static const struct i2c_device_id sy6974b_id[] = {
	{ "sy6974b-charger", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sy6974b_id);

static struct i2c_driver sy6974b_i2c_driver = {
	.driver		= {
		.name = "sy6974b-charger",
		.owner	= THIS_MODULE,
		.of_match_table = sy6974b_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm 	= &sy6974b_pm_ops,
#endif
	},
	.probe		= sy6974b_driver_probe,
	.remove		= sy6974b_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= sy6974b_resume,
	.suspend	= sy6974b_suspend,
#endif
	.shutdown	= sy6974b_shutdown,
	.id_table	= sy6974b_id,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
module_i2c_driver(sy6974b_i2c_driver);
#else
int sy6974b_driver_init(void)
{
	int rc;

	rc = i2c_add_driver(&sy6974b_i2c_driver);
	if (rc < 0)
		chg_err("failed to register sy6974b i2c driver, rc=%d\n", rc);
	else
		chg_debug("Success to register sy6974b i2c driver.\n");

	return rc;
}

void sy6974b_driver_exit(void)
{
	i2c_del_driver(&sy6974b_i2c_driver);
}
oplus_chg_module_register(sy6974b_driver);
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)*/
MODULE_DESCRIPTION("Driver for sy6974b charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:sy6974b-charger");
