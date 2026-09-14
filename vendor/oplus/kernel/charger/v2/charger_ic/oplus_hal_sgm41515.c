// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[SGM41515]([%s][%d]): " fmt, __func__, __LINE__

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
#include <linux/pinctrl/consumer.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/boot_mode.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/thermal.h>

#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_mms_gauge.h>
#include "oplus_hal_sgm41515.h"
#include <oplus_chg_comm.h>
#include <oplus_chg_voter.h>
#include <tcpm.h>
#include <oplus_chg_vooc.h>
#include <linux/of_platform.h>

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mtk_boot_common.h>
#include "charger_class.h"
#endif

#ifndef I2C_ERR_MAX
#define I2C_ERR_MAX 2
#endif

#define BC12_TIMEOUT_MS			msecs_to_jiffies(5000)
#define BC12_RE_CHECK_MS		msecs_to_jiffies(50)
#define BC12_HW_DET_CNT_MAX		10
#define PLUGOUT_VBUS_TH_MV		3000
#define BC12_COMPLETE_COMP_CHECK_MS	1000

#define POWER_SUPPLY_TYPE_USB_HVDCP	13
#define REG_MAX 0x0b

static atomic_t i2c_err_count;

#define CHG_NTC_TEMP_COUNT 292
static int chg_ntc_temp_table[CHG_NTC_TEMP_COUNT];

struct sgm41515_chip {
	struct i2c_client *client;
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	struct oplus_chg_ic_dev *reverse_chg_ic_dev;
#ifdef CONFIG_OPLUS_CHARGER_MTK
	const char *chg_dev_name;
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct power_supply *chg_psy;
	struct power_supply_desc psy_desc;
#endif
	struct mutex pinctrl_lock;
	struct pinctrl *pinctrl;
	struct pinctrl_state *event_default;
	struct pinctrl_state *dis_vbus_active;
	struct pinctrl_state *dis_vbus_sleep;
	struct pinctrl_state *usbtemp_dischg_enable;
	struct pinctrl_state *usbtemp_dischg_disable;
	struct mutex i2c_lock;
	struct regmap *regmap;

	struct mutex dpdm_lock;
	struct regulator *dpdm_reg;
	struct wakeup_source *suspend_ws;
	struct wakeup_source *keep_resume_ws;
	wait_queue_head_t wait;

	struct delayed_work event_work;
	struct delayed_work bc12_timeout_work;
	struct delayed_work bc12_retry_work;
	struct delayed_work bc12_plugin_work;
	struct delayed_work vbus_check_work;
	struct oplus_mms *wired_topic;
	struct oplus_mms *vooc_topic;
	struct mms_subscribe *wired_subs;

	struct iio_channel *chg_temp_adc;
	struct thermal_zone_device *tz_dev;
	int ntcctrl_gpio_amux;

	int event_gpio;
	int event_irq;
	int dis_vbus_gpio;
	int usbtemp_dischg_gpio;

	atomic_t charger_suspended;
	atomic_t is_suspended;
	atomic_t shutdown_mask;

	bool is_hvdcp;
	bool vbus_present;
	bool bc12_retry;
	bool otg_mode;
	bool auto_bc12;
	bool bc12_complete;
	bool event_irq_enabled;
	bool power_good;
	int charge_type;
	int bc12_delay_cnt;
	int hw_aicl_point;
	int sw_aicl_point;
	int pre_aicl_index;
	int part_id;
	int bc12_retried;
	int bc12_hw_detect_count;
	bool usb_aicl_enhance;
	struct votable *wired_fcc_votable;
	struct votable *wired_icl_votable;
	struct work_struct rerun_votable_work;
	struct tcpc_device *tcpc;
	struct completion resume_done;
	struct completion bc12_complete_comp;
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

struct SGM41515_VINDPM_THR {
	int vbat_thr;
	int hw_voltage;
	int sw_voltage;
};
#define AICL_POINT_INDEX_MAX 4

static struct SGM41515_VINDPM_THR sgm41515_vindpm_vol[AICL_POINT_INDEX_MAX] =
{
	{4200, 4500, 4520},
	{4350, 4600, 4620},
	{4500, 4700, 4700},
	{10000, 4700, 4750},
};

static int sgm41515_hw_init(struct sgm41515_chip *chip);
static void sgm41515_get_bc12(struct sgm41515_chip *chip);
static int sgm41515_set_wdt_timer(struct sgm41515_chip *chip, int reg);
static void sgm41515_bc12_clear_detection_status(struct sgm41515_chip *chip);
static int sgm41515_get_charger_type(struct oplus_chg_ic_dev *ic_dev, int *type);
static int sgm41515_set_aicl_point(struct oplus_chg_ic_dev *ic_dev, int vbatt);
static bool is_sgm41515_dpdm_detection_done(struct sgm41515_chip *chip);
static int sgm41515_input_current_limit_without_aicl(struct sgm41515_chip *chip, int current_ma);
#ifdef CONFIG_OPLUS_CHARGER_MTK
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern void oplus_chg_pullup_dp_set(bool is_on);
#endif

static struct regmap_config sgm41515_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static int sgm41515_reg_dump(struct oplus_chg_ic_dev *ic_dev);

static __inline__ void sgm41515_i2c_err_inc(struct sgm41515_chip *chip)
{
	if (atomic_inc_return(&i2c_err_count) > I2C_ERR_MAX) {
		atomic_set(&i2c_err_count, 0);
		oplus_chg_ic_creat_err_msg(chip->ic_dev, OPLUS_IC_ERR_I2C, 0,
					   "continuous error");
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
	}
}

static __inline__ void sgm41515_i2c_err_clr(void)
{
	atomic_set(&i2c_err_count, 0);
}

static void sgm41515_enable_irq(struct sgm41515_chip *chip, bool en)
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

static int _sgm41515_read_byte(struct sgm41515_chip *chip, u8 addr, u8 *data)
{
	int rc;
	bool is_err = false;
	int retry = 3;

	do {
		if (is_err)
			usleep_range(5000, 5000);

		rc = i2c_master_send(chip->client, &addr, 1);
		if (rc < 1) {
			chg_err("read 0x%02x error, rc=%d\n", addr, rc);
			rc = rc < 0 ? rc : -EIO;
			is_err = true;
			continue;
		}

		rc = i2c_master_recv(chip->client, data, 1);
		if (rc < 1) {
			chg_err("read 0x%02x error, rc=%d\n", addr, rc);
			rc = rc < 0 ? rc : -EIO;
			is_err = true;
			continue;
		}
		is_err = false;
	} while (is_err && retry--);

	if (is_err)
		goto error;

	sgm41515_i2c_err_clr();
	return 0;

error:
	sgm41515_i2c_err_inc(chip);
	return rc;
}

static int sgm41515_read_byte(struct sgm41515_chip *chip, u8 addr, u8 *data)
{
	int rc = 0;
	mutex_lock(&chip->i2c_lock);
	rc = _sgm41515_read_byte(chip, addr, data);
	mutex_unlock(&chip->i2c_lock);
	return rc;
}

__maybe_unused static int sgm41515_read_data(struct sgm41515_chip *chip,
					     u8 addr, u8 *buf, int len)
{
	int rc;
	bool is_err = false;
	int retry = 3;

	mutex_lock(&chip->i2c_lock);
	do {
		if (is_err)
			usleep_range(5000, 5000);

		rc = i2c_master_send(chip->client, &addr, 1);
		if (rc < 1) {
			chg_err("read 0x%02x error, rc=%d\n", addr, rc);
			rc = rc < 0 ? rc : -EIO;
			is_err = true;
			continue;
		}

		rc = i2c_master_recv(chip->client, buf, len);
		if (rc < len) {
			chg_err("read 0x%02x error, rc=%d\n", addr, rc);
			rc = rc < 0 ? rc : -EIO;
			is_err = true;
			continue;
		}
		is_err = false;
	} while (is_err && retry--);

	if (is_err)
		goto error;

	mutex_unlock(&chip->i2c_lock);
	sgm41515_i2c_err_clr();
	return 0;

error:
	mutex_unlock(&chip->i2c_lock);
	sgm41515_i2c_err_inc(chip);
	return rc;
}

static int _sgm41515_write_byte(struct sgm41515_chip *chip, u8 addr, u8 data)
{
	u8 buf_temp[2] = { addr, data };
	int rc;
	bool is_err = false;
	int retry = 3;

	do {
		if (is_err)
			usleep_range(5000, 5000);

		rc = i2c_master_send(chip->client, buf_temp, 2);
		if (rc < 2) {
			chg_err("write 0x%02x error, rc=%d\n", addr, rc);
			rc = rc < 0 ? rc : -EIO;
			is_err = true;
			continue;
		}
		is_err = false;
	} while (is_err && retry--);

	if (is_err)
		goto error;

	sgm41515_i2c_err_clr();
	return 0;

error:
	sgm41515_i2c_err_inc(chip);
	return rc;
}

__maybe_unused static int sgm41515_write_byte(struct sgm41515_chip *chip, u8 addr, u8 data)
{
	int rc = 0;
	mutex_lock(&chip->i2c_lock);
	rc = _sgm41515_write_byte(chip, addr, data);
	mutex_unlock(&chip->i2c_lock);
	return rc;
}

__maybe_unused static int sgm41515_write_data(struct sgm41515_chip *chip,
					      u8 addr, u8 *buf, int len)
{
	u8 *buf_temp;
	int i;
	int rc;
	bool is_err = false;
	int retry = 3;

	buf_temp = kzalloc(len + 1, GFP_KERNEL);
	if (!buf_temp) {
		chg_err("alloc memary error\n");
		return -ENOMEM;
	}

	buf_temp[0] = addr;
	for (i = 0; i < len; i++)
		buf_temp[i + 1] = buf[i];

	mutex_lock(&chip->i2c_lock);
	do {
		if (is_err)
			usleep_range(5000, 5000);

		rc = i2c_master_send(chip->client, buf_temp, len + 1);
		if (rc < (len + 1)) {
			chg_err("write 0x%02x error, rc=%d\n", addr, rc);
			rc = rc < 0 ? rc : -EIO;
			is_err = true;
			continue;
		}
		is_err = false;
	} while (is_err && retry--);

	if (is_err)
		goto error;

	mutex_unlock(&chip->i2c_lock);
	kfree(buf_temp);
	sgm41515_i2c_err_clr();
	return 0;

error:
	mutex_unlock(&chip->i2c_lock);
	kfree(buf_temp);
	sgm41515_i2c_err_inc(chip);
	return rc;
}

__maybe_unused static int sgm41515_read_byte_mask(struct sgm41515_chip *chip,
						  u8 addr, u8 mask, u8 *data)
{
	u8 temp;
	int rc;

	rc = sgm41515_read_byte(chip, addr, &temp);
	if (rc < 0)
		return rc;

	*data = mask & temp;

	return 0;
}

__maybe_unused static int sgm41515_write_byte_mask(struct sgm41515_chip *chip,
						   u8 addr, u8 mask, u8 data)
{
	u8 temp;
	int rc;

	mutex_lock(&chip->i2c_lock);
	rc = _sgm41515_read_byte(chip, addr, &temp);

	if (rc < 0) {
		chg_err("read addr = %u failed, rc=%d\n", addr, rc);
		goto error;
	}
	temp = (data & mask) | (temp & (~mask));
	rc = _sgm41515_write_byte(chip, addr, temp);

	if (rc < 0) {
		chg_err("write addr = %u failed, rc=%d\n", addr, rc);
		goto error;
	}

	mutex_unlock(&chip->i2c_lock);
	chg_debug("sgm41515_write_byte_mask write:  addr = %u, temp = %u\n", addr, temp);
	return 0;

error:
	mutex_unlock(&chip->i2c_lock);
	return rc;
}

static int sgm41515_request_dpdm(struct sgm41515_chip *chip, bool enable)
{
	int rc = 0;

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (enable) {
		if (get_boot_mode() != META_BOOT)
			Charger_Detect_Init();
	} else {
		Charger_Detect_Release();
		if (chip->charge_type == POWER_SUPPLY_TYPE_USB_CDP)
			oplus_chg_pullup_dp_set(true);
		else
			oplus_chg_pullup_dp_set(false);
	}
#else
	/* fetch the DPDM regulator */
	if (!chip->dpdm_reg &&
	    of_get_property(chip->dev->of_node, "dpdm-supply", NULL)) {
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
#endif
	return rc;
}

static int sgm41515_enable_hiz_mode(struct sgm41515_chip *chip, bool en)
{
	int rc;

	if(atomic_read(&chip->shutdown_mask) == 1){
		chg_err("shutdown_mask=1, system is in shutdown state, skip hiz mode setting.\n");
		return 0;
	}

	rc = sgm41515_write_byte_mask(chip, REG00_SGM41515_ADDRESS, REG00_SGM41515_HIZ_MODE_MASK,
				      en ? REG00_SGM41515_HIZ_MODE_ENABLE : REG00_SGM41515_HIZ_MODE_DISABLE);
	if (rc < 0) {
		chg_err("can't %s hiz mode, rc=%d\n",
			en ? "enable" : "disable", rc);
		return rc;
	}

	return 0;
}

static void sgm41515_bc12_boot_check(struct sgm41515_chip *chip)
{
	int rc;

	/* set vindpm thr to 4V */
	rc = sgm41515_write_byte_mask(chip, REG06_SGM41515_ADDRESS,
						REG06_SGM41515_VINDPM_MASK, 0x01);
	if (rc < 0)
		chg_err("set vindpm thr error, rc=%d\n", rc);
	rc = sgm41515_write_byte_mask(
		chip, REG0A_SGM41515_ADDRESS, REG0A_SGM41515_VINDPM_INT_MASK | REG0A_SGM41515_IINDPM_INT_MASK,
		REG0A_SGM41515_VINDPM_INT_NOT_ALLOW | REG0A_SGM41515_IINDPM_INT_NOT_ALLOW);
	if (rc < 0)
		chg_err("disable vindpm & iindpm interrupt error, rc=%d\n", rc);

	rc = sgm41515_write_byte_mask(chip, REG05_SGM41515_ADDRESS,
				      REG05_SGM41515_WATCHDOG_TIMER_MASK, 0x00);
	if (rc < 0)
		chg_err("disable watchdog error, rc=%d\n", rc);

	rc = sgm41515_write_byte_mask(chip, REG01_SGM41515_ADDRESS, REG01_SGM41515_CHARGING_MASK,
				      REG01_SGM41515_CHARGING_DISABLE);
	if (rc < 0)
		chg_err("disable charge error, rc=%d\n", rc);

	sgm41515_enable_irq(chip, true);

	/* BC1.2 result bit is bit5-7*/
	chip->bc12_retry = false;
	WRITE_ONCE(chip->auto_bc12, false);
	chip->bc12_retried = 0;
	chip->bc12_hw_detect_count = 0;
	schedule_delayed_work(&chip->bc12_plugin_work, BC12_RE_CHECK_MS);
}

static void sgm41515_bc12_timeout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sgm41515_chip *chip =
		container_of(dwork, struct sgm41515_chip, bc12_timeout_work);

	chg_err("BC1.2 check timeout\n");
	sgm41515_enable_hiz_mode(chip, true);
}

static bool sgm41515_get_bus_gd(struct sgm41515_chip *chip)
{
	int rc = 0;
	u8 reg_val = 0;
	bool bus_gd = false;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_read_byte(chip, REG0A_SGM41515_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read regeister, rc = %d\n", rc);
		return false;
	}

	bus_gd = ((reg_val & REG0A_SGM41515_BUS_GD_MASK) == REG0A_SGM41515_BUS_GD_YES) ? 1 : 0;
	return bus_gd;
}

static __maybe_unused bool sgm41515_get_power_gd(struct sgm41515_chip *chip)
{
	int rc = 0;
	u8 reg_val = 0;
	bool power_gd = false;

	if (!chip)
		return 0;

	if(atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_read_byte(chip, REG08_SGM41515_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't get_power_gd rc = %d\n", rc);
		return false;
	}

	power_gd = ((reg_val & REG08_SGM41515_POWER_GOOD_STAT_MASK) == REG08_SGM41515_POWER_GOOD_STAT_GOOD) ? 1 : 0;
	return power_gd;
}

static void sgm41515_dump_registers(struct sgm41515_chip *chip)
{
	int ret = 0;
	int addr = 0;
	u8 val_buf[SGM41515_REG_NUMBER] = {0x0};

	if (!chip)
		return;

	if (atomic_read(&chip->charger_suspended) == 1)
		return;

	for(addr = SGM41515_FIRST_REG; addr <= SGM41515_LAST_REG; addr++) {
		/*
		 * The register is BC1.2 detection done status.
		 * If read REG0E will reset the register.
		 */
		if (addr == REG0E_SGM41515_ADDRESS)
			continue;
		ret = sgm41515_read_byte(chip, addr, &val_buf[addr]);
		if (ret)
			chg_err("Couldn't read 0x%02x ret = %d\n", addr, ret);
	}

	chg_err("[0x%02x, 0x%02x, 0x%02x, 0x%02x], [0x%02x, 0x%02x, 0x%02x, 0x%02x], "
			"[0x%02x, 0x%02x, 0x%02x, 0x%02x], [0x%02x, 0x%02x, 0x%02x, 0x%02x]\n",
			val_buf[0], val_buf[1], val_buf[2], val_buf[3],
			val_buf[4], val_buf[5], val_buf[6], val_buf[7],
			val_buf[8], val_buf[9], val_buf[10], val_buf[11],
			val_buf[12], val_buf[13], val_buf[14], val_buf[15]);
}

static u8 sgm41515_get_vbus_stat(struct sgm41515_chip *chip)
{
	int rc = 0;
	u8 vbus_stat = 0;

	if (!chip)
		return 0;

	if(atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_read_byte(chip, REG08_SGM41515_ADDRESS, &vbus_stat);
	if (rc) {
		chg_err("Couldn't read REG08_SGM41515_ADDRESS rc = %d\n", rc);
		return 0;
	}

	vbus_stat = vbus_stat & REG08_SGM41515_VBUS_STAT_MASK;

	return vbus_stat;
}

static int sgm41515_set_iindet(struct sgm41515_chip *chip)
{
	int rc;

	if (!chip)
		return 0;

	if(atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_write_byte_mask(chip, REG07_SGM41515_ADDRESS,
			REG07_SGM41515_IINDET_EN_MASK,
			REG07_SGM41515_IINDET_EN_FORCE_DET);
	if (rc < 0)
		chg_err("Couldn't set REG07_SGM41515_IINDET_EN_MASK rc = %d\n", rc);

	return rc;
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
static int sgm41515_inform_charger_type(struct sgm41515_chip *chip)
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
		if (ret < 0)
			chg_err("inform power supply online failed:%d", ret);

		power_supply_changed(chip->psy);
	}

	if (chip->chg_psy)
		power_supply_changed(chip->chg_psy);
	return ret;
}
#endif

#define OPLUS_BC12_DELAY_CNT 	18
static void sgm41515_bc12_retry_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sgm41515_chip *chip = container_of(dwork, struct sgm41515_chip, bc12_retry_work);
	bool bc12_detect_done = is_sgm41515_dpdm_detection_done(chip);

	if (!sgm41515_get_bus_gd(chip)) {
		chg_err("plugout during BC1.2, delay_cnt=%d,return\n", chip->bc12_delay_cnt);
		chip->bc12_delay_cnt = 0;
		return;
	}

	if (chip->bc12_delay_cnt >= OPLUS_BC12_DELAY_CNT) {
		chg_err("BC1.2 not complete delay_cnt to max\n");
		return;
	}
	chip->bc12_delay_cnt++;

	if (bc12_detect_done) {
		sgm41515_input_current_limit_without_aicl(chip, REG00_SGM41515_INIT_INPUT_CURRENT_LIMIT_500MA);
		chg_err("BC1.2 complete, delay_cnt=%d\n", chip->bc12_delay_cnt);
		sgm41515_get_bc12(chip);
		sgm41515_request_dpdm(chip, false);
	} else {
		chg_err("BC1.2 not complete delay 50ms,delay_cnt=%d\n", chip->bc12_delay_cnt);
		schedule_delayed_work(&chip->bc12_retry_work, round_jiffies_relative(msecs_to_jiffies(50)));
	}
}

static void sgm41515_start_bc12_retry(struct sgm41515_chip *chip)
{
	if (!chip)
		return;

	sgm41515_request_dpdm(chip, true);
	msleep(10);
	sgm41515_bc12_clear_detection_status(chip);
	sgm41515_set_iindet(chip);

	schedule_delayed_work(&chip->bc12_retry_work, round_jiffies_relative(msecs_to_jiffies(100)));
}

static bool is_wired_icl_votable_available(struct sgm41515_chip *chip)
{
	if (!chip)
		return false;

	if (!chip->wired_icl_votable)
		chip->wired_icl_votable = find_votable("WIRED_ICL");
	return !!chip->wired_icl_votable;
}

static bool is_wired_fcc_votable_available(struct sgm41515_chip *chip)
{
	if (!chip)
		return false;

	if (!chip->wired_fcc_votable)
		chip->wired_fcc_votable = find_votable("WIRED_FCC");
	return !!chip->wired_fcc_votable;
}

static void sgm41515_rerun_votable_work(struct work_struct *work)
{
	struct sgm41515_chip *chip =
		container_of(work, struct sgm41515_chip, rerun_votable_work);

	if (is_wired_fcc_votable_available(chip))
		rerun_election(chip->wired_fcc_votable, false);
	if (is_wired_icl_votable_available(chip))
		rerun_election(chip->wired_icl_votable, true);
}

#define OPLUS_BC12_RETRY_CNT 	2
static bool sgm41515_bc12_try_retry(struct sgm41515_chip *chip)
{
	if (chip->bc12_retried < OPLUS_BC12_RETRY_CNT) {
		chip->bc12_retried++;
		chg_err("bc1.2 retry cnt=%d\n", chip->bc12_retried);
		sgm41515_start_bc12_retry(chip);
		return false;
	}
	return true;
}

static void sgm41515_bc12_complete_check(struct sgm41515_chip *chip, int charger_type)
{
	if (chip->charge_type != charger_type) {
		chip->charge_type = charger_type;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
	}

	if (chip->bc12_complete) {
		complete_all(&chip->bc12_complete_comp);
		schedule_work(&chip->rerun_votable_work);

#ifdef CONFIG_OPLUS_CHARGER_MTK
		sgm41515_inform_charger_type(chip);
#endif
		msleep(REPORT_BC12_COMPLETE_DELAY);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_BC12_COMPLETED);
	}
}

static void sgm41515_get_bc12(struct sgm41515_chip *chip)
{
	u8 vbus_stat = 0;
	int charger_type = 0;

	if (!chip)
		return;

	if (!chip->bc12_complete) {
		vbus_stat = sgm41515_get_vbus_stat(chip);
		chg_err("vbus_stat=0x%x\n", vbus_stat);
		switch (vbus_stat) {
		case REG08_SGM41515_VBUS_STAT_SDP:
			if (!sgm41515_bc12_try_retry(chip))
				break;
			chip->bc12_complete = true;
			charger_type = POWER_SUPPLY_TYPE_USB;
			break;
		case REG08_SGM41515_VBUS_STAT_CDP:
			if (!sgm41515_bc12_try_retry(chip))
				break;
			chip->bc12_complete = true;
			charger_type = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		case REG08_SGM41515_VBUS_STAT_DCP:
			chg_err("bc1.2 dcp \n");
			chip->bc12_complete = true;
			charger_type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		case REG08_SGM41515_VBUS_STAT_OCP:
		case REG08_SGM41515_VBUS_STAT_FLOAT:
			if (!sgm41515_bc12_try_retry(chip))
				break;
			chip->bc12_complete = true;
			charger_type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		case REG08_SGM41515_VBUS_STAT_UNKNOWN:
			if (!sgm41515_bc12_try_retry(chip))
				break;
			break;
		case REG08_SGM41515_VBUS_STAT_OTG_MODE:
		default:
			chg_err("bc1.2 unknown\n");
			break;
		}

		sgm41515_bc12_complete_check(chip, charger_type);
	}
}

static void oplus_chg_awake_init(struct sgm41515_chip *chip)
{
	if (!chip) {
		chg_err("chip is null\n");
		return;
	}
	chip->suspend_ws = NULL;
	chip->suspend_ws = wakeup_source_register(NULL, "split_chg_wakelock");
	return;
}

static void oplus_chg_wakelock(struct sgm41515_chip *chip, bool awake)
{
	static bool pm_flag = false;

	if (!chip || !chip->suspend_ws)
		return;

	if (awake && !pm_flag) {
		pm_flag = true;
		__pm_stay_awake(chip->suspend_ws);
		chg_info("true\n");
	} else if (!awake && pm_flag) {
		__pm_relax(chip->suspend_ws);
		pm_flag = false;
		chg_info("false\n");
	}
	return;
}

static void oplus_keep_resume_awake_init(struct sgm41515_chip *chip)
{
	if (!chip) {
		chg_err("chip is null\n");
		return;
	}
	chip->keep_resume_ws = NULL;
	chip->keep_resume_ws = wakeup_source_register(NULL, "split_chg_keep_resume");
	return;
}

static void oplus_keep_resume_wakelock(struct sgm41515_chip *chip, bool awake)
{
	static bool pm_flag = false;

	if (!chip || !chip->keep_resume_ws)
		return;

	if (awake && !pm_flag) {
		pm_flag = true;
		__pm_stay_awake(chip->keep_resume_ws);
		chg_info("true\n");
	} else if (!awake && pm_flag) {
		__pm_relax(chip->keep_resume_ws);
		pm_flag = false;
		chg_info("false\n");
	}
	return;
}

static bool is_sgm41515_dpdm_detection_done(struct sgm41515_chip *chip)
{
	bool det_done = false;
	int rc = 0;
	u8 reg07_val = 0;
	u8 reg0e_val = 0;

	if (!chip) {
		chg_err("Couldn't get chip");
		return false;
	}

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger is suspended");
		return false;
	}

	rc = sgm41515_read_byte(chip, REG07_SGM41515_ADDRESS, &reg07_val);
	if (rc) {
		chg_err("Couldn't read REG07_SGM41515_ADDRESS rc = %d\n", rc);
		return false;
	}

	rc = sgm41515_read_byte(chip, REG0E_SGM41515_ADDRESS, &reg0e_val);
	if (rc) {
		chg_err("Couldn't read REG0E_SGM41515_ADDRESS rc = %d\n", rc);
		return false;
	}

	det_done = (((reg07_val & REG07_SGM41515_IINDET_EN_MASK) == REG07_SGM41515_IINDET_EN_DET_COMPLETE) &&
		    ((reg0e_val & REG0E_SGM41515_REG_INPUT_DET_MASK) == REG0E_SGM41515_REG_INPUT_DET_MASK));

	return det_done;
}

static void sgm41515_bc12_clear_detection_status(struct sgm41515_chip *chip)
{
	is_sgm41515_dpdm_detection_done(chip);
}

static void sgm41515_bc12_plugin_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sgm41515_chip *chip = container_of(dwork, struct sgm41515_chip, bc12_plugin_work);
	bool bc12_detect_done = is_sgm41515_dpdm_detection_done(chip);

	if (chip->bc12_hw_detect_count >= BC12_HW_DET_CNT_MAX || bc12_detect_done) {
		chip->bc12_retried++;
		sgm41515_input_current_limit_without_aicl(chip, REG00_SGM41515_INIT_INPUT_CURRENT_LIMIT_500MA);
		chg_info("first bc12 is done, run bc12 retry\n");
		sgm41515_get_bc12(chip);
	} else {
		if (chip->bc12_hw_detect_count < BC12_HW_DET_CNT_MAX)
			chip->bc12_hw_detect_count++;
		chg_info("dpdm detection not done, wait next bc12 retry cycle\n");
		schedule_delayed_work(&chip->bc12_plugin_work, BC12_RE_CHECK_MS);
	}
}

static int sgm41515_get_vooc_sid(struct sgm41515_chip *chip)
{
	union mms_msg_data data = { 0 };
	int vooc_sid = 0;

	if (!chip->vooc_topic)
		chip->vooc_topic = oplus_mms_get_by_name("vooc");

	if (chip->vooc_topic) {
		oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_SID, &data, true);
		vooc_sid = (unsigned int)data.intval;
	} else {
		chg_err("vooc_topic is null\n");
		return 0;
	}

	chg_info("sgm41515_get_vooc_sid: %d\n", sid_to_adapter_chg_type(vooc_sid));
	return sid_to_adapter_chg_type(vooc_sid);
}

static void sgm41515_vbus_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sgm41515_chip *chip =
		container_of(dwork, struct sgm41515_chip, vbus_check_work);
	int chg_vol = 0;

	chg_vol = oplus_wired_get_vbus();

	if (chg_vol < PLUGOUT_VBUS_TH_MV) {
		chg_info("chg_vol=%d, detech changed, continue processing\n", chg_vol);
		chip->power_good = true;
		schedule_delayed_work(&chip->event_work, 0);
	} else {
		chg_info("chg_vol=%d, vooc started, skip irq handle\n", chg_vol);
	}
}

static bool is_tcpc_available(struct sgm41515_chip *chip)
{
	if (!chip)
		return false;

	if (!chip->tcpc)
		chip->tcpc = tcpc_dev_get_by_name("type_c_port0");
	return !!chip->tcpc;
}

static void sgm41515_plug_in_event_handle(struct sgm41515_chip *chip)
{
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PLUGIN);
	oplus_chg_wakelock(chip, true);
	sgm41515_request_dpdm(chip, true);
	chip->bc12_complete = false;
	chip->bc12_retry = 0;
	chip->bc12_delay_cnt = 0;
	sgm41515_set_wdt_timer(chip, REG05_SGM41515_WATCHDOG_TIMER_40S);
	if (chip->charge_type == POWER_SUPPLY_TYPE_UNKNOWN)
		schedule_delayed_work(&chip->bc12_plugin_work, 0);
}

static void sgm41515_plug_out_event_handle(struct sgm41515_chip *chip)
{
	if (is_tcpc_available(chip) && tcpm_inquire_typec_attach_state(chip->tcpc)== TYPEC_ATTACHED_SNK &&
	    sgm41515_get_vooc_sid(chip) == CHARGER_TYPE_VOOC) {
		schedule_delayed_work(&chip->vbus_check_work, msecs_to_jiffies(INIT_WORK_OTHER_DELAY));
		chg_info("vooc or plugout, delay vbus_check_work and skip irq handle\n");
		return;
	}
	sgm41515_set_wdt_timer(chip, REG05_SGM41515_WATCHDOG_TIMER_DISABLE);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PLUGIN);
	chip->bc12_complete = false;
	reinit_completion(&chip->bc12_complete_comp);
	chip->bc12_retried = 0;
	chip->bc12_delay_cnt = 0;
	chip->charge_type = POWER_SUPPLY_TYPE_UNKNOWN;
	sgm41515_request_dpdm(chip, false);
	chip->bc12_hw_detect_count = 0;
	sgm41515_bc12_clear_detection_status(chip);
	cancel_delayed_work(&chip->bc12_plugin_work);
	oplus_chg_wakelock(chip, false);
}

static void sgm41515_no_plug_event_handle(struct sgm41515_chip *chip)
{
	chg_err("prev_pg & now_pg is false\n");
	chip->bc12_complete = false;
	reinit_completion(&chip->bc12_complete_comp);
	chip->bc12_retried = 0;
	chip->bc12_delay_cnt = 0;
}

#define OPLUS_WAIT_RESUME_TIME	200
static void sgm41515_event_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sgm41515_chip *chip =
		container_of(dwork, struct sgm41515_chip, event_work);
	bool prev_pg = false, curr_pg = false, bus_gd = false;
	u8 reg_val = 0;
	int ret = 0;

	if (chip->otg_mode) {
		chg_info("is otg mode\n");
		return;
	}

	chg_err(" sgm41515_irq_handler:enter improve irq time\n");
	oplus_keep_resume_wakelock(chip, true);

	/*for check bus i2c/spi is ready or not*/
	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err(" sgm41515_irq_handler:suspended and wait_event_interruptible %d\n", OPLUS_WAIT_RESUME_TIME);
		wait_event_interruptible_timeout(chip->wait, atomic_read(&chip->charger_suspended) == 0, msecs_to_jiffies(OPLUS_WAIT_RESUME_TIME));
	}
	prev_pg = chip->power_good;
	ret = sgm41515_read_byte(chip, REG0A_SGM41515_ADDRESS, &reg_val);
	if (ret) {
		chg_err("SGM41515_REG_0A read failed ret[%d]\n", ret);
		oplus_keep_resume_wakelock(chip, false);
		return;
	}
	bus_gd = sgm41515_get_bus_gd(chip);
	curr_pg = bus_gd;
	if(curr_pg) {
		oplus_chg_wakelock(chip, true);
	}
	sgm41515_dump_registers(chip);

	chip->vbus_present = curr_pg;
	chip->power_good = curr_pg;

	chg_info("(%d,%d, %d, %d)\n", prev_pg, chip->power_good, curr_pg, bus_gd);

	if (!prev_pg && chip->power_good) {
		sgm41515_plug_in_event_handle(chip);
		goto POWER_CHANGE;
	} else if (prev_pg && !chip->power_good) {
		sgm41515_plug_out_event_handle(chip);
		goto POWER_CHANGE;
	} else if (!prev_pg && !chip->power_good) {
		sgm41515_no_plug_event_handle(chip);
		goto POWER_CHANGE;
	}
POWER_CHANGE:

	oplus_keep_resume_wakelock(chip, false);
}

static irqreturn_t sgm41515_event_handler(int irq, void *dev_id)
{
	struct sgm41515_chip *chip = dev_id;

	chg_info("sgm41515 event irq\n");
	schedule_delayed_work(&chip->event_work, 0);
	return IRQ_HANDLED;
}

struct oplus_chg_ic_virq sgm41515_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_PLUGIN },
	{ .virq_id = OPLUS_IC_VIRQ_CHG_TYPE_CHANGE },
	{ .virq_id = OPLUS_IC_VIRQ_BC12_COMPLETED },
};

static int sgm41515_init(struct oplus_chg_ic_dev *ic_dev)
{
	ic_dev->online = true;
	return 0;
}

static int sgm41515_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	return 0;
}

static int sgm41515_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct sgm41515_chip *chip;
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
	print_hex_dump(KERN_ERR, "OPLUS_CHG[SGM41515]: ", DUMP_PREFIX_OFFSET,
		       32, 1, buf, ARRAY_SIZE(buf), false);
	return 0;
}

static int sgm41515_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int sgm41515_input_present(struct oplus_chg_ic_dev *ic_dev, bool *present)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*present = chip->vbus_present;

	return 0;
}

static int sgm41515_set_wdt_timer(struct sgm41515_chip *chip, int reg)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_write_byte_mask(chip, REG05_SGM41515_ADDRESS,
			REG05_SGM41515_WATCHDOG_TIMER_MASK,
			reg);
	if (rc)
		chg_err("Couldn't set recharging threshold rc = %d\n", rc);

	return 0;
}

static int sgm41512sd_otg_ilim_set_current(struct sgm41515_chip *chip, int ilim)
{
	int rc;
	u8 reg_val;

	if (ilim >= REG10_SGM41512_BOOST_2000MA) {
		rc = sgm41515_write_byte_mask(chip, REG10_SGM41512_ADDRESS,
				REG10_SGM41512_OTG_CURRENT_LIMIT_MASK,
				REG10_SGM41512_OTG_CURRENT_2000MA_EN);
		if (rc) {
			chg_info("sgm41512sd set otg ilim failed\n");
			return rc;
		}
	} else {
		rc = sgm41515_write_byte_mask(chip, REG10_SGM41512_ADDRESS,
				REG10_SGM41512_OTG_CURRENT_LIMIT_MASK,
				REG10_SGM41512_OTG_CURRENT_2000MA_DIS);
		if (rc)
			return rc;
		if (ilim < REG02_SGM41515_BOOSTI_1200MA)
			reg_val = REG02_SGM41515_OTG_CURRENT_LIMIT_500MA;
		else
			reg_val = REG02_SGM41515_OTG_CURRENT_LIMIT_1200MA;
		rc = sgm41515_write_byte_mask(chip, REG02_SGM41515_ADDRESS,
				REG02_SGM41515_OTG_CURRENT_LIMIT_MASK,
				reg_val);
		if (rc)
			return rc;
	}
	return 0;
}

static int sgm41512sd_otg_ilim_set(struct sgm41515_chip *chip, int ilim)
{
	int rc;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	chg_info("set otg ilim %d\n", ilim);
	rc = sgm41512sd_otg_ilim_set_current(chip, ilim);
	if (rc < 0) {
		chg_err("sgm41512sd set otg ilim error!\n");
		return rc;
	}

	rc = sgm41515_write_byte_mask(chip, REG10_SGM41512_ADDRESS,
			REG10_SGM41512_OTG_FREQ_MASK,
			REG10_SGM41512_OTG_FREQ_1_5MHZ);
	if (rc) {
		chg_err("set otg freq error!\n");
		return rc;
	}

	return 0;
}

static int sgm41515_otg_ilim_set(struct sgm41515_chip *chip, int ilim)
{
	int rc;
	u8 reg_val;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;
	if (chip->part_id == SGM41512SD_PART_ID) {
		rc = sgm41512sd_otg_ilim_set(chip, ilim);
	} else {
		if (ilim < REG02_SGM41515_BOOSTI_1200MA)
			reg_val = REG02_SGM41515_OTG_CURRENT_LIMIT_500MA;
		else
			reg_val = REG02_SGM41515_OTG_CURRENT_LIMIT_1200MA;

		rc = sgm41515_write_byte_mask(chip, REG02_SGM41515_ADDRESS,
				REG02_SGM41515_OTG_CURRENT_LIMIT_MASK,
				reg_val);
	}
	if (rc < 0)
		chg_err("Couldn't sgm41515_otg_ilim_set  rc = %d\n", rc);

	return rc;
}

static int sgm41515_wait_for_resume(struct sgm41515_chip *chip)
{
	unsigned long timeout = 0;

	if (atomic_read(&chip->charger_suspended) != 1)
		return 0;

	timeout = wait_for_completion_timeout(&chip->resume_done, msecs_to_jiffies(OPLUS_WAIT_RESUME_TIME));
	if (timeout == 0) {
		chg_err("Timeout (%dms) waiting for resume signal\n", OPLUS_WAIT_RESUME_TIME);
		return -ETIMEDOUT;
	}

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("Signal received, but charger_suspended is still 1 (Race condition?)\n");
		return -EAGAIN;
	}

	chg_info("Resumed successfully, waited %dms\n", OPLUS_WAIT_RESUME_TIME - jiffies_to_msecs(timeout));
	return 0;
}

static int sgm41515_otg_enable(struct sgm41515_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	rc = sgm41515_wait_for_resume(chip);
	if (rc < 0) {
		chg_err("System suspended, I2C transfer rejected, rc=%d\n", rc);
		return rc;
	}

	rc = sgm41515_write_byte_mask(chip, REG01_SGM41515_ADDRESS,
			REG01_SGM41515_OTG_MASK,
			REG01_SGM41515_OTG_ENABLE);
	if (rc < 0)
		chg_err("Couldn't sgm41515_otg_enable  rc = %d\n", rc);

	if (chip->part_id == SGM41515D_PART_ID) {
		chg_err("Set otg_freq to 1.5MHZ\n");
		rc = sgm41515_write_byte_mask(chip, REG0D_SGM41515_ADDRESS,
				REG0D_SGM41515_OTG_CTRL_MASK,
				REG0D_SGM41515_OTG_CTRL_DISABLE);
		if (rc < 0)
			chg_err("Couldn't set otg_freq to 1.5MHZ rc = %d\n", rc);
	}


	if (chip->part_id == SGM41512SD_PART_ID) {
		rc = sgm41515_write_byte_mask(chip, REG10_SGM41512_ADDRESS,
			REG10_SGM41512_OTG_FREQ_MASK,
			REG10_SGM41512_OTG_FREQ_1_5MHZ);
		if (rc < 0) {
			chg_err("Couldn't set OTG_FREQ to 1.5MHZ rc = %d\n", rc);
			return rc;
		}
	}
	return rc;
}

static int sgm41515_otg_disable(struct sgm41515_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	rc = sgm41515_wait_for_resume(chip);
	if (rc < 0) {
		chg_err("System suspended, I2C transfer rejected, rc=%d\n", rc);
		return rc;
	}

	rc = sgm41515_write_byte_mask(chip, REG01_SGM41515_ADDRESS,
			REG01_SGM41515_OTG_MASK,
			REG01_SGM41515_OTG_DISABLE);
	if (rc < 0) {
		chg_err("Couldn't sgm41515_otg_disable rc = %d\n", rc);
		return rc;
	}

	if (chip->part_id == SGM41515D_PART_ID) {
		chg_err("Set otg_freq to 500KHZ\n");
		rc = sgm41515_write_byte_mask(chip, REG0D_SGM41515_ADDRESS,
				REG0D_SGM41515_OTG_CTRL_MASK,
				REG0D_SGM41515_OTG_CTRL_ENABLE);
		if (rc < 0)
			chg_err("Couldn't set otg_freq to 500KHZ rc = %d\n", rc);
	}

	if (chip->part_id == SGM41512SD_PART_ID) {
		int temp_rc = sgm41515_write_byte_mask(chip, REG10_SGM41512_ADDRESS,
			REG10_SGM41512_OTG_FREQ_MASK,
			REG10_SGM41512_OTG_FREQ_500KHZ);
		if (temp_rc < 0) {
			chg_err("Couldn't set OTG_FREQ to 500KHZ rc = %d\n", temp_rc);
			rc = temp_rc;
		}
	}
	return rc;
}

static int sgm41515_enable_gpio(struct sgm41515_chip *chip, bool enable)
{
	if (enable) {
		if (gpio_is_valid(chip->dis_vbus_gpio))
			gpio_direction_output(chip->dis_vbus_gpio, 0);
	} else {
		if (gpio_is_valid(chip->dis_vbus_gpio))
			gpio_direction_output(chip->dis_vbus_gpio, 1);
	}
	return 0;
}

static int sgm41515_enable_charging(struct sgm41515_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	sgm41515_enable_gpio(chip, true);
	rc = sgm41515_write_byte_mask(chip, REG01_SGM41515_ADDRESS,
			REG01_SGM41515_CHARGING_MASK,
			REG01_SGM41515_CHARGING_ENABLE);
	if (rc < 0)
		chg_err("Couldn't sgm41515_enable_charging rc = %d\n", rc);

	chg_info("sgm41515_enable_charging \n");
	return rc;
}

static int sgm41515_disable_charging(struct sgm41515_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	sgm41515_enable_gpio(chip, false);
	rc = sgm41515_write_byte_mask(chip, REG01_SGM41515_ADDRESS,
			REG01_SGM41515_CHARGING_MASK,
			REG01_SGM41515_CHARGING_DISABLE);
	if (rc < 0)
		chg_err("Couldn't sgm41515_disable_charging rc = %d\n", rc);

	chg_info("sgm41515_disable_charging \n");
	return rc;
}

static int sgm41515_get_usb_icl(struct sgm41515_chip *chip)
{
	int rc = 0;
	int icl_ma = 0;
	u8 reg_val = 0;

	if (!chip)
		return 0;

	rc = sgm41515_read_byte(chip, REG00_SGM41515_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG00_SGM41515_ADDRESS rc = %d\n", rc);
		return 0;
	}
	icl_ma = (reg_val & REG00_SGM41515_INPUT_CURRENT_LIMIT_MASK) >> REG00_SGM41515_INPUT_CURRENT_LIMIT_SHIFT;
	icl_ma = (icl_ma * REG00_SGM41515_INPUT_CURRENT_LIMIT_STEP + REG00_SGM41515_INPUT_CURRENT_LIMIT_OFFSET);
	return icl_ma;
}

static int sgm41515_input_current_limit_without_aicl(struct sgm41515_chip *chip, int current_ma)
{
	int rc = 0;
	int tmp = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("in suspend\n");
		return 0;
	}

	if (current_ma > REG00_SGM41515_INPUT_CURRENT_LIMIT_MAX)
		current_ma = REG00_SGM41515_INPUT_CURRENT_LIMIT_MAX;

	if (current_ma < REG00_SGM41515_INPUT_CURRENT_LIMIT_OFFSET)
		current_ma = REG00_SGM41515_INPUT_CURRENT_LIMIT_OFFSET;

	tmp = (current_ma - REG00_SGM41515_INPUT_CURRENT_LIMIT_OFFSET) / REG00_SGM41515_INPUT_CURRENT_LIMIT_STEP;
	chg_err("tmp current [%d]ma\n", current_ma);
	rc = sgm41515_write_byte_mask(chip, REG00_SGM41515_ADDRESS,
			REG00_SGM41515_INPUT_CURRENT_LIMIT_MASK,
			tmp << REG00_SGM41515_INPUT_CURRENT_LIMIT_SHIFT);

	if (rc < 0)
		chg_err("Couldn't set aicl rc = %d\n", rc);

	return rc;
}

static int sgm41515_set_vindpm_vol(struct sgm41515_chip *chip)
{
	int rc = 0;
	int tmp = 0;
	int vindpm = 0;
	int offset = 0, offset_val = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	vindpm = chip->hw_aicl_point;
	chg_info("vindpm = %d\n", vindpm);

	if(vindpm < SGM41515_VINDPM_THRESHOLD_5900MV) {
		offset = VINDPM_OS_3900mV;
		offset_val = SGM41515_VINDPM_THRESHOLD_3900MV;
	} else if (vindpm < SGM41515_VINDPM_THRESHOLD_7500MV) {
		offset = VINDPM_OS_5900mV;
		offset_val = SGM41515_VINDPM_THRESHOLD_5900MV;
	} else if (vindpm < SGM41515_VINDPM_THRESHOLD_10500MV) {
		offset = VINDPM_OS_7500mV;
		offset_val = SGM41515_VINDPM_THRESHOLD_7500MV;
	} else if (vindpm <= SGM41515_VINDPM_THRESHOLD_MAX) {
		offset = VINDPM_OS_10500mV;
		offset_val = SGM41515_VINDPM_THRESHOLD_10500MV;
	}

	/*set input offset*/
	rc = sgm41515_write_byte_mask(chip, REG0F_SGM41515_ADDRESS,
			REG0F_SGM41515_VINDPM_THRESHOLD_OFFSET_MASK,
			offset);

	/*set input vindpm*/
	tmp = (vindpm - offset_val) / REG06_SGM41515_VINDPM_STEP_MV;
	rc = sgm41515_write_byte_mask(chip, REG06_SGM41515_ADDRESS,
			REG06_SGM41515_VINDPM_MASK,
			tmp << REG06_SGM41515_VINDPM_SHIFT);

	return rc;
}

static void oplus_chg_get_batt_volt(int *batt_volt)
{
	union mms_msg_data data = {0};
	struct oplus_mms *gauge_topic;

	gauge_topic = oplus_mms_get_by_name("gauge");
	if (gauge_topic) {
		oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_VOL_MAX, &data, false);
		*batt_volt = data.intval;
	} else {
		chg_info("gauge_topic is null\n");
		*batt_volt = 0;
	}
}

static int oplus_chg_usb_set_input_current(struct sgm41515_chip *chip, int current_ma,
	int aicl_point)
{
	int rc = 0;
	int i = 0;
	int chg_vol = 0;

	bool pre_step = false;

	for (i = 1; i <= current_ma / 100; i++) {
		rc = sgm41515_input_current_limit_without_aicl(chip, i * 100);
		if (rc) {
			chg_err("set icl to %d uA fail, rc=%d\n", i * 100, rc);
			return rc;
		} else {
			chg_err("set icl to %d mA\n", i * 100);
		}
		msleep(90);
		chg_vol = oplus_wired_get_vbus();
		if (chg_vol < aicl_point) {
			chg_err("chg_vol < aicl_point break here\n");
			i = i - 1;
			pre_step = true;
			break;
		}
		if (i == current_ma / 100) {
			chg_err("current_ma / 100 break here\n");
			break;
		}
	}
	if (i <= 0)
		i = 1;
	if (pre_step) {
		rc = sgm41515_input_current_limit_without_aicl(chip, i  * 100);
		if (rc) {
			chg_err("set icl 2 to %d uA fail, rc=%d\n", i * 100, rc);
			return rc;
		} else {
			chg_err("set icl 2 to %d uA\n", i * 100);
		}
	}
	chg_info("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n",
		 chg_vol, i, i * 100, aicl_point);

	return rc;
}

#define AICL_QUICK_CHECK_INTERVAL_MS	5
#define AICL_QUICK_CHECK_UV_THD_MV		120
static bool sgm41515_aicl_voltage_quick_check(struct sgm41515_chip *chip,
		int pre_step_vbus_mv, int collapse_icl_ma, int time_out_ms)
{
	int check_count = time_out_ms / AICL_QUICK_CHECK_INTERVAL_MS;
	int cur_vbus_mv = pre_step_vbus_mv;
	int i;
	bool quick_check_uv_st = false;

	for (i = 0; i < check_count; i++) {
		cur_vbus_mv = oplus_wired_get_vbus();
		if (cur_vbus_mv < pre_step_vbus_mv - AICL_QUICK_CHECK_UV_THD_MV) {
			sgm41515_input_current_limit_without_aicl(chip, collapse_icl_ma);
			chg_info("cur_vbus_mv collapse(%d), cur_vbus_mv = %d, pre_step_vbus_mv = %d, set icl to %dmA",
					i, cur_vbus_mv, pre_step_vbus_mv, collapse_icl_ma);
			quick_check_uv_st = true;
			break;
		}
		usleep_range(AICL_QUICK_CHECK_INTERVAL_MS * 1000,
				(AICL_QUICK_CHECK_INTERVAL_MS + 1) * 1000);
	}

	return quick_check_uv_st;
}

static int sgm41515_usb_icl[] = {
	300, 500, 900, 1000, 1200, 1350, 1500, 1750, 2000, 2300, 2600, 3000,
};

static int sgm41515_icl_aicl_enhance_shortcut(struct sgm41515_chip *chip, int current_ma)
{
	bool present = false;
	int charger_type, batt_volt = 0, rc;
	int sw_aicl_point;

	if (!chip->usb_aicl_enhance)
		return -ENOENT;

	sgm41515_input_present(chip->ic_dev, &present);
	rc = sgm41515_get_charger_type(chip->ic_dev, &charger_type);

	if (rc < 0 || !present)
		return -ENOENT;

	if (charger_type == OPLUS_CHG_USB_TYPE_SDP ||
		charger_type == OPLUS_CHG_USB_TYPE_CDP ||
		(charger_type == OPLUS_CHG_USB_TYPE_UNKNOWN && current_ma == 500))
		goto apply;

	return -ENOENT;

apply:
	oplus_chg_get_batt_volt(&batt_volt);
	sgm41515_set_aicl_point(chip->ic_dev, batt_volt);
	sw_aicl_point = chip->sw_aicl_point;
	return oplus_chg_usb_set_input_current(chip, current_ma, sw_aicl_point);
}

static void sgm41515_icl_ramp_down(struct sgm41515_chip *chip)
{
	int i, rc, pre_icl, pre_icl_index;

	/* first: icl down to 500mA_index+1, step from real_pre_icl_index-1 */
	pre_icl = sgm41515_get_usb_icl(chip);
	for (pre_icl_index = ARRAY_SIZE(sgm41515_usb_icl) - 1;
		pre_icl_index > 0; pre_icl_index--) {
		if (sgm41515_usb_icl[pre_icl_index] < pre_icl)
			break;
	}
	chg_info("pre_icl: %d, pre_icl_index: %d\n", pre_icl, pre_icl_index);
	/* pre_icl_index = real_pre_icl_index-1; down to 500mA_index+1(2) */
	for (i = pre_icl_index; i > 1; i--) {
		if (atomic_read(&chip->charger_suspended) == 1 ||
			atomic_read(&chip->is_suspended) == 1)
			break;
		rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
		if (rc)
			chg_err("icl_down: set icl to %d mA fail, rc=%d\n", sgm41515_usb_icl[i], rc);
		else
			chg_info("icl_down: set icl to %d mA\n", sgm41515_usb_icl[i]);
		msleep(50);
	}
}

static int sgm41515_input_current_limit_write(struct sgm41515_chip *chip, int current_ma)
{
	int i = 0, rc = 0;
	int vbus_stat = 0;
	int chg_vol = 0;
	int sw_aicl_point = 0;
	bool quick_check_uv_st = false;
	int pre_step_vbus_mv = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	if (atomic_read(&chip->is_suspended) == 1) {
		chg_err("suspend,ignore set current=%dmA\n", current_ma);
		return 0;
	}

	rc = sgm41515_icl_aicl_enhance_shortcut(chip, current_ma);
	if (rc >= 0)
		return rc;

	vbus_stat = sgm41515_get_vbus_stat(chip);
	sgm41515_icl_ramp_down(chip);

	/*second: aicl process, step from 500ma*/
	if (current_ma < 500) {
		i = 0;
		goto aicl_end;
	}

	sw_aicl_point = chip->sw_aicl_point;

	i = 1; /* 500 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	usleep_range(90000, 91000);
	chg_vol = oplus_wired_get_vbus();
	if (chg_vol < sw_aicl_point) {
		chg_debug("use 500 here\n");
		goto aicl_end;
	} else if (current_ma < 900)
		goto aicl_end;
	i = 2; /* 900 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	usleep_range(90000, 91000);
	chg_vol = oplus_wired_get_vbus();
	if (chg_vol < sw_aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 1000)
		goto aicl_end;
	i = 3; /* 1000 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	usleep_range(90000, 91000);
	chg_vol = oplus_wired_get_vbus();
	if (chg_vol < sw_aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 1200)
		goto aicl_end;
	i = 4; /* 1200 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	usleep_range(90000, 91000);
	chg_vol = oplus_wired_get_vbus();
	if (chg_vol < sw_aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	}
	i = 5; /* 1350 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	usleep_range(90000, 91000);
	chg_vol = oplus_wired_get_vbus();
	if (chg_vol < sw_aicl_point) {
		i = i - 3; /*We DO NOT use 1.2A here*/
		goto aicl_pre_step;
	} else if (current_ma < 1350) {
		i = i - 1; /*We use 1.2A here*/
		goto aicl_end;
	}
	i = 6; /* 1500 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	usleep_range(90000, 91000);
	chg_vol = oplus_wired_get_vbus();
	if (chg_vol < sw_aicl_point) {
		i = i - 4; /*We DO NOT use 1.2A here*/
		goto aicl_pre_step;
	} else if (current_ma < 1500) {
		i = i - 2; /*We use 1.2A here*/
		goto aicl_end;
	} else if (current_ma < 2000) {
		goto aicl_end;
	}
	i = 7; /* 1750 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	usleep_range(90000, 91000);
	chg_vol = oplus_wired_get_vbus();
	if (chg_vol < sw_aicl_point) {
		i = i - 3; /*1.2*/
		goto aicl_pre_step;
	}
	i = 8; /* 2000 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	usleep_range(90000, 91000);
	chg_vol = oplus_wired_get_vbus();
	pre_step_vbus_mv = chg_vol;
	if (chg_vol < sw_aicl_point) {
		i = i - 2; /*1.5*/
		goto aicl_pre_step;
	} else if ((current_ma < 2300) || (vbus_stat == REG08_SGM41515_VBUS_STAT_FLOAT)) {
		goto aicl_end;
	}
	i = 9; /* 2300 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	quick_check_uv_st = sgm41515_aicl_voltage_quick_check(chip, pre_step_vbus_mv, sgm41515_usb_icl[i - 1], 90);
	chg_vol = oplus_wired_get_vbus();
	pre_step_vbus_mv = chg_vol;
	if (quick_check_uv_st || chg_vol < sw_aicl_point) {
		i = i - 1; /*2.0*/
		goto aicl_pre_step;
	} else if (current_ma < 2600) {
		goto aicl_end;
	}
	i = 10; /* 2600 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	quick_check_uv_st = sgm41515_aicl_voltage_quick_check(chip, pre_step_vbus_mv, sgm41515_usb_icl[i - 2], 90);
	chg_vol = oplus_wired_get_vbus();
	pre_step_vbus_mv = chg_vol;
	if (quick_check_uv_st || chg_vol < sw_aicl_point) {
		i = i - 2; /*2.0*/
		goto aicl_pre_step;
	} else if (current_ma < 3000) {
		goto aicl_end;
	}
	i = 11; /* 3000 */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	quick_check_uv_st = sgm41515_aicl_voltage_quick_check(chip, pre_step_vbus_mv, sgm41515_usb_icl[i - 1], 90);
	chg_vol = oplus_wired_get_vbus();
	if (quick_check_uv_st || chg_vol < sw_aicl_point) {
		i = i -1;
		goto aicl_pre_step;
	} else if (current_ma >= 3000) {
		goto aicl_end;
	}
aicl_pre_step:
	chg_info("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d current_ma=%d, aicl_pre_step\n",
			chg_vol, i, sgm41515_usb_icl[i], sw_aicl_point, current_ma);
	goto aicl_rerun;
aicl_end:
	chg_info("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d current_ma=%d, aicl_end\n",
		chg_vol, i, sgm41515_usb_icl[i], sw_aicl_point, current_ma);
	goto aicl_rerun;
aicl_rerun:
	/* aicl_result = sgm41515_usb_icl[i]; */
	rc = sgm41515_input_current_limit_without_aicl(chip, sgm41515_usb_icl[i]);
	rc = sgm41515_set_vindpm_vol(chip);
	return rc;
}

static int sgm41515_set_prechg_voltage_threshold(struct sgm41515_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_write_byte_mask(chip, REG01_SGM41515_ADDRESS,
			REG01_SGM41515_SYS_VOL_LIMIT_MASK,
			REG01_SGM41515_SYS_VOL_LIMIT_3400MV);

	return rc;
}

static const unsigned int SGM41515D_IPRECHG_CURRENT_STABLE[] = {
	5, 10, 15, 20, 30, 40, 50, 60,
	80, 100, 120, 140, 160, 180, 200, 240
};

int sgm41515_set_prechg_current(struct sgm41515_chip *chip, int ipre_mA)
{
	int tmp = 0;
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	if (chip->part_id == SGM41515D_PART_ID || chip->part_id == SGM41515_PART_ID) {
		/* Disable IPRECHG x6 mode */
		rc = sgm41515_write_byte_mask(chip, REG0D_SGM41515_ADDRESS,
			REG0D_SGM41515_IPRECHG_MASK, REG0D_SGM41515_IPRECHG_X6_DISABLE);
		if (rc < 0) {
			chg_err("Couldn't disable IPRECHG x6 mode rc = %d\n", rc);
			return rc;
		}
	}
	if (chip->part_id == SGM41515D_PART_ID) {
		chg_err("sgm41515d prechg_current = %d\n", ipre_mA);
		for(tmp = 0; tmp < 16; tmp++) {
			if (ipre_mA >= SGM41515D_IPRECHG_CURRENT_STABLE[tmp])
				break;
		}
		tmp--;
	} else {
		chg_err("prechg_current = %d\n", ipre_mA);
		tmp = ipre_mA - REG03_SGM41515_PRE_CHG_CURRENT_LIMIT_OFFSET;
		tmp = tmp / REG03_SGM41515_PRE_CHG_CURRENT_LIMIT_STEP;
	}

	rc = sgm41515_write_byte_mask(chip, REG03_SGM41515_ADDRESS,
			REG03_SGM41515_PRE_CHG_CURRENT_LIMIT_MASK,
			(tmp + 1) << REG03_SGM41515_PRE_CHG_CURRENT_LIMIT_SHIFT);

	return rc;
}

static int sgm41515_set_chging_term_disable(struct sgm41515_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if(atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_write_byte_mask(chip, REG05_SGM41515_ADDRESS,
			REG05_SGM41515_TERMINATION_MASK,
			REG05_SGM41515_TERMINATION_DISABLE);
	if (rc)
		chg_err("Couldn't set chging term disable rc = %d\n", rc);

	return rc;
}

static int sgm41515_suspend_charger(struct sgm41515_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	atomic_set(&chip->is_suspended, 1);

	rc |= sgm41515_disable_charging(chip);
	rc |= sgm41515_enable_hiz_mode(chip, true);

	return rc;
}

static int sgm41515_unsuspend_charger(struct sgm41515_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	atomic_set(&chip->is_suspended, 0);
	rc |= sgm41515_enable_charging(chip);
	rc |= sgm41515_enable_hiz_mode(chip, false);

	return rc;
}

static int sgm41515_input_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	int rc = 0;
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	if (suspend) {
		if (!wait_for_completion_timeout(&chip->bc12_complete_comp, msecs_to_jiffies(BC12_COMPLETE_COMP_CHECK_MS)))
			chg_err("bc12 completion timeout, proceed suspend anyway\n");
		rc = sgm41515_suspend_charger(chip);
	} else {
		rc = sgm41515_unsuspend_charger(chip);
	}

	return rc;
}

static int sgm41515_input_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct sgm41515_chip *chip;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	return atomic_read(&chip->is_suspended);
}

static int sgm41515_output_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	int rc = 0;
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (suspend)
		rc = sgm41515_disable_charging(chip);
	else
		rc = sgm41515_enable_charging(chip);

	chg_info("charger out %s, rc = %d", suspend ? "suspend" : "unsuspend", rc);

	return rc;
}

static int sgm41515_output_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	int rc = 0;
	u8 reg_val = 0;
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_read_byte(chip, REG01_SGM41515_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG01_SGM41515_ADDRESS rc = %d\n", rc);
		return 0;
	}

	*suspend = !!(reg_val & REG01_SGM41515_CHARGING_MASK);

	return 0;
}

static int sgm41515_set_icl(struct oplus_chg_ic_dev *ic_dev, bool vooc_mode, bool step, int icl_ma)
{
	int rc = 0;
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip->bc12_complete && step) {
		chg_info("bc12_complete = %d, skip aicl\n", chip->bc12_complete);
		return rc;
	}

	if (step)
		rc = sgm41515_input_current_limit_write(chip, icl_ma);
	else
		rc = sgm41515_input_current_limit_without_aicl(chip, icl_ma);
	return rc;
}

static int sgm41515_get_icl(struct oplus_chg_ic_dev *ic_dev, int *icl_ma)
{
	int icl = 0;
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	icl = sgm41515_get_usb_icl(chip);
	*icl_ma = icl;

	return 0;
}

static int sgm41515_charging_current_write_fast(struct sgm41515_chip *chip, int fcc_ma)
{
	int rc = 0;
	int tmp = 0;

	if (!chip)
		return 0;

	if(atomic_read(&chip->charger_suspended) == 1)
		return 0;

	chg_info("sgm41515d set charge current = %d\n", fcc_ma);
	if (fcc_ma <= 40)
		tmp = fcc_ma / 5;
	else if (fcc_ma <= 110)
		tmp = 0x08 + (fcc_ma - 40) / 10;
	else if (fcc_ma <= 270)
		tmp = 0x0F + (fcc_ma - 110) / 20;
	else if (fcc_ma <= 540)
		tmp = 0x17 + (fcc_ma - 270) / 30;
	else if (fcc_ma <= 1500)
		tmp = 0x20 + (fcc_ma - 540) / 60;
	else if (fcc_ma <= 2940)
		tmp = 0x30 + (fcc_ma - 1500) / 120;
	else
		tmp = 0x3d;

	rc = sgm41515_write_byte_mask(chip, REG02_SGM41515_ADDRESS,
			REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_MASK,
			tmp << REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_SHIFT);

	return rc;
}

static int sgm41515_set_fcc(struct oplus_chg_ic_dev *ic_dev, int fcc_ma)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->bc12_complete && fcc_ma > REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_500MA) {
		chg_info("bc12_complete = %d, fcc_ma = %d, force fcc_ma to %d\n",
				chip->bc12_complete, fcc_ma, REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_500MA);
		fcc_ma = REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_500MA;
	}

	return sgm41515_charging_current_write_fast(chip, fcc_ma);
}

static int sgm41515_set_vreg_fine_tuning(struct sgm41515_chip *chip, int tuning_mv)
{
	int tuning_val = REG0F_SGM41515_VREG_FT_DISABLE;

	if (!chip || atomic_read(&chip->charger_suspended) == 1)
		return -ENODEV;

	if (tuning_mv >= 4 && tuning_mv < 12) {
		tuning_val = REG0F_SGM41515_VREG_FT_PLUS_8MV;
	} else if (tuning_mv >= -12 && tuning_mv < -4) {
		tuning_val = REG0F_SGM41515_VREG_FT_MINUS_8MV;
	} else if (tuning_mv >= -20 && tuning_mv < -12) {
		tuning_val = REG0F_SGM41515_VREG_FT_MINUS_16MV;
	}

	chg_info("fine_tuning = %dmV, tuning_val = 0x%02x\n", tuning_mv, tuning_val);
	return sgm41515_write_byte_mask(chip, REG0F_SGM41515_ADDRESS,
				       REG0F_SGM41515_VREG_FT_MASK,
				       tuning_val << REG0F_SGM41515_VREG_FT_SHIFT);
}

static int sgm41515_smart_voltage_adjustment(struct sgm41515_chip *chip, int vfloat_mv)
{
	int rc = 0;
	int coarse_tmp, coarse_voltage, fine_tuning_mv;

	if (!chip || atomic_read(&chip->charger_suspended) == 1)
		return -ENODEV;

	if (vfloat_mv < 3856 || vfloat_mv > 4624) {
		chg_err("Failed to set vfloat_mv, vfloat_mv %d below minimum (3856) or exceeds maximum (4624)\n", vfloat_mv);
		return -EINVAL;
	}

	coarse_tmp = (vfloat_mv - REG04_SGM41515_CHG_VOL_LIMIT_OFFSET) / REG04_SGM41515_CHG_VOL_LIMIT_STEP;
	coarse_voltage = REG04_SGM41515_CHG_VOL_LIMIT_OFFSET + coarse_tmp * REG04_SGM41515_CHG_VOL_LIMIT_STEP;
	fine_tuning_mv = vfloat_mv - coarse_voltage;

	if (fine_tuning_mv > SMART_TUNING_MAX_ADJUSTMENT) {
		coarse_tmp += 1;
		coarse_voltage = REG04_SGM41515_CHG_VOL_LIMIT_OFFSET + coarse_tmp * REG04_SGM41515_CHG_VOL_LIMIT_STEP;
		fine_tuning_mv = vfloat_mv - coarse_voltage;
	}

	chg_info("vfloat=%dmV, coarse=%dmV, fine_tuning=%dmV\n", vfloat_mv, coarse_voltage, fine_tuning_mv);

	rc = sgm41515_write_byte_mask(chip, REG04_SGM41515_ADDRESS,
				      REG04_SGM41515_CHG_VOL_LIMIT_MASK,
				      coarse_tmp << REG04_SGM41515_CHG_VOL_LIMIT_SHIFT);
	if (rc < 0) {
		chg_err("Failed to set coarse voltage: %d\n", rc);
		return rc;
	}

	rc = sgm41515_set_vreg_fine_tuning(chip, fine_tuning_mv);
	if (rc < 0) {
		chg_err("Failed to set fine tuning: %d\n", rc);
		return rc;
	}

	return rc;
}

static int sgm41515_float_voltage_write(struct sgm41515_chip *chip, int vfloat_mv)
{
	int rc = 0;
	int tmp = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	chg_err("vfloat_mv = %d\n", vfloat_mv);

	if (chip->part_id == SGM41512SD_PART_ID) {
		rc = sgm41515_smart_voltage_adjustment(chip, vfloat_mv);
	} else {
		tmp = vfloat_mv - REG04_SGM41515_CHG_VOL_LIMIT_OFFSET;

		tmp = tmp / REG04_SGM41515_CHG_VOL_LIMIT_STEP;

		rc = sgm41515_write_byte_mask(chip, REG04_SGM41515_ADDRESS,
				REG04_SGM41515_CHG_VOL_LIMIT_MASK,
				tmp << REG04_SGM41515_CHG_VOL_LIMIT_SHIFT);
	}

	return rc;
}

static int sgm41515_set_fv(struct oplus_chg_ic_dev *ic_dev, int fv_mv)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	if (fv_mv <= 0) {
		chg_err("invalid value ignore");
		return 0;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);

	return sgm41515_float_voltage_write(chip, fv_mv);
}

static const unsigned int SGM41515D_ITERM_CURRENT_STABLE[] = {
	5, 10, 15, 20, 30, 40, 50, 60,
	80, 100, 120, 140, 160, 180, 200, 240
};

static int sgm41515_set_termchg_current(struct sgm41515_chip *chip, int term_curr)
{
	int rc = 0;
	int tmp = 0;

	if (!chip)
		return 0;

	if(atomic_read(&chip->charger_suspended) == 1)
		return 0;

	chg_err("sgm41515d term_current = %d\n", term_curr);
	for(tmp = 0; tmp < 16; tmp++) {
		if(term_curr >= SGM41515D_ITERM_CURRENT_STABLE[tmp])
			break;
	}
	tmp--;

	rc = sgm41515_write_byte_mask(chip, REG03_SGM41515_ADDRESS,
			REG03_SGM41515_TERM_CHG_CURRENT_LIMIT_MASK,
			tmp << REG03_SGM41515_TERM_CHG_CURRENT_LIMIT_SHIFT);

	return 0;
}

static int sgm41515_set_iterm(struct oplus_chg_ic_dev *ic_dev, int iterm_ma)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	return sgm41515_set_termchg_current(chip, iterm_ma);
}

static int sgm41515_set_rechg_voltage(struct sgm41515_chip *chip, int recharge_mv)
{
	int rc = 0;
	u8 reg_val;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	if (recharge_mv < REG04_SGM41515_RECHG_VOLTAGE_200MV)
		reg_val = REG04_SGM41515_RECHG_THRESHOLD_VOL_100MV;
	else
		reg_val = REG04_SGM41515_RECHG_THRESHOLD_VOL_200MV;

	rc = sgm41515_write_byte_mask(chip, REG04_SGM41515_ADDRESS,
			REG04_SGM41515_RECHG_THRESHOLD_VOL_MASK,
			reg_val);

	if (rc)
		chg_err("Couldn't set recharging threshold rc = %d\n", rc);

	return rc;
}

static int sgm41515_set_rechg_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	return sgm41515_set_rechg_voltage(chip, vol_mv);
}

static int sgm41515_get_input_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	/* Not support ibus only sgm41515 */
	*curr_ma = 0;
	return 0;
}

static bool sgm41515_get_vindpm_status(struct sgm41515_chip *chip)
{
	int rc = 0;
	u8 reg_val = 0;
	bool in_vindpm = false;

	if (!chip)
		return false;

	if (atomic_read(&chip->charger_suspended) == 1)
		return false;

	rc = sgm41515_read_byte(chip, REG0A_SGM41515_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read regeister, rc = %d\n", rc);
		return false;
	}

	in_vindpm = ((reg_val & REG0A_SGM41515_VINDPM_MASK) == REG0A_SGM41515_IN_VINDPM) ? 1 : 0;
	return in_vindpm;
}

static int sgm41515_get_input_vol(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	/* Not support vbus only sgm41515 without ADC */
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip || !chip->vbus_present) {
		*vol_mv = 0;
		return 0;
	}

	if (sgm41515_get_vindpm_status(chip)) {
		*vol_mv = chip->hw_aicl_point;
	} else if (chip->sw_aicl_point < SGM41515_FAKE_VBUS_5V) {
		*vol_mv = SGM41515_FAKE_VBUS_5V;
	} else {
		*vol_mv = SGM41515_FAKE_VBUS_9V;
	}

	return 0;
}

static int sgm41515_set_aicl_point(struct oplus_chg_ic_dev *ic_dev, int vbatt)
{
	int rc = 0;
	int index = 0;
	struct sgm41515_chip *chip;
	bool present = false;
	int charger_type;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (chip->usb_aicl_enhance) {
		sgm41515_input_present(chip->ic_dev, &present);
		rc = sgm41515_get_charger_type(chip->ic_dev, &charger_type);
		if (rc >= 0 && charger_type == OPLUS_CHG_USB_TYPE_SDP && present) {
			chip->hw_aicl_point = USB_HW_AICL_POINT;
			chip->sw_aicl_point = USB_SW_AICL_POINT;
			chip->pre_aicl_index = 0;
			rc = sgm41515_set_vindpm_vol(chip);
			return rc;
		}
	}

	for (index = 0; index < AICL_POINT_INDEX_MAX; index++) {
		if (vbatt <= sgm41515_vindpm_vol[index].vbat_thr) {
			chip->sw_aicl_point = sgm41515_vindpm_vol[index].sw_voltage;
			chip->hw_aicl_point = sgm41515_vindpm_vol[index].hw_voltage;
			break;
		}
	}
	if (index == AICL_POINT_INDEX_MAX) {
		chip->hw_aicl_point = HW_AICL_POINT_DEFAULT;
		chip->sw_aicl_point = SW_AICL_POINT_DEFAULT;
		index = 0;
	}

	if (chip->pre_aicl_index != index) {
		rc = sgm41515_set_vindpm_vol(chip);
		chip->pre_aicl_index = index;
	}

	return rc;
}

static int sgm41515_otg_boost_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc;
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chip->otg_mode = en;

	if (en)
		rc = sgm41515_otg_enable(chip);
	else
		rc = sgm41515_otg_disable(chip);

	sgm41515_set_wdt_timer(chip, REG05_SGM41515_WATCHDOG_TIMER_DISABLE);

	if (rc < 0)
		chg_err("can't %s otg boost, rc=%d\n", en ? "enable" : "disable", rc);
	return rc;
}

static int sgm41515_set_otg_voltage(struct sgm41515_chip *chip, int vol_mv)
{
	int rc = 0;
	u8 reg_val = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;
	if (vol_mv < REG06_SGM41515_OTG_VLIM_OFFSET)
		vol_mv = REG06_SGM41515_OTG_VLIM_OFFSET;
	else if (vol_mv > REG06_SGM41515_OTG_VLIM_MAXMV)
		vol_mv = REG06_SGM41515_OTG_VLIM_MAXMV;
	reg_val = (vol_mv - REG06_SGM41515_OTG_VLIM_OFFSET) / REG06_SGM41515_OTG_VLIM_STEP;

	rc = sgm41515_write_byte_mask(chip, REG06_SGM41515_ADDRESS,
			REG06_SGM41515_OTG_VLIM_MASK,
			reg_val << REG06_SGM41515_OTG_VLIM_SHIFT);

	return rc;
}

static int sgm41515_set_otg_boost_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	return sgm41515_set_otg_voltage(chip, vol_mv);
}

static int sgm41515_set_otg_boost_curr_limit(struct oplus_chg_ic_dev *ic_dev, int curr_mA)
{
	int rc;
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if(!chip) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	rc = sgm41515_otg_ilim_set(chip, curr_mA);
	if (rc)
		chg_err("failed to set boost current, ret = %d\n", rc);
	return rc;
}

static int sgm41515_aicl_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	return rc;
}

static int sgm41515_aicl_rerun(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	return rc;
}

static int sgm41515_aicl_reset(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	return rc;
}

static int sgm41515_hardware_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!oplus_is_rf_ftm_mode())
		sgm41515_hw_init(chip);
	return 0;
}

static int sgm41515_get_charger_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (chip->charge_type) {
	case POWER_SUPPLY_TYPE_USB:
		*type = OPLUS_CHG_USB_TYPE_SDP;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		*type = OPLUS_CHG_USB_TYPE_CDP;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		*type = OPLUS_CHG_USB_TYPE_DCP;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		*type = OPLUS_CHG_USB_TYPE_QC2;
		break;
	default:
		*type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		break;
	}

	return 0;
}

static int sgm41515_rerun_bc12(struct oplus_chg_ic_dev *ic_dev)
{
	struct sgm41515_chip *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chg_info("rerun bc1.2\n");
	sgm41515_request_dpdm(chip, true);
	/* no need to retry */
	chip->bc12_retry = true;
	chip->auto_bc12 = false;
	chip->bc12_complete = false;
	reinit_completion(&chip->bc12_complete_comp);
	rc = sgm41515_enable_hiz_mode(chip, false);
	if (rc < 0) {
		chg_err("can't disable hiz mode, rc=%d\n", rc);
		goto err;
	}
	rc = sgm41515_write_byte_mask(chip, REG07_SGM41515_ADDRESS, REG07_SGM41515_IINDET_EN_MASK,
				      REG07_SGM41515_IINDET_EN_FORCE_DET);
	if (rc < 0) {
		chg_err("can't rerun bc1.2, rc=%d", rc);
		goto err;
	}

	return 0;

err:
	chip->bc12_complete = true;
	return rc;
}

static int sgm41515_force_dpdm(struct sgm41515_chip *chip)
{
	u8 val = 0;
	int ret = 0;
	if (chip == NULL)
		return -ENODEV;

	ret = sgm41515_write_byte_mask(chip, REG0D_SGM41515_ADDRESS, REG0D_SGM41515_DPDM_VSEL_MASK,
				      val); /*dp/dm Hiz*/
	if (ret) {
		chg_err("can't force dpdm");
		return ret;
	}
	return ret;
}

#define DP_VOLTAGE_0P6V			0x2<<3
#define DP_VOLTAGE_3P3V_DM_0P6V		0xe << 1
static int sgm41515_enable_hvdcp(struct sgm41515_chip *chip)
{
	int ret = 0;
	if (!chip)
		return -EINVAL;
	if ((chip->charge_type != POWER_SUPPLY_TYPE_USB_DCP) || (chip->bc12_complete == false)) {
		chg_err("charge type is not dcp or bc12 not complete");
		return -EINVAL;
	}
	/*dp and dm connected,dp 0.6V dm 0.6V*/
	ret = sgm41515_write_byte_mask(chip, REG0D_SGM41515_ADDRESS,
				  REG0D_SGM41515_DP_VSEL_MASK, DP_VOLTAGE_0P6V); /*dp 0.6V*/
	if (ret)
		return ret;
	msleep(2500);
	ret = sgm41515_write_byte_mask(chip, REG0D_SGM41515_ADDRESS,
				  REG0D_SGM41515_DPDM_VSEL_MASK, DP_VOLTAGE_3P3V_DM_0P6V); /*dp 3.3v dm 0.6v*/

	return ret;
}

static bool sgm41515_is_hvdcp(struct sgm41515_chip *chip)
{
	bool hvdcp = false;
	int vol_mv;

	if (chip == NULL)
		return false;

	vol_mv = oplus_wired_get_vbus();
	if (vol_mv > SGM41515_VINDPM_THRESHOLD_7500MV)
		hvdcp = true;

	chg_err("finn vbus = %dmV, hvdcp = %d\n", vol_mv, hvdcp);

	chip->is_hvdcp = hvdcp;
	return hvdcp;
}

static int sgm41515_qc_detect_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int count = 0;
	int ret = 0;
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL)
		return -ENODEV;

	chg_info("%d\n", en);
	if (!en)
		return 0;

	if (!sgm41515_is_hvdcp(chip)) {
		do {
			ret = sgm41515_force_dpdm(chip);
			if (ret)
				return ret;
			msleep(100);
			ret = sgm41515_enable_hvdcp(chip);
			if (ret)
				return ret;
			msleep(200);
			if (sgm41515_is_hvdcp(chip)) {
				chip->charge_type = POWER_SUPPLY_TYPE_USB_HVDCP;
				oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
				break;
			}
			if ((chip->charge_type != POWER_SUPPLY_TYPE_USB_DCP)
				|| (chip->bc12_complete == false))
				break;
		} while (count++ < 5);
	}
	chg_err("count=%d\n", count);
	return 0;
}

static int sgm41515_disable_hvdcp(struct sgm41515_chip *chip)
{
	int ret = 0;
	u8 val;

	if (chip == NULL)
		return -ENODEV;

	chg_debug("disable hvdcp\n");

	/*dp and dm connected,dp 0.6V dm Hiz*/
	val = 0x8 << 1;
	ret = sgm41515_write_byte_mask(chip, REG0D_SGM41515_ADDRESS,
			REG0D_SGM41515_DPDM_VSEL_MASK, val);

	return ret;
}

static int sgm41515_adjust_qc_voltage_normal(struct sgm41515_chip *chip)
{
	int ret;

	if (chip == NULL)
		return -ENODEV;

	ret = sgm41515_disable_hvdcp(chip);
	chip->is_hvdcp = false;
	chg_err("adjust 5v success\n");
	return ret;
}

static int sgm41515_wait_for_hvdcp_initial(struct sgm41515_chip *chip)
{
	int i;
 	for (i = 0; i < 10; i++) {
 		if (sgm41515_is_hvdcp(chip))
			return 0;
 		msleep(100);
 	}
	return 0;
}

static int sgm41515_set_dpdm_voltage(struct sgm41515_chip *chip)
{
	u8 val = 0xe << 1;
	int ret;

 	ret = sgm41515_write_byte_mask(chip, REG0D_SGM41515_ADDRESS,
 				  REG0D_SGM41515_DPDM_VSEL_MASK, val); /*dp 3.3v dm 0.6v*/
	return ret;
}

static int sgm41515_enable_hvdcp_retry(struct sgm41515_chip *chip)
{
	int cnt = 0;
	int ret;

	do {
		ret = sgm41515_force_dpdm(chip);
		if (ret)
			return ret;
		msleep(100);
		ret = sgm41515_enable_hvdcp(chip);
		if (ret) {
			chg_err("enable_hvdcp failed, rc=%d\n", ret);
			return ret;
		}
		msleep(200);
		if (sgm41515_is_hvdcp(chip))
			return 0;
		if ((sgm41515_get_vbus_stat(chip) != REG08_SGM41515_VBUS_STAT_DCP)
			|| (chip->bc12_complete == false))
			return 0;
	} while (cnt++ < 5);

	return 0;
}

static int sgm41515_adjust_qc_voltage_fast(struct sgm41515_chip *chip)
{
	int ret = 0;

	if (chip == NULL)
		return -ENODEV;

	sgm41515_wait_for_hvdcp_initial(chip);
	ret = sgm41515_set_dpdm_voltage(chip);
	if (ret < 0)
 		return ret;
	if (!sgm41515_is_hvdcp(chip)) {
		ret = sgm41515_enable_hvdcp_retry(chip);
		if (ret)
			return ret;
 	}

	if (chip->is_hvdcp)
		chg_err("adjust 9v success\n");
	else
		chg_err("adjust 9v failed\n");

 	return ret;
}

#define SGM41515_FAST_CHARGER_VOLTAGE		9000
#define SGM41515_NORMAL_CHARGER_VOLTAGE		5000
static int sgm41515_adjust_qc_voltage(struct sgm41515_chip *chip, u32 value)
{
	if (chip == NULL)
		return -ENODEV;

	if (value == SGM41515_NORMAL_CHARGER_VOLTAGE)
		return sgm41515_adjust_qc_voltage_normal(chip);
	else if (value == SGM41515_FAST_CHARGER_VOLTAGE)
		return sgm41515_adjust_qc_voltage_fast(chip);
	return 0;
}

static int sgm41515_set_qc_config_fast(struct sgm41515_chip *chip, int chg_vol)
{
	int ret;

	if (chg_vol > SGM41515_VINDPM_THRESHOLD_7500MV) {
		chg_info("no allow set 9V, chg_vol = %d\n", chg_vol);
		return 0;
	}
	chg_info("set qc to 9V");
	ret = sgm41515_adjust_qc_voltage(chip, SGM41515_FAST_CHARGER_VOLTAGE);
	if (ret) {
		chg_info("adjust to 9V failed");
		return -EINVAL;
	}
	return ret;
}

static int sgm41515_set_qc_config_normal(struct sgm41515_chip *chip)
{
	int ret;

	chg_info("set qc to 5V\n");
	ret = sgm41515_adjust_qc_voltage(chip, SGM41515_NORMAL_CHARGER_VOLTAGE);
	if (ret) {
		chg_info("adjust to 5V failed");
		return -EINVAL;
	}
	return ret;
}

static int sgm41515_set_config_qc2(struct sgm41515_chip *chip, int vol_mv, int chg_vol)
{
	if (vol_mv != SGM41515_NORMAL_CHARGER_VOLTAGE && vol_mv != SGM41515_FAST_CHARGER_VOLTAGE) {
		chg_err("Unsupported qc voltage(=%d)\n", vol_mv);
		return -EINVAL;
	}

	if (vol_mv == SGM41515_FAST_CHARGER_VOLTAGE)
		return sgm41515_set_qc_config_fast(chip, chg_vol);
	else
		return sgm41515_set_qc_config_normal(chip);
}

static int sgm41515_set_config_qc3(void)
{
	return 0;
}

static int sgm41515_set_qc_config(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_qc_version version, int vol_mv)
{
	struct sgm41515_chip *chip;
	int chg_vol;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}
	chg_info("\n");
	chg_vol = oplus_wired_get_vbus();
	switch (version) {
	case OPLUS_CHG_QC_2_0:
		return sgm41515_set_config_qc2(chip, vol_mv, chg_vol);
	case OPLUS_CHG_QC_3_0:
		return sgm41515_set_config_qc3();
	default:
		chg_err("Unsupported qc version(=%d)\n", version);
		return -EINVAL;
	}
}

static int sgm41515_disable_vbus(struct oplus_chg_ic_dev *ic_dev, bool en,
				 bool delay)
{
	struct sgm41515_chip *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chip->otg_mode = en;

	if (!gpio_is_valid(chip->dis_vbus_gpio)) {
		chg_info("Not support dis_vbus_gpio");
		return 0;
	}

	mutex_lock(&chip->pinctrl_lock);
	if (en) {
		rc = pinctrl_select_state(chip->pinctrl, chip->dis_vbus_active);
	} else {
		/* Wait for VBUS to be completely powered down, usually 20ms */
		if (delay)
			msleep(20);
		rc = pinctrl_select_state(chip->pinctrl, chip->dis_vbus_sleep);
	}
	mutex_unlock(&chip->pinctrl_lock);
	if (rc < 0)
		chg_err("can't set disable vbus gpio to %s, rc=%d\n",
			en ? "active" : "sleep", rc);
	else
		chg_err("set disable vbus gpio to %s\n",
			en ? "active" : "sleep");

	return rc;
}

static int sgm41515_kick_watchdog(struct sgm41515_chip *chip)
{
	int rc = 0;

	if (!chip)
		return 0;

	if(atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_write_byte_mask(chip, REG01_SGM41515_ADDRESS,
			REG01_SGM41515_WDT_TIMER_RESET_MASK,
			REG01_SGM41515_WDT_TIMER_RESET);
	if (rc)
		chg_err("Couldn't sgm41515 kick wdt rc = %d\n", rc);

	return rc;
}

static int sgm41515_kick_wdt(struct oplus_chg_ic_dev *ic_dev)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	return sgm41515_kick_watchdog(chip);
}

static int sgm41515_get_usb_aicl_enhance(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*enable = chip->usb_aicl_enhance;

	chg_info("usb_aicl_enhance: %d", *enable);

	return 0;
}

static int sgm41515_vbus_dischg_enable(struct sgm41515_chip *chip, bool en)
{
	int rc;

	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	if (!gpio_is_valid(chip->usbtemp_dischg_gpio)) {
		chg_info("Not support dischg_gpio");
		return 0;
	}

	mutex_lock(&chip->pinctrl_lock);
	if (en)
		rc = pinctrl_select_state(chip->pinctrl, chip->usbtemp_dischg_enable);
	else
		rc = pinctrl_select_state(chip->pinctrl, chip->usbtemp_dischg_disable);
	mutex_unlock(&chip->pinctrl_lock);
	if (rc < 0)
		chg_err("can't set dischg gpio to %s, rc=%d\n",
			en ? "active" : "sleep", rc);
	else
		chg_err("set dischg gpio to %s\n",
			en ? "active" : "sleep");

	return rc;
}

static int sgm41515_set_usb_dischg_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct sgm41515_chip *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = sgm41515_write_byte_mask(chip, REG00_SGM41515_ADDRESS, REG00_SGM41515_HIZ_MODE_MASK,
		en ? REG00_SGM41515_HIZ_MODE_ENABLE : REG00_SGM41515_HIZ_MODE_DISABLE);
	if (rc < 0)
		chg_err("can't %s hiz mode, rc=%d\n", en ? "enable" : "disable", rc);
	msleep(10); /*vsw_dis->gpio_high need 5-60ms*/
	rc = sgm41515_vbus_dischg_enable(chip, en);
	chg_info("set_usbtemp_dischg_enable=%d\n", en);
	return rc;
}

static int sgm41515_set_shipmode(struct sgm41515_chip *chip, bool enable)
{
	int rc = 0;

	if (chip == NULL)
		return rc;

	if (enable) {
		rc = sgm41515_write_byte_mask(chip, REG07_SGM41515_ADDRESS,
			REG07_SGM41515_BATFET_DIS_MASK,
			REG07_SGM41515_BATFET_DIS_ON);
		if (rc < 0)
			chg_err("Couldn't set REG07_SGM41515_BATFET_DIS_ON rc = %d\n", rc);
	} else {
		rc = sgm41515_write_byte_mask(chip, REG07_SGM41515_ADDRESS,
			REG07_SGM41515_BATFET_DIS_MASK,
			REG07_SGM41515_BATFET_DIS_OFF);
		if (rc < 0)
			chg_err("Couldn't set REG07_SGM41515_BATFET_DIS_OFF rc = %d\n", rc);
	}

	return rc;
}

static int sgm41515_shipmode_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct sgm41515_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);

	sgm41515_set_shipmode(chip, en);

	return 0;
}

static void sgm41515_wired_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_PRESENT:
			chg_info("wired present!");
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void sgm41515_subscribe_wired_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct sgm41515_chip *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    sgm41515_wired_subs_callback, chip->ic_dev->manu_name);
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n", PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_PRESENT, &data,
				true);
	chip->vbus_present = !!data.intval;
	if (chip->vbus_present && !chip->otg_mode) {
		sgm41515_request_dpdm(chip, true);
		sgm41515_bc12_boot_check(chip);
	}

	if (sgm41515_get_bus_gd(chip) && (chip->charge_type == POWER_SUPPLY_TYPE_UNKNOWN))
		schedule_delayed_work(&chip->event_work, msecs_to_jiffies(INIT_WORK_OTHER_DELAY));
}

static void *oplus_chg_get_func(struct oplus_chg_ic_dev *ic_dev,
				enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, sgm41515_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, sgm41515_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, sgm41515_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, sgm41515_smt_test);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_PRESENT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_PRESENT, sgm41515_input_present);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND, sgm41515_input_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND, sgm41515_input_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND, sgm41515_output_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND, sgm41515_output_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ICL, sgm41515_set_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_ICL, sgm41515_get_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FCC, sgm41515_set_fcc);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FV, sgm41515_set_fv);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ITERM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ITERM, sgm41515_set_iterm);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL, sgm41515_set_rechg_vol);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR, sgm41515_get_input_curr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL, sgm41515_get_input_vol);
		break;
	case OPLUS_IC_FUNC_OTG_BOOST_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_OTG_BOOST_ENABLE, sgm41515_otg_boost_enable);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_VOL, sgm41515_set_otg_boost_vol);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT, sgm41515_set_otg_boost_curr_limit);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_ENABLE, sgm41515_aicl_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RERUN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RERUN, sgm41515_aicl_rerun);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RESET, sgm41515_aicl_reset);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_AICL_POINT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_AICL_POINT, sgm41515_set_aicl_point);
		break;
	case OPLUS_IC_FUNC_BUCK_HARDWARE_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_HARDWARE_INIT, sgm41515_hardware_init);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE, sgm41515_get_charger_type);
		break;
	case OPLUS_IC_FUNC_BUCK_RERUN_BC12:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_RERUN_BC12, sgm41515_rerun_bc12);
		break;
	case OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE, sgm41515_qc_detect_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG, sgm41515_set_qc_config);
		break;
	case OPLUS_IC_FUNC_DISABLE_VBUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_DISABLE_VBUS, sgm41515_disable_vbus);
		break;
	case OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE, sgm41515_shipmode_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_KICK_WDT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_KICK_WDT, sgm41515_kick_wdt);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE, sgm41515_get_usb_aicl_enhance);
		break;
	case OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE, sgm41515_set_usb_dischg_enable);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static int sgm41515_gpio_init(struct sgm41515_chip *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -ENODEV;
	}

	chip->dis_vbus_gpio = of_get_named_gpio(node, "oplus,dis_vbus-gpio", 0);
	if (gpio_is_valid(chip->dis_vbus_gpio)) {
		rc = gpio_request(chip->dis_vbus_gpio, "sgm41515_dis_vbus-gpio");
		if (rc < 0) {
			chg_err("event_gpio request error, rc=%d\n", rc);
			return rc;
		}
		chip->dis_vbus_active =
			pinctrl_lookup_state(chip->pinctrl, "dis_vbus_active");
		if (IS_ERR_OR_NULL(chip->dis_vbus_active)) {
			chg_err("get dis_vbus_active fail\n");
			goto free_dis_vbus_gpio;
		}
		chip->dis_vbus_sleep =
			pinctrl_lookup_state(chip->pinctrl, "dis_vbus_sleep");
		if (IS_ERR_OR_NULL(chip->dis_vbus_sleep)) {
			chg_err("get dis_vbus_sleep fail\n");
			goto free_dis_vbus_gpio;
		}
		pinctrl_select_state(chip->pinctrl, chip->dis_vbus_sleep);
	} else {
		chg_err("dis_vbus_gpio not specified\n");
	}

	chip->usbtemp_dischg_gpio = of_get_named_gpio(node, "oplus,dischg-gpio", 0);
	if (gpio_is_valid(chip->usbtemp_dischg_gpio)) {
		rc = gpio_request(chip->usbtemp_dischg_gpio, "sgm41515_dischg-gpio");
		if (rc < 0) {
			chg_err("event_gpio request error, rc=%d\n", rc);
			goto free_dis_vbus_gpio;
		}
		chip->usbtemp_dischg_enable =
			pinctrl_lookup_state(chip->pinctrl, "dischg_enable");
		if (IS_ERR_OR_NULL(chip->usbtemp_dischg_enable)) {
			chg_err("get usbtemp_dischg_enable fail\n");
			goto free_usbtemp_dischg_gpio;
		}
		chip->usbtemp_dischg_disable =
			pinctrl_lookup_state(chip->pinctrl, "dischg_disable");
		if (IS_ERR_OR_NULL(chip->usbtemp_dischg_disable)) {
			chg_err("get usbtemp_dischg_disable fail\n");
			goto free_usbtemp_dischg_gpio;
		}
		pinctrl_select_state(chip->pinctrl, chip->usbtemp_dischg_disable);
	} else {
		chg_err("usbtemp_dischg_gpio not specified\n");
	}

	chip->event_gpio = of_get_named_gpio(node, "oplus,event-gpio", 0);
	if (!gpio_is_valid(chip->event_gpio)) {
		chg_err("event_gpio not specified\n");
		rc = -ENODEV;
		goto free_usbtemp_dischg_gpio;
	}
	rc = gpio_request(chip->event_gpio, "sgm41515_event-gpio");
	if (rc < 0) {
		chg_err("event_gpio request error, rc=%d\n", rc);
		goto free_usbtemp_dischg_gpio;
	}
	chip->event_default =
		pinctrl_lookup_state(chip->pinctrl, "event_default");
	if (IS_ERR_OR_NULL(chip->event_default)) {
		chg_err("get event_default fail\n");
		goto free_event_gpio;
	}
	gpio_direction_input(chip->event_gpio);
	pinctrl_select_state(chip->pinctrl, chip->event_default);
	chip->event_irq = gpio_to_irq(chip->event_gpio);
	rc = devm_request_irq(chip->dev, chip->event_irq,
			      sgm41515_event_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			      "sgm41515_event-irq", chip);
	if (rc < 0) {
		chg_err("event_irq request error, rc=%d\n", rc);
		goto free_event_gpio;
	}
	chip->event_irq_enabled = true;
	sgm41515_enable_irq(chip, false);

	return 0;

free_event_gpio:
	if (gpio_is_valid(chip->event_gpio)) {
		gpio_free(chip->event_gpio);
		chip->event_gpio =  -EINVAL;
	}
free_usbtemp_dischg_gpio:
	if (gpio_is_valid(chip->usbtemp_dischg_gpio)) {
		gpio_free(chip->usbtemp_dischg_gpio);
		chip->usbtemp_dischg_gpio =  -EINVAL;
	}
free_dis_vbus_gpio:
	if (gpio_is_valid(chip->dis_vbus_gpio)) {
		gpio_free(chip->dis_vbus_gpio);
		chip->dis_vbus_gpio =  -EINVAL;
	}

	return rc;
}

static int sgm41515_batfet_reset_disable(struct sgm41515_chip *chip, bool enable)
{
	int rc = 0;
	int val = 0;

	if(enable)
		val = SGM41515_BATFET_RST_DISABLE << REG07_SGM41515_BATFET_RST_EN_SHIFT;
	else
		val = SGM41515_BATFET_RST_ENABLE << REG07_SGM41515_BATFET_RST_EN_SHIFT;

	rc = sgm41515_write_byte_mask(chip, REG07_SGM41515_ADDRESS, REG07_SGM41515_BATFET_RST_EN_MASK, val);

	return rc;
}

static bool sgm41515_get_deivce_online(struct sgm41515_chip *chip)
{
	int rc = 0;
	u8 reg_val = 0;

	if (!chip)
		return 0;

	if(atomic_read(&chip->charger_suspended) == 1)
		return 0;

	chip->part_id = 0xff;
	rc = sgm41515_read_byte(chip, REG0B_SGM41515_ADDRESS, &reg_val);
	if (rc) {
		rc = sgm41515_read_byte(chip, REG0B_SGM41515_ADDRESS, &reg_val);
		if (rc) {
			chg_err("Couldn't read REG0B_SGM41515_ADDRESS rc = %d\n", rc);
			return false;
		}
	}

	chip->part_id = (reg_val & REG0B_SGM41515_PN_MASK) >> SGM41515_DEVID_SHIFT;
	chg_err("sgm41515 part_id=0x%02X\n", chip->part_id);

	if (chip->part_id == SGM41541_PART_ID || chip->part_id == SGM41515_PART_ID ||
	    chip->part_id == SGM41515D_PART_ID || chip->part_id == SGM41512SD_PART_ID)
		return true;

	return false;
}

static int sgm41515_set_ovp(struct sgm41515_chip *chip, int val)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_write_byte_mask(chip, REG06_SGM41515_ADDRESS,
			REG06_SGM41515_OVP_MASK, val);

	return rc;
}

static int sgm41515_set_chg_timer(struct sgm41515_chip *chip, bool enable)
{
	int rc = 0;

	if (!chip)
		return 0;

	if(atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_write_byte_mask(chip, REG05_SGM41515_ADDRESS,
			REG05_SGM41515_CHG_SAFETY_TIMER_MASK,
			enable ? REG05_SGM41515_CHG_SAFETY_TIMER_ENABLE : REG05_SGM41515_CHG_SAFETY_TIMER_DISABLE);
	if (rc)
		chg_err("Couldn't sgm41515_set_chg_timer rc = %d\n", rc);

	return rc;
}

static int sgm41515_set_int_mask(struct sgm41515_chip *chip, int val)
{
	int rc = 0;

	if (!chip)
		return 0;

	if(atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_write_byte_mask(chip, REG0A_SGM41515_ADDRESS,
			REG0A_SGM41515_VINDPM_INT_MASK | REG0A_SGM41515_IINDPM_INT_MASK,
			val);
	if (rc)
		chg_err("Couldn't sgm41515 set_int_mask rc = %d\n", rc);

	return rc;
}

static int sgm41515_set_stat_dis(struct sgm41515_chip *chip, bool enable)
{
	int rc = 0;

	if (!chip)
		return 0;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = sgm41515_write_byte_mask(chip, REG00_SGM41515_ADDRESS,
			REG00_SGM41515_STAT_DIS_MASK,
			enable ? REG00_SGM41515_STAT_DIS_ENABLE : REG00_SGM41515_STAT_DIS_DISABLE);
	if (rc)
		chg_err("Couldn't sgm41515 set_stat_dis rc = %d\n", rc);

	return rc;
}

static int sgm41515_hw_init(struct sgm41515_chip *chip)
{
	chg_err("init sgm41515 hardware! \n");

	if (!chip)
		return 0;

	/*must be before set_vindpm_vol and set_input_current*/
	chip->hw_aicl_point = HW_AICL_POINT_DEFAULT;
	chip->sw_aicl_point = SW_AICL_POINT_DEFAULT;
	chip->pre_aicl_index = 0;

	sgm41515_set_stat_dis(chip, true);
	sgm41515_set_int_mask(chip, REG0A_SGM41515_VINDPM_INT_NOT_ALLOW | REG0A_SGM41515_IINDPM_INT_NOT_ALLOW);
	sgm41515_set_chg_timer(chip, false);
	sgm41515_disable_charging(chip);
	sgm41515_set_ovp(chip, REG06_SGM41515_OVP_14P0V);
	sgm41515_set_chging_term_disable(chip);
	sgm41515_float_voltage_write(chip, SGM41515_DEFAULT_TERMINATION_VOLTAGE);
	sgm41515_otg_ilim_set(chip, REG02_SGM41515_BOOSTI_1200MA);
	sgm41515_set_prechg_voltage_threshold(chip);
	sgm41515_set_prechg_current(chip, SGM41515_DEFAULT_PRECHG_CURRENT);
	sgm41515_charging_current_write_fast(chip, REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_500MA);
	sgm41515_set_termchg_current(chip, 200);
	sgm41515_input_current_limit_without_aicl(chip, REG00_SGM41515_INIT_INPUT_CURRENT_LIMIT_500MA);
	sgm41515_set_rechg_voltage(chip, 1);
	sgm41515_set_vindpm_vol(chip);
	sgm41515_set_otg_voltage(chip, REG06_SGM41515_OTG_VLIM_5000MV);
	sgm41515_batfet_reset_disable(chip, true);
	sgm41515_unsuspend_charger(chip);
	sgm41515_enable_charging(chip);
	sgm41515_set_wdt_timer(chip, REG05_SGM41515_WATCHDOG_TIMER_DISABLE);
	sgm41515_enable_hiz_mode(chip, false);

	return 0;
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
static const struct charger_properties  sgm41515_chg_props = {
	.alias_name = "sgm41515",
};

static int sgm41515_plug_in(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sgm41515_chip *chip = dev_get_drvdata(&chg_dev->dev);

	if (!chip)
		return -EINVAL;

	ret = sgm41515_enable_charging(chip);
	if (ret) {
		chg_err("failed to enable charging:%d", ret);
	}
	return ret;
}

static int sgm41515_plug_out(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sgm41515_chip *chip = dev_get_drvdata(&chg_dev->dev);

	if (!chip)
		return -EINVAL;

	ret = sgm41515_disable_charging(chip);

	if (ret)
		chg_err("failed to disable charging:%d", ret);

	return ret;
}

static int sgm41515_charge_kick_wdt(struct charger_device *chg_dev)
{
	struct sgm41515_chip *chip = dev_get_drvdata(&chg_dev->dev);

	if (!chip)
		return -EINVAL;

	return sgm41515_kick_watchdog(chip);
}

static int sgm41515_charge_enable(struct charger_device *chg_dev, bool en)
{
	struct sgm41515_chip *chip = dev_get_drvdata(&chg_dev->dev);

	if (!chip)
		return -EINVAL;

	if (en)
		return sgm41515_enable_charging(chip);
	else
		return sgm41515_disable_charging(chip);
}

static struct charger_ops sgm41515_chg_ops = {
	.plug_in = sgm41515_plug_in,
	.plug_out = sgm41515_plug_out,
	.kick_wdt = sgm41515_charge_kick_wdt,
	.enable = sgm41515_charge_enable,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
static enum power_supply_usb_type sgm41515_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};
#endif

static enum power_supply_property sgm41515_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static int sgm41515_charger_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sgm41515_chip *chip;
	int ret = 0;
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int boot_mode = get_boot_mode();
#endif

	chip = power_supply_get_drvdata(psy);

	if (!chip) {
		chg_info("oplus_chip not ready!\n");
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->power_good;
		break;
	case POWER_SUPPLY_PROP_TYPE:
	case POWER_SUPPLY_PROP_USB_TYPE:
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
		if (boot_mode == META_BOOT) {
			val->intval = POWER_SUPPLY_TYPE_USB;
		} else {
			val->intval = chip->charge_type;
		}
#endif
		chg_info("sgm41515 get power_supply_type = %d\n", val->intval);
		break;
	default:
		ret = -ENODATA;
	}
	return ret;
}

static char *sgm41515_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static const struct power_supply_desc sgm41515_charger_desc = {
	.type	= POWER_SUPPLY_TYPE_USB,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	.usb_types	= sgm41515_charger_usb_types,
	.num_usb_types	= ARRAY_SIZE(sgm41515_charger_usb_types),
#else
	.usb_types		= BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
				  BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_C)       |
				  BIT(POWER_SUPPLY_USB_TYPE_PD)      |
				  BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
				  BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
#endif
	.properties	= sgm41515_charger_properties,
	.num_properties	= ARRAY_SIZE(sgm41515_charger_properties),
	.get_property	= sgm41515_charger_get_property,
};

static int sgm41515_chg_init_psy(struct sgm41515_chip *chip)
{
	struct power_supply_config cfg = {
		.drv_data = chip,
		.of_node = chip->dev->of_node,
		.supplied_to = sgm41515_charger_supplied_to,
		.num_supplicants = ARRAY_SIZE(sgm41515_charger_supplied_to),
	};

	chg_err("%s\n", __func__);
	memcpy(&chip->psy_desc, &sgm41515_charger_desc, sizeof(chip->psy_desc));
	chip->psy_desc.name = "sgm41515";
	chip->chg_psy = devm_power_supply_register(chip->dev, &chip->psy_desc, &cfg);
	return IS_ERR(chip->chg_psy) ? PTR_ERR(chip->chg_psy) : 0;
}

static int sgm41515_parse_dt(struct sgm41515_chip *chip)
{
	if (of_property_read_string(chip->client->dev.of_node, "charger_name", &chip->chg_dev_name) < 0) {
		chip->chg_dev_name = "primary_chg";
		chg_err("no charger name\n");
	}

	chip->usb_aicl_enhance = of_property_read_bool(chip->client->dev.of_node, "oplus,usb_aicl_enhance");
	chg_info("usb_aicl_enhance:%d", chip->usb_aicl_enhance);
	return 0;
}
#endif

#define THERMAL_TEMP_DEFAULT_VALUE 25000
static int oplus_ntc_convert(int volt, int compensation)
{
	int i;
	int size;
	size = ARRAY_SIZE(chg_ntc_temp_table) - 1;
	for (i = 1; i <= size; (i = i + 2)) {
		if (i >= size || chg_ntc_temp_table[i] <= volt)
			break;
	}
	if (i >= size)
		i = size;
	i = i -1;

	return chg_ntc_temp_table[i] * 1000;
}

static int oplus_chg_v2_get_chg_temp(struct thermal_zone_device *tz, int *temp)
{
	int rc;
	int ntcctrl_gpio_value;
	int chg_thermal_vol;
	struct sgm41515_chip *chip;

	if (!temp) {
		chg_err("temp is null\n");
		return 0;
	}

	if (IS_ERR_OR_NULL(tz) || !tz->devdata) {
		chg_err("tz is null\n");
		*temp = THERMAL_TEMP_DEFAULT_VALUE;
		return 0;
	}
	chip = (struct sgm41515_chip *)tz->devdata;

	if (!chip || IS_ERR_OR_NULL(chip->chg_temp_adc)) {
		chg_err("chip is null\n");
		*temp = THERMAL_TEMP_DEFAULT_VALUE;
		return 0;
	}

	if (gpio_is_valid(chip->ntcctrl_gpio_amux)) {
		ntcctrl_gpio_value = gpio_get_value(chip->ntcctrl_gpio_amux);
		if (ntcctrl_gpio_value) {
			*temp = THERMAL_TEMP_DEFAULT_VALUE;
			return 0;
		}
	}

	rc = iio_read_channel_processed(chip->chg_temp_adc, &chg_thermal_vol);
	if (rc < 0) {
		chg_err("[%s]fail to read chg temp adc rc = %d\n", __func__, rc);
		*temp = THERMAL_TEMP_DEFAULT_VALUE;
		return 0;
	}

	chg_thermal_vol = chg_thermal_vol / 1000;
	*temp = oplus_ntc_convert(chg_thermal_vol, 0);

	chg_debug("chg_vol:%d chg_temp:%d,\n", chg_thermal_vol, *temp);

	return 0;
}


static struct thermal_zone_device_ops charger_temp_ops = {
	.get_temp = oplus_chg_v2_get_chg_temp,
};

static int register_charger_thermal(struct sgm41515_chip *chip)
{
	int ret = 0;

	if (!chip) {
		chg_err("chip not ready!\n");
		return -ENODEV;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	chip->tz_dev = thermal_tripless_zone_device_register("charger_temp_v2",
			chip, &charger_temp_ops, NULL);
#else
	chip->tz_dev = thermal_zone_device_register("charger_temp_v2",
			0, 0, chip, &charger_temp_ops, NULL, 0, 0);
#endif
	if (IS_ERR(chip->tz_dev)) {
		chg_err("charger_temp register fail");
		return -ENODEV;
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	ret = thermal_zone_device_enable(chip->tz_dev);
	if (ret) {
		thermal_zone_device_unregister(chip->tz_dev);
		chg_err("thermal_zone_device_enable fail\n");
	}
#endif
	return ret;
}

static int oplus_v2_ntc_switch_gpio_init(struct sgm41515_chip *chip)
{
	int rc;
	struct device_node *node;

	if (!chip) {
		chg_err("ntc_switch not ready!\n");
		return -EINVAL;
	}

	node = chip->dev->of_node;

	rc = of_property_match_string(node, "io-channel-names", "chg_temp_adc");
	if (rc >= 0) {
		chip->chg_temp_adc = iio_channel_get(chip->dev, "chg_temp_adc");
		if (IS_ERR(chip->chg_temp_adc)) {
			rc = PTR_ERR(chip->chg_temp_adc);
			chg_err("chg_temp_adc get error, rc=%d\n", rc);
			chip->chg_temp_adc = NULL;
			goto err;
		}
	} else {
		chg_err("chg_temp_adc not found\n");
		chip->chg_temp_adc = NULL;
		goto err;
	}

	rc = read_signed_data_from_node(node, "chg_ntc_para", (s32 *)chg_ntc_temp_table,
		CHG_NTC_TEMP_COUNT);
	if (rc < 0) {
		chg_err("get ntc para fail  rc:%d\n", rc);
		goto release_adc_channel;
	}

	chip->ntcctrl_gpio_amux = of_get_named_gpio(node, "qcom,ntc-switch-gpio-amux", 0);
	if (chip->ntcctrl_gpio_amux < 0)
		chg_err("chip->ntcctrl_gpio not specified\n");
	else
		chg_info("chip->ntcctrl_gpio_amux = %d\n", chip->ntcctrl_gpio_amux);

	if (gpio_is_valid(chip->ntcctrl_gpio_amux)) {
		rc = gpio_request(chip->ntcctrl_gpio_amux, "ntc-switch-gpio_amux");
		if (rc) {
			chg_err("unable to request gpio [%d]\n", rc);
			goto release_adc_channel;
		}
	}

	return 0;

release_adc_channel:
	if (!IS_ERR_OR_NULL(chip->chg_temp_adc)) {
		iio_channel_release(chip->chg_temp_adc);
		chip->chg_temp_adc = NULL;
	}
err:
	chg_err("thermal adc and gpio init fail!\n");
	return -1;
}

static void release_charger_thermal_resource(struct sgm41515_chip *chip)
{
	if (!chip) {
		chg_err("chip not ready!\n");
		return;
	}

	if (!IS_ERR_OR_NULL(chip->tz_dev)) {
		thermal_zone_device_unregister(chip->tz_dev);
		chip->tz_dev = NULL;
	}
	if (!IS_ERR_OR_NULL(chip->chg_temp_adc)) {
		iio_channel_release(chip->chg_temp_adc);
		chip->chg_temp_adc = NULL;
	}
	if (gpio_is_valid(chip->ntcctrl_gpio_amux)) {
		gpio_free(chip->ntcctrl_gpio_amux);
	}
}

static void sgm41515_free_gpio(struct sgm41515_chip *chip)
{
	if (!chip) {
		chg_err("chip not ready!\n");
		return;
	}

	sgm41515_enable_irq(chip, false);
	if (gpio_is_valid(chip->event_gpio)) {
		gpio_free(chip->event_gpio);
		chip->event_gpio = -EINVAL;
	}
	if (gpio_is_valid(chip->usbtemp_dischg_gpio)) {
		gpio_free(chip->usbtemp_dischg_gpio);
		chip->usbtemp_dischg_gpio =  -EINVAL;
	}
	if (gpio_is_valid(chip->dis_vbus_gpio)) {
		gpio_free(chip->dis_vbus_gpio);
		chip->dis_vbus_gpio = -EINVAL;
	}
}

static void sgm41515_free_wakeup_source(struct sgm41515_chip *chip)
{
	if (!chip) {
		chg_err("chip not ready!\n");
		return;
	}

	if (chip->suspend_ws) {
		wakeup_source_unregister(chip->suspend_ws);
		chip->suspend_ws = NULL;
	}
	if (chip->keep_resume_ws) {
		wakeup_source_unregister(chip->keep_resume_ws);
		chip->keep_resume_ws = NULL;
	}
}

static void sgm41515_chip_state_init(struct sgm41515_chip *chip)
{
	if (!chip)
		return;

	atomic_set(&chip->charger_suspended, 0);
	atomic_set(&chip->is_suspended, 0);
	atomic_set(&chip->shutdown_mask, 0);
	chip->charge_type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->power_good = false;
	chip->bc12_complete = false;
	chip->bc12_retried = 0;
	chip->bc12_delay_cnt = 0;
}

static void sgm41515_work_init(struct sgm41515_chip *chip)
{
	INIT_DELAYED_WORK(&chip->event_work, sgm41515_event_work);
	INIT_DELAYED_WORK(&chip->bc12_timeout_work, sgm41515_bc12_timeout_work);
	INIT_DELAYED_WORK(&chip->bc12_retry_work, sgm41515_bc12_retry_work);
	INIT_DELAYED_WORK(&chip->bc12_plugin_work, sgm41515_bc12_plugin_work);
	INIT_WORK(&chip->rerun_votable_work, sgm41515_rerun_votable_work);
	INIT_DELAYED_WORK(&chip->vbus_check_work, sgm41515_vbus_check_work);
}

static void sgm41515_completion_init(struct sgm41515_chip *chip)
{
	init_completion(&chip->bc12_complete_comp);
	reinit_completion(&chip->bc12_complete_comp);
	init_completion(&chip->resume_done);
}

struct oplus_chg_ic_virq sgm41515_reverse_chg_virq_table[] = {
	{.virq_id = OPLUS_IC_VIRQ_ERR },
	{.virq_id = OPLUS_IC_VIRQ_ONLINE },
	{.virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int sgm41515_reverse_chg_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);
	return 0;
}

static int sgm41515_reverse_chg_exit(struct oplus_chg_ic_dev *ic_dev)
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

static void *sgm41515_reverse_chg_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) && (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, sgm41515_reverse_chg_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, sgm41515_reverse_chg_exit);
		break;
	case OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP, sgm41515_set_otg_boost_curr_limit);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static int oplus_pd_parse_child_node_ic_info(struct device_node *child,
					      enum oplus_chg_ic_type *ic_type,
					      int *ic_index)
{
	int ret;

	ret = of_property_read_u32(child, "oplus,ic_type", ic_type);
	if (ret >= 0) {
		ret = of_property_read_u32(child, "oplus,ic_index", ic_index);
		if (ret >= 0)
			chg_info("node %s, ic_type = %d, ic_index = %d\n", child->name, *ic_type, *ic_index);
		else
			chg_err("can't get %s ic index, ret=%d\n", child->name, ret);
	} else {
		chg_err("can't get %s ic type, ret=%d\n", child->name, ret);
	}
	return ret;
}

static void sgm41515_register_child_ic(struct sgm41515_chip *chip,
					     struct device_node *child)
{
	int ret;
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct oplus_chg_ic_dev *ic_dev;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };

	ret = oplus_pd_parse_child_node_ic_info(child, &ic_type, &ic_index);
	if(ret < 0)
		return;

	ic_cfg.name = child->name;
	ic_cfg.index = ic_index;
	ic_cfg.type = ic_type;
	ic_cfg.of_node = child;

	switch (ic_type) {
	case OPLUS_CHG_IC_REVERSE:
		snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "reverse-sgm41515");
		snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
		ic_cfg.get_func = sgm41515_reverse_chg_get_func;
		ic_cfg.virq_data = sgm41515_reverse_chg_virq_table;
		ic_cfg.virq_num = ARRAY_SIZE(sgm41515_reverse_chg_virq_table);
		break;
	default:
		chg_err("not support ic_type(=%d)\n", ic_type);
		return;
	}

	ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!ic_dev) {
		ret = -ENODEV;
		chg_err("register %s error\n", child->name);
		return;
	}
	chg_info("register %s success.\n", child->name);

	switch (ic_type) {
	case OPLUS_CHG_IC_REVERSE:
		chip->reverse_chg_ic_dev = ic_dev;
		oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_INIT);
		break;
	default:
		chg_err("not support ic_type(=%d) ...\n", ic_type);
		return;
	}
	of_platform_populate(child, NULL, NULL, chip->dev);
}


static void sgm41515_init_locks(struct sgm41515_chip *chip)
{
	mutex_init(&chip->i2c_lock);
	mutex_init(&chip->dpdm_lock);
	mutex_init(&chip->pinctrl_lock);
}

static void sgm41515_destroy_locks(struct sgm41515_chip *chip)
{
	mutex_destroy(&chip->pinctrl_lock);
	mutex_destroy(&chip->dpdm_lock);
	mutex_destroy(&chip->i2c_lock);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static int sgm41515_driver_probe(struct i2c_client *client)
#else
static int sgm41515_driver_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
#endif
{
	int rc = 0;
	struct sgm41515_chip *chip;
	struct device_node *node = client->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *child;

	chip = devm_kzalloc(&client->dev, sizeof(struct sgm41515_chip),
			    GFP_KERNEL);
	if (!chip) {
		chg_err("kzalloc failed\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);
	sgm41515_init_locks(chip);
	sgm41515_work_init(chip);
	sgm41515_completion_init(chip);

	chip->dpdm_reg = devm_regulator_get_optional(chip->dev, "dpdm");
	if (IS_ERR(chip->dpdm_reg)) {
		rc = PTR_ERR(chip->dpdm_reg);
		chg_err("Couldn't get dpdm regulator, rc=%d\n", rc);
		chip->dpdm_reg = NULL;
	}

	chip->regmap = devm_regmap_init_i2c(client, &sgm41515_regmap_config);
	if (!chip->regmap) {
		rc = -ENODEV;
		goto regmap_init_err;
	}

	sgm41515_chip_state_init(chip);
	oplus_chg_awake_init(chip);
	init_waitqueue_head(&chip->wait);
	oplus_keep_resume_awake_init(chip);

	rc = sgm41515_get_deivce_online(chip);
	if (rc < 0) {
		chg_err("!!!sgm41515 is not detected\n");
		goto gpio_init_err;
	}

	rc = sgm41515_gpio_init(chip);
	if (rc < 0) {
		chg_err("gpio init error, rc=%d\n", rc);
		goto gpio_init_err;
	}

	oplus_v2_ntc_switch_gpio_init(chip);
	if (!IS_ERR_OR_NULL(chip->chg_temp_adc))
		register_charger_thermal(chip);

	sgm41515_hw_init(chip);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	sgm41515_parse_dt(chip);
	rc = sgm41515_chg_init_psy(chip);
	if (rc)
		chg_err("Failed to register sgm41515 ret=%d\n", rc);

	chip->chg_dev = charger_device_register(chip->chg_dev_name,
						&client->dev, chip,
						&sgm41515_chg_ops,
						&sgm41515_chg_props);
	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		rc = PTR_ERR(chip->chg_dev);
		goto err_device_register;
	}
#endif

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
	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "buck-sgm41515");
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = sgm41515_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(sgm41515_virq_table);
	ic_cfg.of_node = node;
	chip->ic_dev =
		devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto reg_ic_err;
	}
	for_each_child_of_node(node, child)
		sgm41515_register_child_ic(chip, child);
	oplus_mms_wait_topic("wired", sgm41515_subscribe_wired_topic, chip);

	sgm41515_enable_irq(chip, true);
	chg_info("success\n");

	return rc;

reg_ic_err:
#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (chip->chg_dev)
		charger_device_unregister(chip->chg_dev);
err_device_register:
#endif
	release_charger_thermal_resource(chip);
	sgm41515_free_gpio(chip);
gpio_init_err:
	sgm41515_free_wakeup_source(chip);
regmap_init_err:
	cancel_work_sync(&chip->rerun_votable_work);
	sgm41515_destroy_locks(chip);
	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, chip);
	chip = NULL;
	chg_err("probe error, rc=%d\n", rc);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void sgm41515_driver_remove(struct i2c_client *client)
#else
static int sgm41515_driver_remove(struct i2c_client *client)
#endif
{
	struct sgm41515_chip *chip = i2c_get_clientdata(client);

	if (chip) {
		cancel_work_sync(&chip->rerun_votable_work);
		sgm41515_free_gpio(chip);
#ifdef CONFIG_OPLUS_CHARGER_MTK
		if (chip->chg_dev)
			charger_device_unregister(chip->chg_dev);
#endif
		release_charger_thermal_resource(chip);
		if (!IS_ERR_OR_NULL(chip->wired_subs))
			oplus_mms_unsubscribe(chip->wired_subs);
		if (chip->ic_dev)
			devm_oplus_chg_ic_unregister(chip->dev, chip->ic_dev);
		chip->ic_dev = NULL;
		sgm41515_free_wakeup_source(chip);
		mutex_destroy(&chip->pinctrl_lock);
		mutex_destroy(&chip->dpdm_lock);
		mutex_destroy(&chip->i2c_lock);
		i2c_set_clientdata(client, NULL);
		devm_kfree(&client->dev, chip);
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	return 0;
#endif
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int sgm41515_pm_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct sgm41515_chip *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("chip is null\n");
		return 0;
	}

	atomic_set(&chip->charger_suspended, 0);
	complete_all(&chip->resume_done);
	return 0;
}

static int sgm41515_pm_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct sgm41515_chip *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("chip is null\n");
		return 0;
	}

	reinit_completion(&chip->resume_done);
	atomic_set(&chip->charger_suspended, 1);
	return 0;
}

static const struct dev_pm_ops sgm41515_pm_ops = {
	.resume = sgm41515_pm_resume,
	.suspend = sgm41515_pm_suspend,
};
#else
static int sgm41515_resume(struct i2c_client *client)
{
	struct sgm41515_chip *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("chip is null\n");
		return 0;
	}

	atomic_set(&chip->charger_suspended, 0);
	complete_all(&chip->resume_done);
	return 0;
}

static int sgm41515_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct sgm41515_chip *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("chip is null\n");
		return 0;
	}

	reinit_completion(&chip->resume_done);
	atomic_set(&chip->charger_suspended, 1);
	return 0;
}
#endif

static void sgm41515_shutdown(struct i2c_client *client)
{
	struct sgm41515_chip *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("chip is null\n");
		return;
	}

	sgm41515_charging_current_write_fast(chip, REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_500MA);

	if(oplus_wired_shipmode_is_enabled()) {
		chg_info("oplus_wired_shipmode_is_enabled\n");
		sgm41515_set_shipmode(chip, true);
	}

	atomic_set(&chip->shutdown_mask, 1);
	/*
	 * HIZ mode needs to be disabled on shutdown to ensure activation
	 * signal is available.
	 */
	if (READ_ONCE(chip->vbus_present))
		(void)sgm41515_write_byte_mask(chip, REG00_SGM41515_ADDRESS, REG00_SGM41515_HIZ_MODE_MASK,
						REG00_SGM41515_HIZ_MODE_DISABLE);
	sgm41515_set_stat_dis(chip, true);
	sgm41515_enable_charging(chip);
}

static const struct of_device_id sgm41515_match[] = {
	{ .compatible = "oplus,sgm41515-charger" },
	{},
};

static const struct i2c_device_id sgm41515_id[] = {
	{ "sgm41515-charger", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm41515_id);

static struct i2c_driver sgm41515_i2c_driver = {
	.driver		= {
		.name = "sgm41515-charger",
		.owner	= THIS_MODULE,
		.of_match_table = sgm41515_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm 	= &sgm41515_pm_ops,
#endif
	},
	.probe		= sgm41515_driver_probe,
	.remove		= sgm41515_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= sgm41515_resume,
	.suspend	= sgm41515_suspend,
#endif
	.shutdown	= sgm41515_shutdown,
	.id_table	= sgm41515_id,
};

int sgm41515_driver_init(void)
{
	int rc;

	rc = i2c_add_driver(&sgm41515_i2c_driver);
	if (rc < 0)
		chg_err("failed to register sgm41515 i2c driver, rc=%d\n", rc);
	else
		chg_debug("Success to register sgm41515 i2c driver.\n");

	return rc;
}

void sgm41515_driver_exit(void)
{
	i2c_del_driver(&sgm41515_i2c_driver);
}
oplus_chg_module_register(sgm41515_driver);

MODULE_DESCRIPTION("Driver for sgm41515 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:sgm41515-charger");
MODULE_SOFTDEP("pre: qcom-spmi-adc5");
