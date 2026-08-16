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
#include <linux/power_supply.h>
#include <linux/iio/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/pm_wakeup.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/reboot.h>
#include <linux/sched/clock.h>
#include <linux/timer.h>
#include <linux/thermal.h>

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
#include <ufcs_class.h>
#include "../voocphy/oplus_voocphy.h"
#include "oplus_hal_sc6607.h"

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mtk_boot_common.h>
#include "charger_class.h"
#else
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#endif
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
#include <linux/pinctrl/consumer.h>
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
#include "thermal_core.h"
#endif

struct sc6607_alert_handler {
	u32 bit_mask;
	int (*handler)(struct sc6607 *);
};

struct sc6607_temp_param {
	__s32 bts_temp;
	__s32 temperature_r;
};

struct sc6607_ntc_temp{
	struct sc6607_temp_param *pst_temp_table;
	int table_size;
};

struct sc6607_track_check_reg {
	u8 addr;
	u8 data;
};

struct tsbus_charger_temp {
	struct thermal_zone_device *tzd;
};

static const u32 sy6607_adc_step[] = {
	2500, 3750, 5000, 1250, 1250, 1220, 1250, 9766, 9766, 5, 156,
};

static int usb_icl[] = {
	100, 500, 900, 1200, 1500, 1750, 2000, 3000,
};

static struct sc6607_temp_param pst_temp_table_1000k[TEMP_TABLE_100K_SIZE] = {{0, 0}, };
static struct sc6607_temp_param pst_temp_table[] = {
	{34, 4966},
	{40, 4043},
	{45, 3396},
	{50, 2847},
	{55, 2386},
	{60, 2000},
	{65, 1678},
	{70, 1411},
	{75, 1189},
	{76, 1149},
	{77, 1111},
	{78, 1074},
	{79, 1039},
	{80, 1005},
	{81, 971},
	{82, 940},
	{83, 909},
	{84, 879},
	{85, 851},
	{90, 723},
	{95, 616},
	{100, 527},
	{110, 389},
	{120, 291},
	{125, 253},
};

static const struct regmap_config sc6607_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const char *const state_str[] = {
	"bc1.2 detect Init",
	"non-standard adapter detection",
	"floating Detection",
	"bc1.2 Primary Detection",
	"hiz set",
	"bc1.2 Secondary Detection",
	"hvdcp hanke",
};

static const char *const dpdm_str[] = {
	"0v to 0.325v",
	"0.325v to 1v",
	"1v to 1.35v",
	"1.35v to 2.2v",
	"2.2v to 3v",
	"higher than 3v",
};

static void sc6607_check_ic_suspend(struct sc6607 *chip);
static int oplus_sc6607_charger_suspend(struct sc6607 *chip);
static int sc6607_dpdm_irq_handle(struct sc6607 *chip);
static int bc12_detect_run(struct sc6607 *chip);
static int sc6607_track_match_hk_err(struct sc6607 *chip, u8 data);
static int sc6607_init_device(struct sc6607 *chip);
static int sc6607_force_dpdm(struct sc6607 *chip, bool enable);
static void oplus_notify_hvdcp_detach_stat(struct sc6607 *chip);
static int sc6607_input_present(struct oplus_chg_ic_dev *ic_dev, bool *present);
static int sc6607_get_bc12_result(struct oplus_chg_ic_dev *ic_dev, int *type);
static int sc6607_set_aicl_point(struct oplus_chg_ic_dev *ic_dev, int vbatt);
static int sc6607_tsbus_tsbat_to_convert(struct sc6607 *chip, u64 adc_value, int adc_module);
#ifndef CONFIG_OPLUS_CHARGER_MTK
static int sc6607_request_dpdm(struct sc6607 *chip, bool enable);
#endif

#ifdef CONFIG_OPLUS_CHARGER_MTK
static const struct charger_properties  sc6607_chg_props = {
	.alias_name = "sc6607",
};
#endif

static int oplus_chg_suspend_charger(bool suspend, const char *client_str)
{
	struct votable *suspend_votable;
	int rc;

	suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	if (!suspend_votable) {
		chg_err("WIRED_CHARGE_SUSPEND votable not found\n");
		return -EINVAL;
	}

	rc = vote(suspend_votable, client_str, suspend, 1, false);
	if (rc < 0)
		chg_err("%s charger error, rc = %d\n",
		        suspend ? "suspend" : "unsuspend", rc);
	else
		chg_info("%s charger\n", suspend ? "suspend" : "unsuspend");

	return rc;
}

static int oplus_chg_set_icl_by_vote(int icl, const char *client_str)
{
	struct votable *icl_votable;
	int rc;

	icl_votable = find_votable("WIRED_ICL");
	if (!icl_votable) {
		chg_err("WIRED_ICL votable not found\n");
		return -EINVAL;
	}

	rc = vote(icl_votable, client_str, true, icl, true);
	if (rc < 0)
		chg_err("set icl error: icl = %d, rc = %d\n", icl, rc);
	else
		chg_info("real icl = %d\n", icl);

	return rc;
}

static void oplus_charger_suspend_recovery_work(struct work_struct *work)
{
	chg_info("voted suspend recovery, unsuspend\n");
	oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
	oplus_chg_suspend_charger(false, USB_IBUS_DRAW_VOTER);
	oplus_chg_suspend_charger(false, TCPC_IBUS_DRAW_VOTER);
}

static int oplus_get_max_current_from_fixed_pdo(struct sc6607 *chip, int volt)
{
	int i = 0;
	if (chip->pdo[0].pdo_data == 0) {
		chg_err("get pdo info error\n");
		return -EINVAL;
	}

	for (i = 0; i < (PPS_PDO_MAX - 1); i++) {
		if (chip->pdo[i].pdo_type != USBPD_PDMSG_PDOTYPE_FIXED_SUPPLY)
			continue;

		if (volt <= PD_PDO_VOL(chip->pdo[i].voltage_50mv)) {
			chg_info("SourceCap[%d]: %08X, FixedSupply PDO V=%d mV, I=%d mA,"
				"UsbCommCapable=%d, USBSuspendSupported:%d\n", i,
				chip->pdo[i].pdo_data, PD_PDO_VOL(chip->pdo[i].voltage_50mv),
				PD_PDO_CURR_MAX(chip->pdo[i].max_current_10ma),
				chip->pdo[i].usb_comm_capable, chip->pdo[i].usb_suspend_supported);
			return PD_PDO_CURR_MAX(chip->pdo[i].max_current_10ma);
		}
	}
	return -EINVAL;
}

static void oplus_sourcecap_done_work(struct work_struct *work)
{
	struct sc6607 *chip = container_of(work, struct sc6607, sourcecap_done_work.work);
	int max_pdo_current = 0;

	/*set default input current from pdo*/
	max_pdo_current = oplus_get_max_current_from_fixed_pdo(chip, VBUS_5V);
	if (max_pdo_current >= 0)
		oplus_chg_set_icl_by_vote(max_pdo_current, PD_PDO_ICL_VOTER);
}

static int oplus_chg_get_fastchg_commu_ing(void)
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

static bool oplus_pd_sdp_port(void)
{
	struct tcpc_device *tcpc;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc) {
		chg_err("get type_c_port0 fail\n");
		return false;
	}

	if (!tcpm_inquire_pd_connected(tcpc))
		return false;
	return (tcpm_inquire_dpm_flags(tcpc) & DPM_FLAGS_PARTNER_USB_COMM) ? true : false;
}

static bool oplus_pd_dcp_port(void)
{
	struct tcpc_device *tcpc;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc) {
		chg_err("get type_c_port0 fail\n");
		return false;
	}

	if (!tcpm_inquire_pd_connected(tcpc))
		return false;
	return (tcpm_inquire_dpm_flags(tcpc) & DPM_FLAGS_PARTNER_USB_COMM) ? false : true;
}

static void oplus_tcpc_complete_work(struct work_struct *work)
{
	static int retry_count = 0;
	struct tcpc_device *tcpc_dev;
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc6607 *chip = container_of(dwork, struct sc6607, tcpc_complete_work);

	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	if (!tcpc_dev) {
		retry_count++;
		chg_info("type_c_port0 not found retry count=%d\n", retry_count);
		if (retry_count < OPLUS_TCPC_RETRY_COUNT) {
			schedule_delayed_work(&chip->tcpc_complete_work, OPLUS_TCPC_WORK_DELAY);
			return;
		} else {
			return;
		}
	}
	tcpc_device_irq_enable(tcpc_dev);
	return;
}

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

	if (!chip || !chip->regmap_fields[field_id]) {
		chg_err("chip or chip->regmap_fields[field_id] is null\n");
		return -ENODEV;
	}

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
		chg_err("i2c write fail: can't write field %d, %d\n", field_id, ret);

	return ret;
}

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

static int sc6607_bulk_write(struct sc6607 *chip, u8 reg, u8 *val, size_t count)
{
	int ret;
	int retry = SC6607_I2C_RETRY_WRITE_MAX_COUNT;

	ret = regmap_bulk_write(chip->regmap, reg, val, count);
	if (ret < 0) {
		while (retry > 0 && atomic_read(&chip->driver_suspended) == 0) {
			usleep_range(SC6607_I2C_RETRY_DELAY_US, SC6607_I2C_RETRY_DELAY_US);
			ret = regmap_bulk_write(chip->regmap, reg, val, count);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	if (ret < 0)
		chg_err("i2c bulk write failed: can't write 0x%0x, ret:%d\n", reg, ret);

	return ret;
}

static int sc6607_voocphy_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	s32 ret;
	int retry = SC6607_I2C_RETRY_WRITE_MAX_COUNT;
	struct sc6607 *chip;

	chip = i2c_get_clientdata(client);

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		while (retry > 0 && (chip && atomic_read(&chip->driver_suspended) == 0)) {
			usleep_range(SC6607_I2C_RETRY_DELAY_US, SC6607_I2C_RETRY_DELAY_US);
			ret = i2c_smbus_read_byte_data(client, reg);
			if (ret < 0)
				retry--;
			else
				break;
		}
	}

	*data = (u8)ret;

	if (ret < 0) {
		chg_err("i2c bulk write failed: can't write 0x%0x, ret:%d\n", reg, ret);
		return ret;
	}
	return 0;
}

__maybe_unused static int sc6607_read_byte(struct sc6607 *chip, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&chip->i2c_rw_lock);
	ret = sc6607_bulk_read(chip, reg, data, 1);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

__maybe_unused static int sc6607_write_byte(struct sc6607 *chip, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&chip->i2c_rw_lock);
	ret = sc6607_bulk_write(chip, reg, &data, 1);
	mutex_unlock(&chip->i2c_rw_lock);
	if (ret)
		chg_err("failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}

__maybe_unused static int sc6607_read_data(struct sc6607 *chip, u8 addr, u8 *buf, int len)
{
	int ret;

	mutex_lock(&chip->i2c_rw_lock);
	ret = sc6607_bulk_read(chip, addr, buf, len);
	mutex_unlock(&chip->i2c_rw_lock);
	if (ret)
		chg_err("failed: reg=%02X, ret=%d\n", addr, ret);

	return ret;
}

__maybe_unused static int sc6607_write_data(struct sc6607 *chip, u8 addr, u8 *buf, int len)
{
	int ret;

	mutex_lock(&chip->i2c_rw_lock);
	ret = sc6607_bulk_write(chip, addr, buf, len);
	mutex_unlock(&chip->i2c_rw_lock);
	if (ret)
		chg_err("failed: reg=%02X, ret=%d\n", addr, ret);

	return ret;
}

static int sc6607_i2c_addr_read_byte(struct sc6607 *chip, u8 i2c_addr, u8 reg, u8 *val)
{
	struct i2c_msg xfer[2];
	int ret;

	if (!chip || !chip->client || !chip->client->adapter)
		return -EINVAL;

	xfer[0].addr = i2c_addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg;

	xfer[1].addr = i2c_addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = val;

	ret = i2c_transfer(chip->client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		if (ret < 0)
			chg_err("i2c transfer failed, addr=0x%02x, reg=0x%02x, ret=%d\n", i2c_addr, reg, ret);
		else
			chg_err("i2c transfer EIO, addr=0x%02x, reg=0x%02x, ret=%d\n", i2c_addr, reg, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int sc6607_i2c_addr_write_byte(struct sc6607 *chip, u8 i2c_addr, u8 reg, u8 val)
{
	struct i2c_msg xfer[1];
	u8 write_buf[2];
	int ret;

	if (!chip || !chip->client || !chip->client->adapter)
		return -EINVAL;

	write_buf[0] = reg;
	write_buf[1] = val;

	xfer[0].addr = i2c_addr;
	xfer[0].flags = 0;
	xfer[0].len = sizeof(write_buf);
	xfer[0].buf = write_buf;

	ret = i2c_transfer(chip->client->adapter, xfer, 1);
	if (ret == 1) {
		chg_info("i2c transfer successfully, addr=0x%02x, reg=0x%02x, val=0x%02x\n", i2c_addr, reg, val);
		return 0;
	} else if (ret < 0) {
		chg_err("i2c transfer failed, addr=0x%02x, reg=0x%02x, ret=%d\n", i2c_addr, reg, ret);
		return ret;
	} else {
		chg_err("i2c transfer EIO, addr=0x%02x, reg=0x%02x, ret=%d\n", i2c_addr, reg, ret);
		return -EIO;
	}
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
	}
}

static void oplus_chg_get_mmi_state(struct sc6607 *chip, int *mmi_chg)
{
	if (!chip)
		return;

	if (!chip->chg_disable_votable)
		chip->chg_disable_votable = find_votable("CHG_DISABLE");
	*mmi_chg = !get_client_vote(chip->chg_disable_votable, MMI_CHG_VOTER);

	return;
}

static void sc6607_detect_init(struct sc6607 *chip)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
	Charger_Detect_Init();
#else
	sc6607_request_dpdm(chip, true);
#endif
}

static void sc6607_detect_release(struct sc6607 *chip)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
	Charger_Detect_Release();
#else
	sc6607_request_dpdm(chip, false);
#endif
}

static void sc6607_bc12_timeout_func(struct timer_list *timer)
{
	struct sc6607 *chip = container_of(timer, struct sc6607, bc12_timeout);

	chg_info("BC1.2 timeout\n");
	schedule_delayed_work(&chip->hw_bc12_detect_work, msecs_to_jiffies(0));
}

static int sc6607_bc12_timeout_start(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	chg_info("start\n");
	del_timer(&chip->bc12_timeout);
	if (chip->is_sc6607a)
		chip->bc12_timeout.expires = jiffies + msecs_to_jiffies(800);
	else
		chip->bc12_timeout.expires = jiffies + msecs_to_jiffies(500);
	chip->bc12_timeout.function = sc6607_bc12_timeout_func;
	add_timer(&chip->bc12_timeout);
	return 0;
}

static int sc6607_bc12_timeout_cancel(struct sc6607 *chip)
{
	if (!chip)
		return 0;

	chg_info("del bc12_timeout\n");
	del_timer(&chip->bc12_timeout);
	return 0;
}

static int oplus_chg_get_cp_enable(struct sc6607 *chip)
{
	u8 cp_enable = 0;
	int ret = 0;

	if (!chip)
		return 0;

	ret = sc6607_field_read(chip, F_CP_EN, &cp_enable);
	if (ret < 0)
		chg_info("read F_CP_EN fail, ret=%d\n", ret);

	return cp_enable;
}

static int bc12_update_dpdm_state(struct sc6607 *chip)
{
	u8 dp;
	u8 dm;
	int ret = -EINVAL;
	struct soft_bc12 *bc;

	if (!chip)
		return ret;

	bc = &(chip->bc12);
	ret = sc6607_write_byte(chip, SC6607_REG_DPDM_INTERNAL, 0xa0);
	if (ret < 0)
		return ret;

	ret = sc6607_read_byte(chip, SC6607_REG_DP_STAT, &dp);
	switch (dp) {
	case 0x00:
		bc->dp_state = DPDM_V0_TO_V0_325;
		break;
	case 0x01:
		bc->dp_state = DPDM_V0_325_TO_V1;
		break;
	case 0x03:
		bc->dp_state = DPDM_V1_TO_V1_35;
		break;
	case 0x07:
		bc->dp_state = DPDM_V1_35_TO_V22;
		break;
	case 0x0F:
		bc->dp_state = DPDM_V2_2_TO_V3;
		break;
	case 0x1F:
		bc->dp_state = DPDM_V3;
		break;
	default:
		break;
	}

	ret = sc6607_read_byte(chip, SC6607_REG_DM_STAT, &dm);
	switch (dm) {
	case 0x00:
		bc->dm_state = DPDM_V0_TO_V0_325;
		break;
	case 0x01:
		bc->dm_state = DPDM_V0_325_TO_V1;
		break;
	case 0x03:
		bc->dm_state = DPDM_V1_TO_V1_35;
		break;
	case 0x07:
		bc->dm_state = DPDM_V1_35_TO_V22;
		break;
	case 0x0F:
		bc->dm_state = DPDM_V2_2_TO_V3;
		break;
	case 0x1F:
		bc->dm_state = DPDM_V3;
		break;
	default:
		break;
	}

	return 0;
}

static int bc12_set_dp_state(struct sc6607 *chip, enum DPDM_SET_STATE state)
{
	int ret;

	if (!chip)
		return -EINVAL;

	switch (state) {
	case DPDM_DOWN_500K:
		ret = sc6607_field_write(chip, F_DP_DRIV, 0);
		ret |= sc6607_field_write(chip, F_DP_500K_PD_EN, 1);
		break;
	default:
		ret = sc6607_field_write(chip, F_DP_DRIV, state);
		break;
	}

	return ret;
}

static int bc12_set_dm_state(struct sc6607 *chip, enum DPDM_SET_STATE state)
{
	int ret;

	if (!chip)
		return -EINVAL;

	ret = sc6607_write_byte(chip, SC6607_REG_DPDM_INTERNAL + 1, 0x00);

	switch (state) {
	case DPDM_DOWN_500K:
		ret |= sc6607_field_write(chip, F_DM_DRIV, 0);
		ret |= sc6607_field_write(chip, F_DM_500K_PD_EN, 1);
		break;
	case DPDM_V1_8:
		ret |= sc6607_write_byte(chip, SC6607_REG_DPDM_INTERNAL + 2, 0x2a);
		ret |= sc6607_write_byte(chip, SC6607_REG_DPDM_INTERNAL + 1, 0x0a);
		break;
	default:
		ret = sc6607_field_write(chip, F_DM_DRIV, state);
		break;
	}

	return ret;
}

static int bc12_set_dp_cap(struct sc6607 *chip, enum DPDM_CAP cap)
{
	if (!chip)
		return -EINVAL;

	switch (cap) {
	case DPDM_CAP_SNK_50UA:
		sc6607_field_write(chip, F_DP_SINK_EN, 1);
		sc6607_field_write(chip, F_BC1_2_DP_DM_SINK_CAP, 0);
		break;
	case DPDM_CAP_SNK_100UA:
		sc6607_field_write(chip, F_DP_SINK_EN, 1);
		sc6607_field_write(chip, F_BC1_2_DP_DM_SINK_CAP, 1);
		break;
	case DPDM_CAP_SRC_10UA:
		sc6607_field_write(chip, F_DP_SINK_EN, 0);
		sc6607_field_write(chip, F_DP_SRC_10UA, 1);
		break;
	case DPDM_CAP_SRC_250UA:
		sc6607_field_write(chip, F_DP_SINK_EN, 0);
		sc6607_field_write(chip, F_DP_SRC_10UA, 0);
		break;
	default:
		break;
	}

	return 0;
}

static int bc12_set_dm_cap(struct sc6607 *chip, enum DPDM_CAP cap)
{
	if (!chip)
		return -EINVAL;

	switch (cap) {
	case DPDM_CAP_SNK_50UA:
		sc6607_field_write(chip, F_DM_SINK_EN, 1);
		sc6607_field_write(chip, F_BC1_2_DP_DM_SINK_CAP, 0);
		break;
	case DPDM_CAP_SNK_100UA:
		sc6607_field_write(chip, F_DM_SINK_EN, 1);
		sc6607_field_write(chip, F_BC1_2_DP_DM_SINK_CAP, 1);
		break;
	default:
		break;
	}

	return 0;
}

static int bc12_init(struct sc6607 *chip)
{
	int ret;
	u8 data = 0;

	ret = sc6607_field_read(chip, F_AUTO_INDET_EN, &data);
	if (data > 0) {
		sc6607_field_write(chip, F_AUTO_INDET_EN, false);
		msleep(10);
	}
	return 0;
}

static inline void bc12_transfer_state(struct sc6607 *chip, u8 state, int time)
{
	if (!chip)
		return;

	chip->bc12.bc12_state = state;
	chip->bc12.next_run_time = time;
}

static inline void bc_set_result(struct sc6607 *chip, enum BC12_RESULT result)
{
	if (!chip)
		return;

	chip->bc12.result = result | (chip->bc12.flag << 3);
	chip->bc12.detect_done = true;

	switch (result) {
	case UNKNOWN_DETECED:
		chip->soft_bc12_type = SC6607_VBUS_TYPE_DCP;
		break;
	case SDP_DETECED:
		if (chip->bc12.first_noti_sdp)
			chip->soft_bc12_type = SC6607_VBUS_TYPE_NONE;
		else
			chip->soft_bc12_type = SC6607_VBUS_TYPE_SDP;
		break;
	case CDP_DETECED:
		if (chip->bc12.first_noti_sdp)
			chip->soft_bc12_type = SC6607_VBUS_TYPE_NONE;
		else
			chip->soft_bc12_type = SC6607_VBUS_TYPE_CDP;
		break;
	case DCP_DETECED:
	case NON_STANDARD_DETECTED:
	case APPLE_3A_DETECTED:
	case APPLE_2_1A_DETECTED:
	case SS_2A_DETECTED:
	case APPLE_1A_DETECTED:
	case APPLE_2_4A_DETECTED:
	case OCP_DETECED:
		chip->soft_bc12_type = SC6607_VBUS_TYPE_DCP;
		break;
	case HVDCP_DETECED:
		chip->soft_bc12_type = SC6607_VBUS_TYPE_DCP;
		break;
	default:
		chip->soft_bc12_type = SC6607_VBUS_TYPE_DCP;
		break;
	}
}

static int bc12_detect_init(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	chip->bc12.detect_done = false;

	bc12_init(chip);
	bc12_set_dm_state(chip, DPDM_HIZ);
	bc12_set_dp_state(chip, DPDM_HIZ);
	bc12_transfer_state(chip, NON_STANDARD_ADAPTER_DETECTION, 40);

	return 0;
}

static int bc12_nostand_adapter_detect_entry(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	if (chip->bc12.dp_state == DPDM_V2_2_TO_V3 && chip->bc12.dm_state == DPDM_V3)
		chip->bc12.flag = 1;
	else if (chip->bc12.dp_state == DPDM_V2_2_TO_V3 && chip->bc12.dm_state == DPDM_V1_35_TO_V22)
		chip->bc12.flag = 2;
	else if (chip->bc12.dp_state == DPDM_V1_TO_V1_35 && chip->bc12.dm_state == DPDM_V1_TO_V1_35)
		chip->bc12.flag = 3;
	else if (chip->bc12.dp_state == DPDM_V1_35_TO_V22 && chip->bc12.dm_state == DPDM_V2_2_TO_V3)
		chip->bc12.flag = 4;
	else if (chip->bc12.dp_state == DPDM_V2_2_TO_V3 && chip->bc12.dm_state == DPDM_V2_2_TO_V3)
		chip->bc12.flag = 5;
	else
		chip->bc12.flag = 0;

	bc12_set_dp_state(chip, DPDM_V2_7);
	bc12_set_dm_state(chip, DPDM_DOWN_20K);
	bc12_set_dp_cap(chip, DPDM_CAP_SRC_10UA);
	bc12_transfer_state(chip, FLOAT_DETECTION, 15);
	return 0;
}

static int bc12_float_detection_entry(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	if (chip->bc12.dp_state >= DPDM_V1_TO_V1_35 && chip->bc12.flag == 0)
		bc_set_result(chip, OCP_DETECED);
	else {
		bc12_set_dp_state(chip, DPDM_V0_6);
		bc12_set_dp_cap(chip, DPDM_CAP_SRC_250UA);

		bc12_set_dm_state(chip, DPDM_HIZ);
		bc12_set_dm_cap(chip, DPDM_CAP_SNK_50UA);

		bc12_transfer_state(chip, BC12_PRIMARY_DETECTION, 100);
	}

	return 0;
}

static int bc12_primary_detect_entry(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	if (chip->bc12.dm_state == DPDM_V0_TO_V0_325 && chip->bc12.flag == 0) {
		bc_set_result(chip, SDP_DETECED);
	} else if (chip->bc12.dm_state == DPDM_V0_325_TO_V1) {
		bc12_set_dp_state(chip, DPDM_HIZ);
		bc12_set_dm_state(chip, DPDM_HIZ);
		bc12_transfer_state(chip, HIZ_SET, 20);
	} else {
		if (chip->bc12.flag == 0)
			bc_set_result(chip, UNKNOWN_DETECED);
		else
			bc_set_result(chip, NON_STANDARD_DETECTED);
	}
	return 0;
}

static int bc12_hiz_set_entry(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	bc12_set_dp_cap(chip, DPDM_CAP_SNK_50UA);
	bc12_set_dm_state(chip, DPDM_V1_8);
	bc12_transfer_state(chip, BC12_SECONDARY_DETECTION, 40);
	return 0;
}

static int bc12_secondary_detect_entry(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	if (chip->bc12.dp_state < DPDM_V1_35_TO_V22)
		bc_set_result(chip, CDP_DETECED);
	else if (chip->bc12.dp_state == DPDM_V1_35_TO_V22) {
		bc12_set_dm_cap(chip, DPDM_CAP_SNK_100UA);
		bc12_set_dm_state(chip, DPDM_HIZ);

		bc12_set_dp_state(chip, DPDM_V0_6);
		bc12_transfer_state(chip, HVDCP_HANKE, 2000);
		bc_set_result(chip, DCP_DETECED);
	} else {
		if (chip->bc12.flag == 0)
			bc_set_result(chip, UNKNOWN_DETECED);
		else
			bc_set_result(chip, NON_STANDARD_DETECTED);
	}
	return 0;
}

static int bc12_hvdcp_hanke_entry(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	if (chip->bc12.dm_state == DPDM_V0_TO_V0_325)
		bc_set_result(chip, HVDCP_DETECED);
	chip->bc12.next_run_time = -1;
	return 0;
}

static bool bc12_should_run(struct sc6607 *chip)
{
	bool ret = true;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->bc12.running_lock);
	if (chip->power_good) {
		chip->bc12.detect_ing = true;
	} else {
		chip->bc12.detect_ing = false;
		ret = false;
	}
	mutex_unlock(&chip->bc12.running_lock);

	return ret;
}

static void sc6607_soft_bc12_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct soft_bc12 *bc = container_of(dwork, struct soft_bc12, detect_work);
	struct sc6607 *chip = container_of(bc, struct sc6607, bc12);

	if (!bc12_should_run(chip))
		return;

	bc12_update_dpdm_state(chip);
	chg_info("dp volt range %s, dm volt range %s, state : %s\n",
		dpdm_str[bc->dp_state], dpdm_str[bc->dm_state],
		state_str[bc->bc12_state]);

	switch (bc->bc12_state) {
	case BC12_DETECT_INIT:
		bc12_detect_init(chip);
		break;
	case NON_STANDARD_ADAPTER_DETECTION:
		bc12_nostand_adapter_detect_entry(chip);
		break;
	case FLOAT_DETECTION:
		bc12_float_detection_entry(chip);
		break;
	case BC12_PRIMARY_DETECTION:
		bc12_primary_detect_entry(chip);
		break;
	case HIZ_SET:
		bc12_hiz_set_entry(chip);
		break;
	case BC12_SECONDARY_DETECTION:
		bc12_secondary_detect_entry(chip);
		break;
	case HVDCP_HANKE:
		bc12_hvdcp_hanke_entry(chip);
		break;
	default:
		break;
	}

	if (bc->detect_done) {
		bc->detect_ing = false;
		if (bc->first_noti_sdp && (bc->result == SDP_DETECED || bc->result == CDP_DETECED)) {
			bc12_set_dp_state(chip, DPDM_HIZ);
			bc12_set_dm_state(chip, DPDM_HIZ);
			sc6607_field_write(chip, F_DPDM_3P3_EN, true);
			bc12_detect_run(chip);
		} else {
			chg_info("set hiz\n");
			bc12_set_dp_state(chip, DPDM_HIZ);
			bc12_set_dm_state(chip, DPDM_HIZ);
			sc6607_field_write(chip, F_DPDM_3P3_EN, true);
			if (bc->result == SDP_DETECED || bc->result == CDP_DETECED)
				chip->usb_connect_start = true;
			sc6607_dpdm_irq_handle(chip);
			sc6607_check_ic_suspend(chip);
		}
		bc->first_noti_sdp = false;
	} else if (bc->next_run_time >= 0) {
		schedule_delayed_work(&bc->detect_work, msecs_to_jiffies(bc->next_run_time));
	}
}

static int bc12_detect_run(struct sc6607 *chip)
{
	struct soft_bc12 *bc;

	if (!chip)
		return -EINVAL;

	chg_info("start\n");
	bc = &(chip->bc12);
	if (bc->detect_ing) {
		chg_info("detect_ing, should return\n");
		return 0;
	}
	bc->bc12_state = BC12_DETECT_INIT;
	schedule_delayed_work(&bc->detect_work, msecs_to_jiffies(300));

	return 0;
}

static int sc6607_charger_get_soft_bc12_type(struct sc6607 *chip)
{
	if (chip)
		return chip->soft_bc12_type;

	return SC6607_VBUS_TYPE_NONE;
}

static void sc6607_hw_bc12_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc6607 *chip = container_of(dwork, struct sc6607, hw_bc12_detect_work);

	sc6607_detect_init(chip);
	chip->bc12_try_count = 0;
	chip->bc12_done = false;
	if (chip->bc12_timeouts <= OPLUS_BC12_MAX_TRY_COUNT)
		sc6607_bc12_timeout_start(chip);
	chip->bc12_timeouts++;
	sc6607_force_dpdm(chip, true);
}

static int sc6607_check_device_id(struct sc6607 *chip)
{
	int ret;
	u8 chip_id;

	if (!chip)
		return -EINVAL;

	chip->is_sc6607a = false;
	ret = sc6607_read_byte(chip, SC6607_REG_DEVICE_ID, &chip_id);
	if (ret < 0) {
		return ret;
	}

	if (chip_id == SC6607A_CHIP_ID)
		chip->is_sc6607a = true;

	chip->chip_id = chip_id;
	chg_info("chip_id:%d\n", chip->chip_id);

	return 0;
}

static int sc6607_enable_otg(struct sc6607 *chip)
{
	int ret = 0;

	if (!chip)
		return -EINVAL;

	chg_info("enter\n");
	ret = sc6607_field_write(chip, F_BOOST_EN, true);

	return ret;
}

static int sc6607_disable_otg(struct sc6607 *chip)
{
	int ret = 0;

	if (!chip)
		return -EINVAL;

	chg_err("enter\n");
	ret = sc6607_field_write(chip, F_BOOST_EN, false);

	return ret;
}

static int sc6607_get_otg_status(struct sc6607 *chip)
{
	u8 data = 0;

	if (!chip)
		return 0;

	sc6607_field_read(chip, F_BOOST_EN, &data);
	chg_info("status = %d\n", data);

	return data;
}

static int sc6607_disable_hvdcp(struct sc6607 *chip)
{
	int ret;

	if (!chip)
		return -EINVAL;

	ret = sc6607_field_write(chip, F_HVDCP_EN, false);
	return ret;
}

static int sc6607_enable_hvdcp(struct sc6607 *chip)
{
	int ret;

	if (!chip)
		return -EINVAL;

	chg_info("enable hvdcp\n");

	ret = sc6607_field_write(chip, F_HVDCP_EN, true);
	return ret;
}

static int sc6607_enable_charger(struct sc6607 *chip)
{
	int ret;

	if (!chip)
		return -EINVAL;

	chg_info("enable\n");
	if (atomic_read(&chip->driver_suspended) || chip->request_otg) {
		chg_err("suspend or camera, ignore\n");
		return 0;
	}

	ret = sc6607_field_write(chip, F_CHG_EN, true);

	return ret;
}

static int sc6607_disable_charger(struct sc6607 *chip)
{
	int ret;

	if (!chip)
		return -EINVAL;

	chg_info("disable\n");
	ret = sc6607_field_write(chip, F_CHG_EN, false);
	return ret;
}

static int sc6607_hk_get_adc(struct sc6607 *chip, enum SC6607_ADC_MODULE id)
{
	u32 reg = SC6607_REG_HK_IBUS_ADC + id * SC6607_ADC_REG_STEP;
	u8 val[2] = { 0 };
	u64 ret = 0;
	int rc = 0;
	u8 adc_open = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->adc_read_lock);
	sc6607_field_read(chip, F_ADC_EN, &adc_open);
	if (!adc_open) {
		if (id == SC6607_ADC_TSBUS || id == SC6607_ADC_TSBAT) {
			if (chip->platform_data->ntc_suport_1000k) {
				rc = sc6607_field_write(chip, F_ADC_EN, true);
				if (rc < 0) {
					if (!chip->open_adc_by_vac)
						sc6607_field_write(chip, F_ADC_EN, false);
					chg_err("sc6607_field_write fail rc =%d\n", rc);
					mutex_unlock(&chip->adc_read_lock);
					return 0;
				}
				msleep(ADC_DELAY_MS);
				sc6607_field_write(chip, F_ADC_FREEZE, 1);
				rc = sc6607_bulk_read(chip, reg, val, sizeof(val));
				sc6607_field_write(chip, F_ADC_FREEZE, 0);
				if (rc < 0) {
					if (!chip->open_adc_by_vac)
						sc6607_field_write(chip, F_ADC_EN, false);
					mutex_unlock(&chip->adc_read_lock);
					return 0;
				}

				ret = val[1] + (val[0] << 8);
				ret = sc6607_tsbus_tsbat_to_convert(chip, ret, id);
				if (!chip->open_adc_by_vac)
					sc6607_field_write(chip, F_ADC_EN, false);
				mutex_unlock(&chip->adc_read_lock);
				return (int)ret;
			} else {
				mutex_unlock(&chip->adc_read_lock);
				ret = sc6607_tsbus_tsbat_to_convert(chip, SC6607_ADC_TSBAT_DEFAULT, ADC_TSBUS_TSBAT_DEFAULT);
				return (int)ret;
			}
		} else {
			mutex_unlock(&chip->adc_read_lock);
			return 0;
		}
	}
	sc6607_field_write(chip, F_ADC_FREEZE, 1);
	rc = sc6607_bulk_read(chip, reg, val, sizeof(val));
	sc6607_field_write(chip, F_ADC_FREEZE, 0);
	mutex_unlock(&chip->adc_read_lock);
	if (rc < 0)
		return -EINVAL;

	ret = val[1] + (val[0] << 8);
	if (id == SC6607_ADC_TDIE) {
		ret = (SC6607_ADC_IDTE_THD - ret) / 2;
	} else if (id == SC6607_ADC_TSBUS) {
		ret = sc6607_tsbus_tsbat_to_convert(chip, ret, SC6607_ADC_TSBUS);
	} else if (id == SC6607_ADC_TSBAT) {
		ret = sc6607_tsbus_tsbat_to_convert(chip, ret, SC6607_ADC_TSBAT);
	} else {
		ret *= sy6607_adc_step[id];
	}
	return (int)ret;
}

static int sc6607_adc_read_ibus(struct sc6607 *chip)
{
	int ibus = 0;

	if (!chip)
		return -EINVAL;

	if (chip->voocphy && oplus_chg_get_fastchg_commu_ing()) {
		chg_info("svooc in communication\n");
		return chip->voocphy->cp_ichg;
	} else {
		ibus = sc6607_hk_get_adc(chip, SC6607_ADC_IBUS);
		ibus /= SC6607_UA_PER_MA;
		return ibus;
	}
}

static int sc6607_adc_read_vbus_volt(struct sc6607 *chip)
{
	int vbus_vol = 0;

	if (!chip)
		return -EINVAL;

	if (chip->voocphy && oplus_chg_get_fastchg_commu_ing()) {
		chg_info("svooc in communication\n");
		return chip->voocphy->cp_vbus;
	}
	vbus_vol = sc6607_hk_get_adc(chip, SC6607_ADC_VBUS);
	vbus_vol /= SC6607_UV_PER_MV;

	return vbus_vol;
}

static int sc6607_adc_read_tsbus(struct sc6607 *chip)
{
	int tsbus = 0;

	if (!chip)
		return -EINVAL;

	tsbus = sc6607_hk_get_adc(chip, SC6607_ADC_TSBUS);

	return tsbus;
}

static int sc6607_adc_read_tsbat(struct sc6607 *chip)
{
	int tsbat = 0;

	if (!chip)
		return -EINVAL;

	tsbat = sc6607_hk_get_adc(chip, SC6607_ADC_TSBAT);

	return tsbat;
}

static int oplus_sc6607_set_ichg(struct sc6607 *chip, int curr)
{
	int ret;
	int val;

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->driver_suspended)) {
		chg_err("suspend,ignore set cur = %d mA\n", curr);
		return 0;
	}

	if (curr > SC6607_CHG_CURRENT_MAX_MA)
		curr = SC6607_CHG_CURRENT_MAX_MA;

	val = (curr - SC6607_BUCK_ICHG_OFFSET) / SC6607_BUCK_ICHG_STEP;
	ret = sc6607_field_write(chip, F_ICHG_CC, val);
	chg_info("current = %d, val=0x%0x\n", curr, val);

	return ret;
}

static int sc6607_set_term_current(struct sc6607 *chip, int curr)
{
	u8 iterm;
	int ret;

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->driver_suspended)) {
		chg_err("suspend,ignore set cur = %d mA\n", curr);
		return 0;
	}

	iterm = (curr - SC6607_BUCK_ITERM_OFFSET) / SC6607_BUCK_ITERM_STEP;
	ret = sc6607_field_write(chip, F_ITERM, iterm);
	chg_info("iterm = %d, val=0x%0x\n", curr, iterm);

	return ret;
}

static int sc6607_set_prechg_current(struct sc6607 *chip, int curr)
{
	u8 iprechg;
	int ret;

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->driver_suspended)) {
		chg_err("suspend,ignore set cur = %d mA\n", curr);
		return 0;
	}

	iprechg = (curr - SC6607_BUCK_IPRECHG_OFFSET) / SC6607_BUCK_IPRECHG_STEP;
	ret = sc6607_field_write(chip, F_IPRECHG, iprechg);
	chg_info("iprechg = %d, val=0x%0x\n", curr, iprechg);

	return ret;
}

static int sc6607_set_chargevolt(struct sc6607 *chip, int volt)
{
	u8 val = 0;
	int ret;

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->driver_suspended)) {
		chg_err("suspend,ignore set volt = %d mv\n", volt);
		return 0;
	}

	val = (volt - SC6607_BUCK_VBAT_OFFSET) / SC6607_BUCK_VBAT_STEP;
	ret = sc6607_field_write(chip, F_VBAT, val);
	chg_info("volt = %d, val=0x%0x\n", volt, val);

	return ret;
}

static int sc6607_set_input_volt_limit(struct sc6607 *chip, int volt)
{
	u8 val = 0;
	int ret;

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->driver_suspended)) {
		chg_err("suspend,ignore set volt_limit = %d mv\n", volt);
		return 0;
	}

	if (volt <= SC6607_VINDPM_VOL_MV(4000))
		val = SC6607_VINDPM_4000;
	else if (volt <= SC6607_VINDPM_VOL_MV(4100))
		val = SC6607_VINDPM_4100;
	else if (volt <= SC6607_VINDPM_VOL_MV(4200))
		val = SC6607_VINDPM_4200;
	else if (volt <= SC6607_VINDPM_VOL_MV(4300))
		val = SC6607_VINDPM_4300;
	else if (volt <= SC6607_VINDPM_VOL_MV(4400))
		val = SC6607_VINDPM_4400;
	else if (volt <= SC6607_VINDPM_VOL_MV(4500))
		val = SC6607_VINDPM_4500;
	else if (volt <= SC6607_VINDPM_VOL_MV(4600))
		val = SC6607_VINDPM_4600;
	else if (volt <= SC6607_VINDPM_VOL_MV(4700))
		val = SC6607_VINDPM_4700;
	else if (volt <= SC6607_VINDPM_VOL_MV(5000))
		val = SC6607_VINDPM_4800;
	else if (volt <= SC6607_VINDPM_VOL_MV(7600))
		val = SC6607_VINDPM_7600;
	else if (volt <= SC6607_VINDPM_VOL_MV(8200))
		val = SC6607_VINDPM_8200;
	else if (volt <= SC6607_VINDPM_VOL_MV(8400))
		val = SC6607_VINDPM_8400;
	else if (volt <= SC6607_VINDPM_VOL_MV(8600))
		val = SC6607_VINDPM_8600;
	else if (volt <= SC6607_VINDPM_VOL_MV(10000))
		val = SC6607_VINDPM_10000;
	else if (volt <= SC6607_VINDPM_VOL_MV(10500))
		val = SC6607_VINDPM_10500;
	else
		val = SC6607_VINDPM_10700;

	ret = sc6607_field_write(chip, F_VINDPM, val);
	chg_info("volt = %d, val=0x%0x\n", volt, val);

	return ret;
}

static int sc6607_set_input_current_limit(struct sc6607 *chip, int curr)
{
	int val;
	int ret;

	if (!chip)
		return -EINVAL;

	if (curr < SC6607_BUCK_IINDPM_OFFSET)
		curr = SC6607_BUCK_IINDPM_OFFSET;

	val = (curr - SC6607_BUCK_IINDPM_OFFSET) / SC6607_BUCK_IINDPM_STEP;
	ret = sc6607_field_write(chip, F_IINDPM, val);
	chg_info("curr:%d, val=0x%x\n", curr, val);

	return ret;
}

static int sc6607_set_watchdog_timer(struct sc6607 *chip, u32 timeout)
{
	u8 val = 0;
	int ret;

	if (!chip)
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

	ret = sc6607_field_write(chip, F_WD_TIMER, val);
	chg_info("timeout:%d, val=0x%x\n", timeout, val);

	return ret;
}

static int sc6607_disable_watchdog_timer(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	return sc6607_set_watchdog_timer(chip, SC6607_WD_TIMEOUT_S(0));
}

static int sc6607_reset_watchdog_timer(struct sc6607 *chip)
{
	int ret;

	if (!chip)
		return -EINVAL;

	chg_info("enter\n");
	ret = sc6607_field_write(chip, F_WD_TIME_RST, true);

	return ret;
}

static int sc6607_force_dpdm(struct sc6607 *chip, bool enable)
{
	int ret;

	if (!chip)
		return -EINVAL;

	ret = sc6607_field_write(chip, F_FORCE_INDET, enable);
	chg_info("force dpdm %s, enable=%d\n", !ret ? "successfully" : "failed", enable);
	return ret;
}

static int sc6607_reset_chip(struct sc6607 *chip)
{
	int ret;
	int rst_en;

	if (!chip || !chip->platform_data)
		return -EINVAL;

	rst_en = chip->platform_data->batfet_rst_en;
	ret = sc6607_field_write(chip, F_REG_RST, true);
	if (!rst_en)
		ret = sc6607_field_write(chip, F_BATFET_RST_EN, false);
	else
		ret = sc6607_field_write(chip, F_BATFET_RST_EN, true);
	chg_info("reset chip %s\n", !ret ? "successfully" : "failed");
	return ret;
}

static int sc6607_enable_enlim(struct sc6607 *chip)
{
	int ret;

	if (!chip)
		return -EINVAL;

	ret = sc6607_field_write(chip, F_IINDPM_DIS, false);
	chg_info("enable ilim %s\n", !ret ? "successfully" : "failed");
	return ret;
}

static int sc6607_enter_hiz_mode(struct sc6607 *chip)
{
	int ret;
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int boot_mode = get_boot_mode();
#endif

	if (!chip)
		return -EINVAL;

	chg_info("enter\n");
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (boot_mode == META_BOOT || boot_mode == FACTORY_BOOT ||
	    boot_mode == ADVMETA_BOOT || boot_mode == ATE_FACTORY_BOOT)
		ret = sc6607_field_write(chip, F_HIZ_EN, false);
	else
		ret = sc6607_field_write(chip, F_HIZ_EN, true);

#else
	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN ||
	    boot_mode == MSM_BOOT_MODE__FACTORY)
		ret = sc6607_field_write(chip, F_HIZ_EN, false);
	else
		ret = sc6607_field_write(chip, F_HIZ_EN, true);
#endif
#endif
	ret |= sc6607_disable_charger(chip);
	oplus_sc6607_charger_suspend(chip);

	return ret;
}

int sc6607_set_boost_current(struct sc6607 *chip, int curr)
{
	int val;
	int ret;

	if (!chip)
		return -EINVAL;

	if (curr <= SC6607_BOOST_CURR_MA(500))
		val = SC6607_BOOST_CURR_500;
	else if (curr <= SC6607_BOOST_CURR_MA(900))
		val = SC6607_BOOST_CURR_900;
	else if (curr <= SC6607_BOOST_CURR_MA(1300))
		val = SC6607_BOOST_CURR_1300;
	else if (curr <= SC6607_BOOST_CURR_MA(1500))
		val = SC6607_BOOST_CURR_1500;
	else if (curr <= SC6607_BOOST_CURR_MA(2100))
		val = SC6607_BOOST_CURR_2100;
	else if (curr <= SC6607_BOOST_CURR_MA(2500))
		val = SC6607_BOOST_CURR_2500;
	else if (curr <= SC6607_BOOST_CURR_MA(2900))
		val = SC6607_BOOST_CURR_2900;
	else
		val = SC6607_BOOST_CURR_32500;

	ret = sc6607_field_write(chip, F_IBOOST, val);
	chg_info("boost current %d mA, val=0x%x\n", curr, val);

	return ret;
}

static int sc6607_vmin_limit(struct sc6607 *chip, u32 volt)
{
	int val;
	int ret;

	if (!chip)
		return -EINVAL;

	if (volt < SC6607_VSYSMIN_VOL_MV(2800))
		val = SC6607_VSYSMIN_2600;
	else if (volt < SC6607_VSYSMIN_VOL_MV(3000))
		val = SC6607_VSYSMIN_2800;
	else if (volt < SC6607_VSYSMIN_VOL_MV(3200))
		val = SC6607_VSYSMIN_3000;
	else if (volt < SC6607_VSYSMIN_VOL_MV(3400))
		val = SC6607_VSYSMIN_3200;
	else if (volt < SC6607_VSYSMIN_VOL_MV(3500))
		val = SC6607_VSYSMIN_3400;
	else if (volt < SC6607_VSYSMIN_VOL_MV(3600))
		val = SC6607_VSYSMIN_3500;
	else if (volt < SC6607_VSYSMIN_VOL_MV(3700))
		val = SC6607_VSYSMIN_3600;
	else
		val = SC6607_VSYSMIN_3700;

	ret = sc6607_write_byte(chip, SC6607_REG_VSYS_MIN, val);
	chg_info("vsys_min %d mv, val=0x%x\n", volt, val);

	return ret;
}

static int sc6607_enable_auto_dpdm(struct sc6607 *chip, bool enable)
{
	int ret;

	if (!chip)
		return -EINVAL;

	ret = sc6607_field_write(chip, F_AUTO_INDET_EN, enable);
	chg_info("%s\n", enable ? "enable" : "disable");

	return ret;
}

static int sc6607_set_boost_voltage(struct sc6607 *chip, int volt)
{
	int ret;
	u8 val;

	if (!chip)
		return -EINVAL;

	if (volt < SC6607_BUCK_VBOOST_OFFSET)
		volt = SC6607_BUCK_VBOOST_OFFSET;

	val = (volt - SC6607_BUCK_VBOOST_OFFSET) / SC6607_BUCK_VBOOST_STEP;
	ret = sc6607_field_write(chip, F_VBOOST, val);
	chg_info("volt:%d, val=0x%x\n", volt, val);

	return ret;
}

static int sc6607_enable_ico(struct sc6607 *chip, bool enable)
{
	int ret;

	if (!chip)
		return -EINVAL;

	ret = sc6607_field_write(chip, F_ICO_EN, enable);
	chg_info("enable:%d\n", enable);

	return ret;
}

static struct sc6607_platform_data *sc6607_parse_dt(struct device_node *np, struct sc6607 *chip)
{
	int ret;
	struct sc6607_platform_data *pdata;

	pdata = devm_kzalloc(chip->dev, sizeof(struct sc6607_platform_data), GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (of_property_read_string(np, "charger_name", &chip->chg_dev_name) < 0) {
		chip->chg_dev_name = "primary_chg";
		pr_warn("no charger name\n");
	}

	ret = of_property_read_u32(np, "sc,vsys-limit", &pdata->vsyslim);
	if (ret) {
		pdata->vsyslim = 3500;
		chg_err("failed to read node of sc,vsys-limit\n");
	}

	ret = of_property_read_u32(np, "sc,batsnc-enable", &pdata->batsns_en);
	if (ret) {
		pdata->batsns_en = 0;
		chg_err("failed to read node of sc,batsnc-enable\n");
	}

	ret = of_property_read_u32(np, "sc,vbat", &pdata->vbat);
	if (ret) {
		pdata->vbat = 4450;
		chg_err("failed to read node of sc,vbat\n");
	}

	ret = of_property_read_u32(np, "sc,charge-curr", &pdata->ichg);
	if (ret) {
		pdata->ichg = 2000;
		chg_err("failed to read node of sc,charge-curr\n");
	}

	ret = of_property_read_u32(np, "sc,iindpm-disable", &pdata->iindpm_dis);
	if (ret) {
		pdata->iindpm_dis = 0;
		chg_err("failed to read node of sc,iindpm-disable\n");
	}

	ret = of_property_read_u32(np, "sc,input-curr-limit", &pdata->iindpm);
	if (ret) {
		pdata->iindpm = 500;
		chg_err("failed to read node of sc,input-curr-limit\n");
	}

	ret = of_property_read_u32(np, "sc,ico-enable", &pdata->ico_enable);
	if (ret) {
		pdata->ico_enable = 0;
		chg_err("failed to read node of sc,ico-enable\n");
	}

	ret = of_property_read_u32(np, "sc,iindpm-ico", &pdata->iindpm_ico);
	if (ret) {
		pdata->iindpm_ico = 100;
		chg_err("failed to read node of sc,iindpm-ico\n");
	}

	ret = of_property_read_u32(np, "sc,precharge-volt", &pdata->vprechg);
	if (ret) {
		pdata->vprechg = 0;
		chg_err("failed to read node of sc,precharge-volt\n");
	}

	ret = of_property_read_u32(np, "sc,precharge-volt", &pdata->vprechg);
	if (ret) {
		pdata->vprechg = 0;
		chg_err("failed to read node of sc,precharge-volt\n");
	}

	ret = of_property_read_u32(np, "sc,precharge-curr", &pdata->iprechg);
	if (ret) {
		pdata->iprechg = 500;
		chg_err("failed to read node of sc,precharge-curr\n");
	}

	ret = of_property_read_u32(np, "sc,term-en", &pdata->iterm_en);
	if (ret) {
		pdata->iterm_en = 0;
		chg_err("failed to read node of sc,term-en\n");
	}

	ret = of_property_read_u32(np, "sc,term-curr", &pdata->iterm);
	if (ret) {
		pdata->iterm = 0;
		chg_err("failed to read node of sc,sc,term-curr\n");
	}

	ret = of_property_read_u32(np, "sc,rechg-dis", &pdata->rechg_dis);
	if (ret) {
		pdata->rechg_dis = 0;
		chg_err("failed to read node of sc,rechg-dis\n");
	}

	ret = of_property_read_u32(np, "sc,rechg-dg", &pdata->rechg_dg);
	if (ret) {
		pdata->rechg_dg = 0;
		chg_err("failed to read node of sc,rechg-dg\n");
	}

	ret = of_property_read_u32(np, "sc,rechg-volt", &pdata->rechg_volt);
	if (ret) {
		pdata->rechg_volt = 0;
		chg_err("failed to read node of sc,rechg-volt\n");
	}

	ret = of_property_read_u32(np, "sc,boost-voltage", &pdata->vboost);
	if (ret) {
		pdata->vboost = 5000;
		chg_err("failed to read node of sc,boost-voltage\n");
	}

	ret = of_property_read_u32(np, "sc,conv-ocp-dis", &pdata->conv_ocp_dis);
	if (ret) {
		pdata->conv_ocp_dis = 0;
		chg_err("failed to read node of sc,conv-ocp-dis\n");
	}

	ret = of_property_read_u32(np, "sc,tsbat-jeita-dis", &pdata->tsbat_jeita_dis);
	if (ret) {
		pdata->tsbat_jeita_dis = 1;
		chg_err("failed to read node of sc,tsbat-jeita-dis\n");
	}

	ret = of_property_read_u32(np, "sc,ibat-ocp-dis", &pdata->ibat_ocp_dis);
	if (ret) {
		pdata->ibat_ocp_dis = 0;
		chg_err("failed to read node of sc,ibat-ocp-dis\n");
	}

	ret = of_property_read_u32(np, "sc,vpmid-ovp-otg-dis", &pdata->vpmid_ovp_otg_dis);
	if (ret) {
		pdata->vpmid_ovp_otg_dis = 0;
		chg_err("failed to read node of sc,vpmid-ovp-otg-dis\n");
	}

	ret = of_property_read_u32(np, "sc,vbat-ovp-buck-dis", &pdata->vbat_ovp_buck_dis);
	if (ret) {
		pdata->vbat_ovp_buck_dis = 0;
		chg_err("failed to read node of sc,vbat-ovp-buck-dis\n");
	}

	ret = of_property_read_u32(np, "sc,ibat-ocp", &pdata->ibat_ocp);
	if (ret) {
		pdata->ibat_ocp = 1;
		chg_err("failed to read node of sc,ibat-ocp\n");
	}

	chip->disable_qc = of_property_read_bool(np, "sc,disable-qc");
	chg_info("disable_qc:%d\n", chip->disable_qc);

	chip->sc6607_switch_ntc = of_property_read_bool(np, "oplus,sc6607_switch_ntc");
	chg_info("sc6607_switch_ntc:%d\n", chip->sc6607_switch_ntc);

	chip->not_support_usb_btb = of_property_read_bool(np, "oplus,not_support_usb_btb");
	chg_info("not_support_usb_btb:%d\n", chip->not_support_usb_btb);

	ret = read_signed_data_from_node(np, "oplus,sc6607_ntc_surport_1000k",
					(s32 *)pst_temp_table_1000k, TEMP_TABLE_100K_SIZE2);
	if (ret == 0)
		pdata->ntc_suport_1000k = true;
	else
		chg_err("not surport 1000k ntc, rc = %d\n", ret);

/********* workaround: Octavian needs to enable adc start *********/
	pdata->enable_adc = of_property_read_bool(np, "sc,enable-adc");
	chg_info("sc,enable-adc = %d\n", pdata->enable_adc);
/********* workaround: Octavian needs to enable adc end *********/

	ret = of_property_read_u32(np, "sc,cc_pull_up_idrive", &pdata->cc_pull_up_idrive);
	if (ret) {
		chg_err("failed to read node of sc,cc_pull_up_idrive set default\n");
		pdata->cc_pull_up_idrive = 0;
	}

	ret = of_property_read_u32(np, "sc,cc_pull_down_idrive", &pdata->cc_pull_down_idrive);
	if (ret) {
		chg_err("failed to read node of sc,cc_pull_down_idrive set default\n");
		pdata->cc_pull_down_idrive = 0;
	}

	ret = of_property_read_u32(np, "sc,continuous_time", &pdata->continuous_time);
	if (ret) {
		chg_err("failed to read node of sc,continuous_time set default\n");
		pdata->continuous_time = 0;
	}

	ret = of_property_read_u32_array(np, "sc,bmc_width", pdata->bmc_width,
					 ARRAY_SIZE(pdata->bmc_width));
	if (ret)
		chg_err("failed to read node of sc,bmc_width set default\n");

	ret = of_property_read_u32(np, "sc,batfet_rst_en", &pdata->batfet_rst_en);
	if (ret) {
		chg_err("failed to read node of sc,batfet_rst_en set default\n");
		pdata->batfet_rst_en = 0;
	}

	chip->disable_tcpc_irq = of_property_read_bool(np, "oplus,disable_tcpc_irq");
	chg_info("disable_tcpc_irq:%d", chip->disable_tcpc_irq);

	chip->use_vooc_phy = of_property_read_bool(np, "oplus,use_vooc_phy");
	chip->use_ufcs_phy = of_property_read_bool(np, "oplus,use_ufcs_phy");
	chg_info("use_vooc_phy=%d use_ufcs_phy=%d\n", chip->use_vooc_phy, chip->use_ufcs_phy);

	chip->batt_btb_temp_chan = devm_iio_channel_get(chip->dev, "bat_btb_therm");
	if (IS_ERR(chip->batt_btb_temp_chan)) {
		chg_info("couldn't get batt_btb_temp_chan\n");
		chip->batt_btb_temp_chan = NULL;
	}

	chip->usb_btb_temp_chan = devm_iio_channel_get(chip->dev, "usb_btb_therm");
	if (IS_ERR(chip->usb_btb_temp_chan)) {
		chg_info("Couldn't get usb_btb_temp_chan\n");
		chip->usb_btb_temp_chan = NULL;
	}
	chip->usb_aicl_enhance = of_property_read_bool(np, "oplus,usb_aicl_enhance");
	chg_info("usb_aicl_enhance:%d", chip->usb_aicl_enhance);

	return pdata;
}

static bool is_wired_icl_votable_available(struct sc6607 *chip)
{
	if (!chip)
		return false;

	if (!chip->wired_icl_votable)
		chip->wired_icl_votable = find_votable("WIRED_ICL");
	return !!chip->wired_icl_votable;
}

static bool is_wired_fcc_votable_available(struct sc6607 *chip)
{
	if (!chip)
		return false;

	if (!chip->wired_fcc_votable)
		chip->wired_fcc_votable = find_votable("WIRED_FCC");
	return !!chip->wired_fcc_votable;
}

static void sc6607_rerun_votable_work(struct work_struct *work)
{
	struct sc6607 *chip =
		container_of(work, struct sc6607, rerun_votable_work);

	if (is_wired_fcc_votable_available(chip))
		rerun_election(chip->wired_fcc_votable, false);
	if (is_wired_icl_votable_available(chip))
		rerun_election(chip->wired_icl_votable, true);
}

static bool sc6607_check_rerun_detect_chg_type(struct sc6607 *chip, u8 type)
{
	bool need_rerun_bc12 = false;

	if (!chip)
		return false;

	if (chip->bc12_try_count == OPLUS_BC12_MAX_TRY_COUNT)
		chip->bc12.first_noti_sdp = false;

	/* If port type is pd_usb, do not need rerun bc12. Will cause input current overwrite */
	need_rerun_bc12 = (type == SC6607_VBUS_TYPE_SDP || type == SC6607_VBUS_TYPE_CDP) &&
			    !oplus_pd_sdp_port();
	if (chip->bc12.first_noti_sdp && need_rerun_bc12) {
		sc6607_detect_init(chip);
		sc6607_disable_hvdcp(chip);
		sc6607_force_dpdm(chip, true);
		sc6607_bc12_timeout_start(chip);
		chg_info("hw rerun bc12\n");
		return true;
	}

	if (chip->bc12_done == false) {
		chg_info("bc12_done\n");
		chip->bc12_done = true;
		sc6607_set_input_current_limit(chip, SC6607_DEFAULT_IBUS_MA);
		schedule_work(&chip->rerun_votable_work);
	}

	return false;
}

static int sc6607_get_charger_type(struct sc6607 *chip, unsigned int *type)
{
	int ret;
	u8 vbus_stat;
	u8 hw_bc12_done = 0;
	unsigned int oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (!chip)
		return -EINVAL;

	if (!chip->soft_bc12) {
		ret = sc6607_field_read(chip, F_BC1_2_DONE, &hw_bc12_done);
		if (ret)
			return ret;

		if (hw_bc12_done) {
			ret = sc6607_field_read(chip, F_VBUS_STAT, &vbus_stat);
			if (ret)
				return ret;
			chip->vbus_type = vbus_stat;
			chip->bc12_try_count++;
			chg_info("bc12_try_count[%d] reg_type:0x%x\n", chip->bc12_try_count, vbus_stat);
		} else {
			chg_err("hw_bc12_done not complete\n");
			return ret;
		}

		if (sc6607_check_rerun_detect_chg_type(chip, vbus_stat))
			return ret;

		chip->usb_connect_start = true;
	} else {
		chip->vbus_type = sc6607_charger_get_soft_bc12_type(chip);
		chg_info("soft_bc12_type:0x%x\n", chip->vbus_type);
	}

	switch (chip->vbus_type) {
	case SC6607_VBUS_TYPE_OTG:
	case SC6607_VBUS_TYPE_NONE:
		oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case SC6607_VBUS_TYPE_SDP:
		if (oplus_pd_dcp_port()) {
			chg_info("force sdp to dcp\n");
			oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		} else {
			oplus_chg_type = POWER_SUPPLY_TYPE_USB;
		}
		break;
	case SC6607_VBUS_TYPE_CDP:
		if (oplus_pd_dcp_port()) {
			chg_info("force cdp to dcp\n");
			oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		} else {
			oplus_chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		}
		break;
	case SC6607_VBUS_TYPE_DCP:
		oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case SC6607_VBUS_TYPE_HVDCP:
		oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		if (!chip->disable_qc) {
			chip->hvdcp_can_enabled = true;
			chg_err("hvdcp_can_enabled is true\n");
		}
		break;
	case SC6607_VBUS_TYPE_UNKNOWN:
		oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case SC6607_VBUS_TYPE_NON_STD:
		oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		oplus_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	}

	*type = oplus_chg_type;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_BC12_COMPLETED);

	return 0;
}

static int sc6607_inform_charger_type(struct sc6607 *chip)
{
	int ret = 0;
#ifdef CONFIG_OPLUS_CHARGER_MTK
	union power_supply_propval propval;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 0))
	if (!chip->psy) {
		chip->psy = power_supply_get_by_name("mtk-master-charger");
		if (IS_ERR_OR_NULL(chip->psy)) {
			chg_err("Couldn't get chip->psy");
			return -ENODEV;
		}
	}
#else
	if (!chip->psy) {
		chip->psy = power_supply_get_by_name("charger");
		if (!chip->psy) {
			chg_err("Couldn't get chip->psy");
			return -ENODEV;
		}
	}
#endif
	propval.intval = chip->power_good;
	ret = power_supply_set_property(chip->psy, POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0)
		chg_err("inform power supply online failed:%d\n", ret);

	propval.intval = chip->oplus_chg_type;
	ret = power_supply_set_property(chip->psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);

	if (ret < 0)
		chg_err("inform power supply charge type failed:%d\n", ret);

	power_supply_changed(chip->psy);
	power_supply_changed(chip->chg_psy);
#endif
	return ret;
}

#ifndef CONFIG_OPLUS_CHARGER_MTK
static int sc6607_request_dpdm(struct sc6607 *chip, bool enable)
{
	int ret = 0;

	if (!chip)
		return -EINVAL;

	if (!chip->dpdm_reg && of_get_property(chip->dev->of_node, "dpdm-supply", NULL)) {
		chip->dpdm_reg = devm_regulator_get(chip->dev, "dpdm");
		if (IS_ERR(chip->dpdm_reg)) {
			ret = PTR_ERR(chip->dpdm_reg);
			chg_err("fail get dpdm regulator ret=%d\n", ret);
			chip->dpdm_reg = NULL;
			return ret;
		}
	}

	mutex_lock(&chip->dpdm_lock);
	if (enable) {
		if (chip->dpdm_reg && !chip->dpdm_enabled) {
			chg_err("enabling dpdm regulator\n");
			ret = regulator_enable(chip->dpdm_reg);
			if (ret < 0)
				chg_err("success enable dpdm regulator ret=%d\n", ret);
			else
				chip->dpdm_enabled = true;
		}
	} else {
		if (chip->dpdm_reg && chip->dpdm_enabled) {
			chg_err("disabling dpdm regulator\n");
			ret = regulator_disable(chip->dpdm_reg);
			if (ret < 0)
				chg_err("fail disable dpdm regulator ret=%d\n", ret);
			else
				chip->dpdm_enabled = false;
		}
	}
	mutex_unlock(&chip->dpdm_lock);

	chg_info("dpdm regulator: enable= %d, ret=%d\n", enable, ret);
	return ret;
}
#endif

static void oplus_chg_awake_init(struct sc6607 *chip)
{
	if (!chip)
		return;

	chip->suspend_ws = wakeup_source_register(NULL, "split chg wakelock");
}

static void oplus_chg_wakelock(struct sc6607 *chip, bool awake)
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
}

static void oplus_keep_resume_awake_init(struct sc6607 *chip)
{
	if (!chip) {
		chg_err("chip is null\n");
		return;
	}

	chip->keep_resume_ws = wakeup_source_register(NULL, "split_chg_keep_resume");
}

static void oplus_keep_resume_wakelock(struct sc6607 *chip, bool awake)
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
}

static void oplus_sc6607_set_mivr_by_battery_vol(struct sc6607 *chip)
{
	u32 mV = 0;
	int vbatt = 0;

	oplus_chg_get_batt_volt(&vbatt);
	if (vbatt > SC6607_VINDPM_VBAT_PHASE1)
		mV = vbatt + SC6607_VINDPM_THRES_PHASE1;
	else if (vbatt > SC6607_VINDPM_VBAT_PHASE2)
		mV = vbatt + SC6607_VINDPM_THRES_PHASE2;
	else
		mV = vbatt + SC6607_VINDPM_THRES_PHASE3;

	if (mV < SC6607_VINDPM_THRES_MIN)
		mV = SC6607_VINDPM_THRES_MIN;

	sc6607_set_input_volt_limit(chip, mV);
	chg_info("mV = %d\n", mV);
}

#define SC6607_DPDM_CTRL_REG_NUM	3
int sc6607a_set_dpdm_ctrl(struct sc6607 *chip, bool enable)
{
	int ret = 0;
	int i;
	u8 addr_buf[SC6607_DPDM_CTRL_REG_NUM] = {
				SC6607_REG_DPDM_CTRL,
				SC6607_REG_DPDM_CTRL_2,
				SC6607_REG_DPDM_NONSTD_STAT };
	u8 cmd_buf[SC6607_DPDM_CTRL_REG_NUM] = {
				enable ? 0xB4 : 0x00,
				enable ? 0x39 : 0x00,
				enable ? 0x08 : 0x00 };

	if (!chip)
		return -EINVAL;

	for (i = 0; i < SC6607_DPDM_CTRL_REG_NUM; i++) {
		ret = sc6607_write_byte(chip, addr_buf[i], cmd_buf[i]);
		if (ret < 0) {
			chg_err("write dpdm ctrl reg failed\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int sc6607_hk_irq_handle(struct sc6607 *chip)
{
	int ret;
	u8 val[2];
	bool prev_pg = false;
	bool vac_present = false;
	bool vbus_present = false;
	u8 val_bk[2];

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->driver_suspended))
		chg_info("suspended and wait %d ms\n", SC6607_WAIT_RESUME_TIME);

	if (chip->voocphy && oplus_chg_get_fastchg_commu_ing()) {
		ret = sc6607_voocphy_read_byte(chip->client, SC6607_REG_HK_INT_STAT, &val[0]);
		if (ret) {
			chg_err("read hk int stat reg failed\n");
			return -EINVAL;
		}

		ret = sc6607_voocphy_read_byte(chip->client, SC6607_REG_CHG_INT_STAT, &val[1]);
		if (ret) {
			chg_err("read chg int stat reg failed\n");
			return -EINVAL;
		}
	} else {
		ret = sc6607_read_byte(chip, SC6607_REG_HK_INT_STAT, &val[0]);
		if (ret) {
			chg_err("read hk int stat reg failed\n");
			return -EINVAL;
		}

		ret = sc6607_read_byte(chip, SC6607_REG_CHG_INT_STAT, &val[1]);
		if (ret) {
			chg_err("read chg int stat reg failed\n");
			return -EINVAL;
		}
	}

	prev_pg = chip->power_good;
	vac_present = !!(val[0] & SC6607_HK_VAC_PRESENT_MASK);
	vbus_present = !!(val[0] & SC6607_HK_VBUS_PRESENT_MASK);
	chip->power_good = (vac_present) && (vbus_present);
	chg_info("prev_pg:%d, now_pg:%d, val[0]:0x%x, val[1]:0x%x, camera_on:%d\n",
			prev_pg, chip->power_good, val[0], val[1], chip->camera_on);

	if (chip->camera_on) {
		chg_info("camera_on\n");
		if(prev_pg && !chip->power_good) {
			chip->hvdcp_can_enabled = false;
			chip->qc_to_9v_count = 0;
		}
		goto out;
	}

	if (vac_present && !chip->open_adc_by_vac) {
		sc6607_field_write(chip, F_ADC_EN, 1);
		chip->open_adc_by_vac = true;
	}

	if (chip->power_good)
		oplus_chg_wakelock(chip, true);

	if (chip->power_good != prev_pg)
		oplus_sc6607_set_mivr_by_battery_vol(chip);

	if ((!prev_pg && chip->power_good) || chip->wd_rerun_detect) {
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PLUGIN);
		if (!chip->camera_on)
			chip->request_otg = 0;
		chip->wd_rerun_detect = false;
		oplus_chg_wakelock(chip, true);
		sc6607_bulk_read(chip, SC6607_REG_HK_ADC_CTRL, val_bk, sizeof(val_bk));
		val_bk[0] &=~SC6607_HK_CTRL3;
		val_bk[1] &=~SC6607_ADC_FUNC_DIS;
		sc6607_bulk_write(chip, SC6607_REG_HK_ADC_CTRL, val_bk, sizeof(val_bk));

		sc6607_field_write(chip, F_IBATOCP, chip->platform_data->ibat_ocp);
		sc6607_field_write(chip, F_CONV_OCP_DIS, chip->platform_data->conv_ocp_dis);
		sc6607_field_write(chip, F_RECHG_DIS, chip->platform_data->rechg_dis);
		sc6607_field_write(chip, F_CHG_TIMER, 0x03);
		sc6607_field_write(chip, F_ACDRV_MANUAL_PRE, 3);
		sc6607_field_write(chip, F_TSBUS_TSBAT_FLT_DIS, false);
		sc6607_field_write(chip, F_TSBAT_JEITA_DIS, false);
		sc6607_field_write(chip, F_VBUS_PD, 0);
		sc6607_enable_enlim(chip);
		if (atomic_read(&chip->charger_suspended))
			oplus_sc6607_charger_suspend(chip);
		sc6607_inform_charger_type(chip);

		if (chip->pr_swap) {
			chip->pr_swap = false;
			chg_info("sdp port\n");
			return 0;
		}
#ifdef CONFIG_OPLUS_CHARGER_MTK
		if (oplus_is_rf_ftm_mode()) {
			chip->oplus_chg_type = POWER_SUPPLY_TYPE_USB;
			sc6607_inform_charger_type(chip);
			chg_err("Meta mode force usb type\n");
		}
#endif
		if (chip->is_force_dpdm) {
			sc6607_detect_init(chip);
			chip->is_force_dpdm = false;
			sc6607_force_dpdm(chip, false);
		} else {
			if (chip->oplus_chg_type == POWER_SUPPLY_TYPE_UNKNOWN) {
				sc6607_detect_init(chip);
				sc6607_disable_hvdcp(chip);
				chip->bc12.first_noti_sdp = true;
				chip->bc12_done = false;
				oplus_sc6607_set_ichg(chip, SC6607_BUCK_ICHG_500MA);
				chip->bc12_timeouts = 0;
				chip->bc12_try_count = 0;
				if (chip->soft_bc12)
					bc12_detect_run(chip);
				else {
					if (chip->hw_bc12_detect_work.work.func)
						schedule_delayed_work(&chip->hw_bc12_detect_work, msecs_to_jiffies(140));
				}
			}
		}
	} else if (prev_pg && !chip->power_good) {
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PLUGIN);
		if (chip->is_sc6607a)
			sc6607a_set_dpdm_ctrl(chip, false);
		sc6607_bulk_read(chip, SC6607_REG_HK_ADC_CTRL, val_bk, sizeof(val_bk));
		val_bk[0] |=SC6607_HK_CTRL3;
		val_bk[1] |=SC6607_ADC_FUNC_DIS;
		sc6607_bulk_write(chip, SC6607_REG_HK_ADC_CTRL, val_bk, sizeof(val_bk));
/********* workaround: Octavian needs to enable adc start *********/
		if ((chip->platform_data->enable_adc == true) && (oplus_is_rf_ftm_mode() == false))
			chg_info("do not disable adc\n");
		else
			sc6607_field_write(chip, F_ADC_EN, 0);
/********* workaround: Octavian needs to enable adc end *********/
		chip->open_adc_by_vac = false;
		if (chip->soft_bc12) {
			bc12_set_dp_state(chip, DPDM_HIZ);
			bc12_set_dm_state(chip, DPDM_HIZ);
		}

		mutex_lock(&chip->bc12.running_lock);
		chip->bc12.detect_ing = false;
		mutex_unlock(&chip->bc12.running_lock);
		chip->usb_connect_start = false;
		chip->is_force_dpdm = false;
		chip->soft_bc12_type = SC6607_VBUS_TYPE_NONE;
		chip->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		chip->hvdcp_can_enabled = false;
		chip->qc_to_9v_count = 0;

#ifdef CONFIG_OPLUS_CHARGER_MTK
		oplus_chg_pullup_dp_set(false);
#endif
		sc6607_enable_enlim(chip);
		sc6607_field_write(chip, F_ACDRV_MANUAL_PRE, 3);
		sc6607_disable_hvdcp(chip);
		if (!chip->disable_qc)
			oplus_notify_hvdcp_detach_stat(chip);
		sc6607_detect_release(chip);
		sc6607_bc12_timeout_cancel(chip);
		if (chip->soft_bc12)
			cancel_delayed_work_sync(&chip->bc12.detect_work);
		else
			cancel_delayed_work_sync(&chip->hw_bc12_detect_work);
		chg_info("removed\n");
		oplus_chg_wakelock(chip, false);
	}
out:
	return 0;
}

static int sc6607_dpdm_irq_handle(struct sc6607 *chip)
{
	int ret;
	u8 hvdcp_en = false;
	static int prev_vbus_type = SC6607_VBUS_TYPE_NONE;
	unsigned int prev_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	unsigned int cur_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (!chip)
		return -EINVAL;

	sc6607_field_read(chip, F_HVDCP_EN, &hvdcp_en);
	if (!hvdcp_en)
		sc6607_set_input_current_limit(chip, SC6607_DEFAULT_IBUS_MA);
	sc6607_bc12_timeout_cancel(chip);
	prev_chg_type = chip->oplus_chg_type;
	ret = sc6607_get_charger_type(chip, &cur_chg_type);

	if (!chip->soft_bc12)
		chg_info("bc12_try_count [%d] : prev_vbus_type[%d] --> cur_vbus_type[%d]\n",
				chip->bc12_try_count, prev_vbus_type, chip->vbus_type);
	if (!chip->bc12_done) {
		prev_vbus_type = chip->vbus_type;
		return 0;
	}

	prev_vbus_type = SC6607_VBUS_TYPE_NONE;
	chg_info("prev_chg_type[%d] --> cur_chg_type[%d]\n", prev_chg_type, cur_chg_type);

	if ((prev_chg_type == POWER_SUPPLY_TYPE_USB_DCP) &&
	    ((cur_chg_type == POWER_SUPPLY_TYPE_USB) || (cur_chg_type == POWER_SUPPLY_TYPE_USB_CDP))) {
		chg_info("keep prev chg type\n");
		cur_chg_type = prev_chg_type;
	}

	if ((cur_chg_type == POWER_SUPPLY_TYPE_USB) || (cur_chg_type == POWER_SUPPLY_TYPE_USB_CDP)) {
		chg_info("usb_connect_start:%d\n", chip->usb_connect_start);
		chip->oplus_chg_type = cur_chg_type;
		if (!chip->is_sc6607a) {
			bc12_set_dm_state(chip, DPDM_HIZ);
			bc12_set_dp_state(chip, DPDM_HIZ);
		}
		if (chip->usb_connect_start)
			sc6607_inform_charger_type(chip);
	} else if (chip->oplus_chg_type == POWER_SUPPLY_TYPE_UNKNOWN && cur_chg_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		chg_info("cur_chg_type = %d, vbus_type = %d\n", cur_chg_type, chip->vbus_type);
		chip->oplus_chg_type = cur_chg_type;
		sc6607_inform_charger_type(chip);
	} else {
		chg_info("oplus_chg_type = %d, vbus_type = %d", chip->oplus_chg_type, chip->vbus_type);
	}

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (cur_chg_type == POWER_SUPPLY_TYPE_USB_CDP) {
		sc6607_detect_release(chip);
		oplus_chg_pullup_dp_set(true);
		return 0;
	} else {
		oplus_chg_pullup_dp_set(false);
	}
	sc6607_detect_release(chip);
#endif
	return 0;
}

__maybe_unused static int sc6607_mask_hk_irq(struct sc6607 *chip, int irq_channel)
{
	int ret = 0;
	u8 val = 0;

	ret = sc6607_read_byte(chip, SC6607_REG_HK_INT_MASK, &val);
	if (ret < 0)
		return ret;

	val |= irq_channel;
	return sc6607_write_byte(chip, SC6607_REG_HK_INT_MASK, val);
}

__maybe_unused static int sc6607_unmask_hk_irq(struct sc6607 *chip, int irq_channel)
{
	int ret;
	u8 val = 0;

	ret = sc6607_read_byte(chip, SC6607_REG_HK_INT_MASK, &val);
	if (ret < 0)
		return ret;

	val &= ~irq_channel;
	return sc6607_write_byte(chip, SC6607_REG_HK_INT_MASK, val);
}

__maybe_unused static int sc6607_mask_buck_irq(struct sc6607 *chip, int irq_channel)
{
	int ret;
	u8 val[3] = { 0 };

	ret = sc6607_bulk_read(chip, SC6607_REG_CHG_INT_MASK, val, 3);
	if (ret < 0)
		return ret;

	val[0] |= irq_channel;
	val[1] |= irq_channel >> 8;
	val[2] |= irq_channel >> 16;

	return sc6607_bulk_write(chip, SC6607_REG_CHG_INT_MASK, val, 3);
}

__maybe_unused static int sc6607_unmask_buck_irq(struct sc6607 *chip, int irq_channel)
{
	int ret;
	u8 val[3] = { 0 };

	ret = sc6607_bulk_read(chip, SC6607_REG_CHG_INT_MASK, val, 3);
	if (ret < 0)
		return ret;

	val[0] &= ~(irq_channel);
	val[1] &= ~(irq_channel >> 8);
	val[2] &= ~(irq_channel >> 16);

	return sc6607_bulk_write(chip, SC6607_REG_CHG_INT_MASK, val, 3);
}

static int sc6607_vooc_irq_handle(struct sc6607 *chip)
{
	if (chip->voocphy && chip->use_vooc_phy)
		return oplus_voocphy_interrupt_handler(chip->voocphy);

	return IRQ_HANDLED;
}

static int sc6607_ufcs_irq_handle(struct sc6607 *chip)
{
	struct ufcs_dev *ufcs;

	if (chip && chip->use_ufcs_phy) {
		ufcs = ufcs_get_ufcs_device();
		if (ufcs && ufcs->ops && ufcs->ops->irq_event_handler)
			ufcs->ops->irq_event_handler(ufcs);
	}
	return IRQ_HANDLED;
}

static int sc6607_buck_irq_handle(struct sc6607 *chip)
{
	return 0;
}

static int sc6607_check_wd_timeout_fault(struct sc6607 *chip)
{
	int ret;
	u8 val;

	if (!chip)
		return -EINVAL;

	ret = sc6607_voocphy_read_byte(chip->client, SC6607_REG_HK_FLT_FLG, &val);
	if (ret < 0) {
		chg_err("read reg 0x%x failed\n", SC6607_REG_HK_FLT_FLG);
		return ret;
	}

	if (val & SC6607_HK_WD_TIMEOUT_MASK) {
		chg_err("wd timeout happened\n");
		chip->wd_rerun_detect = true;
		sc6607_init_device(chip);
		sc6607_hk_irq_handle(chip);
	}
	sc6607_track_match_hk_err(chip, val);

	return 0;
}

static int sc6607_cp_irq_handle(struct sc6607 *chip)
{
	u8 val;
	irqreturn_t ret = IRQ_HANDLED;

	ret = sc6607_read_byte(chip, SC6607_REG_CP_FLT_FLG, &val); /*ibus ucp register*/
	chg_info("SC6607_REG_CP_FLT_FLG(0x6B) data:0x%x", val);
	if (ret < 0) {
		chg_err("SC6607_REG_CP_FLT_FLG failed ret=%d\n", ret);
		return IRQ_HANDLED;
	}

	ret = sc6607_read_byte(chip, SC6607_REG_CP_PMID2OUT_FLG, &val);
	chg_info("SC6607_REG_CP_PMID2OUT_FLG(0x6C) data:0x%x", val);
	if (ret < 0) {
		chg_err("SC6607_REG_CP_PMID2OUT_FLG failed ret=%d\n", ret);
		return IRQ_HANDLED;
	}
	return ret;
}

static int sc6607_led_irq_handle(struct sc6607 *chip)
{
	return 0;
}

static const struct sc6607_alert_handler alert_handler[] = {
	DECL_ALERT_HANDLER(UFCS_FLAG, sc6607_ufcs_irq_handle),
	DECL_ALERT_HANDLER(VOOC_FLAG, sc6607_vooc_irq_handle),
	DECL_ALERT_HANDLER(HK_FLAG, sc6607_hk_irq_handle),
	DECL_ALERT_HANDLER(BUCK_CHARGER_FLAG, sc6607_buck_irq_handle),
	DECL_ALERT_HANDLER(CHARGER_PUMP_FLAG, sc6607_cp_irq_handle),
	DECL_ALERT_HANDLER(DPDM_FLAG, sc6607_dpdm_irq_handle),
	DECL_ALERT_HANDLER(LED_FLAG, sc6607_led_irq_handle),
};

static irqreturn_t sc6607_irq_handler(int irq, void *data)
{
	int ret;
	u8 val;
	int i;
	struct sc6607 *chip = (struct sc6607 *)data;

	if (!chip)
		return IRQ_HANDLED;

	oplus_keep_resume_wakelock(chip, true);

	ret = sc6607_voocphy_read_byte(chip->client, SC6607_REG_HK_GEN_FLG, &val);
	if (ret < 0) {
		chg_err("read reg 0x%x failed\n", SC6607_REG_HK_GEN_FLG);
		goto irq_out;
	}

	chg_info("hk_gen_flg:0x%x\n", val);
	val |= BIT(HK_FLAG);

	for (i = 0; i < ARRAY_SIZE(alert_handler); i++) {
		if ((alert_handler[i].bit_mask & val) && alert_handler[i].handler != NULL) {
			alert_handler[i].handler(chip);
		}
	}

	sc6607_check_wd_timeout_fault(chip);

irq_out:
	oplus_keep_resume_wakelock(chip, false);
	return IRQ_HANDLED;
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
static int sc6607_plug_in(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sc6607 *chip = dev_get_drvdata(&chg_dev->dev);

	if (!chip)
		return -EINVAL;

	ret = sc6607_enable_charger(chip);
	if (ret) {
		chg_err("failed to enable charging:%d", ret);
	}
	return ret;
}

static int sc6607_plug_out(struct charger_device *chg_dev)
{
	int ret = 0;
	struct sc6607 *chip = dev_get_drvdata(&chg_dev->dev);

	if (!chip)
		return -EINVAL;

	ret = sc6607_disable_charger(chip);

	if (ret)
		chg_err("failed to disable charging:%d", ret);

	return ret;
}

static int sc6607_charge_kick_wdt(struct charger_device *chg_dev)
{
	struct sc6607 *chip = dev_get_drvdata(&chg_dev->dev);

	if (!chip)
		return -EINVAL;

	return sc6607_reset_watchdog_timer(chip);
}

static int sc6607_charge_enable(struct charger_device *chg_dev, bool en)
{
	struct sc6607 *chip = dev_get_drvdata(&chg_dev->dev);

	if (!chip)
		return -EINVAL;

	if (en)
		return sc6607_enable_charger(chip);
	else
		return sc6607_disable_charger(chip);
}
#endif

static int sc6607_init_default(struct sc6607 *chip)
{
	u8 val[3] = { 0 };
	uint8_t value = 0;
	int ret;
	u8 val_bk[2];

	if (!chip)
		return -EINVAL;

	ret = sc6607_field_write(chip, F_VAC_OVP, 0x00);
	ret = sc6607_field_write(chip, F_VBUS_OVP, 0x02);
	ret |= sc6607_field_write(chip, F_CHG_TIMER, 0x03);
	ret |= sc6607_field_write(chip, F_ACDRV_MANUAL_PRE, 3);

	ret |= sc6607_field_write(chip, F_ACDRV_EN, 1);
	ret |= sc6607_field_write(chip, F_ACDRV_MANUAL_EN, 1);
	val[0] = 0;
	val[1] = 0x04;
	ret |= sc6607_bulk_write(chip, SC6607_REG_HK_ADC_CTRL, val, 2);

	ret |= sc6607_enable_ico(chip, chip->platform_data->ico_enable);
	ret |= sc6607_field_write(chip, F_EDL_TSBUS_SEL, true);
	ret |= sc6607_field_write(chip, F_RECHG_DIS, chip->platform_data->rechg_dis);
	ret |= sc6607_field_write(chip, F_TERM_EN, chip->platform_data->iterm_en);
	ret |= sc6607_field_write(chip, F_CONV_OCP_DIS, chip->platform_data->conv_ocp_dis);
	ret |= sc6607_field_write(chip, F_TSBUS_TSBAT_FLT_DIS, false);
	ret |= sc6607_field_write(chip, F_TSBAT_JEITA_DIS, false);
	ret |= sc6607_field_write(chip, F_VPMID_OVP_OTG_DIS, chip->platform_data->vpmid_ovp_otg_dis);
	ret |= sc6607_field_write(chip, F_VBAT_OVP_BUCK_DIS, chip->platform_data->vbat_ovp_buck_dis);
	ret |= sc6607_field_write(chip, F_IBATOCP, chip->platform_data->ibat_ocp);

	/* 0xb bit3 mask1, mask adc int*/
	ret = sc6607_read_byte(chip, SC6607_REG_HK_INT_MASK, &value);
	chg_info("SC6607_REG_HK_INT_MASK = 0x%x\n", value);
	value |= 0x8;
	sc6607_write_byte(chip, SC6607_REG_HK_INT_MASK, value);
	/* 0x47 bit4 mask1, , mask adc int*/
	ret = sc6607_read_byte(chip, SC6607_REG_CHG_INT_MASK, &value);
	value |= 0x10;
	sc6607_write_byte(chip, SC6607_REG_CHG_INT_MASK, value);
	/* 0x68 bti1 bit2 bit4 mask1, , mask adc int*/
	ret = sc6607_read_byte(chip, SC6607_REG_CP_INT_MASK, &value);
	value |= 0x16;
	sc6607_write_byte(chip, SC6607_REG_CP_INT_MASK, value);

	/*close others adc,keep tsbus and tsbat only*/
	ret = sc6607_bulk_read(chip, SC6607_REG_HK_ADC_CTRL, val_bk, sizeof(val_bk));
	val_bk[0] |=SC6607_HK_CTRL3;
	val_bk[1] |=SC6607_ADC_FUNC_DIS;
	sc6607_bulk_write(chip, SC6607_REG_HK_ADC_CTRL, val_bk, sizeof(val_bk));

/********* workaround: Octavian needs to enable adc start *********/
	if ((chip->platform_data->enable_adc == true) && (oplus_is_rf_ftm_mode() == false))
		ret |= sc6607_field_write(chip, F_ADC_EN, 1);
/********* workaround: Octavian needs to enable adc end *********/

	if (ret < 0)
		chg_info("fail\n");

	return ret;
}

static bool sc6607_is_enter_test_mode(struct sc6607 *chip, bool en)
{
	uint8_t val;
	int ret;

	if (!chip->is_sc6607a) {
		ret = sc6607_read_byte(chip, SC6607_REG_CHECK_TEST_MODE, &val);
		return ((ret < 0 && !en) || (ret >= 0 && val == 0 && en));
	} else {
		ret = sc6607_read_byte(chip, SC6607_REG_ENTER_TEST_MODE, &val);
		return (ret >= 0 && val == (en ? 0x01 : 0x00));
	}
}

static int sc6607_enter_test_mode(struct sc6607 *chip, bool en)
{
	char str[] = "ENTERPRISE";
	int ret, i;

	if (!chip)
		return -EINVAL;

	if (chip->is_sc6607a) {
		strncpy(str, "DISCOVERYP", sizeof(str) - 1);
	}

	chg_info("enter\n");
	do {
		if (sc6607_is_enter_test_mode(chip, en))
			break;
		for (i = 0; i < (ARRAY_SIZE(str) - 1); i++) {
			ret = sc6607_write_byte(chip, SC6607_REG_ENTER_TEST_MODE, str[i]);
				if (ret < 0)
					return ret;
				if (i == (ARRAY_SIZE(str) - 2)) {
					chg_err("enter\n");
					break;
				}
		}
	} while (true);

	return 0;
}

static void sc6607_set_cc_pull_up_idrive(struct sc6607 *chip)
{
	int ret = 0;

	if (chip->platform_data->cc_pull_up_idrive == 0)
		return;

	ret = sc6607_write_byte(chip, SC6607_REG_CC_PULL_UP_IDRIVE,
				chip->platform_data->cc_pull_up_idrive);
	if (ret)
		chg_err("failed to set cc pull up idrive\n");
}

static void sc6607_set_cc_pull_down_idrive(struct sc6607 *chip)
{
	struct i2c_msg xfer[1];
	uint8_t value[] = {SC6607_REG_CC_PULL_DOWN_IDRIVE, chip->platform_data->cc_pull_down_idrive};
	int ret = 0;

	xfer[0].addr = LED_SLAVE_ADDRESS;
	xfer[0].flags = 0;
	xfer[0].len = sizeof(value);
	xfer[0].buf = value;

	if (chip->platform_data->cc_pull_down_idrive == 0)
		return;

	ret = i2c_transfer(chip->client->adapter, xfer,  ARRAY_SIZE(xfer));
	if (ret == 1)
		chg_info("i2c transfer successfully\n");
	else if (ret < 0)
		chg_info("i2c transfer failed\n");
	else
		chg_info("i2c transfer EIO\n");
}

static void sc6607_set_adc_sampling_mode_config(struct sc6607 *chip)
{
	u8 val = 0;
	int ret;

	if (!chip || !chip->client || !chip->client->adapter)
		return;

	/* read current value of reg 0xAE from I2C addr 0x64 (LED_SLAVE_ADDRESS) */
	ret = sc6607_i2c_addr_read_byte(chip, LED_SLAVE_ADDRESS, SC6607_REG_ADC_SAMPLING_MODE_CFG_REG, &val);
	if (ret < 0)
		return;

	chg_info("read reg 0x%x value:0x%x\n", SC6607_REG_ADC_SAMPLING_MODE_CFG_REG, val);
	/* clear bit2 then write back */
	val &= ~SC6607_LED_ADC_SAMPLING_MODE_CONFIG_MASK;

	ret = sc6607_i2c_addr_write_byte(chip, LED_SLAVE_ADDRESS, SC6607_REG_ADC_SAMPLING_MODE_CFG_REG, val);
	if (ret < 0)
		return;
}

static void sc6607_set_continuous_time(struct sc6607 *chip)
{
	int ret = 0;
	struct i2c_msg xfer[1];
	uint8_t value[] = {SC6607_REG_CONTINUOUS_TIME, chip->platform_data->continuous_time};

	xfer[0].addr = PD_PHY_SLAVE_ADDRESS;
	xfer[0].flags = 0;
	xfer[0].len = sizeof(value);
	xfer[0].buf = value;

	if (chip->platform_data->continuous_time == 0)
		return;

	ret = i2c_transfer(chip->client->adapter, xfer,  ARRAY_SIZE(xfer));
	if (ret == 1)
		chg_info("i2c transfer successfully\n");
	else if (ret < 0)
		chg_info("i2c transfer failed\n");
	else
		chg_info("i2c transfer EIO\n");
}

static void sc6607_set_bmc_width(struct sc6607 *chip)
{
	int ret = 0;
	struct i2c_msg xfer[1];
	uint8_t value[] = {SC6607_REG_BMC_WIDTH_1,
			   chip->platform_data->bmc_width[0],
			   chip->platform_data->bmc_width[1],
			   chip->platform_data->bmc_width[2],
			   chip->platform_data->bmc_width[3]};

	xfer[0].addr = PD_PHY_SLAVE_ADDRESS;
	xfer[0].flags = 0;
	xfer[0].len = sizeof(value);
	xfer[0].buf = value;

	if (chip->platform_data->bmc_width[0] == 0)
		return;
	ret = i2c_transfer(chip->client->adapter, xfer,  ARRAY_SIZE(xfer));
	if (ret == 1)
		chg_info("i2c transfer successfully\n");
	else if (ret < 0)
		chg_info("i2c transfer failed\n");
	else
		chg_info("i2c transfer EIO\n");
}

static int sc6607_set_pd_phy_tx_discard_time(struct sc6607 *chip)
{
	int ret = 0;
	u8 value[3]= {SC6607_REG_PD_TX_DISCARD, 0x00, 0x64};
	struct i2c_msg xfer[1];

	xfer[0].addr = chip->client->addr + 1,
	xfer[0].flags = 0;
	xfer[0].len = sizeof(value);
	xfer[0].buf = value;

	ret = i2c_transfer(chip->client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret == ARRAY_SIZE(xfer)) {
		return 0;
	} else {
		chg_err("err %d\n", ret);
		return ret;
	}
}

static int sc6607_reset_pd_phy(struct sc6607 *chip)
{
	int ret = 0;
	u8 value[2]= {SC6607_REG_PD_SOFT_RESET, 0x01};
	struct i2c_msg xfer[1];

	xfer[0].addr = chip->client->addr + 1,
	xfer[0].flags = 0;
	xfer[0].len = sizeof(value);
	xfer[0].buf = value;

	ret = i2c_transfer(chip->client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret == ARRAY_SIZE(xfer)) {
		return 0;
	} else {
		chg_err("err %d\n", ret);
		return ret;
	}
}

static int sc6607_init_device(struct sc6607 *chip)
{
	int ret = 0;
	u8 val = 0;

	chip->is_force_dpdm = false;

	if (chip->chip_id != SC6607_1P1_CHIP_ID && chip->chip_id != SC6607A_CHIP_ID)
		chip->soft_bc12 = true;
	else
		chip->soft_bc12 = false;

	chip->bc12.first_noti_sdp = true;
	chip->bc12_done = false;
	chip->bc12_timeouts = 0;
	chip->bc12_try_count = 0;

	sc6607_disable_watchdog_timer(chip);
	ret = sc6607_read_byte(chip, SC6607_REG_HK_FLT_FLG, &val);
	if (ret)
		chg_err("clear SC6607_REG_HK_FLT_FLG failed, ret = %d\n", ret);

	if (!chip->is_sc6607a) {
		sc6607_set_cc_pull_up_idrive(chip);
		sc6607_set_cc_pull_down_idrive(chip);
		sc6607_enter_test_mode(chip, true);
		sc6607_set_pd_phy_tx_discard_time(chip);
		sc6607_set_continuous_time(chip);
		sc6607_set_bmc_width(chip);
		sc6607_enter_test_mode(chip, false);
	} else {
		sc6607_enter_test_mode(chip, true);
		sc6607_set_pd_phy_tx_discard_time(chip);
		sc6607_set_continuous_time(chip);
		sc6607_enter_test_mode(chip, false);
		sc6607_set_adc_sampling_mode_config(chip);
	}

	ret = sc6607_set_prechg_current(chip, chip->platform_data->iprechg);
	if (ret)
		chg_err("failed to set prechg current, ret = %d\n", ret);

	ret = sc6607_set_chargevolt(chip, chip->platform_data->vbat);
	if (ret)
		chg_err("failed to set default cv, ret = %d\n", ret);

	ret = sc6607_set_term_current(chip, chip->platform_data->iterm);
	if (ret)
		chg_err("failed to set termination current, ret = %d\n", ret);

	ret = sc6607_set_boost_voltage(chip, chip->platform_data->vboost);
	if (ret)
		chg_err("failed to set boost voltage, ret = %d\n", ret);

	ret = sc6607_enable_enlim(chip);
	if (ret)
		chg_err("failed to set enlim, ret = %d\n", ret);

	ret = sc6607_enable_auto_dpdm(chip, false);
	if (ret)
		chg_err("failed to set auto dpdm, ret = %d\n", ret);

	ret = sc6607_vmin_limit(chip, chip->platform_data->vsyslim);
	if (ret)
		chg_err("failed to set vmin limit, ret = %d\n", ret);

	ret = sc6607_set_input_volt_limit(chip, SC6607_HW_AICL_POINT_VOL_5V_PHASE1);
	if (ret)
		chg_err("failed to set input volt limit, ret = %d\n", ret);

	ret = sc6607_set_input_current_limit(chip, chip->platform_data->iindpm);
	if (ret)
		chg_err("failed to set input current limit, ret = %d\n", ret);

	ret = oplus_sc6607_set_ichg(chip, chip->platform_data->ichg);
	if (ret)
		chg_err("failed to set input current limit, ret = %d\n", ret);

	ret |= sc6607_mask_hk_irq(chip, SC6607_HK_RESET_MASK | SC6607_HK_ADC_DONE_MASK | SC6607_HK_REGN_OK_MASK);

	ret |= sc6607_unmask_hk_irq(chip, SC6607_HK_VAC_PRESENT_MASK);

	ret |= sc6607_mask_buck_irq(chip, SC6607_BUCK_ICO_MASK | SC6607_BUCK_IINDPM_MASK | SC6607_BUCK_VINDPM_MASK |
				  SC6607_BUCK_CHG_MASK | SC6607_BUCK_QB_ON_MASK | SC6607_BUCK_VSYSMIN_MASK);
	ret = sc6607_read_byte(chip, SC6607_REG_HK_GEN_FLG, &val);
	if (ret < 0)
		chg_err("failed to read SC6607_REG_HK_GEN_FLG, ret = %d\n", ret);

	ret = sc6607_read_byte(chip, SC6607_REG_DPDM_INT_FLAG, &val);
	if (ret < 0)
		 chg_err("failed to read SC6607_REG_DPDM_INT_FLAG, ret = %d\n", ret);

	ret |= sc6607_init_default(chip);

	return ret;
}

static ssize_t sc6607_charger_show_registers(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc6607 *chip;
	int addr;
	u8 val[128];
	int ret;
	u8 tmpbuf[SC6607_PAGE_SIZE];
	int idx = 0;
	int index;
	int len;

	chip = dev_get_drvdata(dev);
	if (!chip)
		return idx;

	ret = sc6607_read_data(chip, SC6607_REG_DEVICE_ID, val, (SC6607_REG_HK_TSBAT_ADC - SC6607_REG_DEVICE_ID));
	if (ret < 0)
		return idx;
	index = 0;
	for (addr = SC6607_REG_DEVICE_ID; addr < SC6607_REG_HK_TSBAT_ADC; addr++) {
		len = snprintf(tmpbuf, SC6607_PAGE_SIZE - idx, "[%.2X]=0x%.2x\n", addr, val[index]);
		memcpy(&buf[idx], tmpbuf, len);
		idx += len;
		index++;
		if (index >= sizeof(val))
			break;
	}

	ret = sc6607_read_data(chip, SC6607_REG_VSYS_MIN, val, (SC6607_REG_BUCK_MASK - SC6607_REG_VSYS_MIN));
	if (ret < 0)
		return idx;
	index = 0;
	for (addr = SC6607_REG_VSYS_MIN; addr < SC6607_REG_BUCK_MASK; addr++) {
		len = snprintf(tmpbuf, SC6607_PAGE_SIZE - idx, "[%.2X]=0x%.2x\n", addr, val[index]);
		memcpy(&buf[idx], tmpbuf, len);
		idx += len;
		index++;
		if (index >= sizeof(val))
			break;
	}

	ret = sc6607_read_data(chip, SC6607_REG_VBATSNS_OVP, val, (SC6607_REG_CP_FLT_DIS - SC6607_REG_VBATSNS_OVP));
	if (ret < 0)
		return idx;
	index = 0;
	for (addr = SC6607_REG_VBATSNS_OVP; addr < SC6607_REG_CP_FLT_DIS; addr++) {
		len = snprintf(tmpbuf, SC6607_PAGE_SIZE - idx, "[%.2X]=0x%.2x\n", addr, val[index]);
		memcpy(&buf[idx], tmpbuf, len);
		idx += len;
		index++;
		if (index >= sizeof(val))
			break;
	}

	ret = sc6607_read_data(chip, SC6607_REG_DPDM_EN, val, (SC6607_REG_DPDM_NONSTD_STAT - SC6607_REG_DPDM_EN));
	if (ret < 0)
		return idx;
	index = 0;
	for (addr = SC6607_REG_DPDM_EN; addr < SC6607_REG_DPDM_NONSTD_STAT; addr++) {
		len = snprintf(tmpbuf, SC6607_PAGE_SIZE - idx, "[%.2X]=0x%.2x\n", addr, val[index]);
		memcpy(&buf[idx], tmpbuf, len);
		idx += len;
		index++;
		if (index >= sizeof(val))
			break;
	}
	ret = sc6607_read_data(chip, SC6607_REG_PHY_CTRL, val, (SC6607_REG_DP_HOLD_TIME - SC6607_REG_PHY_CTRL));
	if (ret < 0)
		return idx;
	index = 0;
	for (addr = SC6607_REG_PHY_CTRL; addr < SC6607_REG_DP_HOLD_TIME; addr++) {
		len = snprintf(tmpbuf, SC6607_PAGE_SIZE - idx, "[%.2X]=0x%.2x\n", addr, val[index]);
		memcpy(&buf[idx], tmpbuf, len);
		idx += len;
		index++;
		if (index >= sizeof(val))
			break;
	}
	chg_info("SC6607_REG_PHY_CTRL  idx=%d, buf=%s \n", idx, buf);

	return idx;
}

static ssize_t sc6607_charger_store_register(struct device *dev, struct device_attribute *attr, const char *buf,
					     size_t count)
{
	struct sc6607 *chip;
	int ret;
	unsigned int reg;
	unsigned int val;

	chip = dev_get_drvdata(dev);
	if (!chip)
		return count;

	ret = sscanf(buf, "%x %x", &reg, &val);
	chg_info("reg[0x%2x], val[0x%2x]\n", reg, val);
	if (ret == 2 && reg < SC6607_REG_MAX)
		sc6607_write_byte(chip, (u8)reg, (u8)val);

	if (reg == 0x00 && val == 1)
		chip->disable_wdt = 1;
	else
		chip->disable_wdt = 0;
	return count;
}

static DEVICE_ATTR(charger_registers, 0660, sc6607_charger_show_registers, sc6607_charger_store_register);

static void sc6607_charger_create_device_node(struct device *dev)
{
	int ret = 0;

	if (!dev) {
		chg_err("device is null, cannot create device node.");
		return;
	}

	ret = device_create_file(dev, &dev_attr_charger_registers);

	if (ret != 0)
		chg_err("create device node failed, ret = %d.", ret);
}

static int sc6607_enter_ship_mode(struct sc6607 *chip, bool en)
{
	int ret = 0;

	if (!chip)
		return -EINVAL;

	chg_info("enter\n");
	sc6607_reset_pd_phy(chip);

	ret = sc6607_field_write(chip, F_REG_RST, true);
	if (ret < 0)
		chg_err("write F_REG_RST err\n");

	if (en) {
		ret = sc6607_field_write(chip, F_BATFET_DLY, true);
		ret |= sc6607_field_write(chip, F_BATFET_DIS, true);
	} else {
		ret |= sc6607_field_write(chip, F_BATFET_DIS, false);
	}

	return ret;
}

static int oplus_chg_usb_set_input_current(struct sc6607 *chip, int current_ma,
	int aicl_point)
{
	int rc = 0, i = 0;
	int chg_vol = 0;

	bool pre_step = false;

	for (i = 1; i <= current_ma / 100; i++) {
		rc = sc6607_set_input_current_limit(chip, i * 100);
		if (rc) {
			chg_err("set icl to %d uA fail, rc=%d\n", i * 100, rc);
			return rc;
		} else {
			chg_err("set icl to %d mA\n", i * 100);
		}
		msleep(90);
		chg_vol = sc6607_adc_read_vbus_volt(chip);
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
		rc = sc6607_set_input_current_limit(chip, i  * 100);
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

static int oplus_sc6607_set_aicr(struct sc6607 *chip, int current_ma)
{
	int rc = 0, i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	int aicl_point_temp = 0;
	int main_cur = 0;
	int slave_cur = 0;
	int batt_volt = 0;
	int chg_type = 0;
	int charger_type;
	bool present = false;
	int max_pdo_current;

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->driver_suspended))
		return 0;

	if (atomic_read(&chip->charger_suspended)) {
		chg_err("suspend,ignore set current=%dmA\n", current_ma);
		return 0;
	}
	oplus_chg_get_batt_volt(&batt_volt);
	chg_vol = sc6607_adc_read_vbus_volt(chip);
	chg_type = oplus_wired_get_chg_type();
	if ((chg_vol > SC6607_9V_THRES1_MV) &&
	    (chg_type == CHARGER_SUBTYPE_PD || chg_type == OPLUS_CHG_USB_TYPE_QC2)) {
		if (oplus_gauge_get_batt_num() == 1)
			aicl_point = SC6607_AICL_POINT_VOL_9V;
		else
			aicl_point = SC6607_DUAL_AICL_POINT_VOL_9V;
	} else {
		if (batt_volt > SC6607_AICL_POINT_VOL_5V_LOW)
			aicl_point = SC6607_SW_AICL_POINT_VOL_5V_PHASE2;
		else
			aicl_point = SC6607_SW_AICL_POINT_VOL_5V_PHASE1;
	}

	aicl_point_temp = aicl_point;
	chg_info("usb input max current limit=%d, aicl_point_temp=%d \n", current_ma,
			aicl_point_temp);

	if (oplus_chg_get_common_charge_icl_support_flags()) {
		max_pdo_current = oplus_get_max_current_from_fixed_pdo(chip, chip->pd_chg_volt);
		chg_info("max_pdo_current:%d ma\n", max_pdo_current);

		if (max_pdo_current >= 0)
			current_ma = min(current_ma, max_pdo_current);
		if (current_ma < DEFAULT_CURR_BY_CC) {
			cancel_delayed_work_sync(&chip->charger_suspend_recovery_work);
			oplus_chg_suspend_charger(true, PD_PDO_ICL_VOTER);
			schedule_delayed_work(&chip->charger_suspend_recovery_work,
			                      msecs_to_jiffies(SUSPEND_RECOVERY_DELAY_MS));
			goto aicl_rerun;
		} else {
			oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
		}
	}

	if (chip->usb_aicl_enhance && chip->ic_dev) {
		sc6607_input_present(chip->ic_dev, &present);
		rc = sc6607_get_bc12_result(chip->ic_dev, &charger_type);
		if (rc >= 0  && (charger_type == OPLUS_CHG_USB_TYPE_SDP ||
		    charger_type == OPLUS_CHG_USB_TYPE_CDP ||
		    (charger_type == OPLUS_CHG_USB_TYPE_UNKNOWN && current_ma == UNKONW_CURR)) &&
		    present) {
			if (charger_type == OPLUS_CHG_USB_TYPE_SDP) {
				aicl_point = USB_SW_AICL_POINT;
				sc6607_set_aicl_point(chip->ic_dev, batt_volt);
			}
			rc = oplus_chg_usb_set_input_current(chip, current_ma, aicl_point);
			goto aicl_rerun;
		}
	}

	if (current_ma <= 0) {
		sc6607_set_input_current_limit(chip, 0);
		return 0;
	}

	if (current_ma < 500) {
		i = 0;
		goto aicl_end;
	}

	i = 1; /* 500 */
	sc6607_set_input_current_limit(chip, usb_icl[i]);
	msleep(AICL_DELAY_MS);

	chg_vol = sc6607_adc_read_vbus_volt(chip);
	if (chg_vol < aicl_point_temp) {
		chg_info("use 500 here\n");
		goto aicl_end;
	} else if (current_ma < 900)
		goto aicl_end;

	i = 2; /* 900 */
	sc6607_set_input_current_limit(chip, usb_icl[i]);
	msleep(AICL_DELAY_MS);
	chg_vol = sc6607_adc_read_vbus_volt(chip);
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 1200)
		goto aicl_end;

	i = 3; /* 1200 */
	sc6607_set_input_current_limit(chip, usb_icl[i]);
	msleep(AICL_DELAY_MS);
	chg_vol = sc6607_adc_read_vbus_volt(chip);
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1500 */
	aicl_point_temp = aicl_point + 50;
	sc6607_set_input_current_limit(chip, usb_icl[i]);
	msleep(AICL_DELAY2_MS);
	chg_vol = sc6607_adc_read_vbus_volt(chip);
	if (chg_vol < aicl_point_temp) {
		i = i - 2;
		goto aicl_pre_step;
	} else if (current_ma < 1500) {
		i = i - 1;
		goto aicl_end;
	} else if (current_ma < 2000)
		goto aicl_end;

	i = 5; /* 1750 */
	aicl_point_temp = aicl_point + 50;
	sc6607_set_input_current_limit(chip, usb_icl[i]);
	msleep(AICL_DELAY2_MS);
	chg_vol = sc6607_adc_read_vbus_volt(chip);
	if (chg_vol < aicl_point_temp) {
		i = i - 2;
		goto aicl_pre_step;
	}

	i = 6; /* 2000 */
	aicl_point_temp = aicl_point;
	sc6607_set_input_current_limit(chip, usb_icl[i]);
	msleep(AICL_DELAY_MS);
	chg_vol = sc6607_adc_read_vbus_volt(chip);
	if (chg_vol < aicl_point_temp) {
		i = i - 2;
		goto aicl_pre_step;
	} else if (current_ma < 3000)
		goto aicl_end;

	i = 7; /* 3000 */
	sc6607_set_input_current_limit(chip, usb_icl[i]);
	msleep(AICL_DELAY_MS);
	chg_vol = sc6607_adc_read_vbus_volt(chip);
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma >= 3000)
		goto aicl_end;

aicl_pre_step:
	sc6607_set_input_current_limit(chip, usb_icl[i]);
	chg_info("aicl_pre_step: current limit aicl chg_vol = %d j[%d] = %d sw_aicl_point:%d, \
		main %d mA, slave %d mA\n",
		chg_vol, i, usb_icl[i], aicl_point_temp, main_cur, slave_cur);
	return rc;
aicl_end:
	sc6607_set_input_current_limit(chip, usb_icl[i]);
	chg_info("aicl_end: current limit aicl chg_vol = %d j[%d] = %d sw_aicl_point:%d, \
		main %d mA, slave %d mA\n",
		chg_vol, i, usb_icl[i], aicl_point_temp, main_cur, slave_cur);
aicl_rerun:
	return rc;
}

static int oplus_sc6607_charging_disable(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	chg_info("disable");
	chip->hw_aicl_point = SC6607_HW_AICL_POINT_VOL_5V_PHASE1;
	sc6607_set_input_volt_limit(chip, chip->hw_aicl_point);

	return sc6607_disable_charger(chip);
}

static int oplus_sc6607_is_charging_enabled(struct sc6607 *chip)
{
	int ret;
	u8 val = 0;

	if (!chip)
		return -EINVAL;

	ret = sc6607_field_read(chip, F_CHG_EN, &val);
	chg_info("enabled:%d\n", val);

	return val;
}

static int oplus_sc6607_request_otg_on(struct sc6607 *chip, int index)
{
	int ret = 0;
	int try_count = 10;
	u8 boost_good = false;

	if (!chip)
		return -EINVAL;

	if (!index) {
		set_bit(BOOST_ON_OTG, &chip->request_otg);
	} else {
		sc6607_field_write(chip, F_ACDRV_MANUAL_EN, false);
		set_bit(BOOST_ON_CAMERA, &chip->request_otg);
	}

	ret = sc6607_get_otg_status(chip);
	if (ret) {
		chg_info("already enable, index=%d, ret=%d, request_otg=%ld\n", index, ret, chip->request_otg);
		return ret;
	}

	sc6607_disable_charger(chip);
	sc6607_field_write(chip, F_DIS_SLEEP_FOR_OTG, true);
	msleep(5);
	ret = sc6607_enable_otg(chip);
	if (ret < 0) {
		chg_err("enable otg fail:%d\n", ret);
		return ret;
	}

	do {
		msleep(10);
		ret = sc6607_field_read(chip, F_BOOST_GOOD, &boost_good);
		if (ret < 0) {
			chg_err("read boost good fail:%d\n", ret);
		}
	} while ((try_count--) && (!boost_good));

	if (!boost_good) {
		sc6607_enable_charger(chip);
		if (!index)
			clear_bit(BOOST_ON_OTG, &chip->request_otg);
		else {
			sc6607_field_write(chip, F_ACDRV_MANUAL_EN, true);
			clear_bit(BOOST_ON_CAMERA, &chip->request_otg);
		}
		ret = -EINVAL;
	} else {
		ret = true;
	}
	chg_info("index=%d, ret=0x%x, request_otg=%ld\n", index, ret, chip->request_otg);

	return ret;
}

static int oplus_sc6607_request_otg_off(struct sc6607 *chip, int index)
{
	int ret = 0;
	int mmi_chg = 1;

	if (!chip)
		return -EINVAL;

	if (!index)
		clear_bit(BOOST_ON_OTG, &chip->request_otg);
	else
		clear_bit(BOOST_ON_CAMERA, &chip->request_otg);
	oplus_chg_get_mmi_state(chip, &mmi_chg);
	if (!chip->request_otg) {
		ret = sc6607_disable_otg(chip);
		if (ret < 0) {
			if (!index)
				set_bit(BOOST_ON_OTG, &chip->request_otg);
			else {
				sc6607_field_write(chip, F_ACDRV_MANUAL_EN, true);
				set_bit(BOOST_ON_CAMERA, &chip->request_otg);
			}
			chg_err("disable otg fail:%d\n", ret);
		} else {
			if (oplus_is_rf_ftm_mode() && !mmi_chg && chip->power_good)
				chg_err("ftm_mode cc abnormal interrupt, not en charging!\n");
			else
				sc6607_enable_charger(chip);
			ret = true;
		}
	}
	chg_info("index=%d, ret=%d, request_otg=%ld\n", index, ret, chip->request_otg);

	return ret;
}

static int oplus_sc6607_enable_otg(struct sc6607 *chip)
{
	int ret = 0;

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->driver_suspended))
		return 0;

	sc6607_field_write(chip, F_ACDRV_MANUAL_EN, 1);
	sc6607_field_write(chip, F_ACDRV_EN, 0);
	msleep(5);
	ret = oplus_sc6607_request_otg_on(chip, BOOST_ON_OTG);
	if (ret > 0) {
		sc6607_field_write(chip, F_QB_EN, 1);
		sc6607_field_write(chip, F_ACDRV_MANUAL_EN, 1);
		sc6607_field_write(chip, F_ACDRV_EN, 1);
		sc6607_disable_watchdog_timer(chip);
	}
	chg_info("request_otg=%ld ret=%d\n", chip->request_otg, ret);

	return ret;
}

static int oplus_sc6607_disable_otg(struct sc6607 *chip)
{
	int ret = 0;

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->driver_suspended))
		return 0;

	ret = oplus_sc6607_request_otg_off(chip, BOOST_ON_OTG);
	if (ret > 0)
		sc6607_field_write(chip, F_QB_EN, 0);

	chg_info("request_otg=%ld ret=%d\n", chip->request_otg, ret);

	return ret;
}

static int oplus_sc6607_charger_suspend(struct sc6607 *chip)
{
	int rc;

	if (!chip)
		return -EINVAL;

	chg_info("[%d, %d]\n", atomic_read(&chip->driver_suspended), atomic_read(&chip->charger_suspended));

	if (atomic_read(&chip->driver_suspended))
		return 0;
	/**
	 *hiz bit has an exception, replace it with f_performance_en
	 */
	rc = sc6607_field_write(chip, F_PERFORMANCE_EN, true);
	if (rc < 0)
		 chg_err("failed to write F_PERFORMANCE_EN, rc = %d\n", rc);

	atomic_set(&chip->charger_suspended, 1);
	return rc;
}

static int oplus_sc6607_charger_unsuspend(struct sc6607 *chip)
{
	int rc = 0;

	if (!chip)
		return -EINVAL;

	chg_info("[%d, %d]\n", atomic_read(&chip->driver_suspended), atomic_read(&chip->charger_suspended));

	if (atomic_read(&chip->driver_suspended))
		return 0;

	atomic_set(&chip->charger_suspended, 0);
	/**
	 *hiz bit has an exception, replace it with f_performance_en
	 */
	rc = sc6607_field_write(chip, F_PERFORMANCE_EN, false);
	if (rc < 0)
		 chg_err("failed to write F_PERFORMANCE_EN, rc = %d\n", rc);
	return rc;
}

static void sc6607_check_ic_suspend(struct sc6607 *chip)
{
	u8 val = 0;

	if (!chip)
		return;

	if (atomic_read(&chip->driver_suspended))
		return;

	if (atomic_read(&chip->charger_suspended)) {
		sc6607_field_read(chip, F_CHG_EN, &val);
		if (val)
			sc6607_field_write(chip, F_CHG_EN, false);

		sc6607_field_read(chip, F_DIS_BUCKCHG_PATH, &val);
		if (!val)
			sc6607_field_write(chip, F_PERFORMANCE_EN, true);
	}
}

static int oplus_sc6607_get_vbus(struct sc6607 *chip)
{
	if (!chip)
		return -EINVAL;

	return sc6607_adc_read_vbus_volt(chip);
}

static void sc6607_init_status_work(struct work_struct *work)
{
	bool pg = false;
	u8 val = 0;
	int ret = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc6607 *chip = container_of(dwork, struct sc6607, init_status_work);

	ret = sc6607_read_byte(chip, SC6607_REG_HK_INT_STAT, &val);
	if (!ret)
		pg = (!!(val & SC6607_HK_VAC_PRESENT_MASK)) && (!!(val & SC6607_HK_VBUS_PRESENT_MASK));

	if (!pg) {
		chg_err("sc6607_init_status_work power not good,return\n");
		return;
	}

	if (chip->oplus_chg_type == POWER_SUPPLY_TYPE_USB_CDP ||
	    chip->oplus_chg_type == POWER_SUPPLY_TYPE_USB) {
		chg_err("sc6607_init_status_work BC12 done, inform charger type\n");
		sc6607_inform_charger_type(chip);
		return;
	}

	chip->wd_rerun_detect = true;	/* Rerun detect for META mode */
	sc6607_hk_irq_handle(chip);
	chg_info("enter\n");
}

static void sc6607_init_status_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc6607 *chip = container_of(dwork, struct sc6607, init_status_check_work);

	sc6607_hk_irq_handle(chip);
	return;
}

static s32 sc6607_thermistor_conver_temp(struct sc6607 *chip, s32 res, struct sc6607_ntc_temp *ntc_param)
{
	int i = 0;
	int asize = 0;
	s32 res1 = 0, res2 = 0;
	s32 tap_value = -2000, tmp1 = 0, tmp2 = 0;

	if (!chip)
		return -EINVAL;

	asize = ntc_param->table_size;

	if (res >= ntc_param->pst_temp_table[0].temperature_r) {
		tap_value = ntc_param->pst_temp_table[0].bts_temp * SC6607_ADC_1000; /* min */
	} else if (res <= ntc_param->pst_temp_table[asize - 1].temperature_r) {
		tap_value = ntc_param->pst_temp_table[asize - 1].bts_temp * SC6607_ADC_1000; /* max */
	} else {
		res1 = ntc_param->pst_temp_table[0].temperature_r;
		tmp1 = ntc_param->pst_temp_table[0].bts_temp;

		for (i = 0; i < asize; i++) {
			if (res >= ntc_param->pst_temp_table[i].temperature_r) {
				res2 = ntc_param->pst_temp_table[i].temperature_r;
				tmp2 = ntc_param->pst_temp_table[i].bts_temp;
				break;
			}
			res1 = ntc_param->pst_temp_table[i].temperature_r;
			tmp1 = ntc_param->pst_temp_table[i].bts_temp;
		}
		tap_value = (((res - res2) * tmp1) * SC6607_ADC_1000 + ((res1 - res) * tmp2) * SC6607_ADC_1000) / (res1 - res2);
	}
	if (!chip->platform_data->ntc_suport_1000k)
		tap_value /= SC6607_UV_PER_MV;

	return tap_value;
}

#define SC6607A_NTC_200M_MULTIPLE 2
static u64 sc6607_scale_ntc_1000k_adc(struct sc6607 *chip, u64 adc_value, int adc_module)
{
	if (chip->is_sc6607a)
		return adc_value * sy6607_adc_step[adc_module] / SC6607_ADC_TSBUS_100;

	return adc_value * sy6607_adc_step[adc_module] / SC6607_ADC_TSBUS_200;
}

static u64 sc6607_scale_ntc_default_adc(struct sc6607 *chip, u64 adc_value, int adc_module)
{
	if (chip->is_sc6607a)
		return adc_value * sy6607_adc_step[adc_module] / SC6607_UV_PER_MV
				* SC6607A_NTC_200M_MULTIPLE;

	return adc_value * sy6607_adc_step[adc_module] / SC6607_UV_PER_MV;
}

static int sc6607_tsbus_tsbat_to_convert(struct sc6607 *chip, u64 adc_value, int adc_module)
{
	static struct sc6607_ntc_temp ntc_param = {0};

	if (!chip)
		return 0;

	if (chip->platform_data->ntc_suport_1000k) {
		ntc_param.pst_temp_table = pst_temp_table_1000k;
		ntc_param.table_size = (sizeof(pst_temp_table_1000k) / sizeof(struct sc6607_temp_param));
	} else {
		ntc_param.pst_temp_table = pst_temp_table;
		ntc_param.table_size = (sizeof(pst_temp_table) / sizeof(struct sc6607_temp_param));
	}

	if (chip->platform_data->ntc_suport_1000k) {
		if (adc_module == SC6607_ADC_TSBUS || adc_module == SC6607_ADC_TSBAT) {
			adc_value = sc6607_scale_ntc_1000k_adc(chip, adc_value, adc_module);
			adc_value = SC6607_ADC_1000 * SC6607_ADC_1000 * adc_value / (SC6607_ADC_TSBUS_CONVERT - adc_value);
		}
	} else if (adc_module == ADC_TSBUS_TSBAT_DEFAULT) {
		adc_value = adc_value / SC6607_UV_PER_MV;
	} else if (adc_module == SC6607_ADC_TSBUS || adc_module == SC6607_ADC_TSBAT) {
		adc_value = sc6607_scale_ntc_default_adc(chip, adc_value, adc_module);
	}

	adc_value = sc6607_thermistor_conver_temp(chip, adc_value, &ntc_param);
	return adc_value;
}

static int sc6607_get_tsbus(struct sc6607 *chip)
{
	int ret = 0;

	if (!chip)
		return -EINVAL;

	if (chip->voocphy && oplus_chg_get_fastchg_commu_ing()) {
		chg_info("svooc in communication\n");
		return sc6607_tsbus_tsbat_to_convert(chip, chip->voocphy->cp_tsbus, SC6607_ADC_TSBUS);
	}
	ret = sc6607_adc_read_tsbus(chip);

	return ret;
}

static int sc6607_get_tsbus_temp(struct thermal_zone_device *tzdev, int *temp)
{
	struct sc6607 *chip;

	if (!tzdev || !temp || !tzdev->devdata)
		return -EINVAL;

	chip = tzdev->devdata;

	*temp = sc6607_get_tsbus(chip);

	return 0;
}

static struct thermal_zone_device_ops charger_temp_ops = {
	.get_temp = sc6607_get_tsbus_temp,
};

static int register_charger_thermal(struct sc6607 *chip)
{
	int ret = 0;

	if (!chip)
		return -EINVAL;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	chip->tz_dev = thermal_tripless_zone_device_register("charger_temp", chip, &charger_temp_ops, NULL);
#else
	chip->tz_dev = thermal_zone_device_register("charger_temp", 0, 0, chip, &charger_temp_ops, NULL, 0, 0);
#endif
	if (IS_ERR(chip->tz_dev)) {
		chg_err("charger_temp register fail");
		ret = -ENODEV;
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	ret = thermal_zone_device_enable(chip->tz_dev);
	if (ret)
		thermal_zone_device_unregister(chip->tz_dev);
#endif
	return ret;
}

static int sc6607_get_tsbat(struct sc6607 *chip)
{
	int ret = 0;

	if (!chip)
		return -EINVAL;

	if (chip->voocphy && oplus_chg_get_fastchg_commu_ing()) {
		chg_info("svooc in communication\n");
		return sc6607_tsbus_tsbat_to_convert(chip, chip->voocphy->cp_tsbat, SC6607_ADC_TSBAT);
	}

	ret = sc6607_adc_read_tsbat(chip);

	return ret;
}

static int sc6607_track_check_buck_err(struct sc6607 *chip)
{
	int ret;
	u8 data;

	ret = sc6607_read_byte(chip, SC6607_REG_CHG_FLT_FLG, &data);
	if (ret < 0) {
		chg_err("read 0x%x failed\n", SC6607_REG_CHG_FLT_FLG);
		return -EINVAL;
	}

	chg_info("read reg[0x%0x] = 0x%x \n", SC6607_REG_CHG_FLT_FLG, data);
	return 0;
}

static int sc6607_track_match_hk_err(struct sc6607 *chip, u8 data)
{
	return 0;
}

struct oplus_chg_ic_virq sc6607_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_PLUGIN },
	{ .virq_id = OPLUS_IC_VIRQ_CHG_TYPE_CHANGE },
	{ .virq_id = OPLUS_IC_VIRQ_BC12_COMPLETED },
};

static int sc6607_init(struct oplus_chg_ic_dev *ic_dev)
{
	ic_dev->online = true;
	return 0;
}

static int sc6607_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	return 0;
}

static int sc6607_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int sc6607_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int sc6607_input_present(struct oplus_chg_ic_dev *ic_dev, bool *present)
{
	int rc = 0;
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*present = chip->power_good;

	return rc;
}


static int sc6607_input_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	int rc = 0;
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (suspend)
		rc = oplus_sc6607_charger_suspend(chip);
	else
		rc = oplus_sc6607_charger_unsuspend(chip);

	chg_info("charger input %s, rc = %d\n", suspend ? "suspend" : "unsuspend", rc);

	return rc;
}

static int sc6607_input_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	return 0;
}

static int sc6607_output_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	int rc = 0;
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (suspend)
		rc = oplus_sc6607_charging_disable(chip);
	else
		rc = sc6607_enable_charger(chip);

	chg_info("charger out %s, rc = %d", suspend ? "suspend" : "unsuspend", rc);

	return rc;
}

static int sc6607_output_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*suspend = oplus_sc6607_is_charging_enabled(chip);
	return 0;
}

static int sc6607_set_icl(struct oplus_chg_ic_dev *ic_dev, bool vooc_mode, bool step, int icl_ma)
{
	int rc = 0;
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip->bc12_done && step) {
		chg_info("bc12_done = %d, skip aicl\n", chip->bc12_done);
		return rc;
	}

	if (step)
		rc = oplus_sc6607_set_aicr(chip, icl_ma);
	else
		rc = sc6607_set_input_current_limit(chip, icl_ma);
	return rc;
}

static int sc6607_get_icl(struct oplus_chg_ic_dev *ic_dev, int *icl_ma)
{
	return 0;
}

static int sc6607_set_fcc(struct oplus_chg_ic_dev *ic_dev, int fcc_ma)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip->bc12_done && fcc_ma > SC6607_BUCK_ICHG_500MA) {
		chg_info("bc12_done = %d, fcc_ma = %d, force fcc_ma to %d\n",
				chip->bc12_done, fcc_ma, SC6607_BUCK_ICHG_500MA);
		fcc_ma = SC6607_BUCK_ICHG_500MA;
	}

	return oplus_sc6607_set_ichg(chip, fcc_ma);
}

static int sc6607_set_fv(struct oplus_chg_ic_dev *ic_dev, int fv_mv)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	if (fv_mv <= 0) {
		chg_err("invalid value ignore");
		return 0;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	return sc6607_set_chargevolt(chip, fv_mv);
}

static int sc6607_set_iterm(struct oplus_chg_ic_dev *ic_dev, int iterm_ma)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	return sc6607_set_term_current(chip, iterm_ma);
}

static int sc6607_set_rechg_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	return 0;
}

static int sc6607_get_input_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*curr_ma = sc6607_adc_read_ibus(chip);

	return 0;
}

static int sc6607_get_input_vol(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	struct sc6607 *chip;
	uint8_t value = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	sc6607_field_read(chip, F_ACDRV_EN, &value);
	if (oplus_is_power_off_charging() && (value == 0 || chip->power_good == 0)) {
		*vol_mv = 0;
	} else {
		*vol_mv = oplus_sc6607_get_vbus(chip);
	}

	return 0;
}

static int sc6607_otg_boost_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc;
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (en)
		rc = oplus_sc6607_enable_otg(chip);
	else
		rc = oplus_sc6607_disable_otg(chip);
	if (rc < 0)
		chg_err("can't %s otg boost, rc=%d\n", en ? "enable" : "disable", rc);
	return rc;
}

static int sc6607_set_otg_boost_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	return 0;
}

static int sc6607_set_otg_boost_curr_limit(struct oplus_chg_ic_dev *ic_dev, int curr_mA)
{
	int rc;
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = sc6607_set_boost_current(chip, curr_mA);
	if (rc)
		chg_err("failed to set boost current, ret = %d\n", rc);
	return 0;
}

static int sc6607_aicl_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	return rc;
}

static int sc6607_aicl_rerun(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	return rc;
}

static int sc6607_aicl_reset(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	return rc;
}

static int sc6607_get_bc12_result(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (chip->oplus_chg_type) {
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

static int sc6607_rerun_bc12(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int sc6607_qc_detect_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int ret;
	int retry = QC_DETECT_RETRY;
	u8 vbus_stat;
	struct sc6607 *chip;
	struct votable *icl_votable;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chg_info("%d\n", en);
	if (!en)
		return 0;

	if (chip->disable_qc) {
		chg_info("not support qc\n");
		return 0;
	}

	if (HVDCP_EXIT_ABNORMAL == chip->hvdcp_exit_stat) {
		chg_err("HVDCP_EXIT_ABNORMAL not enable hvdcp \n");
		return 0;
	}

	sc6607_detect_init(chip);
	sc6607_enable_hvdcp(chip);
	sc6607_force_dpdm(chip, true);

	while (retry--) {
		chg_info("hvdcp detect retry:%d", retry);
		msleep(20);
		ret = sc6607_field_read(chip, F_VBUS_STAT, &vbus_stat);
		chip->vbus_type = vbus_stat;
		if (vbus_stat == SC6607_VBUS_TYPE_HVDCP) {
			chip->hvdcp_can_enabled = true;
			chip->oplus_chg_type = POWER_SUPPLY_TYPE_USB_HVDCP;
			oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			chg_info("hvdcp_can_enabled set true");
			break;
		}
	}
	if (chip->oplus_chg_type != POWER_SUPPLY_TYPE_USB_HVDCP) {
		icl_votable = find_votable("WIRED_ICL");
		if (!icl_votable)
			chg_err("WIRED_ICL votable not found\n");
		else
			rerun_election(icl_votable, true);
	}
	sc6607_detect_release(chip);
	return 0;
}

static void sc6607_switch_to_hvdcp(struct sc6607 *chip, enum sc6607_hvdcp_type type)
{
	if (!chip)
		return;

	switch (type) {
	case HVDCP_5V:
		chg_info("set_to_5v start\n");
		sc6607_field_write(chip, F_DP_DRIV, DP_0_6);
		sc6607_field_write(chip, F_DM_DRIV, DM_0_0);
		break;
	case HVDCP_9V:
		chg_info("set_to_9v start\n");
		sc6607_field_write(chip, F_DP_DRIV, DP_3_3);
		sc6607_field_write(chip, F_DM_DRIV, DP_0_6);
		break;
	default:
		chg_err(" not support now\n");
		break;
	}
	return;
}

static void oplus_notify_hvdcp_detach_stat(struct sc6607 *chip)
{
	if (!chip)
		return;

	chip->hvdcp_detach_time = cpu_clock(smp_processor_id()) / CONV_DETACH_TIME;
	chg_info("the hvdcp_detach_time:%llu %llu %d %d\n",
		chip->hvdcp_detach_time, chip->hvdcp_detect_time,
		OPLUS_HVDCP_DETECT_TO_DETACH_TIME, chip->hvdcp_cfg_9v_done);
	if (chip->hvdcp_cfg_9v_done &&
	    (chip->hvdcp_detach_time - chip->hvdcp_detect_time <= OPLUS_HVDCP_DETECT_TO_DETACH_TIME)) {
		chip->hvdcp_exit_stat = HVDCP_EXIT_ABNORMAL;
	} else {
		chip->hvdcp_exit_stat = HVDCP_EXIT_NORMAL;
	}
	chip->hvdcp_detect_time = 0;
	chip->hvdcp_detach_time = 0;
	chip->hvdcp_cfg_9v_done = false;
}

static void oplus_notify_hvdcp_detect_stat(struct sc6607 *chip)
{
	if (!chip)
		return;

	chip->hvdcp_cfg_9v_done = true;
	chip->hvdcp_detect_time = cpu_clock(smp_processor_id()) / CONV_DETACH_TIME;
	chg_info("HVDCP2 detect: %d, the detect time: %llu\n", chip->hvdcp_cfg_9v_done, chip->hvdcp_detect_time);
}

static void sc6607_qc_vol_convert(struct work_struct *work)
{
	int retry = CONVERT_RETRY_COUNT;
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc6607 *chip = container_of(dwork, struct sc6607, qc_vol_convert_work);

	if (!chip->pdqc_setup_5v) {
		if (oplus_sc6607_get_vbus(chip) < SC6607_5V_THRES_MV) {
			sc6607_detect_init(chip);
			msleep(CONVERY_DELAY_MS);
			oplus_notify_hvdcp_detect_stat(chip);
			sc6607_switch_to_hvdcp(chip, HVDCP_9V);
			while(retry--) {
				if (oplus_sc6607_get_vbus(chip) > SC6607_9V_THRES1_MV) {
					chg_info("set_to_9v success\n");
					break;
				}
				msleep(CONVERY_DELAY_MS);
			}
			chg_info("set_to_9v complete\n");
		} else {
			chg_err("set_to_9v already 9V\n");
		}
	} else {
		if (oplus_sc6607_get_vbus(chip) > SC6607_9V_THRES_MV) {
			sc6607_detect_init(chip);
			msleep(CONVERY_DELAY_MS);
			sc6607_switch_to_hvdcp(chip, HVDCP_5V);
			while(retry--) {
				if (oplus_sc6607_get_vbus(chip) < SC6607_5V_THRES_MV) {
					chg_info("set_to_5v success\n");
					break;
				}
				msleep(CONVERY_DELAY_MS);
			}
			chg_info("set_to_5v complete\n");
		} else {
			chg_err("set_to_5v already 5V\n");
		}
	}
	return;
}

static int sc6607_set_qc_config(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_qc_version version, int vol_mv)
{
	int ret = 0;
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (chip->disable_qc) {
		chg_err("not support QC\n");
		return -EINVAL;
	}

	chg_info("\n");

	switch (version) {
	case OPLUS_CHG_QC_2_0:
		if (vol_mv != 5000 && vol_mv != 9000) {
			chg_err("Unsupported qc voltage(=%d)\n", vol_mv);
			return -EINVAL;
		}

		if (vol_mv == 9000) {
			chg_info("set qc to 9V, count:%d\n", chip->qc_to_9v_count);
			chip->pdqc_setup_5v = false;

			if (chip->qc_to_9v_count > BLACK_COOL_DOWN_COUNT) {
				chg_info("set hvdcp_can_enabled as false and disable hvdcp\n");
				chip->hvdcp_can_enabled = false;
				sc6607_disable_hvdcp(chip);
				ret = -EINVAL;
			} else {
				schedule_delayed_work(&chip->qc_vol_convert_work, 0);
				chip->qc_to_9v_count++;
			}
		} else {
			chg_info("set qc to 5V\n");
			chip->qc_to_9v_count = 0;
			chip->pdqc_setup_5v = true;
			schedule_delayed_work(&chip->qc_vol_convert_work, 0);
		}
		break;
	case OPLUS_CHG_QC_3_0:
		break;
	default:
		chg_err("Unsupported qc version(=%d)\n", version);
		break;
	}

	return ret;
}

static int sc6607_shipmode_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	sc6607_enter_ship_mode(chip, true);

	return 0;
}

static int sc6607_get_usb_btb_temp(struct oplus_chg_ic_dev *ic_dev, int *usb_btb_temp)
{
	int rc = 0;
	struct sc6607 *chip;
	int temp = BTB_DEFAULT_TEMP;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (chip->not_support_usb_btb) {
		*usb_btb_temp = temp;
	} else if (chip->usb_btb_temp_chan) {
		rc = iio_read_channel_processed(chip->usb_btb_temp_chan, &temp);
		if (rc < 0)
			chg_err("read batt_btb_err\n");
		else {
			temp = temp / UNIT_TRANS_1000;
			*usb_btb_temp = temp;
		}
	} else {
		if (chip->sc6607_switch_ntc) {
			*usb_btb_temp = sc6607_get_tsbat(chip);
		} else {
			*usb_btb_temp = sc6607_get_tsbus(chip);
		}
		if (chip->platform_data->ntc_suport_1000k)
			*usb_btb_temp = *usb_btb_temp / UNIT_TRANS_1000;
	}
	return 0;
}

static int sc6607_get_batt_btb_temp(struct oplus_chg_ic_dev *ic_dev, int *batt_btb_temp)
{
	struct sc6607 *chip;
	int rc = 0;
	int temp = BTB_DEFAULT_TEMP;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (chip->batt_btb_temp_chan) {
		rc = iio_read_channel_processed(chip->batt_btb_temp_chan, &temp);
		if (rc < 0)
			chg_err("read batt_btb_err\n");
		else {
			temp = temp / UNIT_TRANS_1000;
			*batt_btb_temp = temp;
		}
	} else {
		if (chip->sc6607_switch_ntc)
			*batt_btb_temp = sc6607_get_tsbus(chip);
		else
			*batt_btb_temp = sc6607_get_tsbat(chip);

		if (chip->platform_data->ntc_suport_1000k)
			*batt_btb_temp = *batt_btb_temp / UNIT_TRANS_1000;
	}
	return 0;
}

static int sc6607_set_aicl_point(struct oplus_chg_ic_dev *ic_dev, int vbatt)
{
	bool present = false;
	int chg_vol = 0;
	int chg_type = 0;
	int charger_type;
	int rc = 0;
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chg_vol = sc6607_adc_read_vbus_volt(chip);
	chg_type = oplus_wired_get_chg_type();
	if (chg_vol > SC6607_9V_THRES1_MV &&
	    (chg_type == OPLUS_CHG_USB_TYPE_PD || chg_type == OPLUS_CHG_USB_TYPE_QC2)) {
		if (oplus_gauge_get_batt_num() == 1)
			chip->hw_aicl_point = SC6607_AICL_POINT_VOL_9V;
		else
			chip->hw_aicl_point = SC6607_DUAL_AICL_POINT_VOL_9V;
	} else {
		if (chip->hw_aicl_point > SC6607_HW_AICL_POINT_VOL_5V_PHASE3 || chip->hw_aicl_point == USB_HW_AICL_POINT)
			chip->hw_aicl_point = SC6607_HW_AICL_POINT_VOL_5V_PHASE3;
		if (chip->hw_aicl_point == SC6607_HW_AICL_POINT_VOL_5V_PHASE2 && vbatt > SC6607_AICL_POINT_VOL_5V_HIGH1) {
			chip->hw_aicl_point = SC6607_HW_AICL_POINT_VOL_5V_PHASE3;
		} else if (chip->hw_aicl_point == SC6607_HW_AICL_POINT_VOL_5V_PHASE1 && vbatt > SC6607_AICL_POINT_VOL_5V_HIGH) {
			chip->hw_aicl_point = SC6607_HW_AICL_POINT_VOL_5V_PHASE2;
		} else if (chip->hw_aicl_point == SC6607_HW_AICL_POINT_VOL_5V_PHASE2 && vbatt < SC6607_AICL_POINT_VOL_5V_MID) {
			chip->hw_aicl_point = SC6607_HW_AICL_POINT_VOL_5V_PHASE1;
		}

		sc6607_input_present(chip->ic_dev, &present);
		rc = sc6607_get_bc12_result(chip->ic_dev, &charger_type);
		if (rc >= 0 && chip->usb_aicl_enhance &&
		    charger_type == OPLUS_CHG_USB_TYPE_SDP && present)
			chip->hw_aicl_point = USB_HW_AICL_POINT;
	}
	sc6607_set_input_volt_limit(chip, chip->hw_aicl_point);
	return 0;
}

static int sc6607_hardware_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc6607 *chip;

	chg_info("start\n");
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chip->hw_aicl_point = SC6607_HW_AICL_POINT_VOL_5V_PHASE1;
	sc6607_set_input_volt_limit(chip, chip->hw_aicl_point);
	if (!strcmp(chip->chg_dev_name, "primary_chg")) {
		if (oplus_is_rf_ftm_mode()) {
			sc6607_disable_charger(chip);
			oplus_sc6607_charger_suspend(chip);
		} else {
			if (!oplus_chg_get_cp_enable(chip)) {
				oplus_sc6607_charger_unsuspend(chip);
			}
			sc6607_enable_charger(chip);
		}

		if (atomic_read(&chip->charger_suspended))
			chg_info("ignore set current=500mA\n");
		else {
			sc6607_set_input_current_limit(chip, SC6607_DEFAULT_IBUS_MA);
		}
	}

	return 0;
}

static int sc6607_kick_wdt(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc6607 *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (atomic_read(&chip->charger_suspended))
		return 0;
	rc = sc6607_reset_watchdog_timer(chip);
	if (rc)
		chg_err("Couldn't kick wdt rc = %d\n", rc);
	return rc;
}

static int sc6607_get_usb_aicl_enhance(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*enable = chip->usb_aicl_enhance;

	return 0;
}

static int sc6607_chg_set_flash_mode(struct oplus_chg_ic_dev *ic_dev, bool flash_mode)
{
	int ret = 0;
	struct sc6607 *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chg_info("set flash mode to %s\n", flash_mode ? "true" : "false");
	if (flash_mode) {
		ret = oplus_sc6607_request_otg_on(chip, BOOST_ON_CAMERA);
	} else {
		ret = oplus_sc6607_request_otg_off(chip, BOOST_ON_CAMERA);
		msleep(FLASH_MODE_DELAY);
	}
	return ret;
}

static int sc6607_chg_set_pd_config(struct oplus_chg_ic_dev *ic_dev, u32 pdo)
{
	struct sc6607 *chip;
	int vol_mv;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (PD_SRC_PDO_TYPE(pdo)) {
	case PD_SRC_PDO_TYPE_FIXED:
		vol_mv = PD_SRC_PDO_FIXED_VOLTAGE(pdo) * 50;
		chip->pd_chg_volt = vol_mv;
		chg_info("pd_chg_volt=%d\n", chip->pd_chg_volt);
		break;
	default:
		chg_err("Unsupported pdo type(=%d)\n", PD_SRC_PDO_TYPE(pdo));
		return -EINVAL;
	}

	return 0;
}

static int sc6607_chg_set_usbtemp_dischg_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct sc6607 *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = sc6607_field_write(chip, F_ACDRV_EN, !en);
	if (rc < 0)
		 chg_err("failed to write F_PERFORMANCE_EN, rc = %d\n", rc);
	chg_info("set_usbtemp_dischg_enable=%d\n", en);

	return 0;
}

static void *sc6607_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) && (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, sc6607_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, sc6607_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, sc6607_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, sc6607_smt_test);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_PRESENT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_PRESENT, sc6607_input_present);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND, sc6607_input_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND, sc6607_input_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND, sc6607_output_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND, sc6607_output_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ICL, sc6607_set_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_ICL, sc6607_get_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FCC, sc6607_set_fcc);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FV, sc6607_set_fv);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ITERM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ITERM, sc6607_set_iterm);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL, sc6607_set_rechg_vol);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR, sc6607_get_input_curr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL, sc6607_get_input_vol);
		break;
	case OPLUS_IC_FUNC_OTG_BOOST_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_OTG_BOOST_ENABLE, sc6607_otg_boost_enable);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_VOL, sc6607_set_otg_boost_vol);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT, sc6607_set_otg_boost_curr_limit);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_ENABLE, sc6607_aicl_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RERUN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RERUN, sc6607_aicl_rerun);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RESET, sc6607_aicl_reset);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE, sc6607_get_bc12_result);
		break;
	case OPLUS_IC_FUNC_BUCK_RERUN_BC12:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_RERUN_BC12, sc6607_rerun_bc12);
		break;
	case OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE, sc6607_qc_detect_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG, sc6607_set_qc_config);
		break;
	case OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE, sc6607_shipmode_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP, sc6607_get_usb_btb_temp);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP, sc6607_get_batt_btb_temp);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_AICL_POINT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_AICL_POINT, sc6607_set_aicl_point);
		break;
	case OPLUS_IC_FUNC_BUCK_HARDWARE_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_HARDWARE_INIT, sc6607_hardware_init);
		break;
	case OPLUS_IC_FUNC_BUCK_KICK_WDT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_KICK_WDT, sc6607_kick_wdt);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE, sc6607_get_usb_aicl_enhance);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FLASH_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FLASH_MODE, sc6607_chg_set_flash_mode);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG, sc6607_chg_set_pd_config);
		break;
	case OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE, sc6607_chg_set_usbtemp_dischg_enable);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static int sc6607_irq_gpio_init(struct sc6607 *chip)
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

	chip->charging_inter_active = pinctrl_lookup_state(chip->pinctrl, "charging_inter_active");
	if (IS_ERR_OR_NULL(chip->charging_inter_active)) {
		chg_err("failed to get the pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	chip->charging_inter_sleep = pinctrl_lookup_state(chip->pinctrl, "charging_inter_sleep");
	if (IS_ERR_OR_NULL(chip->charging_inter_sleep)) {
		chg_err("failed to get the pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	gpio_direction_input(chip->irq_gpio);
	pinctrl_select_state(chip->pinctrl, chip->charging_inter_active); /* no_PULL */
	rc = gpio_get_value(chip->irq_gpio);
	chg_info("irq_gpio = %d, irq_gpio_stat = %d\n", chip->irq_gpio, rc);

	return 0;
}

static int sc6607_irq_register(struct sc6607 *chip)
{
	struct irq_desc *desc;
	struct cpumask current_mask;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	cpumask_var_t cpu_highcap_mask;
#endif
	int ret;
	if (!chip)
		return -EINVAL;

	ret = sc6607_irq_gpio_init(chip);
	if (ret < 0) {
		chg_err("failed to irq gpio init(%d)\n", ret);
		return ret;
	}

	if (chip->irq) {
		ret = request_threaded_irq(chip->irq, NULL,
					   sc6607_irq_handler,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   "voocphy_irq", chip);
		if (ret < 0) {
			chg_err("request irq for irq=%d failed, ret =%d\n", chip->irq, ret);
			return ret;
		}
		enable_irq_wake(chip->irq);
		chg_debug("request irq ok\n");
	}

	desc = irq_to_desc(chip->irq);
	if (desc == NULL) {
		free_irq(chip->irq, chip);
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

static void sc6607_flash_mode_checkout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc6607 *chip = container_of(dwork, struct sc6607, flash_mode_checkout_work);

	chg_info("\n");
	if (!chip || !chip->ic_dev) {
		chg_info("chip or ic_dev null");
		return;
	}

	if (oplus_sc6607_get_vbus(chip) < SC6607_VINDPM_THRES_MIN)
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PLUGIN);
	return;
}

static void sc6607_comm_subs_callback(struct mms_subscribe *subs, enum mms_msg_type type, u32 id, bool sync)
{
	struct sc6607 *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_FLASH_MODE:
			oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_FLASH_MODE, &data, false);
			chip->camera_on = data.intval;
			chg_info("set flash mode to %s\n", chip->camera_on ? "true" : "false");
			if (chip->camera_on) {
				cancel_delayed_work_sync(&chip->flash_mode_checkout_work);
				schedule_delayed_work(&chip->flash_mode_checkout_work,
									msecs_to_jiffies(FLASH_MODE_CHECKOUT_DELAY));
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return;
}

static void sc6607_subscribe_comm_topic(struct oplus_mms *topic, void *prv_data)
{
	struct sc6607 *chip = prv_data;

	chip->comm_topic = topic;
	chip->comm_subs = oplus_mms_subscribe(chip->comm_topic, chip, sc6607_comm_subs_callback, chip->ic_dev->manu_name);
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe comm topic error, rc=%ld\n", PTR_ERR(chip->comm_subs));
		return;
	}
}

static void sc6607_subscribe_wired_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct sc6607 *chip = prv_data;
	/* waiting for wired initial and report bc12-complete, needn't callback */
	if (chip->oplus_chg_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		chg_info("report bc12 complete again\n");
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_BC12_COMPLETED);
	}
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
static struct charger_ops sc6607_chg_ops = {
	.plug_in = sc6607_plug_in,
	.plug_out = sc6607_plug_out,
	.kick_wdt = sc6607_charge_kick_wdt,
	.enable = sc6607_charge_enable,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
static enum power_supply_usb_type sc6607_charger_usb_types[] = {
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

static enum power_supply_property sc6607_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static int sc6607_charger_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sc6607 *chip;
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
			val->intval = chip->oplus_chg_type;
		}
#endif
		chg_info("sc6607 get power_supply_type = %d\n", val->intval);
		break;
	default:
		ret = -ENODATA;
	}
	return ret;
}

static char *sc6607_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static const struct power_supply_desc sc6607_charger_desc = {
	.type	= POWER_SUPPLY_TYPE_USB,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	.usb_types	= sc6607_charger_usb_types,
	.num_usb_types	= ARRAY_SIZE(sc6607_charger_usb_types),
#else
	.usb_types		= BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
				  BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
				  BIT(POWER_SUPPLY_USB_TYPE_C)       |
				  BIT(POWER_SUPPLY_USB_TYPE_PD)      |
				  BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
				  BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
				  BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
#endif
	.properties	= sc6607_charger_properties,
	.num_properties	= ARRAY_SIZE(sc6607_charger_properties),
	.get_property	= sc6607_charger_get_property,
};

static int sc6607_chg_init_psy(struct sc6607 *chip)
{
	struct power_supply_config cfg = {
		.drv_data = chip,
		.of_node = chip->dev->of_node,
		.supplied_to = sc6607_charger_supplied_to,
		.num_supplicants = ARRAY_SIZE(sc6607_charger_supplied_to),
	};

	chg_err("%s\n", __func__);
	memcpy(&chip->psy_desc, &sc6607_charger_desc, sizeof(chip->psy_desc));
	chip->psy_desc.name = "sc6607";
	chip->chg_psy = devm_power_supply_register(chip->dev, &chip->psy_desc, &cfg);
	return IS_ERR(chip->chg_psy) ? PTR_ERR(chip->chg_psy) : 0;
}
#endif

static int find_voocphy_i2c_clients(struct device *dev, void *data)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct sc6607 *chip = data;
	if (client) {
		chg_info("addr=0x%x name:%s\n", client->addr, client->name);
		if (strncmp(client->name, SC6607_CP_NAME, strlen(SC6607_CP_NAME)) == 0) {
			chip->voocphy = i2c_get_clientdata(client);
			chg_info("found\n");
		}
	}
	return 0;
}

static void sc6607_get_voocphy_info_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc6607 *chip = container_of(dwork, struct sc6607, get_voocphy_info_work);
	struct i2c_adapter *adap;

	adap = chip->client->adapter;
	device_for_each_child(&adap->dev, chip, find_voocphy_i2c_clients);
	chip->found_cp_client_count++;
	if (!chip->voocphy && chip->found_cp_client_count < FOUND_CP_ADDR_MAX_COUNT)
		schedule_delayed_work(&chip->get_voocphy_info_work, msecs_to_jiffies(1000));
}

static int pd_tcp_notifier_call(struct notifier_block *nb, unsigned long event, void *data)
{
	int i;
	struct tcp_notify *noti = data;
	struct sc6607 *chip = container_of(nb, struct sc6607, pd_nb);
	uint8_t new_state = TYPEC_UNATTACHED;

	switch (event) {
	case TCP_NOTIFY_PR_SWAP:
		chg_info("power role swap, new role = %d\n", noti->swap_state.new_role);
		if (noti->swap_state.new_role == PD_ROLE_SINK)
			chip->pr_swap = true;
		break;

	case TCP_NOTIFY_SINK_VBUS:
		chg_info("pd type:%d. sink vbus %dmV %dmA type(0x%02X)\n",
		         chip->pd_type, noti->vbus_state.mv, noti->vbus_state.ma, noti->vbus_state.type);
		if (chip->is_sc6607a && (noti->vbus_state.type & TCP_VBUS_CTRL_PR_SWAP) &&
		   (chip->tcpc != NULL) && chip->tcpc->pd_port.pe_data.during_swap)
			sc6607_field_write(chip, F_FORCE_REGN_BYPASS, true);
		if (oplus_chg_get_common_charge_icl_support_flags() &&
		    chip->pd_type != PD_CONNECT_NONE &&
		    noti->vbus_state.ma <= SINK_SUSPEND_CURRENT) {
			cancel_delayed_work_sync(&chip->charger_suspend_recovery_work);
			oplus_chg_suspend_charger(true, TCPC_IBUS_DRAW_VOTER);
			schedule_delayed_work(&chip->charger_suspend_recovery_work,
			                      msecs_to_jiffies(SUSPEND_RECOVERY_DELAY_MS));
		} else if (noti->vbus_state.ma > SINK_SUSPEND_CURRENT) {
			oplus_chg_suspend_charger(false, TCPC_IBUS_DRAW_VOTER);
			/* Cancel PD_PDO_ICL_VOTER suspend to allow set_aicr to re-evaluate max_pdo_current */
			if (oplus_chg_get_common_charge_icl_support_flags()) {
				oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
			}
		}
		break;

	case TCP_NOTIFY_TYPEC_STATE:
		new_state = noti->typec_state.new_state;
		if (new_state == TYPEC_UNATTACHED) {
			chip->pr_swap = false;
			chg_info("typec unattach, set pr_swap false!\n");
		}
		break;

	case TCP_NOTIFY_PD_SOURCECAP_DONE:
		chg_info("PD_SOURCECAP_DONE\n");
		chip->cap_nr = (int)noti->caps_msg.caps->nr;
		for (i = 0; i < chip->cap_nr; i++) {
			chip->pdo[i].pdo_data = (u32)noti->caps_msg.caps->pdos[i];
			chg_info("SourceCap[%d]: %08X\n", i + 1, chip->pdo[i].pdo_data);
		}
		if (oplus_chg_get_common_charge_icl_support_flags())
			schedule_delayed_work(&chip->sourcecap_done_work, 0);
		break;

	case TCP_NOTIFY_PD_STATE:
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			chip->pd_type = PD_CONNECT_NONE;
			oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
			for (i = 0; i < chip->cap_nr; i++)
				chip->pdo[i].pdo_data = 0;
			chip->pd_chg_volt = VBUS_5V;
			chg_info("PD Notify Detach\n");
			break;

		case PD_CONNECT_PE_READY_SNK_APDO:
			chip->pd_type = PD_CONNECT_PE_READY_SNK_APDO;
			chg_info("PD Notify APDO Ready\n");
			break;
		}
		break;

	default:
		break;
	}
	return NOTIFY_OK;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sc6607_buck_probe(struct i2c_client *client)
#else
static int sc6607_buck_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct sc6607 *chip;
	struct device_node *node = client->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int ret = 0;
	int i = 0;

	chg_info("start!\n");
	chip = devm_kzalloc(&client->dev, sizeof(struct sc6607), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &client->dev;
	chip->client = client;

	i2c_set_clientdata(client, chip);
	mutex_init(&chip->i2c_rw_lock);
	mutex_init(&chip->dpdm_lock);
	mutex_init(&chip->bc12.running_lock);
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

	chip->platform_data = sc6607_parse_dt(node, chip);
	if (!chip->platform_data) {
		chg_err("No platform data provided.\n");
		ret = -EINVAL;
		goto err_parse_dt;
	}

	ret = sc6607_reset_chip(chip);
	ret |= sc6607_check_device_id(chip);
	ret |= sc6607_init_device(chip);
	if (ret) {
		chg_err("failed to init device\n");
		goto err_init;
	}

	if (!chip->disable_tcpc_irq) {
		INIT_DELAYED_WORK(&chip->tcpc_complete_work, oplus_tcpc_complete_work);
		schedule_delayed_work(&chip->tcpc_complete_work, 0);
	}
	chip->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (chip->tcpc != NULL) {
		chip->pd_nb.notifier_call = pd_tcp_notifier_call;
		ret = register_tcp_dev_notifier(chip->tcpc, &chip->pd_nb,
				TCP_NOTIFY_TYPE_USB | TCP_NOTIFY_TYPE_MISC | TCP_NOTIFY_TYPE_VBUS);
	} else {
		chg_err("get tcpc dev fail\n");
	}

	atomic_set(&chip->driver_suspended, 0);
	atomic_set(&chip->charger_suspended, 0);
	oplus_chg_awake_init(chip);
	init_waitqueue_head(&chip->wait);
	oplus_keep_resume_awake_init(chip);

	chip->oplus_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->usb_connect_start = false;
	chip->bc12.detect_ing = false;

	/*add hvdcp func*/
	chip->hvdcp_can_enabled = false;
	chip->qc_to_9v_count = 0;
	chip->hvdcp_detect_time = 0;
	chip->hvdcp_detach_time = 0;
	chip->hvdcp_cfg_9v_done = false;
	chip->hvdcp_exit_stat = HVDCP_EXIT_NORMAL;
	timer_setup(&chip->bc12_timeout, sc6607_bc12_timeout_func, 0);
	sc6607_disable_hvdcp(chip);
	INIT_DELAYED_WORK(&(chip->bc12.detect_work), sc6607_soft_bc12_work_func);
	INIT_DELAYED_WORK(&chip->hw_bc12_detect_work, sc6607_hw_bc12_work_func);
	INIT_DELAYED_WORK(&chip->init_status_work, sc6607_init_status_work);
	INIT_DELAYED_WORK(&chip->init_status_check_work, sc6607_init_status_check_work);
	INIT_DELAYED_WORK(&chip->qc_vol_convert_work, sc6607_qc_vol_convert);
	INIT_DELAYED_WORK(&chip->get_voocphy_info_work, sc6607_get_voocphy_info_work);
	INIT_DELAYED_WORK(&chip->flash_mode_checkout_work, sc6607_flash_mode_checkout_work);
	INIT_WORK(&chip->rerun_votable_work, sc6607_rerun_votable_work);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	ret = sc6607_chg_init_psy(chip);
	if (ret)
		chg_err("Failed to register sc6607 ret=%d\n", ret);

	chip->chg_dev = charger_device_register(chip->chg_dev_name,
						&client->dev, chip,
						&sc6607_chg_ops,
						&sc6607_chg_props);
	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		ret = PTR_ERR(chip->chg_dev);
		goto err_device_register;
	}
#endif

	sc6607_charger_create_device_node(chip->dev);
	if (oplus_is_rf_ftm_mode()) {
		chg_info("disable_charger for ftm mode.\n");
		sc6607_enter_hiz_mode(chip);
	}

	if (chip->platform_data->ntc_suport_1000k) {
		ret = register_charger_thermal(chip);
		if (ret < 0)
			chg_err("register_charger_thermal fail\n");
	}
	atomic_set(&chip->otg_enable_cnt, 0);
	chip->request_otg = 0;

	ret = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (ret < 0) {
		chg_err("can't get ic type, ret=%d\n", ret);
		goto err_init;
	}
	ret = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (ret < 0) {
		chg_err("can't get ic index, ret=%d\n", ret);
		goto err_init;
	}
	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "buck-SC6607");
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = sc6607_get_func;
	ic_cfg.virq_data = sc6607_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(sc6607_virq_table);
	ic_cfg.of_node = node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		ret = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto err_init;
	}
	chg_info("register %s\n", node->name);

	ret = sc6607_irq_register(chip);
	if (ret < 0)
		goto err_init;

	if (oplus_is_rf_ftm_mode())
		schedule_delayed_work(&chip->init_status_work, msecs_to_jiffies(INIT_STATUS_TIME_5S));
	else
		schedule_delayed_work(&chip->init_status_check_work, msecs_to_jiffies(INIT_STATUS_DELAY_CHECK));

	if (chip->use_vooc_phy)
		schedule_delayed_work(&chip->get_voocphy_info_work, msecs_to_jiffies(1000));

	sc6607_track_check_buck_err(chip);
	chip->pd_chg_volt = VBUS_5V;
	INIT_DELAYED_WORK(&chip->sourcecap_done_work, oplus_sourcecap_done_work);
	INIT_DELAYED_WORK(&chip->charger_suspend_recovery_work, oplus_charger_suspend_recovery_work);
	oplus_mms_wait_topic("common", sc6607_subscribe_comm_topic, chip);
	oplus_mms_wait_topic("wired", sc6607_subscribe_wired_topic, chip);
	chg_info("end!\n");
	return 0;

#ifdef CONFIG_OPLUS_CHARGER_MTK
err_device_register:
	charger_device_unregister(chip->chg_dev);
#endif
err_init:
	cancel_work_sync(&chip->rerun_votable_work);
	if (!gpio_is_valid(chip->irq_gpio))
		gpio_free(chip->irq_gpio);
err_parse_dt:
err_regmap_fields:
err_regmap:
	mutex_destroy(&chip->bc12.running_lock);
	mutex_destroy(&chip->dpdm_lock);
	mutex_destroy(&chip->i2c_rw_lock);
	mutex_destroy(&chip->adc_read_lock);
	devm_kfree(chip->dev, chip);
	chip = NULL;
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int sc6607_pm_resume(struct device *dev)
{
	struct sc6607 *chip = NULL;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip) {
			chg_info("start\n");
			atomic_set(&chip->driver_suspended, 0);
		}
	}
	return 0;
}

static int sc6607_pm_suspend(struct device *dev)
{
	struct sc6607 *chip = NULL;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip) {
			chg_info("start\n");
			atomic_set(&chip->driver_suspended, 1);
		}
	}
	return 0;
}

static const struct dev_pm_ops sc6607_pm_ops = {
	.resume = sc6607_pm_resume,
	.suspend = sc6607_pm_suspend,
};
#else
static int sc6607_resume(struct i2c_client *client)
{
	struct sc6607 *chip = i2c_get_clientdata(client);

	if (chip)
		atomic_set(&chip->driver_suspended, 0);

	return 0;
}

static int sc6607_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct sc6607 *chip = i2c_get_clientdata(client);

	if (!chip)
		atomic_set(&chip->driver_suspended, 1);

	return 0;
}
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void sc6607_buck_remove(struct i2c_client *client)
#else
static int sc6607_buck_remove(struct i2c_client *client)
#endif
{
	struct sc6607 *chip = i2c_get_clientdata(client);

	if (chip) {
		cancel_work_sync(&chip->rerun_votable_work);
#ifdef CONFIG_OPLUS_CHARGER_MTK
		if (chip->chg_dev)
			charger_device_unregister(chip->chg_dev);
#endif
		if (!IS_ERR_OR_NULL(chip->comm_subs))
			oplus_mms_unsubscribe(chip->comm_subs);
		if (!gpio_is_valid(chip->irq_gpio))
			gpio_free(chip->irq_gpio);
		if (chip->irq)
			free_irq(chip->irq, chip);
		mutex_destroy(&chip->bc12.running_lock);
		mutex_destroy(&chip->dpdm_lock);
		mutex_destroy(&chip->i2c_rw_lock);
		mutex_destroy(&chip->adc_read_lock);
		devm_kfree(chip->dev, chip);
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	return 0;
#endif
}

static void sc6607_buck_shutdown(struct i2c_client *client)
{
	struct sc6607 *chip = i2c_get_clientdata(client);
	uint8_t value = 0;

	if (!chip)
		return;

	chg_info("enter\n");
	sc6607_field_write(chip, F_TSBAT_JEITA_DIS, false);
	sc6607_field_write(chip, F_ADC_EN, 0);
	sc6607_field_write(chip, F_ACDRV_MANUAL_PRE, 3);

	sc6607_set_input_current_limit(chip, SC6607_DEFAULT_IBUS_MA);
	sc6607_field_read(chip, F_ACDRV_EN, &value);
	if (value == 0) {
		sc6607_field_write(chip, F_ACDRV_EN, 1);
	}
	if (oplus_wired_shipmode_is_enabled())
		sc6607_enter_ship_mode(chip, true);

	if((chip->hvdcp_can_enabled) && (chip->power_good)) {
		sc6607_disable_hvdcp(chip);
		sc6607_force_dpdm(chip, true);
	}
	return;
}

static struct of_device_id sc6607_charger_match_table[] = {
	{.compatible = "oplus,sc6607-buck", },
	{},
};
MODULE_DEVICE_TABLE(of, sc6607_charger_match_table);

static const struct i2c_device_id sc6607_buck_device_id[] = {
	{ "sc6607,buck", 0x61 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sc6607_buck_device_id);

static struct i2c_driver sc6607_buck_driver = {
	.driver =
		{
			.name = CHARGER_IC_NAME,
			.owner = THIS_MODULE,
			.of_match_table = sc6607_charger_match_table,
			.pm = &sc6607_pm_ops,
		},

	.probe = sc6607_buck_probe,
	.remove = sc6607_buck_remove,
	.shutdown = sc6607_buck_shutdown,
	.id_table = sc6607_buck_device_id,
};

int sc6607_buck_i2c_driver_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&sc6607_buck_driver) != 0)
		chg_err("failed to register sc6607 buck driver\n");
	else
		chg_info("success to register sc6607 buck driver\n");

	return ret;
}

void sc6607_buck_i2c_driver_exit(void)
{
	i2c_del_driver(&sc6607_buck_driver);
}
oplus_chg_module_register(sc6607_buck_i2c_driver);

MODULE_DESCRIPTION("SC6607 BUCK Driver");
MODULE_LICENSE("GPL v2");

