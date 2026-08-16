// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2024 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[NU1669]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/sched/clock.h>
#include <linux/pinctrl/consumer.h>

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_hal_wls.h>
#include <oplus_chg_wls.h>
#include "oplus_hal_nu1669_fw.h"
#include "oplus_hal_nu1669.h"
#include "../monitor/oplus_chg_track.h"

#ifndef I2C_ERR_MAX
#define I2C_ERR_MAX	6
#endif

#define LDO_ON_MA	100
#define NU1669_AC_OV_FLAG	1  /*This IC has an AC_OV issue when used with the 0x0001 in-vehicle unit*/

enum {
	TX_STATUS_OFF,
	TX_STATUS_ON,
	TX_STATUS_READY,
	TX_STATUS_PING_DEVICE,
	TX_STATUS_TRANSFER,
	TX_STATUS_ERR_RXAC,
	TX_STATUS_ERR_OCP,
	TX_STATUS_ERR_OVP,
	TX_STATUS_ERR_LVP,
	TX_STATUS_ERR_FOD,
	TX_STATUS_ERR_OTP,
	TX_STATUS_ERR_CEPTIMEOUT,
	TX_STATUS_ERR_RXEPT,
	TX_STATUS_ERR_VRECTOVP,
	TX_STATUS_UNKNOW,
};

enum wls_ic_err_reason {
	WLS_IC_ERR_NONE,
	WLS_IC_ERR_RXAC,
	WLS_IC_ERR_OCP,
	WLS_IC_ERR_OVP,
	WLS_IC_ERR_LVP,
	WLS_IC_ERR_FOD,
	WLS_IC_ERR_RXEPT,
	WLS_IC_ERR_OTP,
	WLS_IC_ERR_VRECTOVP,
	WLS_IC_ERR_CLAMPOVP,
	WLS_IC_ERR_HARDOVP,
	WLS_IC_ERR_VOUTOVP,
	WLS_IC_ERR_SOFTOTP,
	WLS_IC_ERR_VOUT2V2X_OVP,
	WLS_IC_ERR_VOUT2V2X_UVP,
	WLS_IC_ERR_V2X_OVP,
	WLS_IC_ERR_V2X_UCP,
	WLS_IC_ERR_I2C,
	WLS_IC_ERR_CRC,
	WLS_IC_ERR_VAC_ACDRV,
	WLS_IC_ERR_OTHER,
};

static const char * const wls_ic_err_reason_text[] = {
	[WLS_IC_ERR_NONE] = "none",
	[WLS_IC_ERR_RXAC] = "rxac",
	[WLS_IC_ERR_OCP] = "ocp",
	[WLS_IC_ERR_OVP] = "ovp",
	[WLS_IC_ERR_LVP] = "lvp",
	[WLS_IC_ERR_FOD] = "fod",
	[WLS_IC_ERR_RXEPT] = "rxept",
	[WLS_IC_ERR_OTP] = "otp",
	[WLS_IC_ERR_VRECTOVP] = "vrectovp",
	[WLS_IC_ERR_CLAMPOVP] = "clampovp",
	[WLS_IC_ERR_HARDOVP] = "hardovp",
	[WLS_IC_ERR_VOUTOVP] = "voutovp",
	[WLS_IC_ERR_SOFTOTP] = "softotp",
	[WLS_IC_ERR_VOUT2V2X_OVP] = "vout2v2xovp",
	[WLS_IC_ERR_VOUT2V2X_UVP] = "vout2v2xuvp",
	[WLS_IC_ERR_V2X_OVP] = "v2xovp",
	[WLS_IC_ERR_V2X_UCP] = "v2xucp",
	[WLS_IC_ERR_I2C] = "i2c_err",
	[WLS_IC_ERR_CRC] = "crc_err",
	[WLS_IC_ERR_VAC_ACDRV] = "vac_acdrv_err",
	[WLS_IC_ERR_OTHER] = "other",
};

#define NU1669_CHECK_LDO_ON_DELAY round_jiffies_relative(msecs_to_jiffies(1000))

struct rx_msg_struct {
	void (*msg_call_back)(void *dev_data, u8 data[]);
	void *dev_data;
};

/*DO NOT modify this structure*/
struct chip_info {
	/*0x00*/
	u16 chip_id;
	u8 hw_version;
	u8 customer_id;
	u8 fw_version;
	u8 pad01[11];
	/*0x10*/
	s8 cep_val;
	u8 rpp_type;
	u8 tx_power_cap;
	u8 max_power;
	u16 tx_manu_id;
	u8 cusr_cmd;
	u8 fw_check;
	u8 magcvr_status;
	u8 pad11[3];
	u32 cep_cnt;
	/*0x20*/
	u32 int_flag;
	u32 int_enable;
	u32 int_clear;
	u32 cmd_buffer_busy;
	/*0x30*/
	u16 vout_set;
	u16 tj_alarm;
	u16 vout;
	u16 iout;
	u16 vrect;
	u16 vds;/*vrect2vout*/
	u16 v2x;/*vbat*2*/
	u16 tj;/*rx ic temp*/
	/*0x40*/
	u32 tx_freq;
	u8 b2b_on;
	u8 acdrv_pd_dis;
	u8 b2b_state;
	u8 vac_state;
	u8 bridge;
	u8 full_bridge_enable;
	u8 ocp;
	u8 max_i;
	u8 standby_enable;
	u8 comu_set;
	u8 ac_state;
	u8 vac_acdrv_state;
	/*0x50~0x60*/
	u8 sys_mode;
	u8 ppp_data[10];
	u8 bc_data[10];
	u8 pad50[11];
	/*0x70~0x80*/
	u8 rx_fod_arg1[16];
	u8 rx_fod_arg2[16];
	/*0x90*/
	u8 trx_ss;
	u8 trx_cep;
	u8 trx_rpp;
	u8 trx_status;
	u8 trx_max_power;
	u8 trx_ept_code;
	u16 trx_manu_id;
	u16 tx_duty;
	u8 pad91[4];
	u8 epp_ref_fq[2];
	/*0xA0*/
	u8 rx_fod_wt_cmd;
	u8 rx_fod_len_cmd;
	u8 rx_fod_rd_cmd;
	u8 rx_fod_para1[10];
	u8 rx_fod_para2[10];
	u8 rx_fod_para3[10];
	u8 rx_fod_para4[10];
	u8 rsv[5];
	/*0xD0*/
} __attribute__((packed));

struct oplus_nu1669 {
	struct i2c_client *client;
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	struct oplus_chg_ic_dev *nor_ic;
	struct oplus_chg_ic_dev *cp_ic;
	struct oplus_mms *err_topic;

	struct chip_info info;

	int rx_con_gpio;
	int rx_con_irq;
	int rx_event_gpio;
	int rx_event_irq;
	int rx_en_gpio;
	int mode_sw_gpio;
	int insert_dis_gpio;
	int rx_mode;
	int rx_en_status;

	bool connected_ldo_on;
	bool rx_connected;
	bool support_epp_11w;
	bool standby_config;

	unsigned char *fw_data;
	int fw_length;
	u32 rx_fw_version;
	u32 tx_fw_version;
	int adapter_type;
	int rx_pwr_cap;
	int tx_status;
	int event_code;

	struct mutex i2c_lock;
	struct mutex pinctrl_lock;

	struct pinctrl *pinctrl;
	struct pinctrl_state *rx_con_default;
	struct pinctrl_state *rx_event_default;
	struct pinctrl_state *rx_en_active;
	struct pinctrl_state *rx_en_sleep;
	struct pinctrl_state *mode_sw_active;
	struct pinctrl_state *mode_sw_sleep;
	struct pinctrl_state *insert_dis_active;
	struct pinctrl_state *insert_dis_sleep;

	struct delayed_work update_work;
	struct delayed_work event_work;
	struct delayed_work connect_work;
	struct delayed_work check_ldo_on_work;
	struct delayed_work rx_mode_work;

	struct regmap *rmap;
	struct rx_msg_struct rx_msg;
	struct completion ldo_on;
	struct completion resume_ack;

	atomic_t i2c_err_count;

	u32 debug_force_ic_err;
};

static struct oplus_nu1669 *g_nu1669_chip = NULL;
static int nu1669_get_running_mode(struct oplus_nu1669 *chip);
static int nu1669_get_power_cap(struct oplus_nu1669 *chip);
static int nu1669_set_rx_enable(struct oplus_chg_ic_dev *dev, bool en);
static bool nu1669_ic_fw_is_valid(struct oplus_nu1669 *chip);
static u8 nu1669_checksum_fw(struct oplus_nu1669 *chip);

__maybe_unused static bool is_nor_ic_available(struct oplus_nu1669 *chip)
{
	struct device_node *node = NULL;

	if (!chip) {
		chg_err("chip is NULL!\n");
		return false;
	}
	node = chip->dev->of_node;
	if (!chip->nor_ic)
		chip->nor_ic = of_get_oplus_chg_ic(node, "oplus,nor_ic", 0);
	return !!chip->nor_ic;
}

__maybe_unused static bool is_err_topic_available(struct oplus_nu1669 *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

static int nu1669_rx_is_connected(struct oplus_chg_ic_dev *dev, bool *connected)
{
	struct oplus_nu1669 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		*connected = false;
		return 0;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	if (!gpio_is_valid(chip->rx_con_gpio)) {
		chg_err("rx_con_gpio invalid\n");
		*connected = false;
		return 0;
	}

	if (!!gpio_get_value(chip->rx_con_gpio))
		*connected = true;
	else
		*connected = false;
	return 0;
}

static int nu1669_get_wls_type(struct oplus_nu1669 *chip)
{
	struct oplus_mms *wls_topic = NULL;
	union mms_msg_data data = { 0 };

	wls_topic = oplus_mms_get_by_name("wireless");
	if (wls_topic) {
		oplus_mms_get_item_data(wls_topic, WLS_ITEM_WLS_TYPE, &data, true);
		return data.intval;
	}
	return OPLUS_CHG_WLS_UNKNOWN;
}

static __inline__ void nu1669_i2c_err_inc(struct oplus_nu1669 *chip)
{
	bool connected = false;

	nu1669_rx_is_connected(chip->ic_dev, &connected);
	if (connected && (atomic_inc_return(&chip->i2c_err_count) > I2C_ERR_MAX)) {
		atomic_set(&chip->i2c_err_count, 0);
		chg_err("read i2c error\n");
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
		/*oplus_chg_anon_mod_event(chip->wls_ocm, OPLUS_CHG_EVENT_RX_IIC_ERR);*/
		/*todo, add i2c error callback*/
	}
}

static __inline__ void nu1669_i2c_err_clr(struct oplus_nu1669 *chip)
{
	atomic_set(&chip->i2c_err_count, 0);
}

#define RESUME_TIMEDOUT_MS	1000
static int nu1669_wait_resume(struct oplus_nu1669 *chip)
{
	int rc;

	rc = wait_for_completion_timeout(&chip->resume_ack, msecs_to_jiffies(RESUME_TIMEDOUT_MS));
	if (!rc) {
		chg_err("wait resume timedout\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int __nu1669_read(struct oplus_nu1669 *chip, u16 addr, u8 *buf)
{
	unsigned int temp = 0;
	int rc;

	rc = regmap_read(chip->rmap, addr, &temp);
	if (rc < 0) {
		chg_err("read 0x%04x error, rc=%d\n", addr, rc);
		return rc;
	}
	*buf = (u8)temp;

	return 0;
}

static int nu1669_read(struct oplus_nu1669 *chip, u16 addr, u8 *buf)
{
	int rc;

	mutex_lock(&chip->i2c_lock);
	rc = __nu1669_read(chip, addr, buf);
	mutex_unlock(&chip->i2c_lock);

	return rc;
}

static int __nu1669_write(struct oplus_nu1669 *chip, u16 addr, u8 val)
{
	int rc;

	rc = regmap_write(chip->rmap, addr, val);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", addr, rc);
		return rc;
	}

	return 0;
}

static int nu1669_write(struct oplus_nu1669 *chip, u16 addr, u8 val)
{
	int rc;

	mutex_lock(&chip->i2c_lock);
	rc = __nu1669_write(chip, addr, val);
	mutex_unlock(&chip->i2c_lock);

	return rc;
}

static int __nu1669_reg_read(struct oplus_nu1669 *chip, u8 addr, u8 *buf, u8 len)
{
	int rc;

	rc = regmap_bulk_read(chip->rmap, READ_DATA_REG + addr, buf, len);
	if (rc < 0) {
		chg_err("read 0x%04x error, rc=%d\n", addr, rc);
		return rc;
	}

	return 0;
}

static int __nu1669_reg_write(struct oplus_nu1669 *chip, u8 addr, void *val, u8 len)
{
	u8 *pval = (u8 *)val;
	int rc;
	u8 busy_data;
	int i;

	if (len > WRITE_DATA_LENGTH || !val) {
		chg_err("len or data error\n");
		return -EINVAL;
	}

	/*check and set i2c cmd buffer busy*/
	for (i = 0; i < I2C_BUSY_WAIT; i++) {
		if (__nu1669_read(chip, I2C_BUSY_REG, &busy_data) < 0)
			return -EIO;
		if (busy_data == 0) {
			busy_data = 1;
			if (__nu1669_write(chip, I2C_BUSY_REG, busy_data) < 0)
				return -EIO;
			break;
		}
		usleep_range(1000, 1100);
	}
	if (i >= I2C_BUSY_WAIT)
		chg_info("i2c cmd buffer busy[%d]\n", busy_data);

	rc = __nu1669_write(chip, WRITE_DATA_LENGTH_REG, len);
	if (rc < 0) {
		chg_err("write WRITE_DATA_LENGTH_REG error, rc=%d\n", rc);
		return rc;
	}
	rc = regmap_bulk_write(chip->rmap, WRITE_DATA_REG, pval, len);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", WRITE_DATA_REG, rc);
		return rc;
	}
	rc = __nu1669_write(chip, WRITE_DATA_OFFSET_REG, addr);
	if (rc < 0) {
		chg_err("write WRITE_DATA_OFFSET_REG to 0x%04x error, rc=%d\n", addr, rc);
		return rc;
	}

	return 0;
}

static int nu1669_info_obj_read(struct oplus_nu1669 *chip, void *info_reg, u8 reg_len)
{
	u8 offset = 0;
	int rc;

	mutex_lock(&chip->i2c_lock);
	if (!info_reg) {
		chg_err("info_reg NULL\n");
		mutex_unlock(&chip->i2c_lock);
		return -EINVAL;
	}
	if ((u8 *)info_reg < (u8 *)&chip->info || (u8 *)info_reg > (u8 *)&chip->info + sizeof(struct chip_info)) {
		chg_err("info_reg error\n");
		mutex_unlock(&chip->i2c_lock);
		return -EINVAL;
	}

	offset = (u8 *)info_reg - (u8 *)&chip->info;
	rc = __nu1669_reg_read(chip, offset, info_reg, reg_len);
	mutex_unlock(&chip->i2c_lock);

	return rc;
}

static int nu1669_info_obj_write(struct oplus_nu1669 *chip, void *info_reg, u8 reg_len)
{
	u8 offset = 0;
	int rc;

	mutex_lock(&chip->i2c_lock);
	if (!info_reg) {
		chg_err("info_reg NULL\n");
		mutex_unlock(&chip->i2c_lock);
		return -EINVAL;
	}
	if ((u8 *)info_reg < (u8 *)&chip->info || (u8 *)info_reg > (u8 *)&chip->info + sizeof(struct chip_info)) {
		chg_err("info_reg error\n");
		mutex_unlock(&chip->i2c_lock);
		return -EINVAL;
	}

	offset = (u8 *)info_reg - (u8 *)&chip->info;
	rc = __nu1669_reg_write(chip, offset, info_reg, reg_len);
	mutex_unlock(&chip->i2c_lock);

	return rc;
}

#define TRACK_LOCAL_T_NS_TO_S_THD		1000000000
#define TRACK_UPLOAD_COUNT_MAX		10
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD	(24 * 3600)
static int nu1669_track_get_local_time_s(void)
{
	int local_time_s;

	local_time_s = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	chg_info("local_time_s:%d\n", local_time_s);

	return local_time_s;
}

static int nu1669_track_upload_wls_ic_err_info(struct oplus_nu1669 *chip,
	enum wls_err_scene scene_type, enum wls_ic_err_reason reason_type)
{
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;

	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	if (scene_type >= ARRAY_SIZE(wls_err_scene_text) || scene_type < 0) {
		chg_err("wls err scene inval\n");
		return -EINVAL;
	}

	if (reason_type >= ARRAY_SIZE(wls_ic_err_reason_text) || reason_type < 0) {
		chg_err("wls ic err reason inval\n");
		return -EINVAL;
	}

	chg_info("start\n");
	curr_time = nu1669_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_UPLOAD_COUNT_MAX)
		return 0;

	upload_count++;
	pre_upload_time = nu1669_track_get_local_time_s();

	oplus_chg_ic_creat_err_msg(chip->ic_dev, OPLUS_IC_ERR_WLS_RX, 0,
		"$$err_scene@@%s$$err_reason@@%s",
		wls_err_scene_text[scene_type], wls_ic_err_reason_text[reason_type]);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);

	chg_info("success\n");

	return 0;
}

static int nu1669_track_debugfs_init(struct oplus_nu1669 *chip)
{
	int ret = 0;
	struct dentry *debugfs_root;
	struct dentry *debugfs_nu1669;

	debugfs_root = oplus_chg_track_get_debugfs_root();
	if (!debugfs_root) {
		ret = -ENOENT;
		return ret;
	}

	debugfs_nu1669 = debugfs_create_dir("nu1669", debugfs_root);
	if (!debugfs_nu1669) {
		ret = -ENOENT;
		return ret;
	}

	chip->debug_force_ic_err = WLS_IC_ERR_NONE;
	debugfs_create_u32("debug_force_ic_err", 0644, debugfs_nu1669, &(chip->debug_force_ic_err));

	return ret;
}

static int nu1669_set_trx_boost_enable(struct oplus_nu1669 *chip, bool en)
{
	if (chip == NULL) {
		chg_err("oplus_nu1669 is NULL\n");
		return  -ENODEV;
	}
	if (!is_nor_ic_available(chip)) {
		chg_err("nor_ic is NULL\n");
		return -ENODEV;
	}
	return oplus_chg_wls_nor_set_boost_en(chip->nor_ic, en);
}

static int nu1669_set_trx_boost_vol(struct oplus_nu1669 *chip, int vol_mv)
{
	if (chip == NULL) {
		chg_err("oplus_nu1669 is NULL\n");
		return  -ENODEV;
	}
	if (!is_nor_ic_available(chip)) {
		chg_err("nor_ic is NULL\n");
		return -ENODEV;
	}
	return oplus_chg_wls_nor_set_boost_vol(chip->nor_ic, vol_mv);
}

__maybe_unused static int nu1669_set_trx_boost_curr_limit(struct oplus_nu1669 *chip, int curr_ma)
{
	if (chip == NULL) {
		chg_err("oplus_nu1669 is NULL\n");
		return  -ENODEV;
	}
	if (!is_nor_ic_available(chip)) {
		chg_err("nor_ic is NULL\n");
		return -ENODEV;
	}
	return oplus_chg_wls_nor_set_boost_curr_limit(chip->nor_ic, curr_ma);
}

__maybe_unused static int nu1669_get_rx_event_gpio_val(struct oplus_nu1669 *chip)
{
	if (!gpio_is_valid(chip->rx_event_gpio)) {
		chg_err("rx_event_gpio invalid\n");
		return -ENODEV;
	}

	return gpio_get_value(chip->rx_event_gpio);
}

static int nu1669_set_rx_enable_raw(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	mutex_lock(&chip->pinctrl_lock);
	if (IS_ERR_OR_NULL(chip->pinctrl) ||
	    IS_ERR_OR_NULL(chip->rx_en_active) ||
	    IS_ERR_OR_NULL(chip->rx_en_sleep)) {
		chg_err("rx_en pinctrl error\n");
		mutex_unlock(&chip->pinctrl_lock);
		return -ENODEV;
	}

	rc = pinctrl_select_state(chip->pinctrl,
		en ? chip->rx_en_active : chip->rx_en_sleep);
	mutex_unlock(&chip->pinctrl_lock);
	return rc;
}

static int nu1669_set_rx_enable(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	chip->rx_en_status = en;

	rc = nu1669_set_rx_enable_raw(dev, en);
	if (rc < 0)
		chg_err("can't %s rx\n", en ? "enable" : "disable");
	else
		chg_info("set rx %s\n", en ? "enable" : "disable");

	chg_info("vt_sleep: set value:%d, gpio_val:%d, rx_en_status:%d\n",
		 !en, gpio_get_value(chip->rx_en_gpio), chip->rx_en_status);

	return rc;
}

static int nu1669_rx_is_enable(struct oplus_chg_ic_dev *dev, bool *enable)
{
	struct oplus_nu1669 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		*enable = false;
		return 0;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	if (!gpio_is_valid(chip->rx_en_gpio)) {
		chg_err("rx_en_gpio invalid\n");
		*enable = false;
		return 0;
	}
	/*
	 * This is vt_sleep gpio:
	 * when rx_en_gpio is low, RX is enabled;
	 * when rx_en_gpio is high, RX is sleeps;
	 * the "negation" operation to obtain the appropriate value;
	 */
	if (!gpio_get_value(chip->rx_en_gpio))
		*enable = true;
	else
		*enable = false;
	return 0;
}

static int nu1669_disable_standby(struct oplus_nu1669 *chip)
{
	int rc;

	rc = nu1669_write(chip, SP_CTRL0, DISABLE_STANDBY);
	if (rc < 0) {
		chg_err("disable standby mode fail, rc=%d\n", rc);
	}

	return rc;
}

__maybe_unused static int nu1669_get_tj(struct oplus_chg_ic_dev *dev, int *tj)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.tj, sizeof(chip->info.tj));
	if (rc < 0) {
		chg_err("read tj err, rc=%d\n", rc);
		return rc;
	}
	*tj = chip->info.tj;
	/*if (printk_ratelimit())*/
		chg_info("<~WPC~> temp:%d.\n", *tj);

	return 0;
}

static int nu1669_get_vout(struct oplus_chg_ic_dev *dev, int *vout)
{
	struct oplus_nu1669 *chip;
	int rc;
	int temp;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.vout, sizeof(chip->info.vout));
	if (rc < 0) {
		chg_err("read vout err, rc=%d\n", rc);
		return rc;
	}
	*vout = chip->info.vout;
	if (printk_ratelimit()) {
		chg_info("<~WPC~> vout:%d.\n", *vout);
		nu1669_get_tj(dev, &temp);
	}

	return 0;
}

static int nu1669_set_vout(struct oplus_chg_ic_dev *dev, int vout)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->info.vout_set = vout;
	rc = nu1669_info_obj_write(chip, &chip->info.vout_set, sizeof(chip->info.vout_set));
	if (rc < 0) {
		chg_err("set vout err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> set vout:%d.\n", vout);

	return 0;
}

static int nu1669_get_vrect(struct oplus_chg_ic_dev *dev, int *vrect)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.vrect, sizeof(chip->info.vrect));
	if (rc < 0) {
		chg_err("read vrect err, rc=%d\n", rc);
		return rc;
	}
	*vrect = chip->info.vrect;
	if (printk_ratelimit())
		chg_info("<~WPC~> vrect:%d.\n", *vrect);

	return 0;
}

static int nu1669_get_iout(struct oplus_chg_ic_dev *dev, int *iout)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.iout, sizeof(chip->info.iout));
	if (rc < 0) {
		chg_err("read iout err, rc=%d\n", rc);
		return rc;
	}
	*iout = chip->info.iout;
	if (printk_ratelimit())
		chg_info("<~WPC~> iout:%d.\n", *iout);

	return 0;
}

static int nu1669_get_tx_vout(struct oplus_chg_ic_dev *dev, int *vout)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.vout, sizeof(chip->info.vout));
	if (rc < 0) {
		chg_err("read tx vout err, rc=%d\n", rc);
		return rc;
	}
	*vout = chip->info.vout;
	chg_info("<~WPC~> tx vout:%d.\n", *vout);

	return 0;
}

static int nu1669_get_tx_iout(struct oplus_chg_ic_dev *dev, int *iout)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.iout, sizeof(chip->info.iout));
	if (rc < 0) {
		chg_err("read tx iout err, rc=%d\n", rc);
		return rc;
	}
	*iout = chip->info.iout;
	chg_info("<~WPC~> tx iout:%d.\n", *iout);

	return 0;
}

static int nu1669_get_cep_count(struct oplus_chg_ic_dev *dev, int *count)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.cep_cnt, sizeof(chip->info.cep_cnt));
	if (rc < 0) {
		chg_err("read cep cnt, rc=%d\n", rc);
		return rc;
	}
	*count = chip->info.cep_cnt;

	return 0;
}

static int nu1669_get_cep_val(struct oplus_chg_ic_dev *dev, int *val)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.cep_val, sizeof(chip->info.cep_val));
	if (rc < 0) {
		chg_err("read cep err, rc=%d\n", rc);
		return rc;
	}
	*val = (signed char)chip->info.cep_val;

	if (printk_ratelimit())
		chg_info("<~WPC~> cep value:%d\n", *val);

	return rc;
}


static int nu1669_get_work_freq(struct oplus_chg_ic_dev *dev, int *val)
{
	return 0;
}

static int nu1669_get_power_cap(struct oplus_nu1669 *chip)
{
	int rc;
	static char temp = 0;

	if (chip == NULL) {
		chg_err("oplus_nu1669 is NULL\n");
		return 0;
	}

	if (chip->rx_pwr_cap != 0 && temp != 0)
		return chip->rx_pwr_cap;

	rc = nu1669_info_obj_read(chip, &chip->info.tx_power_cap, sizeof(chip->info.tx_power_cap));
	if (rc < 0) {
		chg_err("Couldn't read power cap, rc = %d\n", rc);
		return 0;
	} else {
		temp = chip->info.tx_power_cap;
	}

	if (!chip->support_epp_11w && temp >= NU1669_RX_PWR_15W) {
		chip->rx_pwr_cap = NU1669_RX_PWR_15W;
	} else if (chip->support_epp_11w && temp >= NU1669_RX_PWR_11W) {
		chip->rx_pwr_cap = NU1669_RX_PWR_11W;
	} else if (temp < NU1669_RX_PWR_10W && temp != 0) {
		/*treat <10W as 5W*/
		chip->rx_pwr_cap = NU1669_RX_PWR_5W;
	} else {
		/*default running mode epp 10w*/
		chip->rx_pwr_cap = NU1669_RX_PWR_10W;
	}
	if (chip->adapter_type == 0)
		chip->adapter_type = nu1669_get_running_mode(chip);
	if (chip->adapter_type == NU1669_RX_MODE_EPP)
		chg_info("running mode epp-%d/2w\n", temp);

	return chip->rx_pwr_cap;
}

static int nu1669_get_running_mode(struct oplus_nu1669 *chip)
{
	int rc;
	char temp = 0;

	if (chip == NULL) {
		chg_err("oplus_nu1669 is NULL\n");
		return 0;
	}

	if (chip->adapter_type != 0)
		return chip->adapter_type;

	rc = nu1669_info_obj_read(chip, &chip->info.rpp_type, sizeof(chip->info.rpp_type));
	if (rc < 0) {
		chg_err("Couldn't read rx mode, rc=%d\n", rc);
		return 0;
	} else {
		temp = chip->info.rpp_type;
	}

	if (temp == NU1669_RX_MODE_EPP) {
		chg_info("rx running in EPP!\n");
		chip->adapter_type = NU1669_RX_MODE_EPP;
	} else if (temp == NU1669_RX_MODE_BPP) {
		chg_info("rx running in BPP!\n");
		chip->adapter_type = NU1669_RX_MODE_BPP;
	} else {
		chg_info("rx running in Others!\n");
		chip->adapter_type = 0;
	}
	if (chip->rx_pwr_cap == 0 && chip->adapter_type == NU1669_RX_MODE_EPP)
		chip->rx_pwr_cap = nu1669_get_power_cap(chip);

	return chip->adapter_type;
}

static int nu1669_get_rx_mode(struct oplus_chg_ic_dev *dev, enum oplus_chg_wls_rx_mode *rx_mode)
{
	struct oplus_nu1669 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->adapter_type = nu1669_get_running_mode(chip);
	chip->rx_pwr_cap = nu1669_get_power_cap(chip);
	if (chip->adapter_type == NU1669_RX_MODE_EPP) {
		if (chip->rx_pwr_cap == NU1669_RX_PWR_15W ||
		    chip->rx_pwr_cap == NU1669_RX_PWR_11W)
			*rx_mode = OPLUS_CHG_WLS_RX_MODE_EPP_PLUS;
		else if (chip->rx_pwr_cap == NU1669_RX_PWR_5W)
			*rx_mode = OPLUS_CHG_WLS_RX_MODE_EPP_5W;
		else
			*rx_mode = OPLUS_CHG_WLS_RX_MODE_EPP;
	} else if (chip->adapter_type == NU1669_RX_MODE_BPP) {
		*rx_mode = OPLUS_CHG_WLS_RX_MODE_BPP;
	} else {
		chip->adapter_type = 0;
		*rx_mode = OPLUS_CHG_WLS_RX_MODE_UNKNOWN;
	}
	chg_debug("!!! rx_mode=%d\n", *rx_mode);

	return 0;
}

static int nu1669_set_rx_mode(struct oplus_chg_ic_dev *dev, enum oplus_chg_wls_rx_mode rx_mode)
{
	struct oplus_nu1669 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->rx_mode = rx_mode;
	chg_info("set rx_mode:%d\n", chip->rx_mode);
	if (chip->rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
		schedule_delayed_work(&chip->rx_mode_work, 0);
	} else {
		cancel_delayed_work(&chip->rx_mode_work);
	}

	return 0;
}

static bool nu1669_get_mode_sw_active(struct oplus_nu1669 *chip)
{
	if (!chip) {
		chg_err("oplus_nu1669 chip is null!\n");
		return false;
	}

	if (!gpio_is_valid(chip->mode_sw_gpio)) {
		/*chg_info("mode_sw_gpio not specified\n");*/
		return false;
	}

	return gpio_get_value(chip->mode_sw_gpio);
}

static int nu1669_set_mode_sw_default(struct oplus_nu1669 *chip)
{
	int rc;

	if (!chip) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	mutex_lock(&chip->pinctrl_lock);
	if (IS_ERR_OR_NULL(chip->pinctrl) ||
	    IS_ERR_OR_NULL(chip->mode_sw_active) ||
	    IS_ERR_OR_NULL(chip->mode_sw_sleep)) {
		chg_err("mode_sw pinctrl error\n");
		mutex_unlock(&chip->pinctrl_lock);
		return -ENODEV;
	}

	rc = pinctrl_select_state(chip->pinctrl, chip->mode_sw_sleep);
	mutex_unlock(&chip->pinctrl_lock);
	if (rc < 0)
		chg_err("can't set mode_sw active, rc=%d\n", rc);
	else
		chg_info("set mode_sw default\n");

	chg_info("mode_sw: gpio_val:%d\n", gpio_get_value(chip->mode_sw_gpio));

	return rc;
}

static int nu1669_set_dcdc_enable(struct oplus_chg_ic_dev *dev, bool en)
{
	return 0;
}

static int nu1669_set_tx_enable(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->info.cusr_cmd = (TX_START_PING | chip->info.cusr_cmd);
	rc = nu1669_info_obj_write(chip, &chip->info.cusr_cmd, sizeof(chip->info.cusr_cmd));
	if (rc < 0) {
		chg_err("set tx enable err, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int nu1669_set_tx_start(struct oplus_chg_ic_dev *dev, enum oplus_chg_wls_tx_start_type start)
{
	struct oplus_nu1669 *chip;
	int rc;
	u8 fw_version = 0;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	if (start != OPLUS_CHG_WLS_TX_STOP) {
		nu1669_disable_standby(chip);
		msleep(10);
		if (nu1669_ic_fw_is_valid(chip)) {
			fw_version = nu1669_checksum_fw(chip);
			if (fw_version == INVALID_FW_VERSION1) {
				chg_info("check tx_fw fail\n");
				chip->event_code = WLS_EVENT_FORCE_UPGRADE;
				oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
				nu1669_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_TX, WLS_IC_ERR_VAC_ACDRV);
				return -ENODEV;
			}
		}
		chip->info.cusr_cmd = TX_ENABLE;
	} else {
		chip->info.cusr_cmd = 0;
	}
	rc = nu1669_info_obj_write(chip, &chip->info.cusr_cmd, sizeof(chip->info.cusr_cmd));
	if (rc < 0) {
		chg_err("set tx start err, rc=%d\n", rc);
		return rc;
	}
	if (start)
		chip->tx_status = TX_STATUS_ON;
	chg_info("set tx start %d\n", start);

	return rc;
}

static int nu1669_get_tx_status(struct oplus_chg_ic_dev *dev, u8 *status)
{
	struct oplus_nu1669 *chip;
	u8 trx_state = 0;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	if (chip->tx_status == TX_STATUS_READY)
		trx_state |= WLS_TRX_STATUS_READY;
	if (chip->tx_status == TX_STATUS_PING_DEVICE)
		trx_state |= WLS_TRX_STATUS_DIGITALPING;
	if (chip->tx_status == TX_STATUS_TRANSFER)
		trx_state |= WLS_TRX_STATUS_TRANSFER;
	*status = trx_state;

	return 0;
}

static int nu1669_get_tx_err(struct oplus_chg_ic_dev *dev, u32 *err)
{
	struct oplus_nu1669 *chip;
	u32 trx_err = 0;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	switch (chip->tx_status) {
	case TX_STATUS_ERR_RXAC:
		trx_err |= WLS_TRX_ERR_RXAC;
		break;
	case TX_STATUS_ERR_OCP:
		trx_err |= WLS_TRX_ERR_OCP;
		break;
	case TX_STATUS_ERR_OVP:
		trx_err |= WLS_TRX_ERR_OVP;
		break;
	case TX_STATUS_ERR_LVP:
		trx_err |= WLS_TRX_ERR_LVP;
		break;
	case TX_STATUS_ERR_FOD:
		trx_err |= WLS_TRX_ERR_FOD;
		break;
	case TX_STATUS_ERR_OTP:
		trx_err |= WLS_TRX_ERR_OTP;
		break;
	case TX_STATUS_ERR_CEPTIMEOUT:
		trx_err |= WLS_TRX_ERR_CEPTIMEOUT;
		break;
	case TX_STATUS_ERR_RXEPT:
		trx_err |= WLS_TRX_ERR_RXEPT;
		break;
	case TX_STATUS_ERR_VRECTOVP:
		trx_err |= WLS_TRX_ERR_VRECTOVP;
		break;
	default:
		break;
	}

	*err = trx_err;

	return 0;
}

static int nu1669_get_headroom(struct oplus_chg_ic_dev *dev, int *val)
{
	return 0;
}

static int nu1669_set_headroom(struct oplus_chg_ic_dev *dev, int val)
{
	return 0;
}

static int nu1669_send_match_q(struct oplus_chg_ic_dev *dev, u8 data[])
{
	struct oplus_nu1669 *chip;
	u8 buf[4] = {0x38, 0x48, data[0], data[1]};
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	chip->info.ppp_data[0] = buf[0];
	chip->info.ppp_data[1] = buf[1];
	chip->info.ppp_data[2] = buf[2];
	chip->info.ppp_data[3] = buf[3];
	rc = nu1669_info_obj_write(chip, &chip->info.ppp_data, sizeof(buf));
	if (rc < 0) {
		chg_err("send match q err, rc=%d\n", rc);
		return rc;
	}
	chg_info("q value:0x%x\n", buf[3]);

	return 0;
}

static int nu1669_epp_send_match_q(struct oplus_chg_ic_dev *dev, u8 data[])
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL || data == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(dev);
	chip->info.epp_ref_fq[0] = data[0];
	chip->info.epp_ref_fq[1] = data[1];
	rc = nu1669_info_obj_write(chip, &chip->info.epp_ref_fq, sizeof(chip->info.epp_ref_fq));
	if (rc < 0) {
		chg_err("send epp match q err, rc=%d\n", rc);
		return rc;
	}
	chg_info("epp f value:0x%x, q value:0x%x\n", data[0], data[1]);

	return 0;
}

static int nu1669_set_fod_parm(struct oplus_chg_ic_dev *dev, u8 data[], int len, int mode, int magcvr)
{
	struct oplus_nu1669 *chip;
	u8 fod_parm[NU1669_FOD_PARM_LEN_MAX] = { 0xff };
	int i;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	if (len <= 0 || len > NU1669_FOD_PARM_LEN_MAX) {
		chg_err("data length error\n");
		return -EINVAL;
	}
	if (mode != FOD_BPP_MODE && mode != FOD_EPP_MODE && mode != FOD_FAST_MODE) {
		chg_info("mode: %d not support, return.\n", mode);
		return 0;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->info.rx_fod_len_cmd = len;
	chip->info.rx_fod_wt_cmd = mode;
	chip->info.magcvr_status = !!magcvr;
	memcpy(&fod_parm[0], &data[0], len);
	memcpy(&chip->info.rx_fod_para1, &fod_parm[0], sizeof(chip->info.rx_fod_para1));
	memcpy(&chip->info.rx_fod_para2, &fod_parm[10], sizeof(chip->info.rx_fod_para2));
	memcpy(&chip->info.rx_fod_para3, &fod_parm[20], sizeof(chip->info.rx_fod_para3));
	memcpy(&chip->info.rx_fod_para4, &fod_parm[30], sizeof(chip->info.rx_fod_para4));
	/*set magcvr status*/
	nu1669_info_obj_write(chip, &chip->info.magcvr_status, sizeof(chip->info.magcvr_status));
	/*set fod len*/
	nu1669_info_obj_write(chip, &chip->info.rx_fod_len_cmd, sizeof(chip->info.rx_fod_len_cmd));
	/*set fod data*/
	nu1669_info_obj_write(chip, &chip->info.rx_fod_para1, sizeof(chip->info.rx_fod_para1));
	nu1669_info_obj_write(chip, &chip->info.rx_fod_para2, sizeof(chip->info.rx_fod_para2));
	nu1669_info_obj_write(chip, &chip->info.rx_fod_para3, sizeof(chip->info.rx_fod_para3));
	nu1669_info_obj_write(chip, &chip->info.rx_fod_para4, sizeof(chip->info.rx_fod_para4));
	/*set type mode*/
	nu1669_info_obj_write(chip, &chip->info.rx_fod_wt_cmd, sizeof(chip->info.rx_fod_wt_cmd));

	printk(KERN_CONT "%s: mode: %d, magcvr: %d, fod_parms:", __func__, mode, magcvr);
	for (i = 0; i < len; i++)
		printk(KERN_CONT " 0x%x", fod_parm[i]);
	printk(KERN_CONT ".\n");

	return 0;
}

static int nu1669_send_msg(struct oplus_chg_ic_dev *dev, unsigned char msg[], int len, int raw_data)
{
	struct oplus_nu1669 *chip;
	int rc;
	int i;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	if (len > 5) {
		chg_err("data length error\n");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	if (raw_data) {
		for (i = 0; i < len; i++)
			chip->info.ppp_data[i] = msg[i];
	} else if (msg[0] == WLS_CMD_GET_TX_PWR) {
		chip->info.ppp_data[0] = 0x18;
		chip->info.ppp_data[1] = msg[0];
		chip->info.ppp_data[2] = ~msg[0];
		chip->info.ppp_data[3] = 0xff;
		chip->info.ppp_data[4] = 0x00;
	} else if (msg[0] == WLS_CMD_GET_TX_ID) {
		chip->info.ppp_data[0] = 0x18;
		chip->info.ppp_data[1] = msg[0];
		/*padding 3 bytes*/
		chip->info.ppp_data[2] = 0x00;
		chip->info.ppp_data[3] = 0x00;
		chip->info.ppp_data[4] = 0x00;
	} else {
		chip->info.ppp_data[0] = 0x48;
		chip->info.ppp_data[1] = msg[0];
		chip->info.ppp_data[2] = msg[1];
		chip->info.ppp_data[3] = msg[2];
		chip->info.ppp_data[4] = msg[3];
	}

	rc = nu1669_info_obj_write(chip, &chip->info.ppp_data, len + 1);
	if (rc < 0) {
		chg_err("send msg err, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int nu1669_register_msg_callback(struct oplus_chg_ic_dev *dev, void *data, void (*call_back)(void *, u8[]))
{
	struct oplus_nu1669 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->rx_msg.dev_data = data;
	chip->rx_msg.msg_call_back = call_back;

	return 0;
}

static int nu1669_get_rx_version(struct oplus_chg_ic_dev *dev, u32 *version)
{
	struct oplus_nu1669 *chip;

	*version = 0;
	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return false;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	if (chip == NULL) {
		chg_err("oplus_nu1669 is NULL\n");
		return -ENODEV;
	}

	*version = chip->rx_fw_version;

	return 0;
}

static int nu1669_get_tx_version(struct oplus_chg_ic_dev *dev, u32 *version)
{
	struct oplus_nu1669 *chip;

	*version = 0;
	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return false;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	if (chip == NULL) {
		chg_err("oplus_nu1669 is NULL\n");
		return -ENODEV;
	}

	*version = chip->tx_fw_version;

	return 0;
}

static int nu1669_check_i2c_ok(struct oplus_nu1669 *chip)
{
	u8 read_data = 0;

	nu1669_write(chip, GEN_RW_REG0, I2C_TEST_DATA);
	msleep(10);
	nu1669_read(chip, GEN_RW_REG0, &read_data);

	if (read_data == I2C_TEST_DATA)
		return 0;
	else
		return -EIO;
}

static int write_mtp_prepare(struct oplus_nu1669 *chip, unsigned char *fw_data)
{
	u8 read_data = 0;
	u8 retry = 0;
	int rc;

	chg_info("<FW UPDATE> mtp prepare enter...\n");

	/*enter test mode*/
	rc = nu1669_write(chip, TM_CUST, TM_CUST_RST);
	if (rc < 0) {
		chg_err("write TM_CUST_RST to 0x%04x error, rc=%d\n", TM_CUST, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, TM_CUST, TM_CUST_CODE0);
	if (rc < 0) {
		chg_err("write TM_CUST_CODE0 to 0x%04x error, rc=%d\n", TM_CUST, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, TM_CUST, TM_CUST_CODE1);
	if (rc < 0) {
		chg_err("write TM_CUST_CODE1 to 0x%04x error, rc=%d\n", TM_CUST, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, TM_CUST, TM_CUST_CODE2);
	if (rc < 0) {
		chg_err("write TM_CUST_CODE2 to 0x%04x error, rc=%d\n", TM_CUST, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, TM_CUST, TM_CUST_CODE3);
	if (rc < 0) {
		chg_err("write TM_CUST_CODE3 to 0x%04x error, rc=%d\n", TM_CUST, rc);
		return -EIO;
	}
	/*disable MCU*/
	rc = nu1669_write(chip, TM_GEN_DIG, DISABLE_MCU);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", TM_GEN_DIG, rc);
		return -EIO;
	}
	/*AP get APB access*/
	rc = nu1669_write(chip, SP_CTRL0, GET_APB_AUTH);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", SP_CTRL0, rc);
		return -EIO;
	}
	/*config M2A*/
	rc = nu1669_write(chip, RECT_CTRL0, RECT_CONFIG);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", RECT_CTRL0, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, MLDO_CTRL0, MLDO_CFG);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MLDO_CTRL0, rc);
		return -EIO;
	}
	/*AP get MTP access*/
	rc = nu1669_write(chip, MCU_MTP_CTRL, GET_MTP_AUTH);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MCU_MTP_CTRL, rc);
		return -EIO;
	}
	/*check MTP ulock*/
	do {
		usleep_range(1000, 1100);
		rc = nu1669_read(chip, MTP_LOCK, &read_data);
		if (rc < 0) {
			chg_err("read 0x%04x error, rc=%d\n", MTP_LOCK, rc);
			return -EIO;
		}
		if ((read_data & MTP_LOCK_BITS_MASK) == MTP_UNLOCK_GOOD) {
			break;
		} else if (++retry >= MTP_UNLOCK_TIMES) {
			chg_err("unlock failed!\n");
			return -EACCES;
		}
	} while (1);

	/*select all sector*/
	rc = nu1669_write(chip, MTP_SECTOR, SEL_SECTOR_ALL);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_SECTOR, rc);
		return -EIO;
	}
	/*download 1st word data*/
	rc = nu1669_write(chip, MTP_ADDR_H, 0x00);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_ADDR_H, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, MTP_ADDR_L, 0x00);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_ADDR_L, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, MTP_WDATA0, fw_data[3]);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_WDATA0, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, MTP_WDATA1, fw_data[2]);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_WDATA1, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, MTP_WDATA2, fw_data[1]);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_WDATA2, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, MTP_WDATA3, fw_data[0]);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_WDATA3, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, MTP_CTRL0, SET_PCS);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_CTRL0, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, MTP_CTRL2, SET_WRT_PULSE);
	if (rc < 0) {
		chg_err("set 0x%04x error, rc=%d\n", MTP_CTRL2, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, MTP_CTRL2, CLR_WRT_PULSE);
	if (rc < 0) {
		chg_err("clear 0x%04x error, rc=%d\n", MTP_CTRL2, rc);
		return -EIO;
	}
	/*check 1st word data write success*/
	retry = 0;
	do {
		rc = nu1669_read(chip, MTP_STAT, &read_data);
		if (rc < 0) {
			chg_err("read 0x%04x error, rc=%d\n", MTP_STAT, rc);
			return -EIO;
		}
		if (!(read_data & (1 << MTP_BUSY_POS))) {
			break;
		} else if (++retry > MTP_BUSY_WAIT) {
			chg_err("write busy!!!\n");
			return -EBUSY;
		}
		usleep_range(100, 110);
	} while (1);

	chg_info("<FW UPDATE> mtp prepare OK!\n");

	return 0;
}

static int write_mtp_main(struct oplus_nu1669 *chip, unsigned char *fw_data, int length)
{
	int i = 0;
	int j = 0;
	u8 read_data = 0;
	int rc;

	chg_info("<FW UPDATE> start write %d FW data enter...\n", length);

	/*enter MTP write mode*/
	rc = nu1669_write(chip, MTP_CMD, START_WRITE);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_CMD, rc);
		return -EIO;
	}
	/*download data*/
	for (i = 0; i < length; i += 4, j = 0) {
		rc = nu1669_write(chip, MTP_WDATA0, fw_data[i + 3]);
		if (rc < 0) {
			chg_err("write 0x%04x error, rc=%d\n", MTP_WDATA0, rc);
			return -EIO;
		}
		rc = nu1669_write(chip, MTP_WDATA1, fw_data[i + 2]);
		if (rc < 0) {
			chg_err("write 0x%04x error, rc=%d\n", MTP_WDATA1, rc);
			return -EIO;
		}
		rc = nu1669_write(chip, MTP_WDATA2, fw_data[i + 1]);
		if (rc < 0) {
			chg_err("write 0x%04x error, rc=%d\n", MTP_WDATA2, rc);
			return -EIO;
		}
		rc = nu1669_write(chip, MTP_WDATA3, fw_data[i + 0]);
		if (rc < 0) {
			chg_err("write 0x%04x error, rc=%d\n", MTP_WDATA3, rc);
			return -EIO;
		}
		/*check data write success*/
		do {
			rc = nu1669_read(chip, MTP_STAT, &read_data);
			if (rc < 0) {
				chg_err("read 0x%04x error, rc=%d\n", MTP_STAT, rc);
				return -EIO;
			}
			if (!(read_data & (1 << MTP_BUSY_POS))) {
				break;
			} else if (++j > MTP_BUSY_WAIT) {
				chg_err("write busy!!!\n");
				return -EBUSY;
			}
			usleep_range(100, 110);
		} while (1);
	}

	chg_info("<FW UPDATE> start write %d FW data OK!\n", length);

	return 0;
}

static int write_mtp_done(struct oplus_nu1669 *chip)
{
	int rc;

	chg_info("<FW UPDATE> mtp done enter...\n");

	/*exit MTP write mode*/
	rc = nu1669_write(chip, MTP_CMD, CLOSE_OPT);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_CMD, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, MTP_CTRL0, CLR_PCS);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MTP_CTRL0, rc);
		return -EIO;
	}
	/*release MTP access*/
	rc = nu1669_write(chip, MCU_MTP_CTRL, RELEASE_MTP_AUTH);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", MCU_MTP_CTRL, rc);
		return -EIO;
	}
	/*enable MCU*/
	rc = nu1669_write(chip, TM_GEN_DIG, ENABLE_MCU);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", TM_GEN_DIG, rc);
		return -EIO;
	}
	/*reset IC*/
	rc = nu1669_write(chip, WDOG_LOAD, RST_IC0);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", WDOG_LOAD, rc);
		return -EIO;
	}
	rc = nu1669_write(chip, WDOG_CTRL, RST_IC1);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", WDOG_CTRL, rc);
		return -EIO;
	}
	/*release APB access*/
	rc = nu1669_write(chip, SP_CTRL0, RELEASE_APB_AUTH);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", SP_CTRL0, rc);
		return -EIO;
	}
	/*release MCU and clock*/
	rc = nu1669_write(chip, TM_GEN_DIG, RELEASE_MCU);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", TM_GEN_DIG, rc);
		return -EIO;
	}
	/*exit test mode*/
	rc = nu1669_write(chip, TM_CUST, EXIT_TEST_MODE);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", TM_CUST, rc);
		return -EIO;
	}

	chg_info("<FW UPDATE> mtp done OK!\n");

	return 0;
}

static int nu1669_download_fw(struct oplus_nu1669 *chip, unsigned char *fw, int fw_length)
{
	if (write_mtp_prepare(chip, fw) < 0)
		goto fw_err;
	if (write_mtp_main(chip, fw, fw_length) < 0)
		goto fw_err;
	if (write_mtp_done(chip) < 0)
		goto fw_err;

	chg_info("<FW UPDATE> nu1669_download_fw OK !!!\n");
	return 0;

fw_err:
	chg_info("<FW UPDATE> nu1669_download_fw FAILED !!!\n");
	return -EIO;
}

static void nu1669_req_checksum(struct oplus_nu1669 *chip)
{
	chip->info.fw_check = FW_CHECK_EN;
	(void)nu1669_info_obj_write(chip, &chip->info.fw_check, sizeof(chip->info.fw_check));
}

static u8 nu1669_get_fw_version(struct oplus_nu1669 *chip)
{
	int rc;

	rc = nu1669_info_obj_read(chip, &chip->info.fw_version, sizeof(chip->info.fw_version));
	if (rc < 0) {
		chg_err("read fw_version err, rc=%d\n", rc);
		return 0;
	} else {
		return chip->info.fw_version;
	}
}

static u8 nu1669_checksum_fw(struct oplus_nu1669 *chip)
{
	u8 fw_version = 0;

	/*ap send fw check cmd*/
	nu1669_req_checksum(chip);

	/*wait 100ms for fw checking*/
	msleep(100);

	/*ap read fw version*/
	fw_version = nu1669_get_fw_version(chip);
	chg_info("fw_version=0x%x\n", fw_version);

	return fw_version;
}

static u16 nu1669_get_chip_id(struct oplus_nu1669 *chip)
{
	int rc;

	rc = nu1669_info_obj_read(chip, &chip->info.chip_id, sizeof(chip->info.chip_id));
	if (rc < 0) {
		chg_err("read chip_id err, rc=%d\n", rc);
		return 0;
	} else {
		chg_info("chip_id=0x%x\n", chip->info.chip_id);
		return chip->info.chip_id;
	}
}

static u8 nu1669_get_hw_version(struct oplus_nu1669 *chip)
{
	int rc;

	rc = nu1669_info_obj_read(chip, &chip->info.hw_version, sizeof(chip->info.hw_version));
	if (rc < 0) {
		chg_err("read hw_version err, rc=%d\n", rc);
		return 0;
	} else {
		chg_info("hw_version=0x%x\n", chip->info.hw_version);
		return chip->info.hw_version;
	}
}

static u8 nu1669_get_customer_id(struct oplus_nu1669 *chip)
{
	int rc;

	rc = nu1669_info_obj_read(chip, &chip->info.customer_id, sizeof(chip->info.customer_id));
	if (rc < 0) {
		chg_err("read customer_id err, rc=%d\n", rc);
		return 0;
	} else {
		chg_info("customer_id=0x%x\n", chip->info.customer_id);
		return chip->info.customer_id;
	}
}

static bool nu1669_ic_fw_is_valid(struct oplus_nu1669 *chip)
{
	u16 chip_id = 0;
	u8 hw_version = 0;
	u8 customer_id = 0;

	chip_id = nu1669_get_chip_id(chip);
	hw_version = nu1669_get_hw_version(chip);
	customer_id = nu1669_get_customer_id(chip);
	/*If !(chip_id&&hw_version&&customer_id), the ic maybe empty*/
	if (chip_id == NU1669_CHIP_ID && hw_version == NU1669_HW_VERSION && customer_id == NU1669_CUSTOMER_ID)
		return true;
	return false;
}

static int nu1669_upgrade_firmware(struct oplus_nu1669 *chip, unsigned char *fw_buf, int fw_size)
{
	int rc;
	u8 fw_version = 0;

	if (fw_buf == NULL) {
		chg_err("fw_buf is NULL\n");
		return -EINVAL;
	}

	chg_info("<FW UPDATE> check idt fw update<><><><><><><><>\n");

	disable_irq(chip->rx_con_irq);
	disable_irq(chip->rx_event_irq);

	rc = nu1669_set_trx_boost_vol(chip, NU1669_MTP_VOL_MV);
	if (rc < 0) {
		chg_err("set trx power vol(=%d), rc=%d\n", NU1669_MTP_VOL_MV, rc);
		goto exit_enable_irq;
	}
	rc = nu1669_set_trx_boost_enable(chip, true);
	if (rc < 0) {
		chg_err("enable trx power error, rc=%d\n", rc);
		goto exit_enable_irq;
	}
	msleep(100);

	nu1669_disable_standby(chip);
	msleep(10);

	if (chip->debug_force_ic_err == WLS_IC_ERR_I2C) {
		chg_err("<FW UPDATE> debug i2c error!\n");
		chip->debug_force_ic_err = WLS_IC_ERR_NONE;
		nu1669_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_UPDATE, WLS_IC_ERR_I2C);
	}
	rc = nu1669_check_i2c_ok(chip);
	if (rc < 0) {
		fw_version = 0;
		chg_err("<FW UPDATE> i2c error!\n");
		nu1669_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_UPDATE, WLS_IC_ERR_I2C);
		goto exit_disable_boost;
	} else {
		chg_info("<FW UPDATE> i2c success!\n");
	}

	if (nu1669_ic_fw_is_valid(chip)) {
		fw_version = nu1669_checksum_fw(chip);
		if (fw_version != INVALID_FW_VERSION0 && fw_version != INVALID_FW_VERSION1 &&
		    fw_version == (~fw_buf[fw_size - FW_VERSION_OFFSET] & 0xFF)) {
			chg_info("<FW UPDATE> fw is the same, fw_version[0x%x]!\n", fw_version);
			goto exit_disable_boost;
		}
	}

	rc = nu1669_download_fw(chip, fw_buf, fw_size);
	if (rc < 0) {
		fw_version = 0;
		nu1669_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_UPDATE, WLS_IC_ERR_OTHER);
	} else {
		msleep(30);
		fw_version = nu1669_checksum_fw(chip);
		if (fw_version != INVALID_FW_VERSION0 && fw_version != INVALID_FW_VERSION1) {
			chg_info("<FW UPDATE> check idt fw update ok<><><><><><><><>\n");
		} else {
			fw_version = 0;
			chg_err("<FW UPDATE> check idt fw update fail<><><><><><><><>\n");
			nu1669_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_UPDATE, WLS_IC_ERR_CRC);
		}
	}

	msleep(100);
exit_disable_boost:
	(void)nu1669_set_trx_boost_enable(chip, false);
	msleep(20);
	chip->rx_fw_version = fw_version;
	chip->tx_fw_version = fw_version;

exit_enable_irq:
	enable_irq(chip->rx_con_irq);
	enable_irq(chip->rx_event_irq);

	return rc;
}

static int nu1669_upgrade_firmware_by_buf(struct oplus_chg_ic_dev *dev, unsigned char *fw_buf, int fw_size)
{
	struct oplus_nu1669 *chip;
	int rc;
	int j;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	if (chip == NULL) {
		chg_err("oplus_nu1669 is NULL\n");
		return -ENODEV;
	}
	if (!fw_buf || fw_size < FW_VERSION_OFFSET) {
		chg_err("fw data failed\n");
		return -EINVAL;
	}

	for (j = 0; j < fw_size; j++)
		fw_buf[j] = ~fw_buf[j];
	rc = nu1669_upgrade_firmware(chip, fw_buf, fw_size);

	return rc;
}

static int nu1669_upgrade_firmware_by_img(struct oplus_chg_ic_dev *dev)
{
	int rc;
	struct oplus_nu1669 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	if (chip == NULL) {
		chg_err("oplus_nu1669 is NULL\n");
		return -ENODEV;
	}
	if (!chip->fw_data || chip->fw_length < FW_VERSION_OFFSET) {
		chg_err("fw data failed\n");
		return -EINVAL;
	}
	rc = nu1669_upgrade_firmware(chip, chip->fw_data, chip->fw_length);

	return rc;
}

static void nu1669_tx_event_config(struct oplus_nu1669 *chip, unsigned int status, unsigned int err)
{
	enum wls_ic_err_reason tx_err_reason = WLS_IC_ERR_NONE;

	if (chip == NULL) {
		chg_err("oplus_nu1669 is NULL\n");
		return;
	}

	if (err != 0) {
		switch (err) {
		case NU1669_TX_ERR_RXAC:
			chip->tx_status = TX_STATUS_ERR_RXAC;
			tx_err_reason = WLS_IC_ERR_RXAC;
			break;
		case NU1669_TX_ERR_OCP:
			chip->tx_status = TX_STATUS_ERR_OCP;
			tx_err_reason = WLS_IC_ERR_OCP;
			break;
		case NU1669_TX_ERR_OVP:
			chip->tx_status = TX_STATUS_ERR_OVP;
			tx_err_reason = WLS_IC_ERR_OVP;
			break;
		case NU1669_TX_ERR_LVP:
			chip->tx_status = TX_STATUS_ERR_LVP;
			tx_err_reason = WLS_IC_ERR_LVP;
			break;
		case NU1669_TX_ERR_FOD:
			chip->tx_status = TX_STATUS_ERR_FOD;
			tx_err_reason = WLS_IC_ERR_FOD;
			break;
		case NU1669_TX_ERR_OTP:
			chip->tx_status = TX_STATUS_ERR_OTP;
			tx_err_reason = WLS_IC_ERR_OTP;
			break;
		case NU1669_TX_ERR_CEPTIMEOUT:
			chip->tx_status = TX_STATUS_ERR_CEPTIMEOUT;
			break;
		case NU1669_TX_ERR_RXEPT:
			chip->tx_status = TX_STATUS_ERR_RXEPT;
			tx_err_reason = WLS_IC_ERR_RXEPT;
			break;
		case NU1669_TX_ERR_VRECTOVP:
			chip->tx_status = TX_STATUS_ERR_VRECTOVP;
			tx_err_reason = WLS_IC_ERR_VRECTOVP;
			break;
		default:
			break;
		}
	} else {
		switch (status) {
		case NU1669_TX_STATUS_READY:
			chip->tx_status = TX_STATUS_READY;
			break;
		case NU1669_TX_STATUS_DIGITALPING:
		/*case NU1669_TX_STATUS_ANALOGPING:*/
			chip->tx_status = TX_STATUS_PING_DEVICE;
			break;
		case NU1669_TX_STATUS_TRANSFER:
			chip->tx_status = TX_STATUS_TRANSFER;
			break;
		default:
			break;
		}
	}

	if (chip->debug_force_ic_err) {
		tx_err_reason = chip->debug_force_ic_err;
		chip->debug_force_ic_err = WLS_IC_ERR_NONE;
	}
	if (tx_err_reason != WLS_IC_ERR_NONE)
		nu1669_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_TX, tx_err_reason);

	return;
}

static void nu1669_clear_irq(struct oplus_nu1669 *chip, u32 int_clear)
{
	chg_info("nu1669_clear_irq----------\n");
	chip->info.int_clear = int_clear;
	(void)nu1669_info_obj_write(chip, &chip->info.int_clear, sizeof(chip->info.int_clear));
	chip->info.int_clear = 0;
	return;
}

static void nu1669_event_process(struct oplus_nu1669 *chip)
{
	bool enable = false;
	u32 int_flag = 0;
	u8 val_buf[6] = { 0 };
	int rc;
	enum wls_ic_err_reason rx_err_reason = WLS_IC_ERR_NONE;

	nu1669_rx_is_enable(chip->ic_dev, &enable);
	if (!enable && nu1669_get_wls_type(chip) != OPLUS_CHG_WLS_TRX && !chip->standby_config &&
	    (chip->rx_mode != OPLUS_CHG_WLS_RX_MODE_BPP)) {
		chg_info("RX is sleep or TX is disable, Ignore events\n");
		return;
	}

	if (nu1669_info_obj_read(chip, &chip->info.int_flag, sizeof(chip->info.int_flag)) < 0) {
		chg_err("read int_flag error\n");
		 return;
	} else {
		int_flag = chip->info.int_flag;
		chg_info("int_flag: [0x%x]\n", int_flag);
	}
	if (int_flag == 0 || int_flag == NU1669_ERR_IRQ_VALUE)
		 goto out;

	if (chip->standby_config && (int_flag & NU1669_TX_ERR_RXAC)) {
		if (nu1669_info_obj_read(chip, &chip->info.ac_state, sizeof(chip->info.ac_state)) < 0) {
			chg_err("read ac_state error\n");
			return;
		}
		if (chip->info.ac_state == NU1669_RXAC_STATE_ON)
			chip->event_code = WLS_EVENT_RXAC_ATTACH;
		else
			chip->event_code = WLS_EVENT_RXAC_DETACH;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
	}

	if (nu1669_get_wls_type(chip) == OPLUS_CHG_WLS_TRX) {
		nu1669_tx_event_config(chip, int_flag & NU1669_TX_STATUS_MASK, int_flag & NU1669_TX_ERR_MASK);
		if (int_flag & NU1669_VAC_PRESENT)
			/*trigger only in tx mode*/
			chip->event_code = WLS_EVENT_VAC_PRESENT;
		else
			chip->event_code = WLS_EVENT_TRX_CHECK;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
		goto out;
	} else {
		chip->tx_status = TX_STATUS_OFF;
	}

	if (int_flag & NU1669_LDO_ON) {
		chg_err("LDO is on, connected.\n");
		complete(&chip->ldo_on);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ONLINE);
		chip->connected_ldo_on = true;
	}

	if (chip->rx_connected == true) {
		if (int_flag & NU1669_EPP_NEGO_FAIL) {
			chip->adapter_type = 0;
			chip->event_code = WLS_EVENT_EPP_NEGO_FAIL;
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
		}

		if (int_flag & NU1669_RX_ERR_OCP) {
			rx_err_reason = WLS_IC_ERR_OCP;
			chg_err("rx OCP happen!\n");
		} else if (int_flag & NU1669_RX_ERR_CLAMPOVP) {
			rx_err_reason = WLS_IC_ERR_CLAMPOVP;
			chg_err("rx CLAMPOVP happen!\n");
		} else if (int_flag & NU1669_RX_ERR_HARDOVP) {
			rx_err_reason = WLS_IC_ERR_HARDOVP;
			chg_err("rx HARDOVP happen!\n");
		} else if (int_flag & NU1669_RX_ERR_VOUTOVP) {
			rx_err_reason = WLS_IC_ERR_VOUTOVP;
			chg_err("rx VOUTOVP happen!\n");
		} else if (int_flag & NU1669_RX_ERR_SOFTOTP) {
			rx_err_reason = WLS_IC_ERR_SOFTOTP;
			chg_err("rx SOFTOTP happen!\n");
		} else if (int_flag & NU1669_RX_ERR_OTP) {
			rx_err_reason = WLS_IC_ERR_OTP;
			chg_err("rx OTP happen!\n");
		} else if (int_flag & NU1669_VOUT2V2X_OVP) {
			rx_err_reason = WLS_IC_ERR_VOUT2V2X_OVP;
			chg_err("rx VOUT2V2X_OVP happen!\n");
		} else if (int_flag & NU1669_VOUT2V2X_UVP) {
			rx_err_reason = WLS_IC_ERR_VOUT2V2X_UVP;
			chg_err("rx VOUT2V2X_UVP happen!\n");
		} else if (int_flag & NU1669_V2X_OVP) {
			rx_err_reason = WLS_IC_ERR_V2X_OVP;
			chg_err("rx V2X_OVP happen!\n");
		} else if (int_flag & NU1669_V2X_UCP) {
			rx_err_reason = WLS_IC_ERR_V2X_UCP;
			chg_err("rx V2X_UCP happen!\n");
		}
		if (chip->debug_force_ic_err) {
			rx_err_reason = chip->debug_force_ic_err;
			chip->debug_force_ic_err = WLS_IC_ERR_NONE;
		}
		if (rx_err_reason != WLS_IC_ERR_NONE && rx_err_reason != WLS_IC_ERR_CLAMPOVP)
			nu1669_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_RX, rx_err_reason);
	}

	if (int_flag & NU1669_EPP_CAP) {
		chip->rx_pwr_cap = nu1669_get_power_cap(chip);
		chip->event_code = WLS_EVENT_RX_EPP_CAP;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
	}

	if (int_flag & NU1669_TX_DATA_RCVD) {
		rc = nu1669_info_obj_read(chip, &chip->info.bc_data, sizeof(val_buf));
		if (rc) {
			chg_err("Couldn't read bc_data, rc=%d\n", rc);
		} else {
			memcpy(&val_buf, &chip->info.bc_data, sizeof(val_buf));
			chg_info("Received TX data: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				val_buf[0], val_buf[1], val_buf[2], val_buf[3], val_buf[4], val_buf[5]);
			if (chip->rx_msg.msg_call_back != NULL)
				chip->rx_msg.msg_call_back(chip->rx_msg.dev_data, val_buf);
		}
	}

	if (int_flag & NU1669_EPP_TX_MANU_ID) {
		chip->event_code = WLS_EVENT_EPP_TX_MANU_ID;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
	}


out:
	nu1669_clear_irq(chip, int_flag);

	return;
}

static int nu1669_connect_check(struct oplus_chg_ic_dev *dev)
{
	struct oplus_nu1669 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	schedule_delayed_work(&chip->connect_work, 0);

	return 0;
}

static int nu1669_get_event_code(struct oplus_chg_ic_dev *dev, enum oplus_chg_wls_event_code *code)
{
	struct oplus_nu1669 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	*code = chip->event_code;
	return 0;
}

static int nu1669_get_bridge_mode(struct oplus_chg_ic_dev *dev, int *mode)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.bridge, sizeof(chip->info.bridge));
	if (rc < 0) {
		chg_err("read bridge mode err, rc=%d\n", rc);
		return rc;
	}
	*mode = chip->info.bridge;
	chg_info("<~WPC~> bridge mode:0x%x.\n", *mode);

	return 0;
}

static int nu1669_set_rx_comu(struct oplus_chg_ic_dev *dev, int comu)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->info.comu_set = comu;
	rc = nu1669_info_obj_write(chip, &chip->info.comu_set, sizeof(chip->info.comu_set));
	if (rc < 0) {
		chg_err("set comu err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> set comu:%d\n", comu);

	return 0;
}

static int nu1669_set_insert_disable(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	mutex_lock(&chip->pinctrl_lock);
	if (IS_ERR_OR_NULL(chip->pinctrl) ||
	    IS_ERR_OR_NULL(chip->insert_dis_active) ||
	    IS_ERR_OR_NULL(chip->insert_dis_sleep)) {
		chg_err("insert_dis pinctrl error\n");
		mutex_unlock(&chip->pinctrl_lock);
		return -ENODEV;
	}

	rc = pinctrl_select_state(chip->pinctrl,
		en ? chip->insert_dis_active : chip->insert_dis_sleep);
	mutex_unlock(&chip->pinctrl_lock);
	if (rc < 0)
		chg_err("can't %s insert_dis\n", en ? "enable" : "disable");
	else
		chg_info("set insert_dis %s\n", en ? "enable" : "disable");

	chg_info("force_pd: set value:%d, gpio_val:%d\n", en, gpio_get_value(chip->insert_dis_gpio));

	return rc;
}

static int nu1669_standby_config(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_nu1669 *chip;
	int rc = 0;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	if (en)
		rc = nu1669_disable_standby(chip);
	chip->standby_config = en;

	return rc;
}

static int nu1669_get_tx_id(struct oplus_chg_ic_dev *dev, int *tx_id)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.tx_manu_id, sizeof(chip->info.tx_manu_id));
	if (rc < 0) {
		chg_err("read tx_id err, rc=%d\n", rc);
		return rc;
	}
	*tx_id = chip->info.tx_manu_id;
	chg_info("<~WPC~> tx_id:0x%x.\n", *tx_id);

	return 0;
}

static int nu1669_set_silent(struct oplus_chg_ic_dev *dev)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->info.comu_set = 0x0F;
	rc = nu1669_info_obj_write(chip, &chip->info.comu_set, sizeof(chip->info.comu_set));
	if (rc < 0) {
		chg_err("set silent sleep err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> set silent sleep...\n");

	return 0;
}

static int nu1669_get_ac_ov_flag(struct oplus_chg_ic_dev *dev, int *ac_ov_flag)
{
	if (dev == NULL || ac_ov_flag == NULL) {
		chg_err("oplus_chg_ic_dev or ac_ov_flag is NULL\n");
		return -ENODEV;
	}
	*ac_ov_flag = NU1669_AC_OV_FLAG;

	return 0;
}


static bool nu1669_vac_acdrv_check(struct oplus_nu1669 *chip)
{
	int rc;

	rc = nu1669_info_obj_read(chip, &chip->info.vac_acdrv_state, sizeof(chip->info.vac_acdrv_state));
	if (rc < 0) {
		chg_err("read vac_acdrv_state err, rc=%d\n", rc);
		return true;
	}

	if (chip->info.vac_acdrv_state == VAC_ACDRV_OK) {
		chg_info("check vac_acdrv OK\n");
		return true;
	} else {
		nu1669_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_RX, WLS_IC_ERR_VAC_ACDRV);
		chg_err("check vac_acdrv fail\n");
		return false;
	}
}

static void nu1669_check_ldo_on_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1669 *chip =
		container_of(dwork, struct oplus_nu1669, check_ldo_on_work);
	int iout = 0;
	bool connected = false;

	chg_info("connected_ldo_on = %s\n", chip->connected_ldo_on ? "true" : "false");
	nu1669_rx_is_connected(chip->ic_dev, &connected);
	if ((!chip->connected_ldo_on) && connected) {
		chg_err("Connect but no ldo on event irq, check again.\n");
		nu1669_get_iout(chip->ic_dev, &iout);
		if (iout >= LDO_ON_MA) {
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ONLINE);
			chip->connected_ldo_on = true;
		} else {
			schedule_delayed_work(&chip->check_ldo_on_work, NU1669_CHECK_LDO_ON_DELAY);
		}
	}
}

static void nu1669_rx_mode_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1669 *chip =
		container_of(dwork, struct oplus_nu1669, rx_mode_work);
	int i = 0;

	chg_info("rx_mode = %d, recover rx_enable:%d\n", chip->rx_mode, chip->rx_en_status);
	if (chip->rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
		for (i = 0; i < RX_MODE_BPP_COUNT; i++) {
			nu1669_set_rx_enable_raw(chip->ic_dev, false);
			msleep(30);
			if (chip->rx_mode == OPLUS_CHG_WLS_RX_MODE_UNKNOWN) {
				nu1669_set_rx_enable_raw(chip->ic_dev, chip->rx_en_status);
				chg_info("rx_mode force exit BPP! count:%d\n", i);
				return;
			}
			nu1669_set_rx_enable_raw(chip->ic_dev, true);
			msleep(20);
			if (chip->rx_mode == OPLUS_CHG_WLS_RX_MODE_UNKNOWN) {
				nu1669_set_rx_enable_raw(chip->ic_dev, chip->rx_en_status);
				chg_info("rx_mode force exit BPP! count:%d\n", i);
				return;
			}
		}
	}
	nu1669_set_rx_enable_raw(chip->ic_dev, chip->rx_en_status);
	chg_info("rx_mode:%d, gpio_val:%d, count:%d\n",
		 chip->rx_mode, gpio_get_value(chip->rx_en_gpio), i);
	return;
}

static void nu1669_event_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1669 *chip = container_of(dwork, struct oplus_nu1669, event_work);

	if (!chip->ic_dev->online) {
		chg_info("ic is not online\n");
		return;
	}

	if (nu1669_wait_resume(chip) < 0)
		return;

	if (chip->rx_connected == true || nu1669_get_wls_type(chip) == OPLUS_CHG_WLS_TRX || chip->standby_config)
		nu1669_event_process(chip);
}

static void nu1669_connect_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1669 *chip =
		container_of(dwork, struct oplus_nu1669, connect_work);
	bool connected = false;
	bool pre_connected = false;

	if (!chip->ic_dev->online) {
		chg_info("ic is not online\n");
		return;
	}
	if (nu1669_wait_resume(chip) < 0)
		return;
	nu1669_rx_is_connected(chip->ic_dev, &pre_connected);
retry:
	reinit_completion(&chip->ldo_on);
	(void)wait_for_completion_timeout(&chip->ldo_on, msecs_to_jiffies(50));
	nu1669_rx_is_connected(chip->ic_dev, &connected);
	if (connected != pre_connected) {
		pre_connected = connected;
		chg_err("retry to check connect\n");
		goto retry;
	}
	if (chip->rx_connected != connected)
		chg_err("!!!!! rx_connected[%d] -> con_gpio[%d]\n", chip->rx_connected, connected);
	if (connected && chip->rx_connected == false) {
		if (nu1669_vac_acdrv_check(chip) == false) {
			chip->event_code = WLS_EVENT_FORCE_UPGRADE;
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
		}
		chip->rx_connected = true;
		chip->connected_ldo_on = false;
		if (nu1669_get_mode_sw_active(chip))
			nu1669_set_mode_sw_default(chip);
		cancel_delayed_work_sync(&chip->check_ldo_on_work);
		schedule_delayed_work(&chip->check_ldo_on_work, NU1669_CHECK_LDO_ON_DELAY);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PRESENT);
		if (nu1669_get_running_mode(chip) == NU1669_RX_MODE_EPP) {
			chip->event_code = WLS_EVENT_RX_EPP_CAP;
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
		}
	} else if (!connected) {
		chip->rx_connected = false;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PRESENT);
		chip->adapter_type = 0;
		chip->rx_pwr_cap = 0;
		chip->connected_ldo_on = false;
		chip->event_code = WLS_EVENT_RX_UNKNOWN;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_OFFLINE);
	}
}

static irqreturn_t nu1669_event_handler(int irq, void *dev_id)
{
	struct oplus_nu1669 *chip = dev_id;

	chg_info("!!!event irq\n");
	schedule_delayed_work(&chip->event_work, 0);
	return IRQ_HANDLED;
}

static irqreturn_t nu1669_connect_handler(int irq, void *dev_id)
{
	struct oplus_nu1669 *chip = dev_id;

	chg_info("!!!connect irq\n");
	schedule_delayed_work(&chip->connect_work, 0);
	return IRQ_HANDLED;
}

static int nu1669_gpio_init(struct oplus_nu1669 *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -ENODEV;
	}

	/*event irq gpio*/
	chip->rx_event_gpio = of_get_named_gpio(node, "oplus,rx_event-gpio", 0);
	if (!gpio_is_valid(chip->rx_event_gpio)) {
		chg_err("rx_event_gpio not specified\n");
		return -ENODEV;
	}
	rc = gpio_request(chip->rx_event_gpio, "rx_event-gpio");
	if (rc < 0) {
		chg_err("rx_event_gpio request error, rc=%d\n", rc);
		return rc;
	}
	chip->rx_event_default = pinctrl_lookup_state(chip->pinctrl, "rx_event_default");
	if (IS_ERR_OR_NULL(chip->rx_event_default)) {
		chg_err("get rx_event_default fail\n");
		goto free_event_gpio;
	}
	gpio_direction_input(chip->rx_event_gpio);
	pinctrl_select_state(chip->pinctrl, chip->rx_event_default);
	chip->rx_event_irq = gpio_to_irq(chip->rx_event_gpio);
	rc = devm_request_irq(chip->dev, chip->rx_event_irq, nu1669_event_handler,
			IRQF_TRIGGER_FALLING, "rx_event_irq", chip);
	if (rc < 0) {
		chg_err("rx_event_irq request error, rc=%d\n", rc);
		goto free_event_gpio;
	}
	enable_irq_wake(chip->rx_event_irq);

	/*connect irq gpio*/
	chip->rx_con_gpio = of_get_named_gpio(node, "oplus,rx_con-gpio", 0);
	if (!gpio_is_valid(chip->rx_con_gpio)) {
		chg_err("rx_con_gpio not specified\n");
		goto disable_event_irq;
	}
	rc = gpio_request(chip->rx_con_gpio, "rx_con-gpio");
	if (rc < 0) {
		chg_err("rx_con_gpio request error, rc=%d\n", rc);
		goto disable_event_irq;
	}
	chip->rx_con_default = pinctrl_lookup_state(chip->pinctrl, "rx_con_default");
	if (IS_ERR_OR_NULL(chip->rx_con_default)) {
		chg_err("get idt_con_default fail\n");
		goto free_con_gpio;
	}
	gpio_direction_input(chip->rx_con_gpio);
	pinctrl_select_state(chip->pinctrl, chip->rx_con_default);
	chip->rx_con_irq = gpio_to_irq(chip->rx_con_gpio);
	rc = devm_request_irq(chip->dev, chip->rx_con_irq, nu1669_connect_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"rx_con_irq", chip);
	if (rc < 0) {
		chg_err("rx_con_irq request error, rc=%d\n", rc);
		goto free_con_gpio;
	}
	enable_irq_wake(chip->rx_con_irq);

	/*vt_sleep gpio*/
	chip->rx_en_gpio = of_get_named_gpio(node, "oplus,rx_en-gpio", 0);
	if (!gpio_is_valid(chip->rx_en_gpio)) {
		chg_err("rx_en_gpio not specified\n");
		goto disable_con_irq;
	}
	rc = gpio_request(chip->rx_en_gpio, "rx_en-gpio");
	if (rc < 0) {
		chg_err("rx_en_gpio request error, rc=%d\n", rc);
		goto disable_con_irq;
	}
	chip->rx_en_active = pinctrl_lookup_state(chip->pinctrl, "rx_en_active");
	if (IS_ERR_OR_NULL(chip->rx_en_active)) {
		chg_err("get rx_en_active fail\n");
		goto free_en_gpio;
	}
	chip->rx_en_sleep = pinctrl_lookup_state(chip->pinctrl, "rx_en_sleep");
	if (IS_ERR_OR_NULL(chip->rx_en_sleep)) {
		chg_err("get rx_en_sleep fail\n");
		goto free_en_gpio;
	}
	pinctrl_select_state(chip->pinctrl, chip->rx_en_active);
	chg_info("set rx_en_gpio value:0, gpio_val:%d\n", gpio_get_value(chip->rx_en_gpio));

	/*wls_force_pd gpio*/
	chip->insert_dis_gpio = of_get_named_gpio(node, "oplus,insert_dis-gpio", 0);
	if (!gpio_is_valid(chip->insert_dis_gpio)) {
		chg_err("insert_dis_gpio not specified\n");
		goto free_en_gpio;
	}
	rc = gpio_request(chip->insert_dis_gpio, "insert_dis-gpio");
	if (rc < 0) {
		chg_err("insert_dis_gpio request error, rc=%d\n", rc);
		goto free_en_gpio;
	}
	chip->insert_dis_active = pinctrl_lookup_state(chip->pinctrl, "insert_dis_active");
	if (IS_ERR_OR_NULL(chip->insert_dis_active)) {
		chg_err("get insert_dis_active fail\n");
		goto free_insert_dis_gpio;
	}
	chip->insert_dis_sleep = pinctrl_lookup_state(chip->pinctrl, "insert_dis_sleep");
	if (IS_ERR_OR_NULL(chip->insert_dis_sleep)) {
		chg_err("get insert_dis_sleep fail\n");
		goto free_insert_dis_gpio;
	}
	pinctrl_select_state(chip->pinctrl, chip->insert_dis_sleep);

	/*mode switch */
	chip->mode_sw_gpio = of_get_named_gpio(node, "oplus,mode_sw-gpio", 0);
	if (!gpio_is_valid(chip->mode_sw_gpio)) {
		chg_info("mode_sw_gpio not specified\n");
		goto exit;
	}
	rc = gpio_request(chip->mode_sw_gpio, "mode_sw-gpio");
	if (rc < 0) {
		chg_err("mode_sw_gpio request error, rc=%d\n", rc);
		goto free_insert_dis_gpio;
	}
	chip->mode_sw_active = pinctrl_lookup_state(chip->pinctrl, "mode_sw_active");
	if (IS_ERR_OR_NULL(chip->mode_sw_active)) {
		chg_err("get mode_sw_active fail\n");
		goto free_mode_sw_gpio;
	}
	chip->mode_sw_sleep = pinctrl_lookup_state(chip->pinctrl, "mode_sw_sleep");
	if (IS_ERR_OR_NULL(chip->mode_sw_sleep)) {
		chg_err("get mode_sw_sleep fail\n");
		goto free_mode_sw_gpio;
	}
	pinctrl_select_state(chip->pinctrl, chip->mode_sw_sleep);

exit:
	return 0;

free_mode_sw_gpio:
	if (gpio_is_valid(chip->mode_sw_gpio))
		gpio_free(chip->mode_sw_gpio);
free_insert_dis_gpio:
	if (gpio_is_valid(chip->insert_dis_gpio))
		gpio_free(chip->insert_dis_gpio);
free_en_gpio:
	if (gpio_is_valid(chip->rx_en_gpio))
		gpio_free(chip->rx_en_gpio);
disable_con_irq:
	disable_irq(chip->rx_con_irq);
free_con_gpio:
	if (gpio_is_valid(chip->rx_con_gpio))
		gpio_free(chip->rx_con_gpio);
disable_event_irq:
	disable_irq(chip->rx_event_irq);
free_event_gpio:
	if (gpio_is_valid(chip->rx_event_gpio))
		gpio_free(chip->rx_event_gpio);
	return rc;
}

static void nu1669_shutdown(struct i2c_client *client)
{
	struct oplus_nu1669 *chip = i2c_get_clientdata(client);
	int wait_wpc_disconn_cnt = 0;
	bool is_connected = false;

	disable_irq(chip->rx_con_irq);
	disable_irq(chip->rx_event_irq);

	/*set TX_EN=0 when shutdown*/
	if (nu1669_get_wls_type(chip) == OPLUS_CHG_WLS_TRX)
		nu1669_set_tx_start(chip->ic_dev, OPLUS_CHG_WLS_TX_STOP);

	nu1669_rx_is_connected(chip->ic_dev, &is_connected);
	if (is_connected &&
	    (nu1669_get_wls_type(chip) == OPLUS_CHG_WLS_VOOC ||
	     nu1669_get_wls_type(chip) == OPLUS_CHG_WLS_SVOOC ||
	     nu1669_get_wls_type(chip) == OPLUS_CHG_WLS_PD_65W)) {
		nu1669_set_silent(chip->ic_dev);
		nu1669_set_rx_enable_raw(chip->ic_dev, false);
		msleep(100);
		while (wait_wpc_disconn_cnt < 10) {
			nu1669_rx_is_connected(chip->ic_dev, &is_connected);
			if (!is_connected)
				break;
			msleep(150);
			wait_wpc_disconn_cnt++;
		}
	}
	return;
}

static struct regmap_config nu1669_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

static int nu1669_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (ic_dev->online)
		return 0;
	ic_dev->online = true;
	if (g_nu1669_chip) {
		schedule_delayed_work(&g_nu1669_chip->connect_work, 0);
		schedule_delayed_work(&g_nu1669_chip->event_work, msecs_to_jiffies(500));
	}
	return 0;
}

static int nu1669_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;
	ic_dev->online = false;

	return 0;
}

static int nu1669_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int nu1669_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static void *oplus_chg_rx_get_func(struct oplus_chg_ic_dev *ic_dev,
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
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
			nu1669_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
			nu1669_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
			nu1669_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
			nu1669_smt_test);
		break;
	case OPLUS_IC_FUNC_RX_SET_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_ENABLE,
			nu1669_set_rx_enable);
		break;
	case OPLUS_IC_FUNC_RX_IS_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_IS_ENABLE,
			nu1669_rx_is_enable);
		break;
	case OPLUS_IC_FUNC_RX_IS_CONNECTED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_IS_CONNECTED,
			nu1669_rx_is_connected);
		break;
	case OPLUS_IC_FUNC_RX_GET_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_VOUT,
			nu1669_get_vout);
		break;
	case OPLUS_IC_FUNC_RX_SET_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_VOUT,
			nu1669_set_vout);
		break;
	case OPLUS_IC_FUNC_RX_GET_VRECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_VRECT,
			nu1669_get_vrect);
		break;
	case OPLUS_IC_FUNC_RX_GET_IOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_IOUT,
			nu1669_get_iout);
		break;
	case OPLUS_IC_FUNC_RX_GET_TRX_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TRX_VOL,
			nu1669_get_tx_vout);
		break;
	case OPLUS_IC_FUNC_RX_GET_TRX_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TRX_CURR,
			nu1669_get_tx_iout);
		break;
	case OPLUS_IC_FUNC_RX_GET_CEP_COUNT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_CEP_COUNT,
			nu1669_get_cep_count);
		break;
	case OPLUS_IC_FUNC_RX_GET_CEP_VAL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_CEP_VAL,
			nu1669_get_cep_val);
		break;
	case OPLUS_IC_FUNC_RX_GET_WORK_FREQ:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_WORK_FREQ,
			nu1669_get_work_freq);
		break;
	case OPLUS_IC_FUNC_RX_GET_RX_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_RX_MODE,
			nu1669_get_rx_mode);
		break;
	case OPLUS_IC_FUNC_RX_SET_RX_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_RX_MODE,
			nu1669_set_rx_mode);
		break;
	case OPLUS_IC_FUNC_RX_SET_DCDC_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_DCDC_ENABLE,
			nu1669_set_dcdc_enable);
		break;
	case OPLUS_IC_FUNC_RX_SET_TRX_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_TRX_ENABLE,
			nu1669_set_tx_enable);
		break;
	case OPLUS_IC_FUNC_RX_SET_TRX_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_TRX_START,
			nu1669_set_tx_start);
		break;
	case OPLUS_IC_FUNC_RX_GET_TRX_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TRX_STATUS,
			nu1669_get_tx_status);
		break;
	case OPLUS_IC_FUNC_RX_GET_TRX_ERR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TRX_ERR,
			nu1669_get_tx_err);
		break;
	case OPLUS_IC_FUNC_RX_GET_HEADROOM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_HEADROOM,
			nu1669_get_headroom);
		break;
	case OPLUS_IC_FUNC_RX_SET_HEADROOM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_HEADROOM,
			nu1669_set_headroom);
		break;
	case OPLUS_IC_FUNC_RX_SEND_MATCH_Q:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SEND_MATCH_Q,
			nu1669_send_match_q);
		break;
	case OPLUS_IC_FUNC_RX_SET_FOD_PARM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_FOD_PARM,
			nu1669_set_fod_parm);
		break;
	case OPLUS_IC_FUNC_RX_SEND_MSG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SEND_MSG,
			nu1669_send_msg);
		break;
	case OPLUS_IC_FUNC_RX_REG_MSG_CALLBACK:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_REG_MSG_CALLBACK,
			nu1669_register_msg_callback);
		break;
	case OPLUS_IC_FUNC_RX_GET_RX_VERSION:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_RX_VERSION,
			nu1669_get_rx_version);
		break;
	case OPLUS_IC_FUNC_RX_GET_TRX_VERSION:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TRX_VERSION,
			nu1669_get_tx_version);
		break;
	case OPLUS_IC_FUNC_RX_UPGRADE_FW_BY_BUF:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_UPGRADE_FW_BY_BUF,
			nu1669_upgrade_firmware_by_buf);
		break;
	case OPLUS_IC_FUNC_RX_UPGRADE_FW_BY_IMG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_UPGRADE_FW_BY_IMG,
			nu1669_upgrade_firmware_by_img);
		break;
	case OPLUS_IC_FUNC_RX_CHECK_CONNECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_CHECK_CONNECT,
			nu1669_connect_check);
		break;
	case OPLUS_IC_FUNC_RX_GET_EVENT_CODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_EVENT_CODE,
			nu1669_get_event_code);
		break;
	case OPLUS_IC_FUNC_RX_GET_BRIDGE_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_BRIDGE_MODE,
			nu1669_get_bridge_mode);
		break;
	case OPLUS_IC_FUNC_RX_DIS_INSERT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_DIS_INSERT,
			nu1669_set_insert_disable);
		break;
	case OPLUS_IC_FUNC_RX_STANDBY_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_STANDBY_CONFIG,
			nu1669_standby_config);
		break;
	case OPLUS_IC_FUNC_RX_SET_COMU:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_COMU,
			nu1669_set_rx_comu);
		break;
	case OPLUS_IC_FUNC_RX_GET_TX_ID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TX_ID,
			nu1669_get_tx_id);
		break;
	case OPLUS_IC_FUNC_RX_SET_SILENT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_SILENT,
			nu1669_set_silent);
		break;
	case OPLUS_IC_FUNC_RX_SEND_EPP_MATCH_Q:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SEND_EPP_MATCH_Q,
			nu1669_epp_send_match_q);
		break;
	case OPLUS_IC_FUNC_RX_GET_AC_OV_FLAG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_AC_OV_FLAG,
			nu1669_get_ac_ov_flag);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq nu1669_rx_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_PRESENT },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{ .virq_id = OPLUS_IC_VIRQ_EVENT_CHANGED },
};

static int nu1669_cp_init(struct oplus_chg_ic_dev *ic_dev)
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

static int nu1669_cp_exit(struct oplus_chg_ic_dev *ic_dev)
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

static int nu1669_cp_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int nu1669_cp_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

__maybe_unused static int nu1669_get_b2b_status(struct oplus_chg_ic_dev *dev, u8 *status)
{
	struct oplus_nu1669 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = nu1669_info_obj_read(chip, &chip->info.b2b_state, sizeof(chip->info.b2b_state));
	if (rc < 0) {
		chg_err("read b2b_status err, rc=%d\n", rc);
		return rc;
	}
	*status = chip->info.b2b_state;
	/*chg_info("read b2b_status: [0x%x]\n", *status);*/

	return 0;
}

static int nu1669_set_direct_charge_enable(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_nu1669 *chip;
	int rc;
	u8 b2b_status = 0;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	if (en)
		chip->info.b2b_on = B2B_ON_EN;
	else
		chip->info.b2b_on = 0;
	chg_info("<~WPC~> set b2b_on:%d\n", en);
	rc = nu1669_info_obj_write(chip, &chip->info.b2b_on, sizeof(chip->info.b2b_on));
	if (rc < 0) {
		chg_err("set b2b_on err, rc=%d\n", rc);
		return rc;
	}
	if (en) {
		msleep(50);
		(void)nu1669_get_b2b_status(dev, &b2b_status);
		chg_info("<~WPC~> b2b status:0x%x.\n", b2b_status);
		if ((b2b_status & B2B_ON_EN) != B2B_ON_EN)
			return -EAGAIN;
	}

	return 0;
}

static int nu1669_cp_start_chg(struct oplus_chg_ic_dev *dev, bool start)
{
	return 0;
}

static void *oplus_chg_cp_get_func(struct oplus_chg_ic_dev *dev,
				enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
			nu1669_cp_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
			nu1669_cp_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
			nu1669_cp_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
			nu1669_cp_smt_test);
		break;
	case OPLUS_IC_FUNC_CP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_ENABLE,
			nu1669_set_direct_charge_enable);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START,
			nu1669_cp_start_chg);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq nu1669_cp_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int nu1669_driver_probe(struct i2c_client *client)
#else
static int nu1669_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
#endif
{
	struct oplus_nu1669 *chip;
	struct device_node *node = client->dev.of_node;
	struct device_node *child_node;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int rc = 0;

	chg_info("call !\n");
	chip = devm_kzalloc(&client->dev, sizeof(struct oplus_nu1669), GFP_KERNEL);
	if (!chip) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}

	chip->dev = &client->dev;
	chip->rmap = devm_regmap_init_i2c(client, &nu1669_regmap_config);
	if (!chip->rmap)
		return -ENODEV;
	chip->client = client;
	i2c_set_clientdata(client, chip);

	chip->support_epp_11w = of_property_read_bool(node, "oplus,support_epp_11w");

	chip->fw_data = (char *)of_get_property(node, "oplus,fw_data", &chip->fw_length);
	if (!chip->fw_data) {
		chg_err("parse fw data failed\n");
		chip->fw_data = nu1669_fw_data;
		chip->fw_length = sizeof(nu1669_fw_data);
	} else {
		chg_info("parse fw data length[%d]\n", chip->fw_length);
	}

	for_each_child_of_node(node, child_node) {
		rc = of_property_read_u32(child_node, "oplus,ic_type", &ic_type);
		if (rc < 0) {
			chg_err("can't get %s ic type, rc=%d\n", child_node->name, rc);
			continue;
		}
		rc = of_property_read_u32(child_node, "oplus,ic_index", &ic_index);
		if (rc < 0) {
			chg_err("can't get %s ic index, rc=%d\n", child_node->name, rc);
			continue;
		}
		ic_cfg.name = child_node->name;
		ic_cfg.type = ic_type;
		ic_cfg.index = ic_index;
		ic_cfg.of_node = child_node;
		switch (ic_type) {
		case OPLUS_CHG_IC_RX:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "rx-nu1669");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.virq_data = nu1669_rx_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(nu1669_rx_virq_table);
			ic_cfg.get_func = oplus_chg_rx_get_func;
			break;
		case OPLUS_CHG_IC_CP:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-nu1669");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.virq_data = nu1669_cp_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(nu1669_cp_virq_table);
			ic_cfg.get_func = oplus_chg_cp_get_func;
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_type);
			continue;
		}

		ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
		if (!ic_dev) {
			rc = -ENODEV;
			chg_err("register %s error\n", child_node->name);
			continue;
		}
		chg_info("register %s\n", child_node->name);

		switch (ic_dev->type) {
		case OPLUS_CHG_IC_RX:
			chip->ic_dev = ic_dev;
			chip->ic_dev->type = ic_type;
			break;
		case OPLUS_CHG_IC_CP:
			chip->cp_ic = ic_dev;
			chip->cp_ic->type = ic_type;
			oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_INIT);
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_dev->type);
			continue;
		}
	}

	rc = nu1669_gpio_init(chip);
	if (rc < 0) {
		chg_err("nu1669 gpio init error.\n");
		goto gpio_init_err;
	}

	nu1669_track_debugfs_init(chip);

	device_init_wakeup(chip->dev, true);

	INIT_DELAYED_WORK(&chip->event_work, nu1669_event_work);
	INIT_DELAYED_WORK(&chip->connect_work, nu1669_connect_work);
	INIT_DELAYED_WORK(&chip->check_ldo_on_work, nu1669_check_ldo_on_work);
	INIT_DELAYED_WORK(&chip->rx_mode_work, nu1669_rx_mode_work);
	mutex_init(&chip->i2c_lock);
	mutex_init(&chip->pinctrl_lock);
	init_completion(&chip->ldo_on);
	init_completion(&chip->resume_ack);
	complete_all(&chip->resume_ack);
	g_nu1669_chip = chip;
	chg_info("call end!\n");
	return 0;

gpio_init_err:
	if (chip->ic_dev)
		devm_oplus_chg_ic_unregister(chip->dev, chip->ic_dev);
	if (chip->cp_ic)
		devm_oplus_chg_ic_unregister(chip->dev, chip->cp_ic);
	i2c_set_clientdata(client, NULL);
	return rc;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
static int nu1669_driver_remove(struct i2c_client *client)
#else
static void nu1669_driver_remove(struct i2c_client *client)
#endif
{
	struct oplus_nu1669 *chip = i2c_get_clientdata(client);

	if (gpio_is_valid(chip->rx_en_gpio))
		gpio_free(chip->rx_en_gpio);
	disable_irq(chip->rx_con_irq);
	if (gpio_is_valid(chip->rx_con_gpio))
		gpio_free(chip->rx_con_gpio);
	disable_irq(chip->rx_event_irq);
	if (gpio_is_valid(chip->rx_event_gpio))
		gpio_free(chip->rx_event_gpio);
	if (chip->ic_dev)
		devm_oplus_chg_ic_unregister(chip->dev, chip->ic_dev);
	if (chip->cp_ic)
		devm_oplus_chg_ic_unregister(chip->dev, chip->cp_ic);
	i2c_set_clientdata(client, NULL);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
	return 0;
#else
	return;
#endif
}

static int nu1669_pm_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct oplus_nu1669 *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("oplus_nu1669 is null\n");
		return 0;
	}

	complete_all(&chip->resume_ack);
	return 0;
}

static int nu1669_pm_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct oplus_nu1669 *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("oplus_nu1669 is null\n");
		return 0;
	}

	reinit_completion(&chip->resume_ack);
	return 0;
}

static const struct dev_pm_ops nu1669_pm_ops = {
	.resume = nu1669_pm_resume,
	.suspend = nu1669_pm_suspend,
};

static const struct of_device_id nu1669_match[] = {
	{ .compatible = "oplus,rx-nu1669" },
	{},
};

static const struct i2c_device_id nu1669_id_table[] = {
	{ "oplus,rx-nu1669", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, nu1669_id_table);

static struct i2c_driver nu1669_driver = {
	.driver = {
		.name = "rx-nu1669",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(nu1669_match),
		.pm = &nu1669_pm_ops,
	},
	.probe = nu1669_driver_probe,
	.remove = nu1669_driver_remove,
	.shutdown = nu1669_shutdown,
	.id_table = nu1669_id_table,
};

static __init int nu1669_driver_init(void)
{
	return i2c_add_driver(&nu1669_driver);
}

static __exit void nu1669_driver_exit(void)
{
	i2c_del_driver(&nu1669_driver);
}

oplus_chg_module_register(nu1669_driver);
