// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2024 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[SC96257]([%s][%d]): " fmt, __func__, __LINE__

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
#include <linux/stddef.h>
#include <linux/debugfs.h>
#include <linux/sched/clock.h>
#include <linux/pinctrl/consumer.h>

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_hal_wls.h>
#include <oplus_chg_wls.h>
#include "oplus_hal_sc96257_fw.h"
#include "oplus_hal_sc96257.h"
#include "../monitor/oplus_chg_track.h"

#ifndef I2C_ERR_MAX
#define I2C_ERR_MAX	6
#endif

#define LDO_ON_MA	100
#define SC96257_AC_OV_FLAG	1  /*This IC has an AC_OV issue when used with the 0x0001 in-vehicle unit*/

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
	WLS_IC_ERR_OVP0,
	WLS_IC_ERR_OVP1,
	WLS_IC_ERR_SOFTOTP,
	WLS_IC_ERR_V2X_UCP,
	WLS_IC_ERR_V2X_OVP,
	WLS_IC_ERR_V2X_VV_UVP,
	WLS_IC_ERR_V2X_VV_OVP,
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
	[WLS_IC_ERR_OVP0] = "ovp0",
	[WLS_IC_ERR_OVP1] = "ovp1",
	[WLS_IC_ERR_SOFTOTP] = "softotp",
	[WLS_IC_ERR_V2X_UCP] = "v2xucp",
	[WLS_IC_ERR_V2X_OVP] = "v2xovp",
	[WLS_IC_ERR_V2X_VV_UVP] = "v2xvvuvp",
	[WLS_IC_ERR_V2X_VV_OVP] = "v2xvvovp",
	[WLS_IC_ERR_I2C] = "i2c_err",
	[WLS_IC_ERR_CRC] = "crc_err",
	[WLS_IC_ERR_VAC_ACDRV] = "vac_acdrv_err",
	[WLS_IC_ERR_OTHER] = "other",
};

#define SC96257_CHECK_LDO_ON_DELAY round_jiffies_relative(msecs_to_jiffies(1000))

struct rx_msg_struct {
	void (*msg_call_back)(void *dev_data, u8 data[]);
	void *dev_data;
};

struct cfg_pkt_type {
	u8 header;
	u8 ref_power : 6;
	u8 power_class : 2;
	u8 reserved0;
	u8 count : 3;
	u8 zero_0 : 1;
	u8 ob : 1;
	u8 reserved1 : 1;
	u8 ai : 1;
	u8 zero_1 : 1;
	u8 window_offset : 3;
	u8 window_size : 5;
	u8 dup : 1;
	u8 buffer_size : 3;
	u8 depth : 2;
	u8 polarity : 1;
	u8 neg : 1;
};

struct id_pkt_type {
	u8 header;
	u8 minor_version : 4;
	u8 major_version : 4;
	u16 mfr_code;
	u8 id3 : 7;
	u8 ext : 1;
	u8 id2 : 8;
	u8 id1 : 8;
	u8 id0 : 8;
};

union fsk_cfg {
	u8 value;
	struct {
		u8 depth : 2;
		u8 polarity : 1;
		u8 cycle_cfg : 2;
		u8 reserved : 3;
	} st;
};

union contract_type {/*power transfer contract*/
	u32 value;
	struct {
		u8 rp_header;
		u8 guaranteed_power;/*The value in this field is in units of 0.5 W.*/
		u8 max_power;
		union fsk_cfg fsk;
	} st;
};

struct capability_pkt_type {
	u8 guaranteed_power : 6;
	u8 power_class : 2;
	u8 potential_power : 6;
	u8 reserved0 : 2;
	u8 not_ressens : 1;
	u8 wpid : 1;
	u8 reserved1 : 6;
};

union ask_pkt_type {
	u8 buf[MAX_ASK_SIZE];
	struct {
		u8 header;
		u8 msg[MAX_ASK_SIZE - 1];
	} st;
};

union fsk_pkt_type {
	u8 buf[MAX_FSK_SIZE];
	struct {
		u8 header;
		u8 msg[MAX_FSK_SIZE - 1];
	};
};

struct fod_cfg {
	u8 gain;
	s8 offset;
};

#define ADDR(TYPE, MEMBER) (CUSTOMER_REGISTERS_BASE_ADDR + offsetof(TYPE, MEMBER))

/*DO NOT modify this structure*/
struct rx_cust_type {/*<offset>*/
	/*information setting*/
	/*0x0000*/
	u32 chip_id;
	u32 fw_ver;
	u32 hw_ver;
	u32 git_ver;
	/*0x0010*/
	u16 mfr_code;
	u16 reserved0012;
	u32 reserved0014;
	u32 reserved0018;
	u32 fw_check;
	/*receiver setting*/
	/*0x0020*/
	u16 start_i;
	u16 start_ovp;
	u16 max_i;
	u16 rx_ocp;
	u16 rx_ovp;
	u16 ldo_opp_soft;
	u16 ovp0;
	u16 ovp1;
	/*0x0030*/
	u16 start_vrect;
	u16 max_vrect;
	u16 target_vout;
	u16 reserved0036;
	u16 epp_vout[4];
	/*0x0040*/
	u16 brg_full_v;
	u16 brg_half_v;
	u16 rect_full_i;
	u16 rect_half_i;
	u16 rect_hys_i;
	u16 reserved004a;
	u16 vrect_curve_x1;
	u16 vrect_curve_x2;
	/*0x0050*/
	u16 vrect_curve_y1;
	u16 vrect_curve_y2;
	u16 vrect_adj_step;
	u16 reserved0056;
	u16 cep_interval;
	u16 rpp_interval;
	u32 reserved005c;
	/*0x0060*/
	struct fod_cfg fod[8];
	/*0x0070*/
	struct fod_cfg epp_fod[8];
	/*0x0080*/
	union fsk_cfg fsk_param;
	u8 reserved0081;
	u8 fsk_dm_th;
	u8 reserved0083;
	u8 rx_otp;
	u8 reserved0085[3];
	u16 wake_time;
	u16 reserved008a;
	u8 ask_cfg;
	u8 reserved008d[3];
	/*0x0090*/
	u8 vsys_cfg;
	u8 vsys_cfg_reserved;
	u8 fod_q;
	u8 fod_r;
	u8 reserved0094[12];
	/*0x00A0*/
	u32 tx_setting[24];
	/*0x0100*/
	u32 cmd;
	u32 irq_en;
	u32 irq_flag;
	u32 irq_clr;
	/*0x0110*/
	u32 cep_cnt;
	u32 mode;
	u32 reserved0118;
	u16 wdg_cnt;
	u16 reserved011e;
	/*0x0120*/
	u32 v2x_state;
	u32 random[2];
	u32 reserved012c;
	/*0x0130*/
	union ask_pkt_type ask_packet;
	/*0x0146*/
	u8 reserved0146[10];
	/*0x0150*/
	union fsk_pkt_type fsk_packet;
	u16 reserved015a[3];
	/*0x0160*/
	u16 vout;
	u16 reserved0162;
	u16 iout;
	u16 reserved0166;
	u16 vrect;
	u16 target_vrect;
	u16 rx_ping_v;
	u16 reserved016e;
	/*0x0170*/
	u16 receive_power;
	u16 transmit_power;
	u16 power_freq;
	u16 power_period;
	s16 tdie;
	u16 reserved017a;
	u8 power_duty;
	u8 reserved017d[3];
	/*0x0180*/
	u32 reserved0180[3];
	u8 sr_state;
	u8 full_bridge;
	u16 reserved018e;
	/*0x0190*/
	u8 ept_reason;
	u8 reserved0191[3];
	u32 ept_type;
	struct id_pkt_type  rxid_packet;
	/*0x01A0*/
	u8 sig_strength;
	u8 pch_value;
	u16 reserved01a2;
	struct cfg_pkt_type rxcfg_packet;
	u16 reserved01aa;
	union contract_type neg_req_contract;
	/*0x01B0*/
	union contract_type neg_cur_contract;
	s8 cep_value;
	u8 chs_value;
	u16 rpp_value;
	u32 reserved01b8[2];
	/*0x01C0*/
	u8 reserved01c0[4];
	u16 tx_mfr_code;
	u8 tx_ver;
	u8 afc_ver;
	struct capability_pkt_type capability_packet;
	u8 reserved01cb;
	u8 reserved01cc[3];
	u8 i2c_check;
	/*0x01D0*/
	u8 silent_sleep;
	u8 rsv01d0[3];
	u8 info0[4];
	u8 mag_case;
	u8 cfg0[3];
	u8 rsv01dc[36];
} __attribute__((packed));

/*DO NOT modify this structure*/
struct tx_cust_type {/*<offset>*/
	/*information setting*/
	/*0x0000*/
	u32 chip_id;
	u32 fw_ver;
	u32 hw_ver;
	u32 git_ver;
	/*0x0010*/
	u16 mfr_code;
	u16 reserved0012;
	u32 reserved0014;
	u32 reserved0018;
	u32 fw_check;
	/*0x0020*/
	u32 rx_setting[32];
	/*transmitter setting*/
	/*0x00A0*/
	u16 min_freq;
	u16 max_freq;
	u16 tx_lvp;
	u16 tx_dcm;
	u16 ping_ocp;
	u16 tx_ocp;
	u16 ping_ovp;
	u16 tx_ovp;
	/*0x00B0*/
	u16 ovp0;
	u16 ovp1;
	u8 tx_otp;
	u8 reserved00b5[3];
	u32 reserved00b8;
	u16 ping_interval;
	u16 ping_timeout;
	/*0x00C0*/
	u16 ping_freq;
	u8 ping_duty;
	u8 reserved00c3;
	u32 reserved00c4[3];
	/*0x00D0*/
	u8 bridge_deadtime;
	u8 min_duty;
	u16 reserved00d2;
	u8 bridge_cfg;
	u8 reserved00d5;
	u16 bridge_halfv;
	u16 bridge_fulli;
	u16 bridge_hysi;
	u16 bridge_adj_freq;
	u8 bridge_adj_duty;
	u8 reserved00df;
	/*0x00E0*/
	u8 tx_fod_index;
	u8 tx_fod_cnt;
	u16 reserved00e2;
	u16 fod_powerloss[6];
	/*0x00F0*/
	u32 reserved00f0[4];
	/*0x0100*/
	u32 cmd;
	u32 irq_en;
	u32 irq_flag;
	u32 irq_clr;
	/*0x0110*/
	u32 cep_cnt;
	u32 mode;
	u32 reserved0118;
	u16 wdg_cnt;
	u16 reserved011e;
	/*0x0120*/
	u32 rsv0120;
	u32 random[2];
	u32 reserved012c;
	/*0x0130*/
	union ask_pkt_type ask_packet;
	/*0x0146*/
	u8 reserved0146[10];
	/*0x0150*/
	union fsk_pkt_type fsk_packet;
	u16 reserved015a[3];
	/*0x0160*/
	u16 vin;
	u16 reserved0162;
	u16 iin;
	u16 reserved0166;
	u16 vbrg;
	u16 rsv016a;
	u32 rsv016c;
	/*0x0170*/
	u16 receive_power;
	u16 transmit_power;
	u16 power_freq;
	u16 power_period;
	s16 tdie;
	u16 reserved017a;
	u8 power_duty;
	u8 reserved017d[3];
	/*0x0180*/
	u32 reserved0180[4];
	/*0x0190*/
	u8 ept_reason;
	u8 reserved0191[3];
	u32 ept_type;
	struct id_pkt_type  rxid_packet;
	/*0x01A0*/
	u8 sig_strength;
	u8 pch_value;
	u16 reserved01a2;
	struct cfg_pkt_type rxcfg_packet;
	u16 reserved01aa;
	union contract_type neg_req_contract;
	/*0x01B0*/
	union contract_type neg_cur_contract;
	s8 cep_value;
	u8 chs_value;
	u16 rpp_value;
	u32 reserved01b8[2];
	/*0x01C0*/
	u8 reserved01c0[4];
	u16 tx_mfr_code;
	u8 tx_ver;
	u8 afc_ver;
	u8 reserved01c8[7];
	u8 i2c_check;
	/*0x01D0*/
	u8 rsv01d0[12];
	/*0x1DC*/
	u8 mac_check[4];
	u8 wlspen_id;
	u8 rsv01e1[3];
	/*0x1E4*/
	u8 mac_addr_l[5];
	u8 rsv01e9[3];
	/*0x1EC*/
	u8 mac_addr_h[5];
	u8 rsv01f1[3];
	/*0x1F4*/
	u8 chg_status[2];
	u8 rsv01f6[10];
} __attribute__((packed));

#pragma pack(1)
struct pgm_data {
	u8 state;
	u8 cmd;
	u16 addr;
	u16 len;
	u16 xor;
	u8 msg[PGM_MSG_SIZE];
};

union pgm_pkt_type {
	u8 value[PGM_MSG_SIZE + PGM_HEADER_SIZE];
	struct pgm_data type;
};
#pragma pack()

struct chip_info {
	struct rx_cust_type rx_info;
	struct tx_cust_type tx_info;
};

struct oplus_sc96257 {
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

	struct regmap *regmap;
	struct rx_msg_struct rx_msg;
	struct completion ldo_on;
	struct completion resume_ack;

	atomic_t i2c_err_count;

	u32 debug_force_ic_err;
};

static struct oplus_sc96257 *g_sc96257_chip = NULL;
static int sc96257_get_running_mode(struct oplus_sc96257 *chip);
static int sc96257_get_power_cap(struct oplus_sc96257 *chip);
static int sc96257_set_rx_enable(struct oplus_chg_ic_dev *dev, bool en);
static int sc96257_fw_checksum(struct oplus_sc96257 *chip);

__maybe_unused static bool is_nor_ic_available(struct oplus_sc96257 *chip)
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

__maybe_unused static bool is_err_topic_available(struct oplus_sc96257 *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

static int sc96257_rx_is_connected(struct oplus_chg_ic_dev *dev, bool *connected)
{
	struct oplus_sc96257 *chip;

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

static int sc96257_get_wls_type(struct oplus_sc96257 *chip)
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

static __inline__ void sc96257_i2c_err_inc(struct oplus_sc96257 *chip)
{
	bool connected = false;

	sc96257_rx_is_connected(chip->ic_dev, &connected);
	if (connected && (atomic_inc_return(&chip->i2c_err_count) > I2C_ERR_MAX)) {
		atomic_set(&chip->i2c_err_count, 0);
		chg_err("read i2c error\n");
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
		/*oplus_chg_anon_mod_event(chip->wls_ocm, OPLUS_CHG_EVENT_RX_IIC_ERR);*/
		/*todo, add i2c error callback*/
	}
}

static __inline__ void sc96257_i2c_err_clr(struct oplus_sc96257 *chip)
{
	atomic_set(&chip->i2c_err_count, 0);
}

#define RESUME_TIMEDOUT_MS	1000
static int sc96257_wait_resume(struct oplus_sc96257 *chip)
{
	int rc;

	rc = wait_for_completion_timeout(&chip->resume_ack, msecs_to_jiffies(RESUME_TIMEDOUT_MS));
	if (!rc) {
		chg_err("wait resume timedout\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int __sc96257_read(struct oplus_sc96257 *chip, u16 addr, void *data, u8 len)
{
	int rc;

	rc = regmap_raw_read(chip->regmap, addr, data, len);
	if (rc < 0) {
		chg_err("read 0x%04x error, rc=%d\n", addr, rc);
		return rc;
	}

	return 0;
}

static int __sc96257_write(struct oplus_sc96257 *chip, u16 addr, void *data, u8 len)
{
	int rc;

	rc = regmap_raw_write(chip->regmap, addr, data, len);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", addr, rc);
		return rc;
	}

	return 0;
}

static int __sc96257_read_block(struct oplus_sc96257 *chip, u16 addr, void *data, u16 len)
{
	int rc;

	addr = (((addr & 0xFF) << 8) | ((addr >> 8) & 0xFF));

	rc = regmap_bulk_read(chip->regmap, addr, data, len);
	if (rc < 0) {
		chg_err("read 0x%04x error, rc=%d\n", addr, rc);
		return rc;
	}

	return 0;
}

static int __sc96257_write_block(struct oplus_sc96257 *chip, u16 addr, void *data, u16 len)
{
	int rc;

	addr = (((addr & 0xFF) << 8) | ((addr >> 8) & 0xFF));

	rc = regmap_bulk_write(chip->regmap, addr, data, len);
	if (rc < 0) {
		chg_err("write 0x%04x error, rc=%d\n", addr, rc);
		return rc;
	}

	return 0;
}

__maybe_unused static int sc96257_read_data(struct oplus_sc96257 *chip, u16 addr, void *data, u8 len)
{
	int rc;

	mutex_lock(&chip->i2c_lock);
	rc = __sc96257_read(chip, addr, data, len);
	mutex_unlock(&chip->i2c_lock);

	return rc;
}

__maybe_unused static int sc96257_write_data(struct oplus_sc96257 *chip, u16 addr, void *data, u8 len)
{
	int rc;

	mutex_lock(&chip->i2c_lock);
	rc = __sc96257_write(chip, addr, data, len);
	mutex_unlock(&chip->i2c_lock);

	return rc;
}

static int sc96257_read_block(struct oplus_sc96257 *chip, u16 addr, void *data, u16 len)
{
	int rc;

	mutex_lock(&chip->i2c_lock);
	rc = __sc96257_read_block(chip, addr, data, len);
	mutex_unlock(&chip->i2c_lock);

	return rc;
}

static int sc96257_write_block(struct oplus_sc96257 *chip, u16 addr, void *data, u16 len)
{
	int rc;

	mutex_lock(&chip->i2c_lock);
	rc = __sc96257_write_block(chip, addr, data, len);
	mutex_unlock(&chip->i2c_lock);

	return rc;
}

static int sc96257_read_byte(struct oplus_sc96257 *chip, u16 addr, u8 *data)
{
	int rc;

	mutex_lock(&chip->i2c_lock);
	rc = __sc96257_read_block(chip, addr, data, 1);
	mutex_unlock(&chip->i2c_lock);

	return rc;
}

static int sc96257_write_byte(struct oplus_sc96257 *chip, u16 addr, u8 data)
{
	int rc;

	mutex_lock(&chip->i2c_lock);
	rc = __sc96257_write_block(chip, addr, &data, 1);
	mutex_unlock(&chip->i2c_lock);

	return rc;
}

#define TRACK_LOCAL_T_NS_TO_S_THD		1000000000
#define TRACK_UPLOAD_COUNT_MAX		10
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD	(24 * 3600)
static int sc96257_track_get_local_time_s(void)
{
	int local_time_s;

	local_time_s = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	chg_info("local_time_s:%d\n", local_time_s);

	return local_time_s;
}

static int sc96257_track_upload_wls_ic_err_info(struct oplus_sc96257 *chip,
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
	curr_time = sc96257_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_UPLOAD_COUNT_MAX)
		return 0;

	upload_count++;
	pre_upload_time = sc96257_track_get_local_time_s();

	oplus_chg_ic_creat_err_msg(chip->ic_dev, OPLUS_IC_ERR_WLS_RX, 0,
		"$$err_scene@@%s$$err_reason@@%s",
		wls_err_scene_text[scene_type], wls_ic_err_reason_text[reason_type]);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);

	chg_info("success\n");

	return 0;
}

static int sc96257_track_debugfs_init(struct oplus_sc96257 *chip)
{
	int ret = 0;
	struct dentry *debugfs_root;
	struct dentry *debugfs_sc96257;

	debugfs_root = oplus_chg_track_get_debugfs_root();
	if (!debugfs_root) {
		ret = -ENOENT;
		return ret;
	}

	debugfs_sc96257 = debugfs_create_dir("sc96257", debugfs_root);
	if (!debugfs_sc96257) {
		ret = -ENOENT;
		return ret;
	}

	chip->debug_force_ic_err = WLS_IC_ERR_NONE;
	debugfs_create_u32("debug_force_ic_err", 0644, debugfs_sc96257, &(chip->debug_force_ic_err));

	return ret;
}

static int sc96257_set_trx_boost_enable(struct oplus_sc96257 *chip, bool en)
{
	if (chip == NULL) {
		chg_err("oplus_sc96257 is NULL\n");
		return  -ENODEV;
	}
	if (!is_nor_ic_available(chip)) {
		chg_err("nor_ic is NULL\n");
		return -ENODEV;
	}
	return oplus_chg_wls_nor_set_boost_en(chip->nor_ic, en);
}

static int sc96257_set_trx_boost_vol(struct oplus_sc96257 *chip, int vol_mv)
{
	if (chip == NULL) {
		chg_err("oplus_sc96257 is NULL\n");
		return  -ENODEV;
	}
	if (!is_nor_ic_available(chip)) {
		chg_err("nor_ic is NULL\n");
		return -ENODEV;
	}
	return oplus_chg_wls_nor_set_boost_vol(chip->nor_ic, vol_mv);
}

__maybe_unused static int sc96257_set_trx_boost_curr_limit(struct oplus_sc96257 *chip, int curr_ma)
{
	if (chip == NULL) {
		chg_err("oplus_sc96257 is NULL\n");
		return  -ENODEV;
	}
	if (!is_nor_ic_available(chip)) {
		chg_err("nor_ic is NULL\n");
		return -ENODEV;
	}
	return oplus_chg_wls_nor_set_boost_curr_limit(chip->nor_ic, curr_ma);
}

__maybe_unused static int sc96257_get_rx_event_gpio_val(struct oplus_sc96257 *chip)
{
	if (!gpio_is_valid(chip->rx_event_gpio)) {
		chg_err("rx_event_gpio invalid\n");
		return -ENODEV;
	}

	return gpio_get_value(chip->rx_event_gpio);
}

static int sc96257_set_rx_enable_raw(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_sc96257 *chip;
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

static int sc96257_set_rx_enable(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_sc96257 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	chip->rx_en_status = en;

	rc = sc96257_set_rx_enable_raw(dev, en);
	if (rc < 0)
		chg_err("can't %s rx\n", en ? "enable" : "disable");
	else
		chg_info("set rx %s\n", en ? "enable" : "disable");

	chg_info("vt_sleep: set value:%d, gpio_val:%d, rx_en_status:%d\n",
		 !en, gpio_get_value(chip->rx_en_gpio), chip->rx_en_status);

	return rc;
}

static int sc96257_rx_is_enable(struct oplus_chg_ic_dev *dev, bool *enable)
{
	struct oplus_sc96257 *chip;

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

#define CMD_RETRY_MAX	10
static int sc96257_rx_set_cmd(struct oplus_sc96257 *chip, u32 rx_cmd)
{
	int rc = 0;
	u16 addr;
	int retry_count = 0;
	u32 cmd_value = 0;

	addr = (u16)ADDR(struct rx_cust_type, cmd);
retry:
	rc = sc96257_read_block(chip, addr, &cmd_value, sizeof(chip->info.rx_info.cmd));
	if (rc < 0) {
		chg_err("read cmd err, rc=%d\n", rc);
		return rc;
	}
	if (cmd_value != 0 && retry_count < CMD_RETRY_MAX) {
		/*need 5~10ms delay after the last cmd*/
		usleep_range(2000, 2100);
		retry_count++;
		chg_info("retry cmd, retry_count=%d\n", retry_count);
		goto retry;
	}
	rc = sc96257_write_block(chip, addr, (u8 *)&rx_cmd, sizeof(chip->info.rx_info.cmd));
	if (rc < 0) {
		chg_err("rx set cmd fail, rc=%d\n", rc);
		return rc;
	}
	/*need 5ms delay for data refresh*/
	if (rx_cmd == AP_CMD_REFRESH)
		usleep_range(5000, 5100);

	return 0;
}

static int sc96257_tx_set_cmd(struct oplus_sc96257 *chip, u32 tx_cmd)
{
	int rc;
	u16 addr;
	int retry_count = 0;
	u32 cmd_value = 0;

	addr = (u16)ADDR(struct tx_cust_type, cmd);
retry:
	rc = sc96257_read_block(chip, addr, &cmd_value, sizeof(chip->info.tx_info.cmd));
	if (rc < 0) {
		chg_err("read cmd err, rc=%d\n", rc);
		return rc;
	}
	if (cmd_value != 0 && retry_count < CMD_RETRY_MAX) {
		/*need some delay after the last cmd*/
		usleep_range(2000, 2100);
		retry_count++;
		chg_info("retry cmd, retry_count=%d\n", retry_count);
		goto retry;
	}
	rc = sc96257_write_block(chip, addr, (u8 *)&tx_cmd, sizeof(chip->info.tx_info.cmd));
	if (rc < 0) {
		chg_err("tx set cmd fail, rc=%d\n", rc);
		return rc;
	}
	/*need 5ms delay for data refresh*/
	if (tx_cmd == AP_CMD_REFRESH)
		usleep_range(5000, 5100);

	return 0;
}

static int sc96257_set_wake_pattern(struct oplus_sc96257 *chip)
{
	u8 pattern = WAKE_PATTERN;
	int rc;

	rc = sc96257_write_block(chip, WAKE_REG, &pattern, 1);
	if (rc < 0) {
		chg_err("wake pattern 1 fail, rc=%d\n", rc);
		return rc;
	}
	pattern = 0;
	rc = sc96257_write_block(chip, WAKE_REG, &pattern, 1);
	if (rc < 0) {
		chg_err("wake pattern 0 fail, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

__maybe_unused static int sc96257_get_tdie(struct oplus_chg_ic_dev *dev, int *tdie)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;
	s16 temp;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, tdie);

	/*refresh*/
	rc = sc96257_rx_set_cmd(chip, AP_CMD_REFRESH);
	if (rc < 0) {
		chg_err("set cmd refresh fail\n");
		return rc;
	}

	rc = sc96257_read_block(chip, addr, &temp, sizeof(chip->info.rx_info.tdie));
	if (rc < 0) {
		chg_err("read tdie err, rc=%d\n", rc);
		return rc;
	}
	*tdie = temp / 10;
	/*if (printk_ratelimit())*/
		chg_info("<~WPC~> temp:%d.\n", *tdie);

	return 0;
}


static int sc96257_get_vout(struct oplus_chg_ic_dev *dev, int *vout)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;
	int temp;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, vout);

	/*refresh*/
	rc = sc96257_rx_set_cmd(chip, AP_CMD_REFRESH);
	if (rc < 0) {
		chg_err("set cmd refresh fail\n");
		return rc;
	}

	rc = sc96257_read_block(chip, addr, (u8 *)vout, sizeof(chip->info.rx_info.vout));
	if (rc < 0) {
		chg_err("read vout err, rc=%d\n", rc);
		return rc;
	}
	if (printk_ratelimit()) {
		chg_info("<~WPC~> vout:%d.\n", *vout);
		sc96257_get_tdie(dev, &temp);
	}

	return 0;
}

static int sc96257_set_vout(struct oplus_chg_ic_dev *dev, int vout)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, target_vout);

	rc = sc96257_write_block(chip, addr, (u8 *)&vout, sizeof(chip->info.rx_info.target_vout));
	if (rc < 0) {
		chg_err("set vout err, rc=%d\n", rc);
		return rc;
	}

	rc = sc96257_rx_set_cmd(chip, AP_CMD_RX_VOUT_CHANGE);
	if (rc < 0) {
		chg_err("set cmd vout change fail\n");
		return rc;
	}
	chg_info("<~WPC~> set vout:%d.\n", vout);

	return 0;
}

static int sc96257_get_vrect(struct oplus_chg_ic_dev *dev, int *vrect)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, vrect);

	/*refresh*/
	rc = sc96257_rx_set_cmd(chip, AP_CMD_REFRESH);
	if (rc < 0) {
		chg_err("set cmd refresh fail\n");
		return rc;
	}

	rc = sc96257_read_block(chip, addr, (u8 *)vrect, sizeof(chip->info.rx_info.vrect));
	if (rc < 0) {
		chg_err("read vrect err, rc=%d\n", rc);
		return rc;
	}
	if (printk_ratelimit())
		chg_info("<~WPC~> vrect:%d.\n", *vrect);

	return 0;
}

static int sc96257_get_iout(struct oplus_chg_ic_dev *dev, int *iout)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, iout);

	/*refresh*/
	rc = sc96257_rx_set_cmd(chip, AP_CMD_REFRESH);
	if (rc < 0) {
		chg_err("set cmd refresh fail\n");
		return rc;
	}

	rc = sc96257_read_block(chip, addr, (u8 *)iout, sizeof(chip->info.rx_info.iout));
	if (rc < 0) {
		chg_err("read iout err, rc=%d\n", rc);
		return rc;
	}
	if (printk_ratelimit())
		chg_info("<~WPC~> iout:%d.\n", *iout);

	return 0;
}

static int sc96257_get_tx_vout(struct oplus_chg_ic_dev *dev, int *vout)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct tx_cust_type, vin);

	/*refresh*/
	rc = sc96257_tx_set_cmd(chip, AP_CMD_REFRESH);
	if (rc < 0) {
		chg_err("set cmd refresh fail\n");
		return rc;
	}

	rc = sc96257_read_block(chip, addr, (u8 *)vout, sizeof(chip->info.tx_info.vin));
	if (rc < 0) {
		chg_err("read tx vout err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> tx vout:%d.\n", *vout);

	return 0;
}

static int sc96257_get_tx_iout(struct oplus_chg_ic_dev *dev, int *iout)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct tx_cust_type, iin);

	/*refresh*/
	rc = sc96257_tx_set_cmd(chip, AP_CMD_REFRESH);
	if (rc < 0) {
		chg_err("set cmd refresh fail\n");
		return rc;
	}

	rc = sc96257_read_block(chip, addr, (u8 *)iout, sizeof(chip->info.tx_info.iin));
	if (rc < 0) {
		chg_err("read tx iout err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> tx iout:%d.\n", *iout);

	return 0;
}

static int sc96257_get_cep_count(struct oplus_chg_ic_dev *dev, int *count)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, cep_cnt);

	/*refresh*/
	rc = sc96257_rx_set_cmd(chip, AP_CMD_REFRESH);
	if (rc < 0) {
		chg_err("set cmd refresh fail\n");
		return rc;
	}

	rc = sc96257_read_block(chip, addr, (u8 *)count, sizeof(chip->info.rx_info.cep_cnt));
	if (rc < 0) {
		chg_err("read cep count err, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int sc96257_get_cep_val(struct oplus_chg_ic_dev *dev, int *val)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;
	u8 temp;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, cep_value);

	/*refresh*/
	rc = sc96257_rx_set_cmd(chip, AP_CMD_REFRESH);
	if (rc < 0) {
		chg_err("set cmd refresh fail\n");
		return rc;
	}

	rc = sc96257_read_block(chip, addr, &temp, sizeof(chip->info.rx_info.cep_value));
	if (rc < 0) {
		chg_err("read cep err, rc=%d\n", rc);
		return rc;
	}
	*val = (signed char)temp;

	if (printk_ratelimit())
		chg_info("<~WPC~> cep value:%d\n", *val);

	return 0;
}


static int sc96257_get_work_freq(struct oplus_chg_ic_dev *dev, int *val)
{
	return 0;
}

static int sc96257_get_power_cap(struct oplus_sc96257 *chip)
{
	int rc;
	static u8 temp = 0;
	union contract_type neg_cur;
	u16 addr;

	if (chip == NULL) {
		chg_err("oplus_sc96257 is NULL\n");
		return 0;
	}

	if (chip->rx_pwr_cap != 0 && temp != 0)
		return chip->rx_pwr_cap;

	addr = (u16)ADDR(struct rx_cust_type, neg_cur_contract);
	rc = sc96257_rx_set_cmd(chip, AP_CMD_REFRESH);
	if (rc < 0) {
		chg_err("set cmd refresh fail\n");
		return rc;
	}
	rc = sc96257_read_block(chip, addr, (u8 *)&neg_cur, sizeof(chip->info.rx_info.neg_cur_contract));
	if (rc < 0) {
		chg_err("Couldn't read power cap, rc = %d\n", rc);
		return 0;
	} else {
		temp = neg_cur.st.max_power;
	}

	if (!chip->support_epp_11w && temp >= SC96257_RX_PWR_15W) {
		chip->rx_pwr_cap = SC96257_RX_PWR_15W;
	} else if (chip->support_epp_11w && temp >= SC96257_RX_PWR_11W) {
		chip->rx_pwr_cap = SC96257_RX_PWR_11W;
	} else if (temp < SC96257_RX_PWR_10W && temp != 0) {
		/*treat <10W as 5W*/
		chip->rx_pwr_cap = SC96257_RX_PWR_5W;
	} else {
		/*default running mode epp 10w*/
		chip->rx_pwr_cap = SC96257_RX_PWR_10W;
	}
	if (chip->adapter_type == 0)
		chip->adapter_type = sc96257_get_running_mode(chip);
	if (chip->adapter_type == SC96257_RX_MODE_EPP)
		chg_info("running mode epp-%d/2w\n", temp);

	return chip->rx_pwr_cap;
}

static int sc96257_get_sys_mode(struct oplus_sc96257 *chip, u32 *sys_mode)
{
	int rc;
	u16 addr;

	if (chip == NULL) {
		chg_err("oplus_sc96257 is NULL\n");
		return -ENODEV;
	}

	addr = (u16)ADDR(struct rx_cust_type, mode);
	rc = sc96257_read_block(chip, addr, sys_mode, sizeof(chip->info.rx_info.mode));
	if (rc < 0) {
		chg_err("Couldn't read sys mode, rc=%d\n", rc);
		return rc;
	}
	return 0;
}

static int sc96257_get_running_mode(struct oplus_sc96257 *chip)
{
	int rc;
	u32 temp;

	if (chip == NULL) {
		chg_err("oplus_sc96257 is NULL\n");
		return 0;
	}

	if (!chip->rx_connected) {
		chip->adapter_type = 0;
		return 0;
	}
	if (chip->adapter_type != 0)
		return chip->adapter_type;

	rc = sc96257_get_sys_mode(chip, &temp);
	if (rc < 0) {
		chg_err("Couldn't read sys mode, rc=%d\n", rc);
		return 0;
	}

	if (temp & SC96257_RX_MODE_EPP) {
		chg_info("rx running in EPP!\n");
		chip->adapter_type = SC96257_RX_MODE_EPP;
	} else if (temp & SC96257_RX_MODE_BPP) {
		chg_info("rx running in BPP!\n");
		chip->adapter_type = SC96257_RX_MODE_BPP;
	} else {
		chg_info("rx running in Others!\n");
		chip->adapter_type = 0;
	}
	if (chip->rx_pwr_cap == 0 && chip->adapter_type == SC96257_RX_MODE_EPP)
		chip->rx_pwr_cap = sc96257_get_power_cap(chip);

	return chip->adapter_type;
}

static int sc96257_get_rx_mode(struct oplus_chg_ic_dev *dev, enum oplus_chg_wls_rx_mode *rx_mode)
{
	struct oplus_sc96257 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->adapter_type = sc96257_get_running_mode(chip);
	chip->rx_pwr_cap = sc96257_get_power_cap(chip);
	if (chip->adapter_type == SC96257_RX_MODE_EPP) {
		if (chip->rx_pwr_cap == SC96257_RX_PWR_15W ||
		    chip->rx_pwr_cap == SC96257_RX_PWR_11W)
			*rx_mode = OPLUS_CHG_WLS_RX_MODE_EPP_PLUS;
		else if (chip->rx_pwr_cap == SC96257_RX_PWR_5W)
			*rx_mode = OPLUS_CHG_WLS_RX_MODE_EPP_5W;
		else
			*rx_mode = OPLUS_CHG_WLS_RX_MODE_EPP;
	} else if (chip->adapter_type == SC96257_RX_MODE_BPP) {
		*rx_mode = OPLUS_CHG_WLS_RX_MODE_BPP;
	} else {
		chip->adapter_type = 0;
		*rx_mode = OPLUS_CHG_WLS_RX_MODE_UNKNOWN;
	}
	chg_debug("!!! rx_mode=%d\n", *rx_mode);

	return 0;
}

static int sc96257_set_rx_mode(struct oplus_chg_ic_dev *dev, enum oplus_chg_wls_rx_mode rx_mode)
{
	struct oplus_sc96257 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->rx_mode = rx_mode;
	chg_info("set rx_mode:%d\n", chip->rx_mode);
	if (chip->rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP)
		schedule_delayed_work(&chip->rx_mode_work, 0);
	else
		cancel_delayed_work(&chip->rx_mode_work);

	return 0;
}

static bool sc96257_get_mode_sw_active(struct oplus_sc96257 *chip)
{
	if (!chip) {
		chg_err("oplus_sc96257 chip is null!\n");
		return false;
	}

	if (!gpio_is_valid(chip->mode_sw_gpio)) {
		/*chg_info("mode_sw_gpio not specified\n");*/
		return false;
	}

	return gpio_get_value(chip->mode_sw_gpio);
}

static int sc96257_set_mode_sw_default(struct oplus_sc96257 *chip)
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

static int sc96257_set_dcdc_enable(struct oplus_chg_ic_dev *dev, bool en)
{
	return 0;
}

static int sc96257_set_tx_enable(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_sc96257 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	rc = sc96257_tx_set_cmd(chip, AP_CMD_TX_START_PING);
	if (rc < 0) {
		chg_err("set tx enable err, rc=%d\n", rc);
		return rc;
	} else {
		chg_info("set tx enable ok\n");
	}

	return rc;
}

static int sc96257_set_tx_start(struct oplus_chg_ic_dev *dev, enum oplus_chg_wls_tx_start_type start)
{
	struct oplus_sc96257 *chip;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	if (start != OPLUS_CHG_WLS_TX_STOP) {
		sc96257_set_wake_pattern(chip);
		msleep(10);
		if (sc96257_fw_checksum(chip) == FW_CHECKSUM_FAIL) {
			chg_info("check tx_fw fail\n");
			chip->event_code = WLS_EVENT_FORCE_UPGRADE;
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
			sc96257_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_TX, WLS_IC_ERR_VAC_ACDRV);
			return -ENODEV;
		}
		if (start == OPLUS_CHG_WLS_TX_START_WLSPEN)
			rc = sc96257_tx_set_cmd(chip, AP_CMD_TX_ENABLE | AP_CMD_TX_ENABLE_WLSPEN);
		else
			rc = sc96257_tx_set_cmd(chip, AP_CMD_TX_ENABLE);
	} else {
		rc = sc96257_tx_set_cmd(chip, AP_CMD_TX_DISABLE);
	}
	if (rc < 0) {
		chg_err("set tx start err, rc=%d\n", rc);
		return rc;
	}
	if (start != OPLUS_CHG_WLS_TX_STOP)
		chip->tx_status = TX_STATUS_ON;
	chg_info("set tx start %d\n", start);

	return rc;
}

static int sc96257_get_tx_status(struct oplus_chg_ic_dev *dev, u8 *status)
{
	struct oplus_sc96257 *chip;
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

static int sc96257_get_tx_err(struct oplus_chg_ic_dev *dev, u32 *err)
{
	struct oplus_sc96257 *chip;
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

static int sc96257_get_headroom(struct oplus_chg_ic_dev *dev, int *val)
{
	return 0;
}

static int sc96257_set_headroom(struct oplus_chg_ic_dev *dev, int val)
{
	return 0;
}

static int sc96257_epp_send_match_q(struct oplus_chg_ic_dev *dev, u8 data[])
{
	struct oplus_sc96257 *chip;
	int rc;
	u8 fod_q;
	u8 fod_f;

	if (dev == NULL || data == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	fod_f = data[0];
	fod_q = data[1];

	rc = sc96257_write_block(chip, (u16)ADDR(struct rx_cust_type, fod_q), (u8 *)&fod_q,
		sizeof(chip->info.rx_info.fod_q));
	if (rc < 0) {
		chg_err("set epp fod_q err, rc=%d\n", rc);
		return rc;
	}

	sc96257_write_block(chip, (u16)ADDR(struct rx_cust_type, fod_r), (u8 *)&fod_f,
		sizeof(chip->info.rx_info.fod_r));
	if (rc < 0) {
		chg_err("send epp fod_f err, rc=%d\n", rc);
		return rc;
	}
	chg_info("epp f value:0x%x, q value:0x%x\n", data[0], data[1]);

	return 0;
}

#define PPP_BUSY_WAIT	30
static int sc96257_send_match_q(struct oplus_chg_ic_dev *dev, u8 data[])
{
	struct oplus_sc96257 *chip;
	union ask_pkt_type ask_packet;
	u8 buf[4] = { 0x38, 0x48, data[0], data[1]};
	int rc;
	u16 addr;
	u32 mode_value;
	int i;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	/*check ppp busy*/
	addr = (u16)ADDR(struct rx_cust_type, mode);
	for (i = 0; i < PPP_BUSY_WAIT; i++) {
		if (sc96257_read_block(chip, addr, &mode_value, sizeof(chip->info.rx_info.mode)) < 0)
			return -EIO;
		if ((mode_value & PPP_BUSY_MODE) == 0)
			break;
		usleep_range(1000, 1100);
	}
	if (i >= PPP_BUSY_WAIT)
		chg_info("ppp busy[0x%x]\n", mode_value);


	ask_packet.st.header = buf[0];
	ask_packet.st.msg[0] = buf[1];
	ask_packet.st.msg[1] = buf[2];
	ask_packet.st.msg[2] = buf[3];
	addr = (u16)ADDR(struct rx_cust_type, ask_packet);
	rc = sc96257_write_block(chip, addr, (u8 *)&(ask_packet.buf), sizeof(buf));
	if (rc < 0) {
		chg_err("send match q err, rc=%d\n", rc);
		return rc;
	}
	rc = sc96257_rx_set_cmd(chip, AP_CMD_SEND_PPP);
	if (rc < 0) {
		chg_err("sc96257 set cmd fail\n");
		return rc;
	}
	chg_info("q value:0x%x, f value:0x%x\n", buf[3], buf[2]);

	return 0;
}

static int sc96257_set_fod_parm(struct oplus_chg_ic_dev *dev, u8 data[], int len, int mode, int magcvr)
{
	struct oplus_sc96257 *chip;
	u8 fod_parm[SC96257_FOD_PARM_LEN_MAX] = { 0xff };
	int i;
	u16 addr;
	int reg_len;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	if (len <= 0 || len > SC96257_FOD_PARM_LEN_MAX) {
		chg_err("data length error\n");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	switch (mode) {
	case FOD_BPP_MODE:
		addr = (u16)ADDR(struct rx_cust_type, fod);
		reg_len = sizeof(chip->info.rx_info.fod);
		break;
	case FOD_EPP_MODE:
		addr = (u16)ADDR(struct rx_cust_type, epp_fod);
		reg_len = sizeof(chip->info.rx_info.epp_fod);
		break;
	case FOD_FAST_MODE:
		addr = (u16)ADDR(struct rx_cust_type, fod);
		reg_len = sizeof(chip->info.rx_info.fod) + sizeof(chip->info.rx_info.epp_fod);
		break;
	default:
		chg_info("mode: %d not support, return.\n", mode);
		return 0;
	}

	memcpy(&fod_parm[0], &data[0], len);
	/*set magcvr status*/
	sc96257_write_block(chip, (u16)ADDR(struct rx_cust_type, mag_case), (u8 *)&magcvr,
		sizeof(chip->info.rx_info.mag_case));
	/*set fod data*/
	sc96257_write_block(chip, addr, &fod_parm[0], reg_len);
	if (mode == FOD_FAST_MODE)
		sc96257_rx_set_cmd(chip, AP_CMD_RX_FOD_ENABLE);

	printk(KERN_CONT "%s: mode: %d, magcvr: %d, fod_parms:", __func__, mode, magcvr);
	for (i = 0; i < len; i++)
		printk(KERN_CONT " 0x%x", fod_parm[i]);
	printk(KERN_CONT ".\n");

	return 0;
}

static int sc96257_send_msg(struct oplus_chg_ic_dev *dev, unsigned char msg[], int len, int raw_data)
{
	struct oplus_sc96257 *chip;
	union ask_pkt_type ask_packet;
	int rc;
	u16 addr;
	u32 mode_value;
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

	/*check ppp busy*/
	addr = (u16)ADDR(struct rx_cust_type, mode);
	for (i = 0; i < PPP_BUSY_WAIT; i++) {
		if (sc96257_read_block(chip, addr, &mode_value, sizeof(chip->info.rx_info.mode)) < 0)
			return -EIO;
		if ((mode_value & PPP_BUSY_MODE) == 0)
			break;
		usleep_range(1000, 1100);
	}
	if (i >= PPP_BUSY_WAIT)
		chg_info("ppp busy[0x%x]\n", mode_value);

	if (raw_data) {
		ask_packet.st.header = msg[0];
		for (i = 0; i < len; i++)
			ask_packet.st.msg[i] = msg[i + 1];
	} else if (msg[0] == WLS_CMD_GET_TX_PWR) {
		ask_packet.st.header = 0x18;
		ask_packet.st.msg[0] = msg[0];
		ask_packet.st.msg[1] = ~msg[0];
		ask_packet.st.msg[2] = 0xff;
		ask_packet.st.msg[3] = 0x00;
	} else if (msg[0] == WLS_CMD_GET_TX_ID) {
		ask_packet.st.header = 0x18;
		ask_packet.st.msg[0] = msg[0];
		/*padding 3 bytes*/
		ask_packet.st.msg[1] = 0x00;
		ask_packet.st.msg[2] = 0x00;
		ask_packet.st.msg[3] = 0x00;
	} else {
		ask_packet.st.header = 0x48;
		ask_packet.st.msg[0] = msg[0];
		ask_packet.st.msg[1] = msg[1];
		ask_packet.st.msg[2] = msg[2];
		ask_packet.st.msg[3] = msg[3];
	}

	addr = (u16)ADDR(struct rx_cust_type, ask_packet);
	rc = sc96257_write_block(chip, addr, (u8 *)&(ask_packet.buf), len + 1);
	if (rc) {
		chg_err("send msg err, rc=%d\n", rc);
		return rc;
	}

	rc = sc96257_rx_set_cmd(chip, AP_CMD_SEND_PPP);
	if (rc < 0) {
		chg_err("sc96257 set cmd fail\n");
		return rc;
	}

	return 0;
}

static int sc96257_register_msg_callback(struct oplus_chg_ic_dev *dev, void *data, void (*call_back)(void *, u8[]))
{
	struct oplus_sc96257 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chip->rx_msg.dev_data = data;
	chip->rx_msg.msg_call_back = call_back;

	return 0;
}

static int sc96257_get_rx_version(struct oplus_chg_ic_dev *dev, u32 *version)
{
	struct oplus_sc96257 *chip;

	*version = 0;
	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return false;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	if (chip == NULL) {
		chg_err("oplus_sc96257 is NULL\n");
		return -ENODEV;
	}

	*version = chip->rx_fw_version;

	return 0;
}

static int sc96257_get_tx_version(struct oplus_chg_ic_dev *dev, u32 *version)
{
	struct oplus_sc96257 *chip;

	*version = 0;
	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return false;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	if (chip == NULL) {
		chg_err("oplus_sc96257 is NULL\n");
		return -ENODEV;
	}

	*version = chip->tx_fw_version;

	return 0;
}

static u32 sc96257_func_crc32(u32 data, u32 crc_init)
{
	u32 crc_poly = CRC_POLY;
	int i;


	for (i = 0; i < 32; i++) {
		crc_init = (crc_init << 1) ^ ((((crc_init >> 31) & 0x01) ^ ((data >> i) & 0x01)) ==
			0x01 ? 0xFFFFFFFF & crc_poly : 0x00000000 & crc_poly);
	}

	return (u32)crc_init;
}

static u32 endian_conversion(u32 value)
{
	return (u32)(((value & 0xFF) << 24) | ((value & 0xFF00) << 8) | ((value & 0xFF0000) >> 8) | ((value & 0xFF000000) >> 24));
}

static u16 pgm_xor(u8 *buffer, u16 len)
{
	u16 val = 0x00;
	int i;

	for (i = 0; i < len; i = i + 2) {
		val ^= (u16)buffer[i] | (((u16)buffer[i + 1]) << 8);
	}

	return (val & 0xffff);
}

#define MAX_COUNT	1000
static uint8_t pgm_state(struct oplus_sc96257 *chip)
{
	int i;
	u8 state;
	int rc;

	for (i = 0; i < MAX_COUNT; i++) {
		usleep_range(1000, 1100);
		rc = sc96257_read_byte(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, &state);
		if (rc < 0) {
			chg_err("read pgm_state failed\n");
			return rc;
		}
		if ((state != PGM_STATE_BUSY) && (state != PGM_STATE_NONE))
			return state;
	}

	return PGM_STATE_TIMEOUT;
}

__maybe_unused static int pgm_sector_erase(struct oplus_sc96257 *chip, u32 addr, u16 length)
{
	u8 state;
	union pgm_pkt_type pgm = { 0 };
	int rc;

	pgm.type.addr = (u16)(addr / PGM_WORD);
	pgm.type.len = length / PGM_WORD;
	pgm.type.xor = pgm_xor(&pgm.value[PGM_XOR_INDEX], PGM_XOR_LEN);
	pgm.type.xor ^= PGM_CMD_SECTOR_ERASE << 8;
	rc = sc96257_write_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, &pgm.value[0], PGM_HEADER_SIZE);
	rc |= sc96257_write_byte(chip, PGM_SRAM_BASE + PGM_CMD_ADDR, PGM_CMD_SECTOR_ERASE);
	if (rc < 0) {
		chg_err("pgm_sector_erase failed\n");
		return rc;
	}

	state = pgm_state(chip);
	if (state != PGM_STATE_DONE) {
		chg_err("Error: pgm sector erase state 0x%02x\n", state);
		return -EINVAL;
	}

	return 0;
}

static int pgm_chip_erase(struct oplus_sc96257 *chip)
{
	u8 state;
	union pgm_pkt_type pgm = { 0 };
	int rc;

	pgm.type.xor = pgm_xor(&pgm.value[PGM_XOR_INDEX], PGM_XOR_LEN);
	pgm.type.xor ^= PGM_CMD_CHIP_ERASE << 8;
	rc = sc96257_write_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, &pgm.value[0], PGM_HEADER_SIZE);
	rc |= sc96257_write_byte(chip, PGM_SRAM_BASE + PGM_CMD_ADDR, PGM_CMD_CHIP_ERASE);
	if (rc < 0) {
		chg_err("pgm_chip_erase failed\n");
		return rc;
	}

	state = pgm_state(chip);
	if (state != PGM_STATE_DONE) {
		chg_err("Error: pgm chip erase state 0x%02x\n", state);
		return -EINVAL;
	}

	return 0;
}

__maybe_unused static int pgm_trim_erase(struct oplus_sc96257 *chip)
{
	u8 state;
	union pgm_pkt_type pgm = { 0 };
	int rc;

	pgm.type.xor = pgm_xor(&pgm.value[PGM_XOR_INDEX], PGM_XOR_LEN);
	pgm.type.xor ^= PGM_CMD_TRIM_ERASE << 8;
	rc = sc96257_write_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, &pgm.value[0], PGM_HEADER_SIZE);
	rc |= sc96257_write_byte(chip, PGM_SRAM_BASE + PGM_CMD_ADDR, PGM_CMD_TRIM_ERASE);
	if (rc < 0) {
		chg_err("pgm_trim_erase failed\n");
		return rc;
	}

	state = pgm_state(chip);
	if (state != PGM_STATE_DONE) {
		chg_err("Error: pgm trim erase state 0x%02x\n", state);
		return -EINVAL;
	}

	return 0;
}

__maybe_unused static int pgm_cust_erase(struct oplus_sc96257 *chip)
{
	u8 state;
	union pgm_pkt_type pgm = { 0 };
	int rc;

	pgm.type.xor = pgm_xor(&pgm.value[PGM_XOR_INDEX], PGM_XOR_LEN);
	pgm.type.xor ^= PGM_CMD_CUST_ERASE << 8;
	rc = sc96257_write_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, &pgm.value[0], PGM_HEADER_SIZE);
	rc |= sc96257_write_byte(chip, PGM_SRAM_BASE + PGM_CMD_ADDR, PGM_CMD_CUST_ERASE);
	if (rc < 0) {
		chg_err("pgm_cust_erase failed\n");
		return rc;
	}

	state = pgm_state(chip);
	if (state != PGM_STATE_DONE) {
		chg_err("Error: pgm cut erase state 0x%02x\n", state);
		return -EINVAL;
	}

	return 0;
}

static int pgm_set_margin(struct oplus_sc96257 *chip, u8 buffer)
{
	u8 state;
	union pgm_pkt_type pgm = { 0 };
	int rc;

	pgm.type.addr = 0x00;
	pgm.type.len = 1;
	memcpy(pgm.type.msg, &buffer, 1);

	pgm.type.xor = pgm_xor(&pgm.value[PGM_XOR_INDEX], PGM_XOR_LEN + 4);
	pgm.type.xor ^= PGM_CMD_MARGIN << 8;

	rc = sc96257_write_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, &pgm.value[0], 4 + PGM_HEADER_SIZE);
	rc |= sc96257_write_byte(chip, PGM_SRAM_BASE + PGM_CMD_ADDR, PGM_CMD_MARGIN);
	if (rc < 0) {
		chg_err("pgm_set_margin failed\n");
		return rc;
	}

	state = pgm_state(chip);
	if (state != PGM_STATE_DONE) {
		chg_err("Error: pgm access state 0x%02x\n", state);
		return -EINVAL;
	}

	return 0;
}

__maybe_unused static int count_num(u8 *buffer, u16 len, u8 num)
{
	int i;
	int count = 0;

	for (i = 0; i < len; i++) {
		if (buffer[i] == num)
			count++;
	}

	return count;
}

__maybe_unused static int pgm_info(struct oplus_sc96257 *chip, char *info)
{
	u8 state;
	int i;
	int index;
	union pgm_pkt_type pgm = { 0 };
	u16 msg_len;
	int rc;

	pgm.type.xor = pgm_xor(&pgm.value[PGM_XOR_INDEX], PGM_XOR_LEN);
	pgm.type.xor ^= PGM_CMD_INFO << 8;
	rc = sc96257_write_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, &pgm.value[0], PGM_HEADER_SIZE);
	rc |= sc96257_write_byte(chip, PGM_SRAM_BASE + PGM_CMD_ADDR, PGM_CMD_INFO);
	if (rc < 0) {
		chg_err("write pgm_info failed\n");
		return rc;
	}

	state = pgm_state(chip);
	if (state == PGM_STATE_DONE) {
		rc = sc96257_read_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, (u8 *)pgm.value, PGM_HEADER_SIZE);
		msg_len = pgm.type.len * PGM_WORD;
		rc |= sc96257_read_block(chip, PGM_SRAM_BASE + PGM_MSG_ADDR, pgm.type.msg, msg_len);
		if (rc < 0) {
			chg_err("read pgm_info failed\n");
			return rc;
		}
		if (pgm_xor((u8 *)pgm.value, msg_len + PGM_HEADER_SIZE) == 0x00) {
			index = 0;
			for (i = 0; i < msg_len; i++) {
				if (pgm.type.msg[i] != 0) {
					info[index] = pgm.type.msg[i];
					index++;
				}
			}
		}
	} else {
		chg_err("Error: pgm info state 0x%02x\n", state);
		return -EINVAL;
	}

	return 0;
}

__maybe_unused static int pgm_access(struct oplus_sc96257 *chip, u8 *key_buffer, u8 key_len)
{
	u8 state;
	union pgm_pkt_type pgm = {0};
	int rc;

	pgm.type.addr = 0x00;
	pgm.type.len = key_len / PGM_WORD;
	memcpy(pgm.type.msg, key_buffer, key_len);

	pgm.type.xor = pgm_xor(&pgm.value[PGM_XOR_INDEX], PGM_XOR_LEN);
	pgm.type.xor ^= PGM_CMD_AUTH << 8;
	rc = sc96257_write_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, &pgm.value[0], key_len + PGM_HEADER_SIZE);
	rc |= sc96257_write_byte(chip, PGM_SRAM_BASE + PGM_CMD_ADDR, PGM_CMD_AUTH);
	if (rc < 0) {
		chg_err("pgm_access failed\n");
		return rc;
	}

	state = pgm_state(chip);
	if (state != PGM_STATE_DONE) {
		chg_err("Error: pgm access state 0x%02x\n", state);
		return -EINVAL;
	}

	return 0;
}

static int pgm_read_crc(struct oplus_sc96257 *chip, u8 *crc, u16 crc_size, u16 addr, u16 len)
{
	volatile u8 state;
	int i;
	int index;
	union pgm_pkt_type pgm = { 0 };
	u16 msg_len;
	int rc;

	pgm.type.addr = addr;
	pgm.type.len = len;
	pgm.type.xor = pgm_xor(&pgm.value[PGM_XOR_INDEX], PGM_XOR_LEN);

	pgm.type.xor ^= PGM_CMD_VERIFY << 8;
	rc = sc96257_write_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, &pgm.value[0], PGM_HEADER_SIZE);
	rc |= sc96257_write_byte(chip, PGM_SRAM_BASE + PGM_CMD_ADDR, PGM_CMD_VERIFY);
	if (rc < 0) {
		chg_err("write pgm_read_crc failed\n");
		return rc;
	}
	usleep_range(1000, 1100);
	state = pgm_state(chip);
	if (state == PGM_STATE_DONE) {
		rc = sc96257_read_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, (uint8_t *)pgm.value, PGM_HEADER_SIZE);
		msg_len = crc_size;
		rc |= sc96257_read_block(chip, PGM_SRAM_BASE + PGM_MSG_ADDR, pgm.type.msg, msg_len);
		if (rc < 0) {
			chg_err("read pgm_read_crc failed\n");
			return rc;
		}
		if (pgm_xor((u8 *)pgm.value, msg_len + PGM_HEADER_SIZE) == 0x00) {
			index = 0;
			for (i = 0; (i < msg_len) && (i < crc_size); i++) {
				crc[index] = pgm.type.msg[i];
				index++;
			}
		}
	} else {
		chg_err("Error: pgm read crc state 0x%02x\n", state);
		return -EINVAL;
	}

	return 0;
}

__maybe_unused static int pgm_read(struct oplus_sc96257 *chip, u32 addr, u8 *buffer, u16 len, u8 sel)
{
	int i;
	u8 state;
	union pgm_pkt_type pgm = { 0 };
	u16 offset;
	u8 *p_buffer;
	int rc;

	p_buffer = buffer;

	for (i = 0; i < len; i += PGM_MSG_SIZE) {
		memset(&pgm.value, 0, sizeof(pgm.value));
		pgm.type.addr = (addr + i) / PGM_WORD;
		offset = PGM_MSG_SIZE;
		if (len < (i + PGM_MSG_SIZE))
			offset = len - i;
		pgm.type.len = offset /PGM_WORD;

		pgm.type.xor = pgm_xor(&pgm.value[PGM_XOR_INDEX], PGM_XOR_LEN);
		pgm.type.xor ^= sel << 8;

		rc = sc96257_write_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, &pgm.value[0], PGM_HEADER_SIZE);
		rc |= sc96257_write_byte(chip, PGM_SRAM_BASE + PGM_CMD_ADDR, sel);
		if (rc < 0) {
			chg_err("pgm_read failed\n");
			return rc;
		}

		state = pgm_state(chip);
		if (state == PGM_STATE_DONE) {
			rc = sc96257_read_block(chip, PGM_SRAM_BASE + PGM_MSG_ADDR, p_buffer, offset);
			if (rc < 0) {
				chg_err("read PGM_MSG_ADDR failed\n");
				return -EINVAL;
			}
		} else {
			chg_err("Error: pgm read state 0x%02x\n", state);
			return -EINVAL;
		}
		p_buffer += offset;
	}

	return 0;
}

static int pgm_write(struct oplus_sc96257 *chip, u32 addr, u8 *buffer, u16 len, u8 sel)
{
	int i;
	u8 state;
	union pgm_pkt_type pgm = { 0 };
	u16 offset;
	int rc;

	for (i = 0; i < len; i += PGM_FRIMWARE_SIZE) {
		memset(pgm.value, 0, sizeof(pgm.value));
		pgm.type.addr = (addr + i) / PGM_WORD;
		offset = PGM_FRIMWARE_SIZE;
		if (len < (i + PGM_FRIMWARE_SIZE))
			offset = len - i;
		pgm.type.len = offset / PGM_WORD;
		memcpy(pgm.type.msg, &buffer[i], offset);
		pgm.type.xor = pgm_xor(&pgm.value[PGM_XOR_INDEX], offset + PGM_XOR_LEN);
		pgm.type.xor ^= (sel << 8);
		rc = sc96257_write_block(chip, PGM_SRAM_BASE + PGM_STATE_ADDR, pgm.value, offset + PGM_HEADER_SIZE);
		rc |= sc96257_write_byte(chip, PGM_SRAM_BASE + PGM_CMD_ADDR, sel);
		if (rc < 0) {
			chg_err("pgm_write failed\n");
			return rc;
		}

		state = pgm_state(chip);
		if (state != PGM_STATE_DONE) {
			chg_err("Error: pgm write state 0x%02x, cmd 0x%02x\n", state, sel);
			return -EINVAL;
		}
	}

	return 0;
}

static int bool_sel(struct oplus_sc96257 *chip, bool en)
{
	int rc;
	u8 data;

	rc = sc96257_read_byte(chip, REG_ATE, &data);
	if (rc < 0)
		return rc;

	if (en)
		data |= BIT(ATE_BOOT_BIT);
	else
		data &= ~BIT(ATE_BOOT_BIT);

	return sc96257_write_byte(chip, REG_ATE, data);
}

static int sys_reset_ctrl(struct oplus_sc96257 *chip, bool en)
{
	if (en)
		return sc96257_write_byte(chip, REG_RST, RST_PW0);
	else
		return sc96257_write_byte(chip, REG_RST, RST_PW1);
}

static int mcu_idle_ctrl(struct oplus_sc96257 *chip, bool en)
{
	if (en)
		return sc96257_write_byte(chip, REG_IDLE, IDLE_PW0);
	else
		return sc96257_write_byte(chip, REG_IDLE, IDLE_PW1);
}

static int wdt_close(struct oplus_sc96257 *chip)
{
	int rc;

	rc = sc96257_write_byte(chip, REG_WDT, WDT_CLOSE_PW0);
	rc |= sc96257_write_byte(chip, REG_WDT, WDT_CLOSE_PW1);
	rc |= sc96257_write_byte(chip, REG_WDT, WDT_CLOSE_PW2);

	return rc;
}

static int sram_write_prepare(struct oplus_sc96257 *chip)
{
	int rc;

	rc = wdt_close(chip);
	usleep_range(1000, 1100);
	rc |= wdt_close(chip);
	if (rc < 0) {
		chg_err("WDT close failed\n");
		return -EINVAL;
	}

	/*reset mcu*/
	rc = sys_reset_ctrl(chip, true);
	rc |= sys_reset_ctrl(chip, false);
	if (rc < 0) {
		chg_err("sys_reset_ctrl failed\n");
		return -EINVAL;
	}
	usleep_range(2000, 2100);

	rc = mcu_idle_ctrl(chip, true);
	if (rc < 0) {
		chg_err("mcu_idle_ctrl failed\n");
		return -EINVAL;
	}

	return 0;
}

static int sram_write(struct oplus_sc96257 *chip)
{
	int i;
	int rc;
	int addr_base = SRAM_BASE_ADDRESS;
	u32 size = sizeof(sc96257_pgm_data);
	u8 *pgm_data_read;
	u32 read_crc = 0;
	static int crc_check_num = 0;

	if (size % PGM_WORD) {
		chg_err("pgm_data error\n");
		return -EINVAL;
	}

	rc = sram_write_prepare(chip);
	if (rc < 0) {
		chg_err("sram write prepare failed\n");
		return rc;
	}

	for (i = 0; i < size; i += PGM_WORD) {
		rc = sc96257_write_block(chip, addr_base + i, &sc96257_pgm_data[i], PGM_WORD);
		if (rc < 0) {
			chg_err("sc96257_write_block failed\n");
			return -EIO;
		}
	}

	pgm_data_read = kzalloc(size, GFP_KERNEL);
	if (pgm_data_read) {
		for (i = 0; i < size; i += PGM_WORD) {
			rc = sc96257_read_block(chip, addr_base + i, &pgm_data_read[i], PGM_WORD);
			if (rc < 0) {
				chg_err("sc96257_read_block failed\n");
				kfree(pgm_data_read);
				return -EINVAL;
			}
		}

		for (i = 0; i < size; i++) {
			if (pgm_data_read[i] != sc96257_pgm_data[i]) {
				chg_err("pgm read != pgm data, num=%d, read=0x%x write=0x%x\n", i, pgm_data_read[i], sc96257_pgm_data[i]);
				kfree(pgm_data_read);
				return -EINVAL;
			}
		}
		kfree(pgm_data_read);
	}

	rc = bool_sel(chip, true);
	rc |= sys_reset_ctrl(chip, true);
	rc |= sys_reset_ctrl(chip, false);
	if (rc < 0) {
		chg_err("sys_reset_ctrl failed\n");
		return rc;
	}

	while(1) {
		/*set margin*/
		rc = pgm_set_margin(chip, CRC_CHECK_MARGIN0);
		if (rc < 0) {
			if (crc_check_num >= 10) {
				chg_err("run pgm set margin fail\n");
				return rc;
			}
		}

		rc = pgm_read_crc(chip, (u8 *)(&read_crc), CRC_SIZE, 0x0000, CHIP_SIZE / PGM_WORD);
		if (rc < 0) {
			if (crc_check_num >= 10) {
				chg_err("run pgm final get crc failed\n");
				return -EINVAL;
			} else {
				chg_err("run pgm get crc failed num=%d\n", crc_check_num);
			}
		} else {
			crc_check_num = 0;
			chg_info("run pgm get crc successfully\n");
			break;
		}
		crc_check_num = crc_check_num + 1;
		usleep_range(1000, 1100);
	}

	/*rc = pgm_info(chip, info);
	info[sizeof(info) - 1] = '\0';
	chg_info("info = %s\n", info);
	if (info[0] != 'S') {
		chg_err("pgm run failed\n");
		return -EINVAL;
	} else {
		chg_info("pgm run sucessfully\n");
	}*/
	chg_info("pgm run sucessfully\n");

	return rc;
}

static int fw_program(struct oplus_sc96257 *chip, u8 *buf, u32 len)
{
	int rc;

	rc = pgm_chip_erase(chip);
	if (rc < 0)
		return -EINVAL;

	return pgm_write(chip, 0x0000, buf, (u16)len, PGM_CMD_WRITE_CODE);
}

static int crc_check(struct oplus_sc96257 *chip, u8 margin, u8 *buf, u32 len)
{
	int i;
	int rc;
	u32 data32;
	u32 read_crc;

	/*set margin*/
	rc = pgm_set_margin(chip, margin);
	if (rc < 0) {
		chg_err("set margin fail\n");
		return -EINVAL;
	}

	rc = pgm_read_crc(chip, (uint8_t *)(&read_crc), CRC_SIZE, 0x0000, CHIP_SIZE / PGM_WORD);
	if (rc < 0) {
		chg_err("pgm read crc failed");
		return -EINVAL;
	}

	data32 = 0xFFFFFFFF;
	for (i = 0; i < (CHIP_SIZE / PGM_WORD); i++)
		data32 = sc96257_func_crc32(*(uint32_t *)(buf + i * PGM_WORD), data32);

	data32 = endian_conversion(data32);

	if (data32 != read_crc) {
		chg_err("---->data = 0x%08x read = 0x%08x\n", data32, read_crc);
		chg_err("check crc fail\n");
		return -EINVAL;
	}

	return 0;
}

static int sc96257_mtp_program(struct oplus_sc96257 *chip, unsigned char *fw_buf, int fw_size)
{
	int rc;

	/*run pgm*/
	rc = sram_write(chip);
	if (rc < 0) {
		chg_err("sram write fail\n");
		goto program_fail;
	}
	rc = fw_program(chip, fw_buf, fw_size);
	if (rc < 0) {
		chg_err("fw program fail\n");
		goto program_fail;
	}
	chg_info("<FW UPDATE> sc96257_mtp_program OK !!!\n");

	return 0;

program_fail:
	chg_info("<FW UPDATE> sc96257_mtp_program FAILED !!!\n");
	return -EIO;
}

static int sc96257_check_fw(struct oplus_sc96257 *chip, unsigned char *fw_buf, int fw_size)
{
	int rc;

	rc = crc_check(chip, CRC_CHECK_MARGIN0, fw_buf, fw_size);
	if (rc < 0) {
		chg_err("margin 0 check crc fail\n");
		return -EIO;
	}

	rc = crc_check(chip, CRC_CHECK_MARGIN7, fw_buf, fw_size);
	if (rc < 0) {
		chg_err("margin 7 check crc fail\n");
		return -EIO;
	}

	return 0;
}

static int sc96257_get_fw_version(struct oplus_sc96257 *chip, u32 *fw_ver)
{
	int rc;
	u16 addr;

	addr = (u16)ADDR(struct rx_cust_type, fw_ver);
	rc = sc96257_read_block(chip, addr, (u8 *)fw_ver, sizeof(chip->info.rx_info.fw_ver));
	if (rc < 0) {
		chg_err("get fw_version fail\n");
		return -EINVAL;
	}

	return 0;
}

static int sc96257_get_fw_check(struct oplus_sc96257 *chip, u32 *fw_check)
{
	int rc;
	u16 addr;

	addr = (u16)ADDR(struct rx_cust_type, fw_check);
	rc = sc96257_read_block(chip, addr, (u8 *)fw_check, sizeof(chip->info.rx_info.fw_check));
	if (rc < 0) {
		chg_err("get fw_check fail\n");
		return -EINVAL;
	}

	return 0;
}

static int sc96257_upgrade_firmware_by_buf(struct oplus_chg_ic_dev *dev, unsigned char *fw_buf, int fw_size)
{
	struct oplus_sc96257 *chip;
	int rc;
	u32 fw_check = 0;
	u32 fw_version = 0;
	u32 new_version = 0;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	if (fw_buf == NULL) {
		chg_err("fw_buf is NULL\n");
		return -EINVAL;
	}

	chg_info("<FW UPDATE> check idt fw update<><><><><><><><>\n");

	disable_irq(chip->rx_con_irq);
	disable_irq(chip->rx_event_irq);

	rc = sc96257_set_trx_boost_vol(chip, SC96257_MTP_VOL_MV);
	if (rc < 0) {
		chg_err("set trx power vol(=%d), rc=%d\n", SC96257_MTP_VOL_MV, rc);
		goto exit_enable_irq;
	}
	rc = sc96257_set_trx_boost_enable(chip, true);
	if (rc < 0) {
		chg_err("enable trx power error, rc=%d\n", rc);
		goto exit_enable_irq;
	}
	msleep(100);

	sc96257_set_wake_pattern(chip);
	msleep(10);

	if (chip->debug_force_ic_err == WLS_IC_ERR_I2C) {
		chg_err("<FW UPDATE> debug i2c error!\n");
		chip->debug_force_ic_err = WLS_IC_ERR_NONE;
		sc96257_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_UPDATE, WLS_IC_ERR_I2C);
	}

	rc = sc96257_get_fw_check(chip, &fw_check);
	if (rc < 0) {
		sc96257_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_UPDATE, WLS_IC_ERR_I2C);
		goto exit_disable_boost;
	} else {
		chg_info("fw_check=0x%x\n", fw_check);
	}
	rc = sc96257_get_fw_version(chip, &fw_version);
	if (rc < 0)
		goto exit_disable_boost;
	else
		chg_info("fw_version=0x%x\n", fw_version);

	new_version = (fw_buf[fw_size - FW_VERSION_OFFSET]) |
		(fw_buf[fw_size - FW_VERSION_OFFSET + 1] << 8) |
		(fw_buf[fw_size - FW_VERSION_OFFSET + 2] << 16) |
		(fw_buf[fw_size - FW_VERSION_OFFSET + 3] << 24);
	if (fw_version == fw_check && fw_version == new_version) {
		chg_info("<FW UPDATE> fw is the same, fw_version[0x%x]!\n", fw_version);
		goto exit_disable_boost;
	}

	rc = sc96257_mtp_program(chip, fw_buf, fw_size);
	if (rc < 0) {
		fw_version = 0;
		sc96257_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_UPDATE, WLS_IC_ERR_OTHER);
	} else {
		rc = sc96257_check_fw(chip, fw_buf, fw_size);
		if (rc < 0) {
			fw_version = 0;
			chg_err("<FW UPDATE> check idt fw update fail<><><><><><><><>\n");
			sc96257_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_UPDATE, WLS_IC_ERR_CRC);
		} else {
			fw_version = new_version;
			chg_info("<FW UPDATE> check idt fw update ok<><><><><><><><>\n");
		}
	}

	msleep(100);
exit_disable_boost:
	(void)sc96257_set_trx_boost_enable(chip, false);
	msleep(20);
	chip->rx_fw_version = fw_version;
	chip->tx_fw_version = fw_version;

exit_enable_irq:
	enable_irq(chip->rx_con_irq);
	enable_irq(chip->rx_event_irq);

	return rc;
}

static int sc96257_upgrade_firmware_by_img(struct oplus_chg_ic_dev *dev)
{
	int rc;
	struct oplus_sc96257 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	if (chip == NULL) {
		chg_err("oplus_sc96257 is NULL\n");
		return -ENODEV;
	}
	if (!chip->fw_data || chip->fw_length < FW_VERSION_OFFSET) {
		chg_err("fw data failed\n");
		return -EINVAL;
	}
	rc = sc96257_upgrade_firmware_by_buf(dev, chip->fw_data, chip->fw_length);

	return rc;
}

static void sc96257_tx_event_config(struct oplus_sc96257 *chip, unsigned int status, unsigned int err)
{
	enum wls_ic_err_reason tx_err_reason = WLS_IC_ERR_NONE;

	if (chip == NULL) {
		chg_err("oplus_sc96257 is NULL\n");
		return;
	}

	if (err != 0) {
		switch (err) {
		case SC96257_TX_ERR_RXAC:
			chip->tx_status = TX_STATUS_ERR_RXAC;
			tx_err_reason = WLS_IC_ERR_RXAC;
			break;
		case SC96257_TX_ERR_OCP:
			chip->tx_status = TX_STATUS_ERR_OCP;
			tx_err_reason = WLS_IC_ERR_OCP;
			break;
		case SC96257_TX_ERR_OVP:
			chip->tx_status = TX_STATUS_ERR_OVP;
			tx_err_reason = WLS_IC_ERR_OVP;
			break;
		case SC96257_TX_ERR_LVP:
			chip->tx_status = TX_STATUS_ERR_LVP;
			tx_err_reason = WLS_IC_ERR_LVP;
			break;
		case SC96257_TX_ERR_FOD:
			chip->tx_status = TX_STATUS_ERR_FOD;
			tx_err_reason = WLS_IC_ERR_FOD;
			break;
		case SC96257_TX_ERR_OTP:
			chip->tx_status = TX_STATUS_ERR_OTP;
			tx_err_reason = WLS_IC_ERR_OTP;
			break;
		case SC96257_TX_ERR_CEPTIMEOUT:
			chip->tx_status = TX_STATUS_ERR_CEPTIMEOUT;
			break;
		case SC96257_TX_ERR_RXEPT:
			chip->tx_status = TX_STATUS_ERR_RXEPT;
			tx_err_reason = WLS_IC_ERR_RXEPT;
			break;
		case SC96257_TX_ERR_VRECTOVP:
			chip->tx_status = TX_STATUS_ERR_VRECTOVP;
			tx_err_reason = WLS_IC_ERR_VRECTOVP;
			break;
		default:
			break;
		}
	} else {
		switch (status) {
		case SC96257_TX_STATUS_READY:
			chip->tx_status = TX_STATUS_READY;
			break;
		case SC96257_TX_STATUS_DIGITALPING:
		/*case SC96257_TX_STATUS_ANALOGPING:*/
			chip->tx_status = TX_STATUS_PING_DEVICE;
			break;
		case SC96257_TX_STATUS_TRANSFER:
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
		sc96257_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_TX, tx_err_reason);

	return;
}

static void sc96257_clear_irq(struct oplus_sc96257 *chip, u32 irq_clr)
{
	u16 addr;
	u32 irq_clr_val = irq_clr;

	chg_info("sc96257_clear_irq----------\n");
	addr = (u16)ADDR(struct rx_cust_type, irq_clr);
	(void)sc96257_write_block(chip, addr, (u8 *)&irq_clr_val, sizeof(chip->info.rx_info.irq_clr));
	return;
}

static void sc96257_event_process(struct oplus_sc96257 *chip)
{
	union fsk_pkt_type fsk_packet;
	bool enable = false;
	u32 irq_flag;
	u8 val_buf[6] = { 0 };
	u16 addr;
	int rc;
	enum wls_ic_err_reason rx_err_reason = WLS_IC_ERR_NONE;
	u32 mode_value;

	sc96257_rx_is_enable(chip->ic_dev, &enable);
	if (!enable && sc96257_get_wls_type(chip) != OPLUS_CHG_WLS_TRX && !chip->standby_config &&
	    (chip->rx_mode != OPLUS_CHG_WLS_RX_MODE_BPP)) {
		chg_info("RX is sleep or TX is disable, Ignore events\n");
		return;
	}

	addr = (u16)ADDR(struct rx_cust_type, irq_flag);
	if (sc96257_read_block(chip, addr, (u8 *)&irq_flag, sizeof(chip->info.rx_info.irq_flag)) < 0) {
		chg_err("read irq_flag error\n");
		 return;
	} else {
		chg_info("irq_flag: [0x%x]\n", irq_flag);
	}
	if (irq_flag == 0 || irq_flag == SC96257_ERR_IRQ_VALUE)
		 goto out;

	if (chip->standby_config && (irq_flag & WP_IRQ_TX_AC_PRESENT)) {
		if (sc96257_get_sys_mode(chip, &mode_value) < 0) {
			chg_err("read sys mode error\n");
			return;
		}
		if (mode_value & AC_MODE)
			chip->event_code = WLS_EVENT_RXAC_ATTACH;
		else
			chip->event_code = WLS_EVENT_RXAC_DETACH;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
	}

	if (sc96257_get_wls_type(chip) == OPLUS_CHG_WLS_TRX) {
		sc96257_tx_event_config(chip, irq_flag & SC96257_TX_STATUS_MASK, irq_flag & SC96257_TX_ERR_MASK);
		if (irq_flag & WP_IRQ_TX_VAC_PRESENT)
			/*trigger only in tx mode*/
			chip->event_code = WLS_EVENT_VAC_PRESENT;
		else if (irq_flag & WP_IRQ_TX_BLE_MAC_ADDR)
			chip->event_code = WLS_EVENT_BLE_MAC_ADDR;
		else if (irq_flag & WP_IRQ_TX_WLSPEN_CHG_STATUS)
			chip->event_code = WLS_EVENT_WLSPEN_CHG_STATUS;
		else
			chip->event_code = WLS_EVENT_TRX_CHECK;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
		goto out;
	} else {
		chip->tx_status = TX_STATUS_OFF;
	}

	if (chip->rx_connected == true && (irq_flag & WP_IRQ_RX_LDO_ON)) {
		chg_err("LDO is on, connected.\n");
		complete(&chip->ldo_on);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ONLINE);
		chip->connected_ldo_on = true;
	}

	if (chip->rx_connected == true) {
		if (irq_flag & WP_IRQ_OCP) {
			rx_err_reason = WLS_IC_ERR_OCP;
			chg_err("rx OCP happen!\n");
		} else if (irq_flag & WP_IRQ_OVP) {
			rx_err_reason = WLS_IC_ERR_OVP;
			chg_err("rx OVP happen!\n");
		} else if (irq_flag & WP_IRQ_OVP0) {
			rx_err_reason = WLS_IC_ERR_OVP0;
			chg_err("rx CLAMPOVP happen!\n");
		} else if (irq_flag & WP_IRQ_OVP1) {
			rx_err_reason = WLS_IC_ERR_OVP1;
			chg_err("rx HARDOVP happen!\n");
		} else if (irq_flag & WP_IRQ_LVP) {
			rx_err_reason = WLS_IC_ERR_LVP;
			chg_err("rx LVP happen!\n");
		} else if (irq_flag & WP_IRQ_OTP_160) {
			rx_err_reason = WLS_IC_ERR_OTP;
			chg_err("rx OTP happen!\n");
		} else if (irq_flag & WP_IRQ_OTP) {
			rx_err_reason = WLS_IC_ERR_SOFTOTP;
			chg_err("rx SOFTOTP happen!\n");
		} else if (irq_flag & WP_IRQ_RX_V2X_UCP) {
			rx_err_reason = WLS_IC_ERR_V2X_UCP;
			chg_err("rx V2XUCP happen!\n");
		} else if (irq_flag & WP_IRQ_RX_V2X_OVP) {
			rx_err_reason = WLS_IC_ERR_V2X_OVP;
			chg_err("rx V2XOVP happen!\n");
		} else if (irq_flag & WP_IRQ_RX_V2X_VV_UVP) {
			rx_err_reason = WLS_IC_ERR_V2X_VV_UVP;
			chg_err("rx V2XVVUVP happen!\n");
		} else if (irq_flag & WP_IRQ_RX_V2X_VV_OVP) {
			rx_err_reason = WLS_IC_ERR_V2X_VV_OVP;
			chg_err("rx V2XVVOVP happen!\n");
		}
		if (chip->debug_force_ic_err) {
			rx_err_reason = chip->debug_force_ic_err;
			chip->debug_force_ic_err = WLS_IC_ERR_NONE;
		}
		if (rx_err_reason != WLS_IC_ERR_NONE)
			sc96257_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_RX, rx_err_reason);
	}

	if (irq_flag & WP_IRQ_POWER_PROFILE) {
		(void)sc96257_get_sys_mode(chip, &mode_value);
		if (mode_value & BPP_MODE) {
			chip->adapter_type = 0;
			chip->event_code = WLS_EVENT_EPP_NEGO_FAIL;
		} else {
			chip->rx_pwr_cap = sc96257_get_power_cap(chip);
			chip->event_code = WLS_EVENT_RX_EPP_CAP;
		}
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
	}

	if (irq_flag & WP_IRQ_PKT_RECV) {
		addr = (u16)ADDR(struct rx_cust_type, fsk_packet);
		rc = sc96257_read_block(chip, addr, (u8 *)&(fsk_packet.buf), MAX_FSK_SIZE);
		if (rc < 0) {
			chg_err("Couldn't read fsk_packet, rc=%d\n", rc);
		} else {
			memcpy(&val_buf, &(fsk_packet.buf), sizeof(val_buf));
			chg_info("Received TX data: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				val_buf[0], val_buf[1], val_buf[2], val_buf[3], val_buf[4], val_buf[5]);
			if (chip->rx_msg.msg_call_back != NULL)
				chip->rx_msg.msg_call_back(chip->rx_msg.dev_data, val_buf);
		}
	}

	if (irq_flag & WP_IRQ_RX_EPP_ID_SUCCESS) {
		chip->event_code = WLS_EVENT_EPP_TX_MANU_ID;
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
	}

out:
	sc96257_clear_irq(chip, irq_flag);

	return;
}

static int sc96257_connect_check(struct oplus_chg_ic_dev *dev)
{
	struct oplus_sc96257 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	schedule_delayed_work(&chip->connect_work, 0);

	return 0;
}

static int sc96257_get_event_code(struct oplus_chg_ic_dev *dev, enum oplus_chg_wls_event_code *code)
{
	struct oplus_sc96257 *chip;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);
	*code = chip->event_code;
	return 0;
}

static int sc96257_get_bridge_mode(struct oplus_chg_ic_dev *dev, int *mode)
{
	struct oplus_sc96257 *chip;
	u16 addr;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, full_bridge);

	/*refresh*/
	rc = sc96257_rx_set_cmd(chip, AP_CMD_REFRESH);
	if (rc < 0) {
		chg_err("set cmd refresh fail\n");
		return rc;
	}

	rc = sc96257_read_block(chip, addr, (u8 *)mode, sizeof(chip->info.rx_info.full_bridge));
	if (rc < 0) {
		chg_err("read bridge mode err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> bridge mode:0x%x.\n", *mode);

	return 0;
}

static int sc96257_set_insert_disable(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_sc96257 *chip;
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

static int sc96257_standby_config(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_sc96257 *chip;
	int rc = 0;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	if (en)
		rc = sc96257_set_wake_pattern(chip);
	chip->standby_config = en;

	return rc;
}

#define COMU_CAP_AUTO	0
#define COMU_CAP_22UF	3
#define COMU_CAP_47UF	5
#define COMU_CAP_69UF	7
static int sc96257_set_rx_comu(struct oplus_chg_ic_dev *dev, int comu)
{
	struct oplus_sc96257 *chip;
	u16 addr;
	u8 comu_cap;
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	switch (comu) {
	case WLS_RX_COMU_CAP_AUTO:
		comu_cap = COMU_CAP_AUTO;
		break;
	case WLS_RX_COMU_CAP_A:
		comu_cap = COMU_CAP_22UF;
		break;
	case WLS_RX_COMU_CAP_B:
		comu_cap = COMU_CAP_47UF;
		break;
	case WLS_RX_COMU_CAP_AB:
		comu_cap = COMU_CAP_69UF;
		break;
	default:
		comu_cap = COMU_CAP_AUTO;
		break;
	}
	addr = (u16)ADDR(struct rx_cust_type, ask_cfg);
	rc = sc96257_write_block(chip, addr, (u8 *)&comu_cap, sizeof(chip->info.rx_info.ask_cfg));
	if (rc < 0) {
		chg_err("set comu err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> set comu:%d\n", comu);

	return 0;
}

static int sc96257_get_tx_id(struct oplus_chg_ic_dev *dev, int *tx_id)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, tx_mfr_code);
	rc = sc96257_read_block(chip, addr, (u8 *)tx_id, sizeof(chip->info.rx_info.tx_mfr_code));
	if (rc < 0) {
		chg_err("read tx_id err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> tx_id:0x%x.\n", *tx_id);

	return 0;
}

static int sc96257_set_silent(struct oplus_chg_ic_dev *dev)
{
	struct oplus_sc96257 *chip;
	u16 addr;
	u8 silent = BIT(3);
	int rc;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, silent_sleep);
	rc = sc96257_write_block(chip, addr, (u8 *)&silent, sizeof(chip->info.rx_info.silent_sleep));
	if (rc < 0) {
		chg_err("set silent err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> set silent...\n");

	return 0;
}

static int sc96257_get_ble_mac_addr(struct oplus_chg_ic_dev *dev, u64 *mac_addr)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;
	u8 mac_check[4] = { 0 };
	u8 mac_addr_l[5] = { 0 };
	u8 mac_addr_h[5] = { 0 };
	u8 decode_addr[6] = { 0 };

	if (dev == NULL || mac_addr == NULL) {
		chg_err("oplus_chg_ic_dev or mac_addr is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct tx_cust_type, mac_check);
	rc = sc96257_read_block(chip, addr, mac_check, sizeof(chip->info.tx_info.mac_check));
	if (rc < 0) {
		chg_err("read mac_check err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> mac_check:0x%x,0x%x,0x%x,0x%x.\n",
		mac_check[0], mac_check[1], mac_check[2], mac_check[3]);
	addr = (u16)ADDR(struct tx_cust_type, mac_addr_l);
	rc = sc96257_read_block(chip, addr, mac_addr_l, sizeof(chip->info.tx_info.mac_addr_l));
	if (rc < 0) {
		chg_err("read mac_addr_l err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> mac_addr_l:0x%x,0x%x,0x%x,0x%x,0x%x.\n",
		mac_addr_l[0], mac_addr_l[1], mac_addr_l[2], mac_addr_l[3], mac_addr_l[4]);
	addr = (u16)ADDR(struct tx_cust_type, mac_addr_h);
	rc = sc96257_read_block(chip, addr, mac_addr_h, sizeof(chip->info.tx_info.mac_addr_h));
	if (rc < 0) {
		chg_err("read mac_addr_h err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> mac_addr_h:0x%x,0x%x,0x%x,0x%x,0x%x.\n",
		mac_addr_h[0], mac_addr_h[1], mac_addr_h[2], mac_addr_h[3], mac_addr_h[4]);
	/*mac_addr_l/h byte[0~1] is packet head, byte[2~4] is MAC data*/
	decode_addr[5] = ((mac_addr_h[4] & 0x0F) << 4) | ((mac_addr_l[4] & 0xF0) >> 4);
	decode_addr[4] = ((mac_addr_h[3] & 0x0F) << 4) | ((mac_addr_l[3] & 0xF0) >> 4);
	decode_addr[3] = ((mac_addr_h[2] & 0x0F) << 4) | ((mac_addr_l[2] & 0xF0) >> 4);

	decode_addr[2] = ((mac_addr_l[4] & 0x0F) << 4) | ((mac_addr_h[4] & 0xF0) >> 4);
	decode_addr[1] = ((mac_addr_l[3] & 0x0F) << 4) | ((mac_addr_h[3] & 0xF0) >> 4);
	decode_addr[0] = ((mac_addr_l[2] & 0x0F) << 4) | ((mac_addr_h[2] & 0xF0) >> 4);

	/*MAC address verify*/
	if ((decode_addr[5] ^ decode_addr[4] ^ decode_addr[3]) != mac_check[2] ||
	    (decode_addr[2] ^ decode_addr[1] ^ decode_addr[0]) != mac_check[3]) {
		chg_err("MAC address verify fail\n");
		return -EINVAL;
	}

	*mac_addr = decode_addr[5]  << 16 | decode_addr[4] << 8 | decode_addr[3];
	*mac_addr = *mac_addr << 24 | decode_addr[2] << 16 | decode_addr[1] << 8 | decode_addr[0];
	chg_debug("<~WPC~> mac addr: 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x, check data: 0x%x,0x%x.\n", decode_addr[5], decode_addr[4],
		decode_addr[3], decode_addr[2], decode_addr[1], decode_addr[0], mac_check[2], mac_check[3]);
	return 0;
}

static int sc96257_get_wlspen_id(struct oplus_chg_ic_dev *dev, u8 *wlspen_id)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;

	if (dev == NULL || wlspen_id == NULL) {
		chg_err("oplus_chg_ic_dev or wlspen_id is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct tx_cust_type, wlspen_id);
	rc = sc96257_read_block(chip, addr, (u8 *)wlspen_id, sizeof(chip->info.tx_info.wlspen_id));
	if (rc < 0) {
		chg_err("read wlspen_id err, rc=%d\n", rc);
		return rc;
	}
	chg_info("<~WPC~> wlspen_id:0x%x.\n", *wlspen_id);

	return 0;
}

static int sc96257_get_wlspen_chg_status(struct oplus_chg_ic_dev *dev, u8 *chg_status)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;
	u8 val[2] = { 0 };

	if (dev == NULL || chg_status == NULL) {
		chg_err("oplus_chg_ic_dev or chg_status is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct tx_cust_type, chg_status);
	rc = sc96257_read_block(chip, addr, (u8 *)val, sizeof(chip->info.tx_info.chg_status));
	if (rc < 0) {
		chg_err("read chg_status err, rc=%d\n", rc);
		return rc;
	}
	/*val[0] is packet head*/
	*chg_status = val[1];
	chg_info("<~WPC~> chg_status:0x%x.\n", *chg_status);

	return 0;
}

static int sc96257_get_ac_ov_flag(struct oplus_chg_ic_dev *dev, int *ac_ov_flag)
{
	if (dev == NULL || ac_ov_flag == NULL) {
		chg_err("oplus_chg_ic_dev or ac_ov_flag is NULL\n");
		return -ENODEV;
	}
	*ac_ov_flag = SC96257_AC_OV_FLAG;

	return 0;
}

static int sc96257_fw_checksum(struct oplus_sc96257 *chip)
{
	u32 fw_check = 0;
	u32 fw_version = 0;
	int rc;

	rc = sc96257_get_fw_check(chip, &fw_check);
	if (rc < 0) {
		chg_err("read fw_check err, rc=%d\n", rc);
		return FW_CHECKSUM_UNKNOWN;
	}
	rc = sc96257_get_fw_version(chip, &fw_version);
	if (rc < 0) {
		chg_err("read fw_version err, rc=%d\n", rc);
		return FW_CHECKSUM_UNKNOWN;
	}

	if (fw_check == fw_version && fw_version != INVALID_FW_VERSION0 && fw_version != INVALID_FW_VERSION1)
		return FW_CHECKSUM_OK;
	return FW_CHECKSUM_FAIL;
}

static bool sc96257_vac_acdrv_check(struct oplus_sc96257 *chip)
{
	int checksum;

	checksum = sc96257_fw_checksum(chip);
	if (checksum == FW_CHECKSUM_OK) {
		chg_info("check vac_acdrv OK\n");
		return true;
	} else if (checksum == FW_CHECKSUM_FAIL) {
		sc96257_track_upload_wls_ic_err_info(chip, WLS_ERR_SCENE_RX, WLS_IC_ERR_VAC_ACDRV);
		chg_err("check vac_acdrv fail\n");
		return false;
	}
	return true;
}

static void sc96257_check_ldo_on_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_sc96257 *chip =
		container_of(dwork, struct oplus_sc96257, check_ldo_on_work);
	int iout = 0;
	bool connected = false;

	chg_info("connected_ldo_on = %s\n", chip->connected_ldo_on ? "true" : "false");
	sc96257_rx_is_connected(chip->ic_dev, &connected);
	if ((!chip->connected_ldo_on) && connected) {
		chg_err("Connect but no ldo on event irq, check again.\n");
		sc96257_get_iout(chip->ic_dev, &iout);
		if (iout >= LDO_ON_MA) {
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ONLINE);
			chip->connected_ldo_on = true;
		} else {
			schedule_delayed_work(&chip->check_ldo_on_work, SC96257_CHECK_LDO_ON_DELAY);
		}
	}
}

static void sc96257_rx_mode_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_sc96257 *chip =
		container_of(dwork, struct oplus_sc96257, rx_mode_work);
	int i = 0;

	chg_info("rx_mode = %d, recover rx_enable:%d\n", chip->rx_mode, chip->rx_en_status);
	if (chip->rx_mode == OPLUS_CHG_WLS_RX_MODE_BPP) {
		for (i = 0; i < RX_MODE_BPP_COUNT; i++) {
			sc96257_set_rx_enable_raw(chip->ic_dev, false);
			msleep(30);
			if (chip->rx_mode == OPLUS_CHG_WLS_RX_MODE_UNKNOWN) {
				sc96257_set_rx_enable_raw(chip->ic_dev, chip->rx_en_status);
				chg_info("rx_mode force exit BPP! count:%d\n", i);
				return;
			}
			sc96257_set_rx_enable_raw(chip->ic_dev, true);
			msleep(20);
			if (chip->rx_mode == OPLUS_CHG_WLS_RX_MODE_UNKNOWN) {
				sc96257_set_rx_enable_raw(chip->ic_dev, chip->rx_en_status);
				chg_info("rx_mode force exit BPP! count:%d\n", i);
				return;
			}
		}
	}
	sc96257_set_rx_enable_raw(chip->ic_dev, chip->rx_en_status);
	chg_info("rx_mode:%d, gpio_val:%d, count:%d\n",
		chip->rx_mode, gpio_get_value(chip->rx_en_gpio), i);
	return;
}

static void sc96257_event_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_sc96257 *chip = container_of(dwork, struct oplus_sc96257, event_work);

	if (!chip->ic_dev->online) {
		chg_info("ic is not online\n");
		return;
	}

	if (sc96257_wait_resume(chip) < 0)
		return;

	if (chip->rx_connected == true || sc96257_get_wls_type(chip) == OPLUS_CHG_WLS_TRX || chip->standby_config)
		sc96257_event_process(chip);
}

static void sc96257_connect_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_sc96257 *chip =
		container_of(dwork, struct oplus_sc96257, connect_work);
	bool connected = false;
	bool pre_connected = false;

	if (!chip->ic_dev->online) {
		chg_info("ic is not online\n");
		return;
	}
	if (sc96257_wait_resume(chip) < 0)
		return;
	sc96257_rx_is_connected(chip->ic_dev, &pre_connected);
retry:
	reinit_completion(&chip->ldo_on);
	(void)wait_for_completion_timeout(&chip->ldo_on, msecs_to_jiffies(50));
	sc96257_rx_is_connected(chip->ic_dev, &connected);
	if (connected != pre_connected) {
		pre_connected = connected;
		chg_err("retry to check connect\n");
		goto retry;
	}
	if (chip->rx_connected != connected)
		chg_err("!!!!! rx_connected[%d] -> con_gpio[%d]\n", chip->rx_connected, connected);
	if (connected && chip->rx_connected == false) {
		if (sc96257_vac_acdrv_check(chip) == false) {
			chip->event_code = WLS_EVENT_FORCE_UPGRADE;
			oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_EVENT_CHANGED);
		}
		chip->rx_connected = true;
		chip->connected_ldo_on = false;
		if (sc96257_get_mode_sw_active(chip))
			sc96257_set_mode_sw_default(chip);
		cancel_delayed_work_sync(&chip->check_ldo_on_work);
		schedule_delayed_work(&chip->check_ldo_on_work, SC96257_CHECK_LDO_ON_DELAY);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_PRESENT);
		if (sc96257_get_running_mode(chip) == SC96257_RX_MODE_EPP) {
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

static irqreturn_t sc96257_event_handler(int irq, void *dev_id)
{
	struct oplus_sc96257 *chip = dev_id;

	chg_info("!!!event irq\n");
	schedule_delayed_work(&chip->event_work, 0);
	return IRQ_HANDLED;
}

static irqreturn_t sc96257_connect_handler(int irq, void *dev_id)
{
	struct oplus_sc96257 *chip = dev_id;

	chg_info("!!!connect irq\n");
	schedule_delayed_work(&chip->connect_work, 0);
	return IRQ_HANDLED;
}

static int sc96257_gpio_init(struct oplus_sc96257 *chip)
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
	rc = devm_request_irq(chip->dev, chip->rx_event_irq, sc96257_event_handler,
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
	rc = devm_request_irq(chip->dev, chip->rx_con_irq, sc96257_connect_handler,
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

static void sc96257_shutdown(struct i2c_client *client)
{
	struct oplus_sc96257 *chip = i2c_get_clientdata(client);
	int wait_wpc_disconn_cnt = 0;
	bool is_connected = false;

	disable_irq(chip->rx_con_irq);
	disable_irq(chip->rx_event_irq);

	/*set TX_EN=0 when shutdown*/
	if (sc96257_get_wls_type(chip) == OPLUS_CHG_WLS_TRX)
		sc96257_set_tx_start(chip->ic_dev, OPLUS_CHG_WLS_TX_STOP);

	sc96257_rx_is_connected(chip->ic_dev, &is_connected);
	if (is_connected &&
	    (sc96257_get_wls_type(chip) == OPLUS_CHG_WLS_VOOC ||
	     sc96257_get_wls_type(chip) == OPLUS_CHG_WLS_SVOOC ||
	     sc96257_get_wls_type(chip) == OPLUS_CHG_WLS_PD_65W)) {
		sc96257_set_silent(chip->ic_dev);
		sc96257_set_rx_enable_raw(chip->ic_dev, false);
		msleep(100);
		while (wait_wpc_disconn_cnt < 10) {
			sc96257_rx_is_connected(chip->ic_dev, &is_connected);
			if (!is_connected)
				break;
			msleep(150);
			wait_wpc_disconn_cnt++;
		}
	}
	return;
}

static struct regmap_config sc96257_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

static int sc96257_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (ic_dev->online)
		return 0;
	ic_dev->online = true;
	if (g_sc96257_chip) {
		schedule_delayed_work(&g_sc96257_chip->connect_work, 0);
		schedule_delayed_work(&g_sc96257_chip->event_work, msecs_to_jiffies(500));
	}
	return 0;
}

static int sc96257_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;
	ic_dev->online = false;

	return 0;
}

static int sc96257_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int sc96257_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
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
			sc96257_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
			sc96257_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
			sc96257_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
			sc96257_smt_test);
		break;
	case OPLUS_IC_FUNC_RX_SET_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_ENABLE,
			sc96257_set_rx_enable);
		break;
	case OPLUS_IC_FUNC_RX_IS_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_IS_ENABLE,
			sc96257_rx_is_enable);
		break;
	case OPLUS_IC_FUNC_RX_IS_CONNECTED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_IS_CONNECTED,
			sc96257_rx_is_connected);
		break;
	case OPLUS_IC_FUNC_RX_GET_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_VOUT,
			sc96257_get_vout);
		break;
	case OPLUS_IC_FUNC_RX_SET_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_VOUT,
			sc96257_set_vout);
		break;
	case OPLUS_IC_FUNC_RX_GET_VRECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_VRECT,
			sc96257_get_vrect);
		break;
	case OPLUS_IC_FUNC_RX_GET_IOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_IOUT,
			sc96257_get_iout);
		break;
	case OPLUS_IC_FUNC_RX_GET_TRX_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TRX_VOL,
			sc96257_get_tx_vout);
		break;
	case OPLUS_IC_FUNC_RX_GET_TRX_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TRX_CURR,
			sc96257_get_tx_iout);
		break;
	case OPLUS_IC_FUNC_RX_GET_CEP_COUNT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_CEP_COUNT,
			sc96257_get_cep_count);
		break;
	case OPLUS_IC_FUNC_RX_GET_CEP_VAL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_CEP_VAL,
			sc96257_get_cep_val);
		break;
	case OPLUS_IC_FUNC_RX_GET_WORK_FREQ:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_WORK_FREQ,
			sc96257_get_work_freq);
		break;
	case OPLUS_IC_FUNC_RX_GET_RX_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_RX_MODE,
			sc96257_get_rx_mode);
		break;
	case OPLUS_IC_FUNC_RX_SET_RX_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_RX_MODE,
			sc96257_set_rx_mode);
		break;
	case OPLUS_IC_FUNC_RX_SET_DCDC_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_DCDC_ENABLE,
			sc96257_set_dcdc_enable);
		break;
	case OPLUS_IC_FUNC_RX_SET_TRX_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_TRX_ENABLE,
			sc96257_set_tx_enable);
		break;
	case OPLUS_IC_FUNC_RX_SET_TRX_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_TRX_START,
			sc96257_set_tx_start);
		break;
	case OPLUS_IC_FUNC_RX_GET_TRX_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TRX_STATUS,
			sc96257_get_tx_status);
		break;
	case OPLUS_IC_FUNC_RX_GET_TRX_ERR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TRX_ERR,
			sc96257_get_tx_err);
		break;
	case OPLUS_IC_FUNC_RX_GET_HEADROOM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_HEADROOM,
			sc96257_get_headroom);
		break;
	case OPLUS_IC_FUNC_RX_SET_HEADROOM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_HEADROOM,
			sc96257_set_headroom);
		break;
	case OPLUS_IC_FUNC_RX_SEND_MATCH_Q:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SEND_MATCH_Q,
			sc96257_send_match_q);
		break;
	case OPLUS_IC_FUNC_RX_SET_FOD_PARM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_FOD_PARM,
			sc96257_set_fod_parm);
		break;
	case OPLUS_IC_FUNC_RX_SEND_MSG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SEND_MSG,
			sc96257_send_msg);
		break;
	case OPLUS_IC_FUNC_RX_REG_MSG_CALLBACK:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_REG_MSG_CALLBACK,
			sc96257_register_msg_callback);
		break;
	case OPLUS_IC_FUNC_RX_GET_RX_VERSION:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_RX_VERSION,
			sc96257_get_rx_version);
		break;
	case OPLUS_IC_FUNC_RX_GET_TRX_VERSION:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TRX_VERSION,
			sc96257_get_tx_version);
		break;
	case OPLUS_IC_FUNC_RX_UPGRADE_FW_BY_BUF:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_UPGRADE_FW_BY_BUF,
			sc96257_upgrade_firmware_by_buf);
		break;
	case OPLUS_IC_FUNC_RX_UPGRADE_FW_BY_IMG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_UPGRADE_FW_BY_IMG,
			sc96257_upgrade_firmware_by_img);
		break;
	case OPLUS_IC_FUNC_RX_CHECK_CONNECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_CHECK_CONNECT,
			sc96257_connect_check);
		break;
	case OPLUS_IC_FUNC_RX_GET_EVENT_CODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_EVENT_CODE,
			sc96257_get_event_code);
		break;
	case OPLUS_IC_FUNC_RX_GET_BRIDGE_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_BRIDGE_MODE,
			sc96257_get_bridge_mode);
		break;
	case OPLUS_IC_FUNC_RX_DIS_INSERT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_DIS_INSERT,
			sc96257_set_insert_disable);
		break;
	case OPLUS_IC_FUNC_RX_STANDBY_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_STANDBY_CONFIG,
			sc96257_standby_config);
		break;
	case OPLUS_IC_FUNC_RX_SET_COMU:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_COMU,
			sc96257_set_rx_comu);
		break;
	case OPLUS_IC_FUNC_RX_GET_TX_ID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_TX_ID,
			sc96257_get_tx_id);
		break;
	case OPLUS_IC_FUNC_RX_SET_SILENT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SET_SILENT,
			sc96257_set_silent);
		break;
	case OPLUS_IC_FUNC_RX_SEND_EPP_MATCH_Q:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_SEND_EPP_MATCH_Q,
			sc96257_epp_send_match_q);
		break;
	case OPLUS_IC_FUNC_RX_GET_BLE_MAC_ADDR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_BLE_MAC_ADDR,
			sc96257_get_ble_mac_addr);
		break;
	case OPLUS_IC_FUNC_RX_GET_WLSPEN_ID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_WLSPEN_ID,
			sc96257_get_wlspen_id);
		break;
	case OPLUS_IC_FUNC_RX_GET_WLSPEN_CHG_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_WLSPEN_CHG_STATUS,
			sc96257_get_wlspen_chg_status);
		break;
	case OPLUS_IC_FUNC_RX_GET_AC_OV_FLAG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RX_GET_AC_OV_FLAG,
			sc96257_get_ac_ov_flag);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq sc96257_rx_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_PRESENT },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{ .virq_id = OPLUS_IC_VIRQ_EVENT_CHANGED },
};

static int sc96257_cp_init(struct oplus_chg_ic_dev *ic_dev)
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

static int sc96257_cp_exit(struct oplus_chg_ic_dev *ic_dev)
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

static int sc96257_cp_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int sc96257_cp_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

__maybe_unused static int sc96257_get_v2x_state(struct oplus_chg_ic_dev *dev, u32 *status)
{
	struct oplus_sc96257 *chip;
	int rc;
	u16 addr;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	addr = (u16)ADDR(struct rx_cust_type, v2x_state);
	rc = sc96257_read_block(chip, addr, (u8 *)status, sizeof(chip->info.rx_info.v2x_state));
	if (rc < 0) {
		chg_err("read v2x_state err, rc=%d\n", rc);
		return rc;
	}
	/*chg_info("read v2x_state: [0x%x]\n", *status);*/

	return 0;
}

static int sc96257_set_direct_charge_enable(struct oplus_chg_ic_dev *dev, bool en)
{
	struct oplus_sc96257 *chip;
	int rc;
	u32 v2x_status = 0;

	if (dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(dev);

	chg_info("<~WPC~> set v2x_on:%d\n", en);
	if (en)
		rc = sc96257_rx_set_cmd(chip, AP_CMD_RX_V2X_ON);
	else
		rc = sc96257_rx_set_cmd(chip, AP_CMD_RX_V2X_OFF);
	if (rc < 0) {
		chg_err("set v2x_on err, rc=%d\n", rc);
		return rc;
	}
	if (en) {
		msleep(50);
		(void)sc96257_get_v2x_state(dev, &v2x_status);
		chg_info("<~WPC~> v2x status:0x%x.\n", v2x_status);
		if ((v2x_status & V2X_STATE_ON) != V2X_STATE_ON)
			return -EAGAIN;
	}

	return 0;
}

static int sc96257_cp_start_chg(struct oplus_chg_ic_dev *dev, bool start)
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
			sc96257_cp_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
			sc96257_cp_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
			sc96257_cp_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
			sc96257_cp_smt_test);
		break;
	case OPLUS_IC_FUNC_CP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_ENABLE,
			sc96257_set_direct_charge_enable);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START,
			sc96257_cp_start_chg);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq sc96257_cp_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sc96257_driver_probe(struct i2c_client *client)
#else
static int sc96257_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
#endif
{
	struct oplus_sc96257 *chip;
	struct device_node *node = client->dev.of_node;
	struct device_node *child_node;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int rc = 0;

	chg_info("call !\n");
	chip = devm_kzalloc(&client->dev, sizeof(struct oplus_sc96257), GFP_KERNEL);
	if (!chip) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}

	chip->dev = &client->dev;
	chip->regmap = devm_regmap_init_i2c(client, &sc96257_regmap_config);
	if (!chip->regmap)
		return -ENODEV;
	chip->client = client;
	i2c_set_clientdata(client, chip);

	chip->support_epp_11w = of_property_read_bool(node, "oplus,support_epp_11w");

	chip->fw_data = (char *)of_get_property(node, "oplus,fw_data", &chip->fw_length);
	if (!chip->fw_data) {
		chg_err("parse fw data failed\n");
		chip->fw_data = sc96257_fw_data;
		chip->fw_length = sizeof(sc96257_fw_data);
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
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "rx-sc96257");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.virq_data = sc96257_rx_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(sc96257_rx_virq_table);
			ic_cfg.get_func = oplus_chg_rx_get_func;
			break;
		case OPLUS_CHG_IC_CP:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-sc96257");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.virq_data = sc96257_cp_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(sc96257_cp_virq_table);
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

	rc = sc96257_gpio_init(chip);
	if (rc < 0) {
		chg_err("sc96257 gpio init error.\n");
		goto gpio_init_err;
	}

	sc96257_track_debugfs_init(chip);

	device_init_wakeup(chip->dev, true);

	INIT_DELAYED_WORK(&chip->event_work, sc96257_event_work);
	INIT_DELAYED_WORK(&chip->connect_work, sc96257_connect_work);
	INIT_DELAYED_WORK(&chip->check_ldo_on_work, sc96257_check_ldo_on_work);
	INIT_DELAYED_WORK(&chip->rx_mode_work, sc96257_rx_mode_work);
	mutex_init(&chip->i2c_lock);
	mutex_init(&chip->pinctrl_lock);
	init_completion(&chip->ldo_on);
	init_completion(&chip->resume_ack);
	complete_all(&chip->resume_ack);
	g_sc96257_chip = chip;
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
static int sc96257_driver_remove(struct i2c_client *client)
#else
static void sc96257_driver_remove(struct i2c_client *client)
#endif
{
	struct oplus_sc96257 *chip = i2c_get_clientdata(client);

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

static int sc96257_pm_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct oplus_sc96257 *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("oplus_sc96257 is null\n");
		return 0;
	}

	complete_all(&chip->resume_ack);
	return 0;
}

static int sc96257_pm_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct oplus_sc96257 *chip = i2c_get_clientdata(client);

	if (!chip) {
		chg_err("oplus_sc96257 is null\n");
		return 0;
	}

	reinit_completion(&chip->resume_ack);
	return 0;
}

static const struct dev_pm_ops sc96257_pm_ops = {
	.resume = sc96257_pm_resume,
	.suspend = sc96257_pm_suspend,
};

static const struct of_device_id sc96257_match[] = {
	{ .compatible = "oplus,rx-sc96257" },
	{},
};

static const struct i2c_device_id sc96257_id_table[] = {
	{ "oplus,rx-sc96257", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sc96257_id_table);

static struct i2c_driver sc96257_driver = {
	.driver = {
		.name = "rx-sc96257",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sc96257_match),
		.pm = &sc96257_pm_ops,
	},
	.probe = sc96257_driver_probe,
	.remove = sc96257_driver_remove,
	.shutdown = sc96257_shutdown,
	.id_table = sc96257_id_table,
};

static __init int sc96257_driver_init(void)
{
	return i2c_add_driver(&sc96257_driver);
}

static __exit void sc96257_driver_exit(void)
{
	i2c_del_driver(&sc96257_driver);
}

oplus_chg_module_register(sc96257_driver);
