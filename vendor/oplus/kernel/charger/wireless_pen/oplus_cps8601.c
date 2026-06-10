// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/jiffies.h>

#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/pm_wakeup.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/firmware.h>
#include <uapi/linux/time.h>

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/oplus_project.h>
#endif /* CONFIG_DISABLE_OPLUS_FUNCTION*/

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <drm/drm_panel.h>
#endif

#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG)
#include <linux/mtk_panel_ext.h>
#include <linux/mtk_disp_notify.h>
#endif

#include "oplus_wireless_pen_glink.h"
#include "oplus_cps8601.h"
#include "oplus_cps8601_bl.h"

#define FIRMWARE_VERSION_POS	0xC4
#define CPS_WLS_CHRG_DRV_NAME	"cps-wls-charger"
#define CPS_WLS_CHRG_PSY_NAME	"wireless"
#define CPS8601_CHIPID	0x8601

#define TX_PKG_LEN	11
#define POWER_EXPIRED_TIME_DEFAULT	120
#define FULL_SOC	100
#define CPS_PROGRAM_WAIT_MS	3000
#define TX_WAKEUP_WAIT_MS	2500
#define Q_VALI_WAIT_MS	5000

#define CPS_MSEC_PER_SEC	1000
#define FW_UPDATE_DELAY_SEC	30
#define LCD_REG_DELAY_SEC	5
#define CPS_OCP_THRESHOLD	500
#define CPS_OVP_THRESHOLD	12000
#define CPS_LVP_THRESHOLD	4000
#define CPS_FOD_THRESHOLD	400
#define SOC_THRESHOLD	90
#define DEFAULT_BOOST_VOLT_MV	5800
#define MAX_BOOST_VOLT_MV	14750
#define MIN_BOOST_VOLT_MV	2000
#define MAX_RETRY	5
#define BOOST_SET_RETRY	60
#define BOOST_SET_DELAY_500MS	500
#define BOOST_WORK_MAX_RETRY	10
#define MAX_FW_NAME_LENGTH	128
#define IS_SUPPORT_BY_HBOOST	1

#define VALUE_OCP_MIN	0
#define VALUE_OCP_MAX	3000
#define VALUE_UVP_MIN	0
#define VALUE_UVP_MAX	5000
#define VALUE_OVP_MIN	3000
#define VALUE_OVP_MAX	20000

int wls_pen_log_level = CPS_LOG_ERR;
module_param(wls_pen_log_level, int, 0644);
MODULE_PARM_DESC(wls_pen_log_level, "wireless_pen log level");

struct cps_wls_chrg_chip {
	struct i2c_client *client;
	struct device *dev;
	struct device *wireless_dev;
	struct regmap *regmap;
	struct regmap *regmap32;
	char *name;

	struct power_supply *wl_psy;
	struct power_supply_desc wl_psd;
	struct power_supply_config wl_psc;

	int wls_charge_int;
	int cps_wls_irq;
	int wls_off_state_gpio;
	int wls_sleep_gpio;
	int wls_scan_gpio;
	int wls_sw_en_gpio;
	int wls_boost_en_gpio;

	struct pinctrl *cps_pinctrl;
	struct pinctrl_state *wls_charge_int_default;
	struct pinctrl_state *wls_charge_int_active;
	struct pinctrl_state *wls_charge_int_sleep;

	struct pinctrl_state *wls_off_state_default;
	struct pinctrl_state *wls_off_state_active;
	struct pinctrl_state *wls_off_state_sleep;

	struct pinctrl_state *wls_sleep_default;
	struct pinctrl_state *wls_sleep_active;
	struct pinctrl_state *wls_sleep_sleep;

	struct pinctrl_state *wls_scan_default;
	struct pinctrl_state *wls_scan_active;
	struct pinctrl_state *wls_scan_sleep;

	struct pinctrl_state *wls_sw_en_default;
	struct pinctrl_state *wls_sw_en_active;
	struct pinctrl_state *wls_sw_en_sleep;

	struct pinctrl_state *wls_boost_en_default;
	struct pinctrl_state *wls_boost_en_active;
	struct pinctrl_state *wls_boost_en_sleep;

	struct delayed_work max_chg_time_check_work;
	struct delayed_work lcd_notify_reg_work;
	struct work_struct set_boost_work;
	struct work_struct fw_update_work;
	struct work_struct rechg_work;
	struct work_struct q_cali_work;
	struct work_struct error_attach_check_work;
	struct work_struct init_work;

	struct wakeup_source *cps_wls_wake_lock;
	struct wakeup_source *cps_iic_wake_lock;
	struct mutex irq_lock;
	struct mutex i2c_lock;
	struct mutex glk_lock;
	const struct firmware *firm_data_bin;
	char *firmware_data;
	int fw_data_length;
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
	struct drm_panel *active_panel;
	void *notifier_cookie;
#else
	struct notifier_block cps_fb_notify;
#endif

	int chip_id;
	int ovp_threshold;
	int ocp_threshold;
	int lvp_threshold;
	int fod_threshold;
	int power_expired_time;
	int pen_support_track;

	int soc_threshould;
	int rx_soc;
	int pen_soc;
	int tx_voltage;
	int tx_current;
	int tx_iin;
	int rx_irect;
	int rx_vrect;
	int rx_vout;

	int is_supply_by_hboost;
	int hboost_default_volt;
	int hboost_attach_volt;
	int hboost_volt;
	int hboost_status;
	bool led_on;
	bool tx_wakeup_flag;
	bool q_cali_int_flag;
	bool i2c_ready;
	uint8_t fw_update_status;
	uint64_t ble_mac_addr;
	uint64_t mac_check_data;

	uint64_t upto_ble_time;
	uint64_t tx_start_time;
	uint64_t ping_succ_time;
	uint16_t fw_version;
	uint8_t q_cali_status;
	struct q_cali_result cali_result;

	uint8_t pen_status;
	uint8_t pen_present;
	uint8_t charge_allow;
	uint32_t power_enable_time;
	uint8_t power_enable_reason;
	uint8_t power_disable_reason;
	uint8_t power_disable_count[PEN_OFF_REASON_MAX];
};

struct cps_wls_chrg_chip *g_chip = NULL;

static DECLARE_WAIT_QUEUE_HEAD(tx_wakeup_irq_waiter);
static DECLARE_WAIT_QUEUE_HEAD(q_cali_irq_waiter);
static DECLARE_WAIT_QUEUE_HEAD(i2c_waiter);
static unsigned long long g_verify_failed_cnt = 0;
static void cps_set_gpio_value(struct cps_wls_chrg_chip *chip, int gp_num, int value);
static void cps_set_charge_allow(struct cps_wls_chrg_chip *chip, bool allow);
static int cps_register_lcd_notify(struct cps_wls_chrg_chip *chip);
static int cps_wls_send_fsk_packet(uint8_t *data, uint8_t data_len);
static int cps_wls_write_word(int reg, int value);
static int cps_wls_read_word(int addr);
static int cps_get_cali_status(struct cps_wls_chrg_chip *chip);

/*define cps tx reg enum*/
enum {
	CPS_TX_REG_PPP_HEADER,
	CPS_TX_REG_PPP_COMMAND,
	CPS_TX_REG_PPP_DATA0,
	CPS_TX_REG_PPP_DATA1,
	CPS_TX_REG_PPP_DATA2,
	CPS_TX_REG_PPP_DATA3,
	CPS_TX_REG_PPP_DATA4,
	CPS_TX_REG_PPP_DATA5,
	CPS_TX_REG_PPP_DATA6,
	CPS_TX_REG_PPP_DATA7,
	CPS_TX_REG_PPP_DATA8,
	CPS_TX_REG_BC_HEADER,
	CPS_TX_REG_BC_COMMAND,
	CPS_TX_REG_BC_DATA0,
	CPS_TX_REG_BC_DATA1,
	CPS_TX_REG_BC_DATA2,
	CPS_TX_REG_BC_DATA3,
	CPS_TX_REG_BC_DATA4,
	CPS_TX_REG_BC_DATA5,
	CPS_TX_REG_BC_DATA6,
	CPS_TX_REG_BC_DATA7,
	CPS_TX_REG_BC_DATA8,
	CPS_TX_REG_CNT_VARIANCE,
	CPS_TX_REG_WIDTH_VARIANCE,
	CPS_TX_REG_CALI_TIMES,
	CPS_TX_REG_NO_RX_CNT,
	CPS_TX_REG_NO_RX_WIDTH,
	CPS_TX_REG_OCP_TH,
	CPS_TX_REG_UVP_TH,
	CPS_TX_REG_OVP_TH,
	CPS_TX_REG_FOP_MIN,
	CPS_TX_REG_FOP_MAX,
	CPS_TX_REG_PING_FREQ,
	CPS_TX_REG_PING_DUTY,
	CPS_TX_REG_PING_TIME,
	CPS_TX_REG_PING_INTERVAL,
	CPS_TX_REG_FOD0_TH,
	CPS_TX_REG_FOP_VAL,
	CPS_TX_REG_ADC_VIN,
	CPS_TX_REG_ADC_VRECT,
	CPS_TX_REG_ADC_I_IN,
	CPS_TX_REG_EPT_RSN,
	CPS_TX_REG_ADC_DIE_TEMP,
	CPS_TX_REG_EPT_CODE,
	CPS_TX_REG_MAX
};

enum {
	CPS_COMM_REG_CHIP_ID,
	CPS_COMM_REG_FW_VER,
	CPS_COMM_REG_SYS_MODE,
	CPS_COMM_REG_INT_EN,
	CPS_COMM_REG_INT_FLAG,
	CPS_COMM_REG_INT_CLR,
	CPS_COMM_REG_CMD,
	CPS_COMN_REG_Q_CALI_CNT,
	CPS_COMN_REG_Q_CALI_WIDTH,
	CPS_COMM_REG_Q_CALI_RES,
	CPS_COMM_REG_Q_CALI_MODE,
	CPS_COMM_REG_MAX
};

struct cps_reg_s {
	uint16_t reg_name;
	uint16_t reg_bytes_len;
	uint32_t reg_addr;
};

struct cps_reg_s cps_comm_reg[CPS_COMM_REG_MAX] = {
	/* reg name bytes number reg address */
	{CPS_COMM_REG_CHIP_ID, 2, 0x0000},
	{CPS_COMM_REG_FW_VER, 2, 0x0002},
	{CPS_COMM_REG_SYS_MODE, 1, 0x0004},
	{CPS_COMM_REG_INT_EN, 2, 0x0005},
	{CPS_COMM_REG_INT_FLAG, 2, 0x0007},
	{CPS_COMM_REG_INT_CLR, 2, 0x0009},
	{CPS_COMM_REG_CMD, 1, 0x000B},
	{CPS_COMN_REG_Q_CALI_CNT, 1, 0x000D},
	{CPS_COMN_REG_Q_CALI_WIDTH, 2, 0x000E},
	{CPS_COMM_REG_Q_CALI_RES, 1, 0x0013},
	{CPS_COMM_REG_Q_CALI_MODE, 1, 0x0015},
};

struct cps_reg_s cps_tx_reg[CPS_TX_REG_MAX] = {
	/* reg name bytes number reg address */
	{CPS_TX_REG_PPP_HEADER, 1, 0x0040},
	{CPS_TX_REG_PPP_COMMAND, 1, 0x0041},
	{CPS_TX_REG_PPP_DATA0, 1, 0x0042},
	{CPS_TX_REG_PPP_DATA1, 1, 0x0043},
	{CPS_TX_REG_PPP_DATA2, 1, 0x0044},
	{CPS_TX_REG_PPP_DATA3, 1, 0x0045},
	{CPS_TX_REG_PPP_DATA4, 1, 0x0046},
	{CPS_TX_REG_PPP_DATA5, 1, 0x0047},
	{CPS_TX_REG_PPP_DATA6, 1, 0x0048},
	{CPS_TX_REG_PPP_DATA7, 1, 0x0049},
	{CPS_TX_REG_PPP_DATA8, 1, 0x004a},
	{CPS_TX_REG_BC_HEADER, 1, 0x004b},
	{CPS_TX_REG_BC_COMMAND, 1, 0x004c},
	{CPS_TX_REG_BC_DATA0, 1, 0x004d},
	{CPS_TX_REG_BC_DATA1, 1, 0x004e},
	{CPS_TX_REG_BC_DATA2, 1, 0x004f},
	{CPS_TX_REG_BC_DATA3, 1, 0x0050},
	{CPS_TX_REG_BC_DATA4, 1, 0x0051},
	{CPS_TX_REG_BC_DATA5, 1, 0x0052},
	{CPS_TX_REG_BC_DATA6, 1, 0x0053},
	{CPS_TX_REG_BC_DATA7, 1, 0x0054},
	{CPS_TX_REG_BC_DATA8, 1, 0x0055},
	{CPS_TX_REG_CNT_VARIANCE, 2, 0x0056},
	{CPS_TX_REG_WIDTH_VARIANCE, 2, 0x0058},
	{CPS_TX_REG_CALI_TIMES, 1, 0x005A},
	{CPS_TX_REG_NO_RX_CNT, 1, 0x0010},
	{CPS_TX_REG_NO_RX_WIDTH, 2, 0x0011},
	{CPS_TX_REG_OCP_TH, 2, 0x0028},
	{CPS_TX_REG_UVP_TH, 2, 0x002C},
	{CPS_TX_REG_OVP_TH, 2, 0x002A},
	{CPS_TX_REG_FOP_MIN, 2, 0x0020},
	{CPS_TX_REG_FOP_MAX, 2, 0x0022},
	{CPS_TX_REG_PING_FREQ, 2, 0x0024},
	{CPS_TX_REG_PING_DUTY, 2, 0x001E},
	{CPS_TX_REG_PING_TIME, 1, 0x001B},
	{CPS_TX_REG_PING_INTERVAL, 2, 0x001C},
	{CPS_TX_REG_FOD0_TH, 2, 0x002F},
	{CPS_TX_REG_FOP_VAL, 2, 0x0032},
	{CPS_TX_REG_ADC_VIN, 2, 0x0034},
	{CPS_TX_REG_ADC_VRECT, 2, 0x0036},
	{CPS_TX_REG_ADC_I_IN, 2, 0x0038},
	{CPS_TX_REG_EPT_RSN, 2, 0x003E},
	{CPS_TX_REG_ADC_DIE_TEMP, 1, 0x003A},
	{CPS_TX_REG_EPT_CODE, 1, 0x003B},
};

static const struct regmap_config cps8601_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static const struct regmap_config cps8601_regmap32_config = {
	.reg_bits = 32,
	.val_bits = 8,
};

static int cps_set_boost_volt_mv(int volt_mv)
{
	uint8_t reg_value = 0;
	int volt_temp = volt_mv;

	if (volt_temp > MAX_BOOST_VOLT_MV)
		volt_temp = MAX_BOOST_VOLT_MV;
	else if (volt_temp < MIN_BOOST_VOLT_MV)
		volt_temp = MIN_BOOST_VOLT_MV;

	reg_value = (uint8_t)((volt_temp - 2000) / 50);
	return wireless_pen_send_hboost_volt_req(reg_value);
}

static inline void do_gettimeofday(struct timeval *tv)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_usec = now.tv_nsec / 1000;
}

static int cps_wls_l_write_reg(int reg, int value)
{
	int ret;

	if (!g_chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}

	mutex_lock(&g_chip->i2c_lock);
	ret = regmap_write(g_chip->regmap, reg, value);
	mutex_unlock(&g_chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
		return CPS_WLS_FAIL;
	}

	return CPS_WLS_SUCCESS;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_l_read_reg(int reg)
{
	int ret;
	int value;

	if (!g_chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}
	mutex_lock(&g_chip->i2c_lock);
	ret = regmap_read(g_chip->regmap, reg, &value);
	mutex_unlock(&g_chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c read error!\n", __func__);
		return CPS_WLS_FAIL;
	}
	return value;
}
/*
return -1 means fail, 0 means success
*/
static int cps_wls_write_reg(int reg, int value, int byte_len)
{
	int i = 0, tmp = 0;

	for (i = 0; i < byte_len; i++) {
		tmp = (value >> (i * 8)) & 0xff;
		if (cps_wls_l_write_reg((reg & 0xffff) + i, tmp) == CPS_WLS_FAIL)
			return CPS_WLS_FAIL;
	}
	cps_wls_log(CPS_LOG_DEBG, "[%s]:[0x%x]=[0x%x]",
		__func__, (reg & 0xffff), value);
	return CPS_WLS_SUCCESS;
}

/*
return -1 means fail, 0 means success
*/
static int cps_wls_read_reg(int reg, int byte_len)
{
	int i = 0, tmp = 0, read_date = 0;
	for (i = 0; i < byte_len; i++) {
		tmp = cps_wls_l_read_reg((reg & 0xffff) + i);
		if (tmp == CPS_WLS_FAIL)
			return CPS_WLS_FAIL;
		read_date |= (tmp << (8 * i));
	}
	cps_wls_log(CPS_LOG_DEBG, "[%s]:[0x%x] = [0x%x]",
		__func__, reg, read_date);
	return read_date;
}

static int cps_wls_write_word(int reg, int value)
{
	int ret;
	u8 write_date[4];

	if (!g_chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}
	write_date[3] = (value >> 24) & 0xff;
	write_date[2] = (value >> 16) & 0xff;
	write_date[1] = (value >> 8) & 0xff;
	write_date[0] = value & 0xff;

	mutex_lock(&g_chip->i2c_lock);
	ret = regmap_raw_write(g_chip->regmap32, reg, write_date, 4);
	mutex_unlock(&g_chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c write 0x%x error!\n",
			__func__, reg);
		return CPS_WLS_FAIL;
	}
	return CPS_WLS_SUCCESS;
}

static int cps_wls_read_word(int addr)
{
	int ret;
	u8 read_date[4];

	if (!g_chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}
	mutex_lock(&g_chip->i2c_lock);
	ret = regmap_raw_read(g_chip->regmap32, addr, read_date, 4);
	mutex_unlock(&g_chip->i2c_lock);

	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] i2c read error!\n", __func__);
		return CPS_WLS_FAIL;
	}
	return *(int *)read_date;
}

#define SENDSZ 32
static int cps_wls_program_sram(int addr, uint8_t *date, int len)
{
	int ret = 0;
	if (!g_chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}
	while (len > SENDSZ) {
		mutex_lock(&g_chip->i2c_lock);
		ret = regmap_raw_write(g_chip->regmap32, addr, date, SENDSZ);
		mutex_unlock(&g_chip->i2c_lock);
		if (ret < 0) {
			cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
			return CPS_WLS_FAIL;
		}
		date = date + SENDSZ;
		addr = addr + SENDSZ;
		len = len - SENDSZ;
	}
	if (len > 0) {
		mutex_lock(&g_chip->i2c_lock);
		ret = regmap_raw_write(g_chip->regmap32, addr, date, len);
		mutex_unlock(&g_chip->i2c_lock);

		if (ret < 0) {
			cps_wls_log(CPS_LOG_ERR, "[%s] i2c write error!\n", __func__);
			return CPS_WLS_FAIL;
		}
	}
	return CPS_WLS_SUCCESS;
}

static int cps_wls_program_cmd_send(int cmd)
{
	return cps_wls_write_word(ADDR_CMD, cmd);
}

static int cps_wls_program_wait_cmd_done(void)
{
	int res;
	int cnt = 0;
	while (1) {
		res = cps_wls_read_word(ADDR_FLAG);
		if (res == CPS_WLS_FAIL)
			return CPS_WLS_FAIL;

		msleep(1); /* wait 1ms for program work*/
		res = res & 0xF0;
		switch (res) {
		case RUNNING:
			break;

		case PASS:
			break;

		case FAIL:
			cps_wls_log(CPS_LOG_ERR, "---> FAIL : %x\n", res);
			return res;

		case ILLEGAL:
			cps_wls_log(CPS_LOG_ERR, "---> ILLEGAL : %x\n", res);
			return res;

		default :
			cps_wls_log(CPS_LOG_ERR, "---> ERROR-CODE : %x\n", res);
			return res;
		}

		if (res == PASS)
			break;

		/*3s over time*/
		if ((cnt++) == CPS_PROGRAM_WAIT_MS) {
			cps_wls_log(CPS_LOG_ERR, "--->[%s] CMD-OVERTIME\n", __func__);
			break;
		}
	}

	return res;
}

static uint16_t get_crc(uint8_t *start, uint16_t len)
{
	uint16_t i, j;
	uint8_t *addr;
	uint16_t crc_in = 0x0000;
	uint16_t crc_poly = 0x1021;

	addr = start;
	for (i = 0; i < len; i++) {
		crc_in ^= (*(addr + i) << 8);
		for (j = 0; j < 8; j++) {
			if (crc_in & 0x8000)
				crc_in = (crc_in << 1) ^ crc_poly;
			else
				crc_in = crc_in << 1;
		}
	}

	return crc_in;
}

static int cps_load_bootloader(void)
{
	int result;
	uint32_t register_data = 0;
	uint16_t crc_cal = 0;

	/*Step1, load to sram*/
	/*enable 32bit i2c*/
	cps_wls_write_word(ADDR_EN_32BIT_IIC, CMD_EN_32BIT_IIC);
	msleep(110); /* wait for 32bit iic start*/
	/*set iic little endian*/
	cps_wls_write_word(ADDR_IIC_ENDIAN, CMD_IIC_LITTLE_ENDIAN);
	/*write password*/
	cps_wls_write_word(ADDR_PASSWORD, PASSWORD);
	/*reset and halt mcu*/
	cps_wls_write_word(ADDR_SYSTEM_CMD, CMD_RESET_HALT_MCU);
	/*disable dma*/
	cps_wls_write_word(ADDR_DISABLE_DMA, CMD_DISABLE_DMA);
	/*prevent watchdog counter overflow*/
	cps_wls_write_word(ADDR_WDOG_COUNT_OVER, CMD_WDOG_COUNT_OVER);
	/*set iic big endian*/
	cps_wls_write_word(ADDR_IIC_ENDIAN, CMD_IIC_BIG_ENDIAN);

	cps_wls_log(CPS_LOG_DEBG, "[%s] START LOAD SRAM HEX!\n", __func__);
	result = cps_wls_program_sram(ADDR_BOOTLOADER, cps8601_bl, BOOTLOADER_LEN);
	if (result != CPS_WLS_SUCCESS) {
		cps_wls_log(CPS_LOG_ERR, "[%s]  ---> BOOTLOADER CRC FAIL1\n", __func__);
		return CPS_WLS_FAIL;
	}

	/*set iic little endian*/
	cps_wls_write_word(ADDR_IIC_ENDIAN, CMD_IIC_LITTLE_ENDIAN);
	/*disable trimming load function*/
	cps_wls_write_word(ADDR_TRIMMING_LOAD, CMD_DIS_TRIM_LOAD);
	/*disable trimming load function*/
	cps_wls_write_word(ADDR_SYSTEM_CMD, CMD_DIS_TRIMMING_LOAD);
	msleep(10); /* wait for sys cmd work*/
	/*enable 32bit i2c*/
	cps_wls_write_word(ADDR_EN_32BIT_IIC, CMD_EN_32BIT_IIC);
	msleep(110); /* wait for 32bit iic start*/

	/*Step2, bootloader crc check */
	/*set iic little endian*/
	cps_wls_write_word(ADDR_IIC_ENDIAN, CMD_IIC_LITTLE_ENDIAN);
	/*write password*/
	cps_wls_write_word(ADDR_PASSWORD, PASSWORD);
	/*Write Bootloader code length to the SRAM*/
	cps_wls_write_word(ADDR_BUFFER0, BOOTLOADER_LEN);
	cps_wls_program_cmd_send(CACL_CRC_TEST);
	result = cps_wls_program_wait_cmd_done();

	if (result != PASS) {
		cps_wls_log(CPS_LOG_ERR, "[%s]-->BOOTLOADER CRC FAIL1\n", __func__);
		return CPS_WLS_FAIL;
	}
	crc_cal = get_crc((uint8_t *)cps8601_bl, BOOTLOADER_LEN);
	register_data  = cps_wls_read_word(ADDR_BUFFER0);
	if (register_data != crc_cal) {
		cps_wls_log(CPS_LOG_ERR, "[%s]-->BOOTLOADER CRC FAIL2\n", __func__);
		return CPS_WLS_FAIL;
	}
	cps_wls_log(CPS_LOG_DEBG, "[%s]-->LOAD BOOTLOADER SUCCESSFUL\n", __func__);
	return CPS_WLS_SUCCESS;
}

static int cps_load_firmware(struct cps_wls_chrg_chip *chip)
{
	int buf0_flag = 0, buf1_flag = 0;
	int addr, page_count;
	int ret, result, i;
	uint32_t register_data = 0;
	uint16_t crc_cal = 0;
	char *firmware = NULL;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}

	firmware = chip->firmware_data;
	if (!firmware) {
		cps_wls_log(CPS_LOG_ERR, "[%s]firmware null\n", __func__);
		return CPS_WLS_FAIL;
	}
	/*enable 32bit i2c*/
	cps_wls_write_word(ADDR_EN_32BIT_IIC, CMD_EN_32BIT_IIC);
	msleep(110); /* wait for 32bit iic start*/
	/*set iic little endian*/
	cps_wls_write_word(ADDR_IIC_ENDIAN, CMD_IIC_LITTLE_ENDIAN);
	/*erase mtp*/
	cps_wls_write_word(ADDR_ERASE_MTP, CMD_ERASE_MTP);
	msleep(10); /* wait for erase cmd work*/
	result = cps_wls_program_wait_cmd_done();
	if (result != PASS) {
		cps_wls_log(CPS_LOG_ERR, "[%s]  ---> ERASE MTP FAIL\n", __func__);
		return CPS_WLS_FAIL;
	}
	cps_wls_log(CPS_LOG_DEBG,  "[%s]  ---> START LOAD APP HEX\n", __func__);

	buf0_flag = 0;
	buf1_flag = 0;
	addr = 0;
	page_count = chip->fw_data_length / CONFIG_BUFF_SIZE;

	for (i = 0; i < page_count; i++) {
		if (buf0_flag == 0) {
			cps_wls_program_sram(ADDR_BUFFER0, firmware + addr, CONFIG_BUFF_SIZE);
			addr  = addr  + CONFIG_BUFF_SIZE;

			if (buf1_flag == 1) {
				result = cps_wls_program_wait_cmd_done();
				if (result != PASS) {
					cps_wls_log(CPS_LOG_ERR,
						"%s:--->WRITE BUFFER1 DATA TO MTP FAIL\n", __func__);
					return CPS_WLS_FAIL;
				}
				buf1_flag = 0;
			}
			cps_wls_program_cmd_send(PGM_BUFFER0);
			buf0_flag = 1;
			continue;
		}

		if (buf1_flag == 0) {
			cps_wls_program_sram(ADDR_BUFFER1, firmware + addr, CONFIG_BUFF_SIZE);
			addr  = addr  + CONFIG_BUFF_SIZE;

			if (buf0_flag == 1) {
				result =  cps_wls_program_wait_cmd_done();
				if (result != PASS) {
					cps_wls_log(CPS_LOG_ERR,
						"%s:--->WRITE BUFFER0 DATA TO MTP FAIL\n", __func__);
					return CPS_WLS_FAIL;
				}
				buf0_flag = 0;
			}
			cps_wls_program_cmd_send(PGM_BUFFER1);
			buf1_flag = 1;
			continue;
		}
	}

	if (buf0_flag == 1) {
		result = cps_wls_program_wait_cmd_done();
		if (result != PASS) {
			cps_wls_log(CPS_LOG_ERR,
				"%s:--->WRITE BUFFER0 DATA TO MTP FAIL\n", __func__);
			return CPS_WLS_FAIL;
		}
		buf0_flag = 0;
	}

	if (buf1_flag == 1) {
		result = cps_wls_program_wait_cmd_done();
		if (result != PASS) {
			cps_wls_log(CPS_LOG_ERR,
				"%s:--->WRITE BUFFER1 DATA TO MTP FAIL\n", __func__);
			return CPS_WLS_FAIL;
		}
		buf1_flag = 0;
	}
	cps_wls_log(CPS_LOG_ERR, "%s:--->LOAD APP HEX SUCCESSFUL\n", __func__);
	/*Step4, check app CRC*/
	/*Write app code length to the SRAM*/
	cps_wls_write_word(ADDR_BUFFER0, FIRMWARE_LEN);
	cps_wls_program_cmd_send(CACL_CRC_APP);

	result = cps_wls_program_wait_cmd_done();
	if (result != PASS) {
		cps_wls_log(CPS_LOG_ERR, "%s: ---> APP CRC FAIL\n", __func__);
		return CPS_WLS_FAIL;
	}

	crc_cal = get_crc((uint8_t *)firmware, FIRMWARE_LEN);
	register_data = cps_wls_read_word(ADDR_BUFFER0);
	if (register_data != crc_cal) {
		cps_wls_log(CPS_LOG_ERR, "%s: ---> APP CRC FAIL\n", __func__);
		return CPS_WLS_FAIL;
	}
	cps_wls_log(CPS_LOG_DEBG, "%s: ---> CHERK APP CRC SUCCESSFUL\n", __func__);
	/*Step5, write mcu start flag*/
	cps_wls_program_cmd_send(PGM_WR_FLAG);
	result = cps_wls_program_wait_cmd_done();
	if (result != PASS) {
		cps_wls_log(CPS_LOG_ERR, "%s:---> WRITE MCU START FLAG FAIL\n", __func__);
		return CPS_WLS_FAIL;
	}

	cps_wls_log(CPS_LOG_DEBG,
		"%s: ---> WRITE MCU START FLAG SUCCESSFUL\n", __func__);
	/*enable 32bit i2c*/
	ret = cps_wls_write_word(ADDR_EN_32BIT_IIC, CMD_EN_32BIT_IIC);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "enable 32bit i2c FAIL\n");
		return CPS_WLS_FAIL;
	}

	msleep(100); /* wait for 32bit iic start*/
	/*set iic little endian*/
	ret = cps_wls_write_word(ADDR_IIC_ENDIAN, CMD_IIC_LITTLE_ENDIAN);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "set iic little endian FAIL\n");
		return CPS_WLS_FAIL;
	}

	/*write password*/
	ret = cps_wls_write_word(ADDR_PASSWORD, PASSWORD);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "write password FAIL\n");
		return CPS_WLS_FAIL;
	}

	/*reset all system*/
	cps_wls_write_word(ADDR_SYSTEM_CMD, CMD_RESET_ALL_SYS);

	ret = cps_wls_write_word(ADDR_PASSWORD, 0x00000000);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "password set 0 fail\n");
		return CPS_WLS_FAIL;
	}

	ret = cps_wls_write_word(ADDR_IIC_ENDIAN, 0x00000000);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "iic endian set 0 fail\n");
		return CPS_WLS_FAIL;
	}

	ret = cps_wls_write_word(ADDR_EN_32BIT_IIC, 0x00000000);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR, "en 32bit set 0 fail\n");
		return CPS_WLS_FAIL;
	}

	return CPS_WLS_SUCCESS;
}

static int update_firmware(struct cps_wls_chrg_chip *chip)
{
	int ret;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}

	ret = cps_load_bootloader();
	if (ret != CPS_WLS_SUCCESS) {
		cps_wls_log(CPS_LOG_ERR, "[%s]load bootloader fail\n", __func__);
		return CPS_WLS_FAIL;
	}

	ret = cps_load_firmware(chip);
	if (ret != CPS_WLS_SUCCESS) {
		cps_wls_log(CPS_LOG_ERR, "[%s]load fw fail\n", __func__);
		return CPS_WLS_FAIL;
	}

	cps_wls_log(CPS_LOG_DEBG, "[%s]success\n", __func__);
	return CPS_WLS_SUCCESS;
}

static int cps_wls_set_cmd(int value)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_CMD]);
	return cps_wls_write_reg(cps_reg->reg_addr, value,
		(int)cps_reg->reg_bytes_len);
}

static uint16_t cps_wls_get_cmd(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_CMD]);
	return cps_wls_read_reg(cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_int_flag(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_INT_FLAG]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_int_clr(int value)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_INT_CLR]);
	return cps_wls_write_reg(cps_reg->reg_addr, value,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_chip_id(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_CHIP_ID]);
	return cps_wls_read_reg(cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_sys_fw_version(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_FW_VER]);
	return cps_wls_read_reg(cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_q_cali_count(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMN_REG_Q_CALI_CNT]);
	return cps_wls_read_reg(cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_q_cali_width(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMN_REG_Q_CALI_WIDTH]);
	return cps_wls_read_reg(cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_is_cali_success(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_Q_CALI_RES]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_cali_mode(bool cali_mode)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_Q_CALI_MODE]);
	if (cali_mode)
		return cps_wls_write_reg((int)cps_reg->reg_addr,
			ENTER_CALI_MODE, (int)cps_reg->reg_bytes_len);
	else
		return cps_wls_write_reg((int)cps_reg->reg_addr,
			EXIT_CALI_MODE, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_i_in(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_ADC_I_IN]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_vin(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_ADC_VIN]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}


static int cps_wls_set_tx_fod_thresh(int value)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_FOD0_TH]);
	return cps_wls_write_reg((int)cps_reg->reg_addr,
		value, (int)cps_reg->reg_bytes_len);
}


static int cps_wls_enable_tx_mode(void)
{
	uint16_t cmd;

	cmd = cps_wls_get_cmd();
	cmd |= TX_CMD_ENTER_TX_MODE;
	return cps_wls_set_cmd(cmd);
}

static int cps_wls_reset_sys(void)
{
	uint16_t cmd;

	cmd = cps_wls_get_cmd();
	cmd |= TX_CMD_RESET_SYS;
	return cps_wls_set_cmd(cmd);
}

static int cps_wls_disable_tx_mode(void)
{
	uint16_t cmd;

	cmd = cps_wls_get_cmd();
	cmd |= TX_CMD_EXIT_TX_MODE;
	return cps_wls_set_cmd(cmd);
}


static int cps_wls_start_q_cali(void)
{
	uint16_t cmd;

	cmd = cps_wls_get_cmd();
	cmd |= TX_CMD_START_Q_CAIL;
	return cps_wls_set_cmd(cmd);
}

static int cps_wls_get_cnt_variance(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_CNT_VARIANCE]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_width_variance(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_WIDTH_VARIANCE]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_cali_times(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_CALI_TIMES]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_no_rx_cnt(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_NO_RX_CNT]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_no_rx_width(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_NO_RX_WIDTH]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_tx_ept_type(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_EPT_RSN]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_get_ept_code(void)
{
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_EPT_CODE]);
	return cps_wls_read_reg((int)cps_reg->reg_addr,
		(int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_ocp_threshold(int value)
{
	struct cps_reg_s *cps_reg;

	if (value < VALUE_OCP_MIN || value > VALUE_OCP_MAX)
		return CPS_WLS_FAIL;
	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_OCP_TH]);
	return cps_wls_write_reg((int)cps_reg->reg_addr,
		value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_uvp_threshold(int value)
{
	struct cps_reg_s *cps_reg;

	if (value < VALUE_UVP_MIN || value > VALUE_UVP_MAX)
		return CPS_WLS_FAIL;
	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_UVP_TH]);
	return cps_wls_write_reg((int)cps_reg->reg_addr,
		value, (int)cps_reg->reg_bytes_len);
}

static int cps_wls_set_tx_ovp_threshold(int value)
{
	struct cps_reg_s *cps_reg;

	if (value < VALUE_OVP_MIN || value > VALUE_OVP_MAX)
		return CPS_WLS_FAIL;
	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_OVP_TH]);
	return cps_wls_write_reg((int)cps_reg->reg_addr,
		value, (int)cps_reg->reg_bytes_len);
}

static void cps_wls_hardware_reset(void)
{
	struct cps_wls_chrg_chip *chip = g_chip;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}
	cps_set_gpio_value(chip, GP_2, 0);
	msleep(50); /* wait for power down*/
	cps_set_gpio_value(chip, GP_2, 1);
	msleep(50);  /* wait for power up*/
}

static int cps_request_firmware(struct cps_wls_chrg_chip *chip)
{
	char *firmware_path = NULL;
	int retry = 0;
	int ret = 0;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}

	firmware_path = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
	if (firmware_path == NULL) {
		cps_wls_log(CPS_LOG_ERR, "[%s] firmware_path alloc fail\n", __func__);
		return CPS_WLS_FAIL;
	}
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	snprintf(firmware_path, MAX_FW_NAME_LENGTH,
		"wireless_pen/%d/cps8601_firmware.bin", get_project());
#else
	snprintf(firmware_path, MAX_FW_NAME_LENGTH,
		"wireless_pen/cps8601_firmware.bin");
#endif /* CONFIG_DISABLE_OPLUS_FUNCTION*/

	for (retry = 0; retry < MAX_RETRY; retry++) {
		ret = request_firmware(&chip->firm_data_bin, firmware_path, chip->dev);
		if (ret < 0)
			cps_wls_log(CPS_LOG_ERR, "[%s]Failed to request firmware retry %d\n",
				__func__, retry);
		else
			break;
	}

	if (chip->firm_data_bin != NULL) {
		chip->fw_data_length = (int)chip->firm_data_bin->size;
		chip->firmware_data = kmalloc(chip->fw_data_length, GFP_KERNEL);
		if (chip->firmware_data != NULL) {
			memset(chip->firmware_data, 0, chip->fw_data_length);
			memcpy(chip->firmware_data, chip->firm_data_bin->data,
				chip->fw_data_length);
		} else {
			cps_wls_log(CPS_LOG_ERR, "[%s] alloc firmware data fail\n", __func__);
			return CPS_WLS_FAIL;
		}
	} else {
		cps_wls_log(CPS_LOG_ERR, " request firmware fail\n");
		return CPS_WLS_FAIL;
	}
	cps_wls_log(CPS_LOG_ERR, "[%s] fw_data count = 0X%x\n",
		__func__, chip->fw_data_length);
	return CPS_WLS_SUCCESS;
}

static int cps_do_fw_update(struct cps_wls_chrg_chip *chip)
{
	int ret, retry = 0, sys_fw_ver = 0;
	uint8_t sys_fw_ver_h, sys_fw_ver_l, bin_fw_ver_h, bin_fw_ver_l;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}

	sys_fw_ver = cps_wls_get_sys_fw_version();
	chip->fw_version = sys_fw_ver;
	sys_fw_ver_l = sys_fw_ver & 0xFF;
	sys_fw_ver_h = (sys_fw_ver >> 8) & 0xFF;

	bin_fw_ver_h = chip->firmware_data[FIRMWARE_VERSION_POS];
	bin_fw_ver_l = chip->firmware_data[FIRMWARE_VERSION_POS + 1];

	cps_wls_log(CPS_LOG_ERR, "[%s]sys_fw_ver_h=0X%x,sys_fw_ver_l=0X%x\n",
		__func__, sys_fw_ver_h, sys_fw_ver_l);
	cps_wls_log(CPS_LOG_ERR, "[%s]bin_fw_ver_h=0X%x,bin_fw_ver_l=0X%x\n",
		__func__, bin_fw_ver_h, bin_fw_ver_l);

	if ((bin_fw_ver_h != sys_fw_ver_h) || (bin_fw_ver_l != sys_fw_ver_l)) {
		cps_wls_log(CPS_LOG_ERR, "[%s] new fw found,to update\n", __func__);
		for (retry = 0; retry < MAX_RETRY; retry++) {
			ret = update_firmware(chip);
			if (ret != CPS_WLS_SUCCESS)
				cps_wls_log(CPS_LOG_ERR,
					"[%s] update_firmware fail,retry = %d\n", __func__, retry);
			else {
				cps_wls_log(CPS_LOG_ERR,
					"[%s] update_firmware success\n", __func__);
				break;
			}
		}
		if (retry == MAX_RETRY)
			return CPS_WLS_FAIL;
	} else
		cps_wls_log(CPS_LOG_ERR, "[%s]firmware version same\n", __func__);
	return CPS_WLS_SUCCESS;
}

static int cps_update_fw(struct cps_wls_chrg_chip *chip)
{
	int ret = 0;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}

	chip->tx_wakeup_flag = false;
	cps_set_gpio_value(chip, GP_0, 1);
	wait_event_interruptible_timeout(tx_wakeup_irq_waiter,
		chip->tx_wakeup_flag, msecs_to_jiffies(TX_WAKEUP_WAIT_MS));

	if (chip->tx_wakeup_flag) {
		ret = cps_do_fw_update(chip);
		cps_set_gpio_value(chip, GP_0, 0);
		chip->tx_wakeup_flag = false;
		return ret;
	} else {
		cps_wls_log(CPS_LOG_ERR,
			"[%s] wait tx_wakeup timeout start download\n", __func__);
		ret = cps_do_fw_update(chip);
		cps_set_gpio_value(chip, GP_0, 0);
		chip->tx_wakeup_flag = false;
		return ret;
	}
}

static void fw_update_work_func(struct work_struct *work)
{
	struct cps_wls_chrg_chip *chip =
		container_of(work, struct cps_wls_chrg_chip, fw_update_work);
	int ret = 0;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}
	__pm_stay_awake(chip->cps_wls_wake_lock);
	chip->fw_update_status = FW_UPDATE_ON_GOING;
	ret = cps_request_firmware(chip);
	if (ret != CPS_WLS_SUCCESS) {
		cps_wls_log(CPS_LOG_ERR, "[%s] request firmware fail\n", __func__);
		chip->fw_update_status = FW_UPDATE_FAIL;
		__pm_relax(chip->cps_wls_wake_lock);
		return;
	}

	ret = cps_update_fw(chip);
	if (ret != CPS_WLS_SUCCESS) {
		cps_wls_log(CPS_LOG_ERR, "[%s] update firmware fail\n", __func__);
		chip->fw_update_status = FW_UPDATE_FAIL;
		__pm_relax(chip->cps_wls_wake_lock);
		return;
	}

	chip->fw_update_status = FW_UPDATE_SUCCESS;
	__pm_relax(chip->cps_wls_wake_lock);
}


static void rechg_work_func(struct work_struct *work)
{
	struct cps_wls_chrg_chip *chip =
		container_of(work, struct cps_wls_chrg_chip, rechg_work);
	struct timeval now_time;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}
	__pm_stay_awake(chip->cps_wls_wake_lock);

	if ((chip->rx_soc > chip->soc_threshould)
		|| (chip->pen_status != PEN_STATUS_NEAR)
		|| (false != chip->charge_allow)
		|| (PEN_REASON_CHARGE_FULL != chip->power_disable_reason)) {
		cps_wls_log(CPS_LOG_ERR, "enable failed for (%d %d %d %d).\n",
			chip->rx_soc, chip->pen_status, chip->charge_allow,
			chip->power_disable_reason);
		__pm_relax(chip->cps_wls_wake_lock);
		return;
	}

	do_gettimeofday(&now_time);
	chip->tx_start_time =
		now_time.tv_sec * 1000 + now_time.tv_usec / 1000;
	chip->power_enable_reason = PEN_REASON_RECHARGE;
	schedule_work(&chip->error_attach_check_work);
	cancel_delayed_work_sync(&chip->max_chg_time_check_work);
	cps_set_charge_allow(chip, true);
	schedule_delayed_work(&chip->max_chg_time_check_work,
		round_jiffies_relative(msecs_to_jiffies(
			chip->power_expired_time * MIN_TO_MS)));

	__pm_relax(chip->cps_wls_wake_lock);
}

static void q_cali_work_func(struct work_struct *work)
{
	struct cps_wls_chrg_chip *chip =
		container_of(work, struct cps_wls_chrg_chip, q_cali_work);

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}
	__pm_stay_awake(chip->cps_wls_wake_lock);

	chip->q_cali_status = Q_CALI_IN_PROGRESS;
	chip->tx_wakeup_flag = false;
	cps_set_gpio_value(chip, GP_0, 1);
	cps_wls_log(CPS_LOG_DEBG, "[%s]wait tx_wakeup irq\n", __func__);
	wait_event_interruptible_timeout(tx_wakeup_irq_waiter,
		chip->tx_wakeup_flag, msecs_to_jiffies(TX_WAKEUP_WAIT_MS));
	chip->tx_wakeup_flag = false;

	chip->q_cali_int_flag = false;
	cps_wls_set_cali_mode(true);
	cps_set_gpio_value(chip, GP_1, 1);
	cps_wls_start_q_cali();
	cps_wls_log(CPS_LOG_DEBG, "[%s]wait q_cali irq\n", __func__);
	wait_event_interruptible_timeout(q_cali_irq_waiter,
		chip->q_cali_int_flag, msecs_to_jiffies(Q_VALI_WAIT_MS));
	if (chip->q_cali_int_flag) {
		chip->q_cali_int_flag = false;

		if (1 == cps_wls_get_is_cali_success()) {
			if (cps_get_cali_status(chip) == CPS_WLS_SUCCESS)
				chip->q_cali_status = Q_CALI_SUCCESS;
			else {
				cps_wls_log(CPS_LOG_ERR, "[%s]read q_cali fail\n", __func__);
				chip->q_cali_status = Q_CALI_FAIL;
			}
		} else {
			cps_get_cali_status(chip);
			cps_wls_log(CPS_LOG_ERR, "[%s]get is success fail\n", __func__);
			chip->q_cali_status = Q_CALI_FAIL;
		}
	} else {
		cps_wls_log(CPS_LOG_ERR, "[%s] q cali timeout\n", __func__);
		chip->q_cali_status = Q_CALI_FAIL;
	}

	cps_wls_set_cali_mode(false);
	cps_set_gpio_value(chip, GP_0, 0);
	__pm_relax(chip->cps_wls_wake_lock);
}

static void power_expired_do_check(struct cps_wls_chrg_chip *chip)
{
	struct timeval now_time;
	uint64_t time_offset = 0;

	cps_wls_log(CPS_LOG_ERR, "[%s] [%d %d]\n",
		__func__, chip->charge_allow, chip->pen_present);
	if (chip->charge_allow && chip->pen_status == PEN_STATUS_NEAR) {
		do_gettimeofday(&now_time);
		time_offset = now_time.tv_sec * 1000 +
			now_time.tv_usec / 1000 - chip->tx_start_time;
		cps_wls_log(CPS_LOG_ERR, "[%s] time_offset = %lld\n",
			__func__, time_offset);
		if (time_offset > chip->power_expired_time * MIN_TO_MS) {
			cps_set_charge_allow(chip, false);
			chip->power_disable_reason = PEN_REASON_CHARGE_TIMEOUT;
		}
	}
}

static void max_chg_time_check_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct cps_wls_chrg_chip *chip =
		container_of(dwork, struct cps_wls_chrg_chip, max_chg_time_check_work);

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}
	__pm_stay_awake(chip->cps_wls_wake_lock);
	power_expired_do_check(chip);
	__pm_relax(chip->cps_wls_wake_lock);
}

#define LCD_REG_RETRY_COUNT_MAX  20
#define LCD_REG_RETRY_DELAY_MS   100
static void lcd_notify_reg_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct cps_wls_chrg_chip *chip =
		container_of(dwork, struct cps_wls_chrg_chip, lcd_notify_reg_work);
	static int retry_count = 0;
	int rc;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}

	if (retry_count >= LCD_REG_RETRY_COUNT_MAX)
		return;

	rc = cps_register_lcd_notify(chip);
	if (rc < 0) {
		if (rc != -EPROBE_DEFER) {
			cps_wls_log(CPS_LOG_ERR, "register lcd notify error, rc=%d\n", rc);
			return;
		}
		retry_count++;
		cps_wls_log(CPS_LOG_ERR, "lcd panel not ready, count=%d\n", retry_count);
		schedule_delayed_work(&chip->lcd_notify_reg_work,
					msecs_to_jiffies(LCD_REG_RETRY_DELAY_MS));
		return;
	}
}

static void set_boost_by_attach_status(bool attach, struct cps_wls_chrg_chip *chip)
{
	if (attach)
		chip->hboost_volt = chip->hboost_attach_volt;
	else
		chip->hboost_volt = chip->hboost_default_volt;

	cps_wls_log(CPS_LOG_ERR, " attach status:%d, hboost_volt:%d!\n",
						attach, chip->hboost_volt);
	schedule_work(&chip->set_boost_work);
}

static void set_boost_work_func(struct work_struct *work)
{
	struct cps_wls_chrg_chip *chip =
		container_of(work, struct cps_wls_chrg_chip, set_boost_work);
	int retry_count = BOOST_SET_RETRY;
	int rc;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}
	__pm_stay_awake(chip->cps_wls_wake_lock);

	if (chip->is_supply_by_hboost != IS_SUPPORT_BY_HBOOST) {
		cps_wls_log(CPS_LOG_ERR, "[%s]not use hboost\n", __func__);
		__pm_relax(chip->cps_wls_wake_lock);
		return;
	}
	chip->hboost_status = HBOOST_IS_SETTING_BOOST;
	while (retry_count > 0) {
		rc = cps_set_boost_volt_mv(chip->hboost_volt);
		if (rc != 0) {
			retry_count--;
			msleep(BOOST_SET_DELAY_500MS); /* try again after 500ms*/
			cps_wls_log(CPS_LOG_ERR, "[%s]set boost fail, retry = %d\n",
				__func__, retry_count);
			continue;
		} else {
			cps_set_charge_allow(chip, true);
			cps_wls_log(CPS_LOG_ERR, "[%s] set boost success\n", __func__);
			chip->hboost_status = HBOOST_SET_SUCCESS;
			break;
		}
	}
	if (retry_count <= 0)
		chip->hboost_status = HBOOST_SET_FAIL;
	__pm_relax(chip->cps_wls_wake_lock);
}

static void error_attach_check_work_func(struct work_struct *work)
{
	int count = 0;
	struct cps_wls_chrg_chip *chip =
		container_of(work, struct cps_wls_chrg_chip, error_attach_check_work);

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}

	__pm_stay_awake(chip->cps_wls_wake_lock);

	while ((count++ < P9418_WAIT_TIME) && (!chip->pen_present) && chip->charge_allow) {
		msleep(100); /* wait pen_present every 100ms*/
		cps_wls_log(CPS_LOG_ERR, "[%s] count %d\n", __func__, count);
		if (chip->pen_status == PEN_STATUS_FAR) {
			__pm_relax(chip->cps_wls_wake_lock);
			return;
		}
	}

	if (!chip->pen_present && chip->pen_status == PEN_STATUS_NEAR) {
		cps_wls_log(CPS_LOG_ERR, "[%s] set allow false %d\n", __func__, count);
		cps_set_charge_allow(chip, false);
	}

	__pm_relax(chip->cps_wls_wake_lock);
}

static int cps_wls_send_fsk_packet(uint8_t *data, uint8_t data_len)
{
	uint16_t cmd;
	uint8_t i;
	struct cps_reg_s *cps_reg;

	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_BC_HEADER]);


	for (i = 0; i < data_len; i++) {
		if (cps_wls_write_reg((int)(cps_reg->reg_addr + i), *(data + i), 1)
			== CPS_WLS_FAIL)
			return CPS_WLS_FAIL;
	}

	cmd = cps_wls_get_cmd();
	cmd |= TX_CMD_SEND_FSK;
	return cps_wls_set_cmd(cmd);
}

static void cps8601_send_uevent(struct device *dev, bool status, uint64_t mac_addr)
{
	char status_string[64], addr_string[64];
	char *envp[] = { status_string, addr_string, NULL };
	int ret = 0;

	sprintf(status_string, "pencil_status=%d", status);
	sprintf(addr_string, "pencil_addr=%llx", mac_addr);
	ret = kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
	if (ret)
		cps_wls_log(CPS_LOG_ERR, "%s: kobject_uevent_fail, ret = %d",
			__func__, ret);

	cps_wls_log(CPS_LOG_DEBG, "send uevent:%s, %s.\n", status_string, addr_string);
}

#define UEVENT_TRIGGER_COOLDOWN_TIME_MS (300)
static void cps8601_send_mislocated_uevent(struct device *dev)
{
	int ret;
	char *envp[] = { "pencil_mislocated=1", NULL };
	static unsigned long last_uevent_time_jiffies = 0;
	unsigned long current_uevent_time_jiffies = jiffies;

	if (current_uevent_time_jiffies - last_uevent_time_jiffies <=
		HZ * UEVENT_TRIGGER_COOLDOWN_TIME_MS / 1000) {
		last_uevent_time_jiffies = current_uevent_time_jiffies;
		return;
	}

	last_uevent_time_jiffies = current_uevent_time_jiffies;
	ret = kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
	if (ret)
		cps_wls_log(CPS_LOG_ERR, "%s: kobject_uevent_fail, ret = %d",
			__func__, ret);
	else
		cps_wls_log(CPS_LOG_ERR, "send mislocated uevent\n");
}

static uint64_t cps_recv_ble_mac_addr(uint8_t *data)
{
	uint64_t ble_addr;
	uint8_t decode_addr[6];

	if (!data) {
		cps_wls_log(CPS_LOG_ERR, "[%s] data null\n", __func__);
		return CPS_WLS_FAIL;
	}

	decode_addr[5] = ((data[9] & 0x0F) << 4) | ((data[4] & 0xF0) >> 4);
	decode_addr[4] = ((data[8] & 0x0F) << 4) | ((data[3] & 0xF0) >> 4);
	decode_addr[3] = ((data[7] & 0x0F) << 4) | ((data[2] & 0xF0) >> 4);

	decode_addr[2] = ((data[4] & 0x0F) << 4) | ((data[9] & 0xF0) >> 4);
	decode_addr[1] = ((data[3] & 0x0F) << 4) | ((data[8] & 0xF0) >> 4);
	decode_addr[0] = ((data[2] & 0x0F) << 4) | ((data[7] & 0xF0) >> 4);

	ble_addr = decode_addr[5]  << 16 | decode_addr[4] << 8 | decode_addr[3];
	ble_addr = ble_addr << 24 | decode_addr[2] << 16
		| decode_addr[1] << 8 | decode_addr[0];

	return ble_addr;
}

static bool cps_ble_mac_valid(struct cps_wls_chrg_chip *chip)
{
	uint64_t addr = 0, check_data = 0;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return false;
	}
	if (chip->ble_mac_addr == CPS_WLS_FAIL) {
		cps_wls_log(CPS_LOG_ERR, "[%s]mac_ckeck_data get fail\n", __func__);
		return false;
	}
	addr = chip->ble_mac_addr;
	check_data = chip->mac_check_data;

	if ((((addr >> 40) & 0xFF) ^ ((addr >> 16) & 0xFF))
		!= ((check_data >> 16) & 0xFF))
		return false;

	if ((((addr >> 32) & 0xFF) ^ ((addr >> 8) & 0xFF))
		!= ((check_data >> 8) & 0xFF))
		return false;

	if ((((addr >> 24) & 0xFF) ^ (addr & 0xFF))
		!= (check_data & 0xFF))
		return false;


	cps_wls_log(CPS_LOG_ERR, "cps8601_valid_check: %02x %02x %02x.\n",
		(int)((check_data >> 16) & 0xFF),
		(int)((check_data >> 8) & 0xFF),
		(int)(check_data & 0xFF));
	return true;
}

static uint64_t cps_recv_mac_check_data(uint8_t *data)
{
	if (!data) {
		cps_wls_log(CPS_LOG_ERR, "[%s] data null\n", __func__);
		return CPS_WLS_FAIL;
	}
	return (uint64_t)(data[4] << 16 | data[3] << 8 | data[2]);
}

static int cps_recv_pen_soc(uint8_t *data)
{
	if (!data) {
		cps_wls_log(CPS_LOG_ERR, "[%s] data null\n", __func__);
		return CPS_WLS_FAIL;
	}
	return (int)data[2];
}

static int cps_wls_get_ask_packet(struct cps_wls_chrg_chip *chip)
{
	uint8_t temp, data[TX_PKG_LEN];
	uint8_t i;
	struct cps_reg_s *cps_reg;
	struct timeval now_time;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}
	cps_reg = (struct cps_reg_s*)(&cps_tx_reg[CPS_TX_REG_PPP_HEADER]);

	for (i = 0; i < TX_PKG_LEN; i++) {
		temp = cps_wls_read_reg((int)(cps_reg->reg_addr + i),
			(int)cps_reg->reg_bytes_len);
		if (temp != CPS_WLS_FAIL)
			data[i] = temp;
		else {
			cps_wls_log(CPS_LOG_ERR, "[%s] read reg fail\n", __func__);
			return CPS_WLS_FAIL;
		}
	}

	if ((data[0] == MAC_PKG_HEAD) && (data[1] == MAC_PKG_CMD_CKECK)) {
		chip->mac_check_data = cps_recv_mac_check_data(data);
		cps_wls_log(CPS_LOG_ERR, "[%s]get add_check\n", __func__);
	}
	if ((data[0] == MAC_PKG_HEAD) && (data[1] == MAC_PKG_CMD_ADD1)&&
		(data[5] == MAC_PKG_HEAD) && data[6] == MAC_PKG_CMD_ADD2) {
		chip->ble_mac_addr = cps_recv_ble_mac_addr(data);
		if (chip->ble_mac_addr != CPS_WLS_FAIL) {
			do_gettimeofday(&now_time);
			chip->upto_ble_time = now_time.tv_sec * 1000 +
				now_time.tv_usec / 1000 - chip->tx_start_time;
			cps_wls_log(CPS_LOG_ERR, "[%s]get ble addr(%lld)\n",
				__func__, chip->upto_ble_time);
		}
		if (cps_ble_mac_valid(chip)) {
			chip->pen_present = 1;
			if (chip->power_enable_reason != PEN_REASON_RECHARGE) {
				cps8601_send_uevent(chip->wireless_dev, chip->pen_present,
					chip->ble_mac_addr);
				cps_wls_log(CPS_LOG_ERR, "[%s]report %d\n",
					__func__, chip->pen_present);
			}
		} else {
			g_verify_failed_cnt++;
			cps_set_charge_allow(chip, false);
		}
	}

	if ((data[0] == POWER_PKG_HEAD) && data[1] == POWER_PKG_CMD) {
		chip->pen_soc = cps_recv_pen_soc(data);

		if (chip->pen_soc == 100) {
			cps_set_charge_allow(chip, false);
			chip->power_disable_reason = PEN_REASON_CHARGE_FULL;
			cps_wls_log(CPS_LOG_ERR, "[%s] soc full\n", __func__);
		} else {
			cps_set_charge_allow(chip, false);
			chip->power_disable_reason = PEN_REASON_CHARGE_STOP;
			cps_wls_log(CPS_LOG_ERR, "[%s] charge stop\n", __func__);
		}
	}

	return CPS_WLS_SUCCESS;
}

static int cps_ept_type_detect_handle(struct cps_wls_chrg_chip *chip)
{
	int ept_type_code = 0;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}

	ept_type_code = cps_wls_get_tx_ept_type();
	if (ept_type_code == CPS_WLS_FAIL) {
		cps_wls_log(CPS_LOG_ERR, "[%s] ept code read fail\n", __func__);
		return CPS_WLS_FAIL;
	}

	chip->power_disable_reason = PEN_REASON_CHARGE_EPT;
	cps_wls_log(CPS_LOG_ERR, "[%s] ept code = 0X%x\n", __func__, ept_type_code);
	if ((ept_type_code & EPT_OCP) || (ept_type_code & EPT_OVP)
		|| (ept_type_code & EPT_FOD) || (ept_type_code & EPT_OTP)
		|| (ept_type_code & EPT_POCP)) {
		cps_set_charge_allow(chip, false);
		cps_set_gpio_value(chip, GP_1, 1);
		chip->power_disable_reason = PEN_REASON_CHARGE_OCP;
	} else if (ept_type_code & EPT_UVP) {
		if ((chip->hboost_status != HBOOST_IS_SETTING_BOOST)
			&& chip->is_supply_by_hboost) {
			schedule_work(&chip->set_boost_work);
		}
		cps_set_charge_allow(chip, false);
		cps_set_gpio_value(chip, GP_1, 1);
		chip->power_disable_reason = PEN_REASON_CHARGE_OCP;
	}

	return ept_type_code;
}


static void cps_notify_tx_wakeup(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}
	chip->tx_wakeup_flag = true;
	wake_up_interruptible(&tx_wakeup_irq_waiter);
}

static void cps_notify_q_cali_int(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}
	chip->q_cali_int_flag = true;
	wake_up_interruptible(&q_cali_irq_waiter);
}

static void cps_set_charge_allow(struct cps_wls_chrg_chip *chip, bool allow)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}

	if (allow) {
		cps_set_gpio_value(chip, GP_3, 0);
		chip->charge_allow = 1;
	} else {
		cps_set_gpio_value(chip, GP_3, 1);
		chip->charge_allow = 0;
	}
}

static void cps_reset_chip_status(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}

	chip->pen_status = PEN_STATUS_FAR;
	chip->pen_present = 0;
	chip->ping_succ_time = 0;
	chip->upto_ble_time = 0;
	chip->tx_start_time = 0;
	chip->ble_mac_addr = 0;
	chip->mac_check_data = 0;
	chip->tx_voltage = 0;
	chip->tx_current = 0;
	if (chip->power_disable_reason == PEN_REASON_CHARGE_OCP
		|| chip->power_disable_reason == PEN_REASON_CHARGE_EPT)
		cps_wls_reset_sys();
	chip->power_disable_reason = PEN_REASON_UNKNOWN;
	chip->power_enable_reason = PEN_REASON_UNDEFINED;
}

static int cps_wls_set_int_enable(void)
{
	uint16_t int_en, ret = 0;
	struct cps_reg_s *cps_reg;

	int_en = 0xFFFF;
	cps_reg = (struct cps_reg_s*)(&cps_comm_reg[CPS_COMM_REG_INT_EN]);

	ret = cps_wls_write_reg((int)cps_reg->reg_addr, int_en,
		(int)cps_reg->reg_bytes_len);
	if (CPS_WLS_FAIL == ret) {
		cps_wls_log(CPS_LOG_ERR, "[%s] fail\n", __func__);
		return CPS_WLS_FAIL;
	}
	return CPS_WLS_SUCCESS;
}

static int cps_wls_tx_irq_handler(struct cps_wls_chrg_chip *chip, int irq_flag)
{
	int rc = 0;
	struct timeval now_time;
	int cnt, width, ept;

	if (!chip) {
		cps_wls_log(CPS_LOG_DEBG, "[%s] chip null\n", __func__);
		return CPS_WLS_FAIL;
	}

	if (chip->q_cali_status != Q_CALI_IN_PROGRESS) {
		if (irq_flag & TX_INT_SSP) {
			do_gettimeofday(&now_time);
			chip->ping_succ_time = now_time.tv_sec * 1000 +
				now_time.tv_usec / 1000 - chip->tx_start_time;
			chip->pen_present = 1;
			cps_wls_log(CPS_LOG_DEBG, "ATO CPS8601 ping_succ_time(%lld)\n",
				chip->ping_succ_time);
		}
		if (irq_flag & TX_INT_RX_ATTACH) {
			cps_wls_log(CPS_LOG_ERR, "Rx attach!\n");
			cnt = cps_wls_get_no_rx_cnt();
			width = cps_wls_get_no_rx_width();
			cps_wls_log(CPS_LOG_ERR, "cnt:%d, width:%d\n", cnt, width);
			do_gettimeofday(&now_time);
			chip->tx_start_time =
				now_time.tv_sec * 1000 + now_time.tv_usec / 1000;
			chip->pen_status = PEN_STATUS_NEAR;
			set_boost_by_attach_status(true, chip);
			schedule_work(&chip->error_attach_check_work);
			schedule_delayed_work(&chip->max_chg_time_check_work,
				round_jiffies_relative(msecs_to_jiffies
					(chip->power_expired_time * MIN_TO_MS)));
		}
		if (irq_flag & TX_INT_RX_REMOVED) {
			cps_wls_log(CPS_LOG_ERR, "Rx remove!\n");
			cnt = cps_wls_get_no_rx_cnt();
			width = cps_wls_get_no_rx_width();
			ept = cps_wls_get_ept_code();
			cps_wls_log(CPS_LOG_ERR, "cnt:%d, width:%d ept_code:%d\n", cnt, width, ept);
			if (chip->led_on)
				cps_set_gpio_value(chip, GP_1, 1);
			else
				cps_set_gpio_value(chip, GP_1, 0);
			chip->pen_present = 0;
			set_boost_by_attach_status(false, chip);
			cps8601_send_uevent(chip->wireless_dev, chip->pen_present,
				chip->ble_mac_addr);
			cancel_delayed_work(&chip->max_chg_time_check_work);
			cps_reset_chip_status(chip);
			cps_set_charge_allow(chip, true);
		}
		if (irq_flag & TX_INT_ASK_PKT) {
			cps_wls_log(CPS_LOG_ERR, "receive ask pkt!\n");
			rc = cps_wls_get_ask_packet(chip);
		}
		if (irq_flag & TX_INT_EPT) {
			cps_wls_log(CPS_LOG_ERR, "Tx int ept!\n");
			rc = cps_ept_type_detect_handle(chip);
		}
	}

	if (irq_flag & TX_INT_WAKEUP) {
		cps_wls_log(CPS_LOG_ERR, "tx wakeup!\n");
		cps_notify_tx_wakeup(chip);
	}

	if (irq_flag & TX_INT_Q_CALI) {
		cps_wls_log(CPS_LOG_ERR, "q cali int!\n");
		cps_notify_q_cali_int(chip);
	}

	if ((irq_flag & TX_INT_MIS_LOC) && chip->led_on) {
		cps_wls_log(CPS_LOG_ERR, "mislocated!\n");
		cps8601_send_mislocated_uevent(chip->wireless_dev);
	}
	return rc;
}

static irqreturn_t cps_wls_irq_handler(int irq, void *dev_id)
{
	int irq_flag;
	struct cps_wls_chrg_chip *chip = (struct cps_wls_chrg_chip *)dev_id;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return IRQ_HANDLED;
	}
	cps_wls_log(CPS_LOG_ERR, "[%s] IRQ triggered\n", __func__);
	__pm_stay_awake(chip->cps_iic_wake_lock);
	wait_event_interruptible_timeout(i2c_waiter, chip->i2c_ready, msecs_to_jiffies(50));
	if (!chip->i2c_ready) {
		cps_wls_log(CPS_LOG_DEBG, "[%s]iic not ready\n", __func__);
		__pm_relax(chip->cps_iic_wake_lock);
		return IRQ_HANDLED;
	}
	mutex_lock(&chip->irq_lock);
	cps_wls_set_int_enable();

	irq_flag = cps_wls_get_int_flag();
	cps_wls_log(CPS_LOG_ERR, "irq_flag = 0x%x\n", irq_flag);
	if(irq_flag == CPS_WLS_FAIL) {
		cps_wls_log(CPS_LOG_ERR, "[%s] read wls irq reg failed\n", __func__);
		mutex_unlock(&chip->irq_lock);
		__pm_relax(chip->cps_iic_wake_lock);
		return IRQ_HANDLED;
	}

	cps_wls_set_int_clr(irq_flag);
	mutex_unlock(&chip->irq_lock);
	cps_wls_tx_irq_handler(chip, irq_flag);
	__pm_relax(chip->cps_iic_wake_lock);
	return IRQ_HANDLED;
}

static ssize_t ble_mac_addr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	if (chip->pen_present) {
		if (chip->ble_mac_addr)
			return sprintf(buf, "0x%llx_(Time:%lldms)\n",
				chip->ble_mac_addr, chip->upto_ble_time);
		else
			return sprintf(buf, "%s", "wait_to_get_addr.\n");
	} else
		return sprintf(buf, "%s", "wait_to_connect.\n");

	return 0;
}
static DEVICE_ATTR_RO(ble_mac_addr);

static ssize_t present_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->pen_present);
}
static DEVICE_ATTR_RO(present);

static ssize_t fw_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "0x%x\n", chip->fw_version);
}
static DEVICE_ATTR_RO(fw_version);

static ssize_t q_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;
	static int cnt = 0, width = 0;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	cnt = cps_wls_get_no_rx_cnt();
	width = cps_wls_get_no_rx_width();

	return sprintf(buf, "cnt:%d width:%d\n", cnt, width);
}
static DEVICE_ATTR_RO(q_value);

static ssize_t tx_voltage_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}
	if (chip->pen_present)
		chip->tx_voltage = cps_wls_get_tx_vin();

	return sprintf(buf, "%d\n", chip->tx_voltage);
}
static DEVICE_ATTR_RO(tx_voltage);

static ssize_t tx_current_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}
	if (chip->pen_present)
		chip->tx_current = cps_wls_get_tx_i_in();

	return sprintf(buf, "%d\n", chip->tx_current);
}
static DEVICE_ATTR_RO(tx_current);

static ssize_t rx_soc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->rx_soc);
}

static ssize_t rx_soc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cps_wls_chrg_chip *chip = NULL;
	int val = 0;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		cps_wls_log(CPS_LOG_ERR, "buf error\n");
		return -EINVAL;
	}
	WRITE_ONCE(chip->rx_soc, val);

	if ((chip->rx_soc <= chip->soc_threshould) && \
		(chip->pen_status == PEN_STATUS_NEAR) && \
		(false == chip->charge_allow) && \
		(PEN_REASON_CHARGE_FULL == chip->power_disable_reason)) {
		schedule_work(&chip->rechg_work);
	} else {
		cps_wls_log(CPS_LOG_ERR, "cps8601 value: %d, %d, %d.\n",
			chip->rx_soc, chip->pen_status, chip->charge_allow);
	}

	return count;
}
static DEVICE_ATTR_RW(rx_soc);

static ssize_t wireless_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "[%d:%d]\n[%d]\n[%d]\n[%d]\n[%d]\n[%d %d %d %d]\n", \
					chip->rx_soc, chip->soc_threshould, \
					chip->charge_allow, \
					chip->power_enable_reason, \
					chip->power_expired_time, \
					chip->power_disable_reason, \
					chip->q_cali_status, chip->cali_result.q_cali_times,
					chip->cali_result.q_cali_cnt, chip->cali_result.q_cali_width);
}

static ssize_t wireless_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		cps_wls_log(CPS_LOG_DEBG, "buf error\n");
		return -EINVAL;
	}
	if (val <= 100)
		/* we use 0-100 as soc threshould */
		chip->soc_threshould = val;
	else if (val < 2000)
		/* we use 100-2000 as power expired time */
		chip->power_expired_time = val - FULL_SOC;
	else {
		/* we use more than 2000 as set hboost vout*/
		cps_wls_log(CPS_LOG_DEBG, "[%s]set v_boost %d mV\n",
			__func__, val);
		cps_set_boost_volt_mv(val);
	}

	return count;
}

static DEVICE_ATTR_RW(wireless_debug);

static ssize_t ping_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%lld\n", chip->ping_succ_time);
}
static DEVICE_ATTR_RO(ping_time);

static ssize_t tx_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	if (chip->pen_present)
		return sprintf(buf, "Connected\n");
	else
		return sprintf(buf, "Disconnect\n");
}
static DEVICE_ATTR_RO(tx_status);


static ssize_t wireless_ic_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;
	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "0x%x\n", chip->chip_id);
}
static DEVICE_ATTR_RO(wireless_ic_id);

static ssize_t q_cali_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;
	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	if (chip->q_cali_status == Q_CALI_IN_PROGRESS)
		return sprintf(buf, "calibrating\n");
	else if (chip->q_cali_status == Q_CALI_SUCCESS)
		return sprintf(buf,
			"pass:cali_times:%d cnt:%d width:%d cnt_var:%d width_var:%d\n",
			chip->cali_result.q_cali_times, chip->cali_result.q_cali_cnt,
			chip->cali_result.q_cali_width, chip->cali_result.q_cali_cnt_var,
			chip->cali_result.q_cali_width_var);
	else if (chip->q_cali_status == Q_CALI_FAIL)
		return sprintf(buf,
			"fail:cali_times:%d cnt:%d width:%d cnt_var:%d width_var:%d\n",
			chip->cali_result.q_cali_times, chip->cali_result.q_cali_cnt,
			chip->cali_result.q_cali_width, chip->cali_result.q_cali_cnt_var,
			chip->cali_result.q_cali_width_var);
	else if (chip->q_cali_status == Q_CALI_CALIBRATED)
		return sprintf(buf,
			"calibtated:cali_times:%d cnt:%d width:%d cnt_var:%d width_var:%d\n",
			chip->cali_result.q_cali_times, chip->cali_result.q_cali_cnt,
			chip->cali_result.q_cali_width, chip->cali_result.q_cali_cnt_var,
			chip->cali_result.q_cali_width_var);
	else if (chip->q_cali_status == Q_CALI_UNCALIBRATED)
		return sprintf(buf,
			"Uncalibtated:cali_times:%d cnt:%d width:%d cnt_var:%d width_var:%d\n",
			chip->cali_result.q_cali_times, chip->cali_result.q_cali_cnt,
			chip->cali_result.q_cali_width, chip->cali_result.q_cali_cnt_var,
			chip->cali_result.q_cali_width_var);
	else
		return sprintf(buf, "unknown status\n");
}

static ssize_t q_cali_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cps_wls_chrg_chip *chip = NULL;
	int val = 0;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		cps_wls_log(CPS_LOG_ERR, "buf error\n");
		return -EINVAL;
	}

	if ((val == 1)
		&& (chip->q_cali_status != Q_CALI_IN_PROGRESS))
		schedule_work(&chip->q_cali_work);

	return count;
}

static DEVICE_ATTR_RW(q_cali);

/*rw 0:read 1:write*/
static int addr = 0, length = 0, rw = 0, write_data = 0;

static ssize_t wireless_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;
	int value = 0;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}
	cps_wls_log(CPS_LOG_ERR, "[%s]:add=0x%x,len=%d,rw=%d,write_data=0x%x\n",
		__func__, addr, length, rw, write_data);
	if (rw == 0) {
		value = cps_wls_read_reg(addr, length);
		if (value < 0)
			return sprintf(buf, "0x%x  read fail\n", addr);
		else
			return sprintf(buf, "0x%04x = 0x%x\n", addr, value);
	} else if (rw == 1) {
		value = cps_wls_write_reg(addr, write_data, length);
		if (value < 0)
			return sprintf(buf, "0x%04x  write fail\n", addr);
		else
			return sprintf(buf, "0x%04x write 0x%x success\n",
						addr, write_data);
	} else {
		return sprintf(buf, "unsupport operator\n");
	}
}

static ssize_t wireless_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	sscanf(buf, "%x %d %d %x", &addr, &length, &rw, &write_data);
	cps_wls_log(CPS_LOG_ERR, "[%s]:add=0x%x,len=%d,rw=%d,write_data=0x%x\n",
		__func__, addr, length, rw, write_data);
	return count;
}
static DEVICE_ATTR_RW(wireless_reg);

static ssize_t gpio_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cps_wls_chrg_chip *chip = NULL;
	int gp0_val = 0, gp1_val = 0, gp2_val = 0, gp3_val = 0, gp4_val = 0;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	gp0_val = gpio_get_value(chip->wls_sleep_gpio);
	gp1_val = gpio_get_value(chip->wls_scan_gpio);
	gp2_val = gpio_get_value(chip->wls_sw_en_gpio);
	gp3_val = gpio_get_value(chip->wls_off_state_gpio);
	if (!chip->is_supply_by_hboost)
		gp4_val = gpio_get_value(chip->wls_boost_en_gpio);
	return sprintf(buf, "gp0=%d, gp1=%d, gp2=%d, gp3=%d, gp4=%d\n",
		gp0_val, gp1_val, gp2_val, gp3_val, gp4_val);
}

static ssize_t gpio_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int gpio, value;
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	sscanf(buf, "%d %d", &gpio, &value);
	cps_set_gpio_value(chip, gpio, value);

	return count;
}
static DEVICE_ATTR_RW(gpio_debug);

static ssize_t rx_mislocate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cps_wls_chrg_chip *chip = NULL;

	chip = (struct cps_wls_chrg_chip *)dev_get_drvdata(dev);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "chip is NULL\n");
		return -EINVAL;
	}

	cps8601_send_mislocated_uevent(dev);

	return count;
}
static DEVICE_ATTR_WO(rx_mislocate);

static struct device_attribute *pencil_attributes[] = {
	&dev_attr_ble_mac_addr,
	&dev_attr_present,
	&dev_attr_fw_version,
	&dev_attr_q_value,
	&dev_attr_tx_current,
	&dev_attr_tx_voltage,
	&dev_attr_rx_soc,
	&dev_attr_wireless_debug,
	&dev_attr_wireless_ic_id,
	&dev_attr_ping_time,
	&dev_attr_tx_status,
	&dev_attr_q_cali,
	&dev_attr_wireless_reg,
	&dev_attr_gpio_debug,
	&dev_attr_rx_mislocate,
	NULL
};

static int init_wireless_device(struct cps_wls_chrg_chip *chip)
{
	int err = 0, status = 0;
	dev_t devt;
	struct class *wireless_class = NULL;
	struct device_attribute **attrs, *attr;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s]: chip is null!\n", __func__);
		return -EINVAL;
	}

#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
	wireless_class = class_create("oplus_wireless");
#else
	wireless_class = class_create(THIS_MODULE, "oplus_wireless");
#endif

	status = alloc_chrdev_region(&devt, 0, 1, "tx_wireless");
	chip->wireless_dev = device_create(wireless_class, NULL,
		devt, NULL, "%s", "pencil");
	chip->wireless_dev->devt = devt;
	dev_set_drvdata(chip->wireless_dev, chip);

	attrs = pencil_attributes;
	while ((attr = *attrs++)) {
		err = device_create_file(chip->wireless_dev, attr);
		if (err) {
			cps_wls_log(CPS_LOG_ERR, "device_create_file fail!\n");
			return err;
		}
	}

	return 0;
}

static int wls_charge_int_gpio_init(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s]: chip is null!\n", __func__);
		return -EINVAL;
	}

	chip->cps_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->cps_pinctrl)) {
		cps_wls_log(CPS_LOG_ERR, "get pinctrl fail\n");
		return -EINVAL;
	}

	chip->wls_charge_int_active =
		pinctrl_lookup_state(chip->cps_pinctrl, "wls_charge_int_active");
	if (IS_ERR_OR_NULL(chip->wls_charge_int_active)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_charge_int_active fail\n");
		return -EINVAL;
	}

	chip->wls_charge_int_sleep =
		pinctrl_lookup_state(chip->cps_pinctrl, "wls_charge_int_sleep");
	if (IS_ERR_OR_NULL(chip->wls_charge_int_sleep)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_charge_int_sleep fail\n");
		return -EINVAL;
	}

	chip->wls_charge_int_default =
		pinctrl_lookup_state(chip->cps_pinctrl, "wls_charge_int_default");
	if (IS_ERR_OR_NULL(chip->wls_charge_int_default)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_charge_int_default fail\n");
		return -EINVAL;
	}

	if (chip->wls_charge_int > 0)
		gpio_direction_input(chip->wls_charge_int);

	pinctrl_select_state(chip->cps_pinctrl, chip->wls_charge_int_active);

	return 0;
}

static void cps_wls_irq_init(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null \n", __func__);
		return;
	}
	chip->cps_wls_irq = gpio_to_irq(chip->wls_charge_int);
	cps_wls_log(CPS_LOG_ERR, "[%s] chip->cps_wls_irq[%d]\n",
		__func__, chip->cps_wls_irq);
}

static int wls_off_state_gpio_init(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s]: chip is null!\n", __func__);
		return -EINVAL;
	}

	chip->cps_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->cps_pinctrl)) {
		cps_wls_log(CPS_LOG_ERR, "get pinctrl fail\n");
		return -EINVAL;
	}

	chip->wls_off_state_active =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_off_state_active");
	if (IS_ERR_OR_NULL(chip->wls_off_state_active)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_off_state_active fail\n");
		return -EINVAL;
	}

	chip->wls_off_state_sleep =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_off_state_sleep");
	if (IS_ERR_OR_NULL(chip->wls_off_state_sleep)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_off_state_sleep fail\n");
		return -EINVAL;
	}

	chip->wls_off_state_default =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_off_state_default");
	if (IS_ERR_OR_NULL(chip->wls_off_state_default)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_off_state_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->wls_off_state_gpio, 0);
	pinctrl_select_state(chip->cps_pinctrl, chip->wls_off_state_active);

	return 0;
}

static int wls_sleep_gpio_init(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s]: chip is null!\n", __func__);
		return -EINVAL;
	}

	chip->cps_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->cps_pinctrl)) {
		cps_wls_log(CPS_LOG_ERR, "get pinctrl fail\n");
		return -EINVAL;
	}

	chip->wls_sleep_active =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_sleep_active");
	if (IS_ERR_OR_NULL(chip->wls_sleep_active)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_sleep_active fail\n");
		return -EINVAL;
	}

	chip->wls_sleep_sleep =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_sleep_sleep");
	if (IS_ERR_OR_NULL(chip->wls_sleep_sleep)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_sleep_sleep fail\n");
		return -EINVAL;
	}

	chip->wls_sleep_default =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_sleep_default");
	if (IS_ERR_OR_NULL(chip->wls_sleep_default)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_sleep_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->wls_sleep_gpio, 0);
	pinctrl_select_state(chip->cps_pinctrl, chip->wls_sleep_sleep);

	return 0;
}

static int wls_scan_gpio_init(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s]: chip is null!\n", __func__);
		return -EINVAL;
	}

	chip->cps_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->cps_pinctrl)) {
		cps_wls_log(CPS_LOG_ERR, "get pinctrl fail\n");
		return -EINVAL;
	}

	chip->wls_scan_active =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_scan_active");
	if (IS_ERR_OR_NULL(chip->wls_scan_active)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_scan_active fail\n");
		return -EINVAL;
	}

	chip->wls_scan_sleep =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_scan_sleep");
	if (IS_ERR_OR_NULL(chip->wls_scan_sleep)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_scan_sleep fail\n");
		return -EINVAL;
	}

	chip->wls_scan_default =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_scan_default");
	if (IS_ERR_OR_NULL(chip->wls_scan_default)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_scan_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->wls_scan_gpio, 0);
	pinctrl_select_state(chip->cps_pinctrl, chip->wls_scan_sleep);

	return 0;
}

static int wls_sw_en_gpio_init(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s]: chip is null!\n", __func__);
		return -EINVAL;
	}

	chip->cps_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->cps_pinctrl)) {
		cps_wls_log(CPS_LOG_ERR, "get pinctrl fail\n");
		return -EINVAL;
	}

	chip->wls_sw_en_active =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_sw_en_active");
	if (IS_ERR_OR_NULL(chip->wls_sw_en_active)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_sw_en_active fail\n");
		return -EINVAL;
	}

	chip->wls_sw_en_sleep =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_sw_en_sleep");
	if (IS_ERR_OR_NULL(chip->wls_sw_en_sleep)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_sw_en_sleep fail\n");
		return -EINVAL;
	}

	chip->wls_sw_en_default =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_sw_en_default");
	if (IS_ERR_OR_NULL(chip->wls_sw_en_default)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_sw_en_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->wls_sw_en_gpio, 0);
	pinctrl_select_state(chip->cps_pinctrl, chip->wls_sw_en_sleep);

	return 0;
}

static int wls_boost_en_gpio_init(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s]: chip is null!\n", __func__);
		return -EINVAL;
	}

	chip->cps_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->cps_pinctrl)) {
		cps_wls_log(CPS_LOG_ERR, "get pinctrl fail\n");
		return -EINVAL;
	}

	chip->wls_boost_en_active =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_boost_en_active");
	if (IS_ERR_OR_NULL(chip->wls_boost_en_active)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_boost_en_active fail\n");
		return -EINVAL;
	}

	chip->wls_boost_en_sleep =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_boost_en_sleep");
	if (IS_ERR_OR_NULL(chip->wls_boost_en_sleep)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_boost_en_sleep fail\n");
		return -EINVAL;
	}

	chip->wls_boost_en_default =
			pinctrl_lookup_state(chip->cps_pinctrl, "wls_boost_en_default");
	if (IS_ERR_OR_NULL(chip->wls_boost_en_default)) {
		cps_wls_log(CPS_LOG_ERR, "get wls_boost_en_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->wls_boost_en_gpio, 0);
	pinctrl_select_state(chip->cps_pinctrl, chip->wls_boost_en_sleep);

	return 0;
}

static int cps_wls_set_protect_parameter(struct cps_wls_chrg_chip *chip) {
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s]: chip is null!\n", __func__);
		return -EINVAL;
	}
	cps_wls_set_tx_ocp_threshold(chip->ocp_threshold);
	cps_wls_set_tx_uvp_threshold(chip->lvp_threshold);
	cps_wls_set_tx_ovp_threshold(chip->ovp_threshold);
	cps_wls_set_tx_fod_thresh(chip->fod_threshold);

	return 0;
}

static int cps_wls_gpio_init(struct cps_wls_chrg_chip *chip)
{
	struct device_node *node;
	int rc = 0;

	 if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null \n", __func__);
		return -EINVAL;
	}
	 node = chip->dev->of_node;

	if (!node) {
		cps_wls_log(CPS_LOG_ERR, "devices tree node missing \n");
		return -EINVAL;
	}

	chip->wls_charge_int = of_get_named_gpio(node, "qcom,cps_wls_int-gpio", 0);
	if (chip->wls_charge_int < 0) {
		cps_wls_log(CPS_LOG_ERR, "wls_charge_int not specified\n");
		return -EINVAL;
	} else {
		if (gpio_is_valid(chip->wls_charge_int)) {
			rc = gpio_request(chip->wls_charge_int, "wls-charge-int");
			if (rc) {
				cps_wls_log(CPS_LOG_ERR, "unable to request gpio [%d]\n",
					chip->wls_charge_int);
				return -EINVAL;
			} else {
				rc = wls_charge_int_gpio_init(chip);
				if (rc) {
					cps_wls_log(CPS_LOG_ERR, "wls_charge_int_gpio_init failed\n");
					return -EINVAL;
				} else {
					cps_wls_irq_init(chip);
				}
			}
		}
		cps_wls_log(CPS_LOG_DEBG, "chip->wls_charge_int =%d\n", chip->wls_charge_int);
	}

	chip->wls_off_state_gpio = of_get_named_gpio(node, "qcom,wls_off_state-gpio", 0);
	if (chip->wls_off_state_gpio < 0) {
		cps_wls_log(CPS_LOG_ERR, "wls_off_state not specified\n");
		return -EINVAL;
	} else {
		if (gpio_is_valid(chip->wls_off_state_gpio)) {
			rc = gpio_request(chip->wls_off_state_gpio, "wls-off-state");
			if (rc) {
				cps_wls_log(CPS_LOG_ERR, "unable to request gpio [%d]\n",
					chip->wls_off_state_gpio);
				return -EINVAL;
			} else {
				rc = wls_off_state_gpio_init(chip);
				if (rc) {
					cps_wls_log(CPS_LOG_ERR, "wls_off_state_gpio_init failed\n");
					return -EINVAL;
				}
			}
		}
		cps_wls_log(CPS_LOG_DEBG, "chip->wls_off_state_gpio =%d\n",
			chip->wls_off_state_gpio);
	}

	chip->wls_sleep_gpio = of_get_named_gpio(node, "qcom,wls_sleep-gpio", 0);
	if (chip->wls_sleep_gpio < 0) {
		cps_wls_log(CPS_LOG_ERR, "wls_sleep_gpio not specified\n");
		return -EINVAL;
	} else {
		if (gpio_is_valid(chip->wls_sleep_gpio)) {
			rc = gpio_request(chip->wls_sleep_gpio, "wls-sleep");
			if (rc) {
				cps_wls_log(CPS_LOG_ERR, "unable to request gpio [%d]\n",
					chip->wls_sleep_gpio);
				return -EINVAL;
			} else {
				rc = wls_sleep_gpio_init(chip);
				if (rc) {
					cps_wls_log(CPS_LOG_ERR, "wls_sleep_gpio_init failed\n");
					return -EINVAL;
				}
			}
		}
		cps_wls_log(CPS_LOG_DEBG, "chip->wls_sleep_gpio =%d\n", chip->wls_sleep_gpio);
	}

	chip->wls_scan_gpio = of_get_named_gpio(node, "qcom,wls_scan-gpio", 0);
	if (chip->wls_scan_gpio < 0) {
		cps_wls_log(CPS_LOG_ERR, "wls_scan_gpio not specified\n");
		return -EINVAL;
	} else {
		if (gpio_is_valid(chip->wls_scan_gpio)) {
			rc = gpio_request(chip->wls_scan_gpio, "wls-scan");
			if (rc) {
				cps_wls_log(CPS_LOG_ERR, "unable to request gpio [%d]\n",
					chip->wls_scan_gpio);
				return -EINVAL;
			} else {
				rc = wls_scan_gpio_init(chip);
				if (rc) {
					cps_wls_log(CPS_LOG_ERR, "wls_scan_gpio_init failed\n");
					return -EINVAL;
				}
			}
		}
		cps_wls_log(CPS_LOG_DEBG, "chip->wls_scan_gpio =%d\n", chip->wls_scan_gpio);
	}

	chip->wls_sw_en_gpio = of_get_named_gpio(node, "qcom,wls_sw_en-gpio", 0);
	if (chip->wls_sw_en_gpio < 0) {
		cps_wls_log(CPS_LOG_ERR, "wls_sw_en_gpio not specified\n");
		return -EINVAL;
	} else {
		if (gpio_is_valid(chip->wls_sw_en_gpio)) {
			rc = gpio_request(chip->wls_sw_en_gpio, "wls-sw-en");
			if (rc) {
				cps_wls_log(CPS_LOG_ERR, "unable to request gpio [%d]\n",
					chip->wls_sw_en_gpio);
				return -EINVAL;
			} else {
				rc = wls_sw_en_gpio_init(chip);
				if (rc) {
					cps_wls_log(CPS_LOG_ERR, "wls_sw_en_gpio failed\n");
					return -EINVAL;
				}
			}
		}
		cps_wls_log(CPS_LOG_DEBG, "chip->wls_sw_en_gpio =%d\n", chip->wls_sw_en_gpio);
	}

	if (!chip->is_supply_by_hboost) {
		chip->wls_boost_en_gpio = of_get_named_gpio(node, "qcom,wls_boost_en-gpio", 0);
		if (chip->wls_boost_en_gpio < 0) {
			cps_wls_log(CPS_LOG_ERR, "wls_boost_en_gpio not specified\n");
			return -EINVAL;
		} else {
			if (gpio_is_valid(chip->wls_boost_en_gpio)) {
				rc = gpio_request(chip->wls_boost_en_gpio, "wls-boost-en");
				if (rc) {
					cps_wls_log(CPS_LOG_ERR, "unable to request gpio [%d]\n",
						chip->wls_boost_en_gpio);
					return -EINVAL;
				} else {
					rc = wls_boost_en_gpio_init(chip);
					if (rc) {
						cps_wls_log(CPS_LOG_ERR, "wls_boost_en_gpio failed\n");
						return -EINVAL;
					}
				}
			}
			cps_wls_log(CPS_LOG_DEBG, "chip->wls_boost_en_gpio =%d\n", chip->wls_boost_en_gpio);
		}
	}

	return 0;
}

static void cps_wls_free_gpio(struct cps_wls_chrg_chip *chip)
{
	if (gpio_is_valid(chip->wls_charge_int))
		gpio_free(chip->wls_charge_int);
	if (gpio_is_valid(chip->wls_off_state_gpio))
		gpio_free(chip->wls_off_state_gpio);
	if (gpio_is_valid(chip->wls_sleep_gpio))
		gpio_free(chip->wls_sleep_gpio);
	if (gpio_is_valid(chip->wls_scan_gpio))
		gpio_free(chip->wls_scan_gpio);
	if (gpio_is_valid(chip->wls_boost_en_gpio))
		gpio_free(chip->wls_boost_en_gpio);
}

static void cps_set_gpio_value(struct cps_wls_chrg_chip *chip, int gp_num, int value)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null \n", __func__);
		return;
	}

	switch (gp_num) {
	case GP_0:
		if (IS_ERR_OR_NULL(chip->cps_pinctrl)
			|| IS_ERR_OR_NULL(chip->wls_sleep_active)
			|| IS_ERR_OR_NULL(chip->wls_sleep_sleep)
				|| IS_ERR_OR_NULL(chip->wls_sleep_default)) {
			cps_wls_log(CPS_LOG_ERR, "gp_num = %d pinctrl null\n", gp_num);
			return;
		}

		if (value) {
			gpio_direction_output(chip->wls_sleep_gpio, 1);
			pinctrl_select_state(chip->cps_pinctrl, chip->wls_sleep_active);
		} else {
			gpio_direction_output(chip->wls_sleep_gpio, 0);
			pinctrl_select_state(chip->cps_pinctrl, chip->wls_sleep_sleep);
		}
		cps_wls_log(CPS_LOG_DEBG, "gp_num:%d, set value:%d, gpio_val:%d\n",
			gp_num, value, gpio_get_value(chip->wls_sleep_gpio));
		break;
	case GP_1:
		if (IS_ERR_OR_NULL(chip->cps_pinctrl)
			|| IS_ERR_OR_NULL(chip->wls_scan_active)
			|| IS_ERR_OR_NULL(chip->wls_scan_sleep)
			|| IS_ERR_OR_NULL(chip->wls_scan_default)) {
			cps_wls_log(CPS_LOG_ERR, "gp_num = %d pinctrl null\n", gp_num);
			return;
		}

		if (value) {
			gpio_direction_output(chip->wls_scan_gpio, 1);
			pinctrl_select_state(chip->cps_pinctrl, chip->wls_scan_active);
		} else {
			gpio_direction_output(chip->wls_scan_gpio, 0);
			pinctrl_select_state(chip->cps_pinctrl, chip->wls_scan_sleep);
		}
		cps_wls_log(CPS_LOG_DEBG, "gp_num:%d, set value:%d, gpio_val:%d\n",
			gp_num, value, gpio_get_value(chip->wls_scan_gpio));
		break;
	case GP_2:
		if (IS_ERR_OR_NULL(chip->cps_pinctrl)
			|| IS_ERR_OR_NULL(chip->wls_sw_en_active)
			|| IS_ERR_OR_NULL(chip->wls_sw_en_sleep)
			|| IS_ERR_OR_NULL(chip->wls_sw_en_default)) {
			cps_wls_log(CPS_LOG_ERR, "gp_num = %d pinctrl null, return\n", gp_num);
			return;
		}

		if (value) {
			gpio_direction_output(chip->wls_sw_en_gpio, 1);
			pinctrl_select_state(chip->cps_pinctrl, chip->wls_sw_en_active);
		} else {
			gpio_direction_output(chip->wls_sw_en_gpio, 0);
			pinctrl_select_state(chip->cps_pinctrl, chip->wls_sw_en_sleep);
		}
		cps_wls_log(CPS_LOG_DEBG, "gp_num:%d, set value:%d, gpio_val:%d\n",
			gp_num, value, gpio_get_value(chip->wls_sw_en_gpio));
		break;
	case GP_3:
		if (IS_ERR_OR_NULL(chip->cps_pinctrl)
			|| IS_ERR_OR_NULL(chip->wls_off_state_active)
			|| IS_ERR_OR_NULL(chip->wls_off_state_sleep)
			|| IS_ERR_OR_NULL(chip->wls_off_state_default)) {
			cps_wls_log(CPS_LOG_ERR, "gp_num = %d pinctrl null, return\n", gp_num);
			return;
		}

		if (value) {
			gpio_direction_output(chip->wls_off_state_gpio, 1);
			pinctrl_select_state(chip->cps_pinctrl, chip->wls_off_state_active);
		} else {
			gpio_direction_output(chip->wls_off_state_gpio, 0);
			pinctrl_select_state(chip->cps_pinctrl, chip->wls_off_state_sleep);
		}
		cps_wls_log(CPS_LOG_DEBG, "gp_num:%d, set value:%d, gpio_val:%d\n",
			gp_num, value, gpio_get_value(chip->wls_off_state_gpio));
		break;
	case GP_4:
		if (IS_ERR_OR_NULL(chip->cps_pinctrl)
			|| IS_ERR_OR_NULL(chip->wls_boost_en_active)
			|| IS_ERR_OR_NULL(chip->wls_boost_en_sleep)
			|| IS_ERR_OR_NULL(chip->wls_boost_en_default)
			|| !gpio_is_valid(chip->wls_boost_en_gpio)) {
			cps_wls_log(CPS_LOG_ERR, "gp_num = %d pinctrl null, return\n", gp_num);
			return;
		}

		if (value) {
			gpio_direction_output(chip->wls_boost_en_gpio, 1);
			pinctrl_select_state(chip->cps_pinctrl, chip->wls_boost_en_active);
		} else {
			gpio_direction_output(chip->wls_boost_en_gpio, 0);
			pinctrl_select_state(chip->cps_pinctrl, chip->wls_boost_en_sleep);
		}
		cps_wls_log(CPS_LOG_DEBG, "gp_num:%d, set value:%d, gpio_val:%d\n",
			gp_num, value, gpio_get_value(chip->wls_boost_en_gpio));
		break;
	default:
		break;
	}
}

static int cps_wls_parse_dt(struct cps_wls_chrg_chip *chip)
{
	int rc;
	struct device_node *node;

	 if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null \n", __func__);
		return -EINVAL;
	}

	node = chip->dev->of_node;

	if (!node) {
		cps_wls_log(CPS_LOG_ERR, "device tree info. missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,ocp_threshold",
		&chip->ocp_threshold);
	if (rc)
		chip->ocp_threshold = CPS_OCP_THRESHOLD;
	cps_wls_log(CPS_LOG_ERR, "ocp_threshold[%d]\n", chip->ocp_threshold);

	rc = of_property_read_u32(node, "qcom,ovp_threshold",
			&chip->ovp_threshold);
	if (rc)
		chip->ovp_threshold = CPS_OVP_THRESHOLD;
	cps_wls_log(CPS_LOG_ERR, "ovp_threshold[%d]\n", chip->ovp_threshold);

	rc = of_property_read_u32(node, "qcom,lvp_threshold",
			&chip->lvp_threshold);
	if (rc)
		chip->lvp_threshold = CPS_LVP_THRESHOLD;
	cps_wls_log(CPS_LOG_ERR, "lvp_threshold[%d]\n", chip->lvp_threshold);

	rc = of_property_read_u32(node, "qcom,fod_threshold",
			&chip->fod_threshold);
	if (rc)
		chip->fod_threshold = CPS_FOD_THRESHOLD;
	cps_wls_log(CPS_LOG_ERR, "fod_threshold[%d]\n",
		chip->fod_threshold);

	rc = of_property_read_u32(node, "rx,soc_threshold",
			&chip->soc_threshould);
	if (rc)
		chip->soc_threshould = SOC_THRESHOLD;

	rc = of_property_read_u32(node, "tx,enable_expired_time",
			&chip->power_expired_time);
	if (rc)
		chip->power_expired_time = POWER_EXPIRED_TIME_DEFAULT;
	cps_wls_log(CPS_LOG_ERR, "enable_expired_time[%d]\n",
		chip->power_expired_time);

	rc = of_property_read_u32(node, "qcom,pen_support_track",
			&chip->pen_support_track);
	if (rc)
		chip->pen_support_track = 0;
	cps_wls_log(CPS_LOG_ERR, "pen_support_track[%d]\n",
		chip->pen_support_track);

	rc = of_property_read_u32(node, "qcom,pen_is_supply_by_hboost",
			&chip->is_supply_by_hboost);
	if (rc)
		chip->is_supply_by_hboost = 0;
	cps_wls_log(CPS_LOG_ERR, "is_supply_by_hboost[%d]\n",
		chip->is_supply_by_hboost);

	rc = of_property_read_u32(node, "qcom,hboost_default_volt_mv",
			&chip->hboost_default_volt);
	if (rc)
		chip->hboost_default_volt = DEFAULT_BOOST_VOLT_MV;

	rc = of_property_read_u32(node, "qcom,hboost_attach_volt_mv",
			&chip->hboost_attach_volt);
	if (rc)
		chip->hboost_attach_volt = chip->hboost_default_volt;

	chip->hboost_volt = chip->hboost_default_volt;
	cps_wls_log(CPS_LOG_ERR, "hboost_default_volt[%d], hboost_attach_volt[%d]\n",
		chip->hboost_default_volt, chip->hboost_attach_volt);

	return 0;
}

static void cps_wls_lock_work_init(struct cps_wls_chrg_chip *chip)
{
	mutex_init(&chip->irq_lock);
	mutex_init(&chip->i2c_lock);
	chip->cps_wls_wake_lock = wakeup_source_register(NULL, "cps_wls_wake_lock");
	chip->cps_iic_wake_lock = wakeup_source_register(NULL, "cps_iic_wake_lock");
}


static void cps_wls_lock_destroy(struct cps_wls_chrg_chip *chip)
{
	mutex_destroy(&chip->irq_lock);
	mutex_destroy(&chip->i2c_lock);
}

static int cps_get_cali_status(struct cps_wls_chrg_chip *chip)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null \n", __func__);
		return CPS_WLS_FAIL;
	}

	chip->cali_result.q_cali_times = cps_wls_get_cali_times();
	chip->cali_result.q_cali_cnt = cps_wls_get_q_cali_count();
	chip->cali_result.q_cali_width = cps_wls_get_q_cali_width();
	chip->cali_result.q_cali_cnt_var = cps_wls_get_cnt_variance();
	chip->cali_result.q_cali_width_var = cps_wls_get_width_variance();

	if (chip->cali_result.q_cali_times < 0 || chip->cali_result.q_cali_cnt < 0
		|| chip->cali_result.q_cali_width < 0 || chip->cali_result.q_cali_cnt_var < 0
		|| chip->cali_result.q_cali_width_var < 0)
		return CPS_WLS_FAIL;
	if (chip->cali_result.q_cali_times > 0)
		chip->q_cali_status = Q_CALI_CALIBRATED;
	else
		chip->q_cali_status = Q_CALI_UNCALIBRATED;

	return CPS_WLS_SUCCESS;
}

static void cps_set_led_on(struct cps_wls_chrg_chip *chip, bool led_on)
{
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null \n", __func__);
		return;
	}
	if (chip->led_on == led_on)
		return;

	cps_wls_log(CPS_LOG_DEBG, "[%s] set led_on = %d\n", __func__, led_on);
	if (led_on) {
		cps_set_gpio_value(chip, GP_1, 1);
		chip->led_on = led_on;
	} else {
		cps_set_gpio_value(chip, GP_1, 0);
		chip->led_on = led_on;
	}
}

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
static void cps_panel_notifier_callback(enum panel_event_notifier_tag tag,
			struct panel_event_notification *notification, void *client_data)
{
	struct cps_wls_chrg_chip *chip = client_data;

	if (!notification) {
		cps_wls_log(CPS_LOG_ERR, "Invalid notification\n");
		return;
	}

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null \n", __func__);
		return;
	}

	cps_wls_log(CPS_LOG_DEBG, "Notification type:%d, early_trigger:%d",
		notification->notif_type, notification->notif_data.early_trigger);

	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_UNBLANK:
		cps_wls_log(CPS_LOG_ERR, "received unblank event: %d\n",
			notification->notif_data.early_trigger);
		if (notification->notif_data.early_trigger)
			cps_set_led_on(chip, true);
		break;
	case DRM_PANEL_EVENT_BLANK:
		cps_wls_log(CPS_LOG_ERR, "received blank event: %d\n",
			notification->notif_data.early_trigger);
		if (notification->notif_data.early_trigger)
				cps_set_led_on(chip, false);
		break;
	case DRM_PANEL_EVENT_BLANK_LP:
		break;
	case DRM_PANEL_EVENT_FPS_CHANGE:
		break;
	default:
		break;
	}
}

#else /* CONFIG_DRM_PANEL_NOTIFY */

#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG)
static int cps_mtk_drm_notifier_callback(struct notifier_block *nb,
			unsigned long event, void *data)
{
	int *blank = (int *)data;
	struct cps_wls_chrg_chip *chip =
		container_of(nb, struct cps_wls_chrg_chip, cps_fb_notify);

	if (!blank) {
		cps_wls_log(CPS_LOG_ERR, "get disp statu err, blank is NULL!");
		return 0;
	}

	if (!chip) {
		return 0;
	}

	switch (event) {
	case MTK_DISP_EARLY_EVENT_BLANK:
		if (*blank == MTK_DISP_BLANK_UNBLANK)
			cps_set_led_on(chip, true);
		else if (*blank == MTK_DISP_BLANK_POWERDOWN)
			cps_set_led_on(chip, false);
		else
			cps_wls_log(CPS_LOG_ERR, "receives wrong data EARLY_BLANK:%d\n", *blank);
		break;
	case MTK_DISP_EVENT_BLANK:
		if (*blank == MTK_DISP_BLANK_UNBLANK)
			cps_set_led_on(chip, true);
		else if (*blank == MTK_DISP_BLANK_POWERDOWN)
			cps_set_led_on(chip, false);
		else
			cps_wls_log(CPS_LOG_ERR, "receives wrong data BLANK:%d\n", *blank);
		break;
	default:
		break;
	}

	return 0;
}

#endif /* CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG */
#endif /* CONFIG_DRM_PANEL_NOTIFY */

static int cps_register_lcd_notify(struct cps_wls_chrg_chip *chip)
{
	int rc = 0;

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
	int i;
	int count;
	struct device_node *node = NULL;
	struct drm_panel *panel = NULL;
	struct device_node *np = NULL;
	void *cookie = NULL;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null \n", __func__);
		return -EINVAL;
	}
	np = chip->dev->of_node;

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY)
	count = of_count_phandle_with_args(np, "oplus,display_panel", NULL);
	if (count <= 0) {
		cps_wls_log(CPS_LOG_ERR, "oplus,display_panel not found\n");
		return 0;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "oplus,display_panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			chip->active_panel = panel;
			rc = 0;
			cps_wls_log(CPS_LOG_ERR, "find active panel\n");
			break;
		} else {
			rc = PTR_ERR(panel);
		}
	}
#else
	np = of_find_node_by_name(NULL, "oplus,dsi-display-dev");
	if (!np) {
		cps_wls_log(CPS_LOG_ERR, "device tree info. missing\n");
		return 0;
	}

	count = of_count_phandle_with_args(np, "oplus,dsi-panel-primary", NULL);
	if (count <= 0) {
		cps_wls_log(CPS_LOG_ERR, "primary panel no found\n");
		return 0;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "oplus,dsi-panel-primary", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			chip->active_panel = panel;
			rc = 0;
			cps_wls_log(CPS_LOG_ERR, "find active_panel rc\n");
			break;
		} else {
			rc = PTR_ERR(panel);
		}
	}
#endif/* CONFIG_DRM_PANEL_NOTIFY */

	if (chip->active_panel) {
		cookie = panel_event_notifier_register(
				PANEL_EVENT_NOTIFICATION_PRIMARY,
				PANEL_EVENT_NOTIFIER_CLIENT_WLS_PEN_CHG,
				chip->active_panel, &cps_panel_notifier_callback,
				chip);
		if (!cookie) {
			cps_wls_log(CPS_LOG_ERR, "Unable to register chg_panel_notifier\n");
			return -EINVAL;
		} else {
			cps_wls_log(CPS_LOG_ERR, "success register chg_panel_notifier\n");
			chip->notifier_cookie = cookie;
		}
	} else {
		cps_wls_log(CPS_LOG_ERR, "can't find active panel, rc=%d\n", rc);
		if (rc == -EPROBE_DEFER)
			return rc;
		else
			return -ENODEV;
	}
#else /* CONFIG_DRM_PANEL_NOTIFY */

#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG)
	chip->cps_fb_notify.notifier_call = cps_mtk_drm_notifier_callback;
	if (mtk_disp_notifier_register("Oplus_Cps8601", &chip->cps_fb_notify)) {
		cps_wls_log(CPS_LOG_ERR, "Failed to register disp notifier client!!\n");
	}
#endif  /* CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG */
#endif  /* CONFIG_DRM_PANEL_NOTIFY */

	return rc;
}

static int cps_wls_chipid_check(struct cps_wls_chrg_chip *chip)
{
	int retry = 0;

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null \n", __func__);
		return -EINVAL;
	}

	for (retry = 0; retry < MAX_RETRY; retry++) {
		chip->chip_id = cps_wls_get_chip_id();
		if (chip->chip_id == CPS8601_CHIPID)
			return CPS_WLS_SUCCESS;
		msleep(100); /* wait for chip work*/
	}
	cps_wls_log(CPS_LOG_ERR, "[%s] chip id = 0x%x\n", __func__, chip->chip_id);
	return CPS_WLS_FAIL;
}

static void init_work_func(struct work_struct *work)
{
	int ret = 0, boost_work_retry = 0, fw_update_retry = 0;
	struct cps_wls_chrg_chip *chip =
		container_of(work, struct cps_wls_chrg_chip, init_work);

	if (!chip) {
		cps_wls_log(CPS_LOG_ERR, "[%s] chip null\n", __func__);
		return;
	}

	__pm_stay_awake(chip->cps_wls_wake_lock);

	if (chip->is_supply_by_hboost) {
		schedule_work(&chip->set_boost_work);
		while (1) {
			msleep(BOOST_SET_DELAY_500MS); /* wait for hboost setup success*/
			if (chip->hboost_status == HBOOST_SET_SUCCESS)
				break;
			else if (chip->hboost_status == HBOOST_SET_FAIL) {
				if (boost_work_retry < BOOST_WORK_MAX_RETRY) {
					if (chip->hboost_status != HBOOST_IS_SETTING_BOOST) {
						schedule_work(&chip->set_boost_work);
						boost_work_retry++;
					}
					continue;
				}
				cps_wls_log(CPS_LOG_ERR,
					"[%s]set boost fail:%d\n", __func__, ret);
				goto init_fail;
			}
		}
	} else {
		cps_set_gpio_value(chip, GP_4, 1);/* boost_en enable */
	}

	cps_set_gpio_value(chip, GP_2, 1); /* sw_en enable */

#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (get_Oplus_Boot_Mode() == FACTORY_MODE) {
		cps_wls_log(CPS_LOG_ERR, "[%s]ftm_mode\n", __func__);
		__pm_relax(chip->cps_wls_wake_lock);
		return;
	}
#endif /*CONFIG_OPLUS_CHARGER_MTK*/


	msleep(10); /* wait for chip work*/
	schedule_work(&chip->fw_update_work);
	while (1) {
		msleep(500); /* wait fw_update result */
		if (chip->fw_update_status == FW_UPDATE_SUCCESS)
			break;
		else if (chip->fw_update_status == FW_UPDATE_FAIL) {
			if (fw_update_retry < MAX_RETRY) {
				if (chip->fw_update_status != FW_UPDATE_ON_GOING) {
					schedule_work(&chip->fw_update_work);
					fw_update_retry++;
				}
				continue;
			}
			cps_wls_log(CPS_LOG_ERR, "[%s]fw update fail:%d\n", __func__, ret);
			goto init_fail;
		}
	}

	cps_set_gpio_value(chip, GP_0, 1);/* wakeup */
	msleep(TX_WAKEUP_WAIT_MS); /* wait for wakeup */
	if (cps_wls_chipid_check(chip) < 0) {
		cps_wls_log(CPS_LOG_ERR, "[%s] check chip id fail\n", __func__);
		goto init_fail;
	}
	chip->fw_version = cps_wls_get_sys_fw_version();
	cps_wls_log(CPS_LOG_DEBG, "[%s] chip id = 0x%x\n", __func__, chip->chip_id);


	ret = cps_wls_set_protect_parameter(chip);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR,
			"[%s] cps_wls_set_protect_parameter Fail ret = %d\n", __func__, ret);
		goto init_fail;
	}

	cps_set_led_on(chip, true);
	cps_set_charge_allow(chip, false);
	cps_get_cali_status(chip);

	if (chip->cps_wls_irq) {
		ret = devm_request_threaded_irq(chip->dev, chip->cps_wls_irq,
			NULL, cps_wls_irq_handler, IRQF_TRIGGER_FALLING| IRQF_ONESHOT,
				"cps_wls_irq", chip);
		if (ret) {
			cps_wls_log(CPS_LOG_ERR,
				"[%s] request cps_wls_int irq failed ret = %d\n", __func__, ret);
			goto init_fail;
		}
		enable_irq_wake(chip->cps_wls_irq);
	}
	cps_set_charge_allow(chip, true);

	schedule_delayed_work(&chip->lcd_notify_reg_work, round_jiffies_relative
		(msecs_to_jiffies(LCD_REG_DELAY_SEC * CPS_MSEC_PER_SEC)));

	cps_set_gpio_value(chip, GP_0, 0);
	cps_wls_hardware_reset();
	__pm_relax(chip->cps_wls_wake_lock);
	return;

init_fail:
	cps_set_gpio_value(chip, GP_2, 0); /* sw_en disable */
	if (chip->is_supply_by_hboost)
		cps_set_boost_volt_mv(MIN_BOOST_VOLT_MV);
	cps_wls_log(CPS_LOG_ERR, "[%s]init error\n", __func__);
	cps_wls_free_gpio(chip);
	cps_wls_lock_destroy(chip);
	__pm_relax(chip->cps_wls_wake_lock);
	i2c_set_clientdata(chip->client, NULL);
	dev_set_drvdata(chip->dev, NULL);
	dev_set_drvdata(chip->wireless_dev, NULL);
	g_chip = NULL;
	devm_kfree(chip->dev, chip);
}

#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
static int cps_wls_chrg_probe(struct i2c_client *client)
#else
static int cps_wls_chrg_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct cps_wls_chrg_chip *chip;
	int ret = 0;

	cps_wls_log(CPS_LOG_ERR, "[%s] ---->start\n", __func__);
	chip = devm_kzalloc(&client->dev, sizeof(struct cps_wls_chrg_chip), GFP_KERNEL);
	if (!chip) {
		cps_wls_log(CPS_LOG_ERR,
			"[%s] cps_debug: Unable to allocate memory\n", __func__);
		return -ENOMEM;
	}
	memset(chip, 0, sizeof(struct cps_wls_chrg_chip));
	chip->client = client;
	chip->dev = &client->dev;
	chip->name = "cps_wls";

	chip->regmap = devm_regmap_init_i2c(client, &cps8601_regmap_config);
	if (IS_ERR(chip->regmap)) {
		cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap!\n", __func__);
		devm_kfree(&client->dev, chip);
		return PTR_ERR(chip->regmap);
	}
	chip->regmap32 = devm_regmap_init_i2c(client, &cps8601_regmap32_config);
	if (IS_ERR(chip->regmap32)) {
		cps_wls_log(CPS_LOG_ERR, "[%s] Failed to allocate regmap!\n", __func__);
		devm_kfree(&client->dev, chip);
		return PTR_ERR(chip->regmap32);
	}

	i2c_set_clientdata(client, chip);
	dev_set_drvdata(&(client->dev), chip);
	chip->i2c_ready = true;
	g_chip = chip;

	cps_wls_lock_work_init(chip);

	ret = cps_wls_parse_dt(chip);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR,
			"[%s] Couldn't parse DT nodes ret = %d\n", __func__, ret);
		goto free_source;
	}

	ret = cps_wls_gpio_init(chip);
	if (ret < 0) {
		cps_wls_log(CPS_LOG_ERR,
			"[%s] cps_wls_gpio_init Fail ret = %d\n", __func__, ret);
		goto free_source;
	}

	INIT_DELAYED_WORK(&chip->max_chg_time_check_work, max_chg_time_check_work_func);
	INIT_DELAYED_WORK(&chip->lcd_notify_reg_work, lcd_notify_reg_work_func);
	INIT_WORK(&chip->set_boost_work, set_boost_work_func);
	INIT_WORK(&chip->fw_update_work, fw_update_work_func);
	INIT_WORK(&chip->rechg_work, rechg_work_func);
	INIT_WORK(&chip->q_cali_work, q_cali_work_func);
	INIT_WORK(&chip->error_attach_check_work, error_attach_check_work_func);
	INIT_WORK(&chip->init_work, init_work_func);

	ret = init_wireless_device(chip);
	if (ret < 0)
		cps_wls_log(CPS_LOG_ERR, "Create wireless charge device error.");

	schedule_work(&chip->init_work);
	if (chip->is_supply_by_hboost)
		wireless_pen_glink_init();

	cps_wls_log(CPS_LOG_ERR, "[%s]successful!\n", __func__);
	return ret;

free_source:
	cps_wls_free_gpio(chip);
	cps_wls_lock_destroy(chip);
	dev_set_drvdata(chip->wireless_dev, NULL);
	i2c_set_clientdata(client, NULL);
	dev_set_drvdata(&client->dev, NULL);
	g_chip = NULL;
	devm_kfree(&client->dev, chip);
	cps_wls_log(CPS_LOG_ERR, "[%s] error: free resource.\n", __func__);

	return ret;
}


static void not_called_api(void)
{
	int rc;
	uint8_t data[2] = {0x1F, 0xAC};
	rc = cps_wls_set_tx_fod_thresh(3000);
	rc = cps_wls_enable_tx_mode();
	rc = cps_wls_disable_tx_mode();
	rc = cps_wls_send_fsk_packet(data, 2);
	return;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
static int cps_wls_chrg_remove(struct i2c_client *client)
#else
static void cps_wls_chrg_remove(struct i2c_client *client)
#endif
{
	struct cps_wls_chrg_chip *chip;
	chip = i2c_get_clientdata(client);

	if (chip) {
		not_called_api();
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
		if (chip->active_panel && chip->notifier_cookie)
			panel_event_notifier_unregister(chip->notifier_cookie);
#endif /* CONFIG_DRM_PANEL_NOTIFY */
		cps_wls_lock_destroy(chip);
		if (chip->is_supply_by_hboost)
			wireless_pen_glink_exit();
		kfree(chip);
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
	return 0;
#else
	return;
#endif
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int cps8601_pm_resume(struct device *dev)
{
	struct cps_wls_chrg_chip *chip = dev_get_drvdata(dev);

	if (chip) {
		chip->i2c_ready = true;
		cps_wls_log(CPS_LOG_ERR, "[%s]cps8601_pm_resume.\n", __func__);
		wake_up_interruptible(&i2c_waiter);
		power_expired_do_check(chip);
	}

	return 0;
}

static int cps8601_pm_suspend(struct device *dev)
{
	struct cps_wls_chrg_chip *chip = dev_get_drvdata(dev);

	if (chip)
		chip->i2c_ready = false;

	return 0;
}

static const struct dev_pm_ops cps8601_pm_ops = {
	.resume = cps8601_pm_resume,
	.suspend = cps8601_pm_suspend,
};
#else
static int cps8601_resume(struct i2c_client *client)
{
	return 0;
}

static int cps8601_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
#endif


static const struct i2c_device_id cps_wls_charger_id[] = {
	{"cps-wls-charger", 0},
	{},
};

static const struct of_device_id cps_wls_chrg_of_tbl[] = {
	{ .compatible = "oplus,wls-charger-cps8601", .data = NULL},
	{},
};
MODULE_DEVICE_TABLE(i2c, cps_wls_charger_id);

static struct i2c_driver cps8601_driver = {
	.driver = {
		.name = CPS_WLS_CHRG_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cps_wls_chrg_of_tbl,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm = &cps8601_pm_ops,
#endif
	},
	.probe = cps_wls_chrg_probe,
	.remove = cps_wls_chrg_remove,
	.id_table = cps_wls_charger_id,
};

static int __init cps8601_init(void)
{
	return i2c_add_driver(&cps8601_driver);
}
module_init(cps8601_init);

static void __exit cps8601_exit(void)
{
	i2c_del_driver(&cps8601_driver);
}
module_exit(cps8601_exit);

MODULE_DESCRIPTION("Wireless Pen Charger Cps8601");
MODULE_LICENSE("GPL v2");
