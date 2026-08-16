// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/cpu.h>
#include <linux/version.h>
#include <linux/sched/clock.h>

#include "inc/pd_dbg_info.h"
#include "inc/tcpci.h"
#include "inc/tcpc_upm7610.h"

#if ENABLE_UPM7610_DBG
#define UPM7610_INFO(format, args...) \
	pd_dbg_info("%s() line-%d: " format, \
	__func__, __LINE__, ##args)
#else
#define UPM7610_INFO(foramt, args...)
#endif

#if IS_ENABLED(CONFIG_RT_REGMAP)
#include "inc/rt-regmap.h"
#endif /* CONFIG_RT_REGMAP */

static int upm7610_set_bist_test_mode(struct tcpc_device *tcpc, bool en);
static int upm7610_set_shutdown_power_mode(struct tcpc_device *tcpc, bool en);
extern void tcpc_late_sync(void);

#define UPM7610_DRV_VERSION	"2.0.6_MTK"

#define UPM7610_IRQ_WAKE_TIME	(500) /* ms */
#define UPM7610_DEINIT_TIME		(500) /* ms */

struct upm7610_chip {
	struct i2c_client *client;
	struct device *dev;
#if IS_ENABLED(CONFIG_RT_REGMAP)
	struct rt_regmap_device *m_dev;
#endif /* CONFIG_RT_REGMAP */
	struct tcpc_desc *tcpc_desc;
	struct tcpc_device *tcpc;

	int irq_gpio;
	int irq;
	int chip_id;
	int chip_func_sw;
	bool lpm_en;
};

#if IS_ENABLED(CONFIG_RT_REGMAP)
RT_REG_DECL(TCPC_V10_REG_VID, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_PID, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_DID, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_TYPEC_REV, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_PD_REV, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_PDIF_REV, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_ALERT, 2, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_ALERT_MASK, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_POWER_STATUS_MASK, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_FAULT_STATUS_MASK, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_TCPC_CTRL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_ROLE_CTRL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_FAULT_CTRL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_POWER_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_CC_STATUS, 1, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_POWER_STATUS, 1, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_FAULT_STATUS, 1, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_COMMAND, 1, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_MSG_HDR_INFO, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_RX_DETECT, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_RX_BYTE_CNT, 1, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_RX_BUF_FRAME_TYPE, 1, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_RX_HDR, 2, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_RX_DATA, 28, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_TRANSMIT, 1, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_TX_BYTE_CNT, 1, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_TX_HDR, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_TX_DATA, 28, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(UPM7610_REG_CC_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(UPM7610_REG_VDR_DEF_STATUS, 1, RT_VOLATILE, {});
RT_REG_DECL(UPM7610_REG_VDR_DEF_ALERT, 1, RT_VOLATILE, {});
RT_REG_DECL(UPM7610_REG_VDR_DEF_ALERT_MASK, 1, RT_VOLATILE, {});
RT_REG_DECL(UPM7610_REG_RESET_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(UPM7610_REG_HIDDEN_MODE, 1, RT_VOLATILE, {});
RT_REG_DECL(UPM7610_REG_DEBUG_B9, 1, RT_VOLATILE, {});
RT_REG_DECL(UPM7610_REG_TRIM_R4, 1, RT_VOLATILE, {});
RT_REG_DECL(UPM7610_REG_TRIM_R5, 1, RT_VOLATILE, {});
RT_REG_DECL(UPM7610_REG_TRIM_C6, 1, RT_VOLATILE, {});

static const rt_register_map_t upm7610_chip_regmap[] = {
	RT_REG(TCPC_V10_REG_VID),
	RT_REG(TCPC_V10_REG_PID),
	RT_REG(TCPC_V10_REG_DID),
	RT_REG(TCPC_V10_REG_TYPEC_REV),
	RT_REG(TCPC_V10_REG_PD_REV),
	RT_REG(TCPC_V10_REG_PDIF_REV),
	RT_REG(TCPC_V10_REG_ALERT),
	RT_REG(TCPC_V10_REG_ALERT_MASK),
	RT_REG(TCPC_V10_REG_POWER_STATUS_MASK),
	RT_REG(TCPC_V10_REG_FAULT_STATUS_MASK),
	RT_REG(TCPC_V10_REG_TCPC_CTRL),
	RT_REG(TCPC_V10_REG_ROLE_CTRL),
	RT_REG(TCPC_V10_REG_FAULT_CTRL),
	RT_REG(TCPC_V10_REG_POWER_CTRL),
	RT_REG(TCPC_V10_REG_CC_STATUS),
	RT_REG(TCPC_V10_REG_POWER_STATUS),
	RT_REG(TCPC_V10_REG_FAULT_STATUS),
	RT_REG(TCPC_V10_REG_COMMAND),
	RT_REG(TCPC_V10_REG_MSG_HDR_INFO),
	RT_REG(TCPC_V10_REG_RX_DETECT),
	RT_REG(TCPC_V10_REG_RX_BYTE_CNT),
	RT_REG(TCPC_V10_REG_RX_BUF_FRAME_TYPE),
	RT_REG(TCPC_V10_REG_RX_HDR),
	RT_REG(TCPC_V10_REG_RX_DATA),
	RT_REG(TCPC_V10_REG_TRANSMIT),
	RT_REG(TCPC_V10_REG_TX_BYTE_CNT),
	RT_REG(TCPC_V10_REG_TX_HDR),
	RT_REG(TCPC_V10_REG_TX_DATA),
	RT_REG(UPM7610_REG_CC_CTRL),
	RT_REG(UPM7610_REG_VDR_DEF_STATUS),
	RT_REG(UPM7610_REG_VDR_DEF_ALERT),
	RT_REG(UPM7610_REG_VDR_DEF_ALERT_MASK),
	RT_REG(UPM7610_REG_RESET_CTRL),
	RT_REG(UPM7610_REG_HIDDEN_MODE),
	RT_REG(UPM7610_REG_DEBUG_B9),
	RT_REG(UPM7610_REG_TRIM_R4),
	RT_REG(UPM7610_REG_TRIM_R5),
	RT_REG(UPM7610_REG_TRIM_C6),
};
#define UPM7610_CHIP_REGMAP_SIZE ARRAY_SIZE(upm7610_chip_regmap)

#endif /* CONFIG_RT_REGMAP */

static int upm7610_read_device(void *client, u32 reg, int len, void *dst)
{
	struct i2c_client *i2c = client;
	int ret = 0, count = 5;

	while (1) {
		ret = i2c_smbus_read_i2c_block_data(i2c, reg, len, dst);
		if (ret < 0 && count > 1)
			count--;
		else
			break;
		udelay(100);
	}
	return ret;
}

static int upm7610_write_device(void *client, u32 reg, int len, const void *src)
{
	struct i2c_client *i2c = client;
	int ret = 0, count = 5;

	while (1) {
		ret = i2c_smbus_write_i2c_block_data(i2c, reg, len, src);
		if (ret < 0 && count > 1)
			count--;
		else
			break;
		udelay(100);
	}
	return ret;
}

static int upm7610_reg_read(struct i2c_client *i2c, u8 reg)
{
	struct upm7610_chip *chip = i2c_get_clientdata(i2c);
	u8 val = 0;
	int ret = 0;

#if IS_ENABLED(CONFIG_RT_REGMAP)
	ret = rt_regmap_block_read(chip->m_dev, reg, 1, &val);
#else
	ret = upm7610_read_device(chip->client, reg, 1, &val);
#endif /* CONFIG_RT_REGMAP */
	if (ret < 0) {
		dev_err(chip->dev, "upm7610 reg read fail\n");
		return ret;
	}
	return val;
}

static int upm7610_reg_write(struct i2c_client *i2c, u8 reg, const u8 data)
{
	struct upm7610_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;

#if IS_ENABLED(CONFIG_RT_REGMAP)
	ret = rt_regmap_block_write(chip->m_dev, reg, 1, &data);
#else
	ret = upm7610_write_device(chip->client, reg, 1, &data);
#endif /* CONFIG_RT_REGMAP */
	if (ret < 0)
		dev_err(chip->dev, "upm7610 reg write fail\n");
	return ret;
}

static int upm7610_block_read(struct i2c_client *i2c, u8 reg, int len, void *dst)
{
	struct upm7610_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;
#if IS_ENABLED(CONFIG_RT_REGMAP)
	ret = rt_regmap_block_read(chip->m_dev, reg, len, dst);
#else
	ret = upm7610_read_device(chip->client, reg, len, dst);
#endif /* #if IS_ENABLED(CONFIG_RT_REGMAP) */
	if (ret < 0)
		dev_err(chip->dev, "upm7610 block read fail\n");
	return ret;
}

static int upm7610_block_write(struct i2c_client *i2c, u8 reg, int len, const void *src)
{
	struct upm7610_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;
#if IS_ENABLED(CONFIG_RT_REGMAP)
	ret = rt_regmap_block_write(chip->m_dev, reg, len, src);
#else
	ret = upm7610_write_device(chip->client, reg, len, src);
#endif /* #if IS_ENABLED(CONFIG_RT_REGMAP) */
	if (ret < 0)
		dev_err(chip->dev, "upm7610 block write fail\n");
	return ret;
}

static int32_t upm7610_write_word(struct i2c_client *client, uint8_t reg_addr, uint16_t data)
{
	int ret;

	/* don't need swap */
	ret = upm7610_block_write(client, reg_addr, 2, (uint8_t *)&data);
	return ret;
}

static int32_t upm7610_read_word(struct i2c_client *client,	uint8_t reg_addr, uint16_t *data)
{
	int ret;

	/* don't need swap */
	ret = upm7610_block_read(client, reg_addr, 2, (uint8_t *)data);
	return ret;
}

static inline int upm7610_i2c_write8(struct tcpc_device *tcpc, u8 reg, const u8 data)
{
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);

	return upm7610_reg_write(chip->client, reg, data);
}

static inline int upm7610_i2c_write16(struct tcpc_device *tcpc, u8 reg, const u16 data)
{
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);

	return upm7610_write_word(chip->client, reg, data);
}

static inline int upm7610_i2c_read8(struct tcpc_device *tcpc, u8 reg)
{
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);

	return upm7610_reg_read(chip->client, reg);
}

static inline int upm7610_i2c_read16(struct tcpc_device *tcpc, u8 reg)
{
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);
	u16 data;
	int ret;

	ret = upm7610_read_word(chip->client, reg, &data);
	if (ret < 0)
		return ret;
	return data;
}

#if IS_ENABLED(CONFIG_RT_REGMAP)
static struct rt_regmap_fops upm7610_regmap_fops = {
	.read_device = upm7610_read_device,
	.write_device = upm7610_write_device,
};
#endif /* CONFIG_RT_REGMAP */

static int upm7610_regmap_init(struct upm7610_chip *chip)
{
#if IS_ENABLED(CONFIG_RT_REGMAP)
	struct rt_regmap_properties *props;
	char name[32];
	int len;

	props = devm_kzalloc(chip->dev, sizeof(*props), GFP_KERNEL);
	if (!props)
		return -ENOMEM;

	props->register_num = UPM7610_CHIP_REGMAP_SIZE;
	props->rm = upm7610_chip_regmap;

	props->rt_regmap_mode = RT_MULTI_BYTE | RT_IO_PASS_THROUGH | RT_DBG_SPECIAL;
	snprintf(name, sizeof(name), "upm7610-%02x", chip->client->addr);

	len = strlen(name);
	props->name = devm_kzalloc(chip->dev, len+1, GFP_KERNEL);
	props->aliases = devm_kzalloc(chip->dev, len+1, GFP_KERNEL);

	if ((!props->name) || (!props->aliases))
		return -ENOMEM;

	strlcpy((char *)props->name, name, len+1);
	strlcpy((char *)props->aliases, name, len+1);
	props->io_log_en = 0;

	chip->m_dev = rt_regmap_device_register(props, &upm7610_regmap_fops,
				chip->dev, chip->client, chip);
	if (!chip->m_dev) {
		dev_err(chip->dev, "upm7610 chip rt_regmap register fail\n");
		return -EINVAL;
	}
#endif
	return 0;
}

static int upm7610_regmap_deinit(struct upm7610_chip *chip)
{
#if IS_ENABLED(CONFIG_RT_REGMAP)
	rt_regmap_device_unregister(chip->m_dev);
#endif
	return 0;
}

static inline int upm7610_software_reset(struct tcpc_device *tcpc)
{
	int ret = upm7610_i2c_write8(tcpc, UPM7610_REG_RESET_CTRL, 1);
#if IS_ENABLED(CONFIG_RT_REGMAP)
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0)
		return ret;
#if IS_ENABLED(CONFIG_RT_REGMAP)
	rt_regmap_cache_reload(chip->m_dev);
#endif /* CONFIG_RT_REGMAP */
	usleep_range(1000, 2000);
	return 0;
}

static inline int upm7610_command(struct tcpc_device *tcpc, uint8_t cmd)
{
	return upm7610_i2c_write8(tcpc, TCPC_V10_REG_COMMAND, cmd);
}

static int upm7610_init_alert_mask(struct tcpc_device *tcpc)
{
	uint16_t mask;
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);

	mask = TCPC_V10_REG_ALERT_CC_STATUS | TCPC_V10_REG_ALERT_POWER_STATUS;

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
	/* Need to handle RX overflow */
	mask |= TCPC_V10_REG_ALERT_TX_SUCCESS | TCPC_V10_REG_ALERT_TX_DISCARDED
			| TCPC_V10_REG_ALERT_TX_FAILED
			| TCPC_V10_REG_ALERT_RX_HARD_RST
			| TCPC_V10_REG_ALERT_RX_STATUS
			| TCPC_V10_REG_EXTENED_STATUS
			| TCPC_V10_REG_RX_OVERFLOW
			| TCPC_V10_REG_EXT_STATUS
			| TCPC_V10_REG_VBUS_SINK_DISCONNECT
			| TCPC_V10_REG_ALERT_VENDOR_DEFINED;
#endif

	mask |= TCPC_REG_ALERT_FAULT;

	return upm7610_write_word(chip->client, TCPC_V10_REG_ALERT_MASK, mask);
}

static int upm7610_rx_alert_mask(struct tcpc_device *tcpc)
{
	int ret;
	uint16_t mask;
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);

	ret = upm7610_i2c_read16(tcpc, TCPC_V10_REG_ALERT_MASK);
	if (ret < 0)
		return ret;

	UPM7610_INFO("get_mask from reg:0x%x\n", ret);
	mask = (uint16_t) ret;

	mask &= ~TCPC_V10_REG_ALERT_RX_STATUS;
	mask &= ~TCPC_V10_REG_RX_OVERFLOW;

	UPM7610_INFO("mask:0x%x\n", mask);
	return upm7610_write_word(chip->client, TCPC_V10_REG_ALERT_MASK, mask);
}

static int upm7610_init_power_status_mask(struct tcpc_device *tcpc)
{
	const uint8_t mask = TCPC_V10_REG_POWER_STATUS_VBUS_PRES;

	return upm7610_i2c_write8(tcpc, TCPC_V10_REG_POWER_STATUS_MASK, mask);
}

static int upm7610_init_fault_mask(struct tcpc_device *tcpc)
{
	const uint8_t mask = TCPC_V10_REG_FAULT_STATUS_VCONN_OV |
			TCPC_V10_REG_FAULT_STATUS_VCONN_OC;

	return upm7610_i2c_write8(tcpc, TCPC_V10_REG_FAULT_STATUS_MASK, mask);
}

static int upm7610_init_up_mask(struct tcpc_device *tcpc)
{
	uint8_t up_mask = 0;

	up_mask |= UPM7610_REG_VSAFE0V_STATUS_MASK;
	up_mask |= UPM7610_REG_REF_DISCNT_MASK;
#ifdef CONFIG_TYPEC_CAP_RA_DETACH
	if (tcpc->tcpc_flags & TCPC_FLAGS_CHECK_RA_DETACH)
		up_mask |= UPM7610_REG_RA_DETACH_MASK;
#endif /* CONFIG_TYPEC_CAP_RA_DETACH */

	return upm7610_i2c_write8(tcpc, UPM7610_REG_VDR_DEF_ALERT_MASK, up_mask);
}

static irqreturn_t upm7610_intr_handler(int irq, void *data)
{
	struct upm7610_chip *chip = data;

	pm_wakeup_event(chip->dev, UPM7610_IRQ_WAKE_TIME);

	tcpci_lock_typec(chip->tcpc);
	tcpci_alert(chip->tcpc);
	tcpci_unlock_typec(chip->tcpc);

	return IRQ_HANDLED;
}

static int upm7610_init_alert(struct tcpc_device *tcpc)
{
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);
	int ret = 0;
	char *name = NULL;

	/* Clear Alert Mask & Status */
	upm7610_write_word(chip->client, TCPC_V10_REG_ALERT_MASK, 0);
	upm7610_write_word(chip->client, TCPC_V10_REG_ALERT, 0xffff);

	name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s-IRQ", chip->tcpc_desc->name);
	if (!name)
		return -ENOMEM;

	dev_info(chip->dev, "%s name = %s, gpio = %d\n",
		__func__, chip->tcpc_desc->name, chip->irq_gpio);

	ret = devm_gpio_request(chip->dev, chip->irq_gpio, name);
	if (ret < 0) {
		dev_notice(chip->dev, "%s request GPIO fail(%d)\n", __func__, ret);
		return ret;
	}

	ret = gpio_direction_input(chip->irq_gpio);
	if (ret < 0) {
		dev_notice(chip->dev, "%s set GPIO fail(%d)\n", __func__, ret);
		return ret;
	}

	ret = gpio_to_irq(chip->irq_gpio);
	if (ret < 0) {
		dev_notice(chip->dev, "%s gpio to irq fail(%d)", __func__, ret);
		return ret;
	}
	chip->irq = ret;

	dev_info(chip->dev, "%s IRQ number = %d\n", __func__, chip->irq);

	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
					upm7610_intr_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					name, chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s request irq fail(%d)\n", __func__, ret);
		return ret;
	}
	device_init_wakeup(chip->dev, true);

	return 0;
}

int upm7610_alert_status_clear(struct tcpc_device *tcpc, uint32_t mask)
{
	int ret;
	uint16_t mask_t1;
	uint8_t mask_t2;

	mask_t2 = mask >> 16;
	if (mask_t2) {
		ret = upm7610_i2c_write8(tcpc, UPM7610_REG_VDR_DEF_ALERT, mask_t2);
		if (ret < 0)
			return ret;
	}

	mask_t1 = mask;
	if (mask_t1) {
		ret = upm7610_i2c_write16(tcpc, TCPC_V10_REG_ALERT, mask_t1);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int upm7610_set_clock_gating(struct tcpc_device *tcpc, bool en)
{
	int ret = 0;

#ifdef CONFIG_TCPC_CLOCK_GATING
	int i = 0;

	if (en) {
		for (i = 0; i < 2; i++)
			ret = upm7610_alert_status_clear(tcpc, TCPC_REG_ALERT_RX_ALL_MASK);
	}
#endif	/* CONFIG_TCPC_CLOCK_GATING */

	return ret;
}

static inline int upm7610_init_cc_params(struct tcpc_device *tcpc, uint8_t cc_res)
{
	int rv = 0;

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
#ifdef CONFIG_USB_PD_SNK_DFT_NO_GOOD_CRC
#endif	/* CONFIG_USB_PD_SNK_DFT_NO_GOOD_CRC */
#endif	/* CONFIG_USB_POWER_DELIVERY */

	return rv;
}

static int upm7610_tcpc_init(struct tcpc_device *tcpc, bool sw_reset)
{
	int ret;
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);
	int data = 0;
	uint8_t tmp1 = 0;

	if (sw_reset) {
		ret = upm7610_software_reset(tcpc);
		if (ret < 0)
			return ret;
	}

	upm7610_set_shutdown_power_mode(tcpc, false);

	/* UFP Both RD setting */
	/* DRP = 0, RpVal = 0 (Default), Rd, Rd */
	upm7610_i2c_write8(tcpc, TCPC_V10_REG_ROLE_CTRL,
			TCPC_V10_REG_ROLE_CTRL_RES_SET(0, 0, CC_RD, CC_RD));

	if (chip->chip_id == UPM7610_DID) {
		upm7610_i2c_write8(tcpc, TCPC_V10_REG_FAULT_CTRL,
				TCPC_V10_REG_FAULT_CTRL_DIS_VCONN_OV);
		upm7610_command(tcpc, TCPM_CMD_ENABLE_VBUS_DETECT);
	}

	/* RX/TX Clock Gating (Auto Mode)*/
	if (!sw_reset)
		upm7610_set_clock_gating(tcpc, true);

	tcpci_alert_status_clear(tcpc, 0xffffffff);

	upm7610_init_power_status_mask(tcpc);
	upm7610_init_alert_mask(tcpc);
	upm7610_init_fault_mask(tcpc);
	upm7610_init_up_mask(tcpc);

	chip->chip_func_sw = 0;

	upm7610_i2c_write8(tcpc, UPM7610_REG_HIDDEN_MODE, 0x6E);

	data = upm7610_i2c_read8(tcpc, UPM7610_REG_DEBUG_B9);
	if (data < 0)
		pr_err("upm7610: read 0xB9 fail, data:0x%x\n", data);
	UPM7610_INFO("upm7610: default trim data [0xB9]:0x%x\n", data);

	tmp1 = (uint8_t) (data & UPM7610_REG_FUNC_SW_MASK) >> UPM7610_REG_FUNC_SW_SHIFT;

	if (tmp1 == UPM7610_REG_FUNC_SW_ID) {
		chip->chip_func_sw =  1;
		upm7610_i2c_write8(tcpc, UPM7610_REG_HIDDEN_MODE, UPM7610_REG_HMODE_EXIT);
		UPM7610_INFO("upm7610: exit hidden mode\n");
		return 0;
	}

	data = upm7610_i2c_read8(tcpc, UPM7610_REG_TRIM_R5);
	if (data < 0)
        	pr_err("upm7610: read 0xC5 fail, data:0x%x\n", data);

	UPM7610_INFO("upm7610: default trim data [0xC5]:0x%x\n", data);

	tmp1 = (uint8_t) data & UPM7610_TRIM_R5_WIN_MASK;

	if (tmp1 != UPM7610_TRIM_R5_WIN_ID)
		tmp1 = UPM7610_TRIM_R5_WIN_ID;

	data = (data & (~UPM7610_TRIM_R5_WIN_MASK)) | tmp1;
	UPM7610_INFO("upm7610: new trim data [0xC5]:0x%x\n", data);

	upm7610_i2c_write8(tcpc, UPM7610_REG_TRIM_R5, data);

	data = upm7610_i2c_read8(tcpc, UPM7610_REG_TRIM_C6);
	if (data < 0)
        	pr_err("upm7610: read 0xC6 fail, data:0x%x\n", data);

	UPM7610_INFO("upm7610: default trim data [0xC6]:0x%x\n", data);

	tmp1 = (uint8_t) data & UPM7610_TRIM_C6_WIN_MASK;
		if (tmp1 != UPM7610_TRIM_C6_WIN_ID)
			tmp1 = UPM7610_TRIM_C6_WIN_ID;

		data =  (data & (~UPM7610_TRIM_C6_WIN_MASK)) | tmp1;
		UPM7610_INFO("upm7610: new trim data [0xC6]:0x%x\n", data);

		upm7610_i2c_write8(tcpc, UPM7610_REG_TRIM_C6, data);

		upm7610_i2c_write8(tcpc, UPM7610_REG_HIDDEN_MODE, UPM7610_REG_HMODE_EXIT);

	mdelay(1);
	return 0;
}

int upm7610_fault_status_clear(struct tcpc_device *tcpc, uint8_t status)
{
	upm7610_i2c_write8(tcpc, TCPC_V10_REG_FAULT_STATUS, status);
	return 0;
}

int upm7610_get_alert_mask(struct tcpc_device *tcpc, uint32_t *mask)
{
	int ret;
	uint8_t v2;

	ret = upm7610_i2c_read16(tcpc, TCPC_V10_REG_ALERT_MASK);
	if (ret < 0)
		return ret;

	*mask = (uint16_t) ret;

	ret = upm7610_i2c_read8(tcpc, UPM7610_REG_VDR_DEF_ALERT_MASK);
	if (ret < 0)
		return ret;

	v2 = (uint8_t) ret;
	*mask |= v2 << 16;

	return 0;
}

int upm7610_get_alert_status(struct tcpc_device *tcpc, uint32_t *alert)
{
	int ret;
	uint8_t v2;

	ret = upm7610_i2c_read16(tcpc, TCPC_V10_REG_ALERT);
	if (ret < 0)
		return ret;

	*alert = (uint16_t) ret;

	ret = upm7610_i2c_read8(tcpc, UPM7610_REG_VDR_DEF_ALERT);
	if (ret < 0)
		return ret;

	v2 = (uint8_t) ret;
	*alert |= v2 << 16;

	return 0;
}

static int upm7610_get_power_status(
		struct tcpc_device *tcpc, uint16_t *pwr_status)
{
	int ret;

	ret = upm7610_i2c_read8(tcpc, TCPC_V10_REG_POWER_STATUS);
	if (ret < 0)
		return ret;

	*pwr_status = 0;

	if (ret & TCPC_V10_REG_POWER_STATUS_VBUS_PRES)
		*pwr_status |= TCPC_REG_POWER_STATUS_VBUS_PRES;

	ret = upm7610_i2c_read8(tcpc, UPM7610_REG_VDR_DEF_STATUS);
	if (ret < 0)
		return ret;

	if (ret & UPM7610_REG_VSAFE0V_STATUS)
		*pwr_status |= TCPC_REG_POWER_STATUS_EXT_VSAFE0V;

	return 0;
}

int upm7610_get_fault_status(struct tcpc_device *tcpc, uint8_t *status)
{
	int ret;

	ret = upm7610_i2c_read8(tcpc, TCPC_V10_REG_FAULT_STATUS);
	if (ret < 0)
		return ret;
	*status = (uint8_t) ret;
	return 0;
}

static int upm7610_get_cc(struct tcpc_device *tcpc, int *cc1, int *cc2)
{
	int status, role_ctrl, cc_role;
	bool act_as_sink, act_as_drp;

	status = upm7610_i2c_read8(tcpc, TCPC_V10_REG_CC_STATUS);
	if (status < 0)
		return status;

	role_ctrl = upm7610_i2c_read8(tcpc, TCPC_V10_REG_ROLE_CTRL);
	if (role_ctrl < 0)
		return role_ctrl;

	*cc1 = TCPC_V10_REG_CC_STATUS_CC1(status);
	*cc2 = TCPC_V10_REG_CC_STATUS_CC2(status);

	act_as_drp = TCPC_V10_REG_ROLE_CTRL_DRP & role_ctrl;

	if (act_as_drp)
		act_as_sink = TCPC_V10_REG_CC_STATUS_DRP_RESULT(status);
	else {
		if (tcpc->typec_polarity)
			cc_role = TCPC_V10_REG_CC_STATUS_CC2(role_ctrl);
		else
			cc_role = TCPC_V10_REG_CC_STATUS_CC1(role_ctrl);
		if (cc_role == TYPEC_CC_RP)
			act_as_sink = false;
		else
			act_as_sink = true;
	}

	/*
	 * If status is not open, then OR in termination to convert to
	 * enum tcpc_cc_voltage_status.
	 */

	if (*cc1 != TYPEC_CC_VOLT_OPEN)
		*cc1 |= (act_as_sink << 2);

	if (*cc2 != TYPEC_CC_VOLT_OPEN)
		*cc2 |= (act_as_sink << 2);

	upm7610_init_cc_params(tcpc, (uint8_t)tcpc->typec_polarity ? *cc2 : *cc1);

	return 0;
}

static int upm7610_enable_vsafe0v_detect(struct tcpc_device *tcpc, bool enable)
{
	int val1 = 0, val2 = 0;

	val1 = upm7610_i2c_read8(tcpc, UPM7610_REG_VDR_DEF_ALERT_MASK);
	if (val1 < 0)
		return val1;

	val2 = upm7610_i2c_read8(tcpc, TCPC_V10_REG_ALERT_MASK+1);
	if (val2 < 0)
		return val2;

	 if (enable) {
		val1 |= (UPM7610_REG_VSAFE0V_STATUS_MASK);
		val2 |= (TCPC_V10_REG_EXTENDED_STATUS>>8);
	} else {
		val1 &= ~(UPM7610_REG_VSAFE0V_STATUS_MASK);
		val2 &= ~(TCPC_V10_REG_EXTENDED_STATUS>>8);
	}

	upm7610_i2c_write8(tcpc, UPM7610_REG_VDR_DEF_ALERT_MASK, (uint8_t) val1);
	upm7610_i2c_write8(tcpc, TCPC_V10_REG_ALERT_MASK+1, (uint8_t) val2);

	return 0;
}

static int upm7610_set_cc(struct tcpc_device *tcpc, int pull)
{
	int ret;
	uint8_t data;
	int rp_lvl = TYPEC_CC_PULL_GET_RP_LVL(pull), pull1, pull2;

	UPM7610_INFO("pull = 0x%02X\n", pull);
	pull = TYPEC_CC_PULL_GET_RES(pull);
	if (pull == TYPEC_CC_DRP) {
		data = TCPC_V10_REG_ROLE_CTRL_RES_SET(1, rp_lvl, TYPEC_CC_RD, TYPEC_CC_RD);

		ret = upm7610_i2c_write8(tcpc, TCPC_V10_REG_ROLE_CTRL, data);

		if (ret == 0) {
			upm7610_enable_vsafe0v_detect(tcpc, false);
			ret = upm7610_command(tcpc, TCPM_CMD_LOOK_CONNECTION);
		}
	} else {
#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
		if (pull == TYPEC_CC_RD && tcpc->pd_wait_pr_swap_complete)
			upm7610_init_cc_params(tcpc, TYPEC_CC_VOLT_SNK_DFT);
#endif	/* CONFIG_USB_POWER_DELIVERY */

		pull1 = pull2 = pull;

		if (pull == TYPEC_CC_RP && tcpc->typec_is_attached_src) {
			if (tcpc->typec_polarity)
				pull1 = TYPEC_CC_OPEN;
			else
				pull2 = TYPEC_CC_OPEN;
		}
		data = TCPC_V10_REG_ROLE_CTRL_RES_SET(0, rp_lvl, pull1, pull2);
		ret = upm7610_i2c_write8(tcpc, TCPC_V10_REG_ROLE_CTRL, data);
	}

	return 0;
}

static int upm7610_set_polarity(struct tcpc_device *tcpc, int polarity)
{
	int data;

	if (polarity >= 0 && polarity < ARRAY_SIZE(tcpc->typec_remote_cc)) {
		data = upm7610_init_cc_params(tcpc, tcpc->typec_remote_cc[polarity]);
		if (data)
			return data;
	}

	data = upm7610_i2c_read8(tcpc, TCPC_V10_REG_TCPC_CTRL);
	if (data < 0)
		return data;

	data &= ~TCPC_V10_REG_TCPC_CTRL_PLUG_ORIENT;
	data |= polarity ? TCPC_V10_REG_TCPC_CTRL_PLUG_ORIENT : 0;

	return upm7610_i2c_write8(tcpc, TCPC_V10_REG_TCPC_CTRL, data);
}

static int upm7610_set_vconn(struct tcpc_device *tcpc, int enable)
{
	int rv;
	int data;

	data = upm7610_i2c_read8(tcpc, TCPC_V10_REG_POWER_CTRL);
	if (data < 0)
		return data;

	data &= ~TCPC_V10_REG_POWER_CTRL_VCONN;
	data |= enable ? TCPC_V10_REG_POWER_CTRL_VCONN : 0;

	rv = upm7610_i2c_write8(tcpc, TCPC_V10_REG_POWER_CTRL, data);
	if (rv < 0)
		return rv;

	return 0;
}

static int upm7610_is_low_power_mode(struct tcpc_device *tcpc)
{
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);

	return chip->lpm_en;
}

static int upm7610_set_low_power_mode(struct tcpc_device *tcpc, bool en, int pull)
{
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);
	chip->lpm_en = en;
	upm7610_enable_vsafe0v_detect(tcpc, !en);
	return 0;
}
static int upm7610_set_shutdown_power_mode(struct tcpc_device *tcpc, bool en)
{
	int data = 0;

	data = upm7610_i2c_read8(tcpc, UPM7610_REG_CC_CTRL);
	if (data < 0) {
		pr_err("%s: read CC_CTRL fail, data:%d\n", __func__, data);
		return data;
	}

	UPM7610_INFO("upm7610: cc_ctrl:0x%x\n", data);

	if (en) {
		data |= UPM7610_REG_DISABLED_REQ;
		UPM7610_INFO("upm7610: enter shutdown mode, cc_ctrl:0x%x\n", data);
	} else {
		data &= ~UPM7610_REG_DISABLED_REQ;
		UPM7610_INFO("upm7610: exit shutdown mode, cc_ctrl:0x%x\n", data);
	}

	return upm7610_i2c_write8(tcpc, UPM7610_REG_CC_CTRL, data);
}

static int upm7610_tcpc_deinit(struct tcpc_device *tcpc)
{
#if IS_ENABLED(CONFIG_RT_REGMAP)
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);
#endif /* CONFIG_RT_REGMAP */
	UPM7610_INFO("upm7610: %s", __func__);
#ifdef CONFIG_TCPC_SHUTDOWN_CC_DETACH
	upm7610_set_cc(tcpc, TYPEC_CC_DRP);
	upm7610_set_cc(tcpc, TYPEC_CC_OPEN);
	msleep(UPM7610_DEINIT_TIME);
#else
	upm7610_i2c_write8(tcpc, UPM7610_REG_RESET_CTRL, 1);
#endif	/* CONFIG_TCPC_SHUTDOWN_CC_DETACH */
#if IS_ENABLED(CONFIG_RT_REGMAP)
	rt_regmap_cache_reload(chip->m_dev);
#endif /* CONFIG_RT_REGMAP */
		upm7610_set_shutdown_power_mode(tcpc, true);

	return 0;
}

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
static int upm7610_set_msg_header(struct tcpc_device *tcpc, uint8_t power_role, uint8_t data_role)
{
	uint8_t msg_hdr = TCPC_V10_REG_MSG_HDR_INFO_SET(data_role, power_role);

	return upm7610_i2c_write8(tcpc, TCPC_V10_REG_MSG_HDR_INFO, msg_hdr);
}

static int upm7610_set_rx_enable(struct tcpc_device *tcpc, uint8_t enable)
{
	int ret = 0;

	if (enable)
		ret = upm7610_set_clock_gating(tcpc, false);

	if (ret == 0)
		ret = upm7610_i2c_write8(tcpc, TCPC_V10_REG_RX_DETECT, enable);

	if ((ret == 0) && (!enable)) {
		ret = upm7610_set_clock_gating(tcpc, true);
	}

	return ret;
}

static int upm7610_get_message(struct tcpc_device *tcpc, uint32_t *payload,
			uint16_t *msg_head, enum tcpm_transmit_type *frame_type)
{
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);
	int rv = 0;
	uint8_t cnt = 0, buf[4];

	rv = upm7610_block_read(chip->client, TCPC_V10_REG_RX_BYTE_CNT, 4, buf);
	if (rv < 0)
		return rv;

	cnt = buf[0];
	*frame_type = buf[1];
	*msg_head = le16_to_cpu(*(uint16_t *)&buf[2]);

	if (*msg_head == 0x77a3)
		upm7610_set_bist_test_mode(tcpc, true);
	/* TCPC 1.0 ==> no need to subtract the size of msg_head */
	if (cnt > 3) {
		cnt -= 3; /* MSG_HDR */
		rv = upm7610_block_read(chip->client, TCPC_V10_REG_RX_DATA, cnt,
				       payload);
	}

	return rv;
}

static int upm7610_set_bist_carrier_mode(
	struct tcpc_device *tcpc, uint8_t pattern)
{
	/* Don't support this function */
	return 0;
}

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
static int upm7610_retransmit(struct tcpc_device *tcpc)
{
	return upm7610_i2c_write8(tcpc, TCPC_V10_REG_TRANSMIT,
			TCPC_V10_REG_TRANSMIT_SET(
			tcpc->pd_retry_count, TCPC_TX_SOP));
}
#endif

#pragma pack(push, 1)
struct tcpc_transmit_packet {
	uint8_t cnt;
	uint16_t msg_header;
	uint8_t data[sizeof(uint32_t)*7];
};
#pragma pack(pop)

static int upm7610_transmit(struct tcpc_device *tcpc,
	enum tcpm_transmit_type type, uint16_t header, const uint32_t *data)
{
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);
	int rv;
	int data_cnt;
	struct tcpc_transmit_packet packet;

	if (type < TCPC_TX_HARD_RESET) {
		data_cnt = sizeof(uint32_t) * PD_HEADER_CNT(header);

		packet.cnt = data_cnt + sizeof(uint16_t);
		packet.msg_header = header;

		if (data_cnt > 0)
			memcpy(packet.data, (uint8_t *) data, data_cnt);

		rv = upm7610_block_write(chip->client,
				TCPC_V10_REG_TX_BYTE_CNT,
				packet.cnt+1, (uint8_t *) &packet);
		if (rv < 0)
			return rv;
	}

	rv = upm7610_i2c_write8(tcpc, TCPC_V10_REG_TRANSMIT,
			TCPC_V10_REG_TRANSMIT_SET(
			tcpc->pd_retry_count, type));
	return rv;
}

static int upm7610_set_bist_test_mode(struct tcpc_device *tcpc, bool en)
{
	struct upm7610_chip *chip = tcpc_get_dev_data(tcpc);
	int data;

	if (en)
		upm7610_rx_alert_mask(tcpc);
	else {
		if (chip->chip_func_sw == 0) {
			upm7610_tcpc_init(tcpc, true);
			upm7610_set_cc(tcpc, TYPEC_CC_DRP);
		}
		upm7610_i2c_write16(tcpc, TCPC_V10_REG_ALERT, 0x0404);
		upm7610_i2c_write16(tcpc, TCPC_V10_REG_ALERT, 0x0404);
		upm7610_init_alert_mask(tcpc);
	}

	data = upm7610_i2c_read8(tcpc, TCPC_V10_REG_TCPC_CTRL);
	if (data < 0)
		return data;

	data &= ~TCPC_V10_REG_TCPC_CTRL_BIST_TEST_MODE;
	data |= en ? TCPC_V10_REG_TCPC_CTRL_BIST_TEST_MODE : 0;

	return upm7610_i2c_write8(tcpc, TCPC_V10_REG_TCPC_CTRL, data);
}
#endif /* CONFIG_USB_POWER_DELIVERY */

static struct tcpc_ops upm7610_tcpc_ops = {
	.init = upm7610_tcpc_init,
	.alert_status_clear = upm7610_alert_status_clear,
	.fault_status_clear = upm7610_fault_status_clear,
	.get_alert_mask = upm7610_get_alert_mask,
	.get_alert_status = upm7610_get_alert_status,
	.get_power_status = upm7610_get_power_status,
	.get_fault_status = upm7610_get_fault_status,
	.get_cc = upm7610_get_cc,
	.set_cc = upm7610_set_cc,
	.set_polarity = upm7610_set_polarity,
	.set_vconn = upm7610_set_vconn,
	.deinit = upm7610_tcpc_deinit,
	.init_alert_mask = upm7610_init_alert_mask,
	.is_low_power_mode = upm7610_is_low_power_mode,
	.set_low_power_mode = upm7610_set_low_power_mode,

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
	.set_msg_header = upm7610_set_msg_header,
	.set_rx_enable = upm7610_set_rx_enable,
	.get_message = upm7610_get_message,
	.transmit = upm7610_transmit,
	.set_bist_test_mode = upm7610_set_bist_test_mode,
	.set_bist_carrier_mode = upm7610_set_bist_carrier_mode,
#endif	/* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
	.retransmit = upm7610_retransmit,
#endif	/* CONFIG_USB_PD_RETRY_CRC_DISCARD */
};

static int rt_parse_dt(struct upm7610_chip *chip, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	pr_info("%s\n", __func__);
	chip->irq_gpio = of_get_named_gpio(np, "upm7610pd,intr_gpio", 0);
	return ret < 0 ? ret : 0;
}

/*
 * In some platform pr_info may spend too much time on printing debug message.
 * So we use this function to test the printk performance.
 * If your platform cannot not pass this check function, please config
 * PD_DBG_INFO, this will provide the threaded debug message for you.
 */
#if TCPC_ENABLE_ANYMSG
static void check_printk_performance(void)
{
	int i;
	u64 t1, t2;
	u32 nsrem;

#if IS_ENABLED(CONFIG_PD_DBG_INFO)
	for (i = 0; i < 10; i++) {
		t1 = local_clock();
		pd_dbg_info("%d\n", i);
		t2 = local_clock();
		t2 -= t1;
		nsrem = do_div(t2, 1000000000);
		pd_dbg_info("pd_dbg_info : t2-t1 = %lu\n",
				(unsigned long)nsrem / 1000);
	}
	for (i = 0; i < 10; i++) {
		t1 = local_clock();
		pr_info("%d\n", i);
		t2 = local_clock();
		t2 -= t1;
		nsrem = do_div(t2, 1000000000);
		pr_info("pr_info : t2-t1 = %lu\n",
				(unsigned long)nsrem / 1000);
	}
#else
	for (i = 0; i < 10; i++) {
		t1 = local_clock();
		pr_info("%d\n", i);
		t2 = local_clock();
		t2 -= t1;
		nsrem = do_div(t2, 1000000000);
		pr_info("t2-t1 = %lu\n",
				(unsigned long)nsrem /  1000);
		PD_BUG_ON(nsrem > 100*1000);
	}
#endif /* CONFIG_PD_DBG_INFO */
}
#endif /* TCPC_ENABLE_ANYMSG */

static int upm7610_tcpcdev_init(struct upm7610_chip *chip, struct device *dev)
{
	struct tcpc_desc *desc;
	struct device_node *np = dev->of_node;
	u32 val, len;
	const char *name = "default";

	dev_info(dev, "%s\n", __func__);

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	if (of_property_read_u32(np, "rt-tcpc,role_def", &val) >= 0) {
		if (val >= TYPEC_ROLE_NR)
			desc->role_def = TYPEC_ROLE_DRP;
		else
			desc->role_def = val;
	} else {
		dev_info(dev, "use default Role DRP\n");
		desc->role_def = TYPEC_ROLE_DRP;
	}

	if (of_property_read_u32(np, "rt-tcpc,rp_level", &val) >= 0) {
		switch (val) {
		case TYPEC_RP_DFT:
		case TYPEC_RP_1_5:
		case TYPEC_RP_3_0:
			desc->rp_lvl = val;
			break;
		default:
			break;
		}
	}

#ifdef CONFIG_TCPC_VCONN_SUPPLY_MODE
	if (of_property_read_u32(np, "rt-tcpc,vconn_supply", &val) >= 0) {
		if (val >= TCPC_VCONN_SUPPLY_NR)
			desc->vconn_supply = TCPC_VCONN_SUPPLY_ALWAYS;
		else
			desc->vconn_supply = val;
	} else {
		dev_info(dev, "use default VconnSupply\n");
		desc->vconn_supply = TCPC_VCONN_SUPPLY_ALWAYS;
	}
#endif	/* CONFIG_TCPC_VCONN_SUPPLY_MODE */

	if (of_property_read_string(np, "rt-tcpc,name", (char const **)&name) < 0)
		dev_info(dev, "use default name\n");

	len = strlen(name);
	desc->name = kzalloc(len+1, GFP_KERNEL);
	if (!desc->name)
		return -ENOMEM;

	strlcpy((char *)desc->name, name, len+1);

	chip->tcpc_desc = desc;

	chip->tcpc = tcpc_device_register(dev, desc, &upm7610_tcpc_ops, chip);
	if (IS_ERR_OR_NULL(chip->tcpc))
		return -EINVAL;

#ifdef CONFIG_USB_PD_DISABLE_PE
	chip->tcpc->disable_pe = of_property_read_bool(np, "rt-tcpc,disable_pe");
#endif	/* CONFIG_USB_PD_DISABLE_PE */

	chip->tcpc->tcpc_flags = TCPC_FLAGS_VCONN_SAFE5V_ONLY | TCPC_FLAGS_CHECK_RA_DETACH;

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
	chip->tcpc->tcpc_flags |= TCPC_FLAGS_RETRY_CRC_DISCARD;
#endif  /* CONFIG_USB_PD_RETRY_CRC_DISCARD */

#ifdef CONFIG_USB_PD_REV30
	chip->tcpc->tcpc_flags |= TCPC_FLAGS_PD_REV30;

	if (chip->tcpc->tcpc_flags & TCPC_FLAGS_PD_REV30)
		dev_info(dev, "PD_REV30\n");
	else
		dev_info(dev, "PD_REV20\n");
#endif	/* CONFIG_USB_PD_REV30 */
	chip->tcpc->tcpc_flags |= TCPC_FLAGS_ALERT_V10;
	return 0;
}

#define UPM_7610_VID	0x362f
#define UPM_7610_PID	0x7610

static inline int upm7610_check_revision(struct i2c_client *client)
{
	u16 vid, pid, did;
	int ret;
	u8 data = 1;

	ret = upm7610_read_device(client, TCPC_V10_REG_VID, 2, &vid);
	if (ret < 0) {
		dev_err(&client->dev, "read chip ID fail\n");
		return -EIO;
	}

	if (vid != UPM_7610_VID) {
		pr_info("%s failed, VID=0x%04x\n", __func__, vid);
		return -ENODEV;
	}

	ret = upm7610_read_device(client, TCPC_V10_REG_PID, 2, &pid);
	if (ret < 0) {
		dev_err(&client->dev, "read product ID fail\n");
		return -EIO;
	}

	if (pid != UPM_7610_PID) {
		pr_info("%s failed, PID=0x%04x\n", __func__, pid);
		return -ENODEV;
	}

	ret = upm7610_write_device(client, UPM7610_REG_RESET_CTRL, 1, &data);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	ret = upm7610_read_device(client, TCPC_V10_REG_DID, 2, &did);
	if (ret < 0) {
		dev_err(&client->dev, "read device ID fail\n");
		return -EIO;
	}

	return did;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static int upm7610_i2c_probe(struct i2c_client *client)
#else
static int upm7610_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct upm7610_chip *chip;
	int ret = 0, chip_id;
	bool use_dt = client->dev.of_node;

	pr_info("%s (%s)\n", __func__, UPM7610_DRV_VERSION);
	if (i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_I2C_BLOCK | I2C_FUNC_SMBUS_BYTE_DATA))
		pr_info("I2C functionality : OK...\n");
	else
		pr_info("I2C functionality check : failuare...\n");

	chip_id = upm7610_check_revision(client);
	if (chip_id < 0)
		return chip_id;

#if TCPC_ENABLE_ANYMSG
	check_printk_performance();
#endif /* TCPC_ENABLE_ANYMSG */

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (use_dt) {
		ret = rt_parse_dt(chip, &client->dev);
		if (ret < 0)
			return ret;
	} else {
		dev_err(&client->dev, "no dts node\n");
		return -ENODEV;
	}
	chip->dev = &client->dev;
	chip->client = client;
	i2c_set_clientdata(client, chip);
	chip->chip_id = chip_id;
	pr_info("upm7610_chipID = 0x%0x\n", chip_id);

	ret = upm7610_regmap_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "upm7610 regmap init fail\n");
		goto err_regmap_init;
	}

	ret = upm7610_tcpcdev_init(chip, &client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "upm7610 tcpc dev init fail\n");
		goto err_tcpc_reg;
	}

	ret = upm7610_init_alert(chip->tcpc);
	if (ret < 0) {
		pr_err("upm7610 init alert fail\n");
		goto err_irq_init;
	}

	pr_info("%s probe OK!\n", __func__);
	return 0;

err_irq_init:
	tcpc_device_unregister(chip->dev, chip->tcpc);
err_tcpc_reg:
	upm7610_regmap_deinit(chip);
err_regmap_init:
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static void upm7610_i2c_remove(struct i2c_client *client)
{
	struct upm7610_chip *chip = i2c_get_clientdata(client);

	if (chip) {
		tcpc_device_unregister(chip->dev, chip->tcpc);
		upm7610_regmap_deinit(chip);
	}
}
#else
static int upm7610_i2c_remove(struct i2c_client *client)
{
	struct upm7610_chip *chip = i2c_get_clientdata(client);

	if (chip) {
		tcpc_device_unregister(chip->dev, chip->tcpc);
		upm7610_regmap_deinit(chip);
	}

	return 0;
}
#endif

#if CONFIG_PM
static int upm7610_i2c_suspend(struct device *dev)
{
	struct upm7610_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irq);

	return 0;
}

static int upm7610_i2c_resume(struct device *dev)
{
	struct upm7610_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	enable_irq(chip->irq);
	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irq);

	return 0;
}

static void upm7610_shutdown(struct i2c_client *client)
{
	struct upm7610_chip *chip = i2c_get_clientdata(client);

	/* Please reset IC here */
	if (chip != NULL) {
		if (chip->irq) {
			disable_irq(chip->irq);
			tcpm_shutdown(chip->tcpc);
		}
	} else {
		i2c_smbus_write_byte_data(client, UPM7610_REG_RESET_CTRL, 0x01);
		i2c_smbus_write_byte_data(client, UPM7610_REG_CC_CTRL, 0x44);
	}
}

#if IS_ENABLED(CONFIG_PM_RUNTIME)
static int upm7610_pm_suspend_runtime(struct device *device)
{
	dev_dbg(device, "pm_runtime: suspending...\n");
	return 0;
}

static int upm7610_pm_resume_runtime(struct device *device)
{
	dev_dbg(device, "pm_runtime: resuming...\n");
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static const struct dev_pm_ops upm7610_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(upm7610_i2c_suspend, upm7610_i2c_resume)
#if IS_ENABLED(CONFIG_PM_RUNTIME)
	SET_RUNTIME_PM_OPS(upm7610_pm_suspend_runtime,
		upm7610_pm_resume_runtime,
		NULL)
#endif /* CONFIG_PM_RUNTIME */
};
#define UPM7610_PM_OPS	(&upm7610_pm_ops)
#else
#define UPM7610_PM_OPS	(NULL)
#endif /* CONFIG_PM */
static const struct i2c_device_id upm7610_id_table[] = {
	{"upm7610", 0},
	{"rt1715", 0},
	{"rt1716", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, upm7610_id_table);

static const struct of_device_id up_match_table[] = {
	{.compatible = "up,upm7610"},
	{},
};

static struct i2c_driver upm7610_driver = {
	.driver = {
		.name = "upm7610",
		.owner = THIS_MODULE,
		.of_match_table = up_match_table,
		.pm = UPM7610_PM_OPS,
	},
	.probe = upm7610_i2c_probe,
	.remove = upm7610_i2c_remove,
	.shutdown = upm7610_shutdown,
	.id_table = upm7610_id_table,
};

static int __init upm7610_init(void)
{
	return i2c_add_driver(&upm7610_driver);
}
subsys_initcall(upm7610_init);

static void __exit upm7610_exit(void)
{
	i2c_del_driver(&upm7610_driver);
}
module_exit(upm7610_exit);

MODULE_LICENSE("GPL");
MODULE_VERSION(UPM7610_DRV_VERSION);


