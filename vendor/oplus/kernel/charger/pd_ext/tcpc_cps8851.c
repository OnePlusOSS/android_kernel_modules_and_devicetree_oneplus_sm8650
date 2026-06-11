/*
 * Copyright (C) 2020 Richtek Inc.
 *
 * Richtek CPS8851H Type-C Port Control Driver
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
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
#include "inc/cps8851.h"

#ifdef CONFIG_RT_REGMAP
#include "inc/rt-regmap.h"
#endif /* CONFIG_RT_REGMAP */

#define CPS8851H_DRV_VERSION	"2.0.6_G"

#define CPS8851H_IRQ_WAKE_TIME	(1000) /* ms */
#define CPS8851_VBUS_PRES_DEB_TIME	500 /* ms */

#define HUSB311_VID     0x2e99
#define HUSB311_PID     0x0311
#define HUSB311_DID     0x0000
#define CPS8851_VID     0x315C
#define CPS8851_PID     0x8851
#define CPS8851_DID     0x2173

#define PD_MSG_CRC_LEN 4
#define PD_MSG_LEN_OVER_TOTAL_LENGTH 3

struct cps8851_chip {
	struct i2c_client *client;
	struct device *dev;
#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_device *m_dev;
#endif /* CONFIG_RT_REGMAP */
	struct tcpc_desc *tcpc_desc;
	struct tcpc_device *tcpc;
#if CPS8851_VBUS_PRES_DEB_TIME
	struct delayed_work	power_change_work;
	bool check_real_vbus;
	bool falling;
#endif

	int irq_gpio;
	int irq;
	int chip_id;
	uint16_t chip_pid;
	uint16_t chip_vid;
};

#ifdef CONFIG_RT_REGMAP
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
RT_REG_DECL(TCPC_V10_REG_RX_BYTE_CNT, 4, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_RX_DATA, 28, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_TRANSMIT, 1, RT_VOLATILE, {});
RT_REG_DECL(TCPC_V10_REG_TX_BYTE_CNT, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_TX_HDR, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(TCPC_V10_REG_TX_DATA, 28, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_CONFIG_GPIO0, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_PHY_CTRL1, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_CLK_CTRL2, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_CLK_CTRL3, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_PRL_FSM_RESET, 1, RT_VOLATILE, {});
RT_REG_DECL(CPS8851H_REG_BMC_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(CPS8851H_REG_BMCIO_RXDZSEL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_VCONN_CLIMITEN, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_RT_STATUS, 1, RT_VOLATILE, {});
RT_REG_DECL(CPS8851H_REG_RT_INT, 1, RT_VOLATILE, {});
RT_REG_DECL(CPS8851H_REG_RT_MASK, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_IDLE_CTRL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_INTRST_CTRL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_WATCHDOG_CTRL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_I2CRST_CTRL, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_SWRESET, 1, RT_VOLATILE, {});
RT_REG_DECL(CPS8851H_REG_TTCPC_FILTER, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_DRP_TOGGLE_CYCLE, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_DRP_DUTY_CTRL, 2, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_BMCIO_RXDZEN, 1, RT_NORMAL_WR_ONCE, {});
RT_REG_DECL(CPS8851H_REG_UNLOCK_PW_2, 2, RT_VOLATILE, {});
RT_REG_DECL(CPS8851H_REG_EFUSE5, 1, RT_VOLATILE, {});

static const rt_register_map_t cps8851_chip_regmap[] = {
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
	RT_REG(TCPC_V10_REG_RX_DATA),
	RT_REG(TCPC_V10_REG_TRANSMIT),
	RT_REG(TCPC_V10_REG_TX_BYTE_CNT),
	RT_REG(TCPC_V10_REG_TX_HDR),
	RT_REG(TCPC_V10_REG_TX_DATA),
	RT_REG(CPS8851H_REG_CONFIG_GPIO0),
	RT_REG(CPS8851H_REG_PHY_CTRL1),
	RT_REG(CPS8851H_REG_CLK_CTRL2),
	RT_REG(CPS8851H_REG_CLK_CTRL3),
	RT_REG(CPS8851H_REG_PRL_FSM_RESET),
	RT_REG(CPS8851H_REG_BMC_CTRL),
	RT_REG(CPS8851H_REG_BMCIO_RXDZSEL),
	RT_REG(CPS8851H_REG_VCONN_CLIMITEN),
	RT_REG(CPS8851H_REG_RT_STATUS),
	RT_REG(CPS8851H_REG_RT_INT),
	RT_REG(CPS8851H_REG_RT_MASK),
	RT_REG(CPS8851H_REG_IDLE_CTRL),
	RT_REG(CPS8851H_REG_INTRST_CTRL),
	RT_REG(CPS8851H_REG_WATCHDOG_CTRL),
	RT_REG(CPS8851H_REG_I2CRST_CTRL),
	RT_REG(CPS8851H_REG_SWRESET),
	RT_REG(CPS8851H_REG_TTCPC_FILTER),
	RT_REG(CPS8851H_REG_DRP_TOGGLE_CYCLE),
	RT_REG(CPS8851H_REG_DRP_DUTY_CTRL),
	RT_REG(CPS8851H_REG_BMCIO_RXDZEN),
	RT_REG(CPS8851H_REG_UNLOCK_PW_2),
	RT_REG(CPS8851H_REG_EFUSE5),
};
#define CPS8851_CHIP_REGMAP_SIZE ARRAY_SIZE(cps8851_chip_regmap)

static inline bool chip_is_cps8851(struct cps8851_chip *chip)
{
	return chip->chip_pid == CPS8851_PID && chip->chip_vid == CPS8851_VID;
}
#endif /* CONFIG_RT_REGMAP */
#ifdef OPLUS_FEATURE_CHG_BASIC
static int cps8851_set_vconn(struct tcpc_device *tcpc, int enable);
#endif

static struct cps8851_chip *g_chip = NULL;
static int cps8851_read_device(void *client, u32 reg, int len, void *dst)
{
	struct i2c_client *i2c = client;
	int ret = 0;
	int count = 5;
	u64 __maybe_unused t1 = 0, t2 = 0;

	while (1) {
		t1 = local_clock();
		ret = i2c_smbus_read_i2c_block_data(i2c, reg, len, dst);
		t2 = local_clock();
		CPS8851_INFO("%s del = %lluus, reg = 0x%02X, len = %d\n",
			    __func__, (t2 - t1) / NSEC_PER_USEC, reg, len);
		if (ret < 0 && count > 1)
			count--;
		else
			break;
		udelay(100);
	}
	return ret;
}

static int cps8851_write_device(void *client, u32 reg, int len, const void *src)
{
	struct i2c_client *i2c = client;
	int ret = 0;
	int count = 5;
	u64 __maybe_unused t1 = 0, t2 = 0;

	while (1) {
		t1 = local_clock();
		ret = i2c_smbus_write_i2c_block_data(i2c, reg, len, src);
		t2 = local_clock();
		CPS8851_INFO("%s del = %lluus, reg = %02X, len = %d\n",
			    __func__, (t2 - t1) / NSEC_PER_USEC, reg, len);
		if (ret < 0 && count > 1)
			count--;
		else
			break;
		udelay(100);
	}
	return ret;
}

static int cps8851_reg_read(struct i2c_client *i2c, u8 reg)
{
	struct cps8851_chip *chip = i2c_get_clientdata(i2c);
	u8 val = 0;
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = cps8851_read_device(chip->client, reg, 1, &val);
#endif /* CONFIG_RT_REGMAP */
	if (ret < 0) {
		dev_err(chip->dev, "cps8851 reg read fail\n");
		return ret;
	}
	return val;
}

static int cps8851_reg_write(struct i2c_client *i2c, u8 reg, const u8 data)
{
	struct cps8851_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;
	int i = 0;

retry:
#ifdef CONFIG_RT_REGMAP
	ret = cps8851_write_device(chip->client, reg, 1, &data);
#endif /* CONFIG_RT_REGMAP */
	if (ret < 0) {
		if (i < 3) {
			i++;
			dev_err(chip->dev, "cps8851 reg write fail i=[%d] < 3 and retry\n", i);
			goto retry;
		}
		dev_err(chip->dev, "cps8851 reg write fail\n");
	}
	return ret;
}

static int cps8851_block_read(struct i2c_client *i2c,
			u8 reg, int len, void *dst)
{
	struct cps8851_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;
#ifdef CONFIG_RT_REGMAP
	ret = cps8851_read_device(chip->client, reg, len, dst);
#else
	ret = cps8851_read_device(chip->client, reg, len, dst);
#endif /* #ifdef CONFIG_RT_REGMAP */
	if (ret < 0)
		dev_err(chip->dev, "cps8851 block read fail\n");
	return ret;
}

static int cps8851_block_write(struct i2c_client *i2c,
			u8 reg, int len, const void *src)
{
	struct cps8851_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;
#ifdef CONFIG_RT_REGMAP
	ret = cps8851_write_device(chip->client, reg, len, src);
#else
	ret = cps8851_write_device(chip->client, reg, len, src);
#endif /* #ifdef CONFIG_RT_REGMAP */
	if (ret < 0)
		dev_err(chip->dev, "cps8851 block write fail\n");
	return ret;
}

static int32_t cps8851_write_word(struct i2c_client *client,
					uint8_t reg_addr, uint16_t data)
{
	int ret;

	/* don't need swap */
	ret = cps8851_block_write(client, reg_addr, 2, (uint8_t *)&data);
	return ret;
}

static int32_t cps8851_read_word(struct i2c_client *client,
					uint8_t reg_addr, uint16_t *data)
{
	int ret;

	/* don't need swap */
	ret = cps8851_block_read(client, reg_addr, 2, (uint8_t *)data);
	return ret;
}

static inline int cps8851_i2c_write8(
	struct tcpc_device *tcpc, u8 reg, const u8 data)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	return cps8851_reg_write(chip->client, reg, data);
}

static inline int cps8851_i2c_write16(
		struct tcpc_device *tcpc, u8 reg, const u16 data)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	return cps8851_write_word(chip->client, reg, data);
}

static inline int cps8851_i2c_read8(struct tcpc_device *tcpc, u8 reg)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	return cps8851_reg_read(chip->client, reg);
}

static inline int cps8851_i2c_read16(
	struct tcpc_device *tcpc, u8 reg)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
	u16 data;
	int ret;

	ret = cps8851_read_word(chip->client, reg, &data);
	if (ret < 0)
		return ret;
	return data;
}

#ifdef CONFIG_RT_REGMAP
static struct rt_regmap_fops cps8851_regmap_fops = {
	.read_device = cps8851_read_device,
	.write_device = cps8851_write_device,
};
#endif /* CONFIG_RT_REGMAP */

static int cps8851_regmap_init(struct cps8851_chip *chip)
{
#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_properties *props;
	char name[32];
	int len;

	props = devm_kzalloc(chip->dev, sizeof(*props), GFP_KERNEL);
	if (!props)
		return -ENOMEM;

	props->register_num = CPS8851_CHIP_REGMAP_SIZE;
	props->rm = cps8851_chip_regmap;

	props->rt_regmap_mode = RT_MULTI_BYTE |
				RT_IO_PASS_THROUGH | RT_DBG_SPECIAL;
	snprintf(name, sizeof(name), "cps8851-%02x", chip->client->addr);

	len = strlen(name);
	props->name = kzalloc(len+1, GFP_KERNEL);
	props->aliases = kzalloc(len+1, GFP_KERNEL);

	if ((!props->name) || (!props->aliases))
		return -ENOMEM;

	strlcpy((char *)props->name, name, len+1);
	strlcpy((char *)props->aliases, name, len+1);
	props->io_log_en = 0;

	chip->m_dev = rt_regmap_device_register(props,
			&cps8851_regmap_fops, chip->dev, chip->client, chip);
	if (!chip->m_dev) {
		dev_err(chip->dev, "cps8851 chip rt_regmap register fail\n");
		return -EINVAL;
	}
#endif
	return 0;
}

static int cps8851_regmap_deinit(struct cps8851_chip *chip)
{
#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(chip->m_dev);
#endif
	return 0;
}

static inline int cps8851_software_reset(struct tcpc_device *tcpc)
{
	int ret = cps8851_i2c_write8(tcpc, CPS8851H_REG_SWRESET, 1);
#ifdef CONFIG_RT_REGMAP
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0)
		return ret;
#ifdef CONFIG_RT_REGMAP
	rt_regmap_cache_reload(chip->m_dev);
#endif /* CONFIG_RT_REGMAP */
	usleep_range(1000, 2000);
	return 0;
}

static inline int cps8851_command(struct tcpc_device *tcpc, uint8_t cmd)
{
	return cps8851_i2c_write8(tcpc, TCPC_V10_REG_COMMAND, cmd);
}

static int cps8851_init_vbus_cal(struct tcpc_device *tcpc)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
	const u8 val_en_test_mode[] = {0x86, 0x62};
	const u8 val_dis_test_mode[] = {0x00, 0x00};
	int ret = 0;
	u8 data = 0;
	s8 cal = 0;

	ret = cps8851_block_write(chip->client, CPS8851H_REG_UNLOCK_PW_2,
			ARRAY_SIZE(val_en_test_mode), val_en_test_mode);
	if (ret < 0)
		dev_notice(chip->dev, "%s en test mode fail(%d)\n",
				__func__, ret);

	ret = cps8851_reg_read(chip->client, CPS8851H_REG_EFUSE5);
	if (ret < 0)
		goto out;

	data = ret;
	data = (data & CPS8851H_REG_M_VBUS_CAL) >> CPS8851H_REG_S_VBUS_CAL;
	cal = (data & BIT(2)) ? (data | GENMASK(7, 3)) : data;
	cal -= 2;
	if (cal < CPS8851H_REG_MIN_VBUS_CAL)
		cal = CPS8851H_REG_MIN_VBUS_CAL;
	data = (cal << CPS8851H_REG_S_VBUS_CAL) | (ret & GENMASK(4, 0));

	ret = cps8851_reg_write(chip->client, CPS8851H_REG_EFUSE5, data);
out:
	ret = cps8851_block_write(chip->client, CPS8851H_REG_UNLOCK_PW_2,
			ARRAY_SIZE(val_dis_test_mode), val_dis_test_mode);
	if (ret < 0)
		dev_notice(chip->dev, "%s dis test mode fail(%d)\n",
				__func__, ret);

	return ret;
}

static int cps8851_init_alert_mask(struct tcpc_device *tcpc)
{
	uint16_t mask;
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	mask = TCPC_V10_REG_ALERT_CC_STATUS | TCPC_V10_REG_ALERT_POWER_STATUS;

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
	/* Need to handle RX overflow */
	mask |= TCPC_V10_REG_ALERT_TX_SUCCESS | TCPC_V10_REG_ALERT_TX_DISCARDED
			| TCPC_V10_REG_ALERT_TX_FAILED
			| TCPC_V10_REG_ALERT_RX_HARD_RST
			| TCPC_V10_REG_ALERT_RX_STATUS
			| TCPC_V10_REG_RX_OVERFLOW;
#endif

	mask |= TCPC_REG_ALERT_FAULT;

	return cps8851_write_word(chip->client, TCPC_V10_REG_ALERT_MASK, mask);
}

static int cps8851_init_power_status_mask(struct tcpc_device *tcpc)
{
	const uint8_t mask = TCPC_V10_REG_POWER_STATUS_VBUS_PRES;

	return cps8851_i2c_write8(tcpc,
			TCPC_V10_REG_POWER_STATUS_MASK, mask);
}

static int cps8851_init_fault_mask(struct tcpc_device *tcpc)
{
	const uint8_t mask =
		TCPC_V10_REG_FAULT_STATUS_VCONN_OV |
		TCPC_V10_REG_FAULT_STATUS_VCONN_OC;

	return cps8851_i2c_write8(tcpc,
			TCPC_V10_REG_FAULT_STATUS_MASK, mask);
}

static int cps8851_init_rt_mask(struct tcpc_device *tcpc)
{
	uint8_t rt_mask = 0;
#ifdef CONFIG_TCPC_WATCHDOG_EN
	rt_mask |= CPS8851H_REG_M_WATCHDOG;
#endif /* CONFIG_TCPC_WATCHDOG_EN */
	rt_mask |= CPS8851H_REG_M_VBUS_80;

#ifdef CONFIG_TYPEC_CAP_RA_DETACH
	if (tcpc->tcpc_flags & TCPC_FLAGS_CHECK_RA_DETACH)
		rt_mask |= CPS8851H_REG_M_RA_DETACH;
#endif /* CONFIG_TYPEC_CAP_RA_DETACH */

#ifdef CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG
	if (tcpc->tcpc_flags & TCPC_FLAGS_LPM_WAKEUP_WATCHDOG)
		rt_mask |= CPS8851H_REG_M_WAKEUP;
#endif	/* CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG */

	return cps8851_i2c_write8(tcpc, CPS8851H_REG_RT_MASK, rt_mask);
}

static irqreturn_t cps8851_intr_handler(int irq, void *data)
{
	struct cps8851_chip *chip = data;

	pm_wakeup_event(chip->dev, CPS8851H_IRQ_WAKE_TIME);
 	tcpci_lock_typec(chip->tcpc);
	tcpci_alert(chip->tcpc);
 	tcpci_unlock_typec(chip->tcpc);

	return IRQ_HANDLED;
}

static int cps8851_init_alert(struct tcpc_device *tcpc)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
	int ret = 0;
	char *name = NULL;

	/* Clear Alert Mask & Status */
	cps8851_write_word(chip->client, TCPC_V10_REG_ALERT_MASK, 0);
	cps8851_write_word(chip->client, TCPC_V10_REG_ALERT, 0xffff);

	name = devm_kasprintf(chip->dev, GFP_KERNEL, "%s-IRQ",
			      chip->tcpc_desc->name);
	if (!name)
		return -ENOMEM;

	dev_info(chip->dev, "%s name = %s, gpio = %d\n",
			    __func__, chip->tcpc_desc->name, chip->irq_gpio);

	ret = devm_gpio_request(chip->dev, chip->irq_gpio, name);

	if (ret < 0) {
		dev_notice(chip->dev, "%s request GPIO fail(%d)\n",
				      __func__, ret);
		return ret;
	}

	ret = gpio_direction_input(chip->irq_gpio);
	if (ret < 0) {
		dev_notice(chip->dev, "%s set GPIO fail(%d)\n", __func__, ret);
		return ret;
	}

	ret = gpio_to_irq(chip->irq_gpio);
	if (ret < 0) {
		dev_notice(chip->dev, "%s gpio to irq fail(%d)",
				      __func__, ret);
		return ret;
 	}
	chip->irq = ret;

	dev_info(chip->dev, "%s IRQ number = %d\n", __func__, chip->irq);

	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
					cps8851_intr_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					name, chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s request irq fail(%d)\n",
				      __func__, ret);
		return ret;
 	}
	device_init_wakeup(chip->dev, true);

	return 0;
}

int cps8851_alert_status_clear(struct tcpc_device *tcpc, uint32_t mask)
{
	int ret;
	uint16_t mask_t1;

	uint8_t mask_t2;


	mask_t1 = mask;
	if (mask_t1) {
		ret = cps8851_i2c_write16(tcpc, TCPC_V10_REG_ALERT, mask_t1);
		if (ret < 0)
			return ret;
	}

	mask_t2 = mask >> 16;
	if (mask_t2) {
		ret = cps8851_i2c_write8(tcpc, CPS8851H_REG_RT_INT, mask_t2);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int cps8851h_set_clock_gating(struct tcpc_device *tcpc, bool en)
{
	int ret = 0;
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
#ifdef CONFIG_TCPC_CLOCK_GATING
	int i = 0;
	uint8_t clk2 = CPS8851H_REG_CLK_DIV_600K_EN
		| CPS8851H_REG_CLK_DIV_300K_EN | CPS8851H_REG_CLK_CK_300K_EN;
	uint8_t clk3 = CPS8851H_REG_CLK_DIV_2P4M_EN;

	if (!en) {
		clk2 |=
			CPS8851H_REG_CLK_BCLK2_EN | CPS8851H_REG_CLK_BCLK_EN;
		clk3 |=
			CPS8851H_REG_CLK_CK_24M_EN | CPS8851H_REG_CLK_PCLK_EN;
	}
	if (en) {
		for (i = 0; i < 2; i++)
			ret = cps8851_alert_status_clear(tcpc,
				TCPC_REG_ALERT_RX_ALL_MASK);
	}
	if (!chip_is_cps8851(chip)) {
		if (ret == 0)
			ret = cps8851_i2c_write8(tcpc, CPS8851H_REG_CLK_CTRL2, clk2);
		if (ret == 0)
			ret = cps8851_i2c_write8(tcpc, CPS8851H_REG_CLK_CTRL3, clk3);
	}
#endif	/* CONFIG_TCPC_CLOCK_GATING */

	return ret;
}

static inline int cps8851h_init_cc_params(
			struct tcpc_device *tcpc, uint8_t cc_res)
{
	int rv = 0;

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
#ifdef CONFIG_USB_PD_SNK_DFT_NO_GOOD_CRC
	uint8_t en, sel;
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	if (cc_res == TYPEC_CC_VOLT_SNK_DFT) {	/* 0.55 */
		en = 0;
		sel = 0x81;
	} else if (chip->chip_id >= RT1715_DID_D) {	/* 0.35 & 0.75 */
		en = 1;
		sel = 0x81;
	} else {	/* 0.4 & 0.7 */
		en = 1;
		sel = 0x80;
	}

	rv = cps8851_i2c_write8(tcpc, CPS8851H_REG_BMCIO_RXDZEN, en);
	if (rv == 0)
		rv = cps8851_i2c_write8(tcpc, CPS8851H_REG_BMCIO_RXDZSEL, sel);
#endif	/* CONFIG_USB_PD_SNK_DFT_NO_GOOD_CRC */
#endif	/* CONFIG_USB_POWER_DELIVERY */

	return rv;
}

static int cps8851_trim(struct tcpc_device *tcpc)
{
	int ret;
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	if (!chip_is_cps8851(chip))
		return 0;
	ret = cps8851_i2c_write8(tcpc, CPS8851_REG_PASSWORD, 0x51);
	/* Modify TRIM values. */
	ret += cps8851_i2c_write8(tcpc, CPS8851_REG_PD_OPT_ZTX_SEL, 0x09);
	ret += cps8851_i2c_write8(tcpc, CPS8851_REG_PD_OPT_TX_SLEW_SEL, 0x01);
	ret += cps8851_i2c_write8(tcpc, CPS8851_REG_PD_OPT_DB_IBIAS_EN, 0x01);
	ret += cps8851_i2c_write8(tcpc, CPS8851_REG_PASSWORD, 0x00);

	if (ret < 0) {
		dev_err(&tcpc->dev, "CPS8851 update trim value fail\n");
		return -EIO;
	}

	return ret;
}

static int cps8851_tcpc_init(struct tcpc_device *tcpc, bool sw_reset)
{
	int ret;
	bool retry_discard_old = false;
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	CPS8851_INFO("\n");

	if (sw_reset) {
		ret = cps8851_software_reset(tcpc);
		if (ret < 0)
			return ret;
	}
#ifdef CONFIG_TCPC_I2CRST_EN
	if (chip->chip_id == HUSB311_DID) {
		cps8851_i2c_write8(tcpc,
			CPS8851H_REG_I2CRST_CTRL, 0x08);
	} else {
		cps8851_i2c_write8(tcpc,
			CPS8851H_REG_I2CRST_CTRL,
			CPS8851H_REG_I2CRST_SET(true, 0x0f));
	}
#endif	/* CONFIG_TCPC_I2CRST_EN */

	/* UFP Both RD setting */
	/* DRP = 0, RpVal = 0 (Default), Rd, Rd */
	cps8851_i2c_write8(tcpc, TCPC_V10_REG_ROLE_CTRL,
		TCPC_V10_REG_ROLE_CTRL_RES_SET(0, 0, CC_RD, CC_RD));

	/*
	 * CC Detect Debounce : 26.7*val us
	 * Transition window count : spec 12~20us, based on 2.4MHz
	 * DRP Toggle Cycle : 51.2 + 6.4*val ms
	 * DRP Duty Ctrl : dcSRC / 1024
	 */

	cps8851_i2c_write8(tcpc, CPS8851H_REG_TTCPC_FILTER, 10);
	cps8851_i2c_write8(tcpc, CPS8851H_REG_DRP_TOGGLE_CYCLE, 4);
	cps8851_i2c_write16(tcpc,
		CPS8851H_REG_DRP_DUTY_CTRL, TCPC_NORMAL_RP_DUTY);
#ifdef OPLUS_FEATURE_CHG_BASIC
	/**
	*Vconn OC Mode
	*0:close Vconn output/default value
	*1:current limit/There is a risk of burning
	*cps8851_i2c_write8(tcpc, CPS8851H_REG_VCONN_CLIMITEN, 1);
	*/
#endif
	/* RX/TX Clock Gating (Auto Mode)*/
	if (!sw_reset)
		cps8851h_set_clock_gating(tcpc, true);

	if (!(tcpc->tcpc_flags & TCPC_FLAGS_RETRY_CRC_DISCARD))
		retry_discard_old = true;

	cps8851_i2c_write8(tcpc, CPS8851H_REG_CONFIG_GPIO0, 0x80);

	if (!chip_is_cps8851(chip)) {
		cps8851_i2c_write8(tcpc, CPS8851H_REG_PHY_CTRL1,
			CPS8851H_REG_PHY_CTRL1_SET(retry_discard_old, 7, 0, 1));
	}
	/* For BIST, Change Transition Toggle Counter (Noise) from 3 to 7 */
/*	cps8851_i2c_write8(tcpc, CPS8851H_REG_PHY_CTRL1,
		CPS8851H_REG_PHY_CTRL1_SET(retry_discard_old, 7, 0, 1));
*/
	tcpci_alert_status_clear(tcpc, 0xffffffff);

	cps8851_init_vbus_cal(tcpc);
	cps8851_init_power_status_mask(tcpc);
	cps8851_init_alert_mask(tcpc);
	cps8851_init_fault_mask(tcpc);
	cps8851_init_rt_mask(tcpc);

	/* CK_300K from 320K, SHIPPING off, AUTOIDLE enable, TIMEOUT = 6.4ms */
	cps8851_i2c_write8(tcpc, CPS8851H_REG_IDLE_CTRL,
		CPS8851H_REG_IDLE_SET(0, 1, 1, 0));
	mdelay(1);

	cps8851_trim(tcpc);

	return 0;
}

static int cps8851_get_chip_id(struct tcpc_device *tcpc, uint32_t *chip_id)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
	*chip_id = chip->chip_id;
	return 0;
}

static int cps8851_get_chip_pid(struct tcpc_device *tcpc, uint32_t *chip_pid)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
	*chip_pid = chip->chip_pid;
	return 0;
}

static int cps8851_get_chip_vid(struct tcpc_device *tcpc, uint32_t *chip_vid)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
	*chip_vid = chip->chip_vid;
	return 0;
}

static inline int cps8851_fault_status_vconn_ov(struct tcpc_device *tcpc)
{
	int ret;
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	if (chip_is_cps8851(chip))
		return 0;
	ret = cps8851_i2c_read8(tcpc, CPS8851H_REG_BMC_CTRL);
	if (ret < 0)
		return ret;

	ret &= ~CPS8851H_REG_DISCHARGE_EN;
	return cps8851_i2c_write8(tcpc, CPS8851H_REG_BMC_CTRL, ret);
}

int cps8851_fault_status_clear(struct tcpc_device *tcpc, uint8_t status)
{
	int ret = 0;

	if (status & TCPC_V10_REG_FAULT_STATUS_VCONN_OV)
		ret = cps8851_fault_status_vconn_ov(tcpc);
	pr_info("%s, ret:%d\n", __func__, ret);

	return cps8851_i2c_write8(tcpc, TCPC_V10_REG_FAULT_STATUS, status);
}

int cps8851_get_alert_mask(struct tcpc_device *tcpc, uint32_t *mask)
{
	int ret;
	uint8_t v2;

	ret = cps8851_i2c_read16(tcpc, TCPC_V10_REG_ALERT_MASK);
	if (ret < 0)
		return ret;

	*mask = (uint16_t) ret;

	ret = cps8851_i2c_read8(tcpc, CPS8851H_REG_RT_MASK);
	if (ret < 0)
		return ret;

	v2 = (uint8_t) ret;
	*mask |= v2 << 16;

	return 0;
}

int cps8851_get_alert_status(struct tcpc_device *tcpc, uint32_t *alert)
{
	int ret;
	uint8_t v2;
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	ret = cps8851_i2c_read16(tcpc, TCPC_V10_REG_ALERT);
	if (ret < 0)
		return ret;

	*alert = (uint16_t) ret;

	if ((chip->chip_pid == HUSB311_PID) && (*alert & TCPC_V10_REG_ALERT_TX_DISCARDED)) {
		pr_info("%s\n", __func__);
		tcpci_init(tcpc, true);
	}
	ret = cps8851_i2c_read8(tcpc, CPS8851H_REG_RT_INT);
	if (ret < 0)
		return ret;

	v2 = (uint8_t) ret;
	*alert |= v2 << 16;

	return 0;
}

static int cps8851_get_power_status(
		struct tcpc_device *tcpc, uint16_t *pwr_status)
{
	int ret;

#if CPS8851_VBUS_PRES_DEB_TIME
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
	bool falling = false;
#endif

	ret = cps8851_i2c_read8(tcpc, TCPC_V10_REG_POWER_STATUS);
	if (ret < 0)
		return ret;

	*pwr_status = 0;

	if (ret & TCPC_V10_REG_POWER_STATUS_VBUS_PRES)
		*pwr_status |= TCPC_REG_POWER_STATUS_VBUS_PRES;

#if CPS8851_VBUS_PRES_DEB_TIME
	if (tcpc->vbus_level == TCPC_VBUS_VALID &&
			!(ret & TCPC_V10_REG_POWER_STATUS_VBUS_PRES))
		falling = true;
#endif

/*#ifdef CONFIG_TCPC_VSAFE0V_DETECT_IC */
	ret = cps8851_i2c_read8(tcpc, CPS8851_REG_CP_STATUS);
	if (ret < 0)
		goto out;

	if (ret & CPS8851_REG_VBUS_80) {
		*pwr_status |= TCPC_REG_POWER_STATUS_EXT_VSAFE0V;
#if CPS8851_VBUS_PRES_DEB_TIME
		falling = false;
#endif
	}

/*#endif*/
	ret = 0;

out:
	if (chip->check_real_vbus) {
		chip->falling = false;
		return ret;
	}

	if (falling && !chip->falling) {
		dev_info(chip->dev, "sched deferred power_change\n");
		cancel_delayed_work(&chip->power_change_work);
		schedule_delayed_work(&chip->power_change_work,
		msecs_to_jiffies(CPS8851_VBUS_PRES_DEB_TIME));
	} else if ((*pwr_status & (TCPC_REG_POWER_STATUS_VBUS_PRES |
		TCPC_REG_POWER_STATUS_EXT_VSAFE0V)) && chip->falling) {
		dev_info(chip->dev, "cancel deferred power_change\n");
		cancel_delayed_work(&chip->power_change_work);
	}

	if (falling) {
		dev_info(chip->dev, "fake vbus_pres\n");
		*pwr_status |= TCPC_REG_POWER_STATUS_VBUS_PRES;
	}

	chip->falling = falling;
	return ret;
}

int cps8851_get_fault_status(struct tcpc_device *tcpc, uint8_t *status)
{
	int ret;

	ret = cps8851_i2c_read8(tcpc, TCPC_V10_REG_FAULT_STATUS);
	if (ret < 0)
		return ret;
	*status = (uint8_t) ret;
	return 0;
}

static int cps8851_get_cc(struct tcpc_device *tcpc, int *cc1, int *cc2)
{
	int status;
	int role_ctrl;
	int cc_role;
	bool act_as_sink;
	bool act_as_drp;

	status = cps8851_i2c_read8(tcpc, TCPC_V10_REG_CC_STATUS);
	if (status < 0)
		return status;

	role_ctrl = cps8851_i2c_read8(tcpc, TCPC_V10_REG_ROLE_CTRL);
	if (role_ctrl < 0)
		return role_ctrl;

	if (status & TCPC_V10_REG_CC_STATUS_DRP_TOGGLING) {
		*cc1 = TYPEC_CC_DRP_TOGGLING;
		*cc2 = TYPEC_CC_DRP_TOGGLING;
		return 0;
	}

	*cc1 = TCPC_V10_REG_CC_STATUS_CC1(status);
	*cc2 = TCPC_V10_REG_CC_STATUS_CC2(status);

	act_as_drp = TCPC_V10_REG_ROLE_CTRL_DRP & role_ctrl;

	if (act_as_drp) {
		act_as_sink = TCPC_V10_REG_CC_STATUS_DRP_RESULT(status);
	} else {
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

	cps8851h_init_cc_params(tcpc,
		(uint8_t)tcpc->typec_polarity ? *cc2 : *cc1);

	return 0;
}

static int cps8851_enable_vsafe0v_detect(
	struct tcpc_device *tcpc, bool enable)
{
	int ret = cps8851_i2c_read8(tcpc, CPS8851H_REG_RT_MASK);

	if (ret < 0)
		return ret;

	if (enable)
		ret |= CPS8851H_REG_M_VBUS_80;
	else
		ret &= ~CPS8851H_REG_M_VBUS_80;

	return cps8851_i2c_write8(tcpc, CPS8851H_REG_RT_MASK, (uint8_t) ret);
}

static inline int swap_rp_rd(int cc)
{
	if (cc == CC_RD)
		cc = CC_RP;
	else if (cc == CC_RP)
		cc = CC_RD;

	return cc;
}

static int cps8851_set_cc(struct tcpc_device *tcpc, int pull)
{
	int ret = 0;
	uint8_t data = 0;
	uint8_t old_data = 0;
	int role_ctrl = 0;
	int cc1;
	int cc2;
	int rp_lvl = TYPEC_CC_PULL_GET_RP_LVL(pull), pull1, pull2;
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	CPS8851_INFO("pull = 0x%02X\n", pull);
	pull = TYPEC_CC_PULL_GET_RES(pull);

	if (pull == TYPEC_CC_DRP) {
		data = TCPC_V10_REG_ROLE_CTRL_RES_SET(
					1, rp_lvl, TYPEC_CC_RD, TYPEC_CC_RD);
		if (old_data != data || chip->chip_vid == CPS8851_VID || chip->chip_vid == HUSB311_VID) {
			ret = cps8851_i2c_write8(tcpc, TCPC_V10_REG_ROLE_CTRL, data);
		}
		if (ret == 0) {
			cps8851_enable_vsafe0v_detect(tcpc, false);
			ret = cps8851_command(tcpc, TCPM_CMD_LOOK_CONNECTION);
		}
	} else {
#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
		if (pull == TYPEC_CC_RD && tcpc->pd_wait_pr_swap_complete)
			cps8851h_init_cc_params(tcpc, TYPEC_CC_VOLT_SNK_DFT);
#endif	/* CONFIG_USB_POWER_DELIVERY */

		pull1 = pull2 = pull;

		if (pull == TYPEC_CC_RP && tcpc->typec_is_attached_src) {
			if (tcpc->typec_polarity)
				pull1 = TYPEC_CC_OPEN;
			else
				pull2 = TYPEC_CC_OPEN;
		}

		data = TCPC_V10_REG_ROLE_CTRL_RES_SET(0, rp_lvl, pull1, pull2);

		if (chip_is_cps8851(chip)) {
			role_ctrl = cps8851_i2c_read8(tcpc, TCPC_V10_REG_ROLE_CTRL);

			if (role_ctrl >= 0 && !(role_ctrl & TCPC_V10_REG_ROLE_CTRL_DRP)) {
				cc1 = role_ctrl & 0x03;
				cc2 = (role_ctrl >> 2) & 0x03;
				if ((cc1 != CC_OPEN || cc2 != CC_OPEN) &&
					(pull1 == TYPEC_CC_OPEN && pull2 == TYPEC_CC_OPEN)) {
					cc1 = swap_rp_rd(cc1);
					cc2 = swap_rp_rd(cc2);
					role_ctrl &= ~0xf;
					role_ctrl |= TCPC_V10_REG_ROLE_CTRL_RES_SET(0, 0, cc1, cc2);
					cps8851_i2c_write8(tcpc, TCPC_V10_REG_ROLE_CTRL, role_ctrl);
					mdelay(1);
				}
			}
		}

		if (old_data != data || chip->chip_vid == CPS8851_VID || chip->chip_vid == HUSB311_VID) {
			ret = cps8851_i2c_write8(tcpc, TCPC_V10_REG_ROLE_CTRL, data);
		}
	}

	return 0;
}

static int cps8851_set_polarity(struct tcpc_device *tcpc, int polarity)
{
	int data;

	data = cps8851h_init_cc_params(tcpc,
		tcpc->typec_remote_cc[polarity]);
	if (data)
		return data;

	data = cps8851_i2c_read8(tcpc, TCPC_V10_REG_TCPC_CTRL);
	if (data < 0)
		return data;

	data &= ~TCPC_V10_REG_TCPC_CTRL_PLUG_ORIENT;
	data |= polarity ? TCPC_V10_REG_TCPC_CTRL_PLUG_ORIENT : 0;

	return cps8851_i2c_write8(tcpc, TCPC_V10_REG_TCPC_CTRL, data);
}

static int cps8851_set_low_rp_duty(struct tcpc_device *tcpc, bool low_rp)
{
	uint16_t duty = low_rp ? TCPC_LOW_RP_DUTY : TCPC_NORMAL_RP_DUTY;

	return cps8851_i2c_write16(tcpc, CPS8851H_REG_DRP_DUTY_CTRL, duty);
}

static int cps8851_set_vconn(struct tcpc_device *tcpc, int enable)
{
	int rv;
	int data;

	data = cps8851_i2c_read8(tcpc, TCPC_V10_REG_POWER_CTRL);
	if (data < 0)
		return data;

#ifdef OPLUS_FEATURE_CHG_BASIC
	pr_info("%s, 0x%x=0x%x, enable=%d\n", __func__, TCPC_V10_REG_POWER_CTRL, data, enable);
#endif
	data &= ~TCPC_V10_REG_POWER_CTRL_VCONN;
	data |= enable ? TCPC_V10_REG_POWER_CTRL_VCONN : 0;

	data &= ~TCPC_V10_REG_POWER_CTRL_VCONN;
	pr_info("%s - write 0x%x to CPS8851_POWER_CTRL_VCONNL, off vconn\n",
				__func__, data);

	rv = cps8851_i2c_write8(tcpc, TCPC_V10_REG_POWER_CTRL, data);
	if (rv < 0)
		return rv;

	return cps8851_i2c_write8(tcpc, CPS8851H_REG_IDLE_CTRL,
		CPS8851H_REG_IDLE_SET(0, 1, enable ? 0 : 1, 0));
}

#ifdef CONFIG_TCPC_LOW_POWER_MODE
static int cps8851_is_low_power_mode(struct tcpc_device *tcpc)
{
	int rv;

	rv = cps8851_i2c_read8(tcpc, CPS8851H_REG_BMC_CTRL);
	if (rv < 0)
		return rv;

	pr_info("%s - read CPS8851_REG_BMC_CTRL=0x%x\n", __func__, rv);
	return (rv & CPS8851H_REG_BMCIO_OSC_EN) == 0;
}

static int cps8851_set_low_power_mode(
		struct tcpc_device *tcpc, bool en, int pull)
{
	int ret = 0;
	uint8_t data;
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	ret = cps8851_i2c_write8(tcpc, CPS8851H_REG_IDLE_CTRL,
		CPS8851H_REG_IDLE_SET(0, 1, en ? 0 : 1, 0));
	if (ret < 0)
		return ret;
	cps8851_enable_vsafe0v_detect(tcpc, !en);
	if (en) {
		data = CPS8851H_REG_BMCIO_LPEN;

		if (pull & TYPEC_CC_RP)
			data |= CPS8851H_REG_BMCIO_LPRPRD;

#ifdef CONFIG_TYPEC_CAP_NORP_SRC
		data |= CPS8851H_REG_BMCIO_BG_EN | CPS8851H_REG_VBUS_DET_EN;
#endif
		pr_info("%s - write CPS8851_REG_BMC_CTRL=0x%x\n",
				__func__, data);
	} else {
		data = CPS8851H_REG_BMCIO_BG_EN |
			CPS8851H_REG_VBUS_DET_EN | CPS8851H_REG_BMCIO_OSC_EN;
		if (chip->chip_pid == HUSB311_PID) {
			data |= CPS8851H_REG_BMCIO_OSC_EN;
			pr_info("%s - write CPS8851_REG_BMC_CTRL=0x%x\n",
				__func__, data);
		}
	}

	return cps8851_i2c_write8(tcpc, CPS8851H_REG_BMC_CTRL, data);
}
#endif	/* CONFIG_TCPC_LOW_POWER_MODE */

#ifdef CONFIG_TCPC_WATCHDOG_EN
int cps8851h_set_watchdog(struct tcpc_device *tcpc, bool en)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
/********* workaround MO.230913213000256759: sc6607 workaround for pd abnormal start*********/
	int data = 0;
	data = CPS8851H_REG_WATCHDOG_CTRL_SET(en, 7);
	return cps8851_i2c_write8(tcpc,
		CPS8851H_REG_WATCHDOG_CTRL, data);
/********* workaround MO.230913213000256759: sc6607 workaround for pd abnormal end *********/
#else
	uint8_t data = CPS8851H_REG_WATCHDOG_CTRL_SET(en, 7);

	return cps8851_i2c_write8(tcpc,
		CPS8851H_REG_WATCHDOG_CTRL, data);
#endif
}
#endif	/* CONFIG_TCPC_WATCHDOG_EN */

#ifdef CONFIG_TCPC_INTRST_EN
int cps8851h_set_intrst(struct tcpc_device *tcpc, bool en)
{
	return cps8851_i2c_write8(tcpc,
		CPS8851H_REG_INTRST_CTRL, CPS8851H_REG_INTRST_SET(en, 3));
}
#endif	/* CONFIG_TCPC_INTRST_EN */

static int cps8851_tcpc_deinit(struct tcpc_device *tcpc)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

#ifdef CONFIG_TCPC_SHUTDOWN_CC_DETACH
	cps8851_set_cc(tcpc, TYPEC_CC_DRP);
	cps8851_set_cc(tcpc, TYPEC_CC_OPEN);

	tcpci_alert_status_clear(tcpc, 0xffffff);/*0x10 0x11 0x98*/

	cps8851_write_word(chip->client, TCPC_V10_REG_ALERT_MASK, 0x0); /*0x12 0x13*/
	cps8851_i2c_write8(tcpc, TCPC_V10_REG_POWER_STATUS_MASK, 0x0); /*0x14*/
	cps8851_i2c_write8(tcpc, CPS8851H_REG_RT_MASK, 0x0); /*0x99*/
	cps8851_i2c_write8(tcpc, CPS8851H_REG_BMC_CTRL, 0x0); /*0x90*/

#ifndef OPLUS_FEATURE_CHG_BASIC
	cps8851_i2c_write8(tcpc, CPS8851H_REG_INTRST_CTRL, CPS8851H_REG_INTRST_SET(true, 0));
#endif
	if (chip->chip_pid == CPS8851_PID) {
		mdelay(150);
		cps8851_i2c_write8(tcpc, CPS8851H_REG_SWRESET, 1);
	}
#else
	cps8851_i2c_write8(tcpc, CPS8851H_REG_SWRESET, 1);
#endif	/* CONFIG_TCPC_SHUTDOWN_CC_DETACH */
#ifdef CONFIG_RT_REGMAP
	rt_regmap_cache_reload(chip->m_dev);
#endif /* CONFIG_RT_REGMAP */

	return 0;
}

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
static int cps8851_set_msg_header(
	struct tcpc_device *tcpc, uint8_t power_role, uint8_t data_role)
{
	uint8_t msg_hdr = TCPC_V10_REG_MSG_HDR_INFO_SET(
		data_role, power_role);

		return cps8851_i2c_write8(tcpc, TCPC_V10_REG_MSG_HDR_INFO, msg_hdr);
}

static int cps8851_protocol_reset(struct tcpc_device *tcpc)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
	if (!chip_is_cps8851(chip)) {
		cps8851_i2c_write8(tcpc, CPS8851H_REG_PRL_FSM_RESET, 0);
		mdelay(1);
		cps8851_i2c_write8(tcpc, CPS8851H_REG_PRL_FSM_RESET, 1);
	}
	return 0;
}

static int cps8851_set_rx_enable(struct tcpc_device *tcpc, uint8_t enable)
{
	int ret = 0;

	if (enable)
		ret = cps8851h_set_clock_gating(tcpc, false);

	if (ret == 0)
		ret = cps8851_i2c_write8(tcpc, TCPC_V10_REG_RX_DETECT, enable);

	if ((ret == 0) && (!enable)) {
		cps8851_protocol_reset(tcpc);
		ret = cps8851h_set_clock_gating(tcpc, true);
	}
	return ret;
}

static int cps8851_get_message(struct tcpc_device *tcpc, uint32_t *payload,
			uint16_t *msg_head, enum tcpm_transmit_type *frame_type)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
	int rv = 0;
	uint8_t cnt = 0, buf[4];

	rv = cps8851_block_read(chip->client, TCPC_V10_REG_RX_BYTE_CNT, 4, buf);
	if (rv < 0) {
		pr_info(" rv < 0 , rv = %d\n", rv);
		return rv;
	}

	cnt = buf[0];
	*frame_type = buf[1];
	*msg_head = le16_to_cpu(*(uint16_t *)&buf[2]);

	if (*msg_head == 0x0){
		/*tcpci_init(tcpc, true); */
		/* return -1;*/
	}


	/* TCPC 1.0 ==> no need to subtract the size of msg_head */
	if (cnt > 3) {
		cnt -= 3; /* MSG_HDR */
		rv = cps8851_block_read(chip->client, TCPC_V10_REG_RX_DATA, cnt,
				       payload);
	}

	return rv;
}

static int cps8851_set_bist_carrier_mode(
	struct tcpc_device *tcpc, uint8_t pattern)
{
	/* Don't support this function */
	return 0;
}

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
static int cps8851_retransmit(struct tcpc_device *tcpc)
{
	return cps8851_i2c_write8(tcpc, TCPC_V10_REG_TRANSMIT,
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

static int cps8851_init_mask(struct tcpc_device *tcpc)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);

	if (!chip_is_cps8851(chip))
		return 0;
	cps8851_init_alert_mask(tcpc);
	cps8851_init_power_status_mask(tcpc);
	cps8851_init_fault_mask(tcpc);
	cps8851_init_rt_mask(tcpc);

	return 0;
}

static int cps8851_transmit(struct tcpc_device *tcpc,
	enum tcpm_transmit_type type, uint16_t header, const uint32_t *data)
{
	struct cps8851_chip *chip = tcpc_get_dev_data(tcpc);
	int rv;
	int data_cnt;
	struct tcpc_transmit_packet packet;

	if (type < TCPC_TX_HARD_RESET) {
		data_cnt = sizeof(uint32_t) * PD_HEADER_CNT(header);

		packet.cnt = data_cnt + sizeof(uint16_t);
		packet.msg_header = header;

		if (data_cnt > 0)
			memcpy(packet.data, (uint8_t *) data, data_cnt);
		rv = cps8851_block_write(chip->client,
					TCPC_V10_REG_TX_BYTE_CNT,
					packet.cnt + 1, (uint8_t *) &packet);

		if (rv < 0)
			return rv;
	}
	rv = cps8851_i2c_write8(tcpc, TCPC_V10_REG_TRANSMIT,
			TCPC_V10_REG_TRANSMIT_SET(
			tcpc->pd_retry_count, type));

	if (chip_is_cps8851(chip))
		if (type == TCPC_TX_HARD_RESET)
			cps8851_init_mask(tcpc);
	return rv;
}

static int cps8851_set_bist_test_mode(struct tcpc_device *tcpc, bool en)
{
	int data;

	data = cps8851_i2c_read8(tcpc, TCPC_V10_REG_TCPC_CTRL);
	if (data < 0)
		return data;

	data &= ~TCPC_V10_REG_TCPC_CTRL_BIST_TEST_MODE;
	data |= en ? TCPC_V10_REG_TCPC_CTRL_BIST_TEST_MODE : 0;
	return cps8851_i2c_write8(tcpc, TCPC_V10_REG_TCPC_CTRL, data);
}
#endif /* CONFIG_USB_POWER_DELIVERY */

static struct tcpc_ops cps8851_tcpc_ops = {
	.init = cps8851_tcpc_init,
	.init_alert_mask = cps8851_init_mask,
	.alert_status_clear = cps8851_alert_status_clear,
	.fault_status_clear = cps8851_fault_status_clear,
	.get_alert_mask = cps8851_get_alert_mask,
	.get_alert_status = cps8851_get_alert_status,
	.get_power_status = cps8851_get_power_status,
	.get_fault_status = cps8851_get_fault_status,
	.get_chip_id = cps8851_get_chip_id,
	.get_chip_vid = cps8851_get_chip_vid,
	.get_chip_pid = cps8851_get_chip_pid,
	.get_cc = cps8851_get_cc,
	.set_cc = cps8851_set_cc,
	.set_polarity = cps8851_set_polarity,
	.set_low_rp_duty = cps8851_set_low_rp_duty,
	.set_vconn = cps8851_set_vconn,
	.deinit = cps8851_tcpc_deinit,

#ifdef CONFIG_TCPC_LOW_POWER_MODE
	.is_low_power_mode = cps8851_is_low_power_mode,
	.set_low_power_mode = cps8851_set_low_power_mode,
#endif	/* CONFIG_TCPC_LOW_POWER_MODE */

#ifdef CONFIG_TCPC_WATCHDOG_EN
	.set_watchdog = cps8851h_set_watchdog,
#endif	/* CONFIG_TCPC_WATCHDOG_EN */

#ifdef CONFIG_TCPC_INTRST_EN
	.set_intrst = cps8851h_set_intrst,
#endif	/* CONFIG_TCPC_INTRST_EN */

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
	.set_msg_header = cps8851_set_msg_header,
	.set_rx_enable = cps8851_set_rx_enable,
	.protocol_reset = cps8851_protocol_reset,
	.get_message = cps8851_get_message,
	.transmit = cps8851_transmit,
	.set_bist_test_mode = cps8851_set_bist_test_mode,
	.set_bist_carrier_mode = cps8851_set_bist_carrier_mode,
#endif	/* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
	.retransmit = cps8851_retransmit,
#endif	/* CONFIG_USB_PD_RETRY_CRC_DISCARD */
};

static int rt_parse_dt(struct cps8851_chip *chip, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	pr_info("%s\n", __func__);

#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(np, "cps8851pd,intr_gpio", 0);
	if (ret < 0) {
		pr_err("%s no intr_gpio info\n", __func__);
		return ret;
	}
	chip->irq_gpio = ret;
#else
	ret = of_property_read_u32(np,
		"cps8851pd,intr_gpio_num", &chip->irq_gpio);
	if (ret < 0)
		pr_err("%s no intr_gpio info\n", __func__);
#endif
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

static int cps8851_tcpcdev_init(struct cps8851_chip *chip, struct device *dev)
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

	if (of_property_read_string(np, "rt-tcpc,name",
				(char const **)&name) < 0) {
		dev_info(dev, "use default name\n");
	}

	len = strlen(name);
	desc->name = kzalloc(len+1, GFP_KERNEL);
	if (!desc->name)
		return -ENOMEM;

	strlcpy((char *)desc->name, name, len+1);

	chip->tcpc_desc = desc;

	chip->tcpc = tcpc_device_register(dev,
			desc, &cps8851_tcpc_ops, chip);
	if (IS_ERR_OR_NULL(chip->tcpc))
		return -EINVAL;

#ifdef CONFIG_USB_PD_DISABLE_PE
	chip->tcpc->disable_pe =
			of_property_read_bool(np, "rt-tcpc,disable_pe");
#endif	/* CONFIG_USB_PD_DISABLE_PE */


	chip->tcpc->tcpc_flags = TCPC_FLAGS_LPM_WAKEUP_WATCHDOG |
			TCPC_FLAGS_VCONN_SAFE5V_ONLY;

	chip->tcpc->tcpc_flags |= TCPC_FLAGS_CHECK_RA_DETACH;

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
	if (chip->chip_pid == CPS8851_PID)
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
#ifdef OPLUS_FEATURE_CHG_BASIC
	/* add for GTS.UsbRoleSwapTest */
	chip->tcpc->support_role_swap_delay = of_property_read_bool(np, "support_role_swap_delay");
	dev_info(dev, "support_role_swap_delay:%d\n", chip->tcpc->support_role_swap_delay);
#endif

	return 0;
}

static inline int cps8851h_check_revision(struct i2c_client *client)
{
	u16 vid, pid, did;
	int ret;
	u8 data = 1;

	ret = cps8851_read_device(client, TCPC_V10_REG_VID, 2, &vid);
	if (ret < 0) {
		dev_err(&client->dev, "read chip ID fail(%d)\n", ret);
		return -EIO;
	}

	if ((vid != CPS8851_VID) && (vid != HUSB311_VID)) {
		pr_info("%s failed, VID=0x%04x\n", __func__, vid);
		return -ENODEV;
	}

	ret = cps8851_read_device(client, TCPC_V10_REG_PID, 2, &pid);
	if (ret < 0) {
		dev_err(&client->dev, "read product ID fail(%d)\n", ret);
		return -EIO;
	}

	if ((pid != CPS8851_PID) && (pid != HUSB311_PID)) {
		pr_info("%s failed, PID=0x%04x\n", __func__, pid);
		return -ENODEV;
	}

	ret = cps8851_write_device(client, CPS8851H_REG_SWRESET, 1, &data);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	ret = cps8851_read_device(client, TCPC_V10_REG_DID, 2, &did);
	if (ret < 0) {
		dev_err(&client->dev, "read device ID fail(%d)\n", ret);
		return -EIO;
	}

	pr_err(" (%s) vid = 0x%x pid = 0x%x did = 0x%x\n", __func__, vid, pid, did);
	return did;
}

#ifndef OPLUS_FEATURE_CHG_BASIC
static int sc2150a_init_cc(struct cps8851_chip *chip)
{
	int ret;
	u8 ctrl = CPS8851H_REG_IDLE_SET(0, 1, 1, 0);
	u8 data = TCPC_V10_REG_ROLE_CTRL_RES_SET(0, 0, CC_OPEN, CC_OPEN);
	u8 int_clr = 0xff;

	ret = cps8851_write_device(chip->client, CPS8851H_REG_IDLE_CTRL, 1, &ctrl);
	if (ret < 0) {
		dev_err(chip->dev, "exit shutdown mode fail\n");
		goto err;
	}

	ret = cps8851_write_device(chip->client, TCPC_V10_REG_ROLE_CTRL, 1, &data);
	if (ret < 0) {
		dev_err(chip->dev, "set cc open fail\n");
		goto err;
	}

	ret = cps8851_write_device(chip->client, TCPC_V10_REG_FAULT_STATUS, 1, &int_clr);
	if (ret < 0) {
		dev_err(chip->dev, "clear fault int fail\n");
		goto err;
	}

	msleep(100);

	ctrl = CPS8851H_REG_IDLE_SET(0, 0, 1, 0);
	ret = cps8851_write_device(chip->client, CPS8851H_REG_IDLE_CTRL, 1, &ctrl);
	if (ret < 0) {
		dev_err(chip->dev, "enter shutdown mode fail\n");
	}

err:
	return ret;
}
#endif

#define OPLUS_CPS8851H_SUB_NODE_NAME_LEN 16
static int oplus_cps8851h_node_name_parse_cmdline_match(char *match_str, char *result, int size)
{
	struct device_node *cmdline_node = NULL;
	const char *cmdline;
	char *match, *match_end;
	int len;
	int match_str_len;
	int ret;

	if (!result || !match_str)
		return -EINVAL;

	memset(result, 0, size);
	match_str_len = strlen(match_str);

	cmdline_node = of_find_node_by_path("/chosen");
	if (!cmdline_node) {
		pr_err("%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return -EINVAL;
	}

	ret = of_property_read_string(cmdline_node, "bootargs", &cmdline);
	if (ret) {
		pr_err("%s failed to read bootargs\n", __func__);
		return -EINVAL;
	}

	match = strstr(cmdline, match_str);
	if (!match) {
		pr_err("match: %s fail in cmdline\n", match_str);
		return -EINVAL;
	}

	match_end = strstr((match + match_str_len), ";");
	if (!match_end) {
		pr_err("match end of : %s fail in cmdline\n", match_str);
		return -EINVAL;
	}

	len = match_end - (match + match_str_len);
	if (len < 0 || len > size) {
		pr_err("match cmdline :%s fail, len = %d\n", match_str, len);
		return -EINVAL;
	}

	memcpy(result, (match + match_str_len), len);

	return 0;
}

static int oplus_get_cps8851h_node_name_str(char *type)
{
	char *str = "typec_sub=";
	char result[32] = {};
	int ret;

	ret = oplus_cps8851h_node_name_parse_cmdline_match(str, result, sizeof(result));
	if (ret < 0) {
		pr_err("match cps8851h node name str fail\n");
		return ret;
	}
	snprintf(type, OPLUS_CPS8851H_SUB_NODE_NAME_LEN, "%s", result);

	return ret;
}

static struct device_node *oplus_get_cps8851h_node_by_charge_ic(struct device_node *father_node)
{
	char cps8851h_node_name_str[OPLUS_CPS8851H_SUB_NODE_NAME_LEN] = { 0 };
	struct device_node *sub_node = NULL;
	struct device_node *node = father_node;

	int rc = oplus_get_cps8851h_node_name_str(cps8851h_node_name_str);
	if (rc == 0) {
		sub_node = of_get_child_by_name(father_node, cps8851h_node_name_str);
		if (sub_node) {
			node = sub_node;
			pr_err("current typec_sub node name str = %s\n", cps8851h_node_name_str);
		}
	}

	return node;
}

#if CPS8851_VBUS_PRES_DEB_TIME
static void cps8851_power_change_work(struct work_struct *work)
{
	struct cps8851_chip *chip = container_of(
			work, struct cps8851_chip, power_change_work.work);

	dev_info(chip->dev, "deferred power_change\n");
	tcpci_lock_typec(chip->tcpc);
	chip->check_real_vbus = true;
	tcpci_alert_power_status_changed(chip->tcpc);
	chip->check_real_vbus = false;
	tcpci_unlock_typec(chip->tcpc);
}
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static int cps8851_i2c_probe(struct i2c_client *client)
#else
static int cps8851_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
#endif
{
	struct cps8851_chip *chip;
	int ret = 0;
	int chip_id;
	u16 chip_pid, chip_vid;
	bool use_dt = client->dev.of_node;
#ifdef CONFIG_TCPC_LOW_POWER_MODE
	/*u8 data;*/
#endif	/* CONFIG_TCPC_LOW_POWER_MODE */

	if (i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_I2C_BLOCK | I2C_FUNC_SMBUS_BYTE_DATA))
		pr_info("I2C functionality : OK...\n");
	else
		pr_info("I2C functionality check : failuare...\n");

	client->dev.of_node = oplus_get_cps8851h_node_by_charge_ic(client->dev.of_node);
	use_dt = client->dev.of_node;

	chip_id = cps8851h_check_revision(client);
	if (chip_id < 0)
		return chip_id;

	cps8851_read_device(client, TCPC_V10_REG_PID, 2, &chip_pid);
	cps8851_read_device(client, TCPC_V10_REG_VID, 2, &chip_vid);

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
	g_chip = chip;
	chip->dev = &client->dev;
	chip->client = client;
	i2c_set_clientdata(client, chip);
#if CPS8851_VBUS_PRES_DEB_TIME
	INIT_DELAYED_WORK(&chip->power_change_work, cps8851_power_change_work);
#endif

	chip->chip_id = chip_id;
	chip->chip_pid = chip_pid;
	chip->chip_vid = chip_vid;
	pr_info("cps8851h_chipID = 0x%0x\n", chip_id);

	ret = cps8851_regmap_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "cps8851 regmap init fail\n");
		goto err_regmap_init;
	}

	ret = cps8851_tcpcdev_init(chip, &client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "cps8851 tcpc dev init fail\n");
		goto err_tcpc_reg;
	}

	ret = cps8851_init_alert(chip->tcpc);
	if (ret < 0) {
		pr_err("cps8851 init alert fail\n");
		goto err_irq_init;
	}

	pr_info("%s probe OK!\n", __func__);
	return 0;

err_irq_init:
	tcpc_device_unregister(chip->dev, chip->tcpc);
err_tcpc_reg:
	cps8851_regmap_deinit(chip);
err_regmap_init:
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void cps8851_i2c_remove(struct i2c_client *client)
#else
static int cps8851_i2c_remove(struct i2c_client *client)
#endif
{
	struct cps8851_chip *chip = i2c_get_clientdata(client);

	if (chip) {
#if CPS8851_VBUS_PRES_DEB_TIME
		cancel_delayed_work_sync(&chip->power_change_work);
 #endif
		tcpc_device_unregister(chip->dev, chip->tcpc);
		cps8851_regmap_deinit(chip);
	}
#if !(LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	return 0;
#endif
}

#ifdef CONFIG_PM
static int cps8851_i2c_suspend(struct device *dev)
{
	struct cps8851_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irq);
	disable_irq(chip->irq);

	return 0;
}

static int cps8851_i2c_resume(struct device *dev)
{
	struct cps8851_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	enable_irq(chip->irq);
	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irq);

	return 0;
}

static void cps8851_shutdown(struct i2c_client *client)
{
	struct cps8851_chip *chip = i2c_get_clientdata(client);

	/* Please reset IC here */
	if (chip != NULL) {
		if (chip->irq)
			disable_irq(chip->irq);
		if (chip->chip_id == HUSB311_DID) {
			i2c_smbus_write_byte_data(
				client, CPS8851H_REG_BMC_CTRL, 0x00);
		}
		tcpm_shutdown(chip->tcpc);
		if (chip->chip_pid == CPS8851_PID)
			mdelay(25);
	} else {
		i2c_smbus_write_byte_data(
			client, CPS8851H_REG_SWRESET, 0x01);
	}
}

#ifdef CONFIG_PM_RUNTIME
static int cps8851_pm_suspend_runtime(struct device *device)
{
	dev_dbg(device, "pm_runtime: suspending...\n");
	return 0;
}

static int cps8851_pm_resume_runtime(struct device *device)
{
	dev_dbg(device, "pm_runtime: resuming...\n");
	return 0;
}
#endif /* #ifdef CONFIG_PM_RUNTIME */

static const struct dev_pm_ops cps8851_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(
			cps8851_i2c_suspend,
			cps8851_i2c_resume)
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(
		cps8851_pm_suspend_runtime,
		cps8851_pm_resume_runtime,
		NULL)
#endif /* #ifdef CONFIG_PM_RUNTIME */
};
#define CPS8851_PM_OPS	(&cps8851_pm_ops)
#else
#define CPS8851_PM_OPS	(NULL)
#endif /* CONFIG_PM */

static const struct i2c_device_id cps8851_id_table[] = {
	{"cps8851", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, cps8851_id_table);

static const struct of_device_id rt_match_table[] = {
	{.compatible = "cps,cps8851h", },
	{},
};

static struct i2c_driver cps8851_driver = {
	.driver = {
		.name = "cps8851",
		.owner = THIS_MODULE,
		.of_match_table = rt_match_table,
		.pm = CPS8851_PM_OPS,
	},
	.probe = cps8851_i2c_probe,
	.remove = cps8851_i2c_remove,
	.shutdown = cps8851_shutdown,
	.id_table = cps8851_id_table,
};

static int __init cps8851_driver_init(void)
{
	int rc;

	rc = i2c_add_driver(&cps8851_driver);
	if (rc < 0)
		pr_err("[OPLUS_CHG]: register cps8851 driver failed, rc=%d", rc);

	return rc;
}
subsys_initcall(cps8851_driver_init);

static void __exit cps8851_driver_exit(void)
{
	i2c_del_driver(&cps8851_driver);
}
module_exit(cps8851_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com>");
MODULE_DESCRIPTION("CPS8851 TCPC Driver");
MODULE_VERSION(CPS8851H_DRV_VERSION);

/**** Release Note ****
 * 2.0.6_G
 * (1) Revert Vconn OC to shutdown mode
 * (2) Revise IRQ handling
 *
 * 2.0.5_G
 * (1) Utilize rt-regmap to reduce I2C accesses
 * (2) Decrease VBUS present threshold (VBUS_CAL) by 60mV (2LSBs)
 *
 * 2.0.4_G
 * (1) Mask vSafe0V IRQ before entering low power mode
 * (2) Disable auto idle mode before entering low power mode
 * (3) Reset Protocol FSM and clear RX alerts twice before clock gating
 *
 * 2.0.3_G
 * (1) Single Rp as Attatched.SRC for Ellisys TD.4.9.4
 *
 * 2.0.2_G
 * (1) Replace wake_lock with wakeup_source
 * (2) Move down the shipping off
 * (3) Add support for NoRp.SRC
 * (4) Reg0x71[7] = 1'b1 to workaround unstable VDD Iq in low power mode
 * (5) Add get_alert_mask of tcpc_ops
 *
 * 2.0.1_G
 * First released PD3.0 Driver
 */

