// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2022 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "[sc6607]:[%s][%d]: " fmt, __func__, __LINE__

#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/proc_fs.h>

#include <trace/events/sched.h>
#include <linux/ktime.h>
#include <uapi/linux/sched/types.h>
#include <oplus_chg.h>
#include <ufcs_class.h>
#include <oplus_mms.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_module.h>
#include "oplus_hal_sc6607_ufcs.h"
#include "../charger_ic/oplus_hal_sc6607.h"

struct sc6607_ufcs {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex i2c_rw_lock;

	atomic_t suspended;
	bool ufcs_enable;
	bool error_reported;
	u8 ufcs_reg_dump[SC6607_FLAG_NUM];
	bool iic_err;
	u32 iic_err_num;
	struct ufcs_dev *ufcs;
	struct oplus_mms *err_topic;
	struct mms_subscribe *err_subs;
	struct work_struct ufcs_regdump_work;
	struct sc6607 *sc6607_buck;
	struct delayed_work get_buck_info_work;
	int found_buck_client_count;
};

#define ERR_MSG_BUF	PAGE_SIZE
__printf(3, 4)
static int sc6607_publish_ic_err_msg(struct sc6607_ufcs *chip, int sub_type, const char *format, ...)
{
	struct mms_msg *topic_msg;
	va_list args;
	char *buf;
	int rc;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	va_start(args, format);
	vsnprintf(buf, ERR_MSG_BUF, format, args);
	va_end(args);

	topic_msg =
		oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
					"[%s]-[%d]-[%d]:%s", "cp-sc6607",
					OPLUS_IC_ERR_UFCS, sub_type,
					buf);
	kfree(buf);
	if (topic_msg == NULL) {
		chg_err("alloc topic msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg(chip->err_topic, topic_msg);
	if (rc < 0) {
		chg_err("publish error topic msg error, rc=%d\n", rc);
		kfree(topic_msg);
	}
	return rc;
}

static void sc6607_i2c_error(struct sc6607_ufcs *chip, bool happen, bool read)
{
	if (!chip || chip->error_reported)
		return;

	if (happen) {
		chip->iic_err = true;
		chip->iic_err_num++;
		if (chip->iic_err_num >= I2C_ERR_NUM) {
			if (chip->err_topic != NULL)
				sc6607_publish_ic_err_msg(
					chip, read ? OPLUS_IC_ERR_UFCS : UFCS_ERR_REG_DUMP,
					"%s error", read ? "read" : "write");
			chip->error_reported = true;
		}
	} else {
		chip->iic_err_num = 0;
	}
}

static int sc6607_read_byte(struct sc6607_ufcs *chip, u8 addr, u8 *data)
{
	int rc = 0;

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_smbus_read_byte_data(chip->client, addr);
	if (rc < 0) {
		chg_err("read 0x%02x error, rc = %d \n", addr, rc);
		sc6607_i2c_error(chip, true, true);
		goto error;
	}
	sc6607_i2c_error(chip, false, true);
	mutex_unlock(&chip->i2c_rw_lock);
	*data = rc;
	return 0;

error:
	mutex_unlock(&chip->i2c_rw_lock);
	return rc;
}

#define I2C_MSG_LEN	2
static int sc6607_read_data(struct sc6607_ufcs *chip, u8 addr, u8 *buf, int len)
{
	int rc = 0;
	struct i2c_msg msg[I2C_MSG_LEN] = {0};

	if (!chip)
		return -EINVAL;

	msg[0].addr = chip->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;

	msg[1].addr = chip->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_transfer(chip->client->adapter, msg, I2C_MSG_LEN);
	if (rc < 0) {
		chg_err("read 0x%02x error, rc=%d\n", addr, rc);
		sc6607_i2c_error(chip, true, true);
		goto error;
	}
	sc6607_i2c_error(chip, false, true);
	mutex_unlock(&chip->i2c_rw_lock);
	return 0;

error:
	mutex_unlock(&chip->i2c_rw_lock);
	return rc;
}

static int sc6607_write_byte(struct sc6607_ufcs *chip, u8 addr, u8 data)
{
	int rc = 0;
	u8 buf[2] = {addr & 0xff, data};

	if (!chip)
		return -EINVAL;

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_master_send(chip->client, buf, 2);
	if (rc < 2) {
		chg_err("write 0x%02x error, rc = %d \n", addr, rc);
		sc6607_i2c_error(chip, true, false);
		mutex_unlock(&chip->i2c_rw_lock);
		return rc;
	}
	sc6607_i2c_error(chip, false, false);
	mutex_unlock(&chip->i2c_rw_lock);
	return 0;
}

static int sc6607_write_data(struct sc6607_ufcs *chip, u8 addr, u16 length, u8 *data)
{
	u8 *buf;
	int rc = 0;

	if (!chip)
		return -EINVAL;

	buf = kzalloc(length + 1, GFP_KERNEL);
	if (!buf) {
		chg_err("alloc memorry for i2c buffer error\n");
		return -ENOMEM;
	}

	buf[0] = addr & 0xff;
	memcpy(&buf[1], data, length);

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_master_send(chip->client, buf, length + 1);
	if (rc < length + 1) {
		chg_err("write 0x%02x error, ret = %d \n", addr, rc);
		sc6607_i2c_error(chip, true, false);
		mutex_unlock(&chip->i2c_rw_lock);
		kfree(buf);
		return rc;
	}
	sc6607_i2c_error(chip, false, false);
	mutex_unlock(&chip->i2c_rw_lock);
	kfree(buf);
	return 0;
}

static int sc6607_write_bit_mask(struct sc6607_ufcs *chip, u8 addr, u8 mask, u8 data)
{
	u8 temp = 0;
	int rc = 0;

	if (!chip)
		return -EINVAL;

	rc = sc6607_read_byte(chip, addr, &temp);
	if (rc < 0)
		return rc;

	temp = (data & mask) | (temp & (~mask));

	rc = sc6607_write_byte(chip, addr, temp);
	if (rc < 0)
		return rc;

	return 0;
}

static int sc6607_ufcs_init(struct ufcs_dev *ufcs)
{
	return 0;
}

static int sc6607_ufcs_write_msg(struct ufcs_dev *ufcs, unsigned char *buf, int len)
{
	int rc;
	struct sc6607_ufcs *chip;

	if (!ufcs || !ufcs->drv_data)
		return -EINVAL;

	chip = ufcs->drv_data;
	if (!chip)
		return -EINVAL;

	if (chip->sc6607_buck)
		mutex_lock(&chip->sc6607_buck->adc_read_lock);
	rc = sc6607_write_byte(chip, SC6607_ADDR_TX_LENGTH, len);
	if (rc < 0) {
		chg_err("write tx buf len error, rc=%d\n", rc);
		goto err;
	}
	rc = sc6607_write_data(chip, SC6607_ADDR_TX_BUFFER0, len, buf);
	if (rc < 0) {
		chg_err("write tx buf error, rc=%d\n", rc);
		goto err;
	}
	rc = sc6607_write_bit_mask(chip, SC6607_ADDR_UFCS_CTRL0, SC6607_MASK_SND_CMP, SC6607_CMD_SND_CMP);
	if (rc < 0) {
		chg_err("write tx buf send cmd error, rc=%d\n", rc);
		goto err;
	}
	usleep_range(4000, 4000);
err:
	if (chip->sc6607_buck)
		mutex_unlock(&chip->sc6607_buck->adc_read_lock);
	return rc;
}

static int sc6607_ufcs_read_msg(struct ufcs_dev *ufcs, unsigned char *buf, int len)
{
	u8 rx_buf_len;
	int rc;
	struct sc6607_ufcs *chip;

	if (!ufcs || !ufcs->drv_data)
		return -EINVAL;

	chip = ufcs->drv_data;

	rc = sc6607_read_byte(chip, SC6607_ADDR_RX_LENGTH, &rx_buf_len);
	if (rc < 0) {
		chg_err("can't read rx buf len, rc=%d\n", rc);
		return rc;
	}
	if (rx_buf_len > len) {
		chg_err("rx_buf_len = %d, limit to %d\n", rx_buf_len, len);
		rx_buf_len = len;
	}
	rc = sc6607_read_data(chip, SC6607_ADDR_RX_BUFFER0, buf, rx_buf_len);
	if (rc < 0) {
		chg_err("can't read rx buf, rc=%d\n", rc);
		return rc;
	}
	return (int)rx_buf_len;
}

static int sc6607_ufcs_handshake(struct ufcs_dev *ufcs)
{
	int rc;
	struct sc6607_ufcs *chip;

	if (!ufcs || !ufcs->drv_data)
		return -EINVAL;

	chip = ufcs->drv_data;

	rc = sc6607_write_bit_mask(chip, SC6607_ADDR_UFCS_CTRL0, SC6607_MASK_EN_HANDSHAKE, SC6607_CMD_EN_HANDSHAKE);
	if (rc < 0)
		chg_err("send handshake error, rc=%d\n", rc);
	return rc;
}

static int sc6607_ufcs_source_hard_reset(struct ufcs_dev *ufcs)
{
	int rc;
	int retry_count = 0;
	struct sc6607_ufcs *chip;

	if (!ufcs || !ufcs->drv_data)
		return -EINVAL;

	chip = ufcs->drv_data;

retry:
	retry_count++;
	if (retry_count > UFCS_HARDRESET_RETRY_CNTS) {
		chg_err("send hard reset, retry count over!\n");
		return -EBUSY;
	}
	rc = sc6607_write_bit_mask(chip, SC6607_ADDR_UFCS_CTRL0,
			SC6607_SEND_SOURCE_HARDRESET, SC6607_SEND_SOURCE_HARDRESET);
	if (rc < 0) {
		chg_err("send source hardreset error\n");
		goto retry;
	}

	msleep(100);
	return rc;
}

static int sc6607_ufcs_cable_hard_reset(struct ufcs_dev *ufcs)
{
	return 0;
}

static int sc6607_ufcs_set_baud_rate(struct ufcs_dev *ufcs, enum ufcs_baud_rate baud)
{
	int rc;
	struct sc6607_ufcs *chip;

	if (!ufcs || !ufcs->drv_data)
		return -EINVAL;

	chip = ufcs->drv_data;

	rc = sc6607_write_bit_mask(chip, SC6607_ADDR_UFCS_CTRL0, SC6607_FLAG_BAUD_RATE_VALUE,
				(baud << SC6607_FLAG_BAUD_NUM_SHIFT));
	if (rc < 0)
		chg_err("set baud rate error, rc=%d\n", rc);
	return rc;
}

static int sc6607_ufcs_enable(struct ufcs_dev *ufcs)
{
	u8 addr_buf[SC6607_ENABLE_REG_NUM] = {
				SC6607_ADDR_UFCS_CTRL0,
				SC6607_ADDR_UFCS_CTRL1,
				SC6607_ADDR_UFCS_INT_MASK1,
				SC6607_ADDR_UFCS_INT_MASK2 };
	u8 cmd_buf[SC6607_ENABLE_REG_NUM] = {
				SC6607_CMD_EN_CHIP,
				SC6607_CMD_CLR_TX_RX,
				SC6607_CMD_MASK_ACK_TIMEOUT,
				SC6607_MASK_TRANING_BYTE_ERROR };
	int i, rc;
	struct sc6607_ufcs *chip;

	if (!ufcs || !ufcs->drv_data)
		return -EINVAL;

	chip = ufcs->drv_data;

	if (chip->sc6607_buck && chip->sc6607_buck->is_sc6607a)
		sc6607a_set_dpdm_ctrl(chip->sc6607_buck, true);

	for (i = 0; i < SC6607_ENABLE_REG_NUM; i++) {
		rc = sc6607_write_byte(chip, addr_buf[i], cmd_buf[i]);
		if (rc < 0) {
			chg_err("write i2c failed!\n");
			return rc;
		}
	}
	chip->ufcs_enable = true;
	/* hardreset signal to 1900us ; ack timeout to 10ms + rx_FRAME_TIME */
	rc = sc6607_write_byte(chip, SC6607_UFCS_OPTION2, 0x80);
	if (rc < 0) {
		chg_err("write UFCS_OPTION2 failed(%d)!\n", rc);
		return rc;
	}

	return 0;
}

static int sc6607_ufcs_disable(struct ufcs_dev *ufcs)
{
	int rc;
	struct sc6607_ufcs *chip;

	if (!ufcs || !ufcs->drv_data)
		return -EINVAL;

	chip = ufcs->drv_data;

	chip->ufcs_enable = false;
	rc = sc6607_write_byte(chip, SC6607_ADDR_UFCS_CTRL0, SC6607_CMD_DIS_CHIP);
	if (rc < 0) {
		chg_err("write i2c failed\n");
		return rc;
	}

	if (chip->sc6607_buck && chip->sc6607_buck->is_sc6607a)
		sc6607a_set_dpdm_ctrl(chip->sc6607_buck, false);

	return 0;
}


static int sc6607_retrieve_reg_flags(struct sc6607_ufcs *chip)
{
	unsigned int err_flag = 0;
	int rc = 0;
	u8 flag_buf[SC6607_FLAG_NUM] = { 0 };

	rc = sc6607_read_data(chip, SC6607_ADDR_GENERAL_INT_FLAG1, flag_buf, SC6607_FLAG_NUM);
	if (rc < 0) {
		chg_err("failed to read flag register\n");
		return -EBUSY;
	}
	memcpy(chip->ufcs_reg_dump, flag_buf, SC6607_FLAG_NUM);
	if (flag_buf[0] & SC6607_FLAG_SENT_PACKET_COMPLETE)
		err_flag |= BIT(UFCS_RECV_ERR_SENT_CMP);
	if (flag_buf[0] & SC6607_FLAG_DATA_READY)
		err_flag |= BIT(UFCS_RECV_ERR_DATA_READY);
	if (flag_buf[0] & SC6607_FLAG_RX_OVERFLOW)
		err_flag |= BIT(UFCS_COMM_ERR_RX_OVERFLOW);
	if (flag_buf[0] & SC6607_FLAG_MSG_TRANS_FAIL)
		err_flag |= BIT(UFCS_RECV_ERR_TRANS_FAIL);
	if (flag_buf[0] & SC6607_FLAG_ACK_RECEIVE_TIMEOUT)
		err_flag |= BIT(UFCS_RECV_ERR_ACK_TIMEOUT);
	if (flag_buf[1] & SC6607_FLAG_BAUD_RATE_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_BAUD_RATE_ERR);
	if (flag_buf[1] & SC6607_FLAG_TRAINING_BYTE_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_TRAINING_ERR);
	if (flag_buf[1] & SC6607_FLAG_DATA_BYTE_TIMEOUT)
		err_flag |= BIT(UFCS_COMM_ERR_BYTE_TIMEOUT);
	if (flag_buf[1] & SC6607_FLAG_LENGTH_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_RX_LEN_ERR);
	if (flag_buf[1] & SC6607_FLAG_START_FAIL)
		err_flag |= BIT(UFCS_COMM_ERR_START_FAIL);
	if (flag_buf[1] & SC6607_FLAG_STOP_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_STOP_ERR);
	if (flag_buf[1] & SC6607_FLAG_CRC_ERROR)
		err_flag |= BIT(SC6607_FLAG_CRC_ERROR);
	if (flag_buf[1] & SC6607_FLAG_HARD_RESET)
		err_flag |= BIT(UFCS_HW_ERR_HARD_RESET);
	if (flag_buf[2] & SC6607_FLAG_BUS_CONFLICT)
		err_flag |= BIT(UFCS_COMM_ERR_BUS_CONFLICT);
	if (flag_buf[2] & SC6607_FLAG_BAUD_RATE_CHANGE)
		err_flag |= BIT(UFCS_COMM_ERR_BAUD_RATE_CHANGE);
	chip->ufcs->err_flag_save = err_flag;
	if (chip->ufcs->handshake_state == UFCS_HS_WAIT) {
		if ((flag_buf[0] & SC6607_FLAG_HANDSHAKE_SUCCESS) && !(flag_buf[0] & SC6607_FLAG_HANDSHAKE_FAIL)) {
			chip->ufcs->handshake_state = UFCS_HS_SUCCESS;
		 } else if (flag_buf[0] & SC6607_FLAG_HANDSHAKE_FAIL) {
			chip->ufcs->handshake_state = UFCS_HS_FAIL;
		 }
	}
	chg_info("[0x%x, 0x%x, 0x%x], err_flag=0x%x\n", flag_buf[0], flag_buf[1], flag_buf[2], err_flag);
	return ufcs_set_error_flag(chip->ufcs, err_flag);
}

static int sc6607_ufcs_event_handler(struct ufcs_dev *ufcs)
{
	int rc;
	struct sc6607_ufcs *chip;

	if (!ufcs || !ufcs->drv_data)
		return -EINVAL;

	chip = ufcs->drv_data;

	sc6607_retrieve_reg_flags(chip);
	rc = ufcs_msg_handler(ufcs);
	return rc;
}

static void sc6607_ufcs_regdump_work(struct work_struct *work)
{
	struct sc6607_ufcs *chip = container_of(work, struct sc6607_ufcs, ufcs_regdump_work);
	struct mms_msg *topic_msg;
	char *buf;
	int rc;
	int i;
	size_t index = 0;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return;

	for (i = 0; i < SC6607_FLAG_NUM; i++)
		index += snprintf(buf + index, ERR_MSG_BUF, "0x%04x=%02x,",
			(SC6607_ADDR_GENERAL_INT_FLAG1 + i), chip->ufcs_reg_dump[i]);
	if (index > 0)
		buf[index - 1] = 0;

	topic_msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
					"[%s]-[%d]-[%d]:$$reg_info@@%s",
					"ufcs-sc6607",
					OPLUS_IC_ERR_UFCS, UFCS_ERR_REG_DUMP,
					buf);
	kfree(buf);
	if (topic_msg == NULL) {
		chg_err("alloc topic msg error\n");
		return;
	}

	rc = oplus_mms_publish_msg(chip->err_topic, topic_msg);
	if (rc < 0) {
		chg_err("publish error topic msg error, rc=%d\n", rc);
		kfree(topic_msg);
	}
}

static void sc6607_err_subs_callback(struct mms_subscribe *subs, enum mms_msg_type type, u32 id, bool sync)
{
	struct sc6607_ufcs *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case ERR_ITEM_UFCS:
			schedule_work(&chip->ufcs_regdump_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void sc6607_subscribe_error_topic(struct oplus_mms *topic, void *prv_data)
{
	struct sc6607_ufcs *chip = prv_data;

	chip->err_topic = topic;
	chip->err_subs = oplus_mms_subscribe(chip->err_topic, chip, sc6607_err_subs_callback, "sc6607");
	if (IS_ERR_OR_NULL(chip->err_subs)) {
		chg_err("subscribe error topic error, rc=%ld\n", PTR_ERR(chip->err_subs));
		return;
	}
}

static struct ufcs_config sc6607_ufcs_config = {
	.check_crc = false,
	.reply_ack = false,
	.msg_resend = false,
	.handshake_hard_retry = true,
	.ic_vendor_id = SC6607_VENDOR_ID,
};

static struct ufcs_dev_ops ufcs_ops = {
	.init = sc6607_ufcs_init,
	.write_msg = sc6607_ufcs_write_msg,
	.read_msg = sc6607_ufcs_read_msg,
	.handshake = sc6607_ufcs_handshake,
	.source_hard_reset = sc6607_ufcs_source_hard_reset,
	.cable_hard_reset = sc6607_ufcs_cable_hard_reset,
	.set_baud_rate = sc6607_ufcs_set_baud_rate,
	.enable = sc6607_ufcs_enable,
	.disable = sc6607_ufcs_disable,
	.irq_event_handler = sc6607_ufcs_event_handler,
};

static int sc6607_charger_choose(struct sc6607_ufcs *chip)
{
	int rc = 0;
	u16 addr = 0x0;
	u8 val_buf = 0x0;

	rc = sc6607_read_byte(chip, addr, &val_buf);
	if (rc < 0) {
		chg_err("couldn't read 0x%02x rc = %d\n", addr, rc);
		return -EPROBE_DEFER;
	} else {
		return 0;
	}
}

static int sc6607_dump_registers(struct sc6607_ufcs *chip)
{
	int rc = 0;
	u16 addr = 0x0;
	u8 val_buf[6] = { 0x0 };

	if (!chip)
		return -EINVAL;

	if (atomic_read(&chip->suspended)) {
		chg_err("ready suspend\n");
		return -ENODEV;
	}

	for (addr = 0x0; addr <= 0x05; addr++) {
		rc = sc6607_read_byte(chip, addr, &val_buf[addr]);
		if (rc < 0) {
			chg_err("couldn't read 0x%02x rc = %d\n", addr, rc);
			break;
		}
	}

	chg_info(":[0~5][0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n", val_buf[0],
		 val_buf[1], val_buf[2], val_buf[3], val_buf[4], val_buf[5]);

	return 0;
}

static int sc6607_hardware_init(struct sc6607_ufcs *chip)
{
	int rc = 0;

	rc = sc6607_charger_choose(chip);
	if (rc < 0)
		return rc;

	sc6607_dump_registers(chip);

	return rc;
}

static bool sc6607_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static struct regmap_config sc6607_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = SC6607_MAX_REG,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = sc6607_is_volatile_reg,
};

static int find_buck_i2c_clients(struct device *dev, void *data)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct sc6607_ufcs *chip = data;
	if (client) {
		chg_info("addr=0x%x name:%s\n", client->addr, client->name);
		if (strncmp(client->name, SC6607_BUCK_NAME, strlen(SC6607_BUCK_NAME)) == 0) {
			chip->sc6607_buck = i2c_get_clientdata(client);
			chg_info("found\n");
		}
	}
	return 0;
}

static void sc6607_get_buck_info_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc6607_ufcs *chip = container_of(dwork, struct sc6607_ufcs, get_buck_info_work);
	struct i2c_adapter *adap;

	adap = chip->client->adapter;
	device_for_each_child(&adap->dev, chip, find_buck_i2c_clients);
	chip->found_buck_client_count++;
	if (!chip->sc6607_buck && chip->found_buck_client_count < FOUND_BUCK_ADDR_MAX_COUNT)
		schedule_delayed_work(&chip->get_buck_info_work, msecs_to_jiffies(FOUND_BUCK_ADDR_MAX_DELAY));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sc6607_ufcs_probe(struct i2c_client *client)
#else
static int sc6607_ufcs_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct sc6607_ufcs *chip;

	int rc;
	chg_info("start!\n");

	chip = devm_kzalloc(&client->dev, sizeof(struct sc6607_ufcs), GFP_KERNEL);
	if (!chip) {
		chg_err("failed to allocate memory\n");
		return -ENOMEM;
	}

	chip->regmap = devm_regmap_init_i2c(client, &sc6607_regmap_config);
	if (!chip->regmap) {
		rc = -ENODEV;
		goto regmap_init_err;
	}

	chip->dev = &client->dev;
	chip->client = client;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->i2c_rw_lock);

	rc = sc6607_hardware_init(chip);
	if (rc < 0) {
		chg_err("init failed, rc = %d!\n", rc);
		goto regmap_init_err;
	}

	INIT_WORK(&chip->ufcs_regdump_work, sc6607_ufcs_regdump_work);
	INIT_DELAYED_WORK(&chip->get_buck_info_work, sc6607_get_buck_info_work);
	chip->ufcs = ufcs_device_register(chip->dev, &ufcs_ops, chip, &sc6607_ufcs_config);
	if (IS_ERR_OR_NULL(chip->ufcs)) {
		chg_err("ufcs device register error\n");
		rc = -ENODEV;
		goto regmap_init_err;
	}
	schedule_delayed_work(&chip->get_buck_info_work, msecs_to_jiffies(FOUND_BUCK_ADDR_MAX_DELAY));
	oplus_mms_wait_topic("error", sc6607_subscribe_error_topic, chip);
	chg_info("end!\n");
	return 0;

regmap_init_err:
	mutex_destroy(&chip->i2c_rw_lock);
	devm_kfree(&client->dev, chip);
	return rc;
}

static int sc6607_pm_resume(struct device *dev_chip)
{
	struct i2c_client *client = container_of(dev_chip, struct i2c_client, dev);
	struct sc6607_ufcs *chip = i2c_get_clientdata(client);

	if (!chip)
		return 0;

	atomic_set(&chip->suspended, 0);
	return 0;
}

static int sc6607_pm_suspend(struct device *dev_chip)
{
	struct i2c_client *client = container_of(dev_chip, struct i2c_client, dev);
	struct sc6607_ufcs *chip = i2c_get_clientdata(client);

	if (!chip)
		return 0;

	atomic_set(&chip->suspended, 1);
	return 0;
}

static const struct dev_pm_ops sc6607_pm_ops = {
	.resume = sc6607_pm_resume,
	.suspend = sc6607_pm_suspend,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void sc6607_ufcs_remove(struct i2c_client *client)
#else
static int sc6607_ufcs_remove(struct i2c_client *client)
#endif
{
	struct sc6607_ufcs *chip = i2c_get_clientdata(client);
	if (!chip) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
		return;
#else
		return 0;
#endif
	}

	mutex_destroy(&chip->i2c_rw_lock);
	if (chip->ufcs)
		ufcs_device_unregister(chip->ufcs);
	devm_kfree(&client->dev, chip);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	return;
#else
	return 0;
#endif
}

static void sc6607_ufcs_shutdown(struct i2c_client *chip_client)
{
	return;
}

static const struct of_device_id sc6607_ufcs_match[] = {
	{.compatible = "oplus,sc6607-ufcs" },
	{},
};
MODULE_DEVICE_TABLE(of, sc6607_ufcs_match);

static const struct i2c_device_id sc6607_ufcs_id[] = {
	{ "sc6607,ufcs", 0x63 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sc6607_ufcs_id);

static struct i2c_driver sc6607_ufcs_driver = {
	.driver =
		{
			.name = "sc6607-ufcs",
			.owner = THIS_MODULE,
			.of_match_table = sc6607_ufcs_match,
			.pm = &sc6607_pm_ops,
		},
	.probe = sc6607_ufcs_probe,
	.remove = sc6607_ufcs_remove,
	.id_table = sc6607_ufcs_id,
	.shutdown = sc6607_ufcs_shutdown,
};

int sc6607_ufcs_i2c_driver_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&sc6607_ufcs_driver) != 0)
		chg_err("failed to register sc6607 ufcs driver\n");
	else
		chg_info("success to register sc6607 ufcs driver\n");

	return ret;
}

void sc6607_ufcs_i2c_driver_exit(void)
{
	i2c_del_driver(&sc6607_ufcs_driver);
}
oplus_chg_module_register(sc6607_ufcs_i2c_driver);

MODULE_DESCRIPTION("SC6607 UFCS Driver");
MODULE_LICENSE("GPL v2");
