/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_sc83107.c
** Description: hybridboost ic
** Date: 2025-11-01
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#define pr_fmt(fmt) "[SC83107]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
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
#include <linux/pinctrl/consumer.h>
#include <linux/time.h>
#include <linux/sched/clock.h>
#include <linux/pm_wakeup.h>

#include "oplus_sc83107.h"
#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_impedance_check.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_voter.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_mutual.h>
#include <oplus_chg_comm.h>

#define SC83107_DRV_VERSION		"1.1"
#define SC83107_REGMAX			0x0F

#define SC83107_I2C_RETRY_MAX_COUNT	3	/* I2C retry max count */
#define SC83107_I2C_STATE		"i2c-state"
#define SC83107_OUTPUT_LOW_STATE	"output-low-state"
#define SC83107_I2C_RECOVERY_DELAY_MS	50	/* I2C recovery delay in milliseconds */
#define SC83107_MODE_AUTO_HYBRID_BP	0
#define SC83107_MODE_FORCE_BP		1
#define SC83107_MODE_SET_RETRY_MAX	3
#define SC83107_UPLOAD_REG_SOC_THRESHOLD 10
static int sc83107_publish_ic_err_msg(int type, int sub_type, const char *format, ...)
{
	va_list args;
	char *buf;
	int rc;
	struct mms_msg *topic_msg;
	struct oplus_mms *err_topic = oplus_mms_get_by_name("error");

	if (!err_topic)
		return -ENODEV;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	va_start(args, format);
	vsnprintf(buf, ERR_MSG_BUF, format, args);
	va_end(args);

	topic_msg =
		oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
		"[%s]-[%d]-[%d]:%s", "sc83107", type, sub_type, buf);
	kfree(buf);
	if (topic_msg == NULL) {
		chg_err("alloc topic msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(err_topic, topic_msg);
	if (rc < 0) {
		chg_err("publish error topic msg error, rc=%d\n", rc);
		kfree(topic_msg);
	}

	return rc;
}

static void sc83107_upload_i2c_err_info(struct sc83107_chip *chip, bool read, s32 *err_info)
{
	char *buf;
	size_t index = 0;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return;

	index += scnprintf(buf + index, ERR_MSG_BUF - index,
		"$$i2c_type@@%s$$err_reg@@0x%x$$err_reason@@%d", read ? "read" : "write", err_info[0], err_info[1]);
	/* scnprintf already null-terminates the string, no need to overwrite the last character */

	sc83107_publish_ic_err_msg(OPLUS_IC_ERR_I2C, 0, "%s", buf);
	kfree(buf);
}

static void sc83107_pull_down_int_gpio(struct sc83107_chip *chip)
{
	if (!chip)
		return;

	if (gpio_is_valid(chip->irq_gpio)) {
		/* Configure GPIO as output and set to low */
		gpio_direction_output(chip->irq_gpio, 0);
		chip->gpio_pulled_down = true;
		chg_err("I2C communication failed, pull down INT GPIO %d\n", chip->irq_gpio);
	} else {
		chg_err("INT GPIO %d is not valid, cannot pull down\n", chip->irq_gpio);
	}
}

static void sc83107_restore_int_gpio(struct sc83107_chip *chip)
{
	if (!chip)
		return;

	/* Only restore if GPIO was previously pulled down */
	if (chip->gpio_pulled_down && gpio_is_valid(chip->irq_gpio)) {
		gpio_direction_input(chip->irq_gpio);
		if (chip->pinctrl && chip->boost_inter_active) {
			pinctrl_select_state(chip->pinctrl, chip->boost_inter_active);
		}
		chip->gpio_pulled_down = false;
		chg_info("I2C communication recovered, restore INT GPIO %d to input mode\n", chip->irq_gpio);
	}
}

/**
 * @brief Use pinctrl to pull down SCL and SDA for I2C bus reset (synchronous)
 *
 * @param chip Pointer to SC83107 chip structure
 *
 * @return 0 on success, negative error code on failure
 *
 * @note This function synchronously uses the I2C adapter's pinctrl to pull down
 *       both SCL and SDA lines for 50ms, then restores them to normal I2C state.
 *       This is used to reset I2C bus when retries fail.
 *       Note: This function should only be called when i2c_bus_reset_enable is true.
 *       The caller is responsible for checking the feature enable flag before calling.
 */
static int sc83107_i2c_bus_reset(struct sc83107_chip *chip)
{
	struct pinctrl *pctrl = NULL;
	struct pinctrl_state *i2c_state = NULL;
	struct pinctrl_state *output_low_state = NULL;
	struct device *adapter_dev = NULL;
	int ret = 0;

	if (!chip || !chip->client || !chip->client->adapter) {
		chg_err("Invalid chip or I2C client/adapter\n");
		return -EINVAL;
	}

	/* Get the adapter's parent device (platform device) to access its pinctrl
	 * The pinctrl is configured on the platform device (i2c12 node), not on the adapter device
	 * From i2c-mt65xx.c: i2c->adap.dev.parent = &pdev->dev, and i2c->pctrl = devm_pinctrl_get(&pdev->dev)
	 */
	adapter_dev = chip->client->adapter->dev.parent;
	if (!adapter_dev) {
		chg_err("Failed to get adapter parent device (platform device)\n");
		return -ENODEV;
	}

	/* Get pinctrl from platform device (same as platform's i2c->pctrl) */
	pctrl = devm_pinctrl_get(adapter_dev);
	if (IS_ERR_OR_NULL(pctrl)) {
		chg_err("Failed to get pinctrl from I2C adapter, rc=%ld\n", PTR_ERR(pctrl));
		return PTR_ERR(pctrl);
	}

	/* Lookup I2C state (normal I2C operation) */
	i2c_state = pinctrl_lookup_state(pctrl, SC83107_I2C_STATE);
	if (IS_ERR_OR_NULL(i2c_state)) {
		chg_err("Failed to get pinctrl state: %s, rc=%ld\n", SC83107_I2C_STATE, PTR_ERR(i2c_state));
		/* devm_pinctrl_get() resources are automatically released by the kernel
		 * when the device is removed. Do not call devm_pinctrl_put() manually.
		 */
		return PTR_ERR(i2c_state);
	}

	/* Lookup output-low-state (pull down SCL/SDA) */
	output_low_state = pinctrl_lookup_state(pctrl, SC83107_OUTPUT_LOW_STATE);
	if (IS_ERR_OR_NULL(output_low_state)) {
		chg_err("Failed to get pinctrl state: %s, rc=%ld\n", SC83107_OUTPUT_LOW_STATE, PTR_ERR(output_low_state));
		/* devm_pinctrl_get() resources are automatically released by the kernel
		 * when the device is removed. Do not call devm_pinctrl_put() manually.
		 */
		return PTR_ERR(output_low_state);
	}

	chg_info("I2C bus reset start: pull down SCL/SDA for %dms\n", SC83107_I2C_RECOVERY_DELAY_MS);

	/* Switch to output-low-state: pull down both SCL and SDA */
	ret = pinctrl_select_state(pctrl, output_low_state);
	if (ret < 0) {
		chg_err("Failed to set pinctrl state: %s, rc=%d\n", SC83107_OUTPUT_LOW_STATE, ret);
		/* devm_pinctrl_get() resources are automatically released by the kernel
		 * when the device is removed. Do not call devm_pinctrl_put() manually.
		 */
		return ret;
	}

	/* Wait with SCL/SDA pulled down (synchronous blocking call) */
	msleep(SC83107_I2C_RECOVERY_DELAY_MS);

	/* Switch back to i2c-state: restore normal I2C operation */
	ret = pinctrl_select_state(pctrl, i2c_state);
	if (ret < 0) {
		chg_err("Failed to set pinctrl state: %s, rc=%d\n", SC83107_I2C_STATE, ret);
		/* devm_pinctrl_get() resources are automatically released by the kernel
		 * when the device is removed. Do not call devm_pinctrl_put() manually.
		 */
		return ret;
	}

	chg_info("I2C bus reset completed successfully\n");

	/* devm_pinctrl_get() resources are automatically released by the kernel
	 * when the device is removed. Do not call devm_pinctrl_put() manually.
	 */

	return 0;
}

/********************* Forward declarations *********************/
static int sc83107_mode_set(struct sc83107_chip *chip, uint8_t mode);

/********************* I2C API *********************/

/**
 * @brief Write multiple bytes to a register
 *
 * @param Pointer to SC850x chip structure
 * @param reg Register address to write to
 * @param len Number of bytes to write (max I2C block size)
 * @param val Pointer to data buffer containing bytes to write
 *
 * @return 0 on successful operation,
 *         negative error code on failure:
 *         - -EIO: I2C transfer error
 *         - -ENXIO: I2C device not found
 *         - Other I2C SMBus error codes
 *
 * @note Uses I2C block write operation for efficient multi-byte transfer
 */
static int sc83107_i2c_write_bytes(struct sc83107_chip *chip, uint8_t reg, uint8_t len, uint8_t *val)
{
	uint8_t *buf = NULL;
	int ret = 0;
	int retry;
	int reset_ret;
	struct i2c_client *i2c = chip->client;

	/* Skip I2C operations if system is suspended */
	if (atomic_read(&chip->suspended)) {
		chg_info("system suspended, skip I2C write, reg=0x%02x\n", reg);
		return -EAGAIN;
	}

	struct i2c_msg msg = {
		.addr = i2c->addr,
		.flags = 0,
		.len = len + 1,
	};

	buf = kzalloc(len + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg;
	memcpy(&(buf[1]), val, len);

	msg.buf = buf;

	/* First round: retry SC83107_I2C_RETRY_MAX_COUNT times */
	for (retry = 0; retry < SC83107_I2C_RETRY_MAX_COUNT; retry++) {
		ret = i2c_transfer(i2c->adapter, &msg, 1);
		if (ret > 0) {
			break;
		}
		/* ret == 0 or ret < 0 means failure, continue retry */
	}

	/* If first round failed, handle according to I2C bus reset feature status */
	if (ret <= 0) {
		/* I2C bus reset feature enabled: reset bus and retry again */
		if (chip->i2c_bus_reset_enable) {
			chg_err("I2C write failed after %d retries, reg=0x%02x, ret=%d, attempting I2C bus reset\n",
				SC83107_I2C_RETRY_MAX_COUNT, reg, ret);
			reset_ret = sc83107_i2c_bus_reset(chip);
			if (reset_ret < 0)
				chg_err("I2C bus reset failed, rc=%d, but will still retry\n", reset_ret);
			/* Wait a small delay after reset before retry */
			msleep(10);

			/* Second round: retry SC83107_I2C_RETRY_MAX_COUNT times after reset */
			for (retry = 0; retry < SC83107_I2C_RETRY_MAX_COUNT; retry++) {
				ret = i2c_transfer(i2c->adapter, &msg, 1);
				if (ret > 0) {
					chg_info("I2C write succeeded after bus reset, reg=0x%02x\n", reg);
					break;
				}
				/* ret == 0 or ret < 0 means failure, continue retry */
			}
			/* If second round also failed, ret still contains the error code from i2c_transfer */
		}
		/* Original code path: I2C bus reset feature disabled */
		if (ret <= 0) {
			if (chip->i2c_bus_reset_enable)
				chg_err("I2C write failed after %d retries + reset + %d retries, reg=0x%02x, ret=%d\n",
					SC83107_I2C_RETRY_MAX_COUNT, SC83107_I2C_RETRY_MAX_COUNT, reg, ret);
			else
				chg_err("I2C write failed after %d retries, reg=0x%02x, ret=%d\n",
					SC83107_I2C_RETRY_MAX_COUNT, reg, ret);
			/* Pull down INT GPIO when I2C retry fails */
			sc83107_pull_down_int_gpio(chip);
			/* Set ret to error code if it's 0 */
			if (ret == 0)
				ret = -EIO;
		}
	}
	kfree(buf);

	return ret > 0 ? 0 : ret;
}

/**
 * @brief Read multiple bytes to a register
 *
 * @param sc Pointer to SC850x chip structure
 * @param reg Register address to write to
 * @param len Number of bytes to write (max I2C block size)
 * @param val Pointer to data buffer containing bytes to write
 *
 * @return 0 on successful operation,
 *         negative error code on failure:
 *         - -EIO: I2C transfer error
 *         - -ENXIO: I2C device not found
 *         - Other I2C SMBus error codes
 *
 * @note Uses I2C block write operation for efficient multi-byte transfer
 */
static int sc83107_i2c_read_bytes(struct sc83107_chip *chip, uint8_t reg, uint8_t len, uint8_t *val)
{
	struct i2c_client *i2c = chip->client;
	uint8_t data = reg;
	int ret = 0;
	int retry;
	int reset_ret;

	/* Skip I2C operations if system is suspended */
	if (atomic_read(&chip->suspended)) {
		chg_info("system suspended, skip I2C read, reg=0x%02x\n", reg);
		return -EAGAIN;
	}

	struct i2c_msg msg[2] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.buf = &data,
			.len = 1,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.buf = val,
			.len = len,
		},
	};

	/* First round: retry SC83107_I2C_RETRY_MAX_COUNT times */
	for (retry = 0; retry < SC83107_I2C_RETRY_MAX_COUNT; retry++) {
		ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
		if (ret > 0) {
			break;
		}
		/* ret == 0 or ret < 0 means failure, continue retry */
	}

	/* If first round failed, handle according to I2C bus reset feature status */
	if (ret <= 0) {
		/* I2C bus reset feature enabled: reset bus and retry again (skip for reg 0x81) */
		if (reg != 0x81 && chip->i2c_bus_reset_enable) {
			chg_err("I2C read failed after %d retries, reg=0x%02x, ret=%d, attempting I2C bus reset\n",
				SC83107_I2C_RETRY_MAX_COUNT, reg, ret);
			reset_ret = sc83107_i2c_bus_reset(chip);
			if (reset_ret < 0)
				chg_err("I2C bus reset failed, rc=%d, but will still retry\n", reset_ret);
			/* Wait a small delay after reset before retry */
			msleep(10);

			/* Second round: retry SC83107_I2C_RETRY_MAX_COUNT times after reset */
			for (retry = 0; retry < SC83107_I2C_RETRY_MAX_COUNT; retry++) {
				ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
				if (ret > 0) {
					chg_info("I2C read succeeded after bus reset, reg=0x%02x\n", reg);
					break;
				}
				/* ret == 0 or ret < 0 means failure, continue retry */
			}
			/* If second round also failed, ret still contains the error code from i2c_transfer */
		}
		/* Original code path: I2C bus reset feature disabled or reg 0x81 */
		if (ret <= 0) {
			if (reg != 0x81 && chip->i2c_bus_reset_enable)
				chg_err("I2C read failed after %d retries + reset + %d retries, reg=0x%02x, ret=%d\n",
					SC83107_I2C_RETRY_MAX_COUNT, SC83107_I2C_RETRY_MAX_COUNT, reg, ret);
			else
				chg_err("I2C read failed after %d retries, reg=0x%02x, ret=%d\n",
					SC83107_I2C_RETRY_MAX_COUNT, reg, ret);
			/* Pull down INT GPIO when I2C retry fails, except for reg 0x81
			 * which is not readable before writing key
			 */
			if (reg != 0x81)
				sc83107_pull_down_int_gpio(chip);
			/* Set ret to error code if it's 0 */
			if (ret == 0)
				ret = -EIO;
		}
	} else {
		/* I2C read succeeded, check and restore GPIO if it was pulled down */
		sc83107_restore_int_gpio(chip);
	}

	return ret > 0 ? 0 : ret;
}

static int sc83107_i2c_write_byte(struct sc83107_chip *chip, uint8_t reg, uint8_t val)
{
	int ret;
	s32 err_info[2] = { 0 };
	ret = sc83107_i2c_write_bytes(chip, reg, 1, &val);

	/* Double check: only filter -EAGAIN if system is suspended */
	if (ret == -EAGAIN) {
		if (atomic_read(&chip->suspended)) {
			/* Suspend caused -EAGAIN, don't upload error */
			chg_info("I2C write skipped due to suspend, reg=0x%02x\n", reg);
			return ret;
		}
		/* -EAGAIN but not suspended, this is a real error, upload it */
		chg_err("I2C write returned -EAGAIN but not in suspend state, reg=0x%02x\n", reg);
		err_info[0] = reg;
		err_info[1] = ret;
		sc83107_upload_i2c_err_info(chip, false, err_info);
		return ret;
	}

	/* Upload other errors normally */
	if (ret < 0) {
		err_info[0] = reg;
		err_info[1] = ret;
		sc83107_upload_i2c_err_info(chip, false, err_info);
	}
	return ret;
}

static int sc83107_i2c_read_byte(struct sc83107_chip *chip, uint8_t reg, uint8_t *val)
{
	int ret;
	s32 err_info[2] = { 0 };
	ret = sc83107_i2c_read_bytes(chip, reg, 1, val);

	/* Double check: only filter -EAGAIN if system is suspended */
	if (ret == -EAGAIN) {
		if (atomic_read(&chip->suspended)) {
			/* Suspend caused -EAGAIN, don't upload error */
			chg_info("I2C read skipped due to suspend, reg=0x%02x\n", reg);
			return ret;
		}
		/* -EAGAIN but not suspended, this is a real error, upload it */
		chg_err("I2C read returned -EAGAIN but not in suspend state, reg=0x%02x\n", reg);
		err_info[0] = reg;
		err_info[1] = ret;
		sc83107_upload_i2c_err_info(chip, true, err_info);
		return ret;
	}

	/* Upload other errors normally */
	if (ret < 0) {
		err_info[0] = reg;
		err_info[1] = ret;
		sc83107_upload_i2c_err_info(chip, true, err_info);
	}
	return ret;
}

static int sc83107_field_read(struct sc83107_chip *chip,
			    enum sc83107_fields field_id, int *val)
{
	int ret;
	uint8_t reg_val = 0;
	uint8_t mask = GENMASK(sc83107_reg_fields[field_id].msb, sc83107_reg_fields[field_id].lsb);

	ret = sc83107_i2c_read_byte(chip, sc83107_reg_fields[field_id].reg, &reg_val);
	if (ret < 0) {
		chg_err("sc83107 read field %d fail: %d\n", field_id, ret);
		return ret;
	}

	reg_val &= mask;
	reg_val >>= sc83107_reg_fields[field_id].lsb;

	*val = reg_val;
	return ret;
}

static int sc83107_field_write(struct sc83107_chip *chip,
			     enum sc83107_fields field_id, int val)
{
	int ret;
	uint8_t reg_val = 0, tmp = 0;
	uint8_t mask = GENMASK(sc83107_reg_fields[field_id].msb, sc83107_reg_fields[field_id].lsb);

	ret = sc83107_i2c_read_byte(chip, sc83107_reg_fields[field_id].reg, &reg_val);
	if (ret < 0) {
		goto out;
	}

	tmp = reg_val & ~mask;
	val <<= sc83107_reg_fields[field_id].lsb;
	tmp |= val & mask;

	if (sc83107_reg_fields[field_id].force_write || tmp != reg_val) {
		ret = sc83107_i2c_write_byte(chip, sc83107_reg_fields[field_id].reg, tmp);
	}

out:
	if (ret < 0) {
		/* Only print error if not suspended (suspend returns -EAGAIN) */
		if (ret != -EAGAIN || !atomic_read(&chip->suspended))
			chg_err("sc83107 write field %d fail: %d\n", field_id, ret);
		/* Don't print log for suspend-induced -EAGAIN to avoid excessive logging */
	}
	return ret;
}

/********************* ops start *********************/
__maybe_unused
static int sc83107_set_key(struct sc83107_chip *chip)
{
	sc83107_i2c_write_byte(chip, 0x7F, 0x59);
	sc83107_i2c_write_byte(chip, 0x7F, 0x43);
	return sc83107_i2c_write_byte(chip, 0x7F, 0x47);
}

#define SC83107_BCL_CLAMP_VOL_100MV 0
#define SC83107_BCL_CLAMP_VOL_400MV 1
__maybe_unused
static int sc83107_bcl_clamp_vol_set(struct sc83107_chip *chip, uint32_t mv)
{
	int ret;
	uint8_t reg_val;

	ret = sc83107_i2c_read_byte(chip, 0x81, &reg_val);
	if (ret) {
		sc83107_set_key(chip);
	}

	ret = sc83107_i2c_read_byte(chip, 0x81, &reg_val);
	if (!ret)
		chg_info("after set key, reg 81 = 0x%02x\n", reg_val);

	if (mv >= 400) {
		reg_val = SC83107_BCL_CLAMP_VOL_400MV;
	} else {
		reg_val = SC83107_BCL_CLAMP_VOL_100MV;
	}

	sc83107_field_write(chip, F_BCL_CLAMP_UP, reg_val);
	sc83107_i2c_read_byte(chip, 0x81, &reg_val);
	chg_info("----> reg 81 = 0x%02x\n", reg_val);

	return sc83107_set_key(chip);
}

__maybe_unused
static int sc83107_reg_reset(struct sc83107_chip *chip)
{
	return sc83107_field_write(chip, F_RESET, 1);
}

#define SC83107_FREQ_1MHZ	0
#define SC83107_FREQ_700KHZ	1
__maybe_unused
static int sc83107_fsw_set(struct sc83107_chip *chip, int fsw)
{
	int reg_val = SC83107_FREQ_1MHZ;

	if (fsw) {
		reg_val = SC83107_FREQ_700KHZ;
	}

	return sc83107_field_write(chip, F_FSW_SET, reg_val);
}

__maybe_unused
static int sc83107_fsw_get(struct sc83107_chip *chip, int *fsw)
{
	int ret;
	int reg_val;

	ret = sc83107_field_read(chip, F_FSW_SET, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	*fsw = !!reg_val;

	return ret;
}

#define SC83107_OVP_STEP_SIZE	100
#define SC83107_OVP_OFFSET	4800
#define SC83107_OVP_MIN		SC83107_OVP_OFFSET
#define SC83107_OVP_MAX		5500
__maybe_unused
static int sc83107_ovp_set(struct sc83107_chip *chip, uint32_t mv)
{
	uint8_t reg_val;

	if (mv < SC83107_OVP_MIN || mv > SC83107_OVP_MAX) {
		chg_err("param error: %d\n", mv);
		return -EINVAL;
	}

	reg_val = (mv - SC83107_OVP_OFFSET) / SC83107_OVP_STEP_SIZE;

	return sc83107_field_write(chip, F_OVP_TH_CT, reg_val);
}

__maybe_unused
static int sc83107_ovp_get(struct sc83107_chip *chip, uint32_t *mv)
{
	int ret;
	int reg_val;

	if (mv == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_OVP_TH_CT, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	*mv = SC83107_OVP_OFFSET + (reg_val * SC83107_OVP_STEP_SIZE);

	return ret;
}

__maybe_unused
static int sc83107_otg_set(struct sc83107_chip *chip, bool enable)
{
	int reg_val = enable;

	return sc83107_field_write(chip, F_OTG_MODE, reg_val);
}

__maybe_unused
static int sc83107_otg_get(struct sc83107_chip *chip, bool *enable)
{
	int ret;
	int reg_val;

	ret = sc83107_field_read(chip, F_OTG_MODE, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	*enable = !!reg_val;

	return ret;
}

#define SC83107_AUTO_FPWM	0
#define SC83107_FORCE_FPWM	1
__maybe_unused
static int sc83107_force_fpwm_set(struct sc83107_chip *chip, bool enable)
{
	int reg_val = SC83107_AUTO_FPWM;

	if (enable) {
		reg_val = SC83107_FORCE_FPWM;
	}

	return sc83107_field_write(chip, F_FPWM_CFG, reg_val);
}

__maybe_unused
static int sc83107_force_fpwm_get(struct sc83107_chip *chip, bool *enable)
{
	int ret;
	int reg_val;

	ret = sc83107_field_read(chip, F_FPWM_CFG, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	*enable = !!reg_val;

	return ret;
}

__maybe_unused
static int sc83107_mode_set(struct sc83107_chip *chip, uint8_t mode)
{
	int reg_val = SC83107_MODE_AUTO_HYBRID_BP;
	int ret = 0;
	int pgood = 0;
	int force_bp = 0;
	int retry_count = 0;
	bool mode_set_success = false;
	bool fpwm_enable = false;

	if (mode) {
		reg_val = SC83107_MODE_FORCE_BP;
		ret = sc83107_field_write(chip, F_FORCE_BP, reg_val);
		if (ret < 0) {
			chg_err("write F_FORCE_BP fail, ret=%d\n", ret);
			return ret;
		}
		/* If setting to force bypass mode, no need to verify */
		return 0;
	} else {
		/* For auto mode, preserve the existing fpwm logic */
		sc83107_force_fpwm_get(chip, &fpwm_enable);
		chg_info("fpwm_enable = %d\n", fpwm_enable);
		if (fpwm_enable == false) {
			sc83107_force_fpwm_set(chip, 1);
			msleep(1);
			sc83107_field_write(chip, F_FORCE_BP, reg_val);
			msleep(1);
			sc83107_force_fpwm_set(chip, 0);
		}
		ret = sc83107_field_write(chip, F_FORCE_BP, reg_val);
		if (ret < 0) {
			chg_err("write F_FORCE_BP fail, ret=%d\n", ret);
			return ret;
		}
	}

	/* For auto mode, need to verify and retry if necessary */
	/* Wait at least 1ms after writing register */
	usleep_range(1000, 2000);

	/* Verify auto mode: check PGOOD (08h[0]) and FORCE_BP (01h[0]) */
	for (retry_count = 0; retry_count <= SC83107_MODE_SET_RETRY_MAX; retry_count++) {
		/* Read PGOOD register (08h[0]) */
		ret = sc83107_field_read(chip, F_PGOOD, &pgood);
		if (ret < 0) {
			chg_err("read F_PGOOD fail, ret=%d\n", ret);
			return ret;
		}

		/* Read FORCE_BP register (01h[0]) */
		ret = sc83107_field_read(chip, F_FORCE_BP, &force_bp);
		if (ret < 0) {
			chg_err("read F_FORCE_BP fail, ret=%d\n", ret);
			return ret;
		}

		chg_info("mode set verify: retry=%d, PGOOD=%d, FORCE_BP=%d\n",
			 retry_count, pgood, force_bp);

		/* Check if auto mode is set successfully:
		 * PGOOD (08h[0]) should be 1 and FORCE_BP (01h[0]) should be 0
		 */
		if (pgood == 1 && force_bp == 0) {
			mode_set_success = true;
			chg_info("auto mode set successfully\n");
			break;
		}

		/* If PGOOD is still 0, auto mode entry failed */
		if (pgood == 0) {
			if (retry_count < SC83107_MODE_SET_RETRY_MAX) {
				chg_err("auto mode entry failed, PGOOD=0, retry %d/%d\n",
					retry_count + 1, SC83107_MODE_SET_RETRY_MAX);
				/* Retry: write 0 to 01h[0] to enter auto mode again */
				ret = sc83107_field_write(chip, F_FORCE_BP, SC83107_MODE_AUTO_HYBRID_BP);
				if (ret < 0) {
					chg_err("retry write F_FORCE_BP fail, ret=%d\n", ret);
					return ret;
				}
				/* Wait at least 1ms before next read */
				usleep_range(1000, 2000);
			} else {
				chg_err("auto mode entry failed after %d retries, PGOOD still 0\n",
					SC83107_MODE_SET_RETRY_MAX);
			}
		} else {
			/* PGOOD is 1 but FORCE_BP is not 0, this is unexpected */
			chg_err("unexpected state: PGOOD=%d, FORCE_BP=%d\n", pgood, force_bp);
		}
	}

	/* If auto mode set failed after all retries, set to force bypass mode and upload track */
	if (!mode_set_success) {
		chg_err("auto mode entry failed, fallback to force bypass mode\n");

		/* Set to force bypass mode */
		ret = sc83107_field_write(chip, F_FORCE_BP, SC83107_MODE_FORCE_BP);
		if (ret < 0) {
			chg_err("set force bypass mode fail, ret=%d\n", ret);
			return ret;
		}
		/* Upload track data for abnormal auto mode entry failure */
		if (!chip->track_upload_pending) {
			chip->track_upload_pending = true;
			schedule_delayed_work(&chip->track_upload_work, 0);
			chg_err("scheduled track upload work for auto mode entry failure\n");
		}

		return -EIO;
	}

	return 0;
}

__maybe_unused
static int sc83107_mode_get(struct sc83107_chip *chip, uint8_t *mode)
{
	int ret;
	int reg_val;

	ret = sc83107_field_read(chip, F_FORCE_BP, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	*mode = !!reg_val;

	return ret;
}

#define SC83107_OVP_ENABLE	0
#define SC83107_OVP_DISABLE	1
__maybe_unused
static int sc83107_ovp_enable(struct sc83107_chip *chip, bool enable)
{
	int reg_val = SC83107_OVP_ENABLE;

	if (enable) {
		reg_val = SC83107_OVP_ENABLE;
	} else {
		reg_val = SC83107_OVP_DISABLE;
	}

	return sc83107_field_write(chip, F_VOUT_OVP_OFF, reg_val);
}

#define SC83107_HYBRID_VOL_STEP_SIZE	50
#define SC83107_HYBRID_VOL_OFFSET	2500
#define SC83107_HYBRID_VOL_MIN		SC83107_HYBRID_VOL_OFFSET
#define SC83107_HYBRID_VOL_MAX		4500
__maybe_unused
static int sc83107_hybrid_output_vol_set(struct sc83107_chip *chip, uint32_t mv)
{
	uint8_t reg_val;

	if (mv < SC83107_HYBRID_VOL_MIN || mv > SC83107_HYBRID_VOL_MAX) {
		chg_err("param error: %d\n", mv);
		return -EINVAL;
	}

	reg_val = (mv - SC83107_HYBRID_VOL_OFFSET) / SC83107_HYBRID_VOL_STEP_SIZE;

	/* Only print log when not suspended to avoid excessive logging during suspend/resume */
	if (!atomic_read(&chip->suspended))
		chg_info("set cv , reg = 0x%x, mv = %d\n", reg_val, mv);

	return sc83107_field_write(chip, F_VREG, reg_val);
}

__maybe_unused
static int sc83107_hybrid_output_vol_get(struct sc83107_chip *chip, uint32_t *mv)
{
	int ret;
	int reg_val;

	if (mv == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_VREG, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	*mv = SC83107_HYBRID_VOL_OFFSET + (reg_val * SC83107_HYBRID_VOL_STEP_SIZE);

	return ret;
}

#define SC83107_MODE_HOTDIE_ALARM_100     0
#define SC83107_MODE_HOTDIE_ALARM_120     1
__maybe_unused
static int sc83107_hotdie_alarm_temp_set(struct sc83107_chip *chip, uint8_t temp)
{
	int reg_val;

	if (temp < 120) {
		reg_val = SC83107_MODE_HOTDIE_ALARM_100;
	} else {
		reg_val = SC83107_MODE_HOTDIE_ALARM_120;
	}

	return sc83107_field_write(chip, F_THOT_DIE_CT, reg_val);
}

#define SC83107_INT_DEGLITCH_DISABLE	0
#define SC83107_INT_DEGLITCH_100MS	1
#define SC83107_INT_DEGLITCH_1000MS	2
#define SC83107_INT_DEGLITCH_10000MS	3
__maybe_unused
static int sc83107_int_deglitch_set(struct sc83107_chip *chip, uint32_t ms)
{
	int reg_val;

	if (ms >= 10000) {
		reg_val = SC83107_INT_DEGLITCH_10000MS;
	} else if (ms >= 1000) {
		reg_val = SC83107_INT_DEGLITCH_1000MS;
	} else if (ms >= 100) {
		reg_val = SC83107_INT_DEGLITCH_100MS;
	} else {
		reg_val = SC83107_INT_DEGLITCH_DISABLE;
	}

	return sc83107_field_write(chip, F_INT_DEGLITCH, reg_val);
}

#define SC83107_CURRENT_LIMIT_ENABLE	0
#define SC83107_CURRENT_LIMIT_DISENABLE	1
__maybe_unused
static int sc83107_current_limet_enable(struct sc83107_chip *chip, bool enable)
{
	int reg_val = SC83107_CURRENT_LIMIT_ENABLE;

	if (enable) {
		reg_val = SC83107_CURRENT_LIMIT_ENABLE;
	} else {
		reg_val = SC83107_CURRENT_LIMIT_DISENABLE;
	}

	return sc83107_field_write(chip, F_ILIM_OFF, reg_val);
}

#define SC83107_HYBRID_LIMIT_CUR_STEP_SIZE       2000
#define SC83107_HYBRID_LIMIT_CUR_OFFSET          6000
#define SC83107_HYBRID_LIMIT_CUR_MIN             SC83107_HYBRID_LIMIT_CUR_OFFSET
#define SC83107_HYBRID_LIMIT_CUR_MAX             16000
__maybe_unused
static int sc83107_hybrid_limit_cur_set(struct sc83107_chip *chip, uint32_t ma)
{
	uint8_t reg_val;

	if (ma < SC83107_HYBRID_LIMIT_CUR_MIN || ma > SC83107_HYBRID_LIMIT_CUR_MAX) {
		chg_err("param error: %d\n", ma);
		return -EINVAL;
	}

	reg_val = (ma - SC83107_HYBRID_LIMIT_CUR_OFFSET) / SC83107_HYBRID_LIMIT_CUR_STEP_SIZE;

	return sc83107_field_write(chip, F_ILIM, reg_val);
}

__maybe_unused
static int sc83107_hybrid_limit_cur_get(struct sc83107_chip *chip, uint32_t *ma)
{
	int ret;
	int reg_val;

	if (ma == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_ILIM, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	*ma = SC83107_HYBRID_LIMIT_CUR_OFFSET + (reg_val * SC83107_HYBRID_LIMIT_CUR_STEP_SIZE);

	return ret;
}

#define SC83107_BP2CV_THRESHOLED_0MV	0
#define SC83107_BP2CV_THRESHOLED_100MV	1
#define SC83107_BP2CV_THRESHOLED_200MV	2
#define SC83107_BP2CV_THRESHOLED_300MV	3
__maybe_unused
static int sc83107_bp2cv_vol_threshold_set(struct sc83107_chip *chip, uint32_t mv)
{
	uint8_t reg_val;

	if (mv >= 300) {
		reg_val = SC83107_BP2CV_THRESHOLED_300MV;
	} else if (mv >= 200) {
		reg_val = SC83107_BP2CV_THRESHOLED_200MV;
	} else if (mv >= 100) {
		reg_val = SC83107_BP2CV_THRESHOLED_100MV;
	} else {
		reg_val = SC83107_BP2CV_THRESHOLED_0MV;
	}

	return sc83107_field_write(chip, F_VTH_BP2CV_CT, reg_val);
}

#define SC83107_BP2CV_DEG_THRESHOLED_0US	0
#define SC83107_BP2CV_DEG_THRESHOLED_1US	1
#define SC83107_BP2CV_DEG_THRESHOLED_10US	2
#define SC83107_BP2CV_DEG_THRESHOLED_32US	3
#define SC83107_BP2CV_DEG_THRESHOLED_64US	4
#define SC83107_BP2CV_DEG_THRESHOLED_128US	5
#define SC83107_BP2CV_DEG_THRESHOLED_256US	6
#define SC83107_BP2CV_DEG_THRESHOLED_512US	7
__maybe_unused
static int sc83107_bp2cv_time_threshold_set(struct sc83107_chip *chip, uint32_t us)
{
	uint8_t reg_val;

	if (us >= 512) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_512US;
	} else if (us >= 256) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_256US;
	} else if (us >= 128) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_128US;
	} else if (us >= 64) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_64US;
	} else if (us >= 32) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_32US;
	} else if (us >= 10) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_10US;
	} else if (us >= 1) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_1US;
	} else {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_0US;
	}

	return sc83107_field_write(chip, F_VTH_BP2CV_DEG, reg_val);
}

#define SC83107_SSFM_ENABLE	1
#define SC83107_SSFM_DISABLE	0
__maybe_unused
static int sc83107_ssfm_enable(struct sc83107_chip *chip, bool enable)
{
	int reg_val;

	if (enable) {
		reg_val = SC83107_SSFM_ENABLE;
	} else {
		reg_val = SC83107_SSFM_DISABLE;
	}
	return sc83107_field_write(chip, F_SSFM, reg_val);
}

#define SC83107_CV2BP_THRESHOLED_100MV	0
#define SC83107_CV2BP_THRESHOLED_200MV	1
#define SC83107_CV2BP_THRESHOLED_300MV	2
#define SC83107_CV2BP_THRESHOLED_400MV	3
__maybe_unused
static int sc83107_cv2bp_vol_threshold_set(struct sc83107_chip *chip, uint32_t mv)
{
	uint8_t reg_val;

	if (mv >= 400) {
		reg_val = SC83107_CV2BP_THRESHOLED_400MV;
	} else if (mv >= 300) {
		reg_val = SC83107_CV2BP_THRESHOLED_300MV;
	} else if (mv >= 200) {
		reg_val = SC83107_CV2BP_THRESHOLED_200MV;
	} else {
		reg_val = SC83107_CV2BP_THRESHOLED_100MV;
	}

	return sc83107_field_write(chip, F_VTH_CV2BP_CT_HYS, reg_val);
}

#define SC83107_CV2BP_DEG_THRESHOLED_0US	0
#define SC83107_CV2BP_DEG_THRESHOLED_64US	1
#define SC83107_CV2BP_DEG_THRESHOLED_320US	2
#define SC83107_CV2BP_DEG_THRESHOLED_1100US	3
#define SC83107_CV2BP_DEG_THRESHOLED_20000US	4
#define SC83107_CV2BP_DEG_THRESHOLED_80000US	5
#define SC83107_CV2BP_DEG_THRESHOLED_320000US	6
#define SC83107_CV2BP_DEG_THRESHOLED_1100000US	7
__maybe_unused
static int sc83107_cv2bp_time_threshold_set(struct sc83107_chip *chip, uint32_t us)
{
	uint8_t reg_val;

	if (us >= 512) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_512US;
	} else if (us >= 256) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_256US;
	} else if (us >= 128) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_128US;
	} else if (us >= 64) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_64US;
	} else if (us >= 32) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_32US;
	} else if (us >= 10) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_10US;
	} else if (us >= 1) {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_1US;
	} else {
		reg_val = SC83107_BP2CV_DEG_THRESHOLED_0US;
	}

	return sc83107_field_write(chip, F_VTH_CV2BP_DEG, reg_val);
}

#define SC83107_TON_MAX_LIM_STEP_SIZE	50
#define SC83107_TON_MAX_LIM_CUR_OFFSET	200
#define SC83107_TON_MAX_LIM_CUR_MIN	SC83107_TON_MAX_LIM_CUR_OFFSET
#define SC83107_TON_MAX_LIM_CUR_MAX	800
__maybe_unused
static int sc83107_ton_max_limit_time_set(struct sc83107_chip *chip, uint32_t ns)
{
	uint8_t reg_val;

	if (ns < SC83107_TON_MAX_LIM_CUR_MIN || ns > SC83107_TON_MAX_LIM_CUR_MAX) {
		chg_err("param error: %d\n", ns);
		return -EINVAL;
	}

	reg_val = (ns - SC83107_TON_MAX_LIM_CUR_OFFSET) / SC83107_TON_MAX_LIM_STEP_SIZE;

	return sc83107_field_write(chip, F_TON_MAX_LIM, reg_val);
}

__maybe_unused
static int sc83107_ton_max_limit_time_get(struct sc83107_chip *chip, uint32_t *ns)
{
	int ret;
	int reg_val;

	if (ns == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_TON_MAX_LIM, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	*ns = SC83107_TON_MAX_LIM_CUR_OFFSET + (reg_val * SC83107_TON_MAX_LIM_STEP_SIZE);

	return ret;
}

#define SC83107_VBAT_SNS_ENABLE	0
#define SC83107_VBAT_SNS_DISABLE	1
__maybe_unused
static int sc83107_vbat_sns_enable(struct sc83107_chip *chip, bool enable)
{
	int reg_val;

	if (enable) {
		reg_val = SC83107_VBAT_SNS_ENABLE;
	} else {
		reg_val = SC83107_VBAT_SNS_DISABLE;
	}
	return sc83107_field_write(chip, F_VBAT_SNS_DIS, reg_val);
}

#define SC83107_WDT_DISABLE		0
#define SC83107_WDT_TIME_500MS		1
#define SC83107_WDT_TIME_1000MS		2
#define SC83107_WDT_TIME_2000MS		3
#define SC83107_WDT_TIME_20000MS	4
#define SC83107_WDT_TIME_40000MS	5
#define SC83107_WDT_TIME_80000MS	6
#define SC83107_WDT_TIME_160000MS	7
__maybe_unused
static int sc83107_wdt_set(struct sc83107_chip *chip, uint32_t ms)
{
	uint8_t reg_val;

	if (ms >= 160000) {
		reg_val = SC83107_WDT_TIME_160000MS;
	} else if (ms >= 80000) {
		reg_val = SC83107_WDT_TIME_80000MS;
	} else if (ms >= 40000) {
		reg_val = SC83107_WDT_TIME_40000MS;
	} else if (ms >= 20000) {
		reg_val = SC83107_WDT_TIME_20000MS;
	} else if (ms >= 2000) {
		reg_val = SC83107_WDT_TIME_2000MS;
	} else if (ms >= 1000) {
		reg_val = SC83107_WDT_TIME_1000MS;
	} else if (ms >= 500) {
		reg_val = SC83107_WDT_TIME_500MS;
	} else {
		reg_val = SC83107_WDT_DISABLE;
	}

	return sc83107_field_write(chip, F_WD_TIMEOUT, reg_val);
}

#define SC83107_LV2_COMP_VOL_1_8V	0
#define SC83107_LV2_COMP_VOL_2_0V	3
#define SC83107_LV2_COMP_VOL_2_2V	4
#define SC83107_LV2_COMP_VOL_2_4V	5
__maybe_unused
static int sc83107_lv2_comp_input_vol_threshold_set(struct sc83107_chip *chip, uint32_t mv)
{
	uint8_t reg_val;

	if (mv >= 2400) {
		reg_val = SC83107_LV2_COMP_VOL_2_4V;
	} else if (mv >= 2200) {
		reg_val = SC83107_LV2_COMP_VOL_2_2V;
	} else if (mv >= 2000) {
		reg_val = SC83107_LV2_COMP_VOL_2_0V;
	} else if (mv >= 1800) {
		reg_val = SC83107_LV2_COMP_VOL_1_8V;
	} else {
		reg_val = SC83107_LV2_COMP_VOL_1_8V;
	}

	return sc83107_field_write(chip, F_LV2_COMP_CT, reg_val);
}

#define SC83107_LV2_COMP_HYS_VOL_100MV	0
#define SC83107_LV2_COMP_HYS_VOL_300MV	1
__maybe_unused
static int sc83107_lv2_comp_vol_hysteresis_set(struct sc83107_chip *chip, uint32_t mv)
{
	uint8_t reg_val;

	if (mv >= 300) {
		reg_val = SC83107_LV2_COMP_HYS_VOL_300MV;
	} else {
		reg_val = SC83107_LV2_COMP_HYS_VOL_100MV;
	}

	return sc83107_field_write(chip, F_LV2_COMP_HYS_CT, reg_val);
}

#define SC83107_LV0_COMP_CT_2_4V	0
#define SC83107_LV0_COMP_CT_2_6V	1
#define SC83107_LV0_COMP_CT_2_8V	2
#define SC83107_LV0_COMP_CT_3_0V	3
__maybe_unused
static int sc83107_lv0_comp_falling_threshold_set(struct sc83107_chip *chip, uint32_t mv)
{
	uint8_t reg_val;

	if (mv >= 3000) {
		reg_val = SC83107_LV0_COMP_CT_3_0V;
	} else if (mv >= 2800) {
		reg_val = SC83107_LV0_COMP_CT_2_8V;
	} else if (mv >= 2600) {
		reg_val = SC83107_LV0_COMP_CT_2_6V;
	} else {
		reg_val = SC83107_LV0_COMP_CT_2_4V;
	}

	return sc83107_field_write(chip, F_LV0_COMP_CT, reg_val);
}

#define SC83107_LV1_COMP_CT_2_2V	0
#define SC83107_LV1_COMP_CT_2_4V	1
#define SC83107_LV1_COMP_CT_2_6V	2
#define SC83107_LV1_COMP_CT_2_8V	3
__maybe_unused
static int sc83107_lv1_comp_falling_threshold_set(struct sc83107_chip *chip, uint32_t mv)
{
	uint8_t reg_val;

	if (mv >= 2800) {
		reg_val = SC83107_LV1_COMP_CT_2_8V;
	} else if (mv >= 2600) {
		reg_val = SC83107_LV1_COMP_CT_2_6V;
	} else if (mv >= 2400) {
		reg_val = SC83107_LV1_COMP_CT_2_4V;
	} else {
		reg_val = SC83107_LV1_COMP_CT_2_2V;
	}

	return sc83107_field_write(chip, F_LV1_COMP_CT, reg_val);
}

#define TONMIN1_DELAY_TIME_20NS 20
#define TONMIN1_DELAY_TIME_13NS 13
#define TONMIN1_DELAY_TIME_8NS 8
#define TONMIN1_DELAY_TIME_3NS 3
__maybe_unused
static int sc83107_tonmin1_set(struct sc83107_chip *chip, uint32_t ns)
{
	int ret = 0;
	int reg_val = 0;

	if (ns >= TONMIN1_DELAY_TIME_20NS) {
		reg_val = 0;
	} else if (ns >= TONMIN1_DELAY_TIME_13NS) {
		reg_val = 1;
	} else if (ns >= TONMIN1_DELAY_TIME_8NS) {
		reg_val = 2;
	} else {
		reg_val = 3;
	}

	ret = sc83107_field_write(chip, F_TON_MIN1, reg_val);
	if (ret) {
		chg_err("F_TON_MIN1 write fail\n");
		return ret;
	}

	return ret;
}

#define TONMIN2_DELAY_TIME_27NS 27
#define TONMIN2_DELAY_TIME_20NS 20
#define TONMIN2_DELAY_TIME_12NS 12
#define TONMIN2_DELAY_TIME_3NS 3
__maybe_unused
static int sc83107_tonmin2_set(struct sc83107_chip *chip, uint32_t ns)
{
	int ret = 0;
	int reg_val = 0;

	if (ns >= TONMIN2_DELAY_TIME_27NS) {
		reg_val = 1;
	} else if (ns >= TONMIN2_DELAY_TIME_20NS) {
		reg_val = 0;
	} else if (ns >= TONMIN2_DELAY_TIME_12NS) {
		reg_val = 3;
	} else {
		reg_val = 2;
	}

	ret = sc83107_field_write(chip, F_TON_MIN2, reg_val);
	if (ret) {
		chg_err("F_TON_MIN2 write fail\n");
		return ret;
	}

	return ret;
}

__maybe_unused
static int sc83107_hot_die_temp_is_alarm(struct sc83107_chip *chip, bool *status)
{
	int ret;
	int reg_val;

	if (status == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_HOTDIE, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	*status = !!reg_val;

	return ret;
}

__maybe_unused
static int sc83107_hybrid_boost_is_ocp(struct sc83107_chip *chip, bool *status)
{
	int ret;
	int reg_val;

	if (status == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_BST_OCP, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	*status = !!reg_val;

	return ret;
}

#define SC83107_DCDC_MODE_PWM	0
#define SC83107_DCDC_MODE_PFM	1
__maybe_unused
static int sc83107_dcdc_mode_get(struct sc83107_chip *chip, bool *mode)
{
	int ret;
	int reg_val;

	if (mode == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_DCDCMODE, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	if (reg_val == 0) {
		*mode = SC83107_DCDC_MODE_PWM;
	} else {
		*mode = SC83107_DCDC_MODE_PFM;
	}

	return ret;
}

#define SC83107_DCDC_MODE_DISABLE	0
#define SC83107_DCDC_MODE_ENABLE	1
__maybe_unused
static int sc83107_dcdc_mode_is_enable(struct sc83107_chip *chip, bool *enable)
{
	int ret;
	int reg_val;

	if (enable == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_OPMODE_BOOST, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	if (reg_val == 0) {
		*enable = SC83107_DCDC_MODE_DISABLE;
	} else {
		*enable = SC83107_DCDC_MODE_ENABLE;
	}

	return ret;
}

#define SC83107_BYPASS_MODE_DISABLE	0
#define SC83107_BYPASS_MODE_ENABLE	1
__maybe_unused
static int sc83107_bypass_mode_is_enable(struct sc83107_chip *chip, bool *enable)
{
	int ret;
	int reg_val;

	if (enable == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_OPMODE_BOOST, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	if (reg_val == 0) {
		*enable = SC83107_DCDC_MODE_DISABLE;
	} else {
		*enable = SC83107_DCDC_MODE_ENABLE;
	}

	return ret;
}

#define SC83107_VAC_GOOD	1
#define SC83107_VAC_NOT_GOOD	0
__maybe_unused
static int sc83107_vac_ok_status_get(struct sc83107_chip *chip, bool *status)
{
	int ret;
	int reg_val;

	if (status == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_VAC_OK, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	if (reg_val == 0) {
		*status = SC83107_VAC_NOT_GOOD;
	} else {
		*status = SC83107_VAC_GOOD;
	}

	return ret;
}

#define SC83107_VBAT_GOOD	1
#define SC83107_VBAT_NOT_GOOD	0
__maybe_unused
static int sc83107_vbat_ok_status_get(struct sc83107_chip *chip, bool *status)
{
	int ret;
	int reg_val;

	if (status == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_VBAT_OK, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	if (reg_val == 0) {
		*status = SC83107_VBAT_NOT_GOOD;
	} else {
		*status = SC83107_VBAT_GOOD;
	}

	return ret;
}

#define SC83107_POWER_GOOD	1
#define SC83107_POWER_NOT_GOOD	0
__maybe_unused
static int sc83107_power_good_status_get(struct sc83107_chip *chip, bool *status)
{
	int ret;
	int reg_val;

	if (status == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}
	ret = sc83107_field_read(chip, F_PGOOD, &reg_val);
	if (ret) {
		chg_err("field read fail\n");
		return ret;
	}
	if (reg_val == 0) {
		*status = SC83107_POWER_NOT_GOOD;
	} else {
		*status = SC83107_POWER_GOOD;
	}

	return ret;
}

static int sc83107_track_get_local_time_s(void)
{
	int local_time_s;

	local_time_s = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	return local_time_s;
}

static int sc83107_track_upload_cp_err_info(struct sc83107_chip *chip, int err_flag)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	char temp_str[REASON_LENGTH_MAX] = {0};
	struct oplus_mms *err_topic;
	struct oplus_mms *comm_topic = NULL;
	struct mms_msg *msg = NULL;
	int rc = 0;
	int ui_soc = -1;
	union mms_msg_data data_soc = { 0 };

	if (NULL == chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -EINVAL;
	}

	curr_time = sc83107_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_UPLOAD_COUNT_MAX) {
		chg_info("cp_err_uploading upload_count = %d > max %d, should return\n",
			 upload_count, TRACK_UPLOAD_COUNT_MAX);
		return 0;
	}

	upload_count++;
	pre_upload_time = sc83107_track_get_local_time_s();

	index += scnprintf(&(temp_str[index]), REASON_LENGTH_MAX - index, "$$device_id@@%s", "sc83107");
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index, "$$err_scene@@sc83107_chip_work_err");

	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$err_reason@@%02x", err_flag);
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$err_position@@%s", "main");

	/* Get UI SOC for track */
	comm_topic = oplus_mms_get_by_name("common");
	if (comm_topic) {
		if (oplus_mms_get_item_data(comm_topic, COMM_ITEM_UI_SOC, &data_soc, false) == 0) {
			ui_soc = data_soc.intval;
			index += scnprintf(&(temp_str[index]),
				REASON_LENGTH_MAX - index,
				"$$ui_soc@@%d", ui_soc);
		}
	}

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
		ERR_ITEM_ERR_PHY_CP_INFO, temp_str);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -EINVAL;
	}
	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return 0;
}

static int sc83107_upload_dischg_boost_err_flag_track(struct sc83107_chip *chip,
						      unsigned int err_flag,
						      u8 reg08_val, u8 reg09_val, u8 reg0a_val)
{
	int index = 0;
	int curr_time;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	char temp_str[REASON_LENGTH_MAX] = {0};
	struct oplus_mms *err_topic;
	struct mms_msg *msg = NULL;
	int rc = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	curr_time = sc83107_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count > TRACK_UPLOAD_COUNT_MAX) {
		chg_info("dischg_boost_err_flag upload_count = %d > max %d, should return\n",
			 upload_count, TRACK_UPLOAD_COUNT_MAX);
		return 0;
	}

	upload_count++;
	pre_upload_time = sc83107_track_get_local_time_s();

	index += scnprintf(&(temp_str[index]), REASON_LENGTH_MAX - index, "$$device_id@@%s", "sc83107");
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index, "$$err_scene@@sc83107_dischg_boost_err");
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$err_reason@@0x%08X", err_flag);
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$reg08@@0x%02X", reg08_val);
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$reg09@@0x%02X", reg09_val);
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$reg0a@@0x%02X", reg0a_val);
	index += scnprintf(&(temp_str[index]),
		REASON_LENGTH_MAX - index,
		"$$err_position@@%s", "main");

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
		ERR_ITEM_ERR_PHY_CP_INFO, temp_str);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return 0;
}

__maybe_unused
static int sc83107_flag_get(struct sc83107_chip *chip, enum sc83107_flag_type flag_type, bool *flag)
{
	int ret;
	uint8_t reg_val;
	uint8_t reg;
	uint8_t bit;

	if (flag == NULL) {
		chg_err("param null\n");
		return -EINVAL;
	}

	if ((flag_type > SC83107_FLAG_MAX) || (flag_type == SC83107_RESERVED0_FLAG) ||\
		(flag_type == SC83107_RESERVED1_FLAG)) {
		chg_err("param error mask type=%d\n", flag_type);
		return -EINVAL;
	}

	if (flag_type <= SC83107_HOTDIE_FLAG) {
		reg = 0x09;
		bit = flag_type;
	} else {
		reg = 0x0A;
		bit = flag_type - 8;
	}

	ret = sc83107_i2c_read_byte(chip, reg, &reg_val);
	if (ret < 0) {
		chg_err("read fail\n");
		return ret;
	}

	*flag = (reg_val >> bit) & 0x01;
	return ret;
}

__maybe_unused
static int sc83107_mask_set(struct sc83107_chip *chip, enum sc83107_mask_type mask_type, bool mask)
{
	int ret;
	uint8_t reg_val;
	uint8_t reg;
	uint8_t bit;

	if ((mask_type > SC83107_MASK_MAX) || (mask_type == SC83107_MASK_RESERVED0) ||\
		(mask_type == SC83107_MASK_RESERVED1) || (mask_type == SC83107_MASK_RESERVED2)) {
		chg_err("param error mask type=%d\n", mask_type);
		return -EINVAL;
	}

	if (mask_type <= SC83107_MASK_HOTDIE) {
		reg = 0x0B;
		bit = mask_type;
	} else {
		reg = 0x0C;
		bit = mask_type - 8;
	}

	ret = sc83107_i2c_read_byte(chip, reg, &reg_val);
	if (ret < 0) {
		chg_err("read fail\n");
		return ret;
	}
	reg_val = reg_val & ~(1 << bit);
	reg_val |= (mask << bit);
	ret = sc83107_i2c_write_byte(chip, reg, reg_val);
	if (ret < 0) {
		chg_err("write fail\n");
		return ret;
	}

	return ret;
}

static int sc83107_check_register_and_upload_track(struct sc83107_chip *chip,
						     uint8_t reg09_val, uint8_t reg0a_val)
{
	int ret = 0;
	unsigned int err_flag = 0;
	uint8_t local_reg09_val = reg09_val;
	uint8_t local_reg0a_val = reg0a_val;

	if (!chip)
		return -EINVAL;

	/* If flag register values are not provided (0xFF means not provided),
	 * read them from the chip
	 */
	if (reg09_val == 0xFF) {
		ret = sc83107_i2c_read_byte(chip, 0x09, &local_reg09_val);
		if (ret < 0) {
			chg_err("Failed to read register 0x09: %d\n", ret);
			return ret;
		}
	}

	if (reg0a_val == 0xFF) {
		ret = sc83107_i2c_read_byte(chip, 0x0A, &local_reg0a_val);
		if (ret < 0) {
			chg_err("Failed to read register 0x0A: %d\n", ret);
			return ret;
		}
	}

	/* Use the flag register values (either provided or read) */

	chg_info("%s: reg09_val=0x%02x, reg0a_val=0x%02x\n", __func__, local_reg09_val, local_reg0a_val);

	/* Record error bit to err_flag */
	if (local_reg09_val & SC83107_POR_FLAG_BIT) {
		err_flag |= BIT(SC83107_POR_FLAG);
		chg_err("detected POR_FLAG\n");
	}
	if (local_reg09_val & SC83107_VBAT_FALLING_FLAG_BIT) {
		err_flag |= BIT(SC83107_VBAT_FALLING_FLAG);
		chg_err("detected VBAT_FALLING_FLAG\n");
	}
	if (local_reg09_val & SC83107_BST_OCP_FLAG_BIT) {
		err_flag |= BIT(SC83107_BST_OCP_FLAG);
		chg_err("detected BST_OCP_FLAG\n");
	}
	if (local_reg09_val & SC83107_HOTDIE_FLAG_BIT) {
		err_flag |= BIT(SC83107_HOTDIE_FLAG);
		chg_err("detected HOTDIE_FLAG\n");
	}
	if (local_reg0a_val & SC83107_PIN_DIAG_FAIL_FLAG_BIT) {
		err_flag |= BIT(SC83107_PIN_DIAG_FAIL_FLAG);
		chg_err("detected PIN_DIAG_FAIL_FLAG\n");
	}
	if (local_reg0a_val & SC83107_Q3_OR_Q6_OCP_FLAG_BIT) {
		err_flag |= BIT(SC83107_Q3Q6_OCP_FLAG);
		chg_err("detected Q3Q6_OCP_FLAG\n");
	}
	if (local_reg0a_val & SC83107_VOUT_UVP_FLAG_BIT) {
		err_flag |= BIT(SC83107_VOUT_UVP_FLAG);
		chg_err("detected VOUT_UVP_FLAG\n");
	}
	if (local_reg0a_val & SC83107_BOOST_VOUT_OVP_FLAG_BIT) {
		err_flag |= BIT(SC83107_BOOST_VOUT_OVP_FLAG);
		chg_err("detected SC83107_BOOST_VOUT_OVP_FLAG\n");
	}
	if (local_reg0a_val & SC83107_BPASS_VOUT_OVP_FLAG_BIT) {
		err_flag |= BIT(SC83107_BPASS_VOUT_OVP_FLAG);
		chg_err("detected BPASS_VOUT_OVP_FLAG\n");
	}
	if (local_reg0a_val & SC83107_TSD_FLAG_BIT) {
		err_flag |= BIT(SC83107_TSD_FLAG);
		chg_err("detected TSD_FLAG\n");
	}

	chg_info("%s: err_flag=0x%x\n", __func__, err_flag);
	if (err_flag != 0) {
		sc83107_track_upload_cp_err_info(chip, err_flag);
	}
	return 0;
}

static int sc83107_dischg_boost_err_flag_obtain_mutual_notifier_call(
	struct notifier_block *nb, unsigned long param, void *v)
{
	struct sc83107_chip *chip;
	struct oplus_chg_mutual_notifier *notifier;
	unsigned int err_flag = 0;
	u8 reg08_val = 0, reg09_val = 0, reg0a_val = 0;
	char *str, *token;

	notifier = container_of(nb, struct oplus_chg_mutual_notifier, nb);
	chip = container_of(notifier, struct sc83107_chip, dischg_boost_err_flag_mutual);

	if (mutual_info_to_cmd(param) != CMD_DISCHG_BOOST_ERR_OBTAIN) {
		/* This is normal - all registered notifiers are called, only matching ones process */
		return NOTIFY_OK;
	}

	if (mutual_info_to_data_size(param) != sizeof(chip->dischg_boost_err_flag_data)) {
		chg_err("data_len is not ok, datas is invalid\n");
		return NOTIFY_DONE;
	}

	if (v)
		memmove(chip->dischg_boost_err_flag_data, v, sizeof(chip->dischg_boost_err_flag_data));

	chip->dischg_boost_err_flag_data[sizeof(chip->dischg_boost_err_flag_data) - 1] = '\0';
	chg_info("dischg_boost_err_flag_data:%s\n", chip->dischg_boost_err_flag_data);

	/* Parse data format: "dischg_boost_err,0x%08X,0x%02X,0x%02X,0x%02X" */
	/* Format: err_flag,reg08,reg09,reg0a */
	str = chip->dischg_boost_err_flag_data;
	if (!strstr(str, "dischg_boost_err")) {
		chg_info("no dischg_boost_err tag found\n");
		return NOTIFY_OK;
	}

	/* Skip "dischg_boost_err," */
	token = strstr(str, "dischg_boost_err,");
	if (!token) {
		chg_err("invalid format\n");
		return NOTIFY_OK;
	}
	token += strlen("dischg_boost_err,");

	/* Parse err_flag */
	if (sscanf(token, "0x%x", &err_flag) != 1) {
		chg_err("failed to parse err_flag\n");
		return NOTIFY_OK;
	}

	/* Find next comma */
	token = strchr(token, ',');
	if (!token) {
		chg_err("invalid format: no reg08\n");
		return NOTIFY_OK;
	}
	token++;

	/* Parse reg08 */
	if (sscanf(token, "0x%hhx", &reg08_val) != 1) {
		chg_err("failed to parse reg08\n");
		return NOTIFY_OK;
	}

	/* Find next comma */
	token = strchr(token, ',');
	if (!token) {
		chg_err("invalid format: no reg09\n");
		return NOTIFY_OK;
	}
	token++;

	/* Parse reg09 */
	if (sscanf(token, "0x%hhx", &reg09_val) != 1) {
		chg_err("failed to parse reg09\n");
		return NOTIFY_OK;
	}

	/* Find next comma */
	token = strchr(token, ',');
	if (!token) {
		chg_err("invalid format: no reg0a\n");
		return NOTIFY_OK;
	}
	token++;

	/* Parse reg0a */
	if (sscanf(token, "0x%hhx", &reg0a_val) != 1) {
		chg_err("failed to parse reg0a\n");
		return NOTIFY_OK;
	}

	chg_info("parsed: err_flag=0x%08X, reg08=0x%02X, reg09=0x%02X, reg0a=0x%02X\n",
		 err_flag, reg08_val, reg09_val, reg0a_val);

	/* If err_flag is not zero, schedule work to upload track in process context
	 * Note: We cannot call sc83107_upload_dischg_boost_err_flag_track directly here because
	 * this notifier callback runs in atomic context and sc83107_upload_dischg_boost_err_flag_track
	 * calls oplus_mms_alloc_str_msg which uses GFP_KERNEL and may sleep.
	 */
	if (err_flag != 0) {
		chg_err("detected error flag from partition: 0x%08X, reg08=0x%02X, reg09=0x%02X, reg0a=0x%02X, schedule work to upload\n",
			err_flag, reg08_val, reg09_val, reg0a_val);
		chip->dischg_boost_err_flag = err_flag;
		chip->dischg_boost_err_reg08_val = reg08_val;
		chip->dischg_boost_err_reg09_val = reg09_val;
		chip->dischg_boost_err_reg0a_val = reg0a_val;
		schedule_work(&chip->dischg_boost_err_flag_upload_work);
	}

	return NOTIFY_OK;
}

static int sc83107_dischg_boost_err_flag_mutual_notify_reg(struct sc83107_chip *chip)
{
	int rc = 0;

	chip->dischg_boost_err_flag_mutual.name = "dischg_boost_err_obtain";
	chip->dischg_boost_err_flag_mutual.cmd = CMD_DISCHG_BOOST_ERR_OBTAIN;
	chip->dischg_boost_err_flag_mutual.nb.notifier_call = sc83107_dischg_boost_err_flag_obtain_mutual_notifier_call;
	rc = oplus_chg_reg_mutual_notifier(&chip->dischg_boost_err_flag_mutual);
	if (rc < 0) {
		chg_err("register dischg boost err flag obtain mutual event notifier error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

#define DISCHG_BOOST_ERR_FLAG_OBTAIN_DELAY_MS	2000
#define DISCHG_BOOST_ERR_FLAG_OBTAIN_RETRY_MAX	3

static void sc83107_dischg_boost_err_flag_upload_work_func(struct work_struct *work)
{
	struct sc83107_chip *chip = container_of(work, struct sc83107_chip,
						  dischg_boost_err_flag_upload_work);

	if (!chip) {
		chg_err("chip is NULL\n");
		return;
	}

	/* Upload error flag in process context (safe to call functions that may sleep) */
	if (chip->dischg_boost_err_flag != 0) {
		chg_info("uploading dischg boost error flag 0x%08X from mutual notifier in process context, reg08=0x%02X, reg09=0x%02X, reg0a=0x%02X\n",
			 chip->dischg_boost_err_flag, chip->dischg_boost_err_reg08_val,
			 chip->dischg_boost_err_reg09_val, chip->dischg_boost_err_reg0a_val);
		sc83107_upload_dischg_boost_err_flag_track(chip, chip->dischg_boost_err_flag,
							   chip->dischg_boost_err_reg08_val,
							   chip->dischg_boost_err_reg09_val,
							   chip->dischg_boost_err_reg0a_val);
		/* Clear after upload */
		chip->dischg_boost_err_flag = 0;
		chip->dischg_boost_err_reg08_val = 0;
		chip->dischg_boost_err_reg09_val = 0;
		chip->dischg_boost_err_reg0a_val = 0;
	}
}

static void sc83107_get_dischg_boost_err_flag_work_func(struct work_struct *work)
{
	struct sc83107_chip *chip = container_of(work, struct sc83107_chip,
						  get_dischg_boost_err_flag_work.work);
	int mutual_rc;
	static int try_count = DISCHG_BOOST_ERR_FLAG_OBTAIN_RETRY_MAX;

	if (!chip) {
		chg_err("chip is NULL\n");
		return;
	}

	chg_info("get_dischg_boost_err_flag_work_func: requesting err flag from partition, try_count=%d\n", try_count);

	/* Request AIDL layer to read partition and return data via mutual */
	mutual_rc = oplus_chg_set_mutual_cmd(CMD_DISCHG_BOOST_ERR_OBTAIN, 0, NULL);
	if (mutual_rc != CMD_ACK_OK && try_count--) {
		/* Retry if AIDL service not ready yet */
		chg_info("AIDL service not ready yet, retry after 2s, remaining=%d\n", try_count);
		schedule_delayed_work(&chip->get_dischg_boost_err_flag_work, msecs_to_jiffies(2000));
		return;
	}

	/* Reset try_count for next time */
	try_count = DISCHG_BOOST_ERR_FLAG_OBTAIN_RETRY_MAX;

	/* Data will be received in notifier callback */
	if (mutual_rc == CMD_ACK_OK)
		chg_info("requested dischg boost err flag from partition successfully\n");
	else
		chg_err("failed to get dischg boost err flag from partition, rc=%d\n", mutual_rc);
}

static void sc83107_get_dischg_boost_err_flag_from_partition(struct sc83107_chip *chip)
{
	static bool update = false;

	if (!chip)
		return;

	if (!update) {
		update = true;
		chg_info("schedule get_dischg_boost_err_flag_work, delay=%d ms\n", DISCHG_BOOST_ERR_FLAG_OBTAIN_DELAY_MS);
		schedule_delayed_work(&chip->get_dischg_boost_err_flag_work,
				     msecs_to_jiffies(DISCHG_BOOST_ERR_FLAG_OBTAIN_DELAY_MS));
	} else {
		chg_info("get_dischg_boost_err_flag_from_partition already called, skip\n");
	}
}

static void sc83107_upload_all_registers(struct sc83107_chip *chip,
					   uint8_t reg09_val, uint8_t reg0a_val,
					   const char *trigger_source)
{
	int ret = 0;
	uint8_t i = 0;
	uint8_t data[SC83107_REGMAX + 1] = {0};
	char *buf = NULL;
	size_t index = 0;
	s32 err_info[2] = { 0 };
	int ui_soc = -1;
	struct oplus_mms *comm_topic = NULL;
	union mms_msg_data data_soc = { 0 };

	if (!chip)
		return;

	/* Read all registers from 0x00 to 0x0F */
	ret = sc83107_i2c_read_bytes(chip, 0x00, SC83107_REGMAX + 1, data);
	if (ret < 0) {
		/* Retry once if first read fails */
		ret = sc83107_i2c_read_bytes(chip, 0x00, SC83107_REGMAX + 1, data);
		if (ret < 0) {
			chg_err("Failed to read all registers [0x00-0x%02x]: %d\n",
				   SC83107_REGMAX, ret);
			err_info[0] = SC83107_REGMAX + 1;
			err_info[1] = ret;
			sc83107_upload_i2c_err_info(chip, true, err_info);
			return;
		}
	}

	/* Restore flag register values that were saved before reading all registers
	 * This ensures the uploaded register dump contains the original flag values
	 * before they were cleared by reading
	 * Note: For charger plug/unplug events, reg09_val and reg0a_val may be 0xFF
	 * (not provided), in which case we use the values read from the chip
	 */
	if (reg09_val != 0xFF)
		data[0x09] = reg09_val;
	if (reg0a_val != 0xFF)
		data[0x0A] = reg0a_val;

	/* Allocate buffer for register dump string */
	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL) {
		chg_err("Failed to allocate buffer for register dump\n");
		return;
	}

	/* Format register values with trigger source identifier */
	if (trigger_source && trigger_source[0] != '\0') {
		index += scnprintf(buf + index, ERR_MSG_BUF - index,
				   "$$trigger_source@@%s$$reg_info@@", trigger_source);
	} else {
		index += scnprintf(buf + index, ERR_MSG_BUF - index, "$$reg_info@@");
	}

	/* Get UI SOC for track */
	comm_topic = oplus_mms_get_by_name("common");
	if (comm_topic) {
		if (oplus_mms_get_item_data(comm_topic, COMM_ITEM_UI_SOC, &data_soc, false) == 0) {
			ui_soc = data_soc.intval;
		}
	}

	/* Format as 0/1/2/3/4/5/6/7/8/9/A/B/C/D/E/F:[...] */
	index += scnprintf(buf + index, ERR_MSG_BUF - index, "0/1/2/3/4/5/6/7/8/9/A/B/C/D/E/F:[");

	/* Add all register values in array format */
	for (i = 0; i <= SC83107_REGMAX; i++) {
		if (i == 0) {
			index += scnprintf(buf + index, ERR_MSG_BUF - index, "0x%02x", data[i]);
		} else {
			index += scnprintf(buf + index, ERR_MSG_BUF - index, ", 0x%02x", data[i]);
		}
	}
	index += scnprintf(buf + index, ERR_MSG_BUF - index, "]");

	/* Add UI SOC information */
	if (ui_soc >= 0) {
		index += scnprintf(buf + index, ERR_MSG_BUF - index, "$$ui_soc@@%d", ui_soc);
	}

	chg_info("Upload all registers (trigger: %s): %s\n",
		 trigger_source ? trigger_source : "unknown", buf);

	/* Upload register dump via error message */
	sc83107_publish_ic_err_msg(OPLUS_IC_ERR_CP, 0, "%s", buf);

	kfree(buf);
}

static void sc83107_comm_subs_callback(struct mms_subscribe *subs,
					enum mms_msg_type type, u32 id, bool sync)
{
	struct sc83107_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int ui_soc = 0;
	uint8_t reg09_val = 0;
	uint8_t reg0a_val = 0;
	int ret = 0;

	if (!chip)
		return;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_BOOT_COMPLETED:
			chg_info("COMM_ITEM_BOOT_COMPLETED received, trigger get_dischg_boost_err_flag\n");
			sc83107_get_dischg_boost_err_flag_from_partition(chip);
			break;
		case COMM_ITEM_UI_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			ui_soc = data.intval;

			/* Detect when UI SOC drops to 10% (entering ultra power saving mode)
			 * This matches the system logic in oplus_chg_comm.c
			 */
			if (ui_soc == SC83107_UPLOAD_REG_SOC_THRESHOLD && chip->last_ui_soc > SC83107_UPLOAD_REG_SOC_THRESHOLD) {
				chg_info("UI SOC dropped to %d%%, entering ultra power saving mode, upload all registers\n",
					 SC83107_UPLOAD_REG_SOC_THRESHOLD);

				/* Read and save flag registers (0x09 and 0x0A) before they are cleared */
				ret = sc83107_i2c_read_byte(chip, 0x09, &reg09_val);
				if (ret < 0) {
					chg_err("Failed to read register 0x09: %d\n", ret);
					reg09_val = 0xFF;
				}

				ret = sc83107_i2c_read_byte(chip, 0x0A, &reg0a_val);
				if (ret < 0) {
					chg_err("Failed to read register 0x0A: %d\n", ret);
					reg0a_val = 0xFF;
				}

				/* Upload all register values when entering ultra power saving mode */
				sc83107_upload_all_registers(chip, reg09_val, reg0a_val,
							     "ultra_power_saving_mode");
			}
			chip->last_ui_soc = ui_soc;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void sc83107_subscribe_comm_topic(struct oplus_mms *topic, void *prv_data)
{
	struct sc83107_chip *chip = prv_data;
	union mms_msg_data data = { 0 };

	chg_info("subscribe comm topic\n");
	chip->comm_topic = topic;
	chip->comm_subs =
		oplus_mms_subscribe(chip->comm_topic, chip,
				    sc83107_comm_subs_callback, "sc83107");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe comm topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	/* Check if boot already completed */
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_BOOT_COMPLETED, &data, true);
	if (data.intval) {
		chg_info("boot already completed when subscribing, trigger get_dischg_boost_err_flag\n");
		sc83107_get_dischg_boost_err_flag_from_partition(chip);
	} else {
		chg_info("boot not completed yet, will wait for COMM_ITEM_BOOT_COMPLETED event\n");
	}

	/* Get initial UI SOC */
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data, true);
	chip->last_ui_soc = data.intval;
	chg_info("initial UI SOC: %d%%\n", chip->last_ui_soc);
}

/********************* ops end *********************/
static int sc83107_dump_registers(struct sc83107_chip *chip)
{
	int ret = 0;
	uint8_t i = 0;
	uint8_t data[SC83107_REGMAX + 1] = {0};
	s32 err_info[2] = { 0 };

	ret = sc83107_i2c_read_bytes(chip, 0x00, SC83107_REGMAX + 1, data);
	if (ret < 0) {
		ret = sc83107_i2c_read_bytes(chip, 0x00, SC83107_REGMAX + 1, data);
		if (ret < 0) {
			chg_err("Failed to read registers [0x00-0x%02x]: %d\n",
				   SC83107_REGMAX, ret);
			err_info[0] = SC83107_REGMAX + 1;
			err_info[1] = ret;
			sc83107_upload_i2c_err_info(chip, true, err_info);
			/* return ret; */
		}
	}

	for (i = 0; i < SC83107_REGMAX + 1; i++) {
		chg_info("dump reg[0x%02x] = 0x%02x\n", i, data[i]);
	}
	return 0;
}

static int sc83107_init_device(struct sc83107_chip *chip)
{
	int ret = 0;
	int i;

	struct {
		enum sc83107_fields field_id;
		int conv_data;
	} props[] = {
		{F_FSW_SET, chip->cfg.freq},
		{F_OTG_MODE, chip->cfg.otg_mode},
		{F_WD_TIMEOUT, chip->cfg.wdt_timeout},
		{F_VOUT_OVP_OFF, chip->cfg.vout_ovp_dis},
		{F_ILIM_OFF, chip->cfg.current_limit_dis},
	};

	/* ret = sc83107_reg_reset(chip);  Do Not Reset ! */

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		ret = sc83107_field_write(chip, props[i].field_id, props[i].conv_data);
		if (ret < 0) {
			 chg_err("%s: Failed to write field %d\n", __func__, props[i].field_id);
		}
	}

#ifdef CONFIG_OPLUS_CHARGER_MTK
	sc83107_vbat_sns_enable(chip, 0);
#else
	sc83107_bcl_clamp_vol_set(chip, 400);
	sc83107_lv0_comp_falling_threshold_set(chip, 2600);
	sc83107_lv1_comp_falling_threshold_set(chip, 2400);
	sc83107_lv2_comp_input_vol_threshold_set(chip, 2000);
#endif
	sc83107_ton_max_limit_time_set(chip, 750);
	sc83107_hybrid_output_vol_set(chip, 3100);
	sc83107_mask_set(chip, SC83107_MASK_VAC_FALLING, 1);
	sc83107_mask_set(chip, SC83107_MASK_PGOOD, 1);
	sc83107_tonmin1_set(chip, 3);
	sc83107_tonmin2_set(chip, 3);
	/* Set INT pull-down effective time to 10s (register 0x03 = 0x64) */
	ret = sc83107_i2c_write_byte(chip, 0x03, 0x64);
	if (ret < 0) {
		chg_err("Failed to set INT deglitch time to 10s, ret=%d\n", ret);
	} else {
		chg_info("Set INT pull-down effective time to 10s (reg 0x03 = 0x64)\n");
	}

	sc83107_mode_set(chip, SC83107_MODE_AUTO_HYBRID_BP);
	sc83107_check_register_and_upload_track(chip, 0xFF, 0xFF);

	ret = sc83107_dump_registers(chip);

	return ret;
};

static int sc83107_parse_dt(struct sc83107_chip *chip, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int i;
	int ret;
	struct {
		char *name;
		int *conv_data;
	} props[] = {
		{"oplus,sc83107,operation-mode", &(chip->cfg.operation_mode)},
		{"oplus,sc83107,freq", &(chip->cfg.freq)},
		{"oplus,sc83107,otg-mode", &(chip->cfg.otg_mode)},
		{"oplus,sc83107,wdt-timeout", &(chip->cfg.wdt_timeout)},
		{"oplus,sc83107,vout-ovp-dis", &(chip->cfg.vout_ovp_dis)},
		{"oplus,sc83107,current-limit-dis", &(chip->cfg.current_limit_dis)},
	};

	/* initialize data for optional properties */
	for (i = 0; i < ARRAY_SIZE(props); i++) {
		ret = of_property_read_u32(np, props[i].name,
						props[i].conv_data);
		if (ret < 0) {
			chg_err("can not read %s \n", props[i].name);
			return ret;
		}
	}

	/* Parse I2C bus reset feature enable flag (optional, default: disabled) */
	chip->i2c_bus_reset_enable = of_property_read_bool(np, "oplus,sc83107,i2c-bus-reset-enable");
	chg_info("I2C bus reset feature: %s\n", chip->i2c_bus_reset_enable ? "enabled" : "disabled");

	return 0;
}

static ssize_t sc83107_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sc83107_chip *chip = dev_get_drvdata(dev);
	uint8_t addr;
	uint8_t val;
	uint8_t tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc83107");
	for (addr = 0x0; addr <= SC83107_REGMAX; addr++) {
		ret = sc83107_i2c_read_byte(chip, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
					"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t sc83107_store_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sc83107_chip *chip = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= SC83107_REGMAX)
		sc83107_i2c_write_byte(chip, reg, val);

	return count;
}

static DEVICE_ATTR(registers, 0660, sc83107_show_registers, sc83107_store_register);

static void sc83107_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
}

static bool sc83107_detect_device(struct sc83107_chip *chip)
{
	int ret;
	int val;

	ret = sc83107_field_read(chip, F_DEVICE_ID, &val);
	if (ret < 0 || (val != SC83107_DEVICE_ID)) {
		chg_err("not find sc83107, ret=%d, device id=%d\n", ret, val);
		return false;
	}

	chip->dev_id = val;

	return true;
}

static int sc83107_get_battery_soc(struct sc83107_chip *chip)
{
	int soc = 0;
	struct oplus_mms *gauge_topic;
	union mms_msg_data data = { 0 };

	if (!chip)
		return -EINVAL;

	gauge_topic = oplus_mms_get_by_name("gauge");
	if (!gauge_topic) {
		chg_err("gauge topic not found\n");
		return -ENODEV;
	}

	if (oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_SOC, &data, true)) {
		chg_err("get soc fail\n");
		return -EINVAL;
	}

	soc = data.intval;
	return soc;
}

#define FORCE_BP_RETRY_SOC_THRESHOLD 40
static void sc83107_force_bp_retry_work_func(struct work_struct *work)
{
	struct sc83107_chip *chip = container_of(work, struct sc83107_chip,
						  force_bp_retry_work.work);
	int soc = 0;
	int ret = 0;
	int force_bp = 0;

	if (!chip || !chip->force_bp_retry_enabled)
		return;

	/* Force bypass retry is critical for system recovery, use wakelock to prevent suspend during I2C operations */
	if (chip->i2c_wake_lock)
		__pm_wakeup_event(chip->i2c_wake_lock, 500);

	/* Check if still in force bypass mode */
	ret = sc83107_field_read(chip, F_FORCE_BP, &force_bp);
	if (ret < 0) {
		chg_err("read F_FORCE_BP fail, ret=%d\n", ret);
		chip->force_bp_retry_enabled = false;
		return;
	}

	/* Get current SOC */
	soc = sc83107_get_battery_soc(chip);
	if (soc < 0) {
		chg_err("get soc fail, ret=%d\n", soc);
		chip->force_bp_retry_enabled = false;
		return;
	}

	/* If still in force bypass mode and SOC < 40%, try to switch to auto mode */
	if (force_bp && soc < FORCE_BP_RETRY_SOC_THRESHOLD) {
		chg_info("force bypass detected, SOC=%d%%, try to switch to auto mode\n", soc);
		ret = sc83107_mode_set(chip, SC83107_MODE_AUTO_HYBRID_BP);
		if (ret < 0) {
			chg_err("switch to auto mode fail, ret=%d\n", ret);
		} else {
			chg_info("successfully switched to AUTO_HYBRID_BP mode\n");
		}
	} else {
		chg_info("force bypass retry: force_bp=%d, soc=%d%%, no need to retry\n",
			 force_bp, soc);
	}

	chip->force_bp_retry_enabled = false;
}

static void sc83107_track_upload_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc83107_chip *chip = container_of(dwork, struct sc83107_chip,
						  track_upload_work);
	uint8_t reg09_val = 0;
	uint8_t reg0a_val = 0;
	int ret = 0;

	if (!chip || !chip->track_upload_pending)
		return;

	/* Exception interrupt handling is critical, use wakelock to prevent suspend during I2C operations */
	if (chip->i2c_wake_lock)
		__pm_wakeup_event(chip->i2c_wake_lock, 500);
	/* First, read and save flag registers (0x09 and 0x0A) before they are cleared
	 * These flag registers will be cleared after reading, so we must read them first
	 * and save the values for later use
	 */
	ret = sc83107_i2c_read_byte(chip, 0x09, &reg09_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x09: %d\n", ret);
		reg09_val = 0;
	}

	ret = sc83107_i2c_read_byte(chip, 0x0A, &reg0a_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x0A: %d\n", ret);
		reg0a_val = 0;
	}

	/* Check and upload error flags using the saved values
	 * This prevents the flag registers from being read again and cleared
	 */
	sc83107_check_register_and_upload_track(chip, reg09_val, reg0a_val);

	/* Upload all register values when exception interrupt is triggered
	 * Use the saved flag register values to ensure the uploaded dump
	 * contains the original flag values before they were cleared
	 * Mark as "exception_interrupt" to distinguish from charger plug/unplug events
	 */
	sc83107_upload_all_registers(chip, reg09_val, reg0a_val, "exception_interrupt");

	chip->track_upload_pending = false;
}

static void sc83107_irq_handler_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sc83107_chip *chip = container_of(dwork, struct sc83107_chip,
						  irq_handler_work);
	int ret = 0;
	int soc = 0;
	int force_bp = 0;

	if (!chip || !chip->irq_handler_work_pending)
		return;

	/* Exception interrupt handling is critical, use wakelock to prevent suspend during I2C operations */
	if (chip->i2c_wake_lock)
		__pm_wakeup_event(chip->i2c_wake_lock, 500);

	chg_info("process IRQ handler work\n");

	/* Step 1: Read register 0x01[0] to check force bypass mode */
	ret = sc83107_field_read(chip, F_FORCE_BP, &force_bp);
	if (ret < 0) {
		chg_err("read F_FORCE_BP fail, ret=%d\n", ret);
		chip->irq_handler_work_pending = false;
		return;
	}

	/* Step 2: If force bypass mode is set (bit 0 = 1) and SOC < 40%,
	 * it indicates abnormal fallback to force bypass mode
	 */
	if (force_bp) {
		/* Get current battery SOC */
		soc = sc83107_get_battery_soc(chip);
		if (soc < 0) {
			chg_err("get soc fail, ret=%d\n", soc);
			chip->irq_handler_work_pending = false;
			return;
		}

		/* If force_bp is 1 and SOC < 40%, it indicates abnormal fallback to force bypass mode */
		if (soc < FORCE_BP_RETRY_SOC_THRESHOLD) {
			chg_err("abnormal fallback to force bypass mode detected, SOC=%d%%, reg01[0]=1\n", soc);
			/* Start retry mechanism: schedule work to retry entering auto mode after 1s */
			if (!chip->force_bp_retry_enabled) {
				chip->force_bp_retry_enabled = true;
				schedule_delayed_work(&chip->force_bp_retry_work,
						     msecs_to_jiffies(1000));
				chg_info("scheduled force bypass retry work, will retry after 1s\n");
			} else {
				chg_info("force bypass retry already scheduled, skip\n");
			}
		}
	}

	chip->irq_handler_work_pending = false;
}

static irqreturn_t sc83107_irq_handler(int irq, void *data)
{
	struct sc83107_chip *chip = data;
	chg_info("enter %s\n", __func__);

	/* Step 1: Record and upload track data for abnormal interrupt */
	if (!chip->track_upload_pending) {
		chip->track_upload_pending = true;
		schedule_delayed_work(&chip->track_upload_work, 0);
		chg_info("scheduled track upload work for abnormal interrupt\n");
	}

	/* Step 2: Schedule work to process interrupt logic in process context. */
	if (!chip->irq_handler_work_pending) {
		chip->irq_handler_work_pending = true;
		schedule_delayed_work(&chip->irq_handler_work, 0);
		chg_info("scheduled IRQ handler work for process context\n");
	}

	return IRQ_HANDLED;
}

static int sc83107_register_interrupt(struct sc83107_chip *chip)
{
	struct device_node *np = chip->dev->of_node;
	int rc = 0;

	chip->irq_gpio = of_get_named_gpio(np, "oplus,sc83107,irq-gpio", 0);
	if (!gpio_is_valid(chip->irq_gpio)) {
		chg_err("irq_gpio not specified, rc=%d\n", chip->irq_gpio);
		return chip->irq_gpio;
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

	chip->boost_inter_active =
	    pinctrl_lookup_state(chip->pinctrl, "boost_inter_active");
	if (IS_ERR_OR_NULL(chip->boost_inter_active)) {
		chg_err("failed to get the pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	chip->boost_inter_sleep =
	    pinctrl_lookup_state(chip->pinctrl, "boost_inter_sleep");
	if (IS_ERR_OR_NULL(chip->boost_inter_sleep)) {
		chg_err("Failed to get the pinctrl state(%d)\n", __LINE__);
		return -EINVAL;
	}

	gpio_direction_input(chip->irq_gpio);
	pinctrl_select_state(chip->pinctrl, chip->boost_inter_active); /* PULL UP */
	rc = gpio_get_value(chip->irq_gpio);
	chg_info("irq_gpio = %d, irq_gpio_stat = %d\n", chip->irq_gpio, rc);

	if (chip->irq) {
		rc = devm_request_threaded_irq(&chip->client->dev, chip->irq,
				NULL, sc83107_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"sc83107-irq", chip);

		if (rc < 0) {
			chg_err("request irq for irq=%d failed, rc =%d\n",
							chip->irq, rc);
			return rc;
		}
	}

	return rc;
}

static void sc83107_charger_plug_work_func(struct work_struct *work)
{
	struct sc83107_chip *chip = container_of(work, struct sc83107_chip,
						  charger_plug_work);
	int soc = 0;
	int ret = 0;
	uint8_t reg01_val = 0;
	uint8_t reg09_val = 0;
	uint8_t reg0a_val = 0;
	bool charger_present = false;
	struct votable *work_mode_votable = NULL;
	int rus_force_bypass = 0;

	if (!chip)
		return;

	/* Charger plug work is critical, use wakelock to prevent suspend during I2C operations */
	if (chip->i2c_wake_lock)
		__pm_wakeup_event(chip->i2c_wake_lock, 500);

	/* Check charger present status */
	if (chip->wired_topic) {
		union mms_msg_data data = { 0 };
		oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_PRESENT, &data, false);
		charger_present = (data.intval != 0);
	}

	/* Get current battery SOC */
	soc = sc83107_get_battery_soc(chip);
	if (soc < 0) {
		chg_err("get soc fail, ret=%d\n", soc);
		return;
	}

	chg_info("charger %s event, SOC=%d%%\n", charger_present ? "plug in" : "plug out", soc);

	/* Read and save flag registers (0x09 and 0x0A) before they are cleared
	 * These flag registers will be cleared after reading, so we save them first
	 */
	ret = sc83107_i2c_read_byte(chip, 0x09, &reg09_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x09: %d\n", ret);
		reg09_val = 0xFF; /* Use 0xFF to indicate not available */
	}

	ret = sc83107_i2c_read_byte(chip, 0x0A, &reg0a_val);
	if (ret < 0) {
		chg_err("Failed to read register 0x0A: %d\n", ret);
		reg0a_val = 0xFF; /* Use 0xFF to indicate not available */
	}

	/* Upload all register values when charger plug/unplug event is triggered
	 * Mark with trigger source to distinguish from exception interrupt
	 */
	sc83107_upload_all_registers(chip, reg09_val, reg0a_val,
				     charger_present ? "charger_plug_in" : "charger_plug_out");

	sc83107_otg_set(chip, 0);
	sc83107_force_fpwm_set(chip, 0);

	/* Check if RUS has forced force bypass mode */
	work_mode_votable = find_votable("BOOST_WORK_MODE");
	if (work_mode_votable) {
		rus_force_bypass = get_client_vote(work_mode_votable, BOOST_RUS_VOTER);
		if (rus_force_bypass > 0) {
			chg_info("RUS force bypass is set, value=%d, force bypass mode has highest priority\n",
				rus_force_bypass);
		}
	} else {
		chg_info("BOOST_WORK_MODE votable not found, use default logic\n");
	}

	/* Read register 0x01 to check current mode */
	ret = sc83107_i2c_read_byte(chip, 0x01, &reg01_val);
	if (ret < 0) {
		chg_err("read register 0x01 fail, ret=%d\n", ret);
		return;
	}

	chg_info("register 0x01 current value: 0x%02x\n", reg01_val);

	/* If RUS has forced force bypass mode, it has the highest priority */
	if (rus_force_bypass > 0) {
		/* RUS force bypass: must be force bypass mode (0x11) regardless of SOC
		 * Note: RUS setting has already been applied through votable callback,
		 * but we still need to verify and correct if register value is wrong
		 * (e.g., changed by other operations or hardware issues)
		 */
		if (reg01_val != 0x11) {
			chg_info("RUS force bypass enabled, SOC=%d%%, but register 0x01=0x%02x != 0x11, correct to force bypass mode\n",
				soc, reg01_val);
			ret = sc83107_i2c_write_byte(chip, 0x01, 0x11);
			if (ret < 0) {
				chg_err("write register 0x01=0x11 fail, ret=%d\n", ret);
			} else {
				chg_info("successfully corrected register 0x01 to 0x11 (force bypass mode) by RUS\n");
			}
		} else {
			chg_info("RUS force bypass enabled, SOC=%d%%, register 0x01=0x11 (force bypass mode) is correct, no need to write again\n", soc);
		}
	} else {
		/* No RUS force bypass: use default logic based on SOC */
		/* Check SOC and verify/correct register 0x01 value */
		if (soc >= FORCE_BP_RETRY_SOC_THRESHOLD) {
			/* SOC >= 40%: register 0x01 should be 0x11 (force bypass mode) */
			if (reg01_val != 0x11) {
				chg_err("SOC=%d%% > 40%%, but register 0x01=0x%02x != 0x11, set to force bypass mode\n",
					soc, reg01_val);
				ret = sc83107_i2c_write_byte(chip, 0x01, 0x11);
				if (ret < 0) {
					chg_err("write register 0x01=0x11 fail, ret=%d\n", ret);
				} else {
					chg_info("successfully set register 0x01 to 0x11 (force bypass mode)\n");
				}
			} else {
				chg_info("SOC=%d%% > 40%%, register 0x01=0x11 (force bypass mode) is correct\n", soc);
			}
		} else {
			/* SOC < 40%: register 0x01 should be 0x10 (auto mode) */
			if (reg01_val != 0x10) {
				chg_err("SOC=%d%% < 40%%, but register 0x01=0x%02x != 0x10, set to auto mode\n",
					soc, reg01_val);
				/* Use sc83107_mode_set to set auto mode (includes verification and retry) */
				ret = sc83107_mode_set(chip, SC83107_MODE_AUTO_HYBRID_BP);
				if (ret < 0) {
					chg_err("set auto mode fail, ret=%d\n", ret);
				} else {
					chg_info("successfully set to auto mode\n");
				}
			} else {
				chg_info("SOC=%d%% < 40%%, register 0x01=0x10 (auto mode) is correct\n", soc);
			}
		}
	}
}

static void sc83107_wired_subs_callback(struct mms_subscribe *subs,
					enum mms_msg_type type, u32 id, bool sync)
{
	struct sc83107_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chg_info("WIRED_ITEM_PRESENT: data.intval=%d\n", data.intval);
			if (data.intval) {
				chg_info("charger plug in detected, schedule charger_plug_work\n");
				schedule_work(&chip->charger_plug_work);
			} else {
				chg_info("charger plug out detected\n");
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void sc83107_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct sc83107_chip *chip = prv_data;
	union mms_msg_data data = { 0 };

	chg_info("subscribe wired topic\n");
	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    sc83107_wired_subs_callback, "sc83107");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	/* Check initial present status */
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_PRESENT, &data, true);
	chg_info("initial WIRED_ITEM_PRESENT status: data.intval=%d\n", data.intval);
	if (data.intval) {
		chg_info("charger already plugged in at init, schedule charger_plug_work\n");
		schedule_work(&chip->charger_plug_work);
	}
}


static int sc83107_boost_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int sc83107_boost_exit(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc83107_chip *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	chip = oplus_chg_ic_get_priv_data(ic_dev);
	if (chip) {
		/* Unsubscribe wired topic */
		if (chip->wired_subs) {
			oplus_mms_unsubscribe(chip->wired_subs);
			chip->wired_subs = NULL;
		}
		chip->wired_topic = NULL;

		/* Unsubscribe comm topic */
		if (chip->comm_subs) {
			oplus_mms_unsubscribe(chip->comm_subs);
			chip->comm_subs = NULL;
		}
		chip->comm_topic = NULL;

		/* Stop force bypass retry */
		if (chip->force_bp_retry_enabled) {
			chip->force_bp_retry_enabled = false;
			cancel_delayed_work_sync(&chip->force_bp_retry_work);
		}
		/* Cancel track upload work */
		if (chip->track_upload_pending) {
			chip->track_upload_pending = false;
			cancel_delayed_work_sync(&chip->track_upload_work);
		}
		/* Cancel IRQ handler work */
		if (chip->irq_handler_work_pending) {
			chip->irq_handler_work_pending = false;
			cancel_delayed_work_sync(&chip->irq_handler_work);
		}
		/* Cancel charger plug work */
		cancel_work_sync(&chip->charger_plug_work);
		/* Cancel dischg boost err flag upload work */
		cancel_work_sync(&chip->dischg_boost_err_flag_upload_work);
	}

	return 0;
}

static int sc83107_boost_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc83107_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	/* reg_dump */
	return 0;
}

static int sc83107_boost_set_cv(struct oplus_chg_ic_dev *ic_dev, int vol)
{
	int ret;
	struct sc83107_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	ret = sc83107_hybrid_output_vol_set(chip, vol);
	if (ret < 0) {
		/* Only print error if not suspended (suspend returns -EAGAIN) */
		if (ret != -EAGAIN || !atomic_read(&chip->suspended))
			chg_err("set_cv fail, ret=%d, vol=%d\n", ret, vol);

		/* Don't print log for suspend-induced -EAGAIN to avoid excessive logging */
		return ret;
	}

	/* Only print success log when not suspended to avoid excessive logging during suspend/resume */
	if (!atomic_read(&chip->suspended))
		chg_info("set boost cv = %d\n", vol);

	return 0;
}

static int sc83107_boost_enable_otg_mode(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int ret;
	struct sc83107_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	ret = sc83107_otg_set(chip, en);
	if (ret < 0)
		chg_err("set otg mode fail, ret=%d, en=%d\n", ret, en);
	else
		chg_info("set otg mode=%d success\n", en);

	return ret;
}

static int sc83107_boost_set_work_mode(struct oplus_chg_ic_dev *ic_dev, int mode)
{
	int ret;
	struct sc83107_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	ret = sc83107_mode_set(chip, mode);
	if (ret < 0) {
		chg_err("set work mode fail, ret=%d, mode=%d\n", ret, mode);
		return ret;
	}

	return 0;
}

static int sc83107_boost_set_bcl_rate(struct oplus_chg_ic_dev *ic_dev, int rate)
{
	struct sc83107_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	/* sc83107 not support set rate */
	return 0;
}

static int sc83107_boost_set_bcl_vol(struct oplus_chg_ic_dev *ic_dev, int vol0, int vol1, int vol2)
{
	int ret;
	struct sc83107_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	ret = sc83107_lv0_comp_falling_threshold_set(chip, vol0);
	if (ret < 0)
		chg_err("set_cv fail, ret=%d, mode=%d\n", ret, vol0);

	ret = sc83107_lv1_comp_falling_threshold_set(chip, vol1);
	if (ret < 0)
		chg_err("set_cv fail, ret=%d, mode=%d\n", ret, vol1);

	ret = sc83107_lv2_comp_input_vol_threshold_set(chip, vol2);
	if (ret < 0)
		chg_err("set_cv fail, ret=%d, mode=%d\n", ret, vol2);

	return 0;
}

static int sc83107_boost_get_in_cv_mode(struct oplus_chg_ic_dev *ic_dev, bool *cv_mode)
{
	int ret;
	struct sc83107_chip *chip;
	bool enable = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	ret = sc83107_dcdc_mode_is_enable(chip, &enable);
	if (ret < 0)
		chg_err("get cv mode fail, ret=%d\n", ret);
	*cv_mode = enable;

	chg_info("cv mode = %d\n", *cv_mode);

	return 0;
}

static int sc83107_boost_get_cv(struct oplus_chg_ic_dev *ic_dev, int *cv)
{
	int ret;
	struct sc83107_chip *chip;
	int cv_mv = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	ret = sc83107_hybrid_output_vol_get(chip, &cv_mv);
	if (ret < 0)
		chg_err("get cv fail, ret=%d\n", ret);
	*cv = cv_mv;

	chg_info("cv = %d\n", *cv);

	return 0;
}

static int sc83107_boost_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct sc83107_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (suspend == NULL) {
		chg_err("suspend pointer is NULL");
		return -EINVAL;
	}

	*suspend = (atomic_read(&chip->suspended) != 0);
	return 0;
}

static int sc83107_boost_set_suspend_resume_cv(struct oplus_chg_ic_dev *ic_dev, int suspend_cv, int resume_cv)
{
	struct sc83107_chip *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chip->suspend_cv_mv = suspend_cv;
	chip->resume_cv_mv = resume_cv;
	chg_info("set suspend_cv=%d, resume_cv=%d\n", suspend_cv, resume_cv);

	return 0;
}

static void *sc83107_boost_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, sc83107_boost_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, sc83107_boost_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, sc83107_boost_reg_dump);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_CV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_CV, sc83107_boost_set_cv);
		break;
	case OPLUS_IC_FUNC_BOOST_ENABLE_OTG_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_ENABLE_OTG_MODE, sc83107_boost_enable_otg_mode);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_WORK_MODE, sc83107_boost_set_work_mode);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_BCL_RATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_BCL_RATE, sc83107_boost_set_bcl_rate);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_BCL_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_BCL_VOL, sc83107_boost_set_bcl_vol);
		break;
	case OPLUS_IC_FUNC_BOOST_GET_IN_CV_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_GET_IN_CV_MODE, sc83107_boost_get_in_cv_mode);
		break;
	case OPLUS_IC_FUNC_BOOST_GET_CV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_GET_CV, sc83107_boost_get_cv);
		break;
	case OPLUS_IC_FUNC_BOOST_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_IS_SUSPEND, sc83107_boost_is_suspend);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_SUSPEND_RESUME_CV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_SUSPEND_RESUME_CV, sc83107_boost_set_suspend_resume_cv);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq sc83107_boost_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int sc83107_ic_register(struct sc83107_chip *chip)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *child;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg;
	int rc;

	for_each_child_of_node(chip->dev->of_node, child) {
		rc = of_property_read_u32(child, "oplus,ic_type", &ic_type);
		if (rc < 0)
			continue;
		rc = of_property_read_u32(child, "oplus,ic_index", &ic_index);
		if (rc < 0)
			continue;
		ic_cfg.name = child->name;
		ic_cfg.index = ic_index;
		ic_cfg.type = ic_type;
		ic_cfg.priv_data = chip;
		ic_cfg.of_node = child;
		switch (ic_type) {
		case OPLUS_CHG_IC_BOOST:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "boost-sc83107:%d", ic_index);
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = sc83107_boost_get_func;
			ic_cfg.virq_data = sc83107_boost_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(sc83107_boost_virq_table);
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_type);
			continue;
		}
		ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
		if (!ic_dev) {
			rc = -ENODEV;
			chg_err("register %s error\n", child->name);
			continue;
		}
		chg_info("register %s\n", child->name);

		switch (ic_dev->type) {
		case OPLUS_CHG_IC_BOOST:
			chip->boost_ic = ic_dev;
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_dev->type);
			continue;
		}

		of_platform_populate(child, NULL, NULL, chip->dev);
	}

	return 0;
}

static const struct of_device_id sc83107_charger_match_table[] = {
	{ .compatible = "oplus,sc83107", },
	{},
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sc83107_charger_probe(struct i2c_client *client)
#else
static int sc83107_charger_probe(struct i2c_client *client,
		    const struct i2c_device_id *id)
#endif
{
	struct sc83107_chip *chip;
	int ret;

	chg_info("(%s)\n", SC83107_DRV_VERSION);

	chip = devm_kzalloc(&client->dev, sizeof(struct sc83107_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	chip->dev = &client->dev;
	chip->client = client;
	atomic_set(&chip->suspended, 0);

	INIT_WORK(&chip->charger_plug_work, sc83107_charger_plug_work_func);
	INIT_DELAYED_WORK(&chip->force_bp_retry_work, sc83107_force_bp_retry_work_func);
	INIT_DELAYED_WORK(&chip->track_upload_work, sc83107_track_upload_work_func);
	INIT_DELAYED_WORK(&chip->irq_handler_work, sc83107_irq_handler_work_func);
	INIT_DELAYED_WORK(&chip->get_dischg_boost_err_flag_work, sc83107_get_dischg_boost_err_flag_work_func);
	INIT_WORK(&chip->dischg_boost_err_flag_upload_work, sc83107_dischg_boost_err_flag_upload_work_func);
	chip->force_bp_retry_enabled = false;
	chip->track_upload_pending = false;
	chip->irq_handler_work_pending = false;
	chip->last_ui_soc = -1; /* Initialize to -1 to detect first UI SOC update */
	chip->suspend_cv_mv = 0;
	chip->resume_cv_mv = 0;

	/* Initialize wakelock for critical I2C operations */
	chip->i2c_wake_lock = wakeup_source_register(chip->dev, "sc83107_i2c_wakeup");
	if (!chip->i2c_wake_lock)
		chg_err("Failed to register wakelock\n");

	i2c_set_clientdata(client, chip);

	/* Parse DTS first so that I2C bus reset feature can be enabled before device detection */
	ret = sc83107_parse_dt(chip, &client->dev);
	if (ret < 0) {
		chg_err("parse dt failed(%d)\n", ret);
		goto err_parse_dt;
	}

	if (!sc83107_detect_device(chip)) {
		ret = -ENODEV;
		goto err_init_device;
	}

	sc83107_create_device_node(&(client->dev));

	ret = sc83107_register_interrupt(chip);
	if (ret < 0) {
		chg_err("register irq fail(%d)\n", ret);
		goto err_register_irq;
	}

	ret = sc83107_init_device(chip);
	if (ret < 0) {
		chg_err("init device failed(%d)\n", ret);
		goto err_init_device;
	}

	ret = sc83107_ic_register(chip);
	if (ret < 0) {
		chg_err("ic register failed(%d)\n", ret);
		goto err_init_device;
	}

	sc83107_boost_init(chip->boost_ic);

	/* Subscribe to wired topic for charger plug events */
	oplus_mms_wait_topic("wired", sc83107_subscribe_wired_topic, chip);

	/* Subscribe to comm topic for boot completed event */
	oplus_mms_wait_topic("common", sc83107_subscribe_comm_topic, chip);

	/* Register mutual notifier for reading error flag from partition */
	ret = sc83107_dischg_boost_err_flag_mutual_notify_reg(chip);
	if (ret < 0) {
		chg_err("register dischg boost err flag mutual notifier failed(%d)\n", ret);
		/* Continue even if registration fails */
	}

	chg_info("sc83107 probe successfully!\n");
	return 0;

err_register_irq:
err_init_device:
err_parse_dt:
	if (chip) {
		cancel_work_sync(&chip->charger_plug_work);
		cancel_delayed_work_sync(&chip->force_bp_retry_work);
		cancel_delayed_work_sync(&chip->track_upload_work);
		cancel_delayed_work_sync(&chip->irq_handler_work);
		cancel_delayed_work_sync(&chip->get_dischg_boost_err_flag_work);
		cancel_work_sync(&chip->dischg_boost_err_flag_upload_work);
	}
err_kzalloc:
	chg_err("sc83107 probe fail\n");
	return ret;
}

static void sc83107_charger_remove(struct i2c_client *client)
{
	struct sc83107_chip *chip = i2c_get_clientdata(client);

	if (!chip)
		return;

	chg_info("enter\n");

	/* Cancel delayed work for getting error flag */
	cancel_delayed_work_sync(&chip->get_dischg_boost_err_flag_work);

	/* Unsubscribe comm topic */
	if (chip->comm_subs) {
		oplus_mms_unsubscribe(chip->comm_subs);
		chip->comm_subs = NULL;
	}
	chip->comm_topic = NULL;

	/* Unregister mutual notifier */
	oplus_chg_unreg_mutual_notifier(&chip->dischg_boost_err_flag_mutual);

	if (chip->boost_ic)
		sc83107_boost_exit(chip->boost_ic);

	if (chip->irq > 0)
		disable_irq(chip->irq);

	device_remove_file(chip->dev, &dev_attr_registers);

	/* Unregister wakelock */
	if (chip->i2c_wake_lock) {
		wakeup_source_unregister(chip->i2c_wake_lock);
		chip->i2c_wake_lock = NULL;
	}

	if (gpio_is_valid(chip->irq_gpio))
		gpio_free(chip->irq_gpio);
}

#ifdef CONFIG_PM_SLEEP
static int sc83107_suspend(struct device *dev)
{
	struct sc83107_chip *chip = dev_get_drvdata(dev);
	int rc;

	/* Set suspend CV before setting suspended flag */
	if (chip->suspend_cv_mv > 0) {
		/* I2C bus should still be active at this point, so operation should succeed */
		rc = sc83107_hybrid_output_vol_set(chip, chip->suspend_cv_mv);
		if (rc < 0 && rc != -EAGAIN)
			chg_err("set suspend cv=%d failed, rc=%d\n", chip->suspend_cv_mv, rc);
		else
			chg_info("set suspend cv=%d\n", chip->suspend_cv_mv);
	}

	atomic_set(&chip->suspended, 1);
	chg_info("Suspend successfully!");
	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irq);
	disable_irq(chip->irq);

	return 0;
}
static int sc83107_resume(struct device *dev)
{
	struct sc83107_chip *chip = dev_get_drvdata(dev);
	int rc;

	atomic_set(&chip->suspended, 0);
	chg_info("Resume successfully!");
	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irq);
	enable_irq(chip->irq);

	/* Set resume CV after setting suspended flag to 0 */
	if (chip->resume_cv_mv > 0) {
		rc = sc83107_hybrid_output_vol_set(chip, chip->resume_cv_mv);
		if (rc < 0)
			chg_err("set resume cv=%d failed, rc=%d\n", chip->resume_cv_mv, rc);
		else
			chg_info("set resume cv=%d\n", chip->resume_cv_mv);
	}

	return 0;
}

static const struct dev_pm_ops sc83107_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(sc83107_suspend, sc83107_resume)
};
#endif

static struct i2c_driver sc83107_i2c_driver = {
	.driver     = {
		.name   = "sc83107",
		.owner  = THIS_MODULE,
		.of_match_table = sc83107_charger_match_table,
#ifdef CONFIG_PM_SLEEP
		.pm = &sc83107_pm,
#endif
	},
	.probe      = sc83107_charger_probe,
	.remove     = sc83107_charger_remove,
};

static __init int sc83107_i2c_driver_init(void)
{
	return i2c_add_driver(&sc83107_i2c_driver);
}

static __exit void sc83107_i2c_driver_exit(void)
{
	i2c_del_driver(&sc83107_i2c_driver);
}

oplus_chg_module_register(sc83107_i2c_driver);

MODULE_DESCRIPTION("OPLUS SC83107 Driver");
MODULE_LICENSE("GPL v2");
