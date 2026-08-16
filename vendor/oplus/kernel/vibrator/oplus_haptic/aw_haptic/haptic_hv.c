/*
 * File: haptic_hv.c
 *
 * Author: Ethan <chelvming@awinic.com>
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/vmalloc.h>
#include <linux/pm_qos.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <linux/mman.h>

#include "haptic_hv.h"
#include "haptic_hv_reg.h"

#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
#include "../../aw8697_haptic/haptic_feedback.h"
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
#include "../haptic_common/haptic_wave.h"
#endif

#define HAPTIC_HV_DRIVER_VERSION	"v0.0.0.13"
/* add for DX-2 bringup */
#define FW_ACTION_HOTPLUG 1

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
#define CPU_LATENCY_QOC_VALUE (0)
static struct pm_qos_request pm_qos_req;
#else
struct pm_qos_request aw_pm_qos_req_vb;
#endif

static void aw_pm_qos_enable(struct aw_haptic *aw_haptic, bool enabled)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	if (enabled)
		cpu_latency_qos_add_request(&pm_qos_req, CPU_LATENCY_QOC_VALUE);
	else
		cpu_latency_qos_remove_request(&pm_qos_req);
#else
	mutex_lock(&aw_haptic->qos_lock);
	if (enabled) {
		if (!pm_qos_request_active(&aw_pm_qos_req_vb))
			pm_qos_add_request(&aw_pm_qos_req_vb,
					PM_QOS_CPU_DMA_LATENCY,
					AW_PM_QOS_VALUE_VB);
		else
			pm_qos_update_request(&aw_pm_qos_req_vb,
					AW_PM_QOS_VALUE_VB);

	} else {
		pm_qos_remove_request(&aw_pm_qos_req_vb);
		/* pm_qos_update_request(&aw_pm_qos_req_vb, PM_QOS_DEFAULT_VALUE); */
	}
	mutex_unlock(&aw_haptic->qos_lock);
#endif
}

struct aw_haptic_container *aw_rtp;
struct aw_haptic *g_aw_haptic;
static int rtp_osc_cali(struct aw_haptic *);
static void rtp_trim_lra_cali(struct aw_haptic *);
int aw_container_size = AW_CONTAINER_DEFAULT_SIZE;

static int container_init(int size)
{
	if (!aw_rtp || size > aw_container_size) {
		if (aw_rtp) {
			vfree(aw_rtp);
		}
		aw_rtp = vmalloc(size);
		if (!aw_rtp) {
			aw_dev_err("%s: error allocating memory\n", __func__);
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
			(void)oplus_haptic_track_mem_alloc_err(HAPTIC_MEM_ALLOC_TRACK, size, __func__);
#endif
			return -ENOMEM;
		}
		aw_container_size = size;
	}

	memset(aw_rtp, 0, size);

	return 0;
}

/*********************************************************
 *
 * I2C Read/Write
 *
 *********************************************************/
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_BSP_DRV_VND_INJECT_TEST)
noinline
#endif
int i2c_r_bytes(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint8_t *buf,
		uint32_t len)
{
	int ret;
	struct i2c_msg msg[] = {
		[0] = {
			.addr = aw_haptic->i2c->addr,
			.flags = 0,
			.len = sizeof(uint8_t),
			.buf = &reg_addr,
			},
		[1] = {
			.addr = aw_haptic->i2c->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
			},
	};

	ret = i2c_transfer(aw_haptic->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		aw_dev_err("%s: transfer failed.", __func__);
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		(void)oplus_haptic_track_dev_err(HAPTIC_I2C_READ_TRACK_ERR, reg_addr, ret);
#endif
		return ret;
	} else if (ret != 2) {
		aw_dev_err("%s: transfer failed(size error).", __func__);
		return -ENXIO;
	}

	return ret;
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_BSP_DRV_VND_INJECT_TEST)
noinline
#endif
int i2c_w_bytes(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint8_t *buf,
		uint32_t len)
{
	uint8_t *data = NULL;
	int ret = -1;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL)
		return -EINVAL;
	data[0] = reg_addr;
	memcpy(&data[1], buf, len);
	ret = i2c_master_send(aw_haptic->i2c, data, len + 1);
	if (ret < 0) {
		aw_dev_err("%s: i2c master send 0x%02x error\n",
			   __func__, reg_addr);
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		(void)oplus_haptic_track_dev_err(HAPTIC_I2C_WRITE_TRACK_ERR, reg_addr, ret);
#endif
	}
	kfree(data);
	return ret;
}

int i2c_w_bits(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint32_t mask,
	       uint8_t reg_data)
{
	uint8_t reg_val = 0;
	int ret = -1;

	ret = i2c_r_bytes(aw_haptic, reg_addr, &reg_val, AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_dev_err("%s: i2c read error, ret=%d\n",
			   __func__, ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= reg_data;
	ret = i2c_w_bytes(aw_haptic, reg_addr, &reg_val, AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_dev_err("%s: i2c write error, ret=%d\n",
			   __func__, ret);
		return ret;
	}
	return 0;
}

/*
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_BSP_DRV_VND_INJECT_TEST)
int aw869xx_i2c_r_byte(uint8_t reg_addr, uint8_t *buf)
{
	int ret = -1;

	ret = i2c_r_bytes(g_aw_haptic, reg_addr, buf, 1);
	if (ret < 0) {
		aw_dev_err("%s: i2c aw869xx_i2c_r_byte 0x%02x error\n",
					__func__, reg_addr);
	}

	return ret;
}
EXPORT_SYMBOL(aw869xx_i2c_r_byte);


int aw869xx_i2c_w_byte(uint8_t reg_addr, uint8_t *buf)
{
	int ret = -1;

	ret = i2c_w_bytes(g_aw_haptic, reg_addr, buf, 1);
	if (ret < 0) {
		aw_dev_err("%s: i2c aw869xx_i2c_w_byte 0x%02x error\n",
					__func__, reg_addr);
	}

	return ret;
}
EXPORT_SYMBOL(aw869xx_i2c_w_byte);
#endif
*/

static void hw_reset(struct aw_haptic *aw_haptic)
{
	aw_dev_info("%s: enter\n", __func__);
	if (aw_haptic && gpio_is_valid(aw_haptic->reset_gpio)) {
		gpio_set_value_cansleep(aw_haptic->reset_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(aw_haptic->reset_gpio, 1);
		usleep_range(8000, 8500);
	} else {
		aw_dev_err("%s: failed\n", __func__);
	}
}

void sw_reset(struct aw_haptic *aw_haptic)
{
	uint8_t reset = AW_BIT_RESET;

	aw_dev_dbg("%s: enter\n", __func__);
	i2c_w_bytes(aw_haptic, AW_REG_CHIPID, &reset, AW_I2C_BYTE_ONE);
	usleep_range(3000, 3500);
}

static int judge_value(uint8_t reg)
{
	int ret = 0;

	if (!reg)
		return -ERANGE;
	switch (reg) {
	case AW86925_BIT_RSTCFG_PRE_VAL:
	case AW86926_BIT_RSTCFG_PRE_VAL:
	case AW86927_BIT_RSTCFG_PRE_VAL:
	case AW86928_BIT_RSTCFG_PRE_VAL:
	case AW86925_BIT_RSTCFG_VAL:
	case AW86926_BIT_RSTCFG_VAL:
	case AW86927_BIT_RSTCFG_VAL:
	case AW86928_BIT_RSTCFG_VAL:
		ret = -ERANGE;
		break;
	default:
		break;
	}
	return ret;
}

static int read_chipid(struct aw_haptic *aw_haptic, uint32_t *reg_val)
{
	uint8_t value[2] = {0};
	int ret = 0;

	aw_dev_dbg("%s: enter!\n", __func__);
	/* try the old way of read chip id */
	ret = i2c_r_bytes(aw_haptic, AW_REG_CHIPID, &value[0], AW_I2C_BYTE_ONE);
	if (ret < 0)
		return ret;

	ret = judge_value(value[0]);
	if (!ret) {
		*reg_val = value[0];
		return ret;
	}
	/* try the new way of read chip id */
	ret = i2c_r_bytes(aw_haptic, AW_REG_CHIPIDH, value, AW_I2C_BYTE_TWO);
	if (ret < 0)
		return ret;
	*reg_val = value[0] << 8 | value[1];
	return ret;
}

static int parse_chipid(struct aw_haptic *aw_haptic)
{
	int ret = -1;
	uint32_t reg = 0;
	uint8_t cnt = 0;

	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		ret = read_chipid(aw_haptic, &reg);
		aw_dev_info("%s: reg_val = 0x%02X\n",
			    __func__, reg);
		if (ret < 0) {
			aw_dev_err("%s: failed to read AW_REG_ID: %d\n",
				   __func__, ret);
		}
		switch (reg) {
		case AW8695_CHIPID:
			aw_haptic->chipid = AW8695_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->i2s_config = false;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_dev_info("%s: detected aw8695.\n",
				    __func__);
			return 0;
		case AW86905_CHIPID:
			aw_haptic->chipid = AW86905_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->i2s_config = false;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_dev_info("%s: detected aw86905.\n", __func__);
			return 0;
		case AW86907_CHIPID:
			aw_haptic->chipid = AW86907_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L2;
			aw_haptic->i2s_config = false;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_dev_info("%s: detected aw86907.\n", __func__);
			return 0;
		case AW86915_CHIPID:
			aw_haptic->chipid = AW86915_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_dev_info("%s: detected aw86915.\n", __func__);
			return 0;
		case AW86917_CHIPID:
			aw_haptic->chipid = AW86917_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L2;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_dev_info("%s: detected aw86917.\n", __func__);
			return 0;

		case AW86925_CHIPID:
			aw_haptic->chipid = AW86925_CHIPID;
			aw_dev_info("%s: detected aw86925.\n",
				    __func__);
			return 0;

		case AW86926_CHIPID:
			aw_haptic->chipid = AW86926_CHIPID;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_dev_info("%s: detected aw86926.\n",
				    __func__);
			return 0;
		case AW86927_CHIPID:
			aw_haptic->chipid = AW86927_CHIPID;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_dev_info("%s: detected aw86927.\n",
				    __func__);
			return 0;
		case AW86928_CHIPID:
			aw_haptic->chipid = AW86928_CHIPID;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_dev_info("%s: detected aw86928.\n",
				    __func__);
			return 0;
		case AW86937S_CHIPID:
			aw_haptic->chipid = AW86937S_CHIPID;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW8693XS_TRIM_LRA_BOUNDARY;
			aw_dev_info("detected aw86937S.");
			return 0;
		case AW86938S_CHIPID:
			aw_haptic->chipid = AW86938S_CHIPID;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW8693XS_TRIM_LRA_BOUNDARY;
			aw_dev_info("detected aw86938S.");
			return 0;
		default:
			aw_dev_info("%s: unsupport device revision (0x%02X)\n",
				    __func__, reg);
			break;
		}
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}
	return -EINVAL;
}

static int ctrl_init(struct aw_haptic *aw_haptic)
{
	uint32_t reg = 0;
	uint8_t cnt = 0;

	aw_dev_info("%s: enter\n", __func__);
	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		/* hardware reset */
		hw_reset(aw_haptic);
		if (read_chipid(aw_haptic, &reg) < 0)
			aw_dev_err("%s: read chip id fail\n", __func__);
		switch (reg) {
		case AW86905_CHIPID:
		case AW86907_CHIPID:
		case AW86915_CHIPID:
		case AW86917_CHIPID:
			aw_haptic->func = &aw869xx_func_list;
			return 0;
		case AW86925_CHIPID:
		case AW86926_CHIPID:
		case AW86927_CHIPID:
		case AW86928_CHIPID:
			aw_haptic->func = &aw8692x_func_list;
			return 0;
		case AW86937S_CHIPID:
		case AW86938S_CHIPID:
			aw_haptic->func = &aw8693xs_func_list;
			aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_DISABLE;
			return 0;
		default:
			aw_dev_err("%s: unexpected chipid\n", __func__);
			break;
		}
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}
	return -EINVAL;
}

static void ram_play(struct aw_haptic *aw_haptic, uint8_t mode)
{
	aw_dev_dbg("%s: enter\n", __func__);
	aw_haptic->func->play_mode(aw_haptic, mode);
	aw_haptic->func->play_go(aw_haptic, true);
}

static int get_ram_num(struct aw_haptic *aw_haptic)
{
	uint8_t wave_addr[2] = {0};
	uint32_t first_wave_addr = 0;

	aw_dev_dbg("%s: enter!\n", __func__);
	if (!aw_haptic->ram_init) {
		aw_dev_err("%s: ram init faild, ram_num = 0!\n",
			   __func__);
		return -EPERM;
	}
	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	/* RAMINIT Enable */
	aw_haptic->func->ram_init(aw_haptic, true);
#ifdef OPLUS_FEATURE_CHG_BASIC
	aw_haptic->func->set_ram_addr(aw_haptic);
#else
	aw_haptic->func->set_ram_addr(aw_haptic, aw_haptic->ram.base_addr);
#endif
	aw_haptic->func->get_first_wave_addr(aw_haptic, wave_addr);
	first_wave_addr = (wave_addr[0] << 8 | wave_addr[1]);
	aw_haptic->ram.ram_num =
			(first_wave_addr - aw_haptic->ram.base_addr - 1) / 4;
	aw_dev_info("%s: first waveform addr = 0x%04x\n",
		    __func__, first_wave_addr);
	aw_dev_info("%s: ram_num = %d\n",
		    __func__, aw_haptic->ram.ram_num);
	/* RAMINIT Disable */
	aw_haptic->func->ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);
	return 0;
}

static void ram_load(const struct firmware *cont, void *context)
{
	uint16_t check_sum = 0;
	int i = 0;
	int ret = 0;
	struct aw_haptic *aw_haptic = context;
	struct aw_haptic_container *awinic_fw;

#ifdef AW_READ_BIN_FLEXBALLY
	static uint8_t load_cont;
	int ram_timer_val = 1000;

	load_cont++;
#endif
	aw_dev_info("%s: enter\n", __func__);

	if (!cont) {
		aw_dev_err("%s: failed to read ram firmware!\n",
			   __func__);
		release_firmware(cont);
#ifdef AW_READ_BIN_FLEXBALLY
		if (load_cont <= 20) {
			schedule_delayed_work(&aw_haptic->ram_work,
					      msecs_to_jiffies(ram_timer_val));
			aw_dev_info("%s:start hrtimer:load_cont%d\n",
				    __func__, load_cont);
		}
#endif
		return;
	}
	aw_dev_info("%s: loaded ram - size: %zu\n",
		    __func__, cont ? cont->size : 0);
	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];
	if (check_sum != (uint16_t)((cont->data[0] << 8) | (cont->data[1]))) {
		aw_dev_err("%s: check sum err: check_sum=0x%04x\n",
			   __func__, check_sum);
		release_firmware(cont);
		return;
	}
	aw_dev_info("%s: check sum pass : 0x%04x\n",
		    __func__, check_sum);
	aw_haptic->ram.check_sum = check_sum;

	/* aw ram update */
	awinic_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!awinic_fw) {
		release_firmware(cont);
		aw_dev_err("%s: Error allocating memory\n",
			   __func__);
		return;
	}
	awinic_fw->len = cont->size;
	memcpy(awinic_fw->data, cont->data, cont->size);
	release_firmware(cont);
	ret = aw_haptic->func->container_update(aw_haptic, awinic_fw);
	if (ret) {
		aw_dev_err("%s: ram firmware update failed!\n",
			   __func__);
	} else {
		aw_haptic->ram_init = true;
		aw_haptic->ram.len = awinic_fw->len - aw_haptic->ram.ram_shift;
		aw_dev_info("%s: ram firmware update complete!\n", __func__);
		get_ram_num(aw_haptic);
	}
	kfree(awinic_fw);
#ifdef AW_BOOT_OSC_CALI
	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	rtp_osc_cali(aw_haptic);
	rtp_trim_lra_cali(aw_haptic);
#endif
//	rtp_update(aw_haptic);
}

static int ram_update(struct aw_haptic *aw_haptic)
{
	uint8_t index = 0;

	aw_haptic->ram_init = false;
	aw_haptic->rtp_init = false;

	aw_haptic->f0 = haptic_common_get_f0();

	/* get f0 from nvram */
	aw_haptic->haptic_real_f0 = (aw_haptic->f0 / 10);
	aw_dev_info("%s: haptic_real_f0 [%d]\n", __func__, aw_haptic->haptic_real_f0);

	if (aw_haptic->device_id == DEVICE_ID_0832) {
		aw_dev_info("%s:19065 haptic bin name  %s\n", __func__,
			    aw_ram_name_19065[index]);
		return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			aw_ram_name_19065[index], aw_haptic->dev, GFP_KERNEL,
			aw_haptic, ram_load);
	} else if (aw_haptic->device_id == DEVICE_ID_0833) {
		aw_dev_info("%s:19065 haptic bin name  %s\n", __func__,
			    aw_ram_name_19161[index]);
		return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			aw_ram_name_19161[index], aw_haptic->dev, GFP_KERNEL,
			aw_haptic, ram_load);
	} else if (aw_haptic->device_id == DEVICE_ID_81538) {
		if (aw_haptic->vibration_style == HAPTIC_VIBRATION_CRISP_STYLE) {
			aw_dev_info("%s:150Hz haptic bin name  %s\n", __func__,
				    haptic_ram_name_150[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name_150[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		} else if (aw_haptic->vibration_style == HAPTIC_VIBRATION_SOFT_STYLE) {
			aw_dev_info("%s:150Hz haptic bin name  %s\n", __func__,
				    haptic_ram_name_150_soft[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name_150_soft[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		} else {
			aw_dev_info("%s:150Hz haptic bin name  %s\n", __func__,
				    haptic_ram_name_150[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name_150[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		}
	} else if (aw_haptic->device_id == DEVICE_ID_1419) {
		if (aw_haptic->vibration_style == HAPTIC_VIBRATION_CRISP_STYLE) {
			aw_dev_info("%s:205Hz haptic bin name  %s\n", __func__,
				    haptic_ram_name_205[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name_205[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		} else if (aw_haptic->vibration_style == HAPTIC_VIBRATION_SOFT_STYLE) {
			aw_dev_info("%s:205Hz haptic bin name  %s\n", __func__,
				    haptic_ram_name_205_soft[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name_205_soft[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		} else {
			aw_dev_info("%s:205Hz haptic bin name  %s\n", __func__,
				    haptic_ram_name_205[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name_205[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		}
	} else if (aw_haptic->device_id == DEVICE_ID_0816) {
		if (aw_haptic->vibration_style == HAPTIC_VIBRATION_CRISP_STYLE) {
			aw_dev_info("%s:130Hz haptic bin name  %s\n", __func__,
				    haptic_ram_name_130[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name_130[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		} else if (aw_haptic->vibration_style == HAPTIC_VIBRATION_SOFT_STYLE) {
			aw_dev_info("%s:130Hz haptic bin name  %s\n", __func__,
				    haptic_ram_name_130_soft[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name_130_soft[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		} else {
			aw_dev_info("%s:130Hz haptic bin name  %s\n", __func__,
				    haptic_ram_name_130[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name_130[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		}
	} else {
		if (aw_haptic->vibration_style == HAPTIC_VIBRATION_CRISP_STYLE) {
			aw_dev_info("%s:170Hz haptic bin name %s\n", __func__,
				    haptic_ram_name[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		} else if (aw_haptic->vibration_style == HAPTIC_VIBRATION_SOFT_STYLE) {
			aw_dev_info("%s:170Hz soft haptic bin name %s\n", __func__,
				    haptic_ram_name_170_soft[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name_170_soft[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		} else {
			aw_dev_info("%s:haptic bin name  %s\n", __func__,
					haptic_ram_name[index]);
			return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				haptic_ram_name[index], aw_haptic->dev, GFP_KERNEL,
				aw_haptic, ram_load);
		}
	}
	return 0;

}

#ifdef AWINIC_RAM_UPDATE_DELAY
static void ram_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
					     ram_work.work);

	aw_dev_info("%s: enter\n", __func__);
	ram_update(aw_haptic);
}
#endif

static void ram_work_init(struct aw_haptic *aw_haptic)
{
#ifdef AWINIC_RAM_UPDATE_DELAY
	int ram_timer_val = AW_RAM_WORK_DELAY_INTERVAL;

	aw_dev_info("%s: enter\n", __func__);
	INIT_DELAYED_WORK(&aw_haptic->ram_work, ram_work_routine);
	schedule_delayed_work(&aw_haptic->ram_work,
			      msecs_to_jiffies(ram_timer_val));
#else
	ram_update(aw_haptic);
#endif
}

static void ram_vbat_comp(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t temp_gain = 0;

	aw_dev_info("%s: enter\n", __func__);
	if (flag) {
		if (aw_haptic->ram_vbat_comp == AW_RAM_VBAT_COMP_ENABLE) {
			aw_haptic->func->get_vbat(aw_haptic);
			if (aw_haptic->vbat > AW_VBAT_REFER) {
				aw_dev_dbg("%s: not need to vbat compensate!\n",
					   __func__);
				return;
			}
			temp_gain = aw_haptic->gain * AW_VBAT_REFER /
				aw_haptic->vbat;
			if (temp_gain >
			    (128 * AW_VBAT_REFER / AW_VBAT_MIN)) {
				temp_gain = 128 * AW_VBAT_REFER / AW_VBAT_MIN;
				aw_dev_dbg("%s: gain limit=%d\n",
					   __func__, temp_gain);
			}
			aw_haptic->func->set_gain(aw_haptic, temp_gain);
		} else {
			aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
		}
	} else {
		aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	}
}

static void calculate_cali_data(struct aw_haptic *aw_haptic)
{
	char f0_cali_lra = 0;
	int f0_cali_step = 0;
	uint32_t f0_limit = 0;
	uint32_t f0_cali_min = aw_haptic->info.f0_pre *
				(100 - aw_haptic->info.f0_cali_percent) / 100;
	uint32_t f0_cali_max = aw_haptic->info.f0_pre *
				(100 + aw_haptic->info.f0_cali_percent) / 100;

/* max and min limit */
	f0_limit = aw_haptic->f0;
	aw_dev_info("%s: f0_pre = %d, f0_cali_min = %d, f0_cali_max = %d, f0 = %d\n",
			__func__, aw_haptic->info.f0_pre,
			f0_cali_min, f0_cali_max, aw_haptic->f0);

	if ((aw_haptic->f0 < f0_cali_min) ||
		aw_haptic->f0 > f0_cali_max) {
		aw_dev_err("%s: f0 calibration out of range = %d!\n",
				__func__, aw_haptic->f0);
		f0_limit = aw_haptic->info.f0_pre;
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		(void)oplus_haptic_track_fre_cail(HAPTIC_F0_CALI_TRACK, aw_haptic->f0, -ERANGE, "f0 out of range");
#endif
		return;
	}
	aw_dev_info("%s: f0_limit = %d\n", __func__,
			(int)f0_limit);

	/* calculate cali step */
	if (aw_haptic->chipid == AW86937S_CHIPID || aw_haptic->chipid == AW86938S_CHIPID) {
		f0_cali_step = AW8693XS_CALI_DATA_FORMULA(f0_limit, aw_haptic->info.f0_pre, aw_haptic->osc_trim_s);
	} else {
		f0_cali_step = 100000 * ((int)f0_limit - (int)aw_haptic->info.f0_pre) /
			((int)f0_limit * AW_OSC_CALI_ACCURACY);
	}
	aw_dev_info("f0_cali_step = %d", f0_cali_step);
	if (f0_cali_step >= 0) {	/*f0_cali_step >= 0 */
		if (f0_cali_step % 10 >= 5)
			f0_cali_step = aw_haptic->trim_lra_boundary + (f0_cali_step / 10 + 1);
		else
			f0_cali_step = aw_haptic->trim_lra_boundary + f0_cali_step / 10;
	} else {	/* f0_cali_step < 0 */
		if (f0_cali_step % 10 <= -5)
			f0_cali_step = aw_haptic->trim_lra_boundary + (f0_cali_step / 10 - 1);
		else
			f0_cali_step = aw_haptic->trim_lra_boundary + f0_cali_step / 10;
	}
	if (f0_cali_step >= aw_haptic->trim_lra_boundary)
		f0_cali_lra = (char)f0_cali_step - aw_haptic->trim_lra_boundary;
	else
		f0_cali_lra = (char)f0_cali_step + aw_haptic->trim_lra_boundary;
	/* update cali step */
	aw_haptic->f0_cali_data = (int)f0_cali_lra;
	aw_dev_err("f0_cali_data = 0x%02X", aw_haptic->f0_cali_data);
}

static int f0_cali(struct aw_haptic *aw_haptic)
{
	aw_dev_info("%s: enter\n", __func__);
	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	if (aw_haptic->func->get_f0(aw_haptic)) {
		aw_dev_err("%s: get f0 error, user defafult f0\n",
			   __func__);
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		(void)oplus_haptic_track_fre_cail(HAPTIC_F0_CALI_TRACK, aw_haptic->f0,
						  0, "aw_haptic->func->get_f0 is null");
#endif
	} else {
		/* calculate cali step */
		calculate_cali_data(aw_haptic);
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	}
	/* restore standby work mode */
	aw_haptic->func->play_stop(aw_haptic);
	return 0;
}

static void rtp_trim_lra_cali(struct aw_haptic *aw_haptic)
{
	uint32_t lra_trim_code = 0;
	/*0.1 percent below no need to calibrate */
	uint32_t osc_cali_threshold = 10;
	int32_t real_code = 0;
	uint32_t theory_time = 0;
	uint32_t real_time = aw_haptic->microsecond;

	aw_dev_info("%s: enter\n", __func__);

	theory_time = aw_haptic->func->get_theory_time(aw_haptic);
	if (theory_time == real_time) {
		aw_dev_info("%s: theory_time == real_time: %d, no need to calibrate!\n",
			    __func__, real_time);
		return;
	} else if (theory_time < real_time) {
		if ((real_time - theory_time) >
			(theory_time / AW_OSC_TRIM_PARAM)) {
			aw_dev_info("%s: (real_time - theory_time) > (theory_time/50), can't calibrate!\n",
				    __func__);
			return;
		}

		if ((real_time - theory_time) <
		    (osc_cali_threshold * theory_time / 10000)) {
			aw_dev_info("%s: real_time: %d, theory_time: %d, no need to calibrate!\n",
				    __func__, real_time, theory_time);
			return;
		}

		real_code = 100000 * ((real_time - theory_time)) /
			    (theory_time * AW_OSC_CALI_ACCURACY);
		real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
		real_code = aw_haptic->trim_lra_boundary + real_code;
	} else if (theory_time > real_time) {
		if ((theory_time - real_time) >
			(theory_time / AW_OSC_TRIM_PARAM)) {
			aw_dev_info("%s: (theory_time - real_time) > (theory_time / 50), can't calibrate!\n",
				    __func__);
			return;
		}
		if ((theory_time - real_time) <
		    (osc_cali_threshold * theory_time / 10000)) {
			aw_dev_info("%s: real_time: %d, theory_time: %d, no need to calibrate!\n",
				    __func__, real_time, theory_time);
			return;
		}

		real_code = (theory_time - real_time) / (theory_time / 100000) / AW_OSC_CALI_ACCURACY;
		real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
		real_code = aw_haptic->trim_lra_boundary - real_code;
	}
	if (aw_haptic->chipid == AW86937S_CHIPID || aw_haptic->chipid == AW86938S_CHIPID) {
		real_code = (10 * ((long)theory_time * (10000 + aw_haptic->osc_trim_s * AW8693XS_OSC_CALI_ACCURACY) -
						10000 * (long)real_time) / ((long)real_time * AW8693XS_OSC_CALI_ACCURACY));
		aw_dev_info("real_code: %d aw_haptic->osc_trim_s=%d",
				real_code, aw_haptic->osc_trim_s);
		if (real_code >= 0) {	/*f0_cali_step >= 0 */
			if (real_code % 10 >= 5)
				real_code = aw_haptic->trim_lra_boundary + (real_code / 10 + 1);
			else
				real_code = aw_haptic->trim_lra_boundary + real_code / 10;
		} else {	/* f0_cali_step < 0 */
			if (real_code % 10 <= -5)
				real_code = aw_haptic->trim_lra_boundary + (real_code / 10 - 1);
			else
				real_code = aw_haptic->trim_lra_boundary + real_code / 10;
		}
	}

	if (real_code >= aw_haptic->trim_lra_boundary)
		lra_trim_code = real_code - aw_haptic->trim_lra_boundary;
	else
		lra_trim_code = real_code + aw_haptic->trim_lra_boundary;

	aw_dev_info("%s: real_time: %d, theory_time: %d\n",
		    __func__, real_time, theory_time);
	aw_dev_info("%s: real_code: %d, trim_lra: 0x%02X\n",
		    __func__, real_code, lra_trim_code);
	if (lra_trim_code >= 0) {
		aw_haptic->osc_cali_data = lra_trim_code;
		aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_BSP_DRV_VND_INJECT_TEST)
noinline
#endif
static int rtp_osc_cali(struct aw_haptic *aw_haptic)
{
	uint32_t buf_len = 0;
	int ret = -1;
	const struct firmware *rtp_file;

	aw_haptic->rtp_cnt = 0;
	aw_haptic->timeval_flags = 1;

	aw_dev_info("%s: enter\n", __func__);
	/* fw loaded */
	ret = request_firmware(&rtp_file, haptic_rtp_name[0], aw_haptic->dev);
	if (ret < 0) {
		aw_dev_err("%s: failed to read %s\n", __func__,
			   haptic_rtp_name[0]);
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		(void)oplus_haptic_track_fre_cail(HAPTIC_OSC_CALI_TRACK, aw_haptic->f0, ret,
						 "rtp_osc_cali request_firmware fail");
#endif
		return ret;
	}
	/*aw_haptic add stop,for irq interrupt during calibrate */
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->rtp_init = false;
	mutex_lock(&aw_haptic->rtp_lock);
#ifndef OPLUS_FEATURE_CHG_BASIC
	kfree(aw_rtp);
	aw_rtp = kzalloc(rtp_file->size+sizeof(int), GFP_KERNEL);
	if (!aw_rtp) {
		release_firmware(rtp_file);
		mutex_unlock(&aw_haptic->rtp_lock);
		aw_dev_err("%s: error allocating memory\n", __func__);
		return -ENOMEM;
	}
#else
	ret = container_init(rtp_file->size+sizeof(int));
	if (ret < 0) {
		release_firmware(rtp_file);
		mutex_unlock(&aw_haptic->rtp_lock);
		aw_dev_err("%s: error allocating memory\n", __func__);
		return -ENOMEM;
	}
#endif
	aw_rtp->len = rtp_file->size;
	aw_haptic->rtp_len = rtp_file->size;
	aw_dev_info("%s: rtp file:[%s] size = %dbytes\n",
		    __func__, haptic_rtp_name[0], aw_rtp->len);
	memcpy(aw_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);
	mutex_unlock(&aw_haptic->rtp_lock);
	/* gain */
	ram_vbat_comp(aw_haptic, false);
	/* rtp mode config */
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	/* bst mode */
	aw_haptic->func->bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
	disable_irq(gpio_to_irq(aw_haptic->irq_gpio));
	/* haptic go */
	aw_haptic->func->play_go(aw_haptic, true);
	while (1) {
		if (!aw_haptic->func->rtp_get_fifo_afs(aw_haptic)) {
#ifdef AW_ENABLE_RTP_PRINT_LOG
			aw_dev_info("%s: not almost_full, aw_haptic->rtp_cnt=%d\n",
				 __func__, aw_haptic->rtp_cnt);
#endif
			mutex_lock(&aw_haptic->rtp_lock);
			aw_pm_qos_enable(aw_haptic, true);
			if (aw_haptic->rtp_cnt < aw_haptic->ram.base_addr) {
				if (aw_rtp->len - aw_haptic->rtp_cnt < aw_haptic->ram.base_addr)
					buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
				else
					buf_len = aw_haptic->ram.base_addr;
			} else if ((aw_rtp->len - aw_haptic->rtp_cnt) <
			    (aw_haptic->ram.base_addr >> 2))
				buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
			else
				buf_len = (aw_haptic->ram.base_addr >> 2);

			if (aw_haptic->rtp_cnt != aw_rtp->len) {
				if (aw_haptic->timeval_flags == 1) {
					aw_haptic->kstart = ktime_get();
					aw_haptic->timeval_flags = 0;
				}
				aw_haptic->func->set_rtp_data(
						aw_haptic, &aw_rtp->data
						[aw_haptic->rtp_cnt], buf_len);
				aw_haptic->rtp_cnt += buf_len;
			}
			aw_pm_qos_enable(aw_haptic, false);
			mutex_unlock(&aw_haptic->rtp_lock);
		}
		if (aw_haptic->func->get_osc_status(aw_haptic)) {
			aw_haptic->kend = ktime_get();
			aw_dev_info("%s: osc trim playback done aw_haptic->rtp_cnt= %d\n",
				    __func__, aw_haptic->rtp_cnt);
			break;
		}
		aw_haptic->kend = ktime_get();
		aw_haptic->microsecond = ktime_to_us(ktime_sub(aw_haptic->kend,
							    aw_haptic->kstart));
		if (aw_haptic->microsecond > AW_OSC_CALI_MAX_LENGTH) {
			aw_dev_info("%s osc trim time out! aw_haptic->rtp_cnt %d\n",
				    __func__, aw_haptic->rtp_cnt);
			break;
		}
	}
	enable_irq(gpio_to_irq(aw_haptic->irq_gpio));
	aw_haptic->microsecond = ktime_to_us(ktime_sub(aw_haptic->kend,
						       aw_haptic->kstart));
	/*calibration osc */
	aw_dev_info("%s: aw_haptic_microsecond: %ld\n",
		    __func__, aw_haptic->microsecond);
	aw_dev_info("%s: exit\n", __func__);
	return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct aw_haptic *aw_haptic = container_of(timer, struct aw_haptic,
						   timer);

	aw_dev_info("%s: enter\n", __func__);
	aw_haptic->state = 0;
	/* schedule_work(&aw_haptic->vibrator_work); */
	queue_work(system_highpri_wq, &aw_haptic->vibrator_work);
	return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   vibrator_work);

	aw_dev_dbg("%s: enter!\n", __func__);

#ifdef OPLUS_FEATURE_CHG_BASIC
	aw_haptic->activate_mode = AW_RAM_LOOP_MODE;
	aw_dev_info("%s enter, aw_haptic->state[%d], aw_haptic->activate_mode[%d], aw_haptic->ram_vbat_comp[%d]\n",
		    __func__, aw_haptic->state, aw_haptic->activate_mode,
		    aw_haptic->ram_vbat_comp);
#endif

	mutex_lock(&aw_haptic->lock);
	/* Enter standby mode */
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->state) {
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
		if (aw_haptic->activate_mode == AW_RAM_LOOP_MODE) {
			if (aw_haptic->device_id == DEVICE_ID_0832
			    || aw_haptic->device_id == DEVICE_ID_0833
			    || aw_haptic->device_id == DEVICE_ID_0815
			    || aw_haptic->device_id == DEVICE_ID_0809
			    || aw_haptic->device_id == DEVICE_ID_1419
			    || aw_haptic->device_id == DEVICE_ID_0816) {
				ram_vbat_comp(aw_haptic, false);
				aw_haptic->func->bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
			} else {
				ram_vbat_comp(aw_haptic, true);
				aw_haptic->func->bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
			}
			ram_play(aw_haptic, AW_RAM_LOOP_MODE);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer,
				      ktime_set(aw_haptic->duration / 1000,
						(aw_haptic->duration % 1000) *
						1000000), HRTIMER_MODE_REL);
		} else if (aw_haptic->activate_mode == AW_CONT_MODE) {
			aw_haptic->func->cont_config(aw_haptic);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer,
				      ktime_set(aw_haptic->duration / 1000,
						(aw_haptic->duration % 1000) *
						1000000), HRTIMER_MODE_REL);
		} else {
			aw_dev_err("%s: activate_mode error\n",
				   __func__);
		}
	}
	mutex_unlock(&aw_haptic->lock);
}

static void rtp_play(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state_val = 0;
	uint32_t buf_len = 0;

	aw_dev_info("%s: enter\n", __func__);
	aw_haptic->rtp_cnt = 0;
	mutex_lock(&aw_haptic->rtp_lock);
	aw_pm_qos_enable(aw_haptic, true);
	if (aw_haptic->func->dump_rtp_regs)
		aw_haptic->func->dump_rtp_regs(aw_haptic);
	while ((!aw_haptic->func->rtp_get_fifo_afs(aw_haptic))
	       && (aw_haptic->play_mode == AW_RTP_MODE)) {
#ifdef AW_ENABLE_RTP_PRINT_LOG
		aw_dev_info("%s: rtp cnt = %d\n", __func__,
			    aw_haptic->rtp_cnt);
#endif
		if (!aw_rtp) {
			aw_dev_info("%s:aw_rtp is null, break!\n", __func__);
			break;
		}
		if (aw_haptic->rtp_cnt < (aw_haptic->ram.base_addr)) {
			if ((aw_rtp->len - aw_haptic->rtp_cnt) <
			    (aw_haptic->ram.base_addr)) {
				buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
			} else {
				buf_len = aw_haptic->ram.base_addr;
			}
		} else if ((aw_rtp->len - aw_haptic->rtp_cnt) <
			   (aw_haptic->ram.base_addr >> 2)) {
			buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
		} else {
			buf_len = aw_haptic->ram.base_addr >> 2;
		}
#ifdef AW_ENABLE_RTP_PRINT_LOG
		aw_dev_info("%s: buf_len = %d\n", __func__,
			    buf_len);
#endif
		aw_haptic->func->set_rtp_data(aw_haptic,
					      &aw_rtp->data[aw_haptic->rtp_cnt],
					      buf_len);
		aw_haptic->rtp_cnt += buf_len;
		glb_state_val = aw_haptic->func->get_glb_state(aw_haptic);
		if ((aw_haptic->rtp_cnt >= aw_rtp->len)
		    || ((glb_state_val & AW_GLBRD_STATE_MASK) ==
							AW_STATE_STANDBY)) {
			if (aw_haptic->rtp_cnt != aw_rtp->len)
				aw_dev_err("%s: rtp play suspend!\n", __func__);
			else
				aw_dev_info("%s: rtp update complete!\n",
					    __func__);
			aw_haptic->rtp_cnt = 0;
			if (aw_haptic->func->dump_rtp_regs)
				aw_haptic->func->dump_rtp_regs(aw_haptic);
			break;
		}
	}

	if (aw_haptic->play_mode == AW_RTP_MODE)
		aw_haptic->func->set_rtp_aei(aw_haptic, true);
	aw_pm_qos_enable(aw_haptic, false);
	aw_dev_info("%s: exit\n", __func__);
	mutex_unlock(&aw_haptic->rtp_lock);
}

static void op_clean_status(struct aw_haptic *aw_haptic)
{
	aw_haptic->audio_ready = false;
	aw_haptic->haptic_ready = false;
	aw_haptic->pre_haptic_number = 0;
	aw_haptic->rtp_routine_on = 0;

	aw_dev_info("%s enter\n", __func__);
}

static void rtp_work_routine(struct work_struct *work)
{
	bool rtp_work_flag = false;
	uint8_t reg_val = 0;
	int cnt = 200;
	int ret = -1;
	const struct firmware *rtp_file;
	const char* rtp_name = NULL;
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   rtp_work);

	aw_dev_info("%s: enter device_id = %d, f0 = %d\n", __func__, aw_haptic->device_id, aw_haptic->f0);
	mutex_lock(&aw_haptic->rtp_lock);
	aw_haptic->rtp_routine_on = 1;
	/* fw loaded */

	rtp_file = rtp_load_file_accord_f0(aw_haptic->rtp_file_num);
	if (!rtp_file) {
		aw_haptic->rtp_routine_on = 1;
		rtp_name = get_rtp_name(aw_haptic->rtp_file_num, aw_haptic->f0);
		if (!rtp_name) {
				aw_dev_info("%s: get rtp name failed.\n", __func__);
				mutex_unlock(&aw_haptic->rtp_lock);
				return;
		}
		ret = request_firmware(&rtp_file, rtp_name, aw_haptic->dev);
		aw_dev_info("%s line:%d: rtp_num:%d name:%s, f0 = %d\n", __func__, __LINE__,
					aw_haptic->rtp_file_num, rtp_name, aw_haptic->f0);
		vfree(rtp_name);
		if (ret < 0) {
			aw_dev_err("%s: failed to read %d, aw_haptic->f0=%d\n",
				   __func__,
				   aw_haptic->rtp_file_num,
				   aw_haptic->f0);
			aw_haptic->rtp_routine_on = 0;
			mutex_unlock(&aw_haptic->rtp_lock);
			return;
		}
	}
	aw_haptic->rtp_init = false;
#ifndef OPLUS_FEATURE_CHG_BASIC
	vfree(aw_rtp);
	aw_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw_rtp) {
		release_firmware(rtp_file);
		aw_dev_err("%s: error allocating memory\n",
			   __func__);
		aw_haptic->rtp_routine_on = 0;
		mutex_unlock(&aw_haptic->rtp_lock);
		return;
	}
#else
	ret = container_init(rtp_file->size + sizeof(int));
	if (ret < 0) {
		release_firmware(rtp_file);
		mutex_unlock(&aw_haptic->rtp_lock);
		aw_dev_err("%s: error allocating memory\n", __func__);

		op_clean_status(aw_haptic);
		aw_haptic->rtp_routine_on = 0;
		return;
	}
#endif
	aw_rtp->len = rtp_file->size;
	aw_dev_info("%s: rtp file:[%s] size = %dbytes f0 = %d\n",
		    __func__, rtp_wave_map[aw_haptic->rtp_file_num],
		    aw_rtp->len, aw_haptic->f0);
	memcpy(aw_rtp->data, rtp_file->data, rtp_file->size);
	mutex_unlock(&aw_haptic->rtp_lock);
	release_firmware(rtp_file);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->rtp_init = true;

	aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	aw_haptic->func->set_rtp_aei(aw_haptic, false);
	aw_haptic->func->irq_clear(aw_haptic);
	aw_haptic->func->play_stop(aw_haptic);

	if (aw_haptic->rtp_file_num == HAPTIC_WAVEFORM_INDEX_ZERO) {
		aw_haptic->rtp_init = false;
		op_clean_status(aw_haptic);
		aw_dev_info("%s: vibrator stopped \n", __func__);
		mutex_unlock(&aw_haptic->lock);
		return;
	}

	/* gain */
	ram_vbat_comp(aw_haptic, false);
	/* boost voltage */
	/*
	if (aw_haptic->info.bst_vol_rtp <= aw_haptic->info.max_bst_vol &&
		aw_haptic->info.bst_vol_rtp > 0)
		aw_haptic->func->set_bst_vol(aw_haptic,
					   aw_haptic->info.bst_vol_rtp);
	else
		aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
	*/
	/* rtp mode config */
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	/* haptic go */
	aw_haptic->func->play_go(aw_haptic, true);
	usleep_range(2000, 2500);
	while (cnt) {
		reg_val = aw_haptic->func->get_glb_state(aw_haptic);
		if ((reg_val & AW_GLBRD_STATE_MASK) == AW_STATE_RTP) {
			cnt = 0;
			rtp_work_flag = true;
			aw_dev_info("%s: RTP_GO! glb_state=0x08\n", __func__);
		} else {
			cnt--;
			aw_dev_dbg("%s: wait for RTP_GO, glb_state=0x%02X\n",
				   __func__, reg_val);
		}
		usleep_range(2000, 2500);
	}
	if (rtp_work_flag) {
		rtp_play(aw_haptic);
	} else {
		/* enter standby mode */
		aw_haptic->func->play_stop(aw_haptic);
		aw_dev_err("%s: failed to enter RTP_GO status!\n", __func__);
	}
	op_clean_status(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
}

static int aw_interrupt_init(void *chip_data)
{

    aw_haptic_t *aw_haptic = (aw_haptic_t *)chip_data;
	aw_haptic->func->interrupt_setup(aw_haptic);
	return 0;
}

static irqreturn_t irq_handle(int irq, void *data)
{
	uint8_t glb_state_val = 0;
	uint32_t buf_len = 0;
	struct aw_haptic *aw_haptic = data;

	aw_dev_dbg("%s: enter\n", __func__);

#ifdef AAC_RICHTAP
	if (aw_haptic->haptic_rtp_mode) {
		aw_dev_info("exit %s:aw_haptic->haptic_rtp_mode = %d\n",
				__func__, aw_haptic->haptic_rtp_mode);
		return IRQ_HANDLED;
	}
#endif

	if (!aw_haptic->func->get_irq_state(aw_haptic)) {
		aw_dev_dbg("%s: aw_haptic rtp fifo almost empty\n", __func__);
		if (aw_haptic->rtp_init) {
			while ((!aw_haptic->func->rtp_get_fifo_afs(aw_haptic))
			       && (aw_haptic->play_mode == AW_RTP_MODE)) {
				mutex_lock(&aw_haptic->rtp_lock);
				aw_pm_qos_enable(aw_haptic, true);
				if (!aw_haptic->rtp_cnt) {
					aw_dev_info("%s:aw_haptic->rtp_cnt is 0!\n",
						    __func__);
					aw_pm_qos_enable(aw_haptic, false);
					mutex_unlock(&aw_haptic->rtp_lock);
					break;
				}
#ifdef AW_ENABLE_RTP_PRINT_LOG
				aw_dev_info("%s:rtp mode fifo update, cnt=%d\n",
					    __func__, aw_haptic->rtp_cnt);
#endif
				if (!aw_rtp) {
					aw_dev_info("%s:aw_rtp is null, break!\n",
						    __func__);
					aw_pm_qos_enable(aw_haptic, false);
					mutex_unlock(&aw_haptic->rtp_lock);
					break;
				}
				if ((aw_rtp->len - aw_haptic->rtp_cnt) <
				    (aw_haptic->ram.base_addr >> 2)) {
					buf_len =
					    aw_rtp->len - aw_haptic->rtp_cnt;
				} else {
					buf_len = (aw_haptic->ram.base_addr >>
						   2);
				}
				aw_haptic->func->set_rtp_data(aw_haptic,
						     &aw_rtp->data
						     [aw_haptic->rtp_cnt],
						     buf_len);
				aw_haptic->rtp_cnt += buf_len;
				glb_state_val =
				      aw_haptic->func->get_glb_state(aw_haptic);
				if ((aw_haptic->rtp_cnt >= aw_rtp->len)
				    || ((glb_state_val & AW_GLBRD_STATE_MASK) ==
							AW_STATE_STANDBY)) {
					if (aw_haptic->rtp_cnt !=
					    aw_rtp->len)
						aw_dev_err("%s: rtp play suspend!\n",
							   __func__);
					else
						aw_dev_info("%s: rtp update complete!\n",
							    __func__);
					op_clean_status(aw_haptic);
					aw_haptic->func->set_rtp_aei(aw_haptic,
								     false);
					aw_haptic->rtp_cnt = 0;
					aw_haptic->rtp_init = false;
					aw_pm_qos_enable(aw_haptic, false);
					mutex_unlock(&aw_haptic->rtp_lock);
					break;
				}
				aw_pm_qos_enable(aw_haptic, false);
				mutex_unlock(&aw_haptic->rtp_lock);
			}
		} else {
			aw_dev_info("%s: init error\n",
				    __func__);
		}
	}
	if (aw_haptic->play_mode != AW_RTP_MODE)
		aw_haptic->func->set_rtp_aei(aw_haptic, false);
	aw_dev_dbg("%s: exit\n", __func__);
	return IRQ_HANDLED;
}

static ssize_t aw_state_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw_haptic->state);
}

static ssize_t aw_state_store(void *chip_data, const char *buf)
{
	return 0;
}

static ssize_t aw_duration_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw_haptic->timer)) {
		time_rem = hrtimer_get_remaining(&aw_haptic->timer);
		time_ms = ktime_to_ms(time_rem);
	}
	return snprintf(buf, PAGE_SIZE, "%lldms\n", time_ms);
}

static ssize_t aw_duration_store(void *chip_data, const char *buf, uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return val;
	aw_haptic->duration = val;
	return 0;
}

static ssize_t aw_activate_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw_haptic->state);
}

static ssize_t aw_activate_store(void *chip_data, const char *buf, uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;
	int rtp_is_going_on = 0;

	rtp_is_going_on = aw_haptic->func->juge_rtp_going(aw_haptic);
	if (rtp_is_going_on) {
		aw_dev_info("%s: rtp is going\n", __func__);
		return -EINVAL;
	}

	if (!aw_haptic->ram_init) {
		aw_dev_err("%s: ram init failed, not allow to play!\n",
			   __func__);
		return -EINVAL;
	}
	mutex_lock(&aw_haptic->lock);
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->state = val;
	mutex_unlock(&aw_haptic->lock);
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (aw_haptic->state) {
		aw_dev_info("%s: gain=0x%02x\n", __func__, aw_haptic->gain);
		if (aw_haptic->gain >= HAPTIC_RAM_VBAT_COMP_GAIN)
			aw_haptic->gain = HAPTIC_RAM_VBAT_COMP_GAIN;

		mutex_lock(&aw_haptic->lock);

		if (aw_haptic->device_id == DEVICE_ID_0815 ||
		    aw_haptic->device_id == DEVICE_ID_0809 ||
		    aw_haptic->device_id == DEVICE_ID_81538 ||
		    aw_haptic->device_id == DEVICE_ID_1419 ||
		    aw_haptic->device_id == DEVICE_ID_0816)
			aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
		//aw_haptic->func->set_repeat_seq(aw_haptic,
		//				HAPTIC_WAVEFORM_INDEX_SINE_CYCLE);
		haptic_set_ftm_wave();
		aw_haptic->func->set_wav_loop(aw_haptic, 0, 0x0F);
		mutex_unlock(&aw_haptic->lock);
		cancel_work_sync(&aw_haptic->vibrator_work);
		queue_work(system_highpri_wq, &aw_haptic->vibrator_work);
	} else {
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->play_stop(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
	}
#endif
	return 0;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static ssize_t oplus_brightness_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw_haptic->amplitude);
}

static ssize_t oplus_brightness_store(void *chip_data, const char *buf ,uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;

	if (!aw_haptic->ram_init) {
		aw_dev_err("%s: ram init failed, not allow to play!\n",
		__func__);
		return -EINVAL;
	}
	aw_haptic->amplitude = val;
	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->amplitude > 0) {
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
		ram_vbat_comp(aw_haptic, false);
		ram_play(aw_haptic, AW_RAM_MODE);
	}
	mutex_unlock(&aw_haptic->lock);

	return 0;
}
#endif

static ssize_t aw_activate_mode_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "activate_mode = %d\n",
			aw_haptic->activate_mode);
}

static ssize_t aw_activate_mode_store(void *chip_data, const char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->activate_mode = val;
	mutex_unlock(&aw_haptic->lock);
	return 0;
}

static ssize_t aw_index_show(void *chip_data, char *buf)
{
	ssize_t count = 0;
	aw_haptic_t *aw_haptic = chip_data;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_wav_seq(aw_haptic, 1);
	aw_haptic->index = aw_haptic->seq[0];
	mutex_unlock(&aw_haptic->lock);
	count += snprintf(buf, PAGE_SIZE, "%d\n", aw_haptic->index);
	return count;
}

static ssize_t aw_index_store(void *chip_data, const char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val > aw_haptic->ram.ram_num) {
		aw_dev_err("%s: input value out of range!\n", __func__);
		return -EINVAL;
	}
	aw_dev_info("%s: value=%d\n", __func__, val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->index = val;
	aw_haptic->func->set_repeat_seq(aw_haptic, aw_haptic->index);
	mutex_unlock(&aw_haptic->lock);
	return 0;
}

static ssize_t aw_vmax_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw_haptic->vmax);
}

static ssize_t aw_vmax_store(void *chip_data, const char *buf, uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;
	struct vmax_map map;

	mutex_lock(&aw_haptic->lock);
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (val <= HAPTIC_MAX_LEVEL) {
		val = val / 100 * 100;
		aw_haptic->func->convert_level_to_vmax(aw_haptic, &map, val);
		aw_haptic->vmax = map.vmax;
		aw_haptic->gain = map.gain;
	} else {
		aw_haptic->vmax = aw_haptic->max_boost_vol;
		aw_haptic->gain = 0x80;
	}

	if (val == HAPTIC_OLD_TEST_LEVEL) {  /* for old test only */
		aw_haptic->gain = HAPTIC_RAM_VBAT_COMP_GAIN;
	}

	if (aw_haptic->device_id == DEVICE_ID_0833) {
		aw_haptic->vmax = aw_haptic->max_boost_vol;
		aw_haptic->gain = 0x80;
	}

	if (vbat_low_soc_flag() && (aw_haptic->vbat_low_vmax_level != 0) && (val > aw_haptic->vbat_low_vmax_level)) {
		aw_haptic->func->convert_level_to_vmax(aw_haptic, &map, aw_haptic->vbat_low_vmax_level);
		aw_haptic->vmax = map.vmax;
		aw_haptic->gain = map.gain;
	}

	aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
#else
	aw_haptic->vmax = val;
	aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
#endif
	mutex_unlock(&aw_haptic->lock);
	aw_dev_info("%s: gain[0x%x], vmax[0x%x] end\n", __func__,
		    aw_haptic->gain, aw_haptic->vmax);

	return 0;
}

static ssize_t aw_gain_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", aw_haptic->gain);
}

static ssize_t aw_gain_store(void *chip_data, const char *buf, uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->gain = val;
	aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	mutex_unlock(&aw_haptic->lock);
	return 0;
}

static ssize_t aw_seq_show(void *chip_data, char *buf)
{
	size_t count = 0;
	int i = 0;
	aw_haptic_t *aw_haptic = chip_data;
	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_wav_seq(aw_haptic, AW_SEQUENCER_SIZE);
	mutex_unlock(&aw_haptic->lock);
	for (i = 0; i < AW_SEQUENCER_SIZE; i++) {
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d = %d\n", i + 1, aw_haptic->seq[i]);
	}
	return count;
}

static ssize_t aw_seq_store(void *chip_data, const char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		if (databuf[0] >= AW_SEQUENCER_SIZE ||
		    databuf[1] > aw_haptic->ram.ram_num) {
			aw_dev_err("%s: input value out of range!\n", __func__);
			return -EINVAL;
		}
		aw_dev_info("%s: seq%d=0x%02X\n", __func__,
			    databuf[0], databuf[1]);
		mutex_lock(&aw_haptic->lock);
		aw_haptic->seq[databuf[0]] = (uint8_t)databuf[1];
		aw_haptic->func->set_wav_seq(aw_haptic, (uint8_t)databuf[0],
					     aw_haptic->seq[databuf[0]]);
		mutex_unlock(&aw_haptic->lock);
	}
	return 0;
}

static ssize_t aw_loop_show(void *chip_data, char *buf)
{
	size_t count = 0;
	aw_haptic_t *aw_haptic = chip_data;

	mutex_lock(&aw_haptic->lock);
	count = aw_haptic->func->get_wav_loop(aw_haptic, buf);
	mutex_unlock(&aw_haptic->lock);
	count += snprintf(buf+count, PAGE_SIZE-count,
 			  "rtp_loop: 0x%02x\n", aw_haptic->rtp_loop);
	return count;
}

static ssize_t aw_loop_store(void *chip_data, const char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	uint32_t databuf[2] = { 0, 0 };
	uint32_t val = 0;
	int rc = 0;

	aw_haptic->rtp_loop = 0;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_dev_info("%s: seq%d loop=0x%02X\n", __func__,
			    databuf[0], databuf[1]);
		mutex_lock(&aw_haptic->lock);
		aw_haptic->loop[databuf[0]] = (uint8_t)databuf[1];
		aw_haptic->func->set_wav_loop(aw_haptic, (uint8_t)databuf[0],
					      aw_haptic->loop[databuf[0]]);
		mutex_unlock(&aw_haptic->lock);
	} else {
		rc = kstrtouint(buf, 0, &val);
		if (rc < 0)
			return -EINVAL;
		aw_haptic->rtp_loop = val;
		aw_dev_info("%s: rtp_loop = 0x%02X", __func__,
			    aw_haptic->rtp_loop);
	}

	return 0;
}

static ssize_t aw_reg_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	len = aw_haptic->func->get_reg(aw_haptic, len, buf);
	mutex_unlock(&aw_haptic->lock);
	return len;
}

static ssize_t aw_reg_store(void *chip_data, const char *buf)
{
	uint8_t val = 0;
	aw_haptic_t *aw_haptic = chip_data;
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		val = (uint8_t)databuf[1];
		if (aw_haptic->func == &aw8692x_func_list &&
		    (uint8_t)databuf[0] == AW8692X_REG_ANACFG20)
			val &= AW8692X_BIT_ANACFG20_TRIM_LRA;
		mutex_lock(&aw_haptic->lock);
		i2c_w_bytes(aw_haptic, (uint8_t)databuf[0], &val,
			    AW_I2C_BYTE_ONE);
		mutex_unlock(&aw_haptic->lock);
	}
	return 0;
}

static ssize_t aw_rtp_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp_cnt = %d\n",
			aw_haptic->rtp_cnt);
	return len;
}

static ssize_t aw_rtp_store(void *chip_data, const char *buf, uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;
	int rtp_is_going_on = 0;

#ifdef AAC_RICHTAP
	if (aw_haptic->haptic_rtp_mode) {
		aw_dev_info("exit %s:aw_haptic->haptic_rtp_mode = %d\n",
				__func__, aw_haptic->haptic_rtp_mode);
		return -EINVAL;
	}
#endif

	if (val == 1025 || val == 1026)
		return -EINVAL;

	mutex_lock(&aw_haptic->lock);
	/*OP add for juge rtp on begin*/
	rtp_is_going_on = aw_haptic->func->juge_rtp_going(aw_haptic);
	if (rtp_is_going_on && (val == AUDIO_READY_STATUS)) {
		aw_dev_info("%s: seem audio status rtp[%d]\n", __func__, val);
		mutex_unlock(&aw_haptic->lock);
		return -EINVAL;
	}
	/*OP add for juge rtp on end*/
	if (get_ringtone_support(val)) {
		if (val == AUDIO_READY_STATUS)
			aw_haptic->audio_ready = true;
		else
			aw_haptic->haptic_ready = true;

		aw_dev_info("%s:audio[%d]and haptic[%d] ready\n", __func__,
			    aw_haptic->audio_ready, aw_haptic->haptic_ready);

		if (aw_haptic->haptic_ready && !aw_haptic->audio_ready)
			aw_haptic->pre_haptic_number = val;

		if (!aw_haptic->audio_ready || !aw_haptic->haptic_ready) {
			mutex_unlock(&aw_haptic->lock);
			return -EINVAL;
		}
	}
	if (val == AUDIO_READY_STATUS && aw_haptic->pre_haptic_number) {
		aw_dev_info("pre_haptic_number:%d\n",
			    aw_haptic->pre_haptic_number);
		val = aw_haptic->pre_haptic_number;
	}
	if (!val) {
		op_clean_status(aw_haptic);
		aw_haptic->func->play_stop(aw_haptic);
		aw_haptic->func->set_rtp_aei(aw_haptic, false);
		aw_haptic->func->irq_clear(aw_haptic);
	}

	mutex_unlock(&aw_haptic->lock);
	if (val < NUM_WAVEFORMS) {
		aw_haptic->rtp_file_num = val;
		if (val) {
			if (get_rtp_key_support(val)) {
				queue_work(system_unbound_wq, &aw_haptic->rtp_key_work);
			} else {
				queue_work(system_unbound_wq, &aw_haptic->rtp_work);
			}
		}

	} else {
		aw_dev_err("%s: rtp_file_num 0x%02x over max value \n",
			   __func__, aw_haptic->rtp_file_num);
	}
	return 0;
}

static ssize_t aw_ram_update_show(void *chip_data, char *buf)
{
	int i = 0;
	ssize_t len = 0;
	aw_haptic_t *aw_haptic = chip_data;
	uint8_t *ram_buf = NULL;
	aw_dev_info("ram len = %d", aw_haptic->ram.len);
	ram_buf = kzalloc(aw_haptic->ram.len, GFP_KERNEL);
	if (!ram_buf) {
		aw_dev_err("Error allocating memory");
		return len;
	}

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	/* RAMINIT Enable */
	aw_haptic->func->ram_init(aw_haptic, true);
	aw_haptic->func->set_ram_addr(aw_haptic);
	aw_haptic->func->get_ram_data(aw_haptic, ram_buf);
	for (i = 1; i <= aw_haptic->ram.len; i++) {
		len += snprintf(buf + len, PAGE_SIZE, "0x%02x,", *(ram_buf + i - 1));
		if (i % 16 == 0 || i == aw_haptic->ram.len) {
			len = 0;
			aw_dev_info("%s", buf);
		}
	}
	kfree(ram_buf);
	/* RAMINIT Disable */
	aw_haptic->func->ram_init(aw_haptic, false);
	len = snprintf(buf, PAGE_SIZE, "Please check log\n");
	mutex_unlock(&aw_haptic->lock);
	return len;
}

static ssize_t aw_ram_update_store(void *chip_data, const char *buf ,uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;

	if (val)
		ram_update(aw_haptic);
	return 0;
}

static ssize_t aw_f0_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	aw_haptic->func->get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", aw_haptic->f0);
	return len;
}

static ssize_t aw_f0_store(void *chip_data, const char *buf ,uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;

	aw_haptic->f0 = val;
	ram_update(aw_haptic);

	return 0;
}

static ssize_t aw_cali_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	aw_haptic->func->get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"%d\n", aw_haptic->f0);
	return len;
}

static ssize_t aw_cali_store(void *chip_data, const char *buf, uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;

	if (val) {
		mutex_lock(&aw_haptic->lock);
		f0_cali(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
	}
	return 0;
}

static ssize_t aw_lra_resistance_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_lra_resistance(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",
			aw_haptic->lra);
	return len;
}

static ssize_t aw_ram_vbat_comp_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"ram_vbat_comp = %d\n",
			aw_haptic->ram_vbat_comp);

	return len;
}

static ssize_t aw_ram_vbat_comp_store(void *chip_data, const char *buf, uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;

	mutex_lock(&aw_haptic->lock);
	if (val)
		aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE;
	else
		aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_DISABLE;
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static ssize_t aw_osc_cali_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	ssize_t len = 0;

	aw_dev_info("microsecond:%ld \n", aw_haptic->microsecond);
	len += snprintf(buf+len, PAGE_SIZE-len, "%ld\n",
			aw_haptic->microsecond);
	return len;
}

static ssize_t aw_osc_cali_store(void *chip_data, const char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	if (val == 3) {
		aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
		rtp_osc_cali(aw_haptic);
		rtp_trim_lra_cali(aw_haptic);
	} else if (val == 1) {
		aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
		rtp_osc_cali(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static ssize_t aw_gun_type_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw_haptic->gun_type);
}

static ssize_t aw_gun_type_store(void *chip_data, const char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw_dev_dbg("%s: value=%d\n", __func__, val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->gun_type = val;
	mutex_unlock(&aw_haptic->lock);
	return 0;
}

static ssize_t aw_bullet_nr_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw_haptic->bullet_nr);
}

static ssize_t aw_bullet_nr_store(void *chip_data, const char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_dev_dbg("%s: value=%d\n", __func__, val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->bullet_nr = val;
	mutex_unlock(&aw_haptic->lock);
	return 0;
}

static ssize_t aw_f0_data_show(void *chip_data, char *buf)
{
	ssize_t len = 0;
	aw_haptic_t *aw_haptic = chip_data;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",
			aw_haptic->f0_cali_data);

	return len;
}

static ssize_t aw_f0_data_store(void *chip_data, const char *buf)
{
	uint32_t val = 0;
	int rc = 0;
	aw_haptic_t *aw_haptic = chip_data;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	aw_haptic->f0_cali_data = val;
	aw_dev_info("%s: f0_cali_data = %d\n", __func__, aw_haptic->f0_cali_data);
	if (aw_haptic->f0_cali_data == 0) {
		calculate_cali_data(aw_haptic);
	}
	aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	mutex_unlock(&aw_haptic->lock);
	return 0;
}

static ssize_t aw_osc_data_show(void *chip_data, char *buf)
{
	ssize_t len = 0;
	aw_haptic_t *aw_haptic = chip_data;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",
			aw_haptic->osc_cali_data);

	return len;
}

static ssize_t aw_osc_data_store(void *chip_data, const char *buf)
{
	uint32_t val = 0;
	int rc = 0;
	aw_haptic_t *aw_haptic = chip_data;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	aw_haptic->osc_cali_data = val;
	aw_dev_info("%s: osc_cali_data = %d\n", __func__, aw_haptic->osc_cali_data);
	aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	mutex_unlock(&aw_haptic->lock);
	return 0;
}

static ssize_t aw_waveform_index_show(void *chip_data, char *buf)
{
	return 0;
}

static ssize_t aw_waveform_index_store(void *chip_data, const char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	unsigned int databuf[1] = {0};

	if (aw_haptic->device_id == DEVICE_ID_0833) {
		mutex_lock(&aw_haptic->lock);
		aw_haptic->vmax = aw_haptic->max_boost_vol;
		aw_haptic->gain = 0x80;
		aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
		aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
		mutex_unlock(&aw_haptic->lock);
	}

	if (1 == sscanf(buf, "%d", &databuf[0])) {
		aw_dev_err("%s: waveform_index = %d\n", __func__, databuf[0]);
		mutex_lock(&aw_haptic->lock);
		aw_haptic->seq[0] = (unsigned char)databuf[0];
		aw_haptic->func->set_wav_seq(aw_haptic, 0, aw_haptic->seq[0]);
		aw_haptic->func->set_wav_seq(aw_haptic, 1, 0);
		aw_haptic->func->set_wav_loop(aw_haptic, 0, 0);
		mutex_unlock(&aw_haptic->lock);
	}
	return 0;
}

static ssize_t aw_ram_test_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	ssize_t len = 0;
	unsigned int ram_test_result = 0;

	if (aw_haptic->ram_test_flag_0 != 0 ||
	    aw_haptic->ram_test_flag_1 != 0) {
		ram_test_result = 1; /* failed */
		len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", ram_test_result);
	} else {
		ram_test_result = 0; /* pass */
		len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", ram_test_result);
	}
	return len;
}

static ssize_t aw_ram_test_store(void *chip_data, const char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	struct aw_haptic_container *aw_ramtest;
	int i, j = 0;
	int rc = 0;
	unsigned int val = 0;
	unsigned int tmp_len, retries;
	char *pbuf = NULL;

	aw_dev_info("%s enter\n", __func__);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_haptic->ram_test_flag_0 = 0;
	aw_haptic->ram_test_flag_1 = 0;
	tmp_len = 1024 ;  /* 1K */
	retries = 8;  /* tmp_len * retries = 8 * 1024 */
	aw_ramtest = kzalloc(tmp_len * sizeof(char) + sizeof(int), GFP_KERNEL);
	if (!aw_ramtest) {
		aw_dev_err("%s: error allocating memory\n", __func__);
		return -EINVAL;
	}
	pbuf = kzalloc(tmp_len * sizeof(char), GFP_KERNEL);
	if (!pbuf) {
		aw_dev_err("%s: Error allocating memory\n", __func__);
		kfree(aw_ramtest);
		return -EINVAL;
	}
	aw_ramtest->len = tmp_len;

	if (val == 1) {
		mutex_lock(&aw_haptic->lock);
		/* RAMINIT Enable */
		aw_haptic->func->ram_init(aw_haptic, true);
		for (j = 0; j < retries; j++) {
			/*test 1-----------start*/
			memset(aw_ramtest->data, 0xff, aw_ramtest->len);
			memset(pbuf, 0x00, aw_ramtest->len);
			/* write ram 1 test */
			aw_haptic->func->set_ram_addr(aw_haptic);
			aw_haptic->func->set_ram_data(aw_haptic,
						      aw_ramtest->data,
						      aw_ramtest->len);

			/* read ram 1 test */
			aw_haptic->func->set_ram_addr(aw_haptic);
			aw_haptic->func->get_ram_data(aw_haptic, pbuf);

			for (i = 0; i < aw_ramtest->len; i++) {
				if (pbuf[i] != 0xff)
					aw_haptic->ram_test_flag_1++;
			}
			 /*test 1------------end*/

			/*test 0----------start*/
			memset(aw_ramtest->data, 0x00, aw_ramtest->len);
			memset(pbuf, 0xff, aw_ramtest->len);

			/* write ram 0 test */
			aw_haptic->func->set_ram_addr(aw_haptic);
			aw_haptic->func->set_ram_data(aw_haptic,
						      aw_ramtest->data,
						      aw_ramtest->len);
			/* read ram 0 test */
			aw_haptic->func->set_ram_addr(aw_haptic);
			aw_haptic->func->get_ram_data(aw_haptic, pbuf);
			for (i = 0; i < aw_ramtest->len; i++) {
				if (pbuf[i] != 0)
					 aw_haptic->ram_test_flag_0++;
			}
			/*test 0 end*/
		}
		/* RAMINIT Disable */
		aw_haptic->func->ram_init(aw_haptic, false);
		mutex_unlock(&aw_haptic->lock);
	}
	kfree(aw_ramtest);
	kfree(pbuf);
	pbuf = NULL;
	aw_dev_info("%s exit\n", __func__);
	return 0;
}

static ssize_t aw_device_id_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw_haptic->device_id);
}

static ssize_t aw_device_id_store(void *chip_data, const char *buf)
{
	return 0;
}

static ssize_t aw_livetap_support_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw_haptic->livetap_support);
}

static ssize_t aw_livetap_support_store(void *chip_data, const char *buf, int val)
{
	aw_haptic_t *aw_haptic = chip_data;

	if (val > 0)
		aw_haptic->livetap_support = true;
	else
		aw_haptic->livetap_support = false;

	return 0;
}

static ssize_t aw_rtp_going_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;
	ssize_t len = 0;
	int val = -1;

	mutex_lock(&aw_haptic->lock);
	val = aw_haptic->func->juge_rtp_going(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", val);
	return len;
}

static ssize_t aw_rtp_going_store(void *chip_data, const char *buf)
{
	return 0;
}

static ssize_t aw_gun_mode_show(void *chip_data, char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw_haptic->gun_mode);
}
static ssize_t aw_gun_mode_store(void *chip_data, const char *buf)
{
	aw_haptic_t *aw_haptic = chip_data;

	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw_dev_dbg("%s: value=%d\n", __func__, val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->gun_mode = val;
	mutex_unlock(&aw_haptic->lock);
	return 0;
}

static void rtp_key_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, rtp_key_work);
	uint8_t *aw_haptic_rtp_key_data = NULL;
	uint32_t aw_haptic_rtp_key_data_len = 0;
	bool rtp_work_flag = false;
	uint8_t reg_val = 0;
	int cnt = 200;

	aw_haptic->rtp_init = false;
	mutex_lock(&aw_haptic->rtp_lock);

	aw_haptic_rtp_key_data = get_rtp_key_data(&aw_haptic_rtp_key_data_len);
	if (aw_haptic_rtp_key_data == NULL)
			goto undef_rtp;

#ifndef OPLUS_FEATURE_CHG_BASIC
	kfree(aw_rtp);
	aw_rtp = kzalloc(aw_haptic_rtp_key_data_len + sizeof(int), GFP_KERNEL);
	if (!aw_rtp) {
		mutex_unlock(&aw_haptic->rtp_lock);//vincent
		aw_dev_err("%s: error allocating memory\n", __func__);
		return;
	}
#else
	if (container_init(aw_haptic_rtp_key_data_len + sizeof(int)) < 0) {
		mutex_unlock(&aw_haptic->rtp_lock);
		aw_dev_err("%s: error allocating memory\n", __func__);
		return;
	}
#endif
	aw_rtp->len = aw_haptic_rtp_key_data_len;
	memcpy(aw_rtp->data, aw_haptic_rtp_key_data, aw_haptic_rtp_key_data_len);
	mutex_unlock(&aw_haptic->rtp_lock);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->rtp_init = true;
	aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	aw_haptic->func->set_rtp_aei(aw_haptic, false);
	aw_haptic->func->irq_clear(aw_haptic);
	aw_haptic->func->play_stop(aw_haptic);
	/* gain */
	ram_vbat_comp(aw_haptic, false);
	/* rtp mode config */
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	/* haptic go */
	aw_haptic->func->play_go(aw_haptic, true);
	mdelay(1);
	while (cnt) {
		reg_val = aw_haptic->func->get_glb_state(aw_haptic);
		if ((reg_val & AW_GLBRD_STATE_MASK) == AW_STATE_RTP) {
			cnt = 0;
			rtp_work_flag = true;
			aw_dev_info("%s: RTP_GO! glb_state=0x08\n", __func__);
		} else {
			cnt--;
			usleep_range(2000, 2500);
			aw_dev_dbg("%s: wait for RTP_GO, glb_state=0x%02X\n",
				   __func__, reg_val);
		}
	}
	if (rtp_work_flag) {
		rtp_play(aw_haptic);
	} else {
		/* enter standby mode */
		aw_haptic->func->play_stop(aw_haptic);
		aw_dev_err("%s: failed to enter RTP_GO status!\n", __func__);
	}
	op_clean_status(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	return;
undef_rtp:
	mutex_unlock(&aw_haptic->rtp_lock);
	return;
}

static int vibrator_init(struct aw_haptic *aw_haptic)
{
	aw_dev_info("%s: enter\n", __func__);

	hrtimer_init(&aw_haptic->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw_haptic->timer.function = vibrator_timer_func;
	INIT_WORK(&aw_haptic->vibrator_work, vibrator_work_routine);
	INIT_WORK(&aw_haptic->rtp_work, rtp_work_routine);
	INIT_WORK(&aw_haptic->rtp_key_work, rtp_key_work_routine);
	mutex_init(&aw_haptic->lock);
	mutex_init(&aw_haptic->rtp_lock);

	return 0;
}

#ifdef AAC_RICHTAP
static void haptic_clean_buf(struct aw_haptic *aw_haptic, int status)
{
	struct mmap_buf_format *opbuf = aw_haptic->start_buf;
	int i = 0;

	for (i = 0; i < RICHTAP_MMAP_BUF_SUM; i++) {
		opbuf->status = status;
		opbuf = opbuf->kernel_next;
	}
}

static inline unsigned int aw_get_sys_msecs(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	struct timespec64 ts64;

	ktime_get_coarse_real_ts64(&ts64);
#else
	struct timespec64 ts64 = current_kernel_time64();
#endif
	return jiffies_to_msecs(timespec64_to_jiffies(&ts64));
}

static void rtp_work_proc(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   haptic_rtp_work);
	struct mmap_buf_format *opbuf = aw_haptic->start_buf;
	uint32_t count = 100;
	uint8_t reg_val = 0x10;
	unsigned int write_start;
	int cnt = 200;
	bool rtp_work_flag = false;
	uint8_t done_state = 0xff;

	aw_dev_info("%s enter\n", __func__);
	aw_haptic->rtp_cnt = 0;
	while (count--) {
		if(!aw_haptic->haptic_rtp_mode){
			aw_dev_info("exit %s:aw_haptic->haptic_rtp_mode = %d, count=%d\n",
					__func__, aw_haptic->haptic_rtp_mode, count);
			return;
		}
		if (opbuf->status == MMAP_BUF_DATA_VALID) {
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
			aw_haptic->func->set_rtp_aei(aw_haptic, true);
			aw_haptic->func->irq_clear(aw_haptic);
			aw_haptic->func->play_go(aw_haptic, true);

			while (cnt) {
				usleep_range(2000, 2500);
				reg_val = aw_haptic->func->get_glb_state(aw_haptic);
				if ((reg_val & AW_GLBRD_STATE_MASK) == AW_STATE_RTP) {
					cnt = 0;
					rtp_work_flag = true;
					aw_dev_info("%s: RTP_GO! glb_state=0x08\n", __func__);
				} else {
					cnt--;
					aw_dev_dbg("%s: wait for RTP_GO, glb_state=0x%02X\n",
							__func__, reg_val);
				}
			}

			if (!rtp_work_flag) {
				aw_haptic->func->set_rtp_aei(aw_haptic, false);
				aw_haptic->haptic_rtp_mode = false;
				aw_dev_err("%s: failed to enter RTP_GO status!\n", __func__);
				mutex_unlock(&aw_haptic->lock);
				return;
			}
			mutex_unlock(&aw_haptic->lock);
			break;
		} else {
			msleep(1);
		}
	}
	write_start = aw_get_sys_msecs();
	reg_val = 0x10;
	while (true) {
		if (aw_get_sys_msecs() > (write_start + 800)) {
			aw_dev_info("Failed ! %s endless loop\n", __func__);
			break;
		}
		if (reg_val & AW_BIT_SYSST_DONES || (aw_haptic->done_flag == true) || (done_state == 0)){
			aw_dev_info("reg_val = 0x%x, aw_haptic->done_flag = %d, opbuf->status = 0x%x done_state = 0x%x\n",
					reg_val, aw_haptic->done_flag, opbuf->status, done_state);
			break;
		} else if (opbuf->status == MMAP_BUF_DATA_VALID && (reg_val & 0x01 << 4)) {
			aw_haptic->func->set_rtp_data(aw_haptic, opbuf->data,
						      opbuf->length);
			memset(opbuf->data, 0, opbuf->length);
			opbuf->status = MMAP_BUF_DATA_INVALID;
			opbuf->length = 0;
			opbuf = opbuf->kernel_next;
			write_start = aw_get_sys_msecs();
		} else {
			msleep(5);
		}
		reg_val = aw_haptic->func->get_chip_state(aw_haptic);
		done_state = aw_haptic->func->get_glb_state(aw_haptic);
	}
	aw_haptic->func->set_rtp_aei(aw_haptic, false);
	aw_haptic->haptic_rtp_mode = false;
}
#endif

static ssize_t proc_vibration_style_write(void *chip_data, int val)
{
	aw_haptic_t *aw_haptic = chip_data;
	if (val == 0) {
		aw_haptic->vibration_style = HAPTIC_VIBRATION_CRISP_STYLE;
		ram_update(aw_haptic);
	} else if (val == 1) {
		aw_haptic->vibration_style = HAPTIC_VIBRATION_SOFT_STYLE;
		ram_update(aw_haptic);
	} else {
		aw_haptic->vibration_style = HAPTIC_VIBRATION_CRISP_STYLE;
	}
	return 0;
}

static void haptic_init(struct aw_haptic *aw_haptic)
{
	mutex_init(&aw_haptic->qos_lock);
	aw_haptic->gun_type = 0xFF;
	aw_haptic->bullet_nr = 0x00;
	aw_haptic->gun_mode = 0x00;
	op_clean_status(aw_haptic);

	/* haptic init */
	mutex_lock(&aw_haptic->lock);
	aw_haptic->rtp_routine_on = 0;
	aw_haptic->activate_mode = AW_CONT_MODE;
	aw_haptic->vibration_style = HAPTIC_VIBRATION_CRISP_STYLE;
	aw_haptic->func->play_mode(aw_haptic, AW_STANDBY_MODE);
	aw_haptic->func->set_pwm(aw_haptic, AW_PWM_24K);
	/* misc value init */
	aw_haptic->func->misc_para_init(aw_haptic);

	aw_haptic->func->set_bst_peak_cur(aw_haptic);
	aw_haptic->func->auto_bst_enable(aw_haptic, false);
	aw_haptic->func->offset_cali(aw_haptic);
	/* vbat compensation */
	aw_haptic->func->vbat_mode_config(aw_haptic, AW_CONT_VBAT_HW_COMP_MODE);
	aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE;

#ifdef OPLUS_FEATURE_CHG_BASIC
	aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->max_boost_vol);
#endif

	aw_haptic->func->trig_init(aw_haptic);
	mutex_unlock(&aw_haptic->lock);

	/* f0 calibration */
	mutex_lock(&aw_haptic->lock);
#ifndef OPLUS_FEATURE_CHG_BASIC
	f0_cali(aw_haptic);
#endif
	mutex_unlock(&aw_haptic->lock);
}

#ifdef AAC_RICHTAP
static int aac_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->rtp_ptr = kmalloc(RICHTAP_MMAP_BUF_SIZE * RICHTAP_MMAP_BUF_SUM, GFP_KERNEL);
	if (aw_haptic->rtp_ptr == NULL) {
		aw_dev_err("%s: malloc rtp memory failed\n", __func__);
		return -ENOMEM;
	}

	aw_haptic->start_buf = (struct mmap_buf_format *)__get_free_pages(GFP_KERNEL, RICHTAP_MMAP_PAGE_ORDER);
	if (aw_haptic->start_buf == NULL) {
		aw_dev_err("%s: Error __get_free_pages failed\n", __func__);
		return -ENOMEM;
	}
	SetPageReserved(virt_to_page(aw_haptic->start_buf));
	{
		struct mmap_buf_format *temp;
		uint32_t i = 0;

		temp = aw_haptic->start_buf;
		for (i = 1; i < RICHTAP_MMAP_BUF_SUM; i++) {
			temp->kernel_next = (aw_haptic->start_buf + i);
			temp = temp->kernel_next;
		}
		temp->kernel_next = aw_haptic->start_buf;
	}
	INIT_WORK(&aw_haptic->haptic_rtp_work, rtp_work_proc);
	/* init_waitqueue_head(&aw8697->doneQ); */
	aw_haptic->done_flag = true;
	aw_haptic->haptic_rtp_mode = false;
	return 0;
}
#endif

static int aw_interface_init(haptic_common_data_t *oh)
{
	int ret = -1;
	aw_haptic_t *aw_haptic = (aw_haptic_t *)oh->chip_data;
	struct i2c_client *i2c = aw_haptic->i2c;
	struct device_node *np = i2c->dev.of_node;
	/* keep gpio resource*/
	aw_haptic->pinctrl = oh->pinctrl;
	aw_haptic->pinctrl_state = oh->pinctrl_state;
	aw_haptic->reset_gpio = oh->reset_gpio;
	aw_haptic->irq_gpio = oh->irq_gpio;
	aw_haptic->device_id = oh->device_id;
	aw_haptic->livetap_support = oh->livetap_support;
	aw_haptic->auto_break_mode_support = oh->auto_break_mode_support;
	aw_haptic->vbat_low_vmax_level = oh->vbat_low_vmax_level;
	//aw_haptic->vibration_style = oh->vibration_style;
	/* aw func ptr init */
	ret = ctrl_init(aw_haptic);
	if (ret < 0) {
		aw_dev_err("%s: ctrl_init failed ret=%d\n", __func__, ret);
		return ret;
	}

	ret = aw_haptic->func->check_qualify(aw_haptic);
	if (ret < 0) {
		aw_dev_err("%s: qualify check failed ret=%d", __func__, ret);
		return ret;
	}

	/* aw_haptic chip id */
	ret = parse_chipid(aw_haptic);
	if (ret < 0) {
		aw_dev_err("%s: read_chipid failed ret=%d\n", __func__, ret);
		return ret;
	}

	if (aw_haptic->func->parse_dt)
		aw_haptic->func->parse_dt(&i2c->dev, aw_haptic, np);

	return 0;

}

static int aw_vibrator_init(void *chip_data)
{
	int ret = -1;
	aw_haptic_t *aw_haptic = chip_data;
	sw_reset(aw_haptic);
	ret = container_init(aw_container_size);
	if (ret < 0)
		aw_dev_err("%s: rtp alloc memory failed\n", __func__);
	if (aw_haptic->func->haptic_value_init)
		aw_haptic->func->haptic_value_init(aw_haptic);
#ifdef AAC_RICHTAP
	aac_init(aw_haptic);
#endif
	g_aw_haptic = aw_haptic;
	ret = vibrator_init(aw_haptic);
	haptic_init(aw_haptic);

	return ret;
}

static enum led_brightness aw_vibra_brightness_get(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	return aw_haptic->amplitude;
}

static void aw_vibra_brightness_set(enum led_brightness level,void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;

	aw_dev_info("%s: enter\n", __func__);
	if (!aw_haptic->ram_init) {
		aw_dev_err("%s: ram init failed, not allow to play!\n",
			   __func__);
		return;
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	return;
#endif
	aw_haptic->amplitude = level;
	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->amplitude > 0) {
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
		ram_vbat_comp(aw_haptic, false);
		ram_play(aw_haptic, AW_RAM_MODE);
	}
	mutex_unlock(&aw_haptic->lock);
}

static int aw_get_f0(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	return aw_haptic->f0;;
}

static int aw_get_rtp_file_num(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	return aw_haptic->rtp_file_num;;
}

static void aw_play_stop(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_haptic->func->play_stop(aw_haptic);
}

static void aw_rtp_mode(void *chip_data,uint32_t val)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->max_boost_vol);//boost 8.414V

	aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	aw_haptic->func->play_go(aw_haptic, true);
	usleep_range(2000, 2500);
	aw_haptic->func->set_rtp_data(aw_haptic,
				&aw_haptic->rtp_ptr[4],val);
}

static void aw_set_gain(void *chip_data,unsigned long arg)
{
	aw_haptic_t *aw_haptic = chip_data;
	int max_gain = HAPTIC_GAIN_LIMIT;
	if (arg > max_gain)
		arg = max_gain;
	aw_haptic->func->set_gain(aw_haptic, arg);
}

static void aw_stream_mode(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	struct vmax_map map;
	aw_haptic->done_flag = true;
	aw_haptic->haptic_rtp_mode = false;
	mutex_unlock(&aw_haptic->lock);
	cancel_work_sync(&aw_haptic->haptic_rtp_work);
	mutex_lock(&aw_haptic->lock);
	haptic_clean_buf(aw_haptic, MMAP_BUF_DATA_INVALID);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->done_flag = false;
	aw_haptic->haptic_rtp_mode = true;
	if (vbat_low_soc_flag() && (aw_haptic->vbat_low_vmax_level != 0)) {
		aw_haptic->func->convert_level_to_vmax(aw_haptic, &map, aw_haptic->vbat_low_vmax_level);
		aw_haptic->vmax = map.vmax;
		aw_haptic->gain = map.gain;
		aw_dev_info("%s:vbat low, max_boost_vol 0x%x, vmax 0x%x\n",
			__FUNCTION__, aw_haptic->max_boost_vol, aw_haptic->vmax);
	}
	if (vbat_low_soc_flag() && (aw_haptic->vbat_low_vmax_level != 0) && (aw_haptic->max_boost_vol > aw_haptic->vmax)) {
		aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
	} else {
		aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->max_boost_vol);//boost 8.414V
	}
	aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	schedule_work(&aw_haptic->haptic_rtp_work);
}

static void aw_stop_mode(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_dev_err("%s,RICHTAP_STOP_MODE  stop enter\n", __func__);
	aw_haptic->done_flag = true;
	op_clean_status(aw_haptic);
	/* hrtimer_cancel(&aw_haptic->timer);
	 * aw_haptic->state = 0;
	 * haptic_clean_buf(aw_haptic, MMAP_BUF_DATA_FINISHED);
	 */
	aw_haptic->haptic_rtp_mode = false;
	aw_haptic->func->set_rtp_aei(aw_haptic, false);
	aw_haptic->func->play_stop(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	cancel_work_sync(&aw_haptic->haptic_rtp_work);
	mutex_lock(&aw_haptic->lock);
}

static void aw_set_wav_seq(void *chip_data, uint8_t seq, uint8_t wave)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_haptic->func->set_wav_seq(aw_haptic, seq, wave);
}
static void aw_set_wav_loop(void *chip_data, uint8_t seq, uint8_t loop)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_haptic->func->set_wav_loop(aw_haptic, seq, loop);
}

static void aw_set_drv_bst_vol(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->max_boost_vol);
}

static void aw_play_go(void *chip_data, bool flag)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_haptic->func->play_go(aw_haptic, flag);
}

static void aw_play_mode(void *chip_data, uint8_t play_mode)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_haptic->func->play_mode(aw_haptic, play_mode);
}

static void aw_set_rtp_aei(void *chip_data, bool flag)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_haptic->func->set_rtp_aei(aw_haptic, flag);
}

static void aw_clear_interrupt_state(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_haptic->func->irq_clear(aw_haptic);
}

static void aw_rtp_work(void *chip_data, uint32_t rtp_num)
{
	aw_haptic_t *aw_haptic = chip_data;
	aw_haptic->rtp_file_num = rtp_num;
	queue_work(system_unbound_wq, &aw_haptic->rtp_work);
}

static unsigned long aw_virt_to_phys(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	return virt_to_phys(aw_haptic->start_buf);
}

static void aw_mutex_lock(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	mutex_lock(&aw_haptic->lock);
}

static void aw_mutex_unlock(void *chip_data)
{
	aw_haptic_t *aw_haptic = chip_data;
	mutex_unlock(&aw_haptic->lock);
}

struct oplus_haptic_operations aw_haptic_ops = {
	.chip_interface_init         = aw_interface_init,
	.chip_interrupt_init         = aw_interrupt_init,
	.chip_irq_isr                = irq_handle,
	.haptic_init                 = aw_vibrator_init,
	.haptic_brightness_set       = aw_vibra_brightness_set,
	.haptic_brightness_get       = aw_vibra_brightness_get,
	.proc_vibration_style_write  = proc_vibration_style_write,

	.cali_show                   = aw_cali_show,
	.f0_show                     = aw_f0_show,
	.seq_show                    = aw_seq_show,
	.reg_show                    = aw_reg_show,
	.gain_show                   = aw_gain_show,
	.state_show                  = aw_state_show,
	.rtp_show                    = aw_rtp_show,

	.duration_show               = aw_duration_show,
	.osc_cali_show               = aw_osc_cali_show,
	.ram_update_show             = aw_ram_update_show,
	.ram_vbat_comp_show          = aw_ram_vbat_comp_show,
	.lra_resistance_show         = aw_lra_resistance_show,
	.activate_show               = aw_activate_show,
	.osc_data_show               = aw_osc_data_show,
	.f0_data_show                = aw_f0_data_show,
	.oplus_brightness_show       = oplus_brightness_show,
	.oplus_duration_show         = aw_duration_show,
	.oplus_activate_show         = aw_activate_show ,
	.oplus_state_show            = aw_state_show,
	.vmax_show                   = aw_vmax_show,
	.waveform_index_show         = aw_waveform_index_show,
	.device_id_show              = aw_device_id_show,
	.livetap_support_show        = aw_livetap_support_show,
	.ram_test_show               = aw_ram_test_show,
	.rtp_going_show              = aw_rtp_going_show,
	.gun_type_show               = aw_gun_type_show,
	.gun_mode_show               = aw_gun_mode_show,
	.bullet_nr_show              = aw_bullet_nr_show,

	.activate_mode_show          = aw_activate_mode_show,
	.index_show                  = aw_index_show,
	.loop_show                   = aw_loop_show,

	.cali_store                  = aw_cali_store,
	.f0_store                    = aw_f0_store,
	.seq_store                   = aw_seq_store,
	.reg_store                   = aw_reg_store,
	.gain_store                  = aw_gain_store,
	.state_store                 = aw_state_store,
	.rtp_store                   = aw_rtp_store,
	.duration_store              = aw_duration_store,
	.osc_cali_store              = aw_osc_cali_store,
	.ram_update_store            = aw_ram_update_store,
	.ram_vbat_comp_store         = aw_ram_vbat_comp_store,
	.activate_store              = aw_activate_store,
	.osc_data_store              = aw_osc_data_store,
	.f0_data_store               = aw_f0_data_store ,
	.oplus_brightness_store      = oplus_brightness_store,
	.oplus_duration_store        = aw_duration_store,
	.oplus_activate_store        = aw_activate_store,
	.oplus_state_store           = aw_state_store,
	.vmax_store                  = aw_vmax_store,
	.waveform_index_store        = aw_waveform_index_store,
	.device_id_store             = aw_device_id_store,
	.livetap_support_store       = aw_livetap_support_store,
	.ram_test_store              = aw_ram_test_store,
	.rtp_going_store             = aw_rtp_going_store,
	.gun_type_store              = aw_gun_type_store,
	.gun_mode_store              = aw_gun_mode_store,
	.bullet_nr_store             = aw_bullet_nr_store,

	.activate_mode_store          = aw_activate_mode_store,
	.index_store                  = aw_index_store,
	.loop_store                   = aw_loop_store,

	.haptic_get_f0                = aw_get_f0,
	.haptic_get_rtp_file_num      = aw_get_rtp_file_num,
	.haptic_play_stop             = aw_play_stop,
	.haptic_rtp_mode              = aw_rtp_mode,
	.haptic_set_gain              = aw_set_gain,
	.haptic_stream_mode           = aw_stream_mode,
	.haptic_stop_mode             = aw_stop_mode,

	.haptic_set_wav_seq           = aw_set_wav_seq,
	.haptic_set_wav_loop          = aw_set_wav_loop,
	.haptic_set_drv_bst_vol       =aw_set_drv_bst_vol,
	.haptic_play_go               =aw_play_go,
	.haptic_play_mode             = aw_play_mode,
	.haptic_set_rtp_aei           = aw_set_rtp_aei,
	.haptic_clear_interrupt_state = aw_clear_interrupt_state,
	.haptic_rtp_work              = aw_rtp_work,
	.haptic_virt_to_phys          = aw_virt_to_phys,
	.haptic_mutex_lock            = aw_mutex_lock,
	.haptic_mutex_unlock          = aw_mutex_unlock,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static int awinic_i2c_probe(struct i2c_client *i2c)
#else
static int awinic_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
#endif
{
	int ret = 0;
	haptic_common_data_t *oh = NULL;
	struct aw_haptic *aw_haptic = NULL;

	aw_dev_info("%s: enter\n", __func__);

	/* 1. Alloc chip_info */
	aw_haptic = devm_kzalloc(&i2c->dev, sizeof(struct aw_haptic),
				 GFP_KERNEL);
	if (aw_haptic == NULL) {
		ret = -ENOMEM;
		goto err_alloc_aw_haptic;
	}
    /* 2. Alloc common oh */
    oh = common_haptic_data_alloc();
	if (oh == NULL) {
		aw_dev_err("oh kzalloc error\n");
		ret = -ENOMEM;
		goto oh_malloc_failed;
	}
	oh->haptic_common_ops = &aw_haptic_ops;
	oh->i2c = i2c;
	oh->dev = &i2c->dev;
	oh->chip_data = aw_haptic;
	aw_haptic->dev = &i2c->dev;
	aw_haptic->i2c = i2c;

	i2c_set_clientdata(i2c, oh);
	mutex_lock(&rst_mutex);
	ret = register_common_haptic_device(oh);
	if(ret) {
		mutex_unlock(&rst_mutex);
		goto err_register_driver;
	}
	mutex_unlock(&rst_mutex);

	aw_haptic->func->creat_node(oh);
	ram_work_init(aw_haptic);

	aw_dev_info("%s:probe completed successfully!\n", __func__);

	return 0;

err_register_driver:
	common_haptic_data_free(oh);
oh_malloc_failed:
	devm_kfree(&i2c->dev, aw_haptic);
err_alloc_aw_haptic:
	aw_haptic = NULL;
	aw_dev_err("%s:probe error\n", __func__);

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void awinic_i2c_remove(struct i2c_client *i2c)
{
	struct aw_haptic *aw_haptic = i2c_get_clientdata(i2c);

	aw_dev_info("%s: enter.\n", __func__);

	cancel_delayed_work_sync(&aw_haptic->ram_work);
	cancel_work_sync(&aw_haptic->rtp_work);
	cancel_work_sync(&aw_haptic->vibrator_work);
	hrtimer_cancel(&aw_haptic->timer);
	mutex_destroy(&aw_haptic->lock);
	mutex_destroy(&aw_haptic->rtp_lock);
	mutex_destroy(&aw_haptic->qos_lock);
#ifndef OPLUS_FEATURE_CHG_BASIC
	kfree(aw_rtp);
#else
	vfree(aw_rtp);
#endif
#ifdef AAC_RICHTAP
	kfree(aw_haptic->rtp_ptr);
	free_pages((unsigned long)aw_haptic->start_buf, RICHTAP_MMAP_PAGE_ORDER);
#endif
#ifdef TIMED_OUTPUT
	timed_output_dev_unregister(&aw_haptic->vib_dev);
#endif
	devm_free_irq(&i2c->dev, gpio_to_irq(aw_haptic->irq_gpio), aw_haptic);
	devm_kfree(&i2c->dev, aw_haptic);

	return;
}
#else
static int awinic_i2c_remove(struct i2c_client *i2c)
{
	struct aw_haptic *aw_haptic = i2c_get_clientdata(i2c);

	aw_dev_info("%s: enter.\n", __func__);

	cancel_delayed_work_sync(&aw_haptic->ram_work);
	cancel_work_sync(&aw_haptic->rtp_work);
	cancel_work_sync(&aw_haptic->vibrator_work);
	hrtimer_cancel(&aw_haptic->timer);
	mutex_destroy(&aw_haptic->lock);
	mutex_destroy(&aw_haptic->rtp_lock);
	mutex_destroy(&aw_haptic->qos_lock);
#ifndef OPLUS_FEATURE_CHG_BASIC
	kfree(aw_rtp);
#else
	vfree(aw_rtp);
#endif
#ifdef AAC_RICHTAP
	kfree(aw_haptic->rtp_ptr);
	free_pages((unsigned long)aw_haptic->start_buf, RICHTAP_MMAP_PAGE_ORDER);
#endif
#ifdef TIMED_OUTPUT
	timed_output_dev_unregister(&aw_haptic->vib_dev);
#endif
	devm_free_irq(&i2c->dev, gpio_to_irq(aw_haptic->irq_gpio), aw_haptic);
	if (gpio_is_valid(aw_haptic->irq_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->irq_gpio);
	if (gpio_is_valid(aw_haptic->reset_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->reset_gpio);
	devm_kfree(&i2c->dev, aw_haptic);

	return 0;
}
#endif

static const struct i2c_device_id awinic_i2c_id[] = {
	{AW_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, awinic_i2c_id);

static const struct of_device_id awinic_dt_match[] = {
	{.compatible = "oplus,aw_haptic"},
	{},
};

static struct i2c_driver awinic_i2c_driver = {
	.driver = {
		   .name = AW_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(awinic_dt_match),
		   },
	.probe = awinic_i2c_probe,
	.remove = awinic_i2c_remove,
	.id_table = awinic_i2c_id,
};

int awinic_i2c_init(void)
{
	int ret = 0;

	aw_dev_info("aw_haptic driver version %s\n", HAPTIC_HV_DRIVER_VERSION);
	ret = i2c_add_driver(&awinic_i2c_driver);
	if (ret) {
		aw_dev_err("%s: fail to add aw_haptic device into i2c\n", __func__);
		return ret;
	}
	return 0;
}

void awinic_i2c_exit(void)
{
	i2c_del_driver(&awinic_i2c_driver);
}

MODULE_DESCRIPTION("AWINIC Haptic Driver");
MODULE_LICENSE("GPL v2");
