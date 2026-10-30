/*
 *  Silicon Integrated Co., Ltd haptic sih688x haptic driver file
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
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
#include <linux/mm.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <linux/errno.h>
#include <linux/mman.h>
#include <linux/proc_fs.h>
#include "haptic_mid.h"
#include "haptic_regmap.h"
#include "haptic.h"
//#include "haptic_misc.h"
#include "sih688x.h"
#include "sih688x_reg.h"
#include "sih688x_func_config.h"
#include "../haptic_common/haptic_wave.h"

#include <linux/wait.h>

#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
#include "../../aw8697_haptic/haptic_feedback.h"
#endif

/* add for DX-2 bringup */
#define FW_ACTION_HOTPLUG 1

/*****************************************************
 *
 * variable
 *
 *****************************************************/

struct cdev cdev;
static sih_haptic_ptr_t g_haptic_t;
static int sih_auto_break_config_regs(sih_haptic_t *sih_haptic);

static void sih_hardware_reset(sih_haptic_t *sih_haptic)
{
	if (gpio_is_valid(sih_haptic->chip_attr.reset_gpio)) {
		gpio_set_value(sih_haptic->chip_attr.reset_gpio, SIH_RESET_GPIO_RESET);
		usleep_range(1000, 2000);
		gpio_set_value(sih_haptic->chip_attr.reset_gpio, SIH_RESET_GPIO_SET);
		usleep_range(1000, 2000);
	}
}
static void sih_chip_state_recovery(sih_haptic_t *sih_haptic)
{
	sih_haptic->chip_ipara.state = SIH_STANDBY_MODE;
	sih_haptic->chip_ipara.play_mode = SIH_IDLE_MODE;
}

static void sih_op_clean_status(
	sih_haptic_t *sih_haptic)
{
	sih_haptic->rtp.audio_ready = false;
	sih_haptic->rtp.haptic_ready = false;
	sih_haptic->rtp.pre_haptic_number = 0;
}

static bool sih_irq_rtp_local_file_handle(sih_haptic_t *sih_haptic)
{
	uint32_t buf_len = 0;
	uint32_t cont_len = 0;
	uint32_t inject_data_cnt;
	int ret = -1;

	/* inject 1/4 fifo size data once max */
	inject_data_cnt = sih_haptic->ram.base_addr >> 2;
	mutex_lock(&sih_haptic->rtp.rtp_lock);

	if (!sih_haptic->rtp.rtp_file_num) {
		hp_err("%s:rtp file num is 0, stop!\n", __func__);
		mutex_unlock(&sih_haptic->rtp.rtp_lock);
		return false;
	}

	if (!sih_haptic->rtp.rtp_cnt) {
		hp_err("%s:rtp_cnt is 0!\n", __func__);
		mutex_unlock(&sih_haptic->rtp.rtp_lock);
		return false;
	}

	if (!sih_haptic->rtp.rtp_cont) {
		hp_err("%s:rtp_container is null, break!\n", __func__);
		mutex_unlock(&sih_haptic->rtp.rtp_lock);
		return false;
	}

	hp_info("%s:rtp_cont->len = %d\n", __func__, sih_haptic->rtp.rtp_cont->len);

	cont_len = sih_haptic->rtp.rtp_cont->len;

	if ((cont_len - sih_haptic->rtp.rtp_cnt) < inject_data_cnt)
		buf_len = cont_len - sih_haptic->rtp.rtp_cnt;
	else
		buf_len = inject_data_cnt;

	hp_info("%s:buf_len:%d\n", __func__, buf_len);
	cpu_latency_qos_add_request(&sih_haptic->pm_qos, CPU_LATENCY_QOC_VALUE);
	if(buf_len > 0) {
		ret = sih_haptic->hp_func->write_rtp_data(sih_haptic,
			&sih_haptic->rtp.rtp_cont->data[sih_haptic->rtp.rtp_cnt], buf_len);

		if (ret < 0) {
			sih_haptic->hp_func->stop(sih_haptic);
			cpu_latency_qos_remove_request(&sih_haptic->pm_qos);
			sih_haptic->hp_func->set_rtp_aei(sih_haptic, false);
			sih_haptic->rtp.rtp_init = false;
			mutex_unlock(&sih_haptic->rtp.rtp_lock);
			hp_err("%s:i2c write rtp data failed\n", __func__);
			return false;
		}

		sih_haptic->rtp.rtp_cnt += buf_len;

		hp_info("%s:rtp cnt:%d\n", __func__, sih_haptic->rtp.rtp_cnt);
	}

	if ((sih_haptic->rtp.rtp_cnt == cont_len) ||
		sih_haptic->hp_func->if_chip_is_mode(sih_haptic, SIH_IDLE_MODE)) {
		if (sih_haptic->rtp.rtp_cnt != cont_len)
			hp_err("%s:rtp play error suspend!\n", __func__);
		else
			hp_info("%s:rtp update complete!\n", __func__);
		sih_haptic->hp_func->set_rtp_aei(sih_haptic, false);
		cpu_latency_qos_remove_request(&sih_haptic->pm_qos);
		sih_haptic->rtp.rtp_init = false;
		sih_chip_state_recovery(sih_haptic);
		mutex_unlock(&sih_haptic->rtp.rtp_lock);
		return false;
	}
	cpu_latency_qos_remove_request(&sih_haptic->pm_qos);
	mutex_unlock(&sih_haptic->rtp.rtp_lock);

	return true;
}

static int sih_interface_init(haptic_common_data_t *oh)
{
	int ret = -1;
	sih_haptic_t *sih_haptic = (sih_haptic_t *)oh->chip_data;
	struct i2c_client *i2c = sih_haptic->i2c;
	struct device_node *np = i2c->dev.of_node;
	/* keep gpio resource*/
	sih_haptic->pinctrl = oh->pinctrl;
	sih_haptic->pinctrl_state = oh->pinctrl_state;
	sih_haptic->chip_attr.reset_gpio = oh->reset_gpio;
	sih_haptic->chip_attr.irq_gpio = oh->irq_gpio;
	sih_haptic->device_id = oh->device_id;
	sih_haptic->livetap_support = oh->livetap_support;
	sih_haptic->auto_break_mode_support = oh->auto_break_mode_support;
	sih_haptic->vbat_low_vmax_level = oh->vbat_low_vmax_level;
	sih_hardware_reset(sih_haptic);

	ret = sih_register_func(sih_haptic);
	if (ret) {
		hp_err("%s:register functions failed\n", __func__);
		return ret;
	}

	/* registers regmap */
	sih_haptic->regmapp.regmapping = haptic_regmap_init(sih_haptic->i2c,
		sih_haptic->regmapp.config);
	if (sih_haptic->regmapp.regmapping == NULL) {
		hp_err("%s:register regmap failed\n", __func__);
		return -EINVAL;
	}

	if (sih_haptic->hp_func->parse_dt)
		sih_haptic->hp_func->parse_dt(&i2c->dev, sih_haptic, np);
	return 0;

}
static int sih_interrupt_init(void *chip_data)
{

	sih_haptic_t *sih_haptic = (sih_haptic_t *)chip_data;
	sih_haptic->hp_func->interrupt_state_init(sih_haptic);
	return 0; 
}
static irqreturn_t sih_irq_isr(int irq, void *data)
{
	sih_haptic_t *sih_haptic = data;

	hp_info("%s:enter! interrupt code number is %d\n", __func__, irq);

	if (sih_haptic->stream_func->is_stream_mode(sih_haptic))
		return IRQ_HANDLED;

	if (sih_haptic->hp_func->get_rtp_fifo_empty_state(sih_haptic)) {
		if (sih_haptic->rtp.rtp_init) {
			while ((!sih_haptic->hp_func->get_rtp_fifo_full_state(sih_haptic)) &&
				(sih_haptic->chip_ipara.play_mode == SIH_RTP_MODE)) {
				if (!sih_irq_rtp_local_file_handle(sih_haptic))
					break;
			}
		} else {
			hp_err("%s: rtp init false\n", __func__);
		}
	}

	if (sih_haptic->chip_ipara.play_mode != SIH_RTP_MODE)
		sih_haptic->hp_func->set_rtp_aei(sih_haptic, false);

	/* detect */
	if ((sih_haptic->detect.trig_detect_en | sih_haptic->detect.ram_detect_en |
		sih_haptic->detect.rtp_detect_en | sih_haptic->detect.cont_detect_en) &&
		(sih_haptic->hp_func->if_chip_is_detect_done(sih_haptic))) {
		hp_info("%s:if chip is detect done\n", __func__);
		sih_haptic->hp_func->ram_init(sih_haptic, true);
		sih_haptic->hp_func->read_detect_fifo(sih_haptic);
		sih_haptic->hp_func->ram_init(sih_haptic, false);
		sih_haptic->hp_func->detect_fifo_ctrl(sih_haptic, false);
		sih_haptic->detect.detect_f0_read_done = true;
	}
	hp_info("%s:exit\n", __func__);
	return IRQ_HANDLED;
}

static void sih_vfree_container(sih_haptic_t *sih_haptic,
	haptic_container_t *cont)
{
	if (cont != NULL)
		vfree(cont);
}

static void sih_rtp_play_func(sih_haptic_t *sih_haptic, uint8_t mode)
{
	uint32_t buf_len = 0;
	uint32_t cont_len = 0;
	haptic_container_t *rtp_cont = sih_haptic->rtp.rtp_cont;

	if (!rtp_cont) {
		hp_err("%s:cont is null\n", __func__);
		sih_chip_state_recovery(sih_haptic);
		return;
	}

	hp_info("%s:the rtp cont len is %d\n", __func__, rtp_cont->len);
	sih_haptic->rtp.rtp_cnt = 0;
	mutex_lock(&sih_haptic->rtp.rtp_lock);
	cpu_latency_qos_add_request(&sih_haptic->pm_qos, CPU_LATENCY_QOC_VALUE);
	while (1) {
		if (!sih_haptic->hp_func->get_rtp_fifo_full_state(sih_haptic)) {
			cont_len = rtp_cont->len;
			if (sih_haptic->rtp.rtp_cnt < sih_haptic->ram.base_addr) {
				if ((cont_len - sih_haptic->rtp.rtp_cnt) <
					sih_haptic->ram.base_addr)
					buf_len = cont_len - sih_haptic->rtp.rtp_cnt;
				else
					buf_len = sih_haptic->ram.base_addr;
			} else if ((cont_len - sih_haptic->rtp.rtp_cnt) <
				(sih_haptic->ram.base_addr >> 2)) {
				buf_len = cont_len - sih_haptic->rtp.rtp_cnt;
			} else {
				buf_len = sih_haptic->ram.base_addr >> 2;
			}

			if (sih_haptic->rtp.rtp_cnt != cont_len) {
				if (mode == SIH_RTP_OSC_PLAY) {
					if (sih_haptic->osc_para.start_flag) {
						sih_haptic->osc_para.kstart = ktime_get();
						sih_haptic->osc_para.start_flag = false;
					}
				}
				sih_haptic->hp_func->write_rtp_data(sih_haptic,
					&rtp_cont->data[sih_haptic->rtp.rtp_cnt], buf_len);
				sih_haptic->rtp.rtp_cnt += buf_len;

				hp_info("%s:rtp cnt=%d\n", __func__, sih_haptic->rtp.rtp_cnt);
			}
		}

		if (sih_haptic->hp_func->get_rtp_fifo_full_state(sih_haptic) &&
			mode == SIH_RTP_NORMAL_PLAY) {
			break;
		}
		if ((sih_haptic->rtp.rtp_cnt == cont_len) ||
			sih_haptic->hp_func->if_chip_is_mode(sih_haptic, SIH_IDLE_MODE)) {
			if (sih_haptic->rtp.rtp_cnt != cont_len)
				hp_err("%s:rtp suspend!\n", __func__);
			else
				hp_info("%s:rtp complete!\n", __func__);

			if (mode == SIH_RTP_OSC_PLAY)
				sih_haptic->osc_para.kend = ktime_get();

			sih_chip_state_recovery(sih_haptic);
			break;
		}
	}

	if (mode == SIH_RTP_NORMAL_PLAY &&
		sih_haptic->chip_ipara.play_mode == SIH_RTP_MODE) {
		sih_haptic->hp_func->set_rtp_aei(sih_haptic, true);
	}
	cpu_latency_qos_remove_request(&sih_haptic->pm_qos);
	mutex_unlock(&sih_haptic->rtp.rtp_lock);
}

static void sih_rtp_play(sih_haptic_t *sih_haptic, uint8_t mode)
{
	hp_info("%s:rtp mode:%d\n", __func__, mode);
	if (mode == SIH_RTP_NORMAL_PLAY) {
		sih_rtp_play_func(sih_haptic, mode);
	} else if (mode == SIH_RTP_OSC_PLAY) {
		sih_haptic->osc_para.start_flag = true;
		sih_rtp_play_func(sih_haptic, mode);
		sih_haptic->osc_para.actual_time =
			ktime_to_us(ktime_sub(sih_haptic->osc_para.kend,
			sih_haptic->osc_para.kstart));
		hp_info("%s:actual time:%d\n", __func__,
			sih_haptic->osc_para.actual_time);
	} else {
		hp_err("%s:err mode %d\n", __func__, mode);
	}
}

static void sih_rtp_local_work(sih_haptic_t *sih_haptic, uint8_t mode)
{
	bool rtp_work_flag = false;
	int cnt = SIH_ENTER_RTP_MODE_MAX_TRY;
	int ret = -1;
	const struct firmware *rtp_file;
	uint8_t rtp_file_index = 0;

	hp_info("%s:enter!\n", __func__);

	if (mode == SIH_RTP_OSC_PLAY)
		rtp_file_index = SIH_OSC_PLAY_FILE_INDEX;
	else
		hp_err("%s:err mode:%d\n", __func__, mode);

	mutex_lock(&sih_haptic->rtp.rtp_lock);

	sih_haptic->rtp.rtp_init = false;
	sih_vfree_container(sih_haptic, sih_haptic->rtp.rtp_cont);
	sih_haptic->rtp.rtp_cont = NULL;

	ret = request_firmware(&rtp_file, haptic_rtp_name[rtp_file_index],
		sih_haptic->dev);
	if (ret < 0) {
		hp_err("%s:fail to read %s\n", __func__, haptic_rtp_name[rtp_file_index]);
		sih_chip_state_recovery(sih_haptic);
		mutex_unlock(&sih_haptic->rtp.rtp_lock);
		return;
	}

	sih_haptic->rtp.rtp_cont = vmalloc(rtp_file->size + sizeof(int));
	if (!sih_haptic->rtp.rtp_cont) {
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		(void)oplus_haptic_track_mem_alloc_err(
			HAPTIC_MEM_ALLOC_TRACK,
			rtp_file->size + sizeof(int), __func__);
#endif
		release_firmware(rtp_file);
		hp_err("%s:error allocating memory\n", __func__);
		sih_chip_state_recovery(sih_haptic);
		mutex_unlock(&sih_haptic->rtp.rtp_lock);
		return;
	}
	sih_haptic->rtp.rtp_cont->len = rtp_file->size;
	if (mode == SIH_RTP_OSC_PLAY)
		sih_haptic->osc_para.osc_rtp_len = rtp_file->size;

	mutex_unlock(&sih_haptic->rtp.rtp_lock);

	memcpy(sih_haptic->rtp.rtp_cont->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);

	mutex_lock(&sih_haptic->lock);
	sih_haptic->rtp.rtp_init = true;
	sih_haptic->chip_ipara.state = SIH_ACTIVE_MODE;
	sih_haptic->hp_func->stop(sih_haptic);
	sih_haptic->hp_func->set_rtp_aei(sih_haptic, false);
	sih_haptic->hp_func->set_play_mode(sih_haptic, SIH_RTP_MODE);
	/* osc rtp cali set trim to zero */
	if (mode == SIH_RTP_OSC_PLAY){
		if (sih_haptic->osc_para.set_trim) {
			sih_haptic->hp_func->upload_f0(sih_haptic, SIH_WRITE_ZERO);
		} else {
			sih_haptic->hp_func->upload_f0(sih_haptic, SIH_OSC_CALI_LRA);
		}
	}
	if (mode != SIH_RTP_NORMAL_PLAY)
		disable_irq(gpio_to_irq(sih_haptic->chip_attr.irq_gpio));

	sih_haptic->hp_func->play_go(sih_haptic, true);
	usleep_range(2000, 2500);
	while (cnt--) {
		if (sih_haptic->hp_func->if_chip_is_mode(sih_haptic, SIH_RTP_MODE)) {
			rtp_work_flag = true;
			hp_info("%s:rtp go!\n", __func__);
			break;
		}

		hp_info("%s:wait for rtp go!\n", __func__);
		usleep_range(2000, 2500);
	}
	if (rtp_work_flag) {
		sih_rtp_play(sih_haptic, mode);
	} else {
		/* enter standby mode */
		sih_haptic->hp_func->stop(sih_haptic);
		sih_haptic->chip_ipara.state = SIH_STANDBY_MODE;
		hp_err("%s:failed to enter rtp_go status!\n", __func__);
	}
	/* enable irq */
	if (mode != SIH_RTP_NORMAL_PLAY)
		enable_irq(gpio_to_irq(sih_haptic->chip_attr.irq_gpio));

	mutex_unlock(&sih_haptic->lock);
}

/*****************************************************
 *
 * ram
 *
 *****************************************************/
static void get_ram_num(sih_haptic_t *sih_haptic)
{
	uint8_t wave_addr[2] = {0};
	uint32_t first_wave_addr = 0;

	hp_info("%s:enter\n", __func__);
	if (!sih_haptic->ram.ram_init) {
		hp_err("%s:ram init failed, wave_num = 0!\n", __func__);
		return;
	}

	mutex_lock(&sih_haptic->lock);
	/* RAMINIT Enable */
	sih_haptic->hp_func->ram_init(sih_haptic, true);
	sih_haptic->hp_func->stop(sih_haptic);
	sih_haptic->hp_func->set_ram_addr(sih_haptic, sih_haptic->ram.base_addr);
	sih_haptic->hp_func->get_first_wave_addr(sih_haptic, wave_addr);
	first_wave_addr = (wave_addr[0] << 8 | wave_addr[1]);
	sih_haptic->ram.wave_num = (first_wave_addr -
		sih_haptic->ram.base_addr - 1) / 4;

	hp_info("%s:first wave addr = 0x%04x, wave_num = %d\n", __func__,
		first_wave_addr, sih_haptic->ram.wave_num);

	/* RAMINIT Disable */
	sih_haptic->hp_func->ram_init(sih_haptic, false);
	mutex_unlock(&sih_haptic->lock);
}

static void sih_ram_load(const struct firmware *cont, void *context)
{
	int i;
	int ret = -1;
	uint16_t check_sum = 0;
	sih_haptic_t *sih_haptic = context;
	haptic_container_t *sih_haptic_fw;

	hp_info("%s:enter\n", __func__);

	if (!cont) {
		hp_err("%s:failed to read firmware\n", __func__);
		release_firmware(cont);
		return;
	}

	hp_info("%s:loaded size: %zu\n", __func__, cont ? cont->size : 0);

	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];
	if (check_sum != (uint16_t)((cont->data[0] << 8) | (cont->data[1]))) {
		hp_err("%s:check sum err: check_sum=0x%04x\n", __func__, check_sum);
		release_firmware(cont);
		return;
	}

	hp_info("%s:check sum pass : 0x%04x\n", __func__, check_sum);

	sih_haptic->ram.check_sum = check_sum;

	/*ram update */
	sih_haptic_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!sih_haptic_fw) {
		release_firmware(cont);
		hp_err("%s:error allocating memory\n", __func__);
		return;
	}

	sih_haptic_fw->len = cont->size;
	memcpy(sih_haptic_fw->data, cont->data, cont->size);
	release_firmware(cont);
	ret = sih_haptic->hp_func->update_ram_config(sih_haptic, sih_haptic_fw);
	if (ret) {
		hp_err("%s:ram firmware update failed!\n", __func__);
	} else {
		sih_haptic->ram.ram_init = true;
		sih_haptic->ram.len = sih_haptic_fw->len - sih_haptic->ram.ram_shift;
		hp_info("%s:ram firmware update complete!\n", __func__);
		get_ram_num(sih_haptic);
	}
	kfree(sih_haptic_fw);
}

static void sih_ram_play(sih_haptic_t *sih_haptic, uint8_t mode)
{
	hp_info("%s:enter\n", __func__);
	sih_haptic->hp_func->set_play_mode(sih_haptic, mode);
	sih_haptic->hp_func->play_go(sih_haptic, true);
}

sih_haptic_t *get_global_haptic_ptr(void)
{
	return g_haptic_t.g_haptic[SIH_HAPTIC_MMAP_DEV_INDEX];
}

int pointer_prehandle(struct device *dev, const char *buf,
	cdev_t **cdev, sih_haptic_t **sih_haptic)
{
	hp_info("%s:enter\n", __func__);
	null_pointer_err_check(dev);
	null_pointer_err_check(buf);
	*cdev = dev_get_drvdata(dev);
	null_pointer_err_check(*cdev);
	*sih_haptic = container_of(*cdev, sih_haptic_t, soft_frame.vib_dev);
	null_pointer_err_check(*sih_haptic);

	return 0;
}

static int ram_update(sih_haptic_t *sih_haptic)
{
	int len = 0;
	int index = 0;

	hp_info("%s:enter\n", __func__);

	sih_haptic->ram.ram_init = false;
	sih_haptic->detect.tracking_f0 = haptic_common_get_f0();

	hp_info("%s: haptic_real_f0 [%d]\n", __func__, (sih_haptic->detect.tracking_f0 / 10));
	if (DEVICE_ID_0815 == sih_haptic->device_id || DEVICE_ID_0809 == sih_haptic->device_id) {
		if (sih_haptic->ram.vibration_style == HAPTIC_VIBRATION_CRISP_STYLE) {
			len =  request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
						haptic_ram_name[index], sih_haptic->dev, GFP_KERNEL,
						sih_haptic, sih_ram_load);
			hp_err("%s line:%d: haptic bin name %s \n", __func__, __LINE__, haptic_ram_name[index]);
		} else if (sih_haptic->ram.vibration_style == HAPTIC_VIBRATION_SOFT_STYLE) {
			len =  request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
						haptic_ram_name_170_soft[index], sih_haptic->dev, GFP_KERNEL,
						sih_haptic, sih_ram_load);
			hp_err("%s line:%d: haptic bin name %s \n", __func__, __LINE__, haptic_ram_name[index]);
		} else {
			len =  request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
						haptic_ram_name[index], sih_haptic->dev, GFP_KERNEL,
						sih_haptic, sih_ram_load);
			hp_err("%s line:%d: haptic bin name %s \n", __func__, __LINE__, haptic_ram_name[index]);
		}
	} else if (DEVICE_ID_1419 == sih_haptic->device_id) {
		if (sih_haptic->ram.vibration_style == HAPTIC_VIBRATION_CRISP_STYLE) {
			len =  request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
						haptic_ram_name_205[index], sih_haptic->dev, GFP_KERNEL,
						sih_haptic, sih_ram_load);
			hp_err("%s line:%d: haptic bin name %s \n", __func__, __LINE__, haptic_ram_name_205[index]);
		} else if (sih_haptic->ram.vibration_style == HAPTIC_VIBRATION_SOFT_STYLE) {
			len =  request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
						 haptic_ram_name_205_soft[index], sih_haptic->dev, GFP_KERNEL,
						sih_haptic, sih_ram_load);
			hp_err("%s line:%d: haptic bin name %s \n", __func__, __LINE__,  haptic_ram_name_205_soft[index]);
		} else {
			len =  request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
						haptic_ram_name_205[index], sih_haptic->dev, GFP_KERNEL,
						sih_haptic, sih_ram_load);
			hp_err("%s line:%d: haptic bin name %s \n", __func__, __LINE__, haptic_ram_name_205[index]);
		}
	}
	return len;
}

static void ram_update_work_func(struct work_struct *work) {
	sih_haptic_t *sih_haptic =
		container_of(work, sih_haptic_t, ram.ram_update_work);

	hp_err("%s: enter\n", __func__);
	ram_update(sih_haptic);
}
/*****************************************************
 *
 * vibrator sysfs node
 *
 *****************************************************/
static enum hrtimer_restart haptic_timer_func(struct hrtimer *timer)
{
	sih_haptic_t *sih_haptic = container_of(timer, sih_haptic_t, timer);

	hp_info("%s:enter!\n", __func__);
	sih_chip_state_recovery(sih_haptic);
	queue_work(system_highpri_wq, &sih_haptic->ram.ram_work);
	return HRTIMER_NORESTART;
}

static void ram_work_func(struct work_struct *work)
{
	sih_haptic_t *sih_haptic = container_of(work, sih_haptic_t, ram.ram_work);

	hp_info("%s:enter!\n", __func__);
	mutex_lock(&sih_haptic->lock);
	/* Enter standby mode */
	sih_haptic->hp_func->stop(sih_haptic);
	if (sih_haptic->chip_ipara.state == SIH_ACTIVE_MODE) {
		switch (sih_haptic->ram.action_mode) {
		case SIH_RAM_MODE:
			sih_haptic->hp_func->set_boost_mode(sih_haptic, true);
			sih_ram_play(sih_haptic, SIH_RAM_MODE);
			break;
		case SIH_RAM_LOOP_MODE:
			if ((DEVICE_ID_1419 == sih_haptic->device_id) || (sih_haptic->device_id == DEVICE_ID_0809)){
				sih_haptic->hp_func->set_boost_mode(sih_haptic, true);
			} else {
				sih_haptic->hp_func->set_boost_mode(sih_haptic, false);
				sih_haptic->hp_func->vbat_comp(sih_haptic);
			}
			sih_ram_play(sih_haptic, SIH_RAM_LOOP_MODE);
			/* run ms timer */
			hrtimer_start(&sih_haptic->timer,
				ktime_set(sih_haptic->chip_ipara.duration / 1000,
				(sih_haptic->chip_ipara.duration % 1000) * 1000000),
				HRTIMER_MODE_REL);
			break;
		default:
			hp_err("%s:err sta = %d\n", __func__, sih_haptic->chip_ipara.state);
			break;
		}
	}
	mutex_unlock(&sih_haptic->lock);
}

static void rtp_work_func(struct work_struct *work)
{
	bool rtp_work_flag = false;
	int cnt = SIH_ENTER_RTP_MODE_MAX_TRY;
	int ret = -1;
	const struct firmware *rtp_file;
	uint8_t *haptic_rtp_key_data = NULL;
	uint32_t haptic_rtp_key_data_len = 0;
	const char* rtp_name = NULL;
	sih_haptic_t *sih_haptic = container_of(work, sih_haptic_t, rtp.rtp_work);

	hp_info("%s:enter!\n", __func__);

	mutex_lock(&sih_haptic->rtp.rtp_lock);

	sih_haptic->rtp.rtp_init = false;
	sih_vfree_container(sih_haptic, sih_haptic->rtp.rtp_cont);
	sih_haptic->rtp.rtp_cont = NULL;
	if (get_rtp_key_support(sih_haptic->rtp.rtp_file_num)) {
		hp_info("%s: key scene, use array data", __func__);
		haptic_rtp_key_data = get_rtp_key_data(&haptic_rtp_key_data_len);
		if (haptic_rtp_key_data == NULL) {
			hp_err("%s: haptic_rtp_key_data is NULL! vibrator id: %d \n", __func__, sih_haptic->device_id);
			mutex_unlock(&sih_haptic->rtp.rtp_lock);
			return;
		}

		sih_haptic->rtp.rtp_cont = vmalloc(haptic_rtp_key_data_len + sizeof(int));
		if (!sih_haptic->rtp.rtp_cont) {
			hp_err("%s:error allocating memory\n", __func__);
			sih_chip_state_recovery(sih_haptic);
			mutex_unlock(&sih_haptic->rtp.rtp_lock);
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
			(void)oplus_haptic_track_mem_alloc_err(HAPTIC_MEM_ALLOC_TRACK,
				haptic_rtp_key_data_len + sizeof(int), __func__);
#endif

			return;
		}
		sih_haptic->rtp.rtp_cont->len = haptic_rtp_key_data_len;
		memcpy(sih_haptic->rtp.rtp_cont->data, haptic_rtp_key_data, haptic_rtp_key_data_len);
	} else {
		rtp_file = rtp_load_file_accord_f0(sih_haptic->rtp.rtp_file_num);
		if (!rtp_file) {
			rtp_name = get_rtp_name(sih_haptic->rtp.rtp_file_num, sih_haptic->detect.tracking_f0);
			if (!rtp_name) {
				hp_err("%s: get rtp name failed.\n", __func__);
				sih_chip_state_recovery(sih_haptic);
				mutex_unlock(&sih_haptic->rtp.rtp_lock);
				return;
			}
			ret = request_firmware(&rtp_file, rtp_name, sih_haptic->dev);
			hp_err("%s line:%d: rtp_num:%d name:%s\n", __func__, __LINE__,
					sih_haptic->rtp.rtp_file_num, rtp_name);
			vfree(rtp_name);
			if (ret < 0) {
				hp_err("%s:no this rtp file\n", __func__);
				sih_chip_state_recovery(sih_haptic);
				mutex_unlock(&sih_haptic->rtp.rtp_lock);
				return;
			}
		}

		sih_haptic->rtp.rtp_cont = vmalloc(rtp_file->size + sizeof(int));
		if (!sih_haptic->rtp.rtp_cont) {
			release_firmware(rtp_file);
			hp_err("%s:error allocating memory\n", __func__);
			sih_chip_state_recovery(sih_haptic);
			mutex_unlock(&sih_haptic->rtp.rtp_lock);
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
			(void)oplus_haptic_track_mem_alloc_err(HAPTIC_MEM_ALLOC_TRACK,
				rtp_file->size + sizeof(int), __func__);
#endif
			return;
		}
		sih_haptic->rtp.rtp_cont->len = rtp_file->size;
		memcpy(sih_haptic->rtp.rtp_cont->data, rtp_file->data, rtp_file->size);
		release_firmware(rtp_file);
	}

	mutex_unlock(&sih_haptic->rtp.rtp_lock);
	hp_info("%s:rtp len is %d\n", __func__, sih_haptic->rtp.rtp_cont->len);

	mutex_lock(&sih_haptic->lock);

	sih_haptic->rtp.rtp_init = true;
	sih_haptic->chip_ipara.state = SIH_ACTIVE_MODE;
	sih_haptic->hp_func->set_play_mode(sih_haptic, SIH_RTP_MODE);
	sih_haptic->hp_func->play_go(sih_haptic, true);
	usleep_range(2000, 2500);
	while (cnt--) {
		if (sih_haptic->hp_func->if_chip_is_mode(sih_haptic, SIH_RTP_MODE)) {
			rtp_work_flag = true;
			hp_info("%s:rtp go!\n", __func__);
			break;
		}

		hp_info("%s:wait for rtp go!\n", __func__);
		usleep_range(2000, 2500);
	}
	if (rtp_work_flag && sih_haptic->rtp.rtp_file_num != 0) {
		sih_rtp_play(sih_haptic, SIH_RTP_NORMAL_PLAY);
	} else {
		sih_haptic->hp_func->stop(sih_haptic);
		sih_haptic->chip_ipara.state = SIH_STANDBY_MODE;
		sih_op_clean_status(sih_haptic);
		hp_err("%s:rtp go failed! not enter rtp status!\n", __func__);
	}
	mutex_unlock(&sih_haptic->lock);
}

static int vibrator_chip_init(sih_haptic_t *sih_haptic)
{
	int ret = -1;
	ret = sih_haptic->hp_func->efuse_check(sih_haptic);
	if (ret < 0)
		return ret;
	sih_haptic->hp_func->init(sih_haptic);
	sih_haptic->hp_func->stop(sih_haptic);
	/* load lra reg config */
	ret = sih_lra_config_load(sih_haptic);
	if (ret < 0)
		return ret;
	if (sih_haptic->auto_break_mode_support) {
		sih_auto_break_config_regs(sih_haptic);
		sih_haptic->hp_func->set_brk_state(sih_haptic, SIH_RAM_MODE, true);
		sih_haptic->hp_func->set_brk_state(sih_haptic, SIH_RTP_MODE, true);
		hp_info("%s: auto break opened\n", __func__);
	}
	sih_op_clean_status(sih_haptic);
	hp_info("%s:end\n", __func__);
	return ret;
}

static ssize_t proc_vibration_style_write(void *chip_data, int val)
{
	sih_haptic_t *sih_haptic = chip_data;

	if (val == 0) {
		sih_haptic->ram.vibration_style = HAPTIC_VIBRATION_CRISP_STYLE;
		schedule_work(&sih_haptic->ram.ram_update_work);
	} else if (val == 1){
		sih_haptic->ram.vibration_style = HAPTIC_VIBRATION_SOFT_STYLE;
		schedule_work(&sih_haptic->ram.ram_update_work);
	} else {
		sih_haptic->ram.vibration_style = HAPTIC_VIBRATION_CRISP_STYLE;
	}
	return 0;
}

static int vibrator_init(sih_haptic_t *sih_haptic)
{
	int ret = -1;
	/* vibrator globle ptr init */
	g_haptic_t.sih_num = SIH_HAPTIC_DEV_NUM;
	g_haptic_t.g_haptic[SIH_HAPTIC_MMAP_DEV_INDEX] = sih_haptic;
	/* timer init */
	hrtimer_init(&sih_haptic->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sih_haptic->timer.function = haptic_timer_func;
	/* work func init */
	INIT_WORK(&sih_haptic->ram.ram_work, ram_work_func);
	INIT_WORK(&sih_haptic->rtp.rtp_work, rtp_work_func);
	INIT_WORK(&sih_haptic->ram.ram_update_work, ram_update_work_func);
	//INIT_WORK(&sih_haptic->motor_old_test_work, motor_old_test_work);
	sih_haptic->motor_old_test_mode = 0;
	sih_haptic->gun_type = 0xFF;
	sih_haptic->bullet_nr = 0x00;
	sih_haptic->gun_mode = 0x00;
	/* mutex init */
	mutex_init(&sih_haptic->lock);
	mutex_init(&sih_haptic->rtp.rtp_lock);

	ret = sih_haptic->stream_func->stream_rtp_work_init(sih_haptic);
	if (ret) {
		hp_err("%s: stream rtp work init failed\n", __func__);
		return ret;
	}
	//ret = init_vibrator_proc(sih_haptic);
	//if (ret) {
	//	hp_err("%s: init vibrator proc failed\n", __func__);
	//	return ret;
	//}
	hp_info("%s:end\n", __func__);
	return ret;
}
static void sih_vibra_brightness_set(enum led_brightness level,void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;

	if (!sih_haptic->ram.ram_init) {
		hp_err("%s:ram init error\n", __func__);
		return;
	}
	hp_info("%s:vibra brightness set\n", __func__);
	mutex_lock(&sih_haptic->lock);
	sih_haptic->hp_func->stop(sih_haptic);
	if (level > 0) {
		sih_haptic->ram.action_mode = SIH_RAM_MODE;
		sih_haptic->chip_ipara.state = SIH_ACTIVE_MODE;

		sih_ram_play(sih_haptic, sih_haptic->ram.action_mode);
	}
	mutex_unlock(&sih_haptic->lock);
}
static enum led_brightness sih_vibra_brightness_get(void *chip_data)
{
	return LED_OFF;
}
static ssize_t sih_cali_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
    static ssize_t len = 0;
    len += snprintf(buf, PAGE_SIZE, "%d\n",
		sih_haptic->detect.tracking_f0);
    return len;
}
static ssize_t sih_cali_store(void *chip_data, const char *buf, uint32_t val)
{
    sih_haptic_t *sih_haptic = chip_data;
	uint8_t i;
	uint32_t sih_f0_min_threshold = 0;
	uint32_t sih_f0_max_threshold = 0;

	if (DEVICE_ID_0815 == sih_haptic->device_id || DEVICE_ID_0809 == sih_haptic->device_id) {
		sih_f0_min_threshold = SIH_F0_MIN_THRESHOLD;
		sih_f0_max_threshold = SIH_F0_MAX_THRESHOLD;
	} else if (DEVICE_ID_1419 == sih_haptic->device_id) {
		sih_f0_min_threshold = SIH_F0_MIN_THRESHOLD_1419;
		sih_f0_max_threshold = SIH_F0_MAX_THRESHOLD_1419;
	} else {
		sih_f0_min_threshold = SIH_F0_MIN_THRESHOLD;
		sih_f0_max_threshold = SIH_F0_MAX_THRESHOLD;
	}
	hp_info("%s:value = %d, f0_min = %d, f0_max = %d\n", __func__,
		val, sih_f0_min_threshold, sih_f0_max_threshold);

	if (val == 1) {
		mutex_lock(&sih_haptic->lock);
		for (i = 0; i < SIH_F0_DETECT_TRY; i++) {
			sih_haptic->hp_func->get_tracking_f0(sih_haptic);
			if (sih_haptic->detect.tracking_f0 <= sih_f0_min_threshold &&
				sih_haptic->detect.tracking_f0 >= sih_f0_max_threshold) {
				break;
			}
			msleep(200);
		}
		sih_haptic->hp_func->upload_f0(sih_haptic, SIH_F0_CALI_LRA);
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		if (sih_haptic->detect.f0_cali_data == 0)
			(void)oplus_haptic_track_fre_cail(HAPTIC_F0_CALI_TRACK,
				sih_haptic->detect.tracking_f0,
				-ERANGE, "f0 out of range");
#endif

		mutex_unlock(&sih_haptic->lock);
	}
    return 0;
}

static ssize_t sih_f0_store(void *chip_data, const char *buf ,uint32_t val)
{
	sih_haptic_t *sih_haptic = chip_data;

	sih_haptic->detect.tracking_f0 = val;
	schedule_work(&sih_haptic->ram.ram_update_work);
	return 0;
}
static ssize_t sih_f0_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
    static ssize_t len = 0;
    len += snprintf(buf, PAGE_SIZE, "%d\n",
		sih_haptic->detect.tracking_f0);
    return len;
}
static ssize_t sih_seq_store(void *chip_data, const char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
    unsigned int databuf[2] = {0, 0};
	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		if (databuf[0] >= SIH_HAPTIC_SEQUENCER_SIZE) {
			hp_err("%s:input value out of range!\n", __func__);
			return 0;
		}
		mutex_lock(&sih_haptic->lock);
		sih_haptic->ram.seq[(uint8_t)databuf[0]] = (uint8_t)databuf[1];
		sih_haptic->hp_func->set_wav_seq(sih_haptic,
			(uint8_t)databuf[0], (uint8_t)databuf[1]);
		mutex_unlock(&sih_haptic->lock);
	}

    return 0;
}
static ssize_t sih_seq_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
    static ssize_t len = 0;
	uint8_t i;
    sih_haptic->hp_func->get_wav_seq(sih_haptic, SIH_HAPTIC_SEQUENCER_SIZE);

	for (i = 0; i < SIH_HAPTIC_SEQUENCER_SIZE; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"seq%d = %d\n", i, sih_haptic->ram.seq[i]);
	}
    return len;
}

static ssize_t sih_reg_store(void *chip_data, const char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
    unsigned int databuf[2] = {0, 0};
	uint8_t val = 0;

    if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		val = (uint8_t)databuf[1];
		haptic_regmap_write(sih_haptic->regmapp.regmapping,
			(uint8_t)databuf[0], SIH_I2C_OPERA_BYTE_ONE, &val);
	}
    return 0;
}
static ssize_t sih_reg_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
    uint32_t i;
	ssize_t len = 0;
	uint8_t reg_val = 0;

	for (i = 0; i <= SIH688X_REG_MAX; i++) {
		haptic_regmap_read(sih_haptic->regmapp.regmapping, i,
			SIH_I2C_OPERA_BYTE_ONE, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
			"0x%02x = 0x%02x\n", i, reg_val);
	}
    return len;

}
static ssize_t sih_gain_store(void *chip_data, const char *buf, uint32_t val)
{
	sih_haptic_t *sih_haptic = chip_data;

	mutex_lock(&sih_haptic->lock);
	sih_haptic->chip_ipara.gain = val;
	sih_haptic->hp_func->set_gain(sih_haptic, sih_haptic->chip_ipara.gain);
	mutex_unlock(&sih_haptic->lock);

    return 0;
}
static ssize_t sih_gain_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
    ssize_t len = 0;
    len = snprintf(buf, PAGE_SIZE, "gain = 0x%02x\n", sih_haptic->chip_ipara.gain);
    return len;
}

static ssize_t sih_state_store(void *chip_data, const char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
    uint32_t val = 0;
	int rc = 0;

    rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val == SIH_STANDBY_MODE) {
		sih_haptic->hp_func->stop(sih_haptic);
		sih_haptic->chip_ipara.state = SIH_STANDBY_MODE;
	}
	return 0;

}
static ssize_t sih_state_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
    ssize_t len = 0;
    
    sih_haptic->hp_func->update_chip_state(sih_haptic);

	len += snprintf(buf + len, PAGE_SIZE - len, "state = %d, play_mode = %d\n",
		sih_haptic->chip_ipara.state, sih_haptic->chip_ipara.play_mode);

    return len;
}

static ssize_t sih_rtp_store(void *chip_data, const char *buf, uint32_t val)
{
	sih_haptic_t *sih_haptic = chip_data;
	int rtp_is_going_on = 0;

	mutex_lock(&sih_haptic->lock);

	rtp_is_going_on = sih_haptic->hp_func->if_chip_is_mode(sih_haptic, SIH_RTP_MODE);
	if (rtp_is_going_on && (val == AUDIO_READY_STATUS)) {
		hp_err("%s: audio status rtp[%d]\n", __func__, val);
		mutex_unlock(&sih_haptic->lock);
		return 0;
	}

	/*OP add for juge rtp on end*/
	if (get_ringtone_support(val)) {
		if (val == AUDIO_READY_STATUS) {
			sih_haptic->rtp.audio_ready = true;
		} else {
			sih_haptic->rtp.haptic_ready = true;
		}

		hp_err("%s:audio[%d] and haptic[%d]\n", __func__,
			sih_haptic->rtp.audio_ready, sih_haptic->rtp.haptic_ready);

		if (sih_haptic->rtp.haptic_ready && !sih_haptic->rtp.audio_ready) {
			sih_haptic->rtp.pre_haptic_number = val;
		}
		if (!sih_haptic->rtp.audio_ready || !sih_haptic->rtp.haptic_ready) {
			mutex_unlock(&sih_haptic->lock);
			return 0;
		}
	}

	if (val == AUDIO_READY_STATUS && sih_haptic->rtp.pre_haptic_number) {
		hp_err("%s:pre_haptic_num:%d\n", __func__, sih_haptic->rtp.pre_haptic_number);
		val = sih_haptic->rtp.pre_haptic_number;
	}

	sih_op_clean_status(sih_haptic);
	sih_haptic->rtp.rtp_file_num = val;
	sih_haptic->hp_func->stop(sih_haptic);
	sih_haptic->hp_func->set_rtp_aei(sih_haptic, false);

	mutex_unlock(&sih_haptic->lock);

	if (val > 0 && val < NUM_WAVEFORMS) {
		schedule_work(&sih_haptic->rtp.rtp_work);
	} else {
		hp_err("%s: input number err:%d\n", __func__, val);
	}

	return 0;
}
static ssize_t sih_rtp_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp_cnt = %d\n",
		sih_haptic->rtp.rtp_cnt);

	return len;
}


static ssize_t sih_ram_store(void *chip_data, const char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	uint32_t databuf[2] = {0, 0};

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) != 2) {
		hp_err("%s:input parameter error\n", __func__);
		return 0;
	}

	if (!sih_haptic->ram.ram_init) {
		hp_err("%s:ram init failed, not allow to play!\n", __func__);
		return 0;
	}

	mutex_lock(&sih_haptic->lock);

	/* RAM MODE */
	if (databuf[0] == 1) {
		sih_haptic->hp_func->stop(sih_haptic);
		sih_haptic->ram.action_mode = SIH_RAM_MODE;
	/* LOOPRAM MODE */
	} else if (databuf[0] == 2) {
		sih_haptic->hp_func->stop(sih_haptic);
		sih_haptic->ram.action_mode = SIH_RAM_LOOP_MODE;
	} else {
		mutex_unlock(&sih_haptic->lock);
		hp_err("%s:mode parameter error\n", __func__);
		return 0;
	}

	if (databuf[1] == 1) {
		sih_haptic->chip_ipara.state = SIH_ACTIVE_MODE;
	} else {
		sih_haptic->hp_func->stop(sih_haptic);
		sih_haptic->chip_ipara.state = SIH_STANDBY_MODE;
		mutex_unlock(&sih_haptic->lock);
		return 0;
	}

	if (hrtimer_active(&sih_haptic->timer))
		hrtimer_cancel(&sih_haptic->timer);

	mutex_unlock(&sih_haptic->lock);
	sih_haptic->hp_func->check_detect_state(sih_haptic, SIH_RAM_MODE);
	schedule_work(&sih_haptic->ram.ram_work);

	return 0;
}


static ssize_t sih_duration_store(void *chip_data, const char *buf, uint32_t val)
{
	sih_haptic_t *sih_haptic = chip_data;

	sih_haptic->chip_ipara.duration = val;
	hp_info("%s: duration = %d\n", __func__, sih_haptic->chip_ipara.duration);
	return 0;
}
static ssize_t sih_duration_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;
	ktime_t time_remain;
	s64 time_ms = 0;

	if (hrtimer_active(&sih_haptic->timer)) {
		time_remain = hrtimer_get_remaining(&sih_haptic->timer);
		time_ms = ktime_to_ms(time_remain);
	}

	len = snprintf(buf, PAGE_SIZE, "%lldms\n", time_ms);

	return len;
}

static ssize_t sih_osc_cali_store(void *chip_data, const char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	hp_info("%s: enter, val = %d\n", __func__, val);
	if (val <= 0)
		return 0;

	if (val == 3) {
		sih_haptic->osc_para.set_trim = true;
		sih_haptic->chip_ipara.state = SIH_ACTIVE_MODE;
		sih_rtp_local_work(sih_haptic, SIH_RTP_OSC_PLAY);
		sih_haptic->hp_func->osc_cali(sih_haptic);
	} else if (val == 1) {
		sih_haptic->osc_para.set_trim = false;
		sih_haptic->chip_ipara.state = SIH_ACTIVE_MODE;
		sih_rtp_local_work(sih_haptic, SIH_RTP_OSC_PLAY);
	}
	return 0;  
}
static ssize_t sih_osc_cali_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",
		sih_haptic->osc_para.actual_time);
	return len; 
}

static ssize_t sih_ram_update_store(void *chip_data, const char *buf, uint32_t val)
{
	sih_haptic_t *sih_haptic = chip_data;

	if (val) {
		schedule_work(&sih_haptic->ram.ram_update_work);
	}
	return 0;
}
static ssize_t sih_ram_update_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;
	int size = 0;
	int i = 0;
	int j = 0;
	uint8_t ram_data[SIH_RAMDATA_READ_SIZE] = {0};
	/* RAMINIT Enable */
	mutex_lock(&sih_haptic->lock);
	sih_haptic->hp_func->stop(sih_haptic);
	sih_haptic->hp_func->ram_init(sih_haptic, true);
	sih_haptic->hp_func->set_ram_addr(sih_haptic, sih_haptic->ram.base_addr);
	len += snprintf(buf + len, PAGE_SIZE - len, "sih_haptic_ram:\n");
	while (i < sih_haptic->ram.len) {
		if ((sih_haptic->ram.len - i) <= SIH_RAMDATA_READ_SIZE)
			size = sih_haptic->ram.len - i;
		else
			size = SIH_RAMDATA_READ_SIZE;
		sih_haptic->hp_func->get_ram_data(sih_haptic, ram_data, size);
		for (j = 0; j < size; j++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"0x%02X,", ram_data[j]);
		}
		i += size;
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	/* RANINIT Disable */
	sih_haptic->hp_func->ram_init(sih_haptic, false);
	mutex_unlock(&sih_haptic->lock);
	return len;
}

static ssize_t sih_ram_vbat_comp_store(void *chip_data, const char *buf, uint32_t val)
{
	sih_haptic_t *sih_haptic = chip_data;

	mutex_lock(&sih_haptic->lock);

	if (val)
		sih_haptic->ram.ram_vbat_comp = SIH_RAM_VBAT_COMP_ENABLE;
	else
		sih_haptic->ram.ram_vbat_comp = SIH_RAM_VBAT_COMP_DISABLE;

	mutex_unlock(&sih_haptic->lock);

	return 0;  
}
static ssize_t sih_ram_vbat_comp_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "ram_vbat_comp = %d\n",
		sih_haptic->ram.ram_vbat_comp);

	return len;   
}

static ssize_t sih_lra_resistance_store(void *chip_data, const char *buf)
{
 	sih_haptic_t *sih_haptic = chip_data;
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtoint(buf, 0 , &val);
	if (rc < 0)
		return rc;

	sih_haptic->detect.rl_offset = val;

	return 0;   
}
static ssize_t sih_lra_resistance_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;

	if (!sih_haptic || !sih_haptic->hp_func ||
		!sih_haptic->hp_func->get_lra_resistance)
		return -EINVAL;

	mutex_lock(&sih_haptic->lock);
	sih_haptic->hp_func->get_lra_resistance(sih_haptic);
	mutex_unlock(&sih_haptic->lock);

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",
				(uint32_t)sih_haptic->detect.resistance);
	return len;
}
static ssize_t sih_f0_save_store(void *chip_data, const char *buf)
{
	uint32_t val = 0;
	sih_haptic_t *sih_haptic = chip_data;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	sih_haptic->detect.tracking_f0 = val;

	if (DEVICE_ID_0815 == sih_haptic->device_id || DEVICE_ID_0809 == sih_haptic->device_id) {
		if (sih_haptic->detect.tracking_f0 < F0_VAL_MIN_0815
				|| sih_haptic->detect.tracking_f0 > F0_VAL_MAX_0815) {
			sih_haptic->detect.tracking_f0 = 1700;
		}
	} else if (DEVICE_ID_1419 == sih_haptic->device_id) {
		if (sih_haptic->detect.tracking_f0 < F0_VAL_MIN_1419
				|| sih_haptic->detect.tracking_f0 > F0_VAL_MAX_1419) {
			sih_haptic->detect.tracking_f0 = 2050;
		}
	}
	hp_err("%s:f0 = %d\n", __func__, val);
	return 0;
}
static ssize_t sih_f0_save_show(void *chip_data, char *buf)
{
	ssize_t len = 0;
	sih_haptic_t *sih_haptic = chip_data;

	len += snprintf(buf + len, PAGE_SIZE - len, "f0_data = %d\n",
		sih_haptic->detect.tracking_f0);

	return len;
}


static ssize_t sih_activate_store(void *chip_data, const char *buf, uint32_t val)
{
	sih_haptic_t *sih_haptic = chip_data;
	int rtp_is_going_on = 0;

	rtp_is_going_on = sih_haptic->hp_func->if_chip_is_mode(sih_haptic, SIH_RTP_MODE);
	if (rtp_is_going_on) {
		hp_info("%s: rtp is going\n", __func__);
		return 0;
	}
	if (!sih_haptic->ram.ram_init) {
		hp_err("%s:ram init failed\n", __func__);
		return 0;
	}
	mutex_lock(&sih_haptic->lock);
	if (hrtimer_active(&sih_haptic->timer))
		hrtimer_cancel(&sih_haptic->timer);
	sih_haptic->chip_ipara.state = val;
	mutex_unlock(&sih_haptic->lock);

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (sih_haptic->chip_ipara.state) {
		hp_info("%s: gain=0x%02x\n", __func__, sih_haptic->chip_ipara.gain);
		mutex_lock(&sih_haptic->lock);
		sih_haptic->hp_func->stop(sih_haptic);
		sih_haptic->chip_ipara.state = SIH_ACTIVE_MODE;
		sih_haptic->ram.action_mode = SIH_RAM_LOOP_MODE;
		haptic_set_ftm_wave();
		if (hrtimer_active(&sih_haptic->timer))
			hrtimer_cancel(&sih_haptic->timer);
		mutex_unlock(&sih_haptic->lock);
		queue_work(system_highpri_wq, &sih_haptic->ram.ram_work);
	} else {
		mutex_lock(&sih_haptic->lock);
		sih_haptic->hp_func->stop(sih_haptic);
		mutex_unlock(&sih_haptic->lock);
	}
#endif

	return 0;    
}
static ssize_t sih_activate_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;

	len += snprintf(buf, PAGE_SIZE, "activate = %d\n",
		sih_haptic->chip_ipara.state);

	return len;   
}

static ssize_t sih_drv_vboost_store(void *chip_data, const char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
	uint32_t val = 0;
	int rc = 0;

	if (!sih_haptic || !sih_haptic->hp_func ||
		!sih_haptic->hp_func->set_drv_bst_vol)
		return -EINVAL;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	hp_info("%s:value=%d\n", __func__, val);

	mutex_lock(&sih_haptic->lock);
	sih_haptic->chip_ipara.drv_vboost = val;
	if (val < SIH688X_DRV_VBOOST_MIN * SIH688X_DRV_VBOOST_COEFFICIENT) {
		hp_info("%s:drv_vboost is too low,set to 60:%d", __func__, val);
		sih_haptic->chip_ipara.drv_vboost = SIH688X_DRV_VBOOST_MIN
			* SIH688X_DRV_VBOOST_COEFFICIENT;
	} else if (val > SIH688X_DRV_VBOOST_MAX * SIH688X_DRV_VBOOST_COEFFICIENT) {
		hp_info("%s:drv_vboost is too high,set to 110:%d", __func__, val);
		sih_haptic->chip_ipara.drv_vboost = SIH688X_DRV_VBOOST_MAX
			* SIH688X_DRV_VBOOST_COEFFICIENT;
	}
	sih_haptic->hp_func->set_drv_bst_vol(sih_haptic,
		sih_haptic->chip_ipara.drv_vboost);
	mutex_unlock(&sih_haptic->lock);

	return 0;
}
static ssize_t sih_drv_vboost_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;

	if (!sih_haptic)
		return -EINVAL;

	len += snprintf(buf, PAGE_SIZE, "drv_vboost = %d\n",
		sih_haptic->chip_ipara.drv_vboost);

	return len;  
}

static ssize_t sih_detect_vbat_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;

	mutex_lock(&sih_haptic->lock);
	sih_haptic->hp_func->get_vbat(sih_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "detect_vbat = %d\n",
		sih_haptic->detect.vbat);
	mutex_unlock(&sih_haptic->lock);

	return len;  
}

static ssize_t sih_audio_delay_store(void *chip_data, const char *buf)
{
	uint32_t val = 0;
	int ret = 0;
	sih_haptic_t *sih_haptic = chip_data;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;
	sih_haptic->rtp.audio_delay = val;

	return 0;  
}
static ssize_t sih_audio_delay_show(void *chip_data, char *buf)
{
 	ssize_t len = 0;
	sih_haptic_t *sih_haptic = chip_data;

	len += snprintf(buf, PAGE_SIZE, "audio_delay = %d\n",
		sih_haptic->rtp.audio_delay);

	return len;   
}
static ssize_t sih_osc_data_store(void *chip_data, const char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	uint32_t val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0) {
        return rc;
    }
	mutex_lock(&sih_haptic->lock);
	sih_haptic->osc_para.osc_data = val;
	mutex_unlock(&sih_haptic->lock);
	return 0;
}
static ssize_t sih_osc_data_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;

	int len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",
			sih_haptic->osc_para.osc_data);

	return len;
}

static ssize_t sih_f0_data_store(void *chip_data, const char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	uint32_t val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0) {
        return rc;
    }
	mutex_lock(&sih_haptic->lock);
	sih_haptic->detect.f0_cali_data = val;
	mutex_unlock(&sih_haptic->lock);
	return 0;

}
static ssize_t sih_f0_data_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	int len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",
			sih_haptic->detect.f0_cali_data);

	return len;
}

static ssize_t sih_brightness_store(void *chip_data, const char *buf, uint32_t val)
{
	sih_haptic_t *sih_haptic = chip_data;

	if (!sih_haptic->ram.ram_init) {
		hp_err("%s:ram init error,not allow to play!\n", __func__);
		return 0;
	}

	sih_haptic->amplitude = val;
	mutex_lock(&sih_haptic->lock);
	sih_haptic->hp_func->stop(sih_haptic);
	if (sih_haptic->amplitude > 0) {
		sih_haptic->ram.action_mode = SIH_RAM_MODE;
		sih_haptic->chip_ipara.state = SIH_ACTIVE_MODE;

		sih_ram_play(sih_haptic, sih_haptic->ram.action_mode);
	}
	mutex_unlock(&sih_haptic->lock);
	return 0;

}
static ssize_t sih_brightness_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	return snprintf(buf, PAGE_SIZE, "%d\n", sih_haptic->amplitude);;
}

static ssize_t sih_vmax_store(void *chip_data, const char *buf, uint32_t val)
{
	sih_haptic_t *sih_haptic = chip_data;
	struct vmax_map map;

	mutex_lock(&sih_haptic->lock);
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (val <= HAPTIC_MAX_LEVEL) {
		val = val / 100 * 100;
		sih_haptic->hp_func->convert_level_to_vmax(sih_haptic, &map, val);
		sih_haptic->chip_ipara.vmax = map.vmax;
		sih_haptic->chip_ipara.gain = map.gain;
	} else {
		sih_haptic->chip_ipara.vmax = sih_haptic->chip_ipara.drv_vboost;
		sih_haptic->chip_ipara.gain = 0x80;
	}

	if (val == HAPTIC_OLD_TEST_LEVEL) {	/* for old test only */
		sih_haptic->chip_ipara.gain = HAPTIC_RAM_VBAT_COMP_GAIN;
	}

	if (sih_haptic->device_id == DEVICE_ID_0833) {
		sih_haptic->chip_ipara.vmax = sih_haptic->chip_ipara.drv_vboost;
		sih_haptic->chip_ipara.gain = 0x80;
	}

	if (vbat_low_soc_flag() && (sih_haptic->vbat_low_vmax_level != 0) && (val > sih_haptic->vbat_low_vmax_level)) {
		sih_haptic->hp_func->convert_level_to_vmax(sih_haptic, &map, sih_haptic->vbat_low_vmax_level);
		sih_haptic->chip_ipara.vmax = map.vmax;
		sih_haptic->chip_ipara.gain = map.gain;
	}

	sih_haptic->hp_func->set_gain(sih_haptic, sih_haptic->chip_ipara.gain);
	sih_haptic->hp_func->set_drv_bst_vol(sih_haptic, sih_haptic->chip_ipara.vmax);
#else
	sih_haptic->chip_ipara.vmax = val;
	sih_haptic->hp_func->set_drv_bst_vol(sih_haptic, sih_haptic->chip_ipara.vmax);
#endif
	mutex_unlock(&sih_haptic->lock);
	hp_info("%s: gain[0x%x], vmax[%d] end\n", __func__,
			sih_haptic->chip_ipara.gain, sih_haptic->chip_ipara.vmax);

	return 0;
}
static ssize_t sih_vmax_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", sih_haptic->chip_ipara.vmax);
}

static ssize_t sih_waveform_index_store(void *chip_data, const char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	unsigned int databuf[1] = {0};

	if (1 == sscanf(buf, "%d", &databuf[0])) {
		hp_err("%s: waveform_index = %d\n", __func__, databuf[0]);
		mutex_lock(&sih_haptic->lock);
		sih_haptic->ram.seq[0] = (unsigned char)databuf[0];
		sih_haptic->hp_func->set_wav_seq(sih_haptic, 0, sih_haptic->ram.seq[0]);
		sih_haptic->hp_func->set_wav_seq(sih_haptic, 1, 0);
		sih_haptic->hp_func->set_wav_loop(sih_haptic, 0, 0);
		mutex_unlock(&sih_haptic->lock);
	}
	return 0;
}
static ssize_t sih_waveform_index_show(void *chip_data, char *buf)
{
    return 0;
}

static ssize_t sih_device_id_store(void *chip_data, const char *buf)
{
    return 0;
}
static ssize_t sih_device_id_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", sih_haptic->device_id);
}

static ssize_t sih_livetap_support_store(void *chip_data, const char *buf, int val)
{
	sih_haptic_t *sih_haptic = chip_data;

	if (val > 0)
		sih_haptic->livetap_support = true;
	else
		sih_haptic->livetap_support = false;

	return 0;
}
static ssize_t sih_livetap_support_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", sih_haptic->livetap_support);
}
static ssize_t sih_ram_test_store(void *chip_data, const char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	struct haptic_container *sh_ramtest;
	int i, j = 0;
	int rc = 0;
	unsigned int val = 0;
	unsigned int start_addr;
	unsigned int tmp_len, retries;
	char *pbuf = NULL;

	hp_err("%s enter\n", __func__);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	start_addr = 0;
	sih_haptic->ram_test_flag_0 = 0;
	sih_haptic->ram_test_flag_1 = 0;
	tmp_len = 1024 ;  /* 1K */
	retries = 8;  /* tmp_len * retries = 8 * 1024 */
	sh_ramtest = kzalloc(tmp_len * sizeof(char) + sizeof(int), GFP_KERNEL);
	if (!sh_ramtest) {
		hp_err("%s: error allocating memory\n", __func__);
		return 0;
	}
	pbuf = kzalloc(tmp_len * sizeof(char), GFP_KERNEL);
	if (!pbuf) {
		hp_err("%s: Error allocating memory\n", __func__);
		kfree(sh_ramtest);
		return 0;
	}
	sh_ramtest->len = tmp_len;

	if (val == 1) {
		mutex_lock(&sih_haptic->lock);
		/* RAMINIT Enable */
		sih_haptic->hp_func->ram_init(sih_haptic, true);
		for (j = 0; j < retries; j++) {
			/*test 1-----------start*/
			memset(sh_ramtest->data, 0xff, sh_ramtest->len);
			memset(pbuf, 0x00, sh_ramtest->len);
			/* write ram 1 test */
			sih_haptic->hp_func->set_ram_addr(sih_haptic, start_addr);
			sih_haptic->hp_func->set_ram_data(sih_haptic,
						      sh_ramtest->data,
						      sh_ramtest->len);

			/* read ram 1 test */
			sih_haptic->hp_func->set_ram_addr(sih_haptic, start_addr);
			sih_haptic->hp_func->get_ram_data(sih_haptic, pbuf,
						      sh_ramtest->len);

			for (i = 0; i < sh_ramtest->len; i++) {
				if (pbuf[i] != 0xff)
					sih_haptic->ram_test_flag_1++;
			}
			 /*test 1------------end*/

			/*test 0----------start*/
			memset(sh_ramtest->data, 0x00, sh_ramtest->len);
			memset(pbuf, 0xff, sh_ramtest->len);

			/* write ram 0 test */
			sih_haptic->hp_func->set_ram_addr(sih_haptic, start_addr);
			sih_haptic->hp_func->set_ram_data(sih_haptic,
						      sh_ramtest->data,
						      sh_ramtest->len);
			/* read ram 0 test */
			sih_haptic->hp_func->set_ram_addr(sih_haptic, start_addr);
			sih_haptic->hp_func->get_ram_data(sih_haptic, pbuf,
						      sh_ramtest->len);
			for (i = 0; i < sh_ramtest->len; i++) {
				if (pbuf[i] != 0)
					 sih_haptic->ram_test_flag_0++;
			}
			/*test 0 end*/
			start_addr += tmp_len;
		}
		/* RAMINIT Disable */
		// sih_haptic->hp_func->ram_init(sih_haptic, false);
		schedule_work(&sih_haptic->ram.ram_update_work);
		mutex_unlock(&sih_haptic->lock);
	}
	kfree(sh_ramtest);
	kfree(pbuf);
	pbuf = NULL;
	hp_err("%s exit\n", __func__);
	return 0;  
}
static ssize_t sih_ram_test_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;
	unsigned int ram_test_result = 0;

	if (sih_haptic->ram_test_flag_0 != 0 ||
	    sih_haptic->ram_test_flag_1 != 0) {
		ram_test_result = 1; /* failed */
		len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", ram_test_result);
	} else {
		ram_test_result = 0; /* pass */
		len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", ram_test_result);
	}
	return len;
}

static ssize_t sih_rtp_going_store(void *chip_data, const char *buf)
{
    return 0;
}
static ssize_t sih_rtp_going_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;
	ssize_t len = 0;
	int val = -1;

	val = sih_haptic->hp_func->if_chip_is_mode(sih_haptic, SIH_RTP_MODE);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", val);
	return len;
}

static ssize_t sih_gun_type_store(void *chip_data, const char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	int rc = 0;
	unsigned int val = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	hp_info("%s: value=%d\n", __func__, val);

	mutex_lock(&sih_haptic->lock);
	sih_haptic->gun_type = val;
	mutex_unlock(&sih_haptic->lock);
	return 0; 
}
static ssize_t sih_gun_type_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", sih_haptic->gun_type);
}

static ssize_t sih_gun_mode_store(void *chip_data, const char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	int rc = 0;
	unsigned int val = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	hp_info("%s: value=%d\n", __func__, val);

	mutex_lock(&sih_haptic->lock);
	sih_haptic->gun_mode = val;
	mutex_unlock(&sih_haptic->lock);
	return 0;
}
static ssize_t sih_gun_mode_show(void *chip_data, char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", sih_haptic->gun_mode);
}

static ssize_t sih_bullet_nr_store(void *chip_data, const char *buf)
{
	sih_haptic_t *sih_haptic = chip_data;
	unsigned int val = 0;
	int rc =0;
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	hp_info("%s: value=%d\n", __func__, val);
	mutex_lock(&sih_haptic->lock);
	sih_haptic->bullet_nr = val;
	mutex_unlock(&sih_haptic->lock);
	return 0;
}
static ssize_t sih_bullet_nr_show(void *chip_data, char *buf)
{
    sih_haptic_t *sih_haptic = chip_data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", sih_haptic->bullet_nr);
}
static int sih_vibrator_init(void *chip_data)
{
    int ret = -1;
    sih_haptic_t *sih_haptic = chip_data;
    ret = vibrator_chip_init(sih_haptic);
	if (ret) {
		hp_err("%s: chip init failed\n", __func__);
		return ret;
	}
    ret = vibrator_init(sih_haptic);
    if (ret) {
		hp_err("%s: chip init failed\n", __func__);
		return ret;
	}
	return ret;
}

static void haptic_clean_buf(sih_haptic_t *sih_haptic, int status)
{
	mmap_buf_format_t *opbuf = sih_haptic->stream_para.start_buf;
	int i;

	for (i = 0; i < IOCTL_MMAP_BUF_SUM; i++) {
		opbuf->status = status;
		opbuf = opbuf->kernel_next;
	}
}

static inline unsigned long int sih_get_sys_msecs(void)
{
	ktime_t get_time;
	unsigned long int get_time_ms;

	get_time = ktime_get();
	get_time_ms = ktime_to_ms(get_time);

	return get_time_ms;
}

static void stream_work_proc(struct work_struct *work)
{
	sih_haptic_t *sih_haptic =
		container_of(work, sih_haptic_t, stream_para.stream_work);
	mmap_buf_format_t *opbuf = sih_haptic->stream_para.start_buf;
	uint32_t count = IOCTL_WAIT_BUFF_VALID_MAX_TRY;
	uint8_t reg_val;
	unsigned int write_start;
	hp_err("stream_work_proc enter !");

	while (true && count--) {
		if (!sih_haptic->stream_para.stream_mode)
			return;
		hp_err("stream_work_proc count = %d status = %x!", count, opbuf->status);
		if (opbuf->status == MMAP_BUF_DATA_VALID) {
			hp_err("stream_work_proc get mutex lock !");
			mutex_lock(&sih_haptic->lock);
			sih_haptic->hp_func->set_play_mode(sih_haptic, SIH_RTP_MODE);
			sih_haptic->hp_func->set_rtp_aei(sih_haptic, true);
			sih_haptic->hp_func->clear_interrupt_state(sih_haptic);
			sih_haptic->hp_func->play_go(sih_haptic, true);
			mutex_unlock(&sih_haptic->lock);
			break;
		}
		msleep(1);
	}
	write_start = sih_get_sys_msecs();
	reg_val = SIH_SYSSST_BIT_FIFO_AE;
	while (1) {
		if (!sih_haptic->stream_para.stream_mode)
			break;
		if (sih_get_sys_msecs() > (write_start + 800)) {
			hp_err("%s:failed!endless loop\n", __func__);
			break;
		}

		if ((reg_val & SIH_SYSSST_BIT_STANDBY) ||
				(sih_haptic->stream_para.done_flag) ||
				(opbuf->status == MMAP_BUF_DATA_FINISHED) ||
				(opbuf->status == MMAP_BUF_DATA_INVALID)) {
			hp_err("%s:buff status:0x%02x length:%d\n", __func__,
			opbuf->status, opbuf->length);
			break;
		} else if (opbuf->status == MMAP_BUF_DATA_VALID &&
			((reg_val & SIH_SYSSST_BIT_FIFO_AE) == SIH_SYSSST_BIT_FIFO_AE)) {
			haptic_regmap_write(sih_haptic->regmapp.regmapping,
				SIH688X_REG_RTPDATA, opbuf->length, opbuf->data);

			hp_info("%s:writes length:%d\n", __func__, opbuf->length);
			memset(opbuf->data, 0, opbuf->length);
			opbuf->status = MMAP_BUF_DATA_INVALID;
			opbuf->length = 0;
			opbuf = opbuf->kernel_next;
			write_start = sih_get_sys_msecs();
		} else {
			usleep_range(100, 200);
		}
		haptic_regmap_read(sih_haptic->regmapp.regmapping, SIH688X_REG_SYSSST,
			SIH_I2C_OPERA_BYTE_ONE, &reg_val);
	}
	sih_haptic->hp_func->set_rtp_aei(sih_haptic, false);
	sih_haptic->stream_para.stream_mode = false;
	hp_err("stream_work_proc exit !");
}

static int sih_stream_rtp_work_init(sih_haptic_t *sih_haptic)
{
	sih_haptic->stream_para.rtp_ptr =
		kmalloc(IOCTL_MMAP_BUF_SIZE * IOCTL_MMAP_BUF_SUM, GFP_KERNEL);
	if (sih_haptic->stream_para.rtp_ptr == NULL) {
		hp_err("malloc rtp memory failed\n");
		return -ENOMEM;
	}

	sih_haptic->stream_para.start_buf =
		(mmap_buf_format_t *)__get_free_pages(GFP_KERNEL, IOCTL_MMAP_PAGE_ORDER);
	if (sih_haptic->stream_para.start_buf == NULL) {
		hp_err("error get page failed\n");
		return -ENOMEM;
	}
	SetPageReserved(virt_to_page(sih_haptic->stream_para.start_buf)); {
		mmap_buf_format_t *temp;
		uint32_t i;

		temp = sih_haptic->stream_para.start_buf;
		for (i = 1; i < IOCTL_MMAP_BUF_SUM; i++) {
			temp->kernel_next = (sih_haptic->stream_para.start_buf + i);
			temp = temp->kernel_next;
		}
		temp->kernel_next = sih_haptic->stream_para.start_buf;
	}

	INIT_WORK(&sih_haptic->stream_para.stream_work, stream_work_proc);
	sih_haptic->stream_para.done_flag = true;
	sih_haptic->stream_para.stream_mode = false;

	hp_info("%s:init ok\n", __func__);

	return 0;
}

static void sih_stream_rtp_work_release(sih_haptic_t *sih_haptic)
{
	kfree(sih_haptic->stream_para.rtp_ptr);
	free_pages((unsigned long)sih_haptic->stream_para.start_buf,
		IOCTL_MMAP_PAGE_ORDER);
}

static bool sih_is_stream_mode(sih_haptic_t *sih_haptic)
{
	if (sih_haptic->stream_para.stream_mode)
		return true;
	return false;
}

haptic_stream_func_t stream_play_func = {
	.is_stream_mode = sih_is_stream_mode,
	.stream_rtp_work_init = sih_stream_rtp_work_init,
	.stream_rtp_work_release = sih_stream_rtp_work_release,
};

static int sih_get_f0(void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;
	return sih_haptic->detect.tracking_f0;
}

static int sih_get_rtp_file_num(void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;
	return sih_haptic->rtp.rtp_file_num;
}

static void sih_play_stop(void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->hp_func->stop(sih_haptic);
}

static void sih_enter_rtp_mode(void *chip_data, uint32_t val)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->hp_func->set_play_mode(sih_haptic, SIH_RTP_MODE);
	sih_haptic->hp_func->play_go(sih_haptic, true);
	usleep_range(2000, 2500);
	haptic_regmap_write(sih_haptic->regmapp.regmapping,
		SIH688X_REG_RTPDATA, val, &sih_haptic->stream_para.rtp_ptr[4]);
}

static void sih_set_gain(void *chip_data, unsigned long arg)
{
	sih_haptic_t *sih_haptic = chip_data;
	if (arg > HAPTIC_GAIN_LIMIT)
		arg = HAPTIC_GAIN_LIMIT;
	sih_haptic->hp_func->set_gain(sih_haptic, arg);
}

static void sih_stream_mode(void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;
	struct vmax_map map;
	sih_haptic->stream_para.stream_mode = false;
	haptic_clean_buf(sih_haptic, MMAP_BUF_DATA_INVALID);
	sih_haptic->hp_func->stop(sih_haptic);
	sih_haptic->stream_para.done_flag = false;
	sih_haptic->stream_para.stream_mode = true;
	if (vbat_low_soc_flag() && (sih_haptic->vbat_low_vmax_level != 0)) {
		sih_haptic->hp_func->convert_level_to_vmax(sih_haptic, &map, sih_haptic->vbat_low_vmax_level);
		sih_haptic->chip_ipara.vmax = map.vmax;
		sih_haptic->chip_ipara.gain = map.gain;
		hp_info("%s:vbat low, max_boost_vol 0x%x, vmax 0x%x\n",
			__FUNCTION__, sih_haptic->chip_ipara.drv_vboost, sih_haptic->chip_ipara.vmax);
	}

	if (vbat_low_soc_flag() && (sih_haptic->vbat_low_vmax_level != 0) &&
		(sih_haptic->chip_ipara.drv_vboost > sih_haptic->chip_ipara.vmax)) {
		sih_haptic->hp_func->set_drv_bst_vol(sih_haptic, sih_haptic->chip_ipara.vmax);
	} else {
		sih_haptic->hp_func->set_drv_bst_vol(sih_haptic,sih_haptic->chip_ipara.drv_vboost);
	}

	schedule_work(&sih_haptic->stream_para.stream_work);
}

static void sih_stop_mode(void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->stream_para.done_flag = true;
	sih_haptic->rtp.audio_ready = false;
	sih_haptic->rtp.haptic_ready = false;
	sih_haptic->rtp.pre_haptic_number = 0;
	sih_haptic->chip_ipara.state = SIH_STANDBY_MODE;
	sih_haptic->chip_ipara.play_mode = SIH_IDLE_MODE;
	usleep_range(2000, 2000);
	sih_haptic->hp_func->set_rtp_aei(sih_haptic, false);
	sih_haptic->hp_func->stop(sih_haptic);
	sih_haptic->hp_func->clear_interrupt_state(sih_haptic);
	sih_haptic->stream_para.stream_mode = false;
}

static void sih_set_wav_seq(void *chip_data, uint8_t seq, uint8_t wave)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->hp_func->set_wav_seq(sih_haptic, seq, wave);
}
static void sih_set_wav_loop(void *chip_data, uint8_t seq, uint8_t loop)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->hp_func->set_wav_loop(sih_haptic, seq, loop);
}

static void sih_set_drv_bst_vol(void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->hp_func->set_drv_bst_vol(sih_haptic, sih_haptic->chip_ipara.drv_vboost );
}

static void sih_play_go(void *chip_data, bool flag)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->hp_func->play_go(sih_haptic, flag);
}

static void sih_play_mode(void *chip_data, uint8_t play_mode)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->hp_func->set_play_mode(sih_haptic, play_mode);
}

static void sih_set_rtp_aei(void *chip_data, bool flag)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->hp_func->set_rtp_aei(sih_haptic, flag);
}

static void sih_clear_interrupt_state(void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->hp_func->clear_interrupt_state(sih_haptic);
}

static void sih_rtp_work(void *chip_data, uint32_t rtp_num)
{
	sih_haptic_t *sih_haptic = chip_data;
	sih_haptic->rtp.rtp_file_num = rtp_num;
	queue_work(system_unbound_wq, &sih_haptic->rtp.rtp_work);
}

static unsigned long sih_virt_to_phys(void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;
	return virt_to_phys(sih_haptic->stream_para.start_buf);
}

static void sih_mutex_lock(void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;
	mutex_lock(&sih_haptic->lock);
}

static void sih_mutex_unlock(void *chip_data)
{
	sih_haptic_t *sih_haptic = chip_data;
	mutex_unlock(&sih_haptic->lock);
}

struct oplus_haptic_operations si_haptic_ops = {
	.chip_interface_init         = sih_interface_init,
	.chip_interrupt_init         = sih_interrupt_init,
	.chip_irq_isr                = sih_irq_isr,
	.haptic_init                 = sih_vibrator_init,
	.haptic_brightness_set       = sih_vibra_brightness_set,
	.haptic_brightness_get       = sih_vibra_brightness_get,
	.proc_vibration_style_write  = proc_vibration_style_write,

	.cali_show                   = sih_cali_show,
	.f0_show                     = sih_f0_show,
	.seq_show                    = sih_seq_show,
	.reg_show                    = sih_reg_show,
	.gain_show                   = sih_gain_show,
	.state_show                  = sih_state_show,
	.rtp_show                    = sih_rtp_show,

	.duration_show               = sih_duration_show,
	.osc_cali_show               = sih_osc_cali_show,
	.ram_update_show             = sih_ram_update_show,
	.ram_vbat_comp_show          = sih_ram_vbat_comp_show,
	.lra_resistance_show         = sih_lra_resistance_show,
	.f0_save_show                = sih_f0_save_show,
	.activate_show               = sih_activate_show,
	.drv_vboost_show             = sih_drv_vboost_show,
	.detect_vbat_show            = sih_detect_vbat_show,
	.audio_delay_show            = sih_audio_delay_show,
	.osc_data_show               = sih_osc_data_show,
	.f0_data_show                = sih_f0_data_show,
	.oplus_brightness_show       = sih_brightness_show,
	.oplus_duration_show         = sih_duration_show,
	.oplus_activate_show         = sih_activate_show,
	.oplus_state_show            = sih_state_show,
	.vmax_show                   = sih_vmax_show,
	.waveform_index_show         = sih_waveform_index_show ,
	.device_id_show              = sih_device_id_show      ,
	.livetap_support_show        = sih_livetap_support_show,
	.ram_test_show               = sih_ram_test_show,
	.rtp_going_show              = sih_rtp_going_show,
	.gun_type_show               = sih_gun_type_show,
	.gun_mode_show               = sih_gun_mode_show,
	.bullet_nr_show              = sih_bullet_nr_show,


	.cali_store                   = sih_cali_store,
	.f0_store                     = sih_f0_store,
	.seq_store                    = sih_seq_store,
	.reg_store                    = sih_reg_store,
	.gain_store                   = sih_gain_store,
	.state_store                  = sih_state_store,
	.rtp_store                    = sih_rtp_store,
	.duration_store               = sih_duration_store,
	.osc_cali_store               = sih_osc_cali_store,
	.ram_store                    = sih_ram_store,
	.ram_update_store             = sih_ram_update_store,
	.ram_vbat_comp_store          = sih_ram_vbat_comp_store,
	.lra_resistance_store         = sih_lra_resistance_store ,
	.f0_save_store                = sih_f0_save_store,
	.activate_store               = sih_activate_store,
	.drv_vboost_store             = sih_drv_vboost_store,
	.audio_delay_store            = sih_audio_delay_store,
	.osc_data_store               = sih_osc_data_store,
	.f0_data_store                = sih_f0_data_store,
	.oplus_brightness_store       = sih_brightness_store,
	.oplus_duration_store         = sih_duration_store ,
	.oplus_activate_store         = sih_activate_store ,
	.oplus_state_store            = sih_state_store,
	.vmax_store                   = sih_vmax_store,
	.waveform_index_store         = sih_waveform_index_store,
	.device_id_store              = sih_device_id_store,
	.livetap_support_store        = sih_livetap_support_store,
	.ram_test_store               = sih_ram_test_store,
	.rtp_going_store              = sih_rtp_going_store,
	.gun_type_store               = sih_gun_type_store,
	.gun_mode_store               = sih_gun_mode_store,
	.bullet_nr_store              = sih_bullet_nr_store,

	.haptic_get_f0                = sih_get_f0,
	.haptic_get_rtp_file_num      = sih_get_rtp_file_num,
	.haptic_play_stop             = sih_play_stop,
	.haptic_rtp_mode              = sih_enter_rtp_mode,
	.haptic_set_gain              = sih_set_gain,
	.haptic_stream_mode           = sih_stream_mode,
	.haptic_stop_mode             = sih_stop_mode,

	.haptic_set_wav_seq           = sih_set_wav_seq,
	.haptic_set_wav_loop          = sih_set_wav_loop,
	.haptic_set_drv_bst_vol       = sih_set_drv_bst_vol,
	.haptic_play_go               = sih_play_go,
	.haptic_play_mode             = sih_play_mode,
	.haptic_set_rtp_aei           = sih_set_rtp_aei,
	.haptic_clear_interrupt_state = sih_clear_interrupt_state,
	.haptic_rtp_work              = sih_rtp_work,
	.haptic_virt_to_phys          = sih_virt_to_phys,
	.haptic_mutex_lock            = sih_mutex_lock,
	.haptic_mutex_unlock          = sih_mutex_unlock,
};

#ifdef OPLUS_FEATURE_CHG_BASIC
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static ssize_t oplus_strlcpy(char *dst, const char *src, ssize_t siz) {
	char *d = dst;
	const char *s = src;
	ssize_t n = siz;

	/* Copy as many bytes as will fit */
	if (n != 0) {
		while (--n != 0) {
			if ((*d++ = *s++) == '\0')
				break;
		}
	}

	/* Not enough room in dst, add NULL and traverse rest of src */
	if (n == 0) {
		if (siz != 0)
			*d = '\0';			/* NULL-terminate dst */
		while (*s++)
			;
	}

	return (s - src - 1);		/* count does not include NUL */
}
#endif
#endif

static int sih_parse_dts(struct device *dev, sih_haptic_t *sih_haptic,
	struct device_node *np)
{
	struct device_node *sih_node = np;
	const char *str = NULL;
	int ret = -1;

	if (sih_node == NULL) {
		hp_err("%s:haptic device node acquire failed\n", __func__);
		return -EINVAL;
	}
		/* acquire lra msg */
	ret = of_property_read_string(np, "lra_name", &str);
	if (ret) {
		hp_err("%s:lra name acquire failed\n", __func__);
	} else {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
		oplus_strlcpy(sih_haptic->chip_attr.lra_name, str, SIH_LRA_NAME_LEN);
#else
		strlcpy(sih_haptic->chip_attr.lra_name, str, SIH_LRA_NAME_LEN);
#endif
		hp_info("%s:lra_name = %s\n", __func__, sih_haptic->chip_attr.lra_name);
	}

	return 0;
}

static int sih_auto_break_config_regs(sih_haptic_t *sih_haptic)
{
	struct device_node *sih_node = sih_haptic->i2c->dev.of_node;
	uint8_t brk_addr_regs[20] = {0};
	uint8_t brk_addr_val[20] = {0};
	int addr_reg_nums = 0;
	int addr_val_nums = 0;
	int ret = -1;
	int i = 0;

	if (sih_node == NULL) {
		hp_err("%s:haptic device node acquire failed\n", __func__);
		return -EINVAL;
	}

	addr_reg_nums = of_property_count_elems_of_size(sih_node, "oplus,brk_addr_regs", sizeof(uint8_t));
	hp_info("%s: brk addr_reg_nums = %d\n", __func__, addr_reg_nums);
	/* read brk regs addr config from dts*/
	ret = of_property_read_u8_array(sih_node, "oplus,brk_addr_regs", brk_addr_regs, addr_reg_nums);
	if (ret != 0) {
		hp_err("%s: brk regs nums acquire failed\n", __func__);
		return -1;
	}

	addr_val_nums = of_property_count_elems_of_size(sih_node, "oplus,brk_addr_val", sizeof(uint8_t));
	hp_info("%s: brk addr_val_nums = %d\n", __func__, addr_val_nums);
	/* read brk regs val config from dts */
	ret = of_property_read_u8_array(sih_node, "oplus,brk_addr_val", brk_addr_val, addr_val_nums);
	if (ret != 0) {
		hp_err("%s: brk regs value acquire failed\n", __func__);
		return -1;
	}
	if (addr_reg_nums != addr_val_nums) {
		hp_err("%s: brk regs nums acquire failed\n", __func__);
		return -1;
	}

	for (i = 0; i < addr_reg_nums; i++) {
		haptic_regmap_write(sih_haptic->regmapp.regmapping, brk_addr_regs[i],
			SIH_I2C_OPERA_BYTE_ONE, &brk_addr_val[i]);
		hp_info("%s: brk write reg = %x, val = %x\n", __func__, brk_addr_regs[i], brk_addr_val[i]);
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static int sih_i2c_probe(struct i2c_client *i2c)
#else
static int sih_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
#endif
{
    haptic_common_data_t *oh = NULL;
    sih_haptic_t *sih_haptic = NULL;
	struct device_node *np = i2c->dev.of_node;
	int ret = -1;

	hp_info("%s:haptic i2c probe enter\n", __func__);
    /* 1. Alloc chip_info */
    sih_haptic = devm_kzalloc(&i2c->dev, sizeof(sih_haptic_t), GFP_KERNEL);
	if (sih_haptic == NULL) {
		hp_err("%s:sih_haptic is null\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_sih_haptic;
	}
	/* 2. Alloc common oh */
	oh = common_haptic_data_alloc();
	if (oh == NULL) {
		hp_err("oh kzalloc error\n");
		ret = -ENOMEM;
		goto oh_malloc_failed;
	}
	oh->haptic_common_ops = &si_haptic_ops;
	oh->i2c = i2c;
	oh->dev = &i2c->dev;
	oh->chip_data = sih_haptic;
	sih_haptic->dev = &i2c->dev;
	sih_haptic->i2c = i2c;
	i2c_set_clientdata(i2c, oh);
	/* matching dts */
	/*step1 parse sih_haptic resource */
	ret = sih_parse_dts(&i2c->dev, sih_haptic, np);
	if (ret) {
		hp_err("%s:dts parse failed\n", __func__);
		goto err_parse_dts;
	}
	mutex_lock(&rst_mutex);
	ret = register_common_haptic_device(oh);
	if(ret) {
		mutex_unlock(&rst_mutex);
		goto err_register_driver;
	}
	mutex_unlock(&rst_mutex);
	/* ram work init */
	schedule_work(&sih_haptic->ram.ram_update_work);
	hp_info("%s:end\n", __func__);
	return 0;

err_register_driver:
err_parse_dts:
	common_haptic_data_free(oh);
oh_malloc_failed:
	devm_kfree(&i2c->dev, sih_haptic);
err_alloc_sih_haptic:
	sih_haptic = NULL;
	hp_err("%s:probe error\n", __func__);
	return ret;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
static int sih_i2c_remove(struct i2c_client *i2c)
{
    haptic_common_data_t *oh = i2c_get_clientdata(i2c);
    sih_haptic_t *sih_haptic = oh->chip_data;
	hp_info("%s:end\n", __func__);
	/* work_struct release */
	cancel_work_sync(&sih_haptic->ram.ram_work);
	cancel_work_sync(&sih_haptic->rtp.rtp_work);
	cancel_work_sync(&sih_haptic->ram.ram_update_work);
	//cancel_work_sync(&sih_haptic->motor_old_test_work);
	/* hrtimer release */
	hrtimer_cancel(&sih_haptic->timer);
	/* mutex release */
	mutex_destroy(&sih_haptic->lock);
	mutex_destroy(&sih_haptic->rtp.rtp_lock);
	/* gpio release */
	if (gpio_is_valid(sih_haptic->chip_attr.irq_gpio))
		devm_gpio_free(&i2c->dev, sih_haptic->chip_attr.irq_gpio);
	if (gpio_is_valid(sih_haptic->chip_attr.reset_gpio))
		devm_gpio_free(&i2c->dev, sih_haptic->chip_attr.reset_gpio);
	/* irq release*/
	unregister_common_haptic_device(oh);
	/* regmap exit */
	haptic_regmap_remove(sih_haptic->regmapp.regmapping);
	/* container release */
	sih_vfree_container(sih_haptic, sih_haptic->rtp.rtp_cont);
	sih_haptic->rtp.rtp_cont = NULL;
	/* reg addr release */
	if (sih_haptic->chip_reg.reg_addr != NULL)
		kfree(sih_haptic->chip_reg.reg_addr);
	sih_haptic->stream_func->stream_rtp_work_release(sih_haptic);
	devm_kfree(&i2c->dev, sih_haptic);
	return 0;
}
#else
static void sih_i2c_remove(struct i2c_client *i2c)
{
    haptic_common_data_t *oh = i2c_get_clientdata(i2c);
    sih_haptic_t *sih_haptic = oh->chip_data;
	hp_info("%s:end\n", __func__);
	/* work_struct release */
	cancel_work_sync(&sih_haptic->ram.ram_work);
	cancel_work_sync(&sih_haptic->rtp.rtp_work);
	cancel_work_sync(&sih_haptic->ram.ram_update_work);
	/* hrtimer release */
	hrtimer_cancel(&sih_haptic->timer);
	/* mutex release */
	mutex_destroy(&sih_haptic->lock);
	mutex_destroy(&sih_haptic->rtp.rtp_lock);
	/* irq release */
	unregister_common_haptic_device(oh);
	/* regmap exit */
	haptic_regmap_remove(sih_haptic->regmapp.regmapping);
	/* container release */
	sih_vfree_container(sih_haptic, sih_haptic->rtp.rtp_cont);
	sih_haptic->rtp.rtp_cont = NULL;
	/* reg addr release */
	if (sih_haptic->chip_reg.reg_addr != NULL)
		kfree(sih_haptic->chip_reg.reg_addr);
	sih_haptic->stream_func->stream_rtp_work_release(sih_haptic);
	return;
}

#endif

static const struct i2c_device_id sih_i2c_id[] = {
	{SIH_HAPTIC_NAME_688X, 0},
	{},
};

static struct of_device_id sih_dt_match[] = {
	{.compatible = SIH_HAPTIC_COMPAT_688X},
	{},
};

static struct i2c_driver sih_i2c_driver = {
	.driver = {
		.name = SIH_HAPTIC_NAME_688X,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sih_dt_match),
	},
	.probe = sih_i2c_probe,
	.remove = sih_i2c_remove,
	.id_table = sih_i2c_id,
};

int sih_i2c_init(void)
{
	int ret = -1;

	ret = i2c_add_driver(&sih_i2c_driver);

	hp_info("%s:i2c_add_driver,ret = %d\n", __func__, ret);

	if (ret) {
		hp_err("%s:fail to add haptic device,ret = %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

void sih_i2c_exit(void)
{
	i2c_del_driver(&sih_i2c_driver);
}

MODULE_DESCRIPTION("Haptic Driver V1.0.3.691");
MODULE_LICENSE("GPL v2");
#if defined(CONFIG_OPLUS_VIBRATOR_GKI_ENABLE)
MODULE_SOFTDEP("pre: aw_haptic");
#endif
