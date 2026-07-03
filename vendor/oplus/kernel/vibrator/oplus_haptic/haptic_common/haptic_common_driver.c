/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 . All rights reserved.
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/thermal.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/iio/consumer.h>
#include <linux/alarmtimer.h>
#include "haptic_common.h"
#include "haptic_wave.h"
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include "haptic_hv_rtp_key_data.h"

#ifdef OPLUS_FEATURE_CHG_BASIC
#include <soc/oplus/boot/boot_mode.h>
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mt-plat/mtk_boot_common.h>
#endif
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
#include "../../aw8697_haptic/haptic_feedback.h"
#endif
#endif

typedef struct led_classdev cdev_t;
haptic_common_data_t *g_oh;
DEFINE_MUTEX(rst_mutex);

static inline void *haptic_kzalloc(size_t size, gfp_t flags)
{
	void *p;

	p = kzalloc(size, flags);

	if (!p) {
		pr_err("%s: Failed to allocate memory\n", __func__);
	}

	return p;
}

struct haptic_common_data *common_haptic_data_alloc(void)
{
	return haptic_kzalloc(sizeof(struct haptic_common_data), GFP_KERNEL);
}

static int init_parse_dts(struct device *dev, struct haptic_common_data *oh,struct device_node *np)
{
	if (np == NULL) {
		oh_err("%s:haptic device node acquire failed\n", __func__);
		return -EINVAL;
	}
#ifdef COMMON_ENABLE_PIN_CONTROL
	oh->pinctrl =  (struct pinctrl *)devm_pinctrl_get(dev);
	if(!IS_ERR_OR_NULL(oh->pinctrl)){
		oh->pinctrl_state = (struct pinctrl_state *)pinctrl_lookup_state(oh->pinctrl,
							"irq_active");
		if (!IS_ERR_OR_NULL(oh->pinctrl_state)){
			pinctrl_select_state(oh->pinctrl,
						oh->pinctrl_state);
		} else {
			oh_err("%s: pinctrl_state error!\n", __func__);
			devm_pinctrl_put(oh->pinctrl);
		}
	} else {
		oh_err("%s: pinctrl error!\n", __func__);
		devm_pinctrl_put(oh->pinctrl);
	}
#endif
		/* acquire reset gpio */
	oh->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (oh->reset_gpio < 0) {
		oh_err("%s:reset gpio acquire failed\n", __func__);
		return -EIO;
	}

	/* acquire irq gpio */
	oh->irq_gpio =
		of_get_named_gpio(np, "irq-gpio", 0);
	if (oh->irq_gpio < 0) {
		oh_err("%s:irq gpio acquire failed\n", __func__);
		return -EIO;
	}

	oh_info("%s:reset_gpio = %d, irq_gpio = %d\n", __func__,
		oh->reset_gpio, oh->irq_gpio);

	if (of_property_read_u32(np, "qcom,device_id", &oh->device_id))
		oh->device_id = 815;
	oh_info("%s: device_id=%d\n", __func__, oh->device_id);

	oh->livetap_support = of_property_read_bool(np, "oplus,livetap_support");
	oh_info("%s: oplus,livetap_support = %d\n", __func__, oh->livetap_support);

	oh->auto_break_mode_support = of_property_read_bool(np, "oplus,auto_break_mode_support");
	oh_info("oplus,auto_break_mode_support = %d\n", oh->auto_break_mode_support);

	if (of_property_read_u32(np, "oplus,vbat_low_soc", &oh->vbat_low_soc)) {
		oh_info("vbat_low_soc not found");
		oh->vbat_low_soc = 0;
	}

	oh_info("%s: vbat_low_soc=%d\n", __func__, oh->vbat_low_soc);

	if (of_property_read_u32(np, "oplus,vbat_low_soc_cold", &oh->vbat_low_soc_cold)) {
		oh_info("vbat_low_soc_cold not found");
		oh->vbat_low_soc_cold = 0;
	}

	oh_info("%s: vbat_low_soc_cold=%d\n", __func__, oh->vbat_low_soc_cold);

	if (of_property_read_s32(np, "oplus,vbat_low_temp", &oh->vbat_low_temp)) {
		oh_info("vbat_low_temp not found");
		oh->vbat_low_temp = 0;
	}

	oh_info("%s: vbat_low_temp=%d\n", __func__, oh->vbat_low_temp);


	if (of_property_read_u32(np, "oplus,vbat_low_vmax_level", &oh->vbat_low_vmax_level)) {
		oh_info("vbat_low_vmax_level not found");
		oh->vbat_low_vmax_level = 0;
	}

	oh_info("%s: vbat_low_vmax_level=%d\n", __func__, oh->vbat_low_vmax_level);

	return 0;
}

static int haptic_acquire_gpio_res(struct device *dev,struct haptic_common_data *oh)
{
	int ret = -1;
	if (gpio_is_valid(oh->irq_gpio)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
		ret = devm_gpio_request_one(dev, oh->irq_gpio,
			GPIOF_IN, "haptic_irq");
#else
		ret = devm_gpio_request_one(dev, oh->irq_gpio,
			GPIOF_DIR_IN, "haptic_irq");
#endif
		if (ret) {
			oh_err("%s:irq gpio request failed,ret = %d\n", __func__,ret);
			return ret;
		}
	}
	if (gpio_is_valid(oh->reset_gpio)) {
		ret = devm_gpio_request_one(dev, oh->reset_gpio,
			GPIOF_OUT_INIT_LOW, "haptic_rst");
		if (ret) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
			devm_gpio_free(dev, oh->irq_gpio);
#endif
			oh_err("%s:reset gpio request failed,ret = %d\n", __func__,ret);
			return ret;
		}
	}
	return ret;
}
static irqreturn_t haptic_irq_thread_fn(int irq, void *dev_id)
{
	struct haptic_common_data *oh = (struct haptic_common_data *)dev_id;
	return oh->haptic_common_ops->chip_irq_isr(irq,oh->chip_data);
}
static int haptic_acquire_irq_res(struct device *dev, struct haptic_common_data *oh)
{
	int ret = -1;
	int irq_flags;

	oh->haptic_common_ops->chip_interrupt_init(oh->chip_data);

	irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

	ret = devm_request_threaded_irq(dev,
		gpio_to_irq(oh->irq_gpio), NULL, haptic_irq_thread_fn,
		irq_flags, "vibrator", oh);
	if (ret) {
		oh_err("%s:request_threaded_irq fail!\n", __func__);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
		devm_gpio_free(&i2c->dev, oh->irq_gpio);
		devm_gpio_free(&i2c->dev, oh->reset_gpio);
#endif
	}
	return ret;
}

static int haptic_common_init(struct haptic_common_data *oh)
{
	return oh->haptic_common_ops->haptic_init(oh->chip_data);
}
static enum led_brightness haptic_brrightness_get(
	struct led_classdev *cdev)
{
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->haptic_brightness_get(oh->chip_data);
}
static void haptic_brrightness_set(struct led_classdev *cdev,enum led_brightness level)
{
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->haptic_brightness_set(level,oh->chip_data);
}


static ssize_t cali_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->cali_show(oh->chip_data,buf);
}

static ssize_t cali_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	oh->haptic_common_ops->cali_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t f0_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->f0_show(oh->chip_data,buf);
}

static ssize_t f0_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	oh_info("%s: f0 = %d\n", __func__, val);

	oh->f0 = val;
	val = haptic_common_get_f0();
	oh->haptic_common_ops->f0_store(oh->chip_data,buf,val);
    return count;
}

static ssize_t seq_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->seq_show(oh->chip_data,buf);
}

static ssize_t seq_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->seq_store(oh->chip_data,buf);
	return count;
}

static ssize_t reg_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->reg_show(oh->chip_data,buf);
}

static ssize_t reg_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->reg_store(oh->chip_data,buf);
	return count;
}

static ssize_t gain_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->gain_show(oh->chip_data,buf);
}

static ssize_t gain_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val > HAPTIC_MAX_GAIN) {
		oh_err("%s:gain out of range!\n", __func__);
		return count;
	}
	oh_info("%s: value=0x%02x\n", __func__, val);

	oh->haptic_common_ops->gain_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->state_show(oh->chip_data,buf);
}

static ssize_t state_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->state_store(oh->chip_data,buf);
	return count;
}

static ssize_t rtp_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->rtp_show(oh->chip_data,buf);
}

static ssize_t rtp_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		oh_err("%s: kstrtouint fail\n", __func__);
		return rc;
	}
	oh_info("%s: val [%d] \n", __func__, val);
	oh->haptic_common_ops->rtp_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t ram_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->ram_store(oh->chip_data,buf);
	return count;
}

static ssize_t duration_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->duration_show(oh->chip_data,buf);
}

static ssize_t duration_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	oh_info("%s: value=%d\n", __func__, val);

	oh->haptic_common_ops->duration_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t osc_cali_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->osc_cali_show(oh->chip_data,buf);
}

static ssize_t osc_cali_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->osc_cali_store(oh->chip_data,buf);
	return count;
}



static ssize_t ram_update_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->ram_update_show(oh->chip_data,buf);
}

static ssize_t ram_update_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	oh_info("%s:ram update is %d\n", __func__, val);
	oh->haptic_common_ops->ram_update_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t ram_vbat_comp_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->ram_vbat_comp_show(oh->chip_data,buf);
}

static ssize_t ram_vbat_comp_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	oh->haptic_common_ops->ram_vbat_comp_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t lra_resistance_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->lra_resistance_show(oh->chip_data,buf);
}

static ssize_t lra_resistance_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->lra_resistance_store(oh->chip_data,buf);
	return count;
}

static ssize_t f0_save_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->f0_save_show(oh->chip_data,buf);
}

static ssize_t f0_save_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->f0_save_store(oh->chip_data,buf);
	return count;
}


static ssize_t activate_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->activate_show(oh->chip_data,buf);
}

static ssize_t activate_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val < 0)
		return 0;

	oh_info("%s:value = %d\n", __func__, val);
	if (val != 0 && val != 1) {
		oh_err("%s: error val, return!\n", __func__);
		return 0;
	}
	oh->haptic_common_ops->activate_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t drv_vboost_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->drv_vboost_show(oh->chip_data,buf);
}

static ssize_t drv_vboost_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->drv_vboost_store(oh->chip_data,buf);
	return count;
}

static ssize_t detect_vbat_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->detect_vbat_show(oh->chip_data,buf);
}

static ssize_t audio_delay_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->audio_delay_show(oh->chip_data,buf);
}

static ssize_t audio_delay_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->audio_delay_store(oh->chip_data,buf);
	return count;
}

static ssize_t osc_data_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->osc_data_show(oh->chip_data,buf);
}

static ssize_t osc_data_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->osc_data_store(oh->chip_data,buf);
	return count;
}

static ssize_t f0_data_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->f0_data_show(oh->chip_data,buf);
}

static ssize_t f0_data_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->f0_data_store(oh->chip_data,buf);
	return count;
}


static ssize_t oplus_brightness_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->oplus_brightness_show(oh->chip_data,buf);
}

static ssize_t oplus_brightness_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	oh_info("%s: enter,val:%d\n", __func__, val);
	oh->haptic_common_ops->oplus_brightness_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t oplus_duration_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->oplus_duration_show(oh->chip_data,buf);
}

static ssize_t oplus_duration_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	oh_info("%s: value=%d\n", __func__, val);
	oh->haptic_common_ops->oplus_duration_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t oplus_activate_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->oplus_activate_show(oh->chip_data,buf);
}

static ssize_t oplus_activate_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	oh_info("%s: value=%d\n", __func__, val);
	oh->haptic_common_ops->oplus_activate_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t oplus_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->oplus_state_show(oh->chip_data,buf);
}

static ssize_t oplus_state_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->oplus_state_store(oh->chip_data,buf);
	return count;
}

static ssize_t vmax_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->vmax_show(oh->chip_data,buf);
}

static ssize_t vmax_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	oh_info("%s: value=%d\n", __func__, val);
	oh->haptic_common_ops->vmax_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t motor_old_test_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return 0;
}

static void motor_old_test_work(struct work_struct *work)
{
	struct haptic_common_data *oh = container_of(work, struct haptic_common_data, motor_old_test_work);

	if (!oh) {
		oh_err("%s: oh is null\n", __func__);
		return;
	}
	oh_err("%s: motor_old_test_mode = %d. gain[0x%02x]\n", __func__,
			oh->motor_old_test_mode, oh->gain);

	if (oh->motor_old_test_mode == MOTOR_OLD_TEST_TRANSIENT) {
		oh->haptic_common_ops->haptic_mutex_lock(oh->chip_data);

		oh->haptic_common_ops->haptic_play_stop(oh->chip_data);
		oh->gain = 0x80;
		oh->haptic_common_ops->haptic_set_gain(oh->chip_data, oh->gain);
		oh->haptic_common_ops->haptic_set_drv_bst_vol(oh->chip_data);
		oh->haptic_common_ops->haptic_set_wav_seq(oh->chip_data, 0,
					     HAPTIC_WAVEFORM_INDEX_TRANSIENT);
		oh->haptic_common_ops->haptic_set_wav_loop(oh->chip_data, 0, 0);
		oh->haptic_common_ops->haptic_play_mode(oh->chip_data, HAPTIC_RAM_MODE);
		oh->haptic_common_ops->haptic_play_go(oh->chip_data, true);
		oh->haptic_common_ops->haptic_mutex_unlock(oh->chip_data);
	} else if (oh->motor_old_test_mode == MOTOR_OLD_TEST_STEADY) {
		oh->haptic_common_ops->haptic_mutex_lock(oh->chip_data);
		oh->haptic_common_ops->haptic_play_stop(oh->chip_data);
		oh->gain = 0x80;
		oh->haptic_common_ops->haptic_set_gain(oh->chip_data, oh->gain);
		oh->haptic_common_ops->haptic_set_drv_bst_vol(oh->chip_data);
		oh->haptic_common_ops->haptic_set_rtp_aei(oh->chip_data, false);
		oh->haptic_common_ops->haptic_clear_interrupt_state(oh->chip_data);
		oh->haptic_common_ops->haptic_mutex_unlock(oh->chip_data);
		if (HAPTIC_WAVEFORM_INDEX_OLD_STEADY < NUM_WAVEFORMS) {
			oh->rtp_file_num = HAPTIC_WAVEFORM_INDEX_OLD_STEADY;
			if (HAPTIC_WAVEFORM_INDEX_OLD_STEADY) {
				/* schedule_work(&aw_haptic->rtp_work); */
				oh->haptic_common_ops->haptic_rtp_work(oh->chip_data, oh->rtp_file_num);
			}
		} else {
			oh_err("%s: rtp_file_num 0x%02x over max value\n",
				   __func__, oh->rtp_file_num);
		}
	} else if (oh->motor_old_test_mode ==
		   MOTOR_OLD_TEST_HIGH_TEMP_HUMIDITY) {
		oh->haptic_common_ops->haptic_mutex_lock(oh->chip_data);
		oh->haptic_common_ops->haptic_play_stop(oh->chip_data);
		oh->gain = 0x80;
		oh->haptic_common_ops->haptic_set_gain(oh->chip_data, oh->gain);
		oh->haptic_common_ops->haptic_set_drv_bst_vol(oh->chip_data);
		oh->haptic_common_ops->haptic_set_rtp_aei(oh->chip_data, false);
		oh->haptic_common_ops->haptic_clear_interrupt_state(oh->chip_data);
		oh->haptic_common_ops->haptic_mutex_unlock(oh->chip_data);
		if (HAPTIC_WAVEFORM_INDEX_HIGH_TEMP < NUM_WAVEFORMS) {
			oh->rtp_file_num = HAPTIC_WAVEFORM_INDEX_HIGH_TEMP;
			if (HAPTIC_WAVEFORM_INDEX_HIGH_TEMP) {
				/* schedule_work(&aw_haptic->rtp_work); */
				oh->haptic_common_ops->haptic_rtp_work(oh->chip_data, oh->rtp_file_num);
			}
		} else {
			oh_err("%s: rtp_file_num 0x%02x over max value\n",
				   __func__, oh->rtp_file_num);
		}
	} else if (oh->motor_old_test_mode == MOTOR_OLD_TEST_LISTEN_POP) {
		oh->haptic_common_ops->haptic_mutex_lock(oh->chip_data);
		oh->haptic_common_ops->haptic_play_stop(oh->chip_data);
		oh->gain = 0x80;
		oh->haptic_common_ops->haptic_set_gain(oh->chip_data, oh->gain);
		oh->haptic_common_ops->haptic_set_drv_bst_vol(oh->chip_data);
		oh->haptic_common_ops->haptic_set_rtp_aei(oh->chip_data, false);
		oh->haptic_common_ops->haptic_clear_interrupt_state(oh->chip_data);
		oh->haptic_common_ops->haptic_mutex_unlock(oh->chip_data);
		if (HAPTIC_WAVEFORM_INDEX_LISTEN_POP < NUM_WAVEFORMS) {
			oh->rtp_file_num = HAPTIC_WAVEFORM_INDEX_LISTEN_POP;
			if (HAPTIC_WAVEFORM_INDEX_LISTEN_POP) {
				/* schedule_work(&aw_haptic->rtp_work); */
				oh->haptic_common_ops->haptic_rtp_work(oh->chip_data, oh->rtp_file_num);

			}
		} else {
			oh_err("%s: rtp_file_num 0x%02x over max value\n",
				   __func__, oh->rtp_file_num);
		}
	} else {
		oh->motor_old_test_mode = 0;
		oh->haptic_common_ops->haptic_mutex_lock(oh->chip_data);
		oh->haptic_common_ops->haptic_play_stop(oh->chip_data);
		oh->haptic_common_ops->haptic_mutex_unlock(oh->chip_data);
	}
}

static ssize_t motor_old_test_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);

	unsigned int databuf[1] = {0};
	if (!oh) {
		oh_err("%s: oh is null\n", __func__);
		return count;
	}

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		if (databuf[0] == 0) {
			cancel_work_sync(&oh->motor_old_test_work);
			oh->haptic_common_ops->haptic_mutex_lock(oh->chip_data);
			oh->haptic_common_ops->haptic_play_stop(oh->chip_data);
			oh->haptic_common_ops->haptic_mutex_unlock(oh->chip_data);
		} else if (databuf[0] <= MOTOR_OLD_TEST_ALL_NUM) {
			cancel_work_sync(&oh->motor_old_test_work);
			oh->motor_old_test_mode = databuf[0];
			oh_err("%s: motor_old_test_mode = %d.\n", __func__,
				oh->motor_old_test_mode);
			schedule_work(&oh->motor_old_test_work);
		}
	}
	return count;
}

static ssize_t waveform_index_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->waveform_index_show(oh->chip_data,buf);
}

static ssize_t waveform_index_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->waveform_index_store(oh->chip_data,buf);
	return count;
}


static ssize_t device_id_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->device_id_show(oh->chip_data,buf);
}

static ssize_t device_id_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->device_id_store(oh->chip_data,buf);
	return count;
}

static ssize_t livetap_support_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->livetap_support_show(oh->chip_data,buf);
}

static ssize_t livetap_support_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	oh->haptic_common_ops->livetap_support_store(oh->chip_data,buf,val);
	return count;
}

static ssize_t ram_test_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->ram_test_show(oh->chip_data,buf);
}

static ssize_t ram_test_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->ram_test_store(oh->chip_data,buf);
	return count;
}


static ssize_t rtp_going_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->rtp_going_show(oh->chip_data,buf);
}

static ssize_t rtp_going_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->rtp_going_store(oh->chip_data,buf);
	return count;
}

static ssize_t gun_type_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->gun_type_show(oh->chip_data,buf);
}

static ssize_t gun_type_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->gun_type_store(oh->chip_data,buf);
	return count;
}

static ssize_t gun_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->gun_mode_show(oh->chip_data,buf);
}

static ssize_t gun_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->gun_mode_store(oh->chip_data,buf);
	return count;
}


static ssize_t bullet_nr_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	return oh->haptic_common_ops->bullet_nr_show(oh->chip_data,buf);
}

static ssize_t bullet_nr_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	oh->haptic_common_ops->bullet_nr_store(oh->chip_data,buf);
	return count;
}

static ssize_t activate_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	if (oh->haptic_common_ops->activate_mode_show)
		return oh->haptic_common_ops->activate_mode_show(oh->chip_data,buf);
	return 0;
}

static ssize_t activate_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	if (oh->haptic_common_ops->activate_mode_store)
		oh->haptic_common_ops->activate_mode_store(oh->chip_data,buf);
	return count;
}

static ssize_t index_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	if (oh->haptic_common_ops->index_show)
		return oh->haptic_common_ops->index_show(oh->chip_data,buf);
	return 0;
}

static ssize_t index_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	if (oh->haptic_common_ops->index_store)
		oh->haptic_common_ops->index_store(oh->chip_data,buf);
	return count;
}

static ssize_t loop_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	if (oh->haptic_common_ops->loop_show)
		return oh->haptic_common_ops->loop_show(oh->chip_data,buf);
	return 0;
}

static ssize_t loop_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	if (oh->haptic_common_ops->loop_store)
		oh->haptic_common_ops->loop_store(oh->chip_data,buf);
	return count;
}

static ssize_t vbat_low_soc_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	int rc = 0;
	int batt_soc;
	int batt_temp;

	rc = read_batt_soc(&batt_soc);
	read_batt_temp(&batt_temp);
	oh_err("%s: batt_soc %d, batt_temp %d\n",
			__func__, batt_soc, batt_temp);
	oh_err("%s: vbat_low_soc %d, vbat_low_temp %d, vbat_low_vmax_level %d\n",
			__func__, oh->vbat_low_soc, oh->vbat_low_temp, oh->vbat_low_vmax_level);
	return rc;
}

static ssize_t vbat_low_soc_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	unsigned int data[3] = { 0, 0, 0};

	if (sscanf(buf, "%d %d %d", &data[0], &data[1], &data[2]) == 3) {
		if (data[0] > HAPTIC_MAX_VBAT_SOC ||
			data[1] > HAPTIC_MAX_LEVEL) {
			oh_err("%s: input value out of range!\n", __func__);
			return -EINVAL;
		}
		oh->vbat_low_soc = data[0];
		oh->vbat_low_temp = data[1];
		oh->vbat_low_vmax_level = data[2];
	}
	return count;
}

static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO,cali_show, cali_store);
static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO,f0_show, f0_store);
static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO,seq_show, seq_store);
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,reg_show, reg_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO,gain_show, gain_store);
static DEVICE_ATTR(state, S_IWUSR | S_IWGRP | S_IRUGO,state_show, state_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO,rtp_show, rtp_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO,NULL, ram_store);
static DEVICE_ATTR(detect_vbat, S_IWUSR | S_IRUGO,detect_vbat_show, NULL);
static DEVICE_ATTR(duration, S_IWUSR | S_IWGRP | S_IRUGO,duration_show, duration_store);
static DEVICE_ATTR(oplus_duration, S_IWUSR | S_IWGRP | S_IRUGO,oplus_duration_show, oplus_duration_store);
static DEVICE_ATTR(osc_cali, S_IWUSR | S_IRUGO,osc_cali_show, osc_cali_store);
static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO,ram_update_show, ram_update_store);
static DEVICE_ATTR(ram_vbat_comp, S_IWUSR | S_IRUGO,ram_vbat_comp_show, ram_vbat_comp_store);
static DEVICE_ATTR(lra_resistance, S_IWUSR | S_IRUGO,lra_resistance_show, lra_resistance_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IWGRP | S_IRUGO,activate_show, activate_store);
static DEVICE_ATTR(oplus_activate, S_IWUSR | S_IWGRP | S_IRUGO,oplus_activate_show, oplus_activate_store);
static DEVICE_ATTR(drv_vboost, S_IWUSR | S_IRUGO,drv_vboost_show, drv_vboost_store);
static DEVICE_ATTR(audio_delay, S_IWUSR | S_IRUGO,audio_delay_show, audio_delay_store);
static DEVICE_ATTR(osc_data, S_IWUSR | S_IRUGO,osc_data_show, osc_data_store);
static DEVICE_ATTR(f0_data, S_IWUSR | S_IRUGO,f0_data_show, f0_data_store);
static DEVICE_ATTR(f0_save, S_IWUSR | S_IRUGO,f0_save_show, f0_save_store);
static DEVICE_ATTR(oplus_brightness, S_IWUSR | S_IWGRP | S_IRUGO, oplus_brightness_show, oplus_brightness_store);
static DEVICE_ATTR(oplus_state, S_IWUSR | S_IWGRP | S_IRUGO, oplus_state_show, oplus_state_store);
static DEVICE_ATTR(motor_old, S_IWUSR | S_IRUGO, motor_old_test_show, motor_old_test_store);
static DEVICE_ATTR(waveform_index, S_IWUSR | S_IRUGO, waveform_index_show, waveform_index_store);
static DEVICE_ATTR(device_id, S_IWUSR | S_IRUGO, device_id_show, device_id_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO,	vmax_show, vmax_store);
static DEVICE_ATTR(livetap_support, S_IWUSR | S_IRUGO, livetap_support_show, livetap_support_store);
static DEVICE_ATTR(ram_test, S_IWUSR | S_IWGRP | S_IRUGO, ram_test_show, ram_test_store);
static DEVICE_ATTR(rtp_going, S_IWUSR | S_IRUGO, rtp_going_show, rtp_going_store);
static DEVICE_ATTR(bullet_nr, S_IWUSR | S_IRUGO, bullet_nr_show, bullet_nr_store);
static DEVICE_ATTR(gun_mode, S_IWUSR | S_IRUGO, gun_mode_show, gun_mode_store);
static DEVICE_ATTR(gun_type, S_IWUSR | S_IRUGO, gun_type_show, gun_type_store);
static DEVICE_ATTR(vbat_low_soc, S_IWUSR | S_IRUGO, vbat_low_soc_show, vbat_low_soc_store);

/*aw*/
static DEVICE_ATTR(activate_mode, S_IWUSR | S_IRUGO, activate_mode_show, activate_mode_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, index_show, index_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, loop_show, loop_store);

static struct attribute *haptic_attribute[] = {
		&dev_attr_cali.attr,
		&dev_attr_f0.attr,
		&dev_attr_seq.attr,
		&dev_attr_reg.attr,
		&dev_attr_gain.attr,
		&dev_attr_state.attr,
		&dev_attr_rtp.attr,
		&dev_attr_ram.attr,
		&dev_attr_duration.attr,
		&dev_attr_osc_cali.attr,
		&dev_attr_ram_update.attr,
		&dev_attr_ram_vbat_comp.attr,
		&dev_attr_lra_resistance.attr,
		&dev_attr_f0_save.attr,
		&dev_attr_activate.attr,
		&dev_attr_drv_vboost.attr,
		&dev_attr_detect_vbat.attr,
		&dev_attr_audio_delay.attr,
		&dev_attr_osc_data.attr,
		&dev_attr_f0_data.attr,
		&dev_attr_oplus_brightness.attr,
		&dev_attr_oplus_duration.attr,
		&dev_attr_oplus_activate.attr,
		&dev_attr_oplus_state.attr,
		&dev_attr_vmax.attr,
		&dev_attr_motor_old.attr,
		&dev_attr_waveform_index.attr,
		&dev_attr_device_id.attr,
		&dev_attr_livetap_support.attr,
		&dev_attr_ram_test.attr,
		&dev_attr_rtp_going.attr,
		&dev_attr_gun_type.attr,
		&dev_attr_gun_mode.attr,
		&dev_attr_bullet_nr.attr,

		&dev_attr_activate_mode.attr,
		&dev_attr_index.attr,
		&dev_attr_loop.attr,
		&dev_attr_vbat_low_soc.attr,
		NULL,
};
static struct attribute_group haptic_attribute_group = {
		.attrs = haptic_attribute,
};
static int haptic_add_sys_node(struct haptic_common_data *oh)
{
	int ret = -1;
	oh->cdev.name = "vibrator";
	oh->cdev.brightness_get = haptic_brrightness_get;
	oh->cdev.brightness_set = haptic_brrightness_set;
	/* led sub system register */
	ret = devm_led_classdev_register(oh->dev,&oh->cdev);
	if (ret < 0) {
		oh_err("%s:dev register failed = %d\n", __func__, ret);
		return ret;
	}
	/* vibrator sysfs node create */
	ret = sysfs_create_group(&oh->cdev.dev->kobj,&haptic_attribute_group);
	if (ret < 0) {
		oh_err("%s:sysfs node create failed = %d\n ", __func__, ret);
		devm_led_classdev_unregister(oh->dev, &oh->cdev);
		return ret;
	}
	return 0;
}

int haptic_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void *)g_oh;

	return 0;
}

static ssize_t haptic_proc_style_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *lo)
{
	haptic_common_data_t *oh = (haptic_common_data_t *)filp->private_data;
	char buffer[5] = { 0 };
	int rc = 0;
	int val;

	if (count >= sizeof(buffer)) {
		return -EFAULT;
	}
	if (buf == NULL) {
		return -EFAULT;
	}
	if (copy_from_user(buffer, buf, count)) {
		oh_err("%s: error.\n", __func__);
		return -EFAULT;
	}

	buffer[count] = '\0';
	oh_err("buffer=%s", buffer);
	rc = kstrtoint(buffer, 0, &val);
	if (rc < 0)
		return rc;
	oh_err("val = %d", val);
	oh->vibration_style = val;
	if (oh->haptic_common_ops->proc_vibration_style_write) {
		oh->haptic_common_ops->proc_vibration_style_write(oh->chip_data,val);
	}
	return count;
}

static ssize_t haptic_proc_style_read(struct file *filp, char __user *buf,
				     size_t count, loff_t *ppos)
{
	haptic_common_data_t *oh = (haptic_common_data_t *)filp->private_data;
	uint8_t ret = 0;
	int style = 0;
	char page[10];

	if (oh == NULL)
		return -EFAULT;
	style = oh->vibration_style;
	oh_err("%s: touch_style=%d\n", __func__, style);
	sprintf(page, "%d\n", style);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));
	return ret;
}

DECLARE_PROC_OPS(haptic_proc_style_ops, haptic_file_open, haptic_proc_style_read, haptic_proc_style_write, NULL);

static int haptic_init_proc(struct haptic_common_data *oh)
{
	int ret = 0;
	oh->prEntry_da = proc_mkdir("vibrator", NULL);
	if (oh->prEntry_da == NULL) {
		ret = -ENOMEM;
		oh_err("%s: Couldn't create vibrator proc entry\n",
			   __func__);
	}
	oh->prEntry_tmp = proc_create_data("touch_style", 0664,
						  oh->prEntry_da,
						  &haptic_proc_style_ops,
						  oh);
	if (oh->prEntry_tmp == NULL) {
		ret = -ENOMEM;
		oh_err("%s: Couldn't create proc entry\n", __func__);
	}
	return ret;
}

static int haptic_file_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long phys;
	haptic_common_data_t *oh = (haptic_common_data_t *)filp->private_data;
	int ret = -1;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 7, 0)
	/* only accept PROT_READ, PROT_WRITE and MAP_SHARED from the API of mmap */
	vm_flags_t vm_flags = calc_vm_prot_bits(PROT_READ|PROT_WRITE, 0);
	vm_flags |= current->mm->def_flags | VM_MAYREAD | VM_MAYWRITE |
		VM_MAYEXEC | VM_SHARED | VM_MAYSHARE;
	if (vma && (pgprot_val(vma->vm_page_prot) !=
		pgprot_val(vm_get_page_prot(vm_flags))))
		return -EPERM;

	if (vma && ((vma->vm_end - vma->vm_start) !=
		(PAGE_SIZE << IOCTL_MMAP_PAGE_ORDER)))
		return -ENOMEM;
#endif
	//phys = virt_to_phys(oh->start_buf);
	phys = oh->haptic_common_ops->haptic_virt_to_phys(oh->chip_data);

	ret = remap_pfn_range(vma, vma->vm_start, (phys >> PAGE_SHIFT),
		(vma->vm_end - vma->vm_start), vma->vm_page_prot);
	if (ret) {
		oh_err("%s:error mmap failed\n", __func__);
		return ret;
	}

	return ret;
}

static int haptic_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static ssize_t haptic_file_read(struct file *filp, char __user *buff,
	size_t len, loff_t *offset)
{
	oh_info("haptic file read\n");
	return len;
}

static ssize_t haptic_file_write(struct file *filp, const char __user *buff,
	size_t len, loff_t *offset)
{
	oh_info("haptic file write\n");
	return len;
}

static long haptic_file_unlocked_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	haptic_common_data_t *oh = (haptic_common_data_t *)filp->private_data;
	uint32_t tmp;
	int ret = 0;

	if (!oh) {
		oh_err("%s: oh is null\n", __func__);
		return -EFAULT;
	}
	oh_info("%s: cmd:0x%x arg:0x%lx\n", __func__, cmd, arg);
	oh->haptic_common_ops->haptic_mutex_lock(oh->chip_data);
	switch (cmd) {
	case IOCTL_GET_HWINFO:
		oh_info("%s:enter get hwinfo\n", __func__);
		tmp = IOCTL_HWINFO;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(uint32_t)))
			ret = -EFAULT;
		break;
	case IOCTL_MODE_RTP_MODE:
		oh_info("%s:enter rtp\n", __func__);
		oh->haptic_common_ops->haptic_play_stop(oh->chip_data);
		if (copy_from_user(oh->rtp_ptr, (void __user *)arg,
			IOCTL_MMAP_BUF_SIZE * IOCTL_MMAP_BUF_SUM)) {
			ret = -EFAULT;
			break;
		}
		tmp = *((uint32_t *)oh->rtp_ptr);
		if (tmp > (IOCTL_MMAP_BUF_SIZE * IOCTL_MMAP_BUF_SUM - 4)) {
			oh_info("%s:rtp mode data len error %d\n", __func__, tmp);
			ret = -EINVAL;
			break;
		}
		oh->haptic_common_ops->haptic_rtp_mode(oh->chip_data,tmp);
		break;
	case IOCTL_OFF_MODE:
		break;
	case IOCTL_GET_F0:
		tmp = oh->haptic_common_ops->haptic_get_f0(oh->chip_data);
		oh_info("%s:enter get f0 = %d\n", __func__, tmp);
		if (copy_to_user((void __user *)arg, &tmp, sizeof(uint32_t)))
			ret = -EFAULT;
		break;
	case IOCTL_SETTING_GAIN:
		oh->haptic_common_ops->haptic_set_gain(oh->chip_data,arg);
		break;
	case IOCTL_STREAM_MODE:
		oh_info("%s:stream mode enter\n", __func__);
		oh->haptic_common_ops->haptic_stream_mode(oh->chip_data);
		break;
	case IOCTL_STOP_MODE:
		oh_info("%s:stop mode enter\n", __func__);
		oh->haptic_common_ops->haptic_stop_mode(oh->chip_data);
		break;
	default:
		break;
	}
	oh_info("%s:mutex_unlock here!\n", __func__);
	oh->haptic_common_ops->haptic_mutex_unlock(oh->chip_data);
	return ret;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = haptic_file_read,
	.write = haptic_file_write,
	.mmap = haptic_file_mmap,
	.unlocked_ioctl = haptic_file_unlocked_ioctl,
	.open = haptic_file_open,
	.release = haptic_file_release,
};

static struct miscdevice oplus_haptic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = OPLUS_DEV_HAPTIC_NAME,
	.fops = &fops,
};

uint8_t *custom_0809_rtp_key_file(uint32_t *data_len)
{
	uint32_t tmp_f0;

	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return NULL;
	}
	if (g_oh->haptic_common_ops == NULL || g_oh->chip_data == NULL) {
		oh_err("%s: haptic_common_ops or chip_data is NULL\n", __func__);
		return NULL;
	}
	if (!g_oh->haptic_common_ops->haptic_get_f0) {
		oh_err("%s: haptic_get_f0 is null\n", __func__);
		return NULL;
	}
	tmp_f0 = g_oh->haptic_common_ops->haptic_get_f0(g_oh->chip_data);
	switch(g_oh->haptic_common_ops->haptic_get_rtp_file_num(g_oh->chip_data)) {
		case SG_INPUT_DOWN_HIGH:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_302_162Hz);
			return aw_haptic_0809_rtp_302_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_302_166Hz);
			return aw_haptic_0809_rtp_302_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_302_170Hz);
			return aw_haptic_0809_rtp_302_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_302_174Hz);
			return aw_haptic_0809_rtp_302_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0809_rtp_302_178Hz);
			return aw_haptic_0809_rtp_302_178Hz;
		}
		break;
		case SG_INPUT_UP_HIGH:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_303_162Hz);
			return aw_haptic_0809_rtp_303_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_303_166Hz);
			return aw_haptic_0809_rtp_303_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_303_170Hz);
			return aw_haptic_0809_rtp_303_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_303_174Hz);
			return aw_haptic_0809_rtp_303_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0809_rtp_303_178Hz);
			return aw_haptic_0809_rtp_303_178Hz;
		}
		break;
		case SG_INPUT_DOWN_LOW:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_304_162Hz);
			return aw_haptic_0809_rtp_304_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_304_166Hz);
			return aw_haptic_0809_rtp_304_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_304_170Hz);
			return aw_haptic_0809_rtp_304_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_304_174Hz);
			return aw_haptic_0809_rtp_304_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0809_rtp_304_178Hz);
			return aw_haptic_0809_rtp_304_178Hz;
		}
		break;
		case SG_INPUT_UP_LOW:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_305_162Hz);
			return aw_haptic_0809_rtp_305_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_305_166Hz);
			return aw_haptic_0809_rtp_305_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_305_170Hz);
			return aw_haptic_0809_rtp_305_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_305_174Hz);
			return aw_haptic_0809_rtp_305_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0809_rtp_305_178Hz);
			return aw_haptic_0809_rtp_305_178Hz;
		}
		break;
		case INPUT_LOW:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_110_162Hz);
			return aw_haptic_0809_rtp_110_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_110_166Hz);
			return aw_haptic_0809_rtp_110_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_110_170Hz);
			return aw_haptic_0809_rtp_110_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_110_174Hz);
			return aw_haptic_0809_rtp_110_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0809_rtp_110_178Hz);
			return aw_haptic_0809_rtp_110_178Hz;
		}
		break;
		case INPUT_MEDI:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_111_162Hz);
			return aw_haptic_0809_rtp_111_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_111_166Hz);
			return aw_haptic_0809_rtp_111_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_111_170Hz);
			return aw_haptic_0809_rtp_111_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_111_174Hz);
			return aw_haptic_0809_rtp_111_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0809_rtp_111_178Hz);
			return aw_haptic_0809_rtp_111_178Hz;
		}
		break;
		case INPUT_HIGH:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_112_162Hz);
			return aw_haptic_0809_rtp_112_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_112_166Hz);
			return aw_haptic_0809_rtp_112_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_112_170Hz);
			return aw_haptic_0809_rtp_112_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0809_rtp_112_174Hz);
			return aw_haptic_0809_rtp_112_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0809_rtp_112_178Hz);
			return aw_haptic_0809_rtp_112_178Hz;
		}
		break;
		default:
			oh_err("%s: can not find rtp file\n", __func__);
			break;
		}

		return NULL;
}

uint8_t *custom_0815_rtp_key_file(uint32_t *data_len)
{
	uint32_t tmp_f0;

	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return NULL;
	}
	if (g_oh->haptic_common_ops == NULL || g_oh->chip_data == NULL) {
		oh_err("%s: haptic_common_ops or chip_data is NULL\n", __func__);
		return NULL;
	}
	if (!g_oh->haptic_common_ops->haptic_get_f0) {
		oh_err("%s: haptic_get_f0 is null\n", __func__);
		return NULL;
	}
	tmp_f0 = g_oh->haptic_common_ops->haptic_get_f0(g_oh->chip_data);
	switch(g_oh->haptic_common_ops->haptic_get_rtp_file_num(g_oh->chip_data)) {
        case SG_INPUT_DOWN_HIGH:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_302_162Hz);
			return aw_haptic_0815_rtp_302_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_302_166Hz);
			return aw_haptic_0815_rtp_302_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_302_170Hz);
			return aw_haptic_0815_rtp_302_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_302_174Hz);
			return aw_haptic_0815_rtp_302_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0815_rtp_302_178Hz);
			return aw_haptic_0815_rtp_302_178Hz;
		}
		break;
		case SG_INPUT_UP_HIGH:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_303_162Hz);
			return aw_haptic_0815_rtp_303_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_303_166Hz);
			return aw_haptic_0815_rtp_303_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_303_170Hz);
			return aw_haptic_0815_rtp_303_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_303_174Hz);
			return aw_haptic_0815_rtp_303_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0815_rtp_303_178Hz);
			return aw_haptic_0815_rtp_303_178Hz;
		}
		break;
		case SG_INPUT_DOWN_LOW:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_304_162Hz);
			return aw_haptic_0815_rtp_304_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_304_166Hz);
			return aw_haptic_0815_rtp_304_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_304_170Hz);
			return aw_haptic_0815_rtp_304_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_304_174Hz);
			return aw_haptic_0815_rtp_304_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0815_rtp_304_178Hz);
			return aw_haptic_0815_rtp_304_178Hz;
		}
		break;
		case SG_INPUT_UP_LOW:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_305_162Hz);
			return aw_haptic_0815_rtp_305_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_305_166Hz);
			return aw_haptic_0815_rtp_305_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_305_170Hz);
			return aw_haptic_0815_rtp_305_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_305_174Hz);
			return aw_haptic_0815_rtp_305_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0815_rtp_305_178Hz);
			return aw_haptic_0815_rtp_305_178Hz;
		}
		break;
		case INPUT_LOW:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_110_162Hz);
			return aw_haptic_0815_rtp_110_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_110_166Hz);
			return aw_haptic_0815_rtp_110_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_110_170Hz);
			return aw_haptic_0815_rtp_110_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_110_174Hz);
			return aw_haptic_0815_rtp_110_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0815_rtp_110_178Hz);
			return aw_haptic_0815_rtp_110_178Hz;
		}
		break;
		case INPUT_MEDI:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_111_162Hz);
			return aw_haptic_0815_rtp_111_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_111_166Hz);
			return aw_haptic_0815_rtp_111_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_111_170Hz);
			return aw_haptic_0815_rtp_111_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_111_174Hz);
			return aw_haptic_0815_rtp_111_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0815_rtp_111_178Hz);
			return aw_haptic_0815_rtp_111_178Hz;
		}
		break;
		case INPUT_HIGH:
		if (tmp_f0 < OPLUS_162HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_112_162Hz);
			return aw_haptic_0815_rtp_112_162Hz;
		} else if (tmp_f0 < OPLUS_166HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_112_166Hz);
			return aw_haptic_0815_rtp_112_166Hz;
		} else if (tmp_f0 < OPLUS_170HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_112_170Hz);
			return aw_haptic_0815_rtp_112_170Hz;
		} else if (tmp_f0 < OPLUS_174HZ_F0) {
			*data_len = sizeof(aw_haptic_0815_rtp_112_174Hz);
			return aw_haptic_0815_rtp_112_174Hz;
		} else {
			*data_len = sizeof(aw_haptic_0815_rtp_112_178Hz);
			return aw_haptic_0815_rtp_112_178Hz;
		}
		break;
		default:
			oh_err("%s: can not find rtp file\n", __func__);
			break;
		}

		return NULL;
}

uint8_t *custom_1419_rtp_key_file(uint32_t *data_len)
{
	uint32_t tmp_f0;

	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return NULL;
	}
	if (g_oh->haptic_common_ops == NULL || g_oh->chip_data == NULL) {
		oh_err("%s: haptic_common_ops or chip_data is NULL\n", __func__);
		return NULL;
	}
	if (!g_oh->haptic_common_ops->haptic_get_f0) {
		oh_err("%s: haptic_get_f0 is null\n", __func__);
		return NULL;
	}
	tmp_f0 = g_oh->haptic_common_ops->haptic_get_f0(g_oh->chip_data);
	switch(g_oh->haptic_common_ops->haptic_get_rtp_file_num(g_oh->chip_data)) {
	case SG_INPUT_DOWN_HIGH:
		if (tmp_f0 <= OPLUS_197HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_302_197Hz);
			return aw_haptic_1419_rtp_302_197Hz;
		} else if (tmp_f0 <= OPLUS_201HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_302_201Hz);
			return aw_haptic_1419_rtp_302_201Hz;
		} else if (tmp_f0 <= OPLUS_205HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_302_205Hz);
			return aw_haptic_1419_rtp_302_205Hz;
		} else if (tmp_f0 <= OPLUS_209HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_302_209Hz);
			return aw_haptic_1419_rtp_302_209Hz;
		} else if (tmp_f0 <= OPLUS_213HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_302_213Hz);
			return aw_haptic_1419_rtp_302_213Hz;
		} else {
			*data_len = sizeof(aw_haptic_1419_rtp_302_205Hz);
			return aw_haptic_1419_rtp_302_205Hz;
		}
		break;
	case SG_INPUT_UP_HIGH:
		if (tmp_f0 <= OPLUS_197HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_303_197Hz);
			return aw_haptic_1419_rtp_303_197Hz;
		} else if (tmp_f0 <= OPLUS_201HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_303_201Hz);
			return aw_haptic_1419_rtp_303_201Hz;
		} else if (tmp_f0 <= OPLUS_205HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_303_205Hz);
			return aw_haptic_1419_rtp_303_205Hz;
		} else if (tmp_f0 <= OPLUS_209HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_303_209Hz);
			return aw_haptic_1419_rtp_303_209Hz;
		} else if (tmp_f0 <= OPLUS_213HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_303_213Hz);
			return aw_haptic_1419_rtp_303_213Hz;
		} else {
			*data_len = sizeof(aw_haptic_1419_rtp_303_205Hz);
			return aw_haptic_1419_rtp_303_205Hz;
		}
		break;
	case SG_INPUT_DOWN_LOW:
		if (tmp_f0 <= OPLUS_197HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_304_197Hz);
			return aw_haptic_1419_rtp_304_197Hz;
		} else if (tmp_f0 <= OPLUS_201HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_304_201Hz);
			return aw_haptic_1419_rtp_304_201Hz;
		} else if (tmp_f0 <= OPLUS_205HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_304_205Hz);
			return aw_haptic_1419_rtp_304_205Hz;
		} else if (tmp_f0 <= OPLUS_209HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_304_209Hz);
			return aw_haptic_1419_rtp_304_209Hz;
		} else if (tmp_f0 <= OPLUS_213HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_304_213Hz);
			return aw_haptic_1419_rtp_304_213Hz;
		} else {
			*data_len = sizeof(aw_haptic_1419_rtp_304_205Hz);
			return aw_haptic_1419_rtp_304_205Hz;
		}
		break;
	case SG_INPUT_UP_LOW:
		if (tmp_f0 <= OPLUS_197HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_305_197Hz);
			return aw_haptic_1419_rtp_305_197Hz;
		} else if (tmp_f0 <= OPLUS_201HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_305_201Hz);
			return aw_haptic_1419_rtp_305_201Hz;
		} else if (tmp_f0 <= OPLUS_205HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_305_205Hz);
			return aw_haptic_1419_rtp_305_205Hz;
		} else if (tmp_f0 <= OPLUS_209HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_305_209Hz);
			return aw_haptic_1419_rtp_305_209Hz;
		} else if (tmp_f0 <= OPLUS_213HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_305_213Hz);
			return aw_haptic_1419_rtp_305_213Hz;
		} else {
			*data_len = sizeof(aw_haptic_1419_rtp_305_205Hz);
			return aw_haptic_1419_rtp_305_205Hz;
		}
		break;
	case INPUT_LOW:
		if (tmp_f0 <= OPLUS_197HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_110_197Hz);
			return aw_haptic_1419_rtp_110_197Hz;
		} else if (tmp_f0 <= OPLUS_201HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_110_201Hz);
			return aw_haptic_1419_rtp_110_201Hz;
		} else if (tmp_f0 <= OPLUS_205HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_110_205Hz);
			return aw_haptic_1419_rtp_110_205Hz;
		} else if (tmp_f0 <= OPLUS_209HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_110_209Hz);
			return aw_haptic_1419_rtp_110_209Hz;
		} else if (tmp_f0 <= OPLUS_213HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_110_213Hz);
			return aw_haptic_1419_rtp_110_213Hz;
		} else {
			*data_len = sizeof(aw_haptic_1419_rtp_110_205Hz);
			return aw_haptic_1419_rtp_110_205Hz;
		}
		break;
	case INPUT_MEDI:
		if (tmp_f0 <= OPLUS_197HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_111_197Hz);
			return aw_haptic_1419_rtp_111_197Hz;
		} else if (tmp_f0 <= OPLUS_201HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_111_201Hz);
			return aw_haptic_1419_rtp_111_201Hz;
		} else if (tmp_f0 <= OPLUS_205HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_111_205Hz);
			return aw_haptic_1419_rtp_111_205Hz;
		} else if (tmp_f0 <= OPLUS_209HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_111_209Hz);
			return aw_haptic_1419_rtp_111_209Hz;
		} else if (tmp_f0 <= OPLUS_213HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_111_213Hz);
			return aw_haptic_1419_rtp_111_213Hz;
		} else {
			*data_len = sizeof(aw_haptic_1419_rtp_111_205Hz);
			return aw_haptic_1419_rtp_111_205Hz;
		}
		break;
	case INPUT_HIGH:
		if (tmp_f0 <= OPLUS_197HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_112_197Hz);
			return aw_haptic_1419_rtp_112_197Hz;
		} else if (tmp_f0 <= OPLUS_201HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_112_201Hz);
			return aw_haptic_1419_rtp_112_201Hz;
		} else if (tmp_f0 <= OPLUS_205HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_112_205Hz);
			return aw_haptic_1419_rtp_112_205Hz;
		} else if (tmp_f0 <= OPLUS_209HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_112_209Hz);
			return aw_haptic_1419_rtp_112_209Hz;
		} else if (tmp_f0 <= OPLUS_213HZ_F0) {
			*data_len = sizeof(aw_haptic_1419_rtp_112_213Hz);
			return aw_haptic_1419_rtp_112_213Hz;
		} else {
			*data_len = sizeof(aw_haptic_1419_rtp_112_205Hz);
			return aw_haptic_1419_rtp_112_205Hz;
		}
		break;
	default:
		oh_err("%s: can not find rtp file\n", __func__);
		break;
	}

	return NULL;
}

uint8_t *custom_0816_rtp_key_file(uint32_t *data_len)
{
	uint32_t tmp_f0;

	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return NULL;
	}
	if (g_oh->haptic_common_ops == NULL || g_oh->chip_data == NULL) {
		oh_err("%s: haptic_common_ops or chip_data is NULL\n", __func__);
		return NULL;
	}
	if (!g_oh->haptic_common_ops->haptic_get_f0) {
		oh_err("%s: haptic_get_f0 is null\n", __func__);
		return NULL;
	}
	tmp_f0 = g_oh->haptic_common_ops->haptic_get_f0(g_oh->chip_data);
	switch(g_oh->haptic_common_ops->haptic_get_rtp_file_num(g_oh->chip_data)) {
	case SG_INPUT_DOWN_HIGH:
		if (tmp_f0 <= OPLUS_124HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_302_122Hz);
			return aw_haptic_0816_rtp_302_122Hz;
		} else if (tmp_f0 <= OPLUS_128HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_302_126Hz);
			return aw_haptic_0816_rtp_302_126Hz;
		} else if (tmp_f0 <= OPLUS_132HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_302_130Hz);
			return aw_haptic_0816_rtp_302_130Hz;
		} else if (tmp_f0 <= OPLUS_136HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_302_134Hz);
			return aw_haptic_0816_rtp_302_134Hz;
		} else if (tmp_f0 <= OPLUS_140HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_302_138Hz);
			return aw_haptic_0816_rtp_302_138Hz;
		} else {
			*data_len = sizeof(aw_haptic_0816_rtp_302_130Hz);
			return aw_haptic_0816_rtp_302_130Hz;
		}
		break;
	case SG_INPUT_UP_HIGH:
		if (tmp_f0 <= OPLUS_124HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_303_122Hz);
			return aw_haptic_0816_rtp_303_122Hz;
		} else if (tmp_f0 <= OPLUS_128HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_303_126Hz);
			return aw_haptic_0816_rtp_303_126Hz;
		} else if (tmp_f0 <= OPLUS_132HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_303_130Hz);
			return aw_haptic_0816_rtp_303_130Hz;
		} else if (tmp_f0 <= OPLUS_136HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_303_134Hz);
			return aw_haptic_0816_rtp_303_134Hz;
		} else if (tmp_f0 <= OPLUS_140HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_303_138Hz);
			return aw_haptic_0816_rtp_303_138Hz;
		} else {
			*data_len = sizeof(aw_haptic_0816_rtp_303_130Hz);
			return aw_haptic_0816_rtp_303_130Hz;
		}
		break;
	case SG_INPUT_DOWN_LOW:
		if (tmp_f0 <= OPLUS_124HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_304_122Hz);
			return aw_haptic_0816_rtp_304_122Hz;
		} else if (tmp_f0 <= OPLUS_128HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_304_126Hz);
			return aw_haptic_0816_rtp_304_126Hz;
		} else if (tmp_f0 <= OPLUS_132HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_304_130Hz);
			return aw_haptic_0816_rtp_304_130Hz;
		} else if (tmp_f0 <= OPLUS_136HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_304_134Hz);
			return aw_haptic_0816_rtp_304_134Hz;
		} else if (tmp_f0 <= OPLUS_140HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_304_138Hz);
			return aw_haptic_0816_rtp_304_138Hz;
		} else {
			*data_len = sizeof(aw_haptic_0816_rtp_304_130Hz);
			return aw_haptic_0816_rtp_304_130Hz;
		}
		break;
	case SG_INPUT_UP_LOW:
		if (tmp_f0 <= OPLUS_124HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_305_122Hz);
			return aw_haptic_0816_rtp_305_122Hz;
		} else if (tmp_f0 <= OPLUS_128HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_305_126Hz);
			return aw_haptic_0816_rtp_305_126Hz;
		} else if (tmp_f0 <= OPLUS_132HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_305_130Hz);
			return aw_haptic_0816_rtp_305_130Hz;
		} else if (tmp_f0 <= OPLUS_136HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_305_134Hz);
			return aw_haptic_0816_rtp_305_134Hz;
		} else if (tmp_f0 <= OPLUS_140HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_305_138Hz);
			return aw_haptic_0816_rtp_305_138Hz;
		} else {
			*data_len = sizeof(aw_haptic_0816_rtp_305_130Hz);
			return aw_haptic_0816_rtp_305_130Hz;
		}
		break;
	case INPUT_LOW:
		if (tmp_f0 <= OPLUS_124HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_110_122Hz);
			return aw_haptic_0816_rtp_110_122Hz;
		} else if (tmp_f0 <= OPLUS_128HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_110_126Hz);
			return aw_haptic_0816_rtp_110_126Hz;
		} else if (tmp_f0 <= OPLUS_132HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_110_130Hz);
			return aw_haptic_0816_rtp_110_130Hz;
		} else if (tmp_f0 <= OPLUS_136HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_110_134Hz);
			return aw_haptic_0816_rtp_110_134Hz;
		} else if (tmp_f0 <= OPLUS_140HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_110_138Hz);
			return aw_haptic_0816_rtp_110_138Hz;
		} else {
			*data_len = sizeof(aw_haptic_0816_rtp_110_130Hz);
			return aw_haptic_0816_rtp_110_130Hz;
		}
		break;
	case INPUT_MEDI:
		if (tmp_f0 <= OPLUS_124HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_111_122Hz);
			return aw_haptic_0816_rtp_111_122Hz;
		} else if (tmp_f0 <= OPLUS_128HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_111_126Hz);
			return aw_haptic_0816_rtp_111_126Hz;
		} else if (tmp_f0 <= OPLUS_132HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_111_130Hz);
			return aw_haptic_0816_rtp_111_130Hz;
		} else if (tmp_f0 <= OPLUS_136HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_111_134Hz);
			return aw_haptic_0816_rtp_111_134Hz;
		} else if (tmp_f0 <= OPLUS_140HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_111_138Hz);
			return aw_haptic_0816_rtp_111_138Hz;
		} else {
			*data_len = sizeof(aw_haptic_0816_rtp_111_130Hz);
			return aw_haptic_0816_rtp_111_130Hz;
		}
		break;
	case INPUT_HIGH:
		if (tmp_f0 <= OPLUS_124HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_112_122Hz);
			return aw_haptic_0816_rtp_112_122Hz;
		} else if (tmp_f0 <= OPLUS_128HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_112_126Hz);
			return aw_haptic_0816_rtp_112_126Hz;
		} else if (tmp_f0 <= OPLUS_132HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_112_130Hz);
			return aw_haptic_0816_rtp_112_130Hz;
		} else if (tmp_f0 <= OPLUS_136HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_112_134Hz);
			return aw_haptic_0816_rtp_112_134Hz;
		} else if (tmp_f0 <= OPLUS_140HZ_F0) {
			*data_len = sizeof(aw_haptic_0816_rtp_112_138Hz);
			return aw_haptic_0816_rtp_112_138Hz;
		} else {
			*data_len = sizeof(aw_haptic_0816_rtp_112_130Hz);
			return aw_haptic_0816_rtp_112_130Hz;
		}
		break;
	default:
		oh_err("%s: can not find rtp file\n", __func__);
		break;
	}

	return NULL;
}

int register_common_haptic_device(struct haptic_common_data *oh)
{
	struct device_node *np = oh->i2c->dev.of_node;
	int ret = -1;
		/* I2C Adapter capability detection */
	if (!i2c_check_functionality(oh->i2c->adapter, I2C_FUNC_I2C)) {
		oh_err("%s:i2c algorithm ability detect failed\n", __func__);
		return -EIO;
	}
	/*step1 : dts parse*/
	ret = init_parse_dts(oh->dev, oh, np);
	if (ret) {
		oh_err("%s:dts parse failed\n", __func__);
		return ret;
	}
	/*step2 : request gpio resource*/
	ret = haptic_acquire_gpio_res(&oh->i2c->dev,oh);
	if (ret) {
		oh_err("%s:acquire gpio failed\n", __func__);
		goto err_gpio_res;
	}
	/*step3 : chip interface init*/
	ret = oh->haptic_common_ops->chip_interface_init(oh);
	if (ret) {
		oh_err("%s:acquire gpio failed\n", __func__);
		goto err_gpio_res;
	}
	/*step4 : request irq resource*/
	ret = haptic_acquire_irq_res(&oh->i2c->dev, oh);
	if (ret) {
		oh_err("%s: irq gpio interrupt request failed\n", __func__);
		goto err_irq;
	}
	/*step5 : work queue init*/
	ret = haptic_common_init(oh);
	if(ret) {
		oh_err("%s: haptic init failed\n", __func__);
		goto err_irq;
	}
	g_oh = oh;
	/*step6 : add sysfs node*/
	ret = haptic_add_sys_node(oh);
	if(ret) {
		oh_err("%s: add sysfs node failed\n", __func__);
		goto err_irq;
	}
	/*step7 : register misc device*/
	ret = misc_register(&oplus_haptic_misc);
	if (ret) {
		oh_err("%s: misc fail: %d\n", __func__, ret);
		goto err_dev_sysfs;
	}

	/*step8 : add proc node*/
	ret = haptic_init_proc(oh);
	if(ret) {
		oh_err("%s: init_vibrator_proc failed\n", __func__);
		goto err_dev_sysfs;
	}

	INIT_WORK(&oh->motor_old_test_work, motor_old_test_work);
	oh->motor_old_test_mode = 0;
	return 0;
err_dev_sysfs:
	misc_deregister(&oplus_haptic_misc);
err_irq:
	devm_free_irq(&oh->i2c->dev, gpio_to_irq(oh->irq_gpio), oh);
err_gpio_res:
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	devm_gpio_free(&i2c->dev, oh->irq_gpio);
	devm_gpio_free(&i2c->dev, oh->reset_gpio);
#endif
    return ret;
}

void unregister_common_haptic_device(struct haptic_common_data *pdata)
{
	struct haptic_common_data *oh = pdata;
	if (!pdata) {
		return;
	}
	remove_proc_entry("vibrator", oh->prEntry_da);
	cancel_work_sync(&oh->motor_old_test_work);
	misc_deregister(&oplus_haptic_misc);
	devm_free_irq(oh->dev, oh->irq_gpio, oh);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	devm_gpio_free(oh->dev, oh->irq_gpio);
	devm_gpio_free(oh->dev, oh->reset_gpio);
#endif
	return;
}

int common_haptic_data_free(struct haptic_common_data *pdata)
{
	if (pdata) {
		kfree(pdata);
	}

	return 0;
}

bool check_soft_rtp_support(uint32_t id)
{
	int i;

	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return NULL;
	}

	if (g_oh->device_id != DEVICE_ID_0816)
		return false;

	for (i = 0; i < ARRAY_SIZE(soft_rtp_wave); i++) {
		if (id == soft_rtp_wave[i])
			return true;
	}

	return false;
}

const char* get_rtp_name(uint32_t id, uint32_t f0) {
	const char* wave_name = NULL;
	const char* f0_suffix = NULL;
	const char* soft_str[] = {"_soft", ""};
	const char* soft = NULL;
	char* rtp_name = NULL;
	size_t len = 0;
	int i = 0;

	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return NULL;
	}

	oh_err("%s: enter. wave_id = %d, f0 = %d.\n", __func__, id, f0);
	for (i = 0; i < sizeof(f0_suffix_map) / sizeof(f0_suffix_map[0]); i++) {
		if (f0 < f0_suffix_map[i].f0_thre) {
			f0_suffix = f0_suffix_map[i].suffix;
			break;
		}
	}
	if (!f0_suffix) {
		oh_err("%s: f0 is %d, not found suffix.\n", __func__, f0);
		return NULL;
	}
	if (id > 0 && id < NUM_WAVEFORMS)
		wave_name = rtp_wave_map[id];
	else
		oh_err("%s: id is %d, out of range.\n", __func__, id);
	if (!wave_name) {
		oh_err("%s: id is %d, not found wave name.\n", __func__, id);
		return NULL;
	}

	if (check_soft_rtp_support(id) && g_oh->vibration_style == HAPTIC_VIBRATION_SOFT_STYLE) {
		oh_err("%s: id is %d, vibration_style is soft\n", __func__, id);
		soft = soft_str[0];
	} else {
		soft = soft_str[1];
	}

	len = strlen(wave_name) + strlen(soft) + strlen(f0_suffix) + 1;
	rtp_name = (char*) vmalloc(len);
	if (!rtp_name) {
		oh_err("%s: vmalloc failed.\n", __func__);
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		(void)oplus_haptic_track_mem_alloc_err(HAPTIC_MEM_ALLOC_TRACK,
			len, __func__);
#endif
		return NULL;
	} else {
		snprintf(rtp_name, len, "%s%s%s", wave_name, soft, f0_suffix);
	}
	return rtp_name;
}

void haptic_set_ftm_wave(void)
{
	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return;
	}
#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT ||
		get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT)
#else
	if (get_boot_mode()== MSM_BOOT_MODE__FACTORY || get_boot_mode() == MSM_BOOT_MODE__RF ||
		get_boot_mode() == MSM_BOOT_MODE__WLAN)
#endif
	{
		g_oh->haptic_common_ops->haptic_set_wav_seq(g_oh->chip_data, 0,
						HAPTIC_WAVEFORM_INDEX_SINE_CYCLE);
	} else {
		g_oh->haptic_common_ops->haptic_set_wav_seq(g_oh->chip_data, 0,
						HAPTIC_WAVEFORM_INDEX_SINE_CYCLE);
	}
}

const struct firmware *old_work_file_load_accord_f0(uint32_t rtp_file_num)
{
	const struct firmware *rtp_file;
	uint32_t f0_file_num = 1024;
	int ret = -1;
	uint32_t tmp_f0;

	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return NULL;
	}
	if (g_oh->haptic_common_ops == NULL || g_oh->chip_data == NULL) {
		oh_err("%s: haptic_common_ops or chip_data is NULL\n", __func__);
		return NULL;
	}
	if (!g_oh->haptic_common_ops->haptic_get_f0) {
		oh_err("%s: haptic_get_f0 is null\n", __func__);
		return NULL;
	}
	tmp_f0 = g_oh->haptic_common_ops->haptic_get_f0(g_oh->chip_data);
	if (rtp_file_num == HAPTIC_WAVEFORM_INDEX_OLD_STEADY
		|| rtp_file_num == HAPTIC_WAVEFORM_INDEX_HIGH_TEMP) {
		if (DEVICE_ID_0815 == g_oh->device_id || DEVICE_ID_0809 == g_oh->device_id) {
			if(tmp_f0 <= OPLUS_161HZ_F0)
				f0_file_num = 0;
			else if(tmp_f0 <= OPLUS_163HZ_F0)
				f0_file_num = 1;
			else if(tmp_f0 <= OPLUS_165HZ_F0)
				f0_file_num = 2;
			else if(tmp_f0 <= OPLUS_167HZ_F0)
				f0_file_num = 3;
			else if(tmp_f0 <= OPLUS_169HZ_F0)
				f0_file_num = 4;
			else if(tmp_f0 <= OPLUS_171HZ_F0)
				f0_file_num = 5;
			else if(tmp_f0 <= OPLUS_173HZ_F0)
				f0_file_num = 6;
			else if(tmp_f0 <= OPLUS_175HZ_F0)
				f0_file_num = 7;
			else if(tmp_f0 <= OPLUS_177HZ_F0)
				f0_file_num = 8;
			else if(tmp_f0 <= OPLUS_179HZ_F0)
				f0_file_num = 9;
			else
				f0_file_num = 10;
		} else if (DEVICE_ID_1419 == g_oh->device_id) {
			if(tmp_f0 <= OPLUS_196HZ_F0)
				f0_file_num = 0;
			else if (tmp_f0 <= OPLUS_198HZ_F0)
				f0_file_num = 1;
			else if (tmp_f0 <= OPLUS_200HZ_F0)
				f0_file_num = 2;
			else if (tmp_f0 <= OPLUS_202HZ_F0)
				f0_file_num = 3;
			else if (tmp_f0 <= OPLUS_204HZ_F0)
				f0_file_num = 4;
			else if (tmp_f0 <= OPLUS_206HZ_F0)
				f0_file_num = 5;
			else if (tmp_f0 <= OPLUS_208HZ_F0)
				f0_file_num = 6;
			else if (tmp_f0 <= OPLUS_210HZ_F0)
				f0_file_num = 7;
			else if (tmp_f0 <= OPLUS_212HZ_F0)
				f0_file_num = 8;
			else if (tmp_f0 <= OPLUS_214HZ_F0)
				f0_file_num = 9;
			else
				f0_file_num = 10;
		} else if (DEVICE_ID_0816 == g_oh->device_id) {
				if(tmp_f0 <= OPLUS_121HZ_F0)
					f0_file_num = 0;
				else if (tmp_f0 <= OPLUS_123HZ_F0)
					f0_file_num = 1;
				else if (tmp_f0 <= OPLUS_125HZ_F0)
					f0_file_num = 2;
				else if (tmp_f0 <= OPLUS_127HZ_F0)
					f0_file_num = 3;
				else if (tmp_f0 <= OPLUS_129HZ_F0)
					f0_file_num = 4;
				else if (tmp_f0 <= OPLUS_131HZ_F0)
					f0_file_num = 5;
				else if (tmp_f0 <= OPLUS_133HZ_F0)
					f0_file_num = 6;
				else if (tmp_f0 <= OPLUS_135HZ_F0)
					f0_file_num = 7;
				else if (tmp_f0 <= OPLUS_137HZ_F0)
					f0_file_num = 8;
				else if (tmp_f0 <= OPLUS_139HZ_F0)
					f0_file_num = 9;
				else
					f0_file_num = 10;
			}
	}

	if (rtp_file_num == HAPTIC_WAVEFORM_INDEX_OLD_STEADY) {
		if (DEVICE_ID_0815 == g_oh->device_id || DEVICE_ID_0809 == g_oh->device_id) {
			ret = request_firmware(&rtp_file,
					haptic_old_steady_test_rtp_name_0815[f0_file_num],
					g_oh->dev);
			oh_err("%s line %d: rtp_num:%d f0:%d name:%s\n", __func__, __LINE__,
				rtp_file_num, f0_file_num, haptic_old_steady_test_rtp_name_0815[f0_file_num]);
		} else if (DEVICE_ID_1419 == g_oh->device_id) {
			ret = request_firmware(&rtp_file,
					haptic_old_steady_test_rtp_name_1419[f0_file_num],
					g_oh->dev);
			oh_err("%s line %d: rtp_num:%d f0:%d name:%s\n", __func__, __LINE__,
				rtp_file_num, f0_file_num, haptic_old_steady_test_rtp_name_1419[f0_file_num]);
		} else if (DEVICE_ID_0816 == g_oh->device_id) {
			ret = request_firmware(&rtp_file,
					haptic_old_steady_test_rtp_name_0816[f0_file_num],
					g_oh->dev);
			oh_err("%s line %d: rtp_num:%d f0:%d name:%s\n", __func__, __LINE__,
				rtp_file_num, f0_file_num, haptic_old_steady_test_rtp_name_0816[f0_file_num]);
		}
	} else {
		if (DEVICE_ID_0815 == g_oh->device_id || DEVICE_ID_0809 == g_oh->device_id) {
			ret = request_firmware(&rtp_file,
					haptic_high_temp_high_humidity_0815[f0_file_num],
					g_oh->dev);
			oh_err("%s line %d: rtp_num:%d f0:%d name:%s\n", __func__, __LINE__,
				rtp_file_num, f0_file_num, haptic_high_temp_high_humidity_0815[f0_file_num]);
		} else if (DEVICE_ID_1419 == g_oh->device_id) {
			ret = request_firmware(&rtp_file,
					haptic_high_temp_high_humidity_1419[f0_file_num],
					g_oh->dev);
			oh_err("%s line %d: rtp_num:%d f0:%d name:%s\n", __func__, __LINE__,
				rtp_file_num, f0_file_num, haptic_old_steady_test_rtp_name_1419[f0_file_num]);
		} else if (DEVICE_ID_0816 == g_oh->device_id) {
			ret = request_firmware(&rtp_file,
					haptic_high_temp_high_humidity_0816[f0_file_num],
					g_oh->dev);
			oh_err("%s line %d: rtp_num:%d f0:%d name:%s\n", __func__, __LINE__,
				rtp_file_num, f0_file_num, haptic_high_temp_high_humidity_0816[f0_file_num]);
		}
	}
	if (ret < 0) {
		oh_err("%s line %d: failed to read index[%d]\n", __func__, __LINE__, f0_file_num);
		return NULL;
	}

	return rtp_file;
}

const struct firmware *rtp_load_file_accord_f0(uint32_t rtp_file_num)
{
	if (rtp_file_num == HAPTIC_WAVEFORM_INDEX_OLD_STEADY
         || rtp_file_num == HAPTIC_WAVEFORM_INDEX_HIGH_TEMP) {
        return old_work_file_load_accord_f0(rtp_file_num);
	}

	return NULL;
}

uint32_t haptic_common_get_f0(void)
{
	uint32_t tmp_f0;

	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return 0;
	}

	tmp_f0 = g_oh->f0;
	if ((g_oh->device_id == DEVICE_ID_0815) || (g_oh->device_id == DEVICE_ID_0809)) {
		if (tmp_f0 < F0_VAL_MIN_0815 || tmp_f0 > F0_VAL_MAX_0815)
			g_oh->f0 = 1700;
	} else if (g_oh->device_id == DEVICE_ID_81538) {
		if (tmp_f0 < F0_VAL_MIN_081538 || tmp_f0 > F0_VAL_MAX_081538)
			g_oh->f0 = 1500;
	} else if (g_oh->device_id == DEVICE_ID_0832) {
		if (tmp_f0 < F0_VAL_MIN_0832 || tmp_f0 > F0_VAL_MAX_0832)
			g_oh->f0 = 2350;
	} else if (g_oh->device_id == DEVICE_ID_1419) {
		if (tmp_f0 < F0_VAL_MIN_1419 || tmp_f0 > F0_VAL_MAX_1419)
			g_oh->f0 = 2050;
	} else if (g_oh->device_id == DEVICE_ID_0816) {
		if (tmp_f0 < F0_VAL_MIN_0816 || tmp_f0 > F0_VAL_MAX_0816)
			g_oh->f0 = 1300;
	} else {
		if (tmp_f0 < F0_VAL_MIN_0833 || tmp_f0 > F0_VAL_MAX_0833)
			g_oh->f0 = 2350;
	}

	return g_oh->f0;
}

bool get_ringtone_support(uint32_t val)
{
	if(((val >=  RINGTONES_START_INDEX && val <= RINGTONES_END_INDEX)
		|| (val >=  NEW_RING_START && val <= NEW_RING_END)
		|| (val >=  OS12_NEW_RING_START && val <= OS12_NEW_RING_END)
		|| (val >=  OPLUS_RING_START && val < OPLUS_RING_END)
		|| (val >=  OS14_NEW_RING_START && val <= OS14_NEW_RING_END)
		|| (val >=  OS15_ALARM_RING_START && val <= OS15_ALARM_RING_END)
		|| (val >=  OS15_OPERATOR_RING_START && val <= OS15_OPERATOR_RING_END)
		|| (val >=  ALCLOUDSCAPE_START && val <= ALCLOUDSCAPE_END)
		|| (val >=  RINGTONE_NOTIF_ALARM_START && val <= RINGTONE_NOTIF_ALARM_END)
		|| val == RINGTONES_SIMPLE_INDEX
		|| val == RINGTONES_PURE_INDEX
		|| val == AUDIO_READY_STATUS))
		return true;
	else
		return false;
}

bool get_rtp_key_support(uint32_t val)
{
	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return false;
	}
	if((DEVICE_ID_0815 == g_oh->device_id || DEVICE_ID_0809 == g_oh->device_id
		|| DEVICE_ID_1419 == g_oh->device_id
		|| DEVICE_ID_0816 == g_oh->device_id) &&
		 ((val >= SG_INPUT_DOWN_HIGH && val <= SG_INPUT_UP_LOW) ||
		 (val >= INPUT_LOW && val <= INPUT_HIGH)))
		return true;
	return false;
}

uint8_t *get_rtp_key_data(uint32_t *haptic_rtp_key_data_len)
{
	uint8_t *haptic_rtp_key_data = NULL;
	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return NULL;
	}
	if (g_oh->device_id == DEVICE_ID_0809) {
		haptic_rtp_key_data = custom_0809_rtp_key_file(haptic_rtp_key_data_len);
	} else if (g_oh->device_id == DEVICE_ID_0815) {
		haptic_rtp_key_data = custom_0815_rtp_key_file(haptic_rtp_key_data_len);
	} else if (g_oh->device_id == DEVICE_ID_1419) {
		haptic_rtp_key_data = custom_1419_rtp_key_file(haptic_rtp_key_data_len);
	} else if (g_oh->device_id == DEVICE_ID_0816) {
		haptic_rtp_key_data = custom_0816_rtp_key_file(haptic_rtp_key_data_len);
	} else {
		oh_info("%s: vibrator id: %d,not found key data name\n", __func__, g_oh->device_id);
		return NULL;
	}
	return haptic_rtp_key_data;
}

#define DEFAULT_BATT_SOC 50
int read_batt_soc(int *val)
{
	static struct power_supply *batt_psy;
	union power_supply_propval ret = {0,};
	int rc = 0;

	*val = DEFAULT_BATT_SOC;
	if (!batt_psy)
		batt_psy = power_supply_get_by_name("battery");

	if (batt_psy) {
		rc = power_supply_get_property(batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		if (rc) {
			oh_err("battery soc read error:%d\n", rc);
			return rc;
		}
		*val = ret.intval;
	} else {
		oh_err("get battery psy failed\n");
	}

	return rc;
}

#define DEFAULT_BATT_TEMP 250
int read_batt_temp(int *val)
{
	static struct power_supply *batt_psy;
	union power_supply_propval ret = {0,};
	int rc = 0;

	*val = DEFAULT_BATT_TEMP;
	if (!batt_psy)
		batt_psy = power_supply_get_by_name("battery");

	if (batt_psy) {
		rc = power_supply_get_property(batt_psy,
				POWER_SUPPLY_PROP_TEMP, &ret);
		if (rc) {
			oh_err("battery temp read error:%d\n", rc);
			return rc;
		}
		*val = ret.intval;
	} else {
		oh_err("get battery temp failed\n");
	}

	return rc;
}

bool vbat_low_soc_flag(void)
{
	int rc = 0;
	int batt_soc;
	int batt_temp;
	bool vbat_is_low = false;
	if (!g_oh) {
		oh_err("%s: g_oh is null\n", __func__);
		return NULL;
	}
	rc = read_batt_soc(&batt_soc);
	read_batt_temp(&batt_temp);
	if ((rc == 0) && (((batt_temp >= g_oh->vbat_low_temp) && (batt_soc < g_oh->vbat_low_soc))
		|| ((batt_temp < g_oh->vbat_low_temp) && (batt_soc < g_oh->vbat_low_soc_cold)))) {
		vbat_is_low = true;
		oh_err("%s: vbat low! batt_soc %d, batt_temp %d, vbat_low_temp %d, vbat_low_soc %d\n",
			__func__, batt_soc, batt_temp, g_oh->vbat_low_temp, g_oh->vbat_low_soc);
		oh_err("%s:vbat_low_soc_cold %d, vbat_low_vmax_level %d\n",
			__func__, g_oh->vbat_low_soc_cold, g_oh->vbat_low_vmax_level);
	}
	return vbat_is_low;
}

static int __init haptic_i2c_init(void)
{
	int ret[HAPTIC_NUM] = {0};
	int i;

	oh_err("haptic_i2c_init enter\n");
	ret[0] = awinic_i2c_init();
	if (ret[0]) {
		oh_err("%s: Failed to add aw I2C driver: %d\n", __func__, ret[0]);
		goto err_aw;
	}

	ret[1] = sih_i2c_init();
	if (ret[1]) {
		oh_err("%s: Failed to add sih I2C driver: %d\n", __func__, ret[1]);
		goto err_sih;
	}

	for (i = 0; i < HAPTIC_NUM; i++) {
		if (ret[i])
			return ret[i];
	}
	return 0;
err_sih:
	sih_i2c_exit();
err_aw:
	awinic_i2c_exit();
	return ret[0] ? ret[0] : ret[1];
}

static void __exit haptic_i2c_exit(void)
{
	awinic_i2c_exit();
	sih_i2c_exit();
}

module_init(haptic_i2c_init);
module_exit(haptic_i2c_exit);

MODULE_DESCRIPTION("Oplus Haptic Common Driver");
MODULE_LICENSE("GPL v2");
