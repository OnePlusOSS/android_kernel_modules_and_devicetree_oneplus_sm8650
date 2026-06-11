/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include "hf_sensor_type.h"
#include "sensor_interface.h"

#define FIFO_SIZE 32
extern int register_lcdinfo_notifier(struct notifier_block *nb);
extern int unregister_lcdinfo_notifier(struct notifier_block *nb);

static struct ssc_interactive *g_ssc_cxt = NULL;

static void ssc_interactive_set_fifo(uint8_t type, uint16_t data)
{
	struct fifo_frame fifo_fm;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	int ret = 0;
	/* DEVINFO_LOG("type= %u, data=%d\n", type, data); */
	memset(&fifo_fm, 0, sizeof(struct fifo_frame));
	fifo_fm.type = type;
	fifo_fm.data = data;
	ret = kfifo_in_spinlocked(&ssc_cxt->fifo, &fifo_fm, 1, &ssc_cxt->fifo_lock);
	if(ret != 1) {
		DEVINFO_LOG("kfifo is full\n");
	}
	wake_up_interruptible(&ssc_cxt->wq);
}

static void ssc_interactive_lcdinfo_to_hal(uint8_t type, uint16_t val)
{
	ssc_interactive_set_fifo(type, val);
}

static void ssc_interactive_lcdinfo_to_scp(void)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	if (ssc_cxt && ssc_cxt->a_info.senstype != 0) {
		schedule_delayed_work(&ssc_cxt->lcdinfo_work, 0);
	}
}

static void transfer_lcdinfo_to_scp(struct work_struct *work)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	int ret = 0;

	if (ssc_cxt->si && ssc_cxt->si->send_lcdinfo) {
		ret = ssc_cxt->si->send_lcdinfo(&ssc_cxt->a_info);
	}
	if (ret < 0) {
		DEVINFO_LOG("send lcd info error\n");
	}
}

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO)
static void ssc_interactive_sf_info_to_scp(void)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	if (ssc_cxt && ssc_cxt->sf_info.senstype != 0) {
		schedule_delayed_work(&ssc_cxt->sf_info_work, 0);
	}
}

static void transfer_sf_info_to_scp(struct work_struct *work)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	int ret = 0;

	if (ssc_cxt->si && ssc_cxt->si->send_sf_info) {
		ret = ssc_cxt->si->send_sf_info(&ssc_cxt->sf_info);
	}
	if (ret < 0) {
		DEVINFO_LOG("send sf info error\n");
	}
}
#endif /* CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO */

static ssize_t ssc_interactive_write(struct file *file, const char __user * buf,
                size_t count, loff_t * ppos)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if (count > *ppos) {
		count -= *ppos;
	} else
		count = 0;

	*ppos += count;
	wake_up_interruptible(&ssc_cxt->wq);
	return count;
}

static unsigned int ssc_interactive_poll(struct file *file, struct poll_table_struct *pt)
{
	unsigned int ptr = 0;
	int count = 0;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	poll_wait(file, &ssc_cxt->wq, pt);
	spin_lock(&ssc_cxt->fifo_lock);
	count = kfifo_len(&ssc_cxt->fifo);
	spin_unlock(&ssc_cxt->fifo_lock);
	if (count > 0) {
		ptr |= POLLIN | POLLRDNORM;
	}
	return ptr;
}

static ssize_t ssc_interactive_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	size_t read = 0;
	int fifo_count = 0;
	int ret;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if (count !=0 && count < sizeof(struct fifo_frame)) {
		DEVINFO_LOG("err count %lu\n", count);
		return -EINVAL;
	}
	while ((read + sizeof(struct fifo_frame)) <= count) {
		struct fifo_frame fifo_fm;
		spin_lock(&ssc_cxt->fifo_lock);
		fifo_count = kfifo_len(&ssc_cxt->fifo);
		spin_unlock(&ssc_cxt->fifo_lock);

		if (fifo_count <= 0) {
			break;
		}
		ret = kfifo_out(&ssc_cxt->fifo, &fifo_fm, 1);
		if (copy_to_user(buf+read, &fifo_fm, sizeof(struct fifo_frame))) {
			DEVINFO_LOG("copy_to_user failed \n");
			return -EFAULT;
		}
		read += sizeof(struct fifo_frame);
	}
	*ppos += read;
	return read;
}

static uint16_t ssc_interactive_converted_brightnes(uint16_t brightness)
{
	uint16_t bri = 0;
	uint16_t cnt = 0;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if (ssc_cxt->brl_info.pri_brl_num == 0) {
		bri = brightness;
	}

	/* set brightness according to each segment */
	for (cnt = 0; cnt < ssc_cxt->brl_info.pri_brl_num; cnt++) {
		if ((0 == cnt) && (brightness <= ssc_cxt->brl_info.pri_brl_thrd[cnt])) {
			/* brl: 0 ~ pri_brl_thrd[cnt] */
			bri = ssc_cxt->brl_info.pri_brl_thrd[cnt] - 1;
		} else if (cnt == (ssc_cxt->brl_info.pri_brl_num - 1) && (brightness > ssc_cxt->brl_info.pri_brl_thrd[cnt])) {
			/* brl: pri_brl_thrd[max] ~ max */
			bri = ssc_cxt->brl_info.pri_brl_thrd[cnt] + 1;
		} else if ((brightness <= ssc_cxt->brl_info.pri_brl_thrd[cnt]) &&
			(brightness > ssc_cxt->brl_info.pri_brl_thrd[cnt - 1])) {
			/* brl: pri_brl_thrd[cnt-1] ~ pri_brl_thrd[cnt] */
			bri = ssc_cxt->brl_info.pri_brl_thrd[cnt] - 1;
		}
	}
	return bri;
}

static int ssc_interactive_release (struct inode *inode, struct file *file)
{
	DEVINFO_LOG("%s\n", __func__);
	return 0;
}

static const struct file_operations under_mdevice_fops = {
	.owner   = THIS_MODULE,
	.read    = ssc_interactive_read,
	.write   = ssc_interactive_write,
	.poll    = ssc_interactive_poll,
	.llseek  = generic_file_llseek,
	.release = ssc_interactive_release,
};

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO)
static ssize_t sf_device_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	DEVINFO_LOG("%s\n", __func__);
	return 0;
}

static ssize_t sf_device_write(struct file *file, const char __user * buf,
                size_t count, loff_t * ppos)
{
	struct screen_sf_info sf_info = {0};
	char tmp[256] = {0};
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;


	if (count >= sizeof(tmp)) {
		DEVINFO_LOG("sf_device_write: Input error count: %lu\n", count);
		return -EINVAL;
	}
	if (copy_from_user(tmp, buf, count)) {
		DEVINFO_LOG("sf_device_write: Failed to copy data from user\n");
		return -EFAULT;
	}

	DEVINFO_LOG("sf_device_write: %s\n", tmp);

	sscanf(tmp, "%lld,%lld,%d", &sf_info.start_ts, &sf_info.end_ts, &sf_info.index);

	ssc_cxt->sf_info.start_ts = sf_info.start_ts;
	ssc_cxt->sf_info.end_ts = sf_info.end_ts;
	ssc_cxt->sf_info.index = sf_info.index;
	ssc_interactive_sf_info_to_scp();

	return count;
}

static struct file_operations sf_device_fops = {
	.owner   = THIS_MODULE,
	.read    = sf_device_read,
	.write   = sf_device_write,
	.llseek  = generic_file_llseek,
};
#endif /* CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO */

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_BLANK_MODE)
static int mtk_lcdinfo_callback(struct notifier_block *nb, unsigned long event,
        void *data)
{
	int val = 0;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if (!data || !ssc_cxt) {
		DEVINFO_LOG("data is NULL\n");
		return 0;
	}

	switch(event) {
	case MTK_DISP_EVENT_BLANK:
		val = *(int*)data;
		if (val != ssc_cxt->a_info.blank_mode) {
			if (ssc_cxt->a_info.blank_mode != OPLUS_DOZE_AND_DOZE_SUSPEND_SWITCH) {
				ssc_cxt->a_info.blank_mode = val;
			}
			if (ssc_cxt->support_bri_to_scp) {
				ssc_interactive_lcdinfo_to_scp();
			}
			if (ssc_cxt->support_bri_to_hal) {
				if (ssc_cxt->a_info.blank_mode != OPLUS_DOZE_AND_DOZE_SUSPEND_SWITCH) {
					ssc_interactive_lcdinfo_to_hal(LCM_BLANK_MODE_TYPE, val);
				}
				ssc_interactive_lcdinfo_to_hal(LCM_BRIGHTNESS_TYPE, ssc_cxt->a_info.rt_bri);
			}
		}
		break;
	default:
		DEVINFO_LOG("event:%d\n", (int)event);
		break;
	}

	return 0;
}
#endif

static int lcdinfo_callback(struct notifier_block *nb, unsigned long event,
        void *data)
{
	int val = 0;
	uint16_t brightness = 0;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if (!data || !ssc_cxt) {
		DEVINFO_LOG("data is NULL\n");
		return 0;
	}

	switch(event) {
	case LCM_DC_MODE_TYPE:
		val = *(bool*)data;
		if (ssc_cxt->support_bri_to_scp) {
			if (val != ssc_cxt->a_info.dc_mode) {
				ssc_cxt->a_info.dc_mode = val;
				ssc_interactive_lcdinfo_to_scp();
			}
		}
		if (ssc_cxt->support_bri_to_hal) {
			if (val != ssc_cxt->a_info.dc_mode) {
				ssc_cxt->a_info.dc_mode = val;
				ssc_interactive_lcdinfo_to_hal(event, val);
			}
		}
		break;
	case LCM_BRIGHTNESS_TYPE:
		val = *(int*)data;
		if (ssc_cxt->support_bri_to_scp) {
			brightness = ssc_interactive_converted_brightnes(val);
			if (brightness != ssc_cxt->a_info.brightness) {
				ssc_cxt->a_info.brightness = brightness;
				ssc_interactive_lcdinfo_to_scp();
			}
		}
		if (ssc_cxt->support_bri_to_hal) {
			if (val != ssc_cxt->a_info.rt_bri) {
				ssc_cxt->a_info.rt_bri = val;
				ssc_interactive_lcdinfo_to_hal(event, val);
			}
		}
		break;
	case LCM_PWM_TURBO:
		val = *(bool*)data;
		if (ssc_cxt->support_pwm_turbo) {
			if (val != ssc_cxt->a_info.pwm_turbo) {
				ssc_cxt->a_info.pwm_turbo = val;
				ssc_interactive_lcdinfo_to_scp();
			}
		}
		break;
	case LCM_ADFR_MIN_FPS:
		val = *(int*)data;
		if (ssc_cxt->need_to_sync_lcd_rate) {
			ssc_cxt->a_info.fps = val;
			ssc_interactive_lcdinfo_to_scp();
		}
		break;
	case OPLUS_POWER_STATUS:
		val = *(int*)data;
		if (val == OPLUS_DOZE || val == OPLUS_DOZE_SUSPEND) {
			ssc_cxt->a_info.blank_mode = OPLUS_DOZE_AND_DOZE_SUSPEND_SWITCH;
			if (ssc_cxt->support_bri_to_hal) {
				ssc_interactive_lcdinfo_to_hal(LCM_BLANK_MODE_TYPE, OPLUS_DOZE_AND_DOZE_SUSPEND_SWITCH);
			}
		}
		break;
	default:
		DEVINFO_LOG("event:%d\n", (int)event);
		break;
	}

	return 0;
}

static void parse_br_level_info_dts(struct device_node *ch_node)
{
	int rc = 0;
	uint32_t value = 0;
	uint32_t brl_num = 0;
	uint32_t cnt = 0;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if (0 == strncmp(ch_node->name, "primary_lb_brl_info", 19)) {
		rc = of_property_read_u32(ch_node, "brl_thrd_num", &value);
		if (!rc) {
			brl_num = value;
			ssc_cxt->brl_info.pri_brl_num = brl_num;
		} else {
			ssc_cxt->brl_info.pri_brl_num = 0;
		}

		if (0 == ssc_cxt->brl_info.pri_brl_num) {
			DEVINFO_LOG("ERROR! can not parse the pri_brl_thrd_num(%u)\n", ssc_cxt->brl_info.pri_brl_num);
			return;
		}

		if (brl_num > 0 && brl_num <= BRL_MAX_LEN) {
			rc = of_property_read_u32_array(ch_node, "brl_thrd", &ssc_cxt->brl_info.pri_brl_thrd[0], brl_num);
			if (!rc) {
				for (cnt = 0; cnt < brl_num; cnt++) {
					DEVINFO_LOG("primary pri_brl_%u: %u\n", cnt, ssc_cxt->brl_info.pri_brl_thrd[cnt]);
				}
			}
		}
	}
}

static int ssc_interactive_parse_dts(void)
{
	int ret = 0;
	int report_brightness = 0;
	int support_bri_to_scp = 0;
	int support_pwm_turbo = 0;
	int need_to_sync_lcd_rate = 0;
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_BLANK_MODE)
	int report_blank_mode = 0;
#endif
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO)
        int receive_screenshot_info = 0;
#endif
	struct device_node *node = NULL;
	struct device_node *ch_node = NULL;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	node = of_find_node_by_name(NULL, "ssc_interactive");
	if (!node) {
		DEVINFO_LOG("find ssc_interactive fail\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(node, "report_brightness", &report_brightness);
	if (ret != 0) {
		DEVINFO_LOG("read report_brightness fail\n");
	}

	if (report_brightness == 1) {
		ssc_cxt->support_bri_to_hal = true;
		DEVINFO_LOG("support brightness report\n");
	}
	ssc_cxt->support_bri_to_hal = true;
	ret = of_property_read_u32(node, "support_bri_to_scp", &support_bri_to_scp);
	if (ret != 0) {
		DEVINFO_LOG("read support_bri_to_scp fail\n");
	}

	if (support_bri_to_scp == 1) {
		ssc_cxt->support_bri_to_scp = true;
		DEVINFO_LOG("support bri_to_scp\n");
	}

	ret = of_property_read_u32(node, "support_pwm_turbo", &support_pwm_turbo);
	if (ret != 0) {
		DEVINFO_LOG("read support_pwm_turbo fail\n");
	}

	if (support_pwm_turbo == 1) {
		ssc_cxt->support_pwm_turbo = true;
		DEVINFO_LOG("support pwm_turbo report\n");
	}


	ret = of_property_read_u32(node, "need_to_sync_lcd_rate", &need_to_sync_lcd_rate);
	if (ret != 0) {
		DEVINFO_LOG("read need_to_sync_lcd_rate fail\n");
	}

	if (need_to_sync_lcd_rate == 1) {
		ssc_cxt->need_to_sync_lcd_rate = true;
		DEVINFO_LOG("need_to_sync_lcd_rate\n");
	}

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_BLANK_MODE)
	ret = of_property_read_u32(node, "report_blank_mode", &report_blank_mode);
	if (ret != 0) {
		DEVINFO_LOG("read report_blank_mode fail\n");
	}

	if (report_blank_mode == 1) {
		ssc_cxt->report_blank_mode = true;
		DEVINFO_LOG("report_blank_mode\n");
	}
#endif

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO)
	ret = of_property_read_u32(node, "receive_screenshot_info", &receive_screenshot_info);
	if (ret != 0) {
		DEVINFO_LOG("read receive_screenshot_info fail\n");
	}

	if (receive_screenshot_info == 1) {
		ssc_cxt->receive_screenshot_info = true;
		DEVINFO_LOG("report screen shot mode\n");
	}
#endif

	for_each_child_of_node(node, ch_node) {
		parse_br_level_info_dts(ch_node);
	}
	return 0;
}

static int scp_ready(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if (ssc_cxt && event == SCP_EVENT_READY) {
		DEVINFO_LOG("receiver SCP_EVENT_READY\n");
		schedule_delayed_work(&ssc_cxt->ready_work, msecs_to_jiffies(5000));
	}

	return NOTIFY_DONE;
}

static void scp_ready_work(struct work_struct *dwork)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	/* to make sure scp is up */
	if (ssc_cxt && ssc_cxt->si && ssc_cxt->si->init_sensorlist
		&& ssc_cxt->si->get_lcdinfo_brocast_type) {
		ssc_cxt->si->init_sensorlist();
		ssc_cxt->a_info.senstype = ssc_cxt->si->get_lcdinfo_brocast_type();
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO)
		ssc_cxt->sf_info.senstype = ssc_cxt->si->get_lcdinfo_brocast_type();
#endif /* CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO */
		DEVINFO_LOG("scp_ready_work success %d\n", ssc_cxt->a_info.senstype);
	} else {
		DEVINFO_LOG("do not get sensor type\n");
	}
}

int ssc_interactive_init(void)
{
	int err = 0;
	struct ssc_interactive *ssc_cxt = kzalloc(sizeof(*ssc_cxt), GFP_KERNEL);

	if (ssc_cxt == NULL) {
		DEVINFO_LOG("kzalloc ssc_cxt failed\n");
		err = -ENOMEM;
		goto alloc_ssc_cxt_failed;
	}

	g_ssc_cxt = ssc_cxt;

	init_sensorhub_interface(&ssc_cxt->si);
	ssc_cxt->a_info.brightness = 10000;
	ssc_cxt->a_info.rt_bri = 10000;
	ssc_cxt->a_info.dc_mode = 0;
	ssc_cxt->a_info.pwm_turbo = 0;
	ssc_cxt->a_info.fps = OPLUS_ADFR_AUTO_MIN_FPS_1HZ;

	err = ssc_interactive_parse_dts();
	if (err != 0) {
		DEVINFO_LOG("parse dts fail\n");
		goto parse_dts_failed;
	}

	err = kfifo_alloc(&ssc_cxt->fifo, FIFO_SIZE, GFP_KERNEL);
	if (err) {
		DEVINFO_LOG("kzalloc kfifo failed\n");
		err = -ENOMEM;
		goto alloc_fifo_failed;
	}
	spin_lock_init(&ssc_cxt->fifo_lock);

	init_waitqueue_head(&ssc_cxt->wq);

	memset(&ssc_cxt->mdev, 0 , sizeof(struct miscdevice));
	ssc_cxt->mdev.minor = MISC_DYNAMIC_MINOR;
	ssc_cxt->mdev.name = "ssc_interactive";
	ssc_cxt->mdev.fops = &under_mdevice_fops;
	if (misc_register(&ssc_cxt->mdev) != 0) {
		DEVINFO_LOG("misc_register  mdev failed\n");
		err = -ENODEV;
		goto register_mdevice_failed;
	}

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO)
	memset(&ssc_cxt->sf_dev, 0 , sizeof(struct miscdevice));
	ssc_cxt->sf_dev.minor = MISC_DYNAMIC_MINOR;
	ssc_cxt->sf_dev.name = "ssc_screenshot_info";
	ssc_cxt->sf_dev.fops = &sf_device_fops;
	if (ssc_cxt->receive_screenshot_info) {
		if (misc_register(&ssc_cxt->sf_dev) != 0) {
			DEVINFO_LOG("misc_register  sf_dev failed\n");
			err = -ENODEV;
			goto register_sf_device_failed;
		}
	}
#endif /* CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO */

	INIT_DELAYED_WORK(&ssc_cxt->ready_work, scp_ready_work);
	ssc_cxt->ready_nb.notifier_call = scp_ready;
	scp_A_register_notify(&ssc_cxt->ready_nb);

	INIT_DELAYED_WORK(&ssc_cxt->lcdinfo_work, transfer_lcdinfo_to_scp);
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO)
	INIT_DELAYED_WORK(&ssc_cxt->sf_info_work, transfer_sf_info_to_scp);
#endif /* CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO */
	ssc_cxt->lcd_nb.notifier_call = lcdinfo_callback;
	register_lcdinfo_notifier(&ssc_cxt->lcd_nb);
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_BLANK_MODE)
	if (ssc_cxt->report_blank_mode) {
		ssc_cxt->mtk_lcd_nb.notifier_call = mtk_lcdinfo_callback;
		if (mtk_disp_notifier_register("oplus_ssc_interact", &ssc_cxt->mtk_lcd_nb)) {
			DEVINFO_LOG("mtk_disp_notifier_register failed\n");
			goto register_mtk_disp_notifier_failed;
		}
	}
#endif
	DEVINFO_LOG("ssc_interactive_init success!\n");
	return 0;

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_BLANK_MODE)
register_mtk_disp_notifier_failed:
	scp_A_unregister_notify(&ssc_cxt->ready_nb);
	unregister_lcdinfo_notifier(&ssc_cxt->lcd_nb);
	mtk_disp_notifier_unregister(&ssc_cxt->mtk_lcd_nb);
#endif
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO)
register_sf_device_failed:
	if (ssc_cxt->receive_screenshot_info) {
		misc_deregister(&ssc_cxt->sf_dev);
	}
#endif /* CONFIG_OPLUS_SENSOR_USE_SCREENSHOT_INFO */
register_mdevice_failed:
	kfifo_free(&ssc_cxt->fifo);
parse_dts_failed:
alloc_fifo_failed:
	kfree(ssc_cxt);
	ssc_cxt = NULL;
	g_ssc_cxt = NULL;
alloc_ssc_cxt_failed:
	return err;
}

void ssc_interactive_exit(void)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	if(ssc_cxt) {
		unregister_lcdinfo_notifier(&ssc_cxt->lcd_nb);
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_USE_BLANK_MODE)
		mtk_disp_notifier_unregister(&ssc_cxt->mtk_lcd_nb);
#endif
		scp_A_unregister_notify(&ssc_cxt->ready_nb);
		misc_deregister(&ssc_cxt->mdev);
		kfifo_free(&ssc_cxt->fifo);
		kfree(ssc_cxt);
		ssc_cxt = NULL;
		g_ssc_cxt = NULL;
	}
}
