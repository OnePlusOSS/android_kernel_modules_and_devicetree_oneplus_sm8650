// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 *
 */

#define pr_fmt(fmt) "[OPLUS_SEC]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms_gauge.h>
#include <oplus_sec.h>

#define SEC_VERIFY_AUTH_DATA_SIZE		32
#define SEC_GET_AUTH_DATA_TIMEOUT_MS		3000000 /* wait for the daemon to start */
#define SEC_MISC_DEV_NAME			"sec_dev"
#define SEC_CMD_BUF_SIZE			128
#define OPLUS_CHG_SEC_IC_PRIKEY_INDEX_DEFAULT	0

struct sec_dev_cmd {
	unsigned int cmd;
	unsigned int data_size;
	unsigned char data_buf[SEC_CMD_BUF_SIZE];
};

enum sec_dev_cmd_type {
	SEC_DEV_CMD_EXIT,
	SEC_DEV_CMD_GET_AUTH_DATA,
};

enum {
	SEC_IC_TEST_CMD_GET_ROMID	= 0,
	SEC_IC_TEST_CMD_WRITE_PAGE,
	SEC_IC_TEST_CMD_READ_PAGE,
	SEC_IC_TEST_CMD_ECDSA,
	SEC_IC_TEST_CMD_ECW,
	SEC_IC_TEST_CMD_SHUTDOWN,
	SEC_IC_TEST_CMD_START_RW_CHECK,
	SEC_IC_TEST_CMD_BATTERY_CHECK,
	SEC_IC_TEST_CMD_GET_RW_CHECK_RESULT,
	SEC_IC_TEST_CMD_MAX,
};

struct oplus_sec {
	struct device *dev;
	struct delayed_work sec_init_work;
	struct delayed_work exit_daemon_work;
	struct work_struct get_verify_data_work;
	struct work_struct set_sec_ic_prikey_work;
	struct work_struct start_rw_check_work;
	struct mutex sec_mutex;
	struct oplus_mms *gauge_topic;

	wait_queue_head_t read_wq;
	struct miscdevice misc_dev;
	struct mutex read_lock;
	struct mutex cmd_data_lock;
	struct completion cmd_ack;
	struct sec_dev_cmd cmd;
	bool cmd_data_ok;
	char auth_data[SEC_VERIFY_AUTH_DATA_SIZE];
	bool auth_data_ok;
	bool wait_auth_data_done;
	atomic_t rw_check_res;

	int sec_ic_index;
};

static struct oplus_sec *g_chip = NULL;

static int oplus_sec_exit_daemon_process(struct oplus_sec *chip)
{
	chg_info("oplus_sec_exit_daemon_process");
	mutex_lock(&chip->cmd_data_lock);
	memset(&chip->cmd, 0, sizeof(struct sec_dev_cmd));
	chip->cmd.cmd = SEC_DEV_CMD_EXIT;
	chip->cmd.data_size = 0;
	chip->cmd_data_ok = true;
	mutex_unlock(&chip->cmd_data_lock);
	reinit_completion(&chip->cmd_ack);
	wake_up(&chip->read_wq);

	return 0;
}

static int oplus_sec_get_verify_data(struct oplus_sec *chip, unsigned int index)
{
	mutex_lock(&chip->cmd_data_lock);
	memset(&chip->cmd, 0, sizeof(struct sec_dev_cmd));
	memset(chip->auth_data, 0, SEC_VERIFY_AUTH_DATA_SIZE);
	chip->cmd.cmd = SEC_DEV_CMD_GET_AUTH_DATA;
	chip->cmd.data_size = sizeof(index);
	*(int *)chip->cmd.data_buf = cpu_to_le32(index);
	chip->cmd_data_ok = true;
	mutex_unlock(&chip->cmd_data_lock);
	reinit_completion(&chip->cmd_ack);
	wake_up(&chip->read_wq);

	schedule_delayed_work(&chip->exit_daemon_work,
		msecs_to_jiffies(SEC_GET_AUTH_DATA_TIMEOUT_MS));

	return 0;
}

#define SEC_IC_TEST_BUF_SIZE 64
#define SEC_IC_TEST_PAGE_NUM 16
#define SEC_IC_TEST_PAGE_ID 3
static int oplus_sec_test_rw_check(struct oplus_mms *gauge_topic)
{
	int i;
	int rc;
	int error_code = 0;
	uint8_t out_buf[SEC_IC_TEST_BUF_SIZE] = {0};
	uint8_t in_buf[SEC_IC_TEST_BUF_SIZE] = {0};
	int out_len;

	for (i = 0; i < SEC_IC_TEST_PAGE_NUM; i++) {
		get_random_bytes(in_buf, sizeof(in_buf));
		rc = oplus_gauge_sec_write_page(gauge_topic, i, in_buf, sizeof(in_buf));
		if (rc == -ENODATA)
			continue;
		if (rc < 0) {
			chg_err("sec write page failed, rc=%d\n", rc);
			break;
		}
		rc = oplus_gauge_sec_read_page(gauge_topic, i, out_buf, &out_len);
		if (rc < 0) {
			chg_err("sec read page failed, rc=%d\n", rc);
			break;
		}
		if (memcmp(in_buf, out_buf, out_len)) {
			chg_err("read/write page check failed\n");
			rc = -EINVAL;
			break;
		}
	}
	error_code = rc;

	/* clear memery after test */
	memset(in_buf, 0, sizeof(in_buf));
	for (;i > 0; i--) {
		rc = oplus_gauge_sec_write_page(gauge_topic, i, in_buf, sizeof(in_buf));
		if (rc < 0)
			chg_err("sec fail to clear page[%d] after test, rc=%d\n", i, rc);
	}
	if (rc < 0 && rc != -ENODATA)
		error_code = rc;

	return error_code;
}

static int oplus_sec_test_start_rw_check(struct oplus_sec *chip)
{
	int status;

	if (chip == NULL)
		return -ENODEV;

	status = atomic_read(&chip->rw_check_res);
	chg_info("status=%d\n", status);
	if (status == -EBUSY)
		return -EBUSY;

	atomic_set(&chip->rw_check_res, -EBUSY);
	schedule_work(&chip->start_rw_check_work);
	return 0;
}

static int oplus_sec_test_check_batt_sn(struct oplus_mms *gauge_topic)
{
	return 0;
}

static int oplus_sec_test_get_rw_check_result(struct oplus_sec *chip)
{
	int status;

	if (chip == NULL)
		return -ENODEV;

	status = atomic_read(&chip->rw_check_res);
	chg_info("status=%d\n", status);

	return status;
}

int oplus_sec_test_helper(struct oplus_mms *topic, int cmd)
{
	int rc;
	uint8_t out_buf[SEC_IC_TEST_BUF_SIZE] = {0};
	uint8_t in_buf[SEC_IC_TEST_BUF_SIZE] = {0};
	int out_len;
	bool valid;
	struct oplus_sec *chip = g_chip;

	switch(cmd) {
	case SEC_IC_TEST_CMD_GET_ROMID:
		rc = oplus_gauge_sec_get_romid(topic, out_buf, &out_len);
		break;
	case SEC_IC_TEST_CMD_WRITE_PAGE:
		memset(in_buf, 0, sizeof(in_buf));
		rc = oplus_gauge_sec_write_page(topic, SEC_IC_TEST_PAGE_ID, in_buf, sizeof(in_buf));
		break;
	case SEC_IC_TEST_CMD_READ_PAGE:
		rc = oplus_gauge_sec_read_page(topic, SEC_IC_TEST_PAGE_ID, out_buf, &out_len);
		break;
	case SEC_IC_TEST_CMD_ECDSA:
		rc = oplus_gauge_sec_ecdsa(topic, &valid);
		break;
	case SEC_IC_TEST_CMD_ECW:
		rc = oplus_gauge_sec_ecw(topic, &valid);
		break;
	case SEC_IC_TEST_CMD_SHUTDOWN:
		rc = oplus_gauge_sec_shutdown(topic, &valid);
		break;
	case SEC_IC_TEST_CMD_START_RW_CHECK:
		rc = oplus_sec_test_start_rw_check(chip);
		break;
	case SEC_IC_TEST_CMD_BATTERY_CHECK:
		rc = oplus_sec_test_check_batt_sn(topic);
		break;
	case SEC_IC_TEST_CMD_GET_RW_CHECK_RESULT:
		rc = oplus_sec_test_get_rw_check_result(chip);
		break;
	default:
		chg_err("cmd:%d not support\n", cmd);
		return -ENOTSUPP;
	}
	if (rc < 0)
		chg_err("cmd:%d failed, rc = %d", cmd, rc);
	return rc;
}

static void oplus_sec_get_verify_data_work(struct work_struct *work)
{
	struct oplus_sec *chip = container_of(work, struct oplus_sec,
		get_verify_data_work);
	int rc = 0;

	if (chip->auth_data_ok) {
		chg_info("auth is ok");
		return;
	}

	rc = oplus_gauge_sec_get_prikey_index(chip->gauge_topic, &chip->sec_ic_index);
	if (rc < 0 & rc != -ENOTSUPP) {
		chg_err("get prikey_index fail, rc=%d\n", rc);
		chip->sec_ic_index = OPLUS_CHG_SEC_IC_PRIKEY_INDEX_DEFAULT;
	}

	chg_info("oplus_sec_get_verify_data index:%d\n", chip->sec_ic_index);
	rc = oplus_sec_get_verify_data(chip, chip->sec_ic_index);
	if (rc < 0)
		chg_err("get verify auth data fail, rc=%d\n", rc);
}

static void oplus_sec_set_sec_ic_prikey_work(struct work_struct *work)
{
	struct oplus_sec *chip = container_of(work, struct oplus_sec,
		set_sec_ic_prikey_work);
	int rc;

	rc = oplus_gauge_sec_set_prikey(chip->gauge_topic, chip->sec_ic_index,
		chip->auth_data, SEC_VERIFY_AUTH_DATA_SIZE);
	if (rc < 0 && rc != -ENOTSUPP)
		chg_err("set prikey fail, rc=%d\n", rc);
}

static void oplus_sec_exit_daemon_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_sec *chip = container_of(dwork, struct oplus_sec, exit_daemon_work);

	oplus_sec_exit_daemon_process(chip);
}

static void oplus_sec_start_rw_check_work(struct work_struct *work)
{
	struct oplus_sec *chip = container_of(work, struct oplus_sec, start_rw_check_work);
	int rc;

	if (chip->gauge_topic == NULL) {
		atomic_set(&chip->rw_check_res, -EINVAL);
		return;
	}

	rc = oplus_sec_test_rw_check(chip->gauge_topic);
	atomic_set(&chip->rw_check_res, rc);
}

static void oplus_sec_subscribe_gauge_topic(struct oplus_mms *topic, void *priv_data)
{
	struct oplus_sec *chip = priv_data;

	chip->gauge_topic = topic;
	schedule_work(&chip->get_verify_data_work);
}

static ssize_t oplus_sec_dev_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *offset)
{
	return count;
}

static ssize_t oplus_sec_dev_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct oplus_sec *chip = filp->private_data;
	struct sec_dev_cmd cmd;
	int rc;

	mutex_lock(&chip->read_lock);
	rc = wait_event_interruptible(chip->read_wq, chip->cmd_data_ok);
	mutex_unlock(&chip->read_lock);
	if (rc)
		return rc;
	if (!chip->cmd_data_ok)
		chg_err("sec false wakeup, rc=%d\n", rc);

	mutex_lock(&chip->cmd_data_lock);
	chip->cmd_data_ok = false;
	memmove(&cmd, &chip->cmd, sizeof(struct sec_dev_cmd));
	mutex_unlock(&chip->cmd_data_lock);
	if (copy_to_user(buf, &cmd, sizeof(struct sec_dev_cmd))) {
		chg_err("failed to copy to user space\n");
		return -EFAULT;
	}

	return sizeof(struct sec_dev_cmd);
}

static int oplus_sec_dev_open(struct inode *inode, struct file *filp)
{
	struct oplus_sec *chip = container_of(filp->private_data,
		struct oplus_sec, misc_dev);

	filp->private_data = chip;
	chg_info("%s: %d-%d\n", SEC_MISC_DEV_NAME, imajor(inode), iminor(inode));
	return 0;
}

#define SEC_IOC_MAGIC			0x73
#define SEC_NOTIFY_GET_AUTH_DATA	_IOW(SEC_IOC_MAGIC, 1, char)

static long oplus_sec_dev_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct oplus_sec *chip = filp->private_data;
	void __user *argp = (void __user *)arg;
	int rc;

	switch (cmd) {
	case SEC_NOTIFY_GET_AUTH_DATA:
		rc = copy_from_user(&chip->auth_data, argp, SEC_VERIFY_AUTH_DATA_SIZE);
		if (rc) {
			chg_err("failed copy to user space\n");
			return rc;
		}
		chip->auth_data_ok = true;
		schedule_work(&chip->set_sec_ic_prikey_work);
		chg_info("auth data ok\n");
		if (delayed_work_pending(&chip->exit_daemon_work)) {
			cancel_delayed_work_sync(&chip->exit_daemon_work);
			schedule_delayed_work(&chip->exit_daemon_work, 0);
		}
		break;
	default:
		chg_err("bad ioctl %u\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations oplus_sec_dev_fops = {
	.owner			= THIS_MODULE,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	.llseek			= noop_llseek,
#else
	.llseek			= no_llseek,
#endif
	.write			= oplus_sec_dev_write,
	.read			= oplus_sec_dev_read,
	.open			= oplus_sec_dev_open,
	.unlocked_ioctl		= oplus_sec_dev_ioctl,
};

static int oplus_sec_misc_dev_reg(struct oplus_sec *chip)
{
	int rc;

	chg_info("oplus_sec_misc_dev_reg");
	mutex_init(&chip->read_lock);
	mutex_init(&chip->cmd_data_lock);
	init_waitqueue_head(&chip->read_wq);
	chip->cmd_data_ok = false;
	init_completion(&chip->cmd_ack);

	chip->misc_dev.minor = MISC_DYNAMIC_MINOR;
	chip->misc_dev.name = SEC_MISC_DEV_NAME;
	chip->misc_dev.fops = &oplus_sec_dev_fops;
	rc = misc_register(&chip->misc_dev);
	if (rc)
		chg_err("misc_register failed, rc=%d\n", rc);

	return rc;
}

struct oplus_sec* oplus_sec_init(void)
{
	int rc;
	struct oplus_sec *chip;

	chg_info("oplus_sec_init");
	chip = kzalloc(sizeof(struct oplus_sec), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return NULL;
	}
	g_chip = chip;
	mutex_init(&chip->sec_mutex);
	INIT_WORK(&chip->get_verify_data_work, oplus_sec_get_verify_data_work);
	INIT_WORK(&chip->set_sec_ic_prikey_work, oplus_sec_set_sec_ic_prikey_work);
	INIT_DELAYED_WORK(&chip->exit_daemon_work, oplus_sec_exit_daemon_work);
	INIT_WORK(&chip->start_rw_check_work, oplus_sec_start_rw_check_work);
	rc = oplus_sec_misc_dev_reg(chip);
	if (rc < 0) {
		chg_err("oplus_sec_misc_dev_reg fail, rc=%d\n", rc);
		kfree(chip);
		return NULL;
	}

	atomic_set(&chip->rw_check_res, 0);

	oplus_mms_wait_topic("gauge", oplus_sec_subscribe_gauge_topic, chip);

	return chip;
}

void oplus_sec_release(struct oplus_sec *chip)
{
	if (chip == NULL)
		return;
	g_chip = NULL;
	mutex_destroy(&chip->sec_mutex);
	kfree(chip);
}
