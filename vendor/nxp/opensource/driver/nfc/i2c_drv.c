/******************************************************************************
 * Copyright (C) 2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013-2022 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/
/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 ****************************************************************************/
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 ****************************************************************************/

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include "common.h"
#include <soc/oplus/boot/boot_mode.h>
//#if IS_ENABLED(CONFIG_NXP_NFC_VBAT_MONITOR)
#include "nfc_vbat_monitor.h"
//#endif CONFIG_NXP_NFC_VBAT_MONITOR

#define MIXED_CHIPSET    "mixed-chipset"
#define MAX_ID_COUNT     5
#define SUPPORT_CHIPSET_LIST     "SN100T|SN110T|SN220T|SN220U|SN220P|SN220E|PN560"
struct id_entry {
    u32 key;
    const char *chipset;
};
/**
 * i2c_disable_irq()
 *
 * Check if interrupt is disabled or not
 * and disable interrupt
 *
 * Return: int
 */
int i2c_disable_irq(struct nfc_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->i2c_dev.irq_enabled_lock, flags);
	if (dev->i2c_dev.irq_enabled) {
		disable_irq_nosync(dev->i2c_dev.client->irq);
		dev->i2c_dev.irq_enabled = false;
	}
	spin_unlock_irqrestore(&dev->i2c_dev.irq_enabled_lock, flags);

	return 0;
}

/**
 * i2c_enable_irq()
 *
 * Check if interrupt is enabled or not
 * and enable interrupt
 *
 * Return: int
 */
int i2c_enable_irq(struct nfc_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->i2c_dev.irq_enabled_lock, flags);
	if (!dev->i2c_dev.irq_enabled) {
		dev->i2c_dev.irq_enabled = true;
		enable_irq(dev->i2c_dev.client->irq);
	}
	spin_unlock_irqrestore(&dev->i2c_dev.irq_enabled_lock, flags);

	return 0;
}

static irqreturn_t i2c_irq_handler(int irq, void *dev_id)
{
	struct nfc_dev *nfc_dev = dev_id;
	struct i2c_dev *i2c_dev = &nfc_dev->i2c_dev;

	if (device_may_wakeup(&i2c_dev->client->dev))
		pm_wakeup_event(&i2c_dev->client->dev, WAKEUP_SRC_TIMEOUT);

	i2c_disable_irq(nfc_dev);
	wake_up(&nfc_dev->read_wq);

	return IRQ_HANDLED;
}

int i2c_read(struct nfc_dev *nfc_dev, char *buf, size_t count, int timeout)
{
	int ret;
	struct i2c_dev *i2c_dev = &nfc_dev->i2c_dev;
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;
	uint16_t i = 0;
	uint16_t disp_len = GET_IPCLOG_MAX_PKT_LEN(count);

	pr_debug("NxpDrv: %s: reading %zu bytes.\n", __func__, count);

	if (timeout > NCI_CMD_RSP_TIMEOUT_MS)
		timeout = NCI_CMD_RSP_TIMEOUT_MS;

	if (count > MAX_NCI_BUFFER_SIZE)
		count = MAX_NCI_BUFFER_SIZE;

	if (!gpio_get_value(nfc_gpio->irq)) {
		while (1) {
			ret = 0;
			if (!i2c_dev->irq_enabled) {
				i2c_dev->irq_enabled = true;
				enable_irq(i2c_dev->client->irq);
			}
			if (!gpio_get_value(nfc_gpio->irq)) {
				if (timeout) {
					ret = wait_event_interruptible_timeout(
						nfc_dev->read_wq,
						!i2c_dev->irq_enabled,
						msecs_to_jiffies(timeout));
					if (ret <= 0) {
						pr_err("NxpDrv: %s: timeout error\n",
						       __func__);
						goto err;
					}
				} else {
					ret = wait_event_interruptible(
						nfc_dev->read_wq,
						!i2c_dev->irq_enabled);
					if (ret) {
						pr_err("NxpDrv: %s: err wakeup of wq\n",
						       __func__);
						goto err;
					}
				}
			}
//#if IS_ENABLED(CONFIG_NXP_NFC_VBAT_MONITOR)
			if (nfc_dev->nfc_vbat_monitor.vbat_monitor_status) {
				pr_debug("%s: NFC recovering  state\n",
					 __func__);
				nfc_dev->nfc_vbat_monitor.vbat_monitor_status =
					false;
				ret = -ENOTCONN;
				goto err;
			}
//#endif /* CONFIG_NXP_NFC_VBAT_MONITOR */
			i2c_disable_irq(nfc_dev);

			if (gpio_get_value(nfc_gpio->irq))
				break;
			if(!nfc_dev->secure_zone) {
				if (!gpio_get_value(nfc_gpio->ven)) {
					pr_info("NxpDrv: %s: releasing read\n", __func__);
					ret = -EIO;
					goto err;
				}
			}
			/*
			 * NFC service wanted to close the driver so,
			 * release the calling reader thread asap.
			 *
			 * This can happen in case of nfc node close call from
			 * eSE HAL in that case the NFC HAL reader thread
			 * will again call read system call
			 */
			if (nfc_dev->release_read) {
				pr_debug("NxpDrv: %s: releasing read\n", __func__);
				return 0;
			}
			pr_warn("NxpDrv: %s: spurious interrupt detected\n", __func__);
		}
	}

	memset(buf, 0x00, count);
	/* Read data */
	ret = i2c_master_recv(nfc_dev->i2c_dev.client, buf, count);
	NFCLOG_IPC(nfc_dev, false, "%s of %d bytes, ret %d", __func__, count,
								ret);
	if (ret <= 0) {
		pr_err("NxpDrv: %s: returned %d\n", __func__, ret);
		goto err;
	}

	for (i = 0; i < disp_len; i++)
		NFCLOG_IPC(nfc_dev, false, " %02x", buf[i]);

	/* check if it's response of cold reset command
	 * NFC HAL process shouldn't receive this data as
	 * command was sent by esepowermanager
	 */
	if (nfc_dev->cold_reset.rsp_pending && nfc_dev->cold_reset.cmd_buf
		&& (buf[0] == PROP_NCI_RSP_GID)
		&& (buf[1] == nfc_dev->cold_reset.cmd_buf[1])) {
		read_cold_reset_rsp(nfc_dev, buf);
		nfc_dev->cold_reset.rsp_pending = false;
		wake_up_interruptible(&nfc_dev->cold_reset.read_wq);
		/*
		 * NFC process doesn't know about cold reset command
		 * being sent as it was initiated by eSE process
		 * we shouldn't return any data to NFC process
		 */
		return 0;
	}

err:
	return ret;
}

int i2c_write(struct nfc_dev *nfc_dev, const char *buf, size_t count,
	      int max_retry_cnt)
{
	int ret = -EINVAL;
	int retry_cnt;
	uint16_t i = 0;
	uint16_t disp_len = GET_IPCLOG_MAX_PKT_LEN(count);
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;

	if (count > MAX_DL_BUFFER_SIZE)
		count = MAX_DL_BUFFER_SIZE;

	pr_debug("NxpDrv: %s: writing %zu bytes.\n", __func__, count);
	NFCLOG_IPC(nfc_dev, false, "%s sending %d B", __func__, count);

	for (i = 0; i < disp_len; i++)
	    NFCLOG_IPC(nfc_dev, false, " %02x", buf[i]);
	/*
	 * Wait for any pending read for max 15ms before write
	 * This is to avoid any packet corruption during read, when
	 * the host cmds resets NFCC during any parallel read operation
	 */
	for (retry_cnt = 1; retry_cnt <= MAX_WRITE_IRQ_COUNT; retry_cnt++) {
		if (gpio_get_value(nfc_gpio->irq)) {
			pr_warn("NxpDrv: %s: irq high during write, wait\n", __func__);
			usleep_range(WRITE_RETRY_WAIT_TIME_US,
				     WRITE_RETRY_WAIT_TIME_US + 100);
		} else {
			break;
		}
		if (retry_cnt == MAX_WRITE_IRQ_COUNT &&
		    gpio_get_value(nfc_gpio->irq)) {
			pr_warn("NxpDrv: %s: allow after maximum wait\n", __func__);
		}
	}

	for (retry_cnt = 1; retry_cnt <= max_retry_cnt; retry_cnt++) {
		ret = i2c_master_send(nfc_dev->i2c_dev.client, buf, count);
		NFCLOG_IPC(nfc_dev, false, "%s ret %d", __func__, ret);
		if (ret <= 0) {
			pr_warn("NxpDrv: %s: write failed ret(%d), maybe in standby\n",
				__func__, ret);
			usleep_range(WRITE_RETRY_WAIT_TIME_US,
				     WRITE_RETRY_WAIT_TIME_US + 100);
		} else if (ret != count) {
			pr_err("NxpDrv: %s: failed to write %d\n", __func__, ret);
			ret = -EIO;
		} else if (ret == count)
			break;
	}
	return ret;
}

ssize_t nfc_i2c_dev_read(struct file *filp, char __user *buf, size_t count,
			 loff_t *offset)
{
	int ret;
	struct nfc_dev *nfc_dev = (struct nfc_dev *)filp->private_data;

	if (!nfc_dev) {
		pr_err("NxpDrv: %s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}
	mutex_lock(&nfc_dev->read_mutex);
	if (count > MAX_NCI_BUFFER_SIZE)
		count = MAX_NCI_BUFFER_SIZE;

	if (filp->f_flags & O_NONBLOCK) {
		ret = i2c_master_recv(nfc_dev->i2c_dev.client, nfc_dev->read_kbuf, count);
		pr_debug("NxpDrv: %s: NONBLOCK read ret = %d\n", __func__, ret);
	} else {
		ret = i2c_read(nfc_dev, nfc_dev->read_kbuf, count, 0);
	}
	if (ret > 0) {
		if (copy_to_user(buf, nfc_dev->read_kbuf, ret)) {
			pr_warn("NxpDrv: %s: failed to copy to user space\n", __func__);
			ret = -EFAULT;
		}
	}
	mutex_unlock(&nfc_dev->read_mutex);
	return ret;
}

ssize_t nfc_i2c_dev_write(struct file *filp, const char __user *buf,
			  size_t count, loff_t *offset)
{
	int ret;
	struct nfc_dev *nfc_dev = (struct nfc_dev *)filp->private_data;

	if (count > MAX_DL_BUFFER_SIZE)
		count = MAX_DL_BUFFER_SIZE;

	if (!nfc_dev) {
		pr_err("NxpDrv: %s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&nfc_dev->write_mutex);
	if (copy_from_user(nfc_dev->write_kbuf, buf, count)) {
		pr_err("NxpDrv: %s: failed to copy from user space\n", __func__);
		mutex_unlock(&nfc_dev->write_mutex);
		return -EFAULT;
	}
	ret = i2c_write(nfc_dev, nfc_dev->write_kbuf, count, NO_RETRY);
	mutex_unlock(&nfc_dev->write_mutex);
	return ret;
}

static const struct file_operations nfc_i2c_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = nfc_i2c_dev_read,
	.write = nfc_i2c_dev_write,
	.open = nfc_dev_open,
	.flush = nfc_dev_flush,
	.release = nfc_dev_close,
	.unlocked_ioctl = nfc_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nfc_dev_compat_ioctl,
#endif
};
static int read_id_properties(struct device_node *np, u32 id_count, struct id_entry *id_entries)
{
    int err;
    u32 i;
    char propname[30];
    for (i = 0; i < id_count; i++) {
        snprintf(propname, sizeof(propname), "id-%u-key", i);
        err = of_property_read_u32(np, propname, &id_entries[i].key);
        if (err) {
          pr_err("Failed to read dts node:%s\n",propname);
          return err;
        }
        snprintf(propname, sizeof(propname), "id-%u-value-chipset", i);
        err = of_property_read_string(np, propname, &id_entries[i].chipset);
        if (err) {
          pr_err("Failed to read dts node:%s\n",propname);
          return err;
        }
    }
    pr_info("read_id_properties success");
    return 0;
}
static int get_gpio_value(struct device_node *np, int *gpio_value)
{
    int gpio_num = of_get_named_gpio(np, "id-gpio", 0);
    if (!gpio_is_valid(gpio_num)) {
        pr_err("id-gpio is not valid\n");
        return -EINVAL;
    }
    *gpio_value = gpio_get_value(gpio_num);
    pr_info("%s, id gpio value is %d", __func__, *gpio_value);
    return 0;
}
static int checkNfcChip(struct device *dev)
{
    struct device_node *np = NULL;
    u32 id_count;
    int i, gpio_value, err;
    bool found = false;
    struct id_entry *id_entries = NULL;
    uint32_t mixed_chipset;
    if (NULL == dev)
    {
        pr_err("%s dev is NULL", __func__);
        return -ENOENT;
    }
    np = dev->of_node;
    if (NULL == np)
    {
        pr_err("%s dev->of_node is NULL", __func__);
        return -ENOENT;
    }
    if (of_property_read_u32(np, MIXED_CHIPSET, &mixed_chipset))
    {
        pr_info("%s, read dts property mixed-chipset failed", __func__);
        return 0;
    }
    else
    {
        if (1 == mixed_chipset)
        {
            pr_info("%s, the value of dts property mixed-chipset is 1(true)", __func__);
            err = of_property_read_u32(np, "id_count", &id_count);
            if (err)
            {
                pr_err("%s read dts property id_count failed", __func__);
                return err;
            }
            if (id_count >= MAX_ID_COUNT)
            {
                pr_err("%s error: id_count is more than %d", __func__, MAX_ID_COUNT);
                return -ENOENT;
            }
            id_entries = kzalloc(sizeof(struct id_entry) * id_count, GFP_DMA | GFP_KERNEL);
            if(NULL == id_entries)
            {
                pr_err("%s error: can not kzalloc memory for id_entry", __func__);
                return -ENOMEM;
            }
            err = read_id_properties(np, id_count,id_entries);
            if (err)
            {
                pr_err("%s error: read_id_properties failed", __func__);
                kfree(id_entries);
                return err;
            }
            err = get_gpio_value(np, &gpio_value);
            if (err)
            {
                pr_err("%s error: get_gpio_value failed", __func__);
                kfree(id_entries);
                return err;
            }
            for (i = 0; i < id_count; i++)
            {
                if (id_entries[i].key == gpio_value)
                {
                    if (strstr(SUPPORT_CHIPSET_LIST, id_entries[i].chipset) == NULL)
                    {
                        pr_err("%s this nfc chipset:%s does not correspond to this nfc driver", __func__, id_entries[i].chipset);
                        err = -EINVAL;
                        kfree(id_entries);
                        return err;
                    }
                    pr_debug("%s this nfc chipset:%s corresponds to this nfc driver", __func__, id_entries[i].chipset);
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                pr_err("%s no matching key found for GPIO value\n", __func__);
                err = -EINVAL;
                kfree(id_entries);
                return err;
            }
            pr_info("%s checkNfcChip success\n", __func__);
            kfree(id_entries);
            return 0;
        }
        else if (0 == mixed_chipset)
        {
            pr_info("%s, the value of dts property mixed-chipset is 0(false)", __func__);
            return 0;
        }
        else
        {
            pr_err("%s, mixed-chipset's value is wrong,it is neither 1 nor 0", __func__);
            return -ENOENT;
        }
    }
}
int nfc_i2c_dev_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct nfc_dev *nfc_dev = NULL;
	struct i2c_dev *i2c_dev = NULL;
	struct platform_configs *nfc_configs = NULL;
	struct platform_gpio *nfc_gpio = NULL;
	pr_debug("NxpDrv: %s: enter\n", __func__);
	ret = checkNfcChip(&client->dev);
	if (ret) {
		pr_err("NxpDrv: %s: failed to checkNfcChip\n", __func__);
		goto err;
	}
	nfc_dev = kzalloc(sizeof(struct nfc_dev), GFP_KERNEL);
	if (nfc_dev == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	nfc_configs = &nfc_dev->configs;
	nfc_gpio = &nfc_configs->gpio;
	/* retrieve details of gpios from dt */
	ret = nfc_parse_dt(&client->dev,nfc_configs, PLATFORM_IF_I2C);
	if (ret) {
		pr_err("NxpDrv: %s: failed to parse dt\n", __func__);
		goto err_free_nfc_dev;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("NxpDrv: %s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_free_nfc_dev;
	}
	nfc_dev->read_kbuf = kzalloc(MAX_NCI_BUFFER_SIZE, GFP_DMA | GFP_KERNEL);
	if (!nfc_dev->read_kbuf) {
		ret = -ENOMEM;
		goto err_free_nfc_dev;
	}
	nfc_dev->write_kbuf = kzalloc(MAX_DL_BUFFER_SIZE, GFP_DMA | GFP_KERNEL);
	if (!nfc_dev->write_kbuf) {
		ret = -ENOMEM;
		goto err_free_read_kbuf;
	}
	nfc_dev->interface = PLATFORM_IF_I2C;
	nfc_dev->nfc_state = NFC_STATE_NCI;
	nfc_dev->i2c_dev.client = client;
	i2c_dev = &nfc_dev->i2c_dev;
	nfc_dev->nfc_read = i2c_read;
	nfc_dev->nfc_write = i2c_write;
	nfc_dev->nfc_enable_intr = i2c_enable_irq;
	nfc_dev->nfc_disable_intr = i2c_disable_irq;

	ret = configure_gpio(nfc_gpio->irq, GPIO_IRQ);
	if (ret <= 0) {
		pr_err("NxpDrv: %s: unable to request nfc irq gpio [%d]\n", __func__,
		       nfc_gpio->irq);
		goto err_free_gpio;
	}
	client->irq = ret;

	/* init mutex and queues */
	init_waitqueue_head(&nfc_dev->read_wq);
	mutex_init(&nfc_dev->read_mutex);
	mutex_init(&nfc_dev->write_mutex);
	mutex_init(&nfc_dev->dev_ref_mutex);
	spin_lock_init(&i2c_dev->irq_enabled_lock);
	ret = nfc_misc_register(nfc_dev, &nfc_i2c_dev_fops, DEV_COUNT,
				NFC_CHAR_DEV_NAME, CLASS_NAME);
	if (ret) {
		pr_err("NxpDrv: %s: nfc_misc_register failed\n", __func__);
		goto err_mutex_destroy;
	}
	/* interrupt initializations */
	pr_info("NxpDrv: %s: requesting IRQ %d\n", __func__, client->irq);
	i2c_dev->irq_enabled = true;
	ret = request_irq(client->irq, i2c_irq_handler, IRQF_TRIGGER_HIGH,
			  client->name, nfc_dev);
	if (ret) {
		pr_err("NxpDrv: %s: request_irq failed\n", __func__);
		goto err_nfc_misc_unregister;
	}
//#if IS_ENABLED(CONFIG_NXP_NFC_VBAT_MONITOR)
	ret = nfc_vbat_monitor_init(nfc_dev, nfc_gpio, client);
	if (ret) {
		pr_err("%s: nfcc vbat monitor init failed, ret: %d\n", __func__, ret);
		//goto err_nfc_misc_unregister;
	}
//#endif /* CONFIG_NXP_NFC_VBAT_MONITOR */
	i2c_disable_irq(nfc_dev);

	ret = nfc_ldo_config(&client->dev, nfc_dev);
	if (ret) {
		pr_err("NxpDrv: LDO config failed\n");
		goto err_ldo_config_failed;
	}
#ifdef NFC_SECURE_PERIPHERAL_ENABLED
	if( nfc_dev->configs.CNSS_NFC_HW_SECURE_ENABLE == true) {
	    /*Check NFC Secure Zone status*/
	    if(!nfc_hw_secure_check()) {
		   nfc_post_init(nfc_dev);
		   nfc_dev->secure_zone = false;
	    }
	    else {
		   nfc_dev->secure_zone = true;
            }
	    pr_info("NxpDrv: %s:nfc secure_zone = %s", __func__, nfc_dev->secure_zone ? "true" : "false");
	}else {
		nfc_post_init(nfc_dev);
	}
#else
	nfc_dev->secure_zone = false;
	nfc_post_init(nfc_dev);
#endif

	if (nfc_dev->configs.clk_pin_voting)
		nfc_dev->clk_run = false;
	else
		nfc_dev->clk_run = true;

	device_init_wakeup(&client->dev, true);
	i2c_set_clientdata(client, nfc_dev);
	i2c_dev->irq_wake_up = false;
	nfc_dev->is_ese_session_active = false;

	dev_err(&client->dev,"%s: get boot mode = %d \n", __func__, get_boot_mode());
	if(get_boot_mode() == MSM_BOOT_MODE__FACTORY){
		dev_err(&client->dev,"%s: enter ftm mode, set ven = 0\n", __func__);
		gpio_set_ven(nfc_dev, 0);
	}
	pr_info("NxpDrv: %s: probing nfc i2c successfully\n", __func__);
	return 0;


err_ldo_config_failed:
	free_irq(client->irq, nfc_dev);
err_nfc_misc_unregister:
	nfc_misc_unregister(nfc_dev, DEV_COUNT);
err_mutex_destroy:
	mutex_destroy(&nfc_dev->dev_ref_mutex);
	mutex_destroy(&nfc_dev->read_mutex);
	mutex_destroy(&nfc_dev->write_mutex);
err_free_gpio:
	gpio_free_all(nfc_dev);
	kfree(nfc_dev->write_kbuf);
err_free_read_kbuf:
	kfree(nfc_dev->read_kbuf);
err_free_nfc_dev:
	kfree(nfc_dev);
err:
	pr_err("NxpDrv: %s: probing not successful, check hardware\n", __func__);
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
void nfc_i2c_dev_remove(struct i2c_client *client)
#else
int nfc_i2c_dev_remove(struct i2c_client *client)
#endif
{
	struct nfc_dev *nfc_dev = NULL;

	pr_info("NxpDrv: %s: remove device\n", __func__);
	nfc_dev = i2c_get_clientdata(client);
	if (!nfc_dev) {
		pr_err("NxpDrv: %s: device doesn't exist anymore\n", __func__);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
		return -ENODEV;
#endif
	}
	if (nfc_dev->dev_ref_count > 0) {
		pr_err("NxpDrv: %s: device already in use\n", __func__);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
		return -EBUSY;
#endif
	}

	gpio_set_value(nfc_dev->configs.gpio.ven, 0);
	// HW dependent delay before LDO goes into LPM mode
	usleep_range(10000, 10100);
	if (nfc_dev->reg) {
		nfc_ldo_unvote(nfc_dev);
		regulator_put(nfc_dev->reg);
	}

	device_init_wakeup(&client->dev, false);
	free_irq(client->irq, nfc_dev);
//#if IS_ENABLED(CONFIG_NXP_NFC_VBAT_MONITOR)
	if (gpio_is_valid(nfc_dev->nfc_vbat_monitor.irq_num)) {
		free_irq(nfc_dev->nfc_vbat_monitor.irq_num, nfc_dev);
	}
//#endif /* CONFIG_NXP_NFC_VBAT_MONITOR */
	nfc_misc_unregister(nfc_dev, DEV_COUNT);
	mutex_destroy(&nfc_dev->dev_ref_mutex);
	mutex_destroy(&nfc_dev->read_mutex);
	mutex_destroy(&nfc_dev->write_mutex);
	gpio_free_all(nfc_dev);
	kfree(nfc_dev->read_kbuf);
	kfree(nfc_dev->write_kbuf);
	kfree(nfc_dev);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	return 0;
#endif
}

int nfc_i2c_dev_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nfc_dev *nfc_dev = i2c_get_clientdata(client);
	struct i2c_dev *i2c_dev = NULL;
	if (!nfc_dev) {
		pr_err("NxpDrv: %s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}
	i2c_dev = &nfc_dev->i2c_dev;

	NFCLOG_IPC(nfc_dev, false, "%s: irq_enabled = %d", __func__,
							i2c_dev->irq_enabled);

	if (device_may_wakeup(&client->dev) && i2c_dev->irq_enabled) {
		if (!enable_irq_wake(client->irq))
			i2c_dev->irq_wake_up = true;
	}
//#if IS_ENABLED(CONFIG_NXP_NFC_VBAT_MONITOR)
    if (gpio_is_valid(nfc_dev->nfc_vbat_monitor.irq_num)) {
        if (enable_irq_wake(nfc_dev->nfc_vbat_monitor.irq_num) != 0) {
            pr_err("%s: vbat irq wake enabled failed\n", __func__);
        }
    }
//#endif /* CONFIG_NXP_NFC_VBAT_MONITOR */
	pr_debug("NxpDrv: %s: irq_wake_up = %d", __func__, i2c_dev->irq_wake_up);
	return 0;
}

int nfc_i2c_dev_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nfc_dev *nfc_dev = i2c_get_clientdata(client);
	struct i2c_dev *i2c_dev = NULL;
	if (!nfc_dev) {
		pr_err("NxpDrv: %s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}
	i2c_dev = &nfc_dev->i2c_dev;

	NFCLOG_IPC(nfc_dev, false, "%s: irq_wake_up = %d", __func__,
							i2c_dev->irq_wake_up);

	if (device_may_wakeup(&client->dev) && i2c_dev->irq_wake_up) {
		if (!disable_irq_wake(client->irq))
			i2c_dev->irq_wake_up = false;
	}
//#if IS_ENABLED(CONFIG_NXP_NFC_VBAT_MONITOR)
    if (gpio_is_valid(nfc_dev->nfc_vbat_monitor.irq_num)) {
        if (disable_irq_wake(nfc_dev->nfc_vbat_monitor.irq_num) != 0) {
            pr_err("%s: vbat irq wake disabled failed\n", __func__);
        }
    }
//#endif /* CONFIG_NXP_NFC_VBAT_MONITOR */
	pr_debug("NxpDrv: %s: irq_wake_up = %d", __func__, i2c_dev->irq_wake_up);
	return 0;
}

static const struct i2c_device_id nfc_i2c_dev_id[] = { { NFC_I2C_DEV_ID, 0 },
						       {} };

static const struct of_device_id nfc_i2c_dev_match_table[] = {
	{
		.compatible = NFC_I2C_DRV_STR,
	},
	{}
};

static const struct dev_pm_ops nfc_i2c_dev_pm_ops = { SET_SYSTEM_SLEEP_PM_OPS(
	nfc_i2c_dev_suspend, nfc_i2c_dev_resume) };

static struct i2c_driver nfc_i2c_dev_driver = {
	.id_table = nfc_i2c_dev_id,
	.probe = nfc_i2c_dev_probe,
	.remove = nfc_i2c_dev_remove,
	.driver = {
		.name = NFC_I2C_DRV_STR,
		.pm = &nfc_i2c_dev_pm_ops,
		.of_match_table = nfc_i2c_dev_match_table,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

MODULE_DEVICE_TABLE(of, nfc_i2c_dev_match_table);

static int __init nfc_i2c_dev_init(void)
{
	int ret = 0;

	pr_info("NxpDrv: %s: Loading NXP NFC I2C driver\n", __func__);
	ret = i2c_add_driver(&nfc_i2c_dev_driver);
	if (ret != 0)
		pr_err("NxpDrv: %s: NFC I2C add driver error ret %d\n", __func__, ret);
	return ret;
}

module_init(nfc_i2c_dev_init);

static void __exit nfc_i2c_dev_exit(void)
{
	pr_info("NxpDrv: %s: Unloading NXP NFC I2C driver\n", __func__);
	i2c_del_driver(&nfc_i2c_dev_driver);
}

module_exit(nfc_i2c_dev_exit);

MODULE_DESCRIPTION("NXP NFC I2C driver");
MODULE_LICENSE("GPL");
