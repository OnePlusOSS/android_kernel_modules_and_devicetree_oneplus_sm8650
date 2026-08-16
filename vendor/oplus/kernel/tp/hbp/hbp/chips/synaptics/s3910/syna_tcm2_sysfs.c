// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/**
 * @file syna_tcm2_sysfs.c
 *
 * This file implements cdev and ioctl interface in the reference driver.
 */

#include <linux/string.h>

#include "syna_tcm2.h"
#include "tcm/synaptics_touchcom_core_dev.h"
#include "tcm/synaptics_touchcom_func_base.h"
#include "tcm/synaptics_touchcom_func_touch.h"
#if (KERNEL_VERSION(5, 9, 0) <= LINUX_VERSION_CODE) || \
	defined(HAVE_UNLOCKED_IOCTL)
#define USE_UNLOCKED_IOCTL
#endif

#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL)
#define USE_COMPAT_IOCTL
#endif

#if (KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE)
#define REPLACE_KTIME
#endif

#define IF_ARG_NULL_OUT(arg) \
		do{ \
			if (IS_ERR_OR_NULL(arg)) { \
				hbp_err("%s:arg is NULL\n", __func__); \
				return -EINVAL; \
			} \
		}while(0) \

#define ENABLE_PID_TASK

#define SIG_ATTN (46)

/* structure for IOCTLs
 */
struct syna_ioctl_data {
	unsigned int data_length;
	unsigned int buf_size;
	unsigned char __user *buf;
};

#ifdef USE_COMPAT_IOCTL
struct syna_tcm_ioctl_data_compat {
	unsigned int data_length;
	unsigned int buf_size;
	compat_uptr_t __user *buf;
};
#endif

#define CHAR_DEVICE_NAME "tcm"
//#define CHAR_DEVICE_NAME "tcm_hbp"
#define PLATFORM_DRIVER_NAME "synaptics_tcm_hbp"
#define CHAR_DEVICE_MODE (0x0600)

/* defines the IOCTLs supported
 */
#define IOCTL_MAGIC 's'

/* Previous IOCTLs in early driver */
#define OLD_RESET_ID		(0x00)
#define OLD_SET_IRQ_MODE_ID	(0x01)
#define OLD_SET_RAW_MODE_ID	(0x02)
#define OLD_CONCURRENT_ID	(0x03)

#define IOCTL_OLD_RESET \
	_IO(IOCTL_MAGIC, OLD_RESET_ID)
#define IOCTL_OLD_SET_IRQ_MODE \
	_IOW(IOCTL_MAGIC, OLD_SET_IRQ_MODE_ID, int)
#define IOCTL_OLD_SET_RAW_MODE \
	_IOW(IOCTL_MAGIC, OLD_SET_RAW_MODE_ID, int)
#define IOCTL_OLD_CONCURRENT \
	_IOW(IOCTL_MAGIC, OLD_CONCURRENT_ID, int)

/* Standard IOCTLs in TCM2 driver */
#define STD_IOCTL_BEGIN		    (0x10)
#define STD_SET_PID_ID		    (0x11)
#define STD_ENABLE_IRQ_ID	    (0x12)
#define STD_RAW_READ_ID		    (0x13)
#define STD_RAW_WRITE_ID	    (0x14)
#define STD_GET_FRAME_ID	    (0x15)
#define STD_SEND_MESSAGE_ID     (0x16)
#define STD_SET_REPORTS_ID      (0x17)
#define STD_CHECK_FRAMES_ID     (0x18)
#define STD_CLEAN_OUT_FRAMES_ID (0x19)
#define STD_SET_APPLICATION_INFO_ID (0x1A)
#define STD_DO_HW_RESET_ID      (0x1B)

#define STD_DRIVER_CONFIG_ID	(0x21)
#define STD_DRIVER_GET_CONFIG_ID	(0x22)

#define CUS_INSERT_REQ_REPORT_DATA_ID (0xC0)
#define CUS_GET_POWER_STATUS_ID       (0xC1)  /* only used for TCM/TSM */
#define CUS_GET_DRIVER_STATUS_ID      (0xC2)  /* only used for tsDaemon */
#define CUS_SET_DRIVER_STATUS_ID      (0xC3)  /* only used for tsDaemon */

#define IOCTL_STD_IOCTL_BEGIN \
	_IOR(IOCTL_MAGIC, STD_IOCTL_BEGIN)
#define IOCTL_STD_SET_PID \
	_IOW(IOCTL_MAGIC, STD_SET_PID_ID, struct syna_ioctl_data *)
#define IOCTL_STD_ENABLE_IRQ \
	_IOW(IOCTL_MAGIC, STD_ENABLE_IRQ_ID, struct syna_ioctl_data *)
#define IOCTL_STD_RAW_READ \
	_IOR(IOCTL_MAGIC, STD_RAW_READ_ID, struct syna_ioctl_data *)
#define IOCTL_STD_RAW_WRITE \
	_IOW(IOCTL_MAGIC, STD_RAW_WRITE_ID, struct syna_ioctl_data *)
#define IOCTL_STD_GET_FRAME \
	_IOWR(IOCTL_MAGIC, STD_GET_FRAME_ID, struct syna_ioctl_data *)
#define IOCTL_STD_SEND_MESSAGE \
	_IOWR(IOCTL_MAGIC, STD_SEND_MESSAGE_ID, struct syna_ioctl_data *)
#define IOCTL_STD_SET_REPORT_TYPES \
	_IOW(IOCTL_MAGIC, STD_SET_REPORTS_ID, struct syna_ioctl_data *)
#define IOCTL_STD_CHECK_FRAMES \
	_IOWR(IOCTL_MAGIC, STD_CHECK_FRAMES_ID, struct syna_ioctl_data *)
#define IOCTL_STD_CLEAN_OUT_FRAMES \
	_IOWR(IOCTL_MAGIC, STD_CLEAN_OUT_FRAMES_ID, struct syna_ioctl_data *)
#define IOCTL_STD_SET_APPLICATION_INFO \
	_IOWR(IOCTL_MAGIC, STD_SET_APPLICATION_INFO_ID, struct syna_ioctl_data *)
#define IOCTL_STD_DO_HW_RESET \
	_IOWR(IOCTL_MAGIC, STD_DO_HW_RESET_ID, struct syna_ioctl_data *)

#define IOCTL_DRIVER_CONFIG \
	_IOW(IOCTL_MAGIC, STD_DRIVER_CONFIG_ID, struct syna_ioctl_data *)
#define IOCTL_DRIVER_GET_CONFIG \
	_IOR(IOCTL_MAGIC, STD_DRIVER_GET_CONFIG_ID, struct syna_ioctl_data *)

#define IOCTL_CUS_INSERT_REQ_REPORT_DATA \
	_IOWR(IOCTL_MAGIC, CUS_INSERT_REQ_REPORT_DATA_ID, struct syna_ioctl_data *)
#define IOCTL_CUS_GET_POWER_STATUS \
	_IOWR(IOCTL_MAGIC, CUS_GET_POWER_STATUS_ID, struct syna_ioctl_data *)
#define IOCTL_CUS_GET_DRIVER_STATUS \
	_IOWR(IOCTL_MAGIC, CUS_GET_DRIVER_STATUS_ID, struct syna_ioctl_data *)
#define IOCTL_CUS_SET_DRIVER_STATUS \
	_IOWR(IOCTL_MAGIC, CUS_SET_DRIVER_STATUS_ID, struct syna_ioctl_data *)

/* g_sysfs_dir represents the root directory of sysfs nodes being created
 */
static struct kobject *g_sysfs_dir;

/* g_cdev_buf is a temporary buffer storing the data from userspace
 */
static struct tcm_buffer g_cdev_cbuf;

/* The g_sysfs_io_polling_interval is used to set the polling interval
 * for syna_tcm_send_command from syna_cdev_ioctl_send_message.
 * It will set to the mode SYSFS_FULL_INTERRUPT for using the full
 * interrupt mode. The way to update this variable is through the
 * syna_cdev_ioctl_enable_irq.
 */
unsigned int g_sysfs_io_polling_interval;

/* The g_sysfs_extra_bytes_read allows caller to ask extra bytes
 * to read. Thus, driver may need to append the requested bytes.
 */
static int g_sysfs_extra_bytes_read;

static int g_sysfs_has_remove = 0;

/* a buffer to record the streaming report
 * considering touch report and another reports may be co-enabled
 * at the same time, give a little buffer here (3 sec x 300 fps)
 */
#define FIFO_QUEUE_MAX_FRAMES		(1200)
#define SEND_MESSAGE_HEADER_LENGTH	(3)

/* Indicate the interrupt status especially for sysfs using */
#define SYSFS_DISABLED_INTERRUPT		(0)
#define SYSFS_ENABLED_INTERRUPT			(1)

#define MINIMUM_WAITING_TIME			(10)

#define SYNA_RETRY_CNT 60

/* Define a data structure for driver parameters configurations
 *
 *      [Integer]  [   Field   ]  [ Description         ]
 *      -------------------------------------------------
 *         1       Drv Connection  bit-0 ~ 23 reserved
 *                                 bit-24~ 31 current touchcomm version being connected
 *         2       reserved
 *         3       reserved
 *         4       Bus Data Chunk  bus config, max chunk size for data transfer
 *         5       reserved
 *         6       reserved
 *         7       reserved
 *         8       reserved
 *         9       Drv Features    bit-0: '1' to enable predict reading
 *                                 bit-1 ~ 7  reserved
 *                                 bit-8 ~15  to set up the number of bytes for extra reads
 *                                 bit-16~31  reserved
 */
struct drv_param {
	union {
		struct {
			/* reserve fields */
			unsigned char reserved_0[3];
			unsigned char connection_touchcomm_version;
			/* bus config */
			unsigned int reserved_2__3[2];
			unsigned int bus_chunk_size;
			/* reserve fields */
			unsigned int reserved_5__8[4];
			/* features */
			unsigned char feature_predict_reads:1;
			unsigned char feature_reserve_b1__7:7;
			unsigned char feature_extra_reads:8;
			unsigned char feature_reserve_b16__23:8;
			unsigned char feature_reserve_b24__31:8;
		} __packed;
		unsigned int parameters[9];
	};
};


/**
 * declaration of sysfs attributes
 */
static struct attribute *attrs[] = {
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

/**
 * syna_sysfs_create_dir()
 *
 * Create a directory and register it with sysfs.
 * Then, create all defined sysfs files.
 *
 * @param
 *    [ in] tcm:  the driver handle
 *    [ in] pdev: an instance of platform device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_sysfs_create_dir(struct syna_tcm *tcm,
		struct platform_device *pdev)
{
	int retval = 0;

	g_sysfs_dir = kobject_create_and_add("sysfs",
			&pdev->dev.kobj);
	if (!g_sysfs_dir) {
		hbp_err("Fail to create sysfs directory\n");
		return -ENOTDIR;
	}

	tcm->sysfs_dir = g_sysfs_dir;

	retval = sysfs_create_group(g_sysfs_dir, &attr_group);
	if (retval < 0) {
		hbp_err("Fail to create sysfs group\n");

		kobject_put(tcm->sysfs_dir);
		return retval;
	}

	return 0;
}

/**
 * syna_cdev_ioctl_do_hw_reset()
 *
 * Perform the hardware reset with the selected reset method. The reset
 * option depends on the hardware design. The user has to add the
 * corresponding implementation in this function for the userspace
 * application.
 *
 * The reset options:
 *    [ Not support]: 0
 *    [   Reset Pin]: 1
 *    [ Power cycle]: 2
 *    [    ........]:
 *
 * @param
 *    [ in] tcm:           the driver handle
 *    [ in] ubuf_ptr:      points to a memory space from userspace
 *    [ in] buf_size:      size of given space
 *    [ in] data_size:     input data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
/*
static int syna_cdev_ioctl_do_hw_reset(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	unsigned int param = 0;
	unsigned char data[4] = {0};

	if (!tcm->is_connected) {
		hbp_err("Not connected\n");
		return _EINVAL;
	}

	if (buf_size < sizeof(data) || data_size < sizeof(data)) {
		hbp_err("Invalid sync data size, buf_size: %u\n", buf_size);
		retval = -EINVAL;
		goto exit;
	}

	retval = copy_from_user(data, ubuf_ptr, sizeof(data));
	if (retval) {
		hbp_err("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	// get the reset option param
	param = syna_pal_le4_to_uint(&data[0]);
	hbp_info("HW reset option: %u\n", param);

	if (param == 1) {
		if (!tcm->hw_if->ops_hw_reset) {
			hbp_err("No hardware reset support\n");
			retval = -ENODEV;
			goto exit;
		}

		tcm->hw_if->ops_hw_reset(tcm->hw_if);

		// to enable the interrupt for processing the identify report
		// after the hardware reset.
		//
		if (!tcm->hw_if->bdata_attn.irq_enabled) {
			tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
			// disable it and back to original status
			syna_pal_sleep_ms(100);
			//tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
			LOGW("HW reset: IRQ is forced to enable\n");
		} else {
			hbp_err("HW reset: IRQ already enabled\n");
		}
	} else {
		LOGW("Unsupported HW reset option(%u) selected\n", param);
		retval = -EINVAL;
		goto exit;
	}

	// check the fw setup in case the settings is changed
	retval = tcm->dev_set_up_app_fw(tcm);
	if (retval < 0) {
		hbp_err("HW reset: failed to set up the app fw\n");
		retval = -ENODATA;
		goto exit;
	}

exit:
	return retval;
}
*/
/**
 * syna_cdev_ioctl_application_info()
 *
 * To keep the userspace application information, the user shall apply
 * the corresponding defined format on userspace. Otherwise, data will
 * be void type.
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: size of actual data
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
/*
static int syna_cdev_ioctl_application_info(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	void *data = NULL;

	if ((buf_size < 1) || (buf_size < data_size)) {
		hbp_err("Invalid input buffer size, buf_size:%u, data_size:%u\n",
			buf_size, data_size);
		return -EINVAL;
	}

	// free the allocated memory
	if (tcm->userspace_app_info != NULL)
		syna_pal_mem_free(tcm->userspace_app_info);

	tcm->userspace_app_info = syna_pal_mem_alloc(1, data_size);
	if (!(tcm->userspace_app_info)) {
		hbp_err("Failed to allocate user app info memory, size = %u\n",
			data_size);
		retval = -ENOMEM;
		goto exit;
	}

	syna_pal_mem_set(tcm->userspace_app_info, 0, data_size);
	data = tcm->userspace_app_info;

	retval = copy_from_user(data, ubuf_ptr, data_size);
	if (retval) {
		hbp_err("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	//
	// The user shall cast the retrieved data to the format defined
	// on userspace for the application.
	//

exit:
	return retval;
}
*/

/**
 * syna_cdev_ioctl_send_message()
 *
 * Send the command/message from userspace.
 *
 * For updating the g_sysfs_io_polling_interval, it need to be configured
 * by syna_cdev_ioctl_enable_irq from userspace.
 *
 * @param
 *    [ in] tcm:           the driver handle
 *    [ in/out] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:      size of given space
 *    [ in/out] msg_size:  size of message
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static unsigned int syna_delay_ms_resp(void)
{
	if (g_sysfs_io_polling_interval == RESP_IN_ATTN)
		return RESP_IN_ATTN;
	else
		return g_sysfs_io_polling_interval;
}

static int syna_cdev_ioctl_send_message(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int *msg_size)
{
	int retval = 0;
	unsigned char *data = NULL;
	unsigned char resp_code = 0;
	unsigned int payload_length = 0;
	unsigned int delay_ms_resp = RESP_IN_POLLING;
	struct tcm_buffer resp_data_buf;

	if (buf_size < SEND_MESSAGE_HEADER_LENGTH) {
		hbp_err("Invalid sync data size, buf_size:%d\n", buf_size);
		return -EINVAL;
	}

	if (*msg_size == 0) {
		hbp_err("Invalid message length, msg size: 0\n");
		return -EINVAL;
	}

	mutex_lock(&tcm->mutex);

	/* init a buffer for the response data */
	syna_tcm_buf_init(&resp_data_buf);

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, buf_size);
	if (retval < 0) {
		hbp_err("Fail to allocate memory for g_cdev_cbuf, size: %d\n",
			buf_size);
		goto exit;
	}

	data = g_cdev_cbuf.buf;

	retval = copy_from_user(data, ubuf_ptr, *msg_size);
	if (retval) {
		hbp_err("Fail to copy data from user space, size:%d\n", *msg_size);
		retval = -EBADE;
		goto exit;
	}

	payload_length = syna_pal_le2_to_uint(&data[1]);
	hbp_info("Command = 0x%02x, payload length = %d data:%*ph\n",
		data[0], payload_length, payload_length, &data[3]);

	delay_ms_resp = syna_delay_ms_resp();

	retval = syna_tcm_send_command(tcm->tcm_dev,
			data[0],
			&data[3],
			payload_length,
			&resp_code,
			&resp_data_buf,
			delay_ms_resp);
	if (retval < 0) {
		hbp_err("Fail to run command 0x%02x with payload len %d\n",
			data[0], payload_length);
		/* even if resp_code returned is not success
		 * this ioctl shall return the packet to caller
		 */
	}

	syna_pal_mem_set(data, 0, buf_size);
	/* status code */
	data[0] = resp_code;
	/* the length for response data */
	data[1] = (unsigned char)(resp_data_buf.data_length & 0xff);
	data[2] = (unsigned char)((resp_data_buf.data_length >> 8) & 0xff);

	hbp_info("resp data: 0x%02x 0x%02x 0x%02x\n",
		data[0], data[1], data[2]);

	/* response data */
	if (resp_data_buf.data_length > 0) {
		retval = syna_pal_mem_cpy(&g_cdev_cbuf.buf[3],
			(g_cdev_cbuf.buf_size - SEND_MESSAGE_HEADER_LENGTH),
			resp_data_buf.buf,
			resp_data_buf.buf_size,
			resp_data_buf.data_length);
		if (retval < 0) {
			hbp_err("Fail to copy resp data\n");
			goto exit;
		}
	}

	if (buf_size < resp_data_buf.data_length) {
		hbp_err("No enough space for data copy, buf_size:%d data:%d\n",
			buf_size, resp_data_buf.data_length);
		retval = -EOVERFLOW;
		goto exit;
	}

	retval = copy_to_user((void *)ubuf_ptr,
			data, resp_data_buf.data_length + 3);
	if (retval) {
		hbp_err("Fail to copy data to user space\n");
		retval = -EBADE;
		goto exit;
	}

	*msg_size = resp_data_buf.data_length + 3;
	retval = *msg_size;

exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);
	mutex_unlock(&tcm->mutex);

	syna_tcm_buf_release(&resp_data_buf);

	return retval;
}

/**
 * syna_cdev_ioctl_enable_irq()
 *
 * Enable or disable the irq via IOCTL.
 *
 * Expect to get 4 bytes unsigned int parameter from userspace:
 *    0:         disable the irq.
 *    1:         enable the irq and set g_sysfs_io_polling_interval
 *               to RESP_IN_ATTN
 *    otherwise: enable the irq and also assign the polling interval
 *               to a specific time, which will be used when calling
 *               syna_cdev_ioctl_send_message.
 *               the min. polling time is RESP_IN_POLLING
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: size of actual data
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */

static int syna_cdev_ioctl_enable_irq(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	unsigned int data;

	if ((buf_size < sizeof(data)) || (buf_size > PAGE_SIZE)
                || (data_size < sizeof(data)) || (data_size > PAGE_SIZE)) {
		hbp_err("Invalid sync data size, buf_size:%d, data_size:%d\n",
		    buf_size, data_size);
		return -EINVAL;
	}

	retval = copy_from_user(&data, ubuf_ptr, sizeof(data));
	if (retval) {
		hbp_err("Fail to copy data from user space, size:%d\n", retval);
		return -EBADE;
	}

	switch (data) {
	case SYSFS_DISABLED_INTERRUPT:
		hbp_dev_set_irq_status(tcm, false);

		g_sysfs_io_polling_interval =
			tcm->tcm_dev->msg_data.default_resp_reading;

		hbp_err("IRQ is disabled by userspace application\n");

		break;
	case SYSFS_ENABLED_INTERRUPT:
		hbp_dev_set_irq_status(tcm, true);

		g_sysfs_io_polling_interval = RESP_IN_ATTN;

		hbp_err("IRQ is enabled by userspace application\n");

		break;
	default:
		// recover the interrupt and also assign the polling interval
		hbp_dev_set_irq_status(tcm, true);

		g_sysfs_io_polling_interval = data;
		if (g_sysfs_io_polling_interval < RESP_IN_POLLING)
			g_sysfs_io_polling_interval = RESP_IN_POLLING;

		hbp_err("IRQ is enabled by userspace application\n");
		hbp_err("Polling interval is set to %d ms\n",
			g_sysfs_io_polling_interval);

		break;
	}

	return 0;
}

/**
 * syna_cdev_ioctl_store_pid()
 *
 * Save PID through IOCTL interface
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: size of actual data
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_store_pid(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	unsigned char *data = NULL;

	if (buf_size < 4 || buf_size > PAGE_SIZE) {
		hbp_err("Invalid sync data size, buf_size:%d\n", buf_size);
		return -EINVAL;
	}

	if (data_size < 4 || data_size > PAGE_SIZE) {
		hbp_err("Invalid data_size\n");
		return -EINVAL;
	}

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, buf_size);
	if (retval < 0) {
		hbp_err("Fail to allocate memory for g_cdev_buf, size: %d\n",
			buf_size);
		goto exit;
	}

	data = g_cdev_cbuf.buf;

	retval = copy_from_user(data, ubuf_ptr, data_size);
	if (retval) {
		hbp_err("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	tcm->proc_pid = syna_pal_le4_to_uint(&data[0]);

	hbp_info("PID: %d\n", (unsigned int)tcm->proc_pid);
#ifdef ENABLE_PID_TASK
	if (tcm->proc_pid) {
		tcm->proc_task = pid_task(
				find_vpid(tcm->proc_pid),
				PIDTYPE_PID);
		if (!tcm->proc_task) {
			hbp_err("Fail to locate task, pid: %d\n",
				(unsigned int)tcm->proc_pid);
			retval = -ESRCH;
			goto exit;
		}
	}
#endif
exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);

	return retval;
}
/**
 * syna_cdev_ioctl_raw_read()
 *
 * Read the data from device directly without routing to command wrapper
 * interface.
 *
 * @param
 *    [ in] tcm:         the driver handle
 *    [in/out] ubuf_ptr: ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size:    size of given space
 *    [ in] rd_size:     reading size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_raw_read(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int rd_size)
{
	int retval = 0;
	unsigned char *data = NULL;

	if (rd_size > buf_size) {
		hbp_err("Invalid sync data size, buf_size:%d, rd_size:%d\n",
			buf_size, rd_size);
		return -EINVAL;
	}

	if (rd_size == 0) {
		hbp_err("The read length is 0\n");
		return 0;
	}

	syna_pal_mutex_lock(&tcm->tcm_dev->msg_data.rw_mutex);

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, rd_size);
	if (retval < 0) {
		hbp_err("Fail to allocate memory for g_cdev_cbuf, size: %d\n",
			rd_size);
		goto exit;
	}

	data = g_cdev_cbuf.buf;

	retval = syna_tcm_read(tcm->tcm_dev,
			data,
			rd_size);
	if (retval < 0) {
		hbp_err("Fail to read raw data, size: %d\n", rd_size);
		goto exit;
	}

	if (copy_to_user((void *)ubuf_ptr, data, rd_size)) {
		hbp_err("Fail to copy data to user space\n");
		retval = -EBADE;
		goto exit;
	}

	retval = rd_size;

exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);

	syna_pal_mutex_unlock(&tcm->tcm_dev->msg_data.rw_mutex);

	return retval;
}
/**
 * syna_cdev_ioctl_raw_write()
 *
 * Write the given data to device directly without routing to command wrapper
 * interface.
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size: size of given space
 *    [ in] wr_size:  size to write
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_raw_write(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int wr_size)
{
	int retval = 0;
	unsigned char *data = NULL;

	if (wr_size > buf_size) {
		hbp_err("Invalid sync data size, buf_size:%d, wr_size:%d\n",
			buf_size, wr_size);
		return -EINVAL;
	}

	if (wr_size == 0) {
		hbp_err("Invalid written size\n");
		return -EINVAL;
	}

	syna_pal_mutex_lock(&tcm->tcm_dev->msg_data.rw_mutex);

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, wr_size);
	if (retval < 0) {
		hbp_err("Fail to allocate memory for g_cdev_cbuf, size: %d\n",
			wr_size);
		goto exit;
	}

	data = g_cdev_cbuf.buf;

	retval = copy_from_user(data, ubuf_ptr, wr_size);
	if (retval) {
		hbp_err("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	hbp_info("Write command: 0x%02x, length: 0x%02x, 0x%02x (size:%u)\n",
		data[0], data[1], data[2], wr_size);

	retval = syna_tcm_write(tcm->tcm_dev,
			data,
			wr_size);
	if (retval < 0) {
		hbp_err("Fail to write raw data, size: %u\n", wr_size);
		goto exit;
	}

	retval = wr_size;

exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);

	syna_pal_mutex_unlock(&tcm->tcm_dev->msg_data.rw_mutex);

	return retval;
}
/**
 * syna_cdev_ioctl_get_config_params()
 *
 * Return current configuration settings to user-space
 * The returned buffer array should be same as struct drv_param
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size: size of given space
 *    [ in] size:     size of array
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_get_config_params(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int size)
{
	int retval = 0;
	struct drv_param *param;

	if (size < sizeof(struct drv_param)) {
		hbp_err("Invalid data input, size: %u (expected: %lu)\n",
			size, sizeof(struct drv_param));
		return -EINVAL;
	}

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, sizeof(struct drv_param));
	if (retval < 0) {
		hbp_err("Fail to allocate memory for g_cdev_cbuf, size: %lu\n",
			sizeof(struct drv_param));
		goto exit;
	}

	syna_pal_mem_set(&g_cdev_cbuf.buf[0], 0x00, sizeof(struct drv_param));

	param = (struct drv_param *)&g_cdev_cbuf.buf[0];

	param->parameters[0] |= (tcm->tcm_dev->id_info.version) << 24;

	param->parameters[3] = MIN(RD_CHUNK_SIZE, WR_CHUNK_SIZE);

	param->parameters[8] = (unsigned int)((tcm->tcm_dev->msg_data.predict_reads & 0x01) |
						(g_sysfs_extra_bytes_read & 0xff) << 8);

	/* copy the info to user-space */
	retval = copy_to_user((void *)ubuf_ptr,
		(unsigned char *)param,
		sizeof(struct drv_param));
	if (retval) {
		hbp_err("Fail to copy data to user space\n");
		retval = -EBADE;
		goto exit;
	}

	retval = sizeof(struct drv_param);

exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);

	return retval;
}
/**
 * syna_cdev_ioctl_config()
 *
 * Set up and connect to touch controller.
 * The given buffer array should be same as struct drv_param
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size: size of given space
 *    [ in] in_size:  input data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_config(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int in_size)
{
	int retval = 0;
	struct drv_param *param;
	bool enable = false;
	int extra_bytes = 0;

	if (in_size < sizeof(struct drv_param)) {
		hbp_err("Invalid data input, size: %u (expected: %lu)\n",
			in_size, sizeof(struct drv_param));
		return -EINVAL;
	}

	syna_tcm_buf_lock(&g_cdev_cbuf);

	retval = syna_tcm_buf_alloc(&g_cdev_cbuf, sizeof(struct drv_param));
	if (retval < 0) {
		hbp_err("Fail to allocate memory for g_cdev_cbuf, size: %lu\n",
			sizeof(struct drv_param));
		goto exit;
	}

	retval = copy_from_user(&g_cdev_cbuf.buf[0], ubuf_ptr, sizeof(struct drv_param));
	if (retval) {
		hbp_err("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	param = (struct drv_param *)&g_cdev_cbuf.buf[0];

	/* update the config based on given data */
	if (tcm->tcm_dev) {
		/* config the read/write chunk, if user provided */
		if (param->bus_chunk_size > 0) {
			tcm->tcm_dev->max_rd_size = param->bus_chunk_size;
			tcm->tcm_dev->max_wr_size = param->bus_chunk_size;
		}
		/* config the feature of predict reading */
		enable = (param->feature_predict_reads == 1);
		if (tcm->tcm_dev->msg_data.predict_reads != enable) {
			hbp_err("request to %s predict reading\n", (enable) ? "enable":"disable");
			syna_tcm_enable_predict_reading(tcm->tcm_dev, enable);
		}
		/* config the feature of extra bytes reading */
		extra_bytes = param->feature_extra_reads;
		if (g_sysfs_extra_bytes_read != extra_bytes) {
			g_sysfs_extra_bytes_read = extra_bytes;
			hbp_err("request to read in %d extra bytes\n", extra_bytes);
		}
	}

exit:
	syna_tcm_buf_unlock(&g_cdev_cbuf);

	return retval;
}
/**
 * syna_cdev_ioctl_dispatch()
 *
 * Dispatch the IOCTLs operation based on the given code
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] code:      code for the target operation
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] ubuf_size: size of given space
 *    [ in] wr_size:   written data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_dispatch(struct syna_tcm *tcm,
		unsigned int code, const unsigned char *ubuf_ptr,
		unsigned int ubuf_size, unsigned int *data_size)
{
	int retval = 0;

	switch (code) {
	case STD_SET_PID_ID:
		hbp_info("STD_SET_PID_ID\n");
		retval = syna_cdev_ioctl_store_pid(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_ENABLE_IRQ_ID:
		retval = syna_cdev_ioctl_enable_irq(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		hbp_err("STD_ENABLE_IRQ_ID not support\n");
		break;
	case STD_RAW_WRITE_ID:
		hbp_info("STD_RAW_WRITE_ID\n");
		retval = syna_cdev_ioctl_raw_write(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_RAW_READ_ID:
		hbp_info("STD_RAW_READ_ID\n");
		retval = syna_cdev_ioctl_raw_read(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_GET_FRAME_ID:
		/*retval = syna_cdev_ioctl_get_frame(tcm,
				ubuf_ptr, ubuf_size, data_size);*/
		hbp_err("STD_GET_FRAME_ID not support\n");
		break;
	case STD_SEND_MESSAGE_ID:
		hbp_info("STD_SEND_MESSAGE_ID\n");
		retval = syna_cdev_ioctl_send_message(tcm,
				ubuf_ptr, ubuf_size, data_size);
		break;
	case STD_SET_REPORTS_ID:
		/*retval = syna_cdev_ioctl_set_reports(tcm,
				ubuf_ptr, ubuf_size, *data_size);*/
		hbp_err("STD_SET_REPORTS_ID not support\n");
		break;
	case STD_CHECK_FRAMES_ID:
		/*retval = syna_cdev_ioctl_check_frame(tcm,
				ubuf_ptr, ubuf_size, *data_size);*/
		hbp_err("STD_CHECK_FRAMES_ID not support\n");
		break;
	case STD_CLEAN_OUT_FRAMES_ID:
		/*hbp_info("STD_CLEAN_OUT_FRAMES_ID called\n");
		syna_cdev_clean_queue(tcm);
		retval = 0;*/
		hbp_err("STD_CLEAN_OUT_FRAMES_ID not support\n");
		break;
	case STD_SET_APPLICATION_INFO_ID:
		/*retval = syna_cdev_ioctl_application_info(tcm,
				ubuf_ptr, ubuf_size, *data_size);*/
		hbp_err("STD_SET_APPLICATION_INFO_ID not support\n");
		break;
	case STD_DO_HW_RESET_ID:
		/*retval = syna_cdev_ioctl_do_hw_reset(tcm,
				ubuf_ptr, ubuf_size, *data_size);*/
		hbp_err("STD_DO_HW_RESET_ID not support\n");
		break;
	case STD_DRIVER_CONFIG_ID:
		hbp_info("STD_DRIVER_CONFIG_ID\n");
		retval = syna_cdev_ioctl_config(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_DRIVER_GET_CONFIG_ID:
		hbp_info("STD_DRIVER_GET_CONFIG_ID\n");
		retval = syna_cdev_ioctl_get_config_params(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case CUS_INSERT_REQ_REPORT_DATA_ID:
		hbp_err("Not support this ioctl code: 0x%x\n", code);
		break;
	case CUS_GET_POWER_STATUS_ID:
		/*retval = syna_cdev_ioctl_get_power_status(tcm,
				ubuf_ptr, ubuf_size, data_size);*/
		hbp_err("CUS_GET_POWER_STATUS_ID not support\n");
		break;
	case CUS_GET_DRIVER_STATUS_ID:
		/*retval = syna_cdev_ioctl_get_driver_status(tcm,
				ubuf_ptr, ubuf_size, data_size);*/
		hbp_err("CUS_GET_DRIVER_STATUS_ID not support\n");
		break;
	case CUS_SET_DRIVER_STATUS_ID:
		/*retval = syna_cdev_ioctl_set_driver_status(tcm,
				ubuf_ptr, ubuf_size, *data_size);*/
		hbp_err("CUS_SET_DRIVER_STATUS_ID not support\n");
		break;
	default:
		hbp_err("Unknown ioctl code: 0x%x\n", code);
		return -EINVAL;
	}

	return retval;
}
/**
 * syna_cdev_ioctl_old_dispatch()
 *
 * Dispatch the old IOCTLs operation based on the given code
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] code:     code for the target operation
 *    [ in] arg:      argument passed from user-space
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_old_dispatch(struct syna_tcm *tcm,
		unsigned int code, unsigned long arg)
{
	int retval = 0;

	switch (code) {
	case OLD_RESET_ID:
		hbp_info("OLD_RESET_ID\n");
		retval = syna_tcm_reset(tcm->tcm_dev);
		if (retval < 0) {
			hbp_err("Fail to do reset\n");
			break;
		}

		syna_tcm_get_app_info(tcm->tcm_dev, &tcm->tcm_dev->app_info);

		break;
	case OLD_SET_IRQ_MODE_ID:
		tcm->char_dev_irq_disabled = !arg;
		hbp_info("OLD_SET_IRQ_MODE_ID, char_dev_irq_disabled = %u\n", tcm->char_dev_irq_disabled);
		break;

	case OLD_SET_RAW_MODE_ID:
		hbp_info("OLD_SET_RAW_MODE_ID, arg=%lu\n", arg);
		tcm->is_attn_redirecting = !!arg;
		break;

	case OLD_CONCURRENT_ID:
		hbp_info("OLD_CONCURRENT_ID\n");
		retval = 0;
		break;

	default:
		hbp_err("Unknown ioctl code: 0x%x\n", code);
		retval = -EINVAL;
		break;
	}

	return retval;
}

/**
 * syna_cdev_ioctls()
 *
 * Used to implements the IOCTL operations
 *
 * @param
 *    [ in] filp: represents the file descriptor
 *    [ in] cmd:  command code sent from userspace
 *    [ in] arg:  arguments sent from userspace
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
#ifdef USE_UNLOCKED_IOCTL
static long syna_cdev_ioctls(struct file *filp, unsigned int cmd,
		unsigned long arg)
#else
static int syna_cdev_ioctls(struct inode *inp, struct file *filp,
		unsigned int cmd, unsigned long arg)
#endif
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;
	struct syna_ioctl_data ioc_data;
	unsigned char *ptr = NULL;

	if (g_sysfs_has_remove == 1) {
		hbp_err("%s:driver is remove!!\n", __func__);
		return -EINVAL;
	}

	IF_ARG_NULL_OUT(g_sysfs_dir);
	p_kobj = g_sysfs_dir->parent;
	IF_ARG_NULL_OUT(p_kobj);
	p_dev = container_of(p_kobj, struct device, kobj);
	IF_ARG_NULL_OUT(p_dev);
	tcm = dev_get_drvdata(p_dev);
	IF_ARG_NULL_OUT(tcm);

	hbp_info("syna_cdev_ioctls Enter, IOC_ID:0x%02X\n", (unsigned int)_IOC_NR(cmd));

	syna_pal_mutex_lock(&tcm->extif_mutex);

	retval = 0;

	hbp_info("IOC_ID:0x%02X received\n", (unsigned int)_IOC_NR(cmd));

	/* handle the old IOCTLs */
	if ((_IOC_NR(cmd)) < STD_IOCTL_BEGIN) {
		retval = syna_cdev_ioctl_old_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd), arg);

		goto exit;
	} else if ((_IOC_NR(cmd)) == STD_IOCTL_BEGIN) {
		retval = 1;
		goto exit;
	}

	// if (_IOC_NR(cmd) == STD_CHECK_FRAMES_ID)
	// 	tcm->waiting_frame = 1;

	retval = copy_from_user(&ioc_data,
			(void __user *) arg,
			sizeof(struct syna_ioctl_data));
	if (retval) {
		hbp_err("Fail to copy ioctl_data from user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

	ptr = ioc_data.buf;

	retval = syna_cdev_ioctl_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd),
			(const unsigned char *)ptr,
			ioc_data.buf_size,
			&ioc_data.data_length);
	if (retval < 0)
		goto exit;

	retval = copy_to_user((void __user *) arg,
			&ioc_data,
			sizeof(struct syna_ioctl_data));
	if (retval) {
		hbp_err("Fail to update ioctl_data to user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

exit:
	syna_pal_mutex_unlock(&tcm->extif_mutex);
	// tcm->waiting_frame = 0;
	hbp_info("syna_cdev_ioctls Exit, retval=%d\n", retval);

	return retval;
}

#ifdef USE_COMPAT_IOCTL
/**
 * syna_cdev_compat_ioctls()
 *
 * Used to implements the IOCTL compatible operations
 *
 * @param
 *    [ in] filp: represents the file descriptor
 *    [ in] cmd: command code sent from userspace
 *    [ in] arg: arguments sent from userspace
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static long syna_cdev_compat_ioctls(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;
	struct syna_tcm_ioctl_data_compat ioc_data;
	unsigned char *ptr = NULL;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	syna_pal_mutex_lock(&tcm->extif_mutex);

	retval = 0;

	/* handle the old IOCTLs */
	if ((_IOC_NR(cmd)) < STD_IOCTL_BEGIN) {
		retval = syna_cdev_ioctl_old_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd), arg);

		goto exit;
	} else if ((_IOC_NR(cmd)) == STD_IOCTL_BEGIN) {
		retval = 1;
		goto exit;
	}

	retval = copy_from_user(&ioc_data,
		(struct syna_tcm_ioctl_data_compat __user *) compat_ptr(arg),
		sizeof(struct syna_tcm_ioctl_data_compat));
	if (retval) {
		hbp_err("Fail to copy ioctl_data from user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

	ptr = compat_ptr((unsigned long)ioc_data.buf);

	retval = syna_cdev_ioctl_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd),
			(const unsigned char *)ptr,
			ioc_data.buf_size,
			&ioc_data.data_length);
	if (retval < 0)
		goto exit;

	retval = copy_to_user(compat_ptr(arg),
			&ioc_data,
			sizeof(struct syna_tcm_ioctl_data_compat));
	if (retval) {
		hbp_err("Fail to update ioctl_data to user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

exit:
	syna_pal_mutex_unlock(&tcm->extif_mutex);

	return retval;
}
#endif

/**
 * syna_cdev_llseek()
 *
 * Used to change the current position in a file.
 *
 * @param
 *    [ in] filp:   represents the file descriptor
 *    [ in] off:    the file position
 *    [ in] whence: flag for seeking
 *
 * @return
 *    not support
 */
static loff_t syna_cdev_llseek(struct file *filp,
		loff_t off, int whence)
{
	return -EFAULT;
}
/**
 * syna_cdev_read()
 *
 * Used to read data through the device file.
 * Function will use raw write approach.
 *
 * @param
 *    [ in] filp:  represents the file descriptor
 *    [out] buf:   given buffer from userspace
 *    [ in] count: size of buffer
 *    [ in] f_pos: the file position
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static ssize_t syna_cdev_read(struct file *filp,
		char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	if (g_sysfs_has_remove == 1) {
		hbp_err("%s:driver is remove!!\n", __func__);
		return -EINVAL;
	}
	hbp_debug("syna_cdev_read beign\n");
	IF_ARG_NULL_OUT(g_sysfs_dir);
	p_kobj = g_sysfs_dir->parent;
	IF_ARG_NULL_OUT(p_kobj);
	p_dev = container_of(p_kobj, struct device, kobj);
	IF_ARG_NULL_OUT(p_dev);
	tcm = dev_get_drvdata(p_dev);
	IF_ARG_NULL_OUT(tcm);

	if (count == 0)
		return 0;

	syna_pal_mutex_lock(&tcm->extif_mutex);

	retval = syna_cdev_ioctl_raw_read(tcm,
			(const unsigned char *)buf, count, count);
	if (retval != count) {
		hbp_err("Invalid read operation, request:%d, return:%d\n",
			(unsigned int)count, retval);
	}

	syna_pal_mutex_unlock(&tcm->extif_mutex);
	hbp_debug("syna_cdev_read end\n");

	return retval;
}
/**
 * syna_cdev_write()
 *
 * Used to send data to device through the device file.
 * Function will use raw write approach.
 *
 * @param
 *    [ in] filp:  represents the file descriptor
 *    [ in] buf:   given buffer from userspace
 *    [ in] count: size of buffer
 *    [ in] f_pos: the file position
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static ssize_t syna_cdev_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	if (g_sysfs_has_remove == 1) {
		hbp_err("%s:driver is remove!!\n", __func__);
		return -EINVAL;
	}

	hbp_debug("syna_cdev_write beign\n");
	p_kobj = g_sysfs_dir->parent;
	IF_ARG_NULL_OUT(p_kobj);
	p_dev = container_of(p_kobj, struct device, kobj);
	IF_ARG_NULL_OUT(p_dev);
	tcm = dev_get_drvdata(p_dev);
	IF_ARG_NULL_OUT(tcm);

	if (count == 0)
		return 0;

	syna_pal_mutex_lock(&tcm->extif_mutex);
	retval = syna_cdev_ioctl_raw_write(tcm,
			(const unsigned char *)buf, count, count);
	if (retval != count) {
		hbp_err("Invalid write operation, request:%d, return:%d\n",
			(unsigned int)count, retval);
	}

	syna_pal_mutex_unlock(&tcm->extif_mutex);
	hbp_debug("syna_cdev_write end\n");

	return retval;
}
/**
 * syna_cdev_open()
 *
 * Invoked when the device file is being open, which should be
 * always the first operation performed on the device file
 *
 * @param
 *    [ in] inp:  represents a file in rootfs
 *    [ in] filp: represents the file descriptor
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_open(struct inode *inp, struct file *filp)
{
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	if (g_sysfs_has_remove == 1) {
		hbp_err("%s:driver is remove!!\n", __func__);
		return -EINVAL;
	}
	p_kobj = g_sysfs_dir->parent;
	IF_ARG_NULL_OUT(p_kobj);
	p_dev = container_of(p_kobj, struct device, kobj);
	IF_ARG_NULL_OUT(p_dev);
	tcm = dev_get_drvdata(p_dev);
	IF_ARG_NULL_OUT(tcm);

	syna_pal_mutex_lock(&tcm->extif_mutex);

	if (tcm->char_dev_ref_count != 0) {
		LOGN("cdev already open, %d\n",
			tcm->char_dev_ref_count);
		syna_pal_mutex_unlock(&tcm->extif_mutex);
		return -EBUSY;
	}

	tcm->char_dev_ref_count++;

	g_sysfs_io_polling_interval = 0;

	g_sysfs_extra_bytes_read = 0;

	syna_pal_mutex_unlock(&tcm->extif_mutex);

	hbp_info("cdev open\n");

	return 0;
}
/**
 * syna_cdev_release()
 *
 * Invoked when the device file is being released
 *
 * @param
 *    [ in] inp:  represents a file in rootfs
 *    [ in] filp: represents the file descriptor
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_release(struct inode *inp, struct file *filp)
{
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	if (g_sysfs_has_remove == 1) {
		hbp_err("%s:driver is remove!!\n", __func__);
		return -EINVAL;
	}

	IF_ARG_NULL_OUT(g_sysfs_dir);

	p_kobj = g_sysfs_dir->parent;
	IF_ARG_NULL_OUT(p_kobj);
	p_dev = container_of(p_kobj, struct device, kobj);
	IF_ARG_NULL_OUT(p_dev);
	tcm = dev_get_drvdata(p_dev);
	IF_ARG_NULL_OUT(tcm);

	mutex_lock(&tcm->mutex);
	syna_pal_mutex_lock(&tcm->extif_mutex);

	if (tcm->char_dev_ref_count <= 0) {
		LOGN("cdev already closed, %d\n",
			tcm->char_dev_ref_count);
		syna_pal_mutex_unlock(&tcm->extif_mutex);
		mutex_unlock(&tcm->mutex);
		return 0;
	}

	tcm->char_dev_ref_count--;

	tcm->is_attn_redirecting = false;
	syna_pal_mutex_unlock(&tcm->extif_mutex);
	mutex_unlock(&tcm->mutex);

	g_sysfs_io_polling_interval = 0;

	g_sysfs_extra_bytes_read = 0;

	hbp_info("cdev close\n");
	return 0;
}

/**
 * Declare the operations of TouchCom device file
 */
static const struct file_operations device_fops = {
	.owner = THIS_MODULE,
#ifdef USE_UNLOCKED_IOCTL
	.unlocked_ioctl = syna_cdev_ioctls,
#ifdef USE_COMPAT_IOCTL
	.compat_ioctl = syna_cdev_compat_ioctls,
#endif
#else
	.ioctl = syna_cdev_ioctls,
#endif
	.llseek = syna_cdev_llseek,
	.read = syna_cdev_read,
	.write = syna_cdev_write,
	.open = syna_cdev_open,
	.release = syna_cdev_release,
};
/**
 * syna_cdev_devnode()
 *
 * Provide the declaration of devtmpfs
 *
 * @param
 *    [ in] dev:  an instance of device
 *    [ in] mode: mode of created node
 *
 * @return
 *    the string of devtmpfs
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static char *syna_cdev_devnode(const struct device *dev, umode_t *mode)
#else
static char *syna_cdev_devnode(struct device *dev, umode_t *mode)
#endif
{
	if (!mode)
		return NULL;

	/* S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH */
	*mode = CHAR_DEVICE_MODE;

	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}
/**
 * syna_cdev_create_sysfs()
 *
 * Create a device node and register it with sysfs.
 *
 * @param
 *    [ in] tcm: the driver handle
 *    [ in] pdev: an instance of platform device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_cdev_create_sysfs(struct syna_tcm *tcm,
		struct platform_device *pdev)
{
	int retval = 0;
	struct class *device_class = NULL;
	struct device *device = NULL;
	static int cdev_major_num;

	tcm->device_class = NULL;
	tcm->device = NULL;

	tcm->is_attn_redirecting = false;

	syna_pal_mutex_alloc(&tcm->mutex);
	syna_pal_mutex_alloc(&tcm->extif_mutex);
	syna_tcm_buf_init(&g_cdev_cbuf);

	if (cdev_major_num) {
		tcm->char_dev_num = MKDEV(cdev_major_num, 0);
		retval = register_chrdev_region(tcm->char_dev_num, 1,
				PLATFORM_DRIVER_NAME);
		if (retval < 0) {
			hbp_err("Fail to register char device\n");
			goto err_register_chrdev_region;
		}
	} else {
		retval = alloc_chrdev_region(&tcm->char_dev_num, 0, 1,
				PLATFORM_DRIVER_NAME);
		if (retval < 0) {
			hbp_err("Fail to allocate char device\n");
			goto err_alloc_chrdev_region;
		}

		cdev_major_num = MAJOR(tcm->char_dev_num);
	}

	cdev_init(&tcm->char_dev, &device_fops);
	tcm->char_dev.owner = THIS_MODULE;

	retval = cdev_add(&tcm->char_dev, tcm->char_dev_num, 1);
	if (retval < 0) {
		hbp_err("Fail to add cdev_add\n");
		goto err_add_chardev;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
	device_class = class_create(THIS_MODULE, PLATFORM_DRIVER_NAME);
#else
	device_class = class_create(PLATFORM_DRIVER_NAME);
#endif
	if (IS_ERR(device_class)) {
		hbp_err("Fail to create device class\n");
		retval = PTR_ERR(device_class);
		goto err_create_class;
	}

	device_class->devnode = syna_cdev_devnode;

	device = device_create(device_class, NULL,
			tcm->char_dev_num, NULL,
			CHAR_DEVICE_NAME"%d", MINOR(tcm->char_dev_num));
	if (IS_ERR(tcm->device)) {
		hbp_err("Fail to create character device\n");
		retval = -ENOENT;
		goto err_create_device;
	}

	tcm->device_class = device_class;

	tcm->device = device;

	tcm->char_dev_ref_count = 0;
	tcm->proc_pid = 0;

	g_sysfs_extra_bytes_read = 0;

	retval = syna_sysfs_create_dir(tcm, pdev);
	if (retval < 0) {
		hbp_err("Fail to create sysfs dir\n");
		retval = -ENOTDIR;
		goto err_create_dir;
	}

	g_sysfs_has_remove = 0;
	return 0;

err_create_dir:
	device_destroy(device_class, tcm->char_dev_num);
err_create_device:
	class_destroy(device_class);
err_create_class:
	cdev_del(&tcm->char_dev);
err_add_chardev:
	unregister_chrdev_region(tcm->char_dev_num, 1);
err_alloc_chrdev_region:
err_register_chrdev_region:
	return retval;
}
