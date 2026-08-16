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
 * @file synaptics_touchcom_func_base.c
 *
 * This file implements generic and foundational functions supported in
 * Synaptics TouchComm communication protocol.
 *
 * The declarations are in synaptics_touchcom_func_base.h.
 */

#include "synaptics_touchcom_func_base.h"
#include "synaptics_touchcom_func_touch.h"
#include"synaptics_touchcom_core_dev.h"
#include "../../../../hbp_spi.h"
#include "../../../../hbp_core.h"

static unsigned char *rx_buf;
static unsigned char *tx_buf;

static unsigned int buf_size;

/**
 * @brief FOR TUNE DEBUG LOG
 * 0 -- disable (default)
 * 1 -- read all the report and response
 * 2 -- don't print the report except identify report
 * 3 -- don't print the report (>REPORT_TOUCH)
 */
// int tune_debug_enable = 1;//FOR_LOG_DEBUG
// module_param_named(tune_debug_enable, tune_debug_enable, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define SYNA_SPI_TRANSFER_READ 0
#define SYNA_SPI_TRANSFER_WRITE 1
#define SYNA_SPI_PRINT_BUF_SIZE 256
#define SYNA_SPI_PRINT_BUF_LEFT_SIZE 5
#define SYNA_TCM_MESSAGE_MARKER 0xa5
// static void syna_print_xfer_data(unsigned char *data, unsigned int length, unsigned int wr_rd)
// {
// 	static unsigned char last_code = 0;
// 	unsigned char code = 0;
// 	char print_buf[SYNA_SPI_PRINT_BUF_SIZE] = {0};
// 	int i, cnt, offset, left;

// 	if(!tune_debug_enable)
// 		return;

// 	if (!data)
// 		goto exit;

// 	if ((wr_rd == SYNA_SPI_TRANSFER_READ) && (data[0] == SYNA_TCM_MESSAGE_MARKER)) {
// 		code = data[1];
// 	}

// 	if(tune_debug_enable == 1) {
// 		// print all the report and response
// 		// print all the write data
// 	} else if (tune_debug_enable == 2) {
// 		/* skip all the report data except identify report */
// 		if (((code == STATUS_CONTINUED_READ) && (last_code > REPORT_IDENTIFY) ) || (code > REPORT_IDENTIFY))
// 			goto exit;
// 	} else if (tune_debug_enable == 3) {
// 		/* skip all the report (> REPORT_TOUCH) */
// 		if (((code == STATUS_CONTINUED_READ) && (last_code > REPORT_TOUCH) ) || (code > REPORT_TOUCH))
// 			goto exit;
// 	} else {
// 		goto exit;
// 	}

// 	//hbp_info("%s:%*ph\n", (wr_rd == SYNA_SPI_TRANSFER_READ) ? "RD" : "WR", length, data);

// 	offset = 0;
// 	cnt = snprintf(print_buf + offset, SYNA_SPI_PRINT_BUF_SIZE - offset, "%s ",
// 					(wr_rd == SYNA_SPI_TRANSFER_READ) ? "RD:" : "WR:");
// 	offset += cnt;
// 	for (i = 0; i < length; i++) {
// 		left = SYNA_SPI_PRINT_BUF_SIZE - offset;
// 		if (left <= SYNA_SPI_PRINT_BUF_LEFT_SIZE) {
// 			//hbp_info("There is unprinted data\n");
// 			break;
// 		}
// 		cnt = snprintf(print_buf + offset, SYNA_SPI_PRINT_BUF_SIZE - offset, "%02x ", data[i]);
// 		offset += cnt;
// 	}
// 	cnt = snprintf(print_buf + offset, SYNA_SPI_PRINT_BUF_SIZE - offset, "\n");
// 	offset += cnt;
// 	hbp_info("%s", print_buf);

// exit:
// 	last_code = code;
// 	return;
// }
/**
 * syna_tcm_spi_alloc_mem()
 *
 * Manage and allocate the memory to buf being as a temporary buffer for IO
 *
 * @param
 *    [ in] count: number of spi_transfer structures to send
 *    [ in] size:  size of temporary buffer
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_spi_alloc_mem(unsigned int count, unsigned int size)
{
	/*static unsigned int xfer_count;

	if (count > xfer_count) {
		syna_pal_mem_free((void *)xfer);
		xfer = syna_pal_mem_alloc(count, sizeof(*xfer));
		if (!xfer) {
			hbp_err("Fail to allocate memory for xfer\n");
			xfer_count = 0;
			return -ENOMEM;
		}
		xfer_count = count;
	} else {
		syna_pal_mem_set(xfer, 0, count * sizeof(*xfer));
	}*/

	if (size > buf_size) {
		if (rx_buf) {
			syna_pal_mem_free((void *)rx_buf);
			rx_buf = NULL;
		}
		if (tx_buf) {
			syna_pal_mem_free((void *)tx_buf);
			tx_buf = NULL;
		}

		rx_buf = syna_pal_mem_alloc(size, sizeof(unsigned char));
		if (!rx_buf) {
			hbp_err("Fail to allocate memory for rx_buf\n");
			buf_size = 0;
			return -ENOMEM;
		}
		tx_buf = syna_pal_mem_alloc(size, sizeof(unsigned char));
		if (!tx_buf) {
			hbp_err("Fail to allocate memory for tx_buf\n");
			buf_size = 0;
			return -ENOMEM;
		}

		buf_size = size;
	}
	memset(rx_buf, 0x0, buf_size);
	memset(tx_buf, 0x0, buf_size);

	return 0;
}
/**
 * syna_spi_read()
 *
 * TouchCom over SPI requires the host to assert the SSB signal to address
 * the device and retrieve the data.
 *
 * @param
 *    [ in] hw_if:   the handle of hw interface
 *    [out] rd_data: buffer for storing data retrieved from device
 *    [ in] rd_len: number of bytes retrieved from device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_tcm_read(struct tcm_dev *tcm_dev,
	unsigned char *rd_data, unsigned int rd_len)
{
	int retval;
	// unsigned int idx;
	struct spi_message msg;
	char payload[64] = {0};
	//struct spi_device *spi = hw_if->pdev;
	//struct syna_hw_bus_data *bus = &hw_if->bdata_io;

	// if (!spi) {
	// 	hbp_err("Invalid bus io device\n");
	// 	return -ENXIO;
	// }

	if (!tcm_dev) {
		hbp_err("Invalid tcm_dev\n");
		return -ENXIO;
	}

	if (!tcm_dev->bus_ops) {
		hbp_err("Invalid tcm_dev->bus_ops\n");
		return -ENXIO;
	}

	if (!tcm_dev->bus_ops->spi_sync) {
		hbp_err("Invalid tcm_dev->bus_ops->spi_sync\n");
		return -ENXIO;
	}

	LOGD("%s start\n", __func__);
	syna_pal_mutex_lock(&tcm_dev->io_mutex);

	spi_message_init(&msg);

	retval = syna_spi_alloc_mem(1, rd_len);
	if (retval < 0) {
		hbp_err("Fail to allocate memory\n");
		goto exit;
	}
/*
	if (bus->spi_byte_delay_us == 0) {
*/
		syna_pal_mem_set(tx_buf, 0xff, rd_len);
/*
		xfer[0].len = rd_len;
		xfer[0].tx_buf = tx_buf;
		xfer[0].rx_buf = rx_buf;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
		if (bus->spi_block_delay_us)
			xfer[0].delay_usecs = bus->spi_block_delay_us;
#endif
		spi_message_add_tail(&xfer[0], &msg);
	} else {
		tx_buf[0] = 0xff;
		for (idx = 0; idx < rd_len; idx++) {
			xfer[idx].len = 1;
			xfer[idx].tx_buf = tx_buf;
			xfer[idx].rx_buf = &rx_buf[idx];
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
			xfer[idx].delay_usecs = bus->spi_byte_delay_us;
			if (bus->spi_block_delay_us && (idx == rd_len - 1))
				xfer[idx].delay_usecs = bus->spi_block_delay_us;
#endif
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}
*/
	retval = tcm_dev->bus_ops->spi_sync(tcm_dev->bus_ops, tx_buf, rx_buf, rd_len);
	if (retval != 0) {
		hbp_err("Failed to complete SPI transfer, error = %d\n", retval);
		if (retval == -5) {
			tcm_dev->tcm_spi_read_err_cnt++;
			snprintf(payload, sizeof(payload), "syna_tcm_read_error cnt %d\n", tcm_dev->tcm_spi_read_err_cnt);
			hbp_exception_report(EXCEP_BUS, payload, sizeof(payload));
		}
		if (tcm_dev->tcm_spi_read_err_cnt > 3) {
			hbp_exception_report(EXCEP_BUS, "tcm_spi_read_err_cnt_over_3times", sizeof("tcm_spi_read_err_cnt_over_3times"));
		}
		goto exit;
	}
	retval = syna_pal_mem_cpy(rd_data, rd_len, rx_buf, rd_len, rd_len);
	if (retval < 0) {
		hbp_err("Fail to copy rx_buf to rd_data\n");
		goto exit;
	}

	// syna_print_xfer_data(rd_data, rd_len, SYNA_SPI_TRANSFER_READ);
	LOGD("%s:%*ph\n", "RD", rd_len, rd_data);

	retval = rd_len;

exit:
	syna_pal_mutex_unlock(&tcm_dev->io_mutex);
	LOGD("%s end\n", __func__);

	return retval;
}

/**
 * syna_spi_write()
 *
 * TouchCom over SPI requires the host to assert the SSB signal to address
 * the device and send the data to the device.
 *
 * @param
 *    [ in] hw_if:   the handle of hw interface
 *    [ in] wr_data: written data
 *    [ in] wr_len: length of written data in bytes
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_tcm_write(struct tcm_dev *tcm_dev,
	unsigned char *wr_data, unsigned int wr_len)
{
	int retval;
	// unsigned int idx;
	struct spi_message msg;
	//struct spi_device *spi = hw_if->pdev;
	//struct syna_hw_bus_data *bus = &hw_if->bdata_io;

	// if (!spi) {
	// 	hbp_err("Invalid bus io device\n");
	// 	return -ENXIO;
	// }

	syna_pal_mutex_lock(&tcm_dev->io_mutex);

	spi_message_init(&msg);

	retval = syna_spi_alloc_mem(1, wr_len);

	if (retval < 0) {
		hbp_err("Failed to allocate memory\n");
		goto exit;
	}

	retval = syna_pal_mem_cpy(tx_buf, wr_len, wr_data, wr_len, wr_len);
	if (retval < 0) {
		hbp_err("Fail to copy wr_data to tx_buf\n");
		goto exit;
	}
/*
	if (bus->spi_byte_delay_us == 0) {
		xfer[0].len = wr_len;
		xfer[0].tx_buf = tx_buf;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
		if (bus->spi_block_delay_us)
			xfer[0].delay_usecs = bus->spi_block_delay_us;
#endif
		spi_message_add_tail(&xfer[0], &msg);
	} else {
		for (idx = 0; idx < wr_len; idx++) {
			xfer[idx].len = 1;
			xfer[idx].tx_buf = &tx_buf[idx];
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
			xfer[idx].delay_usecs = bus->spi_byte_delay_us;
			if (bus->spi_block_delay_us && (idx == wr_len - 1))
				xfer[idx].delay_usecs = bus->spi_block_delay_us;
#endif
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}
*/
	retval = tcm_dev->bus_ops->spi_sync(tcm_dev->bus_ops, tx_buf, rx_buf, wr_len);
	if (retval != 0) {
		hbp_err("Fail to complete SPI transfer, error = %d\n", retval);
		goto exit;
	}

	// syna_print_xfer_data(wr_data, wr_len, SYNA_SPI_TRANSFER_WRITE);
	LOGD("%s:%*ph\n", "WR", wr_len, wr_data);

	retval = wr_len;

exit:
	syna_pal_mutex_unlock(&tcm_dev->io_mutex);

	return retval;
}

/**
 * syna_tcm_init_message_wrap()
 *
 * Initialize internal buffers and related structures for command processing.
 * The function must be called to prepare all essential structures for
 * command wrapper.
 *
 * @param
 *    [in] tcm_msg: message wrapper structure
 *    [in] resp_reading: default method to retrieve the resp data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_init_message_wrap(struct tcm_message_data_blob *tcm_msg,
		unsigned int resp_reading)
{
	/* initialize internal buffers */
	syna_tcm_buf_init(&tcm_msg->in);
	syna_tcm_buf_init(&tcm_msg->out);
	syna_tcm_buf_init(&tcm_msg->temp);

	/* allocate the completion event for command processing */
	if (syna_pal_completion_alloc(&tcm_msg->cmd_completion) < 0) {
		hbp_err("Fail to allocate cmd completion event\n");
		return _EINVAL;
	}

	/* allocate the cmd_mutex for command protection */
	if (syna_pal_mutex_alloc(&tcm_msg->cmd_mutex) < 0) {
		hbp_err("Fail to allocate cmd_mutex\n");
		return _EINVAL;
	}

	/* allocate the rw_mutex for rw protection */
	if (syna_pal_mutex_alloc(&tcm_msg->rw_mutex) < 0) {
		hbp_err("Fail to allocate rw_mutex\n");
		return _EINVAL;
	}

	/* set default state of command_status  */
	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
	tcm_msg->command = CMD_NONE;
	tcm_msg->status_report_code = STATUS_IDLE;
	tcm_msg->payload_length = 0;
	tcm_msg->response_code = 0;
	tcm_msg->report_code = 0;
	tcm_msg->seq_toggle = 0;

	/* allocate the internal buffer.in at first */
	syna_tcm_buf_lock(&tcm_msg->in);

	if (syna_tcm_buf_alloc(&tcm_msg->in, MESSAGE_HEADER_SIZE) < 0) {
		hbp_err("Fail to allocate memory for buf.in (size = %d)\n",
			MESSAGE_HEADER_SIZE);
		tcm_msg->in.buf_size = 0;
		tcm_msg->in.data_length = 0;
		syna_tcm_buf_unlock(&tcm_msg->in);
		return _EINVAL;
	}
	tcm_msg->in.buf_size = MESSAGE_HEADER_SIZE;

	syna_tcm_buf_unlock(&tcm_msg->in);

	tcm_msg->default_resp_reading = resp_reading;

	hbp_info("Resp. reading method (default): %s\n",
		(resp_reading == RESP_IN_ATTN) ? "attn" : "polling");

	/* initialize the features of message handling */
	tcm_msg->predict_reads = false;
	tcm_msg->predict_length = 0;
	tcm_msg->has_crc = false;
	tcm_msg->crc_bytes = 0;
	tcm_msg->has_extra_rc = false;
	tcm_msg->rc_byte = 0;

	return 0;
}

/**
 * syna_tcm_allocate_device()
 *
 * Create the TouchCom core device handle.
 * This function must be called in order to allocate the main device handle,
 * structure syna_tcm_dev, which will be passed to all other operations and
 * functions within the entire source code.
 *
 * Meanwhile, caller has to prepare specific syna_tcm_hw_interface structure,
 * so that all the implemented functions can access hardware components
 * through syna_tcm_hw_interface.
 *
 * @param
 *    [out] ptcm_dev_ptr: a pointer to the device handle returned
 *    [ in] resp_reading: default resp reading method
 *                        set 'RESP_IN_ATTN' to apply ATTN-driven method;
 *                        set 'RESP_IN_POLLING' to read in resp by polling
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_allocate_device(struct tcm_dev **ptcm_dev_ptr, unsigned int resp_reading)
{
	int retval = 0;
	struct tcm_dev *tcm_dev = NULL;

	*ptcm_dev_ptr = NULL;

	/* allocate the core device handle */
	tcm_dev = (struct tcm_dev *)syna_pal_mem_alloc(
			1,
			sizeof(struct tcm_dev));
	if (!tcm_dev) {
		hbp_err("Fail to create tcm device handle\n");
		return _ENOMEM;
	}

	/* initialize the default setup */
	tcm_dev->max_rd_size = RD_CHUNK_SIZE;
	tcm_dev->max_wr_size = WR_CHUNK_SIZE;

	tcm_dev->write_message = NULL;
	tcm_dev->read_message = NULL;

	tcm_dev->cb_custom_touch_entity = NULL;
	tcm_dev->cbdata_touch_entity = NULL;
	tcm_dev->cb_custom_gesture = NULL;
	tcm_dev->cbdata_gesture = NULL;
	tcm_dev->cb_reset_occurrence = NULL;
	tcm_dev->cbdata_reset = NULL;

	tcm_dev->dev_mode = MODE_UNKNOWN;

	tcm_dev->tcm_spi_read_err_cnt = 0;

	/* allocate internal buffers */
	syna_tcm_buf_init(&tcm_dev->report_buf);
	syna_tcm_buf_init(&tcm_dev->resp_buf);
	syna_tcm_buf_init(&tcm_dev->external_buf);
	syna_tcm_buf_init(&tcm_dev->touch_config);

	syna_pal_mutex_alloc(&tcm_dev->io_mutex);

	/* initialize the command wrapper interface */
	retval = syna_tcm_init_message_wrap(&tcm_dev->msg_data,
			resp_reading);
	if (retval < 0) {
		hbp_err("Fail to initialize command interface\n");
		goto err_init_message_wrap;
	}

	/* return the created device handle */
	*ptcm_dev_ptr = tcm_dev;

	hbp_info("TouchComm core module created, ver.: %d.%02d\n",
		(unsigned char)(SYNA_TCM_CORE_LIB_VERSION >> 8),
		(unsigned char)SYNA_TCM_CORE_LIB_VERSION & 0xff);

	hbp_info("Capability: wr_chunk(%d), rd_chunk(%d)\n",
		tcm_dev->max_wr_size, tcm_dev->max_rd_size);

	return 0;

err_init_message_wrap:
	syna_tcm_buf_release(&tcm_dev->touch_config);
	syna_tcm_buf_release(&tcm_dev->external_buf);
	syna_tcm_buf_release(&tcm_dev->report_buf);
	syna_tcm_buf_release(&tcm_dev->resp_buf);

	syna_pal_mutex_free(&tcm_dev->io_mutex);

	syna_pal_mem_free((void *)tcm_dev);

	return retval;
}

/**
 * syna_tcm_detect_protocol()
 *
 * Helper to distinguish which TouchCom firmware is running.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] data:     raw data from device
 *    [ in] data_len: length of input data in bytes
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_detect_protocol(struct tcm_dev *tcm_dev,
		unsigned char *data, unsigned int data_len)
{
	int retval;

	hbp_info("%s is called.\n", __func__);
	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	retval = syna_tcm_v1_detect(tcm_dev, data, data_len);

	return retval;
}

/**
 * syna_tcm_detect_device()
 *
 * Determine the type of device being connected, and distinguish which
 * version of TouchCom firmware running on the device.
 * This function must be called before using this TouchComm core library.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    on success, the current mode running on the device is returned;
 *    otherwise, negative value on error.
 */
int syna_tcm_detect_device(struct tcm_dev *tcm_dev)
{
	int retval = 0;
	unsigned char data[4] = { 0 };

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	tcm_dev->dev_mode = MODE_UNKNOWN;

	/* get the bare data from the bus directly */
	data[0] = 0x07;
	retval = syna_tcm_write(tcm_dev, &data[0], 1);
	if (retval < 0) {
		hbp_err("Fail to write magic to bus\n");
		return _EIO;
	}

	retval = syna_tcm_read(tcm_dev,
			data, (unsigned int)sizeof(data));
	if (retval < 0) {
		hbp_err("Fail to retrieve 4-byte data from bus\n");
		return _EIO;
	}

	hbp_info("bare data: %02x %02x %02x %02x\n",
			data[0], data[1], data[2], data[3]);

	/* distinguish which tcm version running on the device */
	retval = syna_tcm_detect_protocol(tcm_dev,
			data, (unsigned int)sizeof(data));
	if (retval < 0) {
		hbp_err("Fail to detect TouchCom device, %02x %02x %02x %02x\n",
			data[0], data[1], data[2], data[3]);
		return retval;
	}

	if ((!tcm_dev->write_message) || (!tcm_dev->read_message)) {
		hbp_err("Invalid TouchCom rw operations\n");
		return _ENODEV;
	}

	hbp_info("tcm_dev->dev_mode = 0x%x\n", tcm_dev->dev_mode);

	/* check the running mode */
	switch (tcm_dev->dev_mode) {
	case MODE_APPLICATION_FIRMWARE:
		hbp_info("Device in Application FW, build id: %d, %s\n",
			tcm_dev->packrat_number,
			tcm_dev->id_info.part_number);
		retval = syna_tcm_get_app_info(tcm_dev, &tcm_dev->app_info);
		if (retval < 0) {
			hbp_err("Fail to get touch app info.\n");
			return _EAGAIN;
		}
		retval = syna_tcm_preserve_touch_report_config(tcm_dev);
		if (retval < 0) {
			hbp_err("Fail to preserve touch report config\n");
			return _EAGAIN;
		}
		break;
	case MODE_BOOTLOADER:
	case MODE_TDDI_BOOTLOADER:
		hbp_info("Device in Bootloader\n");
		break;
	case MODE_ROMBOOTLOADER:
		hbp_info("Device in ROMBoot uBL\n");
		break;
	case MODE_MULTICHIP_TDDI_BOOTLOADER:
		hbp_info("Device in multi-chip TDDI Bootloader\n");
		break;
	default:
		LOGW("Found TouchCom device, but unsupported mode: 0x%02x\n",
			tcm_dev->dev_mode);
		break;
	}
	retval = tcm_dev->dev_mode;
	return retval;
}

/**
 * syna_tcm_get_event_data()
 *
 * Helper to read TouchComm messages when ATTN signal is asserted.
 * After returning, the ATTN signal should be no longer asserted.
 *
 * The 'code' returned will guide the caller on the next action.
 * For example, do touch reporting once returned code is equal to REPORT_TOUCH.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [out] code:    received report code
 *    [out] data:    a user buffer for data returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_event_data(struct tcm_dev *tcm_dev,
		unsigned char *code, struct tcm_buffer *data)
{
	int retval = 0;

	if (!tcm_dev || !tcm_dev->read_message) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!code) {
		hbp_err("Invalid parameter\n");
		return _EINVAL;
	}

	LOGD("%s start\n", __func__);
	/* retrieve the event data */
	retval = tcm_dev->read_message(tcm_dev,
			code);
	if (retval < 0) {
		hbp_err("Fail to read messages\n");
		return retval;
	}

	/* exit if no buffer provided */
	if (!data)
		goto exit;

	syna_tcm_buf_lock(data);
	syna_tcm_buf_clear(data);
	syna_tcm_buf_unlock(data);
	/* if gathering a report, copy to the user buffer */
	if ((*code >= REPORT_IDENTIFY) && (*code != STATUS_INVALID)) {
		if (tcm_dev->report_buf.data_length == 0)
			goto exit;

		syna_tcm_buf_lock(&tcm_dev->report_buf);

		retval = syna_tcm_buf_copy(data, &tcm_dev->report_buf);
		if (retval < 0) {
			hbp_err("Fail to copy data, report type: %x\n", *code);
			syna_tcm_buf_unlock(&tcm_dev->report_buf);
			goto exit;
		}

		syna_tcm_buf_unlock(&tcm_dev->report_buf);
	}

	/* if gathering a response, copy to the user buffer */
	if ((*code > STATUS_IDLE) && (*code <= STATUS_ERROR)) {
		if (tcm_dev->resp_buf.data_length == 0)
			goto exit;

		syna_tcm_buf_lock(&tcm_dev->resp_buf);

		retval = syna_tcm_buf_copy(data, &tcm_dev->resp_buf);
		if (retval < 0) {
			hbp_err("Fail to copy data, status code: %x\n", *code);
			syna_tcm_buf_unlock(&tcm_dev->resp_buf);
			goto exit;
		}

		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
	}

exit:
	LOGD("%s end\n", __func__);
	return retval;
}

/**
 * syna_tcm_identify()
 *
 * Implement the standard command code, which is used to request
 * an IDENTIFY report packet.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [out] id_info: the identification info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_identify(struct tcm_dev *tcm_dev,
		struct tcm_identification_info *id_info)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_IDENTIFY,
			NULL,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		hbp_err("Fail to send command 0x%02x\n", CMD_IDENTIFY);
		goto exit;
	}

	if (id_info == NULL)
		goto show_info;

	/* copy identify info to caller */
	retval = syna_pal_mem_cpy((unsigned char *)id_info,
			sizeof(struct tcm_identification_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			MIN(sizeof(*id_info), tcm_dev->resp_buf.data_length));
	if (retval < 0) {
		hbp_err("Fail to copy identify info to caller\n");
		goto exit;
	}

show_info:
	hbp_info("TCM Fw mode: 0x%02x, TCM ver.: %d\n",
		tcm_dev->id_info.mode, tcm_dev->id_info.version);

	tcm_dev->dev_mode = tcm_dev->id_info.mode;

exit:
	return retval;
}

/**
 * syna_tcm_reset()
 *
 * Implement the standard command code, which is used to perform a sw reset
 * immediately. After a successful reset, an IDENTIFY report to indicate that
 * device is ready.
 *
 * Caller shall be aware that the firmware will be reloaded after reset.
 * Therefore, if expecting that a different firmware version is loaded, please
 * do app firmware setup after reset.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_reset(struct tcm_dev *tcm_dev)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int board_setting = 0;
	unsigned int resp_handling = 0;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	resp_handling = tcm_dev->msg_data.default_resp_reading;

	/* gather the board settings of reset delay time */
	if (board_setting == 0)
		board_setting = RESET_DELAY_MS;

	/* select the proper period to handle the resp of reset */
	if (resp_handling != RESP_IN_ATTN) {
		if (board_setting > resp_handling) {
			resp_handling = board_setting;
			hbp_info("Use board settings %dms to poll resp of reset\n",
				resp_handling);
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_RESET,
			NULL,
			0,
			&resp_code,
			resp_handling);
	if (retval < 0) {
		hbp_err("Fail to send command 0x%02x\n", CMD_RESET);
		goto exit;
	}

	/* current device mode is expected to be updated
	 * because identification report will be received after reset
	 */
	tcm_dev->dev_mode = tcm_dev->id_info.mode;
	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		hbp_info("Device mode 0x%02X running after reset\n",
			tcm_dev->dev_mode);
	}

	retval = 0;
exit:
	return retval;
}

/**
 * syna_tcm_enable_report()
 *
 * Implement the application fw command code to enable or disable a report.
 *
 * @param
 *    [ in] tcm_dev:     the device handle
 *    [ in] report_code: the requested report code being generated
 *    [ in] en:          '1' for enabling; '0' for disabling
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_enable_report(struct tcm_dev *tcm_dev,
		unsigned char report_code, bool en)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned char command;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		hbp_err("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return _EINVAL;
	}

	command = (en) ? CMD_ENABLE_REPORT : CMD_DISABLE_REPORT;

	retval = tcm_dev->write_message(tcm_dev,
			command,
			&report_code,
			1,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		hbp_err("Fail to send command 0x%02x to %s 0x%02x report\n",
			command, (en)?"enable":"disable", report_code);
		goto exit;
	}

	if (resp_code != STATUS_OK) {
		hbp_err("Fail to %s 0x%02x report, resp_code:%x\n",
			(en) ? "enable" : "disable", report_code, resp_code);
	} else {
		hbp_info("Report 0x%x %s\n", report_code,
			(en) ? "enabled" : "disabled");
	}

exit:
	return retval;
}

/**
 * syna_tcm_run_rom_bootloader_fw()
 *
 * Requests that the rombootloader firmware be run.
 * Once the rombootloader firmware has finished starting, an IDENTIFY report
 * to indicate that it is in the new mode.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_run_rom_bootloader_fw(struct tcm_dev *tcm_dev,
		unsigned int fw_switch_delay)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_REBOOT_TO_ROM_BOOTLOADER,
			NULL,
			0,
			&resp_code,
			fw_switch_delay);
	if (retval < 0) {
		hbp_err("Fail to send command 0x%02x\n",
			CMD_REBOOT_TO_ROM_BOOTLOADER);
		goto exit;
	}

	if (!IS_ROM_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		hbp_err("Fail to enter rom bootloader, mode: %x\n",
			tcm_dev->dev_mode);
		retval = _ENODEV;
		goto exit;
	}

	hbp_info("ROM Bootloader (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}

/**
 * syna_tcm_run_bootloader_fw()
 *
 * Requests that the bootloader firmware be run.
 * Once the bootloader firmware has finished starting, an IDENTIFY report
 * to indicate that it is in the new mode.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_run_bootloader_fw(struct tcm_dev *tcm_dev,
		unsigned int fw_switch_delay)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_RUN_BOOTLOADER_FIRMWARE,
			NULL,
			0,
			&resp_code,
			fw_switch_delay);
	if (retval < 0) {
		hbp_err("Fail to send command 0x%02x\n",
			CMD_RUN_BOOTLOADER_FIRMWARE);
		goto exit;
	}

	if (!IS_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		hbp_err("Fail to enter bootloader, mode: %x\n",
			tcm_dev->dev_mode);
		retval = _ENODEV;
		goto exit;
	}

	hbp_info("Bootloader Firmware (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}

/**
 * syna_tcm_run_application_fw()
 *
 * Requests that the application firmware be run.
 * Once the application firmware has finished starting, an IDENTIFY report
 * to indicate that it is in the new mode.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_run_application_fw(struct tcm_dev *tcm_dev,
		unsigned int fw_switch_delay)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_RUN_APPLICATION_FIRMWARE,
			NULL,
			0,
			&resp_code,
			fw_switch_delay);
	if (retval < 0) {
		hbp_err("Fail to send command 0x%02x\n",
			CMD_RUN_APPLICATION_FIRMWARE);
		goto exit;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGW("Fail to enter application fw, mode: %x\n",
			tcm_dev->dev_mode);
		retval = _ENODEV;
		goto exit;
	}

	hbp_info("Application Firmware (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}

/**
 * syna_tcm_switch_fw_mode()
 *
 * Implement the command code to switch the firmware mode.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] mode:    target firmware mode
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_switch_fw_mode(struct tcm_dev *tcm_dev,
		unsigned char mode, unsigned int fw_switch_delay)
{
	int retval = 0;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	switch (mode) {
	case MODE_APPLICATION_FIRMWARE:
		retval = syna_tcm_run_application_fw(tcm_dev,
				fw_switch_delay);
		if (retval < 0) {
			hbp_err("Fail to switch to application mode\n");
			goto exit;
		}
		break;
	case MODE_BOOTLOADER:
	case MODE_TDDI_BOOTLOADER:
	case MODE_TDDI_HDL_BOOTLOADER:
	case MODE_MULTICHIP_TDDI_BOOTLOADER:
		retval = syna_tcm_run_bootloader_fw(tcm_dev,
				fw_switch_delay);
		if (retval < 0) {
			hbp_err("Fail to switch to bootloader mode\n");
			goto exit;
		}
		break;
	case MODE_ROMBOOTLOADER:
		retval = syna_tcm_run_rom_bootloader_fw(tcm_dev,
				fw_switch_delay);
		if (retval < 0) {
			hbp_err("Fail to switch to rom bootloader mode\n");
			goto exit;
		}
		break;
	default:
		hbp_err("Invalid firmware mode requested\n");
		retval = _EINVAL;
		goto exit;
	}

	retval = 0;

exit:
	return retval;
}

/**
 * syna_tcm_get_boot_info()
 *
 * Implement the bootloader command code, which is used to request a
 * boot information packet.
 *
 * @param
 *    [ in] tcm_dev:   the device handle
 *    [out] boot_info: the boot info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_boot_info(struct tcm_dev *tcm_dev,
		struct tcm_boot_info *boot_info)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int resp_data_len = 0;
	unsigned int copy_size;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_BOOT_INFO,
			NULL,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		hbp_err("Fail to send command 0x%02x\n",
			CMD_GET_BOOT_INFO);
		goto exit;
	}

	resp_data_len = tcm_dev->resp_buf.data_length;
	copy_size = MIN(sizeof(struct tcm_boot_info), resp_data_len);

	/* save the boot_info */
	retval = syna_pal_mem_cpy((unsigned char *)&tcm_dev->boot_info,
			sizeof(struct tcm_boot_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		hbp_err("Fail to copy boot info\n");
		goto exit;
	}

	if (boot_info == NULL)
		goto exit;

	/* copy boot_info to caller */
	retval = syna_pal_mem_cpy((unsigned char *)boot_info,
			sizeof(struct tcm_boot_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		hbp_err("Fail to copy boot info to caller\n");
		goto exit;
	}

exit:
	return retval;
}

/**
 * syna_tcm_get_app_info()
 *
 * Implement the application fw command code to request an application
 * info packet from device.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [out] app_info: the application info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_app_info(struct tcm_dev *tcm_dev,
		struct tcm_application_info *app_info)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int app_status;
	unsigned int resp_data_len = 0;
	unsigned int copy_size;
	struct tcm_application_info *info;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		hbp_err("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return _EINVAL;
	}
	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_APPLICATION_INFO,
			NULL,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		hbp_err("Fail to send command 0x%02x\n",
			CMD_GET_APPLICATION_INFO);
		goto exit;
	}

	resp_data_len = tcm_dev->resp_buf.data_length;
	copy_size = MIN(sizeof(tcm_dev->app_info), resp_data_len);

	info = &tcm_dev->app_info;

	/* save the app_info */
	retval = syna_pal_mem_cpy((unsigned char *)info,
			sizeof(struct tcm_application_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		hbp_err("Fail to copy application info\n");
		goto exit;
	}

	if (app_info == NULL)
		goto show_info;

	/* copy app_info to caller */
	retval = syna_pal_mem_cpy((unsigned char *)app_info,
			sizeof(struct tcm_application_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		hbp_err("Fail to copy application info to caller\n");
		goto exit;
	}

show_info:
	app_status = syna_pal_le2_to_uint(tcm_dev->app_info.status);

	if (app_status == APP_STATUS_BAD_APP_CONFIG) {
		hbp_err("Bad application firmware, status: 0x%x\n", app_status);
		retval = _ENODEV;
		goto exit;
	} else if (app_status != APP_STATUS_OK) {
		hbp_err("Incorrect application status, 0x%x\n", app_status);
		retval = _ENODEV;
		goto exit;
	}

	tcm_dev->max_objects = syna_pal_le2_to_uint(info->max_objects);
	tcm_dev->max_x = syna_pal_le2_to_uint(info->max_x);
	tcm_dev->max_y = syna_pal_le2_to_uint(info->max_y);

	tcm_dev->cols = syna_pal_le2_to_uint(info->num_of_image_cols);
	tcm_dev->rows = syna_pal_le2_to_uint(info->num_of_image_rows);
	syna_pal_mem_cpy((unsigned char *)tcm_dev->config_id,
			MAX_SIZE_CONFIG_ID,
			info->customer_config_id,
			MAX_SIZE_CONFIG_ID,
			MAX_SIZE_CONFIG_ID);

	if (!tcm_dev->max_objects) {
		hbp_err("Invalid max_objects: %d\n", tcm_dev->max_objects);
		tcm_dev->max_objects = MAX_NUM_OBJECTS;
	}
	hbp_info("App info version: %d, status: %d\n",
		syna_pal_le2_to_uint(info->version), app_status);
	hbp_info("App info: max_objs: %d, max_x:%d, max_y: %d, img: %dx%d\n",
		tcm_dev->max_objects, tcm_dev->max_x, tcm_dev->max_y,
		tcm_dev->rows, tcm_dev->cols);

exit:
	return retval;
}

/**
 * syna_tcm_set_dynamic_config()
 *
 * Implement the application fw command code to set the specified value to
 * the selected field of the dynamic configuration.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] id:       target field id
 *    [ in] value:    the value to the selected field
 *    [ in] delay_ms_resp: delay time for response reading.
 *                          a positive value presents the time for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_dynamic_config(struct tcm_dev *tcm_dev,
		unsigned char id, unsigned short value,
		unsigned int delay_ms_resp)
{
	int retval = 0;
	unsigned char out[3];
	unsigned char resp_code;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		hbp_err("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return _EINVAL;
	}

	hbp_info("Set %d to dynamic field 0x%x\n", value, id);

	out[0] = (unsigned char)id;
	out[1] = (unsigned char)value;
	out[2] = (unsigned char)(value >> 8);

	retval = tcm_dev->write_message(tcm_dev,
			CMD_SET_DYNAMIC_CONFIG,
			out,
			sizeof(out),
			&resp_code,
			delay_ms_resp);
	if (retval < 0) {
		hbp_err("Fail to send command 0x%02x to set %d to field 0x%x\n",
			CMD_SET_DYNAMIC_CONFIG, value, (unsigned char)id);
		goto exit;
	}

	retval = 0;

exit:
	return retval;
}

/**
 * syna_tcm_send_command()
 *
 * Helper to forward the custom command to the device
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] command:        TouchComm command
 *    [ in] payload:        data payload, if any
 *    [ in] payload_length: length of data payload, if any
 *    [out] resp_code:      response code returned
 *    [out] resp:           buffer to store the response data
 *    [ in] delay_ms_resp: delay time for response reading.
 *                          a positive value presents the time for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_send_command(struct tcm_dev *tcm_dev,
			unsigned char command, unsigned char *payload,
			unsigned int payload_length, unsigned char *code,
			struct tcm_buffer *resp, unsigned int delay_ms_resp)
{
	int retval = 0;

	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	if (!code) {
		hbp_err("Invalid parameter\n");
		return _EINVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			command,
			payload,
			payload_length,
			code,
			delay_ms_resp);
	if (retval < 0)
		hbp_err("Fail to run command 0x%02x\n", command);

	hbp_info("Status code returned: 0x%02x\n", *code);

	/* exit if no buffer provided */
	if (!resp)
		goto exit;

	/* if gathering a report, copy to the user buffer */
	if ((*code >= REPORT_IDENTIFY) && (*code != STATUS_INVALID)) {
		if (tcm_dev->report_buf.data_length == 0)
			goto exit;

		syna_tcm_buf_lock(&tcm_dev->report_buf);

		if (syna_tcm_buf_copy(resp, &tcm_dev->report_buf) < 0) {
			hbp_err("Fail to copy data, report type: %x\n",
				*code);
			syna_tcm_buf_unlock(&tcm_dev->report_buf);
			retval = _ENOMEM;
			goto exit;
		}

		syna_tcm_buf_unlock(&tcm_dev->report_buf);
	}

	/* if gathering a response, copy to the user buffer */
	if ((*code > STATUS_IDLE) && (*code <= STATUS_ERROR)) {
		if (tcm_dev->resp_buf.data_length == 0)
			goto exit;

		syna_tcm_buf_lock(&tcm_dev->resp_buf);

		if (syna_tcm_buf_copy(resp, &tcm_dev->resp_buf) < 0) {
			hbp_err("Fail to copy resp data, status code: %x\n",
				*code);
			syna_tcm_buf_unlock(&tcm_dev->resp_buf);
			retval = _ENOMEM;
			goto exit;
		}

		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
	}

exit:
	return retval;
}

/**
 * syna_tcm_enable_predict_reading()
 *
 * predict reading aims to retrieve all data in one transfer;
 * while, standard reads will read 4-byte header and payload data separately
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] en:      '1' to low power deep sleep mode; '0' to active mode
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_enable_predict_reading(struct tcm_dev *tcm_dev, bool en)
{
	if (!tcm_dev) {
		hbp_err("Invalid tcm device handle\n");
		return _EINVAL;
	}

	tcm_dev->msg_data.predict_reads = en;
	tcm_dev->msg_data.predict_length = 0;

	hbp_info("Predicted reading is %s\n",
		(en) ? "enabled":"disabled");

	return 0;
}
