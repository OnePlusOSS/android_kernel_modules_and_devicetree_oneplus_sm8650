// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include "synaptics_tcm_core.h"
#include <linux/hrtimer.h>
#if defined(CONFIG_SPI_MT65XX)
#include <linux/platform_data/spi-mt65xx.h>
#endif
#ifdef CONFIG_ARCH_HAS_SYSCALL_WRAPPER
#include <linux/syscalls.h>
#endif
/*
#ifndef REMOVE_OPLUS_FUNCTION
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#include<mt-plat/mtk_boot_common.h>
#else
#include <soc/oplus/system/boot_mode.h>
#endif
#endif
*/
#define PREDICTIVE_READING
#define MIN_READ_LENGTH 9
#define RESPONSE_TIMEOUT_MS_SHORT 300
#define RESPONSE_TIMEOUT_MS_DEFAULT 1000
#define RESPONSE_TIMEOUT_MS_LONG 3000

#define ERASE_FLASH_DELAY_MS 5000
#define WRITE_FLASH_DELAY_MS 200

#define APP_STATUS_POLL_TIMEOUT_MS 1000
#define APP_STATUS_POLL_MS 100

DECLARE_COMPLETION(response_complete);
DECLARE_COMPLETION(report_complete);

extern struct device_hcd *syna_remote_device_init(struct syna_tcm_hcd *tcm_hcd);
extern void wait_zeroflash_firmware_work(void);

static void syna_main_register(struct seq_file *s, void *chip_data);

static int syna_tcm_write_message(struct syna_tcm_hcd *tcm_hcd,
				  unsigned char command, unsigned char *payload,
				  unsigned int length, unsigned char **resp_buf,
				  unsigned int *resp_buf_size, unsigned int *resp_length,
				  unsigned int polling_delay_ms);
static void syna_tcm_test_report(struct syna_tcm_hcd *tcm_hcd);

struct syna_tcm_hcd *g_tcm_hcd = NULL;

#if defined(CONFIG_SPI_MT65XX)
static const struct mtk_chip_config spi_ctrdata = {
#if defined(CONFIG_MACH_MT6833)
		.rx_mlsb = 1,
		.tx_mlsb = 1,
#endif

	.sample_sel = 0,

	.cs_setuptime = 518,
	.cs_holdtime = 0,
	.cs_idletime = 0,
	.deassert_mode = false,
	.tick_delay = 0,
};
#endif
#define SYNA_CMD_GAME_AIUINIT_EN            0xF4
#define SYNA_CMD_GAME_AIUINIT               0xFF
#define AIUNIT_LONG_NUM         MAX_AIUNIT_SET_NUM*10


static int syna_tcm_spi_alloc_mem(struct syna_tcm_hcd *tcm_hcd,
				  unsigned int count, unsigned int size)
{
	struct spi_bus_data *spi_data = NULL;
	spi_data = &tcm_hcd->spi_data;

	if (count > spi_data->xfer_count) {
		kfree(spi_data->xfer);
		spi_data->xfer = kcalloc(count, sizeof(*spi_data->xfer), GFP_KERNEL);
		if (!spi_data->xfer) {
			TPD_INFO("Failed to allocate memory for xfer\n");
			spi_data->xfer_count = 0;
			return -ENOMEM;
		}
		spi_data->xfer_count = count;
	} else {
		memset(spi_data->xfer, 0, count * sizeof(*spi_data->xfer));
	}

	if (size > spi_data->buf_size) {
		if (spi_data->buf_size) {
			kfree(spi_data->buf);
		}
		spi_data->buf = kmalloc(size, GFP_KERNEL);
		if (!spi_data->buf) {
			TPD_INFO("Failed to allocate memory for buf\n");
			spi_data->buf_size = 0;
			return -ENOMEM;
		}
		spi_data->buf_size = size;
	}

	return 0;
}

#ifdef CONFIG_SPI_MT65XX
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);
#endif

inline int syna_tcm_rmi_read(struct syna_tcm_hcd *tcm_hcd,
			     unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval = 0;
	int ubl_byte_delay_us = 20;
	int ubl_max_freq = 1000000;

	unsigned int idx;
	unsigned int mode;
	unsigned int byte_count;
	struct spi_message msg;
	struct spi_device *spi = tcm_hcd->s_client;
	struct spi_bus_data *spi_data;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);
	spi_message_init(&msg);

	spi_data = &tcm_hcd->spi_data;
	byte_count = length + 2;

	TPD_DEBUG("ENTER syna_tcm_rmi_read\n");

	if (ubl_byte_delay_us == 0) {
		retval = syna_tcm_spi_alloc_mem(tcm_hcd, 2, byte_count);
	} else {
		retval = syna_tcm_spi_alloc_mem(tcm_hcd, byte_count, 3);
	}
	if (retval < 0) {
		TPD_INFO("Failed to allocate memory\n");
		goto exit;
	}

	spi_data->buf[0] = (unsigned char)(addr >> 8) | 0x80;
	spi_data->buf[1] = (unsigned char)addr;

	if (ubl_byte_delay_us == 0) {
		spi_data->xfer[0].len = 2;
		spi_data->xfer[0].tx_buf = spi_data->buf;
		spi_data->xfer[0].speed_hz = ubl_max_freq;
		spi_message_add_tail(&spi_data->xfer[0], &msg);
		memset(&spi_data->buf[2], 0xff, length);
		spi_data->xfer[1].len = length;
		spi_data->xfer[1].tx_buf = &spi_data->buf[2];
		spi_data->xfer[1].rx_buf = data;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
		if (tcm_hcd->block_delay_us) {
			spi_data->xfer[1].delay_usecs = tcm_hcd->block_delay_us;
		}
#endif
		spi_data->xfer[1].speed_hz = ubl_max_freq;
		spi_message_add_tail(&spi_data->xfer[1], &msg);
	} else {
		spi_data->buf[2] = 0xff;
		for (idx = 0; idx < byte_count; idx++) {
			spi_data->xfer[idx].len = 1;
			if (idx < 2) {
				spi_data->xfer[idx].tx_buf = &spi_data->buf[idx];
			} else {
				spi_data->xfer[idx].tx_buf = &spi_data->buf[2];
				spi_data->xfer[idx].rx_buf = &data[idx - 2];
			}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
			spi_data->xfer[idx].delay_usecs = ubl_byte_delay_us;
			if (tcm_hcd->block_delay_us && (idx == byte_count - 1)) {
				spi_data->xfer[idx].delay_usecs = tcm_hcd->block_delay_us;
			}
#endif
			spi_data->xfer[idx].speed_hz = ubl_max_freq;
			spi_message_add_tail(&spi_data->xfer[idx], &msg);
		}
	}

	mode = spi->mode;
	spi->mode = SPI_MODE_3;

#ifdef CONFIG_SPI_MT65XX
	mt_spi_enable_master_clk(spi);
#endif
	retval = spi_sync(spi, &msg);
	if (retval == 0) {
		retval = length;
	} else {
		TPD_INFO("Failed to complete SPI transfer, error = %d\n",
			 retval);
	}
#ifdef CONFIG_SPI_MT65XX
	mt_spi_disable_master_clk(spi);
#endif

	spi->mode = mode;

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);
	return retval;
}

inline int syna_tcm_rmi_write(struct syna_tcm_hcd *tcm_hcd,
			      unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval = 0;
	unsigned int mode;
	unsigned int byte_count;
	struct spi_bus_data *spi_data = NULL;
	struct spi_message msg;
	struct spi_device *spi = tcm_hcd->s_client;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	spi_message_init(&msg);
	spi_data = &tcm_hcd->spi_data;
	byte_count = length + 2;

	retval = syna_tcm_spi_alloc_mem(tcm_hcd, 1, byte_count);
	if (retval < 0) {
		TPD_INFO("Failed to allocate memory\n");
		goto exit;
	}

	spi_data->buf[0] = (unsigned char)(addr >> 8) & ~0x80;
	spi_data->buf[1] = (unsigned char)addr;
	retval = secure_memcpy(&spi_data->buf[2],
			       (spi_data->buf_size) - 2,
			       data,
			       length,
			       length);
	if (retval < 0) {
		TPD_INFO("Failed to copy write data\n");
		goto exit;
	}

	spi_data->xfer[0].len = byte_count;
	spi_data->xfer[0].tx_buf = spi_data->buf;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
	if (tcm_hcd->block_delay_us) {
		spi_data->xfer[0].delay_usecs = tcm_hcd->block_delay_us;
	}
#endif
	spi_message_add_tail(&spi_data->xfer[0], &msg);

	mode = spi->mode;
	spi->mode = SPI_MODE_3;

#ifdef CONFIG_SPI_MT65XX
	mt_spi_enable_master_clk(spi);
#endif
	retval = spi_sync(spi, &msg);
	if (retval == 0) {
		retval = length;
	} else {
		TPD_INFO("Failed to complete SPI transfer, error = %d\n",
			 retval);
	}
#ifdef CONFIG_SPI_MT65XX
	mt_spi_disable_master_clk(spi);
#endif

	spi->mode = mode;

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);
	return retval;
}

static inline int syna_tcm_read(struct syna_tcm_hcd *tcm_hcd,
				unsigned char *data, unsigned int length)
{
	int retval = 0;
	unsigned int idx;
	struct spi_message msg;
	struct spi_bus_data *spi_data = NULL;
	struct spi_device *spi = tcm_hcd->s_client;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);
	spi_message_init(&msg);
	spi_data = &tcm_hcd->spi_data;
	if (tcm_hcd->byte_delay_us == 0) {
		retval = syna_tcm_spi_alloc_mem(tcm_hcd, 1, length);
	} else {
		retval = syna_tcm_spi_alloc_mem(tcm_hcd, length, 1);
	}
	if (retval < 0) {
		TPD_INFO("Failed to allocate memory\n");
		goto exit;
	}

	if (tcm_hcd->byte_delay_us == 0) {
		memset(spi_data->buf, 0xff, length);
		spi_data->xfer[0].len = length;
		spi_data->xfer[0].tx_buf = spi_data->buf;
		spi_data->xfer[0].rx_buf = data;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
		if (tcm_hcd->block_delay_us) {
			spi_data->xfer[0].delay_usecs = tcm_hcd->block_delay_us;
		}
#endif
		spi_message_add_tail(&spi_data->xfer[0], &msg);
	} else {
		spi_data->buf[0] = 0xff;
		for (idx = 0; idx < length; idx++) {
			spi_data->xfer[idx].len = 1;
			spi_data->xfer[idx].tx_buf = spi_data->buf;
			spi_data->xfer[idx].rx_buf = &data[idx];
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
			spi_data->xfer[idx].delay_usecs = tcm_hcd->byte_delay_us;
			if (tcm_hcd->block_delay_us && (idx == length - 1)) {
				spi_data->xfer[idx].delay_usecs = tcm_hcd->block_delay_us;
			}
#endif
			spi_message_add_tail(&spi_data->xfer[idx], &msg);
		}
	}

#ifdef CONFIG_SPI_MT65XX
	mt_spi_enable_master_clk(spi);
#endif
	retval = spi_sync(spi, &msg);
	if (retval == 0) {
		retval = length;
	} else {
		TPD_INFO("Failed to complete SPI transfer, error = %d\n",
			 retval);
	}
#ifdef CONFIG_SPI_MT65XX
	mt_spi_disable_master_clk(spi);
#endif

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);
	return retval;
}

static inline int syna_tcm_write(struct syna_tcm_hcd *tcm_hcd,
				 unsigned char *data, unsigned int length)
{
	int retval = 0;
	unsigned int idx;
	struct spi_bus_data *spi_data = NULL;
	struct spi_message msg;
	struct spi_device *spi = tcm_hcd->s_client;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	spi_message_init(&msg);
	spi_data = &tcm_hcd->spi_data;
	if (tcm_hcd->byte_delay_us == 0) {
		retval = syna_tcm_spi_alloc_mem(tcm_hcd, 1, 0);
	} else {
		retval = syna_tcm_spi_alloc_mem(tcm_hcd, length, 0);
	}
	if (retval < 0) {
		TPD_INFO("Failed to allocate memory\n");
		goto exit;
	}

	if (tcm_hcd->byte_delay_us == 0) {
		spi_data->xfer[0].len = length;
		spi_data->xfer[0].tx_buf = data;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
		if (tcm_hcd->block_delay_us) {
			spi_data->xfer[0].delay_usecs = tcm_hcd->block_delay_us;
		}
#endif
		spi_message_add_tail(&spi_data->xfer[0], &msg);
	} else {
		for (idx = 0; idx < length; idx++) {
			spi_data->xfer[idx].len = 1;
			spi_data->xfer[idx].tx_buf = &data[idx];
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0))
			spi_data->xfer[idx].delay_usecs = tcm_hcd->byte_delay_us;
			if (tcm_hcd->block_delay_us && (idx == length - 1)) {
				spi_data->xfer[idx].delay_usecs = tcm_hcd->block_delay_us;
			}
#endif
			spi_message_add_tail(&spi_data->xfer[idx], &msg);
		}
	}
#ifdef CONFIG_SPI_MT65XX
	mt_spi_enable_master_clk(spi);
#endif
	retval = spi_sync(spi, &msg);
	if (retval == 0) {
		retval = length;
	} else {
		TPD_INFO("Failed to complete SPI transfer, error = %d\n",
			 retval);
	}
#ifdef CONFIG_SPI_MT65XX
	mt_spi_disable_master_clk(spi);
#endif

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int syna_get_report_data(struct syna_tcm_hcd *tcm_hcd, unsigned int offset,
				unsigned int bits, unsigned int *data)
{
	int retval = 0;
	unsigned char mask;
	unsigned char byte_data;
	unsigned int output_data;
	unsigned int bit_offset;
	unsigned int byte_offset;
	unsigned int data_bits;
	unsigned int available_bits;
	unsigned int remaining_bits;
	unsigned char *touch_report;

	touch_report = tcm_hcd->report.buffer.buf;
	output_data = 0;
	remaining_bits = bits;
	bit_offset = offset % 8;
	byte_offset = offset / 8;

	if (bits == 0 || bits > 32) {
		TPD_DEBUG("larger than 32 bits:%d\n", bits);
		retval = secure_memcpy((unsigned char *)data, bits / 8, &touch_report[byte_offset], bits / 8, bits / 8);
		if (retval < 0) {
			TPD_INFO("%s: Failed to copy write data[%d]\n", __func__, retval);
			return retval;
		}
		return 0;
	}

	if (offset + bits > tcm_hcd->report.buffer.data_length * 8) {
		*data = 0;
		return 0;
	}

	while (remaining_bits) {
		byte_data = touch_report[byte_offset];
		byte_data >>= bit_offset;

		available_bits = 8 - bit_offset;
		data_bits = MIN(available_bits, remaining_bits);
		mask = 0xff >> (8 - data_bits);

		byte_data &= mask;

		output_data |= byte_data << (bits - remaining_bits);

		bit_offset = 0;
		byte_offset += 1;
		remaining_bits -= data_bits;
	}

	*data = output_data;

	return 0;
}
int get_palm_status_example(void)
{
	if (g_tcm_hcd)
		return g_tcm_hcd->touch_hcd->touch_data.palm_status;
	return -1;
}

/**
 * touch_parse_report() - Parse touch report
 *
 * Traverse through the touch report configuration and parse the touch report
 * generated by the device accordingly to retrieve the touch data.
 */
static int syna_parse_report(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	bool active_only = 0;
	bool num_of_active_objects;
	unsigned char code;
	unsigned int size, idx, obj;
	unsigned int next, data, bits, offset, objects;
	unsigned int active_objects = 0;
	unsigned int report_size, config_size;
	unsigned char *config_data;
	struct touch_hcd *touch_hcd;
	struct touch_data *touch_data;
	struct object_data *object_data;

	touch_hcd = tcm_hcd->touch_hcd;
	if (touch_hcd->report_touch == false) {
		return -1;
	}
	touch_data = &touch_hcd->touch_data;
	object_data = touch_hcd->touch_data.object_data;
	config_data = tcm_hcd->config.buf;
	config_size = tcm_hcd->config.data_length;
	report_size = tcm_hcd->report.buffer.data_length;
	size = sizeof(*object_data) * touch_hcd->max_objects;
	memset(touch_hcd->touch_data.object_data, 0x00, size);

	num_of_active_objects = false;

	idx = 0;
	offset = 0;
	objects = 0;
	obj = 0;
	next = 0;
	while (idx < config_size) {
		code = config_data[idx++];
		switch (code) {
		case TOUCH_END:
			goto exit;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = idx;
			active_only = true;
			break;
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;
		case TOUCH_FOREACH_END:
			tcm_hcd->end_of_foreach = idx;
			if (active_only) {
				if (num_of_active_objects) {
					objects++;
					if (objects < active_objects) {
						idx = next;
					}
				} else if (offset < report_size * 8) {
					idx = next;
				}
			} else {
				obj++;
				if (obj < touch_hcd->max_objects) {
					idx = next;
				}
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = ceil_div(offset, 8) * 8;
			break;
		case TOUCH_TIMESTAMP:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_timestamp");
				}
				TPD_INFO("Failed to get timestamp\n");
				return retval;
			}
			touch_data->timestamp = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_INDEX:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &obj);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_index");
				}
				TPD_INFO("Failed to get object index\n");
				return retval;
			}
			offset += bits;
			break;
		case TOUCH_OBJECT_N_CLASSIFICATION:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_classific");
				}
				TPD_INFO("Failed to get object classification\n");
				return retval;
			}
			object_data[obj].status = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_POSITION:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_xpos");
				}
				TPD_INFO("Failed to get object x position\n");
				return retval;
			}
			object_data[obj].x_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_POSITION:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_ypos");
				}
				TPD_INFO("Failed to get object y position\n");
				return retval;
			}
			object_data[obj].y_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Z:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_zpos");
				}
				TPD_INFO("Failed to get object z\n");
				return retval;
			}
			object_data[obj].z = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_WIDTH:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_xwid");
				}
				TPD_INFO("Failed to get object x width\n");
				return retval;
			}
			object_data[obj].x_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_WIDTH:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_ywid");
				}
				TPD_INFO("Failed to get object y width\n");
				return retval;
			}
			object_data[obj].y_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_TX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_txpos");
				}
				TPD_INFO("Failed to get object tx position\n");
				return retval;
			}
			object_data[obj].tx_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_RX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_rxpos");
				}
				TPD_INFO("Failed to get object rx position\n");
				return retval;
			}
			object_data[obj].rx_pos = data;
			offset += bits;
			break;
		case TOUCH_0D_BUTTONS_STATE:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_btnstate");
				}
				TPD_INFO("Failed to get 0D buttons state\n");
				return retval;
			}
			touch_data->buttons_state = data;
			offset += bits;
			break;
		case TOUCH_GESTURE_DOUBLE_TAP:
		case TOUCH_REPORT_GESTURE_SWIPE:
		case TOUCH_REPORT_GESTURE_CIRCLE:
		case TOUCH_REPORT_GESTURE_UNICODE:
		case TOUCH_REPORT_GESTURE_VEE:
		case TOUCH_REPORT_GESTURE_TRIANGLE:
		case TOUCH_GESTURE_SINGLE_TAP:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_gesturetype");
				}
				TPD_INFO("Failed to get gesture double tap\n");
				return retval;
			}
			touch_data->lpwg_gesture = tcm_hcd->report.buffer.buf[0];
			offset += bits;
			break;
		case TOUCH_REPORT_GESTURE_INFO:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_gestureinfo");
				}
				TPD_INFO("Failed to get gesture double tap\n");
				return retval;
			}
			touch_data->extra_gesture_info = data;
			offset += bits;
			break;
		case TOUCH_REPORT_GESTURE_COORDINATE:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, (unsigned int *)(&touch_data->data_point[0]));
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_gesturepoint");
				}
				TPD_INFO("Failed to get gesture double tap\n");
				return retval;
			}
			offset += bits;
			break;
		case TOUCH_FRAME_RATE:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_framerate");
				}
				TPD_INFO("Failed to get frame rate\n");
				return retval;
			}
			touch_data->frame_rate = data;
			offset += bits;
			break;
		case TOUCH_POWER_IM:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_powerim");
				}
				TPD_INFO("Failed to get power IM\n");
				return retval;
			}
			touch_data->power_im = data;
			offset += bits;
			break;
		case TOUCH_CID_IM:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_cidim");
				}
				TPD_INFO("Failed to get CID IM\n");
				return retval;
			}
			touch_data->cid_im = data;
			offset += bits;
			break;
		case TOUCH_RAIL_IM:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_railim");
				}
				TPD_INFO("Failed to get rail IM\n");
				return retval;
			}
			touch_data->rail_im = data;
			offset += bits;
			break;
		case TOUCH_CID_VARIANCE_IM:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_varianeceim");
				}
				TPD_INFO("Failed to get CID variance IM\n");
				return retval;
			}
			touch_data->cid_variance_im = data;
			offset += bits;
			break;
		case TOUCH_NSM_FREQUENCY:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_nsmfreq");
				}
				TPD_INFO("Failed to get NSM frequency\n");
				return retval;
			}
			touch_data->nsm_frequency = data;
			offset += bits;
			break;
		case TOUCH_NSM_STATE:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_nsmstate");
				}
				TPD_INFO("Failed to get NSM state\n");
				return retval;
			}
			touch_data->nsm_state = data;
			offset += bits;
			break;
		case TOUCH_REPORT_PALM_DETECTED:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_nsmstate");
				}
				TPD_INFO("Failed to get NSM state\n");
				return retval;
			}
			touch_data->palm_status = data;
			offset += bits;
			break;
		case TOUCH_REPORT_GLOVE_DETECTED:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_nsmstate");
				}
				TPD_INFO("Failed to get NSM state\n");
				return retval;
			}
			touch_data->glove_status = data;
			offset += bits;
			break;
		case TOUCH_GET_WATER_MODE:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_nsmstate");
				}
				TPD_INFO("Failed to get NSM state\n");
				return retval;
			}
			touch_data->water_mode = data;
			offset += bits;
			break;
		case TOUCH_NUM_OF_ACTIVE_OBJECTS:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_activeobj");
				}
				TPD_INFO("Failed to get number of active objects\n");
				return retval;
			}
			active_objects = data;
			num_of_active_objects = true;
			touch_data->num_of_active_objects = data;
			offset += bits;
			if (touch_data->num_of_active_objects == 0) {
				idx = tcm_hcd->end_of_foreach;
			}
			break;
		case TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME:
			bits = config_data[idx++];
			retval = syna_get_report_data(tcm_hcd, offset, bits, &data);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "parse_report_err_cpucycleuse");
				}
				TPD_INFO("Failed to get number of CPU cycles used since last frame\n");
				return retval;
			}
			touch_data->num_of_cpu_cycles = data;
			offset += bits;
			break;
		case TOUCH_TUNING_GAUSSIAN_WIDTHS:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_TUNING_SMALL_OBJECT_PARAMS:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_TUNING_0D_BUTTONS_VARIANCE:
			bits = config_data[idx++];
			offset += bits;
			break;
		default:
			bits = config_data[idx++];
			offset += bits;
			break;
		}
	}

exit:
	return 0;
}

static int syna_get_input_params(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;

	LOCK_BUFFER(tcm_hcd->config);


	TPD_DETAIL("syna_get_input_params\n");

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_GET_TOUCH_REPORT_CONFIG,
					NULL,
					0,
					&tcm_hcd->config.buf,
					&tcm_hcd->config.buf_size,
					&tcm_hcd->config.data_length,
					0);
	if (retval < 0) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "get_input_para_err_cmd");
		}
		TPD_INFO("Failed to write command %s\n", STR(CMD_GET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(tcm_hcd->config);
		return retval;
	}
	TPD_DETAIL("syna_get_input_params end\n");

	UNLOCK_BUFFER(tcm_hcd->config);

	return 0;
}

static int syna_set_default_report_config(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	int length = 0;

	LOCK_BUFFER(tcm_hcd->config);

	length = tcm_hcd->default_config.buf_size;

	if (tcm_hcd->default_config.buf) {
		retval = syna_tcm_alloc_mem(tcm_hcd,
					    &tcm_hcd->config,
					    length);
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "set_report_cfg_err_default_alloc");
			}
			TPD_INFO("Failed to alloc mem\n");
			goto exit;
		}

		memcpy(tcm_hcd->config.buf, tcm_hcd->default_config.buf, length);
		tcm_hcd->config.buf_size = tcm_hcd->default_config.buf_size;
		tcm_hcd->config.data_length = tcm_hcd->default_config.data_length;
	}

exit:
	UNLOCK_BUFFER(tcm_hcd->config);

	return retval;
}

static int syna_get_default_report_config(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	/*unsigned int length;*/

	/*length = le2_to_uint(tcm_hcd->app_info.max_touch_report_config_size);*/

	LOCK_BUFFER(tcm_hcd->default_config);

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_GET_TOUCH_REPORT_CONFIG,
					NULL,
					0,
					&tcm_hcd->default_config.buf,
					&tcm_hcd->default_config.buf_size,
					&tcm_hcd->default_config.data_length,
					0);
	if (retval < 0) {
		TPD_INFO("Failed to write command %s\n", STR(CMD_GET_TOUCH_REPORT_CONFIG));
		goto exit;
	}

exit:
	UNLOCK_BUFFER(tcm_hcd->default_config);
	return retval;
}

static int syna_set_normal_report_config(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	unsigned int idx = 0;
	unsigned int length;
	struct touch_hcd *touch_hcd = tcm_hcd->touch_hcd;
	struct touchpanel_data *ts = spi_get_drvdata(tcm_hcd->s_client);

	TPD_INFO("%s:set normal report\n", __func__);
	length = le2_to_uint(tcm_hcd->app_info.max_touch_report_config_size);

	if (length < TOUCH_REPORT_CONFIG_SIZE) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "set_report_cfg_err_normal_len");
		}
		TPD_INFO("Invalid maximum touch report config size\n");
		return -EINVAL;
	}

	LOCK_BUFFER(touch_hcd->out);

	retval = syna_tcm_alloc_mem(tcm_hcd,
				    &touch_hcd->out,
				    length);
	if (retval < 0) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "set_report_cfg_err_normal_alloc");
		}
		TPD_INFO("Failed to allocate memory for touch_hcd->out.buf\n");
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	/*touch_hcd->out.buf[idx++] = TOUCH_GESTURE_DOUBLE_TAP;*/
	/*touch_hcd->out.buf[idx++] = 8;*/
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_GLOVE_DETECTED;
	touch_hcd->out.buf[idx++] = 8;
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_PALM_DETECTED;
	touch_hcd->out.buf[idx++] = 8;
	if (ts->waterproof_support) {
	touch_hcd->out.buf[idx++] = TOUCH_GET_WATER_MODE;
	touch_hcd->out.buf[idx++] = 8;
	}
	touch_hcd->out.buf[idx++] = TOUCH_FOREACH_ACTIVE_OBJECT;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_INDEX;
	touch_hcd->out.buf[idx++] = 4;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_CLASSIFICATION;
	touch_hcd->out.buf[idx++] = 4;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_X_POSITION;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_Y_POSITION;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_X_WIDTH;
	touch_hcd->out.buf[idx++] = 8;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_Y_WIDTH;
	touch_hcd->out.buf[idx++] = 8;
	touch_hcd->out.buf[idx++] = TOUCH_FOREACH_END;
	touch_hcd->out.buf[idx++] = TOUCH_END;

	LOCK_BUFFER(touch_hcd->resp);

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_SET_TOUCH_REPORT_CONFIG,
					touch_hcd->out.buf,
					length,
					&touch_hcd->resp.buf,
					&touch_hcd->resp.buf_size,
					&touch_hcd->resp.data_length,
					0);
	if (retval < 0) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "set_report_cfg_err_normal_cmd");
		}
		TPD_INFO("Failed to write command %s\n", STR(CMD_SET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(touch_hcd->resp);
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	UNLOCK_BUFFER(touch_hcd->resp);
	UNLOCK_BUFFER(touch_hcd->out);

	return retval;
}

static int syna_set_gesture_report_config(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	unsigned int idx = 0;
	unsigned int length;
	struct touch_hcd *touch_hcd = tcm_hcd->touch_hcd;
	struct touchpanel_data *ts = spi_get_drvdata(tcm_hcd->s_client);
	TPD_DEBUG("%s: set gesture report\n", __func__);
	length = le2_to_uint(tcm_hcd->app_info.max_touch_report_config_size);

	if (length < TOUCH_REPORT_CONFIG_SIZE) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "set_report_cfg_err_gesture_len");
		}
		TPD_INFO("Invalid maximum touch report config size\n");
		return -EINVAL;
	}

	LOCK_BUFFER(touch_hcd->out);

	retval = syna_tcm_alloc_mem(tcm_hcd,
				    &touch_hcd->out,
				    length);
	if (retval < 0) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "set_report_cfg_err_gesture_alloc");
		}
		TPD_INFO("Failed to allocate memory for touch_hcd->out.buf\n");
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_DOUBLE_TAP;
	touch_hcd->out.buf[idx++] = 1;
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_GESTURE_CIRCLE;
	touch_hcd->out.buf[idx++] = 1;
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_GESTURE_SWIPE;
	touch_hcd->out.buf[idx++] = 1;
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_GESTURE_UNICODE;
	touch_hcd->out.buf[idx++] = 1;
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_GESTURE_VEE;
	touch_hcd->out.buf[idx++] = 1;
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_GESTURE_TRIANGLE;
	if (ts->incell_aod_gesture_support) {
		touch_hcd->out.buf[idx++] = 1;
		touch_hcd->out.buf[idx++] = TOUCH_GESTURE_SINGLE_TAP;
	}
	touch_hcd->out.buf[idx++] = 1;
	touch_hcd->out.buf[idx++] = TOUCH_PAD_TO_NEXT_BYTE;
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_GESTURE_INFO;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_GESTURE_COORDINATE;
	touch_hcd->out.buf[idx++] = 192;
	touch_hcd->out.buf[idx++] = TOUCH_FOREACH_ACTIVE_OBJECT;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_INDEX;
	touch_hcd->out.buf[idx++] = 4;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_CLASSIFICATION;
	touch_hcd->out.buf[idx++] = 4;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_X_POSITION;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_Y_POSITION;
	touch_hcd->out.buf[idx++] = 16;
	touch_hcd->out.buf[idx++] = TOUCH_FOREACH_END;
	touch_hcd->out.buf[idx++] = TOUCH_END;

	LOCK_BUFFER(touch_hcd->resp);

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_SET_TOUCH_REPORT_CONFIG,
					touch_hcd->out.buf,
					length,
					&touch_hcd->resp.buf,
					&touch_hcd->resp.buf_size,
					&touch_hcd->resp.data_length,
					0);
	if (retval < 0) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "set_report_cfg_err_gesture_cmd");
		}
		TPD_INFO("Failed to write command %s\n", STR(CMD_SET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(touch_hcd->resp);
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	UNLOCK_BUFFER(touch_hcd->resp);
	UNLOCK_BUFFER(touch_hcd->out);

	return 0;
}

int syna_set_input_reporting(struct syna_tcm_hcd *tcm_hcd, bool suspend)
{
	int retval = 0;
	struct touch_hcd *touch_hcd = tcm_hcd->touch_hcd;

	TPD_DETAIL("%s: mode 0x%x, state %d\n", __func__, tcm_hcd->id_info.mode, suspend);
	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) || tcm_hcd->app_status != APP_STATUS_OK) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "set_input_report_notappfw");
		}
		TPD_INFO("Application firmware not running\n");
		return 0;
	}

	touch_hcd->report_touch = false;

	mutex_lock(&touch_hcd->report_mutex);

	if (!suspend) {
		retval = syna_set_normal_report_config(tcm_hcd);
		if (retval < 0) {
			TPD_INFO("Failed to set report config\n");
			goto default_config;
		}
	} else {
		retval = syna_set_gesture_report_config(tcm_hcd);
		if (retval < 0) {
			TPD_INFO("Failed to set report config\n");
			goto default_config;
		}
	}

	retval = syna_get_input_params(tcm_hcd);
	if (retval < 0) {
		TPD_INFO("Failed to get input parameters\n");
	}

	goto exit;

default_config:
	/*if failed to set report config, use default report config */
	retval = syna_set_default_report_config(tcm_hcd);
	if (retval < 0) {
		TPD_INFO("Failed to set default report config");
	}

exit:
	mutex_unlock(&touch_hcd->report_mutex);

	touch_hcd->report_touch = retval < 0 ? false : true;

	return retval;
}

static void syna_set_trigger_reason(struct syna_tcm_hcd *tcm_hcd, irq_reason trigger_reason)
{
	SET_BIT(tcm_hcd->trigger_reason, trigger_reason);
}

/*static void syna_tcm_resize_chunk_size(struct syna_tcm_hcd *tcm_hcd)
{
	unsigned int max_write_size;

	max_write_size = le2_to_uint(tcm_hcd->id_info.max_write_size);
	tcm_hcd->wr_chunk_size = MIN(max_write_size, WR_CHUNK_SIZE);
	if (tcm_hcd->wr_chunk_size == 0) {
		tcm_hcd->wr_chunk_size = max_write_size;
	}
}*/

/**
 * syna_tcm_dispatch_report() - dispatch report received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The report generated by the device is forwarded to the synchronous inbox of
 * each registered application module for further processing. In addition, the
 * report notifier thread is woken up for asynchronous notification of the
 * report occurrence.
 */
static void syna_tcm_dispatch_report(struct syna_tcm_hcd *tcm_hcd)
{
	int ret = 0;
	struct touch_hcd *touch_hcd = tcm_hcd->touch_hcd;
	struct touch_data *touch_data = &touch_hcd->touch_data;
	LOCK_BUFFER(tcm_hcd->in);
	LOCK_BUFFER(tcm_hcd->report.buffer);

	tcm_hcd->report.buffer.buf = &tcm_hcd->in.buf[MESSAGE_HEADER_SIZE];
	tcm_hcd->report.buffer.buf_size = tcm_hcd->in.buf_size;
	tcm_hcd->report.buffer.buf_size -= MESSAGE_HEADER_SIZE;
	tcm_hcd->report.buffer.data_length = tcm_hcd->payload_length;
	tcm_hcd->report.id = tcm_hcd->status_report_code;

	if (tcm_hcd->report.id == REPORT_TOUCH) {
		ret = syna_parse_report(tcm_hcd);
		if (ret < 0) {
			TPD_INFO("Failed to parse report\n");
			goto exit;
		}

		if (*tcm_hcd->in_suspend) {
			syna_set_trigger_reason(tcm_hcd, IRQ_GESTURE);
		} else {
			syna_set_trigger_reason(tcm_hcd, IRQ_TOUCH);
			if (touch_data->palm_status == PALM_TO_SLEEP) {
				syna_set_trigger_reason(tcm_hcd, IRQ_PALM);
				touch_data->palm_status = PALM_TO_DEFAULT;
			}
			if (touch_data->glove_status == GLOVE_TO_TRIGGER && touch_data->glove_flag == 0) {
				TPD_INFO("enter glove_mode = 1\n");
				touch_data->glove_flag = 1;
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_GLOVE, &touch_data->glove_flag);
			}

			if (touch_data->glove_status == GLOVE_TO_HAND && touch_data->glove_flag == 1) {
				TPD_INFO("quit glove_mode = 0\n");
				touch_data->glove_flag = 0;
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_GLOVE, &touch_data->glove_flag);
			}
		}
	} else if (tcm_hcd->report.id == REPORT_IDENTIFY) {
		if (tcm_hcd->id_info.mode == MODE_HOST_DOWNLOAD) {
			zeroflash_download_firmware();
		}
		if (tcm_hcd->id_info.mode == MODE_ROMBOOTLOADER) {
			tcm_hcd->hdl_finished_flag = 0;
			zeroflash_download_firmware();
		}
	} else if (tcm_hcd->report.id == REPORT_HDL_STATUS) {
		/*secure_memcpy((unsigned char * )dest,unsigned int dest_size,const unsigned char * src,unsigned int src_size,unsigned int count)*/
		zeroflash_download_config();
	} else if (tcm_hcd->report.id == REPORT_ROMBOOT) {
		zeroflash_download_firmware();
	} else if (tcm_hcd->report.id == REPORT_FW_PRINTF) {
#define FW_LOG_BUFFER_SIZE 256
		unsigned char fw_log[FW_LOG_BUFFER_SIZE] = {0};
		int cpy_length;
		cpy_length = (tcm_hcd->report.buffer.data_length >= FW_LOG_BUFFER_SIZE - 1)? (FW_LOG_BUFFER_SIZE - 1) : tcm_hcd->report.buffer.data_length;
		ret = secure_memcpy(fw_log, FW_LOG_BUFFER_SIZE - 1, tcm_hcd->report.buffer.buf, tcm_hcd->report.buffer.buf_size, cpy_length);
		if (ret < 0) {
			TPD_INFO("%s: Failed to copy write data[%d]\n", __func__, ret);
			return;
		}
		TPD_INFO("TouchFWLog: %s\n", fw_log);
	} else {
		syna_tcm_test_report(tcm_hcd);
		TPD_INFO("syna_tcm_test_report\n");
	}
	TPD_DEBUG("%s: glove_mode =%d\n", __func__, touch_data->glove_status);

exit:
	UNLOCK_BUFFER(tcm_hcd->report.buffer);
	UNLOCK_BUFFER(tcm_hcd->in);
	return;
}


/**
 * syna_tcm_dispatch_response() - dispatch response received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The response to a command is forwarded to the sender of the command.
 */
static void syna_tcm_dispatch_response(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;

	if (atomic_read(&tcm_hcd->command_status) != CMD_BUSY) {
		return;
	}

	tcm_hcd->response_code = tcm_hcd->status_report_code;
	LOCK_BUFFER(tcm_hcd->resp);

	if (tcm_hcd->payload_length == 0) {
		UNLOCK_BUFFER(tcm_hcd->resp);
		atomic_set(&tcm_hcd->command_status, CMD_IDLE);
		goto exit;
	}

	retval = syna_tcm_alloc_mem(tcm_hcd,
				    &tcm_hcd->resp,
				    tcm_hcd->payload_length);
	if (retval < 0) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "dispatch_resp_err_alloc");
		}
		TPD_INFO("Failed to allocate memory for tcm_hcd->resp.buf\n");
		UNLOCK_BUFFER(tcm_hcd->resp);
		atomic_set(&tcm_hcd->command_status, CMD_ERROR);
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->in);

	retval = secure_memcpy(tcm_hcd->resp.buf,
			       tcm_hcd->resp.buf_size,
			       &tcm_hcd->in.buf[MESSAGE_HEADER_SIZE],
			       tcm_hcd->in.buf_size - MESSAGE_HEADER_SIZE,
			       tcm_hcd->payload_length);
	if (retval < 0) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "dispatch_resp_err_cppld");
		}
		TPD_INFO("Failed to copy payload\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		UNLOCK_BUFFER(tcm_hcd->resp);
		atomic_set(&tcm_hcd->command_status, CMD_ERROR);
		goto exit;
	}

	tcm_hcd->resp.data_length = tcm_hcd->payload_length;

	UNLOCK_BUFFER(tcm_hcd->in);
	UNLOCK_BUFFER(tcm_hcd->resp);

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

exit:
	complete(&response_complete);

	return;
}

/**
 * syna_tcm_dispatch_message() - dispatch message received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The information received in the message read in from the device is dispatched
 * to the appropriate destination based on whether the information represents a
 * report or a response to a command.
 */
static void syna_tcm_dispatch_message(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	unsigned int payload_length;


	if (tcm_hcd->status_report_code == REPORT_IDENTIFY) {
		payload_length = tcm_hcd->payload_length;

		LOCK_BUFFER(tcm_hcd->in);

		retval = secure_memcpy((unsigned char *)&tcm_hcd->id_info,
				       sizeof(tcm_hcd->id_info),
				       &tcm_hcd->in.buf[MESSAGE_HEADER_SIZE],
				       tcm_hcd->in.buf_size - MESSAGE_HEADER_SIZE,
				       MIN(sizeof(tcm_hcd->id_info), payload_length));
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "dispatch_msg_err_cpidinfo");
			}
			TPD_INFO("Failed to copy identification info\n");
			UNLOCK_BUFFER(tcm_hcd->in);
			return;
		}

		UNLOCK_BUFFER(tcm_hcd->in);

		/*syna_tcm_resize_chunk_size(tcm_hcd);*/

		TPD_DETAIL("Received identify report (firmware mode = 0x%02x)\n",
			   tcm_hcd->id_info.mode);

		if (atomic_read(&tcm_hcd->command_status) == CMD_BUSY) {
			switch (tcm_hcd->command) {
			case CMD_RESET:
			case CMD_RUN_BOOTLOADER_FIRMWARE:
			case CMD_RUN_APPLICATION_FIRMWARE:
			case CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE:
				tcm_hcd->response_code = STATUS_OK;
				atomic_set(&tcm_hcd->command_status, CMD_IDLE);
				complete(&response_complete);
				break;
			default:
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "dispatch_msg_err_rst");
				}
				TPD_INFO("Device has been reset\n");
				atomic_set(&tcm_hcd->command_status, CMD_ERROR);
				complete(&response_complete);
				break;
			}
		}

		if (tcm_hcd->id_info.mode == MODE_HOST_DOWNLOAD) {
			return;
		}
	}

	if (tcm_hcd->status_report_code >= REPORT_IDENTIFY) {
		syna_tcm_dispatch_report(tcm_hcd);
	} else {
		syna_tcm_dispatch_response(tcm_hcd);
	}

	return;
}

/**
 * syna_tcm_continued_read() - retrieve entire payload from device
 *
 * @tcm_hcd: handle of core module
 *
 * Read transactions are carried out until the entire payload is retrieved from
 * the device and stored in the handle of the core module.
 */
static int syna_tcm_continued_read(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	unsigned char marker;
	unsigned char code;
	unsigned int idx;
	unsigned int offset;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int total_length;
	unsigned int remaining_length;

	total_length = MESSAGE_HEADER_SIZE + tcm_hcd->payload_length + 1;

	remaining_length = total_length - tcm_hcd->read_length;

	LOCK_BUFFER(tcm_hcd->in);

	retval = syna_tcm_realloc_mem(tcm_hcd,
				      &tcm_hcd->in,
				      total_length);
	if (retval < 0) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "continued_read_err_alloc");
		}
		TPD_INFO("Failed to reallocate memory for tcm_hcd->in.buf\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		return retval;
	}

	/* available chunk space for payload = total chunk size minus header
	 * marker byte and header code byte */
	if (tcm_hcd->rd_chunk_size == 0) {
		chunk_space = remaining_length;
	} else {
		chunk_space = tcm_hcd->rd_chunk_size - 2;
	}

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	offset = tcm_hcd->read_length;

	LOCK_BUFFER(tcm_hcd->temp);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space) {
			xfer_length = chunk_space;
		} else {
			xfer_length = remaining_length;
		}

		if (xfer_length == 1) {
			tcm_hcd->in.buf[offset] = MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}

		retval = syna_tcm_alloc_mem(tcm_hcd,
					    &tcm_hcd->temp,
					    xfer_length + 2);
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "continued_read_err_alloc");
			}
			TPD_INFO("Failed to allocate memory for tcm_hcd->temp.buf\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		retval = syna_tcm_read(tcm_hcd,
				       tcm_hcd->temp.buf,
				       xfer_length + 2);
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "continued_read_err_i2crd");
			}
			TPD_INFO("Failed to read from device\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		marker = tcm_hcd->temp.buf[0];
		code = tcm_hcd->temp.buf[1];

		if (marker != MESSAGE_MARKER) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "continued_read_err_marker");
			}
			TPD_INFO("Incorrect header marker (0x%02x)\n",
				 marker);
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return -EIO;
		}

		if (code != STATUS_CONTINUED_READ) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "continued_read_err_status");
			}
			TPD_INFO("Incorrect header code (0x%02x)\n",
				 code);
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return -EIO;
		}

		retval = secure_memcpy(&tcm_hcd->in.buf[offset],
				       total_length - offset,
				       &tcm_hcd->temp.buf[2],
				       xfer_length,
				       xfer_length);
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "continued_read_err_cppld");
			}
			TPD_INFO("Failed to copy payload\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		offset += xfer_length;

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->temp);
	UNLOCK_BUFFER(tcm_hcd->in);

	return 0;
}

/**
 * syna_tcm_raw_read() - retrieve specific number of data bytes from device
 *
 * @tcm_hcd: handle of core module
 * @in_buf: buffer for storing data retrieved from device
 * @length: number of bytes to retrieve from device
 *
 * Read transactions are carried out until the specific number of data bytes are
 * retrieved from the device and stored in in_buf.
 */
int syna_tcm_raw_read(struct syna_tcm_hcd *tcm_hcd,
		      unsigned char *in_buf, unsigned int length)
{
	int retval = 0;
	unsigned char code;
	unsigned int idx;
	unsigned int offset;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;

	if (length < 2) {
		TPD_INFO("Invalid length information\n");
		return -EINVAL;
	}

	/* minus header marker byte and header code byte */
	remaining_length = length - 2;

	/* available chunk space for data = total chunk size minus header marker
	 * byte and header code byte */
	if (tcm_hcd->rd_chunk_size == 0) {
		chunk_space = remaining_length;
	} else {
		chunk_space = tcm_hcd->rd_chunk_size - 2;
	}

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	offset = 0;

	LOCK_BUFFER(tcm_hcd->temp);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space) {
			xfer_length = chunk_space;
		} else {
			xfer_length = remaining_length;
		}

		if (xfer_length == 1) {
			in_buf[offset] = MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}

		retval = syna_tcm_alloc_mem(tcm_hcd,
					    &tcm_hcd->temp,
					    xfer_length + 2);
		if (retval < 0) {
			TPD_INFO("Failed to allocate memory for tcm_hcd->temp.buf\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		retval = syna_tcm_read(tcm_hcd,
				       tcm_hcd->temp.buf,
				       xfer_length + 2);
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "raw_read_err_i2crd");
			}
			TPD_INFO("Failed to read from device\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		code = tcm_hcd->temp.buf[1];

		if (idx == 0) {
			retval = secure_memcpy(&in_buf[0],
					       length,
					       &tcm_hcd->temp.buf[0],
					       xfer_length + 2,
					       xfer_length + 2);
		} else {
			if (code != STATUS_CONTINUED_READ) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "raw_read_err_status");
				}
				TPD_INFO("Incorrect header code (0x%02x)\n",
					 code);
				UNLOCK_BUFFER(tcm_hcd->temp);
				return -EIO;
			}

			retval = secure_memcpy(&in_buf[offset],
					       length - offset,
					       &tcm_hcd->temp.buf[2],
					       xfer_length,
					       xfer_length);
		}
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "raw_read_err_cpxfer");
			}
			TPD_INFO("Failed to copy data\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		if (idx == 0) {
			offset += (xfer_length + 2);
		} else {
			offset += xfer_length;
		}

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->temp);

	return 0;
}

/**
 * syna_tcm_raw_write() - write command/data to device without receiving
 * response
 *
 * @tcm_hcd: handle of core module
 * @command: command to send to device
 * @data: data to send to device
 * @length: length of data in bytes
 *
 * A command and its data, if any, are sent to the device.
 */
static int syna_tcm_raw_write(struct syna_tcm_hcd *tcm_hcd,
			      unsigned char command, unsigned char *data, unsigned int length)
{
	int retval = 0;
	unsigned int idx;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;

	if (length == 0) {
		return 0;
	}

	remaining_length = length;

	/* available chunk space for data = total chunk size minus command
	 * byte */
	if (tcm_hcd->wr_chunk_size == 0) {
		chunk_space = remaining_length;
	} else {
		chunk_space = tcm_hcd->wr_chunk_size - 1;
	}

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	LOCK_BUFFER(tcm_hcd->out);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space) {
			xfer_length = chunk_space;
		} else {
			xfer_length = remaining_length;
		}

		retval = syna_tcm_alloc_mem(tcm_hcd,
					    &tcm_hcd->out,
					    xfer_length + 1);
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "raw_write_err_alloc");
			}
			TPD_INFO("Failed to allocate memory for tcm_hcd->out.buf\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			return retval;
		}

		if (idx == 0) {
			tcm_hcd->out.buf[0] = command;
		} else {
			tcm_hcd->out.buf[0] = CMD_CONTINUE_WRITE;
		}

		if (xfer_length) {
			retval = secure_memcpy(&tcm_hcd->out.buf[1],
					       xfer_length,
					       &data[idx * chunk_space],
					       remaining_length,
					       xfer_length);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "raw_write_err_cpxfer");
				}
				TPD_INFO("Failed to copy data\n");
				UNLOCK_BUFFER(tcm_hcd->out);
				return retval;
			}
		}

		retval = syna_tcm_write(tcm_hcd,
					tcm_hcd->out.buf,
					xfer_length + 1);
		if (retval < 0) {
			TPD_INFO("Failed to write to device\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			return retval;
		}

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->out);

	return 0;
}

/*add this for debug. remove before pvt*/
static void syna_tcm_debug_message(char *buf, int len)
{
	int i = 0;

	for (i = 0; i < len; i++) {
		if (i > 32) {
			break;
		}

		TPD_DEBUG("0x%x ", buf[i]);
	}

	TPD_DEBUG("\n");
}

/**
 * syna_tcm_read_message() - read message from device
 *
 * @tcm_hcd: handle of core module
 * @in_buf: buffer for storing data in raw read mode
 * @length: length of data in bytes in raw read mode
 *
 * If in_buf is not NULL, raw read mode is used and syna_tcm_raw_read() is
 * called. Otherwise, a message including its entire payload is retrieved from
 * the device and dispatched to the appropriate destination.
 */
void syna_log_data(unsigned char *data, int length)
{
	int i;

	for (i = 0; i < length; i++) {
		TPD_DEBUG("syna data[%d]:%x, ", i, data[i]);
	}
	TPD_DEBUG("syna data end\n");
}
static int syna_tcm_read_message(struct syna_tcm_hcd *tcm_hcd,
				 unsigned char *in_buf, unsigned int length)
{
	int retval = 0;
	bool retry;
	unsigned int total_length;
	struct syna_tcm_message_header *header;

	TPD_DEBUG("%s\n", __func__);
	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	if (in_buf != NULL) {
		retval = syna_tcm_raw_read(tcm_hcd, in_buf, length);
		goto exit;
	}

	retry = true;
retry:
	LOCK_BUFFER(tcm_hcd->in);

	retval = syna_tcm_read(tcm_hcd,
			       tcm_hcd->in.buf,
			       tcm_hcd->read_length);
	if (retval < 0) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "read_msg_err_i2crd");
		}
		TPD_INFO("Failed to read from device\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		if (retry) {
			usleep_range(5000, 10000);
			retry = false;
			goto retry;
		}

		goto exit;
	}

	header = (struct syna_tcm_message_header *)tcm_hcd->in.buf;

	if (header->marker != MESSAGE_MARKER) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "read_msg_err_marker");
		}
		TPD_INFO("header->marker = %02x\n", header->marker);
		UNLOCK_BUFFER(tcm_hcd->in);
		retval = -ENXIO;
		if (retry) {
			usleep_range(5000, 10000);
			retry = false;
			goto retry;
		}
		goto exit;
	}

	tcm_hcd->status_report_code = header->code;
	tcm_hcd->payload_length = le2_to_uint(header->length);

	TPD_DEBUG("Header code = 0x%02x Payload len = %d\n",
		  tcm_hcd->status_report_code, tcm_hcd->payload_length);

	if (tcm_hcd->status_report_code <= STATUS_ERROR ||
	    tcm_hcd->status_report_code == STATUS_INVALID) {
		switch (tcm_hcd->status_report_code) {
		case STATUS_OK:
			break;
		case STATUS_CONTINUED_READ:
			TPD_INFO("Out-of-sync continued read\n");
			tcm_hcd->payload_length = 0;
			UNLOCK_BUFFER(tcm_hcd->in);
			retval = 0;
			goto exit;
		case STATUS_IDLE:
		case STATUS_BUSY:
			tcm_hcd->payload_length = 0;
			UNLOCK_BUFFER(tcm_hcd->in);
			retval = 0;
			goto exit;
		default:
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "read_msg_err_header");
			}
			TPD_INFO("Incorrect header code (0x%02x)\n",
				 tcm_hcd->status_report_code);
			if (tcm_hcd->status_report_code != STATUS_ERROR) {
				UNLOCK_BUFFER(tcm_hcd->in);
				retval = -EIO;
				goto exit;
			}
		}
	}

	total_length = MESSAGE_HEADER_SIZE + tcm_hcd->payload_length + 1;

#ifdef PREDICTIVE_READING
	if (total_length <= tcm_hcd->read_length) {
		goto check_padding;
	} else if (total_length - 1 == tcm_hcd->read_length) {
		tcm_hcd->in.buf[total_length - 1] = MESSAGE_PADDING;
		goto check_padding;
	}
#else
	if (tcm_hcd->payload_length == 0) {
		tcm_hcd->in.buf[total_length - 1] = MESSAGE_PADDING;
		goto check_padding;
	}
#endif

	UNLOCK_BUFFER(tcm_hcd->in);
	usleep_range(5, 10);
	retval = syna_tcm_continued_read(tcm_hcd);
	if (retval < 0) {
		TPD_INFO("Failed to do continued read\n");
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->in);

	tcm_hcd->in.buf[0] = MESSAGE_MARKER;
	tcm_hcd->in.buf[1] = tcm_hcd->status_report_code;
	tcm_hcd->in.buf[2] = (unsigned char)tcm_hcd->payload_length;
	tcm_hcd->in.buf[3] = (unsigned char)(tcm_hcd->payload_length >> 8);

check_padding:
	if (tcm_hcd->in.buf[total_length - 1] != MESSAGE_PADDING) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "read_msg_err_padding");
		}
		TPD_INFO("Incorrect message padding byte (0x%02x)\n",
			 tcm_hcd->in.buf[total_length - 1]);
		UNLOCK_BUFFER(tcm_hcd->in);
		retval = -EIO;
		goto exit;
	}

	UNLOCK_BUFFER(tcm_hcd->in);

#ifdef PREDICTIVE_READING
	total_length = MAX(total_length, MIN_READ_LENGTH);
	tcm_hcd->read_length = MIN(total_length, tcm_hcd->rd_chunk_size);
	if (tcm_hcd->rd_chunk_size == 0) {
		tcm_hcd->read_length = total_length;
	}
#endif

	/*add for debug, remove before pvt*/
	if (LEVEL_DEBUG == tp_debug) {
		syna_tcm_debug_message(&tcm_hcd->in.buf[4], tcm_hcd->payload_length);
	}
	syna_log_data(&tcm_hcd->in.buf[0], tcm_hcd->payload_length + 4);

	syna_tcm_dispatch_message(tcm_hcd);

	retval = 0;

exit:
	if (retval < 0) {
		if (atomic_read(&tcm_hcd->command_status) == CMD_BUSY) {
			atomic_set(&tcm_hcd->command_status, CMD_ERROR);
			complete(&response_complete);
		}
	}

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);

	return retval;
}

/**
 * syna_tcm_write_message() - write message to device and receive response
 *
 * @tcm_hcd: handle of core module
 * @command: command to send to device
 * @payload: payload of command
 * @length: length of payload in bytes
 * @resp_buf: buffer for storing command response
 * @resp_buf_size: size of response buffer in bytes
 * @resp_length: length of command response in bytes
 * @polling_delay_ms: delay time after sending command before resuming polling
 *
 * If resp_buf is NULL, raw write mode is used and syna_tcm_raw_write() is
 * called. Otherwise, a command and its payload, if any, are sent to the device
 * and the response to the command generated by the device is read in.
 */
static int syna_tcm_write_message(struct syna_tcm_hcd *tcm_hcd,
				  unsigned char command, unsigned char *payload,
				  unsigned int length, unsigned char **resp_buf,
				  unsigned int *resp_buf_size, unsigned int *resp_length,
				  unsigned int timeout)
{
	int retval = 0;
	unsigned int idx;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;
	unsigned int command_status = 0;
	unsigned int timeout_ms = 0;

	mutex_lock(&tcm_hcd->command_mutex);

	if (!tcm_hcd->init_okay) {
		TPD_INFO("%s:Command = 0x%02x NOT RUN: init nok\n", __func__, command);
		retval = -EIO;
		goto exit;
	}

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	if (resp_buf == NULL) {
		retval = syna_tcm_raw_write(tcm_hcd, command, payload, length);
		mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
		goto exit;
	}

	atomic_set(&tcm_hcd->command_status, CMD_BUSY);
	reinit_completion(&response_complete);
	tcm_hcd->command = command;

	LOCK_BUFFER(tcm_hcd->resp);

	tcm_hcd->resp.buf = *resp_buf;
	tcm_hcd->resp.buf_size = *resp_buf_size;
	tcm_hcd->resp.data_length = 0;

	UNLOCK_BUFFER(tcm_hcd->resp);

	/* adding two length bytes as part of payload */
	remaining_length = length + 2;

	/* available chunk space for payload = total chunk size minus command
	 * byte */
	if (tcm_hcd->wr_chunk_size == 0) {
		chunk_space = remaining_length;
	} else {
		chunk_space = tcm_hcd->wr_chunk_size - 1;
	}
	if (command == CMD_ROMBOOT_DOWNLOAD) {
		chunk_space = remaining_length;
	}
	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	TPD_DEBUG("%s:Command = 0x%02x\n", __func__, command);

	LOCK_BUFFER(tcm_hcd->out);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space) {
			xfer_length = chunk_space;
		} else {
			xfer_length = remaining_length;
		}

		retval = syna_tcm_alloc_mem(tcm_hcd,
					    &tcm_hcd->out,
					    xfer_length + 1);
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "write_msg_err_alloc");
			}
			TPD_INFO("Failed to allocate memory for tcm_hcd->out.buf\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
			goto exit;
		}

		if (idx == 0) {
			tcm_hcd->out.buf[0] = command;
			tcm_hcd->out.buf[1] = (unsigned char)length;
			tcm_hcd->out.buf[2] = (unsigned char)(length >> 8);

			if (xfer_length > 2) {
				retval = secure_memcpy(&tcm_hcd->out.buf[3],
						       xfer_length - 2,
						       payload,
						       remaining_length - 2,
						       xfer_length - 2);
				if (retval < 0) {
					if (tcm_hcd->health_monitor_support) {
						tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "write_msg_err_cpxfer");
					}
					TPD_INFO("Failed to copy payload\n");
					UNLOCK_BUFFER(tcm_hcd->out);
					mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
					goto exit;
				}
			}
		} else {
			tcm_hcd->out.buf[0] = CMD_CONTINUE_WRITE;

			retval = secure_memcpy(&tcm_hcd->out.buf[1],
					       xfer_length,
					       &payload[idx * chunk_space - 2],
					       remaining_length,
					       xfer_length);
			if (retval < 0) {
				if (tcm_hcd->health_monitor_support) {
					tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "write_msg_err_cpxfer");
				}
				TPD_INFO("Failed to copy payload\n");
				UNLOCK_BUFFER(tcm_hcd->out);
				mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
				goto exit;
			}
		}
		TPD_DEBUG("%s:[%d] buf[0]-0x%02x buf[1]-0x%02x buf[2]-0x%02x buf[3]-0x%02x \n", __func__, idx, \
					tcm_hcd->out.buf[0], tcm_hcd->out.buf[1], tcm_hcd->out.buf[2], tcm_hcd->out.buf[3]);
		retval = syna_tcm_write(tcm_hcd,
					tcm_hcd->out.buf,
					xfer_length + 1);
		if (retval < 0) {
			TPD_INFO("Failed to write to device\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
			goto exit;
		}

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->out);

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);

	if (timeout == 0) {
		timeout_ms = RESPONSE_TIMEOUT_MS_DEFAULT;
	} else {
		timeout_ms = timeout;
	}

	retval = wait_for_completion_timeout(&response_complete,
					     msecs_to_jiffies(timeout_ms));
	if (retval == 0) {
		TPD_INFO("Timed out waiting for response (command 0x%02x)\n",
			 tcm_hcd->command);
		retval = -EIO;
	} else {
		command_status = atomic_read(&tcm_hcd->command_status);

		if (command_status != CMD_IDLE ||
		    tcm_hcd->status_report_code == STATUS_ERROR) {
			TPD_INFO("Failed to get valid response\n");
			retval = -EIO;
			goto exit;
		}

		retval = 0;
	}
	TPD_DEBUG("%s: status code = 0x%02x  command :0x%02x\n",
		 __func__, tcm_hcd->status_report_code, command);

exit:
	if (command_status == CMD_IDLE) {
		LOCK_BUFFER(tcm_hcd->resp);

		if (tcm_hcd->status_report_code == STATUS_ERROR) {
			if (tcm_hcd->resp.data_length) {
				TPD_INFO("Error code = 0x%02x\n",
					 tcm_hcd->resp.buf[0]);
			}
		}

		if (resp_buf != NULL) {
			*resp_buf = tcm_hcd->resp.buf;
			*resp_buf_size = tcm_hcd->resp.buf_size;
			*resp_length = tcm_hcd->resp.data_length;
		}

		UNLOCK_BUFFER(tcm_hcd->resp);
	}

	tcm_hcd->command = CMD_NONE;
	atomic_set(&tcm_hcd->command_status, CMD_IDLE);
	mutex_unlock(&tcm_hcd->command_mutex);

	return retval;
}

#define RESPONSE_TIMEOUT_MS 3000
#define WRITE_DELAY_US_MIN 100
#define WRITE_DELAY_US_MAX 300
static int syna_tcm_write_message_zeroflash(struct syna_tcm_hcd *tcm_hcd,
		unsigned char command, unsigned char *payload,
		unsigned int length, unsigned char **resp_buf,
		unsigned int *resp_buf_size, unsigned int *resp_length,
		unsigned char *response_code, unsigned int polling_delay_ms)
{
	int retval = 0;
	unsigned int idx;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;
	unsigned int command_status;

	if (response_code != NULL) {
		*response_code = STATUS_INVALID;
	}


	mutex_lock(&tcm_hcd->command_mutex);

	if (!tcm_hcd->init_okay) {
		TPD_INFO("%s:Command = 0x%02x NOT RUN: init nok\n", __func__, command);
		retval = -EIO;
		goto exit;
	}

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	if (resp_buf == NULL) {
		retval = syna_tcm_raw_write(tcm_hcd, command, payload, length);
		mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
		goto exit;
	}



	atomic_set(&tcm_hcd->command_status, CMD_BUSY);


	reinit_completion(&response_complete);


	tcm_hcd->command = command;

	LOCK_BUFFER(tcm_hcd->resp);

	tcm_hcd->resp.buf = *resp_buf;
	tcm_hcd->resp.buf_size = *resp_buf_size;
	tcm_hcd->resp.data_length = 0;

	UNLOCK_BUFFER(tcm_hcd->resp);

	/* adding two length bytes as part of payload */
	remaining_length = length + 2;

	/* available chunk space for payload = total chunk size minus command
	 * byte */
	if (tcm_hcd->wr_chunk_size == 0) {
		chunk_space = remaining_length;
	} else {
		chunk_space = tcm_hcd->wr_chunk_size - 1;
	}
	if (command == CMD_DOWNLOAD_CONFIG) {
		chunk_space = WR_CHUNK_LENGHT;
	}

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	TPD_INFO("%s:Command = 0x%02x\n", __func__, command);

	LOCK_BUFFER(tcm_hcd->out);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space) {
			xfer_length = chunk_space;
		} else {
			xfer_length = remaining_length;
		}

		retval = syna_tcm_alloc_mem(tcm_hcd,
					    &tcm_hcd->out,
					    xfer_length + 1);
		if (retval < 0) {
			TPD_INFO("Failed to allocate memory for tcm_hcd->out.buf\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
			goto exit;
		}

		if (idx == 0) {
			tcm_hcd->out.buf[0] = command;
			tcm_hcd->out.buf[1] = (unsigned char)length;
			tcm_hcd->out.buf[2] = (unsigned char)(length >> 8);

			if (xfer_length > 2) {
				retval = secure_memcpy(&tcm_hcd->out.buf[3],
						       tcm_hcd->out.buf_size - 3,
						       payload,
						       remaining_length - 2,
						       xfer_length - 2);
				if (retval < 0) {
					TPD_INFO("Failed to copy payload\n");
					UNLOCK_BUFFER(tcm_hcd->out);
					mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
					goto exit;
				}
			}
		} else {
			tcm_hcd->out.buf[0] = CMD_CONTINUE_WRITE;

			retval = secure_memcpy(&tcm_hcd->out.buf[1],
					       tcm_hcd->out.buf_size - 1,
					       &payload[idx * chunk_space - 2],
					       remaining_length,
					       xfer_length);
			if (retval < 0) {
				TPD_INFO("Failed to copy payload\n");
				UNLOCK_BUFFER(tcm_hcd->out);
				mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
				goto exit;
			}
		}

		retval = syna_tcm_write(tcm_hcd,
					tcm_hcd->out.buf,
					xfer_length + 1);
		if (retval < 0) {
			TPD_INFO("Failed to write to device\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
			goto exit;
		}

		remaining_length -= xfer_length;

		if (chunks > 1) {
			usleep_range(1000, 1000);
		}
	}

	UNLOCK_BUFFER(tcm_hcd->out);

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);


	if (!tcm_hcd->esd_irq_disabled) {
		retval = wait_for_completion_timeout(&response_complete,
						     msecs_to_jiffies(RESPONSE_TIMEOUT_MS));
	} else {
		retval = 0;
	}
	if (retval == 0) {
		TPD_INFO("Timed out waiting for response (command 0x%02x)\n",
			 tcm_hcd->command);
		retval = -EIO;
		goto exit;
	}

	command_status = atomic_read(&tcm_hcd->command_status);
	if (command_status != CMD_IDLE) {
		TPD_INFO("Failed to get valid response (command 0x%02x)\n",
			 tcm_hcd->command);
		retval = -EIO;
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->resp);

	if (tcm_hcd->response_code != STATUS_OK) {
		if (tcm_hcd->resp.data_length) {
			TPD_INFO("Error code = 0x%02x (command 0x%02x)\n",
				 tcm_hcd->resp.buf[0], tcm_hcd->command);
		}
		retval = -EIO;
	} else {
		retval = 0;
	}
	TPD_DEBUG("%s: status code = 0x%02x  command :0x%02x\n",
		 __func__, tcm_hcd->response_code, command);

	*resp_buf = tcm_hcd->resp.buf;
	*resp_buf_size = tcm_hcd->resp.buf_size;
	*resp_length = tcm_hcd->resp.data_length;

	if (response_code != NULL) {
		*response_code = tcm_hcd->response_code;
	}

	UNLOCK_BUFFER(tcm_hcd->resp);

exit:
	tcm_hcd->command = CMD_NONE;

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

	mutex_unlock(&tcm_hcd->command_mutex);

	return retval;
}

static int syna_tcm_get_app_info(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	unsigned int timeout;

	timeout = APP_STATUS_POLL_TIMEOUT_MS;

	resp_buf = NULL;
	resp_buf_size = 0;

get_app_info:
	retval = syna_tcm_write_message(tcm_hcd,
					CMD_GET_APPLICATION_INFO,
					NULL,
					0,
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					0);
	if (retval < 0) {
		TPD_INFO("Failed to write command %s\n",
			 STR(CMD_GET_APPLICATION_INFO));
		goto exit;
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->app_info,
			       sizeof(tcm_hcd->app_info),
			       resp_buf,
			       resp_buf_size,
			       MIN(sizeof(tcm_hcd->app_info), resp_length));
	if (retval < 0) {
		TPD_INFO("Failed to copy application info\n");
		goto exit;
	}

	tcm_hcd->app_status = le2_to_uint(tcm_hcd->app_info.status);

	if (tcm_hcd->app_status == APP_STATUS_BOOTING ||
	    tcm_hcd->app_status == APP_STATUS_UPDATING) {
		if (timeout > 0) {
			msleep(APP_STATUS_POLL_MS);
			timeout -= APP_STATUS_POLL_MS;
			goto get_app_info;
		}
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_get_boot_info(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_GET_BOOT_INFO,
					NULL,
					0,
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					0);
	if (retval < 0) {
		TPD_INFO("Failed to write command %s\n",
			 STR(CMD_GET_BOOT_INFO));
		goto exit;
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->boot_info,
			       sizeof(tcm_hcd->boot_info),
			       resp_buf,
			       resp_buf_size,
			       MIN(sizeof(tcm_hcd->boot_info), resp_length));
	if (retval < 0) {
		TPD_INFO("Failed to copy boot info\n");
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_identify(struct syna_tcm_hcd *tcm_hcd, bool id)
{
	int retval = 0;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;

	mutex_lock(&tcm_hcd->identify_mutex);

	if (!id) {
		goto get_info;
	}

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_IDENTIFY,
					NULL,
					0,
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					0);
	if (retval < 0) {
		TPD_INFO("Failed to write command %s\n",
			 STR(CMD_IDENTIFY));
		goto exit;
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->id_info,
			       sizeof(tcm_hcd->id_info),
			       resp_buf,
			       resp_buf_size,
			       MIN(sizeof(tcm_hcd->id_info), resp_length));
	if (retval < 0) {
		TPD_INFO("Failed to copy identification info\n");
		goto exit;
	}

	/*syna_tcm_resize_chunk_size(tcm_hcd);*/

get_info:
	if (IS_FW_MODE(tcm_hcd->id_info.mode)) {
		retval = syna_tcm_get_app_info(tcm_hcd);
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "identify_err_appinfo");
			}
			TPD_INFO("Failed to get application info\n");
			goto exit;
		}
	} else {
		retval = syna_tcm_get_boot_info(tcm_hcd);
		if (retval < 0) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "identify_err_bootinfo");
			}
			TPD_INFO("Failed to get boot info\n");
			goto exit;
		}
	}

	retval = 0;

exit:
	mutex_unlock(&tcm_hcd->identify_mutex);

	kfree(resp_buf);

	return retval;
}
int syna_tcm_run_bootloader_firmware(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	unsigned char command;

	resp_buf = NULL;
	resp_buf_size = 0;
	command = (tcm_hcd->id_info.mode == MODE_ROMBOOTLOADER) ?
		  CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE :
		  CMD_RUN_BOOTLOADER_FIRMWARE;

	retval = syna_tcm_write_message(tcm_hcd,
					command,
					NULL,
					0,
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					0);
	if (retval < 0) {
		if (tcm_hcd->id_info.mode == MODE_ROMBOOTLOADER) {
			TPD_INFO("Failed to write command %s\n",
				 STR(CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE));
		} else {
			TPD_INFO("Failed to write command %s\n",
				 STR(CMD_RUN_BOOTLOADER_FIRMWARE));
		}
		goto exit;
	}
	if (command != CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE) {
		retval = syna_tcm_identify(tcm_hcd, false);
		if (retval < 0) {
			TPD_INFO("Failed to do identification\n");
			goto exit;
		}

		if (IS_FW_MODE(tcm_hcd->id_info.mode)) {
			if (tcm_hcd->health_monitor_support) {
				tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "app_fw_err_mode");
			}
			TPD_INFO("Failed to enter bootloader mode\n");
			retval = -EINVAL;
			goto exit;
		}
	}
	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

/*
static int syna_tcm_run_application_firmware(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	bool retry;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	retry = true;

	resp_buf = NULL;
	resp_buf_size = 0;

retry:
	retval = syna_tcm_write_message(tcm_hcd,
					CMD_RUN_APPLICATION_FIRMWARE,
					NULL,
					0,
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					0);
	if (retval < 0) {
		TPD_INFO("Failed to write command %s\n",
			 STR(CMD_RUN_APPLICATION_FIRMWARE));
		goto exit;
	}

	retval = syna_tcm_identify(tcm_hcd, false);
	if (retval < 0) {
		TPD_INFO("Failed to do identification\n");
		goto exit;
	}

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode)) {
		if (tcm_hcd->health_monitor_support) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, "bl_fw_err_mode");
		}
		TPD_INFO("Failed to run application firmware (boot status = 0x%02x)\n",
			 tcm_hcd->boot_info.status);
		if (retry) {
			retry = false;
			goto retry;
		}
		retval = -EINVAL;
		goto exit;
	} else if (tcm_hcd->app_status != APP_STATUS_OK) {
		TPD_INFO("Application status = 0x%02x\n", tcm_hcd->app_status);
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}
*/
static int syna_tcm_get_dynamic_config(struct syna_tcm_hcd *tcm_hcd,
				       enum dynamic_config_id id, unsigned short *value)
{
	int retval = 0;
	unsigned char out_buf;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;
	out_buf = (unsigned char)id;

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_GET_DYNAMIC_CONFIG,
					&out_buf,
					sizeof(out_buf),
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					RESPONSE_TIMEOUT_MS_SHORT);
	if (retval < 0 || resp_length < 2) {
		retval = -EINVAL;
		TPD_INFO("Failed to read dynamic config\n");
		goto exit;
	}

	*value = (unsigned short)le2_to_uint(resp_buf);
exit:
	kfree(resp_buf);
	return retval;
}

static int syna_tcm_set_dynamic_config(struct syna_tcm_hcd *tcm_hcd,
				       enum dynamic_config_id id, unsigned short value)
{
	int retval = 0;
	unsigned char out_buf[3];
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	TPD_DEBUG("%s:config 0x%x, value %d\n", __func__, id, value);
	resp_buf = NULL;
	resp_buf_size = 0;

	out_buf[0] = (unsigned char)id;
	out_buf[1] = (unsigned char)value;
	out_buf[2] = (unsigned char)(value >> 8);

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_SET_DYNAMIC_CONFIG,
					out_buf,
					sizeof(out_buf),
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					RESPONSE_TIMEOUT_MS_SHORT);
	if (retval < 0) {
		TPD_INFO("Failed to write command %s\n",
			 STR(CMD_SET_DYNAMIC_CONFIG));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_set_long_config(struct syna_tcm_hcd *tcm_hcd, unsigned char*buf)
{
	int retval = 0;
	char *report = NULL;
	unsigned char out_buf[AIUNIT_LONG_NUM + 1] = {0};
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0, resp_length = 0;
	unsigned int i = 0;

	TPD_DEBUG("%s:config 0x%x\n", __func__, buf[0]);

	for (i = 0; i < (AIUNIT_LONG_NUM+1); i++) {
	   out_buf[i] = buf[i];
	}

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_SET_LONG_CONFIG,
					out_buf,
					sizeof(out_buf),
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					RESPONSE_TIMEOUT_MS_SHORT);

	if (retval < 0) {
		TP_INFO(tcm_hcd->tp_index, "Failed to write command %s\n", STR(CMD_SET_LONG_CONFIG));
		report = tp_kzalloc(30, GFP_KERNEL);
		if (report) {
			tp_healthinfo_report(tcm_hcd->monitor_data, HEALTH_REPORT, report);
			tp_kfree((void **)&report);
		}
		goto exit;
	}

exit:
	tp_kfree((void **)&resp_buf);

	return retval;
}

static int syna_tcm_sleep(struct syna_tcm_hcd *tcm_hcd, bool en)
{
	int retval = 0;

	unsigned char command;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	command = en ? CMD_ENTER_DEEP_SLEEP : CMD_EXIT_DEEP_SLEEP;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = syna_tcm_write_message(tcm_hcd,
					command,
					NULL,
					0,
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					0);
	if (retval < 0) {
		TPD_INFO("Failed to write command %s\n", en ? STR(CMD_ENTER_DEEP_SLEEP):STR(CMD_EXIT_DEEP_SLEEP));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static void syna_rate_white_list_ctrl(void *chip_data, int value)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	unsigned short cmd = 1;
	int retval = 0;
	if (tcm_hcd == NULL) {
		return;
	}

	switch (value) {
	case 120: /* 120Hz */
		cmd = 1;
		break;
	case 180: /* 180Hz */
		cmd = 2;
		break;
	case 240: /* 240Hz */
		cmd = 3;
		break;
	default:
		TPD_INFO("%s: report rate not support\n", __func__);
		return;
	}
	retval = syna_tcm_set_dynamic_config(tcm_hcd,
					     DC_SET_REPORT_FRE,
					     cmd);
	if (retval < 0) {
		TPD_INFO("Failed to set dynamic report frequence config\n");
	}
	TPD_INFO("%s: DC_SET_REPORT_FRE: %d  %s!\n",
		 __func__, cmd, retval < 0 ? "failed" : "success");
}

static int syna_tcm_reset(void *chip_data)
{
	return 0;
}

static int syna_get_chip_info(void *chip_data)
{
	return 0;
}

static int syna_get_vendor(void *chip_data, struct panel_info *panel_data)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;

	tcm_hcd->ihex_name = panel_data->extra;
	tcm_hcd->limit_name = panel_data->test_limit_name;
	tcm_hcd->fw_name = panel_data->fw_name;
	return 0;
}

static u32 syna_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	tcm_hcd->trigger_reason = 0;

	if (tcm_hcd->zeroflash_init_done == 0) {
		TPD_INFO("hdl not ready, disable irq\n");
		disable_irq_nosync(tcm_hcd->s_client->irq);
		return retval;
	}

	syna_tcm_stop_reset_timer(tcm_hcd);

	retval =  syna_tcm_read_message(tcm_hcd, NULL, 0);
	if (retval == -ENXIO) {
		TPD_INFO("Failed to read message, start to do hdl\n");
	}

	return tcm_hcd->trigger_reason;
}

static int syna_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
	unsigned int idx;
	unsigned int status;
	struct object_data *object_data;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	struct touch_hcd *touch_hcd = tcm_hcd->touch_hcd;
	unsigned int obj_attention = 0x00;


	if (points == NULL) {
		return obj_attention;
	}
	object_data = touch_hcd->touch_data.object_data;

	for (idx = 0; idx < touch_hcd->max_objects; idx++) {
		status = object_data[idx].status;
		if (status != LIFT) {
			obj_attention |= (0x1 << idx);
		} else {
			if ((~obj_attention) & ((0x1) << idx)) {
				continue;
			} else {
				obj_attention &= (~(0x1 << idx));
			}
		}

		points[idx].x = object_data[idx].x_pos;
		points[idx].y = object_data[idx].y_pos;
		points[idx].z = (object_data[idx].x_width + object_data[idx].y_width) / 2;
		points[idx].width_major = (object_data[idx].x_width + object_data[idx].y_width) / 2;
		points[idx].touch_major = (object_data[idx].x_width + object_data[idx].y_width) / 2;
		points[idx].status = 1;
	}

	return obj_attention;
}

static int syna_tcm_sw_reset(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_RESET,
					NULL,
					0,
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					RESPONSE_TIMEOUT_MS_SHORT);
	if (retval < 0) {
		retval = -EINVAL;
		TPD_INFO("Failed to do sw reset\n");
	}

	kfree(resp_buf);
	return retval;
}

static int syna_tcm_before_switch_to_gesture_mode(struct syna_tcm_hcd *tcm_hcd, bool enable)
{

	if (enable) {
		/*set gesture fw mode flag = 1*/
		tcm_hcd->request_fw_image_id = 1;
		/*sw reset*/
		syna_tcm_sw_reset(tcm_hcd);

		msleep(50);
		/*wait hostdownload done*/
		tp_wait_hdl_finished();

		tcm_hcd->request_fw_image_id = 0;
	}

	return 0;
}

static int syna_tcm_set_gesture_mode(struct syna_tcm_hcd *tcm_hcd, bool enable)
{
	int retval = 0;
	unsigned short config;
	struct touchpanel_data *ts = spi_get_drvdata(tcm_hcd->s_client);

	if (ts->lpwg_fw_support) {
		/*request lpwg firmware*/
		syna_tcm_before_switch_to_gesture_mode(tcm_hcd, enable);
	}

	/*this command may take too much time, if needed can add flag to skip this */
	retval = syna_tcm_get_dynamic_config(tcm_hcd, DC_IN_WAKEUP_GESTURE_MODE, &config);
	if (retval < 0) {
		TPD_INFO("Failed to get dynamic config\n");
		return retval;
	}

	TPD_DEBUG("config id is %d, enable: %d\n", config, enable);

	if (enable) {
		if (!config) {
			retval = syna_set_input_reporting(tcm_hcd, true);
			if (retval < 0) {
				TPD_INFO("Failed to set input reporting\n");
				return retval;
			}

			retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_IN_WAKEUP_GESTURE_MODE, true);
			if (retval < 0) {
				TPD_INFO("Failed to set dynamic gesture config\n");
				return retval;
			}
			if (ts->incell_aod_gesture_support) {
				retval = syna_tcm_set_dynamic_config(tcm_hcd,
								DC_GESTURE_MASK,
								0xFFFF);
				if (retval < 0) {
					TPD_INFO("%s: Failed to set dynamic gesture mask config\n", __func__);
					return retval;
				}
			}
		}
	}

	/*set to sleep*/
	if (config) {
		retval = syna_tcm_sleep(tcm_hcd, !enable);
		if (retval < 0) {
			TPD_INFO("Failed to set sleep mode");
		}
	}

	return retval;
}

static int syna_tcm_set_aod_mode(struct syna_tcm_hcd *tcm_hcd, bool enable)
{
	int retval = 0;
	unsigned short config;
	struct touchpanel_data *ts = spi_get_drvdata(tcm_hcd->s_client);

	if (ts != NULL && ts->lpwg_fw_support) {
		/*request lpwg firmware*/
		syna_tcm_before_switch_to_gesture_mode(tcm_hcd, enable);
	}

	retval = syna_tcm_get_dynamic_config(tcm_hcd, DC_IN_WAKEUP_GESTURE_MODE, &config);
	if (retval < 0) {
		TPD_INFO("Failed to get dynamic config\n");
		return retval;
	}

	TPD_DEBUG("config id is %d, enable: %d\n", config, enable);
	TPD_DEBUG("aod enable: %d\n", enable);

	if (enable) {
		if (!config) {
			retval = syna_set_input_reporting(tcm_hcd, true);
			if (retval < 0) {
				TPD_INFO("Failed to set input reporting\n");
				return retval;
			}
			retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_IN_WAKEUP_GESTURE_MODE, true);
			if (retval < 0) {
				TPD_INFO("Failed to set dynamic aod gesture config\n");
				return retval;
			}
			retval = syna_tcm_set_dynamic_config(tcm_hcd,
						     DC_GESTURE_MASK,
						     0xFFFF);
			if (retval < 0) {
				TPD_INFO("%s: Failed to set dynamic gesture mask config\n", __func__);
				return retval;
			}
		}
	} else {
			retval = syna_tcm_identify(tcm_hcd, true);
			if (retval < 0) {
				TPD_INFO("Failed to do identification\n");
				return retval;
			}
			retval = syna_set_input_reporting(tcm_hcd, false);
			if (retval < 0) {
				TPD_INFO("Failed to set input reporting\n");
				return retval;
			}

			retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_IN_WAKEUP_GESTURE_MODE, false);
			if (retval < 0) {
				TPD_INFO("Failed to set dynamic gesture config\n");
				return retval;
			}
			/*enable_irq(tcm_hcd->s_client->irq);*/
			TPD_INFO("%s: EXIT MODE_AOD \n", __func__);
	}
	return retval;
}

static int syna_tcm_set_glove_mode(struct syna_tcm_hcd *tcm_hcd, bool enable)
{
	unsigned short regval = 0;
	int retval = 0;

	retval = syna_tcm_get_dynamic_config(tcm_hcd, DC_GLOVE_MODE_ENABLED, &regval);
	if (retval < 0) {
		TPD_INFO("Failed to get glove config\n");
		return 0;
	}
	TPD_INFO("before edit glove mode reg_val = 0x%x\n", regval);

	if (enable) {
		retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_GLOVE_MODE_ENABLED, 1);
	}
	else {
		retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_GLOVE_MODE_ENABLED, 0);
	}
	if (retval < 0) {
		TPD_INFO("Failed to set glove config\n");
		return 0;
	}
	TPD_INFO("sucess to set glove config regval = %d\n", enable);

	retval = syna_tcm_get_dynamic_config(tcm_hcd, DC_GLOVE_MODE_ENABLED, &regval);
	if (retval < 0) {
		TPD_INFO("Failed to get glove config\n");
		return 0;
	}
	TPD_INFO("after edit glove mode reg_val=0x%x", regval);

	return retval;
}

static int syna_tcm_normal_mode(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;

	retval = syna_set_input_reporting(tcm_hcd, false);
	if (retval < 0) {
		TPD_INFO("Failed to set input reporting\n");
		return retval;
	}

	retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_IN_WAKEUP_GESTURE_MODE, false);
	if (retval < 0) {
		TPD_INFO("Failed to set dynamic gesture config\n");
		return retval;
	}

	return retval;
}

static int synaptics_corner_limit_handle(struct syna_tcm_hcd *tcm_hcd, int enable)
{
	int ret = -1;

	if (LANDSCAPE_SCREEN_90 == enable) {
		/*set area parameter*/
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_ROATE_TO_HORIZONTAL_LEVEL, 0x01);
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_GRIP_ROATE_TO_HORIZONTAL_LEVEL\n", __func__);
			return ret;
		}
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_DARKZONE_ENABLE, 0x03);
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_DARKZONE_ENABLE\n", __func__);
			return ret;
		}
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_DARKZONE_X, tcm_hcd->grip_darkzone_x);        /*x part*/
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_GRIP_DARKZONE_X\n", __func__);
			return ret;
		}
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_DARKZONE_Y, tcm_hcd->grip_darkzone_y);        /*y part*/
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_GRIP_DARKZONE_Y\n", __func__);
			return ret;
		}
		TPD_INFO("CORNER_NOTCH_LEFT mode set corner mode\n");
	} else if (LANDSCAPE_SCREEN_270 == enable) {
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_ROATE_TO_HORIZONTAL_LEVEL, 0x01);
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_GRIP_ROATE_TO_HORIZONTAL_LEVEL\n", __func__);
			return ret;
		}
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_DARKZONE_ENABLE, 0x0C);
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_DARKZONE_ENABLE\n", __func__);
			return ret;
		}
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_DARKZONE_X, tcm_hcd->grip_darkzone_x);        /*x part*/
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_GRIP_DARKZONE_X\n", __func__);
			return ret;
		}
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_DARKZONE_Y, tcm_hcd->grip_darkzone_y);        /*y part*/
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_GRIP_DARKZONE_Y\n", __func__);
			return ret;
		}
		TPD_INFO("CORNER_NOTCH_RIGHT mode set corner mode\n");
	} else if (VERTICAL_SCREEN == enable) {
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_ROATE_TO_HORIZONTAL_LEVEL, 0x00);
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_GRIP_ROATE_TO_HORIZONTAL_LEVEL\n", __func__);
			return ret;
		}
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_DARKZONE_ENABLE, 0x05);
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_DARKZONE_ENABLE\n", __func__);
			return ret;
		}
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_DARKZONE_X, tcm_hcd->grip_darkzone_v2_x);        /*x part*/
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_GRIP_DARKZONE_X\n", __func__);
			return ret;
		}
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_DARKZONE_Y, tcm_hcd->grip_darkzone_v2_y);        /*y part*/
		if (ret < 0) {
			TPD_INFO("%s:failed to set DC_GRIP_DARKZONE_Y\n", __func__);
			return ret;
		}
		TPD_INFO("CORNER_CLOSE set corner mode\n");
	}

	return ret;
}

static int synaptics_enable_edge_limit(struct syna_tcm_hcd *tcm_hcd, int enable)
{
	int ret;

	ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_SUPPRESSION_ENABLED_NEW, 1);

	if (ret < 0) {
		TPD_INFO("%s:failed to enable grip suppression\n", __func__);
		return ret;
	}

	ret = synaptics_corner_limit_handle(tcm_hcd, enable);

	return ret;
}

static int synaptics_enable_headset_mode(struct syna_tcm_hcd *tcm_hcd, bool enable)
{
	int8_t ret = -1;

	TPD_DEBUG("%s:enable = %d\n", __func__, enable);

	if (enable) {
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_HEADSET_MODE_ENABLED, 1);
		if (ret < 0) {
			TPD_INFO("%s:failed to enable headset mode\n", __func__);
			return ret;
		}
		TPD_INFO("%s:HEADSET PLUG IN\n", __func__);
	} else {
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_HEADSET_MODE_ENABLED, 0);
		if (ret < 0) {
			TPD_INFO("%s:failed to disable headset mode\n", __func__);
			return ret;
		}
		TPD_INFO("%s:HEADSET PLUG OUT\n", __func__);
	}

	return ret;
}

static void syna_aiunit_game_info(void *chip_data)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	struct touchpanel_data *ts = NULL;
	u8 cmd[MAX_AIUNIT_SET_NUM * 10 + 1] = { 0 };
	int i = 0;
	int ret = 0;
	unsigned short regval = 0;

	if (tcm_hcd == NULL || tcm_hcd->s_client == NULL) {
		TPD_INFO("%s: tcm_hcd == NULL.\n", __func__);
		return;
	}
	ts = spi_get_drvdata(tcm_hcd->s_client);

	if (ts->is_suspended) {
		return;
	}
	if (ts->aiunit_game_enable) {
		ret = syna_tcm_set_dynamic_config(tcm_hcd, SYNA_CMD_GAME_AIUINIT_EN, 1);
		ret = syna_tcm_get_dynamic_config(tcm_hcd, SYNA_CMD_GAME_AIUINIT_EN, &regval);
		if (regval == 1) {
			TPD_INFO("%s: aiunit game info enter suc.\n", __func__);
		} else {
			TPD_INFO("%s: aiunit game info enter fail.\n", __func__);
		}
	} else {
		ret = syna_tcm_set_dynamic_config(tcm_hcd, SYNA_CMD_GAME_AIUINIT_EN, 0);
		msleep(3);
		ret = syna_tcm_get_dynamic_config(tcm_hcd, SYNA_CMD_GAME_AIUINIT_EN, &regval);
		if (regval == 0) {
			TPD_INFO("%s: aiunit game info exit suc.\n", __func__);
		} else {
			TPD_INFO("%s: aiunit game info exit fail.\n", __func__);
		}
	}

	cmd[0] = SYNA_CMD_GAME_AIUINIT;
	for (i = 0; i < MAX_AIUNIT_SET_NUM; i++) {
		cmd[10 * i + 1] = ts->tp_ic_aiunit_game_info[i].gametype;
		cmd[10 * i + 2] = ts->tp_ic_aiunit_game_info[i].aiunit_game_type;
		cmd[10 * i + 3] = ts->tp_ic_aiunit_game_info[i].left & 0xff;
		cmd[10 * i + 4] = (ts->tp_ic_aiunit_game_info[i].left >> 8) & 0xff;
		cmd[10 * i + 5] = ts->tp_ic_aiunit_game_info[i].top & 0xff;
		cmd[10 * i + 6] = (ts->tp_ic_aiunit_game_info[i].top >> 8) & 0xff;
		cmd[10 * i + 7] = ts->tp_ic_aiunit_game_info[i].right & 0xff;
		cmd[10 * i + 8] = (ts->tp_ic_aiunit_game_info[i].right >> 8) & 0xff;
		cmd[10 * i + 9] = ts->tp_ic_aiunit_game_info[i].bottom & 0xff;
		cmd[10 * i + 10] = (ts->tp_ic_aiunit_game_info[i].bottom >> 8) & 0xff;
		TPD_INFO("type:%x,%x left:%x,%x top:%x,%x right:%x,%x bottom:%x,%x.", \
				cmd[10 * i + 1], cmd[10 * i + 2], \
				cmd[10 * i + 3], cmd[10 * i + 4], \
				cmd[10 * i + 5], cmd[10 * i + 6], \
				cmd[10 * i + 7], cmd[10 * i + 8], \
				cmd[10 * i + 9], cmd[10 * i + 10]);
	}

	ret = syna_tcm_set_long_config(tcm_hcd, cmd);
	if (ret < 0) {
		TPD_INFO("fts tp aiunit game write fail");
	}
}

static int synaptics_enable_game_mode(struct syna_tcm_hcd *tcm_hcd, bool enable)
{
	int8_t ret = -1;

	TPD_DEBUG("%s:enable = %d\n", __func__, enable);

	if (enable) {
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GAME_MODE_ENABLED, 1);
		if (ret < 0) {
			TPD_INFO("%s:failed to enable game mode\n", __func__);
			return ret;
		}
	} else {
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GAME_MODE_ENABLED, 0);
		if (ret < 0) {
			TPD_INFO("%s:failed to disable game mode\n", __func__);
			return ret;
		}
	}

	return ret;
}

static int synaptics_enable_waterproof_mode(struct syna_tcm_hcd *tcm_hcd, bool enable)
{
	int8_t ret = -1;

	TPD_DEBUG("%s:enable = %d\n", __func__, enable);

	if (enable) {
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_WATERPROOF_ENABLE, 1);
		if (ret < 0) {
			TPD_INFO("%s:failed to enable waterproof mode\n", __func__);
			return ret;
		}
	} else {
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_WATERPROOF_ENABLE, 0);
		if (ret < 0) {
			TPD_INFO("%s:failed to disable waterproof mode\n", __func__);
			return ret;
		}
	}

	return ret;
}

/* void tp_wait_hdl_finished(void); */

static int syna_mode_switch(void *chip_data, work_mode mode, int flag)
{
	int ret = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(tcm_hcd->s_client);
	if(!tcm_hcd->tp_irq_state) {
		TPD_INFO("tp irq disabled, skip switch mode.\n");
		return 0;
	}
	if (ts != NULL) {
		if (ts->is_suspended == 0) {
			if (ts->incell_aod_gesture_support && MODE_INCELL_AOD == mode) {
					TPD_INFO("enter aod mode switch\n");
					atomic_set(&tcm_hcd->host_downloading, 0);
					/*syna_tcm_hdl_done(tcm_hcd);*/
					enable_irq(tcm_hcd->s_client->irq);
					/*g_tcm_hcd->hdl_finished_flag = 1;*/
					complete(&tcm_hcd->config_complete);
			}
			msleep(100);
			tp_wait_hdl_finished();
		}
	}

	TPD_INFO("syna_mode_switch begin, mode = %d\n", mode);
	switch (mode) {
	case MODE_NORMAL:
		TPD_DETAIL("syna_mode_switch MODE_NORMAL\n");
		/*ret = syna_tcm_normal_mode(tcm_hcd);*/
		if (ret < 0) {
			TPD_INFO("normal mode switch failed\n");
		}
		break;
	case MODE_GESTURE:
		ret = syna_tcm_set_gesture_mode(tcm_hcd, flag);
		if (ret < 0) {
			TPD_INFO("%s:Failed to set gesture mode\n", __func__);
		}
		break;
	case MODE_INCELL_AOD:
		ret = syna_tcm_set_aod_mode(tcm_hcd, flag);
		if (ret < 0) {
			TPD_INFO("%s:Failed to set aod mode\n", __func__);
		}
		break;
	case MODE_GLOVE:
		TPD_INFO("%s: force glove_mode.\n", __func__);
		if (ts != NULL && !ts->is_suspended) {
			TPD_INFO("%s: %s force glove_mode.\n", __func__, flag ? "1" : "0");
			ret = syna_tcm_set_glove_mode(tcm_hcd, flag);
			if (ret < 0) {
				TPD_INFO("%s:Failed to set glove mode\n", __func__);
			}
		}
		break;
	case MODE_SLEEP:
		ret = syna_tcm_sleep(tcm_hcd, true);
		if (ret < 0) {
			TPD_INFO("%s: failed to switch to sleep", __func__);
		}
		break;
	case MODE_CHARGE:
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_CHARGER_CONNECTED, flag?1:0);
		if (ret < 0) {
			TPD_INFO("%s:failed to set charger mode\n", __func__);
		}
		break;

	case MODE_HEADSET:
		ret = synaptics_enable_headset_mode(tcm_hcd, flag);
		if (ret < 0) {
			TPD_INFO("%s: enable headset mode : %d failed\n", __func__, flag);
		}
		break;

	case MODE_GAME:
		ret = synaptics_enable_game_mode(tcm_hcd, flag);
		if (ret < 0) {
			TPD_INFO("%s: enable game mode : %d failed\n", __func__, flag);
		}
		break;

	case MODE_WATERPROOF:
		ret = synaptics_enable_waterproof_mode(tcm_hcd, flag);
		if (ret < 0) {
			TPD_INFO("%s: enable waterproof mode : %d failed\n", __func__, flag);
		}
		break;

	case MODE_EDGE:
		/*ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_GRIP_SUPPRESSION_ENABLED, flag?1:0);
		//if (ret < 0) {
		//    TPD_INFO("%s:failed to set grip suppression\n", __func__);
		}*/
		ret = synaptics_enable_edge_limit(tcm_hcd, flag);
		if (ret < 0) {
			TPD_INFO("%s: synaptics enable edg limit failed.\n", __func__);
		}
		break;
	case MODE_UNDERWATER:
		ret = syna_tcm_set_dynamic_config(tcm_hcd, DC_UNDER_WATER, flag?1:0);
		if (ret < 0) {
			TPD_INFO("%s:failed to set under_water mode\n", __func__);
		}
		break;
	default:
		break;
	}
	return 0;
}

static int syna_ftm_process(void *chip_data)
{
	TPD_INFO("%s: go into sleep\n", __func__);
	syna_reset_gpio(chip_data, false);
	/*syna_get_chip_info(chip_data);
	syna_mode_switch(chip_data, MODE_SLEEP, true);*/
	return 0;
}

static int  syna_tcm_reinit_device(void *chip_data)
{
	/*complete_all(&response_complete);
	  complete_all(&report_complete);*/

	return 0;
}

static int syna_hw_reset(struct syna_tcm_hcd *tcm_hcd, struct hw_resource *hw_res)
{
	if (gpio_is_valid(hw_res->reset_gpio)) {
		TPD_INFO("hardware reset: %d\n", hw_res->reset_gpio);
		gpio_set_value(hw_res->reset_gpio, false);
		msleep(20);
		gpio_set_value(hw_res->reset_gpio, true);
		msleep(200);
		return 0;
	}

	return -EINVAL;
}

int syna_reset_gpio(void *chip_data, bool enable)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;

	TPD_DEBUG("000 %s:gpio enable %d\n", __func__, enable);
	if (gpio_is_valid(tcm_hcd->hw_res->reset_gpio)) {
		gpio_set_value(tcm_hcd->hw_res->reset_gpio, enable);
	}
	return 0;
}

static int syna_power_control(void *chip_data, bool enable)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;

	TPD_DEBUG("%s: %d\n", __func__, enable);

	return syna_hw_reset(tcm_hcd, tcm_hcd->hw_res);
}

static fw_check_state syna_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	char *fw_ver = NULL;

	if (!tcm_hcd->tp_fw_update_headfile) {
		retval = wait_for_completion_timeout(&tcm_hcd->config_complete, msecs_to_jiffies(RESPONSE_TIMEOUT_MS_LONG * 2));
		if (retval == 0) {
			TPD_INFO("Timed out waiting for response config_complete\n");
		}
	}

	TPD_INFO("fw id %d, custom config id 0x%s\n", panel_data->tp_fw, tcm_hcd->app_info.customer_config_id);

	if (strlen(tcm_hcd->app_info.customer_config_id) == 0) {
		return FW_NORMAL;
	}

	fw_ver = kzalloc(strlen(tcm_hcd->app_info.customer_config_id)+1, GFP_KERNEL);
	memcpy(fw_ver, tcm_hcd->app_info.customer_config_id, strlen(tcm_hcd->app_info.customer_config_id));
	fw_ver[strlen(tcm_hcd->app_info.customer_config_id)] = '\0';

	panel_data->tp_fw = le4_to_uint(tcm_hcd->id_info.build_id);
	if (panel_data->tp_fw == 0) {
		kfree(fw_ver);
		return FW_NORMAL;
	}

	if (panel_data->manufacture_info.version) {
		sprintf(panel_data->manufacture_info.version, "0x%s", fw_ver);
	}

	kfree(fw_ver);

	return FW_NORMAL;
}

void syna_fw_version_update(void *chip_data)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(tcm_hcd->s_client);
	char *fw_ver = NULL;

        tcm_hcd->app_info.customer_config_id[9] = '\0';

	if (strlen(tcm_hcd->app_info.customer_config_id) == 0) {
		return;
	}

	fw_ver = kzalloc(strlen(tcm_hcd->app_info.customer_config_id)+1, GFP_KERNEL);
	memcpy(fw_ver, tcm_hcd->app_info.customer_config_id, strlen(tcm_hcd->app_info.customer_config_id));
	fw_ver[strlen(tcm_hcd->app_info.customer_config_id)] = '\0';

	ts->panel_data.tp_fw = le4_to_uint(tcm_hcd->id_info.build_id);
	if (ts->panel_data.tp_fw == 0) {
		kfree(fw_ver);
		return;
	}

	if (ts->panel_data.manufacture_info.version) {
		sprintf(ts->panel_data.manufacture_info.version, "0x%s", fw_ver);
	}

	TPD_DETAIL("Update fw id %d, custom config id 0x%s\n", ts->panel_data.tp_fw, fw_ver);

	kfree(fw_ver);

	return;
}
/*
static int syna_tcm_async_work(void *chip_data)
{
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;

	if (IS_FW_MODE(tcm_hcd->id_info.mode)) {
		retval = syna_tcm_identify(tcm_hcd, false);
		if (retval < 0) {
			TPD_INFO("Failed to do identification\n");
			return retval;
		}
	}
	//syna_set_trigger_reason(tcm_hcd, IRQ_FW_AUTO_RESET);
	return 0;
}*/

static fw_update_state copy_fw_to_buffer(struct syna_tcm_hcd *tcm_hcd, const struct firmware *fw)
{
	struct firmware *tp_fw;
	if (fw) {
		/*free already exist fw data buffer*/
		if (tcm_hcd->zeroflash_hcd->fw_entry) {
			if(!tcm_hcd->tp_fw_update_headfile) {
				vfree(tcm_hcd->zeroflash_hcd->fw_entry->data);
			}
			vfree(tcm_hcd->zeroflash_hcd->fw_entry);
		}

		tp_fw = vmalloc(sizeof(struct firmware));
		if(!tp_fw) {
			TPD_INFO("vmalloc tp firmware error\n");
			goto exit;
		}
		tp_fw->data = vmalloc(fw->size);
		if(!tp_fw->data) {
			TPD_INFO("vmalloc tp firmware data error\n");
			goto exit;
		}
		memcpy((u8 *)tp_fw->data, (u8 *)(fw->data), fw->size);
		tp_fw->size = fw->size;

		tcm_hcd->zeroflash_hcd->fw_entry = tp_fw;
		tcm_hcd->tp_fw_update_headfile = false;
		TPD_INFO("copy fw to buffer success.\n");
	}
	else {
		TPD_INFO("failed to get oplus tp firmware.\n");
		return FW_UPDATE_ERROR;
	}
	return FW_UPDATE_ERROR;

exit:
	if(tp_fw) {
		vfree(tp_fw);
	}
	return FW_UPDATE_ERROR;
}

extern int try_to_recovery_ic(struct syna_tcm_hcd *tcm_hcd, char *iHex);

static fw_update_state syna_tcm_fw_update(void *chip_data, const struct firmware *fw, bool force)
{
	int ret = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	TPD_DEBUG("syna_tcm_fw_update begin\n");

	ret = copy_fw_to_buffer(tcm_hcd, fw);
	tcm_hcd->tp_fw_update_parse = true;

	syna_reset_gpio(tcm_hcd, false);
	msleep(2);
	syna_reset_gpio(tcm_hcd, true);
	msleep(2);

	msleep(100);
	tp_wait_hdl_finished();
	TPD_DEBUG("syna_tcm_fw_update end\n");

	return ret;
}

static int syna_get_gesture_info(void *chip_data, struct gesture_info *gesture)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	struct touch_hcd *touch_hcd = tcm_hcd->touch_hcd;
	struct touch_data *touch_data = &touch_hcd->touch_data;

	gesture->clockwise = 2;
	switch (touch_data->lpwg_gesture) {
	case DTAP_DETECT:
		gesture->gesture_type = DOU_TAP;
		break;
	case STAP_DETECT:
		gesture->gesture_type = SINGLE_TAP;
		break;
	case CIRCLE_DETECT:
		gesture->gesture_type = CIRCLE_GESTURE;
		if (touch_data->extra_gesture_info == 0x10) {
			gesture->clockwise = 0;
		} else if (touch_data->extra_gesture_info == 0x20) {
			gesture->clockwise = 1;
		}
		break;
	case SWIPE_DETECT:
		if (touch_data->extra_gesture_info == 0x41) {
			gesture->gesture_type = LEFT2RIGHT_SWIP;
		} else if (touch_data->extra_gesture_info == 0x42) {
			gesture->gesture_type = RIGHT2LEFT_SWIP;
		} else if (touch_data->extra_gesture_info == 0x44) {
			gesture->gesture_type = UP2DOWN_SWIP;
		} else if (touch_data->extra_gesture_info == 0x48) {
			gesture->gesture_type = DOWN2UP_SWIP;
		} else if (touch_data->extra_gesture_info == 0x81) {
			gesture->gesture_type = DOU_SWIP;
		} else if (touch_data->extra_gesture_info == 0x82) {
			gesture->gesture_type = DOU_SWIP;
		} else if (touch_data->extra_gesture_info == 0x84) {
			gesture->gesture_type = DOU_SWIP;
		} else if (touch_data->extra_gesture_info == 0x88) {
			gesture->gesture_type = DOU_SWIP;
		}
		break;
	case UNICODE_DETECT:
		if (touch_data->extra_gesture_info == 0x6d) {
			gesture->gesture_type = M_GESTRUE;

		} else if (touch_data->extra_gesture_info == 0x77) {
			gesture->gesture_type = W_GESTURE;
		}
		break;
	case VEE_DETECT:
		if (touch_data->extra_gesture_info == 0x02) {
			gesture->gesture_type = UP_VEE;
		} else if (touch_data->extra_gesture_info == 0x01) {
			gesture->gesture_type = DOWN_VEE;
		} else if (touch_data->extra_gesture_info == 0x08) {
			gesture->gesture_type = LEFT_VEE;
		} else if (touch_data->extra_gesture_info == 0x04) {
			gesture->gesture_type = RIGHT_VEE;
		}
		break;
	case TRIANGLE_DETECT:
	default:
		TPD_DEBUG("not support\n");
		break;
	}
	if (gesture->gesture_type != UNKOWN_GESTURE) {
		gesture->Point_start.x = (touch_data->data_point[0] | (touch_data->data_point[1] << 8));
		gesture->Point_start.y = (touch_data->data_point[2] | (touch_data->data_point[3] << 8));
		gesture->Point_end.x    = (touch_data->data_point[4] | (touch_data->data_point[5] << 8));
		gesture->Point_end.y    = (touch_data->data_point[6] | (touch_data->data_point[7] << 8));
		gesture->Point_1st.x    = (touch_data->data_point[8] | (touch_data->data_point[9] << 8));
		gesture->Point_1st.y    = (touch_data->data_point[10] | (touch_data->data_point[11] << 8));
		gesture->Point_2nd.x    = (touch_data->data_point[12] | (touch_data->data_point[13] << 8));
		gesture->Point_2nd.y    = (touch_data->data_point[14] | (touch_data->data_point[15] << 8));
		gesture->Point_3rd.x    = (touch_data->data_point[16] | (touch_data->data_point[17] << 8));
		gesture->Point_3rd.y    = (touch_data->data_point[18] | (touch_data->data_point[19] << 8));
		gesture->Point_4th.x    = (touch_data->data_point[20] | (touch_data->data_point[21] << 8));
		gesture->Point_4th.y    = (touch_data->data_point[22] | (touch_data->data_point[23] << 8));
	}

	TPD_DEBUG("lpwg:0x%x, extra:%x type:%d\n", touch_data->lpwg_gesture,
		  touch_data->extra_gesture_info, gesture->gesture_type);

	touch_data->lpwg_gesture = 0;
	touch_data->extra_gesture_info = 0;

	return 0;
}

static void store_to_file(void *fp, size_t max_count,
			  size_t *pos, char *format, ...)
{
	va_list args;
	char buf[64] = {0};

	va_start(args, format);
	vsnprintf(buf, 64, format, args);
	va_end(args);

	if (!IS_ERR_OR_NULL(fp)) {
		tp_test_write(fp, max_count, buf, strlen(buf), pos);
	}
}

static int testing_run_prod_test_item(struct syna_tcm_hcd *tcm_info,
					  enum test_item_bit test_code)
{
	int retval = 0;
	struct syna_tcm_test *test_hcd = tcm_info->test_hcd;

/*	if (tcm_info->id_info.mode != MODE_APPLICATION
		|| tcm_info->app_status != APP_STATUS_OK) {
		TPD_INFO("Application firmware not running\n");
		return -ENODEV;
	}
*/
	LOCK_BUFFER(test_hcd->test_out);

	retval = syna_tcm_alloc_mem(tcm_info, &test_hcd->test_out, 1);

	if (retval < 0) {
		TPD_INFO("Failed to allocate memory for test_hcd->test_out.buf\n");
		UNLOCK_BUFFER(test_hcd->test_out);
		return retval;
	}

	test_hcd->test_out.buf[0] = test_code;

	LOCK_BUFFER(test_hcd->test_resp);
	retval = syna_tcm_write_message(tcm_info,
					CMD_PRODUCTION_TEST,
					test_hcd->test_out.buf,
					1,
					&test_hcd->test_resp.buf,
					&test_hcd->test_resp.buf_size,
					&test_hcd->test_resp.data_length,
					RESPONSE_TIMEOUT_MS_LONG);

	if (retval < 0) {
		TPD_INFO("Failed to write command %s\n", STR(CMD_PRODUCTION_TEST));
		UNLOCK_BUFFER(test_hcd->test_resp);
		UNLOCK_BUFFER(test_hcd->test_out);
		return retval;
	}

	UNLOCK_BUFFER(test_hcd->test_resp);
	UNLOCK_BUFFER(test_hcd->test_out);

	return 0;
}
static int syna_black_screen_test_noise(struct seq_file *s, void *chip_data,
				   struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int16_t data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	struct syna_tcm_test *test_hcd = tcm_info->test_hcd;
	unsigned char *buf = NULL;


	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);

	} else {
		TPD_INFO("raw cap test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		error_count++;
		return error_count;
	}

	ret = testing_run_prod_test_item(tcm_info, TYPE_DELTA_NOISE);

	if (ret < 0) {
		TPD_INFO("run raw cap test failed.\n");

		error_count++;
		return error_count;
	}

	LOCK_BUFFER(test_hcd->test_resp);
	buf = test_hcd->test_resp.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_hcd->test_resp.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "raw_cap:");

	for (i = 0; i < test_hcd->test_resp.data_length;) {
		index = i / byte_cnt;
		data16 = (buf[i] | (buf[i + 1] << 8));

		if (0 == index % (syna_testdata->rx_num))
			store_to_file(syna_testdata->fp, syna_testdata->length,
				      syna_testdata->pos, "\n");

		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%04d, ", data16);

		if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			TPD_INFO("rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
				 p_mutual_n[index], p_mutual_p[index]);


			error_count++;
		}

		i += byte_cnt;
	}

	UNLOCK_BUFFER(test_hcd->test_resp);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

static int syna_black_screen_test_dynamic(struct seq_file *s, void *chip_data,
				   struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int16_t data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	struct syna_tcm_test *test_hcd = tcm_info->test_hcd;
	unsigned char *buf = NULL;

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);

	} else {
		TPD_INFO("raw cap test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		error_count++;
		return error_count;
	}

	ret = testing_run_prod_test_item(tcm_info, TYPE_DYNAMIC_RANGE);

	if (ret < 0) {
		TPD_INFO("run raw cap test failed.\n");

		error_count++;
		return error_count;
	}

	LOCK_BUFFER(test_hcd->test_resp);
	buf = test_hcd->test_resp.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_hcd->test_resp.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "raw_cap:");

	for (i = 0; i < test_hcd->test_resp.data_length;) {
		index = i / byte_cnt;
		data16 = (buf[i] | (buf[i + 1] << 8));

		if (0 == index % (syna_testdata->rx_num))
			store_to_file(syna_testdata->fp, syna_testdata->length,
				      syna_testdata->pos, "\n");

		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%04d, ", data16);

		if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			TPD_INFO("rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
				 p_mutual_n[index], p_mutual_p[index]);

			error_count++;
		}

		i += byte_cnt;
	}

	UNLOCK_BUFFER(test_hcd->test_resp);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

static int synaptics_auto_test_preoperation(struct seq_file *s, void *chip_data,
		struct auto_testdata *syna_testdata,
		struct test_item_info *p_test_item_info)
{
	int ret = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(tcm_hcd->s_client);
	const struct firmware *fw = NULL;
	char *fw_name_test = NULL;
	char *p_node = NULL;
	char *postfix = "_TEST.img";
	uint8_t copy_len = 0;
	ts->lpwg_fw_support = false;
	TPD_INFO("%s  is called\n", __func__);

	fw_name_test = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
		if (fw_name_test == NULL) {
			TPD_INFO("fw_name_test kzalloc error!\n");
			return -ENOMEM;
		}

	p_node = strstr(ts->panel_data.fw_name, ".");
	copy_len = p_node - ts->panel_data.fw_name;
	memcpy(fw_name_test, ts->panel_data.fw_name, copy_len);
	strlcat(fw_name_test, postfix, MAX_FW_NAME_LENGTH);

	ret = request_firmware(&fw, fw_name_test, ts->dev);
	if (!ret) {
		ts->loading_fw = true;
			if (ts->ts_ops && ts->ts_ops->fw_update)
				ret = ts->ts_ops->fw_update(ts->chip_data, fw, 1);
			ts->loading_fw = false;
	} else {
		TPD_INFO("request_firmware(%s) fail and no need to download test fw !\n", fw_name_test);
	}

	if (fw) {
		release_firmware(fw);
		fw = NULL;
	}

	kfree(fw_name_test);

	return 0;
}

static int synaptics_auto_test_endoperation(struct seq_file *s, void *chip_data,
		struct auto_testdata *syna_testdata,
		struct test_item_info *p_test_item_info)
{
	TPD_INFO("%s  is called\n", __func__);
	return 0;
}

static int synaptics_auto_black_screen_test_preoperation(struct seq_file *s, void *chip_data,
		struct auto_testdata *syna_testdata,
		struct test_item_info *p_test_item_info)
{
	TPD_INFO("%s  is called\n", __func__);
	return 0;
}

static int synaptics_auto_black_screen_test_endoperation(struct seq_file *s, void *chip_data,
		struct auto_testdata *syna_testdata,
		struct test_item_info *p_test_item_info)
{
	int ret = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(tcm_hcd->s_client);
	const struct firmware *fw = NULL;

	TPD_INFO("%s  is called\n", __func__);
	ts->lpwg_fw_support = true;
	ret = request_firmware(&fw, ts->panel_data.fw_name, ts->dev);
	if (!ret) {
		ts->loading_fw = true;
			if (ts->ts_ops && ts->ts_ops->fw_update)
				ret = ts->ts_ops->fw_update(ts->chip_data, fw, 1);
			ts->loading_fw = false;
	} else {
		TPD_INFO("request_firmware(%s) fail and upgrade to normal  firmware fail!\n", ts->panel_data.fw_name);
	}

	if (fw) {
		release_firmware(fw);
		fw = NULL;
	}

	return 0;
}


static int syna_testing_noise(struct seq_file *s, void *chip_data,
				   struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int16_t data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	struct syna_tcm_test *test_hcd = tcm_info->test_hcd;
	unsigned char *buf = NULL;

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);

	} else {
		TPD_INFO("raw cap test limit type(%2x) is wrong.\n",
			 item_header->item_limit_type);

		if (!error_count) {
			seq_printf(s, "raw cap test limit type(%2x) is wrong.\n",
				   item_header->item_limit_type);
		}

		error_count++;
		return error_count;
	}

	ret = testing_run_prod_test_item(tcm_info, TYPE_DELTA_NOISE);

	if (ret < 0) {
		TPD_INFO("run raw cap test failed.\n");

		if (!error_count) {
			seq_printf(s, "run raw cap test failed.\n");
		}

		error_count++;
		return error_count;
	}

	LOCK_BUFFER(test_hcd->test_resp);
	buf = test_hcd->test_resp.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_hcd->test_resp.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "noise:");

	for (i = 0; i < test_hcd->test_resp.data_length;) {
		index = i / byte_cnt;
		data16 = (buf[i] | (buf[i + 1] << 8));

		if (0 == index % (syna_testdata->rx_num))
			store_to_file(syna_testdata->fp, syna_testdata->length,
				      syna_testdata->pos, "\n");

		store_to_file(syna_testdata->fp, syna_testdata->length,
			      syna_testdata->pos, "%04d, ", data16);

		if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			TPD_INFO("rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
				 p_mutual_n[index], p_mutual_p[index]);

			if (!error_count) {
				seq_printf(s, "rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
					   p_mutual_n[index], p_mutual_p[index]);
			}

			error_count++;
		}

		i += byte_cnt;
	}

	UNLOCK_BUFFER(test_hcd->test_resp);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		      syna_testdata->pos, "\n");

	return error_count;
}

static int syna_testing_pt11(struct seq_file *s, void *chip_data,
				  struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int16_t data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	struct syna_tcm_test *test_hcd = tcm_info->test_hcd;
	unsigned char *buf = NULL;

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);
	} else {
		TPD_INFO("raw cap test limit type(%2x) is wrong.\n",
		item_header->item_limit_type);

		if (!error_count) {
			seq_printf(s, "raw cap test limit type(%2x) is wrong.\n",
		item_header->item_limit_type);
	}

		error_count++;
		return error_count;
	}

	ret = testing_run_prod_test_item(tcm_info, TYPE_PT11);

	if (ret < 0) {
		TPD_INFO("run raw cap test failed.\n");

		if (!error_count) {
			seq_printf(s, "run raw cap test failed.\n");
		}

		error_count++;
		return error_count;
	}

	LOCK_BUFFER(test_hcd->test_resp);
	buf = test_hcd->test_resp.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_hcd->test_resp.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
			syna_testdata->pos, "pt11:");

	for (i = 0; i < test_hcd->test_resp.data_length;) {
		index = i / byte_cnt;
		data16 = (buf[i] | (buf[i + 1] << 8));

		if (0 == index % (syna_testdata->rx_num))
			store_to_file(syna_testdata->fp, syna_testdata->length,
				syna_testdata->pos, "\n");

		store_to_file(syna_testdata->fp, syna_testdata->length,
			syna_testdata->pos, "%04d, ", data16);

		if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			TPD_INFO("rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
				p_mutual_n[index], p_mutual_p[index]);

			if (!error_count) {
				seq_printf(s, "rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
					p_mutual_n[index], p_mutual_p[index]);
			}

		error_count++;
		}

		i += byte_cnt;
	}

	UNLOCK_BUFFER(test_hcd->test_resp);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		syna_testdata->pos, "\n");

	return error_count;
}

static int syna_get_test_max_adc_value(struct syna_tcm_hcd *tcm_hcd, unsigned short *value)
{
	int retval;
	unsigned char out_buf;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;
	out_buf = (unsigned char)0xa5;

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_PRODUCTION_TEST,
					&out_buf,
					sizeof(out_buf),
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					RESPONSE_TIMEOUT_MS_SHORT);
	if (retval < 0 || resp_length < 2) {
		retval = -EINVAL;
		TPD_INFO("Failed to read test max adc value\n");
		goto exit;
	}

	*value = (unsigned short)le2_to_uint(resp_buf);
exit:
	kfree(resp_buf);
	return retval;
}

static int syna_testing_pt17(struct seq_file *s, void *chip_data,
				  struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int16_t data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	struct syna_tcm_test *test_hcd = tcm_info->test_hcd;
	unsigned char *buf = NULL;
	unsigned short  max_adc_value = 0;

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);
	} else {
		TPD_INFO("raw cap test limit type(%2x) is wrong.\n",
		item_header->item_limit_type);

		error_count++;
		return error_count;
	}

	ret = syna_get_test_max_adc_value(tcm_info, &max_adc_value);
	if (ret < 0) {
		TPD_INFO("syna_get_test_max_adc_value failed.\n");

		error_count++;
		return error_count;
	}

	if (max_adc_value == 0) {
		TPD_INFO("wrong max adc value:%d\n", max_adc_value);

		error_count++;
		return error_count;
	}

	ret = testing_run_prod_test_item(tcm_info, TYPE_PT17);

	if (ret < 0) {
		TPD_INFO("run raw cap test failed.\n");

		error_count++;
		return error_count;
	}

	LOCK_BUFFER(test_hcd->test_resp);
	buf = test_hcd->test_resp.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_hcd->test_resp.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
			syna_testdata->pos, "pt17:");

	for (i = 0; i < test_hcd->test_resp.data_length;) {
		index = i / byte_cnt;
		data16 = (buf[i] | (buf[i + 1] << 8));
		data16 =((unsigned int)data16 * 200)/ max_adc_value;

		if (0 == index % (syna_testdata->rx_num))
			store_to_file(syna_testdata->fp, syna_testdata->length,
				syna_testdata->pos, "\n");

		store_to_file(syna_testdata->fp, syna_testdata->length,
			syna_testdata->pos, "%04d, ", data16);

		if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			TPD_INFO("rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
				p_mutual_n[index], p_mutual_p[index]);

			error_count++;
		}

		i += byte_cnt;
	}

	UNLOCK_BUFFER(test_hcd->test_resp);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		syna_testdata->pos, "\n");

	return error_count;
}

static int syna_testing_pt18(struct seq_file *s, void *chip_data,
				  struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	int16_t data16 = 0;
	int i = 0, ret = 0, index = 0, byte_cnt = 2;
	int error_count = 0;
	struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	struct auto_test_item_header *item_header = NULL;
	int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	struct syna_tcm_test *test_hcd = tcm_info->test_hcd;
	unsigned char *buf = NULL;
	unsigned short  max_adc_value = 0;

	item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);
	} else {
		TPD_INFO("raw cap test limit type(%2x) is wrong.\n",
		item_header->item_limit_type);

		error_count++;
		return error_count;
	}

	ret = syna_get_test_max_adc_value(tcm_info, &max_adc_value);
	if (ret < 0) {
		TPD_INFO("syna_get_test_max_adc_value failed.\n");

		error_count++;
		return error_count;
	}
	if (max_adc_value == 0) {
		TPD_INFO("wrong max adc value:%d\n", max_adc_value);

		error_count++;
		return error_count;
	}

	ret = testing_run_prod_test_item(tcm_info, TYPE_PT18);

	if (ret < 0) {
		TPD_INFO("run raw cap test failed.\n");

		error_count++;
		return error_count;
	}

	LOCK_BUFFER(test_hcd->test_resp);
	buf = test_hcd->test_resp.buf;
	TPD_INFO("%s read data size:%d\n", __func__, test_hcd->test_resp.data_length);
	store_to_file(syna_testdata->fp, syna_testdata->length,
			syna_testdata->pos, "pt18:");

	for (i = 0; i < test_hcd->test_resp.data_length;) {
		index = i / byte_cnt;
		data16 = (buf[i] | (buf[i + 1] << 8));
		data16 =((unsigned int)data16 * 200)/ max_adc_value;
		if (0 == index % (syna_testdata->rx_num))
			store_to_file(syna_testdata->fp, syna_testdata->length,
				syna_testdata->pos, "\n");

		store_to_file(syna_testdata->fp, syna_testdata->length,
			syna_testdata->pos, "%04d, ", data16);

		if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			TPD_INFO("rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
				p_mutual_n[index], p_mutual_p[index]);

		error_count++;
		}

		i += byte_cnt;
	}

	UNLOCK_BUFFER(test_hcd->test_resp);
	store_to_file(syna_testdata->fp, syna_testdata->length,
		syna_testdata->pos, "\n");

	msleep(100);

	return error_count;
}

static int syna_testing_Doze_noise(struct seq_file *s, void *chip_data,
					struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	 int16_t data16 = 0;
	 int i = 0, ret = 0, index = 0, byte_cnt = 2;
	 int error_count = 0;
	 struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	 struct auto_test_item_header *item_header = NULL;
	 int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	 struct syna_tcm_test *test_hcd = tcm_info->test_hcd;
	 unsigned char *buf = NULL;

	 item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	 if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		 p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		 p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);

	 } else {
		 TPD_INFO("raw cap test limit type(%2x) is wrong.\n",
			  item_header->item_limit_type);

		 if (!error_count) {
			 seq_printf(s, "raw cap test limit type(%2x) is wrong.\n",
					item_header->item_limit_type);
		 }

		 error_count++;
		 return error_count;
	 }

	 ret = testing_run_prod_test_item(tcm_info, TYPE_NOISE_DOZE);

	 if (ret < 0) {
		 TPD_INFO("run raw cap test failed.\n");

		 if (!error_count) {
			 seq_printf(s, "run raw cap test failed.\n");
		 }

		 error_count++;
		 return error_count;
	 }

	 LOCK_BUFFER(test_hcd->test_resp);
	 buf = test_hcd->test_resp.buf;
	 TPD_INFO("%s read data size:%d\n", __func__, test_hcd->test_resp.data_length);
	 store_to_file(syna_testdata->fp, syna_testdata->length,
			   syna_testdata->pos, "raw_cap:");

	 for (i = 0; i < test_hcd->test_resp.data_length;) {
		 index = i / byte_cnt;
		 data16 = (buf[i] | (buf[i + 1] << 8));

		 if (0 == index % (syna_testdata->rx_num))
			 store_to_file(syna_testdata->fp, syna_testdata->length,
					   syna_testdata->pos, "\n");

		 store_to_file(syna_testdata->fp, syna_testdata->length,
				   syna_testdata->pos, "%04d, ", data16);

		 if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			 TPD_INFO("rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
				  p_mutual_n[index], p_mutual_p[index]);

			 if (!error_count) {
				 seq_printf(s, "rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
						p_mutual_n[index], p_mutual_p[index]);
			 }

			 error_count++;
		 }

		 i += byte_cnt;
	 }

	 UNLOCK_BUFFER(test_hcd->test_resp);
	 store_to_file(syna_testdata->fp, syna_testdata->length,
			   syna_testdata->pos, "\n");

	 return error_count;
}

static int syna_testing_dynamic_range(struct seq_file *s, void *chip_data,
					struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	 int16_t data16 = 0;
	 int i = 0, ret = 0, index = 0, byte_cnt = 2;
	 int error_count = 0;
	 struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	 struct auto_test_item_header *item_header = NULL;
	 int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	 struct syna_tcm_test *test_hcd = tcm_info->test_hcd;
	 unsigned char *buf = NULL;

	 item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	 if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		 p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		 p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);

	 } else {
		 TPD_INFO("raw cap test limit type(%2x) is wrong.\n",
			  item_header->item_limit_type);

		 if (!error_count) {
			 seq_printf(s, "raw cap test limit type(%2x) is wrong.\n",
					item_header->item_limit_type);
		 }

		 error_count++;
		 return error_count;
	 }

	 ret = testing_run_prod_test_item(tcm_info, TYPE_DYNAMIC_RANGE);

	 if (ret < 0) {
		 TPD_INFO("run raw cap test failed.\n");

		 if (!error_count) {
			 seq_printf(s, "run raw cap test failed.\n");
		 }

		 error_count++;
		 return error_count;
	 }

	 LOCK_BUFFER(test_hcd->test_resp);
	 buf = test_hcd->test_resp.buf;
	 TPD_INFO("%s read data size:%d\n", __func__, test_hcd->test_resp.data_length);
	 store_to_file(syna_testdata->fp, syna_testdata->length,
			   syna_testdata->pos, "raw_cap:");

	   for (i = 0; i < test_hcd->test_resp.data_length;) {
		   index = i / byte_cnt;
		   data16 = (buf[i] | (buf[i + 1] << 8));

		   if (0 == index % (syna_testdata->rx_num))
			   store_to_file(syna_testdata->fp, syna_testdata->length,
						 syna_testdata->pos, "\n");

		   store_to_file(syna_testdata->fp, syna_testdata->length,
					 syna_testdata->pos, "%04d, ", data16);

		   if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			   TPD_INFO("rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
					p_mutual_n[index], p_mutual_p[index]);

			   if (!error_count) {
				   seq_printf(s, "rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
						  p_mutual_n[index], p_mutual_p[index]);
			   }
			   error_count++;
		   }
		   i += byte_cnt;
	   }


	 UNLOCK_BUFFER(test_hcd->test_resp);
	 store_to_file(syna_testdata->fp, syna_testdata->length,
			   syna_testdata->pos, "\n");

	 return error_count;
}

static int syna_testing_Doze_dynamic_range(struct seq_file *s, void *chip_data,
					struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	 int16_t data16 = 0;
	 int i = 0, ret = 0, index = 0, byte_cnt = 2;
	 int error_count = 0;
	 struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	 struct auto_test_item_header *item_header = NULL;
	 int32_t *p_mutual_p = NULL, *p_mutual_n = NULL;
	 struct syna_tcm_test *test_hcd = tcm_info->test_hcd;
	 unsigned char *buf = NULL;

	 item_header = (struct auto_test_item_header *)(syna_testdata->fw->data + p_test_item_info->item_offset);
	 if (item_header->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
		 p_mutual_p = (int32_t *)(syna_testdata->fw->data + item_header->top_limit_offset);
		 p_mutual_n = (int32_t *)(syna_testdata->fw->data + item_header->floor_limit_offset);

	 } else {
		 TPD_INFO("raw cap test limit type(%2x) is wrong.\n",
			  item_header->item_limit_type);

		 if (!error_count) {
			 seq_printf(s, "raw cap test limit type(%2x) is wrong.\n",
					item_header->item_limit_type);
		}

		 error_count++;
		 return error_count;
	}

	 ret = testing_run_prod_test_item(tcm_info, TYPE_DYNAMIC_RANGE_DOZE);

	 if (ret < 0) {
		TPD_INFO("run raw cap test failed.\n");

		 if (!error_count) {
			 seq_printf(s, "run raw cap test failed.\n");
		 }

		 error_count++;
		 return error_count;
	 }

	 LOCK_BUFFER(test_hcd->test_resp);
	 buf = test_hcd->test_resp.buf;
	 TPD_INFO("%s read data size:%d\n", __func__, test_hcd->test_resp.data_length);
	 store_to_file(syna_testdata->fp, syna_testdata->length,
				syna_testdata->pos, "Doze_dynamic:");

	for (i = 0; i < test_hcd->test_resp.data_length;) {
		   index = i / byte_cnt;
		   data16 = (buf[i] | (buf[i + 1] << 8));

		   if (0 == index % (syna_testdata->rx_num))
			   store_to_file(syna_testdata->fp, syna_testdata->length,
						 syna_testdata->pos, "\n");

		   store_to_file(syna_testdata->fp, syna_testdata->length,
					 syna_testdata->pos, "%04d, ", data16);

		   if ((data16 < p_mutual_n[index]) || (data16 > p_mutual_p[index])) {
			   TPD_INFO("rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
					p_mutual_n[index], p_mutual_p[index]);

			   if (!error_count) {
				   seq_printf(s, "rawcap test failed at node[%d]=%d [%d %d].\n", index, data16,
						  p_mutual_n[index], p_mutual_p[index]);
			   }

			   error_count++;
		   }
		   i += byte_cnt;
	   }

	 UNLOCK_BUFFER(test_hcd->test_resp);
	 store_to_file(syna_testdata->fp, syna_testdata->length,
			   syna_testdata->pos, "\n");
	return error_count;
}

static int syna_testing_Doze_dynamic_range_NULL(struct seq_file *s, void *chip_data,
					struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	return 0;
}
static int syna_testing_dynamic_range_NULL(struct seq_file *s, void *chip_data,
					struct auto_testdata *syna_testdata, struct test_item_info *p_test_item_info)
{
	return 0;
}

static int syna_tcm_collect_reports(struct syna_tcm_hcd *tcm_hcd, enum report_type report_type, unsigned int num_of_reports)
{
	int retval = 0;
	bool completed = false;
	unsigned int timeout;
	struct syna_tcm_test *test_hcd = tcm_hcd->test_hcd;
	unsigned char out[2] = {0};
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;

	test_hcd->report_index = 0;
	test_hcd->report_type = report_type;
	test_hcd->num_of_reports = num_of_reports;

	reinit_completion(&report_complete);

	out[0] = test_hcd->report_type;

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_ENABLE_REPORT,
					out,
					1,
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					0);
	if (retval < 0) {
		TPD_INFO("Failed to write message %s\n", STR(CMD_ENABLE_REPORT));
		completed = false;
		goto exit;
	}

	timeout = REPORT_TIMEOUT_MS * num_of_reports;

	retval = wait_for_completion_timeout(&report_complete,
					     msecs_to_jiffies(timeout));
	if (retval == 0) {
		TPD_INFO("Timed out waiting for report collection\n");
	} else {
		completed = true;
	}

	out[0] = test_hcd->report_type;

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_DISABLE_REPORT,
					out,
					1,
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					0);
	if (retval < 0) {
		TPD_INFO("Failed to write message %s\n", STR(CMD_DISABLE_REPORT));
	}

	if (!completed) {
		retval = -EIO;
	}
exit:

	return retval;
}

static void syna_tcm_test_report(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	unsigned int offset, report_size;
	struct syna_tcm_test *test_hcd = tcm_hcd->test_hcd;

	if (tcm_hcd->report.id != test_hcd->report_type) {
		TPD_INFO("Not request report type\n");
		return;
	}

	report_size = tcm_hcd->report.buffer.data_length;
	LOCK_BUFFER(test_hcd->report);

	if (test_hcd->report_index == 0) {
		retval = syna_tcm_alloc_mem(tcm_hcd,
					    &test_hcd->report,
					    report_size*test_hcd->num_of_reports);
		if (retval < 0) {
			TPD_INFO("Failed to allocate memory\n");

			UNLOCK_BUFFER(test_hcd->report);
			return;
		}
	}

	if (test_hcd->report_index < test_hcd->num_of_reports) {
		offset = report_size * test_hcd->report_index;
		retval = secure_memcpy(test_hcd->report.buf + offset,
				       test_hcd->report.buf_size - offset,
				       tcm_hcd->report.buffer.buf,
				       tcm_hcd->report.buffer.buf_size,
				       tcm_hcd->report.buffer.data_length);
		if (retval < 0) {
			TPD_INFO("Failed to copy report data\n");

			UNLOCK_BUFFER(test_hcd->report);
			return;
		}

		test_hcd->report_index++;
		test_hcd->report.data_length += report_size;
	}

	UNLOCK_BUFFER(test_hcd->report);

	if (test_hcd->report_index == test_hcd->num_of_reports) {
		complete(&report_complete);
	}
	return;
}

static void syna_tcm_format_print(struct seq_file *s, struct syna_tcm_hcd *tcm_hcd, char *buffer)
{
	unsigned int row, col;
	unsigned int rows, cols;
	short *pdata_16;
	struct syna_tcm_test *test_hcd = tcm_hcd->test_hcd;

	rows = le2_to_uint(tcm_hcd->app_info.num_of_image_rows);
	cols = le2_to_uint(tcm_hcd->app_info.num_of_image_cols);

	if (buffer == NULL) {
		pdata_16 = (short *)&test_hcd->report.buf[0];
	} else {
		pdata_16 = (short *)buffer;
	}
	pdata_16 += (rows - 1) * cols;

	for (row = 0; row < rows; row++) {
		seq_printf(s, "[%02d] ", row);
		for (col = 0; col < cols; col++) {
			seq_printf(s, "%5d ", *pdata_16);
			pdata_16++;
		}
		pdata_16 -= cols * 2;
		seq_printf(s, "\n");
	}

	seq_printf(s, "\n");

	return;
}

static void syna_main_register(struct seq_file *s, void *chip_data)
{
	int retval = 0;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = syna_tcm_write_message(tcm_hcd,
					CMD_GET_NSM_INFO,
					NULL,
					0,
					&resp_buf,
					&resp_buf_size,
					&resp_length,
					0);
	if (retval < 0) {
		TPD_INFO("Failed to write command %s\n", STR(CMD_GET_NSM_INFO));
		if (s) {
			seq_printf(s, "Failed to write command %s\n", STR(CMD_GET_NSM_INFO));
		}
		goto exit;
	}

	if (resp_length < 10) {
		TPD_INFO("Error response data\n");
		if (s) {
			seq_printf(s, "Error response data\n");
		}
		goto exit;
	}

	TPD_INFO("Reset reason:0x%02x%02x\n", resp_buf[1], resp_buf[0]);
	TPD_INFO("power im: 0x%02x%02x\n", resp_buf[3], resp_buf[2]);
	TPD_INFO("nsm Frequency: 0x%02x%02x\n", resp_buf[5], resp_buf[4]);
	TPD_INFO("nsm State: 0x%02x%02x\n", resp_buf[7], resp_buf[6]);
	TPD_INFO("esd State: 0x%02x%02x\n", resp_buf[8], resp_buf[9]);
	TPD_INFO("Buid ID:%d, Custom ID:0x%s\n",
		 le4_to_uint(tcm_hcd->id_info.build_id),
		 tcm_hcd->app_info.customer_config_id);

	if (!s) {
		goto exit;
	}

	seq_printf(s, "Reset reason:0x%02x%02x\n", resp_buf[1], resp_buf[0]);
	seq_printf(s, "power im: 0x%02x%02x\n", resp_buf[3], resp_buf[2]);
	seq_printf(s, "nsm Frequency: 0x%02x%02x\n", resp_buf[5], resp_buf[4]);
	seq_printf(s, "nsm State: 0x%02x%02x\n", resp_buf[7], resp_buf[6]);
	seq_printf(s, "esd State: 0x%02x%02x\n", resp_buf[8], resp_buf[9]);
	seq_printf(s, "Buid ID:%d, Custom ID:0x%s\n",
		   le4_to_uint(tcm_hcd->id_info.build_id),
		   tcm_hcd->app_info.customer_config_id);
exit:
	kfree(resp_buf);

	return;
}

static void syna_delta_read(struct seq_file *s, void *chip_data)
{
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;

	retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_NO_DOZE, 1);
	if (retval < 0) {
		TPD_INFO("Failed to exit doze\n");
	}

	msleep(20);

	retval = syna_tcm_collect_reports(tcm_hcd, REPORT_DELTA, 1);
	if (retval < 0) {
		seq_printf(s, "Failed to read delta data\n");
		return;
	}

	syna_tcm_format_print(s, tcm_hcd, NULL);

	/*set normal doze*/
	retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_NO_DOZE, 0);
	if (retval < 0) {
		TPD_INFO("Failed to switch to normal\n");
	}
	return;
}

static void syna_baseline_read(struct seq_file *s, void *chip_data)
{
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;

	retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_NO_DOZE, 1);
	if (retval < 0) {
		TPD_INFO("Failed to exit doze\n");
	}

	msleep(20);

	retval = syna_tcm_collect_reports(tcm_hcd, REPORT_RAW, 1);
	if (retval < 0) {
		seq_printf(s, "Failed to read baseline data\n");
		return;
	}

	syna_tcm_format_print(s, tcm_hcd, NULL);

	/*set normal doze*/
	retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_NO_DOZE, 0);
	if (retval < 0) {
		TPD_INFO("Failed to switch to normal\n");
	}

	return;
}

static struct syna_auto_test_operations synaptics_test_ops = {
	.test1       =  syna_testing_noise,
	.test2       =  syna_testing_pt11,
	.test3       =  syna_testing_Doze_noise,
	.test4       =  syna_testing_dynamic_range,
	.test5       =  syna_testing_Doze_dynamic_range,
	.test6       =  syna_testing_Doze_dynamic_range_NULL,
	.test7       =  syna_testing_dynamic_range_NULL,
	.syna_black_screen_test_noise    =  syna_black_screen_test_noise,
	.syna_black_screen_test_dynamic  =  syna_black_screen_test_dynamic,
	.syna_auto_test_preoperation  =  synaptics_auto_test_preoperation,
	.syna_auto_test_endoperation  =  synaptics_auto_test_endoperation,
	.syna_auto_black_screen_test_preoperation  =  synaptics_auto_black_screen_test_preoperation,
	.syna_auto_black_screen_test_endoperation  =  synaptics_auto_black_screen_test_endoperation,
};

static struct syna_auto_test_operations synaptics_test_ops2 = {
	.test1       =  syna_testing_noise,
	.test2       =  syna_testing_pt17,
	.test3       =  syna_testing_pt18,
	.test4       =  syna_testing_Doze_noise,
	.test5       =  syna_testing_dynamic_range,
	.test6       =  syna_testing_Doze_dynamic_range,
	.test7       =  syna_testing_Doze_dynamic_range_NULL,
	.test8       =  syna_testing_dynamic_range_NULL,
	.syna_black_screen_test_noise    =  syna_black_screen_test_noise,
	.syna_black_screen_test_dynamic  =  syna_black_screen_test_dynamic,
	.syna_auto_test_preoperation  =  synaptics_auto_test_preoperation,
	.syna_auto_test_endoperation  =  synaptics_auto_test_endoperation,
	.syna_auto_black_screen_test_preoperation  =  synaptics_auto_black_screen_test_preoperation,
	.syna_auto_black_screen_test_endoperation  =  synaptics_auto_black_screen_test_endoperation,
};

static struct engineer_test_operations syna_engineer_test_ops = {
	.auto_test                  = synaptics_auto_test,
	.black_screen_test 			= synaptics_black_screen_test,
};

void syna_reserve_read(struct seq_file *s, void *chip_data)
{
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;

	retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_NO_DOZE, 1);
	if (retval < 0) {
		TPD_INFO("Failed to exit doze\n");
	}

	msleep(20);

	retval = syna_tcm_collect_reports(tcm_hcd, REPORT_DEBUG, 1);
	if (retval < 0) {
		seq_printf(s, "Failed to read delta data\n");
		return;
	}

	syna_tcm_format_print(s, tcm_hcd, NULL);

	/*set normal doze*/
	retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_NO_DOZE, 0);
	if (retval < 0) {
		TPD_INFO("Failed to switch to normal\n");
	}

	return;
}

int freq_point = 0;
void syna_freq_hop_trigger(void *chip_data)
{
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	TPD_INFO("send cmd to tigger frequency hopping here!!!\n");

	freq_point = 1 - freq_point;
	retval = syna_tcm_set_dynamic_config(tcm_hcd, DC_FREQUENCE_HOPPING, freq_point);
	if (retval < 0) {
		TPD_INFO("Failed to hop frequency\n");
	}
}

static int syna_tcm_smooth_lv_set(void *chip_data, int level)
{
	struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	unsigned short regval = 0;
	int retval = 0;

	retval = syna_tcm_get_dynamic_config(tcm_info, DC_GAME_MODE_ENABLED, &regval);
	if (retval < 0) {
		TPD_INFO("Failed to get smooth config\n");
		tcm_info->error_state_count++;
		return 0;
	}
	TPD_INFO("syna_tcm_smooth_lv_set 111 regval = %d\n", regval);
	tcm_info->error_state_count = 0;

	retval = syna_tcm_set_dynamic_config(tcm_info, DC_GAME_MODE_ENABLED, (level << 4) | (regval & 0x01));
	if (retval < 0) {
		TPD_INFO("Failed to set smooth config\n");
		return 0;
	}
	TPD_INFO("syna_tcm_smooth_lv_set 222 regval = %d\n", regval);

	retval = syna_tcm_get_dynamic_config(tcm_info, DC_GAME_MODE_ENABLED, &regval);
	if (retval < 0) {
		TPD_INFO("Failed to get smooth config\n");
		return 0;
	}
	TPD_INFO("OK synaptics smooth lv to %d, now reg_val:0x%x", level, regval);
	return 0;
}

static int syna_tcm_sensitive_lv_set(void *chip_data, int level)
{
	struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	unsigned short regval = 0;
	int retval = 0;

	retval = syna_tcm_set_dynamic_config(tcm_info, DC_GAME_MODE_ENABLED, level);
	if (retval < 0) {
		TPD_INFO("Failed to set sensitive config\n");
		tcm_info->error_state_count++;
		return 0;
	}
	tcm_info->error_state_count = 0;

	retval = syna_tcm_get_dynamic_config(tcm_info, DC_GAME_MODE_ENABLED, &regval);
	if (retval < 0) {
		TPD_INFO("Failed to get sensitive config\n");
		return 0;
	}
	TPD_INFO("OK synaptics sensitive lv to %d, now reg_val:%d", level, regval);

	return 0;
}

static struct debug_info_proc_operations syna_debug_proc_ops = {
	.delta_read    = syna_delta_read,
	.baseline_read = syna_baseline_read,
	.main_register_read = syna_main_register,
	.reserve_read  = syna_reserve_read,
};

static int syna_device_report_touch(struct syna_tcm_hcd *tcm_hcd)
{
	int ret = syna_parse_report(tcm_hcd);
	if (ret < 0) {
		TPD_INFO("Failed to parse report\n");
		return -EINVAL;
	}

	syna_set_trigger_reason(tcm_hcd, IRQ_TOUCH);
	return 0;
}

void syna_tcm_hdl_done(struct syna_tcm_hcd *tcm_hcd)
{
	int ret = 0;

	TPD_DETAIL("%s: Enter\n", __func__);

	ret = syna_tcm_identify(tcm_hcd, true);
	if (ret < 0) {
		TPD_INFO("Failed to do identification\n");
		return;
	}

	ret = syna_get_default_report_config(tcm_hcd);
	if (ret < 0) {
		TPD_INFO("failed to get default report config\n");
	}

	ret = syna_tcm_normal_mode(tcm_hcd);
	if (ret < 0) {
		TPD_INFO("failed to set normal mode\n");
	}

	enable_irq(tcm_hcd->s_client->irq);

	/*syna_tcm_get_app_info(tcm_hcd);*/

	syna_fw_version_update(tcm_hcd);

	g_tcm_hcd->hdl_finished_flag = 1;

	return;
}
/*
static int syna_tcm_irq_handle(void *chip_data)
{
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	if (tcm_hcd->zeroflash_init_done == 0) {
		TPD_INFO("hdl not ready, disable irq\n");
		disable_irq_nosync(tcm_hcd->s_client->irq);
		return retval;
	}

	syna_tcm_stop_reset_timer(tcm_hcd);

	retval =  syna_tcm_read_message(tcm_hcd, NULL, 0);
	if (retval == -ENXIO) {
		TPD_INFO("Failed to read message, start to do hdl\n");
	}
	return retval;
}
*/
static void syna_tcm_resume_timedout_operate(void *chip_data)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	TPD_INFO("syna_tcm_resume_timedout_operate ENTER!!\n");
	if (!g_tcm_hcd->hdl_finished_flag) {
		disable_irq_nosync(tcm_hcd->s_client->irq);
		syna_reset_gpio(tcm_hcd, false);
		usleep_range(5000, 5000);
		syna_reset_gpio(tcm_hcd, true);
		msleep(20);
		enable_irq(tcm_hcd->s_client->irq);

		/*syna_tcm_start_reset_timer(tcm_hcd);*/
	} else {
		TPD_INFO("hdl has done, don't reset again!!\n");
	}
}

static void synaptics_set_touch_direction(void *chip_data, uint8_t dir)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;

	tcm_hcd->touch_direction = dir;
}

static uint8_t synaptics_get_touch_direction(void *chip_data)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;

	return tcm_hcd->touch_direction;
}

static void syna_getglove_mode_status(void *chip_data, int *enable, int *count)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	struct touch_hcd *touch_hcd = tcm_hcd->touch_hcd;
	struct touch_data *touch_data = &touch_hcd->touch_data;
	int retval = 0;
	unsigned short config;

	retval = syna_tcm_get_dynamic_config(tcm_hcd, DC_GLOVE_MODE_STATE, &config);
	if (retval < 0) {
		TPD_INFO("Failed to get dynamic DC_GLOVE_MODE_STATE  config\n");
		return;
	}
	/*0x0F indicates the initial state;
	0x00 indicates that stage 1 has been entered, and there is no glove on the detection state surface
	0x01 indicates that stage 1 has been entered, and objects have been detected on the surface, but the conditions for stage 2 have not yet been met
	0x02 indicates that the glove mode is entered, and it will not exit after raising your hand.*/
	*enable = (config == 0x02 ? 1 : 0);
	TPD_INFO("%s: config id is %d, enable: %d ,glove_status%d\n", __func__, config, *enable, touch_data->glove_status);
	return;
}

static void syna_force_water_mode(void *chip_data, bool enable)
{
	TPD_INFO("%s: %s force_water_mode is not supported .\n", __func__, enable ? "Enter" : "Exit");
}

static void syna_read_water_flag(void *chip_data)
{
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(tcm_hcd->s_client);
	struct touch_hcd *touch_hcd = tcm_hcd->touch_hcd;
	struct touch_data *touch_data = &touch_hcd->touch_data;

	TP_INFO(touch_data->water_mode, "%s: water flag.\n", __func__);
	TPD_INFO("water_mode = %d \n", touch_data->water_mode);
	if (touch_data->water_mode == 1) {
		ts->water_mode = 1;
	} else {
		ts->water_mode = 0;
	}
}

static int syna_tcm_diaphragm_touch_lv_set(void *chip_data, int level)
{
	struct syna_tcm_hcd *tcm_info = (struct syna_tcm_hcd *)chip_data;
	unsigned short regval = 0;
	int retval = 0;
	retval = syna_tcm_get_dynamic_config(tcm_info, DC_LOW_TEMP_ENABLE, &regval);
	if (retval < 0) {
		TPD_INFO("Failed to get diaphragm_touch config\n");
		return 0;
	}

	switch (level) {
	case DIAPHRAGM_DEFAULT_MODE:
		regval = 0xfffc & regval;
		break;
	case DIAPHRAGM_FILM_MODE:
		regval = 0xfffc & regval;
		regval = 0x0001 | regval;
		break;
	case DIAPHRAGM_WATERPROO_MODE:
		regval = 0xfffc & regval;
		regval = 0x0002 | regval;
		break;
	case DIAPHRAGM_FILM_WATERPROO_MODE:
		regval = 0xfffc & regval;
		regval = 0x0003 | regval;
		break;
	default:
		TPD_INFO("error, level = %d", level);
		return 0;
	}

	retval = syna_tcm_set_dynamic_config(tcm_info, DC_LOW_TEMP_ENABLE, regval);
	if (retval < 0) {
		TPD_INFO("Failed to set diaphragm_touch config\n");
		return 0;
	}

	retval = syna_tcm_get_dynamic_config(tcm_info, DC_LOW_TEMP_ENABLE, &regval);
	if (retval < 0) {
		TPD_INFO("Failed to get diaphragm_touch config\n");
		return 0;
	}
	TPD_INFO("diaphragm_touch_lv_set level = %d regval = %d", level, regval);

	return 0;
}

static struct oplus_touchpanel_operations syna_tcm_ops = {
	.ftm_process       = syna_ftm_process,
	.get_vendor        = syna_get_vendor,
	.get_chip_info     = syna_get_chip_info,
	.get_touch_points  = syna_get_touch_points,
	.get_gesture_info  = syna_get_gesture_info,
	.power_control     = syna_power_control,
	.reset_gpio_control = syna_reset_gpio,
	.reset             = syna_tcm_reset,
	.trigger_reason    = syna_trigger_reason,
	.mode_switch       = syna_mode_switch,
/*	.irq_handle_unlock = syna_tcm_irq_handle,*/
	.fw_check          = syna_fw_check,
	.fw_update         = syna_tcm_fw_update,
/*	.async_work        = syna_tcm_async_work,*/
	/*.write_ps_status   = syna_tcm_write_ps_status,*/
/*	.black_screen_test = syna_tcm_black_screen_test,*/
	.reinit_device     = syna_tcm_reinit_device,
	.resume_timedout_operate = syna_tcm_resume_timedout_operate,
	.set_touch_direction    = synaptics_set_touch_direction,
	.get_touch_direction    = synaptics_get_touch_direction,
	.rate_white_list_ctrl   = syna_rate_white_list_ctrl,
/*	.freq_hop_trigger = syna_freq_hop_trigger,*/
	.smooth_lv_set    = syna_tcm_smooth_lv_set,
	.sensitive_lv_set = syna_tcm_sensitive_lv_set,
	.get_glove_mode         = syna_getglove_mode_status,
	.get_water_mode         = syna_read_water_flag,
	.force_water_mode       = syna_force_water_mode,
	.diaphragm_touch_lv_set    = syna_tcm_diaphragm_touch_lv_set,
	.aiunit_game_info       = syna_aiunit_game_info,
};

/*
*Interface for lcd to wait tp resume before suspend
*/
void tp_wait_hdl_finished(void)
{
	int retry_cnt = 0;
	struct syna_tcm_hcd *tcm_hcd = g_tcm_hcd;

	if (!g_tcm_hcd) {
		return;
	}

	syna_tcm_stop_reset_timer(tcm_hcd);

	do {
		if (retry_cnt) {
			msleep(100);
		}
		retry_cnt++;
		TPD_INFO("Wait hdl finished retry %d times...  \n", retry_cnt);
	} while (!g_tcm_hcd->hdl_finished_flag && retry_cnt < 20);
}

/*
*Interface for lcd to control tp irq
*mode:0-esd 1-black gesture
*/
int tp_control_irq(bool enable, int mode)
{
	struct syna_tcm_hcd *tcm_hcd = g_tcm_hcd;
	if (mode == 0) {
		if (enable) {
			TPD_INFO("%s enable\n", __func__);
			tcm_hcd->esd_irq_disabled = 0;
			enable_irq(g_tcm_hcd->s_client->irq);
		} else {
			TPD_INFO("%s disable\n", __func__);
			g_tcm_hcd->response_code = STATUS_ERROR;
			atomic_set(&g_tcm_hcd->command_status, CMD_IDLE);
			complete(&response_complete);
			tcm_hcd->esd_irq_disabled = 1;
			wait_zeroflash_firmware_work();
			disable_irq_nosync(g_tcm_hcd->s_client->irq);
		}
	} else if (mode == 1) {
		if (enable) {
			TPD_INFO("%s enable irq-%d\n", __func__, g_tcm_hcd->s_client->irq);
			enable_irq(g_tcm_hcd->s_client->irq);
			tcm_hcd->tp_irq_state = 1;
		} else {
			TPD_INFO("%s disable irq-%d\n", __func__, g_tcm_hcd->s_client->irq);
			disable_irq_nosync(g_tcm_hcd->s_client->irq);
			tcm_hcd->tp_irq_state = 0;
		}
	}

	return 0;
}
EXPORT_SYMBOL(tp_control_irq);
/*100ms*/
#define RESET_TIMEOUT_TIME 100 * 1000 * 1000

static void syna_reset_timeout_work(struct work_struct *work)
{
	return;
}

void syna_tcm_start_reset_timer(struct syna_tcm_hcd *tcm_hcd)
{
	if (tcm_hcd->reset_watchdog_running == 0) {
		TPD_DETAIL("%s hrtimer_start!!\n", __func__);

		hrtimer_start(&tcm_hcd->watchdog,
			      ktime_set(0, RESET_TIMEOUT_TIME),
			      HRTIMER_MODE_REL);
		tcm_hcd->reset_watchdog_running = 1;
	}
}

void syna_tcm_stop_reset_timer(struct syna_tcm_hcd *tcm_hcd)
{
	if (tcm_hcd->reset_watchdog_running == 1) {
		TPD_DETAIL("%s hrtimer_cancel!!\n", __func__);
		hrtimer_cancel(&tcm_hcd->watchdog);
		tcm_hcd->reset_watchdog_running = 0;
	}
}

static enum hrtimer_restart syna_tcm_reset_timeout(struct hrtimer *timer)
{
	struct syna_tcm_hcd *tcm_hcd = g_tcm_hcd;

	schedule_work(&(tcm_hcd->timeout_work));
	hrtimer_forward_now(&tcm_hcd->watchdog, ktime_set(0, RESET_TIMEOUT_TIME));
	return HRTIMER_RESTART;  /*restart the timer*/
}

static int syna_tcm_init_device(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;

	mutex_init(&tcm_hcd->reset_mutex);
	mutex_init(&tcm_hcd->rw_ctrl_mutex);
	mutex_init(&tcm_hcd->command_mutex);
	mutex_init(&tcm_hcd->identify_mutex);
	mutex_init(&tcm_hcd->io_ctrl_mutex);

	INIT_BUFFER(tcm_hcd->in, false);
	INIT_BUFFER(tcm_hcd->out, false);
	INIT_BUFFER(tcm_hcd->resp, true);
	INIT_BUFFER(tcm_hcd->temp, false);
	INIT_BUFFER(tcm_hcd->config, false);
	INIT_BUFFER(tcm_hcd->default_config, false);
	INIT_BUFFER(tcm_hcd->report.buffer, true);

	hrtimer_init(&tcm_hcd->watchdog, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tcm_hcd->watchdog.function = syna_tcm_reset_timeout;
	INIT_WORK(&tcm_hcd->timeout_work, syna_reset_timeout_work);

	retval = syna_tcm_alloc_mem(tcm_hcd,
				    &tcm_hcd->in,
				    tcm_hcd->read_length + 2);
	TPD_INFO("%s read_length:%d\n", __func__, tcm_hcd->read_length);
	if (retval < 0) {
		TPD_INFO("Failed to allocate memory for tcm_hcd->in.buf\n");
		return retval;
	}

	tcm_hcd->touch_hcd = (struct touch_hcd *)kzalloc(sizeof(struct touch_hcd), GFP_KERNEL);
	if (!tcm_hcd->touch_hcd) {
		retval = -ENOMEM;
		return retval;
	}

	tcm_hcd->touch_hcd->touch_data.object_data =
		(struct object_data *)kzalloc(sizeof(struct object_data)*tcm_hcd->max_touch_num, GFP_KERNEL);
	if (!tcm_hcd->touch_hcd->touch_data.object_data) {
		retval = -ENOMEM;
		goto free_touch;
	}

	tcm_hcd->touch_hcd->max_objects = tcm_hcd->max_touch_num;
	mutex_init(&tcm_hcd->touch_hcd->report_mutex);

	INIT_BUFFER(tcm_hcd->touch_hcd->out, false);
	INIT_BUFFER(tcm_hcd->touch_hcd->resp, false);

	tcm_hcd->test_hcd = (struct syna_tcm_test *)kzalloc(sizeof(struct syna_tcm_test), GFP_KERNEL);
	if (!tcm_hcd->test_hcd) {
		retval = -ENOMEM;
		goto free_object_data;
	}

	INIT_BUFFER(tcm_hcd->test_hcd->report, false);

	return retval;

free_object_data:
	kfree(tcm_hcd->touch_hcd->touch_data.object_data);
free_touch:
	kfree(tcm_hcd->touch_hcd);

	return retval;
}

static void syna_tcm_parse_dts(struct syna_tcm_hcd *tcm_hcd, struct spi_device *spi)
{
	int rc;
	int temp = 0;
	int temp_array[4];
	struct device *dev;
	struct device_node *np;

	dev = &spi->dev;
	np = dev->of_node;
	rc = of_property_read_u32_array(np, "synaptics,grip-darkzone-area", temp_array, 4);
	if (rc) {
		tcm_hcd->grip_darkzone_x = 0xC8C8;
		tcm_hcd->grip_darkzone_y = 0x3030;
		tcm_hcd->grip_darkzone_v2_x = 0x1830;
		tcm_hcd->grip_darkzone_v2_y = 0xC83C;
	} else {
		tcm_hcd->grip_darkzone_x = temp_array[0];
		tcm_hcd->grip_darkzone_y = temp_array[1];
		tcm_hcd->grip_darkzone_v2_x = temp_array[2];
		tcm_hcd->grip_darkzone_v2_y = temp_array[3];
	}

	rc = of_property_read_u32(np, "synaptics,max_speed_hz", &temp);
	if (rc) {
		TPD_INFO("synaptics,max_speed_hz not specified\n");
	} else {
		tcm_hcd->s_client->max_speed_hz = temp;
		TPD_INFO("max_speed_hz set to %d\n", tcm_hcd->s_client->max_speed_hz);
	}

	tcm_hcd->irq_trigger_hdl_support = of_property_read_bool(np, "synaptics,irq_trigger_hdl_support");
	tcm_hcd->pt17_pt18_test_support = of_property_read_bool(np, "pt17_pt18_test_support");
}

static int syna_tcm_spi_probe(struct spi_device *spi)
{
	int retval = 0;
	u64 time_counter = 0;
	struct syna_tcm_hcd *tcm_hcd;
	struct touchpanel_data *ts = NULL;
	struct device_hcd *device_hcd;

	TPD_INFO("%s: enter\n", __func__);

	reset_healthinfo_time_counter(&time_counter);

	tcm_hcd = kzalloc(sizeof(*tcm_hcd), GFP_KERNEL);
	if (!tcm_hcd) {
		TPD_INFO("no more memory\n");
		return -ENOMEM;
	}

	ts = common_touch_data_alloc();
	if (ts == NULL) {
		TPD_INFO("failed to alloc common data\n");
		retval = -1;
		goto ts_alloc_failed;
	}

	g_tcm_hcd = tcm_hcd;
	tcm_hcd->s_client = spi;
	tcm_hcd->hw_res = &ts->hw_res;
	tcm_hcd->rd_chunk_size = RD_CHUNK_SIZE;
	tcm_hcd->wr_chunk_size = WR_CHUNK_SIZE;
	tcm_hcd->read_length = MIN_READ_LENGTH;
	tcm_hcd->max_touch_num = 10; /*default*/

	tcm_hcd->ubl_addr = 0x2c;
	tcm_hcd->write_message = syna_tcm_write_message_zeroflash;
	tcm_hcd->tp_fw_update_headfile = false;
	tcm_hcd->tp_fw_update_first = false;
	tcm_hcd->tp_fw_update_parse = true;
	tcm_hcd->tp_irq_state = 1;
	ts->chip_data = tcm_hcd;

	/*tcm_hcd->syna_ops = &syna_proc_ops;*/
	ts->ts_ops = &syna_tcm_ops;
	ts->engineer_ops = &syna_engineer_test_ops;
	ts->com_test_data.chip_test_ops = &synaptics_test_ops;

	ts->debug_info_ops = &syna_debug_proc_ops;
	ts->dev = &spi->dev;
	ts->s_client = spi;
	ts->s_client->mode = SPI_MODE_3;
	ts->s_client->bits_per_word = 8;
	spi->bits_per_word = 8;
	spi->cs_setup.value = 1;
	spi->cs_setup.unit = 0;
	spi->cs_hold.value = 1;
	spi->cs_hold.unit = 0;
	spi->cs_inactive.value = 1;
	spi->cs_inactive.unit = 0;
	ts->irq = spi->irq;
	ts->irq_flags_cover = 0x2008;
	/*ts->has_callback = true;*/
	/*ts->use_resume_notify = true;*/
	tcm_hcd->in_suspend = &ts->is_suspended;
	tcm_hcd->init_okay = false;
	spi_set_drvdata(spi, ts);
	retval = syna_tcm_init_device(tcm_hcd);
	if (retval < 0) {
		TPD_INFO("Failed to init device information\n");
		goto err_alloc_mem;
	}

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

	syna_tcm_parse_dts(tcm_hcd, spi);
	if (tcm_hcd->pt17_pt18_test_support) {
		ts->com_test_data.chip_test_ops = &synaptics_test_ops2;
	}
#if defined(CONFIG_SPI_MT65XX)
	spi->controller_data = (void *)&spi_ctrdata;
#endif
	retval = spi_setup(spi);
	if (retval < 0) {
		TPD_INFO("Failed to set up SPI protocol driver\n");
		return retval;
	}

	retval = register_common_touch_device(ts);

	tcm_hcd->tcm_firmware_headfile = ts->firmware_in_dts;
	if (retval < 0 && (retval != -EFTM)) {
		TPD_INFO("Failed to init device information\n");
		goto err_register_driver;
	}
	tcm_hcd->p_firmware_headfile = &ts->panel_data.firmware_headfile;
	tcm_hcd->health_monitor_support = ts->health_monitor_support;
	if (tcm_hcd->health_monitor_support) {
		tcm_hcd->monitor_data = &ts->monitor_data;
	}
	ts->int_mode = 1;

	ts->tp_suspend_order = LCD_TP_SUSPEND;
	ts->tp_resume_order = LCD_TP_RESUME;
	ts->skip_reset_in_resume = true;
	ts->skip_suspend_operate = true;
	/*ts->mode_switch_type = SINGLE;*/
	if (!ts->irq_trigger_hdl_support) {
		ts->irq_trigger_hdl_support = tcm_hcd->irq_trigger_hdl_support;
	}

	synaptics_create_proc(ts, tcm_hcd->syna_ops);
	init_completion(&tcm_hcd->config_complete);

	device_hcd = syna_remote_device_init(tcm_hcd);
	if (device_hcd) {
		device_hcd->irq = tcm_hcd->s_client->irq;
		device_hcd->read_message = syna_tcm_read_message;
		device_hcd->write_message = syna_tcm_write_message;
		device_hcd->reset = syna_tcm_reset;
		device_hcd->report_touch = syna_device_report_touch;
	}
	tcm_hcd->init_okay = false;
	g_tcm_hcd->hdl_finished_flag = 0;

	tcm_hcd->init_okay = true;
	syna_remote_zeroflash_init(tcm_hcd);

	if (ts->health_monitor_support) {
		tp_healthinfo_report(&ts->monitor_data, HEALTH_PROBE, &time_counter);
	}
/*#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
    if (ts->boot_mode == RECOVERY_BOOT)
#else
    if (ts->boot_mode == MSM_BOOT_MODE__RECOVERY)
#endif
    {
        TPD_INFO("In Recovery mode, no-flash download fw by headfile\n");
        syna_tcm_fw_update(tcm_hcd, NULL, 0);
    }

    if (is_oem_unlocked()) {
        TPD_INFO("Replace system image for cts, download fw by headfile\n");
        syna_tcm_fw_update(tcm_hcd, g_tcm_hcd->tcm_firmware_headfile, 0);
    }*/
	return 0;

err_alloc_mem:
	RELEASE_BUFFER(tcm_hcd->report.buffer);
	RELEASE_BUFFER(tcm_hcd->config);
	RELEASE_BUFFER(tcm_hcd->temp);
	RELEASE_BUFFER(tcm_hcd->resp);
	RELEASE_BUFFER(tcm_hcd->out);
	RELEASE_BUFFER(tcm_hcd->in);

err_register_driver:
	common_touch_data_free(ts);
	ts = NULL;

ts_alloc_failed:
	kfree(tcm_hcd);

	return retval;
}

static void syna_tcm_tp_shutdown(struct spi_device *s_client)
{
        struct touchpanel_data *ts = spi_get_drvdata(s_client);

        TPD_INFO("%s is called\n", __func__);

        tp_shutdown(ts);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void syna_tcm_remove(struct spi_device *spi)
#else
static int syna_tcm_remove(struct spi_device *spi)
#endif
{
	struct touchpanel_data *ts = spi_get_drvdata(spi);
	struct syna_tcm_hcd *tcm_hcd = (struct syna_tcm_hcd *)ts->chip_data;

	RELEASE_BUFFER(tcm_hcd->report.buffer);
	RELEASE_BUFFER(tcm_hcd->config);
	RELEASE_BUFFER(tcm_hcd->temp);
	RELEASE_BUFFER(tcm_hcd->resp);
	RELEASE_BUFFER(tcm_hcd->out);
	RELEASE_BUFFER(tcm_hcd->in);

	kfree(tcm_hcd);
	kfree(ts);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
#else
	return 0;
#endif
}

static struct of_device_id syna_match_table[] = {
#ifdef CONFIG_TOUCHPANEL_MULTI_NOFLASH
	{ .compatible = "oplus,tp_noflash", },
#else
	{ .compatible = TPD_DEVICE, },
#endif
	{ }
};

static int syna_i2c_suspend(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	TPD_INFO("%s: is called\n", __func__);
	tp_pm_suspend(ts);
	return 0;
}

static int syna_i2c_resume(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("%s is called\n", __func__);
	tp_pm_resume(ts);
	return 0;
}

static const struct dev_pm_ops syna_pm_ops = {
	.suspend = syna_i2c_suspend,
	.resume = syna_i2c_resume,
};

static const struct spi_device_id syna_tmc_id[] = {
#ifdef CONFIG_TOUCHPANEL_MULTI_NOFLASH
	{ "oplus,tp_noflash", 0 },
#else
	{ TPD_DEVICE, 0 },
#endif
	{ }
};

static struct spi_driver syna_spi_driver = {
	.probe      = syna_tcm_spi_probe,
	.remove     = syna_tcm_remove,
	.id_table   = syna_tmc_id,
	.shutdown   = syna_tcm_tp_shutdown,
	.driver     = {
		.name   = TPD_DEVICE,
		.of_match_table =  syna_match_table,
		.pm = &syna_pm_ops,
	},
};

static int __init syna_tcm_module_init(void)
{
	if (!tp_judge_ic_match(SYNDRIVER_NAME)) {
		return 0;
	}

	get_oem_verified_boot_state();

	if (spi_register_driver(&syna_spi_driver)!= 0) {
		TPD_INFO("unable to add spi driver.\n");
		return 0;
	}
	TPD_INFO("%s is called\n", __func__);

	return 0;
}

static void __exit syna_tcm_module_exit(void)
{
	spi_unregister_driver(&syna_spi_driver);
	return;
}

late_initcall(syna_tcm_module_init);
module_exit(syna_tcm_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Touch Driver");
MODULE_LICENSE("GPL v2");
