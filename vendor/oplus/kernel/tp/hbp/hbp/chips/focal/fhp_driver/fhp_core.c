/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2020-2022, FocalTech Systems, Ltd., all rigfhps reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
* File Name: fhp_core.c
* Author: Focaltech Driver Team
* Created: 2020-06-17
*
* Abstract: fhp driver for FocalTech
*
*****************************************************************************/
/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/mm.h>
#include "fhp_core.h"

#include "../../../hbp_core.h"
#include "../../../hbp_spi.h"
#include "../../../utils/debug.h"

#define PLATFORM_DRIVER_NAME "fts"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
/*Configuration*/
#define FHP_CHIP_MS_RESET_PROBE              200
#define FHP_CHIP_MS_RESET_RESUME             30


#define FTS_DRIVER_NAME                     "fts_fhp"
#define FTS_FHP_NAME                         "fhp_ft"

/*chip SPI command parameters*/
#define SPI_DUMMY_BYTE                      3
#define SPI_CMD_BYTE                        4
#define SPI_CRC_BYTE                        2
#define SPI_HEADER_BYTE ((SPI_CMD_BYTE) + (SPI_DUMMY_BYTE) + (SPI_CRC_BYTE))
#define SPI_RETRY_NUMBER                    3
#define CS_HIGH_DELAY                       150 /* unit: us */
#define DATA_CRC_EN                         0x20
#define WRITE_CMD                           0x00
#define READ_CMD                            (0x80 | DATA_CRC_EN)
#define SPI_LOGBUF_SIZE                     256

#define FHP_CMD_GET_FRAME                    0x3A

#define FTS_REG_POWER_MODE                  0xA5
#define FTS_REG_GESTURE_EN                  0xD0
#define FTS_REG_GESTURE_OUTPUT_ADDRESS      0xD3


/*
struct
{
        UINT8 type;
        UINT8 value;
        SINT16 mcdiff[16*36];
        SINT16 ScWaterRXdiff[36];
        SINT16 ScWaterTXdiff[36];   多余TX数量的值可以忽略
        SINT16 ScNormalRXdiff[36];
}
*/

#define FTS_CMD_DIFF						0x70
#define FTS_CAP_DATA_LEN					(2 + 16*36*2 + (36+36)*2 + 36*2)


/*****************************************************************************
* Static variabls
*****************************************************************************/
struct fts_core *g_fts;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

static void crckermit(u8 *data, u32 len, u16 *crc_out)
{
	u32 i = 0;
	u32 j = 0;
	u16 crc = 0xFFFF;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++) {
			if (crc & 0x01) {
				crc = (crc >> 1) ^ 0x8408;
			} else {
				crc = (crc >> 1);
			}
		}
	}

	*crc_out = crc;
}

static int rdata_check(u8 *rdata, u32 rlen)
{
	u16 crc_calc = 0;
	u16 crc_read = 0;

	crckermit(rdata, rlen - 2, &crc_calc);
	crc_read = (u16)(rdata[rlen - 1] << 8) + rdata[rlen - 2];
	if (crc_calc != crc_read) {
		hbp_info("CRC check fail,calc(%x)!=read(%x)", crc_calc, crc_read);
		return -EIO;
	}

	return 0;
}

static int fhp_chip_write(struct fts_core *ts_data, u8 *writebuf, u32 writelen)
{
	int ret = 0;
	int i = 0;
	u8 *txbuf = ts_data->bus_tx_buf;
	u8 *rxbuf = ts_data->bus_rx_buf;
	u32 datalen = writelen - 1;
	u32 txlen_need = datalen + SPI_HEADER_BYTE;
	u32 txlen = 0;

	if (!writebuf || !writelen || (txlen_need > PAGE_SIZE)) {
		hbp_err("writebuf/writelen(%d) is invalid", writelen);
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_mutex);
	memset(txbuf, 0x0, txlen_need);
	memset(rxbuf, 0x0, txlen_need);
	txbuf[txlen++] = writebuf[0];
	txbuf[txlen++] = WRITE_CMD;
	txbuf[txlen++] = (datalen >> 8) & 0xFF;
	txbuf[txlen++] = datalen & 0xFF;
	if (datalen > 0) {
		txlen = txlen + SPI_DUMMY_BYTE;
		memcpy(&txbuf[txlen], &writebuf[1], datalen);
		txlen = txlen + datalen;
	}

	for (i = 0; i < SPI_RETRY_NUMBER; i++) {
		ret = ts_data->bus_ops->spi_sync(ts_data->bus_ops, txbuf, rxbuf, txlen);
		if ((0 == ret) && ((rxbuf[3] & 0xA0) == 0)) {
			break;
		} else {
			hbp_err("data write(addr:%x),status:%x,retry:%d,ret:%d",
				writebuf[0], rxbuf[3], i, ret);
			ret = -EIO;
		}
		udelay(CS_HIGH_DELAY);
	}
	if (ret < 0) {
		hbp_err("data write(addr:%x) fail,status:%x,ret:%d",
			writebuf[0], rxbuf[3], ret);
	}

	udelay(CS_HIGH_DELAY);
	mutex_unlock(&ts_data->bus_mutex);
	return ret;
}

static int fhp_chip_read(struct fts_core *ts_data, u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
	int ret = 0;
	int i = 0;
	u8 *txbuf = ts_data->bus_tx_buf;
	u8 *rxbuf = ts_data->bus_rx_buf;
	u32 txlen_need = datalen + SPI_HEADER_BYTE;
	u32 txlen = 0;
	u8 ctrl = READ_CMD;
	u32 dp = 0;

	if (!cmd || !cmdlen || !data || !datalen || (txlen_need > PAGE_SIZE)) {
		hbp_err("cmd/cmdlen(%d)/data/datalen(%d) is invalid", cmdlen, datalen);
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_mutex);
	memset(txbuf, 0x0, txlen_need);
	memset(rxbuf, 0x0, txlen_need);
	txbuf[txlen++] = cmd[0];
	txbuf[txlen++] = ctrl;
	txbuf[txlen++] = (datalen >> 8) & 0xFF;
	txbuf[txlen++] = datalen & 0xFF;
	dp = txlen + SPI_DUMMY_BYTE;
	txlen = dp + datalen;
	if (ctrl & DATA_CRC_EN) {
		txlen = txlen + SPI_CRC_BYTE;
	}

	for (i = 0; i < SPI_RETRY_NUMBER; i++) {
		ret = ts_data->bus_ops->spi_sync(ts_data->bus_ops, txbuf, rxbuf, txlen);
		if ((0 == ret) && ((rxbuf[3] & 0xA0) == 0)) {
			memcpy(data, &rxbuf[dp], datalen);
			/* crc check */
			if (ctrl & DATA_CRC_EN) {
				ret = rdata_check(&rxbuf[dp], txlen - dp);
				if (ret < 0) {
					hbp_debug("data read(addr:%x) crc abnormal,retry:%d",
						  cmd[0], i);
					udelay(CS_HIGH_DELAY);
					continue;
				}
			}
			break;
		} else {
			hbp_err("data read(addr:%x) status:%x,retry:%d,ret:%d",
				cmd[0], rxbuf[3], i, ret);
			ret = -EIO;
			udelay(CS_HIGH_DELAY);
		}
	}

	if (ret < 0) {
		hbp_err("data read(addr:%x) %s,status:%x,ret:%d", cmd[0],
			(i >= SPI_RETRY_NUMBER) ? "crc abnormal" : "fail",
			rxbuf[3], ret);
	}

	udelay(CS_HIGH_DELAY);
	mutex_unlock(&ts_data->bus_mutex);
	return ret;
}

int fhp_chip_write_reg(struct fts_core *ts_data, u8 addr, u8 value)
{
	u8 writebuf[2] = { 0 };

	writebuf[0] = addr;
	writebuf[1] = value;
	return fhp_write(writebuf, 2);
}

int fhp_chip_read_reg(struct fts_core *ts_data, u8 addr, u8 *value)
{
	return fhp_read(&addr, 1, value, 1);
}


int fhp_write(u8 *writebuf, u32 writelen)
{
	return fhp_chip_write(g_fts, writebuf, writelen);
}

int fhp_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
	return fhp_chip_read(g_fts, cmd, cmdlen, data, datalen);
}


#define PROC_READ_REGISTER                      1
#define PROC_WRITE_REGISTER                     2
#define PROC_WRITE_DATA                         6
#define PROC_READ_DATA                          7
#define PROC_SET_TEST_FLAG                      8
#define PROC_HW_RESET                           11
#define PROC_WRITE_DATA_DIRECT                  16
#define PROC_READ_DATA_DIRECT                   17
#define PROC_CONFIGURE                          18
#define PROC_CONFIGURE_INTR                     20
#define PROC_NAME                               "ftxxxx-debug"
#define PROC_BUF_SIZE                           256

static int fhp_chip_get_frame(void *priv, u8 *raw, u32 rawsize)
{
	u8 cmd = 0;
	u8 *offset = raw;
	struct fts_core *ts_data = (struct fts_core *)priv;

	cmd = FTS_CMD_DIFF;
	fhp_chip_read(ts_data, &cmd, 1, offset, FTS_CAP_DATA_LEN);

	return 0;
}

int fhp_spi_sync(void *priv, char *tx, char *rx, int32_t len)
{
	struct fts_core *fts = (struct fts_core *)priv;

	return fts->bus_ops->spi_sync(fts->bus_ops, tx, rx, len);
}

struct fod_info {
	int fp_id;
	int event_type;
	int fp_down;
	int fp_x;
	int fp_y;
	int fp_area_rate;
	int fp_time;
};

static int fhp_read_fod_info(struct fts_core *ts_data, struct fod_info *fod)
{
#define FT3681_REG_FOD_INFO (0xE1)
#define FT3681_REG_FOD_INFO_LEN (10)
	int ret = 0;
	u8 cmd = FT3681_REG_FOD_INFO;
	u8 val[FT3681_REG_FOD_INFO_LEN] = { 0 };

	ret = fhp_chip_read(ts_data, &cmd, 1, &val[0], FT3681_REG_FOD_INFO_LEN);
	if (ret < 0) {
		hbp_err("failed to read fod data\n");
		return ret;
	}

	fod->fp_id = val[0];
	fod->event_type = val[1];
	if (val[8] == 0) {
		fod->fp_down = 1;
	} else if (val[8] == 1) {
		fod->fp_down = 0;
	} else {
		hbp_err("failed to read fp down 0x%x\n", val[8]);
		return -1;
	}

	fod->fp_area_rate = val[2];
	fod->fp_x = (val[4] << 8) + val[5];
	fod->fp_y = (val[6] << 8) + val[7];
	fod->fp_time = val[9];
	return 0;
}

static int fhp_chip_get_gesture(void *priv, struct gesture_info *gesture)
{
#define FT3681_REG_GESTURE_ADDRESS (0xD3)
#define FT3681_GESTURE_DATA_LEN (28)

	struct fts_core *ts_data = (struct fts_core *)priv;
	struct fod_info fod;
	int ret = 0;
	u8 cmd = FT3681_REG_GESTURE_ADDRESS;
	u8 buf[FT3681_GESTURE_DATA_LEN] = { 0 };
	enum gesture_id gesture_id = 0;
	u8 point_num = 0;

	ret = fhp_chip_read(ts_data, &cmd, 1, &buf[0], FT3681_GESTURE_DATA_LEN);
	if (ret < 0) {
		hbp_err("failed to read gesture data\n");
		return ret;
	}

	gesture_id = buf[0];
	point_num = buf[1];
	hbp_info("gesture id=0x%x, point_num=%d\n", gesture_id, point_num);

	switch (gesture_id) {
	case GESTURE_DOUBLE_TAP:
		gesture->type = DoubleTap;
		break;
	case GESTURE_UP_VEE:
		gesture->type = UpVee;
		break;
	case GESTURE_DOWN_VEE:
		gesture->type = DownVee;
		break;
	case GESTURE_LEFT_VEE:
		gesture->type = LeftVee;
		break;
	case GESTURE_RIGHT_VEE:
		gesture->type = RightVee;
		break;
	case GESTURE_O_CLOCKWISE:
		gesture->clockwise = 1;
		gesture->type = Circle;
		break;
	case GESTURE_O_ANTICLOCK:
		gesture->clockwise = 0;
		gesture->type = Circle;
		break;
	case GESTURE_DOUBLE_SWIP:
		gesture->type = DoubleSwip;
		break;
	case GESTURE_LEFT2RIGHT_SWIP:
		gesture->type = Left2RightSwip;
		break;
	case GESTURE_RIGHT2LEFT_SWIP:
		gesture->type = Right2LeftSwip;
		break;
	case GESTURE_UP2DOWN_SWIP:
		gesture->type = Up2DownSwip;
		break;
	case GESTURE_DOWN2UP_SWIP:
		gesture->type = Down2UpSwip;
		break;
	case GESTURE_M:
		gesture->type = Mgestrue;
		break;
	case GESTURE_W:
		gesture->type = Wgestrue;
		break;
	case GESTURE_HEART_CLOCKWISE:
		gesture->clockwise = 1;
		gesture->type = Heart;
		break;
	case GESTURE_HEART_ANTICLOCK:
		gesture->clockwise = 0;
		gesture->type = Heart;
		break;
	case GESTURE_SINGLE_TAP:
		gesture->type = SingleTap;
		break;
	case GESTURE_FINGER_PRINT:
		ret = fhp_read_fod_info(ts_data, &fod);
		if (ret < 0) {
			hbp_err("failed to read fod info\n");
			return ret;
		}

		if (fod.event_type == 0x26) {
			if (fod.fp_down) {
				gesture->type = FingerprintDown;
			} else {
				gesture->type = FingerprintUp;
			}

			gesture->Point_start.x = fod.fp_x;
			gesture->Point_start.y = fod.fp_y;
			gesture->Point_end.x = fod.fp_area_rate;
			gesture->Point_end.y = 0;
		}
		break;
	default:
		gesture->type = UnknownGesture;
	}

	if (gesture->type != UnknownGesture &&
	    gesture->type != FingerprintDown &&
	    gesture->type != FingerprintUp) {
		gesture->Point_start.x = (u16)((buf[2] << 8) + buf[3]);
		gesture->Point_start.y = (u16)((buf[4] << 8) + buf[5]);
		gesture->Point_end.x = (u16)((buf[6] << 8) + buf[7]);
		gesture->Point_end.y = (u16)((buf[8] << 8) + buf[9]);
		gesture->Point_1st.x = (u16)((buf[10] << 8) + buf[11]);
		gesture->Point_1st.y = (u16)((buf[12] << 8) + buf[13]);
		gesture->Point_2nd.x = (u16)((buf[14] << 8) + buf[15]);
		gesture->Point_2nd.y = (u16)((buf[16] << 8) + buf[17]);
		gesture->Point_3rd.x = (u16)((buf[18] << 8) + buf[19]);
		gesture->Point_3rd.y = (u16)((buf[20] << 8) + buf[21]);
		gesture->Point_4th.x = (u16)((buf[22] << 8) + buf[23]);
		gesture->Point_4th.y = (u16)((buf[24] << 8) + buf[25]);
	}

	return 0;
}

struct dev_operations fts_ops = {
	.spi_sync = fhp_spi_sync,
	.get_frame = fhp_chip_get_frame,
	.get_gesture = fhp_chip_get_gesture,
};

static int fts_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct fts_core *fts;
	struct chip_info info;

	if (!match_from_cmdline(&pdev->dev, &info)) {
		return 0;
	}

	fts = kzalloc(sizeof(struct fts_core), GFP_KERNEL);
	if (!fts) {
		return -ENOMEM;
	}

	mutex_init(&fts->bus_mutex);
	fts->bus_rx_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	fts->bus_tx_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);

	g_fts = fts;

	ret = hbp_register_devices(fts, &pdev->dev, &fts_ops, &info, &fts->bus_ops);
	if (ret < 0) {
		hbp_err("failed to register device:%s\n", info.vendor);
		goto err_exit;
	}

	hbp_info("probe end.\n");
	return 0;

err_exit:
	return ret;
}

static int fts_dev_remove(struct platform_device *spi)
{
	return 0;
}

static const struct of_device_id fts_dt_match[] = {
	{.compatible = "focaltech,fts", },
	{},
};
MODULE_DEVICE_TABLE(of, fts_dt_match);

static struct platform_driver fts_dev_driver = {
	.probe = fts_dev_probe,
	.remove = fts_dev_remove,
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(fts_dt_match),
	},
};

static int __init fts_platform_init()
{
	return platform_driver_register(&fts_dev_driver);
}

late_initcall(fts_platform_init);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech FHP Driver");
MODULE_LICENSE("GPL v2");
