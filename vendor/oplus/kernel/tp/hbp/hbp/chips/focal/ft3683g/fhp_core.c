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
#include <linux/version.h>
#include <linux/proc_fs.h>

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
#define CS_HIGH_DELAY                       150//150 /* unit: us */
#define DATA_CRC_EN                         0x20
#define WRITE_CMD                           0x00
#define READ_CMD                            (0x80 | DATA_CRC_EN)
#define SPI_LOGBUF_SIZE                     256

#define FHP_CMD_GET_FRAME                    0x01

#define FTS_REG_POINTS                      0x01
#define FTS_REG_REPORT_MODE                 0x9E
#define FTS_REG_POWER_MODE                  0xA5
#define FTS_REG_GESTURE_EN                  0xD0
#define FTS_REG_GESTURE_MODE                0xD1

#define FTS_REG_GESTURE_OUTPUT_ADDRESS      0xD3

#define FTS_REPORT_MODE_HBP                 0x41
#define FTS_REPORT_MODE_LBP                 0x00

#define FTS_REG_RESET_REASON                0xC4
#define FTS_REG_FUNC_POSITION               0xFC

/*
struct
{
        UINT8 type;
        UINT8 value;
        SINT16 mcdiff[16*36];
        SINT16 ScWaterRXdiff[36];
        SINT16 ScWaterTXdiff[36];   ....TX.?..........?.
        SINT16 ScNormalRXdiff[36];
}
*/

#define FTS_CMD_DATA						0x70
#define FTS_CAP_DATA_LEN					(2 + 16*36*2 + (36+36)*2 + 36*2)


/*****************************************************************************
* Static variabls
*****************************************************************************/
struct fts_core *g_fts;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
/*
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
*/
static int rdata_check(u8 *rdata, u32 rlen)
{
/*
	u16 crc_calc = 0;
	u16 crc_read = 0;

	return 0;
	crckermit(rdata, rlen - 2, &crc_calc);
	crc_read = (u16)(rdata[rlen - 1] << 8) + rdata[rlen - 2];
	if (crc_calc != crc_read) {
		hbp_info("CRC check fail,calc(%x)!=read(%x)", crc_calc, crc_read);
		return -EIO;
	}
*/
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

	//udelay(CS_HIGH_DELAY);
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

static int fhp_spi_sync(void *priv, char *tx, char *rx, int32_t len)
{
	struct fts_core *fts = (struct fts_core *)priv;

	return fts->bus_ops->spi_sync(fts->bus_ops, tx, rx, len);
}

static int fhp_spi_set_para(void *priv, uint8_t mode, uint8_t bits_per_word, int speed)
{
	struct fts_core *fts = (struct fts_core *)priv;

	return fts->bus_ops->spi_set_para(fts->bus_ops, mode, bits_per_word, speed);
}

static int fhp_spi_get_para(void *priv, uint8_t *mode, uint8_t *bits_per_word, int *speed)
{
	struct fts_core *fts = (struct fts_core *)priv;

	return fts->bus_ops->spi_get_para(fts->bus_ops, mode, bits_per_word, speed);
}

static int fhp_chip_get_frame(void *priv, u8 *raw, u32 rawsize)
{
	u8 cmd = 0;
	u8 *offset = raw;
	struct fts_core *ts_data = (struct fts_core *)priv;

	cmd = FTS_CMD_DATA;
	fhp_chip_read(ts_data, &cmd, 1, offset, rawsize);

	return 0;
}

static int fhp_read_fod_info(struct fts_core *ts_data, struct fod_info *fod)
{
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

static int fhp_read_fod_error_info(struct fts_core *ts_data)
{
	int ret = 0;
	u8 cmd = FT3681_REG_FOD_ERROR_INFO;
	u8 val[FT3681_REG_FOD_ERROR_INFO_LEN] = { 0 };

	ret = fhp_chip_read(ts_data, &cmd, 1, &val[0], FT3681_REG_FOD_ERROR_INFO_LEN);
	if (ret < 0) {
		hbp_err("TP_FP_ERROR_REPORT:failed to read fod data\n");
		return ret;
	}

	hbp_info("TP_FP_ERROR_REPORT:fingerprint error type:[%*ph]\n", FT3681_REG_FOD_ERROR_INFO_LEN, val);
	switch (val[FT3681_REG_FOD_ERROR_INFO_LEN - 1]) {
	case FTS_FINGERPRINT_AREA_NOT_MATCH:
		/*tp_healthinfo_report(&tcm->monitor_data, HEALTH_REPORT, "fingerprint_area_not_match_count");*/
		hbp_info("TP_FP_ERROR_REPORT:area size: 0x%x\n", val[12]);
		hbp_info("TP_FP_ERROR_REPORT:FINGERPRINT_AREA_NOT_MATCH\n");
		break;
	case FTS_ANOTHER_FINGER_ON_NON_FP_ZONE:
		/*
		tp_healthinfo_report(&tcm->monitor_data, HEALTH_REPORT, "another_finger_on_non-fingerprint_zone_count");
		*/
		hbp_info("TP_FP_ERROR_REPORT:x:0x%x,y:0x%x\n", (val[4] << 8) + val[5], (val[6] << 8) + val[7]);
		hbp_info("TP_FP_ERROR_REPORT:ANOTHER_FINGER_ON_NON_FP_ZONE\n");
		break;
	case FTS_FINGERPRINT_DOWN_BEFORE_FP_ENABLE:
		/*
			tp_healthinfo_report(&tcm->monitor_data, HEALTH_REPORT, "fingerprint_down_before_fp_enable_count");
		*/
		hbp_info("TP_FP_ERROR_REPORT:down time: %*ph\n", 4, val);
		hbp_info("TP_FP_ERROR_REPORT:FINGERPRINT_DOWN_BEFORE_FP_ENABLE\n");
		break;
	case FTS_FINGERPRINT_X_Y_NOT_MATCH:
		hbp_info("TP_FP_ERROR_REPORT:FINGERPRINT_X_Y_NOT_MATCH\n");
		break;
	case FTS_FINGERPRINT_OUT_MOVE_IN:
		hbp_info("TP_FP_ERROR_REPORT:FINGERPRINT_OUT_MOVE_IN\n");
		break;
	default:
		hbp_info("TP_FP_ERROR_REPORT:unknown fingerprint error type: 0x%x\n", val[FT3681_REG_FOD_ERROR_INFO_LEN - 1]);
		break;
	}

	return 0;
}

static int fhp_read_aod_info(struct fts_core *ts_data, struct aod_info *aod)
{
	int ret = 0;
	u8 cmd = FT3681_REG_AOD_INFO;
	u8 val[FT3681_REG_AOD_INFO_LEN] = { 0 };

	ret = fhp_chip_read(ts_data, &cmd, 1, &val[0], FT3681_REG_AOD_INFO_LEN);
	if (ret < 0) {
		hbp_err("failed to read aod data\n");
		return ret;
	}

	hbp_debug("AOD info buffer:%x %x %x %x %x %x", val[0],
		val[1], val[2], val[3], val[4], val[5]);
	aod->gesture_id = val[0];
	aod->point_num = val[1];

	aod->aod_x = (val[2] << 8) + val[3];
	aod->aod_y = (val[4] << 8) + val[5];

	return 0;
}

static int fhp_chip_get_irq_reason(void *priv, enum irq_reason *reason)
{
	int ret = 0;
	u8 reset_reason = 0;
	struct fts_core *fts = (struct fts_core *)priv;

	ret = fhp_chip_read_reg(fts, FTS_REG_RESET_REASON, &reset_reason);
	if (ret < 0) {
		hbp_err("failed to read reset reason");
		return ret;
	}

	//reset == 0, ignore this type
	if (!reset_reason) {
		return 0;
	}

	switch (reset_reason) {
	case FTS_RST_REASON_FWUPDATE:
		*reason = IRQ_REASON_RESET_FWUPDATE;
		hbp_info("hbp chip reset, reason 0x%x(RESET_FWUPDATE)\n", *reason);
		break;
	case FTS_RST_REASON_WDT:
		*reason = IRQ_REASON_RESET_WDT;
		hbp_info("hbp chip reset, reason 0x%x(RESET_WDT)\n", *reason);
		break;
	case FTS_RST_REASON_EXTERNAL:
		*reason = IRQ_REASON_RESET_EXTERNAL;
		hbp_info("hbp chip reset, reason 0x%x(RESET_EXTERNAL)\n", *reason);
		break;
	case FTS_RST_REASON_PWR:
		*reason = IRQ_REASON_RESET_PWR;
		hbp_info("hbp chip reset, reason 0x%x(RESET_PWR)\n", *reason);
		break;
	case FTS_GESTURE_DIFF:
		*reason = IRQ_REASON_GESTURE_DIFF;
		break;
	default:
		*reason = IRQ_REASON_NORMAL;
		break;
	}

	ret = fhp_chip_write_reg(fts, FTS_REG_RESET_REASON, 0x00);
	if (ret < 0) {
		hbp_err("failed to clear reset reason");
	}

	return 0;
}

static u8 fhp_chip_get_reset_reason(void *priv)
{
	int ret = 0;
	u8 reset_reason = 0;
	struct fts_core *fts = (struct fts_core *)priv;

	ret = fhp_chip_read_reg(fts, FTS_REG_RESET_REASON, &reset_reason);
	hbp_info("reset_reason: %d", reset_reason);
	return reset_reason;
}

static void fhp_chip_get_func_position(void *priv)
{
	int ret = 0;
	struct fts_core *fts = (struct fts_core *)priv;
	u8 data[48] = {0};
	u8 cmd = FTS_REG_FUNC_POSITION;

	ret = fhp_chip_read(fts, &cmd, 1, &data[0], sizeof(data));
	if (ret < 0) {
		hbp_err("failed to read aod data\n");
		return;
	}
	hbp_info("func position: %*ph", 48, data);
}

static int fhp_chip_get_gesture(void *priv, struct gesture_info *gesture)
{
	int ret = 0;
	u8 buf[FTS_GESTURE_DATA_LEN] = { 0 };
	u8 cmd = 0;
	u8 gesture_id = 0;
	u8 gesture_pointnum = 0;
	u8 reset_reason = 0;
	struct fod_info fod;
	struct aod_info aod;
	struct fts_core *fts = (struct fts_core *)priv;

	ret = fhp_chip_read_reg(fts, FTS_REG_GESTURE_EN, &buf[0]);
	if ((ret < 0) || (buf[0] != 1)) {
		hbp_err("gesture not enable in fw, don't process gesture");
		reset_reason = fhp_chip_get_reset_reason(priv);
		if (reset_reason != FTS_RST_REASON_UNKNOWN && reset_reason != FTS_RST_REASON_FWUPDATE) {
			fhp_chip_get_func_position(priv);
			hbp_exception_report(EXCEP_GESTURE, "gesture not enable", sizeof("gesture not enable"));
		}
		return 1;
	}

	cmd = FTS_REG_GESTURE_OUTPUT_ADDRESS;
	ret = fhp_read(&cmd, 1, &buf[2], FTS_GESTURE_DATA_LEN - 2);
	if (ret < 0) {
		hbp_err("read gesture data fail");
		hbp_exception_report(EXCEP_GESTURE, "read gesture data fail", sizeof("read gesture data fail"));
		return ret;
	}

	/*get gesture id, points information*/
	gesture_id = buf[2];
	gesture_pointnum = buf[3];
	hbp_info("Gesture ID:%d, point_num:%d", gesture_id, gesture_pointnum);

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
		ret = fhp_read_fod_info(fts, &fod);
		if (ret < 0) {
			hbp_err("failed to read fod info\n");
			hbp_exception_report(EXCEP_FINGERPRINT, "failed to read fod info", sizeof("failed to read fod info"));
			return ret;
		}

		if (fod.event_type == GESTURE_FINGER_PRINT) {
			if (fod.fp_down) {
				gesture->type = FingerprintDown;
			} else {
				gesture->type = FingerprintUp;
			}
			gesture->Point_start.x = fod.fp_x;
			gesture->Point_start.y = fod.fp_y;
			gesture->Point_end.x = fod.fp_area_rate;
			gesture->Point_end.y = 0;
			gesture->tp_firmware_time = fod.fp_time;
		}
		break;
	case GESTURE_FINGER_PRINT_ERROR:
		ret = fhp_read_fod_error_info(fts);
		if (ret < 0) {
				hbp_err("failed to read fod error info\n");
				return ret;
		}
		gesture->type = UnknownGesture;
		break;
	default:
		gesture->type = UnknownGesture;
	}

	if (gesture->type == SingleTap || gesture->type == DoubleTap) {
		ret = fhp_read_aod_info(fts, &aod);
		if (ret < 0) {
			hbp_err("failed to read aod info\n");
			hbp_exception_report(EXCEP_GESTURE, "failed to read aod", sizeof("failed to read aod"));
			return ret;
		}
		gesture->Point_start.x = aod.aod_x;
		gesture->Point_start.y = aod.aod_y;
		hbp_debug("AOD event type:0x%x", aod.gesture_id);
		hbp_debug("AOD Point_start.x:%d", gesture->Point_start.x);
		hbp_info("AOD event type:0x%x", aod.gesture_id);
	}

	if ((gesture->type != FingerprintDown)
			&& (gesture->type != FingerprintUp)
			&& (gesture->type != UnknownGesture)) {
		gesture->Point_start.x = (u16)((buf[4] << 8) + buf[5]);
		gesture->Point_start.y = (u16)((buf[6] << 8) + buf[7]);
		gesture->Point_end.x = (u16)((buf[8] << 8) + buf[9]);
		gesture->Point_end.y = (u16)((buf[10] << 8) + buf[11]);
		gesture->Point_1st.x = (u16)((buf[12] << 8) + buf[13]);
		gesture->Point_1st.y = (u16)((buf[14] << 8) + buf[15]);
		gesture->Point_2nd.x = (u16)((buf[16] << 8) + buf[17]);
		gesture->Point_2nd.y = (u16)((buf[18] << 8) + buf[19]);
		gesture->Point_3rd.x = (u16)((buf[20] << 8) + buf[21]);
		gesture->Point_3rd.y = (u16)((buf[22] << 8) + buf[23]);
		gesture->Point_4th.x = (u16)((buf[24] << 8) + buf[25]);
		gesture->Point_4th.y = (u16)((buf[26] << 8) + buf[27]);
	}

	return 0;
}

static int fhp_chip_get_touch_points(void *priv, struct point_info *points)
{
	int ret = 0;
	int i = 0;
	int base = 0;
	int event_num = 0;
	int obj_attention = 0;
	u8 touch_buf[FTS_MAX_POINTS_LENGTH] = { 0xFF };
	u8 cmd = FTS_REG_POINTS;
	u8 touch_etype = 0;
	u8 finger_num = 0;
	u8 event_flag = 0;
	u8 pointid = 0;
	struct fts_core *fts = (struct fts_core *)priv;

	ret = fhp_chip_read(fts, &cmd, 1, touch_buf, FTS_MAX_POINTS_LENGTH);
	if (ret < 0) {
		hbp_err("read touch point one fail");
		return ret;
	}

	touch_etype = ((touch_buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F);

	switch (touch_etype) {
	case TOUCH_DEFAULT:
		finger_num = touch_buf[1] & 0xFF;

		if (finger_num > FTS_MAX_POINTS_SUPPORT) {
			hbp_err("invalid point_num(%d),max_num(%d)", finger_num, FTS_MAX_POINTS_SUPPORT);
			return -EIO;
		}

		for (i = 0; i < FTS_MAX_POINTS_SUPPORT; i++) {
			base = 6 * i;
			pointid = (touch_buf[4 + base]) >> 4;

			if (pointid >= FTS_MAX_ID) {
				break;

			} else if (pointid >= FTS_MAX_POINTS_SUPPORT) {
				hbp_err("ID(%d) beyond max_num(%d)", pointid, FTS_MAX_POINTS_SUPPORT);
				return -EINVAL;
			}

			event_num++;

			points[pointid].x = (((touch_buf[2 + base] & 0x0F) << 11) +
									((touch_buf[3 + base] & 0xFF) << 3) +
									((touch_buf[6 + base] >> 5) & 0x07));
			points[pointid].y = (((touch_buf[4 + base] & 0x0F) << 11) +
									((touch_buf[5 + base] & 0xFF) << 3) +
									((touch_buf[6 + base] >> 2) & 0x07));
			points[pointid].touch_major = touch_buf[7 + base];
			points[pointid].width_major = touch_buf[7 + base];
			points[pointid].z =  touch_buf[7 + base];
			event_flag = (touch_buf[2 + base] >> 6);

			points[pointid].status = 0;

			if ((event_flag == 0) || (event_flag == 2)) {
				points[pointid].status = 1;
				obj_attention |= (1 << pointid);

				if (finger_num == 0) {
					hbp_err("abnormal touch data from fw");
					return -EIO;
				}
			}
		}

		if (event_num == 0) {
			hbp_err("no touch point information");
			return -EIO;
		}
		break;

	case TOUCH_PROTOCOL_v2:
		event_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (!event_num || (event_num > FTS_MAX_POINTS_SUPPORT)) {
			hbp_err("invalid touch event num(%d)", event_num);
			return -EINVAL;
		}

		/*ts_data->touch_event_num = event_num;*/

		for (i = 0; i < event_num; i++) {
			base = FTS_ONE_TCH_LEN_V2 * i + 4;
			pointid = (touch_buf[FTS_TOUCH_OFFSET_ID_YH + base]) >> 4;
			if (pointid >= FTS_MAX_POINTS_SUPPORT) {
				hbp_err("touch point ID(%d) beyond max_touch_number(%d)",
							pointid, FTS_MAX_POINTS_SUPPORT);
				return -EINVAL;
			}

			/*points[i].id = pointid;*/
			event_flag = touch_buf[FTS_TOUCH_OFFSET_E_XH + base] >> 6;

			points[pointid].x = ((touch_buf[FTS_TOUCH_OFFSET_E_XH + base] & 0x0F) << 12) \
							+ ((touch_buf[FTS_TOUCH_OFFSET_XL + base] & 0xFF) << 4) \
							+ ((touch_buf[FTS_TOUCH_OFFSET_PRE + base] >> 4) & 0x0F);

			points[pointid].y = ((touch_buf[FTS_TOUCH_OFFSET_ID_YH + base] & 0x0F) << 12) \
							+ ((touch_buf[FTS_TOUCH_OFFSET_YL + base] & 0xFF) << 4) \
							+ (touch_buf[FTS_TOUCH_OFFSET_PRE + base] & 0x0F);

			/*points[pointid].x = points[pointid].x  / FTS_HI_RES_X_MAX;*/
			/*points[pointid].y = points[pointid].y  / FTS_HI_RES_X_MAX;*/
			points[pointid].touch_major = touch_buf[FTS_TOUCH_OFFSET_AREA + base];
			points[pointid].width_major = touch_buf[FTS_TOUCH_OFFSET_AREA + base];
			points[pointid].z = touch_buf[FTS_TOUCH_OFFSET_AREA + base];
			/*if (ts_data->ft3683_grip_v2_support) {
				if (pointid < 7) {
					points[pointid].tx_press = touch_buf[94 + base_prevent];
					points[pointid].rx_press = touch_buf[95 + base_prevent];
					points[pointid].tx_er = touch_buf[97 + base_prevent];
					points[pointid].rx_er = touch_buf[96 + base_prevent];
				} else {
					points[pointid].tx_press = 0;
					points[pointid].rx_press = 0;
					points[pointid].tx_er = 0;
					points[pointid].rx_er = 0;
				}
			}*/

			if (points[pointid].touch_major <= 0) {
				points[pointid].touch_major = 0x09;
			}

			if (points[pointid].width_major <= 0) {
				points[pointid].width_major = 0x09;
			}

			points[pointid].status = 0;

			if ((event_flag == 0) || (event_flag == 2)) {
				points[pointid].status = 1;
				obj_attention |= (1 << pointid);

				if (event_num == 0) {
					hbp_err("abnormal touch data from fw");
					return -EINVAL;
				}
			}
		}

		break;
	default:
		break;
	}

	return obj_attention;
}

static int fhp_chip_enable_hbp_mode(void *priv, bool en)
{
	int ret = 0;
	struct fts_core *fts = (struct fts_core *)priv;

	ret = fhp_chip_write_reg(fts, FTS_REG_REPORT_MODE, en ? FTS_REPORT_MODE_HBP : FTS_REPORT_MODE_LBP);
	if (ret < 0) {
		hbp_err("%s mode enable failed!!", en ? "hbp" : "lbp");
		return ret;
	}

	hbp_info("%s mode enable success", en ? "hbp" : "lbp");
	return 0;
}

/*****************************************************************************
 * Name: fhp_chip_spi_sync_proc
 * Brief: The function is used to transfer SPI values directly. For Proc.
 *
 * Input: @ts_data:
 *        @writebuf: write buffer.
 *        @writelen: size of write buffer.
 *        @msg_rbuf: buffer to store all SPI RX values.
 *        @msg_rlen: the size of values in the SPI message.
 * Output:
 * Return: return 0 if success, otherwise return error code.
 *****************************************************************************/
static int fhp_chip_spi_sync_proc(
	struct fts_core *ts_data,
	u8 *writebuf, u32 writelen, u8 *msg_rbuf, u32 msg_rlen)
{
	int ret = 0;
	u8 *txbuf = ts_data->bus_tx_buf;
	u8 *rxbuf = ts_data->bus_rx_buf;
	bool read_cmd = (msg_rbuf && msg_rlen);
	u32 txlen = (read_cmd) ? msg_rlen : writelen;

	if (!writebuf || !writelen || (msg_rlen > PAGE_SIZE)) {
		hbp_err("writebuf/writelen(%d) is invalid", writelen);
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_mutex);
	memcpy(txbuf, writebuf, writelen);
	ret = fhp_spi_sync(ts_data, txbuf, rxbuf, txlen);
	if (ret < 0) {
		hbp_err("data read(addr:%x) fail,status:%x,ret:%d", txbuf[0], rxbuf[3], ret);
	} else {
		if (read_cmd) {
			memcpy(msg_rbuf, rxbuf, txlen);
		}
	}

	udelay(CS_HIGH_DELAY);
	mutex_unlock(&ts_data->bus_mutex);
	return ret;
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
#if 1
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	u8 *writebuf = NULL;
	u8 tmpbuf[PROC_BUF_SIZE] = { 0 };
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	char tmp[PROC_BUF_SIZE];
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
    struct fts_core *ts_data = pde_data(file_inode(filp));
#else
    struct fts_core *ts_data = PDE_DATA(file_inode(filp));
#endif
	struct ftxxxx_proc *proc = &ts_data->proc;

	if (buflen <= 1) {
		hbp_err("apk proc wirte count(%d) fail", buflen);
		return -EINVAL;
	}

	if (buflen > PROC_BUF_SIZE) {
		writebuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
		if (NULL == writebuf) {
			hbp_err("apk proc wirte buf zalloc fail");
			return -ENOMEM;
		}
	} else {
		writebuf = tmpbuf;
	}

	if (copy_from_user(writebuf, buff, buflen)) {
		hbp_err("[APK]: copy from user error!!");
		ret = -EFAULT;
		goto proc_write_err;
	}

	hbp_info("write buf is %s", writebuf);

	hbp_debug("write opmode %d\n", writebuf[0]);

	proc->opmode = writebuf[0];
	switch (proc->opmode) {
	case PROC_READ_REGISTER:
		proc->cmd[0] = writebuf[1];
		break;

	case PROC_WRITE_REGISTER:
		ret = fhp_chip_write(ts_data, &writebuf[1], 2);
		if (ret < 0) {
			hbp_err("PROC_WRITE_REGISTER write error");
			goto proc_write_err;
		}
		break;

	case PROC_READ_DATA:
		writelen = buflen - 1;
		if (writelen >= FTS_MAX_COMMMAND_LENGTH) {
			hbp_err("cmd(PROC_READ_DATA) length(%d) fail", writelen);
			goto proc_write_err;
		}
		memcpy(proc->cmd, writebuf + 1, writelen);
		proc->cmd_len = writelen;
		break;

	case PROC_WRITE_DATA:
		writelen = buflen - 1;
		ret = fhp_chip_write(ts_data, writebuf + 1, writelen);
		if (ret < 0) {
			hbp_err("PROC_WRITE_DATA write error");
			goto proc_write_err;
		}
		break;

	case PROC_HW_RESET:
		snprintf(tmp, PROC_BUF_SIZE, "%s", writebuf + 1);
		tmp[buflen - 1] = '\0';
		hbp_info("PROC_HW_RESET data is : %s", tmp);
		if (strncmp(tmp, "focal_driver", 12) == 0) {
			hbp_info("APK execute HW Reset");
			fhp_chip_write_reg(ts_data, 0xB6, 0x01);
			//fhp_reset(fhp_data, 0);
		}
		break;

	case PROC_READ_DATA_DIRECT:
		writelen = buflen - 1;
		if (writelen >= FTS_MAX_COMMMAND_LENGTH) {
			hbp_err("cmd(PROC_READ_DATA_DIRECT) length(%d) fail", writelen);
			goto proc_write_err;
		}
		memcpy(proc->cmd, writebuf + 1, writelen);
		proc->cmd_len = writelen;
		break;

	case PROC_WRITE_DATA_DIRECT:
		writelen = buflen - 1;
		ret = fhp_chip_spi_sync_proc(ts_data, writebuf + 1, writelen, NULL, 0);
		if (ret < 0) {
			hbp_err("PROC_WRITE_DATA_DIRECT write error");
			goto proc_write_err;
		}
		break;

	case PROC_CONFIGURE:
		ts_data->bus_ops->spi_set_para(ts_data->bus_ops, writebuf[1], writebuf[2], *(u32 *)(writebuf + 4));
		break;
	default:
		break;
	}

	ret = buflen;
proc_write_err:
	if ((buflen > PROC_BUF_SIZE) && writebuf) {
		kfree(writebuf);
		writebuf = NULL;
	}
	return ret;
}

static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int buflen = count;
	u8 *readbuf = NULL;
	u8 tmpbuf[PROC_BUF_SIZE] = { 0 };
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
    struct fts_core *ts_data = pde_data(file_inode(filp));
#else
    struct fts_core *ts_data = PDE_DATA(file_inode(filp));
#endif
	struct ftxxxx_proc *proc = &ts_data->proc;

	if (buflen <= 0) {
		hbp_err("apk proc read count(%d) fail", buflen);
		return -EINVAL;
	}

	if (buflen > PROC_BUF_SIZE) {
		readbuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
		if (NULL == readbuf) {
			hbp_err("apk proc wirte buf zalloc fail");
			return -ENOMEM;
		}
	} else {
		readbuf = tmpbuf;
	}

	hbp_debug("read opmode %d\n", proc->opmode);

	switch (proc->opmode) {
	case PROC_READ_REGISTER:
		num_read_chars = 1;
		ret = fhp_chip_read(ts_data, proc->cmd, 1, &readbuf[0], 1);
		if (ret < 0) {
			hbp_err("PROC_READ_REGISTER read error");
			goto proc_read_err;
		}
		break;
	case PROC_WRITE_REGISTER:
		break;

	case PROC_READ_DATA:
		num_read_chars = buflen;
		ret = fhp_chip_read(ts_data, proc->cmd, proc->cmd_len, readbuf, num_read_chars);
		if (ret < 0) {
			hbp_err("PROC_READ_DATA read error");
			goto proc_read_err;
		}
		break;

	case PROC_READ_DATA_DIRECT:
		num_read_chars = buflen;
		ret = fhp_chip_spi_sync_proc(ts_data, proc->cmd, proc->cmd_len, readbuf, num_read_chars);
		if (ret < 0) {
			hbp_err("PROC_READ_DATA_DIRECT read error");
			goto proc_read_err;
		}
		break;

	case PROC_WRITE_DATA:
		break;

	default:
		break;
	}

	ret = num_read_chars;
proc_read_err:
	if (copy_to_user(buff, readbuf, num_read_chars)) {
		hbp_err("copy to user error");
		ret = -EFAULT;
	}

	if ((buflen > PROC_BUF_SIZE) && readbuf) {
		kfree(readbuf);
		readbuf = NULL;
	}
	return ret;
}

#if LINUX_VERSION_CODE>= KERNEL_VERSION(5, 10, 0)
static const struct proc_ops fts_proc_fops = {
	.proc_read   = fts_debug_read,
	.proc_write  = fts_debug_write,
};
#else
static const struct file_operations fts_proc_fops = {
	.owner  = THIS_MODULE,
	.read   = fts_debug_read,
	.write  = fts_debug_write,
};
#endif

static int fhp_chip_debug_init(struct fts_core *ts_data)
{
	struct ftxxxx_proc *proc = &ts_data->proc;

	proc->proc_entry = proc_create_data(PROC_NAME, 0777, NULL, &fts_proc_fops, ts_data);
	if (!proc->proc_entry) {
		hbp_err("create proc entry fail");
		return -ENOMEM;
	}

	hbp_info("Create proc entry success");
	return 0;
}
#endif

struct dev_operations fts_ops = {
	.spi_sync = fhp_spi_sync,
	.spi_set_para = fhp_spi_set_para,
	.spi_get_para = fhp_spi_get_para,
	.get_frame = fhp_chip_get_frame,
	.get_gesture = fhp_chip_get_gesture,
	.get_touch_points = fhp_chip_get_touch_points,
	.get_irq_reason = fhp_chip_get_irq_reason,
	.enable_hbp_mode = fhp_chip_enable_hbp_mode,
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
	if (!fts->bus_tx_buf || !fts->bus_tx_buf) {
		return -ENOMEM;
	}
	g_fts = fts;

	ret = hbp_register_devices(fts, &pdev->dev, &fts_ops, &info, &fts->bus_ops);
	if (ret < 0) {
		hbp_err("failed to register device:%s %d\n", info.vendor, ret);
		goto err_exit;
	}

	fhp_chip_debug_init(fts);

	hbp_info("probe end.\n");
	return 0;

err_exit:
	kfree(fts->bus_rx_buf);
	kfree(fts->bus_tx_buf);
	kfree(fts);
	g_fts = NULL;
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void fts_dev_remove(struct platform_device *spi)
#else
static int fts_dev_remove(struct platform_device *spi)
#endif
{
	hbp_info("fts_dev_remove.\n");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
#else
	return 0;
#endif
}

static const struct of_device_id fts_dt_match[] = {
	{.compatible = "focaltech,ft3683g", },
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

static int __init fts_platform_init(void)
{
	return platform_driver_register(&fts_dev_driver);
}

late_initcall(fts_platform_init);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech FHP Driver");
MODULE_LICENSE("GPL v2");
