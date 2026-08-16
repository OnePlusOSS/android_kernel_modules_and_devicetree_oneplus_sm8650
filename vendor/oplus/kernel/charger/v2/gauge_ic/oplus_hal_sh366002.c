/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_hal_sh366002.c
** Description: gauge ic
** Date: 2025-11-18
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#define pr_fmt(fmt) "[SH366002]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/version.h>
#include <linux/acpi.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/random.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <oplus_chg_monitor.h>

#include "oplus_hal_sh366002.h"

#define SH366002_MODEL_NAME		"sino_default_cell_model"

#define DUMP_SH366002_BLOCK		BIT(0)
#define DUMP_SH366002_CURRENT_BLOCK	BIT(1)

#define GAUGE_LOG_MIN_TIMESPAN		10

#define CMD_CNTLSTATUS_SEAL		0x6000
#define CMD_UNSEALKEY			0x19861115
#define CMD_FULLKEY			0xFFFFFFFF

#define AFI_DATE_BLOCKA_INDEX		24
#define AFI_MANU_BLOCKA_INDEX		28
#define AFI_MANU_SINOWEALTH		0x02
#define CHECK_VERSION_ERR		-1
#define CHECK_VERSION_OK		0
#define CHECK_VERSION_FW		BIT(0)
#define CHECK_VERSION_AFI		BIT(1)
#define CHECK_VERSION_TS		BIT(2)
#define CHECK_VERSION_WHOLE_CHIP	(CHECK_VERSION_FW | CHECK_VERSION_AFI | CHECK_VERSION_TS)
#define FILE_DECODE_RETRY		2
#define FILE_DECODE_DELAY		100
#define CADC_OFFSET_RETRY		3

#define WRITE_BUF_MAX_LEN		32
#define GAUGE_LOG_MIN_TIMESPAN		10
#define MAX_BUF_LEN			1024
#define DF_PAGE_LEN			32
#define DF_PAGE_HALF_LEN		16
#define CMD_SBS_DELAY			3
#define CMD_E2ROM_DELAY			1500
#define CMDMASK_MASK			0xFF000000
#define CMDMASK_SINGLE			0x01000000
#define CMDMASK_WRITE			0x80000000
#define CMDMASK_CNTL_R			0x08000000
#define CMDMASK_CNTL_W			(CMDMASK_WRITE | CMDMASK_CNTL_R)
#define CMDMASK_MANUBLOCK_R		0x04000000
#define CMDMASK_MANUBLOCK_W		(CMDMASK_WRITE | CMDMASK_MANUBLOCK_R)
#define CMDMASK_RAMBLOCK_R		0x02000000

#define CMD_CNTLSTATUS			(CMDMASK_CNTL_R | 0x0000)
#define CMD_FORCE_UPDATE_E2ROM		0x55FF
#define CMD_CNTL			0x00
#define CMD_SEAL			0x0020
#define CMD_DFSTART			0x61
#define CMD_DFCLASS			0x3E
#define CMD_DFPAGE			0x3F
#define CMD_BLOCK			0x40
#define CMD_CHECKSUM			0x60
#define CMD_BLOCKA			(CMDMASK_MANUBLOCK_W | 0x01)
#define CMD_FWDATE1			(CMDMASK_CNTL_R | 0xD0)
#define CMD_FWDATE2			(CMDMASK_CNTL_R | 0xE2)

#define CMD_TEMPER			0x06
#define CMD_VOLTAGE			0x08
#define CMD_CURRENT			0x14
#define CMD_FILRC			0x10
#define CMD_FILFCC			0x12
#define CMD_CYCLECOUNT			0x2A
#define CMD_PASSEDC			0x34
#define CMD_DOD0			0x36
#define CMD_PACKCON			0x3A
#define CMD_GAUGE_ENABLE		0x21
#define CMD_GAUGE_DELAY			500
#define CMD_CELLMODEL			(CMDMASK_CNTL_R | 0xE6)
#define CELL_MODEL_COUNT		15
#define BYTE_COUNT_CELL_MODEL		32

#define CMD_GAUGEBLOCK1			(CMDMASK_CNTL_R | 0x00E3)
#define CMD_GAUGEBLOCK2			(CMDMASK_CNTL_R | 0x00E4)
#define CMD_GAUGEBLOCK3			(CMDMASK_CNTL_R | 0x00E5)
#define CMD_GAUGEBLOCK4			(CMDMASK_CNTL_R | 0x00E6)
#define CMD_GAUGEBLOCK5			(CMDMASK_CNTL_R | 0x00E7)
#define CMD_GAUGEBLOCK6			(CMDMASK_RAMBLOCK_R | 0x002B)
#define CMD_CURRENTBLOCK1		(CMDMASK_CNTL_R | 0x00EA)
#define CMD_CURRENTBLOCK2		(CMDMASK_CNTL_R | 0x00EB)
#define CMD_CURRENTBLOCK3		(CMDMASK_CNTL_R | 0x00EC)
#define CMD_CURRENTBLOCK4		(CMDMASK_CNTL_R | 0x00ED)

#define GAUGEINFO_LEN			32
#define GAUGESTR_LEN			512
#define U64_MAXVALUE			0xFFFFFFFFFFFFFFFF
#define TEMPER_OFFSET			2731

#define IIC_ADDR_OF_2_KERNEL(addr)	((u8)((u8)addr >> 1))

/* file_decode_process */
#define OPERATE_READ			1
#define OPERATE_WRITE			2
#define OPERATE_COMPARE			3
#define OPERATE_WAIT			4

#define ERRORTYPE_NONE			0
#define ERRORTYPE_ALLOC			1
#define ERRORTYPE_LINE			2
#define ERRORTYPE_COMM			3
#define ERRORTYPE_COMPARE		4
#define ERRORTYPE_FINAL_COMPARE		5

/* delay: b0: operate, b1: 2, b2-b3: time, big-endian */
/* other: b0: operate, b1: TWIADR, b2: reg, b3: data_length, b4...end: data */
#define INDEX_TYPE			0
#define INDEX_ADDR			1
#define INDEX_REG			2
#define INDEX_LENGTH			3
#define INDEX_DATA			4
#define INDEX_WAIT_LENGTH		1
#define INDEX_WAIT_HIGH			2
#define INDEX_WAIT_LOW			3
#define LINELEN_WAIT			4
#define LINELEN_READ			4
#define LINELEN_COMPARE			4
#define LINELEN_WRITE			4
#define FILEDECODE_STRLEN		128
#define COMPARE_RETRY_CNT		2
#define COMPARE_RETRY_WAIT		50
#define BUF_MAX_LENGTH			512

#define E2ROM_DELAY_MS			1500
#define ADDR_PACKCONFIG			0x4000
#define LENGTH_PACKCONFIG		2
#define PACKCONFIG_SLEEP_MASK		0x20
#define CMD_DFCONFIGVERSION		(CMDMASK_CNTL_R | 0x0C)
#define DFCONFIG_CALIED			0xAA
#define ADDR_DEADBAND			0x6B01
#define LENGTH_DEADBAND			1
#define DEFAULT_DEADBAND		3
#define ADDR_PACKLOTCODE		0x3800
#define ADDR_CADCOFFSET			0x6802
#define LENGTH_CADCOFFSET		2

/* fg_gauge_calibrate_board */
#define CMD_TEMPERATURE			0x06
#define MAXTEMPER			(2731 + 500)
#define MINTEMPER			(2731 + 100)
#define ADDR_DFCONFIGVERSION		0x380A
#define LEN_DFCONFIGVERSION		2
#define MAX_CADCOFFSET			120
#define ADDR_CADCRATIO			0x6800
#define LENGTH_CADCRATIO		4
#define INDEX_CADCRATIO			0
#define INDEX_CADCOFFSET		2
#define BASE_CADCRATIO			16384
#define DELAY_CURRENT			1000
#define CALI_MAX_CNT			6
#define CALI_DELAY_MS			1000
#define CALI_MAX_CURRENT		12
#define CALI_DELTA_CURRENT		4
#define CALI_DEADBAND			4

/* fg_gauge_check_cell_model */
#define MODELFLAG_NONE			0
#define MODELFLAG_NEW_SMALL		0x0010
#define MODELFLAG_NEW_LARGE		0x0020
#define MODELFLAG_NEW_EXTREME		0x0040
#define MODELFLAG_NEW_SLIGHT		0x0080
#define TOOLARGE_STARTGRID		0
#define TOOLARGE_ENDGRID		10
#define TOOLARGE_MAXVAULE		3000
#define TOOLARGE_RATIO			1000
#define TOOSMALL_STARTGRID_0		0
#define TOOSMALL_ENDGRID_0		4
#define TOOSMALL_MINVAULE_0		5
#define TOOSMALL_STARTGRID_1		5
#define TOOSMALL_ENDGRID_1		10
#define TOOSMALL_MINVAULE_1		10
#define TOOSMALL_STARTGRID_2		11
#define TOOSMALL_ENDGRID_2		14
#define TOOSMALL_MINVAULE_2		30
#define SLIGHT_STARTGRID		0
#define SLIGHT_ENDGRID			10
#define SLIGHT_GAP			5
#define SLIGHT_RATIO			1000
#define SLIGHT_MAXRATIO			3000
#define SLIGHT_MAXCNT			2
#define EXTREME_STARTGRID_0		0
#define EXTREME_ENDGRID_0		12
#define EXTREME_GAP_0			5
#define EXTREME_RATIO			1000
#define EXTREME_MAXRATIO_0		4500
#define EXTREME_MAXCNT_0		1
#define EXTREME_STARTGRID_1		12
#define EXTREME_ENDGRID_1		14
#define EXTREME_GAP_1			2
#define EXTREME_MAXRATIO_1		8000
#define EXTREME_MAXRATIO_1_PRO		16000
#define OVER_MAXRATIO_MAGICDATA		0x5AA5
#define EXTREME_MAXRATIO_CHECK		5500
#define EXTREME_MAXCNT_1		1
#define EXTREME_STARTGRID_2		12
#define EXTREME_ENDGRID_2		13
#define EXTREME_MAXRATIO_2		1250

/* fg_gauge_restore_cell_model */
#define ADDR_CELLMODEL			0x7B00
#define LENGTH_CELLMODEL		64
#define INDEX_CELLMODEL			0
#define INDEX_XCELLMODEL		32
#define FLAG_XCELLMODEL			0xFFFF
#define STRLEN				200
#define MODELRATIO_BASE			1000
#define MAX_MODEL			0x7FFF
#define MIN_MODEL			0x0F

/* fg_gauge_check_por_soc */
#define MAX_ABS_CURRENT			200
#define CMD_BLOCK3			(CMDMASK_MANUBLOCK_R | 0x03)
#define ADDR_BLOCK3			0x3A40
#define INDEX_POR_FLAG			0
#define LENGTH_POR_FLAG			1
#define POR_FLAG_TRIGGER		0xAA
#define POR_FLAG_DEFAULT		0
#define CMD_CYCLE_MODEL			(CMDMASK_RAMBLOCK_R | 0x002B)
#define INDEX_CYCLE_MODEL		28
#define LENGTH_CYCLE_MODEL		2
#define CYCLE_MODEL_DIFFER		20
#define CMD_IAP_DEVICE			0xA0
#define IAP_DEVICE_ID			0x3602

#define SH366002_IMP_MODEL_LEN		512
#define SH366002_UNSEAL_TRY_COUNTS	5

static int sh366002_track_model_data_update(struct chip_bq27541 *chip, char *buf);
static int sh366002_track_model_data_init(struct chip_bq27541 *chip);

static s32 __fg_read_byte(struct chip_bq27541 *chip, u8 reg, u8 *val)
{
	s32 ret;

	if (!chip || !chip->client) {
		chg_err("chip or chip->client NULL,return\n");
		return 0;
	}

	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		chg_err("i2c read byte fail: can't read from reg 0x%02X\n", reg);
		mutex_unlock(&chip->chip_mutex);
		return ret;
	}
	mutex_unlock(&chip->chip_mutex);
	*val = (u8)ret;

	return 0;
}

static s32 __fg_write_byte(struct chip_bq27541 *chip, u8 reg, u8 val)
{
	s32 ret;

	if (!chip || !chip->client) {
		chg_err("chip or chip->client NULL,return\n");
		return 0;
	}

	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		chg_err("i2c write byte fail: can't write 0x%02X to reg 0x%02X\n", val, reg);
		mutex_unlock(&chip->chip_mutex);
		return ret;
	}
	mutex_unlock(&chip->chip_mutex);

	return 0;
}

static s32 __fg_read_word(struct chip_bq27541 *chip, u8 reg, u16 *val)
{
	s32 ret;

	if (!chip || !chip->client) {
		chg_err("chip or chip->client NULL,return\n");
		return 0;
	}

	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	ret = i2c_smbus_read_word_data(chip->client, reg);
	if (ret < 0) {
		chg_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		mutex_unlock(&chip->chip_mutex);
		return ret;
	}
	mutex_unlock(&chip->chip_mutex);
	*val = (u16)ret;

	return 0;
}

static s32 __fg_write_word(struct chip_bq27541 *chip, u8 reg, u16 val)
{
	s32 ret;

	if (!chip || !chip->client) {
		chg_err("chip or chip->client NULL,return\n");
		return 0;
	}

	if (oplus_is_rf_ftm_mode())
		return 0;

	mutex_lock(&chip->chip_mutex);
	ret = i2c_smbus_write_word_data(chip->client, reg, val);
	if (ret < 0) {
		chg_err("i2c write word fail: can't write 0x%02X to reg 0x%02X\n", val, reg);
		mutex_unlock(&chip->chip_mutex);
		return ret;
	}
	mutex_unlock(&chip->chip_mutex);

	return 0;
}

static s32 __fg_read_buffer(struct chip_bq27541 *chip, u8 reg, u8 length, u8 *val)
{
	struct i2c_msg msg[2];
	s32 ret;

	if (!chip || !chip->client || !chip->client->adapter)
		return -ENODEV;

	if (oplus_is_rf_ftm_mode())
		return 0;

	msg[0].addr = chip->client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(u8);
	msg[1].addr = chip->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = val;
	msg[1].len = length;

	mutex_lock(&chip->chip_mutex);
	ret = (s32)i2c_transfer(chip->client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&chip->chip_mutex);
	return ret;
}

static s32 __fg_write_buffer(struct chip_bq27541 *chip, u8 reg, u8 length, u8 *val)
{
	struct i2c_msg msg[1];
	u8 write_buf[WRITE_BUF_MAX_LEN];
	s32 ret;

	if (!chip || !chip->client || !chip->client->adapter)
		return -ENODEV;

	if (oplus_is_rf_ftm_mode())
		return 0;

	if ((length <= 0) || (length + 1 >= WRITE_BUF_MAX_LEN)) {
		chg_err("i2c write buffer fail: length invalid!\n");
		return -1;
	}

	memset(write_buf, 0, WRITE_BUF_MAX_LEN * sizeof(u8));
	write_buf[0] = reg;
	memmove(&write_buf[1], val, length);

	msg[0].addr = chip->client->addr;
	msg[0].flags = 0;
	msg[0].buf = write_buf;
	msg[0].len = sizeof(u8) * (length + 1);

	mutex_lock(&chip->chip_mutex);
	ret = i2c_transfer(chip->client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		chg_err("i2c write buffer fail: can't write reg 0x%02X\n", reg);
		mutex_unlock(&chip->chip_mutex);
		return (s32)ret;
	}
	mutex_unlock(&chip->chip_mutex);

	return 0;
}

static s32 fg_read_sbs_word(struct chip_bq27541 *chip, u32 reg, u16 *val)
{
	s32 ret = -1;

	if ((reg & CMDMASK_CNTL_R) == CMDMASK_CNTL_R) {
		mutex_lock(&chip->bq28z610_alt_manufacturer_access);
		ret = __fg_write_word(chip, CMD_CNTL, (u16)reg);
		if (ret < 0) {
			mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
			return ret;
		}

		mdelay(CMD_SBS_DELAY);
		ret = __fg_read_word(chip, CMD_CNTL, val);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	} else {
		ret = __fg_read_word(chip, (u8)reg, val);
	}

	return ret;
}

static int fg_write_sbs_word(struct chip_bq27541 *chip, u32 reg, u16 val)
{
	int ret;

	ret = __fg_write_word(chip, (u8)reg, val);

	return ret;
}

static s32 fg_read_sbs_word_then_check(struct chip_bq27541 *chip, u32 reg, u16* val)
{
	s32 ret = -1;
	s32 dat0, dat1, dat2;

	ret = fg_read_sbs_word(chip, reg, val);
	dat0 = (ret < 0) ? -1 : *val;
	msleep(CMD_SBS_DELAY);

	ret = fg_read_sbs_word(chip, reg, val);
	msleep(CMD_SBS_DELAY);
	dat1 = (ret < 0) ? -1 : *val;

	ret = fg_read_sbs_word(chip, reg, val);
	msleep(CMD_SBS_DELAY);
	dat2 = (ret < 0) ? -1 : *val;

	if ((dat0 == dat1) && (dat0 != -1)) {
		*val = (u16)dat0;
		ret = 0;
	} else if ((dat0 == dat2) && (dat0 != -1)) {
		*val = (u16)dat0;
		ret = 0;
	} else if ((dat1 == dat2) && (dat1 != -1)) {
		*val = (u16)dat1;
		ret = 0;
	} else {
		ret = -1;
	}

	return ret;
}

static int fg_block_checksum_calculate(u8 *buffer, u8 length)
{
	u8 sum = 0;

	if (length > 32)
		return -1;

	while (length--)
		sum += buffer[length];
	sum = ~sum;
	return (int)((u8)sum);
}

static int fg_read_block(struct chip_bq27541 *chip, u32 reg, u8 startIndex, u8 length, u8 *val)
{
	int ret = -1;
	int i;
	u16 temp16;
	int sum;
	u8 checksum;
	u8 readbuf[DF_PAGE_LEN];

	if ((startIndex >= DF_PAGE_LEN) || (length == 0))
		return -1;

	if (length > DF_PAGE_LEN)
		length = DF_PAGE_LEN;
	memset(val, 0, length);
	memset(readbuf, 0, DF_PAGE_LEN);

	if (startIndex + length >= DF_PAGE_LEN)
		length = DF_PAGE_LEN - startIndex;

	if ((reg & CMDMASK_CNTL_R) == CMDMASK_CNTL_R) {
		ret = __fg_write_word(chip, CMD_CNTL, (u16)reg);
		if (ret < 0) {
			chg_err("TYPE CNTL, write 0x00 fail! command=0x%08X\n", reg);
			goto fg_read_block_end;
		}
		msleep(CMD_SBS_DELAY);

		ret = __fg_read_buffer(chip, CMD_BLOCK, DF_PAGE_HALF_LEN, readbuf);
		msleep(CMD_SBS_DELAY);
		ret |= __fg_read_buffer(chip, (u8)(CMD_BLOCK + DF_PAGE_HALF_LEN),
			(u8)(DF_PAGE_LEN - DF_PAGE_HALF_LEN), &readbuf[DF_PAGE_HALF_LEN]);
		if (ret < 0) {
			chg_err("TYPE CNTL, read 0x40 fail! command=0x%08X\n", reg);
			goto fg_read_block_end;
		}
		msleep(CMD_SBS_DELAY);

		/* check buffer */
		ret = __fg_read_word(chip, CMD_CNTL, &temp16);
		if (ret < 0)
			goto fg_read_block_end;

		checksum = (u8)(temp16 >> 8);
		sum = fg_block_checksum_calculate(readbuf, 32);
		if (sum !=(int)((u8)checksum)) {
			ret = -1;
			chg_err("TYPE CNTL, verify checksum fail! command=0x%08X\n", reg);
			goto fg_read_block_end;
		}
		else
			ret = 0;

		memmove(val, &readbuf[startIndex], length);

	} else if ((reg & CMDMASK_MANUBLOCK_R) == CMDMASK_MANUBLOCK_R) {
		ret = __fg_write_byte(chip, CMD_DFSTART, 0x01);
		if (ret < 0) {
			chg_err("TYPE MANUBLOCK, write 0x61 fail! command=0x%08X\n", reg);
			goto fg_read_block_end;
		}
		msleep(CMD_SBS_DELAY);

		ret = __fg_write_byte(chip, CMD_DFPAGE, (u8)reg);
		if (ret < 0) {
			chg_err("TYPE MANUBLOCK, write 0x3F fail! command=0x%08X\n", reg);
			goto fg_read_block_end;
		}
		msleep(15);

		ret = __fg_read_buffer(chip, CMD_BLOCK, DF_PAGE_HALF_LEN, readbuf);
		msleep(CMD_SBS_DELAY);
		ret |= __fg_read_buffer(chip, (u8)(CMD_BLOCK + DF_PAGE_HALF_LEN),
			(u8)(DF_PAGE_LEN - DF_PAGE_HALF_LEN), &readbuf[DF_PAGE_HALF_LEN]);
		if (ret < 0) {
			chg_err("TYPE MANUBLOCK, read 0x40 fail! command=0x%08X\n", reg);
			goto fg_read_block_end;
		}
		msleep(CMD_SBS_DELAY);

		/* check buffer */
		ret = __fg_read_byte(chip, CMD_CHECKSUM, &checksum);
		if (ret < 0)
			goto fg_read_block_end;

		sum = fg_block_checksum_calculate(readbuf, DF_PAGE_LEN);
		if (sum !=(int)((u8)checksum)) {
			ret = -1;
			chg_err("TYPE MANUBLOCK, verify checksum fail! command=0x%08X, cal_sum=0x%08X, read_sum=0x%02X\n",
				reg, sum, checksum);
			goto fg_read_block_end;
		}
		else
			ret = 0;
		memmove(val, &readbuf[startIndex], length);

	} else if ((reg & CMDMASK_RAMBLOCK_R) == CMDMASK_RAMBLOCK_R) { /* donot support checksum */
		ret = fg_read_sbs_word(chip, (CMDMASK_CNTL_R | 0xA3), &temp16);
		if (ret < 0) {
			chg_err("fg_read_block write 0xA3 fail! cmd=0x%08X\n", reg);
			goto fg_read_block_end;
		}
		msleep(CMD_SBS_DELAY);

		sum = (temp16 >> 8);
		checksum = (u8)((0xFF & temp16) ^ sum ^ 0xA3);

		ret = __fg_write_byte(chip, CMD_DFSTART, 0x03);
		if (ret < 0)  {
			chg_err("write 0x61 fail! cmd=0x%08X\n", reg);
			goto fg_read_block_end;
		}
		msleep(CMD_SBS_DELAY);

		ret = __fg_write_byte(chip, CMD_DFCLASS, (u8)reg);
		if (ret < 0)  {
			chg_err("write 0x3E fail! cmd=0x%08X\n", reg);
			goto fg_read_block_end;
		}
		msleep(CMD_SBS_DELAY);

		ret = __fg_write_byte(chip, CMD_DFPAGE, checksum);
		if (ret < 0)  {
			chg_err("write 0x3F fail! cmd=0x%08X\n", reg);
			goto fg_read_block_end;
		}
		msleep(CMD_SBS_DELAY);

		ret = __fg_read_buffer(chip, (u8)(CMD_BLOCK + startIndex), length, val); /* 20230823, Ethan */
		if (ret < 0)  {
			chg_err("read buffer fail! cmd=0x%08X\n", reg);
			goto fg_read_block_end;
		}

		for (i = 0; i < length; i++)
			val[i] = (u8)(val[i] ^ sum);
	} else {
		ret = __fg_read_buffer(chip, reg, length, val);
	}

fg_read_block_end:

	return ret;
}

static __maybe_unused int fg_write_dataflash(struct chip_bq27541 *chip, s32 address, s32 length, u8 *val)
{
	int ret = -1;
	int i = 0;

	s32 sum;
	u8 buf[DF_PAGE_LEN];
	u8 classID = (u8)(address >> 8);
	u8 pageNo;
	u8 sum_read;
	s32 pageLen = 0;
	s32 valIndex = 0;
	address &= 0xFF;

	if (length <= 0)
		return -1;

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);

	while (length > 0) {
		pageLen = DF_PAGE_LEN - (address % DF_PAGE_LEN);
		if (pageLen > length)
			pageLen = length;
		pr_debug("fg_write_dataflash: pageLen=%u, class=0x%02X, index=0x%02X\n", pageLen, classID, address);

		ret = __fg_write_byte(chip, CMD_DFSTART, 0x00);
		if (ret < 0) {
			chg_err("write 0x61 fail! class=0x%02X, index=0x%02X\n", classID, address);
			goto fg_write_dataflash_end;
		}
		msleep(CMD_SBS_DELAY);

		ret = __fg_write_byte(chip, CMD_DFCLASS, classID);
		if (ret < 0) {
			chg_err("write 0x3E fail! class=0x%02X, index=0x%02X\n", classID, address);
			goto fg_write_dataflash_end;
		}
		msleep(CMD_SBS_DELAY);

		pageNo = (u8)(address / DF_PAGE_LEN); /* 20231102, Sinowealth. Read Check */
		ret = __fg_write_byte(chip, CMD_DFPAGE, pageNo);
		if (ret < 0) {
			chg_err("write 0x3F fail! class=0x%02X, index=0x%02X\n", classID, address);
			goto fg_write_dataflash_end;
		}
		msleep(15);

		/* 20231102, Sinowealth. Read Check */
		ret = __fg_read_byte(chip, CMD_DFSTART, &sum_read);
		if ((ret < 0) || (sum_read != 0)) {
			ret = -1;
			chg_err("fg_read_dataflash: chekc 0x61 fail! target=0x%02X, read=0x%02X\n", 0, sum_read);
			goto fg_write_dataflash_end;
		}
		msleep(CMD_SBS_DELAY);

		ret = __fg_read_byte(chip, CMD_DFCLASS, &sum_read);
		if ((ret < 0) || (sum_read != classID)) {
			ret = -1;
			chg_err("fg_read_dataflash: chekc 0x3E fail! target=0x%02X, read=0x%02X\n", classID, sum_read);
			goto fg_write_dataflash_end;
		}
		msleep(CMD_SBS_DELAY);

		ret = __fg_read_byte(chip, CMD_DFPAGE, &sum_read);
		if ((ret < 0) || (sum_read != pageNo)) {
			ret = -1;
			chg_err("fg_read_dataflash: chekc 0x3F fail! target=0x%02X, read=0x%02X\n", pageNo, sum_read);
			goto fg_write_dataflash_end;
		}
		msleep(CMD_SBS_DELAY);

		i = address % DF_PAGE_LEN;
		if ((i != 0) || (pageLen != DF_PAGE_LEN)) { /* fill buffer */
			ret = __fg_read_buffer(chip, CMD_BLOCK, DF_PAGE_HALF_LEN, buf);
			msleep(CMD_SBS_DELAY);
			ret |= __fg_read_buffer(chip, (u8)(CMD_BLOCK + DF_PAGE_HALF_LEN),
				(u8)(DF_PAGE_LEN - DF_PAGE_HALF_LEN), &buf[DF_PAGE_HALF_LEN]);
			if (ret < 0) {
				chg_err("read 0x40 fail! class=0x%02X, index=0x%02X\r\n", classID, address);
				goto fg_write_dataflash_end;
			}
		}
		memmove(&buf[i], &val[valIndex], pageLen); /* fill buffer */

		/* for QualComm Host, cannot write over 16byte buffer!! */
		ret = __fg_write_buffer(chip, CMD_BLOCK, DF_PAGE_HALF_LEN, buf);
		for (i = DF_PAGE_HALF_LEN; i < DF_PAGE_LEN; i++) {
			msleep(CMD_SBS_DELAY);
			ret |= __fg_write_byte(chip, (u8)(CMD_BLOCK + i), buf[i]);
		}
		if (ret < 0) {
			chg_err("write 0x40 fail! class=0x%02X, index=0x%02X\n", classID, address);
			goto fg_write_dataflash_end;
		}

		sum = fg_block_checksum_calculate(buf, DF_PAGE_LEN);

		ret = __fg_write_byte(chip, CMD_CHECKSUM, (u8)sum);
		if (ret < 0) {
			chg_err("write 0x60 fail! class=0x%02X, index=0x%02X\n", classID, address);
			goto fg_write_dataflash_end;
		}
		msleep(100);

		valIndex += pageLen;
		address += pageLen;
		length -= pageLen;
		ret = 0;
	}

fg_write_dataflash_end:
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	return ret;
}

static s32 sh366002_output_content(char *str, s32 strlen, u8 *buf, s32 buflen)
{
#define PRINT_BUFFER_FORMAT_LEN 3
	s32 i, j;
	int index = 0;

	if ((strlen <= 0) || (buflen <= 0))
		return -1;

	memset(str, 0, strlen * sizeof(char));

	j = min(buflen, strlen / PRINT_BUFFER_FORMAT_LEN);
	if (j * PRINT_BUFFER_FORMAT_LEN >= strlen) {
		chg_err("buflen is more than max\n");
		return -1;
	}

	for (i = 0; i < j; i++)
		index += scnprintf(str + index, strlen - index, "%02X ", buf[i]);

	return index;
}

static int fg_read_ram_block(struct chip_bq27541 *chip, u32 reg, u8 startIndex, u8 length, u8* val)
{
	int ret;
	int ret0;
	int ret1;
	int ret2;
	u8 str[200];

	u8 dat0[DF_PAGE_LEN], dat1[DF_PAGE_LEN], dat2[DF_PAGE_LEN];
	if ((length <= 0) || (startIndex + length > DF_PAGE_LEN)) /* 20231102, Sinowealth. Bug Fix */
		return -1;

	ret0 = fg_read_block(chip, reg, startIndex, length, dat0);
	msleep(CMD_SBS_DELAY);
	ret1 = fg_read_block(chip, reg, startIndex, length, dat1);
	msleep(CMD_SBS_DELAY);
	ret2 = fg_read_block(chip, reg, startIndex, length, dat2);
	msleep(CMD_SBS_DELAY);

	/* for debug */
	sh366002_output_content(str, 200, dat0, length);
	pr_debug("fg_read_ram_block ret=%d, dat0=%s\r\n", ret0, str);
	sh366002_output_content(str, 200, dat1, length);
	pr_debug("fg_read_ram_block ret=%d, dat1=%s\r\n", ret1, str);
	sh366002_output_content(str, 200, dat2, length);
	pr_debug("fg_read_ram_block ret=%d, dat2=%s\r\n", ret2, str);

	if (!memcmp(dat0, dat1, length) && (ret0 >= 0)) {
		memmove(val, dat0, length);
		ret = 0;
	} else if (!memcmp(dat0, dat2, length) && (ret0 >= 0)) {
		memmove(val, dat0, length);
		ret = 0;
	} else if (!memcmp(dat1, dat2, length) && (ret1 >= 0)) {
		memmove(val, dat1, length);
		ret = 0;
	} else {
		ret = -1;
	}

	return ret;
}

static s32 fg_gauge_unseal(struct chip_bq27541 *chip)
{
	s32 ret;
	s32 i;
	u16 cntl_status;

	for (i = 0; i < SH366002_UNSEAL_TRY_COUNTS; i++) {
		ret = fg_write_sbs_word(chip, CMD_CNTL, (u16)CMD_UNSEALKEY);
		if (ret < 0)
			goto fg_gauge_unseal_End;
		msleep(CMD_SBS_DELAY);

		ret = fg_write_sbs_word(chip, CMD_CNTL, (u16)(CMD_UNSEALKEY >> 16));
		if (ret < 0)
			goto fg_gauge_unseal_End;
		msleep(CMD_SBS_DELAY);

		ret = fg_write_sbs_word(chip, CMD_CNTL, (u16)CMD_FULLKEY);
		if (ret < 0)
			goto fg_gauge_unseal_End;
		msleep(CMD_SBS_DELAY);

		ret = fg_write_sbs_word(chip, CMD_CNTL, (u16)(CMD_FULLKEY >> 16));
		if (ret < 0)
			goto fg_gauge_unseal_End;
		msleep(CMD_SBS_DELAY);

		ret = fg_read_sbs_word(chip, CMD_CNTLSTATUS, &cntl_status);
		if (ret < 0)
			goto fg_gauge_unseal_End;
		msleep(CMD_SBS_DELAY);

		if ((cntl_status & CMD_CNTLSTATUS_SEAL) == 0)
			break;
		msleep(CMD_SBS_DELAY);
	}

	ret = (i < SH366002_UNSEAL_TRY_COUNTS) ? 0 : -1;
fg_gauge_unseal_End:
	return ret;
}

static s32 fg_gauge_seal(struct chip_bq27541 *chip)
{
	return fg_write_sbs_word(chip, CMD_CNTL, CMD_SEAL);
}


static s32 fg_gauge_get_default_cell_model(struct chip_bq27541 *chip, char *profile_name, u16 *pBuf)
{
	struct device* dev;
	struct device_node* np;
	u16* kzBuf;
	u8 str[STRLEN];
	s32 buflen, ret;
	s32 ratio;
	s32 i, j;
	u16 temp16;
	u16 cycle0;

	if (chip == NULL || chip->client == NULL || profile_name == NULL || pBuf == NULL || chip->ic_dev == NULL) {
		chg_err("fail: Cannot find profile_name\r\n");
		return -1;
	}

	dev = &chip->client->dev;
	if (dev == NULL)
		return -1;

	np = dev->of_node;
	if (np == NULL)
		return -1;

	/* battery_params node*/
	np = of_find_node_by_name(np, "battery_params");
	if ((np == NULL) || (pBuf == NULL)) {
		chg_err("Cannot find child node \"battery_params\"\r\n");
		return -1;
	}

	buflen = of_property_count_elems_of_size(np, profile_name, sizeof(u16));
	if (buflen < CELL_MODEL_COUNT) {
		chg_err("read len too small! ele_len=%d, key=%s\n", buflen, profile_name);
		return -1;
	}

	kzBuf = (u16*)devm_kzalloc(dev, buflen * sizeof(u16), 0);
	if (kzBuf == NULL) {
		chg_err("kzalloc error!\r\n");
		return -1;
	}

	ret = of_property_read_u16_array(np, profile_name, kzBuf, buflen);
	if (ret < 0) {
		chg_err("read model dtsi fail! ret=%d\r\n", ret);
		devm_kfree(dev, kzBuf);
		return ret;
	}

	memset(str, 0, STRLEN * sizeof(u8));
	j = 0;
	for (i = 0; i < CELL_MODEL_COUNT; i++) {
		pBuf[i] = kzBuf[i];
		j += scnprintf(str + j, STRLEN - j, "%u, ", pBuf[i]);
	}
	chg_info("ic_index:%d read model dtsi is %s\r\n", chip->ic_dev->index, str);

	if (pBuf[CELL_MODEL_COUNT - 2])
		temp16 = pBuf[CELL_MODEL_COUNT - 1] * MODELRATIO_BASE / pBuf[CELL_MODEL_COUNT - 2]; /* sino change 20240112 */
	else
		temp16 = 0;
	if (temp16 > EXTREME_MAXRATIO_CHECK)
		pBuf[CELL_MODEL_COUNT] = OVER_MAXRATIO_MAGICDATA;
	else
		pBuf[CELL_MODEL_COUNT] = 0;

	ret = fg_read_sbs_word_then_check(chip, CMD_CYCLECOUNT, &cycle0);
	if (ret < 0) {
		chg_err("cannot read cycle_count! ret=%d\r\n", ret);
		devm_kfree(dev, kzBuf);
		return ret;
	}

	if (cycle0 >= 800)
		ratio = 3000;
	else if (cycle0 >= 600)
		ratio = 2500;
	else if (cycle0 >= 400)
		ratio = 2000;
	else if (cycle0 >= 200)
		ratio = 1500;
	else
		ratio = 1000;

	memset(str, 0, STRLEN);
	j = 0;
	for (i = 0; i < CELL_MODEL_COUNT; i++) {
		temp16 = pBuf[i] * ratio / MODELRATIO_BASE;

		if (temp16 > MAX_MODEL)
			temp16 = MAX_MODEL;
		if (temp16 < MIN_MODEL)
			temp16 = MIN_MODEL;

		pBuf[i] = temp16;
		j += scnprintf(str + j, STRLEN - j, "%u, ", pBuf[i]);
	}
	j += scnprintf(str + j, STRLEN - j, "%u, ", pBuf[i]);
	devm_kfree(dev, kzBuf);
	chg_info("ic_index:%d CycleCount=%u, CycleRatio=%u, Model=%s\r\n",
		chip->ic_dev->index, cycle0, ratio, str);

	return cycle0;
}

/* -1: err; 0: Model OK; else: Error Flag */
static s32 sh36002_gauge_check_cell_model(struct chip_bq27541 *chip, char *profile_name) /* 20220625, Ethan */
{
	s32 ret;
	u8 buf[32];
	u16 model[15];
	s32 i, j, k, modelRatio;
	s32 maxValue, minValue;
	u16 temp16;
	u8 str[200];
	u16 pBuf[16] = {0};
	u8 track_buf[64] = {0};

	if (chip == NULL || chip->ic_dev == NULL || chip->client == NULL || profile_name == NULL)
		return -EFAULT;

	memset(str, 0, sizeof(u8) * 200);
	ret = fg_gauge_get_default_cell_model(chip, profile_name, pBuf);
	if (ret < 0) {
		chg_err("fg_gauge_check_cell_model fail! cannot read default cell-model! ret=%d\r\n", ret);
		return ret;
	}
	temp16 = (u16)ret;

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = fg_read_ram_block(chip, CMD_CELLMODEL, 0, BYTE_COUNT_CELL_MODEL, buf);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	if (ret < 0) {
		chg_err("fail! cannot read cell-model! ret=%d\r\n", ret);
		return ret;
	}

	j = 0;
	for (i = 0; i < 15; i++) {
		model[i] = (u8)(buf[2 * i] ^ 0x5A) + 0x100 * (u8)(buf[2 * i + 1] ^ 0x43);
		if (model[i] == 0)
			return -1;
		j += scnprintf(str + j, 200 - j, "%u,", model[i]);
	}
	chg_info("ic_index:%d model is %s\r\n", chip->ic_dev->index, str);

	ret = MODELFLAG_NONE;

	if (temp16 <= 800) {
		maxValue = 0;
		minValue = 0;

		for (i = TOOLARGE_STARTGRID; i <= TOOLARGE_ENDGRID; i++) {
			maxValue += model[i];
			minValue += pBuf[i];
		}

		/* model < 0x1_0000 =>  max < 0x10_0000. max * 1000 < 0x3e80_0000, won't overflow */
		if (minValue)
			modelRatio = maxValue * TOOLARGE_RATIO / minValue;
		else
			modelRatio = 0;

		if (modelRatio >= TOOLARGE_MAXVAULE) {
			ret |= MODELFLAG_NEW_LARGE;
			maxValue /= (TOOLARGE_ENDGRID - TOOLARGE_STARTGRID + 1);
			minValue /= (TOOLARGE_ENDGRID - TOOLARGE_STARTGRID + 1);
			chg_err("ic_index:%d model too large! avg=%u, default_avg=%u, ratio=%d\r\n",
				chip->ic_dev->index, maxValue, minValue, modelRatio);
		}
	}

	for (i = TOOSMALL_STARTGRID_0; i <= TOOSMALL_ENDGRID_0; i++) {
		if (model[i] < TOOSMALL_MINVAULE_0) {
			ret |= MODELFLAG_NEW_SMALL;
			chg_err("ic_index:%d model too small! index=%d, model=%u\r\n",
				chip->ic_dev->index, i, model[i]);
			break;
		}
	}
	for (i = TOOSMALL_STARTGRID_1; i <= TOOSMALL_ENDGRID_1; i++) {
		if (model[i] < TOOSMALL_MINVAULE_1) {
			ret |= MODELFLAG_NEW_SMALL;
			chg_err("ic_index:%d model too small! index=%d, model=%u\r\n",
				chip->ic_dev->index, i, model[i]);
			break;
		}
	}
	for (i = TOOSMALL_STARTGRID_2; i <= TOOSMALL_ENDGRID_2; i++) {
		if (model[i] < TOOSMALL_MINVAULE_2) {
			ret |= MODELFLAG_NEW_SMALL;
			chg_err("ic_index:%d model too small! index=%d, model=%u\r\n",
				chip->ic_dev->index, i, model[i]);
			break;
		}
	}

	k = 0;
	for (i = SLIGHT_STARTGRID; i <= SLIGHT_ENDGRID - SLIGHT_GAP + 1; i++) {
		maxValue = 0;
		minValue = 0x7FFFFFFF;
		for (j = i; j < i + SLIGHT_GAP; j++) {
			modelRatio = (u16)model[j];
			maxValue = max(maxValue, modelRatio);
			minValue = min(minValue, modelRatio);
		}

		if (minValue)
			modelRatio = maxValue * SLIGHT_RATIO / minValue;
		else
			modelRatio = 0;
		k += !!(modelRatio >= SLIGHT_MAXRATIO);
		chg_err("ic_index:%d model slight singular cnt=%u, index=%u, max=%u, min=%u, ratio=%u\r\n",
			chip->ic_dev->index, k, i, maxValue, minValue, modelRatio);
	}
	if (k >= SLIGHT_MAXCNT) {
		ret |= MODELFLAG_NEW_SLIGHT;
		chg_err("ic_index:%d model slight singular! cnt=%u\r\n", chip->ic_dev->index, k);
	}

	k = 0;
	for (i = EXTREME_STARTGRID_0; i <= EXTREME_ENDGRID_0 - EXTREME_GAP_0 + 1; i++) {
		maxValue = 0;
		minValue = 0x7FFFFFFF;
		for (j = i; j < i + EXTREME_GAP_0; j++) {
			modelRatio = (u16)model[j];
			maxValue = max(maxValue, modelRatio);
			minValue = min(minValue, modelRatio);
		}

		if (minValue)
			modelRatio = maxValue * EXTREME_RATIO / minValue;
		else
			modelRatio = 0;
		k += !!(modelRatio >= EXTREME_MAXRATIO_0);
		chg_err("ic_index:%d model extreme singular cnt=%u, index=%u, max=%u, min=%u, ratio=%u\r\n",
			chip->ic_dev->index, k, i, maxValue, minValue, modelRatio);
	}
	if (k >= EXTREME_MAXCNT_0) {
		ret |= MODELFLAG_NEW_EXTREME;
		chg_err("ic_index:%d model extreme singular! cnt=%u\r\n", chip->ic_dev->index, k);
	}

	temp16 = pBuf[CELL_MODEL_COUNT]; /* sino change 20240112 */
	k = 0;
	for (i = EXTREME_STARTGRID_1; i <= EXTREME_ENDGRID_1 - EXTREME_GAP_1 + 1; i++) {
		maxValue = 0;
		minValue = 0x7FFFFFFF;
		for (j = i; j < i + EXTREME_GAP_1; j++) {
			modelRatio = (u16)model[j];
			maxValue = max(maxValue, modelRatio);
			minValue = min(minValue, modelRatio);
		}

		if (minValue)
			modelRatio = maxValue * EXTREME_RATIO / minValue;
		else
			modelRatio = 0;

		if (temp16 == OVER_MAXRATIO_MAGICDATA) { /* sino change 20240112 */
			chg_err("ic_index:%d fg_gauge_check_cell_model: Code in run OVER_MAXRATIO_MAGICDATA!",
				chip->ic_dev->index);
			if (i == EXTREME_STARTGRID_1 + 1) {
				if (model[EXTREME_STARTGRID_1 + 2] > model[EXTREME_STARTGRID_1 + 1])
					k += !!(modelRatio >= EXTREME_MAXRATIO_1_PRO);
				else
					k += !!(modelRatio >= EXTREME_MAXRATIO_1);
			}
			else {
				k += !!(modelRatio >= EXTREME_MAXRATIO_1);
			}
		}
		else {
			k += !!(modelRatio >= EXTREME_MAXRATIO_1);
		}
		chg_err("ic_index:%d model extreme singular cnt=%u, index=%u, max=%u, min=%u, ratio=%u\r\n",
			chip->ic_dev->index, k, i, maxValue, minValue, modelRatio);
	}
	if (k >= EXTREME_MAXCNT_1) {
		ret |= MODELFLAG_NEW_EXTREME;
		chg_err("ic_index:%d model extreme singular! cnt=%u\r\n",
			chip->ic_dev->index, k);
	}

	if (model[EXTREME_ENDGRID_2])
		modelRatio = model[EXTREME_STARTGRID_2] * EXTREME_RATIO / model[EXTREME_ENDGRID_2];
	else
		modelRatio = 0;
	if (modelRatio >= EXTREME_MAXRATIO_2) {
		ret |= MODELFLAG_NEW_EXTREME;
		chg_err("ic_index:%d model extreme singular! index=%u, ratio=%u, limit=%u\r\n",
			chip->ic_dev->index, EXTREME_STARTGRID_2, modelRatio, EXTREME_MAXRATIO_2);
	}

	if (ret > 0) {
		sh366002_track_model_data_init(chip);
		scnprintf(&(track_buf[0]),  sizeof(track_buf) - 1, "$$check_ret@@0x%04x$$model_b@@", ret);
		sh366002_track_model_data_update(chip, track_buf);
		sh366002_track_model_data_update(chip, str);
		chg_err("ic_index:%d end! ret=0x%04x\r\n", chip->ic_dev->index, ret);
	}

	return ret;
}

static s32 sh366002_gauge_restore_cell_model(struct chip_bq27541 *chip, char *profile_name)
{
	u16 pBuf[16];
	u8 buf_read[BYTE_COUNT_CELL_MODEL];
	u8 buf_write[LENGTH_CELLMODEL];
	u8 str[STRLEN] = {0};
	s32 ret;
	s32 i, j;
	u8 byteH, byteL;
	u16 temp16;
	u16 cycle0 = 0;
	u8 track_buf[64] = {0};

	if (chip == NULL || chip->ic_dev == NULL || chip->client == NULL || profile_name == NULL)
		return -EFAULT;

	ret = fg_gauge_get_default_cell_model(chip, profile_name, pBuf);
	if (ret < 0) {
		chg_err("fg_gauge_restore_cell_model err! cannot get default model! ret=%d\r\n", ret);
		goto fg_gauge_restore_cell_model_end;
	}
	cycle0 = ret;

	ret = fg_gauge_unseal(chip);
	if (ret < 0) {
		chg_err("err! cannot unseal %d ic! ret=%d\r\n", chip->ic_dev->index, ret);
		goto fg_gauge_restore_cell_model_end;
	}

	memset(buf_write, 0, LENGTH_CELLMODEL * sizeof(u8));
	buf_write[INDEX_CELLMODEL] = 0x18;
	buf_write[INDEX_CELLMODEL + 1] = 0x4C;
	buf_write[INDEX_XCELLMODEL] = 0xE7;
	buf_write[INDEX_XCELLMODEL + 1] = 0xE6;

	j = 2;
	for (i = 0; i < CELL_MODEL_COUNT; i++) {
		byteH = (u8)(0x18 ^ j ^ (pBuf[i] >> 8));
		byteL = (u8)(0x18 ^ (j + 1) ^ pBuf[i]);

		buf_write[INDEX_CELLMODEL + j] = byteH;
		buf_write[INDEX_CELLMODEL + j + 1] = byteL;
		buf_write[INDEX_XCELLMODEL + j] = byteH;
		buf_write[INDEX_XCELLMODEL + j + 1] = byteL;

		j += 2;
	}

	ret = fg_write_dataflash(chip, ADDR_CELLMODEL, LENGTH_CELLMODEL, buf_write);
	if (ret < 0) {
		chg_err("fail! ic_index:%d, cannot write cell-model to E2rom, ret=%d\r\n", chip->ic_dev->index, ret);
		goto fg_gauge_restore_cell_model_end;
	}

	fg_write_sbs_word(chip, CMD_CNTL, CMD_FORCE_UPDATE_E2ROM);
	msleep(CMD_E2ROM_DELAY);

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	ret = fg_read_ram_block(chip, CMD_CELLMODEL, 0, BYTE_COUNT_CELL_MODEL, buf_read);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	if (ret < 0) {
		chg_err("fail! cannot read cell-model, ret=%d\r\n", ret);
		goto fg_gauge_restore_cell_model_end;
	}

	ret = 0;
	j = 0;
	for (i = 0; i < CELL_MODEL_COUNT; i++) {
		temp16 = 0x100 * (u8)(buf_read[2 * i + 1] ^ 0x43) + (u8)(buf_read[2 * i] ^ 0x5A);
		if (pBuf[i] != temp16)
			ret = -1;

		j += scnprintf(str + j, STRLEN - j, "%u,", temp16);
	}

	chg_info("ic_index:%d, read model check is %d, model=%s\r\n", chip->ic_dev->index, ret, str);

fg_gauge_restore_cell_model_end:
	chg_info("ic_index:%d end: ret=%d\r\n", chip->ic_dev->index, ret);
	scnprintf(&(track_buf[0]),  sizeof(track_buf) - 1, "$$restore_ret@@%d$$cycle@@%d$$model_a@@", ret, cycle0);
	sh366002_track_model_data_update(chip, track_buf);
	sh366002_track_model_data_update(chip, str);
	fg_gauge_seal(chip);
	return ret;
}

#define ERR_MSG_BUF	PAGE_SIZE
__printf(4, 5)
static int sh366002_publish_ic_err_msg(
	char *manu_name, int type, int sub_type, const char *format, ...)
{
	va_list args;
	char *buf;
	int rc;
	struct mms_msg *topic_msg;
	struct oplus_mms *err_topic = oplus_mms_get_by_name("error");

	if (!err_topic || !manu_name)
		return -ENODEV;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	va_start(args, format);
	vsnprintf(buf, ERR_MSG_BUF, format, args);
	va_end(args);

	topic_msg =
		oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
					"[%s]-[%d]-[%d]:%s", manu_name, type, sub_type, buf);
	kfree(buf);
	if (topic_msg == NULL) {
		chg_err("alloc topic msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(err_topic, topic_msg);
	if (rc < 0) {
		chg_err("publish error topic msg error, rc=%d\n", rc);
		kfree(topic_msg);
	}

	return rc;
}

static int sh366002_track_model_data_init(struct chip_bq27541 *chip)
{
	if (chip == NULL || chip->dev == NULL || chip->ic_dev == NULL)
		return -EFAULT;

	mutex_lock(&chip->imp_model_lock);
	if (chip->imp_model_data) {
		mutex_unlock(&chip->imp_model_lock);
		chg_err("ic_index:%d, model data buf is not null\n", chip->ic_dev->index);
		return -EFAULT;
	}

	chip->imp_model_data = (u8 *)devm_kzalloc(chip->dev, SH366002_IMP_MODEL_LEN, GFP_KERNEL);
	if (chip->imp_model_data == NULL) {
		mutex_unlock(&chip->imp_model_lock);
		chg_err("ic_index:%d, model data buf devm_kzalloc error\n", chip->ic_dev->index);
		return -ENOMEM;
	}
	mutex_unlock(&chip->imp_model_lock);

	return 0;
}

static int sh366002_track_model_data_update(struct chip_bq27541 *chip, char *buf)
{
	int str_len;

	if (chip == NULL || buf == NULL)
		return -EFAULT;

	mutex_lock(&chip->imp_model_lock);
	if (chip->imp_model_data == NULL) {
		mutex_unlock(&chip->imp_model_lock);
		chg_err("ic_index:%d, model data buf is null\n", chip->ic_dev->index);
		return -EFAULT;
	}

	str_len = strlen(chip->imp_model_data);
	if (str_len >= SH366002_IMP_MODEL_LEN - 1) {
		mutex_unlock(&chip->imp_model_lock);
		chg_err("ic_index:%d, model data buf over\n", chip->ic_dev->index);
		return -EFAULT;
	}

	scnprintf(&(chip->imp_model_data[str_len]), SH366002_IMP_MODEL_LEN - str_len - 1, "%s", buf);
	mutex_unlock(&chip->imp_model_lock);

	return 0;
}

static void sh366002_track_model_abnormal_upload(struct chip_bq27541 *chip)
{
	if (chip == NULL || chip->ic_dev == NULL || chip->dev == NULL)
		return;

	mutex_lock(&chip->imp_model_lock);
	if (chip->imp_model_data == NULL) {
		mutex_unlock(&chip->imp_model_lock);
		chg_err("ic_index:%d, model data buf is null\n", chip->ic_dev->index);
		return;
	}

	sh366002_publish_ic_err_msg(
		chip->ic_dev->manu_name, OPLUS_IC_ERR_GAUGE, TRACK_GAUGE_ERR_IMP_MODEL, "%s", chip->imp_model_data);

	devm_kfree(chip->dev, chip->imp_model_data);
	chip->imp_model_data = NULL;
	mutex_unlock(&chip->imp_model_lock);
}

void oplus_sh36002_check_imp_model(struct chip_bq27541 *chip)
{
	int ret;

	if (chip == NULL || chip->ic_dev == NULL || chip->dev == NULL)
		return;

	mutex_lock(&chip->imp_model_lock);
	if (chip->imp_model_checking || chip->imp_model_data) {
		chg_info("ic_index:%d, imp model check is running or data is uploading\n", chip->ic_dev->index);
		mutex_unlock(&chip->imp_model_lock);
		return;
	}
	chip->imp_model_checking = true;
	mutex_unlock(&chip->imp_model_lock);

	ret = sh36002_gauge_check_cell_model(chip, SH366002_MODEL_NAME);
	if (ret > 0) {
		sh366002_gauge_restore_cell_model(chip, SH366002_MODEL_NAME);
		sh366002_track_model_abnormal_upload(chip);
	}

	mutex_lock(&chip->imp_model_lock);
	chip->imp_model_checking = false;
	mutex_unlock(&chip->imp_model_lock);
}

