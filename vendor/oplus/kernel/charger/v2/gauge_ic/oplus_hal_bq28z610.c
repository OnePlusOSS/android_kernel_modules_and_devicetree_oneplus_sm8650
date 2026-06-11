/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_bq28z610.c
** Description: bq28z610 ic
** Date: 2025-11-20
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/


#define pr_fmt(fmt) "[BQ28Z610]([%s][%d]): " fmt, __func__, __LINE__

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
#include <linux/sched/clock.h>
#include <oplus_chg_monitor.h>
#include "oplus_hal_bq28z610.h"


static int __read_unsigned_data_from_node(struct device_node *node,
	const char *prop_str, u32 *addr, int len_max)
{
	int rc = 0;
	int length = 0;

	if (!node || !prop_str || !addr) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u32));
	if (rc < 0) {
		chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;

	if (length != len_max) {
		chg_err("entries(%d) num error, only %d allowed\n", length, len_max);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(node, prop_str, (u32 *)addr, length);
	if (rc < 0) {
		chg_err("Read %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	return length;
}

#define FCC_RA_DELTA_VMAX       32
#define FCC_RA_DELTA_CC         10
#define FCC_RA_T_DELTA_CC0      30
#define FCC_RA_T_DELTA_CC1      20
void bq28z610_parse_fcc_ra_dt(struct chip_bq27541 *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;

	chip->fcc_ra0_support = of_property_read_bool(node, "oplus_spec,fcc-ra-0-support");
	chip->fcc_vdelta_support = of_property_read_bool(node, "oplus_spec,fcc-ra-vdelta-support");
	if (chip->fcc_vdelta_support) {
		rc = of_property_read_u32(node, "oplus_spec,fcc-ra-delta-vmax", &chip->ra_vd_curve.delta_vmax);
		if (rc < 0) {
			chg_err("get oplus_spec,fcc-ra-delta-vmax property error, rc=%d\n", rc);
			chip->ra_vd_curve.delta_vmax = FCC_RA_DELTA_VMAX;
		}
		rc = of_property_read_u32(node, "oplus_spec,fcc-ra-delta-cc", &chip->ra_vd_curve.delta_cc);
		if (rc < 0) {
			chg_err("get oplus_spec,fcc-ra-delta-cc property error, rc=%d\n", rc);
			chip->ra_vd_curve.delta_cc = FCC_RA_DELTA_CC;
		}
	}
	chip->fcc_ra_t_support = of_property_read_bool(node, "oplus_spec,fcc-ra-t-support");
	if (chip->fcc_ra_t_support) {
		rc = of_property_read_u32(node, "oplus_spec,fcc-ra-t-delta-cc0", &chip->ra_t_curve.delta_cc0);
		if (rc < 0) {
			chg_err("get oplus_spec,fcc-ra-t-delta-cc0 property error, rc=%d\n", rc);
			chip->ra_t_curve.delta_cc0 = FCC_RA_T_DELTA_CC0;
		}
		rc = of_property_read_u32(node, "oplus_spec,fcc-ra-t-delta-cc1", &chip->ra_t_curve.delta_cc1);
		if (rc < 0) {
			chg_err("get oplus_spec,fcc-ra-t-delta-cc1 property error, rc=%d\n", rc);
			chip->ra_t_curve.delta_cc1 = FCC_RA_T_DELTA_CC1;
		}
		rc = __read_unsigned_data_from_node(node, "oplus_spec,fcc-ra-default",
			(u32 *)chip->ra_res_curve.ra_default, FCC_RA_DEFAULT_NUM);
		if (rc < 0) {
			chg_err("get oplus_spec,fcc-ra-default property error, rc=%d\n", rc);
			goto RA_T_SUPPORT_ERR;
		}
		rc = __read_unsigned_data_from_node(node, "oplus_spec,fcc-ra-cc-thr",
			(u32 *)chip->ra_t_curve.cc_thr, FCC_RA_CC_THR_NUM);
		if (rc < 0) {
			chg_err("get oplus_spec,fcc-ra-cc-thr property error, rc=%d\n", rc);
			goto RA_T_SUPPORT_ERR;
		}
		rc = __read_unsigned_data_from_node(node, "oplus_spec,fcc-ra-fcc0-thr",
			(u32 *)chip->ra_t_curve.fcc0_thr, FCC_RA_CC_THR_NUM);
		if (rc < 0) {
			chg_err("get oplus_spec,fcc-ra-fcc0-thr property error, rc=%d\n", rc);
			goto RA_T_SUPPORT_ERR;
		}
		rc = __read_unsigned_data_from_node(node, "oplus_spec,fcc-ra-extreme-thr",
			(u32 *)chip->ra_t_curve.extreme_thr, FCC_RA_CC_THR_NUM);
		if (rc < 0) {
			chg_err("get oplus_spec,fcc-ra-extreme-thr property error, rc=%d\n", rc);
			goto RA_T_SUPPORT_ERR;
		}
		rc = __read_unsigned_data_from_node(node, "oplus_spec,fcc-ra-k-curve",
			(u32 *)chip->ra_t_curve.k_curve, FCC_RA_CC_THR_NUM * FCC_RA_CC_THR_NUM);
		if (rc < 0) {
			chg_err("get oplus_spec,fcc-ra-k-ratio property error, rc=%d\n", rc);
			goto RA_T_SUPPORT_ERR;
		}
	}
	chg_info("fcc_ra0_support=%d, fcc_vdelta_support=%d, fcc_ra_t_support=%d\n",
		chip->fcc_ra0_support, chip->fcc_vdelta_support, chip->fcc_ra_t_support);
	return;

RA_T_SUPPORT_ERR:
	chip->fcc_ra_t_support = false;
	chg_info("fcc_ra0_support=%d, fcc_vdelta_support=%d, fcc_ra_t_support=%d\n",
		chip->fcc_ra0_support, chip->fcc_vdelta_support, chip->fcc_ra_t_support);
}

static u8 bq28z610_calc_checksum(u8 *buf, int len)
{
	u8 checksum = 0;

	while (len--)
		checksum += buf[len];

	checksum = 0xff - checksum;
	return checksum;
}

#define BQ28Z610_BLOCK_SIZE 32
static int bq28z610_block_check_conditions(struct chip_bq27541 *chip, u8 *buf, int len, int offset, bool do_checksum)
{
	if (!chip || !buf || offset < 0 || offset >= BQ28Z610_BLOCK_SIZE || len <= 0 ||
	    (len + 2 + do_checksum > BQ28Z610_BLOCK_SIZE) || (offset + len + 2 + do_checksum > BQ28Z610_BLOCK_SIZE)) {
		chg_err("%soffset %d or len %d invalid\n", buf ? "buf is null or " : "", offset, len);
		return -EINVAL;
	}

	if (is_return_pre_value(chip) || atomic_read(&chip->locked))
		return -EINVAL;

	return 0;
}

#define BQ28Z610_SUBCMD_TRY_COUNT	3
static int bq28z610_write_block(struct chip_bq27541 *chip, int addr, u8 *buf, int len, int offset, bool do_checksum)
{
	int ret;
	int data_check;
	int try_count = BQ28Z610_SUBCMD_TRY_COUNT;
	u8 extend_read_data[BQ28Z610_BLOCK_SIZE + 2] = { 0 };
	u8 extend_write_data[BQ28Z610_BLOCK_SIZE + 2] = { 0 };
	u8 check_data[2] = { 0, len + 2 + 2 + do_checksum };

	ret = bq28z610_block_check_conditions(chip, buf, len, offset, do_checksum);
	if (ret < 0)
		return ret;

try:
	ret = bq27541_i2c_txsubcmd(chip, BQ28Z610_REG_CNTL1, addr);
	if (ret < 0)
		goto error;

	usleep_range(1000, 1000);
	ret = bq27541_read_i2c_block(chip, BQ28Z610_REG_CNTL1, (len + 2), extend_read_data);
	if (ret < 0)
		goto error;

	data_check = (extend_read_data[1] << 0x8) | extend_read_data[0];
	if (try_count-- > 0 && data_check != addr) {
		chg_err("0x%04x not match. try_count=%d offset=%d extend_data[0]=0x%2x, extend_data[1]=0x%2x\n",
			addr, try_count, offset, extend_read_data[0], extend_read_data[1]);
		usleep_range(2000, 2000);
		goto try;
	}
	if (try_count < 0)
		goto error;

	memmove(extend_write_data, extend_read_data, len + 2);
	memmove(&extend_write_data[offset + 2], buf, len);
	if (do_checksum)
		extend_write_data[offset + len + 2] = bq28z610_calc_checksum(buf, len);

	ret = bq27541_i2c_txsubcmd(chip, BQ28Z610_REG_CNTL1, addr);
	if (ret < 0)
		goto error;
	ret = bq27541_write_i2c_block(chip, BQ28Z610_REG_CNTL1, len + 2 + do_checksum, extend_write_data);
	if (ret < 0)
		goto error;
	check_data[0] = bq28z610_calc_checksum(extend_write_data, len + 2 + do_checksum);

	ret = bq27541_write_i2c_block(chip, BQ28Z610_TERM_VOLT_CHECK_ADDR, 2, check_data);
	if (ret < 0)
		goto error;

	try_count = BQ28Z610_SUBCMD_TRY_COUNT;
	do {
		data_check = true;
		memset(extend_read_data, 0, len + 2 + do_checksum);
		usleep_range(15000, 15000);
		ret = bq27541_i2c_txsubcmd(chip, BQ28Z610_REG_CNTL1, addr);
		if (ret < 0)
			goto error;
		usleep_range(1000, 1000);
		ret = bq27541_read_i2c_block(chip, BQ28Z610_REG_CNTL1, len + 2 + do_checksum, extend_read_data);
		if (memcmp(extend_read_data, extend_write_data, len + 2)) {
			chg_err("reg not match.extend_read_data =[%*ph]\n", len + 2 + do_checksum, extend_read_data);
			chg_err("reg not match.extend_write_data=[%*ph]\n", len + 2 + do_checksum, extend_write_data);
			data_check = false;
		}
	} while (!data_check && try_count--);
	if (!data_check)
		goto error;

	chg_info(" addr=0x%04x buf=[%*ph] write success\n", addr, len, buf);
	return 0;

error:
	chg_err("addr=0x%04x buf=[%*ph] write fail\n", addr, len, buf);
	return -EINVAL;
}

static void bq28z610_fcc_init_cc0(struct chip_bq27541 *chip)
{
	chip->ra_config.ra_vd_cc0 = bq27541_get_battery_cc(chip);
}

void bq28z610_fcc_init_cc1(struct chip_bq27541 *chip)
{
	int cc;

	cc = bq27541_get_battery_cc(chip);
	chip->ra_config.ra_cc1 = chip->ra_config.dbg_cc > 0 ? chip->ra_config.dbg_cc : cc;
}

#define CC_RA_MAX				5000
#define CC_RA_MIN				30
#define REASON_LENGTH_MAX			1024
#define BQ28Z610_FCC_RA0_SIZE			4
#define BQ28Z610_RA_FLAG_SIZE			2
#define BQ28Z610_RA_COLUMN_SIZE			4
#define BQ28Z610_RA1_CHECK_MAX			250
#define BQ28Z610_REG_RA_FLAG0			0x406B
#define BQ28Z610_REG_RA_FLAG1			0x004E
#define BQ28Z610_RA_FLAG_VALUE			0xFFAA
#define BQ28Z610_REG_RA0_MAIN0			0x4102
#define BQ28Z610_REG_RA0_SUB0			0x4142
#define BQ28Z610_REG_RA0_MAIN1			0x4182
#define BQ28Z610_REG_RA0_SUB1			0x41C2
static bool bq28z610_fcc_ra0_get_status(struct chip_bq27541 *chip)
{
	u8 read_flag_data[BQ28Z610_RA_FLAG_SIZE + 2] = { 0 };

	if (!chip->fcc_ra0_support)
		return false;

	if (chip->ra_config.ra_cc1 >= CC_RA_MAX  || chip->ra_config.ra_cc1 <= CC_RA_MIN) {
		chg_err(" cc=%d\n", chip->ra_config.ra_cc1);
		return false;
	}
	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	bq27541_i2c_txsubcmd(chip, BQ28Z610_REG_CNTL1, BQ28Z610_REG_RA_FLAG1);
	usleep_range(1000, 1000);
	bq27541_read_i2c_block(chip, BQ28Z610_REG_CNTL1, BQ28Z610_RA_FLAG_SIZE + 2, read_flag_data);
	if (((read_flag_data[3] << 8) | read_flag_data[2]) == BQ28Z610_RA_FLAG_VALUE &&
		!chip->ra_config.dbg_cc) {
		chg_err(" read_flag_data[%*ph]\n", BQ28Z610_RA_FLAG_SIZE + 2, read_flag_data);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		return false;
	}
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);

	return true;
}

int bq28z610_fcc_ra0_init(struct chip_bq27541 *chip)
{
	int rc = 0;
	u8 read_data[BQ28Z610_RA_COLUMN_SIZE][BQ28Z610_FCC_RA0_SIZE + 2] = { 0 };
	u8 write_data[BQ28Z610_RA_COLUMN_SIZE][BQ28Z610_FCC_RA0_SIZE] = { 0 };
	int addr_main0 = 0;
	int addr_sub0 = 0;
	int cell0_ra0 = 0;
	int cell0_ra1 = 0;
	int cell1_ra0 = 0;
	int cell1_ra1 = 0;
	int cell0_write_ra0 = 0;
	int cell1_write_ra0 = 0;
	int err_info = 0;

	if (!bq28z610_fcc_ra0_get_status(chip))
		return 0;

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	if (!bq8z610_deep_init(chip)) {
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		return -1;
	}
	bq27541_i2c_txsubcmd(chip, BQ28Z610_REG_CNTL1, BQ28Z610_REG_RA0_MAIN0);
	usleep_range(1000, 1000);
	bq27541_read_i2c_block(chip, BQ28Z610_REG_CNTL1, BQ28Z610_FCC_RA0_SIZE + 2, read_data[0]);

	bq27541_i2c_txsubcmd(chip, BQ28Z610_REG_CNTL1, BQ28Z610_REG_RA0_SUB0);
	usleep_range(1000, 1000);
	bq27541_read_i2c_block(chip, BQ28Z610_REG_CNTL1, BQ28Z610_FCC_RA0_SIZE + 2, read_data[1]);
	addr_main0 = (read_data[0][1] << 8) | read_data[0][0];
	addr_sub0 = (read_data[1][1] << 8) | read_data[1][0];
	cell0_ra0 = (read_data[0][3] << 8) | read_data[0][2];
	cell0_ra1 = (read_data[0][5] << 8) | read_data[0][4];
	cell1_ra0 = (read_data[1][3] << 8) | read_data[1][2];
	cell1_ra1 = (read_data[1][5] << 8) | read_data[1][4];
	chg_info("cell0_ra0=%d, cell0_ra1=%d, cell1_ra0=%d, cell1_ra1=%d\n", cell0_ra0, cell0_ra1, cell1_ra0, cell1_ra1);
	if ((addr_main0 != BQ28Z610_REG_RA0_MAIN0 || addr_sub0 != BQ28Z610_REG_RA0_SUB0 ||
		cell0_ra0 <= cell0_ra1 || cell1_ra0 <= cell1_ra1) && !chip->ra_config.dbg_cc) {
		chg_err("ra0 <= ra1, no need write\n");
		bq8z610_deep_deinit(chip);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		return 0;
	}
	cell0_write_ra0 = min(cell0_ra1, BQ28Z610_RA1_CHECK_MAX);
	write_data[0][0] = cell0_write_ra0 & 0xFF;
	write_data[0][1] = cell0_write_ra0 >> 8;
	write_data[0][2] = write_data[0][0];
	write_data[0][3] = write_data[0][1];
	memmove(write_data[2], write_data[0], 4);

	cell1_write_ra0 = min(cell1_ra1, BQ28Z610_RA1_CHECK_MAX);
	write_data[1][0] = cell1_write_ra0 & 0xFF;
	write_data[1][1] = cell1_write_ra0 >> 8;
	write_data[1][2] = write_data[1][0];
	write_data[1][3] = write_data[1][1];
	memmove(write_data[3], write_data[1], BQ28Z610_FCC_RA0_SIZE);

	rc = bq28z610_write_block(chip, BQ28Z610_REG_RA0_MAIN1, write_data[2], BQ28Z610_FCC_RA0_SIZE, 0, false);
	if (rc) {
		chg_err("BQ28Z610_REG_RA0_MAIN1 set fail rc = %d\n", rc);
		err_info |= BIT(1);
	}
	usleep_range(100000, 100000);

	rc = bq28z610_write_block(chip, BQ28Z610_REG_RA0_SUB1, write_data[3], BQ28Z610_FCC_RA0_SIZE, 0, false);
	if (rc) {
		chg_err("BQ28Z610_REG_RA0_SUB1 set fail rc = %d\n", rc);
		err_info |= BIT(3);
	}
	usleep_range(100000, 100000);

	rc = bq28z610_write_block(chip, BQ28Z610_REG_RA0_MAIN0, write_data[0], BQ28Z610_FCC_RA0_SIZE, 0, false);
	if (rc) {
		chg_err("BQ28Z610_REG_RA0_MAIN0 set fail rc = %d\n", rc);
		err_info |= BIT(0);
	}
	usleep_range(100000, 100000);

	rc = bq28z610_write_block(chip, BQ28Z610_REG_RA0_SUB0, write_data[1], BQ28Z610_FCC_RA0_SIZE, 0, false);
	if (rc || chip->ra_config.dbg_cc) {
		chg_err("BQ28Z610_REG_RA0_SUB0 set fail rc = %d\n", rc);
		err_info |= BIT(2);
	}
	usleep_range(100000, 100000);

	if (err_info) {
		chg_err("retry_write_ra_flag no need err_info = 0x%x\n", err_info);
		goto FFC_RA0_INIT_DONE;
	}

	write_data[0][0] = BQ28Z610_RA_FLAG_VALUE & 0xFF;
	write_data[0][1] = BQ28Z610_RA_FLAG_VALUE >> 8;
	rc = bq28z610_write_block(chip, BQ28Z610_REG_RA_FLAG0, write_data[0], BQ28Z610_RA_FLAG_SIZE, 0, false);
	if (rc) {
		chg_err("BQ28Z610_REG_RA0_SUB0 set fail rc = %d\n", rc);
		err_info |= BIT(4);
	}

	chg_info(" write success\n");

FFC_RA0_INIT_DONE:
	bq8z610_deep_deinit(chip);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);

	memset(&(chip->fcc_msg.ra0_msg), 0, sizeof(chip->fcc_msg.ra0_msg));
	chip->fcc_msg.ra0_index = 0;
	chip->fcc_msg.ra0_index += scnprintf(&(chip->fcc_msg.ra0_msg[chip->fcc_msg.ra0_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.ra0_index,
		"$$err_scene@@charge_ra0_info$$err_reason@@%s$$write_info@@%d", "batt_ra0_info", err_info);

	chip->fcc_msg.ra0_index += scnprintf(&(chip->fcc_msg.ra0_msg[chip->fcc_msg.ra0_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.ra0_index,
		"$$cell0_ra0@@%d$$cell0_ra1@@%d$$cell1_ra0@@%d$$cell1_ra1@@%d",
		cell0_ra0, cell0_ra1, cell1_ra0, cell1_ra1);

	schedule_delayed_work(&chip->track_fcc_ra0_work, msecs_to_jiffies(8000));
	return 0;
}

#define TRACK_LOCAL_T_NS_TO_S_THD		1000000000
#define TRACK_UPLOAD_COUNT_MAX			10
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD	(24 * 3600)
static int bq28z610_track_get_local_time_s(void)
{
	int local_time_s;

	local_time_s = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	return local_time_s;
}

static int bq28z610_track_upload_ra0_msg(struct chip_bq27541 *chip)
{
	int *index = 0;
	int curr_time;
	static int r0_upload_count = 0;
	static int r0_pre_upload_time = 0;
	struct oplus_mms *err_topic;
	struct mms_msg *msg = NULL;
	int rc = 0;

	if (NULL == chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -EINVAL;
	}

	curr_time = bq28z610_track_get_local_time_s();
	if (curr_time - r0_pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		r0_upload_count = 0;

	if (r0_upload_count > TRACK_UPLOAD_COUNT_MAX) {
		chg_info(" r0_upload_count = %d > max %d, should return\n",
			 r0_upload_count, TRACK_UPLOAD_COUNT_MAX);
		return 0;
	}

	r0_upload_count++;
	r0_pre_upload_time = bq28z610_track_get_local_time_s();

	index = &(chip->fcc_msg.ra0_index);
	*index += scnprintf(&(chip->fcc_msg.ra0_msg[*index]), REASON_LENGTH_MAX - *index,
		"$$device_id@@%s", "bq28z610");
	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
        "[%s]-[%d]-[%d]:%s", "bq28z610", OPLUS_IC_ERR_GAUGE, TRACK_GAGUE_FCC_RA0_ERR_INFO, chip->fcc_msg.ra0_msg);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -EINVAL;
	}
	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return 0;
}

static int bq28z610_track_get_ra0_msg(struct chip_bq27541 *chip)
{
	int fcc = 0;
	int qmax_1 = 0;
	int qmax_2 = 0;
	int qmax_passed_q = 0;
	int soc = 0;

	fcc = bq27541_get_battery_fcc(chip);
	bq28z610_get_2cell_voltage(chip);
	bq28z610_get_qmax_parameters(chip, &qmax_1, &qmax_2, &qmax_passed_q);
	soc = bq27541_get_battery_soc(chip);

	chip->fcc_msg.ra0_index += scnprintf(&(chip->fcc_msg.ra0_msg[chip->fcc_msg.ra0_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.ra0_index,
		"$$batt_soc@@%d$$cell0_vbat@@%d$$cell1_vbat@@%d$$batt_cc@@%d$$batt_fcc@@%d$$qmax_1@@%d$$qmax_2@@%d",
		soc, chip->batt_cell_1_vol, chip->batt_cell_2_vol, chip->ra_config.ra_cc1, fcc, qmax_1, qmax_2);

	return 0;
}

void bq27541_track_fcc_ra0_work(struct work_struct *work)
{
	struct chip_bq27541 *chip = container_of(
		work, struct chip_bq27541, track_fcc_ra0_work.work);

	bq28z610_track_get_ra0_msg(chip);
	bq28z610_track_upload_ra0_msg(chip);
}

int bq28z610_fcc_vdelta_init(struct chip_bq27541 *chip)
{
	int ret = 0;
	bq28z610_fcc_init_cc0(chip);
	if (chip && chip->batt_bq28z610 && chip->fcc_vdelta_support) {
		bq28z610_fcc_init_cc1(chip);
		ret = bq28z610_delta_volt_clear(chip, true);
	}

	return ret;
}

#define BQ28Z610_DLETA_VOLT_SIZE        2
#define BQ28Z610_VDLETA_TRACK_DELAY     10000
int bq28z610_delta_volt_clear(struct chip_bq27541 *chip, bool init)
{
	int rc = 0;
	u8 read_data[BQ28Z610_DLETA_VOLT_SIZE + 2] = { 0 };
	u8 write_data[BQ28Z610_DLETA_VOLT_SIZE] = { 0 };
	int err_info = 0;
	int track_time = BQ28Z610_VDLETA_TRACK_DELAY;

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	if (!bq8z610_deep_init(chip)) {
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		return -1;
	}

	bq27541_i2c_txsubcmd(chip, BQ28Z610_REG_CNTL1, BQ28Z610_REG_DELTA_VOL);
	usleep_range(1000, 1000);
	bq27541_read_i2c_block(chip, BQ28Z610_REG_CNTL1, BQ28Z610_DLETA_VOLT_SIZE + 2, read_data);
	chip->ra_config.ra_vd_cc0 = chip->ra_config.ra_cc1;
	if ((((read_data[3] << 8) | read_data[2]) <= chip->ra_vd_curve.delta_vmax) && !chip->ra_config.dbg_cc) {
		chg_err(" clear read_data[%*ph]\n", BQ28Z610_DLETA_VOLT_SIZE + 2, read_data);
		bq8z610_deep_deinit(chip);
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		return 0;
	}

	rc = bq28z610_write_block(chip, BQ28Z610_REG_DELTA_VOL, write_data, 2, 0, false);
	if (rc) {
		chg_err(" set fail rc = %d\n", rc);
		err_info |= BIT(0);
	}

	chg_info(" clear success\n");
	bq8z610_deep_deinit(chip);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);

	track_time = init ? track_time : 0;
	memset(&(chip->fcc_msg.vdelta_msg), 0, sizeof(chip->fcc_msg.vdelta_msg));
	chip->fcc_msg.vdelta_index = 0;
	chip->fcc_msg.vdelta_index += scnprintf(&(chip->fcc_msg.vdelta_msg[chip->fcc_msg.vdelta_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.vdelta_index,
		"$$err_scene@@charge_delta_volt$$err_reason@@%s$$write_info@@%d$$delta_volt@@%d",
		"charge_delta_volt", err_info, ((read_data[3] << 8) | read_data[2]));

	schedule_delayed_work(&chip->track_fcc_vdelta_work, msecs_to_jiffies(track_time));
	return 0;
}

static int bq28z610_track_upload_vdelta_msg(struct chip_bq27541 *chip)
{
	int *index = 0;
	int curr_time;
	static int vdelta_upload_count = 0;
	static int vdelta_pre_upload_time = 0;
	struct oplus_mms *err_topic;
	struct mms_msg *msg = NULL;
	int rc = 0;

	if (NULL == chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -EINVAL;
	}

	curr_time = bq28z610_track_get_local_time_s();
	if (curr_time - vdelta_pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		vdelta_upload_count = 0;

	if (vdelta_upload_count > TRACK_UPLOAD_COUNT_MAX) {
		chg_info(" vdelta_upload_count = %d > max %d, should return\n",
			 vdelta_upload_count, TRACK_UPLOAD_COUNT_MAX);
		return 0;
	}

	vdelta_upload_count++;
	vdelta_pre_upload_time = bq28z610_track_get_local_time_s();

	index = &(chip->fcc_msg.vdelta_index);
	*index += scnprintf(&(chip->fcc_msg.vdelta_msg[*index]), REASON_LENGTH_MAX - *index,
		"$$device_id@@%s", "bq28z610");
	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
		"[%s]-[%d]-[%d]:%s", "bq28z610", OPLUS_IC_ERR_GAUGE, TRACK_GAGUE_FCC_VDELTA_ERR_INFO, chip->fcc_msg.vdelta_msg);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -EINVAL;
	}
	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return 0;
}

static int bq28z610_track_get_vdelta_msg(struct chip_bq27541 *chip)
{
	int fcc = 0;
	int qmax_1 = 0;
	int qmax_2 = 0;
	int qmax_passed_q = 0;
	int soc = 0;

	fcc = bq27541_get_battery_fcc(chip);
	bq28z610_get_2cell_voltage(chip);
	bq28z610_get_qmax_parameters(chip, &qmax_1, &qmax_2, &qmax_passed_q);
	soc = bq27541_get_battery_soc(chip);

	chip->fcc_msg.vdelta_index += scnprintf(&(chip->fcc_msg.vdelta_msg[chip->fcc_msg.vdelta_index]),
        REASON_LENGTH_MAX - chip->fcc_msg.vdelta_index,
		"$$batt_soc@@%d$$cell0_vbat@@%d$$cell1_vbat@@%d$$batt_cc@@%d$$batt_fcc@@%d$$qmax_1@@%d$$qmax_2@@%d",
		soc, chip->batt_cell_1_vol, chip->batt_cell_2_vol, chip->ra_config.ra_cc1, fcc, qmax_1, qmax_2);

	return 0;
}

void bq27541_track_fcc_vdelta_work(struct work_struct *work)
{
	struct chip_bq27541 *chip = container_of(
		work, struct chip_bq27541, track_fcc_vdelta_work.work);

	bq28z610_track_get_vdelta_msg(chip);
	bq28z610_track_upload_vdelta_msg(chip);
}

static int bq28z610_track_get_ra_t_msg(struct chip_bq27541 *chip)
{
	int fcc = 0;
	int qmax_1 = 0;
	int qmax_2 = 0;
	int qmax_passed_q = 0;
	int soc = 0;
	int i = 0;

	fcc = bq27541_get_battery_fcc(chip);
	bq28z610_get_2cell_voltage(chip);
	bq28z610_get_qmax_parameters(chip, &qmax_1, &qmax_2, &qmax_passed_q);
	soc = bq27541_get_battery_soc(chip);

	chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index,
		"$$batt_soc@@%d$$cell0_vbat@@%d$$cell1_vbat@@%d$$batt_cc@@%d$$batt_fcc@@%d$$qmax_1@@%d$$qmax_2@@%d",
		soc, chip->batt_cell_1_vol, chip->batt_cell_2_vol, chip->ra_config.ra_cc1, fcc, qmax_1, qmax_2);

	chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index,
		"$$err_scene@@charge_ra_t_info$$err_reason@@%s$$vd_cc0@@%d$$cc1@@%d$$ra_cnts@@%d$$fcc_thr@@%d$$write_info@@%d",
		"charge_ra_t_info", chip->ra_config.ra_vd_cc0, chip->ra_config.ra_cc1,
		chip->ra_config.ra_cnts, chip->ra_t_curve.fcc_thr, chip->fcc_msg.ra_t_err);

	chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index, "$$ra_0@@");

	for (i = 0; i < FCC_RA_DEFAULT_NUM; i++)
		chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
			REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index, "%d,", chip->ra_res_curve.ra_0[i]);

	chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index, "$$ra_1@@");

	for (i = 0; i < FCC_RA_DEFAULT_NUM; i++)
		chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
			REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index, "%d,", chip->ra_res_curve.ra_1[i]);

	chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index, "$$ra_thr@@");

	for (i = 0; i < FCC_RA_DEFAULT_NUM; i++)
		chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
			REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index, "%d,", chip->ra_res_curve.ra_thr[i]);

	chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index, "$$ra_cali@@");

	for (i = 0; i < FCC_RA_DEFAULT_NUM; i++)
		chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
			REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index, "%d,", chip->ra_res_curve.ra_cali[i]);

	return 0;
}

static int bq28z610_track_upload_ra_t_msg(struct chip_bq27541 *chip)
{
	int *index = 0;
	int curr_time;
	static int vdelta_upload_count = 0;
	static int vdelta_pre_upload_time = 0;
	struct oplus_mms *err_topic;
	struct mms_msg *msg = NULL;
	int rc = 0;

	if (NULL == chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -EINVAL;
	}

	curr_time = bq28z610_track_get_local_time_s();
	if (curr_time - vdelta_pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		vdelta_upload_count = 0;

	if (vdelta_upload_count > TRACK_UPLOAD_COUNT_MAX) {
		chg_info(" vdelta_upload_count = %d > max %d, should return\n",
			 vdelta_upload_count, TRACK_UPLOAD_COUNT_MAX);
		return 0;
	}

	vdelta_upload_count++;
	vdelta_pre_upload_time = bq28z610_track_get_local_time_s();

	index = &(chip->fcc_msg.ra_t_index);
	*index += scnprintf(&(chip->fcc_msg.ra_t_msg[*index]), REASON_LENGTH_MAX - *index,
		"$$device_id@@%s", "bq28z610");
	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
		"[%s]-[%d]-[%d]:%s", "bq28z610", OPLUS_IC_ERR_GAUGE, TRACK_GAGUE_FCC_RA_T_ERR_INFO, chip->fcc_msg.ra_t_msg);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -EINVAL;
	}
	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}

	return 0;
}

void bq27541_track_fcc_ra_t_work(struct work_struct *work)
{
	struct chip_bq27541 *chip = container_of(
		work, struct chip_bq27541, track_fcc_ra_t_work.work);

	bq28z610_track_get_ra_t_msg(chip);
	bq28z610_track_upload_ra_t_msg(chip);
}

#define FCC_RA_T_CC_INIT_DELTA		2
#define BQ28Z610_RA_T_FLAG_CMD		0x0070
#define BQ28Z610_REG_RA_T_FLAG		0x4041
#define BQ28Z610_RA_T_FLAG_SIZE		4
static bool bq28z610_check_ra_t_flag(struct chip_bq27541 *chip)
{
	u8 check_sum = 0;
	u8 read_data[BQ28Z610_RA_T_FLAG_SIZE + 2] = { 0 };

	if (!chip || is_return_pre_value(chip) || !chip->batt_bq28z610)
		return false;

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	bq27541_i2c_txsubcmd(chip, BQ28Z610_REG_CNTL1, BQ28Z610_RA_T_FLAG_CMD);
	usleep_range(1000, 1000);
	bq27541_read_i2c_block(chip, BQ28Z610_REG_CNTL1, BQ28Z610_RA_T_FLAG_SIZE + 2, read_data);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	if (((read_data[1] << 8) + read_data[0]) != BQ28Z610_RA_T_FLAG_CMD) {
		chg_err("addr not BQ28Z610_RA_T_FLAG_CMD, [0x%x, 0x%x]\n", read_data[0], read_data[1]);
		chip->ra_config.ra_cnts = 0;
		return true;
	}
	check_sum = bq28z610_calc_checksum(&read_data[2], 3);
	chip->ra_config.l_cc = (read_data[3] << 8) + read_data[2];
	chip->ra_config.ra_cnts = read_data[4];
	chg_info(" cc[%d, %d, %d, %d]\n", chip->ra_config.ra_cc1, chip->ra_config.l_cc,
		chip->ra_config.dbg_cc, chip->ra_config.ra_cnts);
	if (((read_data[3] << 8) + read_data[2]) && read_data[4] && check_sum == read_data[5] &&
		chip->ra_config.l_cc > CC_RA_MIN && chip->ra_config.l_cc < CC_RA_MAX) {
		return false;
	} else {
		chip->ra_config.ra_cnts = 0;
		return true;
	}
}

static void bq28z610_fcc_init_cc_index(struct chip_bq27541 *chip)
{
	int i;

	bq28z610_fcc_init_cc1(chip);
	for (i = FCC_RA_CC_THR_NUM - 1; i >= 0; i--) {
		if (chip->ra_config.ra_cc1 >= chip->ra_t_curve.cc_thr[i])
			break;
	}
	if (i < 0 || i >= FCC_RA_CC_THR_NUM)
		i = 0;

	chip->ra_t_curve.cc_index = i;
}

static bool bq28z610_check_ra_fcc(struct chip_bq27541 *chip)
{
	int fcc = 0;

	if (!chip || is_return_pre_value(chip) || !chip->batt_bq28z610)
		return false;

	fcc = bq27541_get_battery_fcc(chip) * 2;
	if (fcc < chip->ra_t_curve.fcc_thr || chip->ra_config.dbg_cc) {
		chg_err("fcc = %d, fcc_thr = %d\n", fcc, chip->ra_t_curve.fcc_thr);
		return true;
	}

	return false;
}

#define BQ28Z610_RA_T_VBAT_ERR	    	200
#define BQ28Z610_RA_T_TBAT_MIN		160
#define BQ28Z610_RA_T_TBAT_MAX		440
static bool bq28z610_check_ra_t_delta_cc(struct chip_bq27541 *chip)
{
	int temp = 0;

	if (!chip || is_return_pre_value(chip) || !chip->batt_bq28z610)
		return false;

	bq28z610_get_2cell_voltage(chip);
	temp = bq27541_get_battery_temperature(chip);
	chip->ra_t_curve.fcc_thr = chip->ra_t_curve.fcc0_thr[chip->ra_t_curve.cc_index] -
	chip->ra_config.ra_cc1 * chip->ra_t_curve.k_curve[chip->ra_t_curve.cc_index].k0 / 1000;

	if ((chip->ra_config.ra_cc1 - chip->ra_config.l_cc) > chip->ra_t_curve.delta_cc0 &&
		abs(chip->batt_cell_1_vol - chip->batt_cell_2_vol) < BQ28Z610_RA_T_VBAT_ERR)
		return true;

	if ((chip->ra_config.ra_cc1 - chip->ra_config.l_cc) > chip->ra_t_curve.delta_cc1 &&
		temp > BQ28Z610_RA_T_TBAT_MIN && temp < BQ28Z610_RA_T_TBAT_MAX && bq28z610_check_ra_fcc(chip))
		return true;

	return false;
}

static bool bq28z610_fcc_ra_check_t_status(struct chip_bq27541 *chip)
{
	if (bq28z610_check_ra_t_flag(chip))
		return true;

	if (bq28z610_check_ra_t_delta_cc(chip))
		return true;

	return false;
}

static bool bq28z610_fcc_check_extreme_abnormal(struct chip_bq27541 *chip)
{
	int i = 0;
	int extreme_thr = chip->ra_t_curve.extreme_thr[chip->ra_t_curve.cc_index];

	for (i = 0; i < FCC_RA_DEFAULT_NUM - 1; i++) {
		if (chip->ra_res_curve.ra_0[i] > extreme_thr ||
			chip->ra_res_curve.ra_1[i] > extreme_thr) {
			chg_err("ra[%d] = [%d, %d], extreme_thr[%d] = %d\n", i,
				chip->ra_res_curve.ra_0[i], chip->ra_res_curve.ra_1[i], chip->ra_t_curve.cc_index, extreme_thr);
			return true;
		}
	}
	return false;
}

static int bq28z610_update_ra_t_flag(struct chip_bq27541 *chip)
{
	int rc = 0;
	u8 write_data[BQ28Z610_RA_T_FLAG_SIZE] = { 0 };

	if (!chip || is_return_pre_value(chip) || !chip->batt_bq28z610)
		return -EINVAL;

	chip->ra_config.l_cc = chip->ra_config.ra_cc1;
	chip->ra_config.ra_cnts++;
	write_data[0] = chip->ra_config.l_cc & 0xFF;
	write_data[1] = (chip->ra_config.l_cc >> 8) & 0xFF;
	write_data[2] = chip->ra_config.ra_cnts & 0xFF;

	rc = bq28z610_write_block(chip, BQ28Z610_REG_RA_T_FLAG, write_data, BQ28Z610_RA_T_FLAG_SIZE - 1, 0, true);
	if (rc) {
		chg_err("BQ28Z610_REG_RA_T_FLAG set fail rc = %d\n", rc);
		chip->fcc_msg.ra_t_err |= BIT(6);
		return -EINVAL;
	}

	return rc;
}

#define BQ28Z610_REG_RA_THR_BEGIN		9
#define BQ28Z610_REG_RA_THR_END			13
#define ROUND_DOWN_TO_50(x)			((x) / 50 * 50)
#define ROUND_UP_TO_50(x)			((x) / 50 * 50 + 50)
#define FCC_RA_K_RATIO				1000
static int bq28z610_fcc_ra_cali_t_res(struct chip_bq27541 *chip)
{
	int rc = 0;
	u8 read_data[32] = { 0 };
	u8 write_data[32] = { 0 };
	int cc = 0;
	int i = 0;
	int l_cc = 0;
	bool ra_abnormal = true;

	cc = chip->ra_config.ra_cc1;
	l_cc = chip->ra_config.l_cc;
	for (i = 0; i < FCC_RA_DEFAULT_NUM; i++)
		chip->ra_res_curve.ra_thr[i] = chip->ra_res_curve.ra_default[i] *
			(chip->ra_t_curve.k_curve[chip->ra_t_curve.cc_index].k1 + ROUND_DOWN_TO_50(cc)) / FCC_RA_K_RATIO;

	for (i = 0; i <= 4; i++)
		chip->ra_res_curve.ra_cali[i] = chip->ra_res_curve.ra_default[i] *
			(chip->ra_t_curve.k_curve[chip->ra_t_curve.cc_index].k2 + ROUND_DOWN_TO_50(cc)) / FCC_RA_K_RATIO;

	for (i = 5; i <= 8; i++)
		chip->ra_res_curve.ra_cali[i] = chip->ra_res_curve.ra_default[i] *
			(chip->ra_t_curve.k_curve[chip->ra_t_curve.cc_index].k3 + ROUND_DOWN_TO_50(cc)) / FCC_RA_K_RATIO;

	for (i = 9; i <= 14; i++)
		chip->ra_res_curve.ra_cali[i] = chip->ra_res_curve.ra_default[i] *
			(chip->ra_t_curve.k_curve[chip->ra_t_curve.cc_index].k4 + ROUND_DOWN_TO_50(cc)) / FCC_RA_K_RATIO;

	mutex_lock(&chip->bq28z610_alt_manufacturer_access);
	if (!bq8z610_deep_init(chip)) {
		mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
		return -1;
	}

	bq27541_i2c_txsubcmd(chip, BQ28Z610_REG_CNTL1, BQ28Z610_REG_RA0_MAIN0);
	usleep_range(1000, 1000);
	bq27541_read_i2c_block(chip, BQ28Z610_REG_CNTL1, FCC_RA_DEFAULT_NUM * 2 + 2, read_data);
	if (((read_data[1] << 8) | read_data[0]) != BQ28Z610_REG_RA0_MAIN0) {
		chg_err("addr0 error, read_data[%*ph]\n", 32, read_data);
		goto extreme_check;
	}
	for (i = 0; i < FCC_RA_DEFAULT_NUM * 2; i += 2)
		chip->ra_res_curve.ra_0[i / 2] = ((read_data[i + 3] << 8) | read_data[i + 2]);

	bq27541_i2c_txsubcmd(chip, BQ28Z610_REG_CNTL1, BQ28Z610_REG_RA0_SUB0);
	usleep_range(1000, 1000);
	bq27541_read_i2c_block(chip, BQ28Z610_REG_CNTL1, FCC_RA_DEFAULT_NUM * 2 + 2, read_data);
	if (((read_data[1] << 8) | read_data[0]) != BQ28Z610_REG_RA0_SUB0) {
		chg_err("addr1 error, read_data[%*ph]\n", 32, read_data);
		goto extreme_check;
	}
	for (i = 0; i < FCC_RA_DEFAULT_NUM * 2; i += 2)
		chip->ra_res_curve.ra_1[i / 2] = ((read_data[i + 3] << 8) | read_data[i + 2]);

	for (i = BQ28Z610_REG_RA_THR_BEGIN; i <= BQ28Z610_REG_RA_THR_END; i++) {
		if (chip->ra_res_curve.ra_0[i] < chip->ra_res_curve.ra_thr[i] ||
			chip->ra_res_curve.ra_1[i] < chip->ra_res_curve.ra_thr[i]) {
			ra_abnormal = false;
			goto extreme_check;
		}
	}
	if (ra_abnormal) {
		chg_err("BQ28Z610_REG_RA0_MAIN0 & BQ28Z610_REG_RA0_SUB0 all need cali \n");
		chip->fcc_msg.ra_t_err |= BIT(0);
		goto write_t_ra;
	}

extreme_check:
	if (!bq28z610_fcc_check_extreme_abnormal(chip) && !chip->ra_config.dbg_cc) {
		chip->fcc_msg.ra_t_err |= BIT(7);
		goto ra_check_done;
	} else {
		chip->fcc_msg.ra_t_err |= BIT(1);
	}

write_t_ra:
	for (i = 0; i < FCC_RA_DEFAULT_NUM; i++) {
		write_data[i * 2] = chip->ra_res_curve.ra_cali[i] & 0xFF;
		write_data[i * 2 + 1] = (chip->ra_res_curve.ra_cali[i] >> 8) & 0xFF;
	}
	rc = bq28z610_write_block(chip, BQ28Z610_REG_RA0_MAIN1, write_data, FCC_RA_DEFAULT_NUM * 2, 0, false);
	if (rc) {
		chg_err("BQ28Z610_REG_RA0_MAIN1 set fail rc = %d\n", rc);
		chip->fcc_msg.ra_t_err |= BIT(3);
		goto ra_check_done;
	}
	usleep_range(100000, 100000);

	rc = bq28z610_write_block(chip, BQ28Z610_REG_RA0_SUB1, write_data, FCC_RA_DEFAULT_NUM * 2, 0, false);
	if (rc) {
		chg_err("BQ28Z610_REG_RA0_SUB1 set fail rc = %d\n", rc);
		chip->fcc_msg.ra_t_err |= BIT(5);
		goto ra_check_done;
	}
	usleep_range(100000, 100000);

	rc = bq28z610_write_block(chip, BQ28Z610_REG_RA0_MAIN0, write_data, FCC_RA_DEFAULT_NUM * 2, 0, false);
	if (rc) {
		chg_err("BQ28Z610_REG_RA0_MAIN0 set fail rc = %d\n", rc);
		chip->fcc_msg.ra_t_err |= BIT(2);
		goto ra_check_done;
	}
	usleep_range(100000, 100000);

	rc = bq28z610_write_block(chip, BQ28Z610_REG_RA0_SUB0, write_data, FCC_RA_DEFAULT_NUM * 2, 0, false);
	if (rc) {
		chg_err("BQ28Z610_REG_RA0_SUB0 set fail rc = %d\n", rc);
		chip->fcc_msg.ra_t_err |= BIT(4);
		goto ra_check_done;
	}
	usleep_range(100000, 100000);

	bq28z610_update_ra_t_flag(chip);
	chg_info(" success\n");

ra_check_done:
	bq8z610_deep_deinit(chip);
	mutex_unlock(&chip->bq28z610_alt_manufacturer_access);
	memset(&(chip->fcc_msg.ra_t_msg), 0, sizeof(chip->fcc_msg.ra_t_msg));
	chip->fcc_msg.ra_t_index = 0;
	chip->fcc_msg.ra_t_index += scnprintf(&(chip->fcc_msg.ra_t_msg[chip->fcc_msg.ra_t_index]),
		REASON_LENGTH_MAX - chip->fcc_msg.ra_t_index, "$$last_cc@@%d$$t_cc0@@%d", l_cc,  chip->ra_config.ra_t_cc0);
	chip->ra_config.ra_t_cc0 = chip->ra_config.ra_cc1;
	schedule_delayed_work(&chip->track_fcc_ra_t_work, msecs_to_jiffies(0));
	return 0;
}

void bq28z610_fcc_ra_t_check(struct chip_bq27541 *chip)
{
	if (!chip->fcc_ra_t_support)
		return;

	chip->fcc_msg.ra_t_err = 0;
	bq28z610_fcc_init_cc_index(chip);
	if (chip->ra_config.ra_cc1 >= CC_RA_MAX  || chip->ra_config.ra_cc1 <= CC_RA_MIN) {
		chg_err(" cc=[%d, %d]\n", chip->ra_config.ra_cc1, chip->ra_config.ra_t_cc0);
		return;
	}
	if (chip->ra_config.ra_cc1 < chip->ra_config.ra_t_cc0 + FCC_RA_T_CC_INIT_DELTA) {
		chg_err(" cc=[%d, %d]\n", chip->ra_config.ra_cc1, chip->ra_config.ra_t_cc0);
		return;
	}
	if (bq28z610_fcc_ra_check_t_status(chip))
		bq28z610_fcc_ra_cali_t_res(chip);
}

static ssize_t bq28z610_fcc_cc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#define ERR_MSG_BUF		PAGE_SIZE
	struct chip_bq27541 *chip = dev_get_drvdata(dev);

	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return scnprintf(buf, ERR_MSG_BUF, "chip->ra_config.dbg_cc = %d\n", chip->ra_config.dbg_cc);
}

#define DBG_RA_CC_MAX		5000
#define DBG_RA_CC_MIN		0
static ssize_t bq28z610_fcc_cc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct chip_bq27541 *chip = dev_get_drvdata(dev);
	int dbg_cc = 0;

	if (!buf) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}

	if (!chip) {
		chg_err("bq27541_device is NULL\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &dbg_cc) != 1) {
		chg_err("invalid buff %s\n", buf);
		return -EINVAL;
	}

	if (dbg_cc > DBG_RA_CC_MAX || dbg_cc < DBG_RA_CC_MIN) {
		chg_err("dbg_cc = %d invalid\n", dbg_cc);
		return -EINVAL;
	}
	chip->ra_config.dbg_cc = dbg_cc;
	chg_info(" dbg_cc = %d\n", chip->ra_config.dbg_cc);

	return count;
}
static DEVICE_ATTR(fcc_dbg_cc, 0660, bq28z610_fcc_cc_show, bq28z610_fcc_cc_store);

void bq27541_create_device_node(struct device *dev)
{
	int ret = 0;

	ret = device_create_file(dev, &dev_attr_fcc_dbg_cc);
	if (ret)
		chg_info("create dbg cc node failed\n");
}


