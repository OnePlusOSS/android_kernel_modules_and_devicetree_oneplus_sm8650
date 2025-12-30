// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/delay.h>
#include "ft8057p_core.h"

/*******Part0:LOG TAG Declear********************/

#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "ft8057p-test"
#else
#define TPD_DEVICE "ft8057p-test"
#endif

#define FTS_TEST_FUNC_ENTER() do { \
	TPD_INFO("[FTS_TS][TEST]%s: Enter\n", __func__); \
} while (0)

#define FTS_TEST_FUNC_EXIT()  do { \
	TPD_INFO("[FTS_TS][TEST]%s: Exit(%d)\n", __func__, __LINE__); \
} while (0)


#define FTS_TEST_SAVE_INFO(fmt, args...) do { \
	if (g_fts_data->s) { \
        seq_printf(g_fts_data->s, fmt, ##args); \
	} \
	TPD_INFO(fmt, ##args); \
} while (0)

#define FTS_TEST_SAVE_ERR(fmt, args...)  do { \
	if (g_fts_data->s) { \
        seq_printf(g_fts_data->s, fmt, ##args); \
	} \
	TPD_INFO(fmt, ##args); \
} while (0)

enum byte_mode {
	DATA_ONE_BYTE,
	DATA_TWO_BYTE,
};

enum normalize_type {
	NORMALIZE_OVERALL,
	NORMALIZE_AUTO,
};

static void sys_delay(int ms)
{
	msleep(ms);
}

int focal_abs(int value)
{
	if (value < 0) {
		value = 0 - value;
	}

	return value;
}

void print_buffer(int *buffer, int length, int line_num)
{
	int i = 0;
	int j = 0;
	int tmpline = 0;
	char *tmpbuf = NULL;
	int tmplen = 0;
	int cnt = 0;

	if ((NULL == buffer) || (length <= 0)) {
		TPD_INFO("buffer/length(%d) fail", length);
		return;
	}

	tmpline = line_num ? line_num : length;
	tmplen = tmpline * 6 + 128;
	tmpbuf = kzalloc(tmplen, GFP_KERNEL);

	if (!tmpbuf) {
		TPD_INFO("%s, alloc failed \n", __func__);
		return;
	}

	for (i = 0; i < length; i = i + tmpline) {
		cnt = 0;

		for (j = 0; j < tmpline; j++) {
			cnt += snprintf(tmpbuf + cnt, tmplen - cnt, "%5d ", buffer[i + j]);

			if ((cnt >= tmplen) || ((i + j + 1) >= length)) {
				break;
			}
		}

		TPD_DEBUG("%s", tmpbuf);
	}

	if (tmpbuf) {
		kfree(tmpbuf);
		tmpbuf = NULL;
	}
}

#define NODE_MATCH      1
#define CHANNEL_MATCH   2
#define CHEN_MATCH      3
#define ONE_MATCH       4
int ft8057p_output_data(int *buffer, struct chip_data_ft8057p *ts_data,
                       struct auto_testdata *focal_testdata, int limit_type)
{
	uint8_t data_buf[64];
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int i = 0;
	int num_each_line = 0;
	int data_volumn = 0;

	if (limit_type == NODE_MATCH) {
		num_each_line = rx_num;
		data_volumn = rx_num * tx_num;

	} else if (limit_type == CHANNEL_MATCH) {
		num_each_line = rx_num + tx_num;
		data_volumn = (rx_num + tx_num) * 2;
	} else if (limit_type == CHEN_MATCH) {
		num_each_line = rx_num;
		data_volumn = rx_num + tx_num;
	} else if (limit_type == ONE_MATCH) {
		num_each_line = rx_num;
		data_volumn = 1;
	}

	memset(data_buf, 0, sizeof(data_buf));

	for (i = 0; i < data_volumn; i += 1) {
		snprintf(data_buf, 64, "%d,", buffer[i]);
		tp_test_write(focal_testdata->fp, focal_testdata->length, data_buf,
		              strlen(data_buf), focal_testdata->pos);

		if (!((i + 1) % num_each_line) || (i == data_volumn - 1)) {
			snprintf(data_buf, 64, "\n");
			tp_test_write(focal_testdata->fp, focal_testdata->length, data_buf,
			              strlen(data_buf), focal_testdata->pos);
		}
	}
	return 0;
}

int ft8057p_black_output_data(int *buffer, struct chip_data_ft8057p *ts_data,
                       struct auto_testdata *focal_testdata, int limit_type)
{
	uint8_t data_buf[64];
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int i = 0;
	int num_each_line = 0;
	int data_volumn = 0;

	if (limit_type == NODE_MATCH) {
		num_each_line = rx_num;
		data_volumn = rx_num * tx_num;

	} else if (limit_type == CHANNEL_MATCH) {
		num_each_line = rx_num + tx_num;
		data_volumn = (rx_num + tx_num) * 2;
	} else if (limit_type == CHEN_MATCH) {
		num_each_line = rx_num;
		data_volumn = rx_num + tx_num;
	} else if (limit_type == ONE_MATCH) {
		num_each_line = rx_num;
		data_volumn = 1;
	}

	memset(data_buf, 0, sizeof(data_buf));

	for (i = 0; i < data_volumn; i += 1) {
		snprintf(data_buf, 64, "%d,", buffer[i]);
		tp_test_write(focal_testdata->bs_fp, focal_testdata->bs_length, data_buf,
		              strlen(data_buf), focal_testdata->bs_pos);

		if (!((i + 1) % num_each_line) || (i == data_volumn - 1)) {
			snprintf(data_buf, 64, "\n");
			tp_test_write(focal_testdata->bs_fp, focal_testdata->bs_length, data_buf,
			              strlen(data_buf), focal_testdata->bs_pos);
		}
	}
	return 0;
}

/********************************************************************
 * test read/write interface
 *******************************************************************/
static int fts_test_bus_read(u8 *cmd, int cmdlen, u8 *data, int datalen)
{
	int ret = 0;

	ret = fts_read(cmd, cmdlen, data, datalen);
	if (ret < 0)
		return ret;
	else
		return 0;
}

static int fts_test_bus_write(u8 *writebuf, int writelen)
{
	int ret = 0;

	ret = fts_write(writebuf, writelen);
	if (ret < 0)
		return ret;
	else
		return 0;
}

static int fts_test_read_reg(u8 addr, u8 *val)
{
	return fts_test_bus_read(&addr, 1, val, 1);
}

static int fts_test_write_reg(u8 addr, u8 val)
{
	int ret;
	u8 cmd[2] = {0};

	cmd[0] = addr;
	cmd[1] = val;
	ret = fts_test_bus_write(cmd, 2);

	return ret;
}

static int fts_test_read(u8 addr, u8 *readbuf, int readlen)
{
	int ret = 0;
	int i = 0;
	int packet_length = 0;
	int packet_num = 0;
	int packet_remainder = 0;
	int offset = 0;
	int byte_num = readlen;

	packet_num = byte_num / BYTES_PER_TIME;
	packet_remainder = byte_num % BYTES_PER_TIME;

	if (packet_remainder) {
		packet_num++;
	}

	if (byte_num < BYTES_PER_TIME) {
		packet_length = byte_num;

	} else {
		packet_length = BYTES_PER_TIME;
	}

	/* FTS_TEST_DBG("packet num:%d, remainder:%d", packet_num, packet_remainder); */

	ret = fts_test_bus_read(&addr, 1, &readbuf[offset], packet_length);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read buffer fail\n");
		return ret;
	}

	for (i = 1; i < packet_num; i++) {
		offset += packet_length;

		if ((i == (packet_num - 1)) && packet_remainder) {
			packet_length = packet_remainder;
		}

		ret = fts_test_bus_read(&addr, 1, &readbuf[offset], packet_length);

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("read buffer fail\n");
			return ret;
		}
	}

	return 0;
}



/*
 * read_mass_data - read rawdata/short test data
 * addr - register addr which read data from
 * byte_num - read data length, unit:byte
 * buf - save data
 *
 * return 0 if read data succuss, otherwise return error code
 */
static int read_mass_data(u8 addr, int byte_num, int *buf)
{
	int ret = 0;
	int i = 0;
	u8 *data = NULL;

	data = (u8 *)kzalloc(byte_num * sizeof(u8), GFP_KERNEL);

	if (NULL == data) {
		FTS_TEST_SAVE_ERR("mass data buffer malloc fail\n");
		return -ENOMEM;
	}

	/* read rawdata buffer */
	TPD_INFO("mass data len:%d", byte_num);
	ret = fts_test_read(addr, data, byte_num);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read mass data fail\n");
		goto read_massdata_err;
	}

	for (i = 0; i < byte_num; i = i + 2) {
		buf[i >> 1] = (int)(short)((data[i] << 8) + data[i + 1]);
	}

	ret = 0;
read_massdata_err:
	kfree(data);
	return ret;
}


/********************************************************************
 * test global function enter work/factory mode
 *******************************************************************/
static int enter_work_mode(void)
{
	int ret = 0;
	u8 mode = 0;
	int i = 0;
	int j = 0;

	TPD_INFO("%s +\n", __func__);
	ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);

	if ((ret >= 0) && (0x00 == mode)) {
		return 0;
	}

	for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
		ret = fts_test_write_reg(DEVIDE_MODE_ADDR, 0x00);

		if (ret >= 0) {
			sys_delay(FACTORY_TEST_DELAY);

			for (j = 0; j < 20; j++) {
				ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);

				if ((ret >= 0) && (0x00 == mode)) {
					TPD_INFO("enter work mode success");
					return 0;

				} else {
					sys_delay(FACTORY_TEST_DELAY);
				}
			}
		}

		sys_delay(50);
	}

	if (i >= ENTER_WORK_FACTORY_RETRIES) {
		FTS_TEST_SAVE_ERR("Enter work mode fail\n");
		return -EIO;
	}

	TPD_INFO("%s -\n", __func__);
	return 0;
}


static int fts_special_operation_for_samsung(struct chip_data_ft8057p *ts_data)
{
	int ret = 0;

	if (true ==
	    ts_data->use_panelfactory_limit) {                      /*only for firmware released to samsung factory*/
		ret = fts_test_write_reg(FTS_REG_SAMSUNG_SPECIFAL, 0x01);

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("write FTS_REG_SAMSUNG_SPECIFAL fail at %s,ret=%d\n", __func__, ret);
			return -EIO;
		}
	}

	return ret;
}


#define FTS_FACTORY_MODE 0x40
static int enter_factory_mode(struct chip_data_ft8057p *ts_data)
{
	int ret = 0;
	u8 mode = 0;
	int i = 0;
	int j = 0;

	ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);

	if ((ret >= 0) && (FTS_FACTORY_MODE == mode)) {
		fts_special_operation_for_samsung(ts_data);
		return 0;
	}

	for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
		ret = fts_test_write_reg(DEVIDE_MODE_ADDR, 0x40);

		if (ret >= 0) {
			sys_delay(FACTORY_TEST_DELAY);

			for (j = 0; j < 20; j++) {
				ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);

				if ((ret >= 0) && (FTS_FACTORY_MODE == mode)) {
					TPD_INFO("enter factory mode success");
					sys_delay(200);
					fts_special_operation_for_samsung(ts_data);
					return 0;

				} else {
					sys_delay(FACTORY_TEST_DELAY);
				}
			}
		}

		sys_delay(50);
	}

	if (i >= ENTER_WORK_FACTORY_RETRIES) {
		FTS_TEST_SAVE_ERR("Enter factory mode fail\n");
		return -EIO;
	}

	fts_special_operation_for_samsung(ts_data);
	return 0;
}

static int get_channel_num(struct chip_data_ft8057p *ts_data)
{
	int ret = 0;
	u8 tx_num = 0;
	u8 rx_num = 0;

	ret = fts_test_read_reg(FACTORY_REG_CHX_NUM, &tx_num);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read tx_num register fail\n");
		return ret;
	}

	ret = fts_test_read_reg(FACTORY_REG_CHY_NUM, &rx_num);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read rx_num register fail\n");
		return ret;
	}

	if ((tx_num != ts_data->hw_res->tx_num)
	    || (rx_num != ts_data->hw_res->rx_num)) {
		FTS_TEST_SAVE_ERR("channel num check fail, tx_num:%d-%d, rx_num:%d-%d\n",
		                  tx_num, ts_data->hw_res->tx_num,
		                  rx_num, ts_data->hw_res->rx_num);
		return -EIO;
	}

	return 0;
}

static int read_rawdata(u8 off_addr, u8 off_val, u8 rawdata_addr, int byte_num,
                        int *data)
{
	int ret = 0;

	/* set line addr or rawdata start addr */
	ret = fts_test_write_reg(off_addr, off_val);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("wirte line/start addr fail\n");
		return ret;
	}

	ret = read_mass_data(rawdata_addr, byte_num, data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read rawdata fail\n");
		return ret;
	}

	return 0;
}

static int start_scan(void)
{
	int ret = 0;
	u8 addr = 0;
	u8 val = 0;
	u8 finish_val = 0;
	int times = 0;

	addr = DEVIDE_MODE_ADDR;
	val = 0xC0;
	finish_val = 0x40;

	/* write register to start scan */
	ret = fts_test_write_reg(addr, val);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write start scan mode fail\n");
		return ret;
	}

	/* Wait for the scan to complete */
	while (times++ < FACTORY_TEST_RETRY) {
		sys_delay(FACTORY_TEST_DELAY);

		ret = fts_test_read_reg(addr, &val);

		if ((ret >= 0) && (val == finish_val)) {
			break;

		} else {
			TPD_INFO("reg%x=%x,retry:%d", addr, val, times);
		}
	}

	if (times >= FACTORY_TEST_RETRY) {
		FTS_TEST_SAVE_ERR("scan timeout\n");
		return -EIO;
	}

	return 0;
}

/*
 * start_scan - start to scan a frame
 */
int ft8057p_start_scan(int frame_num)
{
	int ret = 0;
	u8 addr = 0;
	u8 val = 0;
	u8 finish_val = 0;
	int times = 0;

	addr = DEVIDE_MODE_ADDR;
	val = 0xC0;
	finish_val = 0x40;

	/* write register to start scan */
	ret = fts_test_write_reg(addr, val);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write start scan mode fail\n");
		return ret;
	}

	sys_delay(frame_num * FACTORY_TEST_DELAY / 2);
	/* Wait for the scan to complete */
	while (times++ < 100) {
		sys_delay(FACTORY_TEST_DELAY);

		ret = fts_test_read_reg(addr, &val);
		if ((ret >= 0) && (val == finish_val)) {
			break;
		} else
			TPD_INFO("reg%x=%x,retry:%d", addr, val, times);
	}

	if (times >= 100) {
		FTS_TEST_SAVE_ERR("scan timeout\n");
		return -EIO;
	}

	return 0;
}



/*
 * chip_clb - auto clb
 */
int chip_clb(void)
{
	int ret = 0;
	u8 val = 0;
	int times = 0;

	/* start clb */
	ret = fts_test_write_reg(FACTORY_REG_CLB, 0x04);
	if (ret) {
		FTS_TEST_SAVE_ERR("write start clb fail\n");
		return ret;
	}

	while (times++ < FACTORY_TEST_RETRY) {
		sys_delay(FACTORY_TEST_RETRY_DELAY);
		ret = fts_test_read_reg(FACTORY_REG_CLB, &val);
		if ((0 == ret) && (0x02 == val)) {
			/* clb ok */
			break;
		} else
			TPD_DEBUG("reg%x=%x,retry:%d", FACTORY_REG_CLB, val, times);
	}

	if (times >= FACTORY_TEST_RETRY) {
		FTS_TEST_SAVE_ERR("chip clb timeout\n");
		return -EIO;
	}

	return 0;
}


/*
 * get_cb_incell - get cb data for incell IC
 */
int get_cb_incell(u16 saddr, int byte_num, int *cb_buf)
{
	int ret = 0;
	int i = 0;
	u8 cb_addr = 0;
	u8 addr_h = 0;
	u8 addr_l = 0;
	int read_num = 0;
	int packet_num = 0;
	int packet_remainder = 0;
	int offset = 0;
	int addr = 0;
	u8 *data = NULL;

	data = (u8 *)kzalloc(byte_num * sizeof(u8), GFP_KERNEL);
	if (NULL == data) {
		FTS_TEST_SAVE_ERR("cb buffer malloc fail\n");
		return -ENOMEM;
	}

	packet_num = byte_num / BYTES_PER_TIME;
	packet_remainder = byte_num % BYTES_PER_TIME;
	if (packet_remainder)
		packet_num++;
	read_num = BYTES_PER_TIME;

	TPD_INFO("cb packet:%d,remainder:%d", packet_num, packet_remainder);
	cb_addr = FACTORY_REG_CB_ADDR;
	for (i = 0; i < packet_num; i++) {
		offset = read_num * i;
		addr = saddr + offset;
		addr_h = (addr >> 8) & 0xFF;
		addr_l = addr & 0xFF;
		if ((i == (packet_num - 1)) && packet_remainder) {
			read_num = packet_remainder;
		}

		ret = fts_test_write_reg(FACTORY_REG_CB_ADDR_H, addr_h);
		if (ret) {
			FTS_TEST_SAVE_ERR("write cb addr high fail\n");
			goto TEST_CB_ERR;
		}
		ret = fts_test_write_reg(FACTORY_REG_CB_ADDR_L, addr_l);
		if (ret) {
			FTS_TEST_SAVE_ERR("write cb addr low fail\n");
			goto TEST_CB_ERR;
		}

		ret = fts_test_read(cb_addr, data + offset, read_num);
		if (ret) {
			FTS_TEST_SAVE_ERR("read cb fail\n");
			goto TEST_CB_ERR;
		}
	}

	for (i = 0; i < byte_num; i++) {
		cb_buf[i] = data[i];
	}

TEST_CB_ERR:
	if(data)
		kfree(data);

	return ret;
}


int short_get_adcdata_incell(u8 retval, u8 ch_num, int byte_num, int *adc_buf)
{
	int ret = 0;
	int times = 0;
	u8 short_state = 0;

	FTS_TEST_FUNC_ENTER();

	/* Start ADC sample */
	ret = fts_test_write_reg(FACTORY_REG_SHORT_TEST_EN, 0x01);
	if (ret) {
		FTS_TEST_SAVE_ERR("start short test fail\n");
		goto adc_err;
	}

	sys_delay(ch_num * FACTORY_TEST_DELAY);
	for (times = 0; times < FACTORY_TEST_RETRY; times++) {
		ret = fts_test_read_reg(FACTORY_REG_SHORT_TEST_STATE, &short_state);
		if ((ret >= 0) && (retval == short_state))
			break;
		else
			TPD_DEBUG("reg%x=%x,retry:%d",
						FACTORY_REG_SHORT_TEST_STATE, short_state, times);

		sys_delay(FACTORY_TEST_RETRY_DELAY);
	}
	if (times >= FACTORY_TEST_RETRY) {
		FTS_TEST_SAVE_ERR("short test timeout, ADC data not OK\n");
		ret = -EIO;
		goto adc_err;
	}

	ret = read_mass_data(FACTORY_REG_SHORT_ADDR, byte_num, adc_buf);
	if (ret) {
		FTS_TEST_SAVE_ERR("get short(adc) data fail\n");
	}

adc_err:
	FTS_TEST_FUNC_EXIT();
	return ret;
}

static void ft8057p_autotest_populate_result_head(
	struct chip_data_ft8057p *ts_data, struct auto_testdata *p_testdata)
{
	uint8_t  data_buf[256];
	uint32_t buflen = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int line_num = 0;

	FTS_TEST_FUNC_ENTER();

	/*header*/
	buflen = snprintf(data_buf, 256, "ECC, 85, 170, IC Name, %s, IC Code, %x\n",
	                  "ft8057p", 0);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	buflen = snprintf(data_buf, 256, "TestItem Num, %d, ", 12);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num = 11;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "Short Test", 14,
	                  tx_num, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;

	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "Open Test", 14,
	                  tx_num, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;

	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "CB Test", 14,
	                  tx_num, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "Rawdata Test", 7,
	                  tx_num, rx_num, line_num, 2);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ",
	                  "Rawdata Uniformity Test", 16, tx_num, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += (tx_num * 2);
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "LCD Noise Test", 9,
	                  2, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;
	buflen = snprintf(data_buf, 256, "\n\n\n\n\n\n\n\n\n");
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	FTS_TEST_FUNC_EXIT();
	return;
}

static void ft8057p_black_autotest_populate_result_head(
	struct chip_data_ft8057p *ts_data, struct auto_testdata *p_testdata)
{
	uint8_t  data_buf[256];
	uint32_t buflen = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int line_num = 0;

	FTS_TEST_FUNC_ENTER();

	/*header*/
	buflen = snprintf(data_buf, 256, "ECC, 85, 170, IC Name, %s, IC Code, %x\n",
	                  "ft8057p", 0);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	buflen = snprintf(data_buf, 256, "TestItem Num, %d, ", 3);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num = 11;

	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "CB Test", 14,
	                  tx_num, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "Rawdata Test", 7,
	                  tx_num, rx_num, line_num, 2);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += (tx_num * 2);
	buflen = snprintf(data_buf, 256, "%s, %d, %d, %d, %d, %d, ", "LCD Noise Test", 9,
	                  2, rx_num, line_num, 1);
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	line_num += tx_num;
	buflen = snprintf(data_buf, 256, "\n\n\n\n\n\n\n\n\n");
	tp_test_write(p_testdata->fp, p_testdata->length, data_buf, buflen,
	              p_testdata->pos);

	FTS_TEST_FUNC_EXIT();
	return;
}

#define FTS_TEST_FW_NAME    "tp/23706/FW_NF_FT8057P_BOE.img"
static int fts_enter_test_environment(struct touchpanel_data *ts, bool test_state)
{
	int ret = 0;
	u8 detach_flag = 0;
	uint8_t copy_len = 0;
	const struct firmware *fw = NULL;
	char *fw_name_test = NULL;
	char *p_node = NULL;
	char *postfix = "_TEST.img";
	TPD_INFO("fw test download function");
	if (ts->loading_fw) {
		TPD_INFO("fw is loading, not download again");
		return -EINVAL;
	}

	if (test_state) {
		/*update test firmware*/
		fw_name_test = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
		if (fw_name_test == NULL) {
			TPD_INFO("fw_name_test kzalloc error!\n");
			return -ENOMEM;
		}

		p_node = strstr(ts->panel_data.fw_name, ".");
		copy_len = p_node - ts->panel_data.fw_name;
		memcpy(fw_name_test, ts->panel_data.fw_name, copy_len);
		strlcat(fw_name_test, postfix, MAX_FW_NAME_LENGTH);
		TPD_INFO("fw_name_test is %s\n", fw_name_test);
		/*write test firmware.bin*/
		ret = request_firmware(&fw, fw_name_test, ts->dev);
		if (ret) {
			TPD_INFO("request_firmware(%s) fail", fw_name_test);
			return -ENODATA;
		}
	} else {
		/*memcpy(ts->panel_data.fw_name, FTS_TEST_FW_NAME, MAX_FW_NAME_LENGTH);*/
		strncpy(ts->panel_data.fw_name, FTS_TEST_FW_NAME, MAX_FW_NAME_LENGTH - 1);
		ts->panel_data.fw_name[MAX_FW_NAME_LENGTH - 1] = '\0';
		/*write normal firmware.bin*/
		ret = request_firmware(&fw, ts->panel_data.fw_name, ts->dev);
	}

	/*download firmware*/
	ts->loading_fw = true;
	if (ts->ts_ops && ts->ts_ops->fw_update)
		ret = ts->ts_ops->fw_update(ts->chip_data, fw, 1);
	ts->loading_fw = false;

	msleep(50);
	fts_test_read_reg(0xB4, &detach_flag);
	TPD_INFO("regb4:0x%02x\n", detach_flag);

	if (fw) {
		release_firmware(fw);
		fw = NULL;
	}
	return ret;
}


#define NUM_MODE 2
#define TEST_RESULT_NORMAL     0
#define TEST_RESULT_ABNORMAL  -1
int ft8057p_auto_preoperation(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int node_num = ts_data->hw_res->tx_num * ts_data->hw_res->rx_num;
	int channel_num = ts_data->hw_res->tx_num + ts_data->hw_res->rx_num;
	int ret = 0;

	ret = fts_enter_test_environment(ts_data->ts, 1);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("enter test mode fails\n");
		return TEST_RESULT_ABNORMAL;
	}
	ts_data->node_valid = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);
	if (!ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("kzalloc for node_valid fail\n");
		goto alloc_err;
	}

	ts_data->node_valid_sc = (int *)kzalloc(channel_num * sizeof(int), GFP_KERNEL);
	if (!ts_data->node_valid_sc) {
		FTS_TEST_SAVE_ERR("kzalloc for node_valid_sc fail\n");
		goto alloc_err;
	}

	ts_data->rawdata = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->rawdata) {
		FTS_TEST_SAVE_ERR("kzalloc for rawdata fail\n");
		goto alloc_err;
	}

	ts_data->short_data = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->short_data) {
		FTS_TEST_SAVE_ERR("kzalloc for short_data fail\n");
		goto alloc_err;
	}

	ts_data->cb_data = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->cb_data) {
		FTS_TEST_SAVE_ERR("kzalloc for cb_data fail\n");
		goto alloc_err;
	}

	ts_data->open_data = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->open_data) {
		FTS_TEST_SAVE_ERR("kzalloc for open_data fail\n");
		goto alloc_err;
	}

	ts_data->lcd_noise = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->lcd_noise) {
		FTS_TEST_SAVE_ERR("kzalloc for lcd_noise fail\n");
		goto alloc_err;
	}

	ts_data->rawdata_linearity = kzalloc(node_num * 2 * sizeof(int), GFP_KERNEL);

	if (!ts_data->rawdata_linearity) {
		FTS_TEST_SAVE_ERR("ts_data->rawdata_linearity buffer malloc fail\n");
		goto alloc_err;
	}

	ft8057p_autotest_populate_result_head(ts_data, focal_testdata);
	fts_test_entry(ts_data, focal_testdata);

	return TEST_RESULT_NORMAL;

alloc_err:

	if (ts_data->node_valid) {
		kfree(ts_data->node_valid);
		ts_data->node_valid = NULL;
	}

	if (ts_data->node_valid_sc) {
		kfree(ts_data->node_valid_sc);
		ts_data->node_valid_sc = NULL;
	}

	if (ts_data->rawdata_linearity) {
		kfree(ts_data->rawdata_linearity);
		ts_data->rawdata_linearity = NULL;
	}

	if (ts_data->rawdata) {
		kfree(ts_data->rawdata);
		ts_data->rawdata = NULL;
	}

	if (ts_data->short_data) {
		kfree(ts_data->short_data);
		ts_data->short_data = NULL;
	}


	if (ts_data->cb_data) {
		kfree(ts_data->cb_data);
		ts_data->cb_data = NULL;
	}

	if (ts_data->open_data) {
		kfree(ts_data->open_data);
		ts_data->open_data = NULL;
	}

	if (ts_data->lcd_noise) {
		kfree(ts_data->lcd_noise);
		ts_data->lcd_noise = NULL;
	}

	return TEST_RESULT_ABNORMAL;
}
int ft8057p_black_screen_test_preoperation(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int node_num = ts_data->hw_res->tx_num * ts_data->hw_res->rx_num;
	int channel_num = ts_data->hw_res->tx_num + ts_data->hw_res->rx_num;
	int ret = 0;
	TPD_INFO("ft8057p_black_screen_test_preoperation+");
	ret = fts_enter_test_environment(ts_data->ts, 1);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("enter test mode fails\n");
		return TEST_RESULT_ABNORMAL;
	}

	ts_data->node_valid = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);
	if (!ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("kzalloc for node_valid fail\n");
		goto alloc_err;
	}

	ts_data->node_valid_sc = (int *)kzalloc(channel_num * sizeof(int), GFP_KERNEL);
	if (!ts_data->node_valid_sc) {
		FTS_TEST_SAVE_ERR("kzalloc for node_valid_sc fail\n");
		goto alloc_err;
	}

	ts_data->cb_data = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->cb_data) {
		FTS_TEST_SAVE_ERR("kzalloc for cb_data fail\n");
		goto alloc_err;
	}

	ts_data->rawdata = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->rawdata) {
		FTS_TEST_SAVE_ERR("kzalloc for rawdata fail\n");
		goto alloc_err;
	}

	ts_data->lcd_noise = (int *)kzalloc(node_num * sizeof(int), GFP_KERNEL);

	if (!ts_data->lcd_noise) {
		FTS_TEST_SAVE_ERR("kzalloc for lcd_noise fail\n");
		goto alloc_err;
	}

	ts_data->rawdata_linearity = kzalloc(node_num * 2 * sizeof(int), GFP_KERNEL);

	if (!ts_data->rawdata_linearity) {
		FTS_TEST_SAVE_ERR("ts_data->rawdata_linearity buffer malloc fail\n");
		goto alloc_err;
	}

	ft8057p_black_autotest_populate_result_head(ts_data, focal_testdata);
	fts_test_entry(ts_data, focal_testdata);

	return TEST_RESULT_NORMAL;

alloc_err:

	if (ts_data->node_valid) {
		kfree(ts_data->node_valid);
		ts_data->node_valid = NULL;
	}

	if (ts_data->node_valid_sc) {
		kfree(ts_data->node_valid_sc);
		ts_data->node_valid_sc = NULL;
	}

	if (ts_data->rawdata_linearity) {
		kfree(ts_data->rawdata_linearity);
		ts_data->rawdata_linearity = NULL;
	}

	if (ts_data->rawdata) {
		kfree(ts_data->rawdata);
		ts_data->rawdata = NULL;
	}

	if (ts_data->short_data) {
		kfree(ts_data->short_data);
		ts_data->short_data = NULL;
	}


	if (ts_data->cb_data) {
		kfree(ts_data->cb_data);
		ts_data->cb_data = NULL;
	}

	if (ts_data->open_data) {
		kfree(ts_data->open_data);
		ts_data->open_data = NULL;
	}

	if (ts_data->lcd_noise) {
		kfree(ts_data->lcd_noise);
		ts_data->lcd_noise = NULL;
	}

	return TEST_RESULT_ABNORMAL;
}




int ft8057p_rawdata_autotest(struct seq_file *s, void *chip_data,
                            struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int i = 0;
	u8 rawdata_addr = 0;
	bool result = false;
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int byte_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Rawdata Test\n");

	if (!ts_data->fts_autotest_offset->fts_raw_data_P
	    || !ts_data->fts_autotest_offset->fts_raw_data_N) {
		TPD_INFO("fts_raw_data_P || fts_raw_data_N is NULL");
		return 0;
	}

	if (!ts_data->rawdata || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("rawdata is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		goto test_err;
	}

	/* rawdata test enable */
	ret = fts_test_write_reg(FACTORY_REG_RAWDATA_TEST_EN, 0x01);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("rawdata test enable fail\n");
		goto test_err;
	}

	/*********************GET RAWDATA*********************/
	for (i = 0; i < 3; i++) {
		/* lost 3 frames, in order to obtain stable data */
		/* start scanning */
		ret = start_scan();

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("scan fail\n");
			continue;
		}

		/* read rawdata */
		rawdata_addr = FACTORY_REG_RAWDATA_ADDR;
		byte_num = node_num * 2;
		ret = read_rawdata(FACTORY_REG_LINE_ADDR, 0xAD, rawdata_addr, byte_num,
		                   ts_data->rawdata);

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("read rawdata fail\n");
		}
	}

	if (ret < 0) {
		result = false;
		goto restore_reg;
	}

	ft8057p_output_data(ts_data->rawdata, ts_data, focal_testdata, NODE_MATCH);


	/* compare */
	result = true;

	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}

		if ((ts_data->rawdata[i] < ts_data->fts_autotest_offset->fts_raw_data_N[i])
		    || (ts_data->rawdata[i] > ts_data->fts_autotest_offset->fts_raw_data_P[i])) {
			TPD_INFO("raw data ERR [%d]: [%d] > [%d] > [%d] \n", i,
			         ts_data->fts_autotest_offset->fts_raw_data_P[i], ts_data->rawdata[i],
			         ts_data->fts_autotest_offset->fts_raw_data_N[i]);
			FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
			                  i / rx_num + 1, i % rx_num + 1, ts_data->rawdata[i],
			                  ts_data->fts_autotest_offset->fts_raw_data_N[i],
			                  ts_data->fts_autotest_offset->fts_raw_data_P[i]);
			result = false;
		}
	}

restore_reg:
	/* set the origin value */
	ret = fts_test_write_reg(FACTORY_REG_RAWDATA_TEST_EN, 0x0);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("rawdata test disable fail\n");
		goto test_err;
	}

test_err:

	if (result) {
		FTS_TEST_SAVE_INFO("------ rawdata test PASS\n");
		ret = TEST_RESULT_NORMAL;
	} else {
		FTS_TEST_SAVE_INFO("------ rawdata test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}

int ft8057p_black_rawdata_autotest(struct seq_file *s, void *chip_data,
                            struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int i = 0;
	u8 rawdata_addr = 0;
	bool result = false;
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int byte_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Rawdata Test\n");

	if (!ts_data->fts_autotest_offset->fts_raw_data_P
	    || !ts_data->fts_autotest_offset->fts_raw_data_N) {
		TPD_INFO("fts_raw_data_P || fts_raw_data_N is NULL");
		return 0;
	}

	if (!ts_data->rawdata || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("rawdata is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		goto test_err;
	}

	/* rawdata test enable */
	ret = fts_test_write_reg(FACTORY_REG_RAWDATA_TEST_EN, 0x01);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("rawdata test enable fail\n");
		goto test_err;
	}

	/*********************GET RAWDATA*********************/
	for (i = 0; i < 3; i++) {
		/* lost 3 frames, in order to obtain stable data */
		/* start scanning */
		ret = start_scan();

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("scan fail\n");
			continue;
		}

		/* read rawdata */
		rawdata_addr = FACTORY_REG_RAWDATA_ADDR;
		byte_num = node_num * 2;
		ret = read_rawdata(FACTORY_REG_LINE_ADDR, 0xAD, rawdata_addr, byte_num,
		                   ts_data->rawdata);

		if (ret < 0) {
			FTS_TEST_SAVE_ERR("read rawdata fail\n");
		}
	}

	if (ret < 0) {
		result = false;
		goto restore_reg;
	}

	ft8057p_black_output_data(ts_data->rawdata, ts_data, focal_testdata, NODE_MATCH);


	/* compare */
	result = true;

	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}

		if ((ts_data->rawdata[i] < ts_data->fts_autotest_offset->fts_raw_data_N[i])
		    || (ts_data->rawdata[i] > ts_data->fts_autotest_offset->fts_raw_data_P[i])) {
			TPD_INFO("raw data ERR [%d]: [%d] > [%d] > [%d] \n", i,
			         ts_data->fts_autotest_offset->fts_raw_data_P[i], ts_data->rawdata[i],
			         ts_data->fts_autotest_offset->fts_raw_data_N[i]);
			FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
			                  i / rx_num + 1, i % rx_num + 1, ts_data->rawdata[i],
			                  ts_data->fts_autotest_offset->fts_raw_data_N[i],
			                  ts_data->fts_autotest_offset->fts_raw_data_P[i]);
			result = false;
		}
	}

restore_reg:
	/* set the origin value */
	ret = fts_test_write_reg(FACTORY_REG_RAWDATA_TEST_EN, 0x0);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("rawdata test disable fail\n");
		goto test_err;
	}

test_err:

	if (result) {
		FTS_TEST_SAVE_INFO("------ rawdata test PASS\n");
		ret = TEST_RESULT_NORMAL;
	} else {
		FTS_TEST_SAVE_INFO("------ rawdata test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}


int ft8057p_uniformity_autotest(struct seq_file *s, void *chip_data,
                               struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int row = 0;
	int col = 1;
	int i = 0;
	int deviation = 0;
	int max = 0;
	int *rl_tmp = NULL;
	int offset = 0;
	int offset2 = 0;
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	bool result = false;
	bool result2 = false;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Rawdata Unfiormity Test\n");

	if (!ts_data->fts_autotest_offset->fts_uniformity_data_P
	    || !ts_data->fts_autotest_offset->fts_uniformity_data_N) {
		TPD_INFO("fts_uniformity_data_P || fts_uniformity_data_N is NULL");
		return 0;
	}

	if (!ts_data->rawdata || !ts_data->rawdata_linearity || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("rawdata/rawdata_linearity is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	result = true;
	/*    FTS_TEST_SAVE_INFO("Check Tx Linearity\n");*/
	ts_data->rl_cnt = 0;
	rl_tmp = ts_data->rawdata_linearity + ts_data->rl_cnt;

	for (row = 0; row < tx_num; row++) {
		for (col = 0; col < rx_num - 1; col++) {
			offset = row * rx_num + col;
			offset2 = row * rx_num + col + 1;
			deviation = abs(ts_data->rawdata[offset] - ts_data->rawdata[offset2]);
			max = max(ts_data->rawdata[offset], ts_data->rawdata[offset2]);
			max = max ? max : 1;
			rl_tmp[offset] = 100 * deviation / max;
		}
	}

	ft8057p_output_data(rl_tmp, ts_data, focal_testdata, NODE_MATCH);

	/* compare */
	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}

		if ((rl_tmp[i] < ts_data->fts_autotest_offset->fts_uniformity_data_N[i])
		    || (rl_tmp[i] > ts_data->fts_autotest_offset->fts_uniformity_data_P[i])) {
			TPD_INFO("uniformity data ERR [%d]: [%d] > [%d] > [%d] \n", i,
			         ts_data->fts_autotest_offset->fts_uniformity_data_P[i], rl_tmp[i],
			         ts_data->fts_autotest_offset->fts_uniformity_data_N[i]);
			FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
			                  i / rx_num + 1, i % rx_num + 1, rl_tmp[i],
			                  ts_data->fts_autotest_offset->fts_uniformity_data_N[i],
			                  ts_data->fts_autotest_offset->fts_uniformity_data_P[i]);
			result = false;
		}
	}

	ts_data->rl_cnt += node_num;

	result2 = true;
	/*    FTS_TEST_SAVE_INFO("Check Rx Linearity\n");*/
	rl_tmp = ts_data->rawdata_linearity + ts_data->rl_cnt;

	for (row = 0; row < tx_num - 1; row++) {
		for (col = 0; col < rx_num; col++) {
			offset = row * rx_num + col;
			offset2 = (row + 1) * rx_num + col;
			deviation = abs(ts_data->rawdata[offset] - ts_data->rawdata[offset2]);
			max = max(ts_data->rawdata[offset], ts_data->rawdata[offset2]);
			max = max ? max : 1;
			rl_tmp[offset] = 100 * deviation / max;
		}
	}

	ft8057p_output_data(rl_tmp, ts_data, focal_testdata, NODE_MATCH);


	/* compare */
	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}

		if ((rl_tmp[i] < ts_data->fts_autotest_offset->fts_uniformity_data_N[i])
		    || (rl_tmp[i] > ts_data->fts_autotest_offset->fts_uniformity_data_P[i])) {
			TPD_INFO("uniformity data ERR [%d]: [%d] > [%d] > [%d] \n", i,
			         ts_data->fts_autotest_offset->fts_uniformity_data_P[i], rl_tmp[i],
			         ts_data->fts_autotest_offset->fts_uniformity_data_N[i]);
			FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
			                  i / rx_num + 1, i % rx_num + 1, rl_tmp[i],
			                  ts_data->fts_autotest_offset->fts_uniformity_data_N[i],
			                  ts_data->fts_autotest_offset->fts_uniformity_data_P[i]);
			result2 = false;
		}
	}

	ts_data->rl_cnt += node_num;

test_err:
	if (result && result2) {
		FTS_TEST_SAVE_INFO("------Rawdata Uniformity Test PASS\n");
		ret = TEST_RESULT_NORMAL;

	} else {
		FTS_TEST_SAVE_ERR("------Rawdata Uniformity Test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}


int ft8057p_cb_test(struct seq_file *s, void *chip_data,
						   struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int i = 0;
	bool result = false;
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int byte_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	int *cbdata = NULL;

	cbdata = ts_data->cb_data;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: cb Test\n");

	if (!ts_data->fts_autotest_offset->fts_cb_data_P
		|| !ts_data->fts_autotest_offset->fts_cb_data_N) {
		TPD_INFO("fts_cb_data_P || fts_cb_data_N is NULL");
		return 0;
	}

	if (!ts_data->cb_data || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("cb_data is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		goto test_err;
	}

	/* cb test enable */
	ret = fts_test_write_reg(FACTORY_REG_CB_TEST_EN, 0x01);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("cb test enable fail\n");
		goto test_err;
	}

	/* auto clb */
	ret = chip_clb();
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("auto clb fail\n");
		goto test_err;
	}

	byte_num = node_num;
	ret = get_cb_incell(0, byte_num, cbdata);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("get cb fail\n");
		goto test_err;
	}

	ft8057p_output_data(cbdata, ts_data, focal_testdata, NODE_MATCH);


	/* compare */
	result = true;

	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
		continue;
	}

	if ((ts_data->cb_data[i] < ts_data->fts_autotest_offset->fts_cb_data_N[i])
		|| (ts_data->cb_data[i] > ts_data->fts_autotest_offset->fts_cb_data_P[i])) {
		TPD_INFO("cb data ERR [%d]: [%d] > [%d] > [%d] \n", i,
					ts_data->fts_autotest_offset->fts_cb_data_P[i], ts_data->cb_data[i],
					ts_data->fts_autotest_offset->fts_cb_data_N[i]);
		FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
							i / rx_num + 1, i % rx_num + 1, ts_data->cb_data[i],
							ts_data->fts_autotest_offset->fts_cb_data_N[i],
							ts_data->fts_autotest_offset->fts_cb_data_P[i]);
		result = false;
	}
	}

test_err:
	/* set the origin value */
	ret = fts_test_write_reg(FACTORY_REG_CB_TEST_EN, 0x0);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("cb test disable fail\n");
	}

	if (result) {
		FTS_TEST_SAVE_INFO("------ cb test PASS\n");
		ret = TEST_RESULT_NORMAL;
	} else {
		FTS_TEST_SAVE_INFO("------ cb test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}

int ft8057p_black_cb_test(struct seq_file *s, void *chip_data,
						   struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int i = 0;
	bool result = false;
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int byte_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	int *cbdata = NULL;

	cbdata = ts_data->cb_data;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: cb Test\n");

	if (!ts_data->fts_autotest_offset->fts_cb_data_P
		|| !ts_data->fts_autotest_offset->fts_cb_data_N) {
		TPD_INFO("fts_cb_data_P || fts_cb_data_N is NULL");
		return 0;
	}

	if (!ts_data->cb_data || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("cb_data is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		goto test_err;
	}

	/* cb test enable */
	ret = fts_test_write_reg(FACTORY_REG_CB_TEST_EN, 0x01);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("cb test enable fail\n");
		goto test_err;
	}

	/* auto clb */
	ret = chip_clb();
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("auto clb fail\n");
		goto test_err;
	}

	byte_num = node_num;
	ret = get_cb_incell(0, byte_num, cbdata);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("get cb fail\n");
		goto test_err;
	}

	ft8057p_black_output_data(cbdata, ts_data, focal_testdata, NODE_MATCH);


	/* compare */
	result = true;

	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
		continue;
	}

	if ((ts_data->cb_data[i] < ts_data->fts_autotest_offset->fts_cb_data_N[i])
		|| (ts_data->cb_data[i] > ts_data->fts_autotest_offset->fts_cb_data_P[i])) {
		TPD_INFO("cb data ERR [%d]: [%d] > [%d] > [%d] \n", i,
					ts_data->fts_autotest_offset->fts_cb_data_P[i], ts_data->cb_data[i],
					ts_data->fts_autotest_offset->fts_cb_data_N[i]);
		FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
							i / rx_num + 1, i % rx_num + 1, ts_data->cb_data[i],
							ts_data->fts_autotest_offset->fts_cb_data_N[i],
							ts_data->fts_autotest_offset->fts_cb_data_P[i]);
		result = false;
	}
	}

test_err:
	/* set the origin value */
	ret = fts_test_write_reg(FACTORY_REG_CB_TEST_EN, 0x0);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("cb test disable fail\n");
	}

	if (result) {
		FTS_TEST_SAVE_INFO("------ cb test PASS\n");
		ret = TEST_RESULT_NORMAL;
	} else {
		FTS_TEST_SAVE_INFO("------ cb test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}


int ft8057p_open_test(struct seq_file *s, void *chip_data,
						  struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int i = 0;
	u8 state = 0;
	bool result = false;
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int byte_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	int *opendata = NULL;

	opendata = ts_data->open_data;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: open Test\n");

	if (!ts_data->fts_autotest_offset->fts_open_data_P
		|| !ts_data->fts_autotest_offset->fts_open_data_N) {
		TPD_INFO("fts_open_data_P || fts_open_data_N is NULL");
	return 0;
	}

	if (!ts_data->open_data || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("open_data is null\n");
		ret = -EINVAL;
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		goto test_err;
	}

	ret = fts_test_write_reg(FACTORY_REG_OPEN_START, 0x01);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("start open test fail\n");
		goto test_err;
	}

	/* check test status */
	for (i = 0; i < FACTORY_TEST_RETRY; i++) {
		sys_delay(FACTORY_TEST_RETRY_DELAY);
		ret = fts_test_read_reg(FACTORY_REG_OPEN_STATE, &state);
		if ((ret >= 0) && (TEST_RETVAL_AA == state)) {
			break;
		} else {
			TPD_DEBUG("reg%x=%x,retry:%d\n",
						FACTORY_REG_OPEN_STATE, state, i);
		}
	}
	if (i >= FACTORY_TEST_RETRY) {
		FTS_TEST_SAVE_ERR("open test timeout\n");
		goto test_err;
	}

	/* get open data */
	byte_num = node_num * 2;
	ret = read_mass_data(FACTORY_REG_OPEN_ADDR, byte_num, opendata);
	if (ret) {
		FTS_TEST_SAVE_ERR("get open data fail\n");
	}

	ft8057p_output_data(opendata, ts_data, focal_testdata, NODE_MATCH);


	/* compare */
	result = true;

	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}

	if ((ts_data->open_data[i] < ts_data->fts_autotest_offset->fts_open_data_N[i])
		|| (ts_data->open_data[i] > ts_data->fts_autotest_offset->fts_open_data_P[i])) {
		TPD_INFO("open data ERR [%d]: [%d] > [%d] > [%d] \n", i,
				ts_data->fts_autotest_offset->fts_open_data_P[i], ts_data->open_data[i],
				ts_data->fts_autotest_offset->fts_open_data_N[i]);
		FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
							i / rx_num + 1, i % rx_num + 1, ts_data->open_data[i],
							ts_data->fts_autotest_offset->fts_open_data_N[i],
							ts_data->fts_autotest_offset->fts_open_data_P[i]);
		result = false;
	}
	}


test_err:
	/* set the origin value */
	ret = fts_test_write_reg(FACTORY_REG_OPEN_STATE, 0x0);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("open test disable fail\n");
	}

	if (result) {
		FTS_TEST_SAVE_INFO("------ open test PASS\n");
		ret = TEST_RESULT_NORMAL;
	} else {
		FTS_TEST_SAVE_INFO("------ open test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}


int ft8057p_short_test(struct seq_file *s, void *chip_data,
                      struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	bool ca_result = false;
	int byte_num = 0;
	int ch_num = 0;
	int i = 0;
	int tmp_adc = 0;
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	int *adcdata = NULL;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: Short Test\n");
	adcdata = ts_data->short_data;
	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
		goto test_err;
	}

	/* get short resistance and exceptional channel */
	byte_num = node_num * 2;
	ch_num = ts_data->hw_res->rx_num;
	ret = short_get_adcdata_incell(TEST_RETVAL_AA, ch_num, byte_num, adcdata);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("get adc data fail\n");
		goto test_err;
	}

	/* calculate resistor */
	for (i = 0; i < node_num; i++) {
		tmp_adc = adcdata[i];
		if (tmp_adc <= 0) {
			adcdata[i] = -1;
			continue;
		}

		/*adcdata[i] = (124 * tmp_adc + 59059) / (8573 - 2 * tmp_adc);*/

		adcdata[i] = (61 * tmp_adc+24562) / (4096 - tmp_adc);
		if((adcdata[i] >= 3000) && (tmp_adc <= 4095))
			adcdata[i] = 3000;
	}

	ft8057p_output_data(ts_data->short_data, ts_data, focal_testdata, NODE_MATCH);

		/* compare */
		ca_result = true;

		for (i = 0; i < node_num; i++) {
			if (0 == ts_data->node_valid[i]) {
				continue;
			}

			if ((ts_data->short_data[i] < ts_data->fts_autotest_offset->fts_short_data_N[i])
				|| (ts_data->short_data[i] > ts_data->fts_autotest_offset->fts_short_data_P[i])) {
				TPD_INFO("raw data ERR [%d]: [%d] > [%d] > [%d] \n", i,
						 ts_data->fts_autotest_offset->fts_short_data_P[i], ts_data->short_data[i],
						 ts_data->fts_autotest_offset->fts_short_data_N[i]);
				FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
								  i / rx_num + 1, i % rx_num + 1, ts_data->short_data[i],
								  ts_data->fts_autotest_offset->fts_short_data_N[i],
								  ts_data->fts_autotest_offset->fts_short_data_P[i]);
				ca_result = false;
			}
		}

test_err:
	if (ca_result) {
		FTS_TEST_SAVE_INFO("------Short test PASS\n");
		ret = TEST_RESULT_NORMAL;

	} else {
		FTS_TEST_SAVE_ERR("------Short Test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}


static int ft8057_fast_calibration(void)
{
	int ret = 0;
	u8 val = 0;
	int i = 0;
	u8 fts_fast_cal_state = 0xBD;
	u8 fts_fast_cal_start = 0xBC;

	ret = enter_work_mode();
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("enter work mode fail,ret=%d\n", ret);
		return ret;
	}

	ret = fts_test_read_reg(fts_fast_cal_state, &val);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read regBD fail\n");
		return ret;
	}

	if (val == 0xAA) {
		TPD_INFO("already have been fast calibration");
		return 0;
	}

	ret = fts_test_write_reg(fts_fast_cal_start, 0x01);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write regBC fail\n");
		return ret;
	}

	sys_delay(1000);

	for (i = 0; i < 100; i++) {
		ret = fts_test_read_reg(fts_fast_cal_state, &val);
		if (ret < 0) {
			FTS_TEST_SAVE_ERR("read regBD fail\n");
			return ret;
		}

		if (val == 0xAA) {
			TPD_INFO("finish fast calibration");
			return 0;
		}
		sys_delay(50);
	}

	if (i >= 100) {
		FTS_TEST_SAVE_ERR("waite for fast calibration time out!!\n");
		return -EINVAL;
	}
	return 0;
}


int ft8057p_lcd_noise_test(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int i = 0;
	u8 old_mode = 0;
	u8 status = 0;
	bool result = false;
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int byte_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	int *lcdnoise = NULL;
	int frame_num = 0;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: lcd_noise Test\n");

	if (!ts_data->fts_autotest_offset->fts_lcd_noise_P
	    || !ts_data->fts_autotest_offset->fts_lcd_noise_N) {
		TPD_INFO("fts_lcd_noise_P || fts_lcd_noise_N is NULL");
		return 0;
	}

	if (!ts_data->lcd_noise || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("lcd_noise is null\n");
		ret = -EINVAL;
		goto test_err;
	}
	lcdnoise = ts_data->lcd_noise;
	ret = ft8057_fast_calibration();
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("fast calibration fail,ret=%d\n", ret);
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		goto test_err;
	}

	ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &old_mode);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read reg06 fail\n");
		goto test_err;
	}

	ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x01);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write 1 to reg06 fail\n");
		goto test_err;
	}

	ret =  fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xAD);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write reg01 fail\n");
		goto test_err;
	}

	frame_num = 200;
	ret = fts_test_write_reg(FACTORY_REG_LCD_NOISE_FRAME, frame_num / 4);
	if (ret < 0) {
		FTS_TEST_SAVE_INFO("write frame num fail\n");
		goto test_err;
	}

	/* start test */
	ret = fts_test_write_reg(FACTORY_REG_LCD_NOISE_START, 0x01);
	if (ret < 0) {
		FTS_TEST_SAVE_INFO("start lcdnoise test fail\n");
		goto test_err;
	}

	/* check test status */
	sys_delay(frame_num * FACTORY_TEST_DELAY / 2);
	for (i = 0; i < FACTORY_TEST_RETRY; i++) {
		status = 0xFF;
		ret = fts_test_read_reg(FACTORY_REG_LCD_NOISE_TEST_STATE, &status);
		if ((ret >= 0) && (TEST_RETVAL_AA == status)) {
			break;
		} else {
			TPD_DEBUG("reg%x=%x,retry:%d\n",
						FACTORY_REG_LCD_NOISE_TEST_STATE, status, i);
		}
		sys_delay(FACTORY_TEST_RETRY_DELAY);
	}
	if (i >= FACTORY_TEST_RETRY) {
		FTS_TEST_SAVE_ERR("lcdnoise test timeout\n");
		goto test_err;
	}
	/* read lcdnoise */
	byte_num = node_num * 2;
	ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR, byte_num, lcdnoise);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read lcd noise fail\n");
		goto test_err;
	}
	ft8057p_output_data(ts_data->lcd_noise, ts_data, focal_testdata, NODE_MATCH);

	/* compare */
	result = true;

	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}

		if ((ts_data->lcd_noise[i] < ts_data->fts_autotest_offset->fts_lcd_noise_N[i])
			|| (ts_data->lcd_noise[i] > ts_data->fts_autotest_offset->fts_lcd_noise_P[i])) {
			TPD_INFO("raw data ERR [%d]: [%d] > [%d] > [%d] \n", i,
					ts_data->fts_autotest_offset->fts_lcd_noise_P[i], ts_data->lcd_noise[i],
					ts_data->fts_autotest_offset->fts_lcd_noise_N[i]);
			FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
							i / rx_num + 1, i % rx_num + 1, ts_data->lcd_noise[i],
							ts_data->fts_autotest_offset->fts_lcd_noise_N[i],
							ts_data->fts_autotest_offset->fts_lcd_noise_P[i]);
			result = false;
		}
	}

test_err:
	ret = fts_test_write_reg(FACTORY_REG_LCD_NOISE_START, 0x00);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write 0 to reg11 fail\n");
	}

	ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, old_mode);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("restore reg06 fail\n");
	}

	ret = fts_test_write_reg(FACTORY_REG_LCD_NOISE_TEST_STATE, 0x03);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write idle to lcdnoise test state fail\n");
	}

	if (result) {
		FTS_TEST_SAVE_INFO("------lcd noise test PASS\n");
		ret = TEST_RESULT_NORMAL;
	} else {
		FTS_TEST_SAVE_INFO("------lcd noise test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}

int ft8057p_black_lcd_noise_test(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	int ret = 0;
	int i = 0;
	u8 old_mode = 0;
	u8 status = 0;
	bool result = false;
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;
	int byte_num = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	int *lcdnoise = NULL;
	int frame_num = 0;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_INFO("\n============ Test Item: lcd_noise Test\n");

	if (!ts_data->fts_autotest_offset->fts_lcd_noise_P
	    || !ts_data->fts_autotest_offset->fts_lcd_noise_N) {
		TPD_INFO("fts_lcd_noise_P || fts_lcd_noise_N is NULL");
		return 0;
	}

	if (!ts_data->lcd_noise || !ts_data->node_valid) {
		FTS_TEST_SAVE_ERR("lcd_noise is null\n");
		ret = -EINVAL;
		goto test_err;
	}
	lcdnoise = ts_data->lcd_noise;
	ret = ft8057_fast_calibration();
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("fast calibration fail,ret=%d\n", ret);
		goto test_err;
	}

	ret = enter_factory_mode(ts_data);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		goto test_err;
	}

	ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &old_mode);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read reg06 fail\n");
		goto test_err;
	}

	ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x01);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write 1 to reg06 fail\n");
		goto test_err;
	}

	ret =  fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xAD);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write reg01 fail\n");
		goto test_err;
	}

	frame_num = 200;
	ret = fts_test_write_reg(FACTORY_REG_LCD_NOISE_FRAME, frame_num / 4);
	if (ret < 0) {
		FTS_TEST_SAVE_INFO("write frame num fail\n");
		goto test_err;
	}

	/* start test */
	ret = fts_test_write_reg(FACTORY_REG_LCD_NOISE_START, 0x01);
	if (ret < 0) {
		FTS_TEST_SAVE_INFO("start lcdnoise test fail\n");
		goto test_err;
	}

	/* check test status */
	sys_delay(frame_num * FACTORY_TEST_DELAY / 2);
	for (i = 0; i < FACTORY_TEST_RETRY; i++) {
		status = 0xFF;
		ret = fts_test_read_reg(FACTORY_REG_LCD_NOISE_TEST_STATE, &status);
		if ((ret >= 0) && (TEST_RETVAL_AA == status)) {
			break;
		} else {
			TPD_DEBUG("reg%x=%x,retry:%d\n",
						FACTORY_REG_LCD_NOISE_TEST_STATE, status, i);
		}
		sys_delay(FACTORY_TEST_RETRY_DELAY);
	}
	if (i >= FACTORY_TEST_RETRY) {
		FTS_TEST_SAVE_ERR("lcdnoise test timeout\n");
		goto test_err;
	}
	/* read lcdnoise */
	byte_num = node_num * 2;
	ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR, byte_num, lcdnoise);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read lcd noise fail\n");
		goto test_err;
	}
	ft8057p_black_output_data(ts_data->lcd_noise, ts_data, focal_testdata, NODE_MATCH);

	/* compare */
	result = true;

	for (i = 0; i < node_num; i++) {
		if (0 == ts_data->node_valid[i]) {
			continue;
		}

		if ((ts_data->lcd_noise[i] < ts_data->fts_autotest_offset->fts_lcd_noise_N[i])
			|| (ts_data->lcd_noise[i] > ts_data->fts_autotest_offset->fts_lcd_noise_P[i])) {
			TPD_INFO("raw data ERR [%d]: [%d] > [%d] > [%d] \n", i,
					ts_data->fts_autotest_offset->fts_lcd_noise_P[i], ts_data->lcd_noise[i],
					ts_data->fts_autotest_offset->fts_lcd_noise_N[i]);
			FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
							i / rx_num + 1, i % rx_num + 1, ts_data->lcd_noise[i],
							ts_data->fts_autotest_offset->fts_lcd_noise_N[i],
							ts_data->fts_autotest_offset->fts_lcd_noise_P[i]);
			result = false;
		}
	}

test_err:
	ret = fts_test_write_reg(FACTORY_REG_LCD_NOISE_START, 0x00);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write 0 to reg11 fail\n");
	}

	ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, old_mode);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("restore reg06 fail\n");
	}

	ret = fts_test_write_reg(FACTORY_REG_LCD_NOISE_TEST_STATE, 0x03);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("write idle to lcdnoise test state fail\n");
	}

	if (result) {
		FTS_TEST_SAVE_INFO("------lcd noise test PASS\n");
		ret = TEST_RESULT_NORMAL;
	} else {
		FTS_TEST_SAVE_INFO("------lcd noise test NG\n");
		ret = TEST_RESULT_ABNORMAL;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;
}

int ft8057p_auto_endoperation(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info)
{
	struct chip_data_ft8057p *ts_data = (struct chip_data_ft8057p *)chip_data;

	TPD_INFO("%s +\n", __func__);

	if (ts_data->fts_autotest_offset) {
		kfree(ts_data->fts_autotest_offset);
		ts_data->fts_autotest_offset = NULL;
	}

	if (ts_data->node_valid) {
		kfree(ts_data->node_valid);
		ts_data->node_valid = NULL;
	}

	if (ts_data->node_valid_sc) {
		kfree(ts_data->node_valid_sc);
		ts_data->node_valid_sc = NULL;
	}

	if (ts_data->rawdata_linearity) {
		kfree(ts_data->rawdata_linearity);
		ts_data->rawdata_linearity = NULL;
	}

	if (ts_data->rawdata) {
		kfree(ts_data->rawdata);
		ts_data->rawdata = NULL;
	}

	if (ts_data->short_data) {
		kfree(ts_data->short_data);
		ts_data->short_data = NULL;
	}

	if (ts_data->cb_data) {
		kfree(ts_data->cb_data);
		ts_data->cb_data = NULL;
	}

	fts_enter_test_environment(ts_data->ts, 0);

	enter_work_mode();

	TPD_INFO("%s -\n", __func__);
	return 0;
}

#ifdef FTS_KIT
static int fts_get_threshold(struct chip_data_ft8057p *ts_data, struct auto_testdata *p_focal_testdata)
{
	int i = 0;
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;
	const struct firmware *limit_fw = NULL;
	int offset = 32;

	limit_fw = (const struct firmware *) p_focal_testdata->fw;

	if (!limit_fw || !limit_fw->data || !limit_fw->size) {
		TPD_INFO("limit_fw is null");
		return -ENOMEM;
	}

	ts_data->fts_autotest_offset = kzalloc(sizeof(struct fts_autotest_offset), GFP_KERNEL);
	if (!ts_data->fts_autotest_offset) {
		TPD_INFO("fts_autotest_offset is null");
		return -ENOMEM;
	}

	ts_data->fts_autotest_offset->fts_short_data_P = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_short_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_open_data_P = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_open_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_cb_data_P = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_cb_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_raw_data_P = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_raw_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_uniformity_data_P = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_uniformity_data_N  = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);

	ts_data->fts_autotest_offset->fts_lcd_noise_P = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);
	ts_data->fts_autotest_offset->fts_lcd_noise_N  = (int32_t *)(limit_fw->data + offset);
	offset += node_num * sizeof(int32_t);

	for (i = 0; i < node_num; i++) {
		ts_data->fts_autotest_offset->fts_lcd_noise_P[i] = 300;
		ts_data->fts_autotest_offset->fts_lcd_noise_N[i] = 0;

		ts_data->fts_autotest_offset->fts_raw_data_P[i] = 40000;
		ts_data->fts_autotest_offset->fts_raw_data_N[i] = 0;

		ts_data->fts_autotest_offset->fts_cb_data_P[i] = 40000;
		ts_data->fts_autotest_offset->fts_cb_data_N[i] = 0;

		ts_data->fts_autotest_offset->fts_uniformity_data_P[i] = 100;
		ts_data->fts_autotest_offset->fts_uniformity_data_N[i] = 0;

		ts_data->fts_autotest_offset->fts_open_data_P[i] = 40000;
		ts_data->fts_autotest_offset->fts_open_data_N[i] = 0;

		ts_data->fts_autotest_offset->fts_short_data_P[i] = 40000;
		ts_data->fts_autotest_offset->fts_short_data_N[i] = 0;
	}

	return 0;
}
#else
static int fts_get_threshold_from_img(struct chip_data_ft8057p *ts_data, struct auto_testdata *p_focal_testdata)
{
	int ret = 0;
	int i = 0;
	int item_cnt = 0;
	/*uint8_t * p_print = NULL;*/
	uint32_t *p_item_offset = NULL;
	struct auto_test_header *ph = NULL;
	struct auto_test_item_header *item_head = NULL;
	const struct firmware *limit_fw = NULL;
	u8 fwver = 0xFF;

	ret = fts_read_reg(FTS_REG_FW_VER, &fwver);

	if (fwver >= 0x20) {
		ts_data->use_panelfactory_limit = false;

	} else {
		ts_data->use_panelfactory_limit = true;
	}

	TPD_INFO("%s, use_panelfactory_limit = %d \n", __func__,
	         ts_data->use_panelfactory_limit);

	ts_data->fts_autotest_offset = kzalloc(sizeof(struct fts_autotest_offset),
	                                       GFP_KERNEL);
	limit_fw = (const struct firmware *) p_focal_testdata->fw;
	ph = (struct auto_test_header *)(limit_fw->data);
	p_item_offset = (uint32_t *)(limit_fw->data + 16);

	for (i = 0; i < 8 * sizeof(ph->test_item); i++) {
		if ((ph->test_item >> i) & 0x01) {
			item_cnt++;
		}
	}

	TPD_INFO("%s: total test item = %d \n", __func__, item_cnt);
	TPD_INFO("%s: populating fts_test_offset \n", __func__);
	for (i = 0; i < item_cnt; i++) {
		TPD_INFO("%s: i[%d] \n", __func__, i);
		item_head = (struct auto_test_item_header *)(limit_fw->data + p_item_offset[i]);

		if (item_head->item_limit_type == LIMIT_TYPE_NO_DATA) {
			TPD_INFO("[%d] incorrect item type: LIMIT_TYPE_NO_DATA\n", item_head->item_bit);
		} else if (item_head->item_limit_type == LIMIT_TYPE_TOP_FLOOR_DATA) {
				TPD_INFO("test item bit [%d] \n", item_head->item_bit);
				if (ts_data->black_screen_test) {
					if (item_head->item_bit == TYPE_BLACK_NOISE_DATA) {
						ts_data->fts_autotest_offset->fts_lcd_noise_P = (int32_t *)(limit_fw->data + item_head->top_limit_offset);
						ts_data->fts_autotest_offset->fts_lcd_noise_N = (int32_t *)(limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("Black_fts_lcd_noise_P = %p, fts_lcd_noise_N = %p \n",
					         ts_data->fts_autotest_offset->fts_lcd_noise_P,
					         ts_data->fts_autotest_offset->fts_lcd_noise_N);
					} else if (item_head->item_bit == TYPE_BLACK_RAW_DATA) {
						ts_data->fts_autotest_offset->fts_raw_data_P = (int32_t *)(limit_fw->data + item_head->top_limit_offset);
						ts_data->fts_autotest_offset->fts_raw_data_N = (int32_t *)(limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("Black_fts_raw_data_P = %p, fts_raw_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_raw_data_P,
					         ts_data->fts_autotest_offset->fts_raw_data_N);
					} else if (item_head->item_bit == TYPE_BLACK_CB_DATA) {
						ts_data->fts_autotest_offset->fts_cb_data_P = (int32_t *)(limit_fw->data + item_head->top_limit_offset);
						ts_data->fts_autotest_offset->fts_cb_data_N = (int32_t *)(limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("Black_fts_cb_data_P = %p, fts_cb_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_cb_data_P,
					         ts_data->fts_autotest_offset->fts_cb_data_N);
					}
				} else {
				if (item_head->item_bit == TYPE_NOISE_DATA) {
					ts_data->fts_autotest_offset->fts_lcd_noise_P = (int32_t *)(limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_lcd_noise_N = (int32_t *)(limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_lcd_noise_P = %p, fts_lcd_noise_N = %p \n",
					         ts_data->fts_autotest_offset->fts_lcd_noise_P,
					         ts_data->fts_autotest_offset->fts_lcd_noise_N);
				} else if (item_head->item_bit == TYPE_RAW_DATA) {
					ts_data->fts_autotest_offset->fts_raw_data_P = (int32_t *)(limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_raw_data_N = (int32_t *)(limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_raw_data_P = %p, fts_raw_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_raw_data_P,
					         ts_data->fts_autotest_offset->fts_raw_data_N);
				} else if (item_head->item_bit == TYPE_CB_DATA) {
					ts_data->fts_autotest_offset->fts_cb_data_P = (int32_t *)(limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_cb_data_N = (int32_t *)(limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_cb_data_P = %p, fts_cb_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_cb_data_P,
					         ts_data->fts_autotest_offset->fts_cb_data_N);
					}
				}

				if (item_head->item_bit == TYPE_UNIFORMITY_DATA) {
					ts_data->fts_autotest_offset->fts_uniformity_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_uniformity_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_uniformity_data_P = %p, fts_uniformity_data_P = %p \n",
					         ts_data->fts_autotest_offset->fts_uniformity_data_P,
					         ts_data->fts_autotest_offset->fts_uniformity_data_N);
				} else if (item_head->item_bit == TYPE_OPEN_DATA) {
					ts_data->fts_autotest_offset->fts_open_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_open_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_open_data_P = %p, fts_open_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_open_data_P,
					         ts_data->fts_autotest_offset->fts_open_data_N);
				} else if (item_head->item_bit == TYPE_SHORT_DATA) {
					ts_data->fts_autotest_offset->fts_short_data_P = (int32_t *)(
					            limit_fw->data + item_head->top_limit_offset);
					ts_data->fts_autotest_offset->fts_short_data_N = (int32_t *)(
					            limit_fw->data + item_head->floor_limit_offset);
					TPD_INFO("fts_short_data_P = %p, fts_short_data_N = %p \n",
					         ts_data->fts_autotest_offset->fts_short_data_P,
					         ts_data->fts_autotest_offset->fts_short_data_N);
				}
		} else {
			TPD_INFO("[%d] unknown item type \n", item_head->item_bit);
		}
	}

	ret = 0;
	return ret;
}
#endif

static void fts_print_threshold(struct chip_data_ft8057p *ts_data)
{
	int tx_num = ts_data->hw_res->tx_num;
	int rx_num = ts_data->hw_res->rx_num;
	int node_num = tx_num * rx_num;

	TPD_INFO("short threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_short_data_P, node_num, rx_num);
	print_buffer(ts_data->fts_autotest_offset->fts_raw_data_N, node_num, rx_num);

	TPD_INFO("open threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_open_data_P, node_num, rx_num);
	print_buffer(ts_data->fts_autotest_offset->fts_open_data_N, node_num, rx_num);

	TPD_INFO("cb threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_cb_data_P, node_num, rx_num);
	print_buffer(ts_data->fts_autotest_offset->fts_cb_data_N, node_num, rx_num);

	TPD_INFO("rawdata threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_raw_data_P, node_num, rx_num);
	print_buffer(ts_data->fts_autotest_offset->fts_raw_data_N, node_num, rx_num);

	TPD_INFO("uniformity threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_uniformity_data_P, node_num,
	             rx_num);
	print_buffer(ts_data->fts_autotest_offset->fts_uniformity_data_N, node_num,
	             rx_num);

	TPD_INFO("lcd noise threshold max/min:");
	print_buffer(ts_data->fts_autotest_offset->fts_lcd_noise_P, node_num, rx_num);
	print_buffer(ts_data->fts_autotest_offset->fts_lcd_noise_N, node_num, rx_num);
}

int fts_test_entry(struct chip_data_ft8057p *ts_data,
                   struct auto_testdata *focal_testdata)
{
	int ret = 0;
	int i = 0;
	int node_num = ts_data->hw_res->tx_num * ts_data->hw_res->rx_num;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_SAVE_ERR("FW_VER:0x%02x, tx_num:%d, rx_num:%d\n", ts_data->fwver,
	                  ts_data->hw_res->tx_num, ts_data->hw_res->rx_num);

	/*set node valid*/
	for (i = 0; i < node_num; i++) {
		ts_data->node_valid[i] = 1;
	}

	ts_data->black_screen_test = ts_data->ts->gesture_test.flag;
	TPD_INFO("black_screen_test is %d\n", ts_data->black_screen_test);
#ifdef FTS_KIT
	ret = fts_get_threshold(ts_data, focal_testdata);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("get threshold fail,ret=%d\n", ret);
		return 0xFF;
	}
#else
	ret = fts_get_threshold_from_img(ts_data, focal_testdata);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("get threshold from img fail,ret=%d\n", ret);
		return 0xFF;
	}
#endif

	fts_print_threshold(ts_data);

	ret = enter_factory_mode(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
		ret = 0xFF;
		goto test_err;
	}

	ret = fts_test_read_reg(0x14, &ts_data->fre_num);
	if (ret < 0) {
		FTS_TEST_SAVE_ERR("read fre_num fails");
		goto test_err;
	}
	TPD_INFO("fre_num:%d", ts_data->fre_num);

	ret = get_channel_num(ts_data);

	if (ret < 0) {
		FTS_TEST_SAVE_ERR("check channel num fail,ret=%d\n", ret);
		ret = 0xFF;
		goto test_err;
	}

	FTS_TEST_FUNC_EXIT();
	return ret;

test_err:
	enter_work_mode();
	FTS_TEST_FUNC_EXIT();
	return ret;
}

