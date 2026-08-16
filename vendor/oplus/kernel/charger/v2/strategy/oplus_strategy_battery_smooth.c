// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[STRATEGY_BS]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_monitor.h>
#include <oplus_strategy.h>
#include <oplus_battery_log.h>
#include "../monitor/oplus_chg_track.h"

#define MIN_SOC 0
#define MAX_SOC 100
#define SOC_TABLE_SIZE ((MAX_SOC) + 1)

#define MIN_CHG_SPLIT_SOC 20
#define MAX_CHG_SPLIT_SOC MAX_SOC

#define MAX_DISCHG_DELTA_SOC 5
#define MIN_DISCHG_SOC 15

#define MAX_SOC_DIFF_SOC_CENTI 300 /* 3% */

#define MIN_SLOPE 70
#define MAX_SLOPE 150

#define DESGIN_CAP_TO_FCC(cap) ((cap) * 90 / 100)

#define RESERVE_TABLE_PROP "oplus_spec,reserve_table_centipercent"

#define MIN_RESERVE 0
#define MAX_RESERVE 1000 /* 10% */

#define PERCENT_SCALE 100
#define FULL_SCALE (100 * (PERCENT_SCALE))
#define MIN_FULL_SOC 9500
#define INIT_MIN_SOC 1000

#define SOC_CENTI_DELTA_IGNORE	25	/* 0.25% */
#define MAX_SMOOTH_LEAD_CENTI	600	/* 6% */

#define UPDATE_MAP_DEBOUNCE_MS 10000

#define BASE64_ENCODE_LEN(raw_len) (DIV_ROUND_UP((raw_len) * 4, 3) + 1)
#define BASE64_DECODE_LEN(encoded_len) ((encoded_len) * 3 / 4)

#define TRACK_CACHE_SIZE (TOPIC_MSG_STR_BUF - 1)
#define TRACK_FIFO_SIZE BASE64_DECODE_LEN(ALIGN_DOWN(TRACK_CACHE_SIZE - 1, 4))

#define BATTERY_LOG_FIFO_SIZE 256
#define BATTERY_LOG_CACHE_SIZE BASE64_ENCODE_LEN(BATTERY_LOG_FIFO_SIZE)

#define UPLOAD_PERIOD 86400LL /* 1 day */
#define VALID_UTC_TIME 1577808000LL /* 20200101 */

struct reserve_cfg {
	int start_soc;
	int end_soc;

	int dischg_reserve;
	int full_reserve;
	int chg_reserve;
	int chg_split_soc;
};
#define RESERVE_CFG_MEMBERS	(sizeof(struct reserve_cfg) / sizeof(int))

struct bs_strategy {
	struct oplus_chg_strategy strategy;

	struct reserve_cfg *cfgs;
	int cfg_count;

	struct oplus_mms *gauge_topic;
	struct oplus_mms *err_topic;
	int init_ui_soc;
	int soc;
	int soc_centi;
	int smooth_soc;
	int smooth_soc_centi;
	int smooth_map[SOC_TABLE_SIZE];
	int smooth_map_lower;
	int smooth_map_upper;

	bool chg_online_inited;
	bool chg_online;
	bool chg_full;
	bool inited;
	int batt_rm;
	int batt_fcc;
	struct mutex lock;

	unsigned long last_map_update_jiffies;
	struct delayed_work map_update_work;
	bool map_update_pending;
	bool last_chg_online;

	struct kfifo track_fifo;
	char *track_cache;

	struct kfifo battery_log_fifo;
	char *battery_log_cache;

	time64_t last_send_time;

	struct rtc_time tm;

	int last_chg_smooth_soc;
	int last_chg_soc_centi;
	int last_chg_ref_centi;
	bool last_chg_valid;
	int chg_dis_delta_acc_centi;
};

enum map_type {
	MAP_TYPE_INIT_SPLIT_EQ_MAX = 0,
	MAP_TYPE_INIT_DISCHG = 1,
	MAP_TYPE_CHG_SPLIT_EQ_MAX = 2,
	MAP_TYPE_CHG_SMOOTH_EQ_SPLIT = 3,
	MAP_TYPE_CHG_SMOOTH_LT_SPLIT = 4,
	MAP_TYPE_CHG_SMOOTH_GT_SPLIT = 5,
	MAP_TYPE_DISCHG_SMOOTH_EQ_MAX = 6,
	MAP_TYPE_DISCHG_FULL = 7,
	MAP_TYPE_DISCHG_NO_RESERVE = 8,
	MAP_TYPE_DISCHG_OTHER = 9,
	MAP_TYPE_MAX = 0xFF,
};

struct __attribute__((packed)) bs_track {
	uint8_t len;
	uint8_t type;
	time64_t utc;
	union {
		struct __attribute__((packed)) {
			uint16_t chg_reserve;
			uint16_t soc_centi;
			uint8_t init_ui_soc;
		} init_split_eq_max;

		struct __attribute__((packed)) {
			uint16_t soc_centi;
			uint8_t init_ui_soc;
		} init_dischg;

		struct __attribute__((packed)) {
			uint8_t smooth_soc;
			uint16_t smooth_soc_centi;
			uint16_t soc_centi;
			uint16_t chg_reserve;
		} chg_split_eq_max;

		struct __attribute__((packed)) {
			uint8_t smooth_soc;
			uint16_t smooth_upper;
			uint16_t chg_reserve;
		} chg_smooth_eq_split;

		struct __attribute__((packed)) {
			uint8_t smooth_soc;
			uint16_t smooth_soc_centi;
			uint16_t soc_centi;
			uint16_t split_upper;
			uint16_t chg_reserve;
			uint8_t chg_split_soc;
		} chg_smooth_lt_split;

		struct __attribute__((packed)) {
			uint8_t chg_split_soc;
			uint16_t split_upper;
			uint16_t chg_reserve;
			uint16_t smooth_upper;
			uint8_t smooth_soc;
		} chg_smooth_gt_split;

		struct __attribute__((packed)) {
			uint16_t smooth_soc_centi;
			uint16_t soc_centi;
			uint16_t dischg_reserve;
			uint16_t extra_reserve;
		} dischg_smooth_eq_max;

		struct __attribute__((packed)) {
			uint16_t soc_centi;
			uint16_t dischg_reserve;
			uint16_t extra_reserve;
		} dischg_full;

		struct __attribute__((packed)) {
			uint8_t smooth_soc;
			uint16_t smooth_soc_centi;
			uint16_t soc_centi;
			uint8_t soc;
		} dischg_no_reserve;

		struct __attribute__((packed)) {
			uint8_t smooth_soc;
			uint16_t smooth_sub1_upper;
			uint16_t smooth_upper;
			uint16_t dischg_reserve;
		} dischg_other;
	};
};

static time64_t bs_get_current_time_s(struct bs_strategy *bs)
{
	struct timespec64 ts64;
	char tz_buf[16];
	int gmtoff = 0;

	if (!bs)
		return 0;

	ktime_get_real_ts64(&ts64);

	if (oplus_chg_track_time_zone_get(tz_buf) >= 0) {
		if (kstrtoint(tz_buf, 10, &gmtoff) != 0) {
			chg_err("invalid timezone format: %s\n", tz_buf);
			gmtoff = 0;
		}
	}

	ts64.tv_sec = ts64.tv_sec + gmtoff;
	rtc_time64_to_tm(ts64.tv_sec, &bs->tm);

	bs->tm.tm_year += 1900;
	bs->tm.tm_mon += 1;

	return ts64.tv_sec;
}

static const char oplus_base64_table[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static int oplus_base64_encode(const u8 *src, int srclen, char *dst)
{
	u32 ac = 0;
	int bits = 0;
	int i;
	char *cp = dst;

	for (i = 0; i < srclen; i++) {
		ac = (ac << 8) | src[i];
		bits += 8;
		do {
			bits -= 6;
			*cp++ = oplus_base64_table[(ac >> bits) & 0x3f];
		} while (bits >= 6);
	}
	if (bits) {
		*cp++ = oplus_base64_table[(ac << (6 - bits)) & 0x3f];
		bits -= 6;
	}
	while (bits < 0) {
		*cp++ = '=';
		bits += 2;
	}
	return cp - dst;
}

static int bs_track_init(struct bs_strategy *bs)
{
	int rc = 0;

	rc = kfifo_alloc(&bs->track_fifo, TRACK_FIFO_SIZE, GFP_KERNEL);
	if (rc != 0) {
		chg_err("kfifo alloc failed, rc = %d\n", rc);
		return rc;
	}
	bs->last_send_time = bs_get_current_time_s(bs);

	bs->track_cache = kzalloc(TRACK_CACHE_SIZE, GFP_KERNEL);
	if (!bs->track_cache) {
		chg_err("kfifo track_cache failed\n");
		rc = -ENOMEM;
		goto err;
	}
	memset(&bs->tm, 0, sizeof(struct rtc_time));

	return 0;
err:
	kfifo_free(&bs->track_fifo);
	return rc;
}

static int bs_battery_log_init(struct bs_strategy *bs)
{
	int rc = 0;

	rc = kfifo_alloc(&bs->battery_log_fifo, BATTERY_LOG_FIFO_SIZE, GFP_KERNEL);
	if (rc != 0) {
		chg_err("kfifo alloc failed, rc = %d\n", rc);
		return rc;
	}

	bs->battery_log_cache = kzalloc(BATTERY_LOG_CACHE_SIZE, GFP_KERNEL);
	if (!bs->battery_log_cache) {
		chg_err("kfifo battery_log_cache failed\n");
		rc = -ENOMEM;
		goto err;
	}

	return 0;
err:
	kfifo_free(&bs->battery_log_fifo);
	return rc;
}

static uint8_t bs_track_calc_size(enum map_type type) {
	static const uint8_t extra_len_map[] = {
		[MAP_TYPE_INIT_SPLIT_EQ_MAX] = sizeof(((struct bs_track*)0)->init_split_eq_max),
		[MAP_TYPE_INIT_DISCHG] = sizeof(((struct bs_track*)0)->init_dischg),
		[MAP_TYPE_CHG_SPLIT_EQ_MAX] = sizeof(((struct bs_track*)0)->chg_split_eq_max),
		[MAP_TYPE_CHG_SMOOTH_EQ_SPLIT] = sizeof(((struct bs_track*)0)->chg_smooth_eq_split),
		[MAP_TYPE_CHG_SMOOTH_LT_SPLIT] = sizeof(((struct bs_track*)0)->chg_smooth_lt_split),
		[MAP_TYPE_CHG_SMOOTH_GT_SPLIT] = sizeof(((struct bs_track*)0)->chg_smooth_gt_split),
		[MAP_TYPE_DISCHG_SMOOTH_EQ_MAX] = sizeof(((struct bs_track*)0)->dischg_smooth_eq_max),
		[MAP_TYPE_DISCHG_FULL] = sizeof(((struct bs_track*)0)->dischg_full),
		[MAP_TYPE_DISCHG_NO_RESERVE] = sizeof(((struct bs_track*)0)->dischg_no_reserve),
		[MAP_TYPE_DISCHG_OTHER] = sizeof(((struct bs_track*)0)->dischg_other),
	};

	const uint8_t base_len = sizeof(((struct bs_track*)0)->len) +
				sizeof(((struct bs_track*)0)->type) +
				sizeof(((struct bs_track*)0)->utc);

	if (type < 0 || type >= ARRAY_SIZE(extra_len_map)) {
		chg_err("invalid map type %u (out of range)\n", type);
		return 0;
	}

	return base_len + extra_len_map[type];
}

static int bs_track_fifo_pop_and_encode(struct bs_strategy *bs)
{
	int rc = 0;
	unsigned int total_len = kfifo_len(&bs->track_fifo);
	uint8_t *raw_data = NULL;
	int b64_len;

	if (total_len == 0)
		return -ENODATA;

	if (total_len > TRACK_FIFO_SIZE) {
		chg_err("data length exceeds limit (%u > %u)\n", total_len, TRACK_FIFO_SIZE);
		return -EOVERFLOW;
	}

	raw_data = kzalloc(total_len, GFP_KERNEL);
	if (!raw_data) {
		chg_err("failed to allocate raw data buffer\n");
		return -ENOMEM;
	}

	if (kfifo_out(&bs->track_fifo, raw_data, total_len) != total_len) {
		chg_err("incomplete read from FIFO\n");
		rc = -EIO;
	} else {
		b64_len = oplus_base64_encode(raw_data, total_len, bs->track_cache);
		if (b64_len > TRACK_CACHE_SIZE) {
			chg_err("base64 length exceeds limit (%d > %d)\n", b64_len, TRACK_CACHE_SIZE);
			rc = -EOVERFLOW;
		} else {
			bs->track_cache[b64_len] = '\0';
			rc = 0;
		}
	}

	kfree(raw_data);
	return rc;
}

static int bs_battery_log_fifo_pop_and_encode(struct bs_strategy *bs)
{
	int rc = 0;
	unsigned int total_len = kfifo_len(&bs->battery_log_fifo);
	uint8_t *raw_data = NULL;
	int b64_len;

	if (total_len == 0)
		return -ENODATA;

	if (total_len > BATTERY_LOG_FIFO_SIZE) {
		chg_err("data length exceeds limit (%u > %u)\n", total_len, BATTERY_LOG_FIFO_SIZE);
		return -EOVERFLOW;
	}

	raw_data = kzalloc(total_len, GFP_KERNEL);
	if (!raw_data) {
		chg_err("failed to allocate raw data buffer\n");
		return -ENOMEM;
	}

	if (kfifo_out(&bs->battery_log_fifo, raw_data, total_len) != total_len) {
		chg_err("incomplete read from FIFO\n");
		rc = -EIO;
	} else {
		b64_len = oplus_base64_encode(raw_data, total_len, bs->battery_log_cache);
		if (b64_len > BATTERY_LOG_CACHE_SIZE) {
			chg_err("base64 length exceeds limit (%d > %d)\n", b64_len, BATTERY_LOG_CACHE_SIZE);
			rc = -EOVERFLOW;
		} else {
			bs->battery_log_cache[b64_len] = '\0';
			rc = 0;
		}
	}

	kfree(raw_data);
	return rc;
}

static bool is_err_topic_available(struct bs_strategy *bs)
{
	if (!bs->err_topic)
		bs->err_topic = oplus_mms_get_by_name("error");
	return !!bs->err_topic;
}

static int bs_track_upload(struct bs_strategy *bs) {
	int rc = 0;
	struct mms_msg *msg = NULL;

	 rc = bs_track_fifo_pop_and_encode(bs);
	if (rc != 0) {
		chg_err("encode and cache failed, rc=%d\n", rc);
		return rc;
	}

	if (!is_err_topic_available(bs)) {
		chg_err("err topic not available\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_BS_INFO,
					 "%s", bs->track_cache);
	if (!msg) {
		chg_err("alloc ERR_ITEM_BS_INFO msg failed\n");
		memset(bs->track_cache, 0, TRACK_CACHE_SIZE);
		return -ENOMEM;
	}
	memset(bs->track_cache, 0, TRACK_CACHE_SIZE);

	rc = oplus_mms_publish_msg_sync(bs->err_topic, msg);
	if (rc < 0) {
		chg_err("publish bs info failed, rc=%d\n", rc);
		kfree(msg);
	} else {
		bs->last_send_time = bs_get_current_time_s(bs);
	}

	return rc;
}

static int bs_battery_log_fifo_push(struct bs_strategy *bs, struct bs_track *track, uint8_t total_need)
{
	int rc = 0;
	uint8_t old_len_buf[sizeof(((struct bs_track*)0)->len)];
	uint8_t old_len;
	uint8_t *old_buff = NULL;

	while (kfifo_avail(&bs->battery_log_fifo) < total_need) {
		if (kfifo_out_peek(&bs->battery_log_fifo, old_len_buf, sizeof(old_len_buf))
		    != sizeof(old_len_buf)) {
			chg_err("battery_log_fifo peek failed (need %hhu)\n", total_need);
			rc = -ENOSPC;
			goto out;
		}

		old_len = *(uint8_t *) old_len_buf;
		old_buff = kzalloc(old_len, GFP_KERNEL);
		if (!old_buff) {
			chg_err("failed to alloc buffer for old record (len=%hhu)\n", old_len);
			rc = -ENOMEM;
			goto out;
		}

		if (kfifo_out(&bs->battery_log_fifo, old_buff, old_len) != old_len) {
			chg_err("failed to pop old record (len=%hhu)\n", old_len);
			rc = -EIO;
			goto out;
		}
		chg_info("popped old record (len=%hhu hex=[%*ph]) to free space\n", old_len, old_len, old_buff);
		kfree(old_buff);
		old_buff = NULL;
	}

	if (kfifo_in(&bs->battery_log_fifo, track, total_need) != total_need) {
		chg_err("failed to write to battery_log_fifo\n");
		rc = -EIO;
	}
out:
	kfree(old_buff);
	return rc;
}

static int bs_track_push_to_fifo(struct bs_strategy *bs, struct bs_track *track) {
	int rc = 0;
	uint8_t total_need = bs_track_calc_size(track->type);

	if (total_need == 0)
		return -EINVAL;

	track->len = total_need;
	track->utc = bs_get_current_time_s(bs);

	chg_info("type=%d len=%d utc=%llu hex=[%*ph]\n", track->type, track->len, track->utc,
		total_need, (uint8_t *)track);

	if (kfifo_avail(&bs->track_fifo) < total_need || kfifo_len(&bs->track_fifo) + total_need > TRACK_FIFO_SIZE) {
		rc = bs_track_upload(bs);
		chg_err("fifo full, upload rc=%d\n", rc);
	}

	if (kfifo_in(&bs->track_fifo, track, total_need) != total_need) {
		chg_err("failed to write track data\n");
		rc = -EIO;
	} else {
		rc = 0;
	}

	bs_battery_log_fifo_push(bs, track, total_need);

	return rc;
}

static bool is_gauge_topic_available(struct bs_strategy *bs)
{
	if (!bs->gauge_topic)
		bs->gauge_topic = oplus_mms_get_by_name("gauge");
	return !!bs->gauge_topic;
}

struct reserve_cfg *find_reserve_cfg_by_soc(struct bs_strategy *bs, int soc)
{
	int left = 0;
	int right = bs->cfg_count - 1;
	int mid;
	struct reserve_cfg *cfg = NULL;

	if (!bs->cfgs || bs->cfg_count == 0)
		return NULL;

	soc = clamp_val(soc, MIN_SOC, MAX_SOC);

	while (left <= right) {
		mid = left + (right - left) / 2;
		cfg = &bs->cfgs[mid];

		if (soc >= cfg->start_soc && soc <= cfg->end_soc)
			return cfg;
		else if (soc < cfg->start_soc)
			right = mid - 1;
		else
			left = mid + 1;
	}

	chg_err("no config found for soc %d%%, available range: [%d%%, %d%%]\n", soc, bs->cfgs[0].start_soc,
		bs->cfgs[bs->cfg_count - 1].end_soc);
	return NULL;
}

static int smooth_soc_to_soc_centi(struct bs_strategy *bs, int start_index, int end_index,
	int x1, int x2, int y1, int y2)
{
	int i;
	s64 slope;

	if (start_index < 0 || end_index >= SOC_TABLE_SIZE || start_index > end_index || x1 >= x2 || y1 >= y2) {
		chg_err("invalid parameter [%d %d] [%d %d] [%d %d]\n",
			start_index, end_index, x1, x2, y1, y2);
		return -EINVAL;
	}

	/*
	 * slope = (y2 - y1) * 100 * 100 / (x2 - x1)
	 * i=[start_index, end_index]
	 * y = (y1 + (i * 100 - x1) * slope) / (100 * 100)
	 */
	slope = div_s64((s64)(y2 - y1) * PERCENT_SCALE * PERCENT_SCALE, x2 - x1);

	for (i = start_index; i <= end_index; i++)
		bs->smooth_map[i] = clamp_val(
			y1 + div_s64((i * PERCENT_SCALE - x1) * slope, PERCENT_SCALE * PERCENT_SCALE), 0, FULL_SCALE);

	return 0;
}

static void bs_save_dischg_ref(struct bs_strategy *bs)
{
	if (bs->smooth_soc >= MIN_SOC && bs->smooth_soc <= MAX_SOC) {
		bs->last_chg_smooth_soc = bs->smooth_soc;
		bs->last_chg_soc_centi = bs->soc_centi;
		bs->last_chg_ref_centi = bs->smooth_map[bs->smooth_soc];
		bs->last_chg_valid = true;
	} else {
		bs->last_chg_valid = false;
	}
}

static bool bs_calc_chg_lower_bound(struct bs_strategy *bs, struct reserve_cfg *cfg,
	int delta_dis, int *lower_bound)
{
	int last_acc = bs->chg_dis_delta_acc_centi;

	bs->chg_dis_delta_acc_centi += delta_dis;

	if (bs->chg_dis_delta_acc_centi < cfg->dischg_reserve)
		*lower_bound = bs->last_chg_ref_centi - delta_dis;
	else if (last_acc < cfg->dischg_reserve)
		*lower_bound = bs->last_chg_ref_centi - (cfg->dischg_reserve - last_acc);
	else
		*lower_bound = bs->last_chg_ref_centi;

	if (*lower_bound < 0)
		return false;

	chg_info("last=%d delta=%d acc=%d dischg=%d lb=%d\n",
		bs->last_chg_ref_centi, delta_dis, bs->chg_dis_delta_acc_centi,
		cfg->dischg_reserve, *lower_bound);

	return true;
}

static bool bs_get_chg_lower_bound(struct bs_strategy *bs, int *lower_bound)
{
	int delta_dis;
	struct reserve_cfg *cfg = NULL;

	if (!bs->last_chg_valid || bs->last_chg_smooth_soc != bs->smooth_soc) {
		bs->chg_dis_delta_acc_centi = 0;
		return false;
	}

	cfg = find_reserve_cfg_by_soc(bs, bs->smooth_soc);
	if (!cfg) {
		chg_err("no config for smooth_soc %d\n", bs->smooth_soc);
		return false;
	}

	delta_dis = bs->last_chg_soc_centi - bs->soc_centi;
	if (delta_dis < -SOC_CENTI_DELTA_IGNORE)
		return false;

	if (delta_dis <= SOC_CENTI_DELTA_IGNORE ||
	    (bs->smooth_soc * PERCENT_SCALE - bs->soc_centi > MAX_SMOOTH_LEAD_CENTI)) {
		*lower_bound = bs->last_chg_ref_centi;
		return true;
	}

	return bs_calc_chg_lower_bound(bs, cfg, delta_dis, lower_bound);
}

static void handle_chg_map_split_eq_max(struct bs_strategy *bs, struct reserve_cfg *cfg)
{
	struct bs_track track;
	int lower_bound = 0;
	bool has_lower = false;

	track.type = (uint8_t)MAP_TYPE_CHG_SPLIT_EQ_MAX;
	track.chg_split_eq_max.smooth_soc = (uint8_t)bs->smooth_soc;
	track.chg_split_eq_max.smooth_soc_centi = (uint16_t)bs->smooth_soc_centi;
	track.chg_split_eq_max.soc_centi = (uint16_t)bs->soc_centi;
	track.chg_split_eq_max.chg_reserve = (uint16_t)cfg->chg_reserve;

	has_lower = bs_get_chg_lower_bound(bs, &lower_bound);

	if (bs->smooth_soc < MAX_SOC - 1 && bs->smooth_soc != MIN_SOC) {
		if (has_lower) {
			track.chg_split_eq_max.smooth_soc_centi = (uint16_t)(bs->smooth_soc * PERCENT_SCALE);
			track.chg_split_eq_max.soc_centi = (uint16_t)lower_bound;
			/* [0,smooth_soc] -> [0,lower_bound] */
			smooth_soc_to_soc_centi(bs, 0, bs->smooth_soc, 0, bs->smooth_soc * PERCENT_SCALE, 0, lower_bound);
			bs->smooth_map[bs->smooth_soc] = lower_bound;
		} else {
			/* [0,smooth_soc] -> [0,soc_centi] */
			smooth_soc_to_soc_centi(bs, 0, bs->smooth_soc, 0, bs->smooth_soc_centi, 0, bs->soc_centi);
		}
		if (bs->smooth_map[bs->smooth_soc] <= FULL_SCALE - cfg->chg_reserve)
			/* [smooth_soc+1,100] -> [map[smooth_soc],FULL_SCALE-chg_reserve] */
			smooth_soc_to_soc_centi(bs, bs->smooth_soc + 1, MAX_SOC, bs->smooth_soc * PERCENT_SCALE,
				FULL_SCALE, bs->smooth_map[bs->smooth_soc], FULL_SCALE - cfg->chg_reserve);
		else
			/* [0,100] -> [0,FULL_SCALE-chg_reserve] */
			smooth_soc_to_soc_centi(bs, 0, MAX_SOC, 0, FULL_SCALE, 0, FULL_SCALE - cfg->chg_reserve);
	} else {
		/* [0,100] -> [0,FULL_SCALE-chg_reserve] */
		smooth_soc_to_soc_centi(bs, 0, MAX_SOC, 0, FULL_SCALE, 0, FULL_SCALE - cfg->chg_reserve);
	}

	bs_track_push_to_fifo(bs, &track);
}

static void handle_chg_map_smooth_eq_split(struct bs_strategy *bs, struct reserve_cfg *cfg)
{
	struct bs_track track;

	track.type = (uint8_t)MAP_TYPE_CHG_SMOOTH_EQ_SPLIT;
	track.chg_smooth_eq_split.smooth_soc = (uint8_t)bs->smooth_soc;
	track.chg_smooth_eq_split.smooth_upper = (uint16_t)bs->smooth_map[bs->smooth_soc];
	track.chg_smooth_eq_split.chg_reserve = (uint16_t)cfg->chg_reserve;

	if (bs->smooth_soc < MAX_SOC && bs->smooth_soc != MIN_SOC) {
		/* [0,smooth_soc] -> [0,map[smooth_soc]-chg_reserve] */
		smooth_soc_to_soc_centi(bs, 0, bs->smooth_soc, 0, bs->smooth_soc * PERCENT_SCALE,
			0, bs->smooth_map[bs->smooth_soc] - cfg->chg_reserve);

		/* [smooth_soc+1,100] -> [map[smooth_soc], FULL_SCALE] */
		smooth_soc_to_soc_centi(bs, bs->smooth_soc + 1, MAX_SOC, bs->smooth_soc * PERCENT_SCALE,
			FULL_SCALE, bs->smooth_map[bs->smooth_soc], FULL_SCALE);
	} else {
		/* [0,100] -> [0,FULL_SCALE] */
		smooth_soc_to_soc_centi(bs, 0, MAX_SOC, 0, FULL_SCALE, 0, FULL_SCALE);
	}

	bs_track_push_to_fifo(bs, &track);
}

static void handle_chg_map_smooth_lt_split(struct bs_strategy *bs, struct reserve_cfg *cfg)
{
	struct bs_track track;

	track.type = (uint8_t)MAP_TYPE_CHG_SMOOTH_LT_SPLIT;
	track.chg_smooth_lt_split.smooth_soc = (uint8_t)bs->smooth_soc;
	track.chg_smooth_lt_split.smooth_soc_centi = (uint16_t)bs->smooth_soc_centi;
	track.chg_smooth_lt_split.soc_centi = (uint16_t)bs->soc_centi;
	track.chg_smooth_lt_split.split_upper = (uint16_t)bs->smooth_map[cfg->chg_split_soc];
	track.chg_smooth_lt_split.chg_reserve = (uint16_t)cfg->chg_reserve;
	track.chg_smooth_lt_split.chg_split_soc = (uint8_t)cfg->chg_split_soc;

	if (bs->smooth_soc == MIN_SOC)
		bs->smooth_map[MIN_SOC] = 0;
	else
		/* [0,smooth_soc] -> [0,soc_centi] */
		smooth_soc_to_soc_centi(bs, 0, bs->smooth_soc, 0, bs->smooth_soc_centi,
			0, bs->soc_centi);

	if (bs->smooth_map[bs->smooth_soc] < bs->smooth_map[cfg->chg_split_soc] - cfg->chg_reserve) {
		/* [smooth_soc+1,split_soc] -> [map[smooth_soc],map[split_soc]-chg_reserve] */
		smooth_soc_to_soc_centi(bs, bs->smooth_soc + 1, cfg->chg_split_soc, bs->smooth_soc * PERCENT_SCALE,
			cfg->chg_split_soc * PERCENT_SCALE,
			bs->smooth_map[bs->smooth_soc], bs->smooth_map[cfg->chg_split_soc] - cfg->chg_reserve);

		/* [split_soc+1,100] -> [map[split_soc],FULL_SCALE] */
		smooth_soc_to_soc_centi(bs, cfg->chg_split_soc + 1, MAX_SOC, cfg->chg_split_soc * PERCENT_SCALE,
			FULL_SCALE, bs->smooth_map[cfg->chg_split_soc], FULL_SCALE);
	} else {
		/* [smooth_soc+1,100] -> [map[smooth_soc],FULL_SCALE] */
		smooth_soc_to_soc_centi(bs, bs->smooth_soc + 1, MAX_SOC, bs->smooth_soc * PERCENT_SCALE,
			FULL_SCALE, bs->smooth_map[bs->smooth_soc], FULL_SCALE);
	}

	bs_track_push_to_fifo(bs, &track);
}

static void handle_chg_map_smooth_gt_split(struct bs_strategy *bs, struct reserve_cfg *cfg)
{
	struct bs_track track;

	track.type = (uint8_t)MAP_TYPE_CHG_SMOOTH_GT_SPLIT;
	track.chg_smooth_gt_split.chg_split_soc = (uint8_t)cfg->chg_split_soc;
	track.chg_smooth_gt_split.split_upper = (uint16_t)bs->smooth_map[cfg->chg_split_soc];
	track.chg_smooth_gt_split.chg_reserve = (uint16_t)cfg->chg_reserve;
	track.chg_smooth_gt_split.smooth_upper = (uint16_t)bs->smooth_map[bs->smooth_soc];
	track.chg_smooth_gt_split.smooth_soc = (uint8_t)bs->smooth_soc;

	/* [0,split_soc] -> [0, map[split_soc]-chg_reserve] */
	smooth_soc_to_soc_centi(bs, 0, cfg->chg_split_soc, 0, cfg->chg_split_soc * PERCENT_SCALE,
		    0, bs->smooth_map[cfg->chg_split_soc] - cfg->chg_reserve);

	if (bs->smooth_map[cfg->chg_split_soc] < bs->smooth_map[bs->smooth_soc]) {
		/*[split_soc+1,smooth_soc] -> [map[split_soc],map[smooth_soc]] */
		smooth_soc_to_soc_centi(bs, cfg->chg_split_soc + 1, bs->smooth_soc,
			cfg->chg_split_soc * PERCENT_SCALE, bs->smooth_soc * PERCENT_SCALE,
			bs->smooth_map[cfg->chg_split_soc], bs->smooth_map[bs->smooth_soc]);

		/* [smooth_soc+1,100] -> [map[smooth_soc],FULL_SCALE]*/
		smooth_soc_to_soc_centi(bs, bs->smooth_soc + 1, MAX_SOC,
			bs->smooth_soc * PERCENT_SCALE, FULL_SCALE,
			bs->smooth_map[bs->smooth_soc], FULL_SCALE);
	} else {
		/*[split_soc+1,100] -> [map[split_soc],FULL_SCALE] */
		smooth_soc_to_soc_centi(bs, cfg->chg_split_soc + 1, MAX_SOC,
			cfg->chg_split_soc * PERCENT_SCALE, FULL_SCALE,
			bs->smooth_map[cfg->chg_split_soc], FULL_SCALE);
	}

	bs_track_push_to_fifo(bs, &track);
}

static void bs_update_chg_map(struct bs_strategy *bs)
{
	struct reserve_cfg *cfg;

	cfg = find_reserve_cfg_by_soc(bs, bs->smooth_soc);
	if (!cfg) {
		chg_err("no config for smooth_soc %d\n", bs->smooth_soc);
		return;
	}

	if (cfg->chg_split_soc == MAX_SOC)
		handle_chg_map_split_eq_max(bs, cfg);
	else if (bs->smooth_soc == cfg->chg_split_soc)
		handle_chg_map_smooth_eq_split(bs, cfg);
	else if (bs->smooth_soc < cfg->chg_split_soc)
		handle_chg_map_smooth_lt_split(bs, cfg);
	else if (bs->smooth_soc > cfg->chg_split_soc)
		handle_chg_map_smooth_gt_split(bs, cfg);

	bs->smooth_map[MIN_SOC] = 0;
	bs->smooth_map[MAX_SOC] = FULL_SCALE;
}

static bool is_dischg_no_reserve(struct bs_strategy *bs)
{
	return (bs->soc < MIN_DISCHG_SOC ||
		bs->smooth_soc < MIN_DISCHG_SOC ||
		bs->smooth_soc > bs->soc + MAX_DISCHG_DELTA_SOC);
}

static void bs_update_dischg_map(struct bs_strategy *bs)
{
	struct reserve_cfg *cfg;
	int extra_reserve = 0;
	struct bs_track track;

	cfg = find_reserve_cfg_by_soc(bs, bs->smooth_soc);
	if (!cfg) {
		chg_err("no config for smooth_soc %d\n", bs->smooth_soc);
		return;
	}

	if (bs->chg_full)
		extra_reserve = cfg->full_reserve;

	if (bs->smooth_soc == MAX_SOC) {
		track.type = (uint8_t)MAP_TYPE_DISCHG_SMOOTH_EQ_MAX;
		track.dischg_smooth_eq_max.smooth_soc_centi = (uint16_t)bs->smooth_soc_centi;
		track.dischg_smooth_eq_max.soc_centi = (uint16_t)bs->soc_centi;
		track.dischg_smooth_eq_max.dischg_reserve = (uint16_t)cfg->dischg_reserve;
		track.dischg_smooth_eq_max.extra_reserve = (uint16_t)extra_reserve;

		/* [0,100] -> [0,FULL_SCALE-dischg_reserve-full_reserve] */
		smooth_soc_to_soc_centi(bs, 0, MAX_SOC, 0, bs->smooth_soc_centi,
			0, bs->soc_centi - cfg->dischg_reserve - extra_reserve);
	} else if (bs->chg_full) {
		track.type = (uint8_t)MAP_TYPE_DISCHG_FULL;
		track.dischg_full.soc_centi = (uint16_t)bs->soc_centi;
		track.dischg_full.dischg_reserve = (uint16_t)cfg->dischg_reserve;
		track.dischg_full.extra_reserve = (uint16_t)extra_reserve;

		/* [0,100] -> [0,FULL_SCALE-dischg_reserve-full_reserve] */
		smooth_soc_to_soc_centi(bs, 0, MAX_SOC, 0, FULL_SCALE,
			0, max(bs->soc_centi, MIN_FULL_SOC) - cfg->dischg_reserve - extra_reserve);
	} else if (is_dischg_no_reserve(bs)) {
		track.type = (uint8_t)MAP_TYPE_DISCHG_NO_RESERVE;
		track.dischg_no_reserve.smooth_soc = (uint8_t)bs->smooth_soc;
		track.dischg_no_reserve.smooth_soc_centi = (uint16_t)bs->smooth_soc_centi;
		track.dischg_no_reserve.soc_centi = (uint16_t)bs->soc_centi;
		track.dischg_no_reserve.soc = (uint8_t)bs->soc;

		chg_info("soc too low or smooth_soc-soc too large [%d %d]\n", bs->soc, bs->smooth_soc);
		if (bs->smooth_soc == MIN_SOC)
			bs->smooth_map[MIN_SOC] = 0;
		else
			/* [0,smooth_soc] -> [0,soc_centi] */
			smooth_soc_to_soc_centi(bs, 0, bs->smooth_soc, 0, bs->smooth_soc_centi,
				0, bs->soc_centi);

		/* [smooth_soc+1,100] -> [map[smooth_soc], FULL_SCALE] */
		smooth_soc_to_soc_centi(bs, bs->smooth_soc + 1, MAX_SOC,
			bs->smooth_soc * PERCENT_SCALE, FULL_SCALE,
			bs->smooth_map[bs->smooth_soc], FULL_SCALE);
	} else {
		track.type = (uint8_t)MAP_TYPE_DISCHG_OTHER;
		track.dischg_other.smooth_soc = (uint8_t)bs->smooth_soc;
		track.dischg_other.smooth_sub1_upper = (uint16_t)bs->smooth_map[bs->smooth_soc - 1];
		track.dischg_other.smooth_upper = (uint16_t)bs->smooth_map[bs->smooth_soc];
		track.dischg_other.dischg_reserve = (uint16_t)cfg->dischg_reserve;

		/* [0,smooth_soc-1] -> [0, map[smooth_soc-1]-dischg_reserve] */
		smooth_soc_to_soc_centi(bs, 0, bs->smooth_soc - 1, 0, (bs->smooth_soc -1) * PERCENT_SCALE,
			0, bs->smooth_map[bs->smooth_soc - 1] - cfg->dischg_reserve);

		/* map[smooth_soc] = map[smooth_soc] */

		/* [smooth_soc+1, 100] -> [map[smooth_soc], FULL_SCALE] */
		smooth_soc_to_soc_centi(bs, bs->smooth_soc + 1, MAX_SOC, bs->smooth_soc * PERCENT_SCALE, FULL_SCALE,
			bs->smooth_map[bs->smooth_soc], FULL_SCALE);
	}

	bs->smooth_map[MIN_SOC] = 0;
	bs->smooth_map[MAX_SOC] = FULL_SCALE;
	bs->chg_full = false;

	bs_track_push_to_fifo(bs, &track);

	bs_save_dischg_ref(bs);
}

static void bs_update_smmoth_soc(struct bs_strategy *bs)
{
	int i;
	s64 numerator, denominator;

	for (i = 0; i < SOC_TABLE_SIZE; i++) {
		if (bs->soc_centi <= bs->smooth_map[i]) {
			bs->smooth_soc = i;
			if (i == 0) {
				bs->smooth_map_lower = 0;
				bs->smooth_map_upper = 0;
				bs->smooth_soc_centi = 0;
			} else {
				bs->smooth_map_lower = bs->smooth_map[bs->smooth_soc - 1];
				bs->smooth_map_upper = bs->smooth_map[bs->smooth_soc];
				numerator = (int64_t)(bs->soc_centi - bs->smooth_map_lower) * PERCENT_SCALE;
				denominator = bs->smooth_map_upper - bs->smooth_map_lower;
				if (denominator == 0) {
					bs->smooth_soc_centi = bs->smooth_soc * PERCENT_SCALE;
				} else {
					bs->smooth_soc_centi = clamp_val((bs->smooth_soc - 1) * PERCENT_SCALE +
						div_s64(numerator, denominator), 0, FULL_SCALE);
				}
			}
			break;
		}
	}
	if (i == SOC_TABLE_SIZE) {
		bs->smooth_soc = i - 1;
		bs->smooth_map_lower = bs->smooth_map[bs->smooth_soc - 1];
		bs->smooth_map_upper = bs->smooth_map[bs->smooth_soc];
		bs->smooth_soc_centi = FULL_SCALE;
		chg_err("soc %d, max %d, need check smooth_map\n", bs->soc_centi, bs->smooth_map[i - 1]);
	}
}

static int bs_update_data(struct bs_strategy *bs)
{
	union mms_msg_data data = { 0 };
	int soc_centi = 0;

	if (!bs->inited) {
		chg_err("not ready\n");
		return -EAGAIN;
	}

	if (!is_gauge_topic_available(bs)) {
		chg_err("gauge topic is null\n");
		return -ENODEV;
	}

	oplus_mms_get_item_data(bs->gauge_topic, GAUGE_ITEM_RM, &data, false);
	bs->batt_rm = max(data.intval, 0);

	oplus_mms_get_item_data(bs->gauge_topic, GAUGE_ITEM_FCC, &data, false);
	bs->batt_fcc = data.intval;
	if (bs->batt_fcc <= 0) {
		chg_err("fcc %d invalid, use design cap\n", bs->batt_fcc);
		bs->batt_fcc = DESGIN_CAP_TO_FCC(oplus_gauge_get_batt_capacity_mah(bs->gauge_topic));
	}

	oplus_mms_get_item_data(bs->gauge_topic, GAUGE_ITEM_SOC_CENTI, &data, true);
	soc_centi = data.intval;
	if (soc_centi < 0) {
		if (bs->batt_fcc <= 0)
			bs->soc_centi = bs->soc * PERCENT_SCALE;
		else
			bs->soc_centi = clamp_val(div_s64((s64)bs->batt_rm * (s64)FULL_SCALE, bs->batt_fcc),
						0, FULL_SCALE);
	} else {
		bs->soc_centi = clamp_val(soc_centi, 0, FULL_SCALE);
	}
	oplus_mms_get_item_data(bs->gauge_topic, GAUGE_ITEM_SOC, &data, false);
	bs->soc = data.intval;

	if (abs(bs->soc * PERCENT_SCALE - bs->soc_centi) >= MAX_SOC_DIFF_SOC_CENTI) {
		chg_err("|soc %d - soc_centi %d.%02d| too large\n",
			bs->soc, bs->soc_centi / PERCENT_SCALE, bs->soc_centi % PERCENT_SCALE);
		bs->soc_centi = bs->soc * PERCENT_SCALE;
	} else if (bs->soc == 0) {
		bs->soc_centi = bs->soc * PERCENT_SCALE;
	}

	bs_update_smmoth_soc(bs);

	chg_info("online=%d full=%d rm=%d fcc=%d soc=%d %d %d(%d~%d) smooth_soc=%d %d\n",
		bs->chg_online, bs->chg_full, bs->batt_rm, bs->batt_fcc,
		bs->soc, soc_centi, bs->soc_centi, bs->smooth_map_lower,
		bs->smooth_map_upper, bs->smooth_soc, bs->smooth_soc_centi);

	return 0;
}

static void print_smooth_map(struct bs_strategy *bs)
{
	char buf[1024];
	int i, j, end, len = 0, group_size = 10;

	for (i = 0; i <= 90; i += group_size) {
		end = (i == 90) ? 100 : (i + group_size - 1);
		len += scnprintf(buf + len, sizeof(buf) - len, "%d~%d: ", i, end);

		for (j = i; j <= end; j++) {
			len += scnprintf(buf + len, sizeof(buf) - len, " %d.%02d",
					 bs->smooth_map[j] / PERCENT_SCALE, bs->smooth_map[j] % PERCENT_SCALE);
		}

		if (end < 100)
			len += scnprintf(buf + len, sizeof(buf) - len, " | ");
	}
	chg_info("%s\n", buf);
}

static void bs_update_map(struct bs_strategy *bs)
{
	if (!bs->inited) {
		chg_err("not ready\n");
		return;
	}

	bs_update_data(bs);
	if (bs->chg_online)
		bs_update_chg_map(bs);
	else
		bs_update_dischg_map(bs);
	print_smooth_map(bs);
}

static void bs_init_chg_map(struct bs_strategy *bs, struct reserve_cfg *cfg)
{
	int soc_centi = 0;
	struct bs_track track;

	if (cfg->chg_split_soc == MAX_SOC) {
		track.type = (uint8_t)MAP_TYPE_INIT_SPLIT_EQ_MAX;
		track.init_split_eq_max.chg_reserve = (uint16_t)cfg->chg_reserve;
		track.init_split_eq_max.soc_centi = (uint16_t)bs->soc_centi;
		track.init_split_eq_max.init_ui_soc = (uint8_t)bs->init_ui_soc;
		bs_track_push_to_fifo(bs, &track);
		if (bs->init_ui_soc >= MIN_DISCHG_SOC && bs->init_ui_soc < MAX_SOC - 1 &&
		    bs->soc_centi >= INIT_MIN_SOC) {
			soc_centi = clamp_val(bs->soc_centi,
				max(bs->init_ui_soc - MAX_DISCHG_DELTA_SOC, MIN_SOC) * PERCENT_SCALE,
				min(bs->init_ui_soc + MAX_DISCHG_DELTA_SOC, MAX_SOC) * PERCENT_SCALE);
			/* [0,ui_soc] -> [0,soc_centi] */
			smooth_soc_to_soc_centi(bs, 0, bs->init_ui_soc, 0,
				bs->init_ui_soc * PERCENT_SCALE - PERCENT_SCALE / 2, 0, soc_centi);

			/* [ui_soc+1,100] -> [map[soc_centi],FULL_SCALE-chg_reserve] */
			smooth_soc_to_soc_centi(bs, bs->init_ui_soc + 1, MAX_SOC, bs->init_ui_soc * PERCENT_SCALE,
				FULL_SCALE, bs->smooth_map[bs->init_ui_soc], FULL_SCALE - cfg->chg_reserve);
			return;
		}
	}

	/* [0,100] -> [0,FULL_SCALE] */
	smooth_soc_to_soc_centi(bs, 0, MAX_SOC, 0, FULL_SCALE, 0, FULL_SCALE);
}

static void bs_init_dischg_map(struct bs_strategy *bs)
{
	int soc_centi = 0;
	struct bs_track track;

	track.type = (uint8_t)MAP_TYPE_INIT_DISCHG;
	track.init_dischg.soc_centi = (uint16_t)bs->soc_centi;
	track.init_dischg.init_ui_soc = (uint8_t)bs->init_ui_soc;
	bs_track_push_to_fifo(bs, &track);

	if (bs->init_ui_soc >= MIN_DISCHG_SOC && bs->init_ui_soc < MAX_SOC - 1 && bs->soc_centi >= INIT_MIN_SOC) {
		soc_centi = clamp_val(bs->soc_centi,
			max(bs->init_ui_soc - MAX_DISCHG_DELTA_SOC, MIN_SOC) * PERCENT_SCALE,
			min(bs->init_ui_soc + MAX_DISCHG_DELTA_SOC, MAX_SOC) * PERCENT_SCALE);
		/* [0,ui_soc] -> [0,soc_centi] */
		smooth_soc_to_soc_centi(bs, 0, bs->init_ui_soc, 0,
			bs->init_ui_soc * PERCENT_SCALE - PERCENT_SCALE / 2, 0, soc_centi);

		/* [ui_soc+1,100] -> [map[soc_centi],FULL_SCALE] */
		smooth_soc_to_soc_centi(bs, bs->init_ui_soc + 1, MAX_SOC, bs->init_ui_soc * PERCENT_SCALE,
			FULL_SCALE, bs->smooth_map[bs->init_ui_soc], FULL_SCALE);
	} else {
		/* [0,100] -> [0,FULL_SCALE] */
		smooth_soc_to_soc_centi(bs, 0, MAX_SOC, 0, FULL_SCALE, 0, FULL_SCALE);
	}
}

static void bs_init_map(struct bs_strategy *bs)
{
	struct reserve_cfg *cfg;

	if (bs->init_ui_soc < 0 || !bs->chg_online_inited)
		return;

	cfg = find_reserve_cfg_by_soc(bs, bs->init_ui_soc);
	if (!cfg) {
		chg_err("no config found for soc %d\n", bs->init_ui_soc);
		return;
	}

	if (bs->chg_online) {
		bs_init_chg_map(bs, cfg);
	} else {
		bs_init_dischg_map(bs);
	}

	bs->smooth_map[MIN_SOC] = 0;
	bs->smooth_map[MAX_SOC] = FULL_SCALE;
	print_smooth_map(bs);
}

static void bs_map_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bs_strategy *bs = container_of(dwork, struct bs_strategy, map_update_work);
	bool should_update = false;

	mutex_lock(&bs->lock);
	if (bs->map_update_pending) {
		should_update = (bs->chg_online != bs->last_chg_online);

		if (should_update) {
			bs_update_map(bs);
			bs->last_chg_online = bs->chg_online;
			bs->last_map_update_jiffies = jiffies;
		}
		bs->map_update_pending = false;
	}
	mutex_unlock(&bs->lock);
}

static bool is_valid_reserve(int reserve)
{
	return (abs(reserve) >= MIN_RESERVE) && (abs(reserve) <= MAX_RESERVE);
}

static int validate_reserve_value(struct reserve_cfg *cfg)
{
	if (!is_valid_reserve(cfg->dischg_reserve)) {
		chg_err("invalid dischg reserve: %d\n", cfg->dischg_reserve);
		return -EINVAL;
	}

	if (!is_valid_reserve(cfg->full_reserve)) {
		chg_err("invalid full reserve: %d\n", cfg->full_reserve);
		return -EINVAL;
	}

	if (!is_valid_reserve(cfg->chg_reserve)) {
		chg_err("invalid chg reserve: %d\n", cfg->chg_reserve);
		return -EINVAL;
	}

	return 0;
}

static int validate_reserve_soc_range(struct reserve_cfg *cfg)
{
	if (cfg->start_soc < MIN_SOC || cfg->end_soc > MAX_SOC) {
		chg_err("soc range [%d,%d] out of bounds [0,100]\n",
			cfg->start_soc, cfg->end_soc);
		return -EINVAL;
	}

	if (cfg->start_soc > cfg->end_soc) {
		chg_err("start_soc > end_soc [%d,%d]\n",
			cfg->start_soc, cfg->end_soc);
		return -EINVAL;
	}

	if (cfg->chg_split_soc < MIN_CHG_SPLIT_SOC || cfg->chg_split_soc > MAX_CHG_SPLIT_SOC) {
		chg_err("split_soc %d out of range [%d,%d]\n",
			cfg->chg_split_soc, MIN_CHG_SPLIT_SOC, MAX_CHG_SPLIT_SOC);
		return -EINVAL;
	}

	return 0;
}

static int validate_reserve_chg_slope(struct reserve_cfg *cfg)
{
	s64 slope_low, slope_high, numerator, denominator;

	if (cfg->chg_split_soc > 0) {
		numerator = (s64)cfg->chg_split_soc * PERCENT_SCALE - cfg->chg_reserve;
		denominator = cfg->chg_split_soc;
		slope_low = div_s64(numerator, denominator);

		if (slope_low < MIN_SLOPE || slope_low > MAX_SLOPE) {
			chg_err("low slope %lld/soc out of range [%d,%d]: split_soc=%d, chg_reserve=%d\n",
				slope_low, MIN_SLOPE, MAX_SLOPE, cfg->chg_split_soc, cfg->chg_reserve);
			return -EINVAL;
		}
	}

	if (cfg->chg_split_soc < MAX_SOC) {
		numerator = FULL_SCALE - ((s64)cfg->chg_split_soc * PERCENT_SCALE - cfg->chg_reserve);
		denominator = MAX_SOC - cfg->chg_split_soc;
		slope_high = div_s64(numerator, denominator);

		if (slope_high < MIN_SLOPE || slope_high > MAX_SLOPE) {
			chg_err("high slope %lld/soc out of range [%d,%d]: split_soc=%d, chg_reserve=%d\n",
				slope_high, MIN_SLOPE, MAX_SLOPE, cfg->chg_split_soc, cfg->chg_reserve);
			return -EINVAL;
		}
	}

	return 0;
}

static int validate_reserve_continuity(struct reserve_cfg *cfgs, int count)
{
	int i;

	if (count <= 0)
		return -EINVAL;

	if (cfgs[0].start_soc != MIN_SOC) {
		chg_err("first config must start at %d%% (got %d%%)\n", MIN_SOC, cfgs[0].start_soc);
		return -EINVAL;
	}

	if (cfgs[count - 1].end_soc != MAX_SOC) {
		chg_err("last config must end at %d%% (got %d%%)\n", MAX_SOC, cfgs[count - 1].end_soc);
		return -EINVAL;
	}

	for (i = 1; i < count; i++) {
		if (cfgs[i].start_soc != cfgs[i - 1].end_soc + 1) {
			chg_err("gap/overlap between config %d and %d: prev_end=%d%%, current_start=%d%%\n",
				i - 1, i, cfgs[i - 1].end_soc, cfgs[i].start_soc);
			return -EINVAL;
		}
	}

	return 0;
}

static int parse_reserve_config(struct device_node *node, struct bs_strategy *bs)
{
	struct reserve_cfg *cfg;
	int rc = 0, i = 0, count = 0;

	count = of_property_count_u32_elems(node, RESERVE_TABLE_PROP);
	if (count <= 0 || (count % RESERVE_CFG_MEMBERS) || (count / RESERVE_CFG_MEMBERS > SOC_TABLE_SIZE)) {
		chg_err("invalid config count: %d\n", count);
		return -EINVAL;
	}

	bs->cfg_count = count / RESERVE_CFG_MEMBERS;
	bs->cfgs = kcalloc(bs->cfg_count, sizeof(struct reserve_cfg), GFP_KERNEL);
	if (!bs->cfgs) {
		chg_err("alloc cfgs failed\n");
		return -ENOMEM;
	}

	rc = of_property_read_u32_array(node, RESERVE_TABLE_PROP, (u32 *)bs->cfgs, count);
	if (rc) {
		chg_err("read config failed: %d\n", rc);
		return rc;
	}

	for (i = 0; i < bs->cfg_count; i++) {
		cfg = &bs->cfgs[i];

		rc = validate_reserve_value(cfg);
		if (rc)
			return rc;

		rc = validate_reserve_soc_range(cfg);
		if (rc)
			return rc;

		rc = validate_reserve_chg_slope(cfg);
		if (rc)
			return rc;

		chg_info("soc %3d%% - %3d%%: dischg=%d.%02d%%, full=%d.%02d%%, chg=%d.%02d%% %d%%\n",
			cfg->start_soc, cfg->end_soc,
			cfg->dischg_reserve / PERCENT_SCALE, cfg->dischg_reserve % PERCENT_SCALE,
			cfg->full_reserve / PERCENT_SCALE, cfg->full_reserve % PERCENT_SCALE,
			cfg->chg_reserve / PERCENT_SCALE, cfg->chg_reserve % PERCENT_SCALE,
			cfg->chg_split_soc);
	}

	rc = validate_reserve_continuity(bs->cfgs, bs->cfg_count);

	return rc;
}

static int bs_dump_log_data(char *buffer, int size, void *dev_data)
{
	struct bs_strategy *bs = dev_data;

	if (!buffer || !bs)
		return -ENOMEM;

	mutex_lock(&bs->lock);
	if (bs->inited) {
		bs_battery_log_fifo_pop_and_encode(bs);
		snprintf(buffer, size, ",%d.%02d,%d,%d.%02d,%d.%02d,%d.%02d,%s",
			bs->soc_centi / PERCENT_SCALE, bs->soc_centi % PERCENT_SCALE,
			bs->smooth_soc,
			bs->smooth_soc_centi / PERCENT_SCALE, bs->smooth_soc_centi % PERCENT_SCALE,
			bs->smooth_map_lower / PERCENT_SCALE, bs->smooth_map_lower % PERCENT_SCALE,
			bs->smooth_map_upper / PERCENT_SCALE, bs->smooth_map_upper % PERCENT_SCALE,
			bs->battery_log_cache);
		memset(bs->battery_log_cache, 0, BATTERY_LOG_CACHE_SIZE);
	} else {
		snprintf(buffer, size, ",,,,,,");
	}
	mutex_unlock(&bs->lock);

	return 0;
}

static int bs_get_log_head(char *buffer, int size, void *dev_data)
{
	struct oplus_monitor *chip = dev_data;

	if (!buffer || !chip)
		return -ENOMEM;

	snprintf(buffer, size,
		",bs_soc_centi,bs_smooth_soc,bs_smooth_soc_centi,bs_smooth_map_lower,bs_smooth_map_upper,bs_formula");

	return 0;
}

static struct battery_log_ops battlog_bs_ops = {
	.dev_name = "bs_info",
	.dump_log_head = bs_get_log_head,
	.dump_log_content = bs_dump_log_data,
};

static struct oplus_chg_strategy *bs_strategy_alloc_by_node(struct device_node *node)
{
	struct bs_strategy *bs;
	int rc;
	int i = 0;

	if (!node) {
		chg_err("node is NULL\n");
		return ERR_PTR(-EINVAL);
	}

	bs = kzalloc(sizeof(*bs), GFP_KERNEL);
	if (!bs) {
		chg_err("alloc bs failed\n");
		return ERR_PTR(-ENOMEM);
	}

	rc = parse_reserve_config(node, bs);
	if (rc)
		goto err_free_cfgs;

	rc = bs_track_init(bs);
	if (rc)
		goto free_track;

	rc = bs_battery_log_init(bs);
	if (rc)
		goto free_battery_log;

	mutex_init(&bs->lock);
	battlog_bs_ops.dev_data = (void *)bs;
	battery_log_ops_register(&battlog_bs_ops);
	INIT_DELAYED_WORK(&bs->map_update_work, bs_map_update_work);
	bs->smooth_soc = -EINVAL;
	bs->init_ui_soc = -EINVAL;
	for (i = 0; i < SOC_TABLE_SIZE; i++)
		bs->smooth_map[i] = i * PERCENT_SCALE;
	bs->last_map_update_jiffies = jiffies;
	bs->map_update_pending = false;
	bs->last_chg_smooth_soc = -EINVAL;
	bs->last_chg_soc_centi = -EINVAL;
	bs->last_chg_ref_centi = 0;
	bs->last_chg_valid = false;
	bs->chg_dis_delta_acc_centi = 0;
	return &bs->strategy;
free_battery_log:
	kfree(bs->battery_log_cache);
	kfifo_free(&bs->battery_log_fifo);
free_track:
	kfree(bs->track_cache);
	kfifo_free(&bs->track_fifo);
err_free_cfgs:
	kfree(bs->cfgs);
	bs->cfg_count = 0;
	kfree(bs);
	return ERR_PTR(rc);
}

static int bs_strategy_release(struct oplus_chg_strategy *strategy)
{
	struct bs_strategy *bs;

	if (!strategy) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	bs = (struct bs_strategy *)strategy;

	cancel_delayed_work_sync(&bs->map_update_work);
	kfree(bs);

	return 0;
}

static int bs_strategy_init(struct oplus_chg_strategy *strategy)
{
	struct bs_strategy *bs;

	if (!strategy) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	bs = (struct bs_strategy *)strategy;

	mutex_lock(&bs->lock);
	bs->inited = true;
	mutex_unlock(&bs->lock);

	return 0;
}

static int bs_strategy_get_data(struct oplus_chg_strategy *strategy, void *ret)
{
	struct bs_strategy *bs;
	int *soc_ptr = (int *)ret;
	time64_t now;
	int rc = 0;

	if (!strategy || !ret) {
		chg_err("strategy or ret is NULL\n");
		return -EINVAL;
	}
	bs = (struct bs_strategy *)strategy;

	mutex_lock(&bs->lock);
	bs_update_data(bs);
	*soc_ptr = bs->smooth_soc;
	now = bs_get_current_time_s(bs);

	if (bs->last_send_time < VALID_UTC_TIME && now >= VALID_UTC_TIME)
		bs->last_send_time = now;
	if (now - bs->last_send_time >= UPLOAD_PERIOD && bs->last_send_time >= VALID_UTC_TIME) {
		chg_info("now=%lld last=%lld interval=%lld upload\n",
			now, bs->last_send_time, now - bs->last_send_time);
		rc = bs_track_upload(bs);
		if (rc)
			bs->last_send_time = now;
	}
	mutex_unlock(&bs->lock);

	return 0;
}

static int bs_strategy_set_process_data(struct oplus_chg_strategy *strategy, const char *type, unsigned long arg)
{
	struct bs_strategy *bs;
	bool update_map = false;

	if (!type) {
		chg_err("type is NULL\n");
		return -EINVAL;
	}

	bs = (struct bs_strategy *)strategy;

	chg_info("type = %s, arg=%lu", type, arg);
	mutex_lock(&bs->lock);
	if (sysfs_streq(type, "chg_online")) {
		if (bs->chg_online != !!arg) {
			bs->chg_online = !!arg;
			update_map = true;
		}
		if (!bs->chg_online_inited) {
			bs->chg_online_inited = true;
			bs_init_map(bs);
		}
	} else if (sysfs_streq(type, "chg_full")) {
		bs->chg_full = !!arg;
	} else if (sysfs_streq(type, "init_ui_soc")) {
		bs->init_ui_soc = (int)arg;
		bs_init_map(bs);
	} else {
		mutex_unlock(&bs->lock);
		return -ENOTSUPP;
	}

	if (update_map) {
		bs->map_update_pending = true;
		cancel_delayed_work(&bs->map_update_work);

		if (time_is_before_eq_jiffies(bs->last_map_update_jiffies + msecs_to_jiffies(UPDATE_MAP_DEBOUNCE_MS)))
			schedule_delayed_work(&bs->map_update_work, 0);
		else
			schedule_delayed_work(&bs->map_update_work, msecs_to_jiffies(UPDATE_MAP_DEBOUNCE_MS));
	}
	mutex_unlock(&bs->lock);

	return 0;
}

static int bs_strategy_get_metadata(struct oplus_chg_strategy *strategy,
	void *ret)
{
	struct bs_strategy *bs;
	struct reserve_cfg *cfg;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (ret == NULL) {
		chg_err("ret is NULL\n");
		return -EINVAL;
	}

	bs = (struct bs_strategy *)strategy;
	cfg = find_reserve_cfg_by_soc(bs, bs->smooth_soc);
	if (!cfg) {
		*((int *)ret) = 0;
		return 0;
	}

	*((int *)ret) = cfg->chg_reserve / PERCENT_SCALE;
	return 0;
}

static struct oplus_chg_strategy_desc bs_strategy_desc = {
	.name = "bs",
	.strategy_alloc_by_node = bs_strategy_alloc_by_node,
	.strategy_release = bs_strategy_release,
	.strategy_init = bs_strategy_init,
	.strategy_get_data = bs_strategy_get_data,
	.strategy_set_process_data = bs_strategy_set_process_data,
	.strategy_get_metadata = bs_strategy_get_metadata,
};

int bs_strategy_register(void)
{
	return oplus_chg_strategy_register(&bs_strategy_desc);
}

