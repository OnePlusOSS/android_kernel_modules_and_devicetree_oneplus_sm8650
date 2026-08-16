// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/string.h>

#include "../touchpanel_common.h"
#include "touchpanel_prevention.h"
#include "../touch_comon_api/touch_comon_api.h"
#include "../touchpanel_healthinfo/touchpanel_healthinfo.h"

/*******Start of LOG TAG Declear**********************************/
#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "prevent"
#else
#define TPD_DEVICE "prevent"
#endif

static void grip_prase_reclining_mode_handle(struct kernel_grip_info *grip_info);

/*judge if should exit dead zone grip*/
static bool dead_grip_judged(struct kernel_grip_info *grip_info,
			     struct point_info cur_p)
{
	struct list_head *pos = NULL;
	struct grip_zone_area *grip_area = NULL;

	list_for_each(pos, &grip_info->dead_zone_list) {
		grip_area = (struct grip_zone_area *)pos;

		if ((grip_area->support_dir >> grip_info->touch_dir) & 0x01) {
			if ((cur_p.x <= grip_area->start_x + grip_area->x_width)
			    && (cur_p.x >= grip_area->start_x) &&
			    (cur_p.y <= grip_area->start_y + grip_area->y_width)
			    && (cur_p.y >= grip_area->start_y)) {
				return false;
			}
		}
	}

	return true;
}

/* strategy 1: dead rejection*/
int dead_grip_handle(struct kernel_grip_info *grip_info, int obj_attention,
		     struct point_info *points)
{
	int i = 0;
	bool is_exit = false;
	int obj_final = obj_attention;

	for (i = 0; i < TOUCH_MAX_NUM; i++) {
		if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01) {
			if (grip_info->dead_out_status[i]) {    /*if already outside the range, skip handle*/
				continue;
			}

			is_exit = dead_grip_judged(grip_info, points[i]);
			grip_info->dead_out_status[i] = is_exit;        /*set outside flag*/

			if (!is_exit) {
				obj_final = obj_final & (~(1 << i));
			}

		} else {
			grip_info->dead_out_status[i] = 0;
		}
	}

	return obj_final;
}

static void init_filter_data(struct kernel_grip_info *grip_info, uint8_t index,
			     struct point_info point)
{
	int i = 0, cnt = 0;
	struct coord_buffer *buf = NULL;
	struct grip_point_info *latest_point = NULL;

	if (!grip_info || (index >= TOUCH_MAX_NUM)) {
		TPD_INFO("null or index too large:%d.\n", index);
		return;
	}

	cnt = grip_info->coord_filter_cnt;
	buf = &grip_info->coord_buf[cnt * index];
	latest_point = grip_info->latest_points[index];

	for (i = 0; i < cnt; i++) {
		buf[i].x = point.x;
		buf[i].y = point.y;
	}
	for (i = 0; i < POINT_DIFF_CNT; i++) {
		latest_point[i].x = point.x;
		latest_point[i].y = point.y;
	}
}

static void record_latest_point(struct kernel_grip_info *grip_info, uint8_t index, struct point_info point)
{
	int in = 0, cnt = POINT_DIFF_CNT;
	struct grip_point_info *latest_point = NULL;

	if (!grip_info || (index >= TOUCH_MAX_NUM)) {
		TPD_INFO("null or index too large:%d.\n", index);
		return;
	}
	latest_point = grip_info->latest_points[index];

	if ((point.x == latest_point[cnt - 1].x) && (point.y == latest_point[cnt - 1].y)) {
		return;        //return when same point
	}

	//point start move forward
	for (in = 0; in < cnt - 1; in++) {
		latest_point[in].x = latest_point[in + 1].x;
		latest_point[in].y = latest_point[in + 1].y;
	}

	latest_point[cnt - 1].x = point.x;
	latest_point[cnt - 1].y = point.y;
}

static void add_filter_data_tail(struct kernel_grip_info *grip_info,
				 uint8_t index, struct point_info point)
{
	int i = 0, cnt = 0;
	struct coord_buffer *buf = NULL;

	if (!grip_info || (index >= TOUCH_MAX_NUM)) {
		TPD_INFO("null or index too large:%d.\n", index);
		return;
	}

	cnt = grip_info->coord_filter_cnt;
	buf = &grip_info->coord_buf[cnt * index];

	if (cnt < 2) {
		GRIP_TP_INFO("filter buffer size is too small(%d).\n", cnt);
		return;
	}

	for (i = 0; i < cnt - 1; i++) {
		buf[i].x = buf[i + 1].x;
		buf[i].y = buf[i + 1].y;
	}

	buf[cnt - 1].x = point.x;
	buf[cnt - 1].y = point.y;
}

static void assign_filtered_data(struct kernel_grip_info *grip_info,
				 uint8_t index, struct point_info *point)
{
	int i = 0, cnt = 0;
	uint32_t total_x = 0, total_y = 0, total_weight = 0;
	struct coord_buffer *buf = NULL;

	if (!grip_info || !point || (index >= TOUCH_MAX_NUM)) {
		TPD_INFO("null or index too large:%d.\n", index);
		return;
	}

	cnt = grip_info->coord_filter_cnt;
	buf = &grip_info->coord_buf[cnt * index];

	for (i = 0; i < cnt; i++) {
		total_x += buf[i].x * buf[i].weight;
		total_y += buf[i].y * buf[i].weight;
		total_weight += buf[i].weight;
	}

	if (total_weight) {
		point->x = total_x / total_weight;
		point->y = total_y / total_weight;
	}
}

/* for grip 4.3 edge complement point start */
static int get_max_makeup_frame_num(struct kernel_grip_info *grip_info,
                                 uint8_t index, struct point_info *point)
{
	int touch_frame = 0;
	int touch_time = 0;
	int max_frame_num = 5;
	int frame_time_interval = 8;
	int tmp_interval = 8;

	if (!grip_info || (index >= TOUCH_MAX_NUM)) {
	    TPD_INFO("null or index too large:%d.\n", index);
	    return max_frame_num;
	}

	grip_info->uniform_make_up_point_up_time_interval = frame_time_interval;
	touch_frame = grip_info->frame_cnt[index];
	touch_time = grip_info->uniform_make_up_point_last_down_time[index] - grip_info->first_point[index].time_ms;
	max_frame_num = grip_info->uniform_make_up_point_num / 2;

	if(touch_time > 0 && touch_frame > 1) {
	    tmp_interval = touch_time / (touch_frame - 1);
	    if (tmp_interval >= MIN_FRAME_TIME_INTERVAL && tmp_interval <= MAX_FRAME_TIME_INTERVAL) {
	        frame_time_interval = tmp_interval;
	    }
	    grip_info->uniform_make_up_point_up_time_interval = frame_time_interval;
	    max_frame_num = grip_info->uniform_max_make_up_time / frame_time_interval;
	}
	if (max_frame_num < 2) {
	    max_frame_num = 2;
	}
	TPD_DETAIL("id:%d, touch frame:%d time:%d (%d,%d) .\n", index, touch_frame, touch_time, frame_time_interval, max_frame_num);
	return max_frame_num;
}

static void get_make_up_point_num(struct kernel_grip_info *grip_info,
                                 uint8_t index, struct point_info *point)
{
	if (abs(grip_info->uniform_make_up_x_shift_leftover[index]) > abs(grip_info->uniform_make_up_y_shift_leftover[index])) {
		if (grip_info->uniform_make_up_point_x_move_speed[index] == 0) {
			grip_info->uniform_make_up_point_num_status[index] = 2;
		} else if (abs(grip_info->uniform_make_up_x_shift_leftover[index]) >
				    abs(grip_info->uniform_make_up_point_x_move_speed[index]) * grip_info->uniform_make_up_last_point_percent_limit / 10) {
			grip_info->uniform_make_up_point_num_status[index] =
				abs(grip_info->uniform_make_up_x_shift_leftover[index]) / abs(grip_info->uniform_make_up_point_x_move_speed[index]) + 1;
		} else {
			grip_info->uniform_make_up_point_num_status[index] = 0;
		}
	} else {
		if (grip_info->uniform_make_up_point_y_move_speed[index] == 0) {
			grip_info->uniform_make_up_point_num_status[index] = 2;
		} else if (abs(grip_info->uniform_make_up_y_shift_leftover[index]) >
				    abs(grip_info->uniform_make_up_point_y_move_speed[index]) * grip_info->uniform_make_up_last_point_percent_limit / 10) {
			grip_info->uniform_make_up_point_num_status[index] =
				abs(grip_info->uniform_make_up_y_shift_leftover[index]) / abs(grip_info->uniform_make_up_point_y_move_speed[index]) + 1;
		} else {
			grip_info->uniform_make_up_point_num_status[index] = 0;
		}
	}
}

static void get_makeup_point_xy(struct kernel_grip_info *grip_info,
                                 uint8_t index, struct point_info *point)
{
	int32_t speed_x = grip_info->uniform_make_up_point_x_move_speed[index];
	int32_t speed_y = grip_info->uniform_make_up_point_y_move_speed[index];
	int32_t input_x = grip_info->uniform_make_up_last_input_point_x[index];
	int32_t input_y = grip_info->uniform_make_up_last_input_point_y[index];
	int32_t last_output_x = grip_info->uniform_last_make_up_prevent_out_x[index];
	int32_t last_output_y = grip_info->uniform_last_make_up_prevent_out_y[index];
	int32_t tmpx = abs(input_x - last_output_x);
	int32_t tmpy = abs(input_y - last_output_y);
	u16 speed_jitter = grip_info->uniform_make_up_last_point_percent_limit + 10;

	if (((speed_x * speed_x + speed_y * speed_y) * speed_jitter / 10) > (tmpx * tmpx + tmpy * tmpy)) {
		grip_info->uniform_make_up_x_shift_leftover[index] = 0;
		grip_info->uniform_make_up_y_shift_leftover[index] = 0;
		grip_info->uniform_make_up_point_num_status[index] = 0;
		point->x = input_x;
		point->y = input_y;
		return;
	}

	grip_info->uniform_make_up_x_shift_leftover[index] -=
		grip_info->uniform_make_up_x_shift_leftover[index] / grip_info->uniform_make_up_point_num_status[index];
	grip_info->uniform_make_up_y_shift_leftover[index] -=
		grip_info->uniform_make_up_y_shift_leftover[index] / grip_info->uniform_make_up_point_num_status[index];
	grip_info->uniform_make_up_point_num_status[index]--;

	if (grip_info->uniform_make_up_last_input_point_x[index] < grip_info->uniform_make_up_x_shift_leftover[index]) {
		point->x = 0;
	} else if (grip_info->uniform_make_up_last_input_point_x[index] >= grip_info->max_x + grip_info->uniform_make_up_x_shift_leftover[index]) {
		point->x = grip_info->max_x - 1;
	} else {
		point->x = grip_info->uniform_make_up_last_input_point_x[index] - grip_info->uniform_make_up_x_shift_leftover[index];
	}
	if (grip_info->uniform_make_up_last_input_point_y[index] < grip_info->uniform_make_up_y_shift_leftover[index]) {
		point->y = 0;
	} else if (grip_info->uniform_make_up_last_input_point_y[index] >= grip_info->max_y + grip_info->uniform_make_up_y_shift_leftover[index]) {
		point->y = grip_info->max_y - 1;
	} else {
		point->y = grip_info->uniform_make_up_last_input_point_y[index] - grip_info->uniform_make_up_y_shift_leftover[index];
	}
}

static void uniform_makeup_xy_shift_initial(struct kernel_grip_info *grip_info,
                                 uint8_t index, struct point_info *point)
{
	grip_info->uniform_make_up_x_shift_initial[index] = grip_info->uniform_make_up_last_input_point_x[index] - grip_info->first_point[index].x;
	grip_info->uniform_make_up_y_shift_initial[index] = grip_info->uniform_make_up_last_input_point_y[index] - grip_info->first_point[index].y;
	grip_info->uniform_make_up_x_shift_leftover[index] = grip_info->uniform_make_up_x_shift_initial[index];
	grip_info->uniform_make_up_y_shift_leftover[index] = grip_info->uniform_make_up_y_shift_initial[index];
	point->x = grip_info->first_point[index].x;
	point->y = grip_info->first_point[index].y;
	grip_info->uniform_last_make_up_prevent_out_x[index] = point->x;
	grip_info->uniform_last_make_up_prevent_out_y[index] = point->y;
}
/* for grip 4.3 edge complement point end */

/*judge if this area should skip judge*/
static bool skip_handle_judge(struct kernel_grip_info *grip_info,
			      struct point_info *cur_p)
{
	if (!grip_info->no_handle_dir) {        /*default right side*/
		if ((cur_p->x > grip_info->max_x / 2) && (cur_p->y > grip_info->no_handle_y1)
		    && (cur_p->y < grip_info->no_handle_y2)) {
			return true;
		}

	} else {
		if ((cur_p->x < grip_info->max_x / 2) && (cur_p->y > grip_info->no_handle_y1)
		    && (cur_p->y < grip_info->no_handle_y2)) {
			return true;
		}
	}

	return false;
}

//judge if should exit large area
static bool large_area_judged(struct kernel_grip_info *grip_info, uint16_t *grip_side, struct point_info *points, int index)
{
	int thd = 0;
	bool is_exit = true;
	struct list_head *pos = NULL;
	struct grip_zone_area *grip_area = NULL;
	struct point_info cur_p = points[index];

	if (skip_handle_judge(grip_info, &cur_p)) {
		return true;
	}

	list_for_each(pos, &grip_info->large_zone_list) {
		grip_area = (struct grip_zone_area *)pos;

		if ((grip_area->support_dir >> grip_info->touch_dir) & 0x01) {
			if ((cur_p.x <= grip_area->start_x + grip_area->x_width) && (cur_p.x >= grip_area->start_x) &&
			    (cur_p.y <= grip_area->start_y + grip_area->y_width) && (cur_p.y >= grip_area->start_y)) {
				*grip_side |= grip_area->grip_side;
				is_exit = false;
			}

			if ((grip_info->first_point[index].x <= grip_area->start_x + grip_area->x_width) && (grip_info->first_point[index].x >= grip_area->start_x) &&
			    (grip_info->first_point[index].y <= grip_area->start_y + grip_area->y_width) && (grip_info->first_point[index].y >= grip_area->start_y)) {
				if (((grip_area->grip_side >> TYPE_LONG_CORNER_SIDE) & 0x01) || ((grip_area->grip_side >> TYPE_SHORT_CORNER_SIDE) & 0x01)) {
					thd = grip_area->exit_thd;
				}
			}
		}
	}

	if (!is_exit && (TYPE_REJECT_DONE == grip_info->large_reject[index]) && thd) {
		if ((abs(grip_info->first_point[index].x - cur_p.x) > thd) || (abs(grip_info->first_point[index].y - cur_p.y) > thd)) {
			is_exit = true;
		}
	}

	return is_exit;
}

//judge if should exit large area for curved touchscreen
static bool large_area_judged_curved(struct kernel_grip_info *grip_info, uint16_t *grip_side, struct point_info *points, int index)
{
	int thd = 0, tx_er_thd = 0, rx_er_thd = 0;
	bool is_exit = true;
	struct list_head *pos = NULL;
	struct grip_zone_area *grip_area = NULL;
	struct point_info cur_p = points[index];
	struct curved_judge_para long_side_para = grip_info->curved_long_side_para;
	struct curved_judge_para short_side_para = grip_info->curved_short_side_para;

	list_for_each(pos, &grip_info->large_zone_list) {
		grip_area = (struct grip_zone_area *)pos;

		if ((grip_area->support_dir >> grip_info->touch_dir) & 0x01) {
			if ((cur_p.x <= grip_area->start_x + grip_area->x_width) && (cur_p.x >= grip_area->start_x) &&
			    (cur_p.y <= grip_area->start_y + grip_area->y_width) && (cur_p.y >= grip_area->start_y)) {
				*grip_side |= grip_area->grip_side;
				is_exit = false;
			}

			if ((grip_info->first_point[index].x <= grip_area->start_x + grip_area->x_width) && (grip_info->first_point[index].x >= grip_area->start_x) &&
			    (grip_info->first_point[index].y <= grip_area->start_y + grip_area->y_width) && (grip_info->first_point[index].y >= grip_area->start_y)) {
				thd = grip_area->exit_thd;
				tx_er_thd = grip_area->exit_tx_er;
				rx_er_thd = grip_area->exit_rx_er;
			}
		}
	}

	if (!is_exit && (TYPE_REJECT_DONE == grip_info->large_reject[index]) && thd) {
		if ((abs(grip_info->first_point[index].x - cur_p.x) > thd) || (abs(grip_info->first_point[index].y - cur_p.y) > thd)) {
			is_exit = true;
		}
	}

	if (is_exit && (TYPE_REJECT_DONE == grip_info->large_reject[index]) && grip_info->large_finger_status[index] >= TYPE_PALM_SHORT_SIZE) {
		if (cur_p.tx_er >= tx_er_thd || cur_p.rx_er >= rx_er_thd) {
			is_exit = false;
		}
		if (cur_p.tx_press == 0 && grip_info->first_point[index].tx_press != 0) {
			if (cur_p.rx_er <= long_side_para.normal_finger_thd_2 && cur_p.rx_press < long_side_para.large_palm_thd_1) { // finger need to exit right now
				is_exit = true;
			}
		}
		if (cur_p.rx_press == 0 && grip_info->first_point[index].rx_press != 0) {
			if (cur_p.tx_er <= short_side_para.normal_finger_thd_2 && cur_p.tx_press < short_side_para.large_palm_thd_1) { // finger need to exit right now
				is_exit = true;
			}
		}
	}

	return is_exit;
}

//judge if satisify large size
static enum large_judge_status large_shape_judged(struct kernel_grip_info *grip_info, uint16_t side, struct point_info *points, int index)
{
	uint16_t thd = 0;
	bool area_flag = false;
	uint16_t frame = grip_info->frame_cnt[index];
	struct point_info cur_p = points[index];
	enum large_judge_status judge_status = JUDGE_LARGE_CONTINUE;

	if (((side >> TYPE_SHORT_CORNER_SIDE) & 0x01) || ((side >> TYPE_LONG_CORNER_SIDE) & 0x01)) {
		if (frame <= grip_info->large_corner_frame_limit) {
			GRIP_TP_INFO("rx:%d, tx:%d.(%d %d)\n", cur_p.rx_press, cur_p.tx_press, cur_p.x, cur_p.y);
			if ((side >> TYPE_SHORT_CORNER_SIDE) & 0x01) {
				thd = grip_info->large_hor_corner_thd;

				if (cur_p.y < grip_info->max_y / 2) {
					area_flag = grip_info->first_point[index].y < grip_info->large_hor_corner_width;
				} else {
					area_flag = grip_info->first_point[index].y > grip_info->max_y - grip_info->large_hor_corner_width;
				}

				if (area_flag && (grip_info->first_point[index].tx_press >= thd) && (cur_p.tx_press >= thd)
				    && (abs(cur_p.y - grip_info->first_point[index].y) > grip_info->large_corner_distance)) {
					judge_status = JUDGE_LARGE_OK;
				}
			}

			if ((side >> TYPE_LONG_CORNER_SIDE) & 0x01) {
				thd = grip_info->large_ver_corner_thd;

				if (cur_p.x < grip_info->max_x / 2) {
					area_flag = grip_info->first_point[index].x < grip_info->large_ver_corner_width;
				} else {
					area_flag = grip_info->first_point[index].x > grip_info->max_x - grip_info->large_ver_corner_width;
				}

				if (area_flag && (grip_info->first_point[index].rx_press >= thd) && (cur_p.rx_press >= thd)
				    && (abs(cur_p.x - grip_info->first_point[index].x) > grip_info->large_corner_distance)) {
					judge_status = JUDGE_LARGE_OK;
				}
			}
		} else {
			judge_status = JUDGE_LARGE_TIMEOUT;
		}
	} else if (((side >> TYPE_LONG_SIDE) & 0x01) || ((side >> TYPE_SHORT_SIDE) & 0x01)) {
		if (frame <= grip_info->large_frame_limit) {
			GRIP_TP_INFO("rx:%d, tx:%d\n", cur_p.rx_press, cur_p.tx_press);
			if ((side >> TYPE_LONG_SIDE) & 0x01) {
				if (1 == cur_p.tx_press) {
					thd = grip_info->large_ver_thd * 2;
				} else {
					thd = grip_info->large_ver_thd;
				}
				if (cur_p.rx_press * 100 / cur_p.tx_press >= thd) {
					GRIP_TP_INFO("large reject for rx:%d, tx:%d.\n", cur_p.rx_press, cur_p.tx_press);
					judge_status = JUDGE_LARGE_OK;
				}
			}

			if ((side >> TYPE_SHORT_SIDE) & 0x01) {
				if (1 == cur_p.rx_press) {
					thd = grip_info->large_hor_thd * 2;
				} else {
					thd = grip_info->large_hor_thd;
				}
				if (cur_p.tx_press * 100 / cur_p.rx_press >= thd) {
					GRIP_TP_INFO("large reject for tx:%d, rx:%d.\n", cur_p.tx_press, cur_p.rx_press);
					judge_status = JUDGE_LARGE_OK;
				}
			}
		} else {
			judge_status = JUDGE_LARGE_TIMEOUT;
		}
	}

	return judge_status;
}

//judge if satisify large size for curved screen
static enum large_judge_status large_shape_judged_curved(struct kernel_grip_info *grip_info, uint16_t side, struct point_info *points, int index)
{
	struct point_info cur_p = points[index];
	struct curved_judge_para long_side_para = grip_info->curved_long_side_para;
	struct curved_judge_para short_side_para = grip_info->curved_short_side_para;
	enum large_judge_status judge_status = JUDGE_LARGE_CONTINUE;
	s64 now_time_ms = ktime_to_ms(ktime_get());
	s64 delta_time_ms = now_time_ms - grip_info->first_point[index].time_ms;

	if (((side >> TYPE_LONG_SIDE) & 0x01) || ((side >> TYPE_SHORT_SIDE) & 0x01)) {
		if (delta_time_ms <= grip_info->large_detect_time_ms) {
			TP_DETAIL(grip_info->tp_index, "id:%d, rx:%d, tx:%d, rx_er:%d, tx_er:%d. (%d %d)\n",
				index, cur_p.rx_press, cur_p.tx_press, cur_p.rx_er, cur_p.tx_er, cur_p.x, cur_p.y);
			if ((side >> TYPE_LONG_SIDE) & 0x01) {
				if (cur_p.tx_press == 0) { // long side
					if (cur_p.rx_press >= long_side_para.large_palm_thd_1 || (cur_p.rx_er >= long_side_para.large_palm_thd_2 &&
							delta_time_ms <= grip_info->large_detect_time_ms / 5)) {
						GRIP_TP_INFO("palm long size reject id:%d for rx:%d, tx:%d, rx_er:%d, tx_er:%d.\n", index, cur_p.rx_press,
							 cur_p.tx_press, cur_p.rx_er, cur_p.tx_er);
						grip_info->large_finger_status[index] = TYPE_PALM_LONG_SIZE;
						judge_status = JUDGE_LARGE_OK;
					} else if (cur_p.rx_er >= long_side_para.edge_finger_thd && delta_time_ms <= grip_info->large_detect_time_ms / 5) {
						grip_info->large_finger_status[index] = TYPE_EDGE_FINGER;
					} else if (grip_info->first_point[index].rx_er - cur_p.rx_er >= long_side_para.hold_finger_thd) {
						grip_info->large_finger_status[index] = TYPE_HOLD_FINGER;
					} else if (grip_info->first_point[index].rx_er <= long_side_para.normal_finger_thd_1 && cur_p.rx_er <= long_side_para.normal_finger_thd_2
						   && cur_p.rx_press <= long_side_para.normal_finger_thd_3) { // finger need to exit right now
						grip_info->large_finger_status[index] = TYPE_NORMAL_FINGER;
						judge_status = JUDGE_LARGE_TIMEOUT;
					}
				} else if (cur_p.rx_press != 0 && !(grip_info->touch_dir == VERTICAL_SCREEN && grip_info->first_point[index].y <= grip_info->max_y / 2)) { // long corner side
					if (cur_p.rx_press >= long_side_para.large_palm_thd_1 || (cur_p.rx_er >= long_side_para.large_palm_thd_2 &&
							delta_time_ms <= grip_info->large_detect_time_ms / 5)) {
						GRIP_TP_INFO("large palm corner reject id:%d for rx:%d, tx:%d, rx_er:%d, tx_er:%d.\n", index, cur_p.rx_press,
							 cur_p.tx_press, cur_p.rx_er, cur_p.tx_er);
						grip_info->large_finger_status[index] = TYPE_LARGE_PALM_CORNER;
						judge_status = JUDGE_LARGE_OK;
					} else if ((cur_p.rx_press + cur_p.tx_press >= long_side_para.palm_thd_1) &&
						   (cur_p.tx_er >= long_side_para.palm_thd_2 || cur_p.rx_er >= long_side_para.palm_thd_2)) {
						GRIP_TP_INFO("palm corner reject id:%d for rx:%d, tx:%d, rx_er:%d, tx_er:%d.\n", index, cur_p.rx_press,
							 cur_p.tx_press, cur_p.rx_er, cur_p.tx_er);
						grip_info->large_finger_status[index] = TYPE_PALM_CORNER;
						judge_status = JUDGE_LARGE_OK;
					} else if ((cur_p.rx_er / cur_p.rx_press >= long_side_para.small_palm_thd_1) && cur_p.rx_press <= long_side_para.small_palm_thd_2) {
						grip_info->large_finger_status[index] = TYPE_SMALL_PALM_CORNER;
					} else if (cur_p.rx_er >= long_side_para.edge_finger_thd && delta_time_ms <= grip_info->large_detect_time_ms / 5) {
						grip_info->large_finger_status[index] = TYPE_EDGE_FINGER;
					} else if (grip_info->first_point[index].rx_er - cur_p.rx_er >= long_side_para.hold_finger_thd) {
						grip_info->large_finger_status[index] = TYPE_HOLD_FINGER;
					}
				}
			}

			if ((side >> TYPE_SHORT_SIDE) & 0x01) {
				if (cur_p.rx_press == 0) { // short side
					if (cur_p.tx_press >= short_side_para.large_palm_thd_1 || (cur_p.tx_er >= short_side_para.large_palm_thd_2 &&
							delta_time_ms <= grip_info->large_detect_time_ms / 5)) {
						GRIP_TP_INFO("palm short size reject id:%d for rx:%d, tx:%d, rx_er:%d, tx_er:%d.\n", index, cur_p.rx_press,
							 cur_p.tx_press, cur_p.rx_er, cur_p.tx_er);
						grip_info->large_finger_status[index] = TYPE_PALM_SHORT_SIZE;
						judge_status = JUDGE_LARGE_OK;
					} else if (cur_p.tx_er >= short_side_para.edge_finger_thd && delta_time_ms <= grip_info->large_detect_time_ms / 5) {
						grip_info->large_finger_status[index] = TYPE_EDGE_FINGER;
					} else if (grip_info->first_point[index].tx_er <= short_side_para.normal_finger_thd_1 && cur_p.tx_er <= short_side_para.normal_finger_thd_2
						   && cur_p.tx_press <= short_side_para.normal_finger_thd_3) { // finger need to exit right now
						grip_info->large_finger_status[index] = TYPE_NORMAL_FINGER;
						judge_status = JUDGE_LARGE_TIMEOUT;
					}
				} else if (cur_p.tx_press != 0) { // short corner side
					if (cur_p.tx_press >= short_side_para.large_palm_thd_1 || (cur_p.tx_er >= short_side_para.large_palm_thd_2 &&
							delta_time_ms <= grip_info->large_detect_time_ms / 5)) {
						GRIP_TP_INFO("large palm corner reject id:%d for rx:%d, tx:%d, rx_er:%d, tx_er:%d.\n", index, cur_p.rx_press,
							 cur_p.tx_press, cur_p.rx_er, cur_p.tx_er);
						grip_info->large_finger_status[index] = TYPE_LARGE_PALM_CORNER;
						judge_status = JUDGE_LARGE_OK;
					} else if ((cur_p.rx_press + cur_p.tx_press >= short_side_para.palm_thd_1) &&
						   (cur_p.tx_er >= short_side_para.palm_thd_2 || cur_p.rx_er >= short_side_para.palm_thd_2)) {
						GRIP_TP_INFO("palm corner reject id:%d for rx:%d, tx:%d, rx_er:%d, tx_er:%d.\n", index, cur_p.rx_press,
							 cur_p.tx_press, cur_p.rx_er, cur_p.tx_er);
						grip_info->large_finger_status[index] = TYPE_PALM_CORNER;
						judge_status = JUDGE_LARGE_OK;
					} else if ((cur_p.tx_er / cur_p.tx_press >= short_side_para.small_palm_thd_1) && cur_p.tx_press <= short_side_para.small_palm_thd_2) {
						grip_info->large_finger_status[index] = TYPE_SMALL_PALM_CORNER;
					}
				}
			}
		} else {
			if (grip_info->large_finger_status[index] == TYPE_EDGE_FINGER) {
				GRIP_TP_INFO("edge finger reject id:%d for rx:%d, tx:%d, rx_er:%d, tx_er:%d.\n", index, cur_p.rx_press,
					 cur_p.tx_press, cur_p.rx_er, cur_p.tx_er);
				judge_status = JUDGE_LARGE_OK;
			} else if (grip_info->large_finger_status[index] == TYPE_HOLD_FINGER) {
				GRIP_TP_INFO("hold finger reject id:%d for rx:%d, tx:%d, rx_er:%d, tx_er:%d.\n", index, cur_p.rx_press,
					 cur_p.tx_press, cur_p.rx_er, cur_p.tx_er);
				judge_status = JUDGE_LARGE_OK;
			} else if (grip_info->large_finger_status[index] == TYPE_SMALL_PALM_CORNER) {
				GRIP_TP_INFO("small palm corner reject id:%d for rx:%d, tx:%d, rx_er:%d, tx_er:%d.\n", index, cur_p.rx_press,
					 cur_p.tx_press, cur_p.rx_er, cur_p.tx_er);
				judge_status = JUDGE_LARGE_OK;
			} else {
				judge_status = JUDGE_LARGE_TIMEOUT;
			}
		}
	}

	return judge_status;
}

/*judge if should exit conditional area*/
static bool condition_area_judged(struct kernel_grip_info *grip_info,
				  struct point_info *points, int index)
{
	int thd = 0;
	struct list_head *pos = NULL;
	struct grip_zone_area *condition_area = NULL;
	struct point_info cur_p = points[index];

	list_for_each(pos, &grip_info->condition_zone_list) {
		condition_area = (struct grip_zone_area *)pos;

		if ((condition_area->support_dir >> grip_info->touch_dir) & 0x01) {
			if ((cur_p.x <= condition_area->start_x + condition_area->x_width)
			    && (cur_p.x >= condition_area->start_x) &&
			    (cur_p.y <= condition_area->start_y + condition_area->y_width)
			    && (cur_p.y >= condition_area->start_y)) {
				if ((condition_area->grip_side >> TYPE_SHORT_SIDE) & 0x01) {
					if (abs(grip_info->first_point[index].x - cur_p.x) < condition_area->exit_thd) {
						return false;
					}

				} else if ((condition_area->grip_side >> TYPE_LONG_SIDE) & 0x01) {
					if (abs(grip_info->first_point[index].y - cur_p.y) < condition_area->exit_thd) {
						return false;
					}
				}
			}

			if ((grip_info->first_point[index].x <= condition_area->start_x +
			     condition_area->x_width)
			    && (grip_info->first_point[index].x >= condition_area->start_x) &&
			    (grip_info->first_point[index].y <= condition_area->start_y +
			     condition_area->y_width)
			    && (grip_info->first_point[index].y >= condition_area->start_y)) {
				thd = condition_area->exit_thd;
			}
		}
	}

	if (!grip_info->condition_out_status[index]
	    && (grip_info->frame_cnt[index] >= grip_info->condition_frame_limit)) {
		if ((abs(grip_info->first_point[index].y - cur_p.y) < thd) &&
		    (abs(grip_info->first_point[index].x - cur_p.x) < thd)) {
			return false;
		}
	}

	return true;
}

void grip_status_reset(struct kernel_grip_info *grip_info, uint8_t index)
{
	if (!grip_info) {
		TPD_INFO("null and return.\n");
		return;
	}

	if (index >= TOUCH_MAX_NUM) {
		GRIP_TP_INFO("invalid index :%d.\n", index);
		return;
	}

	grip_info->dead_out_status[index] = 0;
	grip_info->frame_cnt[index] = 0;
	grip_info->large_out_status[index] = 0;
	grip_info->large_reject[index] = 0;
	grip_info->condition_out_status[index] = 0;
	grip_info->makeup_cnt[index] = 0;
	grip_info->point_unmoved[index] = 0;
	grip_info->grip_hold_status[index] = 0;
	grip_info->large_finger_status[index] = 0;
	grip_info->large_point_status[index] = UP_POINT;

	grip_info->eli_out_status[index] = 0;
	grip_info->eli_reject_status[index] = 0;
	grip_info->sync_up_makeup[index] = false;
	grip_info->exit_match_times[index] = 0;
	grip_info->fsr_stable_time[index] = 0;
	grip_info->points_pos[index] = 0;
	grip_info->points_center_down[index] = STATUS_CENTER_UNKNOW;
	grip_info->top_shape_match_times[index] = 0;
	grip_info->finger_hold_max_rx_matched[index] = 0;
	grip_info->max_rx_matched[index] = 0;
	grip_info->max_rx_matched_cnt[index] = 0;
	grip_info->max_rx_stable_time[index] = 0;
	grip_info->dynamic_finger_hold_state[index] = 0;

	grip_info->uniform_make_up_last_input_point_x[index] = 0;
	grip_info->uniform_make_up_last_input_point_y[index] = 0;
	grip_info->uniform_make_up_x_shift_initial[index] = 0;
	grip_info->uniform_make_up_y_shift_initial[index] = 0;
	grip_info->uniform_make_up_x_shift_leftover[index] = 0;
	grip_info->uniform_make_up_y_shift_leftover[index] = 0;
	grip_info->uniform_last_make_up_prevent_out_x[index] = 0;
	grip_info->uniform_last_make_up_prevent_out_y[index] = 0;
	grip_info->uniform_make_up_point_num_status[index] = 0;
	grip_info->uniform_make_up_point_x_move_speed[index] = 0;
	grip_info->uniform_make_up_point_y_move_speed[index] = 0;

	TP_DETAIL(grip_info->tp_index, "reset id :%d.\n", index);
}

//return the index of same point
static int acquire_matched_point(struct grip_point_info *point_array, struct coord_buffer sp)
{
	int index = 0;

	for (index = 0; index < POINT_DIFF_CNT; index++) {
		if ((sp.x == point_array[index].x) && (sp.y == point_array[index].y)) {
			return index;
		}
	}
	if (index == POINT_DIFF_CNT) {
		index = 0;      //do not find the matched points
	}

	return index;
}

/*using for report touch up event*/
static  inline void touch_report_work(struct work_struct *work, unsigned int i)
{
	uint8_t up_id = 0;
	struct touchpanel_data *ts = NULL;
	struct kernel_grip_info *grip_info = NULL;
	uint16_t fiter_cnt = 0, index_exp = 0;
	int in = 0, ret = 0, point_x = 0, point_y = 0;
	struct grip_point_info *latest_point = NULL;
	int m_id = 0;
	uint16_t time_interval = 0;

	if (i >= TOUCH_MAX_NUM) {
		TPD_INFO("%s: i %d is too big\n", __func__,  i);
		return;
	}

	grip_info = container_of(work, struct kernel_grip_info, grip_up_work[i]);

	if (!grip_info) {
		TPD_INFO("grip_info is null.\n");
		return;
	}

	ts = grip_info->p_ts;
	if (!ts) {
		GRIP_TP_INFO("ts is null.\n");
		return;
	}

	if (ts->grip_info->is_curved_screen_v4_3) {
		time_interval = ts->grip_info->uniform_make_up_point_up_time_interval * 1000;
	}

	mutex_lock(&ts->report_mutex);

	if (ts->grip_info->is_curved_screen_v4_3 && ts->grip_info->uniform_make_up_point_v2_support) {
		for (m_id = 0; m_id < TOUCH_MAX_NUM; m_id++) {
			if (ts->grip_info->uniform_make_up_point_upid_status & (1 << m_id)) {
				up_id = m_id;
				ts->grip_info->uniform_make_up_point_upid_status &= (~(1 << m_id));
				break;
			}
		}
		ret = 1;
	} else {
		ret = kfifo_get(&ts->grip_info->up_fifo, &up_id);
	}

	if (!ret) {
		GRIP_TP_INFO("upfifo is empty.\n");
		goto OUT;
	}

	if (up_id >= TOUCH_MAX_NUM) {
		GRIP_TP_INFO("up_id %d is too big.\n", up_id);
		goto OUT;
	}

	if (ts->grip_info->uniform_make_up_point_v2_support &&
	    ts->grip_info->is_curved_screen_v4_3 &&
		ts->grip_info->uniform_make_up_point_num_status[up_id] > 0) {
		fiter_cnt = ts->grip_info->uniform_make_up_point_num_status[up_id];
		TPD_DETAIL("fiter_cnt:%d up_id:%d. \n", fiter_cnt, up_id);
		for (in = fiter_cnt; in > 0; in--) {
			if (!ts->grip_info->grip_hold_status[up_id]) {
				TPD_INFO("id:%d is alreay up in report.\n", up_id);
				goto OUT;
			}

			ts->grip_info->uniform_make_up_x_shift_leftover[up_id] -= ts->grip_info->uniform_make_up_x_shift_leftover[up_id] / in;
			ts->grip_info->uniform_make_up_y_shift_leftover[up_id] -= ts->grip_info->uniform_make_up_y_shift_leftover[up_id] / in;
			ts->grip_info->uniform_make_up_point_num_status[up_id]--;

			if (ts->grip_info->uniform_make_up_last_input_point_x[up_id] < ts->grip_info->uniform_make_up_x_shift_leftover[up_id]) {
				point_x = 0;
			} else if (ts->grip_info->uniform_make_up_last_input_point_x[up_id] >= grip_info->max_x + ts->grip_info->uniform_make_up_x_shift_leftover[up_id]) {
				point_x = grip_info->max_x - 1;
			} else {
				point_x = ts->grip_info->uniform_make_up_last_input_point_x[up_id] - ts->grip_info->uniform_make_up_x_shift_leftover[up_id];
			}
			if (ts->grip_info->uniform_make_up_last_input_point_y[up_id] < ts->grip_info->uniform_make_up_y_shift_leftover[up_id]) {
				point_y = 0;
			} else if (ts->grip_info->uniform_make_up_last_input_point_y[up_id] >= grip_info->max_y + ts->grip_info->uniform_make_up_y_shift_leftover[up_id]) {
				point_y = grip_info->max_y - 1;
			} else {
				point_y = ts->grip_info->uniform_make_up_last_input_point_y[up_id] - ts->grip_info->uniform_make_up_y_shift_leftover[up_id];
			}

			input_mt_slot(ts->input_dev, up_id);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, point_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, point_y);
			input_sync(ts->input_dev);

			mutex_unlock(&ts->report_mutex);
			TPD_DETAIL("id:%d, up status makeup point:(%d, %d) point num:%d. shift_leftover:(%d,%d) lastpoint:(%d,%d) \n", up_id, point_x, point_y, \
				grip_info->uniform_make_up_point_num_status[up_id],
				ts->grip_info->uniform_make_up_x_shift_leftover[up_id], ts->grip_info->uniform_make_up_y_shift_leftover[up_id], \
				ts->grip_info->uniform_make_up_last_input_point_x[up_id], ts->grip_info->uniform_make_up_last_input_point_y[up_id]);
			usleep_range(time_interval, time_interval + 1);
			mutex_lock(&ts->report_mutex);
		}
	} else if ((ts->grip_info->is_curved_screen || ts->grip_info->is_curved_screen_V2) && ts->grip_info->sync_up_makeup[up_id]) {
		fiter_cnt = ts->grip_info->coord_filter_cnt;
		latest_point = ts->grip_info->latest_points[up_id];
		index_exp = acquire_matched_point(latest_point, ts->grip_info->coord_buf[(up_id + 1) * fiter_cnt - 1]);
		TP_DETAIL(grip_info->tp_index, "id:%d start makeup from index %d.\n", up_id, index_exp);
		for (in = index_exp; in < POINT_DIFF_CNT; in++) {
			if (!ts->grip_info->grip_hold_status[up_id]) {
				GRIP_TP_INFO("id:%d is alreay up in report.\n", up_id);
				goto OUT;
			}

			point_x = latest_point[in].x;
			point_y = latest_point[in].y;
			input_mt_slot(ts->input_dev, up_id);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, point_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, point_y);
			input_sync(ts->input_dev);

			if ((in < POINT_DIFF_CNT - 1) && (latest_point[in + 1].x == point_x) && (latest_point[in + 1].y == point_y)) { //ignore same point
				continue;
			}
			mutex_unlock(&ts->report_mutex);
			TP_DETAIL(grip_info->tp_index, "makeup the real point:%d(%d, %d).\n", up_id, point_x, point_y);
			msleep(5);
			mutex_lock(&ts->report_mutex);
		}
	}

	if (!ts->grip_info->grip_hold_status[up_id]) {
		GRIP_TP_INFO("id:%d is alreay up.\n", up_id);
		goto OUT;
	}

	input_mt_slot(ts->input_dev, up_id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	grip_status_reset(ts->grip_info, up_id);                  //reset status of this ID
	if (ts->grip_info->record_total_cnt) {
		ts->grip_info->record_total_cnt--;        //update touch down count
	}
	GRIP_TP_INFO("report id(%d) up, left total:%d.\n", up_id, ts->grip_info->record_total_cnt);
	if (!ts->grip_info->record_total_cnt) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
		ts->view_area_touched = 0; //realse all touch point,must clear this flag
		ts->touch_count = 0;
		ts->irq_slot = 0;
	}
	input_sync(ts->input_dev);

#ifdef CONFIG_TOUCHPANEL_ALGORITHM
	if (ts->algo_info && ts->algo_info->point_buf) {
		ts->algo_info->point_buf[up_id].kal_x_last.x = INVALID_POINT;
		ts->algo_info->point_buf[up_id].touch_time = 0;
		ts->algo_info->point_buf[up_id].status = NORMAL;
		TP_DETAIL(grip_info->tp_index, "Reset the algorithm id:%d points\n", up_id);
	}
#endif

	if (ts->health_monitor_support) {
		tp_healthinfo_report(&ts->monitor_data, HEALTH_GRIP_UP, &up_id);
		TP_DETAIL(grip_info->tp_index, "healthinfo point %d report UP in grip\n", up_id);
	}

	ts->grip_info->grip_hold_status[up_id] = 0;
OUT:
	mutex_unlock(&ts->report_mutex);
}

#define touch_report_work(index) \
static void touch_report_work##index(struct work_struct *work) \
{ \
	touch_report_work(work, index); \
}

touch_report_work(0)
touch_report_work(1)
touch_report_work(2)
touch_report_work(3)
touch_report_work(4)
touch_report_work(5)
touch_report_work(6)
touch_report_work(7)
touch_report_work(8)
touch_report_work(9)

static inline enum hrtimer_restart touch_up_timer_func(struct hrtimer *hrtimer,
		unsigned int i)
{
	struct touchpanel_data *ts = NULL;
	struct kernel_grip_info *grip_info = NULL;

	if (i >= TOUCH_MAX_NUM) {
		TPD_INFO("%s: i %d is too big\n", __func__,  i);
		return HRTIMER_NORESTART;
	}

	grip_info = container_of(hrtimer, struct kernel_grip_info, grip_up_timer[i]);

	if (!grip_info) {
		TPD_INFO("grip_info is null.\n");
		return HRTIMER_NORESTART;
	}

	ts = grip_info->p_ts;

	if (!ts) {
		TPD_INFO("ts is null.\n");
		return HRTIMER_NORESTART;
	}

	GRIP_TP_INFO("time called once.\n");
	grip_info->work_id++;

	if (grip_info->work_id >= TOUCH_MAX_NUM) {
		grip_info->work_id = 0;
	}

	queue_work(ts->grip_info->grip_up_handle_wq,
		   &ts->grip_info->grip_up_work[grip_info->work_id]);
	return HRTIMER_NORESTART;
}

#define touch_up_timer_func(index) \
static enum hrtimer_restart touch_up_timer_func##index(struct hrtimer *hrtimer) \
{ \
	return touch_up_timer_func(hrtimer, index); \
}

touch_up_timer_func(0)
touch_up_timer_func(1)
touch_up_timer_func(2)
touch_up_timer_func(3)
touch_up_timer_func(4)
touch_up_timer_func(5)
touch_up_timer_func(6)
touch_up_timer_func(7)
touch_up_timer_func(8)
touch_up_timer_func(9)

static int large_condition_handle(struct kernel_grip_info *grip_info,
				  int obj_attention, struct point_info *points)
{
	int i = 0, up_id = 0, ret = 0;
	bool is_exit = false, dead_out = false;
	int obj_final = obj_attention;
	struct point_info tmp_point;
	uint16_t large_side = 0, fiter_cnt = grip_info->coord_filter_cnt;
	enum large_judge_status judge_state = JUDGE_LARGE_CONTINUE;

	if (grip_info->grip_handle_in_fw) {
		return obj_attention;
	}

	for (i = 0; i < TOUCH_MAX_NUM; i++) {
		if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01) {      /*finger down*/
			grip_info->frame_cnt[i]++;                           /*count down frames*/

			if (grip_info->large_out_status[i]) {        /*if already exit large area judge*/
				if (grip_info->condition_out_status[i]) {
					if ((grip_info->makeup_cnt[i] > 0) && (grip_info->makeup_cnt[i] <= fiter_cnt)) {
						tmp_point = points[i];
						assign_filtered_data(grip_info, i, &points[i]);
						add_filter_data_tail(grip_info, i, tmp_point);
						grip_info->makeup_cnt[i]++;
						GRIP_TP_INFO("make up :%d times.\n", grip_info->makeup_cnt[i]);
					}

				} else {
					grip_info->condition_out_status[i] = condition_area_judged(grip_info, points,
									     i);

					if (grip_info->condition_out_status[i]) {
						tmp_point = points[i];
						assign_filtered_data(grip_info, i, &points[i]);
						add_filter_data_tail(grip_info, i, tmp_point);
						grip_info->makeup_cnt[i]++;
						GRIP_TP_INFO("make up a:%d times.\n", grip_info->makeup_cnt[i]);

					} else {
						obj_final = obj_final & (~(1 <<
									   i));            /*reject for condition grip judge*/
					}
				}

				continue;
			}

			if (!(((grip_info->obj_prev_bit & TOUCH_BIT_CHECK) >> i) &
			      0x01)) {        /*init coord buff when first touch down*/
				init_filter_data(grip_info, i, points[i]);
				grip_info->first_point[i].x = points[i].x;
				grip_info->first_point[i].y = points[i].y;
				grip_info->first_point[i].tx_press = points[i].tx_press;
				grip_info->first_point[i].rx_press = points[i].rx_press;
			}

			grip_info->point_unmoved[i] = ((points[i].x == grip_info->coord_buf[i *
							fiter_cnt].x) &&
						       (points[i].y == grip_info->coord_buf[i * fiter_cnt].y));

			is_exit = large_area_judged(grip_info, &large_side, points, i);

			if (is_exit) {
				grip_info->large_out_status[i] = true;           /*set large outside flag*/
				grip_info->condition_out_status[i] = true;       /*set condition outside flag*/

				if (!grip_info->point_unmoved[i]) {    /*means once in large judge area*/
					tmp_point = points[i];
					assign_filtered_data(grip_info, i, &points[i]);
					add_filter_data_tail(grip_info, i, tmp_point);
					grip_info->makeup_cnt[i]++;
					GRIP_TP_INFO("make up b:%d times.\n", grip_info->makeup_cnt[i]);
				}

			} else if (TYPE_REJECT_DONE == grip_info->large_reject[i]) {
				obj_final = obj_final & (~(1 <<
							   i));                /*if already reject, just mask it*/

			} else {
				grip_info->condition_out_status[i] = condition_area_judged(grip_info, points,
								     i);

				judge_state = large_shape_judged(grip_info, large_side, points, i);

				if (JUDGE_LARGE_OK == judge_state) {
					obj_final = obj_final & (~(1 << i));            /*reject for big area*/
					grip_info->large_reject[i] = TYPE_REJECT_DONE;

				} else if (JUDGE_LARGE_TIMEOUT == judge_state) {
					grip_info->large_out_status[i] = true;       /*set outside flag*/

					if (grip_info->condition_out_status[i]) {
						if (!grip_info->point_unmoved[i]) {
							tmp_point = points[i];
							assign_filtered_data(grip_info, i, &points[i]);  /*makeup points*/
							add_filter_data_tail(grip_info, i, tmp_point);
							grip_info->makeup_cnt[i]++;
							GRIP_TP_INFO("make up c:%d times.\n", grip_info->makeup_cnt[i]);
						}

					} else {
						obj_final = obj_final & (~(1 <<
									   i));            /*reject for condition grip judge*/
					}

				} else {
					obj_final = obj_final & (~(1 << i));            /*reject for continue detect*/
					grip_info->large_reject[i] = TYPE_REJECT_HOLD;
				}
			}

		} else { /*finger up*/
			if (!grip_info->large_out_status[i]
			    && (TYPE_REJECT_DONE == grip_info->large_reject[i])) {
				GRIP_TP_INFO("reject id:%d for large area.\n", i);

			} else if (!grip_info->large_out_status[i]
				   && (TYPE_REJECT_HOLD == grip_info->large_reject[i])) {
				points[i].x = grip_info->coord_buf[i * fiter_cnt].x;
				points[i].y = grip_info->coord_buf[i * fiter_cnt].y;

				dead_out = dead_grip_judged(grip_info, points[i]);

				if (dead_out) {
					points[i].status = 1;
					grip_info->grip_hold_status[i] = 1;
					obj_final = obj_final | (1 << i);

					if (hrtimer_active(&grip_info->grip_up_timer[i])) {
						hrtimer_cancel(&grip_info->grip_up_timer[i]);
						ret = kfifo_get(&grip_info->up_fifo, &up_id);

						if (!ret) {
							GRIP_TP_INFO("large get id failed, empty.\n");
						}

						GRIP_TP_INFO("large get id(%d) and cancel timer.\n", up_id);
					}

					kfifo_put(&grip_info->up_fifo, i);   /*put the up id into up fifo*/
					GRIP_TP_INFO("large put id(%d) into fifo.\n", i);
					hrtimer_start(&grip_info->grip_up_timer[i], ktime_set(0,
							grip_info->condition_updelay_ms * 1000000), HRTIMER_MODE_REL);

				} else {
					GRIP_TP_INFO("reject id:%d for dead zone.\n", i);
				}

			} else if (!grip_info->condition_out_status[i] && grip_info->point_unmoved[i]) {
				if (grip_info->frame_cnt[i] < grip_info->condition_frame_limit) {
					points[i].x = grip_info->coord_buf[i * fiter_cnt].x;
					points[i].y = grip_info->coord_buf[i * fiter_cnt].y;

					dead_out = dead_grip_judged(grip_info, points[i]);

					if (dead_out) {
						points[i].status = 1;
						grip_info->grip_hold_status[i] = 1;
						obj_final = obj_final | (1 << i);

						if (hrtimer_active(&grip_info->grip_up_timer[i])) {
							hrtimer_cancel(&grip_info->grip_up_timer[i]);
							ret = kfifo_get(&grip_info->up_fifo, &up_id);

							if (!ret) {
								GRIP_TP_INFO("get id failed, empty.\n");
							}

							GRIP_TP_INFO("get id(%d) and cancel timer.\n", up_id);
						}

						kfifo_put(&grip_info->up_fifo, i);   /*put the up id into up fifo*/
						GRIP_TP_INFO("put id(%d) into fifo.\n", i);
						hrtimer_start(&grip_info->grip_up_timer[i], ktime_set(0,
								grip_info->condition_updelay_ms * 1000000), HRTIMER_MODE_REL);

					} else {
						GRIP_TP_INFO("reject id:%d for dead zone.\n", i);
					}

				} else {
					GRIP_TP_INFO("conditon reject for down frame:%d(%d).\n", grip_info->frame_cnt[i],
						 grip_info->condition_frame_limit);
				}
			}

			/*reset status of this id*/
			grip_info->frame_cnt[i] = 0;
			grip_info->large_out_status[i] = 0;
			grip_info->large_reject[i] = 0;
			grip_info->condition_out_status[i] = 0;
			grip_info->makeup_cnt[i] = 0;
			grip_info->point_unmoved[i] = 0;
		}
	}

	grip_info->obj_prev_bit = obj_attention;
	return obj_final;
}

static int curved_large_handle(struct kernel_grip_info *grip_info, int obj_attention, struct point_info *points)
{
	int i = 0, up_id = 0, ret = 0, j = 0;
	bool is_exit = false, dead_out = false;
	int obj_final = obj_attention;
	struct point_info tmp_point;
	uint16_t large_side = 0, fiter_cnt = grip_info->coord_filter_cnt;
	enum large_judge_status judge_state = JUDGE_LARGE_CONTINUE;

	for (i = 0; i < TOUCH_MAX_NUM; i++) {
		if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01) {      //finger down
			grip_info->frame_cnt[i]++;                           //count down frames

			if (grip_info->large_out_status[i]) {        //if already exit large area judge
				record_latest_point(grip_info, i, points[i]);       //record different points

				if (grip_info->makeup_cnt[i] > 0) {
					if (grip_info->makeup_cnt[i] <= fiter_cnt) {
						tmp_point = points[i];
						assign_filtered_data(grip_info, i, &points[i]);
						add_filter_data_tail(grip_info, i, tmp_point);
						grip_info->makeup_cnt[i]++;
						GRIP_TP_INFO("id:%d make up :%d times.(%d %d)\n", i, grip_info->makeup_cnt[i], points[i].x, points[i].y);
					} else {    //means we have make up to the real point
						grip_info->makeup_cnt[i] = MAKEUP_REAL_POINT;
					}
				}
				continue;
			}

			if (!(((grip_info->obj_prev_bit & TOUCH_BIT_CHECK) >> i) & 0x01)) {        //init coord buff when first touch down
				init_filter_data(grip_info, i, points[i]);
				grip_info->first_point[i].x = points[i].x;
				grip_info->first_point[i].y = points[i].y;
				grip_info->first_point[i].tx_press = points[i].tx_press;
				grip_info->first_point[i].rx_press = points[i].rx_press;
				grip_info->first_point[i].tx_er = points[i].tx_er;
				grip_info->first_point[i].rx_er = points[i].rx_er;
				grip_info->first_point[i].time_ms = ktime_to_ms(ktime_get());

				if (grip_info->large_point_status[i] == UP_POINT) { // if there is some point touch down, mark previous down points no need to make up point
					grip_info->large_point_status[i] = DOWN_POINT_NEED_MAKEUP;
					for (j = 0; j < TOUCH_MAX_NUM; j++) {
						if ((((obj_attention & TOUCH_BIT_CHECK) >> j) & 0x01) && i != j) {
							if (!(((grip_info->obj_prev_bit & TOUCH_BIT_CHECK) >> j) & 0x01)) {
								grip_info->large_point_status[i] = DOWN_POINT;
								grip_info->large_point_status[j] = DOWN_POINT;
							} else {
								grip_info->large_point_status[j] = DOWN_POINT;
							}
						}
					}
				}

				is_exit = large_area_judged_curved(grip_info, &large_side, points, i);
				if (is_exit) {
					grip_info->lastest_down_time_ms = ktime_to_ms(ktime_get());
				}
			}

			grip_info->point_unmoved[i] = ((points[i].x == grip_info->coord_buf[i * fiter_cnt].x) &&
						       (points[i].y == grip_info->coord_buf[i * fiter_cnt].y));

			record_latest_point(grip_info, i, points[i]);       //record different points
			is_exit = large_area_judged_curved(grip_info, &large_side, points, i);
			if (is_exit) {
				grip_info->large_out_status[i] = true;           //set large outside flag

				if (!grip_info->point_unmoved[i]) {    //means once in large judge area
					tmp_point = points[i];
					assign_filtered_data(grip_info, i, &points[i]);
					add_filter_data_tail(grip_info, i, tmp_point);
					grip_info->makeup_cnt[i]++;
					GRIP_TP_INFO("id:%d make up b:%d times.(%d %d)(%d %d %d %d)\n", i, grip_info->makeup_cnt[i], points[i].x, points[i].y,
						 points[i].tx_press, points[i].rx_press, points[i].tx_er, points[i].rx_er);
				}
			} else if (TYPE_REJECT_DONE == grip_info->large_reject[i]) {
				obj_final = obj_final & (~(1 << i));                //if already reject, just mask it
			} else {
				judge_state = large_shape_judged_curved(grip_info, large_side, points, i);
				if (JUDGE_LARGE_OK == judge_state) {
					obj_final = obj_final & (~(1 << i));            //reject for big area
					grip_info->large_reject[i] = TYPE_REJECT_DONE;
				} else if (JUDGE_LARGE_TIMEOUT == judge_state) {
					grip_info->large_out_status[i] = true;       //set outside flag
					if (!grip_info->point_unmoved[i]) {
						tmp_point = points[i];
						assign_filtered_data(grip_info, i, &points[i]);  //makeup points
						add_filter_data_tail(grip_info, i, tmp_point);
						grip_info->makeup_cnt[i]++;
						GRIP_TP_INFO("id:%d make up c:%d times.(%d %d)(%d %d %d %d)\n", i, grip_info->makeup_cnt[i], points[i].x, points[i].y,
							 points[i].tx_press, points[i].rx_press, points[i].tx_er, points[i].rx_er);
					}
				} else {
					obj_final = obj_final & (~(1 << i));            //reject for continue detect
					grip_info->large_reject[i] = TYPE_REJECT_HOLD;
				}
			}
		} else { //finger up
			if (!grip_info->large_out_status[i] && (TYPE_REJECT_DONE == grip_info->large_reject[i])) {
				GRIP_TP_INFO("reject id:%d for large area.\n", i);
			} else if (grip_info->large_point_status[i] != DOWN_POINT_NEED_MAKEUP && !grip_info->large_out_status[i]
				   && (TYPE_REJECT_HOLD == grip_info->large_reject[i])) {
				GRIP_TP_INFO("reject no need make up click point id:%d for large area.\n", i);
			} else if ((grip_info->first_point[i].time_ms - grip_info->lastest_down_time_ms <= grip_info->down_delta_time_ms)
				   && !grip_info->large_out_status[i] && (TYPE_REJECT_HOLD == grip_info->large_reject[i])) {
				GRIP_TP_INFO("reject short time click point id:%d for large area.\n", i);
			} else if (!grip_info->large_out_status[i] && (TYPE_REJECT_HOLD == grip_info->large_reject[i])) {
				points[i].x = grip_info->coord_buf[i * fiter_cnt].x;
				points[i].y = grip_info->coord_buf[i * fiter_cnt].y;

				dead_out = dead_grip_judged(grip_info, points[i]);
				if (dead_out || !grip_info->point_unmoved[i]) {
					points[i].status = 1;
					grip_info->grip_hold_status[i] = 1;
					obj_final = obj_final | (1 << i);

					if (hrtimer_active(&grip_info->grip_up_timer[i])) {
						hrtimer_cancel(&grip_info->grip_up_timer[i]);
						ret = kfifo_get(&grip_info->up_fifo, &up_id);
						if (!ret) {
							GRIP_TP_INFO("large get id failed, empty.\n");
						}
						GRIP_TP_INFO("large get id(%d) and cancel timer.\n", up_id);
					}

					kfifo_put(&grip_info->up_fifo, i);   //put the up id into up fifo
					TP_DETAIL(grip_info->tp_index, "large put id:%d(%d, %d) into fifo.\n", i, points[i].x, points[i].y);
					grip_info->sync_up_makeup[i] = true;
					hrtimer_start(&grip_info->grip_up_timer[i], ktime_set(0, grip_info->condition_updelay_ms * 1000000), HRTIMER_MODE_REL);
				} else {
					GRIP_TP_INFO("reject id:%d(%d, %d) for dead zone.\n", i, points[i].x, points[i].y);
				}
			} else if (grip_info->makeup_cnt[i] && MAKEUP_REAL_POINT != grip_info->makeup_cnt[i]) {
				points[i].x = grip_info->coord_buf[(i + 1) * fiter_cnt - 1].x;
				points[i].y = grip_info->coord_buf[(i + 1) * fiter_cnt - 1].y;
				points[i].status = 1;
				grip_info->grip_hold_status[i] = 1;
				obj_final = obj_final | (1 << i);

				if (hrtimer_active(&grip_info->grip_up_timer[i])) {
					hrtimer_cancel(&grip_info->grip_up_timer[i]);
					ret = kfifo_get(&grip_info->up_fifo, &up_id);
					if (!ret) {
						GRIP_TP_INFO("large get id failed, empty.\n");
					}
					GRIP_TP_INFO("large get id(%d) and cancel timer.\n", up_id);
				}

				kfifo_put(&grip_info->up_fifo, i);   //put the up id into up fifo
				GRIP_TP_INFO("makeup the real point:%d(%d, %d) into fifo.\n", i, points[i].x, points[i].y);
				grip_info->sync_up_makeup[i] = true;
				hrtimer_start(&grip_info->grip_up_timer[i], ktime_set(0, grip_info->condition_updelay_ms * 1000000), HRTIMER_MODE_REL);
			}

			//reset status of this id
			grip_info->frame_cnt[i] = 0;
			grip_info->large_out_status[i] = 0;
			grip_info->large_reject[i] = 0;
			grip_info->condition_out_status[i] = 0;
			grip_info->makeup_cnt[i] = 0;
			grip_info->point_unmoved[i] = 0;
			grip_info->large_finger_status[i] = 0;
			grip_info->large_point_status[i] = UP_POINT;
		}
	}

	grip_info->obj_prev_bit = obj_attention;
	return obj_final;
}
/*judge if out of the eliminated area*/
static bool eliminated_area_judged(struct kernel_grip_info *grip_info,
				   struct point_info cur_p)
{
	struct list_head *pos = NULL;
	struct grip_zone_area *grip_area = NULL;

	list_for_each(pos, &grip_info->elimination_zone_list) {
		grip_area = (struct grip_zone_area *)pos;

		if ((grip_area->support_dir >> grip_info->touch_dir) & 0x01) {
			if ((cur_p.x <= grip_area->start_x + grip_area->x_width)
			    && (cur_p.x >= grip_area->start_x) &&
			    (cur_p.y <= grip_area->start_y + grip_area->y_width)
			    && (cur_p.y >= grip_area->start_y)) {
				return false;
			}
		}
	}

	return true;
}

static int touch_elimination_handle(struct kernel_grip_info *grip_info,
				    int obj_attention, struct point_info *points)
{
	bool out_status = false;
	int i = 0, obj_final = obj_attention;
	uint16_t left_edge_bit = 0, right_edge_bit = 0;
	int left_edge_cnt = 0, right_edge_cnt = 0, left_center_cnt = 0,
	    right_center_cnt = 0;
	if (grip_info->is_curved_screen || grip_info->is_curved_screen_V2) {
		return obj_attention;
	}

	for (i = 0; i < TOUCH_MAX_NUM; i++) {
		if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01) {
			if (VERTICAL_SCREEN == grip_info->touch_dir) {
				out_status = eliminated_area_judged(grip_info, points[i]);

				if (!out_status) {
					left_edge_cnt++;
					left_edge_bit = left_edge_bit | (1 << i);

				} else {
					left_center_cnt++;
					grip_info->eli_out_status[i] = true;    /*set cross status of i*/
				}

			} else {
				if (points[i].y < grip_info->max_y / 2) {
					out_status = eliminated_area_judged(grip_info, points[i]);

					if (!out_status) {
						left_edge_cnt++;
						left_edge_bit = left_edge_bit | (1 << i);

					} else {
						left_center_cnt++;
						grip_info->eli_out_status[i] = true;    /*set cross status of i*/
					}

				} else {
					out_status = eliminated_area_judged(grip_info, points[i]);

					if (!out_status) {
						right_edge_cnt++;
						right_edge_bit = right_edge_bit | (1 << i);

					} else {
						right_center_cnt++;
						grip_info->eli_out_status[i] = true;    /*set cross status of i*/
					}
				}
			}

		} else {
			grip_info->eli_out_status[i] = false;
			grip_info->eli_reject_status[i] = false;
		}
	}

	if (VERTICAL_SCREEN == grip_info->touch_dir) {      /*in vertical mode*/
		if (left_edge_cnt
		    && left_center_cnt) {     /*need to judge which point shoud be eliminated*/
			for (i = 0; i < TOUCH_MAX_NUM; i++) {
				if ((((left_edge_bit & TOUCH_BIT_CHECK) >> i) & 0x01)
				    && !grip_info->eli_out_status[i]) {
					grip_info->eli_reject_status[i] = true;
					obj_final = obj_final & (~(1 << i));
				}
			}

		} else if (left_edge_cnt) {
			for (i = 0; i < TOUCH_MAX_NUM; i++) {
				if ((((left_edge_bit & TOUCH_BIT_CHECK) >> i) & 0x01)
				    && !grip_info->eli_out_status[i] && grip_info->eli_reject_status[i]) {
					obj_final = obj_final & (~(1 << i));
				}
			}
		}

	} else {
		/*left part of panel*/
		if (left_edge_cnt && left_center_cnt) {
			for (i = 0; i < TOUCH_MAX_NUM; i++) {
				if ((((left_edge_bit & TOUCH_BIT_CHECK) >> i) & 0x01)
				    && !grip_info->eli_out_status[i]) {
					grip_info->eli_reject_status[i] = true;
					obj_final = obj_final & (~(1 << i));
				}
			}

		} else if (left_edge_cnt) {
			for (i = 0; i < TOUCH_MAX_NUM; i++) {
				if ((((left_edge_bit & TOUCH_BIT_CHECK) >> i) & 0x01)
				    && !grip_info->eli_out_status[i] && grip_info->eli_reject_status[i]) {
					obj_final = obj_final & (~(1 << i));
				}
			}
		}

		/*right part of panel*/
		if (right_edge_cnt && right_center_cnt) {
			for (i = 0; i < TOUCH_MAX_NUM; i++) {
				if ((((right_edge_bit & TOUCH_BIT_CHECK) >> i) & 0x01)
				    && !grip_info->eli_out_status[i]) {
					grip_info->eli_reject_status[i] = true;
					obj_final = obj_final & (~(1 << i));
				}
			}

		} else if (right_edge_cnt) {
			for (i = 0; i < TOUCH_MAX_NUM; i++) {
				if ((((right_edge_bit & TOUCH_BIT_CHECK) >> i) & 0x01)
				    && !grip_info->eli_out_status[i] && grip_info->eli_reject_status[i]) {
					obj_final = obj_final & (~(1 << i));
				}
			}
		}
	}

	return obj_final;
}

static void record_point_info(struct kernel_grip_info *grip_info, point_info_type type, uint8_t index, struct point_info point)
{
	int in = 0, cnt = POINT_DIFF_CNT;
	struct grip_point_info *latest_point = NULL;

	if (!grip_info || (index >= TOUCH_MAX_NUM)) {
		TPD_INFO("null or index too large:%d.\n", index);
		return;
	}

	switch (type) {
	case TYPE_START_POINT:
		grip_info->first_point[index].x = point.x;
		grip_info->first_point[index].y = point.y;
		grip_info->first_point[index].tx_press = point.tx_press;
		grip_info->first_point[index].rx_press = point.rx_press;
		grip_info->first_point[index].tx_er = point.tx_er;
		grip_info->first_point[index].rx_er = point.rx_er;
		grip_info->first_point[index].time_ms = ktime_to_ms(ktime_get());
		break;
	case TYPE_SECOND_POINT:
		grip_info->second_point[index].x = point.x;
		grip_info->second_point[index].y = point.y;
		grip_info->second_point[index].tx_press = point.tx_press;
		grip_info->second_point[index].rx_press = point.rx_press;
		grip_info->second_point[index].tx_er = point.tx_er;
		grip_info->second_point[index].rx_er = point.rx_er;
		grip_info->second_point[index].time_ms = ktime_to_ms(ktime_get());
		break;
	case TYPE_LAST_POINT:
		grip_info->last_frame_point[index].x = point.x;
		grip_info->last_frame_point[index].y = point.y;
		grip_info->last_frame_point[index].tx_press = point.tx_press;
		grip_info->last_frame_point[index].rx_press = point.rx_press;
		grip_info->last_frame_point[index].tx_er = point.tx_er;
		grip_info->last_frame_point[index].rx_er = point.rx_er;
		grip_info->last_frame_point[index].time_ms = ktime_to_ms(ktime_get());
		break;
	case TYPE_INIT_TX_POINT:
		grip_info->txMax_frame_point[index].x = point.x;
		grip_info->txMax_frame_point[index].y = point.y;
		grip_info->txMax_frame_point[index].tx_press = point.tx_press;
		grip_info->txMax_frame_point[index].rx_press = point.rx_press;
		grip_info->txMax_frame_point[index].tx_er = point.tx_er;
		grip_info->txMax_frame_point[index].rx_er = point.rx_er;

		grip_info->txChanged_frame_point[index].x = point.x;
		grip_info->txChanged_frame_point[index].y = point.y;
		grip_info->txChanged_frame_point[index].tx_press = point.tx_press;
		grip_info->txChanged_frame_point[index].rx_press = point.rx_press;
		grip_info->txChanged_frame_point[index].tx_er = point.tx_er;
		grip_info->txChanged_frame_point[index].rx_er = point.rx_er;
		grip_info->txChanged_frame_point[index].time_ms = ktime_to_ms(ktime_get());
		break;
	case TYPE_INIT_RX_POINT:
		grip_info->rxMax_frame_point[index].x = point.x;
		grip_info->rxMax_frame_point[index].y = point.y;
		grip_info->rxMax_frame_point[index].tx_press = point.tx_press;
		grip_info->rxMax_frame_point[index].rx_press = point.rx_press;
		grip_info->rxMax_frame_point[index].tx_er = point.tx_er;
		grip_info->rxMax_frame_point[index].rx_er = point.rx_er;
		grip_info->rxMax_frame_point[index].time_ms = ktime_to_ms(ktime_get());

		grip_info->rxChanged_frame_point[index].x = point.x;
		grip_info->rxChanged_frame_point[index].y = point.y;
		grip_info->rxChanged_frame_point[index].tx_press = point.tx_press;
		grip_info->rxChanged_frame_point[index].rx_press = point.rx_press;
		grip_info->rxChanged_frame_point[index].tx_er = point.tx_er;
		grip_info->rxChanged_frame_point[index].rx_er = point.rx_er;
		grip_info->rxChanged_frame_point[index].time_ms = ktime_to_ms(ktime_get());
		break;
	case TYPE_MAX_TX_POINT:
		if ((point.tx_press > grip_info->txMax_frame_point[index].tx_press) || (0 == point.tx_press)) {
			grip_info->txMax_frame_point[index].x = point.x;
			grip_info->txMax_frame_point[index].y = point.y;
			grip_info->txMax_frame_point[index].tx_press = point.tx_press;
			grip_info->txMax_frame_point[index].rx_press = point.rx_press;
			grip_info->txMax_frame_point[index].tx_er = point.tx_er;
			grip_info->txMax_frame_point[index].rx_er = point.rx_er;
			grip_info->txMax_frame_point[index].time_ms = ktime_to_ms(ktime_get());
		}
		break;
	case TYPE_MAX_RX_POINT:
		if ((point.rx_press > grip_info->rxMax_frame_point[index].rx_press) || (0 == point.rx_press)) {
			grip_info->rxMax_frame_point[index].x = point.x;
			grip_info->rxMax_frame_point[index].y = point.y;
			grip_info->rxMax_frame_point[index].tx_press = point.tx_press;
			grip_info->rxMax_frame_point[index].rx_press = point.rx_press;
			grip_info->rxMax_frame_point[index].tx_er = point.tx_er;
			grip_info->rxMax_frame_point[index].rx_er = point.rx_er;
			grip_info->rxMax_frame_point[index].time_ms = ktime_to_ms(ktime_get());
		}
		break;
	case TYPE_RX_CHANGED_POINT:
		if (point.rx_press != grip_info->rxChanged_frame_point[index].rx_press) {
			grip_info->rxChanged_frame_point[index].x = point.x;
			grip_info->rxChanged_frame_point[index].y = point.y;
			grip_info->rxChanged_frame_point[index].tx_press = point.tx_press;
			grip_info->rxChanged_frame_point[index].rx_press = point.rx_press;
			grip_info->rxChanged_frame_point[index].tx_er = point.tx_er;
			grip_info->rxChanged_frame_point[index].rx_er = point.rx_er;
			grip_info->rxChanged_frame_point[index].time_ms = ktime_to_ms(ktime_get());
		}
		break;
	case TYPE_TX_CHANGED_POINT:
		if (point.tx_press != grip_info->txChanged_frame_point[index].tx_press) {
			grip_info->txChanged_frame_point[index].x = point.x;
			grip_info->txChanged_frame_point[index].y = point.y;
			grip_info->txChanged_frame_point[index].tx_press = point.tx_press;
			grip_info->txChanged_frame_point[index].rx_press = point.rx_press;
			grip_info->txChanged_frame_point[index].tx_er = point.tx_er;
			grip_info->txChanged_frame_point[index].rx_er = point.rx_er;
			grip_info->txChanged_frame_point[index].time_ms = ktime_to_ms(ktime_get());
		}
		break;
	case TYPE_LATEST_POINT:
		latest_point = grip_info->latest_points[index];
		if ((point.x == latest_point[cnt - 1].x) && (point.y == latest_point[cnt - 1].y)) {         //return when same point
			return;
		}
		for (in = 0; in < cnt - 1; in++) {            //point start move forward
			latest_point[in].x = latest_point[in + 1].x;
			latest_point[in].y = latest_point[in + 1].y;
		}

		latest_point[cnt - 1].x = point.x;
		latest_point[cnt - 1].y = point.y;
		break;
	default:
		GRIP_TP_INFO("record wrong type.\n");
		break;
	}
}

static void init_latest_data(struct kernel_grip_info *grip_info, uint8_t index, struct point_info point)
{
	return;     //return because of latest data have been inited in init_filter_data function
}

static void init_point_status(struct kernel_grip_info *grip_info, uint8_t index, struct point_info *point)
{
	grip_info->point_touchZ_stability_status[index] = 0;
	grip_info->middle_bottom_long_press_match_times[index] = 0;
	grip_info->middle_bottom_small_press_match_times[index] = 0;
	grip_info->middle_bottom_exit_yfsr_coupling_times[index] = 0;
}

static void start_makeup_timer(struct kernel_grip_info *grip_info, uint8_t index)
{
	int ret = 0, up_id = 0;

	if (hrtimer_active(&grip_info->grip_up_timer[index])) {
		hrtimer_cancel(&grip_info->grip_up_timer[index]);
		ret = kfifo_get(&grip_info->up_fifo, &up_id);
		if (!ret) {
			GRIP_TP_INFO("large get id failed and cancel hrtimer.\n");
		}
		GRIP_TP_INFO("large get id(%d) and cancel timer.\n", up_id);
	}

	grip_info->grip_hold_status[index] = 1;

	if (grip_info->uniform_make_up_point_v2_support && grip_info->is_curved_screen_v4_3) {
		hrtimer_start(&grip_info->grip_up_timer[index], ktime_set(0, grip_info->uniform_make_up_point_up_time_interval * 1000000UL), HRTIMER_MODE_REL);
	} else {
		kfifo_put(&grip_info->up_fifo, index);   /* put the up id into up fifo */
		hrtimer_start(&grip_info->grip_up_timer[index], ktime_set(0, grip_info->report_updelay_ms * 1000000UL), HRTIMER_MODE_REL);
	}
}

static void mask_potential_mistouch(struct kernel_grip_info *grip_info, struct point_info *points, int id)
{
	int index = 0;
	bool out_range = true;
	s64 delta_time_ms = 0;
	uint16_t min_val = 0, max_val = 0;
	struct point_info cur_p = points[id];
	uint8_t pos = grip_info->points_pos[id];
	uint16_t finger_status = 0, debounce_ms = grip_info->large_reject_debounce_time_ms;

	if ((POS_LONG_LEFT == pos) || (POS_LONG_RIGHT == pos)) {
		min_val = cur_p.y - (grip_info->max_y / grip_info->rx_num) * cur_p.rx_press / 2;
		max_val = cur_p.y + (grip_info->max_y / grip_info->rx_num) * cur_p.rx_press / 2;
	} else if ((POS_SHORT_LEFT == pos) || (POS_SHORT_RIGHT == pos)) {
		min_val = cur_p.x - (grip_info->max_x / grip_info->tx_num) * cur_p.tx_press / 2;
		max_val = cur_p.x + (grip_info->max_x / grip_info->tx_num) * cur_p.tx_press / 2;
	} else {
		GRIP_TP_INFO("%s:never go here.\n", __func__);
		return;
	}

	for (index = 0; index < TOUCH_MAX_NUM; index++) {
		if (index == id || grip_info->points_pos[index] != pos) {
			continue;
		}

		delta_time_ms = abs(grip_info->first_point[id].time_ms - grip_info->first_point[index].time_ms);         //time interval
		if ((POS_LONG_LEFT == pos) || (POS_LONG_RIGHT == pos)) {
			finger_status = TYPE_PALM_LONG_SIZE;
			out_range = (grip_info->first_point[index].y > min_val && grip_info->first_point[index].y < max_val) ? false : true;
		} else if ((POS_SHORT_LEFT == pos) || (POS_SHORT_RIGHT == pos)) {
			finger_status = TYPE_PALM_SHORT_SIZE;
			out_range = (grip_info->first_point[index].x > min_val && grip_info->first_point[index].x < max_val) ? false : true;
		}
		if ((delta_time_ms < debounce_ms) && !out_range && (TYPE_REJECT_HOLD == grip_info->large_reject[index])) {      //already down id is under suspicious detect time
			GRIP_TP_INFO("%s: mask short interval id(%d) around the large area.\n", __func__, index);
			grip_info->large_reject[index] = TYPE_REJECT_DONE;
			grip_info->large_finger_status[index] = finger_status;
		}
	}

	return;
}

static bool large_reject_covered(struct kernel_grip_info *grip_info, struct point_info *points, int id)
{
	int index = 0;
	uint16_t min_val = 0, max_val = 0;
	struct point_info cur_p = points[id];
	uint8_t pos = grip_info->points_pos[id];
	uint16_t debounce_ms = grip_info->large_reject_debounce_time_ms;
	bool out_range = true, under_time_interval = true, is_covered = false;

	for (index = 0; index < TOUCH_MAX_NUM; index++) {
		if (UP_POINT == grip_info->large_point_status[index]) {
			if (grip_info->last_points_pos[index] != pos || TYPE_REJECT_DONE != grip_info->last_large_reject[index]) {            //same point id must be handle here
				continue;
			}
			under_time_interval = abs(grip_info->last_frame_point[index].time_ms - grip_info->first_point[id].time_ms) < debounce_ms;  //new down id is under large up suspicious detect time
		} else if (DOWN_POINT == grip_info->large_point_status[index]) {
			if (index == id || grip_info->points_pos[index] != pos || TYPE_REJECT_DONE != grip_info->large_reject[index]) {       //reject id must be different from judge id
				continue;
			}
			under_time_interval = true;
		} else {
			GRIP_TP_INFO("%s:should not go here.\n", __func__);
			continue;
		}

		if ((POS_LONG_LEFT == pos) || (POS_LONG_RIGHT == pos)) {
			min_val = grip_info->rxMax_frame_point[index].y - (grip_info->max_y / grip_info->rx_num) * grip_info->rxMax_frame_point[index].rx_press / 2;
			max_val = grip_info->rxMax_frame_point[index].y + (grip_info->max_y / grip_info->rx_num) * grip_info->rxMax_frame_point[index].rx_press / 2;
			out_range = (cur_p.y > min_val && cur_p.y < max_val) ? false : true;
		} else if ((POS_SHORT_LEFT == pos) || (POS_SHORT_RIGHT == pos)) {
			min_val = grip_info->txMax_frame_point[index].x - (grip_info->max_x / grip_info->tx_num) * grip_info->txMax_frame_point[index].tx_press / 2;
			max_val = grip_info->txMax_frame_point[index].x + (grip_info->max_x / grip_info->tx_num) * grip_info->txMax_frame_point[index].tx_press / 2;
			out_range = (cur_p.x > min_val && cur_p.x < max_val) ? false : true;
		} else {
			GRIP_TP_INFO("%s:never go here.\n", __func__);
			return false;
		}

		is_covered = !out_range && under_time_interval;
		if (is_covered) {
			GRIP_TP_INFO("%s: id(%d) is included around the reject point(%d), status:%s.\n", __func__, id,
				index, DOWN_POINT == grip_info->large_point_status[index] ? "down" : "up");
			break;
		}
	}

	return is_covered;
}

static bool research_point_landed(struct kernel_grip_info *grip_info, uint32_t research_pos_bits, struct point_info *points, int id)
{
	int index = 0;
	bool ret = false;
	uint8_t pos = grip_info->points_pos[id];
	uint16_t debounce_ms = grip_info->large_corner_debounce_ms;

	for (index = 0; index < TOUCH_MAX_NUM; index++) {
		if (index == id || DOWN_POINT != grip_info->large_point_status[index]) {
			continue;
		}

		if ((0 == research_pos_bits) || ((research_pos_bits >> grip_info->points_pos[index]) & 0x01)) {       //judge the pos we need
			//For horizontal, we just judge half the size
			if ((POS_SHORT_LEFT == pos || POS_HORIZON_B_LEFT_CORNER == pos || POS_HORIZON_T_LEFT_CORNER == pos) && (points[index].y >= grip_info->max_y / 2)) {
				continue;
			}
			if ((POS_SHORT_RIGHT == pos || POS_HORIZON_B_RIGHT_CORNER == pos || POS_HORIZON_T_RIGHT_CORNER == pos) && (points[index].y < grip_info->max_y / 2)) {
				continue;
			}
			if (grip_info->points_pos[index] != pos && (grip_info->first_point[index].time_ms > grip_info->first_point[id].time_ms - debounce_ms)) {
				ret = true;
				GRIP_TP_INFO("%s: id(%d) meet soon down point(%d: %d %d).\n", __func__, id, index, grip_info->first_point[index].x, grip_info->first_point[index].y);
				grip_info->grip_moni_data.research_point_landed_times++;
				break;
			} else {
				grip_info->grip_moni_data.research_point_landed_fail_times++;
			}
		}
	}

	return ret;
}

static void get_point_degree_check_thd(struct kernel_grip_info *grip_info, struct point_info *points, int index,
						uint16_t *edge_min_channel, uint16_t *corner_percent_thd, uint16_t *trx_corner_percent_thd) {
	uint8_t pos = grip_info->points_pos[index];
	s64 delta_time_ms = grip_info->uniform_make_up_point_last_down_time[index] - grip_info->first_point[index].time_ms;

	if ((POS_VERTICAL_LEFT_CORNER == pos) || (POS_VERTICAL_RIGHT_CORNER == pos) ||
		(POS_HORIZON_B_LEFT_CORNER == pos) || (POS_HORIZON_B_RIGHT_CORNER == pos) ||
		(POS_HORIZON_T_LEFT_CORNER == pos) || (POS_HORIZON_T_RIGHT_CORNER == pos)) {
			if (grip_info->is_curved_screen_v4_5 && grip_info->soon_meet_point_edge_degree_check_support \
					&& grip_info->soon_meet_level[index] > grip_info->soon_meet_min_level_thd) {
				*edge_min_channel = grip_info->soon_meet_bottom_corner_point_edge_min_channel_thd;
				*corner_percent_thd = grip_info->soon_meet_bottom_corner_point_edge_percent_thd;
				*trx_corner_percent_thd = grip_info->soon_meet_bottom_trx_corner_point_edge_percent_thd;
			} else if (grip_info->is_curved_screen_v4_5 && grip_info->center_down_point_edge_degree_check_support \
					&& grip_info->points_center_down[index] == STATUS_CENTER_DOWN) {
				*edge_min_channel = grip_info->center_down_bottom_corner_point_edge_min_channel_thd;
				*corner_percent_thd = grip_info->center_down_bottom_corner_point_edge_percent_thd;
				*trx_corner_percent_thd = grip_info->center_down_bottom_trx_corner_point_edge_percent_thd;
			} else if (grip_info->is_curved_screen_v4_5 && grip_info->long_press_point_edge_degree_check_support \
					&& delta_time_ms > grip_info->long_press_time_thd) {
				*edge_min_channel = grip_info->long_press_bottom_corner_point_edge_min_channel_thd;
				*corner_percent_thd = grip_info->long_press_bottom_corner_point_edge_percent_thd;
				*trx_corner_percent_thd = grip_info->long_press_bottom_trx_corner_point_edge_percent_thd;
			} else {
				*edge_min_channel = grip_info->corner_point_edge_min_channel_thd;
				*corner_percent_thd = grip_info->corner_point_edge_percent_thd;
				*trx_corner_percent_thd = grip_info->bottom_trx_corner_point_edge_percent_thd;
				if (grip_info->bottom_corner_point_edge_percent_thd > (*trx_corner_percent_thd) * (*trx_corner_percent_thd)) {
					*corner_percent_thd = grip_info->bottom_corner_point_edge_percent_thd;
				}
			}
	} else {
			if (grip_info->is_curved_screen_v4_5 && grip_info->soon_meet_point_edge_degree_check_support \
					&& grip_info->soon_meet_level[index] > grip_info->soon_meet_min_level_thd) {
				*edge_min_channel = grip_info->soon_meet_top_corner_point_edge_min_channel_thd;
				*corner_percent_thd = grip_info->soon_meet_top_corner_point_edge_percent_thd;
				*trx_corner_percent_thd = grip_info->soon_meet_top_trx_corner_point_edge_percent_thd;
			} else if (grip_info->is_curved_screen_v4_5 && grip_info->center_down_point_edge_degree_check_support \
					&& grip_info->points_center_down[index] == STATUS_CENTER_DOWN) {
				*edge_min_channel = grip_info->center_down_top_corner_point_edge_min_channel_thd;
				*corner_percent_thd = grip_info->center_down_top_corner_point_edge_percent_thd;
				*trx_corner_percent_thd = grip_info->center_down_top_trx_corner_point_edge_percent_thd;
			} else if (grip_info->is_curved_screen_v4_5 && grip_info->long_press_point_edge_degree_check_support \
					&& delta_time_ms > grip_info->long_press_time_thd) {
				*edge_min_channel = grip_info->long_press_top_corner_point_edge_min_channel_thd;
				*corner_percent_thd = grip_info->long_press_top_corner_point_edge_percent_thd;
				*trx_corner_percent_thd = grip_info->long_press_top_trx_corner_point_edge_percent_thd;
			} else {
				*edge_min_channel = grip_info->corner_point_edge_min_channel_thd;
				*corner_percent_thd = grip_info->corner_point_edge_percent_thd;
				*trx_corner_percent_thd = grip_info->top_trx_corner_point_edge_percent_thd;
			}
	}
}

/* for v4.3 start */
static bool point_edge_degree_check(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	struct point_info cur_p = points[index];
	uint16_t single_tx_pix = grip_info->max_x / grip_info->tx_num;
	uint16_t single_rx_pix = grip_info->max_y / grip_info->rx_num;
	uint16_t tmp_x = 0;
	uint16_t tmp_y = 0;
	uint16_t tmp_x_percent = 0;
	uint16_t tmp_y_percent = 0;
	uint16_t edge_min_channel = 0;
	uint16_t corner_percent_thd = 0;
	uint16_t trx_corner_percent_thd = 0;
	uint16_t x_coupling_result = 0;
	uint16_t y_coupling_result = 0;

	x_coupling_result = cur_p.rx_press * cur_p.rx_er;
	y_coupling_result = cur_p.tx_press * cur_p.tx_er;

	get_point_degree_check_thd(grip_info, points, index, &edge_min_channel, &corner_percent_thd, &trx_corner_percent_thd);

	if (grip_info->point_edge_degree_check_support && cur_p.rx_press > edge_min_channel && cur_p.tx_press > edge_min_channel) {
		tmp_x = cur_p.x > grip_info->max_x / 2 ? grip_info->max_x - cur_p.x : cur_p.x;
		tmp_y = cur_p.y > grip_info->max_y / 2 ? grip_info->max_y - cur_p.y : cur_p.y;

		tmp_x_percent = 100 * tmp_x / (cur_p.tx_press * single_tx_pix);
		tmp_y_percent = 100 * tmp_y / (cur_p.rx_press * single_rx_pix);

		if (((tmp_x_percent * tmp_x_percent + tmp_y_percent * tmp_y_percent < corner_percent_thd) ||
			(tmp_x_percent < trx_corner_percent_thd && tmp_y_percent < trx_corner_percent_thd)) &&
			(x_coupling_result > grip_info->corner_point_trx_radio_thd ||
			y_coupling_result > grip_info->corner_point_trx_radio_thd)) {
			TPD_DETAIL("%s: id(%d) percent:%d,%d coupling_result:(%d,%d) (thd:%d,%d,%d) point edge degree check matched.\n", __func__, index, \
				tmp_x_percent,
				tmp_y_percent,
				x_coupling_result,
				y_coupling_result,
				trx_corner_percent_thd,
				corner_percent_thd,
				grip_info->corner_point_trx_radio_thd);
			return true;
        }
	}
	return false;
}
/* for v4.3 end */

static bool research_point_landed_ver_v4(struct kernel_grip_info *grip_info, uint32_t research_pos_bits, struct point_info *points, int id)
{
	int index = 0;
	bool ret = false;
	uint8_t pos = grip_info->points_pos[id];
	uint16_t long_hold_debounce_ms = grip_info->current_data.large_long_debounce_ms;
	struct point_info cur_p = points[id];

	if (cur_p.x > grip_info->current_data.large_long_x1_width && (grip_info->max_x - cur_p.x) > grip_info->current_data.large_long_x1_width) {
		return ret;
	}

	for (index = 0; index < TOUCH_MAX_NUM; index++) {
		if (index == id || DOWN_POINT != grip_info->large_point_status[index]) {
			continue;
		}

		if ((0 == research_pos_bits) || ((research_pos_bits >> grip_info->points_pos[index]) & 0x01)) {       /*judge the pos we need*/
			/*For horizontal, we just judge half the size*/
			if ((POS_SHORT_LEFT == pos || POS_HORIZON_B_LEFT_CORNER == pos || POS_HORIZON_T_LEFT_CORNER == pos) && (points[index].y >= grip_info->max_y / 2)) {
				continue;
			}
			if ((POS_SHORT_RIGHT == pos || POS_HORIZON_B_RIGHT_CORNER == pos || POS_HORIZON_T_RIGHT_CORNER == pos) && (points[index].y < grip_info->max_y / 2)) {
				continue;
			}

			if ((abs(grip_info->first_point[index].y - cur_p.y) < grip_info->large_long_y2_width &&
				abs(grip_info->first_point[index].x - cur_p.x) < grip_info->large_long_x2_width)) {
				if (grip_info->points_pos[index] != pos && (grip_info->first_point[index].time_ms > grip_info->first_point[id].time_ms - long_hold_debounce_ms)) {
					ret = true;
					GRIP_TP_INFO("%s: id(%d) meet soon down point(%d: %d %d).\n", __func__, id, index, grip_info->first_point[index].x, grip_info->first_point[index].y);
					grip_info->grip_moni_data.research_point_landed_times++;
					break;
				}
			} else {
				grip_info->grip_moni_data.research_point_landed_fail_times++;
			}
		}
	}

	return ret;
}

static bool research_point_landed_hor_v4(struct kernel_grip_info *grip_info, uint32_t research_pos_bits, struct point_info *points, int id)
{
	int index = 0;
	bool ret = false;
	uint8_t pos = grip_info->points_pos[id];
	uint16_t debounce_ms = grip_info->large_corner_debounce_ms;

	for (index = 0; index < TOUCH_MAX_NUM; index++) {
		if (index == id || DOWN_POINT != grip_info->large_point_status[index]) {
			continue;
		}
		/*judge the pos we need*/
		if ((0 == research_pos_bits) || ((research_pos_bits >> grip_info->points_pos[index]) & 0x01)) {
			/*For horizontal, we just judge half the size*/
			if ((POS_SHORT_LEFT == pos || POS_HORIZON_B_LEFT_CORNER == pos || POS_HORIZON_T_LEFT_CORNER == pos) && (points[index].y >= grip_info->max_y / 2)) {
				continue;
			}
			if ((POS_SHORT_RIGHT == pos || POS_HORIZON_B_RIGHT_CORNER == pos || POS_HORIZON_T_RIGHT_CORNER == pos) && (points[index].y < grip_info->max_y / 2)) {
				continue;
			}
			if ((POS_HORIZON_B_LEFT_CORNER == pos) || (POS_HORIZON_B_RIGHT_CORNER == pos) ||
		   		(POS_HORIZON_T_LEFT_CORNER == pos) || (POS_HORIZON_T_RIGHT_CORNER == pos)) {
				if (grip_info->points_pos[index] != pos) {
					if (grip_info->corner_eliminate_without_time) {
						ret = true;
						grip_info->grip_moni_data.research_point_landed_times_force++;
						GRIP_TP_INFO("%s: id(%d) meet soon down point no time(%d: %d %d).\n", __func__, id,
							index, grip_info->first_point[index].x, grip_info->first_point[index].y);
						break;
					} else if (grip_info->first_point[index].time_ms > grip_info->first_point[id].time_ms - debounce_ms) {
						ret = true;
						grip_info->grip_moni_data.research_point_landed_times++;
						GRIP_TP_INFO("%s: id(%d) meet soon down point(%d: %d %d).\n", __func__, id, index, grip_info->first_point[index].x, grip_info->first_point[index].y);
						break;
					} else {
						grip_info->grip_moni_data.research_point_landed_fail_times++;
					}
				}
		   	} else {
				if (grip_info->points_pos[index] != pos && (grip_info->first_point[index].time_ms > grip_info->first_point[id].time_ms - debounce_ms)) {
					ret = true;
					GRIP_TP_INFO("%s: id(%d) meet soon down point(%d: %d %d).\n", __func__, id, index, grip_info->first_point[index].x, grip_info->first_point[index].y);
					break;
				} else {
					grip_info->grip_moni_data.research_point_landed_fail_times++;
				}
			}
		}
	}

	return ret;
}

static void disable_algo_for_ime_showing(struct kernel_grip_info *grip_info)
{
	if (grip_info->set_ime_showing == 1) {
		if (grip_info->long_eliminate_point_support) {
			grip_info->long_eliminate_point_restore = 1;
			grip_info->long_eliminate_point_support = 0;
		}
		if (grip_info->finger_hold_differ_size_support) {
			grip_info->finger_hold_differ_size_restore = 1;
			grip_info->finger_hold_differ_size_support = 0;
		}
		grip_info->large_corner_detect_time_ms_restore = grip_info->large_corner_detect_time_ms;
		grip_info->large_corner_detect_time_ms = grip_info->large_corner_detect_time_ms_ime;
	} else {
		if (grip_info->long_eliminate_point_restore) {
			grip_info->long_eliminate_point_restore = 0;
			grip_info->long_eliminate_point_support = 1;
		}
		if (grip_info->finger_hold_differ_size_restore) {
			grip_info->finger_hold_differ_size_restore = 0;
			grip_info->finger_hold_differ_size_support = 1;
		}
		grip_info->large_corner_detect_time_ms = grip_info->large_corner_detect_time_ms_restore;
	}
}

static uint8_t large_judge_pos(struct kernel_grip_info *grip_info, int index)
{
	uint8_t pos = POS_CENTER_INNER;
	struct grip_point_info fsp = grip_info->first_point[index];
	uint16_t width = grip_info->large_corner_width, height = grip_info->large_corner_height;
	uint16_t top_width = grip_info->large_top_width, top_height = grip_info->large_top_height;
	uint16_t hor_long_top_width = grip_info->large_hor_top_x_width, hor_long_top_height = grip_info->large_hor_top_y_height;
	uint16_t top_middle_width = grip_info->large_top_middle_width, top_middle_height = grip_info->large_top_middle_height;
	uint16_t top_middle_out_width = 0;
	uint16_t hor_bottom_x_width = grip_info->large_corner_hor_x_width, hor_bottom_y_height = grip_info->large_corner_hor_y_height;

	if (VERTICAL_SCREEN == grip_info->touch_dir) {
		if (fsp.y > grip_info->max_y - height) {                    //judge according to the range
			if (fsp.x < width) {
				return POS_VERTICAL_LEFT_CORNER;
			} else if (fsp.x > grip_info->max_x - width) {
				return POS_VERTICAL_RIGHT_CORNER;
			}
		}

		if ((0 != fsp.tx_press) && (0 != fsp.rx_press) && (fsp.y > grip_info->max_y / 2)) {         //judge according to the potential operation
			if (fsp.x < grip_info->max_x / 2) {
				return POS_VERTICAL_LEFT_CORNER;
			} else {
				return POS_VERTICAL_RIGHT_CORNER;
			}
		}

		if (fsp.y < top_height) {                    //judge according to the press time and move status
			if (fsp.x < top_width) {
				return POS_VERTICAL_LEFT_TOP;
			} else if (fsp.x > grip_info->max_x - top_width) {
				return POS_VERTICAL_RIGHT_TOP;
			}
		}

		if ((0 != fsp.tx_press) && (0 != fsp.rx_press) && (fsp.y < grip_info->max_y / 2)) {         //top corner
			if (fsp.x < grip_info->max_x / 2) {
				return POS_LONG_LEFT;
			} else {
				return POS_LONG_RIGHT;
			}
		}

		if (grip_info->is_curved_screen_v4 && (0 != fsp.tx_press) && fsp.y < top_middle_height) {
			top_middle_out_width = (grip_info->max_x - top_middle_width) / 2;
			if (fsp.x >= top_middle_out_width && fsp.x <= grip_info->max_x - top_middle_out_width) {
				return POS_VERTICAL_MIDDLE_TOP;
			}
		} else if (grip_info->is_curved_screen_v4 && grip_info->large_bottom_middle_support
			    && (0 != fsp.tx_press) && (fsp.y > grip_info->max_y - top_middle_height)) {
		    top_middle_out_width = (grip_info->max_x - top_middle_width) / 2;
			if (fsp.x >= top_middle_out_width && fsp.x <= grip_info->max_x - top_middle_out_width) {
				return POS_VERTICAL_MIDDLE_BOTTOM;
			}
		}
	} else if (LANDSCAPE_SCREEN_90 == grip_info->touch_dir) {
		if (fsp.x < height) {
			if (fsp.y < width) {
				return POS_HORIZON_B_LEFT_CORNER;
			} else if (fsp.y > grip_info->max_y - width) {
				return POS_HORIZON_B_RIGHT_CORNER;
			}
		}

		if (grip_info->is_curved_screen_v4) {
			if (fsp.x < hor_bottom_x_width) {
				if (fsp.y < hor_bottom_y_height) {
					return POS_HORIZON_B_LEFT_CORNER;
				} else if (fsp.y > grip_info->max_y - hor_bottom_y_height) {
					return POS_HORIZON_B_RIGHT_CORNER;
				}
			}
		}

		if ((0 != fsp.tx_press) && (0 != fsp.rx_press) && (fsp.x < grip_info->max_x / 2)) {         //judge according to the potential operation
			if (fsp.y < grip_info->max_y / 2) {
				return POS_HORIZON_B_LEFT_CORNER;
			} else {
				return POS_HORIZON_B_RIGHT_CORNER;
			}
		}

		if (fsp.x > grip_info->max_x - top_height) {
			if (fsp.y < top_width) {
				return POS_HORIZON_T_LEFT_TOP;
			} else if (fsp.y > grip_info->max_y - top_width) {
				return POS_HORIZON_T_RIGHT_TOP;
			}
		}

		if (grip_info->is_curved_screen_v4 && fsp.x > grip_info->max_x - hor_long_top_width) {
			if (fsp.y < hor_long_top_height) {
				return POS_HORIZON_T_LEFT_TOP;
			} else if (fsp.y > grip_info->max_y - hor_long_top_height) {
				return POS_HORIZON_T_RIGHT_TOP;
			}
		}
		if ((0 != fsp.tx_press) && (0 != fsp.rx_press) && (fsp.x > grip_info->max_x / 2)) {         //top corner
			if (fsp.y < grip_info->max_y / 2) {
				return POS_SHORT_LEFT;
			} else {
				return POS_SHORT_RIGHT;
			}
		}
	} else if (LANDSCAPE_SCREEN_270 == grip_info->touch_dir) {      //important message : pos is defined in horizontal 90
		if (fsp.x > grip_info->max_x - height) {
			if (fsp.y < width) {
				return POS_HORIZON_T_LEFT_CORNER;
			} else if (fsp.y > grip_info->max_y - width) {
				return POS_HORIZON_T_RIGHT_CORNER;
			}
		}

		if (grip_info->is_curved_screen_v4) {
			if (fsp.x > grip_info->max_x - hor_bottom_x_width) {
				if (fsp.y < hor_bottom_y_height) {
					return POS_HORIZON_T_LEFT_CORNER;
				} else if (fsp.y > grip_info->max_y - hor_bottom_y_height) {
					return POS_HORIZON_T_RIGHT_CORNER;
				}
			}
		}

		if ((0 != fsp.tx_press) && (0 != fsp.rx_press) && (fsp.x > grip_info->max_x / 2)) {         //judge according to the potential operation
			if (fsp.y < grip_info->max_y / 2) {
				return POS_HORIZON_T_LEFT_CORNER;
			} else {
				return POS_HORIZON_T_RIGHT_CORNER;
			}
		}

		if (fsp.x < top_height) {
			if (fsp.y < top_width) {
				return POS_HORIZON_B_LEFT_TOP;
			} else if (fsp.y > grip_info->max_y - top_width) {
				return POS_HORIZON_B_RIGHT_TOP;
			}
		}

		if (grip_info->is_curved_screen_v4 && fsp.x < hor_long_top_width) {
			if (fsp.y < hor_long_top_height) {
				return POS_HORIZON_B_LEFT_TOP;
			} else if (fsp.y > grip_info->max_y - hor_long_top_height) {
				return POS_HORIZON_B_RIGHT_TOP;
			}
		}
		if ((0 != fsp.tx_press) && (0 != fsp.rx_press) && (fsp.x < grip_info->max_x / 2)) {         //top corner
			if (fsp.y < grip_info->max_y / 2) {
				return POS_SHORT_LEFT;
			} else {
				return POS_SHORT_RIGHT;
			}
		}
	}	else if (VERTICAL_SCREEN_180 == grip_info->touch_dir) {
		if (fsp.y < height) {                    /*judge according to the range*/
			if (fsp.x < width) {
				return POS_VERTICAL_RIGHT_CORNER;
			} else if (fsp.x > grip_info->max_x - width) {
				return POS_VERTICAL_LEFT_CORNER;
			}
		}

		if ((0 != fsp.tx_press) && (0 != fsp.rx_press) && (fsp.y < grip_info->max_y / 2)) {         /*judge according to the potential operation*/
			if (fsp.x < grip_info->max_x / 2) {
				return POS_VERTICAL_RIGHT_CORNER;
			} else {
				return POS_VERTICAL_LEFT_CORNER;
			}
		}

		if (fsp.y > grip_info->max_y - top_height) {                    /*judge according to the press time and move status*/
			if (fsp.x < top_width) {
				return POS_VERTICAL_RIGHT_TOP;
			} else if (fsp.x > grip_info->max_x - top_width) {
				return POS_VERTICAL_LEFT_TOP;
			}
		}

		if ((0 != fsp.tx_press) && (0 != fsp.rx_press) && (fsp.y > grip_info->max_y / 2)) {         /*top corner*/
			if (fsp.x < grip_info->max_x / 2) {
				return POS_LONG_RIGHT;
			} else {
				return POS_LONG_LEFT;
			}
		}

		if (grip_info->is_curved_screen_v4 && (0 != fsp.tx_press) && (fsp.y > grip_info->max_y -top_middle_height)) { /*middle top*/
			top_middle_out_width = (grip_info->max_x - top_middle_width) / 2;
			if (fsp.x >= top_middle_out_width && fsp.x <= grip_info->max_x - top_middle_out_width) {
				return POS_VERTICAL_MIDDLE_TOP;
			}
		} else if (grip_info->is_curved_screen_v4 && grip_info->large_bottom_middle_support
			    && (0 != fsp.tx_press) && (fsp.y < top_middle_height)) {
			top_middle_out_width = (grip_info->max_x - top_middle_width) / 2;
			if (fsp.x >= top_middle_out_width && fsp.x <= grip_info->max_x - top_middle_out_width) {
				return POS_VERTICAL_MIDDLE_BOTTOM;
			}
		}
	}

	if ((0 == fsp.tx_press) && (0 != fsp.rx_press)) {                   /*long side logic*/
		pos = fsp.x < grip_info->max_x / 2 ? POS_LONG_LEFT : POS_LONG_RIGHT;
	} else if ((0 != fsp.tx_press) && (0 == fsp.rx_press) && (VERTICAL_SCREEN != grip_info->touch_dir)
			&& (VERTICAL_SCREEN_180 != grip_info->touch_dir)) {            /*short side logic*/
		pos = fsp.y < grip_info->max_y / 2 ? POS_SHORT_LEFT : POS_SHORT_RIGHT;
	}

	return pos;
}

static uint8_t judge_center_down(struct kernel_grip_info *grip_info, struct point_info *points, int id)
{
	int index = 0;
	uint8_t status = STATUS_CENTER_UP;
	uint8_t tmp = 0, pos = grip_info->points_pos[id];

	if (POS_CENTER_INNER == pos) {          //set already down point
		for (index = 0; index < TOUCH_MAX_NUM; index++) {
			if (index == id || DOWN_POINT != grip_info->large_point_status[index]) {
				continue;
			}

			//For horizontal, we just judge half the size
			tmp = grip_info->points_pos[index];
			if ((POS_SHORT_LEFT == tmp || POS_HORIZON_B_LEFT_CORNER == tmp || POS_HORIZON_T_LEFT_CORNER == tmp) && (points[id].y >= grip_info->max_y / 2)) {
				continue;
			}
			if ((POS_SHORT_RIGHT == tmp || POS_HORIZON_B_RIGHT_CORNER == tmp || POS_HORIZON_T_RIGHT_CORNER == tmp) && (points[id].y < grip_info->max_y / 2)) {
				continue;
			}
			grip_info->points_center_down[index] = STATUS_CENTER_DOWN;
		}

		return status;
	}

	for (index = 0; index < TOUCH_MAX_NUM; index++) {
		if (index == id || DOWN_POINT != grip_info->large_point_status[index]) {
			continue;
		}

		//For horizontal, we just judge half the size
		if ((POS_SHORT_LEFT == pos || POS_HORIZON_B_LEFT_CORNER == pos || POS_HORIZON_T_LEFT_CORNER == pos) && (points[index].y >= grip_info->max_y / 2)) {
			continue;
		}
		if ((POS_SHORT_RIGHT == pos || POS_HORIZON_B_RIGHT_CORNER == pos || POS_HORIZON_T_RIGHT_CORNER == pos) && (points[index].y < grip_info->max_y / 2)) {
			continue;
		}
		if (POS_CENTER_INNER == grip_info->points_pos[index]) {                            //judge the pos we need
			status = STATUS_CENTER_DOWN;
			break;
		}
	}

	return status;
}

static uint8_t corner_shape_matched(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	uint8_t ret = CORNER_SHAPE_NONE;
	struct point_info cur_p = points[index];
	uint8_t pos = grip_info->points_pos[index];
	uint16_t trx_sum_value = 0, x_width = grip_info->single_channel_x_len, y_width = grip_info->single_channel_y_len;
	uint16_t trx_thd = grip_info->trx_reject_thd, rx_thd = grip_info->rx_reject_thd, tx_thd = grip_info->tx_reject_thd;

	if (grip_info->point_corner_er_min_limit_support && grip_info->is_curved_screen_v4_3) {
		if (cur_p.tx_er < grip_info->point_corner_tx_er_min_limit_thd && cur_p.rx_er < grip_info->point_corner_rx_er_min_limit_thd) {
			TPD_DETAIL("%s: id(%d) tx_er:%d,rx_er:%d thd:(%d,%d) point er is too small exit.\n", __func__, index, \
				cur_p.tx_er, cur_p.rx_er, grip_info->point_corner_tx_er_min_limit_thd, grip_info->point_corner_rx_er_min_limit_thd);
			return ret;
		}
	}

	if (STATUS_CENTER_DOWN == grip_info->points_center_down[index]) {
		rx_thd = grip_info->rx_strict_reject_thd;
		tx_thd = grip_info->tx_strict_reject_thd;
		trx_thd = grip_info->trx_strict_reject_thd;
	}

	trx_sum_value = cur_p.rx_press + cur_p.tx_press;
	if (cur_p.rx_press > rx_thd || cur_p.tx_press > tx_thd || trx_sum_value > trx_thd) {
		return CORNER_SHAPE_LARGE;
	}

	if (grip_info->is_curved_screen_v4_3) {
		if (point_edge_degree_check(grip_info, points, index)) {
			return CORNER_SHAPE_RATIO;
		}
	}

	if (POS_VERTICAL_LEFT_CORNER == pos || POS_VERTICAL_RIGHT_CORNER == pos) {
		if (cur_p.x > x_width && cur_p.x < grip_info->max_x - x_width) {
			if ((trx_sum_value >= rx_thd) && (trx_sum_value <= trx_thd) && (cur_p.rx_press >= 2 * cur_p.tx_press)) {
				ret = CORNER_SHAPE_RATIO;
			}
		} else {
			if (cur_p.rx_press > trx_thd / 2) {
				ret = CORNER_SHAPE_RATIO;
			}
		}
		return ret;
	}

	if (cur_p.y > y_width && cur_p.y < grip_info->max_y - y_width) {
		if ((trx_sum_value >= tx_thd) && (trx_sum_value <= trx_thd) && (cur_p.tx_press >= 2 * cur_p.rx_press)) {
			ret = CORNER_SHAPE_RATIO;
		}
	} else {
		if (cur_p.tx_press > trx_thd / 2) {
			ret = CORNER_SHAPE_RATIO;
		}
	}

	return ret;
}

static bool edge_sliding_exit_matched(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	struct point_info cur_p = points[index];
	struct grip_point_info first_p = grip_info->first_point[index];
	bool ret = false;

	if (cur_p.rx_er * cur_p.rx_press >= grip_info->current_data.edge_sliding_exit_yfsr_thd) {
		if (abs(cur_p.y - first_p.y) <= grip_info->current_data.edge_sliding_exit_distance) {
			grip_info->grip_moni_data.edge_sliding_exit_matched_times++;
			ret = false;
			return ret;
		} else {
			grip_info->grip_moni_data.edge_sliding_beyond_distance_times++;
		}
	}
	return true;
}

static bool corner_exit_matched(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	bool ret = false;
	struct point_info cur_p = points[index];
	uint8_t pos = grip_info->points_pos[index];
	uint16_t exit_dis = grip_info->large_corner_exit_distance;
	int16_t x_distance = 0, y_distance = 0, max_distance = 0;
	uint16_t xfsr_coupling_thd = grip_info->xfsr_corner_exit_thd, yfsr_coupling_thd = grip_info->yfsr_corner_exit_thd, exit_thd = grip_info->exit_match_thd;

	if ((POS_VERTICAL_LEFT_CORNER == pos) || (POS_HORIZON_B_RIGHT_CORNER == pos)) {
		x_distance = cur_p.x - grip_info->first_point[index].x;
		y_distance = grip_info->first_point[index].y - cur_p.y;
	} else if ((POS_VERTICAL_RIGHT_CORNER == pos) || (POS_HORIZON_T_RIGHT_CORNER == pos)) {
		x_distance = grip_info->first_point[index].x - cur_p.x;
		y_distance = grip_info->first_point[index].y - cur_p.y;
	} else if (POS_HORIZON_B_LEFT_CORNER == pos) {
		x_distance = cur_p.x - grip_info->first_point[index].x;
		y_distance = cur_p.y - grip_info->first_point[index].y;
	} else if (POS_HORIZON_T_LEFT_CORNER == pos) {
		x_distance = grip_info->first_point[index].x - cur_p.x;
		y_distance = cur_p.y - grip_info->first_point[index].y;
	}
	max_distance = x_distance > y_distance ? x_distance : y_distance;
	if ((STATUS_CENTER_DOWN != grip_info->points_center_down[index]) && (TYPE_CORNER_LARGE_SIZE != grip_info->large_finger_status[index]) && (TYPE_LONG_FINGER_HOLD != grip_info->large_finger_status[index])) {
		if (max_distance > exit_dis) {
			if (grip_info->is_curved_screen_v4 && grip_info->edge_sliding_matched_support) {
				if (!edge_sliding_exit_matched(grip_info, points, index)) {
					return false;
				}
			}
			GRIP_TP_INFO("%s: exit matched_0(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
			grip_info->exit_match_times[index]++;
			if (VERTICAL_SCREEN == grip_info->touch_dir) {
				grip_info->grip_moni_data.vertical_corner_exit_dis_match_times++;
			} else {
				grip_info->grip_moni_data.landscape_corner_exit_dis_match_times++;
			}
		}
	}

	if (STATUS_CENTER_DOWN == grip_info->points_center_down[index]) {
		if (xfsr_coupling_thd > grip_info->xfsr_strict_exit_thd) {
			xfsr_coupling_thd = grip_info->xfsr_strict_exit_thd;
		}
		if (yfsr_coupling_thd > grip_info->yfsr_strict_exit_thd) {
			yfsr_coupling_thd = grip_info->yfsr_strict_exit_thd;
		}
	}

	if ((cur_p.rx_er * cur_p.rx_press <= xfsr_coupling_thd) && (cur_p.tx_er * cur_p.tx_press <= yfsr_coupling_thd)) {
		GRIP_TP_INFO("%s: exit matched_1(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
		grip_info->exit_match_times[index]++;
		if (VERTICAL_SCREEN == grip_info->touch_dir) {
			grip_info->grip_moni_data.vertical_corner_press_exit_match_times++;
		} else {
			grip_info->grip_moni_data.landscape_corner_press_exit_match_times++;
		}
	}

	if (grip_info->exit_match_times[index] > exit_thd) {
		ret = true;
	}

	return ret;
}

static bool top_shape_matched_v4(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	struct point_info cur_p = points[index];
	uint8_t pos = grip_info->points_pos[index];
	uint16_t rx_thd = grip_info->top_matched_xfsr_thd;

	if ((POS_VERTICAL_LEFT_TOP != pos) && (POS_VERTICAL_RIGHT_TOP != pos)) {
		return false;
	}

	if (cur_p.rx_er > rx_thd) {
		grip_info->top_shape_match_times[index]++;
		if (grip_info->top_shape_match_times[index] >= grip_info->top_matched_times_thd) {
			grip_info->top_shape_match_times[index] = 0;
			return true;
		}
	}

	return false;
}

static bool top_exit_matched_v4(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	bool ret = false;
	struct point_info cur_p = points[index];
	int16_t x_distance = 0, y_distance = 0, max_distance = 0;
	uint16_t exit_dis = 0, exit_thd = grip_info->exit_match_thd;

	if (VERTICAL_SCREEN == grip_info->touch_dir || VERTICAL_SCREEN_180 == grip_info->touch_dir) {
	    exit_dis = grip_info->large_ver_top_exit_distance;
	} else {
	    exit_dis = grip_info->large_top_exit_distance;
	}

	x_distance = abs(cur_p.x - grip_info->first_point[index].x);
	y_distance = abs(cur_p.y - grip_info->first_point[index].y);
	max_distance = x_distance > y_distance ? x_distance : y_distance;
	if (max_distance > exit_dis) {
		GRIP_TP_INFO("%s: top exit matched(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
		grip_info->exit_match_times[index]++;
		grip_info->grip_moni_data.top_shape_exit_match_times++;
	}

	if (grip_info->exit_match_times[index] > exit_thd) {
		ret = true;
	}

	return ret;
}

static bool top_exit_matched(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	bool ret = false;
	struct point_info cur_p = points[index];
	int16_t x_distance = 0, y_distance = 0, max_distance = 0;
	uint16_t exit_dis = grip_info->large_top_exit_distance, exit_thd = grip_info->exit_match_thd;

	x_distance = abs(cur_p.x - grip_info->first_point[index].x);
	y_distance = abs(cur_p.y - grip_info->first_point[index].y);
	max_distance = x_distance > y_distance ? x_distance : y_distance;
	if (max_distance > exit_dis) {
		GRIP_TP_INFO("%s: top exit matched(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
		grip_info->exit_match_times[index]++;
	}

	if (grip_info->exit_match_times[index] > exit_thd) {
		ret = true;
	}

	return ret;
}

static bool large_shape_matched(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	struct point_info cur_p = points[index];
	uint16_t rx_thd = grip_info->rx_reject_thd, tx_thd = grip_info->tx_reject_thd;

	if (STATUS_CENTER_DOWN == grip_info->points_center_down[index]) {
		rx_thd = grip_info->rx_strict_reject_thd;
		tx_thd = grip_info->tx_strict_reject_thd;
	}

	if (cur_p.rx_press > rx_thd || cur_p.tx_press > tx_thd) {
		return true;
	}

	return false;
}

static bool middle_bottom_long_press_mached(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	struct point_info cur_p = points[index];
	uint16_t long_width_min_thd = grip_info->middle_bottom_long_press_width_min_thd;
	uint8_t long_match_times_thd = grip_info->middle_bottom_long_press_match_times_thd;
	uint16_t small_width_max_thd = grip_info->middle_bottom_small_press_width_max_thd;
	uint16_t small_er_min_thd = grip_info->middle_bottom_small_press_er_min_thd;
	uint8_t small_match_times_thd = grip_info->middle_bottom_small_press_match_times_thd;
	uint16_t long_press_time_thd = grip_info->middle_bottom_long_press_time_thd;
	int64_t delta_time_ms = grip_info->uniform_make_up_point_last_down_time[index] - grip_info->first_point[index].time_ms;
	uint16_t pos = grip_info->points_pos[index];

	if (pos != POS_VERTICAL_MIDDLE_BOTTOM) {
		return false;
	}

	if (!grip_info->is_curved_screen_v4_3 || !grip_info->large_middle_bottom_exit_quickly_support) {
		return false;
	}

	if (cur_p.tx_press >= long_width_min_thd) {
		grip_info->middle_bottom_long_press_match_times[index]++;
	} else {
		grip_info->middle_bottom_long_press_match_times[index] = 0;
	}

	if (grip_info->middle_bottom_long_press_match_times[index] > long_match_times_thd && delta_time_ms > long_press_time_thd) {
		TPD_INFO("%s: middle_bottom_long_press matched(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, \
				cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
		return true;
	}

	if (cur_p.tx_press <= small_width_max_thd && cur_p.tx_er > small_er_min_thd) {
		grip_info->middle_bottom_small_press_match_times[index]++;
	} else {
		grip_info->middle_bottom_small_press_match_times[index] = 0;
	}

	if (grip_info->middle_bottom_small_press_match_times[index] > small_match_times_thd && delta_time_ms > long_press_time_thd) {
		TPD_INFO("%s: middle_bottom_small_press matched(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, \
				cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
		return true;
	}

	return false;
}

static bool large_exit_matched_for_middle_bottom(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	bool ret = false;
	struct point_info cur_p = points[index];
	struct grip_point_info first_p = grip_info->first_point[index];
	uint16_t pos = grip_info->points_pos[index];
	uint16_t middle_match_x_dis_percent_thd = grip_info->middle_bottom_match_exit_x_dis_percent_thd;
	uint16_t middle_yfsr_coupling_thd = grip_info->middle_bottom_short_exit_yfsr_coupling_thd;
	uint16_t middle_yfsr_coupling_times_thd = grip_info->middle_bottom_exit_yfsr_coupling_times_thd;
	uint8_t middle_normal_press_thd = grip_info->middle_bottom_normal_press_thd;
	uint16_t single_tx_pix = grip_info->max_x / grip_info->tx_num;
	uint16_t x_dis_thd = grip_info->middle_bottom_exit_x_dis_thd;
	uint16_t y_dis_thd = grip_info->middle_bottom_exit_y_dis_thd;
	uint16_t yfsr_coupling = cur_p.tx_press * cur_p.tx_er;

	if (pos != POS_VERTICAL_MIDDLE_BOTTOM) {
		return ret;
	}

	if (!grip_info->is_curved_screen_v4_3 || !grip_info->large_middle_bottom_exit_quickly_support) {
		return false;
	}

	if (cur_p.tx_press > middle_normal_press_thd) {
		x_dis_thd = middle_match_x_dis_percent_thd * single_tx_pix * cur_p.tx_press / 100;
		y_dis_thd = grip_info->middle_bottom_match_exit_y_dis_thd;
		middle_yfsr_coupling_thd = grip_info->middle_bottom_long_exit_yfsr_coupling_thd;
	}

	if (yfsr_coupling <= middle_yfsr_coupling_thd) {
		grip_info->middle_bottom_exit_yfsr_coupling_times[index] += (middle_yfsr_coupling_thd / yfsr_coupling);
	} else {
		grip_info->middle_bottom_exit_yfsr_coupling_times[index] = 0;
	}
	if (grip_info->middle_bottom_exit_yfsr_coupling_times[index] > middle_yfsr_coupling_times_thd) {
		TPD_INFO("%s: middle_bottom_long_press yfsr_coupling matched(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, \
				cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
		ret = true;
		return ret;
	}

	if (abs(cur_p.y - first_p.y) > y_dis_thd || abs(cur_p.x - first_p.x) > x_dis_thd) {
		TPD_INFO("%s: middle_bottom_long_press exit move dis matched(%d) (%d %d %d %d) thd:(%d %d)", __func__, index, cur_p.x, cur_p.y, \
		first_p.x, first_p.y, x_dis_thd, y_dis_thd);
		ret = true;
		return ret;
	}

	return ret;
}

static void finger_max_rx_matched(struct kernel_grip_info *grip_info, struct point_info *points, int id)
{
	struct point_info cur_p = points[id];

	if (VERTICAL_SCREEN != grip_info->touch_dir && VERTICAL_SCREEN_180 != grip_info->touch_dir) {
		return;
	}

	if (grip_info->max_rx_matched[id]) {
		return;
	}

	if (cur_p.rx_press > grip_info->current_data.max_rx_rejec_thd) {
		grip_info->max_rx_matched_cnt[id]++;
		/*record point stable time*/
		if (grip_info->max_rx_matched_cnt[id] > 1) {
			grip_info->max_rx_stable_time[id] += ktime_to_ms(ktime_get()) - grip_info->last_frame_point[id].time_ms;
			if (grip_info->max_rx_stable_time[id] > grip_info->current_data.max_rx_stable_time_thd) {
				grip_info->max_rx_matched[id]++;
			}
		}
	} else {
		grip_info->max_rx_stable_time[id] = 0;
	}
	return;
}

/*find different size real time*/
static bool dynamic_finger_hold_matched(struct kernel_grip_info *grip_info, struct point_info *points, int id)
{
	int index = 0;
	uint8_t pos = grip_info->points_pos[id], tmp_pos = 0;
	struct point_info cur_p = points[id];
	uint16_t coord_range = grip_info->max_y / grip_info->long_hold_divided_factor;
	bool under_range = false;

	if (grip_info->dynamic_finger_hold_state[id] == 1) {
		return true;
	}

	if (cur_p.x > grip_info->current_data.dynamic_finger_hold_size_x
			&& abs(grip_info->max_x - cur_p.x) > grip_info->current_data.dynamic_finger_hold_size_x) {
		return false;
	}

	for (index = 0; index < TOUCH_MAX_NUM; index++) {
		if (index == id || DOWN_POINT != grip_info->large_point_status[index]) {
			continue;
		}

		if ((POS_LONG_LEFT != pos) && (POS_LONG_RIGHT != pos) && (POS_VERTICAL_LEFT_CORNER != pos) && (POS_VERTICAL_RIGHT_CORNER != pos)) {
			TP_DETAIL(grip_info->tp_index, "%s: should never get here.\n", __func__);
			continue;
		}

		tmp_pos = grip_info->points_pos[index];
		if ((tmp_pos == pos) ||
		    (POS_LONG_LEFT == pos && POS_VERTICAL_LEFT_CORNER == tmp_pos) ||
		    (POS_VERTICAL_LEFT_CORNER == pos && POS_LONG_LEFT == tmp_pos) ||
		    (POS_LONG_RIGHT == pos && POS_VERTICAL_RIGHT_CORNER == tmp_pos) ||
		    (POS_VERTICAL_RIGHT_CORNER == pos && POS_LONG_RIGHT == tmp_pos)) {
			under_range = abs(points[index].y - grip_info->first_point[id].y) < coord_range;

			if (under_range) {
				TP_DETAIL(grip_info->tp_index, "%s: id(%d) is matched as operate with point(%d).\n", __func__, id, index);
				grip_info->dynamic_finger_hold_state[id] = 1;
				return true;
			}
		}
	}
	return false;
}

static void dynamic_ctrl_swipe_exit(struct kernel_grip_info *grip_info, struct point_info *points, int index,
	uint16_t *edge_narrow_witdh, uint16_t *edge_exit_dis)
{
	uint16_t exit_thd = grip_info->exit_match_thd;
	#define MAX_RX_THRESHOLD 3

	/*find hold max rx match dynamic ctr swipe*/
	if (grip_info->finger_hold_max_rx_matched[index] > MAX_RX_THRESHOLD) {
		*edge_exit_dis = grip_info->current_data.finger_hold_max_rx_exit_distance;
		*edge_narrow_witdh = grip_info->current_data.finger_hold_max_rx_narrow_witdh;
	} else if (grip_info->dynamic_finger_hold_exit_support && grip_info->exit_match_times[index] > exit_thd - 1) {
		/*finger_hold_matched*/
		if (dynamic_finger_hold_matched(grip_info, points, index)) {
			*edge_exit_dis = grip_info->current_data.dynamic_finger_hold_exit_distance;
			*edge_narrow_witdh = grip_info->current_data.dynamic_finger_hold_narrow_witdh;
		} else {
			*edge_exit_dis = grip_info->edge_swipe_exit_distance;
			*edge_narrow_witdh = grip_info->edge_swipe_narrow_witdh;
		}
	} else if (grip_info->max_rx_matched_support && grip_info->max_rx_matched[index]) {
		/*max rx match dynamic ctr swipe*/
		*edge_exit_dis = grip_info->current_data.max_rx_exit_distance;
		*edge_narrow_witdh = grip_info->current_data.max_rx_narrow_witdh;
	} else {
		*edge_exit_dis = grip_info->edge_swipe_exit_distance;
		*edge_narrow_witdh = grip_info->edge_swipe_narrow_witdh;
	}
}

static bool large_exit_matched(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	bool ret = false;
	struct point_info cur_p = points[index];
	struct grip_point_info first_p = grip_info->first_point[index];
	uint16_t pos = grip_info->points_pos[index], exit_thd = grip_info->exit_match_thd;
	uint16_t xfsr_coupling_thd = grip_info->xfsr_normal_exit_thd, yfsr_coupling_thd = grip_info->yfsr_normal_exit_thd;
	uint16_t edge_narrow_witdh = grip_info->edge_swipe_narrow_witdh, edge_exit_dis = grip_info->edge_swipe_exit_distance;

	if (TYPE_LONG_FINGER_HOLD == grip_info->large_finger_status[index]) {
		xfsr_coupling_thd = grip_info->xfsr_hold_exit_thd;
		yfsr_coupling_thd = grip_info->yfsr_hold_exit_thd;
	}

	if (STATUS_CENTER_DOWN == grip_info->points_center_down[index]) {
		if (xfsr_coupling_thd > grip_info->xfsr_strict_exit_thd) {
			xfsr_coupling_thd = grip_info->xfsr_strict_exit_thd;
		}
		if (yfsr_coupling_thd > grip_info->yfsr_strict_exit_thd) {
			yfsr_coupling_thd = grip_info->yfsr_strict_exit_thd;
		}
	}

	if (grip_info->is_curved_screen_v4) {
		dynamic_ctrl_swipe_exit(grip_info, points, index, &edge_narrow_witdh, &edge_exit_dis);
	}

	if ((cur_p.tx_er * cur_p.tx_press <= yfsr_coupling_thd) && (cur_p.rx_er * cur_p.rx_press <= xfsr_coupling_thd)) {
		grip_info->exit_match_times[index]++;
		GRIP_TP_INFO("%s: exit matched(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
	}

	if (STATUS_CENTER_DOWN != grip_info->points_center_down[index]) {
		if ((POS_LONG_LEFT == pos || POS_LONG_RIGHT == pos) && (TYPE_PALM_LONG_SIZE != grip_info->large_finger_status[index]) && (TYPE_LONG_FINGER_HOLD != grip_info->large_finger_status[index])) {
			if (abs(cur_p.x - first_p.x) < edge_narrow_witdh && abs(cur_p.y - first_p.y) > edge_exit_dis) {
				if (grip_info->is_curved_screen_v4 && grip_info->edge_sliding_matched_support) {
					if (!edge_sliding_exit_matched(grip_info, points, index)) {
						return false;
					}
				}
				grip_info->exit_match_times[index]++;
				GRIP_TP_INFO("%s: exit matched_y(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
				if (VERTICAL_SCREEN == grip_info->touch_dir) {
					grip_info->grip_moni_data.vertical_exit_match_y_times++;
				} else {
					grip_info->grip_moni_data.landscape_exit_match_y_times++;
				}
			}
		}
		if ((POS_SHORT_LEFT == pos || POS_SHORT_RIGHT == pos) && (TYPE_PALM_SHORT_SIZE != grip_info->large_finger_status[index])) {
			if (abs(cur_p.y - first_p.y) < edge_narrow_witdh && abs(cur_p.x - first_p.x) > edge_exit_dis) {
				grip_info->exit_match_times[index]++;
				GRIP_TP_INFO("%s: exit matched_x(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
				if (VERTICAL_SCREEN == grip_info->touch_dir) {
					grip_info->grip_moni_data.vertical_exit_match_x_times++;
				} else {
					grip_info->grip_moni_data.landscape_exit_match_x_times++;
				}
			}
		}
	}

	if (grip_info->exit_match_times[index] > exit_thd) {
		ret = true;
	}

	return ret;
}

static bool large_exit_matched_v4(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	bool ret = false;
	struct point_info cur_p = points[index];
	struct grip_point_info first_p = grip_info->first_point[index];
	uint16_t pos = grip_info->points_pos[index], exit_thd = grip_info->exit_match_thd;
	uint16_t xfsr_coupling_thd = grip_info->xfsr_normal_exit_thd, yfsr_coupling_thd = grip_info->yfsr_normal_exit_thd;
	uint16_t edge_narrow_witdh = grip_info->edge_swipe_narrow_witdh, edge_exit_dis = grip_info->edge_swipe_exit_distance;


	if (TYPE_LONG_FINGER_HOLD == grip_info->large_finger_status[index]) {
		xfsr_coupling_thd = grip_info->xfsr_hold_exit_thd;
		yfsr_coupling_thd = grip_info->yfsr_hold_exit_thd;
	}

	if (STATUS_CENTER_DOWN == grip_info->points_center_down[index]) {
		if (xfsr_coupling_thd > grip_info->xfsr_strict_exit_thd) {
			xfsr_coupling_thd = grip_info->xfsr_strict_exit_thd;
		}
		if (yfsr_coupling_thd > grip_info->yfsr_strict_exit_thd) {
			yfsr_coupling_thd = grip_info->yfsr_strict_exit_thd;
		}
	}

	if ((cur_p.tx_er * cur_p.tx_press <= yfsr_coupling_thd) && (cur_p.rx_er * cur_p.rx_press <= xfsr_coupling_thd)) {
		grip_info->exit_match_times[index]++;
		GRIP_TP_INFO("%s: exit matched(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
	}

	if (STATUS_CENTER_DOWN != grip_info->points_center_down[index]) {
		if ((POS_LONG_LEFT == pos || POS_LONG_RIGHT == pos) &&
			(TYPE_PALM_LONG_SIZE != grip_info->large_finger_status[index]) &&
			(TYPE_LONG_FINGER_HOLD != grip_info->large_finger_status[index])) {
			if (abs(cur_p.x - first_p.x) < edge_narrow_witdh && abs(cur_p.y - first_p.y) > edge_exit_dis) {
				grip_info->exit_match_times[index]++;
				GRIP_TP_INFO("%s: exit matched_y(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
				if (VERTICAL_SCREEN == grip_info->touch_dir) {
					grip_info->grip_moni_data.vertical_exit_match_y_times++;
				} else {
					grip_info->grip_moni_data.landscape_exit_match_y_times++;
				}
			}
		}
		if ((POS_SHORT_LEFT == pos || POS_SHORT_RIGHT == pos) && (TYPE_PALM_SHORT_SIZE != grip_info->large_finger_status[index])) {
			if (abs(cur_p.y - first_p.y) < edge_narrow_witdh && abs(cur_p.x - first_p.x) > edge_exit_dis) {
				grip_info->exit_match_times[index]++;
				GRIP_TP_INFO("%s: exit matched_x(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
				if (VERTICAL_SCREEN == grip_info->touch_dir) {
					grip_info->grip_moni_data.vertical_exit_match_x_times++;
				} else {
					grip_info->grip_moni_data.landscape_exit_match_x_times++;
				}
			}
		}
		if ((POS_VERTICAL_MIDDLE_TOP == pos || POS_VERTICAL_MIDDLE_BOTTOM == pos) && (TYPE_PALM_SHORT_SIZE != grip_info->large_finger_status[index])) {
			if (abs(cur_p.x - first_p.x) < edge_narrow_witdh && abs(cur_p.y - first_p.y) > edge_exit_dis) {
				grip_info->exit_match_times[index]++;
				GRIP_TP_INFO("%s: exit matched_y(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
				if (VERTICAL_SCREEN == grip_info->touch_dir) {
					grip_info->grip_moni_data.vertical_exit_match_y_times++;
				} else {
					grip_info->grip_moni_data.landscape_exit_match_y_times++;
				}
			}
		}
	}

	if (grip_info->exit_match_times[index] > exit_thd) {
		ret = true;
	}

	return ret;
}

static bool finger_hold_matched(struct kernel_grip_info *grip_info, struct point_info *points, int id)
{
	int index = 0;
	uint8_t pos = grip_info->points_pos[id], tmp_pos = 0;
	bool under_range = false, under_time_interval = false;
	uint16_t coord_range = grip_info->max_y / grip_info->long_hold_divided_factor, time_intval = grip_info->long_hold_debounce_time_ms;

	if (VERTICAL_SCREEN != grip_info->touch_dir && VERTICAL_SCREEN_180 != grip_info->touch_dir) {
		return false;
	}

	for (index = 0; index < TOUCH_MAX_NUM; index++) {
		if (index == id || DOWN_POINT != grip_info->large_point_status[index]) {
			continue;
		}

		if ((POS_LONG_LEFT != pos) && (POS_LONG_RIGHT != pos) && (POS_VERTICAL_LEFT_CORNER != pos) && (POS_VERTICAL_RIGHT_CORNER != pos)) {
			GRIP_TP_INFO("%s: should never get here.\n", __func__);
			continue;
		}

		tmp_pos = grip_info->points_pos[index];
		if ((tmp_pos == pos) ||
		    (POS_LONG_LEFT == pos && POS_VERTICAL_LEFT_CORNER == tmp_pos) ||
		    (POS_VERTICAL_LEFT_CORNER == pos && POS_LONG_LEFT == tmp_pos) ||
		    (POS_LONG_RIGHT == pos && POS_VERTICAL_RIGHT_CORNER == tmp_pos) ||
		    (POS_VERTICAL_RIGHT_CORNER == pos && POS_LONG_RIGHT == tmp_pos)) {
			under_range = abs(grip_info->first_point[index].y - grip_info->first_point[id].y) < coord_range;
			under_time_interval = abs(grip_info->first_point[index].time_ms - grip_info->first_point[id].time_ms) < time_intval;

			if (((TYPE_LONG_FINGER_HOLD == grip_info->large_finger_status[index]) && under_range) || under_time_interval) {
				GRIP_TP_INFO("%s: id(%d) is matched as operate with point(%d).\n", __func__, id, index);
				grip_info->grip_moni_data.finger_hold_matched_times++;
				return true;
			} else {
				grip_info->grip_moni_data.finger_hold_matched_max_times++;
			}
		}
	}

	return false;
}

static bool finger_hold_matched_v4(struct kernel_grip_info *grip_info, struct point_info *points, int id)
{
	int index = 0;
	uint8_t pos = grip_info->points_pos[id], tmp_pos = 0;
	bool under_range = false, under_time_interval = false;
	bool is_long_hold_click_range = false;
	struct point_info cur_p = points[id];

	uint16_t coord_range = grip_info->max_y / grip_info->long_hold_divided_factor, time_intval = grip_info->long_hold_debounce_time_ms;

	if (!grip_info->finger_hold_matched_hor_support && VERTICAL_SCREEN != grip_info->touch_dir
		&& VERTICAL_SCREEN_180 != grip_info->touch_dir) {
		return false;
	}

	if (!grip_info->finger_hold_matched_hor_support && !grip_info->finger_hold_matched_ver_support) {
		return false;
	}

	for (index = 0; index < TOUCH_MAX_NUM; index++) {
		if (index == id || DOWN_POINT != grip_info->large_point_status[index]) {
			continue;
		}

		if ((POS_LONG_LEFT != pos) && (POS_LONG_RIGHT != pos) && (POS_VERTICAL_LEFT_CORNER != pos) && (POS_VERTICAL_RIGHT_CORNER != pos)) {
			GRIP_TP_INFO("%s: should never get here.\n", __func__);
			continue;
		}

		tmp_pos = grip_info->points_pos[index];
		if ((tmp_pos == pos) ||
		    (POS_LONG_LEFT == pos && POS_VERTICAL_LEFT_CORNER == tmp_pos) ||
		    (POS_VERTICAL_LEFT_CORNER == pos && POS_LONG_LEFT == tmp_pos) ||
		    (POS_LONG_RIGHT == pos && POS_VERTICAL_RIGHT_CORNER == tmp_pos) ||
		    (POS_VERTICAL_RIGHT_CORNER == pos && POS_LONG_RIGHT == tmp_pos)) {
			if (cur_p.rx_press > grip_info->current_data.finger_hold_rx_rejec_thd) {
				grip_info->finger_hold_max_rx_matched[id]++;
			}
			under_range = abs(grip_info->first_point[index].y - grip_info->first_point[id].y) < coord_range;
			under_time_interval = abs(grip_info->first_point[index].time_ms - grip_info->first_point[id].time_ms) < time_intval;

			if (grip_info->is_curved_screen_v4) {
				is_long_hold_click_range = ((abs(grip_info->first_point[index].x - grip_info->first_point[id].x) > grip_info->long_hold_x_width)
					&& (grip_info->first_point[id].y < grip_info->first_point[index].y));
				if (((TYPE_LONG_FINGER_HOLD == grip_info->large_finger_status[index]) && under_range) || (!is_long_hold_click_range && under_time_interval)) {
					GRIP_TP_INFO("%s: id(%d) is matched as operate with point(%d).\n", __func__, id, index);
					TPD_DEBUG("%s: under_range(%d) ,under_time_interval(%d), is_long_hold_click_range(%d).\n",
					__func__, under_range, under_time_interval, is_long_hold_click_range);
					grip_info->grip_moni_data.finger_hold_matched_times++;
					return true;
				} else {
					grip_info->grip_moni_data.finger_hold_matched_max_times++;
				}
			} else {
				if (((TYPE_LONG_FINGER_HOLD == grip_info->large_finger_status[index]) && under_range) || under_time_interval) {
					GRIP_TP_INFO("%s: id(%d) is matched as operate with point(%d).\n", __func__, id, index);
					return true;
				}
			}
		}
	}

	return false;
}

static bool finger_hold_different_size(struct kernel_grip_info *grip_info, struct point_info *points, int id)
{
	int index = 0;
	uint8_t pos = grip_info->points_pos[id], tmp_pos = 0;
	bool under_time_interval = false;
	struct point_info cur_p = points[id];
	uint8_t is_left_right_pos = 0;
	uint8_t is_left_right_temp_pos = 0;

	if (!grip_info->finger_hold_differ_hor_support && VERTICAL_SCREEN != grip_info->touch_dir
		&& VERTICAL_SCREEN_180 != grip_info->touch_dir) {
		return false;
	}

	if (cur_p.x > grip_info->current_data.finger_hold_differ_size_x && abs(grip_info->max_x - cur_p.x) > grip_info->current_data.finger_hold_differ_size_x) {
		return false;
	}

	for (index = 0; index < TOUCH_MAX_NUM; index++) {
		if (index == id || DOWN_POINT != grip_info->large_point_status[index]) {
			continue;
		}

		if ((POS_LONG_LEFT != pos) && (POS_LONG_RIGHT != pos)
			&& (POS_VERTICAL_LEFT_CORNER != pos) && (POS_VERTICAL_RIGHT_CORNER != pos)
			&& (POS_VERTICAL_LEFT_TOP != pos) && (POS_VERTICAL_RIGHT_TOP != pos)
			&& (POS_HORIZON_B_LEFT_CORNER != pos) && (POS_HORIZON_B_RIGHT_CORNER != pos)
			&& (POS_HORIZON_T_LEFT_CORNER != pos) && (POS_HORIZON_T_RIGHT_CORNER != pos)
			&& (POS_HORIZON_B_LEFT_TOP != pos) && (POS_HORIZON_B_RIGHT_TOP != pos)
			&& (POS_HORIZON_T_LEFT_TOP != pos) && (POS_HORIZON_T_RIGHT_TOP != pos)) {
			GRIP_TP_INFO("%s: should never get here.\n", __func__);
			continue;
		}

		tmp_pos = grip_info->points_pos[index];


		if (grip_info->first_point[index].x > grip_info->current_data.finger_hold_differ_size_x
			&& abs(grip_info->max_x - grip_info->first_point[index].x) > grip_info->current_data.finger_hold_differ_size_x) {
			break;
		}

		if (cur_p.x < grip_info->current_data.finger_hold_differ_size_x) {
			is_left_right_pos = 0;
		} else if (abs(grip_info->max_x - cur_p.x) < grip_info->current_data.finger_hold_differ_size_x) {
		    is_left_right_pos = 1;
		}

		if (grip_info->first_point[index].x < grip_info->current_data.finger_hold_differ_size_x) {
			is_left_right_temp_pos = 0;
		} else if (abs(grip_info->max_x - grip_info->first_point[index].x) < grip_info->current_data.finger_hold_differ_size_x) {
		    is_left_right_temp_pos = 1;
		}

		if (is_left_right_pos != is_left_right_temp_pos) {
			under_time_interval = abs(grip_info->first_point[index].time_ms - grip_info->first_point[id].time_ms) \
				< grip_info->current_data.finger_hold_differ_size_debounce_ms;
			if (under_time_interval) {
				TP_DETAIL(grip_info->tp_index, "%s: id(%d) is matched  with point(%d).\n", __func__, id, index);
				grip_info->grip_moni_data.finger_hold_differ_size_times++;
				return true;
			} else {
				grip_info->grip_moni_data.finger_hold_differ_max_times++;
			}
		}
	}

	return false;
}

//judge whether we match the large size for curved screen
static enum large_judge_status large_shape_judged_V2(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	struct point_info cur_p = points[index];
	uint8_t corner_result = 0, pos = grip_info->points_pos[index];
	enum large_judge_status judge_status = JUDGE_LARGE_CONTINUE;
	s64 delta_time_ms = ktime_to_ms(ktime_get()) - grip_info->first_point[index].time_ms;
	uint16_t long_stable_coupling_thd = grip_info->current_data.long_stable_coupling_thd, short_stable_coupling_thd = grip_info->short_stable_coupling_thd;
	uint16_t x_coupling_result = 0, y_coupling_result = 0, startx_coupling_result = 0, starty_coupling_result = 0, secondx_coupling_result = 0, secondy_coupling_result = 0;
	int16_t tx_er_jitter_thd = 0;
	int16_t rx_er_jitter_thd = 0;
	int16_t tx_er_jitter = 0;
	int16_t rx_er_jitter = 0;
	int16_t tx_press_jitter = 0;
	int16_t rx_press_jitter = 0;

	//caculate current coupling result
	x_coupling_result = cur_p.rx_press * cur_p.rx_er;
	y_coupling_result = cur_p.tx_press * cur_p.tx_er;
	startx_coupling_result = grip_info->first_point[index].rx_press * grip_info->first_point[index].rx_er;
	starty_coupling_result = grip_info->first_point[index].tx_press * grip_info->first_point[index].tx_er;
	secondx_coupling_result = grip_info->second_point[index].rx_press * grip_info->second_point[index].rx_er;
	secondy_coupling_result = grip_info->second_point[index].tx_press * grip_info->second_point[index].tx_er;

	if ((POS_LONG_LEFT == pos) || (POS_LONG_RIGHT == pos)) {
		if (grip_info->is_curved_screen_v4 && grip_info->max_rx_matched_support) {
			finger_max_rx_matched(grip_info, points, index);
		}
		//judge the shape of long side
		if (large_shape_matched(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_PALM_LONG_SIZE;
			GRIP_TP_INFO("%s: id(%d) judge long shape matched.\n", __func__, index);
			return judge_status;
		}

		//judge whether it's reported around the large shape
		if (large_reject_covered(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_PALM_LONG_SIZE;
			GRIP_TP_INFO("%s: id(%d) judge around long large shape.\n", __func__, index);
			return judge_status;
		}

		/*judge whether we match finger hold*/
		if (grip_info->is_curved_screen_v4) {
			if (finger_hold_matched_v4(grip_info, points, index)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_LONG_FINGER_HOLD;
				GRIP_TP_INFO("%s: id(%d) judge long finger hold.\n", __func__, index);
				return judge_status;
			}
		} else if (finger_hold_matched(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_LONG_FINGER_HOLD;
			GRIP_TP_INFO("%s: id(%d) judge long finger hold.\n", __func__, index);
			return judge_status;
		}

		/*judge whether we match finger hold different size*/
		if (grip_info->is_curved_screen_v4 && grip_info->finger_hold_differ_size_support) {
			if (finger_hold_different_size(grip_info, points, index)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_LONG_FINGER_HOLD;
				TP_DETAIL(grip_info->tp_index, "%s: id(%d) judge finger_hold_different_size.\n", __func__, index);
				return judge_status;
			}
		}

		//judge the potential operation of center
		// if (research_point_landed(grip_info, 1 << POS_CENTER_INNER, points, index)) {
		// judge_status = JUDGE_LARGE_OK;
		// grip_info->large_finger_status[index] = TYPE_LONG_CENTER_DOWN;
		// grip_info->points_center_down[index] = STATUS_CENTER_DOWN;
		// TPD_INFO("%s: id(%d) judge long center down.\n", __func__, index);
		// return judge_status;
		// }

		/*judge the potential operation of center*/
		if (grip_info->is_curved_screen_v4 && grip_info->long_eliminate_point_support) {
			if (research_point_landed_ver_v4(grip_info, grip_info->long_eliminate_point_type, points, index)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_LONG_CENTER_DOWN;
				grip_info->points_center_down[index] = STATUS_CENTER_DOWN;
				GRIP_TP_INFO("%s: id(%d) judge long center down.\n", __func__, index);
				return judge_status;
			}
		}

		if (grip_info->is_curved_screen_v4_3) {
			if (point_edge_degree_check(grip_info, points, index)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_PALM_SHORT_SIZE;
				return judge_status;
			}
		}

		//judge whether we should exit the reject status
		if (large_exit_matched(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_TIMEOUT;
			GRIP_TP_INFO("%s: id(%d) judge long exit.\n", __func__, index);
			return judge_status;
		}

		//judge the stable status
		if (grip_info->point_stability_judgment_check_support && grip_info->is_curved_screen_v4_3) {
			rx_er_jitter_thd = (grip_info->point_stability_er_jitter_thd > grip_info->point_stability_er_jitter_percent_thd * cur_p.rx_er / 100) ? \
					grip_info->point_stability_er_jitter_thd : grip_info->point_stability_er_jitter_percent_thd * cur_p.rx_er / 100;
			rx_er_jitter = abs(grip_info->point_stability_rx_er_jitter_status[index] + cur_p.rx_er - grip_info->last_frame_point[index].rx_er);
			rx_press_jitter = abs(grip_info->point_stability_rx_press_jitter_status[index] + cur_p.rx_press - grip_info->last_frame_point[index].rx_press);

			grip_info->point_stability_rx_er_jitter_status[index] += cur_p.rx_er - grip_info->last_frame_point[index].rx_er;
			grip_info->point_stability_rx_press_jitter_status[index] += cur_p.rx_press - grip_info->last_frame_point[index].rx_press;
			if (1 != grip_info->frame_cnt[index] && rx_er_jitter < rx_er_jitter_thd && rx_press_jitter < grip_info->point_stability_press_jitter_thd) {
				grip_info->fsr_stable_time[index] += ktime_to_ms(ktime_get()) - grip_info->last_frame_point[index].time_ms;         /*record point stable time*/
			} else {
				grip_info->fsr_stable_time[index] = 0;
				grip_info->point_stability_rx_er_jitter_status[index] = 0;
				grip_info->point_stability_tx_er_jitter_status[index] = 0;
				grip_info->point_stability_rx_press_jitter_status[index] = 0;
				grip_info->point_stability_tx_press_jitter_status[index] = 0;
			}
		} else {
			if (x_coupling_result == grip_info->last_frame_point[index].rx_er * grip_info->last_frame_point[index].rx_press) {
				grip_info->fsr_stable_time[index] += ktime_to_ms(ktime_get()) - grip_info->last_frame_point[index].time_ms;         /*record point stable time*/
			} else {
				grip_info->fsr_stable_time[index] = 0;
			}
		}

		if (grip_info->fsr_stable_time[index] > grip_info->fsr_stable_time_thd) {
			if (STATUS_CENTER_DOWN == grip_info->points_center_down[index]) {
				long_stable_coupling_thd = grip_info->long_strict_stable_coupling_thd;
			}
			if (x_coupling_result > long_stable_coupling_thd) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_LONG_EDGE_FINGER;
				GRIP_TP_INFO("%s: id(%d) judge finger hold long edge screen.\n", __func__, index);
				return judge_status;
			}

			if (grip_info->is_curved_screen_v4_3) {
				if ((((abs(startx_coupling_result - secondx_coupling_result) < grip_info->current_data.long_hold_maxfsr_gap) &&
				    (startx_coupling_result > grip_info->current_data.long_start_coupling_thd)) ||
					(secondx_coupling_result > grip_info->current_data.long_start_coupling_thd)) &&
				    (startx_coupling_result - x_coupling_result > grip_info->current_data.long_hold_changed_thd)) {
					judge_status = JUDGE_LARGE_OK;
					grip_info->large_finger_status[index] = TYPE_LONG_FINGER_HOLD;
					GRIP_TP_INFO("%s: id(%d) judge finger hold long tight.\n", __func__, index);
				} else {
					judge_status = JUDGE_LARGE_TIMEOUT;
					GRIP_TP_INFO("%s: id(%d) judge long press under detect time.\n", __func__, index);
				}
			} else {
				if ((abs(startx_coupling_result - secondx_coupling_result) < grip_info->current_data.long_hold_maxfsr_gap) &&
				    (startx_coupling_result > grip_info->current_data.long_start_coupling_thd) &&
				    (startx_coupling_result - x_coupling_result > grip_info->current_data.long_hold_changed_thd)) {
					judge_status = JUDGE_LARGE_OK;
					grip_info->large_finger_status[index] = TYPE_LONG_FINGER_HOLD;
					GRIP_TP_INFO("%s: id(%d) judge finger hold long tight.\n", __func__, index);
				} else {
					judge_status = JUDGE_LARGE_TIMEOUT;
					GRIP_TP_INFO("%s: id(%d) judge long press under detect time.\n", __func__, index);
				}
			}

			return judge_status;
		}
		record_point_info(grip_info, TYPE_LAST_POINT, index, points[index]);
	} else if ((POS_SHORT_LEFT == pos) || (POS_SHORT_RIGHT == pos)) {
		//judge the shape of short side
		if (large_shape_matched(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_PALM_SHORT_SIZE;
			GRIP_TP_INFO("%s: id(%d) judge short shape matched.\n", __func__, index);
			return judge_status;
		}

		//judge whether it's reported around the large shape
		if (large_reject_covered(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_PALM_SHORT_SIZE;
			GRIP_TP_INFO("%s: id(%d) judge around short large shape.\n", __func__, index);
			return judge_status;
		}

		//judge the potential operation of center
		if (research_point_landed(grip_info, 1 << POS_CENTER_INNER, points, index)) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_SHORT_CENTER_DOWN;
			grip_info->points_center_down[index] = STATUS_CENTER_DOWN;
			GRIP_TP_INFO("%s: id(%d) judge short center down.\n", __func__, index);
			return judge_status;
		}

		if (grip_info->is_curved_screen_v4_3) {
			if (point_edge_degree_check(grip_info, points, index)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_PALM_SHORT_SIZE;
				return judge_status;
			}
		}

		//judge whether we should exit the reject status
		if (large_exit_matched(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_TIMEOUT;
			GRIP_TP_INFO("%s: id(%d) judge short exit.\n", __func__, index);
			return judge_status;
		}

		//judge the stable status

		if (grip_info->point_stability_judgment_check_support && grip_info->is_curved_screen_v4_3) {
			tx_er_jitter_thd = (grip_info->point_stability_er_jitter_thd > grip_info->point_stability_er_jitter_percent_thd * cur_p.tx_er / 100) ? \
				grip_info->point_stability_er_jitter_thd : grip_info->point_stability_er_jitter_percent_thd * cur_p.tx_er / 100;
			tx_er_jitter = abs(grip_info->point_stability_tx_er_jitter_status[index] + cur_p.tx_er - grip_info->last_frame_point[index].tx_er);
			tx_press_jitter = abs(grip_info->point_stability_tx_press_jitter_status[index] + cur_p.tx_press - grip_info->last_frame_point[index].tx_press);

			grip_info->point_stability_tx_er_jitter_status[index] += cur_p.tx_er - grip_info->last_frame_point[index].tx_er;
			grip_info->point_stability_tx_press_jitter_status[index] += cur_p.tx_press - grip_info->last_frame_point[index].tx_press;
			if (1 != grip_info->frame_cnt[index] && tx_er_jitter < tx_er_jitter_thd && tx_press_jitter < grip_info->point_stability_press_jitter_thd) {
				grip_info->fsr_stable_time[index] += ktime_to_ms(ktime_get()) - grip_info->last_frame_point[index].time_ms;         /*record point stable time*/
			} else {
				grip_info->fsr_stable_time[index] = 0;
				grip_info->point_stability_rx_er_jitter_status[index] = 0;
				grip_info->point_stability_tx_er_jitter_status[index] = 0;
				grip_info->point_stability_rx_press_jitter_status[index] = 0;
				grip_info->point_stability_tx_press_jitter_status[index] = 0;
			}
		} else {
			if (y_coupling_result == grip_info->last_frame_point[index].tx_er * grip_info->last_frame_point[index].tx_press) {
				grip_info->fsr_stable_time[index] += ktime_to_ms(ktime_get()) - grip_info->last_frame_point[index].time_ms;         /*record point stable time*/
			} else {
				grip_info->fsr_stable_time[index] = 0;
			}
		}

		if (grip_info->fsr_stable_time[index] > grip_info->fsr_stable_time_thd) {
			if (STATUS_CENTER_DOWN == grip_info->points_center_down[index]) {
				short_stable_coupling_thd = grip_info->short_strict_stable_coupling_thd;
			}
			if (y_coupling_result > short_stable_coupling_thd) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_SHORT_EDGE_FINGER;
				GRIP_TP_INFO("%s: id(%d) judge finger hold short edge screen.\n", __func__, index);
				return judge_status;
			}

			if ((abs(starty_coupling_result - secondy_coupling_result) < grip_info->short_hold_maxfsr_gap) &&
			    (starty_coupling_result > grip_info->short_stable_coupling_thd) &&
			    (starty_coupling_result - y_coupling_result > grip_info->short_hold_changed_thd)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_SHORT_FINGER_HOLD;
				GRIP_TP_INFO("%s: id(%d) judge finger hold short tight.\n", __func__, index);
			} else {
				judge_status = JUDGE_LARGE_TIMEOUT;
				GRIP_TP_INFO("large_shape_judged_V2: id(%d) judge short press under detect time.\n", index);
			}
			return judge_status;
		}
		record_point_info(grip_info, TYPE_LAST_POINT, index, points[index]);
	} else if ((POS_VERTICAL_LEFT_CORNER == pos) || (POS_VERTICAL_RIGHT_CORNER == pos) ||
		   (POS_HORIZON_B_LEFT_CORNER == pos) || (POS_HORIZON_B_RIGHT_CORNER == pos) ||
		   (POS_HORIZON_T_LEFT_CORNER == pos) || (POS_HORIZON_T_RIGHT_CORNER == pos)) {
		//judge the shape of corner
		if ((corner_result = corner_shape_matched(grip_info, points, index))) {
			judge_status = JUDGE_LARGE_OK;
			if (CORNER_SHAPE_LARGE == corner_result) {
				grip_info->large_finger_status[index] = TYPE_CORNER_LARGE_SIZE;
			} else {
				grip_info->large_finger_status[index] = TYPE_CORNER_SHAPE_SIZE;
			}
			GRIP_TP_INFO("%s: id(%d) judge corner shape matched(%d).\n", __func__, index, corner_result);
			return judge_status;
		}

		//judge whether we match finger hold for vertical corner
		if ((pos == POS_VERTICAL_LEFT_CORNER) || (pos == POS_VERTICAL_RIGHT_CORNER)) {
			if (grip_info->is_curved_screen_v4) {
				if (finger_hold_matched_v4(grip_info, points, index)) {
					judge_status = JUDGE_LARGE_OK;
					grip_info->large_finger_status[index] = TYPE_LONG_FINGER_HOLD;
					GRIP_TP_INFO("%s: id(%d) judge corner long finger hold.\n", __func__, index);
					return judge_status;
				}
			} else if (finger_hold_matched(grip_info, points, index)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_LONG_FINGER_HOLD;
				GRIP_TP_INFO("%s: id(%d) judge corner long finger hold.\n", __func__, index);
				return judge_status;
			}
		}

		/*judge whether we match finger hold different size*/
		if (grip_info->is_curved_screen_v4) {
			if (finger_hold_different_size(grip_info, points, index)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_LONG_FINGER_HOLD;
				TP_DETAIL(grip_info->tp_index, "%s: id(%d) judge finger_hold_different_size.\n", __func__, index);
				return judge_status;
			}
		}

		//judge the potential operation of center
		if (grip_info->is_curved_screen_v4 || grip_info->corner_eliminate_point_support) {
			if (research_point_landed_hor_v4(grip_info, grip_info->corner_eliminate_point_type, points, index)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_CORNER_CENTER_DOWN;
				grip_info->points_center_down[index] = STATUS_CENTER_DOWN;
				GRIP_TP_INFO("%s: id(%d) judge corner center down.\n", __func__, index);
				return judge_status;
			}
		} else {
			if (research_point_landed(grip_info, 1 << POS_CENTER_INNER, points, index)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_CORNER_CENTER_DOWN;
				grip_info->points_center_down[index] = STATUS_CENTER_DOWN;
				GRIP_TP_INFO("%s: id(%d) judge corner center down.\n", __func__, index);
				return judge_status;
			}
		}

		//judge whether we mistouch again
		if ((pos != POS_VERTICAL_LEFT_CORNER) && (pos != POS_VERTICAL_RIGHT_CORNER)) {
			if ((STATUS_CENTER_DOWN == grip_info->points_center_down[index]) && (TYPE_REJECT_DONE == grip_info->last_large_reject[index]) && (pos == grip_info->last_points_pos[index])) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_CORNER_MISTOUCH_AGAIN;
				GRIP_TP_INFO("%s: id(%d) judge corner mistouch again.\n", __func__, index);
				return judge_status;
			}
		}

		//judge the exit condition of corner
		if (corner_exit_matched(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_TIMEOUT;
			GRIP_TP_INFO("%s: id(%d) judge corner long move.\n", __func__, index);
			return judge_status;
		}

		//judge the stable status

		if (grip_info->point_stability_judgment_check_support && grip_info->is_curved_screen_v4_3) {
			rx_er_jitter_thd = (grip_info->point_stability_er_jitter_thd > grip_info->point_stability_er_jitter_percent_thd * cur_p.rx_er / 100) ? \
				grip_info->point_stability_er_jitter_thd : grip_info->point_stability_er_jitter_percent_thd * cur_p.rx_er / 100;
			rx_er_jitter = abs(grip_info->point_stability_rx_er_jitter_status[index] + cur_p.rx_er - grip_info->last_frame_point[index].rx_er);
			rx_press_jitter = abs(grip_info->point_stability_rx_press_jitter_status[index] + cur_p.rx_press - grip_info->last_frame_point[index].rx_press);

			grip_info->point_stability_rx_er_jitter_status[index] += cur_p.rx_er - grip_info->last_frame_point[index].rx_er;
			grip_info->point_stability_rx_press_jitter_status[index] += cur_p.rx_press - grip_info->last_frame_point[index].rx_press;

			tx_er_jitter_thd = (grip_info->point_stability_er_jitter_thd > grip_info->point_stability_er_jitter_percent_thd * cur_p.tx_er / 100) ? \
				grip_info->point_stability_er_jitter_thd : grip_info->point_stability_er_jitter_percent_thd * cur_p.tx_er / 100;
			tx_er_jitter = abs(grip_info->point_stability_tx_er_jitter_status[index] + cur_p.tx_er - grip_info->last_frame_point[index].tx_er);
			tx_press_jitter = abs(grip_info->point_stability_tx_press_jitter_status[index] + cur_p.tx_press - grip_info->last_frame_point[index].tx_press);

			grip_info->point_stability_tx_er_jitter_status[index] += cur_p.tx_er - grip_info->last_frame_point[index].tx_er;
			grip_info->point_stability_tx_press_jitter_status[index] += cur_p.tx_press - grip_info->last_frame_point[index].tx_press;
			if (1 != grip_info->frame_cnt[index] && rx_er_jitter < rx_er_jitter_thd && rx_press_jitter < grip_info->point_stability_press_jitter_thd \
				&& tx_er_jitter < tx_er_jitter_thd && tx_press_jitter < grip_info->point_stability_press_jitter_thd) {
				grip_info->fsr_stable_time[index] += ktime_to_ms(ktime_get()) - grip_info->last_frame_point[index].time_ms;         /*record point stable time*/
			} else {
				grip_info->fsr_stable_time[index] = 0;
				grip_info->point_stability_rx_er_jitter_status[index] = 0;
				grip_info->point_stability_tx_er_jitter_status[index] = 0;
				grip_info->point_stability_rx_press_jitter_status[index] = 0;
				grip_info->point_stability_tx_press_jitter_status[index] = 0;
			}
		} else {
			if ((y_coupling_result == grip_info->last_frame_point[index].tx_er * grip_info->last_frame_point[index].tx_press) &&
				(x_coupling_result == grip_info->last_frame_point[index].rx_er * grip_info->last_frame_point[index].rx_press)) {
				grip_info->fsr_stable_time[index] += ktime_to_ms(ktime_get()) - grip_info->last_frame_point[index].time_ms;         /*record point stable time*/
				if (grip_info->fsr_stable_time[index] > grip_info->fsr_stable_time_thd) {
					if ((starty_coupling_result <= grip_info->yfsr_corner_exit_thd) && (startx_coupling_result <= grip_info->xfsr_corner_exit_thd) &&
						(y_coupling_result <= grip_info->yfsr_corner_exit_thd) && (x_coupling_result <= grip_info->xfsr_corner_exit_thd)) {                    /*free long press*/
						judge_status = JUDGE_LARGE_TIMEOUT;
						TPD_INFO("%s: id(%d) judge stable touch.\n", __func__, index);
						return judge_status;
					}
				}
			} else {
				grip_info->fsr_stable_time[index] = 0;
			}
		}

		record_point_info(grip_info, TYPE_LAST_POINT, index, points[index]);

		//judge timeout, share the final result when stable
		if ((delta_time_ms > grip_info->large_corner_detect_time_ms) && (grip_info->fsr_stable_time[index] > grip_info->fsr_stable_time_thd)) {
			if ((x_coupling_result > grip_info->current_data.long_stable_coupling_thd) || (y_coupling_result > grip_info->short_stable_coupling_thd)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_CORNER_EDGE_FINGER;
				GRIP_TP_INFO("%s: id(%d) judge timeout, maybe finger hold corner edge screen.\n", __func__, index);
				return judge_status;
			}

			if ((STATUS_CENTER_DOWN == grip_info->points_center_down[index]) && !grip_info->point_unmoved[index]) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_CORNER_SHORT_MOVE;
				GRIP_TP_INFO("%s: id(%d) judge corner center down and short move.\n", __func__, index);
				return judge_status;
			}

			judge_status = JUDGE_LARGE_TIMEOUT;
			GRIP_TP_INFO("%s: id(%d) judge timeout.\n", __func__, index);
		}
	} else if ((POS_VERTICAL_LEFT_TOP == pos) || (POS_VERTICAL_RIGHT_TOP == pos) ||
		   (POS_HORIZON_B_LEFT_TOP == pos) || (POS_HORIZON_B_RIGHT_TOP == pos) ||
		   (POS_HORIZON_T_LEFT_TOP == pos) || (POS_HORIZON_T_RIGHT_TOP == pos)) {
		/*judge the exit condition of top corner*/
		if (grip_info->is_curved_screen_v4) {
			if (top_exit_matched_v4(grip_info, points, index)) {
				judge_status = JUDGE_LARGE_TIMEOUT;
				GRIP_TP_INFO("%s: id(%d) judge top long move.\n", __func__, index);
				return judge_status;
			}
		} else if (top_exit_matched(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_TIMEOUT;
			GRIP_TP_INFO("%s: id(%d) judge top long move.\n", __func__, index);
			return judge_status;
		}
		if (grip_info->is_curved_screen_v4 && top_shape_matched_v4(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_TOP_LONG_PRESS;
			GRIP_TP_INFO("%s: id(%d) reject top corner top_shape_matched_v4.\n", __func__, index);
			return judge_status;
		}

		if (delta_time_ms > grip_info->normal_tap_max_time_ms) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_TOP_LONG_PRESS;
			GRIP_TP_INFO("%s: id(%d) reject top corner long press.\n", __func__, index);
			return judge_status;
		}

		if (grip_info->is_curved_screen_v4_3) {
			if (point_edge_degree_check(grip_info, points, index)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_TOP_LONG_PRESS;
				return judge_status;
			}
		}
	} else if ((grip_info->is_curved_screen_v4 && POS_VERTICAL_MIDDLE_TOP == pos) ||
		(grip_info->is_curved_screen_v4 && POS_VERTICAL_MIDDLE_BOTTOM == pos)) {
		/*judge the shape of short side*/
		if (large_shape_matched(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_PALM_SHORT_SIZE;
			GRIP_TP_INFO("%s: id(%d) judge middle shape matched.\n", __func__, index);
			return judge_status;
		}

		/*judge whether it's reported around the large shape*/
		if (large_reject_covered(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_PALM_SHORT_SIZE;
			GRIP_TP_INFO("%s: id(%d) judge around short large shape.\n", __func__, index);
			return judge_status;
		}
		if (grip_info->is_curved_screen_v4_3 && middle_bottom_long_press_mached(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_OK;
			grip_info->large_finger_status[index] = TYPE_PALM_SHORT_SIZE;
			TPD_INFO("%s: id(%d) judge middle bottom long press mached.\n", __func__, index);
			return judge_status;
		}

		/*judge whether we should exit the reject status*/
		if (large_exit_matched_v4(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_TIMEOUT;
			GRIP_TP_INFO("%s: id(%d) judge middle shape exit.\n", __func__, index);
			return judge_status;
		}

		/*judge whether we should exit the reject status*/
		if (grip_info->is_curved_screen_v4_3 && large_exit_matched_for_middle_bottom(grip_info, points, index)) {
			judge_status = JUDGE_LARGE_TIMEOUT;
			TPD_INFO("%s: id(%d) judge for middle bottom exit.\n", __func__, index);
			return judge_status;
		}

		/*judge the stable status*/
		if (grip_info->point_stability_judgment_check_support && grip_info->is_curved_screen_v4_3) {
			tx_er_jitter_thd = (grip_info->point_stability_er_jitter_thd > grip_info->point_stability_er_jitter_percent_thd * cur_p.tx_er / 100) ? \
				grip_info->point_stability_er_jitter_thd : grip_info->point_stability_er_jitter_percent_thd * cur_p.tx_er / 100;
			tx_er_jitter = abs(grip_info->point_stability_tx_er_jitter_status[index] + cur_p.tx_er - grip_info->last_frame_point[index].tx_er);
			tx_press_jitter = abs(grip_info->point_stability_tx_press_jitter_status[index] + cur_p.tx_press - grip_info->last_frame_point[index].tx_press);

			grip_info->point_stability_tx_er_jitter_status[index] += cur_p.tx_er - grip_info->last_frame_point[index].tx_er;
			grip_info->point_stability_tx_press_jitter_status[index] += cur_p.tx_press - grip_info->last_frame_point[index].tx_press;
			if (1 != grip_info->frame_cnt[index] && tx_er_jitter < tx_er_jitter_thd && tx_press_jitter < grip_info->point_stability_press_jitter_thd) {
				grip_info->fsr_stable_time[index] += ktime_to_ms(ktime_get()) - grip_info->last_frame_point[index].time_ms;         /*record point stable time*/
			} else {
				grip_info->fsr_stable_time[index] = 0;
				grip_info->point_stability_rx_er_jitter_status[index] = 0;
				grip_info->point_stability_tx_er_jitter_status[index] = 0;
				grip_info->point_stability_rx_press_jitter_status[index] = 0;
				grip_info->point_stability_tx_press_jitter_status[index] = 0;
			}
		} else {
			if (y_coupling_result == grip_info->last_frame_point[index].tx_er * grip_info->last_frame_point[index].tx_press) {
				grip_info->fsr_stable_time[index] += ktime_to_ms(ktime_get()) - grip_info->last_frame_point[index].time_ms;         /*record point stable time*/
			} else {
				grip_info->fsr_stable_time[index] = 0;
			}
		}

		if (grip_info->fsr_stable_time[index] > grip_info->fsr_stable_time_thd) {
			if (STATUS_CENTER_DOWN == grip_info->points_center_down[index]) {
				short_stable_coupling_thd = grip_info->short_strict_stable_coupling_thd;
			}
			if (y_coupling_result > short_stable_coupling_thd) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_SHORT_EDGE_FINGER;
				GRIP_TP_INFO("%s: id(%d) judge finger hold vertical middle edge screen.\n", __func__, index);
				return judge_status;
			}

			if ((abs(starty_coupling_result - secondy_coupling_result) < grip_info->short_hold_maxfsr_gap) &&
			    (starty_coupling_result > grip_info->short_stable_coupling_thd) &&
			    (starty_coupling_result - y_coupling_result > grip_info->short_hold_changed_thd)) {
				judge_status = JUDGE_LARGE_OK;
				grip_info->large_finger_status[index] = TYPE_SHORT_FINGER_HOLD;
				GRIP_TP_INFO("%s: id(%d) judge finger hold vertical middle tight.\n", __func__, index);
			} else {
				judge_status = JUDGE_LARGE_TIMEOUT;
				GRIP_TP_INFO("large_shape_judged_V2: id(%d) judge vertical middle press under detect time.\n", index);
			}
			return judge_status;
		}
		record_point_info(grip_info, TYPE_LAST_POINT, index, points[index]);
	} else {
		judge_status = JUDGE_LARGE_TIMEOUT;
		GRIP_TP_INFO("%s: should never get here.\n", __func__);
	}

	return judge_status;
}

//judge whether we should exit current grip status
static bool large_area_judged_V2(struct kernel_grip_info *grip_info, struct point_info *points, int index)
{
	bool result = false;
	uint8_t corner_result = 0;
	struct point_info cur_p = points[index];
	uint8_t pos = grip_info->points_pos[index];

	if ((POS_LONG_LEFT == pos) || (POS_LONG_RIGHT == pos)) {
		if (large_shape_matched(grip_info, points, index)) {
			if ((TYPE_REJECT_DONE == grip_info->large_reject[index]) && (TYPE_PALM_LONG_SIZE != grip_info->large_finger_status[index])) {
				GRIP_TP_INFO("%s: long shape matched(%d) (%d %d %d %d %d %d)", __func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
				grip_info->large_finger_status[index] = TYPE_PALM_LONG_SIZE;
			}
			return false;
		}
		if (large_exit_matched(grip_info, points, index)) {
			result = true;
			GRIP_TP_INFO("%s: id(%d) judge long exit.\n", __func__, index);
		}
	} else if ((POS_SHORT_LEFT == pos) || (POS_SHORT_RIGHT == pos)) {
		if (large_shape_matched(grip_info, points, index)) {
			if ((TYPE_REJECT_DONE == grip_info->large_reject[index]) && (TYPE_PALM_SHORT_SIZE != grip_info->large_finger_status[index])) {
				GRIP_TP_INFO("%s: short shape matched(%d) (%d %d %d %d %d %d)", __func__,
					index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
				grip_info->large_finger_status[index] = TYPE_PALM_SHORT_SIZE;
			}
			return false;
		}
		if (large_exit_matched(grip_info, points, index)) {
			result = true;
			GRIP_TP_INFO("%s: id(%d) judge short exit.\n", __func__, index);
		}
	} else if ((POS_VERTICAL_LEFT_CORNER == pos) || (POS_VERTICAL_RIGHT_CORNER == pos) ||
		   (POS_HORIZON_B_LEFT_CORNER == pos) || (POS_HORIZON_B_RIGHT_CORNER == pos) ||
		   (POS_HORIZON_T_LEFT_CORNER == pos) || (POS_HORIZON_T_RIGHT_CORNER == pos)) {
		if ((corner_result = corner_shape_matched(grip_info, points, index))) {
			if ((CORNER_SHAPE_LARGE == corner_result) && (TYPE_CORNER_LARGE_SIZE != grip_info->large_finger_status[index])) {
				GRIP_TP_INFO("%s: id(%d) judge corner large size matched.\n", __func__, index);
				grip_info->large_finger_status[index] = TYPE_CORNER_LARGE_SIZE;
			}
			return false;
		}
		if (corner_exit_matched(grip_info, points, index)) {
			result = true;
			GRIP_TP_INFO("%s: id(%d) judge corner long move.\n", __func__, index);
		}
	} else if ((POS_VERTICAL_LEFT_TOP == pos) || (POS_VERTICAL_RIGHT_TOP == pos) ||
		   (POS_HORIZON_B_LEFT_TOP == pos) || (POS_HORIZON_B_RIGHT_TOP == pos) ||
		   (POS_HORIZON_T_LEFT_TOP == pos) || (POS_HORIZON_T_RIGHT_TOP == pos)) {
		if (top_exit_matched(grip_info, points, index)) {
			result = true;
			GRIP_TP_INFO("%s: id(%d) judge top move exit.\n", __func__, index);
		}
	} else if ((POS_VERTICAL_MIDDLE_TOP == pos) || (POS_VERTICAL_MIDDLE_BOTTOM == pos)) {
		if (grip_info->is_curved_screen_v4 && large_shape_matched(grip_info, points, index)) {
			if ((TYPE_REJECT_DONE == grip_info->large_reject[index]) && (TYPE_PALM_SHORT_SIZE != grip_info->large_finger_status[index])) {
				GRIP_TP_INFO("%s: middle top shape matched(%d) (%d %d %d %d %d %d)",
					__func__, index, cur_p.x, cur_p.y, cur_p.tx_press, cur_p.rx_press, cur_p.tx_er, cur_p.rx_er);
				grip_info->large_finger_status[index] = TYPE_PALM_SHORT_SIZE;
			}
			return false;
		}
		if (grip_info->is_curved_screen_v4 && large_exit_matched_v4(grip_info, points, index)) {
			result = true;
			GRIP_TP_INFO("%s: id(%d) judge middle top exit.\n", __func__, index);
		}
		if (grip_info->is_curved_screen_v4_3 && large_exit_matched_for_middle_bottom(grip_info, points, index)) {
			result = true;
			TPD_INFO("%s: id(%d) judge middle bottom exit.\n", __func__, index);
		}
	} else {
		result = true;
	}

	return result;
}

static bool touchup_judged_V2(struct kernel_grip_info *grip_info, int index)
{
	int ret = true;
	uint8_t pos = grip_info->points_pos[index];
	uint16_t startx_coupling_result = 0, starty_coupling_result = 0;
	uint16_t secondx_coupling_result = 0, secondy_coupling_result = 0;
	uint16_t long_start_coupling_thd = grip_info->current_data.long_start_coupling_thd;
	uint16_t short_start_coupling_thd = grip_info->short_start_coupling_thd;
	uint16_t long_hold_maxfsr_gap = grip_info->current_data.long_hold_maxfsr_gap, short_hold_maxfsr_gap = grip_info->short_hold_maxfsr_gap;
	s64 delta_time_ms = ktime_to_ms(ktime_get()) - grip_info->first_point[index].time_ms;
	int64_t touch_down_time_ms = grip_info->uniform_make_up_point_last_down_time[index] - grip_info->first_point[index].time_ms;

	if (delta_time_ms < grip_info->normal_tap_min_time_ms) {
		GRIP_TP_INFO("%s: id(%d) short click mistouch.\n", __func__, index);
		grip_info->large_finger_status[index] = TYPE_ALL_SHORT_CLICK;
		return false;
	}

	startx_coupling_result = grip_info->first_point[index].rx_press * grip_info->first_point[index].rx_er;
	starty_coupling_result = grip_info->first_point[index].tx_press * grip_info->first_point[index].tx_er;

	if (grip_info->one_frame_down_check_support && grip_info->is_curved_screen_v4_3 && grip_info->frame_cnt[index] == 1) {
		secondx_coupling_result = startx_coupling_result;
		secondy_coupling_result = starty_coupling_result;
	} else {
		secondx_coupling_result = grip_info->second_point[index].rx_press * grip_info->second_point[index].rx_er;
		secondy_coupling_result = grip_info->second_point[index].tx_press * grip_info->second_point[index].tx_er;
	}

	if ((POS_LONG_LEFT == pos) || (POS_LONG_RIGHT == pos)) {
		if ((STATUS_CENTER_DOWN == grip_info->points_center_down[index]) && !grip_info->point_unmoved[index]) {
			long_start_coupling_thd = grip_info->long_strict_start_coupling_thd;
		}
		if (((abs(startx_coupling_result - secondx_coupling_result) < long_hold_maxfsr_gap) && (startx_coupling_result > long_start_coupling_thd)) ||
		    (secondx_coupling_result > long_start_coupling_thd)) {
			GRIP_TP_INFO("%s: point(%d) up, maybe finger touch long edge screen.\n", __func__, index);
			grip_info->large_finger_status[index] = TYPE_LONG_EDGE_TOUCH;
			ret = false;
		}
	} else if ((POS_SHORT_LEFT == pos) || (POS_SHORT_RIGHT == pos)) {
		if ((STATUS_CENTER_DOWN == grip_info->points_center_down[index]) && !grip_info->point_unmoved[index]) {
			short_start_coupling_thd = grip_info->short_strict_start_coupling_thd;
		}
		if (((abs(starty_coupling_result - secondy_coupling_result) < short_hold_maxfsr_gap) && (starty_coupling_result > short_start_coupling_thd)) ||
		    (secondy_coupling_result > short_start_coupling_thd)) {
			GRIP_TP_INFO("%s: point(%d) up, maybe finger touch short edge screen.\n", __func__, index);
			grip_info->large_finger_status[index] = TYPE_SHORT_EDGE_TOUCH;
			ret = false;
		}
	} else if ((POS_VERTICAL_LEFT_CORNER == pos) || (POS_VERTICAL_RIGHT_CORNER == pos) ||
		   (POS_HORIZON_B_LEFT_CORNER == pos) || (POS_HORIZON_B_RIGHT_CORNER == pos) ||
		   (POS_HORIZON_T_LEFT_CORNER == pos) || (POS_HORIZON_T_RIGHT_CORNER == pos)) {
		if (((abs(startx_coupling_result - secondx_coupling_result) < long_hold_maxfsr_gap) && (startx_coupling_result > long_start_coupling_thd)) ||
		    (secondx_coupling_result > long_start_coupling_thd)) {
			GRIP_TP_INFO("%s: point(%d) up, maybe finger touch long corner edge screen.\n", __func__, index);
			grip_info->large_finger_status[index] = TYPE_LONG_EDGE_TOUCH;
			return false;
		}
		if (((abs(starty_coupling_result - secondy_coupling_result) < short_hold_maxfsr_gap) && (starty_coupling_result > short_start_coupling_thd)) ||
		    (secondy_coupling_result > short_start_coupling_thd)) {
			GRIP_TP_INFO("%s: point(%d) up, maybe finger touch short corner edge screen.\n", __func__, index);
			grip_info->large_finger_status[index] = TYPE_SHORT_EDGE_TOUCH;
			return false;
		}
		if (!grip_info->point_unmoved[index] && grip_info->corner_move_rejected) {
			GRIP_TP_INFO("%s: id(%d) judge corner short move.\n", __func__, index);
			grip_info->large_finger_status[index] = TYPE_CORNER_SHORT_MOVE;
			return false;
		}
	} else if ((POS_VERTICAL_LEFT_TOP == pos) || (POS_VERTICAL_RIGHT_TOP == pos) ||
		   (POS_HORIZON_B_LEFT_TOP == pos) || (POS_HORIZON_B_RIGHT_TOP == pos) ||
		   (POS_HORIZON_T_LEFT_TOP == pos) || (POS_HORIZON_T_RIGHT_TOP == pos)) {
		if (delta_time_ms > grip_info->normal_tap_max_time_ms) {
			GRIP_TP_INFO("%s: id(%d) reject top corner long press.\n", __func__, index);
			grip_info->large_finger_status[index] = TYPE_TOP_LONG_PRESS;
			return false;
		}
	} else if ((POS_VERTICAL_MIDDLE_TOP == pos) || (POS_VERTICAL_MIDDLE_BOTTOM == pos)) {
		if (grip_info->is_curved_screen_v4_3 && grip_info->large_middle_bottom_exit_quickly_support
			&& POS_VERTICAL_MIDDLE_BOTTOM == pos && (touch_down_time_ms < grip_info->middle_bottom_long_press_time_thd)) {
			ret = true;
			return ret;
		}
		if ((STATUS_CENTER_DOWN == grip_info->points_center_down[index]) && !grip_info->point_unmoved[index]) {
			short_start_coupling_thd = grip_info->short_strict_start_coupling_thd;
		}
		if (((abs(starty_coupling_result - secondy_coupling_result) < short_hold_maxfsr_gap) && (starty_coupling_result > short_start_coupling_thd)) ||
			(secondy_coupling_result > short_start_coupling_thd)) {
			GRIP_TP_INFO("%s: point(%d) up, maybe finger touch short middle edge screen.\n", __func__, index);
			grip_info->large_finger_status[index] = TYPE_SHORT_EDGE_TOUCH;
			ret = false;
		}
	} else {
		GRIP_TP_INFO("%s: should never get here.\n", __func__);
	}

	return ret;
}

static int curved_large_handle_V2(struct kernel_grip_info *grip_info, int obj_attention, struct point_info *points)
{
	int m_index = 0;
	bool exit_status = false;
	struct point_info tmp_point;
	int obj_final = obj_attention;
	uint16_t fiter_cnt = grip_info->coord_filter_cnt;
	enum large_judge_status judge_state = JUDGE_LARGE_CONTINUE;

	uint16_t uniform_max_make_up_frame = 5;

	for (m_index = 0; m_index < TOUCH_MAX_NUM; m_index++) {
		if (((obj_attention & TOUCH_BIT_CHECK) >> m_index) & 0x01) {      //finger down
			grip_info->frame_cnt[m_index]++;                              //count down frames
			grip_info->uniform_make_up_point_last_down_time[m_index] = ktime_to_ms(ktime_get());
			if (grip_info->uniform_make_up_point_v2_support && grip_info->is_curved_screen_v4_3) {
				if (grip_info->frame_cnt[m_index] >= 2) {
					grip_info->uniform_make_up_point_x_move_speed[m_index] = points[m_index].x - grip_info->uniform_make_up_last_input_point_x[m_index];
					grip_info->uniform_make_up_point_y_move_speed[m_index] = points[m_index].y - grip_info->uniform_make_up_last_input_point_y[m_index];
				}
				grip_info->uniform_make_up_last_input_point_x[m_index] = points[m_index].x;
				grip_info->uniform_make_up_last_input_point_y[m_index] = points[m_index].y;
			}

			if (grip_info->large_out_status[m_index]) {
				if (MAKEUP_REAL_POINT != grip_info->makeup_cnt[m_index]) {
					record_point_info(grip_info, TYPE_LATEST_POINT, m_index, points[m_index]);              //record latest different points
				}
				if (grip_info->uniform_make_up_point_v2_support && grip_info->is_curved_screen_v4_3) {
					if (grip_info->uniform_make_up_point_num_status[m_index] > 0) {
						get_makeup_point_xy(grip_info, m_index, &points[m_index]);
						 /* means that the point is not the first make up point */
						if (grip_info->uniform_make_up_point_num != (grip_info->uniform_make_up_point_num_status[m_index]) + 1) {
							grip_info->uniform_make_up_point_x_move_speed[m_index] = points[m_index].x - grip_info->uniform_last_make_up_prevent_out_x[m_index];
							grip_info->uniform_make_up_point_y_move_speed[m_index] = points[m_index].y - grip_info->uniform_last_make_up_prevent_out_y[m_index];
							TPD_DETAIL("id:%d makeup point speed:(%d,%d) point_in:(%d,%d) out:(%d,%d).make_up_prevent_out:(%d,%d)\n",
							    m_index,
								grip_info->uniform_make_up_point_x_move_speed[m_index], grip_info->uniform_make_up_point_y_move_speed[m_index], \
                                grip_info->uniform_make_up_last_input_point_x[m_index], grip_info->uniform_make_up_last_input_point_y[m_index], \
                                points[m_index].x,
								points[m_index].y,
								grip_info->uniform_last_make_up_prevent_out_x[m_index], grip_info->uniform_last_make_up_prevent_out_y[m_index]);
						}
						grip_info->uniform_last_make_up_prevent_out_x[m_index] = points[m_index].x;
						grip_info->uniform_last_make_up_prevent_out_y[m_index] = points[m_index].y;
						TPD_DETAIL("id:%d down status makeup point:(%d, %d) point num:%d.\n",
						    m_index,
							points[m_index].x, points[m_index].y,
							grip_info->uniform_make_up_point_num_status[m_index]);
					}
				} else if (grip_info->makeup_cnt[m_index] > 0) { /* means we need to continue to make up to the real point */
					if (grip_info->makeup_cnt[m_index] <= fiter_cnt) {
						tmp_point = points[m_index];
						assign_filtered_data(grip_info, m_index, &points[m_index]);
						add_filter_data_tail(grip_info, m_index, tmp_point);
						grip_info->makeup_cnt[m_index]++;
						GRIP_TP_INFO("id:%d makeup :%d times.(%d %d)\n", m_index, grip_info->makeup_cnt[m_index], points[m_index].x, points[m_index].y);
					} else {    //means we have make up to the real point
						grip_info->makeup_cnt[m_index] = MAKEUP_REAL_POINT;
					}
				}
				continue;
			}

			if (1 == grip_info->frame_cnt[m_index]) {                                        //init when first touch down
				grip_info->large_point_status[m_index] = DOWN_POINT;                         //set down status
				init_filter_data(grip_info, m_index, points[m_index]);
				init_latest_data(grip_info, m_index, points[m_index]);
				init_point_status(grip_info, m_index, &points[m_index]);
				record_point_info(grip_info, TYPE_START_POINT, m_index, points[m_index]);
				record_point_info(grip_info, TYPE_INIT_TX_POINT, m_index, points[m_index]);
				record_point_info(grip_info, TYPE_INIT_RX_POINT, m_index, points[m_index]);

				grip_info->points_pos[m_index] = large_judge_pos(grip_info, m_index);                               //judge the position of each id
				grip_info->points_center_down[m_index] = judge_center_down(grip_info, points, m_index);             //judge whether we have center point down already
				TPD_INFO("id:%d pos:%d center_down:%d (x:%u y:%u)(tp:%u rp:%u te:%u re:%u) \n",
					m_index,
					grip_info->points_pos[m_index],
					grip_info->points_center_down[m_index],
					points[m_index].x,
					points[m_index].y,
					points[m_index].tx_press,
					points[m_index].rx_press,
					points[m_index].tx_er,
					points[m_index].rx_er);
			} else if (2 == grip_info->frame_cnt[m_index]) {                                  //record second frame point info
				record_point_info(grip_info, TYPE_SECOND_POINT, m_index, points[m_index]);
			}

			grip_info->point_unmoved[m_index] = (abs(points[m_index].x - grip_info->first_point[m_index].x) <= 0 &&
							     abs(points[m_index].y - grip_info->first_point[m_index].y) <= 0);     //record move status
			record_point_info(grip_info, TYPE_LATEST_POINT, m_index, points[m_index]);              //record latest different points
			record_point_info(grip_info, TYPE_MAX_TX_POINT, m_index, points[m_index]);              //record points info of max tx channel
			record_point_info(grip_info, TYPE_MAX_RX_POINT, m_index, points[m_index]);              //record points info of max rx channel

			exit_status = large_area_judged_V2(grip_info, points, m_index);
			if (exit_status) {
				grip_info->large_out_status[m_index] = true;           //set large outside flag

				if (!grip_info->point_unmoved[m_index]) {    //means once in large judge area
					if (grip_info->uniform_make_up_point_v2_support && grip_info->is_curved_screen_v4_3) {
						uniform_makeup_xy_shift_initial(grip_info, m_index, &points[m_index]);
						grip_info->uniform_make_up_point_num_status[m_index] = grip_info->uniform_make_up_point_num;
						TPD_DETAIL("id:%d exit status makeup point:(%d, %d) point num:%d.\n",
							m_index,
							points[m_index].x, points[m_index].y,
							grip_info->uniform_make_up_point_num_status[m_index]);
					} else {
						tmp_point = points[m_index];
						assign_filtered_data(grip_info, m_index, &points[m_index]);
						add_filter_data_tail(grip_info, m_index, tmp_point);
						grip_info->makeup_cnt[m_index]++;
						GRIP_TP_INFO("id:%d makeup m:%d times.(%d %d)(%d %d %d %d)\n", m_index, grip_info->makeup_cnt[m_index], points[m_index].x, points[m_index].y,
							 points[m_index].tx_press, points[m_index].rx_press, points[m_index].tx_er, points[m_index].rx_er);
					}
				}
			} else if (TYPE_REJECT_DONE == grip_info->large_reject[m_index]) {
				obj_final = obj_final & (~(1 << m_index));                //if already reject, just mask it
			} else {
				TP_DETAIL(grip_info->tp_index, "id:%d, rx:%d, tx:%d, rx_er:%d(%d), tx_er:%d(%d). (%d %d)\n", m_index, points[m_index].rx_press, points[m_index].tx_press,
					points[m_index].rx_er, points[m_index].rx_er * points[m_index].rx_press, points[m_index].tx_er,
						points[m_index].tx_er * points[m_index].tx_press, points[m_index].x, points[m_index].y);
				judge_state = large_shape_judged_V2(grip_info, points, m_index);
				if (JUDGE_LARGE_OK == judge_state) {
					obj_final = obj_final & (~(1 << m_index));                //reject for large shape
					grip_info->large_reject[m_index] = TYPE_REJECT_DONE;
					if ((TYPE_PALM_LONG_SIZE == grip_info->large_finger_status[m_index]) || (TYPE_PALM_SHORT_SIZE == grip_info->large_finger_status[m_index])) {
						mask_potential_mistouch(grip_info, points, m_index);      //mask other seperate reported point around the large shape
					}
				} else if (JUDGE_LARGE_TIMEOUT == judge_state) {
					grip_info->large_out_status[m_index] = true;       //set outside flag
					if (!grip_info->point_unmoved[m_index]) {
						if (grip_info->uniform_make_up_point_v2_support && grip_info->is_curved_screen_v4_3) {
							uniform_makeup_xy_shift_initial(grip_info, m_index, &points[m_index]);
							grip_info->uniform_make_up_point_num_status[m_index] = grip_info->uniform_make_up_point_num;
							TPD_DETAIL("id:%d exit status makeup point:(%d, %d) point num:%d.\n",
							    m_index,
								points[m_index].x, points[m_index].y,
								grip_info->uniform_make_up_point_num_status[m_index]);
						} else {
							tmp_point = points[m_index];
							assign_filtered_data(grip_info, m_index, &points[m_index]);  /* makeup points */
							add_filter_data_tail(grip_info, m_index, tmp_point);
							grip_info->makeup_cnt[m_index]++;
							GRIP_TP_INFO("id:%d makeup n:%d times.(%d %d)(%d %d %d %d)\n",
							    m_index,
								grip_info->makeup_cnt[m_index],
								points[m_index].x, points[m_index].y,
							    points[m_index].tx_press, points[m_index].rx_press, points[m_index].tx_er, points[m_index].rx_er);
						}
					}
				} else {
					obj_final = obj_final & (~(1 << m_index));            //reject for continue detect
					grip_info->large_reject[m_index] = TYPE_REJECT_HOLD;
				}
			}
		} else { //finger up
			if (grip_info->large_point_status[m_index] == DOWN_POINT) {
				TP_DETAIL(grip_info->tp_index, "up id(%d) status: %d, %d, %d, %d.\n", m_index, grip_info->points_pos[m_index],
					grip_info->points_center_down[m_index], grip_info->large_out_status[m_index], grip_info->large_reject[m_index]);
			}

			if (grip_info->large_out_status[m_index]) {       //already exit the grip status
				TPD_DETAIL("large_out_status : grip_info->uniform_make_up_point_num_status[%d]:%d\n",
				m_index, grip_info->uniform_make_up_point_num_status[m_index]);
				if (grip_info->uniform_make_up_point_v2_support &&
				    grip_info->is_curved_screen_v4_3 &&
				    grip_info->uniform_make_up_point_num_status[m_index] > 0) {
					get_make_up_point_num(grip_info, m_index, &points[m_index]);
					if (grip_info->uniform_make_up_point_num_status[m_index] > 0) {
						points[m_index].status = 1;
						obj_final = obj_final | (1 << m_index);
						TPD_INFO("id:%d uniform_point_num:%d shift_leftover:%d,%d move_speed:%d,%d. \n",
						    m_index,
							grip_info->uniform_make_up_point_num_status[m_index],
							grip_info->uniform_make_up_x_shift_leftover[m_index], grip_info->uniform_make_up_y_shift_leftover[m_index],
							grip_info->uniform_make_up_point_x_move_speed[m_index], grip_info->uniform_make_up_point_y_move_speed[m_index]);

						uniform_max_make_up_frame = get_max_makeup_frame_num(grip_info, m_index, &points[m_index]);
						if (grip_info->uniform_make_up_point_num_status[m_index] > uniform_max_make_up_frame) {
							grip_info->uniform_make_up_point_num_status[m_index] = uniform_max_make_up_frame;
						}

						get_makeup_point_xy(grip_info, m_index, &points[m_index]);
						grip_info->uniform_make_up_point_upid_status |= (1 << m_index);
						TPD_DETAIL("id:%d, first up status makeup point:(%d, %d) point num:%d.\n",
						    m_index,
							points[m_index].x, points[m_index].y,
							grip_info->uniform_make_up_point_num_status[m_index]);
						start_makeup_timer(grip_info, m_index);
						grip_info->uniform_last_make_up_prevent_out_x[m_index] = points[m_index].x;
						grip_info->uniform_last_make_up_prevent_out_y[m_index] = points[m_index].y;
					}
				} else if (grip_info->makeup_cnt[m_index] > 0 && MAKEUP_REAL_POINT != grip_info->makeup_cnt[m_index]) {
					points[m_index].status = 1;
					obj_final = obj_final | (1 << m_index);
					if (grip_info->edge_swipe_makeup_optimization_support && grip_info->makeup_cnt[m_index] == 1) {
						points[m_index].x = (grip_info->coord_buf[(m_index + 1) * fiter_cnt - 1].x + grip_info->coord_buf[m_index * fiter_cnt].x) / 2;
						points[m_index].y = (grip_info->coord_buf[(m_index + 1) * fiter_cnt - 1].y + grip_info->coord_buf[m_index * fiter_cnt].y) / 2;
						GRIP_TP_INFO("id:%d makeup middle_point(%d %d) current_point(%d %d)\n", m_index, points[m_index].x, points[m_index].y,
							 grip_info->coord_buf[(m_index + 1) * fiter_cnt - 1].x, grip_info->coord_buf[(m_index + 1) * fiter_cnt - 1].y);
					} else {
						points[m_index].x = grip_info->coord_buf[(m_index + 1) * fiter_cnt - 1].x;
						points[m_index].y = grip_info->coord_buf[(m_index + 1) * fiter_cnt - 1].y;
						GRIP_TP_INFO("start makeup real point:%d(%d, %d) into fifo.\n", m_index, points[m_index].x, points[m_index].y);
					}

					grip_info->sync_up_makeup[m_index] = true;
					start_makeup_timer(grip_info, m_index);
				} else {
					TPD_DEBUG("no need makeup while exit grip status.\n");
				}
				grip_info->last_large_reject[m_index] = TYPE_REJECT_NONE;                   //update last status, should not update this id while no touch down
				grip_info->last_points_pos[m_index] = grip_info->points_pos[m_index];       //update last status, should not update this id while no touch down
				record_point_info(grip_info, TYPE_LAST_POINT, m_index, points[m_index]);    //update last status, should not update this id while no touch down
			} else if (TYPE_REJECT_HOLD == grip_info->large_reject[m_index]) {
				exit_status = touchup_judged_V2(grip_info, m_index);
				if (exit_status) {
					points[m_index].status = 1;
					obj_final = obj_final | (1 << m_index);
					grip_info->large_out_status[m_index] = true;

					if (grip_info->uniform_make_up_point_v2_support && grip_info->is_curved_screen_v4_3) {
						uniform_makeup_xy_shift_initial(grip_info, m_index, &points[m_index]);
						grip_info->uniform_make_up_point_upid_status |= (1 << m_index);

						get_make_up_point_num(grip_info, m_index, &points[m_index]);
						if (grip_info->uniform_make_up_point_num_status[m_index] < 2) {
							grip_info->uniform_make_up_point_num_status[m_index] = 2;
						}
						uniform_max_make_up_frame = get_max_makeup_frame_num(grip_info, m_index, &points[m_index]);
						if (grip_info->uniform_make_up_point_num_status[m_index] > uniform_max_make_up_frame) {
							grip_info->uniform_make_up_point_num_status[m_index] = uniform_max_make_up_frame;
						}
						TPD_INFO("id:%d uniform_point_num:%d shift_leftover:%d,%d move_speed:%d,%d. \n",
						    m_index,
							grip_info->uniform_make_up_point_num_status[m_index], grip_info->uniform_make_up_x_shift_leftover[m_index],
							grip_info->uniform_make_up_y_shift_leftover[m_index], grip_info->uniform_make_up_point_x_move_speed[m_index],
							grip_info->uniform_make_up_point_y_move_speed[m_index]);
						get_makeup_point_xy(grip_info, m_index, &points[m_index]);
						grip_info->uniform_make_up_point_upid_status |= (1 << m_index);
						start_makeup_timer(grip_info, m_index);
						TPD_DETAIL("id:%d, first up status makeup point:(%d, %d) point num:%d.\n",
						    m_index,
							points[m_index].x, points[m_index].y,
							grip_info->uniform_make_up_point_num_status[m_index]);
					} else {
						points[m_index].x = grip_info->coord_buf[m_index * fiter_cnt].x;
						points[m_index].y = grip_info->coord_buf[m_index * fiter_cnt].y;
						GRIP_TP_INFO("makeup start point:%d(%d, %d) into fifo.\n", m_index, points[m_index].x, points[m_index].y);
						grip_info->sync_up_makeup[m_index] = true;
						start_makeup_timer(grip_info, m_index);
					}
				} else {
					GRIP_TP_INFO("reject id:%d for accidental touch.\n", m_index);
				}
				grip_info->last_large_reject[m_index] = TYPE_REJECT_HOLD;                   //update last status, should not update this id while no touch down
				grip_info->last_points_pos[m_index] = grip_info->points_pos[m_index];       //update last status, should not update this id while no touch down
				record_point_info(grip_info, TYPE_LAST_POINT, m_index, points[m_index]);    //update last status, should not update this id while no touch down
			} else if (TYPE_REJECT_DONE == grip_info->large_reject[m_index]) {
				GRIP_TP_INFO("reject id:%d for large touch.\n", m_index);
				grip_info->last_large_reject[m_index] = TYPE_REJECT_DONE;                   //update last status, should not update this id while no touch down
				grip_info->last_points_pos[m_index] = grip_info->points_pos[m_index];       //update last status, should not update this id while no touch down
				record_point_info(grip_info, TYPE_LAST_POINT, m_index, points[m_index]);    //update last status, should not update this id while no touch down
			}

			//reset status of this id
			grip_info->frame_cnt[m_index] = 0;
			grip_info->large_out_status[m_index] = 0;
			grip_info->large_reject[m_index] = 0;
			grip_info->makeup_cnt[m_index] = 0;
			grip_info->point_unmoved[m_index] = 0;
			grip_info->exit_match_times[m_index] = 0;
			grip_info->large_finger_status[m_index] = 0;
			grip_info->fsr_stable_time[m_index] = 0;
			grip_info->points_pos[m_index] = 0;
			grip_info->points_center_down[m_index] = STATUS_CENTER_UNKNOW;
			grip_info->large_point_status[m_index] = UP_POINT;              //set up status
			grip_info->top_shape_match_times[m_index] = 0;
			grip_info->finger_hold_max_rx_matched[m_index] = 0;
			grip_info->max_rx_matched[m_index] = 0;
			grip_info->max_rx_matched_cnt[m_index] = 0;
			grip_info->max_rx_stable_time[m_index] = 0;
			grip_info->dynamic_finger_hold_state[m_index] = 0;
			grip_info->point_stability_tx_press_jitter_status[m_index] = 0;
			grip_info->point_stability_tx_er_jitter_status[m_index] = 0;
			grip_info->point_stability_rx_press_jitter_status[m_index] = 0;
			grip_info->point_stability_rx_er_jitter_status[m_index] = 0;
		}
	}

	grip_info->obj_prev_bit = obj_attention;
	return obj_final;
}

static uint8_t get_bit_count(int var)
{
	int m = 0, bits = 0;

	for (m = 0; m < 8 * sizeof(var); m++) {
		if ((var >> m) & 0x01) {
			bits++;
		}
	}

	return bits;
}

int notify_prevention_handle(struct kernel_grip_info *grip_info,
			     int obj_attention, struct point_info *points)
{
	int i = 0;

	if (!grip_info || !grip_info->p_ts || !points) {
		TPD_INFO("grip_info or points is null.\n");
		return obj_attention;
	}

	mutex_lock(&grip_info->grip_mutex);

	grip_info->obj_bit_rcd = obj_attention;
	if (!(grip_info->grip_disable_level & (1 << GRIP_DISABLE_LARGE))) {
		\
		if (grip_info->is_curved_screen) {
			obj_attention = curved_large_handle(grip_info, obj_attention, points);
		}  else if (grip_info->is_curved_screen_V2) {
			obj_attention = curved_large_handle_V2(grip_info, obj_attention, points);
		} else {
			obj_attention = large_condition_handle(grip_info, obj_attention, points);
		}
	}
	if (!(grip_info->grip_disable_level & (1 << GRIP_DISABLE_ELI))) {
		obj_attention = touch_elimination_handle(grip_info, obj_attention, points);
	}
	grip_info->obj_prced_bit_rcd = obj_attention;

	for (i = 0; i < TOUCH_MAX_NUM; i++) {
		if (grip_info->grip_hold_status[i]) {    /*handle hold id*/
			if (((grip_info->obj_prev_bit & TOUCH_BIT_CHECK) >> i) &
			    0x01) {     /*ID down from IC, report touch up firstly*/
				input_mt_slot(grip_info->p_ts->input_dev, i);
				input_mt_report_slot_state(grip_info->p_ts->input_dev, MT_TOOL_FINGER, 0);
				grip_status_reset(grip_info, i);        /*reset status of this ID*/

				if (grip_info->record_total_cnt) {
					grip_info->record_total_cnt--;        /*update touch down count*/
				}

				grip_info->grip_hold_status[i] = 0;
				GRIP_TP_INFO("id:%d report touch up firstly, left total(%d).\n", i,
					 grip_info->record_total_cnt);

			} else if (
				!grip_info->eli_reject_status[i]) {  /*aviod hold points be cleared by next down frame*/
				obj_attention = obj_attention | (1 <<
								 i);   /*point[i].status is 0, means no down and no up report*/

			} else {
				GRIP_TP_INFO("id:%d grip hold and reject by eli.\n", i);
			}
		}
	}

	if (!grip_info->record_total_cnt) {
		input_report_key(grip_info->p_ts->input_dev, BTN_TOUCH, 0);
		input_report_key(grip_info->p_ts->input_dev, BTN_TOOL_FINGER, 0);
	}

	input_sync(grip_info->p_ts->input_dev);

	mutex_unlock(&grip_info->grip_mutex);

	grip_info->record_total_cnt = get_bit_count(obj_attention & TOUCH_BIT_CHECK);
	return obj_attention;
}

//string size must be short than 64
static const struct key_addr key_addr_arrays[] = {
	{"large_corner_exit_distance",		(u64)&(((struct kernel_grip_info *)0)->large_corner_exit_distance)},
	{"large_corner_detect_time_ms",		(u64)&(((struct kernel_grip_info *)0)->large_corner_detect_time_ms)},
	{"large_corner_debounce_ms",		(u64)&(((struct kernel_grip_info *)0)->large_corner_debounce_ms)},
	{"large_corner_width",			(u64)&(((struct kernel_grip_info *)0)->large_corner_width)},
	{"large_corner_height",			(u64)&(((struct kernel_grip_info *)0)->large_corner_height)},
	{"xfsr_corner_exit_thd",		(u64)&(((struct kernel_grip_info *)0)->xfsr_corner_exit_thd)},
	{"yfsr_corner_exit_thd",		(u64)&(((struct kernel_grip_info *)0)->yfsr_corner_exit_thd)},
	{"exit_match_thd",			(u64)&(((struct kernel_grip_info *)0)->exit_match_thd)},
	{"trx_reject_thd",			(u64)&(((struct kernel_grip_info *)0)->trx_reject_thd)},
	{"rx_reject_thd",			(u64)&(((struct kernel_grip_info *)0)->rx_reject_thd)},
	{"tx_reject_thd",			(u64)&(((struct kernel_grip_info *)0)->tx_reject_thd)},
	{"fsr_stable_time_thd",			(u64)&(((struct kernel_grip_info *)0)->fsr_stable_time_thd)},
	{"single_channel_x_len",		(u64)&(((struct kernel_grip_info *)0)->single_channel_x_len)},
	{"single_channel_y_len",		(u64)&(((struct kernel_grip_info *)0)->single_channel_y_len)},
	{"normal_tap_min_time_ms",		(u64)&(((struct kernel_grip_info *)0)->normal_tap_min_time_ms)},
	{"normal_tap_max_time_ms",		(u64)&(((struct kernel_grip_info *)0)->normal_tap_max_time_ms)},
	{"long_start_coupling_thd",		(u64)&(((struct kernel_grip_info *)0)->normal_data.long_start_coupling_thd)},
	{"long_stable_coupling_thd",		(u64)&(((struct kernel_grip_info *)0)->normal_data.long_stable_coupling_thd)},
	{"long_detect_time_ms",			(u64)&(((struct kernel_grip_info *)0)->long_detect_time_ms)},
	{"long_hold_changed_thd",		(u64)&(((struct kernel_grip_info *)0)->normal_data.long_hold_changed_thd)},
	{"long_hold_maxfsr_gap",		(u64)&(((struct kernel_grip_info *)0)->normal_data.long_hold_maxfsr_gap)},
	{"long_hold_divided_factor",		(u64)&(((struct kernel_grip_info *)0)->long_hold_divided_factor)},
	{"long_hold_debounce_time_ms",		(u64)&(((struct kernel_grip_info *)0)->long_hold_debounce_time_ms)},
	{"xfsr_normal_exit_thd",		(u64)&(((struct kernel_grip_info *)0)->xfsr_normal_exit_thd)},
	{"yfsr_normal_exit_thd",		(u64)&(((struct kernel_grip_info *)0)->yfsr_normal_exit_thd)},
	{"xfsr_hold_exit_thd",			(u64)&(((struct kernel_grip_info *)0)->xfsr_hold_exit_thd)},
	{"yfsr_hold_exit_thd",			(u64)&(((struct kernel_grip_info *)0)->yfsr_hold_exit_thd)},
	{"large_reject_debounce_time_ms",	(u64)&(((struct kernel_grip_info *)0)->large_reject_debounce_time_ms)},
	{"report_updelay_ms",			(u64)&(((struct kernel_grip_info *)0)->report_updelay_ms)},
	{"short_start_coupling_thd",		(u64)&(((struct kernel_grip_info *)0)->short_start_coupling_thd)},
	{"short_stable_coupling_thd",		(u64)&(((struct kernel_grip_info *)0)->short_stable_coupling_thd)},
	{"short_hold_changed_thd",		(u64)&(((struct kernel_grip_info *)0)->short_hold_changed_thd)},
	{"short_hold_maxfsr_gap",		(u64)&(((struct kernel_grip_info *)0)->short_hold_maxfsr_gap)},
	{"large_top_width",			(u64)&(((struct kernel_grip_info *)0)->large_top_width)},
	{"large_top_height",			(u64)&(((struct kernel_grip_info *)0)->large_top_height)},
	{"large_top_exit_distance",		(u64)&(((struct kernel_grip_info *)0)->large_top_exit_distance)},
	{"edge_swipe_narrow_witdh",		(u64)&(((struct kernel_grip_info *)0)->edge_swipe_narrow_witdh)},
	{"edge_swipe_exit_distance",		(u64)&(((struct kernel_grip_info *)0)->edge_swipe_exit_distance)},
	{"long_strict_start_coupling_thd",	(u64)&(((struct kernel_grip_info *)0)->long_strict_start_coupling_thd)},
	{"long_strict_stable_coupling_thd",	(u64)&(((struct kernel_grip_info *)0)->long_strict_stable_coupling_thd)},
	{"trx_strict_reject_thd",		(u64)&(((struct kernel_grip_info *)0)->trx_strict_reject_thd)},
	{"rx_strict_reject_thd",		(u64)&(((struct kernel_grip_info *)0)->rx_strict_reject_thd)},
	{"tx_strict_reject_thd",		(u64)&(((struct kernel_grip_info *)0)->tx_strict_reject_thd)},
	{"short_strict_start_coupling_thd",	(u64)&(((struct kernel_grip_info *)0)->short_strict_start_coupling_thd)},
	{"short_strict_stable_coupling_thd",	(u64)&(((struct kernel_grip_info *)0)->short_strict_stable_coupling_thd)},
	{"xfsr_strict_exit_thd",		(u64)&(((struct kernel_grip_info *)0)->xfsr_strict_exit_thd)},
	{"yfsr_strict_exit_thd",		(u64)&(((struct kernel_grip_info *)0)->yfsr_strict_exit_thd)},
	{"corner_move_rejected",		(u64)&(((struct kernel_grip_info *)0)->corner_move_rejected)},
	{"grip_disable_level",		(u64)&(((struct kernel_grip_info *)0)->grip_disable_level)},
	{"long_hold_x_width",		(u64)&(((struct kernel_grip_info *)0)->long_hold_x_width)},
	{"long_hold_y_width",		(u64)&(((struct kernel_grip_info *)0)->long_hold_y_width)},
	{"finger_hold_matched_hor_support",		(u64)&(((struct kernel_grip_info *)0)->finger_hold_matched_hor_support)},
	{"finger_hold_matched_ver_support", 	(u64)&(((struct kernel_grip_info *)0)->finger_hold_matched_ver_support)},
	{"top_matched_times_thd",		(u64)&(((struct kernel_grip_info *)0)->top_matched_times_thd)},
	{"top_matched_xfsr_thd",		(u64)&(((struct kernel_grip_info *)0)->top_matched_xfsr_thd)},
	{"large_ver_top_exit_distance",		(u64)&(((struct kernel_grip_info *)0)->large_ver_top_exit_distance)},
	{"large_hor_long_top_width",		(u64)&(((struct kernel_grip_info *)0)->large_hor_top_x_width)},
	{"large_hor_long_top_height", 	(u64)&(((struct kernel_grip_info *)0)->large_hor_top_y_height)},
	{"large_top_middle_width",		(u64)&(((struct kernel_grip_info *)0)->large_top_middle_width)},
	{"large_top_middle_height", 	(u64)&(((struct kernel_grip_info *)0)->large_top_middle_height)},
	{"large_top_middle_exit_distance",		(u64)&(((struct kernel_grip_info *)0)->large_top_middle_exit_distance)},
	{"large_bottom_middle_support", 	(u64)&(((struct kernel_grip_info *)0)->large_bottom_middle_support)},
	{"corner_eliminate_point_type", 	(u64)&(((struct kernel_grip_info *)0)->corner_eliminate_point_type)},
	{"large_corner_hor_x_width", 	(u64)&(((struct kernel_grip_info *)0)->large_corner_hor_x_width)},
	{"large_corner_hor_y_height", 	(u64)&(((struct kernel_grip_info *)0)->large_corner_hor_y_height)},
	{"corner_eliminate_without_time", 	(u64)&(((struct kernel_grip_info *)0)->corner_eliminate_without_time)},
	{"long_eliminate_point_support",	(u64)&(((struct kernel_grip_info *)0)->long_eliminate_point_support)},
	{"long_eliminate_point_type",	(u64)&(((struct kernel_grip_info *)0)->long_eliminate_point_type)},
	{"large_long_x2_width",	(u64)&(((struct kernel_grip_info *)0)->large_long_x2_width)},
	{"large_long_y2_width",	(u64)&(((struct kernel_grip_info *)0)->large_long_y2_width)},
	{"large_long_x1_width",	(u64)&(((struct kernel_grip_info *)0)->normal_data.large_long_x1_width)},
	{"large_long_y1_width",	(u64)&(((struct kernel_grip_info *)0)->large_long_y1_width)},
	{"large_long_debounce_ms",	(u64)&(((struct kernel_grip_info *)0)->normal_data.large_long_debounce_ms)},
	{"finger_hold_differ_size_support", (u64)&(((struct kernel_grip_info *)0)->finger_hold_differ_size_support)},
	{"finger_hold_differ_hor_support", (u64)&(((struct kernel_grip_info *)0)->finger_hold_differ_hor_support)},
	{"finger_hold_differ_size_x", (u64)&(((struct kernel_grip_info *)0)->normal_data.finger_hold_differ_size_x)},
	{"finger_hold_differ_size_debounce_ms", (u64)&(((struct kernel_grip_info *)0)->normal_data.finger_hold_differ_size_debounce_ms)},
	{"set_ime_showing",         (u64)&(((struct kernel_grip_info *)0)->set_ime_showing)},
	{"finger_hold_rx_rejec_thd",		(u64)&(((struct kernel_grip_info *)0)->normal_data.finger_hold_rx_rejec_thd)},
	{"finger_hold_max_rx_exit_distance", 	(u64)&(((struct kernel_grip_info *)0)->normal_data.finger_hold_max_rx_exit_distance)},
	{"finger_hold_max_rx_narrow_witdh",		(u64)&(((struct kernel_grip_info *)0)->normal_data.finger_hold_max_rx_narrow_witdh)},
	{"max_rx_matched_support",		(u64)&(((struct kernel_grip_info *)0)->max_rx_matched_support)},
	{"max_rx_rejec_thd",		(u64)&(((struct kernel_grip_info *)0)->normal_data.max_rx_rejec_thd)},
	{"max_rx_stable_time_thd",		(u64)&(((struct kernel_grip_info *)0)->normal_data.max_rx_stable_time_thd)},
	{"max_rx_exit_distance",	(u64)&(((struct kernel_grip_info *)0)->normal_data.max_rx_exit_distance)},
	{"max_rx_narrow_witdh", 	(u64)&(((struct kernel_grip_info *)0)->normal_data.max_rx_narrow_witdh)},
	{"dynamic_finger_hold_exit_support",		(u64)&(((struct kernel_grip_info *)0)->dynamic_finger_hold_exit_support)},
	{"dynamic_finger_hold_exit_distance",		(u64)&(((struct kernel_grip_info *)0)->normal_data.dynamic_finger_hold_exit_distance)},
	{"dynamic_finger_hold_narrow_witdh",	(u64)&(((struct kernel_grip_info *)0)->normal_data.dynamic_finger_hold_narrow_witdh)},
	{"dynamic_finger_hold_size_x", 	(u64)&(((struct kernel_grip_info *)0)->normal_data.dynamic_finger_hold_size_x)},
	{"edge_sliding_matched_support",	(u64)&(((struct kernel_grip_info *)0)->edge_sliding_matched_support)},
	{"edge_sliding_exit_yfsr_thd", 	(u64)&(((struct kernel_grip_info *)0)->normal_data.edge_sliding_exit_yfsr_thd)},
	{"edge_sliding_exit_distance", 	(u64)&(((struct kernel_grip_info *)0)->normal_data.edge_sliding_exit_distance)},
	{"edge_swipe_makeup_optimization_support",	(u64)&(((struct kernel_grip_info *)0)->edge_swipe_makeup_optimization_support)},
	/*add for curved_screen_v4.2  recling mode begin*/
	{"large_long_x1_width_recli",	(u64)&(((struct kernel_grip_info *)0)->reclining_data.large_long_x1_width)},
	{"large_long_debounce_ms_recli",	(u64)&(((struct kernel_grip_info *)0)->reclining_data.large_long_debounce_ms)},
	{"finger_hold_differ_size_x_recli", (u64)&(((struct kernel_grip_info *)0)->reclining_data.finger_hold_differ_size_x)},
	{"finger_hold_differ_size_debounce_ms_recli", (u64)&(((struct kernel_grip_info *)0)->reclining_data.finger_hold_differ_size_debounce_ms)},
	{"finger_hold_rx_rejec_thd_recli",		(u64)&(((struct kernel_grip_info *)0)->reclining_data.finger_hold_rx_rejec_thd)},
	{"finger_hold_max_rx_exit_distance_recli", 	(u64)&(((struct kernel_grip_info *)0)->reclining_data.finger_hold_max_rx_exit_distance)},
	{"finger_hold_max_rx_narrow_witdh_recli",		(u64)&(((struct kernel_grip_info *)0)->reclining_data.finger_hold_max_rx_narrow_witdh)},
	{"max_rx_rejec_thd_recli",		(u64)&(((struct kernel_grip_info *)0)->reclining_data.max_rx_rejec_thd)},
	{"max_rx_stable_time_thd_recli",		(u64)&(((struct kernel_grip_info *)0)->reclining_data.max_rx_stable_time_thd)},
	{"max_rx_exit_distance_recli",	(u64)&(((struct kernel_grip_info *)0)->reclining_data.max_rx_exit_distance)},
	{"max_rx_narrow_witdh_recli", 	(u64)&(((struct kernel_grip_info *)0)->reclining_data.max_rx_narrow_witdh)},
	{"dynamic_finger_hold_exit_distance_recli",		(u64)&(((struct kernel_grip_info *)0)->reclining_data.dynamic_finger_hold_exit_distance)},
	{"dynamic_finger_hold_narrow_witdh_recli",	(u64)&(((struct kernel_grip_info *)0)->reclining_data.dynamic_finger_hold_narrow_witdh)},
	{"dynamic_finger_hold_size_x_recli", 	(u64)&(((struct kernel_grip_info *)0)->reclining_data.dynamic_finger_hold_size_x)},
	{"edge_sliding_exit_yfsr_thd_recli", 	(u64)&(((struct kernel_grip_info *)0)->reclining_data.edge_sliding_exit_yfsr_thd)},
	{"edge_sliding_exit_distance_recli", 	(u64)&(((struct kernel_grip_info *)0)->reclining_data.edge_sliding_exit_distance)},
	{"long_start_coupling_thd_recli", 	(u64)&(((struct kernel_grip_info *)0)->reclining_data.long_start_coupling_thd)},
	{"long_stable_coupling_thd_recli",		(u64)&(((struct kernel_grip_info *)0)->reclining_data.long_stable_coupling_thd)},
	{"long_hold_changed_thd_recli",		(u64)&(((struct kernel_grip_info *)0)->reclining_data.long_hold_changed_thd)},
	{"long_hold_maxfsr_gap_recli",		(u64)&(((struct kernel_grip_info *)0)->reclining_data.long_hold_maxfsr_gap)},
	/*add for curved_screen_v4.2  recling mode end*/
	/*add for curved_screen_v4.3  begin*/
	{"is_curved_screen_v4_3",	(u64)&(((struct kernel_grip_info *)0)->is_curved_screen_v4_3)},
	{"corner_major_minor_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->corner_major_minor_percent_thd)},
	{"corner_major_max_thd",	(u64)&(((struct kernel_grip_info *)0)->corner_major_max_thd)},
	{"corner_angle_max_range_thd",	(u64)&(((struct kernel_grip_info *)0)->corner_angle_max_range_thd)},
	{"point_edge_degree_check_support",	(u64)&(((struct kernel_grip_info *)0)->point_edge_degree_check_support)},
	{"corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->corner_point_edge_percent_thd)},
	{"bottom_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->bottom_corner_point_edge_percent_thd)},
	{"corner_point_edge_min_channel_thd",	(u64)&(((struct kernel_grip_info *)0)->corner_point_edge_min_channel_thd)},
	{"top_trx_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->top_trx_corner_point_edge_percent_thd)},
	{"bottom_trx_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->bottom_trx_corner_point_edge_percent_thd)},
	{"corner_point_trx_radio_thd",	(u64)&(((struct kernel_grip_info *)0)->corner_point_trx_radio_thd)},
	{"corner_point_check_frame_thd",	(u64)&(((struct kernel_grip_info *)0)->corner_point_check_frame_thd)},
	{"corner_point_peakdelta_stable_thd",	(u64)&(((struct kernel_grip_info *)0)->corner_point_peakdelta_stable_thd)},
	{"point_stability_judgment_check_support",	(u64)&(((struct kernel_grip_info *)0)->point_stability_judgment_check_support)},
	{"point_stability_press_jitter_thd",	(u64)&(((struct kernel_grip_info *)0)->point_stability_press_jitter_thd)},
	{"point_stability_er_jitter_thd",	(u64)&(((struct kernel_grip_info *)0)->point_stability_er_jitter_thd)},
	{"point_stability_er_jitter_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->point_stability_er_jitter_percent_thd)},
	{"point_corner_er_min_limit_support",	(u64)&(((struct kernel_grip_info *)0)->point_corner_er_min_limit_support)},
	{"point_corner_tx_er_min_limit_thd",	(u64)&(((struct kernel_grip_info *)0)->point_corner_tx_er_min_limit_thd)},
	{"point_corner_rx_er_min_limit_thd",	(u64)&(((struct kernel_grip_info *)0)->point_corner_rx_er_min_limit_thd)},
	{"one_frame_down_check_support",	(u64)&(((struct kernel_grip_info *)0)->one_frame_down_check_support)},
	{"uniform_make_up_point_v2_support",	(u64)&(((struct kernel_grip_info *)0)->uniform_make_up_point_v2_support)},
	{"uniform_make_up_point_num",	(u64)&(((struct kernel_grip_info *)0)->uniform_make_up_point_num)},
	{"uniform_make_up_exit_point_num",	(u64)&(((struct kernel_grip_info *)0)->uniform_make_up_exit_point_num)},
	{"uniform_max_make_up_time",	(u64)&(((struct kernel_grip_info *)0)->uniform_max_make_up_time)},
	{"uniform_make_up_last_point_percent_limit",	(u64)&(((struct kernel_grip_info *)0)->uniform_make_up_last_point_percent_limit)},
	/*add for curved_screen_v4.3  end*/
	/*add for curved_screen_v4.4  begin*/
	{"touch_point_stability_max_time",	(u64)&(((struct kernel_grip_info *)0)->touch_point_stability_max_time)},
	{"middle_bottom_long_press_time_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_long_press_time_thd)},
	{"middle_bottom_long_press_width_min_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_long_press_width_min_thd)},
	{"middle_bottom_long_press_minor_max_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_long_press_minor_max_thd)},
	{"middle_bottom_long_press_match_times_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_long_press_match_times_thd)},
	{"large_middle_bottom_exit_quickly_support",	(u64)&(((struct kernel_grip_info *)0)->large_middle_bottom_exit_quickly_support)},
	{"middle_bottom_small_press_width_max_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_small_press_width_max_thd)},
	{"middle_bottom_small_press_er_min_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_small_press_er_min_thd)},
	{"middle_bottom_small_press_peakdelta_max_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_small_press_peakdelta_max_thd)},
	{"middle_bottom_small_press_minor_max_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_small_press_minor_max_thd)},
	{"middle_bottom_small_press_match_times_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_small_press_match_times_thd)},
	{"middle_bottom_exit_x_dis_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_exit_x_dis_thd)},
	{"middle_bottom_exit_y_dis_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_exit_y_dis_thd)},
	{"middle_bottom_match_exit_x_dis_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_match_exit_x_dis_percent_thd)},
	{"middle_bottom_match_exit_y_dis_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_match_exit_y_dis_thd)},
	{"middle_bottom_short_exit_yfsr_coupling_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_short_exit_yfsr_coupling_thd)},
	{"middle_bottom_long_exit_yfsr_coupling_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_long_exit_yfsr_coupling_thd)},
	{"middle_bottom_normal_press_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_normal_press_thd)},
	{"middle_bottom_exit_yfsr_coupling_times_thd",	(u64)&(((struct kernel_grip_info *)0)->middle_bottom_exit_yfsr_coupling_times_thd)},
	/*add for curved_screen_v4.4  end*/
	/*add for curved_screen_v4.5  begin*/
	{"is_curved_screen_v4_5",	(u64)&(((struct kernel_grip_info *)0)->is_curved_screen_v4_5)},
	{"long_press_top_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->long_press_top_corner_point_edge_percent_thd)},
	{"long_press_bottom_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->long_press_bottom_corner_point_edge_percent_thd)},
	{"long_press_top_corner_point_edge_min_channel_thd",	(u64)&(((struct kernel_grip_info *)0)->long_press_top_corner_point_edge_min_channel_thd)},
	{"long_press_bottom_corner_point_edge_min_channel_thd",	(u64)&(((struct kernel_grip_info *)0)->long_press_bottom_corner_point_edge_min_channel_thd)},
	{"long_press_top_trx_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->long_press_top_trx_corner_point_edge_percent_thd)},
	{"long_press_bottom_trx_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->long_press_bottom_trx_corner_point_edge_percent_thd)},
	{"long_press_time_thd",	(u64)&(((struct kernel_grip_info *)0)->long_press_time_thd)},
	{"long_press_point_edge_degree_check_support",	(u64)&(((struct kernel_grip_info *)0)->long_press_point_edge_degree_check_support)},
	{"center_down_top_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->center_down_top_corner_point_edge_percent_thd)},
	{"center_down_bottom_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->center_down_bottom_corner_point_edge_percent_thd)},
	{"center_down_top_corner_point_edge_min_channel_thd",	(u64)&(((struct kernel_grip_info *)0)->center_down_top_corner_point_edge_min_channel_thd)},
	{"center_down_bottom_corner_point_edge_min_channel_thd",	(u64)&(((struct kernel_grip_info *)0)->center_down_bottom_corner_point_edge_min_channel_thd)},
	{"center_down_top_trx_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->center_down_top_trx_corner_point_edge_percent_thd)},
	{"center_down_bottom_trx_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->center_down_bottom_trx_corner_point_edge_percent_thd)},
	{"center_down_point_edge_degree_check_support",	(u64)&(((struct kernel_grip_info *)0)->center_down_point_edge_degree_check_support)},
	{"soon_meet_top_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->soon_meet_top_corner_point_edge_percent_thd)},
	{"soon_meet_bottom_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->soon_meet_bottom_corner_point_edge_percent_thd)},
	{"soon_meet_top_corner_point_edge_min_channel_thd",	(u64)&(((struct kernel_grip_info *)0)->soon_meet_top_corner_point_edge_min_channel_thd)},
	{"soon_meet_bottom_corner_point_edge_min_channel_thd",	(u64)&(((struct kernel_grip_info *)0)->soon_meet_bottom_corner_point_edge_min_channel_thd)},
	{"soon_meet_top_trx_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->soon_meet_top_trx_corner_point_edge_percent_thd)},
	{"soon_meet_bottom_trx_corner_point_edge_percent_thd",	(u64)&(((struct kernel_grip_info *)0)->soon_meet_bottom_trx_corner_point_edge_percent_thd)},
	{"soon_meet_min_level_thd",	(u64)&(((struct kernel_grip_info *)0)->soon_meet_min_level_thd)},
	{"soon_meet_point_edge_degree_check_support",	(u64)&(((struct kernel_grip_info *)0)->soon_meet_point_edge_degree_check_support)},
	/*add for curved_screen_v4.5  end*/
};

#define KEY_ADDR_NUMS sizeof(key_addr_arrays)/sizeof(struct key_addr)

static inline uint16_t get_key_addr_value(void *base, int index)
{
	return *(uint16_t *)(base + key_addr_arrays[index].offset);
}

static inline void set_key_addr_value(void *base, int index, uint16_t value)
{
	*(uint16_t *)(base + key_addr_arrays[index].offset) = value;
}

int kernel_grip_print_func(struct seq_file *s,
			   struct kernel_grip_info *grip_info)
{
	int in = 0;
	struct list_head *pos = NULL;
	struct grip_zone_area *grip_area = NULL;
	struct curved_judge_para *long_side_para = &grip_info->curved_long_side_para;
	struct curved_judge_para *short_side_para = &grip_info->curved_short_side_para;

	if (!grip_info) {
		TPD_INFO("%s read grip info failed.\n", __func__);
		return 0;
	}

	seq_printf(s, "grip_disable_level:%d\n", grip_info->grip_disable_level);
	seq_printf(s, "skip area:%d, %d, %d.\n", grip_info->no_handle_dir, grip_info->no_handle_y1, grip_info->no_handle_y2);

	if (grip_info->is_curved_screen_v4) {
		seq_printf(s, "filter count: %d\n", grip_info->coord_filter_cnt);
		seq_printf(s, "large V4 parameter:\n");
		for (in = 0; in < KEY_ADDR_NUMS; in++) {
			seq_printf(s, "%s: %d\n", key_addr_arrays[in].name, get_key_addr_value(grip_info, in));
		}
		seq_printf(s, "\n");
		return 0;
	} else if (grip_info->is_curved_screen_V2) {
		seq_printf(s, "filter count: %d\n", grip_info->coord_filter_cnt);
		seq_printf(s, "large V2 parameter:\n");
		for (in = 0; in < KEY_ADDR_NUMS; in++) {
			seq_printf(s, "%s: %d\n", key_addr_arrays[in].name, get_key_addr_value(grip_info, in));
		}
		seq_printf(s, "\n");
		return 0;
	}

	seq_printf(s, "dead zone:\n");
	list_for_each(pos, &grip_info->dead_zone_list) {
		grip_area = (struct grip_zone_area *)pos;

		seq_printf(s, "name:%24s, start_point:(%4d, %4d), width:(%4d, %4d), exit_thd:%4d, support_dir:0x%02x, side:0x%02x.\n",
			   grip_area->name, grip_area->start_x, grip_area->start_y, grip_area->x_width, grip_area->y_width,
			   grip_area->exit_thd, grip_area->support_dir, grip_area->grip_side);
	}
	seq_printf(s, "\n");

	seq_printf(s, "condition zone:\n");
	list_for_each(pos, &grip_info->condition_zone_list) {
		grip_area = (struct grip_zone_area *)pos;

		seq_printf(s, "name:%24s, start_point:(%4d, %4d), width:(%4d, %4d), exit_thd:%4d, support_dir:0x%02x, side:0x%02x.\n",
			   grip_area->name, grip_area->start_x, grip_area->start_y, grip_area->x_width, grip_area->y_width,
			   grip_area->exit_thd, grip_area->support_dir, grip_area->grip_side);
	}
	seq_printf(s, "condition_frame_limit:%4d, condition_updelay_ms:%4lu\n",
		   grip_info->condition_frame_limit, grip_info->condition_updelay_ms);
	seq_printf(s, "\n");

	seq_printf(s, "large zone:\n");
	list_for_each(pos, &grip_info->large_zone_list) {
		grip_area = (struct grip_zone_area *)pos;
		if (grip_info->is_curved_screen) {
			if (strstr(grip_area->name, "curved")) {
				seq_printf(s, "name:%24s, start_point:(%4d, %4d), width:(%4d, %4d), exit_thd:%4d, exit_tx_er:%4d, exit_rx_er:%4d, support_dir:0x%02x, side:0x%02x.\n",
					   grip_area->name, grip_area->start_x, grip_area->start_y, grip_area->x_width, grip_area->y_width, grip_area->exit_thd,
					   grip_area->exit_tx_er, grip_area->exit_rx_er, grip_area->support_dir, grip_area->grip_side);
			}
		} else {
			seq_printf(s, "name:%24s, start_point:(%4d, %4d), width:(%4d, %4d), exit_thd:%4d, support_dir:0x%02x, side:0x%02x.\n",
				   grip_area->name, grip_area->start_x, grip_area->start_y, grip_area->x_width, grip_area->y_width,
				   grip_area->exit_thd, grip_area->support_dir, grip_area->grip_side);
		}
	}
	seq_printf(s, "large_frame_limit:%4d, large_ver_thd:%4d, large_hor_thd:%4d\n",
		   grip_info->large_frame_limit, grip_info->large_ver_thd, grip_info->large_hor_thd);
	seq_printf(s, "large_corner_frame_limit:%4d, large_ver_corner_thd:%4d, large_hor_corner_thd:%4d\n\
large_ver_corner_width:%4d, large_hor_corner_width:%4d, large_corner_distance:%4d\n",
		   grip_info->large_corner_frame_limit, grip_info->large_ver_corner_thd, grip_info->large_hor_corner_thd,
		   grip_info->large_ver_corner_width, grip_info->large_hor_corner_width, grip_info->large_corner_distance);
	seq_printf(s, "\n");

	if (grip_info->is_curved_screen) {
		seq_printf(s, "large_detect_time_ms:%4d\n", grip_info->large_detect_time_ms);
		seq_printf(s, "down_delta_time_ms:%4lld\n", grip_info->down_delta_time_ms);
		seq_printf(s, "curved_large_judge_para_for_long_side:\n");
		seq_printf(s, "edge_finger_thd:%4d, hold_finger_thd:%4d, normal_finger_thd_1:%4d, normal_finger_thd_2:%4d, normal_finger_thd_3:%4d\n\
    large_palm_thd_1:%4d, large_palm_thd_2:%4d, palm_thd_1:%4d, palm_thd_2:%4d, small_palm_thd_1:%4d, small_palm_thd_2:%4d\n",
			   long_side_para->edge_finger_thd, long_side_para->hold_finger_thd, long_side_para->normal_finger_thd_1,
			   long_side_para->normal_finger_thd_2, long_side_para->normal_finger_thd_3, long_side_para->large_palm_thd_1, long_side_para->large_palm_thd_2,
			   long_side_para->palm_thd_1, long_side_para->palm_thd_2, long_side_para->small_palm_thd_1, long_side_para->small_palm_thd_2);
		seq_printf(s, "curved_large_judge_para_for_short_side:\n");
		seq_printf(s, "edge_finger_thd:%4d, hold_finger_thd:%4d, normal_finger_thd_1:%4d, normal_finger_thd_2:%4d, normal_finger_thd_3:%4d\n\
    large_palm_thd_1:%4d, large_palm_thd_2:%4d, palm_thd_1:%4d, palm_thd_2:%4d, small_palm_thd_1:%4d, small_palm_thd_2:%4d\n",
			   short_side_para->edge_finger_thd, short_side_para->hold_finger_thd, short_side_para->normal_finger_thd_1,
			   short_side_para->normal_finger_thd_2, short_side_para->normal_finger_thd_3, short_side_para->large_palm_thd_1, short_side_para->large_palm_thd_2,
			   short_side_para->palm_thd_1, short_side_para->palm_thd_2, short_side_para->small_palm_thd_1, short_side_para->small_palm_thd_2);
		seq_printf(s, "\n");
	}

	seq_printf(s, "elimination zone:\n");
	list_for_each(pos, &grip_info->elimination_zone_list) {
		grip_area = (struct grip_zone_area *)pos;

		seq_printf(s, "name:%24s, start_point:(%4d, %4d), width:(%4d, %4d), exit_thd:%4d, support_dir:0x%02x, side:0x%02x.\n",
			   grip_area->name, grip_area->start_x, grip_area->start_y, grip_area->x_width, grip_area->y_width,
			   grip_area->exit_thd, grip_area->support_dir, grip_area->grip_side);
	}
	seq_printf(s, "\n");

	return 0;
}

static int kernel_grip_read_func(struct seq_file *s, void *v)
{
	struct kernel_grip_info *grip_info = s->private;

	if (!grip_info) {
		TPD_INFO("%s read grip info failed.\n", __func__);
	}

	return kernel_grip_print_func(s, grip_info);
}

static int get_key_value(char *in, char *check)
{
	int out = 0;
	char *pos = NULL;
	int i = 0, real = 0, in_cnt = 0, check_cnt = 0;

	if (!in || !check) {
		TPD_INFO("%s:null.\n", __func__);
		return -1;
	}

	pos = strstr(in, check);

	if (!pos) {
		TPD_DEBUG("%s:can't find string:%s in %s.\n", __func__, check, in);
		return -1;
	}

	in_cnt = strlen(in);
	check_cnt = strlen(check);
	TPD_DEBUG("checklen:%d, inlen:%d.\n", check_cnt, in_cnt);

	if (':' != pos[check_cnt]) {  /*must match the format of string:value*/
		TPD_INFO("%s:%s do not match the format.\n", __func__, in);
		return -1;
	}

	for (i = check_cnt + 1; i < in_cnt; i++) {     /*check valid character*/
		if (pos[i] < '0' || pos[i] > '9') {
			if ((' ' == pos[i]) || (0 == pos[i]) || ('\n' == pos[i])) {
				break;

			} else if (real) {
				TPD_INFO("%s: incorrect char 0x%02x in %s.\n", __func__, pos[i], in);
				return -1;
			}

		} else {
			real = 1;
			out = (out * 10) + (pos[i] - '0');
			TPD_DEBUG("found char:%c.\n", pos[i]);
		}
	}

	TPD_DEBUG("return:%d.\n", out);
	return out;
}

static int str_to_int(char *in, int start_pos, int end_pos)
{
	int i = 0, value = 0;

	if (start_pos > end_pos) {
		TPD_INFO("wrong pos : (%d, %d).\n", start_pos, end_pos);
		return -1;
	}

	for (i = start_pos; i <= end_pos; i++) {
		value = (value * 10) + (in[i] - '0');
	}

	TPD_DEBUG("%s return %d.\n", __func__, value);
	return value;
}

/*parse string according to name:value1,value2,value3...*/
static int str_parse(char *in, char *name, uint16_t max_len, uint16_t *array,
		     uint16_t array_max)
{
	int i = 0, in_cnt = 0, name_index = 0;
	int start_pos = 0, value_cnt = 0;

	if (!array || !in) {
		TPD_INFO("%s:array or in is null.\n", __func__);
		return -1;
	}

	in_cnt = strlen(in);

	/*parse name*/
	for (i = 0; i < in_cnt; i++) {
		if (':' == in[i]) {     /*split name and parameter by ":" symbol*/
			if (i > max_len) {
				TPD_INFO("%s:string %s name too long.\n", __func__, in);
				return -1;
			}

			name_index = i;
			memcpy(name, in, name_index);   /*copy to name buffer*/
			TPD_DEBUG("%s:set name %s.\n", __func__, name);
		}
	}

	/*parse parameter and put it into split_value array*/
	start_pos = name_index + 1;

	for (i = name_index + 1; i <= in_cnt; i++) {
		if (in[i] < '0' || in[i] > '9') {
			if ((' ' == in[i]) || (0 == in[i]) || ('\n' == in[i]) || (',' == in[i])) {
				if (value_cnt <= array_max) {
					array[value_cnt++] = str_to_int(in, start_pos, i - 1);
					start_pos = i + 1;

				} else {
					TPD_INFO("%s: too many parameter(%s).\n", __func__, in);
					return -1;
				}

			} else {
				TPD_INFO("%s: incorrect char 0x%02x in %s.\n", __func__, in[i], in);
				return -1;
			}
		}
	}

	return 0;
}

static int grip_area_add_modify(struct list_head *handle_list, char *in,
				bool is_add, struct kernel_grip_info *grip_info)
{
	int ret = 0;
	char name[GRIP_TAG_SIZE] = {0};
	uint16_t split_value[MAX_AREA_PARAMETER] = {0};
	struct list_head *pos = NULL;
	struct grip_zone_area *grip_zone = NULL;
	struct fw_grip_operations *op = grip_info->fw_ops;
	struct touchpanel_data *ts = NULL;

	if (!handle_list || !in || !grip_info) {
		TPD_INFO("%s:handle_list or in is null.\n", __func__);
		return -1;
	}

	ts = grip_info->p_ts;

	ret = str_parse(in, name, GRIP_TAG_SIZE, split_value, MAX_AREA_PARAMETER);

	if (ret < 0) {
		GRIP_TP_INFO("%s str parse failed.\n", __func__);
		return -1;
	}

	if (is_add) {
		list_for_each(pos, handle_list) {
			grip_zone = (struct grip_zone_area *)pos;

			if (strstr(name, grip_zone->name)) {
				GRIP_TP_INFO("%s: same string(%s, %s).\n", __func__, name, grip_zone->name);
				return -1;
			}
		}

		grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

		if (!grip_zone) {
			GRIP_TP_INFO("%s kmalloc failed.\n", __func__);
			return -1;
		}

		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "%s", name);
		grip_zone->start_x = split_value[0];
		grip_zone->start_y = split_value[1];
		grip_zone->x_width = split_value[2];
		grip_zone->y_width = split_value[3];
		grip_zone->exit_thd = split_value[4];
		grip_zone->support_dir = split_value[5];
		grip_zone->grip_side = split_value[6];
		list_add_tail(&grip_zone->area_list, handle_list);
		GRIP_TP_INFO("%s add: [%d, %d] [%d %d] %d %d %d.\n",
			 grip_zone->name, grip_zone->start_x, grip_zone->start_y,
			 grip_zone->x_width, grip_zone->y_width, grip_zone->exit_thd,
			 grip_zone->support_dir, grip_zone->grip_side);

		if (grip_info->grip_handle_in_fw && op && op->set_fw_grip_area
		    && ts && !ts->loading_fw) {
			ret = op->set_fw_grip_area(ts->chip_data, grip_zone, true);

			if (ret < 0) {
				GRIP_TP_INFO("%s: set grip area in fw failed !\n", __func__);
				return ret;
			}
		}

	} else {
		list_for_each(pos, handle_list) {
			grip_zone = (struct grip_zone_area *)pos;

			if (strstr(in, grip_zone->name)) {
				grip_zone->start_x = split_value[0];
				grip_zone->start_y = split_value[1];
				grip_zone->x_width = split_value[2];
				grip_zone->y_width = split_value[3];
				grip_zone->exit_thd = split_value[4];
				grip_zone->support_dir = split_value[5];
				grip_zone->grip_side = split_value[6];
				if (grip_info->is_curved_screen) {
					grip_zone->exit_tx_er = split_value[7];
					grip_zone->exit_rx_er = split_value[8];
					GRIP_TP_INFO("%s modify: [%d, %d] [%d %d] %d %d %d %d %d.\n",
						 grip_zone->name, grip_zone->start_x, grip_zone->start_y,
						 grip_zone->x_width, grip_zone->y_width, grip_zone->exit_thd,
						 grip_zone->support_dir, grip_zone->grip_side,
						 grip_zone->exit_tx_er, grip_zone->exit_rx_er);
				} else {
					GRIP_TP_INFO("%s modify: [%d, %d] [%d %d] %d %d %d.\n",
						 grip_zone->name, grip_zone->start_x, grip_zone->start_y,
						 grip_zone->x_width, grip_zone->y_width, grip_zone->exit_thd,
						 grip_zone->support_dir, grip_zone->grip_side);
				}

				if (grip_info->grip_handle_in_fw && op && op->set_fw_grip_area
				    && ts && !ts->loading_fw) {
					ret = op->set_fw_grip_area(ts->chip_data, grip_zone, true);

					if (ret < 0) {
						GRIP_TP_INFO("%s: set grip area in fw failed !\n", __func__);
						return ret;
					}
				}
			}
		}
	}

	return 0;
}

static void grip_area_del(struct list_head *handle_list, char *in,
			  struct kernel_grip_info *grip_info)
{
	struct list_head *pos = NULL;
	struct grip_zone_area *grip_area = NULL;
	struct fw_grip_operations *op = grip_info->fw_ops;
	struct touchpanel_data *ts = NULL;

	if (!handle_list || !in || !grip_info) {
		TPD_INFO("%s:handle_list or in is null.\n", __func__);
		return;
	}

	ts = grip_info->p_ts;

	list_for_each(pos, handle_list) {
		grip_area = (struct grip_zone_area *)pos;

		if (strstr(in, grip_area->name)) {
			if (grip_info->grip_handle_in_fw && op && op->set_fw_grip_area
			    && ts && !ts->loading_fw) {
				op->set_fw_grip_area(ts->chip_data, grip_area, false);
			}

			list_del(&grip_area->area_list);
			kfree(grip_area);
			grip_area = NULL;
			GRIP_TP_INFO("%s:remove area: %s.\n", __func__, in);
			return;
		}
	}

	GRIP_TP_INFO("%s:can not found  area: %s.\n", __func__, in);
	return;
}

static void skip_area_modify(struct kernel_grip_info *grip_info, char *in)
{
	int ret = 0;
	char name[GRIP_TAG_SIZE] = {0};
	uint16_t split_value[MAX_AREA_PARAMETER] = {0};
	struct fw_grip_operations *op = NULL;
	struct touchpanel_data *ts = NULL;

	if (!grip_info || !in) {
		TPD_INFO("%s:grip_info or in is null.\n", __func__);
		return;
	}

	ts = grip_info->p_ts;

	op = grip_info->fw_ops;

	ret = str_parse(in, name, GRIP_TAG_SIZE, split_value, MAX_AREA_PARAMETER);

	if (ret < 0) {
		GRIP_TP_INFO("%s str parse failed.\n", __func__);
		return;
	}

	grip_info->no_handle_dir = split_value[0];
	grip_info->no_handle_y1 = split_value[1];
	grip_info->no_handle_y2 = split_value[2];
	GRIP_TP_INFO("set skip (%d,%d,%d).\n", grip_info->no_handle_dir,
		 grip_info->no_handle_y1, grip_info->no_handle_y2);

	if (grip_info->grip_handle_in_fw && op && op->set_no_handle_area
	    && ts && !ts->loading_fw) {
		ret = op->set_no_handle_area(ts->chip_data, grip_info);

		if (ret < 0) {
			GRIP_TP_INFO("%s: set no handle area in fw failed !\n", __func__);
		}
	}

	return;
}

/*format is opearation object name:x,y,z,m,m*/
static int kernel_grip_parse(struct kernel_grip_info *grip_info, char *input,
			     int len)
{
	int value = 0, ret = 0, in = 0;
	operate_cmd cmd = OPERATE_UNKNOW;
	operate_oject object = OBJECT_UNKNOW;
	struct list_head *handle_list = NULL;
	int i = 0, str_cnt = 0, start_pos = 0, end_pos = 0;
	char split_str[MAX_STRING_CNT][GRIP_TAG_SIZE] = {{0}};
	struct fw_grip_operations *op = grip_info->fw_ops;
	struct curved_judge_para *long_side_para = &grip_info->curved_long_side_para;
	struct curved_judge_para *short_side_para = &grip_info->curved_short_side_para;
	struct touchpanel_data *ts = NULL;

	ts = grip_info->p_ts;

	/*split string using space*/
	for (i = 1; i < len; i++) {
		if ((' ' == input[i]) || (len - 1 == i)) {      /*find a space or to the end*/
			if (' ' == input[i]) {
				end_pos = i - 1;

			} else {
				end_pos = i;
			}

			if (end_pos - start_pos + 1 > GRIP_TAG_SIZE) {
				input[i] = '\0';
				GRIP_TP_INFO("found too long string:%s, return.\n", &input[start_pos]);
				return 0;
			}

			if (str_cnt >= MAX_STRING_CNT) {
				input[i] = '\0';
				GRIP_TP_INFO("found too many string:%d, last:%s.\n", str_cnt, &input[start_pos]);
				return 0;
			}

			memcpy(split_str[str_cnt], &input[start_pos],
			       end_pos - start_pos + 1);   /*copy to str buffer*/
			start_pos = i + 1;
			str_cnt++;
		}
	}

	i = 0;
	str_cnt--;          /*get real count*/

	while (i < str_cnt) {
		if (OPERATE_UNKNOW == cmd) {
			if (!strcmp(split_str[i], "add")) {
				cmd = OPERATE_ADD;
				i++;

			} else if (!strcmp(split_str[i], "del")) {
				cmd = OPERATE_DELTE;
				i++;

			} else if (!strcmp(split_str[i], "mod")) {
				cmd = OPERATE_MODIFY;
				i++;
			}
		}

		if (OBJECT_UNKNOW == object) {
			if (!strcmp(split_str[i], "para")) {
				object = OBJECT_PARAMETER;
				i++;
				TPD_DEBUG("set object to para.\n");
			} else if (!strcmp(split_str[i], "long_curved_para")) {
				object = OBJECT_LONG_CURVED_PARAMETER;
				i++;
				TPD_DEBUG("set object to curved screen judge para for long side.\n");
			} else if (!strcmp(split_str[i], "short_curved_para")) {
				object = OBJECT_SHORT_CURVED_PARAMETER;
				i++;
				TPD_DEBUG("set object to curved screen judge para for short side.\n");
			} else if (!strcmp(split_str[i], "para_V2")) {
				object = OBJECT_PARAMETER_V2;
				i++;
				TPD_DEBUG("set object to V2 judge para.\n");
			} else if (!strcmp(split_str[i], "condition_area")) {
				handle_list = &grip_info->condition_zone_list;
				object = OBJECT_CONDITION_AREA;
				i++;
				TPD_DEBUG("set object to condition_area.\n");

			} else if (!strcmp(split_str[i], "large_area")) {
				handle_list = &grip_info->large_zone_list;
				object = OBJECT_LARGE_AREA;
				i++;
				TPD_DEBUG("set object to large_area.\n");

			} else if (!strcmp(split_str[i], "eli_area")) {
				handle_list = &grip_info->elimination_zone_list;
				object = OBJECT_ELI_AREA;
				i++;
				TPD_DEBUG("set object to eli_area.\n");

			} else if (!strcmp(split_str[i], "dead_area")) {
				handle_list = &grip_info->dead_zone_list;
				object = OBJECT_DEAD_AREA;
				i++;
				TPD_DEBUG("set object to dead_area.\n");

			} else if (!strcmp(split_str[i], "edgescreen")) {
				object = OBJECT_SKIP_HANDLE;
				i++;
				TPD_DEBUG("set object to edgescreen.\n");
			}
		}

		if (OBJECT_PARAMETER == object) {
			if (OPERATE_MODIFY == cmd) {
				if ((value = get_key_value(split_str[i], "condition_frame_limit")) >= 0) {
					grip_info->condition_frame_limit = value;
					GRIP_TP_INFO("change condition_frame_limit to %d.\n", value);

					if (grip_info->grip_handle_in_fw && op && op->set_condition_frame_limit
					    && ts && !ts->loading_fw) {
						ret = op->set_condition_frame_limit(ts->chip_data, value);

						if (ret < 0) {
							GRIP_TP_INFO("%s: set condition frame limit in fw failed !\n", __func__);
						}
					}

				} else if ((value = get_key_value(split_str[i], "condition_updelay_ms")) >= 0) {
					grip_info->condition_updelay_ms = value;
					GRIP_TP_INFO("change condition_updelay_ms to %d.\n", value);

				} else if ((value = get_key_value(split_str[i], "large_frame_limit"))  >= 0) {
					grip_info->large_frame_limit = value;
					GRIP_TP_INFO("change large_frame_limit to %d.\n", value);

					if (grip_info->grip_handle_in_fw && op && op->set_large_frame_limit
					    && ts && !ts->loading_fw) {
						ret = op->set_large_frame_limit(ts->chip_data, value);

						if (ret < 0) {
							GRIP_TP_INFO("%s: set condition frame limit in fw failed !\n", __func__);
						}
					}

				} else if ((value = get_key_value(split_str[i], "large_ver_thd"))  >= 0) {
					grip_info->large_ver_thd = value;
					GRIP_TP_INFO("change large_ver_thd to %d.\n", value);

					if (grip_info->grip_handle_in_fw && op && op->set_large_ver_thd
					    && ts && !ts->loading_fw) {
						ret = op->set_large_ver_thd(ts->chip_data, value);

						if (ret < 0) {
							GRIP_TP_INFO("%s: set large ver thd in fw failed !\n", __func__);
						}
					}

				} else if ((value = get_key_value(split_str[i], "large_hor_thd"))  >= 0) {
					grip_info->large_hor_thd = value;
					GRIP_TP_INFO("change large_hor_thd to %d.\n", value);

				} else if ((value = get_key_value(split_str[i], "large_hor_corner_thd")) >= 0) {
					grip_info->large_hor_corner_thd = value;
					GRIP_TP_INFO("change large_hor_corner_thd to %d.\n", value);

				} else if ((value = get_key_value(split_str[i],
								  "large_ver_corner_thd"))  >= 0) {
					grip_info->large_ver_corner_thd = value;
					GRIP_TP_INFO("change large_ver_corner_thd to %d.\n", value);

				} else if ((value = get_key_value(split_str[i],
								  "large_corner_frame_limit"))  >= 0) {
					grip_info->large_corner_frame_limit = value;
					GRIP_TP_INFO("change large_corner_frame_limit to %d.\n", value);

					if (grip_info->grip_handle_in_fw && op && op->set_large_corner_frame_limit
					    && ts && !ts->loading_fw) {
						ret = op->set_large_corner_frame_limit(ts->chip_data, value);

						if (ret < 0) {
							GRIP_TP_INFO("%s: set large condition frame limit in fw failed !\n", __func__);
						}
					}

				} else if ((value = get_key_value(split_str[i],
								  "large_ver_corner_width"))  >= 0) {
					grip_info->large_ver_corner_width = value;
					GRIP_TP_INFO("change large_ver_corner_width to %d.\n", value);

				} else if ((value = get_key_value(split_str[i],
								  "large_hor_corner_width"))  >= 0) {
					grip_info->large_hor_corner_width = value;
					GRIP_TP_INFO("change large_hor_corner_width to %d.\n", value);

				} else if ((value = get_key_value(split_str[i],
								  "large_corner_distance"))  >= 0) {
					grip_info->large_corner_distance = value;
					GRIP_TP_INFO("change large_corner_distance to %d.\n", value);

				} else if ((value = get_key_value(split_str[i], "grip_disable_level"))  >= 0) {
					grip_info->grip_disable_level |= 1 << value;
					if (grip_info->grip_handle_in_fw && op && op->set_disable_level
					    && ts && !ts->loading_fw) {
						ret = op->set_disable_level(ts->chip_data, grip_info->grip_disable_level);

						if (ret < 0) {
							GRIP_TP_INFO("%s: set disable level in fw failed !\n", __func__);
						}
					}
					GRIP_TP_INFO("change grip_disable_level to %d.\n", value);

				} else if ((value = get_key_value(split_str[i], "grip_enable_level"))  >= 0) {
					grip_info->grip_disable_level &= ~(1 << value);
					if (grip_info->grip_handle_in_fw && op && op->set_disable_level
					    && ts && !ts->loading_fw) {
						ret = op->set_disable_level(ts->chip_data, grip_info->grip_disable_level);

						if (ret < 0) {
							GRIP_TP_INFO("%s: set disable level in fw failed !\n", __func__);
						}
					}
					GRIP_TP_INFO("change grip_enable_level to %d.\n", value);

				} else if ((value = get_key_value(split_str[i], "large_detect_time_ms"))  >= 0) {
					grip_info->large_detect_time_ms = value;
					GRIP_TP_INFO("change large_detect_time_ms to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "down_delta_time_ms"))  >= 0) {
					grip_info->down_delta_time_ms = value;
					GRIP_TP_INFO("change down_delta_time_ms to %d.\n", value);
				} else {
					GRIP_TP_INFO("not support:%s.\n", split_str[i]);
				}

			} else {
				GRIP_TP_INFO("not support %d opeartion for long sid curved screen parameter modify.\n", cmd);
			}
		} else if (OBJECT_LONG_CURVED_PARAMETER == object) {
			if (OPERATE_MODIFY == cmd) {
				if ((value = get_key_value(split_str[i], "edge_finger_thd")) >= 0) {
					long_side_para->edge_finger_thd = value;
					GRIP_TP_INFO("change long side edge_finger_thd to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "hold_finger_thd")) >= 0) {
					long_side_para->hold_finger_thd = value;
					GRIP_TP_INFO("change long side hold_finger_thd to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "normal_finger_thd_1"))  >= 0) {
					long_side_para->normal_finger_thd_1 = value;
					GRIP_TP_INFO("change long side normal_finger_thd_1 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "normal_finger_thd_2"))  >= 0) {
					long_side_para->normal_finger_thd_2 = value;
					GRIP_TP_INFO("change long side normal_finger_thd_2 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "normal_finger_thd_3"))  >= 0) {
					long_side_para->normal_finger_thd_3 = value;
					GRIP_TP_INFO("change long side normal_finger_thd_3 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "large_palm_thd_1"))  >= 0) {
					long_side_para->large_palm_thd_1 = value;
					GRIP_TP_INFO("change long side large_palm_thd_1 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "large_palm_thd_2")) >= 0) {
					long_side_para->large_palm_thd_2 = value;
					GRIP_TP_INFO("change long side large_palm_thd_2 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "small_palm_thd_1"))  >= 0) {
					long_side_para->small_palm_thd_1 = value;
					GRIP_TP_INFO("change long side small_palm_thd_1 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "small_palm_thd_2"))  >= 0) {
					long_side_para->small_palm_thd_2 = value;
					GRIP_TP_INFO("change long side small_palm_thd_2 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "palm_thd_1"))  >= 0) {
					long_side_para->palm_thd_1 = value;
					GRIP_TP_INFO("change long side palm_thd_1 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "palm_thd_2"))  >= 0) {
					long_side_para->palm_thd_2 = value;
					GRIP_TP_INFO("change long side palm_thd_2 to %d.\n", value);
				} else {
					GRIP_TP_INFO("not support:%s.\n", split_str[i]);
				}
			} else {
				GRIP_TP_INFO("not support %d opeartion for long sid curved screen parameter modify.\n", cmd);
			}
		} else if (OBJECT_SHORT_CURVED_PARAMETER == object) {
			if (OPERATE_MODIFY == cmd) {
				if ((value = get_key_value(split_str[i], "edge_finger_thd")) >= 0) {
					short_side_para->edge_finger_thd = value;
					GRIP_TP_INFO("change short side edge_finger_thd to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "hold_finger_thd")) >= 0) {
					short_side_para->hold_finger_thd = value;
					GRIP_TP_INFO("change short side hold_finger_thd to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "normal_finger_thd_1"))  >= 0) {
					short_side_para->normal_finger_thd_1 = value;
					GRIP_TP_INFO("change short side normal_finger_thd_1 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "normal_finger_thd_2"))  >= 0) {
					short_side_para->normal_finger_thd_2 = value;
					GRIP_TP_INFO("change short side normal_finger_thd_2 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "normal_finger_thd_3"))  >= 0) {
					short_side_para->normal_finger_thd_3 = value;
					GRIP_TP_INFO("change short side normal_finger_thd_3 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "large_palm_thd_1"))  >= 0) {
					short_side_para->large_palm_thd_1 = value;
					GRIP_TP_INFO("change short side large_palm_thd_1 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "large_palm_thd_2")) >= 0) {
					short_side_para->large_palm_thd_2 = value;
					GRIP_TP_INFO("change short side large_palm_thd_2 to %d.\n", value);
				}  else if ((value = get_key_value(split_str[i], "small_palm_thd_1"))  >= 0) {
					short_side_para->small_palm_thd_1 = value;
					GRIP_TP_INFO("change short side small_palm_thd_1 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "small_palm_thd_2"))  >= 0) {
					short_side_para->small_palm_thd_2 = value;
					GRIP_TP_INFO("change short side small_palm_thd_2 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "palm_thd_1"))  >= 0) {
					short_side_para->palm_thd_1 = value;
					GRIP_TP_INFO("change short side palm_thd_1 to %d.\n", value);
				} else if ((value = get_key_value(split_str[i], "palm_thd_2"))  >= 0) {
					short_side_para->palm_thd_2 = value;
					GRIP_TP_INFO("change short side palm_thd_2 to %d.\n", value);
				} else {
					GRIP_TP_INFO("not support:%s.\n", split_str[i]);
				}
			} else {
				GRIP_TP_INFO("not support %d opeartion for short sid curved screen parameter modify.\n", cmd);
			}
		} else if ((OBJECT_CONDITION_AREA == object) || (OBJECT_LARGE_AREA == object) ||
			   (OBJECT_ELI_AREA == object) || (OBJECT_DEAD_AREA == object)) {
			if (OPERATE_ADD == cmd) {
				TP_DETAIL(grip_info->tp_index, "add %d by %s.\n", object, split_str[i]);
				grip_area_add_modify(handle_list, split_str[i], true,
						     grip_info);       /*add new area to list*/

			} else if (OPERATE_DELTE == cmd) {
				TP_DETAIL(grip_info->tp_index, "del %d by %s.\n", object, split_str[i]);
				/*del area from list*/
				grip_area_del(handle_list, split_str[i], grip_info);

			} else if (OPERATE_MODIFY == cmd) {
				TP_DETAIL(grip_info->tp_index, "modify %d by %s.\n", object, split_str[i]);
				grip_area_add_modify(handle_list, split_str[i], false,
						     grip_info);       /*modify paramter of the area*/

			} else {
				GRIP_TP_INFO("not support %d opeartion for area modify.\n", cmd);
			}

		} else if (OBJECT_SKIP_HANDLE == object) {
			if (OPERATE_MODIFY == cmd) {
				GRIP_TP_INFO("modify %d by %s.\n", object, split_str[i]);
				skip_area_modify(grip_info, split_str[i]);

			} else {
				GRIP_TP_INFO("not support %d opeartion for skip handle modify.\n", cmd);
			}
		} else if (OBJECT_PARAMETER_V2 == object) {
			if (OPERATE_MODIFY == cmd) {
				for (in = 0; in < KEY_ADDR_NUMS; in++) {
					value = get_key_value(split_str[i], (char *)key_addr_arrays[in].name);
					if (value >= 0) {
						break;
					}
				}
				if (in < KEY_ADDR_NUMS) {
					set_key_addr_value(grip_info, in, value);
					/*update the current reclining_mode_data*/
					grip_prase_reclining_mode_handle(grip_info);
					touch_misc_state_change(ts, IOC_STATE_PREVENTION_PARA_CHANGE, 1);
					GRIP_TP_INFO("change %s to %d.\n", key_addr_arrays[in].name, value);
					if (0 == strcmp(key_addr_arrays[in].name, "set_ime_showing")) {
						disable_algo_for_ime_showing(grip_info);
					}
				} else {
					GRIP_TP_INFO("not found:%d or addr is NULL.\n", in);
				}
			} else {
				GRIP_TP_INFO("not support %d opeartion for V2 parameter modify.\n", cmd);
			}
		}

		handle_list = NULL;         /*reset handle list*/
		cmd = OPERATE_UNKNOW;       /*reset cmd status*/
		object = OBJECT_UNKNOW;     /*reset object status*/
		i++;
	}

	return 0;
}

static void transfer_grip_cmdList_to_single(struct kernel_grip_info *grip_info, char *info, int w_size)
{
	char *token = NULL;
	char single_cmd[GRIP_SINGLE_CMD_SIZE] = {0};
	char *temp = NULL;

	if (info == NULL) {
		TPD_INFO("info is null\n");
		return;
	}

	temp = info;
	while ((token = strsep(&temp, ";")) != NULL) {
		if ((strlen(token) + 1) <= GRIP_SINGLE_CMD_SIZE) {
			memcpy(single_cmd, token, strlen(token));
			single_cmd[strlen(token)] = '\0';
			kernel_grip_parse(grip_info, single_cmd, strlen(single_cmd) + 1);
		} else {
			TPD_INFO("token:%s size is beyond\n", token);
		}
	}

	return;
}

static ssize_t kernel_grip_write(struct file *file, const char __user *buffer,
				 size_t count, loff_t *ppos)
{
	char *buf = NULL;
	struct kernel_grip_info *grip_info = PDE_DATA(file_inode(file));

	if (!grip_info) {
		return count;
	}

	if (count > GRIP_ALL_CMD_SIZE) {
		GRIP_TP_INFO("%s: count is too large :%d.\n",  __func__, (int)count);
		return count;
	}

	buf = kzalloc(GRIP_ALL_CMD_SIZE, GFP_KERNEL);

	if (!buf) {
		GRIP_TP_INFO("%s kmalloc failed.\n", __func__);
		return count;
	}

	if (copy_from_user(buf, buffer, count)) {
		GRIP_TP_INFO("%s: read proc input error.\n", __func__);
		if (buf) {
			kfree(buf);
		}
		return count;
	}

	mutex_lock(&grip_info->grip_mutex);
	transfer_grip_cmdList_to_single(grip_info, buf, GRIP_ALL_CMD_SIZE);
	mutex_unlock(&grip_info->grip_mutex);
	if (buf) {
		kfree(buf);
	}
	return count;
}

static int kernel_grip_open(struct inode *inode, struct file *file)
{
	return single_open(file, kernel_grip_read_func, PDE_DATA(inode));
}

DECLARE_PROC_OPS(tp_kernel_grip_fops, kernel_grip_open, seq_read, kernel_grip_write, single_release);

static ssize_t proc_touch_dir_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char page[PAGESIZE] = {0};
	struct kernel_grip_info *grip_info = PDE_DATA(file_inode(file));

	if (!grip_info) {
		return count;
	}

	snprintf(page, PAGESIZE - 1, "%d\n", grip_info->touch_dir);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t proc_touch_dir_write(struct file *file,
				    const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8] = {0};
	int temp = 0;
	struct kernel_grip_info *grip_info = PDE_DATA(file_inode(file));
	struct fw_grip_operations *op;
	struct touchpanel_data *ts = NULL;

	if (!grip_info) {
		TPD_INFO("%s: no value.\n", __func__);
		return count;
	}

	ts = grip_info->p_ts;
	op = grip_info->fw_ops;

	if (count > 2) {
		return count;
	}

	if (copy_from_user(buf, buffer, count)) {
		TPD_DEBUG("%s: read proc input error.\n", __func__);
		return count;
	}

	sscanf(buf, "%d", &temp);

	touch_misc_state_change(ts, IOC_STATE_DIR, temp);

	mutex_lock(&grip_info->grip_mutex);
	grip_info->touch_dir = temp;

	if (grip_info->grip_handle_in_fw && op && op->set_touch_direction
	    && ts && !ts->loading_fw) {
		op->set_touch_direction(ts->chip_data, temp);
		if (grip_info->dir_change_set_grip) {
			mutex_lock(&ts->mutex);
			ts->ts_ops->enable_kernel_grip(ts->chip_data, ts->grip_info);
			mutex_unlock(&ts->mutex);
		}
	}

	if (ts != NULL) {
		if (ts->edge_pull_out_support) {
			mutex_lock(&ts->mutex);
			ts->ts_ops->set_touch_direction(ts->chip_data, (unsigned char)temp);
			mutex_unlock(&ts->mutex);
		}
	}

	mutex_unlock(&grip_info->grip_mutex);

	GRIP_TP_INFO("%s: value = %d\n", __func__, temp);
	return count;
}

DECLARE_PROC_OPS(touch_dir_proc_fops, simple_open, proc_touch_dir_read, proc_touch_dir_write, NULL);

static ssize_t proc_touch_reclining_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char page[PAGESIZE] = {0};
	struct kernel_grip_info *grip_info = PDE_DATA(file_inode(file));

	if (!grip_info) {
		return count;
	}

	snprintf(page, PAGESIZE - 1, "%d\n", grip_info->touch_reclining_mode);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static void transform_normal_para(struct kernel_grip_info *grip_info)
{
	if (!grip_info) {
		TPD_INFO("%s: grip_info is null.\n", __func__);
		return;
	}
	memset(&grip_info->current_data, 0, sizeof(struct reclining_mode_data));
	memcpy(&grip_info->current_data, &grip_info->normal_data, sizeof(struct reclining_mode_data));
}

static void transform_reclining_para(struct kernel_grip_info *grip_info)
{
	if (!grip_info) {
		TPD_INFO("%s: grip_info is null.\n", __func__);
		return;
	}
	memset(&grip_info->current_data, 0, sizeof(struct reclining_mode_data));
	memcpy(&grip_info->current_data, &grip_info->reclining_data, sizeof(struct reclining_mode_data));
}

static void grip_prase_reclining_mode_handle(struct kernel_grip_info *grip_info)
{

	if (!grip_info) {
		TPD_INFO("%s: grip_info is null.\n", __func__);
		return;
	}

	GRIP_TP_INFO("%s: enter.\n", __func__);
	if (grip_info->reclining_mode_support) {
		if (VERTICAL_RECLINING_MODE == grip_info->touch_reclining_mode) {
			transform_reclining_para(grip_info);

		} else if (LANDSCAPE_RECLINING_MODE == grip_info->touch_reclining_mode) {
			transform_reclining_para(grip_info);

		} else {
			transform_normal_para(grip_info);
		}
	} else {
		transform_normal_para(grip_info);
	}
	return;
}

static void proc_reclining_mode_handle(struct kernel_grip_info *grip_info)
{
	struct touchpanel_data *ts = NULL;

	if (!grip_info) {
		TPD_INFO("%s: grip_info is null.\n", __func__);
		return;
	}
	ts = grip_info->p_ts;
	if (!ts) {
		TPD_INFO("%s: ts is null.\n", __func__);
		return;
	}
	if (VERTICAL_RECLINING_MODE == grip_info->touch_reclining_mode) {
		transform_reclining_para(grip_info);
		grip_info->grip_moni_data.vertical_reclining_mode_times++;
		if (ts->health_monitor_support) {
			reset_healthinfo_time_counter(&grip_info->reclining_start_time);
		}
	} else if (LANDSCAPE_RECLINING_MODE == grip_info->touch_reclining_mode) {
		transform_reclining_para(grip_info);
		grip_info->grip_moni_data.landscape_reclining_mode_times++;
		if (ts->health_monitor_support) {
			reset_healthinfo_time_counter(&grip_info->reclining_start_time);
		}
	} else {
		transform_normal_para(grip_info);
		if (ts->health_monitor_support) {
			tp_healthinfo_report(&ts->monitor_data, HEALTH_GRIP_RECLINING, grip_info);
			grip_info->reclining_start_time = 0;
		}
	}
	return;
}

static ssize_t proc_touch_reclining_write(struct file *file,
				    const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8] = {0};
	int temp = 0;
	struct kernel_grip_info *grip_info = PDE_DATA(file_inode(file));
	struct touchpanel_data *ts = NULL;

	if (!grip_info) {
		TPD_INFO("%s: no value.\n", __func__);
		return count;
	}

	ts = grip_info->p_ts;

	tp_copy_from_user(buf, sizeof(buf), buffer, count, 2);

	if (kstrtoint(buf, 10, &temp)) {
		GRIP_TP_INFO("%s: kstrtoint error\n", __func__);
		return count;
	}

	touch_misc_state_change(ts, IOC_STATE_RECLINING_MODE, temp);

	mutex_lock(&grip_info->grip_mutex);
	grip_info->touch_reclining_mode = temp;
	proc_reclining_mode_handle(grip_info);
	mutex_unlock(&grip_info->grip_mutex);
	grip_info->last_reclining_mode = grip_info->touch_reclining_mode;

	GRIP_TP_INFO("%s: touch_reclining_mode = %d\n", __func__, temp);
	return count;
}

DECLARE_PROC_OPS(touch_reclining_proc_fops, simple_open, proc_touch_reclining_read, proc_touch_reclining_write, NULL);

void init_kernel_grip_proc(struct proc_dir_entry *prEntry_tp,
			   struct kernel_grip_info *grip_info)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_tmp = NULL;

	if (!grip_info || !prEntry_tp) {
		return;
	}

	prEntry_tmp = proc_create_data("kernel_grip_handle", 0666, prEntry_tp,
				       &tp_kernel_grip_fops, grip_info);

	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		GRIP_TP_INFO("%s: Couldn't create kernel grip proc entry, %d\n", __func__,
			 __LINE__);
	}

	prEntry_tmp = proc_create_data("oplus_tp_direction", 0666, prEntry_tp,
				       &touch_dir_proc_fops, grip_info);

	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		GRIP_TP_INFO("%s: Couldn't create oplus_tp_direction proc entry, %d\n", __func__,
			 __LINE__);
	}

	if (grip_info->reclining_mode_support) {
		prEntry_tmp = proc_create_data("reclining_mode", 0666, prEntry_tp,
					       &touch_reclining_proc_fops, grip_info);
		if (prEntry_tmp == NULL) {
			ret = -ENOMEM;
			GRIP_TP_INFO("%s: Couldn't create reclining_mode proc entry, %d\n", __func__,
				 __LINE__);
		}
	}
}

static void kernel_grip_release(struct kernel_grip_info *grip_info)
{
	struct list_head *pos = NULL, *tmp = NULL;
	struct grip_zone_area *grip_area = NULL;

	if (!grip_info) {
		TPD_INFO("%s:grip_info is null.\n", __func__);
		return;
	}

	list_for_each_safe(pos, tmp, &grip_info->dead_zone_list) {
		grip_area = (struct grip_zone_area *)pos;
		list_del(&grip_area->area_list);
		kfree(grip_area);
	}

	list_for_each_safe(pos, tmp, &grip_info->condition_zone_list) {
		grip_area = (struct grip_zone_area *)pos;
		list_del(&grip_area->area_list);
		kfree(grip_area);
	}

	list_for_each_safe(pos, tmp, &grip_info->large_zone_list) {
		grip_area = (struct grip_zone_area *)pos;
		list_del(&grip_area->area_list);
		kfree(grip_area);
	}

	list_for_each_safe(pos, tmp, &grip_info->elimination_zone_list) {
		grip_area = (struct grip_zone_area *)pos;
		list_del(&grip_area->area_list);
		kfree(grip_area);
	}

	if (grip_info->coord_buf) {
		kfree(grip_info->coord_buf);
		grip_info->coord_buf = NULL;
	}

	if (grip_info->grip_up_handle_wq) {
		destroy_workqueue(grip_info->grip_up_handle_wq);
		grip_info->grip_up_handle_wq = NULL;
	}

	kfree(grip_info);
	return;
}

void kernel_grip_reset(struct kernel_grip_info *grip_info)
{
	int i = 0;

	if (!grip_info) {
		TPD_INFO("%s:grip_info is null.\n", __func__);
		return;
	}

	for (i = 0; i < TOUCH_MAX_NUM; i++) {
		grip_status_reset(grip_info, i);
	}

	kfifo_reset(&grip_info->up_fifo);
	grip_info->obj_prev_bit = 0;        /*clear down bit*/
	grip_info->record_total_cnt = 0;    /*clear touch count*/

	return;
}

static int kernel_grip_init_v4_5(struct kernel_grip_info *grip_info, struct device *dev)
{
	int ret = 0;
	int temp_array[10] = {0};
	struct reclining_mode_data *reclining_data = NULL;
	struct reclining_mode_data *normal_data = NULL;

	if (grip_info == NULL) {
		return -1;
	}

	reclining_data = &grip_info->reclining_data;
	normal_data = &grip_info->normal_data;

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,long_press_corner_point_edge_percent_thd", temp_array, 8);
	if (ret) {
		grip_info->long_press_top_corner_point_edge_percent_thd = 1600;
		grip_info->long_press_bottom_corner_point_edge_percent_thd = 2000;
		grip_info->long_press_top_corner_point_edge_min_channel_thd = 1;
		grip_info->long_press_bottom_corner_point_edge_min_channel_thd = 1;
		grip_info->long_press_top_trx_corner_point_edge_percent_thd = 33;
		grip_info->long_press_bottom_trx_corner_point_edge_percent_thd = 35;
		grip_info->long_press_time_thd = 200;
		grip_info->long_press_point_edge_degree_check_support = 0;
		TPD_INFO("V4.5 corner_point_edge_percent_thd using default.\n");
	} else {
		grip_info->long_press_top_corner_point_edge_percent_thd = temp_array[0];
		grip_info->long_press_bottom_corner_point_edge_percent_thd = temp_array[1];
		grip_info->long_press_top_corner_point_edge_min_channel_thd = temp_array[2];
		grip_info->long_press_bottom_corner_point_edge_min_channel_thd = temp_array[3];
		grip_info->long_press_top_trx_corner_point_edge_percent_thd = temp_array[4];
		grip_info->long_press_bottom_trx_corner_point_edge_percent_thd = temp_array[5];
		grip_info->long_press_time_thd = temp_array[6];
		grip_info->long_press_point_edge_degree_check_support = temp_array[7];
		TPD_INFO("V4.5 corner_point_edge_percent_thd[%d][%d][%d][%d][%d][%d][%d][%d]\n", temp_array[0], temp_array[1],
				temp_array[2], temp_array[3], temp_array[4], temp_array[5], temp_array[6], temp_array[7]);
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,center_down_corner_point_edge_percent_thd", temp_array, 8);
	if (ret) {
		grip_info->center_down_top_corner_point_edge_percent_thd = 1600;
		grip_info->center_down_bottom_corner_point_edge_percent_thd = 2000;
		grip_info->center_down_top_corner_point_edge_min_channel_thd = 1;
		grip_info->center_down_bottom_corner_point_edge_min_channel_thd = 1;
		grip_info->center_down_top_trx_corner_point_edge_percent_thd = 33;
		grip_info->center_down_bottom_trx_corner_point_edge_percent_thd = 35;
		grip_info->center_down_point_edge_degree_check_support = 0;
		TPD_INFO("V4.5 corner_point_edge_percent_thd using default.\n");
	} else {
		grip_info->center_down_top_corner_point_edge_percent_thd = temp_array[0];
		grip_info->center_down_bottom_corner_point_edge_percent_thd = temp_array[1];
		grip_info->center_down_top_corner_point_edge_min_channel_thd = temp_array[2];
		grip_info->center_down_bottom_corner_point_edge_min_channel_thd = temp_array[3];
		grip_info->center_down_top_trx_corner_point_edge_percent_thd = temp_array[4];
		grip_info->center_down_bottom_trx_corner_point_edge_percent_thd = temp_array[5];
		grip_info->center_down_point_edge_degree_check_support = temp_array[7];
		TPD_INFO("V4.5 corner_point_edge_percent_thd[%d][%d][%d][%d][%d][%d][%d]\n", temp_array[0], temp_array[1],
				temp_array[2], temp_array[3], temp_array[4], temp_array[5], temp_array[7]);
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,soon_meet_corner_point_edge_percent_thd", temp_array, 8);
	if (ret) {
		grip_info->soon_meet_top_corner_point_edge_percent_thd = 1600;
		grip_info->soon_meet_bottom_corner_point_edge_percent_thd = 2000;
		grip_info->soon_meet_top_corner_point_edge_min_channel_thd = 1;
		grip_info->soon_meet_bottom_corner_point_edge_min_channel_thd = 1;
		grip_info->soon_meet_top_trx_corner_point_edge_percent_thd = 33;
		grip_info->soon_meet_bottom_trx_corner_point_edge_percent_thd = 35;
		grip_info->soon_meet_min_level_thd = 3;
		grip_info->soon_meet_point_edge_degree_check_support = 0;
		TPD_INFO("V4.5 corner_point_edge_percent_thd using default.\n");
	} else {
		grip_info->soon_meet_top_corner_point_edge_percent_thd = temp_array[0];
		grip_info->soon_meet_bottom_corner_point_edge_percent_thd = temp_array[1];
		grip_info->soon_meet_top_corner_point_edge_min_channel_thd = temp_array[2];
		grip_info->soon_meet_bottom_corner_point_edge_min_channel_thd = temp_array[3];
		grip_info->soon_meet_top_trx_corner_point_edge_percent_thd = temp_array[4];
		grip_info->soon_meet_bottom_trx_corner_point_edge_percent_thd = temp_array[5];
		grip_info->soon_meet_min_level_thd = temp_array[6];
		grip_info->soon_meet_point_edge_degree_check_support = temp_array[7];
		TPD_INFO("V4.5 corner_point_edge_percent_thd[%d][%d][%d][%d][%d][%d][%d][%d]\n", temp_array[0], temp_array[1],
				temp_array[2], temp_array[3], temp_array[4], temp_array[5], temp_array[6], temp_array[7]);
	}
	return 0;
}

static int kernel_grip_init_v4_3(struct kernel_grip_info *grip_info, struct device *dev)
{
	int ret = 0;
	int temp_array[10] = {0};
	int m_index = 0;
	struct reclining_mode_data *reclining_data = NULL;
	struct reclining_mode_data *normal_data = NULL;

	if (grip_info == NULL) {
		return -1;
	}

	reclining_data = &grip_info->reclining_data;
	normal_data = &grip_info->normal_data;

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,corner_point_edge_percent_thd", temp_array, 5);
	if (ret) {
		grip_info->corner_point_edge_percent_thd = 1600;
		grip_info->corner_point_edge_min_channel_thd = 1;
		grip_info->top_trx_corner_point_edge_percent_thd = 33;
		grip_info->bottom_trx_corner_point_edge_percent_thd = 35;
		grip_info->point_edge_degree_check_support = 0;
		TPD_INFO("V4.3 corner_point_edge_percent_thd using default.\n");
	} else {
		grip_info->corner_point_edge_percent_thd = temp_array[0];
		grip_info->corner_point_edge_min_channel_thd = temp_array[1];
		grip_info->top_trx_corner_point_edge_percent_thd = temp_array[2];
		grip_info->bottom_trx_corner_point_edge_percent_thd = temp_array[3];
		grip_info->point_edge_degree_check_support = temp_array[4];
		TPD_INFO("V4.3 corner_point_edge_percent_thd[%d][%d][%d][%d][%d]\n", temp_array[0], temp_array[1],
				temp_array[2], temp_array[3], temp_array[4]);
	}

	ret = of_property_read_u32_array(dev->of_node,  (char *)"prevention,corner_point_edge_percent_thd2", temp_array, 5);
	if (ret) {
		grip_info->corner_point_trx_radio_thd = 30;
		grip_info->corner_point_check_frame_thd = 3;
		grip_info->corner_point_peakdelta_stable_thd = 20;
		grip_info->bottom_corner_point_edge_percent_thd = 1300;
		TPD_INFO("corner_point_edge_percent_thd using default.\n");
	} else {
		grip_info->corner_point_trx_radio_thd = temp_array[0];
		grip_info->corner_point_check_frame_thd = temp_array[1];
		grip_info->corner_point_peakdelta_stable_thd = temp_array[2];
		grip_info->bottom_corner_point_edge_percent_thd = temp_array[3];
	}

	ret = of_property_read_u32_array(dev->of_node,  (char *)"prevention,point_stability_judgment_check", temp_array, 5);
	if (ret) {
		grip_info->point_stability_press_jitter_thd = 2;
		grip_info->point_stability_er_jitter_thd = 2;
		grip_info->point_stability_er_jitter_percent_thd = 10;
		grip_info->point_stability_judgment_check_support = 0;
		TPD_INFO("corner_point_edge_percent_thd using default.\n");
	} else {
		grip_info->point_stability_press_jitter_thd = temp_array[0];
		grip_info->point_stability_er_jitter_thd = temp_array[1];
		grip_info->point_stability_er_jitter_percent_thd = temp_array[2];
		grip_info->point_stability_judgment_check_support = temp_array[4];
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,point_corner_er_min_limit_check", temp_array, 5);
	if (ret) {
		grip_info->point_corner_tx_er_min_limit_thd = 3;
		grip_info->point_corner_rx_er_min_limit_thd = 3;
		grip_info->point_corner_er_min_limit_support = 0;
		TPD_INFO("point_corner_er_min_limit_check using default.\n");
	} else {
		grip_info->point_corner_tx_er_min_limit_thd = temp_array[0];
		grip_info->point_corner_rx_er_min_limit_thd = temp_array[1];
		grip_info->point_corner_er_min_limit_support = temp_array[4];
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,one_frame_down_check_support", temp_array, 1);
	if (ret) {
		grip_info->one_frame_down_check_support = 0;
		TPD_INFO("one_frame_down_check_support using default.\n");
	} else {
		grip_info->one_frame_down_check_support = temp_array[0];
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,uniform_make_up_point_v2", temp_array, 5);
	if (ret) {
		grip_info->uniform_make_up_point_num = 10;
		grip_info->uniform_make_up_exit_point_num = 2;
		grip_info->uniform_max_make_up_time = 45;
		grip_info->uniform_make_up_last_point_percent_limit = 5;
		grip_info->uniform_make_up_point_v2_support = 0;
		TPD_INFO("point_corner_er_min_limit_check using default.\n");
	} else {
		grip_info->uniform_make_up_point_num = temp_array[0];
		grip_info->uniform_make_up_exit_point_num = temp_array[1];
		grip_info->uniform_max_make_up_time = temp_array[2];
		grip_info->uniform_make_up_last_point_percent_limit = temp_array[3];
		grip_info->uniform_make_up_point_v2_support = temp_array[4];
	}

	for (m_index = 0; m_index < TOUCH_MAX_NUM; m_index++) {
		grip_info->uniform_make_up_point_num_status[m_index] = 0;
	}
	grip_info->uniform_make_up_point_upid_status = 0;


	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,touch_point_stability_check", temp_array, 5);
	if (ret) {
		grip_info->touch_point_stability_max_time = 100;
		grip_info->middle_bottom_long_press_time_thd = 500;
		TPD_INFO("touch_point_stability_check using default.\n");
	} else {
		grip_info->touch_point_stability_max_time = temp_array[0];
		grip_info->middle_bottom_long_press_time_thd = temp_array[1];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,middle_bottom_long_press_check", temp_array, 5);
	if (ret) {
		grip_info->middle_bottom_long_press_width_min_thd = 6;
		grip_info->middle_bottom_long_press_minor_max_thd = 140;
		grip_info->middle_bottom_long_press_match_times_thd = 10;
		grip_info->large_middle_bottom_exit_quickly_support = 0;
		TPD_INFO("middle_bottom_long_press_check using default.\n");
	} else {
		grip_info->middle_bottom_long_press_width_min_thd = temp_array[0];
		grip_info->middle_bottom_long_press_minor_max_thd = temp_array[1];
		grip_info->middle_bottom_long_press_match_times_thd = temp_array[2];
		grip_info->large_middle_bottom_exit_quickly_support = temp_array[4];
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,middle_bottom_small_press_check", temp_array, 5);
	if (ret) {
		grip_info->middle_bottom_small_press_width_max_thd = 4;
		grip_info->middle_bottom_small_press_er_min_thd = 70;
		grip_info->middle_bottom_small_press_peakdelta_max_thd = 550;
		grip_info->middle_bottom_small_press_minor_max_thd = 140;
		grip_info->middle_bottom_small_press_match_times_thd = 10;
		TPD_INFO("middle_bottom_small_press_check using default.\n");
	} else {
		grip_info->middle_bottom_small_press_width_max_thd = temp_array[0];
		grip_info->middle_bottom_small_press_er_min_thd = temp_array[1];
		grip_info->middle_bottom_small_press_peakdelta_max_thd = temp_array[2];
		grip_info->middle_bottom_small_press_minor_max_thd = temp_array[3];
		grip_info->middle_bottom_small_press_match_times_thd = temp_array[4];
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,middle_bottom_exit_dis", temp_array, 5);
	if (ret) {
		grip_info->middle_bottom_exit_x_dis_thd = 960;
		grip_info->middle_bottom_exit_y_dis_thd = 640;
		grip_info->middle_bottom_match_exit_x_dis_percent_thd = 20;
		grip_info->middle_bottom_match_exit_y_dis_thd = 640;
		TPD_INFO("middle_bottom_exit_dis using default.\n");
	} else {
		grip_info->middle_bottom_exit_x_dis_thd = temp_array[0];
		grip_info->middle_bottom_exit_y_dis_thd = temp_array[1];
		grip_info->middle_bottom_match_exit_x_dis_percent_thd = temp_array[2];
		grip_info->middle_bottom_match_exit_y_dis_thd = temp_array[3];
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,middle_bottom_yfsr_coupling_thd", temp_array, 5);
	if (ret) {
		grip_info->middle_bottom_short_exit_yfsr_coupling_thd = 90;
		grip_info->middle_bottom_long_exit_yfsr_coupling_thd = 60;
		grip_info->middle_bottom_normal_press_thd = 5;
		grip_info->middle_bottom_exit_yfsr_coupling_times_thd = 10;
		TPD_INFO("middle_bottom_yfsr_coupling_thd using default.\n");
	} else {
		grip_info->middle_bottom_short_exit_yfsr_coupling_thd = temp_array[0];
		grip_info->middle_bottom_long_exit_yfsr_coupling_thd = temp_array[1];
		grip_info->middle_bottom_normal_press_thd = temp_array[2];
		grip_info->middle_bottom_exit_yfsr_coupling_times_thd = temp_array[3];
	}

	return 0;
}

static int kernel_grip_init_v4(struct kernel_grip_info *grip_info, struct device *dev)
{
	int ret = 0;
	int temp_array[10] = {0};
	struct reclining_mode_data *reclining_data = NULL;
	struct reclining_mode_data *normal_data = NULL;

	if (grip_info == NULL) {
		return -1;
	}

	reclining_data = &grip_info->reclining_data;
	normal_data = &grip_info->normal_data;

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,long_fingerhold_condition2", temp_array, 4);
	if (ret) {
		grip_info->long_hold_x_width = 0xffff;/*default value 0, no need to change it*/
		grip_info->long_hold_y_width = grip_info->max_y / 4;
		grip_info->finger_hold_matched_hor_support = 0;
		GRIP_TP_INFO("large long finger hold long_fingerhold_condition2 using default.\n");
	} else {
		grip_info->long_hold_x_width = temp_array[0];
		grip_info->long_hold_y_width = temp_array[1];
		grip_info->finger_hold_matched_hor_support = temp_array[2];
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,top_corner_config2", temp_array, 5);
	if (ret) {
		grip_info->top_matched_times_thd = 3;
		grip_info->top_matched_xfsr_thd = 80;
		grip_info->large_ver_top_exit_distance = 200;
		GRIP_TP_INFO("large long finger hold long_fingerhold_condition2 using default.\n");
	} else {
		grip_info->top_matched_times_thd = temp_array[0];
		grip_info->top_matched_xfsr_thd = temp_array[1];
		grip_info->large_ver_top_exit_distance = temp_array[2];
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,hor_long_top_corner_config", temp_array, 4);
	if (ret) {
		grip_info->large_hor_top_x_width = 280;
		grip_info->large_hor_top_y_height = 1000;
		GRIP_TP_INFO("large long finger hold hor_long_top_corner_config using default.\n");
	} else {
		grip_info->large_hor_top_x_width = temp_array[0];
		grip_info->large_hor_top_y_height = temp_array[1];
	}

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,top_middle_config", temp_array, 4);
	if (ret) {
		grip_info->large_top_middle_width = grip_info->max_x;
		grip_info->large_top_middle_height = 500;
		grip_info->large_top_middle_exit_distance = 500;
		grip_info->large_bottom_middle_support = 0;
		GRIP_TP_INFO("vertical top bottom middle config using default.\n");
	} else {
		grip_info->large_top_middle_width = temp_array[0];
		grip_info->large_top_middle_height = temp_array[1];
		grip_info->large_top_middle_exit_distance = temp_array[2];
		grip_info->large_bottom_middle_support = temp_array[3];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,corner_eliminate_point_type", temp_array, 4);
	if (ret) {
		grip_info->corner_eliminate_point_type = 0;
		grip_info->large_corner_hor_x_width = 0;
		grip_info->large_corner_hor_y_height = 0;
		grip_info->corner_eliminate_without_time = 0;
		GRIP_TP_INFO("corner_eliminate_point_type config using default.\n");
	} else {
		grip_info->corner_eliminate_point_type = temp_array[0];
		grip_info->large_corner_hor_x_width = temp_array[1];
		grip_info->large_corner_hor_y_height = temp_array[2];
		grip_info->corner_eliminate_without_time = temp_array[3];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,long_eliminate_point_type", temp_array, 7);
	if (ret) {
		grip_info->long_eliminate_point_type = 1 << POS_CENTER_INNER;
		grip_info->large_long_x2_width = 0;
		grip_info->large_long_y2_width = 0;
		normal_data->large_long_x1_width = 0;
		grip_info->large_long_y1_width = 0;
		normal_data->large_long_debounce_ms = 300;
		grip_info->long_eliminate_point_support = false;
		GRIP_TP_INFO("long_hold_eliminate_point_type config using default.\n");
	} else {
		grip_info->long_eliminate_point_type = temp_array[0];
		grip_info->large_long_x2_width = temp_array[1];
		grip_info->large_long_y2_width = temp_array[2];
		normal_data->large_long_x1_width = temp_array[3];
		grip_info->large_long_y1_width = temp_array[4];
		normal_data->large_long_debounce_ms = temp_array[5];
		grip_info->long_eliminate_point_support = temp_array[6];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,finger_hold_differ_size", temp_array, 4);
	if (ret) {
		grip_info->finger_hold_differ_size_support = 0;
		grip_info->finger_hold_differ_hor_support = 0;
		normal_data->finger_hold_differ_size_x = 0;
		normal_data->finger_hold_differ_size_debounce_ms = 300;
		GRIP_TP_INFO("finger_hold_differ_size config using default.\n");
	} else {
		grip_info->finger_hold_differ_size_support = temp_array[0];
		grip_info->finger_hold_differ_hor_support = temp_array[1];
		normal_data->finger_hold_differ_size_x = temp_array[2];
		normal_data->finger_hold_differ_size_debounce_ms = temp_array[3];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,finger_hold_max_rx_exit_distance", temp_array, 3);
	if (ret) {
		normal_data->finger_hold_rx_rejec_thd = 0xff;
		normal_data->finger_hold_max_rx_narrow_witdh = grip_info->edge_swipe_narrow_witdh;
		normal_data->finger_hold_max_rx_exit_distance = grip_info->edge_swipe_exit_distance;
		GRIP_TP_INFO("finger_hold_max_rx_exit_distance config using default.\n");
	} else {
		normal_data->finger_hold_rx_rejec_thd = temp_array[0];
		normal_data->finger_hold_max_rx_narrow_witdh = temp_array[1];
		normal_data->finger_hold_max_rx_exit_distance = temp_array[2];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,max_rx_exit_distance", temp_array, 5);
	if (ret) {
		grip_info->max_rx_matched_support = 0;
		normal_data->max_rx_rejec_thd = 0xff;
		normal_data->max_rx_stable_time_thd = 150;
		normal_data->max_rx_narrow_witdh = grip_info->edge_swipe_narrow_witdh;
		normal_data->max_rx_exit_distance = grip_info->edge_swipe_exit_distance;
		GRIP_TP_INFO("max_rx_exit_distance config using default.\n");
	} else {
		grip_info->max_rx_matched_support = temp_array[0];
		normal_data->max_rx_rejec_thd = temp_array[1];
		normal_data->max_rx_stable_time_thd = temp_array[2];
		normal_data->max_rx_narrow_witdh = temp_array[3];
		normal_data->max_rx_exit_distance = temp_array[4];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,dynamic_finger_hold_exit_distance", temp_array, 4);
	if (ret) {
		grip_info->dynamic_finger_hold_exit_support = 0;
		normal_data->dynamic_finger_hold_narrow_witdh = grip_info->edge_swipe_narrow_witdh;
		normal_data->dynamic_finger_hold_exit_distance = grip_info->edge_swipe_exit_distance;
		normal_data->dynamic_finger_hold_size_x = 30;
		GRIP_TP_INFO("dynamic_finger_hold_exit_distance config using default.\n");
	} else {
		grip_info->dynamic_finger_hold_exit_support = temp_array[0];
		normal_data->dynamic_finger_hold_narrow_witdh = temp_array[1];
		normal_data->dynamic_finger_hold_exit_distance = temp_array[2];
		normal_data->dynamic_finger_hold_size_x = temp_array[3];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,edge_sliding_matched_support", temp_array, 4);
	if (ret) {
		grip_info->edge_sliding_matched_support = 0;
		normal_data->edge_sliding_exit_yfsr_thd = 0xffff;
		normal_data->edge_sliding_exit_distance = grip_info->max_y / 3;
		grip_info->edge_swipe_makeup_optimization_support = 0;
		GRIP_TP_INFO("edge_sliding_matched_support config using default.\n");
	} else {
		grip_info->edge_sliding_matched_support = temp_array[0];
		normal_data->edge_sliding_exit_yfsr_thd = temp_array[1];
		normal_data->edge_sliding_exit_distance = grip_info->max_y / temp_array[2];
		grip_info->edge_swipe_makeup_optimization_support = temp_array[3];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,large_corner_judge_condition_ime", temp_array, 5);
	if (ret) {
		grip_info->large_corner_detect_time_ms_ime = grip_info->large_corner_detect_time_ms;
		GRIP_TP_INFO("large corner judge condition using default.\n");
	} else {
		grip_info->large_corner_detect_time_ms_ime = temp_array[0];
	}

	transform_normal_para(grip_info);

	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,reclining_mode_support", temp_array, 1);
	if (ret) {
		grip_info->reclining_mode_support = 0;
		GRIP_TP_INFO("reclining_mode_support config using default.\n");
	} else {
		grip_info->reclining_mode_support = temp_array[0];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,long_eliminate_point_type_recli", temp_array, 7);
	if (ret) {
		reclining_data->large_long_x1_width = normal_data->large_long_x1_width;
		reclining_data->large_long_debounce_ms = normal_data->large_long_debounce_ms;
		GRIP_TP_INFO("long_hold_eliminate_point_type config using default.\n");
	} else {
		reclining_data->large_long_x1_width = temp_array[3];
		reclining_data->large_long_debounce_ms = temp_array[5];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,finger_hold_differ_size_recli", temp_array, 4);
	if (ret) {
		reclining_data->finger_hold_differ_size_x = normal_data->finger_hold_differ_size_x;
		reclining_data->finger_hold_differ_size_debounce_ms = normal_data->finger_hold_differ_size_debounce_ms;
		GRIP_TP_INFO("finger_hold_differ_size config using default.\n");
	} else {
		reclining_data->finger_hold_differ_size_x = temp_array[2];
		reclining_data->finger_hold_differ_size_debounce_ms = temp_array[3];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,finger_hold_max_rx_exit_distance_recli", temp_array, 3);
	if (ret) {
		reclining_data->finger_hold_rx_rejec_thd = normal_data->finger_hold_rx_rejec_thd;
		reclining_data->finger_hold_max_rx_narrow_witdh = normal_data->finger_hold_max_rx_narrow_witdh;
		reclining_data->finger_hold_max_rx_exit_distance = normal_data->finger_hold_max_rx_exit_distance;
		GRIP_TP_INFO("finger_hold_max_rx_exit_distance config using default.\n");
	} else {
		reclining_data->finger_hold_rx_rejec_thd = temp_array[0];
		reclining_data->finger_hold_max_rx_narrow_witdh = temp_array[1];
		reclining_data->finger_hold_max_rx_exit_distance = temp_array[2];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,max_rx_exit_distance_recli", temp_array, 5);
	if (ret) {
		reclining_data->max_rx_rejec_thd = normal_data->max_rx_rejec_thd;
		reclining_data->max_rx_stable_time_thd = normal_data->max_rx_stable_time_thd;
		reclining_data->max_rx_narrow_witdh = normal_data->max_rx_exit_distance;
		reclining_data->max_rx_exit_distance = normal_data->max_rx_exit_distance;
		GRIP_TP_INFO("max_rx_exit_distance config using default.\n");
	} else {
		reclining_data->max_rx_rejec_thd = temp_array[1];
		reclining_data->max_rx_stable_time_thd = temp_array[2];
		reclining_data->max_rx_narrow_witdh = temp_array[3];
		reclining_data->max_rx_exit_distance = temp_array[4];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,dynamic_finger_hold_exit_distance_recli", temp_array, 4);
	if (ret) {
		reclining_data->dynamic_finger_hold_narrow_witdh = normal_data->dynamic_finger_hold_narrow_witdh;
		reclining_data->dynamic_finger_hold_exit_distance = normal_data->dynamic_finger_hold_narrow_witdh;
		reclining_data->dynamic_finger_hold_size_x = normal_data->dynamic_finger_hold_narrow_witdh;
		GRIP_TP_INFO("dynamic_finger_hold_exit_distance config using default.\n");
	} else {
		reclining_data->dynamic_finger_hold_narrow_witdh = temp_array[1];
		reclining_data->dynamic_finger_hold_exit_distance = temp_array[2];
		reclining_data->dynamic_finger_hold_size_x = temp_array[3];
	}
	ret = of_property_read_u32_array(dev->of_node, (char *)"prevention,edge_sliding_matched_support_recli", temp_array, 4);
	if (ret) {
		reclining_data->edge_sliding_exit_yfsr_thd = normal_data->edge_sliding_exit_yfsr_thd;
		reclining_data->edge_sliding_exit_distance = normal_data->edge_sliding_exit_distance;
		GRIP_TP_INFO("edge_sliding_matched_support config using default.\n");
	} else {
		reclining_data->edge_sliding_exit_yfsr_thd = temp_array[1];
		reclining_data->edge_sliding_exit_distance = grip_info->max_y / temp_array[2];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,long_edge_condition_recli", temp_array, 4);
	if (ret) {
		reclining_data->long_start_coupling_thd = normal_data->long_start_coupling_thd;
		reclining_data->long_stable_coupling_thd = normal_data->long_stable_coupling_thd;
		reclining_data->long_hold_changed_thd = normal_data->long_hold_changed_thd;
		reclining_data->long_hold_maxfsr_gap = normal_data->long_hold_maxfsr_gap;
		GRIP_TP_INFO("long_edge_condition_recli using default.\n");
	} else {
		reclining_data->long_start_coupling_thd = temp_array[0];
		reclining_data->long_stable_coupling_thd = temp_array[1];
		reclining_data->long_hold_changed_thd = temp_array[2];
		reclining_data->long_hold_maxfsr_gap = temp_array[3];
	}

	return 0;
}

static int kernel_grip_init_V2(struct kernel_grip_info *grip_info, struct device *dev)
{
	int i_index, j_index, cnt = 0, ret = 0;
	int makeup_para[10] = {0}, temp_array[10] = {0};
	struct reclining_mode_data *normal_data = NULL;

	void (*p_touch_report_work[TOUCH_MAX_NUM])(struct work_struct * work) = {
		touch_report_work0, touch_report_work1, touch_report_work2,
		touch_report_work3, touch_report_work4, touch_report_work5,
		touch_report_work6, touch_report_work7, touch_report_work8,
		touch_report_work9
	};
	enum hrtimer_restart(*p_touch_up_timer_func[TOUCH_MAX_NUM])(
	struct hrtimer * hrtimer) = {
		touch_up_timer_func0, touch_up_timer_func1, touch_up_timer_func2,
		touch_up_timer_func3, touch_up_timer_func4, touch_up_timer_func5,
		touch_up_timer_func6, touch_up_timer_func7, touch_up_timer_func8,
		touch_up_timer_func9
	};

	if (grip_info == NULL) {
		return -1;
	}
	normal_data = &grip_info->normal_data;

	//parameter init
	mutex_init(&grip_info->grip_mutex);
	ret = of_property_read_u32_array(dev->of_node, "touchpanel,panel-coords", temp_array, 2);
	if (ret) {
		grip_info->max_x = 1080;
		grip_info->max_y = 2340;
		GRIP_TP_INFO("panel coords using default.\n");
	} else {
		grip_info->max_x = temp_array[0];
		grip_info->max_y = temp_array[1];
	}

	grip_info->tx_rx_num_exchange_support = of_property_read_bool(dev->of_node, "tx_rx_num_exchange_support");
	ret = of_property_read_u32_array(dev->of_node, "touchpanel,tx-rx-num", temp_array, 2);
	if (ret) {
		grip_info->tx_num = 0;
		grip_info->rx_num = 0;
		GRIP_TP_INFO("panel tx rx not set.\n");
	} else {
		if (grip_info->tx_rx_num_exchange_support) {
			grip_info->tx_num = temp_array[1];
			grip_info->rx_num = temp_array[0];
		} else {
			grip_info->tx_num = temp_array[0];
			grip_info->rx_num = temp_array[1];
		}
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,grip_disable_level", temp_array, 1);
	if (ret) {
		grip_info->grip_disable_level = 0;
		GRIP_TP_INFO("grip disable level using default.\n");
	} else {
		grip_info->grip_disable_level = temp_array[0];
	}

	cnt = of_property_count_elems_of_size(dev->of_node, "prevention,makeup_cnt_weight", sizeof(uint32_t));
	if ((cnt > 0) && (cnt < 10)) {
		ret = of_property_read_u32_array(dev->of_node, "prevention,makeup_cnt_weight", makeup_para, cnt);
	} else {
		ret = -1;
	}
	if (ret || (makeup_para[0] + 1 != cnt)) {
		makeup_para[0] = 4;
		makeup_para[1] = 1;
		makeup_para[2] = 2;
		makeup_para[3] = 2;
		makeup_para[4] = 1;
		GRIP_TP_INFO("makeup cnt and weight using default.\n");
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,updelay_time_ms", temp_array, 1);
	if (ret) {
		grip_info->report_updelay_ms = 30;
		GRIP_TP_INFO("grip updelay time using default.\n");
	} else {
		grip_info->report_updelay_ms = temp_array[0];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,large_corner_range", temp_array, 2);
	if (ret) {
		grip_info->large_corner_width = 200;
		grip_info->large_corner_height = 300;
		GRIP_TP_INFO("large corner range using default.\n");
	} else {
		grip_info->large_corner_width = temp_array[0];
		grip_info->large_corner_height = temp_array[1];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,large_corner_judge_condition", temp_array, 5);
	if (ret) {
		grip_info->large_corner_detect_time_ms = 1000;
		grip_info->large_corner_debounce_ms = 80;
		grip_info->large_corner_exit_distance = 75;
		grip_info->xfsr_corner_exit_thd = 4;
		grip_info->yfsr_corner_exit_thd = 4;
		GRIP_TP_INFO("large corner judge condition using default.\n");
	} else {
		grip_info->large_corner_detect_time_ms = temp_array[0];
		grip_info->large_corner_debounce_ms = temp_array[1];
		grip_info->large_corner_exit_distance = temp_array[2];
		grip_info->xfsr_corner_exit_thd = temp_array[3];
		grip_info->yfsr_corner_exit_thd = temp_array[4];
	}
	grip_info->large_corner_detect_time_ms_restore = grip_info->large_corner_detect_time_ms;

	ret = of_property_read_u32_array(dev->of_node, "prevention,trx_reject_condition", temp_array, 3);
	if (ret) {
		grip_info->trx_reject_thd = 8;
		grip_info->rx_reject_thd = 5;
		grip_info->tx_reject_thd = 5;
		GRIP_TP_INFO("tx rx reject condition using default.\n");
	} else {
		grip_info->trx_reject_thd = temp_array[0];
		grip_info->rx_reject_thd = temp_array[1];
		grip_info->tx_reject_thd = temp_array[2];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,large_judge_time_ms", temp_array, 3);
	if (ret) {
		grip_info->long_detect_time_ms = 400;
		grip_info->large_reject_debounce_time_ms = 60;
		grip_info->fsr_stable_time_thd = 30;
		GRIP_TP_INFO("large judge times using default.\n");
	} else {
		grip_info->long_detect_time_ms = temp_array[0];
		grip_info->large_reject_debounce_time_ms = temp_array[1];
		grip_info->fsr_stable_time_thd = temp_array[2];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,large_exit_condition", temp_array, 3);
	if (ret) {
		grip_info->xfsr_normal_exit_thd = 8;
		grip_info->yfsr_normal_exit_thd = 8;
		grip_info->exit_match_thd = 2;
		GRIP_TP_INFO("large exit condition using default.\n");
	} else {
		grip_info->xfsr_normal_exit_thd = temp_array[0];
		grip_info->yfsr_normal_exit_thd = temp_array[1];
		grip_info->exit_match_thd = temp_array[2];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,single_channel_width", temp_array, 2);
	if (ret) {
		grip_info->single_channel_x_len = 40;
		grip_info->single_channel_y_len = 40;
		GRIP_TP_INFO("single channel width using default.\n");
	} else {
		grip_info->single_channel_x_len = temp_array[0];
		grip_info->single_channel_y_len = temp_array[1];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,normal_tap_condition", temp_array, 2);
	if (ret) {
		grip_info->normal_tap_min_time_ms = 45;
		grip_info->normal_tap_max_time_ms = 150;
		GRIP_TP_INFO("normal tap condition using default.\n");
	} else {
		grip_info->normal_tap_min_time_ms = temp_array[0];
		grip_info->normal_tap_max_time_ms = temp_array[1];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,long_edge_condition", temp_array, 4);
	if (ret) {
		normal_data->long_start_coupling_thd = 205;
		normal_data->long_stable_coupling_thd = 95;
		normal_data->long_hold_changed_thd = 160;
		normal_data->long_hold_maxfsr_gap = 200;
		GRIP_TP_INFO("large long edge hold condition using default.\n");
	} else {
		normal_data->long_start_coupling_thd = temp_array[0];
		normal_data->long_stable_coupling_thd = temp_array[1];
		normal_data->long_hold_changed_thd = temp_array[2];
		normal_data->long_hold_maxfsr_gap = temp_array[3];
	}
	transform_normal_para(grip_info);

	ret = of_property_read_u32_array(dev->of_node, "prevention,long_fingerhold_condition", temp_array, 4);
	if (ret) {
		grip_info->long_hold_debounce_time_ms = 100;
		grip_info->long_hold_divided_factor = 6;    //default value, no need to change it
		grip_info->xfsr_hold_exit_thd = 0;
		grip_info->yfsr_hold_exit_thd = 0;
		GRIP_TP_INFO("large long finger hold condition using default.\n");
	} else {
		grip_info->long_hold_debounce_time_ms = temp_array[0];
		grip_info->long_hold_divided_factor = temp_array[1];
		grip_info->xfsr_hold_exit_thd = temp_array[2];
		grip_info->yfsr_hold_exit_thd = temp_array[3];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,short_edge_condition", temp_array, 4);
	if (ret) {
		grip_info->short_start_coupling_thd = 201;
		grip_info->short_stable_coupling_thd = 91;
		grip_info->short_hold_changed_thd = 160;
		grip_info->short_hold_maxfsr_gap = 200;
		GRIP_TP_INFO("large short edge hold condition using default.\n");
	} else {
		grip_info->short_start_coupling_thd = temp_array[0];
		grip_info->short_stable_coupling_thd = temp_array[1];
		grip_info->short_hold_changed_thd = temp_array[2];
		grip_info->short_hold_maxfsr_gap = temp_array[3];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,top_corner_config", temp_array, 3);
	if (ret) {
		grip_info->large_top_width = 400;
		grip_info->large_top_height = 600;
		grip_info->large_top_exit_distance = 200;
		GRIP_TP_INFO("top corner config using default.\n");
	} else {
		grip_info->large_top_width = temp_array[0];
		grip_info->large_top_height = temp_array[1];
		grip_info->large_top_exit_distance = temp_array[2];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,edge_swipe_config", temp_array, 2);
	if (ret) {
		grip_info->edge_swipe_narrow_witdh = 80;
		grip_info->edge_swipe_exit_distance = 300;
		GRIP_TP_INFO("edge swipe config using default.\n");
	} else {
		grip_info->edge_swipe_narrow_witdh = temp_array[0];
		grip_info->edge_swipe_exit_distance = temp_array[1];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,trx_strict_reject_condition", temp_array, 3);
	if (ret) {
		grip_info->trx_strict_reject_thd = 7;
		grip_info->rx_strict_reject_thd = 5;
		grip_info->tx_strict_reject_thd = 5;
		GRIP_TP_INFO("trx strict reject using default.\n");
	} else {
		grip_info->trx_strict_reject_thd = temp_array[0];
		grip_info->rx_strict_reject_thd = temp_array[1];
		grip_info->tx_strict_reject_thd = temp_array[2];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,long_strict_edge_condition", temp_array, 2);
	if (ret) {
		grip_info->long_strict_start_coupling_thd = 89;
		grip_info->long_strict_stable_coupling_thd = 49;
		GRIP_TP_INFO("long strict edge condition using default.\n");
	} else {
		grip_info->long_strict_start_coupling_thd = temp_array[0];
		grip_info->long_strict_stable_coupling_thd = temp_array[1];
	}


	ret = of_property_read_u32_array(dev->of_node, "prevention,short_strict_edge_condition", temp_array, 2);
	if (ret) {
		grip_info->short_strict_start_coupling_thd = 89;
		grip_info->short_strict_stable_coupling_thd = 49;
		GRIP_TP_INFO("short strict edge condition using default.\n");
	} else {
		grip_info->short_strict_start_coupling_thd = temp_array[0];
		grip_info->short_strict_stable_coupling_thd = temp_array[1];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,large_strict_exit_condition", temp_array, 2);
	if (ret) {
		grip_info->xfsr_strict_exit_thd = grip_info->xfsr_normal_exit_thd;
		grip_info->yfsr_strict_exit_thd = grip_info->xfsr_normal_exit_thd;
		GRIP_TP_INFO("large strict exit condition using default.\n");
	} else {
		grip_info->xfsr_strict_exit_thd = temp_array[0];
		grip_info->yfsr_strict_exit_thd = temp_array[1];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,corner_move_rejected", temp_array, 1);
	if (ret) {
		grip_info->corner_move_rejected = 0;
		GRIP_TP_INFO("corner move rejected using default.\n");
	} else {
		grip_info->corner_move_rejected = temp_array[0];
	}

	grip_info->corner_eliminate_point_support = of_property_read_bool(dev->of_node, "prevention,corner_eliminate_point_support");
	if (grip_info->corner_eliminate_point_support) {
		GRIP_TP_INFO("corner eliminate point enable.\n");
	}

	grip_info->coord_filter_cnt = makeup_para[0];
	grip_info->coord_buf = kzalloc(TOUCH_MAX_NUM * grip_info->coord_filter_cnt * sizeof(struct coord_buffer), GFP_KERNEL);
	if (grip_info->coord_buf) {
		for (i_index = 0; i_index < TOUCH_MAX_NUM; i_index++) {
			for (j_index = 0; j_index < grip_info->coord_filter_cnt; j_index++) {
				grip_info->coord_buf[i_index * grip_info->coord_filter_cnt + j_index].weight = makeup_para[j_index + 1];
			}
		}
	} else {
		kernel_grip_release(grip_info);
		TPD_INFO("kzalloc grip_zone_area for coord_buffer failed.\n");
		return -1;
	}

	ret = kfifo_alloc(&grip_info->up_fifo, PAGE_SIZE, GFP_KERNEL);
	if (ret) {
		kernel_grip_release(grip_info);
		TPD_INFO("up_fifo malloc failed.\n");
		return -1;
	}
	for (i_index = 0; i_index < TOUCH_MAX_NUM; i_index++) {
		hrtimer_init(&grip_info->grip_up_timer[i_index], CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		grip_info->grip_up_timer[i_index].function = p_touch_up_timer_func[i_index];
		INIT_WORK(&grip_info->grip_up_work[i_index], p_touch_report_work[i_index]);
	}
	grip_info->grip_up_handle_wq = create_singlethread_workqueue("touch_up_wq");

	grip_info->is_curved_screen_v4 = of_property_read_bool(dev->of_node, "prevention,curved_screen_V4")
		|| of_property_read_bool(dev->of_node, "prevention,curved_screen_v4");
	if (grip_info->is_curved_screen_v4) {
	    GRIP_TP_INFO("this is is_curved_screen_v4.\n");
	    ret = kernel_grip_init_v4(grip_info, dev);
	    if (ret < 0) {
	    	return 0;
	    }
	}

	/* add for v4.3 */
	grip_info->is_curved_screen_v4_3 = of_property_read_bool(dev->of_node, "prevention,curved_screen_v4_3");
	if (grip_info->is_curved_screen_v4_3) {
		GRIP_TP_INFO("this is is_curved_screen_v4_3.\n");
		ret = kernel_grip_init_v4_3(grip_info, dev);
		if (ret < 0) {
			GRIP_TP_INFO("fail to probe is_curved_screen_v4_3\n");
			return 0;
		}
	} else {
		GRIP_TP_INFO("this is is_curved_screen_v4_3 is %d \n", grip_info->is_curved_screen_v4_3);
	}

	/* add for v4.5 */
	grip_info->is_curved_screen_v4_5 = of_property_read_bool(dev->of_node, "prevention,curved_screen_v4_5");
	if (grip_info->is_curved_screen_v4_5) {
		GRIP_TP_INFO("this is is_curved_screen_v4_5.\n");
		ret = kernel_grip_init_v4_5(grip_info, dev);
		if (ret < 0) {
			GRIP_TP_INFO("fail to probe is_curved_screen_v4_5\n");
			return 0;
		}
	} else {
		GRIP_TP_INFO("this is is_curved_screen_v4_5 is %d \n", grip_info->is_curved_screen_v4_5);
	}

	return 0;
}

struct kernel_grip_info *kernel_grip_init(struct device *dev)
{
	int i = 0, j = 0, ret = 0, large_corner_para[6] = {0};
	int dead_width[2] = {0}, makeup_para[10] = {0}, large_para[3] = {0}, cond_para[2] = {0},
			    no_handle_para[3] = {0}, long_judge_para[20] = {0}, short_judge_para[20] = {0};
	int temp_array[2] = {0}, cond_width[4] = {0}, large_width[2] = {0}, large_corner_width[3] = {0}, eli_width[6] = {0}, curved_large_width[5] = {0};
	struct grip_zone_area *grip_zone = NULL;
	struct kernel_grip_info *grip_info = NULL;

	void (*p_touch_report_work[TOUCH_MAX_NUM])(struct work_struct * work) = {
		touch_report_work0, touch_report_work1, touch_report_work2,
		touch_report_work3, touch_report_work4, touch_report_work5,
		touch_report_work6, touch_report_work7, touch_report_work8,
		touch_report_work9
	};
	enum hrtimer_restart(*p_touch_up_timer_func[TOUCH_MAX_NUM])(
	struct hrtimer * hrtimer) = {
		touch_up_timer_func0, touch_up_timer_func1, touch_up_timer_func2,
		touch_up_timer_func3, touch_up_timer_func4, touch_up_timer_func5,
		touch_up_timer_func6, touch_up_timer_func7, touch_up_timer_func8,
		touch_up_timer_func9
	};
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		TPD_INFO("%s:ts is null.\n", __func__);
		return NULL;
	}

	grip_info = kzalloc(sizeof(struct kernel_grip_info), GFP_KERNEL);

	if (!grip_info) {
		TPD_INFO("kzalloc kernel grip info failed.\n");
		return NULL;
	}

	grip_info->p_ts = ts;
	grip_info->tp_index = ts->tp_index;
	grip_info->coord_buf = NULL;
	grip_info->grip_up_handle_wq = NULL;

	/*parameter init*/
	mutex_init(&grip_info->grip_mutex);
	ret = of_property_read_u32_array(dev->of_node, "touchpanel,panel-coords",
					 temp_array, 2);

	if (ret) {
		grip_info->max_x = 1080;
		grip_info->max_y = 2340;
		GRIP_TP_INFO("panel coords using default.\n");

	} else {
		grip_info->max_x = temp_array[0];
		grip_info->max_y = temp_array[1];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,grip_disable_level",
					 temp_array, 1);

	if (ret) {
		grip_info->grip_disable_level = 0;
		GRIP_TP_INFO("grip disable level using default.\n");

	} else {
		grip_info->grip_disable_level = temp_array[0];
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,no_handle_para",
					 no_handle_para, 3);

	if (ret) {
		no_handle_para[0] = 0;
		no_handle_para[1] = 0;
		no_handle_para[2] = 0;
		GRIP_TP_INFO("grip no handle para using default.\n");
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,dead_area_width",
					 dead_width, 2);

	if (ret) {
		dead_width[0] = 10;
		dead_width[1] = 10;
		GRIP_TP_INFO("panel coords using default.\n");
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,makeup_cnt_weight",
					 makeup_para, 5);

	if (ret) {
		makeup_para[0] = 4;
		makeup_para[1] = 1;
		makeup_para[2] = 2;
		makeup_para[3] = 2;
		makeup_para[4] = 1;
		GRIP_TP_INFO("makeup cnt and weight using default.\n");
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,large_judge_para",
					 large_para, 3);

	if (ret) {
		large_para[0] = 3;
		large_para[1] = 300;
		large_para[2] = 300;
		GRIP_TP_INFO("large judge para using default.\n");
	}

	ret = of_property_read_u32_array(dev->of_node,
					 "prevention,large_corner_judge_para", large_corner_para, 6);

	if (ret) {
		large_corner_para[0] = 10;
		large_corner_para[1] = 2;
		large_corner_para[2] = 2;
		large_corner_para[3] = 30;
		large_corner_para[4] = 30;
		large_corner_para[5] = 1;
		GRIP_TP_INFO("large corner judge para using default.\n");
	}
	grip_info->is_curved_screen = of_property_read_bool(dev->of_node, "prevention,curved_screen");
	if (grip_info->is_curved_screen) {
		GRIP_TP_INFO("this is curved screen.\n");
	}
	grip_info->is_curved_screen_V2 = of_property_read_bool(dev->of_node, "prevention,curved_screen_V2");
	if (grip_info->is_curved_screen_V2) {
		GRIP_TP_INFO("this is curved screen V2.\n");
		ret = kernel_grip_init_V2(grip_info, dev);
		if (ret < 0) {
			return NULL;
		}
		return grip_info;
	}

	if (grip_info->is_curved_screen) {
		ret = of_property_read_u32_array(dev->of_node, "prevention,large_curved_long_judge_para", long_judge_para, 11);
		if (ret) {
			long_judge_para[0] = 7;
			long_judge_para[1] = 90;
			long_judge_para[2] = 55;
			long_judge_para[3] = 30;
			long_judge_para[4] = 30;
			long_judge_para[5] = 20;
			long_judge_para[6] = 4;
			long_judge_para[7] = 7;
			long_judge_para[8] = 5;
			long_judge_para[9] = 3;
			long_judge_para[10] = 3;
			GRIP_TP_INFO("curved large long side judge para using default.\n");
		}

		ret = of_property_read_u32_array(dev->of_node, "prevention,large_curved_short_judge_para", short_judge_para, 11);
		if (ret) {
			short_judge_para[0] = 7;
			short_judge_para[1] = 90;
			short_judge_para[2] = 55;
			short_judge_para[3] = 0;
			short_judge_para[4] = 30;
			short_judge_para[5] = 20;
			short_judge_para[6] = 4;
			short_judge_para[7] = 7;
			short_judge_para[8] = 5;
			short_judge_para[9] = 2;
			short_judge_para[10] = 2;
			GRIP_TP_INFO("curved large corner judge para using default.\n");
		}

		ret = of_property_read_u32_array(dev->of_node, "prevention,curved_large_area_width", curved_large_width, 5);
		if (ret) {
			curved_large_width[0] = 100;
			curved_large_width[1] = 100;
			curved_large_width[2] = 80;
			curved_large_width[3] = 2;
			curved_large_width[4] = 2;
			GRIP_TP_INFO("curved large area width para using default.\n");
		}

		ret = of_property_read_u32_array(dev->of_node, "prevention,grip_large_detect_time", temp_array, 1);
		if (ret) {
			grip_info->large_detect_time_ms = 150;
			GRIP_TP_INFO("grip large detect times using default.\n");
		} else {
			grip_info->large_detect_time_ms = temp_array[0];
		}

		ret = of_property_read_u32_array(dev->of_node, "prevention,grip_down_delta_time", temp_array, 1);
		if (ret) {
			grip_info->down_delta_time_ms = 100;
			GRIP_TP_INFO("grip down delta time using default.\n");
		} else {
			grip_info->down_delta_time_ms = temp_array[0];
		}
	}

	ret = of_property_read_u32_array(dev->of_node,
					 "prevention,condition_judge_para", cond_para, 2);

	if (ret) {
		cond_para[0] = 40;
		cond_para[1] = 50;
		GRIP_TP_INFO("condition judge para using default.\n");
	}

	ret = of_property_read_u32_array(dev->of_node,
					 "prevention,condition_area_width", cond_width, 4);

	if (ret) {
		cond_width[0] = 30;
		cond_width[1] = 30;
		cond_width[2] = 100;
		cond_width[3] = 80;
		GRIP_TP_INFO("condition area width using default.\n");
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,large_area_width",
					 large_width, 2);

	if (ret) {
		large_width[0] = 100;
		large_width[1] = 100;
		GRIP_TP_INFO("large area width using default.\n");
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,large_corner_width",
					 large_corner_width, 3);

	if (ret) {
		large_corner_width[0] = 120;
		large_corner_width[1] = 200;
		large_corner_width[2] = 20;
		GRIP_TP_INFO("large corner width using default.\n");
	}

	ret = of_property_read_u32_array(dev->of_node, "prevention,eli_area_width",
					 eli_width, 6);

	if (ret) {
		eli_width[0] = 120;
		eli_width[1] = 500;
		eli_width[2] = 250;
		eli_width[3] = 120;
		eli_width[4] = 250;
		eli_width[5] = 120;
		GRIP_TP_INFO("eli area width using default.\n");
	}

	grip_info->grip_handle_in_fw = of_property_read_bool(dev->of_node,
				       "prevention,grip_handle_in_fw");

	if (grip_info->grip_handle_in_fw) {
		GRIP_TP_INFO("grip area will handle in fw.\n");
	}


	grip_info->dir_change_set_grip = of_property_read_bool(dev->of_node,
					 "prevention,dir_change_set_grip");

	if (grip_info->dir_change_set_grip) {
		GRIP_TP_INFO("dir_change_set_grip in fw.\n");
	}

	/*dead zone grip init*/
	INIT_LIST_HEAD(&grip_info->dead_zone_list);
	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = 0;
		grip_zone->x_width = dead_width[0];
		grip_zone->y_width = grip_info->max_y;
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "ver_left_dead");
		grip_zone->grip_side = 1 << TYPE_LONG_SIDE;
		grip_zone->support_dir = (1 << VERTICAL_SCREEN) | (1 << LANDSCAPE_SCREEN_90) |
					 (1 << LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->dead_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for ver_left_dead failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - dead_width[0];
		grip_zone->start_y = 0;
		grip_zone->x_width = dead_width[0];
		grip_zone->y_width = grip_info->max_y;
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "ver_right_dead");
		grip_zone->grip_side = 1 << TYPE_LONG_SIDE;
		grip_zone->support_dir = (1 << VERTICAL_SCREEN) | (1 << LANDSCAPE_SCREEN_90) |
					 (1 << LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->dead_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for ver_right_dead failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = 0;
		grip_zone->x_width = grip_info->max_x;
		grip_zone->y_width = dead_width[1];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_left_dead");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 <<
					 LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->dead_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_left_dead failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = grip_info->max_y - dead_width[1];
		grip_zone->x_width = grip_info->max_x;
		grip_zone->y_width = dead_width[1];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_right_dead");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 <<
					 LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->dead_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_right_dead failed.\n");
	}

	grip_info->large_frame_limit = large_para[0];
	grip_info->large_ver_thd = large_para[1];
	grip_info->large_hor_thd = large_para[2];
	grip_info->large_corner_frame_limit = large_corner_para[0];
	grip_info->large_ver_corner_thd = large_corner_para[1];
	grip_info->large_hor_corner_thd = large_corner_para[2];
	grip_info->large_ver_corner_width = large_corner_para[3];
	grip_info->large_hor_corner_width = large_corner_para[4];
	grip_info->large_corner_distance = large_corner_para[5];
	grip_info->condition_frame_limit = cond_para[0];
	grip_info->condition_updelay_ms = cond_para[1];
	grip_info->coord_filter_cnt = makeup_para[0];
	grip_info->no_handle_dir = no_handle_para[0];
	grip_info->no_handle_y1 = no_handle_para[1];
	grip_info->no_handle_y2 = no_handle_para[2];
	if (grip_info->is_curved_screen) {
		grip_info->curved_long_side_para.large_palm_thd_1 = long_judge_para[0];
		grip_info->curved_long_side_para.large_palm_thd_2 = long_judge_para[1];
		grip_info->curved_long_side_para.edge_finger_thd = long_judge_para[2];
		grip_info->curved_long_side_para.hold_finger_thd = long_judge_para[3];
		grip_info->curved_long_side_para.normal_finger_thd_1 = long_judge_para[4];
		grip_info->curved_long_side_para.normal_finger_thd_2 = long_judge_para[5];
		grip_info->curved_long_side_para.normal_finger_thd_3 = long_judge_para[6];
		grip_info->curved_long_side_para.palm_thd_1 = long_judge_para[7];
		grip_info->curved_long_side_para.palm_thd_2 = long_judge_para[8];
		grip_info->curved_long_side_para.small_palm_thd_1 = long_judge_para[9];
		grip_info->curved_long_side_para.small_palm_thd_2 = long_judge_para[10];

		grip_info->curved_short_side_para.large_palm_thd_1 = short_judge_para[0];
		grip_info->curved_short_side_para.large_palm_thd_2 = short_judge_para[1];
		grip_info->curved_short_side_para.edge_finger_thd = short_judge_para[2];
		grip_info->curved_short_side_para.hold_finger_thd = short_judge_para[3];
		grip_info->curved_short_side_para.normal_finger_thd_1 = short_judge_para[4];
		grip_info->curved_short_side_para.normal_finger_thd_2 = short_judge_para[5];
		grip_info->curved_short_side_para.normal_finger_thd_3 = short_judge_para[6];
		grip_info->curved_short_side_para.palm_thd_1 = short_judge_para[7];
		grip_info->curved_short_side_para.palm_thd_2 = short_judge_para[8];
		grip_info->curved_short_side_para.small_palm_thd_1 = short_judge_para[9];
		grip_info->curved_short_side_para.small_palm_thd_2 = short_judge_para[10];
	}
	grip_info->coord_buf = kzalloc(TOUCH_MAX_NUM * grip_info->coord_filter_cnt *
				       sizeof(struct coord_buffer), GFP_KERNEL);

	if (grip_info->coord_buf) {
		for (i = 0; i < TOUCH_MAX_NUM; i++) {
			for (j = 0; j < grip_info->coord_filter_cnt; j++) {
				grip_info->coord_buf[i * grip_info->coord_filter_cnt + j].weight = makeup_para[j
						+ 1];
			}
		}

	} else {
		kernel_grip_release(grip_info);
		TPD_INFO("kzalloc grip_zone_area for coord_buffer failed.\n");
		grip_info = NULL;
		return NULL;
	}

	/*condition grip init*/
	INIT_LIST_HEAD(&grip_info->condition_zone_list);
	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = 0;
		grip_zone->x_width = cond_width[0];
		grip_zone->y_width = grip_info->max_y;
		grip_zone->exit_thd = cond_width[2];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "ver_left_condtion");
		grip_zone->grip_side = 1 << TYPE_LONG_SIDE;
		grip_zone->support_dir = (1 << VERTICAL_SCREEN) | (1 << LANDSCAPE_SCREEN_90) |
					 (1 << LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->condition_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for coord_buffer failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - cond_width[0];
		grip_zone->start_y = 0;
		grip_zone->x_width = cond_width[0];
		grip_zone->y_width = grip_info->max_y;
		grip_zone->exit_thd = cond_width[2];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "ver_right_condtion");
		grip_zone->grip_side = 1 << TYPE_LONG_SIDE;
		grip_zone->support_dir = (1 << VERTICAL_SCREEN) | (1 << LANDSCAPE_SCREEN_90) |
					 (1 << LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->condition_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for ver_right_condtion failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = 0;
		grip_zone->x_width = grip_info->max_x;
		grip_zone->y_width = cond_width[1];
		grip_zone->exit_thd = cond_width[3];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_left_condtion");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 <<
					 LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->condition_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_left_condtion failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = grip_info->max_y - cond_width[1];
		grip_zone->x_width = grip_info->max_x;
		grip_zone->y_width = cond_width[1];
		grip_zone->exit_thd = cond_width[3];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_right_condtion");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 <<
					 LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->condition_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_right_condtion failed.\n");
	}

	/*corner large grip init*/
	INIT_LIST_HEAD(&grip_info->large_zone_list);
	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = 0;
		grip_zone->x_width = large_corner_width[1];
		grip_zone->y_width = large_corner_width[0];
		grip_zone->exit_thd = large_corner_width[2];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor90_left_corner_large");
		grip_zone->grip_side = 1 << TYPE_SHORT_CORNER_SIDE;
		grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 <<
					 LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor90_left_large_corner failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = grip_info->max_y - large_corner_width[0];
		grip_zone->x_width = large_corner_width[1];
		grip_zone->y_width = large_corner_width[0];
		grip_zone->exit_thd = large_corner_width[2];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor90_right_corner_large");
		grip_zone->grip_side = 1 << TYPE_SHORT_CORNER_SIDE;
		grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 <<
					 LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor90_right_large_corner failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - large_corner_width[1];
		grip_zone->start_y = 0;
		grip_zone->x_width = large_corner_width[1];
		grip_zone->y_width = large_corner_width[0];
		grip_zone->exit_thd = large_corner_width[2];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor270_left_corner_large");
		grip_zone->grip_side = 1 << TYPE_SHORT_CORNER_SIDE;
		grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 <<
					 LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor270_left_large_corner failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - large_corner_width[1];
		grip_zone->start_y = grip_info->max_y - large_corner_width[0];
		grip_zone->x_width = large_corner_width[1];
		grip_zone->y_width = large_corner_width[0];
		grip_zone->exit_thd = large_corner_width[2];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor270_right_corner_large");
		grip_zone->grip_side = 1 << TYPE_SHORT_CORNER_SIDE;
		grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 <<
					 LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor270_right_large_corner failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = grip_info->max_y - large_corner_width[1];
		grip_zone->x_width = large_corner_width[0];
		grip_zone->y_width = large_corner_width[1];
		grip_zone->exit_thd = large_corner_width[2];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "ver_left_bottom_large");
		grip_zone->grip_side = 1 << TYPE_LONG_CORNER_SIDE;
		grip_zone->support_dir = 1 << VERTICAL_SCREEN;
		list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for ver_left_bottom_large failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - large_corner_width[0];
		grip_zone->start_y = grip_info->max_y - large_corner_width[1];
		grip_zone->x_width = large_corner_width[0];
		grip_zone->y_width = large_corner_width[1];
		grip_zone->exit_thd = large_corner_width[2];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "ver_right_bottom_large");
		grip_zone->grip_side = 1 << TYPE_LONG_CORNER_SIDE;
		grip_zone->support_dir = 1 << VERTICAL_SCREEN;
		list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for ver_right_bottom_large failed.\n");
	}

	/*large grip init*/
	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = 0;
		grip_zone->x_width = grip_info->max_x;
		grip_zone->y_width = large_width[1];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_left_large");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 <<
					 LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_left_large failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = grip_info->max_y - large_width[1];
		grip_zone->x_width = grip_info->max_x;
		grip_zone->y_width = large_width[1];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_right_large");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 <<
					 LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_right_large failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = 0;
		grip_zone->x_width = large_width[0];
		grip_zone->y_width = grip_info->max_y;
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "ver_left_large");
		grip_zone->grip_side = 1 << TYPE_LONG_SIDE;
		grip_zone->support_dir = (1 << VERTICAL_SCREEN) | (1 << LANDSCAPE_SCREEN_90) |
					 (1 << LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for ver_left_large failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - large_width[0];
		grip_zone->start_y = 0;
		grip_zone->x_width = large_width[0];
		grip_zone->y_width = grip_info->max_y;
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "ver_right_large");
		grip_zone->grip_side = 1 << TYPE_LONG_SIDE;
		grip_zone->support_dir = (1 << VERTICAL_SCREEN) | (1 << LANDSCAPE_SCREEN_90) |
					 (1 << LANDSCAPE_SCREEN_270);
		list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for ver_right_large failed.\n");
	}
	//curved large grip init
	if (grip_info->is_curved_screen) {
		grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);
		if (grip_zone) {
			grip_zone->start_x = 0;
			grip_zone->start_y = 0;
			grip_zone->x_width = grip_info->max_x;
			grip_zone->y_width = curved_large_width[1];
			grip_zone->exit_thd = curved_large_width[2];
			grip_zone->exit_tx_er = curved_large_width[3];
			grip_zone->exit_rx_er = curved_large_width[4];
			snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "curved_hor_left_large");
			grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
			grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 << LANDSCAPE_SCREEN_270);
			list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);
		} else {
			GRIP_TP_INFO("kzalloc grip_zone_area for hor_left_large failed.\n");
		}
		grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);
		if (grip_zone) {
			grip_zone->start_x = 0;
			grip_zone->start_y = grip_info->max_y - curved_large_width[1];
			grip_zone->x_width = grip_info->max_x;
			grip_zone->y_width = curved_large_width[1];
			grip_zone->exit_thd = curved_large_width[2];
			grip_zone->exit_tx_er = curved_large_width[3];
			grip_zone->exit_rx_er = curved_large_width[4];
			snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "curved_hor_right_large");
			grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
			grip_zone->support_dir = (1 << LANDSCAPE_SCREEN_90) | (1 << LANDSCAPE_SCREEN_270);
			list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);
		} else {
			GRIP_TP_INFO("kzalloc grip_zone_area for hor_right_large failed.\n");
		}
		grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);
		if (grip_zone) {
			grip_zone->start_x = 0;
			grip_zone->start_y = 0;
			grip_zone->x_width = curved_large_width[0];
			grip_zone->y_width = grip_info->max_y;
			grip_zone->exit_thd = curved_large_width[2];
			grip_zone->exit_tx_er = curved_large_width[3];
			grip_zone->exit_rx_er = curved_large_width[4];
			snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "curved_ver_left_large");
			grip_zone->grip_side = 1 << TYPE_LONG_SIDE;
			grip_zone->support_dir = (1 << VERTICAL_SCREEN) | (1 << LANDSCAPE_SCREEN_90) | (1 << LANDSCAPE_SCREEN_270);
			list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);
		} else {
			GRIP_TP_INFO("kzalloc grip_zone_area for ver_left_large failed.\n");
		}
		grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);
		if (grip_zone) {
			grip_zone->start_x = grip_info->max_x - curved_large_width[0];
			grip_zone->start_y = 0;
			grip_zone->x_width = curved_large_width[0];
			grip_zone->y_width = grip_info->max_y;
			grip_zone->exit_thd = curved_large_width[2];
			grip_zone->exit_tx_er = curved_large_width[3];
			grip_zone->exit_rx_er = curved_large_width[4];
			snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "curved_ver_right_large");
			grip_zone->grip_side = 1 << TYPE_LONG_SIDE;
			grip_zone->support_dir = (1 << VERTICAL_SCREEN) | (1 << LANDSCAPE_SCREEN_90) | (1 << LANDSCAPE_SCREEN_270);
			list_add_tail(&grip_zone->area_list, &grip_info->large_zone_list);
		} else {
			GRIP_TP_INFO("kzalloc grip_zone_area for ver_right_large failed.\n");
		}
	}

	ret = kfifo_alloc(&grip_info->up_fifo, PAGE_SIZE, GFP_KERNEL);

	if (ret) {
		kernel_grip_release(grip_info);
		TPD_INFO("up_fifo malloc failed.\n");
		grip_info = NULL;
		return NULL;
	}

	for (i = 0; i < TOUCH_MAX_NUM; i++) {
		hrtimer_init(&grip_info->grip_up_timer[i], CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		grip_info->grip_up_timer[i].function = p_touch_up_timer_func[i];
		INIT_WORK(&grip_info->grip_up_work[i], p_touch_report_work[i]);
	}

	grip_info->grip_up_handle_wq = create_singlethread_workqueue("touch_up_wq");

	/*elimination grip init*/
	INIT_LIST_HEAD(&grip_info->elimination_zone_list);
	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = grip_info->max_y - eli_width[1];
		grip_zone->x_width = eli_width[0];
		grip_zone->y_width = eli_width[1];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "ver_left_eli");
		grip_zone->grip_side = 1 << TYPE_LONG_SIDE;
		grip_zone->support_dir = 1 << VERTICAL_SCREEN;
		list_add_tail(&grip_zone->area_list, &grip_info->elimination_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for ver_left_eli failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - eli_width[0];
		grip_zone->start_y = grip_info->max_y - eli_width[1];
		grip_zone->x_width = eli_width[0];
		grip_zone->y_width = eli_width[1];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "ver_right_eli");
		grip_zone->grip_side = 1 << TYPE_LONG_SIDE;
		grip_zone->support_dir = 1 << VERTICAL_SCREEN;
		list_add_tail(&grip_zone->area_list, &grip_info->elimination_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for ver_right_eli failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = 0;
		grip_zone->x_width = eli_width[2];
		grip_zone->y_width = eli_width[3];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_90_left_0_eli");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = 1 << LANDSCAPE_SCREEN_90;
		list_add_tail(&grip_zone->area_list, &grip_info->elimination_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_90_left_0_eli failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = 0;
		grip_zone->x_width = eli_width[5];
		grip_zone->y_width = eli_width[4];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_90_left_1_eli");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = 1 << LANDSCAPE_SCREEN_90;
		list_add_tail(&grip_zone->area_list, &grip_info->elimination_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_90_left_1_eli failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = grip_info->max_y - eli_width[3];
		grip_zone->x_width = eli_width[2];
		grip_zone->y_width = eli_width[3];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_90_right_0_eli");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = 1 << LANDSCAPE_SCREEN_90;
		list_add_tail(&grip_zone->area_list, &grip_info->elimination_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_90_right_0_eli failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = 0;
		grip_zone->start_y = grip_info->max_y - eli_width[4];
		grip_zone->x_width = eli_width[5];
		grip_zone->y_width = eli_width[4];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_90_right_1_eli");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = 1 << LANDSCAPE_SCREEN_90;
		list_add_tail(&grip_zone->area_list, &grip_info->elimination_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_90_right_1_eli failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - eli_width[2];
		grip_zone->start_y = 0;
		grip_zone->x_width = eli_width[2];
		grip_zone->y_width = eli_width[3];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_270_left_0_eli");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = 1 << LANDSCAPE_SCREEN_270;
		list_add_tail(&grip_zone->area_list, &grip_info->elimination_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_270_left_0_eli failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - eli_width[5];
		grip_zone->start_y = 0;
		grip_zone->x_width = eli_width[5];
		grip_zone->y_width = eli_width[4];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_270_left_1_eli");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = 1 << LANDSCAPE_SCREEN_270;
		list_add_tail(&grip_zone->area_list, &grip_info->elimination_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_270_left_1_eli failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - eli_width[2];
		grip_zone->start_y = grip_info->max_y - eli_width[3];
		grip_zone->x_width = eli_width[2];
		grip_zone->y_width = eli_width[3];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_270_right_0_eli");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = 1 << LANDSCAPE_SCREEN_270;
		list_add_tail(&grip_zone->area_list, &grip_info->elimination_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_270_right_0_eli failed.\n");
	}

	grip_zone = kzalloc(sizeof(struct grip_zone_area), GFP_KERNEL);

	if (grip_zone) {
		grip_zone->start_x = grip_info->max_x - eli_width[5];
		grip_zone->start_y = grip_info->max_y - eli_width[4];
		grip_zone->x_width = eli_width[5];
		grip_zone->y_width = eli_width[4];
		snprintf(grip_zone->name, GRIP_TAG_SIZE - 1, "hor_270_right_1_eli");
		grip_zone->grip_side = 1 << TYPE_SHORT_SIDE;
		grip_zone->support_dir = 1 << LANDSCAPE_SCREEN_270;
		list_add_tail(&grip_zone->area_list, &grip_info->elimination_zone_list);

	} else {
		GRIP_TP_INFO("kzalloc grip_zone_area for hor_270_right_1_eli failed.\n");
	}

	return grip_info;
}
