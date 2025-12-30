/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _TOUCHPANEL_PREVENTION_H_
#define _TOUCHPANEL_PREVENTION_H_

#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>

#define TOUCH_MAX_NUM               (10)
#define GRIP_TAG_SIZE               (64)
#define MAX_STRING_CNT              (15)
#define MAX_AREA_PARAMETER          (10)
#define UP2CANCEL_PRESSURE_VALUE    (0xFF)
#define GRIP_SINGLE_CMD_SIZE        (128)
#define GRIP_ALL_CMD_SIZE           (8192)

#define MAX_FRAME_TIME_INTERVAL 9
#define MIN_FRAME_TIME_INTERVAL 2

typedef enum edge_grip_side {
	TYPE_UNKNOW,
	TYPE_LONG_SIDE,
	TYPE_SHORT_SIDE,
	TYPE_LONG_CORNER_SIDE,
	TYPE_SHORT_CORNER_SIDE,
} grip_side;

/*shows a rectangle
* x start at @start_x and end at @start_x+@x_width
* y start at @start_y and end at @start_y+@y_width
* @area_list: list used for connected with other area
* @start_x: original x coordinate
* @start_y: original y coordinate
* @x_width: witdh in x axis
* @y_width: witdh in y axis
*/
struct grip_zone_area {
	struct list_head area_list;
	/*must be the first member to easy get the pointer of grip_zone_area*/
	uint16_t                start_x;
	uint16_t                start_y;
	uint16_t                x_width;
	uint16_t                y_width;
	uint16_t                exit_thd;
	uint16_t                exit_tx_er;
	uint16_t                exit_rx_er;
	uint16_t                support_dir;
	uint16_t                grip_side;
	char                    name[GRIP_TAG_SIZE];
};

struct coord_buffer {
	uint16_t            x;
	uint16_t            y;
	uint16_t            weight;
};

enum large_judge_status {
	JUDGE_LARGE_CONTINUE,
	JUDGE_LARGE_TIMEOUT,
	JUDGE_LARGE_OK,
};

enum large_reject_type {
	TYPE_REJECT_NONE,
	TYPE_REJECT_HOLD,
	TYPE_REJECT_DONE,
};

enum large_finger_status {
	TYPE_NORMAL_FINGER,
	TYPE_HOLD_FINGER,
	TYPE_EDGE_FINGER,
	TYPE_PALM_SHORT_SIZE,
	TYPE_PALM_LONG_SIZE,
	TYPE_SMALL_PALM_CORNER,
	TYPE_PALM_CORNER,
	TYPE_LARGE_PALM_CORNER,

	TYPE_LONG_AROUND_TOUCH,
	TYPE_LONG_FINGER_HOLD,
	TYPE_LONG_EDGE_FINGER,
	TYPE_LONG_CENTER_DOWN,
	TYPE_SHORT_AROUND_TOUCH,
	TYPE_SHORT_FINGER_HOLD,
	TYPE_SHORT_EDGE_FINGER,
	TYPE_SHORT_CENTER_DOWN,
	TYPE_CORNER_SHAPE_SIZE,
	TYPE_CORNER_LARGE_SIZE,
	TYPE_CORNER_EDGE_FINGER,
	TYPE_CORNER_CENTER_DOWN,
	TYPE_CORNER_SHORT_MOVE,
	TYPE_CORNER_MISTOUCH_AGAIN,
	TYPE_TOP_LONG_PRESS,
	TYPE_ALL_SHORT_CLICK,
	TYPE_LONG_EDGE_TOUCH,
	TYPE_SHORT_EDGE_TOUCH,
	TYPE_TOP_SHORT_MOVE,
};

enum large_point_status {
	UP_POINT,
	DOWN_POINT,
	DOWN_POINT_NEED_MAKEUP,
};

enum grip_diable_level {
	GRIP_DISABLE_LARGE,
	GRIP_DISABLE_ELI,
	GRIP_DISABLE_UP2CANCEL,
};

typedef enum grip_operate_cmd {
	OPERATE_UNKNOW,
	OPERATE_ADD,
	OPERATE_DELTE,
	OPERATE_MODIFY,
} operate_cmd;

typedef enum grip_operate_object {
	OBJECT_UNKNOW,
	OBJECT_PARAMETER,
	OBJECT_PARAMETER_V2,
	OBJECT_LONG_CURVED_PARAMETER,    //for modify curved srceen judge para for long side
	OBJECT_SHORT_CURVED_PARAMETER,   //for modify curved srceen judge para for short side
	OBJECT_CONDITION_AREA,      //for modify condition area
	OBJECT_LARGE_AREA,          //for modify large judge area
	OBJECT_ELI_AREA,            //for modify elimination area
	OBJECT_DEAD_AREA,           //for modify dead area
	OBJECT_SKIP_HANDLE,         //for modify no handle setting
} operate_oject;

enum grip_position {
	POS_UNDEFINE = 0,
	POS_CENTER_INNER = 1,
	POS_LONG_LEFT,
	POS_LONG_RIGHT,
	POS_SHORT_LEFT,
	POS_SHORT_RIGHT,
	POS_VERTICAL_LEFT_CORNER,
	POS_VERTICAL_RIGHT_CORNER,
	POS_VERTICAL_LEFT_TOP,
	POS_VERTICAL_RIGHT_TOP,
	POS_HORIZON_B_LEFT_CORNER,
	POS_HORIZON_B_RIGHT_CORNER,
	POS_HORIZON_T_LEFT_CORNER,
	POS_HORIZON_T_RIGHT_CORNER,
	POS_HORIZON_B_LEFT_TOP,
	POS_HORIZON_B_RIGHT_TOP,
	POS_HORIZON_T_LEFT_TOP,
	POS_HORIZON_T_RIGHT_TOP,
	POS_VERTICAL_MIDDLE_TOP,
	POS_VERTICAL_MIDDLE_BOTTOM,
};

struct grip_point_info {
	uint16_t x;
	uint16_t y;
	uint8_t  tx_press;
	uint8_t  rx_press;
	uint8_t  tx_er;
	uint8_t  rx_er;
	s64 time_ms;  // record the point first down time
};

struct curved_judge_para {
	uint16_t edge_finger_thd;
	uint16_t hold_finger_thd;
	uint16_t normal_finger_thd_1;
	uint16_t normal_finger_thd_2;
	uint16_t normal_finger_thd_3;
	uint16_t large_palm_thd_1;
	uint16_t large_palm_thd_2;
	uint16_t palm_thd_1;
	uint16_t palm_thd_2;
	uint16_t small_palm_thd_1;
	uint16_t small_palm_thd_2;
};

typedef enum point_info_type {
	TYPE_START_POINT = 1,               //for record first frame point
	TYPE_SECOND_POINT,                  //for record second frame point
	TYPE_LAST_POINT,                    //for record last frame points
	TYPE_LATEST_POINT,                  //for record latest points
	TYPE_MAX_TX_POINT,                  //for record max tx frame points
	TYPE_MAX_RX_POINT,                  //for record max rx frame points
	TYPE_INIT_TX_POINT,                 //for record init rx points
	TYPE_INIT_RX_POINT,                 //for record init rx points
	TYPE_RX_CHANGED_POINT,              // for record rx changed frame points
	TYPE_TX_CHANGED_POINT,              // for record tx changed frame points
} point_info_type;

enum center_down_status {
	STATUS_CENTER_UNKNOW = 0,
	STATUS_CENTER_UP,
	STATUS_CENTER_DOWN,
};

enum corner_judge_shape {
	CORNER_SHAPE_NONE = 0,
	CORNER_SHAPE_LARGE,
	CORNER_SHAPE_RATIO,
};

struct key_addr {
	char name[64];
	u64 offset;
};

struct grip_monitor_data{
	u64 vertical_exit_match_y_times;
	u64 landscape_exit_match_y_times;
	u64 vertical_exit_match_x_times;
	u64 landscape_exit_match_x_times;
	u64 finger_hold_matched_times;
	u64 finger_hold_matched_max_times;
	u64 finger_hold_differ_size_times;
	u64 finger_hold_differ_max_times;
	u64 research_point_landed_times;
	u64 research_point_landed_times_force;
	u64 research_point_landed_fail_times;
	u64 vertical_corner_exit_dis_match_times;
	u64 landscape_corner_exit_dis_match_times;
	u64 vertical_corner_press_exit_match_times;
	u64 landscape_corner_press_exit_match_times;
	u64 top_shape_exit_match_times;
	u64 edge_sliding_exit_matched_times;
	u64 edge_sliding_beyond_distance_times;
	u64 vertical_reclining_mode_times;
	u64 landscape_reclining_mode_times;
	u64 vertical_reclining_mode_5s_count;
	u64 vertical_reclining_mode_10s_count;
	u64 vertical_reclining_mode_15s_count;
	u64 landscape_reclining_mode_5s_count;
	u64 landscape_reclining_mode_10s_count;
	u64 landscape_reclining_mode_15s_count;
};

enum touch_reclining {
	NOT_RECLINING_MODE = 0,
	VERTICAL_RECLINING_MODE,
	LANDSCAPE_RECLINING_MODE,
};

struct reclining_mode_data {
	/*v2*/
	uint16_t                  long_start_coupling_thd;              /*long start threshold of rx_er*rx_press*/
	uint16_t                  long_stable_coupling_thd;             /*long stable threshold of rx_er*rx_press*/
	uint16_t                  long_hold_changed_thd;                /*long stable rx_er changed threshold*/
	uint16_t                  long_hold_maxfsr_gap;                 /*long hold fsr gap max setting*/
	/*V4*/
	uint16_t                  large_long_debounce_ms;               /*large pos long hold judge center down debounce time interval for reclining mode*/
	uint16_t                  large_long_x1_width;                  /*large pos long hold x pos for reclining mode*/
	uint16_t                  finger_hold_differ_size_x;            /*finger hold different x size for reclining mode*/
	uint16_t                  finger_hold_differ_size_debounce_ms;  /*finger_hold_differ_size_debounce_ms for reclining mode*/
	uint16_t                  finger_hold_rx_rejec_thd;             /*finger hold max rx reject threshold for reclining mode*/
	uint16_t                  finger_hold_max_rx_exit_distance;     /*finger hold max rx y exit distance for reclining mode*/
	uint16_t                  finger_hold_max_rx_narrow_witdh;      /*finger hold max rx x exit distance for reclining mode*/
	uint16_t                  max_rx_rejec_thd;                     /*max rx reject threshold for reclining mode*/
	uint16_t                  max_rx_stable_time_thd;               /*max rx stable time threshold for reclining mode*/
	uint16_t                  max_rx_exit_distance;                 /*max rx y exit distance for reclining mode*/
	uint16_t                  max_rx_narrow_witdh;                  /*max rx x exit distance for reclining mode*/
	uint16_t                  dynamic_finger_hold_exit_distance;    /*dynamic finger hold y exit distance for reclining mode*/
	uint16_t                  dynamic_finger_hold_narrow_witdh;     /*dynamic finger hold x exit distance for reclining mode*/
	uint16_t                  dynamic_finger_hold_size_x;           /*dynamic finger hold x size for reclining mode*/
	uint16_t                  edge_sliding_exit_yfsr_thd;           /*edge sliding exit with yfsr threshold for reclining mode*/
	uint16_t                  edge_sliding_exit_distance;           /*edge sliding exit with distance threshold for reclining mode*/
};

#define MAKEUP_REAL_POINT (0xFF)
#define POINT_DIFF_CNT  10
struct kernel_grip_info {
	int                       tp_index;
	int                       touch_dir;                              /*shows touchpanel direction*/
	int                       touch_reclining_mode;                   /*shows touchpanel recliing mode*/
	u64                       reclining_start_time;
	int                       last_reclining_mode;                    /*shows last touchpanel recliing mode*/
	uint32_t                  max_x;                                  /*touchpanel width*/
	uint32_t                  max_y;                                  /*touchpanel height*/
	int                       tx_num;                                 /*touchpanel tx num*/
	int                       rx_num;                                 /*touchpanel rx num*/
	struct mutex              grip_mutex;                             /*using for protect grip parameter working*/
	uint32_t                  no_handle_y1;                           /*min y of no grip handle*/
	uint32_t                  no_handle_y2;                           /*max y of no grip handle*/
	uint8_t                   no_handle_dir;                          /*show which side no grip handle*/
	uint16_t                  grip_disable_level;                     /*show whether if do grip handle*/
	uint8_t                   record_total_cnt;                       /*remember total count*/

	struct grip_point_info    first_point[TOUCH_MAX_NUM];             /*store the fist frame point of each ID*/
	struct grip_point_info    second_point[TOUCH_MAX_NUM];            /*store the second frame point of each ID*/
	struct grip_point_info    latest_points[TOUCH_MAX_NUM][POINT_DIFF_CNT];             /*store the latest different 5 points of each ID*/
	bool                      sync_up_makeup[TOUCH_MAX_NUM];          /*shows this point need make up while report up*/
	bool                      dead_out_status[TOUCH_MAX_NUM];         /*show if exit the dead grip*/
	struct list_head          dead_zone_list;                         /*list all area using dead grip strategy*/

	uint16_t frame_cnt[TOUCH_MAX_NUM];     /*show down frame of each id*/
	int obj_prev_bit;                      /*show last frame obj attention*/
	int obj_bit_rcd;                            /*show current frame obj attention for record*/
	int obj_prced_bit_rcd;                      /*show current frame processed obj attention for record*/
	uint16_t coord_filter_cnt;             /*cnt of make up points*/
	struct coord_buffer       *coord_buf;                             /*store point and its weight to do filter*/
	bool                      large_out_status[TOUCH_MAX_NUM];        /*show if exit the large area grip*/
	uint8_t                   large_reject[TOUCH_MAX_NUM];            /*show if rejected state*/
	uint8_t                   large_finger_status[TOUCH_MAX_NUM];     /*show large area finger status for grip strategy*/
	uint8_t                   large_point_status[TOUCH_MAX_NUM];      /*parameter for judge if ponits in large area need make up or not*/
	uint16_t                  large_detect_time_ms;                   /*large area detection time limit in ms*/
	struct list_head          large_zone_list;                        /*list all area using large area grip strategy*/
	struct list_head          condition_zone_list;                    /*list all area using conditional grip strategy*/
	bool                      condition_out_status[TOUCH_MAX_NUM];    /*show if exit the conditional rejection*/
	uint8_t                   makeup_cnt[TOUCH_MAX_NUM];              /*show makeup count of each id*/
	uint16_t                  large_frame_limit;                      /*max time to judge big area*/
	uint16_t                  large_ver_thd;                          /*threshold to determine large area size in vetical*/
	uint16_t                  large_hor_thd;                          /*threshold to determine large area size in horizon*/
	uint16_t                  large_corner_frame_limit;               /*max time to judge big area in corner area*/
	uint16_t                  large_ver_corner_thd;                   /*threshold to determine large area size in vetical corner*/
	uint16_t                  large_hor_corner_thd;                   /*threshold to determine large area size in horizon corner*/
	uint16_t                  large_ver_corner_width;                 /*threshold to determine should be judged in vertical corner way*/
	uint16_t                  large_hor_corner_width;                 /*threshold to determine should be judged in horizon corner way*/
	uint16_t                  large_corner_distance;                  /*threshold to judge move from edge*/

	struct curved_judge_para  curved_long_side_para;                  /*parameter for curved touchscreen long side judge*/
	struct curved_judge_para  curved_short_side_para;                 /*parameter for curved touchscreen short side judge*/
	bool                      is_curved_screen;                       /*curved screen judge flag*/
	s64                       lastest_down_time_ms;                   /*record the lastest down time out of large judged*/
	s64                       down_delta_time_ms;                     /*threshold to judge whether need to make up point*/
	bool point_unmoved[TOUCH_MAX_NUM];     /*show point have already moved*/
	uint8_t condition_frame_limit;         /*keep rejeected while beyond this time*/
	unsigned long condition_updelay_ms;    /*time after to report touch up*/
	struct kfifo up_fifo;      /*store up touch id according  to the sequence*/
	struct hrtimer grip_up_timer[TOUCH_MAX_NUM];/*using for report touch up event*/
	bool grip_hold_status[TOUCH_MAX_NUM];            /*show if this id is in hold status*/
	struct work_struct grip_up_work[TOUCH_MAX_NUM]; /*using for report touch up*/
	struct workqueue_struct *grip_up_handle_wq; /*just for handle report up event*/

	bool eli_out_status[TOUCH_MAX_NUM];     /*store cross range status of each id*/
	bool eli_reject_status[TOUCH_MAX_NUM];  /*show reject status if each id*/
	struct list_head elimination_zone_list;
	/*list all area using elimination strategy*/

	bool grip_handle_in_fw;  /*show whether we should handle prevention in fw*/
	bool dir_change_set_grip;
	struct fw_grip_operations *fw_ops;     /*fw grip setting func call back*/
	struct touchpanel_data *p_ts;         /*record the ts address*/
	int work_id;
	struct grip_monitor_data  grip_moni_data;
	bool                      is_curved_screen_V2;                    /*curved screen judge flag using new grip function*/
	uint8_t                   exit_match_times[TOUCH_MAX_NUM];        /*record actual match the exit condition times*/
	struct grip_point_info    txMax_frame_point[TOUCH_MAX_NUM];       /*record the max tx frame point info*/
	struct grip_point_info    rxMax_frame_point[TOUCH_MAX_NUM];       /*record the max rx frame point info*/
	struct grip_point_info    last_frame_point[TOUCH_MAX_NUM];        /*record the last frame point info*/
	struct grip_point_info    rxChanged_frame_point[TOUCH_MAX_NUM];   /*record the rx changed frame point info*/
	struct grip_point_info    txChanged_frame_point[TOUCH_MAX_NUM];   /*record the tx changed frame point info*/
	uint32_t                  fsr_stable_time[TOUCH_MAX_NUM];         /*record fsr stable time under detect period*/
	uint8_t                   points_pos[TOUCH_MAX_NUM];              /*record point area distribution*/
	uint8_t                   points_center_down[TOUCH_MAX_NUM];      /*record whether points down after center point down*/
	uint8_t                   last_large_reject[TOUCH_MAX_NUM];       /*show last touch rejected state*/
	uint8_t                   last_points_pos[TOUCH_MAX_NUM];         /*show last touch point position*/
	/*add for curved_screen_v4* begin*/
	bool                      is_curved_screen_v4;                       /*wether is_curved_screen_V4*/
	uint16_t                  long_eliminate_point_restore;              /*long size eliminate point enable*/
	uint16_t                  finger_hold_differ_size_restore;           /*finger hold different enable*/
	uint16_t                  finger_hold_max_rx_matched[TOUCH_MAX_NUM]; /*finger hold max rx matched state*/
	uint32_t                  max_rx_stable_time[TOUCH_MAX_NUM];         /*record max_rx stable time under detect period*/
	uint16_t                  max_rx_matched[TOUCH_MAX_NUM];             /*max rx matched state*/
	uint32_t                  max_rx_matched_cnt[TOUCH_MAX_NUM];         /*max rx matched count*/
	uint16_t                  dynamic_finger_hold_state[TOUCH_MAX_NUM];  /*dynamic finger hold state*/
	/*add for curved_screen_v4* end*/
	uint16_t                  large_corner_exit_distance;             /*large corner distance exit condition*/
	uint16_t                  large_corner_detect_time_ms;            /*large corner judge detect time*/
	uint16_t                  large_corner_detect_time_ms_ime;        /*large corner judge detect time for IME*/
	uint16_t                  large_corner_detect_time_ms_restore;    /*large corner judge detect time for IME restore*/
	uint16_t                  large_corner_debounce_ms;               /*large corner judge center down debounce time interval*/
	uint16_t                  large_corner_width;                     /*large corner width*/
	uint16_t                  large_corner_height;                    /*large corner height*/
	uint16_t                  xfsr_corner_exit_thd;                   /*corner exit fsr threshold of x direction*/
	uint16_t                  yfsr_corner_exit_thd;                   /*corner exit fsr threshold of y direction*/
	uint16_t                  exit_match_thd;                         /*exit match times threshold*/
	uint16_t                  rx_reject_thd;                          /*rx reject threshold*/
	uint16_t                  tx_reject_thd;                          /*tx reject threshold*/
	uint16_t                  trx_reject_thd;                         /*tx and rx reject threshold*/
	uint16_t                  fsr_stable_time_thd;                    /*fsr stable time threshold*/
	uint16_t                  single_channel_x_len;                   /*single channel x coordinate*/
	uint16_t                  single_channel_y_len;                   /*single channel y coordinate*/
	uint16_t                  normal_tap_min_time_ms;                 /*normal touch min time interval*/
	uint16_t                  normal_tap_max_time_ms;                 /*normal touch max time interval*/
	/*uint16_t                long_start_coupling_thd;                long start threshold of rx_er*rx_press*/
	/*uint16_t                long_stable_coupling_thd;               long stable threshold of rx_er*rx_press*/
	uint16_t                  long_detect_time_ms;                    /*long detect time threshold*/
	/*uint16_t                long_hold_changed_thd;                  long stable rx_er changed threshold*/
	/*uint16_t                long_hold_maxfsr_gap;                   long hold fsr gap max setting*/
	uint16_t                  long_hold_divided_factor;               /*long hold neighbouring range setting, default 6*/
	uint16_t                  long_hold_debounce_time_ms;             /*long hold debounce time of judge*/
	uint16_t                  xfsr_normal_exit_thd;                   /*normal exit fsr of x direction*/
	uint16_t                  yfsr_normal_exit_thd;                   /*normal exit fsr of y direction*/
	uint16_t                  xfsr_hold_exit_thd;                     /*finger hold exit fsr of x direction*/
	uint16_t                  yfsr_hold_exit_thd;                     /*finger hold exit fsr of y direction*/
	uint16_t                  large_reject_debounce_time_ms;          /*landed in large reject down or up debounce time will be rejected also*/
	uint16_t                  report_updelay_ms;                      /*time after to report touch up*/
	uint16_t                  short_start_coupling_thd;               /*short start threshold of tx_er*tx_press*/
	uint16_t                  short_stable_coupling_thd;              /*short stable threshold of tx_er*tx_press*/
	uint16_t                  short_hold_changed_thd;                 /*short stable tx_er changed threshold*/
	uint16_t                  short_hold_maxfsr_gap;                  /*short hold fsr gap max setting*/
	uint16_t                  large_top_width;                        /*large top corner width*/
	uint16_t                  large_top_height;                       /*large top corner height*/
	uint16_t                  large_top_exit_distance;                /*large top corner exit distance*/
	uint16_t                  edge_swipe_narrow_witdh;                /*normal edge swipe width limit*/
	uint16_t                  edge_swipe_exit_distance;               /*normal edge swipe distance threshold*/
	uint16_t                  long_strict_start_coupling_thd;         /*long side strict start coupling threshold*/
	uint16_t                  long_strict_stable_coupling_thd;        /*long side strict stable coupling threshold*/
	uint16_t                  rx_strict_reject_thd;                   /*long side strict rx reject threshold*/
	uint16_t                  tx_strict_reject_thd;                   /*short side strict tx reject threshold*/
	uint16_t                  trx_strict_reject_thd;                  /*tx and rx add strict reject threshold**/
	uint16_t                  short_strict_start_coupling_thd;        /*short side strict start coupling threshold*/
	uint16_t                  short_strict_stable_coupling_thd;       /*short side strict stable coupling threshold*/
	uint16_t                  xfsr_strict_exit_thd;                   /*strict exit fsr of x direction*/
	uint16_t                  yfsr_strict_exit_thd;                   /*strict exit fsr of y direction*/
	uint16_t                  corner_move_rejected;                   /*flag to show whether we need to do corner move reject*/
	/*add for curved_screen_v4 begin*/
	uint16_t                  long_hold_x_width;                      /*long hold neighbouring x width range setting*/
	uint16_t                  long_hold_y_width;                      /*long hold neighbouring y width range setting*/
	uint16_t                  top_shape_match_times[TOUCH_MAX_NUM];   /*large vertical top shape match times*/
	uint16_t                  top_matched_times_thd;                  /*large vertical top shape match times threshold*/
	uint16_t                  top_matched_xfsr_thd;                   /*large vertical top shape match xsfr threshold*/
	uint16_t                  large_ver_top_exit_distance;            /*large vertical top exit distance*/
	uint16_t                  large_top_middle_width;                 /*large top middle width*/
	uint16_t                  large_top_middle_height;                /*large top middle height*/
	uint16_t                  large_top_middle_exit_distance;         /*large vertical middle exit distance*/
	uint16_t                  large_bottom_middle_support;            /*large bottom middle support or not*/
	uint16_t                  large_hor_top_x_width;                  /*large horizon top corner width*/
	uint16_t                  large_hor_top_y_height;                 /*large horizon top corner height*/
	uint16_t                  finger_hold_matched_hor_support;        /*finger hold matcherd for horizon support*/
	uint16_t                  finger_hold_matched_ver_support;        /*finger hold matcherd for vertical support*/
	uint16_t                  corner_eliminate_point_support;         /*corner eliminate point support*/
	uint16_t                  corner_eliminate_point_type;            /*corner eliminate point type,research_point_landed type*/
	uint16_t                  large_corner_hor_x_width;               /*large corner horizon bottom width*/
	uint16_t                  large_corner_hor_y_height;              /*large corner horizon bottom height*/
	uint16_t                  corner_eliminate_without_time;          /*corner eliminate point without time*/
	uint16_t                  long_eliminate_point_support;           /*long size eliminate point support*/
	/*uint16_t                large_long_debounce_ms;                 large pos long hold judge center down debounce time interval*/
	uint16_t                  long_eliminate_point_type;              /*large pos long hold judge center down y width*/
	uint16_t                  large_long_x2_width;                    /*large pos long hold judge center down x width*/
	uint16_t                  large_long_y2_width;                    /*large pos long hold judge center down y width*/
	/*uint16_t                large_long_x1_width;                    large pos long hold x pos*/
	uint16_t                  large_long_y1_width;                    /*large pos long hold x pos*/
	uint16_t                  finger_hold_differ_size_support;        /*finger hold different support*/
	uint16_t                  finger_hold_differ_hor_support;         /*finger hold different size for horizon support*/
	/*uint16_t                finger_hold_differ_size_x;              finger hold different x size*/
	/*uint16_t                finger_hold_differ_size_debounce_ms;    finger_hold_differ_size_debounce_ms*/
	uint16_t                  set_ime_showing;                        /*is Input Method APP Ime showing or not*/
	/*uint16_t                finger_hold_rx_rejec_thd;               finger hold max rx reject threshold*/
	/*uint16_t                finger_hold_max_rx_exit_distance;       finger hold max rx y exit distance*/
	/*uint16_t                finger_hold_max_rx_narrow_witdh;        finger hold max rx x exit distance*/
	uint16_t                  max_rx_matched_support;                 /*max rx reject threshold*/
	/*uint16_t                max_rx_rejec_thd;                       max rx reject threshold*/
	/*uint16_t                max_rx_stable_time_thd;                 max rx stable time threshold*/
	/*uint16_t                max_rx_exit_distance;                   max rx y exit distance*/
	/*uint16_t                max_rx_narrow_witdh;                    max rx x exit distance*/
	/*uint16_t                dynamic_finger_hold_exit_distance;      dynamic finger hold y exit distance*/
	/*uint16_t                dynamic_finger_hold_narrow_witdh;       dynamic finger hold x exit distance*/
	/*uint16_t                dynamic_finger_hold_size_x;             dynamic finger hold x size*/
	uint16_t                  dynamic_finger_hold_exit_support;       /*dynamic finger hold exit support*/
	uint16_t                  edge_sliding_matched_support;           /*edge sliding support */
	/*uint16_t                edge_sliding_exit_yfsr_thd;             edge sliding exit with yfsr threshold */
	/*uint16_t                edge_sliding_exit_distance;             edge sliding exit with distance threshold*/
	uint16_t                  edge_swipe_makeup_optimization_support; /*edge sliding two points no speed optimization*/
	bool                      tx_rx_num_exchange_support;             /*kernel grip tx rx num exchange support*/
	/*add for curved_screen_v4 end*/
	/*add for curved_screen_v4.2 begin*/
	uint16_t                   reclining_mode_support;                 /*reclining mode support */
	struct reclining_mode_data normal_data;                           /*normal para*/
	struct reclining_mode_data reclining_data;                        /*recliling mode para*/
	struct reclining_mode_data current_data;                          /*current para*/
	/*add for curved_screen_v4.2 end*/
	/* add for grip 4.3 start */
	bool     is_curved_screen_v4_3;                                   /*wether is_curved_screen_V4_3*/
	uint16_t corner_major_minor_check_support;                        /*major minor check function support */
	uint16_t corner_major_minor_percent_thd;                          /*major check with minor*corner_major_minor_percent_thd*/
	uint16_t corner_major_max_thd;                                    /*major large check */
	uint16_t corner_angle_max_range_thd;                              /*not used */
	uint16_t point_edge_degree_check_support;                         /*point edge degree check  support*/
	uint16_t corner_point_edge_percent_thd;                           /*corner point edge percent,edge small*/
	uint16_t bottom_corner_point_edge_percent_thd;                           /*corner point edge percent,edge small*/
	uint16_t corner_point_edge_min_channel_thd;                       /*set to 1*/
	uint16_t top_trx_corner_point_edge_percent_thd;                   /*top trx corner point edge percent,edge small */
	uint16_t bottom_trx_corner_point_edge_percent_thd;                /*bottom trx corner point edge percent,edge small */
	uint16_t corner_point_trx_radio_thd;                              /*point corner trx press*er min limit thd*/
	uint16_t corner_point_check_frame_thd;                            /*point corner frame not used*/
	uint16_t corner_point_peakdelta_stable_thd;                       /*point corner peakdelta stable check thd*/
	uint16_t point_stability_judgment_check_support;                  /*point stable check support*/
	uint16_t point_stability_press_jitter_thd;                        /*point press stable check thd*/
	uint16_t point_stability_er_jitter_thd;                           /*point er stable check thd*/
	uint16_t point_stability_er_jitter_percent_thd;                   /*point er stable percent check thd*/
	int16_t  point_stability_tx_press_jitter_status[TOUCH_MAX_NUM];   /*point tx press stable check status*/
	int16_t  point_stability_tx_er_jitter_status[TOUCH_MAX_NUM];      /*point tx er stable check status*/
	int16_t  point_stability_rx_press_jitter_status[TOUCH_MAX_NUM];   /*point rx press stable check status*/
	int16_t  point_stability_rx_er_jitter_status[TOUCH_MAX_NUM];      /*point rx er stable check status*/
	uint16_t point_corner_er_min_limit_support;                       /*point corner er min limit check support*/
	uint16_t point_corner_tx_er_min_limit_thd;                        /*point corner tx er min limit thd*/
	uint16_t point_corner_rx_er_min_limit_thd;                        /*point corner rx er min limit thd*/
	uint16_t one_frame_down_check_support;                            /*only one frame down check support*/
	uint16_t uniform_make_up_point_v2_support;
	uint16_t uniform_make_up_point_num;
	uint16_t uniform_make_up_exit_point_num;
	uint16_t uniform_max_make_up_time;
	uint16_t uniform_make_up_point_up_time_interval;
	uint16_t uniform_make_up_last_point_percent_limit;
	uint16_t uniform_make_up_last_input_point_x[TOUCH_MAX_NUM];
	uint16_t uniform_make_up_last_input_point_y[TOUCH_MAX_NUM];
	int32_t  uniform_make_up_x_shift_initial[TOUCH_MAX_NUM];
	int32_t  uniform_make_up_y_shift_initial[TOUCH_MAX_NUM];
	int32_t  uniform_make_up_x_shift_leftover[TOUCH_MAX_NUM];
	int32_t  uniform_make_up_y_shift_leftover[TOUCH_MAX_NUM];
	uint16_t uniform_last_make_up_prevent_out_x[TOUCH_MAX_NUM];
	uint16_t uniform_last_make_up_prevent_out_y[TOUCH_MAX_NUM];
	uint16_t uniform_make_up_point_num_status[TOUCH_MAX_NUM];
	int32_t  uniform_make_up_point_x_move_speed[TOUCH_MAX_NUM];
	int32_t  uniform_make_up_point_y_move_speed[TOUCH_MAX_NUM];
	s64  uniform_make_up_point_last_down_time[TOUCH_MAX_NUM];
	uint16_t uniform_make_up_point_upid_status;
	/* add for grip 4.3 end */
	/* add for grip 4.4 start */
	uint8_t  point_touchZ_stability_status[TOUCH_MAX_NUM];
	uint16_t touch_point_stability_max_time;
	uint16_t middle_bottom_long_press_time_thd;
	uint16_t middle_bottom_long_press_width_min_thd;
	uint16_t middle_bottom_long_press_minor_max_thd;
	uint8_t  middle_bottom_long_press_match_times_thd;
	uint16_t large_middle_bottom_exit_quickly_support;
	uint8_t  middle_bottom_long_press_match_times[TOUCH_MAX_NUM];
	uint16_t middle_bottom_small_press_width_max_thd;
	uint16_t middle_bottom_small_press_er_min_thd;
	uint16_t middle_bottom_small_press_peakdelta_max_thd;
	uint16_t middle_bottom_small_press_minor_max_thd;
	uint8_t  middle_bottom_small_press_match_times_thd;
	uint8_t  middle_bottom_small_press_match_times[TOUCH_MAX_NUM];
	uint16_t middle_bottom_exit_x_dis_thd;
	uint16_t middle_bottom_exit_y_dis_thd;
	uint16_t middle_bottom_match_exit_x_dis_percent_thd;
	uint16_t middle_bottom_match_exit_y_dis_thd;
	uint16_t middle_bottom_short_exit_yfsr_coupling_thd;
	uint16_t middle_bottom_long_exit_yfsr_coupling_thd;
	uint8_t  middle_bottom_normal_press_thd;
	uint8_t  middle_bottom_exit_yfsr_coupling_times_thd;
	uint8_t  middle_bottom_exit_yfsr_coupling_times[TOUCH_MAX_NUM];
	/* add for grip 4.4 end */
	/* add for grip 4.5 start */
	bool     is_curved_screen_v4_5;                                   /*wether is_curved_screen_V4_5*/
	uint16_t long_press_top_corner_point_edge_percent_thd;
	uint16_t long_press_bottom_corner_point_edge_percent_thd;
	uint16_t long_press_top_corner_point_edge_min_channel_thd;
	uint16_t long_press_bottom_corner_point_edge_min_channel_thd;
	uint16_t long_press_top_trx_corner_point_edge_percent_thd;
	uint16_t long_press_bottom_trx_corner_point_edge_percent_thd;
	uint16_t long_press_time_thd;
	uint16_t long_press_point_edge_degree_check_support;
	uint16_t center_down_top_corner_point_edge_percent_thd;
	uint16_t center_down_bottom_corner_point_edge_percent_thd;
	uint16_t center_down_top_corner_point_edge_min_channel_thd;
	uint16_t center_down_bottom_corner_point_edge_min_channel_thd;
	uint16_t center_down_top_trx_corner_point_edge_percent_thd;
	uint16_t center_down_bottom_trx_corner_point_edge_percent_thd;
	uint16_t center_down_point_edge_degree_check_support;
	uint16_t soon_meet_top_corner_point_edge_percent_thd;
	uint16_t soon_meet_bottom_corner_point_edge_percent_thd;
	uint16_t soon_meet_top_corner_point_edge_min_channel_thd;
	uint16_t soon_meet_bottom_corner_point_edge_min_channel_thd;
	uint16_t soon_meet_top_trx_corner_point_edge_percent_thd;
	uint16_t soon_meet_bottom_trx_corner_point_edge_percent_thd;
	uint16_t soon_meet_min_level_thd;
	uint16_t soon_meet_point_edge_degree_check_support;
	uint16_t soon_meet_level[TOUCH_MAX_NUM];
	/* add for grip 4.5 end */
};

struct fw_grip_operations {
	int (*set_fw_grip_area)(void *chip_data, struct grip_zone_area *grip_zone,
				bool enable);	/*set the fw grip area*/
	void (*set_touch_direction)(void *chip_data,
				    uint8_t dir);	/*set touch direction in fw*/
	int (*set_no_handle_area)(void *chip_data,
				  struct kernel_grip_info *grip_info);	/*set no handle area in fw*/
	int (*set_condition_frame_limit)(void *chip_data,
					 int frame);	/*set condition frame limit in fw*/
	int (*set_large_frame_limit)(void *chip_data,
				     int frame);	/*set large frame limit in fw*/
	int (*set_large_corner_frame_limit)(void *chip_data,
					    int frame);	/*set large condition frame limit in fw*/
	int (*set_large_ver_thd)(void *chip_data, int thd); /*set large ver thd in fw*/
	int (*set_disable_level)(void *chip_data, uint8_t level);
};

struct kernel_grip_info *kernel_grip_init(struct device *dev);
void init_kernel_grip_proc(struct proc_dir_entry *prEntry_tp,
			   struct kernel_grip_info *grip_info);
void grip_status_reset(struct kernel_grip_info *grip_info, uint8_t index);
void kernel_grip_reset(struct kernel_grip_info *grip_info);
int kernel_grip_print_func(struct seq_file *s,
			   struct kernel_grip_info *grip_info);
int notify_prevention_handle(struct kernel_grip_info *grip_info,
			     int obj_attention, struct point_info *points);

#endif /*_TOUCHPANEL_PREVENTION_H_*/

