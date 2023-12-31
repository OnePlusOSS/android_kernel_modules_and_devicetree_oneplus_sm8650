/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __TPG_HW_V_1_3_DATA_H__
#define __TPG_HW_V_1_3_DATA_H__

#include "../tpg_hw.h"
#include "../tpg_hw_common.h"
#include "tpg_hw_v_1_3.h"
#include "tpg_hw_v_1_3_0.h"
#include "tpg_hw_v_1_3_1.h"

struct tpg_hw_ops tpg_hw_v_1_3_ops = {
	.start = tpg_hw_v_1_3_start,
	.stop  = tpg_hw_v_1_3_stop,
	.init  = tpg_hw_v_1_3_init,
	.process_cmd = tpg_hw_v_1_3_process_cmd,
	.dump_status = tpg_hw_v_1_3_dump_status,
	.write_settings = tpg_hw_write_settings,
};

struct tpg_hw_info tpg_v_1_3_hw_info = {
	.version = TPG_HW_VERSION_1_3,
	.max_vc_channels = 4,
	.max_dt_channels_per_vc = 4,
	.ops = &tpg_hw_v_1_3_ops,
	.hw_data = &cam_tpg103_reg,
	.layer_init = &tpg_1_3_layer_init,
	.xcfa_debug = -1,
	.shdr_overlap = -1,
	.shdr_offset_num_batch = 0,
	.shdr_line_offset0 = 0x6c,
	.shdr_line_offset1 = 0x6c,
};

struct tpg_hw_info tpg_v_1_3_1_hw_info = {
	.version = TPG_HW_VERSION_1_3_1,
	.max_vc_channels = 4,
	.max_dt_channels_per_vc = 4,
	.ops = &tpg_hw_v_1_3_ops,
	.hw_data = &cam_tpg103_1_reg,
	.layer_init = &tpg_1_3_layer_init,
	.xcfa_debug = 0,
	.shdr_overlap = 0,
	.shdr_offset_num_batch = 0,
	.shdr_line_offset0 = 0x6c,
	.shdr_line_offset1 = 0x6c,
};

#endif
