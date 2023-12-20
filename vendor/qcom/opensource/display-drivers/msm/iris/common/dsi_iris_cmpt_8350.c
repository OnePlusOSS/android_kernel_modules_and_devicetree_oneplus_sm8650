// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2022, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2022.
 */


#include <drm/drm_mipi_dsi.h>
#include <video/mipi_display.h>
#include <dsi_drm.h>

#include "dsi_iris_def.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_cmpt.h"


bool iris_is_read_cmd(struct dsi_cmd_desc *pdesc)
{
	if (!pdesc)
		return false;

	return (pdesc->msg.type == MIPI_DSI_DCS_READ);
}

bool iris_is_last_cmd(const struct mipi_dsi_msg *pmsg)
{
	if (!pmsg)
		return false;

	return (pmsg->flags & MIPI_DSI_MSG_LASTCOMMAND);
}

bool iris_is_curmode_cmd_mode(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 mode = pcfg->panel->cur_mode->panel_mode;

	if (mode == DSI_OP_CMD_MODE)
		return true;

	return false;
}

bool iris_is_curmode_vid_mode(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 mode = pcfg->panel->cur_mode->panel_mode;

	if (mode == DSI_OP_VIDEO_MODE)
		return true;

	return false;
}

void iris_set_msg_flags(struct dsi_cmd_desc *pdesc, int type)
{
	if (!pdesc)
		return;

	switch (type) {
	case LAST_FLAG:
		pdesc->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
		break;
	case READ_FLAG:
		pdesc->msg.flags |= MIPI_DSI_MSG_REQ_ACK;
		break;
	case BATCH_FLAG:
		break;
	}
}

int iris_switch_cmd_type(int type)
{
	int s_type = type;

	switch (type) {
	case MIPI_DSI_DCS_COMPRESSION_MODE:
		s_type = MIPI_DSI_DCS_SHORT_WRITE;
		break;
	case MIPI_DSI_PPS_LONG_WRITE:
		s_type = MIPI_DSI_DCS_LONG_WRITE;
		break;
	}
	return s_type;
}

void iris_set_msg_ctrl(struct dsi_cmd_desc *pdesc)
{
	if (!pdesc)
		return;

	pdesc->msg.ctrl = 0;
}

#if defined(PXLW_IRIS_DUAL)
#define CSC_10BIT_OFFSET       4
#define DGM_CSC_MATRIX_SHIFT       0

extern int iris_sspp_subblk_offset(struct sde_hw_pipe *ctx, int s_id, u32 *idx);

void iris_sde_hw_sspp_setup_csc_v2(void *pctx, const void *pfmt, void *pdata)
{
	u32 idx = 0;
	u32 op_mode = 0;
	u32 clamp_shift = 0;
	u32 val;
	u32 op_mode_off = 0;
	bool csc10 = false;
	const struct sde_sspp_sub_blks *sblk;
	struct sde_hw_pipe *ctx = pctx;
	const struct sde_format *fmt = pfmt;
	struct sde_csc_cfg *data = pdata;

	if (!iris_is_dual_supported())
		return;

	if (!ctx || !ctx->cap || !ctx->cap->sblk)
		return;

	if (SDE_FORMAT_IS_YUV(fmt))
		return;

	if (!iris_is_chip_supported())
		return;

	sblk = ctx->cap->sblk;
	if (iris_sspp_subblk_offset(ctx, SDE_SSPP_CSC_10BIT, &idx))
		return;

	op_mode_off = idx;
	if (test_bit(SDE_SSPP_CSC_10BIT, &ctx->cap->features)) {
		idx += CSC_10BIT_OFFSET;
		csc10 = true;
	}
	clamp_shift = csc10 ? 16 : 8;
	if (data && !SDE_FORMAT_IS_YUV(fmt)) {
		op_mode |= BIT(0);
		sde_hw_csc_matrix_coeff_setup(&ctx->hw,
				idx, data, DGM_CSC_MATRIX_SHIFT);
		/* Pre clamp */
		val = (data->csc_pre_lv[0] << clamp_shift) | data->csc_pre_lv[1];
		SDE_REG_WRITE(&ctx->hw, idx + 0x14, val);
		val = (data->csc_pre_lv[2] << clamp_shift) | data->csc_pre_lv[3];
		SDE_REG_WRITE(&ctx->hw, idx + 0x18, val);
		val = (data->csc_pre_lv[4] << clamp_shift) | data->csc_pre_lv[5];
		SDE_REG_WRITE(&ctx->hw, idx + 0x1c, val);

		/* Post clamp */
		val = (data->csc_post_lv[0] << clamp_shift) | data->csc_post_lv[1];
		SDE_REG_WRITE(&ctx->hw, idx + 0x20, val);
		val = (data->csc_post_lv[2] << clamp_shift) | data->csc_post_lv[3];
		SDE_REG_WRITE(&ctx->hw, idx + 0x24, val);
		val = (data->csc_post_lv[4] << clamp_shift) | data->csc_post_lv[5];
		SDE_REG_WRITE(&ctx->hw, idx + 0x28, val);

		/* Pre-Bias */
		SDE_REG_WRITE(&ctx->hw, idx + 0x2c, data->csc_pre_bv[0]);
		SDE_REG_WRITE(&ctx->hw, idx + 0x30, data->csc_pre_bv[1]);
		SDE_REG_WRITE(&ctx->hw, idx + 0x34, data->csc_pre_bv[2]);

		/* Post-Bias */
		SDE_REG_WRITE(&ctx->hw, idx + 0x38, data->csc_post_bv[0]);
		SDE_REG_WRITE(&ctx->hw, idx + 0x3c, data->csc_post_bv[1]);
		SDE_REG_WRITE(&ctx->hw, idx + 0x40, data->csc_post_bv[2]);
	}
	IRIS_LOGVV("%s(), name:%s offset:%x ctx->idx:%x op_mode:%x",
			__func__, sblk->csc_blk.name, idx, ctx->idx, op_mode);
	SDE_REG_WRITE(&ctx->hw, op_mode_off, op_mode);
	wmb();
}
#endif

