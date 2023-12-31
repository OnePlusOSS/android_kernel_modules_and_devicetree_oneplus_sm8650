/*
 * Copyright (c) 2016-2017,2019-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: cdp_txrx_pflow.h
 * Define the host data path peer flow API functions
 * called by the host control SW and the OS interface module
 */
#ifndef _CDP_TXRX_PFLOW_H_
#define _CDP_TXRX_PFLOW_H_

#include <cdp_txrx_stats_struct.h>
#include "cdp_txrx_ops.h"
#include "cdp_txrx_handle.h"
#include <cdp_txrx_cmn.h>

static inline uint32_t cdp_pflow_update_pdev_params
	(ol_txrx_soc_handle soc, uint8_t pdev_id,
	enum _dp_param_t param, uint32_t val, void *ctx)
{
	if (!soc || !soc->ops) {
		dp_cdp_debug("Invalid Instance");
		QDF_BUG(0);
		return 0;
	}

	if (!soc->ops->pflow_ops ||
	    !soc->ops->pflow_ops->pflow_update_pdev_params)
		return 0;

	return soc->ops->pflow_ops->pflow_update_pdev_params
			(soc, pdev_id, param, val, ctx);
}
#endif
