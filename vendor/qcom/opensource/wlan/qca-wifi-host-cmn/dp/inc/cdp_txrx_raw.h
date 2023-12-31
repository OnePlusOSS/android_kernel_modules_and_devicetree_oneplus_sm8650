/*
 * Copyright (c) 2016-2017, 2019, 2021 The Linux Foundation. All rights reserved.
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
 * DOC: cdp_txrx_raw.h
 * Define the host data path raw mode API functions
 * called by the host control SW and the OS interface module
 */
#ifndef _CDP_TXRX_RAW_H_
#define _CDP_TXRX_RAW_H_

#include "cdp_txrx_handle.h"
#include "cdp_txrx_ops.h"
#include <cdp_txrx_cmn.h>

/**
 * cdp_rawsim_get_astentry() - finds the ast entry for the packet
 * @soc: soc handle
 * @vdev_id: id of the data virtual device object
 * @pnbuf: pointer to nbuf
 * @raw_ast: pointer to fill ast information
 *
 * Finds the ast entry i.e 4th address for the packet based on the
 * details in the netbuf.
 *
 * Return: 0 on success, -1 on error, 1 if more nbufs need to be consumed.
 */

static inline QDF_STATUS
cdp_rawsim_get_astentry(ol_txrx_soc_handle soc, uint8_t vdev_id,
			qdf_nbuf_t *pnbuf, struct cdp_raw_ast *raw_ast)
{

	if (!soc || !soc->ops) {
		dp_cdp_debug("Invalid Instance");
		QDF_BUG(0);
		return QDF_STATUS_E_FAILURE;
	}

	if (!soc->ops->raw_ops ||
	    !soc->ops->raw_ops->rsim_get_astentry)
		return QDF_STATUS_E_FAILURE;

	return soc->ops->raw_ops->rsim_get_astentry(soc, vdev_id,
						    pnbuf, raw_ast);
}

#endif
