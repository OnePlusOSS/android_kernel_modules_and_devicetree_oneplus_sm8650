/*
 * Copyright (c) 2018-2019 The Linux Foundation. All rights reserved.
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
 * DOC: wbuff.h
 * wbuff buffer management APIs
 */

#ifndef _WBUFF_H
#define _WBUFF_H

#include <qdf_status.h>
#include <qdf_nbuf.h>

/* Number of pools supported per module */
#define WBUFF_MAX_POOLS 16
#define WBUFF_MAX_POOL_ID WBUFF_MAX_POOLS

enum wbuff_module_id {
	WBUFF_MODULE_WMI_TX,
	WBUFF_MODULE_CE_RX,
	WBUFF_MAX_MODULES,
};

/**
 * struct wbuff_alloc_request - allocation structure for registering each
 * pool for wbuff module.
 * @pool_id: pool identifier
 * @pool_size: number of buffers for @pool_id
 * @buffer_size: size of each buffer in this @pool_id
 */
struct wbuff_alloc_request {
	uint8_t pool_id;
	uint16_t pool_size;
	uint16_t buffer_size;
};

/* Opaque handle for wbuff */
struct wbuff_mod_handle;

#ifdef WLAN_FEATURE_WBUFF
/**
 * wbuff_module_init() - Initializes the wbuff module
 *
 * Return: QDF_STATUS_SUCCESS - init success
 *         QDF_STATUS_E_NOSUPPORT - init failure
 */
QDF_STATUS wbuff_module_init(void);

/**
 * wbuff_module_deinit() - De-initializes the wbuff module
 *
 * Return: QDF_STATUS_SUCCESS - de-init success
 *         QDF_STATUS_E_INVAL - de-init failure (wbuff not initialized)
 */
QDF_STATUS wbuff_module_deinit(void);

/**
 * wbuff_module_register() - Registers a module with wbuff
 * @req: allocation request from registered module
 * @num_pools: number of pools required
 * @reserve: nbuf headroom to start with
 * @align: alignment for the nbuf
 * @module_id: module identifier
 *
 * Return: Handle if registration success
 *         NULL if registration failure
 */
struct wbuff_mod_handle *
wbuff_module_register(struct wbuff_alloc_request *req, uint8_t num_pools,
		      int reserve, int align, enum wbuff_module_id module_id);

/**
 * wbuff_module_deregister() - De-registers a module with wbuff
 * @hdl: wbuff_handle corresponding to the module
 *
 * Return: QDF_STATUS_SUCCESS - deregistration success
 *         QDF_STATUS_E_INVAL - deregistration failure
 */
QDF_STATUS wbuff_module_deregister(struct wbuff_mod_handle *hdl);

/**
 * wbuff_buff_get() - return buffer to the requester
 * @hdl: wbuff_handle corresponding to the module
 * @pool_id: pool identifier
 * @len: length of buffer requested
 * @func_name: function from which buffer is requested
 * @line_num: line number in the file
 *
 * Return: Network buffer if success
 *         NULL if failure
 */
qdf_nbuf_t
wbuff_buff_get(struct wbuff_mod_handle *hdl, uint8_t pool_id, uint32_t len,
	       const char *func_name, uint32_t line_num);

/**
 * wbuff_buff_put() - put the buffer back to wbuff pool
 * @buf: pointer to network buffer
 *
 * Return: NULL if success (buffer consumed)
 *         @buf if failure (buffer not consumed)
 */
qdf_nbuf_t wbuff_buff_put(qdf_nbuf_t buf);

#else

static inline QDF_STATUS wbuff_module_init(void)
{
	return QDF_STATUS_E_NOSUPPORT;
}

static inline QDF_STATUS wbuff_module_deinit(void)
{
	return QDF_STATUS_E_NOSUPPORT;
}

static inline struct wbuff_mod_handle *
wbuff_module_register(struct wbuff_alloc_request *req, uint8_t num_pools,
		      int reserve, int align, enum wbuff_module_id module_id)
{
	return NULL;
}

static inline QDF_STATUS wbuff_module_deregister(struct wbuff_mod_handle *hdl)
{
	return QDF_STATUS_E_NOSUPPORT;
}

static inline qdf_nbuf_t
wbuff_buff_get(struct wbuff_mod_handle *hdl, uint8_t pool_id, uint32_t len,
	       const char *func_name, uint32_t line_num)
{
	return NULL;
}

static inline qdf_nbuf_t
wbuff_buff_put(qdf_nbuf_t buf)
{
	return buf;
}

#endif
#endif /* _WBUFF_H */
