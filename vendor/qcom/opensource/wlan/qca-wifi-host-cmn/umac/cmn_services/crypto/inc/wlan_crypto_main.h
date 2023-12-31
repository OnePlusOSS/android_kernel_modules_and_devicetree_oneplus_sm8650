/*
 * Copyright (c) 2017-2018, 2021 The Linux Foundation. All rights reserved.
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
 * DOC: Private API for crypto service with object manager handler
 */
#ifndef _WLAN_CRYPTO_MAIN_H_
#define _WLAN_CRYPTO_MAIN_H_
#include "wlan_crypto_global_def.h"

/**
 * wlan_crypto_init() - Init the crypto service with object manager
 *                      Called from umac init context.
 *
 * Return: QDF_STATUS_SUCCESS - in case of success
 */
QDF_STATUS wlan_crypto_init(void);

/**
 * wlan_crypto_deinit() - Deinit the crypto service with object manager
 *                        Called from umac deinit context.
 *
 * Return: QDF_STATUS_SUCCESS - in case of success
 */
QDF_STATUS wlan_crypto_deinit(void);

#ifdef CRYPTO_SET_KEY_CONVERGED
/**
 * wlan_crypto_psoc_enable() - psoc enable API for wlan crypto component
 * @psoc: pointer to PSOC
 *
 * Return: status of operation
 */
QDF_STATUS wlan_crypto_psoc_enable(struct wlan_objmgr_psoc *psoc);

/**
 * wlan_crypto_psoc_disable() - psoc disable API for wlan crypto component
 * @psoc: pointer to PSOC
 *
 * Return: status of operation
 */
QDF_STATUS wlan_crypto_psoc_disable(struct wlan_objmgr_psoc *psoc);
#else
static inline QDF_STATUS wlan_crypto_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static inline QDF_STATUS wlan_crypto_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}
#endif

#endif /* end of _WLAN_CRYPTO_MAIN_H_ */
