/*
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
 * DOC: This file contains ll_sap north bound interface declarations
 */

#ifndef _WLAN_LL_SAP_UCFG_API_H_
#define _WLAN_LL_SAP_UCFG_API_H_

#ifdef WLAN_FEATURE_LL_LT_SAP

/**
 * ucfg_ll_sap_init() - initializes ll_sap component
 *
 * Return: QDF status
 */
QDF_STATUS ucfg_ll_sap_init(void);

/**
 * ucfg_ll_sap_deinit() - De-initializes ll_sap component
 *
 * Return: QDF status
 */
QDF_STATUS ucfg_ll_sap_deinit(void);

/**
 * ucfg_is_ll_lt_sap_supported() - Check if ll_lt_sap is supported or not
 *
 * Return: True/False
 */
bool ucfg_is_ll_lt_sap_supported(void);

/**
 * ucfg_ll_lt_sap_request_for_audio_transport_switch() - Request to switch the
 * audio transport medium
 * @transport_switch_type: Requested transport switch type
 *
 * Return: Accepted/Rejected
 */
QDF_STATUS ucfg_ll_lt_sap_request_for_audio_transport_switch(
						uint8_t transport_switch_type);

#else
static inline QDF_STATUS ucfg_ll_sap_init(void)
{
	return QDF_STATUS_SUCCESS;
}

static inline QDF_STATUS ucfg_ll_sap_deinit(void)
{
	return QDF_STATUS_SUCCESS;
}

static inline bool ucfg_is_ll_lt_sap_supported(void)
{
	return false;
}

static inline QDF_STATUS
ucfg_ll_lt_sap_request_for_audio_transport_switch(uint8_t transport_switch_type)
{
	return QDF_STATUS_E_INVAL;
}

#endif /* WLAN_FEATURE_LL_LT_SAP */
#endif /* _WLAN_LL_SAP_UCFG_API_H_ */

