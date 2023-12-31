/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "wmi_unified_priv.h"
#include "wmi_unified_cfr_api.h"

#ifdef WLAN_CFR_ENABLE

QDF_STATUS wmi_unified_send_peer_cfr_capture_cmd(wmi_unified_t wmi_handle,
						 struct peer_cfr_params *param)
{
	if (wmi_handle->ops->send_peer_cfr_capture_cmd)
		return wmi_handle->ops->send_peer_cfr_capture_cmd(wmi_handle,
								  param);

	return QDF_STATUS_E_FAILURE;
}

QDF_STATUS
wmi_extract_cfr_peer_tx_event_param(wmi_unified_t wmi_handle, void *evt_buf,
				    wmi_cfr_peer_tx_event_param *peer_tx_event)
{
	if (wmi_handle->ops->extract_cfr_peer_tx_event_param)
		return wmi_handle->ops->extract_cfr_peer_tx_event_param(
							wmi_handle,
							evt_buf,
							peer_tx_event);
	return QDF_STATUS_E_FAILURE;
}

#ifdef WLAN_ENH_CFR_ENABLE
QDF_STATUS wmi_unified_send_cfr_rcc_cmd(wmi_unified_t wmi_handle,
					struct cfr_rcc_param *cfg)
{
	if (wmi_handle->ops->send_cfr_rcc_cmd)
		return wmi_handle->ops->send_cfr_rcc_cmd(wmi_handle, cfg);
	return QDF_STATUS_E_FAILURE;
}

QDF_STATUS
wmi_extract_cfr_pdev_phase_delta_event(wmi_unified_t wmi_handle,
				       void *evt_buf,
				       struct wmi_cfr_phase_delta_param *param)
{
	if (wmi_handle->ops->extract_cfr_phase_param)
		return wmi_handle->ops->extract_cfr_phase_param(wmi_handle,
								evt_buf,
								param);
	return QDF_STATUS_E_FAILURE;
}

#ifdef WLAN_RCC_ENHANCED_AOA_SUPPORT
QDF_STATUS
wmi_extract_cfr_pdev_enhanced_aoa_phasedelta_event_fixed_param(
		wmi_unified_t wmi_handle,
		void *evt_buf,
		struct wmi_cfr_enh_phase_delta_param *param)
{
	if (wmi_handle->ops->extract_cfr_enh_phase_fixed_param)
		return wmi_handle->ops->extract_cfr_enh_phase_fixed_param
				(wmi_handle, evt_buf, param);

	return QDF_STATUS_E_FAILURE;
}

QDF_STATUS
wmi_extract_cfr_pdev_enhanced_aoa_phasedelta_event_data(
		wmi_unified_t wmi_handle, void *evt_buf,
		struct wmi_cfr_enh_phase_delta_param *param)
{
	if (wmi_handle->ops->extract_cfr_enh_phase_data)
		return wmi_handle->ops->extract_cfr_enh_phase_data(wmi_handle,
								   evt_buf,
								   param);

	return QDF_STATUS_E_FAILURE;
}
#endif /* WLAN_RCC_ENHANCED_AOA_SUPPORT */
#endif /* WLAN_ENH_CFR_ENABLE */
#endif /* WLAN_CFR_ENABLE */
