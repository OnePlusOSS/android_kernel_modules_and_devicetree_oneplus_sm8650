/*
 * Copyright (c) 2019 The Linux Foundation. All rights reserved.
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
 * DOC: target interface APIs for fw offload
 *
 */

#ifndef __TARGET_IF_FWOL_H__
#define __TARGET_IF_FWOL_H__

/**
 * target_if_fwol_register_event_handler() - register fw offload event handler
 * @psoc: psoc object
 * @arg: argument passed to lmac
 *
 * Return: QDF_STATUS
 */
QDF_STATUS target_if_fwol_register_event_handler(struct wlan_objmgr_psoc *psoc,
						 void *arg);

/**
 * target_if_fwol_unregister_event_handler() - unregister fw offload event
 * handler
 * @psoc: psoc object
 * @arg: argument passed to lmac
 *
 * Return: QDF_STATUS
 */
QDF_STATUS
target_if_fwol_unregister_event_handler(struct wlan_objmgr_psoc *psoc,
					void *arg);

/**
 * target_if_fwol_register_tx_ops() - register fw offload tx ops callback
 * functions
 * @tx_ops: fw offload tx operations
 *
 * Return: QDF_STATUS
 */
QDF_STATUS target_if_fwol_register_tx_ops(struct wlan_fwol_tx_ops *tx_ops);

/**
 * target_if_fwol_notify_thermal_throttle() - Notify thermal throttle level
 * to upper layer
 * @psoc: PSOC object manager
 * @info: Thermal throttle information from target
 *
 * This function is used to notify thermal throttle level to upper layer
 * when thermal management event receive.
 *
 * Return: QDF_STATUS_SUCCESS for success otherwise failure
 */
#ifdef FW_THERMAL_THROTTLE_SUPPORT
QDF_STATUS
target_if_fwol_notify_thermal_throttle(struct wlan_objmgr_psoc *psoc,
				       struct thermal_throttle_info *info);
#else
static inline QDF_STATUS
target_if_fwol_notify_thermal_throttle(struct wlan_objmgr_psoc *psoc,
				       struct thermal_throttle_info *info)
{
	return QDF_STATUS_E_INVAL;
}
#endif
#endif /* __TARGET_IF_FWOL_H__ */
