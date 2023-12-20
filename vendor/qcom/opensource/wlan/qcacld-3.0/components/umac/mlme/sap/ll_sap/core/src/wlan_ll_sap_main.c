/*
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

#include "wlan_ll_sap_main.h"
#include <wlan_objmgr_global_obj.h>
#include "qca_vendor.h"

static QDF_STATUS ll_sap_psoc_obj_created_notification(struct wlan_objmgr_psoc *psoc, void *arg_list)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	ll_sap_debug("ll sap psoc object created");

	/* attach ll_sap_psoc object which will contain cfg items,
	 * tx and rx ops
	 */
	return status;
}

static QDF_STATUS ll_sap_psoc_obj_destroyed_notification(struct wlan_objmgr_psoc *psoc, void *arg_list)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	ll_sap_debug("ll sap psoc object destroyed");

	/* detach ll_sap_psoc object which will contain cfg items,
	 * tx and rx ops
	 */

	return status;
}

QDF_STATUS ll_sap_init(void)
{
	QDF_STATUS status;

	/* register psoc create handler functions. */
	status = wlan_objmgr_register_psoc_create_handler(WLAN_UMAC_COMP_LL_SAP,
							  ll_sap_psoc_obj_created_notification,
							  NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		ll_sap_err("objmgr_register_psoc_create_handler failed");
		return status;
	}

	/* register psoc delete handler functions. */
	status = wlan_objmgr_register_psoc_destroy_handler(WLAN_UMAC_COMP_LL_SAP,
							   ll_sap_psoc_obj_destroyed_notification,
							   NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		ll_sap_err("objmgr_register_psoc_destroy_handler failed");
		wlan_objmgr_unregister_psoc_create_handler(WLAN_UMAC_COMP_LL_SAP,
							   ll_sap_psoc_obj_created_notification,
							   NULL);
	}
	return status;
}

QDF_STATUS ll_sap_deinit(void)
{
	QDF_STATUS ret = QDF_STATUS_SUCCESS, status;

	/* unregister psoc destroy handler functions. */
	status = wlan_objmgr_unregister_psoc_destroy_handler(WLAN_UMAC_COMP_LL_SAP,
							     ll_sap_psoc_obj_destroyed_notification,
							     NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		ll_sap_err("objmgr_deregister_psoc_destroy_handler failed");
		ret = status;
	}

	/* unregister psoc create handler functions. */
	status = wlan_objmgr_unregister_psoc_create_handler(WLAN_UMAC_COMP_LL_SAP,
							    ll_sap_psoc_obj_created_notification,
							    NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		ll_sap_err("objmgr_unregister_psoc_create_handler failed");
		ret = status;
	}

	return ret;
}

QDF_STATUS ll_lt_sap_request_for_audio_transport_switch(
						uint8_t transport_switch_type)
{
	/*
	 * return status as QDF_STATUS_SUCCESS or failure based on the current
	 * pending requests of the transport switch
	 */
	if (transport_switch_type ==
		QCA_WLAN_AUDIO_TRANSPORT_SWITCH_TYPE_NON_WLAN) {
		ll_sap_debug("request SWITCH_TYPE_NON_WLAN accepted");
		return QDF_STATUS_SUCCESS;
	} else if (transport_switch_type ==
				QCA_WLAN_AUDIO_TRANSPORT_SWITCH_TYPE_WLAN) {
		ll_sap_debug("request SWITCH_TYPE_WLAN accepted");
		return QDF_STATUS_SUCCESS;
	}

	return QDF_STATUS_E_RESOURCES;
}
