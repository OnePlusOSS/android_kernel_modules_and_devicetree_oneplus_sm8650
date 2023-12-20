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
 * DOC: contains ll_lt_sap_definitions specific to the ll_lt_sap module
 */

#include "wlan_hdd_ll_lt_sap.h"
#include "wlan_ll_sap_ucfg_api.h"
#include "osif_sync.h"
#include "wlan_hdd_cfg80211.h"

const struct nla_policy
	wlan_hdd_ll_lt_sap_transport_switch_policy
	[QCA_WLAN_VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_MAX + 1] = {
		[QCA_WLAN_VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_TYPE] = {
						.type = NLA_U8},
		[QCA_WLAN_VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_STATUS] = {
						.type = NLA_U8},
};

/**
 * __wlan_hdd_cfg80211_ll_lt_sap_transport_switch() - Request to switch the
 * transport
 * @wiphy:   pointer to wireless wiphy structure.
 * @wdev:    pointer to wireless_dev structure.
 * @data:    Pointer to the data to be passed via vendor interface
 * @data_len:Length of the data to be passed
 *
 * Return:   Return the Success or Failure code.
 */
static int
__wlan_hdd_cfg80211_ll_lt_sap_transport_switch(struct wiphy *wiphy,
					       struct wireless_dev *wdev,
					       const void *data,
					       int data_len)
{
	struct net_device *dev = wdev->netdev;
	struct hdd_context *hdd_ctx = wiphy_priv(wiphy);
	struct hdd_adapter *adapter = WLAN_HDD_GET_PRIV_PTR(dev);
	struct nlattr *tb[QCA_WLAN_VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_MAX + 1];
	uint8_t transport_switch_type;
	uint8_t transport_switch_status;
	QDF_STATUS status;

	hdd_enter_dev(dev);

	if (QDF_GLOBAL_FTM_MODE == hdd_get_conparam()) {
		hdd_err("Command not allowed in FTM mode");
		return -EPERM;
	}

	if (0 != wlan_hdd_validate_context(hdd_ctx))
		return -EINVAL;

	if (!adapter)
		return -EINVAL;

	if (wlan_cfg80211_nla_parse(
			tb, QCA_WLAN_VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_MAX,
			data, data_len,
			wlan_hdd_ll_lt_sap_transport_switch_policy)) {
		hdd_err("Invalid attribute");
		return -EINVAL;
	}

	if (!tb[QCA_WLAN_VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_TYPE]) {
		hdd_err("attr transport switch type failed");
		return -EINVAL;
	}
	transport_switch_type = nla_get_u8(
			tb[QCA_WLAN_VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_TYPE]);

	if (!tb[QCA_WLAN_VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_STATUS]) {
		status = ucfg_ll_lt_sap_request_for_audio_transport_switch(
							transport_switch_type);
		hdd_debug("Transport switch request type %d status %d",
			  transport_switch_type, status);
		return qdf_status_to_os_return(status);
	}

	transport_switch_status = nla_get_u8(
			tb[QCA_WLAN_VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_STATUS]);

	if (transport_switch_status ==
		QCA_WLAN_AUDIO_TRANSPORT_SWITCH_STATUS_COMPLETED) {
		hdd_debug("Transport switch request completed");
		return 0;
	} else if (transport_switch_status ==
		QCA_WLAN_AUDIO_TRANSPORT_SWITCH_STATUS_REJECTED) {
		hdd_debug("Transport switch request rejected");
		return 0;
	}

	hdd_err("Invalid transport switch status");
	return -EINVAL;
}

int wlan_hdd_send_audio_transport_switch_req_event(struct hdd_adapter *adapter,
						   uint8_t audio_switch_mode)
{
	struct sk_buff *vendor_event;
	struct hdd_context *hdd_ctx = cds_get_context(QDF_MODULE_ID_HDD);
	uint32_t len;

	hdd_enter();

	if (wlan_hdd_validate_context(hdd_ctx)) {
		hdd_err("Invalid context");
		return -EINVAL;
	}
	if (hdd_validate_adapter(adapter)) {
		hdd_err("Invalid adapter");
		return -EINVAL;
	}

	len = nla_total_size(sizeof(uint8_t)) + NLMSG_HDRLEN;

	vendor_event = wlan_cfg80211_vendor_event_alloc(
			hdd_ctx->wiphy,
			&adapter->wdev,
			len,
			QCA_NL80211_VENDOR_SUBCMD_AUDIO_TRANSPORT_SWITCH_INDEX,
			GFP_KERNEL);

	if (!vendor_event) {
		hdd_err("wlan_cfg80211_vendor_event_alloc failed");
		return -ENOMEM;
	}

	if (nla_put_u8(vendor_event,
		       QCA_WLAN_VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_TYPE,
		       audio_switch_mode)) {
		hdd_err("VENDOR_ATTR_AUDIO_TRANSPORT_SWITCH_TYPE put fail");
		wlan_cfg80211_vendor_free_skb(vendor_event);
		return -EINVAL;
	}

	wlan_cfg80211_vendor_event(vendor_event, GFP_KERNEL);

	hdd_debug("transport switch request sent on %s interface",
		   netdev_name(adapter->dev));
	return 0;
}

int wlan_hdd_cfg80211_ll_lt_sap_transport_switch(struct wiphy *wiphy,
						 struct wireless_dev *wdev,
						 const void *data,
						 int data_len)
{
	int errno;
	struct osif_vdev_sync *vdev_sync;

	errno = osif_vdev_sync_op_start(wdev->netdev, &vdev_sync);
	if (errno)
		return errno;

	errno = __wlan_hdd_cfg80211_ll_lt_sap_transport_switch(wiphy, wdev,
							       data, data_len);

	osif_vdev_sync_op_stop(vdev_sync);

	return errno;
}
