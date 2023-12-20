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

/*
 * DOC: contains MLO manager Link Switch related functionality
 */
#include <wlan_mlo_mgr_link_switch.h>
#include <wlan_mlo_mgr_main.h>
#include <wlan_mlo_mgr_sta.h>
#include <wlan_serialization_api.h>
#include <wlan_cm_api.h>
#include <wlan_crypto_def_i.h>
#ifdef WLAN_FEATURE_11BE_MLO_ADV_FEATURE
#include "wlan_cm_roam_api.h"
#endif
#include "host_diag_core_event.h"

void mlo_mgr_update_link_info_mac_addr(struct wlan_objmgr_vdev *vdev,
				       struct wlan_mlo_link_mac_update *ml_mac_update)
{
	struct mlo_link_info *link_info;
	uint8_t link_info_iter;
	struct mlo_vdev_link_mac_info *link_mac_info;

	if (!vdev || !vdev->mlo_dev_ctx || !ml_mac_update)
		return;

	link_mac_info = &ml_mac_update->link_mac_info[0];
	link_info = &vdev->mlo_dev_ctx->link_ctx->links_info[0];

	for (link_info_iter = 0; link_info_iter < WLAN_MAX_ML_BSS_LINKS;
	     link_info_iter++) {
		qdf_mem_copy(&link_info->link_addr,
			     &link_mac_info->link_mac_addr,
			     QDF_MAC_ADDR_SIZE);

		link_info->vdev_id = link_mac_info->vdev_id;
		mlo_debug("Update STA Link info for vdev_id %d, link_addr:" QDF_MAC_ADDR_FMT,
			  link_info->vdev_id,
			  QDF_MAC_ADDR_REF(link_info->link_addr.bytes));
		link_mac_info++;
		link_info++;
	}
}

void mlo_mgr_update_ap_link_info(struct wlan_objmgr_vdev *vdev, uint8_t link_id,
				 uint8_t *ap_link_addr,
				 struct wlan_channel channel)
{
	struct mlo_link_info *link_info;
	uint8_t link_info_iter;

	if (!vdev || !vdev->mlo_dev_ctx || !ap_link_addr)
		return;

	link_info = &vdev->mlo_dev_ctx->link_ctx->links_info[0];
	for (link_info_iter = 0; link_info_iter < WLAN_MAX_ML_BSS_LINKS;
	     link_info_iter++) {
		if (qdf_is_macaddr_zero(&link_info->ap_link_addr))
			break;

		link_info++;
	}

	if (link_info_iter == WLAN_MAX_ML_BSS_LINKS)
		return;

	qdf_mem_copy(&link_info->ap_link_addr, ap_link_addr, QDF_MAC_ADDR_SIZE);

	qdf_mem_copy(link_info->link_chan_info, &channel, sizeof(channel));
	link_info->link_status_flags = 0;
	link_info->link_id = link_id;

	mlo_debug("Update AP Link info for link_id: %d, vdev_id:%d, link_addr:" QDF_MAC_ADDR_FMT,
		  link_info->link_id, link_info->vdev_id,
		  QDF_MAC_ADDR_REF(link_info->ap_link_addr.bytes));
}

void mlo_mgr_update_ap_channel_info(struct wlan_objmgr_vdev *vdev, uint8_t link_id,
				    uint8_t *ap_link_addr,
				    struct wlan_channel channel)
{
	struct mlo_link_info *link_info;

	if (!vdev || !vdev->mlo_dev_ctx || !ap_link_addr)
		return;

	link_info = mlo_mgr_get_ap_link_by_link_id(vdev->mlo_dev_ctx, link_id);
	if (!link_info)
		return;

	qdf_mem_copy(link_info->link_chan_info, &channel,
		     sizeof(*link_info->link_chan_info));

	mlo_debug("Update AP Channel info link_id: %d, vdev_id:%d, link_addr:" QDF_MAC_ADDR_FMT,
		  link_info->link_id, link_info->vdev_id,
		  QDF_MAC_ADDR_REF(link_info->ap_link_addr.bytes));
	mlo_debug("Ch_freq: %d, freq1: %d, freq2: %d phy_mode: %d",
		  link_info->link_chan_info->ch_freq,
		  link_info->link_chan_info->ch_cfreq1,
		  link_info->link_chan_info->ch_cfreq2,
		  link_info->link_chan_info->ch_phymode);
}

void mlo_mgr_update_link_info_reset(struct wlan_mlo_dev_context *ml_dev)
{
	struct mlo_link_info *link_info;
	uint8_t link_info_iter;

	if (!ml_dev)
		return;

	link_info = &ml_dev->link_ctx->links_info[0];

	for (link_info_iter = 0; link_info_iter < WLAN_MAX_ML_BSS_LINKS;
	     link_info_iter++) {
		qdf_mem_zero(&link_info->link_addr, QDF_MAC_ADDR_SIZE);
		qdf_mem_zero(&link_info->ap_link_addr, QDF_MAC_ADDR_SIZE);
		qdf_mem_zero(link_info->link_chan_info,
			     sizeof(*link_info->link_chan_info));
		link_info->vdev_id = WLAN_INVALID_VDEV_ID;
		link_info->link_id = WLAN_INVALID_LINK_ID;
		link_info->link_status_flags = 0;
		link_info++;
	}
}

void mlo_mgr_reset_ap_link_info(struct wlan_objmgr_vdev *vdev)
{
	struct mlo_link_info *link_info;
	uint8_t link_info_iter;
	struct wlan_objmgr_psoc *psoc;

	if (!vdev || !vdev->mlo_dev_ctx || !vdev->mlo_dev_ctx->link_ctx)
		return;

	psoc = wlan_vdev_get_psoc(vdev);
	if (!psoc) {
		mlo_err("psoc NULL");
		return;
	}

	link_info = &vdev->mlo_dev_ctx->link_ctx->links_info[0];

	for (link_info_iter = 0; link_info_iter < WLAN_MAX_ML_BSS_LINKS;
	     link_info_iter++) {
		if (!qdf_is_macaddr_zero(&link_info->ap_link_addr) &&
		    !qdf_is_macaddr_zero(&link_info->link_addr))
			wlan_crypto_free_key_by_link_id(
						psoc,
						&link_info->link_addr,
						link_info->link_id);
		qdf_mem_zero(&link_info->ap_link_addr, QDF_MAC_ADDR_SIZE);
		qdf_mem_zero(link_info->link_chan_info,
			     sizeof(*link_info->link_chan_info));
		link_info->link_id = WLAN_INVALID_LINK_ID;
		link_info->link_status_flags = 0;
		link_info++;
	}
}

struct mlo_link_info
*mlo_mgr_get_ap_link_by_link_id(struct wlan_mlo_dev_context *mlo_dev_ctx,
				int link_id)
{
	struct mlo_link_info *link_info;
	uint8_t link_info_iter;

	if (!mlo_dev_ctx || link_id < 0 || link_id > 15)
		return NULL;

	link_info = &mlo_dev_ctx->link_ctx->links_info[0];
	for (link_info_iter = 0; link_info_iter < WLAN_MAX_ML_BSS_LINKS;
	     link_info_iter++) {
		if (link_info->link_id == link_id)
			return link_info;
		link_info++;
	}

	return NULL;
}

struct mlo_link_info
*mlo_mgr_get_ap_link(struct wlan_objmgr_vdev *vdev)
{
	if (!vdev || !vdev->mlo_dev_ctx)
		return NULL;

	return &vdev->mlo_dev_ctx->link_ctx->links_info[0];
}

static
void mlo_mgr_alloc_link_info_wmi_chan(struct wlan_mlo_dev_context *ml_dev)
{
	struct mlo_link_info *link_info;
	uint8_t link_info_iter;

	if (!ml_dev)
		return;

	link_info = &ml_dev->link_ctx->links_info[0];
	for (link_info_iter = 0; link_info_iter < WLAN_MAX_ML_BSS_LINKS;
	     link_info_iter++) {
		link_info->link_chan_info =
			qdf_mem_malloc(sizeof(*link_info->link_chan_info));
		if (!link_info->link_chan_info)
			return;
		link_info++;
	}
}

static
void mlo_mgr_free_link_info_wmi_chan(struct wlan_mlo_dev_context *ml_dev)
{
	struct mlo_link_info *link_info;
	uint8_t link_info_iter;

	if (!ml_dev)
		return;

	link_info = &ml_dev->link_ctx->links_info[0];
	for (link_info_iter = 0; link_info_iter < WLAN_MAX_ML_BSS_LINKS;
	     link_info_iter++) {
		if (link_info->link_chan_info)
			qdf_mem_free(link_info->link_chan_info);
		link_info++;
	}
}

#ifdef WLAN_FEATURE_11BE_MLO_ADV_FEATURE
bool mlo_mgr_update_csa_link_info(struct wlan_mlo_dev_context *mlo_dev_ctx,
				  struct csa_offload_params *csa_param,
				  uint8_t link_id)
{
	struct mlo_link_info *link_info;
	uint16_t bw_val;

	if (!mlo_dev_ctx) {
		mlo_err("invalid mlo dev ctx");
		goto done;
	}

	bw_val = wlan_reg_get_bw_value(csa_param->new_ch_width);

	link_info = mlo_mgr_get_ap_link_by_link_id(mlo_dev_ctx, link_id);
	if (!link_info) {
		mlo_err("invalid link_info");
		goto done;
	}

	link_info->link_chan_info->ch_freq =
				csa_param->csa_chan_freq;
	link_info->link_chan_info->ch_cfreq1 =
				csa_param->new_ch_freq_seg1;
	link_info->link_chan_info->ch_cfreq2 =
				csa_param->new_ch_freq_seg2;

	link_info->link_chan_info->ch_phymode =
			wlan_eht_chan_phy_mode(
				csa_param->csa_chan_freq,
				bw_val, csa_param->new_ch_width);
	return true;
done:
	return false;
}

struct wlan_objmgr_vdev *
mlo_mgr_link_switch_get_assoc_vdev(struct wlan_objmgr_vdev *vdev)
{
	uint8_t vdev_id;
	struct wlan_objmgr_psoc *psoc;
	struct wlan_objmgr_vdev *assoc_vdev;

	if (!vdev)
		return NULL;

	if (!mlo_mgr_is_link_switch_on_assoc_vdev(vdev))
		return NULL;

	vdev_id = vdev->mlo_dev_ctx->link_ctx->last_req.vdev_id;
	psoc = wlan_vdev_get_psoc(vdev);
	if (!psoc) {
		mlo_err("PSOC NULL");
		return NULL;
	}

	assoc_vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
							  WLAN_MLO_MGR_ID);

	return assoc_vdev;
}

bool mlo_mgr_is_link_switch_in_progress(struct wlan_objmgr_vdev *vdev)
{
	enum mlo_link_switch_req_state state;
	struct wlan_mlo_dev_context *mlo_dev_ctx = vdev->mlo_dev_ctx;

	if (!mlo_dev_ctx)
		return false;

	state = mlo_mgr_link_switch_get_curr_state(mlo_dev_ctx);
	return (state > MLO_LINK_SWITCH_STATE_INIT);
}

bool mlo_mgr_is_link_switch_on_assoc_vdev(struct wlan_objmgr_vdev *vdev)
{
	if (!mlo_mgr_is_link_switch_in_progress(vdev))
		return false;

	return vdev->mlo_dev_ctx->link_ctx->last_req.restore_vdev_flag;
}

void mlo_mgr_link_switch_init_state(struct wlan_mlo_dev_context *mlo_dev_ctx)
{
	mlo_dev_lock_acquire(mlo_dev_ctx);
	mlo_dev_ctx->link_ctx->last_req.state = MLO_LINK_SWITCH_STATE_IDLE;
	mlo_dev_lock_release(mlo_dev_ctx);
}

QDF_STATUS
mlo_mgr_link_switch_trans_next_state(struct wlan_mlo_dev_context *mlo_dev_ctx)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	enum mlo_link_switch_req_state cur_state, next_state;

	mlo_dev_lock_acquire(mlo_dev_ctx);
	cur_state = mlo_dev_ctx->link_ctx->last_req.state;
	switch (cur_state) {
	case MLO_LINK_SWITCH_STATE_IDLE:
		next_state = MLO_LINK_SWITCH_STATE_INIT;
		break;
	case MLO_LINK_SWITCH_STATE_INIT:
		next_state = MLO_LINK_SWITCH_STATE_DISCONNECT_CURR_LINK;
		break;
	case MLO_LINK_SWITCH_STATE_DISCONNECT_CURR_LINK:
		next_state = MLO_LINK_SWITCH_STATE_SET_MAC_ADDR;
		break;
	case MLO_LINK_SWITCH_STATE_SET_MAC_ADDR:
		next_state = MLO_LINK_SWITCH_STATE_CONNECT_NEW_LINK;
		break;
	case MLO_LINK_SWITCH_STATE_CONNECT_NEW_LINK:
		next_state = MLO_LINK_SWITCH_STATE_COMPLETE_SUCCESS;
		break;
	case MLO_LINK_SWITCH_STATE_COMPLETE_SUCCESS:
		next_state = MLO_LINK_SWITCH_STATE_IDLE;
		break;
	case MLO_LINK_SWITCH_STATE_ABORT_TRANS:
		next_state = cur_state;
		status = QDF_STATUS_E_PERM;
		mlo_debug("State transition not allowed");
		break;
	default:
		QDF_ASSERT(0);
		break;
	}
	mlo_dev_ctx->link_ctx->last_req.state = next_state;
	mlo_dev_lock_release(mlo_dev_ctx);

	return status;
}

void
mlo_mgr_link_switch_trans_abort_state(struct wlan_mlo_dev_context *mlo_dev_ctx)
{
	enum mlo_link_switch_req_state next_state =
					MLO_LINK_SWITCH_STATE_ABORT_TRANS;

	mlo_dev_lock_acquire(mlo_dev_ctx);
	mlo_dev_ctx->link_ctx->last_req.state = next_state;
	mlo_dev_lock_release(mlo_dev_ctx);
}

enum mlo_link_switch_req_state
mlo_mgr_link_switch_get_curr_state(struct wlan_mlo_dev_context *mlo_dev_ctx)
{
	enum mlo_link_switch_req_state state;

	mlo_dev_lock_acquire(mlo_dev_ctx);
	state = mlo_dev_ctx->link_ctx->last_req.state;
	mlo_dev_lock_release(mlo_dev_ctx);

	return state;
}

#ifdef WLAN_FEATURE_11BE_MLO_ADV_FEATURE
static void
mlo_mgr_reset_roam_state_for_link_vdev(struct wlan_objmgr_vdev *vdev,
				       struct wlan_objmgr_vdev *assoc_vdev)
{
	QDF_STATUS status;

	status = wlan_cm_roam_state_change(wlan_vdev_get_pdev(vdev),
					   wlan_vdev_get_id(assoc_vdev),
					   WLAN_ROAM_DEINIT,
					   REASON_ROAM_LINK_SWITCH_ASSOC_VDEV_CHANGE);
	if (QDF_IS_STATUS_ERROR(status))
		mlo_err("vdev:%d failed to change RSO state to deinit",
			wlan_vdev_get_id(assoc_vdev));
}
#else
static inline void
mlo_mgr_reset_roam_state_for_link_vdev(struct wlan_objmgr_vdev *vdev,
				       struct wlan_objmgr_vdev *assoc_vdev)
{}
#endif

static QDF_STATUS
mlo_mgr_link_switch_osif_notification(struct wlan_objmgr_vdev *vdev,
				      struct wlan_mlo_link_switch_req *lswitch_req)
{
	uint8_t idx;
	uint16_t vdev_count;
	struct wlan_objmgr_vdev *assoc_vdev;
	struct wlan_mlo_sta *sta_ctx;
	struct wlan_objmgr_vdev *vdev_list[WLAN_UMAC_MLO_MAX_VDEVS];
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	struct mlo_mgr_context *g_mlo_ctx = wlan_objmgr_get_mlo_ctx();
	QDF_STATUS(*cb)(struct wlan_objmgr_vdev *vdev,
			uint8_t non_trans_vdev_id);

	if (!vdev->mlo_dev_ctx)
		return status;

	sta_ctx = vdev->mlo_dev_ctx->sta_ctx;
	if (!sta_ctx)
		return status;

	assoc_vdev = wlan_mlo_get_assoc_link_vdev(vdev);
	if (!assoc_vdev)
		return status;

	cb = g_mlo_ctx->osif_ops->mlo_mgr_osif_link_switch_notification;

	if (lswitch_req->restore_vdev_flag) {
		wlan_vdev_mlme_clear_mlo_link_vdev(vdev);
		wlan_vdev_mlme_set_mlo_link_vdev(assoc_vdev);
		mlo_mgr_reset_roam_state_for_link_vdev(vdev, assoc_vdev);

		lswitch_req->restore_vdev_flag = false;

		status = cb(assoc_vdev, wlan_vdev_get_id(vdev));
		if (QDF_IS_STATUS_ERROR(status))
			mlo_debug("OSIF deflink restore failed");

		return status;
	}

	if (wlan_vdev_get_id(assoc_vdev) != lswitch_req->vdev_id) {
		mlo_debug("Not on assoc VDEV no need to swap");
		return QDF_STATUS_SUCCESS;
	}

	mlo_sta_get_vdev_list(vdev, &vdev_count, vdev_list);
	for (idx = 0; idx < vdev_count; idx++) {
		if (wlan_vdev_get_id(vdev_list[idx]) != lswitch_req->vdev_id &&
		    qdf_test_bit(idx, sta_ctx->wlan_connected_links)) {
			wlan_vdev_mlme_clear_mlo_link_vdev(vdev_list[idx]);
			wlan_vdev_mlme_set_mlo_link_vdev(assoc_vdev);
			lswitch_req->restore_vdev_flag = true;

			status = cb(assoc_vdev,
				    wlan_vdev_get_id(vdev_list[idx]));
			break;
		}

		mlo_release_vdev_ref(vdev_list[idx]);
	}

	for (; idx < vdev_count; idx++)
		mlo_release_vdev_ref(vdev_list[idx]);

	return status;
}

QDF_STATUS
mlo_mgr_link_switch_notification(struct wlan_objmgr_vdev *vdev,
				 struct wlan_mlo_link_switch_req *lswitch_req,
				 enum wlan_mlo_link_switch_notify_reason notify_reason)
{
	QDF_STATUS status;

	if (notify_reason == MLO_LINK_SWITCH_NOTIFY_REASON_PRE_START_PRE_SER)
		return QDF_STATUS_SUCCESS;

	status = mlo_mgr_link_switch_osif_notification(vdev, lswitch_req);
	return status;
}

QDF_STATUS mlo_mgr_link_switch_init(struct wlan_mlo_dev_context *ml_dev)
{
	ml_dev->link_ctx =
		qdf_mem_malloc(sizeof(struct mlo_link_switch_context));

	if (!ml_dev->link_ctx)
		return QDF_STATUS_E_NOMEM;

	mlo_mgr_link_switch_init_state(ml_dev);
	mlo_mgr_alloc_link_info_wmi_chan(ml_dev);
	mlo_mgr_update_link_info_reset(ml_dev);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS mlo_mgr_link_switch_deinit(struct wlan_mlo_dev_context *ml_dev)
{
	mlo_mgr_free_link_info_wmi_chan(ml_dev);
	qdf_mem_free(ml_dev->link_ctx);
	ml_dev->link_ctx = NULL;
	return QDF_STATUS_SUCCESS;
}

void
mlo_mgr_osif_update_connect_info(struct wlan_objmgr_vdev *vdev, int32_t link_id)
{
	struct mlo_mgr_context *g_mlo_ctx = wlan_objmgr_get_mlo_ctx();
	struct mlo_link_info *link_info;
	QDF_STATUS(*osif_bss_update_cb)(struct qdf_mac_addr *self_mac,
					struct qdf_mac_addr *bssid,
					int32_t link_id);

	if (!g_mlo_ctx || !vdev->mlo_dev_ctx || !g_mlo_ctx->osif_ops ||
	    !g_mlo_ctx->osif_ops->mlo_mgr_osif_update_bss_info)
		return;

	link_info = mlo_mgr_get_ap_link_by_link_id(vdev->mlo_dev_ctx, link_id);
	if (!link_info)
		return;

	mlo_debug("VDEV ID %d, Link ID %d, STA MAC " QDF_MAC_ADDR_FMT ", BSSID " QDF_MAC_ADDR_FMT,
		  link_info->vdev_id, link_id,
		  QDF_MAC_ADDR_REF(link_info->link_addr.bytes),
		  QDF_MAC_ADDR_REF(link_info->ap_link_addr.bytes));
	osif_bss_update_cb = g_mlo_ctx->osif_ops->mlo_mgr_osif_update_bss_info;

	osif_bss_update_cb(&link_info->link_addr, &link_info->ap_link_addr,
			   link_id);
}

QDF_STATUS mlo_mgr_link_switch_disconnect_done(struct wlan_objmgr_vdev *vdev,
					       QDF_STATUS discon_status,
					       bool is_link_switch_resp)
{
	QDF_STATUS status;
	enum mlo_link_switch_req_state cur_state;
	struct mlo_link_info *new_link_info;
	struct qdf_mac_addr mac_addr, mld_addr;
	struct wlan_mlo_dev_context *mlo_dev_ctx = vdev->mlo_dev_ctx;
	struct wlan_mlo_link_switch_req *req = &mlo_dev_ctx->link_ctx->last_req;

	if (!is_link_switch_resp) {
		mlo_mgr_link_switch_trans_abort_state(mlo_dev_ctx);
		return QDF_STATUS_SUCCESS;
	}

	cur_state = mlo_mgr_link_switch_get_curr_state(mlo_dev_ctx);
	if (QDF_IS_STATUS_ERROR(discon_status) ||
	    cur_state != MLO_LINK_SWITCH_STATE_DISCONNECT_CURR_LINK) {
		mlo_err("VDEV %d link switch disconnect req failed",
			req->vdev_id);
		mlo_mgr_remove_link_switch_cmd(vdev);
		return QDF_STATUS_SUCCESS;
	}

	mlo_debug("VDEV %d link switch disconnect complete",
		  wlan_vdev_get_id(vdev));

	new_link_info =
		mlo_mgr_get_ap_link_by_link_id(vdev->mlo_dev_ctx,
					       req->new_ieee_link_id);
	if (!new_link_info) {
		mlo_err("New link not found in mlo dev ctx");
		mlo_mgr_remove_link_switch_cmd(vdev);
		return QDF_STATUS_E_INVAL;
	}

	qdf_copy_macaddr(&mld_addr, &mlo_dev_ctx->mld_addr);
	qdf_copy_macaddr(&mac_addr, &new_link_info->link_addr);

	status = mlo_mgr_link_switch_trans_next_state(mlo_dev_ctx);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlo_mgr_remove_link_switch_cmd(vdev);
		return status;
	}

	status = wlan_vdev_mlme_send_set_mac_addr(mac_addr, mld_addr, vdev);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlo_debug("VDEV %d set MAC addr FW request failed",
			  req->vdev_id);
		mlo_mgr_remove_link_switch_cmd(vdev);
	}

	return status;
}

QDF_STATUS mlo_mgr_link_switch_set_mac_addr_resp(struct wlan_objmgr_vdev *vdev,
						 uint8_t resp_status)
{
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	enum mlo_link_switch_req_state cur_state;
	struct mlo_mgr_context *g_mlo_ctx = wlan_objmgr_get_mlo_ctx();
	struct wlan_mlo_link_switch_req *req;
	struct mlo_link_info *new_link_info;

	if (resp_status) {
		mlo_err("VDEV %d set MAC address response %d",
			wlan_vdev_get_id(vdev), resp_status);
		mlo_mgr_remove_link_switch_cmd(vdev);
		return status;
	}

	if (!g_mlo_ctx) {
		mlo_err("global mlo ctx NULL");
		mlo_mgr_remove_link_switch_cmd(vdev);
		return status;
	}

	req = &vdev->mlo_dev_ctx->link_ctx->last_req;
	cur_state = mlo_mgr_link_switch_get_curr_state(vdev->mlo_dev_ctx);
	if (cur_state != MLO_LINK_SWITCH_STATE_SET_MAC_ADDR) {
		mlo_err("Link switch cmd flushed, there can be MAC addr mismatch with FW");
		mlo_mgr_remove_link_switch_cmd(vdev);
		return status;
	}

	new_link_info =
		mlo_mgr_get_ap_link_by_link_id(vdev->mlo_dev_ctx,
					       req->new_ieee_link_id);
	if (!new_link_info) {
		mlo_mgr_remove_link_switch_cmd(vdev);
		return status;
	}

	wlan_vdev_mlme_set_macaddr(vdev, new_link_info->link_addr.bytes);
	wlan_vdev_mlme_set_linkaddr(vdev, new_link_info->link_addr.bytes);

	status = g_mlo_ctx->osif_ops->mlo_mgr_osif_update_mac_addr(
							req->curr_ieee_link_id,
							req->new_ieee_link_id,
							req->vdev_id);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlo_debug("VDEV %d OSIF MAC addr update failed %d",
			  req->vdev_id, status);
		mlo_mgr_remove_link_switch_cmd(vdev);
		return status;
	}

	status = mlo_mgr_link_switch_trans_next_state(vdev->mlo_dev_ctx);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlo_mgr_remove_link_switch_cmd(vdev);
		return status;
	}

	status = mlo_mgr_link_switch_start_connect(vdev);

	return status;
}

QDF_STATUS mlo_mgr_link_switch_start_connect(struct wlan_objmgr_vdev *vdev)
{
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	struct wlan_cm_connect_req conn_req = {0};
	struct mlo_link_info *mlo_link_info;
	uint8_t *vdev_mac;
	struct wlan_mlo_sta *sta_ctx;
	struct wlan_mlo_link_switch_req *req =
					&vdev->mlo_dev_ctx->link_ctx->last_req;

	sta_ctx = vdev->mlo_dev_ctx->sta_ctx;

	mlo_link_info =
		mlo_mgr_get_ap_link_by_link_id(vdev->mlo_dev_ctx,
					       req->new_ieee_link_id);

	if (!mlo_link_info) {
		mlo_err("New link ID not found");
		goto out;
	}

	vdev_mac = wlan_vdev_mlme_get_linkaddr(vdev);
	if (!qdf_is_macaddr_equal(&mlo_link_info->link_addr,
				  (struct qdf_mac_addr *)vdev_mac)) {
		mlo_err("MAC address not equal for the new Link ID VDEV: " QDF_MAC_ADDR_FMT ", MLO_LINK: " QDF_MAC_ADDR_FMT,
			QDF_MAC_ADDR_REF(vdev_mac),
			QDF_MAC_ADDR_REF(mlo_link_info->link_addr.bytes));
		goto out;
	}

	wlan_vdev_set_link_id(vdev, req->new_ieee_link_id);
	copied_conn_req_lock_acquire(sta_ctx);
	if (sta_ctx->copied_conn_req) {
		qdf_mem_copy(&conn_req, sta_ctx->copied_conn_req,
			     sizeof(struct wlan_cm_connect_req));
	} else {
		copied_conn_req_lock_release(sta_ctx);
		goto out;
	}
	copied_conn_req_lock_release(sta_ctx);

	conn_req.vdev_id = wlan_vdev_get_id(vdev);
	conn_req.source = CM_MLO_LINK_SWITCH_CONNECT;
	qdf_copy_macaddr(&conn_req.bssid, &mlo_link_info->ap_link_addr);
	mlo_allocate_and_copy_ies(&conn_req, sta_ctx->copied_conn_req);
	conn_req.crypto.auth_type = 0;
	conn_req.ml_parnter_info = sta_ctx->ml_partner_info;
	status = wlan_cm_start_connect(vdev, &conn_req);
	if (QDF_IS_STATUS_SUCCESS(status))
		mlo_update_connected_links(vdev, 1);

	wlan_cm_free_connect_req_param(&conn_req);

out:
	if (QDF_IS_STATUS_ERROR(status)) {
		mlo_err("VDEV %d link switch connect request failed",
			wlan_vdev_get_id(vdev));
		mlo_mgr_remove_link_switch_cmd(vdev);
	}

	return status;
}

void mlo_mgr_link_switch_connect_done(struct wlan_objmgr_vdev *vdev,
				      QDF_STATUS status)
{
	struct wlan_mlo_link_switch_req *req;

	req = &vdev->mlo_dev_ctx->link_ctx->last_req;
	if (QDF_IS_STATUS_SUCCESS(status))
		mlo_mgr_link_switch_trans_next_state(vdev->mlo_dev_ctx);
	else
		mlo_err("VDEV %d link switch connect failed", req->vdev_id);

	mlo_mgr_remove_link_switch_cmd(vdev);
}

static enum wlan_mlo_link_switch_notify_reason
mlo_mgr_link_switch_get_notify_reason(struct wlan_objmgr_vdev *vdev)
{
	enum mlo_link_switch_req_state curr_state;
	enum wlan_mlo_link_switch_notify_reason notify_reason;

	curr_state = mlo_mgr_link_switch_get_curr_state(vdev->mlo_dev_ctx);
	switch (curr_state) {
	case MLO_LINK_SWITCH_STATE_IDLE:
		notify_reason = MLO_LINK_SWITCH_NOTIFY_REASON_PRE_START_PRE_SER;
		break;
	case MLO_LINK_SWITCH_STATE_INIT:
		notify_reason =
			MLO_LINK_SWITCH_NOTIFY_REASON_PRE_START_POST_SER;
		break;
	case MLO_LINK_SWITCH_STATE_COMPLETE_SUCCESS:
		notify_reason = MLO_LINK_SWITCH_NOTIFY_REASON_STOP_SUCCESS;
		break;
	default:
		notify_reason = MLO_LINK_SWITCH_NOTIFY_REASON_STOP_FAILURE;
		break;
	}

	return notify_reason;
}

static QDF_STATUS
mlo_mgr_start_link_switch(struct wlan_objmgr_vdev *vdev,
			  struct wlan_serialization_command *cmd)
{
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	uint8_t vdev_id, old_link_id, new_link_id;
	struct wlan_mlo_dev_context *mlo_dev_ctx = vdev->mlo_dev_ctx;
	struct wlan_mlo_link_switch_req *req = &mlo_dev_ctx->link_ctx->last_req;
	struct qdf_mac_addr bssid;

	vdev_id = wlan_vdev_get_id(vdev);
	old_link_id = req->curr_ieee_link_id;
	new_link_id = req->new_ieee_link_id;

	mlo_debug("VDEV %d start link switch", vdev_id);
	mlo_mgr_link_switch_trans_next_state(mlo_dev_ctx);

	if (!wlan_cm_is_vdev_connected(vdev)) {
		mlo_err("VDEV %d not in connected state", vdev_id);
		return status;
	}

	status = wlan_vdev_get_bss_peer_mac(vdev, &bssid);
	if (QDF_IS_STATUS_ERROR(status))
		return status;

	status = wlan_vdev_get_bss_peer_mld_mac(vdev, &req->peer_mld_addr);
	if (QDF_IS_STATUS_ERROR(status))
		return status;

	status = mlo_mgr_link_switch_notify(vdev, req);
	if (QDF_IS_STATUS_ERROR(status))
		return status;

	wlan_vdev_mlme_set_mlo_link_switch_in_progress(vdev);
	status = mlo_mgr_link_switch_trans_next_state(mlo_dev_ctx);
	if (QDF_IS_STATUS_ERROR(status))
		return status;

	status = wlan_cm_disconnect(vdev, CM_MLO_LINK_SWITCH_DISCONNECT,
				    REASON_FW_TRIGGERED_LINK_SWITCH, &bssid);

	if (QDF_IS_STATUS_ERROR(status))
		mlo_err("VDEV %d disconnect request not handled", req->vdev_id);

	return status;
}

static QDF_STATUS
mlo_mgr_ser_link_switch_cb(struct wlan_serialization_command *cmd,
			   enum wlan_serialization_cb_reason reason)
{
	struct wlan_objmgr_vdev *vdev;
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	struct wlan_mlo_link_switch_req *req;

	if (!cmd) {
		mlo_err("cmd is NULL, reason: %d", reason);
		QDF_ASSERT(0);
		return QDF_STATUS_E_NULL_VALUE;
	}

	vdev = cmd->vdev;
	req = &vdev->mlo_dev_ctx->link_ctx->last_req;

	switch (reason) {
	case WLAN_SER_CB_ACTIVATE_CMD:
		status = mlo_mgr_start_link_switch(vdev, cmd);
		if (QDF_IS_STATUS_ERROR(status)) {
			mlo_mgr_link_switch_trans_abort_state(vdev->mlo_dev_ctx);
			mlo_mgr_link_switch_notify(vdev, req);
		}
		break;
	case WLAN_SER_CB_RELEASE_MEM_CMD:
		mlo_mgr_link_switch_complete(vdev);
		break;
	case WLAN_SER_CB_CANCEL_CMD:
		mlo_err("Link switch cmd cancelled");
		break;
	case WLAN_SER_CB_ACTIVE_CMD_TIMEOUT:
		mlo_err("Link switch active cmd timeout");
		break;
	default:
		QDF_ASSERT(0);
		mlo_mgr_link_switch_complete(vdev);
		break;
	}

	return status;
}

void mlo_mgr_remove_link_switch_cmd(struct wlan_objmgr_vdev *vdev)
{
	struct wlan_serialization_queued_cmd_info cmd_info;
	enum mlo_link_switch_req_state cur_state;
	uint8_t vdev_id = wlan_vdev_get_id(vdev);
	struct wlan_mlo_link_switch_req *req;

	cur_state = mlo_mgr_link_switch_get_curr_state(vdev->mlo_dev_ctx);
	if (cur_state == MLO_LINK_SWITCH_STATE_IDLE)
		return;

	req = &vdev->mlo_dev_ctx->link_ctx->last_req;
	mlo_mgr_link_switch_notify(vdev, req);

	/* Handle any pending disconnect */
	if (cur_state == MLO_LINK_SWITCH_STATE_ABORT_TRANS)
		mlo_handle_pending_disconnect(vdev);

	if (req->reason == MLO_LINK_SWITCH_REASON_HOST_FORCE) {
		mlo_debug("Link switch not serialized");
		mlo_mgr_link_switch_complete(vdev);
		return;
	}

	cmd_info.cmd_id = (vdev_id << 16) + (req->new_ieee_link_id << 8) +
			  (req->curr_ieee_link_id);
	cmd_info.req_type = WLAN_SER_CANCEL_NON_SCAN_CMD;
	cmd_info.cmd_type = WLAN_SER_CMD_MLO_VDEV_LINK_SWITCH;
	cmd_info.vdev = vdev;
	cmd_info.queue_type = WLAN_SERIALIZATION_ACTIVE_QUEUE;

	wlan_serialization_remove_cmd(&cmd_info);
}

#define MLO_MGR_MAX_LSWITCH_TIMEOUT	35000

QDF_STATUS mlo_mgr_ser_link_switch_cmd(struct wlan_objmgr_vdev *vdev,
				       struct wlan_mlo_link_switch_req *req)
{
	QDF_STATUS status;
	enum wlan_serialization_status ser_cmd_status;
	struct wlan_serialization_command cmd = {0};
	uint8_t vdev_id = wlan_vdev_get_id(vdev);
	struct mlo_link_switch_context *link_ctx;

	if (!vdev->mlo_dev_ctx) {
		mlo_err("ML dev ctx NULL, reject link switch");
		return QDF_STATUS_E_INVAL;
	}

	link_ctx = vdev->mlo_dev_ctx->link_ctx;
	link_ctx->last_req = *req;

	cmd.cmd_type = WLAN_SER_CMD_MLO_VDEV_LINK_SWITCH;
	cmd.cmd_id = (vdev_id << 16) + (req->new_ieee_link_id << 8) +
		     (req->curr_ieee_link_id);
	cmd.cmd_cb = mlo_mgr_ser_link_switch_cb;
	cmd.source = WLAN_UMAC_COMP_MLO_MGR;
	cmd.is_high_priority = false;
	cmd.cmd_timeout_duration = MLO_MGR_MAX_LSWITCH_TIMEOUT;
	cmd.vdev = vdev;
	cmd.is_blocking = true;

	if (req->reason == MLO_LINK_SWITCH_REASON_HOST_FORCE) {
		mlo_debug("Do not serialize link switch");
		status = mlo_mgr_start_link_switch(vdev, &cmd);
		if (QDF_IS_STATUS_ERROR(status)) {
			mlo_mgr_link_switch_trans_abort_state(vdev->mlo_dev_ctx);
			mlo_mgr_link_switch_notify(vdev, req);
		}
		return status;
	}

	ser_cmd_status = wlan_serialization_request(&cmd);
	switch (ser_cmd_status) {
	case WLAN_SER_CMD_PENDING:
		mlo_debug("Link switch cmd in pending queue");
		break;
	case WLAN_SER_CMD_ACTIVE:
		mlo_debug("Link switch cmd in active queue");
		break;
	default:
		return QDF_STATUS_E_INVAL;
	}

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS mlo_mgr_link_switch_notify(struct wlan_objmgr_vdev *vdev,
				      struct wlan_mlo_link_switch_req *req)
{
	int8_t i;
	QDF_STATUS status, ret_status = QDF_STATUS_SUCCESS;
	enum wlan_mlo_link_switch_notify_reason notify_reason;
	struct mlo_mgr_context *mlo_mgr_ctx = wlan_objmgr_get_mlo_ctx();

	if (!mlo_mgr_ctx) {
		mlo_err("Global mlo mgr NULL");
		return QDF_STATUS_E_NULL_VALUE;
	}

	notify_reason = mlo_mgr_link_switch_get_notify_reason(vdev);
	for (i = 0; i < WLAN_UMAC_COMP_ID_MAX; i++) {
		if (!mlo_mgr_ctx->lswitch_notifier[i].in_use)
			continue;

		status = mlo_mgr_ctx->lswitch_notifier[i].cb(vdev, req,
							     notify_reason);
		if (QDF_IS_STATUS_SUCCESS(status))
			continue;

		mlo_debug("Link switch notify %d failed in %d",
			  notify_reason, i);
		ret_status = status;
		if (notify_reason == MLO_LINK_SWITCH_NOTIFY_REASON_PRE_START_PRE_SER)
			break;
	}

	return ret_status;
}

QDF_STATUS
mlo_mgr_link_switch_validate_request(struct wlan_objmgr_vdev *vdev,
				     struct wlan_mlo_link_switch_req *req)
{
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	uint8_t vdev_id = wlan_vdev_get_id(vdev);

	if (req->curr_ieee_link_id >= WLAN_INVALID_LINK_ID ||
	    req->new_ieee_link_id >= WLAN_INVALID_LINK_ID) {
		mlo_err("Invalid link params, curr link id %d, new link id %d",
			req->curr_ieee_link_id, req->new_ieee_link_id);
		return status;
	}

	if (!mlo_mgr_get_ap_link_by_link_id(vdev->mlo_dev_ctx,
					    req->new_ieee_link_id)) {
		mlo_err("New link id %d not part of association",
			req->new_ieee_link_id);
		return status;
	}

	if (!mlo_is_mld_sta(vdev)) {
		mlo_err("Link switch req not valid for VDEV %d", vdev_id);
		return status;
	}

	if (!wlan_cm_is_vdev_connected(vdev)) {
		mlo_err("VDEV %d not in connected state", vdev_id);
		return status;
	}

	if (mlo_mgr_is_link_switch_in_progress(vdev)) {
		mlo_err("Link switch already in progress");
		return status;
	}

	if (wlan_vdev_get_link_id(vdev) != req->curr_ieee_link_id) {
		mlo_err("VDEV %d link id wrong, curr link id %d",
			vdev_id, wlan_vdev_get_link_id(vdev));
		return status;
	}

	/* Notify callers on the new link switch request before serializing */
	status = mlo_mgr_link_switch_notify(vdev, req);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlo_err("Link switch rejected in pre-serialize notify");
		return status;
	}

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS mlo_mgr_link_switch_request_params(struct wlan_objmgr_psoc *psoc,
					      void *evt_params)
{
	QDF_STATUS status;
	struct wlan_mlo_link_switch_cnf cnf_params = {0};
	struct wlan_mlo_link_switch_req *req;
	struct wlan_objmgr_vdev *vdev;

	if (!evt_params) {
		mlo_err("Invalid params");
		return QDF_STATUS_E_INVAL;
	}

	req = (struct wlan_mlo_link_switch_req *)evt_params;

	/* The reference is released on Link Switch status confirm to FW */
	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, req->vdev_id,
						    WLAN_MLO_MGR_ID);
	if (!vdev) {
		mlo_err("Invalid link switch VDEV %d", req->vdev_id);

		/* Fill reject params here and send to FW as VDEV is invalid */
		cnf_params.vdev_id = req->vdev_id;
		cnf_params.status = MLO_LINK_SWITCH_CNF_STATUS_REJECT;
		mlo_mgr_link_switch_send_cnf_cmd(psoc, &cnf_params);
		return QDF_STATUS_E_INVAL;
	}

	mlo_debug("VDEV %d, curr_link_id %d, new_link_id %d, new_freq %d, new_phymode: %d, reason %d",
		  req->vdev_id, req->curr_ieee_link_id, req->new_ieee_link_id,
		  req->new_primary_freq, req->new_phymode, req->reason);

	status = mlo_mgr_link_switch_validate_request(vdev, req);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlo_debug("Link switch params/request invalid");
		mlo_mgr_link_switch_complete(vdev);
		return QDF_STATUS_E_INVAL;
	}

	status = mlo_mgr_ser_link_switch_cmd(vdev, req);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlo_err("Failed to serialize link switch command");
		mlo_mgr_link_switch_complete(vdev);
	}

	return status;
}

QDF_STATUS
mlo_mgr_link_state_switch_info_handler(struct wlan_objmgr_psoc *psoc,
				       struct mlo_link_switch_state_info *info)
{
	uint8_t i;

	for (i = 0; i < info->num_params; i++)
		wlan_connectivity_mld_link_status_event(psoc,
							&info->link_switch_param[i]);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS mlo_mgr_link_switch_complete(struct wlan_objmgr_vdev *vdev)
{
	enum mlo_link_switch_req_state state;
	struct wlan_mlo_link_switch_cnf params = {0};
	struct mlo_link_switch_context *link_ctx;
	struct wlan_mlo_link_switch_req *req;
	struct wlan_objmgr_psoc *psoc;

	/* Not checking NULL value as reference is already taken for vdev */
	psoc = wlan_vdev_get_psoc(vdev);

	link_ctx = vdev->mlo_dev_ctx->link_ctx;
	req = &link_ctx->last_req;

	state = mlo_mgr_link_switch_get_curr_state(vdev->mlo_dev_ctx);
	if (state != MLO_LINK_SWITCH_STATE_COMPLETE_SUCCESS)
		params.status = MLO_LINK_SWITCH_CNF_STATUS_REJECT;
	else
		params.status = MLO_LINK_SWITCH_CNF_STATUS_ACCEPT;

	params.vdev_id = wlan_vdev_get_id(vdev);
	params.reason = MLO_LINK_SWITCH_CNF_REASON_BSS_PARAMS_CHANGED;

	mlo_mgr_link_switch_send_cnf_cmd(psoc, &params);

	mlo_mgr_link_switch_init_state(vdev->mlo_dev_ctx);
	wlan_vdev_mlme_clear_mlo_link_switch_in_progress(vdev);
	wlan_objmgr_vdev_release_ref(vdev, WLAN_MLO_MGR_ID);
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
mlo_mgr_link_switch_send_cnf_cmd(struct wlan_objmgr_psoc *psoc,
				 struct wlan_mlo_link_switch_cnf *cnf_params)
{
	QDF_STATUS status;
	struct wlan_lmac_if_mlo_tx_ops *mlo_tx_ops;

	mlo_debug("VDEV %d link switch completed, %s", cnf_params->vdev_id,
		  (cnf_params->status == MLO_LINK_SWITCH_CNF_STATUS_ACCEPT) ?
		  "success" : "fail");

	mlo_tx_ops = &psoc->soc_cb.tx_ops->mlo_ops;
	if (!mlo_tx_ops || !mlo_tx_ops->send_mlo_link_switch_cnf_cmd) {
		mlo_err("handler is not registered");
		return QDF_STATUS_E_INVAL;
	}

	status = mlo_tx_ops->send_mlo_link_switch_cnf_cmd(psoc, cnf_params);
	if (QDF_IS_STATUS_ERROR(status))
		mlo_err("Link switch status update to FW failed");

	return status;
}

QDF_STATUS
mlo_mgr_link_switch_defer_disconnect_req(struct wlan_objmgr_vdev *vdev,
					 enum wlan_cm_source source,
					 enum wlan_reason_code reason)
{
	struct wlan_mlo_dev_context *mlo_dev_ctx;
	struct wlan_mlo_sta *sta_ctx;

	if (!mlo_mgr_is_link_switch_in_progress(vdev)) {
		mlo_info("Link switch not in progress");
		return QDF_STATUS_E_INVAL;
	}

	mlo_dev_ctx = vdev->mlo_dev_ctx;
	sta_ctx = mlo_dev_ctx->sta_ctx;

	if (!sta_ctx) {
		mlo_err("sta ctx null");
		return QDF_STATUS_E_NULL_VALUE;
	}

	/* Move current link switch to abort state */
	mlo_mgr_link_switch_trans_abort_state(mlo_dev_ctx);

	if (sta_ctx->disconn_req) {
		mlo_debug("Pending disconnect from source %d, reason %d",
			  sta_ctx->disconn_req->source,
			  sta_ctx->disconn_req->reason_code);
		return QDF_STATUS_E_ALREADY;
	}

	sta_ctx->disconn_req =
			qdf_mem_malloc(sizeof(struct wlan_cm_disconnect_req));
	if (!sta_ctx->disconn_req)
		return QDF_STATUS_E_NOMEM;

	sta_ctx->disconn_req->vdev_id = wlan_vdev_get_id(vdev);
	sta_ctx->disconn_req->source = source;
	sta_ctx->disconn_req->reason_code = reason;

	mlo_debug("Deferred disconnect source: %d, reason: %d", source, reason);
	return QDF_STATUS_SUCCESS;
}
#endif
