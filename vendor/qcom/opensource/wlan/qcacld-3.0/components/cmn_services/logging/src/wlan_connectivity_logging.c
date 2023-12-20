/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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
 * DOC: wlan_cm_roam_logging.c
 *
 * Implementation for the connectivity and roam logging api.
 */
#include "wlan_connectivity_logging.h"
#include "wlan_cm_api.h"
#include "wlan_mlme_main.h"
#include "wlan_mlo_mgr_sta.h"

#ifdef WLAN_FEATURE_CONNECTIVITY_LOGGING
static struct wlan_connectivity_log_buf_data global_cl;

static void
wlan_connectivity_logging_register_callbacks(
				struct wlan_cl_osif_cbks *osif_cbks,
				void *osif_cb_context)
{
	global_cl.osif_cbks.wlan_connectivity_log_send_to_usr =
			osif_cbks->wlan_connectivity_log_send_to_usr;
	global_cl.osif_cb_context = osif_cb_context;
}

void wlan_connectivity_logging_start(struct wlan_objmgr_psoc *psoc,
				     struct wlan_cl_osif_cbks *osif_cbks,
				     void *osif_cb_context)
{
	global_cl.head = qdf_mem_valloc(sizeof(*global_cl.head) *
					WLAN_MAX_LOG_RECORDS);
	if (!global_cl.head) {
		QDF_BUG(0);
		return;
	}

	global_cl.psoc = psoc;
	global_cl.write_idx = 0;
	global_cl.read_idx = 0;

	qdf_atomic_init(&global_cl.dropped_msgs);
	qdf_spinlock_create(&global_cl.write_ptr_lock);

	global_cl.read_ptr = global_cl.head;
	global_cl.write_ptr = global_cl.head;
	global_cl.max_records = WLAN_MAX_LOG_RECORDS;

	wlan_connectivity_logging_register_callbacks(osif_cbks,
						     osif_cb_context);
	qdf_atomic_set(&global_cl.is_active, 1);
}

void wlan_connectivity_logging_stop(void)
{
	if (!qdf_atomic_read(&global_cl.is_active))
		return;

	qdf_spin_lock_bh(&global_cl.write_ptr_lock);

	global_cl.psoc = NULL;
	global_cl.osif_cb_context = NULL;
	global_cl.osif_cbks.wlan_connectivity_log_send_to_usr = NULL;

	qdf_atomic_set(&global_cl.is_active, 0);
	global_cl.read_ptr = NULL;
	global_cl.write_ptr = NULL;
	global_cl.read_idx = 0;
	global_cl.write_idx = 0;

	qdf_mem_vfree(global_cl.head);
	global_cl.head = NULL;
	qdf_spin_unlock_bh(&global_cl.write_ptr_lock);
	qdf_spinlock_destroy(&global_cl.write_ptr_lock);
}
#endif

#ifdef WLAN_FEATURE_ROAM_OFFLOAD
void wlan_clear_sae_auth_logs_cache(struct wlan_objmgr_psoc *psoc,
				    uint8_t vdev_id)
{
	struct wlan_objmgr_vdev *vdev;
	struct mlme_legacy_priv *mlme_priv;

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
						    WLAN_MLME_OBJMGR_ID);
	if (!vdev) {
		logging_err_rl("Invalid vdev:%d", vdev_id);
		return;
	}

	mlme_priv = wlan_vdev_mlme_get_ext_hdl(vdev);
	if (!mlme_priv) {
		wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);
		logging_err_rl("vdev legacy private object is NULL");
		return;
	}

	qdf_mem_zero(mlme_priv->auth_log, sizeof(mlme_priv->auth_log));
	wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);
}
#endif

#if defined(WLAN_FEATURE_ROAM_OFFLOAD) && defined(CONNECTIVITY_DIAG_EVENT)
QDF_STATUS wlan_print_cached_sae_auth_logs(struct wlan_objmgr_psoc *psoc,
					   struct qdf_mac_addr *bssid,
					   uint8_t vdev_id)
{
	uint8_t i, j;
	struct wlan_objmgr_vdev *vdev;
	struct mlme_legacy_priv *mlme_priv;

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
						    WLAN_MLME_OBJMGR_ID);
	if (!vdev) {
		logging_err_rl("Invalid vdev:%d", vdev_id);
		return QDF_STATUS_E_FAILURE;
	}

	if (wlan_vdev_mlme_get_opmode(vdev) != QDF_STA_MODE) {
		wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);
		return QDF_STATUS_E_FAILURE;
	}

	mlme_priv = wlan_vdev_mlme_get_ext_hdl(vdev);
	if (!mlme_priv) {
		logging_err_rl("vdev legacy private object is NULL");
		wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);
		return QDF_STATUS_E_FAILURE;
	}

	/*
	 * Get the index of matching bssid and queue all the records for
	 * that bssid
	 */
	for (i = 0; i < MAX_ROAM_CANDIDATE_AP; i++) {
		if (!mlme_priv->auth_log[i][0].diag_cmn.ktime_us)
			continue;

		if (qdf_is_macaddr_equal(bssid,
					 (struct qdf_mac_addr *)mlme_priv->auth_log[i][0].diag_cmn.bssid))
			break;
	}

	/*
	 * No matching bssid found in cached log records.
	 * So return from here.
	 */
	if (i >= MAX_ROAM_CANDIDATE_AP) {
		logging_debug("No cached SAE auth logs");
		wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);
		return QDF_STATUS_E_FAILURE;
	}

	for (j = 0; j < WLAN_ROAM_MAX_CACHED_AUTH_FRAMES; j++) {
		if (!mlme_priv->auth_log[i][j].diag_cmn.ktime_us)
			continue;

		WLAN_HOST_DIAG_EVENT_REPORT(&mlme_priv->auth_log[i][j],
					    EVENT_WLAN_MGMT);
		qdf_mem_zero(&mlme_priv->auth_log[i][j],
			     sizeof(struct wlan_diag_packet_info));
	}

	wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);

	return QDF_STATUS_SUCCESS;
}

bool wlan_is_log_record_present_for_bssid(struct wlan_objmgr_psoc *psoc,
					  struct qdf_mac_addr *bssid,
					  uint8_t vdev_id)
{
	struct wlan_diag_packet_info *pkt_info;
	struct wlan_objmgr_vdev *vdev;
	struct mlme_legacy_priv *mlme_priv;
	int i;

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
						    WLAN_MLME_OBJMGR_ID);
	if (!vdev) {
		logging_err_rl("Invalid vdev:%d", vdev_id);
		return false;
	}

	mlme_priv = wlan_vdev_mlme_get_ext_hdl(vdev);
	if (!mlme_priv) {
		wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);
		logging_err_rl("vdev legacy private object is NULL");
		return false;
	}

	for (i = 0; i < MAX_ROAM_CANDIDATE_AP; i++) {
		pkt_info = &mlme_priv->auth_log[i][0];
		if (!pkt_info->diag_cmn.ktime_us)
			continue;

		if (qdf_is_macaddr_equal(bssid,
					 (struct qdf_mac_addr *)pkt_info->diag_cmn.bssid)) {
			wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);
			return true;
		}
	}
	wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);

	return false;
}

/**
 * wlan_add_sae_log_record_to_available_slot() - Add a new log record into the
 * cache for the queue.
 * @psoc: objmgr psoc object
 * @vdev: objmgr vdev object
 * @pkt_info: Log packet record pointer
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS
wlan_add_sae_log_record_to_available_slot(struct wlan_objmgr_psoc *psoc,
					  struct wlan_objmgr_vdev *vdev,
					  struct wlan_diag_packet_info *pkt_info)
{
	struct mlme_legacy_priv *mlme_priv;
	uint8_t i, j;
	uint8_t vdev_id = wlan_vdev_get_id(vdev);
	bool is_entry_exist =
		wlan_is_log_record_present_for_bssid(psoc,
						     (struct qdf_mac_addr *)pkt_info->diag_cmn.bssid,
						     vdev_id);

	mlme_priv = wlan_vdev_mlme_get_ext_hdl(vdev);
	if (!mlme_priv) {
		logging_err_rl("vdev legacy private object is NULL");
		return QDF_STATUS_E_FAILURE;
	}

	for (i = 0; i < MAX_ROAM_CANDIDATE_AP; i++) {
		if (is_entry_exist &&
		    mlme_priv->auth_log[i][0].diag_cmn.ktime_us &&
		    qdf_is_macaddr_equal((struct qdf_mac_addr *)pkt_info->diag_cmn.bssid,
					 (struct qdf_mac_addr *)mlme_priv->auth_log[i][0].diag_cmn.bssid)) {
			/*
			 * Frames for given bssid already exists store the new
			 * frame in corresponding array in empty slot
			 */
			for (j = 0; j < WLAN_ROAM_MAX_CACHED_AUTH_FRAMES; j++) {
				if (mlme_priv->auth_log[i][j].diag_cmn.ktime_us)
					continue;

				mlme_priv->auth_log[i][j] = *pkt_info;
				break;
			}

		} else if (!is_entry_exist &&
			   !mlme_priv->auth_log[i][0].diag_cmn.ktime_us) {
			/*
			 * For given record, there is no existing bssid
			 * so add the entry at first available slot
			 */
			mlme_priv->auth_log[i][0] = *pkt_info;
			break;
		}
	}

	return QDF_STATUS_SUCCESS;
}

static void
wlan_cache_connectivity_log(struct wlan_objmgr_psoc *psoc, uint8_t vdev_id,
			    struct wlan_diag_packet_info *pkt_info)
{
	struct wlan_objmgr_vdev *vdev;

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
						    WLAN_MLME_OBJMGR_ID);
	if (!vdev) {
		logging_err_rl("Invalid vdev:%d", vdev_id);
		return;
	}

	wlan_add_sae_log_record_to_available_slot(psoc, vdev, pkt_info);

	wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);
}
#endif

#define WLAN_SAE_AUTH_ALGO_NUMBER 3
#ifdef CONNECTIVITY_DIAG_EVENT

#ifdef WLAN_FEATURE_11BE_MLO
static QDF_STATUS
wlan_populate_link_addr(struct wlan_objmgr_vdev *vdev,
			struct wlan_diag_sta_info *wlan_diag_event)
{
	uint i = 0;
	struct mlo_link_switch_context *link_ctx = vdev->mlo_dev_ctx->link_ctx;
	struct wlan_channel *link_chan_info;

	if (!link_ctx)
		return QDF_STATUS_E_FAILURE;

	for (i = 0; i < WLAN_MAX_ML_BSS_LINKS; i++) {
		link_chan_info = link_ctx->links_info[i].link_chan_info;

		if (wlan_reg_is_24ghz_ch_freq(
		    (qdf_freq_t)link_chan_info->ch_freq)) {
			qdf_mem_copy(wlan_diag_event->mac_2g,
				     link_ctx->links_info[i].link_addr.bytes,
				     QDF_MAC_ADDR_SIZE);
		} else if (wlan_reg_is_5ghz_ch_freq(
			   (qdf_freq_t)link_chan_info->ch_freq)) {
			qdf_mem_copy(wlan_diag_event->mac_5g,
				     link_ctx->links_info[i].link_addr.bytes,
				     QDF_MAC_ADDR_SIZE);
		} else if (wlan_reg_is_6ghz_chan_freq(
			   link_chan_info->ch_freq)) {
			qdf_mem_copy(wlan_diag_event->mac_6g,
				     link_ctx->links_info[i].link_addr.bytes,
				     QDF_MAC_ADDR_SIZE);
		}
	}

	return QDF_STATUS_SUCCESS;
}

static uint8_t
wlan_populate_band_bitmap(struct mlo_link_switch_context *link_ctx)
{
	uint8_t i, band_bitmap = 0, band;
	struct wlan_channel *link_chan_info;

	for (i = 0; i < WLAN_MAX_ML_BSS_LINKS; i++) {
		link_chan_info = link_ctx->links_info[i].link_chan_info;

		band =  wlan_reg_freq_to_band((qdf_freq_t)
					      link_chan_info->ch_freq);

		band_bitmap |= BIT(band);
	}

	return band_bitmap;
}

static QDF_STATUS
wlan_populate_mlo_mgmt_event_param(struct wlan_objmgr_vdev *vdev,
				   struct wlan_diag_packet_info *data,
				   enum wlan_main_tag tag)
{
	struct mlo_link_switch_context *link_ctx;
	struct qdf_mac_addr peer_mac;
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	if (!mlo_is_mld_sta(vdev))
		return status;

	if (wlan_vdev_mlme_is_mlo_link_vdev(vdev))
		return QDF_STATUS_E_INVAL;

	status = wlan_vdev_get_bss_peer_mac(vdev, &peer_mac);
	if (QDF_IS_STATUS_ERROR(status)) {
		logging_err("vdev: %d bss peer not found",
			    wlan_vdev_get_id(vdev));
		return status;
	}

	qdf_mem_copy(data->diag_cmn.bssid,
		     peer_mac.bytes,
		     QDF_MAC_ADDR_SIZE);

	qdf_mem_copy(data->mld_addr,
		     wlan_vdev_mlme_get_mldaddr(vdev),
		     QDF_MAC_ADDR_SIZE);

	if (tag == WLAN_ASSOC_REQ ||
	    tag == WLAN_REASSOC_REQ) {
		link_ctx = vdev->mlo_dev_ctx->link_ctx;
		if (!link_ctx) {
			logging_debug("vdev: %d link_ctx not found",
				      wlan_vdev_get_id(vdev));
			return QDF_STATUS_E_INVAL;
		}

		data->supported_links =
			wlan_populate_band_bitmap(link_ctx);
	}

	return status;
}

enum wlan_diag_wifi_band
wlan_convert_freq_to_diag_band(uint16_t ch_freq)
{
	enum reg_wifi_band band;

	band = wlan_reg_freq_to_band((qdf_freq_t)ch_freq);

	switch (band) {
	case REG_BAND_2G:
		return WLAN_24GHZ_BAND;
	case REG_BAND_5G:
		return WLAN_5GHZ_BAND;
	case REG_BAND_6G:
		return WLAN_6GHZ_BAND;
	default:
		return WLAN_INVALID_BAND;
	}
}

#define REJECTED_LINK_STATUS 1

void
wlan_connectivity_mlo_setup_event(struct wlan_objmgr_vdev *vdev)
{
	uint i = 0;
	struct mlo_link_switch_context *link_ctx = NULL;
	struct wlan_channel *chan_info;

	WLAN_HOST_DIAG_EVENT_DEF(wlan_diag_event,
				 struct wlan_diag_mlo_setup);

	if (!mlo_is_mld_sta(vdev))
		return;

	qdf_mem_zero(&wlan_diag_event, sizeof(struct wlan_diag_mlo_setup));

	wlan_diag_event.diag_cmn.ktime_us = qdf_ktime_to_us(qdf_ktime_get());
	wlan_diag_event.diag_cmn.timestamp_us = qdf_get_time_of_the_day_us();
	wlan_diag_event.version = DIAG_MLO_SETUP_VERSION;

	if (!vdev->mlo_dev_ctx) {
		logging_err("vdev: %d MLO dev ctx not found",
			    wlan_vdev_get_id(vdev));
		return;
	}

	link_ctx = vdev->mlo_dev_ctx->link_ctx;
	if (!link_ctx) {
		logging_err("vdev: %d mlo link ctx not found",
			    wlan_vdev_get_id(vdev));
		return;
	}

	for (i = 0; i < WLAN_MAX_ML_BSS_LINKS; i++) {
		wlan_diag_event.mlo_cmn_info[i].link_id =
				link_ctx->links_info[i].link_id;
		wlan_diag_event.mlo_cmn_info[i].vdev_id =
				link_ctx->links_info[i].vdev_id;

		qdf_mem_copy(wlan_diag_event.mlo_cmn_info[i].link_addr,
			     link_ctx->links_info[i].ap_link_addr.bytes,
			     QDF_MAC_ADDR_SIZE);

		chan_info = link_ctx->links_info[i].link_chan_info;

		wlan_diag_event.mlo_cmn_info[i].band =
			wlan_convert_freq_to_diag_band(chan_info->ch_freq);

		if (wlan_diag_event.mlo_cmn_info[i].band == WLAN_INVALID_BAND)
			wlan_diag_event.mlo_cmn_info[i].status =
							REJECTED_LINK_STATUS;
	}

	WLAN_HOST_DIAG_EVENT_REPORT(&wlan_diag_event, EVENT_WLAN_MLO_SETUP);
}

#else
static QDF_STATUS
wlan_populate_link_addr(struct wlan_objmgr_vdev *vdev,
			struct wlan_diag_sta_info *wlan_diag_event)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS
wlan_populate_mlo_mgmt_event_param(struct wlan_objmgr_vdev *vdev,
				   struct wlan_diag_packet_info *data,
				   enum wlan_main_tag tag)
{
	return QDF_STATUS_SUCCESS;
}
#endif

void
wlan_connectivity_sta_info_event(struct wlan_objmgr_psoc *psoc, uint8_t vdev_id)
{
	QDF_STATUS status;
	struct wlan_objmgr_vdev *vdev = NULL;

	WLAN_HOST_DIAG_EVENT_DEF(wlan_diag_event, struct wlan_diag_sta_info);

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
						    WLAN_MLME_OBJMGR_ID);
	if (!vdev) {
		logging_err_rl("Invalid vdev:%d", vdev_id);
		return;
	}

	if (wlan_vdev_mlme_get_opmode(vdev) != QDF_STA_MODE)
		goto out;

	wlan_diag_event.diag_cmn.timestamp_us = qdf_get_time_of_the_day_us();
	wlan_diag_event.diag_cmn.ktime_us = qdf_ktime_to_us(qdf_ktime_get());
	wlan_diag_event.diag_cmn.vdev_id = vdev_id;
	wlan_diag_event.is_mlo = mlo_is_mld_sta(vdev);

	if (wlan_diag_event.is_mlo) {
		qdf_mem_copy(wlan_diag_event.diag_cmn.bssid,
			     wlan_vdev_mlme_get_mldaddr(vdev),
			     QDF_MAC_ADDR_SIZE);
		status = wlan_populate_link_addr(vdev, &wlan_diag_event);
		if (QDF_IS_STATUS_ERROR(status)) {
			logging_err_rl("wlan_populate_link_addr failed");
			goto out;
		}
	} else {
		qdf_mem_copy(wlan_diag_event.diag_cmn.bssid,
			     wlan_vdev_mlme_get_macaddr(vdev),
			     QDF_MAC_ADDR_SIZE);
	}

	WLAN_HOST_DIAG_EVENT_REPORT(&wlan_diag_event, EVENT_WLAN_STA_INFO);

out:
	wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);
}

void
wlan_populate_vsie(struct wlan_objmgr_vdev *vdev,
		   struct wlan_diag_packet_info *data,
		   bool is_tx)
{
	struct element_info *vsie_info = NULL;

	if (is_tx)
		vsie_info = mlme_get_self_disconnect_ies(vdev);
	else
		vsie_info = mlme_get_peer_disconnect_ies(vdev);

	if (!vsie_info)
		return;

	data->vsie_len = vsie_info->len;
	if (data->vsie_len > MAX_VSIE_LEN)
		data->vsie_len = MAX_VSIE_LEN;

	qdf_mem_copy(data->vsie, vsie_info->ptr, data->vsie_len);
}

void
wlan_connectivity_mgmt_event(struct wlan_objmgr_psoc *psoc,
			     struct wlan_frame_hdr *mac_hdr,
			     uint8_t vdev_id, uint16_t status_code,
			     enum qdf_dp_tx_rx_status tx_status,
			     int8_t peer_rssi,
			     uint8_t auth_algo, uint8_t auth_type,
			     uint8_t auth_seq, uint16_t aid,
			     enum wlan_main_tag tag)
{
	enum QDF_OPMODE opmode;
	bool is_auth_frame_caching_required, is_initial_connection;
	struct wlan_objmgr_vdev *vdev;
	QDF_STATUS status;

	WLAN_HOST_DIAG_EVENT_DEF(wlan_diag_event, struct wlan_diag_packet_info);

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
						    WLAN_MLME_OBJMGR_ID);
	if (!vdev) {
		logging_debug("Unable to find vdev:%d", vdev_id);
		return;
	}

	opmode = wlan_vdev_mlme_get_opmode(vdev);
	if (opmode != QDF_STA_MODE)
		goto out;

	if (mlo_is_mld_sta(vdev) &&
	    wlan_vdev_mlme_is_mlo_link_vdev(vdev))
		goto out;

	is_initial_connection = wlan_cm_is_vdev_connecting(vdev);

	qdf_mem_zero(&wlan_diag_event, sizeof(struct wlan_diag_packet_info));

	wlan_diag_event.diag_cmn.timestamp_us = qdf_get_time_of_the_day_us();
	wlan_diag_event.diag_cmn.ktime_us = qdf_ktime_to_us(qdf_ktime_get());
	wlan_diag_event.diag_cmn.vdev_id = vdev_id;
	wlan_diag_event.subtype = (uint8_t)tag;

	qdf_mem_copy(wlan_diag_event.diag_cmn.bssid, &mac_hdr->i_addr3[0],
		     QDF_MAC_ADDR_SIZE);

	status = wlan_populate_mlo_mgmt_event_param(vdev, &wlan_diag_event,
						    tag);
	if (QDF_IS_STATUS_ERROR(status))
		goto out;

	wlan_diag_event.version = DIAG_MGMT_VERSION_V2;
	wlan_diag_event.tx_fail_reason = tx_status;
	wlan_diag_event.tx_status = wlan_get_diag_tx_status(tx_status);
	wlan_diag_event.rssi = peer_rssi;
	wlan_diag_event.sn =
		(le16toh(*(uint16_t *)mac_hdr->i_seq) >> WLAN_SEQ_SEQ_SHIFT);
	wlan_diag_event.status = status_code;
	wlan_diag_event.auth_algo = auth_algo;
	wlan_diag_event.auth_frame_type = auth_type;
	wlan_diag_event.auth_seq_num = auth_seq;
	wlan_diag_event.assoc_id = aid;

	if (tag == WLAN_DEAUTH_TX || tag == WLAN_DISASSOC_TX)
		wlan_populate_vsie(vdev, &wlan_diag_event, true);

	if (wlan_diag_event.subtype > WLAN_CONN_DIAG_REASSOC_RESP_EVENT &&
	    wlan_diag_event.subtype < WLAN_CONN_DIAG_BMISS_EVENT)
		wlan_diag_event.reason = status_code;

	wlan_diag_event.is_retry_frame =
			(mac_hdr->i_fc[1] & IEEE80211_FC1_RETRY);
	is_auth_frame_caching_required =
		wlan_psoc_nif_fw_ext2_cap_get(psoc,
					      WLAN_ROAM_STATS_FRAME_INFO_PER_CANDIDATE);
	if (!is_initial_connection &&
	    (tag == WLAN_AUTH_REQ || tag == WLAN_AUTH_RESP) &&
	    auth_algo == WLAN_SAE_AUTH_ALGO_NUMBER &&
	    is_auth_frame_caching_required)
		wlan_cache_connectivity_log(psoc, vdev_id, &wlan_diag_event);
	else
		WLAN_HOST_DIAG_EVENT_REPORT(&wlan_diag_event, EVENT_WLAN_MGMT);

	if (tag == WLAN_ASSOC_RSP || tag == WLAN_REASSOC_RSP)
		wlan_connectivity_mlo_setup_event(vdev);
out:
	wlan_objmgr_vdev_release_ref(vdev, WLAN_MLME_OBJMGR_ID);

}

#ifdef WLAN_FEATURE_11BE_MLO
void wlan_connectivity_mld_link_status_event(struct wlan_objmgr_psoc *psoc,
					     struct mlo_link_switch_params *src)
{
	WLAN_HOST_DIAG_EVENT_DEF(wlan_diag_event,
				 struct wlan_diag_mlo_link_status);

	qdf_mem_zero(&wlan_diag_event,

		     sizeof(struct wlan_diag_mlo_link_status));

	wlan_diag_event.diag_cmn.timestamp_us = qdf_get_time_of_the_day_us();
	wlan_diag_event.diag_cmn.ktime_us = qdf_ktime_to_us(qdf_ktime_get());
	wlan_diag_event.version = DIAG_MLO_LINK_STATUS_VERSION;

	wlan_diag_event.active_link = src->active_link_bitmap;
	wlan_diag_event.prev_active_link = src->prev_link_bitmap;
	wlan_diag_event.reason = src->reason_code;
	wlan_diag_event.diag_cmn.fw_timestamp = src->fw_timestamp;

	WLAN_HOST_DIAG_EVENT_REPORT(&wlan_diag_event,
				    EVENT_WLAN_MLO_LINK_STATUS);
}
#endif
#endif
