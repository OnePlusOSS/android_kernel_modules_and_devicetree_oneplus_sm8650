/*
 * Copyright (c) 2019-2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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
 * DOC: target_if_vdev_mgr_rx_ops.c
 *
 * This file provide definition for APIs registered through events received
 * from FW
 */
#include <target_if_vdev_mgr_rx_ops.h>
#include <target_if_vdev_mgr_tx_ops.h>
#include <wlan_vdev_mgr_tgt_if_rx_defs.h>
#include <wlan_vdev_mgr_tgt_if_tx_defs.h>
#include <wmi_unified_param.h>
#include <wlan_mlme_dbg.h>
#include <target_if.h>
#include <wlan_vdev_mlme_main.h>
#include <wmi_unified_vdev_api.h>
#include <target_if_psoc_wake_lock.h>
#ifdef WLAN_FEATURE_ROAM_OFFLOAD
#include <target_if_cm_roam_offload.h>
#endif
#include <wlan_reg_services_api.h>
#ifdef DP_UMAC_HW_RESET_SUPPORT
#include <cdp_txrx_ctrl.h>
#endif

static inline
void target_if_vdev_mgr_handle_recovery(struct wlan_objmgr_psoc *psoc,
					uint8_t vdev_id,
					enum qdf_hang_reason recovery_reason,
					uint16_t rsp_pos)
{
	mlme_nofl_err("PSOC_%d VDEV_%d: %s rsp timeout", wlan_psoc_get_id(psoc),
		      vdev_id, string_from_rsp_bit(rsp_pos));
	if (target_if_vdev_mgr_is_panic_allowed())
		qdf_trigger_self_recovery(psoc, recovery_reason);
	else
		mlme_nofl_debug("PSOC_%d VDEV_%d: Panic not allowed",
				wlan_psoc_get_id(psoc), vdev_id);
}

#ifdef WLAN_FEATURE_ROAM_OFFLOAD
static inline QDF_STATUS
target_if_send_rso_stop_failure_rsp(struct wlan_objmgr_psoc *psoc,
				    uint8_t vdev_id)
{
	return target_if_cm_send_rso_stop_failure_rsp(psoc, vdev_id);
}
#else
static inline QDF_STATUS
target_if_send_rso_stop_failure_rsp(struct wlan_objmgr_psoc *psoc,
				    uint8_t vdev_id)
{
	return QDF_STATUS_E_NOSUPPORT;
}
#endif

#ifdef WLAN_FEATURE_DYNAMIC_MAC_ADDR_UPDATE
static void
target_if_vdev_mgr_mac_addr_rsp_timeout(struct wlan_objmgr_psoc *psoc,
					struct vdev_response_timer *vdev_rsp,
					uint8_t vdev_id)
{
	uint16_t rsp_pos;
	struct wlan_objmgr_vdev *vdev;
	enum qdf_hang_reason recovery_reason;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops) {
		mlme_err("No Rx Ops");
		return;
	}

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
						    WLAN_VDEV_TARGET_IF_ID);
	if (!vdev) {
		mlme_err("Invalid vdev %d", vdev_id);
		return;
	}

	rsp_pos = UPDATE_MAC_ADDR_RESPONSE_BIT;
	recovery_reason = QDF_VDEV_MAC_ADDR_UPDATE_RESPONSE_TIMED_OUT;
	target_if_vdev_mgr_rsp_timer_stop(psoc, vdev_rsp, rsp_pos);
	target_if_vdev_mgr_handle_recovery(psoc, vdev_id,
					   recovery_reason, rsp_pos);
	rx_ops->vdev_mgr_set_mac_addr_response(vdev, -EAGAIN);
	wlan_objmgr_vdev_release_ref(vdev, WLAN_VDEV_TARGET_IF_ID);
}
#else
static inline void
target_if_vdev_mgr_mac_addr_rsp_timeout(struct wlan_objmgr_psoc *psoc,
					struct vdev_response_timer *vdev_rsp,
					uint8_t vdev_id)
{
}
#endif

#ifdef DP_UMAC_HW_RESET_SUPPORT
/**
 * target_if_check_and_restart_vdev_mgr_rsp_timer - Check and restart the vdev
 * manager response timer if UMAC reset is in progress
 * @vdev_rsp: Pointer to vdev response timer structure
 *
 * Return: QDF_STATUS
 */
static inline QDF_STATUS
target_if_check_and_restart_vdev_mgr_rsp_timer(
		struct vdev_response_timer *vdev_rsp)
{
	ol_txrx_soc_handle soc_txrx_handle;

	soc_txrx_handle = wlan_psoc_get_dp_handle(vdev_rsp->psoc);

	if (!soc_txrx_handle)
		return QDF_STATUS_E_INVAL;

	/* Restart the timer if UMAC reset is inprogress */
	if (cdp_umac_reset_is_inprogress(soc_txrx_handle)) {
		mlme_debug("Umac reset is in progress, restart the vdev manager response timer");
		qdf_timer_mod(&vdev_rsp->rsp_timer, vdev_rsp->expire_time);
		return QDF_STATUS_SUCCESS;
	}

	return QDF_STATUS_E_FAILURE;
}
#else
static inline QDF_STATUS
target_if_check_and_restart_vdev_mgr_rsp_timer(
		struct vdev_response_timer *vdev_rsp)
{
	return QDF_STATUS_E_FAILURE;
}
#endif

void target_if_vdev_mgr_rsp_timer_cb(void *arg)
{
	struct wlan_objmgr_psoc *psoc;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	struct vdev_start_response start_rsp = {0};
	struct vdev_stop_response stop_rsp = {0};
	struct vdev_delete_response del_rsp = {0};
	struct peer_delete_all_response peer_del_all_rsp = {0};
	struct vdev_response_timer *vdev_rsp = arg;
	enum qdf_hang_reason recovery_reason;
	uint8_t vdev_id;
	uint16_t rsp_pos = RESPONSE_BIT_MAX;
	QDF_STATUS status;

	if (!vdev_rsp) {
		mlme_err("Vdev response timer is NULL");
		return;
	}

	psoc = vdev_rsp->psoc;
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->psoc_get_vdev_response_timer_info) {
		mlme_err("No Rx Ops");
		return;
	}

	if (!qdf_atomic_test_bit(START_RESPONSE_BIT, &vdev_rsp->rsp_status) &&
	    !qdf_atomic_test_bit(RESTART_RESPONSE_BIT, &vdev_rsp->rsp_status) &&
	    !qdf_atomic_test_bit(STOP_RESPONSE_BIT, &vdev_rsp->rsp_status) &&
	    !qdf_atomic_test_bit(DELETE_RESPONSE_BIT, &vdev_rsp->rsp_status) &&
	    !qdf_atomic_test_bit(PEER_DELETE_ALL_RESPONSE_BIT,
				 &vdev_rsp->rsp_status) &&
	    !qdf_atomic_test_bit(RSO_STOP_RESPONSE_BIT,
				 &vdev_rsp->rsp_status) &&
	    !qdf_atomic_test_bit(UPDATE_MAC_ADDR_RESPONSE_BIT,
				 &vdev_rsp->rsp_status)) {
		mlme_debug("No response bit is set, ignoring actions :%d",
			   vdev_rsp->vdev_id);
		return;
	}

	vdev_id = vdev_rsp->vdev_id;
	if (vdev_id >= WLAN_UMAC_PSOC_MAX_VDEVS) {
		mlme_err("Invalid VDEV_%d PSOC_%d", vdev_id,
			 wlan_psoc_get_id(psoc));
		return;
	}

	vdev_rsp->timer_status = QDF_STATUS_E_TIMEOUT;
	if (qdf_atomic_test_bit(START_RESPONSE_BIT,
				&vdev_rsp->rsp_status) ||
	    qdf_atomic_test_bit(RESTART_RESPONSE_BIT,
				&vdev_rsp->rsp_status)) {
		start_rsp.vdev_id = vdev_id;
		start_rsp.status = WLAN_MLME_HOST_VDEV_START_TIMEOUT;
		if (qdf_atomic_test_bit(START_RESPONSE_BIT,
					&vdev_rsp->rsp_status)) {
			start_rsp.resp_type =
				WMI_HOST_VDEV_START_RESP_EVENT;
			rsp_pos = START_RESPONSE_BIT;
			recovery_reason = QDF_VDEV_START_RESPONSE_TIMED_OUT;
		} else {
			start_rsp.resp_type =
				WMI_HOST_VDEV_RESTART_RESP_EVENT;
			rsp_pos = RESTART_RESPONSE_BIT;
			recovery_reason = QDF_VDEV_RESTART_RESPONSE_TIMED_OUT;
		}

		target_if_vdev_mgr_rsp_timer_stop(psoc, vdev_rsp, rsp_pos);
		target_if_vdev_mgr_handle_recovery(psoc, vdev_id,
						   recovery_reason, rsp_pos);
		rx_ops->vdev_mgr_start_response(psoc, &start_rsp);
	} else if (qdf_atomic_test_bit(STOP_RESPONSE_BIT,
				       &vdev_rsp->rsp_status)) {
		rsp_pos = STOP_RESPONSE_BIT;
		stop_rsp.vdev_id = vdev_id;
		recovery_reason = QDF_VDEV_STOP_RESPONSE_TIMED_OUT;

		target_if_vdev_mgr_rsp_timer_stop(psoc, vdev_rsp, rsp_pos);
		target_if_vdev_mgr_handle_recovery(psoc, vdev_id,
						   recovery_reason, rsp_pos);
		rx_ops->vdev_mgr_stop_response(psoc, &stop_rsp);
	} else if (qdf_atomic_test_bit(DELETE_RESPONSE_BIT,
				       &vdev_rsp->rsp_status)) {
		status = target_if_check_and_restart_vdev_mgr_rsp_timer(
				vdev_rsp);
		if (QDF_IS_STATUS_SUCCESS(status))
			return;

		del_rsp.vdev_id = vdev_id;
		rsp_pos = DELETE_RESPONSE_BIT;
		recovery_reason = QDF_VDEV_DELETE_RESPONSE_TIMED_OUT;
		target_if_vdev_mgr_rsp_timer_stop(psoc, vdev_rsp, rsp_pos);
		target_if_vdev_mgr_handle_recovery(psoc, vdev_id,
						   recovery_reason, rsp_pos);
		rx_ops->vdev_mgr_delete_response(psoc, &del_rsp);
	} else if (qdf_atomic_test_bit(PEER_DELETE_ALL_RESPONSE_BIT,
				&vdev_rsp->rsp_status)) {
		status = target_if_check_and_restart_vdev_mgr_rsp_timer(
				vdev_rsp);
		if (QDF_IS_STATUS_SUCCESS(status))
			return;

		peer_del_all_rsp.vdev_id = vdev_id;
		peer_del_all_rsp.peer_type_bitmap = vdev_rsp->peer_type_bitmap;
		rsp_pos = PEER_DELETE_ALL_RESPONSE_BIT;
		recovery_reason = QDF_VDEV_PEER_DELETE_ALL_RESPONSE_TIMED_OUT;
		target_if_vdev_mgr_rsp_timer_stop(psoc, vdev_rsp, rsp_pos);
		target_if_vdev_mgr_handle_recovery(psoc, vdev_id,
						   recovery_reason, rsp_pos);
		rx_ops->vdev_mgr_peer_delete_all_response(psoc,
							  &peer_del_all_rsp);
	} else if (qdf_atomic_test_bit(RSO_STOP_RESPONSE_BIT,
				       &vdev_rsp->rsp_status)) {
		rsp_pos = RSO_STOP_RESPONSE_BIT;
		target_if_vdev_mgr_rsp_timer_stop(psoc, vdev_rsp, rsp_pos);
		/**
		 * FW did not respond to rso stop cmd, as roaming is
		 * disabled either due to race condition
		 * that happened during previous disconnect OR
		 * supplicant disabled roaming.
		 * To solve this issue, skip recovery and host will
		 * continue disconnect and cleanup rso state.
		 */
		mlme_debug("No rsp from FW received , continue with disconnect");
		target_if_send_rso_stop_failure_rsp(psoc, vdev_id);
	} else if (qdf_atomic_test_bit(UPDATE_MAC_ADDR_RESPONSE_BIT,
				       &vdev_rsp->rsp_status)) {
		mlme_debug("VDEV %d MAC addr update resp timeout", vdev_id);
		target_if_vdev_mgr_mac_addr_rsp_timeout(psoc,
							vdev_rsp, vdev_id);
	} else {
		mlme_err("PSOC_%d VDEV_%d: Unknown error",
			 wlan_psoc_get_id(psoc), vdev_id);
		return;
	}
}

#ifdef SERIALIZE_VDEV_RESP
static QDF_STATUS target_if_vdev_mgr_rsp_flush_cb_mc(struct scheduler_msg *msg)
{
	struct vdev_response_timer *vdev_rsp;
	struct wlan_objmgr_psoc *psoc;

	if (!msg->bodyptr) {
		mlme_err("Message bodyptr is NULL");
		return QDF_STATUS_E_INVAL;
	}

	vdev_rsp = scheduler_qdf_mc_timer_deinit_return_data_ptr(msg->bodyptr);
	if (!vdev_rsp) {
		mlme_err("vdev response timer is NULL");
		return QDF_STATUS_E_INVAL;
	}

	psoc = vdev_rsp->psoc;
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return QDF_STATUS_E_INVAL;
	}

	if (vdev_rsp->rsp_status)
		wlan_objmgr_psoc_release_ref(psoc, WLAN_PSOC_TARGET_IF_ID);

	return QDF_STATUS_SUCCESS;
}

static void
target_if_vdev_mgr_rsp_cb_mc_ctx(void *arg)
{
	struct scheduler_msg msg = {0};
	struct vdev_response_timer *vdev_rsp = arg;
	struct wlan_objmgr_psoc *psoc;
	struct sched_qdf_mc_timer_cb_wrapper *mc_timer_wrapper;

	psoc = vdev_rsp->psoc;
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return;
	}

	msg.type = SYS_MSG_ID_MC_TIMER;
	msg.reserved = SYS_MSG_COOKIE;

	mc_timer_wrapper = scheduler_qdf_mc_timer_init(
			target_if_vdev_mgr_rsp_timer_cb,
			arg);

	if (!mc_timer_wrapper) {
		mlme_err("failed to allocate sched_qdf_mc_timer_cb_wrapper");
		return;
	}

	msg.callback = scheduler_qdf_mc_timer_callback_t_wrapper;
	msg.bodyptr = mc_timer_wrapper;
	msg.bodyval = 0;
	msg.flush_callback = target_if_vdev_mgr_rsp_flush_cb_mc;

	if (scheduler_post_message(QDF_MODULE_ID_TARGET_IF,
				   QDF_MODULE_ID_TARGET_IF,
				   QDF_MODULE_ID_SYS, &msg) ==
				   QDF_STATUS_SUCCESS)
		return;

	mlme_err("Could not enqueue timer to timer queue");
	qdf_mem_free(mc_timer_wrapper);
	if (psoc)
		wlan_objmgr_psoc_release_ref(psoc, WLAN_PSOC_TARGET_IF_ID);
}

void target_if_vdev_mgr_rsp_timer_mgmt_cb(void *arg)
{
	target_if_vdev_mgr_rsp_cb_mc_ctx(arg);
}

#define VDEV_RSP_RX_CTX WMI_RX_SERIALIZER_CTX
#else
void target_if_vdev_mgr_rsp_timer_mgmt_cb(void *arg)
{
	target_if_vdev_mgr_rsp_timer_cb(arg);
}

#define VDEV_RSP_RX_CTX WMI_RX_UMAC_CTX
#endif

static int target_if_vdev_mgr_start_response_handler(ol_scn_t scn,
						     uint8_t *data,
						     uint32_t datalen)
{
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	struct vdev_start_response vdev_start_resp = {0};
	uint8_t vdev_id;
	struct vdev_response_timer *vdev_rsp;

	if (!scn || !data) {
		mlme_err("Invalid input");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_start_response) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	if (wmi_extract_vdev_start_resp(wmi_handle, data, &vdev_start_resp)) {
		mlme_err("WMI extract failed");
		return -EINVAL;
	}

	vdev_id = vdev_start_resp.vdev_id;
	vdev_rsp = rx_ops->psoc_get_vdev_response_timer_info(psoc, vdev_id);
	if (!vdev_rsp) {
		mlme_err("vdev response timer is null VDEV_%d PSOC_%d",
			 vdev_id, wlan_psoc_get_id(psoc));
		return -EINVAL;
	}

	if (vdev_start_resp.resp_type == WMI_HOST_VDEV_RESTART_RESP_EVENT)
		status = target_if_vdev_mgr_rsp_timer_stop(
							psoc, vdev_rsp,
							RESTART_RESPONSE_BIT);
	else
		status = target_if_vdev_mgr_rsp_timer_stop(psoc, vdev_rsp,
							   START_RESPONSE_BIT);

	if (QDF_IS_STATUS_ERROR(status)) {
		mlme_err("PSOC_%d VDEV_%d: VDE MGR RSP Timer stop failed",
			 psoc->soc_objmgr.psoc_id, vdev_id);
		goto err;
	}

	status = rx_ops->vdev_mgr_start_response(psoc, &vdev_start_resp);

err:
	return qdf_status_to_os_return(status);
}

static int target_if_vdev_mgr_stop_response_handler(ol_scn_t scn,
						    uint8_t *data,
						    uint32_t datalen)
{
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	struct vdev_stop_response rsp = {0};
	uint32_t vdev_id;
	struct vdev_response_timer *vdev_rsp;

	if (!scn || !data) {
		mlme_err("Invalid input");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_stop_response) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	if (wmi_extract_vdev_stopped_param(wmi_handle, data, &vdev_id)) {
		mlme_err("WMI extract failed");
		return -EINVAL;
	}

	vdev_rsp = rx_ops->psoc_get_vdev_response_timer_info(psoc, vdev_id);
	if (!vdev_rsp) {
		mlme_err("vdev response timer is null VDEV_%d PSOC_%d",
			 vdev_id, wlan_psoc_get_id(psoc));
		return -EINVAL;
	}

	status = target_if_vdev_mgr_rsp_timer_stop(psoc, vdev_rsp,
						   STOP_RESPONSE_BIT);

	if (QDF_IS_STATUS_ERROR(status)) {
		mlme_err("PSOC_%d VDEV_%d: VDE MGR RSP Timer stop failed",
			 psoc->soc_objmgr.psoc_id, vdev_id);
		goto err;
	}

	rsp.vdev_id = vdev_id;
	status = rx_ops->vdev_mgr_stop_response(psoc, &rsp);

err:
	return qdf_status_to_os_return(status);
}

static int target_if_vdev_mgr_delete_response_handler(ol_scn_t scn,
						      uint8_t *data,
						      uint32_t datalen)
{
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	struct vdev_delete_response vdev_del_resp = {0};
	struct vdev_response_timer *vdev_rsp;

	if (!scn || !data) {
		mlme_err("Invalid input");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_delete_response) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	if (wmi_extract_vdev_delete_resp(wmi_handle, data, &vdev_del_resp)) {
		mlme_err("WMI extract failed");
		return -EINVAL;
	}

	vdev_rsp = rx_ops->psoc_get_vdev_response_timer_info(psoc,
							 vdev_del_resp.vdev_id);
	if (!vdev_rsp) {
		mlme_err("vdev response timer is null VDEV_%d PSOC_%d",
			 vdev_del_resp.vdev_id, wlan_psoc_get_id(psoc));
		return -EINVAL;
	}

	status = target_if_vdev_mgr_rsp_timer_stop(
						psoc, vdev_rsp,
						DELETE_RESPONSE_BIT);

	if (QDF_IS_STATUS_ERROR(status)) {
		mlme_err("PSOC_%d VDEV_%d: VDE MGR RSP Timer stop failed",
			 wlan_psoc_get_id(psoc), vdev_del_resp.vdev_id);
		goto err;
	}

	status = rx_ops->vdev_mgr_delete_response(psoc, &vdev_del_resp);
	target_if_wake_lock_timeout_release(psoc, DELETE_WAKELOCK);
err:
	return qdf_status_to_os_return(status);
}

static int target_if_vdev_mgr_peer_delete_all_response_handler(
							ol_scn_t scn,
							uint8_t *data,
							uint32_t datalen)
{
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	struct peer_delete_all_response vdev_peer_del_all_resp = {0};
	struct vdev_response_timer *vdev_rsp;

	if (!scn || !data) {
		mlme_err("Invalid input");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_peer_delete_all_response) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	if (wmi_extract_vdev_peer_delete_all_response_event(
						wmi_handle, data,
						&vdev_peer_del_all_resp)) {
		mlme_err("WMI extract failed");
		return -EINVAL;
	}

	vdev_rsp = rx_ops->psoc_get_vdev_response_timer_info(psoc,
						vdev_peer_del_all_resp.vdev_id);
	if (!vdev_rsp) {
		mlme_err("vdev response timer is null VDEV_%d PSOC_%d",
			 vdev_peer_del_all_resp.vdev_id,
			 wlan_psoc_get_id(psoc));
		return -EINVAL;
	}

	status = target_if_vdev_mgr_rsp_timer_stop(
						psoc,
						vdev_rsp,
						PEER_DELETE_ALL_RESPONSE_BIT);

	if (QDF_IS_STATUS_ERROR(status)) {
		mlme_err("PSOC_%d VDEV_%d: VDE MGR RSP Timer stop failed",
			 psoc->soc_objmgr.psoc_id,
			 vdev_peer_del_all_resp.vdev_id);
		goto err;
	}

	vdev_peer_del_all_resp.peer_type_bitmap = vdev_rsp->peer_type_bitmap;

	status = rx_ops->vdev_mgr_peer_delete_all_response(
						psoc,
						&vdev_peer_del_all_resp);

err:
	return qdf_status_to_os_return(status);
}

int target_if_vdev_mgr_offload_bcn_tx_status_handler(
					ol_scn_t scn,
					uint8_t *data,
					uint32_t datalen)
{
	QDF_STATUS status;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	uint32_t vdev_id, tx_status;

	if (!scn || !data) {
		mlme_err("Invalid input");
		return -EINVAL;
	}
	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_offload_bcn_tx_status_event_handle) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	if (wmi_extract_offload_bcn_tx_status_evt(wmi_handle, data,
						  &vdev_id, &tx_status)) {
		mlme_err("WMI extract failed");
		return -EINVAL;
	}

	status = rx_ops->vdev_mgr_offload_bcn_tx_status_event_handle(
								vdev_id,
								tx_status);

	return qdf_status_to_os_return(status);
}

int target_if_vdev_mgr_tbttoffset_update_handler(
						ol_scn_t scn, uint8_t *data,
						uint32_t datalen)
{
	QDF_STATUS status;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	uint32_t num_vdevs = 0;

	if (!scn || !data) {
		mlme_err("Invalid input");
		return -EINVAL;
	}
	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_tbttoffset_update_handle) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	if (wmi_extract_tbttoffset_num_vdevs(wmi_handle, data, &num_vdevs)) {
		mlme_err("WMI extract failed");
		return -EINVAL;
	}

	status = rx_ops->vdev_mgr_tbttoffset_update_handle(num_vdevs,
							   false);

	return qdf_status_to_os_return(status);
}

int target_if_vdev_mgr_ext_tbttoffset_update_handler(
						ol_scn_t scn,
						uint8_t *data,
						uint32_t datalen)
{
	QDF_STATUS status;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	uint32_t num_vdevs = 0;

	if (!scn || !data) {
		mlme_err("Invalid input");
		return -EINVAL;
	}
	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_tbttoffset_update_handle) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	if (wmi_extract_ext_tbttoffset_num_vdevs(wmi_handle, data,
						 &num_vdevs)) {
		mlme_err("WMI extract failed");
		return -EINVAL;
	}

	status = rx_ops->vdev_mgr_tbttoffset_update_handle(num_vdevs,
							   true);

	return qdf_status_to_os_return(status);
}

static int target_if_vdev_mgr_multi_vdev_restart_resp_handler(
							ol_scn_t scn,
							uint8_t *data,
							uint32_t datalen)
{
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	struct multi_vdev_restart_resp restart_resp;
	struct vdev_response_timer *vdev_rsp;
	uint8_t max_vdevs, vdev_idx;

	if (!scn || !data) {
		mlme_err("Invalid input");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_multi_vdev_restart_resp ||
	    !rx_ops->psoc_get_vdev_response_timer_info) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	qdf_mem_zero(&restart_resp, sizeof(restart_resp));
	restart_resp.timestamp = qdf_get_log_timestamp();
	if (wmi_extract_multi_vdev_restart_resp_event(wmi_handle, data,
						      &restart_resp)) {
		mlme_err("WMI extract failed");
		return -EINVAL;
	}

	max_vdevs = wlan_psoc_get_max_vdev_count(psoc);
	for (vdev_idx = 0; vdev_idx < max_vdevs; vdev_idx++) {
		if (!qdf_test_bit(vdev_idx, restart_resp.vdev_id_bmap))
			continue;

		mlme_debug("PSOC_%d VDEV_%d: Restart resp received",
			   wlan_psoc_get_id(psoc), vdev_idx);
		vdev_rsp = rx_ops->psoc_get_vdev_response_timer_info(psoc,
								     vdev_idx);
		if (!vdev_rsp) {
			mlme_err("PSOC_%d VDEV_%d: VDEV RSP is NULL",
				 wlan_psoc_get_id(psoc), vdev_idx);
			continue;
		}

		status = target_if_vdev_mgr_rsp_timer_stop(
				psoc, vdev_rsp, RESTART_RESPONSE_BIT);
		if (QDF_IS_STATUS_ERROR(status))
			mlme_err("PSOC_%d VDEV_%d: VDE MGR RSP Timer stop failed",
				 wlan_psoc_get_id(psoc), vdev_idx);
	}

	status = rx_ops->vdev_mgr_multi_vdev_restart_resp(psoc, &restart_resp);

	return qdf_status_to_os_return(status);
}

/**
 * target_if_vdev_csa_complete - CSA complete event handler
 * @psoc: psoc
 * @vdev_id: vdev id
 *
 * Return: 0 on success
 */
static int target_if_vdev_csa_complete(struct wlan_objmgr_psoc *psoc,
				       uint8_t vdev_id)
{
	QDF_STATUS status = QDF_STATUS_E_FAILURE;
	struct vdev_mlme_obj *vdev_mlme;
	struct wlan_objmgr_vdev *vdev;
	int ret = 0;

	if (!psoc) {
		mlme_err("Invalid input");
		return -EINVAL;
	}
	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
						    WLAN_VDEV_TARGET_IF_ID);
	if (!vdev) {
		mlme_err("VDEV is NULL");
		return -EINVAL;
	}
	vdev_mlme = wlan_vdev_mlme_get_cmpt_obj(vdev);
	if (!vdev_mlme) {
		mlme_err("VDEV_%d: PSOC_%d VDEV_MLME is NULL", vdev_id,
			 wlan_psoc_get_id(psoc));
		ret = -EINVAL;
		goto end;
	}

	if ((vdev_mlme->ops) && vdev_mlme->ops->mlme_vdev_csa_complete) {
		status = vdev_mlme->ops->mlme_vdev_csa_complete(vdev_mlme);
		if (QDF_IS_STATUS_ERROR(status)) {
			mlme_err("vdev csa complete failed");
			ret = -EINVAL;
		}
	}
end:
	wlan_objmgr_vdev_release_ref(vdev, WLAN_VDEV_TARGET_IF_ID);
	return ret;
}

/**
 * target_if_pdev_csa_status_event_handler - CSA event handler
 * @scn: Pointer to scn structure
 * @data: pointer to event data
 * @datalen: event data length
 *
 * Return: 0 on success
 */
static int target_if_pdev_csa_status_event_handler(
		ol_scn_t scn,
		uint8_t *data,
		uint32_t datalen)
{
	struct pdev_csa_switch_count_status csa_status;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct target_psoc_info *tgt_hdl;
	int i;
	QDF_STATUS status;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops = NULL;

	if (!scn || !data) {
		mlme_err("Invalid input");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_set_max_channel_switch_time) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	tgt_hdl = wlan_psoc_get_tgt_if_handle(psoc);
	if (!tgt_hdl) {
		mlme_err("target_psoc_info is null");
		return -EINVAL;
	}

	qdf_mem_zero(&csa_status, sizeof(csa_status));
	status = wmi_extract_pdev_csa_switch_count_status(
			wmi_handle, data, &csa_status);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlme_err("Extracting CSA switch count status event failed");
		return -EINVAL;
	}

	if (csa_status.current_switch_count == 1)
		rx_ops->vdev_mgr_set_max_channel_switch_time
			(psoc, csa_status.vdev_ids, csa_status.num_vdevs);

	if (wlan_psoc_nif_fw_ext_cap_get(psoc, WLAN_SOC_CEXT_CSA_TX_OFFLOAD)) {
		for (i = 0; i < csa_status.num_vdevs; i++) {
			if (!csa_status.current_switch_count)
				target_if_vdev_csa_complete(psoc,
							csa_status.vdev_ids[i]);
		}
	}

	return target_if_csa_switch_count_status(psoc, tgt_hdl, csa_status);
}

/**
 * target_if_vdev_mgr_csa_ie_received_handler - CSA IE Received event handler
 * @scn: Pointer to scn structure
 * @data: pointer to event data
 * @datalen: event data length
 *
 * Return: 0 on success
 */
static int target_if_vdev_mgr_csa_ie_received_handler(ol_scn_t scn,
						      uint8_t *data,
						      uint32_t datalen)
{
	QDF_STATUS status;
	uint8_t vdev_id = 0;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_objmgr_vdev *vdev;
	struct wlan_objmgr_pdev *pdev;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	struct csa_offload_params csa_event = {0};

	if (!scn || !data) {
		mlme_err("Invalid input");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_csa_received) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	status = wmi_extract_csa_ie_received_event(wmi_handle, data,
						   &vdev_id, &csa_event);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlme_err("Extracting CSA IE Received event failed");
		return -EINVAL;
	}

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
						    WLAN_VDEV_TARGET_IF_ID);
	if (!vdev) {
		mlme_err("Null Vdev");
		return -EINVAL;
	}

	pdev = wlan_vdev_get_pdev(vdev);
	wlan_objmgr_vdev_release_ref(vdev, WLAN_VDEV_TARGET_IF_ID);

	if (csa_event.new_op_class &&
	    wlan_reg_is_6ghz_op_class(pdev, csa_event.new_op_class)) {
		csa_event.csa_chan_freq =
			wlan_reg_chan_band_to_freq(pdev, csa_event.channel,
						   BIT(REG_BAND_6G));
	} else {
		csa_event.csa_chan_freq =
			  wlan_reg_legacy_chan_to_freq(pdev, csa_event.channel);
	}

	return rx_ops->vdev_mgr_csa_received(psoc, vdev_id, &csa_event);
}

#ifdef WLAN_FEATURE_DYNAMIC_MAC_ADDR_UPDATE
/**
 * target_if_update_macaddr_conf_evt_handler() - Set MAC address confirmation
 *                                               event handler
 * @scn: Pointer to scn structure
 * @event_buff: event data
 * @len: length
 *
 * Response handler for set MAC address request command.
 *
 * Return: 0 for success or error code
 */
static int target_if_update_macaddr_conf_evt_handler(ol_scn_t scn,
						     uint8_t *event_buff,
						     uint32_t len)
{
	int8_t ret;
	struct wlan_objmgr_psoc *psoc;
	struct wlan_objmgr_vdev *vdev;
	struct wmi_unified *wmi_handle;
	uint8_t vdev_id, resp_status;
	QDF_STATUS status;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	struct vdev_response_timer *vdev_rsp;

	if (!event_buff) {
		mlme_err("Received NULL event ptr from FW");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	status = wmi_extract_update_mac_address_event(wmi_handle, event_buff,
						      &vdev_id, &resp_status);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlme_err("Failed to extract update MAC address event");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_set_mac_addr_response) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc, vdev_id,
						    WLAN_VDEV_TARGET_IF_ID);
	if (!vdev) {
		mlme_err("VDEV NULL");
		return -EINVAL;
	}

	if (!wlan_vdev_mlme_is_mlo_link_switch_in_progress(vdev)) {
		ret = 0;
		goto send_rsp;
	}

	/* This is for LinkSwitch request case */
	vdev_rsp = rx_ops->psoc_get_vdev_response_timer_info(psoc, vdev_id);
	if (!vdev_rsp) {
		mlme_err("vdev response timer is null VDEV_%d PSOC_%d",
			 vdev_id, wlan_psoc_get_id(psoc));
		ret = -EINVAL;
		goto out;
	}

	status =
		target_if_vdev_mgr_rsp_timer_stop(psoc, vdev_rsp,
						  UPDATE_MAC_ADDR_RESPONSE_BIT);

	ret = qdf_status_to_os_return(status);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlme_err("PSOC_%d VDEV_%d: VDE MGR RSP Timer stop failed",
			 wlan_psoc_get_id(psoc), vdev_id);
		goto out;
	}

send_rsp:
	rx_ops->vdev_mgr_set_mac_addr_response(vdev, resp_status);
out:
	wlan_objmgr_vdev_release_ref(vdev, WLAN_VDEV_TARGET_IF_ID);
	return ret;
}

static inline void
target_if_register_set_mac_addr_evt_cbk(struct wmi_unified *wmi_handle)
{
	wmi_unified_register_event_handler(
		   wmi_handle, wmi_vdev_update_mac_addr_conf_eventid,
		   target_if_update_macaddr_conf_evt_handler, VDEV_RSP_RX_CTX);
}

static inline void
target_if_unregister_set_mac_addr_evt_cbk(struct wmi_unified *wmi_handle)
{
	wmi_unified_unregister_event_handler(
			wmi_handle, wmi_vdev_update_mac_addr_conf_eventid);
}
#else
static inline void
target_if_register_set_mac_addr_evt_cbk(struct wmi_unified *wmi_handle)
{
}

static inline void
target_if_unregister_set_mac_addr_evt_cbk(struct wmi_unified *wmi_handle)
{
}
#endif

#ifdef WLAN_FEATURE_11BE_MLO
/**
 * target_if_quiet_offload_event_handler() - Quiet IE offload mlo
 *                                           station event handler
 * @scn: Pointer to scn structure
 * @event_buff: event data
 * @len: length
 *
 * Return: 0 for success or error code
 */
static int target_if_quiet_offload_event_handler(ol_scn_t scn,
						 uint8_t *event_buff,
						 uint32_t len)
{
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	QDF_STATUS status;
	struct wlan_lmac_if_mlme_rx_ops *rx_ops;
	struct vdev_sta_quiet_event sta_quiet_event = {0};

	if (!event_buff) {
		mlme_err("Received NULL event ptr from FW");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	status = wmi_extract_quiet_offload_event(wmi_handle, event_buff,
						 &sta_quiet_event);
	if (QDF_IS_STATUS_ERROR(status)) {
		mlme_err("Failed to extract quiet IE offload event");
		return -EINVAL;
	}

	rx_ops = target_if_vdev_mgr_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->vdev_mgr_quiet_offload) {
		mlme_err("No Rx Ops");
		return -EINVAL;
	}

	rx_ops->vdev_mgr_quiet_offload(psoc, &sta_quiet_event);

	return 0;
}

static inline void
target_if_register_quiet_offload_event(struct wmi_unified *wmi_handle)
{
	wmi_unified_register_event_handler(
		   wmi_handle, wmi_vdev_quiet_offload_eventid,
		   target_if_quiet_offload_event_handler, VDEV_RSP_RX_CTX);
}

static inline void
target_if_unregister_quiet_offload_event(struct wmi_unified *wmi_handle)
{
	wmi_unified_unregister_event_handler(
			wmi_handle, wmi_vdev_quiet_offload_eventid);
}
#else
static inline void
target_if_register_quiet_offload_event(struct wmi_unified *wmi_handle)
{
}

static inline void
target_if_unregister_quiet_offload_event(struct wmi_unified *wmi_handle)
{
}
#endif

QDF_STATUS target_if_vdev_mgr_wmi_event_register(
				struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS retval = QDF_STATUS_SUCCESS;
	struct wmi_unified *wmi_handle;

	if (!psoc) {
		mlme_err("PSOC is NULL");
		return QDF_STATUS_E_NULL_VALUE;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return QDF_STATUS_E_INVAL;
	}

	retval = wmi_unified_register_event_handler(
				wmi_handle,
				wmi_vdev_stopped_event_id,
				target_if_vdev_mgr_stop_response_handler,
				VDEV_RSP_RX_CTX);
	if (QDF_IS_STATUS_ERROR(retval))
		mlme_err("failed to register for stop response");

	retval = wmi_unified_register_event_handler(
				wmi_handle,
				wmi_vdev_delete_resp_event_id,
				target_if_vdev_mgr_delete_response_handler,
				VDEV_RSP_RX_CTX);
	if (QDF_IS_STATUS_ERROR(retval))
		mlme_err("failed to register for delete response");

	retval = wmi_unified_register_event_handler(
				wmi_handle,
				wmi_vdev_start_resp_event_id,
				target_if_vdev_mgr_start_response_handler,
				VDEV_RSP_RX_CTX);
	if (QDF_IS_STATUS_ERROR(retval))
		mlme_err("failed to register for start response");

	retval = wmi_unified_register_event_handler(
			wmi_handle,
			wmi_peer_delete_all_response_event_id,
			target_if_vdev_mgr_peer_delete_all_response_handler,
			VDEV_RSP_RX_CTX);
	if (QDF_IS_STATUS_ERROR(retval))
		mlme_err("failed to register for peer delete all response");

	retval = wmi_unified_register_event_handler(
			wmi_handle,
			wmi_pdev_multi_vdev_restart_response_event_id,
			target_if_vdev_mgr_multi_vdev_restart_resp_handler,
			VDEV_RSP_RX_CTX);
	if (QDF_IS_STATUS_ERROR(retval))
		mlme_err("failed to register for multivdev restart response");

	if (wmi_service_enabled(wmi_handle, wmi_service_beacon_offload)) {
		retval = wmi_unified_register_event_handler(
				wmi_handle,
				wmi_pdev_csa_switch_count_status_event_id,
				target_if_pdev_csa_status_event_handler,
				VDEV_RSP_RX_CTX);
		if (QDF_IS_STATUS_ERROR(retval))
			mlme_err("failed to register for csa event handler");
	}

	retval = wmi_unified_register_event_handler
			(wmi_handle,
			 wmi_csa_ie_received_event_id,
			 target_if_vdev_mgr_csa_ie_received_handler,
			 VDEV_RSP_RX_CTX);
	if (QDF_IS_STATUS_ERROR(retval))
		mlme_err("failed to register for CSA IE Received Event");

	target_if_register_set_mac_addr_evt_cbk(wmi_handle);

	target_if_register_quiet_offload_event(wmi_handle);

	return retval;
}

QDF_STATUS target_if_vdev_mgr_wmi_event_unregister(
					struct wlan_objmgr_psoc *psoc)
{
	struct wmi_unified *wmi_handle;

	if (!psoc) {
		mlme_err("PSOC is NULL");
		return QDF_STATUS_E_INVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return QDF_STATUS_E_INVAL;
	}

	target_if_unregister_quiet_offload_event(wmi_handle);

	target_if_unregister_set_mac_addr_evt_cbk(wmi_handle);

	wmi_unified_unregister_event_handler(wmi_handle,
					     wmi_csa_ie_received_event_id);

	wmi_unified_unregister_event_handler(
			wmi_handle,
			wmi_pdev_multi_vdev_restart_response_event_id);

	wmi_unified_unregister_event_handler(
			wmi_handle,
			wmi_peer_delete_all_response_event_id);

	wmi_unified_unregister_event_handler(wmi_handle,
					     wmi_vdev_start_resp_event_id);

	wmi_unified_unregister_event_handler(wmi_handle,
					     wmi_vdev_delete_resp_event_id);

	wmi_unified_unregister_event_handler(wmi_handle,
					     wmi_vdev_stopped_event_id);

	return QDF_STATUS_SUCCESS;
}
