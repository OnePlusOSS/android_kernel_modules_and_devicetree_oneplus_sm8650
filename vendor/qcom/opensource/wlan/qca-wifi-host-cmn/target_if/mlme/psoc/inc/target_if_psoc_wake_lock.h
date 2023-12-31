/*
 * Copyright (c) 2013-2021 The Linux Foundation. All rights reserved.
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
 * DOC: target_if_psoc_wake_lock.h
 *
 * This file provides declaration for wakelock APIs
 */

#ifndef __TARGET_IF_PSOC_WAKE_LOCK_H__
#define __TARGET_IF_PSOC_WAKE_LOCK_H__

#include <wlan_objmgr_psoc_obj.h>
#include <qdf_lock.h>

#ifdef FEATURE_VDEV_OPS_WAKELOCK
/**
 *  struct psoc_mlme_wakelock - psoc wake lock sub structure
 *  @start_wakelock: wakelock for vdev start
 *  @stop_wakelock: wakelock for vdev stop
 *  @delete_wakelock: wakelock for vdev delete
 *  @wmi_cmd_rsp_runtime_lock: run time lock
 *  @prevent_runtime_lock: run time lock
 *  @roam_sync_runtime_lock: roam sync runtime lock
 *  @is_link_up: flag to check link status
 */
struct psoc_mlme_wakelock {
	qdf_wake_lock_t start_wakelock;
	qdf_wake_lock_t stop_wakelock;
	qdf_wake_lock_t delete_wakelock;
	qdf_runtime_lock_t wmi_cmd_rsp_runtime_lock;
	qdf_runtime_lock_t prevent_runtime_lock;
	qdf_runtime_lock_t roam_sync_runtime_lock;
	bool is_link_up;
};
#endif

enum wakelock_mode {
	START_WAKELOCK,
	STOP_WAKELOCK,
	DELETE_WAKELOCK
};

#ifdef FEATURE_VDEV_OPS_WAKELOCK

/**
 * target_if_wake_lock_init() - API to initialize wakelocks:
 *                              start, stop and delete.
 * @psoc: pointer to psoc
 *
 * This also initialize the runtime lock
 *
 * Return: None
 */
void target_if_wake_lock_init(struct wlan_objmgr_psoc *psoc);

/**
 * target_if_wake_lock_deinit() - API to destroy wakelocks:
 *                                start, stop and delete.
 * @psoc: pointer to psoc
 *
 * This also destroy the runtime lock
 *
 * Return: None
 */
void target_if_wake_lock_deinit(struct wlan_objmgr_psoc *psoc);

/**
 * target_if_wake_lock_timeout_acquire() - acquire the start wakelock
 * @psoc: pointer to psoc
 * @mode: wakelock mode
 *
 * This also acquires the target_if runtime pm lock.
 *
 * Return: Success/Failure
 */
QDF_STATUS target_if_wake_lock_timeout_acquire(struct wlan_objmgr_psoc *psoc,
					       enum wakelock_mode mode);
/**
 * target_if_wake_lock_timeout_release() - release the start wakelock
 * @psoc: pointer to psoc
 * @mode: wakelock mode
 *
 * This also release the target_if runtime pm lock.
 *
 * Return: Success/Failure
 */
QDF_STATUS target_if_wake_lock_timeout_release(struct wlan_objmgr_psoc *psoc,
					       enum wakelock_mode mode);

/**
 * target_if_vdev_start_link_handler() - check for SAP mode and DFS freq
 *						to handle link up/down
 * @vdev: pointer to vdev
 * @is_restart: flag to check if it is vdev restart
 *
 * Return: None
 */
void target_if_vdev_start_link_handler(struct wlan_objmgr_vdev *vdev,
				       bool is_restart);

/**
 * target_if_vdev_stop_link_handler() - check for SAP mode to handle link
 * @vdev: pointer to vdev
 *
 * Return: None
 */
void target_if_vdev_stop_link_handler(struct wlan_objmgr_vdev *vdev);

/**
 * target_if_prevent_pm_during_roam_sync() - prevent runtime PM during roam sync
 * @psoc: pointer to psoc
 *
 * Return: None
 */
void
target_if_prevent_pm_during_roam_sync(struct wlan_objmgr_psoc *psoc);

/**
 * target_if_allow_pm_after_roam_sync() - allow runtime PM after roam
 * sync complete
 * @psoc: pointer to psoc
 *
 * Return: None
 */
void
target_if_allow_pm_after_roam_sync(struct wlan_objmgr_psoc *psoc);

#else
static inline void target_if_wake_lock_init(struct wlan_objmgr_psoc *psoc)
{
}

static inline void target_if_wake_lock_deinit(struct wlan_objmgr_psoc *psoc)
{
}

static inline QDF_STATUS target_if_wake_lock_timeout_acquire(
					struct wlan_objmgr_psoc *psoc,
					enum wakelock_mode mode)
{
	return QDF_STATUS_SUCCESS;
}

static inline QDF_STATUS target_if_wake_lock_timeout_release(
				struct wlan_objmgr_psoc *psoc,
				enum wakelock_mode mode)
{
	return QDF_STATUS_SUCCESS;
}

static inline void
target_if_vdev_start_link_handler(struct wlan_objmgr_vdev *vdev,
				  bool is_restart)
{
}

static inline void
target_if_vdev_stop_link_handler(struct wlan_objmgr_vdev *vdev)
{
}

static inline void
target_if_prevent_pm_during_roam_sync(struct wlan_objmgr_psoc *psoc)
{
}

static inline void
target_if_allow_pm_after_roam_sync(struct wlan_objmgr_psoc *psoc)
{
}
#endif
#endif
