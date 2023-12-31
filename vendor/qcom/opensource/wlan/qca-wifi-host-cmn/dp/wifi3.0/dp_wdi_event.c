/*
 * Copyright (c) 2017-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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


#include "dp_internal.h"
#include "qdf_mem.h"   /* qdf_mem_malloc,free */
#ifdef WIFI_MONITOR_SUPPORT
#include "dp_htt.h"
#include <dp_mon.h>
#endif
#include <qdf_module.h>

#ifdef WDI_EVENT_ENABLE
/**
 * dp_wdi_event_next_sub() - Return handle for Next WDI event
 * @wdi_sub: WDI Event handle
 *
 * Return handle for next WDI event in list
 *
 * Return: Next WDI event to be subscribe
 */
static inline wdi_event_subscribe *
dp_wdi_event_next_sub(wdi_event_subscribe *wdi_sub)
{
	if (!wdi_sub) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			"Invalid subscriber in %s", __func__);
		return NULL;
	}
	return wdi_sub->priv.next;
}


/**
 * dp_wdi_event_del_subs() - Delete Event subscription
 * @wdi_sub: WDI Event handle
 * @event_index: Event index from list
 *
 * This API will delete subscribed event from list
 *
 * Return: None
 */
static inline void
dp_wdi_event_del_subs(wdi_event_subscribe *wdi_sub, int event_index)
{
	/* Subscribers should take care of deletion */
}


/**
 * dp_wdi_event_iter_sub() - Iterate through all WDI event in the list
 * and pass WDI event to callback function
 * @pdev: DP pdev handle
 * @event_index: Event index in list
 * @wdi_sub: WDI event subscriber
 * @data: pointer to data
 * @peer_id: peer id number
 * @status: HTT rx status
 *
 *
 * Return: None
 */
static inline void
dp_wdi_event_iter_sub(
	struct dp_pdev *pdev,
	uint32_t event_index,
	wdi_event_subscribe *wdi_sub,
	void *data,
	uint16_t peer_id,
	int status)
{
	enum WDI_EVENT event = event_index + WDI_EVENT_BASE;

	if (wdi_sub) {
		do {
			wdi_sub->callback(wdi_sub->context, event, data,
					peer_id, status);
		} while ((wdi_sub = dp_wdi_event_next_sub(wdi_sub)));
	}
}


void
dp_wdi_event_handler(
	enum WDI_EVENT event,
	struct dp_soc *soc,
	void *data,
	uint16_t peer_id,
	int status, uint8_t pdev_id)
{
	uint32_t event_index;
	wdi_event_subscribe *wdi_sub;
	struct dp_pdev *txrx_pdev;
	struct dp_soc *soc_t = (struct dp_soc *)soc;
	txrx_pdev = dp_get_pdev_for_mac_id(soc_t, pdev_id);

	if (!event) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			"Invalid WDI event in %s", __func__);
		return;
	}
	if (!txrx_pdev || txrx_pdev->pdev_deinit) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			"Invalid pdev in WDI event handler");
		return;
	}

	/*
	 *  There can be NULL data, so no validation for the data
	 *  Subscribers must do the sanity based on the requirements
	 */
	event_index = event - WDI_EVENT_BASE;

	DP_STATS_INC(txrx_pdev, wdi_event[event_index], 1);
	wdi_sub = txrx_pdev->wdi_event_list[event_index];

	/* Find the subscriber */
	dp_wdi_event_iter_sub(txrx_pdev, event_index, wdi_sub, data,
			peer_id, status);
}

qdf_export_symbol(dp_wdi_event_handler);

int
dp_wdi_event_sub(
	struct cdp_soc_t *soc, uint8_t pdev_id,
	wdi_event_subscribe *event_cb_sub_handle,
	uint32_t event)
{
	uint32_t event_index;
	wdi_event_subscribe *wdi_sub;
	wdi_event_subscribe *wdi_sub_itr;
	struct dp_pdev *txrx_pdev =
		dp_get_pdev_from_soc_pdev_id_wifi3((struct dp_soc *)soc,
						   pdev_id);
	wdi_event_subscribe *event_cb_sub =
		(wdi_event_subscribe *) event_cb_sub_handle;

	if (!txrx_pdev) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			"Invalid txrx_pdev in %s", __func__);
		return -EINVAL;
	}
	if (!event_cb_sub) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			"Invalid callback in %s", __func__);
		return -EINVAL;
	}
	if ((!event) || (event >= WDI_EVENT_LAST) || (event < WDI_EVENT_BASE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			"Invalid event in %s", __func__);
		return -EINVAL;
	}

	dp_monitor_set_pktlog_wifi3(txrx_pdev, event, true);
	event_index = event - WDI_EVENT_BASE;
	wdi_sub = txrx_pdev->wdi_event_list[event_index];

	/*
	 *  Check if it is the first subscriber of the event
	 */
	if (!wdi_sub) {
		wdi_sub = event_cb_sub;
		wdi_sub->priv.next = NULL;
		wdi_sub->priv.prev = NULL;
		txrx_pdev->wdi_event_list[event_index] = wdi_sub;
		return 0;
	}

	/* Check if event is already subscribed */
	wdi_sub_itr = wdi_sub;
	do {
		if (wdi_sub_itr == event_cb_sub) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO,
				  "Duplicate wdi subscribe event detected %s", __func__);
			return 0;
		}
	} while ((wdi_sub_itr = dp_wdi_event_next_sub(wdi_sub_itr)));

	event_cb_sub->priv.next = wdi_sub;
	event_cb_sub->priv.prev = NULL;
	wdi_sub->priv.prev = event_cb_sub;
	txrx_pdev->wdi_event_list[event_index] = event_cb_sub;
	return 0;

}

int
dp_wdi_event_unsub(
	struct cdp_soc_t *soc, uint8_t pdev_id,
	wdi_event_subscribe *event_cb_sub_handle,
	uint32_t event)
{
	uint32_t event_index = event - WDI_EVENT_BASE;
	struct dp_pdev *txrx_pdev =
		dp_get_pdev_from_soc_pdev_id_wifi3((struct dp_soc *)soc,
						   pdev_id);
	wdi_event_subscribe *event_cb_sub =
		(wdi_event_subscribe *) event_cb_sub_handle;

	if (!txrx_pdev || !event_cb_sub) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			"Invalid callback or pdev in %s", __func__);
		return -EINVAL;
	}

	dp_monitor_set_pktlog_wifi3(txrx_pdev, event, false);

	if (!event_cb_sub->priv.prev) {
		txrx_pdev->wdi_event_list[event_index] = event_cb_sub->priv.next;
	} else {
		event_cb_sub->priv.prev->priv.next = event_cb_sub->priv.next;
	}
	if (event_cb_sub->priv.next) {
		event_cb_sub->priv.next->priv.prev = event_cb_sub->priv.prev;
	}

	/* Reset susbscribe event list elems */
	event_cb_sub->priv.next = NULL;
	event_cb_sub->priv.prev = NULL;

	return 0;
}


int
dp_wdi_event_attach(struct dp_pdev *txrx_pdev)
{
	if (!txrx_pdev) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			"Invalid device in %s\nWDI event attach failed",
			__func__);
		return -EINVAL;
	}
	/* Separate subscriber list for each event */
	txrx_pdev->wdi_event_list = (wdi_event_subscribe **)
		qdf_mem_malloc(
			sizeof(wdi_event_subscribe *) * WDI_NUM_EVENTS);
	if (!txrx_pdev->wdi_event_list) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			"Insufficient memory for the WDI event lists");
		return -EINVAL;
	}
	return 0;
}

int
dp_wdi_event_detach(struct dp_pdev *txrx_pdev)
{
	int i;
	wdi_event_subscribe *wdi_sub;
	if (!txrx_pdev) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			"Invalid device in %s\nWDI attach failed", __func__);
		return -EINVAL;
	}
	if (!txrx_pdev->wdi_event_list) {
		return -EINVAL;
	}
	for (i = 0; i < WDI_NUM_EVENTS; i++) {
		wdi_sub = txrx_pdev->wdi_event_list[i];
		/* Delete all the subscribers */
		dp_wdi_event_del_subs(wdi_sub, i);
	}
	qdf_mem_free(txrx_pdev->wdi_event_list);
	return 0;
}
#endif
