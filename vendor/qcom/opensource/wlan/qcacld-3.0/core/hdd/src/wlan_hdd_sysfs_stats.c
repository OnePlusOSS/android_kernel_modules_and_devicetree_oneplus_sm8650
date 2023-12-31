/*
 * Copyright (c) 2011-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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

/**
 * DOC: wlan_hdd_sysfs_stats.c
 *
 * Implementation for creating sysfs files:
 *
 *   stats
 *   dump_stats
 *   clear_stats
 */

#include <wlan_hdd_includes.h>
#include "osif_vdev_sync.h"
#include <wlan_hdd_sysfs.h>
#include "wlan_hdd_sysfs_stats.h"
#include "cdp_txrx_stats.h"
#include "wlan_hdd_object_manager.h"
#include "wlan_dp_ucfg_api.h"

static int stats_id = -1;

static void hdd_sysfs_get_stats(struct hdd_adapter *adapter, ssize_t *length,
				char *buffer, size_t buf_len)
{
	struct hdd_tx_rx_stats *stats =
				&adapter->deflink->hdd_stats.tx_rx_stats;
	struct dp_tx_rx_stats *dp_stats;
	uint32_t len = 0;
	uint32_t total_rx_pkt = 0, total_rx_dropped = 0;
	uint32_t total_rx_delv = 0, total_rx_refused = 0;
	uint32_t total_tx_pkt = 0;
	uint32_t total_tx_dropped = 0;
	uint32_t total_tx_orphaned = 0;
	uint32_t total_tx_classified_ac[WLAN_MAX_AC] = {0};
	uint32_t total_tx_dropped_ac[WLAN_MAX_AC] = {0};
	int i = 0;
	uint8_t ac, rx_ol_con = 0, rx_ol_low_tput = 0;
	struct hdd_context *hdd_ctx = adapter->hdd_ctx;
	struct wlan_objmgr_vdev *vdev;

	vdev = hdd_objmgr_get_vdev_by_user(adapter->deflink, WLAN_DP_ID);
	if (!vdev)
		return;

	dp_stats = qdf_mem_malloc(sizeof(*dp_stats));
	if (!dp_stats) {
		hdd_objmgr_put_vdev_by_user(vdev, WLAN_DP_ID);
		return;
	}

	if (ucfg_dp_get_txrx_stats(vdev, dp_stats)) {
		hdd_err("Unable to get stats from DP component");
		hdd_objmgr_put_vdev_by_user(vdev, WLAN_DP_ID);
		qdf_mem_free(dp_stats);
		return;
	}
	hdd_objmgr_put_vdev_by_user(vdev, WLAN_DP_ID);

	ucfg_dp_get_disable_rx_ol_val(hdd_ctx->psoc,
				      &rx_ol_con, &rx_ol_low_tput);

	for (; i < NUM_CPUS; i++) {
		total_rx_pkt += dp_stats->per_cpu[i].rx_packets;
		total_rx_dropped += dp_stats->per_cpu[i].rx_dropped;
		total_rx_delv += dp_stats->per_cpu[i].rx_delivered;
		total_rx_refused += dp_stats->per_cpu[i].rx_refused;
		total_tx_pkt += dp_stats->per_cpu[i].tx_called;
		total_tx_dropped += dp_stats->per_cpu[i].tx_dropped;
		total_tx_orphaned += dp_stats->per_cpu[i].tx_orphaned;
		for (ac = 0; ac < WLAN_MAX_AC; ac++) {
			total_tx_classified_ac[ac] +=
					 stats->per_cpu[i].tx_classified_ac[ac];
			total_tx_dropped_ac[ac] +=
					    stats->per_cpu[i].tx_dropped_ac[ac];
		}
	}

	len = scnprintf(buffer, buf_len,
			"\nTransmit[%lu] - "
			"called %u, dropped %u orphan %u,"
			"\n[dropped]    BK %u, BE %u, VI %u, VO %u"
			"\n[classified] BK %u, BE %u, VI %u, VO %u"
			"\n\nReceive[%lu] - "
			"packets %u, dropped %u, unsolict_arp_n_mcast_drp %u, delivered %u, refused %u\n"
			"GRO - agg %u non-agg %u flush_skip %u low_tput_flush %u disabled(conc %u low-tput %u)\n",
			qdf_system_ticks(),
			total_tx_pkt,
			total_tx_dropped,
			total_tx_orphaned,
			total_tx_dropped_ac[SME_AC_BK],
			total_tx_dropped_ac[SME_AC_BE],
			total_tx_dropped_ac[SME_AC_VI],
			total_tx_dropped_ac[SME_AC_VO],
			total_tx_classified_ac[SME_AC_BK],
			total_tx_classified_ac[SME_AC_BE],
			total_tx_classified_ac[SME_AC_VI],
			total_tx_classified_ac[SME_AC_VO],
			qdf_system_ticks(),
			total_rx_pkt, total_rx_dropped,
			qdf_atomic_read(&dp_stats->rx_usolict_arp_n_mcast_drp),
			total_rx_delv,
			total_rx_refused,
			dp_stats->rx_aggregated, dp_stats->rx_non_aggregated,
			dp_stats->rx_gro_flush_skip,
			dp_stats->rx_gro_low_tput_flush,
			rx_ol_con,
			rx_ol_low_tput);

	for (i = 0; i < NUM_CPUS; i++) {
		if (dp_stats->per_cpu[i].rx_packets == 0)
			continue;
		len += scnprintf(buffer + len, buf_len - len,
				 "Rx CPU[%d]:"
				 "packets %u, dropped %u, delivered %u, refused %u\n",
				 i, dp_stats->per_cpu[i].rx_packets,
				 dp_stats->per_cpu[i].rx_dropped,
				 dp_stats->per_cpu[i].rx_delivered,
				 dp_stats->per_cpu[i].rx_refused);
	}

	len += scnprintf(buffer + len, buf_len - len,
		"\nTX_FLOW"
		"\nCurrent status: %s"
		"\ntx-flow timer start count %u"
		"\npause count %u, unpause count %u",
		(stats->is_txflow_paused == true ? "PAUSED" : "UNPAUSED"),
		stats->txflow_timer_cnt,
		stats->txflow_pause_cnt,
		stats->txflow_unpause_cnt);

	len += cdp_stats(cds_get_context(QDF_MODULE_ID_SOC),
			 adapter->deflink->vdev_id, &buffer[len],
			 (buf_len - len));
	*length = len + 1;
	qdf_mem_free(dp_stats);
}

static ssize_t
__hdd_sysfs_stats_show(struct net_device *net_dev, char *buf)
{
	struct hdd_adapter *adapter = netdev_priv(net_dev);
	struct hdd_context *hdd_ctx;
	int ret;
	ssize_t length = 0;

	if (hdd_validate_adapter(adapter))
		return -EINVAL;

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	ret = wlan_hdd_validate_context(hdd_ctx);
	if (ret)
		return ret;

	if (!wlan_hdd_validate_modules_state(hdd_ctx))
		return -EINVAL;

	hdd_sysfs_get_stats(adapter, &length, buf, PAGE_SIZE);

	return length;
}

static ssize_t
hdd_sysfs_stats_show(struct device *dev,
		     struct device_attribute *attr,
		     char *buf)
{
	struct net_device *net_dev = container_of(dev, struct net_device, dev);
	struct osif_vdev_sync *vdev_sync;
	ssize_t err_size;

	err_size = osif_vdev_sync_op_start(net_dev, &vdev_sync);
	if (err_size)
		return err_size;

	err_size = __hdd_sysfs_stats_show(net_dev, buf);

	osif_vdev_sync_op_stop(vdev_sync);

	return err_size;
}

static DEVICE_ATTR(stats, 0440,
		   hdd_sysfs_stats_show, NULL);

static ssize_t
__wlan_hdd_sysfs_dump_stats_store(struct net_device *net_dev, char const *buf,
				  size_t count)
{
	struct hdd_adapter *adapter = netdev_priv(net_dev);
	struct hdd_context *hdd_ctx;
	char buf_local[MAX_SYSFS_USER_COMMAND_SIZE_LENGTH + 1];
	char *sptr, *token;
	int value, ret;

	if (hdd_validate_adapter(adapter))
		return -EINVAL;

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	ret = wlan_hdd_validate_context(hdd_ctx);
	if (ret != 0)
		return ret;

	if (!wlan_hdd_validate_modules_state(hdd_ctx))
		return -EINVAL;

	ret = hdd_sysfs_validate_and_copy_buf(buf_local, sizeof(buf_local),
					      buf, count);
	if (ret) {
		hdd_err_rl("invalid input");
		return ret;
	}

	sptr = buf_local;
	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 0, &value))
		return -EINVAL;

	hdd_debug("dump_stats %d", value);

	stats_id = value;

	return count;
}

static ssize_t wlan_hdd_sysfs_dump_stats_store(struct device *dev,
					       struct device_attribute *attr,
					       char const *buf, size_t count)
{
	struct net_device *net_dev = container_of(dev, struct net_device, dev);
	struct osif_vdev_sync *vdev_sync;
	ssize_t err_size;

	err_size = osif_vdev_sync_op_start(net_dev, &vdev_sync);
	if (err_size)
		return err_size;

	err_size = __wlan_hdd_sysfs_dump_stats_store(net_dev, buf, count);

	osif_vdev_sync_op_stop(vdev_sync);

	return err_size;
}

static ssize_t
__wlan_hdd_sysfs_dump_stats_show(struct net_device *net_dev, char *buf)
{
	struct hdd_adapter *adapter = netdev_priv(net_dev);
	struct hdd_context *hdd_ctx;
	int ret;

	if (hdd_validate_adapter(adapter))
		return -EINVAL;

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	ret = wlan_hdd_validate_context(hdd_ctx);
	if (ret != 0)
		return ret;

	if (!wlan_hdd_validate_modules_state(hdd_ctx))
		return -EINVAL;

	ret = hdd_wlan_dump_stats(adapter, stats_id);

	return ret;
}

static ssize_t wlan_hdd_sysfs_dump_stats_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct net_device *net_dev = container_of(dev, struct net_device, dev);
	struct osif_vdev_sync *vdev_sync;
	ssize_t err_size;

	err_size = osif_vdev_sync_op_start(net_dev, &vdev_sync);
	if (err_size)
		return err_size;

	err_size = __wlan_hdd_sysfs_dump_stats_show(net_dev, buf);

	osif_vdev_sync_op_stop(vdev_sync);

	return err_size;
}

static DEVICE_ATTR(dump_stats, 0660, wlan_hdd_sysfs_dump_stats_show,
		   wlan_hdd_sysfs_dump_stats_store);

static ssize_t
__wlan_hdd_sysfs_clear_stats_store(struct net_device *net_dev, char const *buf,
				   size_t count)
{
	struct hdd_adapter *adapter = netdev_priv(net_dev);
	struct hdd_context *hdd_ctx;
	char buf_local[MAX_SYSFS_USER_COMMAND_SIZE_LENGTH + 1];
	char *sptr, *token;
	int value, ret;

	if (hdd_validate_adapter(adapter))
		return -EINVAL;

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	ret = wlan_hdd_validate_context(hdd_ctx);
	if (ret != 0)
		return ret;

	if (!wlan_hdd_validate_modules_state(hdd_ctx))
		return -EINVAL;

	ret = hdd_sysfs_validate_and_copy_buf(buf_local, sizeof(buf_local),
					      buf, count);
	if (ret) {
		hdd_err_rl("invalid input");
		return ret;
	}

	sptr = buf_local;
	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 0, &value))
		return -EINVAL;

	hdd_debug("clear_stats %d", value);

	ret = hdd_wlan_clear_stats(adapter, value);
	if (ret) {
		hdd_err_rl("Failed to clear stats");
		return ret;
	}

	return count;
}

static ssize_t wlan_hdd_sysfs_clear_stats_store(struct device *dev,
						struct device_attribute *attr,
						char const *buf, size_t count)
{
	struct net_device *net_dev = container_of(dev, struct net_device, dev);
	struct osif_vdev_sync *vdev_sync;
	ssize_t err_size;

	err_size = osif_vdev_sync_op_start(net_dev, &vdev_sync);
	if (err_size)
		return err_size;

	err_size = __wlan_hdd_sysfs_clear_stats_store(net_dev, buf, count);

	osif_vdev_sync_op_stop(vdev_sync);

	return err_size;
}

static DEVICE_ATTR(clear_stats, 0220, NULL,
		   wlan_hdd_sysfs_clear_stats_store);

int hdd_sysfs_stats_create(struct hdd_adapter *adapter)
{
	int error;

	error = device_create_file(&adapter->dev->dev, &dev_attr_dump_stats);
	if (error)
		hdd_err("could not create dump_stats sysfs file");

	error = device_create_file(&adapter->dev->dev, &dev_attr_clear_stats);
	if (error)
		hdd_err("could not create clear_stats sysfs file");

	error = device_create_file(&adapter->dev->dev,  &dev_attr_stats);
	if (error)
		hdd_err("could not create stats sysfs file");

	return error;
}

void hdd_sysfs_stats_destroy(struct hdd_adapter *adapter)
{
	device_remove_file(&adapter->dev->dev, &dev_attr_dump_stats);
	device_remove_file(&adapter->dev->dev, &dev_attr_clear_stats);
	device_remove_file(&adapter->dev->dev, &dev_attr_stats);
}
