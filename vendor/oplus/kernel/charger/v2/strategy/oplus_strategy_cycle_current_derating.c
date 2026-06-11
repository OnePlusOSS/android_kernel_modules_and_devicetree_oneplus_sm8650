// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2026 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[STRATEGY_CCD]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <oplus_chg.h>
#include <oplus_chg_monitor.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_strategy.h>

#ifndef CYCLE_CURR_DERATING_CC_THR_MAX
#define CYCLE_CURR_DERATING_CC_THR_MAX	6
#endif

struct ccd_factor {
	u32 threshold_ibus;
	u32 derated_ratio;
};

struct ccd_strategy {
	struct oplus_chg_strategy strategy;
	struct device_node *node;
	struct dentry *dbg_dentry;
	char *dbg_name;

	u32 cycle_thr[CYCLE_CURR_DERATING_CC_THR_MAX];
	u32 cycle_thr_cnt;
	u32 temp_region_cnt;

	u32 head_factor_cnt;
	struct ccd_factor *head_factor;

	u32 non_head_factor_cnt;
	struct ccd_factor *non_head_factor;

	u32 temp_region;
	u32 protocol_temp_region_cnt;
	int curve_ibus;

	bool last_valid;
	int last_curve_ibus;
	int last_derated;
};

struct ccd_track_info {
	int curve_ibus;
	int threshold_ibus;
	int derated_ratio;
	int derated_ibus;
};

static struct ccd_track_info ccd_track_info;
static struct dentry *ccd_debugfs_dir;
static struct oplus_mms *ccd_gauge_topic;
static void ccd_track_workfn(struct work_struct *work);
static DECLARE_WORK(ccd_track_work, ccd_track_workfn);

static struct device_node *oplus_chg_get_common_charge_spec_node(void)
{
	static struct device_node *cached_comm;

	if (!cached_comm)
		cached_comm = of_find_compatible_node(NULL, NULL, "oplus,common-charge");

	return cached_comm;
}

static void ccd_track_workfn(struct work_struct *work)
{
	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	int rc;

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic)
		return;

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
		ERR_ITEM_CYCLE_CURRENT_DERATING,
		"$$cycle_curr_derating_trig$$curve_ibus@@%d$$threshold_ibus@@%d"
		"$$derated_ratio@@%d$$derated_ibus@@%d",
		ccd_track_info.curve_ibus, ccd_track_info.threshold_ibus,
		ccd_track_info.derated_ratio, ccd_track_info.derated_ibus);
	if (!msg)
		return;

	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0)
		kfree(msg);
}

static void ccd_track_try_trigger(int curve_ibus, int thr, int ratio, int derated)
{
	ccd_track_info.curve_ibus = curve_ibus;
	ccd_track_info.threshold_ibus = thr;
	ccd_track_info.derated_ratio = ratio;
	ccd_track_info.derated_ibus = derated;
	schedule_work(&ccd_track_work);
}

static void ccd_dump_factor_table(struct seq_file *m, const struct ccd_strategy *ccd,
				  const struct ccd_factor *factor, u32 factor_cnt,
				  const char *title, u32 cc_max, u32 r_max)
{
	u32 cc_i_loop;
	u32 r;
	u64 pos;

	if (!factor || !factor_cnt)
		return;
	seq_printf(m, "%s(thr,ratio):\n", title);
	for (cc_i_loop = 0; cc_i_loop < cc_max; cc_i_loop++) {
		seq_printf(m, "  cc_idx%u:", cc_i_loop);
		for (r = 0; r < r_max; r++) {
			pos = (u64)cc_i_loop * ccd->temp_region_cnt + r;
			if (pos >= factor_cnt)
				break;
			seq_printf(m, " (%u,%u)",
				factor[pos].threshold_ibus, factor[pos].derated_ratio);
		}
		seq_printf(m, "\n");
	}
}

static void ccd_dump_one_to_seq(struct seq_file *m, struct ccd_strategy *ccd)
{
	u32 cc_i;
	u32 cc_max;
	u32 r_max;
	const char *name;

	if (!ccd)
		return;
	name = ccd->dbg_name ? ccd->dbg_name : "ccd";
	cc_max = ccd->cycle_thr_cnt <= CYCLE_CURR_DERATING_CC_THR_MAX ?
		 ccd->cycle_thr_cnt : CYCLE_CURR_DERATING_CC_THR_MAX;
	r_max = ccd->temp_region_cnt;
	seq_printf(m, "--- %s ---\n", name);
	seq_printf(m, "cycle_thr_cnt=%u temp_region_cnt=%u protocol_temp_region_cnt=%u %s\n",
		ccd->cycle_thr_cnt, ccd->temp_region_cnt, ccd->protocol_temp_region_cnt,
		(ccd->temp_region_cnt == ccd->protocol_temp_region_cnt) ?
		"params_match" : "params_mismatch");
	seq_printf(m, "head_cnt=%u non_head_cnt=%u\n",
		ccd->head_factor_cnt, ccd->non_head_factor_cnt);
	seq_printf(m, "cycle_thr: ");
	for (cc_i = 0; cc_i < cc_max; cc_i++)
		seq_printf(m, "%u ", ccd->cycle_thr[cc_i]);
	seq_printf(m, "\n");
	ccd_dump_factor_table(m, ccd, ccd->head_factor, ccd->head_factor_cnt,
			      "head_factor", cc_max, r_max);
	ccd_dump_factor_table(m, ccd, ccd->non_head_factor, ccd->non_head_factor_cnt,
			      "non_head_factor", cc_max, r_max);
}

static int ccd_dump_show(struct seq_file *m, void *v)
{
	struct ccd_strategy *ccd = m->private;

	if (!ccd)
		return 0;
	ccd_dump_one_to_seq(m, ccd);
	return 0;
}

static int ccd_dump_open(struct inode *inode, struct file *file)
{
	void *data;

	if (!inode)
		return -EINVAL;
	data = inode->i_private;
	return single_open(file, ccd_dump_show, data);
}

static const struct file_operations ccd_dump_fops = {
	.owner   = THIS_MODULE,
	.open    = ccd_dump_open,
	.read    = seq_read,
	.release = single_release,
};

static void ccd_debugfs_init(void)
{
	if (ccd_debugfs_dir)
		return;
	ccd_debugfs_dir = debugfs_create_dir("ccd", NULL);
	if (IS_ERR(ccd_debugfs_dir))
		ccd_debugfs_dir = debugfs_lookup("ccd", NULL);
	if (IS_ERR_OR_NULL(ccd_debugfs_dir))
		ccd_debugfs_dir = NULL;
}

static void ccd_strategy_free(struct ccd_strategy *ccd)
{
	if (!ccd)
		return;
	if (ccd->node)
		of_node_put(ccd->node);
	if (ccd->dbg_name)
		kfree(ccd->dbg_name);
	kfree(ccd->head_factor);
	kfree(ccd->non_head_factor);
	kfree(ccd);
}

static int ccd_parse_cycle_thr(struct ccd_strategy *ccd, struct device_node *node)
{
	int rc;

	rc = of_property_count_elems_of_size(
		node, "oplus_spec,cycle-derating-cc-thr", sizeof(u32));
	if (rc <= 0)
		return -ENODEV;
	if (rc > CYCLE_CURR_DERATING_CC_THR_MAX)
		return -EINVAL;

	ccd->cycle_thr_cnt = rc;
	rc = of_property_read_u32_array(
		node, "oplus_spec,cycle-derating-cc-thr",
		ccd->cycle_thr, ccd->cycle_thr_cnt);
	if (rc < 0)
		return -EINVAL;

	return 0;
}

static struct ccd_factor *ccd_read_thr_ratio_arrays(struct device_node *node,
		const char *thr_prop, const char *ratio_prop, u32 elem_cnt)
{
	u32 *thr;
	u32 *ratio;
	struct ccd_factor *factor;
	u32 i;
	int rc;

	thr = kcalloc(elem_cnt, sizeof(*thr), GFP_KERNEL);
	ratio = kcalloc(elem_cnt, sizeof(*ratio), GFP_KERNEL);
	factor = kcalloc(elem_cnt, sizeof(*factor), GFP_KERNEL);
	if (!thr || !ratio || !factor)
		goto err;

	rc = of_property_read_u32_array(node, thr_prop, thr, elem_cnt);
	if (rc < 0)
		goto err;
	rc = of_property_read_u32_array(node, ratio_prop, ratio, elem_cnt);
	if (rc < 0)
		goto err;

	for (i = 0; i < elem_cnt; i++) {
		factor[i].threshold_ibus = thr[i];
		factor[i].derated_ratio = ratio[i];
	}
	kfree(thr);
	kfree(ratio);
	return factor;

err:
	kfree(thr);
	kfree(ratio);
	kfree(factor);
	return NULL;
}

static int ccd_parse_factor_split(struct device_node *node, const char *thr_prop,
				 const char *ratio_prop, struct ccd_factor **out,
				 u32 *out_cnt)
{
	int cnt;

	if (!node || !thr_prop || !ratio_prop || !out || !out_cnt)
		return -EINVAL;

	cnt = of_property_count_elems_of_size(node, thr_prop, sizeof(u32));
	if (cnt <= 0)
		return -EINVAL;
	if (cnt != of_property_count_elems_of_size(node, ratio_prop, sizeof(u32)))
		return -EINVAL;

	*out = ccd_read_thr_ratio_arrays(node, thr_prop, ratio_prop, cnt);
	if (!*out)
		return -EINVAL;
	*out_cnt = cnt;
	return 0;
}

static int ccd_parse_factors(struct ccd_strategy *ccd, struct device_node *node)
{
	int rc;

	rc = ccd_parse_factor_split(
		node,
		"oplus_spec,cycle-derating-head-threshold-ibus",
		"oplus_spec,cycle-derating-head-derated-ratio",
		&ccd->head_factor, &ccd->head_factor_cnt);
	if (rc < 0)
		return rc;

	rc = ccd_parse_factor_split(
		node,
		"oplus_spec,cycle-derating-non-head-threshold-ibus",
		"oplus_spec,cycle-derating-non-head-derated-ratio",
		&ccd->non_head_factor, &ccd->non_head_factor_cnt);
	if (rc < 0) {
		ccd->non_head_factor = NULL;
		ccd->non_head_factor_cnt = 0;
		rc = 0;
	}

	if (!ccd->head_factor_cnt || !ccd->cycle_thr_cnt)
		return -EINVAL;
	if (ccd->head_factor_cnt % ccd->cycle_thr_cnt)
		return -EINVAL;

	ccd->temp_region_cnt = ccd->head_factor_cnt / ccd->cycle_thr_cnt;
	if (!ccd->temp_region_cnt)
		return -EINVAL;

	if (ccd->non_head_factor &&
	    ccd->non_head_factor_cnt != ccd->head_factor_cnt)
		return -EINVAL;

	return 0;
}

static int ccd_get_cycle_index(const struct ccd_strategy *ccd, int batt_cc, bool *skip_derate)
{
	int i;
	bool skip;

	skip = !ccd || ccd->cycle_thr_cnt == 0 || batt_cc <= 0 || batt_cc < ccd->cycle_thr[0];
	if (skip_derate)
		*skip_derate = skip;
	if (skip)
		return 0;

	for (i = ccd->cycle_thr_cnt - 1; i >= 0; i--) {
		if (batt_cc > ccd->cycle_thr[i])
			return i;
	}
	return 0;
}

static int ccd_apply_table(const struct ccd_factor *factor, u32 factor_cnt, u32 pos, int curve_ibus)
{
	int thr;
	int ratio;
	int processed;
	int derated;

	if (!factor || factor_cnt == 0)
		return curve_ibus;
	if (curve_ibus <= 0 || pos >= factor_cnt)
		return curve_ibus;

	thr = factor[pos].threshold_ibus;
	ratio = factor[pos].derated_ratio;
	if (thr <= 0 || ratio <= 0 || ratio >= 100)
		return curve_ibus;
	if (curve_ibus <= thr)
		return curve_ibus;

	processed = curve_ibus * ratio / 100;
	derated = processed > thr ? processed : thr;
	ccd_track_try_trigger(curve_ibus, thr, ratio, derated);
	return derated;
}

static void ccd_log_derating(const char *tag, int batt_cc, u32 temp_region,
			     int curve_ibus, const struct ccd_factor *factor,
			     u32 factor_cnt, u32 pos, int derated)
{
	const char *log_tag = tag ? tag : "ccd";

	if (!factor || pos >= factor_cnt)
		return;

	chg_info("%s:cc=%d temp_region=%u curve_ibus=%d threshold_ibus=%u ratio=%u derated_ibus=%d\n",
		log_tag, batt_cc, temp_region, curve_ibus,
		factor[pos].threshold_ibus, factor[pos].derated_ratio, derated);
}

static int ccd_apply_derating_table(struct ccd_strategy *ccd,
			     const struct ccd_factor *factor, u32 factor_cnt, const char *tag,
			     int curve_ibus, int batt_cc, u32 pos)
{
	int derated = ccd_apply_table(factor, factor_cnt, pos, curve_ibus);

	if (derated != curve_ibus)
		ccd_log_derating(tag, batt_cc, ccd->temp_region, curve_ibus, factor, factor_cnt, pos, derated);
	return derated;
}

static struct oplus_chg_strategy *ccd_strategy_alloc(unsigned char *buf, size_t size)
{
	return ERR_PTR(-ENOTSUPP);
}

static struct oplus_chg_strategy *ccd_strategy_alloc_by_node(struct device_node *node)
{
	struct ccd_strategy *ccd;
	struct device_node *common_node;
	int rc;

	common_node = oplus_chg_get_common_charge_spec_node();
	if (!common_node)
		return ERR_PTR(-ENODEV);

	ccd = kzalloc(sizeof(*ccd), GFP_KERNEL);
	if (!ccd)
		return ERR_PTR(-ENOMEM);

	ccd->node = node ? of_node_get(node) : NULL;
	rc = ccd_parse_cycle_thr(ccd, common_node);
	if (rc < 0)
		goto err;
	rc = ccd_parse_factors(ccd, common_node);
	if (rc < 0)
		goto err;

	ccd->dbg_name = (node && node->name) ? kstrdup(node->name, GFP_KERNEL) : NULL;
	ccd->temp_region = 0;
	ccd->curve_ibus = 0;
	ccd->last_valid = false;
	ccd->dbg_dentry = NULL;

	chg_info("cycle_thr_cnt=%u temp_region_cnt=%u head_cnt=%u non_head_cnt=%u\n",
		ccd->cycle_thr_cnt, ccd->temp_region_cnt, ccd->head_factor_cnt,
		ccd->non_head_factor_cnt);

	if (ccd_debugfs_dir && ccd->dbg_name) {
		ccd->dbg_dentry = debugfs_create_file(ccd->dbg_name, 0444,
			ccd_debugfs_dir, ccd, &ccd_dump_fops);
		if (IS_ERR_OR_NULL(ccd->dbg_dentry))
			ccd->dbg_dentry = NULL;
	}
	return &ccd->strategy;

err:
	ccd_strategy_free(ccd);
	return ERR_PTR(rc);
}

static int ccd_strategy_release(struct oplus_chg_strategy *strategy)
{
	struct ccd_strategy *ccd;

	if (!strategy)
		return -EINVAL;

	ccd = container_of(strategy, struct ccd_strategy, strategy);
	if (ccd->dbg_dentry) {
		debugfs_remove(ccd->dbg_dentry);
		ccd->dbg_dentry = NULL;
	}
	ccd_strategy_free(ccd);
	return 0;
}

static int ccd_strategy_init(struct oplus_chg_strategy *strategy)
{
	struct ccd_strategy *ccd;

	if (!strategy)
		return -EINVAL;

	ccd = container_of(strategy, struct ccd_strategy, strategy);
	ccd->temp_region = 0;
	ccd->curve_ibus = 0;
	ccd->last_valid = false;
	return 0;
}

static int ccd_strategy_set_process_data(struct oplus_chg_strategy *strategy, const char *type, unsigned long arg)
{
	struct ccd_strategy *ccd;

	if (!strategy || !type)
		return -EINVAL;

	ccd = container_of(strategy, struct ccd_strategy, strategy);
	if (!strncmp(type, "temp_region", sizeof("temp_region"))) {
		ccd->temp_region = arg;
		return 0;
	}
	if (!strncmp(type, "temp_region_cnt", sizeof("temp_region_cnt"))) {
		ccd->protocol_temp_region_cnt = arg;
		return 0;
	}
	if (!strncmp(type, "curve_ibus", sizeof("curve_ibus"))) {
		ccd->curve_ibus = arg;
		return 0;
	}
	return -EINVAL;
}

static bool ccd_get_data_early_exit(struct ccd_strategy *ccd, int curve_ibus, int *result)
{
	if (curve_ibus <= 0) {
		*result = 0;
		return true;
	}
	if (!ccd->protocol_temp_region_cnt ||
	    ccd->temp_region >= ccd->protocol_temp_region_cnt ||
	    ccd->protocol_temp_region_cnt != ccd->temp_region_cnt) {
		*result = curve_ibus;
		return true;
	}
	if (ccd->last_valid && ccd->last_curve_ibus == curve_ibus) {
		*result = ccd->last_derated;
		return true;
	}
	return false;
}

static int ccd_apply_derating(struct ccd_strategy *ccd, int curve_ibus, int batt_cc, u32 pos)
{
	int thr;

	if (!ccd->head_factor || !ccd->head_factor_cnt || pos >= ccd->head_factor_cnt)
		return curve_ibus;
	thr = ccd->head_factor[pos].threshold_ibus;
	if (thr <= 0)
		return curve_ibus;

	if (curve_ibus > thr)
		return ccd_apply_derating_table(ccd, ccd->head_factor, ccd->head_factor_cnt,
					  "ccd_head", curve_ibus, batt_cc, pos);
	if (curve_ibus == thr)
		return curve_ibus;
	if (!ccd->non_head_factor || ccd->non_head_factor_cnt != ccd->head_factor_cnt)
		return curve_ibus;
	return ccd_apply_derating_table(ccd, ccd->non_head_factor, ccd->non_head_factor_cnt,
				  "ccd_non_head", curve_ibus, batt_cc, pos);
}

static int ccd_strategy_get_data(struct oplus_chg_strategy *strategy, void *ret)
{
	struct ccd_strategy *ccd;
	int batt_cc;
	int idx;
	int curve_ibus;
	int derated;
	bool skip_derate;
	u32 pos;
	int *result = ret;

	if (!strategy || !ret)
		return -EINVAL;

	ccd = container_of(strategy, struct ccd_strategy, strategy);
	curve_ibus = ccd->curve_ibus;
	if (ccd_get_data_early_exit(ccd, curve_ibus, result))
		return 0;

	if (!ccd_gauge_topic)
		ccd_gauge_topic = oplus_mms_get_by_name("gauge");
	batt_cc = ccd_gauge_topic ? oplus_gauge_get_dec_cv_soh(ccd_gauge_topic) : 0;
	idx = ccd_get_cycle_index(ccd, batt_cc, &skip_derate);
	if (skip_derate) {
		*result = curve_ibus;
		return 0;
	}
	pos = ccd->temp_region_cnt * idx + ccd->temp_region;
	derated = ccd_apply_derating(ccd, curve_ibus, batt_cc, pos);
	*result = derated;
	ccd->last_valid = true;
	ccd->last_curve_ibus = curve_ibus;
	ccd->last_derated = derated;
	return 0;
}

static struct oplus_chg_strategy_desc ccd_strategy_desc = {
	.name = "cycle_current_derating",
	.strategy_alloc = ccd_strategy_alloc,
	.strategy_alloc_by_node = ccd_strategy_alloc_by_node,
	.strategy_release = ccd_strategy_release,
	.strategy_init = ccd_strategy_init,
	.strategy_get_data = ccd_strategy_get_data,
	.strategy_set_process_data = ccd_strategy_set_process_data,
};

int ccd_strategy_register(void)
{
	int rc;

	rc = oplus_chg_strategy_register(&ccd_strategy_desc);
	if (rc)
		return rc;
	ccd_debugfs_init();
	return 0;
}
