/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2024 Oplus. All rights reserved.
 * this referenced by android cutil/trace.h
 */
#ifndef _OSVELTE_MM_CONFIG_H
#define _OSVELTE_MM_CONFIG_H

/* add feature disable bit here */
enum config_oplus_mm_feature_disable1 {
	COMFD1_EZRECLAIMD = 0,
	COMFD1_KCOMPRESSED = 1,
	COMFD1_MGLRU_OPT = 2,
	COMFD1_QPACE = 3,
	COMFD1_DISABLE_TCACHE_OFF = 4,
	COMFD1_FRR = 5,
};
static const char *module_name_uxmem_opt = "oplus_bsp_uxmem_opt";
struct config_oplus_bsp_uxmem_opt {
	bool enable;
	unsigned int page_pool_order0_mb;
	unsigned int page_pool_order1_mb;
};

static const char *module_name_boost_pool = "oplus_boost_pool";
struct config_oplus_boost_pool {
	bool enable;
};

static const char *module_name_zram_opt = "oplus_bsp_zram_opt";
struct config_oplus_bsp_zram_opt {
	bool balance_anon_file_reclaim_always_true;
};

static const char *module_name_ezreclaimd = "ezreclaimd";
struct config_ezreclaimd {
	bool enable;
};

static const char *module_name_kcompressed = "kcompressed";
struct config_kcompressed {
	bool enable;
};

/* file read record */
static const char *module_name_frr = "oplus_bsp_frr";
struct config_frr {
	bool enable;
};

static const char *module_name_mglru_opt = "oplus_bsp_mglru_opt";
struct config_oplus_bsp_mglru_opt {
	bool enable;
};

extern int mm_config_init(struct proc_dir_entry *root);
extern int mm_config_exit(void);
extern void *oplus_read_mm_config(const char *module_name);
extern bool oplus_test_mm_feature_disable(unsigned long nr);
extern bool oplus_mm_feature_ezreclaimd_enable(void);
#endif /* _OSVELTE_MM_CONFIG_H */
