#ifndef __OPLUS_TVS_DAMAGE_CHECK_H__
#define __OPLUS_TVS_DAMAGE_CHECK_H__

#include <linux/errno.h>
#include <linux/types.h>

struct device;
struct device_node;
struct oplus_chg_ic_dev;
struct oplus_wired_tvs;

struct oplus_wired_tvs_ops {
	struct oplus_chg_ic_dev *(*get_buck_ic)(void *data);
	void (*ccdetect_enable)(void *data, bool en);
	bool (*get_otg_switch_status)(void *data);
	bool (*boot_completed)(void *data);
	bool (*err_topic_available)(void *data);
};

#if IS_ENABLED(CONFIG_OPLUS_TVS_DAMAGE_SUPPORT) && \
	IS_ENABLED(CONFIG_OPLUS_MT6379_CHARGER)
int oplus_wired_tvs_init(struct device *dev, struct oplus_wired_tvs **tvs,
			 void *data, const struct oplus_wired_tvs_ops *ops);
void oplus_wired_tvs_deinit(struct oplus_wired_tvs *tvs);
void oplus_wired_tvs_parse_dt(struct oplus_wired_tvs *tvs,
			      struct device_node *node);
bool oplus_wired_tvs_check_enabled(struct oplus_wired_tvs *tvs);
bool oplus_wired_tvs_repair_enabled(struct oplus_wired_tvs *tvs);
bool oplus_wired_tvs_handle_ccdetect_attach(struct oplus_wired_tvs *tvs);
void oplus_wired_tvs_handle_ccdetect_detach(struct oplus_wired_tvs *tvs);
void oplus_wired_tvs_handle_otg_disable(struct oplus_wired_tvs *tvs);
void oplus_wired_tvs_schedule_status_sync(struct oplus_wired_tvs *tvs);
int oplus_wired_tvs_get_config(struct oplus_wired_tvs *tvs);
int oplus_wired_tvs_get_repair_status(struct oplus_wired_tvs *tvs);
int oplus_wired_tvs_set_config(struct oplus_wired_tvs *tvs, u32 config);
#else
static inline int oplus_wired_tvs_init(struct device *dev,
				       struct oplus_wired_tvs **tvs,
				       void *data,
				       const struct oplus_wired_tvs_ops *ops)
{
	if (tvs)
		*tvs = NULL;
	return 0;
}

static inline void oplus_wired_tvs_deinit(struct oplus_wired_tvs *tvs) {}
static inline void oplus_wired_tvs_parse_dt(struct oplus_wired_tvs *tvs,
					    struct device_node *node) {}
static inline bool oplus_wired_tvs_check_enabled(struct oplus_wired_tvs *tvs)
{
	return false;
}
static inline bool oplus_wired_tvs_repair_enabled(struct oplus_wired_tvs *tvs)
{
	return false;
}
static inline bool oplus_wired_tvs_handle_ccdetect_attach(struct oplus_wired_tvs *tvs)
{
	return false;
}
static inline void oplus_wired_tvs_handle_ccdetect_detach(struct oplus_wired_tvs *tvs) {}
static inline void oplus_wired_tvs_handle_otg_disable(struct oplus_wired_tvs *tvs) {}
static inline void oplus_wired_tvs_schedule_status_sync(struct oplus_wired_tvs *tvs) {}
static inline int oplus_wired_tvs_get_config(struct oplus_wired_tvs *tvs)
{
	return 0;
}
static inline int oplus_wired_tvs_get_repair_status(struct oplus_wired_tvs *tvs)
{
	return 0;
}
static inline int oplus_wired_tvs_set_config(struct oplus_wired_tvs *tvs, u32 config)
{
	return -ENOTSUPP;
}
#endif /* CONFIG_OPLUS_TVS_DAMAGE_SUPPORT && CONFIG_OPLUS_MT6379_CHARGER */

#endif /* __OPLUS_TVS_DAMAGE_CHECK_H__ */
