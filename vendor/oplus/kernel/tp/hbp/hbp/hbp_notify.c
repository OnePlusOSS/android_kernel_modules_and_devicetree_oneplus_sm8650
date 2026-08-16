#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/syscalls.h>
#include <linux/vmalloc.h>
#include <uapi/linux/sched/types.h>
#include <drm/drm_panel.h>

#include "hbp_notify.h"
#include "hbp_core.h"
#include "utils/debug.h"
#include "hbp_exception.h"
#include "hbp_healthinfo.h"

#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
#include <linux/msm_drm_notify.h>
#include <linux/soc/qcom/panel_event_notifier.h>

static void qcom_record_notify_list(struct panel_event_notification *notify,
	struct monitor_data *moni)
{
	char report[MAX_HEALTH_REPORT_LEN] = {0};

	if (!notify) {
		hbp_err("notify is null\n");
		return;
	}
	if (!moni) {
		hbp_err("moni is null\n");
		return;
	}
	/*record the notify list when phone boot up*/
	if (moni->notify.notify_cnt < MAX_NOTIFY_LIST) {
		moni->notify.notif_type_list[moni->notify.notify_cnt] = notify->notif_type;
		moni->notify.early_trigger_list[moni->notify.notify_cnt] = notify->notif_data.early_trigger;
		memset(report, 0, MAX_HEALTH_REPORT_LEN);
		snprintf(report, MAX_HEALTH_REPORT_LEN, "notify_type_%d", moni->notify.notif_type_list[moni->notify.notify_cnt]);
		report[MAX_HEALTH_REPORT_LEN - 1] = '\0';
		hbp_healthinfo_report(moni, report);

		memset(report, 0, MAX_HEALTH_REPORT_LEN);
		snprintf(report, MAX_HEALTH_REPORT_LEN, "notify_early_trigger_%d", moni->notify.early_trigger_list[moni->notify.notify_cnt]);
		report[MAX_HEALTH_REPORT_LEN - 1] = '\0';
		hbp_healthinfo_report(moni, report);
	}
	moni->notify.notify_cnt++;
}

static void qcom_record_screen_off_notify(struct panel_event_notification *notify,
	struct monitor_data *moni)
{
	char report[MAX_HEALTH_REPORT_LEN] = {0};
	ktime_t first_time = 0;

	if (!notify) {
		hbp_err("notify is null\n");
		return;
	}
	if (!moni) {
		hbp_err("moni is null\n");
		return;
	}
	/*record the first screen off notify kernel time*/
	if (notify->notif_type == DRM_PANEL_EVENT_BLANK
			|| notify->notif_type ==  DRM_PANEL_EVENT_BLANK_LP) {
		if (!moni->notify.is_first_screen_off_notify) {
			first_time = ktime_to_ms(ktime_get());
			hbp_err("first_screen_off_notify_time:%lld, below:%d\n", first_time, MAX_SCREEN_OFF_NOTIFY_TIME);
			snprintf(report, MAX_HEALTH_REPORT_LEN, "first_screen_off_notify_time_%lld", first_time);
			report[MAX_HEALTH_REPORT_LEN - 1] = '\0';
			hbp_healthinfo_report(moni, report);
			if (first_time <= MAX_SCREEN_OFF_NOTIFY_TIME) {
				hbp_exception_report(EXCEP_RESUME, SIG_SCREEN_ON_NO_ACK, sizeof(SIG_SCREEN_ON_NO_ACK));
				hbp_err("first_screen_off_notify_time_below:%lld, below:%d\n", first_time, MAX_SCREEN_OFF_NOTIFY_TIME);
				hbp_healthinfo_report(moni, FIRST_SCREEN_OFF_NOTIFY_TIME);
			}
		}
		moni->notify.is_first_screen_off_notify = true;
	}
}

static void qcom_panel_event_healthinfo(struct panel_event_notification *notify,
	struct hbp_device *hbp_dev)
{
	struct monitor_data *moni = &hbp_dev->monitor_data;

	qcom_record_screen_off_notify(notify, moni);
	qcom_record_notify_list(notify, moni);
}

static void qcom_panel_event_callback(enum panel_event_notifier_tag tag,
				      struct panel_event_notification *notify,
				      void *pvt_data)
{
	struct hbp_device *hbp_dev = (struct hbp_device *)pvt_data;
	hbp_panel_event event = HBP_PANEL_EVENT_UNKNOWN;

	hbp_debug("tag %d, notify type %d, early_trigger %d\n", tag,
		  notify->notif_type,
		  notify->notif_data.early_trigger);

	if (!notify) {
		hbp_err("notify is null\n");
		return;
	}

	if (!hbp_dev) {
		hbp_err("hbp_dev is null\n");
		return;
	}

	qcom_panel_event_healthinfo(notify, hbp_dev);

	if (notify->notif_type == DRM_PANEL_EVENT_UNBLANK) {
		if (notify->notif_data.early_trigger) {
			event = HBP_PANEL_EVENT_EARLY_RESUME;
		} else {
			event = HBP_PANEL_EVENT_RESUME;
		}
	}

	if (notify->notif_type == DRM_PANEL_EVENT_BLANK
			|| notify->notif_type ==  DRM_PANEL_EVENT_BLANK_LP) {
		if (notify->notif_data.early_trigger) {
			event = HBP_PANEL_EVENT_EARLY_SUSPEND;
		} else {
			event = HBP_PANEL_EVENT_SUSPEND;
		}
	}

	hbp_dev->panel_cb(event, hbp_dev);
}
#if 0
#include <linux/msm_drm_notify.h>
#include <drm/drm_panel.h>
static int qcom_drm_panel_notifier_callback(struct notifier_block *nb, unsigned long event, void *data)
{
	struct hbp_device *hbp_dev = container_of(nb, struct hbp_device, fb_notif);
	hbp_panel_event touch_event = HBP_PANEL_EVENT_UNKNOWN;
	struct msm_drm_notifier *evdata = data;
	int blank =  *(int *)(evdata->data);

	hbp_debug("blank %d, event %d\n", blank, event);

	/*power on*/
	if (blank == MSM_DRM_BLANK_UNBLANK) {
		if (event == MSM_DRM_EARLY_EVENT_BLANK) {
			touch_event = HBP_PANEL_EVENT_EARLY_RESUME;
		} else {
			touch_event = HBP_PANEL_EVENT_RESUME;
		}
	} else if (blank == MSM_DRM_BLANK_POWERDOWN) { /*poweroff*/
		if (event ==  MSM_DRM_EARLY_EVENT_BLANK) {
			touch_event = HBP_PANEL_EVENT_EARLY_SUSPEND;
		} else {
			touch_event = HBP_PANEL_EVENT_SUSPEND;
		}
	} else {
		hbp_err("unknow blank %d event %d\n", blank, event);
	}

	hbp_dev->panel_cb(touch_event, hbp_dev);

	return 0;
}

#endif

static void hbp_register_notify_cb_qcom(struct hbp_device *hbp_dev, struct drm_panel *drm)
{
#if 1
	void *entry;
	enum panel_event_notifier_tag tag = (hbp_dev->id == 0)?
					    PANEL_EVENT_NOTIFICATION_PRIMARY:
					    PANEL_EVENT_NOTIFICATION_SECONDARY;
	enum panel_event_notifier_client client = (hbp_dev->id == 0)?
			PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH:
			PANEL_EVENT_NOTIFIER_CLIENT_SECONDARY_TOUCH;


	entry = panel_event_notifier_register(tag, client, drm, qcom_panel_event_callback, hbp_dev);
	if (!entry) {
		hbp_err("Failed to register panel notify\n");
		return;
	}
#else
int ret = 0;
	hbp_dev->fb_notif.notifier_call = qcom_drm_panel_notifier_callback;
	if (drm) {
		ret = drm_panel_notifier_register(drm, &hbp_dev->fb_notif);
		if (ret < 0) {
			hbp_err("failed to register notifier\n");
		}
	}
#endif
}
#endif

#if IS_ENABLED(CONFIG_TOUCHPANEL_MTK_PLATFORM)
#include <linux/mtk_disp_notify.h>

static void mtk_record_notify_list(unsigned long event, void *data,
	struct monitor_data *moni)
{
	char report[MAX_HEALTH_REPORT_LEN] = {0};
	int blank =  *(int *)(data);

	if (!moni) {
		hbp_err("moni is null\n");
		return;
	}
	/*record the notify list when phone boot up*/
	if (moni->notify.notify_cnt < MAX_NOTIFY_LIST) {
		moni->notify.notif_type_list[moni->notify.notify_cnt] = blank;
		moni->notify.early_trigger_list[moni->notify.notify_cnt] = event;
		memset(report, 0, MAX_HEALTH_REPORT_LEN);
		snprintf(report, MAX_HEALTH_REPORT_LEN, "notify_type_%d", moni->notify.notif_type_list[moni->notify.notify_cnt]);
		report[MAX_HEALTH_REPORT_LEN - 1] = '\0';
		hbp_healthinfo_report(moni, report);

		memset(report, 0, MAX_HEALTH_REPORT_LEN);
		snprintf(report, MAX_HEALTH_REPORT_LEN, "notify_early_trigger_%d", moni->notify.early_trigger_list[moni->notify.notify_cnt]);
		report[MAX_HEALTH_REPORT_LEN - 1] = '\0';
		hbp_healthinfo_report(moni, report);
	}
	moni->notify.notify_cnt++;
}

static void mtk_record_screen_off_notify(unsigned long event, void *data,
	struct monitor_data *moni)
{
	char report[MAX_HEALTH_REPORT_LEN] = {0};
	ktime_t first_time = 0;
	int blank =  *(int *)(data);

	if (!moni) {
		hbp_err("moni is null\n");
		return;
	}
	/*record the first screen off notify kernel time*/
	if (blank == MTK_DISP_BLANK_POWERDOWN) {
		if (!moni->notify.is_first_screen_off_notify) {
			first_time = ktime_to_ms(ktime_get());
			hbp_err("first_screen_off_notify_time:%lld, below:%d\n", first_time, MAX_SCREEN_OFF_NOTIFY_TIME);
			snprintf(report, MAX_HEALTH_REPORT_LEN, "first_screen_off_notify_time_%lld", first_time);
			report[MAX_HEALTH_REPORT_LEN - 1] = '\0';
			hbp_healthinfo_report(moni, report);
			if (first_time <= MAX_SCREEN_OFF_NOTIFY_TIME) {
				hbp_exception_report(EXCEP_RESUME, SIG_SCREEN_ON_NO_ACK, sizeof(SIG_SCREEN_ON_NO_ACK));
				hbp_err("first_screen_off_notify_time_below:%lld, below:%d\n", first_time, MAX_SCREEN_OFF_NOTIFY_TIME);
				hbp_healthinfo_report(moni, FIRST_SCREEN_OFF_NOTIFY_TIME);
			}
		}
		moni->notify.is_first_screen_off_notify = true;
	}
}

static void mtk_panel_event_healthinfo(unsigned long event, void *data,
	struct hbp_device *hbp_dev)
{
	struct monitor_data *moni = &hbp_dev->monitor_data;

	mtk_record_screen_off_notify(event, data, moni);
	mtk_record_notify_list(event, data, moni);
}

static int mtk_drm_panel_notifier_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct hbp_device *hbp_dev = container_of(nb, struct hbp_device, fb_notif);
	hbp_panel_event touch_event = HBP_PANEL_EVENT_UNKNOWN;
	int blank =  *(int *)(data);

	hbp_debug("blank %d, event %ld\n", blank, event);

	if (!hbp_dev) {
		hbp_err("hbp_dev is null\n");
		return 0;
	}
	mtk_panel_event_healthinfo(event, data, hbp_dev);

	if (blank == MTK_DISP_BLANK_UNBLANK) {
		if (event == MTK_DISP_EARLY_EVENT_BLANK) {
			touch_event = HBP_PANEL_EVENT_EARLY_RESUME;
		} else if (event == MTK_DISP_EVENT_BLANK) {
			touch_event = HBP_PANEL_EVENT_RESUME;
		} else {
			hbp_warn("unkown blank %d, event %ld\n", blank, event);
		}
	} else if (blank == MTK_DISP_BLANK_POWERDOWN) {
		if (event == MTK_DISP_EARLY_EVENT_BLANK) {
			touch_event = HBP_PANEL_EVENT_EARLY_SUSPEND;
		} else if (event == MTK_DISP_EVENT_BLANK) {
			touch_event = HBP_PANEL_EVENT_SUSPEND;
		} else {
			hbp_warn("unkown blank %d, event %ld\n", blank, event);
			return 0;
		}
	} else {
		hbp_warn("unkown blank %d\n", blank);
		return 0;
	}

	hbp_dev->panel_cb(touch_event, hbp_dev);
	return 0;
}

static void hbp_register_notify_cb_mtk(struct hbp_device *hbp_dev, struct drm_panel *drm)
{
	hbp_dev->fb_notif.notifier_call = mtk_drm_panel_notifier_callback;
	if (hbp_dev->id == 0) {
		if (mtk_disp_notifier_register("oplus_touch", &hbp_dev->fb_notif)) {
			hbp_err("failed to register main display notifer\n");
		}
	} else {
		if (mtk_disp_sub_notifier_register("oplus_touch_sub", &hbp_dev->fb_notif)) {
			hbp_err("failed to register main display notifer\n");
		}
	}
}
#endif


int hbp_register_notify_cb(struct hbp_device *hbp_dev, struct device *dev)
{
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	struct drm_panel *drm;
	struct device_node *dsi_np, *panel_np;
	const char *dsi_name;
	int i = 0, count = 0;
	int retry = 0;

	for(retry = 0; retry < 10; retry++) {
		dsi_np = of_find_node_by_name(NULL, "oplus,dsi-display-dev");
		if (!dsi_np) {
			dsi_name = "dsi_panel";
			dsi_np = dev->of_node;
		} else {
			if (hbp_dev->id == 0) {
				dsi_name = "oplus,dsi-panel-primary";
			} else {
				dsi_name = "oplus,dsi-panel-secondary";
			}
		}

		count = of_count_phandle_with_args(dsi_np, dsi_name, NULL);
		for (i = 0; i < count; i++) {
			panel_np = of_parse_phandle(dsi_np, dsi_name, i);
			if (panel_np) {
				hbp_info("panel list name:%s\n", panel_np->name);
				drm = of_drm_find_panel(panel_np);
				of_node_put(panel_np);
				if (!IS_ERR_OR_NULL(drm)) {
					break;
				} else {
					hbp_err("drm err %ld",  PTR_ERR(drm));
				}
			} else {
				hbp_err("failed to find dsi name %s\n", dsi_name);
			}
			if (count < 0 || i == count) {
				hbp_err("Failed to find attached panel, count = %d\n", count);
				break;
			}
		}
		if (!IS_ERR_OR_NULL(drm)) {
			hbp_info("Success to get panel info\n");
			break;
		}
	}
	if (retry == 10) {
		hbp_err("ts check panel dt failed\n");
		return -EPROBE_DEFER;
	}

	hbp_register_notify_cb_qcom(hbp_dev, drm);
#else
	hbp_register_notify_cb_mtk(hbp_dev, NULL);
#endif
	return 0;
}

void hbp_event_call_notifier(unsigned long action, void *data)
{
	touchpanel_event_call_notifier(action, data);
}

/*
int (*tp_gesture_enable_notifier)(unsigned int tp_index) = NULL;
EXPORT_SYMBOL(tp_gesture_enable_notifier);
*/

