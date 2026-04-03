#ifndef __HBP_NOTIFY_H__
#define __HBP_NOTIFY_H__

#ifdef BUILD_BY_BAZEL
#include <soc/oplus/touchpanel_event_notify.h>/* kernel 6.1 */
#else
#include "../../../touchpanel/touchpanel_notify/touchpanel_event_notify.h"
#endif
//#define EVENT_ACTION_FOR_FINGPRINT  (0x01)

struct fp_event {
	int id;
	int x;
	int y;
	int fid;       /* Finger ID */
	char type;     /* 'D' - Down, 'M' - Move, 'U' - Up, */
	int touch_state;
	int area_rate;
	int touch_early_down_flag;
	long is_touch_fp_area_cnt;
	ktime_t touch_fp_area_time;
	ktime_t fp_down_time;
	int tp_firmware_time;
};

struct film_info {
	bool filmed;
	int level;
	bool trusty;
};

struct touch_fp_grip_info {
	int value;
	int reserved;
};

typedef enum {
	HBP_PANEL_EVENT_UNKNOWN = 0,
	HBP_PANEL_EVENT_EARLY_SUSPEND = 0x01,
	HBP_PANEL_EVENT_SUSPEND,
	HBP_PANEL_EVENT_EARLY_RESUME,
	HBP_PANEL_EVENT_RESUME,
} hbp_panel_event;

/*
extern void (*lcm_notify_cb)();
static void inline lcm_notify_hbp_unlocked(enum PANEL_INDEX, hbp_panel_event event, void *data)
{
	if (!)
		return;

}
*/

/*
int hbp_event_register_notifier(struct notifier_block *nb);
int hbp_event_unregister_notifier(struct notifier_block *nb);
*/
void hbp_event_call_notifier(unsigned long action, void *data);

/*
#define touchpanel_event_register_notifier hbp_event_register_notifier
#define touchpanel_event_unregister_notifier hbp_event_unregister_notifier
#define touchpanel_event_call_notifier hbp_event_call_notifier
*/

#endif
