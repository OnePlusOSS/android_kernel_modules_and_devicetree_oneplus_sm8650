#define pr_fmt(fmt) "[TVS_DAMAGE_CHECK]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include <oplus_chg_ic.h>
#include <oplus_chg_mutual.h>
#include <oplus_chg_module.h>
#include <oplus_mms.h>
#if IS_ENABLED(CONFIG_TCPC_CLASS)
#include <tcpm.h>
#endif

#include <oplus_tvs_damage_check.h>

#if IS_ENABLED(CONFIG_OPLUS_TVS_DAMAGE_SUPPORT) && \
	IS_ENABLED(CONFIG_OPLUS_MT6379_CHARGER)

#define OPLUS_CC_STATUS_POLL_FIRST_DELAY_MS	100
#define OPLUS_CC_TVS_STATUS_SYNC_DELAY_MS	2000
#define OPLUS_CC_TVS_STATUS_SYNC_RETRY_MAX	3
#define CC_TVS_STATUS_TAG_LEN			12
#define CC_TVS_STATUS_TAG			"cc_tvs_stat"
#define CC_TVS_CONFIG_DISABLE_CHECK		(1 << 0)
#define CC_TVS_CONFIG_REPAIR_ENABLE		(1 << 1)
#define CC_TVS_CONFIG_MASK			(CC_TVS_CONFIG_DISABLE_CHECK | \
						 CC_TVS_CONFIG_REPAIR_ENABLE)

/* Keep the repair solution open */
#define CC_TVS_CONFIG_REPAIR_ENABLE_OPEN	0x01

struct cc_tvs_status_load {
	char tag_info[CC_TVS_STATUS_TAG_LEN];
	int check_done;
	int tvs_damaged;
};

struct oplus_wired_tvs {
	struct device *dev;
	void *data;
	const struct oplus_wired_tvs_ops *ops;
	bool support_check;
	u32 config;
	bool check_done;
	bool damaged;
	int status_sync_retry_count;
	struct cc_tvs_status_load status_info;
	struct oplus_chg_mutual_notifier status_obtain_nb;
	struct oplus_chg_mutual_notifier status_update_nb;
	struct delayed_work status_sync_work;
	struct work_struct status_update_work;
	struct delayed_work cc_status_poll_work;
};

static struct oplus_chg_ic_dev *oplus_wired_tvs_get_buck_ic(struct oplus_wired_tvs *tvs)
{
	if (!tvs || !tvs->ops || !tvs->ops->get_buck_ic)
		return NULL;

	return tvs->ops->get_buck_ic(tvs->data);
}

static int oplus_wired_tvs_set_cc_open(struct oplus_wired_tvs *tvs)
{
	struct oplus_chg_ic_dev *buck_ic;

	buck_ic = oplus_wired_tvs_get_buck_ic(tvs);
	if (!buck_ic)
		return -ENODEV;

	return oplus_chg_ic_func(buck_ic, OPLUS_IC_FUNC_BUCK_SET_TVS_CC_OPEN);
}

bool oplus_wired_tvs_check_enabled(struct oplus_wired_tvs *tvs)
{
	if (!tvs)
		return false;

	return tvs->support_check &&
	       !(tvs->config & CC_TVS_CONFIG_DISABLE_CHECK);
}

bool oplus_wired_tvs_repair_enabled(struct oplus_wired_tvs *tvs)
{
	if (!tvs)
		return false;
	if (CC_TVS_CONFIG_REPAIR_ENABLE_OPEN)
		return true;
	if (!oplus_wired_tvs_check_enabled(tvs))
		return false;

	return !!(tvs->config & CC_TVS_CONFIG_REPAIR_ENABLE);
}

static bool oplus_wired_tvs_boot_completed(struct oplus_wired_tvs *tvs)
{
	if (!tvs || !tvs->ops || !tvs->ops->boot_completed)
		return false;

	return tvs->ops->boot_completed(tvs->data);
}

static void oplus_wired_tvs_ccdetect_enable(struct oplus_wired_tvs *tvs, bool en)
{
	if (!tvs || !tvs->ops || !tvs->ops->ccdetect_enable)
		return;

	tvs->ops->ccdetect_enable(tvs->data, en);
}

static bool oplus_wired_tvs_get_otg_switch_status(struct oplus_wired_tvs *tvs)
{
	if (!tvs || !tvs->ops || !tvs->ops->get_otg_switch_status)
		return false;

	return tvs->ops->get_otg_switch_status(tvs->data);
}

static bool oplus_wired_tvs_err_topic_available(struct oplus_wired_tvs *tvs)
{
	if (!tvs || !tvs->ops || !tvs->ops->err_topic_available)
		return false;

	return tvs->ops->err_topic_available(tvs->data);
}

static void oplus_wired_tvs_status_sync_cache(struct oplus_wired_tvs *tvs,
					      bool check_done, bool damaged)
{
	if (!tvs)
		return;

	memset(&tvs->status_info, 0, sizeof(tvs->status_info));
	strscpy(tvs->status_info.tag_info, CC_TVS_STATUS_TAG,
		sizeof(tvs->status_info.tag_info));
	tvs->status_info.check_done = check_done ? 1 : 0;
	tvs->status_info.tvs_damaged = damaged ? 1 : 0;
	tvs->check_done = check_done;
	tvs->damaged = damaged;
}

static void oplus_wired_tvs_update_status_partition(struct oplus_wired_tvs *tvs,
						    bool check_done, bool damaged)
{
	if (!oplus_wired_tvs_check_enabled(tvs))
		return;

	oplus_wired_tvs_status_sync_cache(tvs, check_done, damaged);
	queue_work(system_highpri_wq, &tvs->status_update_work);
}

static int oplus_wired_tvs_status_obtain_notifier_call(
		struct notifier_block *nb, unsigned long param, void *v)
{
	struct cc_tvs_status_load *status_info;
	struct oplus_chg_mutual_notifier *notifier;
	struct oplus_wired_tvs *tvs;
	int rc;

	notifier = container_of(nb, struct oplus_chg_mutual_notifier, nb);
	tvs = container_of(notifier, struct oplus_wired_tvs, status_obtain_nb);

	if (mutual_info_to_cmd(param) != CMD_CC_TVS_STATUS_OBTAIN) {
		chg_err("cmd is not matching, should return\n");
		return NOTIFY_OK;
	}

	if (mutual_info_to_data_size(param) != sizeof(struct cc_tvs_status_load)) {
		chg_err("data_len is not ok, datas is invalid\n");
		return NOTIFY_DONE;
	}

	status_info = (struct cc_tvs_status_load *)v;
	if (!status_info)
		return NOTIFY_DONE;

	status_info->tag_info[CC_TVS_STATUS_TAG_LEN - 1] = '\0';
	if (strncmp(status_info->tag_info, CC_TVS_STATUS_TAG,
		    sizeof(CC_TVS_STATUS_TAG))) {
		chg_err("cc tvs status tag is invalid\n");
		oplus_wired_tvs_status_sync_cache(tvs, false, false);
		return NOTIFY_DONE;
	}

	oplus_wired_tvs_status_sync_cache(tvs, !!status_info->check_done,
					  !!status_info->tvs_damaged);
	chg_info("cc tvs status obtain: done=%d, damaged=%d\n",
		 tvs->check_done, tvs->damaged);
	if (oplus_wired_tvs_repair_enabled(tvs) && tvs->damaged) {
		rc = oplus_wired_tvs_set_cc_open(tvs);
		if (rc < 0)
			chg_err("set cc open failed, rc=%d\n", rc);
		oplus_wired_tvs_ccdetect_enable(tvs, false);
	}

	return NOTIFY_OK;
}

static int oplus_wired_tvs_status_update_notifier_call(
		struct notifier_block *nb, unsigned long param, void *v)
{
	if (mutual_info_to_cmd(param) != CMD_CC_TVS_STATUS_UPDATE) {
		chg_err("cmd is not matching, should return\n");
		return NOTIFY_OK;
	}

	return NOTIFY_OK;
}

static void oplus_wired_tvs_status_sync_work(struct work_struct *work)
{
	int rc;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_wired_tvs *tvs =
		container_of(dwork, struct oplus_wired_tvs, status_sync_work);

	if (!oplus_wired_tvs_check_enabled(tvs))
		return;

	rc = oplus_chg_set_mutual_cmd(CMD_CC_TVS_STATUS_OBTAIN, 0, NULL);
	if (rc != CMD_ACK_OK && tvs->status_sync_retry_count-- > 0)
		schedule_delayed_work(&tvs->status_sync_work,
				      msecs_to_jiffies(OPLUS_CC_TVS_STATUS_SYNC_DELAY_MS));
	else if (rc == CMD_ACK_OK)
		chg_info("cc tvs status partition read completed, check_done=%d, tvs_damaged=%d\n",
			 tvs->check_done, tvs->damaged);
}

static void oplus_wired_tvs_status_update_work(struct work_struct *work)
{
	int rc;
	struct oplus_wired_tvs *tvs =
		container_of(work, struct oplus_wired_tvs, status_update_work);

	rc = oplus_chg_set_mutual_cmd(CMD_CC_TVS_STATUS_UPDATE,
		sizeof(tvs->status_info), &tvs->status_info);
	if (rc != CMD_ACK_OK)
		chg_info("cc tvs status update fail, rc=%d\n", rc);
}

static void oplus_wired_tvs_push_damage_track(struct oplus_wired_tvs *tvs,
					      u8 cc1, u8 cc2)
{
	struct oplus_chg_ic_dev *buck_ic;
	int rc;

	if (!oplus_wired_tvs_check_enabled(tvs))
		return;
	if (!oplus_wired_tvs_err_topic_available(tvs)) {
		chg_err("tvs track: error topic not found\n");
		return;
	}
	buck_ic = oplus_wired_tvs_get_buck_ic(tvs);
	if (!buck_ic) {
		chg_err("tvs track: buck_ic not found\n");
		return;
	}

	rc = oplus_chg_ic_creat_err_msg(buck_ic, OPLUS_IC_ERR_CC_TVS, 0,
					"$$err_reason@@TVS_damage$$cc1@@%u$$cc2@@%u",
					cc1, cc2);
	if (rc < 0) {
		chg_err("tvs track: create ic err msg error, rc=%d\n", rc);
		return;
	}
	rc = oplus_chg_ic_virq_trigger(buck_ic, OPLUS_IC_VIRQ_ERR);
	if (rc < 0) {
		chg_err("tvs track: trigger ic err virq error, rc=%d\n", rc);
		return;
	}

	chg_info("tvs track: TVS_damage ic err msg triggered\n");
}

/* 0=hw_detect is 0; <0=IC error; >0=hw_detect nonzero, caller should skip */
static int oplus_wired_tvs_hw_detect_must_clear(struct oplus_chg_ic_dev *buck_ic,
						const char *phase)
{
	int hw_detect;
	int rc;
	bool recheck = phase && !strncmp(phase, "post", sizeof("post"));

	rc = oplus_chg_ic_func(buck_ic, OPLUS_IC_FUNC_BUCK_GET_HW_DETECT, &hw_detect, false);
	if (rc < 0) {
		if (recheck)
			chg_err("cc_status_poll: recheck hw detect failed, rc=%d\n", rc);
		else
			chg_err("cc_status_poll: get hw detect failed, rc=%d\n", rc);
		return rc;
	}
	if (hw_detect != 0) {
		if (recheck)
			chg_info("cc_status_poll: cc status invalid after recheck (hw_detect=%d)\n",
				 hw_detect);
		else
			chg_info("cc_status_poll: skip (hw_detect=%d)\n", hw_detect);
		return hw_detect;
	}
	return 0;
}

static void oplus_wired_tvs_cc_status_poll_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_wired_tvs *tvs =
		container_of(dwork, struct oplus_wired_tvs, cc_status_poll_work);
	struct oplus_chg_ic_dev *buck_ic;
	bool cc_tvs_status;
	int rc;
	u8 cc1, cc2;

	if (!oplus_wired_tvs_check_enabled(tvs))
		return;
	if (!oplus_wired_tvs_boot_completed(tvs)) {
		chg_info("cc_status_poll: boot not completed, skip tvs check\n");
		return;
	}

	buck_ic = oplus_wired_tvs_get_buck_ic(tvs);
	if (!buck_ic) {
		chg_err("cc_status_poll: buck_ic not found\n");
		return;
	}

	rc = oplus_wired_tvs_hw_detect_must_clear(buck_ic, "pre");
	if (rc)
		return;

	rc = oplus_chg_ic_func(buck_ic, OPLUS_IC_FUNC_BUCK_GET_TVS_CC_STATE, &cc1, &cc2);
	if (rc < 0) {
		chg_err("cc_status_poll: get cc state failed, rc=%d\n", rc);
		return;
	}

	rc = oplus_wired_tvs_hw_detect_must_clear(buck_ic, "post");
	if (rc)
		return;

	cc_tvs_status = (cc1 == TYPEC_CC_VOLT_RD || cc2 == TYPEC_CC_VOLT_RD);
	if (tvs->check_done && cc_tvs_status == tvs->damaged) {
		chg_info("cc tvs status unchanged: check_done=%d, damaged=%d, current=%d\n",
			 tvs->check_done, tvs->damaged, cc_tvs_status);
		return;
	}

	oplus_wired_tvs_update_status_partition(tvs, true, cc_tvs_status);
	oplus_wired_tvs_push_damage_track(tvs, cc1, cc2);
}

bool oplus_wired_tvs_handle_ccdetect_attach(struct oplus_wired_tvs *tvs)
{
	if (tvs && oplus_wired_tvs_repair_enabled(tvs) && tvs->damaged) {
		oplus_wired_tvs_ccdetect_enable(tvs,
			oplus_wired_tvs_get_otg_switch_status(tvs));
		return true;
	}

	return false;
}

void oplus_wired_tvs_handle_ccdetect_detach(struct oplus_wired_tvs *tvs)
{
	if (!oplus_wired_tvs_check_enabled(tvs))
		return;

	cancel_delayed_work(&tvs->cc_status_poll_work);
	schedule_delayed_work(&tvs->cc_status_poll_work,
			      msecs_to_jiffies(OPLUS_CC_STATUS_POLL_FIRST_DELAY_MS));
}

void oplus_wired_tvs_handle_otg_disable(struct oplus_wired_tvs *tvs)
{
	int rc;

	if (oplus_wired_tvs_check_enabled(tvs) && tvs->damaged) {
		rc = oplus_wired_tvs_set_cc_open(tvs);
		if (rc < 0)
			chg_err("set cc open failed, rc=%d\n", rc);
	}
}

void oplus_wired_tvs_schedule_status_sync(struct oplus_wired_tvs *tvs)
{
	if (!oplus_wired_tvs_check_enabled(tvs))
		return;

	tvs->status_sync_retry_count = OPLUS_CC_TVS_STATUS_SYNC_RETRY_MAX;
	schedule_delayed_work(&tvs->status_sync_work,
			      msecs_to_jiffies(OPLUS_CC_TVS_STATUS_SYNC_DELAY_MS));
}

int oplus_wired_tvs_get_config(struct oplus_wired_tvs *tvs)
{
	if (!tvs)
		return 0;

	return tvs->config;
}

int oplus_wired_tvs_get_repair_status(struct oplus_wired_tvs *tvs)
{
	return oplus_wired_tvs_repair_enabled(tvs) ? 1 : 0;
}

int oplus_wired_tvs_set_config(struct oplus_wired_tvs *tvs, u32 config)
{
	if (!tvs)
		return -ENODEV;

	tvs->config = config & CC_TVS_CONFIG_MASK;
	if (tvs->config & CC_TVS_CONFIG_DISABLE_CHECK) {
		cancel_delayed_work(&tvs->status_sync_work);
		cancel_delayed_work(&tvs->cc_status_poll_work);
	}
	chg_info("set cc tvs config=0x%x\n", tvs->config);

	return 0;
}

void oplus_wired_tvs_parse_dt(struct oplus_wired_tvs *tvs, struct device_node *node)
{
	if (!tvs || !node)
		return;

	tvs->support_check = of_property_read_bool(node, "oplus,support_cc_tvs_check");
	tvs->config = 0;
	tvs->check_done = false;
	tvs->damaged = false;
	chg_info("cc_tvs_check=%d, config=0x%x\n", tvs->support_check, tvs->config);
}

int oplus_wired_tvs_init(struct device *dev, struct oplus_wired_tvs **tvs,
			 void *data, const struct oplus_wired_tvs_ops *ops)
{
	struct oplus_wired_tvs *chip;
	int rc;

	if (!dev || !tvs || !ops)
		return -EINVAL;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->data = data;
	chip->ops = ops;
	INIT_DELAYED_WORK(&chip->status_sync_work, oplus_wired_tvs_status_sync_work);
	INIT_WORK(&chip->status_update_work, oplus_wired_tvs_status_update_work);
	INIT_DELAYED_WORK(&chip->cc_status_poll_work, oplus_wired_tvs_cc_status_poll_work);

	chip->status_obtain_nb.name = "cc_tvs_status_obtain";
	chip->status_obtain_nb.cmd = CMD_CC_TVS_STATUS_OBTAIN;
	chip->status_obtain_nb.nb.notifier_call =
		oplus_wired_tvs_status_obtain_notifier_call;
	rc = oplus_chg_reg_mutual_notifier(&chip->status_obtain_nb);
	if (rc < 0) {
		chg_err("register cc tvs status obtain notifier error, rc=%d\n", rc);
		return rc;
	}

	chip->status_update_nb.name = "cc_tvs_status_update";
	chip->status_update_nb.cmd = CMD_CC_TVS_STATUS_UPDATE;
	chip->status_update_nb.nb.notifier_call =
		oplus_wired_tvs_status_update_notifier_call;
	rc = oplus_chg_reg_mutual_notifier(&chip->status_update_nb);
	if (rc < 0) {
		chg_err("register cc tvs status update notifier error, rc=%d\n", rc);
		oplus_chg_unreg_mutual_notifier(&chip->status_obtain_nb);
		return rc;
	}

	*tvs = chip;
	return 0;
}

void oplus_wired_tvs_deinit(struct oplus_wired_tvs *tvs)
{
	if (!tvs)
		return;

	oplus_chg_unreg_mutual_notifier(&tvs->status_update_nb);
	oplus_chg_unreg_mutual_notifier(&tvs->status_obtain_nb);
	cancel_delayed_work_sync(&tvs->status_sync_work);
	cancel_work_sync(&tvs->status_update_work);
	cancel_delayed_work_sync(&tvs->cc_status_poll_work);
}

#endif /* CONFIG_OPLUS_TVS_DAMAGE_SUPPORT && CONFIG_OPLUS_MT6379_CHARGER */
