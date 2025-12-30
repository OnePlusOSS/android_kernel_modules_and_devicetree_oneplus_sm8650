#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>

#include "hbp_core.h"
#include "utils/debug.h"

int hbp_init_pinctrl(struct device *dev, struct hbp_device *hbp_dev)
{
	hbp_dev->hw.pctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(hbp_dev->hw.pctrl)) {
		hbp_err("failed to get pinctrl\n");
		return -ENODEV;
	}

	hbp_dev->hw.reset_active = pinctrl_lookup_state(hbp_dev->hw.pctrl, "ts_reset_active");
	if (IS_ERR_OR_NULL(hbp_dev->hw.reset_active)) {
		hbp_err("failed to get pinctrl: reset_active\n");
	}
	hbp_dev->hw.reset_idle = pinctrl_lookup_state(hbp_dev->hw.pctrl, "ts_reset_idle");
	if (IS_ERR_OR_NULL(hbp_dev->hw.reset_idle)) {
		hbp_err("failed to get pinctrl: reset_idle\n");
	}
	hbp_dev->hw.avdd_on = pinctrl_lookup_state(hbp_dev->hw.pctrl, "ts_avdd_on");
	if (IS_ERR_OR_NULL(hbp_dev->hw.avdd_on)) {
		hbp_err("failed to get pinctrl: avdd_on\n");
	}
	hbp_dev->hw.avdd_off = pinctrl_lookup_state(hbp_dev->hw.pctrl, "ts_avdd_off");
	if (IS_ERR_OR_NULL(hbp_dev->hw.avdd_off)) {
		hbp_err("failed to get pinctrl: avdd_off\n");
	}
	hbp_dev->hw.vddi_on = pinctrl_lookup_state(hbp_dev->hw.pctrl, "ts_vddi_on");
	if (IS_ERR_OR_NULL(hbp_dev->hw.vddi_on)) {
		hbp_err("failed to get pinctrl: vddi_on\n");
	}
	hbp_dev->hw.vddi_off = pinctrl_lookup_state(hbp_dev->hw.pctrl, "ts_vddi_off");
	if (IS_ERR_OR_NULL(hbp_dev->hw.vddi_off)) {
		hbp_err("failed to get pinctrl: vddi_off\n");
	}
	hbp_dev->hw.resume = pinctrl_lookup_state(hbp_dev->hw.pctrl, "ts_resume");
	if (IS_ERR_OR_NULL(hbp_dev->hw.resume)) {
		hbp_err("failed to get pinctrl: resume\n");
	}
	hbp_dev->hw.suspend = pinctrl_lookup_state(hbp_dev->hw.pctrl, "ts_suspend");
	if (IS_ERR_OR_NULL(hbp_dev->hw.suspend)) {
		hbp_err("failed to get pinctrl: suspend\n");
	}

	return 0;
}

int hbp_init_avdd(struct device *dev, struct hbp_device *hbp_dev)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

	/*for avdd control init, instead of regulator_get which may return dummy regulator*/
	hbp_dev->hw.avdd_reg = regulator_get_optional(dev, "power,avdd");
	if (IS_ERR_OR_NULL(hbp_dev->hw.avdd_reg)) {
		hbp_info("not support avdd, or dummy device\n");
	} else {
		ret = of_property_read_u32_array(np,
						 "power,avdd_volt",
						 (uint32_t *)&hbp_dev->hw.avdd_volt, 2);
		if (ret) {
			hbp_dev->hw.avdd_volt.min = 3100000; /*uV*/
			hbp_dev->hw.avdd_volt.max = 3100000; /*uV*/
		}
		hbp_info("set avdd volt min %d, max %d\n",
			 hbp_dev->hw.avdd_volt.min, hbp_dev->hw.avdd_volt.max);
		ret = regulator_set_voltage(hbp_dev->hw.avdd_reg,
					    hbp_dev->hw.avdd_volt.min,
					    hbp_dev->hw.avdd_volt.max);
		if (ret < 0) {
			hbp_err("failed to set voltage of avdd %d\n", ret);
			return ret;
		}

		ret = regulator_set_load(hbp_dev->hw.avdd_reg, 200000); /*uA*/
		if (ret < 0) {
			hbp_err("failed to set load of avdd %d\n", ret);
			return ret;
		}
	}

	return 0;
}

int hbp_init_vddi(struct device *dev, struct hbp_device *hbp_dev)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

	/* for vddi control init, instead of regulator_get *
	** which may return dummy regulator */
	hbp_dev->hw.vddi_reg = regulator_get_optional(dev, "power,vddi");
	if (IS_ERR_OR_NULL(hbp_dev->hw.vddi_reg)) {
		hbp_info("not support vddi, or dummy device\n");
	} else {
		ret = of_property_read_u32_array(np,
						 "power,vddi_volt",
						 (uint32_t *)&hbp_dev->hw.vddi_volt, 2);
		if (ret) {
			hbp_dev->hw.vddi_volt.min = 1800000; /*uV*/
			hbp_dev->hw.vddi_volt.max = 1800000; /*uV*/
		}

		hbp_info("set vddi volt min %d, max %d\n",
			 hbp_dev->hw.vddi_volt.min, hbp_dev->hw.vddi_volt.max);
		ret = regulator_set_voltage(hbp_dev->hw.vddi_reg,
					    hbp_dev->hw.vddi_volt.min,
					    hbp_dev->hw.vddi_volt.max);
		if (ret < 0) {
			hbp_err("failed to set voltage of vddi %d\n", ret);
			return ret;
		}

		ret = regulator_set_load(hbp_dev->hw.vddi_reg, 200000); /*uA*/
		if (ret < 0) {
			hbp_err("failed to set load of avdd %d\n", ret);
			return ret;
		}
	}

	return 0;
}

int hbp_init_power(struct device *dev, struct hbp_device *hbp_dev)
{
	int ret = 0;
	hbp_info("%s start.\n", dev->of_node->name);

	/*for avdd control init, instead of regulator_get which may return dummy regulator*/
	ret = hbp_init_avdd(dev, hbp_dev);
	if (ret < 0) {
		goto exit;
	}

	/* for vddi control init, instead of regulator_get *
	** which may return dummy regulator */
	ret = hbp_init_vddi(dev, hbp_dev);
	if (ret < 0) {
		goto exit;
	}

	return 0;

exit:
	return ret;
}

static int hbp_power_ctrl_avdd(struct hbp_device *hbp_dev, bool on)
{
	int ret = -ENODEV;

	if (!IS_ERR_OR_NULL(hbp_dev->hw.avdd_reg)) {
		if (on) {
			ret = regulator_enable(hbp_dev->hw.avdd_reg);
		} else {
			ret = regulator_disable(hbp_dev->hw.avdd_reg);
		}
	}

	if (on) {
		if (!IS_ERR_OR_NULL(hbp_dev->hw.avdd_on)) {
			pinctrl_select_state(hbp_dev->hw.pctrl, hbp_dev->hw.avdd_on);
		}
	} else {
		if (!IS_ERR_OR_NULL(hbp_dev->hw.avdd_off)) {
			pinctrl_select_state(hbp_dev->hw.pctrl, hbp_dev->hw.avdd_off);
		}
	}

	return ret;
}

static int hbp_power_ctrl_vddi(struct hbp_device *hbp_dev, bool on)
{
	int ret = -ENODEV;

	if (!IS_ERR_OR_NULL(hbp_dev->hw.vddi_reg)) {
		if (on) {
			ret = regulator_enable(hbp_dev->hw.vddi_reg);
		} else {
			ret = regulator_disable(hbp_dev->hw.vddi_reg);
		}
	}

	if (on) {
		if (!IS_ERR_OR_NULL(hbp_dev->hw.vddi_on)) {
			pinctrl_select_state(hbp_dev->hw.pctrl, hbp_dev->hw.vddi_on);
		}
	} else {
		if (!IS_ERR_OR_NULL(hbp_dev->hw.vddi_off)) {
			pinctrl_select_state(hbp_dev->hw.pctrl, hbp_dev->hw.vddi_off);
		}
	}

	return ret;
}

static void hbp_power_ctrl_reset(struct hbp_device *hbp_dev, bool on)
{
	if (on) {
		if (!IS_ERR_OR_NULL(hbp_dev->hw.reset_active)) {
			pinctrl_select_state(hbp_dev->hw.pctrl, hbp_dev->hw.reset_active);
		}
	} else {
		if (!IS_ERR_OR_NULL(hbp_dev->hw.reset_idle)) {
			pinctrl_select_state(hbp_dev->hw.pctrl, hbp_dev->hw.reset_idle);
		}
	}
}

static void hbp_power_ctrl_bus(struct hbp_device *hbp_dev, bool on)
{

	if (on) {
		if (!IS_ERR_OR_NULL(hbp_dev->hw.resume)) {
			pinctrl_select_state(hbp_dev->hw.pctrl, hbp_dev->hw.resume);
		}
	} else {
		if (!IS_ERR_OR_NULL(hbp_dev->hw.suspend)) {
			pinctrl_select_state(hbp_dev->hw.pctrl, hbp_dev->hw.suspend);
		}
	}
}

void hbp_power_ctrl(struct hbp_device *hbp_dev, struct power_sequeue sq[])
{
	int i = 0;

	for (i = 0; i < MAX_POWER_SEQ; i++) {
		hbp_info("power type:0x%x en:%d delay:%dms\n", sq[i].type, sq[i].en, sq[i].msleep);
		switch (sq[i].type) {
		case POWER_AVDD:
			hbp_power_ctrl_avdd(hbp_dev, sq[i].en);
			break;
		case POWER_VDDI:
			hbp_power_ctrl_vddi(hbp_dev, sq[i].en);
			break;
		case POWER_RESET:
			hbp_power_ctrl_reset(hbp_dev, sq[i].en);
			break;
		case POWER_BUS:
			hbp_power_ctrl_bus(hbp_dev, sq[i].en);
			break;
		default:
			return;
		}

		if (sq[i].msleep) {
			msleep(sq[i].msleep);
		}
	}
}

void hbp_power_type_ctrl(struct hbp_device *hbp_dev, enum power_type type, bool en)
{
	hbp_info("power type:0x%x en:%d\n", type, en);
	switch (type) {
	case POWER_AVDD:
		hbp_power_ctrl_avdd(hbp_dev, en);
		break;
	case POWER_VDDI:
		hbp_power_ctrl_vddi(hbp_dev, en);
		break;
	case POWER_RESET:
		hbp_power_ctrl_reset(hbp_dev, en);
		break;
	case POWER_BUS:
		hbp_power_ctrl_bus(hbp_dev, en);
		break;
	default:
		return;
	}
}

