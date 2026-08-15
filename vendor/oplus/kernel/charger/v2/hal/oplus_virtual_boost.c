/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_virtual_boost.c
** Description: virtual boost
** Date: 2025-11-01
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#define pr_fmt(fmt) "[VIRTUAL_BOOST]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/iio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/of_irq.h>

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>

struct oplus_virtual_boost_child {
	struct oplus_chg_ic_dev *ic_dev;
	int index;
	enum oplus_chg_ic_func *funcs;
	int func_num;
	enum oplus_chg_ic_virq_id *virqs;
	int virq_num;

	bool initialized;
};

struct oplus_virtual_boost_ic {
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	int child_num;
	struct oplus_virtual_boost_child *child_list;

	struct work_struct data_handler_work;
};

static int oplus_dischg_vb_virq_register(struct oplus_virtual_boost_ic *chip);

static inline bool func_is_support(struct oplus_virtual_boost_child *ic,
				   enum oplus_chg_ic_func func_id)
{
	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
	case OPLUS_IC_FUNC_EXIT:
		return true; /* must support */
	default:
		break;
	}

	if (ic->func_num > 0)
		return oplus_chg_ic_func_check_support_by_table(ic->funcs, ic->func_num, func_id);
	else
		return false;
}

static inline bool virq_is_support(struct oplus_virtual_boost_child *ic,
				   enum oplus_chg_ic_virq_id virq_id)
{
	switch (virq_id) {
	case OPLUS_IC_VIRQ_ERR:
	case OPLUS_IC_VIRQ_ONLINE:
	case OPLUS_IC_VIRQ_OFFLINE:
		return true; /* must support */
	default:
		break;
	}

	if (ic->virq_num > 0)
		return oplus_chg_ic_virq_check_support_by_table(ic->virqs, ic->virq_num, virq_id);
	else
		return false;
}

static int oplus_dischg_vb_child_funcs_init(struct oplus_virtual_boost_ic *chip,
					 int child_num)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *func_node = NULL;
	int i;
	int m;
	int rc = 0;

	for (i = 0; i < child_num; i++) {
		func_node = of_parse_phandle(node, "oplus,boost_ic_func_group", i);
		if (func_node == NULL) {
			chg_err("can't get ic[%d] function group\n", i);
			rc = -ENODATA;
			goto err;
		}
		rc = of_property_count_elems_of_size(func_node, "functions", sizeof(u32));
		if (rc < 0) {
			chg_err("can't get ic[%d] functions size, rc=%d\n", i, rc);
			goto err;
		}
		chip->child_list[i].func_num = rc;
		chip->child_list[i].funcs = devm_kzalloc(
			chip->dev,
			sizeof(enum oplus_chg_ic_func) * chip->child_list[i].func_num,
			GFP_KERNEL);
		if (chip->child_list[i].funcs == NULL) {
			rc = -ENOMEM;
			chg_err("alloc child ic funcs memory error\n");
			goto err;
		}
		rc = of_property_read_u32_array(
			func_node, "functions",
			(u32 *)chip->child_list[i].funcs,
			chip->child_list[i].func_num);
		if (rc) {
			i++;
			chg_err("can't get ic[%d] functions, rc=%d\n", i, rc);
			goto err;
		}
		(void)oplus_chg_ic_func_table_sort(
			chip->child_list[i].funcs,
			chip->child_list[i].func_num);
	}

	return 0;

err:
	for (m = i; m > 0; m--)
		devm_kfree(chip->dev, chip->child_list[m - 1].funcs);
	return rc;
}

static int oplus_dischg_vb_child_virqs_init(struct oplus_virtual_boost_ic *chip,
					 int child_num)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *virq_node = NULL;
	int i;
	int m;
	int rc = 0;

	for (i = 0; i < child_num; i++) {
		virq_node = of_parse_phandle(node, "oplus,boost_ic_func_group", i);
		if (virq_node == NULL) {
			chg_err("can't get ic[%d] function group\n", i);
			rc = -ENODATA;
			goto err;
		}
		rc = of_property_count_elems_of_size(virq_node, "virqs", sizeof(u32));
		if (rc <= 0) {
			chip->child_list[i].virq_num = 0;
			chip->child_list[i].virqs = NULL;
			continue;
		}
		chip->child_list[i].virq_num = rc;
		chip->child_list[i].virqs = devm_kzalloc(
			chip->dev,
			sizeof(enum oplus_chg_ic_func) * chip->child_list[i].virq_num,
			GFP_KERNEL);
		if (chip->child_list[i].virqs == NULL) {
			rc = -ENOMEM;
			chg_err("alloc child ic virqs memory error\n");
			goto err;
		}
		rc = of_property_read_u32_array(
			virq_node, "virqs", (u32 *)chip->child_list[i].virqs,
			chip->child_list[i].virq_num);
		if (rc) {
			i++;
			chg_err("can't get ic[%d] virqs, rc=%d\n", i, rc);
			goto err;
		}
		(void)oplus_chg_ic_irq_table_sort(chip->child_list[i].virqs, chip->child_list[i].virq_num);
	}

	return 0;

err:
	for (m = i; m > 0; m--) {
		if (chip->child_list[m - 1].virqs != NULL)
			devm_kfree(chip->dev, chip->child_list[m - 1].virqs);
	}
	return rc;
}

static int oplus_dischg_vb_child_init(struct oplus_virtual_boost_ic *chip)
{
	struct device_node *node = chip->dev->of_node;
	int i;
	int rc = 0;

	rc = of_property_count_elems_of_size(node, "oplus,boost_ic", sizeof(u32));
	if (rc < 0) {
		chg_err("can't get boost ic number, rc=%d\n", rc);
		return rc;
	}
	chip->child_num = rc;
	chip->child_list = devm_kzalloc(
		chip->dev,
		sizeof(struct oplus_virtual_boost_child) * chip->child_num,
		GFP_KERNEL);
	if (chip->child_list == NULL) {
		rc = -ENOMEM;
		chg_err("alloc child ic memory error\n");
		return rc;
	}

	for (i = 0; i < chip->child_num; i++) {
		chip->child_list[i].ic_dev = of_get_oplus_chg_ic(node, "oplus,boost_ic", i);
		if (chip->child_list[i].ic_dev == NULL) {
			chg_debug("not find boost ic %d\n", i);
			rc = -EAGAIN;
			goto read_property_err;
		}
	}

	rc = oplus_dischg_vb_child_funcs_init(chip, chip->child_num);
	if (rc < 0)
		goto child_funcs_init_err;
	rc = oplus_dischg_vb_child_virqs_init(chip, chip->child_num);
	if (rc < 0)
		goto child_virqs_init_err;

	return 0;

child_virqs_init_err:
	for (i = 0; i < chip->child_num; i++)
		devm_kfree(chip->dev, chip->child_list[i].funcs);
child_funcs_init_err:
read_property_err:
	for (; i >=0; i--)
		chip->child_list[i].ic_dev = NULL;
	devm_kfree(chip->dev, chip->child_list);
	return rc;
}

static int oplus_dischg_vb_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int m;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	vb = oplus_chg_ic_get_drvdata(ic_dev);

	if (ic_dev->online)
		return 0;

	rc = oplus_dischg_vb_child_init(vb);
	if (rc < 0) {
		chg_err("child list init error, rc=%d\n", rc);
		goto child_list_init_err;
	}

	rc = oplus_dischg_vb_virq_register(vb);
	if (rc < 0) {
		chg_err("virq register error, rc=%d\n", rc);
		goto virq_register_err;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_INIT);
		if (rc < 0) {
			chg_err("child ic[%d] init error, rc=%d\n", i, rc);
			goto child_init_err;
		}
		oplus_chg_ic_set_parent(vb->child_list[i].ic_dev, ic_dev);
	}

	ic_dev->online = true;

	return 0;

child_init_err:
	for (m = i + 1; m > 0; m--)
		oplus_chg_ic_func(vb->child_list[m - 1].ic_dev, OPLUS_IC_FUNC_EXIT);
virq_register_err:
child_list_init_err:

	return rc;
}

static int oplus_dischg_vb_exit(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_EXIT);
		if (rc < 0)
			chg_err("child ic[%d] exit error, rc=%d\n", i, rc);
	}
	for (i = 0; i < vb->child_num; i++) {
		if (virq_is_support(&vb->child_list[i], OPLUS_IC_VIRQ_ERR)) {
			oplus_chg_ic_virq_release(vb->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_ERR, vb);
		}
		if (virq_is_support(&vb->child_list[i], OPLUS_IC_VIRQ_ONLINE)) {
			oplus_chg_ic_virq_release(vb->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_ONLINE, vb);
		}
		if (virq_is_support(&vb->child_list[i], OPLUS_IC_VIRQ_OFFLINE)) {
			oplus_chg_ic_virq_release(vb->child_list[i].ic_dev,
				OPLUS_IC_VIRQ_OFFLINE, vb);
		}
	}
	for (i = 0; i < vb->child_num; i++) {
		if (vb->child_list[i].virqs != NULL)
			devm_kfree(vb->dev, vb->child_list[i].virqs);
	}
	for (i = 0; i < vb->child_num; i++)
		devm_kfree(vb->dev, vb->child_list[i].funcs);

	return 0;
}

static int oplus_dischg_vb_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i], OPLUS_IC_FUNC_REG_DUMP)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev, OPLUS_IC_FUNC_REG_DUMP);
		if (rc < 0)
			chg_err("child ic[%d] reg_dump error, rc=%d\n", i, rc);
	}

	return rc;
}

static int oplus_dischg_vb_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int index = 0;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (index >= len)
			return len;
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_SMT_TEST, buf + index, len - index);
		if (rc < 0) {
			if (rc != -ENOTSUPP) {
				chg_err("child ic[%d] smt test error, rc=%d\n", i, rc);
				rc = snprintf(buf + index, len - index,
					"[%s]-[%s]:%d\n",
					vb->child_list[i].ic_dev->manu_name,
					"FUNC_ERR", rc);
			} else {
				rc = 0;
			}
		} else {
			if ((rc > 0) && buf[index + rc - 1] != '\n') {
				buf[index + rc] = '\n';
				index++;
			}
		}
		index += rc;
	}

	return index;
}

static int oplus_dischg_vb_boost_set_cv(struct oplus_chg_ic_dev *ic_dev, int vol)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BOOST_SET_CV)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BOOST_SET_CV, vol);
		if (rc < 0)
			chg_err("child ic[%d] boost set_cv error, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

static int oplus_dischg_vb_boost_enable_otg_mode(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BOOST_ENABLE_OTG_MODE)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BOOST_ENABLE_OTG_MODE, en);
		if (rc < 0)
			chg_err("child ic[%d] boost set_otg_mode_enable error, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

static int oplus_dischg_vb_boost_set_work_mode(struct oplus_chg_ic_dev *ic_dev, int mode)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BOOST_SET_WORK_MODE)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BOOST_SET_WORK_MODE, mode);
		if (rc < 0)
			chg_err("child ic[%d] boost set_work_mode error, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

static int oplus_dischg_vb_boost_set_bcl_rate(struct oplus_chg_ic_dev *ic_dev, int rate)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BOOST_SET_BCL_RATE)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BOOST_SET_BCL_RATE, rate);
		if (rc < 0)
			chg_err("child ic[%d] boost set_bcl_rate error, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

static int oplus_dischg_vb_boost_set_bcl_vol(struct oplus_chg_ic_dev *ic_dev, int vol0, int vol1, int vol2)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BOOST_SET_BCL_VOL)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BOOST_SET_BCL_VOL, vol0, vol1, vol2);
		if (rc < 0)
			chg_err("child ic[%d] set_bcl_vol error, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

static int oplus_dischg_vb_boost_get_in_cv_mode(struct oplus_chg_ic_dev *ic_dev, bool *cv_mode)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BOOST_GET_IN_CV_MODE)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BOOST_GET_IN_CV_MODE, cv_mode);
		if (rc < 0)
			chg_err("child ic[%d] get in cv mode error, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

static int oplus_dischg_vb_boost_get_cv(struct oplus_chg_ic_dev *ic_dev, int *cv)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BOOST_GET_CV)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BOOST_GET_CV, cv);
		if (rc < 0)
			chg_err("child ic[%d] get cv error, rc=%d\n", i, rc);
		break;
	}
	chg_info("cv = %d\n", *cv);

	return rc;
}

static int oplus_dischg_vb_boost_set_fam_en(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BOOST_SET_FAM_EN)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BOOST_SET_FAM_EN, en);
		if (rc < 0)
			chg_err("child ic[%d] set_fam_en error, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

static int oplus_dischg_vb_boost_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	if (suspend == NULL) {
		chg_err("suspend pointer is NULL\n");
		return -EINVAL;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BOOST_IS_SUSPEND)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BOOST_IS_SUSPEND, suspend);
		if (rc < 0)
			chg_err("child ic[%d] is_suspend error, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

static int oplus_dischg_vb_boost_set_suspend_resume_cv(struct oplus_chg_ic_dev *ic_dev, int suspend_cv, int resume_cv)
{
	struct oplus_virtual_boost_ic *vb;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}

	vb = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < vb->child_num; i++) {
		if (!func_is_support(&vb->child_list[i],
				     OPLUS_IC_FUNC_BOOST_SET_SUSPEND_RESUME_CV)) {
			rc = (rc == 0) ? -ENOTSUPP : rc;
			continue;
		}
		rc = oplus_chg_ic_func(vb->child_list[i].ic_dev,
			OPLUS_IC_FUNC_BOOST_SET_SUSPEND_RESUME_CV, suspend_cv, resume_cv);
		if (rc < 0)
			chg_err("child ic[%d] set suspend/resume cv error, rc=%d\n", i, rc);
		break;
	}

	return rc;
}

static void *oplus_dischg_vb_get_func(struct oplus_chg_ic_dev *ic_dev,
				   enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
			    oplus_dischg_vb_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
			    oplus_dischg_vb_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
			    oplus_dischg_vb_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
			    oplus_dischg_vb_smt_test);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_CV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_CV,
			    oplus_dischg_vb_boost_set_cv);
		break;
	case OPLUS_IC_FUNC_BOOST_ENABLE_OTG_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_ENABLE_OTG_MODE,
			    oplus_dischg_vb_boost_enable_otg_mode);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_WORK_MODE,
			    oplus_dischg_vb_boost_set_work_mode);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_BCL_RATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_BCL_RATE,
			    oplus_dischg_vb_boost_set_bcl_rate);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_BCL_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_BCL_VOL,
			    oplus_dischg_vb_boost_set_bcl_vol);
		break;
	case OPLUS_IC_FUNC_BOOST_GET_IN_CV_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_GET_IN_CV_MODE,
			    oplus_dischg_vb_boost_get_in_cv_mode);
		break;
	case OPLUS_IC_FUNC_BOOST_GET_CV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_GET_CV,
				oplus_dischg_vb_boost_get_cv);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_FAM_EN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_FAM_EN,
				oplus_dischg_vb_boost_set_fam_en);
		break;
	case OPLUS_IC_FUNC_BOOST_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_IS_SUSPEND,
				oplus_dischg_vb_boost_is_suspend);
		break;
	case OPLUS_IC_FUNC_BOOST_SET_SUSPEND_RESUME_CV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BOOST_SET_SUSPEND_RESUME_CV,
				oplus_dischg_vb_boost_set_suspend_resume_cv);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static struct oplus_chg_ic_virq oplus_dischg_vb_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static void oplus_dischg_vb_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_boost_ic *chip = virq_data;

	oplus_chg_ic_move_err_msg(chip->ic_dev, ic_dev);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
}

static void oplus_dischg_vb_online_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_boost_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ONLINE);
}

static void oplus_dischg_vb_offline_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_boost_ic *chip = virq_data;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_OFFLINE);
}

static int oplus_dischg_vb_virq_register(struct oplus_virtual_boost_ic *chip)
{
	int i;
	int rc;

	for (i = 0; i < chip->child_num; i++) {
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_ERR)) {
			rc = oplus_chg_ic_virq_register(
				chip->child_list[i].ic_dev, OPLUS_IC_VIRQ_ERR,
				oplus_dischg_vb_err_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_ONLINE)) {
			rc = oplus_chg_ic_virq_register(
				chip->child_list[i].ic_dev, OPLUS_IC_VIRQ_ONLINE,
				oplus_dischg_vb_online_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);
		}
		if (virq_is_support(&chip->child_list[i], OPLUS_IC_VIRQ_OFFLINE)) {
			rc = oplus_chg_ic_virq_register(
				chip->child_list[i].ic_dev, OPLUS_IC_VIRQ_OFFLINE,
				oplus_dischg_vb_offline_handler, chip);
			if (rc < 0)
				chg_err("register OPLUS_IC_VIRQ_OFFLINE error, rc=%d", rc);
		}
	}

	return 0;
}

static int oplus_virtual_boost_probe(struct platform_device *pdev)
{
	struct oplus_virtual_boost_ic *chip;
	struct device_node *node = pdev->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int rc = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_virtual_boost_ic), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	rc = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		goto reg_ic_err;
	}
	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto reg_ic_err;
	}
	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "boost-virtual");
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_dischg_vb_get_func;
	ic_cfg.virq_data = oplus_dischg_vb_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(oplus_dischg_vb_virq_table);
	ic_cfg.of_node = node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto reg_ic_err;
	}

	chg_info("probe success\n");
	return 0;

reg_ic_err:
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

	chg_err("probe error\n");
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_virtual_boost_remove(struct platform_device *pdev)
#else
static int oplus_virtual_boost_remove(struct platform_device *pdev)
#endif
{
	struct oplus_virtual_boost_ic *chip = platform_get_drvdata(pdev);

	if (chip == NULL) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
		return -ENODEV;
#else
		return;
#endif
	}

	if (chip->ic_dev->online)
		oplus_dischg_vb_exit(chip->ic_dev);
	devm_oplus_chg_ic_unregister(&pdev->dev, chip->ic_dev);
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_virtual_boost_match[] = {
	{ .compatible = "oplus,virtual_boost" },
	{},
};

static struct platform_driver oplus_virtual_boost_driver = {
	.driver		= {
		.name = "oplus-virtual_boost",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_virtual_boost_match),
	},
	.probe		= oplus_virtual_boost_probe,
	.remove		= oplus_virtual_boost_remove,
};

static __init int oplus_virtual_boost_init(void)
{
	return platform_driver_register(&oplus_virtual_boost_driver);
}

static __exit void oplus_virtual_boost_exit(void)
{
	platform_driver_unregister(&oplus_virtual_boost_driver);
}

oplus_chg_module_register(oplus_virtual_boost);
