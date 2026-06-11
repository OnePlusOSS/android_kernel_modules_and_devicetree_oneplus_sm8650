/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_dischg_boost.h
** Description: dischg boost
** Date: 2025-11-01
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#ifndef __OPLUS_DISCHG_BOOST_H__
#define __OPLUS_DISCHG_BOOST_H__

#define RATIO_500_THR 500
#define RATIO_550_THR 550

#define INIT_CV_MV 3300
#define PLUGIN_CV_MV 3300
#define PLUGOUT_CV_MV 3300
#define SUSPEND_CV_MV 3300
#define RESUME_CV_MV 3300
#define NORMALCHG_CV_MV 3300
#define FASTCHG_CV_MV 3300

enum boost_ic_type {
	HYBRID_BOOST,
	RG_CP,
};

enum dischg_boost_topic_item {
	DISCHG_BOOST_ITEM_IC_TYPE,
	DISCHG_BOOST_ITEM_DEV_ID,
};

void oplus_boost_cv_mv_store(struct oplus_mms *topic, int val);
int oplus_boost_cv_mv_show(struct oplus_mms *topic);
void oplus_boost_disable_auto_mode_store(struct oplus_mms *topic, int val);
void oplus_boost_set_otg_mode(struct oplus_mms *topic, bool en);
bool oplus_boost_get_cv_mode(struct oplus_mms *topic);

#endif /* __OPLUS_DISCHG_BOOST_H__ */
