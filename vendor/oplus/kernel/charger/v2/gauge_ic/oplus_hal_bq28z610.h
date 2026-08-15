/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_bq28z610.c
** Description: bq28z610 ic
** Date: 2025-11-20
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#ifndef __OPLUS_BQ28Z610_RA_H__
#define __OPLUS_BQ28Z610_RA_H__

#include "oplus_hal_bq27541.h"


#ifdef CONFIG_OPLUS_GAUGE_BQ28Z610_RA
void bq28z610_parse_fcc_ra_dt(struct chip_bq27541 *chip);
int bq28z610_fcc_ra0_init(struct chip_bq27541 *chip);
int bq28z610_fcc_vdelta_init(struct chip_bq27541 *chip);
int bq28z610_delta_volt_clear(struct chip_bq27541 *chip, bool init);
void bq28z610_fcc_ra_t_check(struct chip_bq27541 *chip);
void bq27541_track_fcc_ra0_work(struct work_struct *work);
void bq27541_track_fcc_vdelta_work(struct work_struct *work);
void bq27541_track_fcc_ra_t_work(struct work_struct *work);
void bq28z610_fcc_init_cc1(struct chip_bq27541 *chip);
void bq27541_create_device_node(struct device *dev);
#else
inline void bq28z610_parse_fcc_ra_dt(struct chip_bq27541 *chip)
{
}
inline int bq28z610_fcc_ra0_init(struct chip_bq27541 *chip)
{
	return 0;
}
inline int bq28z610_fcc_vdelta_init(struct chip_bq27541 *chip)
{
	return 0;
}
inline int bq28z610_delta_volt_clear(struct chip_bq27541 *chip, bool init)
{
	return 0;
}
inline void bq28z610_fcc_ra_t_check(struct chip_bq27541 *chip)
{
}
inline void bq27541_track_fcc_ra0_work(struct work_struct *work)
{
}
inline void bq27541_track_fcc_vdelta_work(struct work_struct *work)
{
}
inline void bq27541_track_fcc_ra_t_work(struct work_struct *work)
{
}
inline void bq28z610_fcc_init_cc1(struct chip_bq27541 *chip)
{
}
inline void bq27541_create_device_node(struct device *dev)
{
}
#endif
#endif /* __OPLUS_BQ28Z610_RA_H__ */
