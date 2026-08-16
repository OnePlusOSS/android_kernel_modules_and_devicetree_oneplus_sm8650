/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_hal_sh366002.h
** Description: gauge ic
** Date: 2025-11-18
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#ifndef __OPLUS_SH366002_H__
#define __OPLUS_SH366002_H__

#include "oplus_hal_bq27541.h"

#ifdef CONFIG_OPLUS_GAUGE_SH366002
void oplus_sh36002_check_imp_model(struct chip_bq27541 *chip);
#else
void inline oplus_sh36002_check_imp_model(struct chip_bq27541 *chip)
{
}
#endif

#endif /* __OPLUS_SH366002_H__ */
