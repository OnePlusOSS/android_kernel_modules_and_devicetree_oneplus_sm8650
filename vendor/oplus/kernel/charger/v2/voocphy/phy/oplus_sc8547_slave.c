/*
 * SC8547 battery charging driver
*/

#define pr_fmt(fmt)	"[sc8547] %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/proc_fs.h>
#include <trace/events/sched.h>
#include<linux/ktime.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include "../oplus_voocphy.h"
#include "oplus_sc8547.h"
#include <oplus_chg_voter.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_vooc.h>
#define DEFAULT_OVP_REG_CONFIG	0x2E
#define DEFAULT_OCP_REG_CONFIG	0x8
#define ERR_MSG_BUF	PAGE_SIZE

static struct oplus_voocphy_manager *oplus_voocphy_mg = NULL;
static struct mutex i2c_rw_lock;
static bool error_reported = false;
extern void oplus_chg_sc8547_error(int report_flag, int *buf, int len);
static int sc8547_slave_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data);
static bool ic_sc8547a = false;
static int slave_ovp_reg = DEFAULT_OVP_REG_CONFIG;
static int slave_ocp_reg = DEFAULT_OCP_REG_CONFIG;
static u8 vbus_ovp_reg;
static u8 vooc_vbus_ovp_reg;

#define I2C_ERR_NUM 10
#define SLAVE_I2C_ERROR (1 << 1)

static enum oplus_cp_work_mode g_cp_support_work_mode[] = {
	CP_WORK_MODE_BYPASS,
	CP_WORK_MODE_2_TO_1,
};

struct sc8547a_slave_device {
	struct device *slave_dev;
	struct i2c_client *slave_client;
	struct oplus_voocphy_manager *voocphy;
	struct oplus_chg_ic_dev *cp_ic;

	struct votable *disable_votable;

	enum oplus_cp_work_mode cp_work_mode;
	bool rested;
	bool work_start;
	bool boot_online;
	atomic_t i2c_err_count;

	struct work_struct cp_regdump_work;
	struct work_struct ic_offline_work;
};

static void sc8547_slave_i2c_error(struct oplus_voocphy_manager *voocphy, bool happen, bool read)
{
	struct sc8547a_slave_device *device;
	struct oplus_voocphy_manager *chip;

	if (!voocphy || !voocphy->slave_client) {
		chg_err("voocphy_manager is null\n");
		return;
	}

	chip = i2c_get_clientdata(voocphy->slave_client);

	if (!chip) {
		pr_err("chip is null exit\n");
		return;
	}

	device = chip->priv_data;

	if (happen) {
		if(error_reported)
			return;
		if (atomic_read(&device->i2c_err_count) < I2C_ERR_NUM) {
			atomic_inc(&device->i2c_err_count);
			return;
		}
		error_reported = true;
		vote(device->disable_votable, IIC_VOTER, true, 1, false);
		oplus_chg_ic_creat_err_msg(device->cp_ic, OPLUS_IC_ERR_CP,
					   CP_ERR_I2C, "%s error",
					   read ? "read" : "write");
		oplus_chg_ic_err_trigger_and_clean(device->cp_ic);
	} else {
		vote(device->disable_votable, IIC_VOTER, false, 0, false);
		error_reported = false;
		atomic_set(&device->i2c_err_count, 0);
	}
}


/************************************************************************/
static int __sc8547_slave_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	s32 ret;
	struct oplus_voocphy_manager *chip = i2c_get_clientdata(client);

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		sc8547_slave_i2c_error(chip, true, true);
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}
	sc8547_slave_i2c_error(chip, false, true);
	*data = (u8) ret;

	return 0;
}

static int __sc8547_slave_write_byte(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;
	struct oplus_voocphy_manager *chip = i2c_get_clientdata(client);

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		sc8547_slave_i2c_error(chip, true, false);
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	sc8547_slave_i2c_error(chip, false, false);

	return 0;
}

static int sc8547_slave_read_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&i2c_rw_lock);
	ret = __sc8547_slave_read_byte(client, reg, data);
	mutex_unlock(&i2c_rw_lock);

	return ret;
}

static int sc8547_slave_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&i2c_rw_lock);
	ret = __sc8547_slave_write_byte(client, reg, data);
	mutex_unlock(&i2c_rw_lock);

	return ret;
}


static int sc8547_slave_update_bits(struct i2c_client *client, u8 reg,
                                    u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&i2c_rw_lock);
	ret = __sc8547_slave_read_byte(client, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sc8547_slave_write_byte(client, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
out:
	mutex_unlock(&i2c_rw_lock);
	return ret;
}

static void sc8547_slave_update_data(struct oplus_voocphy_manager *chip)
{
	u8 data_block[2] = {0};
	int i = 0;
	u8 int_flag = 0;
	s32 ret = 0;

	sc8547_slave_read_byte(chip->slave_client, SC8547_REG_0F, &int_flag);

	/*parse data_block for improving time of interrupt*/
	ret = i2c_smbus_read_i2c_block_data(chip->slave_client, SC8547_REG_13, 2, data_block);
	if (ret < 0) {
		sc8547_slave_i2c_error(chip, true, true);
		pr_err("sc8547_update_data slave read error \n");
	} else {
		sc8547_slave_i2c_error(chip, false, true);
	}
	for (i=0; i<2; i++) {
		pr_info("data_block[%d] = %u\n", i, data_block[i]);
	}
	chip->slave_cp_ichg = ((data_block[0] << 8) | data_block[1])*1875 / 1000;
	if (oplus_voocphy_mg)
		oplus_voocphy_mg->slave_cp_ichg = chip->slave_cp_ichg;
	pr_info("slave cp_ichg = %d int_flag = %d", chip->slave_cp_ichg, int_flag);
}
/*********************************************************************/
static int oplus_chg_get_vooc_charging(void)
{
	int vooc_charging_status = 0;
	struct oplus_mms *vooc_topic;
	union mms_msg_data data = { 0 };
	int rc;

	vooc_topic = oplus_mms_get_by_name("vooc");
	if (!vooc_topic)
		return 0;

	rc = oplus_mms_get_item_data(vooc_topic, VOOC_ITEM_VOOC_CHARGING, &data, true);
	if (!rc)
		vooc_charging_status = data.intval;

	return vooc_charging_status;
}

int sc8547_slave_get_ichg(struct oplus_voocphy_manager *chip)
{
	u8 slave_cp_enable;

	if (oplus_chg_get_vooc_charging()) {
		if (oplus_voocphy_mg)
			return oplus_voocphy_mg->slave_cp_ichg;
	}

	sc8547_slave_update_data(chip);

	sc8547_slave_get_chg_enable(chip, &slave_cp_enable);
	if(chip->slave_ops) {
		if(slave_cp_enable == 1)
			return chip->slave_cp_ichg;
		else
			return 0;
	} else {
		return 0;
	}
}

static int sc8547_slave_get_cp_status(struct oplus_voocphy_manager *chip)
{
	u8 data_reg06, data_reg07;
	int ret_reg06, ret_reg07;

	if (!chip) {
		pr_err("Failed\n");
		return 0;
	}

	ret_reg06 = sc8547_slave_read_byte(chip->slave_client, SC8547_REG_06, &data_reg06);
	ret_reg07 = sc8547_slave_read_byte(chip->slave_client, SC8547_REG_07, &data_reg07);

	if (ret_reg06 < 0 || ret_reg07 < 0) {
		pr_err("SC8547_REG_06 or SC8547_REG_07 err\n");
		return 0;
	}
	data_reg06 = data_reg06 & SC8547_CP_SWITCHING_STAT_MASK;
	data_reg06 = data_reg06 >> 2;
	data_reg07 = data_reg07 >> 7;

	pr_err("reg06 = %d reg07 = %d\n", data_reg06, data_reg07);

	if (data_reg06 == 1 && data_reg07 == 1) {
		return 1;
	} else {
		return 0;
	}

}

static int sc8547_slave_reg_reset(struct oplus_voocphy_manager *chip, bool enable)
{
	int ret;
	u8 val;
	if (enable)
		val = SC8547_RESET_REG;
	else
		val = SC8547_NO_REG_RESET;

	val <<= SC8547_REG_RESET_SHIFT;

	ret = sc8547_slave_update_bits(chip->slave_client, SC8547_REG_07,
	                               SC8547_REG_RESET_MASK, val);

	return ret;
}

static void sc8547a_hw_version_check(struct oplus_voocphy_manager *chip)
{
	int ret;
	u8 val;
	ret = sc8547_slave_read_byte(chip->slave_client, SC8547_REG_36, &val);
	if (val == SC8547A_DEVICE_ID)
		ic_sc8547a = true;
	else
		ic_sc8547a = false;
}

static int sc8547_slave_get_chg_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	ret = sc8547_slave_read_byte(chip->slave_client, SC8547_REG_07, data);
	if (ret < 0) {
		pr_err("SC8547_REG_07\n");
		return -1;
	}
	*data = *data >> 7;

	return ret;
}

static int sc8547_slave_set_chg_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	u8 value = 0x85;
	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}
	if (enable && ic_sc8547a)
		value = 0x81;
	else if (enable && !ic_sc8547a)
		value = 0x85;
	else if (!enable && ic_sc8547a)
		value = 0x1;
	else
		value = 0x05;
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, value);
	pr_err(" enable  = %d, value = 0x%x!\n", enable, value);

	return 0;
}

static int sc8547_slave_get_adc_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	ret = sc8547_slave_read_byte(chip->slave_client, SC8547_REG_11, data);
	if (ret < 0) {
		pr_err("SC8547_REG_11\n");
		return -1;
	}

	*data = *data >> 7;

	return ret;
}

static int sc8547_slave_set_adc_enable(struct oplus_voocphy_manager *chip, bool enable)
{
	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	if (enable)
		return sc8547_slave_write_byte(chip->slave_client, SC8547_REG_11, 0x80);
	else
		return sc8547_slave_write_byte(chip->slave_client, SC8547_REG_11, 0x00);
}

static int sc8547_slave_get_voocphy_enable(struct oplus_voocphy_manager *chip, u8 *data)
{
	int ret = 0;

	if (!chip) {
		pr_err("Failed\n");
		return -1;
	}

	ret = sc8547_slave_read_byte(chip->slave_client, SC8547_REG_2B, data);
	if (ret < 0) {
		pr_err("SC8547_REG_2B\n");
		return -1;
	}

	return ret;
}

static void sc8547_slave_dump_reg_in_err_issue(struct oplus_voocphy_manager *chip)
{
	int i = 0, p = 0;
	//u8 value[DUMP_REG_CNT] = {0};
	if(!chip) {
		pr_err( "!!!!! oplus_voocphy_manager chip NULL");
		return;
	}

	for( i = 0; i < 37; i++) {
		p = p + 1;
		sc8547_slave_read_byte(chip->slave_client, i, &chip->slave_reg_dump[p]);
	}
	for( i = 0; i < 9; i++) {
		p = p + 1;
		sc8547_slave_read_byte(chip->slave_client, 43 + i, &chip->slave_reg_dump[p]);
	}
	p = p + 1;
	sc8547_slave_read_byte(chip->slave_client, SC8547_REG_36, &chip->slave_reg_dump[p]);
	p = p + 1;
	sc8547_slave_read_byte(chip->slave_client, SC8547_REG_3A, &chip->slave_reg_dump[p]);
	pr_err( "p[%d], ", p);

	///memcpy(chip->voocphy.reg_dump, value, DUMP_REG_CNT);
	return;
}

static int sc8547_slave_init_device(struct oplus_voocphy_manager *chip)
{
	u8 reg_data;

	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_11, 0x00); /* ADC_CTRL:disable */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_02, 0x01);
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_04, vbus_ovp_reg); /* VBUS_OVP:10 2:1 or 1:1V */
	reg_data = slave_ovp_reg & SC8547_BAT_OVP_MASK;
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_00, reg_data); /* VBAT_OVP:4.65V */
	if (!ic_sc8547a)
		reg_data = (SC8547_IBUS_UCP_FALL_DEGLITCH_SET_5MS << SC8547_IBUS_UCP_FALL_DEGLITCH_SET_SHIFT)
				| (slave_ocp_reg & SC8547_IBUS_OCP_MASK);
	else
		reg_data = (SC8547A_IBUS_UCP_FALL_DEGLITCH_SET_5MS << SC8547A_IBUS_UCP_FALL_DEGLITCH_SET_SHIFT)
				| (slave_ocp_reg & SC8547A_IBUS_OCP_MASK);
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_05, reg_data); /* IBUS_OCP_UCP:3.6A */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_2B, 0x00); /* VOOC_CTRL:disable */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_30, 0x7F);
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_3C, 0x40);
	if (!ic_sc8547a)
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x05);
	else
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x1);

	return 0;
}

static int sc8547_slave_reset_device(struct oplus_voocphy_manager *chip)
{
	u8 reg_data;

	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_11,
				0x00); /* ADC_CTRL:disable */
	/* The logic does not change when the main CP controls OVP */
	if (chip->ovp_ctrl_cpindex == MASTER_CP_ID)
		sc8547_slave_write_byte(chip->slave_client,
					SC8547_REG_02, 0x01);
	sc8547_slave_write_byte(chip->slave_client,
				SC8547_REG_04, vbus_ovp_reg); /* VBUS_OVP:10 2:1 or 1:1V */
	reg_data = slave_ovp_reg & SC8547_BAT_OVP_MASK;
	sc8547_slave_write_byte(chip->slave_client,
				SC8547_REG_00, reg_data); /* VBAT_OVP:4.65V */
	if (!ic_sc8547a) {
		reg_data = (SC8547_IBUS_UCP_FALL_DEGLITCH_SET_5MS << SC8547_IBUS_UCP_FALL_DEGLITCH_SET_SHIFT)
				| (slave_ocp_reg & SC8547_IBUS_OCP_MASK);
	} else {
		reg_data = (SC8547A_IBUS_UCP_FALL_DEGLITCH_SET_5MS << SC8547A_IBUS_UCP_FALL_DEGLITCH_SET_SHIFT)
				| (slave_ocp_reg & SC8547A_IBUS_OCP_MASK);
	}
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_05,
				reg_data); /* IBUS_OCP_UCP:3.6A  */
	sc8547_slave_write_byte(chip->slave_client,
				SC8547_REG_2B, 0x00); /* VOOC_CTRL:disable */
	sc8547_slave_write_byte(chip->slave_client,
				SC8547_REG_30, 0x7F);
	sc8547_slave_write_byte(chip->slave_client,
				SC8547_REG_3C, 0x40);

	/* bit2:0 Set the CP switching frequency 550kHz */
	if (!ic_sc8547a) {
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x05);
	} else {
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x1);
	}

	return 0;
}

/* When OVP control by CP, reset the OVP related registers at the end*/
static int sc8547_slave_reset_ovp(struct oplus_voocphy_manager *chip)
{
	if (chip->ovp_ctrl_cpindex == SLAVE_CP_ID) {
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_02, 0x01);
		pr_err("sc8547_slave_reset_ovp VAC_OVP to 6.5v\n");
	}
	return 0;
}

static int sc8547_slave_init_vooc(struct oplus_voocphy_manager *chip)
{
	pr_err("sc8547_slave_init_vooc\n");

	sc8547_slave_reg_reset(chip, true);
	sc8547_slave_init_device(chip);

	return 0;
}

static int sc8547_slave_svooc_hw_setting(struct oplus_voocphy_manager *chip)
{
	u8 reg_data;

	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_02, 0x01); /* VAC_OVP:12v */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_04, vbus_ovp_reg);  /* VBUS_OVP:10v */
	if (!ic_sc8547a)
		reg_data = (SC8547_IBUS_UCP_FALL_DEGLITCH_SET_5MS << SC8547_IBUS_UCP_FALL_DEGLITCH_SET_SHIFT)
				| (slave_ocp_reg & SC8547_IBUS_OCP_MASK);
	else
		reg_data = (SC8547A_IBUS_UCP_FALL_DEGLITCH_SET_5MS << SC8547A_IBUS_UCP_FALL_DEGLITCH_SET_SHIFT)
				| (slave_ocp_reg & SC8547A_IBUS_OCP_MASK);
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_05, reg_data); /* IBUS_OCP_UCP:3.6A */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_09, 0x03); /* WD:1000ms */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_3C, 0x40); /* VOOC_CTRL:disable */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_11, 0x80); /* ADC_CTRL:ADC_EN */
	if (!ic_sc8547a)
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x05);
	else
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x1);
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_0D, 0x70);

	return 0;
}

static int sc8547_slave_vooc_hw_setting(struct oplus_voocphy_manager *chip)
{
	u8 reg_data;

	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_02, 0x01); /* VAC_OVP */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_04, vooc_vbus_ovp_reg); /* ADC_CTRL */
	if (!ic_sc8547a)
		reg_data = 0x2c;
	else
		reg_data = 0x1c;
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_05, reg_data); /* IBUS_OCP_UCP */
	if (!ic_sc8547a)
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x05);
	else
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x1);
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_09, 0x83); /* WD:5000ms */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_11, 0x80); /* ADC_CTRL */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_2B, 0x00); /* VOOC_CTRL */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_3C, 0x40); /* VOOC_CTRL:disable */

	return 0;
}

static int sc8547_slave_5v2a_hw_setting(struct oplus_voocphy_manager *chip)
{
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_02, 0x01); /* VAC_OVP */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_04, vbus_ovp_reg); /* VBUS_OVP */
	if (!ic_sc8547a)
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x05);
	else
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x1);
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_09, 0x00); /* WD */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_11, 0x00); /* ADC_CTRL */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_2B, 0x00); /* VOOC_CTRL */

	return 0;
}

static int sc8547_slave_pdqc_hw_setting(struct oplus_voocphy_manager *chip)
{
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_02, 0x01); /* VAC_OVP */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_04, vbus_ovp_reg); /* VBUS_OVP */
	if (!ic_sc8547a)
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x05);
	else
		sc8547_slave_write_byte(chip->slave_client, SC8547_REG_07, 0x1);
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_09, 0x00); /* WD */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_11, 0x00); /* ADC_CTRL */
	sc8547_slave_write_byte(chip->slave_client, SC8547_REG_2B, 0x00); /* VOOC_CTRL */

	return 0;
}

static int sc8547_slave_hw_setting(struct oplus_voocphy_manager *chip, int reason)
{
	if (!chip) {
		pr_err("chip is null exit\n");
		return -1;
	}
	switch (reason) {
	case SETTING_REASON_PROBE:
		sc8547_slave_init_device(chip);
		pr_info("SETTING_REASON_PROBE\n");
		break;
	case SETTING_REASON_RESET:
		sc8547_slave_reset_device(chip);
		pr_info("SETTING_REASON_RESET\n");
		break;
	case SETTING_REASON_SVOOC:
		sc8547_slave_svooc_hw_setting(chip);
		pr_info("SETTING_REASON_SVOOC\n");
		break;
	case SETTING_REASON_VOOC:
		sc8547_slave_vooc_hw_setting(chip);
		pr_info("SETTING_REASON_VOOC\n");
		break;
	case SETTING_REASON_5V2A:
		sc8547_slave_5v2a_hw_setting(chip);
		pr_info("SETTING_REASON_5V2A\n");
		break;
	case SETTING_REASON_PDQC:
		sc8547_slave_pdqc_hw_setting(chip);
		pr_info("SETTING_REASON_PDQC\n");
		break;
	default:
		pr_err("do nothing\n");
		break;
	}
	return 0;
}

static int sc8547_slave_reset_voocphy(struct oplus_voocphy_manager *chip)
{
	sc8547_slave_set_chg_enable(chip, false);
	if (ic_sc8547a) {
		/* sc8547b need disable WDT when exit charging, to avoid after WDT time out.
		IF time out, sc8547b will trigger interrupt frequently.
		in addition, sc8547 and sc8547b WDT will disable when disable CP */
		sc8547_slave_update_bits(chip->slave_client, SC8547_REG_09,
				 	 SC8547_WATCHDOG_MASK, SC8547_WATCHDOG_DIS);
	}
	sc8547_slave_hw_setting(chip, SETTING_REASON_RESET);

	return VOOCPHY_SUCCESS;
}

static int sc8547_slave_get_cp_vbat(struct oplus_voocphy_manager *chip)
{
	u8 data_block[2] = { 0 };
	s32 ret = 0;

	/*parse data_block for improving time of interrupt*/
	ret = i2c_smbus_read_i2c_block_data(chip->slave_client, SC8547_REG_1B, 2,
					    data_block);
	if (ret < 0) {
		chg_err("sc8547 read vbat error \n");
		return 0;
	}

	return ((data_block[0] << 8) | data_block[1]) * SC8547_VBAT_ADC_LSB;
}

static int sc8547_slave_get_cp_vbus(struct oplus_voocphy_manager *chip)
{
	u8 data_block[2] = { 0 };
	s32 ret = 0;

	/* parse data_block for improving time of interrupt */
	ret = i2c_smbus_read_i2c_block_data(chip->slave_client, SC8547_REG_15, 2,
					    data_block);
	if (ret < 0) {
		chg_err("sc8547 read vbat error \n");
		return 0;
	}

	return (((data_block[0] & SC8547_VBUS_POL_H_MASK) << 8) | data_block[1]) * SC8547_VBUS_ADC_LSB;
}

static void sc8547_slave_hardware_init(struct oplus_voocphy_manager *chip)
{
	sc8547_slave_reg_reset(chip, true);
	sc8547_slave_init_device(chip);
}

static bool sc8547_slave_check_work_mode_support(enum oplus_cp_work_mode mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_cp_support_work_mode); i++) {
		if (g_cp_support_work_mode[i] == mode)
			return true;
	}
	return false;
}

static int sc8547_slave_cp_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8547a_slave_device *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	chg_info("%s init\n", ic_dev->manu_name);

	vote(chip->disable_votable, OCP_VOTER, false, 0, false);
	vote(chip->disable_votable, UCP_VOTER, false, 0, false);
	vote(chip->disable_votable, IIC_VOTER, false, 0, false);
	vote(chip->disable_votable, PARENT_VOTER, false, 0, false);
	error_reported = false;
	atomic_set(&chip->i2c_err_count, 0);

	return 0;
}

static int sc8547_slave_cp_exit(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8547a_slave_device *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);
	chg_info("%s exit\n", ic_dev->manu_name);
	vote(chip->disable_votable, PARENT_VOTER, true, 1, false);
	schedule_work(&chip->cp_regdump_work);

	return 0;
}

static int sc8547_slave_cp_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8547a_slave_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	sc8547_slave_dump_reg_in_err_issue(chip->voocphy);

	return 0;
}

static int sc8547_slave_cp_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int sc8547_slave_cp_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct sc8547a_slave_device *chip;
	int rc;
	u8 data;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (en) {
		sc8547_slave_reg_reset(chip->voocphy, true);
		sc8547_slave_init_device(chip->voocphy);
		sc8547_slave_write_byte(chip->voocphy->slave_client, SC8547_REG_02, 0x01); /*VAC_OVP:12V*/
		sc8547_slave_write_byte(chip->voocphy->slave_client, SC8547_REG_04, 0x71); /*VBUS_OVP:12V*/
		sc8547_slave_write_byte(chip->voocphy->slave_client, SC8547_REG_05, 0xcf); /*IBUS_OCP:disable/5.7A,disable UCP*/
		sc8547_slave_update_bits(chip->voocphy->slave_client, SC8547_REG_08, SC8547_SS_TIMEOUT_SET_MASK, SC8547_SS_TIMEOUT_DISABLE);
		sc8547_slave_write_byte(chip->voocphy->slave_client, SC8547_REG_09, 0x10); /*WD:disable*/
		sc8547_slave_write_byte(chip->voocphy->slave_client, SC8547_REG_11, 0x80); /*ADC_CTRL:enable ADC*/
		sc8547_slave_write_byte(chip->voocphy->slave_client, SC8547_REG_0D, 0x70); /*PMID2OUT_OVP_UVP:500mV,-100mV*/
		sc8547_slave_write_byte(chip->voocphy->slave_client, SC8547_REG_0C, 0x02); /*CTRL4:force VAC_OK*/
		/*need 50ms delay between force VAC_OK and chg_enable*/
		msleep(50);
	}
	chg_info("[%s] set cp %s...\n", chip->slave_dev->of_node->name, en ? "enable" : "disable");
	rc = sc8547_slave_set_chg_enable(chip->voocphy, en);
	if (rc < 0) {
		chg_err("[%s] set cp %s fail\n", chip->slave_dev->of_node->name, en ? "enable" : "disable");
		return rc;
	}
	if (en) {
		msleep(500);
		sc8547_slave_read_byte(chip->voocphy->slave_client, SC8547_REG_06, &data);
		chg_info("[%s] <~WPC~> cp status:0x%x.\n", chip->slave_dev->of_node->name, data);
		if ((data & SC8547_CP_SWITCHING_STAT_MASK) != SC8547_CP_SWITCHING_STAT_MASK)
			return -EAGAIN;
	}

	return 0;
}

static int sc8547_slave_get_wdt_reg_by_time(unsigned int time_ms, u8 *reg)
{
	if (time_ms == 0) {
		*reg = SC8547_WATCHDOG_DIS;
	} else if (time_ms <= 200) {
		*reg = SC8547_WATCHDOG_200MS;
	} else if (time_ms <= 500) {
		*reg = SC8547_WATCHDOG_500MS;
	} else if (time_ms <= 1000) {
		*reg = SC8547_WATCHDOG_1S;
	} else if (time_ms <= 5000) {
		*reg = SC8547_WATCHDOG_5S;
	} else if (time_ms <= 30000) {
		*reg = SC8547_WATCHDOG_30S;
	} else {
		chg_err("sc8547 watchdog not support %dms(>30s)\n", time_ms);
		return -EINVAL;
	}

	return 0;
}

static int sc8547_slave_cp_wd_enable(struct oplus_chg_ic_dev *ic_dev, int timeout_ms)
{
	struct sc8547a_slave_device *chip;
	u8 reg_val = 0;
	int ret = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL\n");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("set watchdog timeout to %dms\n", timeout_ms);

	ret = sc8547_slave_get_wdt_reg_by_time(timeout_ms, &reg_val);
	if (ret < 0)
		return ret;

	ret = sc8547_slave_update_bits(chip->slave_client, SC8547_REG_09,
				SC8547_WATCHDOG_MASK, reg_val);
	chg_info("set watchdog reg to 0x%02x ok!\n", reg_val);

	if (ret < 0) {
		chg_err("failed to set watchdog reg to 0x%02x, ret=%d\n", reg_val, ret);
		return ret;
	}

	return 0;
}

static int sc8547_slave_cp_hw_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8547a_slave_device *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (chip->rested)
		return 0;

	sc8547_slave_hardware_init(chip->voocphy);
	chip->work_start = false;
	return 0;
}

static int sc8547_slave_cp_set_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	struct sc8547a_slave_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	if (!sc8547_slave_check_work_mode_support(mode)) {
		chg_err("not supported work mode, mode=%d\n", mode);
		return -EINVAL;
	}

	if (mode == CP_WORK_MODE_BYPASS)
		rc = sc8547_slave_vooc_hw_setting(chip->voocphy);
	else
		rc = sc8547_slave_svooc_hw_setting(chip->voocphy);

	if (rc < 0)
		chg_err("set work mode to %d error\n", mode);

	return rc;
}

static int sc8547_slave_cp_get_work_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode *mode)
{
	struct sc8547a_slave_device *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_slave_read_byte(chip->voocphy->slave_client, SC8547_REG_09, &data);
	if (rc < 0) {
		chg_err("read SC8547_REG_07 error, rc=%d\n", rc);
		return rc;
	}

	if (data & BIT(7))
		*mode = CP_WORK_MODE_BYPASS;
	else
		*mode = CP_WORK_MODE_2_TO_1;

	return 0;
}

static int sc8547_slave_cp_check_work_mode_support(struct oplus_chg_ic_dev *ic_dev, enum oplus_cp_work_mode mode)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	return sc8547_slave_check_work_mode_support(mode);
}

static int sc8547_slave_cp_set_iin(struct oplus_chg_ic_dev *ic_dev, int iin)
{
	return 0;
}

static int sc8547_slave_cp_get_vin(struct oplus_chg_ic_dev *ic_dev, int *vin)
{
	struct sc8547a_slave_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_slave_get_cp_vbus(chip->voocphy);
	if (rc < 0) {
		chg_err("can't get cp vin, rc=%d\n", rc);
		return rc;
	}
	*vin = rc;

	return 0;
}

static int sc8547_slave_cp_get_iin(struct oplus_chg_ic_dev *ic_dev, int *iin)
{
	struct sc8547a_slave_device *device;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	device = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_slave_get_ichg(device->voocphy);
	if (rc < 0) {
		chg_err("can't get cp iin, rc=%d\n", rc);
		return rc;
	}
	*iin = rc;
	return 0;
}

static int sc8547_slave_cp_get_vout(struct oplus_chg_ic_dev *ic_dev, int *vout)
{
	struct sc8547a_slave_device *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_slave_get_cp_vbat(chip->voocphy);
	if (rc < 0) {
		chg_err("can't get cp vout, rc=%d\n", rc);
		return rc;
	}
	*vout = rc;

	return 0;
}

static int sc8547_slave_cp_get_iout(struct oplus_chg_ic_dev *ic_dev, int *iout)
{
	struct sc8547a_slave_device *chip;
	int iin;
	bool working;
	enum oplus_cp_work_mode work_mode;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	/*
	 * There is an exception in the iout adc of sc8537a, which is obtained
	 * indirectly through iin
	 */
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_STATUS, &working);
	if (rc < 0)
		return rc;
	if (!working) {
		*iout = 0;
		return 0;
	}
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_IIN, &iin);
	if (rc < 0)
		return rc;
	rc = oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_CP_GET_WORK_MODE, &work_mode);
	if (rc < 0)
		return rc;
	switch (work_mode) {
	case CP_WORK_MODE_BYPASS:
		*iout = iin;
		break;
	case CP_WORK_MODE_2_TO_1:
		*iout = iin * 2;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sc8547_slave_cp_set_work_start(struct oplus_chg_ic_dev *ic_dev, bool start)
{
	struct sc8547a_slave_device *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	chg_info("%s work %s\n", chip->slave_dev->of_node->name, start ? "start" : "stop");

	rc = sc8547_slave_set_chg_enable(chip->voocphy, start);

	if (rc < 0)
		return rc;

	return 0;
}

static int sc8547_slave_cp_get_work_status(struct oplus_chg_ic_dev *ic_dev, bool *start)
{
	struct sc8547a_slave_device *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_slave_read_byte(chip->voocphy->slave_client, SC8547_REG_07, &data);
	if (rc < 0) {
		chg_err("read SC8547_REG_07 error, rc=%d\n", rc);
		return rc;
	}

	*start = data & BIT(7);

	return 0;
}

static int sc8547_slave_cp_adc_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct sc8547a_slave_device *chip;


	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	return sc8547_slave_set_adc_enable(chip->voocphy, en);

	return 0;
}

static int sc8547_slave_cp_watchdog_reset(struct oplus_chg_ic_dev *ic_dev)
{
	struct sc8547a_slave_device *chip;
	u8 data;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_priv_data(ic_dev);

	rc = sc8547_slave_read_byte(chip->slave_client, SC8547_REG_07, &data);
	if (rc < 0)
		return rc;

	return 0;
}

static void *sc8547_slave_cp_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, sc8547_slave_cp_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, sc8547_slave_cp_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, sc8547_slave_cp_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, sc8547_slave_cp_smt_test);
		break;
	case OPLUS_IC_FUNC_CP_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_ENABLE, sc8547_slave_cp_enable);
		break;
	case OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_WATCHDOG_ENABLE, sc8547_slave_cp_wd_enable);
		break;
	case OPLUS_IC_FUNC_CP_HW_INTI:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_HW_INTI, sc8547_slave_cp_hw_init);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_MODE, sc8547_slave_cp_set_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_MODE, sc8547_slave_cp_get_work_mode);
		break;
	case OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_CHECK_WORK_MODE_SUPPORT,
			sc8547_slave_cp_check_work_mode_support);
		break;
	case OPLUS_IC_FUNC_CP_SET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_IIN, sc8547_slave_cp_set_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VIN, sc8547_slave_cp_get_vin);
		break;
	case OPLUS_IC_FUNC_CP_GET_IIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IIN, sc8547_slave_cp_get_iin);
		break;
	case OPLUS_IC_FUNC_CP_GET_VOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_VOUT, sc8547_slave_cp_get_vout);
		break;
	case OPLUS_IC_FUNC_CP_GET_IOUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_IOUT, sc8547_slave_cp_get_iout);
		break;
	case OPLUS_IC_FUNC_CP_SET_WORK_START:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_WORK_START, sc8547_slave_cp_set_work_start);
		break;
	case OPLUS_IC_FUNC_CP_GET_WORK_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_GET_WORK_STATUS, sc8547_slave_cp_get_work_status);
		break;
	case OPLUS_IC_FUNC_CP_SET_ADC_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_SET_ADC_ENABLE, sc8547_slave_cp_adc_enable);
		break;
	case OPLUS_IC_FUNC_CP_WATCHDOG_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_CP_WATCHDOG_RESET, sc8547_slave_cp_watchdog_reset);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static int sc8547a_slave_disable_vote_callback(struct votable *votable, void *data,
					 int disable, const char *client, bool step)
{
	struct sc8547a_slave_device *chip = data;
	bool online;

	if (disable < 0)
		online = true;
	else
		online = !disable;
	chg_info("[%s]%s set sc8547a online to %s\n", votable_name(votable),
		 client, online ? "true" : "false");

	if (chip->cp_ic ==NULL)
		return -ENODEV;
	if (chip->cp_ic->online == online)
		return 0;

	chip->cp_ic->online = online;
	if (online) {
		oplus_chg_ic_virq_trigger(chip->cp_ic, OPLUS_IC_VIRQ_ONLINE);
		/*
		* The first online operation during the startup phase
		* does not require reporting of exceptions
		*/
		if (likely(chip->boot_online)) {
			oplus_chg_ic_creat_err_msg(chip->cp_ic, OPLUS_IC_ERR_CP,
						   CP_ERR_ONLINE_CHANGE,
						   "online");
			oplus_chg_ic_err_trigger_and_clean(chip->cp_ic);
		} else {
			chip->boot_online = true;
		}
	} else {
		oplus_chg_ic_virq_trigger(chip->cp_ic, OPLUS_IC_VIRQ_OFFLINE);
		schedule_work(&chip->ic_offline_work);
		oplus_chg_ic_creat_err_msg(chip->cp_ic, OPLUS_IC_ERR_CP,
					   CP_ERR_ONLINE_CHANGE,
					   "offline");
		oplus_chg_ic_err_trigger_and_clean(chip->cp_ic);
	}

	return 0;
}

struct oplus_chg_ic_virq sc8547_slave_cp_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

#define CP_IC_VOTEABLE_NAME_MAX		128
static int sc8547_slave_ic_register(struct sc8547a_slave_device *device)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *child;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg;
	char votable_name[CP_IC_VOTEABLE_NAME_MAX] = { 0 };
	int rc;

	for_each_child_of_node(device->slave_dev->of_node, child) {
		rc = of_property_read_u32(child, "oplus,ic_type", &ic_type);
		if (rc < 0)
			continue;
		rc = of_property_read_u32(child, "oplus,ic_index", &ic_index);
		if (rc < 0)
			continue;
		ic_cfg.name = child->name;
		ic_cfg.index = ic_index;
		ic_cfg.type = ic_type;
		ic_cfg.priv_data = device;
		ic_cfg.of_node = child;
		switch (ic_type) {
		case OPLUS_CHG_IC_CP:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "cp-sc8547a:%d", ic_index);
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = sc8547_slave_cp_get_func;
			ic_cfg.virq_data = sc8547_slave_cp_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(sc8547_slave_cp_virq_table);
			if (device->disable_votable != NULL)
				break;

			snprintf(votable_name, CP_IC_VOTEABLE_NAME_MAX - 1,
				 "SC8547A_SLAVE_DISABLE:%d", ic_index);
			device->disable_votable = create_votable(votable_name,
				VOTE_SET_ANY, sc8547a_slave_disable_vote_callback, device);
			if (IS_ERR(device->disable_votable)) {
				rc = PTR_ERR(device->disable_votable);
				chg_err("creat disable_votable error, rc=%d\n", rc);
				device->disable_votable = NULL;
				return rc;
			}
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_type);
			continue;
		}

		ic_dev = devm_oplus_chg_ic_register(device->slave_dev, &ic_cfg);
		if (!ic_dev) {
			rc = -ENODEV;
			chg_err("register %s error\n", child->name);
			continue;
		}
		chg_info("register %s\n", child->name);

		switch (ic_dev->type) {
		case OPLUS_CHG_IC_CP:
			device->cp_work_mode = CP_WORK_MODE_UNKNOWN;
			device->cp_ic = ic_dev;
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_dev->type);
			continue;
		}
		of_platform_populate(child, NULL, NULL, device->slave_dev);
	}

	return 0;
}

static ssize_t sc8547_slave_show_registers(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8547");
	for (addr = 0x0; addr <= 0x3C; addr++) {
		if((addr < 0x24) || (addr > 0x2B && addr < 0x33)
		   || addr == 0x36 || addr == 0x3C) {
			ret = sc8547_slave_read_byte(chip->slave_client, addr, &val);
			if (ret == 0) {
				len = snprintf(tmpbuf, PAGE_SIZE - idx,
				               "Reg[%.2X] = 0x%.2x\n", addr, val);
				memcpy(&buf[idx], tmpbuf, len);
				idx += len;
			}
		}
	}

	return idx;
}

static ssize_t sc8547_slave_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct oplus_voocphy_manager *chip = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x3C)
		sc8547_slave_write_byte(chip->slave_client, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, 0660, sc8547_slave_show_registers, sc8547_slave_store_register);

static void sc8547_slave_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
}


static struct of_device_id sc8547_slave_charger_match_table[] = {
	{
		.compatible = "slave_vphy_sc8547",
	},
	{},
};

static struct oplus_voocphy_operations oplus_sc8547_slave_ops = {
	.hw_setting		= sc8547_slave_hw_setting,
	.init_vooc		= sc8547_slave_init_vooc,
	.update_data		= sc8547_slave_update_data,
	.get_chg_enable		= sc8547_slave_get_chg_enable,
	.set_chg_enable		= sc8547_slave_set_chg_enable,
	.get_ichg		= sc8547_slave_get_ichg,
	.reset_voocphy      	= sc8547_slave_reset_voocphy,
	.get_adc_enable		= sc8547_slave_get_adc_enable,
	.set_adc_enable		= sc8547_slave_set_adc_enable,
	.get_cp_status 		= sc8547_slave_get_cp_status,
	.get_voocphy_enable 	= sc8547_slave_get_voocphy_enable,
	.dump_voocphy_reg	= sc8547_slave_dump_reg_in_err_issue,
	.reset_voocphy_ovp	= sc8547_slave_reset_ovp,
};

static int sc8547_slave_parse_dt(struct oplus_voocphy_manager *chip)
{
	int rc;
	struct device_node * node = NULL;

	if (!chip) {
		chg_err("chip null\n");
		return -1;
	}

	node = chip->slave_dev->of_node;

  	rc = of_property_read_u32(node, "ovp_reg",
  	                          &slave_ovp_reg);
  	if (rc)
  		slave_ovp_reg = DEFAULT_OVP_REG_CONFIG;
  	else
  		chg_err("slave_ovp_reg is %d\n", slave_ovp_reg);

	rc = of_property_read_u32(node, "ocp_reg",
  	                          &slave_ocp_reg);
	if (rc)
		slave_ocp_reg = DEFAULT_OCP_REG_CONFIG;
	else
		chg_err("slave_ocp_reg is %d\n", slave_ocp_reg);

	rc = of_property_read_u8(node, "vbus_ovp_reg", &vbus_ovp_reg);
	if (rc)
		vbus_ovp_reg = 0x50; /* VBUS_OVP 10V */
	chg_err("vbus_ovp_reg=0x%02x\n", vbus_ovp_reg);

	rc = of_property_read_u8(node, "vooc_vbus_ovp_reg", &vooc_vbus_ovp_reg);
	if (rc)
		vooc_vbus_ovp_reg = 0x50; /* VBUS_OVP 10V */
	chg_err("vooc_vbus_ovp_reg=0x%02x\n", vooc_vbus_ovp_reg);


	return 0;
}

static int sc8547_slave_charger_choose(struct oplus_voocphy_manager *chip)
{
	int ret;
	int max_count = 5;

	if (oplus_voocphy_chip_is_null()) {
		chg_err("oplus_voocphy_chip null, will do after master cp init!");
		return -EPROBE_DEFER;
	} else {
		while (max_count--) {
			ret = i2c_smbus_read_byte_data(chip->slave_client, 0x07);
			chg_info("0x07 = %d\n", ret);
			if (ret < 0) {
				chg_err("i2c communication fail, count = %d\n", max_count);
				continue;
			} else {
				break;
			}
		}
	}

	return ret;
}

#define SC8547A_SLAVE_CP_STATUS_REG_MAX	4
static u8 g_sc8547a_slave_cp_status_reg[SC8547A_SLAVE_CP_STATUS_REG_MAX] = {
	0x06, 0x09, 0x0e, 0x0f
};

static void sc8547a_slave_cp_regdump_work(struct work_struct *work)
{
	struct sc8547a_slave_device *chip =
		container_of(work, struct sc8547a_slave_device, cp_regdump_work);
	char *buf;
	int i;
	size_t index = 0;
	u8 data;

	buf = kzalloc(ERR_MSG_BUF, GFP_KERNEL);
	if (buf == NULL)
		return;

	for (i = 0; i < SC8547A_SLAVE_CP_STATUS_REG_MAX; i++) {
		data = 0;
		sc8547_slave_read_byte(chip->voocphy->slave_client, g_sc8547a_slave_cp_status_reg[i], &data);
		index += snprintf(buf + index, ERR_MSG_BUF, "0x%02x=%02x,",
			g_sc8547a_slave_cp_status_reg[i], data);
	}
	if (index > 0)
		buf[index - 1] = 0;

	oplus_chg_ic_creat_err_msg(chip->cp_ic, OPLUS_IC_ERR_CP,
		CP_ERR_REG_INFO, "%s", buf);
	oplus_chg_ic_err_trigger_and_clean(chip->cp_ic);
	kfree(buf);
}

static void sc8547a_slave_ic_offline_work(struct work_struct *work)
{
	struct sc8547a_slave_device *chip =
		container_of(work, struct sc8547a_slave_device, ic_offline_work);

	if (!chip->work_start)
		return;
	sc8547_slave_cp_set_work_start(chip->cp_ic, false);
	sc8547_slave_cp_enable(chip->cp_ic, false);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int sc8547_slave_charger_probe(struct i2c_client *client)
#else
static int sc8547_slave_charger_probe(struct i2c_client *client,
                                      const struct i2c_device_id *id)
#endif
{
	struct sc8547a_slave_device *device;
	struct oplus_voocphy_manager *chip;
	int rc;

	pr_err("sc8547_slave_slave_charger_probe enter!\n");

	device = devm_kzalloc(&client->dev, sizeof(*device), GFP_KERNEL);
	if (device == NULL) {
		chg_err("alloc sc8547a device buf error\n");
		return -ENOMEM;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		rc = -ENOMEM;
		goto device_err;
	}

	device->slave_client = client;
	device->slave_dev = &client->dev;
	chip->slave_client = client;
	chip->slave_dev = &client->dev;
	chip->priv_data = device;
	device->voocphy = chip;
	mutex_init(&i2c_rw_lock);
	i2c_set_clientdata(client, chip);

	if (oplus_voocphy_chip_is_null()) {
		pr_err("oplus_voocphy_chip null, will do after master cp init.\n");
		rc = -EPROBE_DEFER;
		goto chip_err;
	}

	rc = sc8547_slave_charger_choose(chip);
	if (rc <= 0) {
		chg_err("slave choose err\n");
		goto chip_err;
	}

	sc8547a_hw_version_check(chip);
	sc8547_slave_create_device_node(&(client->dev));
	sc8547_slave_parse_dt(chip);
	sc8547_slave_reg_reset(chip, true);
	sc8547_slave_init_device(chip);
	chip->slave_ops = &oplus_sc8547_slave_ops;
	oplus_voocphy_slave_init(chip);
	oplus_voocphy_get_chip(&oplus_voocphy_mg);
	rc = sc8547_slave_ic_register(device);
	INIT_WORK(&device->cp_regdump_work, sc8547a_slave_cp_regdump_work);
	INIT_WORK(&device->ic_offline_work, sc8547a_slave_ic_offline_work);
	if (rc < 0) {
		chg_err("slave cp ic register error\n");
		rc = -ENOMEM;
		goto chip_err;
	}
	vote(device->disable_votable, DEF_VOTER, false, 0, false);
	sc8547_slave_cp_init(device->cp_ic);
	pr_err("sc8547_slave_parse_dt successfully!\n");

	return 0;

chip_err:
	destroy_votable(device->disable_votable);
	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, chip);
device_err:
	devm_kfree(&client->dev, device);
	return rc;
}

static void sc8547_slave_charger_shutdown(struct i2c_client *client)
{
	sc8547_slave_write_byte(client, SC8547_REG_11, 0x00);
	sc8547_slave_write_byte(client, SC8547_REG_21, 0x00);

	return;
}

static const struct i2c_device_id sc8547_slave_charger_id[] = {
	{"sc8547-slave", 0},
	{},
};

static struct i2c_driver sc8547_slave_charger_driver = {
	.driver		= {
		.name	= "sc8547-charger-slave",
		.owner	= THIS_MODULE,
		.of_match_table = sc8547_slave_charger_match_table,
	},
	.id_table	= sc8547_slave_charger_id,

	.probe		= sc8547_slave_charger_probe,
	.shutdown	= sc8547_slave_charger_shutdown,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
static int sc8547_slave_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");

	if (i2c_add_driver(&sc8547_slave_charger_driver) != 0) {
		chg_err(" failed to register sc8547 i2c driver.\n");
	} else {
		chg_debug(" Success to register sc8547 i2c driver.\n");
	}

	return ret;
}

subsys_initcall(sc8547_slave_subsys_init);
#else
int sc8547_slave_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");

	if (i2c_add_driver(&sc8547_slave_charger_driver) != 0) {
		chg_err(" failed to register sc8547 i2c driver.\n");
	} else {
		chg_debug(" Success to register sc8547 i2c driver.\n");
	}

	return ret;
}

void sc8547_slave_subsys_exit(void)
{
	i2c_del_driver(&sc8547_slave_charger_driver);
}
oplus_chg_module_register(sc8547_slave_subsys);
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)*/

MODULE_DESCRIPTION("SC SC8547 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aiden-yu@southchip.com");
