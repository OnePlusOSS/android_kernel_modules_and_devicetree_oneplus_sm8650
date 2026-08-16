ifeq ($(CONFIG_OPLUS_SENSOR_MAG_MATCH_NFC), y)
CFLAGS_oplus_sensor_devinfo.o += -DOPLUS_FEATURE_SENSOR_MAG_MATCH_NFC
endif
oplus_sensor-y := oplus_sensor_devinfo.o
oplus_sensor-y += oplus_press_cali_info.o
oplus_sensor-y += oplus_pad_als_info.o
obj-$(CONFIG_OPLUS_FEATURE_SENSOR_CFG) += oplus_sensor.o
obj-$(CONFIG_SSC_INTERACTIVE) += oplus_ssc_interact/
obj-$(CONFIG_OPLUS_SENSOR_FB_QC) += oplus_sensor_feedback/