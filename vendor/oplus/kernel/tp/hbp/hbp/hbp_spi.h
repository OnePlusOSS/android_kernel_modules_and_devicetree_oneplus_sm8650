#ifndef __HBP_BUS_H__
#define __HBP_BUS_H__

#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/syscalls.h>
#include <linux/vmalloc.h>
#include <uapi/linux/sched/types.h>
#include <linux/device.h>
#include <linux/spi/spi.h>

#define HBP_SPI_MODULE_NAME  "hbp_spi_bus"

struct bus_operations {
	int (*read_block)(void *ops, uint8_t *data, size_t len);
	int (*write_block)(void *ops, uint8_t *data, size_t len);
	int(*spi_sync)(void *ops, uint8_t *tx, uint8_t *rx, size_t len);
	void (*shutdown)(void *ops);
	int (*spi_set_para)(void *ops, uint8_t mode, uint8_t bits_per_word, int speed);
	int (*spi_get_para)(void *ops, uint8_t *mode, uint8_t *bits_per_word, int *speed);
};

struct spi_cache {
	struct spi_transfer *xfer;
	uint32_t xfer_count;
	uint8_t *tx_buf;
	size_t tx_count;
	uint8_t *rx_buf;
	size_t rx_count;
};

struct spi_param {
	uint16_t byte_delay_us;
	uint16_t block_delay_us;
	int mode;
	struct spi_cache cache;
};

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#else
struct spi_geni_qcom_ctrl_data {
	u32 spi_cs_clk_delay;
	u32 spi_inter_words_delay;
};
#endif

struct spi_bus {
	struct bus_operations spi_ops;
	struct mutex mtx;
	const char *name;
	struct spi_device *spi_dev;
	struct spi_param param;
	bool bus_ready; /*spi or i2c resume status*/
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#else
	struct spi_geni_qcom_ctrl_data delay_params;
#endif
	wait_queue_head_t spi_wait;
	int irq_need_dev_resume_time; /*control setting of wait resume time*/
};

extern int hw_interface_init(void);

#endif
