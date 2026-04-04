#ifndef __HBP_POWER_H_
#define __HBP_POWER_H_

#include <linux/regulator/consumer.h>
#include "hbp_core.h"

#define MAX_POWER_SEQ	(10)

enum power_type {
	POWER_AVDD = 0x1000,
	POWER_VDDI,
	POWER_RESET,
	POWER_BUS,
	POWER_MAX
};

struct power_sequeue {
	enum power_type type;
	bool en;
	uint32_t msleep;
};

#endif
