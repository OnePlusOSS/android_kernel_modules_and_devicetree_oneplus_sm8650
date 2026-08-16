#ifndef __HBP_API_H__
#define __HBP_API_H__

#include "hbp_core.h"

bool hbp_get_feature_param(struct hbp_feature *features,
			   touch_feature mode,
			   struct param_value *param);

#endif
