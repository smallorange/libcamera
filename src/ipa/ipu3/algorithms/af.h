/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 *
 * af.h - IPU3 Af control
 */
#ifndef __LIBCAMERA_IPU3_ALGORITHMS_AF_H__
#define __LIBCAMERA_IPU3_ALGORITHMS_AF_H__

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

class Af : public Algorithm
{
	/* The format of y_table. From ipu3-ipa repo */
	typedef struct y_table_item {
		uint16_t y1_avg;
		uint16_t y2_avg;
	} y_table_item_t;

public:
	Af();
	~Af();

	void prepare(IPAContext &context, ipu3_uapi_params *params) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const ipu3_uapi_stats_3a *stats) override;

private:
	int vcmSet(int value);

	int vcmFd_;
	/* Used for focus scan. */
	uint32_t focus_;
	/* Recent AF statistic variance. */
	double currentVariance_;
	/* The frames to be ignore before starting measuring. */
	uint32_t ignoreFrame_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_ALGORITHMS_AF_H__ */
