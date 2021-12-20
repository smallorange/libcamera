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
	typedef struct __attribute__((packed)) y_table_item {
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
	void af_coarse_scan(IPAContext &context);
	void af_fine_scan(IPAContext &context);
	bool af_scan(IPAContext &context, int min_step);
	void af_reset(IPAContext &context);

	/* Used for focus scan. */
	uint32_t focus_;
	/* Focus good */
	uint32_t goodFocus_;
	/* Recent AF statistic variance. */
	double currentVariance_;
	/* The frames to be ignore before starting measuring. */
	uint32_t ignoreCounter_;
	/* previous variance. it is used to determine the gradient */
	double previousVariance_;
	/* Max scan steps of each pass of AF scaning */
	uint32_t maxStep_;
	/* coarse scan stable. Complete low pass search (coarse) scan) */
	bool coarseComplete_;
	/* fine scan stable. Complete high pass scan (fine scan) */
	bool fineComplete_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_ALGORITHMS_AF_H__ */
