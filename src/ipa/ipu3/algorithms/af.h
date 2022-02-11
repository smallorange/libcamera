/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 *
 * af.h - IPU3 Af algorithm
 */
#ifndef __LIBCAMERA_IPU3_ALGORITHMS_AF_H__
#define __LIBCAMERA_IPU3_ALGORITHMS_AF_H__

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

/* static variables from repo of chromium */
static constexpr uint8_t kAfMinGridWidth = 16;
static constexpr uint8_t kAfMinGridHeight = 16;
static constexpr uint8_t kAfMaxGridWidth = 32;
static constexpr uint8_t kAfMaxGridHeight = 24;
static constexpr uint16_t kAfMinGridBlockWidth = 4;
static constexpr uint16_t kAfMinGridBlockHeight = 3;
static constexpr uint16_t kAfMaxGridBlockWidth = 6;
static constexpr uint16_t kAfMaxGridBlockHeight = 6;
static constexpr uint16_t kAfDefaultHeightPerSlice = 2;

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
	~Af() = default;

	void prepare(IPAContext &context, ipu3_uapi_params *params) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const ipu3_uapi_stats_3a *stats) override;

private:
	void afCoarseScan(IPAContext &context);
	void afFineScan(IPAContext &context);
	bool afScan(IPAContext &context, int min_step);
	void afReset(IPAContext &context);
	bool afNeedIgnoreFrame();
	void afIgnoreFrameReset();
	double afEstemateVariance(y_table_item_t *y_item, uint32_t len,
				  bool isY1);
	bool afIsOutOfFocus(IPAContext &context);

	/* control the VCM step while sweeping the lens. */
	uint32_t focus_;
	/* remember the best focus value. */
	uint32_t goodFocus_;
	/* recent AF statistic variance. */
	double currentVariance_;
	/* the frames to be ignore before starting measuring. */
	uint32_t ignoreCounter_;
	/* previous variance. it is used to determine the gradient */
	double previousVariance_;
	/* max scan steps of each pass of AF scaning */
	uint32_t maxStep_;
	/* coarse scan stable. Complete low pass search (coarse) scan) */
	bool coarseCompleted_;
	/* fine scan stable. Complete high pass scan (fine scan) */
	bool fineCompleted_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_ALGORITHMS_AF_H__ */
