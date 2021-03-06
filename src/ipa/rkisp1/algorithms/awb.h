/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * awb.h - AWB control algorithm
 */

#pragma once

#include <linux/rkisp1-config.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class Awb : public Algorithm
{
public:
	Awb() = default;
	~Awb() = default;

	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, rkisp1_params_cfg *params) override;
	void process(IPAContext &context, const rkisp1_stat_buffer *stats) override;

private:
	uint32_t estimateCCT(double red, double green, double blue);
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
