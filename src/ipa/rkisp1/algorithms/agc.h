/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * agc.h - RkISP1 AGC/AEC mean-based control algorithm
 */

#pragma once

#include <linux/rkisp1-config.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::rkisp1::algorithms {

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void process(IPAContext &context, const rkisp1_stat_buffer *stats) override;

private:
	void computeExposure(IPAContext &Context, double yGain);
	utils::Duration filterExposure(utils::Duration exposureValue);
	double estimateLuminance(const rkisp1_cif_isp_ae_stat *ae, double gain);

	uint64_t frameCount_;

	uint32_t numCells_;

	utils::Duration filteredExposure_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
