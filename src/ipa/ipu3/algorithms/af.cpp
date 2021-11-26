/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 *
 * af.cpp - IPU3 auto focus control
 */

#include "af.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <numeric>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <libcamera/base/log.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/histogram.h"

/**
 * \file af.h
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::ipu3::algorithms {

/**
 * \class Af
 * \brief A IPU3 auto-focus accelerator based auto focus algorthim
 *
 * This algorithm is used to determine the position of the lens and get a
 * focused image. The IPU3 AF accelerator computes the statistics, composed
 * by high pass and low pass filtered value and stores in a AF buffer.
 * Typically, for a focused image, it has relative high contrast than a
 * blurred image, i.e. an out of focus image. Therefore, if an image with the
 * highest contrast can be found from the AF scan, the lens' position is the
 * best step of the focus.
 *
 */

LOG_DEFINE_CATEGORY(IPU3Af)

/**
 * Maximum focus value of the VCM control
 * \todo should be obtained from the VCM driver
 */
static constexpr uint32_t MaxFocusSteps_ = 1023;

/* Minimum focus step for searching appropriate focus*/
static constexpr uint32_t MinSearchStep_ = 5;

/* max ratio of variance change, 0.0 < MaxChange_ < 1.0*/
static constexpr double MaxChange_ = 0.8;

Af::Af()
	: focus_(0), currentVariance_(0.0)
{
}

Af::~Af()
{
}

void Af::prepare(IPAContext &context, ipu3_uapi_params *params)
{
	/* AF grid config */
	params->acc_param.af.grid_cfg.width = 16;
	params->acc_param.af.grid_cfg.height = 16;
	params->acc_param.af.grid_cfg.block_height_log2 = 3;
	params->acc_param.af.grid_cfg.block_width_log2 = 3;
	params->acc_param.af.grid_cfg.height_per_slice = 2;
	/* Start position of AF area */
	params->acc_param.af.grid_cfg.x_start = context.configuration.af.start_x;
	params->acc_param.af.grid_cfg.y_start = context.configuration.af.start_y | IPU3_UAPI_GRID_Y_START_EN;

	params->acc_param.af.filter_config.y1_sign_vec = 0;
	params->acc_param.af.filter_config.y2_sign_vec = 0;

	/* b + gb + gr + r = 32 */
	params->acc_param.af.filter_config.y_calc.y_gen_rate_b = 8;
	params->acc_param.af.filter_config.y_calc.y_gen_rate_gb = 8;
	params->acc_param.af.filter_config.y_calc.y_gen_rate_gr = 8;
	params->acc_param.af.filter_config.y_calc.y_gen_rate_r = 8;

	/* 2^7 = 128,  a1 + a2 + ... + a12 = 128, log2 of sum of a1 to a12*/
	params->acc_param.af.filter_config.nf.y1_nf = 7;
	params->acc_param.af.filter_config.nf.y2_nf = 7;

	/* Low pass filter configuration (y1_coeff_n) */
	params->acc_param.af.filter_config.y1_coeff_0.a1 = 0;
	params->acc_param.af.filter_config.y1_coeff_0.a2 = 0;
	params->acc_param.af.filter_config.y1_coeff_0.a3 = 0;
	params->acc_param.af.filter_config.y1_coeff_0.a4 = 0;

	params->acc_param.af.filter_config.y1_coeff_1.a5 = 0;
	params->acc_param.af.filter_config.y1_coeff_1.a6 = 0;
	params->acc_param.af.filter_config.y1_coeff_1.a7 = 0;
	params->acc_param.af.filter_config.y1_coeff_1.a8 = 0;

	params->acc_param.af.filter_config.y1_coeff_2.a9 = 0;
	params->acc_param.af.filter_config.y1_coeff_2.a10 = 0;
	params->acc_param.af.filter_config.y1_coeff_2.a11 = 0;
	params->acc_param.af.filter_config.y1_coeff_2.a12 = 128;

	/* High pass filter configuration (y2_coeff_n) */
	params->acc_param.af.filter_config.y2_coeff_0.a1 = 0;
	params->acc_param.af.filter_config.y2_coeff_0.a2 = 0;
	params->acc_param.af.filter_config.y2_coeff_0.a3 = 0;
	params->acc_param.af.filter_config.y2_coeff_0.a4 = 0;

	params->acc_param.af.filter_config.y2_coeff_1.a5 = 0;
	params->acc_param.af.filter_config.y2_coeff_1.a6 = 0;
	params->acc_param.af.filter_config.y2_coeff_1.a7 = 0;
	params->acc_param.af.filter_config.y2_coeff_1.a8 = 0;

	params->acc_param.af.filter_config.y2_coeff_2.a9 = 0;
	params->acc_param.af.filter_config.y2_coeff_2.a10 = 0;
	params->acc_param.af.filter_config.y2_coeff_2.a11 = 0;
	params->acc_param.af.filter_config.y2_coeff_2.a12 = 128;

	/* Enable AF accelerator */
	params->use.acc_af = 1;
}

/**
 * \brief Configure the Af given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Af::configure(IPAContext &context, [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	/* Determined focus value i.e. current focus value */
	context.frameContext.af.focus = 0;
	/* Maximum variance of the AF statistics */
	context.frameContext.af.maxVariance = 0;
	/* is focused? if it is true, the AF should be in a stable state. */
	context.frameContext.af.stable = false;
	/* Frame to be ignored before start to estimate AF variance. */
	ignoreFrame_ = 10;

	/*
	 * AF default area configuration
	 * Move AF area to the center of the image.
	 */
	/* AF width is 16x8 = 128 */
	context.configuration.af.start_x = (1280 / 2) - 64;
	context.configuration.af.start_y = (720 / 2) - 64;

	return 0;
}

/**
 * \brief Determine the max contrast image and lens position. y_table is the
 * statictic data from IPU3 and is composed of low pass and high pass filtered
 * value. High pass filtered value also represents the sharpness of the image.
 * Based on this, if the image with highest variance of the high pass filtered
 * value (contrast) during the AF scan, the position of the len should be the
 * best focus.
 * \param[in] context The shared IPA context.
 * \param[in] stats The statistic buffer of 3A from the IPU3.
 */
void Af::process(IPAContext &context, const ipu3_uapi_stats_3a *stats)
{
	uint32_t total = 0;
	double mean;
	uint64_t var_sum = 0;
	y_table_item_t *y_item;
	int z = 0;

	y_item = (y_table_item_t *)stats->af_raw_buffer.y_table;

	/**
	 * Calculate the mean of each non-zero AF statistics, since IPU3 only determine the AF value
	 * for a given grid.
	 */
	for (z = 0; z < (IPU3_UAPI_AF_Y_TABLE_MAX_SIZE) / 4; z++) {
		printf("%d, ", y_item[z].y2_avg);
		total = total + y_item[z].y2_avg;
		if (y_item[z].y2_avg == 0)
			break;
	}
	mean = total / z;

	/* Calculate the variance of every AF statistic value. */
	for (z = 0; z < (IPU3_UAPI_AF_Y_TABLE_MAX_SIZE) / 4 && y_item[z].y2_avg != 0; z++) {
		var_sum = var_sum + ((y_item[z].y2_avg - mean) * (y_item[z].y2_avg - mean));
		if (y_item[z].y2_avg == 0)
			break;
	}

	/* Determine the average variance of the frame. */
	currentVariance_ = static_cast<double>(var_sum) / static_cast<double>(z);
	LOG(IPU3Af, Debug) << "variance: " << currentVariance_;

	if (context.frameContext.af.stable == true) {
		const uint32_t diff_var = std::abs(currentVariance_ - context.frameContext.af.maxVariance);
		const double var_ratio = diff_var / context.frameContext.af.maxVariance;
		LOG(IPU3Af, Debug) << "Change ratio: "
				   << var_ratio
				   << " current focus: "
				   << context.frameContext.af.focus;
		/**
		 * If the change ratio of contrast is over Maxchange_ (out of focus),
		 * trigger AF again.
		 */
		if (var_ratio > MaxChange_) {
			if (ignoreFrame_ == 0) {
				context.frameContext.af.maxVariance = 0;
				context.frameContext.af.focus = 0;
				focus_ = 0;
				context.frameContext.af.stable = false;
				ignoreFrame_ = 60;
			} else
				ignoreFrame_--;
		} else
			ignoreFrame_ = 10;
	} else {
		if (ignoreFrame_ != 0)
			ignoreFrame_--;
		else {
			/* Find the maximum variance during the AF scan using a greedy strategy */
			if (currentVariance_ > context.frameContext.af.maxVariance) {
				context.frameContext.af.maxVariance = currentVariance_;
				context.frameContext.af.focus = focus_;
			}

			if (focus_ > MaxFocusSteps_) {
				/* If reach the max step, move lens to the position and set "focus stable". */
				context.frameContext.af.stable = true;
			} else {
				focus_ += MinSearchStep_;
				context.frameContext.af.focus = focus_;
			}
			LOG(IPU3Af, Debug) << "Focus searching max variance is: "
					   << context.frameContext.af.maxVariance
					   << " Focus step is "
					   << context.frameContext.af.focus;
		}
	}
}

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */