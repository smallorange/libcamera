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

/* minimum focus step for searching appropriate focus */
static constexpr uint32_t coarseSearchStep_ = 10;
static constexpr uint32_t fineSearchStep_ = 1;

/* max ratio of variance change, 0.0 < MaxChange_ < 1.0 */
static constexpr double MaxChange_ = 0.8;

/* the numbers of frame to be ignored, before performing focus scan. */
static constexpr uint32_t ignoreFrame_ = 10;

/* settings for Auto Focus from the kernel */
static struct ipu3_uapi_af_config_s imgu_css_af_defaults = {
	.filter_config = {
		{ 0, 0, 0, 0 },
		{ 0, 0, 0, 0 },
		{ 0, 0, 0, 128 },
		0,
		{ 0, 0, 0, 0 },
		{ 0, 0, 0, 0 },
		{ 0, 0, 0, 128 },
		0,
		.y_calc = { 8, 8, 8, 8 },
		.nf = { 0, 7, 0, 7, 0 },
	},
	.padding = { 0, 0, 0, 0 },
	.grid_cfg = {
		.width = 16,
		.height = 16,
		.block_width_log2 = 3,
		.block_height_log2 = 3,
		.height_per_slice = 8,
		.x_start = 10,
		.y_start = 2 | IPU3_UAPI_GRID_Y_START_EN,
		.x_end = 0,
		.y_end = 0
	},
};


Af::Af()
	: focus_(0), goodFocus_(0), currentVariance_(0.0), previousVariance_(0.0),
	  coarseComplete_(false), fineComplete_(false)
{
	maxStep_ = MaxFocusSteps_;
}

Af::~Af()
{
}

void Af::prepare(IPAContext &context, ipu3_uapi_params *params)
{
	params->use.acc_af = 1;
	params->acc_param.af = imgu_css_af_defaults;
	params->acc_param.af.grid_cfg.x_start = context.configuration.af.start_x;
	params->acc_param.af.grid_cfg.y_start = context.configuration.af.start_y | IPU3_UAPI_GRID_Y_START_EN;
}

/**
 * \brief Configure the Af given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Af::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	/* determined focus value i.e. current focus value */
	context.frameContext.af.focus = 0;
	/* maximum variance of the AF statistics */
	context.frameContext.af.maxVariance = 0;
	/* is focused? if it is true, the AF should be in a stable state. */
	context.frameContext.af.stable = false;

	/*
	 * AF default area configuration
	 * Move AF area to the center of the image.
	 */
	/* Default AF width is 16x8 = 128 */
	context.configuration.af.start_x = (configInfo.bdsOutputSize.width / 2) -
					   ((imgu_css_af_defaults.grid_cfg.width *
					     pow(2, imgu_css_af_defaults.grid_cfg.block_width_log2) / 2));
	context.configuration.af.start_y = (configInfo.bdsOutputSize.height / 2) -
					   ((imgu_css_af_defaults.grid_cfg.height *
					     pow(2, imgu_css_af_defaults.grid_cfg.block_height_log2) / 2));

	LOG(IPU3Af, Debug) << "BDS X: "
			   << configInfo.bdsOutputSize.width
			   << " Y: "
			   << configInfo.bdsOutputSize.height;
	LOG(IPU3Af, Debug) << "AF start from X: "
			   << context.configuration.af.start_x
			   << " Y: "
			   << context.configuration.af.start_y;

	return 0;
}

/**
 * \brief AF coarse scan
 * \param[in] context The shared IPA context
 *
 */
void Af::af_coarse_scan(IPAContext &context)
{
	if (coarseComplete_ == true)
		return;

	if (af_scan(context, coarseSearchStep_)) {
		coarseComplete_ = true;
		context.frameContext.af.maxVariance = 0;
		focus_ = context.frameContext.af.focus - (context.frameContext.af.focus * 0.1);
		context.frameContext.af.focus = focus_;
		previousVariance_ = 0;
		maxStep_ = focus_ + (focus_ * 0.2);
	}
}

/**
 * \brief AF fine scan
 * \param[in] context The shared IPA context
 *
 */
void Af::af_fine_scan(IPAContext &context)
{
	if (coarseComplete_ != true)
		return;

	if (af_scan(context, fineSearchStep_)) {
		context.frameContext.af.stable = true;
		fineComplete_ = true;
	}
}

/**
 * \brief AF reset
 * \param[in] context The shared IPA context
 *
 */
void Af::af_reset(IPAContext &context)
{
	context.frameContext.af.maxVariance = 0;
	context.frameContext.af.focus = 0;
	focus_ = 0;
	context.frameContext.af.stable = false;
	ignoreCounter_ = ignoreFrame_;
	previousVariance_ = 0.0;
	coarseComplete_ = false;
	fineComplete_ = false;
	maxStep_ = MaxFocusSteps_;
}

/**
 * \brief AF  scan
 * \param[in] context The shared IPA context
 *
 * \return True, if it finds a AF value.
 */
bool Af::af_scan(IPAContext &context, int min_step)
{
	/* find the maximum variance during the AF scan using a greedy strategy */
	if (currentVariance_ > context.frameContext.af.maxVariance) {
		context.frameContext.af.maxVariance = currentVariance_;
		goodFocus_ = focus_;
	}

	if (focus_ > maxStep_) {
		/* if reach the max step, move lens to the position and set "focus stable". */
		context.frameContext.af.focus = goodFocus_;
		return true;
	} else {
		/* check negative gradient */
		if ((currentVariance_ - context.frameContext.af.maxVariance) > -(context.frameContext.af.maxVariance * 0.15)) {
			focus_ += min_step;
			context.frameContext.af.focus = focus_;
		} else {
			context.frameContext.af.focus = goodFocus_;
			previousVariance_ = currentVariance_;
			return true;
		}
	}
	LOG(IPU3Af, Debug) << "Variance prevrious: "
			   << previousVariance_
			   << " current: "
			   << currentVariance_
			   << " Diff: "
			   << (currentVariance_ - context.frameContext.af.maxVariance);
	previousVariance_ = currentVariance_;
	LOG(IPU3Af, Debug) << "Focus searching max variance is: "
			   << context.frameContext.af.maxVariance
			   << " Focus step is "
			   << goodFocus_
			   << " Current scan is "
			   << focus_;
	return false;
}

/**
 * \brief Determine the max contrast image and lens position. y_table is the
 * statictic data from IPU3 and is composed of low pass and high pass filtered
 * value. High pass filtered value also represents the sharpness of the image.
 * Based on this, if the image with highest variance of the high pass filtered
 * value (contrast) during the AF scan, the position of the lens should be the
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
	 * Calculate the mean and the varience of each non-zero AF statistics, since IPU3 only determine the AF value
	 * for a given grid.
	 * For coarse: low pass results are used.
	 * For fine: high pass results are used.
	 */
	if (coarseComplete_) {
		for (z = 0; z < (IPU3_UAPI_AF_Y_TABLE_MAX_SIZE) / 4; z++) {
			total = total + y_item[z].y2_avg;
			if (y_item[z].y2_avg == 0)
				break;
		}
		mean = total / z;

		for (z = 0; z < (IPU3_UAPI_AF_Y_TABLE_MAX_SIZE) / 4 && y_item[z].y2_avg != 0; z++) {
			var_sum = var_sum + ((y_item[z].y2_avg - mean) * (y_item[z].y2_avg - mean));
			if (y_item[z].y2_avg == 0)
				break;
		}
	} else {
		for (z = 0; z < (IPU3_UAPI_AF_Y_TABLE_MAX_SIZE) / 4; z++) {
			total = total + y_item[z].y1_avg;
			if (y_item[z].y1_avg == 0)
				break;
		}
		mean = total / z;

		for (z = 0; z < (IPU3_UAPI_AF_Y_TABLE_MAX_SIZE) / 4 && y_item[z].y1_avg != 0; z++) {
			var_sum = var_sum + ((y_item[z].y1_avg - mean) * (y_item[z].y1_avg - mean));
			if (y_item[z].y1_avg == 0)
				break;
		}
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
			if (ignoreCounter_ == 0) {
				af_reset(context);
			} else
				ignoreCounter_--;
		} else
			ignoreCounter_ = ignoreFrame_;
	} else {
		if (ignoreCounter_ != 0)
			ignoreCounter_--;
		else {
			af_coarse_scan(context);
			af_fine_scan(context);
		}
	}
}

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
