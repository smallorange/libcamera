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

/**
 * \def AF_MIN_GRID_WIDTH
 * \brief the minimum width of AF grid.
 * The minimum grid horizontal dimensions, in number of grid blocks(cells).
*/

/**
 * \def AF_MIN_GRID_HEIGHT
 * \brief the minimum height of AF grid.
 * The minimum grid horizontal dimensions, in number of grid blocks(cells).
*/

/**
 * \def AF_MIN_BLOCK_WIDTH
 * \brief the minimum block size of the width.
 */

/**
 * \def AF_MAX_BLOCK_WIDTH
 * \brief the maximum block size of the width.
 */

/**
 * \def AF_MIN_BLOCK_HEIGHT
 * \brief the minimum block size of the height.
 */

/**
 * \def AF_MAX_BLOCK_HEIGHT
 * \brief the maximum block size of the height.
 */

/**
 * \def AF_DEFAULT_HEIGHT_PER_SLICE
 * \brief The default number of blocks in vertical axis per slice.
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::ipu3::algorithms {

/**
 * \class Af
 * \brief An auto-focus algorithm based on IPU3 statistics
 *
 * This algorithm is used to determine the position of the lens and get a
 * focused image. The IPU3 AF processing block computes the statistics,
 * composed by high pass and low pass filtered value and stores in a AF buffer.
 * Typically, for a focused image, it has relative high contrast than a
 * blurred image, i.e. an out of focus image. Therefore, if an image with the
 * highest contrast can be found from the AF scan, the lens' position is the
 * best step of the focus.
 *
 */

LOG_DEFINE_CATEGORY(IPU3Af)

/**
 * Maximum focus steps of the VCM control
 * \todo should be obtained from the VCM driver
 */
static constexpr uint32_t maxFocusSteps = 1023;

/* minimum focus step for searching appropriate focus */
static constexpr uint32_t coarseSearchStep = 10;
static constexpr uint32_t fineSearchStep = 1;

/* max ratio of variance change, 0.0 < MaxChange_ < 1.0 */
static constexpr double MaxChange = 0.2;

/* the numbers of frame to be ignored, before performing focus scan. */
static constexpr uint32_t ignoreFrame = 10;

/* fine scan range 0 < findRange_ < 1 */
static constexpr double findRange = 0.05;

/* settings for Auto Focus filter of IPU3 */
static struct ipu3_uapi_af_filter_config afFilterConfigDefault = {
	.y1_coeff_0 = { 0, 1, 3, 7 },
	.y1_coeff_1 = { 11, 13, 1, 2 },
	.y1_coeff_2 = { 8, 19, 34, 242 },
	.y1_sign_vec = 0x7fdffbfe,
	.y2_coeff_0 = { 0, 1, 6, 6 },
	.y2_coeff_1 = { 13, 25, 3, 0 },
	.y2_coeff_2 = { 25, 3, 177, 254 },
	.y2_sign_vec = 0x4e53ca72,
	.y_calc = { 8, 8, 8, 8 },
	.nf = { 0, 9, 0, 9, 0 },
};

Af::Af()
	: focus_(0), goodFocus_(0), currentVariance_(0.0), previousVariance_(0.0),
	  coarseComplete_(false), fineComplete_(false)
{
	maxStep_ = maxFocusSteps;
}

Af::~Af()
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Af::prepare (IPAContext &context, ipu3_uapi_params *params)
{
	const struct ipu3_uapi_grid_config &grid = context.configuration.af.afGrid;
	params->acc_param.af.grid_cfg = grid;
	params->acc_param.af.filter_config = afFilterConfigDefault;

	/* enable AF processing block */
	params->use.acc_af = 1;
}

/**
 * \brief Configure the Af given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Af::configure(IPAContext &context,
		  [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	/* determined focus value i.e. current focus value */
	context.frameContext.af.focus = 0;
	/* maximum variance of the AF statistics */
	context.frameContext.af.maxVariance = 0;
	/* the stable AF value flag. if it is true, the AF should be in a stable state. */
	context.frameContext.af.stable = false;
	/* AF buffer length */
	afRawBufferLen_ = context.configuration.af.afGrid.width *
			  context.configuration.af.afGrid.height;

	return 0;
}

/**
 * \brief AF coarse scan
 * \param[in] context The shared IPA context
 *
 */
void Af::afCoarseScan(IPAContext &context)
{
	if (coarseComplete_ == true)
		return;

	if (afScan(context, coarseSearchStep)) {
		coarseComplete_ = true;
		context.frameContext.af.maxVariance = 0;
		focus_ = context.frameContext.af.focus - (context.frameContext.af.focus * findRange);
		context.frameContext.af.focus = focus_;
		previousVariance_ = 0;
		maxStep_ = std::clamp(static_cast<uint32_t>(focus_ + (focus_ * findRange)), 0U, maxFocusSteps);
	}
}

/**
 * \brief AF fine scan
 *
 * Finetune the lens position with moving 1 step for each variance computation.
 *
 * \param[in] context The shared IPA context
 *
 */
void Af::afFineScan(IPAContext &context)
{
	if (coarseComplete_ != true)
		return;

	if (afScan(context, fineSearchStep)) {
		context.frameContext.af.stable = true;
		fineComplete_ = true;
	}
}

/**
 * \brief AF reset
 *
 * Reset all the parameter to start over the AF process.
 *
 * \param[in] context The shared IPA context
 *
 */
void Af::afReset(IPAContext &context)
{
	context.frameContext.af.maxVariance = 0;
	context.frameContext.af.focus = 0;
	focus_ = 0;
	context.frameContext.af.stable = false;
	ignoreCounter_ = ignoreFrame;
	previousVariance_ = 0.0;
	coarseComplete_ = false;
	fineComplete_ = false;
	maxStep_ = maxFocusSteps;
}

/**
 * \brief AF scan
 * \param[in] context The shared IPA context
 *
 * \return True, if it finds a AF value.
 */
bool Af::afScan(IPAContext &context, int min_step)
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
	LOG(IPU3Af, Debug) << "Variance previous: "
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
	uint32_t z = 0;

	y_item = (y_table_item_t *)stats->af_raw_buffer.y_table;

	/**
	 * Calculate the mean and the variance AF statistics, since IPU3 only determine the AF value
	 * for a given grid.
	 * For coarse: low pass results are used.
	 * For fine: high pass results are used.
	 */
	if (coarseComplete_) {
		for (z = 0; z < afRawBufferLen_; z++) {
			total = total + y_item[z].y2_avg;
		}
		mean = total / afRawBufferLen_;

		for (z = 0; z < afRawBufferLen_; z++) {
			var_sum = var_sum + ((y_item[z].y2_avg - mean) * (y_item[z].y2_avg - mean));
		}
	} else {
		for (z = 0; z < afRawBufferLen_; z++) {
			total = total + y_item[z].y1_avg;
		}
		mean = total / afRawBufferLen_;

		for (z = 0; z < afRawBufferLen_; z++) {
			var_sum = var_sum + ((y_item[z].y1_avg - mean) * (y_item[z].y1_avg - mean));
		}
	}

	/* Determine the average variance of the frame. */
	currentVariance_ = static_cast<double>(var_sum) / static_cast<double>(afRawBufferLen_);
	LOG(IPU3Af, Debug) << "variance: " << currentVariance_;

	if (context.frameContext.af.stable == true) {
		const uint32_t diff_var = std::abs(currentVariance_ - context.frameContext.af.maxVariance);
		const double var_ratio = diff_var / context.frameContext.af.maxVariance;
		LOG(IPU3Af, Debug) << "Rate of variance change: "
				   << var_ratio
				   << " current focus step: "
				   << context.frameContext.af.focus;
		/**
		 * If the change ratio of contrast is over Maxchange_ (out of focus),
		 * trigger AF again.
		 */
		if (var_ratio > MaxChange) {
			if (ignoreCounter_ == 0) {
				afReset(context);
			} else
				ignoreCounter_--;
		} else
			ignoreCounter_ = ignoreFrame;
	} else {
		if (ignoreCounter_ != 0)
			ignoreCounter_--;
		else {
			afCoarseScan(context);
			afFineScan(context);
		}
	}
}

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
