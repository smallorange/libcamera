/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 *
 * af.cpp - IPU3 auto focus algorithm
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
 * \var kAfMinGridWidth
 * \brief the minimum width of AF grid.
 * The minimum grid horizontal dimensions, in number of grid blocks(cells).
*/

/**
 * \var kAfMinGridHeight
 * \brief the minimum height of AF grid.
 * The minimum grid vertical dimensions, in number of grid blocks(cells).
*/

/**
 * \var kAfMaxGridWidth
 * \brief the maximum width of AF grid.
 * The maximum grid horizontal dimensions, in number of grid blocks(cells).
*/

/**
 * \var kAfMaxGridHeight
 * \brief The maximum height of AF grid.
 * The maximum grid vertical dimensions, in number of grid blocks(cells).
*/

/**
 * \var kAfMinGridBlockWidth
 * \brief The minimum block size of the width.
 */

/**
 * \var kAfMinGridBlockHeight
 * \brief The minimum block size of the height.
 */

/**
 * \def kAfMaxGridBlockWidth
 * \brief The maximum block size of the width.
 */

/**
 * \var kAfMaxGridBlockHeight
 * \brief The maximum block size of the height.
 */

/**
 * \var kAfDefaultHeightPerSlice
 * \brief The default number of blocks in vertical axis per slice.
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::ipu3::algorithms {

/**
 * \class Af
 * \brief An auto-focus algorithm based on IPU3 statistics
 *
 * This algorithm is used to determine the position of the lens to make a
 * focused image. The IPU3 AF processing block computes the statistics that
 * are composed by two types of filtered value and stores in a AF buffer.
 * Typically, for a clear image, it has a relatively higher contrast than a
 * blurred one. Therefore, if an image with the highest contrast can be
 * found through the scan, the position of the len indicates to a clearest
 * image.
 *
 */

LOG_DEFINE_CATEGORY(IPU3Af)

/**
 * Maximum focus steps of the VCM control
 * \todo should be obtained from the VCM driver
 */
static constexpr uint32_t kMaxFocusSteps = 1023;

/* Minimum focus step for searching appropriate focus */
static constexpr uint32_t kCoarseSearchStep = 30;
static constexpr uint32_t kFineSearchStep = 1;

/* Max ratio of variance change, 0.0 < kMaxChange < 1.0 */
static constexpr double kMaxChange = 0.5;

/* The numbers of frame to be ignored, before performing focus scan. */
static constexpr uint32_t kIgnoreFrame = 10;

/* Fine scan range 0 < kFineRange < 1 */
static constexpr double kFineRange = 0.05;

/* Settings for IPU3 AF filter */
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
	: focus_(0), bestFocus_(0), currentVariance_(0.0), previousVariance_(0.0),
	  coarseCompleted_(false), fineCompleted_(false)
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Af::prepare(IPAContext &context, ipu3_uapi_params *params)
{
	const struct ipu3_uapi_grid_config &grid = context.configuration.af.afGrid;
	params->acc_param.af.grid_cfg = grid;
	params->acc_param.af.filter_config = afFilterConfigDefault;

	/* Enable AF processing block */
	params->use.acc_af = 1;
}

/**
 * \brief Configure the Af given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 * \return 0
 */
int Af::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	struct ipu3_uapi_grid_config &grid = context.configuration.af.afGrid;
	grid.width = kAfMinGridWidth;
	grid.height = kAfMinGridHeight;
	grid.block_width_log2 = kAfMinGridBlockWidth;
	grid.block_height_log2 = kAfMinGridBlockHeight;
	grid.height_per_slice = kAfDefaultHeightPerSlice;

	/* x_start and y start are default to BDS center */
	grid.x_start = (configInfo.bdsOutputSize.width / 2) -
		       (((grid.width << grid.block_width_log2) / 2));
	grid.y_start = (configInfo.bdsOutputSize.height / 2) -
		       (((grid.height << grid.block_height_log2) / 2));

	/* x_start and y_start should be even */
	grid.x_start = (grid.x_start / 2) * 2;
	grid.y_start = (grid.y_start / 2) * 2;
	grid.y_start = grid.y_start | IPU3_UAPI_GRID_Y_START_EN;

	/* Initial max focus step */
	maxStep_ = kMaxFocusSteps;

	/* Initial focus value */
	context.frameContext.af.focus = 0;
	/* Maximum variance of the AF statistics */
	context.frameContext.af.maxVariance = 0;
	/* The stable AF value flag. if it is true, the AF should be in a stable state. */
	context.frameContext.af.stable = false;

	return 0;
}

/**
 * \brief AF coarse scan
 * Find a near focused image using a coarse step. The step is determined by coarseSearchStep.
 * \param[in] context The shared IPA context
 */
void Af::afCoarseScan(IPAContext &context)
{
	if (coarseCompleted_)
		return;

	if (afNeedIgnoreFrame())
		return;

	if (afScan(context, kCoarseSearchStep)) {
		coarseCompleted_ = true;
		context.frameContext.af.maxVariance = 0;
		focus_ = context.frameContext.af.focus -
			 (context.frameContext.af.focus * kFineRange);
		context.frameContext.af.focus = focus_;
		previousVariance_ = 0;
		maxStep_ = std::clamp(focus_ + static_cast<uint32_t>((focus_ * kFineRange)),
				      0U, kMaxFocusSteps);
	}
}

/**
 * \brief AF fine scan
 * Find an optimum lens position with moving 1 step for each search.
 * \param[in] context The shared IPA context
 */
void Af::afFineScan(IPAContext &context)
{
	if (!coarseCompleted_)
		return;

	if (afNeedIgnoreFrame())
		return;

	if (afScan(context, kFineSearchStep)) {
		context.frameContext.af.stable = true;
		fineCompleted_ = true;
	}
}

/**
 * \brief AF reset
 * Reset all the parameters to start over the AF process.
 * \param[in] context The shared IPA context
 */
void Af::afReset(IPAContext &context)
{
	if (afNeedIgnoreFrame())
		return;

	context.frameContext.af.maxVariance = 0;
	context.frameContext.af.focus = 0;
	focus_ = 0;
	context.frameContext.af.stable = false;
	ignoreCounter_ = kIgnoreFrame;
	previousVariance_ = 0.0;
	coarseCompleted_ = false;
	fineCompleted_ = false;
	maxStep_ = kMaxFocusSteps;
}

/**
 * \brief AF variance comparison.
 * It always picks the largest variance to replace the previous one. The image
 * with a larger variance also indicates it is a clearer image than previous
 * one. If it finds the negative sign of derivative, it returns immediately.
 * \param[in] context The IPA context
 * \param min_step The VCM movement step.
 * \return True, if it finds a AF value.
 */
bool Af::afScan(IPAContext &context, int min_step)
{
	if (focus_ > maxStep_) {
		/* If reach the max step, move lens to the position. */
		context.frameContext.af.focus = bestFocus_;
		return true;
	} else {
		/*
		 * Find the maximum of the variance by estimating its
		 * derivative. If the direction changes, it means we have
		 * passed a maximum one step before.
		*/
		if ((currentVariance_ - context.frameContext.af.maxVariance) >=
		    -(context.frameContext.af.maxVariance * 0.1)) {
			/*
			 * Positive and zero derivative:
			 * The variance is still increasing. The focus could be
			 * increased for the next comparison. Also, the max variance
			 * and previous focus value are updated.
			 */
			bestFocus_ = focus_;
			focus_ += min_step;
			context.frameContext.af.focus = focus_;
			context.frameContext.af.maxVariance = currentVariance_;
		} else {
			/*
			 * Negative derivative:
			 * The variance starts to decrease which means the maximum
			 * variance is found. Set focus step to previous good one
			 * then return immediately.
			 */
			context.frameContext.af.focus = bestFocus_;
			return true;
		}
	}

	previousVariance_ = currentVariance_;
	LOG(IPU3Af, Debug) << " Previous step is "
			   << bestFocus_
			   << " Current step is "
			   << focus_;
	return false;
}

/**
 * \brief Determine the frame to be ignored.
 * \return Return true the frame is ignored.
 * \return Return false the frame should be processed.
 */
bool Af::afNeedIgnoreFrame()
{
	if (ignoreCounter_ == 0)
		return false;
	else
		ignoreCounter_--;
	return true;
}

/**
 * \brief Reset frame ignore counter.
 */
void Af::afIgnoreFrameReset()
{
	ignoreCounter_ = kIgnoreFrame;
}

/**
 * \brief Estemate variance
 */
double Af::afEstemateVariance(y_table_item_t *y_item, uint32_t len,
			      bool isY1)
{
	uint32_t z = 0;
	uint32_t total = 0;
	double mean;
	double var_sum = 0;

	for (z = 0; z < len; z++) {
		if(isY1)
			total = total + y_item[z].y1_avg;
		else
			total = total + y_item[z].y2_avg;

	}
	mean = total / len;
	for (z = 0; z < len; z++) {
		if(isY1)
			var_sum = var_sum +
				  pow((y_item[z].y1_avg - mean), 2);
		else
			var_sum = var_sum +
				  pow((y_item[z].y2_avg - mean), 2);
	}

	return var_sum / static_cast<double>(len);
}

/**
 * \brief Determine out-of-focus situation.
 * Out-of-focus means that the variance change rate for a focused and a new
 * variance is greater than a threshold.
 * \param context The IPA context.
 * \return If it is out-of-focus, return true.
 * \return If is is focused, return false.
 */
bool Af::afIsOutOfFocus(IPAContext context)
{
	const uint32_t diff_var = std::abs(currentVariance_ -
					   context.frameContext.af.maxVariance);
	const double var_ratio = diff_var / context.frameContext.af.maxVariance;
	LOG(IPU3Af, Debug) << "Variance change rate: "
			   << var_ratio
			   << " Current VCM step: "
			   << context.frameContext.af.focus;
	if (var_ratio > kMaxChange)
		return true;
	else
		return false;
}

/**
 * \brief Determine the max contrast image and lens position.
 * Ideally, a clear image also has a raletively higher contrast. So, every
 * images for each focus step should be tested to find a optimal focus step.
 * The Hill Climbing Algorithm[1] is used to find the maximum variance of the
 * AF statistic which is the AF output of IPU3. The focus step is increased
 * then the variance of the AF statistic is estimated. If it finds the negative
 * derivative which means we just passed the peak, the best focus is found.
 *
 * [1] Hill Climbing Algorithm, https://en.wikipedia.org/wiki/Hill_climbing
 * \param[in] context The IPA context.
 * \param[in] stats The statistic buffer of IPU3.
 */
void Af::process(IPAContext &context, const ipu3_uapi_stats_3a *stats)
{
	y_table_item_t y_item[IPU3_UAPI_AF_Y_TABLE_MAX_SIZE / sizeof(y_table_item_t)];
	uint32_t afRawBufferLen;

	/* Evaluate the AF buffer length */
	afRawBufferLen = context.configuration.af.afGrid.width *
			  context.configuration.af.afGrid.height;

	memcpy(y_item, stats->af_raw_buffer.y_table,
	       afRawBufferLen * sizeof(y_table_item_t));

	/*
	 * Calculate the mean and the variance of AF statistics for a given grid.
	 * For coarse: y1 are used.
	 * For fine: y2 results are used.
	 */
	if (coarseCompleted_)
		currentVariance_ = afEstemateVariance(y_item, afRawBufferLen, false);
	else
		currentVariance_ = afEstemateVariance(y_item, afRawBufferLen, true);

	if (!context.frameContext.af.stable) {
		afCoarseScan(context);
		afFineScan(context);
	} else {
		if (afIsOutOfFocus(context))
			afReset(context);
		else
			afIgnoreFrameReset();
	}
}

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
