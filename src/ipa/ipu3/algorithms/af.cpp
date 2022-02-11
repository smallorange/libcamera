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
 * \brief the maximum height of AF grid.
 * The maximum grid vertical dimensions, in number of grid blocks(cells).
*/

/**
 * \var kAfMinGridBlockWidth
 * \brief the minimum block size of the width.
 */

/**
 * \var kAfMinGridBlockHeight
 * \brief the minimum block size of the height.
 */

/**
 * \def kAfMaxGridBlockWidth
 * \brief the maximum block size of the width.
 */

/**
 * \var kAfMaxGridBlockHeight
 * \brief the maximum block size of the height.
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

/* minimum focus step for searching appropriate focus */
static constexpr uint32_t kCoarseSearchStep = 30;
static constexpr uint32_t kFineSearchStep = 1;

/* max ratio of variance change, 0.0 < kMaxChange < 1.0 */
static constexpr double kMaxChange = 0.5;

/* the numbers of frame to be ignored, before performing focus scan. */
static constexpr uint32_t kIgnoreFrame = 10;

/* fine scan range 0 < kFineRange < 1 */
static constexpr double kFineRange = 0.05;

/* settings for IPU3 AF filter */
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

	/* enable AF processing block */
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

	/* inital max focus step */
	maxStep_ = kMaxFocusSteps;

	/* determined focus value i.e. current focus value */
	context.frameContext.af.focus = 0;
	/* maximum variance of the AF statistics */
	context.frameContext.af.maxVariance = 0;
	/* the stable AF value flag. if it is true, the AF should be in a stable state. */
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
	if (coarseCompleted_ == true)
		return;

	if (afScan(context, kCoarseSearchStep)) {
		coarseCompleted_ = true;
		context.frameContext.af.maxVariance = 0;
		focus_ = context.frameContext.af.focus - (context.frameContext.af.focus * kFineRange);
		context.frameContext.af.focus = focus_;
		previousVariance_ = 0;
		maxStep_ = std::clamp(static_cast<uint32_t>(focus_ + (focus_ * kFineRange)), 0U, kMaxFocusSteps);
	}
}

/**
 * \brief AF fine scan
 *
 * Find an optimum lens position with moving 1 step for each search.
 *
 * \param[in] context The shared IPA context
 *
 */
void Af::afFineScan(IPAContext &context)
{
	if (coarseCompleted_ != true)
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
 * This fuction compares the previous and current variance. It always picks
 * the largest variance to replace the previous one. The image with a larger
 * variance also indicates it is a clearer image than previous one. If it
 * finds the negative sign of derivative, it returns immediately.
 * \param[in] context The shared IPA context
 * \return True, if it finds a AF value.
 */
bool Af::afScan(IPAContext &context, int min_step)
{
	if (focus_ > maxStep_) {
		/* if reach the max step, move lens to the position. */
		context.frameContext.af.focus = goodFocus_;
		return true;
	} else {
		/*
		 * find the maximum of the variance by estimating its
		 * derivative. If the direction changes, it means we have
		 * passed a maximum one step before.
		*/
		if ((currentVariance_ - context.frameContext.af.maxVariance) >=
		    -(context.frameContext.af.maxVariance * 0.1)) {
			/*
			 * positive and zero derivative:
			 * The variance is still increasing. The focus could be
			 * increased for the next comparison. Also, the max variance
			 * and previous focus value are updated.
			 */
			goodFocus_ = focus_;
			focus_ += min_step;
			context.frameContext.af.focus = focus_;
			context.frameContext.af.maxVariance = currentVariance_;
		} else {
			/*
			 * negative derivative:
			 * The variance starts to decrease which means the maximum
			 * variance is found. Set focus step to previous good one
			 * then returnimmediately.
			 */
			context.frameContext.af.focus = goodFocus_;
			return true;
		}
	}

	previousVariance_ = currentVariance_;
	LOG(IPU3Af, Debug) << " Previous step is "
			   << goodFocus_
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
 * 
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
				  powf64((y_item[z].y1_avg - mean), 2);
		else
			var_sum = var_sum +
				  powf64((y_item[z].y2_avg - mean), 2);
	}

	return var_sum / static_cast<double>(len);
}

/**
 * \brief Determine the max contrast image and lens position.
 * y_table is the AF statistic of IPU3 and is composed of two kinds of filtered
 * values. Based on this, the variance of a image could be used to determine
 * the clearness of the given image. Ideally, a clear image also has a
 * raletively higher contrast. So, the VCM is moved step by step and variance
 * of each frame are calculated to find a maximum variance which corresponds
 * with a specific focus step. If it is found, that is the best focus step of
 * current scene.
 * \param[in] context The shared IPA context.
 * \param[in] stats The statistic buffer of 3A of IPU3.
 */
void Af::process(IPAContext &context, const ipu3_uapi_stats_3a *stats)
{
	uint32_t total = 0;
	double mean;
	uint64_t var_sum = 0;
	y_table_item_t y_item[IPU3_UAPI_AF_Y_TABLE_MAX_SIZE / sizeof(y_table_item_t)];
	uint32_t z = 0;
	uint32_t afRawBufferLen_;

	/* evaluate the AF buffer length */
	afRawBufferLen_ = context.configuration.af.afGrid.width *
			  context.configuration.af.afGrid.height;

	memcpy(y_item, stats->af_raw_buffer.y_table, afRawBufferLen_ * sizeof(y_table_item_t));

	/*
	 * calculate the mean and the variance of AF statistics for a given grid.
	 *
	 * For coarse: y1 are used.
	 * For fine: y2 results are used.
	 */
	if (coarseCompleted_) {
		for (z = 0; z < afRawBufferLen_; z++) {
			total = total + y_item[z].y2_avg;
		}
		mean = total / afRawBufferLen_;

		for (z = 0; z < afRawBufferLen_; z++) {
			var_sum = var_sum + ((y_item[z].y2_avg - mean) *
					     (y_item[z].y2_avg - mean));
		}
	} else {
		for (z = 0; z < afRawBufferLen_; z++) {
			total = total + y_item[z].y1_avg;
		}
		mean = total / afRawBufferLen_;

		for (z = 0; z < afRawBufferLen_; z++) {
			var_sum = var_sum + ((y_item[z].y1_avg - mean) *
					     (y_item[z].y1_avg - mean));
		}
	}

	/* determine the average variance of the AF statistic. */
	currentVariance_ = static_cast<double>(var_sum) / static_cast<double>(afRawBufferLen_);

	if (context.frameContext.af.stable == true) {
		const uint32_t diff_var = std::abs(currentVariance_ -
						   context.frameContext.af.maxVariance);
		const double var_ratio = diff_var / context.frameContext.af.maxVariance;
		LOG(IPU3Af, Debug) << "Variance change rate: "
				   << var_ratio
				   << " Current VCM step: "
				   << context.frameContext.af.focus;

		/*
		 * If the change rate is greater than kMaxChange (out of focus),
		 * trigger AF again.
		 */
		if (var_ratio > kMaxChange) {
			if (!afNeedIgnoreFrame())
				afReset(context);
		} else
			afIgnoreFrameReset();
	} else {
		if (!afNeedIgnoreFrame()) {
			afCoarseScan(context);
			afFineScan(context);
		}
	}
}

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
