/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_lens.cpp - A camera lens
 */

#include "libcamera/internal/camera_lens.h"

#include <libcamera/base/utils.h>

#include "libcamera/internal/v4l2_subdevice.h"

/**
 * \file camera_lens.h
 * \brief A camera lens controller
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(CameraLens)

/**
 * \class CameraLens
 * \brief A camera lens based on V4L2 subdevices
 *
 * The CameraLens class eases handling of lens for pipeline handlers by
 * hiding the details of the V4L2 subdevice kernel API and caching lens
 * information.
 */

/**
 * \brief Construct a CameraLens
 * \param[in] entity The media entity backing the camera lens controller
 *
 * Once constructed the instance must be initialized with init().
 */
CameraLens::CameraLens(const MediaEntity *entity)
	: entity_(entity)
{
}

/**
 * \brief Destroy a CameraLens
 */
CameraLens::~CameraLens() = default;

/**
 * \brief Initialize the camera lens instance
 *
 * This function performs the initialisation steps of the CameraLens that may
 * fail. It shall be called once and only once after constructing the instance.
 *
 * \return 0 on success or a negative error code otherwise
 */
int CameraLens::init()
{
	if (entity_->function() != MEDIA_ENT_F_LENS) {
		LOG(CameraLens, Error)
			<< "Invalid lens function "
			<< utils::hex(entity_->function());
		return -EINVAL;
	}

	/* Create and open the subdev. */
	subdev_ = std::make_unique<V4L2Subdevice>(entity_);
	int ret = subdev_->open();
	if (ret < 0)
		return ret;
	printf("Camera len init\n");
	ret = validateLensDriver();
	if (ret)
		return ret;
printf("Camera len init 22\n");
	model_ = subdev_->model();

	// init max Kate
	ControlList lensCtrls(subdev_->controls());
	subdev_->getAttributes(&lensCtrls);

	return 0;
}

/**
 * \brief This function sets the focal point of the lens to a specific position.
 * \param[in] position The focal point of the lens
 *
 * This function sets the value of focal point of the lens as in \a position.
 *
 * \return 0 on success or -EINVAL otherwise
 */
int CameraLens::setFocusPosition(int32_t position)
{
	ControlList lensCtrls(subdev_->controls());
	lensCtrls.set(V4L2_CID_FOCUS_ABSOLUTE, static_cast<int32_t>(position));

	if (subdev_->setControls(&lensCtrls))
		return -EINVAL;

	return 0;
}

int CameraLens::getFocusCapabilityies()
{
	ControlValue retVal;
	ControlList lensCtrls(subdev_->controls());
	int32_t ret = 2023;

	lensCtrls.set(V4L2_CID_FOCUS_ABSOLUTE, static_cast<int32_t>(ret));
	subdev_->getAttributes(&lensCtrls);

	retVal = lensCtrls.get(V4L2_CID_FOCUS_ABSOLUTE);
	ret = retVal.get<int32_t>();

	printf("ret %d\n", ret);

	return ret;
}

int CameraLens::validateLensDriver()
{
	int ret = 0;
	static constexpr uint32_t mandatoryControls[] = {
		V4L2_CID_FOCUS_ABSOLUTE,
	};

	const ControlInfoMap &controls = subdev_->controls();
	for (uint32_t ctrl : mandatoryControls) {
		if (!controls.count(ctrl)) {
			LOG(CameraLens, Error)
				<< "Mandatory V4L2 control " << utils::hex(ctrl)
				<< " not available";
			ret = -EINVAL;
		}
	}

	if (ret) {
		LOG(CameraLens, Error)
			<< "The lens kernel driver needs to be fixed";
		LOG(CameraLens, Error)
			<< "See Documentation/lens_driver_requirements.rst in"
			<< " the libcamera sources for more information";
		return ret;
	}

	return ret;
}

/**
 * \fn CameraLens::model()
 * \brief Retrieve the lens model name
 *
 * The lens model name is a free-formed string that uniquely identifies the
 * lens model.
 *
 * \return The lens model name
 */

std::string CameraLens::logPrefix() const
{
	return "'" + entity_->name() + "'";
}

/**
 * \fn CameraLens::controls()
 * \brief Retrieve the V4L2 controls of the lens' subdev
 *
 * \return A map of the V4L2 controls supported by the lens' driver
 */
const ControlInfoMap &CameraLens::controls() const
{
	return subdev_->controls();
}

} /* namespace libcamera */
