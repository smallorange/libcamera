/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_lens.h - A camera lens
 */
#ifndef __LIBCAMERA_INTERNAL_CAMERA_LENS_H__
#define __LIBCAMERA_INTERNAL_CAMERA_LENS_H__

#include <memory>
#include <string>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>

namespace libcamera {

class MediaEntity;
class V4L2Subdevice;

class CameraLens : protected Loggable
{
public:
	explicit CameraLens(const MediaEntity *entity);
	~CameraLens();

	int init();
	int setFocusPostion(int32_t position);

	const std::string &model() const { return model_; }

protected:
	std::string logPrefix() const override;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraLens)

	int validateLensDriver();

	const MediaEntity *entity_;
	std::unique_ptr<V4L2Subdevice> subdev_;

	std::string model_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_CAMERA_LENS_H__ */