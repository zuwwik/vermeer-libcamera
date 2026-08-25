/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Frederic Laing
 */

#pragma once

#include <algorithm>
#include <string_view>

#include <libcamera/geometry.h>

namespace libcamera {

struct SensorCfaLayout {
	Size nativeCellSize;
	Size softwareIspInputCellSize;

	unsigned int minimumDebinFactor() const
	{
		return 2 * std::max(nativeCellSize.width, nativeCellSize.height);
	}

	bool softwareIspNeedsCellCollapse() const
	{
		return softwareIspInputCellSize != Size(1, 1);
	}
};

inline SensorCfaLayout sensorCfaLayout(std::string_view model)
{
	/*
	 * Native cell size describes the sensor construction. The software ISP
	 * input size separately describes whether that pipeline receives physical
	 * same-colour cells or an ordinary Bayer stream produced by the sensor.
	 */
	if (model == "imx371")
		return { { 2, 2 }, { 2, 2 } };
	if (model == "imx708" || model == "imx708_wide" ||
	    model == "imx708_noir" || model == "imx708_wide_noir")
		return { { 2, 2 }, { 1, 1 } };

	return { { 1, 1 }, { 1, 1 } };
}

} /* namespace libcamera */
