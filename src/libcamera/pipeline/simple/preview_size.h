/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once

#include <cstdint>
#include <vector>
#include <libcamera/geometry.h>

namespace libcamera {

/* Select a real output size. This only sets a default, never resizes a frame. */
inline Size simplePreviewSize(const std::vector<Size> &sizes, const Size &fallback,
			      uint64_t maxPixels)
{
	if (!maxPixels)
		return fallback;
	Size best;
	uint64_t bestArea = 0;
	for (const Size &size : sizes) {
		uint64_t area = uint64_t(size.width) * size.height;
		if (size.width && size.height && area <= maxPixels && area > bestArea) {
			best = size;
			bestArea = area;
		}
	}
	return bestArea ? best : fallback;
}

} /* namespace libcamera */
