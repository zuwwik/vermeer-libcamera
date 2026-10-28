/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace libcamera {

enum class StillBayerOrder { BGGR, GBRG, GRBG, RGGB };

struct StillImageParameters {
	unsigned int width = 0, height = 0, stride = 0;
	StillBayerOrder order = StillBayerOrder::RGGB;
	float blackLevel = 64, whiteLevel = 1023;
	std::array<float, 3> whiteBalance{ 1, 1, 1 };
	std::array<float, 9> colorMatrix{ 1, 0, 0, 0, 1, 0, 0, 0, 1 };
	float gamma = 2.2, contrast = 1;
	float analogueGain = 1;
	double exposureProduct = 0;
	bool denoise = true, sharpen = true, highlightRolloff = true;
};

struct StillImage {
	unsigned int width = 0, height = 0;
	std::vector<uint8_t> rgb;
};

struct StillRawFrame {
	const uint8_t *data;
	size_t length;
	StillImageParameters parameters;
};

/* Offline RAW processing. Own the input until this call returns, and run it
 * on a background worker, independently of the live camera request queue.
 * The complete RAW10 array is preserved; there is no crop or resize.
 */
class StillImageProcessor
{
public:
	static int processRaw10(const uint8_t *data, size_t length,
				const StillImageParameters &parameters, StillImage *result);
	static int processRaw10Burst(const std::vector<StillRawFrame> &frames,
				     StillImage *result);
};

} /* namespace libcamera */
