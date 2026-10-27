/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace libcamera::ipa::soft::algorithms {
struct ExposureMeterResult {
	double mean, highlight, factor;
};

template<size_t N>
ExposureMeterResult meterExposure(const std::array<uint32_t, N> &histogram,
				  double black, double wbScale, double target = 0.18)
{
	uint64_t total = 0;
	for (auto n : histogram)
		total += n;
	if (!total)
		return { 0, 0, 1 };
	double sum = 0, high = 0;
	uint64_t seen = 0;
	for (size_t i = 0; i < N; i++) {
		double value = std::clamp(((i + .5) * 256 / N - black) / (255 - black), 0., 1.);
		sum += value * histogram[i];
		seen += histogram[i];
	}
	// Use the 95th percentile to protect large highlights; allow small light sources.
	seen = 0;
	for (size_t i = 0; i < N; i++) {
		seen += histogram[i];
		if (seen * 100 >= total * 95) {
			high = std::clamp(((i + .5) * 256 / N - black) / (255 - black), 0., 1.);
			break;
		}
	}
	double mean = sum / total * std::clamp(wbScale, .5, 4.);
	high *= std::clamp(wbScale, .5, 4.);
	double metered = std::max(mean, high * target / .85);
	double ratio = std::clamp(target / std::max(metered, .0001), .0625, 8.);
	double ev = std::log2(ratio);
	// Fast coarse recovery, damped fine corrections, and a small deadband.
	double factor = std::abs(ev) < .08 ? 1 : std::pow(ratio, std::abs(ev) > .5 ? .85 : .35);
	return { mean, high, factor };
}
struct ExposureAllocation {
	int32_t exposure;
	double gain;
};
inline ExposureAllocation allocateExposure(double total, int32_t minExposure,
					   int32_t maxExposure, double minGain,
					   double maxGain)
{
	total = std::clamp(total, minExposure * minGain, maxExposure * maxGain);
	int32_t exposure = std::clamp<int32_t>(std::llround(total / minGain), minExposure, maxExposure);
	return { exposure, std::clamp(total / exposure, minGain, maxGain) };
}
} // namespace libcamera::ipa::soft::algorithms
