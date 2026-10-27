/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once

#include <cstdint>

namespace libcamera {

/* Three signed 16-bit column contributions fit in independent 20-bit lanes.
 * Biasing each component makes ordinary 64-bit addition carry-free between
 * channels, including the complete int16_t range. The gamma table includes
 * constant tails to implement the original clamp without per-pixel branches.
 */
constexpr unsigned int kCcmLaneBits = 20;
constexpr unsigned int kCcmGammaOffset = 3 * 32768;
constexpr unsigned int kCcmGammaSize = 3 * 65535 + 1;

inline uint64_t packCcmColumn(int16_t red, int16_t green, int16_t blue)
{
	return uint64_t(int32_t(blue) + 32768) |
	       (uint64_t(int32_t(green) + 32768) << kCcmLaneBits) |
	       (uint64_t(int32_t(red) + 32768) << (2 * kCcmLaneBits));
}

inline unsigned int packedCcmIndex(uint64_t sum, unsigned int channel)
{
	return (sum >> (channel * kCcmLaneBits)) & ((1U << kCcmLaneBits) - 1);
}

} /* namespace libcamera */
