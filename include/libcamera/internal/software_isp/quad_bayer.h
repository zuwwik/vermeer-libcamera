/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Frederic Laing
 */

#pragma once

#include <algorithm>
#include <array>
#include <string_view>

#include <libcamera/geometry.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/sensor_cfa_layout.h"

namespace libcamera {

inline bool softwareIspNeedsCellCollapse(std::string_view model)
{
	return sensorCfaLayout(model).softwareIspNeedsCellCollapse();
}

inline Size quadBayerLogicalSize(const Size &physicalSize)
{
	return { physicalSize.width / 2, physicalSize.height / 2 };
}

inline bool isQuadBayerInputSizeSupported(const Size &size)
{
	return size.width >= 4 && size.height >= 4 &&
	       size.width % 4 == 0 && size.height % 4 == 0;
}

inline Rectangle softwareIspViewport(const Size &physicalSize,
				     const Size &outputSize,
				     bool quadBayer)
{
	const Size &viewportSize = quadBayer ? outputSize : physicalSize;
	return { 0, 0, viewportSize.width, viewportSize.height };
}

inline Rectangle quadBayerStatsWindow(const Size &physicalSize,
				      const Size &logicalOutputSize)
{
	Size statsSize = physicalSize;
	if (static_cast<uint64_t>(physicalSize.width) * logicalOutputSize.height >
	    static_cast<uint64_t>(physicalSize.height) * logicalOutputSize.width) {
		statsSize.width = static_cast<uint64_t>(physicalSize.height) *
				  logicalOutputSize.width / logicalOutputSize.height;
		statsSize.width &= ~3U;
	} else {
		statsSize.height = static_cast<uint64_t>(physicalSize.width) *
				   logicalOutputSize.height / logicalOutputSize.width;
		statsSize.height &= ~3U;
	}
	return {
		static_cast<int>(((physicalSize.width - statsSize.width) / 2) & ~3U),
		static_cast<int>(((physicalSize.height - statsSize.height) / 2) & ~3U),
		statsSize.width,
		statsSize.height,
	};
}

inline std::array<float, 8> softwareIspTextureCoordinates(const Size &physicalSize,
							  const Rectangle &window,
							  bool quadBayer)
{
	if (!quadBayer) {
		return {
			0.0f,
			0.0f,
			0.0f,
			1.0f,
			1.0f,
			1.0f,
			1.0f,
			0.0f,
		};
	}

	const float left = static_cast<float>(window.x) / physicalSize.width;
	const float top = static_cast<float>(window.y) / physicalSize.height;
	const float right = static_cast<float>(window.x + window.width) / physicalSize.width;
	const float bottom = static_cast<float>(window.y + window.height) / physicalSize.height;
	return {
		left,
		top,
		left,
		bottom,
		right,
		bottom,
		right,
		top,
	};
}

inline Point quadBayerCellOrigin(const Size &physicalSize, const Point &pixel)
{
	return {
		std::clamp(pixel.x & ~1, 0, static_cast<int>(physicalSize.width) - 2),
		std::clamp(pixel.y & ~1, 0, static_cast<int>(physicalSize.height) - 2),
	};
}

inline Point quadBayerOrderShifts(BayerFormat::Order order)
{
	switch (order) {
	case BayerFormat::RGGB:
		return { 0, 0 };
	case BayerFormat::GRBG:
		return { 2, 0 };
	case BayerFormat::GBRG:
		return { 0, 2 };
	case BayerFormat::BGGR:
		return { 2, 2 };
	default:
		return { -1, -1 };
	}
}

inline bool isQuadBayerInputFormatSupported(PixelFormat inputFormat)
{
	const BayerFormat bayerFormat = BayerFormat::fromPixelFormat(inputFormat);
	return bayerFormat.bitDepth == 10 &&
	       bayerFormat.packing == BayerFormat::Packing::CSI2 &&
	       quadBayerOrderShifts(bayerFormat.order).x >= 0;
}

template<typename Sample>
inline auto normalizeQuadBayerTile(Sample sample, BayerFormat::Order order)
{
	using Value = decltype(sample(0, 0));
	const Point redShift = quadBayerOrderShifts(order);
	const Point red(redShift.x / 2, redShift.y / 2);
	const Point blue(1 - red.x, 1 - red.y);
	const Value green0 = sample(1 - red.x, red.y);
	const Value green1 = sample(red.x, 1 - red.y);
	return std::array<Value, 3>{
		sample(red.x, red.y),
		static_cast<Value>((green0 + green1) / 2),
		sample(blue.x, blue.y),
	};
}

template<typename Sample>
inline auto averageQuadBayerCell(Sample sample, unsigned int x, unsigned int y)
{
	const unsigned int physicalX = x * 2;
	const unsigned int physicalY = y * 2;
	return (sample(physicalX, physicalY) +
		sample(physicalX + 1, physicalY) +
		sample(physicalX, physicalY + 1) +
		sample(physicalX + 1, physicalY + 1) + 2) /
	       4;
}

} /* namespace libcamera */
