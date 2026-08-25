/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2026 Frederic Laing
 */

#include <array>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <string_view>

#include <libcamera/formats.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/sensor_cfa_layout.h"
#include "libcamera/internal/software_isp/quad_bayer.h"

#include "test.h"

using namespace libcamera;

class QuadBayerTest : public Test
{
protected:
	int run()
	{
		const SensorCfaLayout imx371Layout = sensorCfaLayout("imx371");
		const SensorCfaLayout ordinaryLayout = sensorCfaLayout("imx519");
		constexpr std::array<std::string_view, 4> imx708Models = {
			"imx708", "imx708_wide", "imx708_noir", "imx708_wide_noir"
		};

		if (imx371Layout.nativeCellSize != Size(2, 2) ||
		    imx371Layout.softwareIspInputCellSize != Size(2, 2) ||
		    imx371Layout.minimumDebinFactor() != 4) {
			std::cerr << "unexpected IMX371 CFA layout" << std::endl;
			return TestFail;
		}

		for (std::string_view model : imx708Models) {
			const SensorCfaLayout layout = sensorCfaLayout(model);
			if (layout.nativeCellSize != Size(2, 2) ||
			    layout.softwareIspInputCellSize != Size(1, 1) ||
			    layout.minimumDebinFactor() != 4 ||
			    softwareIspNeedsCellCollapse(model)) {
				std::cerr << "unexpected IMX708 CFA layout for " << model
					  << std::endl;
				return TestFail;
			}
		}

		if (ordinaryLayout.nativeCellSize != Size(1, 1) ||
		    ordinaryLayout.softwareIspInputCellSize != Size(1, 1) ||
		    ordinaryLayout.minimumDebinFactor() != 2) {
			std::cerr << "unexpected ordinary Bayer CFA layout" << std::endl;
			return TestFail;
		}

		if (!softwareIspNeedsCellCollapse("imx371") ||
		    softwareIspNeedsCellCollapse("imx519") ||
		    softwareIspNeedsCellCollapse("")) {
			std::cerr << "software ISP quad-cell collapse selection is not narrow"
				  << std::endl;
			return TestFail;
		}

		if (!isQuadBayerInputFormatSupported(formats::SRGGB10_CSI2P) ||
		    isQuadBayerInputFormatSupported(formats::SRGGB8) ||
		    isQuadBayerInputFormatSupported(formats::SRGGB10) ||
		    isQuadBayerInputFormatSupported(formats::SRGGB12_CSI2P)) {
			std::cerr << "unexpected quad Bayer input format support" << std::endl;
			return TestFail;
		}

		if (!isQuadBayerInputSizeSupported({ 4656, 3456 }) ||
		    isQuadBayerInputSizeSupported({ 4655, 3456 }) ||
		    isQuadBayerInputSizeSupported({ 4656, 3454 })) {
			std::cerr << "unexpected quad Bayer input size support" << std::endl;
			return TestFail;
		}

		if (quadBayerLogicalSize({ 4656, 3456 }) != Size(2328, 1728)) {
			std::cerr << "unexpected IMX371 logical size" << std::endl;
			return TestFail;
		}

		if (softwareIspViewport({ 4656, 3456 }, { 2328, 1728 }, true) !=
			    Rectangle(0, 0, 2328, 1728) ||
		    softwareIspViewport({ 4656, 3456 }, { 1600, 1200 }, true) !=
			    Rectangle(0, 0, 1600, 1200)) {
			std::cerr << "quad Bayer viewport does not match the logical FBO" << std::endl;
			return TestFail;
		}
		if (softwareIspViewport({ 4656, 3456 }, { 1600, 1200 }, false) !=
		    Rectangle(0, 0, 4656, 3456)) {
			std::cerr << "ordinary Bayer viewport no longer uses physical geometry" << std::endl;
			return TestFail;
		}

		if (quadBayerStatsWindow({ 4656, 3456 }, { 1600, 1200 }) !=
		    Rectangle(24, 0, 4608, 3456)) {
			std::cerr << "quad Bayer statistics window is not centered physically" << std::endl;
			return TestFail;
		}

		const auto ordinaryCoordinates = softwareIspTextureCoordinates(
			{ 4656, 3456 }, { 0, 0, 4656, 3456 }, false);
		constexpr std::array<float, 8> fullTexture = {
			0.0f,
			0.0f,
			0.0f,
			1.0f,
			1.0f,
			1.0f,
			1.0f,
			0.0f,
		};
		if (ordinaryCoordinates != fullTexture) {
			std::cerr << "ordinary Bayer texture coordinates changed" << std::endl;
			return TestFail;
		}

		const Rectangle previewWindow =
			quadBayerStatsWindow({ 4656, 3456 }, { 1454, 1080 });
		const auto quadCoordinates = softwareIspTextureCoordinates(
			{ 4656, 3456 }, previewWindow, true);
		constexpr float tolerance = 0.000001f;
		const std::array<float, 8> expectedQuad = {
			0.0f,
			0.0f,
			0.0f,
			1.0f,
			4652.0f / 4656.0f,
			1.0f,
			4652.0f / 4656.0f,
			0.0f,
		};
		for (unsigned int i = 0; i < quadCoordinates.size(); ++i) {
			if (std::abs(quadCoordinates[i] - expectedQuad[i]) > tolerance) {
				std::cerr << "quad Bayer texture coordinate " << i
					  << " is incorrect" << std::endl;
				return TestFail;
			}
		}

		if (quadBayerCellOrigin({ 4656, 3456 }, { 4656, 3456 }) != Point(4654, 3454) ||
		    quadBayerCellOrigin({ 4656, 3456 }, { -2, -2 }) != Point(0, 0)) {
			std::cerr << "quad Bayer edge sampling is not clamped to complete cells" << std::endl;
			return TestFail;
		}

		if (quadBayerOrderShifts(BayerFormat::RGGB) != Point(0, 0) ||
		    quadBayerOrderShifts(BayerFormat::GRBG) != Point(2, 0) ||
		    quadBayerOrderShifts(BayerFormat::GBRG) != Point(0, 2) ||
		    quadBayerOrderShifts(BayerFormat::BGGR) != Point(2, 2)) {
			std::cerr << "quad Bayer order shifts are incorrect" << std::endl;
			return TestFail;
		}

		constexpr std::array<std::array<uint16_t, 4>, 4> orderTiles = { {
			{ 100, 200, 204, 300 }, /* RGGB */
			{ 200, 100, 300, 204 }, /* GRBG */
			{ 200, 300, 100, 204 }, /* GBRG */
			{ 300, 200, 204, 100 }, /* BGGR */
		} };
		constexpr std::array<BayerFormat::Order, 4> orders = {
			BayerFormat::RGGB,
			BayerFormat::GRBG,
			BayerFormat::GBRG,
			BayerFormat::BGGR,
		};
		for (unsigned int index = 0; index < orders.size(); ++index) {
			auto rgb = normalizeQuadBayerTile(
				[&](unsigned int x, unsigned int y) {
					return orderTiles[index][y * 2 + x];
				},
				orders[index]);
			if (rgb != std::array<uint16_t, 3>{ 100, 202, 300 }) {
				std::cerr << "quad Bayer order normalization failed" << std::endl;
				return TestFail;
			}
		}

		/* Physical quad-RGGB tile: 2x2 samples for each logical colour. */
		constexpr std::array<uint16_t, 16> physical = {
			100,
			104,
			200,
			204,
			108,
			112,
			208,
			212,
			300,
			304,
			400,
			404,
			308,
			312,
			408,
			412,
		};
		constexpr std::array<uint16_t, 4> expected = { 106, 206, 306, 406 };

		for (unsigned int y = 0; y < 2; ++y) {
			for (unsigned int x = 0; x < 2; ++x) {
				uint16_t value = averageQuadBayerCell(
					[&](unsigned int px, unsigned int py) {
						return physical[py * 4 + px];
					},
					x, y);
				if (value != expected[y * 2 + x]) {
					std::cerr << "unexpected logical sample at " << x << ',' << y
						  << ": " << value << std::endl;
					return TestFail;
				}
			}
		}

		return TestPass;
	}
};

TEST_REGISTER(QuadBayerTest)
