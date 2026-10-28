/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <libcamera/still_image.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <new>

namespace libcamera {
namespace {

int mirror(int x, int size)
{
	if (x < 0)
		return -x;
	return x >= size ? 2 * (size - 1) - x : x;
}

int color(StillBayerOrder order, int x, int y)
{
	static constexpr int colors[4][4] = {
		{ 2, 1, 1, 0 }, { 1, 2, 0, 1 },
		{ 1, 0, 2, 1 }, { 0, 1, 1, 2 },
	};
	return colors[static_cast<int>(order)][(y & 1) * 2 + (x & 1)];
}

int sample(const std::vector<uint16_t> &image, int w, int h, int x, int y)
{
	return image[size_t(mirror(y, h)) * w + mirror(x, w)];
}

float estimateNoise(const std::vector<uint16_t> &raw, int w, int h,
		    StillBayerOrder order, float gain)
{
	std::array<unsigned int, 2048> histogram{};
	unsigned int count = 0;
	for (int y = 2; y + 2 < h; y += 8) {
		for (int x = 2; x + 3 < w; x += 8) {
			int gx = x + (color(order, x, y) != 1);
			int d = sample(raw, w, h, gx - 2, y) -
				2 * sample(raw, w, h, gx, y) + sample(raw, w, h, gx + 2, y);
			histogram[std::abs(d)]++;
			count++;
		}
	}
	unsigned int total = 0, quartile = 0;
	for (; quartile + 1 < histogram.size(); quartile++) {
		total += histogram[quartile];
		if (total * 4 >= count)
			break;
	}
	/* For Gaussian noise, the lower quartile of |second difference| is
	 * approximately .7805 sigma. Bound it conservatively by gain so that
	 * a detailed scene cannot turn into a strong smoothing instruction.
	 */
	return std::clamp(quartile / .7805f, .5f, 1.f + 2.f * std::sqrt(gain));
}

int luminance(const uint8_t *p)
{
	return (77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8;
}

int validateRaw(const uint8_t *data, size_t length, const StillImageParameters &p)
{
	if (!data || p.width < 4 || p.height < 4 || p.width > 16384 || p.height > 16384 ||
	    p.width % 4 || p.height % 2 || p.stride < p.width * 5 / 4 ||
	    uint64_t(p.stride) * p.height > length || static_cast<unsigned>(p.order) > 3 ||
	    !std::isfinite(p.gamma) || p.gamma < .1 || p.gamma > 10 ||
	    !std::isfinite(p.contrast) || p.contrast < 0 || p.contrast > 2 ||
	    !std::isfinite(p.blackLevel) || !std::isfinite(p.whiteLevel) ||
	    p.blackLevel < 0 || p.whiteLevel <= p.blackLevel || p.whiteLevel > 1023 ||
	    !std::isfinite(p.analogueGain) || p.analogueGain < 1 || p.analogueGain > 1024)
		return -EINVAL;
	for (float v : p.whiteBalance)
		if (!std::isfinite(v) || v <= 0 || v > 8)
			return -EINVAL;
	for (float v : p.colorMatrix)
		if (!std::isfinite(v) || std::abs(v) > 16)
			return -EINVAL;
	return 0;
}

} /* namespace */

int StillImageProcessor::processRaw10(const uint8_t *data, size_t length,
				     const StillImageParameters &p, StillImage *result)
{
	if (!result)
		return -EINVAL;
	*result = {};
	if (int ret = validateRaw(data, length, p))
		return ret;

	try {
		int w = p.width, h = p.height;
		const size_t pixels = size_t(w) * h;
		std::vector<uint16_t> raw(pixels), clean(pixels), green(pixels);
		for (int y = 0; y < h; y++) {
			const uint8_t *row = data + size_t(y) * p.stride;
			for (int x = 0; x < w; x++)
				raw[size_t(y) * w + x] = (unsigned(row[x / 4 * 5 + x % 4]) << 2) |
						       ((row[x / 4 * 5 + 4] >> (2 * (x % 4))) & 3);
		}

		float sigma = estimateNoise(raw, w, h, p.order, p.analogueGain);
		int threshold = std::max(2, int(std::ceil(sigma * 3)));
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				int center = raw[size_t(y) * w + x];
				int sum = center * threshold * 3, weights = threshold * 3;
				if (p.denoise) {
					for (const auto &offset : { std::array<int, 2>{ -2, 0 }, { 2, 0 }, { 0, -2 }, { 0, 2 } }) {
						int value = sample(raw, w, h, x + offset[0], y + offset[1]);
						int weight = std::max(0, threshold - std::abs(center - value));
						sum += value * weight;
						weights += weight;
					}
				}
				float amount = color(p.order, x, y) == 1 ? .4f : .8f;
				clean[size_t(y) * w + x] = std::clamp(int(std::lround(center + amount * (float(sum) / weights - center))), 0, 1023);
			}
		}
		raw.clear();
		raw.shrink_to_fit();

		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				int value = clean[size_t(y) * w + x];
				if (color(p.order, x, y) != 1) {
					int left = sample(clean, w, h, x - 1, y), right = sample(clean, w, h, x + 1, y);
					int up = sample(clean, w, h, x, y - 1), down = sample(clean, w, h, x, y + 1);
					int dh = std::abs(left - right) + 8, dv = std::abs(up - down) + 8;
					value = ((left + right) * dv + (up + down) * dh) / (2 * (dh + dv));
				}
				green[size_t(y) * w + x] = value;
			}
		}

		std::array<std::array<std::array<float, 3>, 1024>, 3> columns{};
		for (unsigned c = 0; c < 3; c++)
			for (unsigned value = 0; value < 1024; value++)
				for (unsigned out = 0; out < 3; out++)
					columns[c][value][out] = std::max(float(value) - p.blackLevel, 0.f) /
						(p.whiteLevel - p.blackLevel) * p.whiteBalance[c] * p.colorMatrix[out * 3 + c];
		std::array<uint8_t, 4096> gamma;
		std::array<float, 4097> highlights;
		const float contrast = std::tan(std::min(p.contrast * float(M_PI_4), float(M_PI_2) - .00001f));
		for (unsigned i = 0; i < gamma.size(); i++) {
			float value = float(i) / (gamma.size() - 1);
			value = value < .5 ? .5f * std::pow(2 * value, contrast) : 1 - .5f * std::pow(2 * (1 - value), contrast);
			gamma[i] = std::clamp(int(std::lround(std::pow(value, 1.f / p.gamma) * 255)), 0, 255);
		}
		for (unsigned i = 0; i < highlights.size(); i++) {
			float m = i / 512.f;
			highlights[i] = m > .9f ? (.9f + .1f * (1 - std::exp(-(m - .9f) / .1f))) / m : 1;
		}
		std::vector<uint8_t> rgb(pixels * 3);
		auto difference = [&](int x, int y) {
			return sample(clean, w, h, x, y) - sample(green, w, h, x, y);
		};
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				int c = color(p.order, x, y), g = green[size_t(y) * w + x];
				int r = g, b = g;
				if (c == 1) {
					int horizontal = (difference(x - 1, y) + difference(x + 1, y)) / 2;
					int vertical = (difference(x, y - 1) + difference(x, y + 1)) / 2;
					r += color(p.order, x ^ 1, y) == 0 ? horizontal : vertical;
					b += color(p.order, x ^ 1, y) == 2 ? horizontal : vertical;
				} else {
					int diagonal = (difference(x - 1, y - 1) + difference(x + 1, y - 1) +
							difference(x - 1, y + 1) + difference(x + 1, y + 1)) / 4;
					r = c == 0 ? clean[size_t(y) * w + x] : g + diagonal;
					b = c == 2 ? clean[size_t(y) * w + x] : g + diagonal;
				}
				const auto &red = columns[0][std::clamp(r, 0, 1023)];
				const auto &grn = columns[1][g];
				const auto &blue = columns[2][std::clamp(b, 0, 1023)];
				std::array<float, 3> pixel;
				for (unsigned i = 0; i < 3; i++)
					pixel[i] = std::max(red[i] + grn[i] + blue[i], 0.f);
				float m = *std::max_element(pixel.begin(), pixel.end());
				float scale = !p.highlightRolloff ? 1.f : m > 8 ? 1.f / m : highlights[std::min(unsigned(m * 512), 4096U)];
				for (unsigned i = 0; i < 3; i++)
					rgb[(size_t(y) * w + x) * 3 + i] = gamma[std::clamp(int(pixel[i] * scale * 4095), 0, 4095)];
			}
		}
		if (p.sharpen) {
			auto original = rgb;
			int gate = std::max(2, int(sigma * 255 / (p.whiteLevel - p.blackLevel) * 2));
			for (int y = 1; y + 1 < h; y++) {
				for (int x = 1; x + 1 < w; x++) {
					size_t index = (size_t(y) * w + x) * 3;
					int center = luminance(&original[index]), sum = 0, low = 255, high = 0;
					for (int dy = -1; dy <= 1; dy++)
						for (int dx = -1; dx <= 1; dx++) {
							int value = luminance(&original[(size_t(y + dy) * w + x + dx) * 3]);
							sum += value * (dx == 0 ? 2 : 1) * (dy == 0 ? 2 : 1);
							low = std::min(low, value); high = std::max(high, value);
						}
					int delta = high - low > gate ? std::clamp(int((center - sum / 16.f) * .35f), -6, 6) : 0;
					delta = std::clamp(center + delta, low, high) - center;
					for (unsigned c = 0; c < 3; c++)
						rgb[index + c] = std::clamp(int(original[index + c]) + delta, 0, 255);
				}
			}
		}
		result->width = w; result->height = h; result->rgb = std::move(rgb);
		return 0;
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}

int StillImageProcessor::processRaw10Burst(const std::vector<StillRawFrame> &frames,
					  StillImage *result)
{
	if (!result || frames.empty() || frames.size() > 3)
		return -EINVAL;
	*result = {};
	const auto &p = frames.front().parameters;
	for (const auto &frame : frames) {
		const auto &q = frame.parameters;
		if (validateRaw(frame.data, frame.length, q) || q.width != p.width || q.height != p.height ||
		    q.order != p.order || q.blackLevel != p.blackLevel || q.whiteLevel != p.whiteLevel ||
		    !std::isfinite(q.exposureProduct) || q.exposureProduct < 0)
			return -EINVAL;
	}
	if (frames.size() == 1)
		return processRaw10(frames[0].data, frames[0].length, p, result);
	try {
		const int w = p.width, h = p.height;
		std::vector<std::vector<uint16_t>> raw(frames.size(), std::vector<uint16_t>(size_t(w) * h));
		for (size_t f = 0; f < frames.size(); f++)
			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++) {
					const uint8_t *row = frames[f].data + size_t(y) * frames[f].parameters.stride;
					raw[f][size_t(y) * w + x] = (unsigned(row[x / 4 * 5 + x % 4]) << 2) |
						((row[x / 4 * 5 + 4] >> (2 * (x % 4))) & 3);
				}
		struct Alignment { int x = 0, y = 0; bool use = false; };
		std::vector<Alignment> alignment(frames.size());
		for (size_t f = 1; f < frames.size(); f++) {
			double a = p.exposureProduct, b = frames[f].parameters.exposureProduct;
			if (a > 0 && b > 0 && (a > b * 1.05 || a < b / 1.05))
				continue;
			uint64_t best = UINT64_MAX;
			for (int dy = -8; dy <= 8; dy += 2)
				for (int dx = -8; dx <= 8; dx += 2) {
					uint64_t error = 0;
					for (int y = 12; y + 12 < h; y += 32)
						for (int x = 12; x + 13 < w; x += 32) {
							int gx = x + (color(p.order, x, y) != 1);
							error += std::abs(int(raw[0][size_t(y) * w + gx]) - raw[f][size_t(y + dy) * w + gx + dx]);
						}
					if (error < best) {
						best = error;
						alignment[f] = { dx, dy, true };
					}
				}
			/* A best match on the search boundary is not trustworthy. */
			if (std::abs(alignment[f].x) == 8 || std::abs(alignment[f].y) == 8)
				alignment[f].use = false;
		}
		int threshold = std::max(3, int(estimateNoise(raw[0], w, h, p.order, p.analogueGain) * 3));
		std::vector<uint8_t> packed(size_t(p.stride) * h);
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++) {
				int center = raw[0][size_t(y) * w + x];
				int sum = center * threshold, weights = threshold;
				int gradient = std::abs(sample(raw[0], w, h, x - 2, y) - sample(raw[0], w, h, x + 2, y)) +
					std::abs(sample(raw[0], w, h, x, y - 2) - sample(raw[0], w, h, x, y + 2));
				if (gradient < threshold * 2) {
					for (size_t f = 1; f < frames.size(); f++) {
						const auto &shift = alignment[f];
						if (!shift.use || x + shift.x < 0 || x + shift.x >= w || y + shift.y < 0 || y + shift.y >= h)
							continue;
						int value = raw[f][size_t(y + shift.y) * w + x + shift.x];
						int weight = std::max(0, threshold - std::abs(value - center));
						sum += value * weight; weights += weight;
					}
				}
				unsigned value = (sum + weights / 2) / weights;
				size_t offset = size_t(y) * p.stride + x / 4 * 5;
				packed[offset + x % 4] = value >> 2;
				packed[offset + 4] |= (value & 3) << (2 * (x % 4));
			}
		return processRaw10(packed.data(), packed.size(), p, result);
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
}

} /* namespace libcamera */
