/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Histogram exposure control with highlight protection and direct EV allocation. */
#include "agc.h"

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "exposure_meter.h"

namespace libcamera {
LOG_DEFINE_CATEGORY(IPASoftExposure)
namespace ipa::soft::algorithms {
Agc::Agc() = default;
void Agc::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, const SwIspStats *stats,
		  ControlList &metadata)
{
	auto &state = context.activeState.agc;
	auto &config = context.configuration.agc;
	utils::Duration exposureTime = config.lineDuration * frameContext.sensor.exposure;
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);
	if (!state.valid) {
		state.exposure = frameContext.sensor.exposure;
		state.again = frameContext.sensor.gain;
		state.valid = true;
	}
	if (stats->valid) {
		const auto &g = context.activeState.awb.gains;
		auto m = meterExposure(stats->yHistogram, context.activeState.blc.level,
				       .299 * g.r() + .587 * g.g() + .114 * g.b());
		double total = frameContext.sensor.exposure * frameContext.sensor.gain * m.factor;
		auto next = allocateExposure(total, config.exposureMin, config.exposureMax,
					     config.againMin, config.againMax);
		state.exposure = next.exposure;
		state.again = next.gain;
		LOG(IPASoftExposure, Debug) << "mean " << m.mean << " p95 " << m.highlight
					    << " factor " << m.factor << " exp " << state.exposure << " gain " << state.again;
	}
	frameContext.sensor.exposure = state.exposure;
	frameContext.sensor.gain = state.again;
}
REGISTER_IPA_ALGORITHM(Agc, "Agc")
} // namespace ipa::soft::algorithms
} // namespace libcamera
