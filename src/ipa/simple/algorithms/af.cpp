/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Contrast autofocus for a software ISP and a V4L2 lens subdevice. */
#include "af.h"

#include <algorithm>
#include <limits>
#include <linux/v4l2-controls.h>
#include <libcamera/base/log.h>
#include <libcamera/control_ids.h>

namespace libcamera::ipa::soft::algorithms {
LOG_DEFINE_CATEGORY(IPASoftAf)

int Af::init(IPAContext &context, const ValueNode &tuningData)
{
	auto control = context.lensControls.find(V4L2_CID_FOCUS_ABSOLUTE);
	if (control == context.lensControls.end())
		return 0;
	const ControlInfo &range = control->second;
	minimum_ = std::max(range.min().get<int32_t>(), tuningData["min"].get<int32_t>(0));
	maximum_ = std::min(range.max().get<int32_t>(), tuningData["max"].get<int32_t>(1023));
	initial_ = tuningData["default"].get<int32_t>(range.def().get<int32_t>());
	step_ = tuningData["coarseStep"].get<int32_t>(64);
	settle_ = tuningData["settleFrames"].get<uint32_t>(4);
	if (minimum_ >= maximum_ || step_ <= 0 || !settle_)
		return -EINVAL;
	enabled_ = true;
	context.ctrlMap[&controls::AfMode] = ControlInfo(
		Span<const ControlValue>(controls::AfModeValues), controls::AfModeContinuous);
	context.ctrlMap[&controls::AfTrigger] = ControlInfo(Span<const ControlValue>(controls::AfTriggerValues));
	context.ctrlMap[&controls::AfState] = ControlInfo(Span<const ControlValue>(controls::AfStateValues));
	context.ctrlMap[&controls::FocusFoM] = ControlInfo(int32_t(0), std::numeric_limits<int32_t>::max(), int32_t(0));
	/* No guessed dioptre-to-DAC mapping: LensPosition is not advertised. */
	return 0;
}

int Af::configure([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	if (enabled_)
		af_.configure(minimum_, maximum_, initial_, step_, settle_);
	return 0;
}

void Af::queueRequest([[maybe_unused]] IPAContext &context, const uint32_t frame,
		      [[maybe_unused]] IPAFrameContext &frameContext, const ControlList &controls)
{
	if (!enabled_)
		return;
	if (auto mode = controls.get(controls::AfMode))
		af_.setMode(*mode, frame);
	if (auto trigger = controls.get(controls::AfTrigger))
		af_.trigger(*trigger == controls::AfTriggerCancel, frame);
}

void Af::process(IPAContext &context, const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const SwIspStats *stats, ControlList &metadata)
{
	if (!enabled_)
		return;
	if (context.lensFailure) {
		metadata.set(controls::AfMode, af_.mode());
		metadata.set(controls::AfState, controls::AfStateFailed);
		return;
	}
	double score = stats->focusEnergy ?
		static_cast<double>(stats->focusGradient) / stats->focusEnergy : 0;
	if (stats->valid) {
		af_.process(frame, score);
		LOG(IPASoftAf, Debug) << "frame " << frame << " position " << af_.position()
			<< " contrast " << score << " state " << af_.state();
	}
	if (auto position = af_.takePosition())
		context.lensPosition = *position;
	metadata.set(controls::AfMode, af_.mode());
	metadata.set(controls::AfState, af_.state());
	if (stats->valid)
		metadata.set(controls::FocusFoM, static_cast<int32_t>(std::min(
			score * 1000000, static_cast<double>(std::numeric_limits<int32_t>::max()))));
}
REGISTER_IPA_ALGORITHM(Af, "Af")
} /* namespace libcamera::ipa::soft::algorithms */
