/* SPDX-License-Identifier: LGPL-2.1-or-later */
#pragma once
#include "algorithm.h"
#include "contrast_af.h"

namespace libcamera::ipa::soft::algorithms {
class Af : public Algorithm
{
public:
	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext, const ControlList &controls) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext, const SwIspStats *stats,
		     ControlList &metadata) override;
private:
	bool enabled_ = false;
	int minimum_ = 0, maximum_ = 1023, initial_ = 0, step_ = 64;
	unsigned int settle_ = 4;
	ContrastAf af_;
};
} /* namespace libcamera::ipa::soft::algorithms */
