/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Contrast scan controller. It never accesses a device and uses raw VCM units. */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace libcamera::ipa::soft::algorithms {

class ContrastAf
{
public:
	enum Mode { Manual, Auto, Continuous };
	enum State { Idle, Scanning, Focused, Failed };

	void configure(int minimum, int maximum, int initial, int step, unsigned int settle)
	{
		minimum_ = minimum;
		maximum_ = maximum;
		coarseStep_ = std::max(step, 1);
		settleFrames_ = std::max(settle, 1U);
		mode_ = Continuous;
		position_ = std::clamp(initial, minimum_, maximum_);
		pending_ = position_;
		lastMove_ = 0;
		nextScan_ = 24;
		phase_ = Warmup;
		state_ = Scanning;
		badCount_ = 0;
	}

	void setMode(int mode, uint32_t frame)
	{
		if (mode < Manual || mode > Continuous || mode == mode_)
			return;
		mode_ = static_cast<Mode>(mode);
		phase_ = Rest;
		state_ = Idle;
		if (mode_ == Continuous)
			start(frame);
	}

	void trigger(bool cancel, uint32_t frame)
	{
		if (mode_ != Auto)
			return;
		if (cancel) {
			phase_ = Rest;
			state_ = Idle;
		} else {
			start(frame);
		}
	}

	void process(uint32_t frame, double score)
	{
		if (mode_ == Manual || !std::isfinite(score) || score < 0 ||
		    frame < lastMove_ + settleFrames_)
			return;
		if (phase_ == Warmup) {
			if (frame >= nextScan_)
				start(frame);
			return;
		}
		if (phase_ == Rest) {
			if (mode_ != Continuous || frame < nextScan_)
				return;
			/* Require a sustained loss of contrast; don't hunt continuously. */
			if (state_ == Failed || score < reference_ * 0.65)
				badCount_++;
			else
				badCount_ = 0;
			if (badCount_ >= 6)
				start(frame);
			return;
		}
		if (phase_ == Verify) {
			reference_ = score;
			phase_ = Rest;
			state_ = successful_ ? Focused : Failed;
			nextScan_ = frame + (successful_ ? 60 : 150);
			return;
		}

		worst_ = std::min(worst_, score);
		if (score > best_) {
			best_ = score;
			bestPosition_ = position_;
		}
		if (position_ < end_) {
			move(std::min(position_ + step_, end_), frame);
			return;
		}
		if (phase_ == Coarse) {
			/* Scan on both sides of the coarse peak, including both endpoints. */
			phase_ = Fine;
			end_ = std::min(maximum_, bestPosition_ + coarseStep_);
			step_ = std::max(1, coarseStep_ / 4);
			int begin = std::max(minimum_, bestPosition_ - coarseStep_);
			coarseWorst_ = worst_;
			move(begin, frame);
			return;
		}
		/* Flat/noisy scenes must not report a successful lock. */
		successful_ = best_ > 0.0005 && best_ > coarseWorst_ * 1.15;
		phase_ = Verify;
		move(bestPosition_, frame);
	}

	std::optional<int> takePosition()
	{
		auto result = pending_;
		pending_.reset();
		return result;
	}
	int position() const { return position_; }
	int mode() const { return mode_; }
	int state() const { return state_; }

private:
	enum Phase { Warmup, Coarse, Fine, Verify, Rest };
	void move(int position, uint32_t frame)
	{
		position_ = std::clamp(position, minimum_, maximum_);
		pending_ = position_;
		lastMove_ = frame;
	}
	void start(uint32_t frame)
	{
		phase_ = Coarse;
		state_ = Scanning;
		best_ = -1;
		worst_ = std::numeric_limits<double>::infinity();
		bestPosition_ = position_;
		end_ = maximum_;
		step_ = coarseStep_;
		badCount_ = 0;
		move(minimum_, frame);
	}
	int minimum_ = 0, maximum_ = 1023, coarseStep_ = 64, step_ = 64;
	int position_ = 0, bestPosition_ = 0, end_ = 1023;
	unsigned int settleFrames_ = 4, badCount_ = 0;
	uint32_t lastMove_ = 0, nextScan_ = 0;
	double best_ = -1, worst_ = 0, coarseWorst_ = 0, reference_ = 0;
	Mode mode_ = Continuous;
	State state_ = Idle;
	Phase phase_ = Rest;
	bool successful_ = false;
	std::optional<int> pending_;
};

} /* namespace libcamera::ipa::soft::algorithms */
