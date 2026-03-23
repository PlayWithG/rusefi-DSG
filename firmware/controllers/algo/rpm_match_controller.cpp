//
// RPM Match Controller — raises engine RPM to synchronise with target gear
// before DSG downshift clutch engagement.
//
// Strategy:
//   error = targetRpm - currentRpm
//   If error > 0 (RPM below target), add proportional ignition advance boost.
//   Advance boost is capped by rpmMatchMaxAdvanceBoost (config).
//   Success when |error| <= rpmMatchRpmTolerance for two consecutive updates.
//   Timeout when rpmMatchTimeout ms has elapsed without matching.
//

#include "pch.h"

#if EFI_LAUNCH_CONTROL
#include "rpm_match_controller.h"

void RpmMatchController::update() {
	if (!engineConfiguration->rpmMatchEnabled) {
		cancel();
		return;
	}

	if (!isRpmMatchActive) {
		rpmMatchCurrentAdvanceBoost = 0;
		return;
	}

	// Timeout check
	if (m_matchTimer.hasElapsedMs(engineConfiguration->rpmMatchTimeout)) {
		isRpmMatchActive = false;
		isRpmMatchTimeout = true;
		isRpmMatchSuccess = false;
		rpmMatchCurrentAdvanceBoost = 0;
		return;
	}

	float currentRpm = Sensor::getOrZero(SensorType::Rpm);
	rpmMatchError = rpmMatchTarget - currentRpm;

	float tolerance = engineConfiguration->rpmMatchRpmTolerance;

	if (fabsf(rpmMatchError) <= tolerance) {
		isRpmMatchSuccess = true;
		// Stay active until Lua explicitly cancels so it can read the success state.
		rpmMatchCurrentAdvanceBoost = 0;
		return;
	}

	isRpmMatchSuccess = false;

	if (rpmMatchError > 0) {
		// RPM below target: apply proportional advance boost.
		// Full boost kicks in at 500 RPM below target, scales linearly below that.
		float maxBoost = engineConfiguration->rpmMatchMaxAdvanceBoost;
		float normalizedError = minF(rpmMatchError / 500.0f, 1.0f);
		rpmMatchCurrentAdvanceBoost = maxBoost * normalizedError;
	} else {
		// RPM above target (overshoot): no boost, let RPM settle.
		rpmMatchCurrentAdvanceBoost = 0;
	}
}

float RpmMatchController::getIgnitionAdvanceBoost() const {
	if (!isRpmMatchActive) {
		return 0;
	}
	return rpmMatchCurrentAdvanceBoost;
}

void RpmMatchController::setTargetRpm(float targetRpm) {
	rpmMatchTarget = targetRpm;
	rpmMatchError = 0;
	rpmMatchCurrentAdvanceBoost = 0;
	isRpmMatchActive = true;
	isRpmMatchSuccess = false;
	isRpmMatchTimeout = false;
	m_matchTimer.reset();
}

void RpmMatchController::cancel() {
	isRpmMatchActive = false;
	isRpmMatchSuccess = false;
	rpmMatchCurrentAdvanceBoost = 0;
	rpmMatchError = 0;
}

#endif // EFI_LAUNCH_CONTROL
