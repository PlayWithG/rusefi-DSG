//
// RPM Match Controller — tracks downshift RPM sync state for TunerStudio live data.
//
// Primary blip mechanism: ETB add (setEtbAdd) managed from Lua (vw_b6.lua).
// This firmware module provides:
//   - State visibility in TunerStudio
//   - A small secondary advance boost for non-ETB setups (optional)
//   - Timeout safety net
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

	// Safety timeout — in case Lua forgets to cancel
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
		rpmMatchCurrentAdvanceBoost = 0;
		return;
	}

	isRpmMatchSuccess = false;

	// Secondary advance boost: very small, only when RPM is significantly below target.
	// Primary mechanism is ETB add in Lua. Keep this at <= 5° to avoid detonation risk.
	if (rpmMatchError > 0) {
		float maxBoost = minF(engineConfiguration->rpmMatchMaxAdvanceBoost, 5.0f);
		float normalized = minF(rpmMatchError / 500.0f, 1.0f);
		rpmMatchCurrentAdvanceBoost = maxBoost * normalized;
	} else {
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
