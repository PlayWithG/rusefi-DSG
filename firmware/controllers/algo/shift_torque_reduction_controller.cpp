//
// Created by kifir on 9/27/24. Refactored with phase SM and offset-based retard.
//

#include "pch.h"

#if EFI_LAUNCH_CONTROL
#include "shift_torque_reduction_controller.h"
#include "boost_control.h"
#include "launch_control.h"
#include "advance_map.h"
#include "engine_state.h"
#include "advance_map.h"
#include "tinymt32.h"
#include "gppwm_channel_reader.h"

// ---------------------------------------------------------------------------
// update() — called every fast callback (~100 Hz)
// ---------------------------------------------------------------------------

void ShiftTorqueReductionController::update() {
	if (!engineConfiguration->torqueReductionEnabled) {
		m_phase = TorqueReductionPhase::IDLE;
		m_currentRetard   = 0.0f;
		m_currentSparkSkip = 0.0f;
		isFlatShiftConditionSatisfied = false;
		torqueReductionPhase = (uint8_t)TorqueReductionPhase::IDLE;
		return;
	}

	updateTriggerPinState();
	updateRpmConditionSatisfied();
	updateAppConditionSatisfied();

	// Temperature gate — prevent activation below threshold
	isBelowTemperatureThreshold = (engineConfiguration->torqueReductionActivationTemperature != 0)
		&& (Sensor::getOrZero(SensorType::Clt) < engineConfiguration->torqueReductionActivationTemperature);

	bool activated = torqueReductionTriggerPinState
		&& isRpmConditionSatisfied
		&& isAppConditionSatisfied
		&& !isBelowTemperatureThreshold;

	float rampIn  = maxF(engineConfiguration->torqueReductionRampInRate,  0.1f);
	float rampOut = maxF(engineConfiguration->torqueReductionRampOutRate, 0.1f);

	switch (m_phase) {

	case TorqueReductionPhase::IDLE:
		m_currentRetard    = 0.0f;
		m_currentSparkSkip = 0.0f;
		if (activated) {
			m_targetRetard = maxF(-lookupTargetRetard(), 0.0f);  // table stores negative offsets
			m_currentSparkSkip = 0.0f;
			m_phase = TorqueReductionPhase::REDUCING;
			m_phaseTimer.reset();
		}
		break;

	case TorqueReductionPhase::REDUCING:
		if (!activated) {
			// Trigger released before reaching target — go straight to recovery
			m_phase = TorqueReductionPhase::RECOVERY;
			break;
		}
		m_currentRetard = minF(m_currentRetard + rampIn, m_targetRetard);
		// Spark skip ramps at same pace but limited to configured table value
		{
			float targetSkip = lookupSparkSkipTarget();
			m_currentSparkSkip = minF(m_currentSparkSkip + rampIn * 0.05f, targetSkip);
		}
		if (m_currentRetard >= m_targetRetard) {
			m_phase = TorqueReductionPhase::HOLD;
			m_phaseTimer.reset();
		}
		break;

	case TorqueReductionPhase::HOLD:
		m_currentRetard    = m_targetRetard;
		// Transition to RECOVERY when trigger releases OR hold time expires
		if (!activated || m_phaseTimer.hasElapsedMs(lookupHoldTimeMs())) {
			m_phase = TorqueReductionPhase::RECOVERY;
		}
		break;

	case TorqueReductionPhase::RECOVERY:
		m_currentRetard    = maxF(m_currentRetard - rampOut, 0.0f);
		m_currentSparkSkip = maxF(m_currentSparkSkip - rampOut * 0.05f, 0.0f);
		if (m_currentRetard <= 0.0f) {
			m_phase = TorqueReductionPhase::IDLE;
			m_currentSparkSkip = 0.0f;
		}
		break;
	}

	isFlatShiftConditionSatisfied = (m_phase != TorqueReductionPhase::IDLE);
	torqueReductionPhase = (uint8_t)m_phase;
	torqueReductionCurrentRetard = m_currentRetard;
}

// ---------------------------------------------------------------------------
// Output accessors
// ---------------------------------------------------------------------------

float ShiftTorqueReductionController::getCurrentRetardOffset() const {
	return m_currentRetard;  // positive value: degrees to SUBTRACT from advance
}

float ShiftTorqueReductionController::getTorqueReductionIgnitionRetard() {
	return getCurrentRetardOffset();
}

float ShiftTorqueReductionController::getSparkSkipRatio() {
	if (!engineConfiguration->torqueReductionEnabled || m_phase == TorqueReductionPhase::IDLE) {
		return 0.0f;
	}
	// Re-read x-axis value for live data
	auto torqueReductionXAxis = readGppwmChannel(config->torqueReductionCutXaxis);
	trqRedCutXaxisValue = torqueReductionXAxis.Value;
	return m_currentSparkSkip;
}

// ---------------------------------------------------------------------------
// Lookup helpers
// ---------------------------------------------------------------------------

float ShiftTorqueReductionController::lookupTargetRetard() {
	auto xAxis = readGppwmChannel(config->torqueReductionIgnitionRetardXaxis);
	trqRedIgnRetXaxisValue = xAxis.Value;
	int8_t currentGear = Sensor::getOrZero(SensorType::DetectedGear);
	// Table stores negative degrees (retard) — we return the magnitude
	float val = interpolate3d(
		config->torqueReductionIgnitionRetardTable,
		config->torqueReductionIgnitionRetardGearBins, currentGear,
		config->torqueReductionIgnitionRetardXBins, xAxis.Value
	);
	return fabsf(val);
}

float ShiftTorqueReductionController::lookupSparkSkipTarget() {
	auto xAxis = readGppwmChannel(config->torqueReductionCutXaxis);
	int8_t currentGear = Sensor::getOrZero(SensorType::DetectedGear);
	return interpolate3d(
		config->torqueReductionIgnitionCutTable,
		config->torqueReductionCutGearBins, currentGear,
		config->torqueReductionCutXBins, xAxis.Value
	) / 100.0f;
}

float ShiftTorqueReductionController::lookupHoldTimeMs() {
	auto xAxis = readGppwmChannel(config->torqueReductionTimeXaxis);
	trqRedTimeXaxisValue = xAxis.Value;
	int8_t currentGear = Sensor::getOrZero(SensorType::DetectedGear);
	return interpolate3d(
		config->torqueReductionTimeTable,
		config->torqueReductionTimeGearBins, currentGear,
		config->torqueReductionTimeXBins, xAxis.Value
	);
}

// ---------------------------------------------------------------------------
// Condition checks
// ---------------------------------------------------------------------------

void ShiftTorqueReductionController::updateTriggerPinState() {
	switch (engineConfiguration->torqueReductionActivationMode) {
		case TORQUE_REDUCTION_BUTTON: {
			updateTriggerPinState(
				engineConfiguration->torqueReductionTriggerPin,
				engineConfiguration->torqueReductionTriggerPinMode,
				/*invertPhysicalPin*/false,
				engine->engineState.lua.torqueReductionState
			);
			break;
		}
		case LAUNCH_BUTTON: {
			updateTriggerPinState(
				engineConfiguration->launchActivatePin,
				engineConfiguration->launchActivatePinMode,
				/*invertPhysicalPin*/false,
				false
			);
			break;
		}
		case TORQUE_REDUCTION_CLUTCH_DOWN_SWITCH: {
			updateTriggerPinState(
				engineConfiguration->clutchDownPin,
				engineConfiguration->clutchDownPinMode,
				/*invertPhysicalPin*/false,
				engine->engineState.lua.clutchDownState
			);
			break;
		}
		case TORQUE_REDUCTION_CLUTCH_UP_SWITCH: {
			updateTriggerPinState(
				engineConfiguration->clutchUpPin,
				engineConfiguration->clutchUpPinMode,
				/*invertPhysicalPin*/true,
				engine->engineState.lua.clutchUpState
			);
			break;
		}
		default: {
			break;
		}
	}
}

void ShiftTorqueReductionController::updateTriggerPinState(
	const switch_input_pin_e pin,
	const pin_input_mode_e mode,
	const bool invertPhysicalPin,
	const bool invalidPinState
) {
	if (!torqueReductionTriggerPinState) {
		if (isBelowTemperatureThreshold) {
			return;
		}
	}

#if !EFI_SIMULATOR
	isTorqueReductionTriggerPinValid = isBrainPinValid(pin);
	const bool prev = torqueReductionTriggerPinState;
	if (isTorqueReductionTriggerPinValid) {
		torqueReductionTriggerPinState = efiReadPin(pin, mode) ^ invertPhysicalPin;
	} else {
		torqueReductionTriggerPinState = invalidPinState;
	}
	if (!prev && torqueReductionTriggerPinState) {
		m_pinTriggeredTimer.reset();
	}
#else
	UNUSED(pin); UNUSED(mode); UNUSED(invertPhysicalPin); UNUSED(invalidPinState);
#endif
}

void ShiftTorqueReductionController::updateRpmConditionSatisfied() {
	isRpmConditionSatisfied = (engineConfiguration->torqueReductionArmingRpm
		<= Sensor::getOrZero(SensorType::Rpm));
}

void ShiftTorqueReductionController::updateAppConditionSatisfied() {
	const SensorResult app = Sensor::get(SensorType::DriverThrottleIntent);
	isAppConditionSatisfied = app.Valid
		&& (engineConfiguration->torqueReductionArmingApp <= app.Value);
}

#endif // EFI_LAUNCH_CONTROL
