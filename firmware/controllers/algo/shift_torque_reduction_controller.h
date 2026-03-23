//
// Created by kifir on 9/27/24. Refactored with phase state machine.
//

#pragma once

#include "shift_torque_reduction_state_generated.h"

// Phase state machine for upshift torque reduction.
// IDLE     → waiting for shift request
// REDUCING → ramping ignition retard toward targetRetard
// HOLD     → holding retard while DSG completes clutch overlap
// RECOVERY → ramping retard back to zero
enum class TorqueReductionPhase : uint8_t {
	IDLE     = 0,
	REDUCING = 1,
	HOLD     = 2,
	RECOVERY = 3,
};

class ShiftTorqueReductionController : public shift_torque_reduction_state_s {
public:
	void update();

	// Returns the ignition retard to SUBTRACT from base advance (degrees, >= 0).
	// Applied as an offset so base advance is never lost completely.
	float getCurrentRetardOffset() const;

	// Returns the current spark skip ratio for the hard spark limiter (0..1).
	float getSparkSkipRatio();

	// Legacy name kept for API compatibility — returns same as getCurrentRetardOffset().
	float getTorqueReductionIgnitionRetard();

private:
	void updateTriggerPinState();
	void updateTriggerPinState(switch_input_pin_e pin, pin_input_mode_e mode, const bool invertPhysicalPin, bool invalidPinState);

	void updateRpmConditionSatisfied();
	void updateAppConditionSatisfied();

	// Reads the target retard and time from the 2-D tables at (gear, x-axis).
	float lookupTargetRetard();
	float lookupHoldTimeMs();

	TorqueReductionPhase m_phase = TorqueReductionPhase::IDLE;
	float m_currentRetard = 0.0f;   // degrees currently subtracted from advance
	float m_targetRetard  = 0.0f;   // goal for this shift event
	float m_currentSparkSkip = 0.0f;

	Timer m_phaseTimer;     // used for HOLD timing
	Timer m_pinTriggeredTimer;  // kept for backward-compat time table
};
