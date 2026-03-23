//
// RPM Match Controller for DSG/automatic transmission downshifts.
//
// When the TCU requests a downshift via CAN, the engine RPM must be raised
// to match the target gear's synchronisation speed before the clutch engages.
// This controller applies a proportional ignition advance boost to raise RPM
// toward the requested target, then reports success back to Lua.
//
// Lua API:
//   setRpmMatchTarget(rpm)  -- activate, set target RPM
//   getRpmMatchState()      -- 0=inactive, 1=matching, 2=matched, 3=timeout
//   getRpmMatchError()      -- current RPM error (target - actual)
//   cancelRpmMatch()        -- deactivate immediately
//

#pragma once

#include "rpm_match_state_generated.h"

class RpmMatchController : public rpm_match_state_s {
public:
	void update();

	// Returns degrees of advance to ADD to base ignition timing.
	// Zero when inactive.
	float getIgnitionAdvanceBoost() const;

	// Called from Lua to start RPM matching for a downshift.
	void setTargetRpm(float targetRpm);

	// Called from Lua or internally on timeout to stop.
	void cancel();

private:
	Timer m_matchTimer;
};
