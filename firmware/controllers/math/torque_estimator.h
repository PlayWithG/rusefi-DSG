#pragma once

// Fills config->torqueTable[load][rpm] using the VE table and reference values.
// Called once on init or when reference parameters change.
void estimateTorqueTable();

// Returns the estimated engine torque (Nm) at current operating conditions.
//
// Uses:
//   VE table   — volumetric efficiency at current RPM/MAP
//   MAP sensor — manifold absolute pressure for air density
//   IAT sensor — intake air temperature density correction (if enabled in config)
//
// This value is suitable for real-time CAN transmission to a DSG TCU.
// Applies a low-pass filter (alpha = torqueEstimationFilterAlpha in config).
float getInstantTorque();

// Raw (unfiltered) torque estimate at current RPM/MAP/IAT.
// Use for diagnostics or when you need immediate response.
float getRawInstantTorque();
