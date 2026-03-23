#include "pch.h"

// Shared helper: computes the IAT density correction factor.
// Returns the multiplier to apply to base torque.
// Higher IAT → less dense air → less torque.
// Factor = referenceIAT_K / actualIAT_K  (linear density approximation).
static float getIatTorqueCorrectionFactor() {
	if (!engineConfiguration->torqueEstimationIatCorrectionEnabled) {
		return 1.0f;
	}
	const auto iat = Sensor::get(SensorType::Iat);
	if (!iat.Valid) {
		return 1.0f;
	}
	float referenceIatK = engineConfiguration->torqueEstimationReferenceIat + 273.15f;
	float actualIatK    = iat.Value + 273.15f;
	if (actualIatK <= 0) {
		return 1.0f;
	}
	return referenceIatK / actualIatK;
}

void estimateTorqueTable() {
	for (int rpmIndex = 0; rpmIndex < TORQUE_CURVE_RPM_SIZE; rpmIndex++) {
		for (int loadIndex = 0; loadIndex < TORQUE_CURVE_SIZE; loadIndex++) {
			float rpmValue = config->torqueRpmBins[rpmIndex];
			// Load axis is MAP (speed-density assumption)
			float mapValue = config->torqueLoadBins[loadIndex];

			percent_t veValue = interpolate3d(
				config->veTable,
				config->veLoadBins, mapValue,
				config->veRpmBins, rpmValue
			);

			float estimated = engineConfiguration->referenceTorqueForGenerator
				* (veValue / engineConfiguration->referenceVeForGenerator)
				* (mapValue / engineConfiguration->referenceMapForGenerator);

			// Apply IAT correction to the pre-filled table as well
			estimated *= getIatTorqueCorrectionFactor();

			config->torqueTable[loadIndex][rpmIndex] = estimated;

			if (loadIndex == 0 && rpmIndex == 0) {
				efiPrintf("estimateTorqueTable [%f] %f", veValue, estimated);
			}
		}
	}
}

float getInstantTorque() {
	if (engineConfiguration->referenceMapForGenerator <= 0 ||
	    engineConfiguration->referenceVeForGenerator  <= 0 ||
	    engineConfiguration->referenceTorqueForGenerator <= 0) {
		return 0;
	}

	float rpm = Sensor::getOrZero(SensorType::Rpm);
	float map = Sensor::getOrZero(SensorType::Map);

	percent_t ve = interpolate3d(
		config->veTable,
		config->veLoadBins, map,
		config->veRpmBins, rpm
	);

	// Base torque: scales with VE and manifold pressure
	float torque = engineConfiguration->referenceTorqueForGenerator
		* (ve  / engineConfiguration->referenceVeForGenerator)
		* (map / engineConfiguration->referenceMapForGenerator);

	// IAT density correction
	torque *= getIatTorqueCorrectionFactor();

	return maxF(torque, 0.0f);
}
