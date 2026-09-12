// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsDriveBackend.h"
#include "UObject/WeakObjectPtr.h"

class AMjArticulation;

/**
 * Drive backend that steers a URLab/MuJoCo-simulated base by writing torque to
 * its wheel actuators — the MuJoCo counterpart of the Chaos skeletal backend.
 *
 * The MeBot's wheels are torque `<motor>` actuators, so the differential
 * controller's TorqueControl output maps straight through: the per-wheel torque
 * it computes is written to the MuJoCo wheel actuator for that side. In
 * skid-steer configuration the front/rear linkages are lifted and traction
 * comes from the centre wheels, so by default each side names one centre-wheel
 * actuator.
 *
 * Actuator names come from the controller's LeftWheelBoneName / RightWheelBoneName
 * (reused here to name the wheel *actuators* — set them to e.g.
 * "left_center_wheel" / "right_center_wheel" on the MeBot-MuJoCo blueprint).
 *
 * VelocityControl is not yet supported here (the MeBot has no `<velocity>`
 * actuators); the controller's TorqueControl mode is the supported path.
 */
class FRammsMujocoDriveBackend final : public IRammsDriveBackend
{
public:
	virtual bool Initialize(URammsDifferentialDriveController& Controller) override;
	virtual void ReadWheelState(ERammsDriveWheel Wheel, FWheelState& OutState) override;
	virtual void ApplyWheelTorque(ERammsDriveWheel Wheel, float RequestedTorque,
		const FMotorParameters& MotorParams, FWheelState& WheelState) override;
	virtual void ApplyBrake(ERammsDriveWheel Wheel, FWheelState& WheelState) override;

private:
	/** The wheel actuator name for a side (from the controller's wheel-bone fields). */
	FString ActuatorNameFor(ERammsDriveWheel Wheel) const;

	TWeakObjectPtr<URammsDifferentialDriveController> Controller;
	TWeakObjectPtr<AMjArticulation>					  Articulation;
};
