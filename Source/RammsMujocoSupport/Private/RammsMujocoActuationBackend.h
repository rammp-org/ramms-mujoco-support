// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsActuationBackend.h"
#include "UObject/WeakObjectPtr.h"

class AMjArticulation;
class UMjNodeComponent;

/**
 * IRammsActuationBackend backed by a URLab/MuJoCo articulation.
 *
 * A motor's registry Id is the MuJoCo actuator name (FRammsMotorSpec::Id is the
 * canonical, backend-neutral name and MuJoCo's by default). Commands stage onto
 * the actuator's internal (UI/Blueprint) control slot; reads come from the
 * compiled model at the actuator's id. The actuator type in the registry only
 * documents intent — MuJoCo interprets a staged value by the actuator's own
 * kind (a <motor> takes torque, a <position> a target, a <velocity> a speed),
 * so this backend writes the same scalar for every type.
 *
 * It clamps to the compiled actuator's ctrlrange (the base component may also
 * clamp to the registry ControlRange first), selects the UI control source so
 * game-thread writes reach d->ctrl, and resolves actuator nodes once, lazily,
 * caching the UMjNodeComponent* per Id.
 */
class FRammsMujocoActuationBackend final : public IRammsActuationBackend
{
public:
	virtual bool  Initialize(URammsRobotBaseComponent& Base) override;
	virtual void  SetCommand(FName MotorId, float Value) override;
	virtual float GetValue(FName MotorId) const override;
	virtual float GetVelocity(FName MotorId) const override;
	virtual bool  GetMotorTransform(FName MotorId, FTransform& OutWorld) const override;

private:
	/** Resolve (and cache) the actuator node for a motor Id, or nullptr. */
	UMjNodeComponent* ResolveActuator(FName MotorId) const;

	TWeakObjectPtr<AMjArticulation> Articulation;

	/** Id -> actuator node, filled on first use. mutable: reads resolve lazily. */
	mutable TMap<FName, TWeakObjectPtr<UMjNodeComponent>> ActuatorByMotor;
};
