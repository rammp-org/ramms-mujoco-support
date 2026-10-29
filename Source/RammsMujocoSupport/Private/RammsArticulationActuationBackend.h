// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsActuationBackend.h"
#include "RammsMujocoActuationBackend.h"
#include "UObject/WeakObjectPtr.h"

class AMjArticulation;
class URammsChaosArticulationComponent;

/**
 * IRammsActuationBackend for a URLab articulation that URammsChaosArticulationComponent simulates with
 * Chaos. Motors are addressed by MJCF actuator name, as on MuJoCo, and every reading is in the same
 * units: GetValue / GetVelocity are the actuator's joint coordinate and rate (rad, or cm for a slide),
 * GetMotorTransform the body that joint moves. Commands are the actuator's ctrl, read by its MJCF kind
 * (a <motor> takes torque, a <position> a target). SetVelocityDrive runs the speed loop in the
 * component's physics-rate callback. The clock is the game's (Chaos steps with it).
 */
class FRammsChaosArticulationBackend final : public IRammsActuationBackend
{
public:
	virtual bool  Initialize(URammsRobotBaseComponent& Base) override;
	virtual void  SetCommand(FName MotorId, float Value) override;
	virtual float GetValue(FName MotorId) const override;
	virtual float GetVelocity(FName MotorId) const override;
	virtual bool  ReleaseMotor(FName MotorId) override;
	virtual bool  GetMotorTransform(FName MotorId, FTransform& OutWorld) const override;
	virtual bool  SetVelocityDrive(FName MotorId, float TargetRate, const FRammsVelocityGains& Gains,
		float TorqueMin, float TorqueMax) override;
	virtual void  ClearVelocityDrive(FName MotorId) override;
	/** False for a torque motor: a controller's own loop on its angle, closed once a game frame on a
	 *  reading a physics step old, does not hold a wheel against Chaos' implicit solve (it rings, and
	 *  a lightly loaded wheel spins up). SetMotorBrake holds it instead, with the joint's drive. */
	virtual bool SimulatesMotorJoint(FName MotorId) const override;
	virtual bool SetMotorBrake(FName MotorId, bool bEngaged) override;

	/** Whether the articulation is (or is about to be, once its component's BeginPlay builds it)
	 *  simulated by Chaos: it carries a URammsChaosArticulationComponent that is on and has not failed. */
	bool IsActive() const;

private:
	/** The articulation's Chaos component, looked for again at most once a frame while it has none
	 *  (SwitchToChaos adds one at runtime). */
	URammsChaosArticulationComponent* Component() const;

	/** The motor's joint and whether it is a slide (its readings in cm), or None. */
	FName JointOf(FName MotorId, bool& bOutSlide) const;

	TWeakObjectPtr<AMjArticulation>							 Articulation;
	mutable TWeakObjectPtr<URammsChaosArticulationComponent> Chaos;
	mutable uint64											 LastLookFrame = MAX_uint64;
};

/**
 * What RammsMujocoSupport registers as RammsCore's MuJoCo backend: a URLab articulation's motors, on
 * whichever engine simulates it. Everything goes to the Chaos backend while the articulation carries
 * an active URammsChaosArticulationComponent, to the MuJoCo one otherwise, so a robot switched to Chaos
 * (in its Blueprint, or at runtime with SwitchToChaos) keeps its controllers and its control surface.
 */
class FRammsArticulationActuationBackend final : public IRammsActuationBackend
{
public:
	virtual bool   Initialize(URammsRobotBaseComponent& Base) override;
	virtual void   SetCommand(FName MotorId, float Value) override { Active().SetCommand(MotorId, Value); }
	virtual float  GetValue(FName MotorId) const override { return Active().GetValue(MotorId); }
	virtual float  GetVelocity(FName MotorId) const override { return Active().GetVelocity(MotorId); }
	virtual bool   ReleaseMotor(FName MotorId) override { return Active().ReleaseMotor(MotorId); }
	virtual bool   GetMotorTransform(FName MotorId, FTransform& OutWorld) const override { return Active().GetMotorTransform(MotorId, OutWorld); }
	virtual double GetSimulationTime() const override { return Active().GetSimulationTime(); }
	virtual bool   SetVelocityDrive(FName MotorId, float TargetRate, const FRammsVelocityGains& Gains,
		float TorqueMin, float TorqueMax) override;
	virtual void   ClearVelocityDrive(FName MotorId) override;
	virtual bool   SimulatesMotorJoint(FName MotorId) const override { return Active().SimulatesMotorJoint(MotorId); }
	virtual bool   SetMotorBrake(FName MotorId, bool bEngaged) override { return Active().SetMotorBrake(MotorId, bEngaged); }

private:
	IRammsActuationBackend& Active() const;

	FRammsMujocoActuationBackend   Mujoco;
	FRammsChaosArticulationBackend Chaos;
	FString						   OwnerName;
	mutable TOptional<bool>		   bWasChaos; // for logging the engine once, and again on a switch
};
