// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "RammsActuationBackend.h"
#include "UObject/WeakObjectPtr.h"

struct mjModel_;
struct mjData_;

class AMjArticulation;
class UMjNodeComponent;

/**
 * IRammsActuationBackend backed by a URLab/MuJoCo articulation.
 *
 * A motor's registry Id is the MuJoCo actuator name (FRammsMotorSpec::Id is the
 * canonical, backend-neutral name and MuJoCo's by default). Commands stage onto
 * the actuator's internal (UI/Blueprint) control slot, clamped to the compiled
 * ctrlrange (the base component may also clamp to the registry ControlRange
 * first). MuJoCo interprets a staged value by the actuator's own kind (a <motor>
 * takes torque, a <position> a target, a <velocity> a speed), so the registry
 * type only documents intent and the same scalar is written for every type.
 *
 * Reads go through the actuator's **joint transmission** (actuator -> joint via
 * the compiled model), not the actuator itself: GetValue / GetVelocity return
 * the joint's own qpos / qvel (radians, or cm for a slide joint), never the
 * gear-scaled actuator length/velocity; GetMotorTransform reports the body that
 * joint belongs to (the wheel for a drive motor). Actuators with a non-joint
 * transmission (tendon / site / body) are unsupported: reads return 0 and the
 * transform is reported unavailable, with a one-time warning.
 *
 * Only an articulation that IS the owner actor, or is attached under it, is
 * driven — never one found elsewhere in the level — because Initialize also
 * switches that articulation's ControlSource to the UI slot (taking it away
 * from the ZMQ bridge), which must not happen to an unrelated robot.
 *
 * Known conflict: URLab's AMjArticulation::ApplyControls hands the whole step
 * to an enabled, bound UMjArticulationController and returns, so while such a
 * controller is active the staged slots of every other actuator on that
 * articulation are never copied into d->ctrl — commands routed through this
 * backend are silently dropped. Initialize warns when it finds one. (The clean
 * fix is in URLab: apply the staged owned-actuator controls, then let the
 * controller overwrite the ones it owns.)
 */
class FRammsMujocoActuationBackend final : public IRammsActuationBackend
{
public:
	virtual bool  Initialize(URammsRobotBaseComponent& Base) override;
	virtual void  SetCommand(FName MotorId, float Value) override;
	virtual float GetValue(FName MotorId) const override;
	virtual float GetVelocity(FName MotorId) const override;
	virtual bool  ReleaseMotor(FName MotorId) override;
	virtual bool  GetMotorTransform(FName MotorId, FTransform& OutWorld) const override;
	/** mjData::time as of the last snapshot the engine stepping this articulation
	 *  published; negative before the first one. */
	virtual double GetSimulationTime() const override;
	/** Runs the loop in a URLab pre-step callback, every physics step. */
	virtual bool SetVelocityDrive(FName MotorId, float TargetRate, const FRammsVelocityGains& Gains,
		float TorqueMin, float TorqueMax) override;
	virtual void ClearVelocityDrive(FName MotorId) override;

	virtual ~FRammsMujocoActuationBackend() override;

	/** The articulation a base component drives: its owner, else one attached under it (never one
	 *  elsewhere in the level). Null when there is none. */
	static AMjArticulation* FindArticulation(const URammsRobotBaseComponent& Base);

private:
	/** The joint an actuator's transmission drives, plus what a read needs. */
	struct FTransmission
	{
		TWeakObjectPtr<UMjNodeComponent> Joint;
		int32							 BodyId = -1;
		bool							 bSlide = false; // qpos/qvel in metres -> cm
	};

	/** Resolve (and cache) the actuator node for a motor Id, or nullptr. */
	UMjNodeComponent* ResolveActuator(FName MotorId) const;

	/** Resolve (and cache) the joint transmission for a motor Id; nullptr when
	 *  the model isn't compiled yet or the actuator has no joint transmission. */
	const FTransmission* ResolveTransmission(FName MotorId) const;

	TWeakObjectPtr<AMjArticulation> Articulation;

	/** Id -> actuator node, filled on first use. mutable: reads resolve lazily. */
	mutable TMap<FName, TWeakObjectPtr<UMjNodeComponent>> ActuatorByMotor;

	/** Id -> joint transmission, filled on first successful resolve. */
	mutable TMap<FName, FTransmission> TransmissionByMotor;

	/** Motors already warned about for a non-joint transmission. */
	mutable TSet<FName> WarnedUnsupported;

	/** One motor's speed loop, as the physics thread runs it. */
	struct FPhysicsDrive
	{
		int32  ActuatorId = INDEX_NONE;
		int32  DofAdr = INDEX_NONE;
		double Scale = 1.0; // robot units per qvel unit (100 for a slide joint)
		double Target = 0.0;
		double Kp = 0.0;
		double Ki = 0.0;
		double MaxIntegral = 0.0;
		double Lo = 0.0;
		double Hi = 0.0;
		double Integral = 0.0;
	};

	/**
	 * The loops, shared with the pre-step callback. The callback outlives this
	 * backend -- URLab has no way to unregister one -- so it holds the table,
	 * and the destructor marks the table dead under its lock rather than
	 * freeing anything the physics thread may be reading.
	 */
	struct FDriveTable
	{
		FCriticalSection		   Lock;
		TMap<FName, FPhysicsDrive> Drives;
		AMjArticulation*		   Articulation = nullptr;
		bool					   bAlive = true;
	};
	TSharedPtr<FDriveTable, ESPMode::ThreadSafe> DriveTable;
	bool										 bDriveCallbackRegistered = false;

	/** One physics step of every loop; physics thread, under CallbackMutex. */
	static void StepDrives(FDriveTable& Table, const mjModel_* Model, const mjData_* Data);
};
