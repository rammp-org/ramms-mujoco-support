// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "RammsActuationBackend.h"
#include "RammsMotionProfile.h"
#include "UObject/WeakObjectPtr.h"

struct mjModel_;
struct mjData_;

class AMjArticulation;
class UMjNodeComponent;
class UMjPhysicsEngine;

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
	/** Steps the profile in the same pre-step callback, every physics step. */
	virtual bool SetPositionProfile(FName MotorId, float Target, float MaxVelocity, float MaxAcceleration,
		float StartFrom) override;
	virtual void ClearPositionProfile(FName MotorId) override;
	/** A plain force actuator (<motor>) on a hinge or slide joint. */
	virtual bool HasMotorBrake(FName MotorId) const override;
	/** Holds the joint with MuJoCo's own dry friction: its friction loss
	 *  raised to the brake's rating (FRammsMotorSpec::BrakeTorque), else to
	 *  the most the actuator can apply, in the same pre-step
	 *  callback, so the step's solver holds it against the contacts every
	 *  physics step -- a slope's pull, or the robot turning about the other
	 *  wheel. The actuator's own command still acts (a motor holding a slope's
	 *  pull, the brake the rest); a velocity loop on the motor is dropped, and
	 *  SetVelocityDrive releases the brake. */
	virtual bool SetMotorBrake(FName MotorId, bool bEngaged) override;

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

	/** The robot base this backend drives: its motor registry rates the brakes
	 *  (FRammsMotorSpec::BrakeTorque). */
	TWeakObjectPtr<const URammsRobotBaseComponent> BaseComponent;

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

	/** One position actuator's profile (SetPositionProfile), as the physics
	 *  thread steps it. Target and the setpoint are in ctrl's units. */
	struct FPhysicsProfile
	{
		int32				   ActuatorId = INDEX_NONE;
		int32				   QposAdr = INDEX_NONE;
		double				   Scale = 1.0; // robot units per qpos unit (100 for a slide joint)
		double				   Lo = 0.0;	// ctrlrange; Lo >= Hi: none
		double				   Hi = 0.0;
		double				   MaxVelocity = 0.0;
		double				   MaxAcceleration = 0.0;
		double				   Target = 0.0;
		double				   StartFrom = 0.0;
		bool				   bStartGiven = false;
		uint32				   Commands = 0; // counts every command; the physics thread notes the last it saw
		uint32				   SeenCommands = 0;
		bool				   bStarted = false;
		RammsMotion::FFollower Follower;
	};

	/** One motor's brake (SetMotorBrake), as the physics thread applies it:
	 *  its joint's friction loss raised to Torque while engaged, the friction
	 *  made hard, and both put back once released. */
	struct FPhysicsBrake
	{
		/** The model DofAdr indexes; the brake holds only in that one. */
		const mjModel_* Model = nullptr;
		int32			DofAdr = INDEX_NONE;
		double			Torque = 0.0;
		bool			bEngaged = false;
		/** The model the friction is raised in now (null: not raised), and the
		 *  joint's own friction loss and its solver parameters, to put back. */
		const mjModel_* RaisedIn = nullptr;
		double			Original = 0.0;
		double			OriginalRef[2] = {};
		double			OriginalImp[5] = {};
	};

	/** A motor's brake as SetMotorBrake would apply it: the model, the joint's
	 *  dof and the torque it holds with. False when the motor has none (not
	 *  compiled yet, not a plain force actuator on a hinge or slide). */
	bool ResolveBrake(FName MotorId, UMjPhysicsEngine*& OutEngine, const mjModel_*& OutModel, int32& OutDofAdr, double& OutTorque) const;

	/**
	 * The loops, profiles and brakes, shared with the pre-step callback. The
	 * callback outlives this backend -- URLab has no way to unregister one -- so
	 * it holds the table, and the destructor marks the table dead under its
	 * lock rather than freeing anything the physics thread may be reading (the
	 * callback puts back what the brakes raised).
	 */
	struct FDriveTable
	{
		FCriticalSection			 Lock;
		TMap<FName, FPhysicsDrive>	 Drives;
		TMap<FName, FPhysicsProfile> Profiles;
		TMap<FName, FPhysicsBrake>	 Brakes;
		AMjArticulation*			 Articulation = nullptr;
		bool						 bAlive = true;
		/** mjData::time at the last callback; negative before the first. */
		double LastTime = -1.0;
	};
	TSharedPtr<FDriveTable, ESPMode::ThreadSafe> DriveTable;
	bool										 bDriveCallbackRegistered = false;

	/** The table, made on first use, and the callback that steps it registered
	 *  with Engine. */
	FDriveTable& EnsureDriveTable(UMjPhysicsEngine& Engine);

	/** One physics step of every loop, profile and brake; physics thread,
	 *  under CallbackMutex. Time is the simulation's: no step while it is
	 *  paused. */
	static void StepTable(FDriveTable& Table, mjModel_* Model, const mjData_* Data);
	static void StepBrakes(FDriveTable& Table, mjModel_* Model);
	static void StepDrives(FDriveTable& Table, const mjModel_* Model, const mjData_* Data, double Dt);
	static void StepProfiles(FDriveTable& Table, const mjModel_* Model, const mjData_* Data, double Dt);
};
