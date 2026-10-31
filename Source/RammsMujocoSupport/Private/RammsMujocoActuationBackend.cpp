// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMujocoActuationBackend.h"
#include "RammsRobotBaseComponent.h"
#include "MuJoCo/Controllers/MjArticulationController.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "GameFramework/Actor.h"
#include "mujoco/mujoco.h"

bool FRammsMujocoActuationBackend::Initialize(URammsRobotBaseComponent& Base)
{
	Articulation = nullptr;
	ActuatorByMotor.Reset();
	TransmissionByMotor.Reset();
	WarnedUnsupported.Reset();

	AActor* Owner = Base.GetOwner();
	BaseComponent = &Base;
	Articulation = FindArticulation(Base);
	if (!Articulation.IsValid())
	{
		return false; // no MuJoCo articulation on this robot — base component falls back
	}

	// UE owns the drive: select the internal (UI/Blueprint) control slot so
	// staged values reach d->ctrl (0 = ZMQ/network, non-zero = UI).
	Articulation->ControlSource = 1;

	// An enabled articulation controller takes over ApplyControls for the whole
	// articulation: URLab returns right after ComputeAndApply, so the staged
	// slots of actuators that controller doesn't own are never applied. Nothing
	// this backend can do about it from outside — say so loudly.
	if (const UMjArticulationController* Controller = Articulation->FindComponentByClass<UMjArticulationController>())
	{
		if (Controller->bEnabled)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("RammsMujocoActuationBackend on '%s': articulation '%s' has an enabled %s (%s). URLab applies only that controller's ctrl while it is enabled, so motor commands routed through the base component will be ignored for actuators it does not drive. Disable it, or apply the URLab fix that merges staged controls under a controller."),
				*Owner->GetName(), *Articulation->GetName(), *Controller->GetClass()->GetName(), *Controller->GetKindName());
		}
	}
	return true;
}

AMjArticulation* FRammsMujocoActuationBackend::FindArticulation(const URammsRobotBaseComponent& Base)
{
	// The owner itself, else one attached under it (a composed robot whose base
	// component lives on a parent actor). Never an arbitrary articulation
	// elsewhere in the level — with Backend=Auto that would silently hijack an
	// unrelated robot's actuators (and its control source) while leaving this
	// one undriven.
	AActor* Owner = Base.GetOwner();
	if (!Owner)
	{
		return nullptr;
	}
	if (AMjArticulation* AsArt = Cast<AMjArticulation>(Owner))
	{
		return AsArt;
	}
	TArray<AActor*> Children;
	Owner->GetAttachedActors(Children, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
	for (AActor* Child : Children)
	{
		if (AMjArticulation* ChildArt = Cast<AMjArticulation>(Child))
		{
			return ChildArt;
		}
	}
	return nullptr;
}

UMjNodeComponent* FRammsMujocoActuationBackend::ResolveActuator(FName MotorId) const
{
	if (const TWeakObjectPtr<UMjNodeComponent>* Cached = ActuatorByMotor.Find(MotorId))
	{
		if (Cached->IsValid())
		{
			return Cached->Get();
		}
	}
	AMjArticulation* Art = Articulation.Get();
	if (!Art)
	{
		return nullptr;
	}
	UMjNodeComponent* Actuator = Art->GetActuator(MotorId.ToString());
	ActuatorByMotor.Add(MotorId, Actuator);
	return Actuator;
}

const FRammsMujocoActuationBackend::FTransmission* FRammsMujocoActuationBackend::ResolveTransmission(FName MotorId) const
{
	if (const FTransmission* Cached = TransmissionByMotor.Find(MotorId))
	{
		if (Cached->Joint.IsValid())
		{
			return Cached;
		}
		TransmissionByMotor.Remove(MotorId); // stale after a recompile: re-resolve
	}

	UMjNodeComponent* Actuator = ResolveActuator(MotorId);
	AMjArticulation*  Art = Articulation.Get();
	if (!Actuator || !Art)
	{
		return nullptr;
	}
	const TOptional<int32>& BoundId = Actuator->GetBoundId();
	if (!BoundId.IsSet() || BoundId.GetValue() < 0)
	{
		return nullptr; // not compiled yet
	}
	const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	const int32				ActId = BoundId.GetValue();
	if (!Model || ActId >= Model->nu)
	{
		return nullptr;
	}

	if (Model->actuator_trntype[ActId] != mjTRN_JOINT)
	{
		if (!WarnedUnsupported.Contains(MotorId))
		{
			WarnedUnsupported.Add(MotorId);
			UE_LOG(LogTemp, Warning,
				TEXT("RammsMujocoActuationBackend: actuator '%s' has a non-joint transmission (type %d); value/velocity/transform reads are unsupported for it (commands still work)."),
				*MotorId.ToString(), static_cast<int32>(Model->actuator_trntype[ActId]));
		}
		return nullptr;
	}

	const int32 JointId = Model->actuator_trnid[2 * ActId];
	if (JointId < 0 || JointId >= Model->njnt)
	{
		return nullptr;
	}
	UMjNodeComponent* Joint = Art->GetComponentByMjId(mjOBJ_JOINT, JointId);
	if (!Joint)
	{
		return nullptr;
	}

	FTransmission Trn;
	Trn.Joint = Joint;
	Trn.BodyId = Model->jnt_bodyid[JointId];
	Trn.bSlide = Model->jnt_type[JointId] == mjJNT_SLIDE;
	return &TransmissionByMotor.Add(MotorId, Trn);
}

void FRammsMujocoActuationBackend::SetCommand(FName MotorId, float Value)
{
	// A command staged directly wins over a profile, which would otherwise
	// stage over it from the physics thread every step.
	ClearPositionProfile(MotorId);
	UMjNodeComponent* Actuator = ResolveActuator(MotorId);
	if (!Actuator)
	{
		return;
	}
	// Clamp to the compiled actuator's ctrlrange so the staged value matches what
	// the integrator will apply (zero-width range = unlimited, leave as-is).
	const FVector2D Range = UMjActuatorRuntime::GetControlRange(Actuator);
	if (Range.X < Range.Y)
	{
		Value = FMath::Clamp(Value, static_cast<float>(Range.X), static_cast<float>(Range.Y));
	}
	UMjActuatorRuntime::SetControl(Actuator, Value);
}

float FRammsMujocoActuationBackend::GetValue(FName MotorId) const
{
	// The joint's own coordinate (not the gear-scaled actuator length): radians
	// for a hinge, metres -> cm for a slide.
	const FTransmission* Trn = ResolveTransmission(MotorId);
	if (!Trn)
	{
		return 0.0f;
	}
	const float Pos = UMjJointRuntime::GetPosition(Trn->Joint.Get());
	return Trn->bSlide ? Pos * 100.0f : Pos;
}

float FRammsMujocoActuationBackend::GetVelocity(FName MotorId) const
{
	const FTransmission* Trn = ResolveTransmission(MotorId);
	if (!Trn)
	{
		return 0.0f;
	}
	const float Vel = UMjJointRuntime::GetVelocity(Trn->Joint.Get());
	return Trn->bSlide ? Vel * 100.0f : Vel;
}

bool FRammsMujocoActuationBackend::ReleaseMotor(FName MotorId)
{
	// A MuJoCo actuator has no off switch: its ctrl is applied every step. So
	// "release" means staging a ctrl under which the actuator stops acting on
	// the joint — and reporting false whenever that isn't possible, so the
	// caller keeps treating the motor as driven:
	//  - <motor> (force, no bias):   ctrl 0 = zero force              -> released
	//  - <position> (bias kp < 0, no dynamics): ctrl = current actuator length -> parked
	//    where it is (holds; stops pursuing its old target). Read as the
	//    actuator LENGTH — gear- and transmission-aware — not the joint qpos.
	//  - <velocity> (bias kv only):  ctrl 0 would BRAKE the joint     -> false
	//  - anything with actuator dynamics (intvelocity, general):      -> false
	ClearPositionProfile(MotorId);
	UMjNodeComponent* Actuator = ResolveActuator(MotorId);
	if (!Actuator)
	{
		return false;
	}
	const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	const TOptional<int32>& BoundId = Actuator->GetBoundId();
	if (!Model || !BoundId.IsSet() || BoundId.GetValue() < 0 || BoundId.GetValue() >= Model->nu)
	{
		return false; // not compiled / not bound: nothing can be staged
	}
	const int32	  ActId = BoundId.GetValue();
	const mjtNum* Bias = &Model->actuator_biasprm[ActId * mjNBIAS];
	const bool	  bAffine = Model->actuator_biastype[ActId] == mjBIAS_AFFINE;
	// Stateless dynamics only: <intvelocity> also compiles with an affine
	// negative length bias, but its ctrl feeds an integrator — writing the
	// current length there would drive it, not park it.
	const bool bStateless = Model->actuator_dyntype[ActId] == mjDYN_NONE;
	const bool bPositionServo = bStateless && bAffine && Bias[1] < 0.0;
	const bool bPlainForce = Model->actuator_biastype[ActId] == mjBIAS_NONE && Model->actuator_dyntype[ActId] == mjDYN_NONE;
	float	   Park = 0.0f;
	if (bPositionServo)
	{
		Park = UMjActuatorRuntime::GetLength(Actuator);
	}
	else if (!bPlainForce)
	{
		if (!WarnedUnsupported.Contains(MotorId))
		{
			WarnedUnsupported.Add(MotorId);
			UE_LOG(LogTemp, Warning,
				TEXT("RammsMujocoActuationBackend: actuator '%s' cannot be released (a velocity servo / dynamic actuator keeps acting at ctrl 0); it stays driven."),
				*MotorId.ToString());
		}
		return false;
	}
	// A park value the compiled ctrlrange doesn't admit would be clamped into a
	// command that still moves the joint: that is not a release.
	const FVector2D Range = UMjActuatorRuntime::GetControlRange(Actuator);
	if (Range.X < Range.Y && (Park < Range.X - KINDA_SMALL_NUMBER || Park > Range.Y + KINDA_SMALL_NUMBER))
	{
		return false;
	}
	UMjActuatorRuntime::SetControl(Actuator, Park);
	// The staging slot can be gone mid-recompile (SetControl then no-ops):
	// only report released if the value actually landed.
	return FMath::IsNearlyEqual(static_cast<float>(UMjActuatorRuntime::GetControl(Actuator)), Park, 1e-4f);
}

bool FRammsMujocoActuationBackend::GetMotorTransform(FName MotorId, FTransform& OutWorld) const
{
	// <actuator> elements live at the model root, so the actuator node's own
	// transform says nothing about where the motor is; only the body of the
	// joint it drives does. Unavailable (not compiled / non-joint transmission)
	// is reported as such rather than as a bogus root transform.
	const FTransmission* Trn = ResolveTransmission(MotorId);
	AMjArticulation*	 Art = Articulation.Get();
	if (!Trn || !Art || Trn->BodyId < 0)
	{
		return false;
	}
	UMjBody* Body = Art->GetBodyByMjId(Trn->BodyId);
	if (!Body)
	{
		return false;
	}
	OutWorld = Body->GetComponentTransform();
	return true;
}

FRammsMujocoActuationBackend::~FRammsMujocoActuationBackend()
{
	if (DriveTable)
	{
		FScopeLock Lock(&DriveTable->Lock);
		DriveTable->bAlive = false;
		DriveTable->Drives.Reset();
		DriveTable->Profiles.Reset();
		DriveTable->Articulation = nullptr;
		// Kept for the callback to put back the friction they raised.
		for (TPair<FName, FPhysicsBrake>& Pair : DriveTable->Brakes)
		{
			Pair.Value.bEngaged = false;
		}
	}
}

bool FRammsMujocoActuationBackend::SetVelocityDrive(FName MotorId, float TargetRate, const FRammsVelocityGains& Gains,
	float TorqueMin, float TorqueMax)
{
	// False hands the loop back to the base, which then commands the motor once
	// per tick. A loop this backend was already running would otherwise stay in
	// the table and keep staging its old output from the physics thread every
	// step, over the base's command, so every refusal drops it first.
	const auto Refuse = [this, MotorId]() -> bool {
		ClearVelocityDrive(MotorId);
		return false;
	};
	UMjNodeComponent*	 Actuator = ResolveActuator(MotorId);
	const FTransmission* Transmission = ResolveTransmission(MotorId);
	if (!Actuator || !Transmission || !Articulation.IsValid())
	{
		return Refuse(); // not compiled yet: the base runs the loop meanwhile
	}
	const TOptional<int32>& Bound = Actuator->GetBoundId();
	UMjPhysicsEngine*		Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	if (!Bound.IsSet() || !Model)
	{
		return Refuse();
	}
	const int32 ActuatorId = Bound.GetValue();
	if (ActuatorId < 0 || ActuatorId >= Model->nu || Model->actuator_trntype[ActuatorId] != mjTRN_JOINT)
	{
		return Refuse();
	}
	const int32 Joint = Model->actuator_trnid[2 * ActuatorId];
	if (Joint < 0 || Joint >= Model->njnt)
	{
		return Refuse();
	}

	FDriveTable& Table = EnsureDriveTable(*Engine);
	{
		FScopeLock Lock(&Table.Lock);
		// A wheel driven at a speed is not also braked.
		if (FPhysicsBrake* Brake = Table.Brakes.Find(MotorId))
		{
			Brake->bEngaged = false;
		}
		FPhysicsDrive& Drive = Table.Drives.FindOrAdd(MotorId);
		if (Drive.ActuatorId != ActuatorId)
		{
			Drive = FPhysicsDrive(); // a recompile can move the ids
		}
		// A retune starts the integral over, as the base's own loop does: torque
		// accumulated under the old gains would be applied at once under the new
		// ones (and never bleed off with Ki 0). The same gains arrive every tick,
		// so this only fires on a real change.
		if (Drive.Kp != Gains.Kp || Drive.Ki != Gains.Ki || Drive.MaxIntegral != Gains.MaxIntegralTorque)
		{
			Drive.Integral = 0.0;
		}
		Table.Articulation = Articulation.Get();
		Drive.ActuatorId = ActuatorId;
		Drive.DofAdr = Model->jnt_dofadr[Joint];
		Drive.Scale = Transmission->bSlide ? 100.0 : 1.0;
		Drive.Target = TargetRate;
		Drive.Kp = Gains.Kp;
		Drive.Ki = Gains.Ki;
		Drive.MaxIntegral = Gains.MaxIntegralTorque;
		Drive.Lo = TorqueMin;
		Drive.Hi = TorqueMax;
	}
	return true;
}

void FRammsMujocoActuationBackend::ClearVelocityDrive(FName MotorId)
{
	if (DriveTable)
	{
		FScopeLock Lock(&DriveTable->Lock);
		DriveTable->Drives.Remove(MotorId);
	}
}

bool FRammsMujocoActuationBackend::SetPositionProfile(FName MotorId, float Target, float MaxVelocity, float MaxAcceleration,
	float StartFrom)
{
	// False hands the profile back to the base, which then steps it once per
	// tick; one this backend was running would otherwise stage over the base's
	// setpoint every step, so every refusal drops it first.
	const auto Refuse = [this, MotorId]() -> bool {
		ClearPositionProfile(MotorId);
		return false;
	};
	UMjNodeComponent*	 Actuator = ResolveActuator(MotorId);
	const FTransmission* Transmission = ResolveTransmission(MotorId);
	if (!Actuator || !Transmission || !Articulation.IsValid())
	{
		return Refuse(); // not compiled yet: the base eases it meanwhile
	}
	const TOptional<int32>& Bound = Actuator->GetBoundId();
	UMjPhysicsEngine*		Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	if (!Bound.IsSet() || !Model)
	{
		return Refuse();
	}
	const int32 ActuatorId = Bound.GetValue();
	if (ActuatorId < 0 || ActuatorId >= Model->nu || Model->actuator_trntype[ActuatorId] != mjTRN_JOINT)
	{
		return Refuse();
	}
	const int32 Joint = Model->actuator_trnid[2 * ActuatorId];
	if (Joint < 0 || Joint >= Model->njnt)
	{
		return Refuse();
	}
	// Clamped as SetCommand clamps what it stages.
	const FVector2D Range = UMjActuatorRuntime::GetControlRange(Actuator);

	FDriveTable&	 Table = EnsureDriveTable(*Engine);
	FScopeLock		 Lock(&Table.Lock);
	FPhysicsProfile& Profile = Table.Profiles.FindOrAdd(MotorId);
	if (Profile.ActuatorId != ActuatorId)
	{
		Profile = FPhysicsProfile(); // a recompile can move the ids
	}
	Table.Articulation = Articulation.Get();
	Profile.ActuatorId = ActuatorId;
	Profile.QposAdr = Model->jnt_qposadr[Joint];
	Profile.Scale = Transmission->bSlide ? 100.0 : 1.0;
	Profile.Lo = Range.X;
	Profile.Hi = Range.Y;
	Profile.MaxVelocity = MaxVelocity;
	Profile.MaxAcceleration = MaxAcceleration;
	Profile.Target = Target;
	if (!Profile.bStarted && FMath::IsFinite(StartFrom))
	{
		Profile.StartFrom = StartFrom;
		Profile.bStartGiven = true;
	}
	++Profile.Commands;
	return true;
}

void FRammsMujocoActuationBackend::ClearPositionProfile(FName MotorId)
{
	if (DriveTable)
	{
		FScopeLock Lock(&DriveTable->Lock);
		DriveTable->Profiles.Remove(MotorId);
	}
}

bool FRammsMujocoActuationBackend::ResolveBrake(FName MotorId, UMjPhysicsEngine*& OutEngine, const mjModel_*& OutModel, int32& OutDofAdr,
	double& OutTorque) const
{
	UMjNodeComponent*	 Actuator = ResolveActuator(MotorId);
	const FTransmission* Transmission = ResolveTransmission(MotorId);
	if (!Actuator || !Transmission || !Articulation.IsValid())
	{
		return false;
	}
	const TOptional<int32>& Bound = Actuator->GetBoundId();
	UMjPhysicsEngine*		Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	if (!Bound.IsSet() || !Model)
	{
		return false;
	}
	const int32 ActuatorId = Bound.GetValue();
	if (ActuatorId < 0 || ActuatorId >= Model->nu || Model->actuator_trntype[ActuatorId] != mjTRN_JOINT)
	{
		return false;
	}
	const int32 Joint = Model->actuator_trnid[2 * ActuatorId];
	if (Joint < 0 || Joint >= Model->njnt || (Model->jnt_type[Joint] != mjJNT_HINGE && Model->jnt_type[Joint] != mjJNT_SLIDE))
	{
		return false;
	}
	// A motor: its force its gain times its command, nothing more. A servo
	// (position, velocity) or an actuator with dynamics holds by its own law.
	if (Model->actuator_gaintype[ActuatorId] != mjGAIN_FIXED || Model->actuator_biastype[ActuatorId] != mjBIAS_NONE
		|| Model->actuator_dyntype[ActuatorId] != mjDYN_NONE)
	{
		return false;
	}
	// It holds with its own rating (FRammsMotorSpec::BrakeTorque), else with
	// what the motor can do about its joint: a steeper slope rolls the robot.
	FRammsMotorSpec					Spec;
	const URammsRobotBaseComponent* Base = BaseComponent.Get();
	if (Base && Base->GetMotorSpec(MotorId, Spec) && Spec.BrakeTorque > 0.0f)
	{
		OutEngine = Engine;
		OutModel = Model;
		OutDofAdr = Model->jnt_dofadr[Joint];
		OutTorque = Spec.BrakeTorque;
		return true;
	}
	const mjtNum* Range = &Model->actuator_ctrlrange[2 * ActuatorId];
	const mjtNum* Force = &Model->actuator_forcerange[2 * ActuatorId];
	double		  Most = 0.0;
	if (Model->actuator_ctrllimited[ActuatorId])
	{
		Most = FMath::Max(FMath::Abs(Range[0]), FMath::Abs(Range[1])) * FMath::Abs(Model->actuator_gainprm[ActuatorId * mjNGAIN]);
	}
	if (Model->actuator_forcelimited[ActuatorId])
	{
		const double Limit = FMath::Max(FMath::Abs(Force[0]), FMath::Abs(Force[1]));
		Most = Most > 0.0 ? FMath::Min(Most, Limit) : Limit;
	}
	Most *= FMath::Abs(Model->actuator_gear[6 * ActuatorId]);
	if (Model->jnt_actfrclimited[Joint])
	{
		const mjtNum* Joints = &Model->jnt_actfrcrange[2 * Joint];
		const double  Limit = FMath::Max(FMath::Abs(Joints[0]), FMath::Abs(Joints[1]));
		Most = Most > 0.0 ? FMath::Min(Most, Limit) : Limit;
	}
	// An unlimited motor: a brake that holds whatever the robot puts on it.
	constexpr double UnlimitedHoldNm = 1000.0;
	OutEngine = Engine;
	OutModel = Model;
	OutDofAdr = Model->jnt_dofadr[Joint];
	OutTorque = Most > 0.0 ? Most : UnlimitedHoldNm;
	return true;
}

bool FRammsMujocoActuationBackend::HasMotorBrake(FName MotorId) const
{
	UMjPhysicsEngine* Engine = nullptr;
	const mjModel*	  Model = nullptr;
	int32			  DofAdr = INDEX_NONE;
	double			  Torque = 0.0;
	return ResolveBrake(MotorId, Engine, Model, DofAdr, Torque);
}

bool FRammsMujocoActuationBackend::SetMotorBrake(FName MotorId, bool bEngaged)
{
	UMjPhysicsEngine* Engine = nullptr;
	const mjModel*	  Model = nullptr;
	int32			  DofAdr = INDEX_NONE;
	double			  Torque = 0.0;
	const bool		  bHas = ResolveBrake(MotorId, Engine, Model, DofAdr, Torque);
	if (!bEngaged)
	{
		if (DriveTable)
		{
			FScopeLock Lock(&DriveTable->Lock);
			if (FPhysicsBrake* Brake = DriveTable->Brakes.Find(MotorId))
			{
				Brake->bEngaged = false; // the callback puts the friction back
			}
		}
		return bHas;
	}
	if (!bHas)
	{
		return false;
	}
	FDriveTable& Table = EnsureDriveTable(*Engine);
	FScopeLock	 Lock(&Table.Lock);
	// A braked wheel is not also driven at a speed.
	Table.Drives.Remove(MotorId);
	Table.Articulation = Articulation.Get();
	FPhysicsBrake& Brake = Table.Brakes.FindOrAdd(MotorId);
	Brake.Model = Model;
	Brake.DofAdr = DofAdr;
	Brake.Torque = Torque;
	Brake.bEngaged = true;
	return true;
}

FRammsMujocoActuationBackend::FDriveTable& FRammsMujocoActuationBackend::EnsureDriveTable(UMjPhysicsEngine& Engine)
{
	if (!DriveTable)
	{
		DriveTable = MakeShared<FDriveTable, ESPMode::ThreadSafe>();
	}
	if (!bDriveCallbackRegistered)
	{
		TSharedPtr<FDriveTable, ESPMode::ThreadSafe> Table = DriveTable;
		Engine.RegisterPreStepCallback([Table](mjModel* M, mjData* D) { StepTable(*Table, M, D); });
		bDriveCallbackRegistered = true;
	}
	return *DriveTable;
}

void FRammsMujocoActuationBackend::StepTable(FDriveTable& Table, mjModel_* Model, const mjData_* Data)
{
	FScopeLock Lock(&Table.Lock);
	if (!Model || !Data)
	{
		return;
	}
	// First, and on a dead table too: it still puts back what it raised.
	StepBrakes(Table, Model);
	if (!Table.bAlive || !Table.Articulation)
	{
		return;
	}
	// The time since the last callback, on the simulation's own clock: none
	// while it is paused (the callbacks still run then, and a loop integrating
	// a fixed timestep would wind up) and the whole batch when one request
	// steps several. A clock gone back is a reset: the robot is back at its
	// start, so every loop and profile starts over.
	const double Now = Data->time;
	double		 Dt = 0.0;
	if (Table.LastTime >= 0.0 && Now < Table.LastTime)
	{
		for (TPair<FName, FPhysicsDrive>& Pair : Table.Drives)
		{
			Pair.Value.Integral = 0.0;
		}
		for (TPair<FName, FPhysicsProfile>& Pair : Table.Profiles)
		{
			Pair.Value.bStarted = false;
		}
	}
	else if (Table.LastTime >= 0.0)
	{
		Dt = Now - Table.LastTime;
	}
	Table.LastTime = Now;
	StepDrives(Table, Model, Data, Dt);
	StepProfiles(Table, Model, Data, Dt);
}

void FRammsMujocoActuationBackend::StepBrakes(FDriveTable& Table, mjModel_* Model)
{
	// MuJoCo makes its constraints afresh every step from the model, so a
	// joint's friction loss raised before the step holds it in that step's
	// solve: dry friction up to the brake's torque, together with the
	// contacts, at the physics rate. A brake resolved against another model
	// (a recompile since) holds nothing until it is set again.
	//
	// The friction is made hard too. MuJoCo's constraints are soft: under a
	// steady load one slides at about the load times its regulariser over its
	// damping, and the regulariser scales with the inverse of the inertia the
	// dof sees -- a wheel's own spin inertia, small -- so at the default
	// impedance (0.9) a braked wheel carrying a share of a robot on a slope
	// slid at tenths of a rad/s. At 0.999 over a time constant of two steps it
	// slides about a thousand times slower.
	static_assert(sizeof(FPhysicsBrake::OriginalRef) == mjNREF * sizeof(mjtNum), "a dof's solref");
	static_assert(sizeof(FPhysicsBrake::OriginalImp) == mjNIMP * sizeof(mjtNum), "a dof's solimp");
	constexpr double BrakeImpedance = 0.999;
	const double	 BrakeTimeConst = 2.0 * Model->opt.timestep;
	for (auto It = Table.Brakes.CreateIterator(); It; ++It)
	{
		FPhysicsBrake& Brake = It.Value();
		const bool	   bHold = Table.bAlive && Brake.bEngaged && Brake.Model == Model && Brake.DofAdr >= 0 && Brake.DofAdr < Model->nv;
		mjtNum* const  Ref = bHold || Brake.RaisedIn == Model ? &Model->dof_solref[Brake.DofAdr * mjNREF] : nullptr;
		mjtNum* const  Imp = Ref ? &Model->dof_solimp[Brake.DofAdr * mjNIMP] : nullptr;
		if (bHold)
		{
			if (Brake.RaisedIn != Model)
			{
				Brake.Original = Model->dof_frictionloss[Brake.DofAdr];
				FMemory::Memcpy(Brake.OriginalRef, Ref, sizeof(Brake.OriginalRef));
				FMemory::Memcpy(Brake.OriginalImp, Imp, sizeof(Brake.OriginalImp));
				Brake.RaisedIn = Model;
			}
			Model->dof_frictionloss[Brake.DofAdr] = FMath::Max(Brake.Original, Brake.Torque);
			Ref[0] = BrakeTimeConst;
			Ref[1] = 1.0;
			Imp[0] = Imp[1] = BrakeImpedance; // a friction constraint sits at the low end: no violation
			continue;
		}
		if (Brake.RaisedIn == Model)
		{
			Model->dof_frictionloss[Brake.DofAdr] = Brake.Original;
			FMemory::Memcpy(Ref, Brake.OriginalRef, sizeof(Brake.OriginalRef));
			FMemory::Memcpy(Imp, Brake.OriginalImp, sizeof(Brake.OriginalImp));
		}
		Brake.RaisedIn = nullptr;
		if (!Brake.bEngaged)
		{
			It.RemoveCurrent();
		}
	}
}

void FRammsMujocoActuationBackend::StepProfiles(FDriveTable& Table, const mjModel_* Model, const mjData_* Data, double Dt)
{
	// Each profile is stepped over the time since the last callback and its
	// setpoint staged for the step about to run; a command is taken as given at
	// the start of that time.
	const double From = Data->time - Dt;
	for (TPair<FName, FPhysicsProfile>& Pair : Table.Profiles)
	{
		FPhysicsProfile& Profile = Pair.Value;
		if (Profile.ActuatorId < 0 || Profile.ActuatorId >= Model->nu || Profile.QposAdr < 0 || Profile.QposAdr >= Model->nq)
		{
			continue;
		}
		if (!Profile.bStarted)
		{
			// Where the base says (where the joint stood when it was first
			// commanded), else -- after a reset -- where the joint is now.
			Profile.Follower.Reset(Profile.bStartGiven ? Profile.StartFrom : Data->qpos[Profile.QposAdr] * Profile.Scale);
			Profile.bStartGiven = false;
			Profile.bStarted = true;
			Profile.SeenCommands = Profile.Commands - 1;
		}
		if (Profile.SeenCommands != Profile.Commands)
		{
			Profile.Follower.Command(Profile.Target, From);
			Profile.SeenCommands = Profile.Commands;
		}
		double Setpoint = Profile.Follower.Step(From, Dt, Profile.MaxVelocity, Profile.MaxAcceleration);
		if (Profile.Lo < Profile.Hi)
		{
			Setpoint = FMath::Clamp(Setpoint, Profile.Lo, Profile.Hi);
		}
		Table.Articulation->StageInternalControl(Profile.ActuatorId, Setpoint);
	}
}

void FRammsMujocoActuationBackend::StepDrives(FDriveTable& Table, const mjModel_* Model, const mjData_* Data, double Dt)
{
	for (TPair<FName, FPhysicsDrive>& Pair : Table.Drives)
	{
		FPhysicsDrive& Drive = Pair.Value;
		if (Drive.ActuatorId < 0 || Drive.ActuatorId >= Model->nu || Drive.DofAdr < 0 || Drive.DofAdr >= Model->nv)
		{
			continue;
		}
		// The same PI the base runs, in the joint's sense: integral capped,
		// held while the output is saturated so it does not wind up.
		const double Error = Drive.Target - Data->qvel[Drive.DofAdr] * Drive.Scale;
		double		 Integral = Drive.Integral + Drive.Ki * Error * Dt;
		if (Drive.MaxIntegral > 0.0)
		{
			Integral = FMath::Clamp(Integral, -Drive.MaxIntegral, Drive.MaxIntegral);
		}
		double Output = Drive.Kp * Error + Integral;
		if (Drive.Lo < Drive.Hi)
		{
			const double Clamped = FMath::Clamp(Output, Drive.Lo, Drive.Hi);
			if (Clamped != Output)
			{
				Integral = Drive.Integral;
			}
			Output = Clamped;
		}
		Drive.Integral = Integral;
		// Staged, not written to mjData: URLab's ApplyControls copies staged
		// controls into ctrl right after the pre-step callbacks, so a direct
		// write would be overwritten in the same step.
		Table.Articulation->StageInternalControl(Drive.ActuatorId, Output);
	}
}

double FRammsMujocoActuationBackend::GetSimulationTime() const
{
	// The time of the snapshot URLab last published, not live mjData::time:
	// the worker advances that mid-step, under a lock it holds for the whole
	// step, and RK4 writes its stage times there and then rewinds, so an
	// unlocked read can see time run backwards, which a controller takes for a
	// reset. GetValue and GetVelocity read the same published snapshots
	// (URLab's joint accessors do), so the clock, like the measurements, is
	// always the state at the end of a step. A double, unlike the manager's
	// float, which loses the millisecond a controller differentiates against
	// after a few hours.
	const UMjPhysicsEngine* Engine = Articulation.IsValid() ? AAMjManager::ResolveEngine(Articulation.Get()) : nullptr;
	if (!Engine)
	{
		return -1.0;
	}
	double Time = -1.0;
	Engine->WithRenderState([&Time](const FMjRenderSnapshot& Snapshot) {
		if (Snapshot.FrameId > 0) // nothing is published before the first compile
		{
			Time = Snapshot.SimTime;
		}
	});
	return Time;
}
