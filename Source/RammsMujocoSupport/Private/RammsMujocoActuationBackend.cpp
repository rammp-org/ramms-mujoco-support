// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMujocoActuationBackend.h"
#include "RammsRobotBaseComponent.h"
#include "MuJoCo/Controllers/MjArticulationController.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
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

	// Resolve the articulation: the owner itself, else one attached under it
	// (a composed robot whose base component lives on a parent actor). Never
	// an arbitrary articulation elsewhere in the level — with Backend=Auto that
	// would silently hijack an unrelated robot's actuators (and its control
	// source) while leaving this one undriven.
	AActor* Owner = Base.GetOwner();
	if (!Owner)
	{
		return false;
	}
	if (AMjArticulation* AsArt = Cast<AMjArticulation>(Owner))
	{
		Articulation = AsArt;
	}
	else
	{
		TArray<AActor*> Children;
		Owner->GetAttachedActors(Children, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
		for (AActor* Child : Children)
		{
			if (AMjArticulation* ChildArt = Cast<AMjArticulation>(Child))
			{
				Articulation = ChildArt;
				break;
			}
		}
	}
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
		DriveTable->Articulation = nullptr;
	}
}

bool FRammsMujocoActuationBackend::SetVelocityDrive(FName MotorId, float TargetRate, const FRammsVelocityGains& Gains,
	float TorqueMin, float TorqueMax)
{
	UMjNodeComponent*	 Actuator = ResolveActuator(MotorId);
	const FTransmission* Transmission = ResolveTransmission(MotorId);
	if (!Actuator || !Transmission || !Articulation.IsValid())
	{
		return false; // not compiled yet: the base runs the loop meanwhile
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
	if (Joint < 0 || Joint >= Model->njnt)
	{
		return false;
	}

	if (!DriveTable)
	{
		DriveTable = MakeShared<FDriveTable, ESPMode::ThreadSafe>();
	}
	{
		FScopeLock	   Lock(&DriveTable->Lock);
		FPhysicsDrive& Drive = DriveTable->Drives.FindOrAdd(MotorId);
		if (Drive.ActuatorId != ActuatorId)
		{
			Drive = FPhysicsDrive(); // a recompile can move the ids
		}
		DriveTable->Articulation = Articulation.Get();
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
	if (!bDriveCallbackRegistered)
	{
		TSharedPtr<FDriveTable, ESPMode::ThreadSafe> Table = DriveTable;
		Engine->RegisterPreStepCallback([Table](mjModel* M, mjData* D) { StepDrives(*Table, M, D); });
		bDriveCallbackRegistered = true;
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

void FRammsMujocoActuationBackend::StepDrives(FDriveTable& Table, const mjModel_* Model, const mjData_* Data)
{
	FScopeLock Lock(&Table.Lock);
	if (!Table.bAlive || !Table.Articulation || !Model || !Data)
	{
		return;
	}
	const double Dt = Model->opt.timestep;
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
	// mjData::time rather than the manager's float, which loses the
	// millisecond a controller differentiates against after a few hours.
	const UMjPhysicsEngine* Engine = Articulation.IsValid() ? AAMjManager::ResolveEngine(Articulation.Get()) : nullptr;
	const mjData*			Data = Engine ? Engine->GetData() : nullptr;
	return Data ? Data->time : -1.0;
}
