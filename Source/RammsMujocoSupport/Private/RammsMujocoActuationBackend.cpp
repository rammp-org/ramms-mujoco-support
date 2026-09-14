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
