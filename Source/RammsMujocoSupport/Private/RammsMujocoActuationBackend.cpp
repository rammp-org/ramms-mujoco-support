// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMujocoActuationBackend.h"
#include "RammsRobotBaseComponent.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "GameFramework/Actor.h"
#include "mujoco/mujoco.h"

bool FRammsMujocoActuationBackend::Initialize(URammsRobotBaseComponent& Base)
{
	Articulation = nullptr;
	ActuatorByMotor.Reset();

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
	UMjNodeComponent* Actuator = ResolveActuator(MotorId);
	// Actuator transmission length == the driven joint's position/angle for the
	// single-joint transmissions used here.
	return Actuator ? UMjActuatorRuntime::GetLength(Actuator) : 0.0f;
}

float FRammsMujocoActuationBackend::GetVelocity(FName MotorId) const
{
	UMjNodeComponent* Actuator = ResolveActuator(MotorId);
	return Actuator ? UMjActuatorRuntime::GetVelocity(Actuator) : 0.0f;
}

bool FRammsMujocoActuationBackend::GetMotorTransform(FName MotorId, FTransform& OutWorld) const
{
	UMjNodeComponent* Actuator = ResolveActuator(MotorId);
	if (!Actuator)
	{
		return false;
	}

	// <actuator> elements live at the model root, not on the body they drive,
	// so the actuator node's own transform says nothing about where the motor
	// is. Resolve the transmission target instead: actuator -> joint -> body,
	// and report that body's world transform (the wheel, for a drive motor).
	AMjArticulation*		Art = Articulation.Get();
	const TOptional<int32>& BoundId = Actuator->GetBoundId();
	if (Art && BoundId.IsSet() && BoundId.GetValue() >= 0)
	{
		if (const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Actuator))
		{
			if (const mjModel* Model = Engine->GetModel())
			{
				const int32 ActId = BoundId.GetValue();
				if (ActId < Model->nu && Model->actuator_trntype[ActId] == mjTRN_JOINT)
				{
					const int32 JointId = Model->actuator_trnid[2 * ActId];
					if (JointId >= 0 && JointId < Model->njnt)
					{
						if (UMjBody* Body = Art->GetBodyByMjId(Model->jnt_bodyid[JointId]))
						{
							OutWorld = Body->GetComponentTransform();
							return true;
						}
					}
				}
			}
		}
	}

	// Not a joint transmission (tendon/site/body) or not yet compiled: the
	// element node's transform is the best available.
	OutWorld = Actuator->GetComponentTransform();
	return true;
}
