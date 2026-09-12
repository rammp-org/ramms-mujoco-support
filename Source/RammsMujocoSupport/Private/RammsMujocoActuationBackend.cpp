// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMujocoActuationBackend.h"
#include "RammsRobotBaseComponent.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "GameFramework/Actor.h"
#include "EngineUtils.h"

bool FRammsMujocoActuationBackend::Initialize(URammsRobotBaseComponent& Base)
{
	Articulation = nullptr;
	ActuatorByMotor.Reset();

	// Resolve the articulation: the owner itself, else the first one in the
	// world (mirrors the skeletal-pose driver / drive backend).
	if (AActor* Owner = Base.GetOwner())
	{
		if (AMjArticulation* AsArt = Cast<AMjArticulation>(Owner))
		{
			Articulation = AsArt;
		}
	}
	if (!Articulation.IsValid())
	{
		if (UWorld* World = Base.GetWorld())
		{
			for (TActorIterator<AMjArticulation> It(World); It; ++It)
			{
				Articulation = *It;
				break;
			}
		}
	}
	if (!Articulation.IsValid())
	{
		return false; // no MuJoCo articulation — base component stays backend-less
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
	// UMjNodeComponent is a USceneComponent placed at the actuator's element in
	// the articulation hierarchy — good enough for derived geometry like
	// drive-motor separation (skid-steer track width).
	OutWorld = Actuator->GetComponentTransform();
	return true;
}
