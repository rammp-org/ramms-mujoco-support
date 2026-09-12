// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMujocoDriveBackend.h"
#include "RammsDifferentialDriveController.h"
#include "RammsDifferentialDriveTypes.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "GameFramework/Actor.h"
#include "EngineUtils.h"

bool FRammsMujocoDriveBackend::Initialize(URammsDifferentialDriveController& InController)
{
	Controller = &InController;
	Articulation = nullptr;

	// Resolve the articulation: the owner itself, else the first one in the world
	// (mirrors how the skeletal-pose driver finds its articulation).
	if (AActor* Owner = InController.GetOwner())
	{
		if (AMjArticulation* AsArt = Cast<AMjArticulation>(Owner))
		{
			Articulation = AsArt;
		}
	}
	if (!Articulation.IsValid())
	{
		if (UWorld* World = InController.GetWorld())
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
		return false; // no MuJoCo base to drive — controller falls back to Chaos
	}

	// UE owns the drive: select the internal (UI) control slot so the values
	// SetActuatorControl stages actually reach d->ctrl (0 = ZMQ, non-zero = UI).
	Articulation->ControlSource = 1;
	return true;
}

FString FRammsMujocoDriveBackend::ActuatorNameFor(ERammsDriveWheel Wheel) const
{
	const URammsDifferentialDriveController* Ctrl = Controller.Get();
	if (!Ctrl)
	{
		return FString();
	}
	const FName Bone = Wheel == ERammsDriveWheel::Left ? Ctrl->LeftWheelBoneName : Ctrl->RightWheelBoneName;
	return Bone.ToString();
}

void FRammsMujocoDriveBackend::ReadWheelState(ERammsDriveWheel Wheel, FWheelState& OutState)
{
	URammsDifferentialDriveController* Ctrl = Controller.Get();
	AMjArticulation*				   Art = Articulation.Get();
	if (!Ctrl || !Art)
	{
		return;
	}
	const FString	  ActuatorName = ActuatorNameFor(Wheel);
	UMjNodeComponent* Actuator = Art->GetActuator(ActuatorName);
	if (!Actuator)
	{
		return;
	}

	// MuJoCo actuator velocity == the driven wheel joint's angular velocity.
	OutState.AngularVelocity = UMjActuatorRuntime::GetVelocity(Actuator);
	OutState.LinearVelocity = OutState.AngularVelocity * Ctrl->WheelRadius;
	// Contact/slip is resolved inside MuJoCo, so the Chaos-style slip/traction
	// fields stay at their neutral defaults here (unused by this backend).
	OutState.LateralVelocity = 0.0f;
	OutState.SlipRatio = 0.0f;
}

void FRammsMujocoDriveBackend::ApplyWheelTorque(ERammsDriveWheel Wheel, float RequestedTorque,
	const FMotorParameters& /*MotorParams*/, FWheelState&					  WheelState)
{
	AMjArticulation* Art = Articulation.Get();
	if (!Art)
	{
		WheelState.AppliedTorque = 0.0f;
		return;
	}
	const FString ActuatorName = ActuatorNameFor(Wheel);

	// The motor actuator's ctrlrange clamps torque inside MuJoCo; clamp here too
	// so the reported AppliedTorque matches what the sim will use.
	const FVector2D Range = Art->GetActuatorRange(ActuatorName);
	float			Applied = RequestedTorque;
	if (Range.X < Range.Y)
	{
		Applied = FMath::Clamp(RequestedTorque, static_cast<float>(Range.X), static_cast<float>(Range.Y));
	}

	Art->SetActuatorControl(ActuatorName, Applied);
	WheelState.AppliedTorque = Applied;
}

void FRammsMujocoDriveBackend::ApplyBrake(ERammsDriveWheel Wheel, FWheelState& WheelState)
{
	URammsDifferentialDriveController* Ctrl = Controller.Get();
	AMjArticulation*				   Art = Articulation.Get();
	if (!Ctrl || !Art)
	{
		return;
	}
	const FString ActuatorName = ActuatorNameFor(Wheel);

	// Velocity-opposing torque, clamped to the actuator range (mirrors the Chaos
	// brake intent; MuJoCo joint damping does the rest at low speed).
	float			Brake = -FMath::Sign(WheelState.AngularVelocity) * Ctrl->BrakeTorque;
	const FVector2D Range = Art->GetActuatorRange(ActuatorName);
	if (Range.X < Range.Y)
	{
		Brake = FMath::Clamp(Brake, static_cast<float>(Range.X), static_cast<float>(Range.Y));
	}
	Art->SetActuatorControl(ActuatorName, Brake);
	WheelState.AppliedTorque = Brake;
}
