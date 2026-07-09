// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMjArmTeleopComponent.h"
#include "RammsMjEndEffectorController.h"
#include "GameFramework/PlayerController.h"

URammsMjArmTeleopComponent::URammsMjArmTeleopComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void URammsMjArmTeleopComponent::BeginPlay()
{
	Super::BeginPlay();
	ResolveController();
}

URammsMjEndEffectorController* URammsMjArmTeleopComponent::ResolveController()
{
	if (Controller)
	{
		return Controller;
	}
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return nullptr;
	}

	TArray<URammsMjEndEffectorController*> Found;
	Owner->GetComponents<URammsMjEndEffectorController>(Found);
	for (URammsMjEndEffectorController* C : Found)
	{
		if (C && (ControllerComponentName == NAME_None || C->GetFName() == ControllerComponentName))
		{
			Controller = C;
			break;
		}
	}

	if (!Controller && !bLoggedMissing)
	{
		bLoggedMissing = true;
		UE_LOG(LogTemp, Warning, TEXT("[MjTeleop] %s found no RammsMjEndEffectorController on %s"),
			*GetName(), *GetNameSafe(Owner));
	}
	return Controller;
}

void URammsMjArmTeleopComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bTeleopEnabled || !ResolveController())
	{
		return;
	}

	UWorld* World = GetWorld();
	APlayerController* PC = World ? World->GetFirstPlayerController() : nullptr;
	if (!PC || !PC->IsLocalPlayerController())
	{
		return;
	}

	// Speed scale from modifiers.
	float Scale = 1.0f;
	if (PC->IsInputKeyDown(EKeys::LeftShift) || PC->IsInputKeyDown(EKeys::RightShift))
		Scale *= FastMultiplier;
	if (PC->IsInputKeyDown(EKeys::LeftControl) || PC->IsInputKeyDown(EKeys::RightControl))
		Scale *= SlowMultiplier;

	// Semantic inputs (intent), before mapping into the gripper frame. Keys are configurable; a valid
	// FKey that isn't pressed simply reads 0, and an unset (None) key never contributes.
	auto Axis = [PC](const FKey& Pos, const FKey& Neg)
	{
		const float p = (Pos.IsValid() && PC->IsInputKeyDown(Pos)) ? 1.f : 0.f;
		const float n = (Neg.IsValid() && PC->IsInputKeyDown(Neg)) ? 1.f : 0.f;
		return p - n;
	};
	const float Fwd    = Axis(ForwardKey, BackwardKey);       // forward / back
	const float Strafe = Axis(StrafeRightKey, StrafeLeftKey); // right / left
	const float Up     = Axis(UpKey, DownKey);                // up / down
	const float Yaw    = Axis(YawRightKey, YawLeftKey);
	const float Pitch  = Axis(PitchUpKey, PitchDownKey);
	const float Roll   = Axis(RollRightKey, RollLeftKey);

	// Translation in the gripper frame: forward = local Z, strafe = local Y, up = local X.
	FVector Lin;
	Lin.X = Up * UpSign;
	Lin.Y = Strafe * StrafeSign;
	Lin.Z = Fwd * ForwardSign;
	const FVector LinDelta = Lin * (LinearSpeed * Scale * DeltaTime);

	// MoveTargetBy reads FRotator as rotations about local X(Roll)/Y(Pitch)/Z(Yaw). The gripper's
	// up axis is X, so: yaw -> .Roll, pitch -> .Pitch, roll (about forward Z) -> .Yaw.
	const float ARate = AngularSpeed * Scale * DeltaTime;
	FRotator RotDelta = FRotator::ZeroRotator;
	RotDelta.Roll  = Yaw   * YawSign   * ARate;   // yaw about up (X)
	RotDelta.Pitch = Pitch * PitchSign * ARate;   // pitch about Y
	RotDelta.Yaw   = Roll  * RollSign  * ARate;   // roll about forward (Z)

	// Right-mouse drag adds yaw (X) + pitch (Y), in degrees this frame.
	const bool bAllowMouse = bEnableMouseRotation
		&& (!bRequireRightMouseButton || PC->IsInputKeyDown(EKeys::RightMouseButton));
	if (bAllowMouse)
	{
		float MdX = 0.f, MdY = 0.f;
		PC->GetInputMouseDelta(MdX, MdY);
		const float PitchInv = bInvertMouseY ? 1.f : -1.f;
		RotDelta.Roll  += MdX * MouseYawDegreesPerPixel * Scale * YawSign;
		RotDelta.Pitch += PitchInv * MdY * MousePitchDegreesPerPixel * Scale * PitchSign;
	}

	if (!LinDelta.IsNearlyZero() || !RotDelta.IsNearlyZero())
	{
		Controller->MoveTargetBy(LinDelta, RotDelta, bLocalFrame);
	}

	// Gripper.
	if (OpenGripperKey.IsValid() && PC->WasInputKeyJustPressed(OpenGripperKey))
	{
		Controller->OpenGripper();
		bGripClosed = false;
	}
	if (CloseGripperKey.IsValid() && PC->WasInputKeyJustPressed(CloseGripperKey))
	{
		Controller->CloseGripper();
		bGripClosed = true;
	}
	if (ToggleGripperKey.IsValid() && PC->WasInputKeyJustPressed(ToggleGripperKey))
	{
		bGripClosed = !bGripClosed;
		Controller->SetGrip(bGripClosed ? 1.0f : 0.0f);
	}

	if (ResyncTargetKey.IsValid() && PC->WasInputKeyJustPressed(ResyncTargetKey))
	{
		Controller->ResyncTargetToCurrentPose();
	}
}
