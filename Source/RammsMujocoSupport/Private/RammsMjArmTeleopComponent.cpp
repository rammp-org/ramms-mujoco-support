// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMjArmTeleopComponent.h"
#include "RammsMjEndEffectorController.h"
#include "MuJoCo/Core/MjArticulation.h"
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
	if (AMjArticulation* Art = Cast<AMjArticulation>(GetOwner()))
	{
		Art->OnSimulationReset.AddUniqueDynamic(this, &URammsMjArmTeleopComponent::HandleSimulationReset);
	}
}

void URammsMjArmTeleopComponent::HandleSimulationReset()
{
	// The controller re-seeds its target and opens the gripper on reset; drop
	// the cached rates and grip state here so they don't resume it.
	ControlLinear = FVector::ZeroVector;
	ControlAngular = FRotator::ZeroRotator;
	bGripClosed = false;
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

	// Control-surface rate axes (panels, input maps, remote clients).
	if (!ControlLinear.IsNearlyZero() || !ControlAngular.IsNearlyZero())
	{
		ApplyTeleopInput(ControlLinear, ControlAngular, DeltaTime, 1.0f);
	}
	if (!bEnableKeyPolling)
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

	ApplyTeleopInput(FVector(Fwd, Strafe, Up), FRotator(Pitch, Yaw, Roll), DeltaTime, Scale);

	// Right-mouse drag adds yaw (about up = local X) + pitch (Y), in degrees this frame.
	const bool bAllowMouse = bEnableMouseRotation
		&& (!bRequireRightMouseButton || PC->IsInputKeyDown(EKeys::RightMouseButton));
	if (bAllowMouse)
	{
		float MdX = 0.f, MdY = 0.f;
		PC->GetInputMouseDelta(MdX, MdY);
		const float PitchInv = bInvertMouseY ? 1.f : -1.f;
		FRotator RotDelta = FRotator::ZeroRotator;
		RotDelta.Roll = MdX * MouseYawDegreesPerPixel * Scale * YawSign;
		RotDelta.Pitch = PitchInv * MdY * MousePitchDegreesPerPixel * Scale * PitchSign;
		if (!RotDelta.IsNearlyZero())
		{
			Controller->MoveTargetBy(FVector::ZeroVector, RotDelta, bLocalFrame);
		}
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

void URammsMjArmTeleopComponent::ApplyTeleopInput(FVector LinearFSU, FRotator AngularYPR, float DeltaTime, float Scale)
{
	if (!ResolveController())
	{
		return;
	}
	// Translation in the gripper frame: forward = local Z, strafe = local Y, up = local X.
	FVector Lin;
	Lin.X = LinearFSU.Z * UpSign;
	Lin.Y = LinearFSU.Y * StrafeSign;
	Lin.Z = LinearFSU.X * ForwardSign;
	const FVector LinDelta = Lin * (LinearSpeed * Scale * DeltaTime);

	// MoveTargetBy reads FRotator as rotations about local X(Roll)/Y(Pitch)/Z(Yaw). The gripper's
	// up axis is X, so: yaw -> .Roll, pitch -> .Pitch, roll (about forward Z) -> .Yaw.
	const float ARate = AngularSpeed * Scale * DeltaTime;
	FRotator RotDelta = FRotator::ZeroRotator;
	RotDelta.Roll = AngularYPR.Yaw * YawSign * ARate;	   // yaw about up (X)
	RotDelta.Pitch = AngularYPR.Pitch * PitchSign * ARate; // pitch about Y
	RotDelta.Yaw = AngularYPR.Roll * RollSign * ARate;	   // roll about forward (Z)

	if (!LinDelta.IsNearlyZero() || !RotDelta.IsNearlyZero())
	{
		Controller->MoveTargetBy(LinDelta, RotDelta, bLocalFrame);
	}
}

// --- control surface -----------------------------------------------------------

namespace
{
	const FName ArmForward(TEXT("arm.forward"));
	const FName ArmStrafe(TEXT("arm.strafe"));
	const FName ArmUp(TEXT("arm.up"));
	const FName ArmYaw(TEXT("arm.yaw"));
	const FName ArmPitch(TEXT("arm.pitch"));
	const FName ArmRoll(TEXT("arm.roll"));
	const FName ArmResync(TEXT("arm.resync"));
	const FName GripperOpen(TEXT("gripper.open"));
	const FName GripperClose(TEXT("gripper.close"));
	const FName GripperToggle(TEXT("gripper.toggle"));
	const FName GripperClosed(TEXT("gripper.closed"));
} // namespace

void URammsMjArmTeleopComponent::DescribeControls(FRammsControlSurface& OutSurface) const
{
	if (!Controller)
	{
		return;
	}
	auto Rate = [&OutSurface](FName Id, const TCHAR* Name, FName Paired, int32 Order) {
		FRammsControlAxis Axis;
		Axis.Id = Id;
		Axis.Group = FName("Arm");
		Axis.DisplayName = FText::FromString(Name);
		Axis.Kind = ERammsControlKind::Continuous;
		Axis.Units = ERammsControlUnits::Normalized;
		Axis.Range = FVector2D(-1.0, 1.0);
		Axis.PairedAxis = Paired;
		Axis.Order = Order;
		OutSurface.Add(Axis);
	};
	Rate(ArmForward, TEXT("Forward"), ArmStrafe, 0);
	Rate(ArmStrafe, TEXT("Strafe"), ArmForward, 1);
	Rate(ArmUp, TEXT("Up"), NAME_None, 2);
	Rate(ArmPitch, TEXT("Pitch"), ArmYaw, 3); // paired: lower Order = the joystick's vertical axis
	Rate(ArmYaw, TEXT("Yaw"), ArmPitch, 4);
	Rate(ArmRoll, TEXT("Roll"), NAME_None, 5);

	auto Action = [&OutSurface](FName Id, FName Group, const TCHAR* Name, int32 Order) {
		FRammsControlAxis Axis;
		Axis.Id = Id;
		Axis.Group = Group;
		Axis.DisplayName = FText::FromString(Name);
		Axis.Kind = ERammsControlKind::Action;
		Axis.Units = ERammsControlUnits::None;
		Axis.Order = Order;
		OutSurface.Add(Axis);
	};
	Action(ArmResync, FName("Arm"), TEXT("Resync target"), 6);
	Action(GripperOpen, FName("Gripper"), TEXT("Open"), 0);
	Action(GripperClose, FName("Gripper"), TEXT("Close"), 1);
	Action(GripperToggle, FName("Gripper"), TEXT("Toggle"), 2);

	// Readback-only state (0 = open, 1 = closed) for panels / status.
	FRammsControlAxis Closed;
	Closed.Id = GripperClosed;
	Closed.Group = FName("Gripper");
	Closed.DisplayName = FText::FromString(TEXT("Closed"));
	Closed.Kind = ERammsControlKind::Position;
	Closed.Units = ERammsControlUnits::Normalized;
	Closed.Range = FVector2D(0.0, 1.0);
	Closed.bReadback = true;
	Closed.bReadOnly = true; // state only: change it through the actions
	Closed.Order = 3;
	OutSurface.Add(Closed);
}

bool URammsMjArmTeleopComponent::ApplyControl(FName Id, float Value)
{
	if (!bTeleopEnabled || !ResolveController())
	{
		return false;
	}
	const float V = FMath::Clamp(Value, -1.0f, 1.0f);
	if (Id == ArmForward)
	{
		ControlLinear.X = V;
	}
	else if (Id == ArmStrafe)
	{
		ControlLinear.Y = V;
	}
	else if (Id == ArmUp)
	{
		ControlLinear.Z = V;
	}
	else if (Id == ArmYaw)
	{
		ControlAngular.Yaw = V;
	}
	else if (Id == ArmPitch)
	{
		ControlAngular.Pitch = V;
	}
	else if (Id == ArmRoll)
	{
		ControlAngular.Roll = V;
	}
	else
	{
		return false; // gripper.closed is readback-only: use the gripper actions
	}
	return true;
}

bool URammsMjArmTeleopComponent::TriggerControl(FName Id)
{
	if (!bTeleopEnabled || !ResolveController())
	{
		return false; // the master enable covers both advertised paths
	}
	if (Id == ArmResync)
	{
		Controller->ResyncTargetToCurrentPose();
	}
	else if (Id == GripperOpen)
	{
		Controller->OpenGripper();
		bGripClosed = false;
	}
	else if (Id == GripperClose)
	{
		Controller->CloseGripper();
		bGripClosed = true;
	}
	else if (Id == GripperToggle)
	{
		bGripClosed = !bGripClosed;
		Controller->SetGrip(bGripClosed ? 1.0f : 0.0f);
	}
	else
	{
		return false;
	}
	return true;
}

bool URammsMjArmTeleopComponent::ReleaseControl(FName Id)
{
	// Rate axes spring to zero by clearing the cached rate directly — even
	// while teleop is disabled, so no stale rate resumes motion when it is
	// re-enabled. The arm holds wherever the target is. gripper.closed is
	// state: releasing it is a no-op.
	if (Id == ArmForward)
	{
		ControlLinear.X = 0.0f;
	}
	else if (Id == ArmStrafe)
	{
		ControlLinear.Y = 0.0f;
	}
	else if (Id == ArmUp)
	{
		ControlLinear.Z = 0.0f;
	}
	else if (Id == ArmYaw)
	{
		ControlAngular.Yaw = 0.0f;
	}
	else if (Id == ArmPitch)
	{
		ControlAngular.Pitch = 0.0f;
	}
	else if (Id == ArmRoll)
	{
		ControlAngular.Roll = 0.0f;
	}
	else if (Id != GripperClosed)
	{
		return false;
	}
	return true;
}

bool URammsMjArmTeleopComponent::ReadControl(FName Id, float& OutValue) const
{
	if (Id == GripperClosed)
	{
		OutValue = bGripClosed ? 1.0f : 0.0f;
		return true;
	}
	if (Id == ArmForward || Id == ArmStrafe || Id == ArmUp)
	{
		OutValue = Id == ArmForward ? ControlLinear.X : (Id == ArmStrafe ? ControlLinear.Y : ControlLinear.Z);
		return true;
	}
	if (Id == ArmYaw || Id == ArmPitch || Id == ArmRoll)
	{
		OutValue = Id == ArmYaw ? ControlAngular.Yaw : (Id == ArmPitch ? ControlAngular.Pitch : ControlAngular.Roll);
		return true;
	}
	return false;
}
