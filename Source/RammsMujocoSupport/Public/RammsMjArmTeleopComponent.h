// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "InputCoreTypes.h"
#include "RammsMjArmTeleopComponent.generated.h"

class URammsMjEndEffectorController;

/**
 * Mouse + keyboard teleoperation for a MuJoCo arm driven by a URammsMjEndEffectorController.
 *
 * Each tick it reads the local player's input and nudges the controller's end-effector target by
 * a small per-frame delta (so the IK tracks smoothly), and maps keys to the gripper. Add this to
 * the same AMjArticulation actor that has the EE controller.
 *
 * All keys are configurable (see the "MjTeleop|Keys" / "MjTeleop|Gripper" categories). Defaults avoid
 * WASD so they don't clash with base driving:
 *   I/K = forward/back, J/L = strafe left/right, U/O = up/down, M/. = roll, arrows = yaw/pitch;
 *   right-mouse drag = yaw/pitch; [ / ] = open/close gripper, G = toggle, R = re-sync target to EE.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSMUJOCOSUPPORT_API URammsMjArmTeleopComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsMjArmTeleopComponent();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Master enable. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop")
	bool bTeleopEnabled = true;

	/** Optional controller component name if the actor has more than one. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop")
	FName ControllerComponentName = NAME_None;

	/** Move the EE target in its own frame (true) or the MuJoCo world frame (false). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Motion")
	bool bLocalFrame = true;

	/** Translation speed (metres/second). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Motion", meta = (ClampMin = "0.0"))
	float LinearSpeed = 0.15f;

	/** Rotation speed (degrees/second) for arrow / roll keys. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Motion", meta = (ClampMin = "0.0"))
	float AngularSpeed = 45.0f;

	/** Multiplier while a Shift key is held. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Motion", meta = (ClampMin = "0.1"))
	float FastMultiplier = 3.0f;

	/** Multiplier while a Ctrl key is held. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Motion", meta = (ClampMin = "0.01"))
	float SlowMultiplier = 0.25f;

	// Per-axis sign flips — tweak live in the editor to fix any inverted direction without a rebuild.
	// Mapping (gripper frame): forward=local Z, strafe=local Y, up=local X;
	// yaw=rotate about up (X), pitch=about Y, roll=about forward (Z).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Axis Signs")
	float ForwardSign = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Axis Signs")
	float StrafeSign = -1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Axis Signs")
	float UpSign = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Axis Signs")
	float YawSign = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Axis Signs")
	float PitchSign = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Axis Signs")
	float RollSign = 1.0f;

	// --- Keyboard bindings (configurable). Defaults deliberately avoid WASD so the arm and the base
	// can be driven at the same time. Each action is a +/- key pair in the gripper frame. ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey ForwardKey = EKeys::I;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey BackwardKey = EKeys::K;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey StrafeLeftKey = EKeys::J;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey StrafeRightKey = EKeys::L;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey UpKey = EKeys::U;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey DownKey = EKeys::O;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey RollLeftKey = EKeys::M;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey RollRightKey = EKeys::Period;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey YawLeftKey = EKeys::Left;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey YawRightKey = EKeys::Right;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey PitchUpKey = EKeys::Up;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Keys")
	FKey PitchDownKey = EKeys::Down;

	/** Enable right-mouse-drag pitch/yaw. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Mouse")
	bool bEnableMouseRotation = true;

	/** Require the right mouse button held for mouse rotation. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Mouse")
	bool bRequireRightMouseButton = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Mouse")
	bool bInvertMouseY = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Mouse", meta = (ClampMin = "0.001"))
	float MouseYawDegreesPerPixel = 0.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Mouse", meta = (ClampMin = "0.001"))
	float MousePitchDegreesPerPixel = 0.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Gripper")
	FKey OpenGripperKey = EKeys::LeftBracket;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Gripper")
	FKey CloseGripperKey = EKeys::RightBracket;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop|Gripper")
	FKey ToggleGripperKey = EKeys::G;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop")
	FKey ResyncTargetKey = EKeys::R;

	UFUNCTION(BlueprintPure, Category = "MjTeleop")
	URammsMjEndEffectorController* GetController() const { return Controller; }

private:
	URammsMjEndEffectorController* ResolveController();

	UPROPERTY(Transient)
	URammsMjEndEffectorController* Controller = nullptr;

	bool bGripClosed = false;
	bool bLoggedMissing = false;
};
