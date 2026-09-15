// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "InputCoreTypes.h"
#include "RammsControlContributor.h"
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
 *
 * It is also the arm's control-surface contributor, with the same ids as the
 * Chaos arm's URammsEndEffectorTeleopComponent: rate axes arm.forward /
 * arm.strafe / arm.up / arm.yaw / arm.pitch / arm.roll (normalized, integrated
 * per tick at LinearSpeed / AngularSpeed), arm.resync, gripper.open /
 * gripper.close / gripper.toggle and the gripper.closed readback.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSMUJOCOSUPPORT_API URammsMjArmTeleopComponent : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	URammsMjArmTeleopComponent();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Master enable (control-surface and key paths). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop")
	bool bTeleopEnabled = true;

	/** Legacy: poll the keys / mouse below each tick. Off by default — the
	 *  Enhanced Input map drives arm.* / gripper.* through the control surface. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjTeleop")
	bool bEnableKeyPolling = false;

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

	/** Nudge the target by normalized semantic rates (forward / strafe / up,
	 *  yaw / pitch / roll, each -1..1) for DeltaTime at Scale x the teleop
	 *  speeds — the mapping the key polling and the control surface share. */
	UFUNCTION(BlueprintCallable, Category = "MjTeleop")
	void ApplyTeleopInput(FVector LinearFSU, FRotator AngularYPR, float DeltaTime, float Scale = 1.0f);

	// --- IRammsControlContributor: "arm.*", "gripper.*" ------------------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReleaseControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual int32 GetControlOrder() const override { return 30; }

private:
	URammsMjEndEffectorController* ResolveController();

	/** Rate input from the control surface, applied in TickComponent. */
	FVector	 ControlLinear = FVector::ZeroVector;  // X forward, Y strafe, Z up
	FRotator ControlAngular = FRotator::ZeroRotator; // Yaw / Pitch / Roll

	UPROPERTY(Transient)
	URammsMjEndEffectorController* Controller = nullptr;

	bool bGripClosed = false;
	bool bLoggedMissing = false;
};
