// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Controllers/MjArticulationController.h"
#include "RammsMjEndEffectorController.generated.h"

class UMjNodeComponent;

/**
 * End-effector IK controller for a URLab MuJoCo articulation (e.g. the Kinova Gen3 + 2F-85).
 *
 * Runs damped-least-squares IK on the physics thread: drives the arm's position actuators so a
 * chosen end-effector site tracks a target pose, and writes the gripper actuator from a 0..1 grip
 * value. Set the target from Blueprint / teleop with SetTargetPosition / MoveTargetBy; the target
 * auto-initializes to the current EE pose on the first step so it doesn't snap.
 *
 * All target poses are in MuJoCo world coordinates (metres). Add this component to the imported
 * AMjArticulation Blueprint. On an arm composed onto a mobile base in one articulation it drives only
 * the joints between the base and the end effector; the base's actuators keep their own commands.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSMUJOCOSUPPORT_API URammsMjEndEffectorController : public UMjArticulationController
{
	GENERATED_BODY()

public:
	URammsMjEndEffectorController();

	/** End-effector site to control (matched by suffix against the compiled, prefixed site names). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK")
	FString EndEffectorSiteName = TEXT("pinch");

	/** Gripper actuator identified when its (prefix-stripped) name ends with this. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK")
	FString GripperActuatorSuffix = TEXT("fingers_actuator");

	/** The arm's root body, matched by suffix like the site: the IK drives only the joints between it and the
	 *  end-effector site. On an arm composed onto a mobile base that keeps the base's actuators (its wheels, a
	 *  lift the arm rides on) out of the IK. Empty, or no such body above the site: every joint up to the
	 *  articulation's root. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK")
	FString ArmRootBodyName = TEXT("base_link");

	/** Damped-least-squares damping (larger = more stable, slower near singularities). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK", meta = (ClampMin = "0.001"))
	float DampingLambda = 0.08f;

	/** Fraction of the IK joint delta applied per physics step. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK", meta = (ClampMin = "0.01", ClampMax = "1.0"))
	float StepGain = 0.5f;

	/** Track target orientation as well as position (false = position-only, free wrist). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK")
	bool bControlOrientation = true;

	/** On bind, set the arm joints to this home pose (radians, one per arm joint in order) so the
	 *  arm doesn't start in the straight-up zero configuration. Leave empty to keep the model default. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK|Home")
	TArray<float> HomeArmAngles;

	/** Apply HomeArmAngles to the arm joints when the model binds. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK|Home")
	bool bApplyHomeOnBind = true;

	/** Joint speed while stowing (rad/s): Stow() takes each arm joint to HomeArmAngles at this rate. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK|Home", meta = (ClampMin = "0.01"))
	float StowSpeed = 0.8f;

	/** Every arm joint within this of HomeArmAngles (rad) reads as stowed: room for the wrist's servos to sag
	 *  under gravity (the Gen3's sit about 0.05 rad off). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK|Home", meta = (ClampMin = "0.001"))
	float StowToleranceRad = 0.1f;

	/** Keep the EE target fixed relative to the arm's root body (the mocap base_link, or the chassis of
	 *  a base the arm is composed onto) instead of a fixed MuJoCo world pose. Lets the arm ride a moving
	 *  base (e.g. the RAMMP chair) without the IK fighting to hold a world point. When true, target
	 *  poses are expressed in the base frame. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK")
	bool bTrackBaseFrame = true;

	/** Dynamic tracking base: name of the mocap "target" body that base_link is welded to. On startup
	 *  the free base is snapped onto this target so the stiff weld starts satisfied. Empty = disabled. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK|Tracking Base")
	FString BaseTargetBodyName = TEXT("base_target");

	/** Snap the free base onto the mocap target for this many physics steps at startup (covers the
	 *  delay while the target's world pose propagates from the Chaos mount). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK|Tracking Base", meta = (ClampMin = "0"))
	int32 BaseInitSteps = 12;

	/** Own base_target's mocap and advance it smoothly toward the Chaos mount every physics substep,
	 *  instead of URLab's once-per-UE-frame teleport. This removes the stair-step the weld would ring
	 *  on, so the weld can stay stiff (no springiness) and still track smoothly. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK|Tracking Base")
	bool bSmoothBaseTarget = true;

	/** Per-substep smoothing factor toward the latest mount pose (higher = tighter tracking, less lag,
	 *  but less filtering of the per-frame steps). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EE IK|Tracking Base", meta = (ClampMin = "0.02", ClampMax = "1.0"))
	float BaseSmoothingAlpha = 0.25f;

	// --- Target API (metres; thread-safe). Frame is the arm base when bTrackBaseFrame, else MuJoCo world. ---

	/** Set the EE target position (keeps current target orientation). */
	UFUNCTION(BlueprintCallable, Category = "EE IK")
	void SetTargetPosition(FVector PosMeters);

	/** Nudge the target by a position delta (and optional rotation about world axes, degrees). */
	UFUNCTION(BlueprintCallable, Category = "EE IK")
	void MoveTargetBy(FVector DeltaMeters, FRotator DeltaRotDeg, bool bLocalFrame);

	/** Set grip: 0 = open, 1 = closed (mapped to the gripper actuator's ctrl range). */
	UFUNCTION(BlueprintCallable, Category = "EE IK")
	void SetGrip(float Value01);

	UFUNCTION(BlueprintCallable, Category = "EE IK")
	void OpenGripper() { SetGrip(0.0f); }

	UFUNCTION(BlueprintCallable, Category = "EE IK")
	void CloseGripper() { SetGrip(1.0f); }

	/** Re-sync the target to the current EE pose on the next physics step (e.g. after teleporting). */
	UFUNCTION(BlueprintCallable, Category = "EE IK")
	void ResyncTargetToCurrentPose();

	UFUNCTION(BlueprintPure, Category = "EE IK")
	bool IsTargetInitialized() const;

	/** Take the arm to HomeArmAngles joint by joint (at StowSpeed, not through the IK, so it folds the way the home
	 *  pose does), then hold its end effector there: the target re-seeds from the stowed pose. Ignored without a
	 *  home pose for every arm joint. */
	UFUNCTION(BlueprintCallable, Category = "EE IK|Home")
	void Stow();

	/** Every arm joint within StowToleranceRad of HomeArmAngles (false without a home pose). */
	UFUNCTION(BlueprintPure, Category = "EE IK|Home")
	bool IsStowed() const;

	// Game-thread tick: samples the Chaos mount pose for the physics thread to smooth toward.
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// --- UMjArticulationController interface ---
	virtual void Bind(mjModel* m, mjData* d, const TMap<int32, UMjNodeComponent*>& ActuatorIdMap) override;
	virtual void ComputeAndApply(mjModel* m, mjData* d, uint8 Source) override;
	virtual FString GetKindName() const override { return TEXT("ee_ik"); }

private:
	/** Articulation reset: drop the target and grip so they re-seed from the reset pose. */
	UFUNCTION()
	void HandleSimulationReset();

	// Resolved at Bind:
	int32 EeSiteId = -1;
	int32 BaseBodyId = -1;       // arm root body (direct child of world); target frame when bTrackBaseFrame
	int32 FreeBaseQposAddr = -1; // qpos address of base_link's free joint (dynamic tracking base)
	int32 FreeBaseDofAddr = -1;  // qvel address of that free joint
	int32 BaseTargetBodyId = -1; // mocap target the base is welded to
	int32 BaseInitStepsLeft = 0;

	// Physics-substep smoothing of base_target's mocap toward the Chaos mount (owns base_target).
	int32 BaseTargetMocapId = -1;
	TWeakObjectPtr<class UMjBody> BaseTargetBodyComp;   // read on the game thread for the mount pose
	mutable FCriticalSection BaseGoalMutex;
	double GoalPos[3] = {0, 0, 0};      // latest mount pose in MuJoCo coords (game thread writes)
	double GoalQuat[4] = {1, 0, 0, 0};
	bool bGoalValid = false;
	double SmoothPos[3] = {0, 0, 0};    // running smoothed mocap pose (physics thread)
	double SmoothQuat[4] = {1, 0, 0, 0};
	bool bSmoothInit = false;
	TArray<int32> ArmActIds;     // arm position-actuator MuJoCo ids (order = column order)
	TArray<double>				  ArmCmd;		 // what the arm actuators were last commanded (held when a step cannot solve)
	TArray<int32> ArmQposAddr;   // qpos address per arm joint
	TArray<int32> ArmDofAddr;    // qvel/dof address per arm joint (Jacobian column)
	int32 GripActId = -1;
	FVector2D GripCtrlRange = FVector2D(0.0f, 255.0f);

	// Scratch Jacobian buffers (sized in Bind, written each step).
	TArray<double> JacP;
	TArray<double> JacR;

	// Target, guarded by mutex (game thread writes, physics thread reads).
	mutable FCriticalSection TargetMutex;
	double TgtPos[3] = {0, 0, 0};
	double TgtQuat[4] = {1, 0, 0, 0};  // MuJoCo wxyz
	double GripCtrl = 0.0;
	bool bTargetInit = false;
	bool					 bStowRequested = false; // Stow() asked; the physics step takes it up
	bool					 bStowed = false;		 // the arm at its home pose, as of the last physics step

	// Stowing (physics thread): the arm joints driven to HomeArmAngles in joint space, the IK paused.
	bool bStowing = false;

	// Apply HomeArmAngles on the first physics step (physics thread), since the engine's post-bind
	// mj_resetData wipes any qpos we set in Bind(). Set in Bind, consumed once in ComputeAndApply.
	bool bHomePending = false;
};
