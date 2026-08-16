// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsBackendSwitchComponent.generated.h"

UENUM(BlueprintType)
enum class ERammsPhysicsBackend : uint8
{
	/** URLab MuJoCo stepping (or Newton when a RammsNewtonSolverComponent
	 *  is active in the map — Newton is a solver swap behind the same data). */
	MuJoCo,
	/** UE-native Chaos: the generated rig (collision + constraints) simulates;
	 *  every Mj* component is destroyed on BeginPlay so URLab ignores the
	 *  actor. Use in maps without an AMjManager, or for background props. */
	Chaos,
};

/**
 * Backend selector for actors imported from MJCF with a generated Chaos rig
 * (see the "Generate Chaos Rig" editor action). One actor, one description,
 * either physics backend — chosen per instance before BeginPlay.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSMUJOCOSUPPORT_API URammsBackendSwitchComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsBackendSwitchComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Backend")
	ERammsPhysicsBackend Backend = ERammsPhysicsBackend::MuJoCo;

	/** Keep the Chaos rig bodies always awake. A robot that dozes off stops
	 *  responding to joint commands until something wakes it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Backend")
	bool bNeverSleep = true;

	/** Component names of the rig's simulated bodies (filled by the generator). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> ChaosBodyComponents;

	/** Component names of the generated constraints (filled by the generator). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> ChaosConstraintComponents;

	/** Parallel to ChaosConstraintComponents: the constraint's authoritative
	 *  frame LOCAL TO its child body component. Editor-side construction
	 *  reruns (e.g. flipping Backend in the Details panel) can displace
	 *  constraint components; runtime re-derives world frames from these
	 *  before initializing, so displaced editor transforms cannot matter. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FTransform> ConstraintLocalFrames;

	/** Parallel to ChaosConstraintComponents: the child body component the
	 *  local frame is relative to. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> ConstraintChildBodies;

	/** Parallel to ChaosBodyComponents: MJCF inertial mass in kg (0 = unknown,
	 *  runtime falls back to a small default). URLab viz meshes carry no usable
	 *  auto-mass, and near-massless bodies get pushed through the floor. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<float> BodyMasses;

	/** Parallel arrays: extra visual pieces of multi-material bodies get
	 *  reattached to their simulated body piece in Chaos mode. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> PieceBodies;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> PieceMeshes;

	/** Parallel arrays: actuated joints and their generated constraints. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> DriveJoints;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> DriveConstraints;

	/** Wheels whose full-strength drive has been pushed to the live
	 *  constraint (runtime SetAngularDriveParams doesn't propagate — the
	 *  first command re-inits the constraint once). */
	TSet<FName> ActivatedDrives;

	/** Virtual couplers: enforce Follower twist = Ratio * Leader twist via
	 *  an orientation drive on the follower each physics tick. Bypasses
	 *  multi-pin closure loops that Chaos's iterative solver leaks force
	 *  through (the front-caster 4-bar: rod moved the crank, the swing arm
	 *  never followed). Parallel arrays, filled by the generator. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> CouplerLeaders;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> CouplerFollowers;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<float> CouplerRatios;

	/** Slide-leader rest extension (cm), captured on first coupler tick. */
	TArray<float> CouplerLeaderRest;
	TArray<uint8> CouplerLeaderRestSet;

	FTimerHandle CouplerTimer;
	void TickCouplers();

	/** Keep-awake tick handle (bNeverSleep). */
	FTimerHandle KeepAwakeTimer;

	/** Slew-limited command state: joint -> (current, commanded). Position
	 *  drives ease toward the commanded target (5 cm/s / 60 deg/s) so a
	 *  slider step never becomes a force impulse. */
	TMap<FName, FVector2D> SlewTargets;
	FTimerHandle SlewTimer;
	void TickSlew();
	void ApplyJointTarget(FName Joint, float Value);

	/** Wake every simulated rig body. */
	void WakeRigBodies();

	/** Parallel to DriveJoints: true when the joint is a position servo
	 *  (MJCF `position` actuator) — commands set a position/orientation
	 *  target; false = velocity motor (wheels). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<bool> DriveIsPosition;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<bool> DriveIsLinear;

	/** Parallel to DriveJoints: actuator ctrlrange (native MJCF units) for
	 *  UI sliders. Both zero = unknown (panel falls back to a sane span). */
	/** Parallel to DriveJoints: command multiplier (tendon wrap coef sign
	 *  for multi-joint drives; 1 otherwise). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<float> DriveScale;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<float> DriveCtrlMin;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<float> DriveCtrlMax;

	/**
	 * Unified joint command, valid on either backend.
	 * - Servo (slide) joints: Value = target extension in METRES.
	 * - Motor (hinge) joints: MuJoCo mode = ctrl in [-1, 1] (torque
	 *   fraction); Chaos mode = target angular velocity in rad/s. The wheel
	 *   semantics diverge in v1 — unify when the controller seam lands.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Backend")
	void SetJointCommand(FName Joint, float Value);

protected:
	virtual void InitializeComponent() override;
	virtual void BeginPlay() override;

private:
	void ApplyChaos();
	void ApplyMuJoCo();
};
