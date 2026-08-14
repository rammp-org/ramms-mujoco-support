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

	/** Component names of the rig's simulated bodies (filled by the generator). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> ChaosBodyComponents;

	/** Component names of the generated constraints (filled by the generator). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<FName> ChaosConstraintComponents;

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

	/** Parallel to DriveJoints: true when the joint is a position servo
	 *  (MJCF `position` actuator) — commands set a position/orientation
	 *  target; false = velocity motor (wheels). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<bool> DriveIsPosition;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Backend")
	TArray<bool> DriveIsLinear;

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
