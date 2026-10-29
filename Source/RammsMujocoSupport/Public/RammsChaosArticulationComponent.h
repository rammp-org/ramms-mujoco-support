// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/EngineTypes.h"
#include "HAL/CriticalSection.h"
#include "RammsChaosArticulationComponent.generated.h"

class AMjArticulation;
class UBodySetup;
class UPhysicalMaterial;
class USceneComponent;
class FRammsChaosArticulationCallback;
struct FConstraintInstance;

/** How the parts of one Chaos-built articulation collide with each other. */
UENUM(BlueprintType)
enum class ERammsChaosSelfCollision : uint8
{
	/** MuJoCo's rules: parts joined by a joint never collide, nor do <exclude>d pairs or pairs whose
	 *  contype/conaffinity masks cannot meet. Everything else does. */
	MuJoCo,
	/** No part collides with another part of the same articulation (the RAMMS RobotSelf convention:
	 *  every part goes on SelfCollisionChannel and ignores it, so two such articulations ignore each
	 *  other too). For models whose parts overlap at rest. */
	Off,
};

/**
 * One Chaos body of a Chaos-built articulation: a primitive with no render proxy whose collision is
 * built at runtime from the MuJoCo body's (and its welded children's) collision geoms.
 */
UCLASS(NotBlueprintable, ClassGroup = (Ramms))
class RAMMSMUJOCOSUPPORT_API URammsChaosBodyComponent : public UPrimitiveComponent
{
	GENERATED_BODY()

public:
	URammsChaosBodyComponent();

	/** The runtime-built collision: spheres, capsules, boxes and convex hulls in this body's frame. */
	UPROPERTY(Transient)
	TObjectPtr<UBodySetup> ShapeSetup;

	/** The MuJoCo body this is (the root of its welded group). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Chaos Articulation")
	FName MuJoCoBody;

	virtual UBodySetup*			  GetBodySetup() override { return ShapeSetup; }
	virtual FBoxSphereBounds	  CalcBounds(const FTransform& LocalToWorld) const override;
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override { return nullptr; }
};

/**
 * Simulates a URLab articulation (an AMjArticulation and its MJCF component tree) with Chaos instead
 * of MuJoCo, in the world's own physics scene, so it interacts with every other Chaos object there.
 *
 * At BeginPlay it compiles the articulation's spec with MuJoCo (used only as the model compiler: every
 * default resolved, every mass, inertia, convex mesh, joint axis and actuator gain worked out) and
 * builds from the compiled model:
 *
 *  - a URammsChaosBodyComponent per body that has a joint (a body without one is welded into its
 *    parent's), with the MJCF's collision geoms, mass, centre of mass and inertia, and a physical
 *    material carrying the geoms' sliding friction (combined by max, as MuJoCo does);
 *  - a constraint per hinge, slide or ball joint, with its range (asymmetric ranges are centred by the
 *    constraint frames), and per <connect> equality (which closes kinematic loops);
 *  - a position + velocity drive per position servo (<position>, or a <general> with an affine bias),
 *    in force mode with the MJCF's kp / kv, and the force limit scaled by ServoForceLimitScale;
 *  - a physics-rate callback that applies torque motors, joint damping, stiffness and friction loss,
 *    which Chaos constraint drives do not carry.
 *
 * The articulation's body components (and the geom visuals under them) follow the Chaos bodies every
 * frame, so the robot looks the same as it does under MuJoCo. Controls go through SetControl with the
 * MJCF actuator's own ctrl units, so a keyframe's ctrl vector drives the same stance on either engine.
 *
 * The level must not also hold an AAMjManager simulating this articulation (it would compile it into
 * MuJoCo too and move its bodies); this is reported at BeginPlay.
 *
 * Not carried: soft contacts and soft limits (Chaos limits and contacts are hard), torsional / rolling
 * friction, joint armature, spatial tendons, joint equalities, multi-joint bodies (only the first
 * joint is kept), and geoms that exist only in the compiled model (no visuals follow them).
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSMUJOCOSUPPORT_API URammsChaosArticulationComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsChaosArticulationComponent();

	/** Build and simulate with Chaos at BeginPlay. Off leaves the articulation to MuJoCo. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Chaos Articulation")
	bool bSimulateWithChaos = true;

	/** Start from this MJCF keyframe (its qpos, and its ctrl for the actuators) instead of qpos0. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Chaos Articulation")
	FString InitialKeyframe;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Chaos Articulation")
	ERammsChaosSelfCollision SelfCollision = ERammsChaosSelfCollision::MuJoCo;

	/** The object channel the parts use when SelfCollision is Off (RAMMS' RobotSelf). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Chaos Articulation", meta = (EditCondition = "SelfCollision == ERammsChaosSelfCollision::Off"))
	TEnumAsByte<ECollisionChannel> SelfCollisionChannel = ECC_GameTraceChannel2;

	/**
	 * Multiplier on a servo's MJCF force range when it becomes a drive's force limit. Chaos' drive
	 * limit does not carry over 1:1: a lift-drive base's stance servos, which MuJoCo holds with 10-30
	 * N.m against a 207 N.m range, did not move at 1x and did at 10x.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Chaos Articulation", meta = (ClampMin = "0.1"))
	float ServoForceLimitScale = 10.0f;

	/** Sides of the convex polygon a cylinder geom (a wheel) becomes; Chaos has no cylinder. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Chaos Articulation", meta = (ClampMin = "8", ClampMax = "128"))
	int32 CylinderSides = 48;

	/** Floors on a body's mass (kg) and principal inertia (kg.cm^2): Chaos' iterative solver does not
	 *  cope with the gram-scale links some MJCFs carry next to a heavy chassis. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Chaos Articulation", meta = (ClampMin = "0"))
	float MinBodyMass = 0.02f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Chaos Articulation", meta = (ClampMin = "0"))
	float MinBodyInertia = 0.5f;

	/** Restitution of every part (MuJoCo's contacts do not bounce). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Chaos Articulation", meta = (ClampMin = "0", ClampMax = "1"))
	float Restitution = 0.0f;

	/**
	 * Switch an articulation to Chaos at runtime: adds this component to it (or returns the one it has)
	 * and builds it. Null when Articulation is not an AMjArticulation. Check IsBuilt / GetBuildError.
	 */
	UFUNCTION(BlueprintCallable, Category = "Chaos Articulation", meta = (DefaultToSelf = "Articulation"))
	static URammsChaosArticulationComponent* SwitchToChaos(AActor* Articulation, const FString& Keyframe);

	/** Set an actuator's control, in its MJCF ctrl units (clamped to ctrlrange). False: no such actuator. */
	UFUNCTION(BlueprintCallable, Category = "Chaos Articulation")
	bool SetControl(FName Actuator, float Value);

	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	float GetControl(FName Actuator) const;

	/** Apply a generalized force to a joint until changed: N.m on a hinge, N on a slide. */
	UFUNCTION(BlueprintCallable, Category = "Chaos Articulation")
	bool SetJointForce(FName Joint, float Force);

	/** A joint's coordinate (rad, or m for a slide, as MuJoCo reports it), measured from the bodies. An
	 *  unlimited hinge (a wheel) keeps counting past +-pi, as MuJoCo's qpos does. */
	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	float GetJointPosition(FName Joint) const;

	/** A joint's rate (rad/s or m/s). */
	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	float GetJointVelocity(FName Joint) const;

	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	TArray<FName> GetJointNames() const;

	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	TArray<FName> GetActuatorNames() const;

	/** Apply every actuator ctrl a keyframe stores. False: no such keyframe, or it stores none. */
	UFUNCTION(BlueprintCallable, Category = "Chaos Articulation")
	bool ApplyKeyframeControls(const FString& Keyframe);

	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	TArray<FString> GetKeyframeNames() const;

	/** A keyframe's actuator ctrl vector, in GetActuatorNames order (empty: no such keyframe). */
	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	TArray<float> GetKeyframeControls(const FString& Keyframe) const;

	/** An actuator's ctrlrange; false when it has none (or there is no such actuator). */
	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	bool GetControlRange(FName Actuator, float& OutMin, float& OutMax) const;

	/** Whether a joint is a slide (its coordinate in m, its force in N); false for a hinge or ball. */
	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	bool IsSlide(FName Joint) const;

	/** Whether any actuator moves this joint. */
	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	bool IsActuated(FName Joint) const;

	/** Whether an actuator is a torque motor (false: a position or velocity servo, or unknown). */
	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	bool IsMotor(FName Actuator) const;

	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	bool IsBuilt() const { return bBuilt; }

	/** The Chaos body a MuJoCo body moves with (its welded group's root), or null. */
	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	URammsChaosBodyComponent* GetBodyComponent(FName MuJoCoBody) const;

	/** Every Chaos body this built (the free-floating base, if there is one, first among the jointed). */
	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	TArray<URammsChaosBodyComponent*> GetBodyComponents() const;

	/** Why the last build failed, empty when it did not. */
	UFUNCTION(BlueprintPure, Category = "Chaos Articulation")
	FString GetBuildError() const { return BuildError; }

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	enum class EJointKind : uint8
	{
		Hinge,
		Slide,
		Ball
	};
	enum class EActuatorKind : uint8
	{
		PositionServo,
		VelocityServo,
		Motor
	};

	/** One MuJoCo body: which Chaos body carries it and where in that body's frame it sits. */
	struct FMjBodyInfo
	{
		FName							Name;
		int32							Root = INDEX_NONE; // index into Roots
		FTransform						InRoot;			   // this body's frame in its root's frame (Unreal units)
		TWeakObjectPtr<USceneComponent> Visual;			   // the URLab body component that follows it
	};

	struct FJoint
	{
		FName	   Name;
		EJointKind Kind = EJointKind::Hinge;
		int32	   ParentRoot = INDEX_NONE;
		int32	   ChildRoot = INDEX_NONE;
		FVector	   AxisInParent = FVector::XAxisVector;									// unit, parent root frame
		FQuat	   Rel0 = FQuat::Identity;												// parent^-1 * child at q = 0
		FVector	   Origin0 = FVector::ZeroVector;										// child origin in the parent frame at q = 0 (cm)
		double	   Mid = 0.0;															// centre of the range (rad or m)
		int32	   Constraint = INDEX_NONE;												// into Constraints
		double	   Damping = 0.0, Stiffness = 0.0, SpringRef = 0.0, FrictionLoss = 0.0; // MJCF units
		bool	   bContinuous = false;													// an unlimited hinge (a wheel): its angle is counted past +-pi
		double	   Unwrapped = 0.0, LastWrapped = 0.0;
	};

	struct FActuator
	{
		FName						 Name;
		EActuatorKind				 Kind = EActuatorKind::Motor;
		TArray<TPair<int32, double>> Joints; // (joint, coefficient x gear)
		double						 Gain = 1.0, Bias0 = 0.0, Kp = 0.0, Kv = 0.0;
		bool						 bCtrlLimited = false;
		double						 CtrlMin = 0.0, CtrlMax = 0.0;
		bool						 bForceLimited = false;
		double						 ForceMin = 0.0, ForceMax = 0.0;
		float						 Ctrl = 0.0f;
	};

	bool Build(FString& OutError);
	void Teardown();
	void ApplyActuator(int32 Index);
	void PushEfforts();
	void WakeRoots(int32 A, int32 B);
	bool MeasureJoint(const FJoint& J, double& OutQ, double& OutQd) const;
	void SyncVisuals();

	UPROPERTY(Transient)
	TArray<TObjectPtr<URammsChaosBodyComponent>> Roots;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UPhysicalMaterial>> Materials;

	TArray<FMjBodyInfo>						MjBodies; // by MuJoCo body id (0 = world, unused)
	TArray<FJoint>							Joints;
	TArray<FActuator>						Actuators;
	TMap<FName, int32>						JointByName;
	TMap<FName, int32>						ActuatorByName;
	TArray<TUniquePtr<FConstraintInstance>> Constraints;
	TArray<double>							ExternalForce; // per joint, MJCF units
	TMap<FString, TArray<float>>			KeyframeCtrl;

	FRammsChaosArticulationCallback* Callback = nullptr;
	bool							 bBuilt = false;
	FString							 BuildError;
};
