// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsBackendSwitchComponent.h"

#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/MjComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "TimerManager.h"

URammsBackendSwitchComponent::URammsBackendSwitchComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	// Mj-component teardown must beat every other actor's BeginPlay (the
	// MjManager must never compile a Chaos-mode robot into its scene).
	bWantsInitializeComponent = true;
}

void URammsBackendSwitchComponent::InitializeComponent()
{
	Super::InitializeComponent();
	// NOTE: Mj components must NOT be destroyed — the visual meshes are
	// their attach children, and orphaning them scatters the robot (the
	// "explodes on simulate" failure). Without an AMjManager in the map the
	// Mj components are inert; mixed-backend maps are a phase-2 item.
}

void URammsBackendSwitchComponent::BeginPlay()
{
	Super::BeginPlay();
	if (Backend == ERammsPhysicsBackend::Chaos)
	{
		// Defer one tick: every component (constraints included) must have
		// finished its own BeginPlay before we flip bodies to simulated and
		// re-init the constraints, or init order leaves stale handles.
		GetWorld()->GetTimerManager().SetTimerForNextTick(
			FTimerDelegate::CreateWeakLambda(this, [this]() { ApplyChaos(); }));
	}
	else
	{
		ApplyMuJoCo();
	}
}

// Component instance names can drift from the recorded SCS variable names
// by a numeric suffix: regenerating the rig in an open editor leaves the
// removed templates alive until GC, so recreated templates (and therefore
// their PIE instances) get uniquified ("ChaosRig_drive_wheel_l1"). Exact
// FName equality then silently skips them — observed as 53/77 dead
// constraints and bodies free-falling out of the assembly.
static bool NameMatchesRecorded(const FName& Actual, const FName& Recorded)
{
	if (Actual == Recorded)
	{
		return true;
	}
	const FString A = Actual.ToString();
	const FString R = Recorded.ToString();
	return A.StartsWith(R) && A.Mid(R.Len()).IsNumeric();
}

static bool NameInRecorded(const FName& Actual, const TArray<FName>& Recorded)
{
	for (const FName& R : Recorded)
	{
		if (NameMatchesRecorded(Actual, R))
		{
			return true;
		}
	}
	return false;
}

void URammsBackendSwitchComponent::ApplyChaos()
{
	AActor* Owner = GetOwner();
	// Only rig bodies may collide. URLab leaves other collidable primitives
	// on the actor (e.g. the MjArticulation query shell): simulated bodies
	// spawn inside that blocking shell and the depenetration shoves them
	// through the world floor ("robot sinks into the ground").
	{
		TArray<UPrimitiveComponent*> AllPrims;
		Owner->GetComponents(AllPrims);
		for (UPrimitiveComponent* Prim : AllPrims)
		{
			if (!NameInRecorded(Prim->GetFName(), ChaosBodyComponents)
				&& Prim->GetCollisionEnabled() != ECollisionEnabled::NoCollision)
			{
				Prim->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			}
		}
	}
	for (int32 BodyIdx = 0; BodyIdx < ChaosBodyComponents.Num(); ++BodyIdx)
	{
		const FName& Name = ChaosBodyComponents[BodyIdx];
		TArray<UPrimitiveComponent*> Prims;
		Owner->GetComponents(Prims);
		for (UPrimitiveComponent* Prim : Prims)
		{
			if (NameMatchesRecorded(Prim->GetFName(), Name))
			{
				Prim->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
				Prim->SetCollisionProfileName(TEXT("PhysicsActor"));
				// MJCF disables ALL robot self-collision (the CAD bodies
				// interpenetrate by design); pairwise constraint disables
				// are not enough — un-constrained overlapping pairs get
				// huge depenetration impulses at t=0 and the assembly
				// explodes. PhysicsBody-ignores-PhysicsBody reproduces the
				// contype/conaffinity scheme (robot-world still collides).
				// Project channel ECC_GameTraceChannel2 = "RobotSelf" (see
				// Config/DefaultEngine.ini): robot parts ignore each other
				// but still collide with world geometry AND with graspable
				// PhysicsBody props (which a PhysicsBody-wide ignore would
				// wrongly exclude).
				Prim->SetCollisionObjectType(ECC_GameTraceChannel2);
				Prim->SetCollisionResponseToChannel(ECC_GameTraceChannel2, ECR_Ignore);
				Prim->SetSimulatePhysics(true);
				// URLab viz meshes end up effectively massless under Chaos
				// (observed 46/64 bodies at ~0 kg); near-massless bodies in
				// hard constraint chains get pushed through the floor. Use
				// the MJCF inertial mass, or a small default.
				const float Mass = BodyMasses.IsValidIndex(BodyIdx) ? BodyMasses[BodyIdx] : 0.f;
				if (Mass > 0.001f)
				{
					Prim->SetMassOverrideInKg(NAME_None, Mass, true);
				}
				else if (Prim->GetMass() < 0.1f)
				{
					// No recorded mass and no usable auto-mass: small default.
					// (URLab's MassInKgOverride is a blanket 100 kg default,
					// not per-body data — using it put the robot at 5 t.)
					Prim->SetMassOverrideInKg(NAME_None, 2.f, true);
				}
			}
		}
	}
	// Multi-material bodies: reattach the non-simulated visual pieces to
	// their simulated body piece so they ride along (hub + tire stay one
	// wheel).
	TArray<UStaticMeshComponent*> Meshes;
	Owner->GetComponents(Meshes);
	auto FindMesh = [&Meshes](const FName& N) -> UStaticMeshComponent* {
		for (UStaticMeshComponent* M : Meshes)
		{
			if (NameMatchesRecorded(M->GetFName(), N))
			{
				return M;
			}
		}
		return nullptr;
	};
	for (int32 i = 0; i < PieceBodies.Num() && i < PieceMeshes.Num(); ++i)
	{
		UStaticMeshComponent* BodyComp = FindMesh(PieceBodies[i]);
		UStaticMeshComponent* Piece = FindMesh(PieceMeshes[i]);
		if (BodyComp && Piece)
		{
			Piece->AttachToComponent(BodyComp,
				FAttachmentTransformRules::KeepWorldTransform);
		}
	}

	// SetSimulatePhysics recreates each body's physics state, so any
	// constraint that initialized earlier in BeginPlay order now holds a
	// stale handle and silently constrains nothing (observed: one body
	// free-falling out of the assembly). Re-init every rig constraint
	// against the fresh bodies.
	TArray<UPhysicsConstraintComponent*> Constraints;
	Owner->GetComponents(Constraints);
	for (UPhysicsConstraintComponent* C : Constraints)
	{
		if (NameInRecorded(C->GetFName(), ChaosConstraintComponents))
		{
			C->InitComponentConstraint();
		}
	}
}

void URammsBackendSwitchComponent::SetJointCommand(FName Joint, float Value)
{
	AActor* Owner = GetOwner();
	if (Backend == ERammsPhysicsBackend::Chaos)
	{
		const int32 Idx = DriveJoints.IndexOfByKey(Joint);
		if (Idx == INDEX_NONE || !DriveConstraints.IsValidIndex(Idx))
		{
			return;
		}
		TArray<UPhysicsConstraintComponent*> Constraints;
		Owner->GetComponents(Constraints);
		for (UPhysicsConstraintComponent* C : Constraints)
		{
			if (!NameMatchesRecorded(C->GetFName(), DriveConstraints[Idx]))
			{
				continue;
			}
			if (DriveIsLinear.IsValidIndex(Idx) && DriveIsLinear[Idx])
			{
				// metres -> cm along the constraint's X (the slide axis)
				C->SetLinearPositionTarget(FVector(Value * 100.f, 0.f, 0.f));
			}
			else
			{
				// rad/s -> rev/s about the twist axis
				C->SetAngularVelocityTarget(FVector(Value / (2.f * PI), 0.f, 0.f));
			}
			return;
		}
		return;
	}

	// MuJoCo/Newton: route to the matching Mj actuator's ctrl.
	TArray<UActorComponent*> Acts;
	Owner->GetComponents(UMjActuator::StaticClass(), Acts);
	const FString JointStr = Joint.ToString();
	for (UActorComponent* AC : Acts)
	{
		UMjActuator* Act = Cast<UMjActuator>(AC);
		if (Act && (Act->TargetName == JointStr || Act->GetMjName() == JointStr))
		{
			Act->SetControl(Value);
			return;
		}
	}
}

void URammsBackendSwitchComponent::ApplyMuJoCo()
{
	AActor* Owner = GetOwner();

	// The Chaos rig must not fight URLab's render pump: constraints off,
	// bodies kinematic (URLab drives the transforms).
	TArray<UPhysicsConstraintComponent*> Constraints;
	Owner->GetComponents(Constraints);
	for (UPhysicsConstraintComponent* C : Constraints)
	{
		if (NameInRecorded(C->GetFName(), ChaosConstraintComponents))
		{
			C->DestroyComponent();
		}
	}
	for (const FName& Name : ChaosBodyComponents)
	{
		TArray<UPrimitiveComponent*> Prims;
		Owner->GetComponents(Prims);
		for (UPrimitiveComponent* Prim : Prims)
		{
			if (NameMatchesRecorded(Prim->GetFName(), Name))
			{
				Prim->SetSimulatePhysics(false);
			}
		}
	}
}
