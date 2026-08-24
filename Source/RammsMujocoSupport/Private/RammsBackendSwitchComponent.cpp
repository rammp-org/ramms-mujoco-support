// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsBackendSwitchComponent.h"

#include "Components/PrimitiveComponent.h"
#include "EngineUtils.h"
#include "Logging/MessageLog.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/MjComponent.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "TimerManager.h"

namespace RammsChaosDbg
{
	static bool					   bDisableGripperDrives = false;
	static FAutoConsoleVariableRef CVarDisableGripperDrives(
		TEXT("Ramms.Debug.DisableGripperDrives"), bDisableGripperDrives,
		TEXT("Bisect aid: strip all gripper hinge drives at ApplyChaos."));

	static float				   ArmInertiaScale = 1.f;
	static FAutoConsoleVariableRef CVarArmInertiaScale(
		TEXT("Ramms.Debug.ArmInertiaScale"), ArmInertiaScale,
		TEXT("Inertia tensor scale applied to arm_ rig bodies at ApplyChaos "
			 "(angular-conditioning experiment; 1 = off)."));

	static float				   GripperDriveScale = 1.f;
	static FAutoConsoleVariableRef CVarGripperDriveScale(
		TEXT("Ramms.Debug.GripperDriveScale"), GripperDriveScale,
		TEXT("Multiply gripper (arm_2f85) hinge drive stiffness/damping at "
			 "ApplyChaos (drive-magnitude experiment; 1 = off)."));

	static float				   GripperLimitDeg = 0.f;
	static FAutoConsoleVariableRef CVarGripperLimitDeg(
		TEXT("Ramms.Debug.GripperLimitDeg"), GripperLimitDeg,
		TEXT("If > 0, override EVERY arm_2f85 hinge twist window to +-N deg "
			 "HARD at ApplyChaos (limit-row-presence experiment; 0 = off)."));

	static bool					   bDisableGripperPins = false;
	static FAutoConsoleVariableRef CVarDisableGripperPins(
		TEXT("Ramms.Debug.DisableGripperPins"), bDisableGripperPins,
		TEXT("Bisect aid: leave the two gripper follower-coupler closure pins "
			 "un-initialized at ApplyChaos (loop-vs-angular-rows experiment)."));

	static float				   GripperAngularProjection = 0.f;
	static FAutoConsoleVariableRef CVarGripperAngularProjection(
		TEXT("Ramms.Debug.GripperAngularProjection"), GripperAngularProjection,
		TEXT("If > 0, set ProjectionAngularAlpha on arm_2f85 hinge constraints "
			 "at ApplyChaos (UE default is 0 — angular error is never "
			 "projected; 0 = leave as configured)."));

	static bool					   bDisableAllProjection = false;
	static FAutoConsoleVariableRef CVarDisableAllProjection(
		TEXT("Ramms.Debug.DisableAllProjection"), bDisableAllProjection,
		TEXT("Bisect aid: force bEnableProjection=false on EVERY ChaosRig "
			 "constraint at ApplyChaos (whole-assembly momentum-injection "
			 "experiment)."));

	static float				   WheelBrakeScale = 1.f;
	static FAutoConsoleVariableRef CVarWheelBrakeScale(
		TEXT("Ramms.Debug.WheelBrakeScale"), WheelBrakeScale,
		TEXT("Scale the drive-wheel velocity-brake force limit at ApplyChaos "
			 "(1 = baked). Suspension articulation drags the braked wheels "
			 "along the floor; their grip is an EXTERNAL ground force that can "
			 "launch the robot."));

	static float				   RodForceCap = 0.f;
	static FAutoConsoleVariableRef CVarRodForceCap(
		TEXT("Ramms.Debug.RodForceCap"), RodForceCap,
		TEXT("If > 0, override the caster motor-rod linear drive force limit "
			 "(kg*cm/s^2; 1e5 = 1 kN) at ApplyChaos. The baked 3 kN cap was "
			 "calibrated against the broken-frame rig; on the freed linkage a "
			 "full-force stroke launches the robot."));

	static bool					   bDisableArmDrives = false;
	static FAutoConsoleVariableRef CVarDisableArmDrives(
		TEXT("Ramms.Debug.DisableArmDrives"), bDisableArmDrives,
		TEXT("Bisect aid: strip the 7-DOF arm joint drives (ChaosRig_arm_* "
			 "minus the 2f85 gripper) at ApplyChaos — arm PD gains converted "
			 "from MuJoCo kp are far above the 60 Hz explicit-solver stability "
			 "limit for the light wrist links."));

	static bool					   bDisableGripperProjection = false;
	static FAutoConsoleVariableRef CVarDisableGripperProjection(
		TEXT("Ramms.Debug.DisableGripperProjection"), bDisableGripperProjection,
		TEXT("Bisect aid: force bEnableProjection=false on every arm_2f85 "
			 "constraint at ApplyChaos (projection-velocity-pump experiment: "
			 "projection teleports position but never corrects velocity, so an "
			 "inconsistent loop can accumulate unbounded body velocity while "
			 "looking stationary)."));

	// GOTCHA (2026-08-18, measured): -ExecCmds are DEFERRED commands — the
	// engine executes them AFTER the first world tick, i.e. AFTER ApplyChaos
	// has already re-initialized every constraint in a headless -game run.
	// Every cvar read below therefore still holds its default during
	// ApplyChaos when set via -ExecCmds, and the earlier drive/inertia
	// bisects silently tested nothing. Command-line switches exist from
	// process start; pull them in here.
	static void PullCommandLineOverrides()
	{
		float V;
		if (FParse::Value(FCommandLine::Get(), TEXT("RammsArmInertiaScale="), V))
		{
			ArmInertiaScale = V;
		}
		if (FParse::Value(FCommandLine::Get(), TEXT("RammsGripperDriveScale="), V))
		{
			GripperDriveScale = V;
		}
		if (FParse::Value(FCommandLine::Get(), TEXT("RammsGripperLimitDeg="), V))
		{
			GripperLimitDeg = V;
		}
		if (FParse::Value(FCommandLine::Get(), TEXT("RammsGripperAngularProjection="), V))
		{
			GripperAngularProjection = V;
		}
		if (FParse::Param(FCommandLine::Get(), TEXT("RammsDisableGripperDrives")))
		{
			bDisableGripperDrives = true;
		}
		if (FParse::Param(FCommandLine::Get(), TEXT("RammsDisableGripperPins")))
		{
			bDisableGripperPins = true;
		}
		if (FParse::Param(FCommandLine::Get(), TEXT("RammsDisableGripperProjection")))
		{
			bDisableGripperProjection = true;
		}
		if (FParse::Param(FCommandLine::Get(), TEXT("RammsDisableAllProjection")))
		{
			bDisableAllProjection = true;
		}
		if (FParse::Param(FCommandLine::Get(), TEXT("RammsDisableArmDrives")))
		{
			bDisableArmDrives = true;
		}
		if (FParse::Value(FCommandLine::Get(), TEXT("RammsRodForceCap="), V))
		{
			RodForceCap = V;
		}
		if (FParse::Value(FCommandLine::Get(), TEXT("RammsWheelBrakeScale="), V))
		{
			WheelBrakeScale = V;
		}
	}
} // namespace RammsChaosDbg

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
	RammsChaosDbg::PullCommandLineOverrides();
	AActor* Owner = GetOwner();
	// NOTE on solver iterations: raising the global cvars
	// (p.Chaos.Solver.Iterations.Position 30-50) was tried against the
	// pin-stretch leak and made the loaded strokes MORE violent (stiffer
	// loop enforcement stores bigger internal forces that release
	// dynamically: rear+ went from a mild tip to a 90 m/s launch). The
	// per-body 32/4 counts below are the stable calibration.
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
	TArray<UPrimitiveComponent*> RigPrims;
	Owner->GetComponents(RigPrims);
	for (int32 BodyIdx = 0; BodyIdx < ChaosBodyComponents.Num(); ++BodyIdx)
	{
		const FName& Name = ChaosBodyComponents[BodyIdx];
		// Exact name first, ONE component per recorded body: the gripper's
		// shared-asset viz names suffix-match across left/right, and the old
		// match-all loop applied each recorded body's mass to every sibling
		// (last writer won, order-dependent).
		UPrimitiveComponent* Match = nullptr;
		for (UPrimitiveComponent* Prim : RigPrims)
		{
			if (Prim->GetFName() == Name)
			{
				Match = Prim;
				break;
			}
		}
		for (UPrimitiveComponent* Prim : RigPrims)
		{
			if (Match)
			{
				break;
			}
			if (NameMatchesRecorded(Prim->GetFName(), Name))
			{
				Match = Prim;
			}
		}
		{
			UPrimitiveComponent* Prim = Match;
			if (Prim)
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
					// Chaos-side floor: authored gram-scale gripper links are
					// physically real but give ~400:1 constraint mass ratios
					// the 60 Hz solver can't hold. Gripper (arm_) links use a
					// LIGHTER 0.05 kg floor: the 0.15 floor made the finger
					// chain ~8x its real weight, which forced anti-sag spring
					// drives strong enough to overconstrain the underactuated
					// spring_link/follower side (user: "secondary links can't
					// move when the motors activate"). 0.05 holds rest with a
					// 3x softer spring that yields on close.
					const float Floor =
						Name.ToString().StartsWith(TEXT("arm_")) ? 0.05f : 0.15f;
					Prim->SetMassOverrideInKg(NAME_None, FMath::Max(Mass, Floor), true);
				}
				else if (Prim->GetMass() < 0.1f)
				{
					// No recorded mass and no usable auto-mass: small default.
					// (URLab's MassInKgOverride is a blanket 100 kg default,
					// not per-body data — using it put the robot at 5 t.)
					Prim->SetMassOverrideInKg(NAME_None, 2.f, true);
				}
				// MJCF gives the small linkage pieces joint damping (50-500)
				// that never reached Chaos: undamped they windmill about
				// their closure pins and pump energy into the assembly
				// (user-observed free-spinning aux linkages). Body-level
				// damping is the Chaos-native equivalent. Wheels and arm
				// links are excluded — wheels must roll freely and the arm
				// has its own PD drives.
				const FString BodyName = Name.ToString();
				// CHASSIS rocking damping: rhythmic arm reversals near ~0.7 Hz
				// resonate the base's pitch/heave mode on its wheel contacts —
				// measured with Ramms.JointSweep (shoulder 0<->1.2 rad, 1.5 s
				// cycles): amplitude grew until the robot fell out of the
				// world, while 4 s cycles produced only a bounded 12 cm hop.
				// The real robot damps this mode through its dampener struts,
				// whose force path Chaos's iterative solver transmits poorly
				// (the long-standing strut leak). Body-level damping on the
				// chassis is the stand-in: angular damps pitch/roll rocking
				// directly; the small linear term damps heave without
				// meaningfully braking travel (~0.1/s on 267 kg).
				if (BodyName.Contains(TEXT("mebot__chassis")))
				{
					Prim->SetAngularDamping(3.f);
					Prim->SetLinearDamping(0.1f);
				}
				const bool bLinkagePiece = !BodyName.StartsWith(TEXT("arm_"))
					&& !BodyName.Contains(TEXT("wheel"))
					&& (BodyName.Contains(TEXT("linkage")) || BodyName.Contains(TEXT("rod"))
						|| BodyName.Contains(TEXT("pivot")) || BodyName.Contains(TEXT("dampener"))
						|| BodyName.Contains(TEXT("swing_arm")) || BodyName.Contains(TEXT("suspension")));
				if (BodyName.Contains(TEXT("wheel")) && !BodyName.StartsWith(TEXT("arm_")))
				{
					// Rolling resistance (MJCF damping 0.3-0.5 + frictionloss
					// equivalent): without it the wheels coast forever on any
					// settle impulse and the chair slides around at rest.
					// 2.0 (was 0.5): the undriven chair still rolled too
					// freely under a push (user feedback 2026-08-18).
					Prim->SetAngularDamping(2.f);
				}
				if (bLinkagePiece)
				{
					Prim->SetAngularDamping(10.f);
					Prim->SetLinearDamping(1.f);
					// Chaos's iterative solver cannot push loop forces
					// through gram-scale links into a 200 kg chassis (the
					// front chain moved at single-N pin forces while MuJoCo's
					// direct solver transmits ~250 N there). Heavier links
					// keep the mass ratio solvable; the real parts are steel.
					if (Prim->GetMass() < 3.f)
					{
						Prim->SetMassOverrideInKg(NAME_None, 3.f, true);
					}
				}
				// More solver iterations for every rig body: the closure
				// loops are long constraint chains; the default 8/1 leaves
				// them mushy (loop force starvation).
				if (FBodyInstance* BI = Prim->GetBodyInstance())
				{
					BI->PositionSolverIterationCount = 32;
					BI->VelocitySolverIterationCount = 4;
					// Angular-conditioning experiment: gripper links' cm^2
					// inertias vs the 8 kg arm make a ~1e4:1 angular ratio —
					// candidate cause of angular limits/drives being inert on
					// arm_ bodies while linear locks enforce fine.
					// NOTE recorded body names are VIZ component names
					// ("Viz_arm_..."), so the original bare "arm_" prefix test
					// NEVER matched — the earlier x20/x20000 inertia runs were
					// silent no-ops.
					if (RammsChaosDbg::ArmInertiaScale != 1.f
						&& (Name.ToString().StartsWith(TEXT("arm_"))
							|| Name.ToString().StartsWith(TEXT("Viz_arm_"))))
					{
						BI->InertiaTensorScale = FVector(RammsChaosDbg::ArmInertiaScale);
						BI->UpdateMassProperties();
					}
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
		// Exact first: shared-asset viz names ("Viz_arm_2f85_follower1..3")
		// suffix-match each other across left/right bodies.
		for (UStaticMeshComponent* M : Meshes)
		{
			if (M->GetFName() == N)
			{
				return M;
			}
		}
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
	TArray<UStaticMeshComponent*> BodyComps;
	Owner->GetComponents(BodyComps);
	TArray<USceneComponent*> SceneComps;
	Owner->GetComponents(SceneComps);
	int32 NumInited = 0;
	for (UPhysicsConstraintComponent* C : Constraints)
	{
		int32 RecIdx = INDEX_NONE;
		for (int32 i = 0; i < ChaosConstraintComponents.Num(); ++i)
		{
			if (NameMatchesRecorded(C->GetFName(), ChaosConstraintComponents[i]))
			{
				RecIdx = i;
				break;
			}
		}
		if (RecIdx == INDEX_NONE)
		{
			continue;
		}
		// Bisect aid (Ramms.Debug.DisableGripperPins): kill ONLY the gripper
		// closure pins. If the finger hinges' angular rows start enforcing
		// with the loop open, the closure is what defeats them.
		if (RammsChaosDbg::bDisableGripperPins
			&& C->GetName().StartsWith(TEXT("ChaosRig_pin_arm_2f85_")))
		{
			C->TermComponentConstraint();
			UE_LOG(LogTemp, Display, TEXT("[ApplyChaos] pin DISABLED: %s"),
				*C->GetName());
			++NumInited; // counted so the stale-instance check stays quiet
			continue;
		}
		// Editor-side construction reruns (flipping Backend in the Details
		// panel, moving the actor, etc.) can DISPLACE constraint components
		// — the user observed constraints jumping far outside the robot
		// after a backend switch, and frames initialized from those poses
		// produce garbage joints. Re-derive the world frame from the
		// recorded child-body-local transform before initializing.
		//
		// PREFERRED source: the owning MjBody component's live transform +
		// the recorded body-local frame. The viz-chain frames below bake the
		// SCS template chain, which drifts from the runtime attachment on
		// the offset arm/gripper meshes — the residual mm-scale pin error
		// that made the finger four-bar leak (sag at rest, over-curl on
		// close). MjBody names are unique per MJCF body, so no shared-asset
		// ambiguity either.
		bool bFrameSet = false;
		if (ConstraintBodyFrames.IsValidIndex(RecIdx)
			&& ConstraintBodyComponents.IsValidIndex(RecIdx))
		{
			USceneComponent* BodyC = nullptr;
			for (USceneComponent* S : SceneComps)
			{
				if (S->GetFName() == ConstraintBodyComponents[RecIdx])
				{
					BodyC = S;
					break;
				}
			}
			for (USceneComponent* S : SceneComps)
			{
				if (BodyC)
				{
					break;
				}
				if (NameMatchesRecorded(S->GetFName(), ConstraintBodyComponents[RecIdx]))
				{
					BodyC = S;
				}
			}
			if (BodyC)
			{
				// SCALE MUST BE 1: UpdateConstraintFrames divides the computed
				// body-local frame positions by the constraint component's
				// scale (RefScale, "used for limits"). The gripper chain
				// carries the imported-asset compensating scales, so a scale
				// inherited here shrank the coupler-side pin frame ~1000x —
				// the pin initialized at the coupler ORIGIN and the finger
				// four-bar was never closed (the long-standing "pin leak").
				FTransform Wt =
					ConstraintBodyFrames[RecIdx] * BodyC->GetComponentTransform();
				Wt.SetScale3D(FVector::OneVector);
				C->SetWorldTransform(Wt);
				bFrameSet = true;
			}
		}
		if (!bFrameSet && ConstraintLocalFrames.IsValidIndex(RecIdx)
			&& ConstraintChildBodies.IsValidIndex(RecIdx))
		{
			// EXACT name first: the gripper's shared-asset viz components
			// ("Viz_arm_2f85_follower", "...follower1..3" across left AND
			// right) all suffix-match each other's recorded names, and a
			// first-suffix-match here derived the LEFT pin's frame from the
			// RIGHT follower — every gripper pin initialized 4.8 cm off
			// (fingers spawned deformed: followers +51 deg, couplers -15).
			UStaticMeshComponent* Child = nullptr;
			for (UStaticMeshComponent* B : BodyComps)
			{
				if (B->GetFName() == ConstraintChildBodies[RecIdx])
				{
					Child = B;
					break;
				}
			}
			for (UStaticMeshComponent* B : BodyComps)
			{
				if (Child)
				{
					break;
				}
				if (NameMatchesRecorded(B->GetFName(), ConstraintChildBodies[RecIdx]))
				{
					Child = B;
				}
			}
			if (Child)
			{
				// Same scale-1 rule as the body-frame path above.
				FTransform Wt =
					ConstraintLocalFrames[RecIdx] * Child->GetComponentTransform();
				Wt.SetScale3D(FVector::OneVector);
				C->SetWorldTransform(Wt);
			}
		}
		// InitComponentConstraint reuses the SAVED body-local frames — it does
		// NOT recompute them from the component transform we just corrected,
		// and UpdateConstraintFrames is a no-op while the joint is live.
		// Terminate first, recompute the frames from the corrected world pose
		// against the bodies' spawn poses, then init. Without this the
		// gripper closure pins kept TEMPLATE-time frames on the coupler side
		// (4.8 cm off at spawn): the pin initialized permanently torn and the
		// finger four-bar sagged to its window edges.
		// Projection must stay OFF on closure pins even if a stale rig
		// recorded it on (generator emitted it for gripper pins before
		// 2026-08-18): with projection the solver leaves the locked pin torn
		// ~4.8 cm at rest; without it the same pin holds at 0.00-0.03 cm.
		if (C->GetName().StartsWith(TEXT("ChaosRig_pin_")))
		{
			C->ConstraintInstance.ProfileInstance.bEnableProjection = false;
		}
		// Bisect aid (Ramms.Debug.DisableGripperProjection): projection can
		// mask a loop inconsistency as ever-growing body velocity.
		if (RammsChaosDbg::bDisableGripperProjection
			&& C->GetName().StartsWith(TEXT("ChaosRig_arm_2f85_")))
		{
			C->ConstraintInstance.ProfileInstance.bEnableProjection = false;
		}
		// Bisect aid (Ramms.Debug.DisableAllProjection): projection teleports
		// POSITION without touching velocity and ignores momentum conservation
		// — over an inconsistent closure-loop network it can act as a
		// continuous thrust on the whole assembly (measured: the entire robot
		// coherently accelerating to 100+ m/s in mid-air at rest commands).
		if (RammsChaosDbg::bDisableAllProjection
			&& C->GetName().StartsWith(TEXT("ChaosRig_")))
		{
			C->ConstraintInstance.ProfileInstance.bEnableProjection = false;
		}
		// Tuning aid (Ramms.Debug.WheelBrakeScale): scale the drive-wheel
		// brake force limit without a regen.
		if (RammsChaosDbg::WheelBrakeScale != 1.f
			&& (C->GetName().StartsWith(TEXT("ChaosRig_drive_wheel"))))
		{
			FConstraintInstance&	XCI = C->ConstraintInstance;
			const FConstraintDrive& TD = XCI.ProfileInstance.AngularDrive.TwistDrive;
			if (TD.bEnableVelocityDrive)
			{
				const float BaseCap = TD.MaxForce > 0.f ? TD.MaxForce : 6e5f;
				XCI.SetAngularDriveParams(TD.Stiffness, TD.Damping,
					BaseCap * RammsChaosDbg::WheelBrakeScale);
				UE_LOG(LogTemp, Display, TEXT("[ApplyChaos] wheel brake cap x%.2f on %s"),
					RammsChaosDbg::WheelBrakeScale, *C->GetName());
			}
		}
		// Tuning aid (Ramms.Debug.RodForceCap): override the caster rod servo
		// force limit without a regen.
		if (RammsChaosDbg::RodForceCap > 0.f
			&& C->GetName().Contains(TEXT("caster_motor_rod")))
		{
			FConstraintInstance&	XCI = C->ConstraintInstance;
			const FConstraintDrive& XD = XCI.ProfileInstance.LinearDrive.XDrive;
			XCI.SetLinearDriveParams(XD.Stiffness, XD.Damping, RammsChaosDbg::RodForceCap);
			UE_LOG(LogTemp, Display, TEXT("[ApplyChaos] rod force cap %.0f on %s"),
				RammsChaosDbg::RodForceCap, *C->GetName());
		}
		// Bisect aid (Ramms.Debug.DisableArmDrives): strip the 7-DOF arm's
		// joint drives (NOT the gripper's).
		if (RammsChaosDbg::bDisableArmDrives
			&& C->GetName().StartsWith(TEXT("ChaosRig_arm_"))
			&& !C->GetName().StartsWith(TEXT("ChaosRig_arm_2f85_")))
		{
			FConstraintInstance& XCI = C->ConstraintInstance;
			XCI.SetOrientationDriveTwistAndSwing(false, false);
			XCI.SetAngularVelocityDriveTwistAndSwing(false, false);
			XCI.SetAngularDriveParams(0.f, 0.f, 0.f);
		}
		// Bisect aid (Ramms.Debug.DisableGripperDrives): strip the gripper
		// hinge drives to separate constraint-geometry torque from drive
		// dynamics (used to isolate the four-bar rest-pose walk).
		if (RammsChaosDbg::bDisableGripperDrives
			&& C->GetName().StartsWith(TEXT("ChaosRig_arm_2f85_")))
		{
			FConstraintInstance& XCI = C->ConstraintInstance;
			XCI.SetOrientationDriveTwistAndSwing(false, false);
			XCI.SetAngularVelocityDriveTwistAndSwing(false, false);
			XCI.SetAngularDriveParams(0.f, 0.f, 0.f);
		}
		// Bisect aid (Ramms.Debug.GripperDriveScale): scale the gripper hinge
		// drive gains — the 1e4 dt-limit was calibrated against the
		// broken-pin-frame rig; with the loop closed the safe ceiling may be
		// far higher.
		if (RammsChaosDbg::GripperDriveScale != 1.f
			&& C->GetName().StartsWith(TEXT("ChaosRig_arm_2f85_")))
		{
			FConstraintInstance&	XCI = C->ConstraintInstance;
			const FConstraintDrive& TD = XCI.ProfileInstance.AngularDrive.TwistDrive;
			if (TD.bEnablePositionDrive || TD.bEnableVelocityDrive)
			{
				XCI.SetAngularDriveParams(
					TD.Stiffness * RammsChaosDbg::GripperDriveScale,
					TD.Damping * RammsChaosDbg::GripperDriveScale, 0.f);
			}
		}
		// Experiment (Ramms.Debug.GripperAngularProjection): UE defaults
		// ProjectionAngularAlpha to 0 — "projection on" only ever projected
		// LINEAR error, which is why linear rows enforce crisply while the
		// angular windows leak double digits under closure-loop load.
		if (RammsChaosDbg::GripperAngularProjection > 0.f
			&& C->GetName().StartsWith(TEXT("ChaosRig_arm_2f85_")))
		{
			FConstraintInstance& XCI = C->ConstraintInstance;
			XCI.ProfileInstance.bEnableProjection = true;
			XCI.ProfileInstance.ProjectionAngularAlpha =
				FMath::Clamp(RammsChaosDbg::GripperAngularProjection, 0.f, 1.f);
		}
		// Bisect aid (Ramms.Debug.GripperLimitDeg): clamp every gripper hinge
		// twist window to a tiny HARD range. If followers obey it while
		// drivers still fall past it, the driver's angular limit rows are
		// provably absent from the live solver (not just mis-tuned).
		if (RammsChaosDbg::GripperLimitDeg > 0.f
			&& C->GetName().StartsWith(TEXT("ChaosRig_arm_2f85_")))
		{
			FConstraintInstance& XCI = C->ConstraintInstance;
			XCI.SetAngularTwistLimit(ACM_Limited, RammsChaosDbg::GripperLimitDeg);
			XCI.ProfileInstance.TwistLimit.bSoftConstraint = false;
			UE_LOG(LogTemp, Display,
				TEXT("[ApplyChaos] twist window override %.1f deg on %s"),
				RammsChaosDbg::GripperLimitDeg, *C->GetName());
		}
		C->TermComponentConstraint();
		C->UpdateConstraintFrames();
		// ASYMMETRIC twist windows (2f85 four-bar): rotate the PARENT ref
		// frame about the twist axis by the recorded window center, so the
		// symmetric +-half window covers the true MJCF range. Measured twist
		// then reads (physical - center): the 2f85 rest pose sits ON its
		// open stop, exactly like MuJoCo. Drive targets are shifted by the
		// same center (here for the rest hold, ApplyJointTarget for
		// commands).
		if (ConstraintTwistCenters.IsValidIndex(RecIdx)
			&& FMath::Abs(ConstraintTwistCenters[RecIdx]) > 0.01f)
		{
			FConstraintInstance& XCI = C->ConstraintInstance;
			const float			 CenterRad =
				FMath::DegreesToRadians(ConstraintTwistCenters[RecIdx]);
			FTransform F1 = XCI.GetRefFrame(EConstraintFrame::Frame1);
			F1.SetRotation(F1.GetRotation() * FQuat(FVector::XAxisVector, CenterRad));
			XCI.SetRefFrame(EConstraintFrame::Frame1, F1);
			if (XCI.ProfileInstance.AngularDrive.TwistDrive.bEnablePositionDrive)
			{
				// Compose the BAKED drive target with the recentered frame
				// (measured twist = physical - center). The baked roll is 0
				// for plain rest holds — physical 0, i.e. hold the spawn
				// pose — and -springref for preload springs (2f85
				// spring_link), which must keep pulling toward the MJCF
				// spring equilibrium, not get overwritten into a park.
				const float BakedRad = FMath::DegreesToRadians(
					XCI.ProfileInstance.AngularDrive.OrientationTarget.Roll);
				XCI.SetAngularOrientationTarget(
					FQuat(FVector::XAxisVector, BakedRad - CenterRad));
			}
		}
		C->InitComponentConstraint();
		// Verify the LIVE binding: Init resolves ComponentName1/2 by EXACT
		// FName, so a suffix-drifted instance (shared-asset viz uniquification)
		// or a stale body handle silently yields a dead joint — observed as the
		// entire wrist+gripper assembly departing the robot as one intact
		// cluster at ~100 m/s while every relative metric looked healthy.
		if (!C->ConstraintInstance.IsValidConstraintInstance())
		{
			UE_LOG(LogTemp, Error,
				TEXT("[ApplyChaos] DEAD CONSTRAINT after init: %s (P1=%s P2=%s)"),
				*C->GetName(), *C->ComponentName1.ComponentName.ToString(),
				*C->ComponentName2.ComponentName.ToString());
		}
		// Arm drive audit: the LIVE per-constraint gains/limits, so a PIE
		// session and a headless probe can be compared line-for-line (a
		// stale placed instance or template drift shows up here as
		// MaxForce=0 or wrong stiffness — the difference between an arm
		// swing tipping the base or not).
		if (C->GetName().StartsWith(TEXT("ChaosRig_arm_"))
			&& !C->GetName().StartsWith(TEXT("ChaosRig_arm_2f85_")))
		{
			const FConstraintDrive& TD =
				C->ConstraintInstance.ProfileInstance.AngularDrive.TwistDrive;
			if (TD.bEnablePositionDrive)
			{
				UE_LOG(LogTemp, Display,
					TEXT("[ApplyChaos] armdrive %s: k=%.0f d=%.0f maxTorque=%.0f"),
					*C->GetName(), TD.Stiffness, TD.Damping, TD.MaxForce);
			}
		}
		++NumInited;
	}
	if (NumInited < ChaosConstraintComponents.Num())
	{
		// A placed instance from an older rig generation: recorded names no
		// longer resolve, so part of the rig is DEAD (bodies fall out,
		// joints gain DOF, commands hit wrong constraints). Every past
		// "very wrong behavior" report traced back to this.
		UE_LOG(LogTemp, Error,
			TEXT("[%s] STALE ROBOT INSTANCE: only %d of %d rig constraints "
				 "resolved. Delete this actor and re-place it from the "
				 "regenerated blueprint."),
			*Owner->GetActorNameOrLabel(), NumInited, ChaosConstraintComponents.Num());
#if WITH_EDITOR
		FMessageLog("PIE").Error(FText::FromString(FString::Printf(
			TEXT("%s: stale robot instance (%d/%d rig constraints) — delete "
				 "and re-place it from the updated blueprint."),
			*Owner->GetActorNameOrLabel(), NumInited, ChaosConstraintComponents.Num())));
#endif
	}

	if (CouplerLeaders.Num() > 0)
	{
		GetWorld()->GetTimerManager().SetTimer(CouplerTimer,
			FTimerDelegate::CreateWeakLambda(this, [this]() { TickCouplers(); }),
			0.016f, true);
	}

	if (bNeverSleep)
	{
		// Chaos puts the settled robot to sleep and sleeping bodies ignore
		// drive targets (SleepFamily::Custom with multiplier 0 does NOT
		// prevent it — verified asleep). Brute-force keep-awake tick.
		GetWorld()->GetTimerManager().SetTimer(KeepAwakeTimer,
			FTimerDelegate::CreateWeakLambda(this, [this]() { WakeRigBodies(); }),
			0.5f, true);
	}
}

void URammsBackendSwitchComponent::TickCouplers()
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	TArray<UPhysicsConstraintComponent*> Constraints;
	Owner->GetComponents(Constraints);
	for (int32 i = 0; i < CouplerLeaders.Num() && i < CouplerFollowers.Num(); ++i)
	{
		UPhysicsConstraintComponent* Leader = nullptr;
		UPhysicsConstraintComponent* Follower = nullptr;
		for (UPhysicsConstraintComponent* C : Constraints)
		{
			if (!Leader && NameMatchesRecorded(C->GetFName(), CouplerLeaders[i]))
			{
				Leader = C;
			}
			if (!Follower && NameMatchesRecorded(C->GetFName(), CouplerFollowers[i]))
			{
				Follower = C;
			}
		}
		if (!Leader || !Follower)
		{
			continue;
		}
		const float Ratio = CouplerRatios.IsValidIndex(i) ? CouplerRatios[i] : 1.f;
		// Leader value: hinge twist (deg) — or for a SLIDE leader (rod servo),
		// its extension along X in cm from the spawn pose. Ratio units follow.
		float LeaderVal = Leader->GetCurrentTwist();
		if (Leader->ConstraintInstance.GetLinearXMotion() != ELinearConstraintMotion::LCM_Locked)
		{
			UPrimitiveComponent* P1 = nullptr;
			UPrimitiveComponent* P2 = nullptr;
			FName				 B1, B2;
			Leader->GetConstrainedComponents(P1, B1, P2, B2);
			if (P1 && P2)
			{
				const float Ext = FVector::DotProduct(
					P2->GetComponentLocation() - P1->GetComponentLocation(),
					Leader->GetForwardVector());
				if (CouplerLeaderRest.Num() != CouplerLeaders.Num())
				{
					CouplerLeaderRest.SetNumZeroed(CouplerLeaders.Num());
					CouplerLeaderRestSet.SetNumZeroed(CouplerLeaders.Num());
				}
				if (!CouplerLeaderRestSet[i])
				{
					CouplerLeaderRest[i] = Ext;
					CouplerLeaderRestSet[i] = 1;
				}
				LeaderVal = Ext - CouplerLeaderRest[i];
			}
		}
		const float Target = LeaderVal * Ratio; // degrees
		Follower->SetAngularOrientationTarget(FRotator(0.f, 0.f, Target));
	}
}

void URammsBackendSwitchComponent::WakeRigBodies()
{
	AActor*						 Owner = GetOwner();
	TArray<UPrimitiveComponent*> Prims;
	Owner->GetComponents(Prims);
	for (UPrimitiveComponent* Prim : Prims)
	{
		if (Prim->IsSimulatingPhysics()
			&& NameInRecorded(Prim->GetFName(), ChaosBodyComponents))
		{
			Prim->WakeAllRigidBodies();
		}
	}
}

void URammsBackendSwitchComponent::SetJointCommand(FName Joint, float Value)
{
	// Chaos LINEAR rods get a slewed target (~3 cm/s): a 6 cm step at
	// kp=6e5 catapults the mechanism the moment the ground load releases
	// (rear rod threw the robot 130 m through the floor) — MuJoCo's
	// implicit integrator absorbs the same step quasistatically. The slew
	// is safe again at kp=6e5: even a 1 mm lag develops ~600 N (the old
	// force-starvation happened at kp=6e4 with full-stroke ramps).
	// Hinge position/velocity drives still apply directly.
	if (Backend == ERammsPhysicsBackend::Chaos)
	{
		for (int32 Idx = 0; Idx < DriveJoints.Num(); ++Idx)
		{
			if (DriveJoints[Idx] != Joint)
			{
				continue;
			}
			const bool bLinear = DriveIsLinear.IsValidIndex(Idx) && DriveIsLinear[Idx];
			const bool bAngularPosition = !bLinear
				&& DriveIsPosition.IsValidIndex(Idx) && DriveIsPosition[Idx];
			if (bLinear || bAngularPosition)
			{
				// Clamp to the published drive range = mechanism-realizable
				// travel (the caster rods' MJCF ctrlrange overshoots what the
				// linkage can do; a stalled full-force servo flips the robot).
				if (DriveCtrlMin.IsValidIndex(Idx) && DriveCtrlMax.IsValidIndex(Idx)
					&& DriveCtrlMax[Idx] > DriveCtrlMin[Idx])
				{
					Value = FMath::Clamp(Value, DriveCtrlMin[Idx], DriveCtrlMax[Idx]);
				}
				// SLEW every position drive, angular included: dragging the
				// arm_joint_2 panel slider back and forth commanded ~170 deg/s
				// shoulder reversals — the kp 2e7 drive slammed the arm into
				// its hard windows each cycle and pumped the 267 kg base 57 cm
				// off the ground (measured with Ramms.JointSweep x4). The real
				// Kinova joints are velocity-limited to ~50-57 deg/s; slewing
				// the TARGET at 90 deg/s keeps drive tracking error (and
				// therefore torque transients) bounded regardless of how the
				// UI moves.
				FVector2D& S = SlewTargets.FindOrAdd(Joint); // X: slewed cur (starts 0 = rest)
				S.Y = Value;
				if (!GetWorld()->GetTimerManager().IsTimerActive(SlewTimer))
				{
					GetWorld()->GetTimerManager().SetTimer(SlewTimer,
						FTimerDelegate::CreateWeakLambda(this, [this]() { TickSlew(); }),
						0.033f, true);
				}
				return;
			}
		}
	}
	ApplyJointTarget(Joint, Value);
}

void URammsBackendSwitchComponent::TickSlew()
{
	bool bAnyMoving = false;
	for (TPair<FName, FVector2D>& Pair : SlewTargets)
	{
		const float Cur = (float)Pair.Value.X;
		const float Cmd = (float)Pair.Value.Y;
		if (FMath::IsNearlyEqual(Cur, Cmd, 1e-4f))
		{
			continue;
		}
		// Per-type target rate:
		//  - linear rods (metres): 0.03 units/s = 3 cm/s, the quasistatic
		//    pace MuJoCo's strokes settle at;
		//  - angular position drives (radians): 1.6 rad/s (~90 deg/s),
		//    bounding drive tracking error above the real Kinova joint
		//    speed limits (50-57 deg/s) but far below the slider-slam
		//    rates that pumped the base airborne.
		float Rate = 0.03f;
		for (int32 Idx = 0; Idx < DriveJoints.Num(); ++Idx)
		{
			if (DriveJoints[Idx] == Pair.Key)
			{
				if (!(DriveIsLinear.IsValidIndex(Idx) && DriveIsLinear[Idx]))
				{
					Rate = 1.6f;
				}
				break;
			}
		}
		const float Next = FMath::FInterpConstantTo(Cur, Cmd, 0.033f, Rate);
		Pair.Value.X = Next;
		ApplyJointTarget(Pair.Key, Next);
		bAnyMoving = true;
	}
	if (!bAnyMoving)
	{
		GetWorld()->GetTimerManager().ClearTimer(SlewTimer);
	}
}

void URammsBackendSwitchComponent::ApplyJointTarget(FName Joint, float Value)
{
	AActor* Owner = GetOwner();
	if (Backend == ERammsPhysicsBackend::Chaos)
	{
		TArray<UPhysicsConstraintComponent*> Constraints;
		Owner->GetComponents(Constraints);
		bool bAny = false;
		// A drive name may map to SEVERAL constraints (tendon actuators
		// drive every wrapped joint, e.g. both gripper drivers).
		for (int32 Idx = 0; Idx < DriveJoints.Num(); ++Idx)
		{
			if (DriveJoints[Idx] != Joint || !DriveConstraints.IsValidIndex(Idx))
			{
				continue;
			}
			const float Scaled = Value
				* (DriveScale.IsValidIndex(Idx) ? DriveScale[Idx] : 1.f);
			for (UPhysicsConstraintComponent* C : Constraints)
			{
				if (!NameMatchesRecorded(C->GetFName(), DriveConstraints[Idx]))
				{
					continue;
				}
				if (!bAny)
				{
					// Sleeping bodies ignore drive targets — wake the rig.
					WakeRigBodies();
					bAny = true;
				}
				if (DriveIsLinear.IsValidIndex(Idx) && DriveIsLinear[Idx])
				{
					// metres -> cm along the constraint's X (the slide axis)
					C->SetLinearPositionTarget(FVector(Scaled * 100.f, 0.f, 0.f));
				}
				else if (DriveIsPosition.IsValidIndex(Idx) && DriveIsPosition[Idx])
				{
					// radians -> orientation target about the twist axis (X).
					// Asymmetric-window constraints measure twist offset by
					// the recorded window center; shift the target so command
					// values stay in spawn-zero (MJCF) coordinates.
					float CenterDeg = 0.f;
					for (int32 i = 0; i < ChaosConstraintComponents.Num(); ++i)
					{
						if (ChaosConstraintComponents[i] == DriveConstraints[Idx])
						{
							CenterDeg = ConstraintTwistCenters.IsValidIndex(i)
								? ConstraintTwistCenters[i]
								: 0.f;
							break;
						}
					}
					C->SetAngularOrientationTarget(FRotator(0.f, 0.f,
						FMath::RadiansToDegrees(Scaled) - CenterDeg));
				}
				else
				{
					// rad/s -> rev/s about the twist axis
					C->SetAngularVelocityTarget(FVector(Scaled / (2.f * PI), 0.f, 0.f));
				}
				break;
			}
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
			// The articulation resolves effective ctrl from ONE source
			// (ZMQ -> NetworkValue, UI -> InternalValue). Write both so the
			// command takes effect regardless of the map's ControlSource.
			Act->SetControl(Value);
			Act->SetNetworkControl(Value);
			return;
		}
	}
}

// Manual joint actuation from the console (any backend):
//   Ramms.Joint <ActorLabelOrName> <Joint> <Value>
static FAutoConsoleCommandWithWorldAndArgs GRammsJointCmd(
	TEXT("Ramms.Joint"),
	TEXT("Ramms.Joint <ActorLabel> <Joint> <Value> — route a joint command "
		 "through URammsBackendSwitchComponent::SetJointCommand."),
	FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(
		[](const TArray<FString>& Args, UWorld* World) {
			if (Args.Num() < 3 || !World)
			{
				UE_LOG(LogTemp, Warning, TEXT("usage: Ramms.Joint <ActorLabel> <Joint> <Value>"));
				return;
			}
			const float Value = FCString::Atof(*Args[2]);
			for (TActorIterator<AActor> It(World); It; ++It)
			{
				AActor* Actor = *It;
				if (Actor->GetActorNameOrLabel() != Args[0] && Actor->GetName() != Args[0])
				{
					continue;
				}
				if (URammsBackendSwitchComponent* Sw =
						Actor->FindComponentByClass<URammsBackendSwitchComponent>())
				{
					Sw->SetJointCommand(FName(*Args[1]), Value);
					UE_LOG(LogTemp, Display, TEXT("Ramms.Joint %s %s = %f"),
						*Args[0], *Args[1], Value);
					return;
				}
			}
			UE_LOG(LogTemp, Warning, TEXT("Ramms.Joint: no actor '%s' with a backend switch"), *Args[0]);
		}));

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
