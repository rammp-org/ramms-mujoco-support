// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsChaosArticulationComponent.h"

#include "Chaos/SimCallbackObject.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Spec/MjAssetFiles.h"
#include "MuJoCo/Spec/MjSceneSpec.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "PBDRigidsSolver.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

#include <mujoco/mujoco.h>

DEFINE_LOG_CATEGORY_STATIC(LogRammsChaosArticulation, Log, All);

// ---------------------------------------------------------------------------------------------- //
// Physics-rate forces: torque motors, joint damping, stiffness and friction loss.
// ---------------------------------------------------------------------------------------------- //

/**
 * Runs before every Chaos step. Chaos constraint drives cover the position servos; everything that is
 * a force on a joint rather than a target (a <motor>, a joint's damping / stiffness / frictionloss, a
 * test's external push) is applied here, at the physics rate, as equal and opposite torques (or
 * forces, on a slide) on the joint's two bodies.
 */
class FRammsChaosArticulationCallback : public Chaos::TSimCallbackObject<Chaos::FSimCallbackNoInput, Chaos::FSimCallbackNoOutput, Chaos::ESimCallbackOptions::Presimulate>
{
public:
	struct FPtJoint
	{
		FPhysicsActorHandle Parent = nullptr; // null: the parent is the world (StaticParent)
		FPhysicsActorHandle Child = nullptr;
		FTransform			StaticParent = FTransform::Identity;
		bool				bHinge = true;
		FVector				AxisInParent = FVector::XAxisVector;
		FQuat				Rel0 = FQuat::Identity;
		FVector				Origin0 = FVector::ZeroVector;
		// Unreal units: kg.cm^2/s^2 per rad (hinge) or kg/s^2 (slide), and so on.
		double Damping = 0.0, Stiffness = 0.0, SpringRef = 0.0, FrictionLoss = 0.0;
		double Effort = 0.0;
		// The joint as of the latest step, for the game thread's reads: fresher than the bodies' game-
		// thread transforms, which async physics interpolates a step or more behind. A hinge's angle is
		// counted on past +-pi here, where no step turns it half a revolution.
		bool   bMeasured = false;
		double Q = 0.0, Counted = 0.0, Qd = 0.0;
	};

	/** A torque motor's speed loop (SetVelocityDrive), its output the motor's control. */
	struct FPtDrive
	{
		int32						 Joint = INDEX_NONE; // the joint whose rate it measures
		TArray<TPair<int32, double>> Out;				 // (joint, coefficient x gear) the force goes to
		double						 Gain = 1.0, Bias0 = 0.0;
		bool						 bForceLimited = false;
		double						 ForceMin = 0.0, ForceMax = 0.0;
		double						 RateScale = 1.0; // caller's rate units per MJCF unit
		double						 Target = 0.0, Kp = 0.0, Ki = 0.0, MaxIntegral = 0.0, OutMin = 0.0, OutMax = 0.0;
		double						 Integral = 0.0, LastCtrl = 0.0;
	};

	FCriticalSection	  Lock;
	TArray<FPtJoint>	  Joints;
	TMap<FName, FPtDrive> Drives;
	TArray<double>		  DriveEffort; // per joint, this step (Unreal units)

	virtual FName GetFNameForStatId() const override
	{
		static const FName Name(TEXT("RammsChaosArticulation"));
		return Name;
	}

	/** A joint's coordinate and rate (rad, rad/s; cm, cm/s on a slide) and its world axis. */
	static bool Measure(const FPtJoint& J, double& Q, double& Qd, FVector& Axis)
	{
		Chaos::FRigidBodyHandle_Internal* C = J.Child ? J.Child->GetPhysicsThreadAPI() : nullptr;
		if (C == nullptr)
		{
			return false;
		}
		Chaos::FRigidBodyHandle_Internal* P = J.Parent ? J.Parent->GetPhysicsThreadAPI() : nullptr;
		const FQuat						  Rp = P ? FQuat(P->R()) : J.StaticParent.GetRotation();
		const FVector					  Xp = P ? FVector(P->X()) : J.StaticParent.GetLocation();
		const FVector					  Wp = P ? FVector(P->W()) : FVector::ZeroVector;
		const FVector					  Vp = P ? FVector(P->V()) : FVector::ZeroVector;
		Axis = Rp.RotateVector(J.AxisInParent);
		if (J.bHinge)
		{
			const FQuat D = (Rp.Inverse() * FQuat(C->R())) * J.Rel0.Inverse();
			Q = FMath::UnwindRadians(2.0 * FMath::Atan2(FVector(D.X, D.Y, D.Z) | J.AxisInParent, D.W));
			Qd = (FVector(C->W()) - Wp) | Axis;
		}
		else
		{
			Q = (Rp.UnrotateVector(FVector(C->X()) - Xp) - J.Origin0) | J.AxisInParent;
			Qd = (FVector(C->V()) - Vp) | Axis;
		}
		return true;
	}

	/** The speed loops, the same PI the robot base runs: integral capped, and held while the output is
	 *  saturated so it does not wind up. Their forces land in DriveEffort. */
	void StepDrives()
	{
		DriveEffort.SetNumUninitialized(Joints.Num());
		for (double& E : DriveEffort)
		{
			E = 0.0;
		}
		const double Dt = GetDeltaTime_Internal();
		for (TPair<FName, FPtDrive>& Pair : Drives)
		{
			FPtDrive& D = Pair.Value;
			double	  Q = 0.0, Qd = 0.0;
			FVector	  Axis;
			if (!Joints.IsValidIndex(D.Joint) || !Measure(Joints[D.Joint], Q, Qd, Axis))
			{
				continue;
			}
			const double Rate = (Joints[D.Joint].bHinge ? Qd : Qd * 0.01) * D.RateScale;
			const double Error = D.Target - Rate;
			double		 Integral = D.Integral + D.Ki * Error * Dt;
			if (D.MaxIntegral > 0.0)
			{
				Integral = FMath::Clamp(Integral, -D.MaxIntegral, D.MaxIntegral);
			}
			double Ctrl = D.Kp * Error + Integral;
			if (D.OutMin < D.OutMax)
			{
				const double Clamped = FMath::Clamp(Ctrl, D.OutMin, D.OutMax);
				if (Clamped != Ctrl)
				{
					Integral = D.Integral;
				}
				Ctrl = Clamped;
			}
			D.Integral = Integral;
			D.LastCtrl = Ctrl;
			double F = D.Gain * Ctrl + D.Bias0;
			if (D.bForceLimited)
			{
				F = FMath::Clamp(F, D.ForceMin, D.ForceMax);
			}
			for (const TPair<int32, double>& O : D.Out)
			{
				if (DriveEffort.IsValidIndex(O.Key))
				{
					DriveEffort[O.Key] += F * O.Value * (Joints[O.Key].bHinge ? 1.0e4 : 100.0);
				}
			}
		}
	}

	virtual void OnPreSimulate_Internal() override
	{
		FScopeLock Guard(&Lock);
		StepDrives();
		for (int32 j = 0; j < Joints.Num(); ++j)
		{
			FPtJoint& J = Joints[j];
			double	  Q = 0.0, Qd = 0.0;
			FVector	  Axis;
			if (!Measure(J, Q, Qd, Axis))
			{
				continue;
			}
			J.Counted = (J.bMeasured && J.bHinge) ? J.Counted + FMath::UnwindRadians(Q - J.Q) : Q;
			J.Q = Q;
			J.Qd = Qd;
			J.bMeasured = true;
			Chaos::FRigidBodyHandle_Internal* C = J.Child->GetPhysicsThreadAPI();
			Chaos::FRigidBodyHandle_Internal* P = J.Parent ? J.Parent->GetPhysicsThreadAPI() : nullptr;
			double							  Tau = J.Effort + DriveEffort[j] - J.Damping * Qd - J.Stiffness * (Q - J.SpringRef);
			if (J.FrictionLoss > 0.0)
			{
				Tau -= J.FrictionLoss * FMath::Clamp(Qd / (J.bHinge ? 0.01 : 0.1), -1.0, 1.0);
			}
			if (Tau == 0.0)
			{
				continue;
			}
			const Chaos::FVec3 Out = Axis * Tau;
			if (J.bHinge)
			{
				C->AddTorque(Out);
				if (P)
				{
					P->AddTorque(-Out);
				}
			}
			else
			{
				C->AddForce(Out);
				if (P)
				{
					P->AddForce(-Out);
				}
			}
		}
	}
};

// ---------------------------------------------------------------------------------------------- //

URammsChaosBodyComponent::URammsChaosBodyComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	SetGenerateOverlapEvents(false);
	bHiddenInGame = true;
	SetCastShadow(false);
}

FBoxSphereBounds URammsChaosBodyComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	if (ShapeSetup)
	{
		const FBox Box = ShapeSetup->AggGeom.CalcAABB(LocalToWorld);
		if (Box.IsValid)
		{
			return FBoxSphereBounds(Box);
		}
	}
	return FBoxSphereBounds(LocalToWorld.GetLocation(), FVector(1.0), 1.0);
}

// ---------------------------------------------------------------------------------------------- //

namespace
{
	FTransform MjPoseToUe(const mjtNum Pos[3], const mjtNum Quat[4])
	{
		return FTransform(URLabAxisConv::MjQuatToUe(Quat), URLabAxisConv::MjPositionToUe(Pos));
	}

	/** A hinge's axis in Unreal's frame. The axis is a pseudovector, so the mirror that turns MuJoCo's
	 *  right-handed frame into Unreal's also flips it; negating keeps the joint coordinate's sign. */
	FVector HingeAxisToUe(const mjtNum A[3])
	{
		return -URLabAxisConv::MjDirectionToUe(A).GetSafeNormal();
	}

	/** A frame at Location whose X axis is Axis. */
	FTransform AxisFrame(const FVector& Location, const FVector& Axis)
	{
		return FTransform(FRotationMatrix::MakeFromX(Axis).ToQuat(), Location);
	}

	FName MjName(const mjModel* M, int32 Type, int32 Id)
	{
		const char* N = mj_id2name(M, Type, Id);
		return N ? FName(UTF8_TO_TCHAR(N)) : FName(*FString::Printf(TEXT("%s_%d"), Type == mjOBJ_BODY ? TEXT("body") : TEXT("element"), Id));
	}

	/** Points of a geom that has no Chaos primitive (cylinder, ellipsoid, mesh), in the geom's frame (m). */
	void GeomPoints(const mjModel* M, int32 G, int32 Sides, TArray<FVector>& Out)
	{
		const mjtNum* S = M->geom_size + 3 * G;
		switch (M->geom_type[G])
		{
			case mjGEOM_CYLINDER:
				for (int32 i = 0; i < Sides; ++i)
				{
					const double A = 2.0 * UE_DOUBLE_PI * i / Sides;
					Out.Add(FVector(S[0] * FMath::Cos(A), S[0] * FMath::Sin(A), -S[1]));
					Out.Add(FVector(S[0] * FMath::Cos(A), S[0] * FMath::Sin(A), S[1]));
				}
				break;
			case mjGEOM_ELLIPSOID:
				for (int32 i = 0; i <= 8; ++i)
				{
					const double T = UE_DOUBLE_PI * i / 8;
					for (int32 k = 0; k < 16; ++k)
					{
						const double P = 2.0 * UE_DOUBLE_PI * k / 16;
						Out.Add(FVector(S[0] * FMath::Sin(T) * FMath::Cos(P), S[1] * FMath::Sin(T) * FMath::Sin(P), S[2] * FMath::Cos(T)));
					}
				}
				break;
			case mjGEOM_MESH:
			{
				const int32 Mesh = M->geom_dataid[G];
				const int32 Adr = M->mesh_vertadr[Mesh], Num = M->mesh_vertnum[Mesh];
				for (int32 v = 0; v < Num; ++v)
				{
					const float* P = M->mesh_vert + 3 * (Adr + v);
					Out.Add(FVector(P[0], P[1], P[2]));
				}
				break;
			}
			default:
				break;
		}
	}

	/** 3x3 helpers for combining welded bodies' inertia. */
	struct FMat3
	{
		double V[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		void   AddRotatedDiag(const FQuat& R, const FVector& D)
		{
			const FMatrix Rm = FQuatRotationMatrix(R);
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					for (int k = 0; k < 3; ++k)
						V[i][j] += Rm.M[k][i] * D[k] * Rm.M[k][j]; // FMatrix rows are the rotated axes
		}
		void AddPointMass(double Mass, const FVector& Dv)
		{
			const double D2 = Dv.SizeSquared();
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					V[i][j] += Mass * ((i == j ? D2 : 0.0) - Dv[i] * Dv[j]);
		}
	};
} // namespace

// ---------------------------------------------------------------------------------------------- //

URammsChaosArticulationComponent::URammsChaosArticulationComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

URammsChaosArticulationComponent* URammsChaosArticulationComponent::SwitchToChaos(AActor* Articulation, const FString& Keyframe)
{
	if (!Cast<AMjArticulation>(Articulation))
	{
		return nullptr;
	}
	if (URammsChaosArticulationComponent* Existing = Articulation->FindComponentByClass<URammsChaosArticulationComponent>())
	{
		return Existing;
	}
	URammsChaosArticulationComponent* Comp = NewObject<URammsChaosArticulationComponent>(Articulation, NAME_None, RF_Transient);
	Comp->InitialKeyframe = Keyframe;
	Articulation->AddInstanceComponent(Comp);
	Comp->RegisterComponent(); // BeginPlay runs here when the actor is already playing
	return Comp;
}

void URammsChaosArticulationComponent::BeginPlay()
{
	Super::BeginPlay();
	if (!bSimulateWithChaos)
	{
		return;
	}
	if (!Build(BuildError))
	{
		UE_LOG(LogRammsChaosArticulation, Error, TEXT("%s: Chaos build failed: %s"), *GetNameSafe(GetOwner()), *BuildError);
		Teardown();
	}
}

void URammsChaosArticulationComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Teardown();
	Super::EndPlay(EndPlayReason);
}

bool URammsChaosArticulationComponent::Build(FString& OutError)
{
	AMjArticulation* Art = Cast<AMjArticulation>(GetOwner());
	UWorld*			 World = GetWorld();
	if (Art == nullptr || World == nullptr || World->GetPhysicsScene() == nullptr)
	{
		OutError = TEXT("the owner is not an AMjArticulation in a world with physics");
		return false;
	}
	for (TActorIterator<AAMjManager> It(World); It; ++It)
	{
		UE_LOG(LogRammsChaosArticulation, Warning,
			TEXT("%s: the level also has an MuJoCo manager (%s), which will compile this articulation into MuJoCo and move its bodies too"),
			*Art->GetName(), *It->GetName());
		break;
	}

	// 1. Compile this articulation alone; MuJoCo is the model compiler here, never the stepper.
	MjSyncAssetFiles(Art->GetSpec());
	urlab::spec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(Art->GetSpec());
	urlab::spec::FMjCompiledScene Compiled = Builder.Compile();
	if (!Compiled.IsValid())
	{
		TArray<FString> Reasons;
		for (const FMjSpecDiagnostic& D : Compiled.Errors)
		{
			Reasons.Add(D.ToString());
		}
		OutError = FString::Printf(TEXT("the spec did not compile: %s"), *FString::Join(Reasons, TEXT("; ")));
		return false;
	}
	const mjModel* M = Compiled.Model;
	mjData*		   D = mj_makeData(M);
	ON_SCOPE_EXIT
	{
		mj_deleteData(D);
	};

	// Poses at q = 0 (the frames every joint is measured from), then at the starting keyframe.
	mj_resetData(M, D);
	mj_kinematics(M, D);
	TArray<FTransform> Pose0, PoseStart;
	for (int32 b = 0; b < M->nbody; ++b)
	{
		Pose0.Add(MjPoseToUe(D->xpos + 3 * b, D->xquat + 4 * b));
	}
	TArray<FVector> Anchor0, AxisHinge0, AxisSlide0;
	for (int32 j = 0; j < M->njnt; ++j)
	{
		Anchor0.Add(URLabAxisConv::MjPositionToUe(D->xanchor + 3 * j));
		AxisHinge0.Add(HingeAxisToUe(D->xaxis + 3 * j));
		AxisSlide0.Add(URLabAxisConv::MjDirectionToUe(D->xaxis + 3 * j).GetSafeNormal());
	}
	int32 StartKey = INDEX_NONE;
	for (int32 k = 0; k < M->nkey; ++k)
	{
		const FString KeyName = MjName(M, mjOBJ_KEY, k).ToString();
		TArray<float> Ctrl;
		for (int32 a = 0; a < M->nu; ++a)
		{
			Ctrl.Add(static_cast<float>(M->key_ctrl[k * M->nu + a]));
		}
		KeyframeCtrl.Add(KeyName, Ctrl);
		if (!InitialKeyframe.IsEmpty() && KeyName == InitialKeyframe)
		{
			StartKey = k;
		}
	}
	if (StartKey != INDEX_NONE)
	{
		mj_resetDataKeyframe(M, D, StartKey);
		mj_kinematics(M, D);
	}
	else if (!InitialKeyframe.IsEmpty())
	{
		UE_LOG(LogRammsChaosArticulation, Warning, TEXT("%s: no keyframe '%s'; starting at qpos0"), *Art->GetName(), *InitialKeyframe);
	}
	for (int32 b = 0; b < M->nbody; ++b)
	{
		PoseStart.Add(MjPoseToUe(D->xpos + 3 * b, D->xquat + 4 * b));
	}
	const FTransform ActorXf = Art->GetActorTransform();

	// 2. Which bodies are Chaos bodies. One with a joint (or a free joint) is; one without is welded
	// into its parent's. A body welded to a mocap body, or a mocap body itself, is held kinematic.
	TSet<int32> Pinned;
	for (int32 e = 0; e < M->neq; ++e)
	{
		if (M->eq_type[e] == mjEQ_WELD && M->eq_objtype[e] == mjOBJ_BODY)
		{
			const int32 A = M->eq_obj1id[e], B = M->eq_obj2id[e];
			if (A == 0 || M->body_mocapid[A] >= 0)
			{
				Pinned.Add(B);
			}
			if (B == 0 || M->body_mocapid[B] >= 0)
			{
				Pinned.Add(A);
			}
		}
	}
	MjBodies.SetNum(M->nbody);
	TArray<int32> RootOf;
	RootOf.Init(INDEX_NONE, M->nbody);
	TArray<int32> RootBody; // root index -> MuJoCo body id
	for (int32 b = 1; b < M->nbody; ++b)
	{
		MjBodies[b].Name = MjName(M, mjOBJ_BODY, b);
		const bool bOwn = M->body_jntnum[b] > 0 || M->body_parentid[b] == 0 || M->body_mocapid[b] >= 0;
		if (bOwn)
		{
			RootOf[b] = RootBody.Add(b);
		}
		else
		{
			RootOf[b] = RootOf[M->body_parentid[b]];
		}
		MjBodies[b].Root = RootOf[b];
		MjBodies[b].InRoot = Pose0[b].GetRelativeTransform(Pose0[RootBody[RootOf[b]]]);
	}
	for (const TPair<TObjectPtr<const UMjNodeComponent>, urlab::spec::FMjBoundElement>& Pair : Compiled.BoundIds)
	{
		if (Pair.Value.ObjType == mjOBJ_BODY && Pair.Value.Id > 0 && Pair.Value.Id < M->nbody)
		{
			MjBodies[Pair.Value.Id].Visual = const_cast<UMjNodeComponent*>(Pair.Key.Get());
		}
	}

	// 3. Collision, material and mass per Chaos body.
	for (int32 r = 0; r < RootBody.Num(); ++r)
	{
		const int32 RB = RootBody[r];
		const bool	bDynamic = M->body_jntnum[RB] > 0 && !Pinned.Contains(RB) && M->body_mocapid[RB] < 0;

		URammsChaosBodyComponent* Comp = NewObject<URammsChaosBodyComponent>(Art, *FString::Printf(TEXT("Chaos_%s"), *MjBodies[RB].Name.ToString()), RF_Transient);
		Comp->MuJoCoBody = MjBodies[RB].Name;
		UBodySetup* Setup = NewObject<UBodySetup>(Comp, NAME_None, RF_Transient);
		Setup->CollisionTraceFlag = CTF_UseSimpleAsComplex;
		Setup->bGenerateMirroredCollision = false;
		Setup->BodySetupGuid = FGuid::NewGuid();

		double	Friction = 0.0;
		double	Mass = 0.0;
		FVector Com = FVector::ZeroVector;
		struct FMember
		{
			double	Mass;
			FVector Com;
			FQuat	Rot;
			FVector I;
		};
		TArray<FMember> Members;
		for (int32 b = 1; b < M->nbody; ++b)
		{
			if (RootOf[b] != r)
			{
				continue;
			}
			const FTransform& InRoot = MjBodies[b].InRoot;
			for (int32 g = M->body_geomadr[b]; g >= 0 && g < M->body_geomadr[b] + M->body_geomnum[b]; ++g)
			{
				if (M->geom_contype[g] == 0 && M->geom_conaffinity[g] == 0)
				{
					continue;
				}
				const FTransform GeomXf = MjPoseToUe(M->geom_pos + 3 * g, M->geom_quat + 4 * g) * InRoot;
				const mjtNum*	 S = M->geom_size + 3 * g;
				Friction = FMath::Max(Friction, static_cast<double>(M->geom_friction[3 * g]));
				switch (M->geom_type[g])
				{
					case mjGEOM_SPHERE:
					{
						FKSphereElem E(S[0] * 100.0);
						E.Center = GeomXf.GetLocation();
						Setup->AggGeom.SphereElems.Add(E);
						break;
					}
					case mjGEOM_CAPSULE:
					{
						FKSphylElem E(S[0] * 100.0, 2.0 * S[1] * 100.0);
						E.Center = GeomXf.GetLocation();
						E.Rotation = GeomXf.Rotator();
						Setup->AggGeom.SphylElems.Add(E);
						break;
					}
					case mjGEOM_BOX:
					{
						FKBoxElem E(2.0 * S[0] * 100.0, 2.0 * S[1] * 100.0, 2.0 * S[2] * 100.0);
						E.Center = GeomXf.GetLocation();
						E.Rotation = GeomXf.Rotator();
						Setup->AggGeom.BoxElems.Add(E);
						break;
					}
					case mjGEOM_CYLINDER:
					case mjGEOM_ELLIPSOID:
					case mjGEOM_MESH:
					{
						TArray<FVector> Points;
						GeomPoints(M, g, CylinderSides, Points);
						if (Points.Num() < 4)
						{
							break;
						}
						FKConvexElem E;
						for (const FVector& P : Points)
						{
							const mjtNum  Pm[3] = { P.X, P.Y, P.Z };
							const FVector V = GeomXf.TransformPosition(URLabAxisConv::MjPositionToUe(Pm));
							E.VertexData.Add(V);
						}
						E.UpdateElemBox();
						Setup->AggGeom.ConvexElems.Add(MoveTemp(E));
						break;
					}
					default:
						break; // planes and height fields belong to the world
				}
			}
			if (M->body_mass[b] > 0.0)
			{
				FMember Mb;
				Mb.Mass = M->body_mass[b];
				Mb.Com = InRoot.TransformPosition(URLabAxisConv::MjPositionToUe(M->body_ipos + 3 * b));
				Mb.Rot = InRoot.GetRotation() * URLabAxisConv::MjQuatToUe(M->body_iquat + 4 * b);
				Mb.I = FVector(M->body_inertia[3 * b], M->body_inertia[3 * b + 1], M->body_inertia[3 * b + 2]) * 1.0e4; // kg.m^2 -> kg.cm^2
				Members.Add(Mb);
				Mass += Mb.Mass;
				Com += Mb.Com * Mb.Mass;
			}
		}
		// Convex elements are cooked from their points (in the editor through the derived-data cache, at
		// runtime by Chaos' runtime cooker); primitives need nothing.
		Setup->bNeverNeedsCookedCollisionData = false;
		Setup->CreatePhysicsMeshes();

		UPhysicalMaterial* Mat = NewObject<UPhysicalMaterial>(Comp, NAME_None, RF_Transient);
		Mat->Friction = Mat->StaticFriction = Friction > 0.0 ? Friction : 1.0;
		Mat->bOverrideFrictionCombineMode = true;
		Mat->FrictionCombineMode = EFrictionCombineMode::Max;
		Mat->Restitution = Restitution;
		Mat->bOverrideRestitutionCombineMode = true;
		Mat->RestitutionCombineMode = EFrictionCombineMode::Min;
		Materials.Add(Mat);
		Setup->PhysMaterial = Mat;
		Comp->ShapeSetup = Setup;

		Comp->SetMobility(EComponentMobility::Movable);
		Comp->SetWorldTransform(PoseStart[RB] * ActorXf);
		Comp->SetCollisionProfileName(bDynamic ? UCollisionProfile::PhysicsActor_ProfileName : FName(TEXT("BlockAllDynamic")));
		if (SelfCollision == ERammsChaosSelfCollision::Off)
		{
			Comp->SetCollisionObjectType(SelfCollisionChannel);
			Comp->SetCollisionResponseToChannel(SelfCollisionChannel, ECR_Ignore);
		}
		Comp->BodyInstance.bSimulatePhysics = bDynamic;
		Comp->BodyInstance.SetUseCCD(false);
		if (SolverPositionIterations > 0 || SolverVelocityIterations > 0)
		{
			// The island takes the most any of its bodies asks for.
			Comp->BodyInstance.SetOverrideIterationCounts(true);
			Comp->BodyInstance.SetPositionSolverIterationCount((uint8)FMath::Clamp(SolverPositionIterations > 0 ? SolverPositionIterations : 8, 1, 255));
			Comp->BodyInstance.SetVelocitySolverIterationCount((uint8)FMath::Clamp(SolverVelocityIterations > 0 ? SolverVelocityIterations : 1, 1, 255));
			Comp->BodyInstance.SetProjectionSolverIterationCount(1);
		}
		// A moving body with no collision geom still needs a shape to carry its mass: a small sphere that
		// collides with nothing.
		if (bDynamic && Setup->AggGeom.GetElementCount() == 0)
		{
			FKSphereElem Placeholder(1.0f);
			Placeholder.SetCollisionEnabled(ECollisionEnabled::NoCollision);
			Setup->AggGeom.SphereElems.Add(Placeholder);
		}
		Comp->RegisterComponent();
		Roots.Add(Comp);
		UE_LOG(LogRammsChaosArticulation, Log, TEXT("%s: body %s dynamic=%d simulating=%d spheres=%d capsules=%d boxes=%d convex=%d"),
			*Art->GetName(), *MjBodies[RB].Name.ToString(), bDynamic, Comp->IsSimulatingPhysics(), Setup->AggGeom.SphereElems.Num(),
			Setup->AggGeom.SphylElems.Num(), Setup->AggGeom.BoxElems.Num(), Setup->AggGeom.ConvexElems.Num());
		if (bDynamic && !Comp->IsSimulatingPhysics())
		{
			UE_LOG(LogRammsChaosArticulation, Warning, TEXT("%s: body %s did not start simulating"), *Art->GetName(), *MjBodies[RB].Name.ToString());
		}

		// The MJCF's mass, centre of mass and inertia, welded bodies combined (Chaos would derive them from
		// the shapes and a uniform density).
		if (bDynamic && Mass > 0.0)
		{
			Com /= Mass;
			FMat3 I;
			for (const FMember& Mb : Members)
			{
				I.AddRotatedDiag(Mb.Rot, Mb.I);
				I.AddPointMass(Mb.Mass, Mb.Com - Com);
			}
			// A hinge's armature (the reflected inertia of its motor and gearbox) is inertia about the joint
			// axis: add it to the body there. A wheel without it accelerates ~25% too fast.
			for (int32 j = M->body_jntadr[RB]; j >= 0 && j < M->body_jntadr[RB] + M->body_jntnum[RB]; ++j)
			{
				const double Armature = M->dof_armature[M->jnt_dofadr[j]];
				if (M->jnt_type[j] == mjJNT_HINGE && Armature > 0.0)
				{
					const FVector A = Pose0[RB].InverseTransformVectorNoScale(AxisHinge0[j]).GetSafeNormal();
					for (int i = 0; i < 3; ++i)
						for (int k = 0; k < 3; ++k)
							I.V[i][k] += Armature * 1.0e4 * A[i] * A[k];
				}
			}
			mjtNum Mat9[9], Eig[3], Vec[9], Q[4];
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					Mat9[3 * i + j] = I.V[i][j];
			mju_eig3(Eig, Vec, Q, Mat9);
			const float		 BodyMass = FMath::Max(static_cast<float>(Mass), MinBodyMass);
			const FVector	 Principal(FMath::Max(Eig[0], (double)MinBodyInertia), FMath::Max(Eig[1], (double)MinBodyInertia), FMath::Max(Eig[2], (double)MinBodyInertia));
			const FTransform ComPose(FQuat(Q[1], Q[2], Q[3], Q[0]).GetNormalized(), Com);
			FPhysicsCommand::ExecuteWrite(Comp->BodyInstance.GetPhysicsActor(), [&](const FPhysicsActorHandle& Actor) {
				FPhysicsInterface::SetMass_AssumesLocked(Actor, BodyMass);
				FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(Actor, Principal);
				FPhysicsInterface::SetComLocalPose_AssumesLocked(Actor, ComPose);
			});
		}
	}

	// 4. Joints. The parent frame sits at the anchor with X along the axis; the child frame is offset by
	// the middle of the range, so a symmetric Chaos limit covers an asymmetric MJCF range and the
	// constraint's own coordinate is q - mid.
	TArray<bool> SeenBody;
	SeenBody.Init(false, M->nbody);
	ExternalForce.Reset();
	for (int32 j = 0; j < M->njnt; ++j)
	{
		const int32 Type = M->jnt_type[j];
		const int32 B = M->jnt_bodyid[j];
		if (Type == mjJNT_FREE)
		{
			continue;
		}
		if (SeenBody[B])
		{
			UE_LOG(LogRammsChaosArticulation, Warning, TEXT("%s: body %s has more than one joint; %s is not simulated"),
				*Art->GetName(), *MjBodies[B].Name.ToString(), *MjName(M, mjOBJ_JOINT, j).ToString());
			continue;
		}
		SeenBody[B] = true;
		const int32 Child = RootOf[B];
		const int32 Parent = RootOf[M->body_parentid[B]]; // INDEX_NONE: the world
		FJoint		J;
		J.Name = MjName(M, mjOBJ_JOINT, j);
		J.Body = B;
		J.Kind = Type == mjJNT_HINGE ? EJointKind::Hinge : Type == mjJNT_SLIDE ? EJointKind::Slide
																			   : EJointKind::Ball;
		J.ParentRoot = Parent;
		J.ChildRoot = Child;
		const int32 Dof = M->jnt_dofadr[j];
		J.Damping = M->dof_damping[Dof];
		J.Stiffness = M->jnt_stiffness[j];
		J.SpringRef = M->qpos_spring[M->jnt_qposadr[j]];
		J.FrictionLoss = M->dof_frictionloss[Dof];
		const bool	 bLimited = M->jnt_limited[j] != 0;
		const double Lo = M->jnt_range[2 * j], Hi = M->jnt_range[2 * j + 1];
		J.Mid = (bLimited && J.Kind != EJointKind::Ball) ? 0.5 * (Lo + Hi) : 0.0;
		J.bContinuous = J.Kind == EJointKind::Hinge && !bLimited;

		const FTransform ParentWorld0 = Parent != INDEX_NONE ? Pose0[RootBody[Parent]] : FTransform::Identity;
		const FTransform ChildWorld0 = Pose0[RootBody[Child]];
		const FVector	 Axis = J.Kind == EJointKind::Slide ? AxisSlide0[j] : AxisHinge0[j];
		J.AxisInParent = ParentWorld0.InverseTransformVectorNoScale(Axis).GetSafeNormal();
		J.Rel0 = ParentWorld0.GetRotation().Inverse() * ChildWorld0.GetRotation();
		J.Origin0 = ParentWorld0.InverseTransformPosition(ChildWorld0.GetLocation());

		const FTransform F1 = AxisFrame(Anchor0[j], Axis);
		FTransform		 F2 = F1;
		if (J.Kind == EJointKind::Hinge)
		{
			F2.SetRotation(F1.GetRotation() * FQuat(FVector::XAxisVector, -J.Mid));
		}
		else if (J.Kind == EJointKind::Slide)
		{
			F2.SetLocation(Anchor0[j] - Axis * (J.Mid * 100.0));
		}

		TUniquePtr<FConstraintInstance> CI = MakeUnique<FConstraintInstance>();
		CI->SetDisableCollision(true);
		if (J.Kind == EJointKind::Hinge)
		{
			CI->SetLinearLimits(LCM_Locked, LCM_Locked, LCM_Locked, 0.0f);
			CI->SetAngularSwing1Limit(ACM_Locked, 0.0f);
			CI->SetAngularSwing2Limit(ACM_Locked, 0.0f);
			CI->SetAngularTwistLimit(bLimited ? ACM_Limited : ACM_Free, FMath::RadiansToDegrees(0.5 * (Hi - Lo)));
		}
		else if (J.Kind == EJointKind::Slide)
		{
			CI->SetLinearLimits(bLimited ? LCM_Limited : LCM_Free, LCM_Locked, LCM_Locked, 50.0 * (Hi - Lo)); // one size, shared by the axes
			CI->SetAngularSwing1Limit(ACM_Locked, 0.0f);
			CI->SetAngularSwing2Limit(ACM_Locked, 0.0f);
			CI->SetAngularTwistLimit(ACM_Locked, 0.0f);
		}
		else
		{
			CI->SetLinearLimits(LCM_Locked, LCM_Locked, LCM_Locked, 0.0f);
			const float Cone = bLimited ? FMath::RadiansToDegrees(Hi) : 0.0f;
			CI->SetAngularSwing1Limit(bLimited ? ACM_Limited : ACM_Free, Cone);
			CI->SetAngularSwing2Limit(bLimited ? ACM_Limited : ACM_Free, Cone);
			CI->SetAngularTwistLimit(bLimited ? ACM_Limited : ACM_Free, Cone);
		}
		CI->SetRefFrame(EConstraintFrame::Frame1, F1.GetRelativeTransform(ParentWorld0));
		CI->SetRefFrame(EConstraintFrame::Frame2, F2.GetRelativeTransform(ChildWorld0));
		FBodyInstance* P = Parent != INDEX_NONE ? &Roots[Parent]->BodyInstance : nullptr;
		CI->ProfileInstance.bEnableMassConditioning = bJointMassConditioning;
		CI->InitConstraint(P, &Roots[Child]->BodyInstance, 1.0f, Art);
		J.Constraint = Constraints.Add(MoveTemp(CI));
		JointByName.Add(J.Name, Joints.Add(J));
		ExternalForce.Add(0.0);
	}

	// 5. Equalities: a <connect> closes a loop with a point constraint; a <weld> between two simulated
	// bodies locks them; a weld to the world or a mocap body pinned its body above.
	for (int32 e = 0; e < M->neq; ++e)
	{
		if (M->eq_objtype[e] != mjOBJ_BODY && (M->eq_type[e] == mjEQ_CONNECT || M->eq_type[e] == mjEQ_WELD))
		{
			UE_LOG(LogRammsChaosArticulation, Warning, TEXT("%s: equality %d names sites, which is not supported"), *Art->GetName(), e);
			continue;
		}
		const int32 A = M->eq_obj1id[e], B = M->eq_obj2id[e];
		if (M->eq_type[e] == mjEQ_CONNECT || (M->eq_type[e] == mjEQ_WELD && A > 0 && B > 0 && M->body_mocapid[A] < 0 && M->body_mocapid[B] < 0))
		{
			const int32 RA = A > 0 ? RootOf[A] : INDEX_NONE, RB = B > 0 ? RootOf[B] : INDEX_NONE;
			if (RA == RB)
			{
				continue;
			}
			FVector AnchorWorld = Pose0[RA != INDEX_NONE ? RootBody[RA] : 0].GetLocation();
			if (M->eq_type[e] == mjEQ_CONNECT)
			{
				const mjtNum* Local = M->eq_data + mjNEQDATA * e;
				const FVector LocalUe = URLabAxisConv::MjPositionToUe(Local);
				AnchorWorld = Pose0[A].TransformPosition(LocalUe);
			}
			const FTransform				F(FQuat::Identity, AnchorWorld);
			TUniquePtr<FConstraintInstance> CI = MakeUnique<FConstraintInstance>();
			CI->SetDisableCollision(true);
			CI->SetLinearLimits(LCM_Locked, LCM_Locked, LCM_Locked, 0.0f);
			const EAngularConstraintMotion Ang = M->eq_type[e] == mjEQ_CONNECT ? ACM_Free : ACM_Locked;
			CI->SetAngularSwing1Limit(Ang, 0.0f);
			CI->SetAngularSwing2Limit(Ang, 0.0f);
			CI->SetAngularTwistLimit(Ang, 0.0f);
			const FTransform WA = RA != INDEX_NONE ? Pose0[RootBody[RA]] : FTransform::Identity;
			const FTransform WB = RB != INDEX_NONE ? Pose0[RootBody[RB]] : FTransform::Identity;
			CI->SetRefFrame(EConstraintFrame::Frame1, F.GetRelativeTransform(WA));
			CI->SetRefFrame(EConstraintFrame::Frame2, F.GetRelativeTransform(WB));
			CI->InitConstraint(RA != INDEX_NONE ? &Roots[RA]->BodyInstance : nullptr, RB != INDEX_NONE ? &Roots[RB]->BodyInstance : nullptr, 1.0f, Art);
			Constraints.Add(MoveTemp(CI));
		}
		else if (M->eq_type[e] != mjEQ_WELD)
		{
			UE_LOG(LogRammsChaosArticulation, Warning, TEXT("%s: equality %d (type %d) is not simulated on Chaos"), *Art->GetName(), e, (int32)M->eq_type[e]);
		}
	}

	// 6. Self-collision by MuJoCo's rules: a joint already keeps its two bodies apart; pairs that
	// <exclude> names, or whose geoms' contype/conaffinity can never meet, get a collision-only
	// constraint (every axis free) that just turns their contacts off.
	if (SelfCollision == ERammsChaosSelfCollision::MuJoCo && RootBody.Num() > 1)
	{
		TSet<uint64> Off;
		auto		 PairKey = [](int32 X, int32 Y) { return (uint64)FMath::Min(X, Y) << 32 | (uint32)FMath::Max(X, Y); };
		for (const FJoint& J : Joints)
		{
			if (J.ParentRoot != INDEX_NONE)
			{
				Off.Add(PairKey(J.ParentRoot, J.ChildRoot));
			}
		}
		TSet<uint64> Wanted;
		for (int32 x = 0; x < M->nexclude; ++x)
		{
			const int32 A = M->exclude_signature[x] >> 16, B = M->exclude_signature[x] & 0xFFFF;
			if (A > 0 && B > 0 && RootOf[A] != RootOf[B])
			{
				Wanted.Add(PairKey(RootOf[A], RootOf[B]));
			}
		}
		for (int32 ra = 0; ra < RootBody.Num(); ++ra)
		{
			for (int32 rb = ra + 1; rb < RootBody.Num(); ++rb)
			{
				bool bCanMeet = false;
				for (int32 ga = 0; ga < M->ngeom && !bCanMeet; ++ga)
				{
					if (RootOf[M->geom_bodyid[ga]] != ra)
						continue;
					for (int32 gb = 0; gb < M->ngeom && !bCanMeet; ++gb)
					{
						if (RootOf[M->geom_bodyid[gb]] != rb)
							continue;
						bCanMeet = (M->geom_contype[ga] & M->geom_conaffinity[gb]) || (M->geom_contype[gb] & M->geom_conaffinity[ga]);
					}
				}
				// MuJoCo also filters a body against its parent even without a joint between them.
				const int32 BA = RootBody[ra], BB = RootBody[rb];
				if (!bCanMeet || RootOf[M->body_parentid[BA]] == rb || RootOf[M->body_parentid[BB]] == ra)
				{
					Wanted.Add(PairKey(ra, rb));
				}
			}
		}
		// The usual robot MJCF (contype 1, conaffinity 0 on every part) lets no two parts touch. Then the
		// parts go on SelfCollisionChannel and ignore it, as SelfCollision Off does, instead of a
		// constraint per pair: hundreds of them on a mobile base, and with stiff drives they disturb it.
		const int32 Pairs = RootBody.Num() * (RootBody.Num() - 1) / 2;
		if (Wanted.Union(Off).Num() >= Pairs)
		{
			for (URammsChaosBodyComponent* Comp : Roots)
			{
				Comp->SetCollisionObjectType(SelfCollisionChannel);
				Comp->SetCollisionResponseToChannel(SelfCollisionChannel, ECR_Ignore);
			}
			Wanted.Reset();
			UE_LOG(LogRammsChaosArticulation, Log, TEXT("%s: no two parts can collide by MuJoCo's rules: they ignore each other on channel %d"),
				*Art->GetName(), (int32)SelfCollisionChannel.GetValue());
		}
		for (uint64 K : Wanted)
		{
			if (Off.Contains(K))
			{
				continue;
			}
			const int32						A = (int32)(K >> 32), B = (int32)(K & 0xFFFFFFFF);
			TUniquePtr<FConstraintInstance> CI = MakeUnique<FConstraintInstance>();
			CI->SetDisableCollision(true);
			CI->SetLinearLimits(LCM_Free, LCM_Free, LCM_Free, 0.0f);
			CI->SetAngularSwing1Limit(ACM_Free, 0.0f);
			CI->SetAngularSwing2Limit(ACM_Free, 0.0f);
			CI->SetAngularTwistLimit(ACM_Free, 0.0f);
			CI->InitConstraint(&Roots[A]->BodyInstance, &Roots[B]->BodyInstance, 1.0f, Art);
			Constraints.Add(MoveTemp(CI));
		}
	}

	// 7. Actuators.
	for (int32 a = 0; a < M->nu; ++a)
	{
		FActuator Act;
		Act.Name = MjName(M, mjOBJ_ACTUATOR, a);
		const mjtNum* Gp = M->actuator_gainprm + mjNGAIN * a;
		const mjtNum* Bp = M->actuator_biasprm + mjNBIAS * a;
		const double  Gear = M->actuator_gear[6 * a];
		Act.Gain = Gp[0];
		Act.Bias0 = Bp[0];
		if (M->actuator_biastype[a] == mjBIAS_AFFINE && Bp[1] < 0.0)
		{
			Act.Kind = EActuatorKind::PositionServo;
			Act.Kp = -Bp[1];
			Act.Kv = -Bp[2];
		}
		else if (M->actuator_biastype[a] == mjBIAS_AFFINE && Bp[2] < 0.0)
		{
			Act.Kind = EActuatorKind::VelocityServo;
			Act.Kv = -Bp[2];
		}
		Act.bCtrlLimited = M->actuator_ctrllimited[a] != 0;
		Act.CtrlMin = M->actuator_ctrlrange[2 * a];
		Act.CtrlMax = M->actuator_ctrlrange[2 * a + 1];
		Act.bForceLimited = M->actuator_forcelimited[a] != 0;
		Act.ForceMin = M->actuator_forcerange[2 * a];
		Act.ForceMax = M->actuator_forcerange[2 * a + 1];
		if (M->actuator_dyntype[a] != mjDYN_NONE)
		{
			UE_LOG(LogRammsChaosArticulation, Warning, TEXT("%s: actuator %s has activation dynamics, which Chaos ignores"), *Art->GetName(), *Act.Name.ToString());
		}
		const int32 Trn = M->actuator_trnid[2 * a];
		if (M->actuator_trntype[a] == mjTRN_JOINT)
		{
			if (const int32* J = JointByName.Find(MjName(M, mjOBJ_JOINT, Trn)))
			{
				Act.Joints.Add({ *J, Gear });
			}
		}
		else if (M->actuator_trntype[a] == mjTRN_TENDON && M->tendon_num[Trn] > 0 && M->wrap_type[M->tendon_adr[Trn]] == mjWRAP_JOINT)
		{
			for (int32 w = M->tendon_adr[Trn]; w < M->tendon_adr[Trn] + M->tendon_num[Trn]; ++w)
			{
				if (const int32* J = JointByName.Find(MjName(M, mjOBJ_JOINT, M->wrap_objid[w])))
				{
					Act.Joints.Add({ *J, Gear * M->wrap_prm[w] });
				}
			}
		}
		if (Act.Joints.Num() == 0)
		{
			UE_LOG(LogRammsChaosArticulation, Warning, TEXT("%s: actuator %s has a transmission Chaos does not drive"), *Art->GetName(), *Act.Name.ToString());
		}
		if (Act.Kind != EActuatorKind::Motor)
		{
			double Sum = 0.0;
			for (const TPair<int32, double>& JC : Act.Joints)
			{
				Sum += JC.Value;
			}
			for (const TPair<int32, double>& JC : Act.Joints)
			{
				const FJoint&		 J = Joints[JC.Key];
				FConstraintInstance& CI = *Constraints[J.Constraint];
				const double		 Share = JC.Value * Sum; // a tendon servo's stiffness, seen from each of its joints
				const double		 Kp = Act.Kp * Share, Kv = Act.Kv * Share;
				const double		 FMax = Act.bForceLimited ? FMath::Max(FMath::Abs(Act.ForceMin), FMath::Abs(Act.ForceMax)) * FMath::Abs(JC.Value) * ServoForceLimitScale : 0.0;
				const bool			 bPos = Act.Kind == EActuatorKind::PositionServo;
				if (J.Kind == EJointKind::Slide)
				{
					CI.SetLinearDriveAccelerationMode(false);
					CI.SetLinearPositionDrive(bPos, false, false);
					CI.SetLinearVelocityDrive(Kv > 0.0, false, false);
					CI.SetLinearDriveParams(Kp, Kv, FMax * 100.0);
				}
				else
				{
					CI.SetAngularDriveAccelerationMode(false);
					CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
					CI.SetOrientationDriveTwistAndSwing(bPos, false);
					CI.SetAngularVelocityDriveTwistAndSwing(Kv > 0.0, false);
					CI.SetAngularDriveParams(Kp * 1.0e4, Kv * 1.0e4, FMax * 1.0e4);
				}
			}
		}
		ActuatorByName.Add(Act.Name, Actuators.Add(Act));
	}

	// 8. The physics-rate callback.
	Chaos::FPhysicsSolver* Solver = World->GetPhysicsScene()->GetSolver();
	Callback = Solver->CreateAndRegisterSimCallbackObject_External<FRammsChaosArticulationCallback>();
	{
		FScopeLock Guard(&Callback->Lock);
		for (const FJoint& J : Joints)
		{
			FRammsChaosArticulationCallback::FPtJoint P;
			P.Child = Roots[J.ChildRoot]->BodyInstance.GetPhysicsActor();
			P.Parent = J.ParentRoot != INDEX_NONE ? Roots[J.ParentRoot]->BodyInstance.GetPhysicsActor() : nullptr;
			P.StaticParent = ActorXf;
			P.bHinge = J.Kind != EJointKind::Slide;
			P.AxisInParent = J.AxisInParent;
			P.Rel0 = J.Rel0;
			P.Origin0 = J.Origin0;
			const double U = P.bHinge ? 1.0e4 : 1.0; // N.m/rad -> kg.cm^2/s^2/rad;  N/m == kg/s^2
			P.Damping = J.Damping * U;
			P.Stiffness = J.Stiffness * U;
			P.SpringRef = P.bHinge ? J.SpringRef : J.SpringRef * 100.0;
			P.FrictionLoss = J.FrictionLoss * (P.bHinge ? 1.0e4 : 100.0);
			Callback->Joints.Add(P);
		}
	}

	// URLab's own components only show the robot: the Chaos bodies are its collision, for queries too.
	// Without a physics state of their own they also follow the bodies every frame at a fraction of the
	// cost (a query-only shape is moved in the physics scene each time).
	TArray<UPrimitiveComponent*> Prims;
	Art->GetComponents(Prims);
	for (UPrimitiveComponent* Prim : Prims)
	{
		if (!Prim->IsA<URammsChaosBodyComponent>() && Prim->GetCollisionEnabled() != ECollisionEnabled::NoCollision)
		{
			Prim->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		}
	}

	bBuilt = true;
	if (StartKey != INDEX_NONE)
	{
		ApplyKeyframeControls(InitialKeyframe);
	}
	else
	{
		for (int32 a = 0; a < Actuators.Num(); ++a)
		{
			ApplyActuator(a);
		}
	}
	for (const TPair<FName, float>& Pending : PendingCtrl)
	{
		if (!SetControl(Pending.Key, Pending.Value))
		{
			UE_LOG(LogRammsChaosArticulation, Warning, TEXT("%s: no actuator %s for a control set before the build"), *Art->GetName(), *Pending.Key.ToString());
		}
	}
	PendingCtrl.Reset();
	SyncVisuals();
	UE_LOG(LogRammsChaosArticulation, Log, TEXT("%s: built on Chaos: %d bodies (%d MuJoCo), %d joints, %d constraints, %d actuators"),
		*Art->GetName(), Roots.Num(), M->nbody - 1, Joints.Num(), Constraints.Num(), Actuators.Num());
	return true;
}

void URammsChaosArticulationComponent::Teardown()
{
	if (Callback)
	{
		if (UWorld* World = GetWorld(); World && World->GetPhysicsScene())
		{
			World->GetPhysicsScene()->GetSolver()->UnregisterAndFreeSimCallbackObject_External(Callback);
		}
		Callback = nullptr;
	}
	for (TUniquePtr<FConstraintInstance>& CI : Constraints)
	{
		CI->TermConstraint();
	}
	Constraints.Empty();
	for (URammsChaosBodyComponent* Comp : Roots)
	{
		if (IsValid(Comp))
		{
			Comp->DestroyComponent();
		}
	}
	Roots.Empty();
	Materials.Empty();
	MjBodies.Empty();
	Joints.Empty();
	Actuators.Empty();
	JointByName.Empty();
	ActuatorByName.Empty();
	ExternalForce.Empty();
	bBuilt = false;
}

// ---------------------------------------------------------------------------------------------- //

void URammsChaosArticulationComponent::WakeRoots(int32 A, int32 B)
{
	for (int32 R : { A, B })
	{
		if (Roots.IsValidIndex(R) && Roots[R]->IsSimulatingPhysics())
		{
			Roots[R]->WakeAllRigidBodies();
		}
	}
}

void URammsChaosArticulationComponent::ApplyActuator(int32 Index)
{
	const FActuator& Act = Actuators[Index];
	if (Act.Kind == EActuatorKind::Motor)
	{
		PushEfforts();
		return;
	}
	for (const TPair<int32, double>& JC : Act.Joints)
	{
		const FJoint&		 J = Joints[JC.Key];
		FConstraintInstance& CI = *Constraints[J.Constraint];
		if (Act.Kind == EActuatorKind::PositionServo)
		{
			// MuJoCo's servo force is gain*ctrl + bias0 - kp*q: it rests where that is zero.
			const double Target = (Act.Gain * Act.Ctrl + Act.Bias0) / Act.Kp;
			if (J.Kind == EJointKind::Slide)
			{
				CI.SetLinearPositionTarget(FVector((Target - J.Mid) * 100.0, 0.0, 0.0));
			}
			else
			{
				// The drive's target is the parent frame seen from the child's: the opposite sense to the
				// twist the limit measures.
				CI.SetAngularOrientationTarget(FQuat(FVector::XAxisVector, -(Target - J.Mid)));
			}
		}
		else
		{
			const double Rate = Act.Gain * Act.Ctrl / Act.Kv;
			if (J.Kind == EJointKind::Slide)
			{
				CI.SetLinearVelocityTarget(FVector(Rate * 100.0, 0.0, 0.0));
			}
			else
			{
				CI.SetAngularVelocityTarget(FVector(-Rate / (2.0 * UE_DOUBLE_PI), 0.0, 0.0)); // rev/s, same sense as the position target
			}
		}
		WakeRoots(J.ParentRoot, J.ChildRoot);
	}
}

void URammsChaosArticulationComponent::PushEfforts()
{
	if (!Callback)
	{
		return;
	}
	TArray<double> Effort = ExternalForce;
	for (const FActuator& Act : Actuators)
	{
		if (Act.Kind != EActuatorKind::Motor || Act.bVelocityDrive) // a speed loop applies its own
		{
			continue;
		}
		double F = Act.Gain * Act.Ctrl + Act.Bias0;
		if (Act.bForceLimited)
		{
			F = FMath::Clamp(F, Act.ForceMin, Act.ForceMax);
		}
		for (const TPair<int32, double>& JC : Act.Joints)
		{
			Effort[JC.Key] += F * JC.Value;
		}
	}
	FScopeLock Guard(&Callback->Lock);
	for (int32 j = 0; j < Joints.Num() && j < Callback->Joints.Num(); ++j)
	{
		const double U = Joints[j].Kind == EJointKind::Slide ? 100.0 : 1.0e4;
		const double E = Effort[j] * U;
		if (E != Callback->Joints[j].Effort)
		{
			Callback->Joints[j].Effort = E;
			WakeRoots(Joints[j].ParentRoot, Joints[j].ChildRoot);
		}
	}
}

bool URammsChaosArticulationComponent::SetControl(FName Actuator, float Value)
{
	if (!bBuilt)
	{
		if (bSimulateWithChaos && BuildError.IsEmpty())
		{
			PendingCtrl.Add(Actuator, Value);
			return true;
		}
		return false;
	}
	const int32* Index = ActuatorByName.Find(Actuator);
	if (!Index)
	{
		return false;
	}
	FActuator& Act = Actuators[*Index];
	Act.Ctrl = Act.bCtrlLimited ? FMath::Clamp(Value, (float)Act.CtrlMin, (float)Act.CtrlMax) : Value;
	if (Act.bReleased)
	{
		Act.bReleased = false;
		SetServoDrivesEnabled(*Index, true);
	}
	ApplyActuator(*Index);
	return true;
}

void URammsChaosArticulationComponent::SetServoDrivesEnabled(int32 Index, bool bEnabled)
{
	const FActuator& Act = Actuators[Index];
	const bool		 bPos = bEnabled && Act.Kind == EActuatorKind::PositionServo;
	const bool		 bVel = bEnabled && Act.Kv > 0.0;
	for (const TPair<int32, double>& JC : Act.Joints)
	{
		const FJoint&		 J = Joints[JC.Key];
		FConstraintInstance& CI = *Constraints[J.Constraint];
		if (J.Kind == EJointKind::Slide)
		{
			CI.SetLinearPositionDrive(bPos, false, false);
			CI.SetLinearVelocityDrive(bVel, false, false);
		}
		else
		{
			CI.SetOrientationDriveTwistAndSwing(bPos, false);
			CI.SetAngularVelocityDriveTwistAndSwing(bVel, false);
		}
		WakeRoots(J.ParentRoot, J.ChildRoot);
	}
}

bool URammsChaosArticulationComponent::ReleaseActuator(FName Actuator)
{
	const int32* Index = bBuilt ? ActuatorByName.Find(Actuator) : nullptr;
	if (!Index)
	{
		return false;
	}
	FActuator& Act = Actuators[*Index];
	if (Act.Kind == EActuatorKind::Motor)
	{
		ClearVelocityDrive(Actuator);
		Act.Ctrl = 0.0f;
		PushEfforts();
	}
	else if (!Act.bReleased)
	{
		Act.bReleased = true;
		SetServoDrivesEnabled(*Index, false);
	}
	return true;
}

bool URammsChaosArticulationComponent::SetVelocityDrive(FName Actuator, double TargetRate, double Kp, double Ki,
	double MaxIntegral, double OutMin, double OutMax, double RateScale)
{
	const int32* Index = bBuilt && Callback ? ActuatorByName.Find(Actuator) : nullptr;
	if (!Index)
	{
		return false;
	}
	FActuator& Act = Actuators[*Index];
	if (Act.Kind != EActuatorKind::Motor || Act.Joints.Num() == 0)
	{
		return false;
	}
	if (Act.bBraked)
	{
		SetMotorBrake(Actuator, false); // the loop and the brake share the joint's drive
	}
	const bool bStarting = !Act.bVelocityDrive;
	if (bStarting || Act.Loop.Kp != Kp || Act.Loop.Ki != Ki || Act.Loop.MaxIntegral != MaxIntegral)
	{
		Act.Loop.Integral = 0.0; // what the old gains accumulated means nothing under the new ones
		Act.Loop.Time = -1.0;
	}
	const bool bWake = bStarting || Act.Loop.Target != TargetRate || TargetRate != 0.0;
	Act.Loop.Target = TargetRate;
	Act.Loop.Kp = Kp;
	Act.Loop.Ki = Ki;
	Act.Loop.MaxIntegral = MaxIntegral;

	// A single-joint motor whose control pushes its joint forward runs on the joint constraint's velocity
	// drive, which the solver applies implicitly: an explicit loop at Chaos' step (10 ms or more, against
	// MuJoCo's 2) is unstable at the gains a wheel needs once the light link it hangs from is in the loop.
	// The drive is the proportional term; the integral, updated here once a frame, shifts its target, so
	// the drive's force limit clamps the sum as the base's loop clamps its output.
	const FJoint& J0 = Joints[Act.Joints[0].Key];
	const double  PerCtrl = Act.Gain * Act.Joints[0].Value; // joint force per unit of control
	Act.Loop.bImplicit = Act.Joints.Num() == 1 && PerCtrl > 0.0 && Kp > 0.0 && J0.Kind != EJointKind::Ball;
	if (Act.Loop.bImplicit)
	{
		{
			FScopeLock Guard(&Callback->Lock);
			Callback->Drives.Remove(Actuator);
		}
		double Q = 0.0, Qd = 0.0;
		ReadJoint(Act.Joints[0].Key, Q, Qd);
		const double Error = TargetRate - Qd * RateScale;
		const double Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
		const double Dt = Act.Loop.Time >= 0.0 ? FMath::Clamp(Now - Act.Loop.Time, 0.0, 0.1) : 0.0;
		Act.Loop.Time = Now;
		double Integral = Act.Loop.Integral + Ki * Error * Dt;
		if (MaxIntegral > 0.0)
		{
			Integral = FMath::Clamp(Integral, -MaxIntegral, MaxIntegral);
		}
		const double Output = Kp * Error + Integral;
		if (OutMin < OutMax && (Output < OutMin || Output > OutMax))
		{
			Integral = Act.Loop.Integral; // saturated: hold, do not wind up
		}
		Act.Loop.Integral = Integral;
		Act.Loop.LastCtrl = OutMin < OutMax ? FMath::Clamp(Output, OutMin, OutMax) : Output;

		double Limit = OutMin < OutMax ? FMath::Max(FMath::Abs(OutMin), FMath::Abs(OutMax)) * PerCtrl : 0.0;
		if (Act.bForceLimited)
		{
			const double Range = FMath::Max(FMath::Abs(Act.ForceMin), FMath::Abs(Act.ForceMax)) * Act.Joints[0].Value;
			Limit = Limit > 0.0 ? FMath::Min(Limit, Range) : Range;
		}
		const double		 Damping = Kp * RateScale * PerCtrl;			  // N.m per rad/s, or N per m/s
		const double		 Rate = (TargetRate + Integral / Kp) / RateScale; // MJCF units
		FConstraintInstance& CI = *Constraints[J0.Constraint];
		if (J0.Kind == EJointKind::Slide)
		{
			if (bStarting)
			{
				CI.SetLinearDriveAccelerationMode(false);
				CI.SetLinearPositionDrive(false, false, false);
				CI.SetLinearVelocityDrive(true, false, false);
			}
			CI.SetLinearDriveParams(0.0f, Damping, Limit > 0.0 ? Limit * 100.0 : 0.0);
			CI.SetLinearVelocityTarget(FVector(Rate * 100.0, 0.0, 0.0));
		}
		else
		{
			if (bStarting)
			{
				CI.SetAngularDriveAccelerationMode(false);
				CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
				CI.SetOrientationDriveTwistAndSwing(false, false);
				CI.SetAngularVelocityDriveTwistAndSwing(true, false);
			}
			CI.SetAngularDriveParams(0.0f, Damping * 1.0e4, Limit > 0.0 ? Limit * 1.0e4 : 0.0);
			CI.SetAngularVelocityTarget(FVector(-Rate / (2.0 * UE_DOUBLE_PI), 0.0, 0.0)); // rev/s, the drive's sense
		}
	}
	else
	{
		// Anything else (a tendon motor, a reversed gear, a pure integral loop) runs explicitly in the
		// physics-rate callback.
		FScopeLock								   Guard(&Callback->Lock);
		FRammsChaosArticulationCallback::FPtDrive& D = Callback->Drives.FindOrAdd(Actuator);
		if (D.Joint == INDEX_NONE)
		{
			D.Joint = Act.Joints[0].Key;
			D.Out = Act.Joints;
			D.Gain = Act.Gain;
			D.Bias0 = Act.Bias0;
			D.bForceLimited = Act.bForceLimited;
			D.ForceMin = Act.ForceMin;
			D.ForceMax = Act.ForceMax;
		}
		if (D.Kp != Kp || D.Ki != Ki || D.MaxIntegral != MaxIntegral)
		{
			D.Integral = 0.0;
		}
		D.RateScale = RateScale;
		D.Target = TargetRate;
		D.Kp = Kp;
		D.Ki = Ki;
		D.MaxIntegral = MaxIntegral;
		D.OutMin = OutMin;
		D.OutMax = OutMax;
	}
	if (bStarting)
	{
		Act.bVelocityDrive = true;
		PushEfforts(); // its control no longer counts: the loop's does
	}
	if (bWake)
	{
		for (const TPair<int32, double>& JC : Act.Joints)
		{
			WakeRoots(Joints[JC.Key].ParentRoot, Joints[JC.Key].ChildRoot);
		}
	}
	return true;
}

void URammsChaosArticulationComponent::ClearVelocityDrive(FName Actuator)
{
	const int32* Index = ActuatorByName.Find(Actuator);
	if (!Index || !Callback)
	{
		return;
	}
	FActuator& Act = Actuators[*Index];
	double	   Last = Act.Loop.bImplicit ? Act.Loop.LastCtrl : Act.Ctrl;
	{
		FScopeLock Guard(&Callback->Lock);
		if (const FRammsChaosArticulationCallback::FPtDrive* D = Callback->Drives.Find(Actuator))
		{
			Last = D->LastCtrl;
			Callback->Drives.Remove(Actuator);
		}
	}
	if (Act.bVelocityDrive)
	{
		for (const TPair<int32, double>& JC : Act.Joints)
		{
			FConstraintInstance& CI = *Constraints[Joints[JC.Key].Constraint];
			if (Joints[JC.Key].Kind == EJointKind::Slide)
			{
				CI.SetLinearVelocityDrive(false, false, false);
			}
			else
			{
				CI.SetAngularVelocityDriveTwistAndSwing(false, false);
			}
		}
		// The motor keeps what the loop last asked of it, as a MuJoCo actuator keeps its staged control.
		Act.bVelocityDrive = false;
		Act.Loop.bImplicit = false;
		Act.Ctrl = (float)Last;
		PushEfforts();
	}
}

double URammsChaosArticulationComponent::MotorTorqueLimit(const FActuator& Act) const
{
	const double Coef = Act.Joints.Num() > 0 ? FMath::Abs(Act.Joints[0].Value) : 1.0;
	if (Act.bForceLimited)
	{
		return FMath::Max(FMath::Abs(Act.ForceMin), FMath::Abs(Act.ForceMax)) * Coef;
	}
	if (Act.bCtrlLimited)
	{
		return FMath::Max(FMath::Abs(Act.CtrlMin), FMath::Abs(Act.CtrlMax)) * FMath::Abs(Act.Gain) * Coef;
	}
	return 0.0;
}

bool URammsChaosArticulationComponent::SetMotorBrake(FName Actuator, bool bEngaged)
{
	const int32* Index = bBuilt ? ActuatorByName.Find(Actuator) : nullptr;
	if (!Index || Actuators[*Index].Kind != EActuatorKind::Motor || Actuators[*Index].Joints.Num() != 1)
	{
		return false;
	}
	FActuator& Act = Actuators[*Index];
	if (bEngaged == Act.bBraked)
	{
		return true; // an engaged brake keeps the angle it was set at
	}
	const FJoint&		 J = Joints[Act.Joints[0].Key];
	FConstraintInstance& CI = *Constraints[J.Constraint];
	if (J.Kind == EJointKind::Ball)
	{
		return false;
	}
	if (bEngaged)
	{
		ClearVelocityDrive(Actuator);
		// Full torque about a degree (or a millimetre) off, critically damped-ish; held by the solver, so
		// stable at any stiffness. The limit is what the motor can do: a steeper slope rolls the robot.
		double		 Q = 0.0, Qd = 0.0;
		const double Limit = MotorTorqueLimit(Act);
		ReadJoint(Act.Joints[0].Key, Q, Qd);
		const double Most = Limit > 0.0 ? Limit : 1000.0;
		if (J.Kind == EJointKind::Slide)
		{
			CI.SetLinearDriveAccelerationMode(false);
			CI.SetLinearPositionDrive(true, false, false);
			CI.SetLinearVelocityDrive(true, false, false);
			CI.SetLinearDriveParams(Most / 0.1, Most / 1.0, Limit > 0.0 ? Limit * 100.0 : 0.0); // per cm, per cm/s
			CI.SetLinearPositionTarget(FVector((Q - J.Mid) * 100.0, 0.0, 0.0));
			CI.SetLinearVelocityTarget(FVector::ZeroVector);
		}
		else
		{
			CI.SetAngularDriveAccelerationMode(false);
			CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
			CI.SetOrientationDriveTwistAndSwing(true, false);
			CI.SetAngularVelocityDriveTwistAndSwing(true, false);
			CI.SetAngularDriveParams(Most / 0.02 * 1.0e4, Most / 0.2 * 1.0e4, Limit > 0.0 ? Limit * 1.0e4 : 0.0);
			CI.SetAngularOrientationTarget(FQuat(FVector::XAxisVector, -(Q - J.Mid)));
			CI.SetAngularVelocityTarget(FVector::ZeroVector);
		}
	}
	else if (J.Kind == EJointKind::Slide)
	{
		CI.SetLinearPositionDrive(false, false, false);
		CI.SetLinearVelocityDrive(false, false, false);
	}
	else
	{
		CI.SetOrientationDriveTwistAndSwing(false, false);
		CI.SetAngularVelocityDriveTwistAndSwing(false, false);
	}
	Act.bBraked = bEngaged;
	WakeRoots(J.ParentRoot, J.ChildRoot);
	return true;
}

bool URammsChaosArticulationComponent::IsMotorBraked(FName Actuator) const
{
	const int32* Index = ActuatorByName.Find(Actuator);
	return Index && Actuators[*Index].bBraked;
}

FName URammsChaosArticulationComponent::GetActuatorJoint(FName Actuator) const
{
	const int32* Index = ActuatorByName.Find(Actuator);
	return (Index && Actuators[*Index].Joints.Num() > 0) ? Joints[Actuators[*Index].Joints[0].Key].Name : NAME_None;
}

bool URammsChaosArticulationComponent::GetJointBodyTransform(FName Joint, FTransform& OutWorld) const
{
	const int32* Index = JointByName.Find(Joint);
	if (!Index || !MjBodies.IsValidIndex(Joints[*Index].Body))
	{
		return false;
	}
	const FMjBodyInfo& B = MjBodies[Joints[*Index].Body];
	if (!Roots.IsValidIndex(B.Root) || !Roots[B.Root])
	{
		return false;
	}
	OutWorld = B.InRoot * Roots[B.Root]->GetComponentTransform();
	return true;
}

float URammsChaosArticulationComponent::GetControl(FName Actuator) const
{
	const int32* Index = ActuatorByName.Find(Actuator);
	return Index ? Actuators[*Index].Ctrl : 0.0f;
}

bool URammsChaosArticulationComponent::SetJointForce(FName Joint, float Force)
{
	const int32* Index = JointByName.Find(Joint);
	if (!Index)
	{
		return false;
	}
	ExternalForce[*Index] = Force;
	PushEfforts();
	return true;
}

bool URammsChaosArticulationComponent::ApplyKeyframeControls(const FString& Keyframe)
{
	const TArray<float>* Ctrl = KeyframeCtrl.Find(Keyframe);
	if (!Ctrl || Ctrl->Num() != Actuators.Num())
	{
		return false;
	}
	for (int32 a = 0; a < Actuators.Num(); ++a)
	{
		Actuators[a].Ctrl = (*Ctrl)[a];
		ApplyActuator(a);
	}
	return true;
}

TArray<FString> URammsChaosArticulationComponent::GetKeyframeNames() const
{
	TArray<FString> Out;
	KeyframeCtrl.GetKeys(Out);
	return Out;
}

TArray<float> URammsChaosArticulationComponent::GetKeyframeControls(const FString& Keyframe) const
{
	const TArray<float>* Ctrl = KeyframeCtrl.Find(Keyframe);
	return Ctrl ? *Ctrl : TArray<float>();
}

bool URammsChaosArticulationComponent::GetControlRange(FName Actuator, float& OutMin, float& OutMax) const
{
	const int32* Index = ActuatorByName.Find(Actuator);
	if (!Index || !Actuators[*Index].bCtrlLimited)
	{
		OutMin = OutMax = 0.0f;
		return false;
	}
	OutMin = (float)Actuators[*Index].CtrlMin;
	OutMax = (float)Actuators[*Index].CtrlMax;
	return true;
}

bool URammsChaosArticulationComponent::IsSlide(FName Joint) const
{
	const int32* Index = JointByName.Find(Joint);
	return Index && Joints[*Index].Kind == EJointKind::Slide;
}

bool URammsChaosArticulationComponent::IsActuated(FName Joint) const
{
	const int32* Index = JointByName.Find(Joint);
	if (!Index)
	{
		return false;
	}
	for (const FActuator& Act : Actuators)
	{
		for (const TPair<int32, double>& JC : Act.Joints)
		{
			if (JC.Key == *Index)
			{
				return true;
			}
		}
	}
	return false;
}

bool URammsChaosArticulationComponent::IsMotor(FName Actuator) const
{
	const int32* Index = ActuatorByName.Find(Actuator);
	return Index && Actuators[*Index].Kind == EActuatorKind::Motor;
}

bool URammsChaosArticulationComponent::MeasureJoint(const FJoint& J, double& OutQ, double& OutQd) const
{
	if (!Roots.IsValidIndex(J.ChildRoot))
	{
		return false;
	}
	const URammsChaosBodyComponent* C = Roots[J.ChildRoot];
	const URammsChaosBodyComponent* P = Roots.IsValidIndex(J.ParentRoot) ? Roots[J.ParentRoot].Get() : nullptr;
	const FTransform				Xc = C->GetComponentTransform();
	const FTransform				Xp = P ? P->GetComponentTransform() : GetOwner()->GetActorTransform();
	const FVector					Axis = Xp.TransformVectorNoScale(J.AxisInParent);
	const FVector					Wp = P ? P->GetPhysicsAngularVelocityInRadians() : FVector::ZeroVector;
	const FVector					Vp = P ? P->GetPhysicsLinearVelocity() : FVector::ZeroVector;
	if (J.Kind == EJointKind::Slide)
	{
		OutQ = ((Xp.InverseTransformPosition(Xc.GetLocation()) - J.Origin0) | J.AxisInParent) / 100.0;
		OutQd = ((C->GetPhysicsLinearVelocity() - Vp) | Axis) / 100.0;
	}
	else
	{
		const FQuat Dq = (Xp.GetRotation().Inverse() * Xc.GetRotation()) * J.Rel0.Inverse();
		OutQ = FMath::UnwindRadians(2.0 * FMath::Atan2(FVector(Dq.X, Dq.Y, Dq.Z) | J.AxisInParent, Dq.W));
		OutQd = (C->GetPhysicsAngularVelocityInRadians() - Wp) | Axis;
	}
	return true;
}

bool URammsChaosArticulationComponent::ReadJoint(int32 Index, double& OutQ, double& OutQd) const
{
	if (!Joints.IsValidIndex(Index))
	{
		return false;
	}
	const FJoint& J = Joints[Index];
	if (Callback)
	{
		FScopeLock Guard(&Callback->Lock);
		if (Callback->Joints.IsValidIndex(Index) && Callback->Joints[Index].bMeasured)
		{
			const FRammsChaosArticulationCallback::FPtJoint& P = Callback->Joints[Index];
			const double									 U = J.Kind == EJointKind::Slide ? 0.01 : 1.0; // cm -> m
			OutQ = (J.bContinuous ? P.Counted : P.Q) * U;
			OutQd = P.Qd * U;
			return true;
		}
	}
	return MeasureJoint(J, OutQ, OutQd); // before the first step
}

float URammsChaosArticulationComponent::GetJointPosition(FName Joint) const
{
	double		 Q = 0.0, Qd = 0.0;
	const int32* Index = JointByName.Find(Joint);
	return (Index && ReadJoint(*Index, Q, Qd)) ? (float)Q : 0.0f;
}

float URammsChaosArticulationComponent::GetJointVelocity(FName Joint) const
{
	double		 Q = 0.0, Qd = 0.0;
	const int32* Index = JointByName.Find(Joint);
	return (Index && ReadJoint(*Index, Q, Qd)) ? (float)Qd : 0.0f;
}

TArray<FName> URammsChaosArticulationComponent::GetJointNames() const
{
	TArray<FName> Out;
	for (const FJoint& J : Joints)
	{
		Out.Add(J.Name);
	}
	return Out;
}

TArray<FName> URammsChaosArticulationComponent::GetActuatorNames() const
{
	TArray<FName> Out;
	for (const FActuator& A : Actuators)
	{
		Out.Add(A.Name);
	}
	return Out;
}

URammsChaosBodyComponent* URammsChaosArticulationComponent::GetBodyComponent(FName MuJoCoBody) const
{
	for (const FMjBodyInfo& B : MjBodies)
	{
		if (B.Name == MuJoCoBody && Roots.IsValidIndex(B.Root))
		{
			return Roots[B.Root];
		}
	}
	return nullptr;
}

TArray<URammsChaosBodyComponent*> URammsChaosArticulationComponent::GetBodyComponents() const
{
	TArray<URammsChaosBodyComponent*> Out;
	for (const TObjectPtr<URammsChaosBodyComponent>& R : Roots)
	{
		Out.Add(R);
	}
	return Out;
}

void URammsChaosArticulationComponent::SyncVisuals()
{
	// Bodies are in MuJoCo order, parents before children, so each component is placed after the one
	// it is attached to.
	for (const FMjBodyInfo& B : MjBodies)
	{
		USceneComponent* V = B.Visual.Get();
		if (V && Roots.IsValidIndex(B.Root))
		{
			V->SetWorldTransform(B.InRoot * Roots[B.Root]->GetComponentTransform(), false, nullptr, ETeleportType::TeleportPhysics);
		}
	}
}

void URammsChaosArticulationComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (bBuilt)
	{
		SyncVisuals();
	}
}
