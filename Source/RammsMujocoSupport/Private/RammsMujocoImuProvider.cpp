// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMujocoImuProvider.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "PhysicsEngine/BodySetup.h"
#include "RammsChaosArticulationComponent.h"
#include "RammsImuMath.h"
#include "mujoco/mujoco.h"

static_assert(sizeof(mjtNum) == sizeof(double), "the IMU provider copies qvel as doubles");

static TAutoConsoleVariable<int32> CVarRammsImuMuJoCoAcceleration(TEXT("ramms.Imu.MuJoCoAcceleration"), 0,
	TEXT("The acceleration MuJoCo IMU samples report. 0 (default): realized, what the integrator applied over the step, ")
		TEXT("(qvel after - qvel before) / timestep, like Chaos. 1: explicit, the forward pass's qacc, what MuJoCo's own ")
			TEXT("accelerometer reads (not what implicit / implicitfast applied under stiff damping)."));

TSharedPtr<IRammsImuProvider> MakeRammsMujocoImuProvider(UWorld& World)
{
	return MakeShared<FRammsMujocoImuProvider>(World);
}

FRammsMujocoImuProvider::FRammsMujocoImuProvider(UWorld& InWorld)
	: World(&InWorld)
	, Table(MakeShared<FTable, ESPMode::ThreadSafe>())
{
	Table->Acceleration = AccelerationSetting();
}

FRammsMujocoImuProvider::~FRammsMujocoImuProvider()
{
	// The engines keep calling back (URLab has no unregister): the callbacks find the table dead and return.
	FScopeLock Guard(&Table->Lock);
	Table->bAlive = false;
	Table->Points.Reset();
	Table->Models.Reset();
}

// ---------------------------------------------------------------------------
// Game thread
// ---------------------------------------------------------------------------

ERammsMjImuAcceleration FRammsMujocoImuProvider::AccelerationSetting()
{
	return CVarRammsImuMuJoCoAcceleration.GetValueOnGameThread() == 1 ? ERammsMjImuAcceleration::Explicit : ERammsMjImuAcceleration::Realized;
}

UMjPhysicsEngine* FRammsMujocoImuProvider::EngineFor(const UMjBody& Body)
{
	// URLab's singleton manager's engine, when it is this world's (another world's -- a PIE session next to an
	// editor world -- is not); else the first manager in this world.
	UWorld* BodyWorld = Body.GetWorld();
	if (UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(&Body); Engine && Engine->GetWorld() == BodyWorld)
	{
		return Engine;
	}
	if (BodyWorld)
	{
		for (TActorIterator<AAMjManager> It(BodyWorld); It; ++It)
		{
			if (It->PhysicsEngine)
			{
				return It->PhysicsEngine;
			}
		}
	}
	return nullptr;
}

FString FRammsMujocoImuProvider::BodyNameOf(const UMjBody& Body)
{
	return Body.MjName.IsSet() && !Body.MjName.GetValue().IsEmpty() ? Body.MjName.GetValue() : Body.GetName();
}

FVector FRammsMujocoImuProvider::QuickConvertPivot(const UMjBody& Body)
{
	// UMjBody::RefreshMeshPivotOffset's rule (its result is private): the first child static mesh's collision bounds
	// centre, in the body's local frame.
	TArray<USceneComponent*> Children;
	Body.GetChildrenComponents(/*bIncludeAllDescendants=*/true, Children);
	for (const USceneComponent* Child : Children)
	{
		const UStaticMeshComponent* Mesh = Cast<UStaticMeshComponent>(Child);
		const UBodySetup*			Setup = Mesh && Mesh->GetStaticMesh() ? Mesh->GetStaticMesh()->GetBodySetup() : nullptr;
		if (Setup)
		{
			return Setup->AggGeom.CalcAABB(FTransform::Identity).GetCenter();
		}
	}
	return FVector::ZeroVector;
}

FRammsMujocoImuProvider::EBodyScene FRammsMujocoImuProvider::SceneOf(const UMjBody& Body)
{
	const UMjPhysicsEngine* Engine = EngineFor(Body);
	if (!Engine)
	{
		return EBodyScene::NoScene;
	}
	const mjModel* Model = Engine->GetModel();
	if (!Model)
	{
		// Not compiled yet, or its compile failed: nothing steps.
		return Body.GetMocap() ? EBodyScene::Posed : EBodyScene::NotCompiled;
	}
	const TOptional<int32>& Bound = Body.GetBoundId();
	if (!Bound.IsSet() || Bound.GetValue() < 0 || Bound.GetValue() >= Model->nbody)
	{
		return EBodyScene::LeftOut; // the compiled scene has no such body (a recompile left it out)
	}
	return IsMocapDriven(Model, Bound.GetValue()) ? EBodyScene::Posed : EBodyScene::Simulated;
}

bool FRammsMujocoImuProvider::ResolveOnBody(UMjBody& Body, USceneComponent& Component, FName Bone, const FTransform& LocalPose, int32 Depth,
	bool bListing, FRammsImuBodyRef& OutRef)
{
	const FTransform SensorWorld = LocalPose * RammsImuProviders::ComponentFrame(Component, Bone);
	FTransform		 BodyFrame = Body.GetComponentTransform();
	BodyFrame.SetScale3D(FVector::OneVector);
	const FTransform PointInComponent = SensorWorld.GetRelativeTransform(BodyFrame);

	OutRef.Bone = NAME_None;
	OutRef.BodyName = BodyNameOf(Body);
	OutRef.Depth = Depth;

	// A URLab robot that Chaos simulates: the Chaos body that carries this one serves the point.
	const AActor* Owner = Body.GetOwner();
	if (const URammsChaosArticulationComponent* OnChaos = Owner ? Owner->FindComponentByClass<URammsChaosArticulationComponent>() : nullptr;
		OnChaos && OnChaos->IsBuilt())
	{
		URammsChaosBodyComponent* ChaosBody = nullptr;
		FTransform				  InChaosBody;
		if (!OnChaos->FindChaosBody(&Body, ChaosBody, InChaosBody))
		{
			return false;
		}
		OutRef.Source = ERammsImuSource::Chaos;
		OutRef.Body = ChaosBody;
		OutRef.PointInBody = PointInComponent * InChaosBody;
		return true;
	}

	// Else MuJoCo, only while its compiled scene simulates the body: until the scene compiles (or when its compile
	// failed), and once a recompile leaves the body out, another engine or Kinematic serves the point (the subsystem
	// asks again every second). A listing names a body of a scene not compiled yet too. A body Unreal poses (a mocap
	// body, or one welded to it) MuJoCo reports as still: Kinematic serves it.
	const EBodyScene Scene = SceneOf(Body);
	if (!(Scene == EBodyScene::Simulated || (bListing && Scene == EBodyScene::NotCompiled)))
	{
		return false;
	}
	OutRef.Source = ERammsImuSource::MuJoCo;
	OutRef.Body = &Body;
	OutRef.PointInBody = PointInComponent;
	return true;
}

bool FRammsMujocoImuProvider::Resolve(USceneComponent& Component, FName Bone, const FTransform& LocalPose, bool /*bExplicit*/,
	FRammsImuBodyRef& OutRef) const
{
	// The nearest URLab body up the attach chain (Component included).
	int32	 Depth = 0;
	UMjBody* Body = nullptr;
	for (USceneComponent* Node = &Component; Node && !Body; Node = Node->GetAttachParent())
	{
		Body = Cast<UMjBody>(Node);
		Depth += Body ? 0 : 1;
	}
	return Body && ResolveOnBody(*Body, Component, Bone, LocalPose, Depth, /*bListing=*/false, OutRef);
}

int32 FRammsMujocoImuProvider::Track(const FRammsImuBodyRef& Ref)
{
	UMjBody* Body = Cast<UMjBody>(Ref.Body.Get());
	if (!Body || Ref.Source != ERammsImuSource::MuJoCo)
	{
		return INDEX_NONE;
	}
	const int32 Handle = NextHandle++;
	FTracked&	Entry = Tracked.Add(Handle);
	Entry.Body = Body;
	Entry.PointInComponent = Ref.PointInBody;
	{
		FScopeLock Guard(&Table->Lock);
		Table->Points.Add(Handle);
	}
	Refresh(Handle, Entry, /*bForce=*/true);
	return Handle;
}

bool FRammsMujocoImuProvider::GetSample(int32 Handle, FRammsImuSample& OutSample)
{
	FTracked* Entry = Tracked.Find(Handle);
	if (!Entry)
	{
		return false;
	}
	Refresh(Handle, *Entry, /*bForce=*/false);
	FScopeLock	  Guard(&Table->Lock);
	const FPoint* Point = Table->Points.Find(Handle);
	if (!Point || !Point->Latest.bValid)
	{
		return false;
	}
	OutSample = Point->Latest;
	return true;
}

void FRammsMujocoImuProvider::Untrack(int32 Handle)
{
	Tracked.Remove(Handle);
	FScopeLock Guard(&Table->Lock);
	Table->Points.Remove(Handle);
}

bool FRammsMujocoImuProvider::IsTrackValid(int32 Handle) const
{
	// The body is gone, Chaos now simulates the robot, or the compiled scene no longer simulates the body (the scene
	// is gone, a recompile left the body out or failed, Unreal poses it): another provider serves the point (the
	// subsystem resolves it again).
	const FTracked* Entry = Tracked.Find(Handle);
	const UMjBody*	Body = Entry ? Entry->Body.Get() : nullptr;
	if (!Body)
	{
		return false;
	}
	const AActor* Owner = Body->GetOwner();
	if (const URammsChaosArticulationComponent* OnChaos = Owner ? Owner->FindComponentByClass<URammsChaosArticulationComponent>() : nullptr;
		OnChaos && OnChaos->IsBuilt())
	{
		return false;
	}
	return SceneOf(*Body) == EBodyScene::Simulated;
}

void FRammsMujocoImuProvider::SetHistoryCapacity(int32 Handle, int32 Capacity)
{
	// Sized here, on the game thread; the physics thread only writes into it.
	FScopeLock Guard(&Table->Lock);
	if (FPoint* Point = Table->Points.Find(Handle))
	{
		Point->History.SetCapacity(Capacity);
	}
}

bool FRammsMujocoImuProvider::ReadHistory(int32 Handle, int64 Since, int32 Max, TArray<FRammsImuSample>& OutSamples, int64& OutFirst,
	int64& OutOldest, int64& OutNext)
{
	// A recompile is noticed here as well as on GetSample: a point only read through its history follows it too.
	if (FTracked* Entry = Tracked.Find(Handle))
	{
		Refresh(Handle, *Entry, /*bForce=*/false);
	}
	FScopeLock	  Guard(&Table->Lock);
	const FPoint* Point = Table->Points.Find(Handle);
	if (!Point || Point->History.GetCapacity() == 0)
	{
		return false;
	}
	OutFirst = Point->History.Read(Since, Max, OutSamples);
	OutOldest = Point->History.GetOldestSeq();
	OutNext = Point->History.GetNextSeq();
	return true;
}

void FRammsMujocoImuProvider::Refresh(int32 Handle, FTracked& Entry, bool bForce)
{
	// The compile runs on the game thread, so the engine's model pointer and the body's bound id read here are the
	// installed ones.
	UMjBody*		  Body = Entry.Body.Get();
	UMjPhysicsEngine* Engine = Body ? EngineFor(*Body) : nullptr;
	const mjModel*	  Model = Engine ? Engine->GetModel() : nullptr;
	int32			  BodyId = INDEX_NONE;
	if (Body && Model)
	{
		const TOptional<int32>& Bound = Body->GetBoundId();
		if (Bound.IsSet() && Bound.GetValue() >= 0 && Bound.GetValue() < Model->nbody && !IsMocapDriven(Model, Bound.GetValue()))
		{
			BodyId = Bound.GetValue();
		}
	}
	if (Engine && BodyId != INDEX_NONE)
	{
		Hook(*Engine);
	}
	const ERammsMjImuAcceleration Acceleration = AccelerationSetting();
	if (!bForce && Model == Entry.Model && BodyId == Entry.BodyId && Engine == Entry.Engine.Get())
	{
		if (Table->Acceleration != Acceleration)
		{
			FScopeLock Guard(&Table->Lock);
			Table->Acceleration = Acceleration;
		}
		return;
	}
	Entry.Engine = Engine;
	Entry.Model = Model;
	Entry.BodyId = BodyId;

	FTransform PointInMjBody = Entry.PointInComponent;
	if (Body && BodyId != INDEX_NONE && Body->bIsQuickConverted)
	{
		// UMjBody::ApplyRenderState places a converted body's component at its MuJoCo frame less its mesh pivot
		// offset: the MuJoCo frame is that far along the component's axes.
		PointInMjBody.SetLocation(PointInMjBody.GetLocation() - QuickConvertPivot(*Body));
	}

	FScopeLock Guard(&Table->Lock);
	Table->Acceleration = Acceleration;
	if (FPoint* Point = Table->Points.Find(Handle))
	{
		Point->Model = Model;
		Point->NBody = Model ? Model->nbody : 0;
		Point->BodyId = BodyId;
		Point->PointInBody = PointInMjBody;
		Point->Latest = FRammsImuSample(); // a sample of another model's body: the next step makes a new one
	}
}

void FRammsMujocoImuProvider::Hook(UMjPhysicsEngine& Engine)
{
	for (auto It = Hooked.CreateIterator(); It; ++It)
	{
		if (!It->IsValid())
		{
			It.RemoveCurrent();
		}
		else if (It->Get() == &Engine)
		{
			return;
		}
	}
	// Taken under the engine's CallbackMutex, which the physics thread holds through a step: not under our lock.
	TSharedPtr<FTable, ESPMode::ThreadSafe> Shared = Table;
	Engine.RegisterPreStepCallback([Shared](mjModel* M, mjData* D) { PreStep(*Shared, M, D); });
	Engine.RegisterPostStepCallback([Shared](mjModel* M, mjData* D) { PostStep(*Shared, M, D); });
	Hooked.Add(&Engine);
}

void FRammsMujocoImuProvider::ListBodies(AActor& Actor, TArray<FRammsImuBodyRef>& OutBodies) const
{
	TArray<AActor*> Actors;
	Actor.GetAttachedActors(Actors, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
	Actors.Insert(&Actor, 0);

	// Bound bodies in MuJoCo's order (parents first: the base leads), then any of a scene not compiled yet (listed so
	// they can be named; until it compiles they read Kinematic) by attach depth.
	TArray<TPair<int64, FRammsImuBodyRef>> Found;
	for (AActor* Each : Actors)
	{
		if (!IsValid(Each))
		{
			continue;
		}
		TInlineComponentArray<UMjBody*> Bodies(Each);
		for (UMjBody* Body : Bodies)
		{
			FRammsImuBodyRef Ref;
			if (!IsValid(Body) || !ResolveOnBody(*Body, *Body, NAME_None, FTransform::Identity, 0, /*bListing=*/true, Ref))
			{
				continue;
			}
			const TOptional<int32>& Bound = Body->GetBoundId();
			if (Bound.IsSet() && Bound.GetValue() == 0)
			{
				continue; // the world body
			}
			int64 Depth = 0;
			for (const USceneComponent* Node = Body->GetAttachParent(); Node; Node = Node->GetAttachParent())
			{
				++Depth;
			}
			// Listed by the URLab body (tracking it resolves again, to Chaos if Chaos simulates it).
			Ref.Body = Body;
			Ref.Bone = NAME_None;
			Found.Emplace(Bound.IsSet() ? int64(Bound.GetValue()) : (int64(1) << 32) + Depth, Ref);
		}
	}
	Found.StableSort([](const TPair<int64, FRammsImuBodyRef>& A, const TPair<int64, FRammsImuBodyRef>& B) { return A.Key < B.Key; });
	for (const TPair<int64, FRammsImuBodyRef>& Entry : Found)
	{
		OutBodies.Add(Entry.Value);
	}
}

// ---------------------------------------------------------------------------
// Physics thread (URLab's worker, under the engine's CallbackMutex)
// ---------------------------------------------------------------------------

FVector FRammsMujocoImuProvider::MjAxialToUe(const double In[3])
{
	// An axial vector under the Y mirror: the mirror of the vector, negated (det = -1). URLabAxisConv::MjQuatToUe
	// negates a rotation's X and Z the same way.
	return FVector(-In[0], In[1], -In[2]);
}

bool FRammsMujocoImuProvider::IsMocapDriven(const mjModel* M, int32 BodyId)
{
	if (!M || BodyId <= 0 || BodyId >= M->nbody)
	{
		return false;
	}
	// Up the chain while no dof moves the body relative to its parent: a mocap body met on the way poses it.
	for (int32 Body = BodyId; Body > 0; Body = M->body_parentid[Body])
	{
		if (M->body_mocapid[Body] >= 0)
		{
			return true;
		}
		if (M->body_dofnum[Body] > 0)
		{
			return false;
		}
	}
	return false; // welded to the world: still, and simulated as such
}

void FRammsMujocoImuProvider::EnsureAccelerations(const mjModel* M, mjData* D, const double* PreStepQvel, double* Scratch, bool bRecompute)
{
	if (!M || !D || (D->flg_rnepost && !bRecompute))
	{
		return; // the step's own acceleration-stage sensors computed cacc for the step's state
	}
	if (PreStepQvel == nullptr || M->nv <= 0)
	{
		mj_rnePostConstraint(M, D);
		return;
	}
	// cacc = cacc(parent) + cdof_dot qvel + cdof qacc: cdof_dot and qacc are the forward pass's, so qvel must be the
	// one it ran with, not the one the integrator wrote after it.
	const int32	   Count = M->nv;
	TArray<double> Allocated;
	if (Scratch == nullptr)
	{
		Allocated.SetNumUninitialized(Count);
		Scratch = Allocated.GetData();
	}
	FMemory::Memcpy(Scratch, D->qvel, sizeof(double) * Count);
	FMemory::Memcpy(D->qvel, PreStepQvel, sizeof(double) * Count);
	mj_rnePostConstraint(M, D);
	FMemory::Memcpy(D->qvel, Scratch, sizeof(double) * Count);
}

void FRammsMujocoImuProvider::ComputeRealizedAccelerations(const mjModel* M, mjData* D, const double* PreStepQvel, bool bSwapQvel, double* Scratch)
{
	if (!M || !D)
	{
		return;
	}
	const int32	 Count = M->nv;
	const double Timestep = M->opt.timestep;
	if (Count <= 0 || PreStepQvel == nullptr || Timestep <= 0.0)
	{
		mj_rnePostConstraint(M, D); // nothing moves: cacc is the world's -gravity everywhere
		D->flg_rnepost = 0;
		return;
	}
	TArray<double> Allocated;
	if (Scratch == nullptr)
	{
		Allocated.SetNumUninitialized(2 * Count);
		Scratch = Allocated.GetData();
	}
	double* KeptQvel = Scratch;
	double* KeptQacc = Scratch + Count;
	FMemory::Memcpy(KeptQacc, D->qacc, sizeof(double) * Count);
	// The acceleration the integrator applied: it advanced qvel by exactly timestep x the qacc it solved for.
	for (int32 i = 0; i < Count; ++i)
	{
		D->qacc[i] = (D->qvel[i] - PreStepQvel[i]) / Timestep;
	}
	if (bSwapQvel)
	{
		FMemory::Memcpy(KeptQvel, D->qvel, sizeof(double) * Count);
		FMemory::Memcpy(D->qvel, PreStepQvel, sizeof(double) * Count);
	}
	mj_rnePostConstraint(M, D);
	if (bSwapQvel)
	{
		FMemory::Memcpy(D->qvel, KeptQvel, sizeof(double) * Count);
	}
	FMemory::Memcpy(D->qacc, KeptQacc, sizeof(double) * Count);
	// What cacc (and cfrc_int / cfrc_ext) now hold is not MuJoCo's own: anything that needs those computes them again.
	D->flg_rnepost = 0;
}

bool FRammsMujocoImuProvider::ComputeSample(const mjModel* M, const mjData* D, int32 BodyId, const FTransform& PointInBody, double StateTime,
	FRammsImuSample& OutSample)
{
	if (!M || !D || BodyId < 0 || BodyId >= M->nbody)
	{
		return false;
	}
	const FVector BodyLocation = URLabAxisConv::MjPositionToUe(D->xpos + 3 * BodyId);
	const FQuat	  BodyRotation = URLabAxisConv::MjQuatToUe(D->xquat + 4 * BodyId);
	const bool	  bGravity = (M->opt.disableflags & mjDSBL_GRAVITY) == 0;
	const FVector Gravity = bGravity ? URLabAxisConv::MjDirectionToUe(M->opt.gravity) * 100.0 : FVector::ZeroVector;
	const FVector Lever = BodyRotation.RotateVector(PointInBody.GetLocation());

	FRammsImuSample Sample;
	Sample.bValid = true;
	Sample.bHasAcceleration = true;
	Sample.Source = ERammsImuSource::MuJoCo;
	Sample.SimTimeSeconds = StateTime;
	Sample.Location = BodyLocation + Lever;
	Sample.Orientation = BodyRotation * PointInBody.GetRotation();
	Sample.Gravity = Gravity;
	if (M->body_weldid[BodyId] == 0)
	{
		// Welded to the world: MuJoCo reports its velocity and acceleration as zero (its own accelerometer reads 0
		// there too); it is held still, so it reads -gravity like any body at rest.
		Sample.ProperAcceleration = -Gravity;
	}
	else
	{
		// [angular; linear] of the body frame's origin (xpos), world axes: m/s and rad/s, and the classical
		// acceleration of that point with the world's -gravity in it.
		mjtNum Velocity[6];
		mjtNum Acceleration[6];
		mj_objectVelocity(M, D, mjOBJ_XBODY, BodyId, Velocity, 0);
		mj_objectAcceleration(M, D, mjOBJ_XBODY, BodyId, Acceleration, 0);
		const FVector AngularVelocity = MjAxialToUe(Velocity);
		const FVector AngularAcceleration = MjAxialToUe(Acceleration);
		const FVector OriginVelocity = URLabAxisConv::MjDirectionToUe(Velocity + 3) * 100.0;
		const FVector OriginProper = URLabAxisConv::MjDirectionToUe(Acceleration + 3) * 100.0;
		Sample.LinearVelocity = RammsImuMath::PointVelocity(OriginVelocity, AngularVelocity, Lever);
		Sample.AngularVelocity = RammsImuMath::RadiansToDegrees(AngularVelocity);
		Sample.ProperAcceleration = RammsImuMath::PointAcceleration(OriginProper, AngularAcceleration, AngularVelocity, Lever);
		Sample.AngularAcceleration = RammsImuMath::RadiansToDegrees(AngularAcceleration);
	}
	OutSample = Sample;
	return true;
}

void FRammsMujocoImuProvider::PreStep(FTable& Table, const mjModel* M, const mjData* D)
{
	FScopeLock Guard(&Table.Lock);
	if (!Table.bAlive || !M || !D)
	{
		return;
	}
	bool bUsed = false;
	for (const TPair<int32, FPoint>& Pair : Table.Points)
	{
		bUsed |= Pair.Value.Model == M;
	}
	if (!bUsed)
	{
		return;
	}
	FModelState* State = Table.Models.Find(M);
	if (!State)
	{
		// A model met for the first time (a compile): forget the ones no point is on any more, then size this one's
		// buffers. The only allocations on this thread, once per model.
		for (auto It = Table.Models.CreateIterator(); It; ++It)
		{
			bool bReferenced = false;
			for (const TPair<int32, FPoint>& Pair : Table.Points)
			{
				bReferenced |= Pair.Value.Model == It.Key();
			}
			if (!bReferenced)
			{
				It.RemoveCurrent();
			}
		}
		State = &Table.Models.Add(M);
	}
	State->PreStepQvel.SetNumUninitialized(M->nv, EAllowShrinking::No);
	State->Scratch.SetNumUninitialized(2 * M->nv, EAllowShrinking::No);
	State->PreStepData = D;
	State->PreStepTime = D->time;
	if (M->nv > 0)
	{
		FMemory::Memcpy(State->PreStepQvel.GetData(), D->qvel, sizeof(double) * M->nv);
	}
}

void FRammsMujocoImuProvider::PostStep(FTable& Table, const mjModel* M, mjData* D)
{
	FScopeLock Guard(&Table.Lock);
	if (!Table.bAlive || !M || !D)
	{
		return;
	}
	bool bAny = false;
	bool bUnsampled = false;
	bool bMoved = false; // a sampled body's pose differs from its last sample's (a paused scene edited)
	for (const TPair<int32, FPoint>& Pair : Table.Points)
	{
		const FPoint& Point = Pair.Value;
		if (Point.Model == M && Point.NBody == M->nbody && Point.BodyId >= 0 && Point.BodyId < M->nbody)
		{
			bAny = true;
			if (!Point.Latest.bValid)
			{
				bUnsampled = true; // (its SampledPose is no pose of this scene's to compare with)
				continue;
			}
			const mjtNum* Pos = D->xpos + 3 * Point.BodyId;
			const mjtNum* Quat = D->xquat + 4 * Point.BodyId;
			for (int32 i = 0; i < 3 && !bMoved; ++i)
			{
				bMoved |= Pos[i] != Point.SampledPose[i];
			}
			for (int32 i = 0; i < 4 && !bMoved; ++i)
			{
				bMoved |= Quat[i] != Point.SampledPose[3 + i];
			}
		}
	}
	FModelState* State = bAny ? Table.Models.Find(M) : nullptr;
	if (!State)
	{
		return; // no point on M, or no pre-step callback has seen M yet
	}
	// The callbacks run on every pass of URLab's loop, paused too: nothing new until the clock moves, a point has no
	// sample yet, or the paused scene was edited.
	if (D->time == State->LastSampled && !bUnsampled && !bMoved)
	{
		return;
	}

	// What this pass did since the pre-step callback: one step (the usual), several (a client stepping in batches),
	// or none (paused; a reset or a state pushed without a step).
	const double Timestep = M->opt.timestep;
	const double TimeSlack = 1.0e-9 * FMath::Max(1.0, FMath::Abs(D->time));
	const bool	 bStepped = State->PreStepData == D && D->time > State->PreStepTime;
	const bool	 bOneStep = bStepped && State->PreStepQvel.Num() == M->nv && FMath::IsNearlyEqual(D->time, State->PreStepTime + Timestep, TimeSlack);
	const bool	 bRungeKutta = M->opt.integrator == mjINT_RK4;

	// Only a new point to sample, and nothing has run since the last pass that sampled (the scene paused): the derived
	// quantities and cacc are still what that pass left, so the new point is sampled from them and stamped as that
	// pass stamped its own; the points already sampled keep their samples.
	const bool bOnlyNew = !bStepped && !bMoved && D == State->SampledData && D->time == State->LastSampled && State->DerivedTime >= 0.0;

	// The time of the state the derived quantities describe: the start of the last step (its forward pass); RK4's
	// last stage evaluated them at the end; without a step, the current time (a reset, a restore or an edit ran a
	// forward pass there).
	double StateTime = D->time;
	if (bOnlyNew)
	{
		StateTime = State->DerivedTime;
	}
	else if (bStepped && !bRungeKutta)
	{
		StateTime = bOneStep ? State->PreStepTime : D->time - Timestep;
	}

	if (bOnlyNew)
	{
		// cacc is still the last pass's: nothing to compute.
	}
	else if (bOneStep && Table.Acceleration == ERammsMjImuAcceleration::Realized)
	{
		ComputeRealizedAccelerations(M, D, State->PreStepQvel.GetData(), /*bSwapQvel=*/!bRungeKutta, State->Scratch.GetData());
	}
	else
	{
		// Explicit, or nothing to tell what this pass applied: the forward pass's own accelerations. RK4's later
		// stages skip the sensors, so its flg_rnepost (and cacc) may be the first stage's: computed again.
		const bool bSwap = bOneStep && !bRungeKutta;
		EnsureAccelerations(M, D, bSwap ? State->PreStepQvel.GetData() : nullptr, State->Scratch.GetData(), /*bRecompute=*/bRungeKutta);
	}
	State->LastSampled = D->time;
	State->DerivedTime = StateTime;
	State->SampledData = D;

	for (TPair<int32, FPoint>& Pair : Table.Points)
	{
		FPoint& Point = Pair.Value;
		if (bOnlyNew && Point.Latest.bValid)
		{
			continue;
		}
		if (Point.Model == M && Point.NBody == M->nbody && ComputeSample(M, D, Point.BodyId, Point.PointInBody, StateTime, Point.Latest))
		{
			FMemory::Memcpy(Point.SampledPose, D->xpos + 3 * Point.BodyId, sizeof(double) * 3);
			FMemory::Memcpy(Point.SampledPose + 3, D->xquat + 4 * Point.BodyId, sizeof(double) * 4);
			Point.History.Push(Point.Latest); // sized on the game thread: no allocation here
		}
	}
}
