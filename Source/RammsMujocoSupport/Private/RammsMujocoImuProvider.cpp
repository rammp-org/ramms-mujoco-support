// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMujocoImuProvider.h"

#include "Components/SceneComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "RammsChaosArticulationComponent.h"
#include "RammsImuMath.h"
#include "mujoco/mujoco.h"

static_assert(sizeof(mjtNum) == sizeof(double), "the IMU provider copies qvel as doubles");

TSharedPtr<IRammsImuProvider> MakeRammsMujocoImuProvider(UWorld& World)
{
	return MakeShared<FRammsMujocoImuProvider>(World);
}

FRammsMujocoImuProvider::FRammsMujocoImuProvider(UWorld& InWorld)
	: World(&InWorld)
	, Table(MakeShared<FTable, ESPMode::ThreadSafe>())
{
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
	if (!Body)
	{
		return false;
	}
	const FTransform SensorWorld = LocalPose * RammsImuProviders::ComponentFrame(Component, Bone);
	FTransform		 BodyFrame = Body->GetComponentTransform();
	BodyFrame.SetScale3D(FVector::OneVector);
	const FTransform PointInComponent = SensorWorld.GetRelativeTransform(BodyFrame);

	OutRef.Bone = NAME_None;
	OutRef.BodyName = BodyNameOf(*Body);
	OutRef.Depth = Depth;

	// A URLab robot that Chaos simulates: the Chaos body that carries this one serves the point.
	const AActor* Owner = Body->GetOwner();
	if (const URammsChaosArticulationComponent* OnChaos = Owner ? Owner->FindComponentByClass<URammsChaosArticulationComponent>() : nullptr;
		OnChaos && OnChaos->IsBuilt())
	{
		URammsChaosBodyComponent* ChaosBody = nullptr;
		FTransform				  InChaosBody;
		if (!OnChaos->FindChaosBody(Body, ChaosBody, InChaosBody))
		{
			return false;
		}
		OutRef.Source = ERammsImuSource::Chaos;
		OutRef.Body = ChaosBody;
		OutRef.PointInBody = PointInComponent * InChaosBody;
		return true;
	}

	// Else MuJoCo, when this world has a MuJoCo scene (it may not have compiled yet: the point waits for it).
	if (!EngineFor(*Body))
	{
		return false;
	}
	OutRef.Source = ERammsImuSource::MuJoCo;
	OutRef.Body = Body;
	OutRef.PointInBody = PointInComponent;
	return true;
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
		if (Bound.IsSet() && Bound.GetValue() >= 0 && Bound.GetValue() < Model->nbody)
		{
			BodyId = Bound.GetValue();
		}
	}
	if (Engine && BodyId != INDEX_NONE)
	{
		Hook(*Engine);
	}
	if (!bForce && Model == Entry.Model && BodyId == Entry.BodyId && Engine == Entry.Engine.Get())
	{
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
		const FVector Pivot = Body->GetComponentQuat().UnrotateVector(Body->GetWorldPosition() - Body->GetComponentLocation());
		PointInMjBody.SetLocation(PointInMjBody.GetLocation() - Pivot);
	}

	FScopeLock Guard(&Table->Lock);
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

	// Bound bodies in MuJoCo's order (parents first: the base leads), then any not compiled yet by attach depth.
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
			if (!IsValid(Body) || !Resolve(*Body, NAME_None, FTransform::Identity, false, Ref))
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

void FRammsMujocoImuProvider::EnsureAccelerations(const mjModel* M, mjData* D, const double* PreStepQvel)
{
	if (!M || !D || D->flg_rnepost)
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
	const int32							 Count = M->nv;
	TArray<double, TInlineAllocator<64>> Stepped;
	Stepped.SetNumUninitialized(Count);
	FMemory::Memcpy(Stepped.GetData(), D->qvel, sizeof(double) * Count);
	FMemory::Memcpy(D->qvel, PreStepQvel, sizeof(double) * Count);
	mj_rnePostConstraint(M, D);
	FMemory::Memcpy(D->qvel, Stepped.GetData(), sizeof(double) * Count);
}

bool FRammsMujocoImuProvider::ComputeSample(const mjModel* M, const mjData* D, int32 BodyId, const FTransform& PointInBody,
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
	Sample.SimTimeSeconds = D->time;
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
	FModelState& State = Table.Models.FindOrAdd(M);
	State.PreStepTime = D->time;
	State.PreStepQvel.SetNumUninitialized(M->nv);
	if (M->nv > 0)
	{
		FMemory::Memcpy(State.PreStepQvel.GetData(), D->qvel, sizeof(double) * M->nv);
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
	for (const TPair<int32, FPoint>& Pair : Table.Points)
	{
		const FPoint& Point = Pair.Value;
		if (Point.Model == M && Point.NBody == M->nbody && Point.BodyId >= 0 && Point.BodyId < M->nbody)
		{
			bAny = true;
			bUnsampled |= !Point.Latest.bValid;
		}
	}
	if (!bAny)
	{
		return;
	}
	FModelState& State = Table.Models.FindOrAdd(M);
	// The callbacks run on every pass of URLab's loop, paused too: nothing new until the clock moves (or a point
	// has no sample yet).
	if (D->time == State.LastSampled && !bUnsampled)
	{
		return;
	}
	// Exactly one step since the pre-step callback: the forward pass ran with the qvel kept then. Several (a
	// client stepping the scene in batches) or none (paused): the current qvel, which is exact when nothing has
	// been integrated since the forward pass and off by a step's change in velocity otherwise.
	const double TimeSlack = 1.0e-9 * FMath::Max(1.0, FMath::Abs(D->time));
	const bool	 bOneStep = State.PreStepQvel.Num() == M->nv && D->time > State.PreStepTime
		&& FMath::IsNearlyEqual(D->time, State.PreStepTime + M->opt.timestep, TimeSlack);
	EnsureAccelerations(M, D, bOneStep ? State.PreStepQvel.GetData() : nullptr);
	State.LastSampled = D->time;

	for (TPair<int32, FPoint>& Pair : Table.Points)
	{
		FPoint& Point = Pair.Value;
		if (Point.Model == M && Point.NBody == M->nbody)
		{
			ComputeSample(M, D, Point.BodyId, Point.PointInBody, Point.Latest);
		}
	}
}
