// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "RammsImuProvider.h"
#include "UObject/WeakObjectPtr.h"

struct mjModel_;
struct mjData_;

class UMjBody;
class UMjPhysicsEngine;
class UWorld;

/**
 * IMU data from MuJoCo (URLab): the bodies a world's MuJoCo scene simulates, sampled after every step on URLab's
 * physics thread by a post-step callback on the scene's engine.
 *
 * Per step, for the tracked bodies only: mj_rnePostConstraint brings mjData::cacc up to date (once per step, and not
 * at all when the model's own accelerometer / force sensors already did), then mj_objectVelocity /
 * mj_objectAcceleration give the body frame's (xpos, mjOBJ_XBODY) velocity and acceleration, and the lever arm to
 * the point gives the point's. MuJoCo's cacc starts the world at -gravity, so the acceleration it gives is already
 * what an accelerometer reads (+|g| up at rest, 0 in free fall): the same ProperAcceleration the Chaos provider
 * reports. Units and axes are converted as URLab converts poses: m -> cm with Y negated for positions, velocities
 * and accelerations; angular rates with X and Z negated (the sense URLabAxisConv::MjQuatToUe gives rotations).
 *
 * The step's derived quantities (xpos, cvel, qacc) are those of its forward pass, at the start of the step, while
 * its integrator has already advanced qvel; cacc is computed from the qvel the step started with (saved by a
 * pre-step callback), so it is the acceleration MuJoCo's own accelerometer reports for that step. The sample is
 * stamped with mjData::time (the end of the step), the clock the robot backends report.
 *
 * Body ids are resolved on the game thread from each URLab body's bound id; a recompile (new model, new ids) is
 * noticed on the next read and the point re-resolved, and the physics thread ignores points resolved against
 * another model. URLab cannot unregister callbacks: they hold the shared table, which the provider marks dead.
 *
 * A URLab body of an articulation that Chaos simulates (URammsChaosArticulationComponent) resolves to Chaos.
 */
class FRammsMujocoImuProvider final : public IRammsImuProvider
{
public:
	explicit FRammsMujocoImuProvider(UWorld& InWorld);
	virtual ~FRammsMujocoImuProvider() override;

	virtual ERammsImuSource GetSource() const override { return ERammsImuSource::MuJoCo; }
	virtual bool			Resolve(USceneComponent& Component, FName Bone, const FTransform& LocalPose, bool bExplicit,
				   FRammsImuBodyRef& OutRef) const override;
	virtual int32			Track(const FRammsImuBodyRef& Ref) override;
	virtual bool			GetSample(int32 Handle, FRammsImuSample& OutSample) override;
	virtual void			Untrack(int32 Handle) override;
	virtual void			ListBodies(AActor& Actor, TArray<FRammsImuBodyRef>& OutBodies) const override;

	/** The engine of the MuJoCo scene in Body's world, or null. */
	static UMjPhysicsEngine* EngineFor(const UMjBody& Body);

	/** A body's MJCF name, else its component name. */
	static FString BodyNameOf(const UMjBody& Body);

	// --- The physics-thread side (public for tests) -------------------------------

	/** A MuJoCo axial vector (angular velocity or acceleration, rad) in Unreal axes: X and Z negated, the sense
	 *  URLabAxisConv::MjQuatToUe turns rotations, so that w x r holds for positions converted by MjPositionToUe. */
	static FVector MjAxialToUe(const double In[3]);

	/**
	 * Bring D->cacc up to date for the state the last step's forward pass computed, unless that step did already (an
	 * accelerometer in the model sets flg_rnepost). PreStepQvel: qvel as that step started (M->nv values), so cacc's
	 * velocity-product term uses the velocities the forward pass used; null uses D's current qvel.
	 */
	static void EnsureAccelerations(const mjModel_* M, mjData_* D, const double* PreStepQvel);

	/** The sample of the frame PointInBody (Unreal units, in body BodyId's frame) from D. Call EnsureAccelerations
	 *  first. False for a body id outside M. */
	static bool ComputeSample(const mjModel_* M, const mjData_* D, int32 BodyId, const FTransform& PointInBody, FRammsImuSample& OutSample);

	/** One tracked point, as the physics thread samples it. */
	struct FPoint
	{
		// Written on the game thread.
		const mjModel_* Model = nullptr; // the model BodyId indexes (compared, never dereferenced)
		int32			NBody = 0;
		int32			BodyId = INDEX_NONE;
		FTransform		PointInBody = FTransform::Identity; // in the MuJoCo body's frame (Unreal units)

		// Written on the physics thread.
		FRammsImuSample Latest;
	};

	/** The step state kept per model. */
	struct FModelState
	{
		double		   PreStepTime = -1.0;
		TArray<double> PreStepQvel;
		double		   LastSampled = -1.0;
	};

	/** The points, shared by the provider and the engines' callbacks. */
	struct FTable
	{
		FCriticalSection				   Lock;
		TMap<int32, FPoint>				   Points;
		TMap<const mjModel_*, FModelState> Models;
		bool							   bAlive = true;
	};

	/** The pre-step callback: keep qvel as the step starts, for the models points are on. */
	static void PreStep(FTable& Table, const mjModel_* M, const mjData_* D);

	/** The post-step callback: sample every point on M once per step. */
	static void PostStep(FTable& Table, const mjModel_* M, mjData_* D);

private:
	struct FTracked
	{
		TWeakObjectPtr<UMjBody>			 Body;
		FTransform						 PointInComponent = FTransform::Identity; // in the body component's frame
		TWeakObjectPtr<UMjPhysicsEngine> Engine;
		const mjModel_*					 Model = nullptr;
		int32							 BodyId = INDEX_NONE;
	};

	/** Re-resolve a point's engine, model and body id (a compile, a recompile) and hand any change over. */
	void Refresh(int32 Handle, FTracked& Entry, bool bForce);

	/** Register the callbacks with Engine once. */
	void Hook(UMjPhysicsEngine& Engine);

	TWeakObjectPtr<UWorld>					 World;
	TSharedPtr<FTable, ESPMode::ThreadSafe>	 Table;
	TMap<int32, FTracked>					 Tracked;
	TArray<TWeakObjectPtr<UMjPhysicsEngine>> Hooked;
	int32									 NextHandle = 1;
};

/** The factory RammsMujocoSupport registers for ERammsImuSource::MuJoCo. */
TSharedPtr<IRammsImuProvider> MakeRammsMujocoImuProvider(UWorld& World);
