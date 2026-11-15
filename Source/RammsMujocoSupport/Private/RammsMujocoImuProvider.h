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

/** Which acceleration a MuJoCo sample reports (console variable ramms.Imu.MuJoCoAcceleration). */
enum class ERammsMjImuAcceleration : uint8
{
	/** What the integrator applied over the step: qacc = (qvel after - qvel before) / timestep, the same as Chaos's
	 *  velocity change over its step. Exact for Euler, implicit and implicitfast (each advances qvel by timestep x
	 *  the qacc it solved for) and for RK4 (its weighted stages). */
	Realized,
	/** MuJoCo's own accelerometer convention: the forward pass's qacc. Under implicit / implicitfast with stiff
	 *  damping this is not the acceleration the step applied (it can be several times larger). */
	Explicit,
};

/**
 * IMU data from MuJoCo (URLab): the bodies a world's MuJoCo scene simulates, sampled after every step on URLab's
 * physics thread by a post-step callback on the scene's engine.
 *
 * Per step, for the tracked bodies only: mj_rnePostConstraint brings mjData::cacc up to date for the step, then
 * mj_objectVelocity / mj_objectAcceleration give the body frame's (xpos, mjOBJ_XBODY) velocity and acceleration, and
 * the lever arm to the point gives the point's. MuJoCo's cacc starts the world at -gravity, so the acceleration it
 * gives is what an accelerometer reads (+|g| up at rest, 0 in free fall): the same ProperAcceleration the Chaos
 * provider reports. Units and axes are converted as URLab converts poses: m -> cm with Y negated for positions,
 * velocities and accelerations; angular rates with X and Z negated (the sense URLabAxisConv::MjQuatToUe gives
 * rotations).
 *
 * Timing: a step's derived quantities (xpos, cvel, cdof_dot) are those of its forward pass, at the start of the
 * step, while its integrator has already advanced qvel; a pre-step callback keeps the qvel the step started with,
 * which goes back into qvel while cacc is computed. By default (ERammsMjImuAcceleration::Realized) qacc is the one
 * the step applied, (qvel after - qvel before) / timestep, so a sample's acceleration is the velocity change of the
 * step that starts at its state; Explicit keeps the forward pass's qacc, what MuJoCo's own accelerometer reads. A
 * sample is stamped with the time of its state: the start of the step (RK4: the end, where its last stage
 * evaluated the derived quantities). Steps the worker batched (or that ran without the callbacks) are read from the
 * last forward pass with the current qvel and explicit qacc.
 *
 * Body ids are resolved on the game thread from each URLab body's bound id; a recompile (new model, new ids) is
 * noticed on the next read and the point re-resolved, and the physics thread ignores points resolved against
 * another model. URLab cannot unregister callbacks: they hold the shared table, which the provider marks dead.
 *
 * A URLab body of an articulation that Chaos simulates (URammsChaosArticulationComponent) resolves to Chaos. A body
 * a mocap body carries (or a mocap body itself) is posed by Unreal, not simulated: MuJoCo reports it still, so it is
 * left to Kinematic.
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
	virtual bool			IsTrackValid(int32 Handle) const override;
	virtual void			SetHistoryCapacity(int32 Handle, int32 Capacity) override;
	virtual bool			ReadHistory(int32 Handle, int64 Since, int32 Max, TArray<FRammsImuSample>& OutSamples, int64& OutFirst,
				   int64& OutOldest, int64& OutNext) override;
	virtual void			ListBodies(AActor& Actor, TArray<FRammsImuBodyRef>& OutBodies) const override;

	/** The engine of the MuJoCo scene in Body's world, or null. */
	static UMjPhysicsEngine* EngineFor(const UMjBody& Body);

	/** A body's MJCF name, else its component name. */
	static FString BodyNameOf(const UMjBody& Body);

	/** The acceleration convention ramms.Imu.MuJoCoAcceleration asks for. */
	static ERammsMjImuAcceleration AccelerationSetting();

	/** The offset URLab places a quick-converted body's component at from its MuJoCo frame (the first child static
	 *  mesh's collision bounds centre, in the body's frame), computed as UMjBody::RefreshMeshPivotOffset does. */
	static FVector QuickConvertPivot(const UMjBody& Body);

	// --- The physics-thread side (public for tests) -------------------------------

	/** A MuJoCo axial vector (angular velocity or acceleration, rad) in Unreal axes: X and Z negated, the sense
	 *  URLabAxisConv::MjQuatToUe turns rotations, so that w x r holds for positions converted by MjPositionToUe. */
	static FVector MjAxialToUe(const double In[3]);

	/** True when Unreal, not the simulation, moves body BodyId: it is a mocap body or welded (no dofs between) to
	 *  one. MuJoCo reports such a body as still. */
	static bool IsMocapDriven(const mjModel_* M, int32 BodyId);

	/**
	 * Explicit: bring D->cacc up to date for the state the last step's forward pass computed, unless that step did
	 * already (an accelerometer in the model sets flg_rnepost). PreStepQvel: qvel as that step started (M->nv
	 * values), so cacc's velocity-product term uses the velocities the forward pass used; null uses D's current
	 * qvel. Scratch: M->nv doubles to keep the current qvel in (null: allocated).
	 */
	static void EnsureAccelerations(const mjModel_* M, mjData_* D, const double* PreStepQvel, double* Scratch = nullptr);

	/**
	 * Realized: D->cacc for the step that just ran, with qacc = (D->qvel - PreStepQvel) / timestep (the acceleration
	 * the integrator applied) and, when bSwapQvel, the velocity-product term from PreStepQvel (the qvel the forward
	 * pass used; RK4's last stage used neither, so it keeps the current qvel). Leaves qvel, qacc as they were and
	 * flg_rnepost cleared (what cacc holds is not MuJoCo's own). Scratch: 2 x M->nv doubles (null: allocated).
	 */
	static void ComputeRealizedAccelerations(const mjModel_* M, mjData_* D, const double* PreStepQvel, bool bSwapQvel, double* Scratch = nullptr);

	/** The sample of the frame PointInBody (Unreal units, in body BodyId's frame) from D, stamped StateTime. Compute
	 *  the accelerations first. False for a body id outside M. */
	static bool ComputeSample(const mjModel_* M, const mjData_* D, int32 BodyId, const FTransform& PointInBody, double StateTime,
		FRammsImuSample& OutSample);

	/** One tracked point, as the physics thread samples it. */
	struct FPoint
	{
		// Written on the game thread.
		const mjModel_* Model = nullptr; // the model BodyId indexes (compared, never dereferenced)
		int32			NBody = 0;
		int32			BodyId = INDEX_NONE;
		FTransform		PointInBody = FTransform::Identity; // in the MuJoCo body's frame (Unreal units)

		// Written on the physics thread (History sized on the game thread).
		FRammsImuSample		Latest;
		FRammsImuSampleRing History;
		double				SampledPose[7] = {}; // the body's xpos, xquat when Latest was taken
	};

	/** The step state kept per model; its buffers are sized on the physics thread once per model. */
	struct FModelState
	{
		const mjData_* PreStepData = nullptr;
		double		   PreStepTime = -1.0;
		TArray<double> PreStepQvel;
		TArray<double> Scratch; // 2 x nv
		double		   LastSampled = -1.0;
	};

	/** The points, shared by the provider and the engines' callbacks. */
	struct FTable
	{
		FCriticalSection				   Lock;
		TMap<int32, FPoint>				   Points;
		TMap<const mjModel_*, FModelState> Models;
		ERammsMjImuAcceleration			   Acceleration = ERammsMjImuAcceleration::Realized;
		bool							   bAlive = true;
	};

	/** The pre-step callback: keep qvel as the step starts, for the models points are on. */
	static void PreStep(FTable& Table, const mjModel_* M, const mjData_* D);

	/** The post-step callback: sample every point on M once per step (and again when a paused scene is edited). */
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
