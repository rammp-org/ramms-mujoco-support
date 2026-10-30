// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMjGroundSupportLibrary.h"
#include "RammsMujocoActuationBackend.h"
#include "RammsRobotBaseComponent.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "mujoco/mujoco.h"

namespace
{
	/** The simulated time a recorder keeps, and so the longest window a read can average over (s). */
	constexpr double KeepSeconds = 1.0;

	/** One step's load on one robot body: a wheel's (Slot >= 0: its index among the wheels asked about) or another
	 *  body's (Slot < 0: -1 - its body id). */
	struct FStepLoad
	{
		int32  Slot = 0;
		double Force = 0.0;						// the step's contacts on it together (N)
		double Weighted[3] = { 0.0, 0.0, 0.0 }; // their points (MuJoCo world, m) weighted by their force
		int32  Contacts = 0;
	};

	struct FStep
	{
		double								   Dt = 0.0;
		double								   Com[3] = { 0.0, 0.0, 0.0 };
		TArray<FStepLoad, TInlineAllocator<8>> Loads;
	};

	/** A robot as one compiled model numbers it. */
	struct FRobotIds
	{
		const mjModel* Model = nullptr;
		int32		   Root = INDEX_NONE;
		TArray<int32>  WheelBody;
	};

	/** One step's contacts on the robot, gathered per wheel and per other body, and its centre of mass. Under the
	 *  engine's lock. */
	void GatherStep(const FRobotIds& Ids, const mjModel* M, const mjData* D, FStep& Out)
	{
		Out.Loads.Reset();
		for (int32 k = 0; k < 3; ++k)
		{
			Out.Com[k] = D->subtree_com[3 * Ids.Root + k];
		}
		// The wheel a robot body belongs to: itself or the nearest of its ancestors that is a wheel's body.
		auto WheelOf = [&](int32 Body) -> int32 {
			for (int32 B = Body; B > 0; B = M->body_parentid[B])
			{
				const int32 Wheel = Ids.WheelBody.IndexOfByKey(B);
				if (Wheel != INDEX_NONE)
				{
					return Wheel;
				}
				if (B == Ids.Root)
				{
					break;
				}
			}
			return INDEX_NONE;
		};
		auto LoadOn = [&Out](int32 Slot) -> FStepLoad& {
			for (FStepLoad& L : Out.Loads)
			{
				if (L.Slot == Slot)
				{
					return L;
				}
			}
			FStepLoad& L = Out.Loads.AddDefaulted_GetRef();
			L.Slot = Slot;
			return L;
		};
		for (int32 i = 0; i < D->ncon; ++i)
		{
			const mjContact& C = D->contact[i];
			if (C.efc_address < 0 || C.geom[0] < 0 || C.geom[1] < 0)
			{
				continue; // detected, but not in the solve (inside the margin's gap, or excluded)
			}
			const int32 B0 = M->geom_bodyid[C.geom[0]];
			const int32 B1 = M->geom_bodyid[C.geom[1]];
			const bool	bOn0 = B0 > 0 && M->body_rootid[B0] == Ids.Root;
			const bool	bOn1 = B1 > 0 && M->body_rootid[B1] == Ids.Root;
			if (bOn0 == bOn1)
			{
				continue; // the robot against itself, or two other things
			}
			mjtNum Force[6];
			mj_contactForce(M, D, i, Force);
			if (Force[0] <= 0.0)
			{
				continue;
			}
			const int32 Body = bOn0 ? B0 : B1;
			const int32 Wheel = WheelOf(Body);
			FStepLoad&	L = LoadOn(Wheel != INDEX_NONE ? Wheel : -1 - Body);
			for (int32 k = 0; k < 3; ++k)
			{
				L.Weighted[k] += C.pos[k] * Force[0];
			}
			L.Force += Force[0];
			++L.Contacts;
		}
	}

	/** The last KeepSeconds of a robot's steps: recorded on the physics thread after every step, in the engine's
	 *  lock, and read under the same lock. */
	struct FRecorder
	{
		FRobotIds	  Ids;
		TArray<FStep> Ring;
		int32		  Next = 0;
		int32		  Num = 0;
		double		  LastTime = -1.0;
		bool		  bStale = false;

		void Record(const mjModel* M, const mjData* D)
		{
			if (bStale || M != Ids.Model)
			{
				bStale = true; // recompiled: its bodies are numbered anew
				return;
			}
			if (LastTime >= 0.0 && D->time <= LastTime)
			{
				if (D->time < LastTime)
				{
					Num = 0; // reset: start over
					LastTime = D->time;
				}
				return; // paused: the callbacks run without a step
			}
			const double Dt = LastTime >= 0.0 ? D->time - LastTime : M->opt.timestep;
			LastTime = D->time;
			if (Ring.Num() == 0)
			{
				Ring.SetNum(FMath::Clamp(FMath::CeilToInt32(KeepSeconds / FMath::Max(M->opt.timestep, 1e-5)) + 1, 16, 20000));
			}
			FStep& S = Ring[Next];
			GatherStep(Ids, M, D, S);
			S.Dt = Dt;
			Next = (Next + 1) % Ring.Num();
			Num = FMath::Min(Num + 1, Ring.Num());
		}
	};

	/** A body's load over a window, as it is summed. */
	struct FMean
	{
		double Impulse = 0.0;
		double Weighted[3] = { 0.0, 0.0, 0.0 };
		double Bearing = 0.0;
		int32  Contacts = 0;

		void Add(const FStepLoad& L, double Dt)
		{
			Impulse += L.Force * Dt;
			for (int32 k = 0; k < 3; ++k)
			{
				Weighted[k] += L.Weighted[k] * Dt;
			}
			Bearing += Dt;
			Contacts = FMath::Max(Contacts, L.Contacts);
		}

		void To(FRammsMjBodyContact& Out, double Span) const
		{
			Out.Contacts = Contacts;
			Out.NormalForceN = static_cast<float>(Impulse / Span);
			Out.BearingShare = static_cast<float>(Bearing / Span);
			if (Impulse > 0.0)
			{
				const double Mean[3] = { Weighted[0] / Impulse, Weighted[1] / Impulse, Weighted[2] / Impulse };
				Out.Point = URLabAxisConv::MjPositionToUe(Mean);
			}
		}
	};

	FName BodyName(const mjModel* M, int32 Body)
	{
		const char* Name = mj_id2name(M, mjOBJ_BODY, Body);
		return Name ? FName(UTF8_TO_TCHAR(Name)) : FName(*FString::Printf(TEXT("body_%d"), Body));
	}

	/** Steps (newest first) averaged into Out, over Span seconds of them. */
	void Emit(const FRobotIds& Ids, const mjModel* M, TConstArrayView<const FStep*> Steps, FRammsMjGroundSupport& Out)
	{
		TArray<FMean>	   Wheels;
		TMap<int32, FMean> Others;
		Wheels.SetNum(Ids.WheelBody.Num());
		double Com[3] = { 0.0, 0.0, 0.0 };
		double Span = 0.0;
		for (const FStep* S : Steps)
		{
			for (int32 k = 0; k < 3; ++k)
			{
				Com[k] += S->Com[k] * S->Dt;
			}
			for (const FStepLoad& L : S->Loads)
			{
				(L.Slot >= 0 ? Wheels[L.Slot] : Others.FindOrAdd(-1 - L.Slot)).Add(L, S->Dt);
			}
			Span += S->Dt;
		}
		if (Span <= 0.0)
		{
			return;
		}
		const double Mean[3] = { Com[0] / Span, Com[1] / Span, Com[2] / Span };
		Out.CenterOfMass = URLabAxisConv::MjPositionToUe(Mean);
		Out.Wheels.SetNum(Wheels.Num());
		for (int32 w = 0; w < Wheels.Num(); ++w)
		{
			const int32 Body = Ids.WheelBody[w];
			Out.Wheels[w].Body = Body > 0 && Body < M->nbody ? BodyName(M, Body) : NAME_None;
			Wheels[w].To(Out.Wheels[w], Span);
		}
		for (const TPair<int32, FMean>& Other : Others)
		{
			FRammsMjBodyContact& Row = Out.Others.AddDefaulted_GetRef();
			Row.Body = BodyName(M, Other.Key);
			Other.Value.To(Row, Span);
		}
		Out.bValid = true;
	}

	/** The recorders started, one per robot and wheel set (game thread only). */
	struct FRecorderEntry
	{
		TWeakObjectPtr<UMjPhysicsEngine>		   Engine;
		TWeakObjectPtr<AMjArticulation>			   Articulation;
		TArray<FName>							   Wheels;
		TSharedPtr<FRecorder, ESPMode::ThreadSafe> Recorder;
	};
	TArray<FRecorderEntry> GRecorders;
} // namespace

FRammsMjGroundSupport URammsMjGroundSupportLibrary::GetGroundSupport(const URammsRobotBaseComponent* Base, const TArray<FName>& WheelMotors,
	float WindowSeconds)
{
	check(IsInGameThread());
	FRammsMjGroundSupport Out;
	AMjArticulation*	  Art = Base ? FRammsMujocoActuationBackend::FindArticulation(*Base) : nullptr;
	UMjPhysicsEngine*	  Engine = Art ? AAMjManager::ResolveEngine(Art) : nullptr;
	const mjModel*		  M = Engine && Engine->IsInitialized() ? Engine->GetModel() : nullptr;
	if (!M)
	{
		return Out;
	}

	GRecorders.RemoveAll([](const FRecorderEntry& E) { return !E.Engine.IsValid() || !E.Articulation.IsValid(); });
	FRecorderEntry*							   Entry = GRecorders.FindByPredicate([&](const FRecorderEntry& E) {
		   return E.Engine.Get() == Engine && E.Articulation.Get() == Art && E.Wheels == WheelMotors;
	   });
	TSharedPtr<FRecorder, ESPMode::ThreadSafe> Recorder = Entry ? Entry->Recorder : nullptr;
	if (Recorder && Recorder->Ids.Model != M)
	{
		GRecorders.RemoveAll([&](const FRecorderEntry& E) { return E.Recorder == Recorder; });
		Recorder.Reset(); // recompiled: start a recorder for the new model
	}

	if (!Recorder)
	{
		// The robot is the tree under the world its bodies hang from (MuJoCo's body_rootid: the ancestor that is a
		// direct child of the world): the articulation's free-jointed base and all it carries.
		FRobotIds Ids;
		Ids.Model = M;
		for (const UMjBody* Body : Art->GetBodies())
		{
			const TOptional<int32>& Id = Body ? Body->GetBoundId() : TOptional<int32>();
			if (Id.IsSet() && Id.GetValue() > 0 && Id.GetValue() < M->nbody)
			{
				Ids.Root = M->body_rootid[Id.GetValue()];
				break;
			}
		}
		if (Ids.Root == INDEX_NONE)
		{
			return Out; // not compiled yet
		}
		// Each wheel motor's body: the body of the joint its actuator drives.
		for (const FName Motor : WheelMotors)
		{
			int32					Body = INDEX_NONE;
			const UMjNodeComponent* Actuator = Art->GetActuator(Motor.ToString());
			const TOptional<int32>& Id = Actuator ? Actuator->GetBoundId() : TOptional<int32>();
			if (Id.IsSet() && Id.GetValue() >= 0 && Id.GetValue() < M->nu && M->actuator_trntype[Id.GetValue()] == mjTRN_JOINT)
			{
				const int32 Joint = M->actuator_trnid[2 * Id.GetValue()];
				Body = Joint >= 0 && Joint < M->njnt ? M->jnt_bodyid[Joint] : INDEX_NONE;
			}
			Ids.WheelBody.Add(Body);
		}
		Recorder = MakeShared<FRecorder, ESPMode::ThreadSafe>();
		Recorder->Ids = MoveTemp(Ids);
		Engine->RegisterPostStepCallback([Recorder](mjModel* Model, mjData* Data) { Recorder->Record(Model, Data); });
		GRecorders.Add({ Engine, Art, WheelMotors, Recorder });
	}

	// mjData and the ring are the physics thread's: read them between steps, in the lock it records under.
	FScopeLock	  Lock(&Engine->CallbackMutex);
	const mjData* D = Engine->GetData();
	if (!D || Engine->GetModel() != M)
	{
		return Out;
	}
	Out.Time = D->time;
	Out.MassKg = static_cast<float>(M->body_subtreemass[Recorder->Ids.Root]);
	TArray<const FStep*, TInlineAllocator<64>> Steps;
	double									   Span = 0.0;
	for (int32 i = 0; i < Recorder->Num && Span < WindowSeconds; ++i)
	{
		const FStep& S = Recorder->Ring[(Recorder->Next - 1 - i + Recorder->Ring.Num()) % Recorder->Ring.Num()];
		Steps.Add(&S);
		Span += S.Dt;
	}
	if (Steps.Num() > 0)
	{
		Out.WindowSeconds = static_cast<float>(Span);
		Emit(Recorder->Ids, M, Steps, Out);
	}
	else
	{
		// Nothing recorded yet (the first read): the latest step alone.
		FStep Now;
		GatherStep(Recorder->Ids, M, D, Now);
		Now.Dt = 1.0;
		const FStep* One[] = { &Now };
		Emit(Recorder->Ids, M, One, Out);
	}
	return Out;
}
