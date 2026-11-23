// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMjSimControlComponent.h"
#include "HAL/IConsoleManager.h"
#include "Kismet/GameplayStatics.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "TimerManager.h"
#include <mujoco/mujoco.h>

namespace
{
	/** The lowest point (z, MuJoCo world) of the geoms of the bodies whose
	 *  compiled names carry Prefix, from the kinematics as they stand (each
	 *  geom's centre less its bounding radius). */
	double LowestPointOf(const mjModel* Model, const mjData* Data, const FString& Prefix, int32& OutFreeJoint, int32* OutBodies = nullptr, FString* OutGeom = nullptr, FVector* OutPoint = nullptr)
	{
		double Lowest = TNumericLimits<double>::Max();
		int32  LowestGeom = -1;
		OutFreeJoint = -1;
		TArray<bool> Mine;
		Mine.Init(false, Model->nbody);
		int32 Count = 0;
		for (int32 b = 0; b < Model->nbody; ++b)
		{
			const char* Name = mj_id2name(Model, mjOBJ_BODY, b);
			Mine[b] = Name && FString(UTF8_TO_TCHAR(Name)).StartsWith(Prefix);
			Count += Mine[b] ? 1 : 0;
		}
		if (OutBodies)
		{
			*OutBodies = Count;
		}
		for (int32 g = 0; g < Model->ngeom; ++g)
		{
			// Only what can touch the ground: a visual-only or a mass-only geom
			// (the rider's lump) hangs wherever it likes.
			if (!Mine[Model->geom_bodyid[g]] || (Model->geom_contype[g] == 0 && Model->geom_conaffinity[g] == 0))
			{
				continue;
			}
			// The geom's lowest point in the world, exactly for each shape (its
			// bounding box turned by the pose sticks a corner out under a wheel
			// by nearly half the radius, and the base was lifted by that).
			const mjtNum* R = Data->geom_xmat + 9 * g; // R[6..8]: the world z row
			const mjtNum* X = Data->geom_xpos + 3 * g;
			const mjtNum* Size = Model->geom_size + 3 * g;
			const double  Radial = FMath::Sqrt(R[6] * R[6] + R[7] * R[7]);
			double		  Z = TNumericLimits<double>::Max();
			switch (Model->geom_type[g])
			{
				case mjGEOM_SPHERE:
					Z = X[2] - Size[0];
					break;
				case mjGEOM_CAPSULE:
					Z = X[2] - Size[0] - Size[1] * FMath::Abs(R[8]);
					break;
				case mjGEOM_CYLINDER:
					Z = X[2] - Size[0] * Radial - Size[1] * FMath::Abs(R[8]);
					break;
				case mjGEOM_ELLIPSOID:
					Z = X[2] - FMath::Sqrt(FMath::Square(R[6] * Size[0]) + FMath::Square(R[7] * Size[1]) + FMath::Square(R[8] * Size[2]));
					break;
				case mjGEOM_BOX:
					Z = X[2] - FMath::Abs(R[6]) * Size[0] - FMath::Abs(R[7]) * Size[1] - FMath::Abs(R[8]) * Size[2];
					break;
				case mjGEOM_MESH:
					if (Model->geom_dataid[g] >= 0)
					{
						const int32	 M = Model->geom_dataid[g];
						const float* V = Model->mesh_vert + 3 * Model->mesh_vertadr[M];
						for (int32 v = 0; v < Model->mesh_vertnum[M]; ++v)
						{
							Z = FMath::Min(Z, X[2] + R[6] * V[3 * v] + R[7] * V[3 * v + 1] + R[8] * V[3 * v + 2]);
						}
					}
					break;
				default: // a plane, a height field: the ground itself
					break;
			}
			if (Z < Lowest)
			{
				Lowest = Z;
				LowestGeom = g;
			}
		}
		if (OutPoint && LowestGeom >= 0)
		{
			*OutPoint = FVector(Data->geom_xpos[3 * LowestGeom], Data->geom_xpos[3 * LowestGeom + 1], Lowest);
		}
		if (OutGeom)
		{
			const char* Name = LowestGeom >= 0 ? mj_id2name(Model, mjOBJ_GEOM, LowestGeom) : nullptr;
			const char* Body = LowestGeom >= 0 ? mj_id2name(Model, mjOBJ_BODY, Model->geom_bodyid[LowestGeom]) : nullptr;
			*OutGeom = LowestGeom < 0 ? TEXT("none") : FString::Printf(TEXT("%s (type %d) of %s"), Name ? UTF8_TO_TCHAR(Name) : TEXT("?"), LowestGeom >= 0 ? (int32)Model->geom_type[LowestGeom] : -1, Body ? UTF8_TO_TCHAR(Body) : TEXT("?"));
		}
		for (int32 j = 0; j < Model->njnt; ++j)
		{
			if (Model->jnt_type[j] == mjJNT_FREE && Mine[Model->jnt_bodyid[j]])
			{
				OutFreeJoint = j;
				break;
			}
		}
		return Lowest;
	}
} // namespace

namespace
{
	const FName SimReset(TEXT("sim.reset"));
	const FName SimPause(TEXT("sim.pause"));
	const FName SimStep(TEXT("sim.step"));
	const FName SimRunning(TEXT("sim.running"));
	const FName SimFrameLock(TEXT("sim.frame_lock"));
	const FName DbgContacts(TEXT("sim.debug.contacts"));
	const FName DbgVisuals(TEXT("sim.debug.visuals"));
	const FName DbgCollisions(TEXT("sim.debug.collisions"));
	const FName DbgJoints(TEXT("sim.debug.joints"));
	const FName DbgQuickConvert(TEXT("sim.debug.quick_convert_collisions"));
	const FName DbgShader(TEXT("sim.debug.shader_mode"));
	const FName DbgTendons(TEXT("sim.debug.tendons"));

	const FName SimFrameLockState(TEXT("sim.frame_lock.state"));

	// sim.frame_lock: URLab's frame-locked stepping, urlab.LockedFrameSeconds. The choices, in order: off (0),
	// matched to the frame (below 0: each batch the real time its frame took), and fixed -- 1/16 s (the curb
	// runner's locked period: 31 steps of 2 ms), or whatever fixed period is already set elsewhere (the console, the
	// runner), which choosing it keeps. sim.frame_lock.state says in words exactly what is set.
	constexpr float FrameLockMatched = -1.0f;
	constexpr float FrameLockFixed = 0.0625f;

	/** URLab's variable; null with a URLab that has no frame lock (the control is then not offered). */
	IConsoleVariable* FrameLockVariable()
	{
		return IConsoleManager::Get().FindConsoleVariable(TEXT("urlab.LockedFrameSeconds"));
	}

	/** The choice the variable holds now (any fixed period is the fixed choice); -1 without it. */
	int32 FrameLockChoice()
	{
		const IConsoleVariable* Var = FrameLockVariable();
		if (!Var)
		{
			return -1;
		}
		const float Seconds = Var->GetFloat();
		return Seconds == 0.0f ? 0 : (Seconds < 0.0f ? 1 : 2);
	}
} // namespace

void URammsMjSimControlComponent::BeginPlay()
{
	Super::BeginPlay();
	if (!InitialKeyframe.IsEmpty() && !ApplyInitialKeyframe())
	{
		// The scene compiles in the manager's BeginPlay, in no fixed order
		// with this one: again next tick (and a drive mode's initial stance
		// is also taken next tick, from where the joints then are).
		RetryInitialKeyframe();
	}
}

bool URammsMjSimControlComponent::ApplyInitialKeyframe()
{
	AMjArticulation* Art = Cast<AMjArticulation>(GetOwner());
	AAMjManager*	 Manager = GetManager();
	if (!Art || !Manager || !Manager->PhysicsEngine || !Manager->PhysicsEngine->GetModel())
	{
		return false;
	}
	// The pose changes where the robot's lowest point is against the ground
	// it was spawned on: the base is lifted (or lowered) by that much, so the
	// new pose stands where the old one did rather than in the ground --
	// teleported in place, a stance with the rear wheels extended went 4.5 cm
	// into the floor, and the pop-out spun a wheel and yawed the robot.
	mjModel*	  Model = Manager->PhysicsEngine->GetModel();
	mjData*		  Data = Manager->PhysicsEngine->GetData();
	const FString Prefix = Art->GetCompiledPrefix();
	int32		  FreeJoint = -1;
	mj_kinematics(Model, Data);
	int32		 Bodies = 0;
	FString		 BeforeGeom, AfterGeom;
	const double Before = LowestPointOf(Model, Data, Prefix, FreeJoint, &Bodies, &BeforeGeom);
	double		 After = Before;
	// The joints written one by one through the engine, under its lock
	// (AMjArticulation::ResetToKeyframe writes the data directly, and with
	// the physics thread stepping the writes were lost: the ctrl stuck, the
	// joints went on from the zero pose and the servos dragged the robot
	// into its stance).
	int32 KeyId = mj_name2id(Model, mjOBJ_KEY, TCHAR_TO_UTF8(*(Prefix + InitialKeyframe)));
	if (KeyId < 0)
	{
		KeyId = mj_name2id(Model, mjOBJ_KEY, TCHAR_TO_UTF8(*InitialKeyframe));
	}
	const bool bOk = KeyId >= 0;
	if (bOk)
	{
		const mjtNum* KeyQpos = Model->key_qpos + KeyId * Model->nq;
		for (int32 j = 0; j < Model->njnt; ++j)
		{
			const char* BodyName = mj_id2name(Model, mjOBJ_BODY, Model->jnt_bodyid[j]);
			if (Model->jnt_type[j] == mjJNT_FREE || Model->jnt_type[j] == mjJNT_BALL || !BodyName
				|| !FString(UTF8_TO_TCHAR(BodyName)).StartsWith(Prefix))
			{
				continue;
			}
			Manager->PhysicsEngine->ApplyJointPosition(j, KeyQpos[Model->jnt_qposadr[j]]);
			Manager->PhysicsEngine->ApplyJointVelocity(j, 0.0);
		}
		// And the key's controls staged for its actuators: until the drive
		// mode's first commands (next tick) the servos would hold ctrl 0, the
		// zero pose, and yank the legs and seat toward it for a few steps.
		Art->ControlSource = 1; // the internal slot, as the actuation backend selects
		int32		  Staged = 0;
		const mjtNum* KeyCtrl = Model->key_ctrl + KeyId * Model->nu;
		for (int32 a = 0; a < Model->nu; ++a)
		{
			const char* Name = mj_id2name(Model, mjOBJ_ACTUATOR, a);
			FString		Authored = Name ? FString(UTF8_TO_TCHAR(Name)) : FString();
			if (!Authored.StartsWith(Prefix))
			{
				continue;
			}
			Authored.RightChopInline(Prefix.Len());
			if (UMjNodeComponent* Actuator = Art->GetActuator(Authored))
			{
				UMjActuatorRuntime::SetControl(Actuator, KeyCtrl[a]);
				++Staged;
			}
		}
		UE_LOG(LogTemp, Log, TEXT("[SimControl] '%s': %d of the key's controls staged"), *GetNameSafe(GetOwner()), Staged);
	}
	double	Lift = 0.0;
	double	Ground = TNumericLimits<double>::Lowest();
	FString Casts;
	if (bOk && FreeJoint >= 0)
	{
		mj_kinematics(Model, Data);
		FVector Point = FVector::ZeroVector;
		After = LowestPointOf(Model, Data, Prefix, FreeJoint, nullptr, &AfterGeom, &Point);
		// The ground under the pose's lowest point decides the lift: the
		// map's placement was for another pose, and the wheels may already
		// be in the floor. A ray cast down that column with the robot moved
		// out of it (its own wheel's surface can sit exactly at the floor's
		// top, and stepping past it stepped past the floor) finds the top.
		// Without a hit the lowest point is kept where the placement had it.
		Lift = Before - After;
		const int32	 ZAdr = Model->jnt_qposadr[FreeJoint] + 2;
		const mjtNum Down[3] = { 0.0, 0.0, -1.0 };
		const mjtNum Start[3] = { Point.X, Point.Y, After + 0.5 };
		int32		 HitGeom = -1;
		Data->qpos[ZAdr] += 100.0;
		mj_kinematics(Model, Data);
		const mjtNum Dist = mj_ray(Model, Data, Start, Down, nullptr, 1, -1, &HitGeom, nullptr);
		Data->qpos[ZAdr] -= 100.0;
		mj_kinematics(Model, Data);
		if (Dist >= 0.0 && HitGeom >= 0)
		{
			Ground = Start[2] - Dist;
			Lift = Ground - After;
			const char* HitName = mj_id2name(Model, mjOBJ_GEOM, HitGeom);
			const char* HitBody = mj_id2name(Model, mjOBJ_BODY, Model->geom_bodyid[HitGeom]);
			Casts = FString::Printf(TEXT("%s of %s"), HitName ? UTF8_TO_TCHAR(HitName) : TEXT("?"), HitBody ? UTF8_TO_TCHAR(HitBody) : TEXT("?"));
		}
		if (FMath::Abs(Lift) > 1e-4)
		{
			Data->qpos[Model->jnt_qposadr[FreeJoint] + 2] += Lift;
		}
		// And out of the ground: spawned with its wheels in the floor (the
		// map's placement), the robot was pushed out over the first half
		// second, its wheels spinning and the robot yawing. The deepest
		// contact of its geoms, after a forward pass, is how far.
		Manager->PhysicsEngine->ForwardSync();
		double Depth = 0.0;
		for (int32 i = 0; i < Data->ncon; ++i)
		{
			const mjContact& C = Data->contact[i];
			const FString	 Body1 = UTF8_TO_TCHAR(mj_id2name(Model, mjOBJ_BODY, Model->geom_bodyid[C.geom1]));
			const FString	 Body2 = UTF8_TO_TCHAR(mj_id2name(Model, mjOBJ_BODY, Model->geom_bodyid[C.geom2]));
			if ((Body1.StartsWith(Prefix) || Body2.StartsWith(Prefix)) && C.dist < 0.0)
			{
				Depth = FMath::Max(Depth, -C.dist);
			}
		}
		if (Depth > 1e-4)
		{
			Data->qpos[Model->jnt_qposadr[FreeJoint] + 2] += Depth + 0.002;
			Lift += Depth + 0.002;
			Manager->PhysicsEngine->ForwardSync();
		}
	}
	UE_LOG(LogTemp, Log,
		TEXT("[SimControl] '%s': initial keyframe '%s' %s (the base %+.1f cm for the pose's lowest point: %.3f m at %s -> %.3f m at %s, the ground %s, over %d bodies named '%s*', free joint %d)"),
		*GetNameSafe(GetOwner()), *InitialKeyframe, bOk ? TEXT("applied") : TEXT("not found"), Lift * 100.0, Before, *BeforeGeom, After, *AfterGeom,
		*(Ground > TNumericLimits<double>::Lowest() ? FString::Printf(TEXT("at %.3f m, %s"), Ground, *Casts) : FString(TEXT("not found under it"))), Bodies, *Prefix, FreeJoint);
	return true;
}

void URammsMjSimControlComponent::RetryInitialKeyframe()
{
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().SetTimerForNextTick(FTimerDelegate::CreateWeakLambda(this, [this]() {
			if (!ApplyInitialKeyframe() && ++InitialKeyframeTries < 120)
			{
				RetryInitialKeyframe();
			}
		}));
	}
}

AAMjManager* URammsMjSimControlComponent::GetManager() const
{
	if (AAMjManager* M = AAMjManager::GetManager())
	{
		return M;
	}
	return Cast<AAMjManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AAMjManager::StaticClass()));
}

void URammsMjSimControlComponent::DescribeControls(FRammsControlSurface& OutSurface) const
{
	AAMjManager* Manager = GetManager();
	if (!Manager)
	{
		return;
	}
	auto Action = [&OutSurface](FName Id, FName Group, const TCHAR* Name, int32 Order) {
		FRammsControlAxis Axis;
		Axis.Id = Id;
		Axis.Group = Group;
		Axis.DisplayName = FText::FromString(Name);
		Axis.Kind = ERammsControlKind::Action;
		Axis.Units = ERammsControlUnits::None;
		Axis.Order = Order;
		OutSurface.Add(Axis);
	};
	const FName Sim("Sim");
	Action(SimReset, Sim, TEXT("Reset"), 0);
	Action(SimPause, Sim, TEXT("Pause / resume"), 1);
	Action(SimStep, Sim, TEXT("Step"), 2);

	FRammsControlAxis Running;
	Running.Id = SimRunning;
	Running.Group = Sim;
	Running.DisplayName = FText::FromString(TEXT("Running"));
	Running.Kind = ERammsControlKind::Position;
	Running.Units = ERammsControlUnits::Normalized;
	Running.Range = FVector2D(0.0, 1.0);
	Running.DefaultValue = 1.0f;
	Running.bReadback = true;
	Running.Order = 3;
	OutSurface.Add(Running);

	if (FrameLockVariable())
	{
		// Read back from the variable itself (its value and its target both), so it shows a change from the
		// console, the curb runner, or URLab turning the lock off after a stalled batch.
		FRammsControlAxis Lock;
		Lock.Id = SimFrameLock;
		Lock.Group = Sim;
		Lock.DisplayName = FText::FromString(TEXT("Frame lock"));
		Lock.Kind = ERammsControlKind::Enum;
		Lock.Units = ERammsControlUnits::None;
		Lock.EnumLabels = { FText::FromString(TEXT("Off")), FText::FromString(TEXT("Matched")), FText::FromString(TEXT("1/16 s")) };
		Lock.Range = FVector2D(0.0, 2.0);
		Lock.DefaultValue = 0.0f;
		Lock.bReadback = true;
		Lock.Order = 4;
		OutSurface.Add(Lock);

		// What is set, in words: the period and its steps, or that it is matched or off.
		FRammsControlAxis State;
		State.Id = SimFrameLockState;
		State.Group = Sim;
		State.DisplayName = FText::FromString(TEXT("Frame lock is"));
		State.Kind = ERammsControlKind::Text;
		State.Units = ERammsControlUnits::None;
		State.bReadOnly = true;
		State.Range = FVector2D(0.0, static_cast<double>(ERammsControlTextSeverity::Fault));
		State.Order = 5;
		OutSurface.Add(State);
	}

	if (bExposeDebugToggles && Manager->DebugVisualizer)
	{
		const FName Dbg("Sim Debug");
		Action(DbgContacts, Dbg, TEXT("Contacts"), 0);
		Action(DbgVisuals, Dbg, TEXT("Visual meshes"), 1);
		Action(DbgCollisions, Dbg, TEXT("Collision wireframes"), 2);
		Action(DbgJoints, Dbg, TEXT("Joint axes"), 3);
		Action(DbgQuickConvert, Dbg, TEXT("Quick-convert collisions"), 4);
		Action(DbgShader, Dbg, TEXT("Cycle overlay shader"), 5);
		Action(DbgTendons, Dbg, TEXT("Tendons"), 6);
	}
}

bool URammsMjSimControlComponent::ApplyControl(FName Id, float Value)
{
	if (Id == SimFrameLock)
	{
		IConsoleVariable* Var = FrameLockVariable();
		if (!Var)
		{
			return false;
		}
		const int32 Choice = FMath::Clamp(FMath::RoundToInt(Value), 0, 2);
		if (Choice == 2 && Var->GetFloat() > 0.0f)
		{
			return true; // already fixed: the period set elsewhere (the runner's, the console's) stays
		}
		// As the console sets it (the highest priority), so the choice holds whoever set it before. It is a console
		// variable: it lasts beyond this session, as one set from the console does.
		Var->Set(Choice == 0 ? 0.0f : (Choice == 1 ? FrameLockMatched : FrameLockFixed), ECVF_SetByConsole);
		return true;
	}
	// sim.running is writable too: 1 = run, 0 = pause.
	AAMjManager* Manager = GetManager();
	if (Id != SimRunning || !Manager)
	{
		return false;
	}
	Manager->SetPaused(Value < 0.5f);
	return true;
}

bool URammsMjSimControlComponent::TriggerControl(FName Id)
{
	if (Id == SimFrameLock)
	{
		// The next choice, as an Enum's trigger does: Off -> Matched -> fixed -> Off.
		const int32 Choice = FrameLockChoice();
		return Choice >= 0 && ApplyControl(Id, static_cast<float>(Choice == 0 ? 1 : (Choice == 1 ? 2 : 0)));
	}
	AAMjManager* Manager = GetManager();
	if (!Manager)
	{
		return false;
	}
	if (Id == SimReset)
	{
		Manager->ResetSimulation();
		return true;
	}
	if (Id == SimPause)
	{
		Manager->SetPaused(Manager->IsRunning());
		return true;
	}
	if (Id == SimStep)
	{
		Manager->StepSync(FMath::Max(1, StepCount));
		return true;
	}
	UMjDebugVisualizer* Dbg = Manager->DebugVisualizer;
	if (!Dbg)
	{
		return false;
	}
	if (Id == DbgContacts)
	{
		Dbg->ToggleDebugContacts();
	}
	else if (Id == DbgVisuals)
	{
		Dbg->ToggleVisuals();
	}
	else if (Id == DbgCollisions)
	{
		Dbg->ToggleArticulationCollisions();
	}
	else if (Id == DbgJoints)
	{
		Dbg->ToggleDebugJoints();
	}
	else if (Id == DbgQuickConvert)
	{
		Dbg->ToggleQuickConvertCollisions();
	}
	else if (Id == DbgShader)
	{
		Dbg->CycleDebugShaderMode();
	}
	else if (Id == DbgTendons)
	{
		Dbg->ToggleTendons();
	}
	else
	{
		return false;
	}
	return true;
}

bool URammsMjSimControlComponent::ReadControl(FName Id, float& OutValue) const
{
	if (Id == SimFrameLock)
	{
		const int32 Choice = FrameLockChoice();
		OutValue = static_cast<float>(Choice);
		return Choice >= 0;
	}
	if (Id == SimFrameLockState)
	{
		OutValue = static_cast<float>(ERammsControlTextSeverity::Info); // its words are information, never a warning
		return FrameLockVariable() != nullptr;
	}
	AAMjManager* Manager = GetManager();
	if (Id != SimRunning || !Manager)
	{
		return false;
	}
	OutValue = Manager->IsRunning() ? 1.0f : 0.0f;
	return true;
}

bool URammsMjSimControlComponent::ReadControlText(FName Id, FText& OutText) const
{
	if (Id != SimFrameLockState)
	{
		return false;
	}
	const IConsoleVariable* Var = FrameLockVariable();
	if (!Var)
	{
		return false;
	}
	const float	 Seconds = Var->GetFloat();
	AAMjManager* Manager = GetManager();
	const double Dt = Manager && Manager->PhysicsEngine ? Manager->PhysicsEngine->GetTimestep() : 0.0;
	if (Seconds == 0.0f)
	{
		OutText = FText::FromString(TEXT("off: MuJoCo runs on its own, paced to the wall clock"));
	}
	else if (Seconds < 0.0f)
	{
		OutText = FText::FromString(TEXT("matched: each frame's real time, in whole steps"));
	}
	else
	{
		const FString Period = Seconds == FrameLockFixed ? FString(TEXT("1/16 s")) : FString::Printf(TEXT("%g ms"), Seconds * 1000.0f);
		OutText = Dt > 0.0
			? FText::FromString(FString::Printf(TEXT("%s: %d steps of %g ms a frame"), *Period, FMath::Max(1, FMath::RoundToInt(Seconds / Dt)), Dt * 1000.0))
			: FText::FromString(Period + TEXT(" a frame"));
	}
	return true;
}

bool URammsMjSimControlComponent::ReadTarget(FName Id, float& OutTarget) const
{
	// The frame lock's target is whatever the variable holds now: something other than this surface (the console,
	// the runner, URLab's stall reset) changes it, and a stepper steps from the target. sim.running keeps the
	// default (no target).
	return Id == SimFrameLock && ReadControl(Id, OutTarget);
}
