// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMjSimControlComponent.h"
#include "Kismet/GameplayStatics.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"

namespace
{
	const FName SimReset(TEXT("sim.reset"));
	const FName SimPause(TEXT("sim.pause"));
	const FName SimStep(TEXT("sim.step"));
	const FName SimRunning(TEXT("sim.running"));
	const FName DbgContacts(TEXT("sim.debug.contacts"));
	const FName DbgVisuals(TEXT("sim.debug.visuals"));
	const FName DbgCollisions(TEXT("sim.debug.collisions"));
	const FName DbgJoints(TEXT("sim.debug.joints"));
	const FName DbgQuickConvert(TEXT("sim.debug.quick_convert_collisions"));
	const FName DbgShader(TEXT("sim.debug.shader_mode"));
	const FName DbgTendons(TEXT("sim.debug.tendons"));
} // namespace

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
	AAMjManager* Manager = GetManager();
	if (Id != SimRunning || !Manager)
	{
		return false;
	}
	OutValue = Manager->IsRunning() ? 1.0f : 0.0f;
	return true;
}
