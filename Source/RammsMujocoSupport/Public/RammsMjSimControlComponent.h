// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsControlContributor.h"
#include "RammsMjSimControlComponent.generated.h"

class AAMjManager;

/**
 * Contributes the MuJoCo scene's simulation controls to the owning robot's
 * control surface, so panels / input maps / remote clients get the functions
 * URLab's UMjInputHandler binds to hotkeys (R reset, P pause, 1-7 debug
 * toggles) without that handler — which is disabled in RAMMS game modes
 * because its keys collide with robot controls.
 *
 *   sim.reset, sim.pause (toggle), sim.step               — group "Sim"
 *   sim.debug.contacts / visuals / collisions / joints /
 *   quick_convert_collisions / shader_mode / tendons       — group "Sim Debug"
 *   sim.running                                            — readback (1 = running)
 *
 * Add one to any robot actor in a MuJoCo scene; it finds the AMjManager.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSMUJOCOSUPPORT_API URammsMjSimControlComponent : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	/** Steps StepSync runs for sim.step. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Sim Control", meta = (ClampMin = "1"))
	int32 StepCount = 1;

	/** Offer the debug-visualizer toggles (group "Sim Debug"). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Sim Control")
	bool bExposeDebugToggles = true;

	UFUNCTION(BlueprintPure, Category = "Ramms|Sim Control")
	AAMjManager* GetManager() const;

	// --- IRammsControlContributor: "sim.*" ------------------------------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual int32 GetControlOrder() const override { return 200; }
};
