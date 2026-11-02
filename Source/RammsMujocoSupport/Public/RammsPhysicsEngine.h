// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Info.h"
#include "Subsystems/WorldSubsystem.h"
#include "RammsPhysicsEngine.generated.h"

class AActor;
class AMjArticulation;

/** Which engine simulates a level's URLab articulations (robots, SimReady objects). */
UENUM(BlueprintType)
enum class ERammsLevelPhysicsEngine : uint8
{
	/** Leave the level as it is: MuJoCo simulates what an AAMjManager compiles, Chaos what carries an active
	 *  URammsChaosArticulationComponent. */
	Auto UMETA(DisplayName = "Auto (as placed)"),
	/** MuJoCo for every articulation: an AAMjManager is spawned when the level has none, and Chaos
	 *  articulation components are turned off. */
	MuJoCo UMETA(DisplayName = "MuJoCo"),
	/** Chaos for every articulation: each gets a URammsChaosArticulationComponent, and the level's
	 *  AAMjManager is removed before it compiles anything. */
	Chaos UMETA(DisplayName = "Chaos"),
};

/**
 * Picks the physics engine for this level's articulations: place one in a level. The same assets then run
 * on either engine. On Chaos, an articulation whose Blueprint carries its own URammsChaosArticulationComponent
 * keeps that component's settings; the others get one with its defaults. A run can override it: -RammsPhysics=Chaos|MuJoCo on the command line, or the CVar
 * ramms.Physics.Engine (Chaos, MuJoCo, or empty for the level's choice).
 */
UCLASS(ClassGroup = (Ramms), hidecategories = (Actor, Input, Collision, Replication, Rendering, HLOD, Cooking, DataLayers, WorldPartition))
class RAMMSMUJOCOSUPPORT_API ARammsPhysicsSettings : public AInfo
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Physics")
	ERammsLevelPhysicsEngine Engine = ERammsLevelPhysicsEngine::Auto;

	/** On MuJoCo, unpause the simulation once it has compiled (URLab's manager starts paused). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Physics")
	bool bStartMuJoCoRunning = true;

	/** On MuJoCo, when the level gives MuJoCo no static geometry of its own (no UMjQuickConvertComponent
	 *  marked Static, no heightfield), add a flat ground at the floor under the articulations so a mobile
	 *  robot stands. Walls, curbs and furniture are not converted; a fixed object (a SimReady fridge) needs
	 *  none of it. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ramms|Physics")
	bool bMuJoCoGround = true;
};

/**
 * Applies the level's physics engine choice (ARammsPhysicsSettings, overridable per run) at world BeginPlay,
 * before any actor begins play, and to articulations spawned later. Robots follow: their base component's
 * actuation backend drives whichever engine simulates their articulation (Backend Auto or MuJoCo).
 */
UCLASS()
class RAMMSMUJOCOSUPPORT_API URammsPhysicsEngineSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	/** The engine this world's articulations run on (Auto: as placed). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Physics")
	ERammsLevelPhysicsEngine GetEngine() const { return Engine; }

	/** The run's override (-RammsPhysics= or ramms.Physics.Engine), else the level's ARammsPhysicsSettings. */
	static ERammsLevelPhysicsEngine ResolveEngine(const UWorld& World, bool* bOutStartMuJoCoRunning = nullptr, bool* bOutMuJoCoGround = nullptr);

	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
	virtual void OnWorldBeginPlay(UWorld& World) override;
	virtual void Deinitialize() override;

private:
	void ToChaos(AMjArticulation* Articulation);
	void OnActorSpawned(AActor* Actor);
	void TryStartMuJoCo();
	bool SpawnMuJoCoGround(UWorld& World);

	ERammsLevelPhysicsEngine Engine = ERammsLevelPhysicsEngine::Auto;
	FDelegateHandle			 SpawnHandle;
	FTimerHandle			 StartTimer;
	float					 StartElapsed = 0.0f;
};
