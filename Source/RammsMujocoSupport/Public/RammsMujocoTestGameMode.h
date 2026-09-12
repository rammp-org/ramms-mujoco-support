// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "RammsMujocoTestGameMode.generated.h"

class AAMjManager;

/**
 * Game mode for test-driving a MuJoCo robot pawn (a URLab articulation with a
 * RammsRobotBaseComponent + RammsKeyboardTeleopComponent, e.g.
 * BP_LiftDriveHolonomic_Ramms / BP_LiftDriveLinkage_Ramms) as the player.
 *
 * Two things a stock AGameModeBase gets wrong for a MuJoCo pawn:
 *
 *  - The AMjManager compiles the scene from the articulations that exist at
 *    its BeginPlay; a default pawn spawned at player login (after begin-play)
 *    would never be simulated. So the DefaultPawnClass is spawned early, in
 *    InitGame, at the PlayerStart, and handed to the normal possession path
 *    from SpawnDefaultPawnAtTransform.
 *  - The MuJoCo engine starts paused. Once the world has begun play this mode
 *    unpauses it (retrying until the manager reports running), and can hide
 *    the simulate widget.
 *
 * Subclass in Blueprint to pick the robot: set DefaultPawnClass to the pawn
 * Blueprint. PlayerControllerClass defaults to a plain APlayerController (the
 * teleop component polls keys itself; no vehicle HUD).
 */
UCLASS()
class RAMMSMUJOCOSUPPORT_API ARammsMujocoTestGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	ARammsMujocoTestGameMode();

	/** Unpause the MuJoCo simulation once play begins. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	bool bStartSimulationOnBeginPlay = true;

	/** Keep retrying the unpause for this long (the engine compiles/starts a
	 *  little after begin-play). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo", meta = (ClampMin = "0.0"))
	float SimulationStartTimeoutSeconds = 15.0f;

	/** Hide the manager's simulate widget once the sim is running (the pawn is
	 *  driven by keyboard; the panel mostly covers the view). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	bool bHideSimulateWidget = true;

	/** Spawn DefaultPawnClass in InitGame (before the scene compiles) rather
	 *  than at player login, so the pawn is part of the MuJoCo scene. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	bool bSpawnDefaultPawnBeforeCompile = true;

	virtual void   InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage) override;
	virtual APawn* SpawnDefaultPawnAtTransform_Implementation(AController* NewPlayer, const FTransform& SpawnTransform) override;
	virtual void   StartPlay() override;

private:
	void TryStartSimulation();

	UPROPERTY(Transient)
	TObjectPtr<APawn> PreSpawnedPawn;

	FTimerHandle StartSimTimer;
	float		 StartSimElapsed = 0.0f;
	bool		 bWidgetHidden = false;
};
