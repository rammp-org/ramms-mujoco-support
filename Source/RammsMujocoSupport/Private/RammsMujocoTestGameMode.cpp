// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMujocoTestGameMode.h"
#include "MuJoCo/Core/AMjManager.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/PlayerStart.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"

ARammsMujocoTestGameMode::ARammsMujocoTestGameMode()
{
	// A plain controller: the pawn's teleop component polls keys itself and
	// there is no vehicle to put a HUD on.
	PlayerControllerClass = APlayerController::StaticClass();
	HUDClass = nullptr;
	DefaultPawnClass = nullptr;
}

void ARammsMujocoTestGameMode::InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage)
{
	Super::InitGame(MapName, Options, ErrorMessage);

	if (!bSpawnDefaultPawnBeforeCompile || !DefaultPawnClass)
	{
		return;
	}

	// Where the player would normally appear: the first PlayerStart, else the
	// world origin. (FindPlayerStart needs a controller; none exists yet.)
	FTransform SpawnTransform = FTransform::Identity;
	if (AActor* Start = UGameplayStatics::GetActorOfClass(GetWorld(), APlayerStart::StaticClass()))
	{
		SpawnTransform = Start->GetActorTransform();
		SpawnTransform.SetScale3D(FVector::OneVector);
	}

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	PreSpawnedPawn = GetWorld()->SpawnActor<APawn>(DefaultPawnClass, SpawnTransform, Params);
	UE_LOG(LogTemp, Log, TEXT("[MujocoTestGameMode] pre-spawned default pawn %s (%s) at %s so the MuJoCo scene compiles with it."),
		PreSpawnedPawn ? *PreSpawnedPawn->GetName() : TEXT("<failed>"), *DefaultPawnClass->GetName(),
		*SpawnTransform.GetLocation().ToString());
}

APawn* ARammsMujocoTestGameMode::SpawnDefaultPawnAtTransform_Implementation(AController* NewPlayer, const FTransform& SpawnTransform)
{
	// Hand the pre-spawned pawn to the first player instead of spawning a
	// second (unsimulated) one.
	if (PreSpawnedPawn && !PreSpawnedPawn->GetController())
	{
		APawn* Pawn = PreSpawnedPawn;
		PreSpawnedPawn = nullptr;
		return Pawn;
	}
	return Super::SpawnDefaultPawnAtTransform_Implementation(NewPlayer, SpawnTransform);
}

void ARammsMujocoTestGameMode::StartPlay()
{
	Super::StartPlay();

	if (bStartSimulationOnBeginPlay)
	{
		StartSimElapsed = 0.0f;
		bWidgetHidden = false;
		GetWorldTimerManager().SetTimer(StartSimTimer, this, &ARammsMujocoTestGameMode::TryStartSimulation, 0.25f, true, 0.0f);
	}
}

void ARammsMujocoTestGameMode::TryStartSimulation()
{
	StartSimElapsed += 0.25f;
	AAMjManager* Manager = Cast<AAMjManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AAMjManager::StaticClass()));
	if (Manager)
	{
		if (!Manager->IsRunning())
		{
			Manager->SetPaused(false);
		}
		if (Manager->IsRunning())
		{
			if (bHideSimulateWidget && !bWidgetHidden)
			{
				Manager->ToggleSimulateWidget();
				bWidgetHidden = true;
			}
			UE_LOG(LogTemp, Log, TEXT("[MujocoTestGameMode] MuJoCo simulation running (after %.2fs)."), StartSimElapsed);
			GetWorldTimerManager().ClearTimer(StartSimTimer);
			return;
		}
	}
	if (StartSimElapsed >= SimulationStartTimeoutSeconds)
	{
		UE_LOG(LogTemp, Warning, TEXT("[MujocoTestGameMode] gave up starting the MuJoCo simulation after %.1fs (%s)."),
			StartSimElapsed, Manager ? TEXT("manager present but not running") : TEXT("no AMjManager in the level"));
		GetWorldTimerManager().ClearTimer(StartSimTimer);
	}
}
