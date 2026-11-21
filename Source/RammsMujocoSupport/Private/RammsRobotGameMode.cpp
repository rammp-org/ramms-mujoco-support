// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotGameMode.h"
#include "RammsRobotDefinition.h"
#include "RammsRobotPickerPawn.h"
#include "RammsRobotSelectComponent.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsRobotGameMode, Log, All);

ARammsRobotGameMode::ARammsRobotGameMode()
{
	DefaultPawnClass = nullptr; // chosen in InitGame
}

bool ARammsRobotGameMode::LoadRobotClasses(FString& OutWhy)
{
	UClass* Pawn = nullptr;
	if (Choice.Definition)
	{
		Pawn = Choice.Definition->PawnClass.LoadSynchronous();
		if (!Pawn)
		{
			OutWhy = FString::Printf(TEXT("robot '%s': its pawn class %s does not load"), *Choice.GetId(), *Choice.Definition->PawnClass.ToString());
			return false;
		}
		if (!Choice.Definition->PlayerControllerClass.IsNull())
		{
			if (UClass* Controller = Choice.Definition->PlayerControllerClass.LoadSynchronous())
			{
				PlayerControllerClass = Controller;
			}
			else
			{
				UE_LOG(LogRammsRobotGameMode, Warning, TEXT("[RobotGameMode] robot '%s': its player controller %s does not load; keeping %s"),
					*Choice.GetId(), *Choice.Definition->PlayerControllerClass.ToString(), *GetNameSafe(PlayerControllerClass));
			}
		}
	}
	else
	{
		Pawn = LoadClass<APawn>(nullptr, *Choice.ClassPath);
		if (!Pawn)
		{
			OutWhy = FString::Printf(TEXT("pawn class %s does not load (or is not a pawn)"), *Choice.ClassPath);
			return false;
		}
	}
	DefaultPawnClass = Pawn;
	return true;
}

void ARammsRobotGameMode::InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage)
{
	// Chosen before Super, whose pre-spawn reads DefaultPawnClass.
	Choice = URammsRobotCatalog::ResolveForWorld(Options, DefaultRobotId);
	for (const FString& Warning : Choice.Warnings)
	{
		UE_LOG(LogRammsRobotGameMode, Warning, TEXT("[RobotGameMode] %s"), *Warning);
	}
	FString Why;
	if (Choice.HasRobot() && !LoadRobotClasses(Why))
	{
		Choice.Error = Why;
		Choice.Definition = nullptr;
		Choice.ClassPath.Reset();
	}
	if (!Choice.HasRobot())
	{
		DefaultPawnClass = ARammsRobotPickerPawn::StaticClass();
		if (!Choice.Error.IsEmpty())
		{
			UE_LOG(LogRammsRobotGameMode, Error, TEXT("[RobotGameMode] %s: no robot started, the picker instead"), *Choice.Error);
		}
	}

	if (URammsRobotSelectionSubsystem* Selection = GetWorld()->GetSubsystem<URammsRobotSelectionSubsystem>())
	{
		Selection->SetActiveChoice(Choice);
	}

	Super::InitGame(MapName, Options, ErrorMessage);

	APawn* Pawn = GetPreSpawnedPawn();
	if (Pawn && Choice.HasRobot())
	{
		Pawn->Tags.AddUnique(FName(*FString::Printf(TEXT("RammsRobot:%s"), *Choice.GetId())));
		// Added before any actor begins play: the robot's control surface
		// gathers it with its own controls at BeginPlay.
		if (bAddRobotSelectControls && !Pawn->FindComponentByClass<URammsRobotSelectComponent>())
		{
			URammsRobotSelectComponent* Select = NewObject<URammsRobotSelectComponent>(Pawn, TEXT("RobotSelect"));
			Pawn->AddInstanceComponent(Select);
			Select->RegisterComponent();
		}
	}
	UE_LOG(LogRammsRobotGameMode, Log, TEXT("[RobotGameMode] %s: robot %s; pawn %s"), *MapName, *Choice.Describe(), *GetNameSafe(Pawn));
}

FTransform ARammsRobotGameMode::GetPreSpawnTransform() const
{
	FTransform Transform = Super::GetPreSpawnTransform();
	if (Choice.Definition && !Choice.Definition->SpawnOffset.IsNearlyZero())
	{
		Transform.SetLocation(Transform.TransformPosition(Choice.Definition->SpawnOffset));
	}
	return Transform;
}
