// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsMujocoTestGameMode.h"
#include "RammsRobotSelection.h"
#include "RammsRobotGameMode.generated.h"

/**
 * Starts a level with the robot chosen at start, so the level itself names
 * none: a map that sets this C++ class as its GameMode Override references no
 * robot's content, and the same map runs any robot there is a definition (or
 * a pawn class path) for.
 *
 * The robot is chosen in InitGame (URammsRobotCatalog::Resolve): ?Robot= on
 * the map URL, then ramms.Robot (-RammsRobot=), then DefaultRobotId here, then
 * the project's default (URammsRobotSettings, which a mounted plugin's
 * Game.ini can set), then the only robot there is; with none, the picker pawn
 * (ARammsRobotPickerPawn) stands at the start. It is spawned there before the
 * MuJoCo scene compiles, as ARammsMujocoTestGameMode does, and carries the
 * robot choice on its control surface (URammsRobotSelectComponent) to start
 * the level again with another.
 *
 * The level sets no MuJoCo scene options for the robot: with none set, each
 * robot's own MJCF <option> is adopted as it attaches.
 */
UCLASS()
class RAMMSMUJOCOSUPPORT_API ARammsRobotGameMode : public ARammsMujocoTestGameMode
{
	GENERATED_BODY()

public:
	ARammsRobotGameMode();

	/** This level's robot when neither the URL nor ramms.Robot chooses one (an
	 *  id, an alias or a pawn class path; empty for the project's default). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot")
	FString DefaultRobotId;

	/** Put the robot choice (sim.robot, sim.robot.load) on the robot's control
	 *  surface. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot")
	bool bAddRobotSelectControls = true;

	virtual void InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage) override;

	/** The robot this level started with. */
	UFUNCTION(BlueprintCallable, Category = "Robot")
	FRammsRobotChoice GetRobotChoice() const { return Choice; }

protected:
	virtual FTransform GetPreSpawnTransform() const override;

private:
	/** The chosen robot's pawn class (and its player controller's, if it names
	 *  one); false when it cannot be loaded. */
	bool LoadRobotClasses(FString& OutWhy);

	UPROPERTY(Transient)
	FRammsRobotChoice Choice;
};
