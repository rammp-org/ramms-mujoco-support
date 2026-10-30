// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsMjGroundSupportLibrary.generated.h"

class URammsRobotBaseComponent;

/** One robot body's contacts with the world, gathered: a wheel's, or another body's. */
USTRUCT(BlueprintType)
struct RAMMSMUJOCOSUPPORT_API FRammsMjBodyContact
{
	GENERATED_BODY()

	/** The MJCF body: for a wheel, the body its motor turns (every body under it -- an omni wheel's rollers -- is
	 *  gathered to it); otherwise the body that touches. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	FName Body;

	/** Where it bears (world cm): its contact points, weighted by their normal force (and time, over a window). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	FVector Point = FVector::ZeroVector;

	/** Its contacts' normal force together (N), its mean over the window. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	float NormalForceN = 0.0f;

	/** How many contacts bear (MuJoCo's, each with a normal force over zero), the most in any one step of the
	 *  window; 0 for a wheel off the ground throughout. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	int32 Contacts = 0;

	/** The share of the window's time it bore at all (0..1): under 1, it touched and left -- set down, lifted, or
	 *  chattering. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	float BearingShare = 0.0f;
};

/** What a MuJoCo-simulated robot stands on, as the physics has it, and where its mass is. */
USTRUCT(BlueprintType)
struct RAMMSMUJOCOSUPPORT_API FRammsMjGroundSupport
{
	GENERATED_BODY()

	/** False when the robot is not simulated by MuJoCo or its scene has not compiled; nothing else is set. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	bool bValid = false;

	/** The physics' time at the read (s). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	double Time = 0.0;

	/** The simulated time the forces and the centre of mass are averaged over (s): the window asked for, or less
	 *  where fewer steps have been recorded; 0 for the one step of the first read. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	float WindowSeconds = 0.0f;

	/** The whole robot's centre of mass (world cm): its root body and everything it carries, payload bodies too;
	 *  its mean over the window. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	FVector CenterOfMass = FVector::ZeroVector;

	/** The whole robot's mass (kg). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	float MassKg = 0.0f;

	/** One per wheel motor asked about, in that order: Contacts 0 where it bears nothing, Body empty where the motor
	 *  does not resolve to a joint's body. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	TArray<FRammsMjBodyContact> Wheels;

	/** Every other robot body that bears on the world (a chassis on a curb's lip), one entry per body. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ramms|Ground Support")
	TArray<FRammsMjBodyContact> Others;
};

/**
 * The ground under a MuJoCo-simulated robot as the physics has it, for measuring how a robot stands rather than how
 * its controller believes it does: which wheels bear, where and how hard, what else touches, and where the whole
 * robot's centre of mass is.
 *
 * A single step's contact forces are a poor measure of load: a soft contact's force swings from step to step, and an
 * omni wheel hands its load from roller to roller, so one step can read a loaded wheel as bearing nothing or a pair of
 * wheels as carrying a fifth more than the robot weighs. So the first read for a robot starts a recorder on the
 * engine's physics thread that keeps every step's contacts for the last second of simulated time, and each read
 * averages them over the window it asks for, as a load cell filters (the first read returns the latest step alone).
 */
UCLASS()
class RAMMSMUJOCOSUPPORT_API URammsMjGroundSupportLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * The robot whose base component this is (its MuJoCo articulation: the base's owner or one attached under it),
	 * with its contacts gathered per wheel: WheelMotors are motor ids (actuator names), each naming the wheel its
	 * joint turns. Contacts between two of the robot's own bodies are left out. Forces and the centre of mass are
	 * their means over the last WindowSeconds of simulated time (at most a second).
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Ground Support")
	static FRammsMjGroundSupport GetGroundSupport(const URammsRobotBaseComponent* Base, const TArray<FName>& WheelMotors,
		float WindowSeconds = 0.05f);
};
