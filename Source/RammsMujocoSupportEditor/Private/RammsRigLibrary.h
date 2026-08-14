// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsRigLibrary.generated.h"

class UBlueprint;

namespace RammsRigGen
{
void GenerateChaosRigForBlueprint(UBlueprint* BP);
}

/** Python/Blueprint-callable surface for the Chaos rig generator. */
UCLASS()
class URammsRigLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Generate (or regenerate) the Chaos rig for an imported MJCF Blueprint. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Rig")
	static void GenerateChaosRig(UBlueprint* Blueprint)
	{
		RammsRigGen::GenerateChaosRigForBlueprint(Blueprint);
	}
};
