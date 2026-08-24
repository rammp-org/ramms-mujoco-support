// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
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

	/**
	 * List a physics asset's body bone names (SkeletalBodySetups is not
	 * python-readable) so scripts can target per-body edits.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Rig")
	static TArray<FString> ListPhysicsAssetBodies(UPhysicsAsset* PhysicsAsset)
	{
		TArray<FString> Names;
		if (PhysicsAsset)
		{
			for (USkeletalBodySetup* Setup : PhysicsAsset->SkeletalBodySetups)
			{
				if (Setup)
				{
					Names.Add(Setup->BoneName.ToString());
				}
			}
		}
		return Names;
	}

	/**
	 * Assign a physical material to every physics-asset body whose bone name
	 * contains the given substring (case-insensitive). Returns the number of
	 * bodies changed. Used to give the hand-built vehicle bases the same
	 * MJCF-derived contact materials as the generated rigs (e.g. low-friction
	 * caster wheels).
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Rig")
	static int32 SetPhysicsAssetBodyMaterial(
		UPhysicsAsset* PhysicsAsset, const FString& BoneContains, UPhysicalMaterial* Material)
	{
		int32 Changed = 0;
		if (!PhysicsAsset || BoneContains.IsEmpty())
		{
			return 0;
		}
		for (USkeletalBodySetup* Setup : PhysicsAsset->SkeletalBodySetups)
		{
			if (Setup && Setup->BoneName.ToString().Contains(BoneContains)
				&& Setup->PhysMaterial != Material)
			{
				Setup->Modify();
				Setup->PhysMaterial = Material;
				++Changed;
			}
		}
		if (Changed > 0)
		{
			PhysicsAsset->MarkPackageDirty();
		}
		return Changed;
	}
};
