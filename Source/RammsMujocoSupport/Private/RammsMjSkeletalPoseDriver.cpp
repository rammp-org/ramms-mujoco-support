// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMjSkeletalPoseDriver.h"
#include "Components/PoseableMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjBody.h"
#include "GameFramework/Actor.h"
#include "EngineUtils.h"

URammsMjSkeletalPoseDriver::URammsMjSkeletalPoseDriver()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	// Run after URLab has pushed the physics snapshot onto the body components this frame.
	PrimaryComponentTick.TickGroup = TG_PostUpdateWork;
}

void URammsMjSkeletalPoseDriver::BeginPlay()
{
	Super::BeginPlay();
	Resolve();
}

bool URammsMjSkeletalPoseDriver::Resolve()
{
	AActor* Owner = GetOwner();
	if (!Owner)
		return false;

	if (!Poseable)
	{
		TArray<UPoseableMeshComponent*> Found;
		Owner->GetComponents<UPoseableMeshComponent>(Found);
		for (UPoseableMeshComponent* P : Found)
		{
			if (P && (PoseableMeshComponentName == NAME_None || P->GetFName() == PoseableMeshComponentName))
			{
				Poseable = P;
				break;
			}
		}
	}

	if (!Articulation)
	{
		if (AMjArticulation* AsArt = Cast<AMjArticulation>(Owner))
		{
			Articulation = AsArt;
		}
		else if (UWorld* World = GetWorld())
		{
			for (TActorIterator<AMjArticulation> It(World); It; ++It)
			{
				Articulation = *It;
				break;
			}
		}
	}

	return Poseable != nullptr && Articulation != nullptr;
}

void URammsMjSkeletalPoseDriver::BuildMapping()
{
	Pairs.Reset();
	if (!Articulation || !Poseable)
		return;

	const TArray<UMjBody*> Bodies = Articulation->GetBodies();
	if (Bodies.Num() == 0)
		return;  // articulation not compiled yet — retry next tick

	TArray<FString> Unmatched;
	for (UMjBody* Body : Bodies)
	{
		if (!Body)
			continue;

		// v0.6.0-beta: the authored name is a presence-wrapped TOptional on UMjNodeComponent.
		const FString MjName = Body->MjName.Get(FString());
		if (MjName.IsEmpty())
			continue;

		// Resolve the target bone: explicit override first, else name match (after prefix strip).
		FName Bone = NAME_None;
		if (const FName* Override = BodyToBoneOverrides.Find(FName(*MjName)))
		{
			Bone = *Override;
		}
		else if (bAutoMatchByName)
		{
			FString Stripped = MjName;
			for (const FString& Prefix : StripPrefixes)
			{
				if (!Prefix.IsEmpty() && Stripped.StartsWith(Prefix))
				{
					Stripped = Stripped.RightChop(Prefix.Len());
					break;
				}
			}
			Bone = FName(*Stripped);
		}

		if (Bone == NAME_None || Poseable->GetBoneIndex(Bone) == INDEX_NONE)
		{
			Unmatched.Add(MjName);
			continue;
		}

		FLinkPair Pair;
		Pair.Body = Body;
		Pair.Bone = Bone;
		Pairs.Add(Pair);

		// Hide only this driven link's own geom (non-recursive, so child links / an unmapped gripper
		// keep their static-mesh visuals).
		if (bHideDrivenLinkMeshes)
		{
			TArray<USceneComponent*> Children;
			Body->GetChildrenComponents(false, Children);
			for (USceneComponent* Child : Children)
			{
				if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Child))
				{
					SMC->SetHiddenInGame(true);
					SMC->SetVisibility(false);
				}
			}
		}
	}

	// Set bone world transforms parent-before-child. Ref-skeleton bone indices are stored root-first,
	// so ascending bone index gives a safe evaluation order for world-space writes.
	Pairs.Sort([this](const FLinkPair& A, const FLinkPair& B)
	{
		return Poseable->GetBoneIndex(A.Bone) < Poseable->GetBoneIndex(B.Bone);
	});

	bMappingBuilt = true;

	if (!bLoggedMissing)
	{
		bLoggedMissing = true;
		UE_LOG(LogTemp, Log, TEXT("[MjPoseDriver] mapped %d link(s) to bones; %d body(ies) unmatched%s"),
			Pairs.Num(), Unmatched.Num(),
			Unmatched.Num() > 0 ? *FString::Printf(TEXT(": %s"), *FString::Join(Unmatched, TEXT(", "))) : TEXT(""));
	}
}

void URammsMjSkeletalPoseDriver::CaptureOffsets()
{
	if (!Poseable)
		return;

	for (FLinkPair& Pair : Pairs)
	{
		UMjBody* Body = Pair.Body.Get();
		if (!Body)
			continue;

		const FTransform BodyWorld = Body->GetComponentTransform();
		const FTransform BoneWorld = Poseable->GetSocketTransform(Pair.Bone, RTS_World);
		// Constant rigid offset: BoneWorld == Offset * BodyWorld.
		Pair.Offset = BoneWorld.GetRelativeTransform(BodyWorld);
	}
	bOffsetsCaptured = true;
	bPendingCapture = false;
}

void URammsMjSkeletalPoseDriver::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!Resolve())
		return;

	if (!bMappingBuilt)
	{
		BuildMapping();
		if (!bMappingBuilt)
			return;  // still waiting on the articulation to compile
	}

	if (bPendingCapture || (bCaptureOffsetsOnFirstTick && !bOffsetsCaptured))
	{
		CaptureOffsets();
	}

	for (const FLinkPair& Pair : Pairs)
	{
		UMjBody* Body = Pair.Body.Get();
		if (!Body)
			continue;

		const FTransform BoneTarget = Pair.Offset * Body->GetComponentTransform();
		Poseable->SetBoneTransformByName(Pair.Bone, BoneTarget, EBoneSpaces::WorldSpace);
	}
}
