// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsMjSkeletalPoseDriver.generated.h"

class AMjArticulation;
class UMjBody;
class UPoseableMeshComponent;

/**
 * Drives a UPoseableMeshComponent's bones from a URLab AMjArticulation's per-link transforms, so a
 * rigged skeletal-mesh arm renders the MuJoCo simulation instead of URLab's per-body static meshes.
 *
 * A robot link is rigid, so the transform between a MuJoCo body frame and its skeletal bone frame is a
 * constant (pose-independent) offset. We capture that offset once, then each frame set
 *   BoneWorld = Offset * BodyWorld
 * where BodyWorld is the link's UE-space world transform (URLab already converts MuJoCo -> UE).
 *
 * Add this to the arm's visual actor (which has the UPoseableMeshComponent) or to the AMjArticulation
 * actor. Bodies are matched to bones by name (after stripping configured prefixes), with explicit
 * overrides taking priority. Only the driven links' static-mesh visuals are hidden, so an unmapped
 * stock gripper keeps its URLab meshes.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSMUJOCOSUPPORT_API URammsMjSkeletalPoseDriver : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsMjSkeletalPoseDriver();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** MuJoCo articulation to read link transforms from. If null, the first one on this actor is used. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjPoseDriver")
	TObjectPtr<AMjArticulation> Articulation;

	/** Poseable mesh to drive. If None, the first UPoseableMeshComponent on this actor is used. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjPoseDriver")
	FName PoseableMeshComponentName = NAME_None;

	/** Auto-map each MuJoCo body to the skeletal bone of the same name (after stripping prefixes). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjPoseDriver")
	bool bAutoMatchByName = true;

	/** Prefixes stripped from MuJoCo body names before matching a bone (e.g. "2f85_", owner prefix). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjPoseDriver")
	TArray<FString> StripPrefixes;

	/** Explicit MuJoCo-body -> bone-name overrides (take priority over auto-match). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjPoseDriver")
	TMap<FName, FName> BodyToBoneOverrides;

	/** Hide the driven links' static-mesh visuals so only the skeletal mesh shows. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjPoseDriver")
	bool bHideDrivenLinkMeshes = true;

	/** Capture the constant per-bone offsets on the first driven tick. Assumes the skeletal mesh ref
	 *  pose and the MuJoCo model are in the same joint configuration at that moment. If false, offsets
	 *  are identity (bone world = link world), which is correct when bone frames match the MJCF links. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjPoseDriver")
	bool bCaptureOffsetsOnFirstTick = true;

	/** Re-capture per-bone offsets on the next tick (call after aligning both to a matching pose). */
	UFUNCTION(BlueprintCallable, Category = "MjPoseDriver")
	void RequestCaptureOffsets() { bPendingCapture = true; }

private:
	bool Resolve();
	void BuildMapping();
	void CaptureOffsets();

	UPROPERTY(Transient)
	TObjectPtr<UPoseableMeshComponent> Poseable = nullptr;

	// One entry per driven link. Runtime cache (weak body ptr + resolved bone + constant offset).
	struct FLinkPair
	{
		TWeakObjectPtr<UMjBody> Body;
		FName Bone = NAME_None;
		FTransform Offset = FTransform::Identity;
	};
	TArray<FLinkPair> Pairs;

	bool bMappingBuilt = false;
	bool bOffsetsCaptured = false;
	bool bPendingCapture = false;
	bool bLoggedMissing = false;
};
