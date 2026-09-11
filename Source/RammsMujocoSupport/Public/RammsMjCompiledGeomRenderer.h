// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsMjCompiledGeomRenderer.generated.h"

class UMaterialInterface;
class UMaterialInstanceDynamic;
class UMjPhysicsEngine;
class UProceduralMeshComponent;
class USceneComponent;
class UStaticMesh;

/**
 * Renders the geoms that exist only in the compiled MuJoCo model — content spliced in by
 * <attach>/<model> composition or expanded from macros (<replicate>, <composite>) — which URLab
 * draws through per-geom components it does not have for spliced content (its docs call this out:
 * "the bodies, geoms and joints ... exist in the compiled model and not in the component tree").
 *
 * On each (re)compile this walks the compiled model, skips every geom an articulation component
 * already draws, and builds lightweight visuals for the rest: engine basic shapes for primitives
 * and a procedural mesh straight from the model's baked mesh_vert/mesh_face arrays for mesh geoms
 * (no file access — the compiler already embedded the geometry). Each frame the visuals follow the
 * physics thread's render snapshot (GeomXPos/GeomXMat), the same coherent state the component path
 * consumes.
 *
 * Add one to the AAMjManager actor (or any level actor). Purely presentational: no collision, no
 * spec contribution, and a model recompile rebuilds everything from scratch.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSMUJOCOSUPPORT_API URammsMjCompiledGeomRenderer : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsMjCompiledGeomRenderer();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** MuJoCo geom groups to draw, mirroring the groups MuJoCo's own viewer shows by default.
	 *  Menagerie convention puts visual geoms in group 2 and collision geoms in group 3. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjCompiledGeoms")
	TArray<int32> VisibleGeomGroups = {0, 1, 2};

	/** Base material for every generated visual. Its "Color" vector parameter (if present) is set
	 *  to the geom's rgba. Defaults to the engine's BasicShapeMaterial. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjCompiledGeoms")
	TObjectPtr<UMaterialInterface> BaseMaterial;

	/** Also draw geoms whose bodies ARE component-bound (debugging aid: shows everything). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MjCompiledGeoms")
	bool bIncludeComponentBoundGeoms = false;

	/** Drop all generated visuals and rebuild from the current compiled model on the next tick. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "MjCompiledGeoms")
	void ForceRebuild();

private:
	void Rebuild(UMjPhysicsEngine& Engine);
	void ClearVisuals();
	USceneComponent* MakeFrame(int32 GeomId);
	void AddPrimitiveVisual(USceneComponent& Frame, int32 GeomType, const double* SizeMeters, const FLinearColor& Color);
	void AddMeshVisual(USceneComponent& Frame, int32 MeshId, const FLinearColor& Color);
	UMaterialInstanceDynamic* MaterialFor(const FLinearColor& Color);
	UStaticMesh* BasicShape(const TCHAR* Path);

	/** The model the current visuals were built from. A recompile is detected when this pointer
	 *  changes OR the fingerprint below does — the fingerprint guards the case where a recompile
	 *  frees the old model and the allocator hands back the same address (pointer alone would then
	 *  miss the change and keep stale visuals). */
	const void* BuiltForModel = nullptr;

	/** Cheap compiled-model fingerprint (ngeom, nmesh, nq) captured at the last rebuild. */
	int32 BuiltNGeom = -1;
	int32 BuiltNMesh = -1;
	int32 BuiltNQ = -1;

	/** One entry per drawn geom: its MuJoCo geom id and the frame component that follows it. */
	struct FGeomVisual
	{
		int32 GeomId = INDEX_NONE;
		TWeakObjectPtr<USceneComponent> Frame;
	};
	TArray<FGeomVisual> Visuals;

	/** Every component this renderer created (frames + shape/mesh children), for teardown. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<USceneComponent>> OwnedComponents;

	/** One dynamic material instance per distinct rgba, shared across geoms. */
	UPROPERTY(Transient)
	TMap<FLinearColor, TObjectPtr<UMaterialInstanceDynamic>> MaterialCache;
};
