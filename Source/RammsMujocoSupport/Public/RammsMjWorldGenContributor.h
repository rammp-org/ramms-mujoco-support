// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MuJoCo/Spec/MjSceneContributor.h"
#include "RammsWorldGenTypes.h"
#include "RammsMjWorldGenContributor.generated.h"

class ARammsWorldGenerator;
class ULineBatchComponent;
class UMjModel;
class UMjNodeComponent;
struct FRammsCollisionShape;

/**
 * A generated world in MuJoCo, exactly: every collision shape the world reports (ARammsWorldGenerator::
 * GatherCollisionShapes: boxes, convex hulls and height grids for the procedural pieces, the simple collision of every
 * instanced mesh) becomes a static geom on MuJoCo's world body. Boxes stay boxes, so a curb face stays a vertical
 * step, where a sampled heightfield would turn it into a slope one cell wide. Hulls become mesh geoms over shared,
 * content-hashed mesh assets; height grids become hfield geoms; spheres and capsules stay primitives.
 *
 * The spec is contributed at identity: the shapes are in world space already. Static geoms never collide with each
 * other (MuJoCo drops pairs welded to the same body), so their only cost is the broadphase.
 *
 * Contacts: each geom gets its surface's sliding friction (SurfaceFriction), priority 0 and MuJoCo's default solref and
 * solimp, so robot geoms keep their own contact tuning: a higher-priority robot geom (omni rollers) uses its own
 * parameters outright, and an equal-priority one takes the larger friction of the pair and the mean of the solver
 * parameters, exactly as it did against the flat ground this replaces.
 *
 * URammsPhysicsEngineSubsystem places one per world generator in a MuJoCo level (ramms.mj.WorldShapes). The spec is
 * re-authored only when the world changed (a regeneration, RequestRecompile, a new export region); a recompile for any
 * other reason reuses it.
 */
UCLASS(ClassGroup = (Ramms), hidecategories = (Input, Collision, Replication, Rendering, HLOD, Cooking, DataLayers, WorldPartition))
class RAMMSMUJOCOSUPPORT_API ARammsMjWorldGenContributor : public AActor, public IMjSceneContributor
{
	GENERATED_BODY()

public:
	ARammsMjWorldGenContributor();

	/** The generator whose world is exported. None: every procedural piece in the level. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|World")
	TObjectPtr<ARammsWorldGenerator> Generator;

	/** Export only shapes within this distance in plan of ExportCentre (cm); 0 exports the whole world. The console's
	 *  ramms.mj.WorldShapesRadius (m) overrides it when set. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|World", meta = (ClampMin = "0", Units = "cm"))
	double ExportRadius = 0.0;

	/** Centre ExportRadius on the generator's suggested start (the robot's start) when it has one. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|World")
	bool bCentreOnSuggestedStart = true;

	/** The centre otherwise (world, cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|World")
	FVector ExportCentre = FVector::ZeroVector;

	/** Sliding friction per surface (RammsCollisionShapes surfaces: Asphalt, Concrete, Curb, Brick, Cobble, Grass ...).
	 *  Against a robot geom of the same priority MuJoCo uses the larger of the two, so a value under the robot's own
	 *  friction changes nothing; torsional and rolling friction stay at MuJoCo's defaults. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	TMap<FName, float> SurfaceFriction;

	/** Sliding friction of a surface the table does not list. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact", meta = (ClampMin = "0"))
	float DefaultFriction = 0.9f;

	/** Recompile the scene when the generator regenerates (OnWorldGenerated). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|World")
	bool bRecompileOnGenerate = true;

	/** What the last authoring pass exported. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|World")
	int32 NumGeoms = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|World")
	int32 NumMeshAssets = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|World")
	int32 NumHeightfields = 0;

	/** The shapes this would export now (the generator's, within the export region). */
	void GatherShapes(TArray<FRammsCollisionShape>& Out) const;

	/** Whether the world has anything to stand on: some shape to export. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|World")
	bool SuppliesGround() const;

	/** Re-author the world geometry and recompile the scene on the next tick (a mid-play compile keeps the robots'
	 *  state; a failed one keeps the old model). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|World")
	void RequestRecompile();

	/** Compiled geom ids of the exported geoms, from the last compile (empty before one). */
	const TArray<int32>& GetBoundGeomIds() const { return BoundGeomIds; }

	// --- IMjSceneContributor ---
	virtual void	 AuthorSceneSpec() override;
	virtual FSpecRef GetSceneSpec() const override;
	virtual FString	 GetScenePrefix() const override;
	virtual void	 OnSceneBound() override;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	UFUNCTION()
	void HandleWorldGenerated(const FRammsWorldManifest& Manifest);

	void	RecompileNow();
	FVector ResolveCentre() const;
	double	ResolveRadius() const;
	float	FrictionFor(FName Surface) const;
	void	RedrawShapes(int32 Mode, const FVector& ViewCentre, double RadiusCm);

	/** The spec's root; also this actor's root, so the spec is found without walking its components. */
	UPROPERTY(VisibleAnywhere, Category = "MuJoCo|World")
	TObjectPtr<UMjModel> Spec;

	/** The ramms.mj.DrawWorldShapes overlay. Not attached under the spec: re-authoring destroys what hangs off it. */
	UPROPERTY(Transient)
	TObjectPtr<ULineBatchComponent> DebugLines;

	/** Every element the last pass created, in creation order (parents first); geoms are also listed below. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UMjNodeComponent>> Elements;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UMjNodeComponent>> GeomElements;

	TArray<int32> BoundGeomIds;

	/** Re-author at the next compile. Off once authored, on again when the world or the export region changes. */
	bool	bDirty = true;
	bool	bAuthored = false;
	FVector AuthoredCentre = FVector::ZeroVector;
	double	AuthoredRadius = -1.0;
	double	AuthoredAt = 0.0;
	bool	bRecompileQueued = false;

	int32	DrawnMode = 0;
	int32	DrawnBindSerial = -1;
	int32	BindSerial = 0;
	FVector DrawnCentre = FVector::ZeroVector;
	double	DrawnRadius = 0.0;
	float	SinceDrawCheck = 0.0f;
};
