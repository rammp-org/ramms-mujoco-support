// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMjCompiledGeomRenderer.h"

#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"
#include "EngineUtils.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "ProceduralMeshComponent.h"
#include "UObject/ConstructorHelpers.h"

#include <mujoco/mujoco.h>

namespace
{
	// Engine basic shapes are 100 cm across (spheres/cylinders: 50 cm radius; cube: 50 cm half
	// side; cylinder: 50 cm half height), so a MuJoCo size in metres scales by size*100/50.
	constexpr double kMetersToBasicShapeScale = 2.0;

	const TCHAR* const kSphereMeshPath = TEXT("/Engine/BasicShapes/Sphere.Sphere");
	const TCHAR* const kCubeMeshPath = TEXT("/Engine/BasicShapes/Cube.Cube");
	const TCHAR* const kCylinderMeshPath = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
	const TCHAR* const kBasicMaterialPath = TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial");
}

URammsMjCompiledGeomRenderer::URammsMjCompiledGeomRenderer()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	// After URLab has pushed the physics snapshot onto its own body components this frame, so the
	// generated visuals and the component-drawn ones show the same physics frame.
	PrimaryComponentTick.TickGroup = TG_PostUpdateWork;
}

void URammsMjCompiledGeomRenderer::ForceRebuild()
{
	BuiltForModel = nullptr;
}

void URammsMjCompiledGeomRenderer::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	ClearVisuals();
	Super::EndPlay(EndPlayReason);
}

void URammsMjCompiledGeomRenderer::ClearVisuals()
{
	for (USceneComponent* Component : OwnedComponents)
	{
		if (Component != nullptr)
		{
			Component->DestroyComponent();
		}
	}
	OwnedComponents.Reset();
	Visuals.Reset();
	MaterialCache.Reset();
	BuiltForModel = nullptr;
	BuiltNGeom = BuiltNMesh = BuiltNQ = -1;
}

void URammsMjCompiledGeomRenderer::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	if (Engine == nullptr)
	{
		return;
	}
	const mjModel* Model = Engine->GetModel();
	if (Model == nullptr)
	{
		// Not compiled yet — retry next tick (the first compile lands during BeginPlay ordering).
		if (BuiltForModel != nullptr)
		{
			ClearVisuals();
		}
		return;
	}
	// Rebuild on a new model pointer, or on a fingerprint change at the same address (an in-place
	// recompile can reuse the freed model's address, which the pointer alone would miss).
	if (Model != BuiltForModel || Model->ngeom != BuiltNGeom || Model->nmesh != BuiltNMesh || Model->nq != BuiltNQ)
	{
		Rebuild(*Engine);
	}
	if (Visuals.Num() == 0)
	{
		return;
	}

	// Follow the physics thread's coherent snapshot, exactly as the component-drawn geoms do.
	Engine->WithRenderState([this, Model](const FMjRenderSnapshot& Snap) {
		if (Snap.GeomXPos.Num() < 3 * Model->ngeom || Snap.GeomXMat.Num() < 9 * Model->ngeom)
		{
			return; // no snapshot published yet for this model
		}
		for (const FGeomVisual& Visual : Visuals)
		{
			USceneComponent* Frame = Visual.Frame.Get();
			if (Frame == nullptr || Visual.GeomId < 0 || Visual.GeomId >= Model->ngeom)
			{
				continue;
			}
			const mjtNum* Pos = Snap.GeomXPos.GetData() + 3 * Visual.GeomId;
			const mjtNum* Mat = Snap.GeomXMat.GetData() + 9 * Visual.GeomId;
			double Quat[4];
			mju_mat2Quat(Quat, Mat);
			const double PosD[3] = {Pos[0], Pos[1], Pos[2]};
			Frame->SetWorldLocationAndRotation(URLabAxisConv::MjPositionToUe(PosD), URLabAxisConv::MjQuatToUe(Quat));
		}
	});
}

void URammsMjCompiledGeomRenderer::Rebuild(UMjPhysicsEngine& Engine)
{
	ClearVisuals();
	const mjModel* Model = Engine.GetModel();
	if (Model == nullptr)
	{
		return;
	}
	BuiltForModel = Model;
	BuiltNGeom = Model->ngeom;
	BuiltNMesh = Model->nmesh;
	BuiltNQ = Model->nq;

	// Component-bound geoms are already drawn by URLab; collect the articulations once and ask
	// each per geom id. (Ids are scene-global after composition, so any articulation may own one.)
	TArray<AMjArticulation*> Articulations;
	if (UWorld* World = GetWorld())
	{
		for (TActorIterator<AMjArticulation> It(World); It; ++It)
		{
			Articulations.Add(*It);
		}
	}

	int32 Drawn = 0;
	for (int32 GeomId = 0; GeomId < Model->ngeom; ++GeomId)
	{
		if (!bIncludeComponentBoundGeoms)
		{
			bool bBound = false;
			for (AMjArticulation* Articulation : Articulations)
			{
				if (Articulation != nullptr && Articulation->GetComponentByMjId(mjOBJ_GEOM, GeomId) != nullptr)
				{
					bBound = true;
					break;
				}
			}
			if (bBound)
			{
				continue;
			}
		}

		if (!VisibleGeomGroups.Contains(Model->geom_group[GeomId]))
		{
			continue;
		}

		const int32 GeomType = Model->geom_type[GeomId];
		if (GeomType == mjGEOM_PLANE || GeomType == mjGEOM_HFIELD || GeomType == mjGEOM_SDF)
		{
			continue; // no honest preview (URLab's own component previews skip these too)
		}

		// Color: the geom's own rgba unless it defers to a material's.
		FLinearColor Color(Model->geom_rgba[4 * GeomId], Model->geom_rgba[4 * GeomId + 1],
			Model->geom_rgba[4 * GeomId + 2], Model->geom_rgba[4 * GeomId + 3]);
		const int32 MatId = Model->geom_matid[GeomId];
		if (MatId >= 0 && MatId < Model->nmat && Color == FLinearColor(0.5f, 0.5f, 0.5f, 1.0f))
		{
			Color = FLinearColor(Model->mat_rgba[4 * MatId], Model->mat_rgba[4 * MatId + 1],
				Model->mat_rgba[4 * MatId + 2], Model->mat_rgba[4 * MatId + 3]);
		}
		if (Color.A <= 0.0f)
		{
			continue; // authored invisible
		}

		USceneComponent* Frame = MakeFrame(GeomId);
		if (Frame == nullptr)
		{
			continue;
		}
		if (GeomType == mjGEOM_MESH)
		{
			AddMeshVisual(*Frame, Model->geom_dataid[GeomId], Color);
		}
		else
		{
			const double Size[3] = {Model->geom_size[3 * GeomId], Model->geom_size[3 * GeomId + 1],
				Model->geom_size[3 * GeomId + 2]};
			AddPrimitiveVisual(*Frame, GeomType, Size, Color);
		}
		++Drawn;
	}

	UE_LOG(LogTemp, Log, TEXT("[MjCompiledGeomRenderer] drawing %d compiled-only geom(s) of %d total"), Drawn,
		Model->ngeom);
}

USceneComponent* URammsMjCompiledGeomRenderer::MakeFrame(int32 GeomId)
{
	AActor* Owner = GetOwner();
	if (Owner == nullptr)
	{
		return nullptr;
	}
	// Auto-generated name (not "MjCompiledGeom_<id>"): a rebuild destroys the old frames but they
	// are not GC'd immediately, so reusing a deterministic name would collide with the pending-kill
	// object and force a rename (warning, and an assert on some paths).
	USceneComponent* Frame = NewObject<USceneComponent>(Owner);
	// World transforms come straight from the physics snapshot; the owner's own motion is noise.
	Frame->SetUsingAbsoluteLocation(true);
	Frame->SetUsingAbsoluteRotation(true);
	Frame->SetUsingAbsoluteScale(true);
	Frame->SetupAttachment(Owner->GetRootComponent());
	Frame->RegisterComponent();
	OwnedComponents.Add(Frame);

	FGeomVisual Visual;
	Visual.GeomId = GeomId;
	Visual.Frame = Frame;
	Visuals.Add(Visual);
	return Frame;
}

void URammsMjCompiledGeomRenderer::AddPrimitiveVisual(USceneComponent& Frame, int32 GeomType, const double* SizeMeters, const FLinearColor& Color)
{
	AActor* Owner = GetOwner();
	UMaterialInstanceDynamic* Material = MaterialFor(Color);

	const auto AddShape = [this, Owner, &Frame, Material](const TCHAR* MeshPath, const FVector& Scale, const FVector& RelLocationCm) -> void {
		UStaticMesh* Mesh = BasicShape(MeshPath);
		if (Mesh == nullptr)
		{
			return;
		}
		UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(Owner);
		Component->SetStaticMesh(Mesh);
		Component->SetRelativeScale3D(Scale);
		Component->SetRelativeLocation(RelLocationCm);
		Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		if (Material != nullptr)
		{
			Component->SetMaterial(0, Material);
		}
		Component->SetupAttachment(&Frame);
		Component->RegisterComponent();
		OwnedComponents.Add(Component);
	};

	const double S0 = SizeMeters[0] * kMetersToBasicShapeScale;
	const double S1 = SizeMeters[1] * kMetersToBasicShapeScale;
	const double S2 = SizeMeters[2] * kMetersToBasicShapeScale;
	switch (GeomType)
	{
		case mjGEOM_SPHERE:
			AddShape(kSphereMeshPath, FVector(S0), FVector::ZeroVector);
			break;
		case mjGEOM_ELLIPSOID:
			AddShape(kSphereMeshPath, FVector(S0, S1, S2), FVector::ZeroVector);
			break;
		case mjGEOM_BOX:
			AddShape(kCubeMeshPath, FVector(S0, S1, S2), FVector::ZeroVector);
			break;
		case mjGEOM_CYLINDER:
			// MuJoCo cylinder axis is local Z, same as the engine mesh. size = [radius, halflen].
			AddShape(kCylinderMeshPath, FVector(S0, S0, S1), FVector::ZeroVector);
			break;
		case mjGEOM_CAPSULE:
		{
			// Cylinder body plus two sphere caps, the same decomposition URLab's previews use.
			const double HalfLenCm = SizeMeters[1] * 100.0;
			AddShape(kCylinderMeshPath, FVector(S0, S0, S1), FVector::ZeroVector);
			AddShape(kSphereMeshPath, FVector(S0), FVector(0, 0, HalfLenCm));
			AddShape(kSphereMeshPath, FVector(S0), FVector(0, 0, -HalfLenCm));
			break;
		}
		default:
			break;
	}
}

void URammsMjCompiledGeomRenderer::AddMeshVisual(USceneComponent& Frame, int32 MeshId, const FLinearColor& Color)
{
	const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	const mjModel* Model = Engine != nullptr ? Engine->GetModel() : nullptr;
	AActor* Owner = GetOwner();
	if (Model == nullptr || Owner == nullptr || MeshId < 0 || MeshId >= Model->nmesh)
	{
		return;
	}

	const int32 FaceStart = Model->mesh_faceadr[MeshId];
	const int32 FaceCount = Model->mesh_facenum[MeshId];
	const int32 VertStart = Model->mesh_vertadr[MeshId];
	const int32 NormalStart = Model->mesh_normaladr[MeshId];

	// Split-vertex triangle soup: MuJoCo indexes positions and normals independently per face
	// corner, which a straight shared-vertex build cannot represent. Robot visual meshes are small
	// enough that the duplication is irrelevant.
	TArray<FVector> Vertices;
	TArray<FVector> Normals;
	TArray<int32> Triangles;
	Vertices.Reserve(FaceCount * 3);
	Normals.Reserve(FaceCount * 3);
	Triangles.Reserve(FaceCount * 3);

	for (int32 Face = 0; Face < FaceCount; ++Face)
	{
		const int32* FaceVerts = Model->mesh_face + 3 * (FaceStart + Face);
		const int32* FaceNormals = Model->mesh_facenormal + 3 * (FaceStart + Face);
		// The Y-mirror into UE's left-handed frame flips winding; emitting corners in reverse
		// order keeps the faces outward.
		for (int32 Corner = 2; Corner >= 0; --Corner)
		{
			const float* Vert = Model->mesh_vert + 3 * (VertStart + FaceVerts[Corner]);
			const float* Normal = Model->mesh_normal + 3 * (NormalStart + FaceNormals[Corner]);
			const double NormalD[3] = {Normal[0], Normal[1], Normal[2]};
			Triangles.Add(Vertices.Num());
			Vertices.Add(URLabAxisConv::MjPositionToUe(Vert));
			Normals.Add(URLabAxisConv::MjDirectionToUe(NormalD));
		}
	}

	UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(Owner);
	Mesh->CreateMeshSection(0, Vertices, Triangles, Normals, TArray<FVector2D>(), TArray<FColor>(),
		TArray<FProcMeshTangent>(), /*bCreateCollision=*/false);
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	if (UMaterialInstanceDynamic* Material = MaterialFor(Color))
	{
		Mesh->SetMaterial(0, Material);
	}
	Mesh->SetupAttachment(&Frame);
	Mesh->RegisterComponent();
	OwnedComponents.Add(Mesh);
}

UMaterialInstanceDynamic* URammsMjCompiledGeomRenderer::MaterialFor(const FLinearColor& Color)
{
	if (TObjectPtr<UMaterialInstanceDynamic>* Cached = MaterialCache.Find(Color))
	{
		return *Cached;
	}
	UMaterialInterface* Base = BaseMaterial;
	if (Base == nullptr)
	{
		Base = LoadObject<UMaterialInterface>(nullptr, kBasicMaterialPath);
	}
	if (Base == nullptr)
	{
		return nullptr;
	}
	UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(Base, this);
	Material->SetVectorParameterValue(TEXT("Color"), Color);
	MaterialCache.Add(Color, Material);
	return Material;
}

UStaticMesh* URammsMjCompiledGeomRenderer::BasicShape(const TCHAR* Path)
{
	return LoadObject<UStaticMesh>(nullptr, Path);
}
