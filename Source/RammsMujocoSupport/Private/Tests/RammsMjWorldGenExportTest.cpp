// Copyright Epic Games, Inc. All Rights Reserved.

// The generated world in MuJoCo must be the world Chaos collides with. Every procedural piece kind (and pitched,
// rolled pieces, as on a graded street) is exported by an ARammsMjWorldGenContributor, the scene is compiled, and
// MuJoCo's own ray (mj_ray: boxes, mesh hulls, height fields with MuJoCo's own triangle split) is compared with
// Chaos's trace of the piece's mesh. This is what catches a conversion error (the Y mirror, a height field's rows or
// diagonals, a mesh's recentring) that the shapes alone cannot show.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR && URLAB_MJ_GEN

	#include "CollisionQueryParams.h"
	#include "Components/DynamicMeshComponent.h"
	#include "DynamicMesh/DynamicMesh3.h"
	#include "Engine/Engine.h"
	#include "Engine/HitResult.h"
	#include "Engine/World.h"
	#include "Math/RandomStream.h"
	#include "MuJoCo/Core/AMjManager.h"
	#include "MuJoCo/Core/MjPhysicsEngine.h"
	#include "MuJoCo/Utils/URLabAxisConv.h"
	#include "RammsCollisionShapes.h"
	#include "RammsMjWorldGenContributor.h"
	#include "RammsProcPiece.h"
	#include "UDynamicMesh.h"

	#include <mujoco/mujoco.h>

namespace RammsMjWorldGenExportTest
{
	/** A throwaway game world with a MuJoCo manager, torn down on scope exit. */
	struct FExportScene
	{
		UWorld*		 World = nullptr;
		AAMjManager* Manager = nullptr;
		FString		 LastError;

		bool Open()
		{
			World = UWorld::CreateWorld(EWorldType::Game, false);
			if (!World)
			{
				LastError = TEXT("CreateWorld failed");
				return false;
			}
			FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
			Context.SetCurrentWorld(World);
			FActorSpawnParameters Params;
			Manager = World->SpawnActor<AAMjManager>(Params);
			if (!Manager)
			{
				LastError = TEXT("SpawnActor AAMjManager failed");
				return false;
			}
			return true;
		}

		template <typename T>
		T* Spawn(const FTransform& Xf)
		{
			FActorSpawnParameters SP;
			SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
			return World->SpawnActor<T>(T::StaticClass(), Xf, SP);
		}

		~FExportScene()
		{
			if (Manager && Manager->PhysicsEngine)
			{
				Manager->PhysicsEngine->bShouldStopTask = true;
			}
			if (World)
			{
				World->DestroyWorld(false);
				GEngine->DestroyWorldContext(World);
			}
		}
	};

	struct FPiece
	{
		FString			 Name;
		ARammsProcPiece* Actor = nullptr;
		double			 Tolerance = 0.2; // cm
		double			 MissFraction = 0.02;
	};

	FBox MeshBounds(const ARammsProcPiece& Piece)
	{
		FBox			 B(ForceInit);
		const FTransform Xf = Piece.MeshComponent->GetComponentTransform();
		Piece.MeshComponent->GetDynamicMesh()->ProcessMesh([&](const UE::Geometry::FDynamicMesh3& Mesh) {
			for (const int32 Vid : Mesh.VertexIndicesItr())
			{
				B += Xf.TransformPosition(Mesh.GetVertex(Vid));
			}
		});
		return B;
	}
} // namespace RammsMjWorldGenExportTest

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMjWorldGenExportTest, "Ramms.MuJoCo.WorldGenExport.MatchesChaos",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsMjWorldGenExportTest::RunTest(const FString& Parameters)
{
	using namespace RammsMjWorldGenExportTest;
	FExportScene Scene;
	if (!TestTrue(TEXT("world opens"), Scene.Open()))
	{
		AddError(Scene.LastError);
		return false;
	}

	TArray<FPiece> Pieces;
	auto		   Add = [&](const TCHAR* Name, ERammsProcPieceKind Kind, const FRotator& Rotation, TFunctionRef<void(ARammsProcPiece&)> Setup) -> FPiece* {
		  ARammsProcPiece* P = Scene.Spawn<ARammsProcPiece>(FTransform(Rotation, FVector(3000.0 * Pieces.Num(), -800.0, 50.0)));
		  if (!P)
		  {
			  AddError(FString::Printf(TEXT("%s: spawn failed"), Name));
			  return nullptr;
		  }
		  P->Kind = Kind;
		  P->KindId = -1;
		  Setup(*P);
		  P->Rebuild();
		  FPiece& Entry = Pieces.AddDefaulted_GetRef();
		  Entry.Name = Name;
		  Entry.Actor = P;
		  return &Entry;
	};
	Add(TEXT("Slab"), ERammsProcPieceKind::Slab, FRotator(0.0, 20.0, 0.0), [](ARammsProcPiece& P) { P.SizeX = 300.0; P.SizeY = 200.0; P.SizeZ = 20.0; });
	Add(TEXT("Slab pitched and rolled"), ERammsProcPieceKind::Slab, FRotator(4.0, -30.0, 3.0), [](ARammsProcPiece& P) { P.SizeX = 400.0; P.SizeY = 250.0; P.SizeZ = 15.0; });
	Add(TEXT("Curb"), ERammsProcPieceKind::Curb, FRotator(0.0, 45.0, 0.0), [](ARammsProcPiece& P) { P.SizeX = 400.0; P.SizeY = 15.0; P.Rise = 15.0; });
	Add(TEXT("CurbRamp"), ERammsProcPieceKind::CurbRamp, FRotator(0.0, 30.0, 0.0), [](ARammsProcPiece& P) { P.Rise = 15.0; P.Slope = 0.0833; P.SizeY = 120.0; P.FlareSlope = 0.1; P.WarningDepth = 61.0; });
	Add(TEXT("CurbRamp pitched and rolled"), ERammsProcPieceKind::CurbRamp, FRotator(3.0, -25.0, -2.0), [](ARammsProcPiece& P) { P.Rise = 18.0; P.Slope = 0.07; P.SizeY = 150.0; P.FlareSlope = 0.08; });
	Add(TEXT("Steps"), ERammsProcPieceKind::Step, FRotator(0.0, 10.0, 0.0), [](ARammsProcPiece& P) { P.StepCount = 3; P.SizeX = 30.0; P.Rise = 17.0; P.SizeY = 120.0; });
	Add(TEXT("Ramp"), ERammsProcPieceKind::Ramp, FRotator(0.0, -60.0, 0.0), [](ARammsProcPiece& P) { P.Rise = 30.0; P.Slope = 0.0833; P.SizeY = 120.0; });
	Add(TEXT("Wall"), ERammsProcPieceKind::Wall, FRotator(0.0, 5.0, 0.0), [](ARammsProcPiece& P) { P.SizeX = 500.0; P.SizeY = 20.0; P.SizeZ = 300.0; P.OpeningWidth = 100.0; P.OpeningOffset = 120.0; P.Openings = TEXT("340,120,90,210"); });
	Add(TEXT("Bollard"), ERammsProcPieceKind::Bollard, FRotator::ZeroRotator, [](ARammsProcPiece& P) { P.SizeX = 20.0; P.SizeZ = 90.0; });
	Add(TEXT("Roof"), ERammsProcPieceKind::Roof, FRotator(0.0, 40.0, 0.0), [](ARammsProcPiece& P) { P.SizeX = 600.0; P.SizeY = 800.0; P.SizeZ = 150.0; });
	Add(TEXT("Mound"), ERammsProcPieceKind::Mound, FRotator(0.0, 15.0, 0.0), [](ARammsProcPiece& P) { P.SizeX = 400.0; P.SizeY = 300.0; P.Rise = 40.0; });
	Add(TEXT("Hollow pitched"), ERammsProcPieceKind::Mound, FRotator(2.0, 0.0, -1.5), [](ARammsProcPiece& P) { P.SizeX = 300.0; P.SizeY = 300.0; P.Rise = -25.0; });
	Add(TEXT("Patch"), ERammsProcPieceKind::Patch, FRotator::ZeroRotator, [](ARammsProcPiece& P) { P.SizeX = 200.0; P.SizeY = 120.0; P.SizeZ = 1.6; });
	if (FPiece* Tree = Add(TEXT("Tree"), ERammsProcPieceKind::Tree, FRotator::ZeroRotator, [](ARammsProcPiece& P) { P.SizeX = 300.0; P.SizeZ = 600.0; }))
	{
		Tree->Tolerance = 3.0; // canopy blobs are hulls of sphere-boxes
		Tree->MissFraction = 0.1;
	}

	// No generator: the contributor exports every piece in the level.
	ARammsMjWorldGenContributor* Contributor = Scene.Spawn<ARammsMjWorldGenContributor>(FTransform::Identity);
	if (!TestNotNull(TEXT("contributor spawns"), Contributor))
	{
		return false;
	}
	TArray<FRammsCollisionShape> Shapes;
	Contributor->GatherShapes(Shapes);

	if (!TestTrue(TEXT("the scene compiles"), Scene.Manager->PhysicsEngine->InstallCompiledSpec(Scene.LastError)))
	{
		AddError(Scene.LastError);
		return false;
	}
	const mjModel* Model = Scene.Manager->PhysicsEngine->GetModel();
	if (!TestNotNull(TEXT("model exists"), Model))
	{
		return false;
	}
	TestEqual(TEXT("every shape is a geom"), Contributor->NumGeoms, Shapes.Num());
	TestEqual(TEXT("every geom bound"), Contributor->GetBoundGeomIds().Num(), Contributor->NumGeoms);
	TestTrue(TEXT("height fields compiled (flares, mounds)"), Model->nhfield >= 4);
	TestTrue(TEXT("hulls compiled as meshes"), Model->nmesh >= 3);

	// A scene of its own for the rays, so nothing races the session's step.
	mjData* Data = mj_makeData(Model);
	if (!TestNotNull(TEXT("mjData"), Data))
	{
		return false;
	}
	mj_kinematics(Model, Data);

	FRandomStream Rs(1114);
	for (const FPiece& Piece : Pieces)
	{
		UDynamicMeshComponent* Component = Piece.Actor->MeshComponent;
		const FBox			   Region = MeshBounds(*Piece.Actor);
		constexpr int32		   Probes = 250;
		int32				   Misses = 0, Compared = 0;
		double				   Worst = 0.0;
		FVector				   WorstAt = FVector::ZeroVector;
		for (int32 p = 0; p < Probes; ++p)
		{
			const double  X = FMath::Lerp(Region.Min.X, Region.Max.X, Rs.FRand());
			const double  Y = FMath::Lerp(Region.Min.Y, Region.Max.Y, Rs.FRand());
			const FVector From(X, Y, Region.Max.Z + 50.0), To(X, Y, Region.Min.Z - 50.0);

			FHitResult			  Hit;
			FCollisionQueryParams Params(SCENE_QUERY_STAT(RammsMjExportProbe), /*bTraceComplex=*/true);
			const bool			  bChaos = Component->LineTraceComponent(Hit, From, To, Params);

			mjtNum Pnt[3];
			URLabAxisConv::UePositionToMj(From, Pnt);
			const mjtNum Down[3] = { 0.0, 0.0, -1.0 };
			int			 GeomId[1] = { -1 };
			mjtNum		 Normal[3] = { 0.0, 0.0, 0.0 };
			const mjtNum Distance = mj_ray(Model, Data, Pnt, Down, nullptr, 1, -1, GeomId, Normal);
			const bool	 bMuJoCo = Distance >= 0.0;

			if (bChaos != bMuJoCo)
			{
				++Misses;
				continue;
			}
			if (!bChaos)
			{
				continue;
			}
			++Compared;
			const double MuJoCoZ = (Pnt[2] - Distance) * 100.0; // MuJoCo's Z is Unreal's, in metres
			const double Error = FMath::Abs(MuJoCoZ - Hit.ImpactPoint.Z);
			if (Error > Worst)
			{
				Worst = Error;
				WorstAt = Hit.ImpactPoint;
			}
		}
		TestTrue(FString::Printf(TEXT("%s: probes find a top (%d of %d)"), *Piece.Name, Compared, Probes), Compared > Probes / 10);
		TestTrue(FString::Printf(TEXT("%s: %d probes hit in one engine only (allowed %d)"), *Piece.Name, Misses, FMath::CeilToInt(Probes * Piece.MissFraction)),
			Misses <= FMath::CeilToInt(Probes * Piece.MissFraction));
		TestTrue(FString::Printf(TEXT("%s: MuJoCo's top within %.1f mm of Chaos's (worst %.2f mm at %s)"), *Piece.Name, Piece.Tolerance * 10.0, Worst * 10.0, *WorstAt.ToString()),
			Worst <= Piece.Tolerance);
	}
	mj_deleteData(Data);

	// A recompile with nothing changed reuses the authored spec and gives the same geoms.
	const int32 Before = Contributor->NumGeoms;
	if (TestTrue(TEXT("the scene recompiles"), Scene.Manager->PhysicsEngine->InstallCompiledSpec(Scene.LastError)))
	{
		TestEqual(TEXT("the same geoms after a recompile"), Contributor->GetBoundGeomIds().Num(), Before);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR && URLAB_MJ_GEN
