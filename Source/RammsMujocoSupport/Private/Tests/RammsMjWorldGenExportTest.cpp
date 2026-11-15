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
	#include "EngineUtils.h"
	#include "HAL/IConsoleManager.h"
	#include "Math/RandomStream.h"
	#include "Misc/ScopeExit.h"
	#include "MuJoCo/Convert/MjQuickConvertComponent.h"
	#include "MuJoCo/Core/AMjManager.h"
	#include "MuJoCo/Core/MjPhysicsEngine.h"
	#include "MuJoCo/Utils/URLabAxisConv.h"
	#include "RammsCollisionShapes.h"
	#include "RammsMjWorldGenContributor.h"
	#include "RammsPhysicsEngine.h"
	#include "RammsProcPiece.h"
	#include "RammsWorldGenerator.h"
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

	/** The mesh's bounds in the piece's own frame: probes are drawn there, so a thin piece turned 45 degrees (a curb) is
	 *  not probed mostly beside it. */
	FBox LocalMeshBounds(const ARammsProcPiece& Piece)
	{
		FBox B(ForceInit);
		Piece.MeshComponent->GetDynamicMesh()->ProcessMesh([&](const UE::Geometry::FDynamicMesh3& Mesh) {
			for (const int32 Vid : Mesh.VertexIndicesItr())
			{
				B += Mesh.GetVertex(Vid);
			}
		});
		return B;
	}

	/** One GroundPrisms prism: its surface and its top polygon (counter-clockwise from above). The same helpers as
	 *  RammsWorldGen's collision shapes test. */
	struct FPrismSpec
	{
		const TCHAR*	Surface;
		TArray<FVector> Top;
	};

	/** GroundPrisms Parts text, "<material slot>,<surface>:x,y,z x,y,z ...;..." (RammsGround::ParseParts). */
	FString PrismParts(const TArray<FPrismSpec>& Prisms)
	{
		FString Text;
		for (const FPrismSpec& P : Prisms)
		{
			if (!Text.IsEmpty())
			{
				Text += TEXT(";");
			}
			Text += FString::Printf(TEXT("0,%s:"), P.Surface);
			for (int32 k = 0; k < P.Top.Num(); ++k)
			{
				Text += FString::Printf(TEXT("%s%.2f,%.2f,%.2f"), k > 0 ? TEXT(" ") : TEXT(""), P.Top[k].X, P.Top[k].Y, P.Top[k].Z);
			}
		}
		return Text;
	}

	/** A counter-clockwise (from above) level rectangle at height Z. */
	TArray<FVector> Rect(double X0, double Y0, double X1, double Y1, double Z)
	{
		return { FVector(X0, Y0, Z), FVector(X1, Y0, Z), FVector(X1, Y1, Z), FVector(X0, Y1, Z) };
	}

	/** A road 400 x 300 with a pothole: four level boxes around a 100 x 100 hole whose floor is 6 cm down. */
	FString PotholeParts()
	{
		return PrismParts({
			{ TEXT("Asphalt"), Rect(0.0, 0.0, 400.0, 100.0, 0.0) },
			{ TEXT("Asphalt"), Rect(0.0, 200.0, 400.0, 300.0, 0.0) },
			{ TEXT("Asphalt"), Rect(0.0, 100.0, 150.0, 200.0, 0.0) },
			{ TEXT("Asphalt"), Rect(250.0, 100.0, 400.0, 200.0, 0.0) },
			{ TEXT("Asphalt"), Rect(150.0, 100.0, 250.0, 200.0, -6.0) },
		});
	}

	/** A sloped quad and a level pentagon: planar convex tops, one hull each. */
	FString SlopedParts()
	{
		return PrismParts({
			{ TEXT("Asphalt"), { FVector(0.0, 0.0, 0.0), FVector(300.0, 0.0, -12.0), FVector(300.0, 200.0, -12.0), FVector(0.0, 200.0, 0.0) } },
			{ TEXT("Asphalt"), { FVector(300.0, 0.0, -12.0), FVector(450.0, 0.0, -12.0), FVector(500.0, 100.0, -12.0), FVector(450.0, 200.0, -12.0), FVector(300.0, 200.0, -12.0) } },
		});
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
	auto		   Add = [&](const TCHAR* Name, ERammsProcPieceKind Kind, const FRotator& Rotation, TFunctionRef<void(ARammsProcPiece&)> Setup,
					 const FVector& Scale = FVector::OneVector) -> FPiece* {
		  ARammsProcPiece* P = Scene.Spawn<ARammsProcPiece>(FTransform(Rotation, FVector(3000.0 * Pieces.Num(), -800.0, 50.0), Scale));
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
	// Canopy blobs are folded outward when built, so each blob's mesh is its hull: exact like the rest.
	Add(TEXT("Tree"), ERammsProcPieceKind::Tree, FRotator::ZeroRotator, [](ARammsProcPiece& P) { P.SizeX = 300.0; P.SizeZ = 600.0; });
	// The ground kinds of graded streets: boxes, hulls, prism fans and two-row height fields.
	Add(TEXT("GroundPrisms level rectangle"), ERammsProcPieceKind::GroundPrisms, FRotator(0.0, 25.0, 0.0), [](ARammsProcPiece& P) {
		P.SizeZ = 20.0;
		P.Parts = PrismParts({ { TEXT("Concrete"), Rect(0.0, 0.0, 200.0, 150.0, 0.0) } });
	});
	Add(TEXT("GroundPrisms sloped, pitched and rolled"), ERammsProcPieceKind::GroundPrisms, FRotator(3.0, -20.0, 2.0), [](ARammsProcPiece& P) {
		P.SizeZ = 15.0;
		P.Parts = SlopedParts();
	});
	Add(TEXT("GroundPrisms pothole, pitched"), ERammsProcPieceKind::GroundPrisms, FRotator(2.0, 15.0, -1.5), [](ARammsProcPiece& P) {
		P.SizeZ = 20.0;
		P.Parts = PotholeParts();
	});
	Add(TEXT("GroundPrisms twisted"), ERammsProcPieceKind::GroundPrisms, FRotator::ZeroRotator, [](ARammsProcPiece& P) {
		P.SizeZ = 15.0;
		P.Parts = PrismParts({ { TEXT("Concrete"), { FVector(0.0, 0.0, 0.0), FVector(200.0, 0.0, 0.0), FVector(200.0, 200.0, 10.0), FVector(0.0, 200.0, 0.0) } } });
	});
	Add(
		TEXT("GroundPrisms pothole, scaled"), ERammsProcPieceKind::GroundPrisms, FRotator(0.0, 10.0, 0.0), [](ARammsProcPiece& P) {
			P.SizeZ = 20.0;
			P.Parts = PotholeParts();
		},
		FVector(1.5, 0.8, 1.25));
	Add(TEXT("Hump"), ERammsProcPieceKind::Hump, FRotator(0.0, 30.0, 0.0), [](ARammsProcPiece& P) {
		P.SizeX = 370.0;
		P.SizeY = 300.0;
		P.Rise = 8.0;
		P.SizeZ = 5.0;
		P.LandingSize = 0.0;
	});
	Add(TEXT("Hump table, pitched"), ERammsProcPieceKind::Hump, FRotator(2.0, -40.0, 1.0), [](ARammsProcPiece& P) {
		P.SizeX = 600.0;
		P.SizeY = 300.0;
		P.Rise = 10.0;
		P.SizeZ = 5.0;
		P.LandingSize = 300.0;
	});
	Add(
		TEXT("CurbRamp scaled"), ERammsProcPieceKind::CurbRamp, FRotator(0.0, 20.0, 0.0), [](ARammsProcPiece& P) {
			P.Rise = 15.0;
			P.Slope = 0.0833;
			P.SizeY = 120.0;
			P.FlareSlope = 0.1;
			P.WarningDepth = 61.0;
		},
		FVector(1.2, 0.9, 1.1));

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
		const FBox			   Local = LocalMeshBounds(*Piece.Actor);
		const FTransform	   PieceXf = Component->GetComponentTransform();
		constexpr int32		   Probes = 250;
		int32				   Misses = 0, Compared = 0;
		double				   Worst = 0.0;
		FVector				   WorstAt = FVector::ZeroVector;
		for (int32 p = 0; p < Probes; ++p)
		{
			// A plan point drawn in the piece's own frame, probed straight down in the world's.
			const FVector At = PieceXf.TransformPosition(
				FVector(FMath::Lerp(Local.Min.X, Local.Max.X, Rs.FRand()), FMath::Lerp(Local.Min.Y, Local.Max.Y, Rs.FRand()), Local.Max.Z));
			const FVector From(At.X, At.Y, Region.Max.Z + 50.0), To(At.X, At.Y, Region.Min.Z - 50.0);

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

namespace RammsMjWorldGenExportTest
{
	int32 ContributorsFor(UWorld& World, const ARammsWorldGenerator* Generator)
	{
		int32 N = 0;
		for (TActorIterator<ARammsMjWorldGenContributor> It(&World); It; ++It)
		{
			N += It->Generator.Get() == Generator ? 1 : 0;
		}
		return N;
	}

	int32 ShapesFrom(const TArray<FRammsCollisionShape>& Shapes, const AActor& Actor)
	{
		int32 N = 0;
		for (const FRammsCollisionShape& S : Shapes)
		{
			N += S.Source == Actor.GetFName() ? 1 : 0;
		}
		return N;
	}

	ARammsProcPiece* SpawnSlab(FExportScene& Scene, const FVector& At)
	{
		ARammsProcPiece* P = Scene.Spawn<ARammsProcPiece>(FTransform(At));
		if (P)
		{
			P->Kind = ERammsProcPieceKind::Slab;
			P->KindId = -1;
			P->SizeX = 200.0;
			P->SizeY = 200.0;
			P->SizeZ = 20.0;
			P->Rebuild();
		}
		return P;
	}

	/** An actor MuJoCo already converts (left unregistered: only its presence matters here). */
	void AddQuickConvert(AActor& Actor)
	{
		UMjQuickConvertComponent* Quick = NewObject<UMjQuickConvertComponent>(&Actor, TEXT("AlreadyInMuJoCo"));
		Actor.AddInstanceComponent(Quick);
	}
} // namespace RammsMjWorldGenExportTest

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMjWorldGenNoDoubleExportTest, "Ramms.MuJoCo.WorldGenExport.NoDoubleExport",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsMjWorldGenNoDoubleExportTest::RunTest(const FString& Parameters)
{
	using namespace RammsMjWorldGenExportTest;
	// The subsystem's rule as shipped (ramms.mj.WorldShapes 1), whatever this session set.
	IConsoleVariable* ModeVar = IConsoleManager::Get().FindConsoleVariable(TEXT("ramms.mj.WorldShapes"));
	const int32		  WasMode = ModeVar ? ModeVar->GetInt() : 1;
	if (ModeVar)
	{
		ModeVar->Set(1, ECVF_SetByConsole);
	}
	ON_SCOPE_EXIT
	{
		if (ModeVar)
		{
			ModeVar->Set(WasMode, ECVF_SetByConsole);
		}
	};

	FExportScene Scene;
	if (!TestTrue(TEXT("world opens"), Scene.Open()))
	{
		AddError(Scene.LastError);
		return false;
	}
	UWorld& World = *Scene.World;

	// One contributor per generator, and never a second for a generator that has one.
	ARammsWorldGenerator* First = Scene.Spawn<ARammsWorldGenerator>(FTransform(FVector(0.0, 0.0, 0.0)));
	if (!TestNotNull(TEXT("a generator spawns"), First))
	{
		return false;
	}
	TestEqual(TEXT("a generator gets a contributor"), URammsPhysicsEngineSubsystem::AddWorldGenContributors(World), 1);
	TestEqual(TEXT("...for that generator"), ContributorsFor(World, First), 1);
	TestEqual(TEXT("a covered generator gets none again"), URammsPhysicsEngineSubsystem::AddWorldGenContributors(World), 0);
	ARammsWorldGenerator* Second = Scene.Spawn<ARammsWorldGenerator>(FTransform(FVector(50000.0, 0.0, 0.0)));
	TestEqual(TEXT("only the new generator gets one"), URammsPhysicsEngineSubsystem::AddWorldGenContributors(World), 1);
	TestEqual(TEXT("...the second generator"), ContributorsFor(World, Second), 1);

	// A contributor with no generator exports every piece, so a third generator gets no contributor of its own.
	ARammsMjWorldGenContributor* Everything = Scene.Spawn<ARammsMjWorldGenContributor>(FTransform::Identity);
	if (!TestNotNull(TEXT("a contributor with no generator spawns"), Everything))
	{
		return false;
	}
	ARammsWorldGenerator* Third = Scene.Spawn<ARammsWorldGenerator>(FTransform(FVector(-50000.0, 0.0, 0.0)));
	TestEqual(TEXT("with one exporting every piece, no contributor is added"), URammsPhysicsEngineSubsystem::AddWorldGenContributors(World), 0);
	TestEqual(TEXT("...not even for the new generator"), ContributorsFor(World, Third), 0);

	// An actor MuJoCo already converts (a quick-convert component) is not exported again, with or without a generator.
	ARammsProcPiece* Plain = SpawnSlab(Scene, FVector(0.0, 3000.0, 0.0));
	ARammsProcPiece* Converted = SpawnSlab(Scene, FVector(0.0, 6000.0, 0.0));
	ARammsProcPiece* ConvertedOfFirst = SpawnSlab(Scene, FVector(0.0, 9000.0, 0.0));
	if (!TestTrue(TEXT("pieces spawn"), Plain && Converted && ConvertedOfFirst))
	{
		return false;
	}
	AddQuickConvert(*Converted);
	AddQuickConvert(*ConvertedOfFirst);
	ConvertedOfFirst->AttachToActor(First, FAttachmentTransformRules::KeepWorldTransform);
	Plain->AttachToActor(First, FAttachmentTransformRules::KeepWorldTransform);

	TArray<FRammsCollisionShape> All;
	Everything->GatherShapes(All);
	TestTrue(TEXT("no generator: a plain piece is exported"), ShapesFrom(All, *Plain) > 0);
	TestEqual(TEXT("no generator: a quick-converted piece is not"), ShapesFrom(All, *Converted), 0);
	TestEqual(TEXT("no generator: nor one attached to a generator"), ShapesFrom(All, *ConvertedOfFirst), 0);
	TestTrue(TEXT("no generator: it supplies ground"), Everything->SuppliesGround());

	ARammsMjWorldGenContributor* OfFirst = nullptr;
	for (TActorIterator<ARammsMjWorldGenContributor> It(&World); It; ++It)
	{
		if (It->Generator.Get() == First)
		{
			OfFirst = *It;
		}
	}
	if (TestNotNull(TEXT("the first generator's contributor"), OfFirst))
	{
		TArray<FRammsCollisionShape> Ours;
		OfFirst->GatherShapes(Ours);
		TestTrue(TEXT("generator: its plain piece is exported"), ShapesFrom(Ours, *Plain) > 0);
		TestEqual(TEXT("generator: its quick-converted piece is not"), ShapesFrom(Ours, *ConvertedOfFirst), 0);
		TestTrue(TEXT("generator: it supplies ground"), OfFirst->SuppliesGround());
	}
	return true;
}

namespace RammsMjWorldGenExportTest
{
	/** Contact normals of a probe geom resting Depth (m) into two boxes, swept across the seam between them. */
	struct FSeamSweep
	{
		int32  Positions = 0;
		int32  WithoutContact = 0;
		double WorstHorizontal = 0.0;
		double WorstAtMm = 0.0;
	};

	/**
	 * A scene of its own: the two compiled boxes copied as they are, and a free body carrying the probe geom (Type, Size,
	 * Quat: MuJoCo's frame), its lowest point Lowest (m) under its centre. The probe moves along x from 5 cm before the
	 * seam to 5 cm past it in 1 mm steps, Depth into the boxes' top; each contact's normal is read after mj_forward.
	 */
	bool SweepSeam(const mjModel& Model, int32 BoxA, int32 BoxB, mjtGeom Type, const double Size[3], const double Quat[4], double Lowest, double Depth,
		FSeamSweep& Out, FString& Error)
	{
		mjSpec*	 Spec = mj_makeSpec();
		mjsBody* World = Spec ? mjs_findBody(Spec, "world") : nullptr;
		if (!World)
		{
			Error = TEXT("mj_makeSpec failed");
			if (Spec)
			{
				mj_deleteSpec(Spec);
			}
			return false;
		}
		for (const int32 Id : { BoxA, BoxB })
		{
			mjsGeom* Box = mjs_addGeom(World, nullptr);
			Box->type = mjGEOM_BOX;
			for (int32 k = 0; k < 3; ++k)
			{
				Box->size[k] = Model.geom_size[3 * Id + k];
				Box->pos[k] = Model.geom_pos[3 * Id + k];
			}
			for (int32 k = 0; k < 4; ++k)
			{
				Box->quat[k] = Model.geom_quat[4 * Id + k];
			}
		}
		mjsBody* Body = mjs_addBody(World, nullptr);
		mjs_addFreeJoint(Body);
		mjsGeom* Probe = mjs_addGeom(Body, nullptr);
		Probe->type = Type;
		for (int32 k = 0; k < 3; ++k)
		{
			Probe->size[k] = Size[k];
		}
		for (int32 k = 0; k < 4; ++k)
		{
			Probe->quat[k] = Quat[k];
		}
		mjModel* M = mj_compile(Spec, nullptr);
		if (!M)
		{
			Error = UTF8_TO_TCHAR(mjs_getError(Spec));
			mj_deleteSpec(Spec);
			return false;
		}
		mjData* D = mj_makeData(M);
		// Box A is the one at lower x; the seam is its +x face.
		const int32	 A = Model.geom_pos[3 * BoxA] < Model.geom_pos[3 * BoxB] ? BoxA : BoxB;
		const double SeamX = Model.geom_pos[3 * A] + Model.geom_size[3 * A];
		const double TopZ = Model.geom_pos[3 * A + 2] + Model.geom_size[3 * A + 2];
		const double Y = Model.geom_pos[3 * A + 1];
		const int32	 Q = M->jnt_qposadr[0];
		for (int32 s = -50; s <= 50; ++s)
		{
			D->qpos[Q + 0] = SeamX + s * 0.001;
			D->qpos[Q + 1] = Y;
			D->qpos[Q + 2] = TopZ + Lowest - Depth;
			D->qpos[Q + 3] = 1.0;
			D->qpos[Q + 4] = D->qpos[Q + 5] = D->qpos[Q + 6] = 0.0;
			mj_forward(M, D);
			++Out.Positions;
			if (D->ncon == 0)
			{
				++Out.WithoutContact;
			}
			for (int32 c = 0; c < D->ncon; ++c)
			{
				const mjtNum* N = D->contact[c].frame;
				const double  Horizontal = FMath::Sqrt(N[0] * N[0] + N[1] * N[1]);
				if (Horizontal > Out.WorstHorizontal)
				{
					Out.WorstHorizontal = Horizontal;
					Out.WorstAtMm = s;
				}
			}
		}
		mj_deleteData(D);
		mj_deleteModel(M);
		mj_deleteSpec(Spec);
		return true;
	}
} // namespace RammsMjWorldGenExportTest

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMjWorldGenSeamTest, "Ramms.MuJoCo.WorldGenExport.SeamContacts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsMjWorldGenSeamTest::RunTest(const FString& Parameters)
{
	// Coplanar neighbours (pavers, panels, the boxes around a pothole) are separate geoms with a seam between them. A
	// level surface pushes straight up; at a seam MuJoCo can also report a contact on the far geom's edge, whose normal
	// leans sideways. This measures how far, for a sphere, a wheel-like cylinder and a box like RAMMP Gen1's roller
	// pads, each 2 mm into the surface (about what a loaded soft contact sinks): it reports, it does not judge.
	using namespace RammsMjWorldGenExportTest;
	FExportScene Scene;
	if (!TestTrue(TEXT("world opens"), Scene.Open()))
	{
		AddError(Scene.LastError);
		return false;
	}
	ARammsProcPiece* Pavers = Scene.Spawn<ARammsProcPiece>(FTransform::Identity);
	if (!TestNotNull(TEXT("pavers spawn"), Pavers))
	{
		return false;
	}
	Pavers->Kind = ERammsProcPieceKind::GroundPrisms;
	Pavers->KindId = -1;
	Pavers->SizeZ = 10.0;
	Pavers->Parts = PrismParts({ { TEXT("Concrete"), Rect(0.0, 0.0, 100.0, 100.0, 0.0) }, { TEXT("Concrete"), Rect(100.0, 0.0, 200.0, 100.0, 0.0) } });
	Pavers->Rebuild();
	ARammsMjWorldGenContributor* Contributor = Scene.Spawn<ARammsMjWorldGenContributor>(FTransform::Identity);
	if (!TestNotNull(TEXT("contributor spawns"), Contributor)
		|| !TestTrue(TEXT("the scene compiles"), Scene.Manager->PhysicsEngine->InstallCompiledSpec(Scene.LastError)))
	{
		AddError(Scene.LastError);
		return false;
	}
	const mjModel* Model = Scene.Manager->PhysicsEngine->GetModel();
	TArray<int32>  Boxes;
	for (const int32 Id : Contributor->GetBoundGeomIds())
	{
		if (Model && Model->geom_type[Id] == mjGEOM_BOX)
		{
			Boxes.Add(Id);
		}
	}
	if (!TestEqual(TEXT("two paver boxes"), Boxes.Num(), 2))
	{
		return false;
	}

	struct FProbe
	{
		const TCHAR* Name;
		mjtGeom		 Type;
		double		 Size[3];
		double		 Quat[4];
		double		 Lowest;
	};
	const double H = 0.70710678118654752; // the cylinder's axis turned from z onto y
	const FProbe Probes[] = {
		{ TEXT("sphere r 5 cm"), mjGEOM_SPHERE, { 0.05, 0.0, 0.0 }, { 1.0, 0.0, 0.0, 0.0 }, 0.05 },
		{ TEXT("wheel (cylinder r 10 cm, 6 cm wide)"), mjGEOM_CYLINDER, { 0.10, 0.03, 0.0 }, { H, H, 0.0, 0.0 }, 0.10 },
		{ TEXT("roller pad (box 2.2 x 0.8 x 1.9 cm)"), mjGEOM_BOX, { 0.011, 0.004, 0.009375 }, { 1.0, 0.0, 0.0, 0.0 }, 0.009375 },
	};
	for (const FProbe& P : Probes)
	{
		FSeamSweep Sweep;
		FString	   Error;
		if (!TestTrue(FString::Printf(TEXT("%s: the probe scene compiles"), P.Name), SweepSeam(*Model, Boxes[0], Boxes[1], P.Type, P.Size, P.Quat, P.Lowest, 0.002, Sweep, Error)))
		{
			AddError(Error);
			continue;
		}
		TestEqual(FString::Printf(TEXT("%s: every position is in contact"), P.Name), Sweep.WithoutContact, 0);
		AddInfo(FString::Printf(TEXT("%s, 2 mm in, across the seam: largest sideways share of a contact normal %.3f, %+.0f mm from the seam (%d positions)"),
			P.Name, Sweep.WorstHorizontal, Sweep.WorstAtMm, Sweep.Positions));
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR && URLAB_MJ_GEN
