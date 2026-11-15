// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMjWorldGenContributor.h"

#include "Components/LineBatchComponent.h"
#include "CoreGlobals.h"
#include "Engine/EngineTypes.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Hash/xxhash.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Gen/Elements/Assets/MjAsset.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjHfield.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjMesh.gen.h"
#include "MuJoCo/Gen/Elements/Bodies/MjBody.gen.h"
#include "MuJoCo/Gen/Elements/Bodies/MjFrame.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/MjEnums.gen.h"
#include "MuJoCo/Spec/MjFrameTypes.h"
#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "RammsCollisionShapes.h"
#include "RammsProcPiece.h"
#include "RammsWorldGenerator.h"
#include "TimerManager.h"
#include "UObject/UObjectIterator.h"

#if URLAB_MJ_GEN
	#include "MuJoCo/Spec/MjEffective.h"
	#include "MuJoCo/Spec/MjTreeAdapters.h"
#endif

#include <mujoco/mujoco.h>

DEFINE_LOG_CATEGORY_STATIC(LogRammsMjWorldGen, Log, All);

static TAutoConsoleVariable<int32> CVarRammsMjDrawWorldShapes(TEXT("ramms.mj.DrawWorldShapes"), 0,
	TEXT("Draw the generated world's geometry as MuJoCo compiled it (the world-gen contributors' geoms), in Unreal space: ")
		TEXT("0 off, 1 wireframe hidden behind the level, 2 wireframe drawn over everything. Boxes green, meshes cyan, ")
			TEXT("height fields yellow (with MuJoCo's triangle split), spheres and capsules magenta."));

static TAutoConsoleVariable<float> CVarRammsMjDrawWorldShapesRadius(TEXT("ramms.mj.DrawWorldShapesRadius"), 40.0f,
	TEXT("ramms.mj.DrawWorldShapes draws only geoms within this many metres (in plan) of the player's view; 0 draws all."));

static TAutoConsoleVariable<float> CVarRammsMjWorldShapesRadius(TEXT("ramms.mj.WorldShapesRadius"), 0.0f,
	TEXT("Export only the generated world within this many metres (in plan) of each contributor's centre (the ")
		TEXT("generator's suggested start); 0 uses the contributor's ExportRadius. Applies at the next compile ")
			TEXT("(ramms.mj.RecompileWorldShapes)."));

namespace
{
	/** The frame the last mid-play recompile ran in: several contributors asking in one frame share it. */
	uint64 GLastWorldShapesCompileFrame = 0;

	void RecompileWorldShapes(UWorld* World)
	{
		if (!World)
		{
			return;
		}
		int32 Count = 0;
		for (TActorIterator<ARammsMjWorldGenContributor> It(World); It; ++It)
		{
			It->RequestRecompile();
			++Count;
		}
		UE_LOG(LogRammsMjWorldGen, Log, TEXT("%d world-geometry contributor(s) re-export at the next tick"), Count);
	}

	FAutoConsoleCommandWithWorld GRammsMjRecompileWorldShapes(TEXT("ramms.mj.RecompileWorldShapes"),
		TEXT("Re-export the generated world to MuJoCo (every ARammsMjWorldGenContributor) and recompile the scene."),
		FConsoleCommandWithWorldDelegate::CreateStatic(&RecompileWorldShapes));

	/** A name MuJoCo and MJCF take as they are: letters, digits and underscores. */
	FString SanitizedName(FName Source)
	{
		FString S = Source.IsNone() ? FString(TEXT("shape")) : Source.ToString();
		for (int32 i = 0; i < S.Len(); ++i)
		{
			if (!FChar::IsAlnum(S[i]) && S[i] != TEXT('_'))
			{
				S[i] = TEXT('_');
			}
		}
		return S;
	}

	/** Smallest half size a box keeps (cm): MuJoCo rejects a size of zero. */
	constexpr double MinHalfCm = 0.025;

	/** The surface of a shape whose surface nothing names (RammsCollisionShapes): its friction is DefaultFriction. */
	const FName DefaultSurfaceName(TEXT("Default"));

	/**
	 * Whether Points span a volume (they do unless all lie within 0.01 mm of one plane). MuJoCo cannot hull a flat set
	 * and fails the whole compile, so a flat one is exported as the thin box it fills: OutCentre, OutRotation (in the
	 * points' frame) and OutHalf describe it.
	 */
	bool SpansVolume(const TArray<FVector3f>& Points, FVector& OutCentre, FQuat& OutRotation, FVector& OutHalf)
	{
		OutCentre = FVector::ZeroVector;
		OutRotation = FQuat::Identity;
		OutHalf = FVector(MinHalfCm);
		if (Points.Num() == 0)
		{
			return false;
		}
		const FVector P0(Points[0]);
		auto		  Farthest = [&](TFunctionRef<double(const FVector&)> Distance, double& OutDistance) -> FVector {
			 FVector Best = P0;
			 OutDistance = -1.0;
			 for (const FVector3f& P : Points)
			 {
				 const double D = Distance(FVector(P));
				 if (D > OutDistance)
				 {
					 OutDistance = D;
					 Best = FVector(P);
				 }
			 }
			 return Best;
		};
		double		  D = 0.0;
		const FVector A = Farthest([&](const FVector& P) { return FVector::Dist(P, P0); }, D);
		FVector		  U = (A - P0).GetSafeNormal();
		if (D < 1.0e-6)
		{
			OutCentre = P0;
			return false; // one point
		}
		const FVector B = Farthest([&](const FVector& P) { return FVector::CrossProduct(P - P0, U).Size(); }, D);
		FVector		  N = FVector::CrossProduct(U, B - P0).GetSafeNormal();
		if (N.IsNearlyZero())
		{
			N = FVector::CrossProduct(U, FMath::Abs(U.Z) < 0.9 ? FVector::UpVector : FVector::ForwardVector).GetSafeNormal();
		}
		Farthest([&](const FVector& P) { return FMath::Abs(FVector::DotProduct(P - P0, N)); }, D);
		if (Points.Num() >= 4 && D > 1.0e-3)
		{
			return true;
		}
		// Flat: the box in the plane's own axes (U, V, N).
		const FVector V = FVector::CrossProduct(N, U);
		FVector		  Lo(TNumericLimits<double>::Max()), Hi(-TNumericLimits<double>::Max());
		for (const FVector3f& P : Points)
		{
			const FVector L(FVector::DotProduct(FVector(P) - P0, U), FVector::DotProduct(FVector(P) - P0, V), FVector::DotProduct(FVector(P) - P0, N));
			Lo = Lo.ComponentMin(L);
			Hi = Hi.ComponentMax(L);
		}
		const FVector Mid = (Lo + Hi) * 0.5;
		OutCentre = P0 + U * Mid.X + V * Mid.Y + N * Mid.Z;
		OutRotation = FQuat(FMatrix(U, V, N, FVector::ZeroVector));
		OutHalf = ((Hi - Lo) * 0.5).ComponentMax(FVector(MinHalfCm));
		return false;
	}

#if URLAB_MJ_GEN
	/**
	 * Writes the shapes as elements under the contributor's spec root. Elements are built as their generated classes,
	 * not the registered hand subclasses: a UMjGeom builds a preview mesh and a material instance per geom, which
	 * thousands of static world geoms do not want (the level already shows them). Parents stay short: linking a child
	 * re-sorts its siblings, so the geoms go into <frame>s (which MuJoCo dissolves into the world body) and the assets
	 * into several <asset> sections, ChildrenPerParent each.
	 */
	struct FWorldSpecWriter
	{
		static constexpr int32 ChildrenPerParent = 64;

		AActor&								  Owner;
		UMjModel&							  Root;
		TArray<TObjectPtr<UMjNodeComponent>>& Elements;
		TArray<TObjectPtr<UMjNodeComponent>>& Geoms;

		UMjBodyBase* WorldBody = nullptr;
		UMjAsset*	 Assets = nullptr;
		int32		 AssetsUsed = 0;
		UMjFrame*	 Frame = nullptr;
		int32		 FrameUsed = 0;

		/** Shared mesh assets by content: hash -> indices of MeshVertices / MeshNames. */
		TMap<uint64, TArray<int32>> MeshesByHash;
		TArray<TArray<float>>		MeshVertices;
		TArray<FString>				MeshNames;
		/** Shared height fields by content. */
		struct FFieldKey
		{
			int32		   NumX = 0;
			int32		   NumY = 0;
			TArray<double> Size;
			TArray<double> Elevation;
			FString		   Name;
		};
		TMap<uint64, TArray<int32>> FieldsByHash;
		TArray<FFieldKey>			Fields;
		int32						NumBoxes = 0, NumMeshGeoms = 0, NumHfieldGeoms = 0, NumRound = 0, NumFlat = 0;

		FWorldSpecWriter(AActor& InOwner, UMjModel& InRoot, TArray<TObjectPtr<UMjNodeComponent>>& InElements, TArray<TObjectPtr<UMjNodeComponent>>& InGeoms)
			: Owner(InOwner), Root(InRoot), Elements(InElements), Geoms(InGeoms)
		{
		}

		template <class E, class P>
		E& NewNode(P& Parent, const TCHAR* BaseName)
		{
			E* Node = NewObject<E>(&Owner, E::StaticClass(), MakeUniqueObjectName(&Owner, E::StaticClass(), FName(BaseName)), RF_Transactional);
			Node->EnsureSerial();
			urlab::spec::FMjInstanceAdapter::Adopt<E>(Parent, urlab::spec::pssdk::kAppend, Node);
			Elements.Add(Node);
			return *Node;
		}

		UMjAsset& AssetSection()
		{
			if (!Assets || AssetsUsed >= ChildrenPerParent)
			{
				Assets = &NewNode<UMjAsset>(Root, TEXT("MjWorldAssets"));
				AssetsUsed = 0;
			}
			++AssetsUsed;
			return *Assets;
		}

		UMjFrame& GeomParent()
		{
			if (!WorldBody)
			{
				WorldBody = &NewNode<UMjBodyBase>(Root, TEXT("MjWorldBody"));
			}
			if (!Frame || FrameUsed >= ChildrenPerParent)
			{
				Frame = &NewNode<UMjFrame>(*WorldBody, TEXT("MjWorldFrame"));
				FrameUsed = 0;
			}
			++FrameUsed;
			return *Frame;
		}

		/** A static geom at an Unreal world pose, converted by URLab's own frame types (cm -> m, Y mirrored). */
		UMjGeomBase& Geom(const FRammsCollisionShape& Shape, EMjGeomType Type, const FVector& Location, const FQuat& Rotation, float Friction)
		{
			UMjGeomBase& G = NewNode<UMjGeomBase>(GeomParent(), TEXT("MjWorldGeom"));
			G.MjName = FString::Printf(TEXT("%s_%d"), *SanitizedName(Shape.Source), Geoms.Num());
			G.Type = Type;
			G.Pos = FMjPosition3::FromUnreal(Location);
			G.Quat = FMjQuatRot::FromUnreal(Rotation);
			// Sliding friction from the surface; torsional and rolling at MuJoCo's defaults. Group 3 is collision (hidden
			// by MuJoCo's viewers and by URammsMjCompiledGeomRenderer, whose default groups are 0-2).
			G.Friction = TArray<double>{ static_cast<double>(Friction), 0.005, 0.0001 };
			G.Group = 3;
			Geoms.Add(&G);
			return G;
		}

		void AddBox(const FRammsCollisionShape& Shape, const FVector& Location, const FQuat& Rotation, const FVector& HalfCm, float Friction)
		{
			const FVector H = HalfCm.GetAbs().ComponentMax(FVector(MinHalfCm));
			Geom(Shape, EMjGeomType::box, Location, Rotation, Friction).Size = TArray<double>{ H.X / 100.0, H.Y / 100.0, H.Z / 100.0 };
			++NumBoxes;
		}

		/** The <mesh> holding these points (cm, in their shape's frame), shared by every geom with the same points. */
		FString MeshFor(const TArray<FVector3f>& Points)
		{
			TArray<float> Vertex;
			Vertex.Reserve(Points.Num() * 3);
			for (const FVector3f& P : Points)
			{
				double M[3];
				URLabAxisConv::UePositionToMj(FVector(P), M);
				Vertex.Add(static_cast<float>(M[0]));
				Vertex.Add(static_cast<float>(M[1]));
				Vertex.Add(static_cast<float>(M[2]));
			}
			const uint64   Hash = FXxHash64::HashBuffer(Vertex.GetData(), static_cast<uint64>(Vertex.Num()) * sizeof(float)).Hash;
			TArray<int32>& Same = MeshesByHash.FindOrAdd(Hash);
			for (const int32 Index : Same)
			{
				if (MeshVertices[Index] == Vertex)
				{
					return MeshNames[Index];
				}
			}
			const FString Name = FString::Printf(TEXT("hull_%016llx_%d"), Hash, Same.Num());
			UMjMeshBase&  Mesh = NewNode<UMjMeshBase>(AssetSection(), TEXT("MjWorldMesh"));
			Mesh.MjName = Name;
			// No faces: MuJoCo collides a mesh geom as the convex hull of its vertices, and builds that hull itself.
			Mesh.Vertex = Vertex;
			Same.Add(MeshVertices.Num());
			MeshVertices.Add(MoveTemp(Vertex));
			MeshNames.Add(Name);
			return Name;
		}

		void AddConvex(const FRammsCollisionShape& Shape, float Friction)
		{
			FVector Centre, Half;
			FQuat	Rotation;
			if (!SpansVolume(Shape.Points, Centre, Rotation, Half))
			{
				AddBox(Shape, Shape.Transform.TransformPosition(Centre), Shape.Transform.GetRotation() * Rotation, Half, Friction);
				++NumFlat;
				return;
			}
			const FString Mesh = MeshFor(Shape.Points);
			Geom(Shape, EMjGeomType::mesh, Shape.Transform.GetLocation(), Shape.Transform.GetRotation(), Friction).Mesh = Mesh;
			++NumMeshGeoms;
		}

		/**
		 * A height grid as an <hfield> and its geom.
		 *
		 * MuJoCo centres a field on its geom, spans (-size_x, size_x) by (-size_y, size_y), puts the normalised
		 * elevation times size_z above the geom and the base size_base below it. The geom therefore sits at the grid's
		 * centre at its lowest sample. MJCF's `elevation` (which URLab's Elevation is) lists rows from the geom's -Y in
		 * Unreal to its +Y -- MuJoCo stores them the other way round, and its Y is Unreal's mirrored -- so the grid's
		 * row j goes in as row j, columns as they are. MuJoCo then splits a cell along the same diagonal the grid does
		 * ((i+1, j)-(i, j+1) in Unreal), so the surface is the grid's between the samples too.
		 */
		void AddHeightGrid(const FRammsCollisionShape& Shape, float Friction)
		{
			if (Shape.NumX < 2 || Shape.NumY < 2 || Shape.Heights.Num() != Shape.NumX * Shape.NumY || Shape.Spacing.X <= 0.0 || Shape.Spacing.Y <= 0.0)
			{
				UE_LOG(LogRammsMjWorldGen, Warning, TEXT("%s: a malformed height grid (%dx%d, %d heights) is not exported"), *Shape.Source.ToString(), Shape.NumX, Shape.NumY, Shape.Heights.Num());
				return;
			}
			float Lo = Shape.Heights[0], Hi = Shape.Heights[0];
			for (const float H : Shape.Heights)
			{
				Lo = FMath::Min(Lo, H);
				Hi = FMath::Max(Hi, H);
			}
			const double   Range = static_cast<double>(Hi) - Lo;
			const bool	   bFlat = Range < 1.0e-3; // cm: MuJoCo needs a positive elevation scale; flat data is all zero
			TArray<double> Elevation;
			Elevation.Reserve(Shape.Heights.Num());
			for (const float H : Shape.Heights)
			{
				Elevation.Add(bFlat ? 0.0 : (static_cast<double>(H) - Lo) / Range);
			}
			const double		 SpanX = (Shape.NumX - 1) * Shape.Spacing.X, SpanY = (Shape.NumY - 1) * Shape.Spacing.Y;
			const TArray<double> Size = { SpanX / 200.0, SpanY / 200.0, bFlat ? 0.001 : Range / 100.0, FMath::Max(static_cast<double>(Shape.BaseDepth), 0.1) / 100.0 };

			// Identical fields (the flares of every curb ramp of one size) share one <hfield>.
			const uint64 Hash = FXxHash64::HashBuffer(Elevation.GetData(), static_cast<uint64>(Elevation.Num()) * sizeof(double)).Hash
				^ (FXxHash64::HashBuffer(Size.GetData(), static_cast<uint64>(Size.Num()) * sizeof(double)).Hash * 0x9E3779B97F4A7C15ull)
				^ ((static_cast<uint64>(Shape.NumX) << 32) | static_cast<uint64>(Shape.NumY));
			FString		   Name;
			TArray<int32>& Same = FieldsByHash.FindOrAdd(Hash);
			for (const int32 Index : Same)
			{
				const FFieldKey& Key = Fields[Index];
				if (Key.NumX == Shape.NumX && Key.NumY == Shape.NumY && Key.Size == Size && Key.Elevation == Elevation)
				{
					Name = Key.Name;
					break;
				}
			}
			if (Name.IsEmpty())
			{
				Name = FString::Printf(TEXT("hfield_%016llx_%d"), Hash, Same.Num());
				UMjHfield& Field = NewNode<UMjHfield>(AssetSection(), TEXT("MjWorldHfield"));
				Field.MjName = Name;
				Field.Nrow = Shape.NumY;
				Field.Ncol = Shape.NumX;
				Field.Size = Size;
				Field.Elevation = Elevation;
				Same.Add(Fields.Num());
				Fields.Add({ Shape.NumX, Shape.NumY, Size, MoveTemp(Elevation), Name });
			}
			const FVector Centre = Shape.Transform.TransformPosition(FVector(SpanX * 0.5, SpanY * 0.5, Lo));
			Geom(Shape, EMjGeomType::hfield, Centre, Shape.Transform.GetRotation(), Friction).Hfield = Name;
			++NumHfieldGeoms;
		}

		void AddRound(const FRammsCollisionShape& Shape, float Friction)
		{
			const double R = FMath::Max(Shape.HalfExtents.X, MinHalfCm) / 100.0;
			const double Half = Shape.Type == ERammsCollisionShapeType::Capsule ? Shape.HalfExtents.Z / 100.0 : 0.0;
			if (Half > 1.0e-6)
			{
				Geom(Shape, EMjGeomType::capsule, Shape.Transform.GetLocation(), Shape.Transform.GetRotation(), Friction).Size = TArray<double>{ R, Half };
			}
			else
			{
				Geom(Shape, EMjGeomType::sphere, Shape.Transform.GetLocation(), Shape.Transform.GetRotation(), Friction).Size = TArray<double>{ R };
			}
			++NumRound;
		}
	};
#endif // URLAB_MJ_GEN
} // namespace

ARammsMjWorldGenContributor::ARammsMjWorldGenContributor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	// The spec root is the actor's root: FSpecRef finds it at once instead of walking thousands of geom components.
	Spec = CreateDefaultSubobject<UMjModel>(TEXT("Spec"));
	SetRootComponent(Spec);

	DebugLines = CreateDefaultSubobject<ULineBatchComponent>(TEXT("WorldShapeLines"));
	DebugLines->bCalculateAccurateBounds = false;

	// Sliding friction against rubber; see SurfaceFriction for how MuJoCo combines it. Hard surfaces keep the flat
	// ground's 1.0 (as does Default, through DefaultFriction), so a robot tuned on that ground sees the same traction.
	SurfaceFriction.Add(FName(TEXT("Asphalt")), 1.0f);
	SurfaceFriction.Add(FName(TEXT("Concrete")), 1.0f);
	SurfaceFriction.Add(FName(TEXT("Curb")), 1.0f);
	SurfaceFriction.Add(FName(TEXT("Brick")), 0.8f);
	SurfaceFriction.Add(FName(TEXT("Cobble")), 0.7f);
	SurfaceFriction.Add(FName(TEXT("Grass")), 0.6f);
	SurfaceFriction.Add(FName(TEXT("Dirt")), 0.6f);
	SurfaceFriction.Add(FName(TEXT("Gravel")), 0.6f);
	SurfaceFriction.Add(FName(TEXT("Mud")), 0.4f);
	SurfaceFriction.Add(FName(TEXT("Water")), 0.3f);
	SurfaceFriction.Add(FName(TEXT("Metal")), 0.5f);
	SurfaceFriction.Add(FName(TEXT("Wood")), 0.6f);
	SurfaceFriction.Add(FName(TEXT("Glass")), 0.4f);
	SurfaceFriction.Add(FName(TEXT("Foliage")), 0.5f);
}

FVector ARammsMjWorldGenContributor::ResolveCentre() const
{
	if (bCentreOnSuggestedStart && Generator && !Generator->Manifest.SuggestedStart.Equals(FTransform::Identity))
	{
		return Generator->Manifest.SuggestedStart.GetLocation();
	}
	return ExportCentre;
}

double ARammsMjWorldGenContributor::ResolveRadius() const
{
	const float Metres = CVarRammsMjWorldShapesRadius.GetValueOnGameThread();
	return Metres > 0.0f ? Metres * 100.0 : ExportRadius;
}

float ARammsMjWorldGenContributor::FrictionFor(FName Surface) const
{
	// Default (no known surface) is DefaultFriction, whatever the table says.
	if (!Surface.IsNone() && Surface != DefaultSurfaceName)
	{
		if (const float* F = SurfaceFriction.Find(Surface))
		{
			return *F;
		}
	}
	return DefaultFriction;
}

FRammsCollisionShapeFilter ARammsMjWorldGenContributor::MakeFilter() const
{
	FRammsCollisionShapeFilter Filter;
	const double			   Radius = ResolveRadius();
	if (Radius > 0.0)
	{
		Filter.Radius = Radius;
		Filter.Centre = ResolveCentre();
	}
	// MuJoCo already has an actor a quick-convert component converts: exported again, every contact with it would be
	// doubled.
	if (const UWorld* World = GetWorld())
	{
		for (TObjectIterator<UMjQuickConvertComponent> It; It; ++It)
		{
			if (It->GetWorld() == World && It->GetOwner())
			{
				Filter.ExcludedActors.Add(It->GetOwner());
			}
		}
	}
	return Filter;
}

namespace
{
	/** The shapes of every procedural piece in World that Filter keeps (a contributor with no generator). With
	 *  bStopAtFirst it stops at the first piece with one. Returns whether any were added. */
	bool GatherAllPieces(UWorld& World, const FRammsCollisionShapeFilter& Filter, TArray<FRammsCollisionShape>& Out, bool bStopAtFirst)
	{
		const int32					 First = Out.Num();
		TArray<FRammsCollisionShape> PieceShapes;
		for (TActorIterator<ARammsProcPiece> It(&World); It; ++It)
		{
			if (Filter.ExcludedActors.Contains(*It))
			{
				continue;
			}
			PieceShapes.Reset();
			It->AppendCollisionShapes(PieceShapes);
			for (FRammsCollisionShape& S : PieceShapes)
			{
				if (Filter.Accepts(RammsCollisionShapes::GetBounds(S)))
				{
					Out.Add(MoveTemp(S));
				}
			}
			if (bStopAtFirst && Out.Num() > First)
			{
				return true;
			}
		}
		return Out.Num() > First;
	}
} // namespace

void ARammsMjWorldGenContributor::GatherShapes(TArray<FRammsCollisionShape>& Out) const
{
	const FRammsCollisionShapeFilter Filter = MakeFilter();
	if (Generator)
	{
		Generator->GatherCollisionShapes(Out, Filter);
		return;
	}
	// No generator: every procedural piece in the level (hand-placed pieces, tests).
	if (UWorld* World = GetWorld())
	{
		GatherAllPieces(*World, Filter, Out, /*bStopAtFirst=*/false);
	}
}

bool ARammsMjWorldGenContributor::SuppliesGround() const
{
	const FRammsCollisionShapeFilter Filter = MakeFilter();
	if (Generator)
	{
		return Generator->HasCollisionShapes(Filter);
	}
	TArray<FRammsCollisionShape> Any;
	UWorld*						 World = GetWorld();
	return World && GatherAllPieces(*World, Filter, Any, /*bStopAtFirst=*/true);
}

FSpecRef ARammsMjWorldGenContributor::GetSceneSpec() const
{
	// Nothing exported: contribute nothing rather than an empty participant.
	if (!bAuthored || GeomElements.Num() == 0)
	{
		return FSpecRef();
	}
	return FSpecRef::OverActor(const_cast<ARammsMjWorldGenContributor&>(*this));
}

FString ARammsMjWorldGenContributor::GetScenePrefix() const
{
	return GetName() + TEXT("_");
}

void ARammsMjWorldGenContributor::AuthorSceneSpec()
{
	const double  Radius = ResolveRadius();
	const FVector Centre = Radius > 0.0 ? ResolveCentre() : FVector::ZeroVector;
	if (bAuthored && !bDirty && Radius == AuthoredRadius && Centre.Equals(AuthoredCentre, 1.0))
	{
		// The world has not changed since the last pass: the spec it left is still this world (idempotent by reuse).
		AuthoredAt = FPlatformTime::Seconds();
		return;
	}
#if URLAB_MJ_GEN
	if (!Spec)
	{
		return;
	}
	const double Start = FPlatformTime::Seconds();
	MjDestroySpecChildren(*Spec);
	Elements.Reset();
	GeomElements.Reset();
	BoundGeomIds.Reset();
	NumGeoms = NumMeshAssets = NumHeightfields = 0;

	TArray<FRammsCollisionShape> Shapes;
	GatherShapes(Shapes);
	const double Gathered = FPlatformTime::Seconds();

	FWorldSpecWriter Writer(*this, *Spec, Elements, GeomElements);
	for (const FRammsCollisionShape& S : Shapes)
	{
		const float Friction = FrictionFor(S.Surface);
		switch (S.Type)
		{
			case ERammsCollisionShapeType::Box:
				Writer.AddBox(S, S.Transform.GetLocation(), S.Transform.GetRotation(), S.HalfExtents, Friction);
				break;
			case ERammsCollisionShapeType::Convex:
				Writer.AddConvex(S, Friction);
				break;
			case ERammsCollisionShapeType::HeightGrid:
				Writer.AddHeightGrid(S, Friction);
				break;
			case ERammsCollisionShapeType::Sphere:
			case ERammsCollisionShapeType::Capsule:
				Writer.AddRound(S, Friction);
				break;
			default:
				break; // a shape type this exporter does not know yet
		}
	}

	// Register once the tree is whole, under one effective-value scope: registering an element syncs its preview
	// pose from the spec, and outside a shared scope each of those indexes the whole spec again.
	{
		urlab::spec::FMjEffectiveScope Effective(FSpecRef::OverActor(*this));
		for (UMjNodeComponent* Node : Elements)
		{
			if (Node && !Node->IsRegistered())
			{
				Node->RegisterComponent();
			}
		}
	}

	NumGeoms = GeomElements.Num();
	NumMeshAssets = Writer.MeshNames.Num();
	NumHeightfields = Writer.Fields.Num();
	bAuthored = true;
	bDirty = false;
	AuthoredRadius = Radius;
	AuthoredCentre = Centre;
	AuthoredAt = FPlatformTime::Seconds();
	UE_LOG(LogRammsMjWorldGen, Log,
		TEXT("%s: exported %d shapes as %d geoms (%d boxes, %d meshes over %d shared assets, %d hfield geoms over %d fields, %d spheres/capsules, %d flat hulls as boxes)%s in %.0f ms (gather %.0f ms)"),
		*GetName(), Shapes.Num(), NumGeoms, Writer.NumBoxes, Writer.NumMeshGeoms, NumMeshAssets, Writer.NumHfieldGeoms, NumHeightfields, Writer.NumRound, Writer.NumFlat,
		Radius > 0.0 ? *FString::Printf(TEXT(" within %.0f m of %s"), Radius / 100.0, *Centre.ToCompactString()) : TEXT(""),
		(AuthoredAt - Start) * 1000.0, (Gathered - Start) * 1000.0);
#else
	UE_LOG(LogRammsMjWorldGen, Warning, TEXT("%s: URLab was built without its generated spec profile (ProtoSpec): the generated world is not exported"), *GetName());
#endif
}

void ARammsMjWorldGenContributor::OnSceneBound()
{
	BoundGeomIds.Reset(GeomElements.Num());
	int32 Unbound = 0;
	for (const TObjectPtr<UMjNodeComponent>& Geom : GeomElements)
	{
		const int32 Id = Geom ? Geom->GetBoundId().Get(-1) : -1;
		if (Id >= 0)
		{
			BoundGeomIds.Add(Id);
		}
		else
		{
			++Unbound;
		}
	}
	++BindSerial;
	UE_LOG(LogRammsMjWorldGen, Log, TEXT("%s: %d world geoms in the compiled model%s; compiled and bound %.0f ms after authoring"),
		*GetName(), BoundGeomIds.Num(), Unbound > 0 ? *FString::Printf(TEXT(" (%d did not bind)"), Unbound) : TEXT(""),
		(FPlatformTime::Seconds() - AuthoredAt) * 1000.0);
}

void ARammsMjWorldGenContributor::BeginPlay()
{
	Super::BeginPlay();
	if (Generator)
	{
		Generator->OnWorldGenerated.AddUniqueDynamic(this, &ARammsMjWorldGenContributor::HandleWorldGenerated);
	}
	// Placed or spawned after the scene's first compile (which then could not see it): join with a recompile of our own.
	const AAMjManager* Manager = AAMjManager::GetManager();
	if (!bAuthored && Manager && Manager->GetWorld() == GetWorld() && Manager->IsInitialized())
	{
		UE_LOG(LogRammsMjWorldGen, Log, TEXT("%s: the MuJoCo scene compiled before this world was exported; recompiling with it"), *GetName());
		RequestRecompile();
	}
}

void ARammsMjWorldGenContributor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (Generator)
	{
		Generator->OnWorldGenerated.RemoveDynamic(this, &ARammsMjWorldGenContributor::HandleWorldGenerated);
	}
	if (DebugLines)
	{
		DebugLines->Flush();
	}
	Super::EndPlay(EndPlayReason);
}

void ARammsMjWorldGenContributor::HandleWorldGenerated(const FRammsWorldManifest& Manifest)
{
	bDirty = true;
	if (bRecompileOnGenerate)
	{
		RequestRecompile();
	}
}

void ARammsMjWorldGenContributor::RequestRecompile()
{
	bDirty = true;
	UWorld* World = GetWorld();
	if (!World || bRecompileQueued)
	{
		return;
	}
	bRecompileQueued = true;
	World->GetTimerManager().SetTimerForNextTick(this, &ARammsMjWorldGenContributor::RecompileNow);
}

void ARammsMjWorldGenContributor::RecompileNow()
{
	bRecompileQueued = false;
	if (bAuthored && !bDirty)
	{
		// A compile since the request (the manager's first, another contributor's) already exported this world.
		UE_LOG(LogRammsMjWorldGen, Log, TEXT("%s: the scene already compiled with this world since the request; no recompile"), *GetName());
		return;
	}
	AAMjManager* Manager = AAMjManager::GetManager();
	if (!Manager || Manager->GetWorld() != GetWorld())
	{
		UE_LOG(LogRammsMjWorldGen, Log, TEXT("%s: no MuJoCo manager is running here; the world is exported at its first compile"), *GetName());
		return;
	}
	if (GLastWorldShapesCompileFrame == GFrameCounter)
	{
		return; // another contributor's request compiled this frame, and re-authored this one with it
	}
	GLastWorldShapesCompileFrame = GFrameCounter;
	const double Start = FPlatformTime::Seconds();
	const bool	 bCompiled = Manager->CompileModel();
	const double Ms = (FPlatformTime::Seconds() - Start) * 1000.0;
	if (bCompiled)
	{
		UE_LOG(LogRammsMjWorldGen, Log, TEXT("%s: recompiled the MuJoCo scene with the new world in %.0f ms"), *GetName(), Ms);
	}
	else
	{
		UE_LOG(LogRammsMjWorldGen, Warning, TEXT("%s: the recompile failed after %.0f ms (%s); MuJoCo keeps the previous model"), *GetName(), Ms, *Manager->GetLastCompileError());
	}
}

void ARammsMjWorldGenContributor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	SinceDrawCheck += DeltaSeconds;
	if (SinceDrawCheck < 0.25f)
	{
		return;
	}
	SinceDrawCheck = 0.0f;

	const int32	 Mode = CVarRammsMjDrawWorldShapes.GetValueOnGameThread();
	const double Radius = FMath::Max(0.0f, CVarRammsMjDrawWorldShapesRadius.GetValueOnGameThread()) * 100.0;
	FVector		 View = DrawnCentre;
	if (UWorld* World = GetWorld())
	{
		if (APlayerController* PC = World->GetFirstPlayerController())
		{
			FRotator Unused;
			PC->GetPlayerViewPoint(View, Unused);
		}
	}
	const bool bMoved = Radius > 0.0 && FVector::Dist2D(View, DrawnCentre) > Radius * 0.25;
	if (Mode != DrawnMode || BindSerial != DrawnBindSerial || Radius != DrawnRadius || (Mode > 0 && bMoved))
	{
		RedrawShapes(Mode, View, Radius);
		DrawnMode = Mode;
		DrawnBindSerial = BindSerial;
		DrawnRadius = Radius;
		DrawnCentre = View;
	}
}

void ARammsMjWorldGenContributor::RedrawShapes(int32 Mode, const FVector& ViewCentre, double RadiusCm)
{
	if (!DebugLines)
	{
		return;
	}
	DebugLines->Flush();
	UMjPhysicsEngine* Engine = Mode > 0 ? AAMjManager::ResolveEngine(this) : nullptr;
	const mjModel*	  M = Engine ? Engine->GetModel() : nullptr;
	if (!M)
	{
		return;
	}
	const uint8			 Depth = Mode >= 2 ? SDPG_Foreground : SDPG_World;
	TArray<FBatchedLine> Lines;
	auto				 Line = [&](const FVector& A, const FVector& B, const FLinearColor& Color) { Lines.Emplace(A, B, Color, -1.0f, 0.0f, Depth); };
	const FLinearColor	 BoxColor(0.1f, 1.0f, 0.2f), MeshColor(0.0f, 0.9f, 1.0f), FieldColor(1.0f, 0.85f, 0.0f), RoundColor(1.0f, 0.2f, 0.9f);

	for (const int32 Id : BoundGeomIds)
	{
		if (Id < 0 || Id >= M->ngeom)
		{
			continue;
		}
		// The compiled pose (world-body geoms: world frame), in MuJoCo's frame, then into Unreal's through URLab.
		const mjtNum* P = M->geom_pos + 3 * Id;
		const mjtNum* Q = M->geom_quat + 4 * Id;
		const FVector Pm(P[0], P[1], P[2]);
		const FQuat	  Qm(Q[1], Q[2], Q[3], Q[0]);
		auto		  ToUe = [&](double X, double Y, double Z) {
			 const FVector W = Pm + Qm.RotateVector(FVector(X, Y, Z));
			 const double  Mj[3] = { W.X, W.Y, W.Z };
			 return URLabAxisConv::MjPositionToUe(Mj);
		};
		if (RadiusCm > 0.0 && FVector::Dist2D(ToUe(0.0, 0.0, 0.0), ViewCentre) > RadiusCm + M->geom_rbound[Id] * 100.0)
		{
			continue;
		}
		const mjtNum* S = M->geom_size + 3 * Id;
		switch (M->geom_type[Id])
		{
			case mjGEOM_BOX:
			{
				FVector C[8];
				for (int32 k = 0; k < 8; ++k)
				{
					C[k] = ToUe((k & 1) ? S[0] : -S[0], (k & 2) ? S[1] : -S[1], (k & 4) ? S[2] : -S[2]);
				}
				const int32 Edges[12][2] = { { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 } };
				for (const auto& E : Edges)
				{
					Line(C[E[0]], C[E[1]], BoxColor);
				}
				break;
			}
			case mjGEOM_MESH:
			{
				const int32 MeshId = M->geom_dataid[Id];
				if (MeshId < 0 || MeshId >= M->nmesh)
				{
					break;
				}
				const float* Vert = M->mesh_vert + 3 * M->mesh_vertadr[MeshId];
				const int32* Face = M->mesh_face + 3 * M->mesh_faceadr[MeshId];
				for (int32 f = 0; f < M->mesh_facenum[MeshId]; ++f)
				{
					FVector Corner[3];
					for (int32 c = 0; c < 3; ++c)
					{
						const float* V = Vert + 3 * Face[3 * f + c];
						Corner[c] = ToUe(V[0], V[1], V[2]);
					}
					Line(Corner[0], Corner[1], MeshColor);
					Line(Corner[1], Corner[2], MeshColor);
					Line(Corner[2], Corner[0], MeshColor);
				}
				break;
			}
			case mjGEOM_HFIELD:
			{
				const int32 Field = M->geom_dataid[Id];
				if (Field < 0 || Field >= M->nhfield)
				{
					break;
				}
				const int32	  Rows = M->hfield_nrow[Field], Cols = M->hfield_ncol[Field];
				const mjtNum* Size = M->hfield_size + 4 * Field;
				const float*  Data = M->hfield_data + M->hfield_adr[Field];
				if (Rows < 2 || Cols < 2)
				{
					break;
				}
				const double Dx = 2.0 * Size[0] / (Cols - 1), Dy = 2.0 * Size[1] / (Rows - 1);
				auto		 Sample = [&](int32 r, int32 c) { return ToUe(c * Dx - Size[0], r * Dy - Size[1], Data[r * Cols + c] * Size[2]); };
				for (int32 r = 0; r < Rows; ++r)
				{
					for (int32 c = 0; c < Cols; ++c)
					{
						if (c + 1 < Cols)
						{
							Line(Sample(r, c), Sample(r, c + 1), FieldColor);
						}
						if (r + 1 < Rows)
						{
							Line(Sample(r, c), Sample(r + 1, c), FieldColor);
						}
						// MuJoCo's split of the cell, as its collision and ray code make it.
						if (r + 1 < Rows && c + 1 < Cols)
						{
							Line(Sample(r, c), Sample(r + 1, c + 1), FieldColor * 0.6f);
						}
					}
				}
				// The base under the field.
				const FVector B[4] = { ToUe(-Size[0], -Size[1], -Size[3]), ToUe(Size[0], -Size[1], -Size[3]), ToUe(Size[0], Size[1], -Size[3]), ToUe(-Size[0], Size[1], -Size[3]) };
				const FVector T[4] = { Sample(0, 0), Sample(0, Cols - 1), Sample(Rows - 1, Cols - 1), Sample(Rows - 1, 0) };
				for (int32 k = 0; k < 4; ++k)
				{
					Line(B[k], B[(k + 1) % 4], FieldColor);
					Line(B[k], T[k], FieldColor);
				}
				break;
			}
			case mjGEOM_SPHERE:
			case mjGEOM_CAPSULE:
			{
				const double	R = S[0];
				const double	Half = M->geom_type[Id] == mjGEOM_CAPSULE ? S[1] : 0.0;
				constexpr int32 Segments = 16;
				for (int32 k = 0; k < Segments; ++k)
				{
					const double A0 = 2.0 * PI * k / Segments, A1 = 2.0 * PI * (k + 1) / Segments;
					for (const double Z : { Half, -Half })
					{
						Line(ToUe(R * FMath::Cos(A0), R * FMath::Sin(A0), Z), ToUe(R * FMath::Cos(A1), R * FMath::Sin(A1), Z), RoundColor);
					}
					// Two meridians through the caps.
					const double Za0 = FMath::Sin(A0) * R + (FMath::Sin(A0) >= 0.0 ? Half : -Half), Za1 = FMath::Sin(A1) * R + (FMath::Sin(A1) >= 0.0 ? Half : -Half);
					Line(ToUe(R * FMath::Cos(A0), 0.0, Za0), ToUe(R * FMath::Cos(A1), 0.0, Za1), RoundColor);
					Line(ToUe(0.0, R * FMath::Cos(A0), Za0), ToUe(0.0, R * FMath::Cos(A1), Za1), RoundColor);
				}
				break;
			}
			default:
				break;
		}
	}
	DebugLines->DrawLines(Lines);
	UE_LOG(LogRammsMjWorldGen, Verbose, TEXT("%s: drawing %d lines of the compiled world geometry"), *GetName(), Lines.Num());
}
