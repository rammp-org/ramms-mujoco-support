// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsPhysicsEngine.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Convert/AMjHeightfieldActor.h"
#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "RammsChaosArticulationComponent.h"
#include "RammsMjSimControlComponent.h"
#include "RammsMjWorldGenContributor.h"
#include "RammsWorldGenerator.h"
#include "TimerManager.h"
#include "UObject/UObjectIterator.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsPhysicsEngine, Log, All);

static TAutoConsoleVariable<FString> CVarRammsPhysicsEngine(TEXT("ramms.Physics.Engine"), TEXT(""),
	TEXT("Physics engine for the URLab articulations of the next level started: Chaos, MuJoCo, or empty for the level's ARammsPhysicsSettings."));

static TAutoConsoleVariable<int32> CVarRammsMjWorldShapes(TEXT("ramms.mj.WorldShapes"), 1,
	TEXT("Export a generated world (ARammsWorldGenerator) to MuJoCo as exact world geoms when a level runs on MuJoCo: ")
		TEXT("0 never; 1 unless the level's ARammsPhysicsSettings turns it off or the level samples its terrain with an ")
			TEXT("AMjHeightfieldActor (the two together would round every curb face again); 2 always. Read when a level starts."));

namespace
{
	/**
	 * One ARammsMjWorldGenContributor per world generator the level does not already export, so MuJoCo sees the
	 * generated world exactly. Returns how many were added.
	 */
	int32 AddWorldGenContributors(UWorld& World)
	{
		const int32 Mode = CVarRammsMjWorldShapes.GetValueOnGameThread();
		if (Mode <= 0 || !TActorIterator<ARammsWorldGenerator>(&World))
		{
			return 0;
		}
		if (Mode == 1)
		{
			for (TActorIterator<ARammsPhysicsSettings> It(&World); It; ++It)
			{
				if (!It->bMuJoCoWorldShapes)
				{
					UE_LOG(LogRammsPhysicsEngine, Log, TEXT("%s: the generated world is not exported to MuJoCo (ARammsPhysicsSettings)"), *World.GetMapName());
					return 0;
				}
				break;
			}
			if (TActorIterator<AMjHeightfieldActor>(&World))
			{
				UE_LOG(LogRammsPhysicsEngine, Warning,
					TEXT("%s: the level samples its terrain with an AMjHeightfieldActor, so the generated world is not exported to MuJoCo exactly ")
						TEXT("(the field would still turn each curb face into a slope). Remove the heightfield actor, or set ramms.mj.WorldShapes 2 to export both."),
					*World.GetMapName());
				return 0;
			}
		}
		TSet<const ARammsWorldGenerator*> Covered;
		for (TActorIterator<ARammsMjWorldGenContributor> It(&World); It; ++It)
		{
			Covered.Add(It->Generator.Get());
		}
		int32 Added = 0;
		for (TActorIterator<ARammsWorldGenerator> It(&World); It; ++It)
		{
			ARammsWorldGenerator* Generator = *It;
			if (Covered.Contains(Generator))
			{
				continue;
			}
			// Deferred, so the generator is set before its BeginPlay binds to the generator's OnWorldGenerated.
			ARammsMjWorldGenContributor* Contributor = World.SpawnActorDeferred<ARammsMjWorldGenContributor>(
				ARammsMjWorldGenContributor::StaticClass(), FTransform::Identity, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
			if (!Contributor)
			{
				continue;
			}
			Contributor->Generator = Generator;
			Contributor->FinishSpawning(FTransform::Identity);
#if WITH_EDITOR
			Contributor->SetActorLabel(FString::Printf(TEXT("MuJoCoWorld_%s"), *Generator->GetName()));
#endif
			++Added;
		}
		if (Added > 0)
		{
			UE_LOG(LogRammsPhysicsEngine, Log, TEXT("%s: %d generated world(s) exported to MuJoCo as world geoms"), *World.GetMapName(), Added);
		}
		return Added;
	}
} // namespace

namespace
{
	bool ParseEngine(const FString& Text, ERammsLevelPhysicsEngine& Out)
	{
		if (Text.Equals(TEXT("Chaos"), ESearchCase::IgnoreCase))
		{
			Out = ERammsLevelPhysicsEngine::Chaos;
			return true;
		}
		if (Text.Equals(TEXT("MuJoCo"), ESearchCase::IgnoreCase) || Text.Equals(TEXT("Mujoco"), ESearchCase::IgnoreCase))
		{
			Out = ERammsLevelPhysicsEngine::MuJoCo;
			return true;
		}
		if (Text.Equals(TEXT("Auto"), ESearchCase::IgnoreCase))
		{
			Out = ERammsLevelPhysicsEngine::Auto;
			return true;
		}
		return false;
	}
} // namespace

ERammsLevelPhysicsEngine URammsPhysicsEngineSubsystem::ResolveEngine(const UWorld& World, bool* bOutStartMuJoCoRunning, bool* bOutMuJoCoGround)
{
	ERammsLevelPhysicsEngine Result = ERammsLevelPhysicsEngine::Auto;
	bool					 bStart = true;
	bool					 bGround = true;
	for (TActorIterator<ARammsPhysicsSettings> It(const_cast<UWorld*>(&World)); It; ++It)
	{
		Result = It->Engine;
		bStart = It->bStartMuJoCoRunning;
		bGround = It->bMuJoCoGround;
		break;
	}
	ERammsLevelPhysicsEngine Override;
	FString					 Text;
	if (ParseEngine(CVarRammsPhysicsEngine.GetValueOnGameThread(), Override))
	{
		Result = Override;
	}
	if (FParse::Value(FCommandLine::Get(), TEXT("RammsPhysics="), Text) && ParseEngine(Text, Override))
	{
		Result = Override;
	}
	if (bOutStartMuJoCoRunning)
	{
		*bOutStartMuJoCoRunning = bStart;
	}
	if (bOutMuJoCoGround)
	{
		*bOutMuJoCoGround = bGround;
	}
	return Result;
}

bool URammsPhysicsEngineSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
	const UWorld* World = Cast<UWorld>(Outer);
	return World && World->IsGameWorld() && Super::ShouldCreateSubsystem(Outer);
}

void URammsPhysicsEngineSubsystem::OnWorldBeginPlay(UWorld& World)
{
	Super::OnWorldBeginPlay(World);
	bool bStartRunning = true;
	bool bGround = true;
	Engine = ResolveEngine(World, &bStartRunning, &bGround);
	if (Engine == ERammsLevelPhysicsEngine::Auto)
	{
		// As placed: a level with its own MuJoCo manager still gets its generated world in MuJoCo.
		if (TActorIterator<AAMjManager>(&World))
		{
			AddWorldGenContributors(World);
		}
		return;
	}

	int32 Articulations = 0;
	if (Engine == ERammsLevelPhysicsEngine::Chaos)
	{
		// Before any actor begins play: a manager that began would compile every articulation into MuJoCo too.
		TArray<AAMjManager*> Managers;
		for (TActorIterator<AAMjManager> It(&World); It; ++It)
		{
			Managers.Add(*It);
		}
		for (AAMjManager* Manager : Managers)
		{
			World.DestroyActor(Manager);
		}
		for (TActorIterator<AMjArticulation> It(&World); It; ++It)
		{
			ToChaos(*It);
			++Articulations;
		}
		SpawnHandle = World.AddOnActorSpawnedHandler(FOnActorSpawned::FDelegate::CreateUObject(this, &URammsPhysicsEngineSubsystem::OnActorSpawned));
		UE_LOG(LogRammsPhysicsEngine, Log, TEXT("%s: Chaos simulates the level's %d articulation(s)%s"), *World.GetMapName(), Articulations,
			Managers.Num() > 0 ? TEXT("; its MuJoCo manager was removed") : TEXT(""));
		return;
	}

	// MuJoCo.
	for (TActorIterator<AMjArticulation> It(&World); It; ++It)
	{
		if (URammsChaosArticulationComponent* Comp = It->FindComponentByClass<URammsChaosArticulationComponent>())
		{
			Comp->bSimulateWithChaos = false; // before its BeginPlay: it never builds
		}
		++Articulations;
	}
	// The world's contributors before the manager, so they are in the level whenever the manager's first compile
	// gathers its contributors; and before the ground, since an exported world is the ground.
	const int32 Exported = AddWorldGenContributors(World);
	bool		bSpawned = false;
	if (!TActorIterator<AAMjManager>(&World))
	{
		FActorSpawnParameters SP;
		SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		bSpawned = World.SpawnActor<AAMjManager>(AAMjManager::StaticClass(), FTransform::Identity, SP) != nullptr;
	}
	const bool bGroundAdded = bGround && SpawnMuJoCoGround(World);
	if (bStartRunning)
	{
		World.GetTimerManager().SetTimer(StartTimer, FTimerDelegate::CreateUObject(this, &URammsPhysicsEngineSubsystem::TryStartMuJoCo), 0.25f, true);
	}
	UE_LOG(LogRammsPhysicsEngine, Log, TEXT("%s: MuJoCo simulates the level's %d articulation(s)%s%s%s"), *World.GetMapName(), Articulations,
		bSpawned ? TEXT("; a MuJoCo manager was spawned") : TEXT(""), Exported > 0 ? TEXT("; the generated world is exported") : TEXT(""),
		bGroundAdded ? TEXT("; a MuJoCo ground was added") : TEXT(""));
}

bool URammsPhysicsEngineSubsystem::SpawnMuJoCoGround(UWorld& World)
{
	// Only when the level gives MuJoCo nothing static to stand on.
	for (TObjectIterator<UMjQuickConvertComponent> It; It; ++It)
	{
		if (It->GetWorld() == &World && It->Static)
		{
			return false;
		}
	}
	if (TActorIterator<AMjHeightfieldActor>(&World))
	{
		return false;
	}
	for (TActorIterator<ARammsMjWorldGenContributor> It(&World); It; ++It)
	{
		if (It->SuppliesGround())
		{
			return false; // the generated world, exactly
		}
	}
	// The floor under the articulations: the lowest surface a trace from each one's origin meets.
	double Ground = 0.0, SumX = 0.0, SumY = 0.0;
	int32  Hits = 0;
	for (TActorIterator<AMjArticulation> It(&World); It; ++It)
	{
		const FVector		  At = It->GetActorLocation();
		FHitResult			  Hit;
		FCollisionQueryParams Params(TEXT("RammsMuJoCoGround"), false, *It);
		if (World.LineTraceSingleByChannel(Hit, At + FVector(0.0, 0.0, 50.0), At - FVector(0.0, 0.0, 2000.0), ECC_WorldStatic, Params))
		{
			Ground = Hits == 0 ? Hit.ImpactPoint.Z : FMath::Min(Ground, Hit.ImpactPoint.Z);
			SumX += At.X;
			SumY += At.Y;
			++Hits;
		}
	}
	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (Hits == 0 || !Cube)
	{
		return false;
	}
	// A 1 km slab, 2 cm thick, its top at the floor: MuJoCo collides with it, Unreal does not see it.
	FActorSpawnParameters SP;
	SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	const FVector	  Centre(SumX / Hits, SumY / Hits, Ground - 1.0);
	AStaticMeshActor* Slab = World.SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FTransform(FRotator::ZeroRotator, Centre, FVector(1000.0, 1000.0, 0.02)), SP);
	if (!Slab)
	{
		return false;
	}
#if WITH_EDITOR
	Slab->SetActorLabel(TEXT("RammsMuJoCoGround"));
#endif
	Slab->GetStaticMeshComponent()->SetMobility(EComponentMobility::Movable);
	Slab->GetStaticMeshComponent()->SetStaticMesh(Cube);
	Slab->SetActorEnableCollision(false);
	Slab->SetActorHiddenInGame(true);
	UMjQuickConvertComponent* Quick = NewObject<UMjQuickConvertComponent>(Slab, TEXT("MuJoCoGround"));
	Quick->Static = true;
	Slab->AddInstanceComponent(Quick);
	Quick->RegisterComponent();
	return true;
}

void URammsPhysicsEngineSubsystem::ToChaos(AMjArticulation* Articulation)
{
	if (!Articulation)
	{
		return;
	}
	if (URammsChaosArticulationComponent* Comp = Articulation->FindComponentByClass<URammsChaosArticulationComponent>())
	{
		Comp->bSimulateWithChaos = true;
		Comp->EnsureBuilt(); // no-op before play: its BeginPlay builds
		return;
	}
	// A robot starts in the keyframe it would start in on MuJoCo.
	const URammsMjSimControlComponent* Sim = Articulation->FindComponentByClass<URammsMjSimControlComponent>();
	URammsChaosArticulationComponent::SwitchToChaos(Articulation, Sim ? Sim->InitialKeyframe : FString());
}

void URammsPhysicsEngineSubsystem::OnActorSpawned(AActor* Actor)
{
	AMjArticulation* Articulation = Cast<AMjArticulation>(Actor);
	if (!Articulation)
	{
		return;
	}
	if (Articulation->IsActorInitialized())
	{
		ToChaos(Articulation);
		return;
	}
	// A deferred spawn (SpawnActorDeferred, Blueprint Spawn Actor) is announced before its construction
	// script adds the Blueprint's components: convert it once those exist, so a Blueprint's own Chaos
	// component (and its sim-control keyframe) is the one that builds.
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().SetTimerForNextTick(FTimerDelegate::CreateWeakLambda(this, [this, Weak = TWeakObjectPtr<AMjArticulation>(Articulation)]() {
			if (AMjArticulation* A = Weak.Get())
			{
				ToChaos(A);
			}
		}));
	}
}

void URammsPhysicsEngineSubsystem::TryStartMuJoCo()
{
	UWorld* World = GetWorld();
	StartElapsed += 0.25f;
	AAMjManager* Manager = World ? AAMjManager::GetManager() : nullptr;
	if (Manager && Manager->GetWorld() == World)
	{
		if (!Manager->IsRunning())
		{
			Manager->SetPaused(false);
		}
		if (Manager->IsRunning())
		{
			World->GetTimerManager().ClearTimer(StartTimer);
			return;
		}
	}
	if (StartElapsed >= 15.0f && World)
	{
		UE_LOG(LogRammsPhysicsEngine, Warning, TEXT("gave up starting the MuJoCo simulation after %.0f s"), StartElapsed);
		World->GetTimerManager().ClearTimer(StartTimer);
	}
}

void URammsPhysicsEngineSubsystem::Deinitialize()
{
	if (UWorld* World = GetWorld())
	{
		if (SpawnHandle.IsValid())
		{
			World->RemoveOnActorSpawnedHandler(SpawnHandle);
		}
		World->GetTimerManager().ClearTimer(StartTimer);
	}
	Super::Deinitialize();
}
