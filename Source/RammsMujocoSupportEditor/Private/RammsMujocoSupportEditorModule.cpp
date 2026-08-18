// Copyright Epic Games, Inc. All Rights Reserved.
//
// RammsMujocoSupportEditor: cross-backend rig tooling.
//
// "Generate Chaos Rig" (Tools ▸ RAMMS Robots) walks a URLab-imported MJCF
// Blueprint's Mj* components and generates the UE-native physics side into
// the SAME Blueprint: simple collision on the visual meshes (from the
// MjGeom primitives), one PhysicsConstraint per hinge/slide joint (limits,
// centring offset, spring drives, adjacent-collision disabled), point
// constraints for connect equalities (loop closures), and a
// URammsBackendSwitchComponent listing everything it made. One MJCF, one
// actor, MuJoCo/Newton OR Chaos per instance.

#include "AssetRegistry/AssetRegistryModule.h"
#include "ContentBrowserModule.h"
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"
#include "IContentBrowserSingleton.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "FileHelpers.h"
#include "Modules/ModuleManager.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Actuators/MjPositionActuator.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Constraints/MjEquality.h"
#include "MuJoCo/Components/Defaults/MjDefault.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Physics/MjInertial.h"
#include "MuJoCo/Components/Joints/MjFreeJoint.h"
#include "MuJoCo/Components/Joints/MjHingeJoint.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Joints/MjSlideJoint.h"
#include "MuJoCo/Components/Tendons/MjTendon.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "RammsBackendSwitchComponent.h"
#include "ToolMenus.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsRig, Log, All);

namespace
{

	struct FRigBody
	{
		USCS_Node*		   Node = nullptr;
		USCS_Node*		   ParentBody = nullptr; // nearest MjBody ancestor node
		USCS_Node*		   VizNode = nullptr;	 // first StaticMeshComponent child
		TArray<USCS_Node*> ExtraViz;			 // sibling visual pieces (multi-material)
		UMjJoint*		   Joint = nullptr;		 // first joint template under the body
		UMjInertial*	   Inertial = nullptr;	 // MJCF inertial (mass in kg), if any
		TArray<UMjGeom*>   Geoms;
	};

	static USCS_Node* NearestBodyAncestor(USCS_Node* Node, const TMap<USCS_Node*, USCS_Node*>& ParentOf)
	{
		USCS_Node* Cur = ParentOf.FindRef(Node);
		while (Cur && !Cast<UMjBody>(Cur->ComponentTemplate))
		{
			Cur = ParentOf.FindRef(Cur);
		}
		return Cur;
	}

	static void CollectRig(UBlueprint* BP, TMap<USCS_Node*, FRigBody>& OutBodies,
		TArray<UMjEquality*>& OutEqualities, TMap<USCS_Node*, USCS_Node*>& OutParentOf)
	{
		USimpleConstructionScript* SCS = BP->SimpleConstructionScript;
		TArray<USCS_Node*>		   All = SCS->GetAllNodes();
		for (USCS_Node* Node : All)
		{
			for (USCS_Node* Child : Node->GetChildNodes())
			{
				OutParentOf.Add(Child, Node);
			}
		}
		for (USCS_Node* Node : All)
		{
			if (Cast<UMjBody>(Node->ComponentTemplate))
			{
				FRigBody& Body = OutBodies.Add(Node);
				Body.Node = Node;
			}
			if (UMjEquality* Eq = Cast<UMjEquality>(Node->ComponentTemplate))
			{
				OutEqualities.Add(Eq);
			}
		}
		for (TPair<USCS_Node*, FRigBody>& Pair : OutBodies)
		{
			Pair.Value.ParentBody = NearestBodyAncestor(Pair.Key, OutParentOf);
			TArray<USCS_Node*> Stack = Pair.Key->GetChildNodes();
			while (Stack.Num())
			{
				USCS_Node*		 Child = Stack.Pop();
				UActorComponent* Tmpl = Child->ComponentTemplate;
				if (Cast<UMjBody>(Tmpl))
				{
					continue; // nested body: its own subtree
				}
				if (Cast<UStaticMeshComponent>(Tmpl))
				{
					if (!Pair.Value.VizNode)
					{
						Pair.Value.VizNode = Child;
					}
					else
					{
						Pair.Value.ExtraViz.Add(Child);
					}
				}
				if (UMjJoint* J = Cast<UMjJoint>(Tmpl))
				{
					if (!Pair.Value.Joint)
					{
						Pair.Value.Joint = J;
					}
				}
				if (UMjGeom* G = Cast<UMjGeom>(Tmpl))
				{
					Pair.Value.Geoms.Add(G);
				}
				if (UMjInertial* I = Cast<UMjInertial>(Tmpl))
				{
					if (!Pair.Value.Inertial)
					{
						Pair.Value.Inertial = I;
					}
				}
				Stack.Append(Child->GetChildNodes());
			}
		}
	}

	// NOTE: URLab canonicalizes SCENE-TRANSFORM values (component relative
	// pos/rot) to UE units (cm, Y-flip) at import — but the codegen-owned MJCF
	// attribute arrays (geom `size`, joint ranges on slides, equality anchors)
	// stay in NATIVE MuJoCo units (metres). Geom sizes therefore need x100
	// here; reading them as cm produced 100x-too-small (invisible, useless)
	// collision on every primitive-collider body while pose-derived values
	// stayed correct.

	static UPhysicalMaterial* GetOrCreatePhysMaterial(const TCHAR* Name, float Friction,
		EFrictionCombineMode::Type Combine)
	{
		const FString ObjPath = FString::Printf(TEXT("/Game/MuJoCoImports/%s.%s"), Name, Name);
		if (UPhysicalMaterial* Existing = LoadObject<UPhysicalMaterial>(nullptr, *ObjPath))
		{
			// Keep the saved asset in sync with the generator's values (the
			// caster material was retuned after first creation).
			if (!FMath::IsNearlyEqual(Existing->Friction, Friction)
				|| Existing->FrictionCombineMode != Combine)
			{
				Existing->Modify();
				Existing->Friction = Friction;
				Existing->bOverrideFrictionCombineMode = true;
				Existing->FrictionCombineMode = Combine;
				Existing->MarkPackageDirty();
			}
			return Existing;
		}
		UPackage*		   Pkg = CreatePackage(*FString::Printf(TEXT("/Game/MuJoCoImports/%s"), Name));
		UPhysicalMaterial* PM = NewObject<UPhysicalMaterial>(Pkg, Name, RF_Public | RF_Standalone);
		// Register + save NOW: an in-memory package that never hits disk is
		// invisible to LoadObject on the next session, the material was
		// silently recreated in memory each time and NEVER assigned from a
		// saved asset — every wheel ran on default friction (0.7 average)
		// for days. (User: "omniwheels have too little friction / spin the
		// wrong way" — they had none of the intended material at all.)
		FAssetRegistryModule::AssetCreated(PM);
		PM->Friction = Friction;
		PM->bOverrideFrictionCombineMode = true;
		PM->FrictionCombineMode = Combine;
		PM->Restitution = 0.f;
		PM->MarkPackageDirty();
		{
			TArray<UPackage*> ToSave;
			ToSave.Add(Pkg);
			UEditorLoadingAndSavingUtils::SavePackages(ToSave, true);
		}
		return PM;
	}

	// Rubber DRIVE wheels: high friction, MAX combine (MJCF priority=1
	// equivalent) — without it, straight driving slides laterally.
	// OMNIWHEEL casters: the opposite! They are passive rollers whose contact
	// patch must slip (MJCF friction 0.12); giving them grip converts any
	// linkage motion into propulsion (observed: rod commands launched the
	// robot and spun the casters).
	static UPhysicalMaterial* GetDriveWheelPhysMaterial()
	{
		return GetOrCreatePhysMaterial(TEXT("PM_RammsWheel"), 1.2f, EFrictionCombineMode::Max);
	}

	static UPhysicalMaterial* GetCasterPhysMaterial()
	{
		// 0.12/Min let the omniwheels SKATE (user: wheels not rotating, or
		// spinning against the travel direction) — Min-combine takes the
		// lower of wheel/floor so they never grip enough to roll. Chaos has
		// no anisotropic friction; approximate "grippy in roll, free
		// sideways" with a moderate isotropic 0.35 / Average.
		return GetOrCreatePhysMaterial(TEXT("PM_RammsCaster"), 0.35f, EFrictionCombineMode::Average);
	}

	static bool AddCollisionFromGeoms(const FRigBody& Body, TSet<UPackage*>& OutModifiedPackages,
		const TSet<UStaticMesh*>& BodyMeshes)
	{
		UStaticMeshComponent* Viz = Cast<UStaticMeshComponent>(Body.VizNode->ComponentTemplate);
		UStaticMesh*		  Mesh = Viz ? Viz->GetStaticMesh() : nullptr;
		if (!Mesh)
		{
			return false;
		}
		UBodySetup* Setup = Mesh->GetBodySetup();
		if (!Setup)
		{
			Mesh->CreateBodySetup();
			Setup = Mesh->GetBodySetup();
		}
		Setup->Modify();
		Setup->AggGeom.EmptyElements();
		int32 Added = 0;
		for (UMjGeom* Geom : Body.Geoms)
		{
			// Visual-only geoms (contype 0) and meshes/planes don't collide.
			if ((Geom->bOverride_contype && Geom->contype == 0)
				|| Geom->Type == EMjGeomType::Mesh || Geom->Type == EMjGeomType::Plane)
			{
				continue;
			}
			const FTransform Rel(Geom->GetRelativeRotation(), Geom->GetRelativeLocation());
			constexpr float	 M2Cm = 100.f; // codegen `size` array is metres
			TArray<float>	 Sz = Geom->size;
			for (float& V : Sz)
			{
				V *= M2Cm;
			}
			if (Geom->Type == EMjGeomType::Box && Sz.Num() >= 3)
			{
				FKBoxElem Box(Sz[0] * 2.f, Sz[1] * 2.f, Sz[2] * 2.f);
				Box.Center = Rel.GetLocation();
				Box.Rotation = Rel.Rotator();
				Setup->AggGeom.BoxElems.Add(Box);
				++Added;
			}
			else if (Geom->Type == EMjGeomType::Sphere && Sz.Num() >= 1)
			{
				FKSphereElem Sphere(Sz[0]);
				Sphere.Center = Rel.GetLocation();
				Setup->AggGeom.SphereElems.Add(Sphere);
				++Added;
			}
			else if ((Geom->Type == EMjGeomType::Cylinder || Geom->Type == EMjGeomType::Capsule)
				&& Sz.Num() >= 2)
			{
				// Cylinders use a capsule too: a faceted convex cylinder went
				// CONTACT-DEAD under load (weight transferred onto the drive
				// wheels during an elevator lift and the robot fell straight
				// through the floor — convex cook failure). The capsule's
				// point-contact lateral slide is handled by the high-friction
				// Max-combine wheel material instead. MuJoCo axis = local Z.
				FKSphylElem Sphyl(Sz[0], Sz[1] * 2.f);
				Sphyl.Center = Rel.GetLocation();
				Sphyl.Rotation = Rel.Rotator();
				Setup->AggGeom.SphylElems.Add(Sphyl);
				++Added;
			}
		}
		if (Added == 0)
		{
			// All colliders were mesh-type (e.g. the arm links): fall back to a
			// box of the visual mesh bounds so the body still simulates.
			const FBox	  Bounds = Mesh->GetBoundingBox();
			const FVector Ext = Bounds.GetExtent();
			FKBoxElem	  Box(Ext.X * 2.f, Ext.Y * 2.f, Ext.Z * 2.f);
			Box.Center = Bounds.GetCenter();
			Setup->AggGeom.BoxElems.Add(Box);
			++Added;
		}
		Setup->CollisionTraceFlag = CTF_UseDefault;
		{
			const FString BodyName = Body.VizNode->GetVariableName().ToString();
			if (BodyName.Contains(TEXT("drive_wheel")))
			{
				Setup->PhysMaterial = GetDriveWheelPhysMaterial();
			}
			else if (BodyName.Contains(TEXT("caster_wheel")))
			{
				Setup->PhysMaterial = GetCasterPhysMaterial();
			}
			else if (BodyName.Contains(TEXT("mebot__mebot__chassis")))
			{
				// Belly-resting parked pose: the chassis floor IS a contact.
				// With the drive wheels at 1.2/Max and the belly at default
				// 0.7/Avg, an elevator stroke that swings the carriage back
				// ROLLED the whole robot 90 cm (wheels gripped, belly slid);
				// MuJoCo (belly friction 1) lifts the chassis 4 cm instead.
				// Give the belly the same grip as the wheels.
				Setup->PhysMaterial = GetOrCreatePhysMaterial(TEXT("PM_RammsChassis"), 1.0f, EFrictionCombineMode::Max);
			}
		}
		Setup->InvalidatePhysicsData();
		Setup->CreatePhysicsMeshes();
		Mesh->MarkPackageDirty();

		// Extra visual pieces keep their import-time auto collision, which shows
		// up as stray sphere/box colliders on wheels in the collision view (and
		// confuses debugging even though the pieces are NoCollision at runtime).
		// Strip their assets down to no simple collision.
		for (USCS_Node* Extra : Body.ExtraViz)
		{
			UStaticMeshComponent* PieceViz = Cast<UStaticMeshComponent>(Extra->ComponentTemplate);
			UStaticMesh*		  PieceMesh = PieceViz ? PieceViz->GetStaticMesh() : nullptr;
			if (PieceMesh && BodyMeshes.Contains(PieceMesh))
			{
				// SHARED asset: this mesh is some body's collision carrier
				// (left/right arm links reuse one asset) — stripping it here
				// removed the arm's collision entirely. Leave it alone.
				continue;
			}
			UBodySetup* PieceSetup = PieceMesh ? PieceMesh->GetBodySetup() : nullptr;
			if (PieceSetup && (PieceSetup->AggGeom.GetElementCount() > 0))
			{
				PieceSetup->Modify();
				PieceSetup->AggGeom.EmptyElements();
				PieceSetup->InvalidatePhysicsData();
				PieceSetup->CreatePhysicsMeshes();
				PieceMesh->MarkPackageDirty();
				OutModifiedPackages.Add(PieceMesh->GetOutermost());
			}
		}
		// The collision lives in the MESH ASSET — it must be saved to disk or
		// it only exists for this editor session (user saw arm collision, which
		// the arm assets shipped with, but none on any base mesh).
		OutModifiedPackages.Add(Mesh->GetOutermost());
		return Added > 0;
	}

	static void GenerateChaosRig(UBlueprint* BP)
	{
		USimpleConstructionScript* SCS = BP->SimpleConstructionScript;

		// Regeneration: drop everything we made before. RemoveNode() only
		// detaches root-level nodes; constraint nodes are children of body
		// nodes, so use RemoveNodeAndPromoteChildren or repeated runs stack
		// renamed duplicates (l1/l2...) that keep stale, misplaced configs.
		// The backend-switch node is REUSED, never recreated: recreating it
		// orphans per-instance overrides on placed actors (observed: a placed
		// robot's Backend=Chaos silently reverted to MuJoCo after a
		// regeneration — every constraint then read as missing). Suffix-
		// tolerant: historical regens left uniquified names ("...Switch1").
		auto IsSwitchName = [](const FName& N) {
			const FString S = N.ToString();
			const FString Base = TEXT("ChaosRig_BackendSwitch");
			return S == Base || (S.StartsWith(Base) && S.Mid(Base.Len()).IsNumeric());
		};
		TArray<USCS_Node*> Stale;
		for (USCS_Node* Node : SCS->GetAllNodes())
		{
			if (Node->GetVariableName().ToString().StartsWith(TEXT("ChaosRig_"))
				&& !IsSwitchName(Node->GetVariableName()))
			{
				Stale.Add(Node);
			}
		}
		for (USCS_Node* Node : Stale)
		{
			// Free the template's object name immediately: removed templates
			// stay alive until GC, and CreateNode with the same name would be
			// uniquified ("ChaosRig_X1") — the PIE instances then inherit the
			// suffixed name and runtime lookups by recorded name miss them.
			if (UActorComponent* Tmpl = Node->ComponentTemplate)
			{
				Tmpl->Rename(nullptr, GetTransientPackage(),
					REN_DontCreateRedirectors | REN_DoNotDirty);
			}
			SCS->RemoveNodeAndPromoteChildren(Node);
		}
		if (Stale.Num())
		{
			UE_LOG(LogRammsRig, Display, TEXT("removed %d stale ChaosRig nodes"), Stale.Num());
		}

		TMap<USCS_Node*, FRigBody>	 Bodies;
		TArray<UMjEquality*>		 Equalities;
		TMap<USCS_Node*, USCS_Node*> ParentOf;
		CollectRig(BP, Bodies, Equalities, ParentOf);

		TMap<FString, UMjActuator*> ActuatorByJoint;
		TMap<FString, USCS_Node*>	DefaultNodeByClass; // MJCF default class -> node
		for (USCS_Node* Node : SCS->GetAllNodes())
		{
			if (UMjActuator* Act = Cast<UMjActuator>(Node->ComponentTemplate))
			{
				if (!Act->TargetName.IsEmpty())
				{
					ActuatorByJoint.Add(Act->TargetName, Act);
				}
			}
			if (UMjDefault* Def = Cast<UMjDefault>(Node->ComponentTemplate))
			{
				DefaultNodeByClass.Add(Def->ClassName, Node);
			}
		}
		// Class-based actuators (the gen3 arm) carry EMPTY gainprm/biasprm on
		// the actuator itself — the values live on an actuator template nested
		// under the <default class> (possibly up a parent chain). Resolve them.
		auto ResolveActuatorParams = [&DefaultNodeByClass](const UMjActuator* Act) -> const UMjActuator* {
			if (Act->gainprm.Num() > 0 || Act->biasprm.Num() > 0)
			{
				return Act;
			}
			FString Cls = Act->MjClassName;
			for (int32 Hop = 0; Hop < 4 && !Cls.IsEmpty(); ++Hop)
			{
				USCS_Node* const* DefNode = DefaultNodeByClass.Find(Cls);
				if (!DefNode)
				{
					break;
				}
				for (USCS_Node* Child : (*DefNode)->GetChildNodes())
				{
					if (const UMjActuator* DA = Cast<UMjActuator>(Child->ComponentTemplate))
					{
						if (DA->gainprm.Num() > 0 || DA->biasprm.Num() > 0)
						{
							return DA;
						}
					}
				}
				Cls = Cast<UMjDefault>((*DefNode)->ComponentTemplate)->ParentClassName;
			}
			return Act;
		};

		// Class-based JOINTS (the 2f85 gripper) also carry empty local
		// range/ref/axis/stiffness — resolve through the same default chain.
		// UPROPERTY values are already in UE units (deg / cm) per codegen.
		auto ResolveJointTemplate = [&DefaultNodeByClass](const UMjJoint*	J,
										TFunctionRef<bool(const UMjJoint*)> Has) -> const UMjJoint* {
			if (Has(J))
			{
				return J;
			}
			FString Cls = J->MjClassName;
			for (int32 Hop = 0; Hop < 4 && !Cls.IsEmpty(); ++Hop)
			{
				USCS_Node* const* DefNode = DefaultNodeByClass.Find(Cls);
				if (!DefNode)
				{
					break;
				}
				for (USCS_Node* Child : (*DefNode)->GetChildNodes())
				{
					if (const UMjJoint* DJ = Cast<UMjJoint>(Child->ComponentTemplate))
					{
						if (Has(DJ))
						{
							return DJ;
						}
					}
				}
				Cls = Cast<UMjDefault>((*DefNode)->ComponentTemplate)->ParentClassName;
			}
			return J;
		};

		// Accumulated relative transform of Node in Ancestor's frame (walks the
		// SCS chain; identity if Node==Ancestor). Needed because constraint
		// frames are authored body-local but applied at runtime relative to the
		// VIZ component, which can sit under offset geom nodes (arm/gripper).
		auto ChainToAncestor = [&ParentOf](USCS_Node* Node, USCS_Node* Ancestor) -> FTransform {
			FTransform T = FTransform::Identity;
			USCS_Node* Cur = Node;
			int32	   Guard = 0;
			while (Cur && Cur != Ancestor && Guard++ < 32)
			{
				if (USceneComponent* SC = Cast<USceneComponent>(Cur->ComponentTemplate))
				{
					T = T * SC->GetRelativeTransform();
				}
				Cur = ParentOf.FindRef(Cur);
			}
			return T;
		};

		TMap<FString, USCS_Node*> BodyByName; // Mj element name -> node
		for (TPair<USCS_Node*, FRigBody>& Pair : Bodies)
		{
			if (const UMjBody* BodyTmpl = Cast<UMjBody>(Pair.Key->ComponentTemplate))
			{
				BodyByName.Add(BodyTmpl->GetMjName(), Pair.Key);
			}
			BodyByName.Add(Pair.Key->GetVariableName().ToString(), Pair.Key);
		}

		URammsBackendSwitchComponent* Switch = nullptr;
		USCS_Node*					  SwitchNode = nullptr;
		for (USCS_Node* Node : SCS->GetAllNodes())
		{
			if (IsSwitchName(Node->GetVariableName()))
			{
				SwitchNode = Node;
				break;
			}
		}
		if (!SwitchNode)
		{
			SwitchNode = SCS->CreateNode(URammsBackendSwitchComponent::StaticClass(),
				TEXT("ChaosRig_BackendSwitch"));
			SCS->AddNode(SwitchNode);
		}
		Switch = Cast<URammsBackendSwitchComponent>(SwitchNode->ComponentTemplate);
		// Generator-owned arrays reset; user-facing settings (Backend,
		// bNeverSleep) survive regeneration.
		Switch->Modify();
		Switch->ChaosBodyComponents.Empty();
		Switch->ChaosConstraintComponents.Empty();
		Switch->ConstraintLocalFrames.Empty();
		Switch->ConstraintChildBodies.Empty();
		Switch->ConstraintBodyFrames.Empty();
		Switch->ConstraintBodyComponents.Empty();
		Switch->ConstraintTwistCenters.Empty();
		Switch->BodyMasses.Empty();
		Switch->PieceBodies.Empty();
		Switch->PieceMeshes.Empty();
		Switch->DriveJoints.Empty();
		Switch->DriveConstraints.Empty();
		Switch->DriveIsPosition.Empty();
		Switch->DriveIsLinear.Empty();
		Switch->DriveScale.Empty();
		Switch->DriveCtrlMin.Empty();
		Switch->DriveCtrlMax.Empty();
		Switch->CouplerLeaders.Empty();
		Switch->CouplerFollowers.Empty();
		Switch->CouplerRatios.Empty();

		int32										NumBodies = 0, NumConstraints = 0, NumClosures = 0;
		TSet<UPackage*>								ModifiedMeshPackages;
		TMap<FString, UPhysicsConstraintComponent*> ConstraintByJoint;
		TMap<FString, FName>						CNameByJoint;
		TSet<UStaticMesh*>							BodyMeshes;
		for (const TPair<USCS_Node*, FRigBody>& Pair : Bodies)
		{
			if (Pair.Value.VizNode)
			{
				if (UStaticMeshComponent* V = Cast<UStaticMeshComponent>(Pair.Value.VizNode->ComponentTemplate))
				{
					if (UStaticMesh* M = V->GetStaticMesh())
					{
						BodyMeshes.Add(M);
					}
				}
			}
		}

		// PHYSICS BODIES MUST BE UNIT-SCALE (2026-08-18). The arm/gripper
		// meshes import at 1000x geometry with a 0.001 compensating component
		// scale; a scaled physics body runs its ANGULAR constraint math in
		// mesh space, where configured torques are (1/scale)^2 = 1e6x too
		// weak. Measured: finger drives (k=1e4) and HARD 45.8 deg twist
		// windows had zero effect (fingers gravity-fell to 85 deg) while the
		// SAME constraints' linear rows enforced exactly — and the arm's
		// 5e7-scale servos barely held (effective ~50). Fix: bake the
		// component scale into the mesh asset's BuildScale (render-identical)
		// and reset every component using that asset to scale 1, BEFORE
		// collision/mass generation reads the mesh bounds.
		{
			TMap<UStaticMesh*, FVector> BakeScaleByMesh;
			for (TPair<USCS_Node*, FRigBody>& Pair : Bodies)
			{
				if (!Pair.Value.VizNode)
				{
					continue;
				}
				UStaticMeshComponent* Viz =
					Cast<UStaticMeshComponent>(Pair.Value.VizNode->ComponentTemplate);
				UStaticMesh* Mesh = Viz ? Viz->GetStaticMesh() : nullptr;
				if (!Mesh)
				{
					continue;
				}
				const FVector Scale = ChainToAncestor(Pair.Value.VizNode, nullptr).GetScale3D();
				if (Scale.Equals(FVector::OneVector, 1e-3f))
				{
					continue;
				}
				if (const FVector* Prev = BakeScaleByMesh.Find(Mesh))
				{
					if (!Prev->Equals(Scale, 1e-6f))
					{
						UE_LOG(LogRammsRig, Warning,
							TEXT("mesh %s used at conflicting scales (%s vs %s) — not baking"),
							*Mesh->GetName(), *Prev->ToString(), *Scale.ToString());
						BakeScaleByMesh.Remove(Mesh);
					}
					continue;
				}
				BakeScaleByMesh.Add(Mesh, Scale);
			}
			if (BakeScaleByMesh.Num())
			{
				for (TPair<UStaticMesh*, FVector>& MB : BakeScaleByMesh)
				{
					UStaticMesh* Mesh = MB.Key;
					Mesh->Modify();
					for (int32 Lod = 0; Lod < Mesh->GetNumSourceModels(); ++Lod)
					{
						FStaticMeshSourceModel& Src = Mesh->GetSourceModel(Lod);
						Src.BuildSettings.BuildScale3D *= MB.Value;
					}
					Mesh->PostEditChange(); // synchronous rebuild in editor
					Mesh->MarkPackageDirty();
					ModifiedMeshPackages.Add(Mesh->GetOutermost());
				}
				int32 Reset = 0;
				for (USCS_Node* Node : SCS->GetAllNodes())
				{
					UStaticMeshComponent* SMC =
						Cast<UStaticMeshComponent>(Node->ComponentTemplate);
					if (!SMC || !SMC->GetStaticMesh())
					{
						continue;
					}
					const FVector* Baked = BakeScaleByMesh.Find(SMC->GetStaticMesh());
					if (!Baked)
					{
						continue;
					}
					SMC->Modify();
					SMC->SetRelativeScale3D(SMC->GetRelativeScale3D() / *Baked);
					++Reset;
				}
				UE_LOG(LogRammsRig, Display,
					TEXT("unit-scale pre-pass: baked BuildScale into %d meshes, reset %d components"),
					BakeScaleByMesh.Num(), Reset);
			}
		}

		for (TPair<USCS_Node*, FRigBody>& Pair : Bodies)
		{
			FRigBody& Body = Pair.Value;
			if (!Body.VizNode)
			{
				UE_LOG(LogRammsRig, Warning, TEXT("body %s has no visual mesh — skipped"),
					*Pair.Key->GetVariableName().ToString());
				continue;
			}
			// Runtime floors gram-scale masses to 0.15 kg for solvability —
			// which breaks every SPRING equilibrium tuned for the real mass
			// (the 2f85 spring_links hold 12-22 g fingers at ~0 deg in MuJoCo;
			// against a 7-12x heavier floored link the same spring sags the
			// four-bar to its +51 deg window edge: the user's "default finger
			// poses are wrong"). Scale that body's spring drives by the same
			// inflation so the equilibrium ANGLE is preserved.
			float SpringMassScale = 1.f;
			if (AddCollisionFromGeoms(Body, ModifiedMeshPackages, BodyMeshes))
			{
				Switch->ChaosBodyComponents.Add(Body.VizNode->GetVariableName());
				// MJCF inertial mass if authored (gripper links are 12-22 g —
				// tiny but REAL, never "sanity"-replace them); else MuJoCo's
				// convention of geom volume x 1000 kg/m^3 (sizes are cm).
				const bool bHasInertial = Body.Inertial && Body.Inertial->mass > 0.f;
				float	   MassKg = bHasInertial ? Body.Inertial->mass : 0.f;
				if (!bHasInertial)
				{
					float VolCm3 = 0.f;
					for (const UMjGeom* Geom : Body.Geoms)
					{
						if ((Geom->bOverride_contype && Geom->contype == 0)
							|| Geom->Type == EMjGeomType::Plane
							|| Geom->Type == EMjGeomType::Mesh)
						{
							continue;
						}
						TArray<float> Sz = Geom->size;
						for (float& V : Sz)
						{
							V *= 100.f; // metres -> cm (see NOTE above)
						}
						if (Geom->Type == EMjGeomType::Box && Sz.Num() >= 3)
						{
							VolCm3 += 8.f * Sz[0] * Sz[1] * Sz[2];
						}
						else if (Geom->Type == EMjGeomType::Sphere && Sz.Num() >= 1)
						{
							VolCm3 += (4.f / 3.f) * PI * Sz[0] * Sz[0] * Sz[0];
						}
						else if ((Geom->Type == EMjGeomType::Cylinder
									 || Geom->Type == EMjGeomType::Capsule)
							&& Sz.Num() >= 2)
						{
							VolCm3 += PI * Sz[0] * Sz[0] * (2.f * Sz[1]);
						}
					}
					MassKg = VolCm3 / 1000.f; // x1000 kg/m^3, /1e6 cm^3->m^3
				}
				if (!bHasInertial && MassKg <= 0.001f)
				{
					// Mesh-collider body (no primitives): estimate from the
					// visual bounds at a hollow-part fill factor. Asset bounds
					// are mesh-local — apply the component scale (imported OBJ
					// assets can be 100x with a 0.01 compensating scale).
					if (const UStaticMeshComponent* VizC =
							Cast<UStaticMeshComponent>(Body.VizNode->ComponentTemplate))
					{
						if (const UStaticMesh* VizMesh = VizC->GetStaticMesh())
						{
							const FVector Sc = VizC->GetRelativeScale3D().GetAbs();
							const FVector E = VizMesh->GetBoundingBox().GetExtent() * Sc;
							MassKg = (8.f * E.X * E.Y * E.Z) * 0.15f / 1000.f;
						}
					}
				}
				if (!bHasInertial)
				{
					// Estimated masses only: thin shells compute absurdly light
					// (0.02 kg links). Sub-kg bodies against stiff drives give
					// 500:1 mass ratios and the solver kicks the assembly
					// through the world. Real metal parts are >= ~0.5 kg.
					MassKg = FMath::Clamp(MassKg, 0.5f, 200.f);
				}
				Switch->BodyMasses.Add(MassKg);
				if (MassKg > 0.001f && MassKg < 0.15f)
				{
					SpringMassScale = FMath::Min(0.15f / MassKg, 15.f);
				}
				for (USCS_Node* Extra : Body.ExtraViz)
				{
					Switch->PieceBodies.Add(Body.VizNode->GetVariableName());
					Switch->PieceMeshes.Add(Extra->GetVariableName());
				}
				++NumBodies;
			}

			// Joint -> constraint. Free joint = floating body (no constraint).
			// NO joint = rigid attachment in MuJoCo -> full weld constraint,
			// otherwise the body free-falls out of the robot under Chaos.
			UMjJoint* J = Body.Joint;
			if ((J && Cast<UMjFreeJoint>(J)) || !Body.ParentBody)
			{
				continue;
			}
			const FRigBody* ParentRig = Bodies.Find(Body.ParentBody);
			if (!ParentRig || !ParentRig->VizNode)
			{
				continue;
			}
			const bool bHinge = J && Cast<UMjHingeJoint>(J) != nullptr;
			const bool bSlide = J && Cast<UMjSlideJoint>(J) != nullptr;
			const bool bWeld = (J == nullptr);
			if (!bHinge && !bSlide && !bWeld)
			{
				continue; // ball joints TODO
			}

			const FString CName = FString::Printf(TEXT("ChaosRig_%s"),
				*Pair.Key->GetVariableName().ToString());
			USCS_Node*	  CNode = SCS->CreateNode(UPhysicsConstraintComponent::StaticClass(), *CName);
			Pair.Key->AddChildNode(CNode);
			UPhysicsConstraintComponent* C =
				Cast<UPhysicsConstraintComponent>(CNode->ComponentTemplate);

			const UMjJoint* AxisSrc = nullptr;
			const UMjJoint* RangeSrc = nullptr;
			const UMjJoint* RefSrc = nullptr;
			const UMjJoint* SpringSrc = nullptr;
			if (J)
			{
				AxisSrc = ResolveJointTemplate(J, [](const UMjJoint* X) { return X->bOverride_Axis; });
				RangeSrc = ResolveJointTemplate(J, [](const UMjJoint* X) { return X->bOverride_range && X->range.Num() >= 2; });
				RefSrc = ResolveJointTemplate(J, [](const UMjJoint* X) { return X->bOverride_ref; });
				SpringSrc = ResolveJointTemplate(J, [](const UMjJoint* X) { return X->bOverride_stiffness && X->stiffness.Num() > 0; });
				C->SetRelativeLocation(J->Pos);
				// MJCF joint axes are in the OWNING body's local frame (the
				// URLab tooltip's "parent body" means the joint's owner link) —
				// use them directly. An earlier parent-frame Unrotate transform
				// silently corrupted the axis of every rotated body (all arm
				// links): holds looked fine (orientation drives hold any frame)
				// but commanded motion rotated about the wrong axis.
				const FVector RawAxis = AxisSrc->bOverride_Axis ? AxisSrc->Axis : FVector(0, 0, 1);
				const FVector Axis = RawAxis.GetSafeNormal(1e-6f, FVector::ZAxisVector);
				C->SetRelativeRotation(FRotationMatrix::MakeFromX(Axis).Rotator());
			}
			C->ComponentName1.ComponentName = ParentRig->VizNode->GetVariableName();
			C->ComponentName2.ComponentName = Body.VizNode->GetVariableName();

			FConstraintInstance& CI = C->ConstraintInstance;
			CI.ProfileInstance.bDisableCollision = true;
			// Projection: the gripper couples 12-22 g links to the 8 kg arm
			// through limits + closure loops — a ~400:1 mass ratio the 60 Hz
			// solver pumps energy into (robot back-flipped at spawn) unless the
			// error is projected out positionally.
			CI.ProfileInstance.bEnableProjection = true;
			// ANGULAR projection (gripper joints only): UE defaults
			// ProjectionAngularAlpha to 0, so "projection on" only ever
			// projected LINEAR error — the reason the gripper's linear rows
			// enforced crisply while its angular windows leaked 10-40 deg
			// under closure-loop load. Measured 2026-08-18 (rest, loop
			// closed): alpha 0 -> stops overrun 9-16 deg; alpha 1 -> windows
			// crisp but the pins tear to 0.15-0.23 cm; alpha 0.25 -> all
			// windows respected AND pins 0.027-0.036 cm. Gripper only: the
			// mebot ground loops detonate under hard angular enforcement
			// (gotcha 15).
			if (Pair.Key->GetVariableName().ToString().StartsWith(TEXT("arm_2f85_")))
			{
				CI.ProfileInstance.ProjectionAngularAlpha = 0.25f;
			}
			// UE-twist-space window center recorded for the runtime (deg;
			// 0 = symmetric window, nothing to offset).
			float	   TwistCenterDeg = 0.f;
			const bool bLimited = J && RangeSrc->bOverride_range
				&& RangeSrc->range.Num() >= 2 && RangeSrc->range[1] > RangeSrc->range[0];
			if (bWeld)
			{
				CI.SetLinearXLimit(LCM_Locked, 0.f);
				CI.SetLinearYLimit(LCM_Locked, 0.f);
				CI.SetLinearZLimit(LCM_Locked, 0.f);
				CI.SetAngularSwing1Limit(ACM_Locked, 0.f);
				CI.SetAngularSwing2Limit(ACM_Locked, 0.f);
				CI.SetAngularTwistLimit(ACM_Locked, 0.f);
			}
			else if (bHinge)
			{
				CI.SetLinearXLimit(LCM_Locked, 0.f);
				CI.SetLinearYLimit(LCM_Locked, 0.f);
				CI.SetLinearZLimit(LCM_Locked, 0.f);
				CI.SetAngularSwing1Limit(ACM_Locked, 0.f);
				CI.SetAngularSwing2Limit(ACM_Locked, 0.f);
				// Swing locks default to SOFT (cone stiffness 50 — mush).
				CI.ProfileInstance.ConeLimit.bSoftConstraint = false;
				const bool bMebotLinkage = !Pair.Key->GetVariableName().ToString().StartsWith(TEXT("arm_"));
				const bool bGripper = Pair.Key->GetVariableName().ToString().StartsWith(TEXT("arm_2f85_"));
				if (bLimited && bGripper)
				{
					// ASYMMETRIC window, honored exactly (2f85 four-bar). The
					// old sign-safe symmetric window (max half-range both ways)
					// let the fingers gravity-fall ~46 deg into the nonphysical
					// OPEN region (MJCF driver range [0,45.8] — the open stop
					// is AT the spawn pose). Out there the closure-pin loop
					// crosses its toggle singularity and the pin rows shove the
					// hinges to +-83 deg THROUGH their hard windows. Measured
					// (2026-08-18): pins disabled -> every window enforces
					// exactly (follower 50.2, coupler 90.0); 5-deg windows with
					// the loop closed -> healthy. Fix: window half = the true
					// half-range, center offset applied at runtime by rotating
					// the parent ref frame (UE twist = -MJCF angle, the same
					// handedness flip DriveScale=-1 encodes).
					const float LoUE = -RangeSrc->range[1];
					const float HiUE = -RangeSrc->range[0];
					TwistCenterDeg = 0.5f * (LoUE + HiUE);
					const float HalfW = 0.5f * (HiUE - LoUE);
					CI.SetAngularTwistLimit(ACM_Limited, FMath::Max(HalfW, 1.f));
					CI.ProfileInstance.TwistLimit.bSoftConstraint = false;
				}
				else if (bLimited && !bMebotLinkage)
				{
					// SIGN-SAFE window (arm only): symmetric around the
					// modeled pose, wide enough to cover the full range — a
					// wrongly-signed asymmetric window kicks at spawn.
					const float Ref = RefSrc->bOverride_ref ? RefSrc->ref : 0.f;
					const float Half = FMath::Max(
						FMath::Abs(RangeSrc->range[1] - Ref),
						FMath::Abs(Ref - RangeSrc->range[0]));
					CI.SetAngularTwistLimit(ACM_Limited, FMath::Max(Half, 1.f));
					// HARD stop: UE angular limits default to SOFT with
					// stiffness 50 — mush. MuJoCo enforces these ranges stiffly
					// (solreflimit 0.005); hard windows are the equivalent.
					// (Mebot linkage windows stay SOFT below — hard windows
					// against the ground-coupled loops detonate, gotcha 15.)
					CI.ProfileInstance.TwistLimit.bSoftConstraint = false;
				}
				else if (bLimited)
				{
					// Mebot linkage twists: SOFT symmetric windows. Fully FREE
					// twists let the multi-bar front chain fold through its
					// toggle configuration instead of transmitting (probe: the
					// linkage piece at -110 deg twist while the swing arm sat
					// still and the 20 kN rod flipped the chassis) — MuJoCo
					// never jackknifes because its soft +-30 deg joint limits
					// keep the loop in the working range. HARD windows against
					// the closures detonated (gotcha 15); soft springs bound the
					// travel without the solver fight.
					const float Ref = RefSrc->bOverride_ref ? RefSrc->ref : 0.f;
					float		Half = FMath::Max(
						  FMath::Abs(RangeSrc->range[1] - Ref),
						  FMath::Abs(Ref - RangeSrc->range[0]));
					// (A 15-deg clamp on the suspension arms was tried as a
					// strut stand-in and DETONATED — window spring vs rod
					// force is the classic limits-vs-loops energy pump. The
					// caster rods are force-capped instead; see the actuator
					// block.)
					CI.SetAngularTwistLimit(ACM_Limited, FMath::Max(Half, 1.f));
					CI.ProfileInstance.TwistLimit.bSoftConstraint = true;
					CI.ProfileInstance.TwistLimit.Stiffness = 1e6f;
					CI.ProfileInstance.TwistLimit.Damping = 1e5f;
				}
				else
				{
					CI.SetAngularTwistLimit(ACM_Free, 0.f);
				}
				// MuJoCo hinge springs are N*m/rad; UE angular drives take
				// kg*cm^2/s^2 per rad -> x1e4. (Earlier clamps of 5000/500 were
				// guarding against the duplicate-constraint explosions and left
				// the suspension springs ~4 orders too weak — the linkage
				// collapsed under the robot's weight.)
				const float RawStiff = (SpringSrc->bOverride_stiffness && SpringSrc->stiffness.Num())
					? SpringSrc->stiffness[0]
					: 0.f;
				if (RawStiff >= 5e4f)
				{
					// True lock stand-in (exporter models parked DOFs as 1e5
					// springs): hard-lock at the modeled pose. The 2e4 linkage
					// stabilizer springs must stay springs — the driven
					// suspension moves through them (locking froze the
					// elevator chain entirely).
					CI.SetAngularTwistLimit(ACM_Locked, 0.f);
				}
				else if (RawStiff > 0.f)
				{
					// SpringMassScale keeps the spring/gravity equilibrium
					// angle of mass-floored gram-scale links (see above). For
					// the gripper that per-body scale is NOT enough: the
					// spring_link spring restrains the WHOLE floored finger
					// chain (~0.45 kg vs ~0.06 real), so floor the drive at
					// 5e4 (5 N*m/rad) — sag <2 deg for the floored chain, and
					// still 20x weaker than the finger command drive, so it
					// cannot fight commanded motion.
					const bool bArm = Pair.Key->GetVariableName().ToString().StartsWith(TEXT("arm_"));
					float	   Stiff = FMath::Min(RawStiff * 1e4f * SpringMassScale, 2e6f);
					float	   Damp = FMath::Min(
						 (J->damping.Num() ? J->damping[0] : 0.f) * 1e4f * SpringMassScale, 2e5f);
					if (bArm && SpringMassScale > 1.f)
					{
						// 1.5e4 = 1.5 N*m/rad, paired with the LIGHTER 0.05 kg
						// arm mass floor at runtime (1/3 the chain weight needs
						// 1/3 the spring for the same rest angle). The old 5e4
						// held rest but overconstrained the underactuated
						// spring_link/follower side during a close (user
						// report); the driver now only fights ~0.7 N*m of
						// spring to drag the loop.
						Stiff = FMath::Max(Stiff, 1.5e4f);
						Damp = FMath::Max(Damp, 1.5e3f);
					}
					// Mebot linkage STAND-IN springs (front_caster_swing_arm
					// 200, elevator pivots 1000, dampener pivots 20) are
					// MuJoCo-side stabilizers for a soft-closure world. Under
					// Chaos the hard pins + geometry already define those
					// poses, and the x1e4 conversion turns 200 N*m/rad into a
					// wall: measured on the front caster, the rod pushed 6 kN
					// and articulated the linkage_arm 18 deg while the swing
					// arm sat at 0 deg behind a 2e6 orientation hold — the
					// force reacted against the chassis and flipped the robot.
					// Keep them only as light damping (no orientation hold).
					// Real struts (dampener slides, 2e4) are the SLIDE branch.
					if (!bArm)
					{
						// Not ZERO either: with no hold at all the parked front
						// chain has enough slack that the robot settled upside
						// down. Keep 1/10 of the converted spring — enough to
						// park the linkage, ~20x weaker than the rod's lever
						// force so it can never wall the mechanism.
						// The stand-in holds ARE load-bearing at rest: with none
						// the robot never settles (upz 0.28, slides 10 m at
						// spawn); with 1/10 it parks upright. The rod fly-apart
						// is NOT the hold (it flips with hold 0/1e5/2e6 alike;
						// swing arm reaches -27 deg with no hold and it still
						// flips) — see 6.15. Keep 1/10 park hold.
						// 1/10 was NOT enough either — the robot never settled
						// (upz 0.26, slid 10 m at spawn). Full stand-in strength
						// it is; the swing-arm "wall" was a symptom of undamped
						// rod servos hammering, not of the hold (6.15).
						const FString VN = Pair.Key->GetVariableName().ToString();
						// Bell-crank PIVOTS carry the strut and must swing freely:
						// any hold there becomes the chassis reaction point for
						// the whole rear stroke (probe: pivot hinge 10 kN, strut
						// 570 N, robot flips 6 s after a +0.03 rod command). The
						// compose script already gives them a token 20-stiffness
						// spring purely for MuJoCo settling — damping only here.
						// Strut HINGES (elevator_dampener, elevator_dampener_link,
						// rear_caster_dampener) are pivots of a spring-loaded
						// slide — the spring lives on the dampener_rod SLIDE. A
						// hold on the hinge fights the strut geometry: measured
						// 2e6 holds there = the whole chassis bouncing at rest
						// (v_z +5-7 cm/s, pitching 20 deg/s, robot creeping
						// 3-10 cm/s on its casters).
						const bool bPivot = VN.Contains(TEXT("dampener_pivot"))
							|| VN.Contains(TEXT("elevator_dampener"))
							|| VN.Contains(TEXT("rear_caster_dampener"));
						const float ParkStiff = bPivot ? 0.f : Stiff;
						const float ParkDamp = Damp;
						CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
						if (bPivot)
						{
							CI.SetAngularVelocityDriveTwistAndSwing(true, false);
						}
						else
						{
							CI.SetOrientationDriveTwistAndSwing(true, false);
						}
						CI.SetAngularDriveParams(ParkStiff, ParkDamp, 0.f);
						UE_LOG(LogRammsRig, Display,
							TEXT("stand-in spring %s: raw=%.1f -> park hold %.0f/%.0f"),
							*Pair.Key->GetVariableName().ToString(), RawStiff, ParkStiff, ParkDamp);
					}
					else
					{
						CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
						CI.SetOrientationDriveTwistAndSwing(true, false);
						CI.SetAngularDriveParams(Stiff, Damp, 0.f);
					}
					if (Pair.Key->GetVariableName().ToString().StartsWith(TEXT("arm_")))
					{
						UE_LOG(LogRammsRig, Display,
							TEXT("spring drive %s: raw=%.4f scale=%.1f -> stiff=%.0f damp=%.0f"),
							*Pair.Key->GetVariableName().ToString(), RawStiff, SpringMassScale,
							Stiff, Damp);
					}
				}
				// (Damping-only hinges deliberately get NO velocity drive: adding
				// joint-level dampers for the MJCF damping-5 linkage hinges was
				// tried and DESTABILIZED the rear strokes — the extra velocity
				// constraints fight the loop solver. Body-level damping at
				// ApplyChaos covers the windmill problem.)
			}
			else // slide
			{
				CI.SetAngularSwing1Limit(ACM_Locked, 0.f);
				CI.SetAngularSwing2Limit(ACM_Locked, 0.f);
				CI.SetAngularTwistLimit(ACM_Locked, 0.f);
				CI.SetLinearYLimit(LCM_Locked, 0.f);
				CI.SetLinearZLimit(LCM_Locked, 0.f);
				if (bLimited)
				{
					// Slide `range` IS importer-converted to cm (MjJoint tooltip
					// "Slide: centimetres"; verified on the templates: the
					// +-0.08 m rods read +-8.0). Do NOT x100 — that made every
					// rod free-sliding (+-800 cm limit) and the servo, with
					// nothing bounding it, swung the rod's pin 26 deg instead
					// of extending, wedging 6 kN into the chassis (fly-apart).
					CI.SetLinearXLimit(LCM_Limited,
						(RangeSrc->range[1] - RangeSrc->range[0]) * 0.5f);
				}
				else
				{
					CI.SetLinearXLimit(LCM_Free, 0.f);
				}
				// MuJoCo slide stiffness N/m is numerically kg/s^2 = UE force
				// per cm of error; pass through with a stability ceiling only.
				const float RawStiff = (SpringSrc->bOverride_stiffness && SpringSrc->stiffness.Num())
					? SpringSrc->stiffness[0]
					: 0.f;
				if (RawStiff >= 5e4f)
				{
					// Lock stand-in (e.g. dw_main_plate carriage lock): hold the
					// modeled pose rigidly rather than integrating a 1e5 spring.
					CI.SetLinearXLimit(LCM_Locked, 0.f);
				}
				else if (RawStiff > 0.f)
				{
					CI.SetLinearPositionDrive(true, false, false);
					CI.SetLinearDriveParams(FMath::Min(RawStiff, 1e6f),
						FMath::Min(J->damping.Num() ? J->damping[0] : 0.f, 1e5f), 0.f);
				}
			}
			// Unactuated caster (omni) wheels: parked pose rests on them and
			// their contact friction is ~0, so the robot coasts away from any
			// settle impulse. A weak velocity drive at target 0 (0.2 N*m per
			// rad/s) kills the creep but still rolls freely under real force.
			if (J && bHinge
				&& Pair.Key->GetVariableName().ToString().Contains(TEXT("caster_wheel")))
			{
				CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
				CI.SetAngularVelocityDriveTwistAndSwing(true, false);
				// 1 N*m per rad/s: 0.2 was too weak to stop the parked robot
				// coasting on its casters; drive-wheel torque (20 N*m scale)
				// still rolls them easily.
				CI.SetAngularDriveParams(0.f, 1e4f, 0.f);
			}

			// Actuated joint? Configure the constraint drive and record the
			// command route for URammsBackendSwitchComponent::SetJointCommand.
			if (J)
			{
				// Template names carry importer suffixes ("...rod1_GEN_VARIABLE")
				// and MjName is often unset on templates — match the actuator
				// TargetName exactly first, then as name + numeric suffix.
				FString JointMjName = J->GetMjName();
				if (JointMjName.IsEmpty())
				{
					JointMjName = J->GetName(); // MjName is runtime-only
				}
				JointMjName.RemoveFromEnd(TEXT("_GEN_VARIABLE"));
				UMjActuator* const* ActPtr = ActuatorByJoint.Find(JointMjName);
				if (!ActPtr)
				{
					for (const TPair<FString, UMjActuator*>& AP : ActuatorByJoint)
					{
						if (JointMjName.StartsWith(AP.Key)
							&& JointMjName.Mid(AP.Key.Len()).IsNumeric())
						{
							ActPtr = &AP.Value;
							JointMjName = AP.Key;
							break;
						}
					}
				}
				if (ActPtr)
				{
					const UMjActuator* Act = *ActPtr;
					const UMjActuator* Params = ResolveActuatorParams(Act);
					const float		   Kp = Params->gainprm.Num() ? Params->gainprm[0] : 0.f;
					const float		   Kv = Params->biasprm.Num() >= 3 ? -Params->biasprm[2] : 0.f;
					// URLab re-export flattens every actuator to <general>
					// (all import as UMjGeneralActuator) — detect a position
					// servo by its bias term: biasprm = [0, -kp, -kv].
					const bool bPositionServo = Cast<UMjPositionActuator>(Act) != nullptr
						|| (Params->biasprm.Num() >= 2 && Params->biasprm[1] < -KINDA_SMALL_NUMBER);
					if (bSlide)
					{
						// position servo (leadscrew): clamp for ~60 Hz stability
						CI.SetLinearPositionDrive(true, false, false);
						CI.SetLinearVelocityDrive(true, false, false);
						// Keep MuJoCo-scale gains: stronger (x3, x10) drives made
						// the LOADED elevator bind against the linkage limits and
						// explode. Until the limit windows are calibrated to the
						// mechanism's real travel, the elevator stalls under
						// load rather than lifting — stable beats explosive.
						// Bounded force = the real actuator's forcerange (N ->
						// kg*cm/s^2 is x100): an unbounded drive fighting a limit
						// injects energy until the assembly explodes; a bounded
						// one stalls like a real stalled hydraulic rod.
						const float MaxF = Params->forcerange.Num() >= 2
							? FMath::Abs(Params->forcerange[1]) * 100.f
							: 5e5f;
						// x3 was explosive ONLY while the linkage hard-limits
						// were fighting the drive; with those freed it's safe
						// and needed to lift the load through the lever ratio.
						// Force caps, by chain. Elevators: 3e5 (3 kN ~ MuJoCo's
						// measured rod equilibrium) — their load path works and
						// they lift correctly. CASTER rods: 1e5 (1 kN, ~11 N*m
						// at the anchor lever): articulates the arms but cannot
						// jack the chassis — with the strut path leaky, any
						// chassis-lifting force over-rotates the suspension
						// (26 deg vs MuJoCo's 9) and topples or launches the
						// robot (verified at 20/8/3 kN and with a window
						// stand-in, which detonated). Real base-lift via the
						// casters is blocked on the strut/pin-leak work.
						// (Caster rods were briefly capped at 1 kN while the rear
						// strut path was broken; with the virtual strut resisting
						// the suspension they get the working 3 kN back.)
						// Rear caster rod: the mechanism saturates at ~-15 deg
						// suspension travel and the servo then stalls at full
						// force; that reaction (13 kN at the arm hinge, servo
						// slew still piling in) is what flips the robot at
						// +0.06 while the front (no strut, coupler-driven)
						// rides it out. MuJoCo caps this via soft limits; cap
						// the rear rod at 1 kN — enough to articulate (bench:
						// -8 deg at +0.03 with 283 N), not enough to flip.
						const float ChainCap = 3e5f;
						// DAMPING: the MJCF rods use dampratio=1, so biasprm[2]
						// (Kv ~591) is MuJoCo's critical value for ITS reflected
						// inertia and is meaningless here — against kp 6e5 in
						// Chaos it is ratio ~0.003 (undamped). An undamped 6 kN
						// servo overshoots into a hammer blow on the caster
						// contact and pitches the robot at every hold/pin
						// config tested (6.15). Critical for the ~2-6 kg rod
						// end + linkage: c = 2*sqrt(k*m) with m~4 kg ->
						// 2*sqrt(6e5*4) ~ 3e3... but the reflected chassis mass
						// through the lever is ~100x that; use k/10 (6e4) as a
						// safe heavily-damped servo — position tracking stays
						// fine (bench: MuJoCo dampratio=1 lands within 1 mm).
						const float KpC = FMath::Clamp(Kp * 3.f, 1e4f, 6e5f);
						CI.SetLinearDriveParams(KpC, KpC * 0.1f,
							FMath::Clamp(MaxF, 1e4f, ChainCap));
						Switch->DriveJoints.Add(*JointMjName);
						Switch->DriveConstraints.Add(*CName);
						Switch->DriveIsLinear.Add(true);
						Switch->DriveIsPosition.Add(true);
						// Command API is MuJoCo-native METRES; SetJointCommand's
						// linear path converts to cm itself (x100 there — do NOT
						// also scale here). Sign: UE's linear position target
						// moves the child along the constraint frame's -X for a
						// +target here (measured: MuJoCo ctrl +0.06 EXTENDS the
						// rod +2 cm; Chaos +0.06 RETRACTED it — so every Chaos
						// rod test was running the mirror-opposite, loaded
						// stroke). Negate to match MuJoCo ctrl semantics.
						Switch->DriveScale.Add(-1.f);
						{
							float CMin = Params->ctrlrange.Num() >= 2 ? Params->ctrlrange[0] : 0.f;
							float CMax = Params->ctrlrange.Num() >= 2 ? Params->ctrlrange[1] : 0.f;
							// Caster rods: the MJCF ctrlrange (+-8 cm) exceeds what
							// the linkage can physically perform — MuJoCo's own rod
							// stalls at ~2.2 cm (mechanism saturation, joint-space
							// bench). Chaos slews the target all the way out and the
							// stalled servo's reaction (8-13 kN at the suspension
							// arm hinge) flips the robot beyond +0.04. Publish the
							// honest range so panel/teleop/scripts cannot pass it.
							if (JointMjName.Contains(TEXT("front_caster_motor_rod")))
							{
								CMin = FMath::Max(CMin, -0.04f);
								CMax = FMath::Min(CMax, 0.04f);
							}
							else if (JointMjName.Contains(TEXT("rear_caster_motor_rod")))
							{
								// Rear saturates earlier with the RIGID strut (2026-08-16):
								// +0.03 stable (suspension arm -6 deg), +0.04 launches.
								CMin = FMath::Max(CMin, -0.03f);
								CMax = FMath::Min(CMax, 0.03f);
							}
							Switch->DriveCtrlMin.Add(CMin);
							Switch->DriveCtrlMax.Add(CMax);
						}
					}
					else if (bHinge && bPositionServo)
					{
						// MJCF `position` actuator on a hinge (arm servos):
						// orientation-hold PD, or the arm free-falls and flails
						// under Chaos (velocity damping alone holds nothing).
						// N*m/rad -> kg*cm^2/s^2 per rad needs x1e4.
						CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
						CI.SetOrientationDriveTwistAndSwing(true, false);
						CI.SetAngularVelocityDriveTwistAndSwing(true, false);
						CI.SetAngularDriveParams(
							FMath::Clamp(Kp * 1e4f, 1e5f, 5e7f),
							FMath::Clamp(Kv * 1e4f, 1e4f, 5e6f), 0.f);
						Switch->DriveJoints.Add(*JointMjName);
						Switch->DriveConstraints.Add(*CName);
						Switch->DriveIsLinear.Add(false);
						Switch->DriveIsPosition.Add(true);
						Switch->DriveScale.Add(1.f);
						{
							const float CMin = Params->ctrlrange.Num() >= 2 ? Params->ctrlrange[0] : 0.f;
							const float CMax = Params->ctrlrange.Num() >= 2 ? Params->ctrlrange[1] : 0.f;
							Switch->DriveCtrlMin.Add(CMin);
							Switch->DriveCtrlMax.Add(CMax);
						}
					}
					else if (bHinge)
					{
						// torque/velocity motor (wheels): velocity drive
						CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
						// Full-strength velocity drive, always on, target 0:
						// drive wheels are BRAKED at rest — correct for a powered
						// wheelchair (motors hold), and runtime drive-param
						// upgrades do not reach the live solver anyway (verified:
						// commanded wheels crept at idle strength). Casters have
						// no drives and roll freely.
						CI.SetAngularVelocityDriveTwistAndSwing(true, false);
						// 8e5 = 80 N*m per rad/s. History: 2e6 (a HARD brake)
						// turned every elevator/caster stroke into propulsion
						// (the carriage swings the braked wheel along the floor
						// and the wheel's grip pushes the robot); 2e5 stopped
						// that but let the undriven chair roll far too freely
						// when pushed (user feedback 2026-08-18 — a powered
						// wheelchair's motors resist back-driving). 8e5 is the
						// middle: strong rolling resistance at zero command,
						// still yields to deliberate carriage strokes.
						CI.SetAngularDriveParams(0.f, 8e5f, 0.f);
						Switch->DriveJoints.Add(*JointMjName);
						Switch->DriveConstraints.Add(*CName);
						Switch->DriveIsLinear.Add(false);
						Switch->DriveIsPosition.Add(false);
						Switch->DriveScale.Add(1.f);
						{
							const float CMin = Params->ctrlrange.Num() >= 2 ? Params->ctrlrange[0] : 0.f;
							const float CMax = Params->ctrlrange.Num() >= 2 ? Params->ctrlrange[1] : 0.f;
							Switch->DriveCtrlMin.Add(CMin);
							Switch->DriveCtrlMax.Add(CMax);
						}
					}
				}
			}
			Switch->ChaosConstraintComponents.Add(*CName);
			{
				FTransform Local = FTransform::Identity;
				if (J)
				{
					const FVector RaxV = AxisSrc->bOverride_Axis ? AxisSrc->Axis : FVector(0, 0, 1);
					Local = FTransform(
						FRotationMatrix::MakeFromX(RaxV.GetSafeNormal(1e-6f, FVector::ZAxisVector)).ToQuat(),
						J->Pos);
				}
				// body-local -> viz-component-local (the runtime anchor): the
				// viz can be offset from the body node via geom nodes; frames
				// recorded body-local but applied viz-relative shifted every
				// arm/gripper constraint (fingers got yanked into the palm).
				const FTransform VizChain = ChainToAncestor(Body.VizNode, Pair.Key);
				Switch->ConstraintLocalFrames.Add(Local * VizChain.Inverse());
				Switch->ConstraintChildBodies.Add(Body.VizNode->GetVariableName());
				// PREFERRED runtime source (see the switch component): the raw
				// body-local frame + the MjBody component itself. No template
				// chain baked in — the runtime reads the MjBody instance
				// transform, which is authoritative at spawn.
				Switch->ConstraintBodyFrames.Add(Local);
				Switch->ConstraintBodyComponents.Add(Pair.Key->GetVariableName());
				Switch->ConstraintTwistCenters.Add(TwistCenterDeg);
			}
			if (J)
			{
				FString JN = J->GetMjName();
				if (JN.IsEmpty())
				{
					JN = J->GetName();
				}
				JN.RemoveFromEnd(TEXT("_GEN_VARIABLE"));
				ConstraintByJoint.Add(JN, C);
				CNameByJoint.Add(JN, *CName);
			}
			++NumConstraints;
		}

		// VIRTUAL COUPLERS: Chaos's iterative solver leaks force through
		// multi-pin closure loops. The front caster 4-bar (rod -> linkage crank
		// -> linkage_arm/aux pins -> aux_arm pin -> swing arm) transmits ~40%
		// less per pin; the rod articulated the crank 18 deg while the swing
		// arm sat at 0 and the 6 kN reaction flipped the robot. MuJoCo shows a
		// clean linear relation over the working range: swing_arm = 0.667 x
		// linkage (front). Enforce it directly: the follower gets a strong
		// orientation drive whose target the runtime slaves to the leader's
		// twist every tick — same idea as the rear virtual strut, one solve
		// between two bodies instead of a leaky chain.
		{
			struct FCoupler
			{
				const TCHAR* Leader;
				const TCHAR* Follower;
				float		 Ratio;
			};
			const FCoupler Couplers[] = {
				// MuJoCo joint-space ratio is +0.667, but the two Chaos constraint
				// twist frames are opposite-handed (measured: linkage -9 deg drove
				// the swing arm +5 with +ratio) -> negate.
				{ TEXT("front_caster_linkage"), TEXT("front_caster_swing_arm"), -0.667f },
				// Elevator (rigid-strut model): the rod drives the trunnion
				// (motor_elevator) and the carriage swing arm follows it —
				// MuJoCo fit swing = -1.26 x trunnion over the lift range (the
				// pivot stays frozen with a rigid strut). Chaos measured: rod
				// 1.9 kN, trunnion 0.6 deg, force stopped at the rod-link pin.
				// Same handedness convention as the front pair (negate).
				// Leader = the ROD SLIDE itself (leader value = extension in cm
				// from spawn), follower = carriage swing arm. MuJoCo fit:
				// swing = -6.45 rad/m of rod = -3.70 deg/cm. Driving from the
				// rod skips BOTH leaky pins (rod->rod_link->trunnion): the
				// trunnion-leader version reached only 3 deg for -0.03 (MuJoCo
				// -7.5) because the trunnion itself never received the force.
				// Handedness as measured (negate MuJoCo sign).
				// ELEVATOR: leader = rod SLIDE (extension cm), follower = TRUNNION
				// (motor_elevator, NOT the wheel carrier — couplers on the wheel
				// carrier drove the robot off). MuJoCo: trunnion = 4.47 rad/m of
				// rod = 2.56 deg/cm. Chaos measured without it: rod 1.8 kN, trunnion
				// 0.1 deg, force took the easy path — pushed the free-rolling
				// carriage sideways (robot rolled 90 cm) instead of lifting.
				// Enforcing the kinematic path forces the lift. Sign per the
				// front-pair convention (negate MuJoCo).
				{ TEXT("motor_elevator_rod_l"), TEXT("motor_elevator_l"), -2.56f },
				{ TEXT("motor_elevator_rod_r"), TEXT("motor_elevator_r"), -2.56f },
				// Rear: NO coupler with rigid struts (the strut path no longer
				// leaks; the end-coupler slammed the near-saturated swing arm into
				// its stop at 24 kN before). Range clamp +-4 cm handles the rest.
			};
			// Recorded joint keys carry importer suffixes ("front_caster_linkage1")
			auto FindJointKey = [&CNameByJoint](const FString& Base) -> FString {
				if (CNameByJoint.Contains(Base))
				{
					return Base;
				}
				for (const TPair<FString, FName>& KV : CNameByJoint)
				{
					if (KV.Key.StartsWith(Base) && KV.Key.Mid(Base.Len()).IsNumeric())
					{
						return KV.Key;
					}
				}
				return FString();
			};
			for (const FCoupler& Cp : Couplers)
			{
				const FString						LKey = FindJointKey(Cp.Leader);
				const FString						FKey = FindJointKey(Cp.Follower);
				const FName*						LName = LKey.IsEmpty() ? nullptr : CNameByJoint.Find(LKey);
				const FName*						FName_ = FKey.IsEmpty() ? nullptr : CNameByJoint.Find(FKey);
				UPhysicsConstraintComponent* const* FCon = FKey.IsEmpty() ? nullptr : ConstraintByJoint.Find(FKey);
				if (!LName || !FName_ || !FCon)
				{
					UE_LOG(LogRammsRig, Warning, TEXT("coupler %s->%s: joints not found"),
						Cp.Leader, Cp.Follower);
					continue;
				}
				FConstraintInstance& FCI = (*FCon)->ConstraintInstance;
				FCI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
				FCI.SetOrientationDriveTwistAndSwing(true, false);
				// 2e6 = 200 N*m/rad, 2e5 damping: firm enough to carry the swing
				// arm + wheels, target updated at 60 Hz by the runtime.
				FCI.SetAngularDriveParams(2e6f, 2e5f, 0.f);
				Switch->CouplerLeaders.Add(*LName);
				Switch->CouplerFollowers.Add(*FName_);
				Switch->CouplerRatios.Add(Cp.Ratio);
				UE_LOG(LogRammsRig, Display, TEXT("virtual coupler %s -> %s x%.3f"),
					Cp.Leader, Cp.Follower, Cp.Ratio);
			}
		}

		// Tendon actuators (the 2f85 fingers) target a TENDON, not a joint, so
		// the per-joint matching above skips them and the fingers flop. Map the
		// actuator onto every Joint-type wrap of its tendon: orientation-hold
		// drives on the wrapped joints, all registered under ONE drive name so
		// a single command (or panel slider) closes both fingers.
		for (USCS_Node* Node : SCS->GetAllNodes())
		{
			UMjTendon* Tendon = Cast<UMjTendon>(Node->ComponentTemplate);
			if (!Tendon)
			{
				continue;
			}
			FString TendonName = Tendon->GetMjName();
			if (TendonName.IsEmpty())
			{
				TendonName = Node->GetVariableName().ToString();
			}
			TendonName.RemoveFromEnd(TEXT("_GEN_VARIABLE"));
			UMjActuator* const* ActPtr = ActuatorByJoint.Find(TendonName);
			if (!ActPtr)
			{
				continue;
			}
			const UMjActuator* Params = ResolveActuatorParams(*ActPtr);
			const float		   Kp = Params->gainprm.Num() ? Params->gainprm[0] : 0.f;
			const float		   Kv = Params->biasprm.Num() >= 3 ? -Params->biasprm[2] : 0.f;
			int32			   NumWraps = 0;
			for (const FMjTendonWrap& Wrap : Tendon->Wraps)
			{
				if (Wrap.Type != EMjTendonWrapType::Joint)
				{
					continue;
				}
				FString								WrapJoint = Wrap.TargetName;
				UPhysicsConstraintComponent* const* CPtr = ConstraintByJoint.Find(WrapJoint);
				if (!CPtr)
				{
					// try suffix-tolerant lookup
					for (const TPair<FString, UPhysicsConstraintComponent*>& JP : ConstraintByJoint)
					{
						if (JP.Key.StartsWith(WrapJoint) && JP.Key.Mid(WrapJoint.Len()).IsNumeric())
						{
							CPtr = &JP.Value;
							WrapJoint = JP.Key;
							break;
						}
					}
				}
				if (!CPtr)
				{
					UE_LOG(LogRammsRig, Warning, TEXT("tendon %s wrap %s: no constraint"),
						*TendonName, *Wrap.TargetName);
					continue;
				}
				FConstraintInstance& WCI = (*CPtr)->ConstraintInstance;
				WCI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
				WCI.SetOrientationDriveTwistAndSwing(true, false);
				WCI.SetAngularVelocityDriveTwistAndSwing(true, false);
				// Floor at 1e6 (100 N*m/rad). Softer drives (2e5) were tried
				// with the loop fixed and let the four-bar WALK into a
				// contorted rest under the residual pin inconsistency — the
				// strong hold masks it. Known cost: a commanded close
				// over-curls (follower runs to its window edge instead of
				// MuJoCo's -26 deg) because the drive partly tears the leaky
				// pin. Next design: re-derive pin frames from the MjBody
				// component transforms at ApplyChaos init instead of the viz
				// SCS chain (arm viz meshes are offset from body nodes).
				// dt-LIMITED: 1e6 on a 0.15 kg link (I ~ 1.35 kg*cm^2) is a
				// ~137 Hz spring the 60 Hz solver cannot integrate — it aliased
				// into a wild rest oscillation (driver bodies 73 deg/s at rest,
				// shaking the robot into a 3-8 cm/s creep on its casters).
				// Stability needs sqrt(k/I)*dt < ~0.5 -> k <~ 1e4. Heavy
				// damping (5e3, ~20x critical) both kills ringing and resists
				// the four-bar "walk" the 2e5 experiment saw.
				WCI.SetAngularDriveParams(1e4f, 5e3f, 0.f);
				Switch->DriveJoints.Add(*TendonName);
				Switch->DriveConstraints.Add(CNameByJoint.FindRef(WrapJoint));
				Switch->DriveIsLinear.Add(false);
				Switch->DriveIsPosition.Add(true);
				// Wrap coefs are equal (+0.5/+0.5). The old extra sign flip for
				// "_right_" dated from the era of broken pin geometry: with the
				// closures fixed, the mirrored command drove the RIGHT four-bar
				// into its slack direction. Both sides get the SAME sign — and
				// that sign is NEGATIVE: screenshot-verified that a positive
				// twist target physically SPLAYS the fingers open (rest is
				// already at the open stop, so "close" looked dead). Positive
				// command = close, matching MuJoCo's 0..255 ctrl direction.
				Switch->DriveScale.Add(Wrap.Coef >= 0.f ? -1.f : 1.f);
				{
					// Panel range: the WRAPPED JOINT's travel in radians. The
					// actuator ctrlrange is tendon-length units (0..255 on the
					// 2f85) — a slider there commands hundreds of radians and
					// "does nothing" through the slew limiter.
					const UMjJoint* WrapJ = nullptr;
					if (UPhysicsConstraintComponent* const* WC = ConstraintByJoint.Find(WrapJoint))
					{
						(void)WC;
					}
					float LoDeg = 0.f, HiDeg = 46.f;
					for (const TPair<USCS_Node*, FRigBody>& BP2 : Bodies)
					{
						if (BP2.Value.Joint)
						{
							FString JN2 = BP2.Value.Joint->GetMjName();
							if (JN2.IsEmpty())
							{
								JN2 = BP2.Value.Joint->GetName();
							}
							JN2.RemoveFromEnd(TEXT("_GEN_VARIABLE"));
							if (JN2 == WrapJoint)
							{
								WrapJ = BP2.Value.Joint;
								break;
							}
						}
					}
					if (WrapJ)
					{
						const UMjJoint* RS = ResolveJointTemplate(WrapJ,
							[](const UMjJoint* X) { return X->bOverride_range && X->range.Num() >= 2; });
						if (RS->bOverride_range && RS->range.Num() >= 2)
						{
							LoDeg = RS->range[0];
							HiDeg = RS->range[1];
						}
					}
					Switch->DriveCtrlMin.Add(FMath::DegreesToRadians(LoDeg));
					Switch->DriveCtrlMax.Add(FMath::DegreesToRadians(HiDeg));
				}
				++NumWraps;
			}
			if (NumWraps)
			{
				UE_LOG(LogRammsRig, Display, TEXT("tendon actuator %s -> %d joint drives"),
					*TendonName, NumWraps);
			}
		}

		// Loop closures: connect equalities become point constraints.
		for (UMjEquality* Eq : Equalities)
		{
			if (Eq->EqualityType != EMjEqualityType::Connect || Eq->anchor.Num() < 3)
			{
				continue;
			}
			USCS_Node*		B1 = BodyByName.FindRef(Eq->Obj1);
			USCS_Node*		B2 = BodyByName.FindRef(Eq->Obj2);
			const FRigBody* R1 = B1 ? Bodies.Find(B1) : nullptr;
			const FRigBody* R2 = B2 ? Bodies.Find(B2) : nullptr;
			if (!R1 || !R2 || !R1->VizNode || !R2->VizNode)
			{
				UE_LOG(LogRammsRig, Warning, TEXT("closure %s<->%s: bodies unresolved — skipped"),
					*Eq->Obj1, *Eq->Obj2);
				continue;
			}
			const FString CName = FString::Printf(TEXT("ChaosRig_pin_%s__%s"), *Eq->Obj1, *Eq->Obj2);
			USCS_Node*	  CNode = SCS->CreateNode(UPhysicsConstraintComponent::StaticClass(), *CName);
			B1->AddChildNode(CNode);
			UPhysicsConstraintComponent* C =
				Cast<UPhysicsConstraintComponent>(CNode->ComponentTemplate);
			// MJCF `connect` is a BALL — but the real hardware closures are
			// CLEVIS PINS (one axis). A two-ball link can spin freely about its
			// anchor axis (user-observed under-constrained closure DOF). Orient
			// the pin along the link's own hinge axis and leave only twist
			// free; swings get a 5 deg tolerance window instead of a hard lock
			// so slightly imperfect closure geometry can't fight the loop.
			// Pin twist axis = the mechanism's HINGE axis. Taking R1's joint
			// axis blindly aligned rod pins with the rod's SLIDE direction
			// (in-plane), so all relative rotation landed in the swing axes and
			// the pins bound at any window (rear chain stopped at exactly the
			// window angle). Prefer a HINGE axis from either side.
			FVector PinAxis(0.f, -1.f, 0.f);
			auto	HingeAxisOf = [&ResolveJointTemplate](const FRigBody* R, FVector& Out) -> bool {
				   if (!R->Joint || !Cast<UMjHingeJoint>(R->Joint))
				   {
					   return false;
				   }
				   const UMjJoint* AS = ResolveJointTemplate(R->Joint,
					   [](const UMjJoint* X) { return X->bOverride_Axis; });
				   Out = (AS->bOverride_Axis ? AS->Axis : FVector(0, 0, 1))
							 .GetSafeNormal(1e-6f, FVector(0, -1, 0));
				   return true;
			};
			if (!HingeAxisOf(R1, PinAxis))
			{
				HingeAxisOf(R2, PinAxis);
			}
			// Joint Axis is codegen-emitted with y_negate already applied
			// (MjJoint.h: rules.joint.vec3_convert.axis) — use it as-is. An
			// earlier extra Y-negation here was a no-op for the current
			// content (all loop-pin axes are pure Y) but wrong in general.
			// Equality anchors ARE importer-converted to cm (MjEquality.anchor
			// carries the MjUnit="cm" codegen annotation — verified template
			// 7.287 vs XML 0.07287 m), unlike geom sizes/slide ranges which
			// stay native metres. Only the HANDEDNESS is missing: negate Y for
			// the UE frame. History: reading these as if they needed x100 gave
			// 20 m pin offsets (spawn explosion); zeroing them parked every pin
			// at the body origin — zero lever arm, loops transmitted no torque,
			// rod force became rigid-body shoving and the freed linkage pieces
			// windmilled (user reports).
			const FVector AnchorCm(Eq->anchor[0], -Eq->anchor[1], Eq->anchor[2]);
			C->SetRelativeLocation(AnchorCm);
			C->SetRelativeRotation(FRotationMatrix::MakeFromX(PinAxis).Rotator());
			C->ComponentName1.ComponentName = R1->VizNode->GetVariableName();
			C->ComponentName2.ComponentName = R2->VizNode->GetVariableName();
			FConstraintInstance& CI = C->ConstraintInstance;
			CI.ProfileInstance.bDisableCollision = true;
			// NO projection on pins: MuJoCo closures are SOFT (solref), but a
			// hard-locked pin network over an imported linkage with mm-scale
			// closure error is over-constrained — projection then teleports
			// bodies every step and PUMPS energy (the spawn pre-stress drift
			// that killed the first x100 attempt). Soft linear limits emulate
			// solref compliance and let the residual geometric error live in
			// the spring instead of the solver fight.
			// NO projection on ANY pin (2026-08-18). The gripper-pin projection
			// dated from the broken-frame era: the real bug was
			// UpdateConstraintFrames dividing frame positions by the constraint
			// component's inherited scale (the gripper chain carries imported-
			// asset compensating scales), which parked the coupler-side frame
			// at the coupler ORIGIN — the pin never closed, and projection was
			// papering over it. With runtime scale-1 normalization the pins
			// initialize exact (sep 0.000), and measured WITH projection the
			// solver leaves the locked pin torn 4.79 cm at rest while WITHOUT
			// it the pin holds 0.00-0.03 cm through free-flop and drive load.
			CI.ProfileInstance.bEnableProjection = false;
			// Pin linear constraint, two regimes:
			// - arm_ (gripper) pins: HARD lock + projection. Free-floating
			//   four-bars, verified placement, works.
			// - mebot loop pins: SOFT-FROM-ZERO spring (LCM_Limited with
			//   radius 0 — NO free play, unlike the earlier 0.2 cm mistake
			//   whose slack hid the fold mode). A HARD-locked pin that the
			//   iterative solver cannot converge (the strut path) stays
			//   permanently violated and pumps energy every frame — measured:
			//   the rear rod toppled/launched the robot at 20/8/3/1 kN force
			//   caps alike, with the aux pin torn 15+ deg while the pivot sat
			//   still. A stiff spring (1e6 = 10 kN/cm, damping 5e4) transmits
			//   the ~10-100 N strut-path forces faithfully but YIELDS
			//   gracefully where the solver would otherwise fight — MuJoCo's
			//   own connect equalities are exactly this (solref soft).
			if (Eq->Obj1.StartsWith(TEXT("arm_")))
			{
				CI.SetLinearXLimit(LCM_Locked, 0.f);
				CI.SetLinearYLimit(LCM_Locked, 0.f);
				CI.SetLinearZLimit(LCM_Locked, 0.f);
			}
			else
			{
				// Also effectively hard: a "soft-from-zero" LCM_Limited(0)
				// was tried with stiffness 1e6 AND 1e9 — behavior identical
				// to the decimal, so Chaos treats a radius-0 limited linear
				// constraint as LOCKED and ignores the soft params. Keep the
				// honest form.
				CI.SetLinearXLimit(LCM_Locked, 0.f);
				CI.SetLinearYLimit(LCM_Locked, 0.f);
				CI.SetLinearZLimit(LCM_Locked, 0.f);
			}
			// 30 deg swing windows still kill the free-spin DOF the ball
			// closures had while letting the linkage travel; SOFT so window
			// contact during a driven sweep can't detonate the loop.
			// BALL closures (all angular free), exactly the MJCF `connect`.
			// The clevis-pin swing windows (30 deg) were a bind: the front
			// caster's retract stroke jammed at -1.5 cm, its pin twisted to
			// -26 deg and the linkage hinge took 5-11 kN until the robot
			// flipped — while MuJoCo (ball closures) drives the same stroke
			// to -0.25 rad swing-arm travel. Whatever spurious spin a ball
			// pin allows is far cheaper than a wrongly-oriented pin axis
			// fighting the loop. (Per-body hinge locks already remove the
			// links' own off-axis DOF.)
			CI.SetAngularSwing1Limit(ACM_Free, 0.f);
			CI.SetAngularSwing2Limit(ACM_Free, 0.f);
			CI.SetAngularTwistLimit(ACM_Free, 0.f);
			Switch->ChaosConstraintComponents.Add(*CName);
			Switch->ConstraintLocalFrames.Add(
				FTransform(FRotationMatrix::MakeFromX(PinAxis).ToQuat(), AnchorCm)
				* ChainToAncestor(R1->VizNode, B1).Inverse());
			Switch->ConstraintChildBodies.Add(R1->VizNode->GetVariableName());
			Switch->ConstraintBodyFrames.Add(
				FTransform(FRotationMatrix::MakeFromX(PinAxis).ToQuat(), AnchorCm));
			Switch->ConstraintBodyComponents.Add(B1->GetVariableName());
			Switch->ConstraintTwistCenters.Add(0.f);
			++NumClosures;

			// VIRTUAL STRUT (rear caster): the physical strut is a 3-body
			// chain (pivot -> dampener hinge -> dampener_rod slide-spring ->
			// pin -> swing arm) that Chaos's iterative solver cannot push
			// force through — the suspension over-rotates (26 deg vs
			// MuJoCo's 9), any rod force that can articulate also lifts the
			// unresisted rear, and the robot topples (verified at 20/8/3/1
			// kN caps). Add ONE direct axial spring between the strut's END
			// bodies — pivot and swing arm — at the strut's line: Chaos
			// solves a single constraint between two 3-4 kg bodies with no
			// chain to leak through. Stiffness = the MuJoCo strut spring
			// (20000 N/m == 200 N/cm == 2e4 units/cm), damping 500. The
			// elevator struts keep their (working) body chains — a virtual
			// strut there would double-count the spring.
			// Virtual strut is OBSOLETE with rigid dampeners (2026-08-16: the
			// dampener_rod slides are removed at compose time, the rod is a
			// welded child of the dampener — a real rigid strut). Kept as
			// dead code for reference; never taken.
			if (false && Eq->Obj1 == TEXT("rear_caster_dampener_rod"))
			{
				const FRigBody* RodRig = R1;				   // dampener_rod
				USCS_Node*		DampNode = RodRig->ParentBody; // dampener
				const FRigBody* DampRig = DampNode ? Bodies.Find(DampNode) : nullptr;
				USCS_Node*		BaseNode = DampRig ? DampRig->ParentBody : nullptr; // pivot
				const FRigBody* BaseRig = BaseNode ? Bodies.Find(BaseNode) : nullptr;
				if (BaseRig && BaseRig->VizNode)
				{
					const FString SName = TEXT("ChaosRig_strut_rear_caster");
					USCS_Node*	  SNode = SCS->CreateNode(
						   UPhysicsConstraintComponent::StaticClass(), *SName);
					B1->AddChildNode(SNode);
					UPhysicsConstraintComponent* SC2 =
						Cast<UPhysicsConstraintComponent>(SNode->ComponentTemplate);
					// Actor-space strut line: base (pivot origin) -> anchor.
					const FTransform B1Actor = ChainToAncestor(B1, nullptr);
					const FTransform BaseActor = ChainToAncestor(BaseNode, nullptr);
					const FVector	 AnchorActor = B1Actor.TransformPosition(AnchorCm);
					const FVector	 Axis =
						(AnchorActor - BaseActor.GetLocation()).GetSafeNormal(1e-4f, FVector::ZAxisVector);
					const FQuat RelRot = B1Actor.GetRotation().Inverse()
						* FRotationMatrix::MakeFromX(Axis).ToQuat();
					SC2->SetRelativeLocation(AnchorCm);
					SC2->SetRelativeRotation(RelRot.Rotator());
					SC2->ComponentName1.ComponentName = BaseRig->VizNode->GetVariableName();
					SC2->ComponentName2.ComponentName = R2->VizNode->GetVariableName();
					FConstraintInstance& SCI = SC2->ConstraintInstance;
					SCI.ProfileInstance.bDisableCollision = true;
					SCI.ProfileInstance.bEnableProjection = false;
					SCI.SetLinearXLimit(LCM_Free, 0.f);
					SCI.SetLinearYLimit(LCM_Free, 0.f);
					SCI.SetLinearZLimit(LCM_Free, 0.f);
					SCI.SetAngularSwing1Limit(ACM_Free, 0.f);
					SCI.SetAngularSwing2Limit(ACM_Free, 0.f);
					SCI.SetAngularTwistLimit(ACM_Free, 0.f);
					SCI.SetLinearPositionDrive(true, false, false);
					SCI.SetLinearVelocityDrive(true, false, false);
					SCI.SetLinearDriveParams(2e4f, 500.f, 0.f);
					Switch->ChaosConstraintComponents.Add(*SName);
					Switch->ConstraintLocalFrames.Add(
						FTransform(RelRot, AnchorCm)
						* ChainToAncestor(R1->VizNode, B1).Inverse());
					Switch->ConstraintChildBodies.Add(R1->VizNode->GetVariableName());
					Switch->ConstraintBodyFrames.Add(FTransform(RelRot, AnchorCm));
					Switch->ConstraintBodyComponents.Add(B1->GetVariableName());
					Switch->ConstraintTwistCenters.Add(0.f);
					UE_LOG(LogRammsRig, Display,
						TEXT("virtual strut %s: base=%s arm=%s axis=%s"),
						*SName, *BaseRig->VizNode->GetVariableName().ToString(),
						*R2->VizNode->GetVariableName().ToString(), *Axis.ToString());
				}
			}
		}

		FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BP);
		FKismetEditorUtilities::CompileBlueprint(BP);
		BP->MarkPackageDirty();
		for (const TCHAR* PMPath : { TEXT("/Game/MuJoCoImports/PM_RammsWheel.PM_RammsWheel"),
				 TEXT("/Game/MuJoCoImports/PM_RammsCaster.PM_RammsCaster"),
				 TEXT("/Game/MuJoCoImports/PM_RammsChassis.PM_RammsChassis") })
		{
			if (UPhysicalMaterial* PM = LoadObject<UPhysicalMaterial>(nullptr, PMPath))
			{
				ModifiedMeshPackages.Add(PM->GetOutermost());
			}
		}
		if (ModifiedMeshPackages.Num())
		{
			UEditorLoadingAndSavingUtils::SavePackages(ModifiedMeshPackages.Array(), true);
			UE_LOG(LogRammsRig, Display, TEXT("saved %d modified mesh packages"),
				ModifiedMeshPackages.Num());
		}
		UE_LOG(LogRammsRig, Display,
			TEXT("Chaos rig generated for %s: %d bodies, %d joint constraints, %d closures"),
			*BP->GetName(), NumBodies, NumConstraints, NumClosures);
	}

	static void GenerateForSelectedBlueprints()
	{
		FContentBrowserModule& CB =
			FModuleManager::LoadModuleChecked<FContentBrowserModule>("ContentBrowser");
		TArray<FAssetData> Selected;
		CB.Get().GetSelectedAssets(Selected);
		int32 N = 0;
		for (const FAssetData& Asset : Selected)
		{
			if (UBlueprint* BP = Cast<UBlueprint>(Asset.GetAsset()))
			{
				GenerateChaosRig(BP);
				++N;
			}
		}
		if (N == 0)
		{
			UE_LOG(LogRammsRig, Warning,
				TEXT("Generate Chaos Rig: select an imported MJCF Blueprint in the Content Browser"));
		}
	}

} // namespace

namespace RammsRigGen
{
	void GenerateChaosRigForBlueprint(UBlueprint* BP)
	{
		if (BP)
		{
			GenerateChaosRig(BP);
		}
	}
} // namespace RammsRigGen

class FRammsMujocoSupportEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		UToolMenus::RegisterStartupCallback(FSimpleMulticastDelegate::FDelegate::CreateLambda([]() {
			UToolMenu*		  Menu = UToolMenus::Get()->ExtendMenu("LevelEditor.MainMenu.Tools");
			FToolMenuSection& Section = Menu->FindOrAddSection("RammsRobots",
				NSLOCTEXT("RammsRig", "Section", "RAMMS Robots"));
			Section.AddMenuEntry("GenerateChaosRig",
				NSLOCTEXT("RammsRig", "GenRig", "Generate Chaos Rig (selected BP)"),
				NSLOCTEXT("RammsRig", "GenRigTip",
					"Generate Chaos collision + constraints + backend switch from the "
					"Mj components of the selected imported MJCF Blueprint"),
				FSlateIcon(),
				FUIAction(FExecuteAction::CreateStatic(&GenerateForSelectedBlueprints)));
		}));
	}

	virtual void ShutdownModule() override
	{
		UToolMenus::UnregisterOwner(this);
	}
};

IMPLEMENT_MODULE(FRammsMujocoSupportEditorModule, RammsMujocoSupportEditor)
