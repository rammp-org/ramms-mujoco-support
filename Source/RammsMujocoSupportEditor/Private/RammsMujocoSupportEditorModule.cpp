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
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "RammsBackendSwitchComponent.h"
#include "ToolMenus.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsRig, Log, All);

namespace
{

struct FRigBody
{
	USCS_Node* Node = nullptr;
	USCS_Node* ParentBody = nullptr;   // nearest MjBody ancestor node
	USCS_Node* VizNode = nullptr;      // first StaticMeshComponent child
	TArray<USCS_Node*> ExtraViz;       // sibling visual pieces (multi-material)
	UMjJoint* Joint = nullptr;         // first joint template under the body
	UMjInertial* Inertial = nullptr;   // MJCF inertial (mass in kg), if any
	TArray<UMjGeom*> Geoms;
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
	TArray<USCS_Node*> All = SCS->GetAllNodes();
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
			USCS_Node* Child = Stack.Pop();
			UActorComponent* Tmpl = Child->ComponentTemplate;
			if (Cast<UMjBody>(Tmpl))
			{
				continue;   // nested body: its own subtree
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

static bool AddCollisionFromGeoms(const FRigBody& Body, TSet<UPackage*>& OutModifiedPackages)
{
	UStaticMeshComponent* Viz = Cast<UStaticMeshComponent>(Body.VizNode->ComponentTemplate);
	UStaticMesh* Mesh = Viz ? Viz->GetStaticMesh() : nullptr;
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
		constexpr float M2Cm = 100.f;   // codegen `size` array is metres
		TArray<float> Sz = Geom->size;
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
			// MuJoCo cylinder/capsule axis = local Z; FKSphylElem axis = Z too.
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
		const FBox Bounds = Mesh->GetBoundingBox();
		const FVector Ext = Bounds.GetExtent();
		FKBoxElem Box(Ext.X * 2.f, Ext.Y * 2.f, Ext.Z * 2.f);
		Box.Center = Bounds.GetCenter();
		Setup->AggGeom.BoxElems.Add(Box);
		++Added;
	}
	Setup->CollisionTraceFlag = CTF_UseDefault;
	Setup->InvalidatePhysicsData();
	Setup->CreatePhysicsMeshes();
	Mesh->MarkPackageDirty();
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
	TArray<USCS_Node*> Stale;
	for (USCS_Node* Node : SCS->GetAllNodes())
	{
		if (Node->GetVariableName().ToString().StartsWith(TEXT("ChaosRig_")))
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

	TMap<USCS_Node*, FRigBody> Bodies;
	TArray<UMjEquality*> Equalities;
	TMap<USCS_Node*, USCS_Node*> ParentOf;
	CollectRig(BP, Bodies, Equalities, ParentOf);

	TMap<FString, UMjActuator*> ActuatorByJoint;
	TMap<FString, USCS_Node*> DefaultNodeByClass;   // MJCF default class -> node
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

	TMap<FString, USCS_Node*> BodyByName;   // Mj element name -> node
	for (TPair<USCS_Node*, FRigBody>& Pair : Bodies)
	{
		if (const UMjBody* BodyTmpl = Cast<UMjBody>(Pair.Key->ComponentTemplate))
		{
			BodyByName.Add(BodyTmpl->GetMjName(), Pair.Key);
		}
		BodyByName.Add(Pair.Key->GetVariableName().ToString(), Pair.Key);
	}

	URammsBackendSwitchComponent* Switch = nullptr;
	USCS_Node* SwitchNode = SCS->CreateNode(URammsBackendSwitchComponent::StaticClass(),
		TEXT("ChaosRig_BackendSwitch"));
	SCS->AddNode(SwitchNode);
	Switch = Cast<URammsBackendSwitchComponent>(SwitchNode->ComponentTemplate);

	int32 NumBodies = 0, NumConstraints = 0, NumClosures = 0;
	TSet<UPackage*> ModifiedMeshPackages;

	for (TPair<USCS_Node*, FRigBody>& Pair : Bodies)
	{
		FRigBody& Body = Pair.Value;
		if (!Body.VizNode)
		{
			UE_LOG(LogRammsRig, Warning, TEXT("body %s has no visual mesh — skipped"),
				*Pair.Key->GetVariableName().ToString());
			continue;
		}
		if (AddCollisionFromGeoms(Body, ModifiedMeshPackages))
		{
			Switch->ChaosBodyComponents.Add(Body.VizNode->GetVariableName());
			// MJCF inertial mass if authored (gripper links are 12-22 g —
			// tiny but REAL, never "sanity"-replace them); else MuJoCo's
			// convention of geom volume x 1000 kg/m^3 (sizes are cm).
			const bool bHasInertial = Body.Inertial && Body.Inertial->mass > 0.f;
			float MassKg = bHasInertial ? Body.Inertial->mass : 0.f;
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
						V *= 100.f;   // metres -> cm (see NOTE above)
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
						|| Geom->Type == EMjGeomType::Capsule) && Sz.Num() >= 2)
					{
						VolCm3 += PI * Sz[0] * Sz[0] * (2.f * Sz[1]);
					}
				}
				MassKg = VolCm3 / 1000.f;   // x1000 kg/m^3, /1e6 cm^3->m^3
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
			continue;   // ball joints TODO
		}

		const FString CName = FString::Printf(TEXT("ChaosRig_%s"),
			*Pair.Key->GetVariableName().ToString());
		USCS_Node* CNode = SCS->CreateNode(UPhysicsConstraintComponent::StaticClass(), *CName);
		Pair.Key->AddChildNode(CNode);
		UPhysicsConstraintComponent* C =
			Cast<UPhysicsConstraintComponent>(CNode->ComponentTemplate);

		if (J)
		{
			C->SetRelativeLocation(J->Pos);
			const FVector Axis = J->Axis.GetSafeNormal(1e-6f, FVector::ZAxisVector);
			C->SetRelativeRotation(FRotationMatrix::MakeFromX(Axis).Rotator());
		}
		C->ComponentName1.ComponentName = ParentRig->VizNode->GetVariableName();
		C->ComponentName2.ComponentName = Body.VizNode->GetVariableName();

		FConstraintInstance& CI = C->ConstraintInstance;
		CI.ProfileInstance.bDisableCollision = true;
		const bool bLimited = J && J->range.Num() == 2 && J->range[1] > J->range[0];
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
			if (bLimited)
			{
				const float Half = (J->range[1] - J->range[0]) * 0.5f;
				const float Center = (J->range[1] + J->range[0]) * 0.5f;
				CI.SetAngularTwistLimit(ACM_Limited, Half);
				// Centre the symmetric twist window on the modelled pose
				// (which sits at joint value `ref`).
				CI.AngularRotationOffset.Roll = Center - J->ref;
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
			const float RawStiff = J->stiffness.Num() ? J->stiffness[0] : 0.f;
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
				const float Stiff = FMath::Min(RawStiff * 1e4f, 2e6f);
				const float Damp = FMath::Min((J->damping.Num() ? J->damping[0] : 0.f) * 1e4f, 2e5f);
				CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
				CI.SetOrientationDriveTwistAndSwing(true, false);
				CI.SetAngularDriveParams(Stiff, Damp, 0.f);
			}
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
				CI.SetLinearXLimit(LCM_Limited, (J->range[1] - J->range[0]) * 0.5f);
			}
			else
			{
				CI.SetLinearXLimit(LCM_Free, 0.f);
			}
			// MuJoCo slide stiffness N/m is numerically kg/s^2 = UE force
			// per cm of error; pass through with a stability ceiling only.
			const float RawStiff = J->stiffness.Num() ? J->stiffness[0] : 0.f;
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
				JointMjName = J->GetName();   // MjName is runtime-only
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
				const float Kp = Params->gainprm.Num() ? Params->gainprm[0] : 0.f;
				const float Kv = Params->biasprm.Num() >= 3 ? -Params->biasprm[2] : 0.f;
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
					CI.SetLinearDriveParams(FMath::Clamp(Kp, 1000.f, 1e6f),
						FMath::Clamp(Kv, 100.f, 1e5f), 0.f);
					Switch->DriveJoints.Add(*JointMjName);
					Switch->DriveConstraints.Add(*CName);
					Switch->DriveIsLinear.Add(true);
					Switch->DriveIsPosition.Add(true);
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
				}
				else if (bHinge)
				{
					// torque/velocity motor (wheels): velocity drive
					CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
					CI.SetAngularVelocityDriveTwistAndSwing(true, false);
					// velocity-drive "damping" is torque per rad/s of
					// error in kg*cm^2/s^2: 2e5 ~= 20 N*m per rad/s.
					CI.SetAngularDriveParams(0.f, 2e5f, 0.f);
					Switch->DriveJoints.Add(*JointMjName);
					Switch->DriveConstraints.Add(*CName);
					Switch->DriveIsLinear.Add(false);
					Switch->DriveIsPosition.Add(false);
				}
			}
		}
		Switch->ChaosConstraintComponents.Add(*CName);
		++NumConstraints;
	}

	// Loop closures: connect equalities become point constraints.
	for (UMjEquality* Eq : Equalities)
	{
		if (Eq->EqualityType != EMjEqualityType::Connect || Eq->anchor.Num() < 3)
		{
			continue;
		}
		USCS_Node* B1 = BodyByName.FindRef(Eq->Obj1);
		USCS_Node* B2 = BodyByName.FindRef(Eq->Obj2);
		const FRigBody* R1 = B1 ? Bodies.Find(B1) : nullptr;
		const FRigBody* R2 = B2 ? Bodies.Find(B2) : nullptr;
		if (!R1 || !R2 || !R1->VizNode || !R2->VizNode)
		{
			UE_LOG(LogRammsRig, Warning, TEXT("closure %s<->%s: bodies unresolved — skipped"),
				*Eq->Obj1, *Eq->Obj2);
			continue;
		}
		const FString CName = FString::Printf(TEXT("ChaosRig_pin_%s__%s"), *Eq->Obj1, *Eq->Obj2);
		USCS_Node* CNode = SCS->CreateNode(UPhysicsConstraintComponent::StaticClass(), *CName);
		B1->AddChildNode(CNode);
		UPhysicsConstraintComponent* C =
			Cast<UPhysicsConstraintComponent>(CNode->ComponentTemplate);
		C->SetRelativeLocation(FVector(Eq->anchor[0], Eq->anchor[1], Eq->anchor[2]));
		C->ComponentName1.ComponentName = R1->VizNode->GetVariableName();
		C->ComponentName2.ComponentName = R2->VizNode->GetVariableName();
		FConstraintInstance& CI = C->ConstraintInstance;
		CI.ProfileInstance.bDisableCollision = true;
		CI.SetLinearXLimit(LCM_Locked, 0.f);
		CI.SetLinearYLimit(LCM_Locked, 0.f);
		CI.SetLinearZLimit(LCM_Locked, 0.f);
		CI.SetAngularSwing1Limit(ACM_Free, 0.f);
		CI.SetAngularSwing2Limit(ACM_Free, 0.f);
		CI.SetAngularTwistLimit(ACM_Free, 0.f);
		Switch->ChaosConstraintComponents.Add(*CName);
		++NumClosures;
	}

	FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BP);
	FKismetEditorUtilities::CompileBlueprint(BP);
	BP->MarkPackageDirty();
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
}

class FRammsMujocoSupportEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		UToolMenus::RegisterStartupCallback(FSimpleMulticastDelegate::FDelegate::CreateLambda([]() {
			UToolMenu* Menu = UToolMenus::Get()->ExtendMenu("LevelEditor.MainMenu.Tools");
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
