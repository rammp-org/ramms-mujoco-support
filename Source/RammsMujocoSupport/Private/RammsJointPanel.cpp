// Copyright Epic Games, Inc. All Rights Reserved.
//
// Ramms.Panel — backend-agnostic joint control panel.
//
// `Ramms.Panel <ActorLabel>` opens a Slate window with one slider per
// actuated joint (DriveJoints on the actor's URammsBackendSwitchComponent).
// Slider moves route through SetJointCommand, so the SAME panel drives the
// robot under MuJoCo, Newton, or Chaos. Ranges come from the recorded MJCF
// ctrlrange; wheels (velocity motors, ctrlrange [-1,1] or unknown) get a
// +-6 rad/s span instead.

#include "Components/StaticMeshComponent.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "RammsBackendSwitchComponent.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "EngineUtils.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "GameFramework/Actor.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SSlider.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SWindow.h"
#include "Widgets/Text/STextBlock.h"

namespace
{

	// Focusable wrapper adding keyboard teleop while the panel has focus:
	//   W/S forward/back   A/D turn      (wheels, +-3 rad/s)
	//   Q/E elevator rods  R/F front caster rod  T/G rear caster rod
	//   Space = stop wheels
	class SRammsTeleop : public SCompoundWidget
	{
	public:
		SLATE_BEGIN_ARGS(SRammsTeleop) {}
		SLATE_DEFAULT_SLOT(FArguments, Content)
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs, TWeakObjectPtr<URammsBackendSwitchComponent> InSw)
		{
			Sw = InSw;
			ChildSlot[InArgs._Content.Widget];
			RegisterActiveTimer(0.05f,
				FWidgetActiveTimerDelegate::CreateSP(this, &SRammsTeleop::Drive));
		}

		virtual bool SupportsKeyboardFocus() const override { return true; }

		virtual FReply OnKeyDown(const FGeometry&, const FKeyEvent& E) override
		{
			Pressed.Add(E.GetKey());
			return Handled(E.GetKey());
		}

		virtual FReply OnKeyUp(const FGeometry&, const FKeyEvent& E) override
		{
			Pressed.Remove(E.GetKey());
			return Handled(E.GetKey());
		}

	private:
		static FReply Handled(const FKey& K)
		{
			static const FKey Keys[] = { EKeys::W, EKeys::S, EKeys::A, EKeys::D,
				EKeys::Q, EKeys::E, EKeys::R, EKeys::F, EKeys::T, EKeys::G,
				EKeys::SpaceBar };
			for (const FKey& X : Keys)
			{
				if (K == X)
				{
					return FReply::Handled();
				}
			}
			return FReply::Unhandled();
		}

		EActiveTimerReturnType Drive(double, float Dt)
		{
			URammsBackendSwitchComponent* S = Sw.Get();
			if (!S)
			{
				return EActiveTimerReturnType::Stop;
			}
			const float V = 3.f; // rad/s
			float		L = 0.f, R = 0.f;
			bool		bWheelKey = false;
			if (Pressed.Contains(EKeys::W))
			{
				L += V;
				R += V;
				bWheelKey = true;
			}
			if (Pressed.Contains(EKeys::S))
			{
				L -= V;
				R -= V;
				bWheelKey = true;
			}
			if (Pressed.Contains(EKeys::A))
			{
				L -= V;
				R += V;
				bWheelKey = true;
			}
			if (Pressed.Contains(EKeys::D))
			{
				L += V;
				R -= V;
				bWheelKey = true;
			}
			if (Pressed.Contains(EKeys::SpaceBar))
			{
				L = R = 0.f;
				bWheelKey = true;
			}
			if (bWheelKey || bWheelsWereDriven)
			{
				S->SetJointCommand(TEXT("drive_wheel_l"), L);
				S->SetJointCommand(TEXT("drive_wheel_r"), R);
				bWheelsWereDriven = bWheelKey && (L != 0.f || R != 0.f);
			}
			auto Rod = [&](const FKey& Up, const FKey& Dn, float& Target, const TCHAR* JL, const TCHAR* JR) {
				const float Rate = 0.04f; // m/s of target motion
				float		Dir = 0.f;
				if (Pressed.Contains(Up))
				{
					Dir -= 1.f;
				}
				if (Pressed.Contains(Dn))
				{
					Dir += 1.f;
				}
				if (Dir != 0.f)
				{
					Target = FMath::Clamp(Target + Dir * Rate * Dt, -0.08f, 0.08f);
					S->SetJointCommand(JL, Target);
					if (JR)
					{
						S->SetJointCommand(JR, Target);
					}
				}
			};
			Rod(EKeys::Q, EKeys::E, Elevator, TEXT("motor_elevator_rod_l"), TEXT("motor_elevator_rod_r"));
			Rod(EKeys::R, EKeys::F, FrontRod, TEXT("front_caster_motor_rod"), nullptr);
			Rod(EKeys::T, EKeys::G, RearRod, TEXT("rear_caster_motor_rod"), nullptr);
			return EActiveTimerReturnType::Continue;
		}

		TWeakObjectPtr<URammsBackendSwitchComponent> Sw;
		TSet<FKey>									 Pressed;
		bool										 bWheelsWereDriven = false;
		float										 Elevator = 0.f;
		float										 FrontRod = 0.f;
		float										 RearRod = 0.f;
	};

	void OpenJointPanel(const TArray<FString>& Args, UWorld* World)
	{
		if (!World)
		{
			return;
		}
		const FString				  Label = Args.Num() > 0 ? Args[0] : TEXT("");
		URammsBackendSwitchComponent* Found = nullptr;
		AActor*						  FoundActor = nullptr;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			if (!Label.IsEmpty() && It->GetActorNameOrLabel() != Label && It->GetName() != Label)
			{
				continue;
			}
			if (URammsBackendSwitchComponent* Sw =
					It->FindComponentByClass<URammsBackendSwitchComponent>())
			{
				Found = Sw;
				FoundActor = *It;
				break;
			}
		}
		if (!Found)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("Ramms.Panel: no actor%s%s with a URammsBackendSwitchComponent found"),
				Label.IsEmpty() ? TEXT("") : TEXT(" "), *Label);
			return;
		}

		TWeakObjectPtr<URammsBackendSwitchComponent> WeakSw = Found;
		TSharedRef<SVerticalBox>					 Rows = SNew(SVerticalBox);
		// shared per-joint current values for the readouts
		TSharedRef<TArray<float>> Values = MakeShared<TArray<float>>();
		Values->SetNumZeroed(Found->DriveJoints.Num());

		TSet<FName> SeenJoints;
		for (int32 i = 0; i < Found->DriveJoints.Num(); ++i)
		{
			const FName Joint = Found->DriveJoints[i];
			// A tendon registers one drive entry per wrapped joint under the SAME
			// name (both 2f85 drivers) — one slider commands them all; duplicate
			// rows just confused ("two finger motors moving the same finger").
			if (SeenJoints.Contains(Joint))
			{
				continue;
			}
			SeenJoints.Add(Joint);
			float	   Min = Found->DriveCtrlMin.IsValidIndex(i) ? Found->DriveCtrlMin[i] : 0.f;
			float	   Max = Found->DriveCtrlMax.IsValidIndex(i) ? Found->DriveCtrlMax[i] : 0.f;
			const bool bWheel = !Found->DriveIsPosition.IsValidIndex(i)
				|| !Found->DriveIsPosition[i];
			if (bWheel || FMath::IsNearlyEqual(Min, Max))
			{
				// velocity motors: command in rad/s
				Min = -6.f;
				Max = 6.f;
			}
			const int32 Index = i;
			Rows->AddSlot()
				.AutoHeight()
				.Padding(6.f, 3.f)
					[SNew(SHorizontalBox)
						+ SHorizontalBox::Slot()
							.FillWidth(0.36f)
							.VAlign(VAlign_Center)
								[SNew(STextBlock).Text(FText::FromName(Joint))]
						+ SHorizontalBox::Slot()
							.FillWidth(0.46f)
							.VAlign(VAlign_Center)
								[SNew(SSlider)
										.MinValue(Min)
										.MaxValue(Max)
										// bound to the shared value so "Zero all" (and any external
										// change) moves the knob too
										.Value_Lambda([Values, Index]() { return (*Values)[Index]; })
										.OnValueChanged_Lambda([WeakSw, Joint, Values, Index](float V) {
											(*Values)[Index] = V;
											if (URammsBackendSwitchComponent* Sw = WeakSw.Get())
											{
												Sw->SetJointCommand(Joint, V);
											}
										})]
						+ SHorizontalBox::Slot()
							.FillWidth(0.18f)
							.VAlign(VAlign_Center)
							.Padding(6.f, 0.f, 0.f, 0.f)
								[SNew(STextBlock)
										.Text_Lambda([Values, Index]() {
											return FText::FromString(
												FString::Printf(TEXT("%.3f"), (*Values)[Index]));
										})]];
		}

		Rows->AddSlot()
			.AutoHeight()
			.Padding(6.f)
				[SNew(SButton)
						.Text(FText::FromString(TEXT("Zero all")))
						.OnClicked_Lambda([WeakSw, Values]() {
							if (URammsBackendSwitchComponent* Sw = WeakSw.Get())
							{
								for (int32 i = 0; i < Sw->DriveJoints.Num(); ++i)
								{
									(*Values)[i] = 0.f;
									Sw->SetJointCommand(Sw->DriveJoints[i], 0.f);
								}
							}
							return FReply::Handled();
						})];

		Rows->AddSlot()
			.AutoHeight()
			.Padding(6.f, 8.f, 6.f, 4.f)
				[SNew(STextBlock)
						.Text(FText::FromString(TEXT(
							"Teleop (click panel first): W/S drive  A/D turn  Space stop\n"
							"Q/E elevator   R/F front rod   T/G rear rod")))];

		const float				 Height = FMath::Min(120.f + Found->DriveJoints.Num() * 34.f, 760.f);
		TSharedRef<SRammsTeleop> Teleop = SNew(SRammsTeleop, WeakSw)
			[SNew(SScrollBox) + SScrollBox::Slot()[Rows]];
		TSharedRef<SWindow> Window = SNew(SWindow)
										 .Title(FText::FromString(FString::Printf(TEXT("RAMMS Joints — %s"),
											 *FoundActor->GetActorNameOrLabel())))
										 .ClientSize(FVector2D(460.f, Height))
										 .SupportsMaximize(false)
											 [Teleop];
		FSlateApplication::Get().AddWindow(Window);
		FSlateApplication::Get().SetKeyboardFocus(Teleop);
	}

	void ValidateRobot(const TArray<FString>& Args, UWorld* World)
	{
		if (!World)
		{
			return;
		}
		const FString Label = Args.Num() > 0 ? Args[0] : TEXT("");
		bool		  bFoundAny = false;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			if (!Label.IsEmpty() && It->GetActorNameOrLabel() != Label && It->GetName() != Label)
			{
				continue;
			}
			URammsBackendSwitchComponent* Sw =
				It->FindComponentByClass<URammsBackendSwitchComponent>();
			const bool bMebotish = It->GetClass()->GetName().Contains(TEXT("mebot"))
				|| It->GetActorNameOrLabel().Contains(TEXT("mebot"))
				|| It->GetActorNameOrLabel().Contains(TEXT("ChaosBot"));
			if (!Sw && !bMebotish)
			{
				continue;
			}
			bFoundAny = true;
			FString Report = FString::Printf(TEXT("=== Ramms.Validate: %s ===\n"),
				*It->GetActorNameOrLabel());
			Report += FString::Printf(TEXT("  class: %s\n"), *It->GetClass()->GetPathName());
			if (!Sw)
			{
				Report += TEXT("  NO RammsBackendSwitchComponent — this actor has NO rig.\n"
							   "  Place /Game/Maps/URL/NewtonTest/mebot_gen3 instead.\n");
				UE_LOG(LogTemp, Error, TEXT("%s"), *Report);
				continue;
			}
			Report += FString::Printf(TEXT("  backend: %s\n"),
				Sw->Backend == ERammsPhysicsBackend::Chaos ? TEXT("Chaos") : TEXT("MuJoCo/Newton"));
			TArray<UPhysicsConstraintComponent*> Cons;
			It->GetComponents(Cons);
			int32 Resolved = 0;
			for (const FName& Rec : Sw->ChaosConstraintComponents)
			{
				for (UPhysicsConstraintComponent* C : Cons)
				{
					const FString A = C->GetName();
					const FString R = Rec.ToString();
					if (A == R || (A.StartsWith(R) && A.Mid(R.Len()).IsNumeric()))
					{
						++Resolved;
						break;
					}
				}
			}
			// MuJoCo-mode instances destroy their rig constraints at BeginPlay by
			// design (ApplyMuJoCo) — a low resolve count there is NOT staleness.
			const bool bChaos = Sw->Backend == ERammsPhysicsBackend::Chaos;
			Report += FString::Printf(TEXT("  rig constraints: %d recorded, %d resolved%s\n"),
				Sw->ChaosConstraintComponents.Num(), Resolved,
				(bChaos && Resolved < Sw->ChaosConstraintComponents.Num())
					? TEXT("  << STALE INSTANCE — DELETE AND RE-PLACE")
					: (bChaos ? TEXT(" (OK)")
							  : TEXT(" (MuJoCo mode: rig constraints removed at BeginPlay by design)")));
			Report += FString::Printf(TEXT("  drives: %d (incl. gripper tendon: %s)\n"),
				Sw->DriveJoints.Num(),
				Sw->DriveJoints.Contains(FName(TEXT("arm_2f85_split"))) ? TEXT("yes") : TEXT("NO — old rig"));
			int32						  MeshesNoCol = 0, MeshesTotal = 0;
			TArray<UStaticMeshComponent*> Meshes;
			It->GetComponents(Meshes);
			for (UStaticMeshComponent* M : Meshes)
			{
				if (!M->GetStaticMesh() || !M->GetStaticMesh()->GetBodySetup())
				{
					continue;
				}
				++MeshesTotal;
				if (M->GetStaticMesh()->GetBodySetup()->AggGeom.GetElementCount() == 0)
				{
					++MeshesNoCol;
				}
			}
			Report += FString::Printf(TEXT("  meshes: %d, without collision shapes: %d\n"),
				MeshesTotal, MeshesNoCol);
			UE_LOG(LogTemp, Display, TEXT("%s"), *Report);
			if (GEngine)
			{
				GEngine->AddOnScreenDebugMessage(-1, 12.f,
					Resolved < Sw->ChaosConstraintComponents.Num() ? FColor::Red : FColor::Green,
					Report);
			}
		}
		if (!bFoundAny)
		{
			UE_LOG(LogTemp, Warning, TEXT("Ramms.Validate: no robot-like actor found%s%s"),
				Label.IsEmpty() ? TEXT("") : TEXT(" matching "), *Label);
		}
	}

	// Ramms.Probe <ActorLabel> [Seconds] — headless-friendly rest/motion probe.
	// Samples the rig for N seconds (default 5) and logs one report: base pose /
	// velocity (settle + creep), every drive constraint's current twist or linear
	// extension, and every closure pin's anchor separation (frame-1 vs frame-2
	// world anchors — the direct measure of pin leak/stretch). Runs in -game.
	void ProbeRobot(const TArray<FString>& Args, UWorld* World)
	{
		if (!World)
		{
			return;
		}
		const FString				  Label = Args.Num() > 0 ? Args[0] : TEXT("");
		const float					  Seconds = Args.Num() > 1 ? FCString::Atof(*Args[1]) : 5.f;
		URammsBackendSwitchComponent* Sw = nullptr;
		AActor*						  Robot = nullptr;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			if (!Label.IsEmpty() && It->GetActorNameOrLabel() != Label && It->GetName() != Label)
			{
				continue;
			}
			if (URammsBackendSwitchComponent* S =
					It->FindComponentByClass<URammsBackendSwitchComponent>())
			{
				Sw = S;
				Robot = *It;
				break;
			}
		}
		if (!Sw)
		{
			UE_LOG(LogTemp, Warning, TEXT("Ramms.Probe: no robot with a backend switch found"));
			return;
		}

		struct FProbeState
		{
			TWeakObjectPtr<AActor>						 Robot;
			TWeakObjectPtr<URammsBackendSwitchComponent> Sw;
			FVector										 StartLoc = FVector::ZeroVector;
			double										 EndTime = 0.0;
			float										 MaxSpeed = 0.f;
			float										 SpeedAccum = 0.f;
			int32										 Samples = 0;
			int32										 LastConsSeen = 0;
			int32										 LastConsMatched = 0;
			TMap<FName, FVector2D>						 TwistFirstLast; // deg or cm
			TArray<TPair<FName, float>>					 BodyMasses;	 // kg, sampled once
			TMap<FName, FQuat>							 GripInitRel;	 // body1^-1*body2 at t0
			TMap<FName, float>							 GripDirectDeg;	 // true hinge angle
			double										 StartTime = 0.0;
			FVector2D									 ChassisZRange =
				FVector2D(TNumericLimits<float>::Max(), TNumericLimits<float>::Lowest());
			TMap<FName, FVector2D> BodySpeed;	 // X=accum, Y=max (cm/s)
			TMap<FName, FVector>   BodyLastPos;	 // world, for pos-delta speed
			TMap<FName, FVector2D> BodyPosSpeed; // X=accum, Y=max (cm/s)
			double				   LastSampleTime = 0.0;
			int32				   BodySpeedSamples = 0;
		};
		TSharedRef<FProbeState> St = MakeShared<FProbeState>();
		St->Robot = Robot;
		St->Sw = Sw;
		St->StartLoc = Robot->GetActorLocation();
		St->StartTime = World->GetTimeSeconds();
		St->EndTime = World->GetTimeSeconds() + FMath::Max(Seconds, 0.5f);

		TSharedRef<FTimerHandle> Handle = MakeShared<FTimerHandle>();
		World->GetTimerManager().SetTimer(*Handle,
			FTimerDelegate::CreateLambda([St = St, Handle = Handle, World]() mutable {
				// ClearTimer from inside a repeating timer callback destroys the
				// delegate — and with it these captures — IMMEDIATELY. The locals
				// below keep the state/handle alive for the rest of this
				// invocation (first symptom: the final report read a destructed
				// TMap and showed "0 mapped" while the same tick logged 77).
				TSharedRef<FProbeState>		  S = St;
				TSharedRef<FTimerHandle>	  H = Handle;
				UWorld*						  W = World;
				AActor*						  Robot = S->Robot.Get();
				URammsBackendSwitchComponent* Sw = S->Sw.Get();
				if (!Robot || !Sw)
				{
					W->GetTimerManager().ClearTimer(*H);
					return;
				}
				// Base velocity from the first resolvable rig body.
				TArray<UPrimitiveComponent*> Prims;
				Robot->GetComponents(Prims);
				for (UPrimitiveComponent* P : Prims)
				{
					if (P->IsSimulatingPhysics()
						&& Sw->ChaosBodyComponents.Num()
						&& P->GetFName().ToString().StartsWith(
							Sw->ChaosBodyComponents[0].ToString()))
					{
						const float Speed = P->GetPhysicsLinearVelocity().Size();
						S->MaxSpeed = FMath::Max(S->MaxSpeed, Speed);
						S->SpeedAccum += Speed;
						++S->Samples;
						// Rest-bounce metric: base body Z range AFTER the spawn
						// settle (first 3 s excluded). A parked robot should
						// show sub-mm here; the user-visible "bouncing at rest"
						// reads as an amplitude in mm-cm.
						if (W->GetTimeSeconds() > S->StartTime + 3.0)
						{
							const float Z = P->GetComponentLocation().Z;
							S->ChassisZRange.X = FMath::Min(S->ChassisZRange.X, (double)Z);
							S->ChassisZRange.Y = FMath::Max(S->ChassisZRange.Y, (double)Z);
						}
						break;
					}
				}
				// Per-body speed attribution: meanV alone can't say WHICH body
				// is pumping (observed: base meanV 12k cm/s while upZ=1.0 and
				// drift=0 — some rig body oscillating violently in place).
				// BodyPosSpeed tracks the POSITION-DELTA speed at the sampling
				// rate: if it stays near zero while the physics velocity reads
				// 100 m/s, the velocity is phantom (solver-internal, positions
				// clamped by constraints) rather than real displacement.
				{
					const double Now = W->GetTimeSeconds();
					const double Dt = S->LastSampleTime > 0.0 ? Now - S->LastSampleTime : 0.0;
					for (UPrimitiveComponent* P : Prims)
					{
						if (!P->IsSimulatingPhysics())
						{
							continue;
						}
						FVector2D&	BS = S->BodySpeed.FindOrAdd(P->GetFName());
						const float Speed = P->GetPhysicsLinearVelocity().Size();
						BS.X += Speed;
						BS.Y = FMath::Max<double>(BS.Y, Speed);
						const FVector Pos = P->GetComponentLocation();
						if (FVector* Last = S->BodyLastPos.Find(P->GetFName()))
						{
							if (Dt > 1e-4)
							{
								FVector2D&	PS = S->BodyPosSpeed.FindOrAdd(P->GetFName());
								const float PosSpeed = (Pos - *Last).Size() / Dt;
								PS.X += PosSpeed;
								PS.Y = FMath::Max<double>(PS.Y, PosSpeed);
							}
							*Last = Pos;
						}
						else
						{
							S->BodyLastPos.Add(P->GetFName(), Pos);
						}
					}
					S->LastSampleTime = Now;
				}
				++S->BodySpeedSamples;
				// One-shot mass audit: what does Chaos ACTUALLY simulate per
				// rig body? (BodyMasses records intent; floors/overrides/
				// auto-mass can diverge — wrong distribution reads as "the
				// arm tips the 140 kg base".)
				if (S->BodyMasses.Num() == 0)
				{
					for (UPrimitiveComponent* P : Prims)
					{
						if (P->IsSimulatingPhysics())
						{
							S->BodyMasses.Emplace(P->GetFName(), P->GetMass());
						}
					}
				}
				// Per-constraint values (drives by twist/linear, pins by separation
				// — recorded under the constraint's name).
				TArray<UPhysicsConstraintComponent*> Cons;
				Robot->GetComponents(Cons);
				S->LastConsSeen = Cons.Num();
				S->LastConsMatched = 0;
				for (UPhysicsConstraintComponent* C : Cons)
				{
					if (!C->GetFName().ToString().StartsWith(TEXT("ChaosRig_")))
					{
						continue;
					}
					++S->LastConsMatched;
					float Value;
					if (C->GetFName().ToString().StartsWith(TEXT("ChaosRig_pin_")))
					{
						// Pin separation in cm: world positions of the two recorded
						// ref-frame anchors (body-space ref frame x component pose).
						UPrimitiveComponent* P1 = nullptr;
						UPrimitiveComponent* P2 = nullptr;
						FName				 B1, B2;
						C->GetConstrainedComponents(P1, B1, P2, B2);
						if (!P1 || !P2)
						{
							continue;
						}
						// Scale-free body transforms: the solver applies ref-frame
						// positions against unscaled body poses; multiplying
						// through a scaled COMPONENT transform (gripper meshes
						// carry ~0.001 compensating scales) corrupts the metric.
						auto BodyXf = [](UPrimitiveComponent* P) {
							FBodyInstance* BI = P->GetBodyInstance();
							FTransform	   T =
								BI ? BI->GetUnrealWorldTransform() : P->GetComponentTransform();
							T.RemoveScaling();
							return T;
						};
						const FVector A1 = BodyXf(P1).TransformPosition(
							C->ConstraintInstance.GetRefFrame(EConstraintFrame::Frame1).GetLocation());
						const FVector A2 = BodyXf(P2).TransformPosition(
							C->ConstraintInstance.GetRefFrame(EConstraintFrame::Frame2).GetLocation());
						Value = (A1 - A2).Size();
						if (S->Samples <= 1)
						{
							UE_LOG(LogTemp, Display,
								TEXT("[RammsProbe] pin %s: P1=%s(sim=%d) P2=%s(sim=%d) broken=%d"),
								*C->GetName(), *P1->GetName(), P1->IsSimulatingPhysics() ? 1 : 0,
								*P2->GetName(), P2->IsSimulatingPhysics() ? 1 : 0,
								C->ConstraintInstance.IsBroken() ? 1 : 0);
						}
					}
					else if (C->ConstraintInstance.GetLinearXMotion() != ELinearConstraintMotion::LCM_Locked)
					{
						UPrimitiveComponent* P1 = nullptr;
						UPrimitiveComponent* P2 = nullptr;
						FName				 B1, B2;
						C->GetConstrainedComponents(P1, B1, P2, B2);
						Value = (P1 && P2)
							? FVector::DotProduct(P2->GetComponentLocation() - P1->GetComponentLocation(),
								  C->GetForwardVector())
							: 0.f;
					}
					else
					{
						Value = C->GetCurrentTwist();
						// Independent angle measurement for the gripper hinges:
						// GetCurrentTwist trusts the solver's frame bookkeeping.
						// Recompute the joint angle DIRECTLY from the two bodies'
						// world rotations — the rotation accumulated since t0,
						// resolved about the constraint's body1-local hinge axis
						// (PriAxis1). If this disagrees with GetCurrentTwist the
						// "limit violation" is a measurement artifact.
						if (C->GetFName().ToString().Contains(TEXT("arm_2f85")))
						{
							UPrimitiveComponent* P1 = nullptr;
							UPrimitiveComponent* P2 = nullptr;
							FName				 Bq1, Bq2;
							C->GetConstrainedComponents(P1, Bq1, P2, Bq2);
							if (P1 && P2)
							{
								auto BodyQuat = [](UPrimitiveComponent* P) {
									FBodyInstance* BI = P->GetBodyInstance();
									return BI ? BI->GetUnrealWorldTransform().GetRotation()
											  : P->GetComponentQuat();
								};
								const FQuat Rel = BodyQuat(P1).Inverse() * BodyQuat(P2);
								if (const FQuat* Init = S->GripInitRel.Find(C->GetFName()))
								{
									// Delta = Rel * Init^-1 is the relative rotation
									// since t0 expressed in body1 space; its signed
									// angle about axis a is 2*atan2(dot(q.xyz,a), q.w).
									const FQuat	  Delta = Rel * Init->Inverse();
									const FVector Axis =
										C->ConstraintInstance.PriAxis1.GetSafeNormal();
									const float Signed = FMath::RadiansToDegrees(2.f
										* FMath::Atan2(
											FVector::DotProduct(
												FVector(Delta.X, Delta.Y, Delta.Z), Axis),
											Delta.W));
									S->GripDirectDeg.Add(C->GetFName(),
										FRotator::NormalizeAxis(Signed));
								}
								else
								{
									S->GripInitRel.Add(C->GetFName(), Rel);
								}
							}
						}
					}
					FVector2D& FL = S->TwistFirstLast.FindOrAdd(C->GetFName(),
						FVector2D(Value, Value));
					FL.Y = Value;
					if (S->Samples <= 1 && C->GetFName().ToString().Contains(TEXT("arm_2f85"))
						&& !C->GetFName().ToString().Contains(TEXT("pin")))
					{
						// Full constraint anatomy for the gripper hinges — the
						// angular DOFs are mysteriously inert on these links
						// while linear locks enforce; dump everything the
						// solver was given.
						const FConstraintInstance& CI = C->ConstraintInstance;
						const FConstraintDrive&	   TD = CI.ProfileInstance.AngularDrive.TwistDrive;
						UPrimitiveComponent*	   P1 = nullptr;
						UPrimitiveComponent*	   P2 = nullptr;
						FName					   Bn1, Bn2;
						C->GetConstrainedComponents(P1, Bn1, P2, Bn2);
						UE_LOG(LogTemp, Display,
							TEXT("[RammsProbe] gripcon %s: P1=%s(sim=%d scl=%s) P2=%s(sim=%d scl=%s)"),
							*C->GetName(),
							P1 ? *P1->GetName() : TEXT("null"),
							P1 && P1->IsSimulatingPhysics() ? 1 : 0,
							P1 ? *P1->GetComponentScale().ToString() : TEXT("-"),
							P2 ? *P2->GetName() : TEXT("null"),
							P2 && P2->IsSimulatingPhysics() ? 1 : 0,
							P2 ? *P2->GetComponentScale().ToString() : TEXT("-"));
						UE_LOG(LogTemp, Display,
							TEXT("[RammsProbe]   Pos1=%s Pri1=%s | Pos2=%s Pri2=%s | Cscl=%s"),
							*CI.Pos1.ToString(), *CI.PriAxis1.ToString(),
							*CI.Pos2.ToString(), *CI.PriAxis2.ToString(),
							*C->GetComponentScale().ToString());
						UE_LOG(LogTemp, Display,
							TEXT("[RammsProbe]   twist=%.1f swing1=%.1f swing2=%.1f | drive pos=%d vel=%d k=%.0f d=%.0f mode=%d | limits tw=%d(%d,%.0f) sw1=%d sw2=%d"),
							C->GetCurrentTwist(), C->GetCurrentSwing1(), C->GetCurrentSwing2(),
							TD.bEnablePositionDrive ? 1 : 0, TD.bEnableVelocityDrive ? 1 : 0,
							TD.Stiffness, TD.Damping,
							(int32)CI.ProfileInstance.AngularDrive.AngularDriveMode,
							(int32)CI.GetAngularTwistMotion(),
							CI.ProfileInstance.TwistLimit.bSoftConstraint ? 1 : 0,
							CI.ProfileInstance.TwistLimit.TwistLimitDegrees,
							(int32)CI.GetAngularSwing1Motion(), (int32)CI.GetAngularSwing2Motion());
					}
				}
				if (W->GetTimeSeconds() >= S->EndTime)
				{
					W->GetTimerManager().ClearTimer(*H);
					const FVector Drift = Robot->GetActorLocation() - S->StartLoc;
					const FVector Up = Robot->GetActorQuat().GetUpVector();
					FString		  Report = FString::Printf(
						TEXT("=== Ramms.Probe %s ===\n  upZ=%.2f  drift=%.1f cm  ")
							TEXT("meanV=%.2f cm/s  maxV=%.2f cm/s  ")
								TEXT("(constraints: %d seen, %d rig)\n"),
						*Robot->GetActorNameOrLabel(), Up.Z, Drift.Size2D(),
						S->Samples ? S->SpeedAccum / S->Samples : -1.f, S->MaxSpeed,
						S->LastConsSeen, S->LastConsMatched);
					TArray<FName> Keys;
					S->TwistFirstLast.GenerateKeyArray(Keys);
					Keys.Sort(FNameLexicalLess());
					for (const FName& K : Keys)
					{
						const FVector2D& FL = S->TwistFirstLast[K];
						const FString	 KS = K.ToString();
						const float*	 Direct = S->GripDirectDeg.Find(K);
						FString			 LiveCfg;
						if (Direct)
						{
							// LIVE gripper-constraint config at report time (the
							// t=0 anatomy dump can race ApplyChaos re-config).
							TArray<UPhysicsConstraintComponent*> RCons;
							Robot->GetComponents(RCons);
							for (UPhysicsConstraintComponent* RC : RCons)
							{
								if (RC->GetFName() == K)
								{
									const FConstraintInstance& RCI = RC->ConstraintInstance;
									LiveCfg = FString::Printf(
										TEXT("  tw=%d(%.0f soft=%d) k=%.0f"),
										(int32)RCI.GetAngularTwistMotion(),
										RCI.ProfileInstance.TwistLimit.TwistLimitDegrees,
										RCI.ProfileInstance.TwistLimit.bSoftConstraint ? 1 : 0,
										RCI.ProfileInstance.AngularDrive.TwistDrive.Stiffness);
									break;
								}
							}
						}
						Report += FString::Printf(TEXT("  %-52s %8.3f -> %8.3f %s%s%s\n"), *KS,
							FL.X, FL.Y,
							KS.StartsWith(TEXT("ChaosRig_pin_")) ? TEXT("cm sep")
																 : TEXT("deg/cm"),
							Direct ? *FString::Printf(TEXT("  direct=%.1f deg"), *Direct)
								   : TEXT(""),
							*LiveCfg);
					}
					if (S->ChassisZRange.Y >= S->ChassisZRange.X)
					{
						Report += FString::Printf(
							TEXT("  restZ: base Z %.3f..%.3f cm (amplitude %.2f mm, post-settle)\n"),
							S->ChassisZRange.X, S->ChassisZRange.Y,
							(S->ChassisZRange.Y - S->ChassisZRange.X) * 10.f);
					}
					// Fastest bodies: attributes the meanV energy to specific rig
					// pieces (a violent in-place oscillator shows up here with
					// near-zero net drift).
					if (S->BodySpeedSamples > 0)
					{
						TArray<TPair<FName, FVector2D>> Fast;
						for (const TPair<FName, FVector2D>& BP : S->BodySpeed)
						{
							Fast.Add(BP);
						}
						Fast.Sort([](const TPair<FName, FVector2D>& A, const TPair<FName, FVector2D>& B) {
							return A.Value.X > B.Value.X;
						});
						const int32 NumFast = FMath::Min(Fast.Num(), 8);
						for (int32 FI = 0; FI < NumFast; ++FI)
						{
							const FVector2D* PS = S->BodyPosSpeed.Find(Fast[FI].Key);
							Report += FString::Printf(
								TEXT("  bodyV %-58s mean=%8.1f max=%8.1f cm/s  posV mean=%8.1f max=%8.1f cm/s\n"),
								*Fast[FI].Key.ToString(),
								Fast[FI].Value.X / S->BodySpeedSamples, Fast[FI].Value.Y,
								PS ? PS->X / FMath::Max(1, S->BodySpeedSamples - 1) : -1.f,
								PS ? PS->Y : -1.f);
						}
					}
					// ABSOLUTE end positions of key bodies. drift/upZ read the
					// (non-simulated) actor ROOT and stay perfect even when the
					// whole physics assembly departs — a regen against broken
					// source content once had the entire robot coherently flying
					// at 100 m/s while every relative metric looked healthy.
					for (const TPair<FName, FVector>& BP : S->BodyLastPos)
					{
						if (BP.Key.ToString().Contains(TEXT("chassis"))
							|| BP.Key.ToString().Contains(TEXT("2f85_pad"))
							|| BP.Key.ToString().Contains(TEXT("drive_wheel")))
						{
							Report += FString::Printf(TEXT("  endpos %-58s %s\n"),
								*BP.Key.ToString(), *BP.Value.ToCompactString());
						}
					}
					// Mass audit: totals + the heaviest bodies (full per-body
					// list only in the file dump).
					{
						float TotalKg = 0.f, ArmKg = 0.f;
						for (const TPair<FName, float>& M : S->BodyMasses)
						{
							TotalKg += M.Value;
							if (M.Key.ToString().StartsWith(TEXT("Viz_arm_"))
								|| M.Key.ToString().Contains(TEXT("__arm_")))
							{
								ArmKg += M.Value;
							}
						}
						Report += FString::Printf(
							TEXT("  masses: total=%.1f kg, arm-ish=%.1f kg, base=%.1f kg, %d bodies\n"),
							TotalKg, ArmKg, TotalKg - ArmKg, S->BodyMasses.Num());
						TArray<TPair<FName, float>> Sorted = S->BodyMasses;
						Sorted.Sort([](const TPair<FName, float>& A, const TPair<FName, float>& B) {
							return A.Value > B.Value;
						});
						for (const TPair<FName, float>& M : Sorted)
						{
							Report += FString::Printf(TEXT("  mass %-58s %9.3f kg\n"),
								*M.Key.ToString(), M.Value);
						}
					}
					// Log line-by-line (multi-line entries can be truncated by some
					// sinks) and dump the full report to Saved/ for headless runs.
					TArray<FString> ReportLines;
					Report.ParseIntoArrayLines(ReportLines);
					for (const FString& L : ReportLines)
					{
						UE_LOG(LogTemp, Display, TEXT("[RammsProbe] %s"), *L);
					}
					FFileHelper::SaveStringToFile(Report,
						*(FPaths::ProjectSavedDir() / TEXT("RammsProbe.txt")));
				}
			}),
			0.05f, true);
		UE_LOG(LogTemp, Display, TEXT("Ramms.Probe: sampling %s for %.1f s..."),
			*Robot->GetActorNameOrLabel(), Seconds);
	}

	// Ramms.ReproHinge — minimal repro for the gripper "angular DOFs inert"
	// mystery: two plain cubes at gripper-like masses joined by ONE hinge
	// constraint configured exactly like a 2f85 driver joint (linear locked,
	// swings locked hard, twist limited hard, TwistAndSwing pos+vel drive,
	// projection on, runtime Term/Update/Init cycle). The child hangs 10 cm
	// off-axis so gravity torques it about the twist axis; if the limit
	// works it stops at LimitDeg, if not it runs to ~90.
	//   Ramms.ReproHinge [flags] [limitDeg] [k] [d] [childKg] [parentKg]
	//   flags: 1=posDrive 2=velDrive 4=reinit-cycle 8=parent-simulates(+world
	//   weld) 16=projection-OFF 32=soft-limit   (default 15 = driver mimic)
	static void ReproHinge(const TArray<FString>& Args, UWorld* World)
	{
		if (!World)
		{
			return;
		}
		const int32	 Flags = Args.Num() > 0 ? FCString::Atoi(*Args[0]) : 15;
		const float	 LimitDeg = Args.Num() > 1 ? FCString::Atof(*Args[1]) : 45.8f;
		const float	 K = Args.Num() > 2 ? FCString::Atof(*Args[2]) : 1e4f;
		const float	 D = Args.Num() > 3 ? FCString::Atof(*Args[3]) : 5e3f;
		const float	 ChildKg = Args.Num() > 4 ? FCString::Atof(*Args[4]) : 0.05f;
		const float	 ParentKg = Args.Num() > 5 ? FCString::Atof(*Args[5]) : 0.78f;
		UStaticMesh* Cube =
			LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		if (!Cube)
		{
			UE_LOG(LogTemp, Warning, TEXT("Ramms.ReproHinge: no engine cube mesh"));
			return;
		}
		AActor* A = World->SpawnActor<AActor>();
		if (!A)
		{
			return;
		}
		USceneComponent* Root = NewObject<USceneComponent>(A, TEXT("ReproRoot"));
		A->SetRootComponent(Root);
		Root->RegisterComponent();
		A->SetActorLocation(FVector(0.f, 0.f, 300.f));
		auto MakeCube = [&](const TCHAR* Name, const FVector& Pos, float Kg, bool bSim) {
			UStaticMeshComponent* M = NewObject<UStaticMeshComponent>(A, Name);
			M->SetupAttachment(Root);
			M->RegisterComponent();
			M->SetStaticMesh(Cube);
			M->SetWorldScale3D(FVector(0.04f)); // 4 cm cube
			M->SetRelativeLocation(Pos);
			// No contacts: isolate constraint behavior from collision.
			M->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
			M->SetCollisionResponseToAllChannels(ECR_Ignore);
			M->SetSimulatePhysics(bSim);
			if (bSim)
			{
				M->SetMassOverrideInKg(NAME_None, Kg, true);
			}
			return M;
		};
		const bool			  bParentSim = (Flags & 8) != 0;
		UStaticMeshComponent* Parent =
			MakeCube(TEXT("ReproParent"), FVector::ZeroVector, ParentKg, bParentSim);
		UStaticMeshComponent* Child =
			MakeCube(TEXT("ReproChild"), FVector(0.f, 10.f, 0.f), ChildKg, true);
		if (bParentSim)
		{
			// Weld the simulating parent to the world so the pair doesn't
			// free-fall (stand-in for the palm hanging off the held arm).
			UPhysicsConstraintComponent* Weld =
				NewObject<UPhysicsConstraintComponent>(A, TEXT("ReproWeld"));
			Weld->SetupAttachment(Root);
			Weld->RegisterComponent();
			FConstraintInstance& WI = Weld->ConstraintInstance;
			WI.SetLinearXLimit(LCM_Locked, 0.f);
			WI.SetLinearYLimit(LCM_Locked, 0.f);
			WI.SetLinearZLimit(LCM_Locked, 0.f);
			WI.SetAngularSwing1Limit(ACM_Locked, 0.f);
			WI.SetAngularSwing2Limit(ACM_Locked, 0.f);
			WI.SetAngularTwistLimit(ACM_Locked, 0.f);
			Weld->SetConstrainedComponents(nullptr, NAME_None, Parent, NAME_None);
		}
		UPhysicsConstraintComponent* C =
			NewObject<UPhysicsConstraintComponent>(A, TEXT("ReproHinge"));
		C->SetupAttachment(Root);
		C->RegisterComponent();
		C->SetWorldLocation(Parent->GetComponentLocation()); // axis = world +X
		FConstraintInstance& CI = C->ConstraintInstance;
		CI.ProfileInstance.bDisableCollision = true;
		CI.ProfileInstance.bEnableProjection = (Flags & 16) == 0;
		CI.SetLinearXLimit(LCM_Locked, 0.f);
		CI.SetLinearYLimit(LCM_Locked, 0.f);
		CI.SetLinearZLimit(LCM_Locked, 0.f);
		CI.SetAngularSwing1Limit(ACM_Locked, 0.f);
		CI.SetAngularSwing2Limit(ACM_Locked, 0.f);
		CI.ProfileInstance.ConeLimit.bSoftConstraint = false;
		CI.SetAngularTwistLimit(ACM_Limited, LimitDeg);
		CI.ProfileInstance.TwistLimit.bSoftConstraint = (Flags & 32) != 0;
		if (Flags & (1 | 2))
		{
			CI.SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
			CI.SetOrientationDriveTwistAndSwing((Flags & 1) != 0, false);
			CI.SetAngularVelocityDriveTwistAndSwing((Flags & 2) != 0, false);
			CI.SetAngularDriveParams(K, D, 0.f);
		}
		C->SetConstrainedComponents(Parent, NAME_None, Child, NAME_None);
		if (Flags & 4)
		{
			// The runtime ApplyChaos cycle: constraints get terminated,
			// re-framed, and re-initialized after bodies flip to simulated.
			C->TermComponentConstraint();
			C->UpdateConstraintFrames();
			C->InitComponentConstraint();
		}
		UE_LOG(LogTemp, Display,
			TEXT("[RammsRepro] spawned: flags=%d limit=%.1f k=%.0f d=%.0f child=%.2fkg parent=%.2fkg"),
			Flags, LimitDeg, K, D, ChildKg, ParentKg);
		TSharedRef<FTimerHandle> Handle = MakeShared<FTimerHandle>();
		TWeakObjectPtr<AActor>	 WA = A;
		TSharedRef<double>		 End = MakeShared<double>(World->GetTimeSeconds() + 6.0);
		World->GetTimerManager().SetTimer(*Handle,
			FTimerDelegate::CreateLambda([WA, Handle, End, World, Flags]() {
				TSharedRef<FTimerHandle> H = Handle;
				AActor*					 Act = WA.Get();
				if (!Act || World->GetTimeSeconds() >= *End)
				{
					World->GetTimerManager().ClearTimer(*H);
					return;
				}
				UPhysicsConstraintComponent* C =
					Act->FindComponentByClass<UPhysicsConstraintComponent>();
				TArray<UPhysicsConstraintComponent*> Cs;
				Act->GetComponents(Cs);
				for (UPhysicsConstraintComponent* X : Cs)
				{
					if (X->GetFName() == FName(TEXT("ReproHinge")))
					{
						C = X;
					}
				}
				if (C)
				{
					UE_LOG(LogTemp, Display,
						TEXT("[RammsRepro] flags=%d t=%.1f twist=%.1f swing1=%.1f swing2=%.1f"),
						Flags, World->GetTimeSeconds(), C->GetCurrentTwist(),
						C->GetCurrentSwing1(), C->GetCurrentSwing2());
				}
			}),
			0.5f, true);
	}

	FAutoConsoleCommandWithWorldAndArgs GRammsReproHingeCmd(
		TEXT("Ramms.ReproHinge"),
		TEXT("Ramms.ReproHinge [flags] [limitDeg] [k] [d] [childKg] [parentKg] "
			 "— minimal 2-cube hinge repro of the gripper driver constraint."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&ReproHinge));

	FAutoConsoleCommandWithWorldAndArgs GRammsProbeCmd(
		TEXT("Ramms.Probe"),
		TEXT("Ramms.Probe [ActorLabel] [Seconds] — sample the rig and log base "
			 "settle/creep, drive twists, and closure-pin separations."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&ProbeRobot));

	FAutoConsoleCommandWithWorldAndArgs GRammsValidateCmd(
		TEXT("Ramms.Validate"),
		TEXT("Ramms.Validate [ActorLabel] — print rig health for robot actors "
			 "(class path, constraint resolution, drives, collision counts)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&ValidateRobot));

	FAutoConsoleCommandWithWorldAndArgs GRammsPanelCmd(
		TEXT("Ramms.Panel"),
		TEXT("Ramms.Panel [ActorLabel] — open a joint control panel driving "
			 "SetJointCommand (works on MuJoCo, Newton, and Chaos backends)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&OpenJointPanel));

} // namespace
