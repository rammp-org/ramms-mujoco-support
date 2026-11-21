// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMujocoActuationBackend.h"
#include "RammsRobotBaseComponent.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "mujoco/mujoco.h"

static TAutoConsoleVariable<int32>	 CVarRammsMjStiffDrive(TEXT("ramms.mj.StiffDrive"), 1,
	  TEXT("A stiff speed loop (FRammsVelocityGains::StiffDamping): a plain force actuator (<motor>) driven at a speed is made a ")
		  TEXT("MuJoCo velocity servo for as long as it is -- its damping integrated implicitly by the solver, stable at gains a loop ")
			  TEXT("closed outside it cannot use -- with the integral on top, within the motor's torque range; a step that would ask ")
				  TEXT("more than the range runs as the plain motor at its limit. Put back as it was once the loop is dropped. ")
					  TEXT("1 (the default): for the motors whose gains ask for it; 0: for none; 2 (experiments): also for every motor named ")
						  TEXT("in ramms.mj.StiffDriveMotors, at ramms.mj.StiffDriveDamping / StiffDriveIntegral."),
	  ECVF_Default);
static TAutoConsoleVariable<FString> CVarRammsMjStiffDriveMotors(TEXT("ramms.mj.StiffDriveMotors"), TEXT("omni"),
	TEXT("With ramms.mj.StiffDrive 2: the motors whose registry Id contains one of these (comma-separated, any case; empty: ")
		TEXT("every one) get a stiff loop at the gains below."),
	ECVF_Default);
static TAutoConsoleVariable<float>	 CVarRammsMjStiffDriveDamping(TEXT("ramms.mj.StiffDriveDamping"), 60.0f,
	  TEXT("With ramms.mj.StiffDrive 2: the servo's damping at the joint (N*m per rad/s; N per m/s on a slide)."), ECVF_Default);
static TAutoConsoleVariable<float>	 CVarRammsMjStiffDriveIntegral(TEXT("ramms.mj.StiffDriveIntegral"), 1000.0f,
	  TEXT("With ramms.mj.StiffDrive 2: the integral gain at the joint (N*m per rad behind the command), up to the motor's own ")
		  TEXT("integral cap (FRammsVelocityGains::MaxIntegralTorque)."),
	  ECVF_Default);

static TAutoConsoleVariable<int32> CVarRammsMjServoFeedForward(TEXT("ramms.mj.ServoFeedForward"), 1,
	TEXT("A position servo following a physics-rate profile (SetPositionProfile) is also given its setpoint's rate: its ")
		TEXT("command is the setpoint led by kv / kp of that rate, so the PD's damping acts on the speed error rather than the ")
			TEXT("speed and the servo follows a moving setpoint without its velocity lag -- a real servo drive's velocity ")
				TEXT("feedforward -- at the same stiffness. 1 (the default): on; 0: the setpoint alone."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarRammsMjTorqueCaps(TEXT("ramms.mj.TorqueCaps"), 1,
	TEXT("A plain position servo's torque limit (SetMotorTorqueLimit: a drive's per-direction limit, narrowing its force range "
		 "every physics step): 1 (the default) applied; 0 refused, and every limit in force put back from the next physics step."),
	ECVF_Default);

bool FRammsMujocoActuationBackend::Initialize(URammsRobotBaseComponent& Base)
{
	Articulation = nullptr;
	ActuatorByMotor.Reset();
	TransmissionByMotor.Reset();
	WarnedUnsupported.Reset();

	BaseComponent = &Base;
	Articulation = FindArticulation(Base);
	if (!Articulation.IsValid())
	{
		return false; // no MuJoCo articulation on this robot — base component falls back
	}

	// UE owns the drive: select the internal (UI/Blueprint) control slot so
	// staged values reach d->ctrl (0 = ZMQ/network, non-zero = UI).
	Articulation->ControlSource = 1;
	// An articulation controller (an arm's IK on a composed robot) runs after
	// the staged slots and overwrites only the actuators it drives, so the
	// base's motors keep their commands under it.
	return true;
}

AMjArticulation* FRammsMujocoActuationBackend::FindArticulation(const URammsRobotBaseComponent& Base)
{
	// The owner itself, else one attached under it (a composed robot whose base
	// component lives on a parent actor). Never an arbitrary articulation
	// elsewhere in the level — with Backend=Auto that would silently hijack an
	// unrelated robot's actuators (and its control source) while leaving this
	// one undriven.
	AActor* Owner = Base.GetOwner();
	if (!Owner)
	{
		return nullptr;
	}
	if (AMjArticulation* AsArt = Cast<AMjArticulation>(Owner))
	{
		return AsArt;
	}
	TArray<AActor*> Children;
	Owner->GetAttachedActors(Children, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
	for (AActor* Child : Children)
	{
		if (AMjArticulation* ChildArt = Cast<AMjArticulation>(Child))
		{
			return ChildArt;
		}
	}
	return nullptr;
}

UMjNodeComponent* FRammsMujocoActuationBackend::ResolveActuator(FName MotorId) const
{
	if (const TWeakObjectPtr<UMjNodeComponent>* Cached = ActuatorByMotor.Find(MotorId))
	{
		if (Cached->IsValid())
		{
			return Cached->Get();
		}
	}
	AMjArticulation* Art = Articulation.Get();
	if (!Art)
	{
		return nullptr;
	}
	UMjNodeComponent* Actuator = Art->GetActuator(MotorId.ToString());
	ActuatorByMotor.Add(MotorId, Actuator);
	return Actuator;
}

const FRammsMujocoActuationBackend::FTransmission* FRammsMujocoActuationBackend::ResolveTransmission(FName MotorId) const
{
	if (const FTransmission* Cached = TransmissionByMotor.Find(MotorId))
	{
		if (Cached->Joint.IsValid())
		{
			return Cached;
		}
		TransmissionByMotor.Remove(MotorId); // stale after a recompile: re-resolve
	}

	UMjNodeComponent* Actuator = ResolveActuator(MotorId);
	AMjArticulation*  Art = Articulation.Get();
	if (!Actuator || !Art)
	{
		return nullptr;
	}
	const TOptional<int32>& BoundId = Actuator->GetBoundId();
	if (!BoundId.IsSet() || BoundId.GetValue() < 0)
	{
		return nullptr; // not compiled yet
	}
	const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	const int32				ActId = BoundId.GetValue();
	if (!Model || ActId >= Model->nu)
	{
		return nullptr;
	}

	if (Model->actuator_trntype[ActId] != mjTRN_JOINT)
	{
		if (!WarnedUnsupported.Contains(MotorId))
		{
			WarnedUnsupported.Add(MotorId);
			UE_LOG(LogTemp, Warning,
				TEXT("RammsMujocoActuationBackend: actuator '%s' has a non-joint transmission (type %d); value/velocity/transform reads are unsupported for it (commands still work)."),
				*MotorId.ToString(), static_cast<int32>(Model->actuator_trntype[ActId]));
		}
		return nullptr;
	}

	const int32 JointId = Model->actuator_trnid[2 * ActId];
	if (JointId < 0 || JointId >= Model->njnt)
	{
		return nullptr;
	}
	UMjNodeComponent* Joint = Art->GetComponentByMjId(mjOBJ_JOINT, JointId);
	if (!Joint)
	{
		return nullptr;
	}

	FTransmission Trn;
	Trn.Joint = Joint;
	Trn.BodyId = Model->jnt_bodyid[JointId];
	Trn.bSlide = Model->jnt_type[JointId] == mjJNT_SLIDE;
	// <position>: an affine bias that pulls its length (gear x q) to its ctrl. Not <intvelocity>,
	// which has the same bias but integrates its ctrl (a speed) into that target.
	Trn.bServo = Model->actuator_biastype[ActId] == mjBIAS_AFFINE && Model->actuator_biasprm[ActId * mjNBIAS + 1] != 0
		&& Model->actuator_dyntype[ActId] != mjDYN_INTEGRATOR;
	const double Gear = Model->actuator_gear[6 * ActId];
	if (Trn.bServo && FMath::Abs(Gear) > UE_SMALL_NUMBER)
	{
		Trn.CtrlPerUnit = Gear / (Trn.bSlide ? 100.0 : 1.0);
	}
	return &TransmissionByMotor.Add(MotorId, Trn);
}

void FRammsMujocoActuationBackend::SetCommand(FName MotorId, float Value)
{
	// A command staged directly wins over a profile, which would otherwise
	// stage over it from the physics thread every step.
	ClearPositionProfile(MotorId);
	UMjNodeComponent* Actuator = ResolveActuator(MotorId);
	if (!Actuator)
	{
		return;
	}
	// A position servo takes its target as its joint's position, as GetValue
	// reports it (cm for a slide); its ctrl is gear x that, in metres.
	if (const FTransmission* Trn = ResolveTransmission(MotorId))
	{
		Value *= static_cast<float>(Trn->CtrlPerUnit);
	}
	// Clamp to the compiled actuator's ctrlrange so the staged value matches what
	// the integrator will apply (zero-width range = unlimited, leave as-is).
	const FVector2D Range = UMjActuatorRuntime::GetControlRange(Actuator);
	if (Range.X < Range.Y)
	{
		Value = FMath::Clamp(Value, static_cast<float>(Range.X), static_cast<float>(Range.Y));
	}
	UMjActuatorRuntime::SetControl(Actuator, Value);
}

float FRammsMujocoActuationBackend::GetValue(FName MotorId) const
{
	// The joint's own coordinate (not the gear-scaled actuator length): radians
	// for a hinge, metres -> cm for a slide.
	const FTransmission* Trn = ResolveTransmission(MotorId);
	if (!Trn)
	{
		return 0.0f;
	}
	const float Pos = UMjJointRuntime::GetPosition(Trn->Joint.Get());
	return Trn->bSlide ? Pos * 100.0f : Pos;
}

float FRammsMujocoActuationBackend::GetVelocity(FName MotorId) const
{
	const FTransmission* Trn = ResolveTransmission(MotorId);
	if (!Trn)
	{
		return 0.0f;
	}
	const float Vel = UMjJointRuntime::GetVelocity(Trn->Joint.Get());
	return Trn->bSlide ? Vel * 100.0f : Vel;
}

bool FRammsMujocoActuationBackend::ReleaseMotor(FName MotorId)
{
	// A MuJoCo actuator has no off switch: its ctrl is applied every step. So
	// "release" means staging a ctrl under which the actuator stops acting on
	// the joint — and reporting false whenever that isn't possible, so the
	// caller keeps treating the motor as driven:
	//  - <motor> (force, no bias):   ctrl 0 = zero force              -> released
	//  - <position> (bias kp < 0, no dynamics): ctrl = current actuator length -> parked
	//    where it is (holds; stops pursuing its old target). Read as the
	//    actuator LENGTH — gear- and transmission-aware — not the joint qpos.
	//  - <velocity> (bias kv only):  ctrl 0 would BRAKE the joint     -> false
	//  - anything with actuator dynamics (intvelocity, general):      -> false
	ClearPositionProfile(MotorId);
	UMjNodeComponent* Actuator = ResolveActuator(MotorId);
	if (!Actuator)
	{
		return false;
	}
	const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	const TOptional<int32>& BoundId = Actuator->GetBoundId();
	if (!Model || !BoundId.IsSet() || BoundId.GetValue() < 0 || BoundId.GetValue() >= Model->nu)
	{
		return false; // not compiled / not bound: nothing can be staged
	}
	const int32	  ActId = BoundId.GetValue();
	const mjtNum* Bias = &Model->actuator_biasprm[ActId * mjNBIAS];
	const bool	  bAffine = Model->actuator_biastype[ActId] == mjBIAS_AFFINE;
	// Stateless dynamics only: <intvelocity> also compiles with an affine
	// negative length bias, but its ctrl feeds an integrator — writing the
	// current length there would drive it, not park it.
	const bool bStateless = Model->actuator_dyntype[ActId] == mjDYN_NONE;
	const bool bPositionServo = bStateless && bAffine && Bias[1] < 0.0;
	const bool bPlainForce = Model->actuator_biastype[ActId] == mjBIAS_NONE && Model->actuator_dyntype[ActId] == mjDYN_NONE;
	float	   Park = 0.0f;
	if (bPositionServo)
	{
		Park = UMjActuatorRuntime::GetLength(Actuator);
	}
	else if (!bPlainForce)
	{
		if (!WarnedUnsupported.Contains(MotorId))
		{
			WarnedUnsupported.Add(MotorId);
			UE_LOG(LogTemp, Warning,
				TEXT("RammsMujocoActuationBackend: actuator '%s' cannot be released (a velocity servo / dynamic actuator keeps acting at ctrl 0); it stays driven."),
				*MotorId.ToString());
		}
		return false;
	}
	// A park value the compiled ctrlrange doesn't admit would be clamped into a
	// command that still moves the joint: that is not a release.
	const FVector2D Range = UMjActuatorRuntime::GetControlRange(Actuator);
	if (Range.X < Range.Y && (Park < Range.X - KINDA_SMALL_NUMBER || Park > Range.Y + KINDA_SMALL_NUMBER))
	{
		return false;
	}
	UMjActuatorRuntime::SetControl(Actuator, Park);
	// The staging slot can be gone mid-recompile (SetControl then no-ops):
	// only report released if the value actually landed.
	return FMath::IsNearlyEqual(static_cast<float>(UMjActuatorRuntime::GetControl(Actuator)), Park, 1e-4f);
}

bool FRammsMujocoActuationBackend::GetMotorTransform(FName MotorId, FTransform& OutWorld) const
{
	// <actuator> elements live at the model root, so the actuator node's own
	// transform says nothing about where the motor is; only the body of the
	// joint it drives does. Unavailable (not compiled / non-joint transmission)
	// is reported as such rather than as a bogus root transform.
	const FTransmission* Trn = ResolveTransmission(MotorId);
	AMjArticulation*	 Art = Articulation.Get();
	if (!Trn || !Art || Trn->BodyId < 0)
	{
		return false;
	}
	UMjBody* Body = Art->GetBodyByMjId(Trn->BodyId);
	if (!Body)
	{
		return false;
	}
	OutWorld = Body->GetComponentTransform();
	return true;
}

FRammsMujocoActuationBackend::~FRammsMujocoActuationBackend()
{
	if (DriveTable)
	{
		FScopeLock Lock(&DriveTable->Lock);
		DriveTable->bAlive = false;
		DriveTable->Drives.Reset();
		DriveTable->Profiles.Reset();
		DriveTable->Articulation = nullptr;
		// Kept for the callback to put back the friction they raised, and the
		// force ranges the torque limits narrowed.
		for (TPair<FName, FPhysicsBrake>& Pair : DriveTable->Brakes)
		{
			Pair.Value.bEngaged = false;
		}
		for (TPair<FName, FPhysicsCap>& Pair : DriveTable->Caps)
		{
			Pair.Value.bRemove = true;
		}
		DriveTable->Efforts.Reset();
	}
}

bool FRammsMujocoActuationBackend::SetVelocityDrive(FName MotorId, float TargetRate, const FRammsVelocityGains& Gains,
	float TorqueMin, float TorqueMax)
{
	// False hands the loop back to the base, which then commands the motor once
	// per tick. A loop this backend was already running would otherwise stay in
	// the table and keep staging its old output from the physics thread every
	// step, over the base's command, so every refusal drops it first.
	const auto Refuse = [this, MotorId]() -> bool {
		ClearVelocityDrive(MotorId);
		return false;
	};
	UMjNodeComponent*	 Actuator = ResolveActuator(MotorId);
	const FTransmission* Transmission = ResolveTransmission(MotorId);
	if (!Actuator || !Transmission || !Articulation.IsValid())
	{
		return Refuse(); // not compiled yet: the base runs the loop meanwhile
	}
	const TOptional<int32>& Bound = Actuator->GetBoundId();
	UMjPhysicsEngine*		Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	if (!Bound.IsSet() || !Model)
	{
		return Refuse();
	}
	const int32 ActuatorId = Bound.GetValue();
	if (ActuatorId < 0 || ActuatorId >= Model->nu || Model->actuator_trntype[ActuatorId] != mjTRN_JOINT)
	{
		return Refuse();
	}
	const int32 Joint = Model->actuator_trnid[2 * ActuatorId];
	if (Joint < 0 || Joint >= Model->njnt)
	{
		return Refuse();
	}

	FDriveTable& Table = EnsureDriveTable(*Engine);
	{
		FScopeLock Lock(&Table.Lock);
		// A wheel driven at a speed is not also braked.
		if (FPhysicsBrake* Brake = Table.Brakes.Find(MotorId))
		{
			Brake->bEngaged = false;
		}
		FPhysicsDrive& Drive = Table.Drives.FindOrAdd(MotorId);
		if (Drive.ActuatorId != ActuatorId)
		{
			Drive = FPhysicsDrive(); // a recompile can move the ids
		}
		// A retune starts the integral over, as the base's own loop does: torque
		// accumulated under the old gains would be applied at once under the new
		// ones (and never bleed off with Ki 0). The same gains arrive every tick,
		// so this only fires on a real change.
		if (Drive.Kp != Gains.Kp || Drive.Ki != Gains.Ki || Drive.MaxIntegral != Gains.MaxIntegralTorque
			|| Drive.StiffKdGiven != Gains.StiffDamping || Drive.StiffKiGiven != Gains.StiffIntegral)
		{
			Drive.Integral = 0.0;
		}
		Table.Articulation = Articulation.Get();
		Drive.ActuatorId = ActuatorId;
		Drive.DofAdr = Model->jnt_dofadr[Joint];
		Drive.Scale = Transmission->bSlide ? 100.0 : 1.0;
		Drive.Target = TargetRate;
		Drive.Kp = Gains.Kp;
		Drive.Ki = Gains.Ki;
		Drive.MaxIntegral = Gains.MaxIntegralTorque;
		Drive.StiffKdGiven = Gains.StiffDamping;
		Drive.StiffKiGiven = Gains.StiffIntegral;
		Drive.Lo = TorqueMin;
		Drive.Hi = TorqueMax;
		// A stiff loop: as the motor's gains ask (ramms.mj.StiffDrive 1), none
		// (0), or -- experiments, 2 -- the motors named in StiffDriveMotors at
		// the setting's gains.
		const int32 StiffMode = CVarRammsMjStiffDrive.GetValueOnGameThread();
		Drive.StiffKd = StiffMode == 0 ? 0.0 : Gains.StiffDamping;
		Drive.StiffKi = StiffMode == 0 ? 0.0 : Gains.StiffIntegral;
		if (StiffMode == 2)
		{
			TArray<FString> Patterns;
			CVarRammsMjStiffDriveMotors.GetValueOnGameThread().ParseIntoArray(Patterns, TEXT(","), true);
			const FString Name = MotorId.ToString();
			bool		  bNamed = Patterns.Num() == 0;
			for (const FString& Pattern : Patterns)
			{
				bNamed = bNamed || Name.Contains(Pattern.TrimStartAndEnd(), ESearchCase::IgnoreCase);
			}
			if (bNamed)
			{
				Drive.StiffKd = CVarRammsMjStiffDriveDamping.GetValueOnGameThread();
				Drive.StiffKi = CVarRammsMjStiffDriveIntegral.GetValueOnGameThread();
			}
		}
	}
	return true;
}

void FRammsMujocoActuationBackend::ClearVelocityDrive(FName MotorId)
{
	bool bHad = false;
	if (DriveTable)
	{
		FScopeLock Lock(&DriveTable->Lock);
		bHad = DriveTable->Drives.Remove(MotorId) > 0;
	}
	// What the loop staged last would otherwise go on acting until the motor
	// is commanded again: its torque, or -- a stiff loop's -- a speed, which
	// is a torque again once the actuator is put back as a plain motor (a
	// braked wheel held its loop's last 50 N*m against the brake).
	if (bHad)
	{
		if (UMjNodeComponent* Actuator = ResolveActuator(MotorId))
		{
			UMjActuatorRuntime::SetControl(Actuator, 0.0f);
		}
	}
}

bool FRammsMujocoActuationBackend::SetPositionProfile(FName MotorId, float Target, float MaxVelocity, float MaxAcceleration,
	float StartFrom)
{
	// False hands the profile back to the base, which then steps it once per
	// tick; one this backend was running would otherwise stage over the base's
	// setpoint every step, so every refusal drops it first.
	const auto Refuse = [this, MotorId]() -> bool {
		ClearPositionProfile(MotorId);
		return false;
	};
	UMjNodeComponent*	 Actuator = ResolveActuator(MotorId);
	const FTransmission* Transmission = ResolveTransmission(MotorId);
	if (!Actuator || !Transmission || !Articulation.IsValid())
	{
		return Refuse(); // not compiled yet: the base eases it meanwhile
	}
	const TOptional<int32>& Bound = Actuator->GetBoundId();
	UMjPhysicsEngine*		Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	if (!Bound.IsSet() || !Model)
	{
		return Refuse();
	}
	const int32 ActuatorId = Bound.GetValue();
	if (ActuatorId < 0 || ActuatorId >= Model->nu || Model->actuator_trntype[ActuatorId] != mjTRN_JOINT)
	{
		return Refuse();
	}
	const int32 Joint = Model->actuator_trnid[2 * ActuatorId];
	if (Joint < 0 || Joint >= Model->njnt)
	{
		return Refuse();
	}
	// Clamped as SetCommand clamps what it stages.
	const FVector2D Range = UMjActuatorRuntime::GetControlRange(Actuator);

	FDriveTable&	 Table = EnsureDriveTable(*Engine);
	FScopeLock		 Lock(&Table.Lock);
	FPhysicsProfile& Profile = Table.Profiles.FindOrAdd(MotorId);
	if (Profile.ActuatorId != ActuatorId)
	{
		Profile = FPhysicsProfile(); // a recompile can move the ids
	}
	Table.Articulation = Articulation.Get();
	Profile.ActuatorId = ActuatorId;
	Profile.QposAdr = Model->jnt_qposadr[Joint];
	Profile.Scale = Transmission->bSlide ? 100.0 : 1.0;
	Profile.CtrlPerUnit = Transmission->bServo ? Transmission->CtrlPerUnit : 1.0 / Profile.Scale;
	Profile.Lo = Range.X / Profile.CtrlPerUnit;
	Profile.Hi = Range.Y / Profile.CtrlPerUnit;
	if (Profile.CtrlPerUnit < 0.0)
	{
		Swap(Profile.Lo, Profile.Hi); // a negative gear reverses the bounds
	}
	Profile.MaxVelocity = MaxVelocity;
	Profile.MaxAcceleration = MaxAcceleration;
	Profile.Target = Target;
	if (!Profile.bStarted && FMath::IsFinite(StartFrom))
	{
		Profile.StartFrom = StartFrom;
		Profile.bStartGiven = true;
	}
	++Profile.Commands;
	return true;
}

void FRammsMujocoActuationBackend::ClearPositionProfile(FName MotorId)
{
	if (DriveTable)
	{
		FScopeLock Lock(&DriveTable->Lock);
		DriveTable->Profiles.Remove(MotorId);
	}
}

bool FRammsMujocoActuationBackend::ResolveBrake(FName MotorId, UMjPhysicsEngine*& OutEngine, const mjModel_*& OutModel, int32& OutDofAdr,
	double& OutTorque) const
{
	UMjNodeComponent*	 Actuator = ResolveActuator(MotorId);
	const FTransmission* Transmission = ResolveTransmission(MotorId);
	if (!Actuator || !Transmission || !Articulation.IsValid())
	{
		return false;
	}
	const TOptional<int32>& Bound = Actuator->GetBoundId();
	UMjPhysicsEngine*		Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	if (!Bound.IsSet() || !Model)
	{
		return false;
	}
	const int32 ActuatorId = Bound.GetValue();
	if (ActuatorId < 0 || ActuatorId >= Model->nu || Model->actuator_trntype[ActuatorId] != mjTRN_JOINT)
	{
		return false;
	}
	const int32 Joint = Model->actuator_trnid[2 * ActuatorId];
	if (Joint < 0 || Joint >= Model->njnt || (Model->jnt_type[Joint] != mjJNT_HINGE && Model->jnt_type[Joint] != mjJNT_SLIDE))
	{
		return false;
	}
	// A motor: its force its gain times its command, nothing more. A servo
	// (position, velocity) or an actuator with dynamics holds by its own law.
	// (A motor a stiff speed loop has made a servo for now is judged as it was
	// authored: it is put back before the brake takes it.)
	int32 GainType = Model->actuator_gaintype[ActuatorId];
	int32 BiasType = Model->actuator_biastype[ActuatorId];
	if (DriveTable)
	{
		FScopeLock Lock(&DriveTable->Lock);
		if (DriveTable->StiffenedIn == Model)
		{
			if (const FStiffened* Was = DriveTable->Stiffened.Find(ActuatorId))
			{
				GainType = Was->GainType;
				BiasType = Was->BiasType;
			}
		}
	}
	if (GainType != mjGAIN_FIXED || BiasType != mjBIAS_NONE || Model->actuator_dyntype[ActuatorId] != mjDYN_NONE)
	{
		return false;
	}
	// It holds with its own rating (FRammsMotorSpec::BrakeTorque), else with
	// what the motor can do about its joint: a steeper slope rolls the robot.
	FRammsMotorSpec					Spec;
	const URammsRobotBaseComponent* Base = BaseComponent.Get();
	if (Base && Base->GetMotorSpec(MotorId, Spec) && Spec.BrakeTorque > 0.0f)
	{
		OutEngine = Engine;
		OutModel = Model;
		OutDofAdr = Model->jnt_dofadr[Joint];
		OutTorque = Spec.BrakeTorque;
		return true;
	}
	const mjtNum* Range = &Model->actuator_ctrlrange[2 * ActuatorId];
	const mjtNum* Force = &Model->actuator_forcerange[2 * ActuatorId];
	double		  Most = 0.0;
	if (Model->actuator_ctrllimited[ActuatorId])
	{
		Most = FMath::Max(FMath::Abs(Range[0]), FMath::Abs(Range[1])) * FMath::Abs(Model->actuator_gainprm[ActuatorId * mjNGAIN]);
	}
	if (Model->actuator_forcelimited[ActuatorId])
	{
		const double Limit = FMath::Max(FMath::Abs(Force[0]), FMath::Abs(Force[1]));
		Most = Most > 0.0 ? FMath::Min(Most, Limit) : Limit;
	}
	Most *= FMath::Abs(Model->actuator_gear[6 * ActuatorId]);
	if (Model->jnt_actfrclimited[Joint])
	{
		const mjtNum* Joints = &Model->jnt_actfrcrange[2 * Joint];
		const double  Limit = FMath::Max(FMath::Abs(Joints[0]), FMath::Abs(Joints[1]));
		Most = Most > 0.0 ? FMath::Min(Most, Limit) : Limit;
	}
	// An unlimited motor: a brake that holds whatever the robot puts on it.
	constexpr double UnlimitedHoldNm = 1000.0;
	OutEngine = Engine;
	OutModel = Model;
	OutDofAdr = Model->jnt_dofadr[Joint];
	OutTorque = Most > 0.0 ? Most : UnlimitedHoldNm;
	return true;
}

bool FRammsMujocoActuationBackend::HasMotorBrake(FName MotorId) const
{
	UMjPhysicsEngine* Engine = nullptr;
	const mjModel*	  Model = nullptr;
	int32			  DofAdr = INDEX_NONE;
	double			  Torque = 0.0;
	return ResolveBrake(MotorId, Engine, Model, DofAdr, Torque);
}

bool FRammsMujocoActuationBackend::SetMotorBrake(FName MotorId, bool bEngaged)
{
	UMjPhysicsEngine* Engine = nullptr;
	const mjModel*	  Model = nullptr;
	int32			  DofAdr = INDEX_NONE;
	double			  Torque = 0.0;
	const bool		  bHas = ResolveBrake(MotorId, Engine, Model, DofAdr, Torque);
	if (!bEngaged)
	{
		if (DriveTable)
		{
			FScopeLock Lock(&DriveTable->Lock);
			if (FPhysicsBrake* Brake = DriveTable->Brakes.Find(MotorId))
			{
				Brake->bEngaged = false; // the callback puts the friction back
			}
		}
		return bHas;
	}
	if (!bHas)
	{
		return false;
	}
	FDriveTable& Table = EnsureDriveTable(*Engine);
	FScopeLock	 Lock(&Table.Lock);
	// A braked wheel is not also driven at a speed.
	Table.Drives.Remove(MotorId);
	Table.Articulation = Articulation.Get();
	FPhysicsBrake& Brake = Table.Brakes.FindOrAdd(MotorId);
	Brake.Model = Model;
	Brake.DofAdr = DofAdr;
	Brake.Torque = Torque;
	Brake.bEngaged = true;
	return true;
}

FRammsMujocoActuationBackend::FDriveTable& FRammsMujocoActuationBackend::EnsureDriveTable(UMjPhysicsEngine& Engine) const
{
	if (!DriveTable)
	{
		DriveTable = MakeShared<FDriveTable, ESPMode::ThreadSafe>();
	}
	if (!bDriveCallbackRegistered)
	{
		TSharedPtr<FDriveTable, ESPMode::ThreadSafe> Table = DriveTable;
		Engine.RegisterPreStepCallback([Table](mjModel* M, mjData* D) { StepTable(*Table, M, D); });
		bDriveCallbackRegistered = true;
	}
	return *DriveTable;
}

void FRammsMujocoActuationBackend::StepTable(FDriveTable& Table, mjModel_* Model, const mjData_* Data)
{
	FScopeLock Lock(&Table.Lock);
	if (!Model || !Data)
	{
		return;
	}
	// The time since the last callback, on the simulation's own clock: none
	// while it is paused (the callbacks still run then, and a loop integrating
	// a fixed timestep would wind up) and the whole batch when one request
	// steps several. A clock gone back is a reset: the robot is back at its
	// start, so every loop and profile starts over, and the efforts recorded
	// before it go.
	const double Now = Data->time;
	double		 Dt = 0.0;
	if (Table.LastTime >= 0.0 && Now < Table.LastTime)
	{
		for (TPair<FName, FPhysicsDrive>& Pair : Table.Drives)
		{
			Pair.Value.Integral = 0.0;
		}
		for (TPair<FName, FPhysicsProfile>& Pair : Table.Profiles)
		{
			Pair.Value.bStarted = false;
		}
		for (TPair<FName, FPhysicsEffort>& Pair : Table.Efforts)
		{
			Pair.Value.Count = 0;
			Pair.Value.Head = 0;
		}
	}
	else if (Table.LastTime >= 0.0)
	{
		Dt = Now - Table.LastTime;
	}
	Table.LastTime = Now;
	// The forces of the step just finished, before the torque limits change
	// the ranges: a sample's limits are those its step ran under.
	if (Table.bAlive && Dt > 0.0)
	{
		StepEfforts(Table, Model, Data, Dt);
	}
	// Then, on a dead table too: they still put back what they raised and
	// narrowed (and what the stiff loops made servos).
	StepBrakes(Table, Model);
	StepCaps(Table, Model);
	if (!Table.bAlive || !Table.Articulation)
	{
		RestoreStiffened(Table, Model, nullptr);
		return;
	}
	StepDrives(Table, Model, Data, Dt);
	StepProfiles(Table, Model, Data, Dt);
}

void FRammsMujocoActuationBackend::StepBrakes(FDriveTable& Table, mjModel_* Model)
{
	// MuJoCo makes its constraints afresh every step from the model, so a
	// joint's friction loss raised before the step holds it in that step's
	// solve: dry friction up to the brake's torque, together with the
	// contacts, at the physics rate. A brake resolved against another model
	// (a recompile since) holds nothing until it is set again.
	//
	// The friction is made hard too. MuJoCo's constraints are soft: under a
	// steady load one slides at about the load times its regulariser over its
	// damping, and the regulariser scales with the inverse of the inertia the
	// dof sees -- a wheel's own spin inertia, small -- so at the default
	// impedance (0.9) a braked wheel carrying a share of a robot on a slope
	// slid at tenths of a rad/s. At 0.999 over a time constant of two steps it
	// slides about a thousand times slower.
	static_assert(sizeof(FPhysicsBrake::OriginalRef) == mjNREF * sizeof(mjtNum), "a dof's solref");
	static_assert(sizeof(FPhysicsBrake::OriginalImp) == mjNIMP * sizeof(mjtNum), "a dof's solimp");
	constexpr double BrakeImpedance = 0.999;
	const double	 BrakeTimeConst = 2.0 * Model->opt.timestep;
	for (auto It = Table.Brakes.CreateIterator(); It; ++It)
	{
		FPhysicsBrake& Brake = It.Value();
		const bool	   bHold = Table.bAlive && Brake.bEngaged && Brake.Model == Model && Brake.DofAdr >= 0 && Brake.DofAdr < Model->nv;
		mjtNum* const  Ref = bHold || Brake.RaisedIn == Model ? &Model->dof_solref[Brake.DofAdr * mjNREF] : nullptr;
		mjtNum* const  Imp = Ref ? &Model->dof_solimp[Brake.DofAdr * mjNIMP] : nullptr;
		if (bHold)
		{
			if (Brake.RaisedIn != Model)
			{
				Brake.Original = Model->dof_frictionloss[Brake.DofAdr];
				FMemory::Memcpy(Brake.OriginalRef, Ref, sizeof(Brake.OriginalRef));
				FMemory::Memcpy(Brake.OriginalImp, Imp, sizeof(Brake.OriginalImp));
				Brake.RaisedIn = Model;
			}
			Model->dof_frictionloss[Brake.DofAdr] = FMath::Max(Brake.Original, Brake.Torque);
			Ref[0] = BrakeTimeConst;
			Ref[1] = 1.0;
			Imp[0] = Imp[1] = BrakeImpedance; // a friction constraint sits at the low end: no violation
			continue;
		}
		if (Brake.RaisedIn == Model)
		{
			Model->dof_frictionloss[Brake.DofAdr] = Brake.Original;
			FMemory::Memcpy(Ref, Brake.OriginalRef, sizeof(Brake.OriginalRef));
			FMemory::Memcpy(Imp, Brake.OriginalImp, sizeof(Brake.OriginalImp));
		}
		Brake.RaisedIn = nullptr;
		if (!Brake.bEngaged)
		{
			It.RemoveCurrent();
		}
	}
}

void FRammsMujocoActuationBackend::StepProfiles(FDriveTable& Table, const mjModel_* Model, const mjData_* Data, double Dt)
{
	// Each profile is stepped over the time since the last callback and its
	// setpoint staged for the step about to run; a command is taken as given at
	// the start of that time.
	const double From = Data->time - Dt;
	for (TPair<FName, FPhysicsProfile>& Pair : Table.Profiles)
	{
		FPhysicsProfile& Profile = Pair.Value;
		if (Profile.ActuatorId < 0 || Profile.ActuatorId >= Model->nu || Profile.QposAdr < 0 || Profile.QposAdr >= Model->nq)
		{
			continue;
		}
		if (!Profile.bStarted)
		{
			// Where the base says (where the joint stood when it was first
			// commanded), else -- after a reset -- where the joint is now.
			Profile.Follower.Reset(Profile.bStartGiven ? Profile.StartFrom : Data->qpos[Profile.QposAdr] * Profile.Scale);
			Profile.bStartGiven = false;
			Profile.bStarted = true;
			Profile.SeenCommands = Profile.Commands - 1;
		}
		if (Profile.SeenCommands != Profile.Commands)
		{
			Profile.Follower.Command(Profile.Target, From);
			Profile.SeenCommands = Profile.Commands;
		}
		double Setpoint = Profile.Follower.Step(From, Dt, Profile.MaxVelocity, Profile.MaxAcceleration);
		if (Profile.Lo < Profile.Hi)
		{
			Setpoint = FMath::Clamp(Setpoint, Profile.Lo, Profile.Hi);
		}
		// The setpoint's rate fed forward (ramms.mj.ServoFeedForward): a position
		// servo's force is kp (ctrl - q) - kv qdot, so a command led by kv / kp of
		// the setpoint's rate makes it kp (s - q) + kv (sdot - qdot) -- no lag
		// behind a setpoint moving steadily, the stiffness unchanged. (Only for a
		// plain position servo: fixed gain, affine bias of -kp and -kv.)
		double Command = Setpoint;
		if (CVarRammsMjServoFeedForward.GetValueOnAnyThread() != 0 && Model->actuator_gaintype[Profile.ActuatorId] == mjGAIN_FIXED
			&& Model->actuator_biastype[Profile.ActuatorId] == mjBIAS_AFFINE)
		{
			const mjtNum Kp = Model->actuator_gainprm[Profile.ActuatorId * mjNGAIN];
			const mjtNum Kv = -Model->actuator_biasprm[Profile.ActuatorId * mjNBIAS + 2];
			if (Kp > 0.0 && Kv > 0.0 && FMath::IsNearlyEqual(Model->actuator_biasprm[Profile.ActuatorId * mjNBIAS + 1], -Kp, 1e-6 * Kp))
			{
				Command += Kv / Kp * Profile.Follower.Rate;
			}
		}
		Table.Articulation->StageInternalControl(Profile.ActuatorId, Command * Profile.CtrlPerUnit);
	}
}

void FRammsMujocoActuationBackend::RestoreStiffened(FDriveTable& Table, mjModel_* Model, const TSet<int32>* Keep)
{
	if (Table.StiffenedIn != Model)
	{
		// Made servos in a model since recompiled: the new one has its own.
		Table.Stiffened.Reset();
		Table.StiffenedIn = Model;
		return;
	}
	for (auto It = Table.Stiffened.CreateIterator(); It; ++It)
	{
		const int32 Id = It.Key();
		if ((Keep && Keep->Contains(Id)) || Id < 0 || Id >= Model->nu)
		{
			continue;
		}
		const FStiffened& Was = It.Value();
		Model->actuator_gaintype[Id] = Was.GainType;
		Model->actuator_biastype[Id] = Was.BiasType;
		FMemory::Memcpy(&Model->actuator_gainprm[Id * mjNGAIN], Was.GainPrm, sizeof(Was.GainPrm));
		FMemory::Memcpy(&Model->actuator_biasprm[Id * mjNBIAS], Was.BiasPrm, sizeof(Was.BiasPrm));
		Model->actuator_forcelimited[Id] = Was.ForceLimited;
		Model->actuator_forcerange[2 * Id] = Was.ForceRange[0];
		Model->actuator_forcerange[2 * Id + 1] = Was.ForceRange[1];
		Model->actuator_ctrllimited[Id] = Was.CtrlLimited;
		Model->actuator_ctrlrange[2 * Id] = Was.CtrlRange[0];
		Model->actuator_ctrlrange[2 * Id + 1] = Was.CtrlRange[1];
		It.RemoveCurrent();
	}
}

void FRammsMujocoActuationBackend::StepDrives(FDriveTable& Table, mjModel_* Model, const mjData_* Data, double Dt)
{
	static_assert(sizeof(FStiffened::GainPrm) == mjNGAIN * sizeof(mjtNum), "an actuator's gain parameters");
	static_assert(sizeof(FStiffened::BiasPrm) == mjNBIAS * sizeof(mjtNum), "an actuator's bias parameters");
	// (ramms.mj.StiffDrive) Each loop on a plain force actuator made a velocity
	// servo for the step: force = Kd/g^2 (ctrl - g qvel), with ctrl the joint's
	// target rate times its gear g plus the integral's share. The solver
	// (implicitfast) takes the velocity term implicitly, so the damping can be
	// as stiff as a geared drive: a lifted omni wheel's inertia (~0.03 kg*m^2)
	// sets a ceiling of about 25 N*m per rad/s on a loop closed outside it.
	if (Table.StiffenedIn != Model)
	{
		RestoreStiffened(Table, Model, nullptr);
	}
	TSet<int32> Stiff;
	for (TPair<FName, FPhysicsDrive>& Pair : Table.Drives)
	{
		FPhysicsDrive& Drive = Pair.Value;
		if (Drive.ActuatorId < 0 || Drive.ActuatorId >= Model->nu || Drive.DofAdr < 0 || Drive.DofAdr >= Model->nv)
		{
			continue;
		}
		const int32	 Id = Drive.ActuatorId;
		const double Gear = Model->actuator_gear[6 * Id];
		const bool	 bOurs = Table.Stiffened.Contains(Id);
		const bool	 bPlainMotor = Model->actuator_gaintype[Id] == mjGAIN_FIXED && Model->actuator_biastype[Id] == mjBIAS_NONE
			&& Model->actuator_dyntype[Id] == mjDYN_NONE;
		if (Drive.StiffKd > 0.0 && FMath::Abs(Gear) > 1e-9 && (bOurs || bPlainMotor))
		{
			if (!bOurs)
			{
				FStiffened& Was = Table.Stiffened.Add(Id);
				Was.GainType = Model->actuator_gaintype[Id];
				Was.BiasType = Model->actuator_biastype[Id];
				FMemory::Memcpy(Was.GainPrm, &Model->actuator_gainprm[Id * mjNGAIN], sizeof(Was.GainPrm));
				FMemory::Memcpy(Was.BiasPrm, &Model->actuator_biasprm[Id * mjNBIAS], sizeof(Was.BiasPrm));
				Was.ForceLimited = Model->actuator_forcelimited[Id];
				Was.ForceRange[0] = Model->actuator_forcerange[2 * Id];
				Was.ForceRange[1] = Model->actuator_forcerange[2 * Id + 1];
				Was.CtrlLimited = Model->actuator_ctrllimited[Id];
				Was.CtrlRange[0] = Model->actuator_ctrlrange[2 * Id];
				Was.CtrlRange[1] = Model->actuator_ctrlrange[2 * Id + 1];
			}
			Stiff.Add(Id);
			// The torque range: the loop's (the motor's own), else the force
			// the plain motor could put out (its gain of 1 times its ctrlrange).
			FStiffened& Was = Table.Stiffened[Id];
			double		Lo = Drive.Lo, Hi = Drive.Hi;
			if (!(Lo < Hi))
			{
				Lo = Was.CtrlLimited ? Was.CtrlRange[0] * Was.GainPrm[0] : -1e9;
				Hi = Was.CtrlLimited ? Was.CtrlRange[1] * Was.GainPrm[0] : 1e9;
				if (Hi < Lo)
				{
					Swap(Lo, Hi);
				}
			}
			// (For the efforts: the motor's rating while the loop has it.)
			Was.bLoopRange = Lo > -1e8 || Hi < 1e8;
			Was.LoopRange[0] = Lo;
			Was.LoopRange[1] = Hi;
			const double StiffKd = FMath::Max(1.0, Drive.StiffKd);
			const double StiffKi = FMath::Max(0.0, Drive.StiffKi);
			const double KdA = StiffKd / (Gear * Gear);
			// The joint's rate and its target (rad/s; qvel units), and the
			// integral in the actuator's force: held while the force is at
			// its limit, so that it does not wind up.
			const double Qd = Data->qvel[Drive.DofAdr];
			const double QdTarget = Drive.Target / Drive.Scale;
			double		 Integral = Drive.Integral + StiffKi / Gear * (QdTarget - Qd) * Dt;
			if (Drive.MaxIntegral > 0.0)
			{
				Integral = FMath::Clamp(Integral, -Drive.MaxIntegral, Drive.MaxIntegral);
			}
			const double Force = KdA * Gear * (QdTarget - Qd) + Integral;
			const bool	 bSaturated = Force < Lo || Force > Hi;
			if (bSaturated)
			{
				Integral = Drive.Integral;
			}
			Drive.Integral = Integral;
			mjtNum* const GainPrm = &Model->actuator_gainprm[Id * mjNGAIN];
			mjtNum* const BiasPrm = &Model->actuator_biasprm[Id * mjNBIAS];
			if (bSaturated)
			{
				// At the motor's limit the servo would put out a constant force
				// that the solver still took for a damper (its derivative), and
				// the wheel chattered from one limit to the other: for this step
				// the plain motor, at the limit.
				Model->actuator_gaintype[Id] = Was.GainType;
				Model->actuator_biastype[Id] = Was.BiasType;
				FMemory::Memcpy(GainPrm, Was.GainPrm, sizeof(Was.GainPrm));
				FMemory::Memcpy(BiasPrm, Was.BiasPrm, sizeof(Was.BiasPrm));
				Model->actuator_forcelimited[Id] = Was.ForceLimited;
				Model->actuator_forcerange[2 * Id] = Was.ForceRange[0];
				Model->actuator_forcerange[2 * Id + 1] = Was.ForceRange[1];
				Model->actuator_ctrllimited[Id] = Was.CtrlLimited;
				const double Gain = FMath::Abs(Was.GainPrm[0]) > 1e-12 ? Was.GainPrm[0] : 1.0;
				Table.Articulation->StageInternalControl(Id, FMath::Clamp(Force, Lo, Hi) / Gain);
				continue;
			}
			Model->actuator_gaintype[Id] = mjGAIN_FIXED;
			Model->actuator_biastype[Id] = mjBIAS_AFFINE;
			FMemory::Memzero(GainPrm, mjNGAIN * sizeof(mjtNum));
			FMemory::Memzero(BiasPrm, mjNBIAS * sizeof(mjtNum));
			GainPrm[0] = KdA;
			BiasPrm[2] = -KdA;
			Model->actuator_forcelimited[Id] = true;
			Model->actuator_forcerange[2 * Id] = Lo;
			Model->actuator_forcerange[2 * Id + 1] = Hi;
			Model->actuator_ctrllimited[Id] = false;
			Table.Articulation->StageInternalControl(Id, Gear * QdTarget + Integral / KdA);
			continue;
		}
		// The same PI the base runs, in the joint's sense: integral capped,
		// held while the output is saturated so it does not wind up.
		const double Error = Drive.Target - Data->qvel[Drive.DofAdr] * Drive.Scale;
		double		 Integral = Drive.Integral + Drive.Ki * Error * Dt;
		if (Drive.MaxIntegral > 0.0)
		{
			Integral = FMath::Clamp(Integral, -Drive.MaxIntegral, Drive.MaxIntegral);
		}
		double Output = Drive.Kp * Error + Integral;
		if (Drive.Lo < Drive.Hi)
		{
			const double Clamped = FMath::Clamp(Output, Drive.Lo, Drive.Hi);
			if (Clamped != Output)
			{
				Integral = Drive.Integral;
			}
			Output = Clamped;
		}
		Drive.Integral = Integral;
		// Staged, not written to mjData: URLab's ApplyControls copies staged
		// controls into ctrl right after the pre-step callbacks, so a direct
		// write would be overwritten in the same step.
		Table.Articulation->StageInternalControl(Drive.ActuatorId, Output);
	}
	// A loop dropped (released, braked, refused, or the setting off): its
	// actuator a plain force actuator again before this step, so the base's own
	// command means a torque once more.
	RestoreStiffened(Table, Model, &Stiff);
}

double FRammsMujocoActuationBackend::GetSimulationTime() const
{
	// The time of the snapshot URLab last published, not live mjData::time:
	// the worker advances that mid-step, under a lock it holds for the whole
	// step, and RK4 writes its stage times there and then rewinds, so an
	// unlocked read can see time run backwards, which a controller takes for a
	// reset. GetValue and GetVelocity read the same published snapshots
	// (URLab's joint accessors do), so the clock, like the measurements, is
	// always the state at the end of a step. A double, unlike the manager's
	// float, which loses the millisecond a controller differentiates against
	// after a few hours.
	const UMjPhysicsEngine* Engine = Articulation.IsValid() ? AAMjManager::ResolveEngine(Articulation.Get()) : nullptr;
	if (!Engine)
	{
		return -1.0;
	}
	double Time = -1.0;
	Engine->WithRenderState([&Time](const FMjRenderSnapshot& Snapshot) {
		if (Snapshot.FrameId > 0) // nothing is published before the first compile
		{
			Time = Snapshot.SimTime;
		}
	});
	return Time;
}

bool FRammsMujocoActuationBackend::ResolveJointActuator(FName MotorId, UMjPhysicsEngine*& OutEngine, const mjModel_*& OutModel,
	int32& OutActuatorId) const
{
	UMjNodeComponent* Actuator = ResolveActuator(MotorId);
	if (!Actuator || !Articulation.IsValid())
	{
		return false;
	}
	const TOptional<int32>& Bound = Actuator->GetBoundId();
	UMjPhysicsEngine*		Engine = AAMjManager::ResolveEngine(Actuator);
	const mjModel*			Model = Engine ? Engine->GetModel() : nullptr;
	if (!Bound.IsSet() || !Model)
	{
		return false;
	}
	const int32 Id = Bound.GetValue();
	if (Id < 0 || Id >= Model->nu || Model->actuator_trntype[Id] != mjTRN_JOINT)
	{
		return false;
	}
	OutEngine = Engine;
	OutModel = Model;
	OutActuatorId = Id;
	return true;
}

namespace
{
	/** An actuator's force range in its joint's sense (x its gear, the ends
	 *  in order); unbounded where it has none. */
	void JointForceRange(bool bLimited, double Lo, double Hi, double Gear, double& OutLo, double& OutHi)
	{
		if (!bLimited)
		{
			OutLo = -TNumericLimits<double>::Max();
			OutHi = TNumericLimits<double>::Max();
			return;
		}
		OutLo = Lo * Gear;
		OutHi = Hi * Gear;
		if (OutLo > OutHi)
		{
			Swap(OutLo, OutHi);
		}
	}

	/** Within the at-limit tolerance (0.5%) of a range's end. */
	bool AtLower(double F, double Lo)
	{
		return F <= Lo + 0.005 * FMath::Abs(Lo) + 1e-9;
	}
	bool AtUpper(double F, double Hi)
	{
		return F >= Hi - 0.005 * FMath::Abs(Hi) - 1e-9;
	}
} // namespace

bool FRammsMujocoActuationBackend::GetMotorEffort(FName MotorId, double WindowSeconds, FRammsMotorEffort& Out) const
{
	UMjPhysicsEngine* Engine = nullptr;
	const mjModel*	  Model = nullptr;
	int32			  Id = INDEX_NONE;
	if (!ResolveJointActuator(MotorId, Engine, Model, Id))
	{
		return false;
	}
	FDriveTable&	Table = EnsureDriveTable(*Engine);
	FScopeLock		Lock(&Table.Lock);
	FPhysicsEffort& E = Table.Efforts.FindOrAdd(MotorId);
	if (E.ActuatorId != Id || E.Model != Model || E.Ring.Num() != EffortSamples)
	{
		// The first read (or one after a recompile) starts the recording.
		E.ActuatorId = Id;
		E.Model = Model;
		E.Gear = Model->actuator_gear[6 * Id];
		E.Ring.SetNumZeroed(EffortSamples);
		E.Head = 0;
		E.Count = 0;
		return false;
	}
	if (E.Count == 0)
	{
		return false;
	}
	Out = FRammsMotorEffort();
	const FEffortSample& Last = E.Ring[(E.Head - 1 + EffortSamples) % EffortSamples];
	Out.LastNm = Last.Force;
	Out.PeakNm = Last.Force;
	Out.LatestTime = Last.Time;
	// Back from the last sample over the window (at least the last one).
	double Weighted = 0.0;
	for (int32 k = 0; k < E.Count; ++k)
	{
		const FEffortSample& Sample = E.Ring[(E.Head - 1 - k + 2 * EffortSamples) % EffortSamples];
		if (k > 0 && Sample.Time <= Last.Time - WindowSeconds)
		{
			break;
		}
		Weighted += static_cast<double>(Sample.Force) * Sample.Dt;
		Out.Seconds += Sample.Dt;
		Out.AtLowerSeconds += (Sample.Flags & 1) ? Sample.Dt : 0.0;
		Out.AtUpperSeconds += (Sample.Flags & 2) ? Sample.Dt : 0.0;
		Out.AtRatedSeconds += (Sample.Flags & 4) ? Sample.Dt : 0.0;
		if (FMath::Abs(Sample.Force) > FMath::Abs(Out.PeakNm))
		{
			Out.PeakNm = Sample.Force;
		}
	}
	Out.MeanNm = Out.Seconds > 0.0 ? Weighted / Out.Seconds : Last.Force;
	// The limits as they stand: applied now, and the actuator's own.
	bool   bLimited = false, bRated = false;
	double Lo = 0.0, Hi = 0.0, RatedLo = 0.0, RatedHi = 0.0;
	ForceLimitsOf(Table, Model, Id, /*bRated=*/false, bLimited, Lo, Hi);
	ForceLimitsOf(Table, Model, Id, /*bRated=*/true, bRated, RatedLo, RatedHi);
	JointForceRange(bLimited, Lo, Hi, E.Gear, Out.LowerNm, Out.UpperNm);
	JointForceRange(bRated, RatedLo, RatedHi, E.Gear, Out.RatedLowerNm, Out.RatedUpperNm);
	return true;
}

bool FRammsMujocoActuationBackend::SetMotorTorqueLimit(FName MotorId, float MinTorque, float MaxTorque)
{
	if (!(MinTorque < MaxTorque))
	{
		// Cleared: the callback puts the actuator's own range back.
		if (DriveTable)
		{
			FScopeLock Lock(&DriveTable->Lock);
			if (FPhysicsCap* Cap = DriveTable->Caps.Find(MotorId))
			{
				Cap->bRemove = true;
			}
		}
		return true;
	}
	if (CVarRammsMjTorqueCaps.GetValueOnAnyThread() == 0)
	{
		if (DriveTable)
		{
			FScopeLock Lock(&DriveTable->Lock);
			for (TPair<FName, FPhysicsCap>& Pair : DriveTable->Caps)
			{
				Pair.Value.bRemove = true;
			}
		}
		return false;
	}
	UMjPhysicsEngine* Engine = nullptr;
	const mjModel*	  Model = nullptr;
	int32			  Id = INDEX_NONE;
	if (!ResolveJointActuator(MotorId, Engine, Model, Id))
	{
		return false;
	}
	// A plain position servo only: fixed gain, affine bias of -kp and -kv, no
	// activation dynamics.
	const mjtNum Kp = Model->actuator_gainprm[Id * mjNGAIN];
	if (Model->actuator_gaintype[Id] != mjGAIN_FIXED || Model->actuator_biastype[Id] != mjBIAS_AFFINE
		|| Model->actuator_dyntype[Id] != mjDYN_NONE || !(Kp > 0.0)
		|| !FMath::IsNearlyEqual(Model->actuator_biasprm[Id * mjNBIAS + 1], -Kp, 1e-6 * Kp))
	{
		return false;
	}
	const double Gear = Model->actuator_gear[6 * Id];
	if (FMath::Abs(Gear) < 1e-12)
	{
		return false;
	}
	FDriveTable& Table = EnsureDriveTable(*Engine);
	FScopeLock	 Lock(&Table.Lock);
	if (Table.Stiffened.Contains(Id))
	{
		return false; // a stiff speed loop runs it
	}
	// The joint's sense to the actuator's force (divided by the gear; a
	// negative gear swaps the ends); a side at the largest float keeps the
	// actuator's own limit there.
	constexpr double Keep = TNumericLimits<double>::Max();
	const bool		 bKeepLo = MinTorque <= -0.5f * TNumericLimits<float>::Max();
	const bool		 bKeepHi = MaxTorque >= 0.5f * TNumericLimits<float>::Max();
	double			 Lo = bKeepLo ? -Keep : MinTorque / Gear;
	double			 Hi = bKeepHi ? Keep : MaxTorque / Gear;
	if (Gear < 0.0)
	{
		Lo = bKeepHi ? -Keep : MaxTorque / Gear;
		Hi = bKeepLo ? Keep : MinTorque / Gear;
	}
	Table.Articulation = Articulation.Get();
	FPhysicsCap& Cap = Table.Caps.FindOrAdd(MotorId);
	Cap.ActuatorId = Id;
	Cap.Model = Model;
	Cap.Lo = Lo;
	Cap.Hi = Hi;
	Cap.bRemove = false;
	return true;
}

void FRammsMujocoActuationBackend::ForceLimitsOf(const FDriveTable& Table, const mjModel_* Model, int32 Id, bool bRated, bool& bOutLimited,
	double& OutLo, double& OutHi)
{
	// A stiff speed loop's actuator: its loop's range, whichever form (servo,
	// plain motor) the loop left it in. It is never capped (SetMotorTorqueLimit
	// refuses it).
	if (const FStiffened* Was = Table.StiffenedIn == Model ? Table.Stiffened.Find(Id) : nullptr)
	{
		if (Was->bLoopRange)
		{
			bOutLimited = true;
			OutLo = Was->LoopRange[0];
			OutHi = Was->LoopRange[1];
			return;
		}
	}
	if (bRated)
	{
		if (const FCappedRange* Own = Table.CappedIn == Model ? Table.CapOriginal.Find(Id) : nullptr)
		{
			bOutLimited = Own->ForceLimited;
			OutLo = Own->Range[0];
			OutHi = Own->Range[1];
			return;
		}
	}
	bOutLimited = Model->actuator_forcelimited[Id] != 0;
	OutLo = Model->actuator_forcerange[2 * Id];
	OutHi = Model->actuator_forcerange[2 * Id + 1];
}

void FRammsMujocoActuationBackend::StepEfforts(FDriveTable& Table, const mjModel_* Model, const mjData_* Data, double Dt)
{
	for (TPair<FName, FPhysicsEffort>& Pair : Table.Efforts)
	{
		FPhysicsEffort& E = Pair.Value;
		const int32		Id = E.ActuatorId;
		if (E.Model != Model || Id < 0 || Id >= Model->nu || E.Ring.Num() != EffortSamples)
		{
			continue;
		}
		// actuator_force before this step is the force of the step just
		// finished, under the range that step ran with.
		const double F = Data->actuator_force[Id];
		uint8		 Flags = 0;
		bool		 bLimited = false;
		double		 Lo = 0.0, Hi = 0.0;
		ForceLimitsOf(Table, Model, Id, /*bRated=*/false, bLimited, Lo, Hi);
		if (bLimited)
		{
			Flags |= AtLower(F, Lo) ? 1 : 0;
			Flags |= AtUpper(F, Hi) ? 2 : 0;
		}
		// At its own rating, on the side it pushed.
		bool   Rated = false;
		double RatedLo = 0.0, RatedHi = 0.0;
		ForceLimitsOf(Table, Model, Id, /*bRated=*/true, Rated, RatedLo, RatedHi);
		if (Rated && ((F < 0.0 && AtLower(F, RatedLo)) || (F > 0.0 && AtUpper(F, RatedHi))))
		{
			Flags |= 4;
		}
		if (E.Gear < 0.0)
		{
			// The joint's sense: the ends trade places.
			Flags = static_cast<uint8>((Flags & 4) | ((Flags & 1) << 1) | ((Flags & 2) >> 1));
		}
		FEffortSample& Sample = E.Ring[E.Head];
		Sample.Time = Data->time;
		Sample.Force = static_cast<float>(F * E.Gear);
		Sample.Dt = static_cast<float>(Dt);
		Sample.Flags = Flags;
		E.Head = (E.Head + 1) % EffortSamples;
		E.Count = FMath::Min(E.Count + 1, EffortSamples);
	}
}

void FRammsMujocoActuationBackend::StepCaps(FDriveTable& Table, mjModel_* Model)
{
	if (Table.CappedIn != Model)
	{
		// A model since recompiled has its own ranges: nothing to put back.
		Table.CapOriginal.Reset();
		Table.CappedIn = Model;
	}
	// ramms.mj.TorqueCaps 0: every limit in force put back from the next step
	// (kept, to come back with the setting).
	const bool bCapsOn = CVarRammsMjTorqueCaps.GetValueOnAnyThread() != 0;
	for (auto It = Table.Caps.CreateIterator(); It; ++It)
	{
		FPhysicsCap& Cap = It.Value();
		const int32	 Id = Cap.ActuatorId;
		const bool	 bValid = Cap.Model == Model && Id >= 0 && Id < Model->nu;
		const bool	 bLive = bCapsOn && Table.bAlive && !Cap.bRemove && bValid;
		if (bLive)
		{
			if (!Table.CapOriginal.Contains(Id))
			{
				FCappedRange& Own = Table.CapOriginal.Add(Id);
				Own.ForceLimited = Model->actuator_forcelimited[Id] != 0;
				Own.Range[0] = Model->actuator_forcerange[2 * Id];
				Own.Range[1] = Model->actuator_forcerange[2 * Id + 1];
			}
			const FCappedRange& Own = Table.CapOriginal[Id];
			const double		AuthLo = Own.ForceLimited ? Own.Range[0] : -1e9;
			const double		AuthHi = Own.ForceLimited ? Own.Range[1] : 1e9;
			// Within its own range, and never across zero: a limit bounds a push,
			// it does not command one.
			const double Lo = FMath::Min(FMath::Max(AuthLo, Cap.Lo), 0.0);
			const double Hi = FMath::Max(FMath::Min(AuthHi, Cap.Hi), 0.0);
			Model->actuator_forcelimited[Id] = 1;
			Model->actuator_forcerange[2 * Id] = Lo;
			Model->actuator_forcerange[2 * Id + 1] = Hi;
			continue;
		}
		if (bValid)
		{
			if (const FCappedRange* Own = Table.CapOriginal.Find(Id))
			{
				Model->actuator_forcelimited[Id] = Own->ForceLimited ? 1 : 0;
				Model->actuator_forcerange[2 * Id] = Own->Range[0];
				Model->actuator_forcerange[2 * Id + 1] = Own->Range[1];
				Table.CapOriginal.Remove(Id);
			}
		}
		if (Cap.bRemove || !Table.bAlive || !bValid)
		{
			It.RemoveCurrent();
		}
	}
}
