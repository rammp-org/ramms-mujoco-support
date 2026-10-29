// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsArticulationActuationBackend.h"

#include "GameFramework/Actor.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "RammsChaosArticulationComponent.h"
#include "RammsMjSimControlComponent.h"
#include "RammsRobotBaseComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsArticulationBackend, Log, All);

// ---------------------------------------------------------------------------------------------- //
// Chaos
// ---------------------------------------------------------------------------------------------- //

bool FRammsChaosArticulationBackend::Initialize(URammsRobotBaseComponent& Base)
{
	Articulation = FRammsMujocoActuationBackend::FindArticulation(Base);
	Chaos = nullptr;
	LastLookFrame = MAX_uint64;
	return Articulation.IsValid();
}

URammsChaosArticulationComponent* FRammsChaosArticulationBackend::Component() const
{
	if (URammsChaosArticulationComponent* Found = Chaos.Get())
	{
		return Found;
	}
	AMjArticulation* Art = Articulation.Get();
	if (!Art || LastLookFrame == GFrameCounter)
	{
		return nullptr;
	}
	LastLookFrame = GFrameCounter; // an articulation has hundreds of components: not every call
	Chaos = Art->FindComponentByClass<URammsChaosArticulationComponent>();
	return Chaos.Get();
}

bool FRammsChaosArticulationBackend::IsActive() const
{
	const URammsChaosArticulationComponent* Comp = Component();
	return Comp && Comp->bSimulateWithChaos && Comp->GetBuildError().IsEmpty();
}

FName FRammsChaosArticulationBackend::JointOf(FName MotorId, bool& bOutSlide) const
{
	const URammsChaosArticulationComponent* Comp = Component();
	const FName								Joint = Comp ? Comp->GetActuatorJoint(MotorId) : NAME_None;
	bOutSlide = !Joint.IsNone() && Comp->IsSlide(Joint);
	return Joint;
}

void FRammsChaosArticulationBackend::SetCommand(FName MotorId, float Value)
{
	if (URammsChaosArticulationComponent* Comp = Component())
	{
		Comp->SetControl(MotorId, Value); // clamped to ctrlrange; kept until the build when early
	}
}

float FRammsChaosArticulationBackend::GetValue(FName MotorId) const
{
	bool		bSlide = false;
	const FName Joint = JointOf(MotorId, bSlide);
	if (Joint.IsNone())
	{
		return 0.0f;
	}
	const float Q = Component()->GetJointPosition(Joint);
	return bSlide ? Q * 100.0f : Q;
}

float FRammsChaosArticulationBackend::GetVelocity(FName MotorId) const
{
	bool		bSlide = false;
	const FName Joint = JointOf(MotorId, bSlide);
	if (Joint.IsNone())
	{
		return 0.0f;
	}
	const float Qd = Component()->GetJointVelocity(Joint);
	return bSlide ? Qd * 100.0f : Qd;
}

bool FRammsChaosArticulationBackend::ReleaseMotor(FName MotorId)
{
	URammsChaosArticulationComponent* Comp = Component();
	return Comp && Comp->ReleaseActuator(MotorId);
}

bool FRammsChaosArticulationBackend::GetMotorTransform(FName MotorId, FTransform& OutWorld) const
{
	bool		bSlide = false;
	const FName Joint = JointOf(MotorId, bSlide);
	return !Joint.IsNone() && Component()->GetJointBodyTransform(Joint, OutWorld);
}

bool FRammsChaosArticulationBackend::SetVelocityDrive(FName MotorId, float TargetRate, const FRammsVelocityGains& Gains,
	float TorqueMin, float TorqueMax)
{
	// False hands the loop back to the base (before the build, or for a servo); drop any loop this
	// backend was running so the two never both command the motor.
	bool							  bSlide = false;
	const FName						  Joint = JointOf(MotorId, bSlide);
	URammsChaosArticulationComponent* Comp = Component();
	if (Joint.IsNone() || !Comp->SetVelocityDrive(MotorId, TargetRate, Gains.Kp, Gains.Ki, Gains.MaxIntegralTorque, TorqueMin, TorqueMax, bSlide ? 100.0 : 1.0))
	{
		ClearVelocityDrive(MotorId);
		return false;
	}
	return true;
}

bool FRammsChaosArticulationBackend::SimulatesMotorJoint(FName MotorId) const
{
	const URammsChaosArticulationComponent* Comp = Component();
	return !(Comp && Comp->IsMotor(MotorId));
}

bool FRammsChaosArticulationBackend::SetMotorBrake(FName MotorId, bool bEngaged)
{
	URammsChaosArticulationComponent* Comp = Component();
	return Comp && Comp->SetMotorBrake(MotorId, bEngaged);
}

void FRammsChaosArticulationBackend::ClearVelocityDrive(FName MotorId)
{
	if (URammsChaosArticulationComponent* Comp = Component())
	{
		Comp->ClearVelocityDrive(MotorId);
	}
}

// ---------------------------------------------------------------------------------------------- //
// Either engine
// ---------------------------------------------------------------------------------------------- //

bool FRammsArticulationActuationBackend::Initialize(URammsRobotBaseComponent& Base)
{
	OwnerName = GetNameSafe(Base.GetOwner());
	bWasChaos.Reset();
	const bool bMujoco = Mujoco.Initialize(Base);
	Chaos.Initialize(Base);
	if (bMujoco && bSwitchToChaos)
	{
		AMjArticulation*				  Art = FRammsMujocoActuationBackend::FindArticulation(Base);
		URammsChaosArticulationComponent* Comp = Art ? Art->FindComponentByClass<URammsChaosArticulationComponent>() : nullptr;
		if (!Comp)
		{
			// Start in the keyframe the robot would start in on MuJoCo.
			const URammsMjSimControlComponent* Sim = Base.GetOwner()->FindComponentByClass<URammsMjSimControlComponent>();
			Comp = URammsChaosArticulationComponent::SwitchToChaos(Art, Sim ? Sim->InitialKeyframe : FString());
		}
		else if (!Comp->bSimulateWithChaos)
		{
			Comp->bSimulateWithChaos = true; // Backend = Chaos asks for it
			Comp->EnsureBuilt();
		}
		if (!Comp || !Comp->GetBuildError().IsEmpty())
		{
			UE_LOG(LogRammsArticulationBackend, Error, TEXT("%s: Backend = Chaos, but its articulation could not be built on Chaos: %s"),
				*OwnerName, Comp ? *Comp->GetBuildError() : TEXT("no articulation component"));
			return false;
		}
	}
	return bMujoco; // both resolve the same articulation, or neither does
}

IRammsActuationBackend& FRammsArticulationActuationBackend::Active() const
{
	const bool bChaos = Chaos.IsActive();
	if (!bWasChaos.IsSet() || bWasChaos.GetValue() != bChaos)
	{
		UE_LOG(LogRammsArticulationBackend, Log, TEXT("%s: motors driven through %s"), *OwnerName,
			bChaos ? TEXT("Chaos (URammsChaosArticulationComponent)") : TEXT("MuJoCo (URLab)"));
		bWasChaos = bChaos;
	}
	return bChaos ? static_cast<IRammsActuationBackend&>(const_cast<FRammsChaosArticulationBackend&>(Chaos))
				  : static_cast<IRammsActuationBackend&>(const_cast<FRammsMujocoActuationBackend&>(Mujoco));
}

bool FRammsArticulationActuationBackend::SetVelocityDrive(FName MotorId, float TargetRate, const FRammsVelocityGains& Gains,
	float TorqueMin, float TorqueMax)
{
	// Only the active engine runs a loop; the other's (left from before a switch) stops.
	IRammsActuationBackend& Engine = Active();
	IRammsActuationBackend& Other = &Engine == static_cast<IRammsActuationBackend*>(&Chaos)
		? static_cast<IRammsActuationBackend&>(Mujoco)
		: static_cast<IRammsActuationBackend&>(Chaos);
	Other.ClearVelocityDrive(MotorId);
	return Engine.SetVelocityDrive(MotorId, TargetRate, Gains, TorqueMin, TorqueMax);
}

void FRammsArticulationActuationBackend::ClearVelocityDrive(FName MotorId)
{
	Mujoco.ClearVelocityDrive(MotorId);
	Chaos.ClearVelocityDrive(MotorId);
}
