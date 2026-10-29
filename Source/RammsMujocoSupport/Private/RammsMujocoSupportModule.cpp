// Copyright Epic Games, Inc. All Rights Reserved.

#include "Modules/ModuleManager.h"
#include "RammsActuationBackend.h"
#include "RammsActuationBackendRegistry.h"
#include "RammsArticulationActuationBackend.h"

/**
 * RammsMujocoSupport module. On startup it registers the MuJoCo actuation-backend
 * factory with RammsCore's registry, so URammsRobotBaseComponent (and every
 * controller consuming it) can drive a URLab articulation without RammsCore
 * depending on URLab.
 */
class FRammsMujocoSupportModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		RammsActuationBackends::RegisterMujocoFactory(
			FRammsActuationBackendFactory::CreateStatic(&FRammsMujocoSupportModule::CreateMujocoBackend));
		RammsActuationBackends::RegisterChaosArticulationFactory(
			FRammsActuationBackendFactory::CreateStatic(&FRammsMujocoSupportModule::CreateChaosArticulationBackend));
	}

	virtual void ShutdownModule() override
	{
		RammsActuationBackends::UnregisterMujocoFactory();
		RammsActuationBackends::UnregisterChaosArticulationFactory();
	}

private:
	static IRammsActuationBackend* CreateMujocoBackend(URammsRobotBaseComponent& /*Base*/)
	{
		// Constructed only; the base component calls Initialize and frees it (and
		// leaves itself backend-less if Initialize can't resolve an articulation).
		// It drives the articulation on MuJoCo, or on Chaos when the articulation
		// carries a URammsChaosArticulationComponent.
		return new FRammsArticulationActuationBackend();
	}

	static IRammsActuationBackend* CreateChaosArticulationBackend(URammsRobotBaseComponent& /*Base*/)
	{
		// Backend = Chaos: the same backend, but it switches the articulation to Chaos first.
		return new FRammsArticulationActuationBackend(/*bSwitchToChaos=*/true);
	}
};

IMPLEMENT_MODULE(FRammsMujocoSupportModule, RammsMujocoSupport);
