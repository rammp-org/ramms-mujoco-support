// Copyright Epic Games, Inc. All Rights Reserved.

#include "Modules/ModuleManager.h"
#include "RammsActuationBackend.h"
#include "RammsActuationBackendRegistry.h"
#include "RammsMujocoActuationBackend.h"

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
	}

	virtual void ShutdownModule() override
	{
		RammsActuationBackends::UnregisterMujocoFactory();
	}

private:
	static IRammsActuationBackend* CreateMujocoBackend(URammsRobotBaseComponent& /*Base*/)
	{
		// Constructed only; the base component calls Initialize and frees it (and
		// leaves itself backend-less if Initialize can't resolve an articulation).
		return new FRammsMujocoActuationBackend();
	}
};

IMPLEMENT_MODULE(FRammsMujocoSupportModule, RammsMujocoSupport);
