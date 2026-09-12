// Copyright Epic Games, Inc. All Rights Reserved.

#include "Modules/ModuleManager.h"
#include "RammsDriveBackend.h"
#include "RammsDriveBackendRegistry.h"
#include "RammsMujocoDriveBackend.h"

/**
 * RammsMujocoSupport module. On startup it registers the MuJoCo drive-backend
 * factory with RammsCore's registry, so URammsDifferentialDriveController can
 * drive a URLab articulation without RammsCore depending on URLab.
 */
class FRammsMujocoSupportModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		RammsDriveBackends::RegisterMujocoFactory(
			FRammsDriveBackendFactory::CreateStatic(&FRammsMujocoSupportModule::CreateMujocoBackend));
	}

	virtual void ShutdownModule() override
	{
		RammsDriveBackends::UnregisterMujocoFactory();
	}

private:
	static IRammsDriveBackend* CreateMujocoBackend(URammsDifferentialDriveController& /*Controller*/)
	{
		// Constructed only; the controller calls Initialize and frees it (and
		// falls back to Chaos if Initialize can't resolve an articulation).
		return new FRammsMujocoDriveBackend();
	}
};

IMPLEMENT_MODULE(FRammsMujocoSupportModule, RammsMujocoSupport);
