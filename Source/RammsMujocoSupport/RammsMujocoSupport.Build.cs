// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class RammsMujocoSupport : ModuleRules
{
	public RammsMujocoSupport(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			// URLab exposes <mujoco/mujoco.h> (PublicIncludePaths) and links the MuJoCo
			// libs publicly, so depending on it gives us the MuJoCo C API + the
			// UMjArticulationController base class.
			"URLab",
			// IRammsControlContributor lives in RammsCore, but the control types it
			// speaks (FRammsControlSurface / axis) come from ramms-ui's RammsControl.
			"RammsControl",
			// The robot base component + IRammsActuationBackend interface, the
			// cross-plugin actuation-backend registry we register into at startup,
			// and IRammsControlContributor (inherited by public headers here).
			"RammsCore",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			// Runtime-built meshes for the compiled-geom renderer (visuals for
			// <attach>/<model>-spliced content that has no URLab components).
			"ProceduralMeshComponent",
		});
	}
}
