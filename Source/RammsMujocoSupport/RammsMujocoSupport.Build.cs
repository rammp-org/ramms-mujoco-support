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
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			// Ramms.Panel debug joint panel (Slate window)
			"Slate",
			"SlateCore",
		});
	}
}
