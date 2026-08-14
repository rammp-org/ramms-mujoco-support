// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class RammsMujocoSupportEditor : ModuleRules
{
	public RammsMujocoSupportEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		// URLab's codegen headers are not unity-safe in external consumers.
		bUseUnity = false;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"ContentBrowser",
			"Kismet",
			"PhysicsCore",
			"RammsMujocoSupport",
			"Slate",
			"SlateCore",
			"ToolMenus",
			"UnrealEd",
			"URLab",
		});
	}
}
