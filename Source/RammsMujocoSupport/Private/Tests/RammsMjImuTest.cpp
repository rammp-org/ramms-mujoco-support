// Copyright Epic Games, Inc. All Rights Reserved.

// The MuJoCo IMU provider must read what MuJoCo's own sensors read. A small model compiled from MJCF carries an
// accelerometer, a gyro and a velocimeter on a site, offset and turned on a free body that spins and falls; the
// provider's arithmetic (and its per-step callbacks, driven by hand around mj_step) samples the same frame, and the
// two are compared in Unreal units and axes. This pins the gravity convention (+|g| up at rest, 0 in free fall), the
// lever arm, the axis conversions and the step timing of cacc. Then: which engine Auto picks for a point under a
// URLab body.

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "RammsImuMath.h"
#include "RammsMujocoImuProvider.h"
#include "RammsPhysicsImuSubsystem.h"

#include <mujoco/mujoco.h>

#if WITH_DEV_AUTOMATION_TESTS

namespace RammsMjImuTest
{
	/** A free box with a turned, offset IMU site, and a box welded to the world with another. */
	const char* const ImuModelXml = R"MJCF(
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="fixed" pos="1 2 0.5">
      <geom type="box" size="0.1 0.1 0.1"/>
      <site name="fixed_imu" pos="0.05 0 0.1" quat="0.7071068 0 0 0.7071068"/>
    </body>
    <body name="box" pos="0 0 2">
      <freejoint/>
      <geom type="box" size="0.1 0.2 0.3" mass="2"/>
      <site name="imu" pos="0.05 -0.1 0.15" quat="0.9238795 0.2 0.3 0.1"/>
    </body>
  </worldbody>
  <sensor>
    <accelerometer site="imu"/>
    <gyro site="imu"/>
    <velocimeter site="imu"/>
  </sensor>
</mujoco>
)MJCF";

	/** The compiled model and its data, freed on scope exit. */
	struct FImuModel
	{
		mjSpec*	 Spec = nullptr;
		mjModel* Model = nullptr;
		mjData*	 Data = nullptr;
		FString	 Error;

		FImuModel()
		{
			char Message[1024] = {};
			Spec = mj_parseXMLString(ImuModelXml, nullptr, Message, sizeof(Message));
			Model = Spec ? mj_compile(Spec, nullptr) : nullptr;
			Data = Model ? mj_makeData(Model) : nullptr;
			if (!Data)
			{
				Error = Spec ? FString(UTF8_TO_TCHAR(mjs_getError(Spec))) : FString(UTF8_TO_TCHAR(Message));
			}
		}

		~FImuModel()
		{
			if (Data)
			{
				mj_deleteData(Data);
			}
			if (Model)
			{
				mj_deleteModel(Model);
			}
			if (Spec)
			{
				mj_deleteSpec(Spec);
			}
		}

		int32 Id(mjtObj Type, const char* Name) const { return mj_name2id(Model, Type, Name); }

		/** A site's frame in its body's frame, in Unreal units. */
		FTransform SiteInBody(int32 Site) const
		{
			return FTransform(URLabAxisConv::MjQuatToUe(Model->site_quat + 4 * Site), URLabAxisConv::MjPositionToUe(Model->site_pos + 3 * Site));
		}

		/** Spin and throw the free box. */
		void Throw() const
		{
			const int32	 Joint = Model->body_jntadr[Id(mjOBJ_BODY, "box")];
			const int32	 Dof = Model->jnt_dofadr[Joint];
			const double Velocity[6] = { 0.3, -0.2, 1.0, 1.0, 2.0, -3.0 }; // linear (world), angular (body)
			for (int32 i = 0; i < 6; ++i)
			{
				Data->qvel[Dof + i] = Velocity[i];
			}
			mj_forward(Model, Data);
		}
	};

	/** The site's sensor readings (sensor frame) in Unreal units and axes: local vectors convert as world ones do
	 *  (Y negated for polar vectors, X and Z for axial ones). */
	struct FMuJoCoReadings
	{
		FVector Accelerometer; // cm/s^2
		FVector Gyro;		   // deg/s
		FVector Velocimeter;   // cm/s
	};

	FMuJoCoReadings ReadSensors(const FImuModel& Imu)
	{
		const mjtNum*	Data = Imu.Data->sensordata;
		const int32		Accel = Imu.Model->sensor_adr[0];
		const int32		Gyro = Imu.Model->sensor_adr[1];
		const int32		Velo = Imu.Model->sensor_adr[2];
		FMuJoCoReadings Out;
		Out.Accelerometer = URLabAxisConv::MjDirectionToUe(Data + Accel) * 100.0;
		Out.Gyro = RammsImuMath::RadiansToDegrees(FRammsMujocoImuProvider::MjAxialToUe(Data + Gyro));
		Out.Velocimeter = URLabAxisConv::MjDirectionToUe(Data + Velo) * 100.0;
		return Out;
	}

	bool NearlyEqualVectors(FAutomationTestBase& Test, const FString& What, const FVector& Actual, const FVector& Expected, double Slack)
	{
		const bool bNear = (Actual - Expected).GetAbsMax() <= Slack;
		Test.TestTrue(FString::Printf(TEXT("%s: %s, expected %s (+-%g)"), *What, *Actual.ToString(), *Expected.ToString(), Slack), bNear);
		return bNear;
	}

	/** The sample, read in the sensor's frame as an IMU would. */
	FMuJoCoReadings InSensorFrame(const FRammsImuSample& Sample)
	{
		FMuJoCoReadings Out;
		Out.Accelerometer = RammsImuMath::ToSensorFrame(Sample.Orientation, Sample.ProperAcceleration);
		Out.Gyro = RammsImuMath::ToSensorFrame(Sample.Orientation, Sample.AngularVelocity);
		Out.Velocimeter = RammsImuMath::ToSensorFrame(Sample.Orientation, Sample.LinearVelocity);
		return Out;
	}

	void CompareReadings(FAutomationTestBase& Test, const FString& What, const FMuJoCoReadings& Ours, const FMuJoCoReadings& MuJoCo)
	{
		// Double arithmetic on both sides; the slack covers the conversions' rounding.
		NearlyEqualVectors(Test, What + TEXT(": accelerometer"), Ours.Accelerometer, MuJoCo.Accelerometer, 1.0e-6);
		NearlyEqualVectors(Test, What + TEXT(": gyro"), Ours.Gyro, MuJoCo.Gyro, 1.0e-6);
		NearlyEqualVectors(Test, What + TEXT(": velocimeter"), Ours.Velocimeter, MuJoCo.Velocimeter, 1.0e-6);
	}
} // namespace RammsMjImuTest

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMjImuAxesTest, "Ramms.MuJoCo.Imu.Axes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsMjImuAxesTest::RunTest(const FString& Parameters)
{
	using namespace RammsMjImuTest;

	// A MuJoCo rotation at rate w (world) for dt, converted, is the Unreal rotation at MjAxialToUe(w).
	const mjtNum Rate[3] = { 0.4, -1.1, 2.3 };
	const double Dt = 1.0e-3;
	const double Speed = FMath::Sqrt(Rate[0] * Rate[0] + Rate[1] * Rate[1] + Rate[2] * Rate[2]);
	mjtNum		 Start[4] = { 0.9, 0.1, -0.3, 0.2 };
	mju_normalize4(Start);
	const double Half = 0.5 * Speed * Dt;
	const mjtNum Turn[4] = { FMath::Cos(Half), FMath::Sin(Half) * Rate[0] / Speed, FMath::Sin(Half) * Rate[1] / Speed, FMath::Sin(Half) * Rate[2] / Speed };
	mjtNum		 End[4];
	mju_mulQuat(End, Turn, Start);
	NearlyEqualVectors(*this, TEXT("angular velocity"), RammsImuMath::AngularVelocityBetween(URLabAxisConv::MjQuatToUe(Start), URLabAxisConv::MjQuatToUe(End), Dt),
		FRammsMujocoImuProvider::MjAxialToUe(Rate), 1.0e-6);

	// And w x r, converted, is the converted w x the converted r.
	const mjtNum Lever[3] = { 0.2, 0.5, -0.7 };
	mjtNum		 Velocity[3];
	mju_cross(Velocity, Rate, Lever);
	NearlyEqualVectors(*this, TEXT("w x r"), RammsImuMath::PointVelocity(FVector::ZeroVector, FRammsMujocoImuProvider::MjAxialToUe(Rate), URLabAxisConv::MjPositionToUe(Lever)),
		URLabAxisConv::MjDirectionToUe(Velocity) * 100.0, 1.0e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMjImuMatchesSensorsTest, "Ramms.MuJoCo.Imu.MatchesMuJoCoSensors",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsMjImuMatchesSensorsTest::RunTest(const FString& Parameters)
{
	using namespace RammsMjImuTest;
	FImuModel Imu;
	if (!TestTrue(FString::Printf(TEXT("the model compiles (%s)"), *Imu.Error), Imu.Data != nullptr))
	{
		return false;
	}
	const int32		 Box = Imu.Id(mjOBJ_BODY, "box");
	const int32		 Site = Imu.Id(mjOBJ_SITE, "imu");
	const FTransform SiteInBox = Imu.SiteInBody(Site);
	Imu.Throw();

	// The provider's callbacks, driven by hand around each step as URLab's worker runs them, with the box's site
	// tracked as point 1.
	FRammsMujocoImuProvider::FTable	 Table;
	FRammsMujocoImuProvider::FPoint& Point = Table.Points.Add(1);
	Point.Model = Imu.Model;
	Point.NBody = Imu.Model->nbody;
	Point.BodyId = Box;
	Point.PointInBody = SiteInBox;

	for (int32 Step = 0; Step < 20; ++Step)
	{
		FRammsMujocoImuProvider::PreStep(Table, Imu.Model, Imu.Data);
		mj_step(Imu.Model, Imu.Data);
		// MuJoCo's accelerometer computed cacc in the step; forget it, so the callback computes its own from the
		// qvel the step started with.
		Imu.Data->flg_rnepost = 0;
		TArray<double> QvelAfter(Imu.Data->qvel, Imu.Model->nv);
		FRammsMujocoImuProvider::PostStep(Table, Imu.Model, Imu.Data);
		TestTrue(FString::Printf(TEXT("step %d: qvel is put back"), Step), FMemory::Memcmp(QvelAfter.GetData(), Imu.Data->qvel, sizeof(double) * Imu.Model->nv) == 0);
	}
	const FRammsImuSample Sample = Table.Points[1].Latest;
	TestTrue(TEXT("sampled"), Sample.bValid && Sample.bHasAcceleration);
	TestEqual(TEXT("MuJoCo"), Sample.Source, ERammsImuSource::MuJoCo);
	TestEqual(TEXT("stamped with mjData::time"), Sample.SimTimeSeconds, (double)Imu.Data->time, 1.0e-12);

	// The step's sensors are the state its forward pass computed (as are xpos, xquat, cvel): the same as ours.
	const FMuJoCoReadings Theirs = ReadSensors(Imu);
	CompareReadings(*this, TEXT("cacc from the step's qvel"), InSensorFrame(Sample), Theirs);
	NearlyEqualVectors(*this, TEXT("the site's position"), Sample.Location, URLabAxisConv::MjPositionToUe(Imu.Data->site_xpos + 3 * Site), 1.0e-6);
	const mjtNum* Frame = Imu.Data->site_xmat + 9 * Site; // row-major: column i is the site's axis i in world
	const double  AxisX[3] = { Frame[0], Frame[3], Frame[6] };
	const double  AxisZ[3] = { Frame[2], Frame[5], Frame[8] };
	NearlyEqualVectors(*this, TEXT("the site's X axis"), Sample.Orientation.GetAxisX(), URLabAxisConv::MjDirectionToUe(AxisX), 1.0e-6);
	NearlyEqualVectors(*this, TEXT("the site's Z axis"), Sample.Orientation.GetAxisZ(), URLabAxisConv::MjDirectionToUe(AxisZ), 1.0e-6);

	// In free fall the accelerometer reads only the spin's acceleration of the point, not gravity: compare with the
	// lever-arm arithmetic from the box's own spin.
	const FVector Lever = Sample.Location - URLabAxisConv::MjPositionToUe(Imu.Data->xpos + 3 * Box);
	const FVector Spin = RammsImuMath::DegreesToRadians(Sample.AngularVelocity);
	const FVector Alpha = RammsImuMath::DegreesToRadians(Sample.AngularAcceleration);
	NearlyEqualVectors(*this, TEXT("free fall: no gravity in the reading"), Sample.ProperAcceleration,
		RammsImuMath::PointAcceleration(FVector::ZeroVector, Alpha, Spin, Lever), 1.0e-6);

	// With the step's own cacc (the accelerometer's flg_rnepost), the same.
	FRammsImuSample Direct;
	mj_step(Imu.Model, Imu.Data);
	FRammsMujocoImuProvider::EnsureAccelerations(Imu.Model, Imu.Data, nullptr);
	TestTrue(TEXT("computes"), FRammsMujocoImuProvider::ComputeSample(Imu.Model, Imu.Data, Box, SiteInBox, Direct));
	CompareReadings(*this, TEXT("the accelerometer's cacc"), InSensorFrame(Direct), ReadSensors(Imu));

	// A body welded to the world reads +|g| up in its frame and nothing else.
	FRammsImuSample Still;
	const int32		FixedSite = Imu.Id(mjOBJ_SITE, "fixed_imu");
	TestTrue(TEXT("computes the fixed body"), FRammsMujocoImuProvider::ComputeSample(Imu.Model, Imu.Data, Imu.Id(mjOBJ_BODY, "fixed"), Imu.SiteInBody(FixedSite), Still));
	NearlyEqualVectors(*this, TEXT("at rest: +|g| up"), Still.ProperAcceleration, FVector(0.0, 0.0, 981.0), 1.0e-9);
	NearlyEqualVectors(*this, TEXT("at rest: still"), Still.LinearVelocity, FVector::ZeroVector, 0.0);
	NearlyEqualVectors(*this, TEXT("gravity in Unreal units"), Still.Gravity, FVector(0.0, 0.0, -981.0), 1.0e-9);
	NearlyEqualVectors(*this, TEXT("the fixed site's position"), Still.Location, URLabAxisConv::MjPositionToUe(Imu.Data->site_xpos + 3 * FixedSite), 1.0e-6);

	// Points resolved against another model are left alone.
	Point.Model = nullptr;
	Table.Points[1].Latest = FRammsImuSample();
	FRammsMujocoImuProvider::PostStep(Table, Imu.Model, Imu.Data);
	TestFalse(TEXT("a point on another model is not sampled"), Table.Points[1].Latest.bValid);
	return true;
}

// ---------------------------------------------------------------------------
// Which engine serves a point under a URLab body
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMjImuSelectionTest, "Ramms.MuJoCo.Imu.Selection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsMjImuSelectionTest::RunTest(const FString& Parameters)
{
	using namespace RammsMjImuTest;
	UWorld* World = UWorld::CreateWorld(EWorldType::Game, /*bInformEngineOfWorld=*/false);
	if (!TestNotNull(TEXT("world"), World))
	{
		return false;
	}
	FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
	Context.SetCurrentWorld(World);
	ON_SCOPE_EXIT
	{
		GEngine->DestroyWorldContext(World);
		World->DestroyWorld(false);
	};

	URammsPhysicsImuSubsystem* Imu = URammsPhysicsImuSubsystem::Get(World);
	if (!TestNotNull(TEXT("the IMU subsystem"), Imu))
	{
		return false;
	}

	// A robot whose root is a URLab body, with a sensor mount below it.
	AActor*	 Robot = World->SpawnActor<AActor>();
	UMjBody* Chassis = NewObject<UMjBody>(Robot, TEXT("chassis"));
	Chassis->MjName = FString(TEXT("chassis"));
	Robot->SetRootComponent(Chassis);
	Chassis->RegisterComponent();
	USceneComponent* Mount = NewObject<USceneComponent>(Robot, TEXT("Mount"));
	Mount->SetupAttachment(Chassis);
	Mount->SetRelativeLocation(FVector(10.0, 0.0, 20.0));
	Mount->RegisterComponent();

	// No MuJoCo scene in the world: nothing simulates it.
	FRammsImuBodyRef Ref;
	Imu->ResolveBody(*Mount, NAME_None, FTransform::Identity, ERammsImuSource::Auto, Ref);
	TestEqual(TEXT("without a MuJoCo scene: Kinematic"), Ref.Source, ERammsImuSource::Kinematic);

	// With one (compiled or not): MuJoCo, the URLab body being the body, one hop up.
	World->SpawnActor<AAMjManager>();
	Imu->ResolveBody(*Mount, NAME_None, FTransform(FVector(5.0, 0.0, 0.0)), ERammsImuSource::Auto, Ref);
	TestEqual(TEXT("Auto picks MuJoCo under a URLab body"), Ref.Source, ERammsImuSource::MuJoCo);
	TestTrue(TEXT("the URLab body is the body"), Ref.Body.Get() == Chassis);
	TestEqual(TEXT("one hop up"), Ref.Depth, 1);
	TestEqual(TEXT("named by its MJCF name"), Ref.BodyName, FString(TEXT("chassis")));
	NearlyEqualVectors(*this, TEXT("the point in the body's frame"), Ref.PointInBody.GetLocation(), FVector(15.0, 0.0, 20.0), 1.0e-6);

	// Tracked: MuJoCo, with no sample until the scene compiles and steps.
	const int32 Handle = Imu->Track(Mount, FVector::ZeroVector);
	TestEqual(TEXT("tracked on MuJoCo"), Imu->GetTrackedSource(Handle), ERammsImuSource::MuJoCo);
	FRammsImuSample Sample;
	TestFalse(TEXT("no sample before the scene steps"), Imu->GetSample(Handle, Sample));

	// Listed by its MJCF name.
	TArray<FRammsImuBodyRef> Bodies;
	Imu->ListBodies(*Robot, Bodies);
	TestTrue(TEXT("the chassis is listed on MuJoCo"), Bodies.Num() >= 1 && Bodies[0].BodyName == TEXT("chassis") && Bodies[0].Source == ERammsImuSource::MuJoCo);
	Imu->Untrack(Handle);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
