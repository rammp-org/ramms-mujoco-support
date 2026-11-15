// Copyright Epic Games, Inc. All Rights Reserved.

// The MuJoCo IMU provider must read what MuJoCo's own sensors read when asked for MuJoCo's convention (Explicit),
// and the acceleration the integrator actually applied by default (Realized). A small model compiled from MJCF
// carries an accelerometer, a gyro and a velocimeter on a site, offset and turned on a free body that spins and falls;
// the provider's arithmetic (and its per-step callbacks, driven by hand around mj_step) samples the same frame, and the
// two are compared in Unreal units and axes. This pins the gravity convention (+|g| up at rest, 0 in free fall), the
// lever arm, the axis conversions and the step timing of cacc. A damped slider under implicitfast, where the forward
// pass's qacc is several times what the step applies, tells the two conventions apart; RK4's stamp and acceleration
// are checked too. Then: which bodies Unreal poses (mocap), and which engine Auto picks for a point under a URLab body
// (and picks again when the MuJoCo scene goes).

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

	/** A 1 kg slider on a joint damped at 2000 N s/m under implicitfast: timestep x damping / mass = 4, so the forward
	 *  pass's qacc (-damping x v / mass) is five times what the implicit step applies. */
	const char* const DampedModelXml = R"MJCF(
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>
  <worldbody>
    <body name="slider" pos="0 0 1">
      <joint name="slide" type="slide" axis="1 0 0" damping="2000"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
      <site name="imu" pos="0.05 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <accelerometer site="imu"/>
    <gyro site="imu"/>
    <velocimeter site="imu"/>
  </sensor>
</mujoco>
)MJCF";

	/** A ball thrown under RK4. */
	const char* const RungeKuttaModelXml = R"MJCF(
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4"/>
  <worldbody>
    <body name="ball" pos="0 0 2">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
)MJCF";

	/** A mocap body carrying a body without joints, a free body carrying one, and a body welded to the world. */
	const char* const MocapModelXml = R"MJCF(
<mujoco>
  <worldbody>
    <body name="target" mocap="true" pos="0 0 1">
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
      <body name="held" pos="0.1 0 0">
        <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
      </body>
    </body>
    <body name="free" pos="1 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1"/>
      <body name="carried" pos="0.2 0 0">
        <geom type="sphere" size="0.05"/>
      </body>
    </body>
    <body name="fixed" pos="2 0 1">
      <geom type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>
)MJCF";

	/** A model compiled from MJCF and its data, freed on scope exit. */
	struct FImuModel
	{
		mjSpec*	 Spec = nullptr;
		mjModel* Model = nullptr;
		mjData*	 Data = nullptr;
		FString	 Error;

		explicit FImuModel(const char* Xml = ImuModelXml)
		{
			char Message[1024] = {};
			Spec = mj_parseXMLString(Xml, nullptr, Message, sizeof(Message));
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

		/** Set a body's dof velocities (its joint's first dof on) and bring the derived quantities up to date. */
		void SetVelocity(const char* BodyName, const TArray<double>& Velocity) const
		{
			const int32 Joint = Model->body_jntadr[Id(mjOBJ_BODY, BodyName)];
			const int32 Dof = Model->jnt_dofadr[Joint];
			for (int32 i = 0; i < Velocity.Num(); ++i)
			{
				Data->qvel[Dof + i] = Velocity[i];
			}
			mj_forward(Model, Data);
		}

		/** Spin and throw the free box. */
		void Throw() const { SetVelocity("box", { 0.3, -0.2, 1.0, 1.0, 2.0, -3.0 }); } // linear (world), angular (body)
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

	void CompareReadings(FAutomationTestBase& Test, const FString& What, const FMuJoCoReadings& Ours, const FMuJoCoReadings& MuJoCo, double AccelSlack = 1.0e-6)
	{
		// Double arithmetic on both sides; the slack covers the conversions' rounding.
		NearlyEqualVectors(Test, What + TEXT(": accelerometer"), Ours.Accelerometer, MuJoCo.Accelerometer, AccelSlack);
		NearlyEqualVectors(Test, What + TEXT(": gyro"), Ours.Gyro, MuJoCo.Gyro, 1.0e-6);
		NearlyEqualVectors(Test, What + TEXT(": velocimeter"), Ours.Velocimeter, MuJoCo.Velocimeter, 1.0e-6);
	}

	/**
	 * Step Imu's model Steps times with the provider's callbacks around each step, as URLab's worker runs them, the
	 * site Site on body Body tracked as point 1 under Convention. MuJoCo's own accelerometer computes cacc in the step:
	 * flg_rnepost is cleared, so the callback computes its own. Returns each step's sample and, per step, the time the
	 * step started and MuJoCo's sensor readings (the step's forward pass).
	 */
	void RunSteps(FAutomationTestBase& Test, const FImuModel& Imu, int32 Body, int32 Site, ERammsMjImuAcceleration Convention, int32 Steps,
		TArray<FRammsImuSample>& OutSamples, TArray<double>& OutStepStarts, TArray<FMuJoCoReadings>& OutReadings)
	{
		FRammsMujocoImuProvider::FTable	 Table;
		FRammsMujocoImuProvider::FPoint& Point = Table.Points.Add(1);
		Point.Model = Imu.Model;
		Point.NBody = Imu.Model->nbody;
		Point.BodyId = Body;
		Point.PointInBody = Imu.SiteInBody(Site);
		Table.Acceleration = Convention;
		for (int32 Step = 0; Step < Steps; ++Step)
		{
			const double StepStart = Imu.Data->time;
			FRammsMujocoImuProvider::PreStep(Table, Imu.Model, Imu.Data);
			mj_step(Imu.Model, Imu.Data);
			Imu.Data->flg_rnepost = 0;
			const TArray<double> QvelAfter(Imu.Data->qvel, Imu.Model->nv);
			const TArray<double> QaccAfter(Imu.Data->qacc, Imu.Model->nv);
			FRammsMujocoImuProvider::PostStep(Table, Imu.Model, Imu.Data);
			Test.TestTrue(FString::Printf(TEXT("step %d: qvel is put back"), Step),
				FMemory::Memcmp(QvelAfter.GetData(), Imu.Data->qvel, sizeof(double) * Imu.Model->nv) == 0);
			Test.TestTrue(FString::Printf(TEXT("step %d: qacc is put back"), Step),
				FMemory::Memcmp(QaccAfter.GetData(), Imu.Data->qacc, sizeof(double) * Imu.Model->nv) == 0);
			OutSamples.Add(Table.Points[1].Latest);
			OutStepStarts.Add(StepStart);
			OutReadings.Add(ReadSensors(Imu));
		}
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

	// Both conventions on the free box (Euler, no damping: the step applies the forward pass's qacc, so they agree).
	for (const ERammsMjImuAcceleration Convention : { ERammsMjImuAcceleration::Explicit, ERammsMjImuAcceleration::Realized })
	{
		const FString What = Convention == ERammsMjImuAcceleration::Explicit ? TEXT("explicit") : TEXT("realized");
		FImuModel	  Imu;
		if (!TestTrue(FString::Printf(TEXT("the model compiles (%s)"), *Imu.Error), Imu.Data != nullptr))
		{
			return false;
		}
		const int32 Box = Imu.Id(mjOBJ_BODY, "box");
		const int32 Site = Imu.Id(mjOBJ_SITE, "imu");
		Imu.Throw();

		TArray<FRammsImuSample> Samples;
		TArray<double>			StepStarts;
		TArray<FMuJoCoReadings> Readings;
		RunSteps(*this, Imu, Box, Site, Convention, 20, Samples, StepStarts, Readings);
		const FRammsImuSample& Sample = Samples.Last();
		TestTrue(What + TEXT(": sampled"), Sample.bValid && Sample.bHasAcceleration);
		TestEqual(What + TEXT(": MuJoCo"), Sample.Source, ERammsImuSource::MuJoCo);
		TestEqual(What + TEXT(": stamped with the time of its state, the step's start"), Sample.SimTimeSeconds, StepStarts.Last(), 1.0e-12);

		// The step's sensors are the state its forward pass computed (as are xpos, xquat, cvel): the same as ours.
		CompareReadings(*this, What + TEXT(": cacc from the step's qvel"), InSensorFrame(Sample), Readings.Last());
		const int32 SiteId = Imu.Id(mjOBJ_SITE, "imu");
		NearlyEqualVectors(*this, What + TEXT(": the site's position"), Sample.Location, URLabAxisConv::MjPositionToUe(Imu.Data->site_xpos + 3 * SiteId), 1.0e-6);
		const mjtNum* Frame = Imu.Data->site_xmat + 9 * SiteId; // row-major: column i is the site's axis i in world
		const double  AxisX[3] = { Frame[0], Frame[3], Frame[6] };
		const double  AxisZ[3] = { Frame[2], Frame[5], Frame[8] };
		NearlyEqualVectors(*this, What + TEXT(": the site's X axis"), Sample.Orientation.GetAxisX(), URLabAxisConv::MjDirectionToUe(AxisX), 1.0e-6);
		NearlyEqualVectors(*this, What + TEXT(": the site's Z axis"), Sample.Orientation.GetAxisZ(), URLabAxisConv::MjDirectionToUe(AxisZ), 1.0e-6);

		// In free fall the accelerometer reads only the spin's acceleration of the point, not gravity: compare with
		// the lever-arm arithmetic from the box's own spin.
		const FVector Lever = Sample.Location - URLabAxisConv::MjPositionToUe(Imu.Data->xpos + 3 * Box);
		const FVector Spin = RammsImuMath::DegreesToRadians(Sample.AngularVelocity);
		const FVector Alpha = RammsImuMath::DegreesToRadians(Sample.AngularAcceleration);
		NearlyEqualVectors(*this, What + TEXT(": free fall: no gravity in the reading"), Sample.ProperAcceleration,
			RammsImuMath::PointAcceleration(FVector::ZeroVector, Alpha, Spin, Lever), 1.0e-6);

		if (Convention == ERammsMjImuAcceleration::Realized)
		{
			TestFalse(TEXT("realized: leaves cacc marked as not MuJoCo's own"), Imu.Data->flg_rnepost != 0);
		}
	}

	FImuModel Imu;
	if (!TestTrue(TEXT("the model compiles"), Imu.Data != nullptr))
	{
		return false;
	}
	const int32		 Box = Imu.Id(mjOBJ_BODY, "box");
	const FTransform SiteInBox = Imu.SiteInBody(Imu.Id(mjOBJ_SITE, "imu"));
	Imu.Throw();

	// With the step's own cacc (the accelerometer's flg_rnepost), the same.
	FRammsImuSample Direct;
	mj_step(Imu.Model, Imu.Data);
	FRammsMujocoImuProvider::EnsureAccelerations(Imu.Model, Imu.Data, nullptr);
	TestTrue(TEXT("computes"), FRammsMujocoImuProvider::ComputeSample(Imu.Model, Imu.Data, Box, SiteInBox, Imu.Data->time, Direct));
	CompareReadings(*this, TEXT("the accelerometer's cacc"), InSensorFrame(Direct), ReadSensors(Imu));

	// A body welded to the world reads +|g| up in its frame and nothing else.
	FRammsImuSample Still;
	const int32		FixedSite = Imu.Id(mjOBJ_SITE, "fixed_imu");
	TestTrue(TEXT("computes the fixed body"),
		FRammsMujocoImuProvider::ComputeSample(Imu.Model, Imu.Data, Imu.Id(mjOBJ_BODY, "fixed"), Imu.SiteInBody(FixedSite), Imu.Data->time, Still));
	NearlyEqualVectors(*this, TEXT("at rest: +|g| up"), Still.ProperAcceleration, FVector(0.0, 0.0, 981.0), 1.0e-9);
	NearlyEqualVectors(*this, TEXT("at rest: still"), Still.LinearVelocity, FVector::ZeroVector, 0.0);
	NearlyEqualVectors(*this, TEXT("gravity in Unreal units"), Still.Gravity, FVector(0.0, 0.0, -981.0), 1.0e-9);
	NearlyEqualVectors(*this, TEXT("the fixed site's position"), Still.Location, URLabAxisConv::MjPositionToUe(Imu.Data->site_xpos + 3 * FixedSite), 1.0e-6);

	// Points resolved against another model are left alone.
	FRammsMujocoImuProvider::FTable	 Table;
	FRammsMujocoImuProvider::FPoint& Point = Table.Points.Add(1);
	Point.Model = nullptr;
	Point.NBody = Imu.Model->nbody;
	Point.BodyId = Box;
	FRammsMujocoImuProvider::PreStep(Table, Imu.Model, Imu.Data);
	mj_step(Imu.Model, Imu.Data);
	FRammsMujocoImuProvider::PostStep(Table, Imu.Model, Imu.Data);
	TestFalse(TEXT("a point on another model is not sampled"), Table.Points[1].Latest.bValid);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMjImuDampedTest, "Ramms.MuJoCo.Imu.RealizedUnderImplicitFast",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsMjImuDampedTest::RunTest(const FString& Parameters)
{
	using namespace RammsMjImuTest;
	constexpr int32 Steps = 4;

	// Explicit: what MuJoCo's own accelerometer reads, the forward pass's -damping x v / mass.
	TArray<FRammsImuSample> Explicit;
	{
		FImuModel Imu(DampedModelXml);
		if (!TestTrue(FString::Printf(TEXT("the damped model compiles (%s)"), *Imu.Error), Imu.Data != nullptr))
		{
			return false;
		}
		Imu.SetVelocity("slider", { 1.0 });
		TArray<double>			StepStarts;
		TArray<FMuJoCoReadings> Readings;
		RunSteps(*this, Imu, Imu.Id(mjOBJ_BODY, "slider"), Imu.Id(mjOBJ_SITE, "imu"), ERammsMjImuAcceleration::Explicit, Steps, Explicit, StepStarts, Readings);
		for (int32 Step = 0; Step < Steps; ++Step)
		{
			// About 2e5 cm/s^2 on the first step: the slack is relative.
			CompareReadings(*this, FString::Printf(TEXT("explicit step %d matches MuJoCo's accelerometer"), Step), InSensorFrame(Explicit[Step]), Readings[Step],
				1.0e-9 * FMath::Max(1.0, Readings[Step].Accelerometer.GetAbsMax()));
		}
	}

	// Realized: the velocity change each step applied. A sample's state is its step's start, so step k's
	// acceleration is (v[k+1] - v[k]) / timestep, from the samples' own velocities.
	TArray<FRammsImuSample> Realized;
	{
		FImuModel Imu(DampedModelXml);
		if (!TestTrue(TEXT("the damped model compiles"), Imu.Data != nullptr))
		{
			return false;
		}
		Imu.SetVelocity("slider", { 1.0 });
		TArray<double>			StepStarts;
		TArray<FMuJoCoReadings> Readings;
		RunSteps(*this, Imu, Imu.Id(mjOBJ_BODY, "slider"), Imu.Id(mjOBJ_SITE, "imu"), ERammsMjImuAcceleration::Realized, Steps, Realized, StepStarts, Readings);
		const double Timestep = Imu.Model->opt.timestep;
		for (int32 Step = 0; Step + 1 < Steps; ++Step)
		{
			const FVector Applied = (Realized[Step + 1].LinearVelocity - Realized[Step].LinearVelocity) / Timestep;
			const FVector Coordinate = Realized[Step].ProperAcceleration + Realized[Step].Gravity;
			NearlyEqualVectors(*this, FString::Printf(TEXT("realized step %d: the velocity change the step applied"), Step), Coordinate, Applied,
				1.0e-6 * FMath::Max(1.0, Applied.GetAbsMax()));
			NearlyEqualVectors(*this, FString::Printf(TEXT("realized step %d: the joint holds it up"), Step), FVector(0.0, 0.0, Realized[Step].ProperAcceleration.Z),
				FVector(0.0, 0.0, 981.0), 1.0e-6);
			// The velocity itself is the step's forward pass's, as MuJoCo's velocimeter reads it.
			NearlyEqualVectors(*this, FString::Printf(TEXT("realized step %d: velocimeter"), Step), InSensorFrame(Realized[Step]).Velocimeter,
				Readings[Step].Velocimeter, 1.0e-6);
		}
	}

	// The two differ: implicitfast applied a fifth of the forward pass's acceleration (1 m/s damped at 2000 N s/m:
	// -2000 m/s^2 explicit, -400 m/s^2 applied).
	if (Explicit.Num() == Steps && Realized.Num() == Steps)
	{
		const double ExplicitX = Explicit[0].ProperAcceleration.X;
		const double RealizedX = Realized[0].ProperAcceleration.X;
		TestTrue(FString::Printf(TEXT("explicit first step about -2e5 cm/s^2 (%.0f)"), ExplicitX), FMath::IsNearlyEqual(ExplicitX, -200000.0, 1.0));
		TestTrue(FString::Printf(TEXT("realized first step about -4e4 cm/s^2 (%.0f)"), RealizedX), FMath::IsNearlyEqual(RealizedX, -40000.0, 1.0));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMjImuRungeKuttaTest, "Ramms.MuJoCo.Imu.RungeKutta",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsMjImuRungeKuttaTest::RunTest(const FString& Parameters)
{
	using namespace RammsMjImuTest;
	FImuModel Imu(RungeKuttaModelXml);
	if (!TestTrue(FString::Printf(TEXT("the RK4 model compiles (%s)"), *Imu.Error), Imu.Data != nullptr))
	{
		return false;
	}
	Imu.SetVelocity("ball", { 1.0, 0.5, 2.0, 0.0, 0.0, 0.0 });

	// RK4 leaves the derived quantities of its last stage, evaluated at the step's end: the sample is stamped there,
	// and the acceleration its stages applied (gravity alone) leaves a falling ball reading nothing.
	FRammsMujocoImuProvider::FTable	 Table;
	FRammsMujocoImuProvider::FPoint& Point = Table.Points.Add(1);
	Point.Model = Imu.Model;
	Point.NBody = Imu.Model->nbody;
	Point.BodyId = Imu.Id(mjOBJ_BODY, "ball");
	for (int32 Step = 0; Step < 5; ++Step)
	{
		FRammsMujocoImuProvider::PreStep(Table, Imu.Model, Imu.Data);
		mj_step(Imu.Model, Imu.Data);
		FRammsMujocoImuProvider::PostStep(Table, Imu.Model, Imu.Data);
	}
	const FRammsImuSample& Sample = Table.Points[1].Latest;
	TestTrue(TEXT("sampled"), Sample.bValid && Sample.bHasAcceleration);
	TestEqual(TEXT("stamped with the step's end"), Sample.SimTimeSeconds, (double)Imu.Data->time, 1.0e-12);
	NearlyEqualVectors(*this, TEXT("free fall reads nothing"), Sample.ProperAcceleration, FVector::ZeroVector, 1.0e-6);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMjImuMocapTest, "Ramms.MuJoCo.Imu.MocapDriven",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsMjImuMocapTest::RunTest(const FString& Parameters)
{
	using namespace RammsMjImuTest;
	FImuModel Imu(MocapModelXml);
	if (!TestTrue(FString::Printf(TEXT("the mocap model compiles (%s)"), *Imu.Error), Imu.Data != nullptr))
	{
		return false;
	}
	// Unreal poses a mocap body and what it carries without joints; MuJoCo simulates the rest.
	TestTrue(TEXT("a mocap body"), FRammsMujocoImuProvider::IsMocapDriven(Imu.Model, Imu.Id(mjOBJ_BODY, "target")));
	TestTrue(TEXT("a body welded to a mocap body"), FRammsMujocoImuProvider::IsMocapDriven(Imu.Model, Imu.Id(mjOBJ_BODY, "held")));
	TestFalse(TEXT("a free body"), FRammsMujocoImuProvider::IsMocapDriven(Imu.Model, Imu.Id(mjOBJ_BODY, "free")));
	TestFalse(TEXT("a body welded to a free body"), FRammsMujocoImuProvider::IsMocapDriven(Imu.Model, Imu.Id(mjOBJ_BODY, "carried")));
	TestFalse(TEXT("a body welded to the world"), FRammsMujocoImuProvider::IsMocapDriven(Imu.Model, Imu.Id(mjOBJ_BODY, "fixed")));
	TestFalse(TEXT("the world"), FRammsMujocoImuProvider::IsMocapDriven(Imu.Model, 0));
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
	AAMjManager* Manager = World->SpawnActor<AAMjManager>();
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

	// The MuJoCo scene goes (its manager destroyed, as a level switched to Chaos does): the point is no longer
	// MuJoCo's, and the subsystem moves it to what serves it now.
	if (Manager)
	{
		Manager->Destroy();
	}
	Imu->RefreshPoints(/*bForceResolve=*/true);
	TestEqual(TEXT("without its scene the point falls back to Kinematic"), Imu->GetTrackedSource(Handle), ERammsImuSource::Kinematic);
	Imu->Untrack(Handle);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
