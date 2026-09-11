// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMjEndEffectorController.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjBody.h"
#include "GameFramework/Actor.h"

namespace
{
	// Solve A x = b for n<=6 via Gaussian elimination with partial pivoting (A is n x n,
	// row-major in A[6][6]). Returns false if singular.
	bool SolveLinear(double A[6][6], const double b[6], double x[6], int n)
	{
		double M[6][7];
		for (int r = 0; r < n; ++r)
		{
			for (int c = 0; c < n; ++c)
				M[r][c] = A[r][c];
			M[r][n] = b[r];
		}
		for (int col = 0; col < n; ++col)
		{
			int piv = col;
			double best = FMath::Abs(M[col][col]);
			for (int r = col + 1; r < n; ++r)
			{
				const double v = FMath::Abs(M[r][col]);
				if (v > best) { best = v; piv = r; }
			}
			if (best < 1e-12)
				return false;
			if (piv != col)
				for (int c = 0; c <= n; ++c)
					Swap(M[piv][c], M[col][c]);
			const double inv = 1.0 / M[col][col];
			for (int r = 0; r < n; ++r)
			{
				if (r == col) continue;
				const double f = M[r][col] * inv;
				if (f == 0.0) continue;
				for (int c = col; c <= n; ++c)
					M[r][c] -= f * M[col][c];
			}
		}
		for (int r = 0; r < n; ++r)
			x[r] = M[r][n] / M[r][r];
		return true;
	}
}

URammsMjEndEffectorController::URammsMjEndEffectorController()
{
	// Default to the Kinova Gen3 "home" pose (7 arm joints, radians).
	HomeArmAngles = {0.0f, 0.26179939f, 3.14159265f, -2.26892803f, 0.0f, 0.95993109f, 1.57079633f};

	// Game-thread tick samples the Chaos mount pose for base_target smoothing.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void URammsMjEndEffectorController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Sample the mount pose (base_target's component world transform, still driven by the actor's
	// attachment to the Chaos chair) and hand it to the physics thread in MuJoCo coordinates.
	if (!bSmoothBaseTarget)
		return;
	UMjBody* Tgt = BaseTargetBodyComp.Get();
	if (!Tgt)
		return;

	const FTransform T = Tgt->GetComponentTransform();
	const FVector P = T.GetLocation();
	const FQuat Q = T.GetRotation();

	FScopeLock Lock(&BaseGoalMutex);
	// UE (cm, left-handed Z-up) -> MuJoCo (m, right-handed Z-up): X, -Y, Z, x0.01;
	// quat FQuat(X,Y,Z,W) -> MuJoCo [w,x,y,z] = {W, -X, Y, -Z}.
	GoalPos[0] = P.X * 0.01; GoalPos[1] = P.Y * -0.01; GoalPos[2] = P.Z * 0.01;
	GoalQuat[0] = Q.W; GoalQuat[1] = -Q.X; GoalQuat[2] = Q.Y; GoalQuat[3] = -Q.Z;
	const double n = FMath::Sqrt(GoalQuat[0] * GoalQuat[0] + GoalQuat[1] * GoalQuat[1] + GoalQuat[2] * GoalQuat[2] + GoalQuat[3] * GoalQuat[3]);
	if (n > 1e-9)
		for (int i = 0; i < 4; ++i) GoalQuat[i] /= n;
	bGoalValid = true;
}

void URammsMjEndEffectorController::Bind(mjModel* m, mjData* d, const TMap<int32, UMjNodeComponent*>& ActuatorIdMap)
{
	Super::Bind(m, d, ActuatorIdMap);

	ArmActIds.Reset();
	ArmQposAddr.Reset();
	ArmDofAddr.Reset();
	GripActId = -1;

	// URLab's scene assembly prefixes every compiled name with the owning articulation actor's
	// ("{ActorName}_..."), so this controller's elements must be matched within its own prefix.
	// Match on StartsWith(owner prefix) AND EndsWith(authored suffix): unambiguous across multiple
	// arms in one composed model, and — unlike concatenating the two — robust to the intermediate
	// prefixes composition adds (e.g. the gripper's "2f85_" in "{ActorName}_2f85_fingers_actuator").
	// An empty suffix matches nothing rather than binding the first element of its kind.
	const FString OwnerPrefix = GetOwner() != nullptr ? GetOwner()->GetName() + TEXT("_") : FString();
	const auto MatchesInOwner = [&OwnerPrefix](const char* Name, const FString& Suffix) -> bool {
		if (Name == nullptr || Suffix.IsEmpty())
			return false;
		const FString N = ANSI_TO_TCHAR(Name);
		return N.StartsWith(OwnerPrefix) && N.EndsWith(Suffix);
	};

	// Find the gripper actuator by name over ALL model actuators. It drives a tendon (not a
	// joint), so the base class may omit it from Bindings — a name lookup is robust either way.
	for (int a = 0; a < m->nu; ++a)
	{
		if (MatchesInOwner(mj_id2name(m, mjOBJ_ACTUATOR, a), GripperActuatorSuffix))
		{
			GripActId = a;
			float Lo = (float)m->actuator_ctrlrange[a * 2];
			float Hi = (float)m->actuator_ctrlrange[a * 2 + 1];
			GripCtrlRange = (Hi > Lo) ? FVector2D(Lo, Hi) : FVector2D(0.0f, 255.0f);
			break;
		}
	}

	// Arm = the joint-transmission position actuators from Bindings (everything but the gripper).
	for (const FActuatorBinding& B : Bindings)
	{
		if (!B.Component || B.ActuatorMjID == GripActId)
			continue;
		if (B.QposAddr < 0 || B.QvelAddr < 0)
			continue;  // not a single-dof joint actuator
		ArmActIds.Add(B.ActuatorMjID);
		ArmQposAddr.Add(B.QposAddr);
		ArmDofAddr.Add(B.QvelAddr);
	}

	// Resolve the EE site within this arm's prefix against the compiled names.
	EeSiteId = -1;
	for (int s = 0; s < m->nsite; ++s)
	{
		if (MatchesInOwner(mj_id2name(m, mjOBJ_SITE, s), EndEffectorSiteName))
		{
			EeSiteId = s;
			break;
		}
	}

	// Base frame = the arm's root body (the direct child of worldbody above the EE site). When that
	// root is a mocap body driven by a moving base, tracking the target in this frame keeps the EE
	// fixed relative to the base instead of the MuJoCo world.
	BaseBodyId = -1;
	if (EeSiteId >= 0)
	{
		int b = m->site_bodyid[EeSiteId];
		while (b > 0 && m->body_parentid[b] != 0)
			b = m->body_parentid[b];
		BaseBodyId = b;
	}

	// Dynamic tracking base: locate base_link's free joint and the mocap target it's welded to, so we
	// can seat the free base on the target at startup (the stiff weld would otherwise fling the arm
	// from the MuJoCo origin to the chair mount).
	FreeBaseQposAddr = -1; FreeBaseDofAddr = -1; BaseTargetBodyId = -1; BaseInitStepsLeft = 0;
	if (BaseBodyId > 0)
	{
		const int32 jadr = m->body_jntadr[BaseBodyId];
		const int32 jnum = m->body_jntnum[BaseBodyId];
		for (int32 jj = 0; jj < jnum; ++jj)
		{
			const int32 jid = jadr + jj;
			if (jid >= 0 && jid < m->njnt && m->jnt_type[jid] == mjJNT_FREE)
			{
				FreeBaseQposAddr = m->jnt_qposadr[jid];
				FreeBaseDofAddr = m->jnt_dofadr[jid];
				break;
			}
		}
	}
	// Prefixed match like the site/actuator lookups (an exact mj_name2id on the authored name
	// finds nothing under composition's "{ActorName}_base_target"). Owner-scoped so a scene with
	// more than one arm seats each free base on its own mount, not the lowest-id one.
	for (int b = 0; b < m->nbody; ++b)
	{
		if (MatchesInOwner(mj_id2name(m, mjOBJ_BODY, b), BaseTargetBodyName))
		{
			BaseTargetBodyId = b;
			break;
		}
	}
	if (FreeBaseQposAddr >= 0 && BaseTargetBodyId >= 0)
		BaseInitStepsLeft = BaseInitSteps;

	// Take ownership of base_target's mocap for sub-frame smoothing: resolve its mocap slot, and stop
	// URLab's per-frame push (its component tick) so our per-substep writes to d->mocap_pos survive.
	// If the component can't be found, we leave URLab driving it and skip smoothing (graceful).
	BaseTargetMocapId = (BaseTargetBodyId >= 0) ? m->body_mocapid[BaseTargetBodyId] : -1;
	BaseTargetBodyComp = nullptr;
	bSmoothInit = false;
	{
		FScopeLock Lock(&BaseGoalMutex);
		bGoalValid = false;
	}
	if (bSmoothBaseTarget && BaseTargetMocapId >= 0)
	{
		if (AMjArticulation* Art = Cast<AMjArticulation>(GetOwner()))
		{
			if (UMjBody* Tgt = Art->GetBody(BaseTargetBodyName))
			{
				BaseTargetBodyComp = Tgt;
				Tgt->SetComponentTickEnabled(false);  // we own its mocap now
			}
		}
	}

	JacP.SetNumUninitialized(3 * m->nv);
	JacR.SetNumUninitialized(3 * m->nv);

	// Start the arm at its home pose (arm joints only — leaves the object / other free bodies
	// untouched, unlike applying a full keyframe). Avoids the straight-up zero configuration.
	// NOTE: the engine calls mj_resetData AFTER Bind (MjPhysicsEngine::Compile), which wipes any
	// qpos we set here — so the real application happens on the first physics step via bHomePending.
	// This write is only a best-effort seed for anything that reads d before stepping.
	bHomePending = bApplyHomeOnBind && HomeArmAngles.Num() >= ArmActIds.Num() && ArmActIds.Num() > 0;
	if (bHomePending)
	{
		for (int32 j = 0; j < ArmActIds.Num(); ++j)
		{
			d->qpos[ArmQposAddr[j]] = (mjtNum)HomeArmAngles[j];
			d->ctrl[ArmActIds[j]] = (mjtNum)HomeArmAngles[j];
		}
		mj_forward(m, d);  // refresh site poses so the target seeds from the home EE
	}

	{
		FScopeLock Lock(&TargetMutex);
		bTargetInit = false;
	}

	UE_LOG(LogTemp, Log, TEXT("[RammsEE-IK] bound: arm actuators=%d gripper id=%d ee site=%d (%s) base body=%d track-base=%d"),
		ArmActIds.Num(), GripActId, EeSiteId, *EndEffectorSiteName, BaseBodyId, bTrackBaseFrame ? 1 : 0);
}

void URammsMjEndEffectorController::ComputeAndApply(mjModel* m, mjData* d, uint8 /*Source*/)
{
	if (!bIsBound || EeSiteId < 0 || ArmActIds.Num() == 0)
		return;

	// Apply the home pose once, now that the engine's post-bind mj_resetData has run. Do this before
	// reading the EE pose so the IK target seeds from the home configuration, not the reset zeros.
	if (bHomePending)
	{
		for (int32 j = 0; j < ArmActIds.Num(); ++j)
		{
			d->qpos[ArmQposAddr[j]] = (mjtNum)HomeArmAngles[j];
			d->qvel[ArmDofAddr[j]] = 0.0;
			d->ctrl[ArmActIds[j]] = (mjtNum)HomeArmAngles[j];
		}
		mj_forward(m, d);  // refresh site/body poses to the home configuration
		bHomePending = false;
	}

	// Dynamic tracking base: own base_target's mocap and advance it smoothly toward the latest mount
	// pose each substep (exponential filter at the physics rate), so a stiff weld tracks without the
	// once-per-frame stair-step jitter. base_target's URLab push was disabled in Bind.
	if (bSmoothBaseTarget && BaseTargetMocapId >= 0)
	{
		double gp[3], gq[4]; bool bValid;
		{
			FScopeLock Lock(&BaseGoalMutex);
			bValid = bGoalValid;
			gp[0] = GoalPos[0]; gp[1] = GoalPos[1]; gp[2] = GoalPos[2];
			gq[0] = GoalQuat[0]; gq[1] = GoalQuat[1]; gq[2] = GoalQuat[2]; gq[3] = GoalQuat[3];
		}
		if (bValid)
		{
			if (!bSmoothInit)
			{
				SmoothPos[0] = gp[0]; SmoothPos[1] = gp[1]; SmoothPos[2] = gp[2];
				SmoothQuat[0] = gq[0]; SmoothQuat[1] = gq[1]; SmoothQuat[2] = gq[2]; SmoothQuat[3] = gq[3];
				bSmoothInit = true;
			}
			else
			{
				const double a = FMath::Clamp((double)BaseSmoothingAlpha, 0.02, 1.0);
				for (int i = 0; i < 3; ++i)
					SmoothPos[i] += a * (gp[i] - SmoothPos[i]);
				// nlerp toward the goal (shortest arc), then renormalize.
				double dot = SmoothQuat[0] * gq[0] + SmoothQuat[1] * gq[1] + SmoothQuat[2] * gq[2] + SmoothQuat[3] * gq[3];
				double sgn = (dot < 0.0) ? -1.0 : 1.0;
				for (int i = 0; i < 4; ++i)
					SmoothQuat[i] += a * (sgn * gq[i] - SmoothQuat[i]);
				double qn = FMath::Sqrt(SmoothQuat[0] * SmoothQuat[0] + SmoothQuat[1] * SmoothQuat[1] + SmoothQuat[2] * SmoothQuat[2] + SmoothQuat[3] * SmoothQuat[3]);
				if (qn > 1e-9)
					for (int i = 0; i < 4; ++i) SmoothQuat[i] /= qn;
			}
			d->mocap_pos[3 * BaseTargetMocapId + 0] = SmoothPos[0];
			d->mocap_pos[3 * BaseTargetMocapId + 1] = SmoothPos[1];
			d->mocap_pos[3 * BaseTargetMocapId + 2] = SmoothPos[2];
			d->mocap_quat[4 * BaseTargetMocapId + 0] = SmoothQuat[0];
			d->mocap_quat[4 * BaseTargetMocapId + 1] = SmoothQuat[1];
			d->mocap_quat[4 * BaseTargetMocapId + 2] = SmoothQuat[2];
			d->mocap_quat[4 * BaseTargetMocapId + 3] = SmoothQuat[3];
		}
	}

	// Dynamic tracking base: seat the free base on its target for the first few steps, so the stiff
	// weld starts satisfied instead of yanking the arm across the world.
	if (BaseInitStepsLeft > 0 && FreeBaseQposAddr >= 0)
	{
		double tp[3], tq[4];
		if (bSmoothBaseTarget && bSmoothInit)
		{
			tp[0] = SmoothPos[0]; tp[1] = SmoothPos[1]; tp[2] = SmoothPos[2];
			tq[0] = SmoothQuat[0]; tq[1] = SmoothQuat[1]; tq[2] = SmoothQuat[2]; tq[3] = SmoothQuat[3];
		}
		else if (BaseTargetBodyId >= 0)
		{
			const mjtNum* xp = d->xpos + 3 * BaseTargetBodyId;
			const mjtNum* xq = d->xquat + 4 * BaseTargetBodyId;
			tp[0] = xp[0]; tp[1] = xp[1]; tp[2] = xp[2];
			tq[0] = xq[0]; tq[1] = xq[1]; tq[2] = xq[2]; tq[3] = xq[3];
		}
		else
		{
			tp[0] = tp[1] = tp[2] = 0.0; tq[0] = 1.0; tq[1] = tq[2] = tq[3] = 0.0;
		}
		for (int i = 0; i < 3; ++i) d->qpos[FreeBaseQposAddr + i] = tp[i];
		for (int i = 0; i < 4; ++i) d->qpos[FreeBaseQposAddr + 3 + i] = tq[i];
		for (int32 k = 0; k < 6; ++k)
			d->qvel[FreeBaseDofAddr + k] = 0.0;
		mj_forward(m, d);  // refresh kinematics after moving the base
		--BaseInitStepsLeft;
	}

	const mjtNum* sp = d->site_xpos + 3 * EeSiteId;
	const mjtNum* sm = d->site_xmat + 9 * EeSiteId;
	double curQuat[4];
	mju_mat2Quat(curQuat, sm);

	// Base body world pose (identity when not tracking a base frame). The stored target is expressed
	// in this frame, so a moving base carries the target with it.
	const bool bUseBase = bTrackBaseFrame && BaseBodyId >= 0;
	double bpos[3] = {0, 0, 0}, bquat[4] = {1, 0, 0, 0};
	if (bUseBase)
	{
		const mjtNum* bp = d->xpos + 3 * BaseBodyId;
		const mjtNum* bq = d->xquat + 4 * BaseBodyId;
		bpos[0] = bp[0]; bpos[1] = bp[1]; bpos[2] = bp[2];
		bquat[0] = bq[0]; bquat[1] = bq[1]; bquat[2] = bq[2]; bquat[3] = bq[3];
	}

	double tpos[3], tquat[4], grip;
	{
		FScopeLock Lock(&TargetMutex);
		if (!bTargetInit)
		{
			// Seed the target from the current EE pose, expressed in the target frame.
			if (bUseBase)
			{
				double binv[4], off[3];
				mju_negQuat(binv, bquat);
				off[0] = sp[0] - bpos[0]; off[1] = sp[1] - bpos[1]; off[2] = sp[2] - bpos[2];
				mju_rotVecQuat(TgtPos, off, binv);      // world offset -> base frame
				mju_mulQuat(TgtQuat, binv, curQuat);    // world orient -> base frame
			}
			else
			{
				TgtPos[0] = sp[0]; TgtPos[1] = sp[1]; TgtPos[2] = sp[2];
				TgtQuat[0] = curQuat[0]; TgtQuat[1] = curQuat[1]; TgtQuat[2] = curQuat[2]; TgtQuat[3] = curQuat[3];
			}
			bTargetInit = true;
		}
		tpos[0] = TgtPos[0]; tpos[1] = TgtPos[1]; tpos[2] = TgtPos[2];
		tquat[0] = TgtQuat[0]; tquat[1] = TgtQuat[1]; tquat[2] = TgtQuat[2]; tquat[3] = TgtQuat[3];
		grip = GripCtrl;
	}

	// Resolve the stored target into MuJoCo world coordinates for the error computation.
	double wpos[3], wquat[4];
	if (bUseBase)
	{
		double rp[3];
		mju_rotVecQuat(rp, tpos, bquat);            // base frame -> world
		wpos[0] = bpos[0] + rp[0]; wpos[1] = bpos[1] + rp[1]; wpos[2] = bpos[2] + rp[2];
		mju_mulQuat(wquat, bquat, tquat);
	}
	else
	{
		wpos[0] = tpos[0]; wpos[1] = tpos[1]; wpos[2] = tpos[2];
		wquat[0] = tquat[0]; wquat[1] = tquat[1]; wquat[2] = tquat[2]; wquat[3] = tquat[3];
	}

	const int rows = bControlOrientation ? 6 : 3;
	double err[6] = {0, 0, 0, 0, 0, 0};
	err[0] = wpos[0] - sp[0];
	err[1] = wpos[1] - sp[1];
	err[2] = wpos[2] - sp[2];
	if (bControlOrientation)
	{
		double curInv[4], dquat[4], w[3];
		mju_negQuat(curInv, curQuat);
		mju_mulQuat(dquat, wquat, curInv);
		mju_quat2Vel(w, dquat, 1.0);
		err[3] = w[0]; err[4] = w[1]; err[5] = w[2];
	}

	const int nv = m->nv;
	mj_jacSite(m, d, JacP.GetData(), JacR.GetData(), EeSiteId);

	const int K = ArmActIds.Num();
	// J (rows x K): rows 0..2 from JacP, 3..5 from JacR; column j = arm dof address.
	TArray<double, TInlineAllocator<48>> J;
	J.SetNumUninitialized(rows * K);
	for (int r = 0; r < rows; ++r)
	{
		const double* jr = (r < 3) ? (JacP.GetData() + r * nv) : (JacR.GetData() + (r - 3) * nv);
		for (int j = 0; j < K; ++j)
			J[r * K + j] = jr[ArmDofAddr[j]];
	}

	// A = J J^T + lambda^2 I  (rows x rows)
	double A[6][6];
	const double lam2 = (double)DampingLambda * (double)DampingLambda;
	for (int r = 0; r < rows; ++r)
		for (int c = 0; c < rows; ++c)
		{
			double s = 0.0;
			for (int j = 0; j < K; ++j)
				s += J[r * K + j] * J[c * K + j];
			A[r][c] = s + (r == c ? lam2 : 0.0);
		}

	double xsol[6];
	if (!SolveLinear(A, err, xsol, rows))
		return;

	// dq = J^T x ; target qpos = current + StepGain*dq, clamped to joint range.
	for (int j = 0; j < K; ++j)
	{
		double dq = 0.0;
		for (int r = 0; r < rows; ++r)
			dq += J[r * K + j] * xsol[r];

		double tgtq = d->qpos[ArmQposAddr[j]] + (double)StepGain * dq;
		const int jntid = m->actuator_trnid[ArmActIds[j] * 2];
		if (jntid >= 0 && jntid < m->njnt && m->jnt_limited[jntid])
		{
			tgtq = FMath::Clamp(tgtq, (double)m->jnt_range[jntid * 2], (double)m->jnt_range[jntid * 2 + 1]);
		}
		d->ctrl[ArmActIds[j]] = (mjtNum)tgtq;
	}

	if (GripActId >= 0 && GripActId < m->nu)
		d->ctrl[GripActId] = (mjtNum)grip;
}

void URammsMjEndEffectorController::SetTargetPosition(FVector PosMeters)
{
	FScopeLock Lock(&TargetMutex);
	TgtPos[0] = PosMeters.X; TgtPos[1] = PosMeters.Y; TgtPos[2] = PosMeters.Z;
	bTargetInit = true;
}

void URammsMjEndEffectorController::MoveTargetBy(FVector DeltaMeters, FRotator DeltaRotDeg, bool bLocalFrame)
{
	FScopeLock Lock(&TargetMutex);
	if (!bTargetInit)
		return;  // wait for the physics step to seed from the current EE pose

	// --- position ---
	double dpos[3] = {DeltaMeters.X, DeltaMeters.Y, DeltaMeters.Z};
	if (bLocalFrame)
	{
		double rotated[3];
		mju_rotVecQuat(rotated, dpos, TgtQuat);  // delta expressed in the EE frame
		dpos[0] = rotated[0]; dpos[1] = rotated[1]; dpos[2] = rotated[2];
	}
	TgtPos[0] += dpos[0]; TgtPos[1] += dpos[1]; TgtPos[2] += dpos[2];

	// --- orientation (small deltas about X=roll, Y=pitch, Z=yaw) ---
	if (!DeltaRotDeg.IsNearlyZero())
	{
		const double rx = FMath::DegreesToRadians(DeltaRotDeg.Roll);
		const double ry = FMath::DegreesToRadians(DeltaRotDeg.Pitch);
		const double rz = FMath::DegreesToRadians(DeltaRotDeg.Yaw);
		double ax[3] = {1, 0, 0}, ay[3] = {0, 1, 0}, az[3] = {0, 0, 1};
		double qx[4], qy[4], qz[4], dq[4], tmp[4];
		mju_axisAngle2Quat(qx, ax, rx);
		mju_axisAngle2Quat(qy, ay, ry);
		mju_axisAngle2Quat(qz, az, rz);
		mju_mulQuat(tmp, qz, qy);
		mju_mulQuat(dq, tmp, qx);  // dq = qz*qy*qx
		double out[4];
		if (bLocalFrame)
			mju_mulQuat(out, TgtQuat, dq);  // local: post-multiply
		else
			mju_mulQuat(out, dq, TgtQuat);  // world: pre-multiply
		mju_normalize4(out);
		TgtQuat[0] = out[0]; TgtQuat[1] = out[1]; TgtQuat[2] = out[2]; TgtQuat[3] = out[3];
	}
}

void URammsMjEndEffectorController::SetGrip(float Value01)
{
	const float v = FMath::Clamp(Value01, 0.0f, 1.0f);
	FScopeLock Lock(&TargetMutex);
	GripCtrl = FMath::Lerp(GripCtrlRange.X, GripCtrlRange.Y, v);
}

void URammsMjEndEffectorController::ResyncTargetToCurrentPose()
{
	FScopeLock Lock(&TargetMutex);
	bTargetInit = false;  // re-seeded from the live EE pose on the next physics step
}

bool URammsMjEndEffectorController::IsTargetInitialized() const
{
	FScopeLock Lock(&TargetMutex);
	return bTargetInit;
}
