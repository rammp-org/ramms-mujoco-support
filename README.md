# ramms-mujoco-support
Plugin providing Ramms support utilities, classes, and content associated with MuJoCo simulation

## Running an articulation on Chaos

`URammsChaosArticulationComponent` simulates a URLab articulation with Chaos instead of MuJoCo. The
Chaos bodies live in the world's own physics scene, so the articulation collides with, pushes and is
pushed by every other Chaos object. Add the component to an `AMjArticulation`, or call
`URammsChaosArticulationComponent::SwitchToChaos(Actor, Keyframe)` at runtime.

At BeginPlay the component compiles the articulation's spec with MuJoCo. MuJoCo is used only as the
model compiler, which resolves every default, mass, inertia, convex mesh, joint axis and actuator gain.
From the compiled model the component builds:

| MJCF | Chaos |
|---|---|
| body with a joint (or a free joint) | a `URammsChaosBodyComponent` carrying the body's collision geoms, plus those of the jointless bodies welded to it. Mass, centre of mass and inertia come from the MJCF (combined across welded bodies, plus hinge armature about its axis), not from the shapes |
| body with no joint whose parent is the world; mocap body; body welded to one | a body that does not simulate |
| sphere, capsule, box | the same primitive |
| mesh, cylinder (`CylinderSides`), ellipsoid | a convex element, cooked from its points |
| sliding friction | a physical material per body: the largest geom friction, combined by max as MuJoCo does, restitution 0 |
| hinge, slide, ball | a constraint with the joint's range. An asymmetric range is centred by offsetting the child frame by mid-range |
| `<connect>` / `<weld>` between bodies | a point / locked constraint (closes kinematic loops) |
| position servo (`<position>`, or `<general>` with an affine bias), velocity servo | a force-mode drive with the MJCF's kp / kv and its force range as the limit (× `ServoForceLimitScale`, default 1) |
| joint `damping`, `stiffness` (and `springref`) on a joint no actuator drives | the joint constraint's own drives, solved implicitly (`bPassiveJointDrives`, default on): a velocity drive for the damping, a position drive toward `springref` for the spring. Explicit forces at Chaos' 10 ms step ring and blow up on light parts (a 35 g knob, a 6 g button) |
| `<motor>`, damping, stiffness and `frictionloss` on an actuated joint | forces applied by a physics-rate callback (`ISimCallbackObject`), equal and opposite on the joint's two bodies |
| fixed tendon actuator (the 2F-85's) | per-joint drives with coefficient-scaled gains |
| parent/child, `<exclude>`, contype/conaffinity | `SelfCollision = MuJoCo`: those pairs don't collide, everything else does; when that leaves no pair (the usual robot MJCF: every part contype 1, conaffinity 0) it is `Off`. `Off`: no part collides with another (RobotSelf channel) |

The articulation's bodies get 64 position / 8 velocity solver iterations (`SolverPositionIterations`,
`SolverVelocityIterations`) and their joints no mass conditioning (`bJointMassConditioning`). A drive is
only as stiff as the solver converges along the chain: at the project's 8 / 1 a Gen3 shoulder sagged
ten times what its kp gives on MuJoCo, which the old ×10 force limit papered over. With these it sags
1.5×, and a saturated step reaches its target as fast as on MuJoCo with the MJCF force range as is.

The articulation's body components, and the geom visuals under them, follow the Chaos bodies every
frame. The URLab components lose their collision (the Chaos bodies answer queries), which also keeps
following them cheap. Controls use the MJCF actuator's own ctrl units (`SetControl`, `ApplyKeyframeControls`), so a
keyframe's ctrl vector gives the same stance on either engine. `GetJointPosition` / `GetJointVelocity`
report joints in MuJoCo's units, as of the latest physics step (async physics interpolates the bodies'
game-thread transforms a step or more behind), with an unlimited hinge (a wheel) counted past ±π and a
limited one measured about its mid-range (a washer door's 0..180° reads 180°, not -180°).

Chaos scales an angular drive's stiffness and damping by `p.Chaos.JointConstraint.AngularDriveStiffnessScale`
/ `...DampingScale` (1.5 by default) where a linear drive's are not; the component divides that back out,
so an MJCF kp or damping means the same thing on both engines.

### Robot controllers on Chaos

To run a robot on Chaos, set its robot base component's **Backend** to **Chaos**. RammsMujocoSupport
registers RammsCore's Chaos articulation backend, which switches the robot's URLab articulation to
Chaos at BeginPlay (`SwitchToChaos`, starting in its `URammsMjSimControlComponent::InitialKeyframe`
when it has one) and drives it. A robot with no articulation falls back to RammsCore's deprecated
skeletal Chaos backend. **MuJoCo** and **Auto** drive the articulation on whichever engine simulates
it: on Chaos while it carries an active `URammsChaosArticulationComponent` (in its Blueprint, or added
by `SwitchToChaos` at runtime), on MuJoCo otherwise. Either way the robot's controllers, control
surface and input maps work unchanged:

| `IRammsActuationBackend` | on Chaos |
|---|---|
| motor Id | the MJCF actuator name, as on MuJoCo |
| `SetCommand` | `SetControl` in ctrl units (a value set before the build is kept until it). A position servo on a slide takes its target in cm, as `GetValue` reads the joint and as on MuJoCo, and the control gets metres |
| `GetValue` / `GetVelocity` | the actuator's joint, rad or cm (cm/s) |
| `GetMotorTransform` | the MuJoCo body that joint moves |
| `SetVelocityDrive` | a torque motor's speed loop: the proportional term is the joint constraint's velocity drive, solved implicitly; the integral is applied by the physics-rate callback. An explicit loop at Chaos' 10 ms step is unstable at a wheel's gains once the light link it hangs from is in the loop |
| `SetMotorBrake` | the joint constraint holds a torque motor's joint where it stands, up to the motor's torque |
| `SimulatesMotorJoint` | false for a torque motor, so a drive controller parks with the brake rather than its own torque hold on the wheel angle, which, closed once a game frame, rings against the implicit solve |
| `ReleaseMotor` | a servo's drives switch off until its next command; a motor goes to zero |
| `GetSimulationTime` | the game's clock (Chaos steps with it) |

The log names the engine (`LogRammsArticulationBackend: <robot>: motors driven through Chaos`) when
the robot first moves and again on a switch.

Not carried:
- soft contacts and soft limits (Chaos' are hard: a door stops at its limit where MuJoCo's bounces back)
- torsional and rolling friction
- spatial tendons and joint equalities
- a second joint in one body
- geoms that exist only in the compiled model

## Choosing the engine per level

The same articulation assets run on either engine; the level picks which. Put an
**`ARammsPhysicsSettings`** actor in the level and set its **Engine**:

| Engine | at BeginPlay |
|---|---|
| **Auto** (default) | the level as placed: MuJoCo simulates what its `AAMjManager` compiles, Chaos what carries an active `URammsChaosArticulationComponent` (such a level must not also have an `AAMjManager`, or MuJoCo simulates that articulation too). A level with no settings actor behaves the same, so existing levels are unchanged |
| **MuJoCo** | spawns an `AAMjManager` when the level has none (and starts it unless `bStartMuJoCoRunning` is off), turns off every Chaos articulation component, and, when the level has no static MuJoCo geometry (`UMjQuickConvertComponent` with Static, or a heightfield) and `bMuJoCoGround` is on, adds a hidden static slab at the floor under the articulations so they have something to stand on |
| **Chaos** | removes every `AAMjManager` before it compiles and switches every articulation to Chaos (`SwitchToChaos`, starting in its `URammsMjSimControlComponent::InitialKeyframe` when it has one), including ones spawned later |

A run overrides the level with `-RammsPhysics=MuJoCo|Chaos|Auto` on the command line or the
`ramms.Physics.Engine` console variable (the command line wins); `URammsPhysicsEngineSubsystem::GetEngine()`
says which engine the level ended up on, and the log names it (`LogRammsPhysicsEngine: Chaos simulates the
level's 9 articulation(s)`).

Self-collision: Chaos never collides a joint's two bodies. MuJoCo filters the same pair only when the
parent moves; a body hinged to a base welded to the world (a cabinet's door, a bin's lid) collides with
it unless the MJCF `<exclude>`s the pair. RammsWorldGen's `Tools/simready_mjcf.py` writes those
`<exclude>`s, so its SimReady models open and close alike on both engines.

Tests and measurements are in ramms-sim's `Scripts/pie_tests/chaos_articulation/` and
`doc/chaos_physics_backend_plan.md`.
