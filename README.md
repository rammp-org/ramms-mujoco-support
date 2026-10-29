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
| position servo (`<position>`, or `<general>` with an affine bias), velocity servo | a force-mode drive with the MJCF's kp / kv. The force limit is the force range × `ServoForceLimitScale` (default 10; Chaos' limit does not carry over 1:1) |
| `<motor>`, joint `damping`, `stiffness`, `frictionloss` | forces applied by a physics-rate callback (`ISimCallbackObject`), equal and opposite on the joint's two bodies |
| fixed tendon actuator (the 2F-85's) | per-joint drives with coefficient-scaled gains |
| parent/child, `<exclude>`, contype/conaffinity | `SelfCollision = MuJoCo`: those pairs don't collide, everything else does. `Off`: no part collides with another (RobotSelf channel) |

The articulation's body components, and the geom visuals under them, follow the Chaos bodies every
frame. Controls use the MJCF actuator's own ctrl units (`SetControl`, `ApplyKeyframeControls`), so a
keyframe's ctrl vector gives the same stance on either engine. `GetJointPosition` / `GetJointVelocity`
report joints in MuJoCo's units, with an unlimited hinge (a wheel) counted past ±π.

Not carried:
- soft contacts and soft limits (Chaos' are hard: a door stops at its limit where MuJoCo's bounces back)
- torsional and rolling friction
- spatial tendons and joint equalities
- a second joint in one body
- geoms that exist only in the compiled model

The level must not also have an `AAMjManager`, or MuJoCo simulates the articulation too. Tests and
measurements are in ramms-sim's `Scripts/pie_tests/chaos_articulation/` and
`doc/chaos_physics_backend_plan.md`.
