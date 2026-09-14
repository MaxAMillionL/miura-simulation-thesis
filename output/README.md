# Four-panel Miura robot

A Python / MuJoCo simulation of a four-panel Miura-ori robot: four motors per
panel, 16 independently addressable drives, and a confirmed 1:1 motor-to-wheel
ratio. The original meshes are preserved in `assets/`. Generated meshes
separate the three-gear trains and original rotating coupling profiles from
each panel frame.

This trimmed-down version does one thing: build the robot model and drive it
around in the interactive viewer. It does not include CAD-inspection tooling,
render/preview scripts, a hardware bridge, or automated tests.

## Run on this Mac

Open Terminal and run:

```bash
cd /Users/yash/miura-robot
.venv/bin/mjpython -m miura_robot.run --paused --seconds 3600
```

The window starts paused. Press **P** to run physics, then use the controls
below. You can also double-click `Launch-Simulation.command` in Finder. Mouse
drag orbits the camera and scroll zooms; close the window to finish. The
on-screen overlay always shows this same key legend.

| Key | Action |
|---|---|
| Up / Down | Unfold with the motors, then request forward / reverse drive |
| Left / Right | Unfold with the motors, then request left / right turning |
| C / O | Stop the drive request and fold / unfold using motor torque |
| Space | Brake drive wheels and hold the current fold |
| P | Pause / resume physics |
| R / F | Reset to folded / flat inspection pose and pause |

**Tap an arrow to start; tap Space to stop.** Releasing an arrow does not stop
motion: MuJoCo's passive-viewer callback supplies key presses but no
key-release events. Arrows follow panel 0's heading, not the camera. Defaults
are a 0.04 m/s forward request and a 0.4 rad/s turning request; these are
wheel-mixer inputs, not guaranteed chassis speeds.

Normal driving and folding use only the 16 motor actuators. No body forces,
pose assignments or powered hinge actuators produce motion. The eight motors
along shared creases drive their wheels **and** the original interlocking
connectors; their speeds are mechanically constrained by the fold. The other
eight drive the perimeter wheels. Folding lifts those perimeter wheels away
from the ground, so arrow driving first requests a flat pose. Shared wheels
resist rolling; this prototype creeps and slips, and cannot drive freely at a
fixed folded shape with the confirmed rigid 1:1 coupling.

**Simulation torque assumption:** `motor_torque_limit_nm` is **0.08 N·m**. At
that value, the motor controller folds and unfolds under gravity, with finite
tracking error. This is a simulation assumption, not a measured specification
or confirmation that your real motors can lift the robot.

CSV telemetry (time, per-motor targets/speeds/torque, closure error, fold
angle, body position) is written to `output/telemetry.csv` as the sim runs,
with a JSON summary written beside it when the viewer closes. The simulation
uses seconds, metres, kilograms, radians and newton-metres.

For a fresh installation (Python 3.10 or newer):

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e .
.venv/bin/python -m miura_robot.build
```

On macOS, use `mjpython` for the interactive viewer (a native GUI requirement,
not something this project can work around). Linux/Windows can use
`python -m miura_robot.run`.

## Drive from Python

`Simulation` and `RobotMotion` are the two building blocks; the viewer above
is a thin loop around them:

```python
from miura_robot import Simulation
from miura_robot.motion import RobotMotion

robot = Simulation()
motion = RobotMotion(robot)
try:
    motion.set_fold(1.0)             # Close using the motor shafts
    for _ in range(600):
        robot.command(motion.targets(dt=0.01))
        state = robot.step(0.01)
    motion.drive(forward_m_s=0.04)   # Unfold, then drive; negative reverses
    for _ in range(1000):
        robot.command(motion.targets(dt=0.01))
        state = robot.step(0.01)
    motion.stop()                    # Brake and hold; keep refreshing targets
finally:
    robot.stop()                     # Zero torque / coast
```

`motion.drive(yaw_rad_s=0.4)` requests a left turn. `motion.set_fold(0.0)`
opens the panels. `targets(dt)` advances the fold controller's integral
feedback by the control interval; send the returned batch and step physics
for that same interval.

You can also command individual motors directly, bypassing `RobotMotion`:

```python
from miura_robot import Simulation, MotorCommand

robot = Simulation()
try:
    robot.command({'p0_m0': MotorCommand('velocity', 8.0)})
    state = robot.step(0.01)
finally:
    robot.stop()
```

Names use panel index 0–3 and drive index 0–3. Panel colors: 0 cyan, 1 orange,
2 green, 3 purple. In the **original STL XY view**, drive indices are: 0 upper
slanted edge, 1 left vertical edge, 2 right vertical edge, 3 lower slanted
edge. Signs follow the shaft axes recorded in `models/manifest.json`; a
positive command does not mean the same world direction on every folded
panel.

## Mechanical model and assumptions

Confirmed: four panels, four motors on each panel, 1:1 transmission, the
supplied motors fitting beside the gears, and each output shaft driving both
its wheel and its mating interlock. Motor specifications are not yet
available.

The STL envelope is approximately 141.802 × 205.160 × 12 source units; source
units are **assumed to be millimetres**. The wheel is approximately 16 mm in
diameter and 4 mm thick. STL contains geometry, not assembly constraints,
material data or a motor datasheet.

The panels connect **directly through their original interlocking profiles**.
There are no generated brackets, rods, sleeves or spacers. The nominal facet
edge is 150 mm, with zero separation between mating axes. The wheel and motor
placements relative to each panel come from the original CAD.

Three hinge joints and a fourth-crease closure constraint make a single-vertex
Miura cell. The unfolded crease rays are 0°, 60°, 180° and 300°, giving one
internal folding degree of freedom away from the flat singularity. The eight
shared-edge output shafts drive these creases through joint constraints; there
are no additional fold actuators.

Each drive has three separate rotating gears: motor/input gear → intermediate
gear → outer/output gear → wheel. The intermediate gear reverses direction,
and the output returns to the input direction at the confirmed overall 1:1
ratio. Ideal joint constraints transmit the motion; tooth contact, backlash
and electrical motor dynamics are not simulated.

Inter-panel self-collision uses inset frame cores, motor boxes and wheel
cylinders. Ground contact uses full frame hulls. **The interlocks use ideal
hinge motion**: the model permits intersection inside the existing lobed
coupling and circular bearing regions during folding rather than simulating
tooth disengagement, elastic deformation or detailed bearing clearance. Mass,
torque, speed and friction also need measured values.

## Calibrate and rebuild

Edit `config.json`, then run:

```bash
.venv/bin/python -m miura_robot.build
```

The model uses a snapshot of the configuration in `models/manifest.json`;
changing the JSON alone does not change an already-built model.

| Parameter | Initial value | Status |
|---|---:|---|
| Motor-to-wheel ratio | 1:1 | Confirmed |
| Torque limit | 0.08 N·m | Assumed for lifting tests; real motor rating unknown |
| Speed limit | 40 rad/s | Placeholder |
| Panel-frame mass | 0.12 kg | Placeholder |
| Motor / wheel / gear mass | 0.010 / 0.003 / 0.002 kg | Placeholders |
| Gear efficiency | 0.8 | Placeholder |
| Tire friction | 0.9 | Placeholder |
| Initial crease angle | 0.55 rad | Configurable initial condition |
| Wheel axial placement | 9 mm from outer gear center | Inferred, needs confirmation |
| Mating-axis separation | 0 mm | Direct original-profile engagement |

`cad_calibration` stores the nominal connector vertex/edge length, 11 mm
gear-center spacing, motor shaft directions, motor placement and the two
lobed interlock stations in source coordinates. `fixed_base: true` anchors
panel 0 for bench tests. Panel inertia and rotor armature are approximate;
measured mass properties will improve dynamics.

## Source references

The model uses [MuJoCo joint actuators and connect
constraints](https://mujoco.readthedocs.io/en/latest/XMLreference.html), and
the GUI follows the [Python passive-viewer API and macOS launcher
requirement](https://mujoco.readthedocs.io/en/latest/python.html). The solver
treats the fourth crease as a soft equality constraint; closure residual is
logged so its accuracy can be assessed under motion.
