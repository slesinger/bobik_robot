# Mechanical

Notation: no tag means documented in the original design files and confirmed by Honza. **(planned)** means designed but not built. **[?]** means unverified or contradictory; these are collected in [../open-questions.md](../open-questions.md).

## 1. Overall layout

- **Concept:** a PR2-inspired home robot with a holonomic base, a torso lift, and later a head and an arm. Joint and link names follow the PR2 (`docs/pr2_manual_r321.pdf`).
- **Installed today:**
  - triangular aluminium base with 3 swerve casters
  - torso mast with a lift (body plate empty)
  - Jetson TK1, Arduino Mega, battery and power electronics
  - XV-11 lidar, Kinect v1
- **Not built:** head and arm **(planned)**.
- **Overall envelope** from `docs/CAD/bobik-assembly.FCStd`: about 0.52 × 0.55 m footprint and about 1.2 m high, including the torso mast.
- **Mass:** 11.5 kg as of 2026-11-23, and expected to grow as hardware is added (the URDF masses are rough placeholders).

### Kinematic tree (`docs/mechanical/mechanical-schema_v1.drawio`)

```
base_link
├── {fl,fr,r}_caster_rotation_joint (revolute, vertical) → {fl,fr,r}_caster_rotation_link
│   └── {fl,fr,r}_caster_wheel_joint (continuous) → {fl,fr,r}_caster_wheel_link
├── lidar_joint (fixed) → lidar_link
├── imu_joint (fixed) → imu_link
└── torso_lift_joint (prismatic) → torso_lift_link
    ├── head_pan_joint → head_pan_link → head_tilt_joint → head_tilt_link → rgbd_joint → rgbd_link   (planned)
    └── shoulder_pan_joint → … → gripper  (planned, see planned-extensions.md)
```

The current URDF (`urdf/bobik_robot.urdf.xacro`) models only the base, the casters, the lidar and the IMU.

## 2. Base frame

- Welded and bolted aluminium profiles, 20 × 40 mm, forming a triangle. From CAD (`docs/CAD/bobik-urdf-meshes.FCStd`, `base_link/base_link.FCStd`):
  - front side 550 mm
  - left and right sides 530 mm
  - a 20 × 80 × 120–144 mm stand at each corner carries a caster
- A 5 mm acrylic floor plate (about 482 × 556 mm) and a "y-bottom" inner frame carry the electronics. 3D-printed holders are used for the L298N boards, DC-DC converters, charger shelves, power switch and control panel (`docs/CAD/base_link/el-*.stl`, `docs/CAD/electrical/*.stl`).
- **The frame is considered fixed.** Changes to it are out of scope.

### Caster positions (steering axes)

These are in `base_link`, REP-103 (X forward, Y left), in metres.

| Caster | Code (`robot_config.h`) | URDF |
|---|---|---|
| FL ("A") | (0.132, +0.2285) | (0.134, +0.230, 0.173) |
| FR ("B") | (0.132, −0.2285) | (0.134, −0.230, 0.173) |
| R ("C") | (−0.264, 0) | (−0.270, 0, 0.173) |

- The steering axes form an equilateral triangle with side `LEN_AB = 0.457 m`. The robot centre is the centroid: 0.264 m to each axis.
- The firmware uses Y pointing right internally, and the bridge flips it to REP-103.
- The small code/URDF mismatch should be unified to one source in the new software.

## 3. Caster module (×3, swerve type)

Each caster steers and drives independently. The design went through several iterations: `docs/CAD/backup/` contains a differential drive module, a v3 swerve and studies of the SDS MK4 / Falcon swerve modules. The built version is `caster_rotation_link` + `drive-module-holder_v2`.

```
        frame stand ──┐
   steering motor ─ 2GT belt (610 mm) ─ 176T pulley (Ø112 mm) on hollow steering axle (Ø12 mm)
                                 │  AS5048A above the axle, magnet on the axle
                       caster_rotation_link (bracket from 60×60×4 L-profile, 80 mm wide)
                                 │
   drive gear-motor (vertical) ─ straight bevel gear pair ─ horizontal wheel axle (Ø8 mm)
                                 │                                  │
                        inline-skate wheel Ø123 mm, 24 mm wide    code wheel Ø68 mm, 120 slots
                                                                   + optical fork sensor
```

### Drive

- Chinese DC gear-motor with a built-in gearbox, mounted vertically in a 3D-printed clamp (`drive-motor-holder-a/b.stl`).
- A straight bevel gear pair turns the drive 90° onto the horizontal wheel axle (ratio 1:1).
- Wheel: the larger size of inline-skate wheel, D = 123 mm, 24 mm wide.
- Top speed is 0.4 m/s, which is also the firmware cap `CASTER_DRIVE_MAX_SPEED`.
- The electrical schema gives drive motor stall as about 6 A at 16 V. In normal use each caster gets an 11 V / 45 W supply (see [electrical.md](electrical.md)).
- The motor bodies are wrapped in aluminium foil, and the leads carry ferrite coils to suppress EMI.
- Motor: no label or datasheet left (the foil wrap is not to be removed). Body Ø24.4 mm, about $15, built-in gearbox, nominal 12 V; probably driven from the battery voltage by PWM. Most likely the 25GA-370 / JGA25-370 family.
- Gearbox: **estimated 60 rpm output** (Honza recognises 60 rpm). Derived, not measured: 0.4 m/s is the true maximum, wheel circumference π × 123 mm = 386 mm, so 1.035 rev/s ≈ 62 rpm at the wheel. With the 1:1 bevel pair that equals the gearbox output speed. The motor ratio is probably about 1:100.
- To verify (not now, robot stays off): at 100% PWM, wheel rpm = IR ticks per second ÷ 240 × 60.

### Steering

- A similar Chinese DC gear-motor, in `rotation-motor-holder`. It drives a 2GT timing belt (closed, 610 mm) onto a **176-tooth 2GT pulley** (`drive-rotation-tgear176`, Ø112 mm) fixed to the caster.
- A belt idler/tensioner sits on a threaded rod (`rotation-idler.stl`).
- Motor: same family as the drive motor but a different gearbox. Honza thinks it is **about 2× the drive speed, ≈120 rpm** (not verified).
- Motor pulley: **15 teeth** (counted by Honza, may be a miscount; it could be a 16-tooth GT2 pulley with 5 mm bore, bought as a 2-pulley set with a belt). A smooth 20-tooth, 5 mm bore, 6 mm wide GT2 idler tensions the belt. Belt ratio 176 ÷ 15 = 11.7:1.
- Expected caster speed from these numbers: 120 rpm ÷ 11.7 ≈ 10 rpm, so 360° would take about 6 s.
- **Inconsistent with memory:** Honza remembers a full caster turn taking about 3 s (≈ 20 rpm). That needs either a motor output of about 235 rpm (a 4× drive-speed gearbox) or a 30-tooth motor pulley. Resolve by recounting the pulley teeth, or by logging the AS5048A angle against time at full PWM. Belt length alone cannot decide: with 610 mm the centre distance would be about 210 mm (15T) or 202 mm (30T).
- **The steering axle is hollow.** The drive motor and IR sensor cables pass through it. Mechanically the caster could rotate without limit, but the cables do not allow it.
- **Cable limit:** the cables in the axle survive at most **2 full rotations** (Honza). There are no end-stop switches; the limit is purely in software (read from `bobik_arduino`, 2026-11-23):
  - The AS5048A is read as a single-turn angle in units −8192…+8191 (±180°) around the zero offset (`Caster::getRotation`). **The firmware has no multi-turn counter.** It cannot know how many turns the cable has taken.
  - Every target is wrapped into ±π (`simplify_rad`, `setCmdVel`), and the steering control (`Caster::execute`) is a plain `target − current` in those ±180° units. It never goes through the ±180° seam in the short direction; it always turns back through zero. So the caster stays in (−180°, +180°) and the cable never takes more than half a turn each way.
  - `optimize_rotation()` chooses between the target angle and the opposite angle (with the drive reversed), by the cost |angle| + 0.8 × travel. This keeps the casters mostly within about ±90° of forward, so the practical range is about ±90° (matching the URDF limit ±π/2) and only goes further when the caster is already beyond it, with hysteresis.
  - **There is no explicit "unwind when maxed out" routine** in the Arduino code. The unwinding Honza remembers is the effect of the above: when the commanded angle jumps across the seam, the caster swings back through zero (up to about 360° of travel). While the caster is more than 0.5 rad (28°) off target, `STOP_DUE_ROTATION_DIFF` stops the drive.
  - Consequence for the redesign: the ±180° wrap and the swing-back are safe for the cable (never beyond 1 turn of the 2 allowed), but a new controller that counts turns or takes the shortest path across the seam would cross the physical limit. Keep the single-turn range ±180° or add a real turn counter and explicit limits.
- Steering end-stop switches: none installed (Honza, 2026-11-23: "no end stop switches yet").

### Caster sensors

| Sensor | Part | Mounting | Resolution |
|---|---|---|---|
| Steering angle | AS5048A, 14-bit absolute magnetic, SPI | transparent 3D-printed holder (`rotation-absolute-holder.stl`) on the frame above the hollow axle. Neodymium magnet in `rotation-magnet-holder.stl` on the axle | 16384 counts/rev (45.5 counts/°). Zero offsets FL 275, FR 355, R 350 |
| Wheel travel | IR optical fork (slotted interrupter), **single channel** | grey 3D-printed housing (`wheel-odometry-holder.stl`) beside the wheel, reading a 3D-printed comb code wheel (`wheel-codewheel-radial*.stl`, Ø68 mm, 120 slots) on the wheel axle | 240 edges/rev, ≈ 1.61 mm/edge. **No direction information** |

### Measured odometry error (`docs/software/odometry.ods`)

- **Driving test:** the robot drove known distances at 0.4 m/s in several directions, and the reported ticks were compared with the real distance.
  - FL needed ×1.02–1.04
  - FR needed ×1.02–1.06
  - **R needed ×1.15–1.21**
- **Hand test:** when the wheels were turned by hand, they gave 223–240 edges per revolution instead of 240, so edges are lost.
- The bridge hard-codes `REAL_WORLD_COEF_A/B/C = 1.028 / 1.028 / 1.15`.

## 4. Torso

- **Mast** (Czech: svislý nosný sloup / stožár; the vertical milled aluminium column the torso moves on): about 1.1–1.2 m high, 83 × 20 mm section with a pattern of pockets. Dimensions are from Honza's precise CAD model and are trusted on the base (`base_link.FCStd`, "torso").
- **Lift:** NEMA 17 stepper driving a **trapezoidal lead screw with a nut** (`torso_lift_link/trapezoidal-nut-holder`, `base_link/torso-stepper-holder`, `torso-stepper-bearing`). There is also a safety end-stop part (`backup/torso-security-endstop.stl`).
- **Body:** the movable body plate is mounted, with nothing on it yet.
- **Status:** the stepper is mounted but probably not wired.
- The electrical schema plans a stepper driver (A4988 / TMC2130) and a zero-pose end-stop.
- **Decks:** the robot has a **lower deck** (5 mm acrylic floor plate) carrying the torso stepper, battery, Arduino and the power distribution, and an **upper deck** carrying the Jetson TK1.
- **Lead screw (caliper, 2026-11-23, error about ±0.2 mm):**
  - Length 98.5 cm, including the shaft coupler, from the lower deck up to the end of the rod. This is not the usable travel.
  - Major diameter 9.75 mm, thread pitch 2.8 mm. This points to **Tr10×3** (metric trapezoidal, 10 mm nominal, 3 mm pitch); a screw measured slightly under its nominal size is normal. It is not T8 (my first guess from the ruler photo was wrong). Single-start is confirmed by the turn test: 10 turns of the rod moved the nut 30.3 mm, so the lead is **3.03 mm/rev** (a single-turn reading of 3.34 mm is within hand-measurement error).
  - Resulting resolution (if Tr10×3, single start): 200 full steps/rev → 0.015 mm per full step, about 1 µm at 1/16 microstepping. A 1 m traverse is about 333 revolutions.
- **Stepper:** NEMA 17, 40 N·cm, 3.96 V per coil (AliExpress listing "Schrittmotor Nema 17 40Ncm", current not known). It sits on the lower deck in an orange 3D-printed holder. Honza has several more of the same.
- **Coupler:** flexible CNC aluminium shaft coupler, D19 × L25 mm, with a choice of bore sizes up to 10 mm. A 10 mm bore on the screw fits the Tr10 diameter. The motor side is the usual 5 mm NEMA 17 shaft.
- **Carriage:** the nut is held in an orange 3D-printed block (`trapezoidal-nut-holder`) that runs in a window of the milled aluminium mast. The body plate slides on a round chrome guide rod through two zinc-alloy blocks (hardware blocks, not linear bearings) on the side.
- **End-stops:** none installed yet (Honza, 2026-11-23). The torso can drive into its mechanical limits, so wiring needs a homing switch and a safety limit.
- Usable travel: **a design decision, not yet fixed.** It depends on how the moving torso part is designed on the rod. Upper bound: the 98.5 cm rod length minus the coupler (25 mm), the nut block and safety margins at both ends. The carriage in the 2026-11-23 photo is somewhere in between the limits; no height was measured.

## 5. Other mounts

- Lidar (XV-11) on the base: (0.12, 0.13, 0.2325) m from `base_link`, taken from the URDF (Honza: the URDF is the reference).
- IMU holder (`electrical/imu-holder.stl`): (−0.10, 0, 0.16) m from `base_link`, taken from the URDF.
- WiFi antenna holder (TP-Link antenna) and the control panel holder (`electrical/antenna-holder*`, `controlpanel*`).
- Kinect: a temporary head mount exists in CAD (`head_pan_link/temp-kinect.stl`), along with head pan link drafts **(planned)**.
