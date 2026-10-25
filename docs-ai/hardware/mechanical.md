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
- A straight bevel gear pair turns the drive 90° onto the horizontal wheel axle (ratio **[?]**).
- Wheel: the larger size of inline-skate wheel, D = 123 mm, 24 mm wide.
- Top speed is 0.4 m/s, which is also the firmware cap `CASTER_DRIVE_MAX_SPEED`.
- The electrical schema gives drive motor stall as about 6 A at 16 V. In normal use each caster gets an 11 V / 45 W supply (see [electrical.md](electrical.md)).
- The motor bodies are wrapped in aluminium foil, and the leads carry ferrite coils to suppress EMI.
- Motor model and gearbox ratio **[?]**

### Steering

- A similar Chinese DC gear-motor, in `rotation-motor-holder`. It drives a 2GT timing belt (closed, 610 mm) onto a **176-tooth 2GT pulley** (`drive-rotation-tgear176`, Ø112 mm) fixed to the caster.
- A belt idler/tensioner sits on a threaded rod (`rotation-idler.stl`).
- Motor pulley tooth count **[?]**, so the overall steering ratio **[?]**.
- **The steering axle is hollow.** The drive motor and IR sensor cables pass through it. Mechanically the caster could rotate without limit, but the cables do not allow it.
- The limit is in software:
  - The firmware keeps each caster within (−180°, +180°) of the forward-zero position and never wraps.
  - `optimize_rotation()` chooses either the target angle or the opposite angle with the drive reversed. It prefers the smaller absolute angle and the shorter travel.
  - The URDF limit is ±90°, which is inconsistent with the firmware.
- Steering end-stop switches are in the electrical design. Whether they are installed **[?]**

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

- **Mast:** a vertical aluminium profile about 1.1–1.2 m high (83 × 20 mm section, with a pattern of pockets) on the base (`base_link.FCStd`, "torso").
- **Lift:** NEMA 17 stepper driving a **trapezoidal lead screw with a nut** (`torso_lift_link/trapezoidal-nut-holder`, `base_link/torso-stepper-holder`, `torso-stepper-bearing`). There is also a safety end-stop part (`backup/torso-security-endstop.stl`).
- **Body:** the movable body plate is mounted, with nothing on it yet.
- **Status:** the stepper is mounted but probably not wired.
- The electrical schema plans a stepper driver (A4988 / TMC2130) and a zero-pose end-stop.
- Screw pitch and travel range **[?]**

## 5. Other mounts

- Lidar (XV-11) on the base: URDF gives (0.12, 0.13, 0.2325) m **[?]**.
- IMU holder (`electrical/imu-holder.stl`): URDF gives (−0.10, 0, 0.16) m **[?]**.
- WiFi antenna holder (TP-Link antenna) and the control panel holder (`electrical/antenna-holder*`, `controlpanel*`).
- Kinect: a temporary head mount exists in CAD (`head_pan_link/temp-kinect.stl`), along with head pan link drafts **(planned)**.
