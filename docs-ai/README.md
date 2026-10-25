# Bobik Documentation

This is the entry point to the AI-maintained documentation. Everything in `docs-ai/` was written by Claude. Honza's original design files (drawio, ods, FreeCAD, PDF) stay in `docs/` and are referenced from here.

| Document | Contents |
|---|---|
| [project-goals.md](project-goals.md) | Why the project restarted, principles, budget, milestones |
| [plan.md](plan.md) | Target architecture and the phased plan |
| [hardware/mechanical.md](hardware/mechanical.md) | Frame, caster modules, torso, dimensions, kinematic tree |
| [hardware/electrical.md](hardware/electrical.md) | Power distribution, motor drivers, Arduino and Jetson pinouts, sensors |
| [hardware/planned-extensions.md](hardware/planned-extensions.md) | Arm, head and other designed-but-not-built parts |
| [software-legacy.md](software-legacy.md) | How the 2023 software worked: protocol, control, odometry, known defects |
| [open-questions.md](open-questions.md) | The single list of unknowns, grouped by topic |

## Robot at a glance (October 2026)

| Subsystem | State |
|---|---|
| Triangular aluminium base, 3 swerve casters (steer + drive) | built; not driven for about 3 years |
| Torso mast with lift (NEMA 17 + trapezoidal screw) | built, stepper not wired, body plate empty |
| Head, arm | designed only |
| Jetson TK1 + Arduino Mega 2560 | built |
| Li-ion 4S 15 Ah + BMS, charge/power-hold module, motor relay, emergency switch | built |
| XV-11 lidar, Kinect v1, MPU-9150 IMU | built. Lidar and Kinect hardware work |
| WiFi 5 GHz + Ethernet | built |

## Known weaknesses that drove the restart

1. **Wheel odometry.** Single-channel IR ticks with no direction, missed edges, and the rear caster about 18% off.
2. **L298N drivers + cheap gear-motors.** Voltage drop, deadband, and heavy tuning hacks.
3. **Arduino ⇄ Jetson link.** 19200 baud through a 1.8 V level shifter, fragile framing.
4. **Jetson TK1 stuck on Ubuntu 14.04 / L4T**, so ROS cannot run on the robot.
5. **IMU orientation unusable.**
6. **WiFi latency** on a shared home AP.

## Where the original material is

| Path | What |
|---|---|
| `docs/electrical/electrical-schema_v1.drawio` | Power distribution (as built), system block diagram, arm notes, AS5048A wiring |
| `docs/electrical/baterie.ods` | Power budget and battery sizing |
| `docs/mechanical/mechanical-schema_v1.drawio` | Link/joint tree (PR2 naming) |
| `docs/mechanical/arm.FCStd` | Arm design |
| `docs/CAD/` | FreeCAD models and STL of all printed and aluminium parts. `backup/` holds earlier drive-module iterations and swerve studies |
| `docs/software/*.drawio`, `odometry.ods` | Firmware timing, odometry derivation, UI concept, odometry calibration data |
| `urdf/`, `meshes/` | Current robot model |
| `bobik_arduino/README.md` | Arduino pinout and wire colours |
| `bobik_driver/README.md` | Jetson TK1 installation notes (L4T 21.3) |
| `bobik_arduino/docs/` | Caster PID notes, IR tick and steering optimisation spreadsheets |

Photos taken on 2026-10-25 (casters, steering belt, IR encoder, AS5048A) are kept in the project files (`bobik/photos/`). They are not in git.
