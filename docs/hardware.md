# Bobik – Hardware Description

> **Status: DRAFT for review.** I reconstructed this from the code and design files in the Bobik repositories (last activity 2022–2023) and from Honza's answers (2026-10-25).
> Each fact has a source tag:
> - **[H]** confirmed by Honza
> - **[C]** read from code or config (what the software assumes)
> - **[D]** from design or planning files (drawio, ods, CAD); may never have been built
> - **[?]** unknown or contradictory, needs checking
>
> Please correct anything that is wrong and remove the tag once it is verified on the real robot.

## 1. Overview

Bobik is a home service robot inspired by the Willow Garage PR2 (the PR2 manual is in `docs/pr2_manual_r321.pdf`, and joint names follow PR2 conventions). It has not been driven for about 3 years. **[H]**

| Subsystem | Installed today | Notes |
|---|---|---|
| Triangular aluminium base with 3 steered + driven casters | yes **[H]** | aluminium frame is considered fixed **[H]** |
| Torso with a movable body (torso lift) | frame yes, nothing mounted on the body **[H]** | NEMA 17 stepper with lead screw mounted, probably not wired yet **[H]** |
| Arm with gripper | no **[H]** | design exists (`docs/mechanical/arm.FCStd`, electrical schema) **[D]** |
| Head (pan/tilt) | **[?]** | only in the design files **[D]** |
| On-board computer: NVIDIA Jetson TK1 | yes **[H]** | |
| Real-time controller: Arduino Mega 2560 | yes **[H]** | |
| Battery: Li-ion 4S 14.4 V, 15 Ah, with BMS | yes **[H]** | made by a Czech company **[H]** |
| Charging circuit and barrel jack (~18 V laptop adapter, ~70 W) | yes **[H]** | |
| Lidar: Neato XV-11 | yes, working **[H]** | |
| RGB-D: Microsoft Kinect v1 | yes, HW working **[H]** | software was the problem |
| IMU: MPU-9150 (GY-9150 board) | **[C]** | used by the firmware |
| Networking | WiFi and Ethernet **[H]** | 5 GHz WiFi, Intel 7260 card mentioned **[C]** |

Overall mass in the arm design notes: about 3 kg plus 300 g payload **[D]**. The real mass is **[?]**.

## 2. Mechanical – Base

### 2.1 Geometry

- The 3 casters sit near the corners of an equilateral triangle: front-left (FL, "A"), front-right (FR, "B") and rear (R, "C"). **[H][C]**
- Distance between steering axes: `LEN_AB = 0.457 m`. **[C]** (`bobik_arduino/lib/robot/robot_config.h`)
- Robot centre S is the triangle centroid:
  - centre to each steering axis = 0.264 m
  - centre to front edge = 0.132 m **[C]**
- Steering axis positions in `base_link` (REP-103: X forward, Y left):
  - FL (0.132, +0.2285)
  - FR (0.132, −0.2285)
  - R (−0.264, 0)

  **[C]** The firmware uses Y pointing right internally, so its signs are flipped. The URDF has slightly different values: FL (0.134, 0.23, 0.173), FR (0.134, −0.23, 0.173), R (−0.270, 0, 0.173) **[C]** (`urdf/bobik_robot.urdf.xacro`).
- The bridge code also defines an unused `A_TRIANGLE_SIDE 0.53`, possibly the outer frame side. **[?]**
- Nav2 footprint: `robot_radius: 0.75` **[C]**. This looks too large for a base of about 0.5 m and should be checked. **[?]**

### 2.2 Caster module (×3)

Each caster can steer and drive independently, like a swerve module. **[H][C]**

**Drive:**
- Chinese DC motor with a built-in gearbox, mounted vertically. **[H]**
- It drives the wheel directly through a 90° gear (bevel or crown). **[H]**
- Wheel: the larger size of inline-skate wheel. **[H]** The code uses D = 123 mm and the URDF uses a width of 24 mm. **[C]**
- Top speed of the robot is about 0.4 m/s. **[H]** The firmware cap is `CASTER_DRIVE_MAX_SPEED 0.4`. **[C]**
- Motor stall current is about 6 A at 16 V (~90 W). **[D]** (`electrical-schema_v1.drawio`)
- Exact motor model and gear ratio **[?]**

**Steering:**
- A similar Chinese gear-motor with a different ratio. **[H]**
- It turns the whole caster around the vertical axis through a timing belt onto a large timing pulley. **[H]** CAD has `drive-rotation-tgear176` (176 teeth?) **[D]**.
- Motor model and ratio **[?]**
- Mechanically it could turn without limit, but the cables do not allow it, so there is a **software limit**. **[H]**
  - The firmware works with angles in (−180°, +180°) around the forward-zero position. The angle never wraps.
  - `optimize_rotation()` either uses the target angle or the opposite angle with the drive reversed. It prefers the option with the smaller absolute angle and shorter travel.
  - In practice each caster stays within ±180°, with a bias towards ±90°. **[C]** (`lib/robot/base_move.h`)
  - The URDF joint limit is ±90°. **[C]**
- End-stop switches (left/right) are in the electrical design **[D]**. Whether they are installed **[?]**

**Sensors per caster:**

| Sensor | Part | Interface | Details |
|---|---|---|---|
| Steering angle (absolute) | AS5048A magnetic encoder (14-bit), magnet on a holder on the rotating part | SPI | CS pins FL=10, FR=11, R=12. MISO 50, MOSI 51, SCK 52. Zero offsets FL=275, FR=355, R=350 raw units (45.5 units/°) **[C]** |
| Wheel travel | IR slotted optical sensor + 3D-printed code wheel with 120 slots | digital pin, sampled by a ~1.25 kHz timer ISR | 240 edges/rev, ≈1.61 mm/edge. **Single channel, so there is no direction information.** The firmware assumes the commanded direction. **[C]** |

An older electrical note says "AS5048B – read precisely – try better mechanical placement" **[D]**, so the encoder mounting may have been an accuracy issue.

### 2.3 Measured odometry calibration (from `docs/software/odometry.ods`)

The robot drove known distances at 0.4 m/s in several directions, and the reported ticks were compared with the real distance. **[D]**

| Caster | Correction factor (real / reported) |
|---|---|
| FL | ≈ 1.02–1.04 |
| FR | ≈ 1.02–1.06 |
| R | ≈ 1.15–1.21 |

When the wheels were turned by hand, they gave 223–240 ticks per revolution instead of 240, so some edges are missed.

The rear caster under-reports by about 18%, which points to slip, a different wheel diameter, or missed edges. The bridge hard-codes `REAL_WORLD_COEF_A/B/C = 1.028 / 1.028 / 1.15`. **[C]**

## 2.4 Torso

- A movable body (torso lift) is mounted on the base. Nothing is mounted on the body yet. **[H]**
- Actuator: a NEMA 17 stepper with a lead screw. **[H]** Whether it uses a trapezoidal lead screw with a nut, or a nutless type such as a rolling-ring drive, is **[?]**.
- The motor is probably not wired yet. **[H]**
- The design specifies a TMC2130 driver and a zero-pose end-stop **[D]**. CAD files: `torso-stepper-holder`, `torso-stepper-bearing` **[D]**.
- Travel range, screw pitch and torso height **[?]**

## 3. Electrical

### 3.1 Power

The power system from the electrical schema (`docs/electrical/electrical-schema_v1.drawio`) is **built and installed**. **[H]** Rows below that were taken from the schema are marked **[H/D]**: confirmed as built, but the exact values still come from the schema.

| Item | Value | Source |
|---|---|---|
| Battery | Li-ion 4S (14.4 V nominal, 13–16.8 V), 15 Ah (~216 Wh), integrated BMS, Czech maker | **[H]** |
| Battery state | fully disconnected for storage, charged to ~70% | **[H]** |
| Charging input | barrel jack on the side, ~18 V laptop adapter, ~70 W | **[H]** |
| Charging circuit | installed | **[H]** Type **[?]** |
| Docking | 17 V magnetic docking connector + QR code for navigation | **[D]** Whether the docking station itself exists **[?]** |
| Step-down 12 V | for Jetson + Kinect (and drives? see below) | **[H/D]** |
| Step-down 5 V | for Arduino, sensors, audio | **[H/D]** |
| Motor supply | design says "11 V caster 45 W" ×3 and "DC Amper converter 300 W" | **[H/D]** Actual rail voltage **[?]** |
| Safety | drive relay, arm relay, panic switch, voltage display, capacitive "power holder" (keeps power ~2 s for a clean shutdown), Power ON/OFF GPIO signals | **[H/D]** |

The power budget from `docs/electrical/baterie.ods` is about 114 W total with the full arm, with drive at 30 W and Jetson at 15 W. The target was 30 minutes of runtime. **[D]**

**Requirement [H]:** for long sessions the robot should run from mains power (a wire), not from the battery.

### 3.2 Motor drivers

- 3× L298N dual H-bridge boards, one per caster: channel A for drive, channel B for steering. **[C]** (README pinout, CAD `el-l298n-pilar.stl`)
- Control: IN1/IN2 for direction and ENA (PWM) for speed, using the default Arduino PWM frequency (~490 Hz / 980 Hz). **[C]**
- Braking: IN1=IN2=LOW with ENA high. **[C]**
- Current sensing: in the design (`current R/W`) **[D]**, not used in code **[C]**.

### 3.3 Arduino Mega 2560 pin map **[C]**

| Pin | Function |
|---|---|
| 4 / 5 / 6 | PWM ENA drive FL / FR / R |
| 44 / 45 / 46 | PWM ENA steering FL / FR / R |
| 28, 30 | IN1/IN2 drive FL |
| 32, 34 | IN3/IN4 steering FL |
| 36, 38 | IN1/IN2 drive FR |
| 40, 42 | IN3/IN4 steering FR |
| 37, 39 | IN1/IN2 drive R |
| 41, 43 | IN3/IN4 steering R |
| 22 / 24 / 26 | IR wheel-tick input. Note: README says FL=22, FR=24, R=26, but `robot_config.h` uses FL=24, FR=26, R=22 **[?]** |
| 9 | PWM for the XV-11 lidar motor |
| 10 / 11 / 12 | AS5048A SPI chip select FL / FR / R |
| 50 / 51 / 52 | SPI MISO / MOSI / SCK |
| 20 / 21 | I2C SDA/SCL for the MPU-9150 |
| 14 (TX3) / 15 (RX3) | UART to the Jetson. The firmware actually uses `Serial`; which UART is wired **[?]** |
| 13 | debug LED |

Wire colours (README): yellow = PWM, green/blue = IN, stripe count 1/2/3 = FL/FR/R.

### 3.4 Jetson TK1 connections **[C]**

- UART1 `/dev/ttyTHS1` on J3A2 pins 65 (RX) and 68 (TX).
  - Goes through a 1.8 V ↔ 5 V level shifter, with LV and GND from J3A1 pins 3 and 2.
  - Runs at **19200 baud**. The code comment says "500000 is too much for voltage converter".
- XV-11 lidar on USB-serial `/dev/ttyUSB0`, 115200 baud.
- Kinect v1 on USB, through libfreenect.
- WiFi on 5 GHz. The README asks for the router to be set to 80 MHz channel width and power-save off. Ethernet is available. **[H]**
- Static IPs in the code: robot 192.168.1.21, workstation 192.168.1.2. **[C]**

### 3.5 Sensors summary

| Sensor | Connected to | Data path |
|---|---|---|
| 3× AS5048A | Arduino SPI | firmware → `caster_raw` |
| 3× IR wheel ticks | Arduino GPIO | firmware → `caster_raw` |
| MPU-9150 IMU | Arduino I2C | DMP quaternion + accel + gyro → `imu`. The bridge ignores orientation and the accelerometer was unreliable ("best IMU? giving up") **[C]** |
| XV-11 lidar | Jetson USB | 360 ranges at ~5 Hz → `lidar_ranges` → `/scan` |
| Kinect v1 | Jetson USB | RGB (zstd optional) + depth 640×480 mm, sent uncompressed line by line over ZMQ |

## 4. Software stack as of 2023 (for reference)

```
Phone (bobik_web, React + roslib) --wss--> rosbridge_server
Workstation, Ubuntu 20.04 + ROS 2 Foxy:
    bobik_bridge (C++): ZMQ <-> /cmd_vel, /odom, /imu, /scan; computes odometry
    Nav2 + SLAM Toolbox, robot_localization EKF, RViz, robot_state_publisher
    bobik_robot (Python): Dialogflow voice skills (cs-CZ), TTS
        ^ ZeroMQ RADIO/DISH over UDP, ports 7654 (cmd), 7655 (telemetry), 7656/7657 (Kinect)
Jetson TK1, L4T 21.3 / Ubuntu 14.04 + Grinch kernel, no ROS:
    bobik_driver (C++14, upstart service)
        ^ UART 19200 baud, custom binary frames (0xEE start byte, type byte, packed struct)
Arduino Mega 2560:
    bobik_arduino firmware, 20 Hz control loop:
    inverse kinematics of the 3 casters, steering P+I, drive tick control, IMU read
```

The firmware's internal frame has Y pointing right, which is the opposite of REP-103. The bridge flips it. **[C]**

## 5. Known weaknesses of the current HW

1. **Wheel odometry has no direction and misses edges.** A single-channel IR code wheel reports 223–240 of 240 edges per revolution, the rear caster is 18% off, and slip during steering is not modelled.
2. **L298N + cheap gear-motors.** The L298N has a ~2 V drop and no current feedback, and the motors have a large deadband. This forced minimum-PWM hacks, an integral "debt" term, and a "stop driving while steering error > 0.5 rad" rule.
3. **Slow, fragile Jetson ↔ Arduino link.** 19200 baud through a 1.8 V level shifter gives about 96 bytes per 50 ms frame.
4. **Jetson TK1 is stuck on Ubuntu 14.04 / L4T 21.x** (32-bit ARMv7, CUDA 6.5). Modern ROS 2 and most current libraries do not support it.
5. **IMU (MPU-9150) has been discontinued for years**, and its data was not usable in the EKF.
6. **WiFi latency.** A shared 5 GHz home AP with about 6 clients is not suitable for low-latency control loops over the network. **[H]**

## 6. Open questions for the HW review

- Exact motor models, voltages, gear ratios and gear types (drive and steering).
- Steering pulley tooth counts (motor pulley vs. the 176T pulley?).
- Which supply rail feeds the motors: battery direct, or a step-down?
- Torso lift: NEMA 17 driver (TMC2130 per schema?), lead screw pitch, travel range, end-stop. Is anything mounted for the head?
- Real total mass, and the height of the torso.
- Wheel-tick pin assignment (README vs. code mismatch).
- Is the MPU-9150 still mounted, and where is it (URDF says (−0.10, 0, 0.16))?
- Lidar mounting position (URDF: (0.12, 0.13, 0.2325)).
