# Legacy Software (state as of 2023)

This is a reference description of the software that existed before the rewrite. It is useful for porting the knowledge that is in the code. The new design is in [plan.md](plan.md).

## 1. Deployment

```
Phone (bobik_web: React 17 + roslib) ──wss──► rosbridge_server
                                                  │
Workstation "ha.doma" (Ubuntu 20.04, ROS 2 Foxy)  ▼
  bobik_bridge (C++)      ZMQ ⇄ /cmd_vel, /odom, /imu, /scan; odometry computed here
  bobik_robot (Python)    Dialogflow (cs-CZ) voice skills, TTS, services human_query, bobik_say,
                          bobik_poweroff, motor_relay (stubs that only speak)
  Nav2 (DWB, NavFn), slam_toolbox, robot_localization EKF (odom only, 2D), RViz, robot_state_publisher
        ▲  ZeroMQ RADIO/DISH (draft API) over UDP; hard-coded IPs
        │    robot 192.168.1.21:7654 ← cmd_vel
        │    workstation 192.168.1.2:7655 ← caster_raw, imu, lidar_ranges; :7656 Kinect depth (UDP); :7657 Kinect RGB (TCP)
Jetson TK1 (L4T 21.3 / Ubuntu 14.04, Grinch kernel; gcc-9, cmake 3.12, Python 3.6 built from source)
  bobik_driver (C++14, upstart service /etc/init/bobik.conf)
    threads: Arduino reader, XV-11 reader (boost::asio), Kinect RGB sender; main loop = ZMQ cmd_vel receiver
        ▲  Serial3 on the Mega ⇄ /dev/ttyTHS1, 19200 baud
Arduino Mega 2560 – bobik_arduino (PlatformIO, atmelavr)
    20 Hz frame loop: read sensors → emit telemetry → caster control → receive commands until the frame ends
```

## 2. Arduino ⇄ Jetson protocol (`bobik_arduino/lib/protocol`)

`protocol_types.h` is owned by `bobik_arduino`. `bobik_driver` symlinks it, and `bobik_bridge` keeps a hand-made copy of some of the constants.

**Arduino → Jetson.** One burst per 20 Hz frame, in this order:

1. `EA EA EA` (loop start)
2. `EE AA <u32 millis>` (timestamp)
3. `EE <type> <payload>` repeated, one per message
4. `EE FA <u32 crc8>` (CRC of the frame)
5. `AE AE AE` (loop end)

| Type | Name | Payload |
|---|---|---|
| 0x01 | LOG4 | 4 bytes of debug data |
| 0x02 | CASTER_JOINT_STATES | 6× int16: rotation (AS5048 units, ±8192 = ±π) and drive ticks per frame, for FL/FR/R |
| 0x04 | IMU_9DOF | 10× int16: accel, gyro, quaternion (×5000) |
| 0x05 | LOADCELL | int32 (HX711, disabled) |

**Jetson → Arduino:** `FA|FB|FC` length marker (2/6/12 bytes), then the type and the payload.
- Only `DRIVE_CMD 0x01` is used. Its payload is `int16 linear_x, linear_y, rotation`, each ×5000 (`FLOAT_INT16_PRECISION`).
- `linear_y` is inverted to the firmware's Y-right frame.

**Known defects:**
- The CRC over `CASTER_JOINT_STATES` runs over 20 bytes, although the struct is only 12.
- The driver never checks the CRC.
- There is no resynchronisation after a lost byte.
- Telemetry is sent as raw packed structs, which depends on both sides having the same endianness and alignment.

## 3. Base control in firmware (`lib/robot`)

- **Inverse kinematics** in `Bobik::setCmdVel`:
  - Convert the twist from per-second to per-frame values (/20).
  - Rotate each caster's position by γ and add (x, y). The resulting displacement vector gives the steering angle (`point2rad`) and the distance to drive per frame.
  - Speeds are scaled down together if any caster would exceed 0.4 m/s.
- **Steering choice** (`optimize_rotation`): use the target angle as-is, or the opposite angle with the drive reversed. The cost is |target| + 0.8 × travel, with a 0.06 rad hysteresis.
- **Stop while steering:** if any caster's angle error is above 0.5 rad, all drives get 0 until the steering catches up.
- **Steering loop:**
  - P term plus an integral term that grows only while the caster is not moving.
  - PWM is mapped from the error with a minimum of 60.
  - Deadband of about 20 units.
  - The angle is averaged over 10 SPI reads.
- **Drive loop:**
  - Target ticks per frame are mapped to PWM, with full PWM at 14 ticks/frame.
  - Ramp-up limited to 25% per frame.
  - Small integral "debt" term, max 10.
  - Brake when the target is 0.
  - The measured ticks get their sign from the last commanded direction.
- **Timing:** Timer1 CTC ISR at ~1.25 kHz polls the three IR inputs for edges. Pin-change interrupts were tried and gave worse results.
- **Safety:** the drive target is cleared after each frame (`clean_afer_execute`), so a missing command stops the wheels. Steering holds its last target.

## 4. Odometry (`bobik_bridge::calculate_odom`)

- **Per frame:**
  - Move each caster's position by its measured distance along its measured angle. Distance = ticks × 1.61 mm × `REAL_WORLD_COEF` (1.028 / 1.028 / 1.15).
  - New robot centre = 1/3 of the way from the front-pair midpoint to the rear caster.
  - Heading change = the angle of the rear-to-front vector.
- **Integration:**
  - Accumulated without rotating the increment into the world frame. This is a bug: the pose is wrong once the robot has turned.
  - The twist is reported as the per-frame displacement, not as a velocity.
- **IMU:**
  - Accelerometer bias is calibrated over the first 100 frames, and a 0.02 deadband is applied.
  - Orientation is published as identity, so it is effectively unused.

## 5. Other components

- **bobik_web** (React, phone-sized UI for a 1080×2400 display):
  - tabs for control, map, camera, chat/STT and actions
  - Auth0 login
  - served over HTTPS by Apache, see `010-bobik.conf`
- **bobik_interfaces:** copies of std_msgs, `HumanQuery.srv`, and example Fibonacci / AddTwoInts definitions.
- **bobik_arduino_test:** bench tests for the AS5048A and the casters, with serial output.
- **Kinect** (`bobik_driver/src/bobik_kinect.cpp`, libfreenect):
  - depth mode MEDIUM in mm
  - RGB sent in multi-line chunks, with zstd available but disabled
  - depth sent line by line, uncompressed

## 6. Lessons carried into the rewrite

1. Hand-copied constants across three repos led to drift. Use one source of truth.
2. Kinematics on an 8-bit MCU at 20 Hz is hard to tune and test. Move it up the stack.
3. The binary protocol needs framing that recovers from errors, a version byte, and a CRC that is actually checked.
4. Odometry must integrate in the world frame and use measured wheel direction.
5. The robot should stop by itself when commands stop.
