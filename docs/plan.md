# Bobik – Mid-level Plan

> **Status: DRAFT, agreed direction 2026-10-25.**
> Inputs: [hardware.md](hardware.md) and [project-goals.md](project-goals.md).
> Prices are rough estimates (CZK, late 2026) and must be checked before buying.

## 1. Target architecture

```
                        Workstation (latest Ubuntu + latest ROS 2 LTS)
  ┌──────────────────────────────────────────────────────────────────────────┐
  │ bobik_base (C++ ROS 2 node)  kinematics, odometry, /cmd_vel, /odom, TF,  │
  │                              /joint_states, /imu, /scan, camera topics   │
  │ robot_localization EKF · slam_toolbox · navigation · places · voice/LLM  │
  │ rosbridge + bobik_web (phone UI)                                         │
  └───────────────▲──────────────────────────────────────────────────────────┘
                  │ Ethernet when tethered (default) / WiFi when on battery
                  │ ZeroMQ, versioned message schema
  ┌───────────────┴──────────────────────────────────────────────────────────┐
  │ Jetson TK1: Debian armhf + mainline kernel, no ROS                       │
  │ bobik_driver v2 (C++17): Arduino link, XV-11, Kinect (libfreenect),      │
  │ image compression, link watchdog, health/telemetry, systemd service      │
  └───────────────▲──────────────────────────────────────────────────────────┘
                  │ USB serial (Mega's native USB, ≥ 115200 baud), COBS + CRC frames
  ┌───────────────┴──────────────────────────────────────────────────────────┐
  │ Arduino Mega 2560 (C): per-caster loops only                             │
  │ steering angle setpoint → position loop; wheel speed setpoint → speed    │
  │ loop; encoder/IMU readout; command timeout → motors off                  │
  └──────────────────────────────────────────────────────────────────────────┘
```

### Key design decisions (proposed)

1. **Move base kinematics off the Arduino.**
   - Today the Arduino turns a `Twist` into caster angles and speeds.
   - In the new design it only gets per-caster setpoints (angle, wheel speed) and reports raw joint states.
   - Kinematics, the steering limit / flip logic and odometry live in `bobik_base` on the workstation. There they can be unit-tested, simulated and changed without reflashing.
2. **Thin, ROS-free Jetson.**
   - ROS 2 has no binaries for 32-bit ARM, and building it from source on 2 GB RAM is fragile.
   - The TK1 does only I/O and the CPU-heavy streaming jobs (Kinect frame handling and compression).
3. **Mainline Linux on the TK1.**
   - Use a current Debian armhf with a mainline kernel instead of L4T 21.x / Ubuntu 14.04.
   - Mainline kernels are boot-tested on the TK1 by KernelCI, and Debian supports the board.
   - We lose CUDA 6.5, which is not useful for us anyway.
   - Install to an SD card first, and keep the original eMMC system as a fallback.
4. **Arduino over its native USB** instead of the 1.8 V UART + level shifter.
   - Removes the 19200 baud bottleneck and a fragile part.
   - Needs a powered USB hub on the TK1. The schema already lists one.
5. **Proper framing and one shared protocol definition.**
   - COBS-framed packets with CRC and a protocol version byte.
   - A single header shared by firmware, driver and bridge, instead of hand-copied constants.
6. **Safety in depth.**
   - Arduino stops the motors after about 200 ms without a command.
   - The driver stops them when the workstation link drops.
   - The panic switch stays as the hardware override.
7. **Ethernet by default.**
   - Long sessions are on mains power, so the robot is tethered anyway, and this sidesteps the WiFi latency problem.
   - WiFi is used only for untethered demos.

## 2. Phases

Each phase ends with something visible.

### Phase 0 – Bring-up and baseline (no purchases)

- Check the battery and BMS, power up, and boot the existing TK1 system.
- **Back up the TK1 eMMC** image to the workstation before changing anything.
- Flash the current firmware and run the old stack once.
- Record a baseline: does it drive, how good or bad is the odometry (square test), and does each sensor work (casters, AS5048A, IR ticks, IMU, XV-11, Kinect).
- Fix any discrepancies in `hardware.md`. Separate thread: take the drive mechanics apart and measure them, with photos.

**Done when:** the baseline test results are written into `docs/` and the HW doc has no unknowns that block Phase 1.

### Phase 1 – Platform refresh

- **Workstation:** latest Ubuntu LTS + latest ROS 2 LTS. Workspace layout, build script, and a simple simulation of the swerve base (Gazebo) for testing without hardware.
- **TK1:** Debian armhf + mainline kernel on an SD card. Verify:
  - Ethernet
  - WiFi (Intel 7260 / iwlwifi)
  - USB 3 + hub
  - Kinect via libfreenect
  - XV-11 via USB-serial
  - Arduino via USB
  - free CPU and RAM headroom
- **Toolchain:** cross-compile or build natively, an install script, and a systemd unit. Remote flashing of the Arduino from the TK1 (PlatformIO or avrdude).

**Done when:** the TK1 boots the new OS, all devices enumerate, and the Arduino can be flashed remotely.

### Phase 2 – New driver stack (software only)

- `bobik_arduino` v2 (C):
  - per-caster steering position loop and wheel speed loop
  - new protocol
  - command timeout
  - firmware version reporting
- `bobik_driver` v2 (C++17):
  - protocol endpoint
  - XV-11 reader
  - Kinect: RGB as JPEG, depth compressed losslessly
  - watchdog
  - health topic
- `bobik_base` (C++ ROS 2 node, replacing `bobik_bridge`):
  - swerve kinematics with the software steering limit
  - odometry
  - standard topics and TF
  - parameters in YAML instead of `#define`s
- Updated URDF consistent with `hardware.md` (steering limits, caster positions).

**Done when:** teleop from the keyboard or phone drives the robot, and RViz shows the robot model, the lidar scan and the Kinect point cloud.

### Phase 3 – Drive hardware upgrades (budget ≤ 10,000 CZK)

These are the biggest sources of the old tuning pain. The final choice depends on the mechanics thread.

| Item | Why | Rough cost |
|---|---|---|
| Replace 3× L298N with modern MOSFET drivers (e.g. Cytron MDD10A dual 10 A, or BTS7960 modules) | Removes the ~2 V drop and heat. Gives linear low-speed control and better braking | 1,000–1,800 |
| Wheel encoders with direction | Gets rid of the guessed direction and missed edges. Options: a second IR sensor at a quarter-slot offset on the existing code wheel (cheapest, 3D-printed holder), a magnetic encoder on the wheel axle (AS5600, MT6701), or gear-motors with built-in Hall encoders | 150–2,000 |
| Fused-orientation IMU (BNO085 / BNO055) | Gives a reliable yaw for the EKF. The MPU-9150 DMP output was unusable | 400–900 |
| Powered USB 3 hub | Arduino, lidar and Kinect on the TK1's single USB port | 300–600 |
| *Optional:* modern 2D lidar (LDROBOT LD19 / RPLIDAR C1) | 10 Hz, longer range and more points than the XV-11. Makes SLAM and localization much more robust | 1,500–2,500 |

Expected total: about 3,500–7,500 CZK including the optional lidar.

**Done when:** square-test odometry error is under 5% of distance, the robot drives smoothly at low speed, and the steering settles without oscillation.

### Phase 4 – M1: Reliable base motion

- Calibrate the wheel diameters and steering zeros using a repeatable procedure (a script, not hand-edited constants).
- EKF fusing wheel odometry and IMU yaw.
- Phone UI updated for teleop and a status view (battery, link, motor state).

**Done when:** Bobik can be driven around the flat predictably for 10 minutes, and odometry plus IMU stay consistent with reality.

### Phase 5 – M2: Go to a named place

- Map the home once with lidar SLAM (slam_toolbox). Localize on the saved map, so drift is corrected by the lidar instead of relying on dead-reckoning.
- **Places:** named poses stored in a YAML file and set from the phone UI ("this is the kitchen").
- Navigation: start with the simplest thing that works, a global path on the map plus a simple local controller for a holonomic base. Use full Nav2 only if needed.
- *Optional VLM layer:* the Kinect RGB image is sent to a vision-language model to label places, confirm arrival ("am I in the kitchen?") and describe obstacles.

**Done when:** "go to kitchen" from the phone succeeds 9 times out of 10 from anywhere in the mapped area.

### Phase 6 – M3: Voice commands and text-to-action

- Speech-to-text in Czech:
  - Whisper-class model locally on the workstation
  - or the browser STT already used in `bobik_web`
- An LLM with tool calling turns text into robot actions (`go_to(place)`, `say(text)`, `turn(angle)`, `status()`). These are exposed as ROS 2 actions and services in `bobik_interfaces`.
- Czech TTS, for example Piper, for replies.
- Later: evaluate end-to-end text2action / VLA models once the action layer exists.

**Done when:** "Bobiku, dojeď do kuchyně" works end-to-end.

### Later

- Torso lift: wire the NEMA 17 (TMC2130 or similar) to the Arduino, add a homing end-stop, and expose it as a ROS joint.
- Head pan/tilt with the Kinect.
- Arm and gripper (separate project).
- Docking station with automatic charging.

## 3. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Mainline kernel on the TK1 misses something we need (UART, USB quirks) | SD card install, eMMC fallback. The USB-only design avoids board-specific UART pins |
| TK1 CPU too weak for Kinect streaming plus compression | Lower the frame rate or resolution, or send depth only. Measure in Phase 1 |
| Drive mechanics need deeper rework than new drivers and encoders | Decide in the mechanics thread before buying Phase 3 parts |
| Budget overrun | Phase 3 parts are bought in priority order: drivers, encoders, IMU, hub, then lidar |
| Battery health after 3 years of storage | Check the cell voltages and the BMS in Phase 0, and run on mains power by default |

## 4. Open decisions

- Transport between TK1 and workstation: keep ZeroMQ (simple, proven here) or move to Zenoh, which ROS 2 supports natively through `rmw_zenoh`.
- Exact motor driver and encoder parts, after the mechanics thread.
- Keep the XV-11 or buy a new lidar.
- Navigation stack for M2: a simple custom planner or Nav2.
