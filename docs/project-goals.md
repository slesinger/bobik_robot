# Bobik – Project Goals

> **Status: DRAFT for review.** Based on Honza's input from 2026-10-25.

## Why the project is being restarted

Bobik was enjoyable to build, but development stalled. Cheap, imprecise hardware had to be compensated by tuning and software, which took a lot of time without bringing reasonably fast results. The restart uses AI-assisted development to get past that. Quick, satisfying results are a priority.

## Principles

- **Documentation first.** The HW and SW documentation must be accurate and current, so that proposals and new code make sense without frequent correction.
- **Prefer simple, robust solutions over heavy tuning.** Where cheap hardware is the root cause, a modest HW upgrade is preferred over weeks of software compensation.
- **Budget:** reasonable, rather lower. A bigger spend is acceptable when the added value is large, or when not having a component would block development.
- **Partly frozen HW.** The aluminium frame stays. The drive mechanics can be redesigned if there is a better idea. A 3D printer is now available, which most of the robot was built without.
- **Full SW refresh** is wanted: a current Ubuntu and ROS 2 (or an alternative), and a working Kinect even if it needs a custom driver.
- **Languages:**
  - Jetson low-level code in C++
  - high-level code may be Python
  - Arduino in C
  - Rust only after agreement
  - code and docs in English

## Milestones (proposed order)

1. **M0 – Documented baseline.** HW and SW documentation reviewed and corrected by Honza. The robot boots, and each subsystem (casters, lidar, Kinect, IMU, power) is checked one by one.
2. **M1 – Reliable base motion.** Teleoperate Bobik with predictable, repeatable motion and usable odometry.
3. **M2 – Go to a place.** Bobik drives to a named place in the home. Earlier, Bobik could already map part of the home and navigate to a point on the map, with problems. The goal is a simpler and more robust approach than full Nav2 with dead-reckoning, possibly using modern vision or VLM-based methods.
4. **M3 – Voice commands.** Spoken commands, for example in Czech, mapped to robot actions. Text-to-action / VLA-style models are of interest.
5. **Later:** torso and body usage, arm and gripper.

## Operational requirements

- Long sessions run from mains power over a cable, not from the battery.
- The robot has WiFi and Ethernet. The home 5 GHz AP is shared with about 6 clients and is not good for low latency, so a fast and stable link must be solved. Options include a dedicated AP or channel, Ethernet when tethered, or moving latency-critical loops onto the robot.
- The robot can be switched on by arrangement for remote work, such as running builds or flashing the Arduino.

## Open decisions

- On-board compute: keep the Jetson TK1 (stuck on Ubuntu 14.04), or replace it with something that runs current Ubuntu and ROS 2 natively?
- Drive mechanics: improvements to the caster drive and steering (motor drivers, encoders, backlash).
- Depth sensing: keep the Kinect v1 or move to a newer sensor.
- How navigation for M2 will work.
