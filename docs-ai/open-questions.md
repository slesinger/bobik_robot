# Open Questions

This is the single list of what is still unknown. The questions are grouped by topic and asked one topic at a time, when that topic is being worked on. When something is answered, it is removed here and written into the relevant document.

## Caster mechanics (when a caster is taken apart)

- Drive and steering motor models (label), rated voltage, and gearbox ratio.
- Bevel gear pair ratio.
- Steering motor pulley tooth count. The caster pulley is a 176T 2GT.
- Are steering end-stop switches installed?

## Torso

- Lead screw pitch, travel range, and whether an end-stop is installed.

## Sensor placement and mass (tape measure)

- Lidar and IMU positions relative to the robot centre. These are needed for the URDF.
- Real total mass.

## To verify on the bench (no question for Honza)

- IR wheel-tick pin mapping: the README (FL/FR/R = 22/24/26) and `robot_config.h` (24/26/22) disagree.
- What the "DC Amper converter 300 W" is in the charging path.
- Battery cell voltages and BMS state after storage.
