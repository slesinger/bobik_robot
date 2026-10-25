# Planned Extensions (designed, not built)

This is a summary of the design notes for parts of Bobik that do not exist yet. Sources:
- `docs/electrical/electrical-schema_v1.drawio`, pages *Electrical* and *Arm*
- `docs/mechanical/arm.FCStd`
- `docs/mechanical/mechanical-schema_v1.drawio`
- `docs/CAD/head_pan_link/`

## Arm (PR2-style)

- **Joints:**
  - shoulder pan (with counterbalance)
  - shoulder lift (spring-assisted)
  - upper arm roll (268°)
  - elbow flex
  - forearm roll (continuous, differential)
  - wrist flex and wrist roll (servos)
  - gripper (servo, underdriven fingers)
- **Actuators:** 4× stepper (150 g, 0.40 Nm) on timing belts and 3× servo (30 g, 0.8 Nm).
  - Shoulder lift needs about 7 Nm plus 3 Nm payload. Planned: a 25× belt reduction onto an Ø257 mm, 808T pulley.
  - Upper arm roll: Ø180 mm, 565T. Forearm roll: Ø32 mm, 101T.
  - Motor pulley 16T to 50T, about 101 mm.
- **Mass budget:**
  - estimate of 3,050 g + 300 g payload
  - reach: payload at 95 cm from the shoulder axis
- **Control:** a separate Arduino Nano, stepper drivers (TMC2130), and a PWM board for the servos. End-effector pressure sensors, a hand USB camera, and an HX711 load cell for the lift joint (code exists, disabled).

## Head

- Pan and tilt, each with a stepper (A4988) and an IR zero-pose sensor.
- Carries the RGB-D camera (`rgbd_link`). A temporary Kinect mount and head pan link drafts are in CAD.

## Torso lift completion

- Stepper driver (A4988 / TMC2130), zero-pose end-stop, and the safety end-stop part (`backup/torso-security-endstop.stl`). A laser range finder was considered for absolute height.

## Human interface and other sensors

- Touch display (HDMI, from Android Things)
- I2S mic array (DFRobot)
- 3 W audio amplifier + speaker
- front and back USB cameras
- PIR sensor
- LEDs
- Bluetooth over USB
- control panel with a voltage display (built)

## Docking

- Docking station with a 17 V magnetic connector, and a QR code for final alignment.
