# Electrical

Sources:
- `docs/electrical/electrical-schema_v1.drawio`, pages *Power Distribution*, *Power: The Thing*, *Electrical* and *AMS5048A*. Honza confirmed that the power wiring matches it.
- `docs/electrical/baterie.ods`
- `bobik_arduino/README.md` and `lib/robot/robot_config.h`
- `bobik_driver/README.md`

Notation: no tag means documented and confirmed. **(planned)** means designed but not built. **[?]** means unverified ([../open-questions.md](../open-questions.md)).

## 1. Power distribution (as built)

```
  ~18 V laptop adapter (≈70 W) ── barrel jack ──┐       (planned: 17 V magnetic docking connector)
                                                ▼
                                   "The Thing" – charge & power-hold module
                                   (capacitive power holder ≈2 s, Power ON signal,
                                    voltage display, T-Dean battery connector)
                                                │
  Li-ion 4S battery 14.4 V (13–16.8 V), 15 Ah, BMS ──► Emergency switch ──► 13–16 V main bus
                                                                          │
              ┌───────────────────────────────┬───────────────────────────┼───────────────────────────┐
              ▼                               ▼                           ▼                           ▼
      Step-down 12 V                   Step-down 5 V              Relay (switched) ──► 13–16 V motor bus    Voltage display
      Jetson TK1, Kinect               Arduino + sensors              │                                (push button)
                                       ("5 V breadboard"), audio     ├─► 3× step-down 11 V / 45 W, one per caster → L298N
                                                                     └─► steppers (torso; later head/arm)
```

| Item | Details |
|---|---|
| Battery | Li-ion 4S, 14.4 V nominal (13–16.8 V), 15 Ah (~216 Wh), integrated BMS, made by a Czech company. Stored disconnected at ~70% charge. The schema says "4S?P, 14Ah" |
| Charging | ~18 V laptop adapter (~70 W) into the barrel jack on the side, then the on-board charge circuit ("The Thing"). The schema also has a "DC Amper converter 300 W" in the charging path [?] |
| Emergency switch | between the battery and the main bus, cuts everything |
| Motor relay | switches the motor bus. It is software-controllable: `bobik_robot` has a `motor_relay` service stub |
| Caster supply | each caster has its own step-down to 11 V, 45 W, feeding its L298N |
| Logic supply | step-down to 12 V for the Jetson and Kinect, and step-down to 5 V for the Arduino, sensors and audio |
| Power-hold | capacitive holder keeps power for about 2 s. The Power ON / Power OFF signals go to the Jetson: front-panel connector pin 4 is wired to the power system, for a clean shutdown (not used in software yet) |
| Voltage display | battery voltage, shown on a push button |

**Power budget** from `baterie.ods`, with the full planned robot:
- about 114 W total
- drive 30 W, steering 5 W, Jetson 15 W, Kinect 12 W
- the target was 30 min of runtime

Today's load, with no arm, is well below that.

**Operating rule:** for long sessions, run on mains power with a cable attached.

## 2. Motor drivers

- 3× L298N dual H-bridge, one per caster: channel A drives the wheel and channel B steers. Mounted on 3D-printed pillars (`el-l298n-pilar.stl`).
- Control: IN1/IN2 set the direction and ENA takes PWM at the default Arduino frequency (490 / 980 Hz).
  - Brake: IN1 = IN2 = LOW, ENA high.
  - Coast: ENA low.
- The schema shows current-sense outputs (`current R/W`), but the firmware does not use them.

## 3. Arduino Mega 2560 (base controller)

From the README and `robot_config.h`. The README also documents wire colours: yellow = PWM, green/blue = IN, and 1/2/3 stripes = FL/FR/R.

| Pin | Function |
|---|---|
| 4 / 5 / 6 | ENA (PWM) drive FL / FR / R |
| 44 / 45 / 46 | ENA (PWM) steering FL / FR / R |
| 28, 30 / 36, 38 / 37, 39 | IN1, IN2 drive FL / FR / R |
| 32, 34 / 40, 42 / 41, 43 | IN3, IN4 steering FL / FR / R |
| 22 / 24 / 26 | IR wheel-tick inputs. The README says FL/FR/R = 22/24/26, but `robot_config.h` maps FL = 24, FR = 26, R = 22 [?] |
| 10 / 11 / 12 | AS5048A chip select FL / FR / R |
| 50 / 51 / 52 | SPI MISO / MOSI / SCK, shared by all AS5048A |
| 20 / 21 | I2C SDA / SCL for the MPU-9150 IMU |
| 9 | PWM for the XV-11 lidar motor |
| 14 / 15 | TX3 / RX3 to the Jetson UART (through the level shifter) |
| 13 | debug LED |

AS5048A cable colours: CS green, CLK blue, MOSI yellow, MISO white, VCC red, GND black. 5 V supply.

## 4. Jetson TK1 (on-board computer)

| Port | Connected to |
|---|---|
| UART1 `/dev/ttyTHS1` (J3A2 pin 65 RX, pin 68 TX) | Arduino, through a 1.8 V ↔ 5 V level shifter. LV and GND come from J3A1 pins 3 and 2. Runs at 19200 baud, because faster was unreliable through the shifter |
| micro-USB (USB0, OTG as host) | USB-serial converter for the XV-11 lidar (`/dev/ttyUSB0`, 115200 baud) |
| USB-A (the only one) | Kinect v1, through an Xbox 360 Kinect adapter (currently unplugged) |
| Audio jack | speaker, with no amplifier yet. An amplifier is in the design (planned) |
| Front-panel header pin 4 | power system (Power OFF signal for clean shutdown) |
| Ethernet | available, intended for tethered operation |
| WiFi | 5 GHz. The design mentions an Intel 7260 mini-PCIe card, and a TP-Link antenna is mounted on the robot |
| 12 V input | from the 12 V step-down |

Old OS: L4T 21.3 (Ubuntu 14.04) with the Grinch kernel. The full setup notes are in `bobik_driver/README.md`.

## 5. Sensors

| Sensor | Bus | Notes |
|---|---|---|
| 3× AS5048A steering angle | Arduino SPI | see [mechanical.md](mechanical.md) |
| 3× IR wheel ticks | Arduino GPIO, sampled by a ~1.25 kHz timer ISR | single channel, no direction |
| MPU-9150 (GY-9150) IMU | Arduino I2C | DMP quaternion + accelerometer + gyro. The orientation output was unusable in the EKF |
| Neato XV-11 lidar | Jetson USB-serial; motor PWM from Arduino pin 9 | 360 points, ~5 Hz, working |
| Kinect v1 | Jetson USB | RGB + depth 640×480 through libfreenect. The hardware works |

## 6. Designed but not built (from the *Electrical* page)

- Arm: 5 joints with steppers (TMC2130), DC joints, servos on a PWM board and a separate Arduino Nano.
- Head: pan and tilt steppers (A4988) with IR zero sensors.
- Torso stepper driver and end-stop.
- Touch display (HDMI), I2S mic array, audio amplifier, front and back USB cameras, PIR sensor, LEDs, laser range finder, Bluetooth.

See [planned-extensions.md](planned-extensions.md).
