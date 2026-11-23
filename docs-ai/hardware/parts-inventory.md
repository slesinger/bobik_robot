# Parts Inventory and Parts in Use

Honza has accumulated a lot of hardware over the years and **prefers to reuse it before buying new parts**. Names below are the shop listing titles he gave; which exact variant is mounted is noted where known. (Recorded 2026-11-23.)

## In use on the robot

| Part | Use |
|---|---|
| LM393 IR module (infrared slotted sensor) | counts ticks of the code wheel on each drive wheel |
| AS5048A magnetic encoder, 14-bit, SPI | caster steering angle. Honza also bought the AS5048 PWM/SPI "no brush holder" set and is not sure which version is fitted |
| NEMA 17, 40 N·cm, 3.96 V stepper | torso lift screw |
| GT2 idler, 20T, 5 mm bore, 6 mm wide, smooth | tensions the caster steering belt |
| GT2 pulley 16T, 5 mm bore (from a belt + 2 pulleys set), or a 15T pulley | steering motor pulley, tooth count unconfirmed (see [mechanical.md](mechanical.md#steering)) |
| Flexible shaft coupler D19 × L25 mm, aluminium, bores 3–10 mm | torso stepper to lead screw |
| Emergency stop ZB2-BE102C (XB2-ES542), 22 mm mushroom, NC, 10 A | robot emergency switch |
| 12 × 18 × 12 mm drawn-cup needle bearings (2 per wheel), or 625ZZ ball bearings (5 × 16 × 5 mm) | inline-skate wheel bearings. Which one is fitted is unconfirmed |
| uxcell NK12/12 needle bearing, 12 × 19 × 12 mm | possibly used somewhere, unconfirmed |
| L298N dual H-bridge modules | the three caster drivers (Honza has many spare) |

## In the drawer (available for reuse)

- Several more NEMA 17 steppers of the same type.
- 5× A4988 stepper drivers and DRV8825 stepper driver modules, enough for the torso, and later head pan/tilt.
- Many L298N modules.
- Load cell kits (1 / 5 / 10 / 20 kg) with HX711 amplifier, planned for sensing arm load.
