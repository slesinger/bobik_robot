# Open Questions

This is the single list of what is still unknown. The questions are grouped by topic and asked one topic at a time, when that topic is being worked on. When something is answered, it is removed here and written into the relevant document.

## Caster mechanics (parked until the robot is switched on)

- Steering speed: the motor pulley was counted as 15T (maybe 16T), but the remembered 3 s per caster revolution fits 30T or a 4× gearbox better. Recount, or measure the turn rate. See [hardware/mechanical.md](hardware/mechanical.md#steering).
- Drive/steering motor exact model and gearbox ratio: no label. Only estimates exist (drive ≈ 60 rpm, steering ≈ 120 rpm). Verify on the bench later.

## Bench (when the robot is first switched on)

- Battery cell voltages and BMS state (charged to about 70 %, disconnected).
- What the "DC Amper converter 300 W" in the charging path is. The schema text could not be found by search; check the drawio.
