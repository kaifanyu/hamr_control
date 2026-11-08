# Caster orientation calibration — 2026-11-08

[caster_orientation_calibration.json](caster_orientation_calibration.json) preserves
four measured zero/sign references: the left/right ordinary SWIVEL encoders and
the left/right ball CENTER encoders. Measurements were collected on 2026-11-08
(Asia/Shanghai); individual capture timestamps are recorded in UTC.

**Saved data only.** This commit does not activate calibration in firmware, ROS,
or replay. No reflashing is required to retain these values or apply them later
in software to native raw counts.

| Sensor | Native zero count | Counts per turn | Sign |
|---|---:|---:|---:|
| Ordinary left SWIVEL | 3943 | 4096 | -1 |
| Ordinary right SWIVEL | 873 | 4096 | +1 |
| Ball left CENTER | 1540 | 16384 | +1 |
| Ball right CENTER | 15320 | 16384 | -1 |

## Physical references

- Ordinary zero: wheel axle behind the vertical swivel shaft, toward robot rear,
  with the mount upright and fixed. Positive rotation is counterclockwise viewed
  from above; at +90 degrees the axle is toward robot right.
- Ball CENTER zero: H1 front/H2 back, split axis horizontal, fork upright and
  main axle fixed. Positive forward-rolling rotation puts H1 down/H2 up at +90
  degrees. This is carrier phase, not swivel yaw or independent shell spin.

The JSON identifies each board by MAC and includes its native count scale,
zero count/radians, sign, angle formula, reference convention and validation
residuals. Apply calibration once to the matching device's raw `position12`
(ordinary) or `position14` (ball). Preserve the native count scale and the same
encoder/mounting/electronic zero. Older H1-up CAD references have a different
convention from the H1-front reference recorded here.

## Software use and limits

This consolidated JSON is an archive format; existing tools do not load it
automatically. `caster_center_angle --calibration FILE` accepts the original
per-wheel ball CENTER JSON in the evidence archive. Ordinary bridge/replay
currently consumes firmware-calibrated angles and needs an explicit software
calibration step to use this saved reference without reflashing.

All four final pose checks passed. These are manually positioned zero/sign
measurements, not independent angular-accuracy or maximum-rate qualification.
Wheel/H1/H2 CPR, rolling radius, shell spin references and clock synchronization
were not calibrated. The JSON records the remaining unvalidated flags.

## Original evidence

The raw measurements, original per-sensor JSON and checksums are retained in
`caster_orientation_calibration_20261108.zip`, outside Git. On the capture Pi:

```text
/home/hamr/hamster_ws/calibration_results/caster_orientation_20261108/caster_orientation_calibration_20261108.zip
```

The consolidated JSON records the ZIP's SHA-256 and original record hash.
Per-sensor `evidence.calibration_json` and `evidence.raw_history` paths are relative
to the ZIP root, not this repository directory. The original absolute paths are
capture provenance. Obtain the ZIP separately if the full observations or
per-wheel tool inputs are needed; all measured constants are already in this
repository JSON. Failed or incomplete captures are excluded from the four final
references, with their original records retained on the Pi.
