# Config Layout

This folder stores project configuration that is likely to change between:

- hardware setups
- tuning sessions
- vehicles
- filter experiments
- dynamics-model experiments
- animation / plotting preferences

The files are split by concern so we do not end up with one large YAML file
that mixes physical parameters, serial ports, EKF noise values, workbook-
derived CG data, and UI options.

## Suggested Ownership

- `frames.yaml`
  Coordinate frames, axis conventions, mounting assumptions, and steering-sign conventions.
- `vehicle.yaml`
  Vehicle physical properties, CG estimates, axle load split, and first-pass dynamic parameters.
- `sensors.yaml`
  Sensor ports, update rates, wiring assumptions, and sensor noise.
- `controller.yaml`
  Main-loop timing, logging, and runtime defaults.
- `dynamics.yaml`
  Dynamics model structure, desired derived outputs, handling metrics, and validation checks.
- `ekf.yaml`
  EKF state-model choices, process noise, measurement noise, and integration plan.
- `animation.yaml`
  Telemetry animation defaults and display conventions.

## What Belongs In YAML

Good config candidates:

- values that differ by car, sensor package, or laptop vs Pi
- tuning parameters
- noise assumptions
- serial ports and baud rates
- visualization defaults
- sign conventions that may still be under investigation
- measured CG / load-transfer values extracted from experiments or spreadsheets

Poor config candidates:

- formulas that are more maintainable in code
- class behavior that should stay in code
- one-off temporary debug hacks
- values that are derived entirely from other config fields and never adjusted

## Provenance

When a value comes from a spreadsheet, experiment, or external reference, prefer
storing:

- the numeric value used by the code
- the original unit/value when helpful
- a source tag or note
- a confidence note if the value is still provisional

## Current External References

- MathWorks vehicle-dynamics reference:
  https://www.mathworks.com/help/ident/ug/modeling-a-vehicle-dynamics-system.html
- Janudis EKF reference:
  https://github.com/Janudis/Extended-Kalman-Filter-GPS_IMU
- Local extracted notes:
  - `RC_vehicle_dynamics_tables_summary.md`
  - `RC_vehicle_dynamics_tables.xlsx`
  - `Equations.xlsx`

## Loading Strategy

The companion `ConfigLoader.py` module can:

- load one YAML file
- load the whole config bundle from this folder

The companion `ModelFactory.py` module can:

- build one shared runtime bundle from the same YAML config set
- construct the EKF and dynamics model from that shared bundle
- expose frame and controller runtime settings alongside the models

That keeps usage simple while still allowing the codebase to migrate gradually
from hardcoded constants to config-driven values.
