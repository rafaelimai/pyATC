# pyATC

pyATC is an experimental Python playground for simplified aircraft kinematics,
waypoint guidance, autopilot control, and future air-traffic conflict-detection
experiments.

The current core models a constant-total-speed aircraft, independent PID loops
for heading and altitude, sequential waypoint capture, basic airport/runway
geometry, and airport ground capacity. It is a learning project—not a certified
or operational ATC tool.

## Model conventions

- Positions are metres and simulation time is seconds.
- Speeds and vertical speeds are metres per second; turn rates are radians per second.
- Heading `0` is north (`+Y`), `pi/2` is east (`+X`), and positive rotation is clockwise.
- PID errors use `target - actual` and `Aircraft.advance_step(dt)` owns the timestep.
  PID outputs are accelerations; velocity and turn rate integrate them with `* dt`.
- Routes contain `(x, y, z)` waypoints and advance sequentially within a configurable
  capture radius (100 m by default).
- A completed route holds the heading and altitude present at final capture. An
  aircraft without a route holds its initial heading and altitude.
- The ground is flat at sea level and aircraft/runway dynamics are deliberately simple.

## Architecture

- `Aircraft` propagates kinematics and owns per-aircraft state.
- `AircraftPerformance` names the aircraft limits and fuel parameters.
- `Autopilot` manages route progress and its independent control loops.
- `PID` implements a stateful discrete controller.
- `Airport` handles runway endpoints, touchdown distance, and basic ground capacity.
- `Airspace` remains a small container for dimensions and airports.

## Quick start

Python 3.10 or newer is required. Install the test dependency and run the suite:

```bash
python -m pip install -e '.[test]'
python -m pytest
```

Run the deterministic PID grid search with:

```bash
python calibration.py
# or, after installation:
pyatc-calibrate
```

The calibrator evaluates step responses in both directions and reports normalized
mean/terminal error, overshoot, saturation, and a combined score. The current
first-pass defaults produced by this search are heading `(0.4, 0, 1.2)` and
vertical `(0.01, 0, 0.2)` for `(Kp, Ki, Kd)`.

Example simulation:

```python
import math

from aircraft import Aircraft, AircraftPerformance

performance = AircraftPerformance(
    max_speed=38.0,
    max_turn_rate=math.pi / 60,
    max_vertical_speed=3.0,
    fuel_burn_rate=0.00722,
    initial_fuel_level=55.0,
)
aircraft = Aircraft(
    performance=performance,
    x=0,
    y=0,
    z=300,
    heading=0,
    route=[(0, 5_000, 200)],
)

for _ in range(60):
    aircraft.advance_step(1.0)
```

## Known limitations and roadmap

The model omits aerodynamics, realistic engine and fuel behavior, weather,
terrain, navigation databases, takeoff/landing dynamics, and operational ATC
rules. PID gains are automated first-pass values, not aircraft-specific or
operationally validated tuning. The next planned milestone is predictive multi-aircraft conflict
detection using CPA/TCPA; networking and client/server transport remain future
work and should use named schema fields rather than positional parameter lists.
