"""Simplified point-mass aircraft kinematics."""

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import NamedTuple

from autopilot import Autopilot, normalize_angle
from pid import PIDGains


@dataclass(frozen=True)
class AircraftPerformance:
    """Limits in m/s, rad/s, m/s, fuel units/s, and fuel units."""

    max_speed: float
    max_turn_rate: float
    max_vertical_speed: float
    fuel_burn_rate: float
    initial_fuel_level: float = 0.0

    def __post_init__(self) -> None:
        if self.max_speed < 0 or self.max_turn_rate < 0 or self.max_vertical_speed < 0:
            raise ValueError("aircraft speed and rate limits cannot be negative")
        if self.max_vertical_speed > self.max_speed:
            raise ValueError("max_vertical_speed cannot exceed max_speed")
        if self.fuel_burn_rate < 0:
            raise ValueError("fuel_burn_rate cannot be negative")


class AircraftLogEntry(NamedTuple):
    """Named snapshot produced after each simulation step."""

    x: float
    y: float
    z: float
    speed_x: float
    speed_y: float
    speed_z: float
    heading: float
    angular_speed: float
    fuel_level: float
    turning_rate_delta: float
    vertical_speed_delta: float


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class Aircraft:
    """Aircraft in metres, seconds, and aviation-style radians (0 north)."""

    # Backward-compatible indexes for notebooks that access log entries positionally.
    LOG_X_INDEX = 0
    LOG_Y_INDEX = 1
    LOG_Z_INDEX = 2
    LOG_SPEED_X_INDEX = 3
    LOG_SPEED_Y_INDEX = 4
    LOG_SPEED_Z_INDEX = 5
    LOG_HEADING = 6
    LOG_ANGULAR_SPEED = 7
    LOG_FUEL_LEVEL = 8
    LOG_TURNING_RATE_DELTA = 9
    LOG_VERTICAL_SPEED_DELTA = 10

    def __init__(
        self,
        performance: AircraftPerformance,
        x: float,
        y: float,
        z: float,
        heading: float,
        route: Sequence[Sequence[float]] | None = None,
        waypoint_capture_radius: float = 100.0,
        heading_gains: PIDGains | None = None,
        vertical_gains: PIDGains | None = None,
    ) -> None:
        if not isinstance(performance, AircraftPerformance):
            raise TypeError("performance must be an AircraftPerformance instance")
        self.performance = performance
        self.max_speed = performance.max_speed
        self.turning_rate = performance.max_turn_rate
        self.roc = performance.max_vertical_speed
        self.fuel_burn_linear = performance.fuel_burn_rate
        self.fuel_level = performance.initial_fuel_level
        self.x, self.y, self.z = x, y, z
        self.heading = normalize_angle(heading)
        self.speed_x = self.max_speed * math.sin(self.heading)
        self.speed_y = self.max_speed * math.cos(self.heading)
        self.speed_z = 0.0
        self.angular_speed = 0.0
        self.current_time = 0.0
        self.route = route
        self.log: list[AircraftLogEntry] = []
        autopilot_options = {}
        if heading_gains is not None:
            autopilot_options["heading_gains"] = heading_gains
        if vertical_gains is not None:
            autopilot_options["vertical_gains"] = vertical_gains
        self.autopilot = Autopilot(route, waypoint_capture_radius, **autopilot_options)

    def advance_step(self, dt: float) -> None:
        """Advance dynamics; *dt* is also the authoritative PID timestep."""
        if dt <= 0:
            raise ValueError("dt must be greater than zero")
        self.fuel_level -= self.fuel_burn_linear * dt
        turning_acceleration, vertical_acceleration = self.autopilot.iterate(
            self.x, self.y, self.z, self.heading, dt
        )
        turning_delta = turning_acceleration * dt
        vertical_delta = vertical_acceleration * dt
        self.speed_z = clamp(self.speed_z + vertical_delta, -self.roc, self.roc)
        self.angular_speed = clamp(
            self.angular_speed + turning_delta, -self.turning_rate, self.turning_rate
        )
        self.heading = normalize_angle(self.heading + self.angular_speed * dt)
        horizontal_speed = math.sqrt(max(0.0, self.max_speed**2 - self.speed_z**2))
        self.speed_x = horizontal_speed * math.sin(self.heading)
        self.speed_y = horizontal_speed * math.cos(self.heading)
        self.x += self.speed_x * dt
        self.y += self.speed_y * dt
        self.z += self.speed_z * dt
        self.current_time += dt
        self.log.append(AircraftLogEntry(
            self.x, self.y, self.z, self.speed_x, self.speed_y, self.speed_z,
            self.heading, self.angular_speed, self.fuel_level, turning_delta, vertical_delta,
        ))

    def distance_to(self, other_aircraft: "Aircraft") -> float:
        """Return three-dimensional separation in metres."""
        return math.dist(
            (self.x, self.y, self.z),
            (other_aircraft.x, other_aircraft.y, other_aircraft.z),
        )

    def is_separated_from(self, other_aircraft: "Aircraft", minimum_distance: float) -> bool:
        if minimum_distance < 0:
            raise ValueError("minimum_distance cannot be negative")
        return self.distance_to(other_aircraft) >= minimum_distance
