"""Waypoint and state-hold guidance for the simplified aircraft model."""

import math
from collections.abc import Sequence

from pid import PID, PIDGains

Waypoint = tuple[float, float, float]

DEFAULT_HEADING_GAINS = PIDGains(kp=0.4, ki=0.0, kd=1.2)
DEFAULT_VERTICAL_GAINS = PIDGains(kp=0.01, ki=0.0, kd=0.2)


def normalize_angle(angle: float) -> float:
    """Normalize radians to the half-open range [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _validate_route(route: Sequence[Sequence[float]] | None) -> tuple[Waypoint, ...] | None:
    if route is None:
        return None
    if len(route) == 0:
        raise ValueError("route must contain at least one waypoint or be None")
    validated: list[Waypoint] = []
    for waypoint in route:
        valid_shape = (
            isinstance(waypoint, Sequence)
            and not isinstance(waypoint, (str, bytes))
            and len(waypoint) == 3
        )
        if not valid_shape or any(
            not isinstance(value, (int, float)) or not math.isfinite(value)
            for value in waypoint
        ):
            raise ValueError("each waypoint must contain finite numeric x, y, and z values")
        validated.append((float(waypoint[0]), float(waypoint[1]), float(waypoint[2])))
    return tuple(validated)


class Autopilot:
    """Independent lateral and vertical control with sequential route progress."""

    def __init__(
        self,
        route: Sequence[Sequence[float]] | None = None,
        waypoint_capture_radius: float = 100.0,
        heading_gains: PIDGains = DEFAULT_HEADING_GAINS,
        vertical_gains: PIDGains = DEFAULT_VERTICAL_GAINS,
    ) -> None:
        if waypoint_capture_radius < 0:
            raise ValueError("waypoint_capture_radius cannot be negative")
        self.route = _validate_route(route)
        self.waypoint_capture_radius = waypoint_capture_radius
        self.current_waypoint_index = 0
        self.route_complete = False
        self.pid_heading = PID(**vars(heading_gains))
        self.pid_vertical = PID(**vars(vertical_gains))
        self.pid_vs = self.pid_vertical
        self._hold_heading: float | None = None
        self._hold_altitude: float | None = None

    @property
    def active_waypoint(self) -> Waypoint | None:
        if self.route is None:
            return None
        return self.route[self.current_waypoint_index]

    def _capture_hold_state(self, heading: float, altitude: float) -> None:
        self._hold_heading = normalize_angle(heading)
        self._hold_altitude = altitude
        self.pid_heading.reset()
        self.pid_vertical.reset()

    def iterate(
        self, x: float, y: float, z: float, heading: float, dt: float
    ) -> tuple[float, float]:
        """Return angular and vertical accelerations for one simulation step."""
        if dt <= 0:
            raise ValueError("dt must be greater than zero")
        if self.route is None or self.route_complete:
            if self._hold_heading is None:
                self._capture_hold_state(heading, z)
            target_heading = self._hold_heading
            target_altitude = self._hold_altitude
        else:
            waypoint = self.route[self.current_waypoint_index]
            if math.dist((x, y, z), waypoint) <= self.waypoint_capture_radius:
                if self.current_waypoint_index < len(self.route) - 1:
                    self.current_waypoint_index += 1
                    waypoint = self.route[self.current_waypoint_index]
                else:
                    self.route_complete = True
                    self._capture_hold_state(heading, z)
                    target_heading = self._hold_heading
                    target_altitude = self._hold_altitude
            if not self.route_complete:
                dx, dy = waypoint[0] - x, waypoint[1] - y
                target_heading = heading if dx == 0 and dy == 0 else math.atan2(dx, dy)
                target_altitude = waypoint[2]
        heading_error = normalize_angle(target_heading - heading)
        altitude_error = target_altitude - z
        return (
            self.pid_heading.calculate_control_function(heading_error, dt),
            self.pid_vertical.calculate_control_function(altitude_error, dt),
        )
