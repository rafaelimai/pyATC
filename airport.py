"""Minimal airport capacity and runway geometry model."""

import math

DEFAULT_RUNWAY_LENGTH = 1000.0


class AirportCapacityError(RuntimeError):
    """Raised when an aircraft cannot land because the airport is full."""


class AircraftNotGroundedError(ValueError):
    """Raised when takeoff is requested for an aircraft not at the airport."""


class Airport:
    """A sea-level runway whose angle is 0 north and increases clockwise."""

    def __init__(
        self,
        aircraft_limit: int,
        runway_x: float,
        runway_y: float,
        runway_angle: float,
        runway_length: float = DEFAULT_RUNWAY_LENGTH,
    ) -> None:
        if aircraft_limit < 0:
            raise ValueError("aircraft_limit cannot be negative")
        if runway_length <= 0:
            raise ValueError("runway_length must be greater than zero")
        self.aircraft_limit = aircraft_limit
        self.runway_x = runway_x
        self.runway_y = runway_y
        self.runway_angle = runway_angle
        self.runway_length = runway_length
        self.grounded_aircraft: list[object] = []
        self.takeoff_on_upper = True
        half_length = runway_length / 2
        dx = half_length * math.sin(runway_angle)
        dy = half_length * math.cos(runway_angle)
        self.runway_upper_x = runway_x + dx
        self.runway_upper_y = runway_y + dy
        self.runway_lower_x = runway_x - dx
        self.runway_lower_y = runway_y - dy

    @property
    def touchdown_point(self) -> tuple[float, float, float]:
        if self.takeoff_on_upper:
            return self.runway_lower_x, self.runway_lower_y, 0.0
        return self.runway_upper_x, self.runway_upper_y, 0.0

    def land_aircraft(self, aircraft: object) -> bool:
        if aircraft in self.grounded_aircraft:
            return False
        if len(self.grounded_aircraft) >= self.aircraft_limit:
            raise AirportCapacityError("airport has reached its aircraft capacity")
        self.grounded_aircraft.append(aircraft)
        return True

    def takeoff_aircraft(self, aircraft: object) -> bool:
        if aircraft not in self.grounded_aircraft:
            raise AircraftNotGroundedError("aircraft is not grounded at this airport")
        if self.takeoff_on_upper:
            aircraft.x, aircraft.y = self.runway_upper_x, self.runway_upper_y
        else:
            aircraft.x, aircraft.y = self.runway_lower_x, self.runway_lower_y
        self.grounded_aircraft.remove(aircraft)
        return True

    def distance_to_touchdown(self, aircraft: object) -> float:
        """Return 3-D distance in metres to the active touchdown endpoint."""
        return math.dist((aircraft.x, aircraft.y, aircraft.z), self.touchdown_point)
