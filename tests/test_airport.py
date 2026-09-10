import math
from types import SimpleNamespace

import pytest

from airport import AircraftNotGroundedError, Airport, AirportCapacityError


@pytest.mark.parametrize(
    ("angle", "upper"),
    [(0, (0, 500)), (math.pi / 2, (500, 0)), (math.pi / 4, (500 / math.sqrt(2), 500 / math.sqrt(2)))],
)
def test_runway_geometry_uses_aviation_heading(angle, upper):
    airport = Airport(1, 0, 0, angle)
    assert (airport.runway_upper_x, airport.runway_upper_y) == pytest.approx(upper)
    assert (airport.runway_lower_x, airport.runway_lower_y) == pytest.approx((-upper[0], -upper[1]))


def test_touchdown_direction_and_distance_are_coherent():
    airport = Airport(1, 10, 20, 0)
    aircraft = SimpleNamespace(x=10, y=-480, z=12)
    assert airport.touchdown_point == (10, -480, 0)
    assert airport.distance_to_touchdown(aircraft) == 12
    airport.takeoff_on_upper = False
    assert airport.touchdown_point == (10, 520, 0)
    assert airport.distance_to_touchdown(aircraft) == pytest.approx(math.hypot(1000, 12))


def test_airport_state_capacity_and_duplicates_are_explicit():
    first, second = Airport(1, 0, 0, 0), Airport(1, 0, 0, 0)
    aircraft = SimpleNamespace(x=0, y=0, identifier="first")
    another = SimpleNamespace(x=0, y=0, identifier="second")
    assert first.land_aircraft(aircraft)
    assert not first.land_aircraft(aircraft)
    assert second.grounded_aircraft == []
    with pytest.raises(AirportCapacityError):
        first.land_aircraft(another)


def test_takeoff_requires_grounded_aircraft_and_positions_it():
    airport = Airport(1, 0, 0, math.pi / 2)
    aircraft = SimpleNamespace(x=0, y=0)
    with pytest.raises(AircraftNotGroundedError):
        airport.takeoff_aircraft(aircraft)
    airport.land_aircraft(aircraft)
    assert airport.takeoff_aircraft(aircraft)
    assert (aircraft.x, aircraft.y) == pytest.approx((500, 0))
    assert aircraft not in airport.grounded_aircraft
