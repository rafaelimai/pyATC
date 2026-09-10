import math

from aircraft import Aircraft, AircraftPerformance
from autopilot import Autopilot, normalize_angle


def test_route_progresses_in_order_then_holds_current_state():
    route = [(0, 0, 100), (1000, 0, 200), (2000, 0, 300)]
    autopilot = Autopilot(route, waypoint_capture_radius=100)
    autopilot.iterate(0, 0, 100, 0, 1)
    assert autopilot.current_waypoint_index == 1
    autopilot.iterate(1000, 0, 200, 0.5, 1)
    assert autopilot.current_waypoint_index == 2
    autopilot.iterate(2000, 0, 300, 0.75, 1)
    assert autopilot.route_complete
    assert autopilot.current_waypoint_index == 2
    assert autopilot.iterate(0, 0, 300, 0.75, 1) == (0, 0)


def test_altitude_converges_with_different_timesteps():
    performance = AircraftPerformance(20, math.pi / 8, 3, 0)
    for dt in (0.25, 1.0):
        aircraft = Aircraft(performance, 0, 0, 300, 0, [(0, 100_000, 200)])
        initial_error = 100
        for _ in range(round(20 / dt)):
            aircraft.advance_step(dt)
        assert abs(aircraft.z - 200) < initial_error


def test_heading_converges():
    performance = AircraftPerformance(20, math.pi / 12, 3, 0)
    aircraft = Aircraft(performance, 0, 0, 0, -math.pi / 2, [(0, 100_000, 0)])
    initial_error = abs(normalize_angle(0 - aircraft.heading))
    for _ in range(30):
        aircraft.advance_step(0.2)
    assert abs(normalize_angle(0 - aircraft.heading)) < initial_error
