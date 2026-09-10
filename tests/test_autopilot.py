import math

import pytest

from autopilot import Autopilot, normalize_angle


@pytest.mark.parametrize(
    ("waypoint", "expected"),
    [
        ((0, 10, 0), 0), ((0, -10, 0), -math.pi),
        ((10, 0, 0), math.pi / 2), ((-10, 0, 0), -math.pi / 2),
        ((10, 10, 0), math.pi / 4), ((-10, 10, 0), -math.pi / 4),
        ((10, -10, 0), 3 * math.pi / 4), ((-10, -10, 0), -3 * math.pi / 4),
    ],
)
def test_heading_for_cardinal_and_intercardinal_targets(waypoint, expected):
    autopilot = Autopilot([waypoint], waypoint_capture_radius=0)
    turn, _ = autopilot.iterate(0, 0, 0, 0, 1)
    assert turn == pytest.approx(0.4 * expected)


@pytest.mark.parametrize(
    ("current_degrees", "target_degrees", "positive"),
    [(359, 1, True), (1, 359, False)],
)
def test_shortest_turn_across_heading_boundary(current_degrees, target_degrees, positive):
    current = math.radians(current_degrees)
    target = math.radians(target_degrees)
    waypoint = (1000 * math.sin(target), 1000 * math.cos(target), 0)
    turn, _ = Autopilot([waypoint]).iterate(0, 0, 0, current, 1)
    assert (turn > 0) is positive
    assert abs(normalize_angle(target - current)) == pytest.approx(math.radians(2))


def test_vertical_controller_descends_toward_lower_waypoint():
    autopilot = Autopilot([(0, 1000, 200)])
    _, vertical = autopilot.iterate(0, 0, 300, 0, 1)
    assert vertical < 0
    assert autopilot.pid_heading.integral_error == 0
    assert autopilot.pid_vertical.integral_error == -100


def test_no_route_holds_initial_state():
    autopilot = Autopilot()
    assert autopilot.iterate(0, 0, 300, 1.2, 1) == (0, 0)


def test_empty_and_malformed_routes_are_explicit():
    with pytest.raises(ValueError, match="route"):
        Autopilot([])
    with pytest.raises(ValueError, match="waypoint"):
        Autopilot([(1, 2)])
