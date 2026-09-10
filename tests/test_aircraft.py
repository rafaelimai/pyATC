import math

import pytest

from aircraft import Aircraft, AircraftPerformance, clamp


@pytest.fixture
def performance():
    return AircraftPerformance(20, math.pi / 8, 3, 0.1, 10)


def test_instance_state_is_independent(performance):
    first = Aircraft(performance, 0, 0, 0, 0)
    second = Aircraft(performance, 0, 0, 0, 0)
    first.log.append((1,))
    assert second.log == []


@pytest.mark.parametrize(
    ("current", "delta", "expected"),
    [(2, -10, -3), (-2, 10, 3), (1, 1, 2)],
)
def test_candidate_saturation_preserves_sign(current, delta, expected):
    assert clamp(current + delta, -3, 3) == expected


def test_distance_and_separation(performance):
    first = Aircraft(performance, 0, 0, 0, 0)
    second = Aircraft(performance, 3, 4, 12, 0)
    assert first.distance_to(second) == 13
    assert first.is_separated_from(second, 13)
    assert not first.is_separated_from(second, 14)


def test_advance_uses_dt_for_time_and_rejects_invalid(performance):
    aircraft = Aircraft(performance, 0, 0, 0, 0)
    aircraft.advance_step(0.25)
    assert aircraft.current_time == 0.25
    assert aircraft.log[0].vertical_speed_delta == 0
    assert aircraft.log[0][Aircraft.LOG_VERTICAL_SPEED_DELTA] == 0
    assert aircraft.log[0].x == aircraft.x
    with pytest.raises(ValueError, match="dt"):
        aircraft.advance_step(0)


def test_pid_accelerations_are_integrated_over_dt(performance):
    aircraft = Aircraft(performance, 0, 0, 0, 0)
    aircraft.autopilot.iterate = lambda *args: (0.2, 2.0)
    aircraft.advance_step(0.5)
    assert aircraft.angular_speed == pytest.approx(0.1)
    assert aircraft.speed_z == pytest.approx(1.0)
    assert aircraft.log[0].turning_rate_delta == pytest.approx(0.1)
    assert aircraft.log[0].vertical_speed_delta == pytest.approx(1.0)
