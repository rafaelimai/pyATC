import pytest

from pid import PID


def test_proportional_output():
    assert PID(2, 0, 0).calculate_control_function(3, 1) == 6


def test_integral_accumulates():
    pid = PID(0, 1, 0)
    assert pid.calculate_control_function(2, 0.5) == 1
    assert pid.calculate_control_function(2, 0.5) == 2


def test_derivative_uses_previous_error_and_skips_first_kick():
    pid = PID(0, 0, 2)
    assert pid.calculate_control_function(3, 1) == 0
    assert pid.calculate_control_function(5, 0.5) == 8


def test_reset_clears_memory():
    pid = PID(0, 1, 1)
    pid.calculate_control_function(2, 1)
    pid.reset()
    assert pid.integral_error == 0
    assert pid.previous_error is None
    assert pid.calculate_control_function(2, 1) == 2


@pytest.mark.parametrize("dt", [0, -1])
def test_invalid_dt(dt):
    with pytest.raises(ValueError, match="dt"):
        PID(1, 0, 0).calculate_control_function(1, dt)
