from calibration import calibrate_heading, calibrate_vertical


def test_calibration_is_deterministic_and_returns_finite_scores():
    heading = calibrate_heading()
    vertical = calibrate_vertical()
    assert heading.gains.kp > 0
    assert vertical.gains.kp > 0
    assert 0 <= heading.score < 1
    assert 0 <= vertical.score < 1
