"""Deterministic grid-search calibration for pyATC's two PID loops."""

import argparse
import itertools
import math
from dataclasses import dataclass
from typing import Callable, Iterable

from aircraft import clamp
from autopilot import normalize_angle
from pid import PID, PIDGains


@dataclass(frozen=True)
class CalibrationResult:
    gains: PIDGains
    score: float
    mean_absolute_error: float
    terminal_error: float
    overshoot: float
    saturation_fraction: float


def _simulate_step(
    gains: PIDGains,
    target: float,
    duration: float,
    dt: float,
    rate_limit: float,
    normalize: Callable[[float], float] | None = None,
) -> tuple[float, float, float, float]:
    pid = PID(**vars(gains))
    value = 0.0
    rate = 0.0
    absolute_error = 0.0
    overshoot = 0.0
    saturated_steps = 0
    steps = round(duration / dt)

    for _ in range(steps):
        error = target - value
        if normalize is not None:
            error = normalize(error)
        acceleration = pid.calculate_control_function(error, dt)
        candidate_rate = rate + acceleration * dt
        rate = clamp(candidate_rate, -rate_limit, rate_limit)
        saturated_steps += int(not math.isclose(candidate_rate, rate))
        value += rate * dt
        if normalize is not None:
            value = normalize(value)
        absolute_error += abs(error) * dt
        if target >= 0:
            overshoot = max(overshoot, value - target)
        else:
            overshoot = max(overshoot, target - value)

    terminal_error = target - value
    if normalize is not None:
        terminal_error = normalize(terminal_error)
    return absolute_error / duration, abs(terminal_error), overshoot, saturated_steps / steps


def _evaluate(
    gains: PIDGains,
    targets: Iterable[float],
    duration: float,
    dt: float,
    rate_limit: float,
    normalize: Callable[[float], float] | None = None,
) -> CalibrationResult:
    metrics = [
        _simulate_step(gains, target, duration, dt, rate_limit, normalize)
        for target in targets
    ]
    scales = [abs(target) for target in targets]
    mae = sum(metric[0] / scale for metric, scale in zip(metrics, scales)) / len(metrics)
    terminal = sum(metric[1] / scale for metric, scale in zip(metrics, scales)) / len(metrics)
    overshoot = sum(metric[2] / scale for metric, scale in zip(metrics, scales)) / len(metrics)
    saturation = sum(metric[3] for metric in metrics) / len(metrics)
    score = mae + 4.0 * terminal + 2.0 * overshoot + 0.25 * saturation
    return CalibrationResult(gains, score, mae, terminal, overshoot, saturation)


def _search(
    kp_values: Iterable[float],
    ki_values: Iterable[float],
    kd_values: Iterable[float],
    evaluator: Callable[[PIDGains], CalibrationResult],
) -> CalibrationResult:
    results = (
        evaluator(PIDGains(kp, ki, kd))
        for kp, ki, kd in itertools.product(kp_values, ki_values, kd_values)
    )
    return min(results, key=lambda result: result.score)


def calibrate_vertical(dt: float = 0.2) -> CalibrationResult:
    """Calibrate altitude response for representative +/- 300 m steps."""
    return _search(
        kp_values=(0.0005, 0.001, 0.002, 0.005, 0.01, 0.02),
        ki_values=(0.0, 0.000001, 0.000005, 0.00001),
        kd_values=(0.01, 0.02, 0.05, 0.1, 0.2),
        evaluator=lambda gains: _evaluate(
            gains, (-300.0, 300.0), duration=180.0, dt=dt, rate_limit=3.0
        ),
    )


def calibrate_heading(dt: float = 0.1) -> CalibrationResult:
    """Calibrate turns in both directions, including a near-180 degree turn."""
    return _search(
        kp_values=(0.05, 0.1, 0.2, 0.4, 0.8),
        ki_values=(0.0, 0.0001, 0.0005, 0.001),
        kd_values=(0.1, 0.2, 0.4, 0.8, 1.2),
        evaluator=lambda gains: _evaluate(
            gains,
            (-math.pi / 2, math.pi / 2, math.pi - 0.1),
            duration=60.0,
            dt=dt,
            rate_limit=math.pi / 60,
            normalize=normalize_angle,
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vertical-dt", type=float, default=0.2)
    parser.add_argument("--heading-dt", type=float, default=0.1)
    args = parser.parse_args()
    if args.vertical_dt <= 0 or args.heading_dt <= 0:
        parser.error("timesteps must be greater than zero")

    for name, result in (
        ("heading", calibrate_heading(args.heading_dt)),
        ("vertical", calibrate_vertical(args.vertical_dt)),
    ):
        print(
            f"{name}: kp={result.gains.kp:g}, ki={result.gains.ki:g}, "
            f"kd={result.gains.kd:g}, score={result.score:.5f}, "
            f"mae={result.mean_absolute_error:.5f}, "
            f"terminal={result.terminal_error:.5f}, "
            f"overshoot={result.overshoot:.5f}, "
            f"saturation={result.saturation_fraction:.3%}"
        )


if __name__ == "__main__":
    main()
