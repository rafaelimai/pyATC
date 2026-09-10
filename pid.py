"""Small, stateful discrete PID controller."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PIDGains:
    """Explicit proportional, integral, and derivative gains."""

    kp: float
    ki: float
    kd: float


class PID:
    """A PID using the error convention ``target - actual``."""

    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()

    def reset(self) -> None:
        """Clear accumulated integral and derivative history."""
        self.integral_error = 0.0
        self.previous_error: float | None = None

    def calculate_control_function(self, error_value: float, dt: float) -> float:
        """Return the control output for *error_value* over *dt* seconds."""
        if dt <= 0:
            raise ValueError("dt must be greater than zero")
        self.integral_error += error_value * dt
        derivative_error = 0.0
        if self.previous_error is not None:
            derivative_error = (error_value - self.previous_error) / dt
        result = (
            self.kp * error_value
            + self.ki * self.integral_error
            + self.kd * derivative_error
        )
        self.previous_error = error_value
        return result
