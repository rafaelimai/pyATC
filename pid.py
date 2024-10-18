# Simple PID implementation


#https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller

class PID:

    previous_error = 0
    integral_error = 0
    time_delta = 0

    def __init__(self, Kp, Ki, Kd, time_delta):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.time_delta = time_delta
        pass

    def calculate_control_function(self, error_value):
        control_result = self.Kp * error_value
        control_result += self.Ki * (error_value - self.previous_error) * self.time_delta
        control_result += self.Kd * (error_value - self.previous_error) / self.time_delta
        return control_result