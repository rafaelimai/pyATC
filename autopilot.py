# Years ago, I read on a book that the PID control was the best control to be used in the absence of further details about whatever you were controlling.
# Who am I to cast doubt on the literature?

from pid import PID
import math

class Autopilot:

    pid_heading = ""
    pid_vs = ""
    time_delta = 0
    def __init__(self, time_delta):
        self.time_delta = time_delta
        self.pid_heading = PID(Kp = 0.1, Ki = 0.1, Kd = 0.1, time_delta=self.time_delta)
        self.pid_vs = PID(Kp = 0.1, Ki = 0.1, Kd = 0.1, time_delta=self.time_delta)

        pass


    #turning_rate_delta, vertical_speed_delta = self.autopilot.iterate(self.x, self.y, self.z, self.route, self.turning_rate, self.roc)

    def iterate(self, x, y, z, route, heading, turning_rate_limit, roc_limit):

        distances = [math.sqrt((x - i[0])^2+(y - i[1])^2+(z - i[2])^2) for i in route]
        waypoint_index = distances.index(min(distances))

        # closest waypoint
        x_waypoint = route[waypoint_index][0]
        y_waypoint = route[waypoint_index][1]
        z_waypoint = route[waypoint_index][2]

        desired_heading = math.atan((x-x_waypoint)/(y-y_waypoint))
        error_heading = heading - desired_heading
        error_height = z - z_waypoint

        turning_rate_delta = self.pid_heading.calculate_control_function(error_heading)
        vertical_speed_delta = self.pid_heading.calculate_control_function(error_height)
        
        return turning_rate_delta, vertical_speed_delta
        
