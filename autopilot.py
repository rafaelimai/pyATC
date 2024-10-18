# Years ago, I read on a book that the PID control was the best control to be used in the absence of further details about whatever you were controlling.
# Who am I to cast doubt on the literature?

import pid
import math

class Autopilot:

    def __init__(self):
        pass


    #turning_rate_delta, vertical_speed_delta = self.autopilot.iterate(self.x, self.y, self.z, self.route, self.turning_rate, self.roc)

    def iterate(x, y, z, route, turning_rate_limit, roc_limit):

        distances = [math.sqrt((x - i[0])^2+(y - i[1])^2+(z - i[2])^2) for i in route]
        waypoint_index = distances.index(min(distances))

        x_waypoint = route[waypoint_index][0]
        y_waypoint = route[waypoint_index][1]
        z_waypoint = route[waypoint_index][2]

        
