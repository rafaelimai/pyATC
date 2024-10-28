# This paper is really nice. Used it a lot when I was still on academia.
# I'm not that much into modelling lift and aerodynamic stuff for this purpose. I'll make something simpler.


# DOCUMENT CONTROL SHEET
# Title of document: A multi-aircraft model for conflict detection and resolution algorithm
# evaluation
# Authors of document: W. Glover and J. Lygeros
# Deliverable number: D1.3
# Contract: IST-2001-32460 of European Commission
# Project: Distributed Control and Stochastic Analysis of Hybrid Systems Supporting
# Safety Critical Real-Time Systems Design (HYBRIDGE)

import math
from autopilot import Autopilot

INDEX_MAX_SPEED = 0 
INDEX_TURNING_RATE = 1
INDEX_RATE_OF_CLIMB = 2 
INDEX_FUEL_BURN_LINEAR = 3 
INDEX_FUEL_LEVEL = 4

class Aircraft: 

    current_time = 0
    fuel_level = 0
    x, y, z = 0, 0, 0
    speed_x, speed_y, speed_z = 0,0,0
    heading = 0
    angular_speed = 0
    log = []
    route = None
    autopilot = None
    
    LOG_X_INDEX = 0
    LOG_Y_INDEX = 1
    LOG_Z_INDEX = 2
    LOG_SPEED_X_INDEX = 3
    LOG_SPEED_Y_INDEX = 4
    LOG_SPEED_Z_INDEX = 5
    LOG_HEADING = 6
    LOG_ANGULAR_SPEED = 7
    LOG_FUEL_LEVEL = 8
    LOG_TURNING_RATE_DELTA = 9
    LOG_VERTICAL_SPEED_DELTA = 10

    def __init__(self, params, x, y, z, heading, delta_time, route):
        self.max_speed = params[INDEX_MAX_SPEED] 
        self.turning_rate = params[INDEX_TURNING_RATE]
        self.roc = params[INDEX_RATE_OF_CLIMB] 
        self.fuel_burn_linear = params[INDEX_FUEL_BURN_LINEAR] 
        self.fuel_level = params[INDEX_FUEL_LEVEL] 
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading
        self.speed_x = self.max_speed * math.sin(heading)
        self.speed_y = self.max_speed * math.cos(heading)
        self.delta_time = delta_time
        self.autopilot = Autopilot(delta_time)
        self.route = route
    
    def advance_step(self, delta_time):
 
        self.fuel_level -= self.fuel_burn_linear * delta_time 

        turning_rate_delta, vertical_speed_delta = self.autopilot.iterate(self.x, self.y, self.z, self.route, self.heading, self.turning_rate, self.roc)
        
        if(abs(self.speed_z + vertical_speed_delta)>=self.roc):
            self.speed_z = math.copysign(self.roc, self.speed_z)
        else:
            self.speed_z += vertical_speed_delta

        if(abs(self.angular_speed+turning_rate_delta)>= self.turning_rate):
            self.angular_speed = math.copysign(self.turning_rate, self.angular_speed)
        else:   
            self.angular_speed += turning_rate_delta

        self.heading += self.angular_speed * delta_time
        
        horizontal_component = math.sqrt(self.max_speed**2 - self.speed_z**2)
        self.speed_x = horizontal_component * math.sin(self.heading)
        self.speed_y = horizontal_component * math.cos(self.heading)

        
        self.x += self.speed_x * delta_time
        self.y += self.speed_y * delta_time
        self.z += self.speed_z * delta_time


        stats = []
        stats.append(self.x)
        stats.append(self.y)
        stats.append(self.z)
        stats.append(self.speed_x)
        stats.append(self.speed_y)
        stats.append(self.speed_z)
        
        stats.append(self.heading)        
        stats.append(self.angular_speed)
        
        stats.append(self.fuel_level)               
        stats.append(turning_rate_delta)
        stats.append(vertical_speed_delta)

        self.log.append(stats)




    def check_distance(self, aircraft, threshold):
        distance = math.sqrt((self.x - aircraft.x)**2 + (self.y - aircraft.y)**2 + (self.z - aircraft.z)**2)
        return distance > threshold