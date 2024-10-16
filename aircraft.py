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

INDEX_MAX_SPEED = 0
INDEX_ACCEL_DECEL_RATE = 1
INDEX_TURNING_RATE = 2
INDEX_RATE_OF_CLIMB = 3
INDEX_RATE_OF_DESCENT = 4
INDEX_FUEL_BURN_QUADRATIC = 5
INDEX_FUEL_BURN_LINEAR = 6
INDEX_FUEL_BURN_BASE = 7

class Aircraft: 

    current_time = 0
    fuel_level = 0
    x, y, z = 0, 0, 0
    route = []

    def __init__(self, params, fuel_level, x, y, z):
        self.max_speed = params[INDEX_MAX_SPEED]
        self.accel_rate = params[INDEX_ACCEL_DECEL_RATE]
        self.turning_rate = params[INDEX_TURNING_RATE]
        self.roc = params[INDEX_RATE_OF_CLIMB]
        self.rod = params[INDEX_RATE_OF_DESCENT]
        self.fuel_burn_quad = params[INDEX_FUEL_BURN_QUADRATIC]
        self.fuel_burn_linear = params[INDEX_FUEL_BURN_LINEAR]
        self.fuel_burn_base = params[INDEX_FUEL_BURN_BASE] 
        self.fuel_level = fuel_level
        self.x = x
        self.y = y
        self.z = z
    
    def advance_step(delta_time):
        print("todo")

    def check_distance(aircraft, threshold):
        print("todo")