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



class Aircraft:
    INDEX_MAX_SPEED = 0
    INDEX_ACCEL_DECEL_RATE = 1
    INDEX_TURNING_RATE = 2
    INDEX_RATE_OF_CLIMB = 3
    INDEX_RATE_OF_DESCENT = 4
    INDEX_FUEL_BURN_QUADRATIC = 5
    INDEX_FUEL_BURN_LINEAR = 6
    INDEX_FUEL_BURN_BASE = 7


    def __init__(self, params):
        