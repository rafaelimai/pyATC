import math

# A 1000m runway is pretty much enough for general aviation aircraft. 
DEFAULT_RUNWAY_LENGTH = 1000

# If an aircraft's distance (considering x, y and z) is less than 20 meters, let's say it was a landing.
SUCCESSFUL_LANDING_BOUNDARY = 20

class Airport:
    """ 
    Although airports are not technically on the airspace, I'll place them inside the class for the sake of simplicity.

    runway_x/runway_y = (x,y) coordinates of the runway
    runway_angle = angle in respect to the x-axis. A runway parallel to the x-axis would have a runway_angle of 0. Angles in radians.

    Let's simplify the airport as if it was on the sea level (quasi-Netherlands hypothesis)
    """
    grounded_aircraft = []

    # By default, aircraft will take off facing the upper point of the runway.
    takeoff_on_upper = True

    def __init__(self, aircraft_limit, runway_x, runway_y, runway_angle):
        self.aircraft_limit = aircraft_limit
        self.runway_x = runway_x
        self.runway_y = runway_y
        self.runway_angle = runway_angle
        self.runway_upper_x = runway_x + DEFAULT_RUNWAY_LENGTH/2 * math.sin(runway_angle)
        self.runway_upper_y = runway_y + DEFAULT_RUNWAY_LENGTH/2 * math.cos(runway_angle)
        self.runway_lower_x = runway_x - DEFAULT_RUNWAY_LENGTH/2 * math.sin(runway_angle)
        self.runway_lower_y = runway_y - DEFAULT_RUNWAY_LENGTH/2 * math.cos(runway_angle)
        self.complimentary_rwy_angle = runway_angle + math.pi
    
    def land_aircraft(self, aircraft):        
        self.grounded_aircraft.append(aircraft)

    def takeoff_aircraft(self, aircraft):
        if self.takeoff_on_upper:
            aircraft.x = self.runway_upper_x
            aircraft.y = self.runway_upper_y
        else:
            aircraft.x = self.runway_lower_x
            aircraft.y = self.runway_lower_y

        self.grounded_aircraft.remove(aircraft)

    def distance_to_touchdown(self, aircraft):
        #Takeoff on upper, landing on lower
        if self.takeoff_on_upper:
            touchdown_x = self.runway_lower_x
            touchdown_y = self.runway_lower_y
        else:
            touchdown_x = self.runway_upper_x
            touchdown_y = self.runway_lower_y
        
        touchdown_z = 0 #Quasi-Netherlands hypothesis - Maybe something more on the future?
    
        distance = math.sqrt((aircraft.x - touchdown_x)^2 + (aircraft.y - touchdown_y)^2 + (aircraft.z - touchdown_z)^2)

        return distance