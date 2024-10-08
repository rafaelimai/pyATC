
class Airspace:
    """ 
    Although airports are not technically on the airspace, I'll place them inside the class for the sake of simplicity.

    Let's simplify the airspace, assuiming the ground beneath it is at sea-level and is uniform (quasi-Netherlands hypothesis)
    """
    def __init__(self, width, length, height):
        self.width = width
        self.height = height
        self.length = length
        self.airport_list = []

    

