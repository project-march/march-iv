

AMOUNT_OF_DECIMALS = 4


class Setpoint(object):
    """Base class to define the setpoints of a subgait"""
    def __init__(self, time, position, velocity):
        self.time = round(time, AMOUNT_OF_DECIMALS)
        self.position = round(position, AMOUNT_OF_DECIMALS)
        self.velocity = round(velocity, AMOUNT_OF_DECIMALS)

    def __repr__(self):
        return 'Time: %s, Position: %s, Velocity: %s' % (self.time, self.position, self.velocity)
