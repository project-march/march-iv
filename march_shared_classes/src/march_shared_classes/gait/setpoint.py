class Setpoint:
    digits = 4

    def __init__(self, time, position, velocity):
        self.time = round(time, self.digits)
        self.position = round(position, self.digits)
        self.velocity = round(velocity, self.digits)

    def set_time(self, time):
        self.time = round(time, self.digits)

    def set_position(self, position):
        self.position = round(position, self.digits)

    def set_velocity(self, velocity):
        self.velocity = round(velocity, self.digits)

    def __repr__(self):
        return 'Time: %s, Position: %s, Velocity: %s' % (self.time, self.position, self.velocity)
