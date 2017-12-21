class Pid():
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, set_point = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.set_point = set_point

        self.error_sum = 0.0
        self.last_error = 0.0

    def update(self, measurement):
        # Calculate proportional error
        error = self.set_point - measurement
        # Calculate integral error
        self.error_sum += error
        # Calculate derivative
        delta_error = error - self.last_error
        self.last_error = error

        u = (self.kp * error) + (self.ki * self.error_sum) + self.kd * (delta_error)
        return u
