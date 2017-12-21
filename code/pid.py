class Pid():
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, set_point = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.set_point = set_point

        self.integrated_error = 0.0
        self.last_error = 0.0

    def update(self, measurement):
        # Calculate proportional error
        error = self.set_point - measurement
        if (self.set_point < 0.5):
            print("pid error %f" % error)
        # Calculate integral error
        self.integrated_error += error
        # Calculate derivative
        derivative_error = error - self.last_error
        self.last_error = error

        u = (self.kp * error) + (self.ki * self.integrated_error) + self.kd * (derivative_error)
        return u
