import numpy as np

# Returns False when done, else true
class Tip():

    def __init__(self, Rover, target_deg, spin):
        self.Rover = Rover
        self.target_deg = target_deg
        self.spin = spin
        Rover.steer_angle = 15 if self.spin == 'ccw' else -15

    def update(self):
        if self.spin == 'cw':
            if self.Rover.yaw < self.target_deg:
                return False
        elif self.spin == 'ccw'
            if self.Rover.yaw > self.target_deg:
                return False
        return True

class Goto():
    def __init__(self, Rover, target_point, slow_factor):
        self.Rover = Rover
        self.start_point = Rover.pos
        self.target_point = target_point
        self.slow_factor = slow_factor

    def update(self):
        Rover.throttle = 1.0 * ()

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    decision_step.Tip = Tip(Rover, degrees, 'cw')
    while Tip.update():
        pass

    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    #print("Throttle is %d" % Rover.throttle_set)
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    #print("Coasting")
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                wf_offset = 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi) + wf_offset, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    #print("Hit the brakes")
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.debug = "Brake from fwd"
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        elif Rover.mode == "goto_rock":
            print("Goto rock")
            if Rover.nav_rock_angles is not None:
                Rover.steer = np.clip(np.mean(Rover.nav_rock_angles * 180/np.pi), -15, 15)
                approach_scale = 1
                Rover.throttle = np.mean(Rover.nav_rock_dists)
                #print(Rover.throttle)
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.debug = "Braking from stop; vel " + str(Rover.vel)
                #print("Braking from stop v=%d" % Rover.vel)
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                Rover.debug = "Len is " + str(len(Rover.nav_angles))
                #print("Stop case 2 v=%d" % Rover.vel)
                if len(Rover.nav_angles) < Rover.go_forward:
                    # if Rover.throttle == 0:
                    Rover.debug = "Setting 1/4 throttle"
                    Rover.throttle = Rover.throttle_set / 4
                    # else:
                        # Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    #print("Stop case 2 v=%d" % Rover.vel)
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                    Rover.debug = "Fwd, len is " + str(len(Rover.nav_angles))
        elif Rover.mode == "get_unstuck":
            print("Get unstuck")

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    #if True:
    #    # Do nothing
    Rover.throttle = 0
    Rover.steer = 0
    Rover.brake = 0
    #    return

    return Rover
