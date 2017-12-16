import numpy as np

# Returns False when done, else True
class Tip():
    # class variables
    target_deg = 0
    spin = ""
    started = False

    def start(self, Rover, target_deg, spin):
        Tip.target_deg = target_deg
        Tip.spin = spin
        Tip.started = True
        Rover.steer_angle = 15 if Tip.spin == 'ccw' else -15

    def update(self):
        if Tip.spin == 'cw':
            if Rover.yaw < Tip.target_deg:
                print("Tip done %f < %f" % (Rover.yaw, Tip.target_deg))
                Tip.started = False
                Rover.steer_angle = 0
                return False
        elif Tip.spin == 'ccw'
            if Rover.yaw > Tip.target_deg:
                print("Tip done %f > %f" % (Rover.yaw, Tip.target_deg))
                Tip.started = False
                Rover.steer_angle = 0
                return False
        return True

    def pending(self):
        return Tip.started

class Goto():
    # constants
    FAST_SPEED = 0.8
    DEFAULT_SPEED = 0.3
    SLOWDOWN_DIST = 2
    FASTER_DIST = 8
    TERMINATE_DIST = 0.01
    SLOWDOWN_FACTOR = 0.5

    # class variables
    target_pt = 0
    start_pt = 0
    total_dist = 0
    last_dist = 0
    started = False

    def start(self, Rover, target_pt):
        Goto.target_pt = target_pt
        Goto.start_pt = Rover.pos
        Goto.total_dist = np.linalg.norm(Goto.target_pt - Goto.start_pt)
        Goto.last_dist = 0
        Goto.started = True

    def update(self, Rover):
        cur_dist = np.linalg.norm(Goto.target_pt - Rover.pos)
        if cur_dist <= Goto.TERMINATE_POINT:
            print("Goto end w/ cur_dist: %f" % cur_dist)
            Rover.throttle = 0
            Goto.started = False
            return False
        elif cur_dist <= Goto.SLOWDOWN_DIST:
            Rover.throttle = 1.0 * (cur_dist / Goto.total_dist) * Goto.SLOWDOWN_FACTOR
            print("Goto: cur_dist: %f throttle: %f" % (cur_dist, Rover.throttle))
        elif cur_dist >= Goto.FAST_DIST:
            Rover.throttle = Goto.FAST_SPEED
        else:
            Rover.throttle = Goto.DEFAULT_SPEED
        return True

def point_to_subcmd(Rover, end_pt):
    SAME_ANGLE_TOLERANCE = 0.5
    # At the end, we'll always need to goto the point, so push it on the stack
    subcmd = {"cmd": "goto", "target": end_pt}
    print("Adding %d,%d goto to queue" % (end_pt[0], end_pt[1]))
    Rover.subcmd_queue.append(subcmd)
    # Start with starting pt
    start_pt = Rover.pos
    end_pt_angle = np.arctan2((end_pt[1] - start_pt[1]) / (end_pt[0] - start_pt[0]))
    angle_diff = Rover.yaw - end_pt_angle
    if angle_diff > SAME_ANGLE_TOLERANCE:
        # Target point is not in line with where the Rover is pointing. Do a tip
        # before doing the goto subcommand
        turn_angle = angle_diff
        spin = "cw" if turn_angle > 0 else "ccw"
        subcmd = {"cmd": "tip",  "target": angle_diff, "spin": spin}
        print("Adding %d %s tip to queue" % (angle_diff, spin))
        Rover.subcmd_queue.append(subcmd)

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if decision_step.submode = None:
        if Rover.subcmd_queue:
            # If there's no subcmd, pop one from the subcmd queue
            cur_subcmd = Rover.subcmd_queue.pop()
            decision_step.submode = cur_subcmd["cmd"]
        else:
            # Plan subcmd steps with the next point from A* planner

    elif decision_step.submode = "tip":
        if not Tip().started():
            Tip().start(Rover, cur_subcmd["target"], cur_subcmd["spin"])
        elif not Tip().update():
            decision_step.submode = None
    elif decision_step.submode = "goto":
        if not Goto().started():
            Goto().start(Rover, cur_subcmd["target"])
        elif not Goto().update():
            decision_step.submode = None

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
decision_step.submode = ""
