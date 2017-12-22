# TODO Notes
# - [x] clip the throttle
# - [x] if error begins increasing, stop the goto
# - [x] ensure throttle is zero during tip and steer is zero during goto
# - [x] try applying brakes for tip
import numpy as np
from astar import astar, euclidean_heuristic
from pid import Pid

# Returns False when done, else True
class Tip():
    # class variables
    target_deg = 0
    spin = ""
    started = False
    did_start_brake = False
    brake_counter = 0
    total_deg = 0

    Pid = None
    kp = 0.3
    ki = 0.0
    kd = 0.0

    def start(self, Rover, target_deg, spin):
        Tip.target_deg = target_deg
        Tip.total_deg = abs(Rover.yaw - target_deg)
        Tip.spin = spin
        Tip.started = True
        Tip.did_start_brake = False
        Tip.Pid = Pid(Tip.kp, Tip.ki, Tip.kd, target_deg)
        Rover.throttle = 0
        # Rover.steer = 15 if Tip.spin == 'ccw' else -15
        print("Set rover steer to %d" % Rover.steer)

    def update(self, Rover):
        TIP_TOLERANCE = 0.01
        if not Tip.did_start_brake:
            # If Rover is not stopped
            # if Rover.vel <= 0 and Tip.brake_counter >= 10:
            if Rover.vel <= 0:
                Tip.did_start_brake = True
                Rover.brake = 0
            else:
                Rover.brake = Rover.brake_set
        elif Tip.spin == 'cw':
            print("Rover steer=%f y=%f" % (Rover.steer, Rover.yaw))
            # if Rover.yaw < Tip.target_deg or abs(Rover.yaw - Tip.target_deg) < TIP_TOLERANCE:
            if abs(Rover.yaw - Tip.target_deg) < TIP_TOLERANCE:
                Rover.steer = 0
                print("Tip done %f < %f" % (Rover.yaw, Tip.target_deg))
                Tip.started = False
                return False
            else:
                Rover.steer = -Tip.Pid.update(Rover.yaw)
                Rover.throttle = 0
                Rover.brake = 0
        elif Tip.spin == 'ccw':
            print("Rover steer=%f y=%f" % (Rover.steer, Rover.yaw))
            # if Rover.yaw > Tip.target_deg or abs(Rover.yaw - Tip.target_deg) < TIP_TOLERANCE:
            if abs(Rover.yaw - Tip.target_deg) < TIP_TOLERANCE:
                Rover.steer = 0
                print("Tip done %f > %f" % (Rover.yaw, Tip.target_deg))
                Tip.started = False
                return False
            else:
                Rover.steer = Tip.Pid.update(Rover.yaw)
                Rover.throttle = 0
                Rover.brake = 0
        return True

def clamp_throttle(throttle):
    if throttle < -1.0:  return -1.0
    elif throttle > 1.0: return 1.0
    else:                return throttle

class Goto():
    # constants
    FAST_SPEED = 0.8
    DEFAULT_SPEED = 0.3
    SLOWDOWN_DIST = 2
    FAST_DIST = 8
    # TERMINATE_DIST = 0.05
    # TD 0.05 kp 0.01
    # TERMINATE_DIST = 0.2
    TERMINATE_DIST = 0.05
    SLOWDOWN_FACTOR = 0.5

    # class variables
    target_pt = 0
    start_pt = 0
    total_dist = 0
    last_dist = 0
    started = False
    quit_counter = 0

    Pid = None
    kp = 0.01
    ki = 0.0
    kd = 0.0

    # def euclidean_dist(start, end):
    #     return np.linalg.norm(np.array(end) - np.array(start))

    def start(self, Rover, target_pt):
        Goto.target_pt = target_pt
        Goto.start_pt = Rover.pos
        Goto.total_dist = euclidean_heuristic(Goto.start_pt, Goto.target_pt)
        Goto.last_dist = 0
        Goto.started = True
        Goto.quit_counter = 0
        Goto.Pid = Pid(Goto.kp, Goto.ki, Goto.kd, 0.0)
        Rover.steer = 0

    def update(self, Rover):
        DIST_ERROR_LIMIT = 3
        # cur_dist = np.linalg.norm(tuple(Goto.target_pt) - tuple(Rover.pos))
        cur_dist = euclidean_heuristic(Rover.pos, Goto.target_pt)
        if cur_dist <= Goto.TERMINATE_DIST:
            print("Goto end w/ cur_dist: %f and pos %f,%f" % (cur_dist, Rover.pos[0], Rover.pos[1]))
            Rover.throttle = 0
            Goto.started = False
            return False
        elif cur_dist > Goto.last_dist:
            Goto.quit_counter += 1
            print("Goto quit cnt:%d" % Goto.quit_counter)
            if Goto.quit_counter >= DIST_ERROR_LIMIT:
                print("Goto end b/c >>error w/ cur_dist: %f and pos %f,%f" % (cur_dist, Rover.pos[0], Rover.pos[1]))
                Rover.throttle = 0
                Goto.started = False
                return False
        else:
            # print("Goto: cur_dist: %f throttle: %f cell %d,%d" % (cur_dist, Rover.throttle, int(Rover.pos[0]), int(Rover.pos[1])))
            print("Goto: cur_dist: %f err %f throttle: %f" % (cur_dist, 0.01 - cur_dist, Rover.throttle))
            # Rover.throttle = clamp_throttle(Goto.Pid.update(-cur_dist))
            # Negate the PID output since the control output is negative
            Rover.throttle = -clamp_throttle(Goto.Pid.update(cur_dist))
            Goto.quit_counter = 0
        Goto.last_dist = cur_dist
        # elif cur_dist <= Goto.SLOWDOWN_DIST:
        #     Rover.throttle = 1.0 * (cur_dist / Goto.total_dist) * Goto.SLOWDOWN_FACTOR
        #     print("Goto: cur_dist: %f throttle: %f cell %d,%d" % (cur_dist, Rover.throttle, int(Rover.pos[0]), int(Rover.pos[1])))
        # elif cur_dist >= Goto.FAST_DIST:
        #     Rover.throttle = Goto.FAST_SPEED
        # else:
        #     Rover.throttle = Goto.DEFAULT_SPEED
        return True

# Restrict angle to range [0,360]
def clamp_angle(degrees):
    return degrees + 360 if degrees < 0 else degrees

def point_to_subcmd(Rover, end_pt):
    SAME_ANGLE_TOLERANCE = 0.5
    if int(Rover.pos[0]) == end_pt[0] and int(Rover.pos[1]) == end_pt[1]:
        print("Skip wp %d,%d already reached" % (end_pt[0], end_pt[1]))
        return
    # At the end, we'll always need to goto the point, so push it on the stack
    subcmd = {"cmd": "goto", "target": end_pt}
    print("Adding %d,%d goto to queue" % (end_pt[0], end_pt[1]))
    Rover.subcmds.append(subcmd)
    # Start with starting pt
    start_pt = Rover.pos
    end_pt_angle = np.degrees(np.arctan2((end_pt[1] - start_pt[1]), (end_pt[0] - start_pt[0])))
    print("%f,%f --> %f,%f orig_epa=%f" % (start_pt[0], start_pt[1], end_pt[0], end_pt[1], end_pt_angle))
    end_pt_angle = clamp_angle(end_pt_angle)
    angle_diff = end_pt_angle - Rover.yaw
    print("Angle diff %f; yaw %f epa %f" % (angle_diff, Rover.yaw, end_pt_angle))
    if abs(angle_diff) > SAME_ANGLE_TOLERANCE:
        # Target point is not in line with where the Rover is pointing. Do a tip
        # before doing the goto subcommand
        # spin = "cw" if angle_diff > 0 else "ccw" # determine spin based on sign
        # Compute which spin direction is closest to target heading
        if angle_diff < (360 - end_pt_angle + Rover.yaw):
            spin = "ccw" 
        else:
            spin = "cw"
        subcmd = {"cmd": "tip",  "target": end_pt_angle, "spin": spin}
        print("Adding %f %s tip to queue" % (end_pt_angle, spin))
        Rover.subcmds.append(subcmd)

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if decision_step.subcmd == None:
        if Rover.subcmds:
            ### CURRENT SUBCMD ###
            # If there's no subcmd, pop one from the subcmd queue
            decision_step.subcmd = Rover.subcmds.pop()
            print("Popping %s cmd with target " % decision_step.subcmd["cmd"], end='')
            print(decision_step.subcmd["target"])
        else:
            ### SUBCMDS FROM WAYPOINTS ###
            # Plan subcmd steps with the next point from A* planner
            if len(decision_step.waypoints) > 0:
                dest_point = decision_step.waypoints.pop(0)
                print("Popping waypoint", end='')
                print(dest_point)
                print(Rover.pos)
                point_to_subcmd(Rover, dest_point)
            # If there are no waypoints, plan a path to the target with A*
            elif Rover.target:
                ### A* PATH PLANNING ###
                cur_cell = [int(Rover.pos[0]), int(Rover.pos[1])]
                dest_cell = [int(Rover.target[0]), int(Rover.target[1])]
                decision_step.waypoints = astar(Rover.bitmap, cur_cell, dest_cell, 1)
                print("A* wp:")
                print(decision_step.waypoints)
    elif decision_step.subcmd["cmd"] == "tip":
        if not Tip().started:
            print("Starting %s tip with target %f" % (decision_step.subcmd["spin"], decision_step.subcmd["target"]))
            Tip().start(Rover, decision_step.subcmd["target"], decision_step.subcmd["spin"])
        elif not Tip().update(Rover):
            print("Done with %s tip to %f" % (decision_step.subcmd["spin"], decision_step.subcmd["target"]))
            decision_step.subcmd = None
    elif decision_step.subcmd["cmd"] == "goto":
        if not Goto().started:
            Goto().start(Rover, decision_step.subcmd["target"])
        elif not Goto().update(Rover):
            decision_step.subcmd = None

    '''
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
'''
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    #if True:
    #    # Do nothing
    # Rover.throttle = 0
    # Rover.steer = 0
    # Rover.brake = 0
    #    return

    return Rover
decision_step.subcmd = None
decision_step.waypoints = []
