# TODO Notes
# - [x] clip the throttle
# - [x] if error begins increasing, stop the goto
# - [x] ensure throttle is zero during tip and steer is zero during goto
# - [x] try applying brakes for tip
# - [x] debug A* strange pathing
# - [x] apply brakes when Goto gets to its target
# - [x] tune pid
# - [x] integrate wall follow behavior
# - [ ] get stuck mode working
# - [ ] add pickup rock routine
import numpy as np
import matplotlib.pyplot as plt
from astar import astar, euclidean_heuristic
from pid import Pid
import time


# Returns False when done, else True
class Tip():
    # class variables
    target_deg = 0
    spin = ""
    started = False
    did_start_brake = False
    start_yaw = 0
    brake_counter = 0
    total_deg = 0

    Pid = None
    kp = 0.35
    ki = 0.0
    kd = 0.6
    # p    i    d
    #(0.4, 0.0, 0.2) is acceptable speed, but sometimes blows up
    #(0.35, 0.0, 0.2) is acceptable speed, but sometimes blows up
    #(0.35, 0.0, 0.4) increases the speed of convergence
    #(0.35, 0.0, 0.8) once went crazy between 0, 360

    def start(self, Rover, target_deg, spin):
        Tip.target_deg = target_deg
        Tip.total_deg = abs(Rover.yaw - target_deg)
        Tip.spin = spin
        Tip.started = True
        Tip.did_start_brake = False
        Tip.start_yaw = Rover.yaw
        Tip.Pid = Pid(Tip.kp, Tip.ki, Tip.kd, target_deg)
        Rover.throttle = 0
        print("Set rover steer to %d" % Rover.steer)

    def update(self, Rover):
        # TIP_TOLERANCE = 0.01 (too fine)
        # TIP_TOLERANCE = 0.1 (good, but still takes time to converge)
        TIP_TOLERANCE = 0.5
        if not Tip.did_start_brake:
            # If Rover is not stopped
            # if Rover.vel <= 0 and Tip.brake_counter >= 10:
            if Rover.vel <= 0:
                print("Tip skipping starting brake")
                Tip.did_start_brake = True
                Rover.brake = 0
            else:
                print("Tip starting brake")
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
        elif Tip.spin == 'cw':
            print("Rover steer=%f y=%f" % (Rover.steer, Rover.yaw))
            # if Rover.yaw < Tip.target_deg or abs(Rover.yaw - Tip.target_deg) < TIP_TOLERANCE:
            # cw_angle_exceeded = ((Tip.target_deg <= Tip.start_yaw) and (Rover.yaw < Tip.target_deg)) or ((Tip.target_deg > Tip.start_yaw) and (Rover.yaw > Tip.start_yaw) and (Rover.yaw < Tip.target_deg))
            cw_angle_exceeded = False
            if abs(Rover.yaw - Tip.target_deg) < TIP_TOLERANCE or cw_angle_exceeded:
                Rover.steer = 0
                print("Tip done %f < %f exc=%d" % (Rover.yaw, Tip.target_deg, cw_angle_exceeded))
                Tip.started = False
                return False
            else:
                Rover.steer = -Tip.Pid.update(Rover.yaw)
                Rover.throttle = 0
                Rover.brake = 0
        elif Tip.spin == 'ccw':
            print("Rover steer=%f y=%f" % (Rover.steer, Rover.yaw))
            # if Rover.yaw > Tip.target_deg or abs(Rover.yaw - Tip.target_deg) < TIP_TOLERANCE:
            # ccw_angle_exceeded = ((Tip.target_deg >= Tip.start_yaw) and (Rover.yaw > Tip.target_deg)) or ((Tip.target_deg < Tip.start_yaw) and (Rover.yaw < Tip.start_yaw) and (Rover.yaw > Tip.target_deg))
            ccw_angle_exceeded = False
            if abs(Rover.yaw - Tip.target_deg) < TIP_TOLERANCE or ccw_angle_exceeded:
                Rover.steer = 0
                print("Tip done %f > %f exc=%d" % (Rover.yaw, Tip.target_deg, ccw_angle_exceeded))
                Tip.started = False
                return False
            else:
                Rover.steer = Tip.Pid.update(Rover.yaw)
                Rover.throttle = 0
                Rover.brake = 0
        return True

# def clamp_throttle(throttle):
#     if throttle < -1.0:  return -1.0
#     elif throttle > 1.0: return 1.0
#     else:                return throttle

class Goto():
    # constants
    FAST_SPEED = 0.8
    DEFAULT_SPEED = 0.3
    SLOWDOWN_DIST = 2
    FAST_DIST = 8
    # TERMINATE_DIST = 0.05
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
    do_finishing_brake = False

    Pid = None
    # p    i d
    # 0.01 0 0 (very slow)
    # 0.2  0 0 (acceptable)
    # 0.4  0 0 (appears to do no harm, but may want to be more careful)
    # 0.1  0 0 (slow and deliberate)
    kp = 0.4
    ki = 0.0
    kd = 0.2

    # def euclidean_dist(start, end):
    #     return np.linalg.norm(np.array(end) - np.array(start))

    def start(self, Rover, target_pt):
        Goto.target_pt = target_pt
        Goto.start_pt = Rover.pos
        Goto.total_dist = euclidean_heuristic(Goto.start_pt, Goto.target_pt)
        Goto.last_dist = 0
        Goto.started = True
        Goto.quit_counter = 0
        Goto.do_finishing_brake = False
        Goto.Pid = Pid(Goto.kp, Goto.ki, Goto.kd, 0.0)
        Rover.steer = 0

    def update(self, Rover):
        DIST_ERROR_LIMIT = 3
        # cur_dist = np.linalg.norm(tuple(Goto.target_pt) - tuple(Rover.pos))
        cur_dist = euclidean_heuristic(Rover.pos, Goto.target_pt)
        if Goto.do_finishing_brake:
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            if Rover.vel <= 0:
                Goto.do_finishing_brake = False
                Goto.started = False
                Rover.brake = 0
                print("Goto brake done")
                return False
        elif cur_dist <= Goto.TERMINATE_DIST:
            print("Goto end w/ cur_dist: %f and pos %f,%f" % (cur_dist, Rover.pos[0], Rover.pos[1]))
            Goto.do_finishing_brake = True
            # Rover.throttle = 0
            # Goto.started = False
            # return False
        elif cur_dist > Goto.last_dist:
            Goto.quit_counter += 1
            print("Goto quit cnt:%d" % Goto.quit_counter)
            if Goto.quit_counter >= DIST_ERROR_LIMIT:
                print("Goto end b/c >>error w/ cur_dist: %f and pos %f,%f" % (cur_dist, Rover.pos[0], Rover.pos[1]))
                Goto.do_finishing_brake = True
                # Rover.throttle = 0
                # Goto.started = False
                # return False
        else:
            # print("Goto: cur_dist: %f throttle: %f cell %d,%d" % (cur_dist, Rover.throttle, int(Rover.pos[0]), int(Rover.pos[1])))
            print("Goto: cur_dist: %f err %f throttle: %f" % (cur_dist, 0.01 - cur_dist, Rover.throttle))
            # Rover.throttle = clamp_throttle(Goto.Pid.update(-cur_dist))
            # Negate the PID output since the control output is negative
            # Rover.throttle = -clamp_throttle(Goto.Pid.update(cur_dist))
            Rover.throttle = -np.clip(Goto.Pid.update(cur_dist), -1.0, 1.0)
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
    if degrees < 0:     return degrees + 360
    elif degrees > 360: return degrees - 360
    else:               return degrees

def point_to_subcmd(Rover, end_pt):
    SAME_ANGLE_TOLERANCE = 1 #prev 0.5
    if int(Rover.pos[0]) == end_pt[0] and int(Rover.pos[1]) == end_pt[1]:
        print("Skip wp %d,%d already reached" % (end_pt[0], end_pt[1]))
        return False
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
    # Check for stuck condition
    STUCK_DETECTION_TIME = 5 #seconds
    if Rover.mode != "stuck":
        if Rover.throttle > 0 and abs(Rover.steer) > 3 and Rover.vel <= 0.15:
            print("Stuck maybe thr=%f steer=%f vel=%f" % (Rover.throttle, abs(Rover.steer), Rover.vel))
            if not decision_step.stuck_timestamp:
                print("Stuck timeout started")
                decision_step.stuck_timestamp = time.time()
                decision_step.stuck_pos = Rover.pos
            elif ((time.time() - decision_step.stuck_timestamp) > STUCK_DETECTION_TIME and (int(Rover.pos[0]) == int(decision_step.stuck_pos[0]) or
                int(Rover.pos[1]) == int(decision_step.stuck_pos[1]))):
                print("Stuck mode activation")
                Rover.mode = "stuck"
        elif decision_step.stuck_timestamp and Rover.vel >= 0.2:
            print("Stuck timestamp reset")
            decision_step.stuck_timestamp = None

    # Check if we have vision data to make decisions with
    if decision_step.subcmd and decision_step.subcmd["cmd"] == "tip":
        if not Tip().started:
            print("Starting %s tip with target %f" % (decision_step.subcmd["spin"], decision_step.subcmd["target"]))
            Tip().start(Rover, decision_step.subcmd["target"], decision_step.subcmd["spin"])
        elif not Tip().update(Rover):
            print("Done with %s tip to %f" % (decision_step.subcmd["spin"], decision_step.subcmd["target"]))
            decision_step.subcmd = None
    elif decision_step.subcmd and decision_step.subcmd["cmd"] == "goto":
        if not Goto().started:
            Goto().start(Rover, decision_step.subcmd["target"])
        elif not Goto().update(Rover):
            decision_step.subcmd = None
    elif decision_step.subcmd == None and Rover.subcmds:
        ### CURRENT SUBCMD ###
        # If there's no subcmd, pop one from the subcmd queue
        decision_step.subcmd = Rover.subcmds.pop()
        print("Popping %s cmd with target " % decision_step.subcmd["cmd"], end='')
        print(decision_step.subcmd["target"])
    elif Rover.target:
        ### SUBCMDS FROM WAYPOINTS ###
        # Plan subcmd steps with the next point from A* planner
        if len(decision_step.waypoints) > 0:
            dest_point = decision_step.waypoints.pop()
            print("Popping waypoint", end='')
            print(dest_point)
            print(Rover.pos)
            success = point_to_subcmd(Rover, dest_point)
            if not success and len(decision_step.waypoints) <= 1:
                print("Unset target b/c already at waypoint")
                Rover.target = None
        # If there are no waypoints, plan a path to the target with A*
        # if there is no target, then unset it
        elif Rover.target:
            ### A* PATH PLANNING ###
            cur_cell = [int(Rover.pos[0]), int(Rover.pos[1])]
            dest_cell = [int(Rover.target[0]), int(Rover.target[1])]
            # print("No wp, cur cell and dest cell are ", end='')
            # print(cur_cell)
            # print(dest_cell)
            if cur_cell[0] == dest_cell[0] or cur_cell[1] == dest_cell[1]:
                print("Unset target, already at %d,%d" % (cur_cell[0],cur_cell[1]))
                Rover.target = None
            else:
                decision_step.waypoints = astar(Rover.bitmap, cur_cell, dest_cell, 1)
                print("A* wp:")
                print(decision_step.waypoints)
                plt.imshow(Rover.bitmap.T, origin='lower')
                plt.show()
    # Do non-map-based behaviors
    else:
        message = ""
        # Check if we have vision data to make decisions with
        if Rover.nav_angles is not None:
            # Check for Rover.mode status
            if Rover.mode == 'forward':
                message += "fwd mode\n"
                # Check the extent of navigable terrain
                if len(Rover.nav_angles) >= Rover.stop_forward:
                    # If mode is forward, navigable terrain looks good
                    # and velocity is below max, then throttle
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15
                    # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    # wf_offset = 15
                    # OBS_CONE_DEG = 20
                    # OBS_CONE_DEG = 0.22
                    OBS_CONE_DEG = 0.22
                    OBS_CONE_DIST = 129
                    # OBS_WF_OFFSET = -15
                    OBS_WF_OFFSET = -90
                    mean_obs_angle = np.mean(Rover.obs_angles * 180/np.pi)
                    if (mean_obs_angle < OBS_CONE_DEG or mean_obs_angle > 360 - OBS_CONE_DEG) and np.mean(Rover.obs_dists) < OBS_CONE_DIST:
                        obsdet_offset = OBS_WF_OFFSET
                        # print("obsdet mean %f (%f,%f) and dist %f (%f,%f)" % (mean_obs_angle, min(Rover.obs_angles) * 180/np.pi, max(Rover.obs_angles) * 180/np.pi, np.mean(Rover.obs_dists), min(Rover.obs_dists), max(Rover.obs_dists)))
                        print("obsdet mean %f and dist %f (%f,%f), adding %f" % (mean_obs_angle, np.mean(Rover.obs_dists), min(Rover.obs_dists), max(Rover.obs_dists), obsdet_offset))
                    else:
                        obsdet_offset = 0
                    wf_offset = 12 + obsdet_offset
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi) + wf_offset, -15, 15)
                # If there's a lack of navigable terrain pixels then go to 'stop' mode
                elif len(Rover.nav_angles) < Rover.stop_forward:
                    message += "Switch to stop mode\n"
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

            # If we're already in "stop" mode then make different decisions
            elif Rover.mode == 'stop':
                message += "stop mode\n"
                # If we're in stop mode but still moving keep braking
                if Rover.vel > 0.2:
                    message += "keep braking\n"
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                # If we're not moving (vel < 0.2) then do something else
                elif Rover.vel <= 0.2:
                    # Now we're stopped and we have vision data to see if there's a path forward
                    if len(Rover.nav_angles) < Rover.go_forward:
                        message += "do tip\n"
                        Rover.throttle = 0
                        # Release the brake to allow turning
                        Rover.brake = 0
                        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                        Rover.steer = -15 # Could be more clever here about which way to turn
                    # If we're stopped but see sufficient navigable terrain in front then go!
                    if len(Rover.nav_angles) >= Rover.go_forward:
                        # Set throttle back to stored value
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                        Rover.mode = 'forward'
                        message += "switch to fwd mode\n"
            elif Rover.mode == "stuck":
                # STUCK_TURN_DEG = -45 #cw
                STUCK_TURN_DEG = 45 #ccw
                # stuck_target_yaw
                # stuck_start_yaw
                if decision_step.stuck_target_yaw == None:
                    decision_step.stuck_start_yaw = Rover.yaw
                    decision_step.stuck_target_yaw = clamp_angle(Rover.yaw + STUCK_TURN_DEG)
                    Rover.brake = Rover.brake_set
                else:
                    message += "stuck mode {0}-->{1}  start:{2}\n".format(Rover.yaw, decision_step.stuck_target_yaw, decision_step.stuck_start_yaw)
                    # Rover.steer = -15 #cw
                    Rover.steer = 15 #ccw
                    Rover.throttle = 0
                    Rover.brake = 0
                    # simple_exit = (decision_step.stuck_target_yaw <= decision_step.stuck_start_yaw and Rover.yaw < decision_step.stuck_target_yaw) #cw
                    simple_exit = (decision_step.stuck_target_yaw >= decision_step.stuck_start_yaw and Rover.yaw > decision_step.stuck_target_yaw) #ccw
                    # complex_exit = (decision_step.stuck_target_yaw > decision_step.stuck_start_yaw and Rover.yaw > decision_step.stuck_start_yaw and Rover.yaw < decision_step.stuck_target_yaw) #ccw
                    complex_exit = (decision_step.stuck_target_yaw < decision_step.stuck_start_yaw and Rover.yaw < decision_step.stuck_start_yaw and Rover.yaw > decision_step.stuck_target_yaw) #cw
                    if (simple_exit or complex_exit):
                        Rover.throttle = 0
                        Rover.steer = 0
                        Rover.brake = Rover.brake_set
                        decision_step.stuck_target_yaw = None
                        message += "stuck mode end start:{0}, targ:{1} yaw:{2}".format(decision_step.stuck_start_yaw, decision_step.stuck_target_yaw, Rover.yaw)
                        decision_step.stuck_timestamp = None
                        Rover.mode = "forward"
                # bwd_escape_deg = Rover.yaw - 45
                # bwd_escape_deg = clamp_angle(bwd_escape_deg)
                # tip_subcmd = {"cmd": "tip",  "target": bwd_escape_deg, "spin": "cw"}
                # decision_step.subcmd = tip_subcmd
                # decision_step.stuck_timestamp = None
                # Rover.mode = 'forward'
                '''
                # Go backward for a few seconds
                Rover.throttle = -1.0
                if not decision_step.stuck_go_bwd:
                    decision_step.stuck_bwd_timestamp = time.time()
                    decision_step.stuck_go_bwd = True
                # Tip 45 degrees cw and check stuck mode exit condition
                elif (time.time() - decision_step.stuck_bwd_timestamp) > 2:
                    message += "stuck backup done, do tip\n"
                    decision_step.stuck_go_bwd = False
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    bwd_escape_deg = Rover.yaw - 45
                    bwd_escape_deg = clamp_angle(bwd_escape_deg)
                    tip_subcmd = {"cmd": "tip",  "target": bwd_escape_deg, "spin": "cw"}
                    decision_step.subcmd = tip_subcmd
                    if abs(Rover.vel > 0.2):
                        message += "switch from stuck to forward\n"
                        decision_step.stuck_timestamp = None
                        Rover.mode == 'forward'
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                '''
            # Just to make the rover do something
            # even if no modifications have been made to the code
            else:
                message += "Nike: just do it\n"
                Rover.throttle = Rover.throttle_set
                Rover.steer = 0
                Rover.brake = 0

        # If in a state where want to pickup a rock send pickup command
        if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
            Rover.send_pickup = True

        if decision_step.last_message != message:
            print(message)
            decision_step.last_message = message

    return Rover
decision_step.subcmd = None
decision_step.stuck_pos = None
decision_step.stuck_start_yaw = None
decision_step.stuck_target_yaw = None
decision_step.stuck_timestamp = None
decision_step.stuck_go_bwd = False
decision_step.stuck_bwd_timestamp = None
decision_step.waypoints = []
decision_step.last_message = ""
