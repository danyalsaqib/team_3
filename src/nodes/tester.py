#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal

import numpy as np
#import rospy
#from std_msgs.msg import String
#import geometry_msgs.msg
#import json

from potential_field_planner import PotentialFieldPlanner


# Initialize subscriber
#map_sub = rospy.Subscriber('/map', String, map_callback)
#map = None

# Intialize publisher
#cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
#cmd = None
#rate = rospy.Rate(10)  # Publisher frequency

initial_turn = False
right_push = False

time_step = 1/10
k_att     = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.25]])
k_rep     = 1
vel_max   = 0.5
# TODO BEGIN MRSS: Add attributes (If needed)
ini_val = [0, 0, 0]
# END MRSS

while True:
    print("Entering Callback")
    #map = json.loads(msg.data)
    #map = {"/goal" : [1, 2, 0]}
    map = {"/goal" : [1, 2, 0], "/obstacle1" : [0.5, 0.5, 0], "/obstacle2" : [0.5, 4, 0]}
    #obstacle_dict = [key for key in map.keys() if 'obstacle' in key.lower()]
    obstacle_dict = [key for key in map.keys() if 'obstacle' in key.lower()]
    # TODO BEGIN MRSS: Use map for planning
    goals = np.array(map["/goal"])

    norm = np.linalg.norm(goals)

    print("obs_dict: ", obstacle_dict)

    if not obstacle_dict:
        obs_bool = False
    else:
        obs_bool = True

    angle = np.arctan2(goals[1] - ini_val[1], goals[0] - ini_val[0])
    print("check: ", initial_turn)
    print("abs_angle: ", np.abs(angle))
    if np.abs(angle) > 0.1 and initial_turn == False and norm > 0.5:
            print("Initial Turn")
            #ini_val[2] =  ini_val[2] + (angle / norm) * 0.4
            print("turning step: ", ((angle / norm) * 0.4 * time_step))
            ini_val[2] =  ini_val[2] + ((angle / norm) * 0.4 * time_step)
    else:
        print("Initial Turn Complete, entering navigation")
        if initial_turn == False:
            initial_turn = True

        planner = PotentialFieldPlanner([goals[0], goals[1], 0], time_step, k_att, k_rep, vel_max)

        if obs_bool == True:
            for obstacle_ind in obstacle_dict:
                obstacle_arr = np.array(map[obstacle_ind])
                planner.set_obstacle_position_modded([obstacle_arr[0], obstacle_arr[1], 0]) # Set to obtained position of the obstacles by robot
            print("Getting Avoidance Force")
            pos_des, lin_vel =  planner.get_avoidance_force_modded([0, 0, 0])
        else:
            print("Getting Goal Force")
            pos_des, lin_vel =  planner.get_desired_pos_vel([0, 0, 0])

        angle_modded = np.arctan2(pos_des[1] - ini_val[1], pos_des[0] - ini_val[0])
        #angle_modded = (angle_modded / norm) * 0.15
        angle_modded = angle_modded * 0.15

        print("lin_vel", lin_vel)
        print("angle_modded: ", angle_modded)
        # TODO BEGIN MRSS: Update the current command

        if norm > 0.1:
            print("We move")
            ini_val[0] =  ini_val[0] + (lin_vel[0] * time_step)
            ini_val[1] =  ini_val[1] + (lin_vel[1] * time_step)
            ini_val[2] =  ini_val[2] + ((angle_modded / norm) * 0.4 * time_step)
        else:
            print("We stop")
    
    print("ini_val: ", ini_val)    
