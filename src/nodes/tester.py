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

time_step = 1/30
k_att     = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.25]])
k_rep     = 1
vel_max   = 0.5
# TODO BEGIN MRSS: Add attributes (If needed)

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
    #print("Obstacle Dictionary: ", str(obstacle_dict))
    
    '''
    if "/obstacle0" in obstacle_dict:
        print("obstacle 0 detected")
    if "/obstacle1" in obstacle_dict:
        print("obstacle 1 detected")
    obstacle_1 = np.array(map["/obstacle1"])
    if "/obstacle2" in obstacle_dict:
        print("obstacle 2 detected")
    if "/obstacle3" in obstacle_dict:
        print("obstacle 3 detected")
    '''

    norm = np.linalg.norm(goals)
    #goal_vel = goals / norm
    #goal_vel = goal_vel * 0.15
    # END MRSS

    print("obs_dict: ", obstacle_dict)

    if not obstacle_dict:
        obs_bool = False
    else:
        obs_bool = True

    angle = np.arctan2(goals[1], goals[0])

    # Twist
    #cmd = geometry_msgs.msg.Twist()
    #cmd.linear.x = 0.
    #cmd.linear.y = 0.
    #cmd.angular.z = 0.

    #if np.abs(angle) > 0.1 and initial_turn == False and norm > 0.5:
    #    print("Initial Turn")
    ##    cmd.linear.y = 0.
     #   cmd.angular.z = (angle / norm) * 0.4

    print("Initial Turn Complete, entering navigation")
    if initial_turn == False:
        initial_turn = True


    planner = PotentialFieldPlanner([goals[0], goals[1], 0], time_step, k_att, k_rep, vel_max)
    """
    if "/obstacle1" in obstacle_dict:
        planner.set_obstacle_distance(1.0)
        planner.set_obstacle_position_modded([obstacle_1[0], obstacle_1[1], 0]) # Set to obtained position of the obstacles by robot

    #pos_des, lin_vel =  planner.get_avoidance_force (pos)
    if "/obstacle1" in obstacle_dict:
        pos_des, lin_vel =  planner.get_avoidance_force_modded([0, 0, 0])
    else:
        pos_des, lin_vel =  planner.get_desired_pos_vel([0, 0, 0])
    """
    if obs_bool == True:
        for obstacle_ind in obstacle_dict:
            obstacle_arr = np.array(map[obstacle_ind])
            planner.set_obstacle_position_modded([obstacle_arr[0], obstacle_arr[1], 0]) # Set to obtained position of the obstacles by robot
        print("Getting Avoidance Force")
        pos_des, lin_vel =  planner.get_avoidance_force_modded([0, 0, 0])
    else:
        print("Getting Goal Force")
        pos_des, lin_vel =  planner.get_desired_pos_vel([0, 0, 0])
    #hybrid_action, info = controller.update(lin_vel, ang_vel)

    #goals = np.array(map["/goal"])
    #norm = np.linalg.norm(goals)
    #goal_vel = goals / norm
    #goal_vel = goal_vel * 0.15

    angle_modded = np.arctan2(pos_des[1], pos_des[0])
    #angle_modded = (angle_modded / norm) * 0.15
    angle_modded = angle_modded * 0.15

    print("lin_vel", lin_vel)
    print("angle_modded: ", angle_modded)
    # TODO BEGIN MRSS: Update the current command
    '''
    if np.abs(angle) > 0.1:
        cmd.linear.x = 0.
        cmd.linear.y = 0.
        cmd.angular.z = (angle / norm) * 0.1
    '''

    if norm > 0.1:
        # Check if velocity has approached zero (local minima problem)
        
        '''
        if np.linalg.norm(lin_vel) < 0.05:
            if right_push == False:
                cmd.linear.x = 0
                cmd.linear.y = -0.1
                cmd.angular.z = 0
                right_push = True
            else:
                cmd.linear.x = 0
                cmd.linear.y = -0.1
                cmd.angular.z = 0
                right_push = False
        else:
        
            cmd.linear.x = lin_vel[0]
            cmd.linear.y = lin_vel[1]
            cmd.angular.z = angle_modded
        '''
        
        #cmd.linear.x = lin_vel[0]
        #cmd.linear.y = lin_vel[1]
        #cmd.angular.z = angle_modded
        
    else:
        '''
        if np.abs(angle) > 0.1 and initial_turn == False:
            cmd.linear.x = 0.
            cmd.linear.y = 0.
            cmd.angular.z = (angle / norm) * 0.1
        
        else:
        '''

        ##cmd.linear.x = 0.
        #cmd.linear.y = 0.
        #cmd.angular.z = 0.

