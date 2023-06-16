#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal

import numpy as np
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json

from potential_field_planner import PotentialFieldPlanner

class Planner:
    def __init__(self):
        super().__init__()

        # Initialize Node
        rospy.init_node('Planner', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Initialize subscriber
        self.map_sub = rospy.Subscriber('/map', String, self.map_callback)
        self.map = None
        
        # Intialize publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.cmd = None
        self.rate = rospy.Rate(10)  # Publisher frequency

        self.initial_turn = False
        self.right_push = False

        self.time_step = 1/10
        self.k_att     = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.25]])
        self.k_rep     = 1
        self.vel_max   = 0.5

        # TODO BEGIN MRSS: Add attributes (If needed)

        # END MRSS

    def map_callback(self, msg):
        rospy.logerr("Entering Callback")
        self.map = json.loads(msg.data)
        obstacle_dict = [key for key in self.map.keys() if 'obstacle' in key.lower()]
        # TODO BEGIN MRSS: Use map for planning
        goals = np.array(self.map["/goal"])

        if not obstacle_dict:
            obs_bool = False
        else:
            obs_bool = True

        norm = np.linalg.norm(goals)
        #goals = goals / norm
        #goals = goals * 0.15
        # END MRSS
        
        angle = np.arctan2(goals[1], goals[0])


        # Twist
        self.cmd = geometry_msgs.msg.Twist()
        
        if np.abs(angle) > 0.1: # and self.initial_turn == False: # and norm > 0.5:
            rospy.logerr("Initial Turn")
            self.cmd.linear.x = 0.
            self.cmd.linear.y = 0.
            self.cmd.angular.z = (angle / norm) * 0.1

        elif norm > 0.1:
            rospy.logerr("Initial Turn Complete, entering navigation")
            #self.initial_turn = True

            planner = PotentialFieldPlanner([goals[0], goals[1], 0], self.time_step, self.k_att, self.k_rep, self.vel_max)
            
            planner.set_obstacle_distance(0.7)

            if obs_bool:
                for obstacle_ind in obstacle_dict:
                    obstacle_arr = np.array(self.map[obstacle_ind])
                    planner.set_obstacle_position_modded([obstacle_arr[0], obstacle_arr[1], 0]) # Set to obtained position of the obstacles by robot
                rospy.logerr("Getting Avoidance Force")
                pos_des, lin_vel =  planner.get_avoidance_force_modded([0, 0, 0])
            else:
                rospy.logerr("Getting Goal Force")
                pos_des, lin_vel =  planner.get_desired_pos_vel([0, 0, 0])        

            rospy.logerr("Linear Velocity: ")
            rospy.logerr(lin_vel)
            norm_lin_vel = np.linalg.norm(lin_vel[:2])
            lin_vel = (lin_vel / norm_lin_vel) * 0.15
            rospy.logerr("Linear Velocity for moving (modded): ")
            rospy.logerr(lin_vel)
            
            angle_modded = np.arctan2(pos_des[1], pos_des[0])
            rospy.logerr("Angular Velocity: ")
            rospy.logerr(angle_modded)
            norm_angle_modded = np.linalg.norm(pos_des[:2])
            angle_modded = (angle_modded / norm_angle_modded) * 0.1
            rospy.logerr("Angular Velocity for turning (modded): ")
            rospy.logerr(angle_modded)
            
            # TODO BEGIN MRSS: Update the current command

            rospy.logerr("Norm greater than 0.1 - Moving")
            self.cmd.linear.x = lin_vel[0]
            self.cmd.linear.y = lin_vel[1]
            self.cmd.angular.z = angle_modded
            
        else:
            #self.initial_turn = True

            rospy.logerr("NOT MOVING")
            self.cmd.linear.x = 0.
            self.cmd.linear.y = 0.
            self.cmd.angular.z = 0.
    
            
    def spin(self):
        '''
        Spins the node.
        '''
        try:
            while not rospy.is_shutdown():
                if self.cmd is not None:
                    # Publish
                    self.cmd_pub.publish(self.cmd)
                else:
                    rospy.logwarn("SKIP")

                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down planner.")

    def on_shutdown(self):
        '''
        Called on node shutdown.
        '''
        pass


if __name__ == '__main__':
    try:
        node = Planner()
        node.spin()
    except rospy.ROSInterruptException:
        pass
