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

        self.time_step = 1/30
        self.k_att     = numpy.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.25]])
        self.k_rep     = 1
        self.vel_max   = 0.5
        # TODO BEGIN MRSS: Add attributes (If needed)

        # END MRSS

    def map_callback(self, msg):
        self.map = json.loads(msg.data)

        # TODO BEGIN MRSS: Use map for planning
        goals = np.array(self.map["/goal"])
        obstacle_1 = np.array(self.map["/obstacle1"])
        norm = np.linalg.norm(goals)
        #goal_vel = goals / norm
        #goal_vel = goal_vel * 0.15
        # END MRSS
        
        angle = np.arctan2(goals[1], goals[0])

        if np.abs(angle) > 0.1 and self.initial_turn == False:
            self.cmd.linear.x = 0.
            self.cmd.linear.y = 0.
            self.cmd.angular.z = (angle / norm) * 0.1

        else:
            if self.initial_turn == False:
                self.initial_turn = True

            # Twist
            self.cmd = geometry_msgs.msg.Twist()

            planner = PotentialFieldPlanner([goals[0], goals[1], 0], self.time_step, self.k_att, self.k_rep, self.vel_max)

            planner.set_obstacle_distance(1.0)
            planner.set_obstacle_position([obstacle_1[0], obstacle_1[1], 0]) # Set to obtained position of the obstacles by robot

            #pos_des, lin_vel =  planner.get_avoidance_force (pos)
            pos_des, lin_vel =  planner.get_avoidance_force ([0, 0, 0])
            
            #hybrid_action, info = controller.update(lin_vel, ang_vel)

            #goals = np.array(self.map["/goal"])
            #norm = np.linalg.norm(goals)
            #goal_vel = goals / norm
            #goal_vel = goal_vel * 0.15

            #angle_modded = np.arctan2(goals[1], goals[0])
            #angle_modded = (angle / norm) * 0.15
            angle_modded = angle * 0.15



            # TODO BEGIN MRSS: Update the current command
            '''
            if np.abs(angle) > 0.1:
                self.cmd.linear.x = 0.
                self.cmd.linear.y = 0.
                self.cmd.angular.z = (angle / norm) * 0.1
            '''

            if norm > 0.1:
                # Check if velocity has approached zero (minima problem)
                
                '''
                if np.linalg.norm(lin_vel) < 0.05:
                    if self.right_push == False:
                        self.cmd.linear.x = 0
                        self.cmd.linear.y = -0.1
                        self.cmd.angular.z = 0
                        self.right_push = True
                    else:
                        self.cmd.linear.x = 0
                        self.cmd.linear.y = -0.1
                        self.cmd.angular.z = 0
                        self.right_push = False
                else:
                
                    self.cmd.linear.x = lin_vel[0]
                    self.cmd.linear.y = lin_vel[1]
                    self.cmd.angular.z = angle_modded
                '''
                
                self.cmd.linear.x = lin_vel[0]
                self.cmd.linear.y = lin_vel[1]
                self.cmd.angular.z = angle_modded
                
            else:
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
