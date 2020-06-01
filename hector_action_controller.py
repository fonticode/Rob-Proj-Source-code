#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range, LaserScan
from math import pow, atan2, sqrt, sin, cos
import numpy as np
import random
import actionlib

from hector_uav_msgs.srv import EnableMotors
import hector_uav_msgs.msg


class DroneController:

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
            'hector_controller', anonymous=True # name of the node
        )

        # param from launch file
        self.name = 'hector10'

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.kill)

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel',  # name of the topic
            Twist,  # message type
            queue_size=10  # queue size
        )

        # create height subscriber
        self.sonar_height_subscriber = rospy.Subscriber(
            '/sonar_height',  # name of the topic
            Range,  # message type
            self.update_sonar_height  # function that hanldes incoming messages
        )

        # create height subscriber
        self.laser_scanner_subscriber = rospy.Subscriber(
            '/scan',  # name of the topic
            LaserScan,  # message type
            self.update_laser_scanner  # function that hanldes incoming messages
        )

        # create goal client
        self.goal_client = actionlib.SimpleActionClient(
        	'/action/pose', 
        	hector_uav_msgs.msg.PoseAction,
        )



        # initialize sonar height to 0
        self.sonar_height = Range().range

        # initialize laser scanner 
        # LaserScan.ranges consists of 1081 values (array/list)
        # The implemented laser scanner scans from -3/4*pi to +3/4*pi (min_angle, max_angle)
        self.laser_scanner = LaserScan()

        # initialize linear and angular velocities to 0
        self.velocity = Twist()

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)


        self.path = [[1,-1],
                        [0,-1],
                        [-0.5,-1],
                        [-1,-1],
                        [-1.5,-1],
                        [-2,-1],
                        [-2.5,-1],
                        [-3,-1],
                        [-3,-0.5],
                        [-3,0],
                        [-3,0.5],
                        [-3,1],
                        [-3,1.5],
                        [-3,2],
                        [-3,2.5],
                        [-2.5,3],
                        [-2,3],
                        [-1.5,3],
                        [-1,3],
                        [-0.5,3],
                        [0,3],
                        [0.5,3],
                        [1,3],
                        [1,2.5],
                        [1,2],
                        [1,1.5],
                        [1,1],
                        [1,0.5],
                        [1,0],
                        [1,-0.5],
                        [1,-1]
                    ]



    def update_sonar_height(self, data):
        self.sonar_height = data.range

    def update_laser_scanner(self, data):
        self.laser_scanner = data


    def kill(self):
        """Drops UAV from sky."""
        rospy.ServiceProxy('/enable_motors', EnableMotors)(False)


        self.rate.sleep()

    def hover(self):
        """Let's the UAV hover at current position."""

        self.velocity_publisher.publish(
            Twist()  # set velocities to 0
        )

        #self.rate.sleep()

    
        self.goal_client.wait_for_result()

    def run(self):
        """Let's the drone explore at 2m altitude"""
        # enable motors, set velocity message
        rospy.ServiceProxy('/enable_motors', EnableMotors)(True)
        g = hector_uav_msgs.msg.PoseGoal()
        g.target_pose.header.frame_id = 'world'
        g.target_pose.pose.position.x = 0 #-7.5
        g.target_pose.pose.position.y = 0 #-5.5
        g.target_pose.pose.position.z = 2
        rospy.loginfo("goal was prepared")
    
        self.goal_client.wait_for_server()
        self.goal_client.send_goal(g)
        self.goal_client.wait_for_result()
        for x_y in self.path:
            print(x_y)
            g.target_pose.pose.position.x = x_y[0]
            g.target_pose.pose.position.y = x_y[1]
            self.goal_client.wait_for_server()
            self.goal_client.send_goal(g)
            self.goal_client.wait_for_result()

 

        while not rospy.is_shutdown():
            self.hover()

           

    def explore(self):
        """Let's the drone explore the room randomly, while avoiding obstacles
        at the front of the laserscanner"""
        while not rospy.is_shutdown():
            if self.obstacleIsClose():
                self.hover()
                #self.takeRandomTurn()

            vel_msg = Twist()
            vel_msg.linear.x = -0.2 # 0.2 m/s forward
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

    def reached2m(self):
        """Checks whether current sonar height reading is bigger or equal to 2m"""
        return self.sonar_height >= 2

    def obstacleIsClose(self):
        """Checks whether an obstacle is close."""
        current_scan = self.laser_scanner.ranges

        # check front
        return sum(current_scan[540:541])/2 < 0.3
        



if __name__ == '__main__':
    controller = DroneController()
    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
