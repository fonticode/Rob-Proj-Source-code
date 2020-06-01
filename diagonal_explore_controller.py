#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range, LaserScan
from math import pow, atan2, sqrt, sin, cos
import numpy as np
import actionlib
from hector_uav_msgs.msg import PoseAction, PoseGoal

from hector_uav_msgs.srv import EnableMotors


class DroneController:

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
            'hector_controller', anonymous=True # name of the node
        )

        # param from launch file
        #self.name = rospy.get_param('~uav_name')
        self.name = 'hector10'

        # # log robot name to console
        # rospy.loginfo('Controlling %s' % self.name)

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

        self.action_client = actionlib.SimpleActionClient('/action/pose', PoseAction)
        

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.kill)

        # initialize pose to (X=0, Y=0, theta=0)
        self.pose = Pose()

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

    def update_sonar_height(self, data):
        self.sonar_height = data.range

    def update_laser_scanner(self, data):
        self.laser_scanner = data

    def kill(self):
        """Drops UAV from sky."""
        rospy.ServiceProxy('/enable_motors', EnableMotors)(False)
        self.rate.sleep()

    def takeoff(self):
        rospy.ServiceProxy('/enable_motors', EnableMotors)(True)
        vel_msg = Twist()
        vel_msg.linear.z = 0.5
        while not self.reached_min_height():
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

    def hover(self):
        self.velocity_publisher.publish(
            Twist()  # set velocities to 0
        )
        self.rate.sleep()

    def run(self):
        
        self.takeoff()
        self.hover()
        self.explore()


    def explore(self):
        """Let's the drone explore the room randomly, while avoiding obstacles
        at the front of the laserscanner"""
        while not self.obstacle_center():
            vel_msg = Twist()
            vel_msg.linear.x = 0.5 # 0.2 m/s forward
            vel_msg.linear.z = 0.001
            self.velocity_publisher.publish(vel_msg)

            if self.obstacle_right():
                self.hover()
                while self.obstacle_right():
                    vel_msg = Twist()
                    vel_msg.angular.z = 0.5
                    self.velocity_publisher.publish(vel_msg)
                    self.rate.sleep()

            if self.obstacle_left():
                self.hover()
                while self.obstacle_left():
                    vel_msg = Twist()
                    vel_msg.angular.z = -0.5
                    self.velocity_publisher.publish(vel_msg)
                    self.rate.sleep()



        self.hover()
        

    def reached_min_height(self):
        """Checks whether current sonar
         height reading is bigger or equal to 2m"""
        return self.sonar_height >= 1.5

    def obstacle_center(self):
        """Checks whether an obstacle is close."""
        current_scan = self.laser_scanner.ranges
        # check front
        return sum(current_scan[540:541])/2 <= 0.3

    def obstacle_left(self):
        scan = self.laser_scanner.ranges
        return min(scan[360:540]) <= 0.7 

    def obstacle_right(self):
        scan = self.laser_scanner.ranges
        return min(scan[541:721]) <= 0.7  
        



if __name__ == '__main__':
    controller = DroneController()
    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
