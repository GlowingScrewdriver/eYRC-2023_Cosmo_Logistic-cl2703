#!/usr/bin/python3

# Team ID:          CL#2703
# Theme:            Cosmo Logistic
# Author List:		Vedanth Padmaraman, Vamshi Vishruth
# Filename:		    task1c.py
#
# Functions:        __init__(),odom_cb(msg),spin(rate),adjust(goal_angle),navigate(final_pose),gen_pose_msg(pose),main()
# Classes:          Navigator

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import time
from scipy.spatial.transform import Rotation as R

class Navigator (BasicNavigator):
    def __init__ (self):
        BasicNavigator.__init__(self)
        self.robot_pose = None
        self.cb_group = rclpy.callback_groups.ReentrantCallbackGroup ()
        self.create_subscription (Odometry, '/odom', self.odom_cb, 10, callback_group=self.cb_group)
        self.vel_pub = self.create_publisher (Twist, '/cmd_vel', 10)

    def odom_cb (self, msg):
        """
        Callback function for odometry data.

        Updates the robot_pose attribute with the latest pose from the odometry message.
        """
        self.robot_pose = PoseStamped (pose=msg.pose.pose, header=msg.header)
        self.robot_pose.header.frame_id = 'map'

    def spin (self, rate):
        """
        Tells the robot to spin

        Args:
            rate (float): Angular velocity in rad/s.
        """
        vel = Twist ()
        vel.angular.z = rate
        self.vel_pub.publish (vel)

    def adjust (self, goal_angle):
        """
        Fine adjustment to robot's orientation

        Args:
            goal_angle (float): The desired orientation angle in radians.
        """
        cur_angle = R.from_quat ([0.0, 0.0, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w]).as_euler ('xyz')[2]
        rot = 0.3
        if goal_angle - cur_angle < 0: rot *= -1
        self.spin (rot)
        while (goal_angle - cur_angle)**2 > 0.2**2:
            #print (f'at angle {cur_angle}')
            self.get_clock().sleep_for (rclpy.time.Duration(seconds = 0.1))
            cur_angle = R.from_quat ([0.0, 0.0, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w]).as_euler ('xyz')[2]
        #print ('aligned')
        self.spin (0.0)

    def navigate (self, fin_pose):
        """
        Navigate the robot from its current pose to the specified final pose.

        Args:
            fin_pose (dict): A dictionary containing the target pose information. Refer elements of pose_list assignment under __name__ == "__main__"
        """
        while not self.robot_pose:
            print ('waiting for AMCL pose')
            time.sleep (0.5)

        init_pose_msg = self.robot_pose
        fin_pose_msg = self.gen_pose_msg (fin_pose)

        #print (f'moving from {init_pose_msg} to {fin_pose_msg}')
        path = self.getPath (init_pose_msg, fin_pose_msg)

        self.followPath (path)
        while not self.isTaskComplete ():
            time.sleep (2)

        if self.getResult() == TaskResult.SUCCEEDED:
            print ('Complete!')
        else:
            print ('DNF :/')

    def gen_pose_msg (self, pose):
        """
        Convert from pose dictionaries (used by e.g. navigate ()) to PoseStamped.

        Args:
            pose (dict): A dictionary containing the target pose information.

        Returns:
            PoseStamped: The generated PoseStamped message.
        """
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = pose['trans'][0]
        goal.pose.position.y = pose['trans'][1]
        goal.pose.orientation.z = pose['rot']
        goal.pose.orientation.w = 1.0

        return goal

if __name__ == '__main__':
    rclpy.init()

    navigator = Navigator ()
    navigator.waitUntilNav2Active ()

    pose_list = [
        # Each element here is one of the required poses for Task 1C
        { 'trans': [0.0,  0.0],  'rot':  1.55 },
        #{ 'trans': [ 1.8,  1.5],  'rot':  -1.57 },
        #{ 'trans': [ 2.0, -7.0],  'rot': -1.57 },
        #{ 'trans': [-3.0,  2.5],  'rot':  1.57 },
    ]

    init_pose_msg = PoseStamped ()
    for pose in pose_list:
        navigator.navigate (pose)

    rclpy.shutdown ()