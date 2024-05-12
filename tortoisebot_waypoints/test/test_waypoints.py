#!/usr/bin/env python3
import unittest
import rospy
import actionlib
from geometry_msgs.msg import Point, Quaternion
from course_web_dev_ros.msg import WaypointActionAction, WaypointActionGoal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class TestWaypointAction(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_waypoint_action')
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.client.wait_for_server()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_position = Point()
        self.current_yaw = 0.0

    def odom_callback(self, data):
        self.current_position = data.pose.pose.position
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def test_position(self):
        # Setting a test goal position
        goal = WaypointActionGoal()
        goal.position.x = 0.5
        goal.position.y = 0.5
        # Send the goal and wait for the result within 30 seconds
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(30))
        # Test to check the position
        self.assertAlmostEqual(self.current_position.x, goal.position.x, places=1, msg="X position did not match.")
        self.assertAlmostEqual(self.current_position.y, goal.position.y, places=1, msg="Y position did not match.")

    def test_yaw(self):
        # Setting a test goal which should also influence the yaw
        goal = WaypointActionGoal()
        goal.position.x = 0.0
        goal.position.y = 0.0
        expected_yaw = math.atan2(goal.position.x -0.5, goal.position.y- 0.5)  # Robot starts at (0,0), facing along x-axis
        # Send the goal and wait for the result within 30 seconds
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(30))
        # Test to check the yaw
        self.assertLessEqual(abs(self.current_yaw - expected_yaw), 0.3, msg=f"Yaw did not match expected. Expected: {expected_yaw}, Got: {self.current_yaw}, Difference: {abs(self.current_yaw - expected_yaw)}")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('tortoisebot_waypoints', 'test_waypoint_action', TestWaypointAction)
