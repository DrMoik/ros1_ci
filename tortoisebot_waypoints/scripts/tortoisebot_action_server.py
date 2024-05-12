#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math

from course_web_dev_ros.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction


class WaypointActionClass(object):
    def __init__(self):
        self._feedback = WaypointActionFeedback()
        self._result = WaypointActionResult()

        self._pub_cmd_vel = None
        self._sub_odom = None
        self._position = Point()
        self._yaw = 0
        self._state = 'idle'
        self._des_pos = Point()
        self._yaw_precision = math.pi / 90  # +/- 2 degree allowed
        self._dist_precision = 0.05

        rospy.init_node('tortoisebot_as')
        self._as = actionlib.SimpleActionServer(
            "tortoisebot_as", WaypointActionAction, self.goal_callback, False)
        self._as.start()

        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._sub_odom = rospy.Subscriber(
            '/odom', Odometry, self.odom_callback)
        rospy.loginfo("Action server started")

        self._rate = rospy.Rate(25)

    def odom_callback(self, msg):
        self._position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = transformations.euler_from_quaternion(quaternion)
        self._yaw = euler[2]

    def goal_callback(self, goal):
        rospy.loginfo("Goal %s received" % str(goal))
        success = True
        self._des_pos = goal.position
        desired_yaw = math.atan2(
            self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
        err_pos = math.sqrt((self._des_pos.y - self._position.y)
                            ** 2 + (self._des_pos.x - self._position.x) ** 2)

        while err_pos > self._dist_precision and success:
            desired_yaw = math.atan2(
                self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
            err_yaw = desired_yaw - self._yaw
            err_pos = math.sqrt((self._des_pos.y - self._position.y)
                                ** 2 + (self._des_pos.x - self._position.x) ** 2)
           # rospy.loginfo("Current Yaw: %s" % str(self._yaw))
           # rospy.loginfo("Desired Yaw: %s" % str(desired_yaw))
           # rospy.loginfo("Error Yaw: %s" % str(err_yaw))

            if self._as.is_preempt_requested():
                rospy.loginfo("The goal has been cancelled/preempted")
                self._as.set_preempted()
                success = False
            elif abs(err_yaw) > self._yaw_precision:
                rospy.loginfo("Fix yaw")
                self._state = 'fix yaw'
                twist_msg = Twist()
                twist_msg.angular.z = max(min(5*err_yaw, 0.65), -0.65)
                self._pub_cmd_vel.publish(twist_msg)
            else:
                rospy.loginfo("Go to point")
                self._state = 'go to point'
                twist_msg = Twist()
                twist_msg.linear.x = 0.6
                twist_msg.angular.z = 0
                self._pub_cmd_vel.publish(twist_msg)

            self._feedback.position = self._position
            self._feedback.state = self._state
            self._as.publish_feedback(self._feedback)
            self._rate.sleep()

        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self._pub_cmd_vel.publish(twist_msg)

        if success:
            self._result.success = True
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    WaypointActionClass()
    rospy.spin()
