#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from wild_visual_navigation_msgs.msg import RobotState
from geometry_msgs.msg import TwistStamped, PoseStamped


class OdometryProcessor:
    def __init__(self):
        self.odom_subscriber = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.odom_callback
        )  # subscribed to odometry
        self.robot_state_publisher = rospy.Publisher(
            "/wild_visual_navigation_node/robot_state", RobotState, queue_size=10
        )

    def odom_callback(self, msg):
        #rospy.loginfo("Received odometry message.")

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        twist_stamped = TwistStamped()
        twist_stamped.header = msg.header
        twist_stamped.twist = msg.twist.twist

        robot_state_msg = RobotState()
        robot_state_msg.pose = pose_stamped
        robot_state_msg.twist = twist_stamped

        self.robot_state_publisher.publish(robot_state_msg)
        #rospy.loginfo("Published RobotState message.")


if __name__ == "__main__":
    rospy.init_node("odometry_processor", anonymous=True)  # made the node
    odometry_processor = OdometryProcessor()
    #rospy.loginfo("odometry_processor node started. Waiting for message.")
    rospy.spin()
