#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from wild_visual_navigation_msgs.msg import RobotState
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist
from std_msgs.msg import Header


class OdometryProcessor:
    def __init__(self):
        self.odom_subscriber = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.odom_callback
        )  # subscribed to odometry
        self.robot_state_publisher = rospy.Publisher(
            "/wild_visual_navigation_node/robot_state", RobotState, queue_size=10
        ) # set a publisher of Robot State messages

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

class TwistProcessor:
    def __init__(self):
        self.twist_subscriber = rospy.Subscriber(
            "/husky_velocity_controller/cmd_vel", Twist, self.twist_callback
        )  # subscribed to twist stamped
        self.twist_stamped_publisher = rospy.Publisher(
            "/motion_reference/command_twist", TwistStamped, queue_size=10
        )

    def twist_callback(self, msg):
        #rospy.loginfo("Received Twist message.")

        twist_stamped = TwistStamped()

        twist_stamped.header = Header()
        twist_stamped.header.stamp = rospy.Time.now()

        twist_stamped.twist = msg

        self.twist_stamped_publisher.publish(twist_stamped)
        #rospy.loginfo("Published TwistStamped message.")



if __name__ == "__main__":
    rospy.init_node("topic_processor", anonymous=True)  # made the node
    odometry_processor = OdometryProcessor()
    twist_processor = TwistProcessor()
    #rospy.loginfo("odometry_processor node started. Waiting for message.")
    rospy.spin()
