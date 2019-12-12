#!/usr/bin/env python

import rospy
import message_filters

from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped, Twist
from dipy.core.sphere import cart2sphere
from numpy import rad2deg

class DroneController(object):
    def __init__(self):
        self.left_hand_sync_sub = message_filters.Subscriber('/left_hand_from_center', TransformStamped)
        self.right_hand_sync_sub = message_filters.Subscriber('/right_hand_from_center', TransformStamped)
        self.drone_launch_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.drone_land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.drone_twist_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)

        ts = message_filters.TimeSynchronizer([self.left_hand_sync_sub, self.right_hand_sync_sub], 1)
        ts.registerCallback(self.control_output_cb)

    def control_output_cb(self, left_hand, right_hand):
        """
        :type left_hand: TransformStamped
        :type right_hand: TransformStamped
        """
        left_spherical = cart2sphere(left_hand.transform.translation.x,
                                     left_hand.transform.translation.y,
                                     left_hand.transform.translation.z)
        right_spherical = cart2sphere(right_hand.transform.translation.x,
                                      right_hand.transform.translation.y,
                                      right_hand.transform.translation.z)

        # print("Left : (Theta,Phi)=(%f,%f)" % (left_spherical[1], left_spherical[2]))
        # print("Right: (Theta,Phi)=(%f,%f)" % (right_spherical[1], right_spherical[2]))

        theta_left = rad2deg(left_spherical[1])
        theta_right = rad2deg(right_spherical[1])

        phi_left = rad2deg(left_spherical[2])
        phi_right = rad2deg(right_spherical[2])

        # Checking for Launch and Landing Positions
        if theta_left < 20 and theta_right < 20:
            self.drone_launch_pub.publish(Empty())
            rospy.loginfo("Launch")
        elif theta_left > 150 and theta_right > 150:
            self.drone_land_pub.publish(Empty())
            rospy.loginfo("Landing")

        # Forward and Backward
        # rospy.loginfo("Left Phi: %f Right Phi: %f" % (phi_left, phi_right))
        t = Twist()
        if 0 > phi_right > -70 and 0 < phi_left < 70:
            t.linear.x = 0.1
            self.drone_twist_pub.publish(t)
            rospy.loginfo("Forward")
        elif -180 < phi_right < -110 and 180 > phi_left > 110:
            t.linear.x = -0.1
            self.drone_twist_pub.publish(t)
            rospy.loginfo("Backward")

        # Left and Right
        # rospy.loginfo("Left Theta: %f Right Theta: %f" % (theta_left, theta_right))
        if 70 < theta_left < 120 and theta_right < 70:
            t.linear.y = 0.1
            rospy.loginfo("Left")
        elif 70 < theta_right < 120 and theta_left < 70:
            t.linear.y = -0.1
            rospy.loginfo("Right")
        self.drone_twist_pub.publish(t)


if __name__ == "__main__":
    rospy.init_node("DroneController")
    dc = DroneController()
    rospy.spin()
