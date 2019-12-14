#!/usr/bin/env python

import rospy
import message_filters

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from dipy.core.sphere import cart2sphere
from numpy import rad2deg

NEUTRAL     = 0
FORWARD     = 1
BACKWARD    = 2
UP          = 3
DOWN        = 4
TAKEOFF     = 5
LAND        = 6



class DroneController(object):
    def __init__(self):

        self.drone_arrow_pub = rospy.Publisher('/drone_arrow', PoseStamped, queue_size=1)

        self.drone_launch_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.drone_land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.drone_twist_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)

        self.left_hand_sub = rospy.Subscriber('/left_hand_from_center', TransformStamped, self.left_hand_cb)
        self.right_hand_sub = rospy.Subscriber('/right_hand_from_center', TransformStamped, self.right_hand_cb)
        self.left_hand = None
        self.right_hand = None

        self.timer = rospy.Timer(rospy.Duration.from_sec(0.05), self.control_output_cb)

    def left_hand_cb(self, msg):
        """
        :type msg: TransformStamped
        """
        self.left_hand = msg

    def right_hand_cb(self, msg):
        """
        :type msg: TransformStamped
        """
        self.right_hand = msg

    def classify_hand(self, theta, phi):
        theta_classification = NEUTRAL
        phi_classification = NEUTRAL
        phi = abs(phi)

        if 0 < theta < 20:
            theta_classification = TAKEOFF
        elif 21 < theta < 60:
            theta_classification = UP
        elif 61 < theta < 100:
            theta_classification = NEUTRAL
        elif 101 < theta < 150:
            theta_classification = DOWN
        elif 151 < theta < 180:
            theta_classification = LAND

        if 0 < phi < 70:
            phi_classification = FORWARD
        elif 71 < phi < 110:
            phi_classification = NEUTRAL
        elif 111 < phi < 180:
            phi_classification = BACKWARD

        return theta_classification, phi_classification

    # def control_output_cb(self, left_hand, right_hand):
    def control_output_cb(self, event):
        """
        :type left_hand: TransformStamped
        :type right_hand: TransformStamped
        """
        if self.left_hand is None:
            return

        if self.right_hand is None:
            return

        left_spherical = cart2sphere(self.left_hand.transform.translation.x,
                                     self.left_hand.transform.translation.y,
                                     self.left_hand.transform.translation.z)
        right_spherical = cart2sphere(self.right_hand.transform.translation.x,
                                      self.right_hand.transform.translation.y,
                                      self.right_hand.transform.translation.z)


        # print("Left : (Theta,Phi)=(%f,%f)" % (rad2deg(left_spherical[1]), rad2deg(left_spherical[2])))
        # print("Right: (Theta,Phi)=(%f,%f)" % (rad2deg(right_spherical[1]), rad2deg(right_spherical[2])))

        theta_left, phi_left = self.classify_hand(rad2deg(left_spherical[1]), rad2deg(left_spherical[2]))
        theta_right, phi_right = self.classify_hand(rad2deg(right_spherical[1]), rad2deg(right_spherical[2]))

        arrow_pose = PoseStamped()
        arrow_pose.pose.position.x = -1

        # Checking for Launch and Landing Positions
        if theta_left == TAKEOFF and theta_right == TAKEOFF:
            self.drone_launch_pub.publish(Empty())
            arrow_pose.pose.position.x = 0
            self.drone_arrow_pub.publish(arrow_pose)
            rospy.loginfo("Launch")
            return
        elif theta_left == LAND and theta_right == LAND:
            self.drone_land_pub.publish(Empty())
            arrow_pose.pose.position.x = 1
            self.drone_arrow_pub.publish(arrow_pose)
            rospy.loginfo("Landing")
            return

        t = Twist()  # type: Twist
        if phi_left == FORWARD and phi_right == FORWARD:
            t.linear.x = 0.1
            arrow_pose.pose.position.x = 2
            rospy.loginfo("Forward")
        elif phi_left == BACKWARD and phi_right == BACKWARD:
            t.linear.x = -0.1
            arrow_pose.pose.position.x = 3
            rospy.loginfo("Backward")
        elif phi_left == BACKWARD and phi_right == FORWARD:
            t.angular.z = 0.1
            arrow_pose.pose.position.x = 4
            rospy.loginfo("Rotate Left")
        elif phi_left == FORWARD and phi_right == BACKWARD:
            t.angular.z = -0.1
            arrow_pose.pose.position.x = 5
            rospy.loginfo("Rotate Right")

        if theta_left == UP and theta_right == UP:
            t.linear.z = 0.1
            arrow_pose.pose.position.x = 6
            rospy.loginfo("Up")
        elif theta_left == DOWN and theta_right == DOWN:
            t.linear.z = -0.1
            arrow_pose.pose.position.x = 7
            rospy.loginfo("Down")
        elif (theta_left == NEUTRAL or theta_left == DOWN) and theta_right == UP:
            t.linear.y = 0.1
            arrow_pose.pose.position.x = 8
            rospy.loginfo("Left")
        elif theta_left == UP and (theta_right == NEUTRAL or theta_right == DOWN):
            t.linear.y = -0.1
            arrow_pose.pose.position.x = 9
            rospy.loginfo("Right")

        self.drone_arrow_pub.publish(arrow_pose)
        self.drone_twist_pub.publish(t)

        # Forward and Backward
        # rospy.loginfo("Left Phi: %f Right Phi: %f" % (phi_left, phi_right))
        # t = Twist()
        # if 0 > phi_right > -70 and 0 < phi_left < 70:
        #     t.linear.x = 0.1
        #     rospy.loginfo("Forward")
        # elif -180 < phi_right < -110 and 180 > phi_left > 110:
        #     t.linear.x = -0.1
        #     rospy.loginfo("Backward")
        #
        # # Left and Right
        # # rospy.loginfo("Left Theta: %f Right Theta: %f" % (theta_left, theta_right))
        # if 70 < theta_left < 120 and theta_right < 70:
        #     t.linear.y = 0.1
        #     rospy.loginfo("Left")
        # elif 70 < theta_right < 120 and theta_left < 70:
        #     t.linear.y = -0.1
        #     rospy.loginfo("Right")
        # self.drone_twist_pub.publish(t)


if __name__ == "__main__":
    rospy.init_node("DroneController")
    dc = DroneController()
    rospy.spin()
