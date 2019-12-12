#!/usr/bin/env python

import rospy
import message_filters

from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped
from tf_monitor_msgs.srv import SetStaticTf, SetStaticTfRequest, GetStampedTopicTf, GetStampedTopicTfRequest


class HandTracker(object):
    def __init__(self):
        self.right_hand_sub  = rospy.Subscriber("/vicon/right_hand/right_hand",  TransformStamped, self.right_hand_cb)
        self.left_hand_sub   = rospy.Subscriber("/vicon/left_hand/left_hand",    TransformStamped, self.left_hand_cb)
        self.set_center_sub  = rospy.Subscriber("/set_center", Empty, self.set_center_cb)

        self.right_hand_from_center_sub = rospy.Subscriber("/right_hand_from_center", TransformStamped, self.right_hand_center_cb)
        self.left_hand_from_center_sub  = rospy.Subscriber("/left_hand_from_center", TransformStamped, self.left_hand_center_cb)
        self.right_hand_from_center_pub = rospy.Publisher("/right_hand_from_center_pub", TransformStamped, queue_size=1)
        self.left_hand_from_center_pub = rospy.Publisher("/left_hand_from_center_pub", TransformStamped, queue_size=1)
        self.right_hand_position = None
        self.left_hand_position  = None


        rospy.wait_for_service("/tf_monitor/SetStaticTf")
        rospy.wait_for_service("/tf_monitor/RequestStampedTopicTf")

        self.static_srv = rospy.ServiceProxy("/tf_monitor/SetStaticTf", SetStaticTf)
        self.tf_topic_srv = rospy.ServiceProxy("/tf_monitor/RequestStampedTopicTf", GetStampedTopicTf)

    def right_hand_cb(self, msg):
        """
        :type msg: TransformStamped
        """
        self.right_hand_position = msg

    def left_hand_cb(self, msg):
        """
        :type msg: TransformStamped
        """
        self.left_hand_position = msg

    def right_hand_center_cb(self, msg):
        """
        :type msg: TransformStamped
        """
        msg.child_frame_id = "right_hand"
        self.right_hand_from_center_pub.publish(msg)

    def left_hand_center_cb(self, msg):
        """
        :type msg: TransformStamped
        """
        msg.child_frame_id = "left_hand"
        self.left_hand_from_center_pub.publish(msg)

    def set_center_cb(self, msg):
        static_tf = TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = "world"
        static_tf.child_frame_id = "center"
        static_tf.transform.translation.x = self.right_hand_position.transform.translation.x + self.left_hand_position.transform.translation.x
        static_tf.transform.translation.x /= 2

        static_tf.transform.translation.y = self.right_hand_position.transform.translation.y + self.left_hand_position.transform.translation.y
        static_tf.transform.translation.y /= 2

        static_tf.transform.translation.z = self.right_hand_position.transform.translation.z + self.left_hand_position.transform.translation.z
        static_tf.transform.translation.z /= 2

        static_tf.transform.rotation.w = 1

        req = SetStaticTfRequest()
        req.tf = static_tf
        self.static_srv.call(req)
        rospy.loginfo("Request Static Sent")

        topic_request = GetStampedTopicTfRequest()
        topic_request.out_frame = "center"
        topic_request.topic_name = "/vicon/right_hand/right_hand"
        topic_request.out_topic_name_recommendation = "right_hand_from_center"
        self.tf_topic_srv.call(topic_request)

        topic_request = GetStampedTopicTfRequest()
        topic_request.out_frame = "center"
        topic_request.topic_name = "/vicon/left_hand/left_hand"
        topic_request.out_topic_name_recommendation = "left_hand_from_center"
        self.tf_topic_srv.call(topic_request)

        rospy.loginfo("Requesting Topic Transformations to 'center'")


if __name__ == "__main__":
    rospy.init_node("hand_controller")
    hc = HandTracker()
    rospy.spin()
