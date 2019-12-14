#!/usr/bin/env python

import rospy
import message_filters

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf_monitor_msgs.srv import SetStaticTf, SetStaticTfRequest, GetStampedTopicTf, GetStampedTopicTfRequest


class HandTracker(object):
    def __init__(self):
        self.right_hand_sub  = rospy.Subscriber("/pimphand",    PoseStamped, self.right_hand_cb)
        self.left_hand_sub   = rospy.Subscriber("/lefthand",    PoseStamped, self.left_hand_cb)
        self.head_sub        = rospy.Subscriber("/head",        PoseStamped, self.head_cb)

        self.right_hand_tf_pub  = rospy.Publisher("/pimphand_tf", TransformStamped, queue_size=1)
        self.left_hand_tf_pub   = rospy.Publisher("/lefthand_tf", TransformStamped, queue_size=1)
        self.head_tf_pub        = rospy.Publisher("/head_tf",     TransformStamped, queue_size=1)

        self.right_hand_position = None
        self.left_hand_position  = None

        rospy.wait_for_service("/tf_monitor/SetStaticTf")
        rospy.wait_for_service("/tf_monitor/RequestStampedTopicTf")

        self.static_srv = rospy.ServiceProxy("/tf_monitor/SetStaticTf", SetStaticTf)
        self.tf_topic_srv = rospy.ServiceProxy("/tf_monitor/RequestStampedTopicTf", GetStampedTopicTf)

        topic_request = GetStampedTopicTfRequest()
        topic_request.out_frame = "head"
        topic_request.topic_name = "/pimphand_tf"
        topic_request.out_topic_name_recommendation = "right_hand_from_center"
        self.tf_topic_srv.call(topic_request)

        topic_request = GetStampedTopicTfRequest()
        topic_request.out_frame = "head"
        topic_request.topic_name = "/lefthand_tf"
        topic_request.out_topic_name_recommendation = "left_hand_from_center"
        self.tf_topic_srv.call(topic_request)

    def right_hand_cb(self, msg):
        """
        :type msg: PoseStamped
        """
        tf_msg                          = TransformStamped()
        tf_msg.header.frame_id          = "world"
        tf_msg.header.stamp             = msg.header.stamp
        tf_msg.child_frame_id           = "right_hand"
        tf_msg.transform.translation.x  = msg.pose.position.x
        tf_msg.transform.translation.y  = msg.pose.position.y
        tf_msg.transform.translation.z  = msg.pose.position.z
        tf_msg.transform.rotation.x     = msg.pose.orientation.x
        tf_msg.transform.rotation.y     = msg.pose.orientation.y
        tf_msg.transform.rotation.z     = msg.pose.orientation.z
        tf_msg.transform.rotation.w     = msg.pose.orientation.w
        self.right_hand_tf_pub.publish(tf_msg)

    def left_hand_cb(self, msg):
        """
        :type msg: PoseStamped
        """
        tf_msg                          = TransformStamped()
        tf_msg.header.frame_id          = "world"
        tf_msg.header.stamp             = msg.header.stamp
        tf_msg.child_frame_id           = "left_hand"
        tf_msg.transform.translation.x  = msg.pose.position.x
        tf_msg.transform.translation.y  = msg.pose.position.y
        tf_msg.transform.translation.z  = msg.pose.position.z
        tf_msg.transform.rotation.x     = msg.pose.orientation.x
        tf_msg.transform.rotation.y     = msg.pose.orientation.y
        tf_msg.transform.rotation.z     = msg.pose.orientation.z
        tf_msg.transform.rotation.w     = msg.pose.orientation.w
        self.left_hand_tf_pub.publish(tf_msg)

    def head_cb(self, msg):
        """
        :type msg: PoseStamped
        """
        tf_msg                          = TransformStamped()
        tf_msg.header.frame_id          = "world"
        tf_msg.header.stamp             = msg.header.stamp
        tf_msg.child_frame_id           = "head"
        tf_msg.transform.translation.x  = msg.pose.position.x
        tf_msg.transform.translation.y  = msg.pose.position.y
        tf_msg.transform.translation.z  = msg.pose.position.z - 0.3  # about 3 centimeters from chest to head
        tf_msg.transform.rotation.x     = msg.pose.orientation.x
        tf_msg.transform.rotation.y     = msg.pose.orientation.y
        tf_msg.transform.rotation.z     = msg.pose.orientation.z
        tf_msg.transform.rotation.w     = msg.pose.orientation.w
        self.head_tf_pub.publish(tf_msg)

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
