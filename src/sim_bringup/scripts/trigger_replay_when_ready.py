#!/usr/bin/env python3

"""Start EPIC after replayed odometry and cloud inputs are both available."""

import rospy
from rospy.msg import AnyMsg
from std_srvs.srv import Trigger


class ReplayTrigger:
    def __init__(self):
        self.odom_count = 0
        self.cloud_count = 0
        self.triggered = False
        self.minimum_samples = int(rospy.get_param("~minimum_samples", 10))
        odom_topic = rospy.get_param(
            "~odom_topic", "/quad_0/lidar_slam/odom")
        cloud_topic = rospy.get_param(
            "~cloud_topic", "/quad0_pcl_render_node/cloud_cropped")
        rospy.Subscriber(odom_topic, AnyMsg, self.odom_callback,
                         queue_size=10)
        rospy.Subscriber(cloud_topic, AnyMsg, self.cloud_callback,
                         queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

    def odom_callback(self, _msg):
        self.odom_count += 1

    def cloud_callback(self, _msg):
        self.cloud_count += 1

    def timer_callback(self, _event):
        if self.triggered:
            return
        if min(self.odom_count, self.cloud_count) < self.minimum_samples:
            return
        try:
            rospy.wait_for_service("/srv_start", timeout=0.1)
            response = rospy.ServiceProxy("/srv_start", Trigger)()
            if response.success:
                self.triggered = True
                rospy.logwarn(
                    "[TopologyReplay] EPIC started after %d odom and %d "
                    "cloud samples", self.odom_count, self.cloud_count)
        except (rospy.ROSException, rospy.ServiceException):
            return


if __name__ == "__main__":
    rospy.init_node("trigger_replay_when_ready")
    ReplayTrigger()
    rospy.spin()
