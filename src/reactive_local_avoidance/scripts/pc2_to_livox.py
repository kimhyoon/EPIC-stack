#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pc2_to_livox  (SIM 전용 브릿지)

MARSIM 은 LiDAR 스캔을 sensor_msgs/PointCloud2 (world 프레임)로 발행하지만,
reactive local_avoidance 노드는 livox_ros_driver2/CustomMsg (센서/바디 프레임)를 기대한다.
이 노드가 그 간극을 메운다:
  /quad0_pcl_render_node/cloud (PointCloud2, world)  +  odom
    -> world->body 변환 + 다운샘플 -> /livox/lidar (CustomMsg)

avoidance 노드는 점의 x,y,z 만 사용하므로 offset_time/reflectivity/tag/line 은 0 으로 채운다.
실기(real_flight.launch)에선 실제 /livox/lidar 가 있으므로 이 노드는 띄우지 않는다.
"""
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from tf.transformations import quaternion_matrix


class Pc2ToLivox(object):
    def __init__(self):
        self.max_points = int(rospy.get_param("~max_points", 4000))
        self.max_range = float(rospy.get_param("~max_range", 20.0))
        cloud_topic = rospy.get_param("~cloud_topic", "/quad0_pcl_render_node/cloud")
        odom_topic = rospy.get_param("~odom_topic", "/quad_0/lidar_slam/odom")
        livox_topic = rospy.get_param("~livox_topic", "/livox/lidar")

        self.have_odom = False
        self.R = np.eye(3)          # body -> world rotation
        self.t = np.zeros(3)        # drone position in world

        self.pub = rospy.Publisher(livox_topic, CustomMsg, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber(cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)
        rospy.loginfo("[pc2_to_livox] %s + %s -> %s", cloud_topic, odom_topic, livox_topic)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.t = np.array([p.x, p.y, p.z])
        self.R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        self.have_odom = True

    def cloud_cb(self, msg):
        if not self.have_odom:
            return
        pts = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float64,
        )
        if pts.shape[0] == 0:
            return
        # world -> body :  p_body = R^T (p_world - t)  ==  (p_world - t) @ R
        body = (pts - self.t) @ self.R
        # keep only nearby points (avoidance only cares about close obstacles)
        d = np.linalg.norm(body, axis=1)
        body = body[d < self.max_range]
        if body.shape[0] == 0:
            return
        # downsample to a cap (avoidance uses a potential field; density not critical)
        if body.shape[0] > self.max_points:
            stride = int(np.ceil(body.shape[0] / float(self.max_points)))
            body = body[::stride]

        out = CustomMsg()
        out.header = msg.header
        out.header.frame_id = "livox"
        out.timebase = 0
        out.lidar_id = 0
        pts_list = []
        for i in range(body.shape[0]):
            cp = CustomPoint()
            cp.offset_time = 0
            cp.x = float(body[i, 0])
            cp.y = float(body[i, 1])
            cp.z = float(body[i, 2])
            cp.reflectivity = 0
            cp.tag = 0
            cp.line = 0
            pts_list.append(cp)
        out.points = pts_list
        out.point_num = len(pts_list)
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("pc2_to_livox")
    Pc2ToLivox()
    rospy.spin()
