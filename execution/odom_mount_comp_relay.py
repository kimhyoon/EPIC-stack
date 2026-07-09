#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom_mount_comp_relay.py — /Odometry(FAST-LIO, 센서 자세) -> /mavros/odometry/out
라이다 장착각(전방 pitch) 보상 릴레이. 구 `topic_tools relay` 자리를 대체한다.

  - 자세만 변환: q_out = q_in ⊗ R_y(-pitch)  (센서 자세 -> 기체 자세)
    위치/스탬프/frame_id/공분산은 그대로. 원본 /Odometry 는 손대지 않으므로
    EPIC/crop 브릿지/rviz 가 보는 값은 불변 — PX4 로 가는 사본만 보상.
  - twist(linear/angular)도 기체 frame 으로 회전 (FAST-LIO 는 twist=0 이라
    실효 없음이지만 일관성 유지).
  - 이유: PX4 EKF2 EV 는 위치+yaw 를 융합하는데, 기울어진 센서 자세에서 yaw 를
    뽑으면 기동 중 roll 이 yaw 로 새는(×sin(pitch)) 오차가 생김 — 그것만 제거.

파라미터:
  ~mount_pitch_deg : 라이다 전방 기울임 각 [deg]. 0 = 무보상 passthrough(수평 장착).
                     4_tf_odom_relay.sh 상단 MOUNT_PITCH_DEG 에서 설정.
  ~in_topic        : 기본 /Odometry
  ~out_topic       : 기본 /mavros/odometry/out
"""
import math

import rospy
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("odom_mount_comp_relay")
    pitch_deg = float(rospy.get_param("~mount_pitch_deg", 0.0))
    in_topic = rospy.get_param("~in_topic", "/Odometry")
    out_topic = rospy.get_param("~out_topic", "/mavros/odometry/out")

    th = math.radians(pitch_deg)
    cw, sy = math.cos(-th / 2.0), math.sin(-th / 2.0)  # q_comp = R_y(-pitch)
    c, s = math.cos(th), math.sin(th)                  # v_body = R_y(+pitch) v_sensor

    pub = rospy.Publisher(out_topic, Odometry, queue_size=100)

    def rot_vec(v):
        x, z = v.x, v.z
        v.x = c * x + s * z
        v.z = -s * x + c * z

    def cb(m):
        q = m.pose.pose.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        # 해밀턴 곱 q_in ⊗ (cw, 0, sy, 0)
        q.w = w * cw - y * sy
        q.x = x * cw - z * sy
        q.y = w * sy + y * cw
        q.z = z * cw + x * sy
        rot_vec(m.twist.twist.linear)
        rot_vec(m.twist.twist.angular)
        pub.publish(m)

    rospy.Subscriber(in_topic, Odometry, cb, queue_size=100)
    rospy.loginfo("[odom_mount_comp] %s -> %s | mount_pitch=%.1f deg %s",
                  in_topic, out_topic, pitch_deg,
                  "(passthrough)" if pitch_deg == 0.0 else "(attitude compensated)")
    rospy.spin()


if __name__ == "__main__":
    main()
