#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
avoid_mux  (SIM 전용 브릿지)

실기에선 px4_ctrl_bridge 가 reactive-avoidance MUX 를 한다:
  /FSM_flag_avoidance==1 인 동안 EPIC 명령(/position_cmd) 대신
  회피 escape setpoint(/target_avoidance)를 PX4 로 보낸다.

sim 의 컨트롤러는 cascadePID 이고 quadrotor_msgs/PositionCommand 를 /planning/pos_cmd 로 받는다.
이 노드가 그 MUX 역할을 대신한다:
  flag==1 (fresh)  -> /target_avoidance(PositionTarget) -> PositionCommand 로 변환해 발행 (회피)
  그 외           -> 최신 EPIC /position_cmd 를 그대로 통과 (이륙 호버 + 탐색 궤적)
  -> /planning/pos_cmd -> cascadePID

회피 종료(1->0) 시의 "현재 위치 기준 재계획"은 EPIC FSM 의 avoidFlagCallback 이 이미 처리하므로
여기선 명령 라우팅만 담당한다. 실기에선 이 노드를 띄우지 않는다(px4_ctrl_bridge 가 함).
"""
import rospy
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Int16

READY = getattr(PositionCommand, "TRAJECTORY_STATUS_READY", 1)


class AvoidMux(object):
    def __init__(self):
        self.enable = bool(rospy.get_param("~enable_avoidance", True))
        self.flag_timeout = float(rospy.get_param("~flag_timeout", 0.5))
        self.avoid_timeout = float(rospy.get_param("~avoid_cmd_timeout", 0.3))
        rate = float(rospy.get_param("~rate", 100.0))
        epic_topic = rospy.get_param("~epic_topic", "/position_cmd")
        out_topic = rospy.get_param("~out_topic", "/planning/pos_cmd")

        self.last_epic = None
        self.last_avoid = None
        self.last_avoid_t = rospy.Time(0)
        self.flag = 0
        self.last_flag_t = rospy.Time(0)
        self._avoiding_prev = False

        self.pub = rospy.Publisher(out_topic, PositionCommand, queue_size=10)
        rospy.Subscriber(epic_topic, PositionCommand, self.epic_cb, queue_size=10)
        rospy.Subscriber("/target_avoidance", PositionTarget, self.avoid_cb, queue_size=10)
        rospy.Subscriber("/FSM_flag_avoidance", Int16, self.flag_cb, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        rospy.loginfo("[avoid_mux] enable=%s  %s (+/target_avoidance) -> %s",
                      self.enable, epic_topic, out_topic)

    def epic_cb(self, msg):
        self.last_epic = msg

    def avoid_cb(self, msg):
        self.last_avoid = msg
        self.last_avoid_t = rospy.Time.now()

    def flag_cb(self, msg):
        self.flag = int(msg.data)
        self.last_flag_t = rospy.Time.now()

    def _avoiding(self, now):
        return (self.enable
                and self.flag == 1
                and (now - self.last_flag_t).to_sec() < self.flag_timeout
                and self.last_avoid is not None
                and (now - self.last_avoid_t).to_sec() < self.avoid_timeout)

    def tick(self, _evt):
        now = rospy.Time.now()
        avoiding = self._avoiding(now)
        if avoiding != self._avoiding_prev:
            rospy.loginfo("[avoid_mux] avoidance %s", "ENGAGED" if avoiding else "released")
            self._avoiding_prev = avoiding

        if avoiding:
            a = self.last_avoid
            cmd = PositionCommand()
            cmd.header.stamp = now
            cmd.header.frame_id = "odom"
            cmd.position.x = a.position.x
            cmd.position.y = a.position.y
            cmd.position.z = a.position.z
            cmd.yaw = a.yaw
            cmd.trajectory_flag = READY
            self.pub.publish(cmd)
        elif self.last_epic is not None:
            self.pub.publish(self.last_epic)


if __name__ == "__main__":
    rospy.init_node("avoid_mux")
    AvoidMux()
    rospy.spin()
