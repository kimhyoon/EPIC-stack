#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rviz_on_param — real.yaml 스위치로 rviz 를 켜고 끄는 래퍼.

roslaunch 의 <group if=...> 는 yaml 파라미터를 읽을 수 없으므로, 이 노드가
파라미터 서버에서 스위치를 읽고 rviz 를 subprocess 로 띄운다.
-> rviz on/off 도 real.yaml 한 파일에서 제어 (launch 인자 아님).

파라미터 — real.yaml(/exploration_node/rviz/*) 단일 소스:
  rviz/enable (bool) false(기본)면 노드가 뜨자마자 스스로 종료 (rviz 안 띄움).
  rviz/config (str)  rviz 설정 파일. 빈 값(기본)이면 rospkg 로 epic_planner
                     패키지 경로를 동적으로 찾아 <pkg>/config/real.rviz 사용.
"""
import os
import subprocess

import rospkg
import rospy

if __name__ == "__main__":
    rospy.init_node("rviz_on_param")
    if not rospy.get_param("/exploration_node/rviz/enable", False):
        rospy.loginfo("[rviz_on_param] rviz/enable=false (real.yaml) -> rviz off, node exits")
        raise SystemExit(0)

    cfg = rospy.get_param("/exploration_node/rviz/config", "")
    if not cfg:
        try:
            cfg = os.path.join(rospkg.RosPack().get_path("epic_planner"),
                               "config", "real.rviz")
        except Exception as e:
            rospy.logerr("[rviz_on_param] cannot resolve epic_planner path: %s", e)
            raise SystemExit(1)
    cfg = os.path.expanduser(cfg)
    if not os.path.isfile(cfg):
        rospy.logerr("[rviz_on_param] rviz config not found: %s", cfg)
        raise SystemExit(1)

    rospy.loginfo("[rviz_on_param] starting rviz -d %s", cfg)
    proc = subprocess.Popen(["rviz", "-d", cfg])

    def _shutdown():
        if proc.poll() is None:
            proc.terminate()

    rospy.on_shutdown(_shutdown)
    # rviz 가 죽어도(사용자가 창 닫음) launch 전체에는 영향 없음 — 그냥 종료.
    proc.wait()
