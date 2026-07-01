#!/usr/bin/env python3
"""
record_on_goal.py

RViz '2D Nav Goal'(=/move_base_simple/goal) 이 들어오면 그 순간부터
`rosbag record` 를 시작하고, 동시에 현재 ROS 파라미터 서버 전체를
같은 이름의 .params.yaml 로 덤프해 메타데이터로 남긴다.

동작:
  - goal_topic 첫 메시지 수신 -> record_dir/<name_prefix>_<YYYY-mm-dd-HH-MM-SS>/ 폴더를 만들고
        <name_prefix>_<ts>.bag         (rosbag record <record_args>)
        <name_prefix>_<ts>.params.yaml (rosparam dump = 메타데이터)
        <name_prefix>_<ts>.log         (/rosout_agg = 콘솔 로그, log_console 시)
    세 파일을 그 폴더 안에 같은 basename 으로 저장.
    ※ .log 는 ROS_INFO/WARN/ERROR 등 ROS 로깅만. 순수 std::cout/printf 는 담기지 않음
      (그건 launch 를 tee 로 받아야 함).
  - once:=true 면 첫 goal 에서만 녹화 시작(이후 goal 무시).
  - 노드 종료(roslaunch Ctrl-C) 시 rosbag 프로세스에 SIGINT 를 보내
    .bag.active -> .bag 로 정상 마감시킨다.

파라미터(~private):
  ~record_dir   (str)  저장 폴더            default: /home/hmcl/records
  ~goal_topic   (str)  트리거 토픽          default: /move_base_simple/goal
  ~record_args  (str)  rosbag record 인자   default: -a
  ~name_prefix  (str)  파일 접두사          default: epic
  ~once         (bool) 첫 goal 만 트리거     default: true
  ~log_console  (bool) /rosout_agg 를 .log 로 저장  default: true
"""

import os
import signal
import subprocess
from datetime import datetime

import rospy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Log


class RecordOnGoal(object):

    # rosgraph_msgs/Log level -> 이름
    LEVELS = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}

    def __init__(self):
        self.record_dir = os.path.expanduser(
            rospy.get_param("~record_dir", "/home/hmcl/records"))
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.record_args = rospy.get_param("~record_args", "-a")
        self.name_prefix = rospy.get_param("~name_prefix", "epic")
        self.once = rospy.get_param("~once", True)
        # /rosout_agg(=ROS_INFO/WARN/... 콘솔 로그)를 bag 옆에 읽기 쉬운 .log 로도 저장
        self.log_console = rospy.get_param("~log_console", True)

        self.proc = None
        self.started = False
        self.log_file = None
        self.rosout_sub = None

        try:
            os.makedirs(self.record_dir, exist_ok=True)
        except OSError as e:
            rospy.logerr("[record_on_goal] cannot create record_dir '%s': %s",
                         self.record_dir, e)

        rospy.on_shutdown(self.stop_record)
        self.sub = rospy.Subscriber(self.goal_topic, PoseStamped,
                                    self.goal_cb, queue_size=1)
        rospy.loginfo("[record_on_goal] armed: waiting for goal on '%s' "
                      "-> will record (%s) into '%s'",
                      self.goal_topic, self.record_args, self.record_dir)

    def goal_cb(self, _msg):
        if self.once and self.started:
            return
        if self.proc is not None and self.proc.poll() is None:
            # 이미 녹화 중 (once=false 인데 아직 프로세스 살아있음)
            rospy.logwarn_throttle(5.0, "[record_on_goal] already recording, ignoring goal")
            return
        self.start_record()

    def start_record(self):
        self.started = True
        stamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        session = "%s_%s" % (self.name_prefix, stamp)
        # 세션마다 전용 폴더: record_dir/<prefix>_<ts>/<prefix>_<ts>.{bag,params.yaml,log}
        session_dir = os.path.join(self.record_dir, session)
        try:
            os.makedirs(session_dir, exist_ok=True)
        except OSError as e:
            rospy.logerr("[record_on_goal] cannot create session dir '%s': %s",
                         session_dir, e)
            session_dir = self.record_dir  # 폴더 못 만들면 최상위에라도 저장
        base = os.path.join(session_dir, session)
        bag_path = base + ".bag"
        params_path = base + ".params.yaml"
        log_path = base + ".log"

        # 1) 파라미터 서버 전체를 메타데이터로 덤프 (real.yaml 로딩 결과 포함)
        try:
            with open(params_path, "w") as f:
                f.write("# ROS param snapshot for %s\n" % os.path.basename(bag_path))
                f.write("# dumped at %s\n" % stamp)
                f.flush()
                subprocess.check_call(["rosparam", "dump"], stdout=f)
            rospy.loginfo("[record_on_goal] params dumped -> %s", params_path)
        except Exception as e:
            rospy.logerr("[record_on_goal] rosparam dump failed: %s", e)

        # 1.5) 콘솔 로그(/rosout_agg)를 읽기 쉬운 텍스트로 bag 옆에 저장.
        #      (ROS_INFO/WARN/ERROR 는 여기+bag 둘 다에. 순수 std::cout/printf 는 안 잡힘)
        if self.log_console:
            try:
                self.log_file = open(log_path, "w")
                self.log_file.write("# console (/rosout_agg) log for %s\n"
                                    % os.path.basename(bag_path))
                self.log_file.write("# started at %s\n" % stamp)
                self.log_file.flush()
                self.rosout_sub = rospy.Subscriber("/rosout_agg", Log,
                                                   self.rosout_cb, queue_size=100)
                rospy.loginfo("[record_on_goal] console log -> %s", log_path)
            except Exception as e:
                rospy.logerr("[record_on_goal] cannot open console log: %s", e)
                self.log_file = None

        # 2) rosbag record 시작 (새 프로세스 그룹으로 띄워 나중에 그룹 SIGINT)
        cmd = ["rosbag", "record"] + self.record_args.split() + \
              ["-O", bag_path, "__name:=epic_rosbag_recorder"]
        try:
            self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            rospy.loginfo("[record_on_goal] recording started (pid %d) -> %s",
                          self.proc.pid, bag_path)
        except Exception as e:
            rospy.logerr("[record_on_goal] failed to start rosbag: %s", e)
            self.proc = None

    def rosout_cb(self, msg):
        if self.log_file is None:
            return
        lvl = self.LEVELS.get(msg.level, "?")
        try:
            self.log_file.write("[%s] [%.3f] [%s]: %s\n"
                                % (lvl, msg.header.stamp.to_sec(), msg.name, msg.msg))
            self.log_file.flush()
        except Exception:
            pass

    def stop_record(self):
        if self.rosout_sub is not None:
            try:
                self.rosout_sub.unregister()
            except Exception:
                pass
            self.rosout_sub = None
        if self.log_file is not None:
            try:
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None

        if self.proc is None:
            return
        if self.proc.poll() is not None:
            return  # 이미 종료됨
        rospy.loginfo("[record_on_goal] stopping rosbag (SIGINT) for clean bag close...")
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=15)
        except Exception as e:
            rospy.logwarn("[record_on_goal] clean stop failed (%s), killing group", e)
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            except Exception:
                pass


if __name__ == "__main__":
    rospy.init_node("record_on_goal")
    RecordOnGoal()
    rospy.spin()
