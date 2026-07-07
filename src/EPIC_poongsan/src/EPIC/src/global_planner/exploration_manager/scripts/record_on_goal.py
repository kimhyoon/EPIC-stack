#!/usr/bin/env python3
"""
record_on_goal.py

EPIC FSM 이 살아있는 것(/planning/state 첫 수신 = INIT 상태부터)을 감지하면
그 즉시 `rosbag record` 를 시작하고, 현재 ROS 파라미터 서버 전체를
같은 이름의 .params.yaml 로 덤프한다.

  * 시작: /planning/state 첫 메시지 (EPIC 기동 = FSM INIT 부터 바로 기록)
  * 종료: FSM 상태가 LANDED 가 되는 순간 (RTH -> LAND -> 착지+disarm 후 전이)
          bag 을 정상 마감(.bag.active -> .bag)하고 로그 파일을 닫는다.
          LANDED 에 도달하지 못한 세션(조종자 개입 등)은 launch 종료 시 마감.

동작:
  - /planning/state 첫 수신(EPIC FSM 기동) -> record_dir/<name_prefix>_<ts>/ 폴더에
        <name_prefix>_<ts>.bag         (rosbag record <record_args>)
        <name_prefix>_<ts>.params.yaml (rosparam dump = 메타데이터)
        <name_prefix>_<ts>.log         (/rosout_agg = 콘솔 로그, log_console 시)
        <name_prefix>_<ts>.events.log  (/epic/events = 구조화 이벤트 스트림; 헤더는
                                        latched /epic/session_info 파라미터 스냅샷)
    네 파일을 같은 basename 으로 저장.
    ※ .log 는 ROS_INFO/WARN/ERROR 등 ROS 로깅만. 순수 std::cout/printf 는 담기지 않음
      (그건 launch 를 tee 로 받아야 함).
  - FSM 상태가 LANDED 가 되면 녹화를 정상 마감. once:=false 면 이후 EPIC 재기동
    (/planning/state 재수신) 시 새 세션 폴더로 다시 녹화.
  - 노드 종료(roslaunch Ctrl-C) 시에도 rosbag 프로세스에 SIGINT 를 보내
    .bag.active -> .bag 로 정상 마감시킨다.

파라미터(~private):
  ~record_dir   (str)  저장 폴더. 빈 값(기본)이면 rospkg 로 epic_planner 패키지
                       경로를 동적으로 찾아 <pkg>/records 사용.
  ~record_args  (str)  rosbag record 인자   default: -a
  ~name_prefix  (str)  파일 접두사          default: epic
  ~once         (bool) 세션 1회만 녹화       default: true
  --- 산출물별 개별 on/off (launch 인자로 노출) ---
  ~save_bag     (bool) .bag (rosbag record)          default: true
  ~save_log     (bool) .log (/rosout_agg 콘솔 로그)   default: true
  ~save_params  (bool) .params.yaml (rosparam dump)  default: true
  ~save_events  (bool) .events.log (/epic/events)    default: true
"""

import os
import signal
import subprocess
from datetime import datetime

import rospkg
import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class RecordOnGoal(object):

    # rosgraph_msgs/Log level -> 이름
    LEVELS = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}

    def __init__(self):
        # 저장 폴더: 파라미터 미지정(빈 값)이면 epic_planner 패키지 경로를 동적 탐지
        # -> 어느 PC 로 옮겨도 <패키지>/records 에 저장됨.
        rd = rospy.get_param("~record_dir", "")
        if not rd:
            try:
                rd = os.path.join(rospkg.RosPack().get_path("epic_planner"), "records")
            except Exception as e:
                rd = os.path.expanduser("~/records")
                rospy.logwarn("[record_on_goal] cannot resolve epic_planner path (%s) "
                              "-> fallback %s", e, rd)
        self.record_dir = os.path.expanduser(rd)
        self.record_args = rospy.get_param("~record_args", "-a")
        self.name_prefix = rospy.get_param("~name_prefix", "epic")
        self.once = rospy.get_param("~once", True)
        # 산출물별 개별 on/off
        self.save_bag = rospy.get_param("~save_bag", True)
        self.save_log = rospy.get_param("~save_log", True)
        self.save_params = rospy.get_param("~save_params", True)
        self.save_events = rospy.get_param("~save_events", True)
        # 세션 종료(LANDED/셧다운) 시 plot_session.py 자동 실행 -> plots/ + report.html
        self.auto_plot = rospy.get_param("~auto_plot", True)
        if not (self.save_bag or self.save_log or self.save_params or self.save_events):
            rospy.logwarn("[record_on_goal] all save_* flags are false -> nothing will be recorded")

        self.proc = None
        self.started = False
        self.session_active = False  # (bag 비활성 시에도) 세션 진행 중 여부
        self.session_dir = None      # 현재 세션 폴더 (auto_plot 용)
        self.log_file = None
        self.rosout_sub = None
        self.events_file = None
        self.events_sub = None
        self.session_sub = None
        self.session_written = False

        try:
            os.makedirs(self.record_dir, exist_ok=True)
        except OSError as e:
            rospy.logerr("[record_on_goal] cannot create record_dir '%s': %s",
                         self.record_dir, e)

        rospy.on_shutdown(self.stop_record)
        # 시작/종료 신호원: FSM 상태 마커. 첫 수신(INIT 포함) = EPIC 기동 -> 녹화 시작,
        # text == "LANDED" -> 녹화 정상 마감.
        self.state_sub = rospy.Subscriber("/planning/state", Marker,
                                          self.state_cb, queue_size=5)
        rospy.loginfo("[record_on_goal] armed: start on EPIC FSM up "
                      "(/planning/state), stop on state LANDED -> save into '%s' "
                      "[bag=%d log=%d params=%d events=%d]", self.record_dir,
                      self.save_bag, self.save_log, self.save_params,
                      self.save_events)

    def state_cb(self, msg):
        txt = (msg.text or "").strip()
        if not txt:
            return
        if not self.session_active:
            if self.once and self.started:
                return
            rospy.loginfo("[record_on_goal] EPIC FSM up (state=%s) -> start recording",
                          txt)
            self.start_record()
        elif txt == "LANDED":
            rospy.loginfo("[record_on_goal] FSM state LANDED -> finalize recording")
            self.stop_record()
            if not self.once:
                self.started = False  # 다음 EPIC 세션에서 재무장

    def start_record(self):
        self.started = True
        self.session_active = True
        self.session_written = False  # 새 세션의 events.log 헤더 재기록 허용
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
        self.session_dir = session_dir
        base = os.path.join(session_dir, session)
        bag_path = base + ".bag"
        params_path = base + ".params.yaml"
        log_path = base + ".log"
        events_path = base + ".events.log"

        # 1) 파라미터 서버 전체를 메타데이터로 덤프 (real.yaml 로딩 결과 포함)
        if self.save_params:
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
        if self.save_log:
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

        # 1.7) 구조화 이벤트 스트림(/epic/events) -> .events.log
        #      헤더: latched /epic/session_info (파라미터 스냅샷) 를 첫 수신 시 기록.
        if self.save_events:
            try:
                self.events_file = open(events_path, "w")
                self.events_file.write("# EPIC structured flight events for %s\n"
                                       % os.path.basename(bag_path))
                self.events_file.write("# line format: [ros_epoch][+rel_s][FSM_STATE] "
                                       "CATEGORY signature | detail\n")
                self.events_file.flush()
                self.session_sub = rospy.Subscriber("/epic/session_info", String,
                                                    self.session_cb, queue_size=2)
                self.events_sub = rospy.Subscriber("/epic/events", String,
                                                   self.event_cb, queue_size=200)
                rospy.loginfo("[record_on_goal] event log -> %s", events_path)
            except Exception as e:
                rospy.logerr("[record_on_goal] cannot open events log: %s", e)
                self.events_file = None

        # 2) rosbag record 시작 (새 프로세스 그룹으로 띄워 나중에 그룹 SIGINT)
        if self.save_bag:
            cmd = ["rosbag", "record"] + self.record_args.split() + \
                  ["-O", bag_path, "__name:=epic_rosbag_recorder"]
            try:
                self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
                rospy.loginfo("[record_on_goal] recording started (pid %d) -> %s",
                              self.proc.pid, bag_path)
            except Exception as e:
                rospy.logerr("[record_on_goal] failed to start rosbag: %s", e)
                self.proc = None

    def session_cb(self, msg):
        # latched 파라미터 스냅샷 -> events.log 머리에 주석으로 1회 기록
        if self.events_file is None or self.session_written:
            return
        self.session_written = True
        try:
            for ln in msg.data.rstrip("\n").split("\n"):
                self.events_file.write("# " + ln + "\n")
            self.events_file.flush()
        except Exception:
            pass

    def event_cb(self, msg):
        if self.events_file is None:
            return
        try:
            self.events_file.write(msg.data + "\n")
            self.events_file.flush()
        except Exception:
            pass

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
        self.session_active = False
        for sub_attr in ("rosout_sub", "events_sub", "session_sub"):
            sub = getattr(self, sub_attr)
            if sub is not None:
                try:
                    sub.unregister()
                except Exception:
                    pass
                setattr(self, sub_attr, None)
        for f_attr in ("log_file", "events_file"):
            f = getattr(self, f_attr)
            if f is not None:
                try:
                    f.close()
                except Exception:
                    pass
                setattr(self, f_attr, None)

        if self.proc is not None and self.proc.poll() is None:
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
        self.proc = None

        # 세션 산출물이 닫힌 뒤 plot 자동 생성 (별도 프로세스, 논블로킹)
        if self.auto_plot and self.started and self.session_dir:
            script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "plot_session.py")
            try:
                subprocess.Popen(["python3", script, self.session_dir],
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                rospy.loginfo("[record_on_goal] auto-plot started -> %s/plots, report.html",
                              self.session_dir)
            except Exception as e:
                rospy.logwarn("[record_on_goal] auto-plot launch failed: %s", e)
            self.session_dir = None  # 세션당 1회만


if __name__ == "__main__":
    rospy.init_node("record_on_goal")
    RecordOnGoal()
    rospy.spin()
