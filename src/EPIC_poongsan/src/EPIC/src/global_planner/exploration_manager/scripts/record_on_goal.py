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
        <name_prefix>_<ts>.bag            (rosbag record <record_args>)
        <name_prefix>_<ts>.params.yaml    (rosparam dump = 메타데이터)
        <name_prefix>_<ts>.all-rosout.log (/rosout_agg = ROS 콘솔 로그 "전체")
        <name_prefix>_<ts>.epic.log       (EPIC 상태 스냅샷 — 아래 세로 블록 형식)
        <name_prefix>_<ts>.events.log     (/epic/events = 구조화 이벤트 스트림; 헤더는
                                           latched /epic/session_info 파라미터 스냅샷)
    을 같은 basename 으로 저장.
    ※ .all-rosout.log 는 ROS_INFO/WARN/ERROR 등 ROS 로깅만. 순수 std::cout/printf 는
      담기지 않음 (그건 launch 를 tee 로 받아야 함).

  - .epic.log: /planning/expl_diag_kv (FSM 발행 key=value) + odom 을 조합해
    주기적으로 아래 "세로 블록"을 기록. 항목당 1줄 [key] value 형식이라
    나중에 grep '^\\[pos' 식으로 뽑아 plot 하기 쉽다.
        [time / flight_elapsed_s / mavros / pos / fsm / plan_result /
         clusters / viewpoints / vp_pipeline(하위 8단계) / frontier_cells /
         tsp_nodes / goal]
    pos 는 real.yaml odometry_topic 을 직접 구독한 최신 odom 좌표.
    기록 주기 = record/epic_period (기본 1s), FSM 상태·global 결과가 바뀌면 즉시.
  - FSM 상태가 LANDED 가 되면 녹화를 정상 마감. once:=false 면 이후 EPIC 재기동
    (/planning/state 재수신) 시 새 세션 폴더로 다시 녹화.
  - 노드 종료(roslaunch Ctrl-C) 시에도 rosbag 프로세스에 SIGINT 를 보내
    .bag.active -> .bag 로 정상 마감시킨다.

파라미터 — 전부 real.yaml(/exploration_node/record/*) 단일 소스 (launch 인자 아님):
  record/enable (bool) 전체 마스터. false(기본)면 노드가 뜨자마자 스스로 종료.
  record/dir    (str)  저장 폴더. 빈 값(기본)이면 rospkg 로 epic_planner 패키지
                       경로를 동적으로 찾아 <pkg>/records 사용 (없으면 자동 생성).
  record/args   (str)  rosbag record 인자   default: -a
  record/bag    (bool) .bag (rosbag record)                default: true
  record/log    (bool) .all-rosout.log (/rosout_agg 전체)   default: true
  record/epic   (bool) .epic.log (EPIC 상태 스냅샷)         default: true
  record/epic_period (double) .epic.log 기록 주기 [s]       default: 1.0
  record/params (bool) .params.yaml (rosparam dump)        default: true
  record/events (bool) .events.log (/epic/events)          default: true
(~private: ~name_prefix 파일 접두사 default epic, ~once 세션 1회만 default true)
"""

import os
import signal
import subprocess
from datetime import datetime

import rospkg
import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class RecordOnGoal(object):

    # rosgraph_msgs/Log level -> 이름
    LEVELS = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}

    def __init__(self):
        # ── 모든 스위치는 real.yaml 의 record/* (exploration_node ns) 단일 소스.
        #    launch 인자 없음 — 한 파일(real.yaml)에서 전부 끄고 켠다.
        if not rospy.get_param("/exploration_node/record/enable", False):
            rospy.loginfo("[record_on_goal] record/enable=false (real.yaml) "
                          "-> recording disabled, node exits")
            rospy.signal_shutdown("record disabled by real.yaml")
            self.disabled = True
            return
        self.disabled = False
        # 저장 폴더: 미지정(빈 값)이면 epic_planner 패키지 경로를 동적 탐지
        # -> 어느 PC 로 옮겨도 <패키지>/records 에 저장됨.
        rd = rospy.get_param("/exploration_node/record/dir", "")
        if not rd:
            try:
                rd = os.path.join(rospkg.RosPack().get_path("epic_planner"), "records")
            except Exception as e:
                rd = os.path.expanduser("~/records")
                rospy.logwarn("[record_on_goal] cannot resolve epic_planner path (%s) "
                              "-> fallback %s", e, rd)
        self.record_dir = os.path.expanduser(rd)
        self.record_args = rospy.get_param("/exploration_node/record/args", "-a")
        self.name_prefix = rospy.get_param("~name_prefix", "epic")
        self.once = rospy.get_param("~once", True)
        # 산출물별 개별 on/off
        self.save_bag = rospy.get_param("/exploration_node/record/bag", True)
        self.save_log = rospy.get_param("/exploration_node/record/log", True)
        self.save_epic = rospy.get_param("/exploration_node/record/epic", True)
        self.epic_period = rospy.get_param("/exploration_node/record/epic_period", 1.0)
        self.save_params = rospy.get_param("/exploration_node/record/params", True)
        self.save_events = rospy.get_param("/exploration_node/record/events", True)
        # 세션 종료(LANDED/셧다운) 시 plot_session.py 자동 실행 -> plots/ + report.html
        self.auto_plot = rospy.get_param("/exploration_node/record/auto_plot", True)
        if not (self.save_bag or self.save_log or self.save_epic or
                self.save_params or self.save_events):
            rospy.logwarn("[record_on_goal] all save_* flags are false -> nothing will be recorded")

        # epic.log 의 pos 는 real.yaml 의 odometry_topic 을 직접 구독 (폴백 없음 —
        # 키가 없으면 pos 는 '-' 로 남기고 나머지 항목만 기록).
        self.odom_topic = ""
        if self.save_epic:
            try:
                self.odom_topic = rospy.get_param("/exploration_node/odometry_topic")
            except KeyError:
                rospy.logerr("[record_on_goal] /exploration_node/odometry_topic not set "
                             "-> epic.log will have no pos")

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
        # ── epic.log 상태 ──
        self.epic_file = None
        self.diag_kv_sub = None
        self.odom_sub = None
        self.last_pos = None           # 최신 odom 좌표 (x, y, z)
        self.flight_t0 = None          # 비행 명령(트리거) 시각 = FSM 이 INIT/WAIT_TRIGGER 를 벗어난 순간
        self.last_epic_write = 0.0
        self.last_epic_sig = ""        # state+global 변화 감지용 (변하면 즉시 기록)
        self.fsm_state_txt = ""

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
                      "[bag=%d all-rosout=%d epic=%d params=%d events=%d]",
                      self.record_dir, self.save_bag, self.save_log,
                      self.save_epic, self.save_params, self.save_events)

    def state_cb(self, msg):
        txt = (msg.text or "").strip()
        if not txt:
            return
        self.fsm_state_txt = txt
        # "비행 명령후 경과 시간"의 t0: FSM 이 INIT/WAIT_TRIGGER 를 처음 벗어난 순간
        # (= 트리거 수신 직후 TAKEOFF_HOVER 또는 PLAN_TRAJ_EXP 진입).
        if self.flight_t0 is None and txt not in ("INIT", "WAIT_TRIGGER"):
            self.flight_t0 = rospy.get_time()
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
        self.session_dir = session_dir  # auto_plot 용
        try:
            os.makedirs(session_dir, exist_ok=True)
        except OSError as e:
            rospy.logerr("[record_on_goal] cannot create session dir '%s': %s",
                         session_dir, e)
            session_dir = self.record_dir  # 폴더 못 만들면 최상위에라도 저장
        base = os.path.join(session_dir, session)
        bag_path = base + ".bag"
        params_path = base + ".params.yaml"
        log_path = base + ".all-rosout.log"
        epic_path = base + ".epic.log"
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

        # 1.6) EPIC 상태 스냅샷 -> .epic.log (세로 블록, plot/grep 친화)
        if self.save_epic:
            try:
                self.epic_file = open(epic_path, "w")
                self.epic_file.write(
                    "# EPIC status snapshots for %s\n"
                    "# 블록 형식: 항목당 1줄 '[key] value'. 예) grep '^\\[pos' 로 좌표만 추출.\n"
                    "# 소스: /planning/expl_diag_kv (FSM) + %s (odom). 주기 %.1fs + 상태변화 즉시.\n"
                    % (os.path.basename(bag_path),
                       self.odom_topic if self.odom_topic else "(odom 없음)",
                       self.epic_period))
                self.epic_file.flush()
                if self.odom_topic:
                    self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry,
                                                     self.odom_cb, queue_size=10)
                self.diag_kv_sub = rospy.Subscriber("/planning/expl_diag_kv", String,
                                                    self.diag_kv_cb, queue_size=20)
                rospy.loginfo("[record_on_goal] epic log -> %s", epic_path)
            except Exception as e:
                rospy.logerr("[record_on_goal] cannot open epic log: %s", e)
                self.epic_file = None

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

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.last_pos = (p.x, p.y, p.z)

    def diag_kv_cb(self, msg):
        # FSM 이 5Hz 로 발행하는 "k=v;k=v;..." 진단 한 줄을 파싱해 세로 블록으로 기록.
        if self.epic_file is None:
            return
        kv = {}
        for tok in msg.data.split(";"):
            if "=" in tok:
                k, v = tok.split("=", 1)
                kv[k] = v
        now = rospy.get_time()
        sig = kv.get("state", "") + "|" + kv.get("global", "")
        # 주기 기록 + 상태/결과 변화 시 즉시 기록
        if (now - self.last_epic_write) < self.epic_period and sig == self.last_epic_sig:
            return
        self.last_epic_write = now
        self.last_epic_sig = sig

        flight_s = ("%.1f" % (now - self.flight_t0)) if self.flight_t0 is not None else "-"
        pos = ("%.2f %.2f %.2f" % self.last_pos) if self.last_pos is not None else "-"
        armed = "armed" if kv.get("armed", "0") == "1" else "disarmed"
        g = kv.get  # 짧은 별칭
        lines = [
            "-" * 60,
            "[time            ] %s" % g("t", "%.2f" % now),
            "[flight_elapsed_s] %s" % flight_s,
            "[mavros          ] %s (%s)" % (g("mode", "?"), armed),
            "[pos             ] %s" % pos,
            "[fsm             ] %s" % g("state", self.fsm_state_txt or "?"),
            "[plan_result     ] global=%s local=%s (plan %sms)"
            % (g("global", "?"), g("local", "?"), g("plan_ms", "?")),
            "[clusters        ] %s (reachable %s)"
            % (g("clusters", "?"), g("clusters_reach", "?")),
            "[viewpoints      ] %s (reachable %s)" % (g("vp", "?"), g("vp_reach", "?")),
            "[vp_pipeline     ]",
            "    [total           ] %s" % g("pipe_total", "?"),
            "    [dormant         ] %s" % g("pipe_dormant", "?"),
            "    [unreachable_pre ] %s" % g("pipe_unreachable_pre", "?"),
            "    [considered      ] %s" % g("pipe_considered", "?"),
            "    [no_candidates   ] %s" % g("pipe_no_candidates", "?"),
            "    [topo_unreachable] %s" % g("pipe_topo_unreachable", "?"),
            "    [no_visibility   ] %s" % g("pipe_no_visibility", "?"),
            "    [ok              ] %s" % g("pipe_ok", "?"),
            "[frontier_cells  ] %s" % g("frt_cells", "?"),
            "[tsp_nodes       ] %s (tour %s m)" % (g("tsp_nodes", "?"), g("tour_len", "?")),
            "[goal            ] %s (dist %s m)"
            % (g("goal", "?").replace(",", " "), g("goal_dist", "?")),
        ]
        try:
            self.epic_file.write("\n".join(lines) + "\n")
            self.epic_file.flush()
        except Exception:
            pass

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
        for sub_attr in ("rosout_sub", "events_sub", "session_sub",
                         "diag_kv_sub", "odom_sub"):
            sub = getattr(self, sub_attr)
            if sub is not None:
                try:
                    sub.unregister()
                except Exception:
                    pass
                setattr(self, sub_attr, None)
        for f_attr in ("log_file", "events_file", "epic_file"):
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
