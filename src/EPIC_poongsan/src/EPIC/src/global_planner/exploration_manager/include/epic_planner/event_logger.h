#pragma once
// =============================================================================
// EventLogger — EPIC 비행 이벤트 구조화 로거 (사람/AI 겸용, 변화 기반 dedup)
//
// 목표 (로그 체계 개편):
//  * 모든 이벤트 한 줄 형식:
//      [<ros epoch(s)>][+<트리거 후 상대시간>][<FSM state>] <CAT> <signature> | <detail>
//    - epoch 초(%.3f)는 rosbag 타임스탬프와 직접 대조 가능.
//  * 세 싱크에 동시 기록:
//      1) /epic/events (std_msgs/String)  -> rosbag(-a)에 기록 + record_on_goal이
//         <bag이름>.events.log 로 저장
//      2) /rosout (ROS_INFO/WARN/ERROR)   -> 터미널 + <bag이름>.log
//      3) /epic/session_info (latched)    -> 파라미터 스냅샷 헤더 (늦게 구독해도 수신)
//  * 반복 억제:
//      - signature(의미 내용)가 같으면 suppression 카운트만 증가, heartbeat(기본
//        30s)마다 "still ... (xN/Ys)" 한 줄만 발행.
//      - detail(수치)만 바뀐 경우 min_interval 로 코얼레싱(기본 즉시).
//      - A<->B 교대 패턴(리플랜 상태 플래핑 등)은 사이클로 감지해 억제하고
//        "cycling xN" heartbeat 만 발행. 패턴이 깨지면 요약을 붙여 새 이벤트 발행.
// =============================================================================

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cstdio>
#include <map>
#include <mutex>
#include <sstream>
#include <string>

namespace fast_planner {

class EventLogger {
public:
  enum Level { L_INFO = 0, L_WARN = 1, L_ERROR = 2 };

  void init(ros::NodeHandle &nh) {
    pub_ = nh.advertise<std_msgs::String>("/epic/events", 200);
    session_pub_ = nh.advertise<std_msgs::String>("/epic/session_info", 1, true /*latched*/);
    t0_ = ros::Time::now().toSec();
    inited_ = true;
  }

  // FSM 상태 문자열 (라인 프리픽스에 표기)
  void setState(const std::string &s) {
    std::lock_guard<std::mutex> lk(mtx_);
    state_ = s;
  }

  // 상대시간 기준점을 재설정 (트리거 시각 = 미션 t0)
  void markMissionStart() {
    std::lock_guard<std::mutex> lk(mtx_);
    t0_ = ros::Time::now().toSec();
  }

  // 파라미터 스냅샷 등 세션 메타데이터 (latched -> 늦은 구독자도 수신)
  void publishSessionInfo(const std::string &text) {
    std_msgs::String m;
    m.data = text;
    session_pub_.publish(m);
  }

  // 이벤트 발행.
  //   cat          : 채널 (STATE/GLOBAL/LOCAL/RTH/AVOID/PX4/BATT/STUCK/MISSION/EVENT/PARAM)
  //   sig          : 의미 내용(dedup 키). 바뀌면 반드시 발행.
  //   detail       : 수치 등 가변 정보. detail만 바뀌면 min_interval 코얼레싱.
  //   min_interval : detail-only 변화의 최소 발행 간격 [s]
  //   level        : 0=INFO 1=WARN 2=ERROR (rosout 레벨)
  //   force        : dedup 무시하고 무조건 발행 (PARAM 재덤프 등)
  void log(const std::string &cat, const std::string &sig, const std::string &detail = "",
           double min_interval = 0.0, int level = L_INFO, bool force = false) {
    if (!inited_)
      return;
    std::lock_guard<std::mutex> lk(mtx_);
    const double now = ros::Time::now().toSec();
    Chan &c = chans_[cat];

    if (force) {
      emitLine(cat, sig, detail, now, level);
      c = Chan();
      c.sig = sig;
      c.detail = detail;
      c.last_emit = now;
      return;
    }

    if (sig != c.sig) {
      // --- A<->B 교대(사이클) 감지: 새 sig가 직전-직전 sig와 같으면 플래핑 ---
      if (!c.prev_sig.empty() && sig == c.prev_sig) {
        c.cycles++;
        std::swap(c.sig, c.prev_sig); // c.sig <- 현재 sig
        c.detail = detail;
        if (now - c.last_emit >= heartbeat_) {
          emitLine(cat, c.sig + "  <->  " + c.prev_sig,
                   "cycling x" + std::to_string(c.cycles) + " over " +
                       fmt1(now - c.cycle_start) + "s",
                   now, level);
          c.last_emit = now;
        }
        return;
      }
      // --- 진짜 새로운 signature: 직전 억제 요약을 꼬리에 붙여 즉시 발행 ---
      std::string suffix;
      if (c.cycles > 0)
        suffix = " (prev cycling x" + std::to_string(c.cycles) + " over " +
                 fmt1(now - c.cycle_start) + "s)";
      else if (c.repeats > 0)
        suffix = " (prev repeated x" + std::to_string(c.repeats) + " over " +
                 fmt1(now - c.first_rep) + "s)";
      emitLine(cat, sig, detail + suffix, now, level);
      c.prev_sig = c.sig;
      c.sig = sig;
      c.detail = detail;
      c.repeats = 0;
      c.cycles = 0;
      c.cycle_start = now;
      c.first_rep = now;
      c.last_emit = now;
      return;
    }

    // --- signature 동일 ---
    if (detail != c.detail && (now - c.last_emit) >= min_interval) {
      std::string suffix =
          c.repeats > 0 ? " (repeated x" + std::to_string(c.repeats) + ")" : "";
      emitLine(cat, sig, detail + suffix, now, level);
      c.detail = detail;
      c.repeats = 0;
      c.first_rep = now;
      c.last_emit = now;
      return;
    }
    // 완전 반복(또는 코얼레싱 대기) -> 카운트 + heartbeat
    if (c.repeats == 0)
      c.first_rep = now;
    c.repeats++;
    if ((now - c.last_emit) >= heartbeat_) {
      emitLine(cat, sig,
               c.detail + " (still repeating x" + std::to_string(c.repeats) +
                   " over " + fmt1(now - c.first_rep) + "s)",
               now, level);
      c.last_emit = now;
      c.repeats = 0;
      c.first_rep = now;
    }
  }

private:
  struct Chan {
    std::string sig, prev_sig, detail;
    int repeats = 0; // 동일 이벤트 억제 횟수
    int cycles = 0;  // A<->B 교대 억제 횟수
    double first_rep = 0, cycle_start = 0, last_emit = -1e18;
  };

  static std::string fmt1(double v) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.1f", v);
    return b;
  }

  void emitLine(const std::string &cat, const std::string &sig, const std::string &detail,
                double now, int level) {
    char head[112];
    std::snprintf(head, sizeof(head), "[%.3f][%+9.1fs][%s] ", now, now - t0_, state_.c_str());
    std::string line = std::string(head) + cat + " " + sig;
    if (!detail.empty())
      line += " | " + detail;
    std_msgs::String m;
    m.data = line;
    pub_.publish(m);
    if (level >= L_ERROR)
      ROS_ERROR_STREAM("[EV] " << line);
    else if (level == L_WARN)
      ROS_WARN_STREAM("[EV] " << line);
    else
      ROS_INFO_STREAM("[EV] " << line);
  }

  ros::Publisher pub_, session_pub_;
  std::map<std::string, Chan> chans_;
  std::string state_ = "INIT";
  double t0_ = 0.0;
  double heartbeat_ = 30.0; // 반복 억제 중 생존신고 주기 [s]
  bool inited_ = false;
  std::mutex mtx_;
};

} // namespace fast_planner
