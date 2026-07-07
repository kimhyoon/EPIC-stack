# EPIC 비행 로그 읽는 법 (LOG_GUIDE)

`real_flight.launch` / `replay_tuning.launch` 실행 시 세션마다
`<epic_planner 패키지>/records/<접두사>_<날짜-시각>/` 폴더에 다음 4개 파일이 생성된다
(접두사: 실비행 `epic_`, 재생튜닝 `replay_`).

| 파일 | 내용 | 켜고 끄기 (launch 인자) |
|---|---|---|
| `*.bag` | 전체 토픽 원본 (`rosbag record -a`) | `record_bag` |
| `*.log` | **모든 노드**의 콘솔 로그(/rosout_agg): mavros, 브리지, 회피, EPIC 등 전부 | `record_log` |
| `*.events.log` | **EPIC이 발행한 선별 이벤트만** (아래 상세) — 분석은 보통 이것부터 | `record_events` |
| `*.params.yaml` | 세션 시작 시점 rosparam 전체 스냅샷 | `record_params` |

- 녹화 시작 = EPIC FSM 기동(/planning/state 첫 발행, INIT 상태부터).
- 녹화 종료 = FSM 상태가 `LANDED`(착지+disarm)가 되는 순간, 또는 launch 종료(Ctrl-C).
- `.events.log`의 모든 줄은 `[EV]` 접두사로 `.log`에도 포함된다(부분집합).

---

## 1. events.log 줄 형식

```
[ros_epoch][+상대시각][FSM상태] 카테고리 시그니처 | 상세
```

| 부분 | 예 | 의미 |
|---|---|---|
| `[1782989149.277]` | ROS epoch 초 | **bag 타임스탬프와 동일 축** — rosbag/CSV와 1:1 대조 |
| `[+7.9s]` | 상대시각 | 트리거(2D Nav Goal) 시점 = 0초 |
| `[PLAN_TRAJ_EXP]` | FSM 상태 | 발행 순간의 상태 |
| `GLOBAL ...` | 카테고리+내용 | 아래 카테고리별 참조 |

파일 머리의 `#` 주석 = 세션 파라미터 스냅샷 (latched /epic/session_info).

### FSM 상태 목록
`INIT` → `WAIT_TRIGGER` → `TAKEOFF_HOVER` → `PLAN_TRAJ_EXP` ↔ `EXEC_TRAJ` (탐사 리플랜 사이클)
→ `FINISH` → `PLAN_TRAJ_RTH` ↔ `EXEC_TRAJ` (귀환) → `LAND` → `LANDED`.
`CAUTION` = 위험 상황 탈출 중, `PLAN_TRAJ_RTH`는 /srv_rth 수동 호출로도 진입.

---

## 2. 카테고리별 필드

### STATE — FSM 상태 전이
```
STATE PLAN_TRAJ_EXP -> EXEC_TRAJ [plan success: new traj pub]
```
`이전상태 -> 새상태 [전이 사유]`. PLAN↔EXEC 리플랜 플래핑은 사이클 억제됨(§4).

### GLOBAL — 글로벌 탐사/RTH 계획 결과 (~5Hz, 변화 시만 기록)
```
GLOBAL result=OK | clusters=12(reachable 5) viewpoints=5(path_reachable 4)
  pipeline[total=12 dormant=4 prev_unreachable=0 evaluated=8 no_candidate=0
           topo_unreachable=3 no_visibility=0 survived=5]
  tour=5_nodes/33.1m next_goal=(2.4, 12.8, 0.5) plan_time=70ms
```
| 필드 | 의미 |
|---|---|
| `result` | `OK` / `NO_VIEWPOINTS`(뷰포인트 0) / `NO_REACHABLE_VP`(있는데 전부 도달불가) / `RTH_PATH_OK` / `RTH_ASTAR_FAIL` / `RTH_FALLBACK_FRONTIER` / `RTH_NO_FRONTIER` / `RTH_FAIL`. 실패 시 `why:` 사유가 붙음 |
| `clusters=N(reachable M)` | 프론티어 클러스터 총 N개(rviz의 박스), 이번 라운드 도달가능+활성 M개 |
| `viewpoints=N(path_reachable M)` | 생성된 뷰포인트 N개, 현재 위치에서 경로비용 검사 통과 M개 (TSP 입력) |
| `pipeline[...]` | **클러스터가 어느 관문에서 탈락했는지** — §3 참조 |
| `tour=N_nodes/L m` | TSP 투어 노드 수 / 총 길이 |
| `next_goal` | 다음 목표 좌표 (드론이 향하는 곳) |
| `plan_time` | 이번 글로벌 계획 소요시간 |

### LOCAL — 로컬 궤적(MINCO) 생성 결과
```
LOCAL explore OK | plan_time=33.4ms goal_dist=6.8m
LOCAL RTH FAIL | why: start not in corridor (predicted start; ...) | plan_time=33.4ms
```
`explore`/`RTH` = 어느 모드의 로컬 계획인지. FAIL 시 `why:`에 planner 내부 사유
(`start not in corridor`, `corridor pieces < 2`, `minco optimize failed`,
`fast-searcher: no path odom->goal`, `escape traj (flyToSafeRegion)` 등).

### RTH — 귀환 진행 (0.5m 버킷 변화 시만)
```
RTH dist_to_goal=3.0m | auto-home(xy) dist=3.04m tolerance=0.20m goal=(-0.01, -0.02, 1.10)
```
`auto-home(xy)` = 탐사종료 자동귀환(xy거리 판정) / `srv-goal(3D)` = /srv_rth 수동 목표(3D 판정).

### PX4 — 모드/시동 변화 (변화 시만; 사고분석 1급 정보)
```
PX4 mode=OFFBOARD armed=1 | prev=POSCTL
```
미션 중 OFFBOARD 이탈/disarm은 WARN으로 격상되어 `(LEFT OFFBOARD: ...)`가 붙는다.
**세션 내내 PX4 변화 이벤트가 없다 = OFFBOARD에 못 들어갔다는 뜻** (지상 테스트 판별법).

### Avoidance: — 반사회피 상태 (에지 + 기동/트리거 시 스냅샷)
```
Avoidance: Deactivated                       ← 기동/트리거 시 현재 상태 스냅샷
Avoidance: Activated | obstacle close, reactive layer overrides cmd | pos=(-0.03, 0.03, 0.06)
Avoidance: Deactivated | released, replan from current pose | duration=1.9s
Avoidance: Disabled (local_avoidance/enable=false)   ← 마스터 스위치 꺼짐
```
Activated 동안 브리지 MUX가 EPIC 명령 대신 /target_avoidance 를 PX4로 보낸다.
상태 지속 중 반복 발행은 없음(에지에서만), 빠른 플래핑은 사이클 억제.

### STUCK — 무이동 감시 (ERROR)
```
STUCK no motion >8s in PLAN_TRAJ_EXP | last-local: OK | pos=(...) stationary_for=13s
```
미션 상태에서 8초 이상 0.3m 미만 이동 시. `last-local`이 OK인데 STUCK이면
"계획은 되는데 기체가 안 따라옴" = OFFBOARD/시동/조종 문제부터 의심.

### MISSION — 미션 이정표
```
MISSION FINISH (premature? unreached clusters remain) | pos=(...) elapsed=14s clusters_left=1(reachable 0) pipeline[...]
MISSION LANDED (mission complete, disarmed)
```
`premature?` = 클러스터가 남았는데 종료(조기 종료 의심). pipeline으로 원인 단계 확인.

### BATT — 배터리 (0.5V 버킷 변화 시만; 경고전압 미만 WARN)
```
BATT 23.0V | voltage=23.10V percent=52%
```
전압이 비행 내내 미동도 없으면 모터 무부하(=실제 비행 아님) 신호로도 쓸 수 있다.

### PARAM / EVENT
`PARAM box|corridor|frontier|viewpoint|fsm|topics ...` = 핵심 파라미터 스냅샷
(기동 시 1회 + 트리거 시 재발행). `EVENT` = 트리거 수신, /srv_rth 수신.

---

## 3. pipeline[...] 읽기 — "왜 탐사를 안 가/멈추나"의 핵심

`generateTSPViewpoints()`의 클러스터 생존 파이프라인. 검산:
`dormant + prev_unreachable + evaluated = total`,
`no_candidate + topo_unreachable + no_visibility + survived = evaluated`.

```
total=12          프론티어 클러스터 전체
├─ dormant=4            휴면 (이미 관측완료 판정) — 정상 탈락
├─ prev_unreachable=0   이전 라운드 "도달불가" 낙인 → 재평가 없이 스킵
└─ evaluated=8          이번 라운드 평가 대상
    ├─ no_candidate=0        뷰포인트 후보 0 (벽 clearance / box z범위 / topo-region 없음)
    ├─ topo_unreachable=3    토포그래프 경로 도달성 실패
    ├─ no_visibility=0       뷰포인트에서 프론티어 셀 가시성 부족
    └─ survived=5            생존 → viewpoints 개수
```

### 어느 숫자가 크면 어떤 파라미터를 보나

| 증상 | 병목 | 1순위 파라미터 (real.yaml) |
|---|---|---|
| `topo_unreachable` ↑ | 경로 도달성 | `bubble_astar/safe_distance` (경로+끝점 이격 요구), `bubble_topo/bubble_min_radius` (스켈레톤 확장). **주의: `min_obstacle_clearance` < `safe_distance`면 그 사이 밴드의 후보는 무조건 여기서 죽음** — 항상 `clearance > safe_distance` 유지 |
| `no_candidate` ↑ | 후보 생성 | `ViewpointManager/min_obstacle_clearance`(낮추면 후보↑), `box_0` z범위, 샘플링(`sample_pillar_*`, `circle_sample_num`) |
| `no_visibility` ↑ | 가시성 | `lidar_perception/fov_*`, `good_observation_direction_score`, 샘플 각도수 |
| `prev_unreachable` 누적 ↑ | 영구 낙인 | 파라미터로 못 풀음 — 일시적 topo 실패가 영구화되는 코드 동작 (재평가/에이징 코드 수정 필요) |
| `dormant`만 남고 survived=0 | 진짜 탐사 완료 | 정상 종료 |

권장 안전거리 사슬: `DilateRadiusHard < avoidance_trigger_m < safe_distance < min_obstacle_clearance`.

---

## 4. 반복 억제 표기 (스팸 방지)

| 표기 | 의미 |
|---|---|
| `(repeated x19)` | 직전까지 동일 이벤트 19건 생략 후 값 갱신 발행 |
| `(prev repeated x19 over 0.2s)` | 새 내용 발행 시, 직전 동일 이벤트 19건/0.2초 생략됐었음 |
| `(prev cycling x16 over 5.1s)` | A↔B 교대 패턴(리플랜 플래핑 등) 16회 생략 후 패턴 이탈 |
| `A <-> B | cycling x42 over 30.0s` | 교대 패턴 지속 중 30초 주기 생존신고 |
| `(still repeating x500 over 5.0s)` | 동일 이벤트 지속 중 주기 생존신고 |

---

## 5. 분석 순서 요령

1. `events.log`를 위에서 아래로 — STATE 흐름이 정상 시나리오(§1 상태 목록)를 따르는지.
2. 이상 지점의 `GLOBAL`/`LOCAL` `why:`와 `pipeline[]`으로 원인 단계 특정.
3. `PX4` 변화 유무로 OFFBOARD/시동 여부 확인 (없으면 지상/수동 테스트).
4. 디테일 필요 시 같은 epoch로 `.log`(타 노드 메시지) → `.bag`(메시지 단위) 순으로 내려감.
5. 파라미터 확인은 파일 머리 `#` 스냅샷 또는 `params.yaml`.
