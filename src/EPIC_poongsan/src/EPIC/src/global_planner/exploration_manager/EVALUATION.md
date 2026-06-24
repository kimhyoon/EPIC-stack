# Exploration Evaluation System

## Overview

EPIC 드론 자율 탐사 시스템의 성능을 정량적으로 평가하기 위한 시스템.
4가지 메트릭을 수집하고, 반복 실험을 자동화하며, 결과를 시각화한다.

## Evaluation Metrics

| # | Metric | Description | Data Source |
|---|--------|-------------|-------------|
| 1 | **RTH Origin Error** | FINISH 시점 위치와 trigger 시점 위치(원점) 간 유클리드 거리 | `/planning/state` + odometry |
| 2 | **Average Flight Speed** | trigger~FINISH 구간 odometry velocity magnitude 평균 | odometry `twist.twist.linear` |
| 3 | **Path Tracking RMSE** | commanded position vs actual position의 RMSE | `/planning/pos_cmd` + odometry |
| 4 | **Exploration Rate** | explored_cells / total_voxels 시계열 | `/frontier_manager/explored_cell_count` + garage.pcd |

## Key Parameters

### `garage.yaml`

```yaml
# Local planning 주기 제한 (Hz)
# 낮출수록 planning 빈도 감소, CPU 부하 감소
fsm/local_planning_max_hz: 10.0

# RTH 목표 도달 판정 거리 (m)
# 이 거리 이내에 도달하면 FINISH 상태로 전환
fsm/goal_tolerance: 0.2

# Frontier cell 크기 (m) - exploration rate 계산에 사용
FrontierManager/cell_size: 0.4
```

### `garage.launch` (Simulation)

```xml
<!-- Odometry publish rate (Hz) -->
<arg name="odom_publish_rate" default="10.0" />

<!-- Lidar sensing rate (Hz) -->
<arg name="sensing_rate" default="10.0" />
```

- `odom_publish_rate`: 시뮬레이션 dynamics 노드의 odometry 발행 주기. 높을수록 속도/추종 오차 샘플 증가
- `sensing_rate`: 라이다 포인트클라우드 발행 주기. 높을수록 매핑 빈도 증가

### `garage.launch` (Evaluation)

```xml
<!-- 평가 결과 저장 디렉토리 -->
<arg name="eval_output_dir" default="$(find epic_planner)/experiments" />

<!-- FINISH 후 evaluation 노드 자동 종료 (배치 실험용) -->
<arg name="eval_auto_shutdown" default="false" />
```

## Scripts

### `evaluate_exploration.py` - 평가 메트릭 수집 노드

ROS 노드로 실행되며, trigger~FINISH 구간의 메트릭을 수집하여 CSV로 저장한다.
`garage.launch`에 포함되어 자동 실행됨.

**출력 파일 (output_dir 내):**
- `summary.csv` - 최종 메트릭 1행 (duration, error, speed, rmse, exploration_rate, 위치 등)
- `speed_timeseries.csv` - (elapsed_time_s, speed_mps)
- `tracking_error_timeseries.csv` - (elapsed_time_s, tracking_error_m)
- `exploration_timeseries.csv` - (elapsed_time_s, explored_cells, exploration_rate)

### `run_batch_experiment.sh` - 반복 실험 자동화

```bash
# 사용법
./run_batch_experiment.sh [iterations] [rth_delay_sec]

# 10회 반복, 탐사 3분 후 RTH
./run_batch_experiment.sh 10 180

# 2회 반복, 탐사 10초 후 RTH (테스트)
./run_batch_experiment.sh 2 10
```

**자동화 흐름:**
1. `roslaunch epic_planner garage.launch` 실행
2. WAIT_TRIGGER 상태 대기
3. trigger 메시지 publish
4. `rth_delay_sec` 초 대기 (진행바 표시)
5. RTH 서비스 호출 (원점 5.0, 0.0, 2.0 으로 복귀)
6. FINISH 상태 대기
7. 결과 저장 후 종료
8. 다음 iteration 반복

**출력 디렉토리 구조:**
```
experiments/YYYYMMDD_HHMMSS/
  params.yaml              # 실험 파라미터 + garage.yaml 내용
  iter_01/
    summary.csv
    speed_timeseries.csv
    tracking_error_timeseries.csv
    exploration_timeseries.csv
    roslaunch.log
  iter_02/
    ...
```

### `plot_experiment_results.py` - 결과 시각화

```bash
# 가장 최근 실험 플롯
python3 plot_experiment_results.py

# 특정 실험 디렉토리 지정
python3 plot_experiment_results.py experiments/20260409_141912
```

**출력 (experiment 디렉토리 내):**
- `flight_speed.png` - 비행 속도 시계열 (개별 iteration 반투명 + 평균 진한선)
- `tracking_error.png` - 경로 추종 오차 시계열
- `exploration_rate.png` - 탐사율 시계열
- `rth_positions.png` - RTH 최종 위치 2D scatter (400mm threshold 원 포함)

## C++ Changes

### FrontierManager (`frontier_manager.h/cpp`)
- `explored_cell_count_pub_`: `/frontier_manager/explored_cell_count` (std_msgs/Int32) 토픽 publish
- `updateFrontierClusters()` 호출 시마다 `label_map_.size()` publish

### FSM (`fast_exploration_fsm.cpp`)
- RTH 모드에서도 `viz_pocc()`, `visfrtcluster()` 호출하여 RViz 시각화 유지
- `goal_tolerance` 기본값: 0.2m (파라미터로 변경 가능)
