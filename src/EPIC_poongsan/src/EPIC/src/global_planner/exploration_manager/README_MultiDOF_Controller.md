# MultiDOF Trajectory Controller 사용 가이드

## 개요
`MultiDOFJointTrajectory` 메시지를 사용하여 드론을 제어할 수 있는 통합 컨트롤러입니다.

## 특징
- **다중 인터페이스 지원**: MAVROS, SwarmTal, Quadrotor, PoseStamped
- **실시간 trajectory 실행**: 50Hz 제어 루프
- **완전한 6DOF 제어**: 위치, 속도, 가속도, 자세
- **시간 기반 trajectory 실행**: 정확한 시간 동기화

## 토픽 구조

### 입력 토픽
- `/planning/trajectory_multidof` (trajectory_msgs/MultiDOFJointTrajectory)
  - T초 간격으로 N초까지의 trajectory points
  - 각 point에는 position, velocity, acceleration 포함

### 출력 토픽 (설정에 따라)
- `/mavros/setpoint_raw/local` (mavros_msgs/PositionTarget) - MAVROS 제어
- `/drone_pos_ctrl_cmd` (swarmtal_msgs/drone_pos_ctrl_cmd) - SwarmTal 제어  
- `/position_cmd` (quadrotor_msgs/PositionCommand) - Quadrotor 제어
- `/mavros/setpoint_position/local` (geometry_msgs/PoseStamped) - 단순 pose 제어

## 실행 방법

### 1. 기본 실행
```bash
# 전체 시스템 실행
roslaunch epic_planner multidof_trajectory_control.launch

# 또는 개별 노드 실행
rosrun epic_planner multidof_trajectory_controller
rosrun epic_planner custom_trajectory_sampler
```

### 2. 설정 커스터마이징
```bash
# MAVROS 대신 SwarmTal 사용
roslaunch epic_planner multidof_trajectory_control.launch use_mavros:=false use_swarmtal:=true

# 제어 주파수 변경 (기본: 50Hz)
roslaunch epic_planner multidof_trajectory_control.launch control_frequency:=100.0

# 샘플링 간격 변경 (기본: 0.1s)
roslaunch epic_planner multidof_trajectory_control.launch sample_interval:=0.05
```

## 드론 제어 시나리오

### 시나리오 1: MAVROS 기반 드론 (PX4/ArduPilot)
```bash
# 1. MAVROS 연결 확인
rostopic echo /mavros/state

# 2. 컨트롤러 실행 (MAVROS 모드)
roslaunch epic_planner multidof_trajectory_control.launch use_mavros:=true

# 3. Exploration 시작 (trajectory 생성됨)
roslaunch epic_planner your_exploration.launch

# 4. 실시간 제어 상태 확인
rostopic echo /mavros/setpoint_raw/local
```

### 시나리오 2: SwarmTal 기반 드론
```bash
# 1. SwarmTal 시스템 확인
rostopic echo /drone_commander_state

# 2. 컨트롤러 실행 (SwarmTal 모드)  
roslaunch epic_planner multidof_trajectory_control.launch use_mavros:=false use_swarmtal:=true

# 3. 드론 arm & takeoff
rostopic pub /drone_onboard_command swarmtal_msgs/drone_onboard_command "..."

# 4. Exploration 시작
roslaunch epic_planner your_exploration.launch
```

### 시나리오 3: 시뮬레이션 (Quadrotor)
```bash
# 1. 시뮬레이션 시작
roslaunch your_sim sim.launch

# 2. 컨트롤러 실행 (Quadrotor 모드)
roslaunch epic_planner multidof_trajectory_control.launch use_mavros:=false use_quadrotor:=true

# 3. Exploration 시작
roslaunch epic_planner your_exploration.launch
```

## 모니터링 및 디버깅

### 토픽 모니터링
```bash
# MultiDOF trajectory 확인
rostopic echo /planning/trajectory_multidof

# 제어 명령 확인
rostopic echo /mavros/setpoint_raw/local    # MAVROS 모드
rostopic echo /drone_pos_ctrl_cmd           # SwarmTal 모드
rostopic echo /position_cmd                 # Quadrotor 모드

# 컨트롤러 상태 확인  
rostopic echo /multidof_trajectory_controller/status
```

### 로그 확인
```bash
# 컨트롤러 로그
rqt_console

# 또는 직접 로그 확인
rostopic echo /rosout | grep "MultiDOF Controller"
```

### RVIZ 시각화
```bash
# trajectory 시각화 추가
rosrun rviz rviz

# 추가할 토픽들:
# - /planning/trajectory_discrete (nav_msgs/Path)
# - /planning/trajectory_waypoints (visualization_msgs/MarkerArray)
# - /mavros/setpoint_raw/local (mavros_msgs/PositionTarget)
```

## 매개변수 조정

### 샘플링 매개변수
```yaml
# custom_trajectory_sampler 노드
sample_interval: 0.1    # 샘플링 간격 (초)
total_duration: 30.0    # 최대 지속시간 (초)  
auto_duration: true     # trajectory 전체 지속시간 사용
```

### 제어 매개변수
```yaml
# multidof_trajectory_controller 노드
use_mavros: true        # MAVROS 인터페이스 사용
use_swarmtal: false     # SwarmTal 인터페이스 사용
use_quadrotor: false    # Quadrotor 인터페이스 사용
use_pose_setpoint: false # 단순 pose 제어 사용
control_frequency: 50.0  # 제어 루프 주파수 (Hz)
```

## 트러블슈팅

### 1. 컨트롤러가 trajectory를 받지 못함
```bash
# 토픽 연결 확인
rostopic info /planning/trajectory_multidof

# Trajectory sampler 실행 확인
rosnode info custom_trajectory_sampler
```

### 2. 드론이 움직이지 않음
```bash
# 제어 명령 발행 확인
rostopic hz /mavros/setpoint_raw/local

# 드론 상태 확인 (MAVROS)
rostopic echo /mavros/state

# 또는 SwarmTal 상태 확인
rostopic echo /drone_commander_state
```

### 3. Trajectory 실행이 부정확함
```bash
# 시간 동기화 확인
rostopic echo /planning/trajectory_multidof | grep time_from_start

# 제어 주파수 확인
rosparam get /multidof_trajectory_controller/control_frequency
```

## 확장 가능성

### 새로운 드론 인터페이스 추가
1. `multidof_trajectory_controller.cpp`에 새로운 publisher 추가
2. `publishNewInterface()` 함수 구현
3. CMakeLists.txt와 package.xml에 의존성 추가

### 고급 제어 알고리즘
- PID 제어기 추가
- Model Predictive Control (MPC) 구현
- 장애물 회피 알고리즘 통합

이제 `MultiDOFJointTrajectory`를 사용해서 다양한 드론 플랫폼을 제어할 수 있습니다! 