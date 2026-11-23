# GC 6-DOF Robot Arm (ROS 2 Jazzy)

CubeMars AK45 + MKS SERVO42D/57D를 하나의 CAN 버스로 구동하는 6축 로봇팔.
설계 배경과 하드웨어 정리는 [`docs/project_overview.md`](docs/project_overview.md) 참고.

## 패키지

| 패키지 | 내용 | ROS 의존 |
|---|---|---|
| `gc_arm_drivers` | SocketCAN, AK(Servo/MIT)·MKS 프로토콜, 관절 단위 액추에이터, `ArmDriver`, 시뮬레이션 버스, `gc_motor_cli` | 없음 (순수 CMake로도 빌드) |
| `gc_arm_hardware` | ros2_control `SystemInterface` 플러그인 (`ArmDriver` 래퍼) | ros2_control |
| `gc_arm_description` | DH 기반 URDF(xacro), `config/joints.yaml`(관절·하드웨어 설정 단일 출처) | – |
| `gc_arm_bringup` | `controllers.yaml`, 기동 launch | – |

## 빌드

```bash
cd ~/GC_6-DOF_RobotArm          # 저장소 루트 = colcon 워크스페이스
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
colcon test --packages-select gc_arm_drivers && colcon test-result --verbose
source install/setup.bash
```

## 실행

```bash
# 1) URDF 확인 (슬라이더로 관절 움직이기)
ros2 launch gc_arm_description view_robot.launch.py

# 2) 하드웨어 없이 전 관절 mock
ros2 launch gc_arm_bringup gc_arm.launch.py mock_all:=true

# 3) 시뮬레이션 CAN 버스: joints.yaml에서 enabled: true인 관절이 실제 프로토콜 코드 경로로 동작
ros2 launch gc_arm_bringup gc_arm.launch.py can_interface:=sim

# 4) 실제 하드웨어: joints.yaml에서 연결한 관절만 enabled: true → 나머지는 mock
ros2 launch gc_arm_bringup gc_arm.launch.py
```

관절 움직여 보기:
```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
ros2 topic echo /dynamic_joint_states      # 전류·온도 포함
```

## 일부 모터만 연결된 상태로 시험하기

`src/gc_arm_description/config/joints.yaml`에서 실제로 버스에 연결한 관절만 `enabled: true`로 바꾼다.
예: AK45-36 2대(J2, J3) + SERVO42D 1대(J4) → `joint2`, `joint3`, `joint4`만 true.
나머지 관절은 mock으로 동작하므로 6축 컨트롤러/MoveIt을 그대로 쓸 수 있다.

## 안전 동작 (gc_arm_hardware)

- 활성화 시 모든 실제 관절의 첫 피드백을 기다리고, 명령 = 현재 위치로 초기화 (급동작 방지)
- 피드백 타임아웃(기본 100 ms), AK 드라이버 에러 코드, AK 과열(기본 80 °C), CAN BUS-OFF → `read()` 오류
  → controller_manager가 컨트롤러를 정지
- 명령 위치는 `joints.yaml`의 가동범위로 클램프, 모터 속도는 `max_velocity_deg_s` 이내로 제한
- 비활성화/오류 시 AK 토크를 해제하지 않음 (중력에 의한 낙하 방지) — 모터는 마지막 목표 위치 유지
- **소프트웨어 안전 기능은 하드웨어 E-stop을 대체하지 않는다**
