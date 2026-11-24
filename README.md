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
| `gc_arm_moveit_config` | SRDF(그룹·자세·충돌 제외), KDL, OMPL, 컨트롤러 연동, MoveIt launch | MoveIt 2 |

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

MoveIt (RViz에서 마커를 끌어 목표 지정 → Plan & Execute):
```bash
ros2 launch gc_arm_moveit_config moveit.launch.py mock_all:=true
```
- 이름 붙은 자세: `zero`(수평 신전), `upright`(수직, 중력 부하 최소), `ready`(L자, 툴 아래)
- 기본 속도/가속도 스케일 0.1 — 실기 초기 시험은 이 값 유지
- 작업대 상판(base 바닥면 높이)이 충돌 객체로 포함되어 있어 MoveIt은 작업대를 관통하는 경로를 만들지 않음

관절 움직여 보기:
```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
ros2 topic echo /dynamic_joint_states      # 전류·온도 포함
```

## 일부 모터만 연결된 상태로 시험하기

`src/gc_arm_description/config/joints.yaml`에서 실제로 버스에 연결한 관절만 `enabled: true`로 바꾼다.
예: AK45-36 2대(J2, J3) + SERVO42D 1대(J4) → `joint2`, `joint3`, `joint4`만 true.
나머지 관절은 mock으로 동작하므로 6축 컨트롤러/MoveIt을 그대로 쓸 수 있다.

## CAD 메시

`src/gc_arm_description/meshes/visual/*.stl` (CAD 원본, mm) + `meshes/collision/*.stl` (볼록 껍질).
CAD는 조립품 공통 좌표계·upright 자세로 내보낸 것이며, 배치는 `config/meshes.yaml`의 `cad` 항목으로 계산한다.

CAD를 다시 내보냈거나 파트를 추가했을 때:
```bash
cp <새 STL> src/gc_arm_description/meshes/visual/
python3 src/gc_arm_description/scripts/compute_mesh_origins.py --collision   # trimesh 필요: pip install trimesh
# 충돌 제외 목록 재계산 (trimesh, python-fcl 필요) → 출력을 gc_arm.srdf에 반영
xacro src/gc_arm_description/urdf/gc_arm.urdf.xacro > /tmp/gc_arm.urdf
python3 src/gc_arm_description/scripts/compute_collision_matrix.py /tmp/gc_arm.urdf
```
- 단순 도형으로 보기: `use_meshes:=false` (xacro 인자)
- 작업대 상판(1 m × 1 m, base 바닥면 = z 0) 충돌 링크 포함, 크기 변경/제거: `table_size:=0`

## 안전 동작 (gc_arm_hardware)

- 활성화 시 모든 실제 관절의 첫 피드백을 기다리고, 명령 = 현재 위치로 초기화 (급동작 방지)
- **AK 회전수 처리**: AK는 전원을 다시 켜면 출력축 한 바퀴(0~360°) 안의 위치만 기억한다.
  - 동작 중에는 피드백을 연속값으로 이어 붙인다 (360° 경계 통과 대응).
  - 활성화 시 `joints.yaml`의 `startup_reference_deg`(전원 투입 시 자세)에 가장 가까운 회전수를 선택한다.
  - 관절 범위가 모터 한 바퀴를 넘는 AK 관절은 이 값이 없으면 기동을 거부한다.
  - **시작/종료 자세 = `ready`** [0, 90, -90, 0, -90, 0]°: 종료 전 MoveIt으로 ready 이동 → 전원 차단.
    전원 투입 시 실제 자세가 ready에서 J2 ±45°, J3 ±90°, J5 ±70° 이내여야 함
- 피드백 타임아웃(기본 100 ms), AK 드라이버 에러 코드, AK 과열(기본 80 °C), CAN BUS-OFF → `read()` 오류
  → controller_manager가 컨트롤러를 정지
- 명령 위치는 `joints.yaml`의 가동범위로 클램프, 모터 속도는 `max_velocity_deg_s` 이내로 제한
- **AK는 명령이 1 s 끊기면 토크를 해제한다(실기 확인)** → 컨트롤러 정지·비활성화·하드웨어 오류 시
  별도 스레드가 마지막 목표 위치를 20 Hz로 계속 송신해 팔이 처지지 않게 한다.
  - ⚠️ 프로세스 종료(Ctrl+C), PC 정지, cleanup 시에는 송신이 끊겨 **약 1 s 뒤 AK 토크 해제** → 중력 관절(J2/J3)이 처질 수 있음
  - 종료 전 반드시 안정 자세(시작/종료 자세)로 이동할 것. 모터 Timeout 설정 변경(비활성화) 여부 검토 중
- **소프트웨어 안전 기능은 하드웨어 E-stop을 대체하지 않는다**
