# gc_arm_drivers

GC 6축 로봇팔의 CAN 드라이버 라이브러리 + Phase 0 테스트 CLI (`gc_motor_cli`).

| 구성 | 파일 | 상태 |
|---|---|---|
| SocketCAN 래퍼 | `socketcan.hpp/.cpp` | 구현 |
| CubeMars AK Servo 모드 | `ak_servo_protocol.hpp/.cpp` | 구현, 단위 테스트 |
| CubeMars AK MIT 모드 | `ak_mit_protocol.hpp/.cpp` | 구현(실험적), 실기 미검증 |
| MKS SERVO42D/57D | `mks_protocol.hpp/.cpp` | 구현, 단위 테스트 |
| 테스트 CLI | `tools/gc_motor_cli.cpp` | 구현 |

프로토콜 계층은 `CanFrame` 구조체만 다루는 순수 함수라서 하드웨어 없이 테스트할 수 있고,
Phase 2의 ros2_control 하드웨어 플러그인이 같은 코드를 그대로 사용한다.

## 빌드

### ROS 2 Jazzy (colcon)
```bash
cd ~/GC_6-DOF_RobotArm        # 저장소 루트 = colcon 워크스페이스
colcon build --packages-select gc_arm_drivers
colcon test --packages-select gc_arm_drivers && colcon test-result --verbose
source install/setup.bash
ros2 run gc_arm_drivers gc_motor_cli --help
```

### ROS 없이 (순수 CMake)
```bash
sudo apt install cmake g++ libgtest-dev
cmake -S src/gc_arm_drivers -B build/gc_arm_drivers
cmake --build build/gc_arm_drivers -j
(cd build/gc_arm_drivers && ctest --output-on-failure)
./build/gc_arm_drivers/gc_motor_cli --help
```

## CAN 인터페이스 준비 (CANable 2.0 Pro, candleLight 펌웨어)
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000     # 기본 10은 고주기 송신 시 TX 큐 포화
sudo ip link set can0 up
ip -details -statistics link show can0    # 상태(ERROR-ACTIVE), 에러 카운터 확인
```

## Phase 0 시험 절차

> ⚠️ 첫 시험은 모터를 기구에서 분리하거나 부하 없이, 작은 각도로. 비상 시 전원 차단 준비.

### 1. 비트레이트 통일 (MKS → 1 Mbps)
MKS 기본 비트레이트가 1 Mbps가 아니면 드라이버 화면 메뉴(CanRate)에서 바꾸거나 CLI로 변경:
```bash
# can0를 MKS의 현재 비트레이트(예: 500k)로 올린 뒤
gc_motor_cli mks 2 set-bitrate 1000 --yes
# can0를 1M으로 다시 올림
```

### 2. 버스 확인
```bash
gc_motor_cli monitor
```
- AK: Servo 모드 피드백(`AK 피드백 id=...`)이 업로드 주기마다 보여야 함.
  안 보이면 CubeMars 상위 프로그램에서 **피드백 업로드 주기**를 설정 (권장 100~200 Hz)
- `NO_ACK` 에러: 다른 노드가 응답하지 않음 → 비트레이트/배선/종단저항 확인

### 3. 모터 개별 확인
```bash
# AK45-36 (J3)
gc_motor_cli ak 102 status --count 5
gc_motor_cli ak 102 posspd 10 --spd 5000 --acc 10000    # 출력축 +10 deg
gc_motor_cli ak 102 posspd 0  --spd 5000 --acc 10000

# MKS SERVO42D (J4)
gc_motor_cli mks 2 read
gc_motor_cli mks 2 enable
gc_motor_cli mks 2 rel 90 --rpm 60 --acc 50             # 모터축 +90 deg
gc_motor_cli mks 2 read
```

### 4. 단위/방향 검증 (중요)
- **AK 위치 단위**: `posspd 90` 명령 후 출력축이 실제로 90° 돌았는지 확인
  (펌웨어가 출력축 기준 deg를 보고한다는 가정 검증)
- **AK 속도 단위(ERPM)**: `ak <id> rpm 3000` 후 출력축 회전 속도 측정 →
  `ERPM = 출력 RPM × 내장 감속비 × 극쌍수` 로 극쌍수 확인
- **회전 방향**: 각 관절의 + 방향을 기록 (URDF 방향과 맞추기 위해)

### 5. 혼합 버스 시험
```bash
# 수동: AK 피드백 계수 + MKS 0x31 폴링 응답률/지연
gc_motor_cli bustest --ak 102,103 --mks 2,3 --rate 100 --duration 10
# 실제 제어 트래픽 재현: 매 주기 현재 위치 유지 명령도 송신
gc_motor_cli bustest --ak 102,103 --mks 2,3 --rate 200 --duration 30 --hold
```
판정 기준(제안): MKS 응답률 99.9% 이상, 최대 지연 < 제어주기의 1/2, 에러 프레임/송신 실패 0.

### 6. MKS 궤적 추종 방식 비교 (설계 이슈)
```bash
gc_motor_cli mks 2 sweep 30 2 --mode pos   --rpm 300 --acc 0 --csv mks_pos.csv
gc_motor_cli mks 2 sweep 30 2 --mode speed --kp 5          --csv mks_speed.csv
gc_motor_cli ak 102 sweep 20 2 --spd 10000 --acc 30000     --csv ak_posspd.csv
```
- `pos`: 0xF5 절대위치를 매 주기 갱신
- `speed`: 0xF6 속도 모드 + 외부 위치 루프 (속도 분해능 1 rpm = 6 deg/s 한계에 유의)
- CSV(목표 vs 측정)를 그려서 끊김/지연/오버슈트 비교

## 버스 부하 참고
1 Mbps에서 8바이트 프레임 ≈ 110~130 bit → 최대 약 8000 frame/s.
6축 × (명령 + 응답) × 200 Hz = 2400 frame/s ≈ 30~40% 부하.
AK 피드백 업로드 주기를 너무 높이면(예: 3대 × 500 Hz) 부하가 커지므로 제어주기와 맞추는 것을 권장.

## 프로토콜 요약

### CubeMars AK — Servo 모드 (확장 프레임)
| 명령 | packet | ID (id=102) | 데이터 |
|---|---|---|---|
| Duty | 0 | 0x0066 | int32 duty×100000 |
| Current | 1 | 0x0166 | int32 A×1000 |
| Brake | 2 | 0x0266 | int32 A×1000 |
| RPM | 3 | 0x0366 | int32 ERPM |
| Position | 4 | 0x0466 | int32 deg×10000 |
| Set origin | 5 | 0x0566 | uint8 (0 임시, 1 영구, 2 기본복원) |
| Pos-Spd | 6 | 0x0666 | int32 deg×10000, int16 ERPM/10, int16 (ERPM/s)/10 |
| **피드백** | 0x29 | 0x2966 | int16 pos×10 deg, int16 ERPM/10, int16 A×100, int8 온도, uint8 에러 |

### MKS SERVO42D/57D (표준 프레임, `[cmd, ..., CRC]`, CRC = (ID + 바이트합) & 0xFF)
| 명령 | 코드 | 요청 | 응답 |
|---|---|---|---|
| 엔코더(addition) | 0x31 | – | int48 count (16384/rev) |
| 속도 읽기 | 0x32 | – | int16 rpm |
| 상태 조회 | 0xF1 | – | uint8 |
| Enable | 0xF3 | uint8 | uint8 |
| 상대 위치(축) | 0xF4 | uint16 rpm, uint8 acc, int24 | uint8 |
| 절대 위치(축) | 0xF5 | uint16 rpm, uint8 acc, int24 | uint8 |
| 속도 모드 | 0xF6 | dir(bit7)+rpm 12bit, uint8 acc | uint8 |
| 비상 정지 | 0xF7 | – | uint8 |
| 영점 설정 | 0x92 | – | uint8 |

> 프로토콜 값은 제조사 매뉴얼 기준으로 작성했으며, 펌웨어 버전에 따라 다를 수 있으므로 Phase 0에서 실측 확인한다.
