# LiDAR_Vision 프로젝트 진행 기록 및 향후 개발 규칙

작성 목적:  
이 문서는 `lidar_vision` 패키지의 현재 진행 상황, 파일 구조, 버전 관리 규칙, 알고리즘 설계 방향, 코드 생성 규칙, 실행/빌드 명령, 향후 확장 계획을 장기적으로 이어가기 위한 기준 문서이다.  
이후 대화에서 이전 맥락이 손실되더라도, 이 Markdown 파일만 제공하면 현재까지의 진행 상황과 앞으로 만들어야 할 방향을 다시 이어갈 수 있도록 작성한다.

---

# 1. 프로젝트 개요

## 1.1 목표

현재 개발 중인 시스템은 Unitree Go2 기반 로봇에서 다음 기능을 수행하는 것을 목표로 한다.

- RealSense D435i RGB 카메라 입력
- Hesai XT16 LiDAR `/lidar_points` 입력
- YOLO11s-seg 기반 사람 segmentation
- YOLO mask와 LiDAR point cloud association
- 사람의 로봇 기준 3D 위치 추정
- 2D Top-view 시각화
- 사람 위치 신뢰도 계산
- 추후 사람 tracking, smoothing, 이동 방향/속도 추정
- 추후 로봇 상태/속도/cmd_vel/odom 반영
- 추후 위험도, TTC, ASFM/social navigation 기반 회피 판단
- 추후 VLM 기반 고수준 상황 판단
- 최종적으로 로봇 내부 Jetson에서는 연산을 수행하고, 원격 PC에서 zenoh 등을 사용해 시각화

---

# 2. 현재 개발 환경

## 2.1 로봇 및 보드

- 로봇: Unitree Go2
- 확장 보드: Jetson Orin NX
- ROS 환경: ROS 2 Foxy 계열로 작업 중
- 주요 센서:
  - Intel RealSense D435i
  - Hesai XT16 LiDAR
- 주요 인식 모델:
  - YOLO11s-seg
- 현재 ROS workspace:

```bash
~/go2_ws
```

## 2.2 ROS 패키지

현재 패키지명:

```bash
lidar_vision
```

패키지 경로:

```bash
~/go2_ws/src/lidar_vision
```

현재 확인된 구조:

```text
lidar_vision/
├── config
│   ├── calibration_current.yaml
│   └── calibration_previous.yaml
├── launch
│   ├── go2_calibration_verify_adjust.launch.py
│   ├── lidar_vision_topview_v1.launch.py
│   └── lidar_vision_topview_v2.launch.py
├── lidar_vision
│   ├── go2_calibration_verify_adjust.py
│   ├── __init__.py
│   ├── lidar_vision_topview_v1_node.py
│   ├── lidar_vision_topview_v2_node.py
│   └── __pycache__
├── package.xml
├── resource
│   └── lidar_vision
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

주의:  
`lidar_vision_topview_node.py(임시보관)`처럼 괄호나 한글이 포함된 Python 파일은 패키지 내부에 두지 않는 것이 좋다. Python import 대상이 아니더라도 향후 혼란을 만들 수 있으므로 `backup/` 폴더로 이동하는 것을 권장한다.

---

# 3. 현재까지 완료된 기능

## 3.1 V1

V1은 초기 기본 구조이다.

주요 기능:

- RealSense RGB frame 획득
- YOLO segmentation으로 사람 검출
- Hesai LiDAR point cloud subscribe
- LiDAR point를 base frame으로 변환
- LiDAR ROI 적용
- LiDAR point를 camera image plane에 projection
- YOLO bbox/mask 내부의 LiDAR point를 association
- 사람의 base frame 위치 추정
- OpenCV 기반 YOLO View 및 2D Top-view 표시

V1은 안정 보존용으로 둔다.

파일:

```text
lidar_vision/lidar_vision_topview_v1_node.py
launch/lidar_vision_topview_v1.launch.py
```

## 3.2 V2

V2는 현재 정상 동작이 확인된 버전이다.

V2의 핵심 기능:

- V1 기능 유지
- LiDAR association confidence 추가
- 사람 위치 추정 결과에 대해 신뢰도 분류
- Top-view와 YOLO View에 신뢰도 색상 표시
- 로그에 confidence, spread, depth_std, reason 표시

파일:

```text
lidar_vision/lidar_vision_topview_v2_node.py
launch/lidar_vision_topview_v2.launch.py
```

V2 node 클래스명:

```python
class LiDARVisionTopViewV2Node(Node):
```

V2 ROS node 이름:

```python
super().__init__("lidar_vision_topview_v2_node")
```

V2 launch 실행 파일:

```python
executable="lidar_vision_topview_v2_node"
name="lidar_vision_topview_v2_node"
```

---

# 4. V2 confidence 설계

## 4.1 V2에서 추가된 사람 상태 값

V2에서는 각 사람 detection에 다음 값을 추가한다.

```text
position_confidence
xy_spread
depth_std
reason
```

각 person dictionary에는 대략 다음 정보가 들어간다.

```python
{
    "id": int,
    "bbox": (x1, y1, x2, y2),
    "mask": mask,
    "center_uv": (center_u, center_v),
    "confidence": yolo_confidence,
    "base_position": (base_x, base_y, base_z),
    "distance": distance,
    "source": "lidar" or "fallback",
    "lidar_count": lidar_count,
    "position_confidence": "HIGH" or "MEDIUM" or "LOW" or "FALLBACK",
    "xy_spread": xy_spread,
    "depth_std": depth_std,
    "reason": reason,
}
```

## 4.2 confidence 분류

```text
HIGH:
LiDAR 점이 충분하고, XY cluster가 compact하며, depth std가 작음

MEDIUM:
LiDAR association은 사용 가능하지만 HIGH보다는 불안정함

LOW:
LiDAR association은 되었지만 점 수가 부족하거나 spread/depth_std가 커서 신뢰도가 낮음

FALLBACK:
LiDAR association 실패. YOLO pixel 위치와 fixed_distance를 이용해 임시 위치 추정
```

## 4.3 색상 규칙

OpenCV BGR 기준:

```python
HIGH     = (0, 255, 0)      # Green
MEDIUM   = (0, 255, 255)    # Yellow
LOW      = (0, 165, 255)    # Orange
FALLBACK = (0, 0, 255)      # Red
```

화면에서 의미:

```text
Green  = HIGH
Yellow = MEDIUM
Orange = LOW
Red    = FALLBACK
```

## 4.4 V2 confidence threshold 기본값

V2 launch에 포함된 기본 파라미터:

```python
"confidence_high_count": 30,
"confidence_medium_count": 10,
"confidence_high_spread": 0.45,
"confidence_medium_spread": 0.70,
"confidence_high_depth_std": 0.25,
"confidence_medium_depth_std": 0.50,
```

## 4.5 V2 정상 출력 예시

실제 실행 화면에서 정상적으로 확인된 예:

```text
P0 0.81m HIGH
x=0.80, y=-0.13, n=942, std=0.16
reason=compact_cluster
```

의미:

```text
P0: 첫 번째 사람
0.81m: 로봇 기준 거리 약 0.81m
HIGH: LiDAR association 신뢰도 높음
x=0.80: 로봇 전방 약 0.80m
y=-0.13: 로봇 기준 오른쪽 약 0.13m
n=942: 사람 mask/bbox에 association된 LiDAR point 수
std=0.16: 깊이 방향 표준편차가 0.16m
compact_cluster: LiDAR 점들이 사람 주변에 잘 모여 있음
```

---

# 5. 현재 calibration 정보

현재 기준 calibration 파일:

```text
config/calibration_current.yaml
```

이전 calibration 파일:

```text
config/calibration_previous.yaml
```

현재 가장 신뢰하는 calibration 값:

```yaml
/**:
  ros__parameters:
    camera_base_x: 0.350
    camera_base_y: 0.010
    camera_base_z: 0.270
    camera_roll_deg: 0.6
    camera_pitch_deg: -9.8
    camera_yaw_deg: 1.9

    lidar_base_x: 0.150
    lidar_base_y: 0.000
    lidar_base_z: 0.250
    lidar_roll_deg: -6.0
    lidar_pitch_deg: 0.0
    lidar_yaw_deg: 90.0
```

기존/이전 값:

```yaml
camera_base_x: 0.340
camera_base_y: 0.040
camera_base_z: 0.390
camera_roll_deg: 0.0
camera_pitch_deg: 0.0
camera_yaw_deg: -0.5

lidar_base_x: 0.150
lidar_base_y: 0.000
lidar_base_z: 0.250
lidar_roll_deg: -6.0
lidar_pitch_deg: 0.0
lidar_yaw_deg: 90.0
```

주의:

- calibration 값은 V2 실행 시 `calibration_config` 파라미터로 전달한다.
- launch의 inline parameter보다 YAML calibration 파일이 우선 적용되도록 `parameters=[calibration_config, {...}]` 구조를 사용한다.
- 사람이 실제로 1.5m 이상 떨어져 있는데 x=0.8m처럼 나온다면 calibration, mask association, 가까운 물체 포함 여부를 다시 점검해야 한다.

---

# 6. setup.py 규칙

현재 `setup.py`는 다음 구조를 가져야 한다.

```python
from setuptools import setup
from glob import glob
import os

package_name = "lidar_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="leedongsup9149@gmail.com",
    description="LiDAR_Vision package for RealSense, YOLO segmentation, Hesai LiDAR fusion, and calibration verification.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_vision_topview_node = lidar_vision.lidar_vision_topview_v2_node:main",
            "lidar_vision_topview_v1_node = lidar_vision.lidar_vision_topview_v1_node:main",
            "lidar_vision_topview_v2_node = lidar_vision.lidar_vision_topview_v2_node:main",
            "go2_calibration_verify_adjust = lidar_vision.go2_calibration_verify_adjust:main",
        ],
    },
)
```

## 6.1 중요 규칙

`lidar_vision_topview_node`는 latest stable alias로 둔다.

현재 latest stable은 V2이다.

```python
"lidar_vision_topview_node = lidar_vision.lidar_vision_topview_v2_node:main"
```

향후 V3가 안정화되면 이 alias를 다음처럼 바꿀 수 있다.

```python
"lidar_vision_topview_node = lidar_vision.lidar_vision_topview_v3_node:main"
```

단, 정식 버전 실행 파일은 항상 유지한다.

```python
"lidar_vision_topview_v2_node = lidar_vision.lidar_vision_topview_v2_node:main"
"lidar_vision_topview_v3_node = lidar_vision.lidar_vision_topview_v3_node:main"
```

---

# 7. 파일 생성 및 버전 관리 규칙

## 7.1 버전명 규칙

앞으로는 `V2.1`, `V2.2`, `V2.3`처럼 소수점 버전을 쓰지 않는다.

정식 버전만 사용한다.

```text
V1
V2
V3
V4
V5
V6
V7
V8
```

이유:

- 파일 관리가 단순함
- launch 관리가 단순함
- setup.py entry point 관리가 쉬움
- 사용자가 로봇에서 직접 작업할 때 헷갈리지 않음

## 7.2 node 파일명 규칙

```text
lidar_vision_topview_v1_node.py
lidar_vision_topview_v2_node.py
lidar_vision_topview_v3_node.py
lidar_vision_topview_v4_node.py
...
```

## 7.3 launch 파일명 규칙

```text
lidar_vision_topview_v1.launch.py
lidar_vision_topview_v2.launch.py
lidar_vision_topview_v3.launch.py
lidar_vision_topview_v4.launch.py
...
```

## 7.4 class 이름 규칙

```python
class LiDARVisionTopViewV1Node(Node):
class LiDARVisionTopViewV2Node(Node):
class LiDARVisionTopViewV3Node(Node):
class LiDARVisionTopViewV4Node(Node):
```

## 7.5 ROS node 이름 규칙

```python
super().__init__("lidar_vision_topview_v1_node")
super().__init__("lidar_vision_topview_v2_node")
super().__init__("lidar_vision_topview_v3_node")
super().__init__("lidar_vision_topview_v4_node")
```

## 7.6 console script 규칙

```python
"lidar_vision_topview_v1_node = lidar_vision.lidar_vision_topview_v1_node:main"
"lidar_vision_topview_v2_node = lidar_vision.lidar_vision_topview_v2_node:main"
"lidar_vision_topview_v3_node = lidar_vision.lidar_vision_topview_v3_node:main"
```

## 7.7 latest alias 규칙

```python
"lidar_vision_topview_node = lidar_vision.lidar_vision_topview_v{latest}_node:main"
```

현재 latest stable:

```python
"lidar_vision_topview_node = lidar_vision.lidar_vision_topview_v2_node:main"
```

## 7.8 백업 규칙

중간 실험 파일은 정식 버전명에 넣지 않고 `backup/`에 보관한다.

권장 구조:

```text
backup/
├── v2_confidence_ok.py
├── v3_tracking_test_before_fix.py
├── v4_velocity_test_backup.py
└── ...
```

정식 실행 대상 파일은 `lidar_vision/` 패키지 내부에만 둔다.

```text
lidar_vision/
├── __init__.py
├── go2_calibration_verify_adjust.py
├── lidar_vision_topview_v1_node.py
├── lidar_vision_topview_v2_node.py
├── lidar_vision_topview_v3_node.py
└── ...
```

---

# 8. 빌드 및 실행 규칙

## 8.1 기본 빌드

```bash
cd ~/go2_ws
source /opt/ros/foxy/setup.bash

colcon build --packages-select lidar_vision --symlink-install

source install/setup.bash
```

## 8.2 빌드 캐시 정리

launch 파일 이름 변경, setup.py 변경, entry point 변경 후 문제가 생기면 아래를 수행한다.

```bash
cd ~/go2_ws

rm -rf build/lidar_vision
rm -rf install/lidar_vision
rm -rf log/latest
rm -rf log/latest_build

find ~/go2_ws/src/lidar_vision -name "*.egg-info" -type d -exec rm -rf {} +

source /opt/ros/foxy/setup.bash
colcon build --packages-select lidar_vision --symlink-install
source install/setup.bash
```

## 8.3 실행 파일 확인

```bash
ros2 pkg executables lidar_vision
```

예상 출력:

```text
lidar_vision go2_calibration_verify_adjust
lidar_vision lidar_vision_topview_node
lidar_vision lidar_vision_topview_v1_node
lidar_vision lidar_vision_topview_v2_node
```

향후 V3 추가 시:

```text
lidar_vision lidar_vision_topview_v3_node
```

도 보여야 한다.

## 8.4 launch 목록 확인

```bash
ros2 launch lidar_vision
```

예상 목록:

```text
go2_calibration_verify_adjust.launch.py
lidar_vision_topview_v1.launch.py
lidar_vision_topview_v2.launch.py
```

향후 V3 추가 시:

```text
lidar_vision_topview_v3.launch.py
```

도 보여야 한다.

---

# 9. RealSense / LiDAR 실행 주의사항

## 9.1 Go2 LiDAR만 bringup

현재 `lidar_vision` node가 RealSense를 직접 열기 때문에, Go2 bringup에서 RealSense를 같이 켜면 장치 충돌이 날 수 있다.

권장:

```bash
cd ~/go2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 launch go2_bringup go2.launch.py lidar:=True realsense:=False rviz:=False
```

## 9.2 V2 실행

다른 터미널에서:

```bash
cd ~/go2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 launch lidar_vision lidar_vision_topview_v2.launch.py \
  model:=/home/unitree/go2_ws/yolo11s-seg.pt \
  calibration_config:=/home/unitree/go2_ws/src/lidar_vision/config/calibration_current.yaml
```

## 9.3 RealSense 장치 충돌 확인

에러 예:

```text
xioctl(VIDIOC_S_FMT) failed, errno=16 Device or resource busy
```

확인:

```bash
sudo fuser -v /dev/video*
```

종료:

```bash
sudo kill -9 <PID>
```

또는 관련 프로세스 종료:

```bash
pkill -f go2_calibration_verify_adjust.py
pkill -f go2_realsense_lidar_3d_calib
pkill -f lidar_vision
```

---

# 10. 현재 V2 알고리즘 구조

## 10.1 전체 흐름

```text
RealSense RGB frame
        ↓
YOLO11s-seg person segmentation
        ↓
Hesai XT16 /lidar_points 수신
        ↓
LiDAR raw points → base frame 변환
        ↓
LiDAR ROI filtering
        ↓
LiDAR points → camera image projection
        ↓
YOLO bbox/mask 내부 LiDAR point association
        ↓
nearest cluster 선택
        ↓
outlier 제거
        ↓
사람 위치 median 계산
        ↓
LiDAR association confidence 계산
        ↓
YOLO View / Top-view 시각화
```

## 10.2 좌표계

base frame:

```text
x: forward
y: left
z: up
```

camera body:

```text
x: forward
y: left
z: up
```

RealSense optical:

```text
X: right
Y: down
Z: forward
```

V2는 RealSense optical 좌표와 base 좌표 변환을 내부적으로 처리한다.

## 10.3 LiDAR association 주요 단계

1. YOLO bbox 내부 point 선택
2. mask가 있으면 mask 내부 point만 선택
3. 사람 몸통 중앙 영역을 우선 사용
4. forward x가 유효한 점만 사용
5. 가까운 depth cluster 선택
6. XY plane outlier 제거
7. median position 계산
8. xy_spread, depth_std 계산
9. HIGH/MEDIUM/LOW confidence 결정

---

# 11. 향후 버전 개발 계획

## 11.1 V1

```text
초기 기본 구조
```

기능:

- RGB
- YOLO
- LiDAR association
- Top-view

## 11.2 V2

```text
YOLO + LiDAR association + confidence
```

현재 완료/정상 동작 확인.

기능:

- position_confidence
- xy_spread
- depth_std
- reason
- HIGH/MEDIUM/LOW/FALLBACK 시각화

## 11.3 V3

```text
tracking memory + smoothing + lost-frame persistence
```

V3 목표:

- 사람 ID 유지
- 이전 위치 기억
- EMA smoothing
- 감지 실패 시 몇 프레임 동안 STALE로 유지
- 사람이 한두 프레임 놓쳐져도 바로 사라지지 않도록 함

V3에서 아직 넣지 말 것:

- 사람 속도/방향 본격 추정
- robot odom/cmd_vel 반영
- TTC
- ASFM
- VLM
- 실제 제어 명령

V3 예상 person 상태:

```python
{
    "track_id": int,
    "base_position": (x, y, z),
    "smoothed_position": (sx, sy, sz),
    "distance": float,
    "position_confidence": str,
    "source": str,
    "lidar_count": int,
    "missed_frames": int,
    "is_stale": bool,
    "last_seen_time": float,
}
```

V3 권장 파라미터:

```python
"track_match_distance": 0.8,
"smoothing_alpha": 0.35,
"max_missing_frames": 5,
```

10Hz 기준 `max_missing_frames=5`는 약 0.5초 유지에 해당한다.

## 11.4 V4

```text
사람 속도와 이동 방향 추정
```

V4 목표:

- 사람별 velocity 계산
- 이동 방향 vector 계산
- Top-view에 사람 이동 방향 화살표 표시
- approaching / leaving / crossing 판단 준비

V4에서 필요한 값:

```python
velocity_base = (vx, vy, vz)
speed = sqrt(vx^2 + vy^2)
moving_direction = atan2(vy, vx)
motion_state = "approaching" / "leaving" / "crossing" / "static"
```

## 11.5 V5

```text
robot state / odom / cmd_vel 반영
```

V5 목표:

- 로봇 자신의 이동 때문에 생기는 상대 위치 변화를 보정
- 사람의 실제 이동과 로봇 이동을 구분
- 상대속도 계산
- robot velocity, yaw rate 반영

필요 입력 후보:

```text
/odom
/cmd_vel
Unitree SportModeState
/lowstate
/tf
```

V5에서 기억해야 할 robot state:

```python
{
    "robot_pose": ...,
    "robot_linear_velocity": ...,
    "robot_angular_velocity": ...,
    "yaw": ...,
    "yaw_rate": ...,
    "last_cmd_vel": ...,
}
```

## 11.6 V6

```text
위험도 / TTC / 회피 판단
```

V6 목표:

- 사람과의 거리 기반 위험도
- 접근 속도 기반 위험도
- Time-To-Collision
- 좌/우 빈 공간
- crowd density
- Stop / Slow / Left / Right / Forward 판단 준비

예상 출력:

```python
{
    "risk_level": "LOW" / "MEDIUM" / "HIGH" / "CRITICAL",
    "ttc": float,
    "recommended_action": "STOP" / "SLOW" / "LEFT" / "RIGHT" / "FORWARD",
}
```

## 11.7 V7

```text
VLM 비동기 판단 추가
```

V7 목표:

- VLM을 매 프레임 돌리지 않음
- VLM은 저주기 또는 이벤트 기반으로 실행
- 군중, 위험 행동, 통로 막힘, 특수 상황 판단 보조
- 즉각 회피는 VLM을 기다리지 않고 fast loop에서 처리

VLM 실행 조건 예:

```text
사람 3명 이상
전방 혼잡도 높음
회피 방향 판단이 애매함
새로운 위험 상황 발생
1~2초마다 주기적 확인
```

## 11.8 V8

```text
최종 fusion / decision / control 구조
```

V8 목표:

- 로봇 상태
- 사람 상태
- 장애물 상태
- VLM 의미 판단
- 이동 방향 결정
- cmd_vel 생성 또는 상위 제어 명령 생성

최종 제어 우선순위:

```text
긴급 충돌 위험 > 사람 회피 > 경로 추종 > 일반 전진 > VLM 고수준 보조
```

---

# 12. 실시간성 및 주기 목표

## 12.1 실시간성 기준

로봇 회피 관점의 실용적 기준:

```text
50 Hz 이상:
매우 높은 실시간성. 저수준 제어/긴급 정지에 유리

20 Hz:
높은 실시간성. 즉각 회피에 좋음

10 Hz:
모바일 로봇 local perception/control에 현실적으로 사용 가능

5 Hz:
준실시간. 느린 추적/위험도 판단은 가능하지만 즉각회피 단독으로는 부족

2 Hz:
VLM/semantic 판단용. 즉각 회피용으로 부적합

1 Hz 이하:
비실시간에 가까움. 고수준 전략 판단용
```

## 12.2 모듈별 목표 주기

```text
긴급 안전 정지 / 최소거리 체크:
20~50 Hz

LiDAR obstacle processing:
10~20 Hz

YOLO + LiDAR 사람 위치 추정:
8~15 Hz

사람 tracking / smoothing:
10~20 Hz

사람 속도/방향 추정:
5~10 Hz

local decision / cmd_vel 생성:
10~20 Hz

VLM scene reasoning:
0.5~2 Hz

global / semantic planning:
0.2~1 Hz
```

## 12.3 현재 V2 목표

현재 V2는 다음을 목표로 한다.

```text
target_fps = 10 Hz
```

향후 V3/V4에서도 최소 8~10Hz 안정 유지가 목표이다.

VLM은 즉각 회피에 사용하지 않는다.

```text
VLM = 0.5~2 Hz, 고수준 상황 판단 보조
Fast loop = LiDAR + YOLO + tracking + robot state 기반 즉각 회피
```

---

# 13. 시스템 구조 설계 방향

## 13.1 최종 구조 철학

모든 기능을 하나의 while loop에서 순차 실행하지 않는다.

나쁜 구조:

```python
while True:
    get_camera()
    get_lidar()
    run_yolo()
    run_vlm()
    decide()
    publish_cmd()
```

문제:

- VLM이 느리면 전체 loop가 멈춤
- 빠른 회피가 느려짐
- 센서 timing 불일치
- 디버깅 어려움

권장 구조:

```text
비동기 멀티레이트 파이프라인
```

## 13.2 추천 노드 구조

초기에는 하나의 node 안에서 구현하되, 최종적으로는 다음처럼 분리 가능하다.

```text
Sensor Ingest Node:
- RealSense RGB 수신
- LiDAR point cloud 수신
- robot odom/state 수신
- timestamp 정리

Perception Node:
- YOLO segmentation
- LiDAR association
- 사람 위치 및 confidence 계산

Tracking Node:
- smoothing
- short-term memory
- track_id 유지
- 속도/방향 계산

Semantic/VLM Node:
- 저주기 또는 이벤트 기반 VLM 실행
- scene meaning 생성
- crowd/behavior/context 요약

Fusion / Decision Node:
- robot state + tracked humans + obstacle map + VLM result 통합
- 위험도 평가
- 회피 방향 결정
- cmd_vel 생성
```

## 13.3 시간 동기화 개념

모든 모듈이 정확히 같은 CPU 시점에 끝나야 하는 것은 아니다.

중요한 것은:

```text
각 데이터에 timestamp를 붙이고
최종 fusion에서 같은 시간대의 상태로 묶는 것
```

예:

```text
camera_frame @ 12.100
lidar_frame  @ 12.080
odom         @ 12.102
VLM result   @ 12.300
```

처리는 다음처럼 한다.

```text
fast state @ 12.100 근처:
camera + lidar + odom

semantic state:
latest valid VLM result
```

VLM은 느리므로 latest valid semantic context로 사용한다.

---

# 14. 멀티쓰레드 / 병렬 처리 방향

## 14.1 왜 필요한가?

향후에는 다음 연산이 동시에 존재한다.

```text
RealSense frame 획득
LiDAR callback
YOLO inference
tracking update
robot state callback
VLM inference
decision loop
visualization publish
```

따라서 single-thread 구조만으로는 callback blocking이 발생할 수 있다.

## 14.2 ROS2에서 고려할 방식

초기:

```text
SingleThreadedExecutor 가능
```

복잡해지면:

```text
MultiThreadedExecutor 고려
```

단, Python GIL 때문에 CPU-bound Python 코드는 thread만으로 빨라지지 않을 수 있다.  
하지만 ROS callback, I/O, GPU inference 대기 시간 분리에는 도움이 된다.

## 14.3 최종 권장 구조

```text
Fast loop:
LiDAR + YOLO + tracking + local decision

Slow loop:
VLM + semantic reasoning

Fusion:
timestamp 기반 latest state 결합
```

---

# 15. 현재 시각화와 최종 원격 시각화 계획

## 15.1 현재

현재는 로봇에서 직접 작업하고 있으므로 OpenCV window를 띄운다.

현재 출력:

```text
YOLO View
2D Top View
```

이 방식은 개발 및 디버깅에 적합하다.

## 15.2 최종 계획

알고리즘이 어느 정도 안정화되면, 로봇 내부에서는 화면을 직접 띄우지 않고 연산과 publish 중심으로 바꾼다.

최종 구조:

```text
Jetson / Robot:
- perception
- tracking
- fusion
- decision
- topic publish

Remote PC:
- zenoh 기반 수신
- RViz / OpenCV / Web UI 시각화
- recording
- dashboard
```

향후 publish 후보 topic:

```text
/lidar_vision/tracked_persons
/lidar_vision/topview_debug_image
/lidar_vision/yolo_debug_image
/lidar_vision/system_status
/lidar_vision/decision
/lidar_vision/risk_state
```

현재는 계획만 참고하고, V3/V4에서는 아직 zenoh를 직접 넣지 않는다.

---

# 16. 향후 코드 작성 시 반드시 지켜야 할 규칙

## 16.1 사용자가 선호하는 방식

- 한국어로 자세히 설명
- 코드 내부 주석과 print/log 출력은 영어 유지
- 완성형 코드 선호
- 불필요한 생략 금지
- 큰 기능을 한 번에 무리하게 넣지 말고 버전별로 나눔
- 문제가 생기면 원인을 명확히 추적할 수 있어야 함
- 현재 안정 버전을 깨지 않도록 새 버전 파일로 분리
- launch와 setup.py도 버전에 맞게 같이 수정

## 16.2 코드 생성 시 규칙

새 버전 생성 시 반드시 다음을 확인한다.

예: V3 생성 시

1. 파일 복사

```bash
cp lidar_vision_topview_v2_node.py lidar_vision_topview_v3_node.py
cp ../launch/lidar_vision_topview_v2.launch.py ../launch/lidar_vision_topview_v3.launch.py
```

2. class 이름 변경

```python
class LiDARVisionTopViewV3Node(Node):
```

3. ROS node 이름 변경

```python
super().__init__("lidar_vision_topview_v3_node")
```

4. main() 변경

```python
node = LiDARVisionTopViewV3Node()
```

5. launch executable 변경

```python
executable="lidar_vision_topview_v3_node"
name="lidar_vision_topview_v3_node"
```

6. setup.py entry point 추가

```python
"lidar_vision_topview_v3_node = lidar_vision.lidar_vision_topview_v3_node:main"
```

7. V3가 안정화되기 전에는 latest alias를 바꾸지 않는다.

현재 alias:

```python
"lidar_vision_topview_node = lidar_vision.lidar_vision_topview_v2_node:main"
```

V3 안정화 후에만:

```python
"lidar_vision_topview_node = lidar_vision.lidar_vision_topview_v3_node:main"
```

으로 변경한다.

---

# 17. 디버깅 기준

## 17.1 사람이 이상한 위치에 표시될 때

가능 원인:

```text
YOLO mask 오류
LiDAR projection 오류
calibration 오류
LiDAR ROI 오류
mask 내부에 배경/책상/벽 point 포함
near cluster threshold 문제
person_outlier_radius 문제
camera pitch/yaw/height 오류
```

확인 순서:

1. YOLO bbox/mask가 사람을 정확히 잡는지 확인
2. confidence가 HIGH인지 LOW/FALLBACK인지 확인
3. lidar_count 확인
4. depth_std 확인
5. xy_spread 확인
6. reason 확인
7. 실제 거리와 x값 비교
8. 실제 좌우 위치와 y 부호 비교

## 17.2 y 방향 부호 확인

좌표계:

```text
+Y = left
-Y = right
```

테스트:

```text
사람이 로봇 왼쪽에 있음 → y > 0
사람이 로봇 오른쪽에 있음 → y < 0
```

## 17.3 거리 확인

정면 1m 테스트:

```text
x ≈ 1.0
y ≈ 0.0
confidence = HIGH 또는 MEDIUM
```

정면 2m 테스트:

```text
x ≈ 2.0
y ≈ 0.0
confidence = HIGH 또는 MEDIUM
```

## 17.4 V2 confidence 해석

```text
HIGH + n 많음 + std 낮음:
정상 가능성 높음

LOW + few_lidar_points:
LiDAR point 부족

LOW + wide_xy_spread:
mask 안에 여러 물체/배경이 섞였을 가능성

LOW + large_depth_std:
depth 방향으로 점들이 넓게 퍼짐

FALLBACK:
LiDAR association 실패. 위치 신뢰 낮음
```

---

# 18. 앞으로 바로 해야 할 다음 작업

현재 가장 적절한 다음 작업은 V3 생성이다.

## 18.1 V3 목표

```text
V3 = V2 + tracking memory + EMA smoothing + lost-frame persistence
```

## 18.2 V3에서 구현할 기능

1. Track class 또는 dictionary 구조 추가
2. track_id 발급
3. 현재 detections와 기존 tracks를 nearest-neighbor로 매칭
4. 매칭된 track은 위치 업데이트
5. 위치는 EMA smoothing 적용
6. 매칭되지 않은 track은 missed_frames 증가
7. missed_frames <= max_missing_frames이면 STALE로 유지
8. missed_frames 초과 시 track 삭제
9. 화면에 ACTIVE / STALE 표시
10. Top-view에서 stale track은 흐리게 또는 다른 label로 표시

## 18.3 V3에서 아직 하지 말 것

```text
사람 속도 방향 추정의 본격 적용
robot odom 보정
cmd_vel 제어
VLM 호출
TTC 계산
ASFM 회피
zenoh 원격 시각화
```

---

# 19. 장기 설계 핵심 문장

향후 보고서나 논문/발표에서 사용할 수 있는 설계 설명:

```text
본 시스템은 즉각적인 충돌 회피와 고수준 상황 이해를 분리하기 위해 비동기 멀티레이트 구조를 지향한다. LiDAR, YOLO, tracking 기반의 fast perception loop는 8~20 Hz 수준으로 동작하여 사람 위치와 위험 상태를 빠르게 갱신하고, VLM 기반 semantic reasoning은 0.5~2 Hz의 저주기 또는 이벤트 기반으로 수행하여 장면 의미 해석을 보조한다. 최종 decision layer는 timestamp 기반으로 로봇 상태, 사람 상태, 장애물 상태, VLM 결과를 융합하여 Stop, Slow, Left, Right, Forward와 같은 이동 판단을 수행한다.
```

---

# 20. 이 문서를 사용할 때의 지침

이후 대화에서 맥락이 손실되면 다음과 같이 사용한다.

1. 이 Markdown 파일을 업로드한다.
2. 현재 작업하려는 버전을 명확히 말한다.
   - 예: “이 문서를 기준으로 V3를 만들어줘.”
3. 현재 실제 파일도 함께 업로드한다.
   - 예:
     - `lidar_vision_topview_v2_node.py`
     - `lidar_vision_topview_v2.launch.py`
     - `setup.py`
4. 새 코드는 반드시 버전 규칙에 맞춰 생성한다.
5. 안정 버전은 보존하고 새 버전 파일로 작업한다.

---

# 21. 현재 기준 요약

현재 완료:

```text
V2 정상 동작 확인
YOLO + LiDAR association 성공
confidence HIGH/MEDIUM/LOW/FALLBACK 적용
Top-view 정상 출력
calibration_current.yaml 적용
```

다음 목표:

```text
V3 생성
tracking memory
EMA smoothing
lost-frame persistence
ACTIVE/STALE 표시
```

향후 목표:

```text
V4 사람 속도/방향
V5 robot state 반영
V6 위험도/TTC
V7 VLM 비동기 판단
V8 fusion/decision/control
원격 PC zenoh 시각화
```

---

# 22. 마지막 주의사항

- VLM은 즉각회피를 담당하지 않는다.
- 즉각회피는 LiDAR + 사람 tracking + robot state 기반 fast loop가 담당해야 한다.
- VLM은 느린 고수준 의미 판단 보조로 사용한다.
- 사람 추적 메모리는 필수지만, 로봇이 움직이기 시작하면 robot odom/cmd_vel을 반영해야 사람 실제 이동과 로봇 이동을 구분할 수 있다.
- OpenCV window는 현재 개발 단계에서만 사용하고, 최종적으로는 topic publish 및 원격 시각화 구조로 전환한다.
- 파일명, class명, node명, launch executable, setup.py entry point는 항상 같은 버전 번호로 맞춘다.

