# go2_liosam2

Unitree **Go2** 로봇 + **Hesai XT16** LiDAR 환경에서  
**LIO-SAM** (ROS 2 Humble) 을 이용한 3D LiDAR-IMU SLAM 패키지입니다.

데이터 수집은 로봇에서 수행하고, SLAM 처리는 **오프라인**(rosbag 재생) 방식으로 수행하는  
워크플로우를 기본으로 설계되었습니다.

---

## 센서 구성

| 센서 | 모델 | ROS 2 토픽 | 프레임 |
|------|------|------------|--------|
| LiDAR | Hesai XT16 (16채널, 10 Hz) | `/lidar_points` | `hesai_lidar` |
| IMU | Go2 내장 (utlidar 보정) | `/utlidar/imu_corrected` | `base_link` |

### Extrinsics (LiDAR → IMU)

```
Translation : [0.1710, 0.0, -0.0908]  [m]
Rotation    : R_x(π) = diag(1, -1, -1)   (Y·Z 축 반전)
```

---

## 사전 요구사항

### 1. LIO-SAM ROS 2 설치

```bash
sudo apt install ros-humble-lio-sam
```

또는 소스 빌드:

```bash
cd ~/go2_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd .. && colcon build --packages-select lio_sam
```

### 2. Hesai 드라이버가 `ring` 필드를 출력하는지 확인

LIO-SAM의 `imageProjection` 노드는 포인트클라우드에 `ring` (uint16) 필드가  
필요합니다. Hesai XT16 드라이버가 이 필드를 포함하는지 아래 명령으로 확인합니다:

```bash
ros2 topic echo --once /lidar_points | grep ring
# 또는
ros2 run pcl_ros pointcloud_to_pcd   # rviz2 에서 field 목록 확인
```

`ring` 필드가 없는 경우 별도의 변환 노드가 필요합니다 (채널 번호 = ring).

---

## 워크플로우

### Step 1 — 로봇에서 데이터 수집 (녹화)

**방법 A: launch 파일 사용 (권장)**

```bash
# 기본 경로(~/mapping_bags)에 저장
ros2 launch go2_liosam2 record_mapping.launch.py

# 저장 경로 및 bag 이름 지정
ros2 launch go2_liosam2 record_mapping.launch.py \
    save_dir:=/media/usb/bags \
    bag_name:=campus_loop_01
```

**방법 B: 쉘 스크립트 사용**

```bash
bash ~/go2_ws/scripts/record_mapping.sh
# 또는 저장 디렉토리 지정:
bash ~/go2_ws/scripts/record_mapping.sh /media/usb/bags
```

### Step 2 — 오프라인 SLAM 처리

```bash
# bag 자동 재생 + LIO-SAM 실행
ros2 launch go2_liosam2 run_liosam2_offline.launch.py \
    bag_path:=~/mapping_bags/mapping_20250101_120000

# PCD 저장 포함
ros2 launch go2_liosam2 run_liosam2_offline.launch.py \
    bag_path:=~/mapping_bags/mapping_20250101_120000 \
    save_pcd:=true \
    save_pcd_dir:=~/maps/campus_loop_01
```

### Step 3 — 결과 시각화

```bash
rviz2 -d $(ros2 pkg prefix lio_sam)/share/lio_sam/config/rviz2.rviz
```

---

## 파라미터 파일 위치

```
config/params_go2_hesai_xt16.yaml
```

주요 튜닝 파라미터:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `N_SCAN` | 16 | XT16 채널 수 |
| `Horizon_SCAN` | 1800 | 방위각 해상도 (360°/0.2°) |
| `historyKeyframeFitnessScore` | 0.3 | Loop closure ICP 품질 임계값 |
| `loopClosureEnableFlag` | true | Loop closure 활성화 |
| `savePCD` | false | PCD 저장 여부 |

---

## 패키지 구조

```
go2_liosam2/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── params_go2_hesai_xt16.yaml   # LIO-SAM 파라미터
├── launch/
│   ├── record_mapping.launch.py     # 데이터 수집용
│   └── run_liosam2_offline.launch.py # 오프라인 SLAM 처리용
└── README.md
```

---

## 문제 해결

### IMU 초기화 실패

Go2 IMU는 기동 직후 잠시 불안정할 수 있습니다.  
로봇을 평평한 바닥에 정지시킨 상태로 5~10초 후 이동을 시작하세요.

### Loop closure 미발동

- 같은 공간을 지나는 루프를 충분히 형성해야 합니다.
- `historyKeyframeSearchRadius` 값을 환경에 맞게 조정하세요.

### 포인트클라우드 왜곡

- `use_timestamp_type: 0` (포인트별 타임스탬프)으로 설정해야 왜곡 보정이 가능합니다.
- `HesaiLidar_ROS_2.0/config/config.yaml`의 `use_timestamp_type`을 **0**으로 변경하세요.
