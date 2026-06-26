# LiDAR-Vision V4.1 Coordinate Validation

## 목적

V4.1은 사람 검출이나 동적 장애물 추적을 수행하지 않는다.

다음 항목을 독립적으로 검증한다.

1. Hesai XT16 원본 좌표가 Go2 body 좌표로 올바르게 변환되는가
2. `/sportmodestate`의 roll/pitch를 이용한 중력 정렬이 정상인가
3. 지면 normal이 보정 후 `[0, 0, 1]`에 가까워지는가
4. 전방·좌측·우측·후방 고정 물체의 좌표 부호가 올바른가
5. Go2 자세를 변경해도 gravity-aligned BEV에서 고정 물체 위치가 유지되는가

## 입력

- `/lidar_points`
  - Hesai XT16 PointCloud2
  - 64,000 points
  - 약 10 Hz
  - frame: `hesai_lidar`

- `/sportmodestate`
  - `unitree_go/msg/SportModeState`
  - 약 300 Hz
  - `imu_state.rpy` 사용

## 출력 화면

왼쪽:

```text
RAW BODY-FRAME BEV
```

- 고정 LiDAR extrinsic만 적용
- Go2 body roll/pitch가 그대로 포함됨

오른쪽:

```text
GRAVITY-ALIGNED BEV
```

- Go2 IMU roll/pitch 보정 적용
- yaw는 유지

빨간 점:

```text
추정 지면보다 일정 높이 이상인 장애물 포인트
```

## 파일 적용

```bash
cp /mnt/data/lidar_vision_topview_v4_1_node.py \
~/go2_ws/src/lidar_vision/lidar_vision/lidar_vision_topview_v4_1_node.py

cp /mnt/data/lidar_vision_topview_v4_1.launch.py \
~/go2_ws/src/lidar_vision/launch/lidar_vision_topview_v4_1.launch.py

cp /mnt/data/setup_v4_1.py \
~/go2_ws/src/lidar_vision/setup.py

cp /mnt/data/package_v4_1.xml \
~/go2_ws/src/lidar_vision/package.xml

chmod +x \
~/go2_ws/src/lidar_vision/lidar_vision/lidar_vision_topview_v4_1_node.py
```

## 빌드

```bash
cd ~/go2_ws
source /opt/ros/foxy/setup.bash

rm -rf build/lidar_vision
rm -rf install/lidar_vision
rm -rf log/latest
rm -rf log/latest_build

colcon build \
  --packages-select lidar_vision \
  --symlink-install

source install/setup.bash
```

확인:

```bash
ros2 pkg executables lidar_vision | grep v4_1
```

정상 출력:

```text
lidar_vision lidar_vision_topview_v4_1_node
```

## 실행

Go2 및 Hesai 드라이버 실행 후:

```bash
cd ~/go2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 launch lidar_vision \
  lidar_vision_topview_v4_1.launch.py
```

종료:

```text
q 또는 ESC
```

## 첫 번째 검증

로봇은 정지시킨다.

다음 값이 화면 상단과 터미널에 나타나는지 확인한다.

```text
roll
pitch
yaw
IMU age
cloud age
raw ground normal
aligned ground normal
```

정상 기준:

```text
IMU age < 200 ms
cloud age < 약 300 ms
aligned ground normal의 z가 raw보다 1.0에 가까움
aligned ground tilt가 raw보다 작아짐
```

## 두 번째 검증: 전후좌우 1 m

고정 상자를 한 번에 한 방향에 둔다.

| 실제 위치 | 기대 좌표 |
|---|---|
| 전방 1 m | x ≈ +1.0, y ≈ 0.0 |
| 왼쪽 1 m | x ≈ 0.0, y ≈ +1.0 |
| 오른쪽 1 m | x ≈ 0.0, y ≈ -1.0 |
| 후방 1 m | x ≈ -1.0, y ≈ 0.0 |

Top-view 표시 규칙:

```text
화면 위쪽 = 전방 +X
화면 왼쪽 = 왼쪽 +Y
화면 오른쪽 = 오른쪽 -Y
화면 아래쪽 = 후방 -X
```

## 세 번째 검증: 자세 변화

동일한 고정 상자를 움직이지 않고 Go2 자세만 변경한다.

비교:

```text
RAW BODY-FRAME BEV
→ 자세 변화에 따라 지면과 물체 높이가 변할 수 있음

GRAVITY-ALIGNED BEV
→ 고정 물체의 XY 위치와 ground normal이 상대적으로 안정되어야 함
```

## 중력 정렬 방향이 반대로 보일 때

보정 후 ground tilt가 더 커지면 IMU 축 부호 또는 body frame 정의가 현재 가정과 반대일 수 있다.

Launch에서 먼저 다음 조합을 시험한다.

```python
"imu_roll_sign": -1.0,
"imu_pitch_sign": -1.0,
```

한 축씩 개별 시험한다.

```python
"imu_roll_sign": -1.0,
"imu_pitch_sign": 1.0,
```

또는:

```python
"imu_roll_sign": 1.0,
"imu_pitch_sign": -1.0,
```

판정 기준은 단순하다.

```text
gravity-aligned ground normal이 [0, 0, 1]에 가장 가까워지는 조합
```

## 이번 단계에서 하지 않는 것

- 사람 검출
- 동적 객체 분류
- foreground background subtraction
- Kalman tracking
- deskew
- ego-motion compensation
- Range Image segmentation

좌표계 검증이 완료된 뒤 V4.2에서 Range Image segmentation을 진행한다.
