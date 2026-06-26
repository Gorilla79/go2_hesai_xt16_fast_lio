# LiDAR-Vision V4.4 — XT16 Range Image

## 목적

Hesai XT16의 다음 필드를 이용하여 16 × 4000 Range Image를 생성한다.

```text
x
y
z
intensity
ring
timestamp
```

이번 단계에서는 객체 검출이나 사람 판단을 하지 않는다.

검증 대상:

```text
1. ring 0~15가 모두 표시되는가
2. 각 ring이 약 4000개 포인트를 가지는가
3. 360° azimuth가 끊기지 않는가
4. Range Image에서 벽·가구·사람 경계가 보이는가
5. invalid cell과 collision 수가 어느 정도인가
6. 한 scan의 timestamp span이 약 0.1초인가
```

## 파일 적용

```bash
cp /mnt/data/lidar_vision_range_image_v4_4_node.py \
~/go2_ws/src/lidar_vision/lidar_vision/lidar_vision_range_image_v4_4_node.py

cp /mnt/data/lidar_vision_range_image_v4_4.launch.py \
~/go2_ws/src/lidar_vision/launch/lidar_vision_range_image_v4_4.launch.py

cp /mnt/data/setup_v4_4.py \
~/go2_ws/src/lidar_vision/setup.py

chmod +x \
~/go2_ws/src/lidar_vision/lidar_vision/lidar_vision_range_image_v4_4_node.py
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
ros2 pkg executables lidar_vision | grep v4_4
```

정상 출력:

```text
lidar_vision lidar_vision_range_image_v4_4_node
```

## 실행

```bash
ros2 launch lidar_vision \
  lidar_vision_range_image_v4_4.launch.py
```

## 화면

상단:

```text
RANGE IMAGE
```

중단:

```text
INTENSITY IMAGE
```

하단:

```text
VALIDITY MASK
```

오른쪽:

```text
포인트 수
셀 점유율
collision 수
ring 분포
timestamp span
azimuth 범위
range 범위
build time
cloud age
```

## 정상 기준

```text
Ring count         = 16
Ring min/max       = 0 / 15
각 ring point 수   ≈ 4000
Timestamp span     ≈ 0.099~0.100 s
Unique timestamps  ≈ 500
Azimuth min/max    ≈ 0 / 360
```

Coverage는 모든 셀이 항상 채워지는 것은 아니므로 100%일 필요가 없다.

## 다음 단계

V4.4 검증 완료 후:

```text
V4.5
Range Image connected-component segmentation

V4.6
신체 조각 병합 및 3D 객체 후보 생성
```
