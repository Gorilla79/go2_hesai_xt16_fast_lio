# LiDAR-Vision V4.3 Single-BEV Coordinate Validation

## 변경 목적

V4.2에서는 Raw BEV와 Gravity-Aligned BEV를 나란히 표시했다.  
Go2의 roll·pitch가 수 도 이내일 때 두 XY BEV는 거의 동일하게 보이므로 비교 효율이 낮았다.

V4.3은 다음 구조로 변경한다.

```text
왼쪽
- Gravity-Aligned BEV 한 개만 크게 표시
- 클릭 가능한 실제 검증 화면

오른쪽
- IMU 상태
- Raw/Aligned ground normal
- 지면 tilt 개선량
- 클릭 위치의 Raw/Aligned 좌표
- Δx, Δy, Δz
```

V4.3은 아직 사람 검출이나 동적 객체 추적을 수행하지 않는다.

## 파일 적용

```bash
cp /mnt/data/lidar_vision_topview_v4_3_node.py \
~/go2_ws/src/lidar_vision/lidar_vision/lidar_vision_topview_v4_3_node.py

cp /mnt/data/lidar_vision_topview_v4_3.launch.py \
~/go2_ws/src/lidar_vision/launch/lidar_vision_topview_v4_3.launch.py

cp /mnt/data/setup_v4_3.py \
~/go2_ws/src/lidar_vision/setup.py

chmod +x \
~/go2_ws/src/lidar_vision/lidar_vision/lidar_vision_topview_v4_3_node.py
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
ros2 pkg executables lidar_vision | grep v4_3
```

## 실행

```bash
ros2 launch lidar_vision \
  lidar_vision_topview_v4_3.launch.py
```

## 사용

Gravity-Aligned BEV에서 확인할 포인트 군집을 클릭한다.

오른쪽 아래에 다음 값이 표시된다.

```text
RAW      x, y, z
ALIGNED  x, y, z
DELTA    dx, dy, dz
DISTANCE 중심 거리
EXTENT   클릭 주변 포인트 군집 크기
POINTS   클릭 반경 내 포인트 수
```

`C` 키:

```text
클릭 결과 초기화
```

`Q` 또는 `ESC`:

```text
종료
```

## 성공 기준

1. `IMU OK`
2. Aligned ground tilt가 Raw ground tilt보다 작음
3. 전방 물체 클릭 시 `x > 0`
4. 왼쪽 물체 클릭 시 `y > 0`
5. 오른쪽 물체 클릭 시 `y < 0`
6. 후방 물체 클릭 시 `x < 0`
7. 고정 물체의 측정 거리가 실측값과 약 0.10~0.20 m 이내

## 다음 단계

V4.3 좌표 검증 완료 후 다음 기능은 별도 버전에서 진행한다.

```text
V4.4
XT16 ring 0~15와 timestamp를 보존하는 Range Image 생성

V4.5
Range Image connected-component segmentation

V4.6
분리된 신체 조각 병합 및 3D 객체 후보 생성
```
