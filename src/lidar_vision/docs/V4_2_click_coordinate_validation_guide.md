# LiDAR-Vision V4.2 Click-Based Coordinate Validation

## 목적

V4.2는 사람 검출이나 동적 장애물 추적을 수행하지 않는다.

다음 항목을 검증한다.

1. Hesai XT16 좌표가 Go2 body 좌표로 올바르게 변환되는가
2. `/sportmodestate`의 roll/pitch로 gravity alignment가 정상 적용되는가
3. Raw BEV와 Gravity-Aligned BEV의 ground normal 차이가 개선되는가
4. 화면에서 특정 포인트 군집을 클릭했을 때 실제 `x, y, z` 좌표가 올바르게 출력되는가
5. 전방·좌측·우측·후방 1 m 기준 물체의 좌표 부호가 올바른가

## 파일 적용

```bash
cp /mnt/data/lidar_vision_topview_v4_2_node.py \
~/go2_ws/src/lidar_vision/lidar_vision/lidar_vision_topview_v4_2_node.py

cp /mnt/data/lidar_vision_topview_v4_2.launch.py \
~/go2_ws/src/lidar_vision/launch/lidar_vision_topview_v4_2.launch.py

cp /mnt/data/setup_v4_2.py \
~/go2_ws/src/lidar_vision/setup.py

chmod +x \
~/go2_ws/src/lidar_vision/lidar_vision/lidar_vision_topview_v4_2_node.py
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
ros2 pkg executables lidar_vision | grep v4_2
```

정상 출력:

```text
lidar_vision lidar_vision_topview_v4_2_node
```

## 실행

```bash
cd ~/go2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

ros2 launch lidar_vision \
  lidar_vision_topview_v4_2.launch.py
```

## 화면 의미

왼쪽:

```text
RAW BODY-FRAME BEV
```

오른쪽:

```text
GRAVITY-ALIGNED BEV
```

빨간 점:

```text
추정 지면보다 높은 일반 장애물 포인트
```

큰 원형 표시는 제거되어 있다.  
방향별 최근접 장애물은 상단 텍스트로만 표시한다.

## 클릭 좌표 측정

확인하려는 포인트 군집을 마우스 왼쪽 버튼으로 클릭한다.

예:

```text
Probe ALIGNED: x=+0.12m y=+1.36m z=+0.48m points=16
```

좌표 의미:

```text
+x = 전방
+y = 왼쪽
-y = 오른쪽
-x = 후방
+z = 위쪽
```

터미널에는 클릭 반경 내 포인트 군집의 중앙값과 크기가 출력된다.

## 4방향 검증

| 실제 위치 | 기대 좌표 |
|---|---|
| 전방 1 m | x ≈ +1.0, y ≈ 0.0 |
| 왼쪽 1 m | x ≈ 0.0, y ≈ +1.0 |
| 오른쪽 1 m | x ≈ 0.0, y ≈ -1.0 |
| 후방 1 m | x ≈ -1.0, y ≈ 0.0 |

초기 허용 오차:

```text
약 0.10~0.20 m
```

## 성공 기준

1. `IMU OK`
2. aligned ground tilt가 raw ground tilt보다 작음
3. 고정 상자를 클릭했을 때 좌표 부호가 실제 방향과 일치
4. 동일 물체를 Raw/Aligned 화면에서 클릭했을 때 XY 위치가 크게 달라지지 않음
5. Go2 자세를 바꿔도 Aligned 좌표가 Raw보다 안정적임

## 다음 단계

V4.2 검증 완료 후:

```text
V4.3
XT16 ring 0~15 기반 Range Image 생성

V4.4
Range Image connected-component segmentation

V4.5
신체 조각 병합 및 사람 크기 후보 생성
```
