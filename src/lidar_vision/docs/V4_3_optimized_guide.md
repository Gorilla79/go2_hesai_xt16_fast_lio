# LiDAR-Vision V4.3 Optimized

파일명과 실행 파일명은 기존 Optimized 버전을 그대로 유지한다.

이번 수정 내용은 기존 파일에 그대로 반영한다.

```text
lidar_vision_topview_v4_3_optimized_node.py
lidar_vision_topview_v4_3_optimized.launch.py
setup_v4_3_optimized.py
```

## 반영된 최적화

- NumPy 기반 PointCloud2 고속 파싱
- 정수 hash 기반 voxel downsampling
- 표시 포인트 8,000개 제한
- ground fitting 포인트 1,500개 제한
- Raw ground 계산 주기 완화
- Aligned ground 계산 주기 완화
- 클릭 반경 0.25m
- 클릭 최소 포인트 8개
- Parse / Voxel / Ground / Render / Pipeline 시간 표시

## 적용

```bash
cp /mnt/data/lidar_vision_topview_v4_3_optimized_node.py \
~/go2_ws/src/lidar_vision/lidar_vision/lidar_vision_topview_v4_3_optimized_node.py

cp /mnt/data/lidar_vision_topview_v4_3_optimized.launch.py \
~/go2_ws/src/lidar_vision/launch/lidar_vision_topview_v4_3_optimized.launch.py

cp /mnt/data/setup_v4_3_optimized.py \
~/go2_ws/src/lidar_vision/setup.py

chmod +x \
~/go2_ws/src/lidar_vision/lidar_vision/lidar_vision_topview_v4_3_optimized_node.py
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

## 실행

```bash
ros2 launch lidar_vision \
  lidar_vision_topview_v4_3_optimized.launch.py
```
