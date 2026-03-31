# ~/go2_ws/scripts/record_mapping.sh
#!/bin/bash

SAVE_DIR="$HOME/mapping_bags"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="mapping_${TIMESTAMP}"

mkdir -p "$SAVE_DIR"

echo "=========================================="
echo "  GO2 Mapping Bag Recorder"
echo "  저장 위치: ${SAVE_DIR}/${BAG_NAME}"
echo "  녹화 토픽:"
echo "    - /hesai/points (LiDAR)"
echo "    - /imu/data (IMU)"
echo "    - /odom (Odometry)"
echo "    - /tf, /tf_static"
echo ""
echo "  Ctrl+C 로 녹화 종료"
echo "=========================================="
echo ""

ros2 bag record \
  /hesai/points \
  /imu/data \
  /odom \
  /tf \
  /tf_static \
  -o "${SAVE_DIR}/${BAG_NAME}" \
  --compression-mode file \
  --compression-format zstd
