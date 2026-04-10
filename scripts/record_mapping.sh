# ~/go2_ws/scripts/record_mapping.sh
#!/bin/bash
#
# LIO-SAM2 용 rosbag 녹화 스크립트 (Go2 + Hesai XT16)
#
# 올바른 토픽:
#   /lidar_points          — Hesai XT16 포인트클라우드
#   /utlidar/imu_corrected — Go2 내장 IMU (보정값)
#   /utlidar/imu           — Go2 내장 IMU (raw 백업)
#   /tf, /tf_static        — TF 트리

SAVE_DIR="${1:-$HOME/mapping_bags}"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="mapping_${TIMESTAMP}"

mkdir -p "$SAVE_DIR"

echo "=========================================="
echo "  GO2 Mapping Bag Recorder (LIO-SAM2)"
echo "  저장 위치: ${SAVE_DIR}/${BAG_NAME}"
echo "  녹화 토픽:"
echo "    - /lidar_points          (Hesai XT16 포인트클라우드)"
echo "    - /utlidar/imu_corrected (Go2 IMU 보정값)"
echo "    - /utlidar/imu           (Go2 IMU raw)"
echo "    - /tf, /tf_static"
echo ""
echo "  사용법: $0 [저장_디렉토리]"
echo "  Ctrl+C 로 녹화 종료"
echo "=========================================="
echo ""

ros2 bag record \
  /lidar_points \
  /utlidar/imu_corrected \
  /utlidar/imu \
  /tf \
  /tf_static \
  -o "${SAVE_DIR}/${BAG_NAME}" \
  --compression-mode file \
  --compression-format zstd
