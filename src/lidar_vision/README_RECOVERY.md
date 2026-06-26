# lidar_vision 복구 패키지

이 폴더는 대화 중 실제로 생성·보존된 파일을 기반으로 재구성한 ROS 2 Foxy `ament_python` 패키지입니다.

## 포함 범위
- V1 ~ V4.4 주요 top-view/BEV 노드
- V3.5 XT16 optimized
- V4 corrected
- V4.3 optimized 최신본
- V4.4 XT16 range-image 진단본
- RealSense/LiDAR calibration 도구
- 현재/이전 calibration YAML
- launch, setup, package metadata
- 진행 기록과 가이드

## 주의
원래 삭제된 디렉터리의 byte-for-byte 복원은 아닙니다.
하지만 현재 확보된 실제 소스 파일을 기반으로 빌드 가능한 패키지 구조를 다시 구성했습니다.
`build/`, `install/`, `log/`는 복구 대상이 아니며 다시 생성합니다.

## 복구
압축을 해제한 뒤:

```bash
chmod +x restore_and_build.sh
./restore_and_build.sh
```

또는 수동으로 `~/go2_ws/src/lidar_vision`에 복사하고 colcon build를 수행합니다.
