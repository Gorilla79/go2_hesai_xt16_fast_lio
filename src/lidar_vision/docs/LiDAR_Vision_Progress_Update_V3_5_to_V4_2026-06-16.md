---
title: "LiDAR-Vision 360° BEV 연구개발 진행 기록 업데이트"
project: "Unitree Go2 + Hesai XT16 + RealSense D435i 기반 360° 사람·동적 장애물 인지"
document_type: "연구개발 진행상황 및 코드 버전 관리 기록"
status: "V4 수정본 검증 완료, V4.1 좌표계·시간동기화 검증 단계 예정"
updated: "2026-06-16"
tags:
  - ROS2
  - Unitree-Go2
  - Hesai-XT16
  - RealSense-D435i
  - YOLO11-seg
  - LiDAR-Vision-Fusion
  - 360-BEV
  - Dynamic-Object-Detection
  - Kalman-Filter
  - Motion-Segmentation
---

# 1. 문서 목적

이 문서는 **기존 진행상황 정리 Markdown 파일 작성 이후부터 2026년 6월 16일까지 수행한 작업, 실험 결과, 문제점, 토론 내용, 기술적 판단 및 향후 계획**을 이어서 기록하기 위한 문서이다.

이 문서만 제공하더라도 다음 내용을 파악할 수 있도록 작성한다.

- 기존 V3 계열 이후 어떤 기능을 추가했는가
- 각 버전은 왜 제작되었는가
- 실제 실험에서 어떤 문제가 발견되었는가
- 단순 파라미터 조정으로 해결되지 않은 이유는 무엇인가
- 현재 코드의 정확한 상태는 어디까지인가
- 최종적으로 어떤 구조로 발전시켜야 하는가
- 다음 버전에서 무엇을 먼저 검증해야 하는가

---

# 2. 전체 연구 목표

## 2.1 최종 목표

Unitree Go2에 장착된 다음 센서를 활용한다.

- Hesai XT16 3D LiDAR
- Intel RealSense D435i
- Go2 내부 IMU 및 상태 정보
- 추후 odometry, SLAM, VLM 결과

최종적으로 다음 정보를 동시에 생성하는 것이 목표이다.

```text
로봇 상태
- 위치
- 자세
- 속도
- 이동 방향
- 계획 경로

사람 상태
- 위치
- 속도
- 이동 방향
- 추적 ID
- 사람 여부
- 접근·이탈 여부

주변 환경 상태
- 정적 장애물
- 동적 장애물
- 통행 가능 영역
- 충돌 위험
- 360° 점유 정보

상위 의미 정보
- VLM 상황 해석
- 위험 상황 판단
- 행동 정책
```

이 정보들을 **동일한 시간 기준으로 정렬하여 행동 결정과 회피 정책에 활용**하는 것이 장기 목표이다.

---

# 3. 기존 정리 이후 시작 상태

기존 정리 이후의 시작점은 다음과 같다.

## 3.1 완료되어 있던 내용

- RealSense–LiDAR 외부 파라미터 캘리브레이션
- YOLO11s-seg 기반 전방 사람 검출
- LiDAR 포인트를 YOLO 사람 mask 또는 bbox에 연관
- 사람의 로봇 중심 좌표 추정
- 2D Top-view 시각화
- 사람 track ID 유지
- 중복 사람 검출 완화
- LiDAR 신뢰도 분류
- 정면 중심 사람 인지 기능
- V3.4 LiDAR cluster candidate layer
- 먼 객체를 `UNKNOWN_OBSTACLE` 또는 `HUMAN_CANDIDATE`로 유지

## 3.2 당시 주요 한계

- 카메라 FOV 밖에서는 사람을 의미적으로 검출하지 못함
- 측면·후방 LiDAR 포인트는 보이지만 객체 추적이 없음
- 정적 구조물과 움직이는 객체를 구분하지 못함
- 로봇이 움직일 경우 정적 장애물을 동적으로 오인할 위험
- XT16의 희소한 수직 채널로 인해 단순 3D cluster의 안정성이 낮음

---

# 4. 버전별 진행 기록

# 4.1 V3.5 — XT16 기반 360° BEV 기초 구조

## 4.1.1 제작 목적

기존 전방 카메라 중심 구조에서 벗어나 **Hesai XT16의 360° 포인트를 로봇 중심 BEV로 표시하고, 로봇 주변 전체 장애물을 방향별로 파악**하기 위해 제작하였다.

## 4.1.2 구현 내용

- 360° Robot-Centered BEV
- FRONT / LEFT / RIGHT / REAR sector 분리
- 로봇 본체 주변 self-filter
- ground filter
- sector별 거리 및 위험도 계산
- LiDAR cluster candidate 생성
- XT16 특성에 맞춘 deterministic downsampling
- random point sampling 제거
- voxel downsampling 적용
- 방위각 bin 기반 sector 거리 안정화
- 거리별 cluster 최소 포인트 기준 적용
- 먼 거리 sparse cluster 유지
- 화면 범위 및 Top-view 개선

## 4.1.3 XT16 최적화 핵심

기존 무작위 샘플링:

```python
np.random.choice(...)
```

을 제거하고 다음 방식으로 변경하였다.

```text
Voxel Downsampling
+
Deterministic Stride Sampling
```

목적:

- 정지 환경에서도 포인트가 무작위로 흔들리는 현상 감소
- cluster 중심·크기의 프레임 간 변동 감소
- 동일 환경에서 동일한 포인트 구조 유지

## 4.1.4 실험 결과

- 360° 포인트는 표시됨
- 전방뿐 아니라 측면·후방 장애물 포인트도 확인 가능
- XT16의 최대 인식 거리는 기존 8m보다 확장됨
- 약 11~12m 부근의 사람 또는 객체까지 일부 거리 산출 가능
- 단순 비전-only 방식보다 먼 객체 대비가 가능해짐

## 4.1.5 발견된 문제

- 정지 상태에서도 cluster box가 프레임마다 달라짐
- 같은 구조물의 ID가 유지되지 않음
- 사람이 카메라 밖에서 움직여도 의미 있는 동적 객체로 유지되지 않음
- 여러 구조물이 큰 cluster로 묶이는 경우 발생
- 단순 cluster는 사람 여부를 확정할 수 없음

---

# 4.2 V3.6 — 360° LiDAR Cluster Temporal Tracking

## 4.2.1 제작 목적

V3.5에서는 cluster가 매 프레임 새로 생성되었다.  
따라서 **동일한 LiDAR 객체를 시간적으로 연결하고 고정 ID를 유지**하기 위해 V3.6을 제작하였다.

## 4.2.2 구현 내용

각 LiDAR cluster에 다음 정보를 추가하였다.

```text
cluster_track_id
track_state
track_age
hit_count
missed_frames
first_seen_time
last_seen_time
center
size
sector
```

상태:

```text
TENTATIVE
CONFIRMED
STALE
```

## 4.2.3 매칭 방식

이전 프레임 track과 현재 cluster를 다음 기준으로 매칭하였다.

- XY 중심 거리
- cluster 크기 차이
- sector
- match distance gate

동일 객체로 판단되면 같은 ID를 유지하였다.

## 4.2.4 실험 결과

- 정적 구조물에 동일 ID를 일정 시간 유지할 수 있음
- 일시적으로 포인트가 누락되면 `STALE` 상태로 유지
- 측면·후방 cluster도 ID를 부여할 수 있음
- 기존보다 박스가 안정화됨

## 4.2.5 중요한 판단

`CONFIRMED`는 동적 객체라는 뜻이 아니다.

```text
CONFIRMED
= 동일 cluster가 여러 프레임 유지됨

STALE
= 현재는 놓쳤지만 track을 일정 시간 유지함
```

V3.6에는 실제 속도, 방향, 동적·정적 분류가 없었다.

## 4.2.6 발견된 문제

- 사람이 움직여도 `MOVING` 상태가 생성되지 않음
- cluster ID는 유지되지만 실제 움직임 분석이 없음
- 움직이는 사람의 cluster가 구조물과 합쳐질 경우 추적이 불안정
- 카메라 밖의 움직임을 사람 또는 동적 객체로 표현하지 못함

---

# 4.3 V4 초기 버전 — Kalman + Hungarian + Temporal BEV

## 4.3.1 제작 목적

V3.6의 cluster ID 유지 기능을 기반으로 **정지한 로봇 기준의 360° 상대 동적 객체 검출**을 구현하기 위해 제작하였다.

## 4.3.2 구현 목표

```text
360° Cluster Tracking
+
Constant-Velocity Kalman Filter
+
Hungarian Matching
+
Temporal BEV Occupancy Change
+
Multi-frame Motion Confirmation
```

## 4.3.3 Kalman Filter 상태

각 LiDAR track의 상태를 다음과 같이 정의하였다.

```text
[x, y, vx, vy]
```

- `x, y`: 로봇 기준 위치
- `vx, vy`: 로봇 기준 속도

예측:

```text
x(t+1) = x(t) + vx × dt
y(t+1) = y(t) + vy × dt
```

## 4.3.4 Hungarian Matching

이전 track과 현재 cluster 간 전체 비용 행렬을 생성한 뒤, 가능한 경우 Hungarian assignment를 사용하였다.

비용 요소:

- 예측 위치와 측정 위치 간 거리
- cluster 크기 차이
- sector 차이
- 이동 track의 확장 gate

SciPy가 없는 경우 greedy matching으로 fallback하도록 구성하였다.

## 4.3.5 Motion 상태

```text
UNKNOWN
STATIC_RELATIVE
POSSIBLE_MOVING
MOVING_RELATIVE
STALE
RELATIVE_UNKNOWN
```

## 4.3.6 동적 판단 증거

- 추정 속도
- 누적 이동 거리
- 방향 일관성
- BEV 변화 셀 수
- track hit 수
- 다중 프레임 확인

## 4.3.7 초기 실험 결과

초기 V4는 동작하지 않았다.

화면에서는 큰 구조물 전체가 하나의 `STATIC_RELATIVE` cluster로 표시되었다.

원인:

```text
사람 포인트
+
의자
+
책상
+
벽
→ 하나의 큰 cluster
→ STRUCTURE_LIKE
→ motion 판단에서 제외
```

즉, 사람이 실제로 움직여도 정적 구조물에 포함되어 동적 객체가 되지 못했다.

## 4.3.8 결론

속도 임계값을 낮추는 것만으로는 해결할 수 없었다.  
전체 포인트를 먼저 cluster로 묶는 구조 자체가 문제였다.

---

# 4.4 V4 수정본 — 정적 배경 기반 Foreground Motion Layer

## 4.4.1 제작 목적

사람이 가구·벽과 하나의 큰 cluster로 합쳐지는 문제를 해결하기 위해, **정적 배경을 먼저 학습하고 배경과 다른 포인트만 동적 후보로 추출**하도록 구조를 변경하였다.

## 4.4.2 변경된 파이프라인

```text
Hesai XT16 Point Cloud
        ↓
Self-filter
        ↓
Ground Filter
        ↓
Stationary Background Model
        ↓
Foreground Point Extraction
        ↓
Foreground Cluster
        ↓
Kalman + Track ID
        ↓
POSSIBLE_MOVING
        ↓
MOVING_RELATIVE
```

## 4.4.3 배경 학습

초기 일정 프레임 동안 정적 환경을 학습한다.

```text
WARMUP 1/20
...
WARMUP 20/20
READY
```

초기 학습 중에는 주변에 움직이는 사람이 없어야 한다.

## 4.4.4 Foreground 판단

현재 BEV cell이 정적 배경 확률보다 낮으면 foreground로 판단하였다.

주요 파라미터:

```python
motion_background_warmup_frames = 20
motion_background_learning_rate = 0.08
motion_background_threshold = 0.65

motion_foreground_resolution = 0.12
motion_foreground_cluster_resolution = 0.18
motion_foreground_min_points = 4
motion_foreground_min_cells = 2
motion_foreground_max_extent = 1.80
```

## 4.4.5 수정 후 실험 결과

수정 후에는 처음으로 카메라 FOV 밖에서 LiDAR 기반 움직임이 검출되었다.

화면에 다음 정보가 표시됨:

```text
LiDAR Motion [READY]
moving = 1~3
possible = 0
stale = 여러 개
```

사람이 로봇 왼쪽에서 전방으로 이동할 때 `MOVING_RELATIVE` 객체가 생성되었다.

## 4.4.6 성과

- LiDAR-only 움직임 검출 자체는 성공
- 카메라 시야 밖에서도 foreground 생성
- 측면 동적 후보 표시
- Kalman 속도 추정 및 화살표 표시
- 동적 후보를 배경 구조물과 분리하기 시작함

## 4.4.7 새롭게 발견된 핵심 문제

사람은 1명이지만 여러 개의 움직임 객체가 동시에 생성되었다.

예:

```text
L22 MOVING_RELATIVE
L24 MOVING_RELATIVE
L28 MOVING_RELATIVE
L40 MOVING_RELATIVE
...
```

또한 실제 사람이 움직인 위치와 다른 곳에 동적 객체가 표시되었다.

---

# 5. 현재 실험에서 확인된 문제

# 5.1 한 사람을 여러 객체로 분리

한 사람의 LiDAR 포인트가 다음과 같이 나뉜다.

```text
왼쪽 다리
오른쪽 다리
몸통
팔
이동 잔상
배경에서 새롭게 드러난 포인트
```

이 조각들이 각각 cluster가 되어 여러 track ID가 생성된다.

결과:

```text
실제 사람 1명
→ LiDAR 동적 객체 3~7개
```

이는 인원 수 추정과 회피 정책에 심각한 오류를 유발할 수 있다.

---

# 5.2 실제 위치와 표시 위치가 다름

사용자는 로봇을 낮게 엎드린 상태로 두고 다음 경로로 이동하였다.

```text
로봇 전방 기준 왼쪽
→ 정면
→ 다시 왼쪽
```

실제 이동 거리는 약 1~2m였으나, BEV에서는 다른 방향과 먼 위치에도 동적 객체가 표시되었다.

가능한 원인:

- 고정 LiDAR extrinsic만 사용
- 로봇 몸체 roll/pitch 변화 미반영
- IMU 기반 gravity alignment 부재
- LiDAR scan deskew 부재
- PointCloud2 전체를 동일 시각 데이터로 가정
- foreground 잔상
- STALE track이 오래 유지됨
- 2D BEV 셀 기반 cluster fragmentation

---

# 5.3 XT16 회전형 LiDAR의 시간 왜곡

XT16은 한 프레임을 한 순간에 촬영하는 센서가 아니다.

10Hz 기준:

```text
한 바퀴 스캔 ≈ 0.1초
```

사람이 이동하면 한 scan 안에 다음 정보가 섞일 수 있다.

```text
스캔 시작 시점의 다리
중간 시점의 몸통
종료 시점의 팔
```

결과:

- 사람 형상이 늘어남
- 포인트가 여러 조각으로 분리
- 실제 위치보다 퍼져 보임
- 여러 cluster 생성
- 속도 추정 오류

이를 해결하려면 포인트별 timestamp 기반 **deskew**가 필요하다.

---

# 5.4 로봇 자세 변화와 지면 좌표계 문제

현재 BEV는 사실상 다음을 가정한다.

```text
LiDAR XY 평면 = 실제 수평 지면
```

하지만 Go2가 엎드리거나 자세가 변하면 LiDAR의 roll/pitch가 변할 수 있다.

고정 보정값 예:

```text
lidar_roll_deg = -6.0
lidar_pitch_deg = 0.0
lidar_yaw_deg = 90.0
```

이 값만으로는 실시간 자세 변화를 반영할 수 없다.

필요 구조:

```text
lidar_frame
→ body_frame
→ base_stabilized
→ local_map
```

`base_stabilized`는 IMU roll/pitch를 제거한 중력 정렬 좌표계이다.

---

# 5.5 STALE 객체 누적

현재 여러 객체가 동시에 보이는 이유 중 하나는 과거 track이 `STALE` 상태로 남기 때문이다.

```text
현재 foreground cluster
+
이전 위치의 STALE track
+
분리된 신체 조각
+
배경 foreground noise
```

가 동시에 표시된다.

현재 V4 수정본은 motion 검출 검증용이므로 디버그 정보가 많다.

실제 제어용에서는 다음만 사용해야 한다.

```text
사용:
CONFIRMED + MOVING_RELATIVE

주의:
POSSIBLE_MOVING

제외:
STALE
UNKNOWN
TENTATIVE
```

---

# 6. 자율주행 자동차형 LiDAR BEV와 현재 방식의 차이

현재 방식:

```text
포인트 변화
→ foreground cell
→ cluster
→ track
```

자율주행 차량의 일반적인 3D perception 구조:

```text
LiDAR Packet
        ↓
Timestamp Synchronization
        ↓
Deskew
        ↓
IMU / Odometry Motion Compensation
        ↓
Ground Segmentation
        ↓
3D Object Detection
        ↓
3D Bounding Box
        ↓
Multi-Object Tracking
        ↓
Velocity / Heading / Trajectory
        ↓
BEV / Planning
```

핵심 차이:

- 현재는 변한 포인트를 객체로 가정
- 자율주행 방식은 먼저 객체 단위 3D box를 생성
- Kalman filter는 포인트 조각을 사람으로 만드는 기능이 아님
- 검출된 객체 박스를 시간적으로 연결하는 기능임

---

# 7. 논의된 개선 방향

# 7.1 단기: 기하 기반 파이프라인 개선

학습 데이터 없이 단계별로 검증 가능한 방식이다.

```text
Deskew
→ Gravity Alignment
→ Ground Removal
→ Range Image Segmentation
→ 3D Connected Components
→ Fragment Merge
→ Person-Sized Candidate
→ Kalman + Hungarian Tracking
```

장점:

- 학습 데이터 불필요
- 오류 원인 분석 용이
- XT16에 직접 맞춤 가능
- Jetson Orin NX 부담이 낮음

한계:

- 사람과 카트·의자 구분이 제한적
- 복잡한 군중에서 정확도 한계
- 의미 분류는 카메라 또는 학습 모델 필요

---

# 7.2 중기: Range Image 기반 XT16 segmentation

XT16은 16채널 회전형 LiDAR이다.

Range Image 구조:

```text
행 = LiDAR ring 16개
열 = azimuth
값 = range
```

인접 ring·azimuth의 거리 차이를 이용하여 객체를 분리한다.

장점:

- XT16 채널 구조를 직접 활용
- 단순 XY BEV보다 공간 연결성이 자연스러움
- 한 사람의 몸통·다리 조각을 더 안정적으로 연결 가능
- 벽과 사람의 range discontinuity 활용 가능

필요 데이터:

- `ring`
- `time` 또는 `timestamp`
- `intensity`
- `x, y, z`

---

# 7.3 장기: 3D Object Detection

후보 모델:

- PointPillars
- CenterPoint
- SECOND
- VoxelNeXt

권장 방향:

```text
XT16 실내 데이터 수집
→ 사람 3D box 라벨링
→ custom detector 학습
→ TensorRT 변환
→ Jetson Orin NX 배포
```

사전학습 모델은 주로 KITTI, nuScenes, Waymo 등 실외 자동차 데이터 기반이므로, 실내 XT16 사람 검출에 그대로 적용하면 성능이 보장되지 않는다.

---

# 7.4 Camera Semantic Association

카메라 시야 안:

```text
LiDAR 3D Track
+
YOLO Person Mask
→ CONFIRMED_HUMAN
```

카메라 시야 밖:

```text
LiDAR Moving Track
→ MOVING_UNKNOWN
또는
PEDESTRIAN_SHAPED_CANDIDATE
```

카메라 밖 LiDAR 객체를 무조건 사람으로 확정하면 안 된다.

---

# 8. 현재 코드 상태

## 8.1 최신 실행 버전

현재 마지막으로 실행한 코드는 다음이다.

```text
V4 수정본
- Stationary Background Model
- Foreground Point Extraction
- Foreground Cluster
- Kalman Filter
- Hungarian Matching
- Temporal BEV
- MOVING_RELATIVE 표시
```

실제 파일명:

```text
lidar_vision_topview_v4_node.py
lidar_vision_topview_v4.launch.py
setup.py
```

수정본 원본 생성 파일:

```text
lidar_vision_topview_v4_node_corrected.py
lidar_vision_topview_v4_corrected.launch.py
setup_v4_corrected.py
```

## 8.2 현재 성공한 기능

- 360° point cloud BEV
- sector별 위험도
- 전방·측면·후방 장애물 거리
- 전방 YOLO 사람 검출
- LiDAR–camera 사람 위치 연관
- 360° LiDAR cluster 후보
- cluster track ID
- Kalman 위치·속도 상태
- foreground 기반 LiDAR-only 움직임 검출
- 카메라 FOV 밖 `MOVING_RELATIVE` 표시

## 8.3 아직 해결되지 않은 기능

- 한 사람을 하나의 LiDAR 객체로 병합
- 정확한 사람 위치
- 정확한 이동 속도
- 정확한 이동 방향
- scan deskew
- IMU gravity alignment
- 동적 자세 대응 calibration
- ring 기반 range-image segmentation
- 로봇 이동 시 ego-motion compensation
- 실제 정적·동적 분류
- 여러 사람 분리
- LiDAR-only 사람 의미 분류
- 제어 정책 연결

---

# 9. 다음 단계: V4.1 검증 버전

현재 바로 다음에 수행해야 할 작업은 **동적 알고리즘 추가가 아니라 좌표계와 원천 데이터 품질 검증**이다.

## 9.1 V4.1 목표

```text
1. PointCloud2 field 확인
2. ring 필드 확인
3. point별 time/timestamp 확인
4. IMU roll/pitch 확인
5. ground plane 확인
6. 4방향 기준 물체 좌표 검증
7. 현재 posture 변화에 따른 좌표 변동 측정
8. self-filter 실제 범위 검증
```

---

# 10. V4.1 세부 시험 계획

# 10.1 PointCloud2 필드 확인

명령:

```bash
ros2 topic info /lidar_points -v
```

또는:

```bash
ros2 topic echo /lidar_points --once
```

확인할 필드:

```text
x
y
z
intensity
ring
time
timestamp
t
```

`ring`과 `time`이 있어야 range-image 및 deskew 구현이 쉬워진다.

---

# 10.2 4방향 고정 물체 시험

로봇을 정지한 상태로 상자를 다음 위치에 둔다.

```text
전방 1.0m
왼쪽 1.0m
오른쪽 1.0m
후방 1.0m
```

정상 좌표:

```text
전방: +X
왼쪽: +Y
오른쪽: -Y
후방: -X
```

허용 초기 오차:

```text
약 0.10~0.20m
```

방향이 반대거나 큰 오차가 발생하면 알고리즘 문제가 아니라 transform 문제이다.

---

# 10.3 자세 변화 시험

동일 상자를 두고 다음 자세를 비교한다.

```text
Go2 정상 기립
Go2 낮은 자세
Go2 엎드린 자세
```

같은 상자의 BEV 위치가 변하면:

```text
고정 extrinsic만으로는 부족
→ IMU gravity alignment 필요
```

---

# 10.4 Ground Plane 확인

수평 바닥의 point cloud를 이용해 plane normal을 추정한다.

정상적으로 수평화되면:

```text
ground normal ≈ +Z
```

roll/pitch 보정 전후의 normal을 비교한다.

---

# 11. 향후 버전 계획

## V4.1 — 좌표계·센서 데이터 검증

- ring/time field 확인
- IMU 자세 표시
- ground plane normal
- 4방향 기준 좌표
- calibration 검증
- posture 변화 비교

## V4.2 — Gravity Alignment + Ground Stabilization

- IMU roll/pitch 반영
- `base_stabilized` 좌표계 생성
- ground plane 기반 보정
- 자세 변화에도 동일 BEV 좌표 유지

## V4.3 — LiDAR Deskew

- 포인트별 timestamp 사용
- 회전 scan 시간 왜곡 보정
- 로봇 정지 상태에서도 사람 이동 잔상 감소
- 추후 로봇 이동 시 odometry/IMU 보정 연결

## V4.4 — XT16 Range Image Segmentation

- 16 ring × azimuth range image
- ground segmentation
- range discontinuity 기반 연결 성분
- 3D object cluster 생성

## V4.5 — Fragment Merge + Person-Sized Candidate

- 인접 cluster 병합
- 높이 구간 중첩
- 중심 거리
- 속도 방향 유사도
- 동일 시간 생성 여부
- 사람 크기 후보 필터

## V4.6 — 안정적 3D Multi-Object Tracking

- 3D bounding box
- Kalman prediction
- Hungarian assignment
- ID 유지
- 속도·방향
- track lifecycle

## V5 — Ego-Motion Compensation

로봇 이동량을 반영한다.

```text
이전 Point Cloud / Track
→ odom + IMU로 현재 좌표계 변환
→ 현재 관측과 비교
```

목적:

- 로봇 이동 때문에 벽이 동적으로 보이는 문제 방지
- 실제 정적·동적 객체 구분

## V6 — Semantic Fusion

- YOLO/VLM 의미 정보
- LiDAR 3D track
- 사람·카트·장애물 구분
- 위험도
- TTC
- 행동 정책 입력

## V7 — 원격 시각화

알고리즘이 안정된 후:

```text
Go2 / Jetson
→ ROS 2
→ Zenoh Bridge
→ Remote PC
→ BEV / Track / Risk Visualization
```

현재 로봇 내부에서 생성하는 OpenCV GUI는 최종적으로 원격 시각화 구조로 변경한다.

---

# 12. 파일 및 코드 생성 규칙

## 12.1 버전 규칙

큰 기능이 추가되면 단순 버전 증가:

```text
V3
V4
V5
```

세부 검증 단계는 필요한 경우:

```text
V4.1
V4.2
V4.3
```

현재 V4는 기능이 너무 크게 확장되었으므로, 이후 좌표계 검증 단계부터는 `V4.1` 형식을 사용하는 것이 적절하다.

## 12.2 파일명 규칙

Node:

```text
lidar_vision_topview_v4_1_node.py
```

Launch:

```text
lidar_vision_topview_v4_1.launch.py
```

Console script:

```text
lidar_vision_topview_v4_1_node
```

## 12.3 기존 버전 보존

이전 파일을 덮어쓰기 전에 반드시 보존한다.

```text
V3.5 유지
V3.6 유지
V4 초기 유지
V4 수정본 유지
```

검증되지 않은 새 코드를 안정 버전에 덮어쓰지 않는다.

## 12.4 전체 코드 제공 원칙

사용자는 부분 patch보다 전체 코드를 선호한다.

새 버전 작성 시 제공 대상:

```text
Node 전체 코드
Launch 전체 코드
setup.py 전체 코드
필요 시 package.xml
빌드 명령
실행 명령
검증 절차
예상 출력
오류 판단 기준
```

## 12.5 코드 주석 및 로그

- 코드 주석: 영어
- 터미널 로그: 영어
- 설명 문서: 한국어
- 파라미터명: 영어
- 상태명: 영어 대문자

## 12.6 단계별 구현 원칙

한 버전에 기능을 과도하게 넣지 않는다.

```text
한 단계
→ 한 핵심 기능
→ 실험
→ 이상 여부 확인
→ 다음 단계
```

문제 발생 시 어느 기능에서 발생했는지 명확히 구분할 수 있어야 한다.

---

# 13. 현재 기술적 판단

## 13.1 V4 수정본을 폐기할 필요는 없음

V4 수정본은 최종 알고리즘은 아니지만 다음을 확인하는 데 성공했다.

- 카메라 밖에서도 LiDAR만으로 움직임 변화 검출 가능
- 정적 배경 제거가 필요함
- 단순 XY foreground cluster만으로 사람 단위 객체가 만들어지지 않음
- track이 여러 조각으로 분리됨
- 자세·시간 왜곡 문제가 실제 오차에 영향을 줄 가능성이 큼

따라서 V4는 **실패 코드가 아니라 다음 구조 전환 필요성을 확인한 실험 버전**으로 보존한다.

## 13.2 지금 파라미터만 계속 조정하면 안 되는 이유

현재 문제는 단순 threshold 문제가 아니다.

```text
원천 좌표 불안정
+
deskew 부재
+
2D cluster fragmentation
+
한 사람의 신체 조각 분리
+
STALE 누적
```

따라서 다음 순서는 반드시:

```text
좌표 검증
→ 수평화
→ 시간 보정
→ range-image segmentation
→ object merge
→ tracking
```

이어야 한다.

---

# 14. 최종 요약

현재까지의 성과:

```text
전방 사람 인지
→ LiDAR 거리 결합
→ 사람 ID 유지
→ 360° BEV
→ XT16 최적화
→ 360° cluster tracking
→ Kalman 기반 상대 속도
→ foreground 기반 LiDAR-only motion detection
```

현재 핵심 문제:

```text
한 사람을 여러 동적 객체로 분리
실제 위치와 다른 위치에 표시
로봇 자세 변화 미반영
LiDAR scan deskew 부재
2D BEV cluster의 구조적 한계
```

현재 판단:

```text
V4 motion detection 자체는 시작되었음
하지만 사람 단위 3D object perception은 아직 완성되지 않음
```

다음 작업:

```text
V4.1
좌표계·PointCloud2 field·IMU·ground plane 검증
```

최종 목표 구조:

```text
XT16 Raw Scan
→ Time Synchronization
→ Deskew
→ IMU Gravity Alignment
→ Ground Removal
→ Range Image Segmentation
→ 3D Object Cluster
→ Fragment Merge
→ 3D Tracking
→ Camera Semantic Fusion
→ 360° BEV
→ Risk and Motion Policy
→ Zenoh Remote Visualization
```
