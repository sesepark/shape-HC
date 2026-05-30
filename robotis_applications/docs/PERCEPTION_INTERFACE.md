# Perception 팀 코드 인터페이스
> **최종 업데이트**: 2026-05-30  
> **기준 레포**: `~/AI_Worker_HC` (snu-shape/AI_Worker_HC)  
> **상태 범례**: ✅ 검증 완료 | 🔄 진행 중 | ⬜ 개발 예정

---

## 노드 목록 (4개)

| 노드 | 패키지 | 미션 A 역할 | 상태 |
|------|--------|------------|------|
| `monitor_ocr_node` | `monitor_ocr` | A.1 지령 모니터 OCR 파싱 | 🔄 YOLO 전환 중 |
| `detector_node` | `perception_part_detector` | A.2 부품 검출·분류 | 🔄 학습완료, 로봇 검증 필요 |
| `wrist_projection_node` | `perception_2d_to_pcd_wrist` | A.3 wrist 2D→3D 변환 | ✅ 검증 완료 |
| `wrist_pointcloud_node` | `perception_2d_to_pcd_wrist` | A.3 wrist PCD 발행 | ✅ 검증 완료 |

> **내부 파이프라인**: `detector_node` → `wrist_projection_node` → `wrist_pointcloud_node`  
> 이 연결은 Perception 팀 내부에서 이미 구현 완료. Manipulation 팀 PCD 품질 확인 완료.

---

## 노드별 상세

### 1. `monitor_ocr_node` — A.1 지령 파싱

**실행**
```bash
ros2 run monitor_ocr monitor_ocr_node --ros-args -p parts_mode:=true
```

**Subscribe**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/zed/zed_node/left/image_rect_color` | `sensor_msgs/msg/Image` | Head ZED 카메라 |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/monitor_ocr/result` | `std_msgs/msg/String` (JSON) | OCR 파싱 결과 |

**`/monitor_ocr/result` 메시지 구조 (JSON)**
```json
{
  "frames_used": 10,
  "parts": [
    {"name": "플랜지 너트", "count": 1},
    {"name": "기어 링",    "count": 2},
    {"name": "스페이서 링","count": 1},
    {"name": "육각 너트",  "count": 4},
    {"name": "돔 너트",    "count": 2}
  ],
  "latest_elapsed_ms": 1234.5,
  "latest_screen_detected": true
}
```

**System 팀 요청사항**
- `/monitor_ocr/result` → ROS 2 **Action** 구조로 감싸기 (OCR 처리 시간 고려)
- `/command_valid` 는 내부(internal)에서만 사용, 외부 미션 코드에서는 사용 안 함
- OCR 실패 시 fallback: 10초 딜레이 후 강제 OK 출력 (점수 10점 확보)

**현재 이슈**
- OpenCV 방식 실모니터 파싱 실패 → YOLO 기반 모니터 bbox 검출로 전환 중

---

### 2. `detector_node` — A.2 부품 검출·분류

**실행**
```bash
ros2 run perception_part_detector detector_node
```

**Subscribe**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/zed/zed_node/left/image_rect_color` | `sensor_msgs/msg/Image` | Head ZED |
| `/camera_left/camera_left/color/image_rect_raw` | `sensor_msgs/msg/Image` | Wrist Left |
| `/camera_right/camera_right/color/image_rect_raw` | `sensor_msgs/msg/Image` | Wrist Right |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/detections` | `perception_part_detector/msg/PartDetectionArray` | 부품 detection 배열 |
| `/detector_debug_image` | `sensor_msgs/msg/Image` | bbox/mask overlay 이미지 |

**`PartDetectionArray` 메시지 구조**
```
std_msgs/Header header
PartDetection[] detections
```

**`PartDetection` 필드**
```
int32   class_id
string  class_name        # flange_nut / gear_ring / spacer_ring / hex_nut / dome_nut
float32 confidence
int32[] bbox              # [x1, y1, x2, y2]
string  source_camera     # head / wrist_left / wrist_right

float32 center_x
float32 center_y

float32[] mask_x          # segmentation mask
float32[] mask_y
```

**5종 부품 class 목록**
| class_id | class_name | 한국어 |
|----------|-----------|--------|
| 0 | `flange_nut` | 플랜지 너트 |
| 1 | `gear_ring` | 기어 링 |
| 2 | `spacer_ring` | 스페이서 링 |
| 3 | `hex_nut` | 육각 너트 |
| 4 | `dome_nut` | 돔 너트 |

**모델 성능**: YOLO11s-seg, mAP50 = 0.993

**System 팀 요청사항**
- command target 부품 별도 표시 기능 추가 요청
- confidence threshold 이하 부품 별도 표시
- wrist cam 기준 추가 학습 반영 후 재검증 필요

---

### 3. `wrist_projection_node` — A.3 2D→3D 변환

**패키지**: `perception_2d_to_pcd_wrist`

**Subscribe**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/detections` | `PartDetectionArray` | detector_node 출력 |
| `/camera_right/camera_right/depth/image_rect_raw` | `sensor_msgs/msg/Image` | Wrist depth |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/target_pose` | `geometry_msgs/msg/PoseStamped` | 집어야 할 부품 1개 3D pose (base_link 기준) |

**3D 변환 수식** (Wrist Right 카메라 기준)
```
X = (u - 240.8) * Z / 246.2
Y = (v - 134.6) * Z / 246.2
Z = depth(m)
frame_id: camera_right_depth_optical_frame
```
depth↔color extrinsics ≈ 단위행렬 → 별도 extrinsic 보정 불필요

**전제 조건**
- TF tree: `camera_right_depth_optical_frame` → `camera_r_link` static transform 등록 필요
- 현황: 🔄 frame_id 발행 확인 완료 (0527), launch 등록 진행 중

---

### 4. `wrist_pointcloud_node` — A.3 PCD 발행

**패키지**: `perception_2d_to_pcd_wrist`

**Subscribe**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera_right/camera_right/depth/image_rect_raw` | `sensor_msgs/msg/Image` | Wrist depth |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera_right/points_base` | `sensor_msgs/msg/PointCloud2` | base_link 기준 PointCloud2 (GPD 입력용) |

> Manipulation 팀 `gpd_dual_view_node.py` 입력으로 사용. PCD 품질 확인 완료.

---

## 전체 토픽 흐름 요약

```
[Head ZED]──────────────────────────────────────────→ monitor_ocr_node
                                                            │
                                                    /monitor_ocr/result
                                                            │
                                                      mission_a.py (Step 2)

[Head ZED]  ──┐
[Wrist Left]  ├──→ detector_node ──→ /detections ──→ wrist_projection_node
[Wrist Right] ┘                  └──→ /detector_debug_image (UI overlay)
                                                            │
                                                      /target_pose
                                                            │
                                              grasp_filter.py (Manipulation)

[Wrist Right depth] ──→ wrist_pointcloud_node ──→ /camera_right/points_base
                                                            │
                                               gpd_dual_view_node (Manipulation)
                                                            │
                                                    /gpd/grasp_poses
```

---

## 실행 순서 (전체 파이프라인)

```bash
# 1. Head ZED 카메라
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

# 2. Wrist RealSense 카메라 (좌/우)
ros2 launch realsense2_camera rs_launch.py camera_name:=camera_left ...
ros2 launch realsense2_camera rs_launch.py camera_name:=camera_right ...

# 3. TF static transform (camera_right_depth_optical_frame → camera_r_link)
# TODO: launch 파일에 등록 필요

# 4. Perception 노드 실행
ros2 run monitor_ocr monitor_ocr_node --ros-args -p parts_mode:=true
ros2 run perception_part_detector detector_node
ros2 run perception_2d_to_pcd_wrist wrist_projection_node
ros2 run perception_2d_to_pcd_wrist wrist_pointcloud_node
```

---

## 블로커 및 TODO

| 항목 | 담당 | 기한 | 상태 |
|------|------|------|------|
| TF tree `camera_right_depth_optical_frame` → `camera_r_link` launch 등록 | Perception | 이번 주 | 🔄 |
| `/target_pose` base_link 기준 발행 로봇 환경 검증 | Perception | 5.31 | 🔄 |
| `monitor_ocr_node` YOLO 전환 및 실모니터 검증 | Perception | 이번 주 | 🔄 |
| wrist cam 기준 추가 학습 반영 후 `detector_node` 재검증 | Perception | 이번 주 | ⬜ |
| `/monitor_ocr/result` Action 구조 변환 | Perception | 협의 후 | ⬜ |
| command target 부품 별도 표시 기능 | Perception | 협의 후 | ⬜ |

---

## Claude Code 세션 시작 시 확인 명령어

```bash
# 패키지 위치 확인
find ~/AI_Worker_HC -name "monitor_ocr_node*" -o -name "detector_node*" 2>/dev/null

# 토픽 발행 확인 (로봇 연결 시)
ros2 topic echo /monitor_ocr/result
ros2 topic echo /detections
ros2 topic echo /target_pose
ros2 topic hz /camera_right/points_base
```
