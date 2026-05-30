# 미션 A 시나리오 코드 계획
> **최종 업데이트**: 2026-05-30  
> **목표**: 시험 기간 전 미션 A end-to-end 자율 동작 완료  
> **배점**: 원격 40점 / 자율 60점  
> **상태 범례**: ✅ 완료 | 🔄 진행 중 | ⬜ 개발 예정 | ⚠️ 블로커

---

## 레포 구조

> **2026-05-30 업데이트** — Step 1에서 팀별 코드 통합 완료 (방식 A: 팀 레포 클론 후 복사).

```
~/AI_Worker_HC/                                # snu-shape/AI_Worker_HC.git (main)
├── ai_worker/                                 # ROBOTIS 공식 — 수정 금지
├── physical_ai_tools/                         # ROBOTIS 공식 — 수정 금지
├── robotis_applications/                      # 팀별 개발 코드 (여기서만 작업)
│   ├── robotis_applications/                  # 스캐폴드 메타 패키지 (ROBOTIS)
│   ├── robotis_vuer/                          # ROBOTIS Vuer 도구
│   ├── docker/                                # ROBOTIS 도커 컨피그
│   ├── perception/                            # Perception 팀 (hublemon/Humanoid-Challenge-Perception 클론)
│   │   ├── monitor_ocr/                       # A.1 지령 모니터 OCR
│   │   ├── perception_part_detector/          # A.2 부품 검출·분류 (YOLO11s-seg)
│   │   ├── perception_2d_to_pcd_wrist/        # A.3 wrist 2D→3D 변환
│   │   └── perception_2d_to_pcd/              # head 카메라 2D→3D (참고용)
│   ├── manipulation/                          # Manipulation 팀 (chlgkals07/ai-worker-ws 클론)
│   │   └── ai_worker_manipulation/            # MoveIt client, GPD, grasp_filter
│   ├── mission/                               # System 팀 미션 코드
│   │   ├── mission_a.py                       # Mission A State Machine stub (Step 1 생성)
│   │   └── README.md
│   └── docs/                                  # 설계 문서
│       ├── MISSION_A_SCENARIO_PLAN.md         # (본 문서)
│       └── PERCEPTION_INTERFACE.md
└── README.md
```

### 팀별 소스 레포 (개발자 워크스페이스)

| 팀 | 원본 레포 | 로컬 클론 위치 | 비고 |
|----|----------|---------------|------|
| Perception | `hublemon/Humanoid-Challenge-Perception` | `~/perception-ws/` | `~/robotis_ros2_ws/`는 root 소유라 우회 |
| Manipulation | `chlgkals07/ai-worker-ws` | `~/ai-worker-ws/` | 기대 경로와 동일 |

> **동기화 정책**: 팀별 코드 업데이트는 각자 working 워크스페이스에서 작업 후 PR. AI_Worker_HC `robotis_applications/{perception,manipulation}/` 디렉토리는 검증된 시점 스냅샷으로 사용. 동기화 자동화는 후속 작업 (REPO_INTEGRATION_GUIDE.md TBD).

---

## State Machine 전체 구조

```
[*] → INIT
INIT → A1_MONITOR       : /active_mission="A" 발행
A1_MONITOR → A2_SCAN    : /monitor_ocr/result 수신 AND screen_detected=true
A1_MONITOR → A1_MONITOR : 실패 (재시도) / 10초 후 fallback OK
A2_SCAN → A3_PICK       : /target_pose 수신
A3_PICK → A3_PLACE      : /attached_object non-empty
A3_PICK → RECOVERY      : timeout (파지 실패)
A3_PLACE → VERIFY       : /attached_object=""
A3_PLACE → RECOVERY     : timeout (place 실패)
VERIFY → A2_SCAN        : 잔여 수량 > 0
VERIFY → DONE           : 잔여 수량 = 0
RECOVERY → A3_PICK      : 재시도 (max 3회)
RECOVERY → MANUAL_WAIT  : 재시도 초과
MANUAL_WAIT → A3_PICK   : 운용자 재개 신호
DONE → [*]
```

---

## Step별 로직 상세

### Step 1 — INIT (환경 초기화)

**목적**: MoveIt planning scene 등록, CM 노드 확인, 미션 시작 선언

**로직**
```python
1. /active_mission = "A" 발행  →  CM이 Zone A planning scene 자동 등록
2. /manipulator_state = IDLE 수신 대기
3. head/wrist camera timestamp 유효 확인
4. 조건 충족 시 → A1_MONITOR 진입
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/manipulator_state` | IDLE 확인 후 다음 Step 진입 |

**발행 토픽**
| 토픽 | 내용 |
|------|------|
| `/active_mission` | `"A"` 발행 |

**Timeout**: 초기화 전체 목표 30초 / hard 1분

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `competition_manager_node.py` | Manipulation | ✅ 완료 |
| `environment.py` → `setup_zone_a()` | Manipulation | ✅ 완료 |
| `mission_a.py` (INIT 상태) | System | 🔄 stub 생성 (2026-05-30) |

**요청사항**
- Manipulation: `competition_manager` → 시스템 팀 관리 노드로 이전
- Manipulation: `/active_zone` → `/active_mission` 명칭 변경
- Manipulation: `/cm_state` → `/manipulator_state` 명칭 변경

---

### Step 2 — A1_MONITOR (지령 모니터 인식)

**목적**: 지령 모니터 OCR 파싱 → task_list 생성 → OK 사인 출력

**로직**
```python
1. monitor_ocr_node 실행 확인
2. /monitor_ocr/result 구독 → screen_detected=true + parts 파싱 성공 시
   → task_list 내부 저장
   → 모니터에 OK 사인 출력
   → A2_SCAN 진입
3. 파싱 실패 시: 재시도
   → 10초 경과 후 fallback: 강제 OK 출력 후 진입 (점수 10점 확보)
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/monitor_ocr/result` | JSON: parts 배열 + screen_detected |

**발행 토픽**
| 토픽 | 내용 |
|------|------|
| OK 사인 | 모니터 출력 (구현 방식 TBD) |

**Timeout**: 목표 1분 / hard 1분 30초  
- 모니터 탐색·crop: 30초  
- OCR 파싱: 20초  
- OK sign 출력: 5~10초

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `monitor_ocr_node.py` | Perception | 🔄 YOLO 전환 중 |
| `mission_a.py` (A1_MONITOR 상태) | System | 🔄 stub 생성 (2026-05-30) |

**요청사항**
- Perception: `/monitor_ocr/result` → Action 구조 변환 검토
- Perception: 코드 AI_Worker_HC 레포 브랜치에 업로드

---

### Step 3 — A2_SCAN (부품 상자 스캔)

**목적**: 노란 상자 ROI 검출 → 부품 탐지·분류 → target 선정

**로직**
```python
1. detector_node 구동 확인
2. /detections 구독 → 노란 상자 ROI 내 부품 검출
3. task_list 기준 현재 target 부품 class 확인
4. target class 부품 중 confidence 높은 후보 1개 선정
5. /target_pose 수신 (wrist_projection_node 경유)
6. /detector_debug_image overlay 모니터 표시
7. → A3_PICK 진입
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/detections` | PartDetectionArray (5종 부품 bbox/class/confidence) |
| `/target_pose` | 선정된 부품 3D pose (base_link 기준) |

**발행 토픽**
| 토픽 | 내용 |
|------|------|
| (내부 target 선정 결과) | grasp_filter로 전달 |

**Timeout**: 목표 1분 / hard 1분 30초  
- 노란 box ROI 인식: 20초  
- 부품 검출·분류: 40초

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `detector_node.py` | Perception | 🔄 로봇 환경 검증 필요 |
| `wrist_projection_node.py` | Perception | ✅ 완료 |
| `wrist_pointcloud_node.py` | Perception | ✅ 완료 |
| `point_cloud_transformer_node.py` | Manipulation | ✅ 완료 |
| `mission_a.py` (A2_SCAN 상태) | System | 🔄 stub 생성 (2026-05-30) |

**요청사항**
- Perception: command target 부품 `/detections`에서 별도 표시 필드 추가
- Perception: `/camera_left/points_base`, `/camera_right/points_base` 토픽명 확정
- Manipulation: 전달받은 grasp 후보 개수에 따라 grasp 물체 결정 로직

---

### Step 4 — (A3_PICK 전처리) Grasp Pose 계산

**목적**: PCD → GPD → grasp candidate 필터링 → best pose 1개 선택

**로직**
```python
1. /camera_right/points_base (PCD) → gpd_dual_view_node → /gpd/grasp_poses
2. /target_pose + /gpd/grasp_poses → grasp_filter.py
   → IK 가능 여부 확인
   → MoveIt 충돌 여부 확인
   → 작업 반경 내 여부 확인
3. best_grasp_pose 1개 선택 → bin_pick.py로 전달
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/camera_left/points_base` | base_link 기준 left PCD |
| `/camera_right/points_base` | base_link 기준 right PCD |
| `/gpd/grasp_poses` | PoseArray (GPD 출력) |
| `/target_pose` | IK seed로 활용 |

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `gpd_dual_view_node.py` | Manipulation | ✅ 완료 |
| `grasp_filter.py` | Manipulation | 🔄 개발 중 (~5.30) |
| `tf_transformer.py` | Manipulation | ✅ 완료 |

**Timeout**: Pick & Place 1회당 45초

> ⚠️ **Hand-Eye Calibration 미완료 시**: `/target_pose` 좌표 오차로 pick 실패  
> → 우회: `/attach_cmd`로 수동 attach 후 이후 Step 테스트 진행  
> → `GRASP_ASSESSMENT_ENABLED = False` (현재)

---

### Step 5 — A3_PICK (Pick 실행)

**목적**: grasp_filter best pose → 실제 파지 수행 → 파지 성공 확인

**로직**
```python
1. moveit_client.move_to_pose(pre_grasp)   # OMPL 충돌 회피
2. moveit_client.cartesian_move(grasp_pose) # Pilz LIN 직선 하강
3. gripper.close()  →  CM: GRASPING → ATTACHED 자동 전환
4. /attached_object non-empty 대기 (파지 확인)
5. 성공 → A3_PLACE 진입
6. 실패 (timeout) → RECOVERY
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/attached_object` | non-empty → 파지 성공 |
| `/manipulator_state` | GRASPING → ATTACHED 전환 모니터링 |

**발행 토픽**
| 토픽 | 내용 |
|------|------|
| `/attach_cmd` | 수동 attach (캘리브레이션 전 우회용) |

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `bin_pick.py` | Manipulation | ⬜ 개발 예정 |
| `moveit_client.py` | Manipulation | ✅ 완료 |
| `gripper_controller.py` | Manipulation | ✅ 완료 |
| `competition_manager_node.py` | System | ✅ 완료 |

**요청사항**
- Manipulation: `bin_pick.py` 내부 함수 → **Action 서버**로 감싸기 (success/fail/timeout 반환)

---

### Step 6 — A3_PLACE (트레이 검출 및 Place 실행)

**목적**: 파란 트레이 검출 → place pose 계산 → 부품 내려놓기

**로직**
```python
1. /tray_region 수신 (Perception 트레이 검출 노드)
2. 빈 슬롯 place pose 계산
3. moveit_client.move_to_pose(pre_place)    # OMPL
4. moveit_client.cartesian_move(place_pose) # Pilz LIN
5. gripper.open()  →  CM: RELEASING → IDLE 자동 전환
6. /attached_object = "" 대기 (내려놓기 확인)
7. 성공 → VERIFY 진입
8. 실패 (timeout) → RECOVERY
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/tray_region` | 파란 트레이 3D 위치 + 적재 가능 영역 (base_link 기준) |
| `/attached_object` | "" → place 완료 확인 |
| `/manipulator_state` | RELEASING → IDLE 전환 확인 |

**발행 토픽**
| 토픽 | 내용 |
|------|------|
| `/detach_cmd` | 수동 detach (필요 시) |

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| 트레이 검출 노드 | Perception | 🔄 진행 중 |
| `tray_place.py` | Manipulation | ⬜ 개발 예정 |
| `moveit_client.py` | Manipulation | ✅ 완료 |

**요청사항**
- Perception: wrist camera로 적재 가능 내부 영역 보정 로직 포함 개발
- Perception: `/tray_region` 메시지 타입 Manipulation과 합의 필요

---

### Step 7 — VERIFY (적재 검증 및 반복/완료 판단)

**목적**: 적재 성공 확인 → task_list 수량 차감 → 반복 or 완료

**로직**
```python
1. release + retreat 완료 후 tray ROI 재관측
2. 적재 object class == current target type 확인
3. 성공 시: remaining_quantity -= 1
   - remaining_quantity > 0 → A2_SCAN 복귀 (Step 3)
   - remaining_quantity = 0 → DONE
4. 실패 시 (wrong/no/multi-placement): RECOVERY → Step 3 재시도
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/detections_3d` | tray 재스캔 결과 (Phase 3, 추후) |

**분기 로직**
```
잔여 수량 > 0  →  A2_SCAN (Step 3) 복귀
잔여 수량 = 0  →  DONE
파지/적재 실패 →  RECOVERY → Step 3 재시도
```

**Timeout**
- 적재 성공 판정: 20초
- task state update: 즉시 (1~2초)
- 최종 완료 판정: 10~20초

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `mission_a.py` (VERIFY 상태) | System | 🔄 stub 생성 (2026-05-30) |
| 적재 검증 모듈 | Perception + Manipulation | ⬜ Phase 3 (6.1~) |

---

## Blackboard 키 (전 팀 공통 합의 필요)

| 키 | 방향 | 타입 | 설명 | 합의 상태 |
|----|------|------|------|-----------|
| `/active_mission` | mission_a → CM | `std_msgs/String` | 현재 활성 미션 ("A"~"D") | ⬜ 미합의 |
| `/manipulator_state` | CM → mission_a | `std_msgs/String` | IDLE / GRASPING / ATTACHED / RELEASING | ⬜ 미합의 |
| `/monitor_ocr/result` | Perception → mission_a | `std_msgs/String` (JSON) | OCR 결과 | ✅ 확정 |
| `/detections` | Perception → mission_a | `PartDetectionArray` | 부품 검출 배열 | ✅ 확정 |
| `/target_pose` | Perception → grasp_filter | `PoseStamped` | 집어야 할 부품 3D pose | 🔄 검증 중 |
| `/tray_region` | Perception → tray_place | TBD | 파란 트레이 위치 + 적재 영역 | ⬜ 미합의 |
| `/attached_object` | CM → mission_a | `std_msgs/String` | 파지 물체명 (""=없음) | ✅ 확정 |
| `/camera_right/points_base` | Perception → GPD | `PointCloud2` | base_link 기준 PCD | ✅ 확정 |
| `/gpd/grasp_poses` | GPD → grasp_filter | `PoseArray` | grasp pose 후보 | ✅ 확정 |
| `/attach_cmd` | mission_a → CM | `std_msgs/String` | 수동 attach | ✅ 확정 |
| `/detach_cmd` | mission_a → CM | `std_msgs/String` | 수동 detach | ✅ 확정 |

---

## 단기 Action Items

| # | 할 일 | 담당 | 기한 | 상태 |
|---|-------|------|------|------|
| 1 | Hand-Eye Calibration 착수 | Manipulation (조서영) | 이번 주 | ⬜ |
| 2 | TF tree `camera_right_depth_optical_frame`→`camera_r_link` launch 등록 | Perception | 이번 주 | 🔄 |
| 3 | Blackboard 키 이름 전 팀 합의 문서화 | System | 이번 주 | ⬜ |
| 4 | `grasp_filter.py` 완성 | Manipulation | 5.30 | 🔄 |
| 5 | `/target_pose` base_link 기준 발행 로봇 검증 | Perception | 5.31 | 🔄 |
| 6 | `mission_a.py` State Machine 뼈대 생성 | System | 5.31 | ✅ (2026-05-30) |
| 7 | pick-place dummy 루프 1회 end-to-end 테스트 | Manipulation + System | 6.1 | ⬜ |
| 8 | `/tray_region` 발행 구현 및 인터페이스 합의 | Perception | 6.1 | ⬜ |

---

## 전제 조건 / 블로커

| 블로커 | 영향 범위 | 우회 방법 | 상태 |
|--------|----------|-----------|------|
| Hand-Eye Calibration (목표 오차 5mm↓) | Step 4~6 전체 | `/attach_cmd` 수동 attach | ⬜ 미착수 |
| TF tree static transform 등록 | Step 3 3D 변환 | left wrist / ZED head만 임시 운용 | 🔄 |
| Blackboard 키 이름 전 팀 합의 | `mission_a.py` 작성 시작 불가 | — | ⬜ 미착수 |
| `bin_pick.py` 개발 완료 | Step 5 | 수동 attach로 Step 6~7만 테스트 | ⬜ |
