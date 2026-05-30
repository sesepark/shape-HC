# mission/

System 팀의 휴머노이드 챌린지 미션 시나리오 코드.

## 파일

- `mission_a.py` — Mission A State Machine **stub** (rclpy Node, 전이 로직만 구현, 실제 perception/manipulation 호출은 TODO)

## 실행 (stub 검증)

현재는 ament_python 패키지 설정이 없으므로 직접 실행:

```bash
# Terminal 1
python3 ~/AI_Worker_HC/robotis_applications/mission/mission_a.py

# Terminal 2 — 더미 토픽으로 상태 전이 확인
ros2 topic pub --once /manipulator_state std_msgs/String "data: 'IDLE'"
ros2 topic pub --once /monitor_ocr/result std_msgs/String \
  'data: "{\"frames_used\":10,\"parts\":[{\"name\":\"플랜지 너트\",\"count\":1}],\"latest_screen_detected\":true}"'
ros2 topic pub --once /target_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: base_link}, pose: {position: {x: 0.5, y: 0.0, z: 0.3}}}'
ros2 topic pub --once /attached_object std_msgs/String "data: 'flange_nut'"
ros2 topic pub --once /attached_object std_msgs/String "data: ''"
```

## TODO

- [ ] `package.xml` + `setup.py` 추가하여 `ros2 run mission mission_a` 실행 가능하게
- [ ] `perception_part_detector` 메시지 빌드 후 `/detections` 구독 활성화 검증
- [ ] 각 상태 핸들러에 실제 perception/manipulation 호출 추가
- [ ] Timeout 처리 (`TIMEOUT_*` 상수 활용)
- [ ] OCR 실패 시 `FALLBACK_OK_DELAY` 폴백 구현
- [ ] `RECOVERY` 상태에서 ManipulationAction client 연동
