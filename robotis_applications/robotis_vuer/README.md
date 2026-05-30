# ROBOTIS Vuer Overview

**ROBOTIS Vuer** for AI Worker lets you view a 3D scene on a **Meta Quest 3** headset and interact with the robot using hand tracking and related input. A browser-based VR client runs together with the **ROS 2** stack on the robot side.

## Stack summary

| Component | Description |
|-----------|-------------|
| **Headset** | Meta Quest 3 |
| **VR client** | [Vuer](https://github.com/vuer-ai/vuer)-based web app (WebXR). On the headset, open the page in the browser (or built-in browser) to start the VR session. |
| **Vuer version** | **v0.1.5** (version used and validated with AI Worker). Other versions may behave differently. [Official docs](https://docs.vuer.ai) |
| **Robot / PC** | ROS 2 nodes and applications connect to the Vuer server over **WebSocket**, exchanging pose, visualization, and control data. |

In short: **Quest 3 → (HTTPS/WSS) → Vuer** is the user-facing path, and **Vuer ↔ ROS 2** carries robot control and state.

## What is Vuer?

**Vuer** is a **Python** toolkit for **3D visualization and interaction** in the browser, aimed at robotics and VR. The server defines the scene (meshes, frames, markers, etc.) and events; the client renders with **WebXR** on the headset and sends hand and controller input back to the server.

- **Role**: Acts as the in-browser VR viewer and a **bidirectional bridge** to the robot PC. A typical pattern is ROS 2 nodes running alongside the Vuer server, linked via **WebSocket (`wss://`)**.
- **Why HTTPS/WSS**: WebXR and device APIs expect a **secure context**, so setups often use **HTTPS** and a **secure WebSocket** even on a local network.
- **Robotics**: Suited to robot models (e.g. URDF), live poses and sensor data, and teleoperation-style UIs. AI Worker aligns Quest 3 visuals and input with ROS 2 logic on this path.
- **Version in AI Worker**: Packages and Docker images target **Vuer v0.1.5**. Newer releases may change APIs or behavior; when debugging, compare against v0.1.5.

Official docs: [docs.vuer.ai](https://docs.vuer.ai) (may reflect newer releases than v0.1.5) · Source: [github.com/vuer-ai/vuer](https://github.com/vuer-ai/vuer)

## Teleoperation guide

빌드 · 실행 · 토픽 · 조작 매핑 · 파라미터 · 트러블슈팅 등 VR 텔레오퍼레이션 전반은
별도 문서로 정리되어 있습니다:

➡️ [VR 텔레오퍼레이션 가이드 (docs/VR_TELEOPERATION.md)](../docs/VR_TELEOPERATION.md)

launch 인자는 브랜치마다 다릅니다. `feature/mission-a`는 `model` 인자만, head-tracking/leader
기능 인자(`view_only_mode`, `enable_vr_image`, `enable_vr_head_tracking`,
`enable_leader_control`, `vr_head_tracking_*` 등)는 `feature/vr-head-tracking-leader-sg2`
브랜치에서 제공됩니다. 자세한 표는 위 가이드의 "런처 실행" 절을 참고하세요.

## HTTPS 인증서 자동 생성 (cert.pem / key.pem)

Quest WebXR은 secure context(HTTPS)에서만 동작하므로 Vuer 서버에 TLS 인증서가 필요합니다.
SG2 노드(`vr_publisher_sg2`)는 패키지 디렉터리의 `cert.pem` / `key.pem`을 사용하며,
**파일이 없으면 시작 시 자기서명 인증서를 자동으로 생성**합니다(`cryptography` 패키지 사용).

- 인증서 누락 시 과거에는 `Error in VR server thread: [Errno 2] No such file or directory`로
  서버가 기동하지 못했으나, 자동 생성으로 별도 사전 셋업 없이 어느 머신에서나 실행됩니다.
- 신뢰된 인증서를 쓰려면 직접 만든 `cert.pem`/`key.pem`을 패키지 디렉터리
  (`robotis_vuer/robotis_vuer/`)에 두면 자동 생성을 건너뜁니다.
- 자기서명 인증서이므로 Quest 브라우저의 보안 경고는 한 번 수동으로 통과해야 합니다.

> 종료 시 `'Vuer' object has no attribute 'stop'` 메시지가 보이던 문제도 함께 정리되었습니다
> (vuer 0.1.6에는 `stop()`이 없어 `hasattr` 가드 적용). 기능에는 영향이 없는 종료 경로 정리입니다.
