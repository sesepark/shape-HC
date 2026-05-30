#!/usr/bin/env python3
"""Mission A State Machine — stub.

Skeleton ROS 2 node implementing the Mission A state machine.
Each state currently performs minimal transition logic plus TODO markers;
no real perception/manipulation behavior yet.

Reference: robotis_applications/docs/MISSION_A_SCENARIO_PLAN.md
"""
from __future__ import annotations

import json
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# perception_part_detector message package may not yet be on PYTHONPATH
# during the stub phase — guard the import so the node still starts.
try:
    from perception_part_detector.msg import PartDetectionArray
except ImportError:
    PartDetectionArray = None  # type: ignore[assignment]


# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #
GRASP_ASSESSMENT_ENABLED = False  # flip to True after Hand-Eye Calibration

# Timeout (seconds)
TIMEOUT_INIT       = 60
TIMEOUT_A1_MONITOR = 90
TIMEOUT_A1_OCR     = 20
TIMEOUT_A2_SCAN    = 90
TIMEOUT_PICK_PLACE = 45
TIMEOUT_VERIFY     = 20
FALLBACK_OK_DELAY  = 10   # OCR 실패 시 강제 OK 딜레이

MAX_RECOVERY_RETRY = 3


# --------------------------------------------------------------------------- #
# State enum
# --------------------------------------------------------------------------- #
class State(Enum):
    INIT        = auto()
    A1_MONITOR  = auto()
    A2_SCAN     = auto()
    A3_PICK     = auto()
    A3_PLACE    = auto()
    VERIFY      = auto()
    DONE        = auto()
    RECOVERY    = auto()
    MANUAL_WAIT = auto()


# --------------------------------------------------------------------------- #
# Node
# --------------------------------------------------------------------------- #
class MissionA(Node):
    def __init__(self) -> None:
        super().__init__('mission_a')

        # --- Subscribers ---
        self.sub_manipulator_state = self.create_subscription(
            String, '/manipulator_state', self._on_manipulator_state, 10)
        self.sub_monitor_ocr = self.create_subscription(
            String, '/monitor_ocr/result', self._on_monitor_ocr, 10)
        if PartDetectionArray is not None:
            self.sub_detections = self.create_subscription(
                PartDetectionArray, '/detections', self._on_detections, 10)
        else:
            self.get_logger().warning(
                'perception_part_detector.msg not importable — '
                '/detections subscription disabled. Build the message package '
                'and re-source the workspace.')
            self.sub_detections = None
        self.sub_target_pose = self.create_subscription(
            PoseStamped, '/target_pose', self._on_target_pose, 10)
        self.sub_attached_object = self.create_subscription(
            String, '/attached_object', self._on_attached_object, 10)

        # --- Publishers ---
        self.pub_active_mission = self.create_publisher(
            String, '/active_mission', 10)
        self.pub_attach_cmd = self.create_publisher(
            String, '/attach_cmd', 10)
        self.pub_detach_cmd = self.create_publisher(
            String, '/detach_cmd', 10)

        # --- State storage ---
        self.state: State = State.INIT
        self.recovery_count: int = 0

        # Latest topic snapshots (None until first message)
        self.last_manipulator_state: str | None = None
        self.last_ocr_result: dict | None = None
        self.last_detections = None
        self.last_target_pose: PoseStamped | None = None
        self.last_attached_object: str | None = None

        # task_list built from OCR result (Step 2)
        self.task_list: list[dict] = []

        # State tick at 10 Hz
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'mission_a started in state={self.state.name} '
            f'(GRASP_ASSESSMENT_ENABLED={GRASP_ASSESSMENT_ENABLED})')

    # --- Subscription callbacks: store only, decisions happen in _tick ---
    def _on_manipulator_state(self, msg: String) -> None:
        self.last_manipulator_state = msg.data
        self.get_logger().debug(f'[sub] /manipulator_state = {msg.data}')

    def _on_monitor_ocr(self, msg: String) -> None:
        try:
            self.last_ocr_result = json.loads(msg.data)
            self.get_logger().info(
                '[sub] /monitor_ocr/result received '
                f'(screen_detected={self.last_ocr_result.get("latest_screen_detected")})')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'/monitor_ocr/result JSON decode failed: {e}')

    def _on_detections(self, msg) -> None:
        self.last_detections = msg
        self.get_logger().debug('[sub] /detections received')

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self.last_target_pose = msg
        self.get_logger().debug('[sub] /target_pose received')

    def _on_attached_object(self, msg: String) -> None:
        self.last_attached_object = msg.data
        self.get_logger().debug(f'[sub] /attached_object = "{msg.data}"')

    # --- State dispatch ---
    def _tick(self) -> None:
        handler = {
            State.INIT:        self._run_init,
            State.A1_MONITOR:  self._run_a1_monitor,
            State.A2_SCAN:     self._run_a2_scan,
            State.A3_PICK:     self._run_a3_pick,
            State.A3_PLACE:    self._run_a3_place,
            State.VERIFY:      self._run_verify,
            State.DONE:        self._run_done,
            State.RECOVERY:    self._run_recovery,
            State.MANUAL_WAIT: self._run_manual_wait,
        }[self.state]
        handler()

    def _transition(self, new_state: State) -> None:
        if new_state == self.state:
            return
        self.get_logger().info(f'[state] {self.state.name} -> {new_state.name}')
        self.state = new_state

    # --- Per-state stub handlers ---
    def _run_init(self) -> None:
        # TODO: verify head/wrist camera timestamps; honor TIMEOUT_INIT.
        msg = String(data='A')
        self.pub_active_mission.publish(msg)
        self.get_logger().info('[INIT] published /active_mission="A" (stub)')
        if self.last_manipulator_state == 'IDLE':
            self._transition(State.A1_MONITOR)

    def _run_a1_monitor(self) -> None:
        # TODO: render OK sign on monitor; FALLBACK_OK_DELAY on failure;
        # honor TIMEOUT_A1_MONITOR / TIMEOUT_A1_OCR.
        if self.last_ocr_result and self.last_ocr_result.get('latest_screen_detected'):
            self.task_list = self.last_ocr_result.get('parts', [])
            self.get_logger().info(
                f'[A1_MONITOR] task_list built: {self.task_list} (stub)')
            self._transition(State.A2_SCAN)

    def _run_a2_scan(self) -> None:
        # TODO: pick target class from task_list, ROI-filter /detections,
        # forward chosen candidate to grasp_filter; honor TIMEOUT_A2_SCAN.
        if self.last_target_pose is not None:
            self.get_logger().info(
                '[A2_SCAN] /target_pose received -> A3_PICK (stub)')
            self._transition(State.A3_PICK)

    def _run_a3_pick(self) -> None:
        # TODO: dispatch bin_pick action; monitor GRASPING -> ATTACHED;
        # timeout -> RECOVERY (TIMEOUT_PICK_PLACE).
        if self.last_attached_object:
            self.get_logger().info(
                f'[A3_PICK] attached_object="{self.last_attached_object}" (stub)')
            self._transition(State.A3_PLACE)

    def _run_a3_place(self) -> None:
        # TODO: wait for /tray_region, compute place pose, dispatch tray_place,
        # monitor RELEASING -> IDLE; timeout -> RECOVERY (TIMEOUT_PICK_PLACE).
        if self.last_attached_object == '':
            self.get_logger().info('[A3_PLACE] released -> VERIFY (stub)')
            self._transition(State.VERIFY)

    def _run_verify(self) -> None:
        # TODO: re-scan tray ROI, decrement remaining count, branch to
        # A2_SCAN (remaining>0) or DONE (remaining==0); TIMEOUT_VERIFY.
        self.get_logger().info('[VERIFY] stub -> DONE')
        self._transition(State.DONE)

    def _run_done(self) -> None:
        self.get_logger().info('[DONE] mission A complete (stub)')
        self.timer.cancel()

    def _run_recovery(self) -> None:
        # TODO: retry pick/place up to MAX_RECOVERY_RETRY, else MANUAL_WAIT.
        if self.recovery_count < MAX_RECOVERY_RETRY:
            self.recovery_count += 1
            self.get_logger().warning(
                f'[RECOVERY] retry {self.recovery_count}/{MAX_RECOVERY_RETRY} (stub)')
            self._transition(State.A3_PICK)
        else:
            self._transition(State.MANUAL_WAIT)

    def _run_manual_wait(self) -> None:
        # TODO: wait for operator resume signal then -> A3_PICK.
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
