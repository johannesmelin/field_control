"""JSON-safe runtime diagnostics, kept separate from control calculations."""
from __future__ import annotations

from typing import Any

from .runtime import FieldControlRuntime


def status_payload(runtime: FieldControlRuntime) -> dict[str, Any]:
    status = runtime.status()
    observation = status.observation
    vision = None if observation is None or observation.vision is None else observation.vision
    heading = None if observation is None else observation.heading_deg
    reference = None if observation is None else observation.row_heading_reference_deg
    odometry_distance = (None if observation is None else
                         (observation.odometry_sample.forward_distance_m
                          if observation.odometry_sample is not None else observation.distance_m))
    target_x = None if vision is None else vision.target_x
    x_goal_px = runtime.config.vision.x_goal * runtime.config.processing_width
    heading_error = None if heading is None or reference is None else (reference - heading + 180.0) % 360.0 - 180.0
    standby_active, standby_deadline_s = runtime.web_standby_status()
    return {
        "running": status.running,
        "mode": status.mode,
        "state": status.state,
        "state_reason": status.snapshot.reason,
        "fault": status.fault or status.snapshot.fault,
        "row_number": status.snapshot.row_number,
        "pass_number": status.snapshot.pass_number,
        "auto_start_remaining_s": status.snapshot.auto_start_remaining_s,
        "camera": {"ok": False if observation is None else observation.camera_fresh,
                    "age_s": None if observation is None else observation.camera_age_s,
                    "error": None if observation is None else observation.camera_error},
        "imu": {"ok": False if observation is None else observation.imu_fresh,
                "age_s": None if observation is None else observation.imu_age_s,
                "error": None if observation is None else observation.imu_error},
        "odometry": {"ok": False if observation is None else observation.odometry_fresh,
                      "age_s": None if observation is None else observation.odometry_age_s,
                      "distance_m": odometry_distance,
                      "left_distance_m": None if observation is None or observation.odometry_sample is None else observation.odometry_sample.left_distance_m,
                      "right_distance_m": None if observation is None or observation.odometry_sample is None else observation.odometry_sample.right_distance_m,
                      "yaw_change_deg": None if observation is None or observation.odometry_sample is None else observation.odometry_sample.yaw_change_deg},
        "heading": {"filtered_heading_deg": heading, "row_heading_reference_deg": reference,
                    "reference_reliable": False if observation is None else observation.row_heading_reliable,
                    "reference_build_distance_m": runtime.heading.reference.reliable_distance_m,
                    "heading_error_deg": heading_error},
        "vision": {"target_x_px": target_x, "x_goal_px": x_goal_px,
                   "target_valid": target_x is not None,
                   "bud_in_trigger_zone": False if vision is None else vision.bud_in_trigger_zone,
                   "bud_in_pick_zone": False if vision is None else vision.bud_in_pick_zone,
                   "marker_found": False if vision is None else vision.marker_found},
        "search_distance_m": status.snapshot.search_distance_m,
        "post_pick_distance_m": status.snapshot.post_pick_distance_m,
        "motor_output_armed": status.motor_output_armed,
        "motor_fault_reason": getattr(runtime.motor, "fault_reason", None),
        "can": {"ok": getattr(runtime.motor, "fault_reason", None) is None},
        "control_lease": {"active": runtime.lease.valid(None), "watchdog_timeout_s": runtime.config.control_lease_timeout_s},
        "physical_web_standby": {"active": standby_active, "deadline_monotonic_s": standby_deadline_s},
        "marker_armed": status.snapshot.marker_armed,
        "last_command": None if status.last_command is None else {
            "left_rpm": status.last_command.left_rpm, "right_rpm": status.last_command.right_rpm,
            "source": status.last_command.source,
        },
        # Historical evidence only; it is never a live command and cannot
        # renew a lease or re-arm motor output after STOP/fault/shutdown.
        "last_admitted_nonzero_command": (
            None if status.last_admitted_nonzero_command is None else {
                "left_rpm": status.last_admitted_nonzero_command.left_rpm,
                "right_rpm": status.last_admitted_nonzero_command.right_rpm,
                "source": status.last_admitted_nonzero_command.source,
            }
        ),
        "recent_events": runtime.events.recent(),
    }
