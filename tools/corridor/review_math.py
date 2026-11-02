"""Reproduce design-review arithmetic; examples are NOT vehicle calibration.

This tool is offline. It does not import device drivers or issue motion commands.
Run from the repository root:
    python tools/corridor/review_math.py --output build/corridor/review_math.json
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def stopping_clearance_mm(speed_m_s: float, latency_s: float,
                          deceleration_m_s2: float, reserve_mm: float,
                          uncertainty_mm: float) -> float:
    """Static obstacle, constant minimum deceleration, all distances at bumper."""
    if not all(math.isfinite(value) for value in (
        speed_m_s, latency_s, deceleration_m_s2, reserve_mm, uncertainty_mm
    )):
        raise ValueError("Inputs must be finite")
    if deceleration_m_s2 <= 0 or min(latency_s, reserve_mm, uncertainty_mm) < 0:
        raise ValueError("Positive deceleration and nonnegative budgets required")
    speed = abs(speed_m_s)
    return reserve_mm + uncertainty_mm + 1000 * (
        speed * latency_s + speed * speed / (2 * deceleration_m_s2)
    )


def build_report() -> dict:
    examples = [
        {
            "speed_m_s": speed,
            "latency_s": 0.08,
            "minimum_deceleration_m_s2": 0.5,
            "reserve_mm": 50,
            "uncertainty_mm": 10,
            "required_clearance_mm": round(
                stopping_clearance_mm(speed, 0.08, 0.5, 50, 10), 6
            ),
        }
        for speed in (0.1, 0.2, 0.3, 0.4)
    ]
    return {
        "status": "design_review_arithmetic_only",
        "measured_vehicle_parameters": False,
        "assumptions": [
            "Stopping examples concern a static obstacle and constant deceleration.",
            "80ms is an illustrative latency budget, not a measured upper bound.",
            "The body dimensions below are examples, not dimensions of this car.",
            "Rotation in a straight strip does not prove L-corner reachability.",
        ],
        "centering": {
            "parallel_symmetric_body_only": True,
            "max_gap_difference_mm": 15,
            "corresponding_max_center_offset_mm": 7.5,
        },
        "camera": {
            "configured_fps_from_source": 30,
            "frame_period_ms": 1000 / 30,
            "claim_supported": "Frame sampling does not guarantee 5ms scene-to-action.",
        },
        "wheel_control_upgrade_examples": {
            "PI_pipeline_clocks": 7,
            "PI_pipeline_us_at_50MHz": 7 / 50_000_000 * 1e6,
            "legacy_period_ms": 1000 / 100,
            "proposed_period_ms": 1000 / 1000,
            "worst_scheduling_wait_reduction_ms": 1000 / 100 - 1000 / 1000,
            "current_pwm_slew_permille_per_tick": 20,
            "slew_updates_to_350permille": math.ceil(350 / 20),
            "approx_slew_time_to_350permille_ms": math.ceil(350 / 20) * 10,
            "equivalent_permille_per_1ms_for_same_slew_rate": 20 / 10,
            "assumed_uncalibrated_position_units_per_rev": 1500,
            "assumed_wheel_diameter_mm": 65,
            "right_A_edge_position_units_per_event": 2,
            "right_event_spacing_ms_at_50mm_s": math.pi * 65 / 1500 * 2 / 50 * 1000,
            "right_1ms_direct_count_speed_quantum_mm_s": math.pi * 65 / 1500 * 2 * 1000,
            "left_and_right_decoder_modes_differ": True,
            "hardware_control_frequency_changed": False,
        },
        "lidar_budget": {
            "nominal_points_per_second": 21600,
            "control_clock_hz": 50_000_000,
            "average_clocks_per_point": 50_000_000 / 21600,
            "sixteen_iteration_arithmetic_us": 16 / 50_000_000 * 1e6,
            "parser_burst_points": 12,
            "requires_fifo_and_worst_case_service_budget": True,
        },
        "stopping_examples": examples,
        "rotation_example": {
            "width_mm": 220,
            "length_mm": 300,
            "centered_rotation_swept_diameter_mm": math.hypot(220, 300),
            "straight_strip_width_mm": 320,
            "full_centered_rotation_fits_strip": False,
        },
        "docking_example": {
            "lidar_behind_bumper_mm": 120,
            "desired_bumper_to_target_mm": 100,
            "corresponding_front_ray_range_mm": 220,
            "only_valid_for_aligned_front_target": True,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    payload = json.dumps(build_report(), ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")


if __name__ == "__main__":
    main()
