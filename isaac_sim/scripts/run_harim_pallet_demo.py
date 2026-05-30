import argparse
import math
import os
import shutil
import time
import traceback
from enum import Enum, auto
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_PICKUP_X = 0.82
DEFAULT_PICKUP_Y = -0.31
DEFAULT_DROP_X = DEFAULT_PICKUP_X + 10.6
DEFAULT_DROP_Y = DEFAULT_PICKUP_Y
DEFAULT_GIF_OUTPUT_DIR = PROJECT_ROOT / "isaacsim_outputs"
LATEST_REVIEW_GIF_NAME = "latest_review.gif"
GIF_CANVAS_SIZE = (960, 540)
GIF_FRAME_STRIDE = 80
GIF_MAX_FRAMES = 180
GIF_FRAME_DURATION_MS = 80
WORLD_FLOOR_Z = -1.1818
DEFAULT_AMR_Z = WORLD_FLOOR_Z
DEFAULT_LIFT_HEIGHT = 0.11
DEFAULT_MOVE_SPEED = 0.65
PICK_STATION_BIN_POSITION = np.array([0.0, 0.42, -0.15], dtype=float)
CONVEYOR_PICK_WINDOW_Y = 0.68
PICK_READY_EE_POSITION = np.array([0.16, 0.22, -0.02], dtype=float)
POST_RELEASE_CLEARANCE_LIFT = 0.42
POST_RELEASE_RETREAT_OFFSET = np.array([-0.30, 0.0, 0.62], dtype=float)
RELEASE_RETREAT_DURATION = 0.90
SURFACE_GRIPPER_RELEASE_RETRIES = 3
SCRIPTED_PLACE_DURATION = 0.70
SCRIPTED_PLACE_EE_HOVER = 0.30
ACTIVE_BIN_ATTACHED_MAX_FRAME_STEP = 0.05
POST_RELEASE_JOINT_SETTLE_DURATION = 0.65
ARM_CLEAR_SETTLE_TIME = 1.8
REACH_PICK_MAX_DURATION = 12.0
REACH_PLACE_MAX_DURATION = 4.2
RETURN_READY_DURATION = 10.0
RETURN_READY_POSITION_THRESHOLD = 0.04
AMR_START_STANDOFF = 3.2
AMR_APPROACH_STANDOFF = 1.05
AMR_DROP_APPROACH_STANDOFF = 1.05
AMR_DOCK_MOVE_SPEED_SCALE = 0.45
AMR_LIFT_DURATION = 1.25
AMR_LIFT_SETTLE_TIME = 0.25
SLIDE_EXIT_DISTANCE = 1.8
PALLET_TUNNEL_HALF_WIDTH = 0.46
DROP_WORKSTATION_Z = -0.73
PALLET_CENTER_Z = -0.62
PALLET_DECK_SCALE = np.array([1.20, 1.08, 0.06], dtype=float)
PALLET_RUNNER_SCALE = np.array([1.20, 0.14, 0.12], dtype=float)
PALLET_BLOCK_SCALE = np.array([0.18, 0.16, 0.12], dtype=float)
PALLET_GROOVE_SCALE = np.array([1.22, 0.028, 0.012], dtype=float)
PALLET_TOP_SUPPORT_OFFSET = np.array([0.0, 0.0, 0.02], dtype=float)
PALLET_TOP_SUPPORT_SCALE = np.array([1.20, 1.08, 0.035], dtype=float)
LIFT_FORK_SCALE = np.array([1.10, 0.12, 0.035], dtype=float)
LIFT_FORK_OFFSETS = (
    np.array([0.0, -0.24, 0.0], dtype=float),
    np.array([0.0, 0.24, 0.0], dtype=float),
)
LIFT_TO_PALLET_CONTACT_GAP = 0.005
UPSIDE_DOWN_BIN_QUAT = np.array([0.0, 0.0, 1.0, 0.0], dtype=float)
PALLET_DECK_OFFSETS = (
    np.array([0.0, 0.0, 0.02], dtype=float),
)
PALLET_RUNNER_OFFSETS = (
    np.array([0.0, -0.56, -0.06], dtype=float),
    np.array([0.0, 0.56, -0.06], dtype=float),
)
PALLET_GROOVE_OFFSETS = (
    np.array([0.0, -0.18, 0.055], dtype=float),
    np.array([0.0, 0.18, 0.055], dtype=float),
)
PALLET_BLOCK_OFFSETS = (
    np.array([-0.42, -0.56, -0.15], dtype=float),
    np.array([0.0, -0.56, -0.15], dtype=float),
    np.array([0.42, -0.56, -0.15], dtype=float),
    np.array([-0.42, 0.56, -0.15], dtype=float),
    np.array([0.0, 0.56, -0.15], dtype=float),
    np.array([0.42, 0.56, -0.15], dtype=float),
)
PALLET_SUPPORT_OFFSETS = (PALLET_TOP_SUPPORT_OFFSET,)
PALLET_PART_OFFSETS = (
    PALLET_DECK_OFFSETS
    + PALLET_RUNNER_OFFSETS
    + PALLET_BLOCK_OFFSETS
    + PALLET_GROOVE_OFFSETS
    + PALLET_SUPPORT_OFFSETS
)
PALLET_DECK_UNDERSIDE_Z = PALLET_CENTER_Z + PALLET_DECK_OFFSETS[0][2] - PALLET_DECK_SCALE[2] * 0.5
AMR_LIFT_PLATE_OFFSET_Z = (
    PALLET_DECK_UNDERSIDE_Z
    - LIFT_TO_PALLET_CONTACT_GAP
    - WORLD_FLOOR_Z
    - LIFT_FORK_SCALE[2] * 0.5
)
AMR_LIFT_GUIDE_BOTTOM_CLEARANCE = 0.08
AMR_LIFT_GUIDE_MAX_FORK_BOTTOM_Z = AMR_LIFT_PLATE_OFFSET_Z + DEFAULT_LIFT_HEIGHT - LIFT_FORK_SCALE[2] * 0.5
AMR_LIFT_GUIDE_TOP_Z = AMR_LIFT_GUIDE_MAX_FORK_BOTTOM_Z + 0.04
AMR_LIFT_GUIDE_HEIGHT = AMR_LIFT_GUIDE_TOP_Z - AMR_LIFT_GUIDE_BOTTOM_CLEARANCE
AMR_LIFT_GUIDE_CENTER_Z = AMR_LIFT_GUIDE_BOTTOM_CLEARANCE + AMR_LIFT_GUIDE_HEIGHT * 0.5
AMR_LIFT_GUIDE_SCALE = np.array([0.07, 0.07, AMR_LIFT_GUIDE_HEIGHT], dtype=float)
DROP_SLIDE_LANE_Y_OFFSETS = (-0.38, 0.38)
DROP_SLIDE_ROLLER_X_OFFSETS = (-0.62, -0.22, 0.22, 0.62)
DROP_SLIDE_LEG_X_OFFSETS = (-0.78, 0.78)
DROP_SLIDE_SUPPORT_GAP = 0.005
DROP_SLIDE_SUPPORT_TOP_Z = PALLET_DECK_UNDERSIDE_Z - DROP_SLIDE_SUPPORT_GAP
DROP_SLIDE_RAIL_SCALE = np.array([1.80, 0.08, 0.09], dtype=float)
DROP_SLIDE_ROLLER_SCALE = np.array([0.12, 0.08, 0.035], dtype=float)
DROP_SLIDE_TOP_SUPPORT_SCALE = np.array([1.95, 0.08, 0.035], dtype=float)
DROP_SLIDE_ROLLER_CENTER_Z = DROP_SLIDE_SUPPORT_TOP_Z - DROP_SLIDE_ROLLER_SCALE[2] * 0.5
DROP_SLIDE_TOP_SUPPORT_CENTER_Z = DROP_SLIDE_SUPPORT_TOP_Z - DROP_SLIDE_TOP_SUPPORT_SCALE[2] * 0.5
DROP_DOCK_STOP_GAP = 0.035
DROP_DOCK_STOP_BLOCK_SCALE = np.array([0.08, 0.09, 0.16], dtype=float)
DROP_DOCK_STOP_Y_OFFSETS = DROP_SLIDE_LANE_Y_OFFSETS
DROP_DOCK_STOP_X_OFFSET = PALLET_DECK_SCALE[0] * 0.5 + DROP_DOCK_STOP_GAP + DROP_DOCK_STOP_BLOCK_SCALE[0] * 0.5
DROP_DOCK_STOP_CENTER_Z = DROP_SLIDE_SUPPORT_TOP_Z + DROP_DOCK_STOP_BLOCK_SCALE[2] * 0.5
DROP_DOCK_GUIDE_POST_SCALE = np.array([0.06, 0.06, 0.42], dtype=float)
DROP_DOCK_GUIDE_POST_X_OFFSETS = (-0.58, 0.58)
DROP_DOCK_GUIDE_POST_Y_OFFSETS = (-0.72, 0.72)
DROP_DOCK_GUIDE_POST_CENTER_Z = WORLD_FLOOR_Z + DROP_DOCK_GUIDE_POST_SCALE[2] * 0.5
PICKUP_DOCK_STOP_GAP = 0.035
PICKUP_DOCK_STOP_BLOCK_SCALE = np.array([0.08, 0.09, 0.16], dtype=float)
PICKUP_DOCK_STOP_Y_OFFSETS = DROP_SLIDE_LANE_Y_OFFSETS
PICKUP_DOCK_STOP_X_OFFSET = -(
    PALLET_DECK_SCALE[0] * 0.5 + PICKUP_DOCK_STOP_GAP + PICKUP_DOCK_STOP_BLOCK_SCALE[0] * 0.5
)
PICKUP_DOCK_STOP_CENTER_Z = WORLD_FLOOR_Z + PICKUP_DOCK_STOP_BLOCK_SCALE[2] * 0.5
PICKUP_DOCK_GUIDE_POST_SCALE = np.array([0.06, 0.06, 0.38], dtype=float)
PICKUP_DOCK_GUIDE_POST_X_OFFSETS = (-0.58, 0.58)
PICKUP_DOCK_GUIDE_POST_Y_OFFSETS = (-0.72, 0.72)
PICKUP_DOCK_GUIDE_POST_CENTER_Z = WORLD_FLOOR_Z + PICKUP_DOCK_GUIDE_POST_SCALE[2] * 0.5
FLOOR_MARKING_Z = WORLD_FLOOR_Z + 0.004
FLOOR_MARKING_THICKNESS = 0.006
AMR_PATH_MARKING_WIDTH = 0.10
WORK_ZONE_MARKING_SIZE = np.array([1.65, 1.38], dtype=float)
WORK_ZONE_MARKING_EDGE_WIDTH = 0.055
PICKUP_ZONE_MARKING_COLOR = np.array([0.95, 0.74, 0.12], dtype=float)
DROP_ZONE_MARKING_COLOR = np.array([0.15, 0.58, 0.90], dtype=float)
AMR_PATH_MARKING_COLOR = np.array([0.95, 0.62, 0.10], dtype=float)
LOAD_RESTRAINT_EXPECTED_PARTS = 6
LOAD_RESTRAINT_STRAP_WIDTH = 0.035
LOAD_RESTRAINT_STRAP_THICKNESS = 0.012
LOAD_RESTRAINT_SURFACE_OFFSET = 0.006
LOAD_RESTRAINT_COLOR = np.array([0.05, 0.08, 0.12], dtype=float)
CARTON_BODY_SCALE = np.array([0.20, 0.29, 0.14], dtype=float)
CARTON_TAPE_TOP_SCALE = np.array([0.205, 0.030, 0.008], dtype=float)
CARTON_SIDE_LABEL_SCALE = np.array([0.140, 0.006, 0.055], dtype=float)
CARTON_SIDE_STRIPE_SCALE = np.array([0.030, 0.007, 0.065], dtype=float)
CARTON_BODY_COLOR = np.array([0.72, 0.48, 0.26], dtype=float)
CARTON_TAPE_COLOR = np.array([0.86, 0.10, 0.08], dtype=float)
CARTON_LABEL_COLOR = np.array([0.94, 0.90, 0.80], dtype=float)
INFEED_CONVEYOR_START_Y = float(PICK_STATION_BIN_POSITION[1] - 0.22)
INFEED_CONVEYOR_END_Y = float(CONVEYOR_PICK_WINDOW_Y + 0.42)
INFEED_CONVEYOR_WIDTH = 0.58
INFEED_CONVEYOR_THICKNESS = 0.035
INFEED_CONVEYOR_TOP_GAP = 0.008
INFEED_CONVEYOR_TOP_Z = float(PICK_STATION_BIN_POSITION[2] - CARTON_BODY_SCALE[2] * 0.5 - INFEED_CONVEYOR_TOP_GAP)
INFEED_CONVEYOR_CENTER_Y = (INFEED_CONVEYOR_START_Y + INFEED_CONVEYOR_END_Y) * 0.5
INFEED_CONVEYOR_LENGTH = INFEED_CONVEYOR_END_Y - INFEED_CONVEYOR_START_Y
INFEED_GUIDE_RAIL_Y_SCALE = INFEED_CONVEYOR_LENGTH
INFEED_GUIDE_RAIL_SCALE = np.array([0.035, INFEED_GUIDE_RAIL_Y_SCALE, 0.08], dtype=float)
INFEED_GUIDE_RAIL_X_OFFSETS = (-0.34, 0.34)
INFEED_GUIDE_INNER_WIDTH = abs(INFEED_GUIDE_RAIL_X_OFFSETS[1] - INFEED_GUIDE_RAIL_X_OFFSETS[0]) - INFEED_GUIDE_RAIL_SCALE[0]
INFEED_STOP_LINE_Y = float(PICK_STATION_BIN_POSITION[1])
INFEED_ROLLER_Y_OFFSETS = (-0.32, -0.14, 0.04, 0.22, 0.40)
INFEED_MOTION_MARKER_COUNT = 6
INFEED_MOTION_MARKER_SPEED = 0.22
INFEED_MOTION_MARKER_SCALE = np.array([INFEED_CONVEYOR_WIDTH * 0.76, 0.035, 0.006], dtype=float)
INFEED_MOTION_MARKER_Z = INFEED_CONVEYOR_TOP_Z + INFEED_MOTION_MARKER_SCALE[2] * 0.5 + 0.002
INFEED_MOTION_MARKER_COLOR = np.array([0.22, 0.26, 0.28], dtype=float)
INFEED_FEED_CARTON_COUNT = 1
INFEED_FEED_CARTON_SPEED = 0.12
INFEED_FEED_CARTON_MIN_Y = float(INFEED_STOP_LINE_Y + CARTON_BODY_SCALE[1] * 0.5 + 0.07)
INFEED_FEED_CARTON_MAX_Y = float(INFEED_CONVEYOR_END_Y - CARTON_BODY_SCALE[1] * 0.5 - 0.02)
INFEED_FEED_CARTON_PATH_LENGTH = INFEED_FEED_CARTON_MAX_Y - INFEED_FEED_CARTON_MIN_Y
INFEED_FEED_CARTON_TAPE_OFFSET_Z = float(CARTON_BODY_SCALE[2] * 0.5 + CARTON_TAPE_TOP_SCALE[2] * 0.5)
ACTIVE_BIN_CONVEYOR_APPROACH_DURATION = 0.60
ACTIVE_BIN_CONVEYOR_START_POSITION = np.array(
    [PICK_STATION_BIN_POSITION[0], CONVEYOR_PICK_WINDOW_Y, PICK_STATION_BIN_POSITION[2]],
    dtype=float,
)
SAFETY_FENCE_MIN_X = -0.82
SAFETY_FENCE_MAX_X = 1.76
SAFETY_FENCE_MIN_Y = -1.18
SAFETY_FENCE_MAX_Y = 1.22
SAFETY_FENCE_POST_SCALE = np.array([0.055, 0.055, 1.05], dtype=float)
SAFETY_FENCE_RAIL_THICKNESS = 0.035
SAFETY_FENCE_RAIL_Z_OFFSETS = (0.42, 0.92)
SAFETY_FENCE_AMR_GATE_WIDTH = 1.44
SAFETY_FENCE_INFEED_GATE_WIDTH = 0.86
AMR_SAFETY_VISUAL_SPECS = (
    (
        "AmrBeaconPole",
        "cuboid",
        np.array([0.0, 0.0, 0.50], dtype=float),
        np.array([0.035, 0.035, 0.30], dtype=float),
        np.array([0.08, 0.08, 0.08], dtype=float),
    ),
    (
        "AmrBeaconDome",
        "sphere",
        np.array([0.0, 0.0, 0.68], dtype=float),
        0.065,
        np.array([0.95, 0.58, 0.08], dtype=float),
    ),
    (
        "AmrFrontSafetyScanner",
        "cuboid",
        np.array([-0.62, 0.0, 0.16], dtype=float),
        np.array([0.050, 0.56, 0.055], dtype=float),
        np.array([0.08, 0.55, 0.95], dtype=float),
    ),
    (
        "AmrRearSafetyScanner",
        "cuboid",
        np.array([0.62, 0.0, 0.16], dtype=float),
        np.array([0.050, 0.56, 0.055], dtype=float),
        np.array([0.08, 0.55, 0.95], dtype=float),
    ),
    (
        "AmrLeftStatusStrip",
        "cuboid",
        np.array([0.0, -0.42, 0.34], dtype=float),
        np.array([0.32, 0.030, 0.045], dtype=float),
        np.array([0.04, 0.75, 0.28], dtype=float),
    ),
    (
        "AmrRightStatusStrip",
        "cuboid",
        np.array([0.0, 0.42, 0.34], dtype=float),
        np.array([0.32, 0.030, 0.045], dtype=float),
        np.array([0.04, 0.75, 0.28], dtype=float),
    ),
    (
        "AmrLeftWarningStrip",
        "cuboid",
        np.array([0.0, -0.425, 0.42], dtype=float),
        np.array([0.34, 0.030, 0.045], dtype=float),
        np.array([0.95, 0.58, 0.08], dtype=float),
    ),
    (
        "AmrRightWarningStrip",
        "cuboid",
        np.array([0.0, 0.425, 0.42], dtype=float),
        np.array([0.34, 0.030, 0.045], dtype=float),
        np.array([0.95, 0.58, 0.08], dtype=float),
    ),
)
AMR_WARNING_INDICATOR_NAMES = {"AmrBeaconDome", "AmrLeftWarningStrip", "AmrRightWarningStrip"}
AMR_IDLE_INDICATOR_NAMES = {"AmrLeftStatusStrip", "AmrRightStatusStrip"}
AMR_DRIVE_VISUAL_SPECS = (
    (
        "AmrFrontLeftDriveWheel",
        np.array([-0.43, -0.48, 0.075], dtype=float),
        np.array([0.30, 0.095, 0.15], dtype=float),
        np.array([0.035, 0.040, 0.045], dtype=float),
    ),
    (
        "AmrFrontRightDriveWheel",
        np.array([-0.43, 0.48, 0.075], dtype=float),
        np.array([0.30, 0.095, 0.15], dtype=float),
        np.array([0.035, 0.040, 0.045], dtype=float),
    ),
    (
        "AmrRearLeftDriveWheel",
        np.array([0.43, -0.48, 0.075], dtype=float),
        np.array([0.30, 0.095, 0.15], dtype=float),
        np.array([0.035, 0.040, 0.045], dtype=float),
    ),
    (
        "AmrRearRightDriveWheel",
        np.array([0.43, 0.48, 0.075], dtype=float),
        np.array([0.30, 0.095, 0.15], dtype=float),
        np.array([0.035, 0.040, 0.045], dtype=float),
    ),
    (
        "AmrFrontCasterWheel",
        np.array([-0.66, 0.0, 0.055], dtype=float),
        np.array([0.12, 0.22, 0.11], dtype=float),
        np.array([0.050, 0.055, 0.060], dtype=float),
    ),
    (
        "AmrRearCasterWheel",
        np.array([0.66, 0.0, 0.055], dtype=float),
        np.array([0.12, 0.22, 0.11], dtype=float),
        np.array([0.050, 0.055, 0.060], dtype=float),
    ),
)
AMR_LIFT_GUIDE_VISUAL_SPECS = tuple(
    (
        f"AmrLiftGuide_{x_idx}_{y_idx}",
        np.array([x_offset, y_offset, AMR_LIFT_GUIDE_CENTER_Z], dtype=float),
        AMR_LIFT_GUIDE_SCALE.copy(),
        np.array([0.09, 0.11, 0.13], dtype=float),
    )
    for x_idx, x_offset in enumerate((-0.48, 0.48))
    for y_idx, y_offset in enumerate((-0.34, 0.34))
)
CAMERA_RIG_REQUIRED_ROLES = ("overview", "palletizer", "amr_route", "drop_dock")
CAMERA_MIN_HEIGHT_ABOVE_FLOOR = 1.25
CAMERA_MIN_TARGET_DISTANCE = 1.0
CAMERA_DEFAULT_FOCAL_LENGTH = 30.0
WAREHOUSE_LIGHT_REQUIRED_ROLES = ("palletizer_key", "route_fill", "drop_key")
WAREHOUSE_LIGHT_FIXTURE_COLOR = np.array([0.82, 0.84, 0.80], dtype=float)
WAREHOUSE_LIGHT_EMISSIVE_COLOR = np.array([1.0, 0.93, 0.80], dtype=float)
WAREHOUSE_LIGHT_MIN_HEIGHT_ABOVE_FLOOR = 3.2
WAREHOUSE_LIGHT_MIN_ROUTE_SPAN = 8.0
AMR_ROUTE_GUARD_Y_OFFSET = 0.95
AMR_ROUTE_GUARD_X_MARGIN_START = 1.45
AMR_ROUTE_GUARD_X_MARGIN_END = 0.75
AMR_ROUTE_GUARD_BOLLARD_COUNT_PER_SIDE = 6
AMR_ROUTE_GUARD_BOLLARD_SCALE = np.array([0.08, 0.08, 0.72], dtype=float)
AMR_ROUTE_GUARD_RAIL_SCALE_YZ = np.array([0.035, 0.050], dtype=float)
AMR_ROUTE_GUARD_BOLLARD_COLOR = np.array([0.94, 0.74, 0.10], dtype=float)
AMR_ROUTE_GUARD_RAIL_COLOR = np.array([0.08, 0.09, 0.10], dtype=float)


def compute_review_gif_layout(canvas_size=GIF_CANVAS_SIZE):
    width, height = (int(canvas_size[0]), int(canvas_size[1]))
    top = 62
    bottom = height - 38
    side_margin = 28
    panel_width = max(280, min(340, int(width * 0.32)))
    panel_left = width - side_margin - panel_width
    map_rect = (34, top, panel_left - 24, bottom)
    panel_rect = (panel_left, top, width - side_margin, bottom)
    return map_rect, panel_rect


def parse_args():
    parser = argparse.ArgumentParser(description="Harim UR10 palletizing + iw_hub AMR transfer demo.")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without a GUI.")
    parser.add_argument("--cycles", type=int, default=0, help="Completed transfer cycles. 0 means infinite.")
    parser.add_argument("--stack-cols", type=int, default=2, help="Number of stack columns along X.")
    parser.add_argument("--stack-rows", type=int, default=2, help="Number of stack rows along Y.")
    parser.add_argument("--stack-layers", type=int, default=2, help="Number of stack layers along Z.")
    parser.add_argument("--amr-z", type=float, default=DEFAULT_AMR_Z, help="World Z origin for the iw_hub actor.")
    parser.add_argument("--lift-height", type=float, default=DEFAULT_LIFT_HEIGHT, help="Visual lift height for pallet pickup.")
    parser.add_argument("--move-speed", type=float, default=DEFAULT_MOVE_SPEED, help="Scripted iw_hub movement speed in m/s.")
    parser.add_argument("--pickup-x", type=float, default=DEFAULT_PICKUP_X, help="Pickup X under the pallet stack.")
    parser.add_argument("--pickup-y", type=float, default=DEFAULT_PICKUP_Y, help="Pickup Y under the pallet stack.")
    parser.add_argument("--drop-x", type=float, default=DEFAULT_DROP_X, help="Drop X for delivered pallet. Default is over 10 m from pickup.")
    parser.add_argument("--drop-y", type=float, default=DEFAULT_DROP_Y, help="Drop Y for delivered pallet.")
    parser.add_argument(
        "--gif-output-dir",
        type=Path,
        default=DEFAULT_GIF_OUTPUT_DIR,
        help="Directory for per-run review GIFs.",
    )
    parser.add_argument(
        "--gif-frame-stride",
        type=int,
        default=GIF_FRAME_STRIDE,
        help="Capture one review GIF frame every N simulation frames.",
    )
    parser.add_argument(
        "--gif-max-frames",
        type=int,
        default=GIF_MAX_FRAMES,
        help="Maximum frames retained for the review GIF.",
    )
    parser.add_argument(
        "--self-test-require-review-gif",
        action="store_true",
        help="Fail the fixed-frame self-test unless the timestamp review GIF and latest_review.gif are saved.",
    )
    parser.add_argument(
        "--self-test-frames",
        type=int,
        default=0,
        help="Run a fixed number of simulation frames and exit. 0 keeps the demo running.",
    )
    parser.add_argument(
        "--self-test-force-stack-complete",
        action="store_true",
        help="Force stack_complete during self-test so the AMR transfer sequence can be verified quickly.",
    )
    parser.add_argument(
        "--self-test-min-placed-bins",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many bins are placed by the UR10.",
    )
    parser.add_argument(
        "--self-test-min-transfer-cycles",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless the AMR completes at least this many transfer cycles.",
    )
    parser.add_argument(
        "--self-test-debug-bins",
        action="store_true",
        help="Print bin spawn, active-bin, and stack-count transitions during fixed-frame self-tests.",
    )
    parser.add_argument(
        "--self-test-max-pre-grip-offset",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if any scripted pre-grip correction exceeds this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-return-ready-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if return_ready finishes farther than this distance from its target in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-release-drift",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if a released bin drifts from its snapped stack pose by more than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-release-retreat-lift",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test unless the arm retreats upward by at least this distance after releasing a bin. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-scripted-place-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless the final scripted place state runs at least this many times. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-scripted-place-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the final scripted place pose is farther than this distance from the stack target. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-release-separation",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test unless the released box separates from the suction TCP by at least this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-release-vertical-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test unless the suction TCP rises above the released box by at least this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-require-gripper-open-after-release",
        action="store_true",
        help="Fail the fixed-frame self-test if the surface gripper is not open or still reports gripped objects after a scripted release.",
    )
    parser.add_argument(
        "--self-test-min-attached-grasp-sample-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless the suction-attached carton alignment is sampled at least this many times. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-attached-grasp-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the suction-attached carton lags farther than this distance from the gripper target. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-stack-lateral-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if adjacent boxes have a lateral air gap larger than this distance in meters, or overlap by more than 5 mm. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-stack-support-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if boxes float above the pallet or lower layer by more than this distance in meters, or overlap vertically by more than 5 mm. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-stack-pallet-margin",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the carton stack footprint leaves less pallet deck margin than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-load-restraint-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many load restraint visual parts are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-load-restraint-pallet-margin",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if load restraint visuals leave less pallet deck margin than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-infeed-conveyor-length",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the infeed conveyor visual is shorter than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-infeed-spawn-margin",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the conveyor visual does not extend past the spawn point by at least this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-infeed-guide-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if infeed guide rails leave less carton side clearance than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-infeed-belt-support-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the conveyor belt visual is farther below carton bottom than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-infeed-motion-marker-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many moving conveyor belt markers are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-infeed-motion-observed-travel",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test unless conveyor belt markers visibly travel at least this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-infeed-feed-carton-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many upstream infeed carton visuals are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-infeed-feed-carton-observed-travel",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test unless upstream infeed carton visuals visibly travel at least this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-infeed-feed-carton-stop-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if upstream infeed carton visuals come closer to the pick stop line than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-infeed-feed-carton-guide-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if upstream infeed carton visuals have less side guide clearance than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-infeed-feed-carton-belt-support-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if upstream infeed carton visuals float above the belt by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-active-bin-conveyor-approach-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless this many picked cartons completed the scripted conveyor approach into the pick station. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-active-bin-conveyor-observed-travel",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test unless each active carton conveyor approach visibly travels at least this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-active-bin-conveyor-final-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if a carton finishes its conveyor approach farther than this distance from the pick station. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-active-bin-conveyor-lateral-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if a carton drifts laterally from the conveyor centerline during the scripted approach. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-active-bin-conveyor-belt-support-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the active carton visual floats above the infeed belt by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-motion-continuity-sample-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless motion continuity sampled at least this many AMR frames. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-amr-frame-displacement",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR moves farther than this distance between sampled frames. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-arm-ee-frame-displacement",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the robot arm end-effector moves farther than this distance between sampled frames. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-measured-arm-fk-fallbacks",
        type=int,
        default=-1,
        help="Fail the fixed-frame self-test if measured joint-state FK falls back to command FK more than this many times. -1 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-active-bin-frame-displacement",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if any active carton moves farther than this distance between sampled frames. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-attached-bin-frame-displacement",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the suction-attached carton moves farther than this distance between sampled frames. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-scripted-place-bin-frame-displacement",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the scripted place carton moves farther than this distance between sampled frames. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-released-bin-frame-displacement",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if a released carton moves farther than this distance between sampled frames. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-carried-payload-frame-displacement",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if any carried pallet payload part moves farther than this distance between sampled frames. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-carried-pallet-frame-displacement",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if any carried pallet part moves farther than this distance between sampled frames. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-safety-fence-part-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many palletizer safety fence parts are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-safety-fence-amr-gate-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the safety fence AMR gate leaves less loaded-pallet side clearance than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-cell-gate-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the moving AMR/load deviates from the palletizer cell gate centerline enough to leave less clearance than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-safety-fence-infeed-gate-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the safety fence infeed gate leaves less conveyor side clearance than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-safety-part-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many AMR beacon/scanner visual parts are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-safety-beacon-height",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR beacon visual top is lower than this height above the AMR base. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-safety-scanner-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if AMR scanner visuals are closer to the floor than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-amr-safety-pose-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if AMR beacon/scanner visuals drift from the AMR pose by more than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-amr-orientation-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR body rotates away from the planned yaw by more than this angle in radians. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-warning-indicator-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many AMR warning indicator visuals are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-idle-indicator-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many AMR idle indicator visuals are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-warning-observed",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless AMR warning indicators are observed visible at least this many times. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-idle-observed",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless AMR idle indicators are observed visible at least this many times. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-amr-indicator-visibility-mismatches",
        type=int,
        default=-1,
        help="Fail the fixed-frame self-test if AMR indicator visibility writes mismatch the requested state more than this many times. -1 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-drive-part-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many AMR drive wheel/caster visual parts are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-amr-drive-pose-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if AMR drive visuals drift from the AMR pose by more than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-amr-wheel-floor-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR wheel visuals float above the floor by more than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-amr-wheel-floor-penetration",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR wheel visuals penetrate below the floor by more than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-lift-guide-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many AMR lift guide visuals are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-amr-lift-guide-bottom-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if AMR lift guide visuals float above the AMR base by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-lift-guide-travel-cover",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if AMR lift guide visuals do not cover the raised fork travel by at least this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-amr-lift-guide-pose-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if AMR lift guide visuals drift from the AMR pose by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-amr-lift-orientation-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if AMR lift/fork visuals rotate away from the AMR yaw or lift baseline by more than this angle in radians. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-payload-lift",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test unless the AMR visibly lifts the payload by at least this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-lift-offset-frame-step",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR lift offset changes by more than this distance between orchestrator steps. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-dropped-payload-drift",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the delivered pallet assembly drifts after detach by more than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-dropped-stack-item-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless this many cartons are recorded on the dropped pallet after detach. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-dropped-stack-pose-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if dropped cartons are farther from their expected pallet-relative pose than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-dropped-stack-orientation-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if dropped cartons rotate away from their expected pallet-relative orientations by more than this angle in radians. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-dropped-stack-support-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if dropped cartons float above their pallet/lower-carton support by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-dropped-stack-pallet-margin",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if dropped cartons have less pallet footprint margin than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-dropped-pallet-part-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless this many pallet/load-restraint parts are recorded after detach. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-dropped-pallet-part-pose-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if dropped pallet parts drift from their deck-relative offsets by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-dropped-pallet-part-orientation-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if dropped pallet parts rotate away from their deck-relative orientations by more than this angle in radians. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-exit-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR lift forks do not fully clear the dropped pallet by at least this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-lift-contact-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the lift plate is farther below the pallet deck underside than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-pallet-tunnel-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the lift plate has less lateral clearance inside the pallet tunnel than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-lift-fork-inner-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the visual lift forks are not separated by at least this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-pickup-handoff-xy-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR lift center is farther from the pallet center at pickup handoff than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-pickup-handoff-lift-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the lift forks are farther below the pallet underside at pickup handoff than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-pickup-handoff-lift-penetration",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the lift forks penetrate the pallet underside at pickup handoff by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-pickup-entry-y-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR drifts laterally from the pallet tunnel centerline while entering for pickup. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-pickup-entry-tunnel-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if lateral AMR drift leaves less dynamic clearance in the pallet tunnel while entering for pickup. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-pickup-entry-lift-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the lowered lift forks are farther below the pallet underside while entering for pickup than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-pickup-entry-lift-penetration",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the lowered lift forks penetrate the pallet underside while entering for pickup by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-slide-out-y-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR drifts laterally from the dropped pallet centerline while sliding out. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-slide-out-lift-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the lowered lift forks are farther below the dropped pallet underside while sliding out than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-slide-out-lift-penetration",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the lowered lift forks penetrate the dropped pallet underside while sliding out by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-drop-support-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the drop workstation support is farther below the pallet deck underside than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-drop-handoff-xy-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the delivered pallet center is farther from the drop workstation center than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-drop-handoff-support-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the delivered pallet underside is farther above the drop workstation support than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-drop-handoff-support-penetration",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the delivered pallet underside penetrates below the drop workstation support by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-drop-lane-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the drop workstation lanes have less tunnel clearance than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-drop-runner-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the drop workstation lanes are closer to the pallet side runners than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-drop-fork-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the drop workstation lanes are closer to the AMR lift forks than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-drop-dock-stop-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many drop dock stop blocks are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-drop-dock-stop-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the drop dock stop blocks are farther from the pallet front than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-drop-dock-guide-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if drop dock locator posts leave less pallet side clearance than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-drop-dock-fork-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if drop dock stop blocks are closer to AMR lift forks than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-pickup-dock-stop-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many pickup dock stop blocks are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-pickup-dock-stop-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if pickup dock stop blocks are farther from the pallet rear than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-pickup-dock-guide-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if pickup dock locator posts leave less pallet side clearance than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-pickup-dock-fork-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if pickup dock stop blocks are closer to AMR lift forks than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-pickup-dock-runner-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if pickup dock stop blocks are closer to pallet side runners than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-camera-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many story cameras are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-camera-role-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many required camera roles are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-camera-height",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if any story camera is lower than this height above the floor. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-camera-target-distance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if any story camera is closer to its target than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-camera-director-switch-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless the story camera director switches at least this many times. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-camera-director-role-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless the story camera director visits at least this many roles. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-warehouse-light-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many warehouse high-bay lights are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-warehouse-light-role-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many warehouse light roles are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-warehouse-light-height",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if any warehouse high-bay light is lower than this height above the floor. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-warehouse-light-route-span",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if warehouse lights cover less X span than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-warehouse-light-intensity",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if any warehouse high-bay light intensity is below this value. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-route-guard-part-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless at least this many AMR route guard parts are configured. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-route-guard-span",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if AMR route guard visuals cover less X span than this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-route-guard-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if AMR route guard visuals leave less loaded-pallet side clearance than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-amr-route-bollard-height",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if AMR route bollards are shorter than this height above the floor. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-loaded-route-y-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the loaded AMR deviates from the planned route centerline by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-loaded-route-guard-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the carried pallet leaves less side clearance to AMR route guards than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-arm-tcp-amr-route-clearance",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the robot TCP comes closer than this horizontal clearance to the loaded AMR route. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-carried-pallet-pose-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if carried pallet visuals drift from their AMR-relative offsets by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-carried-pallet-orientation-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if carried pallet visuals rotate away from their AMR-relative orientations by more than this angle in radians. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-carried-payload-pose-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if carried stacked payload items drift from their AMR-relative offsets by more than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-carried-payload-orientation-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if carried stacked payload items rotate away from their AMR-relative orientations by more than this angle in radians. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-drop-approach-standoff",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR drop approach waypoint is closer to the final drop pose than this distance. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-min-drop-dock-arrival-count",
        type=int,
        default=0,
        help="Fail the fixed-frame self-test unless the AMR reaches the drop approach waypoint at least this many times. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-drop-dock-final-error",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the AMR final dock-in position is farther from the drop pose than this distance. 0 disables the check.",
    )
    return parser.parse_args()


def configure_local_runtime_dirs():
    runtime_dirs = {
        "OMNI_USER_HOME": PROJECT_ROOT / ".omni_user",
        "OMNI_CACHE_DIR": PROJECT_ROOT / ".omni_cache",
        "OMNI_KIT_CACHE_ROOT": PROJECT_ROOT / ".kit_cache",
    }
    for key, path in runtime_dirs.items():
        path.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault(key, str(path))


def wait_for_stage_loading(simulation_app, usd_context, label, max_updates=600):
    for _ in range(max_updates):
        simulation_app.update()
        status = usd_context.get_stage_loading_status()
        if status[2] <= 0:
            return
    raise RuntimeError(f"Timed out waiting for USD assets to load: {label}; status={status}")


def deactivate_stage_prims_containing(stage, root_path, patterns):
    from pxr import Usd

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return
    lower_patterns = tuple(pattern.lower() for pattern in patterns)
    for prim in Usd.PrimRange(root):
        name = prim.GetName().lower()
        if any(pattern in name for pattern in lower_patterns):
            prim.SetActive(False)


class TransferState(Enum):
    WAIT_STACK_COMPLETE = auto()
    ARM_SETTLE = auto()
    MOVE_TO_APPROACH = auto()
    MOVE_UNDER_PALLET = auto()
    LIFT_UP = auto()
    ATTACH = auto()
    MOVE_TO_DROP_APPROACH = auto()
    MOVE_TO_DROP = auto()
    LIFT_DOWN = auto()
    DETACH = auto()
    SLIDE_OUT_FROM_PALLET = auto()
    RESET_CYCLE = auto()
    DONE_IDLE = auto()


CAMERA_DIRECTOR_ROLE_BY_STATE_NAME = {
    "WAIT_STACK_COMPLETE": "palletizer",
    "ARM_SETTLE": "overview",
    "MOVE_TO_APPROACH": "amr_route",
    "MOVE_UNDER_PALLET": "amr_route",
    "LIFT_UP": "amr_route",
    "ATTACH": "amr_route",
    "MOVE_TO_DROP_APPROACH": "amr_route",
    "MOVE_TO_DROP": "amr_route",
    "LIFT_DOWN": "drop_dock",
    "DETACH": "drop_dock",
    "SLIDE_OUT_FROM_PALLET": "drop_dock",
    "RESET_CYCLE": "overview",
    "DONE_IDLE": "overview",
}


def camera_role_for_transfer_state(state):
    state_name = state.name if hasattr(state, "name") else str(state)
    return CAMERA_DIRECTOR_ROLE_BY_STATE_NAME.get(state_name, "overview")


def compute_camera_director_metrics():
    ordered_roles = [camera_role_for_transfer_state(state) for state in TransferState]
    switch_count = 0
    previous_role = None
    for role in ordered_roles:
        if role != previous_role:
            switch_count += 1
            previous_role = role
    return {
        "camera_director_role_count": len(set(ordered_roles)),
        "camera_director_planned_switch_count": switch_count,
    }


class CompletionSignalController:
    def __init__(self, red_light=None, green_light=None):
        self.red_light = red_light
        self.green_light = green_light
        self.completed = False

    def set_completed(self, completed):
        self.completed = bool(completed)
        if self.red_light is not None:
            self.red_light.set_visibility(not self.completed)
        if self.green_light is not None:
            self.green_light.set_visibility(self.completed)


def clamp(value, low, high):
    return max(low, min(high, value))


def lerp(a, b, t):
    return a + (b - a) * t


def smoothstep(t):
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)


def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    quat = np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )
    return quat / max(np.linalg.norm(quat), 1e-9)


def quat_lerp(q1, q2, t):
    q1 = np.array(q1, dtype=float)
    q2 = np.array(q2, dtype=float)
    if np.dot(q1, q2) < 0.0:
        q2 = -q2
    quat = (1.0 - t) * q1 + t * q2
    return quat / max(np.linalg.norm(quat), 1e-9)


def quat_angular_error(q1, q2):
    q1 = np.array(q1, dtype=float)
    q2 = np.array(q2, dtype=float)
    q1 = q1 / max(np.linalg.norm(q1), 1e-9)
    q2 = q2 / max(np.linalg.norm(q2), 1e-9)
    dot = abs(float(np.dot(q1, q2)))
    return float(2.0 * math.acos(clamp(dot, -1.0, 1.0)))


def get_measured_arm_fk_T(context):
    arm = getattr(getattr(context, "robot", None), "arm", None)
    if arm is None:
        raise RuntimeError("Robot arm is not available")
    try:
        joint_indices = np.array(arm.aji, dtype=int)
        positions = np.array(arm.robot.get_joint_positions(joint_indices=joint_indices), dtype=float)
        if positions.size > 0:
            context.demo_measured_arm_fk_sample_count = int(
                getattr(context, "demo_measured_arm_fk_sample_count", 0)
            ) + 1
            return arm.get_fk_T(positions)
    except Exception:
        pass
    context.demo_measured_arm_fk_fallback_count = int(
        getattr(context, "demo_measured_arm_fk_fallback_count", 0)
    ) + 1
    return arm.get_fk_T()


def get_measured_arm_fk_p(context):
    return np.array(get_measured_arm_fk_T(context)[:3, 3], dtype=float)


def make_stack_coordinates(cols, rows, layers):
    cols = max(1, cols)
    rows = max(1, rows)
    layers = max(1, layers)

    x0 = 1.05
    y0 = -0.62
    z0 = -0.51
    dx = -0.21
    dy = 0.31
    dz = 0.15

    coords = []
    for layer in range(layers):
        for row in range(rows):
            for col in range(cols):
                coords.append(np.array([x0 + dx * col, y0 + dy * row, z0 + dz * layer], dtype=float))
    return coords


def compute_stack_geometry_metrics(stack_coordinates, cols, rows, layers):
    coords = clone_stack_coordinates(stack_coordinates)
    cols = max(1, cols)
    rows = max(1, rows)
    layers = max(1, layers)

    def coord(layer, row, col):
        return coords[(layer * rows * cols) + (row * cols) + col]

    lateral_gaps = []
    support_gaps = []

    for layer in range(layers):
        for row in range(rows):
            for col in range(max(0, cols - 1)):
                gap = abs(coord(layer, row, col + 1)[0] - coord(layer, row, col)[0]) - CARTON_BODY_SCALE[0]
                lateral_gaps.append(float(gap))

    for layer in range(layers):
        for row in range(max(0, rows - 1)):
            for col in range(cols):
                gap = abs(coord(layer, row + 1, col)[1] - coord(layer, row, col)[1]) - CARTON_BODY_SCALE[1]
                lateral_gaps.append(float(gap))

    pallet_support_top_z = PALLET_CENTER_Z + PALLET_TOP_SUPPORT_OFFSET[2] + PALLET_TOP_SUPPORT_SCALE[2] * 0.5
    for row in range(rows):
        for col in range(cols):
            bottom_z = coord(0, row, col)[2] - CARTON_BODY_SCALE[2] * 0.5
            support_gaps.append(float(bottom_z - pallet_support_top_z))

    for layer in range(max(0, layers - 1)):
        for row in range(rows):
            for col in range(cols):
                upper_bottom_z = coord(layer + 1, row, col)[2] - CARTON_BODY_SCALE[2] * 0.5
                lower_top_z = coord(layer, row, col)[2] + CARTON_BODY_SCALE[2] * 0.5
                support_gaps.append(float(upper_bottom_z - lower_top_z))

    lateral_gaps = lateral_gaps or [0.0]
    support_gaps = support_gaps or [0.0]
    return {
        "max_stack_lateral_gap": float(max(lateral_gaps)),
        "min_stack_lateral_gap": float(min(lateral_gaps)),
        "max_stack_support_gap": float(max(support_gaps)),
        "min_stack_support_gap": float(min(support_gaps)),
    }


def compute_stack_pallet_footprint_metrics(
    stack_coordinates,
    pallet_center_x=DEFAULT_PICKUP_X,
    pallet_center_y=DEFAULT_PICKUP_Y,
):
    coords = clone_stack_coordinates(stack_coordinates)
    if not coords:
        return {
            "min_stack_pallet_margin": 0.0,
            "max_stack_pallet_overhang": 0.0,
        }

    stack_min_x = min(float(coord[0]) - CARTON_BODY_SCALE[0] * 0.5 for coord in coords)
    stack_max_x = max(float(coord[0]) + CARTON_BODY_SCALE[0] * 0.5 for coord in coords)
    stack_min_y = min(float(coord[1]) - CARTON_BODY_SCALE[1] * 0.5 for coord in coords)
    stack_max_y = max(float(coord[1]) + CARTON_BODY_SCALE[1] * 0.5 for coord in coords)

    pallet_min_x = float(pallet_center_x) - PALLET_DECK_SCALE[0] * 0.5
    pallet_max_x = float(pallet_center_x) + PALLET_DECK_SCALE[0] * 0.5
    pallet_min_y = float(pallet_center_y) - PALLET_DECK_SCALE[1] * 0.5
    pallet_max_y = float(pallet_center_y) + PALLET_DECK_SCALE[1] * 0.5

    margins = [
        stack_min_x - pallet_min_x,
        pallet_max_x - stack_max_x,
        stack_min_y - pallet_min_y,
        pallet_max_y - stack_max_y,
    ]
    min_margin = float(min(margins))
    return {
        "min_stack_pallet_margin": min_margin,
        "max_stack_pallet_overhang": float(max(0.0, -min_margin)),
    }


def compute_stack_extents(stack_coordinates):
    coords = clone_stack_coordinates(stack_coordinates)
    if not coords:
        return None
    return {
        "min_x": min(float(coord[0]) - CARTON_BODY_SCALE[0] * 0.5 for coord in coords),
        "max_x": max(float(coord[0]) + CARTON_BODY_SCALE[0] * 0.5 for coord in coords),
        "min_y": min(float(coord[1]) - CARTON_BODY_SCALE[1] * 0.5 for coord in coords),
        "max_y": max(float(coord[1]) + CARTON_BODY_SCALE[1] * 0.5 for coord in coords),
        "min_z": min(float(coord[2]) - CARTON_BODY_SCALE[2] * 0.5 for coord in coords),
        "max_z": max(float(coord[2]) + CARTON_BODY_SCALE[2] * 0.5 for coord in coords),
    }


def compute_load_restraint_specs(
    stack_coordinates,
    pallet_center_x=DEFAULT_PICKUP_X,
    pallet_center_y=DEFAULT_PICKUP_Y,
):
    extents = compute_stack_extents(stack_coordinates)
    if extents is None:
        return []

    min_x = extents["min_x"]
    max_x = extents["max_x"]
    min_y = extents["min_y"]
    max_y = extents["max_y"]
    min_z = extents["min_z"]
    max_z = extents["max_z"]
    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5
    center_z = (min_z + max_z) * 0.5
    x_span = (max_x - min_x) + 2.0 * LOAD_RESTRAINT_SURFACE_OFFSET
    y_span = (max_y - min_y) + 2.0 * LOAD_RESTRAINT_SURFACE_OFFSET
    z_span = (max_z - min_z) + 2.0 * LOAD_RESTRAINT_SURFACE_OFFSET
    top_z = max_z + LOAD_RESTRAINT_SURFACE_OFFSET + LOAD_RESTRAINT_STRAP_THICKNESS * 0.5
    front_y = max_y + LOAD_RESTRAINT_SURFACE_OFFSET + LOAD_RESTRAINT_STRAP_THICKNESS * 0.5
    back_y = min_y - LOAD_RESTRAINT_SURFACE_OFFSET - LOAD_RESTRAINT_STRAP_THICKNESS * 0.5
    right_x = max_x + LOAD_RESTRAINT_SURFACE_OFFSET + LOAD_RESTRAINT_STRAP_THICKNESS * 0.5
    left_x = min_x - LOAD_RESTRAINT_SURFACE_OFFSET - LOAD_RESTRAINT_STRAP_THICKNESS * 0.5

    def offset(world_position):
        position = np.array(world_position, dtype=float)
        return position - np.array([pallet_center_x, pallet_center_y, PALLET_CENTER_Z], dtype=float)

    return [
        (
            "LoadStrapTopLongitudinal",
            offset([center_x, center_y, top_z]),
            np.array([LOAD_RESTRAINT_STRAP_WIDTH, y_span, LOAD_RESTRAINT_STRAP_THICKNESS], dtype=float),
        ),
        (
            "LoadStrapTopLateral",
            offset([center_x, center_y, top_z + LOAD_RESTRAINT_STRAP_THICKNESS]),
            np.array([x_span, LOAD_RESTRAINT_STRAP_WIDTH, LOAD_RESTRAINT_STRAP_THICKNESS], dtype=float),
        ),
        (
            "LoadStrapFrontVertical",
            offset([center_x, front_y, center_z]),
            np.array([LOAD_RESTRAINT_STRAP_WIDTH, LOAD_RESTRAINT_STRAP_THICKNESS, z_span], dtype=float),
        ),
        (
            "LoadStrapBackVertical",
            offset([center_x, back_y, center_z]),
            np.array([LOAD_RESTRAINT_STRAP_WIDTH, LOAD_RESTRAINT_STRAP_THICKNESS, z_span], dtype=float),
        ),
        (
            "LoadStrapLeftVertical",
            offset([left_x, center_y, center_z]),
            np.array([LOAD_RESTRAINT_STRAP_THICKNESS, LOAD_RESTRAINT_STRAP_WIDTH, z_span], dtype=float),
        ),
        (
            "LoadStrapRightVertical",
            offset([right_x, center_y, center_z]),
            np.array([LOAD_RESTRAINT_STRAP_THICKNESS, LOAD_RESTRAINT_STRAP_WIDTH, z_span], dtype=float),
        ),
    ]


def compute_load_restraint_metrics(
    stack_coordinates,
    pallet_center_x=DEFAULT_PICKUP_X,
    pallet_center_y=DEFAULT_PICKUP_Y,
):
    specs = compute_load_restraint_specs(stack_coordinates, pallet_center_x, pallet_center_y)
    if not specs:
        return {
            "load_restraint_part_count": 0,
            "min_load_restraint_pallet_margin": 0.0,
            "max_load_restraint_pallet_overhang": 0.0,
        }

    pallet_min_x = float(pallet_center_x) - PALLET_DECK_SCALE[0] * 0.5
    pallet_max_x = float(pallet_center_x) + PALLET_DECK_SCALE[0] * 0.5
    pallet_min_y = float(pallet_center_y) - PALLET_DECK_SCALE[1] * 0.5
    pallet_max_y = float(pallet_center_y) + PALLET_DECK_SCALE[1] * 0.5

    min_margin = float("inf")
    for _name, part_offset, scale in specs:
        center = np.array([pallet_center_x, pallet_center_y, PALLET_CENTER_Z], dtype=float) + part_offset
        part_min_x = center[0] - scale[0] * 0.5
        part_max_x = center[0] + scale[0] * 0.5
        part_min_y = center[1] - scale[1] * 0.5
        part_max_y = center[1] + scale[1] * 0.5
        min_margin = min(
            min_margin,
            float(part_min_x - pallet_min_x),
            float(pallet_max_x - part_max_x),
            float(part_min_y - pallet_min_y),
            float(pallet_max_y - part_max_y),
        )

    return {
        "load_restraint_part_count": len(specs),
        "min_load_restraint_pallet_margin": float(min_margin),
        "max_load_restraint_pallet_overhang": float(max(0.0, -min_margin)),
    }


def compute_infeed_conveyor_metrics():
    carton_bottom_z = float(PICK_STATION_BIN_POSITION[2] - CARTON_BODY_SCALE[2] * 0.5)
    motion_marker_spacing = INFEED_CONVEYOR_LENGTH / max(INFEED_MOTION_MARKER_COUNT, 1)
    feed_carton_stop_clearance = INFEED_FEED_CARTON_MIN_Y - (INFEED_STOP_LINE_Y + CARTON_BODY_SCALE[1] * 0.5)
    return {
        "infeed_conveyor_length": float(INFEED_CONVEYOR_LENGTH),
        "infeed_spawn_margin": float(INFEED_CONVEYOR_END_Y - CONVEYOR_PICK_WINDOW_Y),
        "infeed_pick_margin": float(INFEED_STOP_LINE_Y - INFEED_CONVEYOR_START_Y),
        "infeed_guide_clearance": float(INFEED_GUIDE_INNER_WIDTH - CARTON_BODY_SCALE[0]),
        "infeed_belt_support_gap": float(carton_bottom_z - INFEED_CONVEYOR_TOP_Z),
        "infeed_motion_marker_count": int(INFEED_MOTION_MARKER_COUNT),
        "infeed_motion_marker_spacing": float(motion_marker_spacing),
        "infeed_motion_marker_speed": float(INFEED_MOTION_MARKER_SPEED),
        "infeed_feed_carton_count": int(INFEED_FEED_CARTON_COUNT),
        "infeed_feed_carton_path_length": float(INFEED_FEED_CARTON_PATH_LENGTH),
        "infeed_feed_carton_speed": float(INFEED_FEED_CARTON_SPEED),
        "infeed_feed_carton_stop_clearance": float(feed_carton_stop_clearance),
        "infeed_feed_carton_guide_clearance": float(INFEED_GUIDE_INNER_WIDTH - CARTON_BODY_SCALE[0]),
        "infeed_feed_carton_belt_support_gap": float(carton_bottom_z - INFEED_CONVEYOR_TOP_Z),
    }


def compute_active_bin_conveyor_metrics():
    carton_bottom_z = float(PICK_STATION_BIN_POSITION[2] - CARTON_BODY_SCALE[2] * 0.5)
    travel = float(np.linalg.norm(PICK_STATION_BIN_POSITION - ACTIVE_BIN_CONVEYOR_START_POSITION))
    return {
        "active_bin_conveyor_travel_distance": travel,
        "active_bin_conveyor_duration": float(ACTIVE_BIN_CONVEYOR_APPROACH_DURATION),
        "active_bin_conveyor_nominal_speed": float(travel / max(ACTIVE_BIN_CONVEYOR_APPROACH_DURATION, 1e-6)),
        "active_bin_conveyor_start_y": float(ACTIVE_BIN_CONVEYOR_START_POSITION[1]),
        "active_bin_conveyor_pick_y": float(PICK_STATION_BIN_POSITION[1]),
        "active_bin_conveyor_lateral_error": float(
            abs(ACTIVE_BIN_CONVEYOR_START_POSITION[0] - PICK_STATION_BIN_POSITION[0])
        ),
        "active_bin_conveyor_belt_support_gap": float(carton_bottom_z - INFEED_CONVEYOR_TOP_Z),
    }


def compute_active_bin_conveyor_approach_position(start_position, elapsed):
    start_position = np.array(start_position, dtype=float)
    t = smoothstep(float(elapsed) / max(ACTIVE_BIN_CONVEYOR_APPROACH_DURATION, 1e-6))
    return lerp(start_position, PICK_STATION_BIN_POSITION, t)


def compute_infeed_motion_marker_y(base_y, sim_time):
    distance_from_start = (float(base_y) - INFEED_CONVEYOR_START_Y) - INFEED_MOTION_MARKER_SPEED * float(sim_time)
    return float(INFEED_CONVEYOR_START_Y + (distance_from_start % INFEED_CONVEYOR_LENGTH))


def make_infeed_motion_marker_specs(sim_time=0.0):
    spacing = INFEED_CONVEYOR_LENGTH / max(INFEED_MOTION_MARKER_COUNT, 1)
    specs = []
    for marker_idx in range(INFEED_MOTION_MARKER_COUNT):
        base_y = INFEED_CONVEYOR_START_Y + spacing * (marker_idx + 0.5)
        marker_y = compute_infeed_motion_marker_y(base_y, sim_time)
        specs.append(
            (
                f"InfeedMotionMarker_{marker_idx}",
                np.array([0.0, marker_y, INFEED_MOTION_MARKER_Z], dtype=float),
                INFEED_MOTION_MARKER_SCALE.copy(),
                INFEED_MOTION_MARKER_COLOR.copy(),
                float(base_y),
            )
        )
    return specs


def compute_infeed_feed_carton_y(base_y, sim_time):
    distance_from_min = (float(base_y) - INFEED_FEED_CARTON_MIN_Y) - INFEED_FEED_CARTON_SPEED * float(sim_time)
    return float(INFEED_FEED_CARTON_MIN_Y + (distance_from_min % INFEED_FEED_CARTON_PATH_LENGTH))


def make_infeed_feed_carton_specs(sim_time=0.0):
    spacing = INFEED_FEED_CARTON_PATH_LENGTH / max(INFEED_FEED_CARTON_COUNT, 1)
    specs = []
    for carton_idx in range(INFEED_FEED_CARTON_COUNT):
        base_y = INFEED_FEED_CARTON_MIN_Y + spacing * (carton_idx + 0.5)
        carton_y = compute_infeed_feed_carton_y(base_y, sim_time)
        body_position = np.array([0.0, carton_y, PICK_STATION_BIN_POSITION[2]], dtype=float)
        tape_position = body_position + np.array([0.0, 0.0, INFEED_FEED_CARTON_TAPE_OFFSET_Z], dtype=float)
        specs.append(
            (
                f"InfeedFeedCarton_{carton_idx}",
                body_position,
                tape_position,
                CARTON_BODY_SCALE.copy(),
                CARTON_TAPE_TOP_SCALE.copy(),
                float(base_y),
            )
        )
    return specs


class InfeedConveyorMotionController:
    def __init__(self, marker_parts, marker_base_y_values):
        self.marker_parts = list(marker_parts)
        self.marker_base_y_values = [float(value) for value in marker_base_y_values]
        self.initial_positions = {}
        self.max_marker_observed_travel = 0.0
        self.update_count = 0

    def update(self, sim_time):
        self.update_count += 1
        for marker, base_y in zip(self.marker_parts, self.marker_base_y_values):
            marker_y = compute_infeed_motion_marker_y(base_y, sim_time)
            position = np.array([0.0, marker_y, INFEED_MOTION_MARKER_Z], dtype=float)
            marker_name = getattr(marker, "name", str(id(marker)))
            if marker_name not in self.initial_positions:
                self.initial_positions[marker_name] = position.copy()
            marker.set_world_pose(position=position)
            travel = float(np.linalg.norm(position - self.initial_positions[marker_name]))
            self.max_marker_observed_travel = max(self.max_marker_observed_travel, travel)


class InfeedFeedCartonMotionController:
    def __init__(self, carton_part_pairs, carton_base_y_values):
        self.carton_part_pairs = list(carton_part_pairs)
        self.carton_base_y_values = [float(value) for value in carton_base_y_values]
        self.initial_positions = {}
        self.max_carton_observed_travel = 0.0
        self.update_count = 0

    def update(self, sim_time):
        self.update_count += 1
        for (body_part, tape_part), base_y in zip(self.carton_part_pairs, self.carton_base_y_values):
            carton_y = compute_infeed_feed_carton_y(base_y, sim_time)
            body_position = np.array([0.0, carton_y, PICK_STATION_BIN_POSITION[2]], dtype=float)
            tape_position = body_position + np.array([0.0, 0.0, INFEED_FEED_CARTON_TAPE_OFFSET_Z], dtype=float)
            body_part.set_world_pose(position=body_position)
            tape_part.set_world_pose(position=tape_position)
            part_name = getattr(body_part, "name", str(id(body_part)))
            if part_name not in self.initial_positions:
                self.initial_positions[part_name] = body_position.copy()
            travel = float(np.linalg.norm(body_position - self.initial_positions[part_name]))
            self.max_carton_observed_travel = max(self.max_carton_observed_travel, travel)


class MotionContinuityTracker:
    def __init__(self):
        self.previous_positions = {}
        self.max_displacement_by_group = {}
        self.max_detail_by_group = {}
        self.sample_count_by_group = {}
        self.tracked_keys = set()

    def sample(self, group, item_id, position, phase=None, frame_index=None):
        group = str(group)
        item_id = str(item_id)
        key = (group, item_id)
        position = np.array(position, dtype=float)
        self.tracked_keys.add(key)
        self.max_displacement_by_group.setdefault(group, 0.0)
        if key in self.previous_positions:
            previous_position = self.previous_positions[key]
            displacement = float(np.linalg.norm(position - self.previous_positions[key]))
            if displacement > self.max_displacement_by_group[group]:
                self.max_displacement_by_group[group] = displacement
                self.max_detail_by_group[group] = {
                    "item_id": item_id,
                    "previous_position": previous_position.copy(),
                    "position": position.copy(),
                    "displacement": displacement,
                }
                if phase is not None:
                    self.max_detail_by_group[group]["phase"] = str(phase)
                if frame_index is not None:
                    self.max_detail_by_group[group]["frame_index"] = int(frame_index)
            self.sample_count_by_group[group] = int(self.sample_count_by_group.get(group, 0)) + 1
        self.previous_positions[key] = position.copy()

    def max_displacement(self, group):
        return float(self.max_displacement_by_group.get(str(group), 0.0))

    def sample_count(self, group):
        return int(self.sample_count_by_group.get(str(group), 0))

    def max_detail(self, group):
        return self.max_detail_by_group.get(str(group))

    def tracked_item_count(self, group=None):
        if group is None:
            return len(self.tracked_keys)
        group = str(group)
        return len([key for key in self.tracked_keys if key[0] == group])


def make_safety_fence_specs(pickup_y=DEFAULT_PICKUP_Y):
    specs = []
    post_color = np.array([0.95, 0.72, 0.12], dtype=float)
    rail_color = np.array([0.08, 0.09, 0.10], dtype=float)
    gate_color = np.array([0.92, 0.18, 0.12], dtype=float)
    post_center_z = WORLD_FLOOR_Z + SAFETY_FENCE_POST_SCALE[2] * 0.5

    def add_post(name, x, y, color=post_color):
        specs.append(
            (
                name,
                np.array([x, y, post_center_z], dtype=float),
                SAFETY_FENCE_POST_SCALE.copy(),
                color,
            )
        )

    def add_x_rail(name, x_min, x_max, y, z_offset, color=rail_color):
        length = float(x_max - x_min)
        if length <= 0.08:
            return
        specs.append(
            (
                name,
                np.array([(x_min + x_max) * 0.5, y, WORLD_FLOOR_Z + z_offset], dtype=float),
                np.array([length, SAFETY_FENCE_RAIL_THICKNESS, SAFETY_FENCE_RAIL_THICKNESS], dtype=float),
                color,
            )
        )

    def add_y_rail(name, x, y_min, y_max, z_offset, color=rail_color):
        length = float(y_max - y_min)
        if length <= 0.08:
            return
        specs.append(
            (
                name,
                np.array([x, (y_min + y_max) * 0.5, WORLD_FLOOR_Z + z_offset], dtype=float),
                np.array([SAFETY_FENCE_RAIL_THICKNESS, length, SAFETY_FENCE_RAIL_THICKNESS], dtype=float),
                color,
            )
        )

    amr_gate_min_y = float(pickup_y) - SAFETY_FENCE_AMR_GATE_WIDTH * 0.5
    amr_gate_max_y = float(pickup_y) + SAFETY_FENCE_AMR_GATE_WIDTH * 0.5
    infeed_gate_min_x = -SAFETY_FENCE_INFEED_GATE_WIDTH * 0.5
    infeed_gate_max_x = SAFETY_FENCE_INFEED_GATE_WIDTH * 0.5

    post_points = [
        ("SafetyFencePost_SW", SAFETY_FENCE_MIN_X, SAFETY_FENCE_MIN_Y),
        ("SafetyFencePost_SE", SAFETY_FENCE_MAX_X, SAFETY_FENCE_MIN_Y),
        ("SafetyFencePost_NW", SAFETY_FENCE_MIN_X, SAFETY_FENCE_MAX_Y),
        ("SafetyFencePost_NE", SAFETY_FENCE_MAX_X, SAFETY_FENCE_MAX_Y),
        ("SafetyFencePost_WGateLow", SAFETY_FENCE_MIN_X, amr_gate_min_y),
        ("SafetyFencePost_WGateHigh", SAFETY_FENCE_MIN_X, amr_gate_max_y),
        ("SafetyFencePost_EGateLow", SAFETY_FENCE_MAX_X, amr_gate_min_y),
        ("SafetyFencePost_EGateHigh", SAFETY_FENCE_MAX_X, amr_gate_max_y),
        ("SafetyFencePost_InfeedLeft", infeed_gate_min_x, SAFETY_FENCE_MAX_Y),
        ("SafetyFencePost_InfeedRight", infeed_gate_max_x, SAFETY_FENCE_MAX_Y),
    ]
    for name, x, y in post_points:
        add_post(name, x, y, gate_color if "Gate" in name or "Infeed" in name else post_color)

    for rail_idx, z_offset in enumerate(SAFETY_FENCE_RAIL_Z_OFFSETS):
        add_x_rail(
            f"SafetyFenceSouthRail_{rail_idx}",
            SAFETY_FENCE_MIN_X,
            SAFETY_FENCE_MAX_X,
            SAFETY_FENCE_MIN_Y,
            z_offset,
        )
        add_x_rail(
            f"SafetyFenceNorthRailLeft_{rail_idx}",
            SAFETY_FENCE_MIN_X,
            infeed_gate_min_x,
            SAFETY_FENCE_MAX_Y,
            z_offset,
        )
        add_x_rail(
            f"SafetyFenceNorthRailRight_{rail_idx}",
            infeed_gate_max_x,
            SAFETY_FENCE_MAX_X,
            SAFETY_FENCE_MAX_Y,
            z_offset,
        )
        for side_name, x in (("West", SAFETY_FENCE_MIN_X), ("East", SAFETY_FENCE_MAX_X)):
            add_y_rail(
                f"SafetyFence{side_name}RailLow_{rail_idx}",
                x,
                SAFETY_FENCE_MIN_Y,
                amr_gate_min_y,
                z_offset,
            )
            add_y_rail(
                f"SafetyFence{side_name}RailHigh_{rail_idx}",
                x,
                amr_gate_max_y,
                SAFETY_FENCE_MAX_Y,
                z_offset,
            )
    return specs


def compute_safety_fence_metrics(pickup_y=DEFAULT_PICKUP_Y):
    specs = make_safety_fence_specs(pickup_y)
    return {
        "safety_fence_part_count": len(specs),
        "safety_fence_amr_gate_clearance": compute_amr_cell_gate_clearance(pickup_y, pickup_y),
        "safety_fence_infeed_gate_clearance": float(
            SAFETY_FENCE_INFEED_GATE_WIDTH - SAFETY_FENCE_POST_SCALE[0] - INFEED_CONVEYOR_WIDTH
        ),
    }


def compute_amr_cell_gate_clearance(amr_y=DEFAULT_PICKUP_Y, pickup_y=DEFAULT_PICKUP_Y):
    centered_clearance = float(SAFETY_FENCE_AMR_GATE_WIDTH - SAFETY_FENCE_POST_SCALE[1] - PALLET_DECK_SCALE[1])
    lateral_error = abs(float(amr_y) - float(pickup_y))
    return float(centered_clearance - 2.0 * lateral_error)


def make_amr_safety_visual_specs():
    return [
        (
            name,
            shape,
            np.array(offset, dtype=float),
            size if isinstance(size, float) else np.array(size, dtype=float),
            np.array(color, dtype=float),
        )
        for name, shape, offset, size, color in AMR_SAFETY_VISUAL_SPECS
    ]


def get_amr_safety_visual_role(name):
    if name in AMR_WARNING_INDICATOR_NAMES:
        return "warning"
    if name in AMR_IDLE_INDICATOR_NAMES:
        return "idle"
    return "static"


def compute_amr_safety_visual_metrics():
    specs = make_amr_safety_visual_specs()
    beacon_tops = []
    scanner_bottoms = []
    warning_indicator_count = 0
    idle_indicator_count = 0
    for name, shape, offset, size, _color in specs:
        if shape == "sphere":
            part_top = float(offset[2]) + float(size)
            part_bottom = float(offset[2]) - float(size)
        else:
            part_top = float(offset[2]) + float(size[2]) * 0.5
            part_bottom = float(offset[2]) - float(size[2]) * 0.5
        role = get_amr_safety_visual_role(name)
        if role == "warning":
            warning_indicator_count += 1
        elif role == "idle":
            idle_indicator_count += 1
        if "Beacon" in name:
            beacon_tops.append(part_top)
        if "Scanner" in name:
            scanner_bottoms.append(part_bottom)
    return {
        "amr_safety_part_count": len(specs),
        "amr_safety_beacon_height": float(max(beacon_tops)) if beacon_tops else 0.0,
        "amr_safety_scanner_clearance": float(min(scanner_bottoms)) if scanner_bottoms else 0.0,
        "amr_warning_indicator_count": warning_indicator_count,
        "amr_idle_indicator_count": idle_indicator_count,
    }


def make_amr_drive_visual_specs():
    return [
        (
            name,
            np.array(offset, dtype=float),
            np.array(scale, dtype=float),
            np.array(color, dtype=float),
        )
        for name, offset, scale, color in AMR_DRIVE_VISUAL_SPECS
    ]


def compute_amr_drive_visual_metrics():
    specs = make_amr_drive_visual_specs()
    wheel_specs = [(name, offset, scale) for name, offset, scale, _color in specs if "Wheel" in name]
    wheel_bottoms = [float(offset[2] - scale[2] * 0.5) for _name, offset, scale in wheel_specs]
    wheel_x = [float(offset[0]) for _name, offset, _scale in wheel_specs]
    side_wheel_y = [float(offset[1]) for name, offset, _scale in wheel_specs if "Left" in name or "Right" in name]
    min_bottom = min(wheel_bottoms) if wheel_bottoms else 0.0
    max_bottom = max(wheel_bottoms) if wheel_bottoms else 0.0
    return {
        "amr_drive_part_count": len(specs),
        "amr_wheel_count": len(wheel_specs),
        "amr_wheel_floor_gap": max(0.0, min_bottom),
        "amr_wheel_floor_penetration": max(0.0, -min_bottom),
        "amr_wheel_max_floor_gap": max(0.0, max_bottom),
        "amr_drive_wheelbase": float(max(wheel_x) - min(wheel_x)) if wheel_x else 0.0,
        "amr_drive_track_width": float(max(side_wheel_y) - min(side_wheel_y)) if side_wheel_y else 0.0,
    }


def make_amr_lift_guide_visual_specs():
    return [
        (
            name,
            np.array(offset, dtype=float),
            np.array(scale, dtype=float),
            np.array(color, dtype=float),
        )
        for name, offset, scale, color in AMR_LIFT_GUIDE_VISUAL_SPECS
    ]


def compute_amr_lift_guide_visual_metrics():
    specs = make_amr_lift_guide_visual_specs()
    bottoms = [float(offset[2] - scale[2] * 0.5) for _name, offset, scale, _color in specs]
    tops = [float(offset[2] + scale[2] * 0.5) for _name, offset, scale, _color in specs]
    heights = [float(scale[2]) for _name, _offset, scale, _color in specs]
    min_bottom = min(bottoms) if bottoms else 0.0
    max_bottom = max(bottoms) if bottoms else 0.0
    min_top = min(tops) if tops else 0.0
    return {
        "amr_lift_guide_count": len(specs),
        "amr_lift_guide_bottom_gap": max(0.0, max_bottom),
        "amr_lift_guide_bottom_penetration": max(0.0, -min_bottom),
        "amr_lift_guide_travel_cover": float(min_top - AMR_LIFT_GUIDE_MAX_FORK_BOTTOM_Z) if tops else 0.0,
        "amr_lift_guide_min_height": min(heights) if heights else 0.0,
    }


def make_camera_rig_specs(
    pickup_x=DEFAULT_PICKUP_X,
    pickup_y=DEFAULT_PICKUP_Y,
    drop_x=DEFAULT_DROP_X,
    drop_y=DEFAULT_DROP_Y,
):
    route_mid_x = (float(pickup_x) + float(drop_x)) * 0.5
    route_mid_y = (float(pickup_y) + float(drop_y)) * 0.5
    return [
        (
            "OverviewCamera",
            "overview",
            np.array([route_mid_x, route_mid_y - 7.0, WORLD_FLOOR_Z + 5.0], dtype=float),
            np.array([route_mid_x, route_mid_y, WORLD_FLOOR_Z + 0.45], dtype=float),
            24.0,
        ),
        (
            "PalletizerCamera",
            "palletizer",
            np.array([float(pickup_x) - 1.65, float(pickup_y) - 2.15, WORLD_FLOOR_Z + 2.35], dtype=float),
            np.array([float(pickup_x), float(pickup_y), PALLET_CENTER_Z + 0.28], dtype=float),
            35.0,
        ),
        (
            "AmrRouteCamera",
            "amr_route",
            np.array([route_mid_x, route_mid_y - 3.80, WORLD_FLOOR_Z + 2.75], dtype=float),
            np.array([route_mid_x, route_mid_y, WORLD_FLOOR_Z + 0.35], dtype=float),
            CAMERA_DEFAULT_FOCAL_LENGTH,
        ),
        (
            "DropDockCamera",
            "drop_dock",
            np.array([float(drop_x) + 1.85, float(drop_y) - 2.15, WORLD_FLOOR_Z + 2.35], dtype=float),
            np.array([float(drop_x), float(drop_y), PALLET_CENTER_Z + 0.18], dtype=float),
            35.0,
        ),
    ]


def compute_camera_rig_metrics(
    pickup_x=DEFAULT_PICKUP_X,
    pickup_y=DEFAULT_PICKUP_Y,
    drop_x=DEFAULT_DROP_X,
    drop_y=DEFAULT_DROP_Y,
):
    specs = make_camera_rig_specs(pickup_x, pickup_y, drop_x, drop_y)
    roles = {role for _name, role, _eye, _target, _focal_length in specs}
    heights = [float(eye[2] - WORLD_FLOOR_Z) for _name, _role, eye, _target, _focal_length in specs]
    target_distances = [
        float(np.linalg.norm(np.array(eye, dtype=float) - np.array(target, dtype=float)))
        for _name, _role, eye, target, _focal_length in specs
    ]
    return {
        "camera_rig_count": len(specs),
        "camera_required_role_count": len(roles.intersection(CAMERA_RIG_REQUIRED_ROLES)),
        "camera_min_height": min(heights) if heights else 0.0,
        "camera_min_target_distance": min(target_distances) if target_distances else 0.0,
    }


def make_warehouse_light_specs(
    pickup_x=DEFAULT_PICKUP_X,
    pickup_y=DEFAULT_PICKUP_Y,
    drop_x=DEFAULT_DROP_X,
    drop_y=DEFAULT_DROP_Y,
):
    pickup_x = float(pickup_x)
    pickup_y = float(pickup_y)
    drop_x = float(drop_x)
    drop_y = float(drop_y)
    route_y = (pickup_y + drop_y) * 0.5 - 0.72
    light_z = WORLD_FLOOR_Z + 4.35
    route_a_x = pickup_x + (drop_x - pickup_x) * 0.33
    route_b_x = pickup_x + (drop_x - pickup_x) * 0.66
    return [
        (
            "PalletizerHighBayLight",
            "palletizer_key",
            np.array([pickup_x, pickup_y - 0.82, light_z], dtype=float),
            np.array([1.20, 0.18, 0.055], dtype=float),
            4200.0,
        ),
        (
            "RouteHighBayLightA",
            "route_fill",
            np.array([route_a_x, route_y, light_z], dtype=float),
            np.array([1.35, 0.18, 0.055], dtype=float),
            3600.0,
        ),
        (
            "RouteHighBayLightB",
            "route_fill",
            np.array([route_b_x, route_y, light_z], dtype=float),
            np.array([1.35, 0.18, 0.055], dtype=float),
            3600.0,
        ),
        (
            "DropDockHighBayLight",
            "drop_key",
            np.array([drop_x, drop_y - 0.82, light_z], dtype=float),
            np.array([1.20, 0.18, 0.055], dtype=float),
            4200.0,
        ),
    ]


def compute_warehouse_lighting_metrics(
    pickup_x=DEFAULT_PICKUP_X,
    pickup_y=DEFAULT_PICKUP_Y,
    drop_x=DEFAULT_DROP_X,
    drop_y=DEFAULT_DROP_Y,
):
    specs = make_warehouse_light_specs(pickup_x, pickup_y, drop_x, drop_y)
    roles = {role for _name, role, _position, _fixture_scale, _intensity in specs}
    x_positions = [float(position[0]) for _name, _role, position, _fixture_scale, _intensity in specs]
    heights = [float(position[2] - WORLD_FLOOR_Z) for _name, _role, position, _fixture_scale, _intensity in specs]
    intensities = [float(intensity) for _name, _role, _position, _fixture_scale, intensity in specs]
    return {
        "warehouse_light_count": len(specs),
        "warehouse_light_role_count": len(roles.intersection(WAREHOUSE_LIGHT_REQUIRED_ROLES)),
        "warehouse_light_min_height": min(heights) if heights else 0.0,
        "warehouse_light_route_span": (max(x_positions) - min(x_positions)) if x_positions else 0.0,
        "warehouse_light_min_intensity": min(intensities) if intensities else 0.0,
    }


def make_amr_route_guard_specs(
    pickup_x=DEFAULT_PICKUP_X,
    pickup_y=DEFAULT_PICKUP_Y,
    drop_x=DEFAULT_DROP_X,
    drop_y=DEFAULT_DROP_Y,
):
    route_min_x = min(float(pickup_x), float(drop_x))
    route_max_x = max(float(pickup_x), float(drop_x))
    guard_start_x = route_min_x + AMR_ROUTE_GUARD_X_MARGIN_START
    guard_end_x = route_max_x - AMR_ROUTE_GUARD_X_MARGIN_END
    if guard_end_x <= guard_start_x:
        guard_start_x = route_min_x
        guard_end_x = route_max_x
    path_center_y = (float(pickup_y) + float(drop_y)) * 0.5
    bollard_center_z = WORLD_FLOOR_Z + AMR_ROUTE_GUARD_BOLLARD_SCALE[2] * 0.5
    rail_center_z = WORLD_FLOOR_Z + 0.58
    x_positions = np.linspace(
        guard_start_x,
        guard_end_x,
        max(2, AMR_ROUTE_GUARD_BOLLARD_COUNT_PER_SIDE),
    )
    specs = []
    for side_name, side_sign in (("Left", -1.0), ("Right", 1.0)):
        guard_y = path_center_y + side_sign * AMR_ROUTE_GUARD_Y_OFFSET
        for bollard_idx, x_position in enumerate(x_positions):
            specs.append(
                (
                    f"AmrRoute{side_name}Bollard_{bollard_idx}",
                    "bollard",
                    np.array([x_position, guard_y, bollard_center_z], dtype=float),
                    AMR_ROUTE_GUARD_BOLLARD_SCALE.copy(),
                    AMR_ROUTE_GUARD_BOLLARD_COLOR,
                )
            )
        specs.append(
            (
                f"AmrRoute{side_name}GuardRail",
                "rail",
                np.array([(guard_start_x + guard_end_x) * 0.5, guard_y, rail_center_z], dtype=float),
                np.array(
                    [
                        guard_end_x - guard_start_x,
                        AMR_ROUTE_GUARD_RAIL_SCALE_YZ[0],
                        AMR_ROUTE_GUARD_RAIL_SCALE_YZ[1],
                    ],
                    dtype=float,
                ),
                AMR_ROUTE_GUARD_RAIL_COLOR,
            )
        )
    return specs


def compute_amr_route_guard_metrics(
    pickup_x=DEFAULT_PICKUP_X,
    pickup_y=DEFAULT_PICKUP_Y,
    drop_x=DEFAULT_DROP_X,
    drop_y=DEFAULT_DROP_Y,
):
    specs = make_amr_route_guard_specs(pickup_x, pickup_y, drop_x, drop_y)
    if not specs:
        return {
            "amr_route_guard_part_count": 0,
            "amr_route_guard_span": 0.0,
            "amr_route_guard_clearance": 0.0,
            "amr_route_bollard_height": 0.0,
        }
    min_x = min(float(position[0] - scale[0] * 0.5) for _name, _role, position, scale, _color in specs)
    max_x = max(float(position[0] + scale[0] * 0.5) for _name, _role, position, scale, _color in specs)
    path_center_y = (float(pickup_y) + float(drop_y)) * 0.5
    side_clearances = [
        abs(float(position[1]) - path_center_y) - float(scale[1]) * 0.5 - PALLET_DECK_SCALE[1] * 0.5
        for _name, _role, position, scale, _color in specs
    ]
    bollard_heights = [
        float(position[2] + scale[2] * 0.5 - WORLD_FLOOR_Z)
        for _name, role, position, scale, _color in specs
        if role == "bollard"
    ]
    return {
        "amr_route_guard_part_count": len(specs),
        "amr_route_guard_span": float(max_x - min_x),
        "amr_route_guard_clearance": float(min(side_clearances)) if side_clearances else 0.0,
        "amr_route_bollard_height": float(min(bollard_heights)) if bollard_heights else 0.0,
    }


def compute_loaded_route_y_error(
    amr_x,
    amr_y,
    pickup_x=DEFAULT_PICKUP_X,
    pickup_y=DEFAULT_PICKUP_Y,
    drop_x=DEFAULT_DROP_X,
    drop_y=DEFAULT_DROP_Y,
):
    pickup_x = float(pickup_x)
    drop_x = float(drop_x)
    if abs(drop_x - pickup_x) <= 1e-6:
        planned_y = float(pickup_y)
    else:
        route_t = clamp((float(amr_x) - pickup_x) / (drop_x - pickup_x), 0.0, 1.0)
        planned_y = lerp(float(pickup_y), float(drop_y), route_t)
    return abs(float(amr_y) - planned_y)


def compute_loaded_route_guard_clearance(amr_y, pickup_y=DEFAULT_PICKUP_Y, drop_y=DEFAULT_DROP_Y):
    path_center_y = (float(pickup_y) + float(drop_y)) * 0.5
    lateral_error = abs(float(amr_y) - path_center_y)
    guard_half_width = max(
        float(AMR_ROUTE_GUARD_BOLLARD_SCALE[1]) * 0.5,
        float(AMR_ROUTE_GUARD_RAIL_SCALE_YZ[0]) * 0.5,
    )
    return float(AMR_ROUTE_GUARD_Y_OFFSET - guard_half_width - PALLET_DECK_SCALE[1] * 0.5 - lateral_error)


def compute_arm_tcp_amr_route_clearance(
    arm_position,
    pickup_x=DEFAULT_PICKUP_X,
    pickup_y=DEFAULT_PICKUP_Y,
    drop_x=DEFAULT_DROP_X,
    drop_y=DEFAULT_DROP_Y,
):
    point = np.array(arm_position, dtype=float)[:2]
    start = np.array([float(pickup_x), float(pickup_y)], dtype=float)
    end = np.array([float(drop_x), float(drop_y)], dtype=float)
    route_vector = end - start
    route_length_sq = float(np.dot(route_vector, route_vector))
    if route_length_sq <= 1e-9:
        closest = start
    else:
        t = clamp(float(np.dot(point - start, route_vector) / route_length_sq), 0.0, 1.0)
        closest = start + t * route_vector
    loaded_half_width = float(PALLET_DECK_SCALE[1]) * 0.5
    return float(np.linalg.norm(point - closest) - loaded_half_width)


def compute_drop_docking_metrics(pickup_x=DEFAULT_PICKUP_X, drop_x=DEFAULT_DROP_X):
    travel_direction = 1.0 if float(drop_x) >= float(pickup_x) else -1.0
    drop_approach_x = float(drop_x) - travel_direction * AMR_DROP_APPROACH_STANDOFF
    return {
        "drop_approach_standoff": abs(float(drop_x) - drop_approach_x),
        "drop_approach_x": drop_approach_x,
        "drop_travel_direction": travel_direction,
        "dock_move_speed_scale": float(AMR_DOCK_MOVE_SPEED_SCALE),
    }


def compute_lift_contact_gap(amr_z=DEFAULT_AMR_Z, lift_offset=0.0):
    pallet_underside_z = PALLET_DECK_UNDERSIDE_Z + float(lift_offset)
    lift_plate_top_z = float(amr_z) + AMR_LIFT_PLATE_OFFSET_Z + float(lift_offset) + LIFT_FORK_SCALE[2] * 0.5
    return float(pallet_underside_z - lift_plate_top_z)


def compute_lift_fork_outer_half_width():
    return float(max(abs(offset[1]) + LIFT_FORK_SCALE[1] * 0.5 for offset in LIFT_FORK_OFFSETS))


def compute_lift_fork_inner_gap():
    y_offsets = sorted(float(offset[1]) for offset in LIFT_FORK_OFFSETS)
    if len(y_offsets) < 2:
        return 0.0
    return float(min(y_offsets[index + 1] - y_offsets[index] for index in range(len(y_offsets) - 1)) - LIFT_FORK_SCALE[1])


def compute_pallet_tunnel_clearance():
    return float(PALLET_TUNNEL_HALF_WIDTH - compute_lift_fork_outer_half_width())


def compute_drop_workstation_lane_outer_half_width():
    lane_half_width = max(
        DROP_SLIDE_RAIL_SCALE[1],
        DROP_SLIDE_ROLLER_SCALE[1],
        DROP_SLIDE_TOP_SUPPORT_SCALE[1],
    ) * 0.5
    return float(max(abs(y_offset) + lane_half_width for y_offset in DROP_SLIDE_LANE_Y_OFFSETS))


def compute_drop_workstation_support_gap():
    return float(PALLET_DECK_UNDERSIDE_Z - DROP_SLIDE_SUPPORT_TOP_Z)


def compute_drop_workstation_tunnel_clearance():
    return float(PALLET_TUNNEL_HALF_WIDTH - compute_drop_workstation_lane_outer_half_width())


def compute_drop_workstation_runner_clearance():
    runner_inner_half_width = min(
        abs(float(offset[1])) - PALLET_RUNNER_SCALE[1] * 0.5 for offset in PALLET_RUNNER_OFFSETS
    )
    return float(runner_inner_half_width - compute_drop_workstation_lane_outer_half_width())


def compute_drop_workstation_fork_clearance():
    lane_half_width = max(
        DROP_SLIDE_RAIL_SCALE[1],
        DROP_SLIDE_ROLLER_SCALE[1],
        DROP_SLIDE_TOP_SUPPORT_SCALE[1],
    ) * 0.5
    fork_half_width = LIFT_FORK_SCALE[1] * 0.5
    gaps = []
    for lane_y in DROP_SLIDE_LANE_Y_OFFSETS:
        for fork_offset in LIFT_FORK_OFFSETS:
            gaps.append(abs(float(lane_y) - float(fork_offset[1])) - lane_half_width - fork_half_width)
    return float(min(gaps)) if gaps else 0.0


def compute_drop_dock_metrics():
    stop_inner_clearances_to_forks = []
    stop_inner_clearances_to_runners = []
    for stop_y in DROP_DOCK_STOP_Y_OFFSETS:
        stop_inner_half_width = abs(float(stop_y)) - DROP_DOCK_STOP_BLOCK_SCALE[1] * 0.5
        for fork_offset in LIFT_FORK_OFFSETS:
            fork_outer_half_width = abs(float(fork_offset[1])) + LIFT_FORK_SCALE[1] * 0.5
            stop_inner_clearances_to_forks.append(stop_inner_half_width - fork_outer_half_width)
        runner_inner_half_width = min(
            abs(float(offset[1])) - PALLET_RUNNER_SCALE[1] * 0.5 for offset in PALLET_RUNNER_OFFSETS
        )
        stop_inner_clearances_to_runners.append(runner_inner_half_width - (abs(float(stop_y)) + DROP_DOCK_STOP_BLOCK_SCALE[1] * 0.5))

    guide_side_clearances = [
        abs(float(y_offset)) - DROP_DOCK_GUIDE_POST_SCALE[1] * 0.5 - PALLET_DECK_SCALE[1] * 0.5
        for y_offset in DROP_DOCK_GUIDE_POST_Y_OFFSETS
    ]
    return {
        "drop_dock_stop_count": len(DROP_DOCK_STOP_Y_OFFSETS),
        "drop_dock_stop_gap": float(DROP_DOCK_STOP_GAP),
        "drop_dock_guide_clearance": float(min(guide_side_clearances)) if guide_side_clearances else 0.0,
        "drop_dock_fork_clearance": (
            float(min(stop_inner_clearances_to_forks)) if stop_inner_clearances_to_forks else 0.0
        ),
        "drop_dock_runner_clearance": (
            float(min(stop_inner_clearances_to_runners)) if stop_inner_clearances_to_runners else 0.0
        ),
    }


def make_pickup_dock_alignment_specs(
    pickup_x=DEFAULT_PICKUP_X,
    pickup_y=DEFAULT_PICKUP_Y,
):
    stop_color = np.array([0.92, 0.58, 0.08], dtype=float)
    post_color = np.array([0.08, 0.09, 0.10], dtype=float)
    cap_color = np.array([0.95, 0.74, 0.12], dtype=float)
    dock_parts = []

    for stop_idx, y_offset in enumerate(PICKUP_DOCK_STOP_Y_OFFSETS):
        dock_parts.append(
            (
                f"PickupDockStopBlock_{stop_idx}",
                "stop",
                np.array(
                    [
                        pickup_x + PICKUP_DOCK_STOP_X_OFFSET,
                        pickup_y + y_offset,
                        PICKUP_DOCK_STOP_CENTER_Z,
                    ],
                    dtype=float,
                ),
                PICKUP_DOCK_STOP_BLOCK_SCALE.copy(),
                stop_color,
            )
        )
    for post_idx, x_offset in enumerate(PICKUP_DOCK_GUIDE_POST_X_OFFSETS):
        for side_idx, y_offset in enumerate(PICKUP_DOCK_GUIDE_POST_Y_OFFSETS):
            dock_parts.append(
                (
                    f"PickupDockLocatorPost_{post_idx}_{side_idx}",
                    "post",
                    np.array(
                        [
                            pickup_x + x_offset,
                            pickup_y + y_offset,
                            PICKUP_DOCK_GUIDE_POST_CENTER_Z,
                        ],
                        dtype=float,
                    ),
                    PICKUP_DOCK_GUIDE_POST_SCALE.copy(),
                    post_color,
                )
            )
            dock_parts.append(
                (
                    f"PickupDockLocatorCap_{post_idx}_{side_idx}",
                    "cap",
                    np.array(
                        [
                            pickup_x + x_offset,
                            pickup_y + y_offset,
                            PICKUP_DOCK_GUIDE_POST_CENTER_Z
                            + PICKUP_DOCK_GUIDE_POST_SCALE[2] * 0.5
                            + 0.025,
                        ],
                        dtype=float,
                    ),
                    np.array([0.10, 0.10, 0.05], dtype=float),
                    cap_color,
                )
            )
    return dock_parts


def compute_pickup_dock_metrics():
    stop_inner_clearances_to_forks = []
    stop_inner_clearances_to_runners = []
    for stop_y in PICKUP_DOCK_STOP_Y_OFFSETS:
        stop_inner_half_width = abs(float(stop_y)) - PICKUP_DOCK_STOP_BLOCK_SCALE[1] * 0.5
        for fork_offset in LIFT_FORK_OFFSETS:
            fork_outer_half_width = abs(float(fork_offset[1])) + LIFT_FORK_SCALE[1] * 0.5
            stop_inner_clearances_to_forks.append(stop_inner_half_width - fork_outer_half_width)
        runner_inner_half_width = min(
            abs(float(offset[1])) - PALLET_RUNNER_SCALE[1] * 0.5 for offset in PALLET_RUNNER_OFFSETS
        )
        stop_inner_clearances_to_runners.append(
            runner_inner_half_width - (abs(float(stop_y)) + PICKUP_DOCK_STOP_BLOCK_SCALE[1] * 0.5)
        )

    guide_side_clearances = [
        abs(float(y_offset)) - PICKUP_DOCK_GUIDE_POST_SCALE[1] * 0.5 - PALLET_DECK_SCALE[1] * 0.5
        for y_offset in PICKUP_DOCK_GUIDE_POST_Y_OFFSETS
    ]
    return {
        "pickup_dock_stop_count": len(PICKUP_DOCK_STOP_Y_OFFSETS),
        "pickup_dock_stop_gap": float(PICKUP_DOCK_STOP_GAP),
        "pickup_dock_guide_clearance": float(min(guide_side_clearances)) if guide_side_clearances else 0.0,
        "pickup_dock_fork_clearance": (
            float(min(stop_inner_clearances_to_forks)) if stop_inner_clearances_to_forks else 0.0
        ),
        "pickup_dock_runner_clearance": (
            float(min(stop_inner_clearances_to_runners)) if stop_inner_clearances_to_runners else 0.0
        ),
    }


def compute_amr_exit_clearance(amr_x, drop_x=DEFAULT_DROP_X):
    fork_rear_x = float(amr_x) - LIFT_FORK_SCALE[0] * 0.5
    dropped_pallet_front_x = float(drop_x) + PALLET_DECK_SCALE[0] * 0.5
    return float(fork_rear_x - dropped_pallet_front_x)


def clone_stack_coordinates(stack_coordinates):
    return [np.array(coord, dtype=float).copy() for coord in stack_coordinates]


def random_bin_spawn_transform():
    x = 0.0
    y = CONVEYOR_PICK_WINDOW_Y
    z = -0.15
    position = np.array([x, y, z], dtype=float)
    orientation = UPSIDE_DOWN_BIN_QUAT.copy()
    return position, orientation


def resolve_project_path(path_value):
    path = Path(path_value)
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


class DemoGifRecorder:
    def __init__(
        self,
        enabled=True,
        output_dir=DEFAULT_GIF_OUTPUT_DIR,
        frame_stride=GIF_FRAME_STRIDE,
        max_frames=GIF_MAX_FRAMES,
        frame_duration_ms=GIF_FRAME_DURATION_MS,
        canvas_size=GIF_CANVAS_SIZE,
    ):
        self.requested_enabled = bool(enabled)
        self.enabled = self.requested_enabled
        self.output_dir = resolve_project_path(output_dir)
        self.frame_stride = max(1, int(frame_stride))
        self.max_frames = max(1, int(max_frames))
        self.frame_duration_ms = max(20, int(frame_duration_ms))
        self.canvas_size = tuple(canvas_size)
        self.frames = []
        self.saved_path = None
        self.latest_path = None
        self.last_captured_frame = None
        self.disabled_reason = None

    def maybe_capture(self, frame_index, orchestrator, context, args, force=False):
        if not self.enabled:
            return
        if len(self.frames) >= self.max_frames and not force:
            return
        frame_index = int(frame_index)
        if not force and frame_index % self.frame_stride != 0:
            return
        if self.last_captured_frame == frame_index:
            return
        try:
            frame = self._draw_frame(frame_index, orchestrator, context, args)
            if len(self.frames) >= self.max_frames:
                self.frames[-1] = frame
            else:
                self.frames.append(frame)
            self.last_captured_frame = frame_index
        except Exception as exc:
            self.disabled_reason = f"capture skipped: {exc}"
            print(f"[HarimDemo] GIF {self.disabled_reason}", flush=True)
            self.enabled = False

    def save(self):
        if not self.requested_enabled:
            return None
        if self.saved_path is not None:
            return self.saved_path
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            output_path = self.output_dir / f"harim_amr_review_{stamp}_{os.getpid()}.gif"
            frames = list(self.frames)
            if not frames:
                frames = [self._draw_fallback_frame()]
            if len(frames) == 1:
                frames = [frames[0], frames[0].copy()]
            frames[0].save(
                output_path,
                save_all=True,
                append_images=frames[1:],
                duration=self.frame_duration_ms,
                loop=0,
                optimize=True,
            )
            self.saved_path = str(output_path)
            print(f"[HarimDemo] review GIF saved: {self.saved_path}", flush=True)
            try:
                latest_output_path = self.output_dir / LATEST_REVIEW_GIF_NAME
                shutil.copyfile(output_path, latest_output_path)
                self.latest_path = str(latest_output_path)
                print(f"[HarimDemo] latest review GIF updated: {self.latest_path}", flush=True)
            except Exception as exc:
                print(f"[HarimDemo] latest review GIF update failed: {exc}", flush=True)
            return self.saved_path
        except Exception as exc:
            print(f"[HarimDemo] review GIF save failed: {exc}", flush=True)
            self.enabled = False
            return None

    def _draw_fallback_frame(self):
        from PIL import Image, ImageDraw

        width, height = self.canvas_size
        image = Image.new("RGB", (width, height), (245, 247, 250))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, width, 52), fill=(24, 31, 40))
        draw.text((18, 17), "Harim AMR palletizing review", fill=(255, 255, 255))
        draw.text((18, 86), "Review GIF fallback frame", fill=(24, 31, 40))
        reason = self.disabled_reason or "No simulation review frames were captured before shutdown."
        draw.text((18, 124), reason, fill=(89, 99, 110))
        draw.text((18, 166), "The run still created a GIF so the latest execution has a visible artifact.", fill=(89, 99, 110))
        draw.rectangle((18, height - 88, width - 18, height - 36), outline=(154, 164, 178), width=2)
        draw.text((32, height - 70), "Check the matching log for the self-test status and metric output.", fill=(89, 99, 110))
        return image

    def _draw_frame(self, frame_index, orchestrator, context, args):
        from PIL import Image, ImageDraw

        width, height = self.canvas_size
        image = Image.new("RGB", (width, height), (245, 247, 250))
        draw = ImageDraw.Draw(image)
        map_rect, panel_rect = compute_review_gif_layout(self.canvas_size)
        self._draw_header(draw, frame_index, orchestrator, context, width)
        self._draw_map(draw, map_rect, orchestrator, context, args)
        self._draw_panel(draw, panel_rect, orchestrator, context)
        return image

    def _draw_header(self, draw, frame_index, orchestrator, context, width):
        state_name = getattr(getattr(orchestrator, "state", None), "name", "UNKNOWN")
        sim_time = float(getattr(context, "demo_sim_time", 0.0))
        draw.rectangle((0, 0, width, 44), fill=(24, 31, 40))
        draw.text((18, 12), "Harim AMR palletizing review", fill=(255, 255, 255))
        draw.text(
            (330, 12),
            f"frame {frame_index}  t={sim_time:6.2f}s  state={state_name}",
            fill=(226, 232, 240),
        )

    def _draw_map(self, draw, rect, orchestrator, context, args):
        left, top, right, bottom = rect
        draw.rectangle(rect, fill=(255, 255, 255), outline=(203, 213, 225))
        draw.text((left + 8, top + 8), "top view", fill=(51, 65, 85))

        x_min = float(args.pickup_x) - 1.4
        x_max = float(args.drop_x) + 2.2
        y_mid = (float(args.pickup_y) + float(args.drop_y)) * 0.5
        y_min = y_mid - 1.65
        y_max = y_mid + 1.65

        def to_px(position):
            x = float(position[0])
            y = float(position[1])
            px = left + 22 + (x - x_min) / max(x_max - x_min, 1e-6) * (right - left - 44)
            py = bottom - 24 - (y - y_min) / max(y_max - y_min, 1e-6) * (bottom - top - 58)
            return px, py

        def draw_world_rect(center, scale, fill, outline=(30, 41, 59)):
            cx, cy = to_px(center)
            sx = float(scale[0]) / max(x_max - x_min, 1e-6) * (right - left - 44)
            sy = float(scale[1]) / max(y_max - y_min, 1e-6) * (bottom - top - 58)
            draw.rectangle(
                (cx - sx * 0.5, cy - sy * 0.5, cx + sx * 0.5, cy + sy * 0.5),
                fill=fill,
                outline=outline,
            )

        pickup = np.array([args.pickup_x, args.pickup_y, 0.0], dtype=float)
        drop = np.array([args.drop_x, args.drop_y, 0.0], dtype=float)
        pickup_px = to_px(pickup)
        drop_px = to_px(drop)
        draw.line((pickup_px[0], pickup_px[1], drop_px[0], drop_px[1]), fill=(245, 158, 11), width=4)
        conveyor_center = np.array([0.0, INFEED_CONVEYOR_CENTER_Y, 0.0], dtype=float)
        draw_world_rect(conveyor_center, [INFEED_CONVEYOR_WIDTH, INFEED_CONVEYOR_LENGTH], (31, 41, 55), (15, 23, 42))
        sim_time = float(getattr(context, "demo_sim_time", 0.0))
        for _name, marker_position, marker_scale, _color, _base_y in make_infeed_motion_marker_specs(sim_time):
            draw_world_rect(marker_position, marker_scale, (148, 163, 184), (71, 85, 105))
        for _name, body_position, _tape_position, body_scale, _tape_scale, _base_y in make_infeed_feed_carton_specs(
            sim_time
        ):
            draw_world_rect(body_position, body_scale, (191, 120, 66), (124, 45, 18))
        draw_world_rect(pickup, [1.75, 1.35], (254, 243, 199), (217, 119, 6))
        draw_world_rect(drop, [1.95, 1.45], (219, 234, 254), (37, 99, 235))
        draw.text((pickup_px[0] - 28, pickup_px[1] - 48), "PICK", fill=(146, 64, 14))
        draw.text((drop_px[0] - 24, drop_px[1] - 48), "DROP", fill=(30, 64, 175))

        pallet_center = self._pallet_center(orchestrator, args)
        draw_world_rect(pallet_center, PALLET_DECK_SCALE, (180, 132, 84), (120, 53, 15))

        for bin_state in getattr(context, "stacked_bins", []):
            bin_obj = getattr(bin_state, "bin_obj", None)
            position = self._world_position(bin_obj)
            if position is not None:
                draw_world_rect(position, CARTON_BODY_SCALE, (191, 120, 66), (124, 45, 18))

        active_bin = (
            getattr(context, "demo_released_bin", None)
            or getattr(context, "demo_carried_bin", None)
            or getattr(context, "active_bin", None)
            or getattr(context, "demo_infeed_active_bin", None)
        )
        active_position = self._world_position(getattr(active_bin, "bin_obj", None))
        if active_position is not None and active_bin not in getattr(context, "stacked_bins", []):
            draw_world_rect(active_position, CARTON_BODY_SCALE, (239, 68, 68), (127, 29, 29))

        amr_position = np.array(orchestrator.get_amr_position(), dtype=float)
        draw_world_rect(amr_position, [1.0, 0.72], (59, 130, 246), (30, 64, 175))
        for name, offset, scale, _color in make_amr_drive_visual_specs():
            wheel_center = amr_position + np.array(offset, dtype=float)
            fill = (15, 23, 42) if "DriveWheel" in name else (51, 65, 85)
            draw_world_rect(wheel_center, [scale[0], scale[1]], fill, (15, 23, 42))
        amr_px = to_px(amr_position)
        draw.polygon(
            [(amr_px[0] + 16, amr_px[1]), (amr_px[0] + 2, amr_px[1] - 7), (amr_px[0] + 2, amr_px[1] + 7)],
            fill=(255, 255, 255),
        )

    def _draw_panel(self, draw, rect, orchestrator, context):
        left, top, right, bottom = rect
        draw.rectangle(rect, fill=(255, 255, 255), outline=(203, 213, 225))
        state_name = getattr(getattr(orchestrator, "state", None), "name", "UNKNOWN")
        motion_phase = self._short_panel_value(getattr(context, "demo_motion_phase", "-"), max_chars=24)
        stack_count = len(getattr(context, "stacked_bins", []))
        total_stack = len(getattr(context, "stack_coordinates", []))
        lift_offset = float(getattr(orchestrator, "lift_offset", 0.0))
        cycles = int(getattr(orchestrator, "completed_cycles", 0))
        grasp_gap = float(getattr(context, "demo_max_attached_grasp_error", 0.0))
        lines = [
            "status",
            f"state: {state_name}",
            f"phase: {motion_phase}",
            f"stack: {stack_count}/{total_stack}",
            f"cycles: {cycles}",
            f"lift: {lift_offset:.3f} m",
            f"grasp max: {grasp_gap:.3f} m",
        ]
        y = top + 12
        for idx, line in enumerate(lines):
            fill = (15, 23, 42) if idx == 0 else (51, 65, 85)
            draw.text((left + 12, y), line, fill=fill)
            y += 22

        release_bin = getattr(context, "demo_released_bin", None)
        release_position = self._world_position(getattr(release_bin, "bin_obj", None))
        arm_position = self._arm_position(context)
        clearance = 0.0
        if release_position is not None and arm_position is not None:
            clearance = float(arm_position[2] - release_position[2])
        y += 10
        draw.text((left + 12, y), "release clearance", fill=(15, 23, 42))
        y += 22
        draw.text((left + 12, y), f"vertical: {clearance:.3f} m", fill=(51, 65, 85))

        gauge_left = left + 30
        gauge_right = right - 30
        gauge_bottom = bottom - 34
        gauge_top = gauge_bottom - 155
        draw.rectangle(
            (gauge_left, gauge_top, gauge_right, gauge_bottom),
            fill=(248, 250, 252),
            outline=(226, 232, 240),
        )
        box_y = gauge_bottom - 32
        draw.rectangle(
            (gauge_left + 18, box_y - 10, gauge_right - 18, box_y + 10),
            fill=(191, 120, 66),
            outline=(124, 45, 18),
        )
        arm_y = max(gauge_top + 14, min(gauge_bottom - 20, box_y - clearance * 110.0))
        draw.line((gauge_left + 20, arm_y, gauge_right - 20, arm_y), fill=(239, 68, 68), width=4)
        draw.text((gauge_left + 18, arm_y - 18), "TCP", fill=(127, 29, 29))
        draw.text((gauge_left + 18, box_y + 14), "box", fill=(124, 45, 18))

    def _pallet_center(self, orchestrator, args):
        pallet_parts = getattr(orchestrator, "pallet_parts", [])
        if pallet_parts:
            position = self._world_position(pallet_parts[0])
            if position is not None:
                return np.array(position, dtype=float) - PALLET_DECK_OFFSETS[0]
        return np.array([args.pickup_x, args.pickup_y, PALLET_CENTER_Z], dtype=float)

    def _world_position(self, obj):
        if obj is None:
            return None
        try:
            position, _orientation = obj.get_world_pose()
            return np.array(position, dtype=float)
        except Exception:
            return None

    def _arm_position(self, context):
        try:
            return get_measured_arm_fk_p(context)
        except Exception:
            return None

    def _short_panel_value(self, value, max_chars=24):
        text = str(value)
        if len(text) <= max_chars:
            return text
        return f"{text[: max(0, max_chars - 1)]}~"


class HarimTransferOrchestrator:
    def __init__(
        self,
        *,
        world,
        context,
        task,
        amr_prim,
        lift_plate,
        pallet_parts,
        stack_coordinates,
        args,
        pallet_part_offsets=None,
        load_restraint_parts=None,
        lift_plate_parts=None,
        amr_lift_prim=None,
        amr_safety_parts=None,
        amr_safety_offsets=None,
        amr_safety_roles=None,
        amr_drive_parts=None,
        amr_drive_offsets=None,
        amr_lift_guide_parts=None,
        amr_lift_guide_offsets=None,
        completion_signal=None,
        camera_director=None,
    ):
        self.world = world
        self.context = context
        self.task = task
        self.amr = amr_prim
        self.amr_lift = amr_lift_prim
        self.lift_plate = lift_plate
        self.lift_plate_parts = list(lift_plate_parts) if lift_plate_parts is not None else [lift_plate]
        self.amr_safety_parts = list(amr_safety_parts) if amr_safety_parts is not None else []
        self.amr_safety_offsets = (
            tuple(np.array(offset, dtype=float) for offset in amr_safety_offsets)
            if amr_safety_offsets is not None
            else tuple()
        )
        self.amr_safety_roles = tuple(amr_safety_roles) if amr_safety_roles is not None else tuple()
        self.amr_drive_parts = list(amr_drive_parts) if amr_drive_parts is not None else []
        self.amr_drive_offsets = (
            tuple(np.array(offset, dtype=float) for offset in amr_drive_offsets)
            if amr_drive_offsets is not None
            else tuple()
        )
        self.amr_lift_guide_parts = list(amr_lift_guide_parts) if amr_lift_guide_parts is not None else []
        self.amr_lift_guide_offsets = (
            tuple(np.array(offset, dtype=float) for offset in amr_lift_guide_offsets)
            if amr_lift_guide_offsets is not None
            else tuple()
        )
        self.pallet_parts = pallet_parts
        self.pallet_part_offsets = (
            tuple(np.array(offset, dtype=float) for offset in pallet_part_offsets)
            if pallet_part_offsets is not None
            else PALLET_PART_OFFSETS
        )
        self.load_restraint_parts = list(load_restraint_parts) if load_restraint_parts is not None else []
        self.stack_coordinates = stack_coordinates
        self.args = args
        self.completion_signal = completion_signal
        self.camera_director = camera_director

        self.state = TransferState.WAIT_STACK_COMPLETE
        self.state_time = 0.0
        self.completed_cycles = 0
        self.carrying = False
        self.attached_items = []
        self.item_offsets = {}
        self.pallet_base_offsets = {}
        self.dropped_item_poses = {}
        self.dropped_pallet_poses = {}
        self.locked_stack_poses = {}
        self.payload_lift_baseline_poses = {}

        self.stack_center = self._compute_stack_center()
        self.amr_yaw = 0.0
        self.lift_offset = 0.0
        self.lift_offset_motion_sample_count = 0
        self.max_lift_offset_frame_step = 0.0
        self._previous_lift_offset_sample = None
        self.max_payload_lift_observed = 0.0
        self.max_dropped_payload_drift = 0.0
        initial_lift_contact_gap = compute_lift_contact_gap(args.amr_z, 0.0)
        self.max_lift_contact_gap_observed = initial_lift_contact_gap
        self.min_lift_contact_gap_observed = initial_lift_contact_gap
        self.max_amr_safety_pose_error = 0.0
        self.max_amr_orientation_error = 0.0
        self.max_amr_drive_pose_error = 0.0
        self.max_amr_lift_guide_pose_error = 0.0
        self.max_amr_lift_orientation_error = 0.0
        self.min_amr_cell_gate_clearance = compute_amr_cell_gate_clearance(args.pickup_y, args.pickup_y)
        self.max_loaded_route_y_error = 0.0
        self.min_loaded_route_guard_clearance = compute_loaded_route_guard_clearance(
            args.pickup_y,
            args.pickup_y,
            args.drop_y,
        )
        self.arm_tcp_amr_route_clearance_sample_count = 0
        self.min_arm_tcp_amr_route_clearance = float("inf")
        self.max_carried_pallet_pose_error = 0.0
        self.max_carried_pallet_orientation_error = 0.0
        self.max_carried_payload_pose_error = 0.0
        self.max_carried_payload_orientation_error = 0.0
        self.dropped_stack_item_count = 0
        self.max_dropped_stack_pose_error = 0.0
        self.max_dropped_stack_orientation_error = 0.0
        self.max_dropped_stack_support_gap = 0.0
        self.min_dropped_stack_support_gap = 0.0
        self.min_dropped_stack_pallet_margin = 0.0
        self.max_dropped_stack_pallet_overhang = 0.0
        self.dropped_pallet_part_count = 0
        self.max_dropped_pallet_part_pose_error = 0.0
        self.max_dropped_pallet_part_orientation_error = 0.0
        self.drop_handoff_xy_error = 0.0
        self.drop_handoff_support_gap = compute_drop_workstation_support_gap()
        self.drop_handoff_support_penetration = 0.0
        self.pickup_handoff_count = 0
        self.max_pickup_handoff_xy_error = 0.0
        self.max_pickup_handoff_lift_gap = initial_lift_contact_gap
        self.max_pickup_handoff_lift_penetration = 0.0
        self.pickup_entry_sample_count = 0
        self.max_pickup_entry_y_error = 0.0
        self.min_pickup_entry_tunnel_clearance = compute_pallet_tunnel_clearance()
        self.max_pickup_entry_lift_gap = initial_lift_contact_gap
        self.max_pickup_entry_lift_penetration = 0.0
        self.slide_out_sample_count = 0
        self.max_slide_out_y_error = 0.0
        self.max_slide_out_lift_gap = initial_lift_contact_gap
        self.max_slide_out_lift_penetration = 0.0
        self.amr_warning_indicator_on_observed = 0
        self.amr_idle_indicator_on_observed = 0
        self.amr_indicator_visibility_mismatch_count = 0
        self.camera_director_switch_count = 0
        self.camera_director_requested_roles = set()
        self.camera_director_last_role = None
        self.pallet_tunnel_clearance = compute_pallet_tunnel_clearance()
        self.lift_fork_inner_gap = compute_lift_fork_inner_gap()
        self.amr_lift_base_offset = None
        self.amr_lift_orientation = None
        self.move_target = None
        self.move_start_pose = None
        self.move_duration = 0.0
        self.drop_dock_arrival_count = 0
        self.drop_approach_final_error = 0.0
        self.drop_dock_final_error = 0.0

        self.travel_direction = 1.0 if float(args.drop_x) >= float(args.pickup_x) else -1.0
        self.start_pose = np.array([args.pickup_x + AMR_START_STANDOFF, args.pickup_y, args.amr_z], dtype=float)
        self.approach_pose = np.array([args.pickup_x + AMR_APPROACH_STANDOFF, args.pickup_y, args.amr_z], dtype=float)
        self.pickup_pose = np.array([args.pickup_x, args.pickup_y, args.amr_z], dtype=float)
        self.drop_approach_pose = np.array(
            [
                args.drop_x - self.travel_direction * AMR_DROP_APPROACH_STANDOFF,
                args.drop_y,
                args.amr_z,
            ],
            dtype=float,
        )
        self.drop_pose = np.array([args.drop_x, args.drop_y, args.amr_z], dtype=float)
        self.exit_pose = np.array(
            [args.drop_x + self.travel_direction * SLIDE_EXIT_DISTANCE, args.drop_y, args.amr_z],
            dtype=float,
        )

        self.reset_visual_state()

    def _compute_stack_center(self):
        coords = np.array(self.stack_coordinates)
        return np.array([float(coords[:, 0].mean()), float(coords[:, 1].mean()), float(coords[:, 2].min())], dtype=float)

    def _transition(self, state):
        self.state = state
        self.state_time = 0.0
        if state == TransferState.MOVE_TO_APPROACH:
            self.move_target = self.approach_pose
        elif state == TransferState.MOVE_UNDER_PALLET:
            self.move_target = self.pickup_pose
        elif state == TransferState.MOVE_TO_DROP_APPROACH:
            self.move_target = self.drop_approach_pose
        elif state == TransferState.MOVE_TO_DROP:
            self.move_target = self.drop_pose
        elif state == TransferState.SLIDE_OUT_FROM_PALLET:
            self.move_target = self.exit_pose
        else:
            self.move_target = None
        if self.move_target is not None:
            self.move_start_pose = self.get_amr_position()
            move_distance = float(np.linalg.norm((self.move_target - self.move_start_pose)[:2]))
            self.move_duration = self._compute_move_duration(move_distance)
        else:
            self.move_start_pose = None
            self.move_duration = 0.0
        print(f"[HarimDemo] state -> {state.name}")
        self._set_amr_indicator_visibility()
        self._request_camera_for_state()

    def reset_visual_state(self):
        self.carrying = False
        self.attached_items = []
        self.item_offsets = {}
        self.pallet_base_offsets = {}
        self.dropped_item_poses = {}
        self.dropped_pallet_poses = {}
        self.locked_stack_poses = {}
        self.payload_lift_baseline_poses = {}
        self.lift_offset = 0.0
        self._previous_lift_offset_sample = None
        self.set_amr_pose(self.start_pose)
        self._set_lift_plate_pose()
        self._reset_pallet_pose()
        self._set_load_restraint_visibility(False)
        self._set_amr_indicator_visibility()
        self._request_camera_for_state()
        if self.completion_signal is not None:
            self.completion_signal.set_completed(False)

    def _request_camera_for_state(self):
        role = camera_role_for_transfer_state(self.state)
        self.camera_director_requested_roles.add(role)
        if role == self.camera_director_last_role:
            return
        self.camera_director_last_role = role
        self.camera_director_switch_count += 1
        if self.camera_director is None:
            return
        try:
            self.camera_director(role)
        except Exception as exc:
            print(f"[HarimDemo] camera director skipped for role {role}: {exc}", flush=True)

    def _set_load_restraint_visibility(self, visible):
        for part in self.load_restraint_parts:
            set_visibility = getattr(part, "set_visibility", None)
            if set_visibility is not None:
                try:
                    set_visibility(bool(visible))
                except Exception:
                    pass

    def set_amr_pose(self, position):
        self.amr.set_world_pose(position=np.array(position, dtype=float), orientation=yaw_to_quat(self.amr_yaw))
        self._record_amr_orientation()
        self._set_amr_safety_visual_pose()
        self._set_amr_drive_visual_pose()
        self._set_amr_lift_guide_visual_pose()
        self._record_amr_cell_gate_clearance()

    def get_amr_position(self):
        position, _orientation = self.amr.get_world_pose()
        return np.array(position, dtype=float)

    def _record_amr_cell_gate_clearance(self):
        try:
            amr_pos = self.get_amr_position()
        except Exception:
            return
        self.min_amr_cell_gate_clearance = min(
            self.min_amr_cell_gate_clearance,
            compute_amr_cell_gate_clearance(amr_pos[1], self.args.pickup_y),
        )

    def _record_arm_tcp_route_clearance(self):
        if self.state in (TransferState.WAIT_STACK_COMPLETE, TransferState.DONE_IDLE):
            return
        try:
            arm_position = get_measured_arm_fk_p(self.context)
        except Exception:
            return
        clearance = compute_arm_tcp_amr_route_clearance(
            arm_position,
            self.args.pickup_x,
            self.args.pickup_y,
            self.args.drop_x,
            self.args.drop_y,
        )
        self.arm_tcp_amr_route_clearance_sample_count += 1
        self.min_arm_tcp_amr_route_clearance = min(self.min_arm_tcp_amr_route_clearance, clearance)

    def _record_amr_orientation(self):
        try:
            _position, orientation = self.amr.get_world_pose()
        except Exception:
            return
        self.max_amr_orientation_error = max(
            self.max_amr_orientation_error,
            quat_angular_error(orientation, yaw_to_quat(self.amr_yaw)),
        )

    def _lift_plate_position(self, fork_offset=None):
        amr_pos = self.get_amr_position()
        offset = np.array([0.0, 0.0, 0.0], dtype=float) if fork_offset is None else np.array(fork_offset, dtype=float)
        return amr_pos + offset + np.array([0.0, 0.0, AMR_LIFT_PLATE_OFFSET_Z + self.lift_offset], dtype=float)

    def _set_lift_plate_pose(self):
        if len(self.lift_plate_parts) == len(LIFT_FORK_OFFSETS):
            fork_offsets = LIFT_FORK_OFFSETS
        else:
            fork_offsets = tuple(np.array([0.0, 0.0, 0.0], dtype=float) for _ in self.lift_plate_parts)
        for part, fork_offset in zip(self.lift_plate_parts, fork_offsets):
            part.set_world_pose(position=self._lift_plate_position(fork_offset), orientation=yaw_to_quat(self.amr_yaw))
        self._set_actual_lift_pose()
        self._record_lift_orientation()
        self._record_lift_geometry()

    def _record_lift_orientation(self):
        expected_fork_orientation = yaw_to_quat(self.amr_yaw)
        for part in self.lift_plate_parts:
            try:
                _position, orientation = part.get_world_pose()
            except Exception:
                continue
            self.max_amr_lift_orientation_error = max(
                self.max_amr_lift_orientation_error,
                quat_angular_error(orientation, expected_fork_orientation),
            )
        if self.amr_lift is None or self.amr_lift_orientation is None:
            return
        try:
            _position, orientation = self.amr_lift.get_world_pose()
        except Exception:
            return
        self.max_amr_lift_orientation_error = max(
            self.max_amr_lift_orientation_error,
            quat_angular_error(orientation, self.amr_lift_orientation),
        )

    def _set_amr_safety_visual_pose(self):
        if not self.amr_safety_parts:
            return
        amr_pos = self.get_amr_position()
        if len(self.amr_safety_offsets) == len(self.amr_safety_parts):
            offsets = self.amr_safety_offsets
        else:
            offsets = tuple(np.array([0.0, 0.0, 0.0], dtype=float) for _ in self.amr_safety_parts)
        for part, offset in zip(self.amr_safety_parts, offsets):
            expected = amr_pos + np.array(offset, dtype=float)
            part.set_world_pose(position=expected, orientation=yaw_to_quat(self.amr_yaw))
            try:
                current, _ = part.get_world_pose()
                self.max_amr_safety_pose_error = max(
                    self.max_amr_safety_pose_error,
                    float(np.linalg.norm(np.array(current, dtype=float) - expected)),
                )
            except Exception:
                pass

    def _set_amr_drive_visual_pose(self):
        if not self.amr_drive_parts:
            return
        amr_pos = self.get_amr_position()
        if len(self.amr_drive_offsets) == len(self.amr_drive_parts):
            offsets = self.amr_drive_offsets
        else:
            offsets = tuple(np.array([0.0, 0.0, 0.0], dtype=float) for _ in self.amr_drive_parts)
        for part, offset in zip(self.amr_drive_parts, offsets):
            expected = amr_pos + np.array(offset, dtype=float)
            part.set_world_pose(position=expected, orientation=yaw_to_quat(self.amr_yaw))
            try:
                current, _ = part.get_world_pose()
                self.max_amr_drive_pose_error = max(
                    self.max_amr_drive_pose_error,
                    float(np.linalg.norm(np.array(current, dtype=float) - expected)),
                )
            except Exception:
                pass

    def _set_amr_lift_guide_visual_pose(self):
        if not self.amr_lift_guide_parts:
            return
        amr_pos = self.get_amr_position()
        if len(self.amr_lift_guide_offsets) == len(self.amr_lift_guide_parts):
            offsets = self.amr_lift_guide_offsets
        else:
            offsets = tuple(np.array([0.0, 0.0, 0.0], dtype=float) for _ in self.amr_lift_guide_parts)
        for part, offset in zip(self.amr_lift_guide_parts, offsets):
            expected = amr_pos + np.array(offset, dtype=float)
            part.set_world_pose(position=expected, orientation=yaw_to_quat(self.amr_yaw))
            try:
                current, _ = part.get_world_pose()
                self.max_amr_lift_guide_pose_error = max(
                    self.max_amr_lift_guide_pose_error,
                    float(np.linalg.norm(np.array(current, dtype=float) - expected)),
                )
            except Exception:
                pass

    def _set_amr_indicator_visibility(self):
        if not self.amr_safety_parts:
            return
        active_warning = self.state not in (TransferState.WAIT_STACK_COMPLETE, TransferState.DONE_IDLE)
        if len(self.amr_safety_roles) == len(self.amr_safety_parts):
            roles = self.amr_safety_roles
        else:
            roles = tuple("static" for _ in self.amr_safety_parts)
        for part, role in zip(self.amr_safety_parts, roles):
            if role == "warning":
                visible = active_warning
                if visible:
                    self.amr_warning_indicator_on_observed += 1
            elif role == "idle":
                visible = not active_warning
                if visible:
                    self.amr_idle_indicator_on_observed += 1
            else:
                visible = True
            set_visibility = getattr(part, "set_visibility", None)
            if set_visibility is None:
                continue
            try:
                set_visibility(bool(visible))
                if hasattr(part, "visible") and part.visible != bool(visible):
                    self.amr_indicator_visibility_mismatch_count += 1
            except Exception:
                self.amr_indicator_visibility_mismatch_count += 1

    def _record_lift_geometry(self):
        amr_pos = self.get_amr_position()
        lift_contact_gap = compute_lift_contact_gap(amr_pos[2], self.lift_offset)
        self.max_lift_contact_gap_observed = max(self.max_lift_contact_gap_observed, lift_contact_gap)
        self.min_lift_contact_gap_observed = min(self.min_lift_contact_gap_observed, lift_contact_gap)

    def _record_lift_offset_continuity(self):
        current_offset = float(self.lift_offset)
        if self._previous_lift_offset_sample is not None:
            frame_step = abs(current_offset - float(self._previous_lift_offset_sample))
            self.max_lift_offset_frame_step = max(self.max_lift_offset_frame_step, frame_step)
            self.lift_offset_motion_sample_count += 1
        self._previous_lift_offset_sample = current_offset

    def _record_pickup_handoff_geometry(self):
        if not self.pallet_parts:
            return
        try:
            deck_pos, _deck_orient = self.pallet_parts[0].get_world_pose()
        except Exception:
            return
        deck_pos = np.array(deck_pos, dtype=float)
        pallet_center = deck_pos - PALLET_DECK_OFFSETS[0]
        amr_pos = self.get_amr_position()
        xy_error = float(np.linalg.norm((amr_pos - pallet_center)[:2]))
        lift_top_zs = []
        for part in self.lift_plate_parts:
            try:
                lift_pos, _lift_orient = part.get_world_pose()
                lift_top_zs.append(float(np.array(lift_pos, dtype=float)[2] + LIFT_FORK_SCALE[2] * 0.5))
            except Exception:
                pass
        if lift_top_zs:
            lift_top_z = max(lift_top_zs)
        else:
            lift_top_z = float(amr_pos[2] + AMR_LIFT_PLATE_OFFSET_Z + self.lift_offset + LIFT_FORK_SCALE[2] * 0.5)
        deck_underside_z = float(deck_pos[2] - PALLET_DECK_SCALE[2] * 0.5)
        lift_gap = deck_underside_z - lift_top_z
        self.pickup_handoff_count += 1
        self.max_pickup_handoff_xy_error = max(self.max_pickup_handoff_xy_error, xy_error)
        self.max_pickup_handoff_lift_gap = max(self.max_pickup_handoff_lift_gap, float(lift_gap))
        self.max_pickup_handoff_lift_penetration = max(
            self.max_pickup_handoff_lift_penetration,
            max(0.0, float(-lift_gap)),
        )

    def _record_pickup_entry_geometry(self):
        if not self.pallet_parts:
            return
        try:
            deck_pos, _deck_orient = self.pallet_parts[0].get_world_pose()
        except Exception:
            return
        deck_pos = np.array(deck_pos, dtype=float)
        pallet_center = deck_pos - PALLET_DECK_OFFSETS[0]
        amr_pos = self.get_amr_position()
        y_error = abs(float(amr_pos[1] - pallet_center[1]))
        dynamic_tunnel_clearance = compute_pallet_tunnel_clearance() - y_error
        lift_top_zs = []
        for part in self.lift_plate_parts:
            try:
                lift_pos, _lift_orient = part.get_world_pose()
                lift_top_zs.append(float(np.array(lift_pos, dtype=float)[2] + LIFT_FORK_SCALE[2] * 0.5))
            except Exception:
                pass
        if lift_top_zs:
            lift_top_z = max(lift_top_zs)
        else:
            lift_top_z = float(amr_pos[2] + AMR_LIFT_PLATE_OFFSET_Z + self.lift_offset + LIFT_FORK_SCALE[2] * 0.5)
        deck_underside_z = float(deck_pos[2] - PALLET_DECK_SCALE[2] * 0.5)
        lift_gap = deck_underside_z - lift_top_z
        self.pickup_entry_sample_count += 1
        self.max_pickup_entry_y_error = max(self.max_pickup_entry_y_error, y_error)
        self.min_pickup_entry_tunnel_clearance = min(
            self.min_pickup_entry_tunnel_clearance,
            float(dynamic_tunnel_clearance),
        )
        self.max_pickup_entry_lift_gap = max(self.max_pickup_entry_lift_gap, float(lift_gap))
        self.max_pickup_entry_lift_penetration = max(
            self.max_pickup_entry_lift_penetration,
            max(0.0, float(-lift_gap)),
        )

    def _record_slide_out_geometry(self):
        if not self.pallet_parts:
            return
        try:
            deck_pos, _deck_orient = self.pallet_parts[0].get_world_pose()
        except Exception:
            return
        deck_pos = np.array(deck_pos, dtype=float)
        pallet_center = deck_pos - PALLET_DECK_OFFSETS[0]
        amr_pos = self.get_amr_position()
        lift_top_zs = []
        for part in self.lift_plate_parts:
            try:
                lift_pos, _lift_orient = part.get_world_pose()
                lift_top_zs.append(float(np.array(lift_pos, dtype=float)[2] + LIFT_FORK_SCALE[2] * 0.5))
            except Exception:
                pass
        if lift_top_zs:
            lift_top_z = max(lift_top_zs)
        else:
            lift_top_z = float(amr_pos[2] + AMR_LIFT_PLATE_OFFSET_Z + self.lift_offset + LIFT_FORK_SCALE[2] * 0.5)
        deck_underside_z = float(deck_pos[2] - PALLET_DECK_SCALE[2] * 0.5)
        lift_gap = deck_underside_z - lift_top_z
        self.slide_out_sample_count += 1
        self.max_slide_out_y_error = max(self.max_slide_out_y_error, abs(float(amr_pos[1] - pallet_center[1])))
        self.max_slide_out_lift_gap = max(self.max_slide_out_lift_gap, float(lift_gap))
        self.max_slide_out_lift_penetration = max(
            self.max_slide_out_lift_penetration,
            max(0.0, float(-lift_gap)),
        )

    def _set_actual_lift_pose(self):
        if self.amr_lift is None:
            return
        amr_pos = self.get_amr_position()
        if self.amr_lift_base_offset is None:
            lift_pos, lift_orient = self.amr_lift.get_world_pose()
            self.amr_lift_base_offset = np.array(lift_pos, dtype=float) - amr_pos
            self.amr_lift_orientation = lift_orient
        target = amr_pos + self.amr_lift_base_offset + np.array([0.0, 0.0, self.lift_offset], dtype=float)
        self.amr_lift.set_world_pose(position=target, orientation=self.amr_lift_orientation)

    def _reset_pallet_pose(self):
        center = np.array([self.args.pickup_x, self.args.pickup_y, PALLET_CENTER_Z + self.lift_offset], dtype=float)
        for part, offset in zip(self.pallet_parts, self.pallet_part_offsets):
            part.set_world_pose(position=center + offset, orientation=yaw_to_quat(0.0))

    def _compute_move_duration(self, move_distance):
        speed = max(float(self.args.move_speed), 1e-6)
        if self.state in (TransferState.MOVE_UNDER_PALLET, TransferState.MOVE_TO_DROP):
            speed *= AMR_DOCK_MOVE_SPEED_SCALE
        return max(float(move_distance) / max(speed, 1e-6), 0.35)

    def _move_amr_toward_target(self, dt):
        if self.move_target is None:
            return True

        if self.move_start_pose is None:
            self.move_start_pose = self.get_amr_position()
            move_distance = float(np.linalg.norm((self.move_target - self.move_start_pose)[:2]))
            self.move_duration = self._compute_move_duration(move_distance)

        t = clamp(self.state_time / max(self.move_duration, 1e-6), 0.0, 1.0)
        next_pos = lerp(self.move_start_pose, self.move_target, smoothstep(t))
        self.set_amr_pose(next_pos)
        self._set_lift_plate_pose()
        self._sync_payload_pose()
        if self.state == TransferState.MOVE_UNDER_PALLET:
            self._record_pickup_entry_geometry()
        if self.state == TransferState.SLIDE_OUT_FROM_PALLET:
            self._record_slide_out_geometry()
        if t >= 1.0:
            self.set_amr_pose(self.move_target)
            self._set_lift_plate_pose()
            self._sync_payload_pose()
            if self.state == TransferState.MOVE_UNDER_PALLET:
                self._record_pickup_entry_geometry()
            if self.state == TransferState.SLIDE_OUT_FROM_PALLET:
                self._record_slide_out_geometry()
            return True
        return False

    def _get_stacked_items(self):
        items = []
        for bin_state in getattr(self.context, "stacked_bins", []):
            bin_obj = getattr(bin_state, "bin_obj", None)
            if bin_obj is not None:
                items.append(bin_obj)
        return items

    def _stop_dynamic_item(self, item):
        for method_name in ("set_linear_velocity", "set_angular_velocity"):
            method = getattr(item, method_name, None)
            if method is not None:
                try:
                    method(np.array([0.0, 0.0, 0.0], dtype=float))
                except TypeError:
                    method([0.0, 0.0, 0.0])

    def _set_item_kinematic(self, item, enabled=True):
        prim = getattr(item, "prim", None)
        prim_path = getattr(item, "prim_path", None)
        if prim is None and prim_path is None:
            return

        try:
            from isaacsim.core.utils.prims import get_prim_at_path
            from pxr import UsdPhysics
        except Exception:
            return

        if prim is None:
            prim = get_prim_at_path(prim_path)
        if prim is None or not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return

        rigid_body = UsdPhysics.RigidBodyAPI(prim)
        attr = rigid_body.GetKinematicEnabledAttr()
        if not attr.IsValid():
            attr = rigid_body.CreateKinematicEnabledAttr()
        attr.Set(bool(enabled))

    def _lock_stack_items(self):
        self.locked_stack_poses = {}
        self.payload_lift_baseline_poses = {}
        self._set_load_restraint_visibility(True)
        for item in self._get_stacked_items():
            try:
                pos, orient = item.get_world_pose()
                self.locked_stack_poses[item.name] = (item, np.array(pos, dtype=float), orient)
                self.payload_lift_baseline_poses[item.name] = np.array(pos, dtype=float)
                self._stop_dynamic_item(item)
                self._set_item_kinematic(item, True)
            except Exception as exc:
                print(f"[HarimDemo] could not lock stacked item: {exc}")

    def _hold_locked_stack(self):
        for item, pos, orient in self.locked_stack_poses.values():
            try:
                item.set_world_pose(position=pos, orientation=orient)
                self._stop_dynamic_item(item)
            except Exception as exc:
                print(f"[HarimDemo] could not hold stacked item: {exc}")

    def _apply_lift_delta_to_stack(self, dz):
        if abs(dz) <= 1e-6:
            return
        for item in self._get_stacked_items():
            try:
                pos, orient = item.get_world_pose()
                lifted = np.array(pos, dtype=float)
                lifted[2] += dz
                item.set_world_pose(position=lifted, orientation=orient)
                self._stop_dynamic_item(item)
                baseline = self.payload_lift_baseline_poses.get(item.name)
                if baseline is not None:
                    self.max_payload_lift_observed = max(
                        self.max_payload_lift_observed,
                        float(lifted[2] - baseline[2]),
                    )
                if item.name in self.locked_stack_poses:
                    self.locked_stack_poses[item.name] = (item, lifted, orient)
            except Exception as exc:
                print(f"[HarimDemo] could not lift stacked item: {exc}")

    def _attach_assembly(self):
        amr_pos = self.get_amr_position()
        self.attached_items = self._get_stacked_items()
        self.item_offsets = {}
        for item in self.attached_items:
            try:
                pos, orient = item.get_world_pose()
                self.item_offsets[item.name] = (np.array(pos, dtype=float) - amr_pos, orient)
                self._stop_dynamic_item(item)
            except Exception as exc:
                print(f"[HarimDemo] could not attach item: {exc}")

        self.pallet_base_offsets = {}
        for part in self.pallet_parts:
            pos, orient = part.get_world_pose()
            self.pallet_base_offsets[part.name] = (np.array(pos, dtype=float) - amr_pos, orient)
        self.carrying = True
        self.locked_stack_poses = {}
        self.payload_lift_baseline_poses = {}
        self._update_attached_items()
        print(f"[HarimDemo] attached {len(self.attached_items)} stacked items and {len(self.pallet_parts)} pallet parts")

    def _update_attached_items(self):
        if not self.carrying:
            return
        amr_pos = self.get_amr_position()
        for item in self.attached_items:
            data = self.item_offsets.get(item.name)
            if data is None:
                continue
            offset, orient = data
            try:
                item.set_world_pose(position=amr_pos + offset, orientation=orient)
                self._stop_dynamic_item(item)
            except Exception as exc:
                print(f"[HarimDemo] could not update carried item: {exc}")
        for part in self.pallet_parts:
            data = self.pallet_base_offsets.get(part.name)
            if data is None:
                continue
            offset, orient = data
            part.set_world_pose(position=amr_pos + offset, orientation=orient)
        self._record_loaded_route_geometry()

    def _record_loaded_route_geometry(self):
        if not self.carrying:
            return
        amr_pos = self.get_amr_position()
        route_y_error = compute_loaded_route_y_error(
            amr_pos[0],
            amr_pos[1],
            self.args.pickup_x,
            self.args.pickup_y,
            self.args.drop_x,
            self.args.drop_y,
        )
        self.max_loaded_route_y_error = max(self.max_loaded_route_y_error, route_y_error)
        route_guard_clearance = compute_loaded_route_guard_clearance(
            amr_pos[1],
            self.args.pickup_y,
            self.args.drop_y,
        )
        self.min_loaded_route_guard_clearance = min(
            self.min_loaded_route_guard_clearance,
            route_guard_clearance,
        )
        for part in self.pallet_parts:
            data = self.pallet_base_offsets.get(part.name)
            if data is None:
                continue
            offset, expected_orient = data
            try:
                current_pos, current_orient = part.get_world_pose()
                expected_pos = amr_pos + offset
                self.max_carried_pallet_pose_error = max(
                    self.max_carried_pallet_pose_error,
                    float(np.linalg.norm(np.array(current_pos, dtype=float) - expected_pos)),
                )
                self.max_carried_pallet_orientation_error = max(
                    self.max_carried_pallet_orientation_error,
                    quat_angular_error(current_orient, expected_orient),
                )
            except Exception:
                pass
        for item in self.attached_items:
            data = self.item_offsets.get(item.name)
            if data is None:
                continue
            offset, expected_orient = data
            try:
                current_pos, current_orient = item.get_world_pose()
                expected_pos = amr_pos + offset
                self.max_carried_payload_pose_error = max(
                    self.max_carried_payload_pose_error,
                    float(np.linalg.norm(np.array(current_pos, dtype=float) - expected_pos)),
                )
                self.max_carried_payload_orientation_error = max(
                    self.max_carried_payload_orientation_error,
                    quat_angular_error(current_orient, expected_orient),
                )
            except Exception:
                pass

    def _record_dropped_assembly(self):
        self.dropped_item_poses = {}
        for item in self.attached_items:
            try:
                pos, orient = item.get_world_pose()
                self.dropped_item_poses[item.name] = (item, np.array(pos, dtype=float), orient)
            except Exception as exc:
                print(f"[HarimDemo] could not record dropped item: {exc}")

        self.dropped_pallet_poses = {}
        for part in self.pallet_parts:
            pos, orient = part.get_world_pose()
            self.dropped_pallet_poses[part.name] = (part, np.array(pos, dtype=float), orient)

    def _record_drop_handoff_geometry(self):
        if not self.pallet_parts:
            return
        try:
            deck_pos, _deck_orient = self.pallet_parts[0].get_world_pose()
        except Exception:
            return
        deck_pos = np.array(deck_pos, dtype=float)
        pallet_center = deck_pos - PALLET_DECK_OFFSETS[0]
        drop_center = np.array([self.args.drop_x, self.args.drop_y, PALLET_CENTER_Z])
        self.drop_handoff_xy_error = float(
            np.linalg.norm((pallet_center - drop_center)[:2])
        )
        deck_underside_z = float(deck_pos[2] - PALLET_DECK_SCALE[2] * 0.5)
        support_gap = deck_underside_z - DROP_SLIDE_SUPPORT_TOP_Z
        self.drop_handoff_support_gap = float(support_gap)
        self.drop_handoff_support_penetration = max(0.0, float(-support_gap))

    def _record_dropped_stack_geometry(self):
        if not self.pallet_parts:
            return
        try:
            deck_pos, _deck_orient = self.pallet_parts[0].get_world_pose()
        except Exception:
            return
        pallet_center = np.array(deck_pos, dtype=float) - PALLET_DECK_OFFSETS[0]
        pickup_center = np.array([self.args.pickup_x, self.args.pickup_y, PALLET_CENTER_Z], dtype=float)
        actual_positions = []
        pose_errors = []
        orientation_errors = []
        for item, expected_coord in zip(self.attached_items, self.stack_coordinates):
            try:
                pos, orient = item.get_world_pose()
            except Exception:
                continue
            pos = np.array(pos, dtype=float)
            actual_positions.append(pos)
            expected_pos = pallet_center + (np.array(expected_coord, dtype=float) - pickup_center)
            pose_errors.append(float(np.linalg.norm(pos - expected_pos)))
            offset_data = self.item_offsets.get(item.name)
            expected_orient = offset_data[1] if offset_data is not None else UPSIDE_DOWN_BIN_QUAT
            orientation_errors.append(quat_angular_error(orient, expected_orient))

        self.dropped_stack_item_count = len(actual_positions)
        self.max_dropped_stack_pose_error = float(max(pose_errors)) if pose_errors else 0.0
        self.max_dropped_stack_orientation_error = float(max(orientation_errors)) if orientation_errors else 0.0
        if not actual_positions:
            self.max_dropped_stack_support_gap = 0.0
            self.min_dropped_stack_support_gap = 0.0
            self.min_dropped_stack_pallet_margin = 0.0
            self.max_dropped_stack_pallet_overhang = 0.0
            return

        stack_min_x = min(float(pos[0]) - CARTON_BODY_SCALE[0] * 0.5 for pos in actual_positions)
        stack_max_x = max(float(pos[0]) + CARTON_BODY_SCALE[0] * 0.5 for pos in actual_positions)
        stack_min_y = min(float(pos[1]) - CARTON_BODY_SCALE[1] * 0.5 for pos in actual_positions)
        stack_max_y = max(float(pos[1]) + CARTON_BODY_SCALE[1] * 0.5 for pos in actual_positions)
        pallet_min_x = float(pallet_center[0]) - PALLET_DECK_SCALE[0] * 0.5
        pallet_max_x = float(pallet_center[0]) + PALLET_DECK_SCALE[0] * 0.5
        pallet_min_y = float(pallet_center[1]) - PALLET_DECK_SCALE[1] * 0.5
        pallet_max_y = float(pallet_center[1]) + PALLET_DECK_SCALE[1] * 0.5
        margins = [
            stack_min_x - pallet_min_x,
            pallet_max_x - stack_max_x,
            stack_min_y - pallet_min_y,
            pallet_max_y - stack_max_y,
        ]
        min_margin = float(min(margins))
        self.min_dropped_stack_pallet_margin = min_margin
        self.max_dropped_stack_pallet_overhang = float(max(0.0, -min_margin))

        half_height = float(CARTON_BODY_SCALE[2] * 0.5)
        pallet_support_top_z = float(
            pallet_center[2] + PALLET_TOP_SUPPORT_OFFSET[2] + PALLET_TOP_SUPPORT_SCALE[2] * 0.5
        )
        support_gaps = []
        for pos in actual_positions:
            bottom_z = float(pos[2] - half_height)
            lower_candidates = [
                lower for lower in actual_positions if float(lower[2]) < float(pos[2]) - half_height
            ]
            if not lower_candidates:
                support_gaps.append(float(bottom_z - pallet_support_top_z))
                continue
            lower = min(
                lower_candidates,
                key=lambda candidate: float(np.linalg.norm((candidate - pos)[:2])),
            )
            lower_top_z = float(lower[2] + half_height)
            support_gaps.append(float(bottom_z - lower_top_z))
        self.max_dropped_stack_support_gap = float(max(support_gaps)) if support_gaps else 0.0
        self.min_dropped_stack_support_gap = float(min(support_gaps)) if support_gaps else 0.0

    def _record_dropped_pallet_geometry(self):
        if not self.pallet_parts:
            return
        try:
            deck_pos, deck_orient = self.pallet_parts[0].get_world_pose()
        except Exception:
            return
        pallet_center = np.array(deck_pos, dtype=float) - PALLET_DECK_OFFSETS[0]
        part_count = 0
        max_pose_error = 0.0
        max_orientation_error = 0.0
        for part, offset in zip(self.pallet_parts, self.pallet_part_offsets):
            try:
                pos, orient = part.get_world_pose()
            except Exception:
                continue
            expected_pos = pallet_center + np.array(offset, dtype=float)
            base_data = self.pallet_base_offsets.get(part.name)
            expected_orient = base_data[1] if base_data is not None else deck_orient
            part_count += 1
            max_pose_error = max(
                max_pose_error,
                float(np.linalg.norm(np.array(pos, dtype=float) - expected_pos)),
            )
            max_orientation_error = max(max_orientation_error, quat_angular_error(orient, expected_orient))
        self.dropped_pallet_part_count = int(part_count)
        self.max_dropped_pallet_part_pose_error = float(max_pose_error)
        self.max_dropped_pallet_part_orientation_error = float(max_orientation_error)

    def _hold_dropped_assembly(self):
        for item, pos, orient in self.dropped_item_poses.values():
            try:
                current_pos, current_orient = item.get_world_pose()
                drift = float(np.linalg.norm(np.array(current_pos, dtype=float) - np.array(pos, dtype=float)))
                self.max_dropped_payload_drift = max(self.max_dropped_payload_drift, drift)
                self.max_dropped_stack_orientation_error = max(
                    self.max_dropped_stack_orientation_error,
                    quat_angular_error(current_orient, orient),
                )
                item.set_world_pose(position=pos, orientation=orient)
                self._stop_dynamic_item(item)
            except Exception as exc:
                print(f"[HarimDemo] could not hold dropped item: {exc}")
        for part, pos, orient in self.dropped_pallet_poses.values():
            current_pos, _ = part.get_world_pose()
            drift = float(np.linalg.norm(np.array(current_pos, dtype=float) - np.array(pos, dtype=float)))
            self.max_dropped_payload_drift = max(self.max_dropped_payload_drift, drift)
            part.set_world_pose(position=pos, orientation=orient)

    def _sync_payload_pose(self):
        if self.carrying:
            self._update_attached_items()
        elif self.locked_stack_poses:
            self._hold_locked_stack()
        else:
            self._hold_dropped_assembly()

    def _detach_assembly(self):
        self._update_attached_items()
        for item in self.attached_items:
            self._stop_dynamic_item(item)
        self._record_dropped_assembly()
        self._record_drop_handoff_geometry()
        self._record_dropped_stack_geometry()
        self._record_dropped_pallet_geometry()
        self.carrying = False
        self.attached_items = []
        self.item_offsets = {}
        self.pallet_base_offsets = {}
        self._hold_dropped_assembly()
        print("[HarimDemo] slide-released pallet assembly at drop pose")

    def _reset_for_next_cycle(self):
        self.completed_cycles += 1
        print(f"[HarimDemo] completed transfer cycle {self.completed_cycles}")

        if self.args.cycles > 0 and self.completed_cycles >= self.args.cycles:
            self._transition(TransferState.DONE_IDLE)
            return

        self.world.reset()
        self.world.play()
        self.context.stack_coordinates = clone_stack_coordinates(self.stack_coordinates)
        self.context.demo_stack_coordinates = clone_stack_coordinates(self.stack_coordinates)
        self.reset_visual_state()
        self._transition(TransferState.WAIT_STACK_COMPLETE)

    def step(self, dt):
        self.state_time += dt
        self._set_lift_plate_pose()
        self._sync_payload_pose()
        self._record_arm_tcp_route_clearance()

        if self.state == TransferState.WAIT_STACK_COMPLETE:
            if getattr(self.context, "stack_complete", False):
                print("[HarimDemo] stack_complete detected")
                if self.completion_signal is not None:
                    self.completion_signal.set_completed(True)
                self._lock_stack_items()
                self._transition(TransferState.ARM_SETTLE)

        elif self.state == TransferState.ARM_SETTLE:
            if self.state_time >= ARM_CLEAR_SETTLE_TIME:
                self._transition(TransferState.MOVE_TO_APPROACH)

        elif self.state in (
            TransferState.MOVE_TO_APPROACH,
            TransferState.MOVE_UNDER_PALLET,
            TransferState.MOVE_TO_DROP_APPROACH,
            TransferState.MOVE_TO_DROP,
            TransferState.SLIDE_OUT_FROM_PALLET,
        ):
            arrived = self._move_amr_toward_target(dt)
            if arrived:
                if self.state == TransferState.MOVE_TO_APPROACH:
                    self._transition(TransferState.MOVE_UNDER_PALLET)
                elif self.state == TransferState.MOVE_UNDER_PALLET:
                    self._record_pickup_handoff_geometry()
                    self._transition(TransferState.LIFT_UP)
                elif self.state == TransferState.MOVE_TO_DROP_APPROACH:
                    self.drop_dock_arrival_count += 1
                    self.drop_approach_final_error = float(
                        np.linalg.norm((self.get_amr_position() - self.drop_approach_pose)[:2])
                    )
                    self._transition(TransferState.MOVE_TO_DROP)
                elif self.state == TransferState.MOVE_TO_DROP:
                    self.drop_dock_final_error = float(
                        np.linalg.norm((self.get_amr_position() - self.drop_pose)[:2])
                    )
                    self._transition(TransferState.LIFT_DOWN)
                elif self.state == TransferState.SLIDE_OUT_FROM_PALLET:
                    self._transition(TransferState.RESET_CYCLE)

        elif self.state == TransferState.LIFT_UP:
            t = clamp(self.state_time / AMR_LIFT_DURATION, 0.0, 1.0)
            previous_offset = self.lift_offset
            self.lift_offset = lerp(0.0, self.args.lift_height, smoothstep(t))
            dz = self.lift_offset - previous_offset
            self._set_lift_plate_pose()
            self._reset_pallet_pose()
            self._apply_lift_delta_to_stack(dz)
            if t >= 1.0 and self.state_time >= AMR_LIFT_DURATION + AMR_LIFT_SETTLE_TIME:
                self._transition(TransferState.ATTACH)

        elif self.state == TransferState.ATTACH:
            self._attach_assembly()
            self._transition(TransferState.MOVE_TO_DROP_APPROACH)

        elif self.state == TransferState.LIFT_DOWN:
            t = clamp(self.state_time / AMR_LIFT_DURATION, 0.0, 1.0)
            previous_offset = self.lift_offset
            self.lift_offset = lerp(self.args.lift_height, 0.0, smoothstep(t))
            dz = self.lift_offset - previous_offset
            if self.carrying and abs(dz) > 1e-6:
                for name, (offset, orient) in list(self.item_offsets.items()):
                    new_offset = np.array(offset, dtype=float)
                    new_offset[2] += dz
                    self.item_offsets[name] = (new_offset, orient)
                for name, (offset, orient) in list(self.pallet_base_offsets.items()):
                    new_offset = np.array(offset, dtype=float)
                    new_offset[2] += dz
                    self.pallet_base_offsets[name] = (new_offset, orient)
            self._set_lift_plate_pose()
            self._update_attached_items()
            if t >= 1.0 and self.state_time >= AMR_LIFT_DURATION + AMR_LIFT_SETTLE_TIME:
                self._transition(TransferState.DETACH)

        elif self.state == TransferState.DETACH:
            self._detach_assembly()
            self._transition(TransferState.SLIDE_OUT_FROM_PALLET)

        elif self.state == TransferState.RESET_CYCLE:
            self._reset_for_next_cycle()

        elif self.state == TransferState.DONE_IDLE:
            pass

        self._record_lift_offset_continuity()


def main():
    args = parse_args()
    configure_local_runtime_dirs()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": args.headless,
            "width": 1280,
            "height": 720,
            "sync_loads": True,
            "renderer": "RaytracedLighting",
        }
    )

    import omni
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension

    if not enable_extension("isaacsim.robot.surface_gripper"):
        raise RuntimeError("Failed to enable required extension: isaacsim.robot.surface_gripper")
    simulation_app.update()

    from isaacsim.core.api.objects.cuboid import FixedCuboid, VisualCuboid
    from isaacsim.core.api.objects.capsule import VisualCapsule
    from isaacsim.core.api.objects.sphere import VisualSphere
    from isaacsim.core.api.tasks.base_task import BaseTask
    from isaacsim.core.prims import SingleXFormPrim
    from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.nucleus import get_assets_root_path
    from isaacsim.core.utils.viewports import set_active_viewport_camera, set_camera_view
    import isaacsim.cortex.framework.math_util as cortex_math_util
    from isaacsim.cortex.framework.cortex_world import CortexWorld
    from isaacsim.cortex.framework.cortex_rigid_prim import CortexRigidPrim
    from isaacsim.cortex.framework.df import (
        DfDecider,
        DfDecision,
        DfNetwork,
        DfSetLockState,
        DfState,
        DfStateMachineDecider,
        DfStateSequence,
        DfWaitState,
    )
    from isaacsim.cortex.framework.dfb import make_go_home
    from isaacsim.cortex.framework.motion_commander import ApproachParams, MotionCommand, PosePq
    from isaacsim.cortex.framework.robot import CortexUr10
    from isaacsim.cortex.behaviors.ur10 import bin_stacking_behavior as behavior
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("Could not resolve the Isaac Sim assets root path.")

    class Ur10Assets:
        def __init__(self, root_path):
            self.assets_root_path = root_path
            self.ur10_table_usd = root_path + "/Isaac/Samples/Leonardo/Stage/ur10_bin_stacking_short_suction.usd"
            self.small_klt_usd = root_path + "/Isaac/Props/KLT_Bin/small_KLT.usd"
            self.background_usd = root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"

    class BinStackingTask(BaseTask):
        def __init__(self, env_path, assets) -> None:
            super().__init__("bin_stacking")
            self.assets = assets
            self.env_path = env_path
            self.bins = []
            self.on_conveyor = None
            self.context = None

        def _spawn_bin(self, rigid_bin):
            position, orientation = random_bin_spawn_transform()
            rigid_bin.set_world_pose(position=position, orientation=orientation)
            rigid_bin.set_linear_velocity(np.array([0.0, -0.45, 0.0], dtype=float))
            rigid_bin.set_visibility(True)
            if args.self_test_debug_bins:
                print(f"[HarimDemo] spawned {rigid_bin.name} at {position.tolist()}", flush=True)

        def _add_carton_visual(self, prim_path, name):
            VisualCuboid(
                f"{prim_path}/HarimCartonBody",
                name=f"{name}_carton_body",
                translation=np.array([0.0, 0.0, 0.0], dtype=float),
                scale=CARTON_BODY_SCALE,
                color=CARTON_BODY_COLOR,
            )
            VisualCuboid(
                f"{prim_path}/HarimCartonTopTape",
                name=f"{name}_carton_top_tape",
                translation=np.array([0.0, 0.0, 0.074], dtype=float),
                scale=CARTON_TAPE_TOP_SCALE,
                color=CARTON_TAPE_COLOR,
            )
            label_y = CARTON_BODY_SCALE[1] * 0.5 + CARTON_SIDE_LABEL_SCALE[1] * 0.5 + 0.001
            for side_name, y_sign in (("Front", 1.0), ("Back", -1.0)):
                VisualCuboid(
                    f"{prim_path}/HarimCartonSideLabel{side_name}",
                    name=f"{name}_carton_side_label_{side_name.lower()}",
                    translation=np.array([0.0, y_sign * label_y, 0.018], dtype=float),
                    scale=CARTON_SIDE_LABEL_SCALE,
                    color=CARTON_LABEL_COLOR,
                )
                VisualCuboid(
                    f"{prim_path}/HarimCartonSideStripe{side_name}",
                    name=f"{name}_carton_side_stripe_{side_name.lower()}",
                    translation=np.array([-0.073, y_sign * (label_y + 0.0008), 0.018], dtype=float),
                    scale=CARTON_SIDE_STRIPE_SCALE,
                    color=CARTON_TAPE_COLOR,
                )

        def post_reset(self) -> None:
            if len(self.bins) > 0:
                for rigid_bin in self.bins:
                    self.scene.remove_object(rigid_bin.name)
                self.bins.clear()
            self.on_conveyor = None

        def pre_step(self, time_step_index, simulation_time) -> None:
            if self.context is not None and getattr(self.context, "stack_complete", False):
                return
            if self.context is not None and (
                getattr(self.context, "active_bin", None) is not None
                or getattr(self.context, "demo_pre_grip_bin", None) is not None
                or getattr(self.context, "demo_carried_bin", None) is not None
                or getattr(self.context, "demo_released_bin", None) is not None
            ):
                return

            spawn_new = False
            if self.on_conveyor is None:
                spawn_new = True
            else:
                (x, y, _z), _ = self.on_conveyor.get_world_pose()
                is_on_conveyor = y > 0.0 and -0.4 < x < 0.4
                if not is_on_conveyor:
                    spawn_new = True

            if spawn_new:
                name = f"bin_{len(self.bins)}"
                prim_path = f"{self.env_path}/bins/{name}"
                add_reference_to_stage(usd_path=self.assets.small_klt_usd, prim_path=prim_path)
                self._add_carton_visual(prim_path, name)
                self.on_conveyor = self.scene.add(CortexRigidPrim(name=name, prim_path=prim_path))
                self._spawn_bin(self.on_conveyor)
                self.bins.append(self.on_conveyor)

        def world_cleanup(self):
            self.bins = []
            self.on_conveyor = None

    def stop_dynamic_prim(obj):
        for method_name in ("set_linear_velocity", "set_angular_velocity"):
            method = getattr(obj, method_name, None)
            if method is not None:
                try:
                    method(np.array([0.0, 0.0, 0.0], dtype=float))
                except TypeError:
                    method([0.0, 0.0, 0.0])

    def set_kinematic_for_demo(obj, enabled=True):
        prim = getattr(obj, "prim", None)
        prim_path = getattr(obj, "prim_path", None)
        if prim is None and prim_path is not None:
            prim = get_prim_at_path(prim_path)
        if prim is None or not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return
        rigid_body = UsdPhysics.RigidBodyAPI(prim)
        attr = rigid_body.GetKinematicEnabledAttr()
        if not attr.IsValid():
            attr = rigid_body.CreateKinematicEnabledAttr()
        attr.Set(bool(enabled))

    def get_demo_carried_bin(context):
        return getattr(context, "demo_carried_bin", None)

    def get_demo_pre_grip_bin(context):
        return getattr(context, "demo_pre_grip_bin", None)

    def restore_demo_carried_active_bin(context):
        carried_bin = get_demo_carried_bin(context)
        if (
            carried_bin is None
            or getattr(carried_bin, "demo_force_released", False)
            or not getattr(carried_bin, "demo_attached", False)
        ):
            return None
        context.active_bin = carried_bin
        carried_bin.is_attached = True
        return carried_bin

    def clear_demo_carry_context(context):
        context.active_bin = None
        context.demo_carried_bin = None
        context.demo_pre_grip_bin = None
        context.demo_scripted_place_bin = None
        context.demo_pre_grip_initial_offset = None

    def mark_demo_bin_released(context, bin_state, target_position, target_orientation):
        bin_state.demo_attached = False
        bin_state.demo_attach_T = None
        bin_state.demo_force_released = True
        bin_state.is_attached = False
        bin_state.is_grasp_reached = False
        bin_state.needs_flip = False
        bin_state.demo_release_target_p = np.array(target_position, dtype=float)
        bin_state.demo_release_target_q = np.array(target_orientation, dtype=float)
        context.demo_released_bin = bin_state
        clear_demo_carry_context(context)

    def force_open_suction_gripper(context):
        gripper = getattr(getattr(context, "robot", None), "suction_gripper", None)
        if gripper is None:
            return
        interface = getattr(gripper, "_surface_gripper_interface", None)
        gripper_path = getattr(gripper, "_surface_gripper_path", None)
        for _attempt in range(SURFACE_GRIPPER_RELEASE_RETRIES):
            try:
                gripper.open()
            except Exception as exc:
                print(f"[HarimDemo] suction open fallback: {exc}", flush=True)
            if interface is None or gripper_path is None:
                continue
            try:
                interface.open_gripper(gripper_path)
            except Exception:
                pass
            set_gripper_action_batch = getattr(interface, "set_gripper_action_batch", None)
            if set_gripper_action_batch is not None:
                try:
                    set_gripper_action_batch([gripper_path], [-1.0])
                except Exception:
                    pass

    def release_demo_bin_at_target(context, bin_state, target_position, target_orientation):
        if bin_state is None:
            return
        force_open_suction_gripper(context)
        mark_demo_bin_released(context, bin_state, target_position, target_orientation)
        bin_state.bin_obj.set_world_pose(position=target_position, orientation=target_orientation)
        stop_dynamic_prim(bin_state.bin_obj)
        set_kinematic_for_demo(bin_state.bin_obj, True)

    def record_release_gripper_state(context):
        gripper = getattr(getattr(context, "robot", None), "suction_gripper", None)
        context.demo_release_gripper_samples = int(getattr(context, "demo_release_gripper_samples", 0)) + 1
        if gripper is None:
            context.demo_release_gripper_probe_failures = (
                int(getattr(context, "demo_release_gripper_probe_failures", 0)) + 1
            )
            return

        try:
            if not gripper.is_open():
                context.demo_release_gripper_not_open_samples = (
                    int(getattr(context, "demo_release_gripper_not_open_samples", 0)) + 1
                )
        except Exception:
            context.demo_release_gripper_probe_failures = (
                int(getattr(context, "demo_release_gripper_probe_failures", 0)) + 1
            )

        interface = getattr(gripper, "_surface_gripper_interface", None)
        gripper_path = getattr(gripper, "_surface_gripper_path", None)
        if interface is None or gripper_path is None:
            return
        try:
            gripped_objects_batch = interface.get_gripped_objects_batch([gripper_path])
            gripped_objects = gripped_objects_batch[0] if gripped_objects_batch else []
            gripped_count = len(gripped_objects)
            context.demo_release_gripped_object_max = max(
                int(getattr(context, "demo_release_gripped_object_max", 0)),
                int(gripped_count),
            )
        except Exception:
            context.demo_release_gripper_probe_failures = (
                int(getattr(context, "demo_release_gripper_probe_failures", 0)) + 1
            )

    def record_release_visual_separation(context, released_bin):
        if released_bin is None:
            return
        robot = getattr(context, "robot", None)
        arm = getattr(robot, "arm", None)
        if arm is None:
            return
        try:
            eff_p = get_measured_arm_fk_p(context)
            bin_p, _ = released_bin.bin_obj.get_world_pose()
            bin_p = np.array(bin_p, dtype=float)
        except Exception:
            return
        separation = float(np.linalg.norm(eff_p - bin_p))
        vertical_clearance = float(eff_p[2] - bin_p[2])
        context.demo_max_release_separation = max(
            float(getattr(context, "demo_max_release_separation", 0.0)),
            separation,
        )
        context.demo_max_release_vertical_clearance = max(
            float(getattr(context, "demo_max_release_vertical_clearance", 0.0)),
            vertical_clearance,
        )

    def hold_demo_released_bin_at_target(context):
        released_bin = getattr(context, "demo_released_bin", None)
        if released_bin is None:
            return

        target_p = getattr(released_bin, "demo_release_target_p", None)
        if target_p is None:
            return
        target_q = getattr(released_bin, "demo_release_target_q", UPSIDE_DOWN_BIN_QUAT)

        current_p, _ = released_bin.bin_obj.get_world_pose()
        release_drift = float(np.linalg.norm(np.array(current_p, dtype=float) - np.array(target_p, dtype=float)))
        context.demo_max_release_drift = max(
            float(getattr(context, "demo_max_release_drift", 0.0)),
            release_drift,
        )

        released_bin.demo_attached = False
        released_bin.demo_attach_T = None
        released_bin.demo_force_released = True
        released_bin.is_attached = False
        if getattr(context, "active_bin", None) is released_bin:
            context.active_bin = None
        released_bin.bin_obj.set_world_pose(position=target_p, orientation=target_q)
        stop_dynamic_prim(released_bin.bin_obj)
        set_kinematic_for_demo(released_bin.bin_obj, True)

    def get_demo_time(context):
        return float(getattr(context, "demo_sim_time", time.time()))

    def set_demo_motion_phase(context, phase):
        context.demo_motion_phase = str(phase)

    def get_demo_stack_coordinate(context, index):
        canonical_coordinates = getattr(context, "demo_stack_coordinates", None)
        if canonical_coordinates is not None:
            return np.array(canonical_coordinates[index], dtype=float)
        return np.array(context.stack_coordinates[index], dtype=float)

    def advance_active_bin_from_conveyor(context, active_bin):
        current_time = get_demo_time(context)
        if not getattr(active_bin, "demo_conveyor_approach_started", False):
            start_position, start_orientation = active_bin.bin_obj.get_world_pose()
            active_bin.demo_conveyor_approach_started = True
            active_bin.demo_conveyor_start_position = np.array(start_position, dtype=float)
            active_bin.demo_conveyor_orientation = np.array(start_orientation, dtype=float)
            active_bin.demo_conveyor_start_time = current_time
            context.demo_active_bin_conveyor_approach_count = int(
                getattr(context, "demo_active_bin_conveyor_approach_count", 0)
            ) + 1

        start_position = np.array(
            getattr(active_bin, "demo_conveyor_start_position", ACTIVE_BIN_CONVEYOR_START_POSITION),
            dtype=float,
        )
        orientation = np.array(
            getattr(active_bin, "demo_conveyor_orientation", UPSIDE_DOWN_BIN_QUAT),
            dtype=float,
        )
        elapsed = current_time - float(getattr(active_bin, "demo_conveyor_start_time", current_time))
        position = compute_active_bin_conveyor_approach_position(start_position, elapsed)
        set_kinematic_for_demo(active_bin.bin_obj, True)
        active_bin.bin_obj.set_world_pose(position=position, orientation=orientation)
        stop_dynamic_prim(active_bin.bin_obj)
        context.demo_infeed_active_bin = active_bin
        context.demo_active_bin_conveyor_observed_travel = max(
            float(getattr(context, "demo_active_bin_conveyor_observed_travel", 0.0)),
            float(np.linalg.norm(position - start_position)),
        )
        context.demo_active_bin_conveyor_lateral_error = max(
            float(getattr(context, "demo_active_bin_conveyor_lateral_error", 0.0)),
            float(abs(position[0] - PICK_STATION_BIN_POSITION[0])),
        )
        if elapsed < ACTIVE_BIN_CONVEYOR_APPROACH_DURATION:
            context.active_bin = None
            return False

        active_bin.bin_obj.set_world_pose(position=PICK_STATION_BIN_POSITION, orientation=orientation)
        stop_dynamic_prim(active_bin.bin_obj)
        final_position, _ = active_bin.bin_obj.get_world_pose()
        context.demo_active_bin_conveyor_final_error = max(
            float(getattr(context, "demo_active_bin_conveyor_final_error", 0.0)),
            float(np.linalg.norm(np.array(final_position, dtype=float) - PICK_STATION_BIN_POSITION)),
        )
        context.demo_active_bin_conveyor_observed_travel = max(
            float(getattr(context, "demo_active_bin_conveyor_observed_travel", 0.0)),
            float(np.linalg.norm(PICK_STATION_BIN_POSITION - start_position)),
        )
        if not getattr(active_bin, "demo_conveyor_completion_recorded", False):
            context.demo_active_bin_conveyor_completed_count = int(
                getattr(context, "demo_active_bin_conveyor_completed_count", 0)
            ) + 1
            active_bin.demo_conveyor_completion_recorded = True
        active_bin.demo_pick_stationed = True
        context.active_bin = None
        return False

    def set_attached_active_bin_pose(context, active_bin, target_position, target_orientation):
        target_position = np.array(target_position, dtype=float)
        try:
            current_position, current_orientation = active_bin.bin_obj.get_world_pose()
        except Exception:
            active_bin.bin_obj.set_world_pose(position=target_position, orientation=target_orientation)
            return

        current_position = np.array(current_position, dtype=float)
        displacement = float(np.linalg.norm(target_position - current_position))
        if displacement > ACTIVE_BIN_ATTACHED_MAX_FRAME_STEP:
            t = ACTIVE_BIN_ATTACHED_MAX_FRAME_STEP / max(displacement, 1e-6)
            position = lerp(current_position, target_position, t)
            orientation = quat_lerp(current_orientation, target_orientation, t)
        else:
            position = target_position
            orientation = target_orientation
        active_bin.bin_obj.set_world_pose(position=position, orientation=orientation)
        attached_grasp_error = float(np.linalg.norm(target_position - np.array(position, dtype=float)))
        context.demo_attached_grasp_sample_count = int(
            getattr(context, "demo_attached_grasp_sample_count", 0)
        ) + 1
        context.demo_max_attached_grasp_error = max(
            float(getattr(context, "demo_max_attached_grasp_error", 0.0)),
            attached_grasp_error,
        )
        context.demo_max_attached_bin_sync_gap = max(
            float(getattr(context, "demo_max_attached_bin_sync_gap", 0.0)),
            attached_grasp_error,
        )

    def hold_active_bin_for_pick(context):
        active_bin = get_demo_pre_grip_bin(context) or getattr(context, "active_bin", None)
        if active_bin is None:
            return
        if active_bin in getattr(context, "stacked_bins", []):
            if getattr(context, "active_bin", None) is active_bin:
                context.active_bin = None
            return
        if getattr(active_bin, "demo_force_released", False):
            if getattr(context, "active_bin", None) is active_bin:
                context.active_bin = None
            return
        if getattr(active_bin, "demo_attached", False):
            return
        context.active_bin = active_bin
        if not getattr(active_bin, "demo_pick_stationed", False):
            advance_active_bin_from_conveyor(context, active_bin)
            return
        else:
            context.demo_infeed_active_bin = None
            _position, orientation = active_bin.bin_obj.get_world_pose()
            set_kinematic_for_demo(active_bin.bin_obj, True)
            active_bin.bin_obj.set_world_pose(position=PICK_STATION_BIN_POSITION, orientation=orientation)
        stop_dynamic_prim(active_bin.bin_obj)

    def compute_active_bin_grasp_pose_at_effector(context, active_bin=None):
        active_bin = active_bin or getattr(context, "active_bin", None)
        if active_bin is None:
            return None
        grasp_T = getattr(active_bin, "grasp_T", None)
        if grasp_T is None:
            return None

        eff_T = get_measured_arm_fk_T(context)
        bin_T = cortex_math_util.pq2T(*active_bin.bin_obj.get_world_pose())
        grasp_to_bin_T = cortex_math_util.invert_T(grasp_T).dot(bin_T)
        desired_bin_T = eff_T.dot(grasp_to_bin_T)
        position, orientation = cortex_math_util.T2pq(desired_bin_T)
        offset = float(np.linalg.norm(eff_T[:3, 3] - grasp_T[:3, 3]))
        return position, orientation, offset

    def place_active_bin_grasp_at_effector(context, active_bin=None):
        active_bin = active_bin or getattr(context, "active_bin", None)
        if getattr(active_bin, "demo_force_released", False):
            return None
        target = compute_active_bin_grasp_pose_at_effector(context, active_bin)
        if active_bin is None or target is None:
            return None
        position, orientation, offset = target
        context.active_bin = active_bin
        set_kinematic_for_demo(active_bin.bin_obj, True)
        active_bin.bin_obj.set_world_pose(position=position, orientation=orientation)
        stop_dynamic_prim(active_bin.bin_obj)
        return offset

    class DemoSettleBinAtGripper(DfState):
        def __init__(self, min_duration=0.25, max_duration=1.10):
            self.min_duration = min_duration
            self.max_duration = max_duration
            self.duration = min_duration
            self.entry_time = None
            self.start_position = None
            self.start_orientation = None

        def enter(self):
            set_demo_motion_phase(self.context, "pre_grip_settle")
            self.entry_time = get_demo_time(self.context)
            active_bin = getattr(self.context, "active_bin", None)
            self.context.demo_pre_grip_bin = active_bin
            target = compute_active_bin_grasp_pose_at_effector(self.context, active_bin)
            offset = None
            if active_bin is not None and target is not None:
                self.start_position, self.start_orientation = active_bin.bin_obj.get_world_pose()
                _target_position, _target_orientation, offset = target
                self.duration = clamp(0.30 + offset * 0.60, self.min_duration, self.max_duration)
                set_kinematic_for_demo(active_bin.bin_obj, True)
                stop_dynamic_prim(active_bin.bin_obj)
            self.context.demo_pre_grip_initial_offset = offset
            if offset is not None:
                self.context.demo_max_pre_grip_offset = max(
                    float(getattr(self.context, "demo_max_pre_grip_offset", 0.0)),
                    float(offset),
                )
            if args.self_test_debug_bins and offset is not None:
                print(f"[HarimDemo] pre-grip offset {offset:.4f} m; settle={self.duration:.2f}s", flush=True)

        def step(self):
            set_demo_motion_phase(self.context, "pre_grip_settle")
            if self.entry_time is None:
                return None
            active_bin = get_demo_pre_grip_bin(self.context) or getattr(self.context, "active_bin", None)
            target = compute_active_bin_grasp_pose_at_effector(self.context, active_bin)
            if active_bin is None or target is None:
                return None
            self.context.active_bin = active_bin
            if self.start_position is None or self.start_orientation is None:
                self.start_position, self.start_orientation = active_bin.bin_obj.get_world_pose()
            target_position, target_orientation, _offset = target
            t = clamp((get_demo_time(self.context) - self.entry_time) / max(self.duration, 1e-6), 0.0, 1.0)
            position = lerp(np.array(self.start_position, dtype=float), np.array(target_position, dtype=float), smoothstep(t))
            orientation = quat_lerp(self.start_orientation, target_orientation, smoothstep(t))
            set_kinematic_for_demo(active_bin.bin_obj, True)
            active_bin.bin_obj.set_world_pose(position=position, orientation=orientation)
            stop_dynamic_prim(active_bin.bin_obj)
            if t < 1.0:
                return self
            return None

        def exit(self):
            active_bin = get_demo_pre_grip_bin(self.context)
            if active_bin is not None:
                place_active_bin_grasp_at_effector(self.context, active_bin)
            self.entry_time = None
            self.start_position = None
            self.start_orientation = None

    class DemoAttachBin(DfState):
        def enter(self):
            set_demo_motion_phase(self.context, "attach_bin")
            print("<close gripper>", flush=True)
            active_bin = get_demo_pre_grip_bin(self.context) or getattr(self.context, "active_bin", None)
            if active_bin is None:
                if args.self_test_debug_bins:
                    print("[HarimDemo] demo attach skipped: no active bin", flush=True)
                self.context.demo_pre_grip_bin = None
                return

            self.context.active_bin = active_bin
            pre_grip_offset = getattr(self.context, "demo_pre_grip_initial_offset", None)
            force_open_suction_gripper(self.context)
            if args.self_test_debug_bins:
                if pre_grip_offset is None:
                    print("[HarimDemo] suction close skipped for fallback attach; using scripted attach", flush=True)
                else:
                    print(
                        f"[HarimDemo] suction close skipped for fallback attach; "
                        f"pre-grip offset={pre_grip_offset:.4f} m; using scripted attach",
                        flush=True,
                    )

            place_active_bin_grasp_at_effector(self.context, active_bin)
            eff_T = get_measured_arm_fk_T(self.context)
            bin_T = cortex_math_util.pq2T(*active_bin.bin_obj.get_world_pose())
            active_bin.demo_attached = True
            active_bin.demo_attach_T = cortex_math_util.invert_T(eff_T).dot(bin_T)
            active_bin.demo_force_released = False
            active_bin.is_attached = True
            active_bin.demo_pick_stationed = True
            self.context.demo_carried_bin = active_bin
            self.context.demo_pre_grip_bin = None
            stop_dynamic_prim(active_bin.bin_obj)
            set_kinematic_for_demo(active_bin.bin_obj, True)
            print(f"[HarimDemo] demo-attached {active_bin.bin_obj.name}", flush=True)

        def step(self):
            return None

    class DemoScriptedPlaceBin(DfState):
        def __init__(self, duration=SCRIPTED_PLACE_DURATION):
            self.duration = duration
            self.entry_time = None
            self.active_bin = None
            self.start_position = None
            self.start_orientation = None
            self.target_p = None
            self.target_q = None

        def enter(self):
            set_demo_motion_phase(self.context, "scripted_place")
            self.entry_time = get_demo_time(self.context)
            active_bin = get_demo_carried_bin(self.context) or getattr(self.context, "active_bin", None)
            if active_bin is None:
                return

            self.active_bin = active_bin
            self.start_position, self.start_orientation = active_bin.bin_obj.get_world_pose()
            self.target_p = get_demo_stack_coordinate(self.context, len(self.context.stacked_bins))
            self.target_q = UPSIDE_DOWN_BIN_QUAT.copy()
            self.context.active_bin = active_bin
            self.context.demo_scripted_place_bin = active_bin
            self.context.demo_scripted_place_count = int(
                getattr(self.context, "demo_scripted_place_count", 0)
            ) + 1
            active_bin.demo_attached = True
            active_bin.demo_force_released = False
            active_bin.is_attached = True
            force_open_suction_gripper(self.context)
            set_kinematic_for_demo(active_bin.bin_obj, True)
            stop_dynamic_prim(active_bin.bin_obj)
            print(f"[HarimDemo] scripted-place {active_bin.bin_obj.name} -> {self.target_p.tolist()}", flush=True)

        def step(self):
            set_demo_motion_phase(self.context, "scripted_place")
            if self.active_bin is None or self.target_p is None:
                return None
            force_open_suction_gripper(self.context)
            elapsed = get_demo_time(self.context) - self.entry_time
            t = clamp(elapsed / max(self.duration, 1e-6), 0.0, 1.0)
            eased_t = smoothstep(t)
            position = lerp(np.array(self.start_position, dtype=float), np.array(self.target_p, dtype=float), eased_t)
            orientation = quat_lerp(self.start_orientation, self.target_q, eased_t)
            self.active_bin.bin_obj.set_world_pose(position=position, orientation=orientation)
            stop_dynamic_prim(self.active_bin.bin_obj)
            set_kinematic_for_demo(self.active_bin.bin_obj, True)
            hover_target = np.array(self.target_p, dtype=float) + np.array(
                [0.0, 0.0, SCRIPTED_PLACE_EE_HOVER], dtype=float
            )
            self.context.robot.arm.send(
                MotionCommand(target_position=hover_target, posture_config=self.context.robot.default_config)
            )
            if t < 1.0:
                return self
            return None

        def exit(self):
            if self.active_bin is not None and self.target_p is not None:
                release_demo_bin_at_target(self.context, self.active_bin, self.target_p, self.target_q)
                place_error = float(
                    np.linalg.norm(np.array(self.active_bin.bin_obj.get_world_pose()[0], dtype=float) - self.target_p)
                )
                self.context.demo_max_scripted_place_error = max(
                    float(getattr(self.context, "demo_max_scripted_place_error", 0.0)),
                    place_error,
                )
                if args.self_test_debug_bins:
                    print(f"[HarimDemo] scripted-release {self.active_bin.bin_obj.name}", flush=True)
            self.entry_time = None
            self.active_bin = None
            self.start_position = None
            self.start_orientation = None
            self.target_p = None
            self.target_q = None

    class DemoReleaseBin(DfState):
        def __init__(self, release_duration=RELEASE_RETREAT_DURATION):
            self.release_duration = release_duration
            self.entry_time = None
            self.released_bin = None
            self.target_p = None
            self.retreat_position = None
            self.release_start_arm_z = None

        def _send_retreat_command(self):
            if self.retreat_position is None:
                return
            self.context.robot.arm.send(
                MotionCommand(
                    target_position=self.retreat_position,
                    posture_config=self.context.robot.default_config,
                )
            )

        def enter(self):
            set_demo_motion_phase(self.context, "release_retreat")
            self.entry_time = get_demo_time(self.context)
            print("<open gripper>", flush=True)
            force_open_suction_gripper(self.context)

            active_bin = getattr(self.context, "demo_released_bin", None)
            active_bin = active_bin or get_demo_carried_bin(self.context) or self.context.active_bin
            if active_bin is None:
                return

            self.context.demo_scripted_place_bin = None
            target_index = len(self.context.stacked_bins)
            self.target_p = get_demo_stack_coordinate(self.context, target_index)
            self.released_bin = active_bin
            current_fk_p = get_measured_arm_fk_p(self.context)
            self.release_start_arm_z = float(current_fk_p[2])
            self.retreat_position = np.array(self.target_p, dtype=float) + POST_RELEASE_RETREAT_OFFSET
            self.retreat_position[2] = max(
                float(self.retreat_position[2]),
                float(current_fk_p[2] + POST_RELEASE_CLEARANCE_LIFT),
            )
            release_demo_bin_at_target(self.context, active_bin, self.target_p, UPSIDE_DOWN_BIN_QUAT)
            self._send_retreat_command()
            record_release_visual_separation(self.context, active_bin)
            if args.self_test_require_gripper_open_after_release:
                record_release_gripper_state(self.context)
            print(f"[HarimDemo] demo-placed {active_bin.bin_obj.name} at {self.target_p.tolist()}", flush=True)

        def step(self):
            set_demo_motion_phase(self.context, "release_retreat")
            if self.released_bin is None or self.target_p is None:
                return None
            force_open_suction_gripper(self.context)
            mark_demo_bin_released(self.context, self.released_bin, self.target_p, UPSIDE_DOWN_BIN_QUAT)
            hold_demo_released_bin_at_target(self.context)
            self._send_retreat_command()
            record_release_visual_separation(self.context, self.released_bin)
            if self.release_start_arm_z is not None:
                current_arm_z = float(get_measured_arm_fk_p(self.context)[2])
                self.context.demo_max_release_retreat_lift = max(
                    float(getattr(self.context, "demo_max_release_retreat_lift", 0.0)),
                    current_arm_z - self.release_start_arm_z,
                )
            if args.self_test_require_gripper_open_after_release:
                record_release_gripper_state(self.context)
            if get_demo_time(self.context) - self.entry_time < self.release_duration:
                return self
            return None

        def exit(self):
            if self.released_bin is not None and self.target_p is not None:
                force_open_suction_gripper(self.context)
                mark_demo_bin_released(self.context, self.released_bin, self.target_p, UPSIDE_DOWN_BIN_QUAT)
                hold_demo_released_bin_at_target(self.context)
                record_release_visual_separation(self.context, self.released_bin)
            self.entry_time = None
            self.released_bin = None
            self.target_p = None
            self.retreat_position = None
            self.release_start_arm_z = None

    class DemoTimedArmLift(DfState):
        def __init__(self, height, duration):
            self.height = height
            self.duration = duration
            self.entry_time = None
            self.target_pq = None

        def enter(self):
            set_demo_motion_phase(self.context, "post_pick_lift")
            self.entry_time = get_demo_time(self.context)
            self.target_pq = self.context.robot.arm.get_fk_pq()
            self.target_pq.p[2] += self.height

        def step(self):
            set_demo_motion_phase(self.context, "post_pick_lift")
            hold_demo_released_bin_at_target(self.context)
            self.context.robot.arm.send(MotionCommand(self.target_pq))
            if get_demo_time(self.context) - self.entry_time < self.duration:
                return self
            return None

        def exit(self):
            self.entry_time = None
            self.target_pq = None

    class DemoStableReachToPick(DfState):
        def __init__(self, p_thresh=0.005, r_thresh=2.0):
            self.p_thresh = p_thresh
            self.r_thresh = r_thresh
            self.target_p = None
            self.target_R = None
            self.target_ax = None
            self.posture_config = np.array([-1.2654234, -2.9708025, -2.219733, 0.6445836, 1.5186214, 0.30098662])

        def enter(self):
            active_bin = getattr(self.context, "active_bin", None)
            if active_bin is None or getattr(active_bin, "grasp_T", None) is None:
                self.target_p = None
                self.target_R = None
                self.target_ax = None
                return

            target_R, target_p = cortex_math_util.unpack_T(active_bin.grasp_T)
            target_ax, _target_ay, _target_az = cortex_math_util.unpack_R(target_R)
            eff_R = get_measured_arm_fk_T(self.context)[:3, :3]
            self.target_R = behavior.adjust_about_x_if_opposite(eff_R, target_R)
            self.target_p = np.array(target_p, dtype=float)
            self.target_ax = np.array(target_ax, dtype=float)
            try:
                self.context.flip_station_obs_monitor.activate_autotoggle()
                self.context.navigation_obs_monitor.activate_autotoggle()
            except Exception:
                pass

        def step(self):
            if self.target_p is None or self.target_R is None or self.target_ax is None:
                return None

            current_p = get_measured_arm_fk_p(self.context)
            distance_to_target = float(np.linalg.norm(self.target_p - current_p))
            approach_length = min(0.10, distance_to_target)
            command = MotionCommand(
                target_pose=PosePq(self.target_p, cortex_math_util.matrix_to_quat(self.target_R)),
                approach_params=ApproachParams(direction=approach_length * self.target_ax, std_dev=0.005),
                posture_config=self.posture_config,
            )
            self.context.robot.arm.send(command)
            fk_T = get_measured_arm_fk_T(self.context)
            if cortex_math_util.transforms_are_close(
                command.target_pose.to_T(),
                fk_T,
                p_thresh=self.p_thresh,
                R_thresh=self.r_thresh,
            ):
                return None
            return self

        def exit(self):
            try:
                self.context.flip_station_obs_monitor.deactivate_autotoggle()
                self.context.navigation_obs_monitor.deactivate_autotoggle()
            except Exception:
                pass
            self.target_p = None
            self.target_R = None
            self.target_ax = None

    class DemoTimedArmJointSettle(DfState):
        def __init__(self, duration=POST_RELEASE_JOINT_SETTLE_DURATION):
            self.duration = duration
            self.entry_time = None
            self.target_pq = None
            self.active = False

        def enter(self):
            set_demo_motion_phase(self.context, "post_release_joint_settle")
            self.entry_time = get_demo_time(self.context)
            self.active = False
            robot = self.context.robot
            try:
                self.target_pq = robot.arm.get_fk_pq()
            except Exception as exc:
                print(f"[HarimDemo] joint settle skipped: {exc}", flush=True)
                return
            self.active = True
            self.context.demo_joint_settle_count = int(getattr(self.context, "demo_joint_settle_count", 0)) + 1
            robot.arm.clear()

        def step(self):
            set_demo_motion_phase(self.context, "post_release_joint_settle")
            hold_demo_released_bin_at_target(self.context)
            if not self.active:
                return None
            elapsed = get_demo_time(self.context) - self.entry_time
            try:
                self.context.robot.arm.send(MotionCommand(self.target_pq))
            except Exception as exc:
                print(f"[HarimDemo] joint settle aborted: {exc}", flush=True)
                return None
            if elapsed < self.duration:
                return self
            return None

        def exit(self):
            self.entry_time = None
            self.target_pq = None
            self.active = False

    class DemoTimedArmMoveTo(DfState):
        def __init__(self, position, duration, label, position_threshold=RETURN_READY_POSITION_THRESHOLD):
            self.position = np.array(position, dtype=float)
            self.duration = duration
            self.label = label
            self.position_threshold = position_threshold
            self.entry_time = None
            self.target_position = None

        def enter(self):
            set_demo_motion_phase(self.context, self.label)
            self.entry_time = get_demo_time(self.context)
            self.target_position = self.position.copy()
            print(f"[HarimDemo] {self.label} start", flush=True)

        def step(self):
            set_demo_motion_phase(self.context, self.label)
            hold_demo_released_bin_at_target(self.context)
            self.context.robot.arm.send(
                MotionCommand(target_position=self.target_position, posture_config=self.context.robot.default_config)
            )
            current_position = get_measured_arm_fk_p(self.context)
            position_error = float(np.linalg.norm(current_position - self.target_position))
            elapsed = get_demo_time(self.context) - self.entry_time
            if position_error <= self.position_threshold:
                self._record_final_error(position_error)
                print(f"[HarimDemo] {self.label} reached; error={position_error:.4f} m", flush=True)
                return None
            if elapsed < self.duration:
                return self
            self._record_final_error(position_error)
            print(f"[HarimDemo] {self.label} timed release; error={position_error:.4f} m", flush=True)
            return None

        def exit(self):
            self.entry_time = None
            self.target_position = None

        def _record_final_error(self, position_error):
            if self.label != "return_ready":
                return
            self.context.demo_max_return_ready_error = max(
                float(getattr(self.context, "demo_max_return_ready_error", 0.0)),
                float(position_error),
            )

    class DemoTimedState(DfState):
        def __init__(self, state, max_duration, label):
            self.state = state
            self.max_duration = max_duration
            self.label = label
            self.entry_time = None

        def bind(self, context, params):
            self.context = context
            self.params = params
            if hasattr(self.state, "bind"):
                self.state.bind(context, params)

        def enter(self):
            set_demo_motion_phase(self.context, self.label)
            self.entry_time = get_demo_time(self.context)
            print(f"[HarimDemo] {self.label} start", flush=True)
            restore_demo_carried_active_bin(self.context)
            self.state.enter()

        def step(self):
            set_demo_motion_phase(self.context, self.label)
            restore_demo_carried_active_bin(self.context)
            next_state = self.state.step()
            if next_state is None:
                return None
            if get_demo_time(self.context) - self.entry_time >= self.max_duration:
                print(f"[HarimDemo] {self.label} timed release", flush=True)
                return None
            return self

        def exit(self):
            self.state.exit()
            self.entry_time = None

    class DemoMarkCarriedBinComplete(DfState):
        def enter(self):
            set_demo_motion_phase(self.context, "mark_bin_complete")
            hold_demo_released_bin_at_target(self.context)
            active_bin = (
                getattr(self.context, "demo_released_bin", None)
                or get_demo_carried_bin(self.context)
                or self.context.active_bin
            )
            if active_bin is not None and active_bin not in self.context.stacked_bins:
                self.context.stacked_bins.append(active_bin)
                print(
                    f"[HarimDemo] stack-count {len(self.context.stacked_bins)}/"
                    f"{len(self.context.stack_coordinates)} after {active_bin.bin_obj.name}",
                    flush=True,
                )
            clear_demo_carry_context(self.context)
            self.context.demo_released_bin = None

    class DemoWaitForNextBin(DfState):
        def enter(self):
            set_demo_motion_phase(self.context, "wait_next_bin")
            self.context.robot.arm.clear()

        def step(self):
            set_demo_motion_phase(self.context, "wait_next_bin")
            return self

    class DemoPickAndPlaceBin(DfStateMachineDecider):
        def __init__(self):
            super().__init__(
                DfStateSequence(
                    [
                        DemoTimedState(DemoStableReachToPick(), max_duration=REACH_PICK_MAX_DURATION, label="reach_pick"),
                        DemoSettleBinAtGripper(min_duration=0.25, max_duration=1.10),
                        DfSetLockState(set_locked_to=True, decider=self),
                        DemoAttachBin(),
                        DemoTimedArmLift(height=0.24, duration=0.35),
                        DemoTimedState(behavior.ReachToPlace(), max_duration=REACH_PLACE_MAX_DURATION, label="reach_place"),
                        DfWaitState(wait_time=0.15),
                        DemoScriptedPlaceBin(),
                        DemoReleaseBin(),
                        DemoTimedArmJointSettle(),
                        DemoTimedArmMoveTo(
                            PICK_READY_EE_POSITION,
                            duration=RETURN_READY_DURATION,
                            label="return_ready",
                        ),
                        DemoMarkCarriedBinComplete(),
                        DfSetLockState(set_locked_to=False, decider=self),
                    ]
                )
            )

        def decide(self):
            if self.state is None:
                self.enter()
            return super().decide()

    def sync_demo_attached_bin(context):
        active_bin = restore_demo_carried_active_bin(context)
        if active_bin is None:
            if not context.has_active_bin:
                return
            active_bin = context.active_bin
        if active_bin in getattr(context, "stacked_bins", []) or getattr(active_bin, "demo_force_released", False):
            if getattr(context, "active_bin", None) is active_bin:
                context.active_bin = None
            return
        if getattr(context, "demo_scripted_place_bin", None) is active_bin:
            return
        if not getattr(active_bin, "demo_attached", False):
            return
        attach_T = getattr(active_bin, "demo_attach_T", None)
        if attach_T is None:
            return
        context.active_bin = active_bin
        bin_T = get_measured_arm_fk_T(context).dot(attach_T)
        position, orientation = cortex_math_util.T2pq(bin_T)
        set_attached_active_bin_pose(context, active_bin, position, orientation)
        stop_dynamic_prim(active_bin.bin_obj)
        active_bin.is_attached = True

    class NoFlipDispatch(DfDecider):
        def __init__(self):
            super().__init__()
            self.add_child("pick_place_bin", DemoPickAndPlaceBin())
            self.add_child("wait_next_bin", DfStateMachineDecider(DemoWaitForNextBin()))
            self.add_child("go_home", make_go_home())
            self.add_child("do_nothing", DfStateMachineDecider(behavior.DoNothing()))

        def decide(self):
            restore_demo_carried_active_bin(self.context)
            hold_demo_released_bin_at_target(self.context)
            active_name = self.context.active_bin.bin_obj.name if self.context.has_active_bin else None
            if args.self_test_debug_bins and active_name != getattr(self.context, "demo_last_active_name", None):
                print(f"[HarimDemo] active-bin -> {active_name}", flush=True)
                self.context.demo_last_active_name = active_name
            hold_active_bin_for_pick(self.context)
            if self.context.stack_complete:
                return DfDecision("go_home")
            if self.context.has_active_bin:
                return DfDecision("pick_place_bin")
            return DfDecision("wait_next_bin")

    def make_no_flip_decider_network(robot, monitor_fn):
        return DfNetwork(NoFlipDispatch(), context=behavior.BinStackingContext(robot, monitor_fn))

    usd_context = omni.usd.get_context()
    world = CortexWorld()

    ur10_assets = Ur10Assets(assets_root)
    add_reference_to_stage(ur10_assets.ur10_table_usd, "/World/Ur10Table")
    add_reference_to_stage(ur10_assets.background_usd, "/World/Background")
    wait_for_stage_loading(simulation_app, usd_context, "UR10 palletizing scene")
    deactivate_stage_prims_containing(usd_context.get_stage(), "/World/Ur10Table", ("flip", "pallet", "pallet_holder"))
    SingleXFormPrim("/World/Background", position=[10.00, 2.00, -1.18180], orientation=[0.7071, 0, 0, 0.7071])

    robot = world.add_robot(CortexUr10(name="robot", prim_path="/World/Ur10Table/ur10"))

    obs = world.scene.add(
        VisualSphere(
            "/World/Ur10Table/Obstacles/FlipStationSphere",
            name="flip_station_sphere",
            position=np.array([0.73, 0.76, -0.13]),
            radius=0.2,
            visible=False,
        )
    )
    robot.register_obstacle(obs)

    obs = world.scene.add(
        VisualSphere(
            "/World/Ur10Table/Obstacles/NavigationDome",
            name="navigation_dome_obs",
            position=[-0.031, -0.018, -1.086],
            radius=1.1,
            visible=False,
        )
    )
    robot.register_obstacle(obs)

    az = np.array([1.0, 0.0, -0.3])
    ax = np.array([0.0, 1.0, 0.0])
    ay = np.cross(az, ax)
    rotation = cortex_math_util.pack_R(ax, ay, az)
    quat = cortex_math_util.matrix_to_quat(rotation)
    obs = world.scene.add(
        VisualCapsule(
            "/World/Ur10Table/Obstacles/NavigationBarrier",
            name="navigation_barrier_obs",
            position=[0.471, 0.276, -0.563],
            orientation=quat,
            radius=0.5,
            height=0.9,
            visible=False,
        )
    )
    robot.register_obstacle(obs)

    obs = world.scene.add(
        VisualCapsule(
            "/World/Ur10Table/Obstacles/NavigationFlipStation",
            name="navigation_flip_station_obs",
            position=np.array([0.766, 0.755, -0.5]),
            radius=0.5,
            height=0.5,
            visible=False,
        )
    )
    robot.register_obstacle(obs)

    stack_coordinates = clone_stack_coordinates(make_stack_coordinates(args.stack_cols, args.stack_rows, args.stack_layers))
    task = BinStackingTask("/World/Ur10Table", ur10_assets)
    task.set_up_scene(world.scene)
    world.add_task(task)

    decider_network = make_no_flip_decider_network(robot, lambda _diagnostic: None)
    decider_network.context.stack_coordinates = clone_stack_coordinates(stack_coordinates)
    decider_network.context.demo_stack_coordinates = clone_stack_coordinates(stack_coordinates)
    task.context = decider_network.context
    world.add_decider_network(decider_network)

    harim_root = "/World/HarimDemo"
    omni.kit.commands.execute("CreatePrim", prim_path=harim_root, prim_type="Xform")

    def create_camera_rig():
        camera_root = f"{harim_root}/Cameras"
        omni.kit.commands.execute("CreatePrim", prim_path=camera_root, prim_type="Xform")
        created_camera_paths = []
        camera_paths_by_role = {}
        stage = usd_context.get_stage()

        for camera_name, camera_role, camera_eye, camera_target, focal_length in make_camera_rig_specs(
            args.pickup_x,
            args.pickup_y,
            args.drop_x,
            args.drop_y,
        ):
            camera_path = f"{camera_root}/{camera_name}"
            omni.kit.commands.execute("CreatePrim", prim_path=camera_path, prim_type="Camera")
            camera_prim = stage.GetPrimAtPath(camera_path)
            if camera_prim and camera_prim.IsValid():
                camera = UsdGeom.Camera(camera_prim)
                camera.CreateFocalLengthAttr().Set(float(focal_length))
                camera_prim.CreateAttribute("harim:cameraRole", Sdf.ValueTypeNames.String).Set(camera_role)
            try:
                set_camera_view(eye=camera_eye, target=camera_target, camera_prim_path=camera_path)
            except Exception as exc:
                print(f"[HarimDemo] camera view setup failed for {camera_name}: {exc}", flush=True)
            created_camera_paths.append(camera_path)
            camera_paths_by_role[camera_role] = camera_path

        if "overview" in camera_paths_by_role:
            try:
                set_active_viewport_camera(camera_paths_by_role["overview"])
            except Exception as exc:
                print(f"[HarimDemo] active viewport camera setup skipped: {exc}", flush=True)
        print(f"[HarimDemo] camera rig ready: {len(created_camera_paths)} cameras", flush=True)
        return camera_paths_by_role

    camera_paths_by_role = create_camera_rig()

    def set_story_camera(role):
        camera_path = camera_paths_by_role.get(role)
        if camera_path is None:
            camera_path = camera_paths_by_role.get("overview")
        if camera_path is None:
            return
        set_active_viewport_camera(camera_path)
        print(f"[HarimDemo] camera director -> {role} ({camera_path})", flush=True)

    def create_warehouse_lighting():
        lighting_root = f"{harim_root}/Lighting"
        omni.kit.commands.execute("CreatePrim", prim_path=lighting_root, prim_type="Xform")
        created_light_paths = []
        stage = usd_context.get_stage()

        def set_light_attribute(prim, attr_name, value):
            attr = prim.GetAttribute(attr_name)
            if attr and attr.IsValid():
                attr.Set(value)

        for light_name, light_role, light_position, fixture_scale, intensity in make_warehouse_light_specs(
            args.pickup_x,
            args.pickup_y,
            args.drop_x,
            args.drop_y,
        ):
            light_path = f"{lighting_root}/{light_name}"
            omni.kit.commands.execute("CreatePrim", prim_path=light_path, prim_type="RectLight")
            light_prim = stage.GetPrimAtPath(light_path)
            if light_prim and light_prim.IsValid():
                UsdGeom.XformCommonAPI(light_prim).SetTranslate(tuple(float(value) for value in light_position))
                set_light_attribute(light_prim, "inputs:intensity", float(intensity))
                set_light_attribute(light_prim, "inputs:width", float(fixture_scale[0]))
                set_light_attribute(light_prim, "inputs:height", float(max(fixture_scale[1], 0.1)))
                set_light_attribute(light_prim, "inputs:color", Gf.Vec3f(*WAREHOUSE_LIGHT_EMISSIVE_COLOR.tolist()))
                light_prim.CreateAttribute("harim:lightRole", Sdf.ValueTypeNames.String).Set(light_role)
            world.scene.add(
                VisualCuboid(
                    f"{lighting_root}/{light_name}Fixture",
                    name=f"harim_{light_name.lower()}_fixture",
                    position=np.array(light_position, dtype=float) + np.array([0.0, 0.0, 0.045], dtype=float),
                    scale=np.array(fixture_scale, dtype=float),
                    color=WAREHOUSE_LIGHT_FIXTURE_COLOR,
                )
            )
            world.scene.add(
                VisualCuboid(
                    f"{lighting_root}/{light_name}GlowPanel",
                    name=f"harim_{light_name.lower()}_glow_panel",
                    position=np.array(light_position, dtype=float) + np.array([0.0, 0.0, -0.002], dtype=float),
                    scale=np.array([fixture_scale[0] * 0.92, fixture_scale[1] * 0.72, 0.012], dtype=float),
                    color=WAREHOUSE_LIGHT_EMISSIVE_COLOR,
                )
            )
            created_light_paths.append(light_path)

        print(f"[HarimDemo] warehouse lighting ready: {len(created_light_paths)} high-bay lights", flush=True)
        return created_light_paths

    create_warehouse_lighting()

    def create_infeed_conveyor_visual():
        visual_parts = []
        motion_parts = []
        motion_base_y_values = []
        feed_carton_part_pairs = []
        feed_carton_base_y_values = []
        belt_color = np.array([0.11, 0.12, 0.13], dtype=float)
        rail_color = np.array([0.60, 0.64, 0.66], dtype=float)
        roller_color = np.array([0.34, 0.37, 0.39], dtype=float)
        sensor_color = np.array([0.08, 0.11, 0.14], dtype=float)
        beam_color = np.array([0.95, 0.78, 0.12], dtype=float)

        visual_parts.append(
            world.scene.add(
                VisualCuboid(
                    f"{harim_root}/InfeedConveyorBelt",
                    name="harim_infeed_conveyor_belt",
                    position=np.array(
                        [
                            0.0,
                            INFEED_CONVEYOR_CENTER_Y,
                            INFEED_CONVEYOR_TOP_Z - INFEED_CONVEYOR_THICKNESS * 0.5,
                        ],
                        dtype=float,
                    ),
                    scale=np.array(
                        [INFEED_CONVEYOR_WIDTH, INFEED_CONVEYOR_LENGTH, INFEED_CONVEYOR_THICKNESS],
                        dtype=float,
                    ),
                    color=belt_color,
                )
            )
        )
        for marker_name, marker_position, marker_scale, marker_color, marker_base_y in make_infeed_motion_marker_specs():
            marker = world.scene.add(
                VisualCuboid(
                    f"{harim_root}/{marker_name}",
                    name=f"harim_{marker_name.lower()}",
                    position=np.array(marker_position, dtype=float),
                    scale=np.array(marker_scale, dtype=float),
                    color=np.array(marker_color, dtype=float),
                )
            )
            visual_parts.append(marker)
            motion_parts.append(marker)
            motion_base_y_values.append(marker_base_y)
        for carton_name, body_position, tape_position, body_scale, tape_scale, carton_base_y in make_infeed_feed_carton_specs():
            body = world.scene.add(
                VisualCuboid(
                    f"{harim_root}/{carton_name}Body",
                    name=f"harim_{carton_name.lower()}_body",
                    position=np.array(body_position, dtype=float),
                    scale=np.array(body_scale, dtype=float),
                    color=CARTON_BODY_COLOR,
                )
            )
            tape = world.scene.add(
                VisualCuboid(
                    f"{harim_root}/{carton_name}TopTape",
                    name=f"harim_{carton_name.lower()}_top_tape",
                    position=np.array(tape_position, dtype=float),
                    scale=np.array(tape_scale, dtype=float),
                    color=CARTON_TAPE_COLOR,
                )
            )
            visual_parts.extend([body, tape])
            feed_carton_part_pairs.append((body, tape))
            feed_carton_base_y_values.append(carton_base_y)
        for rail_idx, x_offset in enumerate(INFEED_GUIDE_RAIL_X_OFFSETS):
            visual_parts.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/InfeedGuideRail_{rail_idx}",
                        name=f"harim_infeed_guide_rail_{rail_idx}",
                        position=np.array(
                            [
                                x_offset,
                                INFEED_CONVEYOR_CENTER_Y,
                                INFEED_CONVEYOR_TOP_Z + INFEED_GUIDE_RAIL_SCALE[2] * 0.5,
                            ],
                            dtype=float,
                        ),
                        scale=INFEED_GUIDE_RAIL_SCALE,
                        color=rail_color,
                    )
                )
            )
        for roller_idx, y_offset in enumerate(INFEED_ROLLER_Y_OFFSETS):
            visual_parts.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/InfeedRoller_{roller_idx}",
                        name=f"harim_infeed_roller_{roller_idx}",
                        position=np.array(
                            [
                                0.0,
                                INFEED_CONVEYOR_CENTER_Y + y_offset,
                                INFEED_CONVEYOR_TOP_Z + 0.006,
                            ],
                            dtype=float,
                        ),
                        scale=np.array([INFEED_CONVEYOR_WIDTH * 0.92, 0.028, 0.014], dtype=float),
                        color=roller_color,
                    )
                )
            )
        visual_parts.append(
            world.scene.add(
                VisualCuboid(
                    f"{harim_root}/InfeedStopLine",
                    name="harim_infeed_stop_line",
                    position=np.array([0.0, INFEED_STOP_LINE_Y, INFEED_CONVEYOR_TOP_Z + 0.014], dtype=float),
                    scale=np.array([INFEED_CONVEYOR_WIDTH * 0.88, 0.018, 0.010], dtype=float),
                    color=beam_color,
                )
            )
        )
        for sensor_idx, x_offset in enumerate((-INFEED_CONVEYOR_WIDTH * 0.58, INFEED_CONVEYOR_WIDTH * 0.58)):
            visual_parts.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/InfeedPhotoEye_{sensor_idx}",
                        name=f"harim_infeed_photo_eye_{sensor_idx}",
                        position=np.array([x_offset, INFEED_STOP_LINE_Y, INFEED_CONVEYOR_TOP_Z + 0.10], dtype=float),
                        scale=np.array([0.045, 0.045, 0.12], dtype=float),
                        color=sensor_color,
                    )
                )
            )
        visual_parts.append(
            world.scene.add(
                VisualCuboid(
                    f"{harim_root}/InfeedPhotoEyeBeam",
                    name="harim_infeed_photo_eye_beam",
                    position=np.array([0.0, INFEED_STOP_LINE_Y, INFEED_CONVEYOR_TOP_Z + 0.115], dtype=float),
                    scale=np.array([INFEED_CONVEYOR_WIDTH * 0.80, 0.010, 0.010], dtype=float),
                    color=beam_color,
                )
            )
        )
        return (
            visual_parts,
            InfeedConveyorMotionController(motion_parts, motion_base_y_values),
            InfeedFeedCartonMotionController(feed_carton_part_pairs, feed_carton_base_y_values),
        )

    _infeed_conveyor_visuals, infeed_motion_controller, infeed_feed_carton_controller = create_infeed_conveyor_visual()

    def create_completion_signal():
        signal_x = args.pickup_x + 0.92
        signal_y = args.pickup_y - 0.92
        base_z = WORLD_FLOOR_Z + 0.04
        post_center_z = WORLD_FLOOR_Z + 0.48
        housing_center_z = WORLD_FLOOR_Z + 0.96
        world.scene.add(
            VisualCuboid(
                f"{harim_root}/StackCompleteSignalBase",
                name="harim_stack_complete_signal_base",
                position=np.array([signal_x, signal_y, base_z], dtype=float),
                scale=np.array([0.28, 0.22, 0.08]),
                color=np.array([0.08, 0.08, 0.08]),
            )
        )
        world.scene.add(
            VisualCuboid(
                f"{harim_root}/StackCompleteSignalPost",
                name="harim_stack_complete_signal_post",
                position=np.array([signal_x, signal_y, post_center_z], dtype=float),
                scale=np.array([0.045, 0.045, 0.86]),
                color=np.array([0.12, 0.12, 0.12]),
            )
        )
        world.scene.add(
            VisualCuboid(
                f"{harim_root}/StackCompleteSignalHousing",
                name="harim_stack_complete_signal_housing",
                position=np.array([signal_x, signal_y, housing_center_z], dtype=float),
                scale=np.array([0.18, 0.13, 0.34]),
                color=np.array([0.05, 0.05, 0.05]),
            )
        )
        red_light = world.scene.add(
            VisualSphere(
                f"{harim_root}/StackCompleteSignalRed",
                name="harim_stack_complete_signal_red",
                position=np.array([signal_x, signal_y - 0.072, WORLD_FLOOR_Z + 1.04], dtype=float),
                radius=0.055,
                color=np.array([0.85, 0.05, 0.04]),
            )
        )
        green_light = world.scene.add(
            VisualSphere(
                f"{harim_root}/StackCompleteSignalGreen",
                name="harim_stack_complete_signal_green",
                position=np.array([signal_x, signal_y - 0.072, WORLD_FLOOR_Z + 0.88], dtype=float),
                radius=0.055,
                visible=False,
                color=np.array([0.08, 0.78, 0.18]),
            )
        )
        return CompletionSignalController(red_light=red_light, green_light=green_light)

    completion_signal = create_completion_signal()

    def create_floor_markings():
        marking_parts = []

        def add_zone_outline(prefix, center_x, center_y, color):
            size_x, size_y = WORK_ZONE_MARKING_SIZE
            edge = WORK_ZONE_MARKING_EDGE_WIDTH
            for edge_name, y_offset in (("Front", size_y * 0.5), ("Back", -size_y * 0.5)):
                marking_parts.append(
                    world.scene.add(
                        VisualCuboid(
                            f"{harim_root}/{prefix}Zone{edge_name}",
                            name=f"harim_{prefix.lower()}_zone_{edge_name.lower()}",
                            position=np.array([center_x, center_y + y_offset, FLOOR_MARKING_Z], dtype=float),
                            scale=np.array([size_x, edge, FLOOR_MARKING_THICKNESS], dtype=float),
                            color=color,
                        )
                    )
                )
            for edge_name, x_offset in (("Left", -size_x * 0.5), ("Right", size_x * 0.5)):
                marking_parts.append(
                    world.scene.add(
                        VisualCuboid(
                            f"{harim_root}/{prefix}Zone{edge_name}",
                            name=f"harim_{prefix.lower()}_zone_{edge_name.lower()}",
                            position=np.array([center_x + x_offset, center_y, FLOOR_MARKING_Z], dtype=float),
                            scale=np.array([edge, size_y, FLOOR_MARKING_THICKNESS], dtype=float),
                            color=color,
                        )
                    )
                )

        path_center_x = (args.pickup_x + args.drop_x) * 0.5
        path_center_y = (args.pickup_y + args.drop_y) * 0.5
        path_length = max(abs(args.drop_x - args.pickup_x), 0.1)
        marking_parts.append(
            world.scene.add(
                VisualCuboid(
                    f"{harim_root}/AmrPathCenterLine",
                    name="harim_amr_path_center_line",
                    position=np.array([path_center_x, path_center_y, FLOOR_MARKING_Z], dtype=float),
                    scale=np.array([path_length, AMR_PATH_MARKING_WIDTH, FLOOR_MARKING_THICKNESS], dtype=float),
                    color=AMR_PATH_MARKING_COLOR,
                )
            )
        )
        add_zone_outline("Pickup", args.pickup_x, args.pickup_y, PICKUP_ZONE_MARKING_COLOR)
        add_zone_outline("Drop", args.drop_x, args.drop_y, DROP_ZONE_MARKING_COLOR)
        return marking_parts

    create_floor_markings()

    def create_pickup_dock_alignment_visual():
        dock_parts = []
        for part_name, _role, position, scale, color in make_pickup_dock_alignment_specs(
            args.pickup_x,
            args.pickup_y,
        ):
            dock_parts.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/{part_name}",
                        name=f"harim_{part_name.lower()}",
                        position=np.array(position, dtype=float),
                        scale=np.array(scale, dtype=float),
                        color=np.array(color, dtype=float),
                    )
                )
            )
        return dock_parts

    create_pickup_dock_alignment_visual()

    def create_amr_route_guard_visuals():
        route_guard_parts = []
        for part_name, _role, position, scale, color in make_amr_route_guard_specs(
            args.pickup_x,
            args.pickup_y,
            args.drop_x,
            args.drop_y,
        ):
            route_guard_parts.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/{part_name}",
                        name=f"harim_{part_name.lower()}",
                        position=np.array(position, dtype=float),
                        scale=np.array(scale, dtype=float),
                        color=np.array(color, dtype=float),
                    )
                )
            )
        return route_guard_parts

    create_amr_route_guard_visuals()

    def create_safety_fence_visual():
        fence_parts = []
        for part_idx, (part_name, position, scale, color) in enumerate(make_safety_fence_specs(args.pickup_y)):
            fence_parts.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/{part_name}",
                        name=f"harim_safety_fence_{part_idx}",
                        position=position,
                        scale=scale,
                        color=color,
                    )
                )
            )
        return fence_parts

    create_safety_fence_visual()

    iw_hub_usd = assets_root + "/Isaac/Samples/AnimRobot/iw_hub.usd"
    add_reference_to_stage(iw_hub_usd, f"{harim_root}/iw_hub")
    wait_for_stage_loading(simulation_app, usd_context, "iw_hub")
    amr = SingleXFormPrim(f"{harim_root}/iw_hub", name="harim_iw_hub")
    amr_lift_path = f"{harim_root}/iw_hub/chassis/lift"
    amr_lift = None
    amr_lift_prim = get_prim_at_path(amr_lift_path) if is_prim_path_valid(amr_lift_path) else None
    amr_lift_xformable = UsdGeom.Xformable(amr_lift_prim) if amr_lift_prim is not None else None
    if (
        amr_lift_prim is not None
        and amr_lift_prim.IsValid()
        and amr_lift_xformable is not None
        and amr_lift_xformable.GetPrim().IsValid()
    ):
        amr_lift = SingleXFormPrim(amr_lift_path, name="harim_iw_hub_asset_lift", reset_xform_properties=False)
        print(f"[HarimDemo] using iw_hub lift prim: {amr_lift_path}")
    else:
        print(f"[HarimDemo] xformable iw_hub lift prim not found, using visual lift plate only: {amr_lift_path}")

    def create_amr_safety_visuals():
        safety_parts = []
        safety_offsets = []
        safety_roles = []
        start_pose = np.array([args.pickup_x + AMR_START_STANDOFF, args.pickup_y, args.amr_z], dtype=float)
        for part_idx, (part_name, shape, offset, size, color) in enumerate(make_amr_safety_visual_specs()):
            prim_path = f"{harim_root}/{part_name}"
            object_name = f"harim_amr_safety_{part_idx}"
            position = start_pose + offset
            if shape == "sphere":
                part = world.scene.add(
                    VisualSphere(
                        prim_path,
                        name=object_name,
                        position=position,
                        radius=float(size),
                        color=color,
                    )
                )
            else:
                part = world.scene.add(
                    VisualCuboid(
                        prim_path,
                        name=object_name,
                        position=position,
                        scale=size,
                        color=color,
                    )
                )
            safety_parts.append(part)
            safety_offsets.append(offset)
            safety_roles.append(get_amr_safety_visual_role(part_name))
        return safety_parts, safety_offsets, safety_roles

    amr_safety_parts, amr_safety_offsets, amr_safety_roles = create_amr_safety_visuals()

    def create_amr_drive_visuals():
        drive_parts = []
        drive_offsets = []
        start_pose = np.array([args.pickup_x + AMR_START_STANDOFF, args.pickup_y, args.amr_z], dtype=float)
        for part_idx, (part_name, offset, scale, color) in enumerate(make_amr_drive_visual_specs()):
            drive_parts.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/{part_name}",
                        name=f"harim_amr_drive_{part_idx}",
                        position=start_pose + offset,
                        scale=scale,
                        color=color,
                    )
                )
            )
            drive_offsets.append(offset)
        return drive_parts, drive_offsets

    amr_drive_parts, amr_drive_offsets = create_amr_drive_visuals()

    def create_amr_lift_guide_visuals():
        guide_parts = []
        guide_offsets = []
        start_pose = np.array([args.pickup_x + AMR_START_STANDOFF, args.pickup_y, args.amr_z], dtype=float)
        for guide_idx, (part_name, offset, scale, color) in enumerate(make_amr_lift_guide_visual_specs()):
            guide_parts.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/{part_name}",
                        name=f"harim_amr_lift_guide_{guide_idx}",
                        position=start_pose + offset,
                        scale=scale,
                        color=color,
                    )
                )
            )
            guide_offsets.append(offset)
        return guide_parts, guide_offsets

    amr_lift_guide_parts, amr_lift_guide_offsets = create_amr_lift_guide_visuals()

    def create_drop_slide_workstation():
        workstation_parts = []
        rail_color = np.array([0.18, 0.20, 0.21])
        roller_color = np.array([0.62, 0.66, 0.68])
        leg_color = np.array([0.12, 0.13, 0.14])
        leg_height = DROP_WORKSTATION_Z - WORLD_FLOOR_Z
        leg_center_z = WORLD_FLOOR_Z + leg_height * 0.5

        for side_idx, y_offset in enumerate(DROP_SLIDE_LANE_Y_OFFSETS):
            workstation_parts.append(
                world.scene.add(
                    FixedCuboid(
                        f"{harim_root}/DropSlideRail_{side_idx}",
                        name=f"harim_drop_slide_rail_{side_idx}",
                        position=np.array([args.drop_x, args.drop_y + y_offset, DROP_WORKSTATION_Z], dtype=float),
                        scale=DROP_SLIDE_RAIL_SCALE,
                        color=rail_color,
                    )
                )
            )
            for roller_idx, x_offset in enumerate(DROP_SLIDE_ROLLER_X_OFFSETS):
                workstation_parts.append(
                    world.scene.add(
                        VisualCuboid(
                            f"{harim_root}/DropSlideRoller_{side_idx}_{roller_idx}",
                            name=f"harim_drop_slide_roller_{side_idx}_{roller_idx}",
                            position=np.array(
                                [args.drop_x + x_offset, args.drop_y + y_offset, DROP_SLIDE_ROLLER_CENTER_Z],
                                dtype=float,
                            ),
                            scale=DROP_SLIDE_ROLLER_SCALE,
                            color=roller_color,
                        )
                    )
                )
            for leg_idx, x_offset in enumerate(DROP_SLIDE_LEG_X_OFFSETS):
                workstation_parts.append(
                    world.scene.add(
                        FixedCuboid(
                            f"{harim_root}/DropSlideLeg_{side_idx}_{leg_idx}",
                            name=f"harim_drop_slide_leg_{side_idx}_{leg_idx}",
                            position=np.array(
                                [args.drop_x + x_offset, args.drop_y + y_offset, leg_center_z],
                                dtype=float,
                            ),
                            scale=np.array([0.10, 0.10, leg_height]),
                            color=leg_color,
                        )
                    )
                )
            workstation_parts.append(
                world.scene.add(
                    FixedCuboid(
                        f"{harim_root}/DropSlideTopSupport_{side_idx}",
                        name=f"harim_drop_slide_top_support_{side_idx}",
                        position=np.array(
                            [args.drop_x, args.drop_y + y_offset, DROP_SLIDE_TOP_SUPPORT_CENTER_Z],
                            dtype=float,
                        ),
                        scale=DROP_SLIDE_TOP_SUPPORT_SCALE,
                        visible=False,
                        color=roller_color,
                    )
                )
            )
        return workstation_parts

    def create_drop_dock_alignment_visual():
        dock_parts = []
        stop_color = np.array([0.82, 0.68, 0.20], dtype=float)
        post_color = np.array([0.12, 0.14, 0.16], dtype=float)
        cap_color = np.array([0.95, 0.72, 0.12], dtype=float)
        for stop_idx, y_offset in enumerate(DROP_DOCK_STOP_Y_OFFSETS):
            dock_parts.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/DropDockStopBlock_{stop_idx}",
                        name=f"harim_drop_dock_stop_block_{stop_idx}",
                        position=np.array(
                            [
                                args.drop_x + DROP_DOCK_STOP_X_OFFSET,
                                args.drop_y + y_offset,
                                DROP_DOCK_STOP_CENTER_Z,
                            ],
                            dtype=float,
                        ),
                        scale=DROP_DOCK_STOP_BLOCK_SCALE,
                        color=stop_color,
                    )
                )
            )
        for post_idx, x_offset in enumerate(DROP_DOCK_GUIDE_POST_X_OFFSETS):
            for side_idx, y_offset in enumerate(DROP_DOCK_GUIDE_POST_Y_OFFSETS):
                dock_parts.append(
                    world.scene.add(
                        VisualCuboid(
                            f"{harim_root}/DropDockLocatorPost_{post_idx}_{side_idx}",
                            name=f"harim_drop_dock_locator_post_{post_idx}_{side_idx}",
                            position=np.array(
                                [
                                    args.drop_x + x_offset,
                                    args.drop_y + y_offset,
                                    DROP_DOCK_GUIDE_POST_CENTER_Z,
                                ],
                                dtype=float,
                            ),
                            scale=DROP_DOCK_GUIDE_POST_SCALE,
                            color=post_color,
                        )
                    )
                )
                dock_parts.append(
                    world.scene.add(
                        VisualCuboid(
                            f"{harim_root}/DropDockLocatorCap_{post_idx}_{side_idx}",
                            name=f"harim_drop_dock_locator_cap_{post_idx}_{side_idx}",
                            position=np.array(
                                [
                                    args.drop_x + x_offset,
                                    args.drop_y + y_offset,
                                    DROP_DOCK_GUIDE_POST_CENTER_Z + DROP_DOCK_GUIDE_POST_SCALE[2] * 0.5 + 0.025,
                                ],
                                dtype=float,
                            ),
                            scale=np.array([0.10, 0.10, 0.05], dtype=float),
                            color=cap_color,
                        )
                    )
                )
        return dock_parts

    create_drop_slide_workstation()
    create_drop_dock_alignment_visual()

    lift_plate_parts = []
    for fork_idx, fork_offset in enumerate(LIFT_FORK_OFFSETS):
        lift_plate_parts.append(
            world.scene.add(
                VisualCuboid(
                    f"{harim_root}/IwHubLiftFork_{fork_idx}",
                    name=f"harim_iw_hub_lift_fork_{fork_idx}",
                    position=np.array(
                        [
                            args.pickup_x + AMR_START_STANDOFF + fork_offset[0],
                            args.pickup_y + fork_offset[1],
                            args.amr_z + AMR_LIFT_PLATE_OFFSET_Z + fork_offset[2],
                        ]
                    ),
                    scale=LIFT_FORK_SCALE,
                    visible=True,
                    color=np.array([0.10, 0.12, 0.13]),
                )
            )
        )
    lift_plate = lift_plate_parts[0]

    pallet_parts = []
    pallet_part_offsets = list(PALLET_PART_OFFSETS)
    load_restraint_parts = []
    plank_color = np.array([0.62, 0.44, 0.23])
    groove_color = np.array([0.30, 0.20, 0.10])
    block_color = np.array([0.42, 0.29, 0.15])
    pallet_parts.append(
        world.scene.add(
            FixedCuboid(
                f"{harim_root}/PalletDeck_0",
                name="harim_pallet_connected_top_deck",
                scale=PALLET_DECK_SCALE,
                color=plank_color,
            )
        )
    )
    for idx in range(2):
        pallet_parts.append(
            world.scene.add(
                FixedCuboid(
                    f"{harim_root}/PalletRunner_{idx}",
                    name=f"harim_pallet_side_runner_{idx}",
                    scale=PALLET_RUNNER_SCALE,
                    color=block_color,
                )
            )
        )
    for idx in range(6):
        pallet_parts.append(
            world.scene.add(
                FixedCuboid(
                    f"{harim_root}/PalletBlock_{idx}",
                    name=f"harim_pallet_block_{idx}",
                    scale=PALLET_BLOCK_SCALE,
                    color=block_color,
                )
            )
        )
    for idx in range(2):
        pallet_parts.append(
            world.scene.add(
                VisualCuboid(
                    f"{harim_root}/PalletGroove_{idx}",
                    name=f"harim_pallet_top_groove_{idx}",
                    scale=PALLET_GROOVE_SCALE,
                    color=groove_color,
                )
            )
        )
    pallet_parts.append(
        world.scene.add(
            FixedCuboid(
                f"{harim_root}/PalletTopSupport",
                name="harim_pallet_top_support",
                scale=PALLET_TOP_SUPPORT_SCALE,
                visible=False,
                color=plank_color,
            )
        )
    )
    for strap_idx, (strap_name, strap_offset, strap_scale) in enumerate(
        compute_load_restraint_specs(stack_coordinates, args.pickup_x, args.pickup_y)
    ):
        strap = world.scene.add(
            VisualCuboid(
                f"{harim_root}/{strap_name}",
                name=f"harim_load_restraint_{strap_idx}",
                scale=strap_scale,
                visible=False,
                color=LOAD_RESTRAINT_COLOR,
            )
        )
        pallet_parts.append(strap)
        pallet_part_offsets.append(strap_offset)
        load_restraint_parts.append(strap)

    class SelfTestBinState:
        def __init__(self, bin_obj):
            self.bin_obj = bin_obj

    self_test_payload = []
    if args.self_test_force_stack_complete:
        for idx, coord in enumerate(stack_coordinates[: max(1, min(4, len(stack_coordinates)))]):
            self_test_payload.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/SelfTestPayload_{idx}",
                        name=f"harim_self_test_payload_{idx}",
                        position=np.array(coord, dtype=float),
                        scale=np.array([0.18, 0.26, 0.12]),
                        color=np.array([0.78, 0.42, 0.16]),
                    )
                )
            )

    orchestrator = HarimTransferOrchestrator(
        world=world,
        context=decider_network.context,
        task=task,
        amr_prim=amr,
        amr_lift_prim=amr_lift,
        amr_safety_parts=amr_safety_parts,
        amr_safety_offsets=amr_safety_offsets,
        amr_safety_roles=amr_safety_roles,
        amr_drive_parts=amr_drive_parts,
        amr_drive_offsets=amr_drive_offsets,
        amr_lift_guide_parts=amr_lift_guide_parts,
        amr_lift_guide_offsets=amr_lift_guide_offsets,
        lift_plate=lift_plate,
        lift_plate_parts=lift_plate_parts,
        pallet_parts=pallet_parts,
        pallet_part_offsets=pallet_part_offsets,
        load_restraint_parts=load_restraint_parts,
        stack_coordinates=stack_coordinates,
        args=args,
        completion_signal=completion_signal,
        camera_director=set_story_camera,
    )

    world.reset()
    world.play()
    decider_network.context.stack_coordinates = clone_stack_coordinates(stack_coordinates)
    decider_network.context.demo_stack_coordinates = clone_stack_coordinates(stack_coordinates)
    decider_network.context.demo_sim_time = 0.0
    decider_network.context.demo_motion_phase = "initializing"
    decider_network.context.demo_max_pre_grip_offset = 0.0
    decider_network.context.demo_max_return_ready_error = 0.0
    decider_network.context.demo_max_release_drift = 0.0
    decider_network.context.demo_max_release_retreat_lift = 0.0
    decider_network.context.demo_scripted_place_bin = None
    decider_network.context.demo_scripted_place_count = 0
    decider_network.context.demo_max_scripted_place_error = 0.0
    decider_network.context.demo_max_release_separation = 0.0
    decider_network.context.demo_max_release_vertical_clearance = 0.0
    decider_network.context.demo_release_gripper_samples = 0
    decider_network.context.demo_release_gripper_not_open_samples = 0
    decider_network.context.demo_release_gripped_object_max = 0
    decider_network.context.demo_release_gripper_probe_failures = 0
    decider_network.context.demo_attached_grasp_sample_count = 0
    decider_network.context.demo_max_attached_grasp_error = 0.0
    decider_network.context.demo_max_attached_bin_sync_gap = 0.0
    decider_network.context.demo_measured_arm_fk_sample_count = 0
    decider_network.context.demo_measured_arm_fk_fallback_count = 0
    decider_network.context.demo_joint_settle_count = 0
    decider_network.context.demo_infeed_active_bin = None
    decider_network.context.demo_active_bin_conveyor_approach_count = 0
    decider_network.context.demo_active_bin_conveyor_completed_count = 0
    decider_network.context.demo_active_bin_conveyor_observed_travel = 0.0
    decider_network.context.demo_active_bin_conveyor_final_error = 0.0
    decider_network.context.demo_active_bin_conveyor_lateral_error = 0.0
    orchestrator.reset_visual_state()
    gif_recorder = DemoGifRecorder(
        enabled=True,
        output_dir=args.gif_output_dir,
        frame_stride=args.gif_frame_stride,
        max_frames=args.gif_max_frames,
    )
    motion_continuity = MotionContinuityTracker()
    demo_frame_index = 0
    review_gif_path = None

    def save_review_gif():
        nonlocal review_gif_path
        if review_gif_path is None:
            review_gif_path = gif_recorder.save()
        return review_gif_path

    def force_self_test_stack_complete():
        if not args.self_test_force_stack_complete:
            return
        if self_test_payload:
            decider_network.context.stacked_bins = [SelfTestBinState(item) for item in self_test_payload]
            decider_network.context.stack_coordinates = [
                np.array(item.get_world_pose()[0], dtype=float) for item in self_test_payload
            ]

    def sample_motion_continuity():
        phase = getattr(decider_network.context, "demo_motion_phase", None)
        motion_continuity.sample("amr", "iw_hub", orchestrator.get_amr_position(), phase, demo_frame_index)
        try:
            motion_continuity.sample(
                "arm_ee",
                "ur10_ee",
                get_measured_arm_fk_p(decider_network.context),
                phase,
                demo_frame_index,
            )
        except Exception:
            pass

        context = decider_network.context
        active_bin = (
            getattr(context, "demo_infeed_active_bin", None)
            or getattr(context, "demo_pre_grip_bin", None)
            or getattr(context, "demo_carried_bin", None)
            or getattr(context, "demo_released_bin", None)
            or getattr(context, "active_bin", None)
        )
        if active_bin is not None and active_bin not in getattr(context, "stacked_bins", []):
            bin_obj = getattr(active_bin, "bin_obj", None)
            if bin_obj is not None:
                try:
                    position, _orientation = bin_obj.get_world_pose()
                    motion_group = "active_bin"
                    if getattr(context, "demo_released_bin", None) is active_bin:
                        motion_group = "released_bin"
                    elif getattr(context, "demo_scripted_place_bin", None) is active_bin:
                        motion_group = "scripted_place_bin"
                    elif (
                        getattr(active_bin, "demo_attached", False)
                    ):
                        motion_group = "attached_bin"
                    motion_continuity.sample(
                        motion_group,
                        getattr(bin_obj, "name", motion_group),
                        position,
                        phase,
                        demo_frame_index,
                    )
                except Exception:
                    pass

        if getattr(orchestrator, "carrying", False):
            for item in getattr(orchestrator, "attached_items", []):
                try:
                    position, _orientation = item.get_world_pose()
                    motion_continuity.sample(
                        "carried_payload",
                        getattr(item, "name", str(id(item))),
                        position,
                        phase,
                        demo_frame_index,
                    )
                except Exception:
                    pass
            for part in getattr(orchestrator, "pallet_parts", []):
                try:
                    position, _orientation = part.get_world_pose()
                    motion_continuity.sample(
                        "carried_pallet",
                        getattr(part, "name", str(id(part))),
                        position,
                        phase,
                        demo_frame_index,
                    )
                except Exception:
                    pass

    def step_demo_frame():
        nonlocal demo_frame_index
        physics_dt = world.get_physics_dt()
        decider_network.context.demo_sim_time = getattr(decider_network.context, "demo_sim_time", 0.0) + physics_dt
        hold_demo_released_bin_at_target(decider_network.context)
        infeed_motion_controller.update(decider_network.context.demo_sim_time)
        infeed_feed_carton_controller.update(decider_network.context.demo_sim_time)
        world.step(render=not args.headless)
        sync_demo_attached_bin(decider_network.context)
        hold_demo_released_bin_at_target(decider_network.context)
        force_self_test_stack_complete()
        orchestrator.step(physics_dt)
        sample_motion_continuity()
        gif_recorder.maybe_capture(demo_frame_index, orchestrator, decider_network.context, args)
        demo_frame_index += 1

    self_test_failure_message = None
    try:
        if args.self_test_frames > 0:
            for _frame_count in range(args.self_test_frames):
                step_demo_frame()
            gif_recorder.maybe_capture(demo_frame_index, orchestrator, decider_network.context, args, force=True)
            placed_count = len(getattr(decider_network.context, "stacked_bins", []))
            transfer_cycles = getattr(orchestrator, "completed_cycles", 0)
            self_test_failures = []
            if args.self_test_min_placed_bins > 0 and placed_count < args.self_test_min_placed_bins:
                self_test_failures.append(
                    f"UR10 placed {placed_count} bins, expected at least {args.self_test_min_placed_bins}"
                )
            if args.self_test_min_transfer_cycles > 0 and transfer_cycles < args.self_test_min_transfer_cycles:
                self_test_failures.append(
                    f"AMR completed {transfer_cycles} transfer cycles, "
                    f"expected at least {args.self_test_min_transfer_cycles}"
                )
            max_pre_grip_offset = float(getattr(decider_network.context, "demo_max_pre_grip_offset", 0.0))
            if args.self_test_max_pre_grip_offset > 0 and max_pre_grip_offset > args.self_test_max_pre_grip_offset:
                self_test_failures.append(
                    f"max pre-grip offset {max_pre_grip_offset:.4f} m exceeded "
                    f"{args.self_test_max_pre_grip_offset:.4f} m"
                )
            max_return_ready_error = float(getattr(decider_network.context, "demo_max_return_ready_error", 0.0))
            if (
                args.self_test_max_return_ready_error > 0
                and max_return_ready_error > args.self_test_max_return_ready_error
            ):
                self_test_failures.append(
                    f"max return-ready error {max_return_ready_error:.4f} m exceeded "
                    f"{args.self_test_max_return_ready_error:.4f} m"
                )
            max_release_drift = float(getattr(decider_network.context, "demo_max_release_drift", 0.0))
            if args.self_test_max_release_drift > 0 and max_release_drift > args.self_test_max_release_drift:
                self_test_failures.append(
                    f"max release drift {max_release_drift:.4f} m exceeded "
                    f"{args.self_test_max_release_drift:.4f} m"
                )
            max_release_retreat_lift = float(getattr(decider_network.context, "demo_max_release_retreat_lift", 0.0))
            if (
                args.self_test_min_release_retreat_lift > 0
                and max_release_retreat_lift < args.self_test_min_release_retreat_lift
            ):
                self_test_failures.append(
                    f"release retreat lift {max_release_retreat_lift:.4f} m was below "
                    f"{args.self_test_min_release_retreat_lift:.4f} m"
                )
            scripted_place_count = int(getattr(decider_network.context, "demo_scripted_place_count", 0))
            if args.self_test_min_scripted_place_count > 0 and scripted_place_count < args.self_test_min_scripted_place_count:
                self_test_failures.append(
                    f"scripted place count {scripted_place_count} was below "
                    f"{args.self_test_min_scripted_place_count}"
                )
            max_scripted_place_error = float(getattr(decider_network.context, "demo_max_scripted_place_error", 0.0))
            if (
                args.self_test_max_scripted_place_error > 0
                and max_scripted_place_error > args.self_test_max_scripted_place_error
            ):
                self_test_failures.append(
                    f"scripted place error {max_scripted_place_error:.4f} m exceeded "
                    f"{args.self_test_max_scripted_place_error:.4f} m"
                )
            max_release_separation = float(getattr(decider_network.context, "demo_max_release_separation", 0.0))
            max_release_vertical_clearance = float(
                getattr(decider_network.context, "demo_max_release_vertical_clearance", 0.0)
            )
            if args.self_test_min_release_separation > 0 and max_release_separation < args.self_test_min_release_separation:
                self_test_failures.append(
                    f"release separation {max_release_separation:.4f} m was below "
                    f"{args.self_test_min_release_separation:.4f} m"
                )
            if (
                args.self_test_min_release_vertical_clearance > 0
                and max_release_vertical_clearance < args.self_test_min_release_vertical_clearance
            ):
                self_test_failures.append(
                    f"release vertical clearance {max_release_vertical_clearance:.4f} m was below "
                    f"{args.self_test_min_release_vertical_clearance:.4f} m"
                )
            release_gripper_samples = int(getattr(decider_network.context, "demo_release_gripper_samples", 0))
            release_gripper_not_open = int(
                getattr(decider_network.context, "demo_release_gripper_not_open_samples", 0)
            )
            release_gripped_object_max = int(
                getattr(decider_network.context, "demo_release_gripped_object_max", 0)
            )
            release_gripper_probe_failures = int(
                getattr(decider_network.context, "demo_release_gripper_probe_failures", 0)
            )
            attached_grasp_sample_count = int(
                getattr(decider_network.context, "demo_attached_grasp_sample_count", 0)
            )
            max_attached_grasp_error = float(getattr(decider_network.context, "demo_max_attached_grasp_error", 0.0))
            joint_settle_count = int(getattr(decider_network.context, "demo_joint_settle_count", 0))
            if args.self_test_require_gripper_open_after_release:
                if placed_count > 0 and release_gripper_samples <= 0:
                    self_test_failures.append("release gripper state was not sampled after placed bins")
                if release_gripper_not_open > 0:
                    self_test_failures.append(
                        f"release gripper was not open for {release_gripper_not_open} sampled frames"
                    )
                if release_gripped_object_max > 0:
                    self_test_failures.append(
                        f"release gripper still reported {release_gripped_object_max} gripped objects"
                    )
                if release_gripper_probe_failures > 0:
                    self_test_failures.append(
                        f"release gripper state probe failed {release_gripper_probe_failures} times"
                    )
            if (
                args.self_test_min_attached_grasp_sample_count > 0
                and attached_grasp_sample_count < args.self_test_min_attached_grasp_sample_count
            ):
                self_test_failures.append(
                    f"attached grasp sample count {attached_grasp_sample_count} was below "
                    f"{args.self_test_min_attached_grasp_sample_count}"
                )
            if (
                args.self_test_max_attached_grasp_error > 0
                and max_attached_grasp_error > args.self_test_max_attached_grasp_error
            ):
                self_test_failures.append(
                    f"attached bin grasp error {max_attached_grasp_error:.4f} m exceeded "
                    f"{args.self_test_max_attached_grasp_error:.4f} m"
                )
            stack_geometry = compute_stack_geometry_metrics(
                stack_coordinates,
                args.stack_cols,
                args.stack_rows,
                args.stack_layers,
            )
            max_stack_lateral_gap = stack_geometry["max_stack_lateral_gap"]
            min_stack_lateral_gap = stack_geometry["min_stack_lateral_gap"]
            max_stack_support_gap = stack_geometry["max_stack_support_gap"]
            min_stack_support_gap = stack_geometry["min_stack_support_gap"]
            if args.self_test_max_stack_lateral_gap > 0:
                if max_stack_lateral_gap > args.self_test_max_stack_lateral_gap:
                    self_test_failures.append(
                        f"max stack lateral gap {max_stack_lateral_gap:.4f} m exceeded "
                        f"{args.self_test_max_stack_lateral_gap:.4f} m"
                    )
                if min_stack_lateral_gap < -0.005:
                    self_test_failures.append(
                        f"stack lateral overlap {-min_stack_lateral_gap:.4f} m exceeded 0.0050 m"
                    )
            if args.self_test_max_stack_support_gap > 0:
                if max_stack_support_gap > args.self_test_max_stack_support_gap:
                    self_test_failures.append(
                        f"max stack support gap {max_stack_support_gap:.4f} m exceeded "
                        f"{args.self_test_max_stack_support_gap:.4f} m"
                    )
                if min_stack_support_gap < -0.005:
                    self_test_failures.append(
                        f"stack vertical overlap {-min_stack_support_gap:.4f} m exceeded 0.0050 m"
                    )
            stack_footprint = compute_stack_pallet_footprint_metrics(
                stack_coordinates,
                args.pickup_x,
                args.pickup_y,
            )
            min_stack_pallet_margin = stack_footprint["min_stack_pallet_margin"]
            max_stack_pallet_overhang = stack_footprint["max_stack_pallet_overhang"]
            if (
                args.self_test_min_stack_pallet_margin > 0
                and min_stack_pallet_margin < args.self_test_min_stack_pallet_margin
            ):
                self_test_failures.append(
                    f"stack pallet margin {min_stack_pallet_margin:.4f} m was below "
                    f"{args.self_test_min_stack_pallet_margin:.4f} m"
                )
            load_restraint_metrics = compute_load_restraint_metrics(
                stack_coordinates,
                args.pickup_x,
                args.pickup_y,
            )
            load_restraint_part_count = load_restraint_metrics["load_restraint_part_count"]
            min_load_restraint_pallet_margin = load_restraint_metrics["min_load_restraint_pallet_margin"]
            max_load_restraint_pallet_overhang = load_restraint_metrics["max_load_restraint_pallet_overhang"]
            if (
                args.self_test_min_load_restraint_count > 0
                and load_restraint_part_count < args.self_test_min_load_restraint_count
            ):
                self_test_failures.append(
                    f"load restraint count {load_restraint_part_count} was below "
                    f"{args.self_test_min_load_restraint_count}"
                )
            if (
                args.self_test_min_load_restraint_pallet_margin > 0
                and min_load_restraint_pallet_margin < args.self_test_min_load_restraint_pallet_margin
            ):
                self_test_failures.append(
                    f"load restraint pallet margin {min_load_restraint_pallet_margin:.4f} m was below "
                    f"{args.self_test_min_load_restraint_pallet_margin:.4f} m"
                )
            infeed_metrics = compute_infeed_conveyor_metrics()
            infeed_conveyor_length = infeed_metrics["infeed_conveyor_length"]
            infeed_spawn_margin = infeed_metrics["infeed_spawn_margin"]
            infeed_pick_margin = infeed_metrics["infeed_pick_margin"]
            infeed_guide_clearance = infeed_metrics["infeed_guide_clearance"]
            infeed_belt_support_gap = infeed_metrics["infeed_belt_support_gap"]
            infeed_motion_marker_count = infeed_metrics["infeed_motion_marker_count"]
            infeed_motion_marker_spacing = infeed_metrics["infeed_motion_marker_spacing"]
            infeed_motion_marker_speed = infeed_metrics["infeed_motion_marker_speed"]
            infeed_feed_carton_count = infeed_metrics["infeed_feed_carton_count"]
            infeed_feed_carton_path_length = infeed_metrics["infeed_feed_carton_path_length"]
            infeed_feed_carton_speed = infeed_metrics["infeed_feed_carton_speed"]
            infeed_feed_carton_stop_clearance = infeed_metrics["infeed_feed_carton_stop_clearance"]
            infeed_feed_carton_guide_clearance = infeed_metrics["infeed_feed_carton_guide_clearance"]
            infeed_feed_carton_belt_support_gap = infeed_metrics["infeed_feed_carton_belt_support_gap"]
            infeed_motion_observed_travel = float(
                getattr(infeed_motion_controller, "max_marker_observed_travel", 0.0)
            )
            infeed_feed_carton_observed_travel = float(
                getattr(infeed_feed_carton_controller, "max_carton_observed_travel", 0.0)
            )
            if (
                args.self_test_min_infeed_conveyor_length > 0
                and infeed_conveyor_length < args.self_test_min_infeed_conveyor_length
            ):
                self_test_failures.append(
                    f"infeed conveyor length {infeed_conveyor_length:.4f} m was below "
                    f"{args.self_test_min_infeed_conveyor_length:.4f} m"
                )
            if (
                args.self_test_min_infeed_spawn_margin > 0
                and infeed_spawn_margin < args.self_test_min_infeed_spawn_margin
            ):
                self_test_failures.append(
                    f"infeed spawn margin {infeed_spawn_margin:.4f} m was below "
                    f"{args.self_test_min_infeed_spawn_margin:.4f} m"
                )
            if (
                args.self_test_min_infeed_guide_clearance > 0
                and infeed_guide_clearance < args.self_test_min_infeed_guide_clearance
            ):
                self_test_failures.append(
                    f"infeed guide clearance {infeed_guide_clearance:.4f} m was below "
                    f"{args.self_test_min_infeed_guide_clearance:.4f} m"
                )
            if args.self_test_max_infeed_belt_support_gap > 0:
                if infeed_belt_support_gap > args.self_test_max_infeed_belt_support_gap:
                    self_test_failures.append(
                        f"infeed belt support gap {infeed_belt_support_gap:.4f} m exceeded "
                        f"{args.self_test_max_infeed_belt_support_gap:.4f} m"
                    )
                if infeed_belt_support_gap < -0.005:
                    self_test_failures.append(
                        f"infeed belt overlapped carton bottom {-infeed_belt_support_gap:.4f} m exceeded 0.0050 m"
                    )
            if (
                args.self_test_min_infeed_motion_marker_count > 0
                and infeed_motion_marker_count < args.self_test_min_infeed_motion_marker_count
            ):
                self_test_failures.append(
                    f"infeed motion marker count {infeed_motion_marker_count} was below "
                    f"{args.self_test_min_infeed_motion_marker_count}"
                )
            if (
                args.self_test_min_infeed_motion_observed_travel > 0
                and infeed_motion_observed_travel < args.self_test_min_infeed_motion_observed_travel
            ):
                self_test_failures.append(
                    f"infeed motion observed travel {infeed_motion_observed_travel:.4f} m was below "
                    f"{args.self_test_min_infeed_motion_observed_travel:.4f} m"
                )
            if (
                args.self_test_min_infeed_feed_carton_count > 0
                and infeed_feed_carton_count < args.self_test_min_infeed_feed_carton_count
            ):
                self_test_failures.append(
                    f"infeed feed carton count {infeed_feed_carton_count} was below "
                    f"{args.self_test_min_infeed_feed_carton_count}"
                )
            if (
                args.self_test_min_infeed_feed_carton_observed_travel > 0
                and infeed_feed_carton_observed_travel < args.self_test_min_infeed_feed_carton_observed_travel
            ):
                self_test_failures.append(
                    f"infeed feed carton observed travel {infeed_feed_carton_observed_travel:.4f} m was below "
                    f"{args.self_test_min_infeed_feed_carton_observed_travel:.4f} m"
                )
            if (
                args.self_test_min_infeed_feed_carton_stop_clearance > 0
                and infeed_feed_carton_stop_clearance < args.self_test_min_infeed_feed_carton_stop_clearance
            ):
                self_test_failures.append(
                    f"infeed feed carton stop clearance {infeed_feed_carton_stop_clearance:.4f} m was below "
                    f"{args.self_test_min_infeed_feed_carton_stop_clearance:.4f} m"
                )
            if (
                args.self_test_min_infeed_feed_carton_guide_clearance > 0
                and infeed_feed_carton_guide_clearance < args.self_test_min_infeed_feed_carton_guide_clearance
            ):
                self_test_failures.append(
                    f"infeed feed carton guide clearance {infeed_feed_carton_guide_clearance:.4f} m was below "
                    f"{args.self_test_min_infeed_feed_carton_guide_clearance:.4f} m"
                )
            if args.self_test_max_infeed_feed_carton_belt_support_gap > 0:
                if infeed_feed_carton_belt_support_gap > args.self_test_max_infeed_feed_carton_belt_support_gap:
                    self_test_failures.append(
                        f"infeed feed carton belt support gap {infeed_feed_carton_belt_support_gap:.4f} m exceeded "
                        f"{args.self_test_max_infeed_feed_carton_belt_support_gap:.4f} m"
                    )
                if infeed_feed_carton_belt_support_gap < -0.005:
                    self_test_failures.append(
                        f"infeed feed carton overlapped belt {-infeed_feed_carton_belt_support_gap:.4f} m exceeded 0.0050 m"
                    )
            active_bin_conveyor_metrics = compute_active_bin_conveyor_metrics()
            active_bin_conveyor_travel_distance = active_bin_conveyor_metrics["active_bin_conveyor_travel_distance"]
            active_bin_conveyor_duration = active_bin_conveyor_metrics["active_bin_conveyor_duration"]
            active_bin_conveyor_nominal_speed = active_bin_conveyor_metrics["active_bin_conveyor_nominal_speed"]
            active_bin_conveyor_belt_support_gap = active_bin_conveyor_metrics[
                "active_bin_conveyor_belt_support_gap"
            ]
            active_bin_conveyor_approach_count = int(
                getattr(decider_network.context, "demo_active_bin_conveyor_approach_count", 0)
            )
            active_bin_conveyor_completed_count = int(
                getattr(decider_network.context, "demo_active_bin_conveyor_completed_count", 0)
            )
            active_bin_conveyor_observed_travel = float(
                getattr(decider_network.context, "demo_active_bin_conveyor_observed_travel", 0.0)
            )
            active_bin_conveyor_final_error = float(
                getattr(decider_network.context, "demo_active_bin_conveyor_final_error", 0.0)
            )
            active_bin_conveyor_lateral_error = float(
                getattr(decider_network.context, "demo_active_bin_conveyor_lateral_error", 0.0)
            )
            if (
                args.self_test_min_active_bin_conveyor_approach_count > 0
                and active_bin_conveyor_completed_count < args.self_test_min_active_bin_conveyor_approach_count
            ):
                self_test_failures.append(
                    f"active bin conveyor approach count {active_bin_conveyor_completed_count} was below "
                    f"{args.self_test_min_active_bin_conveyor_approach_count}"
                )
            if (
                args.self_test_min_active_bin_conveyor_observed_travel > 0
                and active_bin_conveyor_observed_travel < args.self_test_min_active_bin_conveyor_observed_travel
            ):
                self_test_failures.append(
                    f"active bin conveyor observed travel {active_bin_conveyor_observed_travel:.4f} m was below "
                    f"{args.self_test_min_active_bin_conveyor_observed_travel:.4f} m"
                )
            if (
                args.self_test_max_active_bin_conveyor_final_error > 0
                and active_bin_conveyor_final_error > args.self_test_max_active_bin_conveyor_final_error
            ):
                self_test_failures.append(
                    f"active bin conveyor final error {active_bin_conveyor_final_error:.4f} m exceeded "
                    f"{args.self_test_max_active_bin_conveyor_final_error:.4f} m"
                )
            if (
                args.self_test_max_active_bin_conveyor_lateral_error > 0
                and active_bin_conveyor_lateral_error > args.self_test_max_active_bin_conveyor_lateral_error
            ):
                self_test_failures.append(
                    f"active bin conveyor lateral error {active_bin_conveyor_lateral_error:.4f} m exceeded "
                    f"{args.self_test_max_active_bin_conveyor_lateral_error:.4f} m"
                )
            if args.self_test_max_active_bin_conveyor_belt_support_gap > 0:
                if active_bin_conveyor_belt_support_gap > args.self_test_max_active_bin_conveyor_belt_support_gap:
                    self_test_failures.append(
                        f"active bin conveyor belt support gap {active_bin_conveyor_belt_support_gap:.4f} m exceeded "
                        f"{args.self_test_max_active_bin_conveyor_belt_support_gap:.4f} m"
                    )
                if active_bin_conveyor_belt_support_gap < -0.005:
                    self_test_failures.append(
                        f"active bin conveyor overlapped belt {-active_bin_conveyor_belt_support_gap:.4f} m exceeded 0.0050 m"
                    )
            motion_continuity_sample_count = motion_continuity.sample_count("amr")
            motion_continuity_tracked_item_count = motion_continuity.tracked_item_count()
            active_bin_motion_sample_count = motion_continuity.sample_count("active_bin")
            attached_bin_motion_sample_count = motion_continuity.sample_count("attached_bin")
            scripted_place_bin_motion_sample_count = motion_continuity.sample_count("scripted_place_bin")
            released_bin_motion_sample_count = motion_continuity.sample_count("released_bin")
            carried_payload_motion_sample_count = motion_continuity.sample_count("carried_payload")
            carried_pallet_motion_sample_count = motion_continuity.sample_count("carried_pallet")
            arm_ee_motion_sample_count = motion_continuity.sample_count("arm_ee")
            max_amr_frame_displacement = motion_continuity.max_displacement("amr")
            max_arm_ee_frame_displacement = motion_continuity.max_displacement("arm_ee")
            max_active_bin_frame_displacement = motion_continuity.max_displacement("active_bin")
            max_attached_bin_frame_displacement = motion_continuity.max_displacement("attached_bin")
            max_scripted_place_bin_frame_displacement = motion_continuity.max_displacement("scripted_place_bin")
            max_released_bin_frame_displacement = motion_continuity.max_displacement("released_bin")
            max_carried_payload_frame_displacement = motion_continuity.max_displacement("carried_payload")
            max_carried_pallet_frame_displacement = motion_continuity.max_displacement("carried_pallet")
            measured_arm_fk_sample_count = int(
                getattr(decider_network.context, "demo_measured_arm_fk_sample_count", 0)
            )
            measured_arm_fk_fallback_count = int(
                getattr(decider_network.context, "demo_measured_arm_fk_fallback_count", 0)
            )
            amr_motion_detail = motion_continuity.max_detail("amr")
            arm_ee_motion_detail = motion_continuity.max_detail("arm_ee")
            active_bin_motion_detail = motion_continuity.max_detail("active_bin")
            attached_bin_motion_detail = motion_continuity.max_detail("attached_bin")
            scripted_place_bin_motion_detail = motion_continuity.max_detail("scripted_place_bin")
            released_bin_motion_detail = motion_continuity.max_detail("released_bin")
            carried_payload_motion_detail = motion_continuity.max_detail("carried_payload")
            carried_pallet_motion_detail = motion_continuity.max_detail("carried_pallet")

            def format_motion_detail(detail):
                if not detail:
                    return ""
                previous_position = np.array(detail["previous_position"], dtype=float)
                position = np.array(detail["position"], dtype=float)
                previous_text = np.array2string(previous_position, precision=3, separator=", ")
                position_text = np.array2string(position, precision=3, separator=", ")
                detail_parts = []
                if "phase" in detail:
                    detail_parts.append(f"phase={detail['phase']}")
                if "frame_index" in detail:
                    detail_parts.append(f"frame={detail['frame_index']}")
                detail_suffix = f"; {'; '.join(detail_parts)}" if detail_parts else ""
                return f" ({detail['item_id']}: {previous_text} -> {position_text}{detail_suffix})"

            if (
                args.self_test_min_motion_continuity_sample_count > 0
                and motion_continuity_sample_count < args.self_test_min_motion_continuity_sample_count
            ):
                self_test_failures.append(
                    f"motion continuity sample count {motion_continuity_sample_count} was below "
                    f"{args.self_test_min_motion_continuity_sample_count}"
                )
            if (
                args.self_test_max_amr_frame_displacement > 0
                and max_amr_frame_displacement > args.self_test_max_amr_frame_displacement
            ):
                self_test_failures.append(
                    f"AMR frame displacement {max_amr_frame_displacement:.4f} m exceeded "
                    f"{args.self_test_max_amr_frame_displacement:.4f} m"
                    f"{format_motion_detail(amr_motion_detail)}"
                )
            if args.self_test_max_arm_ee_frame_displacement > 0:
                if arm_ee_motion_sample_count <= 0:
                    self_test_failures.append("arm end-effector motion continuity was not sampled")
                elif max_arm_ee_frame_displacement > args.self_test_max_arm_ee_frame_displacement:
                    self_test_failures.append(
                        f"arm end-effector frame displacement {max_arm_ee_frame_displacement:.4f} m exceeded "
                        f"{args.self_test_max_arm_ee_frame_displacement:.4f} m"
                        f"{format_motion_detail(arm_ee_motion_detail)}"
                    )
            if args.self_test_max_measured_arm_fk_fallbacks >= 0:
                if measured_arm_fk_fallback_count > args.self_test_max_measured_arm_fk_fallbacks:
                    self_test_failures.append(
                        f"measured arm FK fallback count {measured_arm_fk_fallback_count} exceeded "
                        f"{args.self_test_max_measured_arm_fk_fallbacks}"
                    )
            if args.self_test_max_active_bin_frame_displacement > 0:
                if active_bin_motion_sample_count <= 0:
                    self_test_failures.append("active bin motion continuity was not sampled")
                elif max_active_bin_frame_displacement > args.self_test_max_active_bin_frame_displacement:
                    self_test_failures.append(
                        f"active bin frame displacement {max_active_bin_frame_displacement:.4f} m exceeded "
                        f"{args.self_test_max_active_bin_frame_displacement:.4f} m"
                        f"{format_motion_detail(active_bin_motion_detail)}"
                    )
            if args.self_test_max_attached_bin_frame_displacement > 0:
                if attached_bin_motion_sample_count <= 0:
                    self_test_failures.append("attached bin motion continuity was not sampled")
                elif max_attached_bin_frame_displacement > args.self_test_max_attached_bin_frame_displacement:
                    self_test_failures.append(
                        f"attached bin frame displacement {max_attached_bin_frame_displacement:.4f} m exceeded "
                        f"{args.self_test_max_attached_bin_frame_displacement:.4f} m"
                        f"{format_motion_detail(attached_bin_motion_detail)}"
                    )
            if args.self_test_max_scripted_place_bin_frame_displacement > 0:
                if scripted_place_bin_motion_sample_count <= 0:
                    self_test_failures.append("scripted place bin motion continuity was not sampled")
                elif (
                    max_scripted_place_bin_frame_displacement
                    > args.self_test_max_scripted_place_bin_frame_displacement
                ):
                    self_test_failures.append(
                        f"scripted place bin frame displacement {max_scripted_place_bin_frame_displacement:.4f} m exceeded "
                        f"{args.self_test_max_scripted_place_bin_frame_displacement:.4f} m"
                        f"{format_motion_detail(scripted_place_bin_motion_detail)}"
                    )
            if args.self_test_max_released_bin_frame_displacement > 0:
                if released_bin_motion_sample_count <= 0:
                    self_test_failures.append("released bin motion continuity was not sampled")
                elif max_released_bin_frame_displacement > args.self_test_max_released_bin_frame_displacement:
                    self_test_failures.append(
                        f"released bin frame displacement {max_released_bin_frame_displacement:.4f} m exceeded "
                        f"{args.self_test_max_released_bin_frame_displacement:.4f} m"
                        f"{format_motion_detail(released_bin_motion_detail)}"
                    )
            if args.self_test_max_carried_payload_frame_displacement > 0:
                if carried_payload_motion_sample_count <= 0:
                    self_test_failures.append("carried payload motion continuity was not sampled")
                elif max_carried_payload_frame_displacement > args.self_test_max_carried_payload_frame_displacement:
                    self_test_failures.append(
                        f"carried payload frame displacement {max_carried_payload_frame_displacement:.4f} m exceeded "
                        f"{args.self_test_max_carried_payload_frame_displacement:.4f} m"
                        f"{format_motion_detail(carried_payload_motion_detail)}"
                    )
            if args.self_test_max_carried_pallet_frame_displacement > 0:
                if carried_pallet_motion_sample_count <= 0:
                    self_test_failures.append("carried pallet motion continuity was not sampled")
                elif max_carried_pallet_frame_displacement > args.self_test_max_carried_pallet_frame_displacement:
                    self_test_failures.append(
                        f"carried pallet frame displacement {max_carried_pallet_frame_displacement:.4f} m exceeded "
                        f"{args.self_test_max_carried_pallet_frame_displacement:.4f} m"
                        f"{format_motion_detail(carried_pallet_motion_detail)}"
                    )
            safety_fence_metrics = compute_safety_fence_metrics(args.pickup_y)
            safety_fence_part_count = safety_fence_metrics["safety_fence_part_count"]
            safety_fence_amr_gate_clearance = safety_fence_metrics["safety_fence_amr_gate_clearance"]
            safety_fence_infeed_gate_clearance = safety_fence_metrics["safety_fence_infeed_gate_clearance"]
            amr_cell_gate_clearance = float(getattr(orchestrator, "min_amr_cell_gate_clearance", 0.0))
            if (
                args.self_test_min_safety_fence_part_count > 0
                and safety_fence_part_count < args.self_test_min_safety_fence_part_count
            ):
                self_test_failures.append(
                    f"safety fence part count {safety_fence_part_count} was below "
                    f"{args.self_test_min_safety_fence_part_count}"
                )
            if (
                args.self_test_min_safety_fence_amr_gate_clearance > 0
                and safety_fence_amr_gate_clearance < args.self_test_min_safety_fence_amr_gate_clearance
            ):
                self_test_failures.append(
                    f"safety fence AMR gate clearance {safety_fence_amr_gate_clearance:.4f} m was below "
                    f"{args.self_test_min_safety_fence_amr_gate_clearance:.4f} m"
                )
            if (
                args.self_test_min_amr_cell_gate_clearance > 0
                and amr_cell_gate_clearance < args.self_test_min_amr_cell_gate_clearance
            ):
                self_test_failures.append(
                    f"AMR cell gate clearance {amr_cell_gate_clearance:.4f} m was below "
                    f"{args.self_test_min_amr_cell_gate_clearance:.4f} m"
                )
            if (
                args.self_test_min_safety_fence_infeed_gate_clearance > 0
                and safety_fence_infeed_gate_clearance < args.self_test_min_safety_fence_infeed_gate_clearance
            ):
                self_test_failures.append(
                    f"safety fence infeed gate clearance {safety_fence_infeed_gate_clearance:.4f} m was below "
                    f"{args.self_test_min_safety_fence_infeed_gate_clearance:.4f} m"
                )
            amr_safety_metrics = compute_amr_safety_visual_metrics()
            amr_safety_part_count = amr_safety_metrics["amr_safety_part_count"]
            amr_safety_beacon_height = amr_safety_metrics["amr_safety_beacon_height"]
            amr_safety_scanner_clearance = amr_safety_metrics["amr_safety_scanner_clearance"]
            amr_warning_indicator_count = amr_safety_metrics["amr_warning_indicator_count"]
            amr_idle_indicator_count = amr_safety_metrics["amr_idle_indicator_count"]
            max_amr_safety_pose_error = float(getattr(orchestrator, "max_amr_safety_pose_error", 0.0))
            max_amr_orientation_error = float(getattr(orchestrator, "max_amr_orientation_error", 0.0))
            amr_warning_indicator_observed = int(getattr(orchestrator, "amr_warning_indicator_on_observed", 0))
            amr_idle_indicator_observed = int(getattr(orchestrator, "amr_idle_indicator_on_observed", 0))
            amr_indicator_visibility_mismatches = int(
                getattr(orchestrator, "amr_indicator_visibility_mismatch_count", 0)
            )
            if (
                args.self_test_min_amr_safety_part_count > 0
                and amr_safety_part_count < args.self_test_min_amr_safety_part_count
            ):
                self_test_failures.append(
                    f"AMR safety part count {amr_safety_part_count} was below "
                    f"{args.self_test_min_amr_safety_part_count}"
                )
            if (
                args.self_test_min_amr_safety_beacon_height > 0
                and amr_safety_beacon_height < args.self_test_min_amr_safety_beacon_height
            ):
                self_test_failures.append(
                    f"AMR safety beacon height {amr_safety_beacon_height:.4f} m was below "
                    f"{args.self_test_min_amr_safety_beacon_height:.4f} m"
                )
            if (
                args.self_test_min_amr_safety_scanner_clearance > 0
                and amr_safety_scanner_clearance < args.self_test_min_amr_safety_scanner_clearance
            ):
                self_test_failures.append(
                    f"AMR safety scanner clearance {amr_safety_scanner_clearance:.4f} m was below "
                    f"{args.self_test_min_amr_safety_scanner_clearance:.4f} m"
                )
            if (
                args.self_test_max_amr_safety_pose_error > 0
                and max_amr_safety_pose_error > args.self_test_max_amr_safety_pose_error
            ):
                self_test_failures.append(
                    f"AMR safety pose error {max_amr_safety_pose_error:.4f} m exceeded "
                    f"{args.self_test_max_amr_safety_pose_error:.4f} m"
                )
            if (
                args.self_test_max_amr_orientation_error > 0
                and max_amr_orientation_error > args.self_test_max_amr_orientation_error
            ):
                self_test_failures.append(
                    f"AMR orientation error {max_amr_orientation_error:.4f} rad exceeded "
                    f"{args.self_test_max_amr_orientation_error:.4f} rad"
                )
            if (
                args.self_test_min_amr_warning_indicator_count > 0
                and amr_warning_indicator_count < args.self_test_min_amr_warning_indicator_count
            ):
                self_test_failures.append(
                    f"AMR warning indicator count {amr_warning_indicator_count} was below "
                    f"{args.self_test_min_amr_warning_indicator_count}"
                )
            if (
                args.self_test_min_amr_idle_indicator_count > 0
                and amr_idle_indicator_count < args.self_test_min_amr_idle_indicator_count
            ):
                self_test_failures.append(
                    f"AMR idle indicator count {amr_idle_indicator_count} was below "
                    f"{args.self_test_min_amr_idle_indicator_count}"
                )
            if (
                args.self_test_min_amr_warning_observed > 0
                and amr_warning_indicator_observed < args.self_test_min_amr_warning_observed
            ):
                self_test_failures.append(
                    f"AMR warning indicator observed {amr_warning_indicator_observed} was below "
                    f"{args.self_test_min_amr_warning_observed}"
                )
            if (
                args.self_test_min_amr_idle_observed > 0
                and amr_idle_indicator_observed < args.self_test_min_amr_idle_observed
            ):
                self_test_failures.append(
                    f"AMR idle indicator observed {amr_idle_indicator_observed} was below "
                    f"{args.self_test_min_amr_idle_observed}"
                )
            if (
                args.self_test_max_amr_indicator_visibility_mismatches >= 0
                and amr_indicator_visibility_mismatches > args.self_test_max_amr_indicator_visibility_mismatches
            ):
                self_test_failures.append(
                    f"AMR indicator visibility mismatches {amr_indicator_visibility_mismatches} exceeded "
                    f"{args.self_test_max_amr_indicator_visibility_mismatches}"
                )
            amr_drive_metrics = compute_amr_drive_visual_metrics()
            amr_drive_part_count = amr_drive_metrics["amr_drive_part_count"]
            amr_wheel_count = amr_drive_metrics["amr_wheel_count"]
            amr_wheel_floor_gap = amr_drive_metrics["amr_wheel_floor_gap"]
            amr_wheel_floor_penetration = amr_drive_metrics["amr_wheel_floor_penetration"]
            amr_drive_wheelbase = amr_drive_metrics["amr_drive_wheelbase"]
            amr_drive_track_width = amr_drive_metrics["amr_drive_track_width"]
            max_amr_drive_pose_error = float(getattr(orchestrator, "max_amr_drive_pose_error", 0.0))
            if (
                args.self_test_min_amr_drive_part_count > 0
                and amr_drive_part_count < args.self_test_min_amr_drive_part_count
            ):
                self_test_failures.append(
                    f"AMR drive part count {amr_drive_part_count} was below "
                    f"{args.self_test_min_amr_drive_part_count}"
                )
            if (
                args.self_test_max_amr_drive_pose_error > 0
                and max_amr_drive_pose_error > args.self_test_max_amr_drive_pose_error
            ):
                self_test_failures.append(
                    f"AMR drive pose error {max_amr_drive_pose_error:.4f} m exceeded "
                    f"{args.self_test_max_amr_drive_pose_error:.4f} m"
                )
            if (
                args.self_test_max_amr_wheel_floor_gap > 0
                and amr_wheel_floor_gap > args.self_test_max_amr_wheel_floor_gap
            ):
                self_test_failures.append(
                    f"AMR wheel floor gap {amr_wheel_floor_gap:.4f} m exceeded "
                    f"{args.self_test_max_amr_wheel_floor_gap:.4f} m"
                )
            if (
                args.self_test_max_amr_wheel_floor_penetration > 0
                and amr_wheel_floor_penetration > args.self_test_max_amr_wheel_floor_penetration
            ):
                self_test_failures.append(
                    f"AMR wheel floor penetration {amr_wheel_floor_penetration:.4f} m exceeded "
                    f"{args.self_test_max_amr_wheel_floor_penetration:.4f} m"
                )
            amr_lift_guide_metrics = compute_amr_lift_guide_visual_metrics()
            amr_lift_guide_count = amr_lift_guide_metrics["amr_lift_guide_count"]
            amr_lift_guide_bottom_gap = amr_lift_guide_metrics["amr_lift_guide_bottom_gap"]
            amr_lift_guide_bottom_penetration = amr_lift_guide_metrics["amr_lift_guide_bottom_penetration"]
            amr_lift_guide_travel_cover = amr_lift_guide_metrics["amr_lift_guide_travel_cover"]
            amr_lift_guide_min_height = amr_lift_guide_metrics["amr_lift_guide_min_height"]
            max_amr_lift_guide_pose_error = float(getattr(orchestrator, "max_amr_lift_guide_pose_error", 0.0))
            max_amr_lift_orientation_error = float(
                getattr(orchestrator, "max_amr_lift_orientation_error", 0.0)
            )
            if (
                args.self_test_min_amr_lift_guide_count > 0
                and amr_lift_guide_count < args.self_test_min_amr_lift_guide_count
            ):
                self_test_failures.append(
                    f"AMR lift guide count {amr_lift_guide_count} was below "
                    f"{args.self_test_min_amr_lift_guide_count}"
                )
            if (
                args.self_test_max_amr_lift_guide_bottom_gap > 0
                and amr_lift_guide_bottom_gap > args.self_test_max_amr_lift_guide_bottom_gap
            ):
                self_test_failures.append(
                    f"AMR lift guide bottom gap {amr_lift_guide_bottom_gap:.4f} m exceeded "
                    f"{args.self_test_max_amr_lift_guide_bottom_gap:.4f} m"
                )
            if amr_lift_guide_bottom_penetration > 0.005:
                self_test_failures.append(
                    f"AMR lift guide floor penetration {amr_lift_guide_bottom_penetration:.4f} m exceeded 0.0050 m"
                )
            if (
                args.self_test_min_amr_lift_guide_travel_cover > 0
                and amr_lift_guide_travel_cover < args.self_test_min_amr_lift_guide_travel_cover
            ):
                self_test_failures.append(
                    f"AMR lift guide travel cover {amr_lift_guide_travel_cover:.4f} m was below "
                    f"{args.self_test_min_amr_lift_guide_travel_cover:.4f} m"
                )
            if (
                args.self_test_max_amr_lift_guide_pose_error > 0
                and max_amr_lift_guide_pose_error > args.self_test_max_amr_lift_guide_pose_error
            ):
                self_test_failures.append(
                    f"AMR lift guide pose error {max_amr_lift_guide_pose_error:.4f} m exceeded "
                    f"{args.self_test_max_amr_lift_guide_pose_error:.4f} m"
                )
            if (
                args.self_test_max_amr_lift_orientation_error > 0
                and max_amr_lift_orientation_error > args.self_test_max_amr_lift_orientation_error
            ):
                self_test_failures.append(
                    f"AMR lift orientation error {max_amr_lift_orientation_error:.4f} rad exceeded "
                    f"{args.self_test_max_amr_lift_orientation_error:.4f} rad"
                )
            max_payload_lift = float(getattr(orchestrator, "max_payload_lift_observed", 0.0))
            lift_offset_motion_sample_count = int(getattr(orchestrator, "lift_offset_motion_sample_count", 0))
            max_lift_offset_frame_step = float(getattr(orchestrator, "max_lift_offset_frame_step", 0.0))
            if args.self_test_min_payload_lift > 0 and max_payload_lift < args.self_test_min_payload_lift:
                self_test_failures.append(
                    f"payload lift {max_payload_lift:.4f} m was below "
                    f"{args.self_test_min_payload_lift:.4f} m"
                )
            if args.self_test_max_lift_offset_frame_step > 0:
                if lift_offset_motion_sample_count <= 0:
                    self_test_failures.append("AMR lift offset continuity was not sampled")
                elif max_lift_offset_frame_step > args.self_test_max_lift_offset_frame_step:
                    self_test_failures.append(
                        f"AMR lift offset frame step {max_lift_offset_frame_step:.4f} m exceeded "
                        f"{args.self_test_max_lift_offset_frame_step:.4f} m"
                    )
            max_dropped_payload_drift = float(getattr(orchestrator, "max_dropped_payload_drift", 0.0))
            if (
                args.self_test_max_dropped_payload_drift > 0
                and max_dropped_payload_drift > args.self_test_max_dropped_payload_drift
            ):
                self_test_failures.append(
                    f"max dropped payload drift {max_dropped_payload_drift:.4f} m exceeded "
                    f"{args.self_test_max_dropped_payload_drift:.4f} m"
                )
            dropped_stack_item_count = int(getattr(orchestrator, "dropped_stack_item_count", 0))
            max_dropped_stack_pose_error = float(getattr(orchestrator, "max_dropped_stack_pose_error", 0.0))
            max_dropped_stack_orientation_error = float(
                getattr(orchestrator, "max_dropped_stack_orientation_error", 0.0)
            )
            max_dropped_stack_support_gap = float(getattr(orchestrator, "max_dropped_stack_support_gap", 0.0))
            min_dropped_stack_support_gap = float(getattr(orchestrator, "min_dropped_stack_support_gap", 0.0))
            min_dropped_stack_pallet_margin = float(getattr(orchestrator, "min_dropped_stack_pallet_margin", 0.0))
            max_dropped_stack_pallet_overhang = float(
                getattr(orchestrator, "max_dropped_stack_pallet_overhang", 0.0)
            )
            dropped_pallet_part_count = int(getattr(orchestrator, "dropped_pallet_part_count", 0))
            max_dropped_pallet_part_pose_error = float(
                getattr(orchestrator, "max_dropped_pallet_part_pose_error", 0.0)
            )
            max_dropped_pallet_part_orientation_error = float(
                getattr(orchestrator, "max_dropped_pallet_part_orientation_error", 0.0)
            )
            if (
                args.self_test_min_dropped_stack_item_count > 0
                and dropped_stack_item_count < args.self_test_min_dropped_stack_item_count
            ):
                self_test_failures.append(
                    f"dropped stack item count {dropped_stack_item_count} was below "
                    f"{args.self_test_min_dropped_stack_item_count}"
                )
            if args.self_test_max_dropped_stack_pose_error > 0:
                if transfer_cycles <= 0:
                    self_test_failures.append("dropped stack pose error was not available before a completed transfer")
                elif max_dropped_stack_pose_error > args.self_test_max_dropped_stack_pose_error:
                    self_test_failures.append(
                        f"dropped stack pose error {max_dropped_stack_pose_error:.4f} m exceeded "
                        f"{args.self_test_max_dropped_stack_pose_error:.4f} m"
                    )
            if args.self_test_max_dropped_stack_orientation_error > 0:
                if transfer_cycles <= 0:
                    self_test_failures.append(
                        "dropped stack orientation error was not available before a completed transfer"
                    )
                elif max_dropped_stack_orientation_error > args.self_test_max_dropped_stack_orientation_error:
                    self_test_failures.append(
                        f"dropped stack orientation error {max_dropped_stack_orientation_error:.4f} rad exceeded "
                        f"{args.self_test_max_dropped_stack_orientation_error:.4f} rad"
                    )
            if args.self_test_max_dropped_stack_support_gap > 0:
                if transfer_cycles <= 0:
                    self_test_failures.append("dropped stack support gap was not available before a completed transfer")
                elif max_dropped_stack_support_gap > args.self_test_max_dropped_stack_support_gap:
                    self_test_failures.append(
                        f"dropped stack support gap {max_dropped_stack_support_gap:.4f} m exceeded "
                        f"{args.self_test_max_dropped_stack_support_gap:.4f} m"
                    )
                if min_dropped_stack_support_gap < -0.005:
                    self_test_failures.append(
                        f"dropped stack vertical overlap {-min_dropped_stack_support_gap:.4f} m exceeded 0.0050 m"
                    )
            if (
                args.self_test_min_dropped_stack_pallet_margin > 0
                and min_dropped_stack_pallet_margin < args.self_test_min_dropped_stack_pallet_margin
            ):
                self_test_failures.append(
                    f"dropped stack pallet margin {min_dropped_stack_pallet_margin:.4f} m was below "
                    f"{args.self_test_min_dropped_stack_pallet_margin:.4f} m"
                )
            if (
                args.self_test_min_dropped_pallet_part_count > 0
                and dropped_pallet_part_count < args.self_test_min_dropped_pallet_part_count
            ):
                self_test_failures.append(
                    f"dropped pallet part count {dropped_pallet_part_count} was below "
                    f"{args.self_test_min_dropped_pallet_part_count}"
                )
            if args.self_test_max_dropped_pallet_part_pose_error > 0:
                if transfer_cycles <= 0:
                    self_test_failures.append(
                        "dropped pallet part pose error was not available before a completed transfer"
                    )
                elif max_dropped_pallet_part_pose_error > args.self_test_max_dropped_pallet_part_pose_error:
                    self_test_failures.append(
                        f"dropped pallet part pose error {max_dropped_pallet_part_pose_error:.4f} m exceeded "
                        f"{args.self_test_max_dropped_pallet_part_pose_error:.4f} m"
                    )
            if args.self_test_max_dropped_pallet_part_orientation_error > 0:
                if transfer_cycles <= 0:
                    self_test_failures.append(
                        "dropped pallet part orientation error was not available before a completed transfer"
                    )
                elif (
                    max_dropped_pallet_part_orientation_error
                    > args.self_test_max_dropped_pallet_part_orientation_error
                ):
                    self_test_failures.append(
                        f"dropped pallet part orientation error {max_dropped_pallet_part_orientation_error:.4f} rad exceeded "
                        f"{args.self_test_max_dropped_pallet_part_orientation_error:.4f} rad"
                    )
            amr_exit_clearance = compute_amr_exit_clearance(orchestrator.get_amr_position()[0], args.drop_x)
            if args.self_test_min_amr_exit_clearance > 0:
                if transfer_cycles <= 0:
                    self_test_failures.append("AMR exit clearance was not available before a completed transfer")
                elif amr_exit_clearance < args.self_test_min_amr_exit_clearance:
                    self_test_failures.append(
                        f"AMR exit clearance {amr_exit_clearance:.4f} m was below "
                        f"{args.self_test_min_amr_exit_clearance:.4f} m"
                    )
            max_lift_contact_gap = float(getattr(orchestrator, "max_lift_contact_gap_observed", 0.0))
            min_lift_contact_gap = float(getattr(orchestrator, "min_lift_contact_gap_observed", 0.0))
            if args.self_test_max_lift_contact_gap > 0:
                if max_lift_contact_gap > args.self_test_max_lift_contact_gap:
                    self_test_failures.append(
                        f"max lift contact gap {max_lift_contact_gap:.4f} m exceeded "
                        f"{args.self_test_max_lift_contact_gap:.4f} m"
                    )
                if min_lift_contact_gap < -0.005:
                    self_test_failures.append(
                        f"lift plate overlapped pallet underside {-min_lift_contact_gap:.4f} m exceeded 0.0050 m"
                    )
            pallet_tunnel_clearance = float(getattr(orchestrator, "pallet_tunnel_clearance", 0.0))
            if (
                args.self_test_min_pallet_tunnel_clearance > 0
                and pallet_tunnel_clearance < args.self_test_min_pallet_tunnel_clearance
            ):
                self_test_failures.append(
                    f"pallet tunnel clearance {pallet_tunnel_clearance:.4f} m was below "
                    f"{args.self_test_min_pallet_tunnel_clearance:.4f} m"
                )
            lift_fork_inner_gap = float(getattr(orchestrator, "lift_fork_inner_gap", 0.0))
            if (
                args.self_test_min_lift_fork_inner_gap > 0
                and lift_fork_inner_gap < args.self_test_min_lift_fork_inner_gap
            ):
                self_test_failures.append(
                    f"lift fork inner gap {lift_fork_inner_gap:.4f} m was below "
                    f"{args.self_test_min_lift_fork_inner_gap:.4f} m"
                )
            pickup_handoff_count = int(getattr(orchestrator, "pickup_handoff_count", 0))
            max_pickup_handoff_xy_error = float(getattr(orchestrator, "max_pickup_handoff_xy_error", 0.0))
            max_pickup_handoff_lift_gap = float(getattr(orchestrator, "max_pickup_handoff_lift_gap", 0.0))
            max_pickup_handoff_lift_penetration = float(
                getattr(orchestrator, "max_pickup_handoff_lift_penetration", 0.0)
            )
            if args.self_test_max_pickup_handoff_xy_error > 0:
                if pickup_handoff_count <= 0:
                    self_test_failures.append("pickup handoff geometry was not recorded before lift-up")
                elif max_pickup_handoff_xy_error > args.self_test_max_pickup_handoff_xy_error:
                    self_test_failures.append(
                        f"pickup handoff XY error {max_pickup_handoff_xy_error:.4f} m exceeded "
                        f"{args.self_test_max_pickup_handoff_xy_error:.4f} m"
                    )
            if args.self_test_max_pickup_handoff_lift_gap > 0:
                if pickup_handoff_count <= 0:
                    self_test_failures.append("pickup handoff lift gap was not recorded before lift-up")
                elif max_pickup_handoff_lift_gap > args.self_test_max_pickup_handoff_lift_gap:
                    self_test_failures.append(
                        f"pickup handoff lift gap {max_pickup_handoff_lift_gap:.4f} m exceeded "
                        f"{args.self_test_max_pickup_handoff_lift_gap:.4f} m"
                    )
            if (
                args.self_test_max_pickup_handoff_lift_penetration > 0
                and max_pickup_handoff_lift_penetration > args.self_test_max_pickup_handoff_lift_penetration
            ):
                self_test_failures.append(
                    f"pickup handoff lift penetration {max_pickup_handoff_lift_penetration:.4f} m exceeded "
                    f"{args.self_test_max_pickup_handoff_lift_penetration:.4f} m"
                )
            pickup_entry_sample_count = int(getattr(orchestrator, "pickup_entry_sample_count", 0))
            max_pickup_entry_y_error = float(getattr(orchestrator, "max_pickup_entry_y_error", 0.0))
            min_pickup_entry_tunnel_clearance = float(
                getattr(orchestrator, "min_pickup_entry_tunnel_clearance", 0.0)
            )
            max_pickup_entry_lift_gap = float(getattr(orchestrator, "max_pickup_entry_lift_gap", 0.0))
            max_pickup_entry_lift_penetration = float(
                getattr(orchestrator, "max_pickup_entry_lift_penetration", 0.0)
            )
            if args.self_test_max_pickup_entry_y_error > 0:
                if pickup_entry_sample_count <= 0:
                    self_test_failures.append("pickup entry geometry was not recorded while AMR entered the pallet")
                elif max_pickup_entry_y_error > args.self_test_max_pickup_entry_y_error:
                    self_test_failures.append(
                        f"pickup entry Y error {max_pickup_entry_y_error:.4f} m exceeded "
                        f"{args.self_test_max_pickup_entry_y_error:.4f} m"
                    )
            if args.self_test_min_pickup_entry_tunnel_clearance > 0:
                if pickup_entry_sample_count <= 0:
                    self_test_failures.append("pickup entry tunnel clearance was not recorded while AMR entered the pallet")
                elif min_pickup_entry_tunnel_clearance < args.self_test_min_pickup_entry_tunnel_clearance:
                    self_test_failures.append(
                        f"pickup entry tunnel clearance {min_pickup_entry_tunnel_clearance:.4f} m was below "
                        f"{args.self_test_min_pickup_entry_tunnel_clearance:.4f} m"
                    )
            if args.self_test_max_pickup_entry_lift_gap > 0:
                if pickup_entry_sample_count <= 0:
                    self_test_failures.append("pickup entry lift gap was not recorded while AMR entered the pallet")
                elif max_pickup_entry_lift_gap > args.self_test_max_pickup_entry_lift_gap:
                    self_test_failures.append(
                        f"pickup entry lift gap {max_pickup_entry_lift_gap:.4f} m exceeded "
                        f"{args.self_test_max_pickup_entry_lift_gap:.4f} m"
                    )
            if (
                args.self_test_max_pickup_entry_lift_penetration > 0
                and max_pickup_entry_lift_penetration > args.self_test_max_pickup_entry_lift_penetration
            ):
                self_test_failures.append(
                    f"pickup entry lift penetration {max_pickup_entry_lift_penetration:.4f} m exceeded "
                    f"{args.self_test_max_pickup_entry_lift_penetration:.4f} m"
                )
            slide_out_sample_count = int(getattr(orchestrator, "slide_out_sample_count", 0))
            max_slide_out_y_error = float(getattr(orchestrator, "max_slide_out_y_error", 0.0))
            max_slide_out_lift_gap = float(getattr(orchestrator, "max_slide_out_lift_gap", 0.0))
            max_slide_out_lift_penetration = float(
                getattr(orchestrator, "max_slide_out_lift_penetration", 0.0)
            )
            if args.self_test_max_slide_out_y_error > 0:
                if slide_out_sample_count <= 0:
                    self_test_failures.append("slide-out geometry was not recorded while AMR exited the pallet")
                elif max_slide_out_y_error > args.self_test_max_slide_out_y_error:
                    self_test_failures.append(
                        f"slide-out Y error {max_slide_out_y_error:.4f} m exceeded "
                        f"{args.self_test_max_slide_out_y_error:.4f} m"
                    )
            if args.self_test_max_slide_out_lift_gap > 0:
                if slide_out_sample_count <= 0:
                    self_test_failures.append("slide-out lift gap was not recorded while AMR exited the pallet")
                elif max_slide_out_lift_gap > args.self_test_max_slide_out_lift_gap:
                    self_test_failures.append(
                        f"slide-out lift gap {max_slide_out_lift_gap:.4f} m exceeded "
                        f"{args.self_test_max_slide_out_lift_gap:.4f} m"
                    )
            if (
                args.self_test_max_slide_out_lift_penetration > 0
                and max_slide_out_lift_penetration > args.self_test_max_slide_out_lift_penetration
            ):
                self_test_failures.append(
                    f"slide-out lift penetration {max_slide_out_lift_penetration:.4f} m exceeded "
                    f"{args.self_test_max_slide_out_lift_penetration:.4f} m"
                )
            drop_support_gap = compute_drop_workstation_support_gap()
            if args.self_test_max_drop_support_gap > 0:
                if drop_support_gap > args.self_test_max_drop_support_gap:
                    self_test_failures.append(
                        f"drop support gap {drop_support_gap:.4f} m exceeded "
                        f"{args.self_test_max_drop_support_gap:.4f} m"
                    )
                if drop_support_gap < -0.005:
                    self_test_failures.append(
                        f"drop support overlapped pallet underside {-drop_support_gap:.4f} m exceeded 0.0050 m"
                    )
            drop_handoff_xy_error = float(getattr(orchestrator, "drop_handoff_xy_error", 0.0))
            drop_handoff_support_gap = float(getattr(orchestrator, "drop_handoff_support_gap", 0.0))
            drop_handoff_support_penetration = float(
                getattr(orchestrator, "drop_handoff_support_penetration", 0.0)
            )
            if args.self_test_max_drop_handoff_xy_error > 0:
                if transfer_cycles <= 0:
                    self_test_failures.append("drop handoff XY error was not available before a completed transfer")
                elif drop_handoff_xy_error > args.self_test_max_drop_handoff_xy_error:
                    self_test_failures.append(
                        f"drop handoff XY error {drop_handoff_xy_error:.4f} m exceeded "
                        f"{args.self_test_max_drop_handoff_xy_error:.4f} m"
                    )
            if args.self_test_max_drop_handoff_support_gap > 0:
                if transfer_cycles <= 0:
                    self_test_failures.append("drop handoff support gap was not available before a completed transfer")
                elif drop_handoff_support_gap > args.self_test_max_drop_handoff_support_gap:
                    self_test_failures.append(
                        f"drop handoff support gap {drop_handoff_support_gap:.4f} m exceeded "
                        f"{args.self_test_max_drop_handoff_support_gap:.4f} m"
                    )
            if (
                args.self_test_max_drop_handoff_support_penetration > 0
                and drop_handoff_support_penetration > args.self_test_max_drop_handoff_support_penetration
            ):
                self_test_failures.append(
                    f"drop handoff support penetration {drop_handoff_support_penetration:.4f} m exceeded "
                    f"{args.self_test_max_drop_handoff_support_penetration:.4f} m"
                )
            drop_lane_clearance = compute_drop_workstation_tunnel_clearance()
            if (
                args.self_test_min_drop_lane_clearance > 0
                and drop_lane_clearance < args.self_test_min_drop_lane_clearance
            ):
                self_test_failures.append(
                    f"drop lane tunnel clearance {drop_lane_clearance:.4f} m was below "
                    f"{args.self_test_min_drop_lane_clearance:.4f} m"
                )
            drop_runner_clearance = compute_drop_workstation_runner_clearance()
            if (
                args.self_test_min_drop_runner_clearance > 0
                and drop_runner_clearance < args.self_test_min_drop_runner_clearance
            ):
                self_test_failures.append(
                    f"drop lane runner clearance {drop_runner_clearance:.4f} m was below "
                    f"{args.self_test_min_drop_runner_clearance:.4f} m"
                )
            drop_fork_clearance = compute_drop_workstation_fork_clearance()
            if (
                args.self_test_min_drop_fork_clearance > 0
                and drop_fork_clearance < args.self_test_min_drop_fork_clearance
            ):
                self_test_failures.append(
                    f"drop lane fork clearance {drop_fork_clearance:.4f} m was below "
                    f"{args.self_test_min_drop_fork_clearance:.4f} m"
                )
            drop_dock_metrics = compute_drop_dock_metrics()
            drop_dock_stop_count = drop_dock_metrics["drop_dock_stop_count"]
            drop_dock_stop_gap = drop_dock_metrics["drop_dock_stop_gap"]
            drop_dock_guide_clearance = drop_dock_metrics["drop_dock_guide_clearance"]
            drop_dock_fork_clearance = drop_dock_metrics["drop_dock_fork_clearance"]
            drop_dock_runner_clearance = drop_dock_metrics["drop_dock_runner_clearance"]
            if (
                args.self_test_min_drop_dock_stop_count > 0
                and drop_dock_stop_count < args.self_test_min_drop_dock_stop_count
            ):
                self_test_failures.append(
                    f"drop dock stop count {drop_dock_stop_count} was below "
                    f"{args.self_test_min_drop_dock_stop_count}"
                )
            if args.self_test_max_drop_dock_stop_gap > 0:
                if drop_dock_stop_gap > args.self_test_max_drop_dock_stop_gap:
                    self_test_failures.append(
                        f"drop dock stop gap {drop_dock_stop_gap:.4f} m exceeded "
                        f"{args.self_test_max_drop_dock_stop_gap:.4f} m"
                    )
                if drop_dock_stop_gap < 0.0:
                    self_test_failures.append(
                        f"drop dock stop overlapped pallet front {-drop_dock_stop_gap:.4f} m"
                    )
            if (
                args.self_test_min_drop_dock_guide_clearance > 0
                and drop_dock_guide_clearance < args.self_test_min_drop_dock_guide_clearance
            ):
                self_test_failures.append(
                    f"drop dock guide clearance {drop_dock_guide_clearance:.4f} m was below "
                    f"{args.self_test_min_drop_dock_guide_clearance:.4f} m"
                )
            if (
                args.self_test_min_drop_dock_fork_clearance > 0
                and drop_dock_fork_clearance < args.self_test_min_drop_dock_fork_clearance
            ):
                self_test_failures.append(
                    f"drop dock fork clearance {drop_dock_fork_clearance:.4f} m was below "
                    f"{args.self_test_min_drop_dock_fork_clearance:.4f} m"
                )
            pickup_dock_metrics = compute_pickup_dock_metrics()
            pickup_dock_stop_count = pickup_dock_metrics["pickup_dock_stop_count"]
            pickup_dock_stop_gap = pickup_dock_metrics["pickup_dock_stop_gap"]
            pickup_dock_guide_clearance = pickup_dock_metrics["pickup_dock_guide_clearance"]
            pickup_dock_fork_clearance = pickup_dock_metrics["pickup_dock_fork_clearance"]
            pickup_dock_runner_clearance = pickup_dock_metrics["pickup_dock_runner_clearance"]
            if (
                args.self_test_min_pickup_dock_stop_count > 0
                and pickup_dock_stop_count < args.self_test_min_pickup_dock_stop_count
            ):
                self_test_failures.append(
                    f"pickup dock stop count {pickup_dock_stop_count} was below "
                    f"{args.self_test_min_pickup_dock_stop_count}"
                )
            if args.self_test_max_pickup_dock_stop_gap > 0:
                if pickup_dock_stop_gap > args.self_test_max_pickup_dock_stop_gap:
                    self_test_failures.append(
                        f"pickup dock stop gap {pickup_dock_stop_gap:.4f} m exceeded "
                        f"{args.self_test_max_pickup_dock_stop_gap:.4f} m"
                    )
                if pickup_dock_stop_gap < 0.0:
                    self_test_failures.append(
                        f"pickup dock stop overlapped pallet rear {-pickup_dock_stop_gap:.4f} m"
                    )
            if (
                args.self_test_min_pickup_dock_guide_clearance > 0
                and pickup_dock_guide_clearance < args.self_test_min_pickup_dock_guide_clearance
            ):
                self_test_failures.append(
                    f"pickup dock guide clearance {pickup_dock_guide_clearance:.4f} m was below "
                    f"{args.self_test_min_pickup_dock_guide_clearance:.4f} m"
                )
            if (
                args.self_test_min_pickup_dock_fork_clearance > 0
                and pickup_dock_fork_clearance < args.self_test_min_pickup_dock_fork_clearance
            ):
                self_test_failures.append(
                    f"pickup dock fork clearance {pickup_dock_fork_clearance:.4f} m was below "
                    f"{args.self_test_min_pickup_dock_fork_clearance:.4f} m"
                )
            if (
                args.self_test_min_pickup_dock_runner_clearance > 0
                and pickup_dock_runner_clearance < args.self_test_min_pickup_dock_runner_clearance
            ):
                self_test_failures.append(
                    f"pickup dock runner clearance {pickup_dock_runner_clearance:.4f} m was below "
                    f"{args.self_test_min_pickup_dock_runner_clearance:.4f} m"
                )
            camera_metrics = compute_camera_rig_metrics(args.pickup_x, args.pickup_y, args.drop_x, args.drop_y)
            camera_rig_count = camera_metrics["camera_rig_count"]
            camera_required_role_count = camera_metrics["camera_required_role_count"]
            camera_min_height = camera_metrics["camera_min_height"]
            camera_min_target_distance = camera_metrics["camera_min_target_distance"]
            if args.self_test_min_camera_count > 0 and camera_rig_count < args.self_test_min_camera_count:
                self_test_failures.append(
                    f"camera rig count {camera_rig_count} was below {args.self_test_min_camera_count}"
                )
            if (
                args.self_test_min_camera_role_count > 0
                and camera_required_role_count < args.self_test_min_camera_role_count
            ):
                self_test_failures.append(
                    f"camera role count {camera_required_role_count} was below "
                    f"{args.self_test_min_camera_role_count}"
                )
            if args.self_test_min_camera_height > 0 and camera_min_height < args.self_test_min_camera_height:
                self_test_failures.append(
                    f"camera height {camera_min_height:.4f} m was below "
                    f"{args.self_test_min_camera_height:.4f} m"
                )
            if (
                args.self_test_min_camera_target_distance > 0
                and camera_min_target_distance < args.self_test_min_camera_target_distance
            ):
                self_test_failures.append(
                    f"camera target distance {camera_min_target_distance:.4f} m was below "
                    f"{args.self_test_min_camera_target_distance:.4f} m"
                )
            camera_director_switch_count = int(getattr(orchestrator, "camera_director_switch_count", 0))
            camera_director_role_count = len(getattr(orchestrator, "camera_director_requested_roles", set()))
            if (
                args.self_test_min_camera_director_switch_count > 0
                and camera_director_switch_count < args.self_test_min_camera_director_switch_count
            ):
                self_test_failures.append(
                    f"camera director switch count {camera_director_switch_count} was below "
                    f"{args.self_test_min_camera_director_switch_count}"
                )
            if (
                args.self_test_min_camera_director_role_count > 0
                and camera_director_role_count < args.self_test_min_camera_director_role_count
            ):
                self_test_failures.append(
                    f"camera director role count {camera_director_role_count} was below "
                    f"{args.self_test_min_camera_director_role_count}"
                )
            lighting_metrics = compute_warehouse_lighting_metrics(args.pickup_x, args.pickup_y, args.drop_x, args.drop_y)
            warehouse_light_count = lighting_metrics["warehouse_light_count"]
            warehouse_light_role_count = lighting_metrics["warehouse_light_role_count"]
            warehouse_light_min_height = lighting_metrics["warehouse_light_min_height"]
            warehouse_light_route_span = lighting_metrics["warehouse_light_route_span"]
            warehouse_light_min_intensity = lighting_metrics["warehouse_light_min_intensity"]
            if (
                args.self_test_min_warehouse_light_count > 0
                and warehouse_light_count < args.self_test_min_warehouse_light_count
            ):
                self_test_failures.append(
                    f"warehouse light count {warehouse_light_count} was below "
                    f"{args.self_test_min_warehouse_light_count}"
                )
            if (
                args.self_test_min_warehouse_light_role_count > 0
                and warehouse_light_role_count < args.self_test_min_warehouse_light_role_count
            ):
                self_test_failures.append(
                    f"warehouse light role count {warehouse_light_role_count} was below "
                    f"{args.self_test_min_warehouse_light_role_count}"
                )
            if (
                args.self_test_min_warehouse_light_height > 0
                and warehouse_light_min_height < args.self_test_min_warehouse_light_height
            ):
                self_test_failures.append(
                    f"warehouse light height {warehouse_light_min_height:.4f} m was below "
                    f"{args.self_test_min_warehouse_light_height:.4f} m"
                )
            if (
                args.self_test_min_warehouse_light_route_span > 0
                and warehouse_light_route_span < args.self_test_min_warehouse_light_route_span
            ):
                self_test_failures.append(
                    f"warehouse light route span {warehouse_light_route_span:.4f} m was below "
                    f"{args.self_test_min_warehouse_light_route_span:.4f} m"
                )
            if (
                args.self_test_min_warehouse_light_intensity > 0
                and warehouse_light_min_intensity < args.self_test_min_warehouse_light_intensity
            ):
                self_test_failures.append(
                    f"warehouse light intensity {warehouse_light_min_intensity:.4f} was below "
                    f"{args.self_test_min_warehouse_light_intensity:.4f}"
                )
            route_guard_metrics = compute_amr_route_guard_metrics(
                args.pickup_x,
                args.pickup_y,
                args.drop_x,
                args.drop_y,
            )
            amr_route_guard_part_count = route_guard_metrics["amr_route_guard_part_count"]
            amr_route_guard_span = route_guard_metrics["amr_route_guard_span"]
            amr_route_guard_clearance = route_guard_metrics["amr_route_guard_clearance"]
            amr_route_bollard_height = route_guard_metrics["amr_route_bollard_height"]
            if (
                args.self_test_min_amr_route_guard_part_count > 0
                and amr_route_guard_part_count < args.self_test_min_amr_route_guard_part_count
            ):
                self_test_failures.append(
                    f"AMR route guard part count {amr_route_guard_part_count} was below "
                    f"{args.self_test_min_amr_route_guard_part_count}"
                )
            if (
                args.self_test_min_amr_route_guard_span > 0
                and amr_route_guard_span < args.self_test_min_amr_route_guard_span
            ):
                self_test_failures.append(
                    f"AMR route guard span {amr_route_guard_span:.4f} m was below "
                    f"{args.self_test_min_amr_route_guard_span:.4f} m"
                )
            if (
                args.self_test_min_amr_route_guard_clearance > 0
                and amr_route_guard_clearance < args.self_test_min_amr_route_guard_clearance
            ):
                self_test_failures.append(
                    f"AMR route guard clearance {amr_route_guard_clearance:.4f} m was below "
                    f"{args.self_test_min_amr_route_guard_clearance:.4f} m"
                )
            if (
                args.self_test_min_amr_route_bollard_height > 0
                and amr_route_bollard_height < args.self_test_min_amr_route_bollard_height
            ):
                self_test_failures.append(
                    f"AMR route bollard height {amr_route_bollard_height:.4f} m was below "
                    f"{args.self_test_min_amr_route_bollard_height:.4f} m"
                )
            max_loaded_route_y_error = float(getattr(orchestrator, "max_loaded_route_y_error", 0.0))
            min_loaded_route_guard_clearance = float(
                getattr(orchestrator, "min_loaded_route_guard_clearance", 0.0)
            )
            max_carried_pallet_pose_error = float(getattr(orchestrator, "max_carried_pallet_pose_error", 0.0))
            max_carried_pallet_orientation_error = float(
                getattr(orchestrator, "max_carried_pallet_orientation_error", 0.0)
            )
            max_carried_payload_pose_error = float(getattr(orchestrator, "max_carried_payload_pose_error", 0.0))
            max_carried_payload_orientation_error = float(
                getattr(orchestrator, "max_carried_payload_orientation_error", 0.0)
            )
            arm_tcp_amr_route_clearance_sample_count = int(
                getattr(orchestrator, "arm_tcp_amr_route_clearance_sample_count", 0)
            )
            min_arm_tcp_amr_route_clearance = float(
                getattr(orchestrator, "min_arm_tcp_amr_route_clearance", 0.0)
            )
            if not math.isfinite(min_arm_tcp_amr_route_clearance):
                min_arm_tcp_amr_route_clearance = 0.0
            if (
                args.self_test_max_loaded_route_y_error > 0
                and max_loaded_route_y_error > args.self_test_max_loaded_route_y_error
            ):
                self_test_failures.append(
                    f"loaded route Y error {max_loaded_route_y_error:.4f} m exceeded "
                    f"{args.self_test_max_loaded_route_y_error:.4f} m"
                )
            if (
                args.self_test_min_loaded_route_guard_clearance > 0
                and min_loaded_route_guard_clearance < args.self_test_min_loaded_route_guard_clearance
            ):
                self_test_failures.append(
                    f"loaded route guard clearance {min_loaded_route_guard_clearance:.4f} m was below "
                    f"{args.self_test_min_loaded_route_guard_clearance:.4f} m"
                )
            if args.self_test_min_arm_tcp_amr_route_clearance > 0:
                if arm_tcp_amr_route_clearance_sample_count <= 0:
                    self_test_failures.append("arm TCP AMR route clearance was not sampled")
                elif min_arm_tcp_amr_route_clearance < args.self_test_min_arm_tcp_amr_route_clearance:
                    self_test_failures.append(
                        f"arm TCP AMR route clearance {min_arm_tcp_amr_route_clearance:.4f} m was below "
                        f"{args.self_test_min_arm_tcp_amr_route_clearance:.4f} m"
                    )
            if (
                args.self_test_max_carried_pallet_pose_error > 0
                and max_carried_pallet_pose_error > args.self_test_max_carried_pallet_pose_error
            ):
                self_test_failures.append(
                    f"carried pallet pose error {max_carried_pallet_pose_error:.4f} m exceeded "
                    f"{args.self_test_max_carried_pallet_pose_error:.4f} m"
                )
            if (
                args.self_test_max_carried_pallet_orientation_error > 0
                and max_carried_pallet_orientation_error > args.self_test_max_carried_pallet_orientation_error
            ):
                self_test_failures.append(
                    f"carried pallet orientation error {max_carried_pallet_orientation_error:.4f} rad exceeded "
                    f"{args.self_test_max_carried_pallet_orientation_error:.4f} rad"
                )
            if (
                args.self_test_max_carried_payload_pose_error > 0
                and max_carried_payload_pose_error > args.self_test_max_carried_payload_pose_error
            ):
                self_test_failures.append(
                    f"carried payload pose error {max_carried_payload_pose_error:.4f} m exceeded "
                    f"{args.self_test_max_carried_payload_pose_error:.4f} m"
                )
            if (
                args.self_test_max_carried_payload_orientation_error > 0
                and max_carried_payload_orientation_error > args.self_test_max_carried_payload_orientation_error
            ):
                self_test_failures.append(
                    f"carried payload orientation error {max_carried_payload_orientation_error:.4f} rad exceeded "
                    f"{args.self_test_max_carried_payload_orientation_error:.4f} rad"
                )
            drop_docking_metrics = compute_drop_docking_metrics(args.pickup_x, args.drop_x)
            drop_approach_standoff = drop_docking_metrics["drop_approach_standoff"]
            dock_move_speed_scale = drop_docking_metrics["dock_move_speed_scale"]
            drop_dock_arrival_count = int(getattr(orchestrator, "drop_dock_arrival_count", 0))
            drop_approach_final_error = float(getattr(orchestrator, "drop_approach_final_error", 0.0))
            drop_dock_final_error = float(getattr(orchestrator, "drop_dock_final_error", 0.0))
            if (
                args.self_test_min_drop_approach_standoff > 0
                and drop_approach_standoff < args.self_test_min_drop_approach_standoff
            ):
                self_test_failures.append(
                    f"drop approach standoff {drop_approach_standoff:.4f} m was below "
                    f"{args.self_test_min_drop_approach_standoff:.4f} m"
                )
            if (
                args.self_test_min_drop_dock_arrival_count > 0
                and drop_dock_arrival_count < args.self_test_min_drop_dock_arrival_count
            ):
                self_test_failures.append(
                    f"drop dock arrival count {drop_dock_arrival_count} was below "
                    f"{args.self_test_min_drop_dock_arrival_count}"
                )
            if (
                args.self_test_max_drop_dock_final_error > 0
                and drop_dock_final_error > args.self_test_max_drop_dock_final_error
            ):
                self_test_failures.append(
                    f"drop dock final error {drop_dock_final_error:.4f} m exceeded "
                    f"{args.self_test_max_drop_dock_final_error:.4f} m"
                )
            review_gif_path = save_review_gif()
            review_gif_latest_path = getattr(gif_recorder, "latest_path", None)
            review_gif_frame_count = len(getattr(gif_recorder, "frames", []))
            review_gif_width = int(gif_recorder.canvas_size[0])
            review_gif_height = int(gif_recorder.canvas_size[1])
            if args.self_test_require_review_gif:
                if not review_gif_path or not Path(review_gif_path).exists():
                    self_test_failures.append("review GIF was not saved")
                if not review_gif_latest_path or not Path(review_gif_latest_path).exists():
                    self_test_failures.append("latest review GIF was not updated")
                if review_gif_frame_count <= 0:
                    self_test_failures.append("review GIF captured no simulation frames")

            if self_test_failures:
                self_test_failure_message = "; ".join(self_test_failures)
                print(f"[HarimDemo] self-test failed: {self_test_failure_message}", flush=True)
                print("[HarimDemo] preserving failure exit; skipping SimulationApp.close()", flush=True)
                os._exit(1)
            else:
                print(
                    f"[HarimDemo] self-test completed after {args.self_test_frames} frames; "
                    f"placed_bins={placed_count}; transfer_cycles={transfer_cycles}; "
                    f"max_pre_grip_offset={max_pre_grip_offset:.4f}; "
                    f"max_return_ready_error={max_return_ready_error:.4f}; "
                    f"max_release_drift={max_release_drift:.4f}; "
                    f"max_release_retreat_lift={max_release_retreat_lift:.4f}; "
                    f"scripted_place_count={scripted_place_count}; "
                    f"max_scripted_place_error={max_scripted_place_error:.4f}; "
                    f"max_release_separation={max_release_separation:.4f}; "
                    f"max_release_vertical_clearance={max_release_vertical_clearance:.4f}; "
                    f"release_gripper_samples={release_gripper_samples}; "
                    f"release_gripper_not_open={release_gripper_not_open}; "
                    f"release_gripped_object_max={release_gripped_object_max}; "
                    f"release_gripper_probe_failures={release_gripper_probe_failures}; "
                    f"attached_grasp_sample_count={attached_grasp_sample_count}; "
                    f"max_attached_grasp_error={max_attached_grasp_error:.4f}; "
                    f"joint_settle_count={joint_settle_count}; "
                    f"max_stack_lateral_gap={max_stack_lateral_gap:.4f}; "
                    f"min_stack_lateral_gap={min_stack_lateral_gap:.4f}; "
                    f"max_stack_support_gap={max_stack_support_gap:.4f}; "
                    f"min_stack_support_gap={min_stack_support_gap:.4f}; "
                    f"min_stack_pallet_margin={min_stack_pallet_margin:.4f}; "
                    f"max_stack_pallet_overhang={max_stack_pallet_overhang:.4f}; "
                    f"load_restraint_part_count={load_restraint_part_count}; "
                    f"min_load_restraint_pallet_margin={min_load_restraint_pallet_margin:.4f}; "
                    f"max_load_restraint_pallet_overhang={max_load_restraint_pallet_overhang:.4f}; "
                    f"infeed_conveyor_length={infeed_conveyor_length:.4f}; "
                    f"infeed_spawn_margin={infeed_spawn_margin:.4f}; "
                    f"infeed_pick_margin={infeed_pick_margin:.4f}; "
                    f"infeed_guide_clearance={infeed_guide_clearance:.4f}; "
                    f"infeed_belt_support_gap={infeed_belt_support_gap:.4f}; "
                    f"infeed_motion_marker_count={infeed_motion_marker_count}; "
                    f"infeed_motion_marker_spacing={infeed_motion_marker_spacing:.4f}; "
                    f"infeed_motion_marker_speed={infeed_motion_marker_speed:.4f}; "
                    f"infeed_motion_observed_travel={infeed_motion_observed_travel:.4f}; "
                    f"infeed_feed_carton_count={infeed_feed_carton_count}; "
                    f"infeed_feed_carton_path_length={infeed_feed_carton_path_length:.4f}; "
                    f"infeed_feed_carton_speed={infeed_feed_carton_speed:.4f}; "
                    f"infeed_feed_carton_observed_travel={infeed_feed_carton_observed_travel:.4f}; "
                    f"infeed_feed_carton_stop_clearance={infeed_feed_carton_stop_clearance:.4f}; "
                    f"infeed_feed_carton_guide_clearance={infeed_feed_carton_guide_clearance:.4f}; "
                    f"infeed_feed_carton_belt_support_gap={infeed_feed_carton_belt_support_gap:.4f}; "
                    f"active_bin_conveyor_approach_count={active_bin_conveyor_approach_count}; "
                    f"active_bin_conveyor_completed_count={active_bin_conveyor_completed_count}; "
                    f"active_bin_conveyor_travel_distance={active_bin_conveyor_travel_distance:.4f}; "
                    f"active_bin_conveyor_duration={active_bin_conveyor_duration:.4f}; "
                    f"active_bin_conveyor_nominal_speed={active_bin_conveyor_nominal_speed:.4f}; "
                    f"active_bin_conveyor_observed_travel={active_bin_conveyor_observed_travel:.4f}; "
                    f"active_bin_conveyor_final_error={active_bin_conveyor_final_error:.4f}; "
                    f"active_bin_conveyor_lateral_error={active_bin_conveyor_lateral_error:.4f}; "
                    f"active_bin_conveyor_belt_support_gap={active_bin_conveyor_belt_support_gap:.4f}; "
                    f"motion_continuity_sample_count={motion_continuity_sample_count}; "
                    f"motion_continuity_tracked_item_count={motion_continuity_tracked_item_count}; "
                    f"arm_ee_motion_sample_count={arm_ee_motion_sample_count}; "
                    f"active_bin_motion_sample_count={active_bin_motion_sample_count}; "
                    f"attached_bin_motion_sample_count={attached_bin_motion_sample_count}; "
                    f"scripted_place_bin_motion_sample_count={scripted_place_bin_motion_sample_count}; "
                    f"released_bin_motion_sample_count={released_bin_motion_sample_count}; "
                    f"carried_payload_motion_sample_count={carried_payload_motion_sample_count}; "
                    f"carried_pallet_motion_sample_count={carried_pallet_motion_sample_count}; "
                    f"max_amr_frame_displacement={max_amr_frame_displacement:.4f}; "
                    f"max_arm_ee_frame_displacement={max_arm_ee_frame_displacement:.4f}; "
                    f"measured_arm_fk_sample_count={measured_arm_fk_sample_count}; "
                    f"measured_arm_fk_fallback_count={measured_arm_fk_fallback_count}; "
                    f"max_active_bin_frame_displacement={max_active_bin_frame_displacement:.4f}; "
                    f"max_attached_bin_frame_displacement={max_attached_bin_frame_displacement:.4f}; "
                    f"max_scripted_place_bin_frame_displacement={max_scripted_place_bin_frame_displacement:.4f}; "
                    f"max_released_bin_frame_displacement={max_released_bin_frame_displacement:.4f}; "
                    f"max_carried_payload_frame_displacement={max_carried_payload_frame_displacement:.4f}; "
                    f"max_carried_pallet_frame_displacement={max_carried_pallet_frame_displacement:.4f}; "
                    f"safety_fence_part_count={safety_fence_part_count}; "
                    f"safety_fence_amr_gate_clearance={safety_fence_amr_gate_clearance:.4f}; "
                    f"amr_cell_gate_clearance={amr_cell_gate_clearance:.4f}; "
                    f"safety_fence_infeed_gate_clearance={safety_fence_infeed_gate_clearance:.4f}; "
                    f"amr_safety_part_count={amr_safety_part_count}; "
                    f"amr_safety_beacon_height={amr_safety_beacon_height:.4f}; "
                    f"amr_safety_scanner_clearance={amr_safety_scanner_clearance:.4f}; "
                    f"max_amr_safety_pose_error={max_amr_safety_pose_error:.4f}; "
                    f"max_amr_orientation_error={max_amr_orientation_error:.4f}; "
                    f"amr_warning_indicator_count={amr_warning_indicator_count}; "
                    f"amr_idle_indicator_count={amr_idle_indicator_count}; "
                    f"amr_warning_indicator_observed={amr_warning_indicator_observed}; "
                    f"amr_idle_indicator_observed={amr_idle_indicator_observed}; "
                    f"amr_indicator_visibility_mismatches={amr_indicator_visibility_mismatches}; "
                    f"amr_drive_part_count={amr_drive_part_count}; "
                    f"amr_wheel_count={amr_wheel_count}; "
                    f"amr_wheel_floor_gap={amr_wheel_floor_gap:.4f}; "
                    f"amr_wheel_floor_penetration={amr_wheel_floor_penetration:.4f}; "
                    f"amr_drive_wheelbase={amr_drive_wheelbase:.4f}; "
                    f"amr_drive_track_width={amr_drive_track_width:.4f}; "
                    f"max_amr_drive_pose_error={max_amr_drive_pose_error:.4f}; "
                    f"amr_lift_guide_count={amr_lift_guide_count}; "
                    f"amr_lift_guide_bottom_gap={amr_lift_guide_bottom_gap:.4f}; "
                    f"amr_lift_guide_bottom_penetration={amr_lift_guide_bottom_penetration:.4f}; "
                    f"amr_lift_guide_travel_cover={amr_lift_guide_travel_cover:.4f}; "
                    f"amr_lift_guide_min_height={amr_lift_guide_min_height:.4f}; "
                    f"max_amr_lift_guide_pose_error={max_amr_lift_guide_pose_error:.4f}; "
                    f"max_amr_lift_orientation_error={max_amr_lift_orientation_error:.4f}; "
                    f"max_payload_lift={max_payload_lift:.4f}; "
                    f"lift_offset_motion_sample_count={lift_offset_motion_sample_count}; "
                    f"max_lift_offset_frame_step={max_lift_offset_frame_step:.4f}; "
                    f"max_dropped_payload_drift={max_dropped_payload_drift:.4f}; "
                    f"dropped_stack_item_count={dropped_stack_item_count}; "
                    f"max_dropped_stack_pose_error={max_dropped_stack_pose_error:.4f}; "
                    f"max_dropped_stack_orientation_error={max_dropped_stack_orientation_error:.4f}; "
                    f"max_dropped_stack_support_gap={max_dropped_stack_support_gap:.4f}; "
                    f"min_dropped_stack_support_gap={min_dropped_stack_support_gap:.4f}; "
                    f"min_dropped_stack_pallet_margin={min_dropped_stack_pallet_margin:.4f}; "
                    f"max_dropped_stack_pallet_overhang={max_dropped_stack_pallet_overhang:.4f}; "
                    f"dropped_pallet_part_count={dropped_pallet_part_count}; "
                    f"max_dropped_pallet_part_pose_error={max_dropped_pallet_part_pose_error:.4f}; "
                    f"max_dropped_pallet_part_orientation_error={max_dropped_pallet_part_orientation_error:.4f}; "
                    f"amr_exit_clearance={amr_exit_clearance:.4f}; "
                    f"max_lift_contact_gap={max_lift_contact_gap:.4f}; "
                    f"min_lift_contact_gap={min_lift_contact_gap:.4f}; "
                    f"pallet_tunnel_clearance={pallet_tunnel_clearance:.4f}; "
                    f"lift_fork_inner_gap={lift_fork_inner_gap:.4f}; "
                    f"pickup_handoff_count={pickup_handoff_count}; "
                    f"max_pickup_handoff_xy_error={max_pickup_handoff_xy_error:.4f}; "
                    f"max_pickup_handoff_lift_gap={max_pickup_handoff_lift_gap:.4f}; "
                    f"max_pickup_handoff_lift_penetration={max_pickup_handoff_lift_penetration:.4f}; "
                    f"pickup_entry_sample_count={pickup_entry_sample_count}; "
                    f"max_pickup_entry_y_error={max_pickup_entry_y_error:.4f}; "
                    f"min_pickup_entry_tunnel_clearance={min_pickup_entry_tunnel_clearance:.4f}; "
                    f"max_pickup_entry_lift_gap={max_pickup_entry_lift_gap:.4f}; "
                    f"max_pickup_entry_lift_penetration={max_pickup_entry_lift_penetration:.4f}; "
                    f"slide_out_sample_count={slide_out_sample_count}; "
                    f"max_slide_out_y_error={max_slide_out_y_error:.4f}; "
                    f"max_slide_out_lift_gap={max_slide_out_lift_gap:.4f}; "
                    f"max_slide_out_lift_penetration={max_slide_out_lift_penetration:.4f}; "
                    f"drop_support_gap={drop_support_gap:.4f}; "
                    f"drop_handoff_xy_error={drop_handoff_xy_error:.4f}; "
                    f"drop_handoff_support_gap={drop_handoff_support_gap:.4f}; "
                    f"drop_handoff_support_penetration={drop_handoff_support_penetration:.4f}; "
                    f"drop_lane_clearance={drop_lane_clearance:.4f}; "
                    f"drop_runner_clearance={drop_runner_clearance:.4f}; "
                    f"drop_fork_clearance={drop_fork_clearance:.4f}; "
                    f"drop_dock_stop_count={drop_dock_stop_count}; "
                    f"drop_dock_stop_gap={drop_dock_stop_gap:.4f}; "
                    f"drop_dock_guide_clearance={drop_dock_guide_clearance:.4f}; "
                    f"drop_dock_fork_clearance={drop_dock_fork_clearance:.4f}; "
                    f"drop_dock_runner_clearance={drop_dock_runner_clearance:.4f}; "
                    f"pickup_dock_stop_count={pickup_dock_stop_count}; "
                    f"pickup_dock_stop_gap={pickup_dock_stop_gap:.4f}; "
                    f"pickup_dock_guide_clearance={pickup_dock_guide_clearance:.4f}; "
                    f"pickup_dock_fork_clearance={pickup_dock_fork_clearance:.4f}; "
                    f"pickup_dock_runner_clearance={pickup_dock_runner_clearance:.4f}; "
                    f"camera_rig_count={camera_rig_count}; "
                    f"camera_required_role_count={camera_required_role_count}; "
                    f"camera_min_height={camera_min_height:.4f}; "
                    f"camera_min_target_distance={camera_min_target_distance:.4f}; "
                    f"camera_director_switch_count={camera_director_switch_count}; "
                    f"camera_director_role_count={camera_director_role_count}; "
                    f"warehouse_light_count={warehouse_light_count}; "
                    f"warehouse_light_role_count={warehouse_light_role_count}; "
                    f"warehouse_light_min_height={warehouse_light_min_height:.4f}; "
                    f"warehouse_light_route_span={warehouse_light_route_span:.4f}; "
                    f"warehouse_light_min_intensity={warehouse_light_min_intensity:.4f}; "
                    f"amr_route_guard_part_count={amr_route_guard_part_count}; "
                    f"amr_route_guard_span={amr_route_guard_span:.4f}; "
                    f"amr_route_guard_clearance={amr_route_guard_clearance:.4f}; "
                    f"amr_route_bollard_height={amr_route_bollard_height:.4f}; "
                    f"max_loaded_route_y_error={max_loaded_route_y_error:.4f}; "
                    f"min_loaded_route_guard_clearance={min_loaded_route_guard_clearance:.4f}; "
                    f"arm_tcp_amr_route_clearance_sample_count={arm_tcp_amr_route_clearance_sample_count}; "
                    f"min_arm_tcp_amr_route_clearance={min_arm_tcp_amr_route_clearance:.4f}; "
                    f"max_carried_pallet_pose_error={max_carried_pallet_pose_error:.4f}; "
                    f"max_carried_pallet_orientation_error={max_carried_pallet_orientation_error:.4f}; "
                    f"max_carried_payload_pose_error={max_carried_payload_pose_error:.4f}; "
                    f"max_carried_payload_orientation_error={max_carried_payload_orientation_error:.4f}; "
                    f"drop_approach_standoff={drop_approach_standoff:.4f}; "
                    f"dock_move_speed_scale={dock_move_speed_scale:.4f}; "
                    f"drop_dock_arrival_count={drop_dock_arrival_count}; "
                    f"drop_approach_final_error={drop_approach_final_error:.4f}; "
                    f"drop_dock_final_error={drop_dock_final_error:.4f}; "
                    f"review_gif_canvas_width={review_gif_width}; "
                    f"review_gif_canvas_height={review_gif_height}; "
                    f"review_gif_frame_count={review_gif_frame_count}; "
                    f"review_gif_path={review_gif_path or ''}; "
                    f"latest_review_gif_path={review_gif_latest_path or ''}",
                    flush=True,
                )
        else:
            while simulation_app.is_running():
                step_demo_frame()
    except Exception as exc:
        print(f"[HarimDemo] simulation aborted: {exc}", flush=True)
        traceback.print_exc()
        save_review_gif()
        print("[HarimDemo] preserving failure exit; skipping SimulationApp.close()", flush=True)
        os._exit(1)
    finally:
        save_review_gif()
        simulation_app.close()


if __name__ == "__main__":
    main()
