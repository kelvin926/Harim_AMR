import argparse
import math
import os
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
WORLD_FLOOR_Z = -1.1818
DEFAULT_AMR_Z = WORLD_FLOOR_Z
DEFAULT_LIFT_HEIGHT = 0.11
DEFAULT_MOVE_SPEED = 0.65
PICK_STATION_BIN_POSITION = np.array([0.0, 0.42, -0.15], dtype=float)
CONVEYOR_PICK_WINDOW_Y = 0.68
PICK_READY_EE_POSITION = np.array([0.16, 0.22, -0.02], dtype=float)
POST_RELEASE_CLEARANCE_LIFT = 0.22
POST_RELEASE_JOINT_SETTLE_DURATION = 0.65
ARM_CLEAR_SETTLE_TIME = 1.8
REACH_PICK_MAX_DURATION = 12.0
REACH_PLACE_MAX_DURATION = 4.2
RETURN_READY_DURATION = 10.0
RETURN_READY_POSITION_THRESHOLD = 0.04
AMR_START_STANDOFF = 3.2
AMR_APPROACH_STANDOFF = 1.05
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
CARTON_BODY_SCALE = np.array([0.20, 0.29, 0.14], dtype=float)
CARTON_TAPE_TOP_SCALE = np.array([0.205, 0.030, 0.008], dtype=float)
CARTON_BODY_COLOR = np.array([0.72, 0.48, 0.26], dtype=float)
CARTON_TAPE_COLOR = np.array([0.86, 0.10, 0.08], dtype=float)


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
        "--self-test-require-gripper-open-after-release",
        action="store_true",
        help="Fail the fixed-frame self-test if the surface gripper is not open or still reports gripped objects after a scripted release.",
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
        "--self-test-min-payload-lift",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test unless the AMR visibly lifts the payload by at least this distance in meters. 0 disables the check.",
    )
    parser.add_argument(
        "--self-test-max-dropped-payload-drift",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the delivered pallet assembly drifts after detach by more than this distance in meters. 0 disables the check.",
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
        "--self-test-max-drop-support-gap",
        type=float,
        default=0.0,
        help="Fail the fixed-frame self-test if the drop workstation support is farther below the pallet deck underside than this distance in meters. 0 disables the check.",
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
    MOVE_TO_DROP = auto()
    LIFT_DOWN = auto()
    DETACH = auto()
    SLIDE_OUT_FROM_PALLET = auto()
    RESET_CYCLE = auto()
    DONE_IDLE = auto()


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
        lift_plate_parts=None,
        amr_lift_prim=None,
        completion_signal=None,
    ):
        self.world = world
        self.context = context
        self.task = task
        self.amr = amr_prim
        self.amr_lift = amr_lift_prim
        self.lift_plate = lift_plate
        self.lift_plate_parts = list(lift_plate_parts) if lift_plate_parts is not None else [lift_plate]
        self.pallet_parts = pallet_parts
        self.stack_coordinates = stack_coordinates
        self.args = args
        self.completion_signal = completion_signal

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
        self.max_payload_lift_observed = 0.0
        self.max_dropped_payload_drift = 0.0
        initial_lift_contact_gap = compute_lift_contact_gap(args.amr_z, 0.0)
        self.max_lift_contact_gap_observed = initial_lift_contact_gap
        self.min_lift_contact_gap_observed = initial_lift_contact_gap
        self.pallet_tunnel_clearance = compute_pallet_tunnel_clearance()
        self.lift_fork_inner_gap = compute_lift_fork_inner_gap()
        self.amr_lift_base_offset = None
        self.amr_lift_orientation = None
        self.move_target = None
        self.move_start_pose = None
        self.move_duration = 0.0

        self.start_pose = np.array([args.pickup_x + AMR_START_STANDOFF, args.pickup_y, args.amr_z], dtype=float)
        self.approach_pose = np.array([args.pickup_x + AMR_APPROACH_STANDOFF, args.pickup_y, args.amr_z], dtype=float)
        self.pickup_pose = np.array([args.pickup_x, args.pickup_y, args.amr_z], dtype=float)
        self.drop_pose = np.array([args.drop_x, args.drop_y, args.amr_z], dtype=float)
        self.exit_pose = np.array([args.drop_x + SLIDE_EXIT_DISTANCE, args.drop_y, args.amr_z], dtype=float)

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
        elif state == TransferState.MOVE_TO_DROP:
            self.move_target = self.drop_pose
        elif state == TransferState.SLIDE_OUT_FROM_PALLET:
            self.move_target = self.exit_pose
        else:
            self.move_target = None
        if self.move_target is not None:
            self.move_start_pose = self.get_amr_position()
            move_distance = float(np.linalg.norm((self.move_target - self.move_start_pose)[:2]))
            self.move_duration = max(move_distance / max(float(self.args.move_speed), 1e-6), 0.35)
        else:
            self.move_start_pose = None
            self.move_duration = 0.0
        print(f"[HarimDemo] state -> {state.name}")

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
        self.set_amr_pose(self.start_pose)
        self._set_lift_plate_pose()
        self._reset_pallet_pose()
        if self.completion_signal is not None:
            self.completion_signal.set_completed(False)

    def set_amr_pose(self, position):
        self.amr.set_world_pose(position=np.array(position, dtype=float), orientation=yaw_to_quat(self.amr_yaw))

    def get_amr_position(self):
        position, _orientation = self.amr.get_world_pose()
        return np.array(position, dtype=float)

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
        self._record_lift_geometry()

    def _record_lift_geometry(self):
        amr_pos = self.get_amr_position()
        lift_contact_gap = compute_lift_contact_gap(amr_pos[2], self.lift_offset)
        self.max_lift_contact_gap_observed = max(self.max_lift_contact_gap_observed, lift_contact_gap)
        self.min_lift_contact_gap_observed = min(self.min_lift_contact_gap_observed, lift_contact_gap)

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
        for part, offset in zip(self.pallet_parts, PALLET_PART_OFFSETS):
            part.set_world_pose(position=center + offset, orientation=yaw_to_quat(0.0))

    def _move_amr_toward_target(self, dt):
        if self.move_target is None:
            return True

        if self.move_start_pose is None:
            self.move_start_pose = self.get_amr_position()
            move_distance = float(np.linalg.norm((self.move_target - self.move_start_pose)[:2]))
            self.move_duration = max(move_distance / max(float(self.args.move_speed), 1e-6), 0.35)

        t = clamp(self.state_time / max(self.move_duration, 1e-6), 0.0, 1.0)
        next_pos = lerp(self.move_start_pose, self.move_target, smoothstep(t))
        self.set_amr_pose(next_pos)
        self._set_lift_plate_pose()
        self._sync_payload_pose()
        if t >= 1.0:
            self.set_amr_pose(self.move_target)
            self._set_lift_plate_pose()
            self._sync_payload_pose()
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

    def _hold_dropped_assembly(self):
        for item, pos, orient in self.dropped_item_poses.values():
            try:
                current_pos, _ = item.get_world_pose()
                drift = float(np.linalg.norm(np.array(current_pos, dtype=float) - np.array(pos, dtype=float)))
                self.max_dropped_payload_drift = max(self.max_dropped_payload_drift, drift)
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
            TransferState.MOVE_TO_DROP,
            TransferState.SLIDE_OUT_FROM_PALLET,
        ):
            arrived = self._move_amr_toward_target(dt)
            if arrived:
                if self.state == TransferState.MOVE_TO_APPROACH:
                    self._transition(TransferState.MOVE_UNDER_PALLET)
                elif self.state == TransferState.MOVE_UNDER_PALLET:
                    self._transition(TransferState.LIFT_UP)
                elif self.state == TransferState.MOVE_TO_DROP:
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
            self._transition(TransferState.MOVE_TO_DROP)

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
    from isaacsim.cortex.framework.motion_commander import MotionCommand
    from isaacsim.cortex.framework.robot import CortexUr10
    from isaacsim.cortex.behaviors.ur10 import bin_stacking_behavior as behavior
    from pxr import UsdGeom, UsdPhysics

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
        if carried_bin is None or not getattr(carried_bin, "demo_attached", False):
            return None
        context.active_bin = carried_bin
        carried_bin.is_attached = True
        return carried_bin

    def clear_demo_carry_context(context):
        context.active_bin = None
        context.demo_carried_bin = None
        context.demo_pre_grip_bin = None
        context.demo_pre_grip_initial_offset = None

    def mark_demo_bin_released(context, bin_state, target_position, target_orientation):
        bin_state.demo_attached = False
        bin_state.demo_attach_T = None
        bin_state.is_attached = False
        bin_state.demo_release_target_p = np.array(target_position, dtype=float)
        bin_state.demo_release_target_q = np.array(target_orientation, dtype=float)
        context.demo_released_bin = bin_state
        clear_demo_carry_context(context)

    def force_open_suction_gripper(context):
        gripper = getattr(getattr(context, "robot", None), "suction_gripper", None)
        if gripper is None:
            return
        try:
            gripper.open()
        except Exception as exc:
            print(f"[HarimDemo] suction open fallback: {exc}", flush=True)

        interface = getattr(gripper, "_surface_gripper_interface", None)
        gripper_path = getattr(gripper, "_surface_gripper_path", None)
        if interface is not None and gripper_path is not None:
            try:
                interface.open_gripper(gripper_path)
            except Exception:
                pass

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
        released_bin.is_attached = False
        if getattr(context, "active_bin", None) is released_bin:
            context.active_bin = None
        released_bin.bin_obj.set_world_pose(position=target_p, orientation=target_q)
        stop_dynamic_prim(released_bin.bin_obj)
        set_kinematic_for_demo(released_bin.bin_obj, True)

    def get_demo_time(context):
        return float(getattr(context, "demo_sim_time", time.time()))

    def get_demo_stack_coordinate(context, index):
        canonical_coordinates = getattr(context, "demo_stack_coordinates", None)
        if canonical_coordinates is not None:
            return np.array(canonical_coordinates[index], dtype=float)
        return np.array(context.stack_coordinates[index], dtype=float)

    def hold_active_bin_for_pick(context):
        active_bin = get_demo_pre_grip_bin(context) or getattr(context, "active_bin", None)
        if active_bin is None or getattr(active_bin, "demo_attached", False):
            return
        context.active_bin = active_bin
        if active_bin in getattr(context, "stacked_bins", []):
            return
        if not getattr(active_bin, "demo_pick_stationed", False):
            _position, orientation = active_bin.bin_obj.get_world_pose()
            set_kinematic_for_demo(active_bin.bin_obj, True)
            active_bin.bin_obj.set_world_pose(position=PICK_STATION_BIN_POSITION, orientation=orientation)
            active_bin.demo_pick_stationed = True
        else:
            set_kinematic_for_demo(active_bin.bin_obj, True)
        stop_dynamic_prim(active_bin.bin_obj)

    def compute_active_bin_grasp_pose_at_effector(context, active_bin=None):
        active_bin = active_bin or getattr(context, "active_bin", None)
        if active_bin is None:
            return None
        grasp_T = getattr(active_bin, "grasp_T", None)
        if grasp_T is None:
            return None

        eff_T = context.robot.arm.get_fk_T()
        bin_T = cortex_math_util.pq2T(*active_bin.bin_obj.get_world_pose())
        grasp_to_bin_T = cortex_math_util.invert_T(grasp_T).dot(bin_T)
        desired_bin_T = eff_T.dot(grasp_to_bin_T)
        position, orientation = cortex_math_util.T2pq(desired_bin_T)
        offset = float(np.linalg.norm(eff_T[:3, 3] - grasp_T[:3, 3]))
        return position, orientation, offset

    def place_active_bin_grasp_at_effector(context, active_bin=None):
        active_bin = active_bin or getattr(context, "active_bin", None)
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
            eff_T = self.context.robot.arm.get_fk_T()
            bin_T = cortex_math_util.pq2T(*active_bin.bin_obj.get_world_pose())
            active_bin.demo_attached = True
            active_bin.demo_attach_T = cortex_math_util.invert_T(eff_T).dot(bin_T)
            active_bin.is_attached = True
            active_bin.demo_pick_stationed = True
            self.context.demo_carried_bin = active_bin
            self.context.demo_pre_grip_bin = None
            stop_dynamic_prim(active_bin.bin_obj)
            set_kinematic_for_demo(active_bin.bin_obj, True)
            print(f"[HarimDemo] demo-attached {active_bin.bin_obj.name}", flush=True)

        def step(self):
            return None

    class DemoReleaseBin(DfState):
        def __init__(self, release_duration=0.35):
            self.release_duration = release_duration
            self.entry_time = None
            self.released_bin = None
            self.target_p = None

        def enter(self):
            self.entry_time = get_demo_time(self.context)
            print("<open gripper>", flush=True)
            force_open_suction_gripper(self.context)

            active_bin = self.context.active_bin
            active_bin = get_demo_carried_bin(self.context) or active_bin
            if active_bin is None:
                return

            target_index = len(self.context.stacked_bins)
            self.target_p = get_demo_stack_coordinate(self.context, target_index)
            self.released_bin = active_bin
            active_bin.demo_attached = False
            active_bin.demo_attach_T = None
            active_bin.is_attached = False
            set_kinematic_for_demo(active_bin.bin_obj, True)
            stop_dynamic_prim(active_bin.bin_obj)
            mark_demo_bin_released(self.context, active_bin, self.target_p, UPSIDE_DOWN_BIN_QUAT)
            active_bin.bin_obj.set_world_pose(position=self.target_p, orientation=UPSIDE_DOWN_BIN_QUAT)
            stop_dynamic_prim(active_bin.bin_obj)
            set_kinematic_for_demo(active_bin.bin_obj, True)
            if args.self_test_require_gripper_open_after_release:
                record_release_gripper_state(self.context)
            print(f"[HarimDemo] demo-placed {active_bin.bin_obj.name} at {self.target_p.tolist()}", flush=True)

        def step(self):
            if self.released_bin is None or self.target_p is None:
                return None
            force_open_suction_gripper(self.context)
            mark_demo_bin_released(self.context, self.released_bin, self.target_p, UPSIDE_DOWN_BIN_QUAT)
            hold_demo_released_bin_at_target(self.context)
            if get_demo_time(self.context) - self.entry_time < self.release_duration:
                return self
            return None

        def exit(self):
            self.entry_time = None
            self.released_bin = None
            self.target_p = None

    class DemoTimedArmLift(DfState):
        def __init__(self, height, duration):
            self.height = height
            self.duration = duration
            self.entry_time = None
            self.target_pq = None

        def enter(self):
            self.entry_time = get_demo_time(self.context)
            self.target_pq = self.context.robot.arm.get_fk_pq()
            self.target_pq.p[2] += self.height

        def step(self):
            hold_demo_released_bin_at_target(self.context)
            self.context.robot.arm.send(MotionCommand(self.target_pq))
            if get_demo_time(self.context) - self.entry_time < self.duration:
                return self
            return None

        def exit(self):
            self.entry_time = None
            self.target_pq = None

    class DemoTimedArmJointSettle(DfState):
        def __init__(self, duration=POST_RELEASE_JOINT_SETTLE_DURATION):
            self.duration = duration
            self.entry_time = None
            self.start_positions = None
            self.target_positions = None
            self.active = False

        def enter(self):
            self.entry_time = get_demo_time(self.context)
            self.active = False
            robot = self.context.robot
            try:
                self.start_positions = np.array(robot.get_joint_positions(), dtype=float)
                self.target_positions = np.array(robot.default_config, dtype=float)
            except Exception as exc:
                print(f"[HarimDemo] joint settle skipped: {exc}", flush=True)
                return
            if self.start_positions.shape != self.target_positions.shape:
                print(
                    f"[HarimDemo] joint settle skipped: shape {self.start_positions.shape} "
                    f"!= {self.target_positions.shape}",
                    flush=True,
                )
                return
            self.active = True
            self.context.demo_joint_settle_count = int(getattr(self.context, "demo_joint_settle_count", 0)) + 1
            robot.arm.clear()

        def step(self):
            hold_demo_released_bin_at_target(self.context)
            if not self.active:
                return None
            elapsed = get_demo_time(self.context) - self.entry_time
            t = clamp(elapsed / max(self.duration, 1e-6), 0.0, 1.0)
            joint_positions = lerp(self.start_positions, self.target_positions, smoothstep(t))
            robot = self.context.robot
            try:
                robot.set_joint_positions(joint_positions)
                if hasattr(robot, "set_joint_velocities"):
                    robot.set_joint_velocities(np.zeros_like(joint_positions))
                robot.arm.soft_reset()
            except Exception as exc:
                print(f"[HarimDemo] joint settle aborted: {exc}", flush=True)
                return None
            if t < 1.0:
                return self
            return None

        def exit(self):
            try:
                self.context.robot.arm.soft_reset()
            except Exception:
                pass
            self.entry_time = None
            self.start_positions = None
            self.target_positions = None
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
            self.entry_time = get_demo_time(self.context)
            self.target_position = self.position.copy()
            print(f"[HarimDemo] {self.label} start", flush=True)

        def step(self):
            hold_demo_released_bin_at_target(self.context)
            self.context.robot.arm.send(
                MotionCommand(target_position=self.target_position, posture_config=self.context.robot.default_config)
            )
            current_position = np.array(self.context.robot.arm.get_fk_p(), dtype=float)
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
            self.entry_time = get_demo_time(self.context)
            print(f"[HarimDemo] {self.label} start", flush=True)
            restore_demo_carried_active_bin(self.context)
            self.state.enter()

        def step(self):
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
            self.context.robot.arm.clear()

        def step(self):
            return self

    class DemoPickAndPlaceBin(DfStateMachineDecider):
        def __init__(self):
            super().__init__(
                DfStateSequence(
                    [
                        DemoTimedState(behavior.ReachToPick(), max_duration=REACH_PICK_MAX_DURATION, label="reach_pick"),
                        DemoSettleBinAtGripper(min_duration=0.25, max_duration=1.10),
                        DfSetLockState(set_locked_to=True, decider=self),
                        DemoAttachBin(),
                        DemoTimedArmLift(height=0.24, duration=0.35),
                        DemoTimedState(behavior.ReachToPlace(), max_duration=REACH_PLACE_MAX_DURATION, label="reach_place"),
                        DfWaitState(wait_time=0.15),
                        DemoReleaseBin(),
                        DemoTimedArmLift(height=POST_RELEASE_CLEARANCE_LIFT, duration=0.35),
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
        if not getattr(active_bin, "demo_attached", False):
            return
        attach_T = getattr(active_bin, "demo_attach_T", None)
        if attach_T is None:
            return
        context.active_bin = active_bin
        bin_T = context.robot.arm.get_fk_T().dot(attach_T)
        position, orientation = cortex_math_util.T2pq(bin_T)
        active_bin.bin_obj.set_world_pose(position=position, orientation=orientation)
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

    create_drop_slide_workstation()

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
        lift_plate=lift_plate,
        lift_plate_parts=lift_plate_parts,
        pallet_parts=pallet_parts,
        stack_coordinates=stack_coordinates,
        args=args,
        completion_signal=completion_signal,
    )

    world.reset()
    world.play()
    decider_network.context.stack_coordinates = clone_stack_coordinates(stack_coordinates)
    decider_network.context.demo_stack_coordinates = clone_stack_coordinates(stack_coordinates)
    decider_network.context.demo_sim_time = 0.0
    decider_network.context.demo_max_pre_grip_offset = 0.0
    decider_network.context.demo_max_return_ready_error = 0.0
    decider_network.context.demo_max_release_drift = 0.0
    decider_network.context.demo_release_gripper_samples = 0
    decider_network.context.demo_release_gripper_not_open_samples = 0
    decider_network.context.demo_release_gripped_object_max = 0
    decider_network.context.demo_release_gripper_probe_failures = 0
    decider_network.context.demo_joint_settle_count = 0
    orchestrator.reset_visual_state()

    def force_self_test_stack_complete():
        if not args.self_test_force_stack_complete:
            return
        if self_test_payload:
            decider_network.context.stacked_bins = [SelfTestBinState(item) for item in self_test_payload]
            decider_network.context.stack_coordinates = [
                np.array(item.get_world_pose()[0], dtype=float) for item in self_test_payload
            ]

    def step_demo_frame():
        physics_dt = world.get_physics_dt()
        decider_network.context.demo_sim_time = getattr(decider_network.context, "demo_sim_time", 0.0) + physics_dt
        hold_demo_released_bin_at_target(decider_network.context)
        world.step(render=not args.headless)
        sync_demo_attached_bin(decider_network.context)
        hold_demo_released_bin_at_target(decider_network.context)
        force_self_test_stack_complete()
        orchestrator.step(physics_dt)

    self_test_failure_message = None
    try:
        if args.self_test_frames > 0:
            for _frame_count in range(args.self_test_frames):
                step_demo_frame()
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
            max_payload_lift = float(getattr(orchestrator, "max_payload_lift_observed", 0.0))
            if args.self_test_min_payload_lift > 0 and max_payload_lift < args.self_test_min_payload_lift:
                self_test_failures.append(
                    f"payload lift {max_payload_lift:.4f} m was below "
                    f"{args.self_test_min_payload_lift:.4f} m"
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
                    f"release_gripper_samples={release_gripper_samples}; "
                    f"release_gripper_not_open={release_gripper_not_open}; "
                    f"release_gripped_object_max={release_gripped_object_max}; "
                    f"release_gripper_probe_failures={release_gripper_probe_failures}; "
                    f"joint_settle_count={joint_settle_count}; "
                    f"max_stack_lateral_gap={max_stack_lateral_gap:.4f}; "
                    f"min_stack_lateral_gap={min_stack_lateral_gap:.4f}; "
                    f"max_stack_support_gap={max_stack_support_gap:.4f}; "
                    f"min_stack_support_gap={min_stack_support_gap:.4f}; "
                    f"min_stack_pallet_margin={min_stack_pallet_margin:.4f}; "
                    f"max_stack_pallet_overhang={max_stack_pallet_overhang:.4f}; "
                    f"max_payload_lift={max_payload_lift:.4f}; "
                    f"max_dropped_payload_drift={max_dropped_payload_drift:.4f}; "
                    f"amr_exit_clearance={amr_exit_clearance:.4f}; "
                    f"max_lift_contact_gap={max_lift_contact_gap:.4f}; "
                    f"min_lift_contact_gap={min_lift_contact_gap:.4f}; "
                    f"pallet_tunnel_clearance={pallet_tunnel_clearance:.4f}; "
                    f"lift_fork_inner_gap={lift_fork_inner_gap:.4f}; "
                    f"drop_support_gap={drop_support_gap:.4f}; "
                    f"drop_lane_clearance={drop_lane_clearance:.4f}; "
                    f"drop_runner_clearance={drop_runner_clearance:.4f}; "
                    f"drop_fork_clearance={drop_fork_clearance:.4f}",
                    flush=True,
                )
        else:
            while simulation_app.is_running():
                step_demo_frame()
    except Exception as exc:
        print(f"[HarimDemo] simulation aborted: {exc}", flush=True)
        traceback.print_exc()
        print("[HarimDemo] preserving failure exit; skipping SimulationApp.close()", flush=True)
        os._exit(1)
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
