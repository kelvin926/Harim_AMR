import argparse
import math
import os
import random
from enum import Enum, auto
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_PICKUP_X = 0.82
DEFAULT_PICKUP_Y = -0.31
DEFAULT_DROP_X = DEFAULT_PICKUP_X + 10.6
DEFAULT_DROP_Y = DEFAULT_PICKUP_Y
DEFAULT_AMR_Z = -1.05
DEFAULT_LIFT_HEIGHT = 0.11
DEFAULT_MOVE_SPEED = 0.65
AMR_START_STANDOFF = 3.2
AMR_APPROACH_STANDOFF = 1.05
SLIDE_EXIT_DISTANCE = 1.8
UPSIDE_DOWN_BIN_QUAT = np.array([0.0, 0.0, 1.0, 0.0], dtype=float)
REFERENCE_STATION_FLOOR_Z = -1.18180
REFERENCE_STATION_WIDTH = 4.80
REFERENCE_STATION_DEPTH = 0.58
REFERENCE_STATION_HEIGHT = 0.66
REFERENCE_STATION_BLACK = np.array([0.008, 0.009, 0.009], dtype=float)
REFERENCE_STATION_BLACK_HIGHLIGHT = np.array([0.050, 0.052, 0.050], dtype=float)
REFERENCE_STATION_SILVER = np.array([0.70, 0.71, 0.68], dtype=float)
REFERENCE_STATION_RAIL_GROOVE = np.array([0.42, 0.43, 0.41], dtype=float)
REFERENCE_STATION_WHITE_PANEL = np.array([0.90, 0.90, 0.86], dtype=float)
REFERENCE_STATION_WARNING_YELLOW = np.array([0.95, 0.72, 0.08], dtype=float)
IMAGE_MATCHED_SLIDING_STATION_USD = PROJECT_ROOT / "sliding_station.usd"
BACKGROUND_CARD_BOX_PRIM_PATH = "/World/Background/SM_CardBoxA_02"
BACKGROUND_CARD_BOX_NAME_PREFIX = "SM_CardBoxA_"
SMALL_KLT_VISUAL_NAME_PREFIX = "SmallKLT_Visual_"
SMALL_KLT_VISUAL_INDEX_MIN = 2
SMALL_KLT_VISUAL_INDEX_MAX = 164
SMALL_KLT_VISUAL_ROOT_NAME = "Visuals"
DYNAMIC_SMALL_KLT_VISUAL_ROOT_PREFIXES = (
    "/World/Ur10Table/bins/",
    "/World/HarimDemo/SelfTestPayload_",
)


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


def is_small_klt_visual_box_prim_name(name):
    if not name.startswith(SMALL_KLT_VISUAL_NAME_PREFIX):
        return False
    suffix = name[len(SMALL_KLT_VISUAL_NAME_PREFIX) :]
    return suffix.isdigit() and SMALL_KLT_VISUAL_INDEX_MIN <= int(suffix) <= SMALL_KLT_VISUAL_INDEX_MAX


def is_background_card_box_prim_name(name):
    if not name.startswith(BACKGROUND_CARD_BOX_NAME_PREFIX):
        return False
    suffix = name[len(BACKGROUND_CARD_BOX_NAME_PREFIX) :]
    return suffix.isdigit()


def is_dynamic_small_klt_visual_root_path(path):
    return path.endswith(f"/{SMALL_KLT_VISUAL_ROOT_NAME}") and any(
        path.startswith(prefix) for prefix in DYNAMIC_SMALL_KLT_VISUAL_ROOT_PREFIXES
    )


def is_cardboard_box_style_target_prim(prim):
    name = prim.GetName()
    if is_small_klt_visual_box_prim_name(name) or is_background_card_box_prim_name(name):
        return True
    return name == SMALL_KLT_VISUAL_ROOT_NAME and is_dynamic_small_klt_visual_root_path(str(prim.GetPath()))


def resolve_background_card_box_style(stage, source_path=BACKGROUND_CARD_BOX_PRIM_PATH):
    from pxr import Usd, UsdGeom, UsdShade

    source = stage.GetPrimAtPath(source_path)
    if not source.IsValid():
        raise RuntimeError(f"Background card box source prim was not found: {source_path}")

    source_material = None
    source_display_color = None
    for prim in Usd.PrimRange(source):
        if source_material is None:
            material, _relationship = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            if material is not None and material.GetPrim().IsValid():
                source_material = material
        if source_display_color is None and prim.IsA(UsdGeom.Gprim):
            display_color_attr = UsdGeom.Gprim(prim).GetDisplayColorAttr()
            if display_color_attr.HasAuthoredValueOpinion():
                source_display_color = display_color_attr.Get()
        if source_material is not None and source_display_color is not None:
            break

    if source_material is None and source_display_color is None:
        raise RuntimeError(f"No material or displayColor was found on background card box: {source_path}")
    return source_material, source_display_color


def apply_background_card_box_style_to_box_targets(
    stage,
    root_path,
    source_material,
    source_display_color,
    styled_paths,
):
    from pxr import Usd, UsdGeom, UsdShade

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0

    applied_count = 0
    for prim in Usd.PrimRange(root):
        target_root_path = str(prim.GetPath())
        if any(target_root_path.startswith(styled_path + "/") for styled_path in styled_paths):
            continue
        if not is_cardboard_box_style_target_prim(prim):
            continue

        if target_root_path in styled_paths:
            continue

        for target in Usd.PrimRange(prim):
            if source_material is not None:
                UsdShade.MaterialBindingAPI.Apply(target).Bind(source_material)
            if source_display_color is not None and target.IsA(UsdGeom.Gprim):
                UsdGeom.Gprim(target).CreateDisplayColorAttr().Set(source_display_color)

        styled_paths.add(target_root_path)
        applied_count += 1
    return applied_count


def create_front_triangle_label(stage, prim_path, *, center, width, height, color):
    from pxr import Gf, UsdGeom

    center = np.array(center, dtype=float)
    half_width = width * 0.5
    half_height = height * 0.5
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(
        [
            Gf.Vec3f(float(center[0]), float(center[1]), float(center[2] + half_height)),
            Gf.Vec3f(float(center[0] - half_width), float(center[1]), float(center[2] - half_height)),
            Gf.Vec3f(float(center[0] + half_width), float(center[1]), float(center[2] - half_height)),
        ]
    )
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    mesh.CreateDoubleSidedAttr(True)
    UsdGeom.Gprim(mesh.GetPrim()).CreateDisplayColorAttr(
        [Gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))]
    )


def create_extruded_xz_polygon(stage, prim_path, *, points_xz, y_min, y_max, color):
    from pxr import Gf, UsdGeom

    front_points = [Gf.Vec3f(float(x), float(y_min), float(z)) for x, z in points_xz]
    back_points = [Gf.Vec3f(float(x), float(y_max), float(z)) for x, z in points_xz]
    count = len(points_xz)
    face_counts = [count, count]
    face_indices = list(range(count)) + list(range(count, count * 2))
    for idx in range(count):
        next_idx = (idx + 1) % count
        face_counts.append(4)
        face_indices.extend([idx, next_idx, next_idx + count, idx + count])

    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(front_points + back_points)
    mesh.CreateFaceVertexCountsAttr(face_counts)
    mesh.CreateFaceVertexIndicesAttr(face_indices)
    mesh.CreateDoubleSidedAttr(True)
    UsdGeom.Gprim(mesh.GetPrim()).CreateDisplayColorAttr(
        [Gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))]
    )


def create_reference_slide_station(*, stage, VisualCuboid, station_root, center_x, center_y, floor_z, name_prefix):
    import omni

    total_width = REFERENCE_STATION_WIDTH
    depth = REFERENCE_STATION_DEPTH
    height = REFERENCE_STATION_HEIGHT
    support_width = 1.05
    central_width = total_width - support_width * 2.0
    end_face_width = 0.40
    bottom_z = floor_z + 0.030
    top_z = bottom_z + height
    front_y = center_y - depth * 0.5
    mid_y = center_y
    bridge_depth = depth * 0.43
    bridge_height = height * 0.145
    bridge_z = bottom_z + height * 0.66
    bridge_bottom = bridge_z - bridge_height * 0.5
    bridge_top = bridge_z + bridge_height * 0.5
    left_outer_x = center_x - total_width * 0.5
    right_outer_x = center_x + total_width * 0.5
    left_inner_x = center_x - central_width * 0.5
    right_inner_x = center_x + central_width * 0.5

    omni.kit.commands.execute("CreatePrim", prim_path=station_root, prim_type="Xform")

    def add_box(suffix, position, scale, color, orientation=None):
        VisualCuboid(
            f"{station_root}/{suffix}",
            name=f"{name_prefix}_{suffix}",
            position=np.array(position, dtype=float),
            orientation=np.array(orientation if orientation is not None else [1.0, 0.0, 0.0, 0.0], dtype=float),
            scale=np.array(scale, dtype=float),
            color=np.array(color, dtype=float),
        )

    add_box(
        "CentralBlackBridge",
        [center_x, front_y + bridge_depth * 0.56, bridge_z],
        [central_width + 0.08, bridge_depth, bridge_height],
        REFERENCE_STATION_BLACK,
    )
    add_box(
        "CentralBridgeFrontGloss",
        [center_x, front_y - 0.013, bridge_z + bridge_height * 0.22],
        [central_width * 0.98, 0.018, bridge_height * 0.18],
        REFERENCE_STATION_BLACK_HIGHLIGHT,
    )

    for side_name, side_sign, outer_x, inner_x in (
        ("Left", -1.0, left_outer_x, left_inner_x),
        ("Right", 1.0, right_outer_x, right_inner_x),
    ):
        inward = -side_sign
        support_span = abs(inner_x - outer_x)
        face_center_x = outer_x + inward * end_face_width * 0.5
        body_front_min_y = front_y - 0.035
        body_back_max_y = front_y + depth * 0.44
        rail_length = support_span * 0.70
        rail_center_x = outer_x + inward * support_span * 0.55
        rail_z = top_z - height * 0.085
        rail_pitch = pitch_to_quat(-inward * math.radians(11.0))
        web_pitch = pitch_to_quat(inward * math.radians(20.0))

        body_profile = [
            (outer_x, bottom_z + 0.015),
            (outer_x, bottom_z + height * 0.56),
            (outer_x + inward * support_span * 0.14, bottom_z + height * 0.56),
            (outer_x + inward * support_span * 0.25, bottom_z + height * 0.51),
            (outer_x + inward * support_span * 0.48, top_z - height * 0.075),
            (inner_x, bridge_top),
            (inner_x, bridge_bottom),
            (outer_x + inward * support_span * 0.62, bridge_bottom - height * 0.015),
            (outer_x + inward * support_span * 0.34, bottom_z + height * 0.27),
            (outer_x + inward * support_span * 0.20, bottom_z + height * 0.145),
            (outer_x + inward * support_span * 0.20, bottom_z + 0.015),
        ]
        create_extruded_xz_polygon(
            stage,
            f"{station_root}/{side_name}AngularFrontSilhouette",
            points_xz=body_profile,
            y_min=body_front_min_y,
            y_max=body_back_max_y,
            color=REFERENCE_STATION_BLACK,
        )

        add_box(
            f"{side_name}RearBulkMass",
            [outer_x + inward * support_span * 0.38, mid_y + depth * 0.08, bottom_z + height * 0.29],
            [support_span * 0.72, depth * 0.58, height * 0.42],
            REFERENCE_STATION_BLACK,
        )
        add_box(
            f"{side_name}LowerFrontFoot",
            [outer_x + inward * support_span * 0.42, front_y + depth * 0.16, bottom_z + height * 0.16],
            [support_span * 0.76, depth * 0.32, height * 0.25],
            REFERENCE_STATION_BLACK,
        )
        add_box(
            f"{side_name}TopSlopedArm",
            [outer_x + inward * support_span * 0.56, front_y + bridge_depth * 0.55, top_z - height * 0.18],
            [support_span * 0.86, bridge_depth, height * 0.105],
            REFERENCE_STATION_BLACK,
            rail_pitch,
        )
        add_box(
            f"{side_name}FrontSlopedWebHighlight",
            [outer_x + inward * support_span * 0.45, front_y + 0.030, bottom_z + height * 0.32],
            [support_span * 0.62, 0.095, height * 0.24],
            REFERENCE_STATION_BLACK_HIGHLIGHT,
            web_pitch,
        )
        add_box(
            f"{side_name}InnerHangingBracket",
            [inner_x - inward * 0.065, front_y + depth * 0.18, bridge_bottom - height * 0.12],
            [0.13, depth * 0.15, height * 0.25],
            REFERENCE_STATION_BLACK,
        )
        add_box(
            f"{side_name}WhiteLowerFrontPanel",
            [face_center_x, front_y - 0.017, bottom_z + height * 0.18],
            [0.145, 0.018, height * 0.33],
            REFERENCE_STATION_WHITE_PANEL,
        )

        warning_center = [face_center_x, front_y - 0.030, bottom_z + height * 0.43]
        create_front_triangle_label(
            stage,
            f"{station_root}/{side_name}YellowWarningTriangle",
            center=warning_center,
            width=0.115,
            height=0.102,
            color=REFERENCE_STATION_WARNING_YELLOW,
        )
        add_box(
            f"{side_name}WarningIconBar",
            [warning_center[0], front_y - 0.039, warning_center[2] + 0.002],
            [0.010, 0.010, 0.033],
            REFERENCE_STATION_BLACK,
        )
        add_box(
            f"{side_name}WarningIconDot",
            [warning_center[0], front_y - 0.039, warning_center[2] - 0.035],
            [0.014, 0.010, 0.010],
            REFERENCE_STATION_BLACK,
        )

        add_box(
            f"{side_name}SilverLinearRail",
            [rail_center_x, front_y + bridge_depth * 0.54, rail_z],
            [rail_length, 0.155, 0.035],
            REFERENCE_STATION_SILVER,
            rail_pitch,
        )
        for stopper_name, stopper_offset in (
            ("OuterRailStopper", -inward * rail_length * 0.46),
            ("InnerRailStopper", inward * rail_length * 0.46),
        ):
            add_box(
                f"{side_name}{stopper_name}",
                [rail_center_x + stopper_offset, front_y + bridge_depth * 0.54, rail_z + 0.034],
                [0.080, 0.175, 0.043],
                REFERENCE_STATION_BLACK,
                rail_pitch,
            )
        for groove_idx, offset in enumerate(np.linspace(-0.30, 0.30, 5)):
            add_box(
                f"{side_name}RailGroove{groove_idx}",
                [rail_center_x + inward * rail_length * float(offset), front_y + bridge_depth * 0.44, rail_z + 0.002],
                [0.010, 0.014, 0.039],
                REFERENCE_STATION_RAIL_GROOVE,
                rail_pitch,
            )

    print(
        f"[HarimDemo] created {name_prefix} reference slide station: "
        f"width={total_width:.2f}, height={height:.2f}, open_center={central_width:.2f}"
    )


def place_image_matched_sliding_station(
    *,
    stage,
    station_root,
    center_x,
    center_y,
    floor_z,
    name_prefix,
):
    from pxr import Gf, UsdGeom

    if not IMAGE_MATCHED_SLIDING_STATION_USD.exists():
        raise RuntimeError(
            "Image-matched sliding station USD was not found: "
            f"{IMAGE_MATCHED_SLIDING_STATION_USD}. Run create_sliding_station.py first."
        )

    station_prim = UsdGeom.Xform.Define(stage, station_root).GetPrim()
    station_prim.GetReferences().AddReference(str(IMAGE_MATCHED_SLIDING_STATION_USD), "/World/SlidingStation")

    xform = UsdGeom.Xformable(station_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(float(center_x), float(center_y), float(floor_z)))

    print(
        f"[HarimDemo] placed {name_prefix} image-matched sliding station: "
        f"{IMAGE_MATCHED_SLIDING_STATION_USD}"
    )


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


def clamp(value, low, high):
    return max(low, min(high, value))


def lerp(a, b, t):
    return a + (b - a) * t


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)


def pitch_to_quat(pitch):
    half = pitch * 0.5
    return np.array([math.cos(half), 0.0, math.sin(half), 0.0], dtype=float)


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


def random_bin_spawn_transform():
    x = random.uniform(-0.15, 0.15)
    y = 1.5
    z = -0.15
    position = np.array([x, y, z], dtype=float)
    orientation = quat_multiply(yaw_to_quat(random.uniform(-0.05, 0.05)), UPSIDE_DOWN_BIN_QUAT)
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
        amr_lift_prim=None,
    ):
        self.world = world
        self.context = context
        self.task = task
        self.amr = amr_prim
        self.amr_lift = amr_lift_prim
        self.lift_plate = lift_plate
        self.pallet_parts = pallet_parts
        self.stack_coordinates = stack_coordinates
        self.args = args

        self.state = TransferState.WAIT_STACK_COMPLETE
        self.state_time = 0.0
        self.completed_cycles = 0
        self.carrying = False
        self.attached_items = []
        self.item_offsets = {}
        self.pallet_base_offsets = {}
        self.dropped_item_poses = {}
        self.dropped_pallet_poses = {}
        self.initial_pallet_poses = self._capture_pallet_poses()

        self.stack_center = self._compute_stack_center()
        self.amr_yaw = 0.0
        self.lift_offset = 0.0
        self.amr_lift_base_offset = None
        self.amr_lift_orientation = None
        self.move_target = None

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
        print(f"[HarimDemo] state -> {state.name}")

    def reset_visual_state(self):
        self.carrying = False
        self.attached_items = []
        self.item_offsets = {}
        self.pallet_base_offsets = {}
        self.dropped_item_poses = {}
        self.dropped_pallet_poses = {}
        self.lift_offset = 0.0
        self.set_amr_pose(self.start_pose)
        self._set_lift_plate_pose()
        self._reset_pallet_pose()

    def set_amr_pose(self, position):
        self.amr.set_world_pose(position=np.array(position, dtype=float), orientation=yaw_to_quat(self.amr_yaw))

    def get_amr_position(self):
        position, _orientation = self.amr.get_world_pose()
        return np.array(position, dtype=float)

    def _lift_plate_position(self):
        amr_pos = self.get_amr_position()
        return amr_pos + np.array([0.0, 0.0, 0.33 + self.lift_offset], dtype=float)

    def _set_lift_plate_pose(self):
        if self.lift_plate is not None:
            self.lift_plate.set_world_pose(position=self._lift_plate_position(), orientation=yaw_to_quat(self.amr_yaw))
        self._set_actual_lift_pose()

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
        for part in self.pallet_parts:
            data = self.initial_pallet_poses.get(part.name)
            if data is None:
                continue
            position, orientation = data
            self._set_pallet_position(part, position)

    def _move_amr_toward_target(self, dt):
        if self.move_target is None:
            return True

        current = self.get_amr_position()
        delta = self.move_target - current
        distance = float(np.linalg.norm(delta[:2]))
        if distance <= 0.01:
            self.set_amr_pose(self.move_target)
            self._set_lift_plate_pose()
            self._sync_payload_pose()
            return True

        step = min(distance, self.args.move_speed * max(dt, 1.0 / 120.0))
        next_pos = current + delta * (step / max(distance, 1e-6))
        self.set_amr_pose(next_pos)
        self._set_lift_plate_pose()
        self._sync_payload_pose()
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
            except Exception as exc:
                print(f"[HarimDemo] could not lift stacked item: {exc}")

    def _capture_pallet_poses(self):
        poses = {}
        for part in self.pallet_parts:
            position, orientation = part.get_world_pose()
            poses[part.name] = (np.array(position, dtype=float), orientation)
        return poses

    def _apply_lift_delta_to_pallet(self, dz):
        if abs(dz) <= 1e-6:
            return
        for part in self.pallet_parts:
            position, orientation = part.get_world_pose()
            lifted = np.array(position, dtype=float)
            lifted[2] += dz
            self._set_pallet_position(part, lifted)

    def _set_pallet_position(self, part, position):
        part.set_world_pose(position=np.array(position, dtype=float))

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
            self._set_pallet_position(part, amr_pos + offset)

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
                item.set_world_pose(position=pos, orientation=orient)
                self._stop_dynamic_item(item)
            except Exception as exc:
                print(f"[HarimDemo] could not hold dropped item: {exc}")
        for part, pos, orient in self.dropped_pallet_poses.values():
            self._set_pallet_position(part, pos)

    def _sync_payload_pose(self):
        if self.carrying:
            self._update_attached_items()
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
        self.context.stack_coordinates = self.stack_coordinates
        self.reset_visual_state()
        self._transition(TransferState.WAIT_STACK_COMPLETE)

    def step(self, dt):
        self.state_time += dt
        self._set_lift_plate_pose()
        self._sync_payload_pose()

        if self.state == TransferState.WAIT_STACK_COMPLETE:
            if getattr(self.context, "stack_complete", False):
                print("[HarimDemo] stack_complete detected")
                self._transition(TransferState.ARM_SETTLE)

        elif self.state == TransferState.ARM_SETTLE:
            if self.state_time >= 1.0:
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
            t = clamp(self.state_time / 1.0, 0.0, 1.0)
            previous_offset = self.lift_offset
            self.lift_offset = lerp(0.0, self.args.lift_height, t)
            dz = self.lift_offset - previous_offset
            self._set_lift_plate_pose()
            self._apply_lift_delta_to_stack(dz)
            self._apply_lift_delta_to_pallet(dz)
            if t >= 1.0:
                self._transition(TransferState.ATTACH)

        elif self.state == TransferState.ATTACH:
            self._attach_assembly()
            self._transition(TransferState.MOVE_TO_DROP)

        elif self.state == TransferState.LIFT_DOWN:
            t = clamp(self.state_time / 1.0, 0.0, 1.0)
            previous_offset = self.lift_offset
            self.lift_offset = lerp(self.args.lift_height, 0.0, t)
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
            if t >= 1.0:
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

    from isaacsim.core.api.objects.capsule import VisualCapsule
    from isaacsim.core.api.objects.cuboid import VisualCuboid
    from isaacsim.core.api.objects.sphere import VisualSphere
    from isaacsim.core.api.tasks.base_task import BaseTask
    from isaacsim.core.prims import SingleXFormPrim
    from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.nucleus import get_assets_root_path
    import isaacsim.cortex.framework.math_util as cortex_math_util
    from isaacsim.cortex.framework.cortex_world import CortexWorld
    from isaacsim.cortex.framework.cortex_rigid_prim import CortexRigidPrim
    from isaacsim.cortex.framework.df import DfDecider, DfDecision, DfNetwork, DfStateMachineDecider
    from isaacsim.cortex.framework.dfb import make_go_home
    from isaacsim.cortex.framework.robot import CortexUr10
    from isaacsim.cortex.behaviors.ur10 import bin_stacking_behavior as behavior
    from pxr import Gf, Usd, UsdGeom

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

        def _spawn_bin(self, rigid_bin):
            position, orientation = random_bin_spawn_transform()
            rigid_bin.set_world_pose(position=position, orientation=orientation)
            rigid_bin.set_linear_velocity(np.array([0.0, -0.30, 0.0], dtype=float))
            rigid_bin.set_visibility(True)

        def post_reset(self) -> None:
            if len(self.bins) > 0:
                for rigid_bin in self.bins:
                    self.scene.remove_object(rigid_bin.name)
                self.bins.clear()
            self.on_conveyor = None

        def pre_step(self, time_step_index, simulation_time) -> None:
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
                self.on_conveyor = self.scene.add(CortexRigidPrim(name=name, prim_path=prim_path))
                self._spawn_bin(self.on_conveyor)
                self.bins.append(self.on_conveyor)

        def world_cleanup(self):
            self.bins = []
            self.on_conveyor = None

    class NoFlipDispatch(DfDecider):
        def __init__(self):
            super().__init__()
            self.add_child("pick_bin", behavior.PickBin())
            self.add_child("place_bin", behavior.PlaceBin())
            self.add_child("go_home", make_go_home())
            self.add_child("do_nothing", DfStateMachineDecider(behavior.DoNothing()))

        def decide(self):
            if self.context.stack_complete:
                return DfDecision("go_home")
            if self.context.has_active_bin:
                if not self.context.active_bin.is_attached:
                    return DfDecision("pick_bin")
                return DfDecision("place_bin")
            return DfDecision("go_home")

    def make_no_flip_decider_network(robot, monitor_fn):
        return DfNetwork(NoFlipDispatch(), context=behavior.BinStackingContext(robot, monitor_fn))

    usd_context = omni.usd.get_context()
    world = CortexWorld()

    ur10_assets = Ur10Assets(assets_root)
    add_reference_to_stage(ur10_assets.ur10_table_usd, "/World/Ur10Table")
    add_reference_to_stage(ur10_assets.background_usd, "/World/Background")
    wait_for_stage_loading(simulation_app, usd_context, "UR10 palletizing scene")
    deactivate_stage_prims_containing(usd_context.get_stage(), "/World/Ur10Table", ("flip", "pallet_holder"))
    SingleXFormPrim("/World/Background", position=[10.00, 2.00, -1.18180], orientation=[0.7071, 0, 0, 0.7071])

    card_box_material, card_box_display_color = resolve_background_card_box_style(usd_context.get_stage())
    styled_cardboard_box_paths = set()

    def apply_cardboard_box_color(root_path="/World"):
        applied_count = apply_background_card_box_style_to_box_targets(
            usd_context.get_stage(),
            root_path,
            card_box_material,
            card_box_display_color,
            styled_cardboard_box_paths,
        )
        if applied_count > 0:
            print(f"[HarimDemo] matched {applied_count} cardboard box visuals to background card box color")

    apply_cardboard_box_color("/World")

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

    stack_coordinates = make_stack_coordinates(args.stack_cols, args.stack_rows, args.stack_layers)
    task = BinStackingTask("/World/Ur10Table", ur10_assets)
    task.set_up_scene(world.scene)
    world.add_task(task)

    decider_network = make_no_flip_decider_network(robot, lambda _diagnostic: None)
    decider_network.context.stack_coordinates = stack_coordinates
    world.add_decider_network(decider_network)

    harim_root = "/World/HarimDemo"
    omni.kit.commands.execute("CreatePrim", prim_path=harim_root, prim_type="Xform")

    place_image_matched_sliding_station(
        stage=usd_context.get_stage(),
        station_root=f"{harim_root}/PickupSlideStation",
        center_x=args.pickup_x,
        center_y=args.pickup_y,
        floor_z=REFERENCE_STATION_FLOOR_Z,
        name_prefix="PickupSlideStation",
    )
    place_image_matched_sliding_station(
        stage=usd_context.get_stage(),
        station_root=f"{harim_root}/DropSlideStation",
        center_x=args.drop_x,
        center_y=args.drop_y,
        floor_z=REFERENCE_STATION_FLOOR_Z,
        name_prefix="DropSlideStation",
    )
    wait_for_stage_loading(simulation_app, usd_context, "image-matched sliding stations")

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
        print(f"[HarimDemo] xformable iw_hub lift prim not found; using iw_hub asset body only: {amr_lift_path}")

    class ExampleStagePalletPrim:
        def __init__(self, prim, name):
            self.prim = prim
            self.name = name
            self.xformable = UsdGeom.Xformable(prim)
            self.translate_op = None
            for op in self.xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    self.translate_op = op
                    break
            if self.translate_op is None:
                self.translate_op = self.xformable.AddTranslateOp()

        def get_world_pose(self):
            matrix = self.xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = matrix.ExtractTranslation()
            return np.array([translation[0], translation[1], translation[2]], dtype=float), yaw_to_quat(0.0)

        def set_world_pose(self, position=None, orientation=None):
            if position is None:
                return
            position = np.array(position, dtype=float)
            self.translate_op.Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))

    def collect_example_pallet_parts():
        stage = usd_context.get_stage()
        root = stage.GetPrimAtPath("/World/Ur10Table")
        pallet_parts = []
        selected_paths = []
        if not root.IsValid():
            return pallet_parts
        for prim in Usd.PrimRange(root):
            name = prim.GetName().lower()
            path = str(prim.GetPath())
            if "pallet" not in name or "pallet_holder" in name:
                continue
            if any(path.startswith(selected_path + "/") for selected_path in selected_paths):
                continue
            if not UsdGeom.Xformable(prim).GetPrim().IsValid():
                continue
            pallet_parts.append(ExampleStagePalletPrim(prim, name=f"harim_example_pallet_{len(pallet_parts)}"))
            selected_paths.append(path)
        return pallet_parts

    lift_plate = None
    pallet_parts = collect_example_pallet_parts()
    if not pallet_parts:
        raise RuntimeError("No pallet prim was found in the UR10 example scene; custom pallet creation is disabled.")
    print(f"[HarimDemo] using {len(pallet_parts)} example pallet prims")

    class SelfTestBinState:
        def __init__(self, bin_obj):
            self.bin_obj = bin_obj

    self_test_payload = []
    if args.self_test_force_stack_complete:
        payload_specs = []
        for idx, coord in enumerate(stack_coordinates[: max(1, min(4, len(stack_coordinates)))]):
            payload_path = f"{harim_root}/SelfTestPayload_{idx}"
            add_reference_to_stage(usd_path=ur10_assets.small_klt_usd, prim_path=payload_path)
            payload_specs.append((idx, payload_path, coord))
        wait_for_stage_loading(simulation_app, usd_context, "self-test KLT payload")
        apply_cardboard_box_color(harim_root)
        for idx, payload_path, coord in payload_specs:
            payload = world.scene.add(
                CortexRigidPrim(name=f"harim_self_test_payload_{idx}", prim_path=payload_path)
            )
            payload.set_world_pose(position=np.array(coord, dtype=float), orientation=UPSIDE_DOWN_BIN_QUAT)
            payload.set_linear_velocity(np.zeros(3))
            payload.set_angular_velocity(np.zeros(3))
            self_test_payload.append(payload)

    orchestrator = HarimTransferOrchestrator(
        world=world,
        context=decider_network.context,
        task=task,
        amr_prim=amr,
        amr_lift_prim=amr_lift,
        lift_plate=lift_plate,
        pallet_parts=pallet_parts,
        stack_coordinates=stack_coordinates,
        args=args,
    )

    world.reset()
    world.play()
    decider_network.context.stack_coordinates = stack_coordinates
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
        world.step(render=not args.headless)
        apply_cardboard_box_color("/World/Ur10Table/bins")
        force_self_test_stack_complete()
        orchestrator.step(world.get_physics_dt())

    try:
        if args.self_test_frames > 0:
            for _frame_count in range(args.self_test_frames):
                step_demo_frame()
            print(f"[HarimDemo] self-test completed after {args.self_test_frames} frames", flush=True)
        else:
            while simulation_app.is_running():
                step_demo_frame()
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
