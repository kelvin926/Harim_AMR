import argparse
import math
import os
import random
from enum import Enum, auto
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_PICKUP_X = 1.25
DEFAULT_PICKUP_Y = -0.31
DEFAULT_DROP_X = DEFAULT_PICKUP_X + 10.6
DEFAULT_DROP_Y = DEFAULT_PICKUP_Y
DEFAULT_PALLET_X = 0.98
DEFAULT_AMR_Z = -1.18
DEFAULT_AMR_SCALE = np.array([0.75, 1.0, 1.35], dtype=float)
DEFAULT_LIFT_HEIGHT = 0.11
DEFAULT_MOVE_SPEED = 0.65
AMR_START_STANDOFF = 3.2
AMR_APPROACH_STANDOFF = 1.05
SLIDE_EXIT_DISTANCE = 1.8
UPSIDE_DOWN_BIN_QUAT = np.array([0.0, 0.0, 1.0, 0.0], dtype=float)
DROP_UR10_FOLDED_HOME_JOINT_TARGETS_DEG = {
    "shoulder_pan_joint": math.degrees(-1.57),
    "shoulder_lift_joint": math.degrees(-1.57),
    "elbow_joint": math.degrees(-1.57),
    "wrist_1_joint": math.degrees(-1.57),
    "wrist_2_joint": math.degrees(1.57),
    "wrist_3_joint": math.degrees(0.0),
}
UR10_FOLDED_LINK_CHAIN = (
    (
        "shoulder_pan_joint",
        "base_link",
        "shoulder_link",
        (0.0, 0.0, 0.1273),
        (-4.371139e-8, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    ),
    (
        "shoulder_lift_joint",
        "shoulder_link",
        "upper_arm_link",
        (0.0, 0.0, 0.0),
        (0.70710677, 0.70710677, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    ),
    (
        "elbow_joint",
        "upper_arm_link",
        "forearm_link",
        (-0.612, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    ),
    (
        "wrist_1_joint",
        "forearm_link",
        "wrist_1_link",
        (-0.5723, 0.0, 0.163941),
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    ),
    (
        "wrist_2_joint",
        "wrist_1_link",
        "wrist_2_link",
        (0.0, -0.1157, 0.0),
        (0.70710677, 0.70710677, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    ),
    (
        "wrist_3_joint",
        "wrist_2_link",
        "wrist_3_link",
        (0.0, 0.0922, 0.0),
        (0.70710677, -0.70710677, -6.181724e-8, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    ),
)
UR10_FOLDED_FIXED_LINKS = (
    (
        "ee_link",
        "wrist_3_link",
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (6.123234e-17, 0.70710677, 4.3297803e-17, 0.70710677),
    ),
    (
        "ee_suction_link",
        "ee_link",
        (0.161709, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    ),
)
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
IMAGE_MATCHED_SLIDING_STATION_USD = Path(
    os.environ.get("HARIM_SLIDING_STATION_USD", "/home/mh/Downloads/harim_v3.usd")
)
IMAGE_MATCHED_SLIDING_STATION_PRIM = os.environ.get(
    "HARIM_SLIDING_STATION_PRIM",
    "/World/HarimDemo/PickupSlideStation",
)
PICKUP_STATION_LEFT_Y = -0.19
PICKUP_STATION_RIGHT_Y = 0.24
V3_DROPOFF_CART_MODE = os.environ.get("HARIM_V3_DROPOFF_CART", "auto").lower()
V3_PUSHCART_SOURCE_NAME = "SM_PushcartA_02_22"
V3_PUSHCART_TARGET_ROOT = "/World/V3PushcartA_02_22"
V3_PUSHCART_PALLET_ROOT = "/World/V3PushcartPallet"
V3_PUSHCART_FALLBACK_RIGHT_OFFSET = np.array([0.0, -1.35, 0.0], dtype=float)
V3_PUSHCART_WORLD_Y = -1.7
V3_PUSHCART_YAW_DEG = 90.0
V3_PUSHCART_PALLET_WORLD_X = 12.56
V3_PUSHCART_PALLET_TRANSLATE_Z = -0.831
V3_PUSHCART_PALLET_Z_OFFSET = -0.0079
V3_AMR_DROP_STATION_BACKOFF = 0.08
V3_DROP_ROBOT_HOME_JOINTS_DEG = DROP_UR10_FOLDED_HOME_JOINT_TARGETS_DEG
V3_DROP_ROBOT_PICK_JOINTS_DEG = {
    "shoulder_pan_joint": -55.0,
    "shoulder_lift_joint": -92.0,
    "elbow_joint": -118.0,
    "wrist_1_joint": -72.0,
    "wrist_2_joint": 90.0,
    "wrist_3_joint": 0.0,
}
V3_DROP_ROBOT_PLACE_JOINTS_DEG = {
    "shoulder_pan_joint": -135.0,
    "shoulder_lift_joint": -88.0,
    "elbow_joint": -112.0,
    "wrist_1_joint": -76.0,
    "wrist_2_joint": 90.0,
    "wrist_3_joint": 0.0,
}
V3_DROP_ROBOT_ITEM_SECONDS = 6.0
V3_DROP_ROBOT_MAX_DT = 1.0 / 30.0
V3_DROP_ROBOT_PICK_APPROACH_DISTANCE = 0.10
V3_DROP_ROBOT_PICK_LIFT_DISTANCE = 0.30
V3_DROP_ROBOT_PLACE_APPROACH_DISTANCE = 0.35
V3_DROP_ROBOT_PLACE_LIFT_DISTANCE = 0.10
V3_DROP_ROBOT_PLACE_RELEASE_CLEARANCE = 0.006
V3_DROP_ROBOT_PICK_SURFACE_CLEARANCE = 0.0025
V3_DROP_ROBOT_PICK_APPROACH_PHASE_END = 0.20
V3_DROP_ROBOT_PICK_REACH_PHASE_END = 0.32
V3_DROP_ROBOT_PICK_WAIT_PHASE_END = 0.40
V3_DROP_ROBOT_PICK_LIFT_PHASE_END = 0.50
V3_DROP_ROBOT_PLACE_APPROACH_PHASE_END = 0.68
V3_DROP_ROBOT_PLACE_REACH_PHASE_END = 0.78
V3_DROP_ROBOT_PLACE_WAIT_PHASE_END = 0.84
V3_DROP_ROBOT_PLACE_LIFT_PHASE_END = 0.92
V3_DROP_ROBOT_GRIPPER_CLEARANCE = 0.035
V3_DROP_ROBOT_IK_TOLERANCE = 0.018
V3_DROP_ROBOT_IK_ROTATION_TOLERANCE = 0.18
V3_DROP_ROBOT_IK_MAX_ITERATIONS = 45
V3_DROP_ROBOT_IK_ROTATION_WEIGHT = 0.22
V3_DROP_ROBOT_IK_END_EFFECTOR_LINK = "ee_suction_link"
V3_DROP_ROBOT_GRASP_COLLISION_RELATIVE_PATH = "Collision/Cube_03"
V3_DROP_ROBOT_ORIGINAL_UPSIDE_DOWN_MARGIN = -0.0025
V3_DROP_ROBOT_MOTION_COMMANDER_TARGET = "/World/v3_drop_motion_commander_target"
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
KLT_MAGENTA_BOX_MESH_NAME = "FOF_Mesh_Magenta_Box"


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


def deactivate_stage_child_prims_containing(stage, root_path, patterns):
    from pxr import Usd

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return
    lower_patterns = tuple(pattern.lower() for pattern in patterns)
    for prim in Usd.PrimRange(root):
        if prim == root:
            continue
        name = prim.GetName().lower()
        if any(pattern in name for pattern in lower_patterns):
            prim.SetActive(False)


def matrix_from_quat_wxyz(quat):
    quat = np.array(quat, dtype=float)
    quat = quat / max(np.linalg.norm(quat), 1.0e-9)
    w, x, y, z = quat
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quat_from_matrix_wxyz(matrix):
    matrix = np.array(matrix, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return np.array(
            [
                0.25 * s,
                (matrix[2, 1] - matrix[1, 2]) / s,
                (matrix[0, 2] - matrix[2, 0]) / s,
                (matrix[1, 0] - matrix[0, 1]) / s,
            ],
            dtype=float,
        )

    if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        quat = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / s,
                0.25 * s,
                (matrix[0, 1] + matrix[1, 0]) / s,
                (matrix[0, 2] + matrix[2, 0]) / s,
            ],
            dtype=float,
        )
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        quat = np.array(
            [
                (matrix[0, 2] - matrix[2, 0]) / s,
                (matrix[0, 1] + matrix[1, 0]) / s,
                0.25 * s,
                (matrix[1, 2] + matrix[2, 1]) / s,
            ],
            dtype=float,
        )
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        quat = np.array(
            [
                (matrix[1, 0] - matrix[0, 1]) / s,
                (matrix[0, 2] + matrix[2, 0]) / s,
                (matrix[1, 2] + matrix[2, 1]) / s,
                0.25 * s,
            ],
            dtype=float,
        )
    return quat / max(np.linalg.norm(quat), 1.0e-9)


def pose_matrix(position, orientation):
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = matrix_from_quat_wxyz(orientation)
    matrix[:3, 3] = np.array(position, dtype=float)
    return matrix


def z_axis_rotation_matrix(degrees):
    radians = math.radians(float(degrees))
    cos_value = math.cos(radians)
    sin_value = math.sin(radians)
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.array(
        [
            [cos_value, -sin_value, 0.0],
            [sin_value, cos_value, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return matrix


def set_prim_local_matrix(stage, prim_path, matrix):
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        prim = stage.OverridePrim(prim_path)

    gf_values = [float(value) for value in np.array(matrix, dtype=float).T.reshape(-1)]
    gf_matrix = Gf.Matrix4d(*gf_values)

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(gf_matrix)


def compute_ur10_link_matrices(joint_targets_deg):
    link_matrices = {"base_link": np.eye(4, dtype=float)}

    for joint_name, body0, body1, local_pos0, local_rot0, local_pos1, local_rot1 in UR10_FOLDED_LINK_CHAIN:
        link_matrices[body1] = (
            link_matrices[body0]
            @ pose_matrix(local_pos0, local_rot0)
            @ z_axis_rotation_matrix(joint_targets_deg[joint_name])
            @ np.linalg.inv(pose_matrix(local_pos1, local_rot1))
        )

    for body1, body0, local_pos0, local_rot0, local_pos1, local_rot1 in UR10_FOLDED_FIXED_LINKS:
        link_matrices[body1] = (
            link_matrices[body0]
            @ pose_matrix(local_pos0, local_rot0)
            @ np.linalg.inv(pose_matrix(local_pos1, local_rot1))
        )

    return link_matrices


def compute_ur10_folded_home_link_matrices():
    return compute_ur10_link_matrices(DROP_UR10_FOLDED_HOME_JOINT_TARGETS_DEG)


def set_ur10_visual_pose_from_joint_targets(stage, ur10_root_path, joint_targets_deg, *, log_label=None):
    link_matrices = compute_ur10_link_matrices(joint_targets_deg)
    for link_name, matrix in link_matrices.items():
        set_prim_local_matrix(stage, f"{ur10_root_path}/{link_name}", matrix)
    wrist_position = link_matrices["wrist_3_link"][:3, 3]
    if log_label is not None:
        print(
            f"[HarimDemo] set {log_label} visual pose for static drop UR10: "
            f"root={ur10_root_path}, wrist_3_local={wrist_position.tolist()}"
        )


def set_ur10_folded_home_visual_pose(stage, ur10_root_path):
    set_ur10_visual_pose_from_joint_targets(
        stage,
        ur10_root_path,
        DROP_UR10_FOLDED_HOME_JOINT_TARGETS_DEG,
        log_label="folded",
    )


def make_visual_only_static(stage, root_path):
    from pxr import Gf, Sdf, Usd, UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return

    for prim in Usd.PrimRange(root):
        name = prim.GetName().lower()
        type_name = prim.GetTypeName().lower()
        if (
            "collision" in name
            or "surfacegripper" in name
            or "surface_gripper" in name
            or "suction_joint" in name
        ):
            prim.SetActive(False)
            continue

        if prim.IsA(UsdPhysics.Joint) or "joint" in name or "joint" in type_name:
            UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
            prim.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(False)
            continue

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body = UsdPhysics.RigidBodyAPI(prim)
            rigid_body.CreateRigidBodyEnabledAttr(False)
            rigid_body.CreateKinematicEnabledAttr(False)
            rigid_body.CreateVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))
            rigid_body.CreateAngularVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0))


def set_ur10_folded_home_joint_targets(stage, ur10_root_path):
    from pxr import Sdf, UsdPhysics

    try:
        from pxr import PhysxSchema
    except ImportError:
        PhysxSchema = None

    for joint_name, target_deg in DROP_UR10_FOLDED_HOME_JOINT_TARGETS_DEG.items():
        joint_path = f"{ur10_root_path}/joints/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)
        if not joint_prim.IsValid():
            joint_prim = stage.OverridePrim(joint_path)
        joint_prim.SetActive(True)
        UsdPhysics.Joint(joint_prim).CreateJointEnabledAttr(True)
        joint_prim.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(True)
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
        drive.CreateTargetPositionAttr(float(target_deg))
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateStiffnessAttr(1.0e8)
        drive.CreateDampingAttr(5.0e7)
        drive.CreateMaxForceAttr(1.0e8)
        if PhysxSchema is not None:
            joint_state = PhysxSchema.JointStateAPI.Apply(joint_prim, "angular")
            joint_state.CreatePositionAttr(float(target_deg))
            joint_state.CreateVelocityAttr(0.0)
        else:
            joint_prim.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float).Set(float(target_deg))
            joint_prim.CreateAttribute("state:angular:physics:velocity", Sdf.ValueTypeNames.Float).Set(0.0)


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
    path = str(prim.GetPath()).lower()
    if "/pallet" in path:
        return False

    name = prim.GetName()
    if (
        is_small_klt_visual_box_prim_name(name)
        or is_background_card_box_prim_name(name)
        or name == KLT_MAGENTA_BOX_MESH_NAME
    ):
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


def capture_gprim_visual_styles(stage, root_path):
    from pxr import Usd, UsdGeom, UsdShade

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return {}

    styles = {}
    root_prefix = str(root.GetPath())
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Gprim):
            continue
        prim_path = str(prim.GetPath())
        relative_path = prim_path[len(root_prefix) :]
        material, _relationship = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        material_path = str(material.GetPrim().GetPath()) if material is not None and material.GetPrim().IsValid() else None
        display_color_attr = UsdGeom.Gprim(prim).GetDisplayColorAttr()
        has_display_color = display_color_attr.HasAuthoredValueOpinion()
        display_color = display_color_attr.Get() if has_display_color else None
        styles[relative_path] = (material_path, has_display_color, display_color)
    return styles


def restore_gprim_visual_styles(stage, root_path, styles):
    from pxr import UsdGeom, UsdShade

    if not styles:
        return
    for relative_path, (material_path, has_display_color, display_color) in styles.items():
        prim = stage.GetPrimAtPath(f"{root_path}{relative_path}")
        if not prim.IsValid() or not prim.IsA(UsdGeom.Gprim):
            continue
        if material_path is not None:
            material_prim = stage.GetPrimAtPath(material_path)
            if material_prim.IsValid():
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(material_prim))
        display_color_attr = UsdGeom.Gprim(prim).GetDisplayColorAttr()
        if has_display_color:
            display_color_attr.Set(display_color)
        elif display_color_attr.HasAuthoredValueOpinion():
            display_color_attr.Clear()


def apply_box_style_to_all_gprims(stage, root_path, source_material, source_display_color):
    from pxr import Usd, UsdGeom, UsdShade

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0

    applied_count = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Gprim):
            continue
        if source_material is not None:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(source_material)
        if source_display_color is not None:
            UsdGeom.Gprim(prim).CreateDisplayColorAttr().Set(source_display_color)
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
    name_prefix,
):
    from pxr import UsdGeom

    if not IMAGE_MATCHED_SLIDING_STATION_USD.exists():
        raise RuntimeError(
            "Sliding station source USD was not found: "
            f"{IMAGE_MATCHED_SLIDING_STATION_USD}. Save the edited harim_v2.usd first."
        )

    station_prim = UsdGeom.Xform.Define(stage, station_root).GetPrim()
    station_prim.GetReferences().AddReference(str(IMAGE_MATCHED_SLIDING_STATION_USD), IMAGE_MATCHED_SLIDING_STATION_PRIM)

    print(
        f"[HarimDemo] placed {name_prefix} image-matched sliding station: "
        f"{IMAGE_MATCHED_SLIDING_STATION_USD}{IMAGE_MATCHED_SLIDING_STATION_PRIM}"
    )


def set_slide_station_side_y_offsets(stage, station_root, *, left_y, right_y):
    from pxr import Gf, Usd, UsdGeom

    root = stage.GetPrimAtPath(station_root)
    if not root.IsValid():
        return

    updated = 0
    for prim in Usd.PrimRange(root):
        name = prim.GetName()
        if name.startswith("Left"):
            target_y = left_y
        elif name.startswith("Right"):
            target_y = right_y
        else:
            continue

        xformable = UsdGeom.Xformable(prim)
        translate_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)

        current = translate_op.Get() or Gf.Vec3d(0.0, 0.0, 0.0)
        value_type = Gf.Vec3f if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Vec3d
        translate_op.Set(value_type(float(current[0]), float(target_y), float(current[2])))
        updated += 1

    print(
        "[HarimDemo] set slide station side Y offsets: "
        f"root={station_root}, left_y={left_y:.3f}, right_y={right_y:.3f}, updated={updated}"
    )


def compute_world_z_bounds(stage, prim_or_path):
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_or_path) if isinstance(prim_or_path, str) else prim_or_path
    if not prim.IsValid():
        raise RuntimeError(f"Prim is not valid for Z bounds: {prim_or_path}")

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    min_z = float(box.GetMin()[2])
    max_z = float(box.GetMax()[2])
    if not np.isfinite(min_z) or not np.isfinite(max_z) or min_z > max_z or abs(min_z) > 1.0e20:
        raise RuntimeError(f"Could not compute finite Z bounds for prim: {prim.GetPath()}")
    return min_z, max_z


def compute_world_aligned_bounds(stage, prim_or_path):
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_or_path) if isinstance(prim_or_path, str) else prim_or_path
    if not prim.IsValid():
        raise RuntimeError(f"Prim is not valid for bounds: {prim_or_path}")

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    bounds_min = np.array(box.GetMin(), dtype=float)
    bounds_max = np.array(box.GetMax(), dtype=float)
    if (
        not np.all(np.isfinite(bounds_min))
        or not np.all(np.isfinite(bounds_max))
        or np.any(bounds_min > bounds_max)
        or np.max(np.abs(bounds_min)) > 1.0e20
    ):
        raise RuntimeError(f"Could not compute finite bounds for prim: {prim.GetPath()}")
    return bounds_min, bounds_max


def set_prim_scale(stage, prim_path, scale):
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim is not valid for scale: {prim_path}")

    xformable = UsdGeom.Xformable(prim)
    scale_op = None
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_op = op
            break
    if scale_op is None:
        scale_op = xformable.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble)
    scale_value_type = Gf.Vec3f if scale_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Vec3d
    scale_op.Set(scale_value_type(float(scale[0]), float(scale[1]), float(scale[2])))


def set_prim_translate(stage, prim_path, position):
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim is not valid for translate: {prim_path}")

    xformable = UsdGeom.Xformable(prim)
    translate_op = None
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)

    translate_value_type = Gf.Vec3f if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Vec3d
    translate_op.Set(translate_value_type(float(position[0]), float(position[1]), float(position[2])))


def set_prim_translate_rotate_z_scale(stage, prim_path, position, yaw_degrees, scale):
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim is not valid for translate/rotateZ/scale: {prim_path}")

    xformable = UsdGeom.Xformable(prim)
    translate_op = None
    rotate_z_op = None
    scale_op = None
    for op in xformable.GetOrderedXformOps():
        op_type = op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeTranslate and translate_op is None:
            translate_op = op
        elif op_type == UsdGeom.XformOp.TypeRotateZ and rotate_z_op is None:
            rotate_z_op = op
        elif op_type == UsdGeom.XformOp.TypeScale and scale_op is None:
            scale_op = op

    xformable.ClearXformOpOrder()
    if translate_op is None:
        translate_op = xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    if rotate_z_op is None:
        rotate_z_op = xformable.AddRotateZOp(precision=UsdGeom.XformOp.PrecisionDouble)
    if scale_op is None:
        scale_op = xformable.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble)
    xformable.SetXformOpOrder([translate_op, rotate_z_op, scale_op])

    translate_value_type = Gf.Vec3f if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Vec3d
    scale_value_type = Gf.Vec3f if scale_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Vec3d
    translate_op.Set(translate_value_type(float(position[0]), float(position[1]), float(position[2])))
    rotate_z_op.Set(float(yaw_degrees))
    scale_op.Set(scale_value_type(float(scale[0]), float(scale[1]), float(scale[2])))


def set_prim_pose(stage, prim_path, position, orientation):
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim is not valid for pose: {prim_path}")

    xformable = UsdGeom.Xformable(prim)
    translate_op = None
    orient_op = None
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and translate_op is None:
            translate_op = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient and orient_op is None:
            orient_op = op

    if translate_op is None:
        translate_op = xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    if orient_op is None:
        orient_op = xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat)

    translate_value_type = Gf.Vec3f if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Vec3d
    orient_value_type = Gf.Quatd if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble else Gf.Quatf
    imag_value_type = Gf.Vec3d if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble else Gf.Vec3f

    translate_op.Set(translate_value_type(float(position[0]), float(position[1]), float(position[2])))
    orient_op.Set(
        orient_value_type(
            float(orientation[0]),
            imag_value_type(float(orientation[1]), float(orientation[2]), float(orientation[3])),
        )
    )


def get_prim_xform_components(stage, prim_path):
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim is not valid for transform read: {prim_path}")

    position = np.zeros(3, dtype=float)
    orientation = yaw_to_quat(0.0)
    scale = np.ones(3, dtype=float)
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        value = op.Get()
        if value is None:
            continue
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            position = np.array(value, dtype=float)
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            imag = value.GetImaginary()
            orientation = np.array([value.GetReal(), imag[0], imag[1], imag[2]], dtype=float)
        elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale = np.array(value, dtype=float)
    return position, orientation, scale


def get_prim_world_xform_components(stage, prim_path):
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim is not valid for world transform read: {prim_path}")

    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    position = np.array(matrix.ExtractTranslation(), dtype=float)
    rotation = matrix.ExtractRotationQuat()
    imag = rotation.GetImaginary()
    orientation = np.array([rotation.GetReal(), imag[0], imag[1], imag[2]], dtype=float)
    basis = np.array(
        [[float(matrix[row][col]) for col in range(3)] for row in range(3)],
        dtype=float,
    )
    scale = np.array([np.linalg.norm(basis[:, idx]) for idx in range(3)], dtype=float)
    scale[scale < 1.0e-9] = 1.0
    return position, orientation, scale


def transform_world_point_to_prim_local(stage, prim_path, world_position):
    from pxr import Gf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim is not valid for world-to-local transform: {prim_path}")

    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    local = matrix.GetInverse().Transform(Gf.Vec3d(*[float(value) for value in world_position]))
    return np.array(local, dtype=float)


def transform_world_rotation_to_prim_local(stage, prim_path, world_rotation):
    position, orientation, _scale = get_prim_world_xform_components(stage, prim_path)
    _ = position
    root_rotation = matrix_from_quat_wxyz(orientation)
    return root_rotation.T @ np.array(world_rotation, dtype=float)


def mirrored_root_transform(source_position, source_orientation, source_scale, mirror_plane_x):
    source_position = np.array(source_position, dtype=float)
    source_scale = np.array(source_scale, dtype=float)
    root_position = np.array(source_position, dtype=float)
    root_position[0] = 2.0 * mirror_plane_x - source_position[0]
    root_scale = np.array(source_scale, dtype=float)
    root_scale[0] *= -1.0
    return root_position, np.array(source_orientation, dtype=float), root_scale


def rotate_yaw_180(vector):
    vector = np.array(vector, dtype=float)
    return np.array([-vector[0], -vector[1], vector[2]], dtype=float)


def reversed_static_root_transform(
    source_position,
    source_orientation,
    source_scale,
    source_anchor,
    target_anchor,
):
    source_position = np.array(source_position, dtype=float)
    source_anchor = np.array(source_anchor, dtype=float)
    target_anchor = np.array(target_anchor, dtype=float)
    root_position = target_anchor - rotate_yaw_180(source_anchor - source_position)
    root_orientation = quat_multiply(yaw_to_quat(math.pi), np.array(source_orientation, dtype=float))
    root_scale = np.abs(np.array(source_scale, dtype=float))
    return root_position, root_orientation, root_scale


def reversed_amr_drop_pose_from_pickup(pickup_pose, source_pallet_position, target_pallet_position, amr_z):
    pickup_pose = np.array(pickup_pose, dtype=float)
    source_pallet_position = np.array(source_pallet_position, dtype=float)
    target_pallet_position = np.array(target_pallet_position, dtype=float)
    pickup_offset_from_pallet = pickup_pose - source_pallet_position
    drop_pose = target_pallet_position + rotate_yaw_180(pickup_offset_from_pallet)
    drop_pose[2] = float(amr_z)
    return drop_pose


def backoff_amr_drop_pose_toward_reversed_pickup(nominal_drop_pose, reversed_drop_pose, backoff_distance):
    nominal_drop_pose = np.array(nominal_drop_pose, dtype=float)
    reversed_drop_pose = np.array(reversed_drop_pose, dtype=float)
    delta = reversed_drop_pose - nominal_drop_pose
    distance = float(np.linalg.norm(delta[:2]))
    if distance <= 1.0e-9:
        return nominal_drop_pose
    step = min(float(backoff_distance), distance)
    adjusted = nominal_drop_pose + delta * (step / distance)
    adjusted[2] = nominal_drop_pose[2]
    return adjusted


def apply_mirror_root_transform(stage, prim_path, position, orientation, scale):
    set_prim_pose(stage, prim_path, position, orientation)
    set_prim_scale(stage, prim_path, scale)


def place_reversed_drop_area(
    *,
    stage,
    ur10_table_usd,
    add_reference_to_stage,
    wait_for_stage_loading,
    simulation_app,
    usd_context,
    harim_root,
    source_pallet_position,
    target_pallet_position,
):
    mirror_plane_x = (float(source_pallet_position[0]) + float(target_pallet_position[0])) * 0.5
    source_table_position, source_table_orientation, source_table_scale = get_prim_xform_components(
        stage,
        "/World/Ur10Table",
    )
    source_station_position, source_station_orientation, source_station_scale = get_prim_xform_components(
        stage,
        f"{harim_root}/PickupSlideStation",
    )
    drop_table_position, drop_table_orientation, drop_table_scale = reversed_static_root_transform(
        source_table_position,
        source_table_orientation,
        source_table_scale,
        source_pallet_position,
        target_pallet_position,
    )
    drop_station_position, drop_station_orientation, drop_station_scale = mirrored_root_transform(
        source_station_position,
        source_station_orientation,
        source_station_scale,
        mirror_plane_x,
    )
    drop_table_root = f"{harim_root}/DropUr10Table"
    drop_station_root = f"{harim_root}/DropSlideStation"

    add_reference_to_stage(ur10_table_usd, drop_table_root)
    wait_for_stage_loading(simulation_app, usd_context, "drop Ur10Table")
    apply_mirror_root_transform(stage, drop_table_root, drop_table_position, drop_table_orientation, drop_table_scale)
    deactivate_stage_child_prims_containing(
        stage,
        drop_table_root,
        ("flip", "pallet_holder", "dolly", "conveyor", "bin", "pallet", "obstacle"),
    )
    simulation_app.update()
    drop_table_position = align_prim_bottom_to_target_z(stage, drop_table_root, REFERENCE_STATION_FLOOR_Z)
    simulation_app.update()
    drop_table_min, drop_table_max = compute_world_aligned_bounds(stage, drop_table_root)
    drop_ur10_prim = stage.GetPrimAtPath(f"{drop_table_root}/ur10")
    if drop_ur10_prim.IsValid():
        drop_ur10_prim.SetActive(True)
        set_ur10_folded_home_joint_targets(stage, str(drop_ur10_prim.GetPath()))
        set_ur10_folded_home_visual_pose(stage, str(drop_ur10_prim.GetPath()))
        make_visual_only_static(stage, str(drop_ur10_prim.GetPath()))

    place_image_matched_sliding_station(
        stage=stage,
        station_root=drop_station_root,
        name_prefix="DropSlideStation",
    )
    wait_for_stage_loading(simulation_app, usd_context, "drop image-matched sliding station")
    print("[HarimDemo] kept DropSlideStation source side spacing to preserve image-matched mesh shape")
    apply_mirror_root_transform(stage, drop_station_root, drop_station_position, drop_station_orientation, drop_station_scale)
    simulation_app.update()
    drop_station_top_z = compute_world_z_bounds(stage, drop_station_root)[1]
    station_z_shift = float(target_pallet_position[2] - drop_station_top_z)
    if abs(station_z_shift) > 1e-6:
        drop_station_position = np.array(drop_station_position, dtype=float)
        drop_station_position[2] += station_z_shift
        apply_mirror_root_transform(
            stage,
            drop_station_root,
            drop_station_position,
            drop_station_orientation,
            drop_station_scale,
        )
        simulation_app.update()
        drop_station_top_z = compute_world_z_bounds(stage, drop_station_root)[1]

    print(
        "[HarimDemo] placed reversed drop area: "
        f"source_pallet={source_pallet_position.tolist()}, "
        f"target_pallet={target_pallet_position.tolist()}, "
        f"mirror_plane_x={mirror_plane_x:.6f}, "
        f"table_root_translate={drop_table_position.tolist()}, "
        f"table_root_orientation={drop_table_orientation.tolist()}, "
        f"table_root_scale={drop_table_scale.tolist()}, "
        f"table_bottom_z={drop_table_min[2]:.6f}, "
        f"table_top_z={drop_table_max[2]:.6f}, "
        f"station_root_translate={drop_station_position.tolist()}, "
        f"station_root_scale={drop_station_scale.tolist()}, "
        f"station_top_z={drop_station_top_z:.6f}, "
        f"drop_ur10_active={drop_ur10_prim.IsValid() and drop_ur10_prim.IsActive()}, "
        "mirror_axis=X"
    )
    return {
        "drop_table_root": drop_table_root,
        "drop_station_root": drop_station_root,
        "drop_table_position": np.array(drop_table_position, dtype=float),
        "drop_table_orientation": np.array(drop_table_orientation, dtype=float),
        "drop_station_position": np.array(drop_station_position, dtype=float),
        "drop_station_top_z": float(drop_station_top_z),
        "mirror_plane_x": float(mirror_plane_x),
    }


def is_v3_dropoff_cart_enabled():
    if V3_DROPOFF_CART_MODE in ("1", "true", "yes", "on"):
        return True
    if V3_DROPOFF_CART_MODE in ("0", "false", "no", "off"):
        return False
    return IMAGE_MATCHED_SLIDING_STATION_USD.name == "harim_v3.usd"


def find_first_prim_path_by_name(stage, name, root_path="/World"):
    from pxr import Usd

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def delete_prim_if_present(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        stage.RemovePrim(prim_path)


def copy_prim_for_v3(stage, source_path, target_path):
    import omni.kit.commands
    import omni.usd

    if not stage.GetPrimAtPath(source_path).IsValid():
        raise RuntimeError(f"V3 source prim was not found: {source_path}")

    delete_prim_if_present(stage, target_path)
    omni.usd.duplicate_prim(stage, source_path, target_path, duplicate_layers=False)
    copied = stage.GetPrimAtPath(target_path).IsValid()
    try:
        if not copied:
            omni.kit.commands.execute(
                "CopyPrimCommand",
                path_from=source_path,
                path_to=target_path,
                duplicate_layers=False,
                combine_layers=False,
                exclusive_select=False,
                flatten_references=False,
            )
            copied = stage.GetPrimAtPath(target_path).IsValid()
    except Exception as exc:
        print(f"[HarimDemo] CopyPrimCommand fallback for {source_path}: {exc}")

    if not copied:
        raise RuntimeError(f"Could not copy V3 prim: {source_path} -> {target_path}")
    return stage.GetPrimAtPath(target_path)


def align_prim_bottom_center_to_target(stage, prim_path, *, target_center_xy, target_bottom_z):
    position, _orientation, _scale = get_prim_xform_components(stage, prim_path)
    bounds_min, bounds_max = compute_world_aligned_bounds(stage, prim_path)
    current_center = (bounds_min + bounds_max) * 0.5
    adjusted = np.array(position, dtype=float)
    adjusted[0] += float(target_center_xy[0] - current_center[0])
    adjusted[1] += float(target_center_xy[1] - current_center[1])
    adjusted[2] += float(target_bottom_z - bounds_min[2])
    set_prim_translate(stage, prim_path, adjusted)
    return adjusted


def align_prim_bottom_to_target_z(stage, prim_path, target_bottom_z):
    position, _orientation, _scale = get_prim_xform_components(stage, prim_path)
    bounds_min, _bounds_max = compute_world_aligned_bounds(stage, prim_path)
    adjusted = np.array(position, dtype=float)
    adjusted[2] += float(target_bottom_z - bounds_min[2])
    set_prim_translate(stage, prim_path, adjusted)
    return adjusted


def place_v3_dropoff_pushcart_target(
    *,
    stage,
    simulation_app,
    harim_root,
    source_pallet_path,
    source_pallet_position,
    target_pallet_position,
    drop_area_info,
):
    if not is_v3_dropoff_cart_enabled():
        return None

    source_pushcart_path = find_first_prim_path_by_name(stage, V3_PUSHCART_SOURCE_NAME, "/World/Background")
    if source_pushcart_path is None:
        raise RuntimeError(f"Could not find source pushcart prim in background: {V3_PUSHCART_SOURCE_NAME}")

    copy_prim_for_v3(stage, source_pushcart_path, V3_PUSHCART_TARGET_ROOT)
    _source_position, _source_orientation, pushcart_scale = get_prim_world_xform_components(stage, source_pushcart_path)

    drop_table_position = np.array(drop_area_info["drop_table_position"], dtype=float)
    pushcart_position = drop_table_position + V3_PUSHCART_FALLBACK_RIGHT_OFFSET
    pushcart_position[1] = V3_PUSHCART_WORLD_Y

    set_prim_translate_rotate_z_scale(
        stage,
        V3_PUSHCART_TARGET_ROOT,
        pushcart_position,
        V3_PUSHCART_YAW_DEG,
        pushcart_scale,
    )
    simulation_app.update()
    pushcart_position = align_prim_bottom_to_target_z(stage, V3_PUSHCART_TARGET_ROOT, REFERENCE_STATION_FLOOR_Z)
    simulation_app.update()

    copy_prim_for_v3(stage, source_pallet_path, V3_PUSHCART_PALLET_ROOT)
    make_visual_only_static(stage, V3_PUSHCART_PALLET_ROOT)
    cart_min, cart_max = compute_world_aligned_bounds(stage, V3_PUSHCART_TARGET_ROOT)
    cart_center = (cart_min + cart_max) * 0.5
    pallet_center_xy = np.array(cart_center[:2], dtype=float)
    pallet_center_xy[0] = V3_PUSHCART_PALLET_WORLD_X
    pallet_position = align_prim_bottom_center_to_target(
        stage,
        V3_PUSHCART_PALLET_ROOT,
        target_center_xy=pallet_center_xy,
        target_bottom_z=cart_max[2] + V3_PUSHCART_PALLET_Z_OFFSET,
    )
    pallet_position[2] = V3_PUSHCART_PALLET_TRANSLATE_Z
    set_prim_translate(stage, V3_PUSHCART_PALLET_ROOT, pallet_position)
    simulation_app.update()
    pallet_min, pallet_max = compute_world_aligned_bounds(stage, V3_PUSHCART_PALLET_ROOT)

    print(
        "[HarimDemo] placed V3 dropoff pushcart target: "
        f"source={source_pushcart_path}, target={V3_PUSHCART_TARGET_ROOT}, "
        f"pushcart_translate={pushcart_position.tolist()}, "
        f"pushcart_yaw_deg={V3_PUSHCART_YAW_DEG:.1f}, "
        f"cart_bottom_z={cart_min[2]:.6f}, "
        f"cart_top_z={cart_max[2]:.6f}, "
        f"pallet={V3_PUSHCART_PALLET_ROOT}, pallet_translate={pallet_position.tolist()}, "
        f"pallet_world_x={V3_PUSHCART_PALLET_WORLD_X:.3f}, "
        f"pallet_translate_z={V3_PUSHCART_PALLET_TRANSLATE_Z:.4f}, "
        f"pallet_z_offset={V3_PUSHCART_PALLET_Z_OFFSET:.4f}, "
        f"pallet_bottom_z={pallet_min[2]:.6f}, "
        f"pallet_top_z={pallet_max[2]:.6f}"
    )
    return {
        "pushcart_root": V3_PUSHCART_TARGET_ROOT,
        "pallet_root": V3_PUSHCART_PALLET_ROOT,
        "pushcart_position": np.array(pushcart_position, dtype=float),
        "pallet_position": np.array(pallet_position, dtype=float),
    }


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

    y0 = -0.62
    z0 = -0.51
    dx = -0.21
    dy = 0.31
    dz = 0.135
    x0 = DEFAULT_PALLET_X - dx * (cols - 1) * 0.5

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
        stage=None,
        amr_lift_path=None,
        lift_surface_z=None,
        drop_pose_override=None,
        stabilize_source_stack=False,
    ):
        self.world = world
        self.context = context
        self.task = task
        self.amr = amr_prim
        self.amr_lift = amr_lift_prim
        self.stage = stage
        self.amr_lift_path = amr_lift_path
        self.lift_surface_z = None if lift_surface_z is None else float(lift_surface_z)
        self.lift_plate = lift_plate
        self.pallet_parts = pallet_parts
        self.stack_coordinates = stack_coordinates
        self.args = args
        self.stabilize_source_stack = bool(stabilize_source_stack)

        self.state = TransferState.WAIT_STACK_COMPLETE
        self.state_time = 0.0
        self.completed_cycles = 0
        self.carrying = False
        self.attached_items = []
        self.item_offsets = {}
        self.pallet_base_offsets = {}
        self.dropped_item_poses = {}
        self.dropped_item_sequence = []
        self.dropped_pallet_poses = {}
        self.initial_pallet_poses = self._capture_pallet_poses()
        self.stabilized_stack_count = 0

        self.stack_center = self._compute_stack_center()
        self.amr_yaw = 0.0
        self.lift_offset = 0.0
        self.amr_lift_base_offset = None
        self.amr_lift_orientation = None
        self.amr_lift_top_offset = None
        self.move_target = None

        self.start_pose = np.array([args.pickup_x + AMR_START_STANDOFF, args.pickup_y, args.amr_z], dtype=float)
        self.approach_pose = np.array([args.pickup_x + AMR_APPROACH_STANDOFF, args.pickup_y, args.amr_z], dtype=float)
        self.pickup_pose = np.array([args.pickup_x, args.pickup_y, args.amr_z], dtype=float)
        if drop_pose_override is None:
            self.drop_pose = np.array([args.drop_x, args.drop_y, args.amr_z], dtype=float)
        else:
            self.drop_pose = np.array(drop_pose_override, dtype=float)
        self.exit_pose = np.array(
            [self.drop_pose[0] - SLIDE_EXIT_DISTANCE, self.drop_pose[1], args.amr_z],
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
        self.dropped_item_sequence = []
        self.dropped_pallet_poses = {}
        self.stabilized_stack_count = 0
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
        if self.lift_surface_z is not None and self.stage is not None and self.amr_lift_path:
            if self.amr_lift_top_offset is None:
                self.amr_lift.set_world_pose(position=target, orientation=self.amr_lift_orientation)
                _lift_min, lift_max = compute_world_aligned_bounds(self.stage, self.amr_lift_path)
                self.amr_lift_top_offset = float(lift_max[2] - target[2])
                print(
                    "[HarimDemo] calibrated iw_hub lift top to pallet underside: "
                    f"surface_z={self.lift_surface_z:.6f}, top_offset={self.amr_lift_top_offset:.6f}"
                )
            target[2] = self.lift_surface_z - self.amr_lift_top_offset + self.lift_offset
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

    def _stabilize_newly_stacked_items(self):
        if not self.stabilize_source_stack:
            return

        stacked_bins = list(getattr(self.context, "stacked_bins", []))
        if self.stabilized_stack_count > len(stacked_bins):
            self.stabilized_stack_count = 0

        target_count = min(len(stacked_bins), len(self.stack_coordinates))
        for index in range(self.stabilized_stack_count, target_count):
            bin_obj = getattr(stacked_bins[index], "bin_obj", None)
            if bin_obj is None:
                continue
            try:
                bin_obj.set_world_pose(
                    position=np.array(self.stack_coordinates[index], dtype=float),
                    orientation=np.array(UPSIDE_DOWN_BIN_QUAT, dtype=float),
                )
                self._stop_dynamic_item(bin_obj)
            except Exception as exc:
                print(f"[HarimDemo] could not stabilize source stacked item {index + 1}: {exc}")
        self.stabilized_stack_count = target_count

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
        self.dropped_item_sequence = []
        for item in self.attached_items:
            try:
                pos, orient = item.get_world_pose()
                self.dropped_item_poses[item.name] = (item, np.array(pos, dtype=float), orient)
                self.dropped_item_sequence.append((item, np.array(pos, dtype=float), orient))
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
        self._stabilize_newly_stacked_items()

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


def interpolate_joint_targets(start_targets, end_targets, t):
    t = clamp(t, 0.0, 1.0)
    return {
        joint_name: lerp(float(start_targets[joint_name]), float(end_targets[joint_name]), t)
        for joint_name in start_targets
    }


def catmull_rom_scalar(p0, p1, p2, p3, t):
    t = clamp(t, 0.0, 1.0)
    t2 = t * t
    t3 = t2 * t
    return 0.5 * (
        (2.0 * p1)
        + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
    )


def interpolate_joint_targets_catmull_rom(keyframes, t):
    if not keyframes:
        return dict(V3_DROP_ROBOT_HOME_JOINTS_DEG)
    if t <= keyframes[0][0]:
        return dict(keyframes[0][1])
    if t >= keyframes[-1][0]:
        return dict(keyframes[-1][1])

    segment_index = 0
    for index in range(len(keyframes) - 1):
        if keyframes[index][0] <= t <= keyframes[index + 1][0]:
            segment_index = index
            break

    t1, p1 = keyframes[segment_index]
    t2, p2 = keyframes[segment_index + 1]
    p0 = keyframes[max(0, segment_index - 1)][1]
    p3 = keyframes[min(len(keyframes) - 1, segment_index + 2)][1]
    local_t = 0.0 if abs(t2 - t1) < 1.0e-9 else (t - t1) / (t2 - t1)
    targets = {}
    for joint_name in UR10_IK_JOINT_NAMES:
        start_value = float(p1[joint_name])
        end_value = float(p2[joint_name])
        value = catmull_rom_scalar(
            float(p0[joint_name]),
            start_value,
            end_value,
            float(p3[joint_name]),
            local_t,
        )
        targets[joint_name] = clamp(value, min(start_value, end_value), max(start_value, end_value))
    return targets


UR10_IK_JOINT_NAMES = tuple(DROP_UR10_FOLDED_HOME_JOINT_TARGETS_DEG.keys())
UR10_IK_SEEDS = (
    V3_DROP_ROBOT_PICK_JOINTS_DEG,
    V3_DROP_ROBOT_PLACE_JOINTS_DEG,
    V3_DROP_ROBOT_HOME_JOINTS_DEG,
)
UR10_IK_LIMITS_DEG = {
    "shoulder_pan_joint": (-360.0, 360.0),
    "shoulder_lift_joint": (-270.0, 180.0),
    "elbow_joint": (-270.0, 270.0),
    "wrist_1_joint": (-360.0, 360.0),
    "wrist_2_joint": (-180.0, 180.0),
    "wrist_3_joint": (-360.0, 360.0),
}


def ur10_joint_targets_to_array(joint_targets):
    return np.array([float(joint_targets[joint_name]) for joint_name in UR10_IK_JOINT_NAMES], dtype=float)


def ur10_joint_array_to_targets(joint_values):
    return {
        joint_name: float(joint_values[index])
        for index, joint_name in enumerate(UR10_IK_JOINT_NAMES)
    }


def clamp_ur10_joint_array(joint_values):
    result = np.array(joint_values, dtype=float)
    for index, joint_name in enumerate(UR10_IK_JOINT_NAMES):
        lower, upper = UR10_IK_LIMITS_DEG[joint_name]
        result[index] = clamp(result[index], lower, upper)
    return result


def compute_ur10_ee_local_position_from_array(joint_values):
    joint_targets = ur10_joint_array_to_targets(joint_values)
    return np.array(
        compute_ur10_link_matrices(joint_targets)[V3_DROP_ROBOT_IK_END_EFFECTOR_LINK][:3, 3],
        dtype=float,
    )


def compute_ur10_ee_local_pose_from_array(joint_values):
    joint_targets = ur10_joint_array_to_targets(joint_values)
    matrix = compute_ur10_link_matrices(joint_targets)[V3_DROP_ROBOT_IK_END_EFFECTOR_LINK]
    return np.array(matrix[:3, 3], dtype=float), np.array(matrix[:3, :3], dtype=float)


def rotation_error_vector(target_rotation, current_rotation):
    error_rotation = np.array(target_rotation, dtype=float) @ np.array(current_rotation, dtype=float).T
    return 0.5 * np.array(
        [
            error_rotation[2, 1] - error_rotation[1, 2],
            error_rotation[0, 2] - error_rotation[2, 0],
            error_rotation[1, 0] - error_rotation[0, 1],
        ],
        dtype=float,
    )


def orthonormal_rotation_from_axes(ax, ay):
    target_ax = np.array(ax, dtype=float)
    target_ax = target_ax / max(np.linalg.norm(target_ax), 1.0e-9)
    target_ay = np.array(ay, dtype=float)
    target_ay = target_ay - target_ax * float(np.dot(target_ax, target_ay))
    target_ay = target_ay / max(np.linalg.norm(target_ay), 1.0e-9)
    target_az = np.cross(target_ax, target_ay)
    target_az = target_az / max(np.linalg.norm(target_az), 1.0e-9)
    return np.column_stack((target_ax, target_ay, target_az))


def original_place_target_rotation():
    target_ax = np.array([0.0, 0.0, -1.0], dtype=float)
    target_az = np.array([0.0, -1.0, 0.0], dtype=float)
    target_ay = np.cross(target_az, target_ax)
    return orthonormal_rotation_from_axes(target_ax, target_ay)


def adjust_rotation_about_x_if_opposite(eff_rotation, target_rotation, threshold=-0.9):
    target_x = np.array(target_rotation[:3, 0], dtype=float)
    eff_x = np.array(eff_rotation[:3, 0], dtype=float)
    if float(np.dot(target_x, eff_x)) < threshold:
        return np.array(target_rotation, dtype=float) @ np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
            ],
            dtype=float,
        )
    return np.array(target_rotation, dtype=float)


def build_ur10_ik_seed_targets(target_local_position, seed_targets):
    target = np.array(target_local_position, dtype=float)
    seeds = [dict(seed_targets), *[dict(seed) for seed in UR10_IK_SEEDS]]
    target_pan = math.degrees(math.atan2(float(target[1]), float(target[0])))
    target_seed = dict(V3_DROP_ROBOT_PICK_JOINTS_DEG)
    target_seed["shoulder_pan_joint"] = target_pan
    seeds.append(target_seed)

    unique_seeds = []
    seen = set()
    for seed in seeds:
        key = tuple(round(float(seed[joint_name]), 3) for joint_name in UR10_IK_JOINT_NAMES)
        if key in seen:
            continue
        seen.add(key)
        unique_seeds.append(seed)
    return unique_seeds


def solve_ur10_pose_ik(target_local_position, target_local_rotation, seed_targets):
    target = np.array(target_local_position, dtype=float)
    target_rotation = np.array(target_local_rotation, dtype=float)
    best_values = None
    best_error_norm = float("inf")
    best_position_error_norm = float("inf")
    best_rotation_error_norm = float("inf")

    for seed in build_ur10_ik_seed_targets(target, seed_targets):
        values = clamp_ur10_joint_array(ur10_joint_targets_to_array(seed))
        for _iteration in range(V3_DROP_ROBOT_IK_MAX_ITERATIONS):
            current, current_rotation = compute_ur10_ee_local_pose_from_array(values)
            position_error = target - current
            rotation_error = rotation_error_vector(target_rotation, current_rotation)
            error = np.concatenate(
                [
                    position_error,
                    V3_DROP_ROBOT_IK_ROTATION_WEIGHT * rotation_error,
                ]
            )
            error_norm = float(np.linalg.norm(error))
            position_error_norm = float(np.linalg.norm(position_error))
            rotation_error_norm = float(np.linalg.norm(rotation_error))
            if error_norm < best_error_norm:
                best_error_norm = error_norm
                best_position_error_norm = position_error_norm
                best_rotation_error_norm = rotation_error_norm
                best_values = np.array(values, dtype=float)
            if (
                position_error_norm <= V3_DROP_ROBOT_IK_TOLERANCE
                and rotation_error_norm <= V3_DROP_ROBOT_IK_ROTATION_TOLERANCE
            ):
                break

            jacobian = np.zeros((6, len(UR10_IK_JOINT_NAMES)), dtype=float)
            epsilon = 0.25
            for joint_index in range(len(UR10_IK_JOINT_NAMES)):
                perturbed = np.array(values, dtype=float)
                perturbed[joint_index] += epsilon
                perturbed = clamp_ur10_joint_array(perturbed)
                perturbed_position, perturbed_rotation = compute_ur10_ee_local_pose_from_array(perturbed)
                jacobian[:3, joint_index] = (perturbed_position - current) / epsilon
                jacobian[3:, joint_index] = (
                    V3_DROP_ROBOT_IK_ROTATION_WEIGHT
                    * rotation_error_vector(perturbed_rotation, current_rotation)
                    / epsilon
                )

            damping = 0.025
            normal = jacobian @ jacobian.T + (damping * damping) * np.eye(jacobian.shape[0], dtype=float)
            try:
                delta = jacobian.T @ np.linalg.solve(normal, error)
            except np.linalg.LinAlgError:
                delta = jacobian.T @ error
            values = clamp_ur10_joint_array(values + np.clip(delta, -7.0, 7.0))

    return ur10_joint_array_to_targets(best_values), best_position_error_norm, best_rotation_error_norm


def solve_ur10_position_only_ik(target_local_position, seed_targets):
    target = np.array(target_local_position, dtype=float)
    best_values = None
    best_position_error_norm = float("inf")

    for seed in build_ur10_ik_seed_targets(target, seed_targets):
        values = clamp_ur10_joint_array(ur10_joint_targets_to_array(seed))
        for _iteration in range(V3_DROP_ROBOT_IK_MAX_ITERATIONS):
            current = compute_ur10_ee_local_position_from_array(values)
            position_error = target - current
            position_error_norm = float(np.linalg.norm(position_error))
            if position_error_norm < best_position_error_norm:
                best_position_error_norm = position_error_norm
                best_values = np.array(values, dtype=float)
            if position_error_norm <= V3_DROP_ROBOT_IK_TOLERANCE:
                break

            jacobian = np.zeros((3, len(UR10_IK_JOINT_NAMES)), dtype=float)
            epsilon = 0.25
            for joint_index in range(len(UR10_IK_JOINT_NAMES)):
                perturbed = np.array(values, dtype=float)
                perturbed[joint_index] += epsilon
                perturbed = clamp_ur10_joint_array(perturbed)
                perturbed_position = compute_ur10_ee_local_position_from_array(perturbed)
                jacobian[:, joint_index] = (perturbed_position - current) / epsilon

            damping = 0.025
            normal = jacobian @ jacobian.T + (damping * damping) * np.eye(jacobian.shape[0], dtype=float)
            try:
                delta = jacobian.T @ np.linalg.solve(normal, position_error)
            except np.linalg.LinAlgError:
                delta = jacobian.T @ position_error
            values = clamp_ur10_joint_array(values + np.clip(delta, -7.0, 7.0))

    return ur10_joint_array_to_targets(best_values), best_position_error_norm


def solve_ur10_position_ik(target_local_position, seed_targets):
    target_rotation = compute_ur10_ee_local_pose_from_array(ur10_joint_targets_to_array(seed_targets))[1]
    joint_targets, position_error, _rotation_error = solve_ur10_pose_ik(
        target_local_position,
        target_rotation,
        seed_targets,
    )
    return joint_targets, position_error


class V3DropRobotTransferController:
    def __init__(self, *, stage, ur10_root_path, target_pallet_root, motion_target_path=None):
        self.stage = stage
        self.ur10_root_path = ur10_root_path
        self.target_pallet_root = target_pallet_root
        self.motion_target_path = motion_target_path or V3_DROP_ROBOT_MOTION_COMMANDER_TARGET
        self._ensure_motion_commander_target()
        self.reset_for_next_cycle()

    def _ensure_motion_commander_target(self):
        from pxr import Gf, UsdGeom

        prim = self.stage.GetPrimAtPath(self.motion_target_path)
        if not prim.IsValid():
            cube = UsdGeom.Cube.Define(self.stage, self.motion_target_path)
            cube.CreateSizeAttr(0.01)
            prim = cube.GetPrim()
        UsdGeom.Gprim(prim).CreateDisplayColorAttr().Set([Gf.Vec3f(0.15, 0.15, 0.15)])

    def _set_motion_commander_target_pose(self, world_position, world_rotation):
        target_quat = quat_from_matrix_wxyz(world_rotation)
        set_prim_pose(self.stage, self.motion_target_path, world_position, target_quat)
        target_position, target_orientation, _scale = get_prim_world_xform_components(
            self.stage,
            self.motion_target_path,
        )
        return target_position, matrix_from_quat_wxyz(target_orientation)

    def reset_for_next_cycle(self):
        self.started = False
        self.done = False
        self.item_index = 0
        self.item_time = 0.0
        self.items = []
        self.last_reported_item = None
        self.ik_seed_joints = dict(V3_DROP_ROBOT_HOME_JOINTS_DEG)
        self.current_joint_targets = dict(V3_DROP_ROBOT_HOME_JOINTS_DEG)

    def _get_object_prim_path(self, obj):
        prim_path = getattr(obj, "prim_path", None)
        if prim_path is None and hasattr(obj, "prim"):
            prim_path = str(obj.prim.GetPath())
        return prim_path

    def _get_drop_source_pallet_bounds(self, orchestrator):
        bounds = []
        for part, _position, _orientation in orchestrator.dropped_pallet_poses.values():
            prim_path = self._get_object_prim_path(part)
            if prim_path is None:
                continue
            try:
                bounds.append(compute_world_aligned_bounds(self.stage, prim_path))
            except Exception as exc:
                print(f"[HarimDemo] could not read dropped pallet bounds for V3 layout: {exc}")

        if not bounds:
            dropped_positions = [data["start_position"] for data in self.items]
            if not dropped_positions:
                raise RuntimeError("Cannot build V3 target layout without dropped pallet or item positions.")
            positions = np.array(dropped_positions, dtype=float)
            return positions.min(axis=0), positions.max(axis=0)

        bounds_min = np.min(np.array([pair[0] for pair in bounds], dtype=float), axis=0)
        bounds_max = np.max(np.array([pair[1] for pair in bounds], dtype=float), axis=0)
        return bounds_min, bounds_max

    def _measure_item_geometry(self, data):
        prim_path = self._get_object_prim_path(data["item"])
        if prim_path is None:
            data["item_height"] = 0.135
            data["item_bottom_offset"] = -0.0675
            data["item_top_offset"] = 0.0675
            data["pick_surface_root_offset"] = np.array(
                [0.0, 0.0, 0.0675 + V3_DROP_ROBOT_PICK_SURFACE_CLEARANCE],
                dtype=float,
            )
            self._set_fallback_grasp_reference(data)
            return
        try:
            item_min, item_max = compute_world_aligned_bounds(self.stage, prim_path)
            start_position = np.array(data["start_position"], dtype=float)
            data["item_height"] = max(0.08, float(item_max[2] - item_min[2]))
            data["item_bottom_offset"] = float(item_min[2] - start_position[2])
            data["item_top_offset"] = float(item_max[2] - start_position[2])
            pick_surface_world = np.array(
                [
                    (float(item_min[0]) + float(item_max[0])) * 0.5,
                    (float(item_min[1]) + float(item_max[1])) * 0.5,
                    float(item_max[2]) + V3_DROP_ROBOT_PICK_SURFACE_CLEARANCE,
                ],
                dtype=float,
            )
            data["pick_surface_root_offset"] = pick_surface_world - start_position
        except Exception as exc:
            print(f"[HarimDemo] could not read V3 drop item geometry: {exc}")
            data["item_height"] = 0.135
            data["item_bottom_offset"] = -0.0675
            data["item_top_offset"] = 0.0675
            data["pick_surface_root_offset"] = np.array(
                [0.0, 0.0, 0.0675 + V3_DROP_ROBOT_PICK_SURFACE_CLEARANCE],
                dtype=float,
            )
        self._set_original_grasp_reference(data, prim_path)

    def _set_fallback_grasp_reference(self, data):
        data["grasp_uses_original_collision"] = False
        data["grasp_base_root_offset"] = np.zeros(3, dtype=float)
        data["grasp_base_orientation"] = UPSIDE_DOWN_BIN_QUAT
        data["grasp_margin"] = V3_DROP_ROBOT_ORIGINAL_UPSIDE_DOWN_MARGIN
        data["grasp_target_axis"] = np.array([0.0, 0.0, -1.0], dtype=float)
        data["pick_target_rotation"] = orthonormal_rotation_from_axes(
            np.array([0.0, 0.0, -1.0], dtype=float),
            np.array([-1.0, 0.0, 0.0], dtype=float),
        )
        data["place_target_rotation"] = original_place_target_rotation()

    def _set_original_grasp_reference(self, data, prim_path):
        collision_path = f"{prim_path}/{V3_DROP_ROBOT_GRASP_COLLISION_RELATIVE_PATH}"
        collision_prim = self.stage.GetPrimAtPath(collision_path)
        if not collision_prim.IsValid():
            self._set_fallback_grasp_reference(data)
            return
        try:
            base_position, base_orientation, _base_scale = get_prim_world_xform_components(self.stage, collision_path)
        except Exception as exc:
            print(f"[HarimDemo] could not read original V3 grasp reference: {exc}")
            self._set_fallback_grasp_reference(data)
            return

        base_rotation = matrix_from_quat_wxyz(base_orientation)
        bin_ax = np.array(base_rotation[:, 0], dtype=float)
        bin_az = np.array(base_rotation[:, 2], dtype=float)
        bin_az_norm = np.linalg.norm(bin_az)
        if bin_az_norm < 1.0e-9:
            self._set_fallback_grasp_reference(data)
            return
        bin_az = bin_az / bin_az_norm
        needs_flip = float(np.dot(np.array([0.0, 0.0, 1.0], dtype=float), bin_az)) > 0.0
        target_axis = -bin_az if needs_flip else bin_az
        target_axis = target_axis / max(np.linalg.norm(target_axis), 1.0e-9)
        target_ay = -bin_ax if float(bin_ax[1]) < 0.0 else bin_ax
        target_rotation = orthonormal_rotation_from_axes(target_axis, target_ay)

        data["grasp_uses_original_collision"] = True
        data["grasp_base_root_offset"] = np.array(base_position, dtype=float) - np.array(data["start_position"], dtype=float)
        data["grasp_base_orientation"] = np.array(base_orientation, dtype=float)
        data["grasp_margin"] = 0.0025 if needs_flip else V3_DROP_ROBOT_ORIGINAL_UPSIDE_DOWN_MARGIN
        data["grasp_target_axis"] = target_axis
        data["grasp_bin_x_axis_points_to_robot"] = bool(float(bin_ax[1]) < 0.0)
        data["pick_target_rotation"] = target_rotation
        data["place_target_rotation"] = original_place_target_rotation()

    def _estimate_items_per_layer(self, dropped_sequence):
        z_values = sorted(float(position[2]) for _item, position, _orientation in dropped_sequence)
        if not z_values:
            return max(1, len(self.items))

        groups = []
        tolerance = 0.045
        for z_value in z_values:
            if not groups or abs(z_value - groups[-1][0]) > tolerance:
                groups.append([z_value])
            else:
                groups[-1].append(z_value)
        return max(1, max(len(group) for group in groups))

    def _build_target_positions(self, orchestrator, dropped_sequence):
        for data in self.items:
            self._measure_item_geometry(data)

        source_min, source_max = self._get_drop_source_pallet_bounds(orchestrator)
        source_center = (source_min + source_max) * 0.5

        pallet_min, pallet_max = compute_world_aligned_bounds(self.stage, self.target_pallet_root)
        pallet_center = (pallet_min + pallet_max) * 0.5
        target_top_z = float(pallet_max[2])
        items_per_layer = self._estimate_items_per_layer(dropped_sequence)
        layer_height = max(0.08, float(np.median([data["item_height"] for data in self.items])))
        target_bottom_offset = float(np.median([data["item_bottom_offset"] for data in self.items]))

        target_slots = []
        for _item, position, _orientation in dropped_sequence:
            source_position = np.array(position, dtype=float)
            source_offset = source_position - source_center
            target_slots.append(
                np.array(
                    [
                        pallet_center[0] + source_offset[0],
                        pallet_center[1] - source_offset[1],
                    ],
                    dtype=float,
                )
            )

        if not target_slots:
            target_slots = [np.array(pallet_center[:2], dtype=float) for _data in self.items]

        for target_index, data in enumerate(self.items):
            target_slot = target_slots[target_index % len(target_slots)]
            layer = target_index // items_per_layer
            target_bottom_z = target_top_z + layer * layer_height
            data["target_position"] = np.array(
                [
                    target_slot[0],
                    target_slot[1],
                    target_bottom_z - target_bottom_offset,
                ],
                dtype=float,
            )
            data["target_bottom_offset"] = target_bottom_offset
            data["target_orientation"] = np.array(UPSIDE_DOWN_BIN_QUAT, dtype=float)
            data["place_release_position"] = data["target_position"] + np.array(
                [0.0, 0.0, V3_DROP_ROBOT_PLACE_RELEASE_CLEARANCE],
                dtype=float,
            )

        print(
            "[HarimDemo] mapped V3 drop robot targets from delivered pallet layout: "
            f"source_center={source_center.tolist()}, target_center={pallet_center.tolist()}, "
            f"items_per_layer={items_per_layer}, layer_height={layer_height:.6f}, "
            f"target_bottom_offset={target_bottom_offset:.6f}, "
            f"target_y=mirrored_for_drop_robot, target_stack=bottom_first"
        )

    def _get_ee_world_pose(self):
        position, orientation, _scale = get_prim_world_xform_components(
            self.stage,
            f"{self.ur10_root_path}/{V3_DROP_ROBOT_IK_END_EFFECTOR_LINK}",
        )
        return np.array(position, dtype=float), matrix_from_quat_wxyz(orientation)

    def _get_ee_world_position(self):
        position, _rotation = self._get_ee_world_pose()
        return position

    def _joint_targets_ee_world_pose(self, joint_targets):
        local_matrix = compute_ur10_link_matrices(joint_targets)[V3_DROP_ROBOT_IK_END_EFFECTOR_LINK]
        root_position, root_orientation, _root_scale = get_prim_world_xform_components(
            self.stage,
            self.ur10_root_path,
        )
        root_rotation = matrix_from_quat_wxyz(root_orientation)
        ee_position = np.array(root_position, dtype=float) + root_rotation @ local_matrix[:3, 3]
        ee_rotation = root_rotation @ local_matrix[:3, :3]
        return ee_position, ee_rotation

    def _original_grasp_tcp_world(self, data, item_root_position, *, target_geometry=False):
        root_position = np.array(item_root_position, dtype=float)
        if data.get("grasp_uses_original_collision"):
            base_position = root_position + np.array(data["grasp_base_root_offset"], dtype=float)
            base_rotation = matrix_from_quat_wxyz(data["grasp_base_orientation"])
            bin_az = np.array(base_rotation[:, 2], dtype=float)
            bin_az = bin_az / max(np.linalg.norm(bin_az), 1.0e-9)
            tcp_position = base_position + float(data["grasp_margin"]) * bin_az
            if not target_geometry and data.get("pick_surface_root_offset") is not None:
                surface_position = root_position + np.array(data["pick_surface_root_offset"], dtype=float)
                tcp_position = np.array(tcp_position, dtype=float)
                tcp_position[2] = max(float(tcp_position[2]), float(surface_position[2]))
                data["pick_surface_world_z"] = float(surface_position[2])
            return tcp_position
        if not target_geometry:
            return root_position + np.array(data["pick_surface_root_offset"], dtype=float)
        return root_position + np.array([0.0, 0.0, data["item_height"] * 0.5 + V3_DROP_ROBOT_GRIPPER_CLEARANCE])

    def _prepare_item_arm_offsets(self, data):
        motion_start_joints = dict(self.current_joint_targets)
        previous_joints = dict(motion_start_joints)
        pick_tcp_world = self._original_grasp_tcp_world(data, data["start_position"])
        pick_axis = np.array(data.get("grasp_target_axis", [0.0, 0.0, -1.0]), dtype=float)
        pick_axis = pick_axis / max(np.linalg.norm(pick_axis), 1.0e-9)
        pick_rotation = np.array(data["pick_target_rotation"], dtype=float)
        pick_waypoint_world_poses = {
            "pick_approach_joints": (
                pick_tcp_world - pick_axis * V3_DROP_ROBOT_PICK_APPROACH_DISTANCE,
                pick_rotation,
            ),
            "pick_joints": (pick_tcp_world, pick_rotation),
            "pick_lift_joints": (
                pick_tcp_world + np.array([0.0, 0.0, V3_DROP_ROBOT_PICK_LIFT_DISTANCE], dtype=float),
                pick_rotation,
            ),
        }

        waypoint_errors = {}
        waypoint_rotation_errors = {}
        for key, (world_position, world_rotation) in pick_waypoint_world_poses.items():
            target_position, target_rotation = self._set_motion_commander_target_pose(world_position, world_rotation)
            local_position = transform_world_point_to_prim_local(self.stage, self.ur10_root_path, target_position)
            local_rotation = transform_world_rotation_to_prim_local(self.stage, self.ur10_root_path, target_rotation)
            joint_targets, position_error, rotation_error = solve_ur10_pose_ik(
                local_position,
                local_rotation,
                previous_joints,
            )
            data[key] = joint_targets
            waypoint_errors[key] = position_error
            waypoint_rotation_errors[key] = rotation_error
            previous_joints = joint_targets

        set_ur10_visual_pose_from_joint_targets(self.stage, self.ur10_root_path, data["pick_joints"])
        pick_ee_position, pick_ee_rotation = self._joint_targets_ee_world_pose(data["pick_joints"])
        item_start_rotation = matrix_from_quat_wxyz(data["orientation"])
        data["grip_local_position"] = pick_ee_rotation.T @ (np.array(data["start_position"], dtype=float) - pick_ee_position)
        data["grip_local_rotation"] = pick_ee_rotation.T @ item_start_rotation

        set_ur10_visual_pose_from_joint_targets(self.stage, self.ur10_root_path, data["pick_lift_joints"])
        _pick_lift_ee_position, pick_lift_ee_rotation = self._joint_targets_ee_world_pose(data["pick_lift_joints"])
        place_rotation = adjust_rotation_about_x_if_opposite(
            pick_lift_ee_rotation,
            np.array(data["place_target_rotation"], dtype=float),
        )
        place_tcp_world = np.array(data["target_position"], dtype=float) - place_rotation @ data["grip_local_position"]
        data["place_target_rotation"] = place_rotation
        place_waypoint_world_poses = {
            "place_approach_joints": (
                place_tcp_world + np.array([0.0, 0.0, V3_DROP_ROBOT_PLACE_APPROACH_DISTANCE], dtype=float),
                place_rotation,
            ),
            "place_joints": (place_tcp_world, place_rotation),
            "place_lift_joints": (
                place_tcp_world + np.array([0.0, 0.0, V3_DROP_ROBOT_PLACE_LIFT_DISTANCE], dtype=float),
                place_rotation,
            ),
        }

        for key, (world_position, world_rotation) in place_waypoint_world_poses.items():
            target_position, target_rotation = self._set_motion_commander_target_pose(world_position, world_rotation)
            local_position = transform_world_point_to_prim_local(self.stage, self.ur10_root_path, target_position)
            local_rotation = transform_world_rotation_to_prim_local(self.stage, self.ur10_root_path, target_rotation)
            joint_targets, position_error = solve_ur10_position_only_ik(
                local_position,
                previous_joints,
            )
            _actual_position, actual_rotation = compute_ur10_ee_local_pose_from_array(
                ur10_joint_targets_to_array(joint_targets)
            )
            rotation_error = float(np.linalg.norm(rotation_error_vector(local_rotation, actual_rotation)))
            data[key] = joint_targets
            waypoint_errors[key] = position_error
            waypoint_rotation_errors[key] = rotation_error
            previous_joints = joint_targets
        self.ik_seed_joints = dict(previous_joints)
        if os.environ.get("HARIM_V3_IK_DEBUG"):
            root_position, _root_orientation, _root_scale = get_prim_world_xform_components(
                self.stage, self.ur10_root_path
            )
            print(
                "[HarimDemo] V3 drop robot IK debug: "
                f"root={root_position.tolist()}, start={data['start_position'].tolist()}, "
                f"target={data['target_position'].tolist()}, pick_tcp_world={pick_tcp_world.tolist()}, "
                f"place_tcp_world={place_tcp_world.tolist()}"
            )

        set_ur10_visual_pose_from_joint_targets(self.stage, self.ur10_root_path, data["place_joints"])
        place_ee_position, place_ee_rotation = self._joint_targets_ee_world_pose(data["place_joints"])
        data["pick_ee_position"] = pick_ee_position
        data["place_ee_position"] = place_ee_position
        data["pick_grip_offset"] = np.array(data["start_position"], dtype=float) - pick_ee_position
        data["place_grip_offset"] = np.array(data["target_position"], dtype=float) - place_ee_position
        data["place_attached_position"] = place_ee_position + place_ee_rotation @ data["grip_local_position"]
        data["place_attached_orientation"] = data["target_orientation"]
        data["release_position"] = None
        data["release_orientation"] = None
        data["motion_start_joints"] = motion_start_joints
        data["gripper_closed"] = False
        data["gripper_opened"] = False
        data["arm_prepared"] = True
        data["motion_keyframes"] = [
            (0.0, data["motion_start_joints"]),
            (V3_DROP_ROBOT_PICK_APPROACH_PHASE_END, data["pick_approach_joints"]),
            (V3_DROP_ROBOT_PICK_REACH_PHASE_END, data["pick_joints"]),
            (V3_DROP_ROBOT_PICK_WAIT_PHASE_END, data["pick_joints"]),
            (V3_DROP_ROBOT_PICK_LIFT_PHASE_END, data["pick_lift_joints"]),
            (V3_DROP_ROBOT_PLACE_APPROACH_PHASE_END, data["place_approach_joints"]),
            (V3_DROP_ROBOT_PLACE_REACH_PHASE_END, data["place_joints"]),
            (V3_DROP_ROBOT_PLACE_WAIT_PHASE_END, data["place_joints"]),
            (V3_DROP_ROBOT_PLACE_LIFT_PHASE_END, data["place_lift_joints"]),
            (1.0, data["place_lift_joints"]),
        ]
        grasp_mode = "original_collision_cube" if data.get("grasp_uses_original_collision") else "fallback_root_height"
        print(
            "[HarimDemo] prepared V3 drop robot grasp target: "
            f"pick_error={waypoint_errors['pick_joints']:.4f}, "
            f"pick_rot_error={waypoint_rotation_errors['pick_joints']:.4f}, "
            f"place_error={waypoint_errors['place_joints']:.4f}, "
            f"place_rot_error={waypoint_rotation_errors['place_joints']:.4f}, "
            f"grasp={grasp_mode}, "
            f"motion_target={self.motion_target_path}, "
            "process=motion_commander_target_pose_ik_reach_wait_close_lift_then_reach_wait_open_lift"
        )
        set_ur10_visual_pose_from_joint_targets(self.stage, self.ur10_root_path, data["motion_start_joints"])

    def _start(self, orchestrator):
        self.items = []
        dropped_sequence = list(getattr(orchestrator, "dropped_item_sequence", []))
        if not dropped_sequence:
            dropped_sequence = list(orchestrator.dropped_item_poses.values())

        for item, position, orientation in reversed(dropped_sequence):
            self.items.append(
                {
                    "item": item,
                    "start_position": np.array(position, dtype=float),
                    "orientation": orientation,
                    "target_position": None,
                    "target_orientation": np.array(UPSIDE_DOWN_BIN_QUAT, dtype=float),
                    "place_release_position": None,
                    "pick_grip_offset": None,
                    "place_grip_offset": None,
                    "grip_local_position": None,
                    "grip_local_rotation": None,
                    "place_attached_position": None,
                    "place_attached_orientation": None,
                    "release_position": None,
                    "release_orientation": None,
                    "item_height": None,
                    "item_bottom_offset": None,
                    "item_top_offset": None,
                    "pick_surface_root_offset": None,
                    "target_bottom_offset": None,
                    "grasp_uses_original_collision": False,
                    "grasp_base_root_offset": None,
                    "grasp_base_orientation": None,
                    "grasp_margin": None,
                    "grasp_target_axis": None,
                    "pick_target_rotation": None,
                    "place_target_rotation": None,
                    "pick_approach_joints": None,
                    "pick_joints": None,
                    "pick_lift_joints": None,
                    "place_approach_joints": None,
                    "place_joints": None,
                    "place_lift_joints": None,
                    "motion_start_joints": None,
                    "motion_keyframes": None,
                    "arm_prepared": False,
                }
            )

        if not self.items:
            self.done = True
            return

        self._build_target_positions(orchestrator, dropped_sequence)
        self.started = True
        self.item_index = 0
        self.item_time = 0.0
        print(
            "[HarimDemo] started V3 drop robot transfer after AMR exit: "
            f"items={len(self.items)}, order=reverse_loaded, target_pallet={self.target_pallet_root}"
        )

    def _set_item_pose(self, data, position, orientation=None):
        item = data["item"]
        pose_orientation = data["orientation"] if orientation is None else orientation
        try:
            item.set_world_pose(position=np.array(position, dtype=float), orientation=pose_orientation)
            item.set_linear_velocity(np.zeros(3))
            item.set_angular_velocity(np.zeros(3))
        except Exception:
            item.set_world_pose(position=np.array(position, dtype=float), orientation=pose_orientation)

    def _hold_transfer_items(self):
        for index, data in enumerate(self.items):
            if not self.done and index == self.item_index:
                continue
            if self.done or index < self.item_index:
                position = data["release_position"] if data.get("release_position") is not None else data["target_position"]
                orientation = (
                    data["release_orientation"]
                    if data.get("release_orientation") is not None
                    else data["target_orientation"]
                )
                self._set_item_pose(data, position, orientation)
            else:
                self._set_item_pose(data, data["start_position"])

    def _set_drop_robot_motion_pose(self, data, t):
        joint_targets = interpolate_joint_targets_catmull_rom(data["motion_keyframes"], t)
        set_ur10_visual_pose_from_joint_targets(self.stage, self.ur10_root_path, joint_targets)
        return self._joint_targets_ee_world_pose(joint_targets)

    def _item_pose_from_drop_robot(self, data, ee_position, ee_rotation, t):
        if t < V3_DROP_ROBOT_PICK_WAIT_PHASE_END:
            return np.array(data["start_position"], dtype=float), data["orientation"]
        if data.get("release_position") is not None:
            return np.array(data["release_position"], dtype=float), data["release_orientation"]
        attached_position = np.array(ee_position, dtype=float) + np.array(ee_rotation, dtype=float) @ np.array(
            data["grip_local_position"],
            dtype=float,
        )
        attached_orientation = data["target_orientation"]
        if t >= V3_DROP_ROBOT_PLACE_WAIT_PHASE_END:
            data["release_position"] = attached_position
            data["release_orientation"] = data["target_orientation"]
            return data["release_position"], data["release_orientation"]
        return attached_position, attached_orientation

    def _item_position_from_drop_robot(self, data, ee_position, ee_rotation, t):
        position, _orientation = self._item_pose_from_drop_robot(data, ee_position, ee_rotation, t)
        return position

    def step(self, dt, orchestrator):
        if self.done:
            self._hold_transfer_items()
            return

        if not self.started:
            can_start_from_reset_gate = (
                orchestrator.state == TransferState.RESET_CYCLE
                and bool(getattr(orchestrator, "dropped_item_poses", {}))
            )
            can_start_from_done_idle = orchestrator.state == TransferState.DONE_IDLE and orchestrator.completed_cycles > 0
            if can_start_from_reset_gate or can_start_from_done_idle:
                self._start(orchestrator)
            return

        if self.item_index >= len(self.items):
            set_ur10_visual_pose_from_joint_targets(self.stage, self.ur10_root_path, V3_DROP_ROBOT_HOME_JOINTS_DEG)
            self.done = True
            print("[HarimDemo] completed V3 drop robot transfer to pushcart pallet")
            return

        self._hold_transfer_items()
        data = self.items[self.item_index]
        if not data.get("arm_prepared"):
            print(f"[HarimDemo] V3 drop robot preparing box {self.item_index + 1}/{len(self.items)}")
            self._prepare_item_arm_offsets(data)
            return

        if self.last_reported_item != self.item_index:
            self.last_reported_item = self.item_index
            print(f"[HarimDemo] V3 drop robot moving box {self.item_index + 1}/{len(self.items)}")

        self.item_time += min(float(dt), V3_DROP_ROBOT_MAX_DT)
        t = clamp(self.item_time / V3_DROP_ROBOT_ITEM_SECONDS, 0.0, 1.0)
        ee_position, ee_rotation = self._set_drop_robot_motion_pose(data, t)
        item = data["item"]
        if not data["gripper_closed"] and t >= V3_DROP_ROBOT_PICK_WAIT_PHASE_END:
            data["gripper_closed"] = True
            print(f"[HarimDemo] V3 drop robot closed gripper on box {self.item_index + 1}/{len(self.items)}")
        if not data["gripper_opened"] and t >= V3_DROP_ROBOT_PLACE_WAIT_PHASE_END:
            data["gripper_opened"] = True
            print(f"[HarimDemo] V3 drop robot opened gripper on box {self.item_index + 1}/{len(self.items)}")
        position, orientation = self._item_pose_from_drop_robot(data, ee_position, ee_rotation, t)
        try:
            self._set_item_pose(data, position, orientation)
            orchestrator._stop_dynamic_item(item)
        except Exception as exc:
            print(f"[HarimDemo] could not move V3 transferred box: {exc}")

        if t >= 1.0:
            final_position = data["release_position"] if data.get("release_position") is not None else data["target_position"]
            final_orientation = (
                data["release_orientation"]
                if data.get("release_orientation") is not None
                else data["target_orientation"]
            )
            data["start_position"] = np.array(final_position, dtype=float)
            self._set_item_pose(data, final_position, final_orientation)
            print(f"[HarimDemo] V3 drop robot placed box {self.item_index + 1}/{len(self.items)}")
            self.current_joint_targets = dict(data["place_lift_joints"])
            self.item_index += 1
            self.item_time = 0.0


def main():
    args = parse_args()
    configure_local_runtime_dirs()

    from isaacsim import SimulationApp

    window_width = int(os.environ.get("HARIM_WINDOW_WIDTH", "1440"))
    window_height = int(os.environ.get("HARIM_WINDOW_HEIGHT", "900"))
    render_width = int(os.environ.get("HARIM_RENDER_WIDTH", os.environ.get("HARIM_WINDOW_WIDTH", "1280")))
    render_height = int(os.environ.get("HARIM_RENDER_HEIGHT", os.environ.get("HARIM_WINDOW_HEIGHT", "720")))
    fullscreen = os.environ.get("HARIM_WINDOW_FULLSCREEN", "").lower() in ("1", "true", "yes", "on")
    extra_args = ["--/app/window/fullscreen=true"] if fullscreen else []

    simulation_app = SimulationApp(
        {
            "headless": args.headless,
            "width": render_width,
            "height": render_height,
            "window_width": window_width,
            "window_height": window_height,
            "sync_loads": True,
            "renderer": "RaytracedLighting",
            "extra_args": extra_args,
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
    deactivate_stage_prims_containing(usd_context.get_stage(), "/World/Ur10Table", ("flip", "pallet_holder", "dolly"))
    SingleXFormPrim("/World/Background", position=[10.00, 2.00, -1.18180], orientation=[0.7071, 0, 0, 0.7071])

    card_box_material, card_box_display_color = resolve_background_card_box_style(usd_context.get_stage())
    styled_cardboard_box_paths = set()

    def apply_cardboard_box_color(root_path):
        applied_count = apply_background_card_box_style_to_box_targets(
            usd_context.get_stage(),
            root_path,
            card_box_material,
            card_box_display_color,
            styled_cardboard_box_paths,
        )
        if applied_count > 0:
            print(f"[HarimDemo] matched {applied_count} cardboard box visuals to background card box color")

    def apply_pallet_box_color():
        applied_count = apply_box_style_to_all_gprims(
            usd_context.get_stage(),
            "/World/Ur10Table/pallet",
            card_box_material,
            card_box_display_color,
        )
        if applied_count > 0:
            print(f"[HarimDemo] matched {applied_count} pallet visuals to cardboard box color")

    apply_cardboard_box_color("/World/Background")
    apply_pallet_box_color()

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
    stack_center_x = float(np.array(stack_coordinates, dtype=float)[:, 0].mean())
    print(f"[HarimDemo] set stack drop center X: {stack_center_x:.6f} for pallet X {DEFAULT_PALLET_X:.6f}")
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
        name_prefix="PickupSlideStation",
    )
    wait_for_stage_loading(simulation_app, usd_context, "image-matched sliding station")
    set_slide_station_side_y_offsets(
        usd_context.get_stage(),
        f"{harim_root}/PickupSlideStation",
        left_y=PICKUP_STATION_LEFT_Y,
        right_y=PICKUP_STATION_RIGHT_Y,
    )
    station_top_z = compute_world_z_bounds(usd_context.get_stage(), f"{harim_root}/PickupSlideStation")[1]

    iw_hub_usd = assets_root + "/Isaac/Samples/AnimRobot/iw_hub.usd"
    add_reference_to_stage(iw_hub_usd, f"{harim_root}/iw_hub")
    wait_for_stage_loading(simulation_app, usd_context, "iw_hub")
    set_prim_scale(usd_context.get_stage(), f"{harim_root}/iw_hub", DEFAULT_AMR_SCALE)
    print(f"[HarimDemo] set iw_hub AMR scale: {DEFAULT_AMR_SCALE.tolist()}")
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
    for part in pallet_parts:
        position, _orientation = part.get_world_pose()
        corrected_position = np.array(position, dtype=float)
        original_x = corrected_position[0]
        corrected_position[0] = DEFAULT_PALLET_X
        part.set_world_pose(corrected_position)
        print(
            "[HarimDemo] set pallet initial X: "
            f"{part.name} x={original_x:.6f} -> {DEFAULT_PALLET_X:.6f}"
        )
    original_pallet_bottom_z = min(compute_world_z_bounds(usd_context.get_stage(), part.prim)[0] for part in pallet_parts)
    original_pallet_top_z = max(compute_world_z_bounds(usd_context.get_stage(), part.prim)[1] for part in pallet_parts)
    pallet_z_shift = station_top_z - original_pallet_bottom_z
    if abs(pallet_z_shift) > 1e-6:
        for part in pallet_parts:
            position, _orientation = part.get_world_pose()
            corrected_position = np.array(position, dtype=float)
            corrected_position[2] += pallet_z_shift
            part.set_world_pose(corrected_position)
        stack_coordinates = [
            np.array(coord, dtype=float) + np.array([0.0, 0.0, pallet_z_shift], dtype=float)
            for coord in stack_coordinates
        ]
        decider_network.context.stack_coordinates = stack_coordinates
    pallet_bottom_z = original_pallet_bottom_z + pallet_z_shift
    pallet_top_z = original_pallet_top_z + pallet_z_shift
    pallet_height = max(0.0, pallet_top_z - pallet_bottom_z)
    pickup_pallet_position = np.array(pallet_parts[0].get_world_pose()[0], dtype=float)
    drop_pallet_position = pickup_pallet_position + np.array(
        [args.drop_x - args.pickup_x, args.drop_y - args.pickup_y, 0.0],
        dtype=float,
    )
    print(
        "[HarimDemo] aligned pallet to PickupSlideStation top: "
        f"station_top_z={station_top_z:.6f}, pallet_bottom_z={original_pallet_bottom_z:.6f}, "
        f"pallet_height={pallet_height:.6f}, "
        f"z_shift={pallet_z_shift:.6f}"
    )
    apply_pallet_box_color()
    drop_area_info = place_reversed_drop_area(
        stage=usd_context.get_stage(),
        ur10_table_usd=ur10_assets.ur10_table_usd,
        add_reference_to_stage=add_reference_to_stage,
        wait_for_stage_loading=wait_for_stage_loading,
        simulation_app=simulation_app,
        usd_context=usd_context,
        harim_root=harim_root,
        source_pallet_position=pickup_pallet_position,
        target_pallet_position=drop_pallet_position,
    )
    v3_dropoff_info = place_v3_dropoff_pushcart_target(
        stage=usd_context.get_stage(),
        simulation_app=simulation_app,
        harim_root=harim_root,
        source_pallet_path=str(pallet_parts[0].prim.GetPath()),
        source_pallet_position=pickup_pallet_position,
        target_pallet_position=drop_pallet_position,
        drop_area_info=drop_area_info,
    )
    amr_pickup_pose = np.array([args.pickup_x, args.pickup_y, args.amr_z], dtype=float)
    nominal_amr_drop_pose = np.array([args.drop_x, args.drop_y, args.amr_z], dtype=float)
    reversed_amr_drop_pose = reversed_amr_drop_pose_from_pickup(
        amr_pickup_pose,
        pickup_pallet_position,
        drop_pallet_position,
        args.amr_z,
    )
    amr_drop_pose = backoff_amr_drop_pose_toward_reversed_pickup(
        nominal_amr_drop_pose,
        reversed_amr_drop_pose,
        V3_AMR_DROP_STATION_BACKOFF,
    )
    print(
        "[HarimDemo] matched AMR pallet drop pose toward reversed pickup depth: "
        f"pickup_pose={amr_pickup_pose.tolist()}, "
        f"source_pallet={pickup_pallet_position.tolist()}, "
        f"target_pallet={drop_pallet_position.tolist()}, "
        f"nominal_drop_pose={nominal_amr_drop_pose.tolist()}, "
        f"reversed_reference_drop_pose={reversed_amr_drop_pose.tolist()}, "
        f"backoff={V3_AMR_DROP_STATION_BACKOFF:.3f}, "
        f"drop_pose={amr_drop_pose.tolist()}"
    )

    class SelfTestBinState:
        def __init__(self, bin_obj):
            self.bin_obj = bin_obj

    self_test_payload = []
    self_test_payload_poses = []
    if args.self_test_force_stack_complete:
        payload_specs = []
        for idx, coord in enumerate(stack_coordinates):
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
            self_test_payload_poses.append(np.array(coord, dtype=float))

    orchestrator = HarimTransferOrchestrator(
        world=world,
        context=decider_network.context,
        task=task,
        amr_prim=amr,
        amr_lift_prim=amr_lift,
        stage=usd_context.get_stage(),
        amr_lift_path=amr_lift_path if amr_lift is not None else None,
        lift_surface_z=pallet_bottom_z,
        lift_plate=lift_plate,
        pallet_parts=pallet_parts,
        stack_coordinates=stack_coordinates,
        args=args,
        drop_pose_override=amr_drop_pose,
        stabilize_source_stack=is_v3_dropoff_cart_enabled(),
    )
    v3_drop_robot_controller = None
    if v3_dropoff_info is not None:
        v3_drop_robot_controller = V3DropRobotTransferController(
            stage=usd_context.get_stage(),
            ur10_root_path=f"{drop_area_info['drop_table_root']}/ur10",
            target_pallet_root=v3_dropoff_info["pallet_root"],
            motion_target_path=V3_DROP_ROBOT_MOTION_COMMANDER_TARGET,
        )

    world.reset()
    world.play()
    decider_network.context.stack_coordinates = stack_coordinates
    orchestrator.reset_visual_state()

    def reset_cycle_visual_state():
        styled_cardboard_box_paths.clear()
        apply_cardboard_box_color("/World/Background")
        apply_pallet_box_color()

    def force_self_test_stack_complete():
        if not args.self_test_force_stack_complete:
            return
        if self_test_payload:
            for item, position in zip(self_test_payload, self_test_payload_poses):
                item.set_world_pose(position=np.array(position, dtype=float), orientation=UPSIDE_DOWN_BIN_QUAT)
                item.set_linear_velocity(np.zeros(3))
                item.set_angular_velocity(np.zeros(3))
            decider_network.context.stacked_bins = [SelfTestBinState(item) for item in self_test_payload]
            decider_network.context.stack_coordinates = [
                np.array(position, dtype=float) for position in self_test_payload_poses
            ]

    def step_demo_frame():
        world.step(render=not args.headless)
        apply_cardboard_box_color("/World/Ur10Table/bins")
        force_self_test_stack_complete()
        physics_dt = world.get_physics_dt()
        if (
            v3_drop_robot_controller is not None
            and orchestrator.state == TransferState.RESET_CYCLE
            and not v3_drop_robot_controller.done
            and getattr(orchestrator, "dropped_item_poses", {})
        ):
            v3_drop_robot_controller.step(physics_dt, orchestrator)
            return

        completed_cycles_before_step = orchestrator.completed_cycles
        orchestrator.step(physics_dt)
        if (
            orchestrator.completed_cycles != completed_cycles_before_step
            and orchestrator.state == TransferState.WAIT_STACK_COMPLETE
        ):
            reset_cycle_visual_state()
            if v3_drop_robot_controller is not None:
                v3_drop_robot_controller.reset_for_next_cycle()
            return

        if v3_drop_robot_controller is not None:
            v3_drop_robot_controller.step(physics_dt, orchestrator)

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
