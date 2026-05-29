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
PALLET_TUNNEL_HALF_WIDTH = 0.42
DROP_WORKSTATION_Z = -0.73
UPSIDE_DOWN_BIN_QUAT = np.array([0.0, 0.0, 1.0, 0.0], dtype=float)
PALLET_DECK_OFFSETS = (
    np.array([0.0, -0.44, 0.0], dtype=float),
    np.array([0.0, 0.0, 0.0], dtype=float),
    np.array([0.0, 0.44, 0.0], dtype=float),
)
PALLET_BLOCK_OFFSETS = (
    np.array([-0.38, -0.56, -0.075], dtype=float),
    np.array([0.0, -0.56, -0.075], dtype=float),
    np.array([0.38, -0.56, -0.075], dtype=float),
    np.array([-0.38, 0.56, -0.075], dtype=float),
    np.array([0.0, 0.56, -0.075], dtype=float),
    np.array([0.38, 0.56, -0.075], dtype=float),
)
PALLET_PART_OFFSETS = PALLET_DECK_OFFSETS + PALLET_BLOCK_OFFSETS


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
        center = np.array([self.args.pickup_x, self.args.pickup_y, -0.60 + self.lift_offset], dtype=float)
        for part, offset in zip(self.pallet_parts, PALLET_PART_OFFSETS):
            part.set_world_pose(position=center + offset, orientation=yaw_to_quat(0.0))

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
                item.set_world_pose(position=pos, orientation=orient)
                self._stop_dynamic_item(item)
            except Exception as exc:
                print(f"[HarimDemo] could not hold dropped item: {exc}")
        for part, pos, orient in self.dropped_pallet_poses.values():
            part.set_world_pose(position=pos, orientation=orient)

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
            self._reset_pallet_pose()
            self._apply_lift_delta_to_stack(dz)
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

    from isaacsim.core.api.objects.cuboid import VisualCuboid
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
    from isaacsim.cortex.framework.df import DfDecider, DfDecision, DfNetwork, DfStateMachineDecider
    from isaacsim.cortex.framework.dfb import make_go_home
    from isaacsim.cortex.framework.robot import CortexUr10
    from isaacsim.cortex.behaviors.ur10 import bin_stacking_behavior as behavior
    from pxr import UsdGeom

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

    stack_coordinates = make_stack_coordinates(args.stack_cols, args.stack_rows, args.stack_layers)
    task = BinStackingTask("/World/Ur10Table", ur10_assets)
    task.set_up_scene(world.scene)
    world.add_task(task)

    decider_network = make_no_flip_decider_network(robot, lambda _diagnostic: None)
    decider_network.context.stack_coordinates = stack_coordinates
    world.add_decider_network(decider_network)

    harim_root = "/World/HarimDemo"
    omni.kit.commands.execute("CreatePrim", prim_path=harim_root, prim_type="Xform")

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

        for side_idx, y_offset in enumerate((-0.58, 0.58)):
            workstation_parts.append(
                world.scene.add(
                    VisualCuboid(
                        f"{harim_root}/DropSlideRail_{side_idx}",
                        name=f"harim_drop_slide_rail_{side_idx}",
                        position=np.array([args.drop_x, args.drop_y + y_offset, DROP_WORKSTATION_Z], dtype=float),
                        scale=np.array([1.80, 0.10, 0.09]),
                        color=rail_color,
                    )
                )
            )
            for roller_idx, x_offset in enumerate((-0.62, -0.22, 0.22, 0.62)):
                workstation_parts.append(
                    world.scene.add(
                        VisualCuboid(
                            f"{harim_root}/DropSlideRoller_{side_idx}_{roller_idx}",
                            name=f"harim_drop_slide_roller_{side_idx}_{roller_idx}",
                            position=np.array(
                                [args.drop_x + x_offset, args.drop_y + y_offset, DROP_WORKSTATION_Z + 0.07],
                                dtype=float,
                            ),
                            scale=np.array([0.12, 0.24, 0.035]),
                            color=roller_color,
                        )
                    )
                )
            for leg_idx, x_offset in enumerate((-0.78, 0.78)):
                workstation_parts.append(
                    world.scene.add(
                        VisualCuboid(
                            f"{harim_root}/DropSlideLeg_{side_idx}_{leg_idx}",
                            name=f"harim_drop_slide_leg_{side_idx}_{leg_idx}",
                            position=np.array(
                                [args.drop_x + x_offset, args.drop_y + y_offset, DROP_WORKSTATION_Z - 0.18],
                                dtype=float,
                            ),
                            scale=np.array([0.10, 0.10, 0.36]),
                            color=leg_color,
                        )
                    )
                )
        return workstation_parts

    create_drop_slide_workstation()

    lift_plate = world.scene.add(
        VisualCuboid(
            f"{harim_root}/IwHubLiftPlate",
            name="harim_iw_hub_lift_plate",
            position=np.array([args.pickup_x + AMR_START_STANDOFF, args.pickup_y, args.amr_z + 0.33]),
            scale=np.array([1.10, 0.72, 0.035]),
            color=np.array([0.10, 0.12, 0.13]),
        )
    )

    pallet_parts = []
    plank_color = np.array([0.62, 0.44, 0.23])
    block_color = np.array([0.42, 0.29, 0.15])
    for idx in range(3):
        pallet_parts.append(
            world.scene.add(
                VisualCuboid(
                    f"{harim_root}/PalletDeck_{idx}",
                    name=f"harim_pallet_deck_{idx}",
                    scale=np.array([1.10, 0.12, 0.055]),
                    color=plank_color,
                )
            )
        )
    for idx in range(6):
        pallet_parts.append(
            world.scene.add(
                VisualCuboid(
                    f"{harim_root}/PalletBlock_{idx}",
                    name=f"harim_pallet_block_{idx}",
                    scale=np.array([0.16, 0.14, 0.09]),
                    color=block_color,
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
