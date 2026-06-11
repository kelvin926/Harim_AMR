import importlib.util
import unittest
from pathlib import Path

import numpy as np


DEMO_PATH = Path(__file__).resolve().parents[1] / "scripts" / "run_harim_pallet_demo.py"


def load_demo_module():
    spec = importlib.util.spec_from_file_location("harim_demo", DEMO_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rotate_vector_by_quat(vector, quat):
    w, x, y, z = quat
    q_vec = np.array([x, y, z], dtype=float)
    vector = np.array(vector, dtype=float)
    return vector + 2.0 * np.cross(q_vec, np.cross(q_vec, vector) + w * vector)


class FakePosePrim:
    def __init__(self, name, position=(0.0, 0.0, 0.0)):
        self.name = name
        self.position = np.array(position, dtype=float)
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.linear_velocity = None
        self.angular_velocity = None

    def set_world_pose(self, position=None, orientation=None):
        if position is not None:
            self.position = np.array(position, dtype=float)
        if orientation is not None:
            self.orientation = np.array(orientation, dtype=float)

    def get_world_pose(self):
        return self.position.copy(), self.orientation.copy()

    def set_linear_velocity(self, velocity):
        self.linear_velocity = np.array(velocity, dtype=float)

    def set_angular_velocity(self, velocity):
        self.angular_velocity = np.array(velocity, dtype=float)


class FakeBinState:
    def __init__(self, item):
        self.bin_obj = item


class FakeContext:
    def __init__(self, items):
        self.stack_complete = False
        self.stack_coordinates = []
        self.stacked_bins = [FakeBinState(item) for item in items]


class FakeWorld:
    def __init__(self):
        self.reset_count = 0
        self.play_count = 0

    def reset(self):
        self.reset_count += 1

    def play(self):
        self.play_count += 1


class FakeSimulationApp:
    def __init__(self):
        self.update_count = 0

    def update(self):
        self.update_count += 1


class FakeUsdContext:
    def __init__(self, statuses):
        self.statuses = list(statuses)
        self.status_index = 0

    def get_stage_loading_status(self):
        status = self.statuses[min(self.status_index, len(self.statuses) - 1)]
        self.status_index += 1
        return status


class Args:
    pickup_x = 0.82
    pickup_y = -0.31
    drop_x = 2.45
    drop_y = -0.31
    amr_z = -1.05
    lift_height = 0.11
    move_speed = 10.0
    cycles = 1


class RepeatArgs(Args):
    cycles = 0


class HarimTransferOrchestratorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.demo = load_demo_module()

    def build_orchestrator(self, args, amr_lift_prim=None):
        items = [FakePosePrim(f"bin_{idx}", (0.7 + 0.1 * idx, -0.31, -0.50)) for idx in range(3)]
        context = FakeContext(items)
        world = FakeWorld()
        orchestrator = self.demo.HarimTransferOrchestrator(
            world=world,
            context=context,
            task=None,
            amr_prim=FakePosePrim("iw_hub"),
            amr_lift_prim=amr_lift_prim,
            lift_plate=None,
            pallet_parts=[FakePosePrim("example_pallet", (Args.pickup_x, Args.pickup_y, -0.60))],
            stack_coordinates=self.demo.make_stack_coordinates(2, 2, 1),
            args=args,
        )
        return orchestrator, context, world, items

    def run_until(self, orchestrator, predicate, max_steps=200):
        for _ in range(max_steps):
            orchestrator.step(0.1)
            if predicate():
                return
        self.fail(f"orchestrator did not reach expected state; current state={orchestrator.state}")

    def test_single_cycle_detaches_and_idles(self):
        orchestrator, context, world, items = self.build_orchestrator(Args())
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.DONE_IDLE)

        self.assertEqual(orchestrator.completed_cycles, 1)
        self.assertFalse(orchestrator.carrying)
        self.assertEqual(world.reset_count, 0)
        self.assertLess(orchestrator.get_amr_position()[0], Args.drop_x)
        for item in items:
            np.testing.assert_allclose(item.linear_velocity, np.zeros(3))
            np.testing.assert_allclose(item.angular_velocity, np.zeros(3))

    def test_default_drop_distance_is_over_ten_meters(self):
        self.assertGreaterEqual(self.demo.DEFAULT_DROP_X - self.demo.DEFAULT_PICKUP_X, 10.0)

    def test_amr_starts_far_from_table_side_and_approaches_from_drop_side(self):
        orchestrator, _context, _world, _items = self.build_orchestrator(Args())

        self.assertGreater(orchestrator.start_pose[0], Args.pickup_x + 3.0)
        self.assertGreater(orchestrator.approach_pose[0], Args.pickup_x)
        self.assertLess(orchestrator.pickup_pose[0], orchestrator.approach_pose[0])

    def test_spawned_bins_are_upside_down_to_skip_flip_station(self):
        for _ in range(10):
            _position, orientation = self.demo.random_bin_spawn_transform()
            local_z_in_world = rotate_vector_by_quat([0.0, 0.0, 1.0], orientation)

            self.assertLess(local_z_in_world[2], -0.99)

    def test_demo_uses_no_flip_dispatch(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("class NoFlipDispatch", source)
        self.assertIn("make_no_flip_decider_network", source)
        self.assertIn("deactivate_stage_prims_containing", source)
        self.assertIn('"pallet_holder"', source)
        self.assertNotIn('("flip", "pallet", "pallet_holder")', source)
        self.assertNotIn('add_child("flip_bin"', source)
        self.assertNotIn("behavior.make_decider_network", source)

    def test_demo_uses_image_matched_v2_slide_station_and_drop_area(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("collect_example_pallet_parts", source)
        self.assertIn("using {len(pallet_parts)} example pallet prims", source)
        self.assertIn("custom pallet creation is disabled", source)
        self.assertIn("IMAGE_MATCHED_SLIDING_STATION_USD", source)
        self.assertIn('"/home/mh/Downloads/harim_v3.usd"', source)
        self.assertIn("IMAGE_MATCHED_SLIDING_STATION_PRIM", source)
        self.assertIn("place_image_matched_sliding_station", source)
        self.assertIn("place_reversed_drop_area", source)
        self.assertIn("reversed_static_root_transform", source)
        self.assertIn("PickupSlideStation", source)
        self.assertIn("DropSlideStation", source)
        self.assertIn("DropUr10Table", source)
        self.assertIn("PICKUP_STATION_LEFT_Y = -0.19", source)
        self.assertIn("PICKUP_STATION_RIGHT_Y = 0.24", source)
        self.assertIn("set_slide_station_side_y_offsets", source)
        self.assertIn("kept DropSlideStation source side spacing", source)
        self.assertIn("DROP_UR10_FOLDED_HOME_JOINT_TARGETS_DEG", source)
        self.assertIn("set_ur10_folded_home_joint_targets", source)
        self.assertIn('f"{ur10_root_path}/joints/{joint_name}"', source)
        self.assertIn("set_ur10_folded_home_visual_pose", source)
        self.assertIn("compute_ur10_folded_home_link_matrices", source)
        self.assertIn("make_visual_only_static", source)
        self.assertIn("align_prim_bottom_to_target_z(stage, drop_table_root, REFERENCE_STATION_FLOOR_Z)", source)
        self.assertIn("surfacegripper", source)
        self.assertIn("ClearXformOpOrder", source)
        self.assertIn("AddTransformOp", source)
        self.assertIn("CreateJointEnabledAttr(False)", source)
        self.assertIn("CreateJointEnabledAttr(True)", source)
        self.assertIn("CreateRigidBodyEnabledAttr(False)", source)
        self.assertIn("CreateKinematicEnabledAttr(False)", source)
        self.assertIn("JointStateAPI.Apply", source)
        self.assertIn("DEFAULT_PALLET_X = 0.98", source)
        self.assertNotIn("build_slide_station_box_specs", source)
        self.assertNotIn("def create_slide_station", source)
        self.assertNotIn("IwHubLiftPlate", source)
        self.assertNotIn("PalletDeck", source)
        self.assertNotIn("PalletBlock", source)
        self.assertNotIn("BlueFloorRail", source)
        self.assertNotIn("BlueBasePlate", source)
        self.assertNotIn("WhiteSideFrame", source)
        self.assertNotIn("LevelingFoot", source)
        self.assertNotIn("NO RIDING", source)

    def test_v3_adds_separate_dropoff_pushcart_without_touching_source_robot_flow(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn('V3_PUSHCART_SOURCE_NAME = "SM_PushcartA_02_22"', source)
        self.assertIn('V3_PUSHCART_TARGET_ROOT = "/World/V3PushcartA_02_22"', source)
        self.assertIn('V3_PUSHCART_PALLET_ROOT = "/World/V3PushcartPallet"', source)
        self.assertIn("V3_PUSHCART_WORLD_Y = -1.7", source)
        self.assertIn("V3_PUSHCART_YAW_DEG = 90.0", source)
        self.assertIn("V3_PUSHCART_PALLET_WORLD_X = 12.56", source)
        self.assertIn("V3_PUSHCART_PALLET_TRANSLATE_Z = -0.831", source)
        self.assertIn("V3_AMR_DROP_STATION_BACKOFF = 0.08", source)
        self.assertIn("dz = 0.135", source)
        self.assertIn("V3_PUSHCART_PALLET_Z_OFFSET = -0.0079", source)
        self.assertIn("place_v3_dropoff_pushcart_target", source)
        self.assertIn("CopyPrimCommand", source)
        self.assertIn("copy_prim_for_v3", source)
        self.assertIn("find_first_prim_path_by_name", source)
        self.assertIn("def set_prim_translate_rotate_z_scale", source)
        self.assertIn("xformable.SetXformOpOrder([translate_op, rotate_z_op, scale_op])", source)
        self.assertIn("set_prim_translate_rotate_z_scale(", source)
        self.assertIn("pushcart_position[1] = V3_PUSHCART_WORLD_Y", source)
        self.assertIn("pallet_center_xy[0] = V3_PUSHCART_PALLET_WORLD_X", source)
        self.assertIn("target_bottom_z=cart_max[2] + V3_PUSHCART_PALLET_Z_OFFSET", source)
        self.assertIn("align_prim_bottom_to_target_z(stage, V3_PUSHCART_TARGET_ROOT, REFERENCE_STATION_FLOOR_Z)", source)
        self.assertIn("class V3DropRobotTransferController", source)
        self.assertIn("started V3 drop robot transfer after AMR exit", source)
        self.assertIn("order=reverse_loaded", source)
        self.assertIn("V3 drop robot moving box", source)
        self.assertIn("V3 drop robot placed box", source)
        self.assertIn("completed V3 drop robot transfer to pushcart pallet", source)
        self.assertIn("dropped_item_sequence", source)
        self.assertIn("for item, position, orientation in reversed(dropped_sequence):", source)
        self.assertIn("mapped V3 drop robot targets from delivered pallet layout", source)
        self.assertIn("target_y=mirrored_for_drop_robot", source)
        self.assertIn("target_stack=bottom_first", source)
        self.assertIn("item_bottom_offset", source)
        self.assertIn("target_bottom_z - data[\"item_bottom_offset\"]", source)
        self.assertIn("V3_DROP_ROBOT_PICK_APPROACH_DISTANCE = 0.10", source)
        self.assertIn("V3_DROP_ROBOT_PICK_LIFT_DISTANCE = 0.30", source)
        self.assertIn("V3_DROP_ROBOT_PLACE_APPROACH_DISTANCE = 0.35", source)
        self.assertIn("V3_DROP_ROBOT_PLACE_LIFT_DISTANCE = 0.10", source)
        self.assertIn("V3_DROP_ROBOT_PLACE_RELEASE_CLEARANCE = 0.006", source)
        self.assertIn("V3_DROP_ROBOT_IK_ROTATION_TOLERANCE = 0.18", source)
        self.assertIn("V3_DROP_ROBOT_IK_ROTATION_WEIGHT = 0.22", source)
        self.assertIn('V3_DROP_ROBOT_GRASP_COLLISION_RELATIVE_PATH = "Collision/Cube_03"', source)
        self.assertIn("V3_DROP_ROBOT_ORIGINAL_UPSIDE_DOWN_MARGIN = -0.0025", source)
        self.assertIn("orthonormal_rotation_from_axes", source)
        self.assertIn("original_place_target_rotation", source)
        self.assertIn("rotation_error_vector", source)
        self.assertIn("transform_world_rotation_to_prim_local", source)
        self.assertIn("solve_ur10_pose_ik", source)
        self.assertIn("_set_original_grasp_reference", source)
        self.assertIn('collision_path = f"{prim_path}/{V3_DROP_ROBOT_GRASP_COLLISION_RELATIVE_PATH}"', source)
        self.assertIn('data["grasp_base_root_offset"]', source)
        self.assertIn('data["pick_target_rotation"]', source)
        self.assertIn('data["place_target_rotation"]', source)
        self.assertIn("_original_grasp_tcp_world", source)
        self.assertIn('"original_collision_cube"', source)
        self.assertIn("_prepare_item_arm_offsets", source)
        self.assertIn('data["arm_prepared"]', source)
        self.assertIn('data["motion_keyframes"]', source)
        self.assertIn("interpolate_joint_targets_catmull_rom", source)
        self.assertIn("catmull_rom_scalar", source)
        self.assertIn("targets[joint_name] = clamp(value, min(start_value, end_value), max(start_value, end_value))", source)
        self.assertIn("V3 drop robot preparing box", source)
        self.assertIn("process=pose_ik_reach_wait_close_lift_then_reach_wait_open_lift", source)
        self.assertIn('get_prim_world_xform_components(self.stage, f"{self.ur10_root_path}/ee_link")', source)
        self.assertIn("_item_position_from_drop_robot", source)
        self.assertIn('data["place_release_position"]', source)
        self.assertIn('position = lerp(position, data["place_release_position"], settle_t)', source)
        self.assertIn('position[2] = max(float(position[2]), float(data["target_position"][2]))', source)
        self.assertIn("transform_world_point_to_prim_local(self.stage, self.ur10_root_path, world_position)", source)
        self.assertIn('data["pick_approach_joints"]', source)
        self.assertIn('data["pick_joints"]', source)
        self.assertIn('data["pick_lift_joints"]', source)
        self.assertIn('data["place_approach_joints"]', source)
        self.assertIn('data["place_joints"]', source)
        self.assertIn('data["place_lift_joints"]', source)
        self.assertIn("prepared V3 drop robot grasp target", source)
        self.assertIn("V3 drop robot closed gripper on box", source)
        self.assertIn("V3 drop robot opened gripper on box", source)
        self.assertIn("can_start_from_reset_gate", source)
        self.assertIn("orchestrator.state == TransferState.RESET_CYCLE", source)
        self.assertIn("not v3_drop_robot_controller.done", source)
        self.assertIn("def reset_for_next_cycle(self):", source)
        self.assertIn("v3_drop_robot_controller.reset_for_next_cycle()", source)
        self.assertIn("styled_cardboard_box_paths.clear()", source)
        controller_source = source[source.index("class V3DropRobotTransferController"):]
        self.assertNotIn("def _prepare_arm_offsets", controller_source)
        self.assertNotIn("self._prepare_arm_offsets()", controller_source)
        self.assertNotIn("arc_lerp_position(", source[source.index("class V3DropRobotTransferController"):])
        self.assertIn("reversed_amr_drop_pose_from_pickup", source)
        self.assertIn("backoff_amr_drop_pose_toward_reversed_pickup", source)
        self.assertIn("drop_pose_override=amr_drop_pose", source)
        self.assertIn("matched AMR pallet drop pose toward reversed pickup depth", source)
        self.assertIn("source_pallet_path=str(pallet_parts[0].prim.GetPath())", source)
        self.assertIn("v3_drop_robot_controller.step(physics_dt, orchestrator)", source)
        self.assertIn("task = BinStackingTask(\"/World/Ur10Table\", ur10_assets)", source)
        self.assertNotIn("task = BinStackingTask(\"/World/HarimDemo/DropUr10Table\"", source)

    def test_amr_lift_is_calibrated_to_pallet_underside_without_touching_source_robot(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("lift_surface_z=pallet_bottom_z", source)
        self.assertIn("self.amr_lift_top_offset", source)
        self.assertIn("calibrated iw_hub lift top to pallet underside", source)
        self.assertIn("target[2] = self.lift_surface_z - self.amr_lift_top_offset + self.lift_offset", source)
        self.assertIn('task = BinStackingTask("/World/Ur10Table", ur10_assets)', source)

    def test_reversed_static_root_transform_aligns_table_by_pallet_anchors(self):
        source_table = np.array([0.0, 0.0, 0.0])
        source_pallet = np.array([0.98, -0.32810499266628176, 0.0])
        target_pallet = np.array([11.58, -0.32810499266628176, 0.0])

        position, orientation, scale = self.demo.reversed_static_root_transform(
            source_position=source_table,
            source_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            source_scale=np.array([1.0, 1.0, 1.0]),
            source_anchor=source_pallet,
            target_anchor=target_pallet,
        )

        np.testing.assert_allclose(position, np.array([12.56, -0.6562099853325635, 0.0]))
        np.testing.assert_allclose(orientation, np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-12)
        np.testing.assert_allclose(scale, np.ones(3))

    def test_reversed_amr_drop_pose_matches_pickup_depth_from_pallet(self):
        pickup_pose = np.array([1.25, -0.31, -1.18])
        source_pallet = np.array([0.98, -0.32810499266628176, -0.8168284034659473])
        target_pallet = np.array([11.58, -0.32810499266628176, -0.8168284034659473])

        drop_pose = self.demo.reversed_amr_drop_pose_from_pickup(
            pickup_pose,
            source_pallet,
            target_pallet,
            -1.18,
        )

        np.testing.assert_allclose(drop_pose, np.array([11.31, -0.34620998533256353, -1.18]))

    def test_amr_drop_backoff_moves_pallet_drop_toward_reversed_pose(self):
        nominal_pose = np.array([11.85, -0.31, -1.18])
        reversed_pose = np.array([11.31, -0.34620998533256353, -1.18])

        drop_pose = self.demo.backoff_amr_drop_pose_toward_reversed_pickup(nominal_pose, reversed_pose, 0.08)

        self.assertLess(drop_pose[0], nominal_pose[0])
        self.assertGreater(drop_pose[0], reversed_pose[0])
        self.assertAlmostEqual(float(np.linalg.norm((drop_pose - nominal_pose)[:2])), 0.08)
        self.assertEqual(drop_pose[2], nominal_pose[2])

    def test_default_amr_transform_is_explicit_v2_value(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("DEFAULT_AMR_Z = -1.18", source)
        self.assertIn("DEFAULT_AMR_SCALE = np.array([0.75, 1.0, 1.35], dtype=float)", source)
        self.assertIn("set_prim_scale(usd_context.get_stage(), f\"{harim_root}/iw_hub\", DEFAULT_AMR_SCALE)", source)
        self.assertNotIn("DEFAULT_AMR_Z = None", source)
        self.assertNotIn("compute_floor_aligned_amr_z", source)

    def test_small_klt_visual_boxes_match_background_card_box_color(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn('BACKGROUND_CARD_BOX_PRIM_PATH = "/World/Background/SM_CardBoxA_02"', source)
        self.assertIn('BACKGROUND_CARD_BOX_NAME_PREFIX = "SM_CardBoxA_"', source)
        self.assertIn('SMALL_KLT_VISUAL_NAME_PREFIX = "SmallKLT_Visual_"', source)
        self.assertIn('KLT_MAGENTA_BOX_MESH_NAME = "FOF_Mesh_Magenta_Box"', source)
        self.assertIn("SMALL_KLT_VISUAL_INDEX_MIN = 2", source)
        self.assertIn("SMALL_KLT_VISUAL_INDEX_MAX = 164", source)
        self.assertIn('SMALL_KLT_VISUAL_ROOT_NAME = "Visuals"', source)
        self.assertIn("DYNAMIC_SMALL_KLT_VISUAL_ROOT_PREFIXES", source)
        self.assertIn("def is_cardboard_box_style_target_prim", source)
        self.assertIn("resolve_background_card_box_style", source)
        self.assertIn("apply_background_card_box_style_to_box_targets", source)
        self.assertIn("apply_box_style_to_all_gprims", source)
        self.assertIn("apply_pallet_box_color", source)
        self.assertIn("matched {applied_count} pallet visuals to cardboard box color", source)
        self.assertIn('if "/pallet" in path:', source)
        self.assertNotIn('apply_cardboard_box_color("/World")', source)
        self.assertIn('apply_cardboard_box_color("/World/Background")', source)
        self.assertIn('apply_cardboard_box_color("/World/Ur10Table/bins")', source)
        self.assertNotIn("pallet_material", source.lower())

    def test_box_color_change_preserves_original_bin_stacking_task_flow(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("def __init__(self, env_path, assets) -> None:", source)
        self.assertIn('task = BinStackingTask("/World/Ur10Table", ur10_assets)', source)
        self.assertIn("self._spawn_bin(self.on_conveyor)", source)
        self.assertNotIn("apply_box_color_fn", source)

    def test_demo_wrappers_forward_capture_path(self):
        project_root = DEMO_PATH.parents[2]
        sh_source = (project_root / "run_harim_demo.sh").read_text(encoding="utf-8")
        ps1_source = (project_root / "run_harim_demo.ps1").read_text(encoding="utf-8")

        self.assertIn("--capture-path PATH", sh_source)
        self.assertIn('ARGS+=("--capture-path" "$CAPTURE_PATH")', sh_source)
        self.assertIn("[string]$CapturePath", ps1_source)
        self.assertIn('"--capture-path"', ps1_source)

    def test_lift_up_moves_example_pallet_from_initial_pose(self):
        orchestrator, context, _world, _items = self.build_orchestrator(Args())
        pallet = orchestrator.pallet_parts[0]
        initial_z = pallet.get_world_pose()[0][2]
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.MOVE_TO_DROP)

        self.assertAlmostEqual(pallet.get_world_pose()[0][2], initial_z + Args.lift_height, places=6)

    def test_slide_out_keeps_dropped_payload_stationary(self):
        orchestrator, context, _world, items = self.build_orchestrator(Args())
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.SLIDE_OUT_FROM_PALLET)

        dropped_item_positions = {item.name: item.get_world_pose()[0] for item in items}
        dropped_pallet_positions = {part.name: part.get_world_pose()[0] for part in orchestrator.pallet_parts}
        amr_x_before = orchestrator.get_amr_position()[0]

        orchestrator.step(0.1)
        orchestrator.step(0.1)

        self.assertLess(orchestrator.get_amr_position()[0], amr_x_before)
        for item in items:
            np.testing.assert_allclose(item.get_world_pose()[0], dropped_item_positions[item.name])
        for part in orchestrator.pallet_parts:
            np.testing.assert_allclose(part.get_world_pose()[0], dropped_pallet_positions[part.name])

    def test_infinite_cycles_reset_for_next_stack(self):
        orchestrator, context, world, _items = self.build_orchestrator(RepeatArgs())
        context.stack_complete = True

        self.run_until(
            orchestrator,
            lambda: orchestrator.completed_cycles == 1
            and orchestrator.state == self.demo.TransferState.WAIT_STACK_COMPLETE,
        )

        self.assertEqual(world.reset_count, 1)
        self.assertEqual(world.play_count, 1)
        self.assertFalse(orchestrator.carrying)

    def test_actual_iw_hub_lift_prim_tracks_lift_offset(self):
        start_pose = np.array([Args.pickup_x + self.demo.AMR_START_STANDOFF, Args.pickup_y, Args.amr_z])
        lift_prim = FakePosePrim("asset_lift", start_pose + np.array([0.0, 0.0, 0.25]))
        initial_lift_z = lift_prim.get_world_pose()[0][2]
        orchestrator, context, _world, _items = self.build_orchestrator(Args(), amr_lift_prim=lift_prim)
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.MOVE_TO_DROP)

        lifted_z = lift_prim.get_world_pose()[0][2]
        self.assertAlmostEqual(lifted_z, initial_lift_z + Args.lift_height, places=6)

    def test_surface_gripper_extension_is_enabled_before_cortex_ur10_import(self):
        source = DEMO_PATH.read_text(encoding="utf-8")
        enable_index = source.index('enable_extension("isaacsim.robot.surface_gripper")')
        cortex_import_index = source.index("from isaacsim.cortex.framework.robot import CortexUr10")

        self.assertLess(enable_index, cortex_import_index)

    def test_demo_does_not_depend_on_interactive_or_anim_robot_extensions(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertNotIn("isaacsim.examples.interactive", source)
        self.assertNotIn('enable_extension("isaacsim.anim.robot")', source)

    def test_self_test_runs_fixed_frames_without_is_running_gate(self):
        source = DEMO_PATH.read_text(encoding="utf-8")
        self_test_index = source.index("if args.self_test_frames > 0:")
        is_running_index = source.index("while simulation_app.is_running():")

        self.assertLess(self_test_index, is_running_index)
        self.assertIn("for _frame_count in range(args.self_test_frames):", source)

    def test_navigation_obstacle_uses_cortex_math_helpers(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("import isaacsim.cortex.framework.math_util as cortex_math_util", source)
        self.assertIn("cortex_math_util.pack_R", source)
        self.assertNotIn("from isaacsim.core.utils import math as math_util", source)

    def test_wait_for_stage_loading_updates_until_no_pending_assets(self):
        app = FakeSimulationApp()
        usd_context = FakeUsdContext([(0, 0, 2), (0, 0, 1), (0, 0, 0)])

        self.demo.wait_for_stage_loading(app, usd_context, "test", max_updates=5)

        self.assertEqual(app.update_count, 3)

    def test_wait_for_stage_loading_times_out_with_pending_assets(self):
        app = FakeSimulationApp()
        usd_context = FakeUsdContext([(0, 0, 1)])

        with self.assertRaisesRegex(RuntimeError, "Timed out waiting for USD assets"):
            self.demo.wait_for_stage_loading(app, usd_context, "test", max_updates=2)


if __name__ == "__main__":
    unittest.main()
