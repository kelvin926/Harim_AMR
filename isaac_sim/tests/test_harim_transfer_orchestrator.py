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
            lift_plate=FakePosePrim("lift_plate"),
            pallet_parts=[FakePosePrim(f"pallet_{idx}") for idx in range(len(self.demo.PALLET_PART_OFFSETS))],
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
        self.assertGreater(orchestrator.get_amr_position()[0], Args.drop_x)
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

    def test_default_amr_z_is_on_warehouse_floor(self):
        self.assertAlmostEqual(self.demo.DEFAULT_AMR_Z, self.demo.WORLD_FLOOR_Z)
        self.assertLess(self.demo.DEFAULT_AMR_Z, -1.15)

    def test_spawned_bins_are_upside_down_to_skip_flip_station(self):
        for _ in range(10):
            _position, orientation = self.demo.random_bin_spawn_transform()
            local_z_in_world = rotate_vector_by_quat([0.0, 0.0, 1.0], orientation)

            self.assertLess(local_z_in_world[2], -0.99)

    def test_demo_uses_no_flip_dispatch(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("class NoFlipDispatch", source)
        self.assertIn("make_no_flip_decider_network", source)
        self.assertIn("class DemoAttachBin", source)
        self.assertIn("class DemoReleaseBin", source)
        self.assertIn("class DemoPickAndPlaceBin", source)
        self.assertIn('label="reach_pick"', source)
        self.assertIn("hold_active_bin_for_pick", source)
        self.assertIn("class DemoSettleBinAtGripper", source)
        self.assertIn("place_active_bin_grasp_at_effector", source)
        self.assertIn("demo_pre_grip_initial_offset", source)
        self.assertIn("suction close skipped for fallback attach", source)
        self.assertIn("DemoSettleBinAtGripper(duration=0.20)", source)
        self.assertIn("set_kinematic_for_demo(active_bin.bin_obj, True)", source)
        self.assertIn("PICK_STATION_BIN_POSITION", source)
        self.assertIn("demo_pick_stationed", source)
        self.assertIn("if self.state is None", source)
        self.assertIn("class DemoWaitForNextBin", source)
        self.assertIn("sync_demo_attached_bin", source)
        self.assertIn("demo_carried_bin", source)
        self.assertIn("task.context = decider_network.context", source)
        self.assertIn("release_duration=0.35", source)
        self.assertIn("self.context.robot.suction_gripper.open()", source)
        self.assertIn('"wait_next_bin"', source)
        self.assertIn('return DfDecision("wait_next_bin")', source)
        self.assertIn("deactivate_stage_prims_containing", source)
        self.assertIn('"pallet_holder"', source)
        self.assertNotIn('add_child("flip_bin"', source)
        self.assertNotIn("behavior.make_decider_network", source)

    def test_normal_arm_self_test_can_require_placed_bins(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("--self-test-min-placed-bins", source)
        self.assertIn("UR10 placed", source)
        self.assertIn("expected at least", source)
        self.assertIn("[HarimDemo] self-test failed", source)
        self.assertIn("raise SystemExit(1)", source)

    def test_drop_slide_workstation_is_created(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("create_drop_slide_workstation", source)
        self.assertIn("DropSlideRail", source)
        self.assertIn("DropSlideRoller", source)
        self.assertIn("DropSlideLeg", source)
        self.assertIn("DropSlideTopSupport", source)

    def test_connected_pallet_uses_fixed_collision_support(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("FixedCuboid", source)
        self.assertIn("harim_pallet_connected_top_deck", source)
        self.assertIn("PalletRunner", source)
        self.assertIn("PalletTopSupport", source)

    def test_asset_lift_hides_floating_visual_lift_plate(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("AMR_LIFT_PLATE_OFFSET_Z", source)
        self.assertIn("visible=amr_lift is None", source)

    def test_pallet_layout_leaves_center_tunnel_for_under_ride(self):
        block_offsets = self.demo.PALLET_BLOCK_OFFSETS
        tunnel_half_width = self.demo.PALLET_TUNNEL_HALF_WIDTH

        self.assertGreater(tunnel_half_width, 0.0)
        for offset in block_offsets:
            self.assertGreaterEqual(abs(offset[1]), tunnel_half_width)

    def test_stack_is_locked_after_stack_complete(self):
        orchestrator, context, _world, items = self.build_orchestrator(Args())
        original_pose = items[0].get_world_pose()[0]
        context.stack_complete = True

        orchestrator.step(0.1)
        self.assertIn(items[0].name, orchestrator.locked_stack_poses)

        items[0].set_world_pose(position=np.array([4.0, 4.0, -4.0]))
        orchestrator.step(0.1)

        np.testing.assert_allclose(items[0].get_world_pose()[0], original_pose)
        np.testing.assert_allclose(items[0].linear_velocity, np.zeros(3))
        np.testing.assert_allclose(items[0].angular_velocity, np.zeros(3))

    def test_slide_out_keeps_dropped_payload_stationary(self):
        orchestrator, context, _world, items = self.build_orchestrator(Args())
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.SLIDE_OUT_FROM_PALLET)

        dropped_item_positions = {item.name: item.get_world_pose()[0] for item in items}
        dropped_pallet_positions = {part.name: part.get_world_pose()[0] for part in orchestrator.pallet_parts}
        amr_x_before = orchestrator.get_amr_position()[0]

        orchestrator.step(0.1)
        orchestrator.step(0.1)

        self.assertGreater(orchestrator.get_amr_position()[0], amr_x_before)
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
