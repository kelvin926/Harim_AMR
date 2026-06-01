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

    def test_demo_creates_rail_style_slide_stations(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("collect_example_pallet_parts", source)
        self.assertIn("using {len(pallet_parts)} example pallet prims", source)
        self.assertIn("custom pallet creation is disabled", source)
        self.assertIn("def create_slide_station", source)
        self.assertIn("PickupSlideStation", source)
        self.assertIn("DropSlideStation", source)

        pickup_specs = self.demo.slide_station_component_specs("/World/HarimDemo/PickupSlideStation", "Pickup", 0.0, 0.0)
        drop_specs = self.demo.slide_station_component_specs("/World/HarimDemo/DropSlideStation", "Drop", 0.0, 0.0)
        spec_names = {spec["name"] for spec in pickup_specs + drop_specs}
        self.assertIn("PickupSlideRailLeft", spec_names)
        self.assertIn("PickupSlideRailRight", spec_names)
        self.assertIn("DropSlideRailLeft", spec_names)
        self.assertIn("DropSlideRailRight", spec_names)

        rail_specs = [spec for spec in pickup_specs if "SlideRail" in spec["name"]]
        self.assertEqual(len(rail_specs), 2)
        self.assertGreater(self.demo.SLIDE_STATION_CLEARANCE_Y, 0.5)
        self.assertLess(max(spec["scale"][1] for spec in pickup_specs), self.demo.SLIDE_STATION_CLEARANCE_Y)
        self.assertGreater(max(spec["position"][1] for spec in rail_specs), 0.0)
        self.assertLess(min(spec["position"][1] for spec in rail_specs), 0.0)
        self.assertNotIn("PalletDeck", source)
        self.assertNotIn("PalletBlock", source)

    def test_box_material_helper_recolors_spawned_and_self_test_bins(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("def apply_box_material", source)
        self.assertIn("BOX_MATERIAL_PATH", source)
        self.assertIn("BOX_DISPLAY_COLOR", source)
        self.assertGreaterEqual(source.count("apply_box_material("), 3)
        self.assertIn("apply_box_material(self.stage, prim_path)", source)
        self.assertIn("apply_box_material(stage, payload_path)", source)

    def test_stack_z_base_is_derived_from_station_height(self):
        self.assertAlmostEqual(
            self.demo.make_stack_base_z(self.demo.PALLET_SUPPORT_TOP_Z),
            self.demo.DEFAULT_STACK_Z_BASE,
            places=6,
        )
        coords = self.demo.make_stack_coordinates(1, 1, 2)
        self.assertAlmostEqual(coords[0][2], self.demo.DEFAULT_STACK_Z_BASE, places=6)
        self.assertAlmostEqual(coords[1][2] - coords[0][2], self.demo.STACK_LAYER_SPACING_Z, places=6)

        source = DEMO_PATH.read_text(encoding="utf-8")
        self.assertIn("PALLET_SUPPORT_TOP_Z", source)
        self.assertIn("BOX_STACK_CENTER_ABOVE_SUPPORT", source)
        self.assertNotIn("z0 = -0.51", source)

    def test_amr_default_z_is_derived_from_loaded_bounds(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIsNone(self.demo.DEFAULT_AMR_Z)
        self.assertIn("AMR_GROUND_CLEARANCE", source)
        self.assertIn("WAREHOUSE_FLOOR_Z", source)
        self.assertIn("compute_amr_ground_aligned_z", source)
        self.assertIn("resolve_amr_z(stage", source)
        self.assertIn("UsdGeom.BBoxCache", source)
        self.assertNotIn("DEFAULT_AMR_Z = -1.05", source)

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
        orchestrator, context, _world, _items = self.build_orchestrator(Args(), amr_lift_prim=lift_prim)
        initial_lift_z = lift_prim.get_world_pose()[0][2]
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
