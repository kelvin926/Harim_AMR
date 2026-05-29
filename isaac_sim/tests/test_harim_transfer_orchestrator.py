import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


DEMO_PATH = Path(__file__).resolve().parents[1] / "scripts" / "run_harim_pallet_demo.py"
RUNNER_PATH = Path(__file__).resolve().parents[2] / "run_harim_demo.ps1"
STRICT_RUNNER_PATH = Path(__file__).resolve().parents[2] / "run_harim_strict_self_test.ps1"


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
        self.visible = None

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

    def set_visibility(self, visible):
        self.visible = bool(visible)


class FakeLight:
    def __init__(self):
        self.visible = None

    def set_visibility(self, visible):
        self.visible = bool(visible)


class FakeCompletionSignal:
    def __init__(self):
        self.completed_values = []

    def set_completed(self, completed):
        self.completed_values.append(bool(completed))


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


class SlowArgs(Args):
    move_speed = 1.0


class HarimTransferOrchestratorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.demo = load_demo_module()

    def build_orchestrator(
        self,
        args,
        amr_lift_prim=None,
        completion_signal=None,
        pallet_parts=None,
        pallet_part_offsets=None,
        load_restraint_parts=None,
        amr_safety_parts=None,
        amr_safety_offsets=None,
        amr_safety_roles=None,
        amr_drive_parts=None,
        amr_drive_offsets=None,
        amr_lift_guide_parts=None,
        amr_lift_guide_offsets=None,
        camera_director=None,
    ):
        items = [FakePosePrim(f"bin_{idx}", (0.7 + 0.1 * idx, -0.31, -0.50)) for idx in range(3)]
        context = FakeContext(items)
        world = FakeWorld()
        if pallet_parts is None:
            pallet_parts = [FakePosePrim(f"pallet_{idx}") for idx in range(len(self.demo.PALLET_PART_OFFSETS))]
        orchestrator = self.demo.HarimTransferOrchestrator(
            world=world,
            context=context,
            task=None,
            amr_prim=FakePosePrim("iw_hub"),
            amr_lift_prim=amr_lift_prim,
            lift_plate=FakePosePrim("lift_plate"),
            amr_safety_parts=amr_safety_parts,
            amr_safety_offsets=amr_safety_offsets,
            amr_safety_roles=amr_safety_roles,
            amr_drive_parts=amr_drive_parts,
            amr_drive_offsets=amr_drive_offsets,
            amr_lift_guide_parts=amr_lift_guide_parts,
            amr_lift_guide_offsets=amr_lift_guide_offsets,
            pallet_parts=pallet_parts,
            pallet_part_offsets=pallet_part_offsets,
            load_restraint_parts=load_restraint_parts,
            stack_coordinates=self.demo.make_stack_coordinates(2, 2, 1),
            args=args,
            completion_signal=completion_signal,
            camera_director=camera_director,
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

    def test_arm_timing_constants_allow_visible_settle(self):
        self.assertGreaterEqual(self.demo.REACH_PICK_MAX_DURATION, 5.5)
        self.assertGreaterEqual(self.demo.REACH_PLACE_MAX_DURATION, 3.5)
        self.assertGreaterEqual(self.demo.RETURN_READY_DURATION, 5.0)
        self.assertLessEqual(self.demo.RETURN_READY_POSITION_THRESHOLD, 0.05)
        self.assertGreaterEqual(self.demo.POST_RELEASE_CLEARANCE_LIFT, 0.40)
        self.assertLessEqual(self.demo.POST_RELEASE_RETREAT_OFFSET[0], -0.20)
        self.assertGreaterEqual(self.demo.POST_RELEASE_RETREAT_OFFSET[2], 0.55)
        self.assertGreaterEqual(self.demo.RELEASE_RETREAT_DURATION, 0.80)
        self.assertGreaterEqual(self.demo.SCRIPTED_PLACE_DURATION, 0.60)
        self.assertGreaterEqual(self.demo.SCRIPTED_PLACE_EE_HOVER, 0.25)

    def test_completion_signal_controller_toggles_red_green(self):
        red = FakeLight()
        green = FakeLight()
        signal = self.demo.CompletionSignalController(red_light=red, green_light=green)

        signal.set_completed(False)
        self.assertTrue(red.visible)
        self.assertFalse(green.visible)

        signal.set_completed(True)
        self.assertFalse(red.visible)
        self.assertTrue(green.visible)

    def test_stack_complete_signal_turns_green_before_amr_approach(self):
        signal = FakeCompletionSignal()
        orchestrator, context, _world, _items = self.build_orchestrator(Args(), completion_signal=signal)
        context.stack_complete = True

        orchestrator.step(0.1)

        self.assertEqual(signal.completed_values[-1], True)
        self.assertEqual(orchestrator.state, self.demo.TransferState.ARM_SETTLE)

    def test_arm_settle_time_delays_amr_approach(self):
        orchestrator, context, _world, _items = self.build_orchestrator(Args())
        context.stack_complete = True

        orchestrator.step(0.1)
        orchestrator.step(self.demo.ARM_CLEAR_SETTLE_TIME - 0.1)
        self.assertEqual(orchestrator.state, self.demo.TransferState.ARM_SETTLE)

        orchestrator.step(0.2)
        self.assertEqual(orchestrator.state, self.demo.TransferState.MOVE_TO_APPROACH)

    def test_carton_visual_dimensions_are_box_like(self):
        body = self.demo.CARTON_BODY_SCALE
        tape = self.demo.CARTON_TAPE_TOP_SCALE
        label = self.demo.CARTON_SIDE_LABEL_SCALE
        stripe = self.demo.CARTON_SIDE_STRIPE_SCALE

        self.assertGreater(body[1], body[0])
        self.assertGreater(body[0], body[2])
        self.assertLess(tape[1], body[1])
        self.assertLess(tape[2], body[2])
        self.assertLess(label[0], body[0])
        self.assertLess(label[1], 0.01)
        self.assertLess(label[2], body[2])
        self.assertLess(stripe[0], label[0])
        self.assertGreater(stripe[2], label[2])
        self.assertGreater(self.demo.CARTON_TAPE_COLOR[0], self.demo.CARTON_TAPE_COLOR[1])
        self.assertGreater(self.demo.CARTON_LABEL_COLOR[0], self.demo.CARTON_BODY_COLOR[0])

    def test_floor_marking_dimensions_are_visual_only_and_thin(self):
        self.assertGreater(self.demo.WORK_ZONE_MARKING_SIZE[0], self.demo.PALLET_DECK_SCALE[0])
        self.assertGreater(self.demo.WORK_ZONE_MARKING_SIZE[1], self.demo.PALLET_DECK_SCALE[1])
        self.assertLessEqual(self.demo.FLOOR_MARKING_THICKNESS, 0.01)
        self.assertLessEqual(self.demo.WORK_ZONE_MARKING_EDGE_WIDTH, 0.08)
        self.assertGreaterEqual(self.demo.FLOOR_MARKING_Z, self.demo.WORLD_FLOOR_Z)
        self.assertGreater(self.demo.PICKUP_ZONE_MARKING_COLOR[0], self.demo.PICKUP_ZONE_MARKING_COLOR[2])
        self.assertGreater(self.demo.DROP_ZONE_MARKING_COLOR[2], self.demo.DROP_ZONE_MARKING_COLOR[0])

    def test_amr_route_guard_keeps_clear_loaded_path(self):
        metrics = self.demo.compute_amr_route_guard_metrics()
        specs = self.demo.make_amr_route_guard_specs()
        bollards = [spec for spec in specs if spec[1] == "bollard"]
        rails = [spec for spec in specs if spec[1] == "rail"]

        self.assertEqual(metrics["amr_route_guard_part_count"], len(specs))
        self.assertEqual(len(bollards), self.demo.AMR_ROUTE_GUARD_BOLLARD_COUNT_PER_SIDE * 2)
        self.assertEqual(len(rails), 2)
        self.assertGreaterEqual(metrics["amr_route_guard_part_count"], 14)
        self.assertGreaterEqual(metrics["amr_route_guard_span"], 8.0)
        self.assertGreaterEqual(metrics["amr_route_guard_clearance"], 0.30)
        self.assertGreaterEqual(metrics["amr_route_bollard_height"], 0.65)

    def test_loaded_route_clearance_accounts_for_lateral_error(self):
        centered_error = self.demo.compute_loaded_route_y_error(
            self.demo.DEFAULT_PICKUP_X,
            self.demo.DEFAULT_PICKUP_Y,
        )
        centered_clearance = self.demo.compute_loaded_route_guard_clearance(self.demo.DEFAULT_PICKUP_Y)
        shifted_clearance = self.demo.compute_loaded_route_guard_clearance(self.demo.DEFAULT_PICKUP_Y + 0.05)

        self.assertAlmostEqual(centered_error, 0.0)
        self.assertGreaterEqual(centered_clearance, 0.35)
        self.assertAlmostEqual(centered_clearance - shifted_clearance, 0.05)

    def test_stack_coordinate_clone_preserves_canonical_grid(self):
        original = self.demo.make_stack_coordinates(2, 2, 2)
        cloned = self.demo.clone_stack_coordinates(original)

        cloned[0][0] += 1.0

        self.assertNotAlmostEqual(original[0][0], cloned[0][0])

    def test_stack_geometry_has_close_non_overlapping_clearances(self):
        coordinates = self.demo.make_stack_coordinates(2, 2, 2)
        metrics = self.demo.compute_stack_geometry_metrics(coordinates, 2, 2, 2)

        self.assertGreaterEqual(metrics["min_stack_lateral_gap"], 0.0)
        self.assertLessEqual(metrics["max_stack_lateral_gap"], 0.03)
        self.assertGreaterEqual(metrics["min_stack_support_gap"], -0.005)
        self.assertLessEqual(metrics["max_stack_support_gap"], 0.02)
        np.testing.assert_allclose(coordinates[0], np.array([1.05, -0.62, -0.51]))

    def test_stack_footprint_stays_inside_pallet_deck(self):
        coordinates = self.demo.make_stack_coordinates(2, 2, 2)
        metrics = self.demo.compute_stack_pallet_footprint_metrics(
            coordinates,
            self.demo.DEFAULT_PICKUP_X,
            self.demo.DEFAULT_PICKUP_Y,
        )

        self.assertGreaterEqual(metrics["min_stack_pallet_margin"], 0.08)
        self.assertAlmostEqual(metrics["max_stack_pallet_overhang"], 0.0)

    def test_load_restraint_visuals_wrap_stack_inside_pallet_deck(self):
        coordinates = self.demo.make_stack_coordinates(2, 2, 2)
        specs = self.demo.compute_load_restraint_specs(
            coordinates,
            self.demo.DEFAULT_PICKUP_X,
            self.demo.DEFAULT_PICKUP_Y,
        )
        metrics = self.demo.compute_load_restraint_metrics(
            coordinates,
            self.demo.DEFAULT_PICKUP_X,
            self.demo.DEFAULT_PICKUP_Y,
        )

        self.assertEqual(len(specs), self.demo.LOAD_RESTRAINT_EXPECTED_PARTS)
        self.assertEqual(metrics["load_restraint_part_count"], self.demo.LOAD_RESTRAINT_EXPECTED_PARTS)
        self.assertGreaterEqual(metrics["min_load_restraint_pallet_margin"], 0.06)
        self.assertAlmostEqual(metrics["max_load_restraint_pallet_overhang"], 0.0)
        for _name, _offset, scale in specs:
            self.assertTrue(
                np.any(
                    np.isclose(
                        scale,
                        self.demo.LOAD_RESTRAINT_STRAP_THICKNESS,
                    )
                )
            )

    def test_load_restraints_are_hidden_until_stack_complete(self):
        load_restraints = [FakePosePrim("strap_0"), FakePosePrim("strap_1")]
        pallet_parts = [FakePosePrim(f"pallet_{idx}") for idx in range(len(self.demo.PALLET_PART_OFFSETS))]
        pallet_parts.extend(load_restraints)
        pallet_part_offsets = list(self.demo.PALLET_PART_OFFSETS)
        pallet_part_offsets.extend([np.array([0.0, 0.0, 0.20]), np.array([0.1, 0.0, 0.20])])
        orchestrator, context, _world, _items = self.build_orchestrator(
            Args(),
            pallet_parts=pallet_parts,
            pallet_part_offsets=pallet_part_offsets,
            load_restraint_parts=load_restraints,
        )

        for strap in load_restraints:
            self.assertFalse(strap.visible)

        context.stack_complete = True
        orchestrator.step(0.1)

        for strap in load_restraints:
            self.assertTrue(strap.visible)

    def test_infeed_conveyor_visual_spans_spawn_and_pick_points(self):
        metrics = self.demo.compute_infeed_conveyor_metrics()
        marker_specs = self.demo.make_infeed_motion_marker_specs()
        feed_carton_specs = self.demo.make_infeed_feed_carton_specs()

        self.assertGreaterEqual(metrics["infeed_conveyor_length"], 0.80)
        self.assertGreaterEqual(metrics["infeed_spawn_margin"], 0.30)
        self.assertGreaterEqual(metrics["infeed_pick_margin"], 0.20)
        self.assertGreaterEqual(metrics["infeed_guide_clearance"], 0.40)
        self.assertGreaterEqual(metrics["infeed_belt_support_gap"], 0.0)
        self.assertLessEqual(metrics["infeed_belt_support_gap"], 0.02)
        self.assertEqual(metrics["infeed_motion_marker_count"], 6)
        self.assertEqual(len(marker_specs), metrics["infeed_motion_marker_count"])
        self.assertGreaterEqual(metrics["infeed_motion_marker_spacing"], 0.10)
        self.assertGreaterEqual(metrics["infeed_motion_marker_speed"], 0.15)
        self.assertEqual(metrics["infeed_feed_carton_count"], 1)
        self.assertEqual(len(feed_carton_specs), metrics["infeed_feed_carton_count"])
        self.assertGreaterEqual(metrics["infeed_feed_carton_path_length"], 0.25)
        self.assertGreaterEqual(metrics["infeed_feed_carton_stop_clearance"], 0.05)
        self.assertGreaterEqual(metrics["infeed_feed_carton_guide_clearance"], 0.40)
        self.assertGreaterEqual(metrics["infeed_feed_carton_belt_support_gap"], 0.0)
        self.assertLessEqual(metrics["infeed_feed_carton_belt_support_gap"], 0.02)
        self.assertLess(self.demo.INFEED_CONVEYOR_START_Y, self.demo.PICK_STATION_BIN_POSITION[1])
        self.assertGreater(self.demo.INFEED_CONVEYOR_END_Y, self.demo.CONVEYOR_PICK_WINDOW_Y)

    def test_infeed_conveyor_motion_controller_moves_markers(self):
        marker_specs = self.demo.make_infeed_motion_marker_specs()
        marker_parts = [FakePosePrim(name) for name, _position, _scale, _color, _base_y in marker_specs]
        marker_base_y_values = [base_y for _name, _position, _scale, _color, base_y in marker_specs]
        controller = self.demo.InfeedConveyorMotionController(marker_parts, marker_base_y_values)

        controller.update(0.0)
        first_position = marker_parts[3].get_world_pose()[0].copy()
        controller.update(1.0)
        second_position = marker_parts[3].get_world_pose()[0]

        self.assertEqual(controller.update_count, 2)
        self.assertGreater(controller.max_marker_observed_travel, 0.10)
        self.assertLess(second_position[1], first_position[1])

    def test_infeed_feed_carton_controller_moves_upstream_cartons(self):
        carton_specs = self.demo.make_infeed_feed_carton_specs()
        body_parts = [FakePosePrim(f"{name}_body") for name, *_rest in carton_specs]
        tape_parts = [FakePosePrim(f"{name}_tape") for name, *_rest in carton_specs]
        base_y_values = [base_y for _name, _body_position, _tape_position, _body_scale, _tape_scale, base_y in carton_specs]
        controller = self.demo.InfeedFeedCartonMotionController(
            list(zip(body_parts, tape_parts)),
            base_y_values,
        )

        controller.update(0.0)
        first_body_position = body_parts[0].get_world_pose()[0].copy()
        first_tape_position = tape_parts[0].get_world_pose()[0].copy()
        controller.update(1.0)
        second_body_position = body_parts[0].get_world_pose()[0]
        second_tape_position = tape_parts[0].get_world_pose()[0]

        self.assertEqual(controller.update_count, 2)
        self.assertGreater(controller.max_carton_observed_travel, 0.05)
        self.assertLess(second_body_position[1], first_body_position[1])
        np.testing.assert_allclose(
            first_tape_position - first_body_position,
            np.array([0.0, 0.0, self.demo.INFEED_FEED_CARTON_TAPE_OFFSET_Z], dtype=float),
        )
        np.testing.assert_allclose(
            second_tape_position - second_body_position,
            np.array([0.0, 0.0, self.demo.INFEED_FEED_CARTON_TAPE_OFFSET_Z], dtype=float),
        )

    def test_safety_fence_leaves_amr_and_infeed_gate_clearance(self):
        metrics = self.demo.compute_safety_fence_metrics()
        specs = self.demo.make_safety_fence_specs()

        self.assertGreaterEqual(metrics["safety_fence_part_count"], 20)
        self.assertEqual(metrics["safety_fence_part_count"], len(specs))
        self.assertGreaterEqual(metrics["safety_fence_amr_gate_clearance"], 0.25)
        self.assertGreaterEqual(metrics["safety_fence_infeed_gate_clearance"], 0.20)
        self.assertLess(self.demo.SAFETY_FENCE_MIN_X, self.demo.PICK_STATION_BIN_POSITION[0])
        self.assertGreater(self.demo.SAFETY_FENCE_MAX_X, self.demo.DEFAULT_PICKUP_X)

    def test_amr_cell_gate_clearance_accounts_for_lateral_error(self):
        centered = self.demo.compute_amr_cell_gate_clearance(Args.pickup_y, Args.pickup_y)
        shifted = self.demo.compute_amr_cell_gate_clearance(Args.pickup_y + 0.02, Args.pickup_y)

        self.assertAlmostEqual(centered, self.demo.compute_safety_fence_metrics()["safety_fence_amr_gate_clearance"])
        self.assertAlmostEqual(centered - shifted, 0.04)
        self.assertGreaterEqual(shifted, 0.25)

    def test_amr_safety_visuals_have_beacon_and_scanner_clearance(self):
        metrics = self.demo.compute_amr_safety_visual_metrics()
        specs = self.demo.make_amr_safety_visual_specs()

        self.assertEqual(metrics["amr_safety_part_count"], len(specs))
        self.assertGreaterEqual(metrics["amr_safety_part_count"], 8)
        self.assertGreaterEqual(metrics["amr_safety_beacon_height"], 0.60)
        self.assertGreaterEqual(metrics["amr_safety_scanner_clearance"], 0.10)
        self.assertEqual(metrics["amr_warning_indicator_count"], 3)
        self.assertEqual(metrics["amr_idle_indicator_count"], 2)

    def test_amr_drive_visuals_have_floor_contact_and_stable_track(self):
        metrics = self.demo.compute_amr_drive_visual_metrics()
        specs = self.demo.make_amr_drive_visual_specs()

        self.assertEqual(metrics["amr_drive_part_count"], len(specs))
        self.assertGreaterEqual(metrics["amr_drive_part_count"], 6)
        self.assertGreaterEqual(metrics["amr_wheel_count"], 6)
        self.assertLessEqual(metrics["amr_wheel_floor_gap"], 0.01)
        self.assertLessEqual(metrics["amr_wheel_floor_penetration"], 0.005)
        self.assertGreaterEqual(metrics["amr_drive_wheelbase"], 1.0)
        self.assertGreaterEqual(metrics["amr_drive_track_width"], 0.90)

    def test_amr_lift_guides_connect_forks_to_chassis_travel(self):
        metrics = self.demo.compute_amr_lift_guide_visual_metrics()
        specs = self.demo.make_amr_lift_guide_visual_specs()

        self.assertEqual(metrics["amr_lift_guide_count"], len(specs))
        self.assertGreaterEqual(metrics["amr_lift_guide_count"], 4)
        self.assertLessEqual(metrics["amr_lift_guide_bottom_gap"], 0.10)
        self.assertLessEqual(metrics["amr_lift_guide_bottom_penetration"], 0.005)
        self.assertGreaterEqual(metrics["amr_lift_guide_travel_cover"], 0.02)
        self.assertGreaterEqual(metrics["amr_lift_guide_min_height"], 0.50)

    def test_amr_lift_guides_follow_amr_pose(self):
        specs = self.demo.make_amr_lift_guide_visual_specs()
        parts = [FakePosePrim(name) for name, _offset, _scale, _color in specs]
        offsets = [offset for _name, offset, _scale, _color in specs]
        orchestrator, _context, _world, _items = self.build_orchestrator(
            Args(),
            amr_lift_guide_parts=parts,
            amr_lift_guide_offsets=offsets,
        )

        target = np.array([Args.pickup_x + 0.5, Args.pickup_y + 0.02, Args.amr_z], dtype=float)
        orchestrator.set_amr_pose(target)

        self.assertAlmostEqual(orchestrator.max_amr_lift_guide_pose_error, 0.0)
        for part, offset in zip(parts, offsets):
            np.testing.assert_allclose(part.position, target + offset)

    def test_camera_rig_has_required_story_cuts(self):
        metrics = self.demo.compute_camera_rig_metrics()
        specs = self.demo.make_camera_rig_specs()
        roles = {role for _name, role, _eye, _target, _focal_length in specs}
        director_metrics = self.demo.compute_camera_director_metrics()

        self.assertEqual(metrics["camera_rig_count"], len(specs))
        self.assertGreaterEqual(metrics["camera_rig_count"], 4)
        self.assertTrue(set(self.demo.CAMERA_RIG_REQUIRED_ROLES).issubset(roles))
        self.assertGreaterEqual(metrics["camera_required_role_count"], 4)
        self.assertGreaterEqual(metrics["camera_min_height"], 1.25)
        self.assertGreaterEqual(metrics["camera_min_target_distance"], 1.0)
        self.assertEqual(self.demo.camera_role_for_transfer_state(self.demo.TransferState.WAIT_STACK_COMPLETE), "palletizer")
        self.assertEqual(self.demo.camera_role_for_transfer_state(self.demo.TransferState.MOVE_TO_APPROACH), "amr_route")
        self.assertEqual(self.demo.camera_role_for_transfer_state(self.demo.TransferState.LIFT_DOWN), "drop_dock")
        self.assertGreaterEqual(director_metrics["camera_director_role_count"], 4)
        self.assertGreaterEqual(director_metrics["camera_director_planned_switch_count"], 4)

    def test_warehouse_lighting_covers_work_cells_and_route(self):
        metrics = self.demo.compute_warehouse_lighting_metrics()
        specs = self.demo.make_warehouse_light_specs()
        roles = {role for _name, role, _position, _fixture_scale, _intensity in specs}

        self.assertEqual(metrics["warehouse_light_count"], len(specs))
        self.assertGreaterEqual(metrics["warehouse_light_count"], 4)
        self.assertTrue(set(self.demo.WAREHOUSE_LIGHT_REQUIRED_ROLES).issubset(roles))
        self.assertGreaterEqual(metrics["warehouse_light_role_count"], 3)
        self.assertGreaterEqual(metrics["warehouse_light_min_height"], 3.2)
        self.assertGreaterEqual(metrics["warehouse_light_route_span"], 8.0)
        self.assertGreaterEqual(metrics["warehouse_light_min_intensity"], 3000.0)

    def test_lift_plate_sits_just_below_pallet_underside(self):
        contact_gap = self.demo.compute_lift_contact_gap(self.demo.DEFAULT_AMR_Z)
        lift_top_z = (
            self.demo.DEFAULT_AMR_Z
            + self.demo.AMR_LIFT_PLATE_OFFSET_Z
            + self.demo.LIFT_FORK_SCALE[2] * 0.5
        )

        self.assertGreaterEqual(contact_gap, 0.0)
        self.assertLessEqual(contact_gap, 0.01)
        self.assertAlmostEqual(lift_top_z, self.demo.PALLET_DECK_UNDERSIDE_Z - contact_gap)

    def test_lift_plate_fits_inside_pallet_center_tunnel(self):
        clearance = self.demo.compute_pallet_tunnel_clearance()

        self.assertGreaterEqual(clearance, 0.10)
        self.assertLess(self.demo.compute_lift_fork_outer_half_width(), self.demo.PALLET_TUNNEL_HALF_WIDTH)

    def test_lift_visual_uses_two_separated_forks(self):
        self.assertEqual(len(self.demo.LIFT_FORK_OFFSETS), 2)
        self.assertLess(self.demo.LIFT_FORK_SCALE[1], 0.20)
        self.assertGreaterEqual(self.demo.compute_lift_fork_inner_gap(), 0.30)

    def test_drop_slide_lanes_support_pallet_without_runner_overlap(self):
        support_gap = self.demo.compute_drop_workstation_support_gap()
        lane_clearance = self.demo.compute_drop_workstation_tunnel_clearance()
        runner_clearance = self.demo.compute_drop_workstation_runner_clearance()
        fork_clearance = self.demo.compute_drop_workstation_fork_clearance()

        self.assertGreaterEqual(support_gap, 0.0)
        self.assertLessEqual(support_gap, 0.01)
        self.assertGreaterEqual(lane_clearance, 0.03)
        self.assertGreaterEqual(runner_clearance, 0.05)
        self.assertGreaterEqual(fork_clearance, 0.03)
        self.assertLess(
            self.demo.compute_drop_workstation_lane_outer_half_width(),
            self.demo.PALLET_TUNNEL_HALF_WIDTH,
        )
        self.assertGreater(
            abs(self.demo.DROP_SLIDE_LANE_Y_OFFSETS[0]),
            abs(self.demo.LIFT_FORK_OFFSETS[0][1]),
        )

    def test_drop_handoff_records_delivered_pallet_on_workstation_support(self):
        orchestrator, context, _world, _items = self.build_orchestrator(Args())
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.SLIDE_OUT_FROM_PALLET)

        self.assertAlmostEqual(orchestrator.drop_handoff_xy_error, 0.0)
        self.assertAlmostEqual(
            orchestrator.drop_handoff_support_gap,
            self.demo.compute_drop_workstation_support_gap(),
        )
        self.assertAlmostEqual(orchestrator.drop_handoff_support_penetration, 0.0)

    def test_pickup_handoff_records_amr_centered_under_pallet_before_lift(self):
        default_height_args = type("DefaultHeightArgs", (Args,), {"amr_z": self.demo.DEFAULT_AMR_Z})
        orchestrator, context, _world, _items = self.build_orchestrator(default_height_args)
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.LIFT_UP)

        self.assertEqual(orchestrator.pickup_handoff_count, 1)
        self.assertAlmostEqual(orchestrator.max_pickup_handoff_xy_error, 0.0)
        self.assertAlmostEqual(
            orchestrator.max_pickup_handoff_lift_gap,
            self.demo.compute_lift_contact_gap(default_height_args.amr_z, 0.0),
        )
        self.assertAlmostEqual(orchestrator.max_pickup_handoff_lift_penetration, 0.0)

    def test_pickup_entry_records_clearance_while_moving_under_pallet(self):
        default_height_args = type("DefaultHeightArgs", (Args,), {"amr_z": self.demo.DEFAULT_AMR_Z})
        orchestrator, context, _world, _items = self.build_orchestrator(default_height_args)
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.LIFT_UP)

        self.assertGreater(orchestrator.pickup_entry_sample_count, 0)
        self.assertAlmostEqual(orchestrator.max_pickup_entry_y_error, 0.0)
        self.assertAlmostEqual(
            orchestrator.min_pickup_entry_tunnel_clearance,
            self.demo.compute_pallet_tunnel_clearance(),
        )
        self.assertAlmostEqual(
            orchestrator.max_pickup_entry_lift_gap,
            self.demo.compute_lift_contact_gap(default_height_args.amr_z, 0.0),
        )
        self.assertAlmostEqual(orchestrator.max_pickup_entry_lift_penetration, 0.0)

    def test_pickup_entry_tunnel_clearance_accounts_for_lateral_drift(self):
        default_height_args = type("DefaultHeightArgs", (Args,), {"amr_z": self.demo.DEFAULT_AMR_Z})
        orchestrator, _context, _world, _items = self.build_orchestrator(default_height_args)

        orchestrator.set_amr_pose([default_height_args.pickup_x, default_height_args.pickup_y + 0.02, default_height_args.amr_z])
        orchestrator._set_lift_plate_pose()
        orchestrator._record_pickup_entry_geometry()

        self.assertEqual(orchestrator.pickup_entry_sample_count, 1)
        self.assertAlmostEqual(orchestrator.max_pickup_entry_y_error, 0.02)
        self.assertAlmostEqual(
            orchestrator.min_pickup_entry_tunnel_clearance,
            self.demo.compute_pallet_tunnel_clearance() - 0.02,
        )
        self.assertAlmostEqual(orchestrator.max_pickup_entry_lift_penetration, 0.0)

    def test_slide_out_records_fork_clearance_while_exiting_dropped_pallet(self):
        default_height_args = type("DefaultHeightArgs", (Args,), {"amr_z": self.demo.DEFAULT_AMR_Z})
        orchestrator, context, _world, _items = self.build_orchestrator(default_height_args)
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.RESET_CYCLE)

        self.assertGreater(orchestrator.slide_out_sample_count, 0)
        self.assertAlmostEqual(orchestrator.max_slide_out_y_error, 0.0)
        self.assertAlmostEqual(
            orchestrator.max_slide_out_lift_gap,
            self.demo.compute_lift_contact_gap(default_height_args.amr_z, 0.0),
        )
        self.assertAlmostEqual(orchestrator.max_slide_out_lift_penetration, 0.0)

    def test_dropped_stack_geometry_records_actual_payload_on_dropped_pallet(self):
        orchestrator, context, _world, items = self.build_orchestrator(Args())
        for item, coord in zip(items, orchestrator.stack_coordinates):
            item.set_world_pose(position=coord)
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.SLIDE_OUT_FROM_PALLET)

        self.assertEqual(orchestrator.dropped_stack_item_count, len(items))
        self.assertAlmostEqual(orchestrator.max_dropped_stack_pose_error, 0.0)
        self.assertLessEqual(orchestrator.max_dropped_stack_support_gap, 0.02)
        self.assertGreaterEqual(orchestrator.min_dropped_stack_support_gap, -0.005)
        self.assertGreaterEqual(orchestrator.min_dropped_stack_pallet_margin, 0.08)
        self.assertAlmostEqual(orchestrator.max_dropped_stack_pallet_overhang, 0.0)

    def test_dropped_pallet_geometry_records_connected_part_offsets(self):
        orchestrator, context, _world, _items = self.build_orchestrator(Args())
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.SLIDE_OUT_FROM_PALLET)

        self.assertEqual(orchestrator.dropped_pallet_part_count, len(self.demo.PALLET_PART_OFFSETS))
        self.assertAlmostEqual(orchestrator.max_dropped_pallet_part_pose_error, 0.0)

        orchestrator.pallet_parts[-1].set_world_pose(
            position=orchestrator.pallet_parts[-1].get_world_pose()[0] + np.array([0.03, 0.0, 0.0])
        )
        orchestrator._record_dropped_pallet_geometry()

        self.assertGreater(orchestrator.max_dropped_pallet_part_pose_error, 0.029)

    def test_drop_dock_stops_and_locator_posts_leave_clearance(self):
        metrics = self.demo.compute_drop_dock_metrics()

        self.assertEqual(metrics["drop_dock_stop_count"], 2)
        self.assertGreaterEqual(metrics["drop_dock_stop_gap"], 0.0)
        self.assertLessEqual(metrics["drop_dock_stop_gap"], 0.05)
        self.assertGreaterEqual(metrics["drop_dock_guide_clearance"], 0.10)
        self.assertGreaterEqual(metrics["drop_dock_fork_clearance"], 0.03)
        self.assertGreaterEqual(metrics["drop_dock_runner_clearance"], 0.05)

    def test_pickup_dock_stops_leave_amr_entry_clearance(self):
        metrics = self.demo.compute_pickup_dock_metrics()
        specs = self.demo.make_pickup_dock_alignment_specs()
        stop_parts = [spec for spec in specs if spec[1] == "stop"]
        post_parts = [spec for spec in specs if spec[1] == "post"]
        cap_parts = [spec for spec in specs if spec[1] == "cap"]

        self.assertEqual(metrics["pickup_dock_stop_count"], 2)
        self.assertEqual(len(stop_parts), 2)
        self.assertEqual(len(post_parts), 4)
        self.assertEqual(len(cap_parts), 4)
        self.assertGreaterEqual(metrics["pickup_dock_stop_gap"], 0.0)
        self.assertLessEqual(metrics["pickup_dock_stop_gap"], 0.05)
        self.assertGreaterEqual(metrics["pickup_dock_guide_clearance"], 0.10)
        self.assertGreaterEqual(metrics["pickup_dock_fork_clearance"], 0.03)
        self.assertGreaterEqual(metrics["pickup_dock_runner_clearance"], 0.05)
        self.assertLess(self.demo.PICKUP_DOCK_STOP_X_OFFSET, 0.0)

    def test_review_gif_layout_has_readable_panel(self):
        map_rect, panel_rect = self.demo.compute_review_gif_layout()

        self.assertGreaterEqual(self.demo.GIF_CANVAS_SIZE[0], 900)
        self.assertGreaterEqual(self.demo.GIF_CANVAS_SIZE[1], 500)
        self.assertEqual(panel_rect[0] - map_rect[2], 24)
        self.assertGreaterEqual(map_rect[2] - map_rect[0], 520)
        self.assertGreaterEqual(panel_rect[2] - panel_rect[0], 280)
        self.assertGreaterEqual(panel_rect[3] - panel_rect[1], 430)

    def test_review_gif_recorder_exports_per_run_gif(self):
        orchestrator, context, _world, _items = self.build_orchestrator(Args())
        with tempfile.TemporaryDirectory() as tmp_dir:
            recorder = self.demo.DemoGifRecorder(
                enabled=True,
                output_dir=tmp_dir,
                frame_stride=1,
                max_frames=3,
            )
            recorder.maybe_capture(0, orchestrator, context, Args())
            recorder.maybe_capture(1, orchestrator, context, Args())
            output_path = Path(recorder.save())

            self.assertEqual(output_path.suffix, ".gif")
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)
            from PIL import Image

            with Image.open(output_path) as image:
                self.assertEqual(image.size, self.demo.GIF_CANVAS_SIZE)
            latest_path = Path(tmp_dir) / self.demo.LATEST_REVIEW_GIF_NAME
            self.assertTrue(latest_path.exists())
            self.assertEqual(recorder.latest_path, str(latest_path))
            self.assertGreater(latest_path.stat().st_size, 0)

    def test_review_gif_recorder_saves_fallback_when_no_frames_exist(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            recorder = self.demo.DemoGifRecorder(
                enabled=True,
                output_dir=tmp_dir,
                frame_stride=1,
                max_frames=3,
            )
            output_path = Path(recorder.save())

            self.assertEqual(output_path.suffix, ".gif")
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)
            self.assertTrue((Path(tmp_dir) / self.demo.LATEST_REVIEW_GIF_NAME).exists())

    def test_review_gif_recorder_saves_fallback_after_capture_failure(self):
        orchestrator, context, _world, _items = self.build_orchestrator(Args())
        with tempfile.TemporaryDirectory() as tmp_dir:
            recorder = self.demo.DemoGifRecorder(
                enabled=True,
                output_dir=tmp_dir,
                frame_stride=1,
                max_frames=3,
            )

            def fail_draw_frame(_frame_index, _orchestrator, _context, _args):
                raise RuntimeError("forced capture failure")

            recorder._draw_frame = fail_draw_frame
            recorder.maybe_capture(0, orchestrator, context, Args())
            output_path = Path(recorder.save())

            self.assertFalse(recorder.enabled)
            self.assertEqual(output_path.suffix, ".gif")
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)
            self.assertIn("forced capture failure", recorder.disabled_reason)

    def test_slide_exit_clears_dropped_pallet_footprint(self):
        orchestrator, _context, _world, _items = self.build_orchestrator(Args())
        clearance = self.demo.compute_amr_exit_clearance(orchestrator.exit_pose[0], Args.drop_x)

        self.assertGreaterEqual(clearance, 0.60)
        self.assertGreater(self.demo.SLIDE_EXIT_DISTANCE, self.demo.PALLET_DECK_SCALE[0])

    def test_amr_starts_far_from_table_side_and_approaches_from_drop_side(self):
        orchestrator, _context, _world, _items = self.build_orchestrator(Args())

        self.assertGreater(orchestrator.start_pose[0], Args.pickup_x + 3.0)
        self.assertGreater(orchestrator.approach_pose[0], Args.pickup_x)
        self.assertLess(orchestrator.pickup_pose[0], orchestrator.approach_pose[0])
        self.assertLess(orchestrator.drop_approach_pose[0], Args.drop_x)
        self.assertGreater(orchestrator.drop_approach_pose[0], Args.pickup_x)

    def test_drop_docking_metrics_split_route_from_final_dock_in(self):
        metrics = self.demo.compute_drop_docking_metrics(Args.pickup_x, Args.drop_x)

        self.assertGreaterEqual(metrics["drop_approach_standoff"], 0.90)
        self.assertLess(metrics["drop_approach_x"], Args.drop_x)
        self.assertGreater(metrics["drop_approach_x"], Args.pickup_x)
        self.assertLess(metrics["dock_move_speed_scale"], 1.0)
        self.assertGreater(metrics["dock_move_speed_scale"], 0.0)

    def test_default_amr_z_is_on_warehouse_floor(self):
        self.assertAlmostEqual(self.demo.DEFAULT_AMR_Z, self.demo.WORLD_FLOOR_Z)
        self.assertLess(self.demo.DEFAULT_AMR_Z, -1.15)

    def test_amr_waypoint_motion_uses_eased_interpolation(self):
        orchestrator, _context, _world, _items = self.build_orchestrator(SlowArgs())

        orchestrator._transition(self.demo.TransferState.MOVE_TO_APPROACH)
        start = orchestrator.move_start_pose.copy()
        target = orchestrator.move_target.copy()
        duration = orchestrator.move_duration

        orchestrator.step(duration * 0.25)

        smooth_t = 0.25 * 0.25 * (3.0 - 2.0 * 0.25)
        expected = start + (target - start) * smooth_t
        np.testing.assert_allclose(orchestrator.get_amr_position(), expected)
        self.assertLess(abs(orchestrator.get_amr_position()[0] - start[0]), abs((target[0] - start[0]) * 0.25))

    def test_drop_route_stops_at_approach_before_final_dock(self):
        orchestrator, context, _world, _items = self.build_orchestrator(Args())
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.ATTACH)
        orchestrator.step(0.1)
        self.assertEqual(orchestrator.state, self.demo.TransferState.MOVE_TO_DROP_APPROACH)
        np.testing.assert_allclose(orchestrator.move_target, orchestrator.drop_approach_pose)

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.MOVE_TO_DROP)
        self.assertEqual(orchestrator.drop_dock_arrival_count, 1)
        np.testing.assert_allclose(orchestrator.move_target, orchestrator.drop_pose)
        self.assertAlmostEqual(orchestrator.drop_approach_final_error, 0.0)

    def test_loaded_route_metrics_track_y_error_and_carried_payload_pose(self):
        orchestrator, _context, _world, _items = self.build_orchestrator(Args())

        orchestrator._attach_assembly()
        shifted_pose = np.array([Args.pickup_x + 1.0, Args.pickup_y + 0.02, Args.amr_z])
        orchestrator.set_amr_pose(shifted_pose)
        orchestrator._update_attached_items()

        self.assertAlmostEqual(orchestrator.max_loaded_route_y_error, 0.02)
        self.assertAlmostEqual(
            orchestrator.min_loaded_route_guard_clearance,
            self.demo.compute_loaded_route_guard_clearance(Args.pickup_y + 0.02, Args.pickup_y, Args.drop_y),
        )
        self.assertAlmostEqual(orchestrator.max_carried_pallet_pose_error, 0.0)
        self.assertAlmostEqual(orchestrator.max_carried_payload_pose_error, 0.0)

    def test_amr_cell_gate_clearance_tracks_moving_amr_lateral_error(self):
        orchestrator, _context, _world, _items = self.build_orchestrator(Args())

        orchestrator.set_amr_pose(np.array([Args.pickup_x + 1.0, Args.pickup_y + 0.02, Args.amr_z]))

        self.assertAlmostEqual(
            orchestrator.min_amr_cell_gate_clearance,
            self.demo.compute_amr_cell_gate_clearance(Args.pickup_y + 0.02, Args.pickup_y),
        )

    def test_amr_lift_uses_eased_motion_and_settles_before_attach(self):
        orchestrator, context, _world, _items = self.build_orchestrator(Args())
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.LIFT_UP)
        orchestrator.step(self.demo.AMR_LIFT_DURATION * 0.25)

        expected = Args.lift_height * self.demo.smoothstep(0.25)
        self.assertAlmostEqual(orchestrator.lift_offset, expected)
        self.assertLess(orchestrator.lift_offset, Args.lift_height * 0.25)

        orchestrator.step(self.demo.AMR_LIFT_DURATION * 0.75)
        self.assertAlmostEqual(orchestrator.lift_offset, Args.lift_height)
        self.assertEqual(orchestrator.state, self.demo.TransferState.LIFT_UP)

        orchestrator.step(self.demo.AMR_LIFT_SETTLE_TIME + 0.01)
        self.assertEqual(orchestrator.state, self.demo.TransferState.ATTACH)

    def test_release_forces_suction_open_and_clears_attach_state(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("def force_open_suction_gripper", source)
        self.assertIn("SURFACE_GRIPPER_RELEASE_RETRIES", source)
        self.assertIn("def release_demo_bin_at_target", source)
        self.assertIn("def record_release_gripper_state", source)
        self.assertIn("interface.open_gripper(gripper_path)", source)
        self.assertIn("set_gripper_action_batch([gripper_path], [-1.0])", source)
        self.assertIn("interface.get_gripped_objects_batch([gripper_path])", source)
        self.assertIn("def record_release_visual_separation", source)
        self.assertIn("bin_state.demo_attached = False", source)
        self.assertIn("bin_state.demo_attach_T = None", source)
        self.assertIn("bin_state.demo_force_released = True", source)
        self.assertIn("active_bin.demo_force_released = False", source)
        self.assertIn("bin_state.is_attached = False", source)
        self.assertIn("bin_state.is_grasp_reached = False", source)
        self.assertIn("demo_scripted_place_bin = None", source)

    def test_spawned_bins_are_upside_down_to_skip_flip_station(self):
        for _ in range(10):
            position, orientation = self.demo.random_bin_spawn_transform()
            local_z_in_world = rotate_vector_by_quat([0.0, 0.0, 1.0], orientation)

            self.assertAlmostEqual(position[1], self.demo.CONVEYOR_PICK_WINDOW_Y)
            self.assertAlmostEqual(position[0], 0.0)
            np.testing.assert_allclose(orientation, self.demo.UPSIDE_DOWN_BIN_QUAT)
            self.assertLess(local_z_in_world[2], -0.99)

    def test_demo_uses_no_flip_dispatch(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("class NoFlipDispatch", source)
        self.assertIn("make_no_flip_decider_network", source)
        self.assertIn("class DemoAttachBin", source)
        self.assertIn("class DemoReleaseBin", source)
        self.assertIn("class DemoScriptedPlaceBin", source)
        self.assertIn("class DemoPickAndPlaceBin", source)
        self.assertIn("class DemoGifRecorder", source)
        self.assertIn("DEFAULT_GIF_OUTPUT_DIR", source)
        self.assertIn("LATEST_REVIEW_GIF_NAME", source)
        self.assertIn("GIF_FRAME_STRIDE", source)
        self.assertIn("GIF_MAX_FRAMES", source)
        self.assertIn("compute_review_gif_layout", source)
        self.assertIn("gif_recorder.maybe_capture", source)
        self.assertIn("save_review_gif", source)
        self.assertIn("review_gif_path=", source)
        self.assertIn("latest_review_gif_path=", source)
        self.assertIn("latest review GIF updated", source)
        self.assertIn("requested_enabled", source)
        self.assertIn("disabled_reason", source)
        self.assertIn("def _draw_fallback_frame", source)
        self.assertIn("PICK_READY_EE_POSITION", source)
        self.assertIn("class DemoTimedArmMoveTo", source)
        self.assertIn("class DemoTimedArmJointSettle", source)
        self.assertIn("CompletionSignalController", source)
        self.assertIn("StackCompleteSignalGreen", source)
        self.assertIn("create_floor_markings", source)
        self.assertIn("AmrPathCenterLine", source)
        self.assertIn("compute_load_restraint_specs", source)
        self.assertIn("LoadStrapTopLongitudinal", source)
        self.assertIn("load_restraint_parts", source)
        self.assertIn("_set_load_restraint_visibility", source)
        self.assertIn("create_infeed_conveyor_visual", source)
        self.assertIn("InfeedConveyorBelt", source)
        self.assertIn("InfeedMotionMarker", source)
        self.assertIn("make_infeed_motion_marker_specs", source)
        self.assertIn("InfeedConveyorMotionController", source)
        self.assertIn("infeed_motion_controller.update", source)
        self.assertIn("InfeedFeedCarton", source)
        self.assertIn("make_infeed_feed_carton_specs", source)
        self.assertIn("InfeedFeedCartonMotionController", source)
        self.assertIn("infeed_feed_carton_controller.update", source)
        self.assertIn("InfeedGuideRail", source)
        self.assertIn("InfeedPhotoEyeBeam", source)
        self.assertIn("create_safety_fence_visual", source)
        self.assertIn("SafetyFenceSouthRail", source)
        self.assertIn("SafetyFencePost_WGateLow", source)
        self.assertIn("compute_safety_fence_metrics", source)
        self.assertIn("create_amr_safety_visuals", source)
        self.assertIn("AmrBeaconDome", source)
        self.assertIn("AmrLeftWarningStrip", source)
        self.assertIn("AMR_WARNING_INDICATOR_NAMES", source)
        self.assertIn("AMR_IDLE_INDICATOR_NAMES", source)
        self.assertIn("get_amr_safety_visual_role", source)
        self.assertIn("AmrFrontSafetyScanner", source)
        self.assertIn("compute_amr_safety_visual_metrics", source)
        self.assertIn("_set_amr_safety_visual_pose", source)
        self.assertIn("_set_amr_indicator_visibility", source)
        self.assertIn("create_amr_drive_visuals", source)
        self.assertIn("AmrFrontLeftDriveWheel", source)
        self.assertIn("compute_amr_drive_visual_metrics", source)
        self.assertIn("_set_amr_drive_visual_pose", source)
        self.assertIn("MOVE_TO_DROP_APPROACH", source)
        self.assertIn("drop_approach_pose", source)
        self.assertIn("compute_drop_docking_metrics", source)
        self.assertIn("create_drop_dock_alignment_visual", source)
        self.assertIn("DropDockStopBlock", source)
        self.assertIn("DropDockLocatorPost", source)
        self.assertIn("compute_drop_dock_metrics", source)
        self.assertIn("create_pickup_dock_alignment_visual", source)
        self.assertIn("PickupDockStopBlock", source)
        self.assertIn("PickupDockLocatorPost", source)
        self.assertIn("PickupDockLocatorCap", source)
        self.assertIn("make_pickup_dock_alignment_specs", source)
        self.assertIn("compute_pickup_dock_metrics", source)
        self.assertIn("make_camera_rig_specs", source)
        self.assertIn("compute_camera_rig_metrics", source)
        self.assertIn("create_camera_rig", source)
        self.assertIn("OverviewCamera", source)
        self.assertIn("PalletizerCamera", source)
        self.assertIn("AmrRouteCamera", source)
        self.assertIn("DropDockCamera", source)
        self.assertIn("set_camera_view", source)
        self.assertIn("set_active_viewport_camera", source)
        self.assertIn("CAMERA_DIRECTOR_ROLE_BY_STATE_NAME", source)
        self.assertIn("camera_role_for_transfer_state", source)
        self.assertIn("compute_camera_director_metrics", source)
        self.assertIn("_request_camera_for_state", source)
        self.assertIn("camera director ->", source)
        self.assertIn("make_warehouse_light_specs", source)
        self.assertIn("compute_warehouse_lighting_metrics", source)
        self.assertIn("create_warehouse_lighting", source)
        self.assertIn("PalletizerHighBayLight", source)
        self.assertIn("RouteHighBayLightA", source)
        self.assertIn("DropDockHighBayLight", source)
        self.assertIn("RectLight", source)
        self.assertIn('add_zone_outline("Pickup"', source)
        self.assertIn('add_zone_outline("Drop"', source)
        self.assertIn("Zone{edge_name}", source)
        self.assertIn("FLOOR_MARKING_THICKNESS", source)
        self.assertIn("WORK_ZONE_MARKING_SIZE", source)
        self.assertIn("make_amr_route_guard_specs", source)
        self.assertIn("compute_amr_route_guard_metrics", source)
        self.assertIn("compute_loaded_route_y_error", source)
        self.assertIn("compute_loaded_route_guard_clearance", source)
        self.assertIn("_record_loaded_route_geometry", source)
        self.assertIn("create_amr_route_guard_visuals", source)
        self.assertIn("AmrRoute{side_name}Bollard", source)
        self.assertIn("AmrRoute{side_name}GuardRail", source)
        self.assertIn("AMR_ROUTE_GUARD_BOLLARD_SCALE", source)
        self.assertIn("ARM_CLEAR_SETTLE_TIME", source)
        self.assertIn("CARTON_BODY_SCALE", source)
        self.assertIn("HarimCartonBody", source)
        self.assertIn("HarimCartonTopTape", source)
        self.assertIn("HarimCartonSideLabel", source)
        self.assertIn("HarimCartonSideStripe", source)
        self.assertIn('("Front", 1.0)', source)
        self.assertIn('("Back", -1.0)', source)
        self.assertIn("CARTON_SIDE_LABEL_SCALE", source)
        self.assertIn("CARTON_SIDE_STRIPE_SCALE", source)
        self.assertIn("_add_carton_visual", source)
        self.assertIn("target_position=self.target_position", source)
        self.assertIn("posture_config=self.context.robot.default_config", source)
        self.assertIn("move_start_pose", source)
        self.assertIn("move_duration", source)
        self.assertIn("smoothstep", source)
        self.assertIn("AMR_LIFT_DURATION", source)
        self.assertIn("AMR_LIFT_SETTLE_TIME", source)
        self.assertIn("REACH_PICK_MAX_DURATION", source)
        self.assertIn("REACH_PLACE_MAX_DURATION", source)
        self.assertIn("RETURN_READY_DURATION", source)
        self.assertIn("RETURN_READY_POSITION_THRESHOLD", source)
        self.assertIn("POST_RELEASE_JOINT_SETTLE_DURATION", source)
        self.assertIn("position_error", source)
        self.assertIn("_record_final_error", source)
        self.assertIn('f"[HarimDemo] {self.label} reached; error=', source)
        self.assertIn('label="return_ready"', source)
        self.assertIn("hold_active_bin_for_pick", source)
        self.assertIn('getattr(self.context, "stack_complete", False)', source)
        self.assertIn("class DemoSettleBinAtGripper", source)
        self.assertIn("quat_lerp", source)
        self.assertIn("compute_active_bin_grasp_pose_at_effector", source)
        self.assertIn("place_active_bin_grasp_at_effector", source)
        self.assertIn("DemoScriptedPlaceBin()", source)
        self.assertIn("scripted-place", source)
        self.assertIn("scripted-release", source)
        self.assertIn("SCRIPTED_PLACE_DURATION", source)
        self.assertIn("SCRIPTED_PLACE_EE_HOVER", source)
        self.assertIn("get_demo_pre_grip_bin", source)
        self.assertIn("restore_demo_carried_active_bin", source)
        self.assertIn("clear_demo_carry_context", source)
        self.assertIn("mark_demo_bin_released", source)
        self.assertIn("release_demo_bin_at_target", source)
        self.assertIn("hold_demo_released_bin_at_target", source)
        self.assertIn("clone_stack_coordinates", source)
        self.assertIn("get_demo_stack_coordinate", source)
        self.assertIn("demo_stack_coordinates", source)
        self.assertIn("compute_stack_geometry_metrics", source)
        self.assertIn("get_demo_time", source)
        self.assertIn("demo_sim_time", source)
        self.assertIn('getattr(self.context, "demo_pre_grip_bin", None) is not None', source)
        self.assertIn("self.context.demo_pre_grip_bin = active_bin", source)
        self.assertIn("self.context.demo_pre_grip_bin = None", source)
        self.assertIn("demo_pre_grip_initial_offset", source)
        self.assertIn("suction close skipped for fallback attach", source)
        self.assertIn("using scripted attach", source)
        self.assertNotIn("suction_gripper.close()", source)
        self.assertIn("DemoSettleBinAtGripper(min_duration=0.25, max_duration=1.10)", source)
        self.assertIn("set_kinematic_for_demo(active_bin.bin_obj, True)", source)
        self.assertIn("PICK_STATION_BIN_POSITION", source)
        self.assertIn("demo_pick_stationed", source)
        self.assertIn("if self.state is None", source)
        self.assertIn("class DemoWaitForNextBin", source)
        self.assertIn("sync_demo_attached_bin", source)
        self.assertIn("demo_carried_bin", source)
        self.assertIn("demo_released_bin", source)
        self.assertIn("demo_release_target_p", source)
        self.assertIn("demo_max_release_drift", source)
        self.assertIn("demo_release_gripper_samples", source)
        self.assertIn("demo_release_gripped_object_max", source)
        self.assertIn("demo_scripted_place_count", source)
        self.assertIn("demo_max_scripted_place_error", source)
        self.assertIn("demo_max_release_separation", source)
        self.assertIn("demo_max_release_vertical_clearance", source)
        self.assertIn('getattr(context, "demo_scripted_place_bin", None) is active_bin', source)
        self.assertIn("max_payload_lift_observed", source)
        self.assertIn("max_dropped_payload_drift", source)
        self.assertIn("task.context = decider_network.context", source)
        self.assertIn("RELEASE_RETREAT_DURATION", source)
        self.assertIn("release_duration=RELEASE_RETREAT_DURATION", source)
        self.assertIn("POST_RELEASE_CLEARANCE_LIFT", source)
        self.assertIn("POST_RELEASE_RETREAT_OFFSET", source)
        self.assertIn("def _send_retreat_command", source)
        self.assertIn("self.retreat_position = np.array(self.target_p, dtype=float) + POST_RELEASE_RETREAT_OFFSET", source)
        self.assertIn("self.retreat_position[2] = max", source)
        self.assertIn("target_position=self.retreat_position", source)
        self.assertIn("posture_config=self.context.robot.default_config", source)
        self.assertIn("DemoTimedArmJointSettle()", source)
        self.assertIn("robot.set_joint_positions(joint_positions)", source)
        self.assertIn("robot.arm.soft_reset()", source)
        self.assertIn("force_open_suction_gripper(self.context)", source)
        self.assertIn('"wait_next_bin"', source)
        self.assertIn('return DfDecision("wait_next_bin")', source)
        self.assertIn("deactivate_stage_prims_containing", source)
        self.assertIn('"pallet_holder"', source)
        self.assertNotIn('add_child("flip_bin"', source)
        self.assertNotIn("behavior.make_decider_network", source)

    def test_normal_arm_self_test_can_require_placed_bins(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("--self-test-min-placed-bins", source)
        self.assertIn("--self-test-min-transfer-cycles", source)
        self.assertIn("--no-gif", source)
        self.assertIn("--gif-output-dir", source)
        self.assertIn("--gif-frame-stride", source)
        self.assertIn("--gif-max-frames", source)
        self.assertIn("--self-test-max-pre-grip-offset", source)
        self.assertIn("--self-test-max-return-ready-error", source)
        self.assertIn("--self-test-max-release-drift", source)
        self.assertIn("--self-test-min-release-retreat-lift", source)
        self.assertIn("--self-test-min-scripted-place-count", source)
        self.assertIn("--self-test-max-scripted-place-error", source)
        self.assertIn("--self-test-min-release-separation", source)
        self.assertIn("--self-test-min-release-vertical-clearance", source)
        self.assertIn("--self-test-require-gripper-open-after-release", source)
        self.assertIn("--self-test-max-stack-lateral-gap", source)
        self.assertIn("--self-test-max-stack-support-gap", source)
        self.assertIn("--self-test-min-stack-pallet-margin", source)
        self.assertIn("--self-test-min-load-restraint-count", source)
        self.assertIn("--self-test-min-load-restraint-pallet-margin", source)
        self.assertIn("--self-test-min-infeed-conveyor-length", source)
        self.assertIn("--self-test-min-infeed-spawn-margin", source)
        self.assertIn("--self-test-min-infeed-guide-clearance", source)
        self.assertIn("--self-test-max-infeed-belt-support-gap", source)
        self.assertIn("--self-test-min-infeed-motion-marker-count", source)
        self.assertIn("--self-test-min-infeed-motion-observed-travel", source)
        self.assertIn("--self-test-min-infeed-feed-carton-count", source)
        self.assertIn("--self-test-min-infeed-feed-carton-observed-travel", source)
        self.assertIn("--self-test-min-infeed-feed-carton-stop-clearance", source)
        self.assertIn("--self-test-min-infeed-feed-carton-guide-clearance", source)
        self.assertIn("--self-test-max-infeed-feed-carton-belt-support-gap", source)
        self.assertIn("--self-test-min-safety-fence-part-count", source)
        self.assertIn("--self-test-min-safety-fence-amr-gate-clearance", source)
        self.assertIn("--self-test-min-amr-cell-gate-clearance", source)
        self.assertIn("--self-test-min-safety-fence-infeed-gate-clearance", source)
        self.assertIn("--self-test-min-amr-safety-part-count", source)
        self.assertIn("--self-test-min-amr-safety-beacon-height", source)
        self.assertIn("--self-test-min-amr-safety-scanner-clearance", source)
        self.assertIn("--self-test-max-amr-safety-pose-error", source)
        self.assertIn("--self-test-min-amr-warning-indicator-count", source)
        self.assertIn("--self-test-min-amr-idle-indicator-count", source)
        self.assertIn("--self-test-min-amr-warning-observed", source)
        self.assertIn("--self-test-min-amr-idle-observed", source)
        self.assertIn("--self-test-max-amr-indicator-visibility-mismatches", source)
        self.assertIn("--self-test-min-amr-lift-guide-count", source)
        self.assertIn("--self-test-max-amr-lift-guide-bottom-gap", source)
        self.assertIn("--self-test-min-amr-lift-guide-travel-cover", source)
        self.assertIn("--self-test-max-amr-lift-guide-pose-error", source)
        self.assertIn("--self-test-min-payload-lift", source)
        self.assertIn("--self-test-max-dropped-payload-drift", source)
        self.assertIn("--self-test-min-dropped-stack-item-count", source)
        self.assertIn("--self-test-max-dropped-stack-pose-error", source)
        self.assertIn("--self-test-max-dropped-stack-support-gap", source)
        self.assertIn("--self-test-min-dropped-stack-pallet-margin", source)
        self.assertIn("--self-test-min-dropped-pallet-part-count", source)
        self.assertIn("--self-test-max-dropped-pallet-part-pose-error", source)
        self.assertIn("--self-test-min-amr-exit-clearance", source)
        self.assertIn("--self-test-max-lift-contact-gap", source)
        self.assertIn("--self-test-min-pallet-tunnel-clearance", source)
        self.assertIn("--self-test-min-lift-fork-inner-gap", source)
        self.assertIn("--self-test-max-pickup-handoff-xy-error", source)
        self.assertIn("--self-test-max-pickup-handoff-lift-gap", source)
        self.assertIn("--self-test-max-pickup-handoff-lift-penetration", source)
        self.assertIn("--self-test-max-pickup-entry-y-error", source)
        self.assertIn("--self-test-min-pickup-entry-tunnel-clearance", source)
        self.assertIn("--self-test-max-pickup-entry-lift-gap", source)
        self.assertIn("--self-test-max-pickup-entry-lift-penetration", source)
        self.assertIn("--self-test-max-slide-out-y-error", source)
        self.assertIn("--self-test-max-slide-out-lift-gap", source)
        self.assertIn("--self-test-max-slide-out-lift-penetration", source)
        self.assertIn("--self-test-max-drop-support-gap", source)
        self.assertIn("--self-test-max-drop-handoff-xy-error", source)
        self.assertIn("--self-test-max-drop-handoff-support-gap", source)
        self.assertIn("--self-test-max-drop-handoff-support-penetration", source)
        self.assertIn("--self-test-min-drop-lane-clearance", source)
        self.assertIn("--self-test-min-drop-runner-clearance", source)
        self.assertIn("--self-test-min-drop-fork-clearance", source)
        self.assertIn("--self-test-min-drop-dock-stop-count", source)
        self.assertIn("--self-test-max-drop-dock-stop-gap", source)
        self.assertIn("--self-test-min-drop-dock-guide-clearance", source)
        self.assertIn("--self-test-min-drop-dock-fork-clearance", source)
        self.assertIn("--self-test-min-pickup-dock-stop-count", source)
        self.assertIn("--self-test-max-pickup-dock-stop-gap", source)
        self.assertIn("--self-test-min-pickup-dock-guide-clearance", source)
        self.assertIn("--self-test-min-pickup-dock-fork-clearance", source)
        self.assertIn("--self-test-min-pickup-dock-runner-clearance", source)
        self.assertIn("--self-test-min-camera-count", source)
        self.assertIn("--self-test-min-camera-role-count", source)
        self.assertIn("--self-test-min-camera-height", source)
        self.assertIn("--self-test-min-camera-target-distance", source)
        self.assertIn("--self-test-min-camera-director-switch-count", source)
        self.assertIn("--self-test-min-camera-director-role-count", source)
        self.assertIn("--self-test-min-warehouse-light-count", source)
        self.assertIn("--self-test-min-warehouse-light-role-count", source)
        self.assertIn("--self-test-min-warehouse-light-height", source)
        self.assertIn("--self-test-min-warehouse-light-route-span", source)
        self.assertIn("--self-test-min-warehouse-light-intensity", source)
        self.assertIn("--self-test-min-amr-route-guard-part-count", source)
        self.assertIn("--self-test-min-amr-route-guard-span", source)
        self.assertIn("--self-test-min-amr-route-guard-clearance", source)
        self.assertIn("--self-test-min-amr-route-bollard-height", source)
        self.assertIn("--self-test-max-loaded-route-y-error", source)
        self.assertIn("--self-test-min-loaded-route-guard-clearance", source)
        self.assertIn("--self-test-max-carried-pallet-pose-error", source)
        self.assertIn("--self-test-max-carried-payload-pose-error", source)
        self.assertIn("--self-test-min-drop-approach-standoff", source)
        self.assertIn("--self-test-min-drop-dock-arrival-count", source)
        self.assertIn("--self-test-max-drop-dock-final-error", source)
        self.assertIn("UR10 placed", source)
        self.assertIn("AMR completed", source)
        self.assertIn("max pre-grip offset", source)
        self.assertIn("max return-ready error", source)
        self.assertIn("max release drift", source)
        self.assertIn("release retreat lift", source)
        self.assertIn("scripted place count", source)
        self.assertIn("scripted place error", source)
        self.assertIn("release separation", source)
        self.assertIn("release vertical clearance", source)
        self.assertIn("release gripper was not open", source)
        self.assertIn("release gripper still reported", source)
        self.assertIn("max stack lateral gap", source)
        self.assertIn("max stack support gap", source)
        self.assertIn("stack vertical overlap", source)
        self.assertIn("stack pallet margin", source)
        self.assertIn("load restraint count", source)
        self.assertIn("load restraint pallet margin", source)
        self.assertIn("infeed conveyor length", source)
        self.assertIn("infeed spawn margin", source)
        self.assertIn("infeed guide clearance", source)
        self.assertIn("infeed belt support gap", source)
        self.assertIn("infeed motion marker count", source)
        self.assertIn("infeed motion observed travel", source)
        self.assertIn("infeed feed carton count", source)
        self.assertIn("infeed feed carton observed travel", source)
        self.assertIn("infeed feed carton stop clearance", source)
        self.assertIn("infeed feed carton guide clearance", source)
        self.assertIn("infeed feed carton belt support gap", source)
        self.assertIn("safety fence part count", source)
        self.assertIn("safety fence AMR gate clearance", source)
        self.assertIn("AMR cell gate clearance", source)
        self.assertIn("safety fence infeed gate clearance", source)
        self.assertIn("AMR safety part count", source)
        self.assertIn("AMR safety beacon height", source)
        self.assertIn("AMR safety scanner clearance", source)
        self.assertIn("AMR safety pose error", source)
        self.assertIn("AMR warning indicator count", source)
        self.assertIn("AMR idle indicator count", source)
        self.assertIn("AMR warning indicator observed", source)
        self.assertIn("AMR idle indicator observed", source)
        self.assertIn("AMR indicator visibility mismatches", source)
        self.assertIn("AMR lift guide count", source)
        self.assertIn("AMR lift guide bottom gap", source)
        self.assertIn("AMR lift guide travel cover", source)
        self.assertIn("AMR lift guide pose error", source)
        self.assertIn("payload lift", source)
        self.assertIn("max dropped payload drift", source)
        self.assertIn("dropped stack item count", source)
        self.assertIn("dropped stack pose error", source)
        self.assertIn("dropped stack support gap", source)
        self.assertIn("dropped stack pallet margin", source)
        self.assertIn("dropped pallet part count", source)
        self.assertIn("dropped pallet part pose error", source)
        self.assertIn("AMR exit clearance", source)
        self.assertIn("max lift contact gap", source)
        self.assertIn("pallet tunnel clearance", source)
        self.assertIn("lift fork inner gap", source)
        self.assertIn("pickup handoff geometry", source)
        self.assertIn("pickup handoff XY error", source)
        self.assertIn("pickup handoff lift gap", source)
        self.assertIn("pickup handoff lift penetration", source)
        self.assertIn("slide-out geometry", source)
        self.assertIn("slide-out Y error", source)
        self.assertIn("slide-out lift gap", source)
        self.assertIn("slide-out lift penetration", source)
        self.assertIn("drop support gap", source)
        self.assertIn("drop handoff XY error", source)
        self.assertIn("drop handoff support gap", source)
        self.assertIn("drop handoff support penetration", source)
        self.assertIn("drop lane tunnel clearance", source)
        self.assertIn("drop lane runner clearance", source)
        self.assertIn("drop lane fork clearance", source)
        self.assertIn("drop dock stop count", source)
        self.assertIn("drop dock stop gap", source)
        self.assertIn("drop dock guide clearance", source)
        self.assertIn("drop dock fork clearance", source)
        self.assertIn("pickup dock stop count", source)
        self.assertIn("pickup dock stop gap", source)
        self.assertIn("pickup dock guide clearance", source)
        self.assertIn("pickup dock fork clearance", source)
        self.assertIn("pickup dock runner clearance", source)
        self.assertIn("camera rig count", source)
        self.assertIn("camera role count", source)
        self.assertIn("camera height", source)
        self.assertIn("camera target distance", source)
        self.assertIn("camera director switch count", source)
        self.assertIn("camera director role count", source)
        self.assertIn("warehouse light count", source)
        self.assertIn("warehouse light role count", source)
        self.assertIn("warehouse light height", source)
        self.assertIn("warehouse light route span", source)
        self.assertIn("warehouse light intensity", source)
        self.assertIn("AMR route guard part count", source)
        self.assertIn("AMR route guard span", source)
        self.assertIn("AMR route guard clearance", source)
        self.assertIn("AMR route bollard height", source)
        self.assertIn("loaded route Y error", source)
        self.assertIn("loaded route guard clearance", source)
        self.assertIn("carried pallet pose error", source)
        self.assertIn("carried payload pose error", source)
        self.assertIn("drop approach standoff", source)
        self.assertIn("drop dock arrival count", source)
        self.assertIn("drop dock final error", source)
        self.assertIn("max_pre_grip_offset=", source)
        self.assertIn("max_return_ready_error=", source)
        self.assertIn("max_release_drift=", source)
        self.assertIn("max_release_retreat_lift=", source)
        self.assertIn("scripted_place_count=", source)
        self.assertIn("max_scripted_place_error=", source)
        self.assertIn("max_release_separation=", source)
        self.assertIn("max_release_vertical_clearance=", source)
        self.assertIn("release_gripper_not_open=", source)
        self.assertIn("release_gripped_object_max=", source)
        self.assertIn("joint_settle_count=", source)
        self.assertIn("max_stack_lateral_gap=", source)
        self.assertIn("max_stack_support_gap=", source)
        self.assertIn("min_stack_pallet_margin=", source)
        self.assertIn("max_stack_pallet_overhang=", source)
        self.assertIn("load_restraint_part_count=", source)
        self.assertIn("min_load_restraint_pallet_margin=", source)
        self.assertIn("max_load_restraint_pallet_overhang=", source)
        self.assertIn("infeed_conveyor_length=", source)
        self.assertIn("infeed_spawn_margin=", source)
        self.assertIn("infeed_pick_margin=", source)
        self.assertIn("infeed_guide_clearance=", source)
        self.assertIn("infeed_belt_support_gap=", source)
        self.assertIn("infeed_motion_marker_count=", source)
        self.assertIn("infeed_motion_marker_spacing=", source)
        self.assertIn("infeed_motion_marker_speed=", source)
        self.assertIn("infeed_motion_observed_travel=", source)
        self.assertIn("infeed_feed_carton_count=", source)
        self.assertIn("infeed_feed_carton_path_length=", source)
        self.assertIn("infeed_feed_carton_speed=", source)
        self.assertIn("infeed_feed_carton_observed_travel=", source)
        self.assertIn("infeed_feed_carton_stop_clearance=", source)
        self.assertIn("infeed_feed_carton_guide_clearance=", source)
        self.assertIn("infeed_feed_carton_belt_support_gap=", source)
        self.assertIn("safety_fence_part_count=", source)
        self.assertIn("safety_fence_amr_gate_clearance=", source)
        self.assertIn("amr_cell_gate_clearance=", source)
        self.assertIn("safety_fence_infeed_gate_clearance=", source)
        self.assertIn("amr_safety_part_count=", source)
        self.assertIn("amr_safety_beacon_height=", source)
        self.assertIn("amr_safety_scanner_clearance=", source)
        self.assertIn("max_amr_safety_pose_error=", source)
        self.assertIn("amr_warning_indicator_count=", source)
        self.assertIn("amr_idle_indicator_count=", source)
        self.assertIn("amr_warning_indicator_observed=", source)
        self.assertIn("amr_idle_indicator_observed=", source)
        self.assertIn("amr_indicator_visibility_mismatches=", source)
        self.assertIn("amr_drive_part_count=", source)
        self.assertIn("amr_wheel_count=", source)
        self.assertIn("amr_wheel_floor_gap=", source)
        self.assertIn("amr_wheel_floor_penetration=", source)
        self.assertIn("amr_drive_wheelbase=", source)
        self.assertIn("amr_drive_track_width=", source)
        self.assertIn("max_amr_drive_pose_error=", source)
        self.assertIn("amr_lift_guide_count=", source)
        self.assertIn("amr_lift_guide_bottom_gap=", source)
        self.assertIn("amr_lift_guide_bottom_penetration=", source)
        self.assertIn("amr_lift_guide_travel_cover=", source)
        self.assertIn("amr_lift_guide_min_height=", source)
        self.assertIn("max_amr_lift_guide_pose_error=", source)
        self.assertIn("max_payload_lift=", source)
        self.assertIn("max_dropped_payload_drift=", source)
        self.assertIn("dropped_stack_item_count=", source)
        self.assertIn("max_dropped_stack_pose_error=", source)
        self.assertIn("max_dropped_stack_support_gap=", source)
        self.assertIn("min_dropped_stack_support_gap=", source)
        self.assertIn("min_dropped_stack_pallet_margin=", source)
        self.assertIn("max_dropped_stack_pallet_overhang=", source)
        self.assertIn("dropped_pallet_part_count=", source)
        self.assertIn("max_dropped_pallet_part_pose_error=", source)
        self.assertIn("amr_exit_clearance=", source)
        self.assertIn("max_lift_contact_gap=", source)
        self.assertIn("pallet_tunnel_clearance=", source)
        self.assertIn("lift_fork_inner_gap=", source)
        self.assertIn("pickup_handoff_count=", source)
        self.assertIn("max_pickup_handoff_xy_error=", source)
        self.assertIn("max_pickup_handoff_lift_gap=", source)
        self.assertIn("max_pickup_handoff_lift_penetration=", source)
        self.assertIn("pickup_entry_sample_count=", source)
        self.assertIn("max_pickup_entry_y_error=", source)
        self.assertIn("min_pickup_entry_tunnel_clearance=", source)
        self.assertIn("max_pickup_entry_lift_gap=", source)
        self.assertIn("max_pickup_entry_lift_penetration=", source)
        self.assertIn("pickup entry geometry", source)
        self.assertIn("pickup entry Y error", source)
        self.assertIn("pickup entry tunnel clearance", source)
        self.assertIn("pickup entry lift gap", source)
        self.assertIn("pickup entry lift penetration", source)
        self.assertIn("slide_out_sample_count=", source)
        self.assertIn("max_slide_out_y_error=", source)
        self.assertIn("max_slide_out_lift_gap=", source)
        self.assertIn("max_slide_out_lift_penetration=", source)
        self.assertIn("drop_support_gap=", source)
        self.assertIn("drop_handoff_xy_error=", source)
        self.assertIn("drop_handoff_support_gap=", source)
        self.assertIn("drop_handoff_support_penetration=", source)
        self.assertIn("drop_lane_clearance=", source)
        self.assertIn("drop_runner_clearance=", source)
        self.assertIn("drop_fork_clearance=", source)
        self.assertIn("drop_dock_stop_count=", source)
        self.assertIn("drop_dock_stop_gap=", source)
        self.assertIn("drop_dock_guide_clearance=", source)
        self.assertIn("drop_dock_fork_clearance=", source)
        self.assertIn("drop_dock_runner_clearance=", source)
        self.assertIn("pickup_dock_stop_count=", source)
        self.assertIn("pickup_dock_stop_gap=", source)
        self.assertIn("pickup_dock_guide_clearance=", source)
        self.assertIn("pickup_dock_fork_clearance=", source)
        self.assertIn("pickup_dock_runner_clearance=", source)
        self.assertIn("camera_rig_count=", source)
        self.assertIn("camera_required_role_count=", source)
        self.assertIn("camera_min_height=", source)
        self.assertIn("camera_min_target_distance=", source)
        self.assertIn("camera_director_switch_count=", source)
        self.assertIn("camera_director_role_count=", source)
        self.assertIn("warehouse_light_count=", source)
        self.assertIn("warehouse_light_role_count=", source)
        self.assertIn("warehouse_light_min_height=", source)
        self.assertIn("warehouse_light_route_span=", source)
        self.assertIn("warehouse_light_min_intensity=", source)
        self.assertIn("amr_route_guard_part_count=", source)
        self.assertIn("amr_route_guard_span=", source)
        self.assertIn("amr_route_guard_clearance=", source)
        self.assertIn("amr_route_bollard_height=", source)
        self.assertIn("max_loaded_route_y_error=", source)
        self.assertIn("min_loaded_route_guard_clearance=", source)
        self.assertIn("max_carried_pallet_pose_error=", source)
        self.assertIn("max_carried_payload_pose_error=", source)
        self.assertIn("drop_approach_standoff=", source)
        self.assertIn("dock_move_speed_scale=", source)
        self.assertIn("drop_dock_arrival_count=", source)
        self.assertIn("drop_approach_final_error=", source)
        self.assertIn("drop_dock_final_error=", source)
        self.assertIn("review_gif_path=", source)
        self.assertIn("demo_max_return_ready_error", source)
        self.assertIn("demo_max_release_drift", source)
        self.assertIn("demo_max_release_retreat_lift", source)
        self.assertIn("preserving failure exit", source)
        self.assertIn("os._exit(1)", source)
        self.assertIn("transfer_cycles=", source)
        self.assertIn("expected at least", source)
        self.assertIn("[HarimDemo] self-test failed", source)

    def test_runner_exposes_release_gripper_open_gate(self):
        source = RUNNER_PATH.read_text(encoding="utf-8")

        self.assertIn("$SelfTestRequireGripperOpenAfterRelease", source)
        self.assertIn("--self-test-require-gripper-open-after-release", source)
        self.assertIn("$NoGif", source)
        self.assertIn("$GifOutputDir", source)
        self.assertIn("$GifFrameStride", source)
        self.assertIn("$GifMaxFrames", source)
        self.assertIn("--no-gif", source)
        self.assertIn("--gif-output-dir", source)
        self.assertIn("--gif-frame-stride", source)
        self.assertIn("--gif-max-frames", source)
        self.assertIn("$SelfTestMinReleaseRetreatLift", source)
        self.assertIn("$SelfTestMinScriptedPlaceCount", source)
        self.assertIn("$SelfTestMaxScriptedPlaceError", source)
        self.assertIn("$SelfTestMinReleaseSeparation", source)
        self.assertIn("$SelfTestMinReleaseVerticalClearance", source)
        self.assertIn("$SelfTestMaxStackLateralGap", source)
        self.assertIn("$SelfTestMaxStackSupportGap", source)
        self.assertIn("$SelfTestMinStackPalletMargin", source)
        self.assertIn("$SelfTestMinLoadRestraintCount", source)
        self.assertIn("$SelfTestMinLoadRestraintPalletMargin", source)
        self.assertIn("$SelfTestMinInfeedConveyorLength", source)
        self.assertIn("$SelfTestMinInfeedSpawnMargin", source)
        self.assertIn("$SelfTestMinInfeedGuideClearance", source)
        self.assertIn("$SelfTestMaxInfeedBeltSupportGap", source)
        self.assertIn("$SelfTestMinInfeedMotionMarkerCount", source)
        self.assertIn("$SelfTestMinInfeedMotionObservedTravel", source)
        self.assertIn("$SelfTestMinInfeedFeedCartonCount", source)
        self.assertIn("$SelfTestMinInfeedFeedCartonObservedTravel", source)
        self.assertIn("$SelfTestMinInfeedFeedCartonStopClearance", source)
        self.assertIn("$SelfTestMinInfeedFeedCartonGuideClearance", source)
        self.assertIn("$SelfTestMaxInfeedFeedCartonBeltSupportGap", source)
        self.assertIn("$SelfTestMinSafetyFencePartCount", source)
        self.assertIn("$SelfTestMinSafetyFenceAmrGateClearance", source)
        self.assertIn("$SelfTestMinAmrCellGateClearance", source)
        self.assertIn("$SelfTestMinSafetyFenceInfeedGateClearance", source)
        self.assertIn("$SelfTestMinAmrSafetyPartCount", source)
        self.assertIn("$SelfTestMinAmrSafetyBeaconHeight", source)
        self.assertIn("$SelfTestMinAmrSafetyScannerClearance", source)
        self.assertIn("$SelfTestMaxAmrSafetyPoseError", source)
        self.assertIn("$SelfTestMinAmrWarningIndicatorCount", source)
        self.assertIn("$SelfTestMinAmrIdleIndicatorCount", source)
        self.assertIn("$SelfTestMinAmrWarningObserved", source)
        self.assertIn("$SelfTestMinAmrIdleObserved", source)
        self.assertIn("$SelfTestMaxAmrIndicatorVisibilityMismatches", source)
        self.assertIn("$SelfTestMinAmrDrivePartCount", source)
        self.assertIn("$SelfTestMaxAmrDrivePoseError", source)
        self.assertIn("$SelfTestMaxAmrWheelFloorGap", source)
        self.assertIn("$SelfTestMaxAmrWheelFloorPenetration", source)
        self.assertIn("$SelfTestMinAmrLiftGuideCount", source)
        self.assertIn("$SelfTestMaxAmrLiftGuideBottomGap", source)
        self.assertIn("$SelfTestMinAmrLiftGuideTravelCover", source)
        self.assertIn("$SelfTestMaxAmrLiftGuidePoseError", source)
        self.assertIn("$SelfTestMinDroppedStackItemCount", source)
        self.assertIn("$SelfTestMaxDroppedStackPoseError", source)
        self.assertIn("$SelfTestMaxDroppedStackSupportGap", source)
        self.assertIn("$SelfTestMinDroppedStackPalletMargin", source)
        self.assertIn("$SelfTestMinDroppedPalletPartCount", source)
        self.assertIn("$SelfTestMaxDroppedPalletPartPoseError", source)
        self.assertIn("$SelfTestMinAmrExitClearance", source)
        self.assertIn("$SelfTestMaxLiftContactGap", source)
        self.assertIn("$SelfTestMinPalletTunnelClearance", source)
        self.assertIn("$SelfTestMinLiftForkInnerGap", source)
        self.assertIn("$SelfTestMaxPickupHandoffXyError", source)
        self.assertIn("$SelfTestMaxPickupHandoffLiftGap", source)
        self.assertIn("$SelfTestMaxPickupHandoffLiftPenetration", source)
        self.assertIn("$SelfTestMaxPickupEntryYError", source)
        self.assertIn("$SelfTestMinPickupEntryTunnelClearance", source)
        self.assertIn("$SelfTestMaxPickupEntryLiftGap", source)
        self.assertIn("$SelfTestMaxPickupEntryLiftPenetration", source)
        self.assertIn("$SelfTestMaxSlideOutYError", source)
        self.assertIn("$SelfTestMaxSlideOutLiftGap", source)
        self.assertIn("$SelfTestMaxSlideOutLiftPenetration", source)
        self.assertIn("$SelfTestMaxDropSupportGap", source)
        self.assertIn("$SelfTestMaxDropHandoffXyError", source)
        self.assertIn("$SelfTestMaxDropHandoffSupportGap", source)
        self.assertIn("$SelfTestMaxDropHandoffSupportPenetration", source)
        self.assertIn("$SelfTestMinDropLaneClearance", source)
        self.assertIn("$SelfTestMinDropRunnerClearance", source)
        self.assertIn("$SelfTestMinDropForkClearance", source)
        self.assertIn("$SelfTestMinDropDockStopCount", source)
        self.assertIn("$SelfTestMaxDropDockStopGap", source)
        self.assertIn("$SelfTestMinDropDockGuideClearance", source)
        self.assertIn("$SelfTestMinDropDockForkClearance", source)
        self.assertIn("$SelfTestMinPickupDockStopCount", source)
        self.assertIn("$SelfTestMaxPickupDockStopGap", source)
        self.assertIn("$SelfTestMinPickupDockGuideClearance", source)
        self.assertIn("$SelfTestMinPickupDockForkClearance", source)
        self.assertIn("$SelfTestMinPickupDockRunnerClearance", source)
        self.assertIn("$SelfTestMinCameraCount", source)
        self.assertIn("$SelfTestMinCameraRoleCount", source)
        self.assertIn("$SelfTestMinCameraHeight", source)
        self.assertIn("$SelfTestMinCameraTargetDistance", source)
        self.assertIn("$SelfTestMinCameraDirectorSwitchCount", source)
        self.assertIn("$SelfTestMinCameraDirectorRoleCount", source)
        self.assertIn("$SelfTestMinWarehouseLightCount", source)
        self.assertIn("$SelfTestMinWarehouseLightRoleCount", source)
        self.assertIn("$SelfTestMinWarehouseLightHeight", source)
        self.assertIn("$SelfTestMinWarehouseLightRouteSpan", source)
        self.assertIn("$SelfTestMinWarehouseLightIntensity", source)
        self.assertIn("$SelfTestMinAmrRouteGuardPartCount", source)
        self.assertIn("$SelfTestMinAmrRouteGuardSpan", source)
        self.assertIn("$SelfTestMinAmrRouteGuardClearance", source)
        self.assertIn("$SelfTestMinAmrRouteBollardHeight", source)
        self.assertIn("$SelfTestMaxLoadedRouteYError", source)
        self.assertIn("$SelfTestMinLoadedRouteGuardClearance", source)
        self.assertIn("$SelfTestMaxCarriedPalletPoseError", source)
        self.assertIn("$SelfTestMaxCarriedPayloadPoseError", source)
        self.assertIn("$SelfTestMinDropApproachStandoff", source)
        self.assertIn("$SelfTestMinDropDockArrivalCount", source)
        self.assertIn("$SelfTestMaxDropDockFinalError", source)
        self.assertIn("--self-test-max-stack-lateral-gap", source)
        self.assertIn("--self-test-max-stack-support-gap", source)
        self.assertIn("--self-test-min-stack-pallet-margin", source)
        self.assertIn("--self-test-min-load-restraint-count", source)
        self.assertIn("--self-test-min-load-restraint-pallet-margin", source)
        self.assertIn("--self-test-min-infeed-conveyor-length", source)
        self.assertIn("--self-test-min-infeed-spawn-margin", source)
        self.assertIn("--self-test-min-infeed-guide-clearance", source)
        self.assertIn("--self-test-max-infeed-belt-support-gap", source)
        self.assertIn("--self-test-min-infeed-motion-marker-count", source)
        self.assertIn("--self-test-min-infeed-motion-observed-travel", source)
        self.assertIn("--self-test-min-infeed-feed-carton-count", source)
        self.assertIn("--self-test-min-infeed-feed-carton-observed-travel", source)
        self.assertIn("--self-test-min-infeed-feed-carton-stop-clearance", source)
        self.assertIn("--self-test-min-infeed-feed-carton-guide-clearance", source)
        self.assertIn("--self-test-max-infeed-feed-carton-belt-support-gap", source)
        self.assertIn("--self-test-min-safety-fence-part-count", source)
        self.assertIn("--self-test-min-safety-fence-amr-gate-clearance", source)
        self.assertIn("--self-test-min-amr-cell-gate-clearance", source)
        self.assertIn("--self-test-min-safety-fence-infeed-gate-clearance", source)
        self.assertIn("--self-test-min-amr-safety-part-count", source)
        self.assertIn("--self-test-min-amr-safety-beacon-height", source)
        self.assertIn("--self-test-min-amr-safety-scanner-clearance", source)
        self.assertIn("--self-test-max-amr-safety-pose-error", source)
        self.assertIn("--self-test-min-amr-warning-indicator-count", source)
        self.assertIn("--self-test-min-amr-idle-indicator-count", source)
        self.assertIn("--self-test-min-amr-warning-observed", source)
        self.assertIn("--self-test-min-amr-idle-observed", source)
        self.assertIn("--self-test-max-amr-indicator-visibility-mismatches", source)
        self.assertIn("--self-test-min-amr-drive-part-count", source)
        self.assertIn("--self-test-max-amr-drive-pose-error", source)
        self.assertIn("--self-test-max-amr-wheel-floor-gap", source)
        self.assertIn("--self-test-max-amr-wheel-floor-penetration", source)
        self.assertIn("--self-test-min-amr-lift-guide-count", source)
        self.assertIn("--self-test-max-amr-lift-guide-bottom-gap", source)
        self.assertIn("--self-test-min-amr-lift-guide-travel-cover", source)
        self.assertIn("--self-test-max-amr-lift-guide-pose-error", source)
        self.assertIn("--self-test-min-dropped-stack-item-count", source)
        self.assertIn("--self-test-max-dropped-stack-pose-error", source)
        self.assertIn("--self-test-max-dropped-stack-support-gap", source)
        self.assertIn("--self-test-min-dropped-stack-pallet-margin", source)
        self.assertIn("--self-test-min-dropped-pallet-part-count", source)
        self.assertIn("--self-test-max-dropped-pallet-part-pose-error", source)
        self.assertIn("--self-test-min-amr-exit-clearance", source)
        self.assertIn("--self-test-max-lift-contact-gap", source)
        self.assertIn("--self-test-min-pallet-tunnel-clearance", source)
        self.assertIn("--self-test-min-lift-fork-inner-gap", source)
        self.assertIn("--self-test-max-pickup-handoff-xy-error", source)
        self.assertIn("--self-test-max-pickup-handoff-lift-gap", source)
        self.assertIn("--self-test-max-pickup-handoff-lift-penetration", source)
        self.assertIn("--self-test-max-pickup-entry-y-error", source)
        self.assertIn("--self-test-min-pickup-entry-tunnel-clearance", source)
        self.assertIn("--self-test-max-pickup-entry-lift-gap", source)
        self.assertIn("--self-test-max-pickup-entry-lift-penetration", source)
        self.assertIn("--self-test-max-slide-out-y-error", source)
        self.assertIn("--self-test-max-slide-out-lift-gap", source)
        self.assertIn("--self-test-max-slide-out-lift-penetration", source)
        self.assertIn("--self-test-max-drop-support-gap", source)
        self.assertIn("--self-test-max-drop-handoff-xy-error", source)
        self.assertIn("--self-test-max-drop-handoff-support-gap", source)
        self.assertIn("--self-test-max-drop-handoff-support-penetration", source)
        self.assertIn("--self-test-min-drop-lane-clearance", source)
        self.assertIn("--self-test-min-drop-runner-clearance", source)
        self.assertIn("--self-test-min-drop-fork-clearance", source)
        self.assertIn("--self-test-min-drop-dock-stop-count", source)
        self.assertIn("--self-test-max-drop-dock-stop-gap", source)
        self.assertIn("--self-test-min-drop-dock-guide-clearance", source)
        self.assertIn("--self-test-min-drop-dock-fork-clearance", source)
        self.assertIn("--self-test-min-camera-count", source)
        self.assertIn("--self-test-min-camera-role-count", source)
        self.assertIn("--self-test-min-camera-height", source)
        self.assertIn("--self-test-min-camera-target-distance", source)
        self.assertIn("--self-test-min-camera-director-switch-count", source)
        self.assertIn("--self-test-min-camera-director-role-count", source)
        self.assertIn("--self-test-min-warehouse-light-count", source)
        self.assertIn("--self-test-min-warehouse-light-role-count", source)
        self.assertIn("--self-test-min-warehouse-light-height", source)
        self.assertIn("--self-test-min-warehouse-light-route-span", source)
        self.assertIn("--self-test-min-warehouse-light-intensity", source)
        self.assertIn("--self-test-min-amr-route-guard-part-count", source)
        self.assertIn("--self-test-min-amr-route-guard-span", source)
        self.assertIn("--self-test-min-amr-route-guard-clearance", source)
        self.assertIn("--self-test-min-amr-route-bollard-height", source)
        self.assertIn("--self-test-max-loaded-route-y-error", source)
        self.assertIn("--self-test-min-loaded-route-guard-clearance", source)
        self.assertIn("--self-test-max-carried-pallet-pose-error", source)
        self.assertIn("--self-test-max-carried-payload-pose-error", source)
        self.assertIn("--self-test-min-drop-approach-standoff", source)
        self.assertIn("--self-test-min-drop-dock-arrival-count", source)
        self.assertIn("--self-test-max-drop-dock-final-error", source)
        self.assertIn("--self-test-min-release-retreat-lift", source)
        self.assertIn("--self-test-min-scripted-place-count", source)
        self.assertIn("--self-test-max-scripted-place-error", source)
        self.assertIn("--self-test-min-release-separation", source)
        self.assertIn("--self-test-min-release-vertical-clearance", source)
        self.assertIn("--self-test-min-pickup-dock-stop-count", source)
        self.assertIn("--self-test-max-pickup-dock-stop-gap", source)
        self.assertIn("--self-test-min-pickup-dock-guide-clearance", source)
        self.assertIn("--self-test-min-pickup-dock-fork-clearance", source)
        self.assertIn("--self-test-min-pickup-dock-runner-clearance", source)

    def test_strict_runner_enables_all_current_realism_gates(self):
        source = STRICT_RUNNER_PATH.read_text(encoding="utf-8")

        self.assertIn("run_harim_demo.ps1", source)
        self.assertIn("[int]$SelfTestFrames = 12000", source)
        self.assertIn("$RunnerArgs = @", source)
        self.assertIn("SelfTestMinPlacedBins", source)
        self.assertIn("8", source)
        self.assertIn("SelfTestMinTransferCycles", source)
        self.assertIn("1", source)
        self.assertIn("SelfTestMaxPreGripOffset", source)
        self.assertIn("0.05", source)
        self.assertIn("SelfTestMaxReturnReadyError", source)
        self.assertIn("SelfTestMaxReleaseDrift", source)
        self.assertIn("0.005", source)
        self.assertIn("SelfTestMinReleaseRetreatLift", source)
        self.assertIn("0.20", source)
        self.assertIn("SelfTestMinScriptedPlaceCount", source)
        self.assertIn("SelfTestMaxScriptedPlaceError", source)
        self.assertIn("SelfTestMinReleaseSeparation", source)
        self.assertIn("SelfTestMinReleaseVerticalClearance", source)
        self.assertIn("0.35", source)
        self.assertIn("SelfTestRequireGripperOpenAfterRelease = $true", source)
        self.assertIn("SelfTestMaxStackLateralGap", source)
        self.assertIn("0.03", source)
        self.assertIn("SelfTestMaxStackSupportGap", source)
        self.assertIn("0.02", source)
        self.assertIn("SelfTestMinStackPalletMargin", source)
        self.assertIn("0.08", source)
        self.assertIn("SelfTestMinLoadRestraintCount", source)
        self.assertIn("6", source)
        self.assertIn("SelfTestMinLoadRestraintPalletMargin", source)
        self.assertIn("0.06", source)
        self.assertIn("SelfTestMinInfeedConveyorLength", source)
        self.assertIn("0.80", source)
        self.assertIn("SelfTestMinInfeedSpawnMargin", source)
        self.assertIn("0.30", source)
        self.assertIn("SelfTestMinInfeedGuideClearance", source)
        self.assertIn("0.40", source)
        self.assertIn("SelfTestMaxInfeedBeltSupportGap", source)
        self.assertIn("SelfTestMinInfeedMotionMarkerCount", source)
        self.assertIn("SelfTestMinInfeedMotionObservedTravel", source)
        self.assertIn("SelfTestMinInfeedFeedCartonCount", source)
        self.assertIn("SelfTestMinInfeedFeedCartonObservedTravel", source)
        self.assertIn("SelfTestMinInfeedFeedCartonStopClearance", source)
        self.assertIn("SelfTestMinInfeedFeedCartonGuideClearance", source)
        self.assertIn("SelfTestMaxInfeedFeedCartonBeltSupportGap", source)
        self.assertIn("SelfTestMinSafetyFencePartCount", source)
        self.assertIn("20", source)
        self.assertIn("SelfTestMinSafetyFenceAmrGateClearance", source)
        self.assertIn("SelfTestMinAmrCellGateClearance", source)
        self.assertIn("0.25", source)
        self.assertIn("SelfTestMinSafetyFenceInfeedGateClearance", source)
        self.assertIn("SelfTestMinAmrSafetyPartCount", source)
        self.assertIn("8", source)
        self.assertIn("SelfTestMinAmrSafetyBeaconHeight", source)
        self.assertIn("0.60", source)
        self.assertIn("SelfTestMinAmrSafetyScannerClearance", source)
        self.assertIn("SelfTestMaxAmrSafetyPoseError", source)
        self.assertIn("SelfTestMinAmrWarningIndicatorCount", source)
        self.assertIn("SelfTestMinAmrIdleIndicatorCount", source)
        self.assertIn("SelfTestMinAmrWarningObserved", source)
        self.assertIn("SelfTestMinAmrIdleObserved", source)
        self.assertIn("SelfTestMaxAmrIndicatorVisibilityMismatches", source)
        self.assertIn("SelfTestMinAmrDrivePartCount", source)
        self.assertIn("SelfTestMaxAmrDrivePoseError", source)
        self.assertIn("SelfTestMaxAmrWheelFloorGap", source)
        self.assertIn("SelfTestMaxAmrWheelFloorPenetration", source)
        self.assertIn("SelfTestMinAmrLiftGuideCount", source)
        self.assertIn("SelfTestMaxAmrLiftGuideBottomGap", source)
        self.assertIn("SelfTestMinAmrLiftGuideTravelCover", source)
        self.assertIn("SelfTestMaxAmrLiftGuidePoseError", source)
        self.assertIn("SelfTestMinPayloadLift", source)
        self.assertIn("0.10", source)
        self.assertIn("SelfTestMaxDroppedPayloadDrift", source)
        self.assertIn("SelfTestMinDroppedStackItemCount", source)
        self.assertIn("SelfTestMaxDroppedStackPoseError", source)
        self.assertIn("SelfTestMaxDroppedStackSupportGap", source)
        self.assertIn("SelfTestMinDroppedStackPalletMargin", source)
        self.assertIn("SelfTestMinDroppedPalletPartCount", source)
        self.assertIn("SelfTestMaxDroppedPalletPartPoseError", source)
        self.assertIn("SelfTestMinAmrExitClearance", source)
        self.assertIn("0.60", source)
        self.assertIn("SelfTestMaxLiftContactGap", source)
        self.assertIn("0.01", source)
        self.assertIn("SelfTestMinPalletTunnelClearance", source)
        self.assertIn("SelfTestMinLiftForkInnerGap", source)
        self.assertIn("0.30", source)
        self.assertIn("SelfTestMaxPickupHandoffXyError", source)
        self.assertIn("SelfTestMaxPickupHandoffLiftGap", source)
        self.assertIn("SelfTestMaxPickupHandoffLiftPenetration", source)
        self.assertIn("SelfTestMaxPickupEntryYError", source)
        self.assertIn("SelfTestMinPickupEntryTunnelClearance", source)
        self.assertIn("SelfTestMaxPickupEntryLiftGap", source)
        self.assertIn("SelfTestMaxPickupEntryLiftPenetration", source)
        self.assertIn("SelfTestMaxSlideOutYError", source)
        self.assertIn("SelfTestMaxSlideOutLiftGap", source)
        self.assertIn("SelfTestMaxSlideOutLiftPenetration", source)
        self.assertIn("SelfTestMaxDropSupportGap", source)
        self.assertIn("SelfTestMaxDropHandoffXyError", source)
        self.assertIn("SelfTestMaxDropHandoffSupportGap", source)
        self.assertIn("SelfTestMaxDropHandoffSupportPenetration", source)
        self.assertIn("SelfTestMinDropLaneClearance", source)
        self.assertIn("SelfTestMinDropRunnerClearance", source)
        self.assertIn("SelfTestMinDropForkClearance", source)
        self.assertIn("SelfTestMinDropDockStopCount", source)
        self.assertIn("2", source)
        self.assertIn("SelfTestMaxDropDockStopGap", source)
        self.assertIn("0.05", source)
        self.assertIn("SelfTestMinDropDockGuideClearance", source)
        self.assertIn("0.10", source)
        self.assertIn("SelfTestMinDropDockForkClearance", source)
        self.assertIn("SelfTestMinPickupDockStopCount", source)
        self.assertIn("SelfTestMaxPickupDockStopGap", source)
        self.assertIn("SelfTestMinPickupDockGuideClearance", source)
        self.assertIn("SelfTestMinPickupDockForkClearance", source)
        self.assertIn("SelfTestMinPickupDockRunnerClearance", source)
        self.assertIn("SelfTestMinCameraCount", source)
        self.assertIn("SelfTestMinCameraRoleCount", source)
        self.assertIn("SelfTestMinCameraHeight", source)
        self.assertIn("1.25", source)
        self.assertIn("SelfTestMinCameraTargetDistance", source)
        self.assertIn("SelfTestMinCameraDirectorSwitchCount", source)
        self.assertIn("SelfTestMinCameraDirectorRoleCount", source)
        self.assertIn("SelfTestMinWarehouseLightCount", source)
        self.assertIn("SelfTestMinWarehouseLightRoleCount", source)
        self.assertIn("SelfTestMinWarehouseLightHeight", source)
        self.assertIn("SelfTestMinWarehouseLightRouteSpan", source)
        self.assertIn("SelfTestMinWarehouseLightIntensity", source)
        self.assertIn("SelfTestMinAmrRouteGuardPartCount", source)
        self.assertIn("SelfTestMinAmrRouteGuardSpan", source)
        self.assertIn("SelfTestMinAmrRouteGuardClearance", source)
        self.assertIn("SelfTestMinAmrRouteBollardHeight", source)
        self.assertIn("SelfTestMaxLoadedRouteYError", source)
        self.assertIn("SelfTestMinLoadedRouteGuardClearance", source)
        self.assertIn("SelfTestMaxCarriedPalletPoseError", source)
        self.assertIn("SelfTestMaxCarriedPayloadPoseError", source)
        self.assertIn("SelfTestMinDropApproachStandoff", source)
        self.assertIn("SelfTestMinDropDockArrivalCount", source)
        self.assertIn("SelfTestMaxDropDockFinalError", source)
        self.assertIn('if (-not $ShowGui)', source)
        self.assertIn("Headless = $true", source)

    def test_drop_slide_workstation_is_created(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("create_drop_slide_workstation", source)
        self.assertIn("DropSlideRail", source)
        self.assertIn("DropSlideRoller", source)
        self.assertIn("DropSlideLeg", source)
        self.assertIn("DropSlideTopSupport", source)
        self.assertIn("DROP_SLIDE_LANE_Y_OFFSETS", source)
        self.assertIn("DROP_SLIDE_SUPPORT_TOP_Z", source)
        self.assertIn("create_drop_dock_alignment_visual", source)
        self.assertIn("DropDockStopBlock", source)
        self.assertIn("DropDockLocatorPost", source)

    def test_connected_pallet_uses_fixed_collision_support(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("FixedCuboid", source)
        self.assertIn("harim_pallet_connected_top_deck", source)
        self.assertIn("PalletRunner", source)
        self.assertIn("PalletTopSupport", source)
        self.assertIn("PALLET_TOP_SUPPORT_SCALE", source)

    def test_lift_plate_remains_visible_as_contact_surface(self):
        source = DEMO_PATH.read_text(encoding="utf-8")

        self.assertIn("AMR_LIFT_PLATE_OFFSET_Z", source)
        self.assertIn("LIFT_FORK_SCALE", source)
        self.assertIn("LIFT_FORK_OFFSETS", source)
        self.assertIn("IwHubLiftFork", source)
        self.assertIn("visible=True", source)

    def test_lift_fork_parts_move_with_amr(self):
        lift_parts = [FakePosePrim("fork_0"), FakePosePrim("fork_1")]
        orchestrator, _context, _world, _items = self.build_orchestrator(
            Args(),
            completion_signal=None,
        )
        orchestrator.lift_plate_parts = lift_parts

        orchestrator.set_amr_pose(np.array([1.0, -0.3, Args.amr_z]))
        orchestrator._set_lift_plate_pose()

        for fork, offset in zip(lift_parts, self.demo.LIFT_FORK_OFFSETS):
            expected = np.array([1.0, -0.3, Args.amr_z]) + offset
            expected[2] += self.demo.AMR_LIFT_PLATE_OFFSET_Z
            np.testing.assert_allclose(fork.get_world_pose()[0], expected)

    def test_amr_safety_visual_parts_move_with_amr(self):
        safety_parts = [FakePosePrim(f"safety_{idx}") for idx in range(2)]
        safety_offsets = [np.array([0.1, 0.2, 0.3]), np.array([-0.2, -0.1, 0.4])]
        orchestrator, _context, _world, _items = self.build_orchestrator(
            Args(),
            amr_safety_parts=safety_parts,
            amr_safety_offsets=safety_offsets,
        )

        target_pose = np.array([1.4, -0.2, Args.amr_z])
        orchestrator.set_amr_pose(target_pose)

        for part, offset in zip(safety_parts, safety_offsets):
            np.testing.assert_allclose(part.get_world_pose()[0], target_pose + offset)
        self.assertAlmostEqual(orchestrator.max_amr_safety_pose_error, 0.0)

    def test_amr_drive_visual_parts_move_with_amr(self):
        drive_parts = [FakePosePrim(f"drive_{idx}") for idx in range(2)]
        drive_offsets = [np.array([0.43, -0.48, 0.075]), np.array([-0.43, 0.48, 0.075])]
        orchestrator, _context, _world, _items = self.build_orchestrator(
            Args(),
            amr_drive_parts=drive_parts,
            amr_drive_offsets=drive_offsets,
        )

        target_pose = np.array([1.8, -0.35, Args.amr_z])
        orchestrator.set_amr_pose(target_pose)

        for part, offset in zip(drive_parts, drive_offsets):
            np.testing.assert_allclose(part.get_world_pose()[0], target_pose + offset)
        self.assertAlmostEqual(orchestrator.max_amr_drive_pose_error, 0.0)

    def test_amr_indicator_visibility_tracks_transfer_state(self):
        warning = FakePosePrim("warning")
        idle = FakePosePrim("idle")
        static = FakePosePrim("static")
        orchestrator, _context, _world, _items = self.build_orchestrator(
            Args(),
            amr_safety_parts=[warning, idle, static],
            amr_safety_offsets=[np.zeros(3), np.zeros(3), np.zeros(3)],
            amr_safety_roles=["warning", "idle", "static"],
        )

        self.assertFalse(warning.visible)
        self.assertTrue(idle.visible)
        self.assertTrue(static.visible)

        orchestrator._transition(self.demo.TransferState.MOVE_TO_APPROACH)

        self.assertTrue(warning.visible)
        self.assertFalse(idle.visible)
        self.assertTrue(static.visible)
        self.assertGreater(orchestrator.amr_warning_indicator_on_observed, 0)
        self.assertGreater(orchestrator.amr_idle_indicator_on_observed, 0)
        self.assertEqual(orchestrator.amr_indicator_visibility_mismatch_count, 0)

    def test_camera_director_tracks_transfer_state_story(self):
        requested_roles = []
        orchestrator, _context, _world, _items = self.build_orchestrator(
            Args(),
            camera_director=requested_roles.append,
        )

        self.assertEqual(requested_roles, ["palletizer"])
        orchestrator._transition(self.demo.TransferState.ARM_SETTLE)
        orchestrator._transition(self.demo.TransferState.MOVE_TO_APPROACH)
        orchestrator._transition(self.demo.TransferState.MOVE_UNDER_PALLET)
        orchestrator._transition(self.demo.TransferState.LIFT_DOWN)
        orchestrator._transition(self.demo.TransferState.RESET_CYCLE)

        self.assertEqual(requested_roles, ["palletizer", "overview", "amr_route", "drop_dock", "overview"])
        self.assertEqual(orchestrator.camera_director_switch_count, 5)
        self.assertEqual(orchestrator.camera_director_requested_roles, {"palletizer", "overview", "amr_route", "drop_dock"})

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
        self.assertAlmostEqual(orchestrator.max_dropped_payload_drift, 0.0)

    def test_dropped_payload_drift_metric_records_external_motion(self):
        orchestrator, context, _world, items = self.build_orchestrator(Args())
        context.stack_complete = True

        self.run_until(orchestrator, lambda: orchestrator.state == self.demo.TransferState.SLIDE_OUT_FROM_PALLET)
        items[0].set_world_pose(position=items[0].get_world_pose()[0] + np.array([0.03, 0.0, 0.0]))
        orchestrator.pallet_parts[0].set_world_pose(
            position=orchestrator.pallet_parts[0].get_world_pose()[0] + np.array([0.0, 0.02, 0.0])
        )

        orchestrator.step(0.1)

        self.assertGreater(orchestrator.max_dropped_payload_drift, 0.029)

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
        self.assertGreaterEqual(orchestrator.max_payload_lift_observed, Args.lift_height)

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
