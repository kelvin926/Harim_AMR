import importlib.util
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
            pallet_parts=pallet_parts,
            pallet_part_offsets=pallet_part_offsets,
            load_restraint_parts=load_restraint_parts,
            stack_coordinates=self.demo.make_stack_coordinates(2, 2, 1),
            args=args,
            completion_signal=completion_signal,
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
        self.assertGreaterEqual(self.demo.POST_RELEASE_CLEARANCE_LIFT, 0.28)
        self.assertGreaterEqual(self.demo.RELEASE_RETREAT_DURATION, 0.45)
        self.assertGreaterEqual(self.demo.SCRIPTED_PLACE_DURATION, 0.60)
        self.assertGreaterEqual(self.demo.SCRIPTED_PLACE_EE_HOVER, 0.15)

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

        self.assertGreaterEqual(metrics["infeed_conveyor_length"], 0.80)
        self.assertGreaterEqual(metrics["infeed_spawn_margin"], 0.30)
        self.assertGreaterEqual(metrics["infeed_pick_margin"], 0.20)
        self.assertGreaterEqual(metrics["infeed_guide_clearance"], 0.40)
        self.assertGreaterEqual(metrics["infeed_belt_support_gap"], 0.0)
        self.assertLessEqual(metrics["infeed_belt_support_gap"], 0.02)
        self.assertLess(self.demo.INFEED_CONVEYOR_START_Y, self.demo.PICK_STATION_BIN_POSITION[1])
        self.assertGreater(self.demo.INFEED_CONVEYOR_END_Y, self.demo.CONVEYOR_PICK_WINDOW_Y)

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

    def test_drop_dock_stops_and_locator_posts_leave_clearance(self):
        metrics = self.demo.compute_drop_dock_metrics()

        self.assertEqual(metrics["drop_dock_stop_count"], 2)
        self.assertGreaterEqual(metrics["drop_dock_stop_gap"], 0.0)
        self.assertLessEqual(metrics["drop_dock_stop_gap"], 0.05)
        self.assertGreaterEqual(metrics["drop_dock_guide_clearance"], 0.10)
        self.assertGreaterEqual(metrics["drop_dock_fork_clearance"], 0.03)
        self.assertGreaterEqual(metrics["drop_dock_runner_clearance"], 0.05)

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
        self.assertIn("def record_release_gripper_state", source)
        self.assertIn("interface.open_gripper(gripper_path)", source)
        self.assertIn("interface.get_gripped_objects_batch([gripper_path])", source)
        self.assertIn("def record_release_visual_separation", source)
        self.assertIn("active_bin.demo_attached = False", source)
        self.assertIn("active_bin.demo_attach_T = None", source)
        self.assertIn("active_bin.demo_force_released = True", source)
        self.assertIn("active_bin.demo_force_released = False", source)
        self.assertIn("active_bin.is_attached = False", source)
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
        self.assertIn("InfeedGuideRail", source)
        self.assertIn("InfeedPhotoEyeBeam", source)
        self.assertIn("create_drop_dock_alignment_visual", source)
        self.assertIn("DropDockStopBlock", source)
        self.assertIn("DropDockLocatorPost", source)
        self.assertIn("compute_drop_dock_metrics", source)
        self.assertIn('add_zone_outline("Pickup"', source)
        self.assertIn('add_zone_outline("Drop"', source)
        self.assertIn("Zone{edge_name}", source)
        self.assertIn("FLOOR_MARKING_THICKNESS", source)
        self.assertIn("WORK_ZONE_MARKING_SIZE", source)
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
        self.assertIn("SCRIPTED_PLACE_DURATION", source)
        self.assertIn("SCRIPTED_PLACE_EE_HOVER", source)
        self.assertIn("get_demo_pre_grip_bin", source)
        self.assertIn("restore_demo_carried_active_bin", source)
        self.assertIn("clear_demo_carry_context", source)
        self.assertIn("mark_demo_bin_released", source)
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
        self.assertIn("self.retreat_pq = self.context.robot.arm.get_fk_pq()", source)
        self.assertIn("self.retreat_pq.p[2] += POST_RELEASE_CLEARANCE_LIFT", source)
        self.assertIn("self.context.robot.arm.send(MotionCommand(self.retreat_pq))", source)
        self.assertIn("POST_RELEASE_CLEARANCE_LIFT", source)
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
        self.assertIn("--self-test-max-pre-grip-offset", source)
        self.assertIn("--self-test-max-return-ready-error", source)
        self.assertIn("--self-test-max-release-drift", source)
        self.assertIn("--self-test-min-release-retreat-lift", source)
        self.assertIn("--self-test-min-scripted-place-count", source)
        self.assertIn("--self-test-max-scripted-place-error", source)
        self.assertIn("--self-test-min-release-separation", source)
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
        self.assertIn("--self-test-min-payload-lift", source)
        self.assertIn("--self-test-max-dropped-payload-drift", source)
        self.assertIn("--self-test-min-amr-exit-clearance", source)
        self.assertIn("--self-test-max-lift-contact-gap", source)
        self.assertIn("--self-test-min-pallet-tunnel-clearance", source)
        self.assertIn("--self-test-min-lift-fork-inner-gap", source)
        self.assertIn("--self-test-max-drop-support-gap", source)
        self.assertIn("--self-test-min-drop-lane-clearance", source)
        self.assertIn("--self-test-min-drop-runner-clearance", source)
        self.assertIn("--self-test-min-drop-fork-clearance", source)
        self.assertIn("--self-test-min-drop-dock-stop-count", source)
        self.assertIn("--self-test-max-drop-dock-stop-gap", source)
        self.assertIn("--self-test-min-drop-dock-guide-clearance", source)
        self.assertIn("--self-test-min-drop-dock-fork-clearance", source)
        self.assertIn("UR10 placed", source)
        self.assertIn("AMR completed", source)
        self.assertIn("max pre-grip offset", source)
        self.assertIn("max return-ready error", source)
        self.assertIn("max release drift", source)
        self.assertIn("release retreat lift", source)
        self.assertIn("scripted place count", source)
        self.assertIn("scripted place error", source)
        self.assertIn("release separation", source)
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
        self.assertIn("payload lift", source)
        self.assertIn("max dropped payload drift", source)
        self.assertIn("AMR exit clearance", source)
        self.assertIn("max lift contact gap", source)
        self.assertIn("pallet tunnel clearance", source)
        self.assertIn("lift fork inner gap", source)
        self.assertIn("drop support gap", source)
        self.assertIn("drop lane tunnel clearance", source)
        self.assertIn("drop lane runner clearance", source)
        self.assertIn("drop lane fork clearance", source)
        self.assertIn("drop dock stop count", source)
        self.assertIn("drop dock stop gap", source)
        self.assertIn("drop dock guide clearance", source)
        self.assertIn("drop dock fork clearance", source)
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
        self.assertIn("max_payload_lift=", source)
        self.assertIn("max_dropped_payload_drift=", source)
        self.assertIn("amr_exit_clearance=", source)
        self.assertIn("max_lift_contact_gap=", source)
        self.assertIn("pallet_tunnel_clearance=", source)
        self.assertIn("lift_fork_inner_gap=", source)
        self.assertIn("drop_support_gap=", source)
        self.assertIn("drop_lane_clearance=", source)
        self.assertIn("drop_runner_clearance=", source)
        self.assertIn("drop_fork_clearance=", source)
        self.assertIn("drop_dock_stop_count=", source)
        self.assertIn("drop_dock_stop_gap=", source)
        self.assertIn("drop_dock_guide_clearance=", source)
        self.assertIn("drop_dock_fork_clearance=", source)
        self.assertIn("drop_dock_runner_clearance=", source)
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
        self.assertIn("$SelfTestMinReleaseRetreatLift", source)
        self.assertIn("$SelfTestMinScriptedPlaceCount", source)
        self.assertIn("$SelfTestMaxScriptedPlaceError", source)
        self.assertIn("$SelfTestMinReleaseSeparation", source)
        self.assertIn("$SelfTestMaxStackLateralGap", source)
        self.assertIn("$SelfTestMaxStackSupportGap", source)
        self.assertIn("$SelfTestMinStackPalletMargin", source)
        self.assertIn("$SelfTestMinLoadRestraintCount", source)
        self.assertIn("$SelfTestMinLoadRestraintPalletMargin", source)
        self.assertIn("$SelfTestMinInfeedConveyorLength", source)
        self.assertIn("$SelfTestMinInfeedSpawnMargin", source)
        self.assertIn("$SelfTestMinInfeedGuideClearance", source)
        self.assertIn("$SelfTestMaxInfeedBeltSupportGap", source)
        self.assertIn("$SelfTestMinAmrExitClearance", source)
        self.assertIn("$SelfTestMaxLiftContactGap", source)
        self.assertIn("$SelfTestMinPalletTunnelClearance", source)
        self.assertIn("$SelfTestMinLiftForkInnerGap", source)
        self.assertIn("$SelfTestMaxDropSupportGap", source)
        self.assertIn("$SelfTestMinDropLaneClearance", source)
        self.assertIn("$SelfTestMinDropRunnerClearance", source)
        self.assertIn("$SelfTestMinDropForkClearance", source)
        self.assertIn("$SelfTestMinDropDockStopCount", source)
        self.assertIn("$SelfTestMaxDropDockStopGap", source)
        self.assertIn("$SelfTestMinDropDockGuideClearance", source)
        self.assertIn("$SelfTestMinDropDockForkClearance", source)
        self.assertIn("--self-test-max-stack-lateral-gap", source)
        self.assertIn("--self-test-max-stack-support-gap", source)
        self.assertIn("--self-test-min-stack-pallet-margin", source)
        self.assertIn("--self-test-min-load-restraint-count", source)
        self.assertIn("--self-test-min-load-restraint-pallet-margin", source)
        self.assertIn("--self-test-min-infeed-conveyor-length", source)
        self.assertIn("--self-test-min-infeed-spawn-margin", source)
        self.assertIn("--self-test-min-infeed-guide-clearance", source)
        self.assertIn("--self-test-max-infeed-belt-support-gap", source)
        self.assertIn("--self-test-min-amr-exit-clearance", source)
        self.assertIn("--self-test-max-lift-contact-gap", source)
        self.assertIn("--self-test-min-pallet-tunnel-clearance", source)
        self.assertIn("--self-test-min-lift-fork-inner-gap", source)
        self.assertIn("--self-test-max-drop-support-gap", source)
        self.assertIn("--self-test-min-drop-lane-clearance", source)
        self.assertIn("--self-test-min-drop-runner-clearance", source)
        self.assertIn("--self-test-min-drop-fork-clearance", source)
        self.assertIn("--self-test-min-drop-dock-stop-count", source)
        self.assertIn("--self-test-max-drop-dock-stop-gap", source)
        self.assertIn("--self-test-min-drop-dock-guide-clearance", source)
        self.assertIn("--self-test-min-drop-dock-fork-clearance", source)
        self.assertIn("--self-test-min-release-retreat-lift", source)
        self.assertIn("--self-test-min-scripted-place-count", source)
        self.assertIn("--self-test-max-scripted-place-error", source)
        self.assertIn("--self-test-min-release-separation", source)

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
        self.assertIn("SelfTestMinPayloadLift", source)
        self.assertIn("0.10", source)
        self.assertIn("SelfTestMaxDroppedPayloadDrift", source)
        self.assertIn("SelfTestMinAmrExitClearance", source)
        self.assertIn("0.60", source)
        self.assertIn("SelfTestMaxLiftContactGap", source)
        self.assertIn("0.01", source)
        self.assertIn("SelfTestMinPalletTunnelClearance", source)
        self.assertIn("SelfTestMinLiftForkInnerGap", source)
        self.assertIn("0.30", source)
        self.assertIn("SelfTestMaxDropSupportGap", source)
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
