"""Deterministic mission tests without ROS or aircraft."""

import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


class Message:
    def __init__(self, data=None):
        self.data = data


class PoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id='', stamp=0)
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0., y=0., z=0.),
            orientation=types.SimpleNamespace(x=0., y=0., z=0., w=0.))


class Publisher:
    def __init__(self, *args, **kwargs):
        self.messages = []
        self.connections = 1
        self.hook = lambda msg: None

    def publish(self, msg):
        self.messages.append(msg)
        self.hook(msg)

    def get_num_connections(self):
        return self.connections


def install_ros_stubs():
    ros = types.ModuleType("rospy")
    ros.get_param = lambda name, default=None: default
    ros.Publisher = Publisher
    ros.Subscriber = lambda *a, **k: None
    ros.Timer = lambda *a, **k: None
    ros.Duration = float
    ros.Time = types.SimpleNamespace(now=lambda: 0)
    ros.is_shutdown = lambda: False
    ros.loginfo = ros.logerr = ros.logfatal = lambda *a: None
    sys.modules["rospy"] = ros
    for package, names in (("nav_msgs", ["Odometry"]),
                           ("geometry_msgs", ["PoseStamped"]),
                           ("quadrotor_msgs", ["PositionCommand"]),
                           ("std_msgs", ["Float32MultiArray", "Int16", "String"])):
        module = types.ModuleType(package)
        messages = types.ModuleType(package + ".msg")
        for name in names:
            setattr(messages, name, PoseStamped if name == 'PoseStamped' else Message)
        module.msg = messages
        sys.modules[package] = module
        sys.modules[package + ".msg"] = messages


install_ros_stubs()
services = types.ModuleType("std_srvs.srv")
services.Trigger = object
services.TriggerResponse = lambda **kwargs: types.SimpleNamespace(**kwargs)
sys.modules["std_srvs"] = types.ModuleType("std_srvs")
sys.modules["std_srvs.srv"] = services
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fcu_auto_mission import Drone, MissionError
import fcu_auto_mission.drone as driver

entry_spec = importlib.util.spec_from_file_location(
    "mission_entry", Path(__file__).resolve().parent / "fcu_auto_mission_node.py")
entry = importlib.util.module_from_spec(entry_spec)
entry_spec.loader.exec_module(entry)


def odometry(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return types.SimpleNamespace(pose=types.SimpleNamespace(pose=types.SimpleNamespace(
        position=types.SimpleNamespace(x=x, y=y, z=z),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2)))))


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.step = lambda: None
        self.monotonic = patch.object(driver.time, "monotonic", lambda: self.now)
        self.sleep = patch.object(driver.time, "sleep", self.advance)
        self.monotonic.start()
        self.sleep.start()
        self.addCleanup(self.monotonic.stop)
        self.addCleanup(self.sleep.stop)
        self.drone = Drone()
        self.feedback()

    def advance(self, seconds):
        self.now += seconds
        self.step()
        self.drone._publish_target(None)

    def feedback(self, x=0, y=0, z=0, yaw=0):
        self.drone._on_odom(odometry(x, y, z, yaw))

    def commands(self):
        return [m.data for m in self.drone.command_pub.messages]

    def test_full_six_action_mission(self):
        self.drone.prepare()
        self.step = self.feedback
        self.drone.arm(wait=0.2)
        self.drone.takeoff(wait=0.2)
        # Deliberately wrong Z and yaw: arrival must still depend only on XY.
        self.step = lambda: self.feedback(x=1, y=2, z=99, yaw=2)
        self.drone.goto(1, 2, 1, 0.4, timeout=2)
        target = self.drone.target_pub.messages[-1].data
        self.assertEqual(target, [-0.4, 0, 1, -2, 1, 0, 0, 0, 0, 0, 0])
        before = len(self.drone.target_pub.messages)
        self.drone.hover(0.2)
        self.assertGreater(len(self.drone.target_pub.messages), before)
        before = len(self.drone.target_pub.messages)
        self.drone.land(wait=0.3)
        self.assertIsNone(self.drone._target)
        self.drone.disarm(wait=0.2)
        self.assertEqual(len(self.drone.target_pub.messages), before)
        self.assertEqual(self.commands(), [1, 3, 4, 2])

    def test_xy_must_stay_inside_tolerance(self):
        self.feedback()
        started = self.now
        self.step = lambda: self.feedback(x=1 if self.now - started < 0.3 or self.now - started > 0.6 else 0)
        self.drone.goto(1, 0, 10, timeout=2)
        self.assertGreaterEqual(self.now - started, 1.1)

    def test_goto_times_out_when_xy_does_not_arrive(self):
        self.feedback()
        self.step = self.feedback
        with self.assertRaisesRegex(MissionError, "Goto timed out"):
            self.drone.goto(10, 0, 1, timeout=0.2)

    def test_timed_commands_need_no_state_or_odometry_feedback(self):
        self.drone._odom_at = None
        started = self.now
        for action in (self.drone.arm, self.drone.takeoff, self.drone.land, self.drone.disarm):
            before = self.now
            action(wait=0.2)
            self.assertGreaterEqual(self.now - before, 0.2)
        self.assertEqual(self.commands(), [1, 3, 4, 2])
        self.assertGreaterEqual(self.now - started, 0.8)

    def test_default_command_waits(self):
        for action, duration in ((self.drone.arm, 2), (self.drone.takeoff, 5),
                                 (self.drone.land, 15), (self.drone.disarm, 2)):
            before = self.now
            action()
            self.assertGreaterEqual(self.now - before, duration)
            self.assertLess(self.now - before, duration + 0.1)

    def test_invalid_wait_never_sends_command(self):
        for action in (self.drone.arm, self.drone.takeoff, self.drone.land, self.drone.disarm):
            for value in (-1, float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    action(wait=value)
        self.assertEqual(self.commands(), [])

    def test_cancel_during_land_prevents_following_disarm(self):
        self.drone._target = (1, 2, 3, 0)
        self.step = self.drone.cancel
        with self.assertRaisesRegex(MissionError, 'cancelled'):
            self.drone.land(wait=1)
            self.drone.disarm()
        self.assertEqual(self.commands(), [4])
        self.assertIsNone(self.drone._target)
        self.assertEqual(self.drone.target_pub.messages, [])

    def test_cancel_holds_current_pose_and_blocks_commands(self):
        self.feedback(x=0.3, y=0.4, z=1)
        self.drone._target = (9, 9, 1, 0)
        self.drone.cancel()
        self.drone._publish_target(None)
        self.assertEqual(self.drone._target[:2], (0.3, 0.4))
        self.assertEqual(self.drone.target_pub.messages[-1].data[2:5], [0.3, -0.4, 1])
        with self.assertRaisesRegex(MissionError, "cancelled"):
            self.drone.land()
        self.assertEqual(self.commands(), [])

    def test_cancel_during_landing_does_not_publish_hold(self):
        self.feedback()
        self.drone.cancel()
        self.drone._publish_target(None)
        self.assertEqual(self.drone.target_pub.messages, [])

    def test_stale_feedback_stops_target_publication(self):
        self.feedback()
        self.drone._target = (1, 2, 3, 0)
        self.now += 4
        self.drone._publish_target(None)
        self.assertEqual(self.drone.target_pub.messages, [])
        with self.assertRaisesRegex(MissionError, "Odometry"):
            self.drone.hover(1)

    def test_target_publishes_without_armed_feedback(self):
        self.drone._target = (1, 2, 3, 0)
        self.drone._publish_target(None)
        self.assertEqual(self.drone.target_pub.messages[-1].data[2:5], [1, -2, 3])

    def test_prepare_requires_bridge_connections(self):
        self.drone.command_pub.connections = 0
        with self.assertRaisesRegex(MissionError, "not connected"):
            self.drone.prepare()

    def test_invalid_target_is_never_published(self):
        self.feedback()
        for value in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                self.drone.goto(value, 0, 1)
        self.assertIsNone(self.drone._target)

    def test_hover_without_goto_captures_current_pose(self):
        self.feedback(x=2, y=3, z=4, yaw=0.7)
        self.step = lambda: self.feedback(x=99)
        self.drone.hover(0.2)
        self.assertEqual(self.drone._target[:3], (2, 3, 4))
        self.assertAlmostEqual(self.drone._target[3], 0.7)

    def runner(self, mission):
        runner = entry.MissionRunner.__new__(entry.MissionRunner)
        runner.drone = self.drone
        runner.mission = mission
        runner._lock = driver.threading.Lock()
        runner._worker = None
        return runner

    def test_runner_reports_failure_and_holds_on_action_timeout(self):
        self.feedback(x=0.2, y=0.3, z=1)
        self.step = lambda: self.feedback(x=0.2, y=0.3, z=1)
        runner = self.runner(lambda drone: drone.goto(10, 0, 1, timeout=0.2))
        runner._run()
        self.assertIn("FAILED: Goto timed out", self.drone.status_pub.messages[-1].data)
        self.assertEqual(self.drone._target[:2], (0.2, 0.3))

    def test_runner_refuses_concurrent_start(self):
        runner = self.runner(lambda drone: None)
        runner._worker = types.SimpleNamespace(is_alive=lambda: True)
        self.assertFalse(runner.start(None).success)

    def test_runner_start_refuses_missing_state(self):
        runner = self.runner(lambda drone: None)
        self.now += 4
        response = runner.start(None)
        self.assertFalse(response.success)
        self.assertIn("Odometry", response.message)
        self.assertIsNone(runner._worker)

    def test_cancellation_interrupts_wait(self):
        self.feedback()
        self.step = self.drone.cancel
        with self.assertRaisesRegex(MissionError, "cancelled"):
            self.drone.goto(10, 0, 1, timeout=2)
        self.assertEqual(self.drone._target[:2], (0, 0))

    def planning(self):
        self.drone.use_planner = True
        self.drone._planning_active = True
        self.drone._planner_started = self.now
        self.drone._target = (10, 0, 1, 0)

    def planner_sample(self, x=1, y=2, z=3, yaw=0.7, flag=1):
        return types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=z),
                                     yaw=yaw, trajectory_flag=flag)

    def test_planner_goto_publishes_goal_once_and_forwards_samples(self):
        self.drone.use_planner = True
        self.step = lambda: (self.feedback(x=1, y=2),
                             self.drone._on_pos_cmd(self.planner_sample()))
        self.drone.goto(1, 2, 9, 0.7, timeout=2)
        self.assertEqual(len(self.drone.target_pub.messages), 1)
        self.assertEqual(self.drone.target_pub.messages[0].data,
                         [-0.7, 0, 1, -2, 9, 0, 0, 0, 0, 0, 0])
        sample = self.drone.control_pub.messages[0].data
        self.assertEqual(sample, [-0.7, 0, 1, -2, 3, 0, 0, 0, 0, 0, 0])
        self.assertFalse(self.drone._planning_active)
        self.drone.hover(0.2)
        self.assertEqual(len(self.drone.target_pub.messages), 1)
        self.assertEqual(self.drone.control_pub.messages[-1].data,
                         [-0.7, 0, 1, -2, 9, 0, 0, 0, 0, 0, 0])

    def test_planner_goto_waits_for_fc_round_trip_without_repeating_goal(self):
        self.drone.use_planner = True
        self.step = self.feedback  # No planner samples until the goal returns.
        with self.assertRaisesRegex(MissionError, 'Goto timed out'):
            self.drone.goto(1, -2, 3, -0.4, timeout=0.2)
        self.assertEqual(len(self.drone.target_pub.messages), 1)
        self.assertEqual(self.drone.target_pub.messages[0].data,
                         [0.4, 0, 1, 2, 3, 0, 0, 0, 0, 0, 0])
        self.assertEqual(self.drone.control_pub.messages, [])
        self.drone.hold_if_controlling()
        self.drone._publish_target(None)
        self.assertEqual(len(self.drone.target_pub.messages), 1)
        self.assertEqual(self.drone.control_pub.messages[-1].data[2:5], [0, 0, 0])

    def test_planner_does_not_publish_pose_goals(self):
        with patch.object(driver.rospy, 'get_param',
                          lambda name, default=None: True if name == '~use_planner' else default), \
                patch.object(driver.rospy, 'Publisher', side_effect=Publisher) as publishers:
            Drone()
        topics = [call.args[0] for call in publishers.call_args_list]
        self.assertIn('~mission_001', topics)
        self.assertIn('~control_001', topics)
        self.assertNotIn('~planner_goal', topics)
        self.assertNotIn('/move_base_simple/goal', topics)

    def test_planner_wait_does_not_publish_final_target_directly(self):
        self.planning()
        self.drone._publish_target(None)
        self.assertEqual(self.drone.control_pub.messages, [])
        self.assertEqual(self.drone.target_pub.messages, [])

    def test_planner_output_is_gated_during_idle_takeoff_and_landing(self):
        self.drone.use_planner = True
        self.drone._on_pos_cmd(self.planner_sample())
        self.assertEqual(self.drone.control_pub.messages, [])
        self.planning()
        self.step = lambda: self.drone._on_pos_cmd(self.planner_sample())
        self.drone.land(wait=0.2)
        self.assertEqual(self.drone.control_pub.messages, [])

    def test_cancel_planning_holds_pose_and_ignores_late_samples(self):
        self.planning()
        self.drone.cancel()
        self.drone._on_pos_cmd(self.planner_sample(x=99))
        self.drone._publish_target(None)
        self.assertEqual(len(self.drone.control_pub.messages), 1)
        self.assertEqual(self.drone.control_pub.messages[-1].data[2:5], [0, 0, 0])

    def test_planner_missing_output_fails_and_holds(self):
        self.planning()
        self.now += self.drone.planner_start_timeout + 0.1
        self.feedback(x=0.5)
        self.drone._publish_target(None)
        self.assertFalse(self.drone._planning_active)
        with self.assertRaisesRegex(MissionError, 'Planner output'):
            self.drone._check()
        self.assertEqual(self.drone.control_pub.messages[-1].data[2], 0.5)

    def test_planner_stale_stream_is_not_replayed(self):
        self.planning()
        self.drone._on_pos_cmd(self.planner_sample())
        self.now += self.drone.planner_timeout + 0.1
        self.feedback(x=0.4)
        self.drone._publish_target(None)
        self.assertEqual(len(self.drone.control_pub.messages), 2)
        self.assertEqual(self.drone.control_pub.messages[-1].data[2], 0.4)
        self.assertFalse(self.drone._planning_active)

    def test_planner_invalid_or_aborted_sample_is_not_forwarded(self):
        for sample in (self.planner_sample(x=float('nan')), self.planner_sample(flag=4)):
            self.planning()
            self.drone._planner_error = None
            self.drone._on_pos_cmd(sample)
            self.assertFalse(self.drone._planning_active)
            self.assertIsNotNone(self.drone._planner_error)
        self.assertEqual(self.drone.control_pub.messages, [])

    def test_planner_requires_mission_and_control_connections(self):
        self.drone.use_planner = True
        self.drone.control_pub.connections = 0
        with self.assertRaisesRegex(MissionError, 'control_001'):
            self.drone.prepare()
        with self.assertRaisesRegex(MissionError, 'control_001'):
            self.drone.goto(1, 2, 3)
        self.drone.control_pub.connections = 1
        self.drone.target_pub.connections = 0
        with self.assertRaisesRegex(MissionError, 'command/target'):
            self.drone.prepare()
        with self.assertRaisesRegex(MissionError, 'mission_001'):
            self.drone.goto(1, 2, 3)
        self.assertEqual(self.drone.target_pub.messages, [])
        self.assertEqual(self.drone.control_pub.messages, [])

    def test_launch_routes_goals_through_bridge_and_samples_through_control(self):
        root = ET.parse(Path(__file__).resolve().parents[2] / 'launch' / 'fcu_core.launch').getroot()
        self.assertIsNone(root.find("arg[@name='set_goal']"))
        bridge = root.find("node[@name='fcu_bridge_001']")
        mission = root.find("node[@name='fcu_auto_mission']")
        self.assertEqual(bridge.find("param[@name='set_goal']").get('value'), 'false')
        self.assertEqual(mission.find("param[@name='use_planner']").get('value'), 'false')
        self.assertEqual(bridge.find("remap[@from='~goal_001']").get('to'), '/move_base_simple/goal')
        self.assertIn('/fcu_auto_mission/mission_001', bridge.find("remap[@from='~mission_001']").get('to'))
        self.assertEqual(bridge.find("remap[@from='~control_001']").get('to'), '/fcu_auto_mission/control_001')
        self.assertIsNone(mission.find("remap[@from='~planner_goal']"))


if __name__ == "__main__":
    unittest.main()
