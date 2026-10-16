"""Blocking user API; ROS callbacks and the target timer remain independent."""

import math
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Float32MultiArray, Int16, String


class MissionError(RuntimeError):
    pass


def number(value, name, minimum=None):
    value = float(value)
    if not math.isfinite(value) or (minimum is not None and value < minimum):
        raise ValueError("Invalid {}: {}".format(name, value))
    return value


class Drone:
    """Coordinates follow odom_global_001 (FLU); yaw is radians, positive left.

    goto completion checks XY only. Arm/takeoff/land/disarm send one command
    and wait a configured duration; no flight-state feedback is required.
    """

    def __init__(self):
        self._lock = threading.RLock()
        self._cancelled = threading.Event()
        self._pose = None
        self._odom_at = None
        self._target = None
        self.use_planner = rospy.get_param("~use_planner", False)
        self._planning_active = False
        self._planner_started = None
        self._planner_at = None
        self._planner_error = None
        self.planner_timeout = number(rospy.get_param("~planner_timeout", 2.0), "planner_timeout", 0.1)
        self.planner_start_timeout = number(rospy.get_param("~planner_start_timeout", 10.0), "planner_start_timeout", 0.1)
        self.xy_tolerance = number(rospy.get_param("~xy_tolerance", 0.15), "xy_tolerance", 0.001)
        self.settle_time = number(rospy.get_param("~settle_time", 0.5), "settle_time", 0)
        self.odom_timeout = number(rospy.get_param("~odom_timeout", 2.0), "odom_timeout", 0.1)
        self.action_timeout = number(rospy.get_param("~action_timeout", 30.0), "action_timeout", 0.1)
        self.takeoff_wait = number(rospy.get_param("~takeoff_wait", 5.0), "takeoff_wait", 0.1)
        self.arm_wait = number(rospy.get_param("~arm_wait", 2.0), "arm_wait", 0)
        self.land_wait = number(rospy.get_param("~land_wait", 15.0), "land_wait", 0)
        self.disarm_wait = number(rospy.get_param("~disarm_wait", 2.0), "disarm_wait", 0)
        rate = number(rospy.get_param("~target_publish_rate", 10.0), "target_publish_rate", 1)
        self.command_pub = rospy.Publisher("~command", Int16, queue_size=1)
        self.target_pub = rospy.Publisher("~mission_001", Float32MultiArray, queue_size=1)
        self.control_pub = rospy.Publisher("~control_001", Float32MultiArray, queue_size=1)
        self.goal_pub = rospy.Publisher("~planner_goal", PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)
        self._subscribers = [
            rospy.Subscriber("~odometry", Odometry, self._on_odom, queue_size=1),
        ]
        if self.use_planner:
            self._subscribers.append(rospy.Subscriber("~pos_cmd", PositionCommand, self._on_pos_cmd, queue_size=1))
        self._timer = rospy.Timer(rospy.Duration(1.0 / rate), self._publish_target)

    def report(self, message):
        self.status_pub.publish(String(data=message))
        rospy.loginfo("[auto mission] %s", message)

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6:
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        with self._lock:
            self._pose = (p.x, p.y, p.z, yaw)
            self._odom_at = time.monotonic()

    def _fresh(self, stamp, timeout):
        return stamp is not None and time.monotonic() - stamp <= timeout

    def _position_message(self, target):
        x, y, z, yaw = target
        return Float32MultiArray(data=[-yaw, 0.0, x, -y, z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def _on_pos_cmd(self, msg):
        with self._lock:
            self._check_planner()
            if (not self._planning_active or self._cancelled.is_set()
                    or not self._fresh(self._odom_at, self.odom_timeout)):
                return
            if msg.trajectory_flag in (4, 5, 6, 7):
                self._planner_error = "Planner rejected/aborted trajectory: {}".format(msg.trajectory_flag)
                self.hold_if_controlling()
                return
            target = (msg.position.x, msg.position.y, msg.position.z, msg.yaw)
            if not all(math.isfinite(v) for v in target):
                self._planner_error = "Planner sent non-finite position/yaw"
                self.hold_if_controlling()
                return
            self._planner_at = time.monotonic()
            # Forward each planner sample once; do not replay stale samples.
            self.control_pub.publish(self._position_message(target))

    def _check_planner(self):
        if self._planning_active:
            stamp = self._planner_started if self._planner_at is None else self._planner_at
            limit = self.planner_start_timeout if self._planner_at is None else self.planner_timeout
            if not self._fresh(stamp, limit):
                self._planner_error = "Planner output unavailable or stale"
                self.hold_if_controlling()

    def _check(self, require_odom=False):
        if self._cancelled.is_set() or rospy.is_shutdown():
            raise MissionError("Mission cancelled")
        with self._lock:
            self._check_planner()
            if self._planner_error:
                raise MissionError(self._planner_error)
            if require_odom and not self._fresh(self._odom_at, self.odom_timeout):
                raise MissionError("Odometry unavailable or stale")

    def prepare(self):
        """Called by the runner before each mission; never resumes an old target."""
        with self._lock:
            self._cancelled.clear()
            self._target = None
            self._planning_active = False
            self._planner_error = None
        self._check(require_odom=True)
        if self.command_pub.get_num_connections() == 0 or self.target_pub.get_num_connections() == 0:
            raise MissionError("bridge_001 command/target subscribers are not connected")
        if self.use_planner and self.control_pub.get_num_connections() == 0:
            raise MissionError("bridge_001 control_001 subscriber is not connected")

    def _wait(self, predicate, timeout, label, require_odom=False):
        deadline = time.monotonic() + number(timeout, "timeout", 0.01)
        while True:
            self._check(require_odom=require_odom)
            with self._lock:
                if predicate():
                    return
            if time.monotonic() >= deadline:
                raise MissionError("{} timed out".format(label))
            time.sleep(0.05)

    def _send(self, command):
        # Non-latched commands; never blindly retry takeoff/land.
        with self._lock:
            self._check()
            self.command_pub.publish(Int16(data=command))

    def _timed_command(self, command, duration, label):
        duration = number(duration, "wait", 0)
        self._check()
        self.report("{}: command sent, wait {} seconds (no state verification)".format(label, duration))
        with self._lock:
            self._check()
            self._target = None
            self._planning_active = False
            self._send(command)
        until = time.monotonic() + duration
        self._wait(lambda: time.monotonic() >= until, duration + 1, label + " wait")

    def arm(self, wait=None):
        self._timed_command(1, self.arm_wait if wait is None else wait, "ARM")

    def takeoff(self, wait=None):
        self._timed_command(3, self.takeoff_wait if wait is None else wait, "TAKEOFF")

    def goto(self, x, y, z, yaw=0.0, timeout=None):
        target = tuple(number(v, key) for key, v in zip(("x", "y", "z", "yaw"), (x, y, z, yaw)))
        timeout = number(self.action_timeout if timeout is None else timeout, "timeout", 0.01)
        self._check(require_odom=True)
        self.report("GOTO {} (XY arrival only)".format(target))
        with self._lock:
            self._check(require_odom=True)
            self._target = target
            if self.use_planner:
                if self.goal_pub.get_num_connections() == 0:
                    self._target = None
                    raise MissionError("Planner goal subscriber is not connected")
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = target[:3]
                goal.pose.orientation.z = math.sin(target[3] / 2)
                goal.pose.orientation.w = math.cos(target[3] / 2)
                self._planner_started = time.monotonic()
                self._planner_at = None
                self._planning_active = True
                self.goal_pub.publish(goal)
        stable_since = [None]

        def reached():
            distance = math.hypot(self._pose[0] - target[0], self._pose[1] - target[1])
            if distance > self.xy_tolerance:
                stable_since[0] = None
                return False
            if stable_since[0] is None:
                stable_since[0] = time.monotonic()
            return time.monotonic() - stable_since[0] >= self.settle_time

        self._wait(reached, timeout, "Goto",
                   require_odom=True)
        with self._lock:
            self._planning_active = False

    def hover(self, seconds):
        duration = number(seconds, "seconds", 0)
        self._check(require_odom=True)
        self.report("HOVER {} seconds".format(duration))
        with self._lock:
            self._check(require_odom=True)
            if self._target is None:
                self._target = self._pose
        until = time.monotonic() + duration
        self._wait(lambda: time.monotonic() >= until, duration + 1, "Hover",
                   require_odom=True)

    def land(self, wait=None):
        self._timed_command(4, self.land_wait if wait is None else wait, "LAND")

    def disarm(self, wait=None):
        self._timed_command(2, self.disarm_wait if wait is None else wait, "DISARM")

    def cancel(self):
        with self._lock:
            self._cancelled.set()
            self.hold_if_controlling()

    def hold_if_controlling(self):
        """On failure/cancel hold current pose only when position control was active.

        During takeoff/landing there is no position target to override the FC mode.
        Stale feedback suppresses publishing; FC failsafe remains responsible.
        """
        with self._lock:
            self._planning_active = False
            if self._target is not None:
                self._target = self._pose if self._fresh(self._odom_at, self.odom_timeout) else None

    def _publish_target(self, _event):
        with self._lock:
            self._check_planner()
            if (self._target is None
                    or not self._fresh(self._odom_at, self.odom_timeout)):
                return
            if self._planning_active:
                return  # Only the planner callback supplies in-flight position samples.
            # ROS FLU -> bridge position FRU and yaw FRD. No velocity/acceleration API.
            publisher = self.control_pub if self.use_planner else self.target_pub
            publisher.publish(self._position_message(self._target))
