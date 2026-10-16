#!/usr/bin/env python3
"""Load a trusted local mission(drone) function and run it in a worker thread.

The entry filename differs from the fcu_auto_mission package to avoid shadowing
that package when rosrun executes this file directly from the source tree.
"""

import importlib.util
import os
import sys
import threading

import rospy
from std_srvs.srv import Trigger, TriggerResponse

# Source/devel execution: expose the parent src directory. Installed execution:
# the package is installed beside the executable and already on sys.path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from fcu_auto_mission import Drone


class MissionRunner:
    def __init__(self):
        path = os.path.abspath(os.path.expanduser(rospy.get_param("~mission_file")))
        spec = importlib.util.spec_from_file_location("user_fcu_mission", path)
        if spec is None or spec.loader is None:
            raise ValueError("Cannot load mission file: {}".format(path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.mission = getattr(module, "mission", None)
        if not callable(self.mission):
            raise ValueError("Mission file must define mission(drone)")
        self.drone = Drone()
        self._lock = threading.Lock()
        self._worker = None
        self._start_service = rospy.Service("~start", Trigger, self.start)
        self._cancel_service = rospy.Service("~cancel", Trigger, self.cancel)
        rospy.on_shutdown(self.drone.cancel)
        self.drone.report("READY: call ~start to execute the loaded mission")

    def start(self, _request):
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                return TriggerResponse(success=False, message="A mission is already running")
            try:
                self.drone.prepare()
            except Exception as exc:
                return TriggerResponse(success=False, message=str(exc))
            self._worker = threading.Thread(target=self._run, name="fcu-mission", daemon=True)
            self._worker.start()
        return TriggerResponse(success=True, message="Mission started; observe ~status for result")

    def cancel(self, _request):
        with self._lock:
            self.drone.cancel()
        self.drone.report("CANCEL_REQUESTED: no automatic landing or disarm")
        return TriggerResponse(success=True, message="Cancellation requested")

    def _run(self):
        try:
            self.drone.report("RUNNING")
            self.mission(self.drone)
            self.drone._check()
            self.drone.report("SUCCEEDED: program finished; timed commands were not state-verified")
        except Exception as exc:
            self.drone.hold_if_controlling()
            self.drone.report("FAILED: {}".format(exc))
            rospy.logerr("Mission ended: %s", exc)


if __name__ == "__main__":
    rospy.init_node("fcu_auto_mission")
    try:
        runner = MissionRunner()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("Cannot start auto mission node: %s", exc)
        raise
