"""Edit this function to compose your own mission. Coordinates: metres/radians.

The example starts ONLY after calling /fcu_auto_mission/start.
The takeoff command uses the flight controller's configured takeoff behaviour.
goto checks XY only, even when z or yaw changes.
"""


def mission(drone):
    drone.arm()
    drone.takeoff()
    drone.goto(x=3.0, y=0.0, z=1.0, yaw=0.0)
    drone.hover(seconds=5.0)
    drone.goto(x=3.0, y=1.0, z=1.0, yaw=0.0)
    drone.hover(seconds=5.0)
    drone.land()  # Uses land_wait; no landing-state feedback is checked.
    drone.disarm()
