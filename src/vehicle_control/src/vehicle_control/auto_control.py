#!/usr/bin/env python3
import rospy
from carla import Location, Transform, Rotation, ActorBlueprint, Actor, WheelPhysicsControl, VehiclePhysicsControl, \
    command
from traffic_scenarios.utils.connect import Connect
from traffic_scenarios.models.scenario import get_current_scenario
from traffic_scenarios.helpers.location_helpers import is_at_end
import carla

TOLERANCE = 10
END = Location(x=71.362877, y=-7.414977, z=0.450000)
MAX_TIME = 120


def get_friction(precipitation):
    if 0 < precipitation < 20:
        return 1.0
    elif 20 <= precipitation < 50:
        return 0.8
    elif 50 <= precipitation < 80:
        return 0.6
    elif 80 <= precipitation < 100:
        return 0.4
    else:
        return 2.0


def set_tire_friction(vehicle: Actor, world):
    weather = world.get_weather()
    rain = weather.precipitation
    friction = get_friction(rain)

    # Create Wheels Physics Control
    front_left_wheel = carla.WheelPhysicsControl(tire_friction=friction, damping_rate=1.0, max_steer_angle=70.0,
                                                 radius=30.0)
    front_right_wheel = carla.WheelPhysicsControl(tire_friction=friction, damping_rate=1.5, max_steer_angle=70.0,
                                                  radius=25.0)
    rear_left_wheel = carla.WheelPhysicsControl(tire_friction=friction, damping_rate=0.2, max_steer_angle=0.0,
                                                radius=15.0)
    rear_right_wheel = carla.WheelPhysicsControl(tire_friction=friction, damping_rate=1.3, max_steer_angle=0.0,
                                                 radius=20.0)

    wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]

    # Change Vehicle Physics Control parameters of the vehicle
    physics_control = vehicle.get_physics_control()

    physics_control.torque_curve = [carla.Vector2D(x=0, y=400), carla.Vector2D(x=1300, y=600)]
    physics_control.max_rpm = 10000
    physics_control.moi = 1.0
    physics_control.damping_rate_full_throttle = 0.0
    physics_control.use_gear_autobox = True
    physics_control.gear_switch_time = 0.5
    physics_control.clutch_strength = 10
    physics_control.mass = 10000
    physics_control.drag_coefficient = 0.25
    physics_control.steering_curve = [carla.Vector2D(x=0, y=1), carla.Vector2D(x=100, y=1), carla.Vector2D(x=300, y=1)]
    physics_control.wheels = wheels
    vehicle.apply_physics_control(physics_control)


class AutoDrive:
    """Class that when called spawns cars on the road with a frequency between 0 and 100 where 100 is the max
    frequency """

    def __init__(self, connect: Connect):
        self._connect = connect
        self._world = self._connect.get_world()
        vehicle = self._connect.get_blueprint_lib().filter('vehicle.*')[0]
        self.spawn_point = Transform(Location(x=129.585846, y=-72.483788, z=8.000000),
                                     Rotation(pitch=0.000000, yaw=-178.490906, roll=0.000000))
        self.player = self._world.spawn_actor(vehicle, self.spawn_point)
        set_tire_friction(self.player, self._world)

    def main(self):
        # rospy.sleep(13)  # wait for carla to start
        self.player.set_autopilot(True)
        time = rospy.Time.now()
        while not rospy.is_shutdown():
            if time + rospy.Duration(MAX_TIME) < rospy.Time.now():
                rospy.logwarn("Time is up")
                return
            loc = self.player.get_location()
            if is_at_end(loc, END, tolerance=TOLERANCE):
                rospy.logwarn("Player is at end")
                return
            rospy.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node("car_spawning_node", anonymous=True)
    _connect = Connect()
    mc = AutoDrive(_connect)
    mc.main()
