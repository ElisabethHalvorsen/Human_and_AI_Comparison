#!/usr/bin/env python3
import rospy
from carla import Location, Transform, Rotation
import random
from traffic_scenarios.utils.connect import Connect
from traffic_scenarios.models.scenario import get_current_scenario

TOLERANCE = 10

# TODO: put in yaml file
# TODO: remove all cars every n seconds
# TODO: change angle off sun with respect to intensity
RIGHT_START = Transform(Location(x=81.236206, y=-121.202599, z=9.501081),
                        Rotation(pitch=-13.459345, yaw=93.259354, roll=0.000039))

LEFT_START = Transform(Location(x=111.329628, y=-7.114077, z=1.522039),
                       Rotation(pitch=-29.011528, yaw=-179.341934, roll=-0.000610))


def run(car: list):
    if car is not None:
        car.set_autopilot(True)


class MovingCars:
    """Class that when called spawns cars on the road with a frequency between 0 and 100 where 100 is the max
    frequency """

    def __init__(self, connect: Connect, frequency: int):
        if not (0 <= frequency <= 100):
            raise ValueError("Frequency must be between 0 and 100")
        self._connect = connect
        if frequency == 0:
            self._frequency = 0
        else:
            self._frequency = 10 * (1 + (1 - frequency / 100))
        self.vehicle_choices = self._connect.get_blueprint_lib().filter('vehicle.*')
        self.current = 0
        self._cars = []

    def spawn_actor(self, transform: Transform, bp):
        return self._connect.get_world().try_spawn_actor(bp, transform)

    def add_car(self):
        bp = random.choice(self.vehicle_choices)
        car = self.spawn_actor(RIGHT_START, bp)
        self._cars.append(car)

    def spawn_left_and_right_car(self):
        car = self.spawn_actor(RIGHT_START, random.choice(self.vehicle_choices))
        car2 = self.spawn_actor(LEFT_START, random.choice(self.vehicle_choices))
        rospy.sleep(0.5)
        run(car)
        rospy.sleep(0.5)
        run(car2)

    def main(self):
        if self._frequency == 0:
            return
        self.spawn_left_and_right_car()
        time = rospy.Time.now()

        while not rospy.is_shutdown():
            if time + rospy.Duration(self._frequency) < rospy.Time.now():
                self.spawn_left_and_right_car()
                time = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node("car_spawning_node", anonymous=True)
    _connect = Connect()
    scenario = get_current_scenario()
    frequency = scenario["cars"]
    mc = MovingCars(_connect, frequency)
    mc.main()
    rospy.spin()
