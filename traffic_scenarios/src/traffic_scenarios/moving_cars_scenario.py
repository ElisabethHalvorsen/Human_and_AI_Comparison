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
RIGHT_START_1 = Transform(Location(x=80.600075, y=-101.315620, z=8.996277),
          Rotation(pitch=-4.953977, yaw=91.457039, roll=0.000357))

RIGHT_START_2 = Transform(Location(x=80.641212, y=-107.752243, z=9.444014), Rotation(pitch=-6.322911, yaw=87.704124, roll=0.000361))

RIGHT_START_3 = Transform(Location(x=81.236206, y=-121.202599, z=9.501081),
                        Rotation(pitch=-13.459345, yaw=93.259354, roll=0.000039))

LEFT_START = Transform(Location(x=111.329628, y=-7.114077, z=1.522039),
                       Rotation(pitch=-29.011528, yaw=-179.341934, roll=-0.000610))

LEFT_START_2 = Transform(Location(x=121.496513, y=-7.414468, z=1.234665),
                         Rotation(pitch=-2.658173, yaw=-177.537262, roll=0.000368))

LEFT_START_3 = Transform(Location(x=132.927795, y=-6.922813, z=1.765878),
                         Rotation(pitch=-7.909851, yaw=-179.484848, roll=0.000368))

class MovingCars:
    """Class that when called spawns cars on the road with a frequency between 0 and 100 where 100 is the max
    frequency """

    def __init__(self, connect: Connect, frequency: int):
        if not (0 <= frequency <= 100):
            raise ValueError("Frequency must be between 0 and 100")
        rospy.sleep(5)  # wait for carla to start
        self._connect = connect
        self._client = self._connect.get_client()
        self._frequency = (-0.53 * frequency) + 60
        vehicles = self._connect.get_blueprint_lib().filter('vehicle.*')
        tmp_vehicle_choices = [v if idx != 0 else None for idx, v in enumerate(vehicles)]
        self.vehicle_choices = tmp_vehicle_choices[1:]
        self.current = 0
        self._cars = []
        self._tm = self._client.get_trafficmanager()

    def set_dangerous_behaviour(self, car):
        self._tm.distance_to_leading_vehicle(car, 0)
        self._tm.vehicle_percentage_speed_difference(car, -30)
        self._tm.ignore_vehicles_percentage(car, 50)
        self._tm.keep_right_rule_percentage(car, 0)

    def run(self, cars: list, tm_port):
        for car in cars:
            if car is not None:
                car.set_autopilot(True, tm_port)
                rospy.sleep(0.5)
                self.set_dangerous_behaviour(car)
                print("setting behaviour")

    def spawn_actor(self, transform: Transform, bp):
        return self._connect.get_world().try_spawn_actor(bp, transform)

    def spawn_left_and_right_car(self):
        carr1 = self.spawn_actor(RIGHT_START_1, random.choice(self.vehicle_choices))
        carr2 = self.spawn_actor(RIGHT_START_2, random.choice(self.vehicle_choices))
        carr3 = self.spawn_actor(RIGHT_START_3, random.choice(self.vehicle_choices))

        carl1 = self.spawn_actor(LEFT_START, random.choice(self.vehicle_choices))
        carl2 = self.spawn_actor(LEFT_START_2, random.choice(self.vehicle_choices))
        carl3 = self.spawn_actor(LEFT_START_3, random.choice(self.vehicle_choices))

        cars = [carr1, carr2, carr3, carl1, carl2, carl3]
        self.run(cars, self._tm.get_port())
        # rospy.sleep(0.5)
        # port = self._tm.get_port()
        # run(carl1, port)
        # rospy.sleep(0.5)
        # run(carr1, port)
        # rospy.sleep(0.5)
        # run(carl2, port)
        # rospy.sleep(0.5)
        # run(carr2, port)
        # rospy.sleep(0.5)
        # run(carl3, port)
        # rospy.sleep(0.5)
        # run(carr3, port)

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
