#!/usr/bin/env python3
import enum

import rospy
from carla import Location, Client, Transform, Rotation, World
import random
import carla

TOLERANCE = 10

# TODO: put in yaml file
# TODO: remove all cars every n seconds
# TODO: change angle off sun with respect to intensity
RIGHT_START = Transform(Location(x=81.236206, y=-121.202599, z=9.501081),
                        Rotation(pitch=-13.459345, yaw=93.259354, roll=0.000039))

LEFT_START = Transform(Location(x=111.329628, y=-7.114077, z=1.522039),
                       Rotation(pitch=-29.011528, yaw=-179.341934, roll=-0.000610))

PEDESTRIAN_START = Transform(Location(x=92.980034, y=-69.512383, z=8.472011),
                             Rotation(pitch=-1.411739, yaw=-85.510864, roll=0.000913))


class Connect:
    def __init__(self):
        self._client = Client('localhost', 2000)
        self._world = self._client.get_world()
        self._bp_lib = self._world.get_blueprint_library()
        self._spawn_points = self._world.get_map().get_spawn_points()

    def get_client(self) -> Client:
        return self._client

    def get_world(self) -> World:
        return self._world

    def get_blueprint_lib(self):
        return self._bp_lib

    def get_spawn_points(self):
        return self._spawn_points


def is_at_end(loc: Location, end: Location) -> bool:
    return (end.x - TOLERANCE < loc.x < end.x + TOLERANCE) and (end.y - TOLERANCE < loc.y < end.y + TOLERANCE)


def get_current_location_carla(connect: Connect):
    spectator = connect.get_world().get_spectator()
    return spectator.get_transform()


def destroy_car(car):
    car.set_autopilot(False)
    car.destroy()


def run(car: list):
    if car is not None:
        car.set_autopilot(True)


def destroy_all_vehicles(connect: Connect):
    actor_list = connect.get_world().get_actors()
    for a in actor_list.filter('vehicle.*'):
        a.set_autopilot(False)
        a.destroy()


def destroy_all_pedestrians(connect: Connect):
    actor_list = connect.get_world().get_actors()
    for a in actor_list.filter('walker.pedestrian.*'):
        a.destroy()


class MovingCars:
    """Class that when called spawns cars on the road with a frequency between 0 and 100 where 100 is the max
    frequency """

    def __init__(self, connect: Connect, frequency: int):
        if not (0 <= frequency <= 100):
            raise ValueError("Frequency must be between 0 and 100")
        self._connect = connect
        self._frequency = 10 * (1 + (1 - frequency / 100))
        self.vehicle_choices = self._connect.get_blueprint_lib().filter('vehicle.*')
        self.current = 0
        self._cars = []
        print(self._frequency)

    def spawn_actor(self, transform: Transform, bp):
        return self._connect.get_world().try_spawn_actor(bp, transform)

    def add_car(self):
        bp = random.choice(self.vehicle_choices)
        car = self.spawn_actor(RIGHT_START, bp)
        self._cars.append(car)

    def main(self):
        car = self.spawn_actor(RIGHT_START, random.choice(self.vehicle_choices))
        car2 = self.spawn_actor(LEFT_START, random.choice(self.vehicle_choices))
        time = rospy.Time.now()
        rospy.sleep(0.5)
        run(car)
        rospy.sleep(0.5)
        run(car2)
        while not rospy.is_shutdown():
            if time + rospy.Duration(self._frequency) < rospy.Time.now():
                time = rospy.Time.now()
                car = self.spawn_actor(RIGHT_START, random.choice(self.vehicle_choices))
                rospy.sleep(0.5)  # let actor spawn
                run(car)
                car2 = self.spawn_actor(LEFT_START, random.choice(self.vehicle_choices))
                rospy.sleep(0.5)  # let actor spawn
                run(car2)


class Weather(enum.Enum):
    FOGGY = 0
    SUNNY = 1
    RAINY = 2


class WeatherScenario:
    def __init__(self, connect: Connect, weather: Weather, intensity: int):
        if not (0 <= intensity <= 100):
            raise ValueError("Frequency must be between 0 and 100")
        self._weather = weather
        self._connect = connect
        self._intensity = intensity

    # TODO: change tire friction
    def get_rain_parameters(self):
        w = carla.WeatherParameters()
        w.precipitation = self._intensity
        w.wetness = self._intensity
        w.cloudiness = 40
        return w

    def get_fogg_parameters(self):
        w = carla.WeatherParameters()
        w.fog_density = self._intensity
        return w

    def get_sun_parameters(self):
        w = carla.WeatherParameters()
        w.sun_altitude_angle = 17.0
        w.sun_azimuth_angle = 5.0
        return w

    def main(self):
        weather = carla.WeatherParameters()
        if self._weather == Weather.FOGGY:
            weather = self.get_fogg_parameters()
        elif self._weather == Weather.SUNNY:
            weather = self.get_sun_parameters()
        elif self._weather == Weather.RAINY:
            weather = self.get_rain_parameters()
        self._connect.get_world().set_weather(weather)


class CrosserScenario:
    def __init__(self, connect: Connect, frequency: int):
        if not (0 <= frequency <= 100):
            raise ValueError("Frequency must be between 0 and 100")
        self._frequency = 100 * (1 + (1 - frequency / 100))
        self._connect = connect
        self._world = connect.get_world()
        self._walker_bp = self._connect.get_blueprint_lib().filter('walker.pedestrian.*')
        self._controller_bp = self._connect.get_blueprint_lib().find('controller.ai.walker')

    def spawn_pedestrian(self):
        walker = random.choice(self._walker_bp)
        actor = self._world.try_spawn_actor(walker, PEDESTRIAN_START)
        self._world.wait_for_tick()
        controller = self._world.spawn_actor(self._controller_bp, Transform(), actor)
        self._world.wait_for_tick()
        controller.start()
        controller.go_to_location(Location(x=116.000916, y=-124.104126, z=9.909193))

    def main(self):
        self.spawn_pedestrian()
        time = rospy.Time.now()
        while not rospy.is_shutdown():
            if time + rospy.Duration(self._frequency) < rospy.Time.now():
                time = rospy.Time.now()
                self.spawn_pedestrian()


# TODO: objective function
# TODO: Define safety targets
# TODO: Measure where on road car must be, get points every second of a perfect drive
# TODO: how far away is the car from other cars

if __name__ == '__main__':
    rospy.init_node("moving_cars_node", anonymous=True)
    _connect = Connect()
    print(get_current_location_carla(_connect))
    destroy_all_pedestrians(_connect)
    # print(tf)

    # mc = MovingCars(_connect, 100)
    # mc.main()

    # ws = WeatherScenario(_connect, Weather.SUNNY, 100)
    # ws.main()

    cs = CrosserScenario(_connect, 100)
    cs.main()
