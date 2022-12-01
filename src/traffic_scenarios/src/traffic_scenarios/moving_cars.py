#!/usr/bin/env python3

import rospy
from carla import Location, Client, Transform, Rotation, World
import random

TOLERANCE = 10

# ! initial idea was to remove the cars but carla gives among other things segmentation errors
# TODO: put in yaml file
RIGHT_START = Transform(Location(x=81.236206, y=-121.202599, z=9.501081),
                        Rotation(pitch=-13.459345, yaw=93.259354, roll=0.000039))

# RIGHT_END = Transform(Location(x=78.129173, y=-34.854458, z=3.451789), Rotation(pitch=-15.281154, yaw=91.382317, roll=0.000035))

LEFT_START = Transform(Location(x=111.329628, y=-7.114077, z=1.522039),
                       Rotation(pitch=-29.011528, yaw=-179.341934, roll=-0.000610))


# Transform(Location(x=83.294762, y=-61.189644, z=8.028113), Rotation(pitch=1.039521, yaw=-93.168274, roll=0.001194))
# Transform(Location(x=81.152046, y=-8.710766, z=1.173693),
# Rotation(pitch=-0.262354, yaw=-88.032150, roll=0.003327))


# LEFT_END = Transform(Location(x=84.823921, y=-113.856148, z=9.510290),
#                       Rotation(pitch=-0.611664, yaw=-89.593483, roll=0.003309))


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



class MovingCars:
    """Class that when called spawns cars on the road with a frequency between 0 and 100 where 100 is the max
    frequency """
    def __init__(self, connect: Connect, frequency: int):
        if not (0 <= frequency <= 100):
            raise ValueError("Frequency must be between 0 and 100")
        self._connect = connect
        self._frequency = 10*(1+(1-frequency/100))
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
        time = rospy.Time.now()
        run(car)
        while not rospy.is_shutdown():
            if time + rospy.Duration(self._frequency) < rospy.Time.now():
                time = rospy.Time.now()
                car = self.spawn_actor(RIGHT_START, random.choice(self.vehicle_choices))
                rospy.sleep(1)  # let actor spawn
                run(car)


# TODO: objective function
# Sun = [0, 100]
# Rain = [0, 100]
# Biker driving over [0, 100] -> 100 drives over without notice, 50 stops and then drives over, 0 no driver
# TODO: Define safety targets
# TODO: Measure where on road car must be, get points every second of a perfect drive
# TODO: how far away is the car from other cars

if __name__ == '__main__':
    rospy.init_node("moving_cars_node", anonymous=True)
    _connect = Connect()
    # print(get_current_location_carla(_connect))
    mc = MovingCars(_connect, -1)
    mc.main()
    #

    actor_list = _connect.get_world().get_actors()
    # Print the location of all the speed limit signs in the world.

    # for a in actor_list.filter('vehicle.*'):
    #     a.set_autopilot(False)
    #     a.destroy()
