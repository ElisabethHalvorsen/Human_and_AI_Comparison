#!/usr/bin/env python3

import rospy
import random
from traffic_scenarios.utils.connect import Connect
from carla import Location, Transform, Rotation

PEDESTRIAN_START = Transform(Location(x=92.980034, y=-69.512383, z=8.472011),
                             Rotation(pitch=-1.411739, yaw=-85.510864, roll=0.000913))


class CrosserScenario:
    def __init__(self, connect: Connect, frequency: int):
        if not (0 <= frequency <= 100):
            raise ValueError("Frequency must be between 0 and 100")
        if frequency == 0:
            self._frequency = 0
        else:
            self._frequency = 50 * (1 + (1 - frequency / 100))
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
        if self._frequency == 0:
            return
        self.spawn_pedestrian()
        time = rospy.Time.now()
        while not rospy.is_shutdown():
            if time + rospy.Duration(self._frequency) < rospy.Time.now():
                time = rospy.Time.now()
                self.spawn_pedestrian()


if __name__ == '__main__':
    rospy.init_node("pedestrian_node", anonymous=True)
    _connect = Connect()
    ws = CrosserScenario(_connect, 0)
    ws.main()
    rospy.spin()
