#!/usr/bin/env python3

import rospy
import random
from traffic_scenarios.utils.connect import Connect
from carla import Location, Transform, Rotation
from traffic_scenarios.models.scenario import get_current_scenario


PEDESTRIAN_START_1 = Transform(Location(x=92.399376, y=-63.213696, z=10.673277),
                               Rotation(pitch=-28.841585, yaw=51.162003, roll=0.000086))

PEDESTRIAN_START_2 = Transform(Location(x=100.714096, y=-60.009426, z=10.660802),
                               Rotation(pitch=-13.440220, yaw=-76.404381, roll=0.000323))
PEDESTRIAN_START_3 = Transform(Location(x=96.509575, y=-65.188225, z=10.456425), Rotation(pitch=-4.503478, yaw=-130.127121, roll=0.000327))

STOP_1 = Location(x=116.000916, y=-124.104126, z=9.909193)
STOP_2 = Location(x=92.827477, y=-80.995079, z=9.568231)
STOP_3 = Location(x=96.721649, y=-89.021240, z=10.152753)
PED_STARTS = [PEDESTRIAN_START_1, PEDESTRIAN_START_2, PEDESTRIAN_START_3]
PED_STOPS = [STOP_1, STOP_2, STOP_3]
class CrosserScenario:
    def __init__(self, connect: Connect, frequency: int):
        if not (0 <= frequency <= 100):
            raise ValueError("Frequency must be between 0 and 100")
        rospy.sleep(5)  # wait for carla to start
        self._frequency = (-0.53 * frequency) + 60
        self._connect = connect
        self._world = connect.get_world()
        self._walker_bp = self._connect.get_blueprint_lib().filter('walker.pedestrian.*')
        self._controller_bp = self._connect.get_blueprint_lib().find('controller.ai.walker')

    def spawn_pedestrian(self, start, stop):
        walker = random.choice(self._walker_bp)
        actor = self._world.try_spawn_actor(walker, start)
        self._world.wait_for_tick()
        controller = self._world.spawn_actor(self._controller_bp, Transform(), actor)
        self._world.wait_for_tick()
        controller.start()
        controller.go_to_location(stop)

    def spawn_pedestrians(self):
        for i in range(len(PED_STARTS)):
            self.spawn_pedestrian(PED_STARTS[i], PED_STOPS[i])
            rospy.sleep(0.5)

    def main(self):
        if self._frequency == 0:
            return
        self.spawn_pedestrians()
        time = rospy.Time.now()
        while not rospy.is_shutdown():
            if time + rospy.Duration(self._frequency) < rospy.Time.now():
                time = rospy.Time.now()
                self.spawn_pedestrians()


if __name__ == '__main__':
    rospy.init_node("pedestrian_node", anonymous=True)
    _connect = Connect()
    scenario = get_current_scenario()
    frequency = scenario["people"]
    ws = CrosserScenario(_connect, frequency)
    ws.main()
    rospy.spin()
