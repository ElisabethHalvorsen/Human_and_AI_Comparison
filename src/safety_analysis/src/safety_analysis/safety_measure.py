#!/usr/bin/env python3
import rospy
from traffic_scenarios.utils.connect import Connect

from carla import ActorBlueprint, Actor
import pandas as pd
from math import sqrt
from traffic_scenarios.create_optimal_path import WaypointPath, START, LEFT_END, RIGHT_END
import sys
from datetime import datetime as dt
import os
import rospkg
from traffic_scenarios.models.scenario import get_current_scenario

SCENARIO_GEN_PATH = os.path.join(rospkg.RosPack().get_path('safety_analysis'), 'src', 'safety_analysis', 'data')


class SafetyMeasure:
    def __init__(self):
        rospy.sleep(5)  # wait for carla to start
        self._connect = Connect()
        self._world = self._connect.get_world()
        vehicles = self._connect.get_blueprint_lib().filter('vehicle.*')
        self.player = self.get_player_id(vehicles[0])
        path = WaypointPath(self._connect)
        self._pp = path.get_waypoint_path(START, LEFT_END)[2:85]

    @staticmethod
    def get_scenario_name():
        scenario = get_current_scenario()
        name = ''
        for s in scenario:
            name += f'{s}_{scenario[s]}_'
        return name[:-1]
    def get_player_id(self, player_bp: ActorBlueprint) -> Actor:
        actors = self._world.get_actors()
        for a in actors:
            if a.type_id == player_bp.id:
                return a

    @staticmethod
    def get_euclidean_distance(p1, p2):
        return sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

    def get_closest_transform(self):
        # w = Waypoint()
        min_dist = sys.maxsize
        id = 0
        try:
            player_location = self.player.get_location()
            player_rot = self.player.get_transform().rotation
        except AttributeError:
            raise Exception("Have you remembered to spawn the player?")

        for i, point in (enumerate(self._pp)):
            dist = self.get_euclidean_distance(player_location, point[0].transform.location)
            if dist < min_dist:
                min_dist = dist
                id = i
        print("Closest point: ", self._pp[id][0].transform.location, "Distance: ", min_dist, "ID: ", id)
        return self._pp[id][0].transform, min_dist, player_rot

    def main(self):
        while not rospy.is_shutdown():
            pp_trans, dist, player_rot = self.get_closest_transform()
            df_outcome = pd.DataFrame({'Time': [dt.now()], 'Distance': [dist]})
            name_id = self.get_scenario_name()
            out_name = f'{SCENARIO_GEN_PATH}/{name_id}.csv'
            if os.path.exists(out_name):
                df_outcome.to_csv(out_name, index=False, mode='a', header=False)
            else:
                df_outcome.to_csv(out_name, index=False, mode='w')
            rospy.sleep(0.5)
        # print(self.player)


if __name__ == '__main__':
    rospy.init_node("safety_measure_node", anonymous=True)
    s = SafetyMeasure()
    s.main()
