#!/usr/bin/env python3
import rospy
from traffic_scenarios.utils.connect import Connect

from carla import ActorBlueprint, Actor, Waypoint
import pandas as pd
from math import sqrt
from traffic_scenarios.create_optimal_path import WaypointPath, START, LEFT_END, RIGHT_END
import sys
import numpy as np
import transformations as tr
from datetime import datetime as dt
import os
import rospkg
SCENARIO_GEN_PATH = os.path.join(rospkg.RosPack().get_path('safety_analysis'), 'src', 'safety_analysis', 'data')
class SafetyMeasure:
    def __init__(self):
        self._connect = Connect()
        self._world = self._connect.get_world()
        vehicles = self._connect.get_blueprint_lib().filter('vehicle.*')
        self.player = self.get_player_id(vehicles[0])
        path = WaypointPath(self._connect)
        self._pp = path.get_waypoint_path(START, LEFT_END)[2:85]

    def get_player_id(self, player_bp: ActorBlueprint) -> Actor:
        actors = self._world.get_actors()
        for a in actors:
            if a.type_id == player_bp.id:
                return a

    @staticmethod  # DOESNT WORK
    def get_geodesic_distance(rot1, rot2):
        quat1 = tr.quaternion_from_euler(np.radians(rot1.roll), np.radians(rot1.pitch), np.radians(rot1.yaw), 'sxyz')
        quat2 = tr.quaternion_from_euler(np.radians(rot2.roll), np.radians(rot2.pitch), np.radians(rot2.yaw), 'sxyz')
        print("Input angles (radians):", np.radians(rot1.roll), np.radians(rot1.pitch), np.radians(rot1.yaw))
        print("Resulting quaternion:", quat1)
        R1 = tr.quaternion_matrix(quat1)[:3, :3]
        R2 = tr.quaternion_matrix(quat2)[:3, :3]
        R = np.dot(R2, np.linalg.inv(R1))
        angle, axis, _ = tr.rotation_from_matrix(R)
        return np.degrees(angle)

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
            angle = 1#self.get_geodesic_distance(pp_trans.rotation, player_rot)
            df_outcome = pd.DataFrame({'Time': [dt.now()], 'Distance': [dist], 'Angle': [angle]})
            out_name = f'{SCENARIO_GEN_PATH}/safety_analysis.csv'
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
