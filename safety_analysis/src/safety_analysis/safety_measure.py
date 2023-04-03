#!/usr/bin/env python3

import rospy
from traffic_scenarios.utils.connect import Connect
from carla import ActorBlueprint, Actor, Transform, Location
import pandas as pd
from math import sqrt
from traffic_scenarios.create_optimal_path import WaypointPath, START, LEFT_END, RIGHT_END
import sys
from datetime import datetime as dt
import os
import rospkg
from traffic_scenarios.models.scenario import get_current_scenario
from safety_analysis.rss.rss_sensor import RssStateInfo
from traffic_scenarios.helpers.location_helpers import is_at_end

TOLERANCE = 10
END = Location(x=71.362877, y=-7.414977, z=0.450000)
SCENARIO_GEN_PATH = os.path.join(rospkg.RosPack().get_path('safety_analysis'), 'src', 'safety_analysis', 'data')

class SafetyMeasure:
    def __init__(self):
        rospy.sleep(5)  # wait for carla to start
        self._connect = Connect()
        self._world = self._connect.get_world()
        self._bp = self._connect.get_blueprint_lib()
        vehicles = self._connect.get_blueprint_lib().filter('vehicle.*')
        self.player = self.get_player_id(vehicles[0])
        # print("player:", self.player, '+'*50)
        path = WaypointPath(self._connect)
        self._pp = path.get_waypoint_path(START, LEFT_END)[2:85]
        collision_bp = self._bp.find('sensor.other.collision')
        self._collision_sensor = self._world.spawn_actor(collision_bp, Transform(), attach_to=self.player)
        self._collision_sensor.listen(lambda event: self.collision_callback(event, None))
        self.collided = False
        rss_bp = self._bp.find('sensor.other.rss')
        self._rss_sensor = self._world.spawn_actor(rss_bp, Transform(Location(x=0.0,z=0.0)), attach_to=self.player)
        self._rss_sensor.listen(self._on_rss_response)
        self.rss_res = None
        self._state_info = None

    def _on_rss_response(self, res):  # res = RssResponse
        if res.response_valid:
            proper_response = res.proper_response  # ProperResponse
            rss_state_snapshot = res.rss_state_snapshot  # RssStateSnapshot
            ego_dynamics_on_route = res.ego_dynamics_on_route  # RssEgoDynamicsOnRoute
            world_model = res.world_model  # WorldModel
            situation_snapshot = res.situation_snapshot  # SituationSnapshot

            new_states = []
            for rss_state in rss_state_snapshot.individualResponses:
                new_states.append(RssStateInfo(rss_state, ego_dynamics_on_route, world_model))

            self.rss_res = new_states
        else:
            self.rss_res = None
            rospy.logwarn("RSS response was not valid")
            return

    @staticmethod
    def get_scenario_name():
        scenario = get_current_scenario()
        name = ''
        for s in scenario:
            if isinstance(scenario[s], dict):
                for s2 in scenario[s]:
                    name += f'{s2}_{scenario[s][s2]}_'
            else:
                name += f'{s}_{str(scenario[s])}_'
        name = name[:-1]
        name += f'_daytime_{dt.now().strftime("%d-%m-%Y_%H:%M:%S")}'
        return name

    def collision_callback(self, event, data_dict):
        self.collided = True

    def get_player_id(self, player_bp: ActorBlueprint) -> Actor:
        actors = self._world.get_actors()
        for a in actors:
            if a.type_id == player_bp.id:
                return a
        else:
            print("no player")
            return None

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
        # print("Closest point: ", self._pp[id][0].transform.location, "Distance: ", min_dist, "ID: ", id)
        return self._pp[id][0].transform, min_dist, player_rot

    def main(self):
        name_id = self.get_scenario_name()
        out_name = f'{SCENARIO_GEN_PATH}/{name_id}.csv'
        while not rospy.is_shutdown():
            player_loc = self.player.get_transform().location
            pp_trans, dist, player_rot = self.get_closest_transform()
            if self.rss_res:
                for i in self.rss_res:
                    df_outcome = pd.DataFrame({'Time': [dt.now()],
                                               'Distance path': [dist],
                                               'Collided': self.collided,
                                               'Lateral Response': [i.get_lateral_margin()],
                                               'Distance Actor':[i.get_distance()],
                                               'Stopping Distance':i.get_min_stopping_dist(),
                                               'Crossing Border': i.get_is_crossing_border(),
                                               'Dangerous': i.get_is_dangerous(),
                                               'Player Loc X': player_loc.x,
                                               'Player Loc Y': player_loc.y,
                                               'Player Loc Z': player_loc.z,
                                               'Speed player X': self.player.get_velocity().x,
                                               'Speed player Y': self.player.get_velocity().y,
                                               'Speed player Z': self.player.get_velocity().z})
                    if os.path.exists(out_name):
                        df_outcome.to_csv(out_name, index=False, mode='a', header=False)
                    else:
                        df_outcome.to_csv(out_name, index=False, mode='w')
            else:
                df_outcome = pd.DataFrame({'Time': [dt.now()],
                                           'Distance path': [dist],
                                           'Collided': self.collided,
                                           'Lateral Response': [None],
                                           'Distance Actor': [None],
                                           'Stopping Distance': None,
                                           'Crossing Border': None,
                                           'Dangerous': None,
                                           'Player Loc X': player_loc.x,
                                           'Player Loc Y': player_loc.y,
                                           'Player Loc Z': player_loc.z,
                                           'Speed player X': self.player.get_velocity().x,
                                           'Speed player Y': self.player.get_velocity().y,
                                           'Speed player Z': self.player.get_velocity().z})
                if os.path.exists(out_name):
                    df_outcome.to_csv(out_name, index=False, mode='a', header=False)
                else:
                    df_outcome.to_csv(out_name, index=False, mode='w')
            if self.collided:
                rospy.logwarn("The car collided")
                rospy.logwarn("Stopping the run")
                return
            loc = self.player.get_location()
            if is_at_end(loc, END, tolerance=TOLERANCE):
                rospy.logwarn("Player is at end")
                return
            rospy.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node("safety_measure_node", anonymous=True)
    s = SafetyMeasure()
    s.main()
