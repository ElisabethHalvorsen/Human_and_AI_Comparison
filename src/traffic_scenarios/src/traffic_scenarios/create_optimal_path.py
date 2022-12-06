#!/usr/bin/env python3
"""
Code is based upon: https://github.com/carla-simulator/carla/issues/3890
"""

from carla import Transform, Location, Rotation, Color
from traffic_scenarios.agents.navigation.global_route_planner import GlobalRoutePlanner
from traffic_scenarios.agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from traffic_scenarios.utils.connect import Connect
import rospy

START = Transform(Location(x=134.911606, y=-75.612572, z=8.305596),
                  Rotation(pitch=0.000000, yaw=-178.490875, roll=0.000000))
RIGHT_END = Transform(Location(x=84.184067, y=-105.191704, z=8.305596),
                      Rotation(pitch=0.000000, yaw=-87.975883, roll=0.000000))
LEFT_END = Transform(Location(x=71.362877, y=-7.414977, z=0.450000),
                     Rotation(pitch=0.000000, yaw=-179.144165, roll=0.000000))


class WaypointPath:
    def __init__(self, connect):
        self._connect = connect
        self._world = connect.get_world()
        self._map = self._world.get_map()

    def get_waypoint_path(self, start: Transform,
                          end: Transform,
                          sampling_resolution: int = 1) -> list:
        """
        Get a waypoint path from start to end
        :rtype: list(Waypoint, RoadOption)
        """
        dao = GlobalRoutePlannerDAO(self._map, sampling_resolution)
        grp = GlobalRoutePlanner(dao)
        grp.setup()
        a = Location(start.location)
        b = Location(end.location)
        return grp.trace_route(a, b)

    def display_waypoint_path(self, path: list, life_time: float = 10.0):
        """
        Display a waypoint path in the simulator
        :param path: list(Waypoint, RoadOption)
        :param life_time: float
        """
        for w in path:
            self._world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
                                          color=Color(r=155, g=0, b=255), life_time=life_time,
                                          persistent_lines=True)


if __name__ == '__main__':
    rospy.init_node('waypoint_path')
    _connect = Connect()
    waypoint_path = WaypointPath(_connect)
    path_left = waypoint_path.get_waypoint_path(START, LEFT_END)[1:85]
    waypoint_path.display_waypoint_path(path_left, 30.0)
    path_right = waypoint_path.get_waypoint_path(START, RIGHT_END)[5:]
    waypoint_path.display_waypoint_path(path_right, 30.0)
