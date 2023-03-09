#!/usr/bin/env python3
#
# Copyright (c) 2020 Intel Corporation
#
# Modified to this projects purposes by Elisabeth K. Halvorsen


import math
from carla import ad
class RssStateInfo(object):

    def __init__(self, rss_state, ego_dynamics_on_route, world_model):
        print("going into rss state")
        self.rss_state = rss_state
        self.distance = -1
        self.is_dangerous = ad.rss.state.isDangerous(rss_state)
        if rss_state.situationType == ad.rss.situation.SituationType.Unstructured:
            self.actor_calculation_mode = ad.rss.map.RssMode.Unstructured
        else:
            self.actor_calculation_mode = ad.rss.map.RssMode.Structured

        self.ego_dynamics_on_route = ego_dynamics_on_route

        # calculate distance to other vehicle
        object_state = None
        for scene in world_model.scenes:
            if scene.object.objectId == rss_state.objectId:
                object_state = scene.object.state
                break

        if object_state:
            self.distance = math.sqrt(
                (float(ego_dynamics_on_route.ego_center.x) - float(object_state.centerPoint.x)) ** 2 +
                (float(ego_dynamics_on_route.ego_center.y) - float(object_state.centerPoint.y)) ** 2)

        self.longitudinal_margin = float(
            rss_state.longitudinalState.rssStateInformation.currentDistance - rss_state.longitudinalState.rssStateInformation.safeDistance)
        self.margin = max(0.0, self.longitudinal_margin)
        self.lateral_margin = None
        if rss_state.lateralStateLeft.rssStateInformation.evaluator != "None":
            self.lateral_margin = rss_state.lateralStateLeft.rssStateInformation.currentDistance - rss_state.lateralStateLeft.rssStateInformation.safeDistance
        if rss_state.lateralStateRight.rssStateInformation.evaluator != "None":
            lateral_margin_right = rss_state.lateralStateRight.rssStateInformation.currentDistance - rss_state.lateralStateRight.rssStateInformation.safeDistance
            if self.lateral_margin == None or self.lateral_margin > lateral_margin_right:
                self.lateral_margin = lateral_margin_right
        if self.lateral_margin != None and self.lateral_margin > 0:
            self.margin += self.lateral_margin

    def get_actor(self, world):
        if self.rss_state.objectId == 18446744073709551614:
            return None  # "Border Left"
        elif self.rss_state.objectId == 18446744073709551615:
            return None  # "Border Right"
        else:
            return world.get_actor(self.rss_state.objectId)

    def get_lateral_margin(self):
        return self.lateral_margin

    def get_is_dangerous(self):
        return self.is_dangerous

    def get_distance(self):
        return self.distance

    def get_min_stopping_dist(self):
        return self.ego_dynamics_on_route.min_stopping_distance

    def get_is_crossing_border(self):
        return self.ego_dynamics_on_route.crossing_border

    def __str__(self):
        return "RssStateInfo: object=" + str(self.rss_state.objectId) + \
            " dangerous=" + str(self.is_dangerous) + \
            " distance=" + str(round(self.distance, 2)) + \
            " lateral margin=" + str(self.lateral_margin) + \
            " min stop dist=" + str(self.ego_dynamics_on_route.min_stopping_distance) + \
            " crossing border=" + str(self.ego_dynamics_on_route.crossing_border)
