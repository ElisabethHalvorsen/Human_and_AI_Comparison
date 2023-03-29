#!/usr/bin/env python3

from carla import Client, World


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
