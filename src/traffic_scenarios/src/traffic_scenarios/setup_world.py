#!/usr/bin/env python3

from traffic_scenarios.utils.connect import Connect
from carla import Transform, Location, Rotation

VIEWING_POINT = Transform(
    Location(x=99.611725, y=-78.628754, z=54.584332),
    Rotation(pitch=-86.137238, yaw=-178.290970, roll=0.000341)
)

connect = Connect()
world = connect.get_world()
spectator = world.get_spectator()
spectator.set_transform(VIEWING_POINT)
