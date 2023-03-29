#!/usr/bin/env python3

from traffic_scenarios.utils.connect import Connect
from carla import Transform, Location, Rotation
from scenario_generation.scenarios_generation import InitialiseScenario


VIEWING_POINT = Transform(
    Location(x=102.215004, y=-78.551079, z=93.157410),
    Rotation(pitch=-86.137344, yaw=-178.290955, roll=0.000343)
)


if __name__ == '__main__':
    connect = Connect()
    client = connect.get_client()
    world = connect.get_world()
    if 'Town03' in world.get_map().name:
        print("already in Town03")
    else:
        print("going to Town03")
        client.load_world('Town03')
    spectator = world.get_spectator()
    spectator.set_transform(VIEWING_POINT)
    i = InitialiseScenario()
    i.main()


# update weather
