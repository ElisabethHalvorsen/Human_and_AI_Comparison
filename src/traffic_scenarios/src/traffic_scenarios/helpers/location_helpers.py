
from carla import Location, Transform, Rotation
from traffic_scenarios.utils.connect import Connect

TOLERANCE = 10


def is_at_end(loc: Location, end: Location) -> bool:
    return (end.x - TOLERANCE < loc.x < end.x + TOLERANCE) and (end.y - TOLERANCE < loc.y < end.y + TOLERANCE)


def get_current_location_carla(connect: Connect):
    spectator = connect.get_world().get_spectator()
    return spectator.get_transform()