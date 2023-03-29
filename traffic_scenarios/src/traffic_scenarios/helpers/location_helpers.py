
from carla import Location, Transform, Rotation
from traffic_scenarios.utils.connect import Connect


def is_at_end(loc: Location, end: Location, tolerance=10) -> bool:
    return (end.x - tolerance < loc.x < end.x + tolerance) and (end.y - tolerance < loc.y < end.y + tolerance)


def get_current_location_carla(connect: Connect):
    spectator = connect.get_world().get_spectator()
    return spectator.get_transform()