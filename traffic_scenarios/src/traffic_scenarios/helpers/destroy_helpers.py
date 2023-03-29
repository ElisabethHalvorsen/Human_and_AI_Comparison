#!/usr/bin/env python3
from traffic_scenarios.utils.connect import Connect


def destroy_all_vehicles(connect: Connect):
    actor_list = connect.get_world().get_actors()
    for a in actor_list.filter('vehicle.*'):
        a.set_autopilot(False)
        a.destroy()


def destroy_all_pedestrians(connect: Connect):
    actor_list = connect.get_world().get_actors()
    for a in actor_list.filter('walker.pedestrian.*'):
        a.destroy()


def destroy_car(car):
    car.set_autopilot(False)
    car.destroy()


if __name__ == '__main__':
    connect = Connect()
    client = connect.get_client()
    destroy_all_vehicles(connect)
    destroy_all_pedestrians(client)