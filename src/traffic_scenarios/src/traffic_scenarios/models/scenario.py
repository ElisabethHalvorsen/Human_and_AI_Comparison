#!/usr/bin/env python3

from traffic_scenarios.models.weather import Weather
import rospkg
import os

SCENARIO_GEN_PATH = os.path.join(rospkg.RosPack().get_path('scenario_generation'), 'src', 'scenario_generation',
                                 'scenario.txt')


def get_current_scenario():
    with open(SCENARIO_GEN_PATH, 'r') as in_f:
        lines = in_f.readlines()
        for l in lines:
            if 'cars' in l:
                cars = float(l.strip().split(':')[1])
            if 'people' in l:
                people = float(l.strip().split(':')[1])
            if 'weather' in l:
                weather = float(l.strip().split(':')[1])
            if 'weather_type' in l:
                weather_type = float(l.strip().split(':')[1])
    return {'cars': cars, 'people': people, 'weather': {'type': Weather(weather_type), 'intensity': weather}}
