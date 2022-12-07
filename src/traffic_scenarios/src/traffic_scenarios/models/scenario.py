import enum
from traffic_scenarios.models.weather import Weather


class Scenario(enum.Enum):
    S1 = 1
    S2 = 2
    S3 = 3
    S4 = 4
    S5 = 5
    S6 = 6
    S7 = 7
    S8 = 8
    S9 = 9

    def get_scenario_params(self):
        return SCENARIOS.get(self)


SCENARIOS = {
    Scenario.S1: {'cars': 100, 'people': 100, 'weather': {'type': Weather.RAINY, 'intensity': 100}},
    Scenario.S2: {'cars': 100, 'people': 100, 'weather': {'type': Weather.FOGGY, 'intensity': 100}},
    Scenario.S3: {'cars': 100, 'people': 100, 'weather': {'type': Weather.SUNNY, 'intensity': 100}},
    Scenario.S4: {'cars': 100, 'people': 0, 'weather': {'type': Weather.RAINY, 'intensity': 0}},  # multiple cars
    Scenario.S5: {'cars': 0, 'people': 100, 'weather': {'type': Weather.RAINY, 'intensity': 0}},  # multiple people
    Scenario.S6: {'cars': 0, 'people': 0, 'weather': {'type': Weather.FOGGY, 'intensity': 100}},  # foggy
    Scenario.S7: {'cars': 0, 'people': 0, 'weather': {'type': Weather.SUNNY, 'intensity': 100}},  # sunny
    Scenario.S8: {'cars': 0, 'people': 0, 'weather': {'type': Weather.RAINY, 'intensity': 100}},  # rainy
    Scenario.S9: {'cars': 0, 'people': 0, 'weather': {'type': Weather.RAINY, 'intensity': 0}},  # nothing
}


def current_scenario():
    return SCENARIOS[Scenario.S1]
