import enum
from weather import Weather


class Scenarios(enum.Enum):
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
    Scenarios.S1: {'cars': 100, 'people': 100, 'weather': {'type': Weather.RAINY, 'intensity': 100}},
    Scenarios.S2: {'cars': 100, 'people': 100, 'weather': {'type': Weather.FOGGY, 'intensity': 100}},
    Scenarios.S3: {'cars': 100, 'people': 100, 'weather': {'type': Weather.SUNNY, 'intensity': 100}},
    Scenarios.S4: {'cars': 100, 'people': 0, 'weather': {'type': Weather.RAINY, 'intensity': 0}},  # multiple cars
    Scenarios.S5: {'cars': 0, 'people': 100, 'weather': {'type': Weather.RAINY, 'intensity': 0}},  # multiple people
    Scenarios.S6: {'cars': 0, 'people': 0, 'weather': {'type': Weather.FOGGY, 'intensity': 100}},  # foggy
    Scenarios.S7: {'cars': 0, 'people': 0, 'weather': {'type': Weather.SUNNY, 'intensity': 100}},  # sunny
    Scenarios.S8: {'cars': 0, 'people': 0, 'weather': {'type': Weather.RAINY, 'intensity': 100}},  # rainy
    Scenarios.S9: {'cars': 0, 'people': 0, 'weather': {'type': Weather.RAINY, 'intensity': 0}},  # nothing
}


def current_scenario():
    return Scenarios.S1
