#!/usr/bin/env python3
import random

import pandas as pd
from traffic_scenarios.models.weather import Weather
import rospkg
import os

RANGE = 100
LIM = 1000000
SCENARIO_GEN_PATH = os.path.join(rospkg.RosPack().get_path('scenario_generation'), 'src', 'scenario_generation')
SCENARIOS_PATH = f'{SCENARIO_GEN_PATH}/scenarios'
SCENARIOS_TXT_PATH = f'{SCENARIO_GEN_PATH}/scenario.txt'


class ScenariosGeneration:
    def __init__(self, permutations: int):
        if not (1 <= permutations <= 20000):
            raise ValueError('Permutations must be between 10 and 20000 ')
        self.permutations = permutations
        self.out_names = [f'{SCENARIOS_PATH}/scenarios_{w}.csv' for w in Weather]
        for on in self.out_names:
            if os.path.exists(on):
                os.remove(on)

    def main(self):
        skip = int(LIM / self.permutations)
        curr = 0
        hits_available = 1
        scenarios = 0
        try:
            for cars in range(RANGE):
                for people in range(RANGE):
                    for weather in range(RANGE):
                        if curr % skip == 0:
                            hits_available += 1
                        if (200 <= cars + people + weather) and (hits_available > 0):
                            hits_available -= 1
                            scenarios += 1
                            df_outcome = pd.DataFrame(
                                {'cars': [cars], 'people': [people], 'weather': [weather], 'tests': [0]})
                            for on in self.out_names:
                                if os.path.exists(on):
                                    df_outcome.to_csv(on, index=False, mode='a', header=False)
                                else:
                                    df_outcome.to_csv(on, index=False, mode='w')
                            if scenarios >= self.permutations:
                                raise StopIteration
                        curr += 1
        except StopIteration:
            pass


class InitialiseScenario:
    def __init__(self):
        self._weather_type = Weather(random.choice([w.value for w in Weather]))
        self._file = f'{SCENARIOS_PATH}/scenarios_{self._weather_type}.csv'
        self._weather_intensity = None
        self._car_intensity = None
        self._pedestrian_intensity = None

    def main(self):
        df = pd.read_csv(self._file)
        for row in df.iterrows():
            if row[1]['tests'] == 0:
                self._weather_intensity = row[1]['weather']
                self._car_intensity = row[1]['cars']
                self._pedestrian_intensity = row[1]['people']
                df.iloc[row[0]]['tests'] += 1
                df.to_csv(self._file, index=False)
                break
        with open(SCENARIOS_TXT_PATH, 'w') as f:
            f.write('cars:%i\n' % self._car_intensity)
            f.write('people:%i\n' % self._pedestrian_intensity)
            f.write('weather:%i\n' % self._weather_intensity)
            f.write('weather_type:%s\n' % self._weather_type.value)


if __name__ == "__main__":
    # os.system('ls -l')  # run command in terminal
    # scenario_generation = ScenariosGeneration(100)
    # scenario_generation.main()
    i = InitialiseScenario()
    i.main()
