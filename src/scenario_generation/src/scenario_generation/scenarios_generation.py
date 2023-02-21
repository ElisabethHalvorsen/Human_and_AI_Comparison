#!/usr/bin/env python3
import random
import sys
import pandas as pd
from traffic_scenarios.models.weather import Weather
import rospkg
import os
import pygad
import numpy as np
from math import ceil

SCENARIO_GEN_PATH = os.path.join(rospkg.RosPack().get_path('scenario_generation'), 'src', 'scenario_generation')
SCENARIOS_PATH = f'{SCENARIO_GEN_PATH}/scenarios'
SCENARIOS_TXT_PATH = f'{SCENARIO_GEN_PATH}/scenario.txt'

# Parameters
WEIGHTS = np.array([1, 1, 1])
N_GENERATIONS = 10
N_GENES = WEIGHTS.shape[0]
INIT_RANGE_LOW = -3
INIT_RANGE_HIGH = 12


class ScenariosGeneration:
    def __init__(self, permutations: int):
        if not (100 <= permutations <= 1000):
            raise ValueError('Permutations must be between 10 and 20000 ')
        self.permutations = permutations

        # remove the previously generated scenarios
        self.out_names = [f'{SCENARIOS_PATH}/scenarios_{w}.csv' for w in Weather]
        for on in self.out_names:
            if os.path.exists(on):
                os.remove(on)

    @staticmethod
    def _fitness_func(solution, solution_idx) -> int:
        total = np.sum(solution) * WEIGHTS
        if np.logical_and(total >= 15, total <= 25).all():
            fitness = 1
        else:
            fitness = 0
        return fitness

    @staticmethod
    def reformat_population(population) -> np.ndarray:
        population[population > 100] = 100
        population[population < 0] = 0
        return population

    def _generate_one_population(self, n_instances) -> np.ndarray:
        sol_per_pop = n_instances
        n_parents_mating = n_instances
        ga_instance = pygad.GA(num_generations=N_GENERATIONS,
                               num_parents_mating=n_parents_mating,
                               fitness_func=self._fitness_func,
                               sol_per_pop=sol_per_pop,
                               num_genes=N_GENES,
                               init_range_low=INIT_RANGE_LOW,
                               init_range_high=INIT_RANGE_HIGH)
        # Run the GA
        ga_instance.run()
        return (ga_instance.population * 10).astype(int)

    def generate_random_population(self, instances: int) -> np.ndarray:
        n_instances = instances
        if instances < 10:
            raise ValueError("n_instances must be greater than 60")
        instances = ceil(instances / 10)
        population = None
        for i in range(10):
            if isinstance(population, type(None)):
                population = self._generate_one_population(instances)
            else:
                population = np.concatenate((population, self._generate_one_population(instances)), axis=0)
        population = self.reformat_population(population)
        unique_population = np.unique(population, axis=0)
        if len(unique_population) > n_instances:
            n_remove = len(unique_population) - n_instances
            return unique_population[:-n_remove]
        else:
            return unique_population

    def main(self):
        population = self.generate_random_population(self.permutations)
        tests = np.zeros((population.shape[0], 1))
        population = np.concatenate((population, tests), axis=1)
        df = pd.DataFrame(population, columns=["cars", "pedestrians", "weather", "tests"])
        df.to_csv("dbscan_results.csv", index=False)
        for on in self.out_names:
            if os.path.exists(on):
                df.to_csv(on, index=False, mode='a', header=False)
            else:
                df.to_csv(on, index=False, mode='w')


class InitialiseScenario:
    def __init__(self):
        self._weather_type = Weather(random.choice([w.value for w in Weather]))
        self._file = f'{SCENARIOS_PATH}/scenarios_{self._weather_type}.csv'
        self._weather_intensity = 0
        self._car_intensity = 0
        self._pedestrian_intensity = 0

    def _update_txt_file(self):
        with open(SCENARIOS_TXT_PATH, 'w') as f:
            f.write('cars:%i\n' % self._car_intensity)
            f.write('people:%i\n' % self._pedestrian_intensity)
            f.write('weather:%i\n' % self._weather_intensity)
            f.write('weather_type:%s\n' % self._weather_type.value)

    def _update_csv_file(self, df: pd.DataFrame, row: int):
        self._weather_intensity = df.iloc[row]['weather']
        self._car_intensity = df.iloc[row]['cars']
        self._pedestrian_intensity = df.iloc[row]['people']
        df.iloc[row]['tests'] += 1
        df.to_csv(self._file, index=False)

    def main(self):
        df = pd.read_csv(self._file)
        smallest_row = 0
        smallest_val = sys.maxsize
        is_updated = False
        for row in df.iterrows():
            if row[1]['tests'] == 0:
                is_updated = True
                self._update_csv_file(df, row[0])
                break
            if row[1]['tests'] < smallest_val:
                smallest_row = row[0]
                smallest_val = row[1]['tests']

        if not is_updated:
            self._update_csv_file(df, smallest_row)

        self._update_txt_file()


if __name__ == "__main__":
    scenario_generation = ScenariosGeneration(100)
    scenario_generation.main()
    # i = InitialiseScenario()
    # i.main()
