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
N_GENERATIONS = 20
N_GENES = WEIGHTS.shape[0]
INIT_RANGE_LOW = -5
INIT_RANGE_HIGH = 55

MAX = 300
MIN = 200
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
        total = np.sum(np.multiply(solution, WEIGHTS))
        if MIN <= total <= MAX:
            fitness = 5
        else:
            fitness = -5
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
        population = np.array([])
        while population.shape[0] < n_instances:
            for i in range(10):
                if population.shape[0] == 0:
                    population = self._generate_one_population(instances)
                else:
                    population = np.concatenate((population, self._generate_one_population(instances)), axis=0)
            population = self.reformat_population(population)
            population = np.unique(population, axis=0)
            # filter out invalid rows
            row_sums = np.sum(population, axis=1)
            mask = (row_sums >= MIN) & (row_sums <= MAX)
            # use the boolean mask to select the rows that meet the criteria
            population = population[mask]
            if len(population) > n_instances:
                n_remove = len(population) - n_instances
                return population[:-n_remove]
        return population

    def main(self):
        population = self.generate_random_population(self.permutations)
        print(population.shape)
        tests = np.zeros((population.shape[0], 1))
        population = np.concatenate((population, tests), axis=1)
        df = pd.DataFrame(population, columns=["cars", "people", "weather", "tests"])
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
    scenario_generation = ScenariosGeneration(1000)
    scenario_generation.main()
    # i = InitialiseScenario()
    # i.main()
