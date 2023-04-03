#!/usr/bin/env python3
import rospy
from safety_analysis.safety_evaluation import score_file
import os
import rospkg
from traffic_scenarios.models.weather import Weather
import pandas as pd
from carla import Location, Color
from traffic_scenarios.utils.connect import Connect


DATA_PATH = os.path.join(rospkg.RosPack().get_path('safety_analysis'), 'src', 'safety_analysis', 'data')
EVALUATION_PATH = os.path.join(rospkg.RosPack().get_path('safety_analysis'), 'src', 'safety_analysis')


def get_first_10_weather_files(weather: Weather):
    files = os.listdir(DATA_PATH)
    files = [f for f in files if weather.name in f]
    try:
        return files[:10]
    except IndexError:
        rospy.logwarn("Not enough files for weather: ", weather.name)
        return files


def get_dataframe(file: str) -> pd.DataFrame:
    return pd.read_csv(os.path.join(DATA_PATH, file))


class Evaluation:
    def __init__(self):
        self._out_name = f'{EVALUATION_PATH}/evaluation.csv'
        if os.path.exists(self._out_name):
            os.remove(self._out_name)

    def get_collistion_points(self, file: str):
        df = get_dataframe(file)
        collision_row = df.loc[df['Collided'] == True]
        x = collision_row['Player Loc X'].values[0]
        y = collision_row['Player Loc Y'].values[0]
        z = collision_row['Player Loc Z'].values[0]
        return x, y, z

    def main(self):
        for weather in Weather:
            print(f"Weather: {weather.name}")
            for file in get_first_10_weather_files(weather):
                print(f"File: {file}")
                score, total_behaviours, dangerous_behaviours = score_file(file)
                print("score: ", score)
                df = pd.DataFrame({
                    'Weather': [weather.name],
                    'File': [file],
                    'Score': [score],
                    'Total_behaviours': [total_behaviours],
                    'Dangerous_behaviours': [dangerous_behaviours],
                    'Collision Point': [self.get_collistion_points(file) if score == -1 else None]})
                if os.path.exists(self._out_name):
                    df.to_csv(self._out_name, index=False, mode='a', header=False)
                else:
                    df.to_csv(self._out_name, index=False, mode='w')


class DisplayDangerousPoints:
    def __init__(self):
        self._files = self.get_all_files()
        self._connect = Connect()
        self._world = self._connect.get_world()

    @staticmethod
    def get_all_files():
        files = []
        for weather in Weather:
            files += get_first_10_weather_files(weather)
        return files

    @staticmethod
    def get_collision_point(file: str):
        try:
            df = get_dataframe(file)
            collision_row = df.loc[df['Collided'] == True].index[0]
            x = df.loc[collision_row]['Player Loc X']
            y = df.loc[collision_row]['Player Loc Y']
            z = df.loc[collision_row]['Player Loc Z']
            return Location(x, y, z)
        except IndexError:
            return None

    @staticmethod
    def get_dangerous_points(file: str):
        df = get_dataframe(file)
        condition = df['Dangerous'] == True
        indices = df.loc[condition].index.tolist()
        return [Location(df['Player Loc X'][i], df['Player Loc Y'][i], df['Player Loc Z'][i]) for i in indices]

    def display_all_collision_points(self, lifetime=10):
        for file in self._files:
            point = self.get_collision_point(file)
            if point:
                self._world.debug.draw_point(point, color=Color(r=155, g=0, b=255), life_time=lifetime)

    def display_all_dangerous_points(self):
        danger_points = []
        for file in self._files:
            danger_points += self.get_dangerous_points(file)
        if danger_points:
            for point in danger_points:
                self._world.debug.draw_point(point, color=Color(r=255, g=0, b=0), life_time=10)



if __name__ == "__main__":
    rospy.init_node('complete_safety_evaluation')
    # evaluation = Evaluation()
    # evaluation.main()
    display = DisplayDangerousPoints()
    display.display_all_collision_points()
