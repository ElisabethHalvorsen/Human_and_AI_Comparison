#!/usr/bin/env python3
import os
import rospkg
import pandas as pd
import rospy
from carla import Vector3D

DATA_PATH = os.path.join(rospkg.RosPack().get_path('safety_analysis'), 'src', 'safety_analysis', 'data')


def get_last_file():
    files = os.listdir(DATA_PATH)
    files.sort(key=lambda x: os.path.getmtime(os.path.join(DATA_PATH, x)))
    return files[-1]

def score_file(file: str):
    df = pd.read_csv(os.path.join(DATA_PATH, file))
    rows = df.shape[0]
    total_behaviours = rows
    dangerous_behaviours = df['Dangerous'].sum()
    if df['Collided'].any():
        score = -1
        rospy.logwarn("Collision detected")
    else:
        score = (rows - df['Dangerous'].sum()) / rows
    return score, total_behaviours, dangerous_behaviours


if __name__ == "__main__":
    print("Last file: ", get_last_file())
    score, total_behaviours, dangerous_behaviours = score_file(get_last_file())
    print("score: ", score)
    print("Total behaviours: ", total_behaviours)
    print("Dangerous behaviours: ", dangerous_behaviours)


