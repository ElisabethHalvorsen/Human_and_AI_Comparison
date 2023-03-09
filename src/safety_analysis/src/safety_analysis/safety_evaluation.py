#!/usr/bin/env python3
import os
import rospkg
import pandas as pd

DATA_PATH = os.path.join(rospkg.RosPack().get_path('safety_analysis'), 'src', 'safety_analysis', 'data')


def get_last_file():
    files = os.listdir(DATA_PATH)
    files.sort(key=lambda x: os.path.getmtime(os.path.join(DATA_PATH, x)))
    return files[-1]


def score_file(file: str):
    df = pd.read_csv(os.path.join(DATA_PATH, file))
    rows = df.shape[0]
    if df['Collided'].any():
        score = -1
        print("Collision detected")
    else:
        print("Dangerous behaviours: ", df['Dangerous'].sum())
        score = (rows - df['Dangerous'].sum()) / rows
    return score


if __name__ == "__main__":
    print("Last file: ", get_last_file())
    score = score_file(get_last_file())
    print("score: ", score)
