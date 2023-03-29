#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_3d(population, labels, title=None):
    ax = plt.axes(projection='3d')
    numpy_population = population.to_numpy()
    ax.scatter(numpy_population[:, 0], numpy_population[:, 1], numpy_population[:, 2], c=labels, s=50)
    if title:
        ax.set_title(title)
    plt.show()
