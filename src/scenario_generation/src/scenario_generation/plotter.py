#!/usr/bin/env python3

import matplotlib.pyplot as plt


def plot_3d(population, assignments=None, title=None):
    ax = plt.axes(projection='3d')
    if assignments is None:
        ax.plot3D(population[:, 0], population[:, 1], population[:, 2], 'o')
    else:
        ax.scatter(population[:, 0], population[:, 1], population[:, 2], c=assignments, s=50)
    if title:
        ax.set_title(title)
    plt.show()
