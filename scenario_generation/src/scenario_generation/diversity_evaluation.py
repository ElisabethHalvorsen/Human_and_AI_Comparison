#!/usr/bin/env python3

from sklearn.cluster import DBSCAN
from scipy.stats import entropy
import pandas as pd
from traffic_scenarios.models.weather import Weather
import os
import rospkg
from scenario_generation.plotter import plot_3d
import numpy as np

SCENARIO_PATH = os.path.join(rospkg.RosPack().get_path('scenario_generation'), 'src', 'scenario_generation',
                             'scenarios', f'scenarios_{Weather(0)}.csv')

population = pd.read_csv(SCENARIO_PATH)
print("Population shape:", int(population.shape[0]/100))
dbscan = DBSCAN(eps=5, min_samples=int(population.shape[0]/100))
dbscan.fit_predict(population)
pred = dbscan.fit_predict(population)
labels = dbscan.labels_

n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
n_noise_ = list(labels).count(-1)

print("Estimated number of clusters: %d" % n_clusters_)
print("Estimated number of noise points: %d" % n_noise_)

labels = dbscan.labels_
unique_labels = np.unique(labels)
# unique_labels = unique_labels[unique_labels != -1]  # remove noise points, as we don't want to count them
cluster_sizes = []
for i in unique_labels:
    if np.equal(-1, i):  # treat noise points as an individual cluster
        n_noise_points = np.sum(labels == i)
        for _ in range(n_noise_points):
            cluster_sizes.append(1)
    else:
        cluster_sizes.append(np.sum(labels == i))

# Calculate a diversity metric
diversity = entropy(cluster_sizes)  # shannon entropy
# print("Number of clusters:", len(unique_labels))
print("Cluster sizes:", cluster_sizes)
print("Diversity:", diversity)

# plot
plot_3d(population, labels, title='DBSCAN')

# Therefore, a low entropy value suggests that the data is concentrated in a few clusters, while a high entropy value
# suggests that the data is more evenly spread across the clusters. In the context of measuring diversity,
# a high entropy value indicates that the dataset is more diverse, as there are many clusters with a roughly equal
# number of data points. On the other hand, a low entropy value indicates that the dataset is less diverse,
# as most of the data is concentrated in one or a few clusters.
