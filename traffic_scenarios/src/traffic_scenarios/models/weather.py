#!/usr/bin/env python3
import enum


class Weather(enum.Enum):
    FOGGY = 0
    SUNNY = 1
    RAINY = 2

    def __str__(self):
        return self.name
