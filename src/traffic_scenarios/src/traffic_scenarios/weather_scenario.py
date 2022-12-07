#!/usr/bin/env python3

import rospy
from carla import WeatherParameters
from traffic_scenarios.utils.connect import Connect
from traffic_scenarios.models.weather import Weather
from traffic_scenarios.models.scenario import Scenario, get_current_scenario

class WeatherScenario:
    def __init__(self, connect: Connect, weather: Weather, intensity: int):
        if not (0 <= intensity <= 100):
            raise ValueError("Frequency must be between 0 and 100")
        self._weather = weather
        self._connect = connect
        self._intensity = intensity

    # TODO: change tire friction
    def get_rain_parameters(self):
        w = WeatherParameters()
        w.precipitation = self._intensity
        w.wetness = self._intensity
        w.cloudiness = 40
        return w

    def get_fogg_parameters(self):
        w = WeatherParameters()
        w.fog_density = self._intensity
        return w

    def get_sun_parameters(self):
        w = WeatherParameters()
        w.sun_altitude_angle = 17.0
        w.sun_azimuth_angle = 5.0
        return w

    def main(self):
        weather = WeatherParameters()
        if self._weather == Weather.FOGGY:
            weather = self.get_fogg_parameters()
        elif self._weather == Weather.SUNNY:
            weather = self.get_sun_parameters()
        elif self._weather == Weather.RAINY:
            weather = self.get_rain_parameters()
        self._connect.get_world().set_weather(weather)


if __name__ == '__main__':
    rospy.init_node("weather_node", anonymous=True)
    _connect = Connect()
    scenario = get_current_scenario()
    weather = scenario["weather"]["type"]
    intensity = scenario["weather"]["intensity"]
    ws = WeatherScenario(_connect, weather, intensity)
    ws.main()
    rospy.spin()