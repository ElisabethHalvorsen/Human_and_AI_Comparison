# Safety Analysis of Autonomous vehicles
Requirements to the installation 
- Operating system is Ubuntu 20.04 LTS
- A GPU with at least 6GB of memory
- Disk space of at least 50GB

## Installation instructions
### Installing CARLA
This installation is based on the following links:
[CARLA quick start guide](https://carla.readthedocs.io/en/latest/start_quickstart/) and 
[CARLA RSS installation](https://carla.readthedocs.io/en/latest/adv_rss/#dependencies)

1. Install the necessary dependencies
   1. `sudo apt-get install libgtest-dev libpython3.8 libpugixml-dev libtbb-dev`
   2. ` pip3 install --upgrade pip`
   3. `pip3 install --user pygame numpy`
   4. `pip3 install --user -U colcon-common-extensions`
   5. `sudo apt-get install castxml`
   6. `pip3 install --user pygccxml pyplusplus`
2. Install CARLA
   1. go into [this link](https://github.com/carla-simulator/carla/releases/tag/0.9.13/) and download the file called **CARLA_0.9.13_RSS.tar.gz**
   2. extract the file and place it where you want to use it 
3. export the needed variables, it's recommended to add it to the bashrc file, where <PATH_TO_CARLA> is the path to the file downloaded in step 6
   1. `export PYTHONPATH=$PYTHONPATH:/<PATH_TO_CARLA>/CARLA_0.9.13_RSS/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:/<PATH_TO_CARLA>/CARLA_0.9.13_RSS/PythonAPI/carla`

   
### Installing ROS Noetic
[This guide follows](http://wiki.ros.org/noetic/Installation/Ubuntu)

1. Setup your sources.list
   1. `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
2. Set up your keys
   1. `sudo apt install curl # if you haven't already installed curl1`
   2. `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
3. Installation
   1. `sudo apt update`
   2. `sudo apt install ros-noetic-desktop-full`
4. Install build dependencies
   1. `sudo apt-get install python3-catkin-tools`


### Installing python packages 
It is recommended to first crate a virtual environment.
To install the virtual environment do the following in the terminal
1. install the virtual environment package
   1. `python -m pip install --user virtualenv`
2. Creat a virtual environment, be sure to be in the directory where you want the virtual environment to be created
   1. `python -m venv venv`
3. activate the virtual environment
   1. `source venv/bin/activate`
4. Install the required packages, which can be found in the file Human_and_AI_Comparison/requirements.txt
   1. `pip install -r requirements.txt`

### Initialising the workspace
To initialise the workspace do the following in the terminal: 
1. `mkdir -p name_ws/src`
2. `cd name_ws/src`
3. place an extracted version of folder Human_and_AI_Comparison in the src folder
4. `cd ..`
5. `catkin build`

## Run instructions
### Setup
Before running anything you need to start the simulator and start ros. To do so do the following in the terminal:
1. `cd <PATH_TO_CARLA>/CARLA_0.9.13_RSS`
2. `./CarlaUE4.sh --opengl4`
   1. NB: sometimes the simulator doesn't start, if this happens try to run the simulator again
3. in another terminal do `roscore`

### Run autonomous vehicle
1. Source the environment:
   1. `source /opt/ros/noetic/setup.bash`
   2. `source /<PATH_TO_WORKSPACE>/devel/setup.bash`
2. Generate a diverse set of scenarios:
   1. `rosrun scenario_generation scenarios_generation.py`
   2. NB: if you want to change the population size go into the file Human_and_AI_Comparison/scenario_generation/src/scenario_generation/scenarios_generation.py
and change the value at line 141 the value of the variable population_size to the desired value
3. See the diversity of the diverse set of scenarios just created:
   1. `rosrun scenario_generation diversity_evaluation.py`
4. Run a scenario:
   1. `roslaunch traffic_scenarios traffic_scenarios.launch`
5. Analyse the scenario just run:
   1. `rosrun traffic_scenarios analyse_scenario.py`
6. Reset the simulation:
   1. `rosrun safety_analysis safety_evaluation.py`
7. Repeat steps 4-6 for as many times as you want to generate as much data you want. The data generated is located in /Human_and_AI_Comparison/safety_analysis/src/safety_analysis/data
8. Extract the main data collected during the runs
   1. `rosrun safety_analysis complete_evaluation.py`
   2. NB: if you want more than at most 10 files increase number at line 20 in the file Human_and_AI_Comparison/safety_analysis/src/safety_analysis/complete_evaluation.py

### Run manual driver
All steps are the same except for the Run scenario step, in this case you have to run all the nodes manually. Before running anything in any terminal you need to source the environment as described in step 1 in Run autonomous vehicle.

To do so do the following in the terminal in the order shown:
1. `rosrun traffic_scenarios setup_world.py`
2. `rosrun traffic_scenarios weather_scenario.py`
3. `rosrun traffic_scenarios cross_scenario.py`
4. `rosrun traffic_scenarios moving_cars_scenario.py`
5. `rosrun vehicle_control manual_control.py`
6. `rosrun safety_analysis safety_measure.py`