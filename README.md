![GitHub contributors](https://img.shields.io/github/contributors/alejontnu/am-robot?label=Number%20of%20button%20mashers&style=plastic)

# am-robot
Package for using a Franka Emika Panda robot maniplator together with an extruder for additive manufacturing. 

This is a continuation of Alexander Waller Johnsgaard master's thesis. Forked from: https://github.com/alejontnu/am-robot

Andreas Moltumyr developed the control of the extruder: https://github.com/mltmyr/extruderCtrl/tree/main

## About the package
The AM-robot package aims to enable additive manufacturing on a robot manipulator.  A G-code file is used as input and after some pro-processing of the file, motion trajectories for the robot is generated. An FDM extruder have been used when testing, controlled by an Arduino Mega2560 over a serial connection. 

## Feature summary
Features are under development and spesifics can change as the project continues...
- Parsing and pre-processing of G-code input file
- Robot motion trajectory generation from G-code targets utilizing Frankx/Ruckig
- Automatic build area level detection and non-level compensation
- Visualization of tool-path trajectories and bed level mesh
- Generalization of tool and robot choice (Still needs work)
- Non-realtime syncronization of tool to robot
 
## Getting started
A Docker file is made to build a Docker container which sets up an environment with the necessary dependencies for working with the Panda robot and the AM-robot project.
The Docker file and Docker commands can be found here:  https://github.com/theacharlotte/AM_robot_docker
After building and runnung the docker, write the following command:
```ruby
pip install -e .
'''
The Docker file includes installation of libfranka v0.9.2 and frankx v0.2.0 for communication with Franka Emika Panda robot, and the dependencies for libfranka, frankx and AM_robot.
Python 3.10 is installed as well as usefull packages such as git and GCC.

To use this container, ensure Docker is installed on your system and build the image by looking at the commands in the init.md file. Once built, you can run the container to get environment needed to run the functions in this (AM-robot) project.

### Dependencies
This project is dependent on:
- plotly: For visualizion of trajectories and bed level mesh
- argparse: For parsing input arguments
- gcodeparser: For initial parsing of G-code into Dictionary object containing G-code lines and more
- numpy
- pandas
- matplotlib
- mgen

Franka Emika Panda robot is dependent on:
- libfranka: C++ library for controlling Franka Emika Panda robot. 
- frankx: Python wrapper around libfranka. 

Arduino FDM extruder controller is dependent on:
- pyserial: For communication with the Arduino
- struct: To pack and unpack format used for serial communication


### Quick guide
**Arguments taken**
| Argument | info |
|----------|------|
| --host | ip string to connect to robot over ethernet |
| --tool | serial port string to open serial connection to tool controller |
| --gfile | G-code file string, can be with or without the .gcode extension |
| --visualize | Enables visualization of tool path trajectories and bed level mesh. Default: False |
| --skip_connection | Skips connection to robot. Useful if only visualizing G-code or if no robot is physically connected. Default: False |


**G-code start code lines for machine**
```G-code
G28 ; home all axes
G1 Z5 F5000 ; lift nozzle
M107 ; enable fan
M104 S200 ; set temperature
```
This is so that homing and bed probing can be done before nozzle is heated up.


## Current status TO BE DELETED LATER
Preliminary result show need for better control of the extrusion process. As can be seen in the test print below, motion trajectories are aborted if the robot encounters an error (such as acceleration contraint violation) and the remainder of the motion trajectory is left incomplete.

![First Benchy](https://github.com/alejontnu/am-robot/blob/main/images/blackprofile.png?raw=true)

## Future plans
- Currently up to user to determine feasable build area size, can possibly implement an automatic check for this.
- Convert to true 3D with use of additional DoF. Currently building 2.5D layer-by-layer.
- Make the extrusion process follow tool-head trajectory in real-time.
- Improve the interval generating method used to determine motion trajectory segments

## Known issues
- Different slicer software result in inconsistent pre-processing of the G-code generated.
- Bed probing method leaves toolhead with an offset from where it should be placed.
- Motion trajectories are sometimes aborted when an robot error is detected, need to reduce or handle such cases.
