import os
import time
import sys
from gcodeparser import GcodeParser
from frankx import Affine, LinearRelativeMotion, Robot
import pandas as pd
from am_robot import utility

# TODO - filename from argparse
# Currently requires .gcode to be in data folder

def main():
    # Initialize robot
    #robot = Robot("172.16.0.2")
    # Input frequency
    #Hz 1000

    # Initialize extruder status
    am_geometry = utility.Geometry_Status(0,0,0,0,0,0,0,0,0,'idle')
    am_machine = utility.Machine_Status(0,0,0)

    # Gcode file location
    filename = 'dontprint.gcode'
    folder = 'data'

    fullpath = os.path.join('.',folder,filename)

    # for root, dirs, files in os.walk(".", topdown=True):
    #     print('start')
    #     for dir in dirs:
    #         print(os.path.join(root, dir))
    #     print('stop-start')
    #     for file in files:
    #         print(os.path.join(root, file))
    #     print('stop')

    # print(am_robot.__file__.split)

    # Gcode load, read and parse
    with open(fullpath,'r') as file:
        gcode = file.read()
    parsed_gcode = GcodeParser(gcode)

    # Single line version, slower as the parsing is done each 
    with open(fullpath,'r') as f:
        for index, linje in enumerate(f):
            #print("line {}: {}".format(index, linje.strip()))
            if linje[0] == 'G' or linje[0] == 'M':
                parsedlinje = GcodeParser(linje)
                pandalinje = pd.DataFrame(parsedlinje.lines,columns=["command","params","comment"])
                ownparsed = utility.format_gcodeline(linje)
                
    GcodePandas = pd.DataFrame(parsed_gcode.lines,columns=["command","params","comment"])
    
    #print(GcodePandas.iloc[1])
    #print(GcodePandas)

    # Examble GcodeLine format
    # GcodeLine(
    #     command=('G',1),
    #     params={'X':12.3,'Y':7.4,'E':15.5},
    #     comment='Linear Move and Extrude')

    for line in parsed_gcode.lines:
        # Start time for checking loop time
        start_time = time.time()
        current_command = line.command
        current_params = line.params
        current_comment = line.comment

        if current_command[0] == 'G':
            am_geometry.gcommand = current_command[1]
            if current_command[1] == 0 or current_command[1] == 1:
                am_geometry.move_type = 'linear'
                for key in current_params:
                    #am_geometry.key = line.get_param(key)
                    if key == 'X':
                        am_geometry.X = line.get_param(key)
                    elif key == 'Y':
                        am_geometry.Y = line.get_param(key)
                    elif key == 'Z':
                        am_geometry.Z = line.get_param(key)
                    elif key == 'F':
                        am_geometry.F = line.get_param(key)
                    elif key == 'E':
                        am_geometry.E = line.get_param(key)
                # robot_movement(am_geometry)
                # update_extruder(am_geometry)

            elif current_command[1] == 2 or current_command[1] == 3:
                if current_command[1] == 2:
                    am_geometry.move_type = 'cw_arc'
                else:
                    am_geometry.move_type = 'ccw_arc'
                for key in current_params:
                    if key == 'X':
                         am_geometry.X = line.get_param(key)
                    elif key == 'Y':
                         am_geometry.Y = line.get_param(key)
                    elif key == 'Z':
                         am_geometry.Z = line.get_param(key)
                    elif key == 'F':
                         am_geometry.F = line.get_param(key)
                    elif key == 'E':
                         am_geometry.E = line.get_param(key)
                    elif key == 'R':
                        # TODO allow arc moves, or make them linear segments
                        print("R move - ignored")
                        #am_geometry.R = line.get_param(key)
                    elif key == 'I':
                        print("I move - ignored")
                        #am_geometry.I = line.get_param(key)
                    elif key == 'J':
                        print("J move - ignored")
                        #am_geometry.J = line.get_param(key)
                    else:
                        am_geometry.__name__ = line.get_param(key)

            elif current_command[1] == 10:
                print("start retraction move")
                am_geometry.move_type = 'retraction'
            elif current_command[1] == 11:
                print("Start recover move after a G10")
                am_geometry.move_type = recover
            elif current_command[1] == 20:
                print("set inches")
            elif current_command[1] == 21:
                print("set millimeters")
            elif current_command[1] == 28:
                print("auto home move")
                am_geometry.move_type = 'home'
            elif current_command[1] == 90:
                print("Set absolute positioning")
            elif current_command[1] == 91:
                print("Set relative positioning")
            elif current_command[1] == 92:
                print("Set positioning")
            else:
                print(f"No action for given G-command number: {current_command[1]}")
            
            #do stuff
        elif current_command[0] == 'M':
            am_machine.mcommand = current_command[1]
            if current_command[1] == 82:
                print("E absolute")
            elif current_command[1] == 84:
                print("Disable motors")
            elif current_command[1] == 104:
                print("Set hotend temperature")
            elif current_command[1] == 105:
                print("Report temperature")
            elif current_command[1] == 106:
                print("Set fan speed")
            elif current_command[1] == 107:
                print("Fan off")
            elif current_command[1] == 109:
                print("Wait for hotend temperature")
            elif current_command[1] == 140:
                print("Set bed temperature")
            else:
                print(f"No action for M command number: {current_command[1]}")
            #do other stuff
        else:
            print("Neither G nor M command")

        #print(str(am_geometry))

        end_time = time.time()
        if end_time-start_time > 0.0003: # Have about 300 us to spare to achieve 1kHz
            print(f"runtime loop = {end_time-start_time} > 300 us.")
        #time.sleep(0.5) # TODO - change to interval instead of pause

if __name__ == '__main__':
    main()