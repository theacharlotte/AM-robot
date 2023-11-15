import argparse
import numpy as np
import threading
from time import sleep
from am_robot.ExtruderTool import ExtruderTool
from am_robot.FrankaRobot import FrankaRobot

# Ting for tester
from frankx import Affine, LinearMotion, Robot, RobotMode, RobotState, WaypointMotion, JointMotion, Waypoint, Reaction, LinearRelativeMotion, Measure, PathMotion, MotionData
from _movex import Path, TimeParametrization, Trajectory




def move_robot_square(self):
    translations = [Affine(10.0, 0.0, 0.0), Affine(0.0, 10.0, 0.0), Affine(-10.0, 0.0, 0.0), Affine(0.0, -10.0, 0.0)]
    for i in range(len(translations)):
        motion_line = self.make_linear_relative_motion(translations(i))
        self.move(motion_line)

def move_robot_angle(self, angle_deg):
    next_translation = Affine(10.0, 0.0, 0.0)
    motion_line = self.make_linear_relative_motion(next_translation)
    self.move(motion_line)

    # Calculate the x and y position for the new waypoint based on the angle
    angle_rad = np.radians(angle_deg)
    new_x = 10 * np.cos(angle_rad)
    new_y = 10 * np.sin(angle_rad)

    next_translation = Affine(new_x, new_y, 0.0) 
    motion_line = self.make_linear_relative_motion(next_translation)
    self.move(motion_line)




def research_experiments(line, lineVar,deskMotion):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.MetavarTypeHelpFormatter,
        description=('''Package for controlling a 3D printing on a 7 DoF robotic arm'''),
        epilog='This is still under development',
        add_help=True)

    parser.add_argument('--host', default='10.0.0.2', type=str, help='FCI IP of the robot')
    parser.add_argument('--tool', default='/dev/ttyUSB0', type=str, help='Serial connection of the tool used')

    parser.add_argument('--t_tool', default=[0,0,-0.1], type=list, help='Translation due to Tool as [x,y,z]')
    parser.add_argument('--d_nozzle', default=0.8, type=float, help='Hot-End Nozzle diameter')
    parser.add_argument('--f_width', default=2.85, type=float, help='Width of filament used')

    parser.add_argument('--skip_connection', action='store_true', help='If specified, skips the connection to robot. For testing out-of-lab. Also defaults too True if visualize is True')

    args = parser.parse_args()

    tool = ExtruderTool(args.tool,'FDM',args.f_width,args.d_nozzle,args.t_tool,args.skip_connection)
    robot = FrankaRobot(args.host,args.skip_connection)
     
    #tool.set_feedrate(0) 
    #tool.set_nozzletemp(0)

    
    temp = 200
    feedrate = 150 # Maybe change to 1800 ####################################

    # Set velocity, acceleration and jerk to 1% of the maximum
    robot.set_dynamic_rel(0.01)
    
    print("INFO: Press EMERGENCY STOP button DOWN and position the robot manually.")
    print("After positioning robot, open the EMERGENCY STOP button to its UP position! The robot should the have BLUE lights")
    input("Position end-effector nozzle 10 cm over the bed\nWhen satisfied with position, press Enter to continue...")
    
    # Heat up extruder
    tool.set_nozzletemp(temp)

    while tool.read_temperature() < (temp - 5.0):
        print(tool.read_temperature())
        pass
        
    # Start extruder
    tool.set_feedrate(feedrate) 
    input("Press enter when the material is continously deposited")
    tool.set_feedrate(0) 
    
    # Make a 10 cm line feed rate constant
    print("Manually move the robot to a satisfactory starting point for printing")
    input("Press enter when the nozzle position for printing is satisfactory.")
     
    # Start extruder and move ten cm relative  
    if line: 
        #tool.set_feedrate(feedrate) 
        next_pose = Affine(0.0, 0.1, 0.0)
        motion_forward = robot.make_linear_relative_motion(next_pose)
        robot.execute_move(None, motion_forward)
    elif lineVar:
        thread_extruder = threading.Thread(target=tool.thread_varying_feedrate, args=(feedrate,))
        next_translation = Affine(0.0, 0.1, 0.0)
        motion_line = robot.make_linear_relative_motion(next_translation)

        thread_extruder.start()
        robot.execute_move(None, motion_line)

        thread_extruder.join()
    elif deskMotion:
        tool.set_feedrate(50)
        sleep(10)
        tool.set_feedrate(0.0)

    # Disconnect extruder
    tool.set_feedrate(0.0) 
    tool.set_nozzletemp(0.0)
    tool.disconnect

    
def research_experiments2(deskMotion):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.MetavarTypeHelpFormatter,
        description=('''Package for controlling a 3D printing on a 7 DoF robotic arm'''),
        epilog='This is still under development',
        add_help=True)

    parser.add_argument('--tool', default='/dev/ttyUSB0', type=str, help='Serial connection of the tool used')

    parser.add_argument('--t_tool', default=[0,0,-0.1], type=list, help='Translation due to Tool as [x,y,z]')
    parser.add_argument('--d_nozzle', default=0.8, type=float, help='Hot-End Nozzle diameter')
    parser.add_argument('--f_width', default=2.85, type=float, help='Width of filament used')

    parser.add_argument('--skip_connection', action='store_true', help='If specified, skips the connection to robot. For testing out-of-lab. Also defaults too True if visualize is True')

    args = parser.parse_args()

    tool = ExtruderTool(args.tool,'FDM',args.f_width,args.d_nozzle,args.t_tool,args.skip_connection)
     
    #tool.set_feedrate(0) 
    #tool.set_nozzletemp(0)

    
    temp = 200
    feedrate = 150 # Maybe change to 1800 ####################################

    # Set velocity, acceleration and jerk to 1% of the maximum
    
    print("INFO: Press EMERGENCY STOP button DOWN and position the robot manually.")
    print("After positioning robot, open the EMERGENCY STOP button to its UP position! The robot should the have BLUE lights")
    input("Position end-effector nozzle 10 cm over the bed\nWhen satisfied with position, press Enter to continue...")
    
    # Heat up extruder
    tool.set_nozzletemp(temp)

    while tool.read_temperature() < (temp - 5.0):
        print(tool.read_temperature())
        pass
        
    # Start extruder
    tool.set_feedrate(feedrate) 
    input("Press enter when the material is continously deposited")
    tool.set_feedrate(0) 
    
    # Make a 10 cm line feed rate constant
    print("Manually move the robot to a satisfactory starting point for printing")
    input("Press enter when the nozzle position for printing is satisfactory.")
     
    tool.set_feedrate(100)
    sleep(10)


    # Disconnect extruder
    tool.set_feedrate(0.0) 
    tool.set_nozzletemp(0.0)
    tool.disconnect



