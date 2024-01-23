# import sys
import argparse
import time
import math

from am_robot.GCodeExecutor import GCodeExecutor
from am_robot.ExtruderTool import ExtruderTool
from am_robot.FrankaRobot import FrankaRobot
from am_robot.MotionPlanner import MotionPlanner
from am_robot.Transformations import Transformations

import logging
log = logging.getLogger(__name__)


def main():
    '''
    Additive manufactureing package for the Franka Emika Panda robot manipulator.

    Arguments:
    -----------
    host: string
        ip string used for robot connection (default: 10.0.0.2)
    tool: string
        serial string used for tool connection (default: /dev/ttyUSB0)
    gfile: string
         string name of gcode file used for additive manufacturing, with or without .gcode ending (default: Circle.gcode)
    Returns:
    -----------
    3D object
    '''

    ''' Parsing input arguments '''

    parser = argparse.ArgumentParser(
        formatter_class=argparse.MetavarTypeHelpFormatter,
        description=('''Package for controlling a 3D printing on a 7 DoF robotic arm'''),
        epilog='This is still under development',
        add_help=True)

    parser.add_argument('--host', default='10.0.0.2', type=str, help='FCI IP of the robot')
    parser.add_argument('--tool', default='/dev/ttyUSB0', type=str, help='Serial connection of the tool used')
    parser.add_argument('--home_mode', default='Guiding', type=str, help='Mode type for homing to (0,0) of Gcode point. Guiding to manually position end-effector nozzle')
    parser.add_argument('--gfile', default='Circle.gcode', type=str, help='Gcode file name')

    parser.add_argument('--lines', default=100000000, type=int, help='Max number of lines to process, default is higher than ever expected')
    parser.add_argument('--skip_probe', action='store_true', help='If specified, skips the bed probing step')
    #This does not work for now parser.add_argument('--use_pose_transformation', default=[0,0,0,0,0,0], type=list, help='If specified, changes the plane bed with the affine transformation: [x,y,z,z_rot,y_rot,x_rot]')
    args = parser.parse_args()

    time_elapsed_task = time.time()
    time_elapsed_total = time.time()
    
    use_pose_transformation = [0,0.005,0.03598076211,60*math.pi/180,0,0]
    use_pose_transformation = [0, 0, 0, 0, 0, 0]

    tool = ExtruderTool(args.tool,'FDM')
    robot = FrankaRobot(args.host)
    transformations= Transformations(use_pose_transformation,robot)
    motionPlanner = MotionPlanner(args.gfile,robot,transformations)
    executor = GCodeExecutor(robot,motionPlanner,tool,transformations)

    motionPlanner.load_gcode(args.lines)

    print("Done pre-processing gcode")

    if executor.robot.is_connected:
        # Manually position end effector/extrusion nozzle at 'home' point
        executor.home_gcode(args.home_mode)

        # Uses force feedback to determine where n points of the print bed are located

        if not args.skip_probe:
            bed_points = executor.probe_bed(False, use_pose_transformation)
        else:
            bed_points = executor.probe_bed(True, use_pose_transformation)
                
        if bed_points:
            transformations.set_bed_points(bed_points)

            transformations.fra_probe_bed()
            time_elapsed_task = time.time()

            try:
                executor.run_code_segments()
            except KeyboardInterrupt:
                tool.set_feedrate(0.0)
                tool.set_nozzletemp(0.0)
                exit()

            time_elapsed_task = time.time() - time_elapsed_task

        else:
            print("One of more points of the bed was not found, check and level bed roughly")

    time_elapsed_total = time.time() - time_elapsed_total

    print(f"Task done in {time_elapsed_task:.5f}s")
    print(f"Total time elapsed: {time_elapsed_total:.5f}s")
   
    tool.set_feedrate(0.0)
    tool.disconnect()

    
if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s: %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    handlers=[logging.FileHandler("robot_debug.log"),
                              logging.StreamHandler()])
    main()
    
  
   

    