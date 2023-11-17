# import sys
import argparse
import time
import math

from am_robot.GCodeExecutor import GCodeExecutor
from am_robot.ExtruderTool import ExtruderTool
from am_robot.FrankaRobot import FrankaRobot
import am_robot.ResearchExperiments as re

# if sys.platform == 'linux':
#     from frankx import Robot
# elif sys.platform == 'win32':
#     try:
#         from frankx import Robot
#     except Exception as e:
#         print(e)
#     finally:
#         print('Running on OS: ' + sys.platform)



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
    visualize: bool, optional
        flag for enabling visualization for gcode (default: False)
    skip_connection: bool, optional
        flag for skipping connection with robot, useful when hardware is not connected (default: False)

    Returns:
    -----------
    3D object or visualization of object
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

    parser.add_argument('--t_tool', default=[0,0,-0.1], type=list, help='Translation due to Tool as [x,y,z]') ##########
    parser.add_argument('--d_nozzle', default=0.8, type=float, help='Hot-End Nozzle diameter')
    parser.add_argument('--f_width', default=2.85, type=float, help='Width of filament used')

    parser.add_argument('--visualize', action='store_true', help='If specified, visualize the given Gcode as a 3D plot')
    parser.add_argument('--lines', default=100000000, type=int, help='Max number of lines to process, default is higher than ever expected')
    parser.add_argument('--skip_connection', action='store_true', help='If specified, skips the connection to robot. For testing out-of-lab. Also defaults too True if visualize is True')
    parser.add_argument('--skip_probe', action='store_true', help='If specified, skips the bed probing step')
    parser.add_argument('--skip_segments', action='store_true', help='If specified, skips the G-code segments')
    args = parser.parse_args()

    time_elapsed_task = time.time()
    time_elapsed_total = time.time()

    tool = ExtruderTool(args.tool,'FDM',args.f_width,args.d_nozzle,args.t_tool,args.skip_connection)
    robot = FrankaRobot(args.host,args.skip_connection)
    executor = GCodeExecutor(args.gfile,robot,tool)
    executor.load_gcode(args.lines)

    print("Done pre-processing gcode")

    if args.visualize:
        time_elapsed_task = time.time()
        executor.display()
        executor.visualize_gcode()
        time_elapsed_task = time.time() - time_elapsed_task

        print(f"Visualization done in {time_elapsed_task:.5f}s")

        input("Press Enter to continue if satisfied with model plot...")

    if executor.robot.is_connected:
        # Manually position end effector/extrusion nozzle at 'home' point
        executor.home_gcode(args.home_mode)

        # Check bounds for build area
        # proceed = executor.is_build_feasible()
        # executor.robot.gripper.clamp(0.005)

        # Uses force feedback to determine where n points of the print bed are located
        if not args.skip_probe:
            bed_found = executor.probe_bed()
        else:
            bed_found = True
            T_bed_to_plane = executor.robot.make_affine_object(0.0, 0.0, 2.679, a=0.0, b=math.pi/12, c=0.0) # Found from calculating the rotation and translation
            executor.robot.tool_frame = executor.robot.make_affine_object(0.034670, -0.011100, -0.093030, a = 0.000000, b = -0.790352, c = 0.003971) # Found from probing the bed
            executor.robot.tool_frame = executor.robot.tool_frame * T_bed_to_plane # rotate and translate toolframe to new surface top. Rotate nozzle and move nozzle normal and on top of surface. Get correct coordinate system ift. nytt koordinatsystem
            executor.bed_plane_transformation_matrix = [[ 9.99983362e-01, -9.95404090e-06,  5.76853559e-03],
                                                        [ 0.00000000e+00,  9.99998511e-01,  1.72557245e-03],
                                                        [-5.76854417e-03, -1.72554374e-03,  9.99981873e-01]] # Found when probing the bed # rotation matrix
            R_bed_to_plane = executor.rotation_matrix(0, math.pi/12, 0)
            executor.bed_plane_transformation_matrix = executor.bed_plane_transformation_matrix * R_bed_to_plane
            # executor.bed_plane_transformation_matrix = executor.rotation_matrix()

        if bed_found and not args.skip_segments:
            # Make a bed mesh for knowing the surface flatness and location of build area
            if args.visualize:
                executor.visualize_bed_mesh()
                input("When happy with bed mesh press enter...")

            time_elapsed_task = time.time()

            try:
                executor.run_code_segments()
            except KeyboardInterrupt:
                executor.tool.set_feedrate(0.0)
                executor.tool.set_nozzletemp(0.0)
                exit()

            time_elapsed_task = time.time() - time_elapsed_task

        else:
            print("One of more points of the bed was not found, check and level bed roughly")

    time_elapsed_total = time.time() - time_elapsed_total

    print(f"Task done in {time_elapsed_task:.5f}s")
    print(f"Total time elapsed: {time_elapsed_total:.5f}s")

    if not args.skip_connection:
        executor.tool.set_feedrate(0.0)
        executor.tool.disconnect()


def main_read_temp():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.MetavarTypeHelpFormatter,
        description=('''Package for controlling a 3D printing on 7 DoF robotic arm'''),
        epilog='This is still under development',
        add_help=True)

    parser.add_argument('--tool', default='/dev/ttyUSB0', type=str, help='Serial connection of the tool used')

    parser.add_argument('--t_tool', default=[0,0,-0.1], type=list, help='Translation due to Tool as [x,y,z]')
    parser.add_argument('--d_nozzle', default=0.8, type=float, help='Hot-End Nozzle diameter')
    parser.add_argument('--f_width', default=2.85, type=float, help='Width of filament used')

    parser.add_argument('--skip_connection', action='store_true', help='If specified, skips the connection to robot. For testing out-of-lab. Also defaults too True if visualize is True')

    args = parser.parse_args()
    tool = ExtruderTool(args.tool,'FDM',args.f_width,args.d_nozzle,args.t_tool,args.skip_connection)

    while True:
        print(tool.read_temperature())


    
if __name__ == '__main__':
    main()
    #re.research_experiments_height()
    #main_read_temp()
    #re.research_experiments(False, False, True)
    #re.research_experiments2(True)
    #re.research_experiments_angle(45)
  
   

    