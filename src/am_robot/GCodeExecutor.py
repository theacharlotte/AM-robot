import time
import math
import sys
import pickle

import argparse
import numpy as np

#from gcodeparser import GcodeParser
from am_robot.GCodeCommands import GCodeCommands
from am_robot.MotionPlanner import MotionPlanner


if sys.platform == 'linux':
    from frankx import Measure, MotionData, Reaction

import logging
log = logging.getLogger(__name__)

class GCodeExecutor(GCodeCommands):
    '''
    Read and parse gcode into a full object variable

    Attributes:
    -----------
    filename: string
        filename to look for when loading G-code
    robot: Class
        Class object of the robot used
    tool: Class
        Class object of the tool head used

    Methods:
    -----------
    load_gcode():
        finds and runs pre-processing of G-code given by the attribute 'filename'
    run_code_segment():
        Used to run a single interval of G-code, such as a motion trajectory or machine setting change
    probe_bed():
        Used to run the bed probing sequence for determining bed flatness
    '''
    def __init__(self,robot,motionPlanner,tool, transformations):
        '''
        Initializes the Class object

        Input:
        -----
        filename: string
            filename to look for when loading G-code
        robot: Class
            Class object of the robot used
        tool: Class
            Class object of the tool head used

        Returns:
        -----
        Initialized Class object

        '''
        super().__init__()
        self.motionPlanner = motionPlanner
        #self.filename_ = filename
        #self.interval = [0,0]  # On the assumption that the first and second gcode command will allways be unique from eachother. This is a valid assumption
        self.list_of_intervals = motionPlanner.list_of_intervals

        # initial values that should never be used before finding new values anyway
        #self.prev_X = 0
        #self.prev_Y = 0
        #self.prev_Z = 0
        #self.prev_E_total = 0
        #self.X = 0
        #self.Y = 0
        #self.Z = 0
        #self.E = 0
        self.F = 0
        self.x_rot = 0.0
        self.y_rot = 0.0
        #self.move_type = 'idle'
        self.extrusion_mode = 'absolute'
        #self.X_positioning = 'abs'
        #self.Y_positioning = 'abs'
        #self.Z_positioning = 'abs'

        self.robot = robot
        self.tool = tool
        self.transformations = transformations

        # default planar bed
        self.bed_plane_abcd = [0,0,0,0]
        self.next_segment = False
        
        self.bed_plane_transformation_matrix = None
        self.tool_frame = None


    

    def home_gcode(self,homing_type):
        '''
        Pauses to let the operator manually position the nozzle a small distance from the desired zero point of the G-code. A known point can also be used

        Input:
        -----
        homing_type: string
            Either 'Guiding' or 'known'. Guiding pauses to let the operator manually adjust nozzle position, known uses pre-provided position and continues.
        '''
        if homing_type == 'Guiding':
            self.robot.robot_init_move()

            print("INFO: To enter guiding mode, EMERGENCY STOP button needs to be pressed DOWN and the robot light should be continuously WHITE!")
            print("After positioning robot, open the EMERGENCY STOP button to its UP position! The robot should the have BLUE lights")
            input("Position end-effector nozzle < 1cm from desired (0,0) location.\nWhen satisfied with position, press Enter to continue...")

            self.gcode_home_pose_vec = self.robot.read_current_pose().vector()
            
            with open('gcode_home_pose_vec.pkl', 'wb') as file:
                pickle.dump(self.gcode_home_pose_vec, file)

        elif homing_type == 'known':
            self.robot.robot_init_move()

            print("Set gcode_home to this value. Home point assumed known")
            with open('gcode_home_pose_vec.pkl', 'rb') as file:
                self.gcode_home_pose_vec = pickle.load(file)

            #self.gcode_home_pose_vec = gcode_home_pose_vec *0

            print('gcode_home_pose_vec sat to: ', self.gcode_home_pose_vec)
            #self.gcode_home_pose_vec = [0.32325372, 0.09250938, -0.11237811,0.0,0.0,0.0] # Changes to where you want home ved to be
        else:
            print("Failed to home gcode zero... Check RobotMode input")

    def move_to_point(self,x,y,z):      
        '''
        Move to desired location, using x,y,z and G-code home pose offset and tool_pose reference frame

        Input:
        -----
        x: float
            X value of target position
        y: float
            Y value to target position
        z: float
            Z value of target position

        '''
        self.robot.set_dynamic_rel(0.1) # vel
        base_point = np.array([x,y,z])
        transformed_point = np.matmul(self.transformations.bed_plane_transformation_matrix,base_point)
        motion = self.robot.make_linear_motion(self.robot.make_affine_object(transformed_point[0] + self.gcode_home_pose_vec[0],transformed_point[1] + self.gcode_home_pose_vec[1],transformed_point[2] + self.gcode_home_pose_vec[2]))
        self.robot.execute_move(frame=self.robot.tool_frame,motion=motion)
        self.robot.recover_from_errors()

    def probe_bed(self, skip_probe, use_pose_transformation):
        '''
        Probe the bed in a grid using motion force feedback to detect surface. Contact points are used to calculate the surface flatness.

        Returns:
        -----
        contact_found: bool
            flag to inticate whether all probe points detected the surface or not. True if all points were successful.

        '''
        print("probing...")
        probe_locations_xy = [[[0.05,0.05],[0,0.05],[-0.05,0.05]],[[0.05,0],[0,0],[-0.05,0]],[[0.05,-0.05],[0,-0.05],[-0.05,-0.05]]]  # relative to ghome_gcode

        bed_grid = [[[],[],[]],[[],[],[]],[[],[],[]]]  # ugly ik

        contact_found = True

        if skip_probe == False:
            for axis1 in range(3):
                for axis2 in range(3):
    
                    self.robot.set_dynamic_rel(0.1)
                    # Move to probe location
                    affine1 = self.robot.make_affine_object(probe_locations_xy[axis1][axis2][0] + self.gcode_home_pose_vec[0],probe_locations_xy[axis1][axis2][1] + self.gcode_home_pose_vec[1],self.gcode_home_pose_vec[2]+0.02)
                    m1 = self.robot.make_linear_motion(affine1)
                    self.robot.execute_move(frame=self.robot.tool_frame,motion=m1)

                    # Reset data reaction motion, may need to tweek trigger force when extruder is mounted
                    d2 = MotionData().with_reaction(Reaction(Measure.ForceZ < -5.0))

                    # Reduce dynamics for probing
                    self.robot.set_dynamic_rel(0.02)

                    # Move slowly towards print bed
                    affine2 = self.robot.make_affine_object(0.1,0.0,-0.1)
                    m2 = self.robot.make_linear_relative_motion(affine2)
                    self.robot.execute_move(motion=m2,data=d2)

                    # Check if the reaction was triggered
                    if d2.did_break:
                        self.robot.recover_from_errors()
                        print('Force exceeded 5N!')
                        print(f"Hit something for probe location x: {axis1}, y: {axis2}")

                        current_pose = self.robot.read_current_pose()
                        vector_pose = current_pose.vector()
                        probe_point = vector_pose[0:3]
                        bed_grid[axis1][axis2] = probe_point

                    elif not d2.did_break:
                        print(f"Did not hit anything for probe location x: {axis1}, y: {axis2}")
                        contact_found = False
                    
            # Save bed grid to file for use later without probing
            with open('bed_grid.pkl', 'wb') as file:
                pickle.dump(bed_grid, file)
            
        else:
            # Read bed_points from the file
            with open('bed_grid.pkl', 'rb') as file:
                bed_grid = pickle.load(file)
            
            if not bed_grid:
                contact_found = False

        self.robot.recover_from_errors()
        self.robot.set_dynamic_rel(0.1)
        m1 = self.robot.robot_home_pose* self.robot.make_affine_object(0, 0.0, 0.02,0,0,0) 
        m1 = self.robot.make_linear_motion(m1)
        self.robot.execute_move(frame=self.robot.tool_frame,motion=m1)
        self.robot.recover_from_errors()

        self.gcode_home_pose_vec = [x + y for x, y in zip(bed_grid[1][1],use_pose_transformation[0:3])]
        print(f"Gcode home location: {self.gcode_home_pose_vec}")
        
        if contact_found:
            return bed_grid
        else:
            print("One or more bed points was not found")
            
    

   
    
    def run_code_segments(self):
        prev_progress = 0
        self.robot.set_velocity_rel(1.0)
        for interval in self.list_of_intervals: 
            progress = math.floor((100 * interval[0])/(self.motionPlanner.number_of_lines - 1.0)) #PERCENTAGE
            if progress > prev_progress:
                print(f"Current progress is {progress}%, on line {interval[0]} of {self.motionPlanner.number_of_lines}.")
                prev_progress = progress
            self.interval = interval
            start_time = time.time()
            self.run_code_segment()
            elapsed_time = time.time()-start_time
            log.debug(f"Segment took in seconds to complete segment {elapsed_time}")
            log.debug(f"Segment printed: {interval}")

    # Blocking action
    def run_code_segment(self):
        '''
        Processes one interval based on the Command in that interval. Machine settings, movement, extrusion is set based on command and parameters.
        '''
        self.command = self.motionPlanner.read_command(self.interval[0])
        # Handle different M (machine) commands
        if self.command[0] == 'M':
            # Call method for each M-command (M79(),M42(), etc)
            getattr(self,self.command,getattr(self,'default'))()

        elif self.command[0] == 'G':
            # Find desired feedrate / working speed / max velocity
            if self.motionPlanner.read_param(self.interval[0],'F') is not False:
                self.F = self.motionPlanner.read_param(self.interval[0],'F')

            # Call method for G-command
            getattr(self,self.command,getattr(self,'default'))()

        else:
            print(f"Command other than M or G... Silently passing on command {self.command}")
            pass

    def make_path(self,interval,corner_blending,motion_data):
        '''
        Generate waypoints for path following with a blending distance for smoothing motion when changing taget waypoint

        Input:
        -----
        interval: [int,int]
            Interval for which to make the PathMotion object for
        corner_blend_threshold: float
            radius for which a new waypoint should be used as motion target

        Returns:
        -----
        path: PathMotion object

        '''
        """if self.extrusion_mode == 'absolute':
            self.path_extrusion = 0
            self.prev_E_total = self.E
        else:
            self.path_extrusion = 0"""

        path_points = []
        start_pose = self.robot.read_current_pose()
        z_translation = start_pose.vector()
        for point in range(interval[0],interval[1]+1):
            start_point = np.array([self.motionPlanner.X,self.motionPlanner.Y,self.motionPlanner.Z])
            for key in self.motionPlanner.gcodelines[point].params:
                if key == 'X' or key == 'Y' or key == 'Z':
                    if self.motionPlanner.X_positioning == 'rel' or self.motionPlanner.Y_positioning == 'rel' or self.motionPlanner.Z_positioning == 'rel':
                        self.__dict__[key] += self.motionPlanner.read_param(point,key)
                    else:
                        self.__dict__[key] = self.motionPlanner.read_param(point,key)
                """if key == 'E':
                    if self.extrusion_mode == 'abs':
                        self.__dict__[key] = self.motionPlanner.read_param(point,key)
                        self.path_extrusion = self.__dict__[key]
                    else:
                        self.__dict__[key] += self.motionPlanner.read_param(point,key)
                        self.path_extrusion = self.__dict__[key]"""
            base_point = np.array([self.motionPlanner.X,self.motionPlanner.Y,self.motionPlanner.Z])
            transformed_point = np.matmul(self.transformations.bed_plane_transformation_matrix,base_point)

            # Pure translation for non-extrusion move, using previous rotation
            if self.motionPlanner.read_param(point,'E') is False:
                # y_rot = -z_translation[4]
                # x_rot = -z_translation[5]
                # if abs(y_rot) > abs(self.y_rot):
                #     self.y_rot = y_rot
                # if abs(x_rot) > abs(self.x_rot):
                #     self.x_rot = x_rot
                if point == interval[0]:
                    path_points.append(start_pose)
            else:
                if point == interval[0]:
                    first_point = self.robot.make_affine_object(z_translation[0],z_translation[1],z_translation[2],b=self.y_rot,c=-self.x_rot)
                    path_points.append(first_point)
                    self.robot.execute_move(frame=self.robot.tool_frame,motion=self.robot.make_linear_motion(first_point),data=motion_data)
                    self.robot.recover_from_errors()
            affine = self.robot.make_affine_object(transformed_point[0] + self.gcode_home_pose_vec[0],transformed_point[1] + self.gcode_home_pose_vec[1],transformed_point[2] + self.gcode_home_pose_vec[2],b=self.y_rot,c=-self.x_rot)
            path_points.append(affine)
        path_motion, path = self.robot.make_path_motion(path_points,corner_blending)
        """if self.extrusion_mode == 'absolute':
            self.path_extrusion -= self.prev_E_total"""

        return path_motion, path
    
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=True)

    parser.add_argument('--gfile',default='Circle')

    args = parser.parse_args()

    model = GCodeExecutor(args.gfile,{},{})
    model.load_gcode()
    model.visualize_gcode()
