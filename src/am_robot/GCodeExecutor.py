import os
import time
import math
import sys
import pickle

import argparse
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import matplotlib.pyplot as plt

from gcodeparser import GcodeParser
from am_robot.GCodeCommands import GCodeCommands
from mgen import rotation_from_angles


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
    def __init__(self,filename,robot,tool):
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

        self.filename_ = filename
        self.interval = [0,0]  # On the assumption that the first and second gcode command will allways be unique from eachother. This is a valid assumption
        self.list_of_intervals = []

        # initial values that should never be used before finding new values anyway
        self.prev_X = 0
        self.prev_Y = 0
        self.prev_Z = 0
        self.prev_E_total = 0
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.E = 0
        self.F = 0
        self.x_rot = 0.0
        self.y_rot = 0.0
        self.move_type = 'idle'
        self.extrusion_mode = 'absolute'
        self.X_positioning = 'abs'
        self.Y_positioning = 'abs'
        self.Z_positioning = 'abs'

        self.robot = robot
        self.tool = tool

        # default planar bed
        self.bed_plane_abcd = [0,0,0,0]
        self.next_segment = False
        #self.use_frenet_serret = True
        
        self.bed_plane_transformation_matrix = None
        self.tool_frame = None

    def get_interval(self):
        '''
        Returns the current G-code interval

        Returns:
        -----
        self.interval: [int,int]
            list of index of start and end line of interval in G-code
        '''
        return self.interval

    #  Set the current interval of gcode lines
    def set_interval(self,interval):
        '''
        Sets the G-code interval

        Input:
        -----
        interval: [int,int]
            Sets the interval of indexes for the start and end line of G-code interval
        '''
        self.interval = interval

    def append_interval(self):
        ''' Appends current self.interval to a list of intervals '''
        self.list_of_intervals.append(self.interval)

    def read_command(self,line_number):
        '''
        Returns the command for the given G-code line number

        Input:
        -----
        line_number: int
            G-code line index for reading command

        Returns:
        -----
        command: string
            Command for the specific line as 'letter+number' i.e. 'G28'

        '''
        return self.gcodelines[line_number].command[0] + str(self.gcodelines[line_number].command[1])


    # read given param from gcode
    def read_param(self,line_number,param):
        '''
        Reads the parameter specified by 'param' for the given G-code line. If no such param exists, returns False

        Input:
        -----
        line_number: int
            G-code line index for reading parameter
        param: string
            String type of parameter to read, i.e. 'X' or 'F'

        Returns:
        -----
        param_value: float
            Float value of parameter

        '''
        try:
            return self.gcodelines[line_number].params[param]
        except:
            return False

    #  get the current param from self
    def get_param(self,param):
        '''
        Gets the current value of the parameter

        Input:
        -----
        param: string
            String type of parameter to return, i.e. 'X' or 'F'

        Returns:
        -----
        param_value: float
            Float value of parameter

        '''
        return self.__dict__[param]

    #  set the current param to self
    def set_param(self,line_number,param):
        '''
        Sets the parameter value from the given G-code line to the object

        Input:
        -----
        line_number: int
            G-code line index for reading parameter
        param: string
            String type of parameter to set, i.e. 'X' or 'F'

        '''
        if self.read_param(line_number,param) is not False:
            self.__dict__[param] = self.read_param(line_number,param)

    def set_extremes(self,param,extreme):
        '''
        Update the extremity values of given parameter

        Input:
        -----
        param: string
            String type of parameter to update, i.e. 'X' or 'F'
        extreme: float
            Value of paremeter to update with

        '''
        try:
            if param > self.__dict__[extreme][1]:
                self.__dict__[extreme][1] = param
            if param < self.__dict__[extreme][0]:
                self.__dict__[extreme][0] = param
        except:
            self.__dict__[extreme] = [param,param]

    def set_params(self,line_number):
        '''
        Sets all present parameters for the given G-code line

        Input:
        -----
        line_number: int
            G-code line index to read parameters from
        '''
        for key in self.gcodelines[line_number].params:
            self.__dict__[key] = self.read_param(line_number,key)

    def set_prev_xyz(self):
        self.prev_X = self.X
        self.prev_Y = self.Y
        self.prev_Z = self.Z

    # Find the next instance of retraction (NB may be 0mm retraction)
    def find_next_interval(self):
        '''
        Finds the next interval bounded by certain criteria from when an interval should end and adds the interval to the list of intervals. Critria include change in material extrusion direction, command type change, layer change, change in process speed, etc..
        
        Returns:
        -----
        'End of code': string
            String indicating end of G-code has been reached

        '''
        '''
        priority:
        change on command change
        change on retraction/reversing of material/tool feedrate

        '''
        if self.list_of_intervals == []:
            #self.append_interval()
            self.list_of_intervals.append(self.interval)

        if self.interval[1]+1 >= self.number_of_lines:
            print("End of code")
            return 'End of code'

        line_number = self.interval[1]+1
        interval = [line_number,line_number]
        while (line_number < self.number_of_lines):
            # Find dimensions of model
            if self.read_param(line_number,'X') is not False:
                self.set_extremes(self.read_param(line_number,'X'),'Xmax')
            if self.read_param(line_number,'Y') is not False:
                self.set_extremes(self.read_param(line_number,'Y'),'Ymax')
            if self.read_param(line_number,'Z') is not False:
                self.set_extremes(self.read_param(line_number,'Z'),'Zmax')
            if self.read_param(line_number,'F') is not False:
                self.set_extremes(self.read_param(line_number,'F'),'Fmax')

            # Set relative process speed to override default absolute
            if self.read_command(line_number) == 'M83':
                self.extrusion_mode = 'relative'

            # Find desired interval change condition
            # Check if next line has a command has changed
            if self.read_command(line_number) != self.read_command(self.interval[1]+1):
                interval = [self.interval[1]+1,line_number-1]
                break

            # Check reversing of material extrusion
            elif self.read_param(line_number,'E') is not False and self.read_param(line_number,'E') < self.get_param('E') and self.extrusion_mode == 'absolute':
                interval = [self.interval[1]+1,line_number-1]
                break

            # Check for change in process speed
            elif self.read_param(line_number,'F') is not False and self.read_param(line_number,'F') != self.get_param('F'):
                interval = [self.interval[1]+1,line_number-1]
                break

            # Check if extrusion should stop
            elif self.read_param(line_number-1,'E') is not False and self.read_param(line_number,'E') is False and self.read_command(line_number-1) == 'G1' and line_number != interval[0]:
                interval = [self.interval[1]+1,line_number-1]
                break

            # Check if extrusion should start
            elif self.read_param(line_number,'E') is not False and self.read_param(line_number-1,'E') is False and self.read_command(line_number-1) == 'G1' and line_number != interval[0]:
                interval = [self.interval[1]+1,line_number-1]
                break

            # Check if no movement or extrusion
            elif self.read_param(line_number,'X') is False and self.read_param(line_number,'Y') is False and self.read_param(line_number,'Z') is False and self.read_param(line_number,'E') is False:
                interval = [self.interval[1]+1,line_number-1]
                break

            else:
                # Typically when the next line is just another X-Y coordinate
                self.set_prev_xyz()
                self.set_params(line_number)
                line_number = line_number + 1

        if interval[1] < interval[0]:
            interval = [line_number,line_number]
            self.set_prev_xyz()
            self.set_params(line_number)
     
        self.set_interval(interval)
        self.list_of_intervals.append(self.interval)

    def find_intervals(self):
        ''' Calls find_next_interval() repeatedly until all lines have been added to an interval '''
        while self.number_of_lines > self.interval[1] + 1:
            self.find_next_interval()
        self.reset_parameters()

    def load_gcode(self,lines):
        ''' Finds the 'filename' file with '.gcode' extension in the data folder and parses the file using GcodeParser. X, Y, Z parameters are converted to the size used by the robot, i.e. G-code millimeter is changed to robot meter. Calls find_intervals() to starts pre-processing of G-code. '''
        # Add .gcode if not given
        filename, extension = os.path.splitext(self.filename_)
        if extension == '':
            print("No file extension given, assuming '.gcode'")
            extension = '.gcode'
        elif extension.lower() == '.gcode':
            filename, extension = os.path.splitext(self.filename_)
        else:
            print("File extension not empty or .gcode")
            input("finding for '.gcode' instead. Enter to continue...")
            extension = '.gcode'

        cwd = os.getcwd()

        for root, dirs, files in os.walk(cwd):
            if filename+extension in files:  
                fullpath = os.path.join(root, filename+extension)

        # Read file
        with open(fullpath,'r') as file:
            gcodedata = file.read()

        # Change instances of "-." to "-0.". Was causing issues with visualization plot
        gcodedata = gcodedata.replace('-.','-0.')

        # Parse lines into Dict object
        self.gcodelines = GcodeParser(gcodedata).lines

        # gcode is in mm, change to m for robot
        for line in self.gcodelines:
            for element in line.params:
                if element == 'X':
                    line.update_param('X',line.get_param('X')/1000.0)
                if element == 'Y':
                    line.update_param('Y',line.get_param('Y')/1000.0)
                if element == 'Z':
                    line.update_param('Z',line.get_param('Z')/1000.0)

        self.number_of_lines = len(self.gcodelines)
        if self.number_of_lines > lines:
            self.number_of_lines = lines
        self.find_intervals()

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
                gcode_home_pose_vec = pickle.load(file)

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
        transformed_point = np.matmul(self.bed_plane_transformation_matrix,base_point)
        if self.next_segment:
            transformed_point = np.matmul(self.active_plane,transformed_point)
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

        if contact_found:
            self.bed_points = bed_grid
            self.calculate_bed_surface_plane(use_pose_transformation)
            self.calculate_bed_rotation_matrice()
            self.gcode_home_pose_vec = [x + y for x, y in zip(bed_grid[1][1],use_pose_transformation[0:3])]
            print(f"Gcode home location: {self.gcode_home_pose_vec}")
        else:
            print("One or more bed points was not found")

        self.robot.recover_from_errors()

        self.robot.set_dynamic_rel(0.1)
        m1 = self.robot.robot_home_pose* self.robot.make_affine_object(0, 0.0, 0.2,0,0,0) 
        m1 = self.robot.make_linear_motion(m1)
        self.robot.execute_move(frame=self.robot.tool_frame,motion=m1)

        self.robot.recover_from_errors()

        return contact_found

    def calculate_bed_surface_plane(self, use_pose_transformation):
        '''
        Calculate the plane equation given by ax + by + cz + d = 0 with points taken from the bed probing process. Uses 3 non-collinear points A, B and C and calculates the cross product AB x AC = [a,b,c] and d = -(aAx + bAy + cAz)
        '''
        # Points
        A = self.bed_points[1][1]
        B = self.bed_points[2][1]
        C = self.bed_points[1][2]

        AB = [B[0]-A[0],B[1]-A[1],B[2]-A[2]]
        AC = [C[0]-A[0],C[1]-A[1],C[2]-A[2]]

        ABxAC = np.cross(AB,AC)
        a = ABxAC[0]
        b = ABxAC[1]
        c = ABxAC[2]
        A = A + [0.015*a,0.015*b,0.015*c]
        d = 0 # Is not used. d = -(a*A[0] + b*A[1] + c*A[2])
        self.bed_plane_abcd = [a,b,c,d]
        if use_pose_transformation:
            print('use_pose_transformation[0:6]: ',use_pose_transformation[0:6] )
            self.bed_plane_abcd[0:3] = np.matmul(self.rotation_matrix(use_pose_transformation[3] ,use_pose_transformation[4] ,use_pose_transformation[5]), self.bed_plane_abcd[0:3])

    def calculate_bed_rotation_matrice(self):
        offset_z_probe_to_bed = -0.00075 # From probing, the robot moved 0.0015 m "under the bed" due to the robot having flexible joints

        bed_normal_vec = np.array(self.bed_plane_abcd[0:3])
        robot_normal_vec = np.array([0.0,0.0,-1.0])  # Normal vector down

        bed_normal_vec_x = np.copy(bed_normal_vec)
        bed_normal_vec_x[1] = 0.0
        bed_normal_vec_y = np.copy(bed_normal_vec)
        bed_normal_vec_y[0] = 0.0

        y_rot = self.angle_between(bed_normal_vec_x,robot_normal_vec)
        x_rot = self.angle_between(bed_normal_vec_y,robot_normal_vec)

        print(f"Angle between plane normals: {self.angle_between(bed_normal_vec,robot_normal_vec)*180/math.pi}")
        print(f"Angle between plane normals - Rotation around x-axis: {x_rot*180/math.pi}")
        print(f"Angle between plane normals - Rotation around y-axis: {y_rot*180/math.pi}")

        T_robot_bed = self.robot.make_affine_object(0.0, 0.0, offset_z_probe_to_bed, a=0.0, b=y_rot, c=-x_rot) 
   
        self.robot.tool_frame = self.robot.tool_frame * T_robot_bed 

        print('Tool frame: ', self.robot.tool_frame)
        self.bed_plane_transformation_matrix = self.rotation_matrix(x_rot=x_rot,y_rot=-y_rot)
        print('Rotation matrix bed plane: ', self.bed_plane_transformation_matrix)


    def rotation_matrix(self,x_rot=0.0,y_rot=0.0,z_rot=0.0):
        return rotation_from_angles([z_rot, y_rot, x_rot], 'ZYX')

    def unit_vector(self,vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def angle_between(self,v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'. """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -3.14, 3.14))
        if np.count_nonzero(v1) == len(v1):
            return angle
        else:
            values = v1[np.nonzero(v1)]
            return np.sign(values[0])*angle

   
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
        if self.extrusion_mode == 'absolute':
            self.path_extrusion = 0
            self.prev_E_total = self.E
        else:
            self.path_extrusion = 0

        path_points = []
        start_pose = self.robot.read_current_pose()
        z_translation = start_pose.vector()
        for point in range(interval[0],interval[1]+1):
            start_point = np.array([self.X,self.Y,self.Z])
            for key in self.gcodelines[point].params:
                if key == 'X' or key == 'Y' or key == 'Z':
                    if self.X_positioning == 'rel' or self.Y_positioning == 'rel' or self.Z_positioning == 'rel':
                        self.__dict__[key] += self.read_param(point,key)
                    else:
                        self.__dict__[key] = self.read_param(point,key)
                if key == 'E':
                    if self.extrusion_mode == 'abs':
                        self.__dict__[key] = self.read_param(point,key)
                        self.path_extrusion = self.__dict__[key]
                    else:
                        self.__dict__[key] += self.read_param(point,key)
                        self.path_extrusion = self.__dict__[key]
            base_point = np.array([self.X,self.Y,self.Z])
            transformed_point = np.matmul(self.bed_plane_transformation_matrix,base_point)
            if self.next_segment is True:
                transformed_point = np.matmul(self.active_plane,transformed_point)

            # Pure translation for non-extrusion move, using previous rotation
            if self.read_param(point,'E') is False:
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
        if self.extrusion_mode == 'absolute':
            self.path_extrusion -= self.prev_E_total

        return path_motion, path
    
    def run_code_segments(self):
        prev_progress = 0
        self.robot.set_velocity_rel(1.0)
        for interval in self.list_of_intervals: 
            progress = math.floor((100 * interval[0])/(self.number_of_lines - 1.0)) #PERCENTAGE
            if progress > prev_progress:
                print(f"Current progress is {progress}%, on line {interval[0]} of {self.number_of_lines}.")
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
        self.command = self.read_command(self.interval[0])
        # Handle different M (machine) commands
        if self.command[0] == 'M':
            # Call method for each M-command (M79(),M42(), etc)
            getattr(self,self.command,getattr(self,'default'))()

        elif self.command[0] == 'G':
            # Find desired feedrate / working speed / max velocity
            if self.read_param(self.interval[0],'F') is not False:
                self.F = self.read_param(self.interval[0],'F')

            # Call method for G-command
            getattr(self,self.command,getattr(self,'default'))()

        else:
            print(f"Command other than M or G... Silently passing on command {self.command}")
            pass
    
    def reset_parameters(self):
        '''
        Resets parameters to zero
        '''
        # corresponding to 0,0,0 of start of printing
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.F = 0
        self.E = 0
        self.set_prev_xyz()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=True)

    parser.add_argument('--gfile',default='Circle')

    args = parser.parse_args()

    model = GCodeExecutor(args.gfile,{},{})
    model.load_gcode()
    model.visualize_gcode()
