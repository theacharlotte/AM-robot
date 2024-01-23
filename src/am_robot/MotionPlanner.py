import os
import numpy as np

from gcodeparser import GcodeParser

from am_robot.GCodeCommands import GCodeCommands


class MotionPlanner(GCodeCommands):
    
    def __init__(self,filename,robot,transformations):
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
        #self.extrusion_mode = 'absolute'
        self.X_positioning = 'abs'
        self.Y_positioning = 'abs'
        self.Z_positioning = 'abs'
        self.number_of_lines = 0

        self.robot = robot
        self.transformations = transformations
        #self.tool = tool

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
        
            """if self.read_command(line_number) == 'M83':
                self.extrusion_mode = 'relative'"""

            # Find desired interval change condition
            # Check if next line has a command has changed
            if self.read_command(line_number) != self.read_command(self.interval[1]+1):
                interval = [self.interval[1]+1,line_number-1]
                break

            # Check reversing of material extrusion
            elif self.read_param(line_number,'E') is not False and self.read_param(line_number,'E') < self.get_param('E'): #and self.extrusion_mode == 'absolute' 
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

