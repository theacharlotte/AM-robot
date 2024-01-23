
class AbstractRobot():
    '''
    Detailing robot methods used by main and GCodeExecutor...
    '''
    def __init__(self,host):
        self.host = host
        self.robot = {}  # Change
    
    def robot_init_move(self):
        print("Function 'robot_init_move()' missing in Specific robot Class, please implement...")
        print("Assumed function: Move robot to desired initial joint configuration")

    def read_current_pose(self):
        print("Function 'read_current_pose()' missing in Specific robot Class, please implement...")

    def recover_from_errors(self):
        print("Function 'recover_from_errors()' missing in Specific robot Class, please implement...")

    def set_dynamic_rel(self,value):
        print("Function 'set_dynamic_rel(value)' missing in Specific robot Class, please implement...")

    def set_velocity_rel(self,value):
        print("Function 'set_velocity_rel()' missing in Specific robot Class, please implement...")

    def set_acceleration_rel(self,value):
        print("Function 'set_acceleration_rel(value)' missing in Specific robot Class, please implement...")

    def set_jerk_rel(self,value):
        print("Function 'set_jerk_rel(value)' missing in Specific robot Class, please implement...")

    def execute_move(self,frame=None,motion=None, data=None):
        print("Function 'execute_move(frame,motion)' missing in Specific robot Class, please implement...")
    
    def execute_threaded_move(self,frame=None,motion=None,data=None):
        print("Function 'execute_async_move(frame,motion,data)' missing in Specific robot Class, please implement...")

    def make_linear_motion(self,affine):
        print("Function 'make_linear_move(affine)' missing in Specific robot Class, please implement...")

    def make_linear_relative_motion(self,affine):
        print("Function 'make_linear_reaction_move(affine)' missing in Specific robot Class, please implement...")
    
    def make_affine_object(self,x,y,z,a=0,b=0,c=0):
        print("Function 'make_affine_object(x,y,z,a,b,c)' missing in Specific robot Class, please implement...")

    def make_path_motion(self,path_points,blending_distance):
        print("Function 'make_path_motion(frame,motion,data)' missing in Specific robot Class, please implement...")
