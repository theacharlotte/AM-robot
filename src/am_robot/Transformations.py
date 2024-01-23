import numpy as np
import math
from mgen import rotation_from_angles



class Transformations():
    def __init__(self,use_pose_transformation,robot):
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

        self.bed_points = [[[],[],[]],[[],[],[]],[[],[],[]]] 
        self.use_pose_transformation = use_pose_transformation
        self.robot = robot
        
    def set_bed_points(self, bed_points):
        self.bed_points = bed_points
        
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
        
    def fra_probe_bed(self):
        #self.bed_points = bed_grid
        self.calculate_bed_surface_plane(self.use_pose_transformation)
        self.calculate_bed_rotation_matrice()
        