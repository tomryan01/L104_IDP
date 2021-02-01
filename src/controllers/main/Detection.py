from MyRobot import MyRobot
import numpy as np

class Detection(MyRobot):

    def __init__(self):
        #calls constructor for super class
        super().__init__()
        
        #set origin/start point for robot
        self.origin = [1, 1]

    def find_wall_distance(self, robot_position, robot_orientation, wall_position, wall_direction):
        #find the intersection of two 2D vector lines
        #return the distance from robot to given wall, positive if forwards and negative if backwards
        #robot orientation should be a unit vector

        #form matrix of coeficients
        coef_matrix = np.array([[robot_orientation[0], -1*wall_direction[0]],
                                [robot_orientation[1], -1*wall_direction[1]]])
        
        #form vector of position data
        position_vector = np.array([[wall_position[0] - robot_position[0]],
                                    [wall_position[1] - robot_position[1]]])
        
        #find inverse of coeficients matrix
        inverse_coef_matrix = np.linalg.inv(coef_matrix)

        #vector lambda_solution = (lambda_robot, lambda_wall)
        #lambda_robot is the distance, if orientation was a unit vector
        lambda_solution = inverse_coef_matrix.dot(position_vector)

        #return distance found
        return lambda_solution[0][0]
        

    def block_in_sight(self):
        #Second iteration: return true if distance sensor less than wall distance
        #Must consider distance to all 4 walls
        #only accept walls infront of robot (positive distance) and choose minmum of the positive two
        #all vectors are (x, z)

        #Define the 4 walls
        wall_1_position = [1.1875, 0]
        wall_1_direction = [0, 1]
        wall_2_position = [-1.1875, 0]
        wall_2_direction = [0, 1]
        wall_3_position = [0, -1.1875]
        wall_3_direction = [1, 0]
        wall_4_position = [0, 1.1875]
        wall_4_direction = [1, 0]

        #Find position of the robot, will use front position for finding distance
        front_position_xyz = self.gpsSensors[0].getValues()
        mid_position_xyz = self.gpsSensors[1].getValues()
        front_position_xz = [front_position_xyz[0], front_position_xyz[2]]
        mid_position_xz = [mid_position_xyz[0], mid_position_xyz[2]]

        #Find orientation of the robot
        robot_orientation = []
        for i in range(2):
            robot_orientation.append(front_position_xz[i] - mid_position_xz[i])
        
        #normalise orientation vector to unit length
        norm_robot_orientation = [robot_orientation[i] / self.get_magnitude(robot_orientation) for i in range(2)]

        #find distance to each wall
        dist_wall_1 = self.find_wall_distance(front_position_xz, norm_robot_orientation, wall_1_position, wall_1_direction)
        dist_wall_2 = self.find_wall_distance(front_position_xz, norm_robot_orientation, wall_2_position, wall_2_direction)
        dist_wall_3 = self.find_wall_distance(front_position_xz, norm_robot_orientation, wall_3_position, wall_3_direction)
        dist_wall_4 = self.find_wall_distance(front_position_xz, norm_robot_orientation, wall_4_position, wall_4_direction)

        #store distances in an array
        wall_distances = [dist_wall_1, dist_wall_2, dist_wall_3, dist_wall_4]

        #extract positive distances
        positive_wall_dists = []
        for i in range(4):
            if wall_distances[i] > 0:
                positive_wall_dists.append(wall_distances[i])

        #extract the critical (minium positive) wall distance
        critical_wall_dist = min(positive_wall_dists[0], positive_wall_dists[1])
     
        #return true if the distance sensor value is less than the wall distance
        if self.distanceSensors[0].getValue() < 1000 * critical_wall_dist:
            return True
        else:
            return False

    def block_colour(self):
        pass


    def block_in_distance(self):
        #return true if distance sensor less than 2.2cm
        if self.distanceSensors[0].getValue() < 22:
            return True
        else:
            return False
    
    def get_distance(self):
        #return true if distance sensor value in mm
        return self.distanceSensors[0].getValue()
