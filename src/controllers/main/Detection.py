from MyRobot import MyRobot
import numpy as np

class Detection(MyRobot):

    def __init__(self):
        #calls constructor for super class
        super().__init__()
        
        #set origin/start point for robot
        self.origin = [1, 1]


    def find_wall_distance(self, robot_position, robot_orientation, wall_position, wall_direction):
        """find the intersection of two 2D vector lines
        return the [distance from robot to given wall, distance along wall], positive if forwards and negative if backwards
        robot orientation should be a unit vector"""

        #form matrix of coeficients
        coef_matrix = np.array([[robot_orientation[0], -1*wall_direction[0]],
                                [robot_orientation[1], -1*wall_direction[1]]])
        
        #form vector of position data
        position_vector = np.array([[wall_position[0] - robot_position[0]],
                                    [wall_position[1] - robot_position[1]]])
        
        #find inverse of coeficients matrix
        inverse_coef_matrix = np.linalg.inv(coef_matrix)

        #vector lambda_solutions = (lambda_robot, lambda_wall)
        #lambda_robot is the distance, if orientation was a unit vector
        lambda_solutions = inverse_coef_matrix.dot(position_vector)

        #return distance found
        return [lambda_solutions[0][0], lambda_solutions[1][0]]
        

    def block_in_sight(self):
        "Return true if distance sensor less than wall distance, block is closer than the wall"
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
        front_position_xz = self.front_position()
        mid_position_xz = self.mid_position()

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
        wall_distances = [dist_wall_1[0], dist_wall_2[0], dist_wall_3[0], dist_wall_4[0]]

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
        "return the level of red recieved by the colour sensor"
        cameraData = self.colourSensors[0].getImage()
        red = self.colourSensors[0].imageGetRed(cameraData, self.colourSensors[0].getWidth(), 0, 0)
        return red


    def block_in_distance(self):
        "return true if distance sensor less than 2.2cm"
        if self.distanceSensors[0].getValue() < 22:
            return True
        else:
            return False

    
    def block_in_colour_sensor_range(self):
        "return true if distance sensor less than 10cm"
        if self.distanceSensors[0].getValue() < 100:
            return True
        else:
            return False


    def get_distance(self):
        "return true if distance sensor value in mm"
        return self.distanceSensors[0].getValue()


    def distance_inside_friend_corner(self):
        "return True if distance measured is inside the friend robot region"

        #Define the 2 boundaries
        wall_1_position = [0.8, -0.8]
        wall_1_direction = [1, 0]
        wall_2_position = [0.8, -0.8]
        wall_2_direction = [0, -1]

        #Find position of the robot, will use front position for finding distance
        front_position_xz = self.front_position()

        #Find normalised orientation of the robot
        norm_robot_orientation = self.norm_robot_orientation()

        #find distance to each wall and distance along each wall [distance to, distance along]
        dist_wall_1 = self.find_wall_distance(front_position_xz, norm_robot_orientation, wall_1_position, wall_1_direction)
        dist_wall_2 = self.find_wall_distance(front_position_xz, norm_robot_orientation, wall_2_position, wall_2_direction)

        #store distances in an array
        wall_distances = [dist_wall_1, dist_wall_2]

        #extract distance to wall if looking at the correct part of wall
        true_wall_dist = []
        for i in range(2):
            if wall_distances[i][1] >= 0 and wall_distances[i][1] <= 0.4:
                if wall_distances[i][0] > 0:
                    true_wall_dist.append(wall_distances[i][0])

        #return false if nothing added to true walls
        if len(true_wall_dist) == 0:
            return False
        
        #if 2 items in walls then looking through area and compare to max
        if len(true_wall_dist) == 2:
            max_wall_dist = max(true_wall_dist)
            min_wall_dist = min(true_wall_dist)
            #if between then looking through
            if self.distanceSensors[0].getValue() > 1000 * min_wall_dist and self.distanceSensors[0].getValue() < 1000 * max_wall_dist:
                return True
            else:
                return False

        #extract minimum wall distance
        critical_wall_dist = min(true_wall_dist)
        
        #return true if the distance sensor value is looking into the arena
        if self.distanceSensors[0].getValue() > 1000 * critical_wall_dist:
            return True
        else:
            return False


    def coordinate_looking_at(self):
        "return the coordinate of the thing in the vision sensor"

        #Find position of the robot, will use front position for finding distance
        front_position_xz = self.front_position()

        #Find orientation of the robot
        norm_robot_orientation = self.norm_robot_orientation()

        #get distance seen by sensor
        distance_seen = self.distanceSensors[0].getValue() / 1000

        coordinate_seen = []
        for i in range(2):
            coordinate_seen.append(distance_seen * norm_robot_orientation[i] + front_position_xz[i])
        
        return coordinate_seen


