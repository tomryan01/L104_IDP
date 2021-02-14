from MyRobot import MyRobot
import numpy as np

class Detection(MyRobot):

    def __init__(self):
        #calls constructor for super class
        super().__init__()
        
        #set origin/start point for robot
        self.origin = [1, -1]

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

        #vector lambda_solution = (lambda_robot, lambda_wall)
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
        if self.get_distance() < 1000 * critical_wall_dist and self.get_distance() < 1390:
            #found a block
            if self.looking_in_list(self.blockLocations) == None:
                false_list = []
                for item in self.blockLocations:
                    false_list.append(self.same_block_coordinate(item, self.coordinate_looking_at()))
                if not(True in false_list):
                    coordinate = self.correct_coordinate(self.coordinate_looking_at())
                    self.blockLocations.append([coordinate[0], coordinate[1], 0])
                    self.emit_position(self.blockLocations[-1])
            return True
        else:
            #remove from blockLocations list if block is no longer there
            #print(self.looking_in_list(self.blockLocations))
            #if self.looking_in_list(self.blockLocations) != None:
            #    self.blockLocations.remove(self.blockLocations[self.looking_in_list(self.blockLocations)])
            return False

    def red_colour(self):
        "return the level of red recieved by the colour sensor"
        cameraData = self.colourSensors[0].getImage()
        red = self.colourSensors[0].imageGetRed(cameraData, self.colourSensors[0].getWidth(), 0, 0)
        return red
    
    def blue_colour(self):
        "return the level of blue recieved by the colour sensor"
        cameraData = self.colourSensors[0].getImage()
        blue = self.colourSensors[0].imageGetBlue(cameraData, self.colourSensors[0].getWidth(), 0, 0)
        return blue


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
    

    def looking_at_coordinate(self, coordinate_list_item):
        "return true if what you see on the distance sensor is the block at this coordinate"

        #takes only x and y coordinates
        coordinate = [coordinate_list_item[0],coordinate_list_item[1]]

        #get position of front of robot
        front_position_xz = self.front_position()
        #get position of middle of robot
        mid_position_xz = self.mid_position()

        #find notmal orientation of robot in [x,z]
        norm_robot_orientation = self.norm_robot_orientation()

        #fiend direction of friend
        coordinate_direction = []
        for i in range(2):
            coordinate_direction.append(coordinate[i] - mid_position_xz[i])
        #normalise
        norm_coordinate_direction = [coordinate_direction[i] / self.get_magnitude(coordinate_direction) for i in range(2)]
        
        #find out if they are parallel
        dot_product = self.dot_product(norm_robot_orientation, norm_coordinate_direction)

        #dot product is cos(angle between them) and we can calc the critical angle between them from the extreme case:
        vector_between = [coordinate[i] - front_position_xz[i] for i in range(2)]
        dist_between = self.get_magnitude(vector_between)
        critical_cos_theta = dist_between / (dist_between**2 + 0.05**2)**0.5

        #distance seen by sensor
        distance_seen = self.get_distance() / 1000
        if abs(dot_product) > critical_cos_theta and dot_product > 0:
            if (distance_seen > dist_between + 0.05) or (distance_seen < dist_between - 0.05):
                return False
            else:
                return True
        else:
            return False

    
    def same_block_coordinate(self, coordinate_list_item_1, coordinate_list_item_2):
        "return true if the cordinate_1 passed and corrdinate_2 passed could describe the same block"

        #takes only x and y coordinates
        coordinate_1 = [coordinate_list_item_1[0],coordinate_list_item_1[1]]
        coordinate_2 = [coordinate_list_item_2[0],coordinate_list_item_2[1]]

        x_difference = coordinate_1[0] - coordinate_2[0]
        z_difference = coordinate_1[1] - coordinate_2[1]

        #if differences less than block size then describe same block
        if abs(x_difference) < 0.055 and abs(z_difference) < 0.055:
            return True
        else:
            return False


    def looking_in_list(self, coordinate_list):
        "return true if looking at any coordinate in a list"
        
        #list of true/false
        true_false_list = [self.looking_at_coordinate(self.correct_coordinate(v)) for v in coordinate_list]

        #return none if all are false, but the index if true
        for i in range(len(true_false_list)):
            if true_false_list[i]:
                #print(i)
                return i
        return None
    
    
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

    def correct_coordinate(self, coordinate_item):
        "correct the inital block seen to middle"
        coordinate = [coordinate_item[0], coordinate_item[1]]

        my_position = self.mid_position()
        vector_to_point = [coordinate[i] - my_position[i] for i in range(2)]

        norm_vector = [vector_to_point[i] / self.get_magnitude(vector_to_point) for i in range(2)]

        perp_vector = [ norm_vector[1], -1*norm_vector[0]]

        corrected_coordinate = [coordinate[i] + 0.01 * perp_vector[i] for i in range(2)]

        return [corrected_coordinate[0], corrected_coordinate[1]]