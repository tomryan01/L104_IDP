from MyRobot import MyRobot
import math

class Gps(MyRobot):

    def get_magnitude(self, a):
        "return magnitude of 2d vector"
        return (a[0]**2 + a[1]**2)**0.5

    def dot_product(self, a ,b):
        "return dot procduct of two 2d vectors"
        return a[0]*b[0]+a[1]*b[1]

    def cross_product(self, a, b):
        "return cross product of two 2d vectors"
        return a[0]*b[1]-a[1]*b[0]

    def mid_position(self):
        "return xz coordinate of middle of robot"
        mid_position_xyz = self.gpsSensors[1].getValues()
        mid_position_xz = [mid_position_xyz[0], mid_position_xyz[2]]
        return mid_position_xz


    def front_position(self):
        "return xz coordinate of front of robot"
        front_position_xyz = self.gpsSensors[0].getValues()
        front_position_xz = [front_position_xyz[0], front_position_xyz[2]]
        return front_position_xz

    def norm_robot_orientation(self):
        "return xz orientaion of robot as unit vector"
        #get position of front of robot
        front_position_xz = self.front_position()
        #get position of middle of robot
        mid_position_xz = self.mid_position()

        #find orientation of robot in [x,z]
        robot_orientation = []
        for i in range(2):
            robot_orientation.append(front_position_xz[i] - mid_position_xz[i])
        
        #normalise vectors to unit length
        norm_robot_orientation = [robot_orientation[i] / self.get_magnitude(robot_orientation) for i in range(2)]
        return norm_robot_orientation
        

    def distance_from_start(self):
        "return distance from the start, of the front of the robot"
        #get position of front of robot
        front_position_xz = self.front_position()

        #find distance from orgin
        vector_to_origin = []
        for i in range(2):
            vector_to_origin.append(front_position_xz[i] - self.origin[i])
        distance_to_origin = self.get_magnitude(vector_to_origin)

        #return distance from origin
        return distance_to_origin
    

    def mid_distance_from_start(self):
        "return distance from the start, of the middle of the robot"
        #get position of front of robot
        mid_position_xz = self.mid_position()

        #find distance from orgin
        vector_to_origin = []
        for i in range(2):
            vector_to_origin.append(mid_position_xz[i] - self.origin[i])
        distance_to_origin = self.get_magnitude(vector_to_origin)

        #return distance from origin
        return distance_to_origin


    def back_distance_from_start(self):
        "return distance from the start, of the back of the robot"
        #get positions of robot
        front_position_xz = self.front_position()
        mid_position_xz = self.mid_position()

        difference_xz = [front_position_xz[i] - mid_position_xz[i] for i in range(2)]
        back_position_xz = [mid_position_xz[i] - difference_xz[i] for i in range(2)]

        #find distance from orgin
        vector_to_origin = []
        for i in range(2):
            vector_to_origin.append(back_position_xz[i] - self.origin[i])
        distance_to_origin = self.get_magnitude(vector_to_origin)

        #return distance from origin
        return distance_to_origin


    def direction_from_vector(self, v1, v2):
        " get the angle between two vectors "

        #normalise vectors to unit length
        norm_v1 = [v1[i] / self.get_magnitude(v1) for i in range(2)]
        norm_v2 = [v2[i] / self.get_magnitude(v2) for i in range(2)]

        #find their dot product, if positive then robot facing start
        dot_product = self.dot_product(norm_v1, norm_v2)

        #find their cross product (origin x robot), if positive then need to turn right, negative if need to turn left
        cross_product = self.cross_product(norm_v1, norm_v2)

        #find and return desired angle
        if dot_product > 0:
            angle =  math.asin(cross_product)
        else:
            if cross_product > 0:
                angle =  (math.pi - math.asin(cross_product))
            else:
                angle =  (-math.pi - math.asin(cross_product))

        return angle

    def direction_from_start(self):
        """return the direction of the robot facing, relative to start direction
        funtion will return number of radians needed to turn
        negative for angle needed left, positive for angle needed right"""
        
        #get position of middle of robot
        mid_position_xz = self.mid_position()

        #find direction to start from middle of robot in [x,z]
        origin_direction = []
        for i in range(2):
            origin_direction.append(self.origin[i] - mid_position_xz[i])

        #TODO: Have this simply use the direction_from_vector method
        #normalise vectors to unit length
        norm_robot_orientation = self.norm_robot_orientation()
        norm_origin_direction = [origin_direction[i] / self.get_magnitude(origin_direction) for i in range(2)]

        #find their dot product, if positive then robot facing start
        dot_product = self.dot_product(norm_origin_direction, norm_robot_orientation)

        #find their cross product (origin x robot), if positive then need to turn right, negative if need to turn left
        cross_product = self.cross_product(norm_robot_orientation, norm_origin_direction)

        #find and return desired angle
        if dot_product > 0:
            directionFromStart =  math.asin(cross_product)
        else:
            if cross_product > 0:
                directionFromStart =  (math.pi - math.asin(cross_product))
            else:
                directionFromStart =  (-math.pi - math.asin(cross_product))

        return directionFromStart


    def direction_away_from_start(self):
        """return angle robot is facing away from start, 0 is facing away
        positive means turn right to face away and negative means turn left"""
        
        #get direction from start
        directionFromStart = self.direction_from_start()

        if directionFromStart > 0:
            directionAwayStart = -3.1416 + directionFromStart
        else:
            directionAwayStart = 3.1416 + directionFromStart
        
        return directionAwayStart


    def coordinate_in_my_box(self, coordinate):
        "return true if coordinate is in robots start box"
        if coordinate[0] >= 0.8 and coordinate[0] <= 1.2:
            if coordinate[1] >= 0.8 and coordinate[1] <= 1.2:
                return True
            else:
                return False
        else:
            return False
