from MyRobot import MyRobot
import math

class Gps(MyRobot):

    def get_magnitude(self, a):
        #return magnitude of 2d vector
        return (a[0]**2 + a[1]**2)**0.5

    def dot_product(self, a ,b):
        #return dot procduct of two 2d vectors
        return a[0]*b[0]+a[1]*b[1]

    def cross_product(self, a, b):
        #return cross product of two 2d vectors
        return a[0]*b[1]-a[1]*b[0]

    def distance_from_start(self):
        #funtion to return distance from the start, of the front of the robot
        #get position of front of robot
        front_position_xyz = self.gpsSensors[0].getValues()
        front_position_xz = [front_position_xyz[0], front_position_xyz[2]]

        #find distance from orgin
        vector_to_origin = []
        for i in range(2):
            vector_to_origin.append(front_position_xz[i] - self.origin[i])
        distance_to_origin = (vector_to_origin[0]**2 + vector_to_origin[1]**2)**0.5

        #return distance from origin
        return distance_to_origin

    def direction_from_start(self):
        #funtion to return the direction of the robot facing, relative to start direction
        #funtion will return number of radians needed to turn
        #negative for angle needed left, positive for angle needed right
        #get position of front of robot
        front_position_xyz = self.gpsSensors[0].getValues()
        front_position_xz = [front_position_xyz[0], front_position_xyz[2]]
        #get position of middle of robot
        mid_position_xyz = self.gpsSensors[1].getValues()
        mid_position_xz = [mid_position_xyz[0], mid_position_xyz[2]]

        #find orientation of robot in [x,z]
        robot_orientation = []
        for i in range(2):
            robot_orientation.append(front_position_xz[i] - mid_position_xz[i])

        #find direction to start from middle of robot in [x,z]
        origin_direction = []
        for i in range(2):
            origin_direction.append(self.origin[i] - mid_position_xz[i])

        #normalise vectors to unit length
        norm_robot_orientation = [robot_orientation[i] / self.get_magnitude(robot_orientation) for i in range(2)]
        norm_origin_direction = [origin_direction[i] / self.get_magnitude(origin_direction) for i in range(2)]

        #find their dot product, if positive then robot facing start
        dot_product = self.dot_product(norm_origin_direction, norm_robot_orientation)

        #find their cross product (origin x robot), if positive then need to turn right, negative if need to turn left
        cross_product = self.cross_product(norm_robot_orientation, norm_origin_direction)

        #find and return desired angle
        if dot_product > 0:
            return math.asin(cross_product)
        else:
            if cross_product > 0:
                return (math.pi - math.asin(cross_product))
            else:
                return (-math.pi - math.asin(cross_product))
