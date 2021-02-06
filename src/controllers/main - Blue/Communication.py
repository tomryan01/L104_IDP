from MyRobot import MyRobot
import struct

class Communication(MyRobot):
    
    def emit_position(self):
        "Emit the position of the robot to his friend"
        #find robot position
        mid_position_xz = self.mid_position()
        position_x = mid_position_xz[0]
        position_z = mid_position_xz[1]
        
        #encode x position onto +5 and x position onto -5
        coded_x = 5 + position_x
        coded_z = -5 + position_z

        message_x = struct.pack("d", coded_x)
        message_z = struct.pack("d", coded_z)

        #emit signals
        self.emitter[0].send(message_x)
        self.emitter[0].send(message_z)
   
        
    def receieve_position(self):
        "Return the position of the other robot [x,z]"
        #initialise position variables
        position_x = 10
        position_z = 10

        #ensure there is data in the queue
        #extract first message, x position
        queue_length = self.receiver[0].getQueueLength()
        if queue_length != 0:
            #get siganl
            message = self.receiver[0].getData()
            self.receiver[0].nextPacket()
            coded_position = struct.unpack("d", message)
            #decide if data is x or z
            if coded_position[0] > 0:
                position_x = coded_position[0] - 5
            else:
                position_z = coded_position[0] + 5
            
        #extract second message, z position
        queue_length = self.receiver[0].getQueueLength()
        if queue_length != 0:
            #get siganl
            message = self.receiver[0].getData()
            self.receiver[0].nextPacket()
            coded_position = struct.unpack("d", message)
            #decide if data is x or z
            if coded_position[0] > 0:
                position_x = coded_position[0] - 5
            else:
                position_z = coded_position[0] + 5
        
        return [position_x, position_z]


    def looking_at_my_friend(self):
        "return true if looking at his friend, consideres angle and distance"
        friend_position = self.receieve_position()

        #get position of front of robot
        front_position_xz = self.front_position()
        #get position of middle of robot
        mid_position_xz = self.mid_position()

        #find notmal orientation of robot in [x,z]
        norm_robot_orientation = self.norm_robot_orientation()

        #fiend direction of friend
        friend_direction = []
        for i in range(2):
            friend_direction.append(friend_position[i] - mid_position_xz[i])
        #normalise
        norm_friend_direction = [friend_direction[i] / self.get_magnitude(friend_direction) for i in range(2)]
        
        #find out if they are parallel
        dot_product = self.dot_product(norm_robot_orientation, norm_friend_direction)

        #dot product is cos(angle between them) and we can calc the critical angle between them from the extreme case:
        vector_between = [friend_position[i] - front_position_xz[i] for i in range(2)]
        dist_between = self.get_magnitude(vector_between)
        critical_cos_theta = dist_between / (dist_between**2 + 0.2**2)**0.5
        
        #return true if looking at friend
        if abs(dot_product) > critical_cos_theta and dot_product > 0:
            if self.get_distance() / 1000 > dist_between - 0.2:
                return True
            else:
                return False
        else:
            return False
