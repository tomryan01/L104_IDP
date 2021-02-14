from MyRobot import MyRobot
import struct

class Communication(MyRobot):
    
    def emit_position(self, data):
        """Emit the positional data, takes argument of data = [x, z, info_bit]
        where info_bit = 0 for unknown block, 1 for red block, 2 for blue block and 3 for robot"""

        message = struct.pack("ddB", data[0], data[1], data[2])
        #emit signal
        self.emitter[0].send(message)
   
        
    def receieve_position(self):
        """Return the position recieved [x,z, info_bit]
        where info_bit = 0 for unknown block, 1 for red block, 2 for blue block and 3 for robot"""

        #ensure there is data in the queue
        queue_length = self.receiver[0].getQueueLength()
        if queue_length != 0:
            #get siganl
            message = self.receiver[0].getData()
            self.receiver[0].nextPacket()
            data = struct.unpack("ddB", message)
            return [data[0], data[1], data[2]]
        
    

    def emit_my_position(self):
        "Emit location of this robot"
        my_position = self.mid_position()
        self.emit_position([my_position[0], my_position[1], 3])


    def update_block_locations(self):
        "Update the list of block locations"
        data = self.receieve_position()
        #If data is null then no new block to update
        if data != None:
            false_list = []
            for item in self.blockLocations:
                false_list.append(self.same_block_coordinate(item, data))
            if not(True in false_list):
                self.blockLocations.append(data)
                
        


    def looking_at_my_friend(self):
        "return true if looking at his friend, consideres angle and distance"
        data = self.receieve_position()
        #if data was null assume not looking at friend
        if data != None:
            friend_position = [data[0], data[1]]
        else:
            return False

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
