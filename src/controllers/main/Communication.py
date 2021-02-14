from MyRobot import MyRobot
import struct

class Communication(MyRobot):
    
    def emit_position(self, data):
        """Emit the positional data, takes argument of data = [x, z, info_bit]
        where info_bit = 0 for unknown block, 1 for red block, 2 for blue block, 3 for robot
        4 for delete block, 5 for stuck on collision"""

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
            #If the message is about removing item
            if data[2] == 4:
                for b in self.blockLocations:
                    same_block = self.same_block_coordinate(b, data)
                    if same_block:
                        #remove one from blockToFind since the list is shorter
                        if self.blockToFind >= self.blockLocations.index(b):
                            self.blockToFind -= 1
                        self.blockLocations.remove(b)
                        return "Deleted"
            #if robot is stuck
            if data[2] == 5:
                self.friendStuck = True
            #if message is about adding item
            else:
                self.friendStuck = False
                false_list = []
                for item in self.blockLocations:
                    false_list.append(self.same_block_coordinate(item, data))
                if not(True in false_list):
                    self.blockLocations.append(data)
                

    def friend_position(self):
        "return friend position [x,z]"
        data = self.receieve_position()
        #if data was null assume not looking at friend
        if data != None:
            return [data[0], data[1]]
        else:
            #first clock cycle
            return [0.95, -0.95]


    def will_collide_with_friend(self, friend_position):
        "return none if not looking at friend and the distance away if they are"
        front_position = self.front_position()
        my_orientation = self.norm_robot_orientation()
        perp_orientation = [my_orientation[1], -1*my_orientation[0]]
        #dist is distance from my robot to intersection, width is distance along line of intersection
        dist, width = self.find_wall_distance(front_position, my_orientation, friend_position, perp_orientation)
        #if width is less than half width of robot then looking at him
        if abs(width) < self.ROBOT_WIDTH/1000:
            if abs(dist - self.ROBOT_WIDTH/2000) < 0.2:
                return True
            else: 
                return False
        else:
            return False
