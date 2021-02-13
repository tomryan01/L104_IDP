from Detection import Detection
from Drive import Drive
from Gps import Gps
from Grabber import Grabber
from Communication import Communication
import math

class Behaviour(Detection, Drive, Gps, Grabber, Communication):

    def __init__(self):
        #calls constructor for parent class
        super().__init__()

        #use to store distance from start of a found block
        self.blockOriginalDistance = 0

        #use for knowledge of state
        self.state = [0,1]

        #use to count to 4 blocks
        self.blocksDelivered = 0

        #store location of blocks
        self.blockLocations = []

        
    def goToCoordinate(self, coordinate):
        "takes x,y,c coordinate, and drives to that point"
        coordinate = [coordinate[0] - self.mid_position()[0], coordinate[1] - self.mid_position()[1]]
        orientation = self.norm_robot_orientation()
        mag_coordinate = [coordinate[0]/self.get_magnitude(coordinate), coordinate[1]/self.get_magnitude(coordinate)]
        cross_product = self.cross_product(orientation, mag_coordinate)
        if(abs(cross_product) < 0.05):
            if(self.get_magnitude(coordinate) > 0.2):
                self.forwards(5)
        elif(cross_product < 0):
            self.spin(3, -1)
        else:
            self.spin(3, 1)

    def findBlocks(self):
        "main block finding algorithm"
        #print(self.state)

        #must call this every time
        #self.emit_position()
        #TODO: emit position again

        #if not called then reciver queue fills up with old data
        looking_at_friend = self.looking_at_my_friend()

        #initial spin at the base
        if(self.state == [0,1]):
            self.spin(1, 1)
            if(self.block_in_sight() and not(self.coordinate_in_my_box(self.coordinate_looking_at())) and (self.get_distance() < 1390)):
                if(self.looking_in_list(self.blockLocations) == None and not(self.distance_inside_friend_corner())):
                    if(not(looking_at_friend)):
                        self.state[1] += 1
        #initially go forwards towards block
        if(self.state == [0,2]):
            self.forwards(5)
            if(not(self.block_in_sight()) and self.mid_distance_from_start() < 0.2):
                self.state = [0,1]
            if(not(self.block_in_sight()) and self.back_distance_from_start() > 0.5):
                self.state = [2, 2]
            if(self.block_in_colour_sensor_range()):
                if(self.red_colour() > self.blue_colour() + 20):
                    self.state = [1,1]
                else:
                    self.state = [2,1]
                    self.blockOriginalDistance = self.distance_from_start()
            if(looking_at_friend):
                self.state = self.state = [2,2]
        #keep going forwards, because found block is red
        if(self.state == [1,1]):
            self.forwards(5)
            if(not(self.block_in_sight())):
                self.state = [1,10]
            if(self.block_in_distance()):
                self.state[1] += 1
        #pick up block
        if(self.state == [1,2]):
            if(self.block_in_distance()):
                self.pick_up()
                if(self.armsPosition == 1):
                    self.state[1] += 1
                    self.blockOriginalDistance = self.distance_from_start()
            elif(self.get_distance() < 50):
                self.state = [1,1]
        #reverse 10% of the distance travelled
        if(self.state == [1,3]):
            self.backwards(5)
            if(self.distance_from_start() < 0.9*self.blockOriginalDistance):
                if(self.get_distance() < 40):
                    self.state[1] += 1
                else:
                    self.state = [1,10]
        #spin 180 degrees to face back towards start
        if(self.state == [1,4]):
            if(self.direction_from_start() > 0):
                self.spin(1, 1)
            else:
                self.spin(1, -1)
            if(abs(self.direction_from_start()) < 0.05):
                self.state[1] += 1
        #go forwards towards start
        if(self.state == [1,5]):
            #continous course correction
            if(self.distance_from_start() < 0.1):
                self.state[1] += 1
            elif(abs(self.direction_from_start()) < 0.05):
                self.forwards(5)
            else:
                self.state[1] -= 1
        #put the block down at the start
        if(self.state == [1,6]):
            self.put_down()
            if(self.armsPosition == 0):
                self.state[1] += 1
                self.blocksDelivered += 1
                if self.blocksDelivered >= 4:
                    self.state = [3,1]
        #reverse slightly so as not to spin block around
        if(self.state == [1,7]):
            self.backwards(5)
            if(self.get_distance() > 55):
                self.state = [0,1]
        #if he tried to pick up a block and failed put arms down
        if(self.state == [1,10]):
            self.put_down()
            if(self.armsPosition == 0):
                self.state = [2,2]
        #reverse back after detecting blue block
        if(self.state == [2,1]):
            self.backwards(5)
            if(self.distance_from_start() < 0.9*self.blockOriginalDistance):
                self.state[1] += 1
        #spin so facing (away from) home
        if(self.state == [2,2]):
            if(self.direction_away_from_start() > 0):
                self.spin(1, 1)
            else:
                self.spin(1, -1)
            if(abs(self.direction_away_from_start()) < 0.05):
                self.state[1] += 1
        #reverse back home
        if(self.state == [2,3]):
            #continous course correction
            if(self.mid_distance_from_start() < 0.1):
                self.state[1] += 1
            elif(abs(self.direction_away_from_start()) < 0.05):
                self.backwards(5)
            else:
                self.state[1] -= 1
        #spin until blue block is no longer in sight
        if(self.state == [2,4]):
            self.spin(1, 1)
            if(not(self.block_in_sight())):
                self.state = [0,1]
        #reverse back if collected all blocks
        if(self.state == [3,1]):
            self.backwards(5)
            if(self.distance_from_start() > 0.2):
                self.state[1] += 1
        #spin to face home
        if(self.state == [3,2]):
            if(self.direction_from_start() > 0):
                self.spin(1, 1)
            else:
                self.spin(1, -1)
            if(abs(self.direction_from_start()) < 0.05):
                self.state[1] += 1
        #drive home
        if(self.state == [3,3]):
            if(abs(self.direction_from_start()) < 0.05):
                self.forwards(3)
            else:
                self.state[1] -= 1
            if(self.mid_distance_from_start() < 0.1):
                self.state[1] += 1
        #finished
        if(self.state == [3,4]):
            self.reset()
        

        
            

    
        
        

