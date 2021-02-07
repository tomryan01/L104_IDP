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

        #distance of a detected block
        self.blockFoundDistance = 0

        #orientation of block that has just been avoided
        self.directionBlockJustAvoided = 0

        #used to store direction facing when first finding a block
        self.blockFoundOrientation = 0

        #used to temporarily store additionally found blocks
        self.tempFoundBlocks = []

        #use for knowledge of state
        self.state = [0,1]

        #use to count to 4 blocks
        self.blocksDelivered = 0

        #store location of blue blocks
        self.blueLocations = []

    def checkCollision(self, values, blockDistance):
        #values should be a list of lists of length 2 in the form [distance, orientation]
        print("Values:",len(values))
        for v in values:
            if(v[0] * math.sin(v[1]) < self.robotWidth/2):
                if(v[0] < blockDistance):
                    print(v[0], blockDistance)
                    return True
        return False

    def findRelativeDirection(self, dir1, dir2):
        #returns acute angle between two directions
        if(abs(dir1) < 1.571 and abs(dir2) < 1.571):
            #print("a")
            result = abs(dir1-dir2)
        elif((dir1 < 0 and dir2 > 0)):
            #print("b")
            result = abs((3.14+dir1) + (3.14-dir2))
        elif((dir1 > 0 and dir2 < 0)):
            #print("c")
            result = abs((3.14-dir1) + (3.14+dir2))
        else:
            #print("d")
            result = abs(dir1-dir2)
        #print("%a : %f : %s\n" % (dir1, dir2, result))
        #print(result)
        return result

    def findBlocks(self):
        "main block finding algorithm"
        #print(self.state)

        #must call this every time
        self.emit_position()
        #if not called then reciver queue fills up with old data
        looking_at_friend = self.looking_at_my_friend()

        #initial spin at the base
        #print(self.state[1])
        if(self.state == [0,1]):
            self.spin(1, 1)
            if(self.block_in_sight() and not(self.coordinate_in_my_box(self.coordinate_looking_at()))):
                if(not(self.looking_in_list(self.blueLocations)) and not(self.distance_inside_friend_corner())):
                    if(not(looking_at_friend)):
                        #doesn't go for same block again
                        if(abs(self.directionBlockJustAvoided - self.direction_from_start()) > 0.2):
                            self.blockFoundOrientation = self.direction_from_start()
                            self.blockFoundDistance = self.get_distance()
                            self.directionBlockJustAvoided = 0
                            self.state[1] += 1
                        #print("block found!")
        #keep spinning to check robot won't crash into another block on the way
        if(self.state == [0,2]):
            self.spin(1,1)
            relative_orientation = self.findRelativeDirection(self.blockFoundOrientation, self.direction_from_start())
            #print(relative_orientation)
            #spins 45 degrees
            if(relative_orientation > 0.79):
                self.state[1] += 1
            elif(self.block_in_sight()):
                can_append = False
                #ensure no block is counted twice
                if(abs(self.get_distance() - self.blockFoundDistance) > 50):
                    can_append = True
                    for v in self.tempFoundBlocks:
                        if(abs(self.get_distance() - v[0]) < 50):
                            can_append = False
                if(can_append):
                    #add distance, and relative orientation of newly found block
                    self.tempFoundBlocks.append([self.get_distance(), abs(relative_orientation)])
        #spin back to first block found
        if(self.state == [0,3]):
            self.spin(1, -1)
            relative_orientation = self.findRelativeDirection(self.blockFoundOrientation, self.direction_from_start())
            #print(relative_orientation)
            if(relative_orientation < 0.05):
                if(self.checkCollision(self.tempFoundBlocks, self.blockFoundDistance)):
                    self.state[1] = 1
                    self.directionBlockJustAvoided = self.direction_from_start()
                    self.tempFoundBlocks = []
                else:
                    self.state[1] += 1
                    self.tempFoundBlocks = []
        #initially go forwards towards block
        if(self.state == [0,4]):
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
                    self.blueLocations.append(self.coordinate_looking_at())
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
            if(self.mid_distance_from_start() < 0.1):
                self.state[1] += 1
            elif(abs(self.direction_from_start() - 3.14) < 0.05):
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
        

        
            

    
        
        

