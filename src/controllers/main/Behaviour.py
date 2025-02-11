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

        #the current block to look for
        self.blockToFind = 0

        #counter
        self.count = 0

        #position of other robot
        self.friend_location = None

        #*static* variables for checkForBlock functionn
        self.firstDirection = None
        self.spinDirection = 1

        self.friendStuck = False
        self.phase2 = False

    def goToCoordinate(self, coordinate, check_collision):
        "takes x,y,c coordinate, and drives to that point"
        coordinate = [coordinate[0] - self.mid_position()[0], coordinate[1] - self.mid_position()[1]]
        orientation = self.norm_robot_orientation()
        mag_coordinate = [coordinate[0]/self.get_magnitude(coordinate), coordinate[1]/self.get_magnitude(coordinate)]
        cross_product = self.cross_product(orientation, mag_coordinate)
        dot_product = self.dot_product(orientation, mag_coordinate)
        if(self.get_magnitude(coordinate) > 0.3):
            if(abs(cross_product) < 0.02 and dot_product > 0):
                if(not self.will_collide_with_friend(self.friend_location)):
                    if(not self.checkCollision(coordinate) or not check_collision):
                        self.forwards(5)
                    else:
                        #print("Red robot cannot collect block or it would collide")
                        return "Collision"
                else:
                    return "Robot"
            elif(cross_product < 0):
                if(abs(cross_product) < 0.05):
                    self.spin(1, -1)
                else:
                    self.spin(3, -1)
            else:
                if(abs(cross_product) < 0.05):
                    self.spin(1, 1)
                else:
                    self.spin(3, 1)
        else:
            return "Done"

    def checkCollision(self, coordinate):
        "Takes x,y coordinate: Checks to see if a coordinate to be travelled to will result in the robot colliding with a different block"
        mag_coord = self.get_magnitude(coordinate)
        norm_coord = [coordinate[0]/mag_coord, coordinate[1]/mag_coord]
        for b in self.blockLocations:
            b = [b[0] - self.mid_position()[0], b[1] - self.mid_position()[1]]
            if(b != coordinate):
                mag_b = self.get_magnitude(b)
                if(mag_b < mag_coord):
                    norm_b = [b[0]/mag_b, b[1]/mag_b]
                    angle = math.acos(self.dot_product(norm_coord, norm_b))
                    d = mag_b * math.sin(angle)
                    if d < self.ROBOT_WIDTH / 2000:
                        #they will collide
                        return True
        #they won't collide
        return False

    def spinLeftRight(self, max_angle):
        "spins left and right defined by a maximum angle, initial direction is -1 for left and +1 for right"
        #initial setup of 'static' variable
        if(self.firstDirection == None):
            self.firstDirection = self.norm_robot_orientation()
        #spin in given direction
        self.spin(1, self.spinDirection)
        #change direction if reached 45 degrees
        angle = self.direction_from_vector(self.firstDirection, self.norm_robot_orientation())
        if(angle > max_angle):
            self.spinDirection = -1
        elif(angle < -max_angle):
            self.spinDirection = 1
        return angle

    def checkForBlock(self):
        "checks for a lost block"
        #block has been found
        """
        Ideally check here to see if the found block is actually the
        same block that was fumbled, but for small fumbles this works
        okay - but worth thinking about if problem cases come up
        """
        if(self.block_in_sight(self.friend_location)):
            #make the found block the next block to collect
            self.blockToFind = len(self.blockLocations) -1
            return "Found"
        angle = self.spinLeftRight(0.785)
        if(abs(angle) < 0.05):
            if(self.spinDirection == 1 and angle < 0):
                return "Done"

    def setBlockToFind(self):
        "Sets blockToFind to the nearest block"
        magnitudes = []
        for b in self.blockLocations:
            if(b[2] == 0 or b[2] == 1):
                b = [b[0] - self.mid_position()[0], b[1] - self.mid_position()[1]]
                magnitudes.append(self.get_magnitude(b))
            else:
                magnitudes.append(float('inf'))
        #if list is empty
        if len(magnitudes) > 0:
            return magnitudes.index(min(magnitudes))
        else:
            return 0

    def findBlocks2(self):
        "Block finding algorithm"
        #must call this every time
        self.emit_my_position()
        self.friend_location = self.friend_position()
        self.update_block_locations()

        #initial spin to get block positions
        if self.state == [0,1]:
            if(abs(self.direction_from_start()) < 0.05):
                #after one initial spin begin to look for blocks
                self.state[1] += 1
                print("\033[0;31;40mThey know the location of", len(self.blockLocations), "Blocks")
                self.blockToFind = self.setBlockToFind()
            else:
                #on initial spin look for block positions but don't collect them
                self.spin(1,self.spinDirection)
                self.block_in_sight(self.friend_location)
        #go to block
        if self.state == [0,2]:
            try:
                #go to the current coordinate
                if(self.blockLocations[self.blockToFind][2] == 0 or self.blockLocations[self.blockToFind][2] == 1):
                    result = self.goToCoordinate(self.blockLocations[self.blockToFind], True)
                else:
                    result = "Blue"
            except IndexError:
                #there is an exception if blockToFind exceeds the length of blockLocations
                #print("Red has found no blocks!")
                self.blockToFind = 0
                """
                IMPORTANT TODO: This ommits cases where some blocks were unable to be collected due
                to them being obstructed
                """
                if(len(self.blockLocations) == 0):
                    self.state = [5,1]
            else:
                if(result == "Done"):
                    #once reached block move on
                    self.state[1] += 1
                elif(result == "Robot"):
                    self.state = [3,1]
                elif(result == "Collision"):
                    #the robot should not collect the block
                    self.blockToFind += 1
                    #if friend is also stuck, both move to phase 2
                    isStuck = True
                    for b in self.blockLocations:
                        coord = [b[0] - self.mid_position()[0], b[1] - self.mid_position()[1]]
                        if(b[2] == 0 or b[2] == 1) and not self.checkCollision(coord):
                            isStuck = False
                    if(isStuck):
                        self.emit_position([0,0,5])
                        if self.friendStuck:
                            self.state = [5,1]
                            self.friendStuck = False
                elif(result == "Blue"):
                    #the block to find is blue
                    #check to see if all remaining blocks are blue
                    allBlue = True
                    for b in self.blockLocations:
                        if(b[2] == 0 or b[1] == 1):
                            allBlue = False
                    if(allBlue):
                        self.state = [5,1]
                    else:
                        self.blockToFind += 1 
        #check block colour
        if self.state == [0,3]:
            if(self.red_colour() > self.blue_colour() + 20):
                #block is red
                self.forwards(2)
                if(self.block_in_distance()):
                    print("\033[0;31;40mRed found a Red Block")
                    self.state[1] += 1
            elif(self.blue_colour() > self.red_colour() + 20):
                #block is blue
                print("\033[0;31;40mRed found a Blue Block")
                self.state = [2,1]
                #update colour of found block
                self.blockLocations[self.blockToFind][2] = 2
                self.blockOriginalDistance = self.distance_from_start()
            else:
                #block has gone
                emit = [self.blockLocations[self.blockToFind][0], self.blockLocations[self.blockToFind][1], 4]
                self.emit_position(emit)
                self.blockLocations.remove(self.blockLocations[self.blockToFind])
                self.blockToFind -= 1
                #go back to start, block wasn't found
                if(self.phase2 == False):
                    self.state = [0,10]
                else:
                    self.state = [0,9]
        #pick up red block
        if self.state == [0,4]:
            self.pick_up()
            #if arms are up i.e block is picked up
            if(self.armsPosition == 1):
                #remove found block from blockLocations
                emit = [self.blockLocations[self.blockToFind][0], self.blockLocations[self.blockToFind][1], 4]
                self.emit_position(emit)
                self.blockLocations.remove(self.blockLocations[self.blockToFind])
                #remove one from blockToFind since the list is shorter
                self.blockToFind -= 1
                self.state[1] += 1
                self.blockOriginalDistance = self.distance_from_start()
        #reverse some distance
        if self.state == [0,5]:
            self.backwards(5)
            if(self.count > 10):
                self.count = 0
                if(self.get_distance() < 40):
                    if(self.phase2 == False):
                        self.state = [0,10]
                    else:
                        self.state = [0,9]
                else:
                    self.state = [1,1]
            else:
                self.count += 1
        #go back to start
        if self.state == [0,6]:
            result = self.goToCoordinate([1.13, 1.13], False)
            if(result == "Done"):
                self.state[1] += 1
            elif(result == "Robot"):
                self.state = [4,1]
            elif(result == "Collision"):
                pass
        #put down block
        if self.state == [0,7]:
            self.put_down()
            if(self.armsPosition == 0):
                print("\033[0;31;40mRed delivered a Block")
                #iterate to next block to find
                self.state[1] += 1
        #reverse a little bit so don't hit block on spin
        if self.state == [0,8]:
            self.backwards(5)
            if(self.get_distance() > 55):
                #update self.blockToFind
                self.blockToFind = self.setBlockToFind()
                #go back to find block state
                self.state = [0,2]
        #in phase 2
        if self.state == [0,9]:
            result = self.goToCoordinate([0, 0.56], False)
            if(result == "Done"):
                self.state = [0,6]
            elif(result == "Robot"):
                self.state = [4,1]
        #little add for going home
        if self.state == [0,10]:
            result = self.goToCoordinate([0.8, 0.8], False)
            if(result == "Done"):
                self.state = [0,6]
            elif(result == "Robot"):
                self.state = [4,1]
        #robot has reversed, but doesn't have block i.e. it fumbled it
        #first, put down grabbers
        if self.state == [1,1]:
            self.put_down()
            if(self.armsPosition == 0):
                self.state[1] += 1
        if self.state == [1,2]:
            if(self.checkForBlock() == "Done"):
                #go back to start, block wasn't found
                if(self.phase2 == False):
                    self.state = [0,10]
                else:
                    self.state = [0,9]
            elif(self.checkForBlock() == "Found"):
                #block was found, go get it
                self.state = [0,2]
        #block is blue
        #first reverse a little
        if self.state == [2,1]:
            self.backwards(5)
            self.count += 1
            if(self.count > 10):
                self.count = 0
                if(self.phase2 == False):
                    self.state[1] += 1
                else:
                    self.state = [2,3]
        #go back to start
        if self.state == [2,2]:
            result = self.goToCoordinate([0.95, 0.95], False)
            if(result == "Done"):
                self.state = [0,2]
        #in phase 2
        if self.state == [2,3]:
            result = self.goToCoordinate([0, 0.56], False)
            if(result == "Done"):
                self.state = [2,2]
        #reverse a little on a potential robot collision (when doesn't have block)
        if self.state == [3,1]:
            self.backwards(1)
            if(self.count > 1):
                self.state = [0,2]
                self.count = 0
            else:
                self.count += 1
        #reverse a little on a potential robot collision (when has block)
        if self.state == [4,1]:
            self.backwards(10)
            if(self.count > 10):
                self.count = 0
                if self.phase2 == False:
                    self.state = [0,10]
                else:
                    self.state = [0,9]
            else:
                self.count += 1
        #initiate phase 2
        #go to start position
        if self.state == [5,1]:
            result = self.goToCoordinate([0, 0.56], False)
            if(result == "Done"):
                self.phase2 = True
                self.emit_position([0,0,6])
                self.state[1] += 1
                self.block_in_sight(self.friend_location)
        #do initial sweep
        if self.state == [5,2]:
            self.spinDirection = -1
            angle = self.spinLeftRight(3.14)
            self.block_in_sight(self.friend_location)
            if(angle < -3):
                self.state[1] += 1
        if self.state == [5,3]:
            self.spinDirection = 1
            angle = self.spinLeftRight(3.14)
            self.block_in_sight(self.friend_location)
            if(angle > 3):
                print("\033[0;31;40mThey know the location of", len(self.blockLocations), "Blocks")
                allBlue = True
                for b in self.blockLocations:
                    if(b[2] == 0 or b[2] == 1):
                        allBlue = False
                if(len(self.blockLocations) == 0) or allBlue: 
                    self.state = [6,1]
                else:
                    self.state = [0,2]
                    self.blockToFind = self.setBlockToFind()
        #go home if no blocks left
        if self.state == [6,1]:
            self.backwards(5)
            if(self.count > 10):
                self.state[1] += 1
                self.count = 0
            else:
                self.count += 1
        if self.state == [6,2]:
            result = self.goToCoordinate([1.13, 1.13], False)
            if(result == "Done"):
                self.state[1] += 1
        if self.state == [6,3]:
            if(len(self.blockLocations)) > 0:
                self.state = [0,2]
        if self.robot.getTime() > 280:
            result = self.goToCoordinate([1.13, 1.13], False)
            if(result == "Done"):
                self.state[1] = [7,1]
        if self.state == [7,1]:
            self.reset()
