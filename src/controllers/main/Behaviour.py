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

        #used to store distance from start of a found block
        self.blockOriginalDistance = 0

        #used for knowledge of state
        self.state = [0,1]

        #used to store location of blocks found
        self.blockLocations = []

        #inicates the current block to look for
        self.blockToFind = 0

        #counter used for reversing, to do an activity for certain number of clock cycles
        self.count = 0

        #store position of other robot
        self.friend_location = None

        #*static* variables for checkForBlock functionn
        self.firstDirection = None
        #spin direction different for each robot, used when correcting coordinate
        self.spinDirection = 1

        #indication of if friend robot is 'stuck' at this clock cycle
        self.friendStuck = False

        #used to know if in phase 1 or 2, decide if go to centre
        self.phase2 = False

    def goToCoordinate(self, coordinate, check_collision):
        """takes [x,y,c] blockLocations list coordinate, and drives to that point
        returns 'collision' if will collide with block, returns 'Robot' if it will collide with friend
        in either of these cases it doesnt move (spin or go forward)
        returns 'done' once arrived and wheels to spin or drive as appropriate"""
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
        """Takes [x,y] coordinate: Checks to see if a coordinate to be travelled to will result in the robot colliding with a different block
        Returns true if it will collide with a block and false if not"""
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
        """spins left and right defined by a maximum angle, initial direction is -1 for left and +1 for right
        returns the angle turned"""
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
        """checks for a 'lost' block
        returns 'Found' if the block is located and 'done' when completed its spin"""
        #block has been found
        """Ideally check here to see if the found block is actually the
        same block that was fumbled, but for small fumbles this works okay"""
        if(self.block_in_sight(self.friend_location)):
            #make the found block the next block to collect
            self.blockToFind = len(self.blockLocations) -1
            return "Found"
        angle = self.spinLeftRight(0.785)
        if(abs(angle) < 0.05):
            if(self.spinDirection == 1 and angle < 0):
                return "Done"

    def setBlockToFind(self):
        """finds the nearest block in blockLocations, only considering red or unknown blocks
        returns the distance away if there are red or uknown blocks and 0 if there are no blocks/all are blue"""
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
        "Main block finding algorithm"
        #must call these every time to find friend location and update blockLocations list
        self.emit_my_position()
        self.friend_location = self.friend_position()
        self.update_block_locations()

        #[0,1] initial spin at home to get block positions
        if self.state == [0,1]:
            if(abs(self.direction_from_start()) < 0.05):
                #after one initial spin begin to look for blocks
                self.state[1] += 1
                print("They know the location of", len(self.blockLocations), "Blocks")
                self.blockToFind = self.setBlockToFind()
            else:
                #on initial spin look for block positions but don't collect them
                self.spin(1,self.spinDirection)
                self.block_in_sight(self.friend_location)
        #[0,2] go to block, looking for collisions
        if self.state == [0,2]:
            try:
                #go to the current coordinate
                if(self.blockLocations[self.blockToFind][2] == 0 or self.blockLocations[self.blockToFind][2] == 1):
                    result = self.goToCoordinate(self.blockLocations[self.blockToFind], True)
                else:
                    result = "Blue"
            except IndexError:
                #there is an exception if blockToFind exceeds the length of blockLocations
                self.blockToFind = 0
                """
                This ommits cases where some blocks were unable to be collected due
                to them being obstructed but this is unlikely to be a problem
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
        #[0,3] check block colour when in range
        if self.state == [0,3]:
            if(self.red_colour() > self.blue_colour() + 20):
                #block is red
                self.forwards(2)
                if(self.block_in_distance()):
                    print("Red found a Red Block")
                    self.state[1] += 1
            elif(self.blue_colour() > self.red_colour() + 20):
                #block is blue
                print("Red found a Blue Block")
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
                #go back to start, block wasn't found, via different coordinates for different phases
                if(self.phase2 == False):
                    self.state = [0,10]
                else:
                    self.state = [0,9]
        #[0,4] pick up red block
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
        #[0,5] reverse a small distance so not to collide as turn
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
        #[0,6] go back to start - called after states [0,9] or [0,10], having gone via different coordinates for different phases
        if self.state == [0,6]:
            result = self.goToCoordinate([1.13, 1.13], False)
            if(result == "Done"):
                self.state[1] += 1
            elif(result == "Robot"):
                self.state = [4,1]
            elif(result == "Collision"):
                pass
        #[0,7] put block down
        if self.state == [0,7]:
            self.put_down()
            if(self.armsPosition == 0):
                print("Red delivered a Block")
                #iterate to next block to find
                self.state[1] += 1
        #[0,8] reverse a little bit so don't hit block on spin
        if self.state == [0,8]:
            self.backwards(5)
            if(self.get_distance() > 55):
                #update self.blockToFind
                self.blockToFind = self.setBlockToFind()
                #go back to find block state
                self.state = [0,2]
        #[0,9] in phase 2 go via centre before going home in state [0,6]
        if self.state == [0,9]:
            result = self.goToCoordinate([-0.1, 0.46], False)
            if(result == "Done"):
                self.state = [0,6]
            elif(result == "Robot"):
                self.state = [4,1]
        #[0,10] in phase 1 go via edge of box before going home in state [0,6]
        if self.state == [0,10]:
            result = self.goToCoordinate([0.8, 0.8], False)
            if(result == "Done"):
                self.state = [0,6]
            elif(result == "Robot"):
                self.state = [4,1]
        #[1,1] robot has reversed, but doesn't have block i.e. it fumbled it
        if self.state == [1,1]:
            #first, put down grabbers
            self.put_down()
            if(self.armsPosition == 0):
                self.state[1] += 1
        #[1,2] robot tries to find fumbled block
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
        #[2,1] if block is blue, first reverse
        if self.state == [2,1]:
            self.backwards(5)
            self.count += 1
            if(self.count > 10):
                self.count = 0
                if(self.phase2 == False):
                    self.state[1] += 1
                else:
                    self.state = [2,3]
        #[2,2] go back to start, in phase 1
        if self.state == [2,2]:
            result = self.goToCoordinate([0.95, 0.95], False)
            if(result == "Done"):
                self.state = [0,2]
        #[2,3] go back to start, in phase 2
        if self.state == [2,3]:
            result = self.goToCoordinate([-0.1, 0.46], False)
            if(result == "Done"):
                self.state = [2,2]
        #[3,1] reverse a little on a potential robot collision (when doesn't have block)
        if self.state == [3,1]:
            self.backwards(1)
            if(self.count > 1):
                self.state = [0,2]
                self.count = 0
            else:
                self.count += 1
        #[4,1] reverse a little on a potential robot collision (when has block)
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
        #[5,1] start of phase 2, go to center
        if self.state == [5,1]:
            result = self.goToCoordinate([-0.1, 0.46], False)
            if(result == "Done"):
                self.phase2 = True
                self.emit_position([0,0,6])
                self.state[1] += 1
                self.block_in_sight(self.friend_location)
        #[5,2] do initial sweep
        if self.state == [5,2]:
            self.spinDirection = -1
            angle = self.spinLeftRight(3.14)
            self.block_in_sight(self.friend_location)
            if(angle < -3):
                self.state[1] += 1
        #[5,3] continue initial sweep and decide what to do next
        if self.state == [5,3]:
            self.spinDirection = 1
            angle = self.spinLeftRight(3.14)
            self.block_in_sight(self.friend_location)
            if(angle > 3):
                print("They know the location of", len(self.blockLocations), "Blocks")
                allBlue = True
                for b in self.blockLocations:
                    if(b[2] == 0 or b[2] == 1):
                        allBlue = False
                if(len(self.blockLocations) == 0) or allBlue: 
                    self.state = [6,1]
                else:
                    self.state = [0,2]
                    self.blockToFind = self.setBlockToFind()
        #[6,1] go home if no blocks left, first reverse
        if self.state == [6,1]:
            self.backwards(5)
            if(self.count > 10):
                self.state[1] += 1
                self.count = 0
            else:
                self.count += 1
        #[6,2] go home
        if self.state == [6,2]:
            result = self.goToCoordinate([1.13, 1.13], False)
            if(result == "Done"):
                self.state[1] += 1
        #[6,3] if other block found by friend then go look at it in state [0,2]
        if self.state == [6,3]:
            if(len(self.blockLocations)) > 0:
                self.state = [0,2]
        #Check if out of time, if so go home
        if self.robot.getTime() > 280:
            result = self.goToCoordinate([1.13, 1.13], False)
            if(result == "Done"):
                self.state[1] = [7,1]
        #[7,1] Rest peacefully
        if self.state == [7,1]:
            self.reset()
