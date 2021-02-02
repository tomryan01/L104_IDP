from Detection import Detection
from Drive import Drive
from Gps import Gps
from Grabber import Grabber

class Behaviour(Detection, Drive, Gps, Grabber):

    def __init__(self):
        #calls constructor for parent class
        super().__init__()

        #use to store distance from start of a found block
        self.blockOriginalDistance = 0

    def findBlocks(self):
        #main block finding algorithm
        #If there is no block within range, spin
        if (not(self.block_in_sight()) and self.distance_from_start() < 0.2) or (abs(self.direction_from_start() < 1.57 and self.armsPosition == 0)):
            print("a")
            self.spin(1, 1)
        #When block seen, drive forwards unless block close enough to pick up
        elif not(self.block_in_distance()) and self.armsPosition == 0:
            print("b")
            self.forwards(3)
        #If arms are down, or moving up then pick up the block and record its position
        elif (self.armsPosition == 0 or self.armsPosition == 2 and self.block_colour() > 70):
            print("c")
            self.pick_up()
            self.blockOriginalDistance = self.distance_from_start()
        #When block picked up, reverse until travelled 30% of the way
        elif not(self.distance_from_start() < 0.7*self.blockOriginalDistance):
            print("d")
            self.backwards(3)
        #When past 30% of way, check to see if robot still has block
        elif self.block_colour() < 70:
            # this will drop the arms, but not put anything down as it doesn't have the block
            # it will then re-initiate the pickup sequence
            self.put_down()
        #When past 30% of the way, ensure robot is facing origin
        elif not(abs(self.direction_from_start()) < 0.1) and not(self.armsPosition == 0) :
            print("e")
            #spin the correct direction, positive is turn right
            #important for later corrections
            if self.direction_from_start() > 0:
                self.spin(1, 1)
            else:
                self.spin(1, -1)
        #Keep going forward until it returns to start
        elif self.distance_from_start() > 0.05:
            print("f")
            self.forwards(3)
        #When at the start put block down until arms in down position
        elif self.armsPosition == 1 or self.armsPosition == 3:
            print("g")
            self.put_down()
        
        

