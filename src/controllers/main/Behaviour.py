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
        if not(self.block_in_sight()) or (abs(self.direction_from_start()) < 1 and self.armsPosition == 0):
            #print("a")
            if self.get_distance() < 55:
                self.backwards(5)
            else:
                self.spin(1, 1)
        #When block seen, drive forwards unless block close enough to pick up
        elif not(self.block_in_distance()):
            #print("b")
            self.forwards(5)
        #If arms are down, or moving up then pick up the block and record its position
        elif self.armsPosition == 0 or self.armsPosition == 2:
            #print("c")
            self.pick_up()
            self.blockOriginalDistance = self.distance_from_start()
        #When block picked up, reverse until travelled 25% of the way
        elif not(self.distance_from_start() < 0.75*self.blockOriginalDistance):
            #print("d")
            self.backwards(5)
        #When past 30% of the way, ensure robot is facing origin
        elif not(abs(self.direction_from_start()) < 0.08):
            #print("e")
            #spin the correct direction, positive is turn right
            #important for later corrections
            if self.direction_from_start() > 0:
                self.spin(1, 1)
            else:
                self.spin(1, -1)
        #Keep going forward until it returns to start
        elif self.distance_from_start() > 0.01:
            #print("f")
            self.forwards(5)
        #When at the start put block down until arms in down position
        elif self.armsPosition == 1 or self.armsPosition == 3:
            #print("g")
            self.put_down()

        
        

