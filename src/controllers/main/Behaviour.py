from Detection import Detection
from Drive import Drive
from Gps import Gps
from Grabber import Grabber

class Behaviour(Detection, Drive, Gps, Grabber):

    def findBlocks(self):
        #main block finding algorithm
        #If there is no block within range, spin
        if not(self.block_in_sight_2()):
            self.spin(1, 1)
        #When block seen, drive forwards unless block close enough to pick up
        elif not(self.block_in_distance()):
            self.forwards(3)
        #If arms are down, or moving up then pick up the block and record its position
        elif self.armsPosition == 0 or self.armsPosition == 2:
            self.pick_up()
            self.blockOriginalDistance = self.distance_from_start()
        #When block picked up, reverse until travelled 30% of the way
        elif not(self.distance_from_start() < 0.7*self.blockOriginalDistance):
            self.backwards(3)
        #When past 30% of the way, ensure robot is facing origin
        elif not(abs(self.direction_from_start()) < 0.1):
            #spin the correct direction, positive is turn right
            #important for later corrections
            if self.direction_from_start() > 0:
                self.spin(1, 1)
            else:
                self.spin(1, -1)
        #Keep going forward until it returns to start
        elif self.distance_from_start() > 0.01:
            self.forwards(3)
        #When at the start put block down until arms in down position
        elif self.armsPosition == 1 or self.armsPosition == 3:
            self.put_down()
        
        

