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

        self.state = [0,1]

    def findBlocks(self):
        #main block finding algorithm
        #initial spin at the base
        if(self.state == [0,1]):
            self.spin(1, 1)
            if(self.block_in_sight()):
                self.state[1] += 1
        #initially go forwards towards block
        elif(self.state == [0,2]):
            self.forwards(5)
            if(self.block_in_colour_sensor_range()):
                if(self.block_colour() > 70):
                    self.state = [1,1]
                else:
                    self.state = [2,1]
        #keep going forwards, because found block is red
        elif(self.state == [1,1]):
            self.forwards(5)
            if(self.block_in_distance()):
                self.state[1] += 1
        #pick up block
        elif(self.state == [1,2]):
            self.pick_up()
            self.blockOriginalDistance = self.distance_from_start()
            if(self.armsPosition == 1):
                self.state[1] += 1
        #reverse 10% of the distance travelled
        elif(self.state == [1,3]):
            self.backwards(5)
            if(self.distance_from_start() < 0.9*self.blockOriginalDistance):
                if(self.block_colour() > 70):
                    self.state[1] += 1
                else:
                    self.state[1] = 1
        #spin 180 degrees to face back towards start
        elif(self.state == [1,4]):
            if self.direction_from_start() > 0:
                self.spin(1, 1)
            else:
                self.spin(1, -1)
            if(abs(self.direction_from_start()) < 0.1):
                self.state[1] += 1
        #go forwards towards start
        elif(self.state == [1,5]):
            self.forwards(5)
            if(self.distance_from_start() < 0.05):
                self.state[1] += 1
        #put the block down at the start
        elif(self.state == [1,6]):
            self.put_down()
            if(self.armsPosition == 0):
                self.state[1] += 1
        #reverse slightly so as not to spin block around
        elif(self.state == [1,7]):
            self.backwards(5)
            if(self.distance_from_start() > 0.1):
                self.state[1] += 1
        #spin until facing out of start position
        elif(self.state == [1,8]):
            self.spin(1, 1)
            if(abs(self.direction_from_start() > 1.57)):
                self.state = [0,1]
        #reverse after detecting blue block
        elif(self.state == [2,1]):
            self.backwards(5)
            if(self.distance_from_start() < 0.1):
                self.state[1] += 1
        #spin until blue block is no longer in sight
        elif(self.state == [2,2]):
            self.spin(1, 1)
            if(not(self.block_in_sight())):
                self.state = [0,1]
            

    
        
        

