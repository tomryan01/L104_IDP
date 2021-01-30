from Detection import Detection
from Drive import Drive
from Gps import Gps
from Grabber import Grabber

class Behaviour(Detection, Drive, Gps, Grabber):

    def findBlocks(self):
        #TODO: implement the pseudocode from the initial report here
        self.forwards(3)
