from MyRobot import MyRobot

class Grabber(MyRobot):

    #def __init__(self):
    #    #use counter for arm pick up
    #    self.armCount = 0
    #    #arm position: 0 if no block, 1 if holding block up, 2 if moving block up, 3 if moving block down
    #    self.armsPosition = 0

    def pick_up(self):
        #set target position as 2 pi radians so block is picked up
        for i in range(self.numArms):
            self.arms[i].setVelocity(2)
            self.arms[i].setPosition(1.57)
        
        #arms being moved up
        self.armsPosition = 2

        #use count to determin if arms in final position and change position to 1, reset count
        self.armCount += 1
        if self.armCount == 20:
            self.armsPosition = 1
            self.armCount = 0

    def put_down(self):
        #set target position as 0 so arms down
        for i in range(self.numArms):
            self.arms[i].setVelocity(2)
            self.arms[i].setPosition(0)
        
        #arms being moved down
        self.armsPosition = 3

        #use count to determin if arms in final position and change position to 0, reset count
        self.armCount += 1
        if self.armCount == 20:
            self.armsPosition = 0
            self.armCount = 0
