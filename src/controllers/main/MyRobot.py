#This file contains the MyRobot class which sets out a template for an entire robot

from controller import Robot

class MyRobot:

    def __init__(self):
        #the robot class
        self.robot = Robot()

        #lists of devices
        self.wheels = []
        self.arms = []
        self.distanceSensors = []
        self.colourSensors = []

        #constants for how many of each device
        #TO CHANGE THE NUMBER OF DEVICES THESE ARE THE ONLY NUMBER THAT SHOULD BE CHANGED WITHIN THIS CLASS
        self.numWheels = 2
        self.numArms = 2
        self.numDistanceSensors = 1
        self.numColourSensors = 1

    #The following methods are used to set up the devices for a robot
    #Each method begins by asserting the length of names in the list
    #Is the number of sensors specified
    def setWheels(self, wheelNames):
        assert len(wheelNames) == self.numWheels
        for i in range(2):
            self.wheels.append(self.robot.getDevice(wheelNames[i]))   

    def setArms(self, armNames):
        assert len(armNames) == self.numArms
        for i in range(2):
            self.arms.append(self.robot.getDevice(armNames[i]))

    def setDistanceSensors(self, distanceSensors):
        assert len(distanceSensors) == self.numDistanceSensors
        for i in range(1):
            self.distanceSensors.append(self.robot.getDevice(distanceSensors[i]))

    def setColourSensors(self, colourSensors):
        assert len(colourSensors) == self.numColourSensors
        for i in range(1):
            self.colourSensors.append(self.robot.getDevice(colourSensors[i]))

    def reset(self):
        self.wheels[0].setVelocity(0)
        self.wheels[1].setVelocity(0)
        self.wheels[0].setPosition(0)
        self.wheels[1].setPosition(0)