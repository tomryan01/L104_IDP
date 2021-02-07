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
        self.gpsSensors = []
        self.emitter = []
        self.receiver = []

        #constants for how many of each device
        #TO CHANGE THE NUMBER OF DEVICES THESE ARE THE ONLY NUMBER THAT SHOULD BE CHANGED WITHIN THIS CLASS
        self.numWheels = 2
        self.numArms = 2
        self.numDistanceSensors = 1
        self.numColourSensors = 1
        self.numGps = 2
        self.numEmitters = 1
        self.numReceivers = 1
        self.robotWidth = 100 #mm

    #The following methods are used to set up the devices for a robot
    #Each method begins by asserting the length of names in the list
    #Is the number of sensors specified
    def setWheels(self, wheelNames):
        assert len(wheelNames) == self.numWheels
        for i in range(self.numWheels):
            self.wheels.append(self.robot.getDevice(wheelNames[i]))
            
            #wheel positions will be defined by velocity
            self.wheels[i].setPosition(float('inf'))

    def setArms(self, armNames):
        assert len(armNames) == self.numArms
        for i in range(self.numArms):
            self.arms.append(self.robot.getDevice(armNames[i]))

    def setDistanceSensors(self, distanceSensors, timeStep):
        assert len(distanceSensors) == self.numDistanceSensors
        for i in range(self.numDistanceSensors):
            self.distanceSensors.append(self.robot.getDevice(distanceSensors[i]))
            self.distanceSensors[i].enable(timeStep)

    def setColourSensors(self, colourSensors, timeStep):
        assert len(colourSensors) == self.numColourSensors
        for i in range(self.numColourSensors):
            self.colourSensors.append(self.robot.getDevice(colourSensors[i]))
            self.colourSensors[i].enable(timeStep)

    def setGps(self, gpsSensors, timeStep):
        assert len(gpsSensors) == self.numGps
        for i in range(self.numGps):
            self.gpsSensors.append(self.robot.getDevice(gpsSensors[i]))
            self.gpsSensors[i].enable(timeStep)

    def setEmitter(self, emitterName):
        assert len(emitterName) == self.numEmitters
        for i in range(self.numEmitters):
            self.emitter.append(self.robot.getDevice(emitterName[i]))
    
    def setReceiver(self, receiverName, samplingPeriod):
        assert len(receiverName) == self.numReceivers
        for i in range(self.numReceivers):
            self.receiver.append(self.robot.getDevice(receiverName[i]))
            self.receiver[i].enable(samplingPeriod)

    def reset(self):
        self.wheels[0].setVelocity(0)
        self.wheels[1].setVelocity(0)
        self.arms[0].setVelocity(0)
        self.arms[1].setVelocity(0)