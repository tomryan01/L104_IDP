#This is the main class, which is executed at run time and from which all other code is run

from Behaviour import Behaviour

TIME_STEP = 64

#note that when we create a robot instance we create a BEHAVIOUR
#this is the class that inherits all others, do not create an instance
#of MyRobot
testRobot = Behaviour()

testRobot.setWheels(["wheel1", "wheel2"])
testRobot.setArms(["arm_1", "arm_2"])
testRobot.setDistanceSensors(["ds_mid"])
testRobot.setColourSensors(["camera1"])

while testRobot.robot.step(TIME_STEP) != -1:
    #every single tick we call the reset method to
    #set all motor velocities etc. to 0, then when findBlocks() is
    #called they're set to the appropriate values given the
    #current conditions
    testRobot.reset()
    testRobot.findBlocks()
