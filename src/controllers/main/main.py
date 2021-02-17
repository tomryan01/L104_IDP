#This is the main class, which is executed at run time and from which all other code is run

from Behaviour import Behaviour

TIME_STEP = 64

#note that when we create a robot instance we create a BEHAVIOUR
#this is the class that inherits all others, do not create an instance
#of MyRobot
testRobot = Behaviour()

testRobot.setWheels(["wheel1", "wheel2"])
testRobot.setArms(["arm_1", "arm_2"])
testRobot.setDistanceSensors(["ds_mid"], TIME_STEP)
testRobot.setColourSensors(["camera1"], TIME_STEP)
testRobot.setGps(["gps_front", "gps_mid"], TIME_STEP)
testRobot.setEmitter(["emitter"])
testRobot.setReceiver(["receiver"], TIME_STEP)

while testRobot.robot.step(TIME_STEP) != -1:
    #every single tick we call the reset method to
    #set all motor velocities etc. to 0, then when findBlocks() is
    #called they're set to the appropriate values given the
    #current conditions
    testRobot.reset()
    
    testRobot.findBlocks2()


"""
When copying for second robot, things that need to be changed are:
Behaviour: 
    - block colour red/blue, spin direction
    - update setting red found blocks to 1, and can only go for 0 and 2
    - update setBlockToFind function
    - update location of home
Detection: origin position, friend corner wall definitions
Gps: my box coordinate definitions
"""