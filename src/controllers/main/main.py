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
    
    testRobot.emit_my_position()
    looking_at_friend = testRobot.looking_at_my_friend()
    testRobot.update_block_locations()
    testRobot.spin(1,1)
    testRobot.block_in_sight()
    print(len(testRobot.blockLocations))
    #testRobot.findBlocks()

    #testRobot.goToCoordinate([0.4277508075365773, 0.40352656326150915, 0])


"""
When copying for second robot, things that need to be changed are:
Behaviour: block colour red/blue, spin direction x2
Detection: origin position, friend corner wall definitions, spin direction correct coordinate
Gps: my box coordinate definitions
"""