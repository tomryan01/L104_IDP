"""arm_test_controller controller."""

from controller import Robot

TIME_STEP = 64
robot = Robot()
ds = []
dsNames = ['ds_mid']
for i in range(1):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)
wheels = []
wheelsNames = ['wheel1', 'wheel2']
for i in range(2):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
arms = []
armsNames = ['arm_1', 'arm_2']
for i in range(2):
    arms.append(robot.getDevice(armsNames[i]))
    arms[i].setPosition(float('inf'))
    arms[i].setVelocity(0.0)
#avoidObstacleCounter = 0

def pick_up():
    arm_speed = 1
    arms[0].setVelocity(arm_speed)
    arms[1].setVelocity(arm_speed)
    arm_speed1 = 2
    arms[0].setPosition(arm_speed1)
    arms[1].setPosition(arm_speed1)

while robot.step(TIME_STEP) != -1:
    leftSpeed = 1.0
    rightSpeed = 1.0

    arm_speed = 0
    if ds[0].getValue() > 998:
        leftSpeed = 0
        rightSpeed = 0
        pick_up()
    
    #if avoidObstacleCounter > 0:
    #    avoidObstacleCounter -= 1
    #    leftSpeed = 1.0
    #    rightSpeed = -1.0
    #else:  # read sensors
    #    for i in range(2):
    #        if ds[i].getValue() < 950.0:
    #            avoidObstacleCounter = 100
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass