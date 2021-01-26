"""gps_test_controller controller."""

from controller import Robot
import math

#inital stuff
TIME_STEP = 64
robot = Robot()

#intro distance sensors
ds = []
dsNames = ['ds_mid']
for i in range(1):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

#intro wheels
wheels = []
wheelsNames = ['wheel1', 'wheel2']
for i in range(2):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

#intro arms
arms = []
armsNames = ['arm_1', 'arm_2']
for i in range(2):
    arms.append(robot.getDevice(armsNames[i]))
    arms[i].setPosition(float('inf'))
    arms[i].setVelocity(0.0)

#intro camera
#cameras = []
#cameraNames = ['red_camera']
#for i  in range(1):
#    cameras.append(robot.getDevice(cameraNames[i]))
#    cameras[i].enable(TIME_STEP)
camera = robot.getDevice('camera1')
camera.enable(TIME_STEP)


#intro gps
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

#pick up function
def pick_up():
    arm_speed = 1
    arms[0].setVelocity(arm_speed)
    arms[1].setVelocity(arm_speed)
    arm_speed1 = 2
    arms[0].setPosition(arm_speed1)
    arms[1].setPosition(arm_speed1)

#find intial position
robot_position_xyz = gps.getValues()
robot_position_xz = [robot_position_xyz[0], robot_position_xyz[2]]
robot_velocity = [0, 0]
heading = [0, 0]
turning = True

#find magnitude of a vector
def get_magnitude(l):
    return (l[0]**2 + l[1]**2)**0.5

#find dot product of vectors
def dot_product(a ,b):
    return a[0]*b[0]+a[1]*b[1]

#find cross product of vectors
def cross_product(a, b):
    return a[0]*b[1]-a[1]*b[0]

def get_me_to(coordinate):
    global robot_position_xz, turning, arrived

    #calc positions
    previous_position = robot_position_xz
    robot_position_xyz = gps.getValues()
    robot_position_xz = [robot_position_xyz[0], robot_position_xyz[2]]
    #robot_speed = gps.getSpeed()

    #calc velcoity
    for i in range(2):
        robot_velocity[i] = (robot_position_xz[i] - previous_position[i])/(64e-3)

    #find required velocity direction
    for i in range(2):
        heading[i] = desired_position[i] - robot_position_xz[i]
    
    dot_head_vel = dot_product(heading, robot_velocity)
    #cross_head_vel = cross_product(heading, robot_velocity)

    #if robot is turning then turn
    if turning:
        #check not pointing backwards
        #correct_direction = False
        #if (heading[0] > 0 and robot_velocity[0] > 0) or (heading[0] < 0 and robot_velocity[0] < 0):
        #    correct_direction = True
        #if pointing the right way and forwards, stop turning
        if (abs(dot_head_vel) <  get_magnitude(heading) * get_magnitude(robot_velocity) * 0.05):
            turning = False
        else:
            turning = True
    #if not turning, check on right course
    else:
        distance_left = get_magnitude([robot_position_xz[0]-coordinate[0], robot_position_xz[1]-coordinate[1]])
        #check parallel heading and velocity if not too close to destination
        if (dot_head_vel < get_magnitude(heading) * get_magnitude(robot_velocity) * 0.99) and distance_left > 0.2:
            turning = True
    
    distance_left = get_magnitude([robot_position_xz[0]-coordinate[0], robot_position_xz[1]-coordinate[1]])
    if distance_left < 0.05:
        leftSpeed = 0
        rightSpeed = 0
        arrived = True

    #if turning then turn and if not then go to destination
    if not(arrived):
        if turning:
            leftSpeed = -1
            rightSpeed = 1
        else:
            leftSpeed = 5
            rightSpeed = 5
    
    return leftSpeed, rightSpeed


#main
while robot.step(TIME_STEP) != -1:

    arrived = False
    desired_position = [1,1]
    leftSpeed, rightSpeed = get_me_to(desired_position)

    
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass