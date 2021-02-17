from MyRobot import MyRobot

#to inherit a class in python we pass it as a parameter, its weird
#since 'arms' for example is an attribute in MyRobot, because this inherits MyRobot, we can 
#use self.arms to reference 'arms'
class Drive(MyRobot):

    def forwards(self, speed):
        self.wheels[0].setVelocity(speed)
        self.wheels[1].setVelocity(speed)

    def backwards(self, speed):
        self.wheels[0].setVelocity(-1 * speed)
        self.wheels[1].setVelocity(-1 * speed)

    def spin(self, speed, direction):
        "direction parameter is -1 to left, 1 to right"
        assert direction == -1 or direction == 1
        self.wheels[0].setVelocity(direction * speed)
        self.wheels[1].setVelocity(-1 * direction * speed)
