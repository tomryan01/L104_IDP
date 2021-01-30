from MyRobot import MyRobot

#to inherit a class in python we pass it as a parameter, its weird
#since 'arms' for example is an attribute in MyRobot, because this inherits MyRobot, we can 
#use self.arms to reference 'arms'
class Drive(MyRobot):

    def forwards(self, speed):
        self.wheels[0].setVelocity(speed)
        self.wheels[1].setVelocity(speed)
        self.wheels[0].setPosition(float('inf'))
        self.wheels[1].setPosition(float('inf'))

    def backwards(self):
        pass

    def spin(self):
        pass
