from MyRobot import MyRobot

class Detection(MyRobot):

    def block_in_sight(self):
        #First iteration: return true if distance sensor less than 1.5m
        if self.distanceSensors[0].getValue() < 1500:
            return True
        else:
            return False

    def block_colour(self):
        pass

    def block_in_distance(self):
        #return true if distance sensor less than 1cm
        if self.distanceSensors[0].getValue() < 10:
            return True
        else:
            return False