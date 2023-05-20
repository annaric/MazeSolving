import time

class ThymioRLControl():
    def __init__(self):
        self.lastMove="WEST"

    def reset(self):
        ''' Resets the simulation'''
        self.lastMove="WEST"

    def setMove(self, move):
        self.lastMove=move

    def getMove(self):
        return self.lastMove       

    def getProximity(self):
        return 0

    def setSpeeds(self,left,right):
        motor_left_target = left
        motor_right_target = right

    def north(self,distance):
        lastMove = self.getMove()
        if lastMove == "EAST":
            self.turn(-90)
        elif lastMove == "SOUTH":
            self.turn(180)
        elif lastMove == "WEST":
            self.turn(90)
        
        prox=self.getProximity()
        self.setMove("NORTH")
        if (prox > 0):
            return False
        else:
            self.forward(distance)
            return True
        
    def east(self,distance):
        lastMove = self.getMove()
        if lastMove == "NORTH":
            self.turn(90)
        elif lastMove == "SOUTH":
            self.turn(-90)
        elif lastMove == "WEST":
            self.turn(180)

        prox=self.getProximity()
        self.setMove("EAST")
        if (prox > 0):
            return False
        else:
            self.forward(distance)
            return True

    def south(self,distance):
        lastMove = self.getMove()
        if lastMove == "NORTH":
            self.turn(180)
        elif lastMove == "EAST":
            self.turn(90)
        elif lastMove == "WEST":
            self.turn(-90)

        prox=self.getProximity()
        self.setMove("SOUTH")
        if (prox > 0):
            return False
        else:
            self.forward(distance)
            return True

    def west(self,distance):
        lastMove = self.getMove()
        if lastMove == "NORTH":
            self.turn(-90)
        elif lastMove == "EAST":
            self.turn(180)
        elif lastMove == "SOUTH":
            self.turn(90)

        prox=self.getProximity()
        self.setMove("WEST")
        if (prox > 0):
            return False
        else:
            self.forward(distance)
            return True

    def forward(self,distance):
        '''Robot moves forward a given distance'''
        speed=1.0
        self.setSpeeds(speed,speed)
        time.sleep(2)
        self.setSpeeds(0,0)

    def turn(self,angle):
        speed=1.0
        if angle>0:
            self.setSpeeds(-speed,speed)
        else:
            self.setSpeeds(speed,-speed)
        time.sleep(2)
        self.setSpeeds(0,0)
