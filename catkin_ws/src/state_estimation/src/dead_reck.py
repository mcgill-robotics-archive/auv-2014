

class dead_reck():
    #Makes the assumption that pitch and roll are always zero

    hasTarget = False
    position = [0,0,0]
    yawOfTarget = 0
    #I'm assuming here that the targets have IDs
    targetID = 0
    velocity = [0,0,0]
    bearing = 0
    heading = 0
    pitch = 0
    roll = 0

    def __init__(self):
        pass

    def updateOrientation(self, newHeading):

        pass

    def updateCV(self):
        pass

    def updateVelocity(self):
        pass

    def getState(self):
        pass