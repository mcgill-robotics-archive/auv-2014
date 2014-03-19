import math

class dead_reck():
    #This estimator makes the assumption that pitch and roll are always zero
    hasTarget = False

    #Position of target
    x = 0
    y = 0
    z = 0

    #I'm assuming here that the targets have IDs
    targetID = 0

    #These angles are along z = down. AKA measured clockwise
    yawOfTarget = 0
    pitchOfTarget = 0 #Currently unused - always zero
    heading = -123456789

    depth = 0


    def __init__(self):
        pass

    def updateOrientation(self, newHeading):
        if self.heading != -123456789:
            headingChange = newHeading - self.heading
            self.yawOfTarget -= headingChange
            self.x = self.x*math.cos(headingChange) + self.y*math.sin(headingChange)
            self.y = self.y*math.cos(headingChange) - self.x*math.sin(headingChange)
        self.heading = newHeading

    def updateCV(self, hasTarget, targetID, x, y, z, yawOfTarget, pitchOfTarget):
        if hasTarget:
            self.targetID = targetID
            self.x = x
            self.y = y
            self.z = z
            self.yawOfTarget = yawOfTarget
            self.pitchOfTarget = pitchOfTarget
        self.hasTarget = hasTarget

    def updateDepth(self, depth):
        self.depth = depth

    def getState(self):
        return {
                'hasTarget':self.hasTarget
                ,'x':self.x
                ,'y':self.y
                ,'z':self.z
                ,'targetID':self.targetID
                ,'yawOfTarget':self.yawOfTarget
                ,'pitchOfTarget':self.pitchOfTarget
                ,'depth':self.depth
                }