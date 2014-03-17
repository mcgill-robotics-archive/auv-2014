import math

class dead_reck():
    #This estimator makes the assumption that pitch and roll are always zero
    hasTarget = False

    #Position of target
    x = 0
    y = 0

    #I'm assuming here that the targets have IDs
    targetID = 0

    #Velocity
    time = -123456789

    #These angles are along z = down. AKA measured clockwise
    yawOfTarget = 0
    heading = -123456789


    def __init__(self):
        pass

    def updateOrientation(self, newHeading):
        if self.heading != -123456789:
            headingChange = newHeading - self.heading
            self.yawOfTarget -= headingChange
            self.x = self.x*math.cos(headingChange) + self.y*math.sin(headingChange)
            self.y = self.y*math.cos(headingChange) - self.x*math.sin(headingChange)
        self.heading = newHeading

    def updateCV(self, hasTarget, targetID, x, y, z, yawOfTarget):
        if hasTarget:
            self.targetID = targetID
            self.x = x
            self.y = y
            self.z = z
            self.yawOfTarget = yawOfTarget
        self.hasTarget = hasTarget



    def updateVelocity(self, vx, vy, time):
        if self.time != -123456789:
            timeDelta = time - self.time
            self.x += vx*timeDelta
            self.y += vy*timeDelta
        self.time = time


    def getState(self):
        return {
                'hasTarget':self.hasTarget
                ,'x':self.x
                ,'y':self.y
                ,'z':self.z
                ,'targetID':self.targetID
                ,'time':self.time
                ,'yawOfTarget':self.yawOfTarget
                }