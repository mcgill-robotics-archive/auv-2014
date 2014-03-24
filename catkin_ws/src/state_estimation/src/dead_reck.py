import math

class dead_reck():
    #This estimator makes the assumption that pitch and roll are always zero
    hasTarget = False

    #Position of target
    x = 0
    y = 0
    z = 0

    #Velocity of target
    vx = 0
    vy = 0
    vz = 0

    #I'm assuming here that the targets have IDs
    targetID = 255# A large value that we will never reach.

    #These angles are along z = down. AKA measured clockwise
    yawOfTarget = 0
    pitchOfTarget = 0 #Currently unused - always zero
    heading = -123456789

    depth = 0
    last_time = None



    def __init__(self, ros):
        self.ros = ros

    def updateOrientation(self, newHeading):
        if self.heading != -123456789:
            headingChange = newHeading - self.heading
            self.yawOfTarget -= headingChange
            self.x = self.x*math.cos(headingChange) + self.y*math.sin(headingChange)
            self.y = self.y*math.cos(headingChange) - self.x*math.sin(headingChange)
        self.heading = newHeading

    def updateCV(self, hasTarget, targetID, x, y, z, yawOfTarget, pitchOfTarget):
        if hasTarget:
            if targetID != self.targetID:
                self.last_time = None


            if self.last_time:
                t_change = self.ros.get_time() - self.last_time
                #self.vx = (x - self.x)/t_change
                #self.vy = (y - self.y)/t_change
                #self.vz = (z - self.z)/t_change

            self.last_time = self.ros.get_time()

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
        t_change = 0
        if self.last_time:
            t_change = self.ros.get_time() - self.last_time

        return {
                'hasTarget':self.hasTarget
                ,'x':self.x + self.vx*t_change
                ,'y':self.y + self.vy*t_change
                ,'z':self.z + self.vz*t_change
                ,'targetID':self.targetID
                ,'yawOfTarget':self.yawOfTarget
                ,'pitchOfTarget':self.pitchOfTarget
                ,'depth':self.depth
                }