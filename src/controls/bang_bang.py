import math
import numpy as np
from ..dynamics.dubins_car import DubinsCar

class BangBang:
    
    def __init__(self, points, car: DubinsCar) -> None:
        '''
        points is an array of dictionarys with desired x and y values outputted 
        from any path planning algorithm
        each point in points is a dictionary of the form {x:val, y:val2}
        '''
        self.points = points
        self.car = car

    def GetPhi(self, target):
        thetaError = self.car.calcThetaError(target)
        eps = 0.1*np.pi/180
        if thetaError > eps:
            return -self.car.maxSteer

        elif thetaError < -eps:
            return self.car.maxSteer
        else:
            return 0

    def GetControlInputsToTarget(self, target):
        distToGoal = np.linalg.norm(self.car.pos - target)
        maxThetaDot = (self.car.speed/self.car.l)*np.tan(self.car.maxSteer)
        timeStepsToTurnCircle = (4*np.pi)/(maxThetaDot) 
        timeStepsToGoStraightToTarget = distToGoal/self.car.speed
        maxControlInputs = timeStepsToTurnCircle + timeStepsToGoStraightToTarget
        print("Max control inputs: {}".format(maxControlInputs))
        # THINK ABOUT THIS CUTOFF CASE
        phis = []
        while distToGoal > self.car.l/2:
            phi = self.GetPhi(target)
            phis.append(phi)
            self.car.step(phi)
            distToGoal = np.linalg.norm(self.car.pos - target)
            # print("Phi: {}".format(phi))
            # print("Car Position: {}".format(self.car.pos))
            # print("Target: {}".format(target))
            if len(phis) > maxControlInputs:
                break
            # print("Distance to Goal: {}".format(distToGoal))
            # We think that the actual max length is pi*originaldistanceToGoal/2*car.speed
        return phis
    
    def GetControlTrajectory(self):
        u = []
        for point in self.points:
            u.extend(self.GetControlInputsToTarget(point))
        return u        