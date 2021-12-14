"""
Instances of the bang bang class read in the current state of a Dubin's car,
and the location of a given target node. 

The class has helper methods which return instructions for the Dubin's 
car to reach its goal. 
"""

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
        '''Returns steering angle

            Args:
        target: a point with an x and y position.
 
            Returns:
        float: steering angle (left or right) for Dubin's car.

        Phi is the steeing angle of the front wheels of a Dubin's car. This method
        calcualtes 'theta error', which is the difference between the car's current
        orientation and the required orientation to point directly at the target.
        The needed steering angle (phi) is calculated based on the current and 
        desired orientation. 

        To eliminate oscilation at small theta errors, steering is only implemented for theta errors above
        the tolerance epsilon (eps).

        '''
        thetaError = self.car.calcThetaError(target)
        eps = 0.1*np.pi/180
        if thetaError > eps:
            return -self.car.maxSteer

        elif thetaError < -eps:
            return self.car.maxSteer
        else:
            return 0

    def GetControlInputsToTarget(self, target):
        '''Returns array of steering angles to get from current poisition to target

            Args:
        target: a point with x and y position.
 
            Returns:
        float[]: an array of steering angles for a Dubin's car. 
        

        This function takes in a Dubin's car and a target node. It returns an array,
        phis, which specifiy steering angle at each time step between the current state
        and its goal. 
        '''
        distToGoal = np.linalg.norm(self.car.pos - target)
        maxThetaDot = (self.car.speed/self.car.l)*np.tan(self.car.maxSteer)
        timeStepsToTurnCircle = (4*np.pi)/(maxThetaDot) 
        timeStepsToGoStraightToTarget = distToGoal/self.car.speed
        maxControlInputs = timeStepsToTurnCircle + timeStepsToGoStraightToTarget
        # THINK ABOUT THIS CUTOFF CASE
        phis = []
        while distToGoal > 1:
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
        '''Calculates an array of steering angles for a series of target points

            Returns: 
        float[]: an array of steering angles for a series of target points. 

        This method returns u, an array of steering angles to hit each of the waypoints.
        '''
        u = []
        for point in self.points:
            u.extend(self.GetControlInputsToTarget(point))
        return u        