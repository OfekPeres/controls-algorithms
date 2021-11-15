import math
import numpy as np
class DubinsCar:

    def __init__(self, l, x, y, theta, initialPhi, maxSpeed=5) -> None:
        """
        l is the length of the car
        maxSpeed is the max speed of the car
        """
        self.l = l
        self.speed = maxSpeed
        self.phi = initialPhi
        self.pos = np.array([x,y])
        self.theta = theta
        self.prevState = None
        self.maxSteer = 0.5


    def step(self, u) -> None:
        '''
        u is the control input that controls phi
        '''
        self.phi = u
        x_dot =  self.speed*np.cos(self.theta)
        y_dot = self.speed*np.sin(self.theta)
        theta_dot = (self.speed/self.l)*np.tan(self.phi)
        self.pos = self.pos + np.array([x_dot, y_dot])
        self.theta = self.theta + theta_dot


    def calcThetaError(self, goalPoint):
        """
        Given a target point, calculate the theta error
        """
        carHeading = np.array([np.cos(self.theta), np.sin(self.theta)])
        # thetaError = np.arccos(np.dot(carHeading,goalPoint))
        angle = math.atan2( carHeading[0]*goalPoint[1] - carHeading[1]*goalPoint[0], 
                            carHeading[0]*goalPoint[0]+carHeading[1]*goalPoint[1])
        return angle


if __name__ == "__main__":
    car = DubinsCar(10,0,0,0,0,10)
    target = np.array([40,40])
    thetaErr = car.calcThetaError(target)
    print(np.rad2deg(thetaErr))