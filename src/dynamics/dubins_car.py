"""
The DubinsCar class models the dynamics of a Dubin's car as well as stores
the state of a single vehicle and allows updates to the pose via the step 
function
"""
import math
from typing_extensions import Self
import numpy as np
class DubinsCar:
    '''Initializes a Dubins Car Instance

    Args:
        l: length of the car.
        x: initial x position.
        y: initial y position.
        theta: initial orientation.
        initialPhi: initial wheel orientation.
        maxSpeed: the max speed of the car.
    Returns:
        An instance of DubinsCar 
        
    '''

    def __init__(self, l:float, x:float, y:float, theta:float, initialPhi:float, maxSpeed:float=5) -> Self:
        self.l = l
        self.speed = maxSpeed
        self.phi = initialPhi
        self.pos = np.array([x,y])
        self.theta = theta
        self.prevState = None
        self.maxSteer = 0.5
        self.turningRadius = self.l / math.tan(self.maxSteer)


    def step(self, u:float) -> None:
        ''' Steps the dynamics forwards one time step with the specified turn angle u
        
        Args
            u: is the control input that controls phi
        '''
        self.phi = u
        x_dot =  self.speed*np.cos(self.theta)
        y_dot = self.speed*np.sin(self.theta)
        theta_dot = (self.speed/self.l)*np.tan(self.phi)
        self.pos = self.pos + np.array([x_dot, y_dot])
        self.theta = self.theta + theta_dot


    def calcThetaError(self, goalPoint:np.ndarray) -> float:
        """Calculates the directional angle error between the direction the car
        is facing and where it should be facing if it were to travel straight 
        to the goal point
        
        Args:
            goalPoint (numpy array): the (x,y) position that the car should travel to
        
        Returns:
            float: The signed angle theta between the car's direction and the direction towards the goal

        """
        # This step is highly important - get the vector pointing from the car 
        # to the target point
        car2goal = self.pos - goalPoint
        car2goal = car2goal / np.linalg.norm(car2goal)
        carHeading = np.array([np.cos(self.theta), np.sin(self.theta)])
        
        # Calculate the signed angle between two vectors (can potentially be 
        # done with the "perpendicular dot product" but here is accomplished
        # by utilizing code inspired by p5.js source code for the angleBetween
        # function)
        theta = math.acos(min(1, max(-1, carHeading.dot(car2goal))))
        theta = theta * np.sign(np.cross(carHeading, car2goal)) or 1
        return theta



if __name__ == "__main__":
    car = DubinsCar(10,0,0,0,0,10)
    target = np.array([40,40])
    thetaErr = car.calcThetaError(target)
    print(np.rad2deg(thetaErr))