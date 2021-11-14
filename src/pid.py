import math
import numpy as np
from .dubins_car import DubinsCar

class PID:
    
    def __init__(self, points, kp, ki, kd, car: DubinsCar) -> None:
        '''
        points is an array of dictionarys with desired x and y values outputted 
        from any path planning algorithm
        each point in points is a dictionary of the form {x:val, y:val2}
        '''
        self.points = points
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.car = car

    def GetPhi(self):
        point = self.points[0]
        x,y, theta = self.car.state.values
        carHeading = np.array([np.cos(theta), np.sin(theta)])
        goalPoint = np.array([point['x'],point['y']])
        goalPoint = goalPoint / np.linalg.norm(goalPoint)
        thetaError = np.arccos(np.dot(carHeading,goalPoint))

        




if __name__ == "__main__":
    pass