import math
import numpy as np
from dubins_car import DubinsCar

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
        eps = 1e-3
        if thetaError > eps:
            return self.car.maxSteer

        elif thetaError < -eps:
            return -self.car.maxSteer
        else:
            return 0

    def GetControlInputsToTarget(self, target):
        distToGoal = np.linalg.norm(self.car.pos - target)
        maxControlInputs = np.pi*distToGoal/(2*self.car.speed)
        # THINK ABOUT THIS CUTOFF CASE
        print(self.car.pos)
        phis = []
        while distToGoal > self.car.l:
            phi = self.GetPhi(target)
            phis.append(phi)
            distToGoal = np.linalg.norm(self.car.pos - target)
            self.car.step(phi)
            print("Phi: {}".format(phi))
            print("Car Position: {}".format(self.car.pos))
            print("Target: {}".format(target))

            print("Distance to Goal: {}".format(distToGoal))
            # We think that the actual max length is pi*originaldistanceToGoal/2*car.speed
        return phis
    
    def GetControlTrajectory(self):
        u = []
        for point in self.points:
            u.extend(self.GetControlInputsToTarget(point))
        return u
        


if __name__ == "__main__":
    payloadFromFrontEnd = [{'x':40, 'y':40}, {'x':80, 'y':80}]
    points = []
    for point in payloadFromFrontEnd:
        points.append(np.array([point['x'], point['y']]))

    car = DubinsCar(10, 0, 0, 0*np.pi/4, 0, 10)

    controller = BangBang(points, car)
    traj = controller.GetControlTrajectory()

    print("# of control inputs: {}".format(len(traj)))
    print(traj)
    print("0 control input: {}".format(np.where(traj == 0)))
    print(np.where(traj <0))
    # np.where(traj >0)
    print("Final car position: {}".format(car.pos))


        