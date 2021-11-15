import numpy as np
from src.dynamics import dubins_car
from src.dynamics.dubins_car import DubinsCar
from src.controls.bang_bang import BangBang
def testBangBang():
    payloadFromFrontEnd = [{'x':40, 'y':40}]
    points = []
    for point in payloadFromFrontEnd:
        points.append(np.array([point['x'], point['y']]))

    car = DubinsCar(10, 0, 0, 0*np.pi/4, 0, 1)

    controller = BangBang(points, car)
    traj = controller.GetControlTrajectory()

    print("# of control inputs: {}".format(len(traj)))
    # print(traj)
    print("Final car position: {}".format(car.pos))



def testStep():
    car = DubinsCar(10, 0, 0, 0, 0, 1)
    # car.step(0.5)
    car.step(0.5)
    for i in range(10):
        car.step(0)
    print(car.pos)


def testThetaError():
    theta = 0
    theta = np.deg2rad(theta)
    car = DubinsCar(10,0,0,theta,0,1)
    # goalPoint = np.array([100,200])
    # controller = BangBang([[100,200], car])
    thetaErrorRadians = car.calcThetaError([1,-1])
    print(np.rad2deg(thetaErrorRadians))

if __name__ == "__main__":
    # testBangBang()
    # testStep()
    testThetaError()