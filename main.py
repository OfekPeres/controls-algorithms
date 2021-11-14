import numpy as np
from src.dynamics.dubins_car import DubinsCar
from src.controls.bang_bang import BangBang
def testBangBang():
    payloadFromFrontEnd = [{'x':40, 'y':40}, {'x':80, 'y':80}]
    points = []
    for point in payloadFromFrontEnd:
        points.append(np.array([point['x'], point['y']]))

    car = DubinsCar(10, 0, 0, 0*np.pi/4, 0, 10)

    controller = BangBang(points, car)
    traj = controller.GetControlTrajectory()

    print("# of control inputs: {}".format(len(traj)))
    print(traj)
    print("Final car position: {}".format(car.pos))

if __name__ == "__main__":
    testBangBang()