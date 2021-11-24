import numpy as np
from numpy.core.numeric import outer
from src.dynamics import dubins_car
from src.dynamics.dubins_car import DubinsCar
from src.controls.bang_bang import BangBang
from src.controls.numerical_optimal_controls import GetOptimalPath, GetOptimalControlTry2
from src.controls.dubins_path_utils import GetOuterTangentPointsAndLines, GetInnerTangentPointsAndLines
import src.controls.reeds_shepp as rs
import matplotlib.pyplot as plt
import random

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

def testNumericalOptimalControls():
    startPos = np.array([0,0,0])
    finalPos = np.array([600,400,3.14])
    l = 50
    speed = 50
    maxSteer = 1

    GetOptimalControlTry2(l,startPos, finalPos, speed, maxSteer)

def testReedShepp():
    # ROUTE = [(-2,4,180), (2,4,0), (2,-3,90), (-5,-6,240), (-6, -7, 160), (-7,-1,80)]
    ROUTE = [(0,0,0), (-2,4,180), (0,0,0)]

    full_path = []
    total_length = 0

    for i in range(len(ROUTE) - 1):
        path = rs.get_optimal_path(ROUTE[i], ROUTE[i+1])
        full_path += path
        total_length += rs.path_length(path)

    print("Shortest path length: {}".format(round(total_length, 2)))

    for e in full_path:
        print(e) 
        # e.steering (LEFT/RIGHT/STRAIGHT), e.gear (FORWARD/BACKWARD), e.param (distance)

def testGetCircleTangents(c1,c2):    
    tangentPoints, outerTangentLines = GetOuterTangentPointsAndLines(c1,c2)
    p1,p2,p3,p4 = tangentPoints
    outerTangentLine1 = outerTangentLines[0]
    outerTangentLine2 = outerTangentLines[1]

    innerTangentPoints, innerTangentLines = GetInnerTangentPointsAndLines(c1,c2)
    if innerTangentLines is not None:
        p1_inner,p2_inner,p3_inner,p4_inner = innerTangentPoints

    # Visualize the 2 circles
    circle1 = plt.Circle(c1[:2], c1[2], color="g", alpha=0.5)
    circle2 = plt.Circle(c2[:2], c2[2], color="g", alpha=0.5)
    fig, ax = plt.subplots()
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    # Visualize the outer tangent points
    ax.scatter(p1[0],p1[1], c="r")
    ax.scatter(p2[0],p2[1], c="k")
    ax.scatter(p3[0],p3[1], c="y")
    ax.scatter(p4[0],p4[1], c="b")
    # Visualize the inner tangent points
    if innerTangentLines is not None:
        ax.scatter(p1_inner[0],p1_inner[1], c="r", s=80)
        ax.scatter(p2_inner[0],p2_inner[1], c="k", s=80)
        ax.scatter(p3_inner[0],p3_inner[1], c="m", s=80)
        ax.scatter(p4_inner[0],p4_inner[1], c="b", s=80)
    # Make the aspect ratio square so the circles dont look like ellipses
    ax.set_aspect("equal", adjustable='datalim')
    # Visualize the outer tangent lines
    plt.plot([outerTangentLine1[0][0], outerTangentLine1[1][0]], [outerTangentLine1[0][1], outerTangentLine1[1][1]])
    plt.plot([outerTangentLine2[0][0], outerTangentLine2[1][0]], [outerTangentLine2[0][1], outerTangentLine2[1][1]])
    # visualize the inner tangent lines
    if innerTangentLines is not None:
        plt.plot([innerTangentLines[0][0][0], innerTangentLines[0][1][0]], [innerTangentLines[0][0][1], innerTangentLines[0][1][1]])
        plt.plot([innerTangentLines[1][0][0], innerTangentLines[1][1][0]], [innerTangentLines[1][0][1], innerTangentLines[1][1][1]])
        
    plt.show()

def testVaryingCirclesForTangents():
    r = 10
    x1,y1 = 0,0
    x2,y2 = 10,10
    for i in range(10):
        c1 = np.array([random.random()*100, random.random()*100, r])
        c2 = np.array([random.random()*100, random.random()*100, r])
        testGetCircleTangents(c1,c2)
    


if __name__ == "__main__":
    # testBangBang()
    # testStep()
    # testThetaError()
    # testReedShepp()
    testVaryingCirclesForTangents()
