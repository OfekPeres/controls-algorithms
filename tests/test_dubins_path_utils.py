from src.controls.dubins_path_utils import CalcDirectionalArcLength, Direction, GetAdjacentCircles, PickTangentLine, calculateAngleBetween
import numpy as np
import matplotlib.pyplot as plt

def test_CalcArcLength():
    radius = 5
    c1 = np.array([0, 0, radius])
    p2 = np.array([-5, 0])
    p3 = np.array([5, 0])
    arclen = CalcDirectionalArcLength(c1, p2, p3, Direction.LEFT)
    print(arclen)
    expectedArclen = np.pi * radius
    print(expectedArclen)
    assert arclen == expectedArclen

def test_rightAndLeftTurnArcLen():
    r = 5
    c1 = np.array([5,5,r])
    p2 = np.array([0,5])
    p3 = np.array([5,10])
    arclenRight = CalcDirectionalArcLength(c1, p2, p3, Direction.RIGHT)
    arclenLeft = CalcDirectionalArcLength(c1, p2, p3, Direction.LEFT)
    expectedArclenRight = np.pi/2*r
    expectedArclenLeft = 3*np.pi/2*r

    assert expectedArclenLeft == arclenLeft
    assert expectedArclenRight == arclenRight

def test_getAdjacentCircles():
    r = 1
    startPose = [0,0,0]
    goalPose = [1,1,0]
    c1,c2 = GetAdjacentCircles(startPose, r)
    c3,c4 = GetAdjacentCircles(goalPose, r)
    
    # Visualize the 4 circles
    circle1 = plt.Circle(c1[:2], c1[2], color="blue", alpha=0.5)
    circle2 = plt.Circle(c2[:2], c2[2], color="red", alpha=0.5)
    circle3 = plt.Circle(c3[:2], c3[2], color="black", alpha=0.5)
    circle4 = plt.Circle(c4[:2], c4[2], color="yellow", alpha=0.5)
    fig, ax = plt.subplots()
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    ax.add_patch(circle3)
    ax.add_patch(circle4)
    ax.set_aspect("equal", adjustable='datalim')
    ax.set_xlim([-5,10])
    ax.set_ylim([-5,10])
    plt.show()
def test_calculateAngleBetween1():
    v1 = np.array([1,0])
    v2 = np.array([0,1])
    angle = calculateAngleBetween(v1, v2)
    print(np.rad2deg(angle))
    assert np.pi/2 == angle

def test_PickTangentLine():
    startPose = np.array([0,0,0])
    goalPose = np.array([0,50,np.pi])
    radius = 10
    angle_1, angle_2 = PickTangentLine(startPose, goalPose, radius, Direction.LEFT, Direction.LEFT)
    print(np.rad2deg(angle_1))
    print(np.rad2deg(angle_2))
    assert False

