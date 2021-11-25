from src.controls.dubins_path_utils import CalcDirectionalArcLength, Direction, GetAdjacentCircles
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
    pose = [1,1,-np.pi/4]
    c1,c2 = GetAdjacentCircles(pose, r)
    print(c1)
    print(c2)
    # Visualize the 2 circles
    # circle1 = plt.Circle(c1[:2], c1[2], color="blue", alpha=0.5)
    # circle2 = plt.Circle(c2[:2], c2[2], color="red", alpha=0.5)
    # fig, ax = plt.subplots()
    # ax.add_patch(circle1)
    # ax.add_patch(circle2)
    # ax.set_aspect("equal", adjustable='datalim')
    # ax.set_xlim([-5,10])
    # ax.set_ylim([-5,10])
    # plt.show()
