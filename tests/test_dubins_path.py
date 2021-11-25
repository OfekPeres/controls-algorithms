import pprint
from src.controls.dubins_path_utils import CalcDirectionalArcLength, Direction, GetAdjacentCircles
from src.controls.dubins_path import DubinsPath
from src.dynamics.dubins_car import DubinsCar
from numpy import pi
import numpy as np
import matplotlib.pyplot as plt


def test_RSR():
    l = 5
    car = DubinsCar(l, 0, 0, 0, 0, 1)
    path_generator = DubinsPath(car)

    startPose = np.array([0, 0, pi / 2])
    goalPose = np.array([50, 0, -pi / 2])
    output = path_generator.GetRSR(startPose, goalPose)
    print(f'short RSR {output["totalDistance"]=}')
    c_start_right, _ = GetAdjacentCircles(startPose, car.turningRadius)
    c_goal_right, _ = GetAdjacentCircles(goalPose, car.turningRadius)

    # circle1 = plt.Circle(c_start_right[:2], c_start_right[2], color="blue", alpha=0.5)
    # circle2 = plt.Circle(c_goal_right[:2], c_goal_right[2], color="red", alpha=0.5)
    # fig, ax = plt.subplots()
    # ax.add_patch(circle1)
    # ax.add_patch(circle2)
    # ax.set_aspect("equal", adjustable='datalim')
    # # Plot waypoints
    # for i, p in enumerate(output['waypoints']):
    #     plt.scatter(p[0], p[1])
    #     plt.text(p[0], p[1], s=i)
    # plt.show()


def test_LSL():
    l = 5
    car = DubinsCar(l, 0, 0, 0, 0, 1)
    path_generator = DubinsPath(car)

    startPose = np.array([0, 0, pi / 2])
    goalPose = np.array([-50, 0, -pi / 2])
    output = path_generator.GetLSL(startPose, goalPose)
    print(f'short RSR {output["totalDistance"]=}')
    _, c_start_left = GetAdjacentCircles(startPose, car.turningRadius)
    _, c_goal_left = GetAdjacentCircles(goalPose, car.turningRadius)

    # circle1 = plt.Circle(c_start_left[:2],
    #                      c_start_left[2],
    #                      color="blue",
    #                      alpha=0.5)
    # circle2 = plt.Circle(c_goal_left[:2],
    #                      c_goal_left[2],
    #                      color="red",
    #                      alpha=0.5)
    # fig, ax = plt.subplots()
    # ax.add_patch(circle1)
    # ax.add_patch(circle2)
    # ax.set_aspect("equal", adjustable='datalim')
    # # Plot waypoints
    # for i, p in enumerate(output['waypoints']):
    #     plt.scatter(p[0], p[1])
    #     plt.text(p[0], p[1], s=i)
    # plt.show()


def test_lowerBoundForCCC():
    l = 5
    car = DubinsCar(l, 0, 0, 0, 0, 1)
    path_generator = DubinsPath(car)

    turningRadius = car.turningRadius
    startPose = np.array([0, 0, pi / 2])
    goalPose = np.array([3 * turningRadius, 0, -pi / 2])
    c_start_right, c_start_left = GetAdjacentCircles(startPose,
                                                     car.turningRadius)
    c_goal_right, c_goal_left = GetAdjacentCircles(goalPose, car.turningRadius)
    # circle1 = plt.Circle(c_start_left[:2],
    #                      c_start_left[2],
    #                      color="blue",
    #                      alpha=0.5)
    # circle2 = plt.Circle(c_start_right[:2],
    #                      c_start_right[2],
    #                      color="red",
    #                      alpha=0.5)
    # circle3 = plt.Circle(c_goal_left[:2],
    #                      c_goal_left[2],
    #                      color="blue",
    #                      alpha=0.5)
    # circle4 = plt.Circle(c_goal_right[:2],
    #                      c_goal_right[2],
    #                      color="red",
    #                      alpha=0.5)
    # fig, ax = plt.subplots()
    # ax.add_patch(circle1)
    # ax.add_patch(circle2)
    # ax.add_patch(circle3)
    # ax.add_patch(circle4)
    # plt.text(c_start_right[0],c_start_right[1], s=1)
    # plt.text(c_start_left[0],c_start_left[1], s=2)
    # plt.text(c_goal_right[0],c_goal_right[1], s=3)
    # plt.text(c_goal_left[0],c_goal_left[1], s=4)
    # ax.set_aspect("equal", adjustable="datalim")
    # ax.set_xlim([-50, 100])
    # ax.set_ylim([-50, 100])
    # plt.show()


def test_LSR():
    l = 5
    car = DubinsCar(l, 0, 0, 0, 0, 1)
    path_generator = DubinsPath(car)

    startPose = np.array([0, 0, pi / 2])
    goalPose = np.array([-50, 0, pi / 2])
    output = path_generator.GetLSR(startPose, goalPose)
    print(f'short LSR {output["totalDistance"]=}')
    _, c_start_left = GetAdjacentCircles(startPose, car.turningRadius)
    c_goal_right, _ = GetAdjacentCircles(goalPose, car.turningRadius)

    # circle1 = plt.Circle(c_start_left[:2],
    #                      c_start_left[2],
    #                      color="blue",
    #                      alpha=0.5)
    # circle2 = plt.Circle(c_goal_right[:2],
    #                      c_goal_right[2],
    #                      color="red",
    #                      alpha=0.5)
    # fig, ax = plt.subplots()
    # ax.add_patch(circle1)
    # ax.add_patch(circle2)
    # ax.set_aspect("equal", adjustable='datalim')
    # # Plot waypoints
    # for i, p in enumerate(output['waypoints']):
    #     plt.scatter(p[0], p[1])
    #     plt.text(p[0], p[1], s=i)
    # plt.show()


def test_RSL():
    l = 5
    car = DubinsCar(l, 0, 0, 0, 0, 1)
    path_generator = DubinsPath(car)

    startPose = np.array([0, 0, pi / 2])
    goalPose  = np.array([50, 20, pi / 2])
    output = path_generator.GetRSL(startPose, goalPose)
    print(f'short RSL {output["totalDistance"]=}')
    c_start_right, _ = GetAdjacentCircles(startPose, car.turningRadius)
    _, c_goal_left = GetAdjacentCircles(goalPose, car.turningRadius)

    # circle1 = plt.Circle(c_start_right[:2],
    #                      c_start_right[2],
    #                      color="blue",
    #                      alpha=0.5)
    # circle2 = plt.Circle(c_goal_left[:2],
    #                      c_goal_left[2],
    #                      color="red",
    #                      alpha=0.5)
    # fig, ax = plt.subplots()
    # ax.add_patch(circle1)
    # ax.add_patch(circle2)
    # ax.set_aspect("equal", adjustable='datalim')
    # # Plot waypoints
    # for i, p in enumerate(output['waypoints']):
    #     plt.scatter(p[0], p[1])
    #     plt.text(p[0], p[1], s=i)
    # plt.show()