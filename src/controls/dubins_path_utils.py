"""
This file contains helper functions needed for the Dubins Path Control Algorithm 
module. 
"""

import numpy as np
from math import atan, asin, atan2, sin, cos, sqrt, acos
from enum import Enum


class Direction(Enum):
    LEFT = 1
    RIGHT = -1
    STRAIGHT = 0


def calculateAngleBetween(v1: np.ndarray, v2: np.ndarray):
    '''Calculates the signed angle between two pose vectors

    Args:
        v1 (np.ndarry): start pose vector
        v2 (np.ndarray): end pose vector
    Returns:
        Float: The signed angle between v1 and v2 
        
    '''

    # Calculate the signed angle between two vectors (can potentially be
    # done with the "perpendicular dot product" but here is accomplished
    # by utilizing code inspired by p5.js source code for the angleBetween
    # function)
    theta = acos(min(1, max(-1, v1.dot(v2))))
    theta = theta * np.sign(np.cross(v1, v2)) or 1
    return theta


def CalcDirectionalArcLength(c1: np.ndarray, p2: np.ndarray, p3: np.ndarray,
                             direction: Direction) -> float:
    '''Calculates the arc length between two points on a circle

    Args:
        c1 (np.ndarray): input circle (center_x,center_y,radius)
        p2 (np.ndarray): point one on the circle (x,y)
        p3 (np.ndarray): point two on the circle (x,y)
        direction (Direction): left or right 
    Returns:
        Float: the arc length between p2 and p3

    '''
    p1 = c1[:2]
    r = c1[2]
    v1 = p2 - p1
    v2 = p3 - p1

    theta = atan2(v2[1], v2[0]) - atan2(v1[1], v1[0])
    if theta < 0 and direction == Direction.LEFT:
        theta = theta + 2 * np.pi
    elif theta > 0 and direction == Direction.RIGHT:
        theta = theta - 2 * np.pi
    return abs(theta * r)

def CalcDirectionArcAngle(c1:np.ndarray, p2:np.ndarray, p3:np.ndarray, direction:Direction):
    '''Calculates the directional angle between two points on a circle

    Args:
        c1 (np.ndarray): input circle (center_x,center_y,radius)
        p2 (np.ndarray): starting point on the circle circumference (x,y)
        p3 (np.ndarray): ending point on the circle circumference (x,y)
        direction (Direction): left or right 
    Returns:
        Float: the (directional) arc angle between p2 and p3

    '''
    p1 = c1[:2]
    v1 = p2 - p1
    v2 = p3 - p1

    theta = atan2(v2[1], v2[0]) - atan2(v1[1], v1[0])
    if theta < 0 and direction == Direction.LEFT:
        theta = theta + 2 * np.pi
    elif theta > 0 and direction == Direction.RIGHT:
        theta = theta - 2 * np.pi
    return theta


def GetOuterTangentPointsAndLines(c1: np.ndarray, c2: np.ndarray):
    '''Calculates the outer tangent points and lines between two circles
    https://en.wikipedia.org/wiki/Tangent_lines_to_circles

    Args:
        c1 (np.ndarray): circle 1 (center_x,center_y,radius)
        c2 (np.ndarray): circle 2 (center_x,center_y,radius)
    Returns:
        Tuple: (4 potential tangent points, Outer Tangent lines between tangent points)
               ([c1 up-right, c1 down-left, c2 up-right, c2 down-left], 
               [[c1 up-right, c2 up-right], [c1 down-left, c2 down-left]])

    '''

    # Calculate Outer tangents
    x1, y1, r1 = c1
    x2, y2, r2 = c2
    eps = 1e-20
    gamma = -atan((y2 - y1) / (x2 - x1 + eps))
    beta = 0  # Dubins path circles are always the same radius
    alpha = gamma - beta

    x3_1 = x1 + r1 * sin(alpha)
    x3_2 = x1 - r1 * sin(alpha)
    y3_1 = y1 + r1 * cos(alpha)
    y3_2 = y1 - r1 * cos(alpha)

    x4_1 = x2 + r2 * sin(alpha)
    x4_2 = x2 - r2 * sin(alpha)
    y4_1 = y2 + r2 * cos(alpha)
    y4_2 = y2 - r2 * cos(alpha)

    p1 = [x3_1, y3_1]
    p2 = [x3_2, y3_2]
    p3 = [x4_1, y4_1]
    p4 = [x4_2, y4_2]

    tangentPonts = [p1, p2, p3, p4]
    outerTangentLines = [[p1, p3], [p2, p4]]
    return tangentPonts, outerTangentLines


def GetInnerTangentPointsAndLines(c1, c2):
    '''Calculates the inner tangent points and lines between two circles
    https://math.stackexchange.com/questions/719758/inner-tangent-between-two-circles-formula

    Args:
        c1 (np.ndarray): circle 1 (center_x,center_y,radius)
        c2 (np.ndarray): circle 2 (center_x,center_y,radius)
    Returns:
        Tuple: (4 potential tangent points, Outer Tangent lines between tangent points)
               ([c1 top, c2 bottom, c1 bottom, c2 top], 
               [[c1 top, c2 bottom], [c1 bottom, c2 top]])

    '''

    x1, y1, r1 = c1
    x2, y2, r2 = c2
    eps = 1e-20
    hypotenuse = np.linalg.norm(c1[:2] - c2[:2])
    opposite = r1 + r2
    if opposite > hypotenuse:
        print("Cannot compute arcsin of this scenario")
        return None, None
    # Get the angle betwen the x-axis to the line from the circle center to its
    # tangent point and shift by the rotational offset between the centers of the
    # 2 circles
    phi = atan2(y2 - y1, x2 - x1 + eps) + asin(
        opposite / hypotenuse) - np.pi / 2

    p1x = x1 + r1 * cos(phi)
    p1y = y1 + r1 * sin(phi)
    p2x = x2 + r2 * cos(phi + np.pi)
    p2y = y2 + r2 * sin(phi + np.pi)
    p1 = np.array([p1x, p1y])
    p2 = np.array([p2x, p2y])

    # Flip phi to point to the other set of tangent points
    phi2 = atan2(y2 - y1, x2 - x1) - asin(opposite / hypotenuse) + np.pi / 2
    p3x = x1 + r1 * cos(phi2)
    p3y = y1 + r1 * sin(phi2)
    p4x = x2 + r2 * cos(phi2 + np.pi)
    p4y = y2 + r2 * sin(phi2 + np.pi)
    p3 = np.array([p3x, p3y])
    p4 = np.array([p4x, p4y])

    points = np.array([p1, p2, p3, p4])
    innerTangentLines = np.array([np.array([p1, p2]), np.array([p3, p4])])
    return points, innerTangentLines


def GetAdjacentCircles(p, r):
    '''Given a pose, p (x, y, theta) and a turning radius r, calculate the dubins circles around the pose

    Args:
        p (np.ndarray): the pose of the car (x,y,theta)
        r (float): turning radius of the car
    Returns:
        List: Two turning circles [[x1_center,y1_center,r1],[x2_center,y2_center,r2]]

    '''

    x, y, theta = p
    cx_right = x + r * cos(theta - np.pi / 2)
    cy_right = y + r * sin(theta - np.pi / 2)
    cx_left = x - r * cos(theta - np.pi / 2)
    cy_left = y - r * sin(theta - np.pi / 2)

    return [np.array([cx_right, cy_right, r]), np.array([cx_left, cy_left, r])]


def PickTangentLine(startPose: np.ndarray, goalPose: np.ndarray, radius: float,
                    startCircleDirection: Direction,
                    goalCircleDirection: Direction):
    '''A function that picks the correct tangent line for a dubins car to travel
    over between a start pose and an end pose when knowing which turns it is
    supposed to make (i.e. if it knows its RSR vs LSR)

    Args:
        startPose (np.ndarray): the start pose of the car (x,y,theta)
        goalPose (np.ndarray): the goal pose of the car (x,y,theta)
        radius (float): the turning radius of the car
        startCircleDirection (Direction): first turning direction
        goalCircleDirection (Direction): end turning direction
    Returns:
        List: the pair of points that creates the tangent line to travel from the 
              start circle to the goal circle [[x,y],[x,y]]

    '''
    x1, y1, theta1 = startPose
    # Pick the correct start circle
    start_circle_right, start_circle_left = GetAdjacentCircles(
        startPose, radius)
    start_circle = start_circle_right if startCircleDirection == Direction.RIGHT else start_circle_left
    # Pick the correct goal circle
    goal_circle_right, goal_circle_left = GetAdjacentCircles(goalPose, radius)
    goal_circle = goal_circle_right if goalCircleDirection == Direction.RIGHT else goal_circle_left

    # Get the tangent points and lines between the two circles
    tangentPoints, tangentLines = GetOuterTangentPointsAndLines(
        start_circle, goal_circle)

    # for the two possible tangent lines, rotate the car as if it was traveling
    # to that tangent point and see what direction it will be facing pick
    # the tangent point that when traveled to will be pointing closer to the target POSITION
    start_circle_tangent_1 = tangentLines[0][0]
    start_circle_tangent_2 = tangentLines[1][0]
    # Get the "rotation" from the start pose to each tangent point
    startHeading = np.array([cos(theta1), sin(theta1)])
    # Get the normalized direction pointing from the start pt to the goal pt
    start2goal = goalPose[:2] - startPose[:2] 
    start2goal = start2goal / np.linalg.norm(start2goal)

    # Calculate the orientation at each tangent point
    orientation_at_tangent_1 = theta1 + CalcDirectionArcAngle(start_circle,startPose[:2],start_circle_tangent_1, startCircleDirection)
    orientation_at_tangent_2 = theta1 + CalcDirectionArcAngle(start_circle,startPose[:2],start_circle_tangent_2, startCircleDirection)
    # Convert theta orientation to direction vectors
    v1 = np.array([cos(orientation_at_tangent_1), sin(orientation_at_tangent_1)])
    v2 = np.array([cos(orientation_at_tangent_2), sin(orientation_at_tangent_2)])
    # Calculate the error between the would be orientation at a given tangent point vs the desired heading of start2goal
    
    thetaErr1 = calculateAngleBetween(start2goal, v1)
    thetaErr2 = calculateAngleBetween(start2goal, v2)
    if abs(thetaErr1) < abs(thetaErr2):
        return tangentLines[0]
    else:
        return tangentLines[1]