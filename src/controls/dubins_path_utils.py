from typing import List
import numpy as np
from math import atan, asin, atan2, sin, cos, sqrt


def GetOuterTangentPointsAndLines(c1: np.ndarray, c2: np.ndarray):
    """
    Both define a circle
    c2 is an array of (x2,y2,r2)
    c1 is an array of (x1,y1,r1)
    
    https://en.wikipedia.org/wiki/Tangent_lines_to_circles
    
    @returns the 4 potential tangent points
    """

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
    """
    c1 and c2 are both circles of form (x, y, r)
    """
    x1, y1, r1 = c1
    x2, y2, r2 = c2
    eps = 1e-20
    hypotenuse = np.linalg.norm(c1[:2] - c2[:2])
    opposite = r1 + r2
    if opposite > hypotenuse:
        print("Cannot compute arcsin of this scenario")
        return None,None
    # Get the angle betwen the x-axis to the line from the circle center to its
    # tangent point and shift by the rotational offset between the centers of the
    # 2 circles
    phi = atan2(y2 - y1, x2 - x1 + eps) + asin(opposite / hypotenuse)  - np.pi / 2

    p1x = x1 + r1*cos(phi)
    p1y = y1 + r1*sin(phi)
    p2x = x2 + r2*cos(phi + np.pi)
    p2y = y2 + r2*sin(phi + np.pi)
    p1 = [p1x, p1y]
    p2 = [p2x, p2y]
    
    # Flip phi to point to the other set of tangent points
    phi2 = atan2(y2 - y1, x2 - x1) - asin(opposite / hypotenuse) + np.pi / 2
    p3x = x1 + r1*cos(phi2)
    p3y = y1 + r1*sin(phi2)
    p4x = x2 + r2*cos(phi2 + np.pi)
    p4y = y2 + r2*sin(phi2 + np.pi)
    p3 = [p3x, p3y]
    p4 = [p4x, p4y]

    points = [p1,p2,p3,p4]
    innerTangentLines = [[p1,p2], [p3,p4]]
    return points, innerTangentLines
