from math import cos, sin
from .dubins_path_utils import CalcDirectionalArcLength, Direction, GetAdjacentCircles, GetInnerTangentPointsAndLines, GetOuterTangentPointsAndLines
import numpy as np
import matplotlib.pyplot as plt
from ..dynamics.dubins_car import DubinsCar
"""
Notes: Due to the fact that in Dubins Path, all circles are the same radius
and circles are stored as (x,y,r), np.linalg.norm(c1-c2) will be the same as
np.linalg.norm(c1[:2] - c2[:2]) 

"""


class DubinsPath:
    def __init__(self, pCar: DubinsCar) -> None:
        self.car = pCar

    def GetDubinsPath(self, startPose: np.ndarray, goalPose: np.ndarray):
        """
        Given a starting orientation and an ending orientation, picks the 
        correct dubins path and calculates it
        @param startPose - the initial position of the car (x,y,theta)
        @param goalPose - the final position of the car (x,y,theta)
        @returns a dictionary with the form {path: [path elements], 
        waypoints:[4 points to travel to]}.
        """
        dist = np.linalg.norm(startPose[:2] - goalPose[:2])
        if dist >= self.car.turningRadius * 4:
            return self.GetCSCPath(startPose, goalPose)
        else:
            return self.GetCCCPath(startPose, goalPose)

    def GetCSCPath(self, startPose: np.ndarray, goalPose: np.ndarray):
        """
        The shortest dubins path will correspond to the path between the two
        closest adjacent circles. i.e. if the right start circle is closest to 
        the right goal circle, the path with will be RSR

        RSR -> c_start_right closest to c_goal_right
        LSL -> c_start_left closest to c_goal_left
        RSL -> c_start_right closest to c_goal_left
        LSR -> c_start_left closest to c_goal_right

        @param start: start pose (x,y,theta)
        @param goal: goal pose (x,y,theta)
        @returns a dictionary with the form {path: [path elements], 
        waypoints:[4 points to travel to]}.
        """
        c_start_right, c_start_left = GetAdjacentCircles(
            startPose, self.car.turningRadius)
        c_goal_right, c_goal_left = GetAdjacentCircles(goalPose,
                                                       self.car.turningRadius)

        RSR_dist = np.linalg.norm(c_start_right - c_goal_right)
        LSL_dist = np.linalg.norm(c_start_left  - c_goal_left)
        RSL_dist = np.linalg.norm(c_start_right - c_goal_left)
        LSR_dist = np.linalg.norm(c_start_left  - c_goal_right)

        distances = np.array([RSR_dist, LSL_dist, RSL_dist, LSR_dist])
        print(distances)
        pathFunctions = np.array(
            [self.GetRSR, self.GetLSL, self.GetRSL, self.GetLSR])
        shortestPathIndex = np.argmin(distances)
        return pathFunctions[shortestPathIndex](startPose, goalPose)

    def GetCCCPath(self, startPose: np.ndarray, goalPose: np.ndarray):
        print("Not implemented yet")
        return None

    def GetRSR(self, startPose: np.ndarray, goalPose: np.ndarray):
        """
        Get the Right -> Straight -> Right path
        @param start: start pose (x,y,theta)
        @param goal: goal pose (x,y,theta)
        @param r: the turning radius of the car
        @returns a dictionary with the form {path: [path elements], 
        waypoints:[4 points to travel to]}.
        """
        r = self.car.turningRadius
        # Calculate the center of the circle adjacent and to the 'right' of
        # the start point (in the reference frame of the car)
        x_start, y_start, theta_start = startPose
        x_goal, y_goal, theta_goal = goalPose
        cx_start = x_start + r * cos(theta_start - np.pi / 2)
        cy_start = y_start + r * sin(theta_start - np.pi / 2)
        c_start = np.array([cx_start, cy_start, r])

        cx_goal = x_goal + r * cos(theta_goal - np.pi / 2)
        cy_goal = y_goal + r * sin(theta_goal - np.pi / 2)
        c_goal = np.array([cx_goal, cy_goal, r])

        tangentPoints, tangentLines = GetOuterTangentPointsAndLines(
            c_start, c_goal)

        # Pick the WRONG tangent lines to support p5js unfortunate use of a 
        #  left handed system
        c_start_t = tangentLines[1][0]
        c_goal_t = tangentLines[1][1]

        # Turn Right from the original pose to the first tangent point
        firstRightTurnDistance = CalcDirectionalArcLength(
            c_start, startPose[:2], c_start_t, Direction.RIGHT)
        numTimeStepsForFirstRightTurn = firstRightTurnDistance / self.car.speed
        firstTurn = {
            "direction": Direction.RIGHT.name,
            "distance": firstRightTurnDistance,
            "numSteps": numTimeStepsForFirstRightTurn
        }

        # Turn right from the second tangent point to the final pose
        secondRightTurnDistance = CalcDirectionalArcLength(
            c_goal, c_goal_t, goalPose[:2], Direction.RIGHT)
        numTimeStepsForSecondrightTurn = secondRightTurnDistance / self.car.speed
        secondTurn = {
            "direction": Direction.RIGHT.name,
            "distance": secondRightTurnDistance,
            "numSteps": numTimeStepsForSecondrightTurn
        }

        # Go straight between the two circles
        straightLineDistance = np.linalg.norm(c_goal - c_start)
        numTimeStepsForStraightLine = straightLineDistance / self.car.speed
        straightSegment = {
            "direction": Direction.STRAIGHT.name,
            "distance": straightLineDistance,
            "numSteps": numTimeStepsForStraightLine
        }
        waypoints = [startPose[:2], c_start_t, c_goal_t, goalPose[:2]]
        return {
            "path": [firstTurn, straightSegment, secondTurn],
            "waypoints": waypoints,
            "totalDistance": firstRightTurnDistance + straightLineDistance +
            secondRightTurnDistance,
            "type": "RSR"
        }

    def GetLSL(self, startPose: np.ndarray, goalPose: np.ndarray):
        """
        Get the Left -> Straight -> Left path
        @param start: start pose (x,y,theta)
        @param goal: goal pose (x,y,theta)
        @param r: the turning radius of the car
        @returns a dictionary with the form {path: [path elements], 
        waypoints:[4 points to travel to]}.
        """
        r = self.car.turningRadius
        # Pick the "Left" adjacent Circles as on the LSL path they will be the
        # circles the car drives on
        _, c_start_left = GetAdjacentCircles(startPose, r)
        _, c_goal_left = GetAdjacentCircles(goalPose, r)

        # Calculate the tangent points
        tangentPoints, tangentLines = GetOuterTangentPointsAndLines(
            c_start_left, c_goal_left)

        # Pick the closest pair of tangent points on the same tangent line
         # Pick the WRONG tangent lines to support p5js unfortunate use of a 
        #  left handed system
        c_start_t = tangentLines[0][0]
        c_goal_t = tangentLines[0][1]

        # Turn LEFT from the original pose to the first tangent point
        firstLeftTurnDistance = CalcDirectionalArcLength(
            c_start_left, startPose[:2], c_start_t, Direction.LEFT)
        numStepsFirstLeftTurn = firstLeftTurnDistance / self.car.speed
        firstTurn = {
            "direction": Direction.LEFT.name,
            "distance": firstLeftTurnDistance,
            "numSteps": numStepsFirstLeftTurn
        }

        # Calculate straight section
        straightSegmentDistance = np.linalg.norm(c_goal_left - c_start_left)
        numStepsStraightSegment = straightSegmentDistance / self.car.speed

        straightSegment = {
            "direction": Direction.STRAIGHT.name,
            "distance": straightSegmentDistance,
            "numSteps": numStepsStraightSegment
        }
        secondLeftTurnDistance = CalcDirectionalArcLength(
            c_goal_left, c_goal_t, goalPose[:2], Direction.LEFT)
        numStepsSecondLeftTurn = secondLeftTurnDistance / self.car.speed

        secondTurn = {
            "direction": Direction.LEFT.name,
            "distance": secondLeftTurnDistance,
            "numSteps": numStepsSecondLeftTurn
        }

        return {
            "path": [firstTurn, straightSegment, secondTurn],
            "waypoints": [startPose[:2], c_start_t, c_goal_t, goalPose[:2]],
            "totalDistance": firstLeftTurnDistance + straightSegmentDistance +
            secondLeftTurnDistance,
            "type": "LSL"
        }

    def GetLSR(self, startPose: np.ndarray, goalPose: np.ndarray):
        """
        Get the Left -> Straight -> Right path
        @param start: start pose (x,y,theta)
        @param goal: goal pose (x,y,theta)
        @param r: the turning radius of the car
        @returns a dictionary with the form {path: [path elements], 
        waypoints:[4 points to travel to]}.
        """
        r = self.car.turningRadius
        # Pick the "Left" adjacent Circles as on the LSL path they will be the
        # circles the car drives on
        _, c_start_left = GetAdjacentCircles(startPose, r)
        c_goal_right, _ = GetAdjacentCircles(goalPose, r)

        # Calculate the tangent points
        tangentPoints, tangentLines = GetInnerTangentPointsAndLines(
            c_start_left, c_goal_right)

        # Pick the closest pair of tangent points on the same tangent line
        c_start_t = tangentLines[0][0]
        c_goal_t = tangentLines[0][1]

        firstLeftTurnDistance = CalcDirectionalArcLength(
            c_start_left, startPose[:2], c_start_t, Direction.LEFT)
        numStepsFirstLeftTurn = firstLeftTurnDistance / self.car.speed
        firstTurn = {
            "direction": Direction.LEFT.name,
            "distance": firstLeftTurnDistance,
            "numSteps": numStepsFirstLeftTurn
        }

        straightSegmentDistance = np.linalg.norm(c_start_t - c_goal_t)
        numStepsStraightSegment = straightSegmentDistance / self.car.speed

        straightSegment = {
            "direction": Direction.STRAIGHT.name,
            "distance": straightSegmentDistance,
            "numSteps": numStepsStraightSegment
        }
        secondRightTurnDistance = CalcDirectionalArcLength(
            c_goal_right, c_goal_t, goalPose[:2], Direction.RIGHT)
        numStepsSecondRightTurn = secondRightTurnDistance / self.car.speed
        secondTurn = {
            "direction": Direction.RIGHT.name,
            "distance": secondRightTurnDistance,
            "numSteps": numStepsSecondRightTurn
        }

        return {
            "path": [firstTurn, straightSegment, secondTurn],
            "waypoints": [startPose[:2], c_start_t, c_goal_t, goalPose[:2]],
            "totalDistance": firstLeftTurnDistance + straightSegmentDistance +
            secondRightTurnDistance,
            "type": "LSR"
        }

    def GetRSL(self, startPose: np.ndarray, goalPose: np.ndarray):
        """
        Get the Right -> Straight -> Left path
        @param start: start pose (x,y,theta)
        @param goal: goal pose (x,y,theta)
        @param r: the turning radius of the car
        @returns a dictionary with the form {path: [path elements], 
        waypoints:[4 points to travel to]}.
        """
        r = self.car.turningRadius
        # Pick the "Left" adjacent Circles as on the LSL path they will be the
        # circles the car drives on
        c_start_right, _ = GetAdjacentCircles(startPose, r)
        _, c_goal_left = GetAdjacentCircles(goalPose, r)

        # Calculate the tangent points
        tangentPoints, tangentLines = GetInnerTangentPointsAndLines(
            c_start_right, c_goal_left)

        # Pick the closest pair of tangent points on the same tangent line
        c_start_t = tangentLines[1][0]
        c_goal_t = tangentLines[1][1]

        firstRightTurnDistance = CalcDirectionalArcLength(
            c_start_right, startPose[:2], c_start_t, Direction.RIGHT)
        numStepsFirstLeftTurn = firstRightTurnDistance / self.car.speed
        firstTurn = {
            "direction": Direction.RIGHT.name,
            "distance": firstRightTurnDistance,
            "numSteps": numStepsFirstLeftTurn
        }

        straightSegmentDistance = np.linalg.norm(c_goal_t - c_start_t)
        numStepsStraightSegment = straightSegmentDistance / self.car.speed
        straightSegment = {
            "direction": Direction.STRAIGHT.name,
            "distance": straightSegmentDistance,
            "numSteps": numStepsStraightSegment
        }

        secondLeftTurnDistance = CalcDirectionalArcLength(
            c_goal_left, c_goal_t, goalPose[:2], Direction.LEFT)

        numStepsSecondLeftTurn = secondLeftTurnDistance / self.car.speed
        secondTurn = {
            "direction": Direction.LEFT.name,
            "distance": secondLeftTurnDistance,
            "numSteps": numStepsSecondLeftTurn
        }

        return {
            "path": [firstTurn, straightSegment, secondTurn],
            "waypoints": [startPose[:2], c_start_t, c_goal_t, goalPose[:2]],
            "totalDistance": firstRightTurnDistance + straightSegmentDistance +
            secondLeftTurnDistance,
            "type": "RSL"
        }
