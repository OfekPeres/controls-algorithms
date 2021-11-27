from math import atan2, cos, sin, acos, sqrt
from pprint import pprint
from .dubins_path_utils import CalcDirectionalArcLength, Direction, GetAdjacentCircles, GetInnerTangentPointsAndLines, GetOuterTangentPointsAndLines, PickTangentLine
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

        # Check if LRL is feasible by checking if the two left circles are
        # within 4*turning_radius of each other
        # repeat for RLR, if not feasible do CSC paths
        r = self.car.turningRadius
        c_start_right, c_start_left = GetAdjacentCircles(startPose, r)
        c_goal_right, c_goal_left = GetAdjacentCircles(goalPose, r)
        # Check LRL
        left_circle_dist = np.linalg.norm(c_start_left[:2] - c_goal_left[:2])
        right_circle_dist = np.linalg.norm(c_start_right[:2] -
                                           c_goal_right[:2])

        # Pick the path algorithm based on which turning circles are closest!
        
        if left_circle_dist < 4 * r or right_circle_dist < 4*r: 
            if left_circle_dist < right_circle_dist:
                return self.GetLRL(startPose, goalPose)
            elif right_circle_dist < left_circle_dist:
                return self.GetRLR(startPose, goalPose)
            else:
                print("PANIK! this should be impossible")
        else:
            return self.GetCSCPath(startPose, goalPose)

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
        LSL_dist = np.linalg.norm(c_start_left - c_goal_left)
        RSL_dist = np.linalg.norm(c_start_right - c_goal_left)
        LSR_dist = np.linalg.norm(c_start_left - c_goal_right)

        distances = np.array([RSR_dist, LSL_dist, RSL_dist, LSR_dist])
        pathFunctions = np.array(
            [self.GetRSR, self.GetLSL, self.GetRSL, self.GetLSR])
        shortestPathIndex = np.argmin(distances)
        return pathFunctions[shortestPathIndex](startPose, goalPose)

    def GetCCCPath(self, startPose: np.ndarray, goalPose: np.ndarray):

        self.GetLRL(startPose, goalPose)
        return None

    def GetRLR(self, startPose: np.ndarray, goalPose: np.ndarray):
        r = self.car.turningRadius
        c1, _ = GetAdjacentCircles(startPose, self.car.turningRadius)
        c2, _ = GetAdjacentCircles(goalPose, self.car.turningRadius)
        c1Toc2 = c2[:2] - c1[:2]
        D = np.linalg.norm(c1Toc2)
        # normalize c1Toc2
        c1Toc2 = c1Toc2 / D
        midPt = c1[:2] + c1Toc2 * D / 2
        # Use the definition of perpendicular to get direction to center of c3
        vec2p3 = -np.array([c2[1] - c1[1], -(c2[0] - c1[0])]) / D
        # From geometry
        distMidPtToP3 = sqrt(4 * r**2 - D**2 / 4)
        # The center of the tangent circle
        p3 = midPt + vec2p3 * distMidPtToP3
        c3 = np.array([p3[0], p3[1], r])

        # Find the tangent pt between c1 and c3
        c1Toc3 = c3[:2] - c1[:2]
        c1Toc3 = c1Toc3 / np.linalg.norm(c1Toc3)
        c1c3_tangent = c1[:2] + c1Toc3 * r

        # Find the tangent pt between c2 and c3
        c2ToC3 = p3 - c2[:2]
        c2ToC3 = c2ToC3 / np.linalg.norm(c2ToC3)
        c2c3_tangent = c2[:2] + c2ToC3 * r

        # Calculate the three turns

        # First: Right Turn - from origin to first tangent pt
        firstRightTurnDistance = CalcDirectionalArcLength(
            c1, startPose[:2], c1c3_tangent, Direction.RIGHT)
        numTimeStepsForFirstRightTurn = firstRightTurnDistance / self.car.speed
        firstTurn = {
            "direction": Direction.RIGHT.name,
            "distance": firstRightTurnDistance,
            "numSteps": numTimeStepsForFirstRightTurn
        }
        # Second: Left Turn - on c3 from c1c3 tangent to c2c3 tangent
        secondLeftTurnDistance = CalcDirectionalArcLength(
            c3, c1c3_tangent, c2c3_tangent, Direction.LEFT)
        numTimeStepsForSecondLeftTurn = secondLeftTurnDistance / self.car.speed
        secondTurn = {
            "direction": Direction.LEFT.name,
            "distance": secondLeftTurnDistance,
            "numSteps": numTimeStepsForSecondLeftTurn
        }

        # Third: Right Turn - from c2c3 tangent to goal pt
        thirdRightTurnDistance = CalcDirectionalArcLength(
            c2, c2c3_tangent, goalPose[:2], Direction.RIGHT)
        numTimeStepsForThirdRightTurn = thirdRightTurnDistance / self.car.speed
        thirdTurn = {
            "direction": Direction.RIGHT.name,
            "distance": thirdRightTurnDistance,
            "numSteps": numTimeStepsForThirdRightTurn
        }

        waypoints = [startPose[:2], c1c3_tangent, c2c3_tangent, goalPose[:2]]
        return {
            "path": [firstTurn, secondTurn, thirdTurn],
            "waypoints": waypoints,
            "totalDistance": firstRightTurnDistance + secondLeftTurnDistance +
            thirdRightTurnDistance,
            "type": "RLR"
        }

    def GetLRL(self, startPose: np.ndarray, goalPose: np.ndarray):
        r = self.car.turningRadius
        _, c1 = GetAdjacentCircles(startPose, self.car.turningRadius)
        _, c2 = GetAdjacentCircles(goalPose, self.car.turningRadius)
        c1Toc2 = c2[:2] - c1[:2]
        D = np.linalg.norm(c1Toc2)
        c1Toc2 = c1Toc2 / D
        midPt = c1[:2] + c1Toc2 * D / 2
        # Use definition of perpendicular for vectors (can make this negative to
        # get other tangent circle)
        vec2p3 = np.array([c2[1] - c1[1], -(c2[0] - c1[0])]) / D
        distMidPtToP3 = sqrt(4 * r**2 - D**2 / 4)
        # The center of the tangent circle
        p3 = midPt + vec2p3 * distMidPtToP3
        c3 = np.array([p3[0], p3[1], r])
        # Find the tangent pt between c1 and c3
        c1ToC3 = p3 - c1[:2]
        c1ToC3 = c1ToC3 / np.linalg.norm(c1ToC3)
        c1c3_tangent = c1[:2] + c1ToC3 * r

        # Find the tangent pt between c2 and c3
        c2ToC3 = p3 - c2[:2]
        c2ToC3 = c2ToC3 / np.linalg.norm(c2ToC3)
        c2c3_tangent = c2[:2] + c2ToC3 * r

        # First: Left Turn - from origin to first tangent pt
        firstLeftTurnDistance = CalcDirectionalArcLength(
            c1, startPose[:2], c1c3_tangent, Direction.LEFT)
        numTimeStepsForFirstLeftTurn = firstLeftTurnDistance / self.car.speed
        firstTurn = {
            "direction": Direction.LEFT.name,
            "distance": firstLeftTurnDistance,
            "numSteps": numTimeStepsForFirstLeftTurn
        }

        # Second: Right Turn - from first tangent pt to second tangent point
        secondRightTurnDistance = CalcDirectionalArcLength(
            c3, c1c3_tangent, c2c3_tangent, Direction.RIGHT)
        numTimeStepsForSecondRightTurn = secondRightTurnDistance / self.car.speed
        secondTurn = {
            "direction": Direction.RIGHT.name,
            "distance": secondRightTurnDistance,
            "numSteps": numTimeStepsForSecondRightTurn
        }

        # Third: Left Turn - from second tangent pt to goal pose
        thirdLeftTurnDistance = CalcDirectionalArcLength(
            c2, c2c3_tangent, goalPose[:2], Direction.LEFT)
        numTimeStepsForThirdLeftTurn = thirdLeftTurnDistance / self.car.speed
        thirdTurn = {
            "direction": Direction.LEFT.name,
            "distance": thirdLeftTurnDistance,
            "numSteps": numTimeStepsForThirdLeftTurn
        }

        waypoints = [startPose[:2], c1c3_tangent, c2c3_tangent, goalPose[:2]]
        # TODO: Delete this visualization code
        # circle1 = plt.Circle(c1[:2], r, color="blue", alpha=0.2)
        # circle2 = plt.Circle(c2[:2], r, color="red", alpha=0.2)
        # circle3 = plt.Circle(p3, r, color="black", alpha=0.2)

        # fig, ax = plt.subplots()
        # ax.add_patch(circle1)
        # ax.add_patch(circle2)
        # ax.add_patch(circle3)
        # ax.arrow(startPose[0],
        #          startPose[1],
        #          10 * cos(startPose[2]),
        #          10 * sin(startPose[2]),
        #          head_width=2)
        # ax.arrow(goalPose[0],
        #          goalPose[1],
        #          10 * cos(goalPose[2]),
        #          10 * sin(goalPose[2]),
        #          head_width=2)
        # plt.scatter(midPt[0], midPt[1])
        # plt.scatter(c1c3_tangent[0], c1c3_tangent[1])
        # plt.scatter(c2c3_tangent[0], c2c3_tangent[1])
        # ax.set_aspect("equal", adjustable='datalim')
        # plt.show()

        return {
            "path": [firstTurn, secondTurn, thirdTurn],
            "waypoints": waypoints,
            "totalDistance": firstLeftTurnDistance + secondRightTurnDistance +
            thirdLeftTurnDistance,
            "type": "LRL"
        }

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
        # Pick the "Right" adjacent Circles as on the RSR path they will be the
        # circles the car drives on
        c_start_right, _ = GetAdjacentCircles(startPose, r)
        c_goal_right, _ = GetAdjacentCircles(goalPose, r)

        c_start_t, c_goal_t = PickTangentLine(startPose, goalPose, r,
                                              Direction.RIGHT, Direction.RIGHT)

        # Turn Right from the original pose to the first tangent point
        firstRightTurnDistance = CalcDirectionalArcLength(
            c_start_right, startPose[:2], c_start_t, Direction.RIGHT)
        numTimeStepsForFirstRightTurn = firstRightTurnDistance / self.car.speed
        firstTurn = {
            "direction": Direction.RIGHT.name,
            "distance": firstRightTurnDistance,
            "numSteps": numTimeStepsForFirstRightTurn
        }

        # Turn right from the second tangent point to the final pose
        secondRightTurnDistance = CalcDirectionalArcLength(
            c_goal_right, c_goal_t, goalPose[:2], Direction.RIGHT)
        numTimeStepsForSecondrightTurn = secondRightTurnDistance / self.car.speed
        secondTurn = {
            "direction": Direction.RIGHT.name,
            "distance": secondRightTurnDistance,
            "numSteps": numTimeStepsForSecondrightTurn
        }

        # Go straight between the two circles
        straightLineDistance = np.linalg.norm(c_goal_right - c_start_right)
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

        c_start_t, c_goal_t = PickTangentLine(startPose, goalPose, r,
                                              Direction.LEFT, Direction.LEFT)
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
