import numpy as np
from math import sin, cos, sqrt
import matplotlib.pyplot as plt
from src.controls.dubins_path import DubinsPath
from src.dynamics.dubins_car import DubinsCar
from src.controls.dubins_path_utils import Direction, GetAdjacentCircles, GetInnerTangentPointsAndLines, GetOuterTangentPointsAndLines, CalcDirectionalArcLength, CalcDirectionArcAngle


def drawPose(pose, ax, color="black", l=10):
    x, y, theta = pose
    dx, dy = cos(theta), sin(theta)
    ax.arrow(x, y, l * dx, l * dy, head_width=1, color=color, alpha=0.8)


def drawCircle(c, ax, color="blue", text=None, label=None, ls="-"):
    if label is not None:
        circle = plt.Circle(c[:2], c[2], alpha=0.7, color=color, fill=False, ls=ls, lw=2, label=label)
    else:
        circle = plt.Circle(c[:2], c[2], alpha=0.7, color=color, fill=False, ls=ls, lw=2)

    ax.add_patch(circle)
    ax.set_aspect("equal", adjustable="datalim")
    if text:
        ax.text(c[0], c[1], text)


def drawTangentLineandPoints(line, ax):
    p1, p2 = line
    ax.scatter(p1[0], p1[1])
    ax.scatter(p2[0], p2[1])
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]])


def drawOuterTangentPointsandLines(circle1, circle2, ax):
    tangentPoints, tangentLines = GetOuterTangentPointsAndLines(
        circle1, circle2)
    for line in tangentLines:
        drawTangentLineandPoints(line, ax)


def drawInnerTangentPointsandLines(circle1, circle2, ax):
    tangentPoints, tangentLines = GetInnerTangentPointsAndLines(
        circle1, circle2)
    for line in tangentLines:
        drawTangentLineandPoints(line, ax)


def drawPoseWithOuterTangents(startPose, goalPose, circle1, circle2, ax):
    drawPose(startPose, ax, "blue")
    drawPose(goalPose, ax, "green")
    drawOuterTangentPointsandLines(circle1, circle2, ax)


def drawPoseWithInnerTangents(startPose, goalPose, circle1, circle2, ax):
    drawPose(startPose, ax, "blue")
    drawPose(goalPose, ax, "green")
    drawInnerTangentPointsandLines(circle1, circle2, ax)


def VisualizeAdjacentCircles(poses):
    fix, ax = plt.subplots()
    for i, pose in enumerate(poses):
        r = 10
        right_circle, left_circle = GetAdjacentCircles(pose, r)
        ls = "-" if i % 2 == 0 else "--"
        drawCircle(right_circle, ax, "blue",label=f"Pose {i+1} Right", ls=ls)
        drawCircle(left_circle, ax, "green",label=f"Pose {i+1} Left", ls=ls)
        ax.set_aspect("equal", adjustable="datalim")
        plt.scatter(left_circle[0],  left_circle[1], alpha=0.3)
        plt.scatter(right_circle[0], right_circle[1], alpha=0.3)
        color = "blue" if i % 2 == 0 else "green"
        drawPose(pose, ax, color)
    ax.legend()
    title = "Visualizing Adjacent Turning Circles"
    plt.title(title)
    plt.savefig(f"./plots/{title}_{np.random.randint(0,1000)}.png")
    plt.show()


def VisualizeOuterTangentPointsFor2Circles(startPose, goalPose):
    r = 10
    start_right_circle, start_left_circle = GetAdjacentCircles(startPose, r)
    goal_right_circle, goal_left_circle = GetAdjacentCircles(goalPose, r)
    fig, ax = plt.subplots()
    drawPoseWithOuterTangents(startPose, goalPose, start_left_circle, goal_left_circle, ax)
    drawCircle(start_left_circle, ax, color="blue", label="Start Left Circle")
    drawCircle(goal_left_circle, ax, color="green", label="Goal Left Circle")
    title = "Outer Tangent Points and Lines on One Candidate Path"
    plt.title(title)
    plt.savefig(f"./plots/{title}.png")
    ax.legend()
    plt.show()


def VisualizeAllOuterTangentPoints(startPose, goalPose):
    r = 10
    start_right_circle, start_left_circle = GetAdjacentCircles(startPose, r)
    goal_right_circle, goal_left_circle = GetAdjacentCircles(goalPose, r)

    circlePairs = [[start_left_circle, goal_left_circle],
                   [start_left_circle, goal_right_circle],
                   [start_right_circle, goal_left_circle],
                   [start_right_circle, goal_right_circle]]

    fig, ax = plt.subplots()
    # plt.show()
    for circle1, circle2 in circlePairs:
        drawPoseWithOuterTangents(startPose, goalPose, circle1, circle2, ax)

    drawCircle(start_right_circle, ax, color="blue",  label="Start Right Circle", ls="--")
    drawCircle(start_left_circle,  ax, color="black", label="Start Left Circle", ls="--")
    drawCircle(goal_right_circle,  ax, color="blue",  label="Goal Right Circle", ls=":")
    drawCircle(goal_left_circle,   ax, color="black", label="Goal Left Circle",  ls=":")
    ax.legend()
    title = "All Outer Tangent Points and Lines"
    plt.title(title)
    plt.savefig(f"./plots/{title}.png")
    plt.show()
    


def VisualizeInnerTangentPointsFor2Circles(startPose, goalPose):
    r = 10
    _, start_left_circle = GetAdjacentCircles(startPose, r)
    goal_right_circle, _ = GetAdjacentCircles(goalPose, r)
    fig, ax = plt.subplots()
    
    drawPoseWithInnerTangents(startPose, goalPose,start_left_circle, goal_right_circle, ax)
    drawCircle(start_left_circle, ax, color="blue", label="Start Left Circle", ls="--")
    drawCircle(goal_right_circle, ax, color="green", label="Goal Right Circle", ls="--")
    ax.legend()
    title = "Inner Tangent Points and Lines on One Candidate Path"
    plt.title(title)
    plt.savefig(f"./plots/{title}.png")
    plt.show()


def VisualizeAllInnerTangentPoints(startPose, goalPose):
    r = 10
    start_right_circle, start_left_circle = GetAdjacentCircles(startPose, r)
    goal_right_circle, goal_left_circle = GetAdjacentCircles(goalPose, r)

    circlePairs = [[start_left_circle, goal_left_circle],
                   [start_left_circle, goal_right_circle],
                   [start_right_circle, goal_left_circle],
                   [start_right_circle, goal_right_circle]]
    fig, ax = plt.subplots()
    for circle1, circle2 in circlePairs:
        drawPoseWithInnerTangents(startPose, goalPose, circle1, circle2, ax)
    
    drawCircle(start_right_circle, ax, color="blue", label="Start Right Circle", ls="--")
    drawCircle(goal_left_circle, ax, color="blue", label="Goal Left Circle", ls=":")
    
    drawCircle(start_left_circle, ax, color="green", label="Start Left Circle", ls="--")
    drawCircle(goal_right_circle, ax, color="green", label="Goal Right Circle", ls=":")
    ax.legend()
    title = "All Inner Tangent Points and Lines Between a Start and Goal Pose"
    plt.title(title)
    plt.savefig(f"./plots/{title}.png")
    plt.show()

def VisualizeRLRTangentCircle(startPose, goalPose):
    r = 10
    c1, _ = GetAdjacentCircles(startPose, r)
    c2, _ = GetAdjacentCircles(goalPose, r)
    c1Toc2 = c2[:2] - c1[:2]
    D = np.linalg.norm(c1Toc2)
    # normalize c1Toc2
    c1Toc2 = c1Toc2 / D
    midPt = c1[:2] + c1Toc2 * D / 2
    # Use the definition of perpendicular to get direction to center of c3
    vec2p3 = np.array([c2[1] - c1[1], -(c2[0] - c1[0])]) / D
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

    thetaAtFirstTangent = CalcDirectionArcAngle(c1, startPose[:2],
                                                c1c3_tangent, Direction.RIGHT)

    topOfTangentCircle = p3 + np.array([0, r])
    thetaAtTopOfCircle = CalcDirectionArcAngle(c3, c1c3_tangent,
                                               topOfTangentCircle,
                                               Direction.LEFT)

    thetaAtSecondTangentPoint = CalcDirectionArcAngle(c3, c1c3_tangent[:2],
                                                      c2c3_tangent,
                                                      Direction.LEFT)

    fig, ax = plt.subplots()
    drawCircle(c1, ax, color="blue", ls="--", label="Start Right Circle")
    drawCircle(c3, ax, color="green", ls=":", label="Tangent Circle (Left)")
    drawCircle(c2, ax, color="blue", ls="-", label="Goal Right Circle")
    drawPose(startPose, ax, "blue")
    drawPose(goalPose, ax, "green")
    drawPose([*c1c3_tangent[:2], startPose[2] + thetaAtFirstTangent], ax)
    drawPose([
        *topOfTangentCircle[:2],
        startPose[2] + thetaAtFirstTangent + thetaAtTopOfCircle
    ], ax)
    drawPose([
        *c2c3_tangent[:2],
        startPose[2] + thetaAtFirstTangent + thetaAtSecondTangentPoint
    ], ax)
    ax.scatter(c1c3_tangent[0], c1c3_tangent[1])
    ax.scatter(c2c3_tangent[0], c2c3_tangent[1])
    ax.scatter(topOfTangentCircle[0], topOfTangentCircle[1])
    ax.legend()
    title = "RLR Tangent Circle Paths"
    plt.title(title)
    plt.savefig(f"./plots/{title}.png")
    plt.show()


def VisualizeAllTangentCircles(startPose, goalPose):
    r = 10
    c1_right, c1_left = GetAdjacentCircles(startPose, r)
    c2_right, c2_left = GetAdjacentCircles(goalPose, r)
    c1Toc2_right = c2_right[:2] - c1_right[:2]
    c1Toc2_left = c2_left[:2] - c1_left[:2]
    D_right = np.linalg.norm(c1Toc2_right)
    D_left = np.linalg.norm(c1Toc2_left)
    # normalize c1Toc2
    c1Toc2_right = c1Toc2_right / D_right
    c1Toc2_left = c1Toc2_left / D_left
    midPt_right = c1_right[:2] + c1Toc2_right * D_right / 2
    midPt_left = c1_left[:2] + c1Toc2_left * D_left / 2
    # Use the definition of perpendicular to get direction to center of c3
    vec2p3_right_1 = np.array(
        [c2_right[1] - c1_right[1], -(c2_right[0] - c1_right[0])]) / D_right
    vec2p3_right_2 = -np.array(
        [c2_right[1] - c1_right[1], -(c2_right[0] - c1_right[0])]) / D_right
    vec2p3_left_1 = np.array(
        [c2_left[1] - c1_left[1], -(c2_left[0] - c1_left[0])]) / D_left
    vec2p3_left_2 = -np.array(
        [c2_left[1] - c1_left[1], -(c2_left[0] - c1_left[0])]) / D_left

    # From geometry
    distMidPtToP3_right = sqrt(4 * r**2 - D_right**2 / 4)
    distMidPtToP3_left = sqrt(4 * r**2 - D_left**2 / 4)
    # The center of the tangent circle
    p3_right_1 = midPt_right + vec2p3_right_1 * distMidPtToP3_right
    p3_right_2 = midPt_right + vec2p3_right_2 * distMidPtToP3_right

    p3_left_1 = midPt_left + vec2p3_left_1 * distMidPtToP3_left
    p3_left_2 = midPt_left + vec2p3_left_2 * distMidPtToP3_left

    c3_right_1 = np.array([p3_right_1[0], p3_right_1[1], r])
    c3_right_2 = np.array([p3_right_2[0], p3_right_2[1], r])

    c3_left_1 = np.array([p3_left_1[0], p3_left_1[1], r])
    c3_left_2 = np.array([p3_left_2[0], p3_left_2[1], r])

    # Find the tangent pt between c1 and c3
    # c1Toc3 = c3[:2] - c1[:2]
    # c1Toc3 = c1Toc3 / np.linalg.norm(c1Toc3)
    # c1c3_tangent = c1[:2] + c1Toc3 * r

    # # Find the tangent pt between c2 and c3
    # c2ToC3 = p3 - c2[:2]
    # c2ToC3 = c2ToC3 / np.linalg.norm(c2ToC3)
    # c2c3_tangent = c2[:2] + c2ToC3 * r
    fig, ax = plt.subplots()
    drawCircle(c1_right, ax, color="blue", label="Start Right", ls="--")
    drawCircle(c1_left , ax, color="green", label="Start Left", ls="--")
    drawCircle(c3_right_1, ax, color="orange", label = "RLR Tangent Option 1")
    drawCircle(c3_right_2, ax, color="orange", label = "RLR Tangent Option 2")
    drawCircle(c3_left_1, ax, color="red", label = "LRL Tangent Option 1")
    drawCircle(c3_left_2, ax, color="red", label = "LRL Tangent Option 2")
    drawCircle(c2_right , ax, color="blue", label="Goal Right", ls=":")
    drawCircle(c2_left , ax, color="green", label="Goal Left", ls=":")
    drawPose(startPose, ax, "blue")
    drawPose(goalPose, ax, "green")

    # ax.scatter(c1_right[0], c1_right[1])
    # ax.scatter(c1_left[0], c1_left[1])
    # ax.scatter(c2_left[0], c2_left[1])
    # ax.scatter(c2_right[0], c2_right[1])
    ax.legend()
    title = "All Potential CCC Curves for a given start and goal pose"
    plt.title(title)
    plt.savefig(f"./plots/{title}.png")
    plt.show()

def VisualizeBothTangentCirclesForAPairOfCircles(startPose, goalPose):
    r = 10
    c1_right, _ = GetAdjacentCircles(startPose, r)
    c2_right, _ = GetAdjacentCircles(goalPose, r)
    c1Toc2_right = c2_right[:2] - c1_right[:2]
    D_right = np.linalg.norm(c1Toc2_right)
    # normalize c1Toc2
    c1Toc2_right = c1Toc2_right / D_right
    midPt_right = c1_right[:2] + c1Toc2_right * D_right / 2
    # Use the definition of perpendicular to get direction to center of c3
    vec2p3_right_1 = np.array(
        [c2_right[1] - c1_right[1], -(c2_right[0] - c1_right[0])]) / D_right
    vec2p3_right_2 = -np.array(
        [c2_right[1] - c1_right[1], -(c2_right[0] - c1_right[0])]) / D_right

    # From geometry
    distMidPtToP3_right = sqrt(4 * r**2 - D_right**2 / 4)
    # The center of the tangent circle
    p3_right_1 = midPt_right + vec2p3_right_1 * distMidPtToP3_right
    p3_right_2 = midPt_right + vec2p3_right_2 * distMidPtToP3_right

    c3_right_1 = np.array([p3_right_1[0], p3_right_1[1], r])
    c3_right_2 = np.array([p3_right_2[0], p3_right_2[1], r])

    # Find the tangent pt between c1 and c3_right_1
    c1Toc3_1 = c3_right_1[:2] - c1_right[:2]
    c1Toc3_1 = c1Toc3_1 / np.linalg.norm(c1Toc3_1)
    c1c3_1_tangent = c1_right[:2] + c1Toc3_1 * r

    # Find the tangent pt between c2 and c3_right_1
    c2Toc3_1 = c3_right_1[:2] - c2_right[:2]
    c2Toc3_1 = c2Toc3_1 / np.linalg.norm(c2Toc3_1)
    c2c3_1_tangent = c2_right[:2] + c2Toc3_1 * r

    # Find the tangent pt between c1 and c3_right_2
    c1Toc3_2 = c3_right_2[:2] - c1_right[:2]
    c1Toc3_2 = c1Toc3_2 / np.linalg.norm(c1Toc3_2)
    c1c3_2_tangent = c1_right[:2] + c1Toc3_2 * r

    # Find the tangent pt between c2 and c3_right_2
    c2Toc3_2 = c3_right_2[:2] - c2_right[:2]
    c2Toc3_2 = c2Toc3_2 / np.linalg.norm(c2Toc3_2)
    c2c3_2_tangent = c2_right[:2] + c2Toc3_2 * r


    fig, ax = plt.subplots()
    drawCircle(c1_right, ax, color="blue", label="Circle 1", ls="--")
    drawCircle(c3_right_1, ax, color="orange", label = "Tangent Option 1")
    drawCircle(c3_right_2, ax, color="red", label = "Tangent Option 2")
    drawCircle(c2_right , ax, color="blue", label="Circle 2", ls=":")
    drawPose(startPose, ax, "blue")
    drawPose(goalPose, ax, "green")

    ax.scatter(c1c3_1_tangent[0], c1c3_1_tangent[1])
    ax.scatter(c1c3_2_tangent[0], c1c3_2_tangent[1])
    ax.scatter(c2c3_1_tangent[0], c2c3_1_tangent[1])
    ax.scatter(c2c3_2_tangent[0], c2c3_2_tangent[1])
    # ax.scatter(c1_right[0], c1_right[1])
    # ax.scatter(c1_right[0], c1_right[1])
    # ax.scatter(c1_left[0], c1_left[1])
    # ax.scatter(c2_left[0], c2_left[1])
    # ax.scatter(c2_right[0], c2_right[1])
    ax.legend()
    title = "Both Tangent Circles for a Given Pair of Circles"
    plt.title(title)
    plt.savefig(f"./plots/{title}.png")
    plt.show()


def VisualizeRSR(startPose, goalPose):
    startPose = np.array(startPose)
    goalPose = np.array(goalPose)
    car = DubinsCar(5,0,0,0,0,1)
    path_generator = DubinsPath(car)
    start_right_circle, _ = GetAdjacentCircles(startPose, car.turningRadius)
    goal_right_circle, _  = GetAdjacentCircles(goalPose, car.turningRadius)
    output = path_generator.GetDubinsPath(startPose, goalPose)
    waypoints = output["waypoints"]
    fig, ax = plt.subplots()
    drawCircle(start_right_circle, ax, color="blue", label="Start Right Circle", ls="-")
    drawCircle(goal_right_circle, ax, color="green", label="Goal Right Circle", ls="--")
    ax.scatter(waypoints[1][0], waypoints[1][1])
    ax.scatter(waypoints[2][0], waypoints[2][1])
    ax.plot([waypoints[1][0], waypoints[2][0]], [waypoints[1][1], waypoints[2][1]])
    drawPose(startPose, ax, color="blue")
    drawPose(goalPose, ax, color="green")
    ax.legend()
    title = f"{output['type']} path from a Start Pose to a Goal Pose"
    plt.title(title)
    plt.savefig(f"./plots/{title}.png")
    plt.show()

def VisualizeRSL(startPose, goalPose):
    startPose = np.array(startPose)
    goalPose = np.array(goalPose)
    car = DubinsCar(5,0,0,0,0,1)
    path_generator = DubinsPath(car)
    start_right_circle, _ = GetAdjacentCircles(startPose, car.turningRadius)
    _, goal_left_circle  = GetAdjacentCircles(goalPose, car.turningRadius)
    output = path_generator.GetDubinsPath(startPose, goalPose)
    waypoints = output["waypoints"]
    fig, ax = plt.subplots()
    drawCircle(start_right_circle, ax, color="blue", label="Start Right Circle", ls="-")
    drawCircle(goal_left_circle, ax, color="green", label="Goal Left Circle", ls="--")
    ax.scatter(waypoints[1][0], waypoints[1][1])
    ax.scatter(waypoints[2][0], waypoints[2][1])
    ax.plot([waypoints[1][0], waypoints[2][0]], [waypoints[1][1], waypoints[2][1]])
    drawPose(startPose, ax, color="blue")
    drawPose(goalPose, ax, color="green")
    ax.legend()
    title = f"{output['type']} path from a Start Pose to a Goal Pose"
    plt.title(title)
    plt.savefig(f"./plots/{title}.png")
    plt.show()
    
if __name__ == "__main__":
    startPose = [0, 0, np.pi / 2]
    goalPose = [40, 0, np.pi / 2]
    VisualizeAdjacentCircles([startPose, goalPose])
    # VisualizeAdjacentCircles([[0,0,np.pi/2], [50,0,np.pi/2]])
    # VisualizeAllOuterTangentPoints(startPose, goalPose)
    # VisualizeOuterTangentPointsFor2Circles(startPose, goalPose)
    # VisualizeInnerTangentPointsFor2Circles(startPose, goalPose)
    # VisualizeAllInnerTangentPoints(startPose, goalPose)

    
    startPose = [0, 0, np.pi / 2]
    goalPose = [100, 0, -np.pi / 2]
    # VisualizeRSR(startPose, goalPose)
    startPose = [0, 0, np.pi / 2]
    goalPose = [100, 0, np.pi / 2]
    # VisualizeRSL(startPose, goalPose)
    # Visualize CCC paths
    startPose = [8, 5, np.pi / 2]
    goalPose = [0, 0, -np.pi / 2]
    # VisualizeAllTangentCircles(startPose, goalPose)
    # VisualizeRLRTangentCircle(startPose, goalPose)
    # VisualizeBothTangentCirclesForAPairOfCircles(startPose,goalPose)
