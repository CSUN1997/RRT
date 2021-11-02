import numpy as np


def boundary_check(robot, point):
    for pnt in robot:
        x, y = pnt[0] + point[0], pnt[1] + point[1]
        if x < 0 or x > 10 or y < 0 or y > 10:
            return True
    return False


def ccw(point1, point2, point3):
    return (point3[1] - point1[1]) * (point2[0] - point1[0]) > (point2[1] - point1[1]) * (point3[0] - point1[0])


# Return true if line segments AB and CD intersect
def intersect(point1, point2, point3, point4):
    return ccw(point1, point3, point4) != ccw(point2, point3, point4) and \
           ccw(point1, point2, point3) != ccw(point1, point2, point4)


def isCollisionFree(robot, point, obstacles):
    if boundary_check(robot, point):
        return False
    robot_pos = [[p[0] + point[0], p[1] + point[1]] for p in robot]
    robot_pos.append(robot_pos[0])
    for obstacle in obstacles:
        pos = obstacle.copy()
        pos.append(pos[0])
        for i in range(len(pos) - 1):
            for j in range(len(robot_pos) - 1):
                if intersect(pos[i], pos[i + 1], robot_pos[j], robot_pos[j + 1]):
                    return False
    return True


def isCollisionFreePnt(point, configuration):
    x, y = point
    m, n = int(np.floor(x / 0.1)), int(np.floor(y / 0.1))
    if configuration[m, n] == 0:
        return True
    return False


def isCollisionFree2(robot, obstacles, point1, point2, configuration_grid):
    if not isCollisionFreePnt(point1, configuration_grid) or not isCollisionFreePnt(point2, configuration_grid):
        return False
    for obstacle in obstacles:
        pos = obstacle.copy()
        pos.append(pos[0])
        for i in range(len(pos) - 1):
            if intersect(pos[i], pos[i + 1], point1, point2):
                return False
    return True

