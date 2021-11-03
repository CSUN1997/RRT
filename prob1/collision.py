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


def intersect_grid(configuration, point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    m1, n1 = int(x1 // 0.1), int(y1 // 0.1)
    m2, n2 = int(x2 // 0.1), int(y2 // 0.1)
    dx = x2 - x1
    dy = y2 - y1

    counter = 0
    for m in range(m1, m2):
        x = x1 + counter * 0.1
        y = y1 + dy * (x - x1) / dx
        n = int(y // 0.1)
        if configuration[m, n] != 0:
            return True
        counter += 1
    return False



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
            # if intersect_grid(configuration_grid, point1, point2):
                return False
    return True

