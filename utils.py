import numpy as np


# Crossing number method: http://www.dgp.toronto.edu/~mac/e-stuff/point_in_polygon.py
def cn_PnPoly(P, V):
    cn = 0  # the crossing number counter

    # repeat the first vertex at end
    V = tuple(V[:]) + (V[0],)

    # loop through all edges of the polygon
    for i in range(len(V) - 1):  # edge from V[i] to V[i+1]
        if ((V[i][1] <= P[1] < V[i + 1][1])  # an upward crossing
                or (V[i][1] > P[1] >= V[i + 1][1])):  # a downward crossing
            # compute the actual edge-ray intersect x-coordinate
            vt = (P[1] - V[i][1]) / float(V[i + 1][1] - V[i][1])
            if P[0] < V[i][0] + vt * (V[i + 1][0] - V[i][0]):  # P[0] < intersect
                cn += 1  # a valid crossing of y=P[1] right of P[0]

    return cn % 2  # 0 if even (out), and 1 if odd (in)


def discretize(robot, obstacles):
    obstacle_points = []
    robot_points = []
    # objects = [robot] + obstacles
    for i in range(101):
        for j in range(101):
            x = i * 0.1
            y = j * 0.1
            for obstacle in obstacles:
                if cn_PnPoly((x, y), obstacle):
                    obstacle_points.append([i, j])
                if cn_PnPoly((x, y), robot):
                    robot_points.append([i, j])
    return robot_points, obstacle_points


def minkowski_sum(pnts1, pnts2):
    results = []
    for pnt1 in pnts1:
        for pnt2 in pnts2:
            results.append([pnt1[0] + pnt2[0], pnt1[1] + pnt2[1]])
    return results


def get_grid(robot, obstacles):
    grid = np.zeros([101, 101])
    robot_pnts, obstalce_pnts = discretize(robot, obstacles)
    extended_obstacles = minkowski_sum(robot_pnts, obstalce_pnts)
    for coord in extended_obstacles:
        x = max(0, min(100, coord[0]))
        y = max(0, min(100, coord[1]))
        grid[x, y] = 1
    return grid