from tree import RRT
from sampler import sample
import numpy as np
from collision import isCollisionFreePnt


def find_path(tree, start, goal):
    path = [goal]
    point = goal
    while point[0] != start[0] or point[1] != start[1]:
        print('path', point)
        point = tree.parent(point)
        path.append(point)
    return path


def rrt(robot, obstacles, start, goal, iter_n):
    tree = RRT(robot, obstacles, start, goal)
    for i in range(iter_n):
        random_point = sample()
        if not isCollisionFreePnt(random_point, tree.configuration):
            continue
        nearest = tree.nearest(random_point)
        random_point = tree.extend(nearest, random_point)
        # print(random_point)
        dist2goal = np.sqrt((random_point[0] - goal[0]) ** 2 + (random_point[1] - goal[1]) ** 2)
        if dist2goal <= 0.1:
            goal_atempt = tree.extend(random_point, goal)
            if goal_atempt[0] == goal[0] and goal_atempt[1] == goal[1]:
                print('Success')
                return find_path(tree, start, goal), tree
    print('Failed')
    return None, tree

