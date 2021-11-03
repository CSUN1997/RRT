import numpy as np
from collision import isCollisionFree, isCollisionFree2
from utils import get_grid

class Node:
    def __init__(self, p):
        self.p = p
        self.parent = None
        # for rrt*, records the distance between current node and the parent
        self.cost = None
        self.child = []


class RRT:

    def __init__(self, robot, obstacles, start, goal):
        self.robot = robot
        self.obstacles = obstacles
        self.start = Node(start)
        self.goal = Node(goal)
        self.configuration = get_grid(robot, obstacles)
        # use a dictionary to keep the nodes for convenient search
        self.node_list = {start[0]: [self.start]}

    def add(self, point1, point2):
        # find node 1
        node1 = None
        for node in self.node_list[point1[0]]:
            if node.p[1] == point1[1]:
                node1 = node
        new_node = Node(point2)
        node1.child.append(new_node)
        new_node.parent = node1
        print("new", new_node.p, "parent", new_node.parent.p)
        if point2[0] in self.node_list:
            self.node_list[point2[0]].append(new_node)
        else:
            self.node_list[point2[0]] = [new_node]

    def exists(self, point):
        x, y = point
        if x in self.node_list:
            for pnt in self.node_list[x]:
                if pnt.p[1] == y:
                    return True
        return False

    def parent(self, point):
        x, y = point
        for node in self.node_list[x]:
            if y == node.p[1]:
                return node.parent.p
        return None

    def nearest(self, point):
        x, y = point
        closest_dist = 999
        closest_node = None
        for x_coord in self.node_list:
            for node in self.node_list[x_coord]:
                dist = (x - x_coord) ** 2 + (y - node.p[1]) ** 2
                if dist < closest_dist:
                    closest_dist = dist
                    closest_node = node
        return closest_node.p

    def extend(self, point1, point2):
        if isCollisionFree2(self.robot, self.obstacles, point1, point2, self.configuration):
            self.add(point1, point2)
            return point2
        x1, y1 = point1
        x2, y2 = point2
        dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        n_steps = int(np.floor(dist / 0.1))
        angle = np.arctan2(y2 - y1, x2 - x1)
        for i in range(1, n_steps + 1):
            next_x = round(x1 + 0.1 * i * np.cos(angle), 2)
            next_y = round(y1 + 0.1 * i * np.sin(angle), 2)
            if isCollisionFree2(self.robot, self.obstacles, point1, [next_x, next_y], self.configuration):
                continue
            last_x = round(x1 + 0.1 * (i - 1) * np.cos(angle), 2)
            last_y = round(y1 + 0.1 * (i - 1) * np.sin(angle), 2)
            self.add(point1, [last_x, last_y])
            return [last_x, last_y]

        next_x = round(x1 + 0.1 * n_steps * np.cos(angle), 2)
        next_y = round(y1 + 0.1 * n_steps * np.sin(angle), 2)
        self.add(point1, [next_x, next_y])
        return [next_x, next_y]

