import matplotlib.pyplot as plt
from utils import cn_PnPoly, discretize, minkowski_sum
from rrt import rrt
import numpy as np


def plot_polygon(coords, color):
    pos = coords.copy()
    pos.append(pos[0])
    xs, ys = zip(*pos)
    plt.plot(xs, ys, color=color)


def visualize_problem(robot, obstacles, start, goal):
    plot_polygon(robot, color='blue')
    for obstacle in obstacles:
        plot_polygon(obstacle, color='black')
    plt.scatter(start[0], start[1], color='red')
    plt.scatter(goal[0], goal[1], color='green')
    plt.show()


def visualize_points(points, robot, obstacles, start, goal):
    for pnt in points:
        plt.scatter(pnt[0], pnt[1], color='yellow')
    visualize_problem(robot, obstacles, start, goal)


def visualize_configuration(robot, obstacles, start, goal):
    grid = np.zeros([101, 101])
    robot_pnts, obstalce_pnts = discretize(robot, obstacles)
    extended_obstacles = minkowski_sum(robot_pnts, obstalce_pnts)
    for coord in extended_obstacles:
        x = max(0, min(100, coord[0]))
        y = max(0, min(100, coord[1]))
        grid[100 - y, x] = 1
    plt.figure(figsize=(5, 5))
    plt.imshow(grid, cmap='Greys')
    plt.scatter(start[0] * 10, 100 - start[1] * 10, color='red')
    plt.scatter(goal[0] * 10, 100 - goal[1] * 10, color='green')
    plt.xticks([0, 100], ['0', '10'])
    plt.yticks([0, 100], ['10', '0'])
    plt.xlabel('x')
    plt.ylabel('y')
    # plt.show()


def visualize_path(robot, obstacles, path):
    for i in range(len(path) - 1):
        xs, ys = zip(*path)
        plt.plot(xs, ys, color='blue')


def visualize_rrt(tree, robot, obstacles, start, goal):
    visualize_configuration(robot, obstacles, start, goal)
    for x in tree.node_list:
        for node in tree.node_list[x]:
            if node.p[0] == start[0] and node.p[1] == start[1]:
                continue
            parent = tree.parent([node.p[0], node.p[1]])
            parent = [10 * parent[0], 100 - 10 * parent[1]]
            cur_pos = [10 * node.p[0], 100 - 10 * node.p[1]]
            # pos = [parent, cur_pos]
            x = [parent[0], cur_pos[0]]
            y = [parent[1], cur_pos[1]]
            plt.plot(x, y, color='green')
    plt.show()


if __name__ == '__main__':
    robot = [(0, 0), (0.5, 0.5), (1, 0)]
    obstacles = [[(8, 6), (4, 7), (9, 9)], [(1, 1), (1, 5), (5, 5), (5, 1), (3, 0.5)]]
    start, goal = (0.5, 0.5), (7, 9)
    visualize_configuration(robot, obstacles, start, goal)
    path, tree = rrt(robot, obstacles, start, goal, 1000)
    visualize_rrt(tree, robot, obstacles, start, goal)

