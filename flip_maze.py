#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt


def read_maze(path_to_map):
    points = []
    with open(path_to_map) as f:
        lines = f.readlines()
        for line in lines:
            if line[0] == '#':
                continue
            coords = line.split()
            x1, y1, x2, y2 = float(coords[0]), float(coords[1]), float(coords[2]), float(coords[3])
            points.append([x1, y1])
            points.append([x2, y2])

    return np.array(points)


def write_maze(points, path_to_write):

    n, _ = points.shape
    with open(path_to_write, 'w+') as f:
        for i in range(0, n, 2):
            x1, y1 = points[i]
            x2, y2 = points[i+1]
            s = '%.4f %.4f %.4f %.4f\n' % (x1, y1, x2, y2)
            f.write(s)


def plot_points(points):

    n, _ = points.shape

    for i in range(0, n ,2):
        x1 = points[i, 0]
        x2 = points[i+1, 0]
        y1 = points[i, 1]
        y2 = points[i+1, 1]
        plt.plot([x1, x2], [y1, y2])

    plt.show()


def translate(points, translation_vector):

    return points + translation_vector


def rotate(points, theta):

    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                [np.sin(theta), np.cos(theta)]])

    return points.dot(rotation_matrix.T)


def flip(points):
    return np.flip(points, 1)


def transform(points, to_flip=False):

    transl = -points[0, :]
    p1 = points[0, :]
    p2 = points[1, :]
    diff = p2 - p1
    angle = -np.arctan2(diff[1], diff[0])
    new_points = rotate(translate(points, transl), angle)
    if to_flip:
        new_points = flip(new_points)

    furthest_point = new_points[np.argmax(np.linalg.norm(points, axis=1))]
    theta = np.arctan2(furthest_point[1], furthest_point[0])
    if -np.pi/2 < theta < 0:
        new_points = rotate(new_points, np.pi / 2)

    elif -np.pi < theta < -np.pi/2:
        new_points = rotate(new_points, np.pi)

    elif np.pi > theta > np.pi / 2:
        new_points = rotate(new_points, -np.pi / 2)

    return new_points


if __name__ == '__main__':

    points = read_maze('stupid_maze.txt')
    plot_points(points)

    # Remember to check if the maps needs flipping or not!!!
    new_points = transform(points, to_flip=True)
    plot_points(new_points)

    new_path = 'fixed_maze.txt'

    write_maze(new_points, new_path)