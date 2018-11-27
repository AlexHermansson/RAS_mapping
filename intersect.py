from __future__ import print_function
from __future__ import division

import numpy as np
import matplotlib.pyplot as plt


class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return "(%f, %f)" % (self.x, self.y)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def dist(self, other):
        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    def mult(self, real_number):
        return Point(self.x * real_number, self.y * real_number)

    def cross(self, other):
        return self.x * other.y - self.y * other.x

    def plot(self):
        plt.plot(self.x, self.y, 'ro')


class Line:

    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def __repr__(self):
        return "%s --- %s" % (self.p1, self.p2)

    def plot(self):
        x = [self.p1.x, self.p2.x]
        y = [self.p1.y, self.p2.y]
        plt.plot(x, y, 'b')

    def intersects(self, other):
        # First line is parameterized: p + t * r for t in [0, 1]
        p = self.p1
        r = self.p2 - self.p1

        # First line is parameterized: q + u * s for u in [0, 1]
        q = other.p1
        s = other.p2 - other.p1

        qp = q - p
        rs = r.cross(s)
        if rs == 0:
            return False, None

        t = qp.cross(s) / rs
        u = qp.cross(r) / rs

        if 0 <= t <= 1 and 0 <= u <= 1:
            intersection_point = p + r.mult(t)
            return True, intersection_point

        return False, None


class Pose:

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __repr__(self):
        return "(%f, %f, %f)" % (self.x, self.y, self.theta)

    def plot(self):
        r = 0.1
        dx = r * np.cos(self.theta)
        dy = r * np.sin(self.theta)
        plt.arrow(self.x, self.y, dx, dy, head_width=0.03)


class Intersections:

    def __init__(self):
        self.walls = []
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        self.read_map()

    def show_walls(self):
        for wall in self.walls:
            wall.plot()

    def find_intersections(self, pose, num_angles=10, plot=False):
        pose_point = Point(pose.x, pose.y)
        angles = np.linspace(0, 2 * np.pi, num_angles, endpoint=False)
        distances = []
        intersections = []
        for angle in angles:
            extended_line = self.extend_line(pose, angle)
            points = []
            for wall in self.walls:
                intersects, point = extended_line.intersects(wall)
                if intersects:
                    points.append((point, point.dist(pose_point)))

            closest_point, dist = min(points, key=lambda p: p[1])
            intersections.append(closest_point)
            distances.append(dist)

        if plot:
            [p.plot() for p in intersections]

        return distances

    def extend_line(self, pose, dtheta):
        line_range = 100
        p1 = Point(pose.x, pose.y)
        x2 = pose.x + line_range * np.cos(pose.theta + dtheta)
        y2 = pose.y + line_range * np.sin(pose.theta + dtheta)  # todo: plus pi in there?
        p2 = Point(x2, y2)
        return Line(p1, p2)

    def read_map(self):
        min_x = 100
        max_x = -100
        min_y = 100
        max_y = -100

        with open("maze.txt") as f:
            lines = f.readlines()
            for line in lines:
                coords = line.split()
                x1, y1, x2, y2 = float(coords[0]), float(coords[1]), float(coords[2]), float(coords[3])
                p1 = Point(x1, y1)
                p2 = Point(x2, y2)
                self.walls.append(Line(p1, p2))

                if x1 < min_x:
                    min_x = x1
                if x2 < min_x:
                    min_x = x2
                if x1 > max_x:
                    max_x = x1
                if x2 > max_x:
                    max_x = x2

                if y1 < min_y:
                    min_y = y1
                if y2 < min_y:
                    min_y = y2
                if y1 > max_y:
                    max_y = y1
                if y2 > max_y:
                    max_y = y2

        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y


if __name__ == "__main__":

    intersections = Intersections()
    intersections.show_walls()
    pose = Pose(0.225, 0.225, np.pi/2)
    pose.plot()
    distances = intersections.find_intersections(pose, 30)
    print(distances)

    plt.show()