import numpy as np
import matplotlib.pyplot as plt

from intersect import Intersections


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

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def dist(self, other):
        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    def norm(self):
        return self.dist(Point(0, 0))

    def mult(self, real_number):
        return Point(self.x * real_number, self.y * real_number)

    def cross(self, other):
        return self.x * other.y - self.y * other.x

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def plot(self, cm='ro'):
        plt.plot(self.x, self.y, cm, markersize=2)


class Segment:

    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def __repr__(self):
        return "(%s, %s)" % (self.p1, self.p2)

    def dist_from_segment(self, point):
        t_hat = (point - self.p1).dot(self.p2 - self.p1) / ((self.p2 - self.p1).norm() ** 2)
        t_star = min(max(t_hat, 0), 1)
        dist = (self.p1 + (self.p2 - self.p1).mult(t_star) - point).norm()
        return dist


class Line:

    def __init__(self, p1=None, p2=None):

        if p1 is None and p2 is None:
            self.theta = None
            self.p = None
        else:
            if p1.norm() < p2.norm():
                self.p = p1
                dp = p2 - p1
            else:
                self.p = p2
                dp = p1 - p2
            self.theta = np.arctan2(dp.y, dp.x)

        self.inliers = []

    def __repr__(self):
        return "k: %s, m: %s" % (self.k, self.m)

    def _dist_from_line(self, point):
        q_moved = point - self.p
        theta_q = np.arctan2(q_moved.y, q_moved.x)
        if theta_q < 0:
            theta_q += 2 * np.pi
        dist = abs(q_moved.norm() * np.sin(theta_q - self.theta))
        return dist

    def find_inliers(self, points, inlier_threshold):
        for point in points:
            dist = self._dist_from_line(point)
            if dist < inlier_threshold:
                self.inliers.append(point)

    def plot_inliers(self):
        [point.plot() for point in self.inliers]


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


class Measurements:

    def __init__(self):
        self.intersector = Intersections()
        self.num_angles = 360
        self.range_treshold = 0.15
        self.poses = []
        self.walls = []
        self.measurements = []
        self.interesting_points = []
        self.interesting_points2 = []
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        self._read_map()
        self._read_measurements()

    def get_interesting_points(self):

        measurement_positions = []
        angles = np.linspace(np.pi, 3 * np.pi, self.num_angles, endpoint=False)

        for pose, measurements in zip(self.poses, self.measurements):

            for dtheta, m in zip(angles, measurements):
                if np.isinf(m):
                    continue
                x = pose.x + m * np.cos(pose.theta + dtheta)
                y = pose.y + m * np.sin(pose.theta + dtheta)
                point = Point(x, y)
                min_dist = 100
                for wall in self.walls:
                    d = wall.dist_from_segment(point)
                    if d < min_dist:
                        min_dist = d

                measurement_positions.append(point)
                if min_dist > self.range_treshold:
                    if self.max_x > point.x > self.min_x and self.max_y > point.y > self.min_y:
                        self.interesting_points2.append(point)

        return measurement_positions

    def transform(self):

        measurement_positions = []
        angles = np.linspace(np.pi, 3 * np.pi, self.num_angles, endpoint=False)

        for pose, measurements in zip(self.poses, self.measurements):

            modified_pose = Pose(pose.x, pose.y, pose.theta + np.pi)
            intersection_distances = self.intersector.find_intersections(modified_pose, num_angles=360)
            for dtheta, m, r in zip(angles, measurements, intersection_distances):
                if np.isinf(m):
                    continue
                x = pose.x + m * np.cos(pose.theta + dtheta)
                y = pose.y + m * np.sin(pose.theta + dtheta)
                point = Point(x, y)
                range_difference = abs(m - r)
                measurement_positions.append(point)
                if range_difference > self.range_treshold:
                    if self.intersector.max_x > point.x > self.intersector.min_x and\
                            self.intersector.max_y > point.y > self.intersector.min_y:
                        self.interesting_points.append(point)

        return measurement_positions

    def _read_measurements(self):
        with open('measurements.txt') as f:
            lines = f.readlines()
            for line in lines:
                coords = [float(c) for c in line.split(')')[0][1:].split(',')]
                ranges = [float(r) for r in line.split(')')[1][3:].split(',')]
                self.poses.append(Pose(*coords))
                self.measurements.append(ranges)

    def plot(self):
        plt.axis([-0.5, 3, -0.5, 3])
        self.transform()
        self.get_interesting_points()

        self.intersector.show_walls()
        # [p.plot() for p in meas_pos[::10]]
        [p.plot('bo') for p in self.interesting_points[::1]]
        plt.show()

        self.intersector.show_walls()
        [p.plot('ro') for p in self.interesting_points2[::1]]
        plt.show()

    def _read_map(self):
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
                self.walls.append(Segment(p1, p2))

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


class Mapper:
    """ Have points_threshold > num_inlier_threshold """

    def __init__(self, inlier_distance_threshold=0.1, points_treshold=200, num_inlier_threshold=100):
        self.inlier_distance_threshold = inlier_distance_threshold
        self.num_inlier_threshold = num_inlier_threshold
        self.points_treshold = points_treshold
        self.interesting_points = []

    def RANSAC(self, points):
        num_points = len(points)
        best_line = Line()
        for i in range(num_points):
            for j in range(num_points):
                p1 = points[i]
                p2 = points[j]
                if p1 == p2:
                    continue

                line = Line(p1, p2)
                line.find_inliers(points, self.inlier_distance_threshold)
                if len(line.inliers) > len(best_line.inliers):
                    best_line = line

        return best_line

    def sequential_RANSAC(self, points):
        walls = []
        points_left = points
        count = 0
        while len(points_left) > self.points_treshold:  # todo: Maybe simply a while True/len > 0:  ?
            best_wall = self.RANSAC(points_left)  # todo: Can we use Segments instead of Lines?
            if len(best_wall.inliers) < self.num_inlier_threshold:
                break

            count += 1
            print("Line: %s" % count)
            print("wall: %s" % best_wall)

            walls.append(best_wall)

            #REMOVE FROM POINTS_LEFT BEST_WALL.INLIERS BUT KEEPING THE ORDER
            outliers = []
            for point in points_left:
                found = False
                for inlier in best_wall.inliers:
                    if point == inlier:
                        found = True

                if not found:
                    outliers.append(point)

            points_left = outliers

        return walls


if __name__ == "__main__":

    meas = Measurements()
    meas.plot()

    meas.get_interesting_points()
    points = meas.interesting_points2

    mapper = Mapper(inlier_distance_threshold=0.1)
    mapper.sequential_RANSAC(points)

    #
    # line = mapper.RANSAC(points)
    # line.plot_inliers()
    #
    # plt.axis([0, 3, 0, 3])
    # plt.show()