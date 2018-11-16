
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

    def __init__(self, p1=None, p2=None):

        if p1 is None and p2 is None:
            self.k = 0
            self.m = 0
        else:
            try:
                self.k = abs((p2.y - p1.y) / (p2.x - p1.x))
            except Warning:
                a = 0
            self.m = p2.y - self.k * p2.x

        self.inliers = []

    def __repr__(self):
        return "k: %s, m: %s" % (self.k, self.m)

    def _dist_from_line(self, point):
        return abs(self.k * point.x - point.y + self.m) / np.sqrt(self.k ** 2 + 1)

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

    def __init__(self, num_angles):
        self.num_angles = num_angles
        self.measurements = []
        self.pose = Pose(0.225, 0.225, np.pi / 2)
        self.read_measurements()

    def transform(self):

        measurement_positions = []
        angles = np.linspace(np.pi, 3 * np.pi, self.num_angles, endpoint=False)

        for a, m in zip(angles, self.measurements):
            if np.isinf(m):
                continue
            x = self.pose.x + m * np.cos(self.pose.theta + a)
            y = self.pose.x + m * np.sin(self.pose.theta + a)
            measurement_positions.append(Point(x, y))

        return measurement_positions

    def read_measurements(self):
        with open("meas.txt") as f:
            readings = f.readlines()[0]
            m = readings.split(", ")

        for i in range(num_angles):
            index = i * int(360 / num_angles)
            measurement = float(m[index])
            if index == 177:
                a = 0
            self.measurements.append(measurement)

    def plot(self):
        meas_pos = self.transform()
        [p.plot() for p in meas_pos]
        plt.axis([0, 3, 0, 3])
        plt.show()


class Mapper:

    def __init__(self, inlier_threshold = 0.1):
        self.inlier_threshold = inlier_threshold

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
                line.find_inliers(points, self.inlier_threshold)
                if len(line.inliers) > len(best_line.inliers):
                    best_line = line

        return best_line

    def seqential_RANSAC(self, points):
        pass


if __name__ == "__main__":

    num_angles = 360
    threshold = 0.05

    pose = Pose(0.225, 0.225, np.pi / 2)
    meas = Measurements(num_angles)
    meas.plot()

    points = meas.transform()
    mapper = Mapper(threshold)

    line = mapper.RANSAC(points)
    line.plot_inliers()

    plt.axis([0, 3, 0, 3])
    plt.show()