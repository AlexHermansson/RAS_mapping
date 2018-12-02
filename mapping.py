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

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def dist(self, other):
        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    def norm(self):
        return self.dist(Point(0, 0))

    def mult(self, real_number):
        return Point(self.x * real_number, self.y * real_number)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def plot(self, cm='ro', mz=2):
        plt.plot(self.x, self.y, cm, markersize=mz)

    def number_of_neighbors(self, points, dist_threshold):
        neighbor_count = 0
        for point in points:
            dist = self.dist(point)
            if dist < dist_threshold:
                neighbor_count += 1

        return neighbor_count


class Segment:

    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        self.inliers = []

    def __repr__(self):
        return "(%s, %s)" % (self.p1, self.p2)

    def dist_from_segment(self, point):
        t_hat = (point - self.p1).dot(self.p2 - self.p1) / ((self.p2 - self.p1).norm() ** 2)
        t_star = min(max(t_hat, 0), 1)
        dist = (self.p1 + (self.p2 - self.p1).mult(t_star) - point).norm()
        return dist

    def find_inliers(self, points, inlier_threshold):
        for point in points:
            dist = self.dist_from_segment(point)
            if dist < inlier_threshold:
                self.inliers.append(point)

    def plot(self, c='g'):
        xs = [self.p1.x, self.p2.x]
        ys = [self.p1.y, self.p2.y]
        plt.plot(xs, ys, c)

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

    def __init__(self, range_treshold, path_to_map='maze_new.txt', path_to_measurements='measurements_new.txt'):
        self.num_angles = 20
        self.range_treshold = range_treshold
        self.poses = []
        self.walls = []
        self.measurements = []
        self.interesting_points = []
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        self._read_map(path_to_map)
        self._read_measurements(path_to_measurements)

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
                        self.interesting_points.append(point)

        return measurement_positions

    def _read_measurements(self, path_to_measurements):
        indices_to_use = range(0, 360, 360/self.num_angles)
        with open(path_to_measurements) as f:
            lines = f.readlines()
            for line in lines:
                coords = [float(c) for c in line.split(')')[0][1:].split(',')]
                ranges = [float(r) for i, r in enumerate(line.split(')')[1][3:].split(','))
                          if i in indices_to_use]
                self.poses.append(Pose(*coords))
                self.measurements.append(ranges)

    def plot(self):
        self.get_interesting_points()
        print("Number of interesting points: %d" % len(self.interesting_points))

        # [p.plot() for p in meas_pos[::1]]
        # [p.plot('bo') for p in self.interesting_points[::1]]
        # plt.show()

        self.show_walls()
        [p.plot('ro') for p in self.interesting_points[::1]]
        plt.show()

    def _read_map(self, path_to_map):
        min_x = 100
        max_x = -100
        min_y = 100
        max_y = -100

        with open(path_to_map) as f:
            lines = f.readlines()
            for line in lines:
                if line[0] == '#':
                    continue
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

    def show_walls(self):
        [w.plot() for w in self.walls]


class Mapper:
    """ Have points_threshold > num_inlier_threshold """

    def __init__(self, inlier_distance_threshold=0.1, num_inlier_threshold=30):
        self.inlier_distance_threshold = inlier_distance_threshold
        self.num_inlier_threshold = num_inlier_threshold
        self.interesting_points = []

    def RANSAC(self, points):
        num_points = len(points)
        best_num_inliers = 0
        for i in range(num_points):
            for j in range(num_points):
                p1 = points[i]
                p2 = points[j]
                if p1 == p2:
                    continue

                segment = Segment(p1, p2)

                segment.find_inliers(points, self.inlier_distance_threshold)
                if len(segment.inliers) > best_num_inliers:
                    best_segment = segment
                    best_num_inliers = len(best_segment.inliers)

        return best_segment

    def sequential_RANSAC(self, points, neighbor_dist_threshold, number_of_neighbors_threshold):
        segments = []
        points_left = points
        count = 0

        while True:
            print('Number of interesting points: %d \n' % len(points_left))
            print('Starting to fit line #%d \n' % (count+1))
            best_segment = self.RANSAC(points_left)
            if len(best_segment.inliers) < self.num_inlier_threshold:
                print('Breaking, not enough inliers anymore!')
                break

            count += 1
            print("Line: %s" % count)
            print("wall: %s" % best_segment)

            segments.append(best_segment)

            outliers = []
            for point in points_left:
                found = False
                for inlier in best_segment.inliers:
                    if point == inlier:
                        found = True

                if not found:
                    outliers.append(point)

            points_left = outliers

        final_walls = []
        for segment in segments:
            final_walls.append(self.get_wall(segment, neighbor_dist_threshold, number_of_neighbors_threshold))

        return final_walls

    def get_wall(self, segment, neighbor_dist_threshold, number_of_neighbors_threshold):
        points = segment.inliers
        points_close_to_wall = self._find_cluster(points, neighbor_dist_threshold, number_of_neighbors_threshold)
        wall = self.RANSAC(points_close_to_wall)
        return wall

    @staticmethod
    def _find_cluster(points, neighbor_dist_threshold, number_of_neighbors_threshold):
        indices_to_remove = []
        for i, p in enumerate(points):
            num_neighbors = p.number_of_neighbors(points, neighbor_dist_threshold) - 1  # Removing the point itself from its neighbors
            if num_neighbors < number_of_neighbors_threshold:
                indices_to_remove.append(i)

        return [p for i, p in enumerate(points) if i not in indices_to_remove]


if __name__ == "__main__":

    range_from_wall_threshold = 0.15
    inlier_distance_threshold = 0.03
    num_inliers_threshold = 15  # maybe reduce further?

    meas = Measurements(range_from_wall_threshold)
    # meas.plot()

    meas.get_interesting_points()
    points = meas.interesting_points

    mapper = Mapper(inlier_distance_threshold, num_inlier_threshold=num_inliers_threshold)
    segments = mapper.sequential_RANSAC(points, 0.05, 1)

    meas.show_walls()
    # [p.plot('ro') for p in meas.interesting_points]
    [s.plot('r') for s in segments]
    plt.show()
