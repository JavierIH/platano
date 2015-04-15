import cv2
import numpy as np

__author__ = 'def'

class SimpleCollisionChecker:
    def __init__(self):
        pass

    def collides(self, obstacle, point):
        return cv2.pointPolygonTest(obstacle, point, False) >= 0 # Inside/over contour

    def line_collides(self, obstacle, origin_point, end_point, n_samples = 10):
        origin = np.array(origin_point)
        end = np.array(end_point)
        v = end - origin

        for i in range(n_samples):
            point_to_be_checked = tuple(origin + v * np.true_divide(i, n_samples))
            if self.collides(obstacle, point_to_be_checked):
                return True

        return False
