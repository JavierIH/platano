__author__ = 'def'
import cv2
import numpy as np

__author__ = 'def'


class CircleCollisionChecker:
    def __init__(self, environment_size, radius = 1):
        self.env_size = environment_size
        self.radius = radius

    def collides(self, obstacle, point):
        # Create two images:
        image_point = np.zeros((self.env_size[0], self.env_size[1], 1), np.uint8)
        cv2.circle(image_point, point, self.radius, 255, cv2.cv.CV_FILLED)
        # cv2.imshow("image_point", image_point)

        image_obstacle = np.zeros((self.env_size[0], self.env_size[1], 1), np.uint8)
        cv2.fillConvexPoly(image_obstacle, obstacle, 255)
        # cv2.imshow("image_obstacle", image_obstacle)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        result = cv2.bitwise_and(image_point, image_obstacle)

        return cv2.countNonZero(result) > 0

    def line_collides(self, obstacle, origin_point, end_point, n_samples = 10):
        origin = np.array(origin_point)
        end = np.array(end_point)
        v = end - origin

        for i in range(n_samples):
            point_to_be_checked = tuple((origin + v * np.true_divide(i, n_samples)).astype('int'))
            if self.collides(obstacle, point_to_be_checked):
                return True

        return False
