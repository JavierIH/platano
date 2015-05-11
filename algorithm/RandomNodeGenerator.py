import numpy as np

__author__ = 'def'


class RandomNodeGenerator:
    def __init__(self):
        pass

    def generate_raw_nodes(self, environment, n ):
        samples = np.random.random((n, 2))
        samples[:, 0] = samples[:, 0] * environment.x_limit
        samples[:, 1] = samples[:, 1] * environment.y_limit
        random_points_list = np.array(samples, np.uint32).tolist()
        random_points = [ tuple(point) for point in random_points_list]

        return random_points

    def generate_nodes(self, environment, n):
        return [ point for point in self.generate_raw_nodes(environment, n) if environment.is_valid(point) ]

