from RandomNodeGenerator import RandomNodeGenerator
from HammersleyNodeGenerator import HammersleyNodeGenerator

from SimpleCollisionChecker import SimpleCollisionChecker
from CircleCollisionChecker import CircleCollisionChecker

from Environment import Environment
from DilatedEnvironment import DilatedEnvironment

from a_algorithm import a_algorithm

__author__ = 'def'

class Planner:
    def __init__(self, environment_image,
                 node_gen_method='random', nodes=200, threshold_neighbors='??',
                 collision_method='simple', collision_radius=None):
        """
        :param environment_image: matrix containing the segmented b/w map
        :param node_gen_method: method for generating the nodes
            'random' -> random
            'Hammersley' -> Hammersley point distribution
            'Halton' -> Halton point distribution
        :param nodes: number of nodes generated (before collision checking)
        :param threshold_neighbors: maximum distance to consider two nodes connected
        :param collision_method: method to check collision
            'simple' -> point to obstacle checking
            'circle' -> circle (robot) to obstacle checking
            'dilate' -> point to dilated obstacle checking
        :param collision_radius: in methods using this parameter, this is the radius of the
            robot
        """
        pass

    def find_path(self, node_origin, node_destination):
        """
        Finds the shortest path between node_origin and node_destination
        :return: A list of points containing the shortest path
        """
        return []


