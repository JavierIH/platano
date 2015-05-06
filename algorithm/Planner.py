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
        pass

    def find_path(self, node_origin, node_destination):
        return []


