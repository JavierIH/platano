from RandomNodeGenerator import RandomNodeGenerator
from HammersleyNodeGenerator import HammersleyNodeGenerator
from HaltonNodeGenerator import HaltonNodeGenerator

from SimpleCollisionChecker import SimpleCollisionChecker
from CircleCollisionChecker import CircleCollisionChecker

from Environment import Environment
from DilatedEnvironment import DilatedEnvironment

from a_algorithm import a_algorithm

import numpy as np

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
        # Configure planner:
        self.environment_image = environment_image
        self.n_nodes = nodes
        self.threshold_neighbors = threshold_neighbors

        # Node generation
        self.node_generator = None
        if node_gen_method == 'random':
            self.node_generator = RandomNodeGenerator()
        elif node_gen_method == 'Hammersley':
            self.node_generator = HammersleyNodeGenerator()
        elif node_gen_method == 'Hammersley':
            self.node_generator = HaltonNodeGenerator()
        else:
            raise Exception('Node generator requested (%s) not found!'%node_gen_method)

        # Collision checker
        self.collision_checker = None
        self.environment = None

        if collision_method == 'simple' or collision_method == 'dilate':
            self.collision_checker = SimpleCollisionChecker()
        elif collision_method == 'circle':
            self.collision_checker = CircleCollisionChecker(environment_image.shape[:2], collision_radius)
        else:
            raise Exception('Collision method requested (%s) not found!'%collision_method)

        if collision_method == 'simple' or collision_method == 'circle':
            self.environment = Environment(self.environment_image, self.collision_checker)
        elif collision_method == 'dilate':
            self.environment = DilatedEnvironment(self.environment_image, self.collision_checker, collision_radius)

        # Generate nodes:
        self.nodes = self.node_generator.generatenodes(self.environment, self.n_nodes)

        #points[0] = (int(start[0]), int(start[1]))
        #points[len(points)-1] = (int(goal[0]), int(goal[1]))
        size = len(self.nodes)
        self.distance_matrix = np.zeros((size, size))
        for i, origin in enumerate(self.nodes):
            for j, end in enumerate(self.nodes):
                dist = np.linalg.norm(np.array(origin) - np.array(end))
                if dist <= self.threshold_neighbors and self.environment.is_line_valid(origin, end):
                    self.distance_matrix[i, j] = dist
                else:
                    self.distance_matrix[i, j] = -1


    def find_path(self, node_origin, node_goal):
        """
        Finds the shortest path between node_origin and node_goal
        :return: A list of points containing the shortest path
        """

        # Put origin and goal in the graph

        # Calculate shortest path using A*
        path = a_algorithm(0, len(self.nodes)-1, self.nodes, self.distance_matrix)

        return []

    def find_path_and_simplify(self, node_origin, node_goal):
        path = self.find_path(node_origin, node_goal)
        useless_node = True
        aux = len(path)-1
        aux2 = aux - 1

        # erase the useless nodes in the path
        while aux > 0:
            while useless_node and aux2 > 0:
                aux2 -= 1
                useless_node = self.environment.is_line_valid(points[path[aux]], points[path[aux2]])
                if useless_node:
                    del path[aux2+1]
                    aux -= 1
            aux = aux2
            aux2 -= 1
            useless_node = True

if __name__ == '__main__':
    pass