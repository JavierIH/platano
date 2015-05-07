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
                 node_gen_method='random', nodes=200, threshold_neighbors=50,
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
        self.nodes = self.node_generator.generate_nodes(self.environment, self.n_nodes)

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
        :return: A list of points containing the shortest path and the graph
        extended with origin and goal nodes
        """

        # Put origin and goal in the graph
        graph_nodes = list(self.nodes)
        graph_nodes.insert(0, node_origin)
        graph_nodes.append(node_goal)

        # Recalculate connection matrix:
        size = len(graph_nodes)
        connection_matrix = np.zeros((size, size))
        connection_matrix[1:-1,1:-1] = self.distance_matrix

        for j, end in enumerate(graph_nodes):
            # Calculate connections to origin node:
            dist_origin = np.linalg.norm(np.array(graph_nodes[0]) - np.array(end))
            if dist_origin <= self.threshold_neighbors and self.environment.is_line_valid(graph_nodes[0], end):
                connection_matrix[0, j] = dist_origin
            else:
                connection_matrix[0, j] = -1

            # Calculate connections to goal node:
            dist_goal = np.linalg.norm(np.array(graph_nodes[-1]) - np.array(end))
            if dist_goal <= self.threshold_neighbors and self.environment.is_line_valid(graph_nodes[-1], end):
                connection_matrix[-1, j] = dist_goal
            else:
                connection_matrix[-1, j] = -1

        # Calculate shortest path using A*
        path = a_algorithm(0, len(graph_nodes)-1, graph_nodes, connection_matrix)

        return path, graph_nodes

    def find_path_and_simplify(self, node_origin, node_goal):
        path, points= self.find_path(node_origin, node_goal)
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

        return path, points

if __name__ == '__main__':
    import cv2

    image_to_load = 'environment_test.png'

    planner = Planner(image_to_load, 'Hammersley', 200, 50, 'dilate', 10)

    # Ask for input
    # Note: second check does not work

    # Ask for the initial point and the goal point
    print("Los limites del mapa son: ", planner.environment.x_limit, planner.environment.y_limit)

    i_point = [0,0]
    i_point[0] = int(input("Introduzca la coordenada x del punto inicial:"))
    i_point[1] = int(input("Introduzca la coordenada y del punto inicial:"))
    start = tuple(i_point)
    valid_start = planner.environment.is_valid(start)

    while start[0] < 0 or start[0] > planner.environment.x_limit or start[1] < 0 or start [1] > planner.environment.y_limit or valid_start == False:
        print("el punto seleccionado no es valido")
        start[0] = int(input("Introduzca la coordenada x del punto inicial:"))
        start[1] = int(input("Introduzca la coordenada y del punto inicial:"))
        valid_start = planner.environment.is_valid(start)

    g_point = [0,0]
    g_point[0] = int(input("Introduzca la coordenada x del punto final:"))
    g_point[1] = int(input("Introduzca la coordenada y del punto final:"))
    goal = tuple(g_point)
    valid_goal = planner.environment.is_valid(goal)

    while goal[0] < 0 or goal[0] > planner.environment.x_limit or goal[1] < 0 or goal[1] > planner.environment.y_limit or valid_goal == False:
        print("el punto seleccionado no es valido")
        goal[0] = int(input("Introduzca la coordenada x del punto final:"))
        goal[1] = int(input("Introduzca la coordenada y del punto final:"))
        valid_goal = planner.environment.is_valid(goal)

    # Calculate path
    # path, points = planner.find_path_and_simplify(start, goal)
    path, points = planner.find_path(start, goal)

    # Draw paths
    show = cv2.cvtColor(planner.environment.image, cv2.COLOR_GRAY2BGR)
    for i in range(len(path)-1):
        origin = points[path[i]]
        end = points[path[i+1]]
        cv2.line(show, origin, end, (0, 0, 255))
    for point in points:
        cv2.circle(show, point, 2, (255, 0, 0), 2)
    cv2.circle(show, points[0], 2, (0, 255, 0), 2)
    cv2.circle(show, points[len(points)-1], 2, (255, 255, 255), 2)
    cv2.imshow("Path", show)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
