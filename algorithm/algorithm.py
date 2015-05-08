import cv2
import numpy as np

from RandomNodeGenerator import RandomNodeGenerator
from HammersleyNodeGenerator import HammersleyNodeGenerator

from SimpleCollisionChecker import SimpleCollisionChecker
from CircleCollisionChecker import CircleCollisionChecker

from Environment import Environment
from DilatedEnvironment import DilatedEnvironment

from a_algorithm import a_algorithm

from dijkstra import dijkstra

__author__ = 'def'


def main():

    image_to_load = 'environment_test.png'

    # Create collision checker
    simple_collision_checker = SimpleCollisionChecker()
    circle_collision_checker = CircleCollisionChecker((400, 640), 10)
    collision_checker = simple_collision_checker

    env1 = Environment(image_to_load, circle_collision_checker)
    env2 = DilatedEnvironment(image_to_load, collision_checker, 25)
    env = env2
    cv2.imshow("Loaded img", env.image)

    # Ask for the initial point and the goal point
    print("Los limites del mapa son: ", env.x_limit, env.y_limit)

    i_point = np.zeros((2, 1))
    i_point[0] = int(input("Introduzca la coordenada x del punto inicial:"))
    i_point[1] = int(input("Introduzca la coordenada y del punto inicial:"))
    start = tuple(i_point)
    valid_start = env.is_valid(start)

    while start[0] < 0 or start[0] > env.x_limit or start[1] < 0 or start [1] > env.y_limit or valid_start == False:
        print("el punto seleccionado no es valido")
        i_point[0] = int(input("Introduzca la coordenada x del punto inicial:"))
        i_point[1] = int(input("Introduzca la coordenada y del punto inicial:"))
        start = tuple(i_point)
        valid_start = env.is_valid(start)

    g_point = np.zeros((2, 1))
    g_point[0] = int(input("Introduzca la coordenada x del punto final:"))
    g_point[1] = int(input("Introduzca la coordenada y del punto final:"))
    goal = tuple(g_point)
    valid_goal = env.is_valid(goal)

    while goal[0] < 0 or goal[0] > env.x_limit or goal[1] < 0 or goal[1] > env.y_limit or valid_goal == False:
        print("el punto seleccionado no es valido")
        g_point[0] = int(input("Introduzca la coordenada x del punto final:"))
        g_point[1] = int(input("Introduzca la coordenada y del punto final:"))
        goal = tuple(g_point)
        valid_goal = env.is_valid(goal)

    #cv2.waitKey(0)
    #cv2.destroyAllWindows()


    # Find obstacles
    #----------------------------------
    show = cv2.cvtColor(env.image, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(show, env.obstacles, -1, (0, 0, 255), 3)
    #cv2.imshow("Obstacles", show)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    # Add 200 random points:
    random_gen = RandomNodeGenerator()
    random_points = random_gen.generate_nodes(env, 200)

    hammersley_gen = HammersleyNodeGenerator()
    hammersley_points = hammersley_gen.generate_nodes(env, 200)


    show = cv2.cvtColor(env.image, cv2.COLOR_GRAY2BGR)
    for point in random_points:
        cv2.circle(show, point, 2, (0, 255, 0), 1)
    cv2.drawContours(show, env.obstacles, -1, (0, 0, 255), 3)
    #cv2.imshow("Random points", show)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()


    show = cv2.cvtColor(env.image, cv2.COLOR_GRAY2BGR)
    for point in hammersley_points:
        cv2.circle(show, point, 2, (255, 0, 0), 2)
    cv2.drawContours(show, env.obstacles, -1, (0, 0, 255), 3)
    #cv2.imshow("hammersley points", show)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    # Generate graph:
    points = hammersley_points
    points[0] = (int(start[0]), int(start[1]))
    points[len(points)-1] = (int(goal[0]), int(goal[1]))
    threshold = 50
    size = len(points)
    distance_matrix = np.zeros((size, size))
    for i, origin in enumerate(points):
        for j, end in enumerate(points):
            dist = np.linalg.norm(np.array(origin) - np.array(end))
            if dist <= threshold and env.is_line_valid(origin, end):
                distance_matrix[i, j] = dist
            else:
                distance_matrix[i, j] = -1

    # Draw connections:
    show = cv2.cvtColor(env.image, cv2.COLOR_GRAY2BGR)
    for point in points:
        cv2.circle(show, point, 2, (255, 0, 0), 2)
    cv2.drawContours(show, env.obstacles, -1, (0, 0, 255))

    for i in range(size):
        for j in range(size):
            if distance_matrix[i, j] > 0:
                origin = points[i]
                end = points[j]
                cv2.line(show, origin, end, (255, 255, 0))
    #cv2.imshow("Connections", show)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    #print distance_matrix

    #path = a_algorithm(0, len(points)-1, points, distance_matrix)
    path = dijkstra(0, len(points)-1, distance_matrix, points)
    useless_node = True
    aux = len(path)-1
    aux2 = aux - 1

    # erase the useless nodes in the path
    #while aux > 0:
    #    while useless_node and aux2 > 0:
    #        aux2 -= 1
    #        useless_node = env.is_line_valid(points[path[aux]], points[path[aux2]])
    #        if useless_node:
    #            del path[aux2+1]
    #            aux -= 1
    #    aux = aux2
    #    aux2 -= 1
    #    useless_node = True

    show = cv2.cvtColor(env.image, cv2.COLOR_GRAY2BGR)
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


if __name__ == '__main__':
    main()