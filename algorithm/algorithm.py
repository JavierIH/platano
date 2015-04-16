import cv2
import numpy as np

from RandomNodeGenerator import RandomNodeGenerator
from HammersleyNodeGenerator import HammersleyNodeGenerator
from HaltonNodeGenerator import HaltonNodeGenerator

from SimpleCollisionChecker import SimpleCollisionChecker

from Environment import Environment

__author__ = 'def'


def generate_graph(environment, points, threshold=50):
    size = len(points)
    distance_matrix = np.zeros((size, size))
    for i, origin in enumerate(points):
        for j, end in list(enumerate(points))[i:]:
            dist = np.linalg.norm(np.array(origin) - np.array(end))
            if dist <= threshold and environment.is_line_valid(origin, end):
                distance_matrix[i, j] = dist
                distance_matrix[j, i] = dist
            else:
                distance_matrix[i, j] = -1
                distance_matrix[j, i] = -1

    return distance_matrix, size


def main():

    image_to_load = 'environment_test.png'

    # Create collision checker
    collision_checker = SimpleCollisionChecker()

    env = Environment(image_to_load, collision_checker)
    #cv2.imshow("Loaded img", env.image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    # Find obstacles
    #----------------------------------
    show = cv2.cvtColor(env.image, cv2.COLOR_GRAY2BGR)
    #cv2.drawContours(show, env.obstacles, -1, (0, 0, 255), 3)
    #cv2.imshow("Obstacles", show)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    # Add 200 random points:
    random_gen = RandomNodeGenerator()
    random_points = random_gen.generate_nodes(env, 200)

    hammersley_gen = HammersleyNodeGenerator()
    hammersley_points = hammersley_gen.generate_nodes(env, 200)

    halton_gen = HaltonNodeGenerator()
    halton_points = halton_gen.generate_nodes(env, 200)

    show = cv2.cvtColor(env.image, cv2.COLOR_GRAY2BGR)
    for point in random_points:
        cv2.circle(show, point, 2, (0, 255, 0), 1)
    cv2.drawContours(show, env.obstacles, -1, (0, 0, 255), 3)
    cv2.imshow("Random points", show)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()


    show = cv2.cvtColor(env.image, cv2.COLOR_GRAY2BGR)
    for point in hammersley_points:
        cv2.circle(show, point, 2, (255, 0, 0), 2)
    cv2.drawContours(show, env.obstacles, -1, (0, 0, 255), 3)
    cv2.imshow("hammersley points", show)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    show = cv2.cvtColor(env.image, cv2.COLOR_GRAY2BGR)
    for point in halton_points:
        cv2.circle(show, point, 2, (0, 0, 255), 2)
    cv2.drawContours(show, env.obstacles, -1, (0, 0, 255), 3)
    cv2.imshow("halton points", show)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()


    # Generate graph:
    # --------------------------------------------------------------
    points = hammersley_points
    distance_matrix, size = generate_graph(env, points, 50)

    # Draw connections:
    show = cv2.cvtColor(env.image, cv2.COLOR_GRAY2BGR)
    for point in points:
        cv2.circle(show, point, 2, (255, 0, 0), 2)
    cv2.drawContours(show, env.obstacles, -1, (0, 0, 255))

    for i in range(size):
        for j in range(i, size):
            if distance_matrix[i, j] > 0:
                origin = points[i]
                end = points[j]
                cv2.line(show, origin, end, (255, 255, 0))
    cv2.imshow("Connections", show)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print distance_matrix

if __name__ == '__main__':
    main()