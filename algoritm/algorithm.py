import cv2
import numpy as np

from RandomNodeGenerator import RandomNodeGenerator
from HammersleyNodeGenerator import HammersleyNodeGenerator

from SimpleCollisionChecker import SimpleCollisionChecker

from Environment import Environment

__author__ = 'def'

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
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()