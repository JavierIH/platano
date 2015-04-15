import cv2
import numpy as np
import cgkit.hammersley

__author__ = 'def'

class Environment:
    def __init__(self, image_to_load):
        self.image_to_load = image_to_load

        # Load image as environment:
        self.image = cv2.imread(image_to_load, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        self.y_limit, self.x_limit = self.image.shape

        self.obstacles = []
        self.findObstacles()

    def findObstacles(self):
        # Find contours
        image2 = self.image.copy()
        contours, dummy = cv2.findContours(image2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Simplify contours
        # hull_contours = [ cv2.convexHull(contour) for contour in contours]
        simple_contours = [ cv2.approxPolyDP(contour, 2, True) for contour in contours]

        self.obstacles = simple_contours

    def generate_raw_random_nodes(self, n):
        samples = np.random.random((n, 2))
        samples[:, 0] = samples[:, 0] * self.x_limit
        samples[:, 1] = samples[:, 1] * self.y_limit
        random_points_list = np.array(samples, np.uint32).tolist()
        random_points = [ tuple(point) for point in random_points_list]

        return random_points

    def generate_random_nodes(self, n):
        return [ point for point in self.generate_raw_random_nodes(n) if self.is_legal(point) ]

    def generate_raw_hammersley_nodes(self, n):
        return [ (int(point[0]*self.x_limit), int(point[1]*self.y_limit)) for point in cgkit.hammersley.planeHammersley(n) ]

    def generate_hammersley_nodes(self, n):
        return [ point for point in self.generate_raw_hammersley_nodes(n) if self.is_legal(point) ]

    def is_legal(self, point):
        legal = True
        for obstacle in self.obstacles:
            if cv2.pointPolygonTest(obstacle, point, False) >= 0: # Inside/over contour
                legal = False
        return legal

def main():

    image_to_load = 'environment_test.png'


    env = Environment(image_to_load)
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
    random_points = env.generate_random_nodes(200)
    hammersley_points = env.generate_hammersley_nodes(200)

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