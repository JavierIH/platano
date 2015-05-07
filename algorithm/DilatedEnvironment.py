import cv2

__author__ = 'def'


class DilatedEnvironment:
    def __init__(self, image_to_load, collision_checker, robot_radius):
        self.image_to_load = image_to_load

        # Load image as environment:
        self.image = cv2.imread(image_to_load, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        self.y_limit, self.x_limit = self.image.shape

        # Perform dilation of obstacles based on robot
        self.robot_radius = robot_radius

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.robot_radius*2,self.robot_radius*2))
        self.expanded_env = cv2.dilate(self.image, kernel)

        # Get obstacles from image
        self.obstacles = []
        self.findObstacles()

        # Set collision_checker
        self.collision_checker = collision_checker

    def findObstacles(self):
        # Find contours
        image2 = self.expanded_env.copy()
        contours, dummy = cv2.findContours(image2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Simplify contours
        # hull_contours = [ cv2.convexHull(contour) for contour in contours]
        simple_contours = [ cv2.approxPolyDP(contour, 2, True) for contour in contours]

        self.obstacles = simple_contours

    def is_valid(self, point):
        valid = True
        for obstacle in self.obstacles:
            if self.collision_checker.collides(obstacle, point):
                valid = False
        return valid

    def is_line_valid(self, origin, end):
        valid = True
        for obstacle in self.obstacles:
            if self.collision_checker.line_collides(obstacle, origin, end):
                valid = False
        return valid
