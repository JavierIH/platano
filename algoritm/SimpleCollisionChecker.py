import cv2

__author__ = 'def'

class SimpleCollisionChecker:
    def __init__(self):
        pass

    def collides(self, obstacle, point):
        return cv2.pointPolygonTest(obstacle, point, False) >= 0 # Inside/over contour