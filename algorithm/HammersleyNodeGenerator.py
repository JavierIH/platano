import cgkit.hammersley

__author__ = 'def'


class HammersleyNodeGenerator:
    def __init__(self):
        pass

    def generate_raw_nodes(self, environment, n ):
        return [ (int(point[0]*environment.x_limit), int(point[1]*environment.y_limit)) for point in cgkit.hammersley.planeHammersley(n)]

    def generate_nodes(self, environment, n):
        return [ point for point in self.generate_raw_nodes(environment, n) if environment.is_valid(point) ]