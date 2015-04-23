import numpy as np


__author__ = 'kike'


def a_algorithm(start, goal, points, dist):

    openset = [start]
    node_conections = [(start, 0)]
    openset_dist = [15]
    path = []
    closedset = set()
    current = start
    pos = 0
    nodes = 1

    while nodes > 0 and current != goal:

        # Look for the neighbours of the current point. Add them to the openset list, with their cost value
        for i in range(len(points)):
            if dist[current, i] > 0 and i not in closedset and i not in openset:
                goal_dist = np.sqrt(((points[goal][0] - points[i][0])**2)+((points[goal][1] - points[i][1])**2))
                total_dist = dist[current, i] + goal_dist + openset_dist[pos]
                openset.append(i)
                node_conections.append((i, current))
                openset_dist.append(total_dist)
                nodes += 1

        # Select the best node in openset
        best_dist = 0
        best_node = 0

        for i in range(len(openset_dist)):
            if openset_dist[i] < best_dist:
                best_dist = openset_dist[i]
                best_node = i

        current = openset[best_node]
        pos = best_node

        # Move the current node to the closedset list and delete it from the openset list
        closedset.add(current)
        del openset[pos]
        del openset_dist[pos]
        nodes -= 1

    path.append(current)
    while current != start:
        for i in range(len(node_conections)):
            if node_conections[i][0] == current:
                current = node_conections[i][1]
                path.append(current)

    print("Fin del algoritmo")
    return path