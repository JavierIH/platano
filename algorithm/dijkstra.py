import heapq


class PriorityQueue(object):
    """Priority queue based on heap, capable of inserting a new node with
    desired priority, updating the priority of an existing node and deleting
    an abitrary node while keeping invariant"""

    def __init__(self, heap=[]):
        """if 'heap' is not empty, make sure it's heapified"""

        heapq.heapify(heap)
        self.heap = heap
        self.entry_finder = dict({i[-1]: i for i in heap})
        self.REMOVED = '<remove_marker>'

    def insert(self, node, priority=0):
        """'entry_finder' bookkeeps all valid entries, which are bonded in
        'heap'. Changing an entry in either leads to changes in both."""

        if node in self.entry_finder:
            self.delete(node)
        entry = [priority, node]
        self.entry_finder[node] = entry
        heapq.heappush(self.heap, entry)

    def delete(self, node):
        """Instead of breaking invariant by direct removal of an entry, mark
        the entry as "REMOVED" in 'heap' and remove it from 'entry_finder'.
        Logic in 'pop()' properly takes care of the deleted nodes."""

        entry = self.entry_finder.pop(node)
        entry[-1] = self.REMOVED
        return entry[0]

    def pop(self):
        """Any popped node marked by "REMOVED" does not return, the deleted
        nodes might be popped or still in heap, either case is fine."""

        while self.heap:
            priority, node = heapq.heappop(self.heap)
            if node is not self.REMOVED:
                del self.entry_finder[node]
                return priority, node
        raise KeyError('pop from an empty priority queue')


def dijkstra(start, goal, dist, points):

    conection = False
    path = []
    current = start
    pq = PriorityQueue()
    closedset = set()
    node_conections = [(start, 0)]

    for i in range(len(points)):
        if i == start:
            pq.insert(i, 0)
        else:
            pq.insert(i, 10000)

    while len(pq.heap) > 0 and current != goal:
        min_dist, current = pq.pop()
        closedset.add(current)

        for i in range(len(points)):
            if dist[current, i] > 0 and i not in closedset:
                act_dist = pq.delete(i)
                new_dist = min(dist[current, i] + min_dist, act_dist)
                pq.insert(i, new_dist)
                node_conections.append((i, current))

    path.append(current)

    while current != start:
        conection = False

        for i in range(len(node_conections)):
            if node_conections[i][0] == current:
                current = node_conections[i][1]
                path.append(current)
                conection = True

        if conection == False:
            print "No existe una trayectoria directa entre los nodos inicial y final"
            path=0
            break

    return path
























