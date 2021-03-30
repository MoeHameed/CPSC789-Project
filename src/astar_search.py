import utils
from modules.a_star import AStar

class AStarSearch(AStar):
    def __init__(self):
        pass

    def search(self, start, end):
        path = list(self.astar(start, end))
        if path[-1] != end:
            path.append(end)
        return path

    def heuristic_cost_estimate(self, n1, n2):
        return utils.distanceCalc(n1, n2)

    def distance_between(self, n1, n2):
        return utils.distanceCalc(n1, n2)

    def neighbors(self, node):
        x, y, z = node
        nlist = [(nx, ny, nz) for nx, ny, nz in [(x, y, z - 3), (x, y, z + 3), (x, y - 3, z), (x, y - 3, z - 3), (x, y - 3, z + 3), (x, y + 3, z), (x, y + 3, z - 3), (x, y + 3, z + 3), (x - 3, y, z), (x - 3, y, z - 3), (x - 3, y, z + 3), (x - 3, y - 3, z), (x - 3, y - 3, z - 3), (x - 3, y - 3, z + 3), (x - 3, y + 3, z), (x - 3, y + 3, z - 3), (x - 3, y + 3, z + 3), (x + 3, y, z), (x + 3, y, z - 3), (x + 3, y, z + 3), (x + 3, y - 3, z), (x + 3, y - 3, z - 3), (x + 3, y - 3, z + 3), (x + 3, y + 3, z), (x + 3, y + 3, z - 3), (x + 3, y + 3, z + 3)]]
        # TODO: Check if any node in nodeList is intersecting with an obstacle
        return nlist

    def is_goal_reached(self, current, goal):
        return current == goal or utils.distanceCalc(current, goal) < 4