import utils
from a_star import AStar

class PathPlanner(AStar):
    def __init__(self, useNetQual=True):
        self.useNetQual = useNetQual
    
    def search(self, start, end):
        path = list(self.astar(start, end))

        if path[-1] != end:
            path.append(end)

        return path

    def heuristic_cost_estimate(self, n1, n2):
        if self.useNetQual:
            return utils.distanceCalc(n1, n2) * (1 - utils.nodeNetworkQualCalc(n1))
        else:
            return utils.distanceCalc(n1, n2)

    def distance_between(self, n1, n2):
        if self.useNetQual:
            return utils.nodeNetworkQualCalc(n2) - utils.nodeNetworkQualCalc(n1)
        else:
            return utils.distanceCalc(n1, n2)

    def neighbors(self, node):
        x, y, z = node
        nlist = [(nx, ny, nz) for nx, ny, nz in [(x, y, z - 3), (x, y, z + 3), (x, y - 3, z), (x, y - 3, z - 3), (x, y - 3, z + 3), (x, y + 3, z), (x, y + 3, z - 3), (x, y + 3, z + 3), (x - 3, y, z), (x - 3, y, z - 3), (x - 3, y, z + 3), (x - 3, y - 3, z), (x - 3, y - 3, z - 3), (x - 3, y - 3, z + 3), (x - 3, y + 3, z), (x - 3, y + 3, z - 3), (x - 3, y + 3, z + 3), (x + 3, y, z), (x + 3, y, z - 3), (x + 3, y, z + 3), (x + 3, y - 3, z), (x + 3, y - 3, z - 3), (x + 3, y - 3, z + 3), (x + 3, y + 3, z), (x + 3, y + 3, z - 3), (x + 3, y + 3, z + 3)]]
        
        for n in nlist:
            if utils.isNodeInBS(n):
                nlist.remove(n)
        
        return nlist

    def is_goal_reached(self, current, goal):
        return current == goal or utils.distanceCalc(current, goal) < 4