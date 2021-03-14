from base_station import BaseStation
import utils

class BaseStationManager:
    def __init__(self):
        self.baseStations = []

        for i in range(utils.BS_NUM):
            self.baseStations.append(BaseStation(utils.BS_POS_LIST[i]))

    def GetNetQuality(self, cell):
        # TODO: Add angle calc
        dist = utils.distanceCalc(cell, self.baseStations[0].position)
        q = -0.002770936 + 0.04449319*dist - 0.00001817358*dist**2 - 0.00001205633*dist**3
        return max(0, min(1, q))