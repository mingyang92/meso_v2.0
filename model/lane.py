JAMDENSITY = 124
import numpy as np

class Lane(object):
    # as it is lane, it has only one direction; this is the different part from link
    def __init__(self, id, type, link, freeSpeed, freeTravelTime, fixedCharge, speed, network):
        self.id = id
        self.type = type
        self.link = link

        self.length = None
        self.freeSpeed = int(freeSpeed)
        self.freeTravelTime = freeTravelTime
        self.fixedCharge = fixedCharge

        self.speed = speed
        self.averageDelay = 0.5 ## not sure this number
        self.maxPcu = 20 # in Pcu
        self.minPcu = 5 # in Pcu
        self.maxDelay = 10 # in second !!! not sure
        self.minDelay = 5 # in second !!! not sure


        self.countPcu = 0
        self.density = 0.1
        self.travelTime = 0.1
        self.charge = 0.1

        #self.timeSpeedMap = {}
        #self.timeCountMap = {}
        #self.timeDensityMap = {}

        self.network = network
        self.network.registerLane(self)

    def __repr__(self):
        return "<" + " ".join(["lane" + self.id, self.type, str(self.link.node1.id), str(self.link.node2.id),
                               "density: "+str(self.density), "speed: "+str(self.speed),
                               "travelTime: "+str(self.travelTime) ]) + ">"

    def freeTimeInSec(self):
        """
        This function calculates the TRAVEL TIME for this LANE in FREE SPEED
        :return: travel time
        """
        return self.link.lengthInKm / self.freeSpeed * 3600.0

    def travelTimeInSec(self):
        """
        This function calculates the TRAVEL TIME for this LANE IN ? SPEED
        :return: travel time
        """
        return self.freeTimeInSec() * 1.25 #magic number

    def updatePropertiesBasedOnPcu(self):
        """
        This function is to update PCU properties
        :return: None
        """
        def densitySpeed(density, freespeed):
            """
            This function calculates the density speed based on density
            :param density:
            :param freespeed:
            :return:
            """
            # this function returns the speed on a lane when inputting a density
            # negative linear relationship between density and speed is used
            # additional parameters of free speed and jam  density are needed.
            if density >= JAMDENSITY:  # to avoid getting 0 or minus speed
                #speed = 0.001  # km/h
                speed = 0.99
            else:
                speed = freespeed * (1.0 - 1.0 * density / JAMDENSITY)
            return speed
        #print('countPcu', self.countPcu, 'length in km:', self.link.lengthInKm)
        self.density = self.countPcu / self.link.lengthInKm
        self.speed = densitySpeed(self.density, self.freeSpeed)
        self.travelTime = self.link.lengthInKm * 3600.0 / self.speed
        #print(self.id, 'The current density is:', self.density, 'the speed is:', self.speed, 'JAM', JAMDENSITY)

    def delayCalculation(self, strategy):
        if strategy == 'vol_sim':
            delayingTime = self.countPcu * self.averageDelay

        elif strategy == 'vol_dist':
            if self.countPcu > self.maxPcu:
                delayingTime = self.maxDelay
            elif self.countPcu < self.minPcu:
                delayingTime = self.minDelay
            else:
                delayingTime = int(self.minDelay + 1.0 * (self.maxDelay - self.minDelay)
                                   * (self.countPcu - self.minPcu)/(self.maxPcu - self.minPcu))
        elif strategy == 'random':
            delayingTime = np.random.randint(self.minDelay, self.maxDelay)

        elif strategy == 'fix':
            delayingTime = 5
        return delayingTime

