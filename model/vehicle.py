#from lib.Dijkstra2 import bestLaneBestNodeTimeCost
from lib.astar import bestLaneBestNodeTimeCost

class Vehicle(object):
    def __init__(self, id, type, driverType, maxSpeed, valueTime, probLaneChange, startTs, nodeOrigin, nodeDest, network):
        self.id = id
        self.type = type
        self.driverType = driverType
        self.maxSpeed = maxSpeed
        self.valueTime = valueTime
        self.probLaneChange = probLaneChange
        self.startTs = startTs
        self.nodeOrigin = nodeOrigin
        self.nodeDest = nodeDest
        self.delayingTime = 0

        self.network = network
        self.network.registerVehicle(self)

        self.finishTs = None

        self.laneType = '0' #['1': high speed, '0': low speed]

        # store real-time info
        self.bestLaneRoute = None
        self.currentLane = None
        self.currentLaneProgress = None # percentage on the lane
        self.timeBudget = None
        # parameters below should be calculated on every tick
        #self.timeBudget = timeBudget
        #self.expectTimeToEnd = 10000
        self.change_lane = 0
        #self.listTs = listTs
        #self.tsLocationMap = tsLocationMap
        #self.tsRouteMap = tsRouteMap

    def isBegin(self, ts):
        """
        This function judge whether the simulation starts
        :param ts: current time
        :return: boolean
        """
        return ts >= self.startTs

    def isFinish(self, ts):
        """
        This function judge whether the simulation finishes
        :param ts: current time stamp
        :return: boolean
        """
        return not not self.finishTs

    def isRunning(self, ts):
        """
        This function judge whether the vehicle is running or not
        :param ts: current time
        :return: y/n
        """
        return self.isBegin(ts) and not self.isFinish(ts)

    def __repr__(self):
        return "<" + " ".join(["vehicle" + str(self.id), "type: " + self.type,
                               "driver type: " + str(self.driverType),
                               "maxSpeed: " + str(self.maxSpeed),
                               "valueTime: " + str(self.valueTime),
                               "prob of change: " + str(self.probLaneChange),
                               "startTs: " + str(self.startTs),
                               "nodeOrigin: " + str(self.nodeOrigin),
                               "nodeDest: " + str(self.nodeDest),
                               "change lane: " + str(self.change_lane),
                               "current lane: " + str(self.currentLane)]) + ">"

    def updateShortestPath(self):
        """
        This function is to update next shortest path
        :return: None
        """
        startNode = self.currentLane.link.node2 if self.currentLane else self.nodeOrigin #determine node1 of next lane

        #print(self.network.typeGraphMap[self.laneType])
        (bestLaneRoute, bestNodeMap, timeCost) = bestLaneBestNodeTimeCost(self.network.typeGraphMap[self.laneType],
                                                                          startNode.id, self.nodeDest.id, self.network)
        self.bestLaneRoute = bestLaneRoute
        self.bestNodeMap = bestNodeMap
        self.timeBudget = timeCost
        if not self.currentLane:
            self.currentLane = self.network.typeGraphMap[self.laneType][startNode.id][bestNodeMap[startNode.id]]
            self.currentLaneProgress = 0
        #print(self.bestLaneRoute, self.timeBudget, bestNodeMap, self.currentLane)


    def updateLocation(self, timeInSecond, delayType='fix'):
        """
        This function updates the location of vehicle in A LANE
        :param timeInSecond: update time, gap
        :param delayType: delaying type at the interaction
        :return: None
        """
        remainingTime = timeInSecond

        if self.currentLane.link.node2 == self.nodeDest and remainingTime < 1:
            # finish
            self.finishTs = self.network.ts
            self.currentLane = None
            self.currentLaneProgress = None
            print(self, "finished at", self.finishTs)
            return

        while True:
            # if self.id == 1: print(self.currentLane, self.currentLaneProgress, self.bestLaneRoute)
            timeUseToFinishLane = 3600.0 * (
                        1.0 - self.currentLaneProgress) * self.currentLane.link.lengthInKm / self.currentLane.speed
            # if self.id == 1: print(timeUseToFinishLane)
            # if delay in node, updating time as below
            if self.delayingTime > 0:
                if self.delayingTime < remainingTime:
                    remainingTime -= self.delayingTime
                    self.delayingTime = 0
                else:
                    self.delayingTime -= remainingTime
                    remainingTime = 0
                    break
            else:
                if remainingTime > timeUseToFinishLane:
                    remainingTime -= timeUseToFinishLane

                    if self.currentLane.link.node2 == self.nodeDest:
                        # finish
                        self.finishTs = self.network.ts
                        self.currentLane = None
                        self.currentLaneProgress = None
                        print(self, "finished at", self.finishTs)
                        return
                    else:
                        #self.delayingTime = 5 # currently the delaying time is fix, later it may be change
                        # There are 4 types of delay strategy
                        self.delayingTime = self.currentLane.delayCalculation(delayType)
                        self.updateShortestPath()
                        # todo
                        self.currentLane = self.network.typeGraphMap[self.laneType][self.currentLane.link.node2.id][
                            self.bestNodeMap[self.currentLane.link.node2.id]]
                        self.currentLaneProgress = 0.0
                else:
                    break
        #update location
        self.currentLaneProgress += (self.currentLane.speed * remainingTime) / self.currentLane.link.lengthInKm / 3600.0


    def updateProbLaneChange(self, medianValueTime):
        """
        This function is to calculate the probability of changing lanes.
        目测暂时是用不到的
        :param medianValueTime: the value of time for A vehicle
        :return: None
        """
        if self.valueTime >= medianValueTime:

            self.probLaneChange = self.probLaneChange * 2

            if self.probLaneChange > 1:
                self.probLaneChange = 1

            if self.probLaneChange > 0.7 and self.laneType == '0':
                self.laneType = '1'

            else:
                if self.laneType == '1':
                    self.laneType = '0'

    def changeLane(self, originTimeCost, timeInSecond, medianValueTime, countTime, NO_CHARGE=False):
        """
        this function returns a result of whether to change to a faster and more expensive lane-network at a current
        time stamp after obtaining its current location.
        是否换道的决策取决于换道的概率：1.在fast上，如果换道的概率小于等于0.5，则换到低速道上；2.在slow上，如果概率大于0.5，则换到高速上
        :param originTimeCost: the total time used at current location
        :param timeInSecond: time stamp
        :param medianValueTime: median value
        :param countTime: decision should be made countTime sec before entering the interaction
        :param NO_CHARGE: if True, both lane are free of charge
        :return: None
        """


        # Step 1: whether it is in the final link

        if not self.bestLaneRoute:
            #print('Best route not found!')
            return

        self.change_lane = 0
        leftTimeToEnd = originTimeCost  # 至今为止还剩余的时间
        timeUseToFinishLane = (3600.0 * (
                1.0 - self.currentLaneProgress) * self.currentLane.link.lengthInKm / self.currentLane.speed) \
                              - countTime * timeInSecond
        expectTimeCostLow = timeUseToFinishLane + self.calculateExpTimeToEnd('0')
        expectTimeCostHigh = timeUseToFinishLane + self.calculateExpTimeToEnd('1')
        neighborLaneSpeed = self.network.typeGraphMap[str(1 - int(self.laneType))][
            self.currentLane.link.node1.id][self.currentLane.link.node2.id].speed

        # Step 2: if not in the final link, then calculate the finish time
        # 对于两种情况，当道路发生堵车时
        if self.currentLane.speed < 1:
            if neighborLaneSpeed < 1:
                #print('neigborLane', neighborLaneSpeed)
                #print('Neighbor lane is congest，stay in the current lane!')
                self.currentLane = self.network.typeGraphMap[self.laneType][
                    self.currentLane.link.node1.id][self.currentLane.link.node2.id]
                return

        # In the case of no charge lane
        if NO_CHARGE==True:
            # no charge的情况下，换道的判断仅限于当前路段。
            neighborLaneSpeed = self.network.typeGraphMap[str(1 - int(self.laneType))][
                self.currentLane.link.node1.id][self.currentLane.link.node2.id].speed
            timeUseToFinishLane_neighbor = (3600.0 * (
                    1.0 - self.currentLaneProgress) * self.currentLane.link.lengthInKm / neighborLaneSpeed) \
                                           - countTime * timeInSecond
            if timeUseToFinishLane_neighbor < timeUseToFinishLane:
                self.change_lane = 1
                self.laneType = str(1 - int(self.laneType))

            self.currentLane = self.network.typeGraphMap[self.laneType][
                self.currentLane.link.node1.id][self.currentLane.link.node2.id]
            return

        # In the case of charge lane
        if leftTimeToEnd > expectTimeCostLow:
            # 剩下的时间充足
            if self.laneType == '1':
                # 如果已经在高速道上，则换回道低速道
                self.laneType = '0'
                #print('CHARGE LANE: change to low speed lane.')
                self.change_lane = 1
            else:
                # 如果在低速道上则保持在低速道
                return
        elif leftTimeToEnd < expectTimeCostLow and leftTimeToEnd > expectTimeCostHigh:
            # 剩下的时间走高速道可以到达
            if self.type == 'bus':
                #print('CHARGE LANE: bus change due to jam.')
                self.change_lane = 1
                self.laneType = "1"
            elif self.type == 'car' and self.valueTime > medianValueTime:
                #print('CHARGE LANE: Prior car change due to jam!')
                self.change_lane = 1
                self.laneType = "1"
        else:
            # 剩下的时间不充足
            if self.type == 'bus':
                self.change_lane = 1
                self.laneType = "1"
                #print('CHARGE LANE: bus change to high!')


        # Step 3: when the vehicle in node, make changing decision based on expecting finishing time.
        #if timeUseToFinishLane > timeInSecond:
        #    return

        #self.updateProbLaneChange(originTimeCost, timeUseToFinishLane, timeInSecond, medianValueTime, countTime)
        self.currentLane = self.network.typeGraphMap[self.laneType][
            self.currentLane.link.node1.id][self.currentLane.link.node2.id]


    def calculateExpTimeToEnd(self, laneType):
        '''
        This function calculate the expecting time to destination from next lane in bestLaneRoute
        :param self:
        :param laneType: current lane type
        :return:
        '''
        # 计算现在条件下到达终点的预期时间
        # 计算是从下一条道开始
        expTime = 0
        for laneid in self.bestLaneRoute.keys():
            #print('lane id is:', laneid)
            if laneid != self.currentLane.id:
                expTime += self.network.typeGraphMap[laneType][
                            self.currentLane.link.node1.id][self.currentLane.link.node2.id].travelTime
        #print('expTime:', expTime)
        return expTime
