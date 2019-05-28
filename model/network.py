class Network(object):
    def __init__(self, ts):
        self.ts = ts
        self.idNodeMap = {}
        self.idLinkMap = {}
        self.idLaneMap = {}
        self.idVehicleMap = {}
        self.typeGraphMap = {}

    def registerNode(self, node):
        """
        This function registers the NODE in MAP
        :param node:
        :return: None
        """
        if node.id in self.idNodeMap: raise("duplicated node id")
        self.idNodeMap[node.id] = node

    def registerLink(self, link):
        """
        This function registers LINK in MAP
        :param link:
        :return: None
        """
        if link.id in self.idLinkMap: raise("duplicated link id")
        self.idLinkMap[link.id] = link

    def registerLane(self, lane):
        """
        This function registers LANE in MAP
        :param lane:
        :return: None
        """
        if lane.id in self.idLaneMap: raise("duplicated lane id")
        self.idLaneMap[lane.id] = lane

        if lane.type not in self.typeGraphMap:
            self.typeGraphMap[lane.type] = {}
        if lane.link.node1.id not in self.typeGraphMap[lane.type]:
            self.typeGraphMap[lane.type][lane.link.node1.id] = {}
        self.typeGraphMap[lane.type][lane.link.node1.id][lane.link.node2.id] = lane

    def registerVehicle(self, vehicle):
        """
        This function registers VEHICLE in MAP
        :param vehicle:
        :return: None
        """
        if vehicle.id in self.idVehicleMap: raise("duplicated vehicle id")
        self.idVehicleMap[vehicle.id] = vehicle

    def updateLanes(self):
        """
        This function update
        1) lane.countCpu for ALL LANE;
        2) add countCpu for vehicle.currentLane
        :return: None
        """
        for lane in self.idLaneMap.values():
            lane.countPcu = 0
        for vehicle in [ v for v in self.idVehicleMap.values() if v.isRunning(self.ts) ]:
            pcu = {"car": 1.0, 0: 1.0, "bus": 3.5, 1: 3.5, "truck": 3.5, 2: 3.5}[vehicle.type]
            vehicle.currentLane.countPcu += pcu
        for lane in self.idLaneMap.values():
            lane.updatePropertiesBasedOnPcu()
            #print('The current lane is:', lane.id, ';lane type:',lane.type, ';count PCU:', lane.countPcu)

    def runningVehicleCount(self):
        """
        This function counts ALL running vehicles in current time
        :return: number of vehicle
        """
        return len([None for v in self.idVehicleMap.values() if v.isRunning(self.ts)])

    def finishVehicleCount(self):
        """
        This function counts ALL finished vehicles in current time
        :return: number of vehicle
        """
        return len([None for v in self.idVehicleMap.values() if v.isFinish(self.ts)])

    # cross
    def findCross(self):
        countNode = {}
        for link in self.idLinkMap.values():
            if link.node1 not in countNode:
                countNode[link.node1] = 1
            else:
                countNode[link.node1] += 1

        cross = []
        for node in countNode.keys():
            if countNode[node] == 4:
                cross.append(node)
        return cross

    def findCross_(self):
        countNode = {}
        for link in self.idLinkMap.values():
            if link.node1 not in countNode: countNode[link.node1] = 0
            countNode[link.node1] += 1
        return [node for node in countNode.keys() if countNode[node] == 4]