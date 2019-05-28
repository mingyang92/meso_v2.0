class Turn(object):
    def __init__(self, node2, node3, node1, network):
        self.node2 = node2
        self.node3 = node3
        self.node1 = node1

        self.network = network
        self.registerTurn = registerTurn()

        self.stress = None
        self.turn = None
        self.stressMap = None

    def calculateDirection(self, point1, point2, checkPoint):
        '''
        This function currently nor in use
        This function calculates is at the left side or the right side of line
        This line is connected by point1 and point2
        Direction: point1 to point2
        :param point1: start point (Cartesian coordinates)
        :param point2: end point (Cartesian coordinates)
        :param checkPoint: the point you want to figure out (Cartesian coordinates)
        :return: tmp (num), direction (str)
        '''

        x1 = 1.0 * point1.coorX
        y1 = 1.0 * point1.coorY
        x2 = 1.0 * point2.coorX
        y2 = 1.0 * point2.coorY
        x = 1.0 * checkPoint.coorX
        y = 1.0 * checkPoint.coorY

        direction = ""
        tmp = (y1 - y2) * x + (x2 - x1) * y + x1 * y2 - x2 * y1
        if tmp > 0:  # point is at left side of the line
            direction = "left"
        elif tmp < 0:  # point is at the right side of the line
            direction = "right"
        else:  # the point is on the line
            direction = "on"
        return tmp, direction

    def calculateDegress(self, node1id, node2id):
        '''
        This function checks the position of the ray and adjusts the result calculated by arctan value of delta_y/delta_x
        Node1 is set as the origin at a Cartesian system.
        The degree of ray is the angle between ray and the positive horizon axis
        :param p1: node1 id (input point has the same features as node in the road network.)
        :param p2: node2 id (input point has the same features as node in the road network.)
        :param dic_nodes:
        :return: the degree of of ray (from 0 to 2pi) with starting point of node1 and end point of node2;
        '''

        node1 = self.network.idNodeMap[node1id]
        node2 = self.network.idNodeMap[node2id]

        x1 = float(node1.x)
        y1 = float(node1.y)
        x2 = float(node2.x)
        y2 = float(node2.y)

        degree = 0  # initialize the output value
        delta_y = y2 - y1
        delta_x = x2 - x1

        if delta_y >= 0 and delta_x > 0:  # Quadrant I
            degree = atan(delta_y / delta_x)
        elif delta_y >= 0 and delta_x < 0:  # Quadrant II
            degree = pi + atan(delta_y / delta_x)
        elif delta_y <= 0 and delta_x < 0:  # Quadrant III
            degree = pi + atan(delta_y / delta_x)
        elif delta_y <= 0 and delta_x > 0:  # Quadrant Iv
            degree = 2 * pi + atan(delta_y / delta_x)
        elif delta_y > 0 and delta_x == 0:
            degree = pi / 2.0
        elif delta_y < 0 and delta_x == 0:
            degree = 3.0 * pi / 2.0
        else:
            print("error when calculating the degree")
        return degree


    def calculateTurnDegreeDirection(self, laneid12, laneid23):
        '''
        This function returns the direction changed when a vehicle moving from lane12 to lane23
        These two lanes are connected by intersection of node2.
        :param laneid12:  from lane
        :param laneid23: to lane
        :return: pi to pi,
                the positive means left turn while the negative means right turn.
                zero means strict straight and slightly moving left or right is not treated as through traffic or straight.
        '''

        node11 = self.network.idLaneMap[laneid12].link.node1  # id of node1 of lane1
        node12 = self.network.idLaneMap[laneid12].link.node2  # id of node2 of lane1
        node21 = self.network.idLaneMap[laneid23].link.node1  # id of node1 of lane2
        node22 = self.network.idLaneMap[laneid23].link.node2  # id of node2 of lane2

        turned_degree = 0  # initialize the turned_degree

        if node12 != node21:
            print("error, two lanes is not connected by the same node")
        else:
            degree12 = self.calculateDegress(node11, node12)
            degree23 = self.calculateDegress(node21, node22)
            turned_degree = degree23 - degree12
            # if the output degree difference is more than 2pi or less than -2pi
            # we need to minus 2pi or plus 2pi to make sure the result is from -pi to pi.
            if turned_degree > pi:
                turned_degree = turned_degree - 2 * pi
            elif turned_degree < -pi:
                turned_degree = turned_degree + 2 * pi
            if turned_degree == pi or turned_degree == -pi:
                if rule_of_road == "Left":
                    turned_degree = -pi
                elif rule_of_road == "Right":
                    turned_degree = pi
        return turned_degree


    def generateTurn(self):
        """
        This function returns a dictionary of turn and degree of each pair of lanes
        (one inbound and one outbound of an intersection) at each intersection( or node);
        Inbound lane and outbound lane can be different types of lanes(slow lane and fast lane);
        Roundabout is not applicable.
        Laneid1 and laneid2 must be two of the legs joint at intersection of this node
        :return: turn[lane1][lane2]={'degree': value,
                                    'direction': left/right/through/U-turn,
                                    'abs_degree': degree}
        """
        turn = {}

        for node in self.network.idNodeMap.keys():  # at intersection of nodeid
            print('The node id in network is:',node)
            nodeStartLaneList = []  # store the lane id which starts from nodeid
            nodeEndLanelist = []  # store the lane id which ends at nodeid

            # check and assign the lane-ids to the above two lists
            for lane in self.network.idLaneMap.keys():
                if self.network.idLaneMap[lane].link.node1 == nodeid:  # laneid starts from nodeid
                    if lane not in nodeStartLaneList:
                        nodeStartLaneList.append(lane)
                    else:
                        print("ERROR, same lanes occur")
                if self.network.idLaneMap[lane].link.node2 == nodeid:  # laneid ends at nodeid
                    if lane not in nodeEndLanelist:
                        nodeEndLanelist.append(lane)
                    else:
                        print("ERROR, same lanes occur")

            # calculate the values of dic_turns
            for lane1 in nodeEndLanelist:  # laneid1 ends at nodeid
                lane1 = self.network.idLaneMap[lane1]
                turn[lane1] = {}

                for lane2 in nodeStartLaneList:  # laneid2 starts from nodeid
                    lane2 = self.network.idLaneMap[lane2]
                    turned_degree = self.calculateTurnDegreeDirection(lane1, lane2)

                    direction = ""
                    if turned_degree > 0:
                        direction = "left"
                        if turned_degree == pi:
                            direction = "U-turn"
                    elif turned_degree < 0:
                        direction = "right"
                        if turned_degree == -pi:
                            direction = "U-turn"
                    else:
                        direction = "through"
                    turn[lane1][lane2] = {'degree': turned_degree, 'direction': direction, 'abs_degree': abs(
                        turned_degree)}  # turned-degree is a value with positive or negative sign
        return turn


    def calculateStressFromTurn(self, direction, turned_degree):
        '''
        This function quantifies the stress from the turning direction and degree turned of a vehicle
        at an intersection (or node)
        :param direction: direction turned
        :param turned_degree: degree turned
        :return: stress value: num
        '''

        # 1. U-turn is assumed to have the largest stress, while right-turn has a larger stress compared to
        # left-turn at a same degree in case of left-hand traffic;
        # 2. A linear function between stress and degree turned is used currently, as no existing research results
        # regarding this relationship has been found;
        # 3. absolute value of degree turned is used in the linear function, and additional parameters
        # for U-turn is 3, right-turn is 2, and left-turn is 1, in case of left-hand traffic.

        # decide the parameter in the linear function
        if direction == "U-turn":
            para = 3
        elif direction == "through":
            para = 0
        elif direction == "left" and rule_of_road == "Left":
            para = 1
        elif direction == "left" and rule_of_road == "Right":
            para = 2
        elif direction == "right" and rule_of_road == "Left":
            para = 2
        elif direction == "right" and rule_of_road == "Right":
            para = 1
        else:
            print("Error when quantifying stress")
        stress = para * float(turned_degree)
        # return {'stress':stress,'direction':dire,'turned-degree':turned_degree}
        return stress


    def stressMap(self, turn):
        '''
        This function converts turn degree and turn direction between each lane pair to calculate stress;
        The output graph of stress is used in the stress minimization based route search；
        Note： node1, node11,node12 are the node ids which are upstream node ids of lanes connected to
                lane from node2 to node3 at node2；
        :param turn: dictionary of turns from the function of TurnGen
        :return: graph_stress1[node2][node3] = {node1: stress-value1,
                                            node11: stress-value11,
                                            node12: stress-value12...}
        '''
        #########################################################################
        ## so the formats of the output graphs are consistent in the Dijkstra4 ##
        # two structures, corresponding to G_stress1 and G_stress2 in Dijkstra4 #
        #########################################################################

        # two steps.
        # 1st step, convert the dictionary of turns whose key is (lane id and lane id) to (node id and node id and node id).
        # 2nd step, calculate the stress by the function of Cal_Stress_from_turn

        # e.g., two lanes connected by a node whose id is 2. lane1 is from node1 to node2,
        # while lane2 is from node2 to node3;
        # it means that the stress of lane2 is not a fixed value, depending on the previous
        # connected lane which decides the direction turned and degree turned.

        dic_graph_stress1 = {}  # initialize the graph based on stress, structure 1
        dic_graph_stress2 = {}  # initialize the graph based on stress, structure 2

        for lane1 in turn.keys():
            node1 = self.network.idLaneMap[lane1].link.node1
            dic_graph_stress2[lane1] = {}

            for lane2 in turn[lane1].keys():
                direction = turn[lane1][lane2]['direction']
                turned_degree = turn[lane1][lane2]['abs_degree']
                node2 = self.network.idLaneMap[lane2].link.node1
                node3 = self.network.idLaneMap[lane2].link.node2
                stress = self.calculateStressFromTurn(direction, turned_degree)

                # write dic_graph_stress2
                if lane2 in dic_graph_stress2[lane1].keys():
                    if dic_graph_stress2[lane1][lane2] != stress:
                        print("stress is not the same for the same turnings")
                else:
                    dic_graph_stress2[lane1][lane2] = stress

                # write dic_graph_stress1
                if node2 in dic_graph_stress1:
                    if node3 in dic_graph_stress1[node2]:
                        if node1 in dic_graph_stress1[node2][node3]:  # check if the quantified stress are the same
                            if stress != dic_graph_stress1[node2][node3][node1]:
                                print("error: same direction at same node but different quantified stress")
                        else:
                            dic_graph_stress1[node2][node3][node1] = stress
                    else:
                        dic_graph_stress1[node2][node3] = {}
                        dic_graph_stress1[node2][node3][node1] = stress
                else:
                    dic_graph_stress1[node2] = {}
                    dic_graph_stress1[node2][node3] = {}
                    dic_graph_stress1[node2][node3][node1] = stress

        return dic_graph_stress1, dic_graph_stress2


