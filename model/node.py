class Node(object):
    def __init__(self, id, type, x, y, network):
        self.id = id
        self.type = type
        self.x = float(x)
        self.y = float(y)
        self.network = network
        self.hCost = 20


        self.network.registerNode(self)

    def __repr__(self):
        return "<" + " ".join(["node" + self.id, self.type, str(self.x), str(self.y), str(self.hCost)]) + ">"

    def dist(self, node):
        """
        This function is to calculate distance between current point with given NODE
        :param node: given NODE
        :return: distance
        """
        return ((self.x - node.x) ** 2 + (self.y - node.y) ** 2) ** 0.5

    def manhattanDist(self, node):
        '''
        This function calculates the Manhattan distance between current point to given point
        :param node:
        :return:
        '''
        return abs(self.x - node.x) + abs(self.y - node.y)
