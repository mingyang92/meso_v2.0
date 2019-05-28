class Link(object):
    # as it is lane, it has only one direction; this is the different part from link
    def __init__(self, id, type, node1, node2, network):
        self.id = id
        self.type = type
        self.node1 = node1
        self.node2 = node2

        self.network = network
        self.network.registerLink(self)

        self.lengthInKm = node1.dist(node2) / 1000.0
