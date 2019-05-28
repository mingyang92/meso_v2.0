'''
Created on 2018/07/10

@author: gong
'''
from model.node import Node
from model.link import Link
from model.lane import Lane
import csv
import copy

def readNodes(nodefile, network):
    """
    This function reads NODE from Nodefile in network
    :param nodefile: path
    :param network:
    :return: Node
    """
    f = csv.reader(nodefile)
    for row in f:
        (id, type, x, y) = (row[0], row[1], row[3], row[4])
        Node(id, type, x, y, network)

def readLanes(lanefile, network):
    """
    This function reads LANE from lanefile in network
    :param lanefile:
    :param network:
    :return: Lane
    """
    f = csv.reader(lanefile)
    for row in f:
        linkId = row[4]
        if linkId in network.idLaneMap:
            link = network.idLinkMap[linkId]
        else:
            link = Link(row[4], row[5], network.idNodeMap[row[2]], network.idNodeMap[row[3]], network)
        Lane(row[1], row[0], link, row[6], row[7], row[8], None, network)