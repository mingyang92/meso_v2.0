from util.readNetwork import *
from util.readOd import *
from model.network import Network
from model.turn import *
import time, datetime
import copy
import random
import csv
import json


# calculation time
calculationStart = time.clock()

startTs = datetime.datetime(2019, 1, 1, 7, 0, 0)
totalSteps = 20000 #2500
timeStep = 1
jamDensity = 124
medianValueTime = 50
random.seed(10)

vehicleId = 0
GEN_VEH_DIST = 'uniform_whole' # ["uniform", "random", "random_whole", "normal_whole"]
STRATEGY = 'vol_sim' # ['vol_sim', 'vol_dist', 'random', 'fix']
MULTIVEH = 1 #[default=1, 2, 3,...]
NO_CHARGE = False

network = Network(startTs)

fNode = open("C:/Users/lyy90/OneDrive/Documents/GitHub/meso_v2.0/Sioux Falls network/nodes-SiouxFalls_gong.csv")
#fNode = open("F:/meso_v2.0/Sioux Falls network/nodes-SiouxFalls_gong.csv")
fNode.readline()
fLane = open("C:/Users/lyy90/OneDrive/Documents/GitHub/meso_v2.0/Sioux Falls network/lanes-SiouxFalls_gong.csv")
#fLane = open("F:/meso_v2.0/Sioux Falls network/lanes-SiouxFalls_gong.csv")
fLane.readline()
pOd = 'C:/Users/lyy90/OneDrive/Documents/GitHub/meso_v2.0/OD_data'
#pOd = "F:/meso_v2.0/OD_data"

readNodes(fNode, network)
readLanes(fLane, network)
tsPairNodePairTypeMap = readOd(pOd)
#print(tsPairNodePairTypeMap)
genVehicle(tsPairNodePairTypeMap, GEN_VEH_DIST, vehicleId, medianValueTime, network, MULTIVEH)
t = sorted([vehicle.startTs for vehicle in network.idVehicleMap.values()])


with open(GEN_VEH_DIST+'output.csv', mode='w', newline='') as csv_file:
    fieldnames = ['vid', 'start_time']
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()
    for vid in network.idVehicleMap:
        timestamp = time.mktime(network.idVehicleMap[vid].startTs.timetuple()) - 1546293600
        writer.writerow({'vid': vid, 'start_time': timestamp})