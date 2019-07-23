from util.readNetwork import *
from util.readOd import *
from model.network import Network
from model.turn import *
import time, datetime
import copy
import random
import csv
import json

def timestamp(dt):
    return dt.timestamp() if dt else None

def serializeNode(node):
    return {'id': node.id, 'type': node.type, 'x': node.x, 'y': node.y}

def serializeLink(link):
    return {'id': link.id, 'type': link.type, 'node1_id': link.node1.id, 'node2_id': link.node2.id,
        'lengthInKm': link.lengthInKm}

def serializeLane(lane):
    return {'id': lane.id, 'type': lane.type, 'link_id': lane.link.id,
        'freeSpeed': lane.freeSpeed, 'freeTravelTime': lane.freeTravelTime, 'fixedCharge': lane.fixedCharge,
        'speed': lane.speed, 'countPcu': lane.countPcu, 'density': lane.density, 'travelTime': lane.travelTime,
        'charge': lane.charge}

def serializeVehicle(ve):
    return {'id': ve.id, 'type': ve.type, 'driverType': ve.driverType, 'maxSpeed': ve.maxSpeed,
        'valueTime': ve.valueTime, 'probLaneChange': ve.probLaneChange, 'startTs': timestamp(ve.startTs),
        'nodeOrigin_id': ve.nodeOrigin.id, 'nodeDest_id': ve.nodeDest.id, 'finishTs': timestamp(ve.finishTs),
        'laneType': ve.laneType, 'currentLane_id': ve.currentLane.id if ve.currentLane else None, 'currentLaneProgress': ve.currentLaneProgress,
        'timeBudget': ve.timeBudget, 'changeLane': ve.change_lane}

def serializeNetwork(network):
    return {'ts': timestamp(network.ts), 'lanes': { l.id: serializeLane(l) for l in network.idLaneMap.values() },
        'vehicles': { v.id: serializeVehicle(v) for v in network.idVehicleMap.values() if v.isRunning(network.ts)},
        'endVehicles': { v.id: serializeVehicle(v) for v in network.idVehicleMap.values() if v.isFinish(network.ts)} }


# calculation time
calculationStart = time.clock()

startTs = datetime.datetime(2019, 1, 1, 7, 0, 0)
totalSteps = 20000 #2500
timeStep = 1
jamDensity = 124
medianValueTime = 50
random.seed(10)

vehicleId = 0
GEN_VEH_DIST = 'normal_whole' # ["uniform", "random", "random_whole", "normal_whole"]
STRATEGY = 'vol_sim' # ['vol_sim', 'vol_dist', 'random', 'fix']
MULTIVEH = 2 #[default=1, 2, 3,...]
NO_CHARGE = False

network = Network(startTs)

#fNode = open("C:/Users/lyy90/OneDrive/Documents/GitHub/meso_v2.0/Sioux Falls network/nodes-SiouxFalls_gong.csv")
fNode = open("F:/meso_v2.0/Sioux Falls network/nodes-SiouxFalls_gong.csv")
fNode.readline()
#fLane = open("C:/Users/lyy90/OneDrive/Documents/GitHub/meso_v2.0/Sioux Falls network/lanes-SiouxFalls_gong.csv")
fLane = open("F:/meso_v2.0/Sioux Falls network/lanes-SiouxFalls_gong.csv")
fLane.readline()
#pOd = 'C:/Users/lyy90/OneDrive/Documents/GitHub/meso_v2.0/OD_data'
pOd = "F:/meso_v2.0/OD_data"

readNodes(fNode, network)
readLanes(fLane, network)
tsPairNodePairTypeMap = readOd(pOd)
#print(tsPairNodePairTypeMap)
genVehicle(tsPairNodePairTypeMap, GEN_VEH_DIST, vehicleId, medianValueTime, network, MULTIVEH)
t = sorted([vehicle.startTs for vehicle in network.idVehicleMap.values()])

for vid in network.idVehicleMap:
    print(vid, network.idVehicleMap[vid].startTs)

networks = [network]

y_running = []
y_stopping = []
x_time = list(range(totalSteps))
dictTimeCost = {}

countTime = 5 # 到路口前提前多少秒做判断
FILE_NUMBER = 0
GAP_TS = 10

for i in range(totalSteps):
    network = networks[-1] # current network
    print('This is step', i, 'in', totalSteps, 'and current time is:', network.ts)
    #break
    # update best route
    for vid in network.idVehicleMap:
        vehicle = network.idVehicleMap[vid]
        #print('vehicle id:', vid)
        if not vehicle.isRunning(network.ts): continue
        if (network.ts-vehicle.startTs).seconds % GAP_TS != 0:
            continue
        vehicle.updateShortestPath()
        #print('This current lane is:', vehicle.currentLane)
    # update lane features
    network.updateLanes()

    # update vehicle location
    for vehicle in network.idVehicleMap.values():
        # calculate the dictTimeCost at the startTs for every vehicle
        if vehicle.startTs == network.ts:
            dictTimeCost[vehicle.id] = 0
            for laneid in vehicle.bestLaneRoute.keys():
                dictTimeCost[vehicle.id] += network.idLaneMap[laneid].travelTime
            #print('dictTimeCost[vehicle]1',dictTimeCost[vehicle])

        if not vehicle.isRunning(network.ts): continue
        vehicle.updateLocation(1, delayType=STRATEGY) #update location in the lane for 1 SECOND!
        if (network.ts-vehicle.startTs).seconds % GAP_TS == 0:
            vehicle.changeLane(dictTimeCost[vehicle.id], 10, medianValueTime, countTime, NO_CHARGE)

        # print the vehicle information when it takes too long to finish its trip
        if (network.ts - vehicle.startTs).seconds > 1000:
            print("ts: ",(network.ts - vehicle.startTs).seconds)
            print("vehicle: ", vehicle)
            print("route: ", vehicle.bestLaneRoute)

    print(network.runningVehicleCount(), 'running')
    print(network.finishVehicleCount(), 'finished')
    y_running.append(network.runningVehicleCount())
    y_stopping.append(network.finishVehicleCount())


    if (i > 10 and i % 1000 == 0) or (network.runningVehicleCount() == 0 and network.finishVehicleCount()>1000):
        # this part for output JSON file
        print("writing json file....")
        output = {}
        output["nodes"] = {n.id: serializeNode(n) for n in network.idNodeMap.values()}
        output["links"] = {l.id: serializeLink(l) for l in network.idLinkMap.values()}
        output["networks"] = []
        for network in networks:
            output["networks"].append(serializeNetwork(network))

        #f_path = 'visualization/' + GEN_VEH_DIST + '_' + STRATEGY \
        #        + str(MULTIVEH) + '_' + str(NO_CHARGE) + '_' + str(FILE_NUMBER) + '_output.json'
        f_path = 'F:/meso_v2.0/visualization/' + GEN_VEH_DIST + '_' + STRATEGY \
                 + str(MULTIVEH) + '_' + str(NO_CHARGE) + '_' + str(FILE_NUMBER) + '_output.json'

        f = open(f_path, "w")
        # f.write("networkData = ")
        f.write(json.dumps(output))
        FILE_NUMBER += 1
        networks = [network]

    # copy network to i+1
    network_next = copy.deepcopy(network)
    network_next.ts += datetime.timedelta(seconds=1)
    networks.append(network_next)

    if network.runningVehicleCount() == 0 and network.finishVehicleCount()>1000:
        break

    #countTime += 1

elapsed = (time.process_time() - calculationStart)
print('The total calculation time is:', elapsed, 'seconds')

