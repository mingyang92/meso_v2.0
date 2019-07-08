from util.readNetwork import *
from util.readOd import *
from model.network import Network
from model.turn import *
from lib.astar import *
import time, datetime
import copy
import random
import csv
import json
import matplotlib.pyplot as plt

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
        'laneType': ve.laneType, 'currentLane_id': ve.currentLane.id, 'currentLaneProgress': ve.currentLaneProgress,
        'timeBudget': ve.timeBudget, 'changeLane': ve.change_lane}

def serializeNetwork(network):
    return {'ts': timestamp(network.ts), 'lanes': { l.id: serializeLane(l) for l in network.idLaneMap.values() },
        'vehicles': { v.id: serializeVehicle(v) for v in network.idVehicleMap.values() if v.isRunning(network.ts)}}

# set the location of the output results

#for checking the simulation logs
#outfile = open(r"C:\Users\lyy90\OneDrive\Documents\GitHub\twinwell\twinwell\outfile_process_during_simulation.csv","w")
#writer_log = csv.DictWriter(outfile)

#for storing the status of lanes and vehicles

#outfile_lane_features = open(r'C:\Users\lyy90\OneDrive\Documents\GitHub\twinwell\twinwell\result\outfile_lane_features_uniform.csv', "w", newline='')
#lane_fieldnames = ['ts', 'id', 'density', 'travel time', 'count PCU']
#writer_lane_features = csv.DictWriter(outfile_lane_features, fieldnames=lane_fieldnames)
#writer_lane_features.writeheader()

#outfile_veh_features = open(r'C:\Users\lyy90\OneDrive\Documents\GitHub\twinwell\twinwell\result\outfile_veh_features.csv',
#                            "w",newline='')
#vehicle_filenames = ['ts', "veh-id", "veh-type", "value-time", "lane change prob", "origin",
#                                              "destination", "start-time","end-time"]
#writer_veh_features = csv.DictWriter(outfile_veh_features, fieldnames=vehicle_filenames)
#writer_veh_features.writeheader()

#outfile_OD_features = open(r'C:\Users\lyy90\OneDrive\Documents\GitHub\twinwell\twinwell\outfile_OD_features.csv',"w")
#writer_OD_features = csv.DictWriter(outfile_OD_features)

#outfile_statistic1 = open(r'C:\Users\lyy90\OneDrive\Documents\GitHub\twinwell\twinwell\
#                           outfile_statistics_along_timestamps.csv',"w")
#writer_statistic_along_timestamps = csv.DictWriter(outfile_statistic1)

#outfile_statistic2 = open(r'C:\Users\lyy90\OneDrive\Documents\GitHub\twinwell\twinwell\
#                              outfile_statistics_whole_simulation.csv',"w")
#writer_statistic_whole_simulation = csv.DictWriter(outfile_statistic2)

# calculation time
calculationStart = time.clock()

startTs = datetime.datetime(2019, 1, 1, 7, 0, 0)
totalSteps = 15000 #2500
timeStep = 1

jamDensity = 124
medianValueTime = 50
random.seed(10)

#writer_log.writerow('This simulation is for:', 'delay type:', delayingType, 'vehicle generation:', genVehicle)

vehicleId = 0
GEN_VEH_DIST = 'normal_whole' # ["uniform", "random", "random_whole", "normal_whole"]
STRATEGY = 'vol_sim' # ['vol_sim', 'vol_dist', 'random', 'fix']
MULTIVEH = 1 #[default=1, 2, 3,...]
NO_CHARGE = True

network = Network(startTs)

#fNode = open("C:/Users/lyy90/OneDrive/Documents/GitHub/meso_v2.0/Sioux Falls network/nodes-SiouxFalls_gong.csv")
fNode = open("F:/meso_v2.0/Sioux Falls network/nodes-SiouxFalls_gong.csv")
fNode.readline()
#fLane = open("C:/Users/lyy90/OneDrive/Documents/GitHub/meso_v2.0/Sioux Falls network/lanes-SiouxFalls_gong.csv")
fLane = open("F:/meso_v2.0/Sioux Falls network/lanes-SiouxFalls_gong.csv")
fLane.readline()
pOd = "F:/meso_v2.0/OD_data"
#pOd = 'C:/Users/lyy90/OneDrive/Documents/GitHub/meso_v2.0/OD_data'

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
countTime = 5
FILE_NUMBER = 0

for i in range(totalSteps):
    network = networks[-1] # current network
    print('This is step', i, 'in', totalSteps, 'and current time is:', network.ts)
    #break
    # update best route
    for vid in network.idVehicleMap:
        vehicle = network.idVehicleMap[vid]
        #print('vehicle.isRunning(network.ts):',vehicle.isRunning(network.ts))
        if not vehicle.isRunning(network.ts): continue

        vehicle.updateShortestPath() #TODO: add vid as an parameter in the updating process
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
        # todo: check this function
        vehicle.updateLocation(1, delayType=STRATEGY) #update for 1 SECOND!
        vehicle.changeLane(dictTimeCost[vehicle.id], 10, medianValueTime, countTime, NO_CHARGE)
        #print(vehicle.id, 'The current lane is:',vehicle.laneType)

    # decision

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

    countTime += 1

    # output features of lanes along the time stamps
    '''
    for lane_id in network.idLaneMap:
        writer_lane_features.writerow({'ts': i,
                                       'id':network.idLaneMap[lane_id].id,
                                       'density': network.idLaneMap[lane_id].density,
                                       'travel time':network.idLaneMap[lane_id].travelTime})  
    for veh_id in network.idVehicleMap:
        writer_veh_features.writerow({'ts': i,
                                      "veh-id": network.idVehicleMap[veh_id].id,
                                      "veh-type": network.idVehicleMap[veh_id].type,
                                      "value-time": network.idVehicleMap[veh_id].valueTime,
                                      "lane change prob": network.idVehicleMap[veh_id].probLaneChange,
                                      "origin": network.idVehicleMap[veh_id].nodeOrigin,
                                      "destination": network.idVehicleMap[veh_id].nodeDest,
                                      "start-time": network.idVehicleMap[veh_id].startTs,
                                      "end-time": network.idVehicleMap[veh_id].finishTs})
        '''



elapsed = (time.process_time() - calculationStart)
print('The total calculation time is:', elapsed, 'seconds')

'''
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
        'laneType': ve.laneType, 'currentLane_id': ve.currentLane.id, 'currentLaneProgress': ve.currentLaneProgress,
        'timeBudget': ve.timeBudget, 'changeLane': ve.change_lane}

def serializeNetwork(network):
    return {'ts': timestamp(network.ts), 'lanes': { l.id: serializeLane(l) for l in network.idLaneMap.values() },
        'vehicles': { v.id: serializeVehicle(v) for v in network.idVehicleMap.values() if v.isRunning(network.ts)}}

output = {}
output["nodes"] = { n.id: serializeNode(n) for n in network.idNodeMap.values()}
output["links"] = { l.id: serializeLink(l) for l in network.idLinkMap.values()}
output["networks"] = []
for network in networks:
    output["networks"].append(serializeNetwork(network))



f_path = 'visualization/'+GEN_VEH_DIST+'_'+STRATEGY+str(MULTIVEH)+'_output.json'
f = open(f_path, "w")
#f.write("networkData = ")
f.write(json.dumps(output))
'''

#plt.figure()
#plt.plot(x_time, y_running, color = 'black', label='running vehicle')
#plt.plot(x_time, y_stopping, color='red', label='finish vehicle', linestyle = '--')
#plt.ylabel('Number of vehicle')
#plt.xlabel('Time')
#plt.show()

