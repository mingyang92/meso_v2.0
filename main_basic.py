'''
Version 1.0 : 2018/07/23
Version 2.0 : 2019/04/04

@author: Gong Lei(version 1.0), Li Yanyan (Version 2.0)
The objectives of this project are:
1. test the performance of lane-based toll-collection system;
2. test whether driver-stress based route choice (v2.0);

Conditions:
1.Bus can change to a faster lane without external cost. 
2.Private car changes to a faster lane depending on the value-time, 
	driver's preference type.
3.OD matrices are increasing from 10% to 100% with a step of 10% on 
	both bus and private car.

Output:
1) travel time from each OD pair at each simulation step
   average travel time for bus and private vehicles by OD pair
2) composition of private car's decision : changed lane, not changed lane, 
	in whole simulation period and every 5 minutes
3) density of each lane at each simulation step
4) start-time,end-time,saved-time if changing the lane of each vehicle
'''

import time, datetime
import copy
import random
import csv
import json

# set the rule of the road, can be either "Left" or "Right"
rule_of_road="Left"
def check_position(p0, p1, p2):
    # this function checks the distance between point p0 to the line (connected from p1 to p2).
    # it is used to check whether the vehicle (update location is p0) is following the lane (line from p1 to p2)
    # if the distance is more than a threshold, it means the vehicle did not follow the lane and some error exists in the locate function.
    # input three points are in class of node {'coor-X':coor_X,'coor-Y':coor_Y,....}
    dummy_result = 0

    return dummy_result


def locate(veh_id, curr_timestamp):
    # time_stamp is the current time, yyyy-mm-dd hh:mm:ss
    # this function update the vehicle location at current time
    # in case of receiving continuously instant GPS signal, it returns the map-matched coordinates(matched to the central line of lane (not included in this function yet).
    # in case of a simulated circumstance, the current location is calculated based on the location of previous time stamp and the speed on related links.
    pre_timestamp = get_pre_timestamp(curr_timestamp, timestep)  # previous time stamp
    dummy_vanishing_veh = 0  # an indicator to show at current time stamp whether this vehicle will arrive its destination or not: 0, did not arrive; 1, arrived.
    # print "veh-id, current time, previous time", veh_id,time_stamp,pre_timestamp#****************for checking only
    # print "list-timestamps:", dic_vehicles[veh_id]['list-timestamps']#****************for checking only
    # print "locations are:", dic_vehicles[veh_id]['dic-locations']#****************for checking only
    # print "routes are:", dic_vehicles[veh_id]['dic-routes']#****************for checking only

    pre_location = dic_vehicles[veh_id]['dic-locations'][pre_timestamp]  # location at previous stamp
    pre_route = dic_vehicles[veh_id]['dic-routes'][pre_timestamp]  # best route at previous stamp,lane-based routes

    # two outputs of this function
    curr_location = {}  # initialize the location at current time stamp

    ###---------- calculate the location at current time stamp ------------###
    lane_id = pre_location['lane-id']
    speed = dic_lanes[lane_id]['speed']
    # nodeid1=dic_lanes[lane_id]['node-id1']# node1 of the lane focused on
    nodeid2 = dic_lanes[lane_id]['node-id2']  # node2 of the lane focused on
    dist_to_id2 = Dist(pre_location, dic_nodes[
        nodeid2]) / 1000.0  # in km, make sure each item has a dictionary structure containing keys of "coor-X" and "coor-Y"
    time_to_id2 = dist_to_id2 * 1.0 / float(speed) * 3600.0
    time_to_travel = copy.deepcopy(timestep)  # the time that vehicle can move
    while time_to_travel >= time_to_id2:  ## check until vehicle cannot reach the end of a lane on the best path.
        if nodeid2 == dic_vehicles[veh_id]['destination']:  # this vehicle arrives its destination
            coor_X = dic_nodes[nodeid2]['coor-X']
            coor_Y = dic_nodes[nodeid2]['coor-Y']
            list_van_vehids.append(veh_id)  # put its id into list of vanishing veh-ids
            dic_vehicles[veh_id]['end-timestamp'] = curr_timestamp
            dic_vehicles[veh_id]['dic-speeds'][curr_timestamp] = speed
            # update its location to id2
            curr_location['coor-X'] = coor_X
            curr_location['coor-Y'] = coor_Y
            curr_location['lane-id'] = ''
            dic_vehicles[veh_id]['dic-locations'][curr_timestamp] = curr_location
            dic_vehicles[veh_id]['dic-routes'][curr_timestamp] = ''
            dic_vehicles[veh_id]['list-timestamps'].append(curr_timestamp)
            dummy_vanishing_veh = 1
            break
        else:
            # if true, update lane to the next along the best route at previous time stamp;update the location of vehicle to the start node of updated lane (or end node of previous lane)
            time_to_travel = time_to_travel - time_to_id2
            lane_id = pre_route[
                lane_id]  # update lane to the next lane of current lane along the best route at the previous time stamp
            dic_vehicles[veh_id]['list-laneid-used'].append(lane_id)
            pre_location = {'coor-X': dic_nodes[nodeid2]['coor-X'], 'coor-Y': dic_nodes[nodeid2]['coor-Y'],
                            'lane-id': lane_id}  # update the location of vehicle to the start node of next lane (end node of previous lane)
            # nodeid1=dic_lanes[lane_id]['node-id1']
            nodeid2 = dic_lanes[lane_id]['node-id2']
            speed = dic_lanes[lane_id]['speed']
            dist_to_id2 = Dist(pre_location, dic_nodes[nodeid2]) / 1000.0
            time_to_id2 = dist_to_id2 * 1.0 / float(speed) * 3600
    if dummy_vanishing_veh == 0:
        # now veh-id cannot arrive node-id2 at the end of this simulation step
        dic_vehicles[veh_id]['dic-speeds'][
            curr_timestamp] = speed  # !!!!!!!!!!if the time step is long enough for vehicle to pass several links, this is the speed of the final link
        # if speed<=1:
        # speed=1# set the min speed for traveling to avoid dead congestion
        X1 = pre_location['coor-X']  # coordinate X of previous location
        Y1 = pre_location['coor-Y']  # coordinate Y of previous location
        X2 = dic_nodes[nodeid2]['coor-X']  # coordinate X of end node of focused lane
        Y2 = dic_nodes[nodeid2]['coor-Y']  # coordinate Y of end node of focused lane
        coor_X = X1 + (X2 - X1) * time_to_travel / 3600.0 * speed * 1000.0 / Dist(dic_nodes[nodeid2],
                                                                                  pre_location)  # in meter, coordinate X of current position of vehicle
        coor_Y = Y1 + (Y2 - Y1) * time_to_travel / 3600.0 * speed * 1000.0 / Dist(dic_nodes[nodeid2],
                                                                                  pre_location)  # in meter, coordinate Y of current position of vehicle
        # cur_location={'coor-X':coor_X,'coor-Y':coor_Y,'lane-id':lane_id}
        curr_location['coor-X'] = coor_X
        curr_location['coor-Y'] = coor_Y
        curr_location['lane-id'] = lane_id
        # update the dictionaries of location and list of time stamps
        dic_vehicles[veh_id]['list-timestamps'].append(curr_timestamp)
        dic_vehicles[veh_id]['dic-locations'][curr_timestamp] = curr_location
        ###------------- best route is updated again if it decides to change lanes -----------------###
        # best route of current time stamp is set as the same as previous time stamp
        dic_vehicles[veh_id]['dic-routes'][curr_timestamp] = copy.deepcopy(pre_route)
    return ()


def densityspeed(density, freespeed):
    # this function returns the speed on a lane when inputting a density
    # negative linear relationship between density and speed is used
    # additional parameters of free speed and jam  density are needed.

    if density >= jamdensity:  # to avoid getting 0 or minus speed
        speed = 0.001  # km/h
    else:
        speed = freespeed * (1.0 - 1.0 * density / jamdensity)

    return speed


def updatelanes(curr_timestamp):
    # this function update features of all lanes in the dic_lanes, dic_graph_low and dic_graph_high at a time stamp.
    # the updated features include counts (the number of vehicles, pcu).
    # then the density, speed, and travel time are updated according to the counts.

    # initialize the counts on each lane is 0
    for lane_id in dic_lanes.keys():
        dic_lanes[lane_id]['counts'] = 0
        id1 = dic_lanes[lane_id]['node-id1']
        id2 = dic_lanes[lane_id]['node-id2']
        lane_type = dic_lanes[lane_id]['lane-type']
        if lane_type == 0:
            dic_graph_low[id1][id2]['counts'] = 0
        elif lane_type == 1:
            dic_graph_high[id1][id2]['counts'] = 0
        else:
            print('sth wrong when initialize counts on the lanes')

    # check which lane each running vehicle is on and count them
    for veh_id in list_running_vehids:
        # check the vehicle type
        veh_type = dic_vehicles[veh_id]['veh-type']
        pcu = 0
        if veh_type == 0 or veh_type == 'car':  # car
            pcu = 1.0
        elif veh_type == 1 or veh_type == 'bus':  # bus
            pcu = 3.5
        elif veh_type == 2 or veh_type == 'truck':  # truck
            pcu = 3.5
        else:
            print ("ERROR, no corresponding pcu value to the vehicle type")
        lane_id = dic_vehicles[veh_id]['dic-locations'][curr_timestamp][
            'lane-id']  # lane id where this running vehicle is on
        dic_lanes[lane_id]['counts'] += pcu
        lane_type = dic_lanes[lane_id]['lane-type']
        id1 = dic_lanes[lane_id]['node-id1']
        id2 = dic_lanes[lane_id]['node-id2']
        if lane_type == 0:
            dic_graph_low[id1][id2]['counts'] += pcu
        elif lane_type == 1:
            dic_graph_high[id1][id2]['counts'] += pcu
        else:
            print("sth wrong when counting the vehicles on the lanes")
    # update the density and speed on each lane based on the counts on the lane
    for lane_id in dic_lanes.keys():
        # update density, speed, travel-time in dic_lanes
        density = dic_lanes[lane_id]['density'] = dic_lanes[lane_id]['counts'] / dic_lanes[lane_id][
            'length']  # density=counts/length, unit is pcu/km
        speed = dic_lanes[lane_id]['speed'] = densityspeed(density, dic_lanes[lane_id][
            'free-speed'])  # three input, real-time density, free-speed, jam density, unit is km/h
        travel_time = dic_lanes[lane_id]['travel-time'] = dic_lanes[lane_id]['length'] * 3600.0 / dic_lanes[lane_id][
            'speed']  # real-time travel-time=length/real-time speed, unit is sec
        dic_lanes[lane_id]['dic-speeds'][curr_timestamp] = speed
        dic_lanes[lane_id]['dic-counts'][curr_timestamp] = dic_lanes[lane_id]['counts']
        dic_lanes[lane_id]['dic-densities'][curr_timestamp] = density
        # obtain lane type and id1 and id2 to update the lane features in the other two lane dictionaries
        lane_type = dic_lanes[lane_id]['lane-type']
        id1 = dic_lanes[lane_id]['node-id1']
        id2 = dic_lanes[lane_id]['node-id2']
        # update density,speed,travel-time in two other lane dictionaries
        if lane_type == 0:
            dic_graph_low[id1][id2]['density'] = density
            dic_graph_low[id1][id2]['speed'] = speed
            dic_graph_low[id1][id2]['travel-time'] = travel_time
            dic_graph_low[id1][id2]['dic-speeds'][curr_timestamp] = speed
            dic_graph_low[id1][id2]['dic-counts'][curr_timestamp] = copy.deepcopy(dic_lanes[lane_id]['counts'])
            dic_graph_low[id1][id2]['dic-densities'][curr_timestamp] = density
        elif lane_type == 1:
            dic_graph_high[id1][id2]['density'] = density
            dic_graph_high[id1][id2]['speed'] = speed
            dic_graph_high[id1][id2]['travel-time'] = travel_time
            dic_graph_high[id1][id2]['dic-speeds'][curr_timestamp] = speed
            dic_graph_high[id1][id2]['dic-counts'][curr_timestamp] = copy.deepcopy(dic_lanes[lane_id]['counts'])
            dic_graph_high[id1][id2]['dic-densities'][curr_timestamp] = density
    return ()


def best_route(veh_id, curr_timestamp):
    # this function search the best route for a vehicle at a time stamp, based on the lane features at previous time stamp
    # make sure this function is used after the previous function to obtain the latest travel time.
    curr_location = dic_vehicles[veh_id]['dic-locations'][curr_timestamp]
    lane_id = curr_location['lane-id']
    nodeid1 = dic_lanes[lane_id]['node-id1']
    nodeid2 = dic_lanes[lane_id][
        'node-id2']  # it causes excluding the current lane (and this lanes' ids) in the best route search results. Manually include them in the results.

    results = {}
    D = dic_vehicles[veh_id]['destination']
    # search best route on low-slow lane-network
    D_low, P_low = shortestPathNode(dic_graph_low, nodeid2, D)
    best_route_lanes_l = convert_ppath_to_pathids(P_low, dic_graph_low, nodeid2, D)
    results['best-route-lanes-low'] = best_route_lanes_l
    results['time-to-destination-low'] = D_low[D]
    results['best-route-nodes-low'] = P_low
    # add current lane information into the above results
    next_nodeid = P_low[nodeid2]
    next_laneid = dic_graph_low[nodeid2][next_nodeid]['lane-id']
    results['best-route-nodes-low'][nodeid1] = nodeid2
    results['best-route-lanes-low'][lane_id] = next_laneid

    # search best route on high-fast lane-network
    D_high, P_high = shortestPathNode(dic_graph_high, nodeid2, D)
    best_route_lanes_h = convert_ppath_to_pathids(P_high, dic_graph_high, nodeid2, D)
    results['best-route-lanes-high'] = best_route_lanes_h
    results['time-to-destination-high'] = D_high[D]
    results['best-route-nodes-high'] = P_high
    # add current lane information into the above results
    next_nodeid = P_high[nodeid2]
    next_laneid = dic_graph_high[nodeid2][next_nodeid]['lane-id']
    results['best-route-nodes-high'][nodeid1] = nodeid2
    results['best-route-lanes-high'][lane_id] = next_laneid
    return results


def decision(veh_id, curr_timestamp):
    # this function returns a result of whether to change to a faster and more expensive lane-network at a current time stamp after obtaining its current location.
    # this function is only used when the vehicle is not in the last section to its destination; this is an if-condition in the main program
    chan_lane = 0  # code of whether changing lanes to fast-expensive network: 1 is to change and 0 is not to change
    # start_time=dic_vehicles[veh_id]['start-timestamp']
    expect_endtime = dic_vehicles[veh_id]['expected-endtime']
    # print veh_id,dic_vehicles[veh_id]['start-timestamp'],dic_vehicles[veh_id]['origin'],dic_vehicles[veh_id]['destination'],"current time and expect end time are:", curr_timestamp,expect_endtime
    left_time_budget = Caltime(curr_timestamp, expect_endtime)
    best_route_results = best_route(veh_id, curr_timestamp)
    curr_location = dic_vehicles[veh_id]['dic-locations'][curr_timestamp]
    lane_id = curr_location['lane-id']
    nodeid2 = dic_lanes[lane_id]['node-id2']
    speed = dic_vehicles[veh_id]['dic-speeds'][curr_timestamp]
    needed_time_low = best_route_results['time-to-destination-low'] + Dist(curr_location,
                                                                           dic_nodes[nodeid2]) / 1000.0 / speed * 3600.0
    needed_time_high = best_route_results['time-to-destination-high'] + Dist(curr_location, dic_nodes[
        nodeid2]) / 1000.0 / speed * 3600.0
    # check the left time budget and the value of time to decide the lane-changing probability
    if left_time_budget < needed_time_low:
        if dic_vehicles[veh_id]['value-time'] >= median_value_time:
            dic_vehicles[veh_id]['lane-chan-p'] = dic_vehicles[veh_id]['lane-chan-p'] * 2
            if dic_vehicles[veh_id]['lane-chan-p'] > 1:
                dic_vehicles[veh_id]['lane-chan-p'] = 1
        # conclude the decision based on the lane-changing probability
        if dic_vehicles[veh_id]['lane-chan-p'] >= 0.5:
            chan_lane = 1  # change lane
        else:
            chan_lane = 0  # do not change lane
        if left_time_budget < needed_time_high:
            #######################print("even changing to the faster-expensive lane networks cannot arrive on time")
            chan_lane = 0  # do not change lane; need further study regarding loss
            dic_vehicles[veh_id]['lane-changed'] = [9,
                                                    curr_timestamp]  # changing lane still cannot meet the time budget
    return chan_lane


def lane_change(veh_id, curr_timestamp):
    # update the current-lane id and best route as the vehicle changes to a fast-expensive lane network
    cur_location = dic_vehicles[veh_id]['dic-locations'][curr_timestamp]
    lane_id = cur_location['lane-id']
    id1 = dic_lanes[lane_id]['node-id1']
    id2 = dic_lanes[lane_id]['node-id2']
    fast_lane_id = dic_graph_high[id1][id2]['lane-id']
    low_lane_id = dic_graph_low[id1][id2]['lane-id']
    best_route_results = best_route(veh_id, curr_timestamp)
    best_route_h = best_route_results['best-route-lanes-high']
    # one more pair should be included in the best_route_h, as lane-id-before-changing:next-lane-id has been included while lane-id-after-changing:next-lane-id has not been. the latter should also be included.
    next_lane_id = best_route_h[low_lane_id]
    best_route_h[fast_lane_id] = next_lane_id

    ##################################################
    # lane-change is assumed to be instantly, which means the location and best-route should be updated simultaneously
    # update the location and best route of the vehicle
    dic_vehicles[veh_id]['dic-locations'][curr_timestamp]['lane-id'] = fast_lane_id
    dic_vehicles[veh_id]['dic-routes'][curr_timestamp] = best_route_h
    dic_vehicles[veh_id]['lane-type'] = 1
    dic_vehicles[veh_id]['lane-changed'] = [1, curr_timestamp]
    dic_vehicles[veh_id]['list-laneid-used'].append(fast_lane_id)
    dic_vehicles[veh_id]['dic-speeds'][curr_timestamp] = dic_lanes[fast_lane_id]['speed']
    return ()

from util.readNetwork import *
from util.readOd import *
from model.network import Network
# set the start time for the simulation
startTs = datetime.datetime(2019, 1, 1, 7, 0, 0)
totalSteps = 5 #2000
# one second for one step; total 3600 steps
timeStep = 1

# set jam density, 200 pcu/mile = 124 pcu/km
# ref:https://lost-contact.mit.edu/afs/eos.ncsu.edu/info/ce400_info/www2/flow1.html
jamDensity = 124
medianValueTime = 50
random.seed(10)

# initialize vehicle-id; it is a global variable, as there may be many OD-matrix
# the first vehicle id is 1 and is increased subsequently with a step of 1
vehicleId = 0

network = Network(startTs)

fNode = open("Sioux Falls network/nodes-SiouxFalls_gong.csv")
fNode.readline()
fLane = open("Sioux Falls network/lanes-SiouxFalls_gong.csv")
fLane.readline()
pOd = "OD_data"

readNodes(fNode, network)
readLanes(fLane, network)
tsPairNodePairTypeMap = readOd(pOd)
# five types of vehicle generation method ['uniform', 'uniform_whole', 'random', 'random_whole', 'normal_whole']
genVehicle(tsPairNodePairTypeMap, "uniform", vehicleId, medianValueTime, network)


for vid in network.idVehicleMap:
    print(network.idVehicleMap[vid])

networks = [network]
for i in range(totalSteps):
    #print(i)
    network = networks[-1] # current network

    print(network.ts)

    # update best route
    for vid in network.idVehicleMap:
        vehicle = network.idVehicleMap[vid]
        if not vehicle.isRunning(network.ts): continue

        vehicle.updateShortestPath()

    # update lane features
    network.updateLanes()

    # update vehicle location
    for vehicle in network.idVehicleMap.values():
        if not vehicle.isRunning(network.ts): continue
        vehicle.updateLocation(1) #update for 1 SECOND!

    # decision

    print(network.runningVehicleCount(), 'running')
    print(network.finishVehicleCount(), 'finished')

    # copy network to i+1
    network_next = copy.deepcopy(network)
    network_next.ts += datetime.timedelta(seconds=1)
    networks.append(network_next)

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
        'timeBudget': ve.timeBudget}

def serializeNetwork(network):
    return {'ts': timestamp(network.ts), 'lanes': { l.id: serializeLane(l) for l in network.idLaneMap.values() },
        'vehicles': { v.id: serializeVehicle(v) for v in network.idVehicleMap.values() if v.isRunning(network.ts)}}

output = {}
output["nodes"] = { n.id: serializeNode(n) for n in network.idNodeMap.values()}
output["links"] = { l.id: serializeLink(l) for l in network.idLinkMap.values()}
output["networks"] = []
for network in networks:
    output["networks"].append(serializeNetwork(network))

print(output)
f = open("visualization/output.json", "w")
f.write("networkData = ")
f.write(json.dumps(output))