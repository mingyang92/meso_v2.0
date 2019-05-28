'''
Created on 2018/07/02
This module is used to read the OD matrices from csv files.

csv file of OD matrix should be named as the time slot with beginning and ending time;
names following the above rule is to suit the loading of traffic demand to the road network;
all csv files for one road network or a possible scenario should be put in one folder.

The format of OD demand in the csv file consists of the following columns:
time slot, vehicle type, origin ID, destination ID,vehicle volume

currently, we use a time slot of 5 minutes. vehicles generated during this time slot are put randomly onto the road network.

done and checked on 2018/7/4

module of generating vehicles is also included in this module as vehicle-generation is just the next step of reading OD-matrix; and these two modules cannot be separated.

@author: Gong, L., LI, Y.
'''
import time, datetime
import random
import numpy as np

random.seed(10)
from model.vehicle import Vehicle
import os, csv, math

def timeslot2tsPair(timeslot):
    """
    This function modifies time format to TIME PAIRS
    :param timeslot:
    :return:
    """
    tlist = [t.split(".") for t in timeslot.split("-")]
    return datetime.datetime(2019, 1, 1, int(tlist[0][0]), int(tlist[0][1]), 0), datetime.datetime(2019, 1, 1, int(tlist[1][0]), int(tlist[1][1]))

def vehicleMaxSpeed(type):
    """
    This function determines max vehicle speed: random pick from (min speed, max speed)
    :param type:
    :return: int, speed km/hour
    """
    maxSpeedInterval = {"car": [70, 80], "truck": [50, 70], "bus": [40, 60]}
    return random.randint(maxSpeedInterval[type][0], maxSpeedInterval[type][1])

def genDriverType():
    """
    This function determines driver types: randomly assign
    :return: int, 0/1
    """
    return random.randint(0, 1)

def genDriverValueTimeGen(medianValueTime):
    """
    This functions generates driver value of time
    random generate by normal distribution (mean=medianValueTime, sigma = mean/5)
    :param medianValueTime:
    :return:
    """
    sigma = medianValueTime / 5.0
    return np.random.normal(medianValueTime, sigma, 1)[0]

def genProbLaneChange(type):
    """
    This function calculate the probability of changing lane
    :param type: driver type
    :return: number in (0,1)
    """
    if type == 0:
        return random.randint(0, 5) / 10.0
    elif type == 1:
        return random.randint(6, 10) / 10.0

def readOd(path):
    """
    This function read OD file
    :param path:
    :return:
    """
    filenames = os.listdir(path)

    tsPairNodePairTypeMap = {}
    for filename in filenames:
        pathfile = path + "/" + filename
        f = open(pathfile)
        f.readline()
        r = csv.reader(f)
        for row in r:
            print(row)
            (timeslot, vehicleType, origin, dest, volume) = (row[0], row[1], row[2], row[3], int(row[4]))
            tsPair = timeslot2tsPair(timeslot)

            if tsPair not in tsPairNodePairTypeMap: tsPairNodePairTypeMap[tsPair] = {}
            if (origin, dest) not in tsPairNodePairTypeMap[tsPair]: tsPairNodePairTypeMap[tsPair][(origin, dest)] = {}
            if vehicleType not in tsPairNodePairTypeMap[tsPair][(origin, dest)]: tsPairNodePairTypeMap[tsPair][(origin, dest)][vehicleType] = 0

            tsPairNodePairTypeMap[tsPair][(origin, dest)][vehicleType] += volume

    return tsPairNodePairTypeMap




def genVehicle(tsPairNodePairTypeMap, distribution, vehicleId, medianValueTime, network, multiple = 1):
    """
    This function generate vehicle by "distribution"
    :param tsPairNodePairTypeMap: in tsPAIR and nodePAIR generate type of VEHICLE
    :param distribution: "uniform";"uniform_whole";"random";"random_whole";"normal_whole"
    :param vehicleId:
    :param medianValueTime:
    :param network:
    :return:
    """
    if distribution == "uniform":
        for tsPair in tsPairNodePairTypeMap.keys():
            for nodePair in tsPairNodePairTypeMap[tsPair].keys():
                #print('nodePair',nodePair)
                for vehicleType in tsPairNodePairTypeMap[tsPair][nodePair]:
                    #print('vehicle type:', vehicleType)
                    vehicleVolume = multiple * tsPairNodePairTypeMap[tsPair][nodePair][vehicleType]
                    startTs, endTs = tsPair[0], tsPair[1]
                    duration = endTs - startTs
                    #print('duration:',duration)
                    origin, dest = network.idNodeMap[nodePair[0]], network.idNodeMap[nodePair[1]]
                    #print('vehicle vol',vehicleVolume)
                    if vehicleVolume <= 0:
                        continue
                    interval = duration / vehicleVolume
                    #print('interval:',interval, vehicleVolume)
                    for i in range(vehicleVolume):
                        vehicleStartTs = startTs + datetime.timedelta(seconds=int(math.ceil(interval.seconds * i ) + 1 ))
                        print('vehicleStartTime:', vehicleStartTs)
                        vehicleId += 1
                        maxSpeed = vehicleMaxSpeed(vehicleType)
                        #print(maxSpeed)
                        driverType = genDriverType()
                        valueTime = genDriverValueTimeGen(medianValueTime)
                        probLaneChange = genProbLaneChange(driverType)
                        Vehicle(vehicleId, vehicleType, driverType, maxSpeed, valueTime, probLaneChange,
                                vehicleStartTs, origin, dest, network)
        print('--------vehicle generation finish: uniform-------------------')

    elif distribution == 'uniform_whole':
        # THIS METHOD IS PROBLEMATIC
        listTsPair = []  # collect all the time stamps of OD start and end
        vehicleTotal = 0  # count the total number of vehicles in the OD matrix; not in pcu
        for tsPair in tsPairNodePairTypeMap.keys():
            startTs, endTs = tsPair[0], tsPair[1]
            if not startTs in listTsPair:
                listTsPair.append(startTs)
            if not endTs in listTsPair:
                listTsPair.append(endTs)
            for nodePair in tsPairNodePairTypeMap[tsPair].keys():
                for vehicleType in tsPairNodePairTypeMap[tsPair][nodePair]:
                    vehicleTypeValue = multiple * tsPairNodePairTypeMap[tsPair][nodePair][vehicleType]
                    vehicleTotal += vehicleTypeValue
        #print(vehicleTotal)
        listTsPair.sort()  # re-order the time stamps of OD starts and ends
        startTs = listTsPair[0]
        endTs = listTsPair[-1]
        duration = endTs - startTs
        #print('duration:',duration)

        if vehicleTotal > 0:
            interval = duration / vehicleTotal
            # TODO: BUG HERE
            #print('interval',interval.seconds, 'vehicle total:', vehicleTotal)

        count_id = list(range(vehicleTotal))
        for tsPair in tsPairNodePairTypeMap.keys():
            for nodePair in tsPairNodePairTypeMap[tsPair].keys():
                origin, dest = network.idNodeMap[nodePair[0]], network.idNodeMap[nodePair[1]]
                for vehicleType in tsPairNodePairTypeMap[tsPair][nodePair]:
                    vehicleVolume = multiple * tsPairNodePairTypeMap[tsPair][nodePair][vehicleType]
                    #print('current vehicle vol:',vehicleVolume,'in', tsPair)
                    if not vehicleVolume: continue
                    for i in range(vehicleVolume):
                        vehicleId += 1
                        vehicleStartTs = startTs + datetime.timedelta(seconds=int(math.ceil(interval.seconds) + 1)*count_id.pop(0))
                        print('vehicle',i, 'start ts', vehicleStartTs)
                        maxSpeed = vehicleMaxSpeed(vehicleType)
                        driverType = genDriverType()
                        valueTime = genDriverValueTimeGen(medianValueTime)
                        probLaneChange = genProbLaneChange(driverType)
                        Vehicle(vehicleId, vehicleType, driverType, maxSpeed, valueTime,
                                probLaneChange,vehicleStartTs, origin, dest, network)
        print('--------vehicle generation finish: uniform_whole-------------------')
    # TODO: speed problem when multiple
    elif distribution == "random":
        # the vehicles are randomly generated/put onto the network during the time slot
        # a random time-stamp during the time slot
        # the calculated start-time-stamp of vehicle has to be modified a bit to suit the convenience of time-step
        # for simulation
        for tsPair in tsPairNodePairTypeMap.keys():
            for nodePair in tsPairNodePairTypeMap[tsPair].keys():
                for vehicleType in tsPairNodePairTypeMap[tsPair][nodePair]:
                    vehicleVolume = multiple * tsPairNodePairTypeMap[tsPair][nodePair][vehicleType]
                    startTs, endTs = tsPair[0], tsPair[1] # two time stamps are in string type
                    #print start_timestamp,end_timestamp
                    #print type(start_timestamp),type(end_timestamp)
                    duration = endTs - startTs # the duration of time slot of each csv file; unit in seconds
                    origin, dest = network.idNodeMap[nodePair[0]], network.idNodeMap[nodePair[1]] ######*******Origin and Destination

                    if vehicleVolume>0:
                        for i in range(vehicleVolume):# totally generate OD_value vehicles in this iteration
                            pickId = random.randint(0, duration.seconds - 1)
                            vehicleId += 1
                            vehicleStartTs = startTs + datetime.timedelta(seconds=int(math.ceil(pickId)))
                            #print(vehicleStartTs)
                            maxSpeed = vehicleMaxSpeed(vehicleType)
                            driverType = genDriverType()
                            valueTime = genDriverValueTimeGen(medianValueTime)
                            probLaneChange = genProbLaneChange(driverType)
                            Vehicle(vehicleId, vehicleType, driverType, maxSpeed, valueTime,
                                    probLaneChange, vehicleStartTs, origin, dest, network)
        print('--------vehicle generation finish: random-------------------')

    elif distribution == "random_whole":
        listTsPair = []  # collect all the time stamps of OD start and end
        vehicleTotal = 0  # count the total number of vehicles in the OD matrix; not in pcu
        for tsPair in tsPairNodePairTypeMap.keys():
            startTs, endTs = tsPair[0], tsPair[1]
            if not startTs in listTsPair:
                listTsPair.append(startTs)
            if not endTs in listTsPair:
                listTsPair.append(endTs)
            for nodePair in tsPairNodePairTypeMap[tsPair].keys():
                for vehicleType in tsPairNodePairTypeMap[tsPair][nodePair]:
                    vehicleTypeValue = multiple * tsPairNodePairTypeMap[tsPair][nodePair][vehicleType]
                    vehicleTotal += vehicleTypeValue
        print(vehicleTotal)
        listTsPair.sort()  # re-order the time stamps of OD starts and ends
        startTs = listTsPair[0]
        endTs = listTsPair[-1]
        duration = endTs - startTs
        #print(duration.seconds)

        if vehicleTotal > 0:
            for tsPair in tsPairNodePairTypeMap.keys():
                for nodePair in tsPairNodePairTypeMap[tsPair].keys():
                    origin, dest = network.idNodeMap[nodePair[0]], network.idNodeMap[nodePair[1]]
                    for vehicleType in tsPairNodePairTypeMap[tsPair][nodePair]:
                        vehicleVolume = multiple* tsPairNodePairTypeMap[tsPair][nodePair][vehicleType]
                        if not vehicleVolume: continue
                        for i in range(vehicleVolume):
                            pickId = random.randint(0, duration.seconds - 1)
                            vehicleId += 1
                            vehicleStartTs = startTs + datetime.timedelta(seconds=int(math.ceil(pickId) + 1 ))
                            #print(vehicleStartTs, 'origin', origin, 'end', dest)
                            maxSpeed = vehicleMaxSpeed(vehicleType)
                            driverType = genDriverType()
                            valueTime = genDriverValueTimeGen(medianValueTime)
                            probLaneChange = genProbLaneChange(driverType)
                            Vehicle(vehicleId, vehicleType, driverType, maxSpeed, valueTime,
                                    probLaneChange,vehicleStartTs, origin, dest, network)
        print('--------vehicle generation finish: random_whole-------------------')

    elif distribution == "normal_whole":
        listTsPair = []  # collect all the time stamps of OD start and end
        vehicleTotal = 0  # count the total number of vehicles in the OD matrix; not in pcu
        for tsPair in tsPairNodePairTypeMap.keys():
            startTs, endTs = tsPair[0], tsPair[1]
            if not startTs in listTsPair:
                listTsPair.append(startTs)
            if not endTs in listTsPair:
                listTsPair.append(endTs)
            for nodePair in tsPairNodePairTypeMap[tsPair].keys():
                for vehicleType in tsPairNodePairTypeMap[tsPair][nodePair]:
                    vehicleTypeValue = multiple * tsPairNodePairTypeMap[tsPair][nodePair][vehicleType]
                    vehicleTotal += vehicleTypeValue
        print(vehicleTotal)
        listTsPair.sort()  # re-order the time stamps of OD starts and ends
        startTs = listTsPair[0]
        endTs = listTsPair[-1]
        duration = endTs - startTs
        mu = 900 ## need to be set
        sigma = 100 ## need to be set

        def normalTs(mu, sigma, size):
            """
            special attention for mu and sigma!!!
            :param mu:
            :param sigma:
            :param size:
            :return:
            """
            # set seeds
            np.random.seed(10)
            s = np.random.normal(mu, sigma, size)
            listTs = []  # initialize the output list of time stamps
            # convert items in s to int
            for i in range(0, len(s)):
                s[i] = int(math.ceil(s[i]))
                if s[i] <= 0:
                    s[i] = 1
                timestamp = startTs + datetime.timedelta(seconds=s[i])
                listTs.append(timestamp)
            listTs.sort()
            return listTs

        normTsList = normalTs(mu, sigma, vehicleTotal)
        print(normTsList[:10])


        if duration.seconds > 0 and vehicleTotal > 0:
            for tsPair in tsPairNodePairTypeMap.keys():
                for nodePair in tsPairNodePairTypeMap[tsPair].keys():
                    origin, dest = network.idNodeMap[nodePair[0]], network.idNodeMap[nodePair[1]]
                    for vehicleType in tsPairNodePairTypeMap[tsPair][nodePair]:
                        vehicleVolume = multiple * tsPairNodePairTypeMap[tsPair][nodePair][vehicleType]
                        for i in range(0, vehicleVolume):
                            vehicleId += 1
                            vehicleStartTs = normTsList.pop(0)
                            maxSpeed = vehicleMaxSpeed(vehicleType)
                            driverType = genDriverType()
                            valueTime = genDriverValueTimeGen(medianValueTime)
                            probLaneChange = genProbLaneChange(driverType)
                            Vehicle(vehicleId, vehicleType, driverType, maxSpeed, valueTime,
                                    probLaneChange,vehicleStartTs, origin, dest, network)
        print('--------vehicle generation finish: normal_whole-------------------')

