#!/usr/bin/python3
 
import json
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

laneDict = {}
vehDict = {}
# 将 JSON 对象转换为 Python 字典
path = 'C:/Users/lyy90/OneDrive/Documents/GitHub/twinwell/twinwell/visualization/uniform_fix1_output.json'

with open(path, 'r') as f:
    data = json.load(f)
    #print(data['networks'][0])
    count = 0

    for network in data['networks']:
        #print(network.keys())
        res = []
        res1 = []
        #print(network['ts'])
        for lane in network['lanes'].keys():
            travel_time = network['lanes'][lane]['travelTime']
            speed = network['lanes'][lane]['speed']
            lane_density = network['lanes'][lane]['density']
            lane_type = network['lanes'][lane]['type']
            pcu_count = network['lanes'][lane]['countPcu']
            res.append([travel_time, speed, lane_density, pcu_count, lane_type])
            #print(res)
        laneDict[count] = res
    
        for vehicle in network['vehicles'].keys():
            pr = network['vehicles'][vehicle]['laneType']
            change = network['vehicles'][vehicle]['changeLane']
            #print(pr)
            res1.append((vehicle, pr, change, lane_type, type))
            #print(res1)
        vehDict[count] = res1

        count+=1
        #if count==2: break


print(laneDict[100])
# [travel_time, speed, lane_density, pcu_count, lane_type]
x1 = []
x2 = []
y = list(range(count))

# 计算车辆利用high speed的时间

for ts in laneDict.keys():
    #x1.append(np.mean([x[1] for x in laneDict[ts] if x[4]=='0'])) # low speed
	x1.append(max([x[1] for x in laneDict[ts] if x[4]=='0']) - min([x[1] for x in laneDict[ts] if x[4]=='0']))
    #x2.append(np.mean([x[1] for x in laneDict[ts] if x[4]=='1'])) # high speed
	x2.append(max([x[1] for x in laneDict[ts] if x[4]=='1']) - min([x[1] for x in laneDict[ts] if x[4]=='1']))


#print(laneDict[0], len(laneDict[0]))
plt.figure()
plt.plot(y, x1, label = 'low speed', linewidth = 2.0)
plt.plot(y, x2, color='red', linewidth=2.0, label = 'high speed')
plt.legend(loc='upper right')
plt.ylim((0, 100))
plt.xlim((0, 3000))
plt.xlabel('Time')
plt.ylabel('mean speed gap')
plt.show()

