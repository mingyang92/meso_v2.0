# MESO v2.0

This document is a reference for MESO V2.0 simulation. A earlier version of this simulation framework by Dr. GONG Lei can be found in MESO V1.0 (2017).

The final update is on 2019/08/31 by LI, Yanyan (https://github.com/lyy901207/meso_v2.0). 



*Please note that this project is NOT finished, the code and explanation will be update irregularly...*



## 0. Background

This is a simulation framework designed for analyzing the vehicle behavior to the whole road network. This new version allows you to build up the vehicle behavior by yourself as well as modify the road network setting easily. The detail description for module structure and example can be found in the following parts.





## 1. Network

This module reads the network and vehicle generation information from **CSV format data**. The default case in this simulation is Sioux-Falls Network (xxx).  

![](C:\Users\lyy90\OneDrive\Documents\GitHub\meso_v2.0\Sioux Falls network\SiouxFalls.jpg)

*You should not change this part if you want to use the default simulation setting. If you want to use other network, please transfer your network information as the data format shown in the following part.*



The required variables for **vehicle generation** data are:

```python
# Vehicle generation data format
time_slot, vehicle_type, Origin (node id), Destination (node id), volume
7.00-7.04, car, 12, 1, 6
```

The require variables for **network generation** are:

```python
# Network generation data format
# Lane
lanetype, laneid, nodeid1, nodeid2, linkid, linktype, freespeed, freetraveltime, fixedcharge
0, 1, 1, 2, 1, 2, 60, 0.1, 0.1

# Node
nodeid, nodetype, coortype?, corrX, corrY
1, 2, 1, 500, 5100
```

*The variable name with "?" (e.g. coortype?) are currently not used.*





## 2. Structure

This part introduces the main components of the code. 

Basically, there are 5 major parts for running the basic scenario and one additional part that is designed for the extension case.



### Class Network

This class contains the information related to **Network**.

```python
# var time stamp network
Network.ts

# dict containing node info
Network.idNodeMap

# dict containing link info
Network.idLinkMap

# dict containing lane info
Network.idLaneMap

# dict containing vehicle info
Network.idVehicleMap

# dict containing lane type info
Network.typeGraphMap

# func register Node info in the current network
Network.registerNode(node)

# func register Link info in the current network
Network.registerLink(link)

# func register Lane info in the current network
Network.registerLane(lane)

# func register Vehicle info in the current network
Network.registerVehicle(vehicle)

# func update lane feature in the current network
Network.updateLanes()

# func count running vehicle in the current network
Network.runningVehicleCount()

# func count finished vehicle in the current network
Network.finishVehicleCount()

```



### Class Vehicle

This class contains the information related to **Vehicle**.

```python
# var vehicle id
Vehicle.id

# var vehicle type
Vehicle.type

# var driver type
# assign the driver type randomly (higher value of time & lower value of time)
Vehicle.driverType

# var vehicle max speed
# random pick from given interval (km/h)
# passenger car: [70, 80]
# bus: [40, 60]
Vehicle.maxSpeed

# var the value of time for vehicle
# normal(medianValueofTime, medianValueofTime/5)
Vehicle.valueTime

# var probability of changing lane
# the starting value are generated based on driver type.
# 这个方程目前没有在使用，实际上应该根据不同的车辆和道路属性对这个值进行更新。
Vehicle.probLaneChange

# var vehicle's starting time stamp
Vehicle.startTs

# var vehicle's starting node id
Vehicle.nodeOrigin

# var vehicle's destination node id
Vehicle.nodeDest 

# dict the network info of current time stamp
Vehicle.network

# func decide whether the vehicle start running in the current time stamp
Vehicle.isBegin(ts)

# func decide whether the vehicle finish it trip in the curreny time stamp
Vehicle.isFinish(ts)

# func decide whether the vehicle is running in the current time stamp
Vehicle.isRunning(ts)

# func updathe the shortest path for vehicle
Vehicle.updateShortestPath()

# func update the location of vehicle in the lane
# param timeInSecond: time period for updating location, default is 1s.
Vehicle.updateLocation(timeInSecond, delayType)
    
# func calculate cost of lane change for each vehicle (option)
# TODO: this function should consider the different feature that will influence the lane change behaviors.
# This function can be included in the following one
Vehicle.LaneChangeCost()

# func update the probability for each vehicle (need update!!!)
# TODO: calculte the probability of changing lane (e.g. descrete choice model)
Vehicle.updateProbLaneChange(medianValueTime)

# func decide whether the vehicle should change lane
Vehicle.changeLane(param...)

```



### Class Link

This class contains the information related to **Link**.

```python
# var link id
Link.id

# var link type
Link.type

# var link node1
Link.node1

# var link node2
Link.node2

# dict network
Link.network

# func length of link in kilometer
Link.lengthInKm()
```



### Class Node

This class contains the information related to **Node**.

```python
# var node id
Node.id

# var node type
Node.type

# var node coordinate x-axis
Node.x

# var node coordinate y-axis
Node.y

# dict the current network
Node.network

# func calculte the Euclideau distance between 2 nodes
Node.dist(node)

# func calculate Manhattan distance between 2 nodes
Node.manhattanDist(node)
```



### Class Lane

This class contains the information related to **Lane**. Be carefully, one link can contains several lanes (default 4 lanes).

```python
# var lane id
Lane.id

# var lane type
# 0: no charge; 1: charge
Lane.type

# var the link the current lane belongs to 
Lane.link

# var the free speed of current lane
# 60 km/h
Lane.freeSpeed

# var the travel time calculated in free speed for the current lane
Lane.freeTravelTime

# var the monetory charge if using the current lane
Lane.fixedCharge

# var the current speed in the current lane
Lane.speed

# dict the current network
Lane.network

# func update lane property based on PCU
Lane.updatePropertiesBasedOnPcu()

# func calculation delay time by different type of delay strategies
Lane.delayCalculation(strategy)
```



### Class Turn (under development)

This class contains the information related to **Turning Behavior**.





## 3. Route Search Algorithm

The detail description for these two algorithm will not be explained in this document. If you are interested in them, please click the links which can give you more information.

### Dijkstra

The explanation by Wikipedia: [https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm](https://en.wikipedia.org/wiki/Dijkstra's_algorithm)



### A*

The explanation by Wikipedia: [https://en.wikipedia.org/wiki/A*_search_algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)



### Other route search algorithm (option)

This framework allows you to specify your own route search algorithm. However, the algorithm should follow the format of the previous two algorithms (e.g. parameters) to make sure the whole simulation process can work as expecting.



*If you need to change the route search algorithm, please import the corresponding file in **Class Vehicle**!!!*

```python
# A*
from lib.astar import bestLaneBestNodeTimeCost

# Dijkstra
from lib.Dijkstra2 import bestLaneBestNodeTimeCost
```





## 4. Output

### Output file

The default output format is  **JSON**. You can also export the records in any format as you wish.

All the information in the class mentioned in the previous part can be export.

*Since all the information will be record in default settings, the size of output would be really large and make it difficult to load into memory the next time you want to use it. Therefore, currently we divided the  the output file into several sub-files. **Be sure you load ALL files when you use them.***



### Figure

The plotting function is written for the default file format (JSON). Unless you want to plot the same figure, you should write the function by yourself.

All the example in this documents are drawn by python package **Plotly**. Please check the website of **Plotly** for examples of using this package. https://plot.ly/python/





## 5. Example

*Please make sure you have install **all the necessary packages** before you test this example.*

Python 3.6.X or higher version is recommended. A sample is provided in ***test.py***. You need to set the path of OD data as well as the output file in this sample.



### Scenarios

**Test scenario:**  we assume that there are two lanes for each link.  One lane is free of charge for all types of vehicles. The other is free for buses but not for passenger cars, and the fee is calculated on the total travel time on charged lane.

**Basic scenario: ** we assume that there are two lanes for each link. In this scenario, both two types of lanes are free of charge for all types of vehicles.



### Process

This is a simple explanation for the simulation process in **test.py**. For more details, please have a look at the flow chart and code.



![](C:\Users\lyy90\OneDrive\Documents\GitHub\meso_v2.0\flow chart.png)



```python
for i in range(totalSteps):
    1. use the current network

    2. for vehcile in the current network:
          if this vehicle is running: continue
          if current time step - previous decision time step: continue
          vehicle updates the shortest path
    
    3. update lane features in the current network
    
    4. for vehicle in the current network:
          calculate the Time Cost for every vehicle
          if not vehicle.isRunning(network.ts): continue
          vehicle updates location in the current lane
          vehicle make decision whether to change lane

    5. copy the current network to i+1
    
    6. if no vehicle is running:
          break
```



### Parameters

The is the default model setting for running this example. If needed, you can change the settings.

```python
# default starting value: 2019-01-01 07:00:00
startTs = datetime.datetime(2019, 1, 1, 7, 0, 0)

# default total running time
totalSteps = 15000

# default time step
timeStep = 1

# default jam density (pcu/km)
jamDensity = 124

# default median value of time (normal distribution)
medianValueTime = 50

# vehicle generation distribution
# options: "uniform", "random", "random_whole", "normal_whole"
GEN_VEH_DIST = 'normal_whole' 

# delay strategy at interations
# options: 'vol_sim', 'vol_dist', 'random', 'fix'
STRATEGY = 'vol_sim' 

# number of vehicle
# options: 1, 2, 3,...
MULTIVEH = 1

# default charging (used to evaluate benchmark scenario)
# options: True, False
NO_CHARGE = False

# default decision time before entering interation
countTime = 5

# default time for updating best route
GAP_TS = 5
```



### Results

#### a. Vehicle Generation by Time Stamp

![](C:\Users\lyy90\OneDrive\Documents\GitHub\meso_v2.0\Vehicle Number by Time Stamp.png)



#### b. Travel time comparison

Positive time difference means **"introducing charged lane system increase the travel time of specific vehicle"**.



![](C:\Users\lyy90\OneDrive\Documents\GitHub\meso_v2.0\Travel Time Difference.png)

![](C:\Users\lyy90\OneDrive\Documents\GitHub\meso_v2.0\Travel Time Difference 2.png)

![](C:\Users\lyy90\OneDrive\Documents\GitHub\meso_v2.0\Travel Time Difference 3.png)

Most of the vehicles have longer travel time in **Charged Lane** scenario. Which means the current strategy is not so efficient.



#### c. Travel Speed by Time Stamp

We show the travel speed as an example here. You can use any feature you want.

![](C:\Users\lyy90\OneDrive\Documents\GitHub\meso_v2.0\Mean Speed.png)