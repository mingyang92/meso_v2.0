
#from lib.PrioDict import priorityDictionary
import copy
#import model.node
#import model.turn
import model.lane


def astar(G, start, end, network):
    '''
    self.typeGraphMap[lane.type][lane.link.node1.id][lane.link.node2.id] = lane
    This function applied a star algorithm to find the lowest cost path
        g = travel time cost
        h = mahattanDist(end)
        f = g + h
    D: {node1: distance from start to node1, node2: distance from start to node2,...}
    P: {node1: parent of node1, node2: parent of node2,...}
    :param G: self.network.typeGraphMap[self.laneType]
    :param start: start node
    :param end: end node
    :param network: network in current timestamp
    :return: (D, P)
    '''
    #print('parameter',G, start, end, network)
    aMap = {} # dict to store g, h, f
    P = {} # dict to store parent node
    D = {} # dict to store final cost

    start_node = start
    end_node = end
    #print('start node:', start_node, '; end node is:',end_node)

    aMap[start_node] = {'g_cost':0.0, 'h_cost':0.0, 'f_cost':0.0}
    aMap[end_node] = {'g_cost':0.0, 'h_cost':0.0, 'f_cost':0.0}

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while open_list:
        #print('come into this part:')

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            #print('item:', item, ';current_node:', current_node)
            #D[current_node] = aMap[current_node]['f_cost']
            D[item] = aMap[item]['f_cost']

            #if current_node == end_node:
            if item == end_node:
                #print('end node is:', current_node)
                # break
                path = []
                current = current_node
                # break
                while current:
                    # print('path current is:', current)
                    path.append(current)
                    if current != start_node:
                        current = P[current]
                    else:
                        break
                #print(D, path[::-1])
                return (D, P)
                # return path[::-1] # Return reversed path

            if aMap[item]['f_cost'] < aMap[current_node]['f_cost']:
                current_node = item
                current_index = index
                #print('aMap[item]:',aMap[item]['f_cost'], ';aMap[current]:', aMap[current_node]['f_cost'])


        # Pop current off open list, add to closed list
        if current_node not in closed_list:
            open_list.pop(current_index)
            closed_list.append(current_node)
            #print("open list is:", open_list, 'closed list is:', closed_list)
        #break

        #print(current_node, 'D is:', D)

        # Found the goal


        # Generate children
        children = []
        #print('current_node is:', current_node)
        for success_node in G[current_node]:
            #print('This is success_node for' , current_node, ':', success_node)
            #G[success_node]['parent'] = current_node
            # Append
            children.append(success_node)
            if success_node not in closed_list:
                P[success_node] = current_node
            #print('P is :', P)
        #break

        # Loop through children
        for child in children:
            #print('child is:', child, children)
            #print('G is:', G[child])

            # Create the f, g, and h values
            if child in closed_list:
                continue
            aMap[child] = {'g_cost':0.0, 'h_cost':0.0, 'f_cost':0.0}
            aMap[child]['g_cost'] = aMap[current_node]['g_cost'] + G[current_node][child].travelTime
            #print('gCost:',aMap[child]['g_cost'])
            #child.h = child.manhattanDist(end) + [current_node][child]
            manhattanDist = abs(network.idNodeMap[child].x - network.idNodeMap[end_node].x) \
                            + abs(network.idNodeMap[child].y - network.idNodeMap[end_node].y)
            # todo: speed==None
            #print('G[P[child]][child]',G[P[child]][child].freeSpeed)
            if G[P[child]][child].speed == None:
                print('The speed is None!!')
                G[P[child]][child].speed = G[P[child]][child].freeSpeed
            heuristicCost = manhattanDist / G[P[child]][child].speed  # magic speed
            #print('heuristic:', heuristicCost)
            aMap[child]['h_cost'] = heuristicCost
            aMap[child]['f_cost'] = aMap[child]['g_cost'] + aMap[child]['h_cost']
            #print('node id', child, ';h_cost:',aMap[child])
            # Child is already in the open list
            #for open_node in open_list:
            #    if child in open_list or aMap[child]['f_cost'] > aMap[open_node]['f_cost'] or child in closed_list:
            #        continue
            # Add the child to the open list
            if (child not in closed_list) and (child not in open_list):
                #print('before add to open list, the current open list is:', open_list)
                open_list.append(child)
        #if len(open_list)>15: break
    #print(D,P)



def shortestPathNode(G, start, end, network):
    # revised by Gong
    # this function returns a dictionary of cost (time or distance) along the path, \
    # and a dictionary of node-pair of links in the shortest path given a graph and a pair of origin-destination.
    # these two dictionaries have the same style as Dijkstra
    #D, P = Dijkstra(G, start, end)
    #print(G, start, end, network)
    D, P = astar(G, start, end, network)
    Path = []

    reach = 0
    if end in P:
        reach = 1
    else:
        reach = 0

    dic_path = {}  # {node3:node2,node2:node5,node5:node1....}
    if reach == 1:
        while 1:
            Path.append(end)
            if end == start: break
            end = P[end]
        Path.reverse()
        for i in range(0, len(Path) - 1):
            dic_path[Path[i]] = Path[i + 1]
        return D, dic_path
    else:
        return D, dic_path

def convert_ppath_to_pathids(dic_ppath, dic_graph, start, end):
    '''
    This function converts node id based shortest path output by shortestPathNode()
    Further info @ Dijkstra2
    :param dic_ppath: dic_ppath = P in shortestPathNode()
    :param dic_graph: G is the input Graph
    :param start_nodeid: start node id
    :param end_nodeid: end node id
    :return: dictionary of lane graph: {laneid1:laneid12,laneid12:laneid21,laneid21:laneid3,...}
    '''
    id1 = start
    list_laneids = []  # result in the process, list of lanes that compose the shortest path
    dic_routes = {}  # output
    # print "convert the node-pairs to list-of-lanes"
    if len(dic_ppath) > 0:
        while id1 != end:
            id2 = dic_ppath[id1]
            laneid = dic_graph[id1][id2].id
            list_laneids.append(laneid)
            id1 = copy.deepcopy(id2)
    # print "convert the list-of-lanes to dic-of-lanes"
    if len(list_laneids) > 1:
        for i in range(0, len(list_laneids) - 1):
            dic_routes[list_laneids[i]] = list_laneids[i + 1]

    # check the number of items in each dictionary and list
    # print "the number of node-pairs:", len(dic_ppath)
    # print "the number of lanes:     ", len(list_laneids)
    # print "the number of lane-pairs:", len(dic_routes)
    return dic_routes


def bestLaneBestNodeTimeCost(G, start, end, network):
    D, P = shortestPathNode(G, start, end, network)
    bestRouteLane = convert_ppath_to_pathids(P, G, start, end)  # dictionary of lane id of best route
    #print('Start:', start, 'End:', end)
    #print('This is bestRouteLane:',bestRouteLane)
    #print('This is P:', P)
    #print('This is D[end]:', D[end])
    return (bestRouteLane, P, D[end])


def main():
    '''
    G = {'s':{'u':{'travel-time':-9}, 'x':{'travel-time':5}},'v':{'y':{'travel-time':4}},
    'x':{'y':{'travel-time':-4},'s':{'travel-time':3}},'u':{'y':{'travel-time':1},'z':{'travel-time':3}}}
    D1,P1=Dijkstra(G,'x','z')
    print D1,D1['z']
    print P1
    print shortestPathNode(G,'x','u')
    '''
    # As an example of the input format, here is the graph from Cormen, Leiserson,
    #    and Rivest (Introduction to Algorithms, 1st edition), page 528:A
    G = {'s':('u', 'v'),
         'u':('y'),
         'v':('y'),
         'y':('u'),
         }

    costMap = { 's':{'u':{'tc': 10, 'stress': 1}, 'v':{'tc': 5, 'stress': 4}},
                'u':{'y':{'tc': 1, 'stress': 0}},
                'v':{'y':{'tc': 4, 'stress': 2}},
                'y':{'u':{'tc':20, 'stress': 0}}
                }

    #print(shortestPath(G, 'x', 'u'))
    # The shortest path from s to v is ['s', 'x', 'u', 'v'] and has length 9.

    # print "next example"
    # G = {'s':{'u':10, 'x':5}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 'x':{'u':3, 'v':9, 'y':2}, 'y':{'s':7, 'v':6}}
    # Path = shortestPath(G,'s','v')
    # print 'The shortest path from s to v: ', Path

    # not reachable
    # print "the 3rd example"
    # G = {'s':{'u':10, 'x':5}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 'x':{'u':3, 'v':9, 'y':2}, 'y':{'v':6}}
    # Path = shortestPath(G,'y','s')
    # print 'The shortest path from y to s: ', Path

    # test for broken roads
    # G = {'s':{'u':10, 'x':5, 'p':7}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 'x':{'u':3, 'v':9, 'y':2}, 'y':{'v':6}}
    # Path = shortestPath(G,'s','v')
    # print 'The shortest path from s to v: ', Path

    #start = (0, 0)
    #end = (7, 6)

    #print(generateCostMap(G))
    #aMap , P = astar(G, 's', 'y', network)
    #print('The fist return of astar() is:', aMap)
    #print('The second return of astar() is:', P)
    #path = bestLaneBestNodeTimeCost(G, 's', 'y', costMap)
    #print(path)


if __name__ == '__main__':
    main()