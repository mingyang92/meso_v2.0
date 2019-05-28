#coding=utf-8
# Dijkstra's algorithm for shortest paths
# David Eppstein, UC Irvine, 4 April 2002
# http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/117228

# modified a bit on 2018/7/14 by Lei GONG
# the structure of dictionary of the graph has been changed a bit.
# the length/travel time of a link is now located in a dictionary,
# which means it needs one more step to get this value.
from lib.PrioDict import priorityDictionary
import copy

def Dijkstra(G,start,end=None):
    """
    Find shortest paths from the start vertex to all
    vertices nearer than or equal to the end.

    The input graph G is assumed to have the following
    representation: A vertex can be any object that can
    be used as an index into a dictionary.  G is a
    dictionary, indexed by vertices.  For any vertex v,
    G[v] is itself a dictionary, indexed by the neighbors
    of v.  For any edge v->w, G[v][w] is the length of
    the edge.  This is related to the representation in
    <http://www.python.org/doc/essays/graphs.html>
    where Guido van Rossum suggests representing graphs
    as dictionaries mapping vertices to lists of neighbors,
    however dictionaries of edges have many advantages
    over lists: they can store extra information (here,
    the lengths), they support fast existence tests,
    and they allow easy modification of the graph by edge
    insertion and removal.  Such modifications are not
    needed here but are important in other graph algorithms.
    Since dictionaries obey iterator protocol, a graph
    represented as described here could be handed without
    modification to an algorithm using Guido's representation.

    Of course, G and G[v] need not be Python dict objects;
    they can be any other object that obeys dict protocol,
    for instance a wrapper in which vertices are URLs
    and a call to G[v] loads the web page and finds its links.
    
    The output is a pair (D,P) where D[v] is the distance
    from start to v and P[v] is the predecessor of v along
    the shortest path from s to v.
    
    Dijkstra's algorithm is only guaranteed to work correctly
    when all edge lengths are positive. This code does not
    verify this property for all edges (only the edges seen
    before the end vertex is reached), but will correctly
    compute shortest paths even for some graphs with negative
    edges, and will raise an exception if it discovers that
    a negative edge has caused it to make a mistake.
    """

    D = {}    # dictionary of final distances
    P = {}    # dictionary of predecessors
    Q = priorityDictionary()   # est.dist. of non-final vert.
    Q[start] = 0
    
    for v in Q:
        D[v] = Q[v]
        if v == end: break
        # preventing broken road occur
        if v in G:
            for w in G[v]:
                vwLength = D[v] + G[v][w]
                if w in D:
                    if vwLength < D[w]:
                        raise ValueError("Dijkstra: found better path to already-final vertex")
                elif w not in Q or vwLength < Q[w]:
                    Q[w] = vwLength
                    P[w] = v
    return (D,P)

def shortestPath(G,start,end):
    """
    Find a single shortest path from the given start vertex
    to the given end vertex.
    The input has the same conventions as Dijkstra().
    The output is a list of the vertices in order along
    the shortest path.
    """

    D,P = Dijkstra(G,start,end)
#    flog1 = open('flog1.txt', 'w')
#    flog1.write(prnDict(P))
    Path = []
   
    reach=0
    if end in P:
        reach=1
    #problem of referencing P
#    elif end not in P.keys():
#        for i_key in P.keys():
#            if end==P[i_key]:
#                reach=1
#                break    
    else:
        reach=0
    if reach==1:
        while 1:
            Path.append(end)
            if end == start: break
            end = P[end]
        Path.reverse()
        return Path
    else:
        return Path

def shortestPathNode(G,start,end):
    # revised by Gong
    # this function returns a dictionary of cost (time or distance) along the path, \ 
    # and a dictionary of node-pair of links in the shortest path given a graph and a pair of origin-destination.
    # these two dictionaries have the same style as Dijkstra
    D,P = Dijkstra(G,start,end)
    Path = []
    
    reach=0
    if end in P:
        reach=1

    else:
        reach=0
    
    # change Path from list to a dictionary (by Lei GONG)
    dic_path={}# {node3:node2,node2:node5,node5:node1....}
    if reach==1:
        while 1:
            Path.append(end)
            if end == start: break
            end = P[end]
        Path.reverse()
        for i in range(0,len(Path)-1):
            dic_path[Path[i]]=Path[i+1]
        return D,dic_path
    else:
        return D,dic_path

def convert_ppath_to_pathids(dic_ppath,dic_graph,start_nodeid,end_nodeid):
    # this function converts the node-id-based shortest path output by function of shortestPathNode(G,start,end) to lane-id pair which will be used later.
    # input format:  {nodeid1:nodeid2,nodeid2:nodeid5,nodeid5:nodeid11,nodeid11:nodeid21,....}
    # output format: {laneid1:laneid12,laneid12:laneid21,laneid21:laneid3,...}
    # where lane1 is from node1 to node2, lane12 is from node2 to node5,...
    id1=start_nodeid
    list_laneids=[]# result in the process, list of lanes that compose the shortest path
    dic_routes={}# output
    #print "convert the node-pairs to list-of-lanes"
    if len(dic_ppath)>0:        
        while id1!=end_nodeid:
            id2=dic_ppath[id1]
            laneid=dic_graph[id1][id2].id
            list_laneids.append(laneid)
            id1=copy.deepcopy(id2)
    #print "convert the list-of-lanes to dic-of-lanes"
    if len(list_laneids)>1:
        for i in range(0,len(list_laneids)-1):
            dic_routes[list_laneids[i]]=list_laneids[i+1]       
        
    # check the number of items in each dictionary and list
    #print "the number of node-pairs:", len(dic_ppath)
    #print "the number of lanes:     ", len(list_laneids)
    #print "the number of lane-pairs:", len(dic_routes)
    return dic_routes

def bestLaneBestNodeTimeCost(G, start, end):
    D, P=shortestPathNode(G, start, end)
    bestRouteLane=convert_ppath_to_pathids(P, G, start, end)
    return (bestRouteLane, P, D[end])


#G = {'s':{'u':9, 'x':5},'v':{'y':4},'x':{'y':4,'s':3},'u':{'y':1,'z':3}}
#D1,P1=Dijkstra(G,'s','v')
#print(D1)
#print P1
#print shortestPath(G,'s','v')
'''
G = {'s':{'u':{'travelTime':-9}, 'x':{'travelTime':5}},'v':{'y':{'travelTime':4}},
        'x':{'y':{'travelTime':-4},'s':{'travelTime':3}},'u':{'y':{'travelTime':1},'z':{'travelTime':3}}}
D1,P1 = Dijkstra(G,'x','z')
print(D1, D1['z'])
print(P1)
print(shortestPathNode(G,'x','u'))
'''
#As an example of the input format, here is the graph from Cormen, Leiserson, 
#    and Rivest (Introduction to Algorithms, 1st edition), page 528:
#G = {'s':{'u':10, 'x':5}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 
#     'x':{'u':3, 'v':9, 'y':2}, 'y':{'s':7, 'v':6}}
#The shortest path from s to v is ['s', 'x', 'u', 'v'] and has length 9.

print("next example")
G = {'s':{'u':10, 'x':5}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 'x':{'u':3, 'v':9, 'y':2}, 'y':{'s':7, 'v':6}}
D1,P1 = Dijkstra(G,'x','z')
print('D is:', D1)
print('P is:', P1)
Path = shortestPath(G,'s','v')
print('The shortest path from s to v: ', Path)

# not reachable
#print "the 3rd example" 
#G = {'s':{'u':10, 'x':5}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 'x':{'u':3, 'v':9, 'y':2}, 'y':{'v':6}}
#Path = shortestPath(G,'y','s')
#print 'The shortest path from y to s: ', Path

# test for broken roads
#G = {'s':{'u':10, 'x':5, 'p':7}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 'x':{'u':3, 'v':9, 'y':2}, 'y':{'v':6}}
#Path = shortestPath(G,'s','v')
#print 'The shortest path from s to v: ', Path