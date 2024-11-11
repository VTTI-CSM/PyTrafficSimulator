import sys
import pandas as pd

class Dijkstra:

    def __init__(self, sim_object):
        # Set default configuration
        self.set_default_config()

        # Calculate properties
        self.init_properties(sim_object)


    def set_default_config(self):
        self.links = []
        self.nodes = {}
        self.graph = [[]]

    def init_properties(self, sim_object):
        self.set_functions(sim_object)


    def set_functions(self, sim_object):
        self.links = sim_object.segments
        # self.nodes = self.get_nodes()
        self.nodes = sim_object.nodes.keys()
        self.graph = {node: {node: 0 for node in self.nodes} for node in self.nodes}
        
        # [[0 for column in self.nodes] for row in self.nodes]
        self.link_vertices = {}
        self.calculate_graph()
    
    def get_nodes(self):
        nodes = []
        for link in self.links:
            if link.start_node not in nodes:
                nodes.append(link.start_node)
            if link.end_node not in nodes:
                nodes.append(link.end_node)
        return nodes

            



    def calculate_graph(self):
        # print(len(self.graph), self.nodes)
        for link in self.links:
            self.link_vertices[(link.start_node, link.end_node)] = link.id
            self.graph[link.start_node][link.end_node] = link.get_length()
    
    
    def printSolution(self, dist, prev_node):
        print("Vertex \tDistance from Source")
        for node in self.nodes:
            print(node, "\t", dist[node], "\t", prev_node[node])
    
    
    # Function that implements Dijkstra's single source shortest path algorithm for a graph represented
    # using adjacency matrix representation
    def dijkstra(self, src):
        dist = {i: 999999 for i in self.nodes}
        prev_node = {i: None for i in self.nodes}
        dist[src] = 0
        sptSet = {i: False for i in self.nodes}
 
        for cout in self.nodes:
 
            # Pick the minimum distance vertex from the set of vertices not yet processed.
            # x is always equal to src in first iteration
            x = self.minDistance(dist, sptSet)
 
            # Put the minimum distance vertex in the shortest path tree
            sptSet[x] = True
 
            # Update dist value of the adjacent vertices of the picked vertex only if the current
            # distance is greater than new distance and the vertex in not in the shortest path tree
            for y in self.nodes:
                if self.graph[x][y] > 0 and sptSet[y] == False and \
                        dist[y] > dist[x] + self.graph[x][y]:
                    dist[y] = round(dist[x] + self.graph[x][y], 2)
                    # print(self.graph[x][y])
                    prev_node[y] = x
 
        # self.printSolution(dist, prev_node)
        return prev_node
    

    def minDistance(self, dist, sptSet):
        # Initialize minimum distance for next node
        min = sys.maxsize
        min_index = 0
 
        # Search not nearest vertex not in the shortest path tree
        for u in self.nodes:
            if dist[u] < min and sptSet[u] == False:
                min = dist[u]
                min_index = u
        return min_index
    
    def get_path(self, src, dest):
        # print(pd.DataFrame(self.graph))
        prev_node = self.dijkstra(src)
        path = []
        while dest != src:
            path.append(dest)
            dest = prev_node[dest]
        path.append(src)
        path_reversed = path[::-1]

        vehicle_path_links = []

        for i in range(len(path_reversed) - 1):
            vehicle_path_links.append(self.link_vertices[(path_reversed[i], path_reversed[i + 1])])
        # print(path_reversed)
        return vehicle_path_links


