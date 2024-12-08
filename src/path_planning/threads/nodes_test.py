import networkx as nx
import time

G = nx.read_graphml("Competition_track_graph.graphml")    #read graph

class node:
    def __init__(self, x, y, next):
        self.x = x
        self.y = y
        self.next = next
        self.dashed = False

path = []

dashed = []
dashed_sources = []

for edge in G.edges(data=True):
    if "dotted" in edge[2] and edge[2]["dotted"] == True:
        dashed.append(edge)         #edges over dashed lines (overtaking allowed)

dashed_sources = [int(edge[0]) for edge in dashed]  #source nodes from which dashed lines to other nodes exist

# Create a dictionary to store nodes
graph = {}

for n in G.nodes():
    node_coordinates = G.nodes[n]
    n = int(n)
    node_info = node(node_coordinates['x'], node_coordinates['y'], [int(node) for node in list(G.successors(str(n)))])
    if n in dashed_sources:
        node_info.dashed = True     #from this node starts a dash line to any of its successors
    graph[n] = node_info