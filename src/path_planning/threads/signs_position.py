import networkx as nx

highway_entry_pos = [*range(181,196,1), *range(363,366,1), *range(259,264,1)]
highway_exit_pos = [*range(275,282,1) , *range(294,302,1), *range(196,201,1) ,*range(224,239,1), *range(248,259,1)]
roundabout_pos = [*range(228,239,1), *range(249,259,1), *range(325,332,1), 371, *range(393,399,1) ,357, *range(349,357,1) ,370]
parking_pos = [*range(452,466,1)]
crosswalk_pos = [*range(447,453,1) , *range(159,166,1), 70, 66, *range(171,179,1), 90, 7, *range(429,442,1), *range(399,405,1), *range(412,418,1)]
stop_pos = priority_pos =  [55, 447, 59, 92, 47, 46, 91, 60, *range(115,120,1),51, *range(107,112,1), 57, *range(99,102,1), 
                    28, 27, *range(102,105,1), 58, 48, *range(93,96,1), 16, 15, *range(96,99,1), 49, *range(480,487,1) , *range(488,490,1), 30, *range(441,447,1) ,17,21, 26, 
                    23, 122, 28, 84, 120, 14, 13, 123, 85, 31, *range(124,127,1), 3, *range(127,130,1), 32, 19, *range(130,133,1) ,40, 39, *range(133,136,1), 20, 86, 
                    *range(136,139,1), 74, *range(139,142,1) ,73, 87, *range(501,504,1) ,6, 1, 143, 42, 41, 142, 2, 75, 145, 38, 37, 144, 76, 77, *range(146,149,1), 65, 
                    *range(149,152,1) ,78, 64, *range(210,216,1), 69, *range(379,386,1), 399  , *range(418,424,1), *range(404,409,1), 478, 479, *range(464,471,1)]

#print(priority_pos)

G = nx.read_graphml("test.graphml")    #read graph

# for i in priority_pos:
#     G.nodes[str(i)]['priority'] = True
# nx.write_graphml_lxml(G, "test.graphml")

