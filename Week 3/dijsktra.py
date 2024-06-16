import networkx as nx
import matplotlib.pyplot as plt

G = nx.Graph()

# Adding the nodes
for i in range(7):
    for j in range(7):
        node_label = i * 7 + j
        G.add_node(node_label, pos=(i, j))

# Adding the edges
for i in range(7):
    for j in range(7):
        node_label = i * 7 + j
        if i < 6:
            G.add_edge(node_label, (i + 1) * 7 + j, weight=1)  # Add downward edge
        if j < 6:
            G.add_edge(node_label, i * 7 + (j + 1), weight=1)  # Add rightward edge

# Setting start and end nodes
start_node = 0
end_node = 48

# Defining the Manhattan distance heuristic function
def manhattan_distance(u, v):
    u_pos = G.nodes[u]['pos']
    v_pos = G.nodes[v]['pos']
    return abs(u_pos[0] - v_pos[0]) + abs(u_pos[1] - v_pos[1])

# Finding the shortest path using A* algorithm
shortest_path = nx.astar_path(G, start_node, end_node, heuristic=manhattan_distance)

# Printing the shortest path
print(shortest_path)

# Drawing the graph
pos = nx.get_node_attributes(G, 'pos')
labels = nx.get_edge_attributes(G, 'weight')

plt.figure(figsize=(5, 5))
nx.draw(G, pos, with_labels=False, node_size=150, node_color='w', edgecolors='k', linewidths=0.5)
nx.draw_networkx_edges(G, pos, edgelist=labels.keys(), width=0.5)
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
nx.draw_networkx_nodes(G, pos, nodelist=[start_node], node_color='g', node_size=200)
nx.draw_networkx_nodes(G, pos, nodelist=[end_node], node_color='r', node_size=200)

# Highlighting the shortest path
path_edges = [(shortest_path[i], shortest_path[i + 1]) for i in range(len(shortest_path) - 1)]
nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='b', width=2.0)

plt.axis('equal')
plt.axis('off')
plt.show()
