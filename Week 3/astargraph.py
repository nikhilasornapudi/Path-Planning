from graph_tool.all import *

N = 10  # Number of vertices

# Create an Undirected Graph
ug = Graph(directed=False)

# Add N vertices
ug.add_vertex(N)

# Add edges
ug.add_edge_list([(0, 1), (0, 2), (1, 3), (1, 4), (2, 5), (2, 6), (3, 7), (4, 8), (5, 9)])

# Define labels for vertices
vertex_labels = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J"]

# Define weights for edges
edge_weights = [1, 2, 3, 4, 5, 6, 7, 8, 9]

# Creating vertex label and edge weight property maps
vertex_label_map = ug.new_vertex_property("string", vertex_labels)
edge_weight_map = ug.new_edge_property("int", edge_weights)

# Drawing the graph
pos_ug = graph_draw(
    ug,
    output_size=(600, 600),
    vertex_size=30,
    vertex_fill_color="white",
    vertex_color="black",
    vertex_pen_width=1.0,
    edge_pen_width=2.0,
    vertex_text=vertex_label_map,
    edge_text=edge_weight_map,
)

_, parent = shortest_path(ug, ug.vertex(0), weights=edge_weight_map)
destination = 8

path = [vertex_labels[destination]]

while parent[destination] != destination:
    destination = parent[destination]
    path.append(vertex_labels[destination])

path.reverse()
print(path)
