from graph_tool.all import *

# Creating an Undirected Graph
ug = Graph(directed=False)

# Adding N Vertices
N = 7 * 7
ug.add_vertex(N)

# Adding List of Edges with Source and Destination Vertex in Tuples
for i in range(7):
    for j in range(7):
        vertex_label = i * 7 + j
        if i < 6:
            ug.add_edge(vertex_label, (i + 1) * 7 + j)
        if j < 6:
            ug.add_edge(vertex_label, i * 7 + (j + 1))

# Adding List of Labels that needs to be provided for Vertex
vertex_label_map = ug.new_vertex_property('int', [i for i in range(N)])

# Adding List of Weights that needs to be provided for Edge
edge_weight_map = ug.new_edge_property("int", [1 for _ in range(2 * (N - 7))])

# Drawing the graph
pos_ug = graph_draw(ug,
                    output_size=(600, 600),
                    vertex_size=30,
                    vertex_fill_color='white',
                    vertex_color='black',
                    vertex_pen_width=1.0,
                    edge_pen_width=2.0,
                    vertex_text=vertex_label_map,
                    edge_text=edge_weight_map)

_, parent = dijkstra_search(ug, weight=edge_weight_map, source=ug.vertex(0))
destination = 48
path = [vertex_label_map[destination]]

while parent[destination] != destination:
    destination = parent[destination]
    path.append(vertex_label_map[destination])
path.reverse()
print(path)
