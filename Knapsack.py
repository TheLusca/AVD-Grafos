import networkx as nx
import matplotlib.pyplot as plt

# Create the graph
G = nx.DiGraph()
G.add_edge("x", "a", capacity=3.0)
G.add_edge("a", "x", capacity=1.0)
G.add_edge("x", "b", capacity=1.0)
G.add_edge("a", "c", capacity=3.0)
G.add_edge("b", "c", capacity=5.0)
G.add_edge("b", "d", capacity=4.0)
G.add_edge("d", "e", capacity=2.0)
G.add_edge("c", "y", capacity=2.0)
G.add_edge("e", "y", capacity=3.0)

# Draw the graph
pos = nx.spring_layout(G)  # Position nodes using the spring layout algorithm
nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=1500, arrowsize=20)
labels = nx.get_edge_attributes(G, 'capacity')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)

# Show the plot
plt.title('Directed Graph')
plt.show()
