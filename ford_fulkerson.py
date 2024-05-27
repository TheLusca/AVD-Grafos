import random
from collections import defaultdict
import networkx as nx
import matplotlib.pyplot as plt

class PipeNetwork:
    def __init__(self):
        self.graph = defaultdict(int)
        self.residual_graph = defaultdict(int)

    def add_pipe(self, source, target, capacity):
        self.graph[source, target] = capacity
        self.residual_graph[source, target] = capacity
        self.residual_graph[target, source] = 0

    def bfs(self, source, sink, parent):
        visited = set()
        queue = [source]
        visited.add(source)

        while queue:
            u = queue.pop(0)

            for (u2, v), capacity in self.residual_graph.items():
                if u2 == u and v not in visited and capacity > 0:
                    queue.append(v)
                    visited.add(v)
                    parent[v] = u
                    if v == sink:
                        return True
        return False

    def max_flow(self, source, sink):
        parent = {}
        max_flow = 0
        max_flow_path = []

        while self.bfs(source, sink, parent):
            path_flow = float('Inf')
            s = sink
            max_flow_path = [s]

            while s != source:
                path_flow = min(path_flow, self.residual_graph[parent[s], s])
                s = parent[s]
                max_flow_path.append(s)

            max_flow_path.reverse()

            v = sink
            while v != source:
                u = parent[v]
                self.residual_graph[u, v] -= path_flow
                self.residual_graph[v, u] += path_flow
                v = parent[v]

            max_flow += path_flow

        return max_flow, max_flow_path

def plot_network(network, max_flow_path=None):
    G = nx.DiGraph()
    edge_labels = {}
    for (u, v), capacity in network.graph.items():
        G.add_edge(u, v, capacity=capacity)
        edge_labels[(u, v)] = f'{capacity}'

    pos = nx.spring_layout(G)
    plt.figure(figsize=(10, 7))
    nx.draw(G, pos, with_labels=True, node_color='skyblue', node_size=1500, arrowstyle='-|>', arrowsize=20)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red')
    
    if max_flow_path:
        edges = [(max_flow_path[i], max_flow_path[i + 1]) for i in range(len(max_flow_path) - 1)]
        nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color='green', width=2)
        
    plt.title('Pipe Network Graph')
    plt.show()


def main():
    teste = PipeNetwork()
    teste.add_pipe(1, 2, 16)
    teste.add_pipe(1, 3, 13)
    teste.add_pipe(2, 3, 10)
    teste.add_pipe(3, 2, 4)
    teste.add_pipe(2, 4, 12)
    teste.add_pipe(3, 5, 14)
    teste.add_pipe(4, 3, 9)
    teste.add_pipe(5, 4, 7)
    teste.add_pipe(5, 6, 4)
    teste.add_pipe(4, 6, 20)
    
    max_flow, max_flow_path = teste.max_flow(1, 6)
    print(f"Specific Example: Maximum Flow 1 - 6: {max_flow}")
    plot_network(teste, max_flow_path)

if __name__ == "__main__":
    main()
