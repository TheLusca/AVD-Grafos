import heapq
import time

# Função para heurística (distância estimada entre dois pontos)
def heuristic(node, goal):
    # Exemplo de heurística: Distância Manhattan
    coord = {
        'A': (0, 0),
        'B': (1, 0),
        'C': (0, 1),
        'D': (2, 0),
        'E': (1, 1),
        'F': (0, 2),
        'G': (1, 2),
        'H': (2, 1),
        'I': (1, 3),
        'J': (2, 2),
        'K': (1, 4),
        'L': (3, 2)
    }
    return abs(coord[node][0] - coord[goal][0]) + abs(coord[node][1] - coord[goal][1])

# Função para o algoritmo A*
def a_star(graph, start, goal):
    # Inicializa a pontuação do caminho mais curto (g_score) e a estimativa para o destino (f_score)
    g_score = {node: float('inf') for node in graph.keys()}
    f_score = {node: float('inf') for node in graph.keys()}
    previous = {node: None for node in graph.keys()}

    g_score[start] = 0
    f_score[start] = heuristic(start, goal)

    open_set = [(f_score[start], start)]

    while open_set:
        _, current_node = heapq.heappop(open_set)

        if current_node == goal:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = previous[current_node]
            return path[::-1], g_score[goal]

        for neighbor, weight in graph[current_node]:
            tentative_g_score = g_score[current_node] + weight
            if tentative_g_score < g_score[neighbor]:
                previous[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return [], float('inf')  # Se não encontrar um caminho

# Grafo para teste
graph = {
    'A': [('B', 1), ('C', 3)],
    'B': [('D', 2), ('E', 4)],
    'C': [('F', 5), ('G', 1)],
    'D': [('H', 1)],
    'E': [('H', 3), ('I', 2)],
    'F': [('J', 1)],
    'G': [('J', 3), ('K', 4)],
    'H': [('L', 2)],
    'I': [('L', 1)],
    'J': [('L', 1)],
    'K': [('L', 1)],
    'L': []
}

# Medir o tempo com `perf_counter`
repeticoes = 100  # Para aumentar a carga do algoritmo

start_time = time.perf_counter()  # Medir com precisão

for _ in range(repeticoes):
    start_node = 'A'
    end_node = 'L'
    path, distance = a_star(graph, start_node, end_node)

end_time = time.perf_counter()  # Parar a medição

# Tempo em milissegundos
execution_time_ms = (end_time - start_time) * 1000  # Converter para milissegundos

# Tempo total dividido pelo número de repetições
average_execution_time = execution_time_ms / repeticoes

print("Caminho mais curto usando A*:", path)
print("Distância:", distance)
print("Tempo médio de processamento: {:.3f} ms".format(average_execution_time))
