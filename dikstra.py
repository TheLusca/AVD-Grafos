import heapq
import time

# Função para o algoritmo de Dijkstra
def dijkstra(graph, start):
    # Inicializa a distância para todos os nós como infinito
    distances = {node: float('inf') for node in graph.keys()}  # Variável "node" definida no escopo correto
    # Inicializa o dicionário para armazenar o nó anterior para cada nó
    previous = {node: None for node in graph.keys()}  # Corrigir erro no uso da variável "node"

    distances[start] = 0  # O nó inicial tem distância zero
    priority_queue = [(0, start)]  # Fila de prioridade para o algoritmo Dijkstra

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # Se a distância extraída é maior do que a conhecida, continue
        if current_distance > distances[current_node]:
            continue

        # Para cada vizinho, atualize as distâncias se encontrar um caminho mais curto
        for neighbor, weight in graph[current_node]:
            new_distance = current_distance + weight
            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                previous[neighbor] = current_node
                heapq.heappush(priority_queue, (new_distance, neighbor))  # Adicionar à fila de prioridade

    return distances, previous  # Retorna os resultados do algoritmo Dijkstra

# Função para obter o caminho mais curto do algoritmo de Dijkstra
def shortest_path(graph, start, end):
    distances, previous = dijkstra(graph, start)
    
    # Reconstruir o caminho mais curto do final ao início
    path = []
    current_node = end
    while current_node is not None:
        path.append(current_node)
        current_node = previous[current_node]  # Caminha para trás

    return path[::-1], distances[end]  # Retorna o caminho e a distância do caminho mais curto

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

# Medir o tempo para encontrar o caminho mais curto com Dijkstra
repeticoes = 10000  # Para aumentar a precisão da medição

start_time = time.perf_counter()  # Medir com alta precisão

# Executar o algoritmo várias vezes para melhorar a precisão da medição
for _ in range(repeticoes):
    start_node = 'A'
    end_node = 'L'
    path, distance = shortest_path(graph, start_node, end_node)

end_time = time.perf_counter()  # Fim da medição

# Converter para milissegundos
execution_time_ms = (end_time - start_time) * 1000

# Tempo médio por repetição
average_execution_time = execution_time_ms / repeticoes

print("Caminho mais curto com Dijkstra:", path)
print("Distância:", distance)
print("Tempo médio de processamento: {:.3f} ms".format(average_execution_time))
