import heapq
import time
import matplotlib.pyplot as plt
import numpy as np


class Grafo:
    def __init__(self):
        self.vertices = {}  # Inicializa o dicionário de vértices

    def adicionar_vertice(self, vertice):
        if vertice not in self.vertices:
            self.vertices[vertice] = {}

    def adicionar_aresta(self, origem, destino, peso):
        self.adicionar_vertice(origem)
        self.adicionar_vertice(destino)
        self.vertices[origem][destino] = peso

    def dijkstra(self, origem, destino):

        # Dicionário para manter o custo mínimo de alcançar cada vértice a partir da origem
        custo_minimo = {vertice: float('inf') for vertice in self.vertices}
        custo_minimo[origem] = 0

        # Fila de prioridade para selecionar o vértice com o menor custo
        fila = [(0, origem)]

        while fila:
            custo_atual, vertice_atual = heapq.heappop(fila)

            # Se já encontramos um caminho mais curto para esse vértice, ignoramos
            if custo_atual > custo_minimo[vertice_atual]:
                continue

            # Verifica os vizinhos do vértice atual
            for vizinho, peso in self.vertices[vertice_atual].items():
                custo_total = custo_atual + peso
                # Atualiza o custo mínimo se encontramos um caminho mais curto
                if custo_total < custo_minimo[vizinho]:
                    custo_minimo[vizinho] = custo_total
                    heapq.heappush(fila, (custo_total, vizinho))

        # Retorna o custo mínimo para alcançar o destino
        return custo_minimo[destino]

# Função de comparação para usar o A* - heurística é a distância em linha reta entre os vértices


def distancia_heuristica(grafo, origem, destino):
    coordenadas = {'A': (0, 0), 'B': (3, 1), 'C': (5, 2), 'D': (
        4, 4), 'E': (1, 5), 'F': (2, 5), 'G': (5, 5), 'H': (5, 8)}
    x1, y1 = coordenadas[origem]
    x2, y2 = coordenadas[destino]
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


def a_estrela(grafo, origem, destino):
    # Dicionário para manter o custo mínimo de alcançar cada vértice a partir da origem
    custo_minimo = {vertice: float('inf') for vertice in grafo.vertices}
    custo_minimo[origem] = 0

    # Fila de prioridade para selecionar o vértice com o menor custo
    fila = [(0, origem)]

    while fila:
        custo_atual, vertice_atual = heapq.heappop(fila)

        # Se já encontramos um caminho mais curto para esse vértice, ignoramos
        if custo_atual > custo_minimo[vertice_atual]:
            continue

        # Verifica os vizinhos do vértice atual
        for vizinho, peso in grafo.vertices[vertice_atual].items():
            custo_total = custo_minimo[vertice_atual] + peso
            # Atualiza o custo mínimo se encontramos um caminho mais curto
            if custo_total < custo_minimo[vizinho]:
                custo_minimo[vizinho] = custo_total
                heapq.heappush(
                    fila, (custo_total + distancia_heuristica(grafo, vizinho, destino), vizinho))

    # Retorna o custo mínimo para alcançar o destino
    return custo_minimo[destino]


# Criar o grafo com as cidades e distâncias
grafo = Grafo()
grafo.adicionar_aresta('A', 'B', 10)
grafo.adicionar_aresta('A', 'C', 15)
grafo.adicionar_aresta('B', 'D', 12)
grafo.adicionar_aresta('B', 'E', 15)
grafo.adicionar_aresta('C', 'E', 10)
grafo.adicionar_aresta('D', 'E', 2)
grafo.adicionar_aresta('E', 'F', 8)
grafo.adicionar_aresta('E', 'G', 6)
grafo.adicionar_aresta('F', 'G', 5)
grafo.adicionar_aresta('G', 'H', 15)
# Testar os algoritmos e medir o tempo de execução
num_execucoes = 10000
tempos_dijkstra = []
tempos_a_estrela = []

# Executar os algoritmos várias vezes para medir o desempenho
for _ in range(num_execucoes):
    inicio_dijkstra = time.perf_counter()
    grafo.dijkstra('A', 'F')
    fim_dijkstra = time.perf_counter()
    print(f'{(fim_dijkstra - inicio_dijkstra)*1000},  dijkstra')
    tempos_dijkstra.append((fim_dijkstra - inicio_dijkstra)
                           * 1000)  # Tempo em milissegundos


for _ in range(num_execucoes):

    inicio_a_estrela = time.perf_counter()
    a_estrela(grafo, 'A', 'F')
    fim_a_estrela = time.perf_counter()

    print(f'{(fim_a_estrela - inicio_a_estrela) * 1000},  Estrela')
    # Tempo em milissegundos
    tempos_a_estrela.append((fim_a_estrela - inicio_a_estrela) * 1000)

# Calcular a média dos tempos de execução
media_dijkstra = np.sum(tempos_dijkstra)
media_a_estrela = np.sum(tempos_a_estrela)

# Imprimir os resultados
print("Desempenho do Dijkstra (média de {} execuções): {:.6f} milissegundos".format(
    num_execucoes, media_dijkstra))
print("Desempenho do A* (média de {} execuções): {:.6f} milissegundos".format(num_execucoes, media_a_estrela))

# Plotar um gráfico comparativo do desempenho
plt.bar(['Dijkstra', 'A*'], [media_dijkstra,
        media_a_estrela], color=['blue', 'green'])
plt.xlabel('Algoritmo')
plt.ylabel('Tempo médio de execução (ms)')
plt.title('Desempenho de Dijkstra vs A*')
plt.show()


plt.plot(range(1, num_execucoes + 1), tempos_dijkstra, label='Dijkstra')
plt.plot(range(1, num_execucoes + 1), tempos_a_estrela, label='A*')
plt.xlabel('Execução')
plt.ylabel('Tempo (ms)')
plt.title('Tempo de execução dos algoritmos')
plt.legend()
plt.grid(True)
plt.show()


plt.scatter(range(1, num_execucoes + 1), tempos_dijkstra, label='Dijkstra')
plt.scatter(range(1, num_execucoes + 1), tempos_a_estrela, label='A*')
plt.xlabel('Execução')
plt.ylabel('Tempo (ms)')
plt.title('Distribuição dos tempos de execução dos algoritmos')
plt.legend()
plt.grid(True)
plt.show()