import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt

# Definir la ubicación y descargar la red de calles
ubicacion = "Xalapa, Veracruz, Mexico"
grafo_calles = ox.graph_from_place(ubicacion, network_type="drive")

# Seleccionar nodos de inicio y fin usando coordenadas
inicio = ox.distance.nearest_nodes(grafo_calles, X = -96.903537, Y = 19.527566)
fin = ox.distance.nearest_nodes(grafo_calles, X = -96.881061, Y = 19.518218)

# Implementación de Dijkstra con conteo de iteraciones
iteraciones_dijkstra = 0

def dijkstra_iterativo(grafo, inicio, fin):
    global iteraciones_dijkstra
    iteraciones_dijkstra = 0
    ruta = nx.shortest_path(grafo, source=inicio, target=fin, weight='length')
    iteraciones_dijkstra = len(ruta) - 1
    return ruta

ruta_dijkstra = dijkstra_iterativo(grafo_calles, inicio, fin)
distancia_dijkstra = nx.shortest_path_length(grafo_calles, source=inicio, target=fin, weight='length')

fig, ax = ox.plot_graph_route(grafo_calles, route=ruta_dijkstra, route_color='blue', route_linewidth=4, node_size=0, bgcolor='k')
print(f'\nRuta de Dijkstra: {ruta_dijkstra}')
print(f'\nDistancia total (en metros): {distancia_dijkstra}')
print(f'\nIteraciones Dijkstra: {iteraciones_dijkstra}')

# Implementación de A* con conteo de iteraciones y tiempo de ejecución
def heuristica(nodo1, nodo2):
    x1, y1 = grafo_calles.nodes[nodo1]['x'], grafo_calles.nodes[nodo1]['y']
    x2, y2 = grafo_calles.nodes[nodo2]['x'], grafo_calles.nodes[nodo2]['y']
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5  # Distancia Euclidiana

# Implementación de A* con conteo de iteraciones
iteraciones_astar = 0

def astar_iterativo(grafo, inicio, fin, heuristic):
    global iteraciones_astar
    iteraciones_astar = 0
    ruta = nx.astar_path(grafo, inicio, fin, heuristic=heuristic)
    iteraciones_astar = len(ruta) - 1
    return ruta

camino_astar = astar_iterativo(grafo_calles, inicio, fin, heuristica)
distancia_astar = sum(grafo_calles.edges[camino_astar[i], camino_astar[i + 1], 0]['length'] for i in range(len(camino_astar) - 1))

fig, ax = ox.plot_graph_route(grafo_calles, camino_astar, route_linewidth=4, node_size=0, bgcolor='k')
plt.show()

print(f'\nRuta de A*: {camino_astar}')
print(f'\nDistancia total (en metros): {distancia_astar}')
print(f'\nIteraciones A*: {iteraciones_astar}')
