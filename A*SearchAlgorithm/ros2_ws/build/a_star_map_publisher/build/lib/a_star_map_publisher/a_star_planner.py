import numpy as np
import heapq # Para la cola de prioridad (min-heap)

class Node:
    """
    Clase para representar un nodo en el grid para el algoritmo A*.
    Cada nodo corresponde a una celda del OccupancyGrid.
    """
    def __init__(self, x, y, cost_g=0.0, cost_h=0.0, parent=None):
        self.x = x # Coordenada x de la celda (columna)
        self.y = y # Coordenada y de la celda (fila)
        self.cost_g = cost_g # Costo real desde el inicio hasta este nodo
        self.cost_h = cost_h # Costo heurístico desde este nodo hasta el objetivo
        self.cost_f = self.cost_g + self.cost_h # Costo total (f = g + h)
        self.parent = parent # Nodo padre en el camino óptimo

    def __lt__(self, other):
        """
        Necesario para comparar nodos en la cola de prioridad (heapq).
        Los nodos con menor costo_f tendrán mayor prioridad.
        """
        return self.cost_f < other.cost_f

    def __eq__(self, other):
        """
        Necesario para comparar nodos (para verificar si ya está en la lista).
        Dos nodos son iguales si tienen las mismas coordenadas.
        """
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        """
        Necesario para usar nodos como claves en diccionarios o sets (lista cerrada).
        """
        return hash((self.x, self.y))

class AStarPlanner:
    """
    Implementación del algoritmo de búsqueda A*.
    """
    def __init__(self, occupancy_grid_info, resolution):
        """
        Inicializa el planificador A*.
        :param occupancy_grid_info: nav_msgs/OccupancyGrid.info del mapa.
        :param resolution: Resolución del mapa en metros/celda.
        """
        self.width = occupancy_grid_info.width
        self.height = occupancy_grid_info.height 
        self.resolution = resolution
        
        # Origen del OccupancyGrid en el frame del mapa (inferior izquierda)
        self.origin_x = occupancy_grid_info.origin.position.x
        self.origin_y = occupancy_grid_info.origin.position.y

        # Definir los movimientos posibles (8 direcciones)
        # (dx, dy, cost)
        self.movements = [
            (0, 1, self.resolution),     # Abajo (ROS Y+)
            (0, -1, self.resolution),    # Arriba (ROS Y-)
            (1, 0, self.resolution),     # Derecha (ROS X+)
            (-1, 0, self.resolution),    # Izquierda (ROS X-)
            (1, 1, self.resolution * np.sqrt(2)),   # Diagonal abajo-derecha
            (1, -1, self.resolution * np.sqrt(2)),  # Diagonal arriba-derecha
            (-1, 1, self.resolution * np.sqrt(2)),  # Diagonal abajo-izquierda
            (-1, -1, self.resolution * np.sqrt(2))  # Diagonal arriba-izquierda
        ]

    def _calculate_heuristic(self, node, goal_node):
        """
        Calcula la heurística (distancia euclidiana) entre el nodo actual y el objetivo.
        """
        return np.sqrt((node.x - goal_node.x)**2 + (node.y - goal_node.y)**2) * self.resolution

    def plan_path(self, occupancy_grid_data, start_coords, goal_coords):
        """
        Encuentra un camino desde el punto de inicio al punto objetivo usando A*.
        :param occupancy_grid_data: Lista 1D de datos del OccupancyGrid (0-100, -1).
        :param start_coords: (x, y) en metros del punto de inicio.
        :param goal_coords: (x, y) en metros del punto objetivo.
        :return: Una lista de (x, y) en metros representando el camino, o None si no se encuentra.
        """
        # Convertir coordenadas de inicio y objetivo a índices de celda
        start_col, start_row = self._coords_to_grid_idx(start_coords[0], start_coords[1])
        goal_col, goal_row = self._coords_to_grid_idx(goal_coords[0], goal_coords[1])

        start_node = Node(start_col, start_row)
        goal_node = Node(goal_col, goal_row)

        # Verificar si el inicio o el objetivo están en un obstáculo
        if self._is_obstacle(occupancy_grid_data, start_node.x, start_node.y) or \
           self._is_obstacle(occupancy_grid_data, goal_node.x, goal_node.y):
            print("Inicio o objetivo están en un obstáculo.")
            return None

        open_list = [] # Cola de prioridad para los nodos a explorar
        heapq.heappush(open_list, start_node)

        closed_list = set() # Nodos ya explorados
        
        # Diccionario para almacenar el nodo padre de cada celda y su costo_g
        # Esto es más eficiente que buscar en open_list o closed_list completos.
        came_from = {}
        g_score = {start_node: 0} # Costo de g para cada nodo
        f_score = {start_node: self._calculate_heuristic(start_node, goal_node)} # Costo de f para cada nodo

        while open_list:
            current_node = heapq.heappop(open_list)

            if current_node == goal_node:
                # Se encontró el camino, reconstruirlo
                return self._reconstruct_path(came_from, current_node)

            closed_list.add(current_node)

            # Explorar vecinos
            for dx, dy, cost in self.movements:
                neighbor_x, neighbor_y = current_node.x + dx, current_node.y + dy

                # Verificar límites del grid
                if not (0 <= neighbor_x < self.width and 0 <= neighbor_y < self.height):
                    continue

                # Verificar si es un obstáculo o ya está en la lista cerrada
                if self._is_obstacle(occupancy_grid_data, neighbor_x, neighbor_y) or \
                   Node(neighbor_x, neighbor_y) in closed_list:
                    continue

                # Calcular el costo g para el vecino
                tentative_g_score = g_score[current_node] + cost

                neighbor_node = Node(neighbor_x, neighbor_y)

                if neighbor_node not in [node for node in open_list] or tentative_g_score < g_score.get(neighbor_node, float('inf')):
                    # Este camino al vecino es mejor o es un nuevo nodo
                    neighbor_node.parent = current_node
                    g_score[neighbor_node] = tentative_g_score
                    neighbor_node.cost_g = tentative_g_score # Actualizar el costo_g del nodo
                    neighbor_node.cost_h = self._calculate_heuristic(neighbor_node, goal_node)
                    neighbor_node.cost_f = neighbor_node.cost_g + neighbor_node.cost_h
                    
                    came_from[neighbor_node] = current_node # Almacenar el padre
                    
                    if neighbor_node not in [node for node in open_list]:
                        heapq.heappush(open_list, neighbor_node)

        print("No se encontró un camino.")
        return None # No se encontró un camino

    def _reconstruct_path(self, came_from, current_node):
        """
        Reconstruye el camino desde el nodo objetivo hasta el nodo de inicio.
        """
        path = []
        while current_node in came_from:
            path.append((current_node.x, current_node.y)) # Coordenadas de celda
            current_node = came_from[current_node]
        path.append((current_node.x, current_node.y)) # Agregar el nodo de inicio
        
        # Convertir índices de celda a coordenadas de mundo (metros)
        world_path = []
        for col, row in reversed(path): # Revertir para ir del inicio al objetivo
            x_world, y_world = self._grid_idx_to_coords(col, row)
            world_path.append((x_world, y_world))
        return world_path

    def _is_obstacle(self, occupancy_grid_data, col, row):
        """
        Verifica si una celda es un obstáculo.
        """
        if not (0 <= col < self.width and 0 <= row < self.height):
            return True # Fuera de límites es un obstáculo virtual
        
        # ROS OccupancyGrid.data es una lista 1D, row-major order
        # grid_idx = row * width + col
        idx = int(row * self.width + col)
        if 0 <= idx < len(occupancy_grid_data):
            # Valor 100 es obstáculo, -1 es desconocido (considerar como obstáculo), 0 es libre.
            return occupancy_grid_data[idx] > 50 # Considerar >50 como obstáculo
        return True # Fuera de rango de datos también es obstáculo

    def _coords_to_grid_idx(self, x_world, y_world):
        """
        Convierte coordenadas de mundo (metros) a índices de celda (col, row).
        Asume que x_world, y_world están en el sistema de coordenadas de ROS
        (X+ derecha, Y+ arriba, origen en grid_origin_x_ros, grid_origin_y_ros).
        """
        col = int((x_world - self.origin_x) / self.resolution)
        row = int((y_world - self.origin_y) / self.resolution)
        return col, row

    def _grid_idx_to_coords(self, col, row):
        """
        Convierte índices de celda (col, row) a coordenadas de mundo (metros).
        Retorna la coordenada central de la celda.
        """
        x_world = self.origin_x + (col + 0.5) * self.resolution
        y_world = self.origin_y + (row + 0.5) * self.resolution
        return x_world, y_world