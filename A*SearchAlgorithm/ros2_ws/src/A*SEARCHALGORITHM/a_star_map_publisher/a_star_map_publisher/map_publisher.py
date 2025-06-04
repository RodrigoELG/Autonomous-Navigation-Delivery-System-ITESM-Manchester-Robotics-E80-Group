import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import numpy as np
from scipy.ndimage import binary_dilation

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # --- Parámetros del Mapa ---
        self.resolution = 0.05  # 5 cm por celda
        self.map_frame_id = 'map' # Frame de coordenadas para el mapa

        # Dimensiones del área de juego (rectángulo delimitador del mapa en el sistema ABSOLUTO)
        # Origen (0,0) en la esquina inferior izquierda del rectángulo de 2.1m x 2.2m
        self.map_width_m = 2.1
        self.map_height_m = 2.2

        self.grid_width = int(round(self.map_width_m / self.resolution))
        self.grid_height = int(round(self.map_height_m / self.resolution))

        # El origen (position.x, position.y) de OccupancyGrid es la esquina INFERIOR IZQUIERDA del grid.
        # En este caso, será (0.0, 0.0) porque estamos trabajando con el sistema absoluto.
        self.grid_origin_x_ros = 0.0
        self.grid_origin_y_ros = 0.0

        # --- Footprint del PuzzleBot para Inflación ---
        self.inflation_radius = 0.15 # metros (radio alrededor del robot para evitar obstáculos)
        
        selem_size = int(round(self.inflation_radius / self.resolution)) * 2 + 1
        x, y = np.meshgrid(np.arange(selem_size), np.arange(selem_size))
        dist_sq = (x - selem_size//2)**2 + (y - selem_size//2)**2
        self.selem = dist_sq <= (self.inflation_radius / self.resolution)**2
        self.selem = self.selem.astype(bool)

        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 1)
        self.timer_ = self.create_timer(1.0, self.publish_map) # Publica el mapa cada 1 segundo

        self.get_logger().info(f'Map publisher node started. Grid dimensions: {self.grid_width}x{self.grid_height} cells.')
        self.get_logger().info(f'Map origin in RViz will be at ({self.grid_origin_x_ros}, {self.grid_origin_y_ros})')

    def abs_coord_to_grid_idx(self, x_abs, y_abs):
        """
        Convierte coordenadas ABSOLUTAS (X+ derecha, Y+ arriba, origen en (0,0) inferior izquierda)
        a índices de celda (col, row) para el numpy array del OccupancyGrid.
        """
        col = int((x_abs - self.grid_origin_x_ros) / self.resolution)
        row = int((y_abs - self.grid_origin_y_ros) / self.resolution)

        # Asegurarse de que los índices estén dentro de los límites
        col = max(0, min(col, self.grid_width - 1))
        row = max(0, min(row, self.grid_height - 1))
        return col, row

    def mark_rectangle_on_grid(self, grid_2d_array, x_min_abs, y_min_abs, x_max_abs, y_max_abs, value):
        """
        Marca un rectángulo en el grid_2d_array (numpy).
        Coordenadas x_min/y_min/x_max/y_max están en el sistema ABSOLUTO
        (X+ derecha, Y+ arriba, origen en (0,0) inferior izquierda).
        """
        # Convertir las coordenadas de las esquinas a índices de celda
        col_min, row_min = self.abs_coord_to_grid_idx(x_min_abs, y_min_abs)
        col_max, row_max = self.abs_coord_to_grid_idx(x_max_abs, y_max_abs)

        # Asegurarse de que los rangos estén ordenados para la indexación de NumPy
        col_start = min(col_min, col_max)
        col_end = max(col_min, col_max)
        row_start = min(row_min, row_max)
        row_end = max(row_min, row_max)

        # Asegurarse que los índices estén dentro de los límites del grid
        col_start = max(0, col_start)
        col_end = min(self.grid_width - 1, col_end)
        row_start = max(0, row_start)
        row_end = min(self.grid_height - 1, row_end)

        if col_start <= col_end and row_start <= row_end:
            grid_2d_array[row_start : row_end + 1, col_start : col_end + 1] = value
        
        # self.get_logger().info(f"Marked rect (abs coords): ({x_min_abs}, {y_min_abs}) to ({x_max_abs}, {y_max_abs}) -> cols {col_start}-{col_end}, rows {row_start}-{row_end}")


    def publish_map(self):
        # 1. Inicializar el grid como un array NumPy 2D
        # numpy_grid[row, col]. row=0 es la fila más inferior (Y_min), col=0 es la columna más a la izquierda (X_min)
        map_data_2d = np.full((self.grid_height, self.grid_width), 0, dtype=np.int8) # 0 = libre

        # 2. Marcar los obstáculos internos (en el sistema ABSOLUTO)
        # Rectángulo (x_min, y_min, x_max, y_max)
        
        # Obstáculo Superior Izquierdo: (0.47, 1.42, 0.77, 1.72)
        self.mark_rectangle_on_grid(map_data_2d, 0.47, 1.42, 0.77, 1.72, 100)

        # Obstáculo Inferior Izquierdo: (0.47, 0.67, 0.77, 0.97)
        self.mark_rectangle_on_grid(map_data_2d, 0.47, 0.67, 0.77, 0.97, 100)

        # Obstáculo Superior Derecho: (1.115, 1.295, 1.515, 1.595)
        self.mark_rectangle_on_grid(map_data_2d, 1.115, 1.295, 1.515, 1.595, 100)

        # Obstáculo Inferior Derecho: (1.115, 0.40, 1.515, 0.70)
        self.mark_rectangle_on_grid(map_data_2d, 1.115, 0.40, 1.515, 0.70, 100)
        
        # 3. Marcar los bordes del mapa (en el sistema ABSOLUTO)
        # Usamos un ancho de 'resolution' para las líneas del borde
        
        # Borde Superior
        self.mark_rectangle_on_grid(map_data_2d, 0.0, self.map_height_m - self.resolution, self.map_width_m, self.map_height_m, 100)
        # Borde Inferior
        self.mark_rectangle_on_grid(map_data_2d, 0.0, 0.0, self.map_width_m, self.resolution, 100)
        # Borde Izquierdo
        self.mark_rectangle_on_grid(map_data_2d, 0.0, 0.0, self.resolution, self.map_height_m, 100)
        # Borde Derecho
        self.mark_rectangle_on_grid(map_data_2d, self.map_width_m - self.resolution, 0.0, self.map_width_m, self.map_height_m, 100)

        # Marcar la región del "indent" (parte superior derecha que no es navegable)
        # Este es el rectángulo de (1.8, 1.9) a (2.1, 2.2) en coordenadas absolutas.
        self.mark_rectangle_on_grid(map_data_2d, 1.8, 1.9, 2.1, 2.2, 100)


        # 4. Aplicar inflación de obstáculos usando SciPy binary_dilation
        obstacles_bool = (map_data_2d == 100)
        inflated_obstacles_bool = binary_dilation(obstacles_bool, structure=self.selem)
        
        map_data_inflated = np.where(inflated_obstacles_bool, 100, 0).astype(np.int8)


        # 5. Crear el mensaje OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = self.map_frame_id

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height

        # El origen del OccupancyGrid es la esquina inferior izquierda del grid
        # en el frame 'map'. Aquí lo establecemos en (0,0) del sistema absoluto.
        grid_msg.info.origin.position.x = self.grid_origin_x_ros
        grid_msg.info.origin.position.y = self.grid_origin_y_ros
        grid_msg.info.origin.position.z = 0.0
        
        # Orientación de identidad para el frame 'map' (X+ derecha, Y+ arriba)
        # Esto es el estándar en ROS y RViz.
        grid_msg.info.origin.orientation.x = 0.0
        grid_msg.info.origin.orientation.y = 0.0
        grid_msg.info.origin.orientation.z = 0.0 
        grid_msg.info.origin.orientation.w = 1.0 # Cuaternión de identidad

        grid_msg.data = map_data_inflated.flatten().tolist()

        self.publisher_.publish(grid_msg)
        self.get_logger().info('Published static map in standard ROS/RViz orientation.')

def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()