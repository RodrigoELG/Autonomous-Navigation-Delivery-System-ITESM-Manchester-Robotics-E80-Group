import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped#, Point 
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from a_star_map_publisher.a_star_planner import AStarPlanner

class AStarNode(Node):
    def __init__(self):
        super().__init__('a_star_node')

        self.get_logger().info('AStarNode has been started.')

        # Almacenar la última versión del mapa
        self.occupancy_grid = None
        self.map_received = False
        self.current_start_pose = None # almacenar la pose de inicio actual
        self.current_goal_pose = None  # almacenar la pose objetivo actual

        # Suscriptor al mapa
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            rclpy.qos.qos_profile_sensor_data # QoS para datos de sensores (mapa)
        )

        # Suscriptor para la pose de inicio (2D Pose Estimate en RViz)
        self.initial_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, # O PoseWithCovarianceStamped si usas el "2D Pose Estimate" más robusto
            '/initialpose', # Tópico común para la pose inicial
            self.initial_pose_callback,
            10
        )

        # Suscriptor para el objetivo (2D Nav Goal en RViz)
        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose', # Tópico común para el objetivo de navegación
            self.goal_pose_callback,
            10
        )

        # Publicador de la trayectoria planificada
        self.path_publisher = self.create_publisher(Path, '/planned_path', 1)

        self.planner = None # La instancia del AStarPlanner se creará cuando se reciba el mapa

    def map_callback(self, msg):
        """
        Callback para recibir el OccupancyGrid.
        """
        # Solo actualiza el mapa si ha cambiado o si no lo hemos recibido antes
        if self.occupancy_grid is None or self.occupancy_grid.header.stamp != msg.header.stamp:
            self.occupancy_grid = msg
            if not self.map_received:
                self.map_received = True
                # Inicializar el AStarPlanner una vez que tengamos la info del mapa
                self.planner = AStarPlanner(self.occupancy_grid.info, self.occupancy_grid.info.resolution)
                self.get_logger().info('Map received and AStarPlanner initialized.')
            # Si el mapa se actualiza, intentar replanificar si ya tenemos inicio y objetivo
            self._try_to_plan_path()

    def initial_pose_callback(self, msg):
        self.get_logger().info('*** Initial pose callback triggered! ***') # Debug line
        """
        Callback para recibir la pose de inicio desde RViz (2D Pose Estimate).
        """
        try: 
            self.current_start_pose = msg.pose.pose 
            self.get_logger().info(f'Initial pose received: X={self.current_start_pose.position.x:.2f}, Y={self.current_start_pose.position.y:.2f}')
            self._try_to_plan_path()
        except AttributeError as e:
            self.get_logger().error(f'Error in initial_pose_callback: {e} - Message type might be wrong or structure unexpected.')
            self.get_logger().error(f'Received message type: {type(msg)}') # Print type of received msg
            # print(dir(msg)) # Uncomment to see all attributes of msg
            # print(dir(msg.pose)) # Uncomment to see all attributes of msg.pose

    def goal_pose_callback(self, msg):
        self.get_logger().info('*** Goal pose callback triggered! ***') 
        """
        Callback para recibir la pose objetivo desde RViz (2D Nav Goal).
        """
        try:
            self.current_goal_pose = msg.pose # Esto debería ser correcto
            self.get_logger().info(f'Goal pose received: X={self.current_goal_pose.position.x:.2f}, Y={self.current_goal_pose.position.y:.2f}')
            self._try_to_plan_path()
        except AttributeError as e:
            self.get_logger().error(f'Error in goal_pose_callback: {e} - Message type might be wrong or structure unexpected.')
            self.get_logger().error(f'Received message type: {type(msg)}') # Print type of received msg
            # print(dir(msg)) # Uncomment to see all attributes of msg
            # print(dir(msg.pose)) # Uncomment to see all attributes of msg.pose
        

    def _try_to_plan_path(self):
        """
        Intenta planificar un camino si se ha recibido el mapa, la pose de inicio y la pose objetivo.
        """
        if not self.map_received:
            self.get_logger().info('Waiting for map...')
            return
        
        if self.planner is None:
            self.get_logger().warn('AStarPlanner not initialized. Map callback might not have been called yet.')
            return

        if self.current_start_pose is None:
            self.get_logger().info('Waiting for initial pose...')
            return

        if self.current_goal_pose is None:
            self.get_logger().info('Waiting for goal pose...')
            return

        self.get_logger().info('Planning path...')

        start_x = self.current_start_pose.position.x
        start_y = self.current_start_pose.position.y
        
        goal_x = self.current_goal_pose.position.x
        goal_y = self.current_goal_pose.position.y


        # Convertir a tuplas (x, y) para el planner
        start_coords_m = (start_x, start_y)
        goal_coords_m = (goal_x, goal_y)

        path_in_grid_coords = self.planner.plan_path(
            self.occupancy_grid.data,
            start_coords_m,
            goal_coords_m
        )

        if path_in_grid_coords:
            path_msg = Path()
            path_msg.header = Header()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = self.occupancy_grid.header.frame_id 

            for x_m, y_m in path_in_grid_coords:
                pose = PoseStamped()
                pose.header.stamp = path_msg.header.stamp
                pose.header.frame_id = path_msg.header.frame_id
                pose.pose.position.x = x_m
                pose.pose.position.y = y_m
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0 
                path_msg.poses.append(pose)
            
            self.path_publisher.publish(path_msg)
            self.get_logger().info(f'Path found with {len(path_msg.poses)} points and published.')
        else:
            self.get_logger().warn('No path found by A* planner.')
            # Publicar un camino vacío para limpiar el display en RViz si no se encuentra un camino
            empty_path_msg = Path()
            empty_path_msg.header = Header()
            empty_path_msg.header.stamp = self.get_clock().now().to_msg()
            empty_path_msg.header.frame_id = self.occupancy_grid.header.frame_id
            self.path_publisher.publish(empty_path_msg)

def main(args=None):
    rclpy.init(args=args)
    a_star_node = AStarNode()
    rclpy.spin(a_star_node)
    a_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
