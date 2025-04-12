import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class FramePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')
        
        # Pose inicial del robot
        self.initial_pos_x = 1.0
        self.initial_pos_y = 1.0
        self.initial_pos_z = 0.0
        self.initial_pos_yaw = 0.0
        
        # Velocidad angular (para simular giro y movimiento)
        self.omega = 0.5

        # Crear broadcasters de transformaciones
        self.br_map2odom = TransformBroadcaster(self)
        self.br_odom2footprint = TransformBroadcaster(self)

        # Publisher de estados de juntas (ruedas)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Inicializar mensaje JointState
        self.joint_msg = JointState()
        self.joint_msg.name = ['wheel_left_joint', 'wheel_right_joint']
        self.joint_msg.position = [0.0, 0.0]
        self.joint_msg.velocity = []
        self.joint_msg.effort = []

        # Timer que actualiza transformaciones y joint states
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9

        # ----- map -> odom (estático en este caso) -----
        tf_map_odom = TransformStamped()
        tf_map_odom.header.stamp = now.to_msg()
        tf_map_odom.header.frame_id = 'map'
        tf_map_odom.child_frame_id = 'odom'
        tf_map_odom.transform.translation.x = 0.0
        tf_map_odom.transform.translation.y = 0.0
        tf_map_odom.transform.translation.z = 0.0
        q_map = transforms3d.euler.euler2quat(0, 0, 0)
        tf_map_odom.transform.rotation.x = q_map[1]
        tf_map_odom.transform.rotation.y = q_map[2]
        tf_map_odom.transform.rotation.z = q_map[3]
        tf_map_odom.transform.rotation.w = q_map[0]
        self.br_map2odom.sendTransform(tf_map_odom)

        # ----- odom -> base_footprint (moviéndose en círculo) -----
        tf_odom_footprint = TransformStamped()
        tf_odom_footprint.header.stamp = now.to_msg()
        tf_odom_footprint.header.frame_id = 'odom'
        tf_odom_footprint.child_frame_id = 'base_footprint'
        tf_odom_footprint.transform.translation.x = self.initial_pos_x + 0.5 * np.cos(self.omega * t)
        tf_odom_footprint.transform.translation.y = self.initial_pos_y + 0.5 * np.sin(self.omega * t)
        tf_odom_footprint.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, self.initial_pos_yaw + self.omega * t)
        tf_odom_footprint.transform.rotation.x = q[1]
        tf_odom_footprint.transform.rotation.y = q[2]
        tf_odom_footprint.transform.rotation.z = q[3]
        tf_odom_footprint.transform.rotation.w = q[0]
        self.br_odom2footprint.sendTransform(tf_odom_footprint)

        # ----- JointStates para las ruedas -----
        self.joint_msg.header.stamp = now.to_msg()
        self.joint_msg.position[0] = self.omega * t
        self.joint_msg.position[1] = -self.omega * t
        self.joint_pub.publish(self.joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
