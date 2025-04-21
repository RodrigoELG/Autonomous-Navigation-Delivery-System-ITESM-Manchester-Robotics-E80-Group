import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np
from nav_msgs.msg import Odometry

class FramePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')
        
        # Initial position and orientation
        self.initial_pos_x = 0.0
        self.initial_pos_y = 0.0
        self.initial_pos_yaw = 0.0
        
        # Robot parameters
        self.r = 0.05   # Wheel radius
        self.l = 0.19   # Distance between wheels
        self.dt = 0.05  # Timer period

        # Motion state
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0

        # Transform broadcasters
        self.br_map2odom = TransformBroadcaster(self)
        self.br_odom2footprint = TransformBroadcaster(self)

        # JointState publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Odometry subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize JointState message
        self.joint_msg = JointState()
        self.joint_msg.name = ['wheel_left_joint', 'wheel_right_joint']
        self.joint_msg.position = [0.0, 0.0]

        # Timer for publishing transforms and JointState
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
    def odom_callback(self, msg):
        self.initial_pos_x = msg.pose.pose.position.x
        self.initial_pos_y = msg.pose.pose.position.y    
        self.initial_pos_yaw = transforms3d.euler.quat2euler([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])[2]
        
        self.velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def timer_callback(self):
        now = self.get_clock().now()

        # --- map -> odom ---
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

        # --- odom -> base_footprint ---
        tf_odom_footprint = TransformStamped()
        tf_odom_footprint.header.stamp = now.to_msg()
        tf_odom_footprint.header.frame_id = 'odom'
        tf_odom_footprint.child_frame_id = 'base_footprint'
        tf_odom_footprint.transform.translation.x = self.initial_pos_x
        tf_odom_footprint.transform.translation.y = self.initial_pos_y
        tf_odom_footprint.transform.translation.z = 0.0
        q_odom = transforms3d.euler.euler2quat(0, 0, self.initial_pos_yaw)
        tf_odom_footprint.transform.rotation.x = q_odom[1]
        tf_odom_footprint.transform.rotation.y = q_odom[2]
        tf_odom_footprint.transform.rotation.z = q_odom[3]
        tf_odom_footprint.transform.rotation.w = q_odom[0]
        self.br_odom2footprint.sendTransform(tf_odom_footprint)

        # --- JointStates ---
        wr = (2 * self.velocity + self.angular_velocity * self.l) / (2 * self.r)
        wl = (2 * self.velocity - self.angular_velocity * self.l) / (2 * self.r)

        self.right_pos += wr * self.dt
        self.left_pos += wl * self.dt

        self.joint_msg.header.stamp = now.to_msg()
        self.joint_msg.position[0] = self.left_pos
        self.joint_msg.position[1] = self.right_pos
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
