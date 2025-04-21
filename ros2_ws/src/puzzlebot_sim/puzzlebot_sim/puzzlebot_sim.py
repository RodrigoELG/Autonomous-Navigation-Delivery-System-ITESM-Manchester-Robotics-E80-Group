import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
import numpy as np


class KinematicModelNode(Node):
    def __init__(self):
        super().__init__('puzzlebot_kinematic_model')

        # Subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_cb, 10)

        # Publishers for wheel velocities and pose
        self.pose_sim_pub = self.create_publisher(PoseStamped, '/pose_sim', 10)
        self.wr_pub = self.create_publisher(Float32, '/wr', 10)
        self.wl_pub = self.create_publisher(Float32, '/wl', 10)

        # Robot parameters
        self.wheel_radius = 0.05   # meters
        self.wheel_base = 0.19     # meters
        
        # Initial pose
        self.x = 0.0               # X position (m)
        self.y = 0.0               # Y position (m)
        self.theta = 0.0           # Orientation (rad)

        # Velocity values
        self.v = 0.0               # Linear velocity (m/s)
        self.w = 0.0               # Angular velocity (rad/s)

        # Timer for publishing
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def vel_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def timer_cb(self):
        #  Compute left and right wheel velocities using inverse kinematics
        wl = (self.v - (self.w * self.wheel_base) / 2) / self.wheel_radius
        wr = (self.v + (self.w * self.wheel_base) / 2) / self.wheel_radius

        # Publish wheel velocities
        self.wl_pub.publish(Float32(data=wl))
        self.wr_pub.publish(Float32(data=wr))
        
        # Compute the change in position
        v = self.wheel_radius * (wl + wr) / 2.0
        w = self.wheel_radius * (wr - wl) / self.wheel_base
        
        # Update the robot's position
        self.x += v * np.cos(self.theta) * self.dt
        self.y += v * np.sin(self.theta) * self.dt
        self.theta += w * self.dt

        
        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        # Set the pose
        pose_msg.pose.orientation.w = 1.0  # identity quaternion
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        # Publish the pose
        self.pose_sim_pub.publish(pose_msg)



def main(args=None):
    rclpy.init(args=args)
    node = KinematicModelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
