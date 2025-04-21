import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import transforms3d
import signal

class DeadReckoning(Node):
    def __init__(self):
        super().__init__('localisation_node')
        
        # Subscribers for wheel velocities
        self.wr_sub = self.create_subscription(Float32, '/wr', self.wr_cb, 10)
        self.wl_sub = self.create_subscription(Float32, '/wl', self.wl_cb, 10)
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Robot parameters
        self.wheel_radius = 0.05   # meters
        self.wheel_base = 0.19     # meters
        
        # Initial pose
        self.wl = 0.0
        self.wr = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Time step
        self.dt = 0.05
        
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def wr_cb(self, msg):
        self.wr = msg.data

    def wl_cb(self, msg):
        self.wl = msg.data
        
    def timer_callback(self):
        # Compute the change in position
        v = self.wheel_radius * (self.wl + self.wr) / 2.0
        w = self.wheel_radius * (self.wr - self.wl) / self.wheel_base
        
        # Update the robot's position
        self.x += v * np.cos(self.theta) * self.dt
        self.y += v * np.sin(self.theta) * self.dt
        self.theta += w * self.dt
        
        # Create a Odometry message
        Odometry_msg = Odometry()
        Odometry_msg.header.stamp = self.get_clock().now().to_msg()
        Odometry_msg.header.frame_id = 'odom' # Reference frame
        Odometry_msg.child_frame_id = 'base_footprint' # Robot frame
        
        # Set the position and orientation
        Odometry_msg.pose.pose.position.x = self.x
        Odometry_msg.pose.pose.position.y = self.y
        Odometry_msg.pose.pose.position.z = 0.0
        
        # Convert yaw (theta) to quaternion
        q = transforms3d.euler.euler2quat(0, 0, self.theta)
        Odometry_msg.pose.pose.orientation.x = q[1]
        Odometry_msg.pose.pose.orientation.y = q[2]
        Odometry_msg.pose.pose.orientation.z = q[3]
        Odometry_msg.pose.pose.orientation.w = q[0]
        
        # Set the linear and angular velocities
        Odometry_msg.twist.twist.linear.x = v
        Odometry_msg.twist.twist.angular.z = w
        
        # Publish the odometry message
        self.odom_pub.publish(Odometry_msg)
        
    
def main(args=None):

    rclpy.init(args=args)

    node = DeadReckoning()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): 
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()