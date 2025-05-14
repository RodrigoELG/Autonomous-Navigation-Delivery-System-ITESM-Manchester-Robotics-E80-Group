import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import transforms3d

class PointStabilisationNode(Node):
    def __init__(self):
        super().__init__('control')

        # Subscribe to the robot's current odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscribe to the target goal position
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # Publisher for velocity commands (cmd_vel)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control gains
        self.kp_v = 0.8   # Proportional gain for linear velocity
        self.kp_w = 2.5   # Proportional gain for angular velocity

        # Timer interval
        self.dt = 0.05  # 20 Hz
        
        # Target goal (x, y)
        self.setpoint = np.array([0.0, 0.0])

        # Flag to indicate if the goal has been reached        
        self.goal_reached = False

        # Current robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Control tolerances
        self.dist_tolerance = 0.05  # Minimum distance to goal (meters)
        self.angle_tolerance = 0.1  # Minimum angle alignment (radians)
        
        # Start control loop timer
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
    def goal_cb(self, msg):
        # Callback to update goal position
        self.setpoint[0] = msg.pose.position.x
        self.setpoint[1] = msg.pose.position.y
        self.goal_reached = False  # Reset goal reached flag
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Convert quaternion to yaw (theta)
        self.theta = transforms3d.euler.quat2euler([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])[2]

    
    def timer_callback(self):
        # Compute distance and angle to goal
        error_x = self.setpoint[0] - self.x
        error_y = self.setpoint[1] - self.y
        error = np.hypot(error_x, error_y)

        angle_to_goal = np.arctan2(error_y, error_x)
        angle_error = angle_to_goal - self.theta
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))  # normalize

        cmd_vel_msg = Twist()

        if error < self.dist_tolerance:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            if not self.goal_reached:
                self.get_logger().info("Goal reached")
                self.goal_reached = True
        else:
            self.goal_reached = False

            # Rotation prioritization: only rotate in place if not well-aligned
            if abs(angle_error) > self.angle_tolerance:
                cmd_vel_msg.linear.x = 0.0
            else:
                cmd_vel_msg.linear.x = np.clip(self.kp_v * error, -0.07, 0.07)  # Limit linear speed

            cmd_vel_msg.angular.z = np.clip(self.kp_w * angle_error, -0.07, 0.07) # Limit angular speed

        # DEBUG LOGGING
        #self.get_logger().debug(f"Pos: ({self.x:.2f}, {self.y:.2f}), Goal: ({self.setpoint[0]:.2f}, {self.setpoint[1]:.2f}), Error: {error:.2f}, Angle error: {angle_error:.2f}")

        self.cmd_vel_pub.publish(cmd_vel_msg)




def main(args=None):
    rclpy.init(args=args)

    node = PointStabilisationNode()

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
