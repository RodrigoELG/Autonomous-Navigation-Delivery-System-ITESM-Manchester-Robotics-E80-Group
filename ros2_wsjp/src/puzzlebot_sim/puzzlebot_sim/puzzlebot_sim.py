import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
import numpy as np
import tf_transformations
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class KinematicModelNode(Node):
    def __init__(self):
        super().__init__('puzzlebot_kinematic_model')

        # Future implementation: velocity commands
        # self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_cb, 10)

        qos_profile = QoSProfile(depth=10,reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Subscriptions to encoder values
        self.encoderRsub = self.create_subscription(Float32, '/VelocityEncR', self.encoderR_cb, qos_profile)
        self.encoderLsub = self.create_subscription(Float32, '/VelocityEncL', self.encoderL_cb, qos_profile)
        self.encoderR = 0.0
        self.encoderL = 0.0

        # Publishers
        self.wr_pub = self.create_publisher(Float32, '/wr', 10)
        self.wl_pub = self.create_publisher(Float32, '/wl', 10)

        # Robot physical parameters
        self.wheel_radius = 0.05   # meters
        self.wheel_base = 0.19     # meters

        # Velocities (currently updated only from encoders)
        self.v = 0.0
        self.w = 0.0

        # Timer callback
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.timer_cb)


    def encoderR_cb(self, msg):
        self.encoderR = msg.data

    def encoderL_cb(self, msg):
        self.encoderL = msg.data

    # Future implementation: velocity from /cmd_vel
    # def vel_cb(self, msg):
    #     self.v = msg.linear.x
    #     self.w = msg.angular.z

    def timer_cb(self):

        # Estimate velocities from encoders
        self.v = self.wheel_radius * (self.encoderR + self.encoderL) / 2.0
        self.w = self.wheel_radius * (self.encoderR - self.encoderL) / self.wheel_base

        # Compute individual wheel velocities using inverse kinematics
        wl = (self.v - (self.w * self.wheel_base) / 2.0) / self.wheel_radius
        wr = (self.v + (self.w * self.wheel_base) / 2.0) / self.wheel_radius

        # Log individual wheel velocities

        # Publish wheel velocities
        self.wl_pub.publish(Float32(data=wl))
        self.wr_pub.publish(Float32(data=wr))


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

