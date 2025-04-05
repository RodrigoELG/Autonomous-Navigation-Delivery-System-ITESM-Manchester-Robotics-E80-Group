import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np


def create_transform(clock, parent, child, translation, rotation=(0.0, 0.0, 0.0)):
    tf = TransformStamped()
    tf.header.stamp = clock.now().to_msg()
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x = translation[0]
    tf.transform.translation.y = translation[1]
    tf.transform.translation.z = translation[2]

    q = transforms3d.euler.euler2quat(*rotation)  # w, x, y, z
    tf.transform.rotation.x = q[1]
    tf.transform.rotation.y = q[2]
    tf.transform.rotation.z = q[3]
    tf.transform.rotation.w = q[0]

    return tf


class FramePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        # Broadcasters
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Tiempo inicial
        self.start_time = self.get_clock().now()

        # Transformaciones estáticas
        self.static_transforms = self.define_static_transforms()
        self.static_broadcaster.sendTransform(self.static_transforms)

        # Timer para transformaciones dinámicas
        self.timer = self.create_timer(0.1, self.timer_cb)

    def define_static_transforms(self):
        clock = self.get_clock()
        return [
            create_transform(clock, 'map', 'odom', (-2.0, -2.0, 0.0)),
            create_transform(clock, 'base_footprint', 'base_link', (0.0, 0.0, 0.05)),
            create_transform(clock, 'base_link', 'caster_link', (-0.095, 0.0, -0.03))
        ]

    def timer_cb(self):
        now = self.get_clock().now().to_msg()

        # odom -> base_footprint (posición dinámica)
        tf_odom_to_base = TransformStamped()
        tf_odom_to_base.header.stamp = now
        tf_odom_to_base.header.frame_id = 'odom'
        tf_odom_to_base.child_frame_id = 'base_footprint'
        tf_odom_to_base.transform.translation.x = -1.0
        tf_odom_to_base.transform.translation.y = -1.0
        tf_odom_to_base.transform.translation.z = 0.0

        q = transforms3d.euler.euler2quat(0.0, 0.0, 0.0)
        tf_odom_to_base.transform.rotation.x = q[1]
        tf_odom_to_base.transform.rotation.y = q[2]
        tf_odom_to_base.transform.rotation.z = q[3]
        tf_odom_to_base.transform.rotation.w = q[0]

        # base_link -> rueda izquierda
        tf_wheel_l = create_transform(
            self.get_clock(),
            'base_link',
            'wheel_l_link',
            (0.052, -0.095, -0.0025)
        )
        tf_wheel_l.header.stamp = now

        # base_link -> rueda derecha
        tf_wheel_r = create_transform(
            self.get_clock(),
            'base_link',
            'wheel_r_link',
            (0.052, 0.095, -0.0025)
        )
        tf_wheel_r.header.stamp = now

        self.dynamic_broadcaster.sendTransform([
            tf_odom_to_base,
            tf_wheel_l,
            tf_wheel_r
        ])


def main(args=None):
    rclpy.init(args=args)
    node = FramePublisher()
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
