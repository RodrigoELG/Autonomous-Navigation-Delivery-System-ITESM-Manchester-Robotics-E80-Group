import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np


class FramePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        # Tipo de mensaje para las transformaciones
        self.map2odom = TransformStamped()
        self.odom2base_footprint = TransformStamped()
        self.base_footprint2base_link = TransformStamped()
        self.base_link2wheel_l = TransformStamped()
        self.base_link2wheel_r = TransformStamped()
        self.base_link2caster = TransformStamped()

        # Broadcasters
        self.tf_dynamic = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        # Tiempo inicial
        self.start_time = self.get_clock().now()
        self.omega = 0.5  # Velocidad angular de ruedas

        # Transformaciones estáticas
        self.define_static_transforms()

        # Timer para transformaciones dinámicas
        self.timer = self.create_timer(0.1, self.timer_cb)

    def define_static_transforms(self):
        # map -> odom
        self.map2odom.header.stamp = self.get_clock().now().to_msg()
        self.map2odom.header.frame_id = 'map'
        self.map2odom.child_frame_id = 'odom'
        self.map2odom.transform.translation.x = -2.0
        self.map2odom.transform.translation.y = -2.0
        self.map2odom.transform.translation.z = 0.0
        self.map2odom.transform.rotation.w = 1.0

        # base_footprint -> base_link
        self.base_footprint2base_link.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint2base_link.header.frame_id = 'base_footprint'
        self.base_footprint2base_link.child_frame_id = 'base_link'
        self.base_footprint2base_link.transform.translation.z = 0.05
        self.base_footprint2base_link.transform.rotation.w = 1.0

        # base_link -> caster_link
        self.base_link2caster.header.stamp = self.get_clock().now().to_msg()
        self.base_link2caster.header.frame_id = 'base_link'
        self.base_link2caster.child_frame_id = 'caster_link'
        self.base_link2caster.transform.translation.x = -0.095
        self.base_link2caster.transform.translation.z = -0.03
        self.base_link2caster.transform.rotation.w = 1.0

        # Enviar los static tf
        self.static_br.sendTransform([
            self.map2odom,
            self.base_footprint2base_link,
            self.base_link2caster
        ])

    def timer_cb(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # odom -> base_footprint
        self.odom2base_footprint.header.stamp = self.get_clock().now().to_msg()
        
        self.odom2base_footprint.header.frame_id = 'odom'
        self.odom2base_footprint.child_frame_id = 'base_footprint'
        
        self.odom2base_footprint.transform.translation.x = -1.0
        self.odom2base_footprint.transform.translation.y = -1.0
        self.odom2base_footprint.transform.translation.z = 0.0
        self.odom2base_footprint.transform.rotation.w = 1.0
        
        self.tf_dynamic.sendTransform(self.odom2base_footprint)

        # base_link -> wheel_l_link
        self.base_link2wheel_l.header.stamp = self.get_clock().now().to_msg()
        
        self.base_link2wheel_l.header.frame_id = 'base_link'
        self.base_link2wheel_l.child_frame_id = 'wheel_l_link'
        
        self.base_link2wheel_l.transform.translation.x = 0.052
        self.base_link2wheel_l.transform.translation.y = -0.095
        self.base_link2wheel_l.transform.translation.z = -0.0025
        
        q = transforms3d.euler.euler2quat(0, elapsed_time * self.omega, 0)
        self.base_link2wheel_l.transform.rotation.x = q[1]
        self.base_link2wheel_l.transform.rotation.y = q[2]
        self.base_link2wheel_l.transform.rotation.z = q[3]
        self.base_link2wheel_l.transform.rotation.w = q[0]
        
        self.tf_dynamic.sendTransform(self.base_link2wheel_l)

        # base_link -> wheel_r_link
        self.base_link2wheel_r.header.stamp = self.get_clock().now().to_msg()
        
        self.base_link2wheel_r.header.frame_id = 'base_link'
        self.base_link2wheel_r.child_frame_id = 'wheel_r_link'
        
        self.base_link2wheel_r.transform.translation.x = 0.052
        self.base_link2wheel_r.transform.translation.y = 0.095
        self.base_link2wheel_r.transform.translation.z = -0.0025
        
        q2 = transforms3d.euler.euler2quat(0, elapsed_time * self.omega, 0)
        self.base_link2wheel_r.transform.rotation.x = q2[1]
        self.base_link2wheel_r.transform.rotation.y = q2[2]
        self.base_link2wheel_r.transform.rotation.z = q2[3]
        self.base_link2wheel_r.transform.rotation.w = q2[0]
        
        self.tf_dynamic.sendTransform(self.base_link2wheel_r)


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
