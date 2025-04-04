import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np


class FramePublisher(Node):
    
    def __init__(self):
        super().__init__('frame_publisher')
        
        #Create Transform Messages
        self.map2odom = TransformStamped()
        self.odom2base_footprint = TransformStamped()
        self.base_footprint2base_link= TransformStamped()
        self.base_link2wheel_l = TransformStamped()
        self.base_link2wheel_r = TransformStamped()
        self.base_link2caster = TransformStamped()
        
        #Create Transform Broadcasters
        self.map_br = TransformBroadcaster(self)
        self.odom_br= TransformBroadcaster(self)
        self.base_footprint_br = TransformBroadcaster(self)
        self.base_link_br = TransformBroadcaster(self)
        self.wheel_l_br = TransformBroadcaster(self)
        self.wheel_r_br = TransformBroadcaster(self)
        self.caster_br = TransformBroadcaster(self)
        
        # Guardar tiempo de inicio
        self.start_time = self.get_clock().now()
        
        #Create a Timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.omega = 0.1
        
    #Timer Callback
    def timer_cb(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # map -> odom
        self.map2odom.header.stamp = self.get_clock().now().to_msg()
        self.map2odom.header.frame_id = 'map'
        self.map2odom.child_frame_id = 'odom'
        self.map2odom.transform.translation.x = 0.0
        self.map2odom.transform.translation.y = 0.0
        self.map2odom.transform.translation.z = 0.0
        self.map2odom.transform.rotation.x = 0.0
        self.map2odom.transform.rotation.y = 0.0
        self.map2odom.transform.rotation.z = 0.0
        self.map2odom.transform.rotation.w = 1.0
        self.map_br.sendTransform(self.map2odom)

        # odom -> base_footprint
        self.odom2base_footprint.header.stamp = self.get_clock().now().to_msg()
        self.odom2base_footprint.header.frame_id = 'odom'
        self.odom2base_footprint.child_frame_id = 'base_footprint'
        self.odom2base_footprint.transform.translation.x = 0.0
        self.odom2base_footprint.transform.translation.y = 0.0
        self.odom2base_footprint.transform.translation.z = 0.0
        self.odom2base_footprint.transform.rotation.x = 0.0
        self.odom2base_footprint.transform.rotation.y = 0.0
        self.odom2base_footprint.transform.rotation.z = 0.0
        self.odom2base_footprint.transform.rotation.w = 1.0
        self.odom_br.sendTransform(self.odom2base_footprint)
        
        # base_footprint -> base_link   
        self.base_footprint2base_link.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint2base_link.header.frame_id = 'base_footprint'
        self.base_footprint2base_link.child_frame_id = 'base_link'
        self.base_footprint2base_link.transform.translation.x = 0.0
        self.base_footprint2base_link.transform.translation.y = 0.0
        self.base_footprint2base_link.transform.translation.z = 0.05
        self.base_footprint2base_link.transform.rotation.x = 0.0
        self.base_footprint2base_link.transform.rotation.y = 0.0
        self.base_footprint2base_link.transform.rotation.z = 0.0
        self.base_footprint2base_link.transform.rotation.w = 1.0
        self.base_footprint_br.sendTransform(self.base_footprint2base_link)
        
        #base_link -> wheel_l
        self.base_link2wheel_l.header.stamp = self.get_clock().now().to_msg()
        self.base_link2wheel_l.header.frame_id = 'base_link'
        self.base_link2wheel_l.child_frame_id = 'wheel_l'
        self.base_link2wheel_l.transform.translation.x = 0.052
        self.base_link2wheel_l.transform.translation.y = -0.095
        self.base_link2wheel_l.transform.translation.z = -0.0025
        self.base_link2wheel_l.transform.rotation.x = 0.0
        self.base_link2wheel_l.transform.rotation.y = 0.0
        self.base_link2wheel_l.transform.rotation.z = 0.0
        self.base_link2wheel_l.transform.rotation.w = 1.0
        self.wheel_l_br.sendTransform(self.base_link2wheel_l)
        #base_link -> wheel_r
        self.base_link2wheel_r.header.stamp = self.get_clock().now().to_msg()
        self.base_link2wheel_r.header.frame_id = 'base_link'
        self.base_link2wheel_r.child_frame_id = 'wheel_r'
        self.base_link2wheel_r.transform.translation.x = 0.052
        self.base_link2wheel_r.transform.translation.y = 0.095
        self.base_link2wheel_r.transform.translation.z = -0.0025
        self.base_link2wheel_r.transform.rotation.x = 0.0
        self.base_link2wheel_r.transform.rotation.y = 0.0
        self.base_link2wheel_r.transform.rotation.z = 0.0
        self.base_link2wheel_r.transform.rotation.w = 1.0
        self.wheel_r_br.sendTransform(self.base_link2wheel_r)
        #base_link -> caster
        self.base_link2caster.header.stamp = self.get_clock().now().to_msg()
        self.base_link2caster.header.frame_id = 'base_link'
        self.base_link2caster.child_frame_id = 'caster'
        self.base_link2caster.transform.translation.x = -0.095
        self.base_link2caster.transform.translation.y = 0.0
        self.base_link2caster.transform.translation.z = -0.03
        self.base_link2caster.transform.rotation.x = 0.0
        self.base_link2caster.transform.rotation.y = 0.0
        self.base_link2caster.transform.rotation.z = 0.0
        self.base_link2caster.transform.rotation.w = 1.0
        self.caster_br.sendTransform(self.base_link2caster)

        
        

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