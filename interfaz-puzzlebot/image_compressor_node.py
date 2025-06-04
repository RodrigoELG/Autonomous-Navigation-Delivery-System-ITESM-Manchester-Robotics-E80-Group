import os
import numpy as np
import cv2
from time import time
import ctypes

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

class ImageCompressorNode(Node):
    def __init__(self):
        super().__init__('image_compressor')

        # Publisher para las imágenes comprimidas
        self.compressed_image_publisher = self.create_publisher(
            CompressedImage, '/camera_image/compressed', 10)

        # Suscriptor a los frames de la cámara
        self.raw_image_subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.raw_image_callback,
            10)
        self.raw_image_subscription  # prevent unused variable warning

        self.bridge = CvBridge()
        self.target_width = 200
        self.target_height = 100

        self.get_logger().info('Nodo de compresión de imágenes iniciado')

    def raw_image_callback(self, msg):
        try:
            # Convertir el mensaje ROS Image a una imagen de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error al convertir la imagen: {e}')
            return

        # Redimensionar la imagen
        resized_image = cv2.resize(cv_image, (self.target_width, self.target_height))

        # Comprimir la imagen a formato JPEG
        success, compressed_image = cv2.imencode('.jpg', resized_image)

        if success:
            # Crear el mensaje CompressedImage
            compressed_msg = CompressedImage()
            compressed_msg.format = 'jpeg'
            compressed_msg.data = compressed_image.tobytes()

            # Publicar el mensaje comprimido
            self.compressed_image_publisher.publish(compressed_msg)
            # self.get_logger().info('Imagen sin comprimir recibida y publicada comprimida')
        else:
            self.get_logger().error('Error al comprimir la imagen')

def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()