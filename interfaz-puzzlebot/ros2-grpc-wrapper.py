# Este script expone los datos de ROS2 como un servidor gRPC.
# Suscribe a los tópicos de coordenadas y cámara, y sirve esos datos a los clientes.
# Funciona como puente o conexion entre ROS2 (Jetson) y el sistema Flask vía gRPC o gRPC-Gateway.
# Esencial para conectar los datos en tiempo real del robot con la interfaz web.

import signal
import threading
from concurrent import futures

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CompressedImage              

import grpc
import sys
import os # Importar os para construir la ruta absoluta de forma más robusta

# --- MODIFICACIONES CRÍTICAS AQUÍ ---
# Ruta ABSOLUTA al directorio donde se generaron los archivos .pb2.py y _pb2_grpc.py
# Ahora que tu script ros2-grpc-wrapper.py está en /home/rodrigo/docker_shared/interfaz-puzzlebot/
# y los protos generados están en /home/rodrigo/docker_shared/interfaz-puzzlebot/generated_proto_python/
# esta es la forma más robusta de asegurar que Python los encuentre.
sys.path.insert(1,"./generated_proto_python") # Añadir la ruta al directorio de los protos generados

import rpc_demo_gw_pb2
import rpc_demo_gw_pb2_grpc


from google.protobuf.empty_pb2 import Empty              

# CORRECTED LINE HERE
class RPCDemoImpl(rpc_demo_gw_pb2_grpc.RPCDemoServicer):
    def __init__(self, node):
        self.node = node
        self.data = [0.0, 0.0, 0.0]
        self.image_data = b''                            
        
        # Suscripciones existentes
        node.create_subscription(Float64MultiArray,
                                 '/object_position',
                                 self.update_data,
                                 10)
        # Suscripción al tópico de imagen comprimida
        node.create_subscription(CompressedImage,
                                 '/camera_image/compressed',
                                 self.update_image,
                                 10)
        print("Initialized gRPC Server with image subscription")

    def update_data(self, msg):
        self.data[0] = msg.data[0]
        self.data[1] = msg.data[1]
        self.data[2] = msg.data[2]

    def update_image(self, msg: CompressedImage):      
        # Guardamos los bytes JPEG recibidos
        self.image_data = msg.data

    def GetMultCoords(self, request, context):
        results = rpc_demo_gw_pb2.MultCoords()
        results.values.extend(self.data)
        return results 

    def GetImage(self, request: Empty, context):
        # Devolvemos los bytes JPEG al cliente
        return rpc_demo_gw_pb2.ImageData(data=self.image_data)

terminate = threading.Event()
def terminate_server(signum, frame):
    print(f"Got signal {signum}, shutting down...")
    rclpy.shutdown()
    terminate.set()

def main(args=None):
    print("----ROS-gRPC-Wrapper----")
    signal.signal(signal.SIGINT, terminate_server)

    print("Starting ROS Node")
    rclpy.init(args=args)
    node = rclpy.create_node('object_position_wrapper')

    print("Starting gRPC Server")
    server_addr = "[::]:7042" # Escucha en todas las interfaces en el puerto 7042
    service = RPCDemoImpl(node)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    rpc_demo_gw_pb2_grpc.add_RPCDemoServicer_to_server(service, server)
    server.add_insecure_port(server_addr)
    server.start()
    print("gRPC Server listening on " + server_addr)

    # Ejecutar el spin de ROS en paralelo
    executor = futures.ThreadPoolExecutor()
    executor.submit(lambda: rclpy.spin(node))
    
    terminate.wait()
    print("Stopping gRPC Server")
    server.stop(1).wait()
    print("Exited")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()