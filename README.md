Autonomous Navigation & Delivery System – ITESM + Manchester Robotics + E80 Group
Feb 2025 – Jul 2025 | ROS2, OpenCV, YOLOv8, Jetson Nano, LiDAR, gRPC

• Objective: Build a full-stack delivery robot for trailer docking using SLAM, path planning, obstacle avoidance, and computer vision.

• Solution:
→ Designed and implemented an A* path planner and Tangent Bug algorithm in ROS2 with obstacle inflation and safety heuristics.
→ Developed a vision-based alignment system using ORB features, homography, and real-time tracking to center the robot on trailer targets.
→ Integrated ROS2 with a web dashboard using gRPC for camera streaming and delivery status.
→ Tuned PD control loops and inflated occupancy maps for safe motion execution.

• Impact: Achieved autonomous package delivery to target trailers with high alignment precision. System successfully handled real-world uncertainty (noise, lighting, geometry) with robust CV + planning integration.

Integración de robótica y sistemas inteligentes

**Actividad 0\. Preparación**

José Pablo Cedano Serna  			|A00832019|  
Rodrigo Escandón López Guerrero   	|A01704287|  
Luis Antonio Zermeño De Gorordo 	|A01781835|  
Luis Mario Lozoya Chairez	 		|A00833364|

Tareas:

1) Actividad de Fotogrametria 100% Completada
2) Bug Algorithms 
Referencias utiles:

https://www.youtube.com/watch?v=FUjHgDvn6s0

https://utkuolcar.com/Blog/2021/1/10/bug-algorithm-application-using-ros-robotik-operating-system

https://github.com/ManchesterRoboticsLtd/TE3003B_Integration_of_Robotics_and_Intelligent_Systems_2025/blob/main/Week6/Presentations/PDF/MCR2_Reactive_Navigation.pdf

https://www.youtube.com/watch?v=cnOBXyoFPVg

Comandos basicos:

Inicializar mi contenedor y entrar con el usuario de robotics. 
sudo docker start -ai ros2-humble-robotics-cuda
su - robotics 

Dirección de mi ws: cd ~/ros2_ws 

Comandos para inicializar y correr mi imagen de docker de interfaz:

 Inicializar docker 
1) docker start te3003b-2025-work
 Entrar con usuario robotics
2) su - robotics
3) cd /docker-share
4) conda activate te300xb
   
Correr los primero dos codigos dentro de la carpeta 1 

/linux-pt1/ROS-Obj..

5) python3 ros2-object-detection.py

/linux-pt1/PY-RPC..

6) python3 ros2-grpc-wrapper.py
   
 Correr los primero dos codigos dentro de la carpeta 2
 
 /linux-pt2(1)/CS-RPC../cs-rpc-demo/bin/Debug/net4.5
 
7) mono --trace=M:System.Diagnostics.Debug:WriteLine cs-rpc-demo.exe
   
 /linux-pt2(1)/GO-REST..

8) ./go-gateway
   
 /linux-pt2(1)/FLASK...

9) python3 app.py
