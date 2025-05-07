# integracion-de-robotica-y-sistemas-inteligentes

 **Instituto Tecnológico y de Estudios Superiores de Monterrey**  
 Campus Monterrey

![][image1]

Integración de robótica y sistemas inteligentes

**Actividad 0\. Preparación**

José Pablo Cedano Serna  			|A00832019|  
Rodrigo Escandón López Guerrero   	|A01704287|  
Luis Antonio Zermeño De Gorordo 	|A01781835|  
Luis Mario Lozoya Chairez	 		|A00833364|

Tareas:

1) Actividad de Fotogrametria 100% Completada

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
