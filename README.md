
# 🧩 Puzzlebot Jetson Edition – Guía de Conexión y Ejecución

**Instituto Tecnológico y de Estudios Superiores de Monterrey**  
**Campus Monterrey**  
**Curso: Integración de Robótica y Sistemas Inteligentes**

---

## 🖥️ 1. Configuración del Entorno de Desarrollo (Local o Docker)

Esta sección es para configurar tu computadora local o un entorno en contenedor (Docker) para trabajar con ROS2 y el ecosistema del Puzzlebot.

---

### 🐳 Opción A: Docker con ROS2 (Recomendado para entornos reproducibles)

#### Paso 1: Requisitos previos

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y nano tmux terminator mesa-utils htop curl gnupg lsb-release
```

#### Paso 2: Instalar Docker Engine

```bash
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin docker-buildx-plugin
```

#### Paso 3: Verificar Docker

```bash
sudo docker run hello-world
```

---

### 🧪 Git y Visual Studio Code

```bash
sudo apt install -y git
git config --global user.name "Tu Nombre"
git config --global user.email "tuemail@dominio.com"

sudo snap install code --classic
```

Extensiones recomendadas:
- Docker
- Remote - Containers
- GitLens
- Python
- ROS

---

### 🐢 ROS2 en Docker

```bash
sudo docker pull ros:humble-perception

sudo docker run --network="host" \
-v /dev/shm:/dev/shm \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ~/robotica:/shared-folder \
-it --privileged --name ros2-humble-robotics ros:humble-perception
```

Configuraciones opcionales dentro del contenedor:
```bash
apt-get update && apt-get install -y sudo nano tmux terminator mesa-utils htop
adduser robotics
usermod -aG sudo robotics
echo 'export DISPLAY=:0' >> /home/robotics/.bashrc
```

---

### ⚙️ Integración con GPU (CUDA)

1. Instala drivers y CUDA desde [https://developer.nvidia.com/cuda-downloads](https://developer.nvidia.com/cuda-downloads)
2. Instala `nvidia-container-toolkit`:
```bash
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```
3. Ejecuta contenedor con acceso a GPU:

```bash
xhost +local:root

sudo docker run --gpus all --runtime=nvidia --network="host" \
-v /dev/shm:/dev/shm \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ~/robotica:/shared-folder \
-it --privileged \
--name ros2-humble-cuda ros:humble-perception
```

---

### ✅ Validación del entorno

```bash
docker --version
docker info
git --version
git config --list
code .
nvidia-smi
ros2 --version
```

---

### 🧰 Comandos útiles de Docker

| Acción                      | Comando                                     |
| --------------------------- | ------------------------------------------- |
| Ver contenedores activos    | `docker ps`                                 |
| Entrar a un contenedor      | `sudo docker attach <nombre>`               |
| Salir sin cerrar contenedor | `Ctrl+P` + `Ctrl+Q`                         |
| Guardar cambios             | `sudo docker commit <nombre> <usuario/img>` |
| Eliminar contenedor         | `sudo docker rm <nombre>`                   |
| Ver imágenes locales        | `sudo docker images`                        |
| Borrar imagen               | `sudo docker rmi <usuario/imagen>`          |
## 🔌 2. Conexión de Hardware

1. **Conectar Hackerboard (ESP32) al Jetson Nano**  
   - Usa un cable **USB a micro-USB**.  
   - Alimenta ambos con **PowerBanks independientes** (5V, 3A mínimo por puerto).

2. **Sensores y Periféricos al Jetson Nano**
   - Conecta el **LiDAR SLLIDAR A1** a un puerto USB.
   - Conecta el **módulo Wi-Fi**.

---

## 📶 3. Conexión de Red

1. Enciende el Puzzlebot.  
   El Hackerboard emitirá una red Wi-Fi (el **SSID aparece en la pantalla OLED**).
2. Desde tu PC:
   - Conéctate a esa red Wi-Fi.
   - Asegúrate de que el **Jetson Nano también esté en esa red**.

3. Accede a la **interfaz web del Hackerboard** (opcional):  
   ```
   http://192.168.1.1
   ```

---

## 🖥️ 4. Acceso Remoto al Jetson Nano

```bash
ssh puzzlebot@192.168.1.#
```
- Reemplaza `#` con el número de IP (usualmente entre 2 y 10).
- Contraseña por defecto: `Puzzlebot72`

---

## 🧠 5. Iniciar micro-ROS Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB#
```

> Verifica el puerto con:
```bash
ls -l /dev/ttyUSB*
```

---

## 🚀 6. Ejecutar el LiDAR

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

### Visualización con RViz2:

```bash
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
```

> ❗ Si aparece un error **TIMEOUT**, desconecta y vuelve a conectar el LiDAR.

---

## 🎮 7. Teleoperar Puzzlebot

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🗺️ 8. Crear y Guardar Mapa SLAM

Este launch corre el puzzlebot con `slam_toolbox`:

```bash
ros2 launch puzzlebot_sim puzzlebot_mapping_launch.py
```

### Visualización con RViz2 (solo si estás en Jetson con interfaz gráfica):

```bash
ros2 launch puzzlebot_sim view_puzzlebot_mapping_launch.py
```

---

## 📷 9. Activar Cámara del Puzzlebot (CSI)

Dentro del Jetson Nano:

```bash
ros2 launch ros_deep_learning video_source.ros2.launch input_width:=1920 input_height:=1080 input_framerate:=15
```

---


