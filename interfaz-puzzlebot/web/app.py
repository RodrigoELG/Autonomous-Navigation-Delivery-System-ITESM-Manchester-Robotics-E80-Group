# Esta app Flask actúa como interfaz web para visualizar los datos del PuzziBot.
# Consulta el servidor gRPC-Gateway para obtener coordenadas e imágenes del robot.
# Expone endpoints REST que son consumidos por el frontend HTML.
# Permite acceder al video en vivo y a la posición del objeto desde un navegador.

from flask import Flask, render_template, Response, jsonify # Añadido jsonify
import requests
import time
import cv2
import numpy as np
from flask import stream_with_context
import base64 # Añadido para decodificar base64
import json # Añadido para manejar errores de JSON

app = Flask(__name__)

# --- MODIFICACIONES CLAVE AQUÍ ---
# URLs del gRPC-gateway
# La IP '127.0.0.1' es para localhost. Si tu gateway corre en una máquina diferente (ej. la Jetson),
# DEBES CAMBIAR ESTA IP a la IP de la máquina donde corre el go-gateway.
GATEWAY_BASE_URL = 'http://127.0.0.1:8042/restgatewaydemo' # Base URL para los endpoints del gateway
GPRC_COORDS_URL = f'{GATEWAY_BASE_URL}/getmultcoords' # Endpoint para coordenadas
GPRC_IMAGE_URL = f'{GATEWAY_BASE_URL}/getimage'       # Endpoint para imagen
# --- FIN DE MODIFICACIONES CLAVE ---


@app.route('/')
def interfaz():
    return render_template('interfaz.html')

@app.route('/api/result')
def get_result():
    try:
        resp = requests.post(GPRC_COORDS_URL, json={}) # Post para GetMultCoords
        resp.raise_for_status() # Lanza una excepción para errores HTTP (4xx o 5xx)
        data = resp.json().get('values', [])
        if len(data) >= 2:
            return f"{data[0]:.2f},{data[1]:.2f}" # Formateo a 2 decimales para claridad
        return "0.00,0.00" # Asegura un formato consistente incluso si no hay datos
    except requests.exceptions.RequestException as e:
        app.logger.error(f"Error al obtener coordenadas del gateway: {e}")
        return "Error,Error" # Mensaje de error para el frontend
    except json.JSONDecodeError as e:
        app.logger.error(f"Error al decodificar JSON de coordenadas: {e}")
        return "Error,Error"

def gen_mjpeg():
    """Generador de frames MJPEG llamando al RPC de imagen."""
    while True:
        try:
            resp = requests.get(GPRC_IMAGE_URL)
            resp.raise_for_status() # Lanza una excepción para errores HTTP (4xx o 5xx)

            # El gRPC-Gateway convierte el mensaje ImageData { bytes data = 1; }
            # en un JSON {"data": "base64_encoded_bytes"}
            image_json = resp.json()
            b64_img_data = image_json.get('data') # Obtener el campo 'data'

            if b64_img_data:
                # Decodificar de base64 a bytes binarios
                frame_bytes = base64.b64decode(b64_img_data)
                
                # Decodificar los bytes JPEG a una imagen OpenCV
                np_arr = np.frombuffer(frame_bytes, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if cv_image is not None:
                    # Codificar la imagen OpenCV a JPEG para el stream MJPEG
                    _, jpeg_frame = cv2.imencode('.jpg', cv_image)
                    if _: # Si la codificación fue exitosa
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' +
                               jpeg_frame.tobytes() + b'\r\n')
                    else:
                        app.logger.warning("Error al codificar imagen a JPEG.")
                else:
                    app.logger.warning("Error al decodificar la imagen JPEG recibida.")
            else:
                app.logger.warning("No se recibió campo 'data' (imagen base64) del gateway.")

        except requests.exceptions.RequestException as e:
            app.logger.error(f"Error de conexión al gRPC-Gateway (imagen): {e}")
        except json.JSONDecodeError as e:
            app.logger.error(f"Error al decodificar JSON de imagen: {e}. Respuesta: {resp.text[:100] if 'resp' in locals() else 'N/A'}")
        except Exception as e:
            app.logger.error(f"Error general en gen_mjpeg: {e}")
        
        time.sleep(0.01) # Pequeño delay para evitar saturar si hay errores o no hay frames

@app.route('/api/stream')
def mjpeg_stream():
    """Stream MJPEG continuo para el navegador."""
    return Response(
        stream_with_context(gen_mjpeg()),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == '__main__':
    # Ejecutar la app Flask. Asegúrate de que el puerto 8002 esté disponible.
    # 'host='0.0.0.0'' permite que la app sea accesible desde otras IPs en la red.
    app.run(debug=True, host='0.0.0.0', port=8002)