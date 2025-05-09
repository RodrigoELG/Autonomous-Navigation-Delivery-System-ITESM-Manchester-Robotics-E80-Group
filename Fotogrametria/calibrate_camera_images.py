import cv2
import numpy as np
import glob
import os

def calibrate_camera_from_images(chessboard_size, square_size, images_folder):
    """
    Realiza la calibración de la cámara dados un conjunto de imágenes con un tablero de ajedrez.
    
    Parameters:
    - chessboard_size: (cols, rows) número de esquinas internas del tablero 
      (ejemplo: (7, 6) si el tablero tiene 7 esquinas horizontales internas y 6 verticales).
      en nuestro caso es de 9x6
    """

    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp = objp * square_size

    objpoints = []  # puntos 3D en el mundo real
    imgpoints = []  # puntos 2D en la imagen

    images = glob.glob(images_folder)
    if not images:
        print(f"No se encontraron imágenes con el patrón: {images_folder}")
        return None, None, None, None, None

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"No se pudo leer la imagen: {fname}")
            continue
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            corners_sub = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            objpoints.append(objp)
            imgpoints.append(corners_sub)

            cv2.drawChessboardCorners(img, chessboard_size, corners_sub, ret)
            cv2.imshow('Tablero detectado', img)
            cv2.waitKey(500) 
        else:
            print(f"Tablero NO detectado en {fname}")

    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    return ret, mtx, dist, rvecs, tvecs

def main():
    chessboard_size = (9, 6)    
    square_size = 0.024         

    images_folder = os.path.join('calibration_imgs', '*.jpg')  # Patron tipo 'calibration_imgs/*.jpg'

    ret, mtx, dist, rvecs, tvecs = calibrate_camera_from_images(
        chessboard_size, square_size, images_folder
    )

    if mtx is not None:
        print(f"RMS Error: {ret}")
        print(f"Matriz de Calibración (mtx):\n{mtx}")
        print(f"Coeficientes de Distorsión (dist): {dist.ravel()}")

        # Guardamos en un archivo npz
        np.savez('calibration_data.npz', camera_matrix=mtx, dist_coeffs=dist)
        print("Parámetros de calibración guardados en 'calibration_data.npz'")

if __name__ == '__main__':
    main()