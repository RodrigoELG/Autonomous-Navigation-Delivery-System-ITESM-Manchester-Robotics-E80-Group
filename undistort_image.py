import cv2
import numpy as np
import glob
import os

def main():
    data = np.load('calibration_data.npz')
    mtx = data['camera_matrix']
    dist = data['dist_coeffs']

    images_path = os.path.join('paisaje1_imgs', '*.jpg')
    images = glob.glob(images_path)

    if not images:
        print(f"No hay imágenes en tu carpeta {images_path}")
        return

    for img_file in images:
        img = cv2.imread(img_file)
        h, w = img.shape[:2]

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        undistorted = cv2.undistort(img, mtx, dist, None, newcameramtx)

        x, y, w_roi, h_roi = roi
        undistorted = undistorted[y:y+h_roi, x:x+w_roi]

        # Guardar la o las imagenes undistort en ...
        base_name = os.path.basename(img_file)
        out_dir = '/home/rodrigo/Documents/Fotogrametria/undistorted_imgs'
        out_name = os.path.join(out_dir, f"undistorted_{base_name}")
        cv2.imwrite(out_name, undistorted)
        print(f"Guardado: {out_name}")                                                                                                                                                                                                                                                                                                                                                                                                                    

if __name__ == '__main__':
    main()

