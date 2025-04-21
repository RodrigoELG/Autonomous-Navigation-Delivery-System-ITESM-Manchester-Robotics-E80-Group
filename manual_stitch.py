import cv2
import numpy as np
import os

""" PROBLEMAS ENFRENTADOS 
1) CRASHEO DE LA COMPU:
    Hipotesis
    Tuve que gregar una funcion con un constante para redimensionar las imagenes dado que 
    la carga computacional era muy grande al correr con GPU provocando que mi compu crasheara.
    El codigo estaba produciendo las imagenes intermedias con muchos features al hacer el stitching con 
    imagenes que son de alta resolucion.Consumiendo mucha RAM y CPU/GPU. 
    Otro posible problema es 
    que las imagenes varien demasiado en cuanto a perspectiva haciendo que el warp sobrecompensara 
    con una imagen de mayor tamaño y al ser un proceso iterativo la imagen es tan grande que nos
    quedamos sin recursos. 

2) AREAS NEGRAS EN LA IMAGEN FINAL:
    Hipotesis
    Creo que la homografia es un tanto extrema. Al tener una variacion de perspectiva, angulos muy grandes on gran profundidad entre de cada imagen
     la homogria no es capaz de compensar y por ende proyecta areas negras.
     
3) STITCHING ITERATIVO CON ERROR:
    Hipotesis:
    Creo que el problema puede estar en que al coser en cadena, la imagen resultante de (1-2) se warpea y adquiere una perspectiva distinta de la foto 2 original. 
    Así, cuando se compara con la foto 3, hay más variación de características de la esperada si se usaran solo fotos originales. Aunque cada 
    par consecutivo (1-2, 2-, 3-4, etc..) parezca similar, la foto resultante intermedia distorsiona la siguiente coincidencia. Esto nos da como resultado
    errores en la homografía y genera la zonas negras o el stitching raro que vemo en las imagenes. 
    Posible solucion:
    * Utilizar algo para cada par de imágenes consecutivas n, n+1 
    * Usar una imagen de referencia
"""



MAX_WIDTH = 1000

def resize_image(image, max_width=MAX_WIDTH):
    h, w = image.shape[:2]
    if w > max_width:
        ratio = max_width / w
        new_dim = (max_width, int(h * ratio))
        return cv2.resize(image, new_dim, interpolation=cv2.INTER_AREA)
    return image

def crop_black_areas(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    if cv2.countNonZero(mask) == 0:
        return image
    x, y, w, h = cv2.boundingRect(mask)
    return image[y:y+h, x:x+w]


def stitch_two_images(img1, img2, debug=False):
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # Crear ORB y detectar/ nfeatires se puede ajustar intente un numero bajo por el problema 1) planteado 
    orb = cv2.ORB_create(nfeatures=400)
    kp1, des1 = orb.detectAndCompute(gray1, None)
    kp2, des2 = orb.detectAndCompute(gray2, None)

    if debug:
        img1_feat = cv2.drawKeypoints(img1, kp1, None, color=(0,255,0))
        img2_feat = cv2.drawKeypoints(img2, kp2, None, color=(0,255,0))
        cv2.imshow("Features en Imagen 1", img1_feat)
        cv2.imshow("Features en Imagen 2", img2_feat)
        print("[DEBUG] Mostrando features detectados. Presiona 'esc' para continuar...")
        while True:
            key = cv2.waitKey(0) & 0xFF
            if key == 27:
                break
        cv2.destroyAllWindows()

    # Matching
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)
    good_matches = matches[:40]

    if debug:
        match_vis = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, None,
                                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow("Matching de Caracteristicas (ORB)", match_vis)
        print("[DEBUG] Mostrando matches ORB. Presiona 'esc' para continuar...")
        while True:
            key = cv2.waitKey(0) & 0xFF
            if key == 27:
                break        
        cv2.destroyAllWindows()

    # Extraer puntos (features) 
    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1,1,2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1,1,2)

    #  Calcular homografía
    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 4.0)
    if H is None:
        print("Error: Homografía no calculada.")
        return img1

    # Obtener dimensiones
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]

    # Calcular bounding box
    corners_img1 = np.float32([[0,0],[0,h1],[w1,h1],[w1,0]]).reshape(-1,1,2)
    transformed_corners = cv2.perspectiveTransform(corners_img1, H).reshape(-1,2)
    fixed_corners = np.float32([[0,0],[0,h2],[w2,h2],[w2,0]])
    corners_combined = np.vstack((transformed_corners, fixed_corners))
    [xmin, ymin] = np.int32(corners_combined.min(axis=0).ravel() - 0.5)
    [xmax, ymax] = np.int32(corners_combined.max(axis=0).ravel() + 0.5)

    print("[DEBUG] xmin, ymin, xmax, ymax:", xmin, ymin, xmax, ymax)

    translation = [-xmin, -ymin]
    H_translation = np.array([[1, 0, translation[0]],
                              [0, 1, translation[1]],
                              [0, 0, 1]], dtype=np.float32)

    # Warp de img1 sin recorte
    result = cv2.warpPerspective(img1, H_translation.dot(H), (xmax - xmin, ymax - ymin))

    # Pega img2 en el lienzo. Creo que hay algo mejorable en el proceso del warp para minimizar el error, pero ya no di con que. 
    y_start = translation[1]
    x_start = translation[0]
    y_end = y_start + h2
    x_end = x_start + w2
    result[y_start:y_end, x_start:x_end] = img2

    if debug:
        cv2.imshow("Panorama Parcial", result)
        print("[DEBUG] Mostrando resultado parcial de stitching. Presiona 'esc' para continuar ...")
        while True:
            key = cv2.waitKey(0) & 0xFF
            if key == 27:
                break
        cv2.destroyAllWindows()

    return result

def main():
    images_list = [
        '/home/rodrigo/Documents/Fotogrametria/undistorted_imgs/undistorted_paisaje1.jpg',
        '/home/rodrigo/Documents/Fotogrametria/undistorted_imgs/undistorted_paisaje2.jpg',
        '/home/rodrigo/Documents/Fotogrametria/undistorted_imgs/undistorted_paisaje3.jpg',
        '/home/rodrigo/Documents/Fotogrametria/undistorted_imgs/undistorted_paisaje4.jpg',
    ]

    base_image = cv2.imread(images_list[0])
    base_image = resize_image(base_image)
    if base_image is None:
        print(f"Error leyendo {images_list[0]}")
        return

    for i in range(1, len(images_list)):
        next_image = cv2.imread(images_list[i])
        next_image = resize_image(next_image)
        if next_image is None:
            print(f"Error leyendo {images_list[i]}")
            continue
        
        print(f"\n--- Stitching {images_list[i-1]} con {images_list[i]} ---")

        base_image = stitch_two_images(base_image, next_image, debug=True)
        base_image = crop_black_areas(base_image)
        base_image = resize_image(base_image)

    cv2.imwrite("panorama_final.jpg", base_image)
    print("\n[INFO] Panorama final guardado en 'panorama_final.jpg'")
    cv2.imshow("Panorama Final", base_image)
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key == 27:
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()