import cv2
import numpy as np
import math
import os

# ------------------------------
# PARÁMETROS A AJUSTAR
# ------------------------------
# Puedes cambiar el warper (plane, cylindrical, spherical, etc.)
WARP_TYPE = 'spherical'  
# Otras opciones: 'plane', 'cylindrical', 'fisheye', 'stereographic', ...
# El "Stitcher" por defecto suele usar 'spherical' o 'cylindrical' para panoramas amplios.

# Ajusta el método de ajuste de exposición
EXPOSURE_COMPENSATOR = 'gain_blocks'  
# Otras opciones: 'no', 'gain', 'channels', 'channels_blocks'

# Ajusta el método de costura (seam):
SEAM_FINDER = 'gc_color'  
# Otras opciones: 'gc_color', 'gc_colorgrad', 'dp_color', ...

# Ajusta el blending
BLEND_TYPE = 'multiband'  
# Otras opciones: 'feather'
BLEND_STRENGTH = 5  # Ajusta fuerza de blending (depende de tu escena)

# Este es un ancho de referencia (en píxeles) para calibrar la “focal”
# de forma más estable. Suele ser la dimensión con la que quieres
# que se trabaje internamente.  
COMPOSITING_RESOLUTION = 800  # px

# ------------------------------
# LISTA DE IMÁGENES
# ------------------------------
images_list = [
    '/home/rodrigo/Documents/Fotogrametria/undistorted_imgs/undistorted_paisaje1.jpg',
    '/home/rodrigo/Documents/Fotogrametria/undistorted_imgs/undistorted_paisaje2.jpg',
    '/home/rodrigo/Documents/Fotogrametria/undistorted_imgs/undistorted_paisaje3.jpg',
    '/home/rodrigo/Documents/Fotogrametria/undistorted_imgs/undistorted_paisaje4.jpg',
    '/home/rodrigo/Documents/Fotogrametria/undistorted_imgs/undistorted_paisaje5.jpg'
]

# ------------------------------
# FUNCIÓN PRINCIPAL
# ------------------------------
def main():
    # 1. Cargar imágenes en memoria
    full_imgs = []
    for fname in images_list:
        img = cv2.imread(fname)
        if img is None:
            print(f"[ERROR] No se pudo leer la imagen: {fname}")
            return
        full_imgs.append(img)

    num_images = len(full_imgs)
    if num_images < 2:
        print("[ERROR] Se requieren al menos 2 imágenes.")
        return

    # 2. Encontrar features en cada imagen (SIFT, ORB, AKAZE, etc.)
    finder = cv2.SIFT_create()  # O cv2.AKAZE_create(), cv2.ORB_create(), etc.

    feature_config = cv2.detail.FeaturesFinder_create(finder)
    # Alternativa en OpenCV <= 4.5:
    # feature_config = cv2.detail_SiftFeaturesFinder_create()

    img_features = []
    for i, full_img in enumerate(full_imgs):
        # Para agilizar, reescalamos a COMPOSITING_RESOLUTION en ancho
        # y mantenemos proporción.
        scale = None
        if full_img.shape[1] > COMPOSITING_RESOLUTION:
            scale = COMPOSITING_RESOLUTION / full_img.shape[1]
            cur_img = cv2.resize(full_img, None, fx=scale, fy=scale,
                                 interpolation=cv2.INTER_LINEAR)
        else:
            cur_img = full_img.copy()
            scale = 1.0

        # Detectar features
        feature = feature_config.compute(cur_img)
        feature.img_idx = i
        feature.img_size = (cur_img.shape[1], cur_img.shape[0])
        # Guardamos la escala para usarla luego
        # (la necesitamos a la hora de estimar la focal)
        feature.scale = scale
        img_features.append(feature)

    # 3. Emparejar características entre las imágenes
    matcher = cv2.detail_BestOf2NearestMatcher_create(True, 0.3)
    pairwise_matches = matcher.apply2(img_features)
    matcher.collectGarbage()

    # 4. Estimar la cámara (focal, R, t) usando el estimador de homografías
    #    o puedes probar "HomographyBasedEstimator" si tu escena es esencialmente plana
    estimator = cv2.detail_HomographyBasedEstimator()
    b, cameras = estimator.apply(img_features, pairwise_matches, None)
    if not b:
        print("[ERROR] No se pudo estimar la cámara (Homografía).")
        return

    # Ajuste de parámetros focales y trasformaciones
    for cam in cameras:
        # La focal estimada la escalamos de vuelta, en caso de que hayamos reescalado la imagen
        sf = cam.focal
        # Ten en cuenta que la “focal” de OpenCV detail es en píxeles
        # Normalmente: cam.focal *= 1.0 / (escala)
        cam.focal = cam.focal / img_features[cam.idx].scale
        cam.ppx = cam.ppx / img_features[cam.idx].scale
        cam.ppy = cam.ppy / img_features[cam.idx].scale

    # 5. Ajustar rotaciones usando "Bundle Adjuster" (opcional, a veces no converge)
    adjuster = cv2.detail_BundleAdjusterRay()
    adjuster.setConfThresh(1)
    refine_mask = np.zeros((3,3), np.uint8)
    refine_mask[0,0] = 1  # refine focal length?
    refine_mask[0,1] = 1  # refine aspect ratio?
    refine_mask[0,2] = 1  # refine principal point?
    refine_mask[1,1] = 1
    adjuster.setRefinementMask(refine_mask)
    b, cameras = adjuster.apply(img_features, pairwise_matches, cameras)
    if not b:
        print("[ERROR] No se pudo ajustar la cámara (Bundle Adjust).")
        return

    # 6. Componer el “warp” de cada imagen. Seleccionar warper (plane, spherical, cylindrical, etc.)
    if WARP_TYPE == 'plane':
        warper_creator = cv2.detail.PlaneWarper_create()
    elif WARP_TYPE == 'cylindrical':
        warper_creator = cv2.detail.CylindricalWarper_create()
    elif WARP_TYPE == 'spherical':
        warper_creator = cv2.detail.SphericalWarper_create()
    else:
        # Por simplicidad, usa Spherical si no reconocemos la cadena
        warper_creator = cv2.detail.SphericalWarper_create()

    # 7. Definir la compensación de exposición
    if EXPOSURE_COMPENSATOR == 'no':
        compensator = cv2.detail.NoExposureCompensator_create()
    elif EXPOSURE_COMPENSATOR == 'gain':
        compensator = cv2.detail.GainCompensator_create()
    elif EXPOSURE_COMPENSATOR == 'channels':
        compensator = cv2.detail.ChannelsCompensator_create()
    elif EXPOSURE_COMPENSATOR == 'channels_blocks':
        compensator = cv2.detail.ChannelsCompensator_create(32, 32)
    else:
        # gain_blocks
        compensator = cv2.detail.GainBlocksCompensator_create(expos_comp_nr_feeds=1,
                                                              expos_comp_nr_filtering_iterations=2,
                                                              block_size=32)

    # 8. Definir el seam finder
    if SEAM_FINDER == 'gc_color':
        seam_finder = cv2.detail_GraphCutSeamFinder('COST_COLOR')
    elif SEAM_FINDER == 'gc_colorgrad':
        seam_finder = cv2.detail_GraphCutSeamFinder('COST_COLOR_GRAD')
    else:
        # Por defecto
        seam_finder = cv2.detail_DpSeamFinder('COLOR')

    # 9. Definir el blender (multiband o feather)
    if BLEND_TYPE == 'no':
        blender = cv2.detail.Blender_createDefault(cv2.detail.Blender_NO)
    elif BLEND_TYPE == 'feather':
        blender = cv2.detail_FeatherBlender()
        blender.setSharpness(1.0 / BLEND_STRENGTH)
    else:
        # multiband
        blender = cv2.detail_MultiBandBlender()
        blender.setNumBands(BLEND_STRENGTH)

    # Preparar las proyecciones y máscaras para seam y blending
    corners = []
    sizes = []
    masks_warped = []
    images_warped = []
    masks = []

    for i in range(num_images):
        # Carga la imagen original (full size)
        full_img = full_imgs[i]
        # Extraer la matriz de rotación y la focal calculada
        K = np.zeros((3, 3), dtype=np.float32)
        f = cameras[i].focal
        cx = cameras[i].ppx
        cy = cameras[i].ppy
        K[0, 0] = f
        K[1, 1] = f
        K[0, 2] = cx
        K[1, 2] = cy
        K[2, 2] = 1

        # Warper
        warper = warper_creator if callable(warper_creator) else warper_creator
        # Rotación
        R = cameras[i].R
        # Hacer el warp
        corner, image_wp = warper.warp(full_img, K, R, cv2.INTER_LINEAR, cv2.BORDER_REFLECT)
        corners.append(corner)
        sizes.append((image_wp.shape[1], image_wp.shape[0]))
        images_warped.append(image_wp)

        # Crear máscara base (todo 255)
        mask = np.ones((full_img.shape[0], full_img.shape[1]), dtype=np.uint8) * 255
        _, mask_wp = warper.warp(mask, K, R, cv2.INTER_NEAREST, cv2.BORDER_CONSTANT)
        masks_warped.append(mask_wp)

    # Compensación de exposición
    compensator.prepare(corners=corners, sizes=sizes, images=images_warped, masks=masks_warped)
    for i in range(num_images):
        compensator.apply(i, corners[i], images_warped[i], masks_warped[i])

    # Encontrar costuras
    seam_finder.find(images_warped, corners, masks_warped)

    # Configurar el blender (determinar bounding box global)
    # Calcular el rectángulo que engloba todas las imágenes warp
    dst_sz = cv2.detail.resultRoi(corners=corners, sizes=sizes)
    blend_width = math.sqrt(dst_sz[2] * dst_sz[3]) * 0.02
    blender.prepare(dst_sz)

    # Añadir cada imagen al blender
    for idx, img_wp in enumerate(images_warped):
        blender.feed(cv2.UMat(img_wp), cv2.UMat(masks_warped[idx]), corners[idx])

    # Combinar (blend) final
    result_img_umat, result_mask_umat = blender.blend(None, None)
    result_img = result_img_umat.get()
    result_mask = result_mask_umat.get()

    # Guardar y/o mostrar resultado
    cv2.imwrite("panorama_detail.jpg", result_img)
    print("[INFO] Panorama final guardado en panorama_detail.jpg")

    # Mostrar
    cv2.imshow("Panorama (detail)", result_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
