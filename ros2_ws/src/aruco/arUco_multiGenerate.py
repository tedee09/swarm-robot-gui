import cv2
import numpy as np

# Dictionary ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

for marker_id in range(8):
    marker_size = 200  # Ukuran marker
    border_bits = 1    # Jumlah kotak putih di pinggir (default = 1)

    # Buat marker
    marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)

    # Tambahkan margin putih
    border_size = 20
    marker_image_with_border = cv2.copyMakeBorder(
        marker_image, border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT, value=255
    )

    filename = f"marker_{marker_id}.png"
    cv2.imwrite(filename, marker_image_with_border)
    print(f"Marker {marker_id} disimpan sebagai {filename}")
