import cv2
import numpy as np

# Pilih dictionary ArUco (bisa 4x4, 5x5, 6x6, dst.)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# ID marker yang ingin dibuat (0 - 49 untuk DICT_4X4_50)
marker_id = 2  

# Ukuran gambar marker (dalam piksel)
marker_size = 200  

# Buat gambar marker
marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# Simpan gambar marker
cv2.imwrite(f"marker_{marker_id}.png", marker_image)

# Tampilkan marker
cv2.imshow("ArUco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()


