import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

image = cv2.imread("marker_0.png")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

if ids is not None:
    print("Detected ID:", ids[0][0])
else:
    print("Marker not recognized")
