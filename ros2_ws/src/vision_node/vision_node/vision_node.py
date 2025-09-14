import rclpy
from rclpy.node import Node
from controller import Robot
import cv2
import numpy as np
from geometry_msgs.msg import Point
import cv2.aruco as aruco
from std_msgs.msg import Int32MultiArray, Float32
from visualization_msgs.msg import Marker, MarkerArray
import math
from filterpy.kalman import KalmanFilter

# constants
ARENA_WIDTH = 1.9     # Total lebar arena (x)
ARENA_HEIGHT = 1.1    # Total tinggi arena (y)
X_MIN = -ARENA_WIDTH / 2   # (kiri)
Y_MIN = -ARENA_HEIGHT / 2  # (bawah)
GRID_WIDTH = 40
GRID_HEIGHT = 30
INFLATE_RADIUS = 2.0

# Konversi dari pixel ke koordinat dunia (hasil regresi 4 titik kalibrasi)
def pixel_to_world(px, py):
    x_world = 0.0015 * px + 0.0 * py - 0.985
    y_world = 0.0 * px - 0.0016 * py + 0.579
    return x_world, y_world

# Konversi dari world ke grid
def world_to_grid(x_world, y_world, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    grid_x = int((x_world - X_MIN) / ARENA_WIDTH * grid_width)
    grid_y = int((y_world - Y_MIN) / ARENA_HEIGHT * grid_height)
    grid_y = grid_height - 1 - grid_y  # Y di-flip agar origin grid di kiri atas
    return grid_y, grid_x

# Konversi dari grid ke world
def grid_to_world(grid_y, grid_x, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    cell_width = ARENA_WIDTH / grid_width
    cell_height = ARENA_HEIGHT / grid_height
    x_world = X_MIN + (grid_x + 0.5) * cell_width
    y_world = Y_MIN + (grid_height - 1 - grid_y + 0.5) * cell_height
    return x_world, y_world

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter("robot_ids", [1, 2, 3, 4, 5])
        self.robot_ids = self.get_parameter("robot_ids").get_parameter_value().integer_array_value

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)

        self.get_logger().info("Vision node started with ArUco + Color tracking...")

        # Publisher
        self.leader_goal_pub = self.create_publisher(Point, '/leader_goal_position', 10)
        self.obstacle_pub = self.create_publisher(Int32MultiArray, '/colored_obstacle_grids', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self.robots_marker_pub = self.create_publisher(MarkerArray, '/robot_markers', 10)
        self.leader_goal_marker_pub = self.create_publisher(MarkerArray, '/leader_goal_marker', 10)

        self.robot_position_pubs = {
            rid: self.create_publisher(Point, f'/robot{rid}/robot_position', 10) for rid in self.robot_ids
        }

        self.previous_positions = {rid: None for rid in self.robot_ids}
        self.last_goal_seen_time = 0
        self.kalman_filters = {}
        for rid in self.robot_ids:
            kf = KalmanFilter(dim_x=4, dim_z=2)
            dt = 1/10  # set 10 FPS aja
            
            # State: [x, y, vx, vy]
            kf.F = np.array([[1, 0, dt, 0],
                            [0, 1, 0, dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
            
            kf.H = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0]])
            
            kf.P *= 10.0  # inisialisasi ketidakpastian
            kf.R *= 0.2   # noise pengukuran (semakin kecil = lebih percaya sensor)
            kf.Q *= 0.05  # noise proses (semakin besar = lebih adaptif terhadap perubahan cepat)
            
            kf.x = np.array([[0], [0], [0], [0]])  # inisialisasi posisi awal
            self.kalman_filters[rid] = kf

        # ArUco config
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        # HSV bounds for yellow
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])

    def detect_aruco_markers(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.equalizeHist(gray)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        robot_ids = self.robot_ids  # Daftar ID robot
        robot_colors = {
            1: (1.0, 0.0, 0.0),  # Merah
            2: (0.0, 1.0, 0.0),  # Hijau
            3: (0.0, 0.0, 1.0),  # Biru
            4: (1.0, 1.0, 0.0),  # Kuning (Merah + Hijau)
            5: (1.0, 0.0, 1.0),  # Magenta (Merah + Biru)
        }
        robot_markers = MarkerArray()
        marker_id = 0
        if ids is not None:
            for i in range(len(ids)):
                id = int(ids[i][0])
                c = corners[i][0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                self.get_logger().info(f"[PIXEL] ({cx:.2f}, {cy:.2f})")

                x_world, y_world = pixel_to_world(cx, cy)
                msg = Point()
                msg.x = x_world
                msg.y = y_world
                msg.z = 0.0

                if id == 0:
                    self.leader_goal_pub.publish(msg)
                    self.last_goal_seen_time = self.get_clock().now().nanoseconds
                    self.get_logger().info(f"[GOAL] Published to leader robot: ({x_world:.2f}, {y_world:.2f})")
                    color = (255, 0, 0)
                    label = "GOAL"
                    
                    # Marker untuk goal
                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.id = 999
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.pose.position.x = x_world
                    text_marker.pose.position.y = y_world
                    text_marker.pose.position.z = 0.15  # Di atas permukaan
                    text_marker.pose.orientation.w = 1.0
                    text_marker.scale.z = 0.15  # Ukuran teks
                    text_marker.color.r = 1.0
                    text_marker.color.g = 0.2
                    text_marker.color.b = 0.6
                    text_marker.color.a = 1.0
                    text_marker.text = "FINISH"
                    text_marker.lifetime.sec = 1  # Diperbarui setiap frame
                    self.leader_goal_marker_pub.publish(MarkerArray(markers=[text_marker]))
                elif id in self.robot_ids:
                    kf = self.kalman_filters[id]
                    z = np.array([msg.x, msg.y])  # pengamatan
                    if kf.x[0] == 0 and kf.x[1] == 0:
                        kf.x[:2] = np.array([[msg.x], [msg.y]])
                    kf.predict()
                    kf.update(z)

                    # Ambil hasil filter
                    filtered_x = float(kf.x[0])
                    filtered_y = float(kf.x[1])

                    filtered_msg = Point()
                    filtered_msg.x = filtered_x
                    filtered_msg.y = filtered_y
                    filtered_msg.z = 0.0
                    self.robot_position_pubs[id].publish(filtered_msg)
                    prev_pos = self.previous_positions[id]
                    if prev_pos is not None:
                        dx = filtered_msg.x - prev_pos[0]
                        dy = filtered_msg.y - prev_pos[1]
                        if abs(dx) > 1e-3 or abs(dy) > 1e-3:
                            heading = math.atan2(dy, dx)
                            heading_msg = Float32()
                            heading_msg.data = heading
                            # self.heading_publishers[id].publish(heading_msg)

                    self.previous_positions[id] = (filtered_msg.x, filtered_msg.y)
                    self.get_logger().info(f"[ROBOT {id}] Pos: ({x_world:.2f}, {y_world:.2f})")
                    # self.get_logger().info(f"[ROBOT] ({x_world:.2f}, {y_world:.2f})")
                    color = (0, 0, 255)
                    label = f"ROBOT {id}"
                    r, g, b = robot_colors.get(id, (1.0, 1.0, 1.0))  # default putih jika ID tidak dikenal

                    # Marker untuk robot
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.id = marker_id
                    marker_id += 1
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = x_world
                    marker.pose.position.y = y_world
                    marker.pose.position.z = 0.05
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.05
                    marker.color.r = r
                    marker.color.g = g
                    marker.color.b = b
                    marker.color.a = 1.0
                    marker.lifetime.sec = 1
                    robot_markers.markers.append(marker)

                    # Marker TEKS di atas CUBE
                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.id = marker_id
                    marker_id += 1
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.pose.position.x = x_world 
                    text_marker.pose.position.y = y_world
                    text_marker.pose.position.z = 0.18  # Lebih tinggi dari CUBE
                    text_marker.pose.orientation.w = 1.0
                    text_marker.scale.z = 0.08
                    text_marker.color.r = 1.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 1.0
                    text_marker.color.a = 1.0
                    text_marker.text = f"ROBOT{id}"
                    text_marker.lifetime.sec = 1
                    robot_markers.markers.append(text_marker)
                else:
                    color = (0, 255, 0)
                    label = f"ID {id}"

                cv2.circle(image, (cx, cy), 6, color, -1)
                cv2.putText(image, label, (cx, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
            
            if robot_markers.markers:
                self.robots_marker_pub.publish(robot_markers)
        
        # Check timeout for goal marker
        now = self.get_clock().now().nanoseconds
        if now - self.last_goal_seen_time > 3e9:
            self.get_logger().warn("⚠️ Goal marker (ID=0) tidak terdeteksi selama 3 detik!")

    def detect_colored_obstacles(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        grid_coords = set()
        marker_array = MarkerArray()
        marker_id = 0
        image_h, image_w = image.shape[:2]

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2
                cy = y + h // 2

                x_world_min, y_world_min = pixel_to_world(x, y + h)
                x_world_max, y_world_max = pixel_to_world(x + w, y)

                for gx in range(40): 
                    for gy in range(30):
                        wx, wy = grid_to_world(gy, gx, 40, 30)
                        if x_world_min <= wx <= x_world_max and y_world_min <= wy <= y_world_max:
                            grid_coords.add((gy, gx))

                # Visual feedback on image
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(image, "Obstacle", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                # Marker for RViz
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                x_world_center, y_world_center = pixel_to_world(cx, cy)
                marker.pose.position.x = x_world_center
                marker.pose.position.y = y_world_center
                marker.pose.position.z = 0.05  # slightly above ground

                # Estimasi ukuran berdasarkan bounding box kamera
                x_world_left, y_world_bottom = pixel_to_world(x, y + h)
                x_world_right, y_world_top = pixel_to_world(x + w, y)
                size_x = abs(x_world_right - x_world_left)
                size_y = abs(y_world_top - y_world_bottom)

                marker.scale.x = max(size_x, 0.05)
                marker.scale.y = max(size_y, 0.05)
                marker.scale.z = 0.05

                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.lifetime.sec = 1  # auto-hapus jika tidak diperbarui

                marker_array.markers.append(marker)

                self.get_logger().info(f"[OBSTACLE COLOR] area size ({size_x:.2f}, {size_y:.2f}) covers {len(grid_coords)} grid cells")

        # Publish obstacles as Int32MultiArray (for path planner)
        msg = Int32MultiArray()
        msg.data = [coord for gxgy in grid_coords for coord in gxgy]

        # self.get_logger().info(f"[GRID SEND] Total {len(grid_coords)} obstacles sent: {sorted(grid_coords)}")

        self.obstacle_pub.publish(msg)

        # Publish markers to RViz
        self.marker_pub.publish(marker_array)

    def run(self):
        while self.robot.step(self.timestep) != -1:
            width = self.camera.getWidth()
            height = self.camera.getHeight()
            img = self.camera.getImage()
            image = np.frombuffer(img, np.uint8).reshape((height, width, 4))
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

            self.detect_aruco_markers(image)
            self.detect_colored_obstacles(image)

            cv2.imshow("Camera View", image)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break

def main(args=None):
    import sys
    filtered_args = [arg for arg in sys.argv if not arg.startswith("--webots-robot-name")]
    rclpy.init(args=filtered_args)
    node = VisionNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
