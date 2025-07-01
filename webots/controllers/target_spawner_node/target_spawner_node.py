import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String, Int32
from geometry_msgs.msg import Point
import random
from controller import Supervisor

# Konfigurasi arena dan grid
ARENA_WIDTH = 1.9     # Total lebar arena (x)
ARENA_HEIGHT = 1.1    # Total tinggi arena (y)
X_MIN = -ARENA_WIDTH / 2   # = -0.95
Y_MIN = -ARENA_HEIGHT / 2  # = -0.55
GRID_WIDTH = 40
GRID_HEIGHT = 30
TOTAL_ROBOTS = 5

# Konversi dari grid ke koordinat dunia
def grid_to_world(grid_y, grid_x, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    cell_width = ARENA_WIDTH / grid_width
    cell_height = ARENA_HEIGHT / grid_height
    x_world = X_MIN + (grid_x + 0.5) * cell_width
    y_world = Y_MIN + (grid_height - 1 - grid_y + 0.5) * cell_height
    return x_world, y_world

class TargetSpawnerNode(Node):
    def __init__(self):
        super().__init__('target_spawner_node')

        self.supervisor = Supervisor()
        self.robot_node = self.supervisor.getFromDef("target")
        self.translation_field = self.robot_node.getField("translation")
        self.timestep = int(self.supervisor.getBasicTimeStep())

        self.leader_id = None  # Default tidak ada leader
        self.obstacles = set()
        self.robot_positions = {}

        for robot_id in range(1, TOTAL_ROBOTS + 1):
            self.create_subscription(
                Point,
                f'/robot{robot_id}/robot_position',
                lambda msg, rid=robot_id: self.update_robot_position(msg, rid),
                10
            )

        for robot_id in range(1, TOTAL_ROBOTS + 1):
            self.create_subscription(
                String,
                f'/robot{robot_id}/assigned_role',
                lambda msg, rid=robot_id: self.role_update_callback(msg, rid),
                10
            )

        # Langganan target_reached dari semua robot
        for robot_id in range(1, TOTAL_ROBOTS + 1):
            self.create_subscription(
                Bool,
                f'/robot{robot_id}/leader_target_reached',
                lambda msg, rid=robot_id: self.leader_target_reached_callback(msg, rid),
                10
            )

        # Langganan obstacle grid
        self.obstacle_sub = self.create_subscription(
            Int32MultiArray, '/colored_obstacle_grids', self.obstacle_callback, 10)

        self.get_logger().info("Target Spawner Node started. Waiting for /target_reached...")

    def update_robot_position(self, msg, robot_id):
        self.robot_positions[robot_id] = (msg.x, msg.y)

    def obstacle_callback(self, msg):
        data = msg.data
        self.obstacles = set()
        for i in range(0, len(data), 2):
            gy = data[i]
            gx = data[i+1]
            self.obstacles.add((gy, gx))

    def leader_update_callback(self, msg):
        if 1 <= msg.data <= TOTAL_ROBOTS:
            self.leader_id = msg.data
            self.get_logger().info(f"🔁 Leader robot updated to robot{self.leader_id}")
        else:
            self.get_logger().warn(f"⚠️ Invalid leader id received: {msg.data}")

    def role_update_callback(self, msg, robot_id):
        if msg.data.lower() == "leader":
            if self.leader_id != robot_id:
                self.leader_id = robot_id
                self.get_logger().info(f"🔁 Leader updated to robot{robot_id}")
        elif self.leader_id == robot_id:
            self.get_logger().info(f"❌ Robot{robot_id} is no longer leader.")
            self.leader_id = None

    def leader_target_reached_callback(self, msg, robot_id):
        if msg.data and robot_id == self.leader_id:
            self.get_logger().info(f"✅ Leader robot{robot_id} reached target. Spawning new target...")
            self.spawn_random_target()

    def spawn_random_target(self):
        max_attempts = 100
        margin = 1

        for _ in range(max_attempts):
            gx = random.randint(margin, GRID_WIDTH - margin - 1)
            gy = random.randint(margin, GRID_HEIGHT - margin - 1)

            if self.is_safe(gx, gy):
                x, y = grid_to_world(gy, gx)

                # 1. Pindahkan objek fisik di Webots
                self.translation_field.setSFVec3f([x, y, 0.002])

                self.get_logger().info(f"[SPAWN] Grid ({gx},{gy}) -> World ({x:.2f}, {y:.2f})")
                return

        self.get_logger().warn("Failed to spawn target after 100 attempts.")

    def is_safe(self, gx, gy):
        # Cek overlap dengan obstacle
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                check = (gy + dy, gx + dx)
                if check in self.obstacles:
                    return False
        
        # Konversi grid ke world
        x_world, y_world = grid_to_world(gy, gx)

        # Cek tidak terlalu dekat dengan robot
        for pos in self.robot_positions.values():
            distance = ((pos[0] - x_world) ** 2 + (pos[1] - y_world) ** 2) ** 0.5
            if distance < 0.15:
                return False
            
        return True

def main():
    rclpy.init()
    node = TargetSpawnerNode()

    try:
        while node.supervisor.step(node.timestep) != -1:
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


