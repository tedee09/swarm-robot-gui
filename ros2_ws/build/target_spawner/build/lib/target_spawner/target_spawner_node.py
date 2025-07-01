import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import random

# Konfigurasi arena dan grid
ARENA_WIDTH = 2.2
ARENA_HEIGHT = 1.7
X_MIN, Y_MIN = -1.1, -0.85
GRID_WIDTH = 40
GRID_HEIGHT = 30

# Fungsi konversi grid ke world
def grid_to_world(grid_y, grid_x, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    cell_width = ARENA_WIDTH / grid_width
    cell_height = ARENA_HEIGHT / grid_height
    x_world = X_MIN + (grid_x + 0.5) * cell_width
    y_world = Y_MIN + (grid_height - 1 - grid_y + 0.5) * cell_height
    return x_world, y_world

class TargetSpawnerNode(Node):
    def __init__(self):
        super().__init__('target_spawner_node')

        # Obstacle grid list bisa diupdate nanti jika node ini diintegrasikan lebih lanjut
        self.obstacles = set()

        # Publisher untuk target baru
        self.goal_pub = self.create_publisher(Point, 'goal_position', 10)

        # Subscriber dari path executor yang memberitahu bahwa robot mencapai target
        self.target_reached_sub = self.create_subscription(Bool, '/target_reached', self.target_reached_callback, 10)

        self.get_logger().info("Target Spawner Node started. Waiting for /target_reached...")

    def target_reached_callback(self, msg):
        if msg.data:
            self.spawn_random_target()

    def spawn_random_target(self):
        max_attempts = 100
        margin = 1  # grid cell

        for _ in range(max_attempts):
            gx = random.randint(margin, GRID_WIDTH - margin - 1)
            gy = random.randint(margin, GRID_HEIGHT - margin - 1)

            if self.is_safe(gx, gy):
                x, y = grid_to_world(gy, gx)
                target = Point()
                target.x = x
                target.y = y
                target.z = 0.0
                self.goal_pub.publish(target)
                self.get_logger().info(f"Spawned new target at grid ({gx}, {gy}) -> world ({x:.2f}, {y:.2f})")
                return

        self.get_logger().warn("Failed to spawn target after max attempts.")

    def is_safe(self, gx, gy):
        # Hindari obstacle dan sekitarnya (margin 1 sel)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                check = (gy + dy, gx + dx)
                if check in self.obstacles:
                    return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = TargetSpawnerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
