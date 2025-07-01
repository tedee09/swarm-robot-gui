import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int32MultiArray
from geometry_msgs.msg import Point
import math
import time

def world_to_grid(x_world, y_world, grid_width=40, grid_height=30):
    ARENA_WIDTH = 2.2
    ARENA_HEIGHT = 1.7
    X_MIN = -ARENA_WIDTH / 2
    Y_MIN = -ARENA_HEIGHT / 2

    grid_x = int((x_world - X_MIN) / ARENA_WIDTH * grid_width)
    grid_y = int((y_world - Y_MIN) / ARENA_HEIGHT * grid_height)
    grid_y = grid_height - 1 - grid_y  # flip Y
    return grid_y, grid_x

def a_star(grid, start, goal):
    import heapq
    import math

    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    def h(p1, p2): return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from.get(current)
            return path[::-1], g_score[goal]

        for dy, dx in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            ny, nx = current[0]+dy, current[1]+dx
            if 0 <= ny < rows and 0 <= nx < cols and grid[ny][nx] == 0:
                step = 1.414 if dx != 0 and dy != 0 else 1.0
                tentative_g = g_score[current] + step
                if tentative_g < g_score.get((ny,nx), float('inf')):
                    came_from[(ny,nx)] = current
                    g_score[(ny,nx)] = tentative_g
                    heapq.heappush(open_set, (tentative_g + h((ny,nx), goal), (ny,nx)))
    return [], float('inf')

class RoleManagerNode(Node):
    def __init__(self):
        super().__init__('role_manager_node')

        self.declare_parameter("robot_ids", [1, 2, 3, 4, 5])
        self.robot_ids = self.get_parameter("robot_ids").get_parameter_value().integer_array_value

        self.robot_positions = {}
        self.goal_position = None
        self.role_publishers = {}
        self.goal_publishers = {}
        self.heading_publishers = {}
        self.colored_obstacles = set()
        self.obstacle_updated = False
        self.leader_last_position = None
        self.leader_stuck_start_time = None
        self.stuck_timeout = 2.5  # detik

        self.grid = [[0 for _ in range(40)] for _ in range(30)]  # kosong, tidak ada rintangan

        for rid in self.robot_ids:
            self.create_subscription(Point, f"/robot{rid}/robot_position", self.make_robot_position_callback(rid), 10)
            self.role_publishers[rid] = self.create_publisher(String, f"/robot{rid}/assigned_role", 10)
            self.heading_publishers[rid] = self.create_publisher(Float32, f"/robot{rid}/goal_heading", 10)
            self.create_subscription(Bool, f"/robot{rid}/target_reached", self.make_target_reached_callback(rid), 10)

        self.create_subscription(Point, "/leader_goal_position", self.goal_callback, 10)
        self.subscription_obstacle = self.create_subscription(Int32MultiArray, '/colored_obstacle_grids', self.obstacle_callback, 10)

        for rid in self.robot_ids:
            self.goal_publishers[rid] = self.create_publisher(Point, f"/robot{rid}/goal_position", 10)

        self.follower_goal_publishers = {
            rid: self.create_publisher(Point, f"/follower_goal_position/robot{rid}", 10)
            for rid in self.robot_ids
        }

        self.current_leader = None
        self.get_logger().info("Role Manager Node started.")

    def make_robot_position_callback(self, rid):
        def callback(msg):
            self.robot_positions[rid] = (msg.x, msg.y)
            self.assign_roles()
        return callback
    
    def make_target_reached_callback(self, rid):
        def callback(msg):
            if msg.data and self.current_leader == rid:
                self.get_logger().info(f"🏁 Robot{rid} reached target. Releasing leader lock.")
                self.current_leader = None  # Lepas lock
                self.assign_roles()         # Evaluasi ulang leader
        return callback

    def goal_callback(self, msg):
        self.goal_position = (msg.x, msg.y)
        self.current_leader = None  # 🔓 Lepas lock agar leader baru bisa dipilih
        self.get_logger().info("🎯 New goal received. Releasing leader lock for reassignment.")
        self.assign_roles()

    def obstacle_callback(self, msg):
        data = msg.data
        updated = set()
        for i in range(0, len(data), 2):
            updated.add((data[i], data[i+1]))
        if updated != self.colored_obstacles:
            self.colored_obstacles = updated
            self.obstacle_updated = True
            self.grid = [[0 for _ in range(40)] for _ in range(30)]
            for (y, x) in updated:
                if 0 <= y < 30 and 0 <= x < 40:
                    self.grid[y][x] = 1

            self.get_logger().info(f"🧱 Obstacle grid updated: {len(updated)} cells")
            self.assign_roles()

    def assign_roles(self):
        if self.goal_position is None or len(self.robot_positions) < len(self.robot_ids):
            self.get_logger().debug("❌ Waiting for complete robot positions or goal...")
            return

        current_time = time.time()

        # Hitung path dari tiap robot ke goal
        goal_grid = world_to_grid(self.goal_position[0], self.goal_position[1])
        distances = {}

        for rid, pos in self.robot_positions.items():
            robot_grid = world_to_grid(pos[0], pos[1])
            path, cost = a_star(self.grid, robot_grid, goal_grid)
            distances[rid] = cost

        sorted_robots = sorted(distances.items(), key=lambda x: x[1])
        best_rid, best_cost = sorted_robots[0]
        second_best_cost = sorted_robots[1][1] if len(sorted_robots) > 1 else float('inf')

        # ======== DETEKSI LEADER STUCK ========
        if self.current_leader in self.robot_positions:
            current_pos = self.robot_positions[self.current_leader]
            
            # Tidak bergerak
            if current_pos == self.leader_last_position:
                if self.leader_stuck_start_time is None:
                    self.leader_stuck_start_time = current_time
                elif current_time - self.leader_stuck_start_time > self.stuck_timeout:
                    self.get_logger().warn(f"🚫 Leader robot{self.current_leader} appears stuck. Re-evaluating...")
                    self.current_leader = None  # Ganti leader
            else:
                self.leader_last_position = current_pos
                self.leader_stuck_start_time = None

        # ======== CEK APAKAH LEADER MASIH LAYAK DIPERTAHANKAN ========
        if self.current_leader in distances:
            current_leader_cost = distances[self.current_leader]
            if current_leader_cost <= best_cost + 2 and current_leader_cost != float('inf'):
                self.get_logger().debug("🔒 Retaining current leader due to small path cost difference")
                self.publish_roles_and_goals()
                return

        # ======== PILIH LEADER BARU ========
        self.current_leader = best_rid
        self.leader_last_position = self.robot_positions[self.current_leader]
        self.leader_stuck_start_time = None
        self.get_logger().info(f"👑 New leader selected: robot{self.current_leader}")
        self.publish_roles_and_goals()


    def publish_roles_and_goals(self):
        leader_pos = self.robot_positions[self.current_leader]

        for rid in self.robot_ids:
            role_msg = String()
            role_msg.data = "leader" if rid == self.current_leader else "follower"
            self.role_publishers[rid].publish(role_msg)

        for rid in self.robot_ids:
            goal_msg = Point()
            if rid == self.current_leader:
                # Untuk leader, kirim goal sebenarnya
                goal_msg.x = self.goal_position[0]
                goal_msg.y = self.goal_position[1]
                goal_msg.z = 0.0
            else:
                # Untuk follower, goal = posisi leader
                angle_offset = (rid % 3) * (2 * math.pi / 3)  # 0, 120, 240 derajat
                offset = 0.15  # 15cm offset
                goal_msg.x = leader_pos[0] + offset * math.cos(angle_offset)
                goal_msg.y = leader_pos[1] + offset * math.sin(angle_offset)
                goal_msg.z = 0.0

                follower_pos = self.robot_positions[rid]
                dx = goal_msg.x - follower_pos[0]
                dy = goal_msg.y - follower_pos[1]
                heading = math.atan2(dy, dx)

                heading_msg = Float32()
                heading_msg.data = heading
                self.heading_publishers[rid].publish(heading_msg)
                self.follower_goal_publishers[rid].publish(goal_msg)

                self.get_logger().debug(f"[R{rid}] Goal heading to leader: {heading:.2f} rad")

            self.goal_publishers[rid].publish(goal_msg)

        self.get_logger().info(f"👑 Leader: robot_{self.current_leader}")
        self.get_logger().debug(f"[DEBUG] Robot positions: {self.robot_positions}")
        self.get_logger().debug(f"[DEBUG] Goal position: {self.goal_position}")

def main(args=None):
    rclpy.init(args=args)
    node = RoleManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
