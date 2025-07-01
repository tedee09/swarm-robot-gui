import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Int32MultiArray, String
import heapq
import math
import time
import copy
from tf_transformations import quaternion_from_euler

# constants
ARENA_WIDTH = 2.2     # Total lebar arena (x)
ARENA_HEIGHT = 1.7    # Total tinggi arena (y)
X_MIN = -ARENA_WIDTH / 2   # = -1.1
Y_MIN = -ARENA_HEIGHT / 2  # = -0.85
GRID_WIDTH = 40
GRID_HEIGHT = 30
INFLATE_RADIUS = 2.0

# Konversi dari pixel ke koordinat dunia (hasil regresi 4 titik kalibrasi)
def pixel_to_world(px, py):
    x_world = 0.001716 * px + 0.000002 * py - 1.07155
    y_world = -0.001716 * py + 0.000001 * px + 0.83868
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

def inflate_obstacles(grid, radius=INFLATE_RADIUS):
    rows = len(grid)
    cols = len(grid[0])
    inflated = [row[:] for row in grid]

    for y in range(rows):
        for x in range(cols):
            if grid[y][x] == 1:
                for dy in range(-int(radius), int(radius)+1):
                    for dx in range(-int(radius), int(radius)+1):
                        distance = math.sqrt(dx*dx + dy*dy)
                        ny = y + dy
                        nx = x + dx
                        if 0 <= ny < rows and 0 <= nx < cols and distance <= radius:
                            inflated[ny][nx] = 1
    return inflated

def find_offset_goal(leader_pos, follower_pos, other_follower_positions, distance_cells=3):
    ly, lx = leader_pos
    fy, fx = follower_pos

    # Arah dari goal ke robot
    dy = fy - ly
    dx = fx - lx
    angle = math.atan2(dy, dx)
    
    # Hitung offset grid berdasarkan angle
    oy = round(math.sin(angle) * distance_cells)
    ox = round(math.cos(angle) * distance_cells)

    # Coba posisi ideal lebih dahulu
    gy, gx = ly + oy, lx + ox
    occupied = set(other_follower_positions)

    def is_position_valid(gy, gx, occupied):
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                ny, nx = gy + dy, gx + dx
                if (ny, nx) in occupied:
                    return False
        return True

    if is_position_valid(gy, gx, occupied):
        return (gy, gx)

    # Kalau gagal, fallback ke offset tetap seperti sebelumnya
    candidate_offsets = [
        (2, 0), (2, -2), (2, 2), (2, -4), (2, 4),
        (3, -3), (3, -1), (3, 1), (3, 3),
        (4, -2), (4, 0), (4, 2),
        (5, -3), (5, -1), (5, 1), (5, 3),
        (6, 0)
    ]
    for oy, ox in candidate_offsets:
        gy, gx = ly + oy, lx + ox
        if is_position_valid(gy, gx, occupied):
            return (gy, gx)

    return follower_pos

def mark_dynamic_obstacles(grid, dynamic_positions, self_pos, radius=1):
    for y, x in dynamic_positions:
        if (y, x) == self_pos:
            continue  # Jangan blok posisi robot sendiri
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                ny, nx = y + dy, x + dx
                if 0 <= ny < GRID_HEIGHT and 0 <= nx < GRID_WIDTH:
                    grid[ny][nx] = 1

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.declare_parameter("robot_id", 1)
        rid = self.get_parameter("robot_id").get_parameter_value().integer_value

        self.original_grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
        self.grid = inflate_obstacles(self.original_grid, radius=INFLATE_RADIUS)

        self.robot_pos = None
        self.goal_pos = None
        self.colored_obstacles = set()
        self.obstacle_updated = False
        self.current_role = "follower"
        self.distance_cells = 5
        self.last_pos = None
        self.last_move_time = time.time()
        self.stuck_timeout = 2.0  # detik, bisa kamu ubah sesuai eksperimen
        self.stuck_timer = self.create_timer(1.0, self.check_stuck)
        self.last_path = []
        self.last_plan_time = time.time()
        self.replan_interval = 1.0

        self.robot_id = rid
        self.subscription_robot = self.create_subscription(Point, f'/robot{self.robot_id}/robot_position', self.robot_callback, 10)
        self.leader_goal_sub = self.create_subscription(Point, '/leader_goal_position', self.leader_goal_callback, 10)
        self.follower_goal_sub = self.create_subscription(Point, f'/follower_goal_position/robot{self.robot_id}', self.follower_goal_callback, 10)
        self.subscription_obstacle = self.create_subscription(Int32MultiArray, '/colored_obstacle_grids', self.obstacle_callback, 10)
        self.subscription_role = self.create_subscription(String, f'/robot{self.robot_id}/assigned_role', self.role_callback, 10)
        self.path_pub = self.create_publisher(Path, f'/robot{self.robot_id}/path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, f"/robot_{self.robot_id}/cmd_vel", 10)

        self.other_follower_positions = []
        self.sub_all_follower_pos = self.create_subscription(
            Int32MultiArray,
            '/all_follower_positions',
            self.all_follower_callback,
            10
        )

        self.get_logger().info("Path Planner Node with dynamic obstacle handling started.")

    def role_callback(self, msg):
        self.get_logger().info(f"[R{self.robot_id}] role_callback triggered with data: {msg.data}")
        if msg.data != self.current_role:
            self.current_role = msg.data
            self.get_logger().info(f"[R{self.robot_id}] 🧠 Role updated to: {self.current_role}")
            self.plan_path()

    def robot_callback(self, msg):
        new_pos = world_to_grid(msg.x, msg.y)
        if new_pos != self.robot_pos:
            distance = math.hypot(new_pos[0] - self.robot_pos[0], new_pos[1] - self.robot_pos[1]) if self.robot_pos else float('inf')
            self.robot_pos = new_pos
            if self.goal_pos is not None:
                dist_to_goal = math.hypot(self.robot_pos[0] - self.goal_pos[0], self.robot_pos[1] - self.goal_pos[1])
                if dist_to_goal < 2:  # threshold 2 grid
                    self.get_logger().info(f"[R{self.robot_id}] 🟡 Near goal ({dist_to_goal:.2f} < 2), skipping replan.")
                    return
            if distance >= 1:  # Hanya replan kalau beda grid 1 atau lebih
                self.plan_path()
    
    def try_unstuck_maneuver(self):
        self.get_logger().info(f"[R{self.robot_id}] 🔄 Attempting unstuck maneuver...")

        twist = Twist()
        twist.linear.x = -0.05      # Gerak mundur pelan
        twist.angular.z = 0.5       # Putar kiri sambil mundur
        self.cmd_vel_pub.publish(twist)

        # Tunggu 0.5 detik untuk melakukan manuver
        time.sleep(0.5)

        # Stop pergerakan
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def leader_goal_callback(self, msg):
        if self.current_role == "leader":
            new_goal = world_to_grid(msg.x, msg.y)
            if self.goal_pos is None or math.hypot(new_goal[0] - self.goal_pos[0], new_goal[1] - self.goal_pos[1]) >= 1:
                self.goal_pos = new_goal
                self.plan_path()

    def follower_goal_callback(self, msg):
        if self.current_role == "follower":
            new_goal = world_to_grid(msg.x, msg.y)
            if new_goal != self.goal_pos:
                self.goal_pos = new_goal
                self.plan_path()

    def all_follower_callback(self, msg):
        data = msg.data
        self.other_follower_positions = [(data[i], data[i+1]) for i in range(0, len(data), 2)]

    def check_stuck(self):
        if self.robot_pos == self.last_pos:
            if time.time() - self.last_move_time > self.stuck_timeout:
                self.get_logger().warn(f"[R{self.robot_id}] 🚨 Robot might be stuck at {self.robot_pos}.")
                
                # 👉 Tambahkan manuver sebelum replan
                self.try_unstuck_maneuver()

                self.plan_path()  # Tetap replan setelah itu
                self.last_move_time = time.time()
        else:
            self.last_move_time = time.time()
            self.last_pos = self.robot_pos

    def goal_callback(self, msg):
        new_goal = world_to_grid(msg.x, msg.y)
        if new_goal != self.goal_pos:
            self.goal_pos = new_goal
            self.plan_path()

    def obstacle_callback(self, msg):
        data = msg.data
        updated = set()
        for i in range(0, len(data), 2):
            updated.add((data[i], data[i+1]))
        if updated != self.colored_obstacles:
            self.colored_obstacles = updated
            self.obstacle_updated = True
            self.plan_path()

    def plan_path(self):
        now = time.time()
        if now - self.last_plan_time < self.replan_interval:
            self.get_logger().info("🕒 Skipping replan due to cooldown.")
            return
        self.last_plan_time = now

        if self.robot_pos is None or self.goal_pos is None:
            return

        self.get_logger().info(f"[R{self.robot_id}] 🧠 Planning path as {self.current_role.upper()} from {self.robot_pos} to {self.goal_pos}...")

        runtime_grid = copy.deepcopy(self.original_grid)
        for y, x in self.colored_obstacles:
            if 0 <= y < len(runtime_grid) and 0 <= x < len(runtime_grid[0]):
                runtime_grid[y][x] = 1

        runtime_grid = inflate_obstacles(runtime_grid, radius=INFLATE_RADIUS)
        mark_dynamic_obstacles(runtime_grid, self.other_follower_positions, self.robot_pos, radius=2)

        if self.robot_pos == self.goal_pos:
            self.get_logger().info("Start == goal, no path needed.")
            return

        self.get_logger().info(f"Planning path from {self.robot_pos} to {self.goal_pos}...")
        planner = DSLitePlanner(runtime_grid)
        if self.current_role == "follower":
            adjusted_goal_grid = find_offset_goal(self.goal_pos, self.robot_pos, self.other_follower_positions, distance_cells=self.distance_cells)
            self.get_logger().info(f"[R{self.robot_id}] Adjusted follower goal from {self.goal_pos} to {adjusted_goal_grid}")
            path = planner.plan(self.robot_pos, adjusted_goal_grid)
        else:
            path = planner.plan(self.robot_pos, self.goal_pos)

        if not path:
            self.get_logger().warn("No path found.")
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"

        if path != self.last_path:
            self.last_path = path  # Update path terakhir
            path_msg = Path()
            path_msg.header.frame_id = "map"
            
            for i, (y, x) in enumerate(path):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                wx, wy = grid_to_world(y, x)
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                pose.pose.position.z = 0.0

                if i < len(path) - 1:
                    dy, dx = path[i+1][0] - y, path[i+1][1] - x
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = 0.0

                qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.get_logger().info(f"✅ Published path with {len(path)} points.")
        else:
            self.get_logger().info("🟡 Path not changed, skipping republish.")

class DSLitePlanner:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.rhs = {}
        self.g = {}
        self.U = []
        self.km = 0

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def calculate_key(self, s, start):
        g_rhs = min(self.g.get(s, float('inf')), self.rhs.get(s, float('inf')))
        return (g_rhs + self.heuristic(start, s) + self.km, g_rhs)

    def update_vertex(self, u, start, goal):
        if u != goal:
            min_rhs = float('inf')
            for s in self.get_neighbors(u):
                if self.grid[s[0]][s[1]] == 1:
                    continue
                cost = self.heuristic(u, s)
                min_rhs = min(min_rhs, self.g.get(s, float('inf')) + cost)
            self.rhs[u] = min_rhs

        self.U = [item for item in self.U if item[1] != u]
        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            heapq.heappush(self.U, (self.calculate_key(u, start), u))

    def compute_shortest_path(self, start, goal):
        while self.U and (self.U[0][0] < self.calculate_key(start, start) or self.rhs.get(start, float('inf')) != self.g.get(start, float('inf'))):
            k_old, u = heapq.heappop(self.U)
            k_new = self.calculate_key(u, start)
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                for s in self.get_neighbors(u):
                    self.update_vertex(s, start, goal)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u, start, goal)
                for s in self.get_neighbors(u):
                    self.update_vertex(s, start, goal)

    def get_neighbors(self, pos):
        y, x = pos
        directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
        result = []
        for dy, dx in directions:
            ny, nx = y + dy, x + dx
            if 0 <= ny < self.rows and 0 <= nx < self.cols:
                result.append((ny, nx))
        return result

    def reconstruct_path(self, start, goal):
        path = [start]
        current = start
        visited = set()
        while current != goal:
            neighbors = self.get_neighbors(current)
            min_cost = float('inf')
            next_node = None
            for n in neighbors:
                if self.grid[n[0]][n[1]] == 1 or n in visited:
                    continue
                cost = self.g.get(n, float('inf')) + self.heuristic(current, n)
                if cost < min_cost:
                    min_cost = cost
                    next_node = n
            if next_node is None:
                return []  # No path
            visited.add(current)
            current = next_node
            path.append(current)
        return path

    def plan(self, start, goal):
        self.g = {}
        self.rhs = {}
        self.U = []
        self.rhs[goal] = 0.0
        self.g[goal] = float('inf')
        heapq.heappush(self.U, (self.calculate_key(goal, start), goal))
        self.compute_shortest_path(start, goal)
        if self.rhs.get(start, float('inf')) == float('inf'):
            return []
        return self.reconstruct_path(start, goal)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
