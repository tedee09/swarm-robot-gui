import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Int32MultiArray, String, Float32
import heapq
import math
import time
import copy
from tf_transformations import quaternion_from_euler
from collections import deque

# ===== constants =====
ARENA_WIDTH = 1.9
ARENA_HEIGHT = 1.1
X_MIN = -ARENA_WIDTH / 2   # = -1.1
Y_MIN = -ARENA_HEIGHT / 2  # = -0.85
GRID_WIDTH = 40
GRID_HEIGHT = 30
INFLATE_RADIUS = 2.0

# ===== helpers: world/pixel/grid =====
def pixel_to_world(px, py):
    x_world = 0.001716 * px + 0.000002 * py - 1.07155
    y_world = -0.001716 * py + 0.000001 * px + 0.83868
    return x_world, y_world

def world_to_grid(x_world, y_world, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    grid_x = int((x_world - X_MIN) / ARENA_WIDTH * grid_width)
    grid_y = int((y_world - Y_MIN) / ARENA_HEIGHT * grid_height)
    grid_y = grid_height - 1 - grid_y  # flip Y agar origin grid di kiri atas
    # --- CLAMP ke batas grid (FIX) ---
    grid_x = max(0, min(grid_width - 1, grid_x))
    grid_y = max(0, min(grid_height - 1, grid_y))
    return grid_y, grid_x

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
    r_int = int(math.ceil(radius))
    for y in range(rows):
        for x in range(cols):
            if grid[y][x] == 1:
                for dy in range(-r_int, r_int + 1):
                    for dx in range(-r_int, r_int + 1):
                        distance = math.sqrt(dx*dx + dy*dy)
                        ny = y + dy
                        nx = x + dx
                        if 0 <= ny < rows and 0 <= nx < cols and distance <= radius:
                            inflated[ny][nx] = 1
    return inflated

def _is_behind(leader_pos, cand_pos, leader_heading, eps=0.2):
    ly, lx = leader_pos; cy, cx = cand_pos
    fwd = (math.sin(leader_heading), math.cos(leader_heading))  # frame GRID
    v   = (cy - ly, cx - lx)
    return (v[0]*fwd[0] + v[1]*fwd[1]) <= -eps

def find_offset_goal_smart(leader_pos, follower_pos, grid, occupied,
                           distance_cells, max_expand=8, min_move=2,
                           leader_heading=None):
    ly, lx = leader_pos
    fy, fx = follower_pos

    # jika ada heading leader → belakang = heading + pi
    if leader_heading is not None:
        angle = leader_heading + math.pi
    else:
        # fallback lama: arah dari leader ke follower
        dy = fy - ly
        dx = fx - lx
        angle = math.atan2(dy, dx)

    H, W = len(grid), len(grid[0])

    def free_and_not_near(y, x):
        if y < 0 or y >= H or x < 0 or x >= W:
            return False
        if grid[y][x] == 1:
            return False
        for ddy in (-1,0,1):
            for ddx in (-1,0,1):
                if (y+ddy, x+ddx) in occupied:
                    return False
        return True

    def dir_point(r):
        oy = round(math.sin(angle) * r)
        ox = round(math.cos(angle) * r)
        return ly + oy, lx + ox

    # 1) coba radius mengecil (tengah-belakang lebih dulu)
    for r in range(distance_cells, 0, -1):
        gy, gx = dir_point(r)
        if (leader_heading is not None) and (not _is_behind((ly, lx), (gy, gx), leader_heading)):
            continue
        if free_and_not_near(gy, gx) and math.hypot(gy - fy, gx - fx) >= min_move:
            return (gy, gx)

    # 2) ekspansi 8-arah + arah belakang
    for r in range(distance_cells, distance_cells + max_expand + 1):
        cands = [
            (ly, lx + r), (ly, lx - r), (ly + r, lx), (ly - r, lx),
            (ly + r, lx + r), (ly + r, lx - r), (ly - r, lx + r), (ly - r, lx - r),
            dir_point(r)
        ]
        for gy, gx in cands:
            if (leader_heading is not None) and (not _is_behind((ly, lx), (gy, gx), leader_heading)):
                continue
            if free_and_not_near(gy, gx) and math.hypot(gy - fy, gx - fx) >= min_move:
                return (gy, gx)

    return follower_pos

def mark_dynamic_obstacles(grid, dynamic_positions, self_pos, radius=1):
    rows = len(grid)         # (FIX) pakai ukuran grid aktual
    cols = len(grid[0])
    for y, x in dynamic_positions:
        if (y, x) == self_pos:
            continue
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                ny, nx = y + dy, x + dx
                if 0 <= ny < rows and 0 <= nx < cols:
                    grid[ny][nx] = 1

# (Opsional) Clearance BFS (tidak dipakai sekarang, tapi sudah benar & siap pakai)
def compute_clearance(grid):
    H, W = len(grid), len(grid[0])
    INF = 10**9
    dist = [[INF]*W for _ in range(H)]
    q = deque()
    for y in range(H):
        for x in range(W):
            if grid[y][x] == 1:
                dist[y][x] = 0
                q.append((y, x))
    while q:
        y, x = q.popleft()
        for dy, dx in [(-1,0), (1,0), (0,-1), (0,1)]:
            ny, nx = y + dy, x + dx
            if 0 <= ny < H and 0 <= nx < W and dist[ny][nx] == INF:
                dist[ny][nx] = dist[y][x] + 1
                q.append((ny, nx))
    return dist

def build_social_penalty(H, W, others, keep_dist=4):
    # Soft repulsion: makin dekat robot lain → penalti makin besar (bukan di-block)
    pen = [[0.0]*W for _ in range(H)]
    R = keep_dist + 2
    k = 1.0 / float(max(1, keep_dist)**2)
    for oy, ox in others:
        y0 = max(0, oy - R); y1 = min(H-1, oy + R)
        x0 = max(0, ox - R); x1 = min(W-1, ox + R)
        for y in range(y0, y1+1):
            for x in range(x0, x1+1):
                d = math.hypot(y - oy, x - ox)
                if d < 1e-6:
                    pen[y][x] += 1.0         # sangat tidak diinginkan
                elif d < keep_dist:
                    pen[y][x] += (keep_dist - d) * k
    return pen

# ===== Planner: Dijkstra (uniform-cost) =====
class DijkstraPlanner:
    def __init__(self, grid, allow_diagonal=True, prevent_corner_cut=True,
                 clearance=None, w_clearance=0.0, clear_thresh=0,
                 social_penalty=None, w_social=0.0):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.allow_diagonal = allow_diagonal
        self.prevent_corner_cut = prevent_corner_cut
        self.clearance = clearance              # matrix jarak ke obstacle (dlm sel)
        self.w_clearance = w_clearance          # bobot penalti
        self.clear_thresh = clear_thresh        # target minimal clearance (sel)
        self.social_penalty = social_penalty       # <<< simpan
        self.w_social = w_social                   # <<< simpan

    def move_cost(self, u, v):
        base = math.hypot(v[0]-u[0], v[1]-u[1])
        if self.clearance is not None:
            c = self.clearance[v[0]][v[1]]
            if c < 10**8:
                base += self.w_clearance * max(0.0, self.clear_thresh - float(c))
        if self.social_penalty is not None:        # <<< tambahkan penalti sosial
            base += self.w_social * self.social_penalty[v[0]][v[1]]
        return base

    def in_bounds(self, y, x):
        return 0 <= y < self.rows and 0 <= x < self.cols

    def get_neighbors(self, pos):
        y, x = pos
        directions = [(-1,0),(1,0),(0,-1),(0,1)] if not self.allow_diagonal else \
                     [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        result = []
        for dy, dx in directions:
            ny, nx = y + dy, x + dx
            if not self.in_bounds(ny, nx):
                continue
            if self.grid[ny][nx] == 1:
                continue
            # cegah corner-cutting
            if self.allow_diagonal and self.prevent_corner_cut and dy != 0 and dx != 0:
                if self.grid[y][nx] == 1 or self.grid[ny][x] == 1:
                    continue
            result.append((ny, nx))
        return result

    def plan(self, start, goal):
        sy, sx = start; gy, gx = goal
        if not (self.in_bounds(sy, sx) and self.in_bounds(gy, gx)):
            return []
        if self.grid[sy][sx] == 1 or self.grid[gy][gx] == 1:
            return []
        if start == goal:
            return [start]

        dist = {start: 0.0}
        parent = {}
        pq = [(0.0, start)]
        visited = set()

        while pq:
            g, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)
            if u == goal:
                break
            for v in self.get_neighbors(u):
                new_g = g + self.move_cost(u, v)
                if new_g < dist.get(v, float('inf')):
                    dist[v] = new_g
                    parent[v] = u
                    heapq.heappush(pq, (new_g, v))

        if goal not in parent and goal != start:
            return []
        path = [goal]
        cur = goal
        while cur != start:
            cur = parent[cur]
            path.append(cur)
        path.reverse()
        return path

# ===== ROS2 Node =====
class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.declare_parameter("robot_id", 4)
        rid = self.get_parameter("robot_id").get_parameter_value().integer_value

        self.declare_parameter('leader_id', 1)
        self.leader_id = int(self.get_parameter('leader_id').value)

        self.current_role = "leader" if rid == self.leader_id else "follower"

        self.declare_parameter('inflate_radius', 3.0)
        # === PARAM UKURAN ROBOT (meter) ===
        self.declare_parameter('robot_body_width_m', 0.25)   # lebar bodi 
        self.declare_parameter('wheel_outset_m',     0.03)   # roda nongol per sisi 
        self.declare_parameter('safety_margin_m',    0.02)   # margin aman per sisi

        body_w   = float(self.get_parameter('robot_body_width_m').value)
        outset   = float(self.get_parameter('wheel_outset_m').value)
        margin   = float(self.get_parameter('safety_margin_m').value)

        # ukuran sel (pakai yang lebih kecil biar konservatif)
        cell_w = ARENA_WIDTH  / GRID_WIDTH
        cell_h = ARENA_HEIGHT / GRID_HEIGHT
        cell   = min(cell_w, cell_h)

        # jari-jari efektif = setengah lebar bodi + roda nongol + margin
        rad_m  = 0.5*body_w + outset + margin
        rad_cells = math.ceil(rad_m / cell)       # ≥ supaya tidak kurang inflasi

        # === pakai radius hasil hitung ===
        self.inflate_radius = float(rad_cells)
        self.get_logger().info(f"[Planner] inflate_radius≈{self.inflate_radius} sel (rad≈{rad_m:.3f} m, cell≈{cell:.3f} m)")

        self.original_grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
        # (opsional) self.grid tidak dipakai, tapi biarkan jika ingin referensi awal
        self.grid = inflate_obstacles(self.original_grid, radius=self.inflate_radius)

        # --- CHAIN FOLLOW MODE ---
        self.declare_parameter('chain_mode', True)
        self.chain_mode = bool(self.get_parameter('chain_mode').value)

        # anchor = robot yang diikuti (predecessor). Untuk barisan sederhana: rid-1
        self.anchor_id = None
        if self.chain_mode and rid != self.leader_id:
            self.anchor_id = rid - 1  # 2→1, 3→2, 4→3

        # state posisi & heading anchor
        self.anchor_grid_pos = None
        self.anchor_heading = None
        self._anchor_heading_grid = None

        self.leader_heading = None
        self.robot_pos = None
        self.goal_pos = None
        self.colored_obstacles = set()
        self.obstacle_updated = False
        # self.current_role = "follower"
        self.distance_cells = 3
        self.stuck_timeout = 2.0
        self.last_path = []
        self.last_plan_time = time.time()
        self.replan_interval = 1.0

        self.robot_id = rid
        self.subscription_robot = self.create_subscription(Point, f'/robot{self.robot_id}/robot_position', self.robot_callback, 10)
        self.leader_goal_sub = self.create_subscription(Point, '/leader_goal_position', self.leader_goal_callback, 10)
        self.follower_goal_sub = self.create_subscription(Point, f'/follower_goal_position/robot{self.robot_id}', self.follower_goal_callback, 10)
        self.subscription_obstacle = self.create_subscription(Int32MultiArray, '/colored_obstacle_grids', self.obstacle_callback, 10)
        self.subscription_role = self.create_subscription(String, f'/robot{self.robot_id}/assigned_role', self.role_callback, 10)
        self.sub_leader_pos = None
        if rid != self.leader_id:
            self.sub_leader_pos = self.create_subscription(
                Point, f'/robot{self.leader_id}/robot_position',
                self.leader_position_callback, 10
            )
        self.sub_leader_heading = self.create_subscription(
            Float32, f'/robot{self.leader_id}/robot_heading',
            self.leader_heading_callback, 10
        )
        self._leader_heading_grid = None
        self.path_pub = self.create_publisher(Path, f'/robot{self.robot_id}/path', 10)
        self.sub_anchor_pos = None
        self.sub_anchor_heading = None
        if self.anchor_id is not None:
            self.sub_anchor_pos = self.create_subscription(
                Point, f'/robot{self.anchor_id}/robot_position',
                self.anchor_position_callback, 10
            )
            self.sub_anchor_heading = self.create_subscription(
                Float32, f'/robot{self.anchor_id}/robot_heading',
                self.anchor_heading_callback, 10
            )
        self.declare_parameter('min_clearance_cells', 5)   # minta minimal 3 sel dari obstacle
        self.declare_parameter('w_clearance', 0.9)         # bobot penalti clearance
        self.min_clearance_cells = int(self.get_parameter('min_clearance_cells').value)
        self.w_clearance = float(self.get_parameter('w_clearance').value)

        self.leader_grid_pos = None
        self._last_leader_goal = None  # biar aman saat role switch
        
        # replan lebih responsif
        self.declare_parameter('replan_interval', 0.30)
        self.replan_interval = float(self.get_parameter('replan_interval').value)

        self.other_follower_positions = []
        self.sub_all_follower_pos = self.create_subscription(
            Int32MultiArray, '/all_follower_positions', self.all_follower_callback, 10
        )
        self._last_anchor_heading_grid_seen = None
        self._last_leader_heading_grid_seen = None
        self.declare_parameter('heading_replan_deg', 8.0)
        self.heading_replan_rad = math.radians(float(self.get_parameter('heading_replan_deg').value))

        self.get_logger().info("Path Planner Node with dynamic obstacle handling started.")

    def _changed_angle(self, a, b):
        if a is None or b is None:
            return True
        d = (b - a + math.pi) % (2*math.pi) - math.pi
        return abs(d) >= self.heading_replan_rad

    def anchor_heading_callback(self, msg: Float32):
        h_world = float(msg.data)
        if math.isnan(h_world):
            self._anchor_heading_grid = None
            return
        self._anchor_heading_grid = -h_world  # world→grid
        if self.current_role == "follower" and self.robot_pos is not None:
            # replan bila heading berubah cukup besar
            if self._changed_angle(self._last_anchor_heading_grid_seen, self._anchor_heading_grid):
                self._last_anchor_heading_grid_seen = self._anchor_heading_grid
                self.plan_path()

    def _get_anchor_heading_for_planner(self):
        if self._anchor_heading_grid is not None:
            return self._anchor_heading_grid
        return self.anchor_heading  # fallback dari delta posisi

    def anchor_position_callback(self, msg: Point):
        new_pos = world_to_grid(msg.x, msg.y)
        if self.anchor_grid_pos is not None:
            dy = new_pos[0] - self.anchor_grid_pos[0]
            dx = new_pos[1] - self.anchor_grid_pos[1]
            if abs(dy) + abs(dx) >= 1:
                self.anchor_heading = math.atan2(dy, dx)  # heading di grid
        self.anchor_grid_pos = new_pos
        if self.current_role == "follower" and self.robot_pos is not None:
            self.plan_path()

    def leader_heading_callback(self, msg: Float32):
        h_world = float(msg.data)
        if math.isnan(h_world):
            self._leader_heading_grid = None
            return
        self._leader_heading_grid = -h_world
        if self.current_role == "follower" and self.robot_pos is not None:
            if self._changed_angle(self._last_leader_heading_grid_seen, self._leader_heading_grid):
                self._last_leader_heading_grid_seen = self._leader_heading_grid
                self.plan_path()

    def _get_leader_heading_for_planner(self):
        if self._leader_heading_grid is not None:
            return self._leader_heading_grid
        # fallback: heading dari pergeseran posisi (punyamu yang lama)
        return self.leader_heading  # atau None kalau belum ada

    # ----- callbacks -----
    def role_callback(self, msg):
        if msg.data != self.current_role:
            self.current_role = msg.data
            self.get_logger().info(f"[R{self.robot_id}] role -> {self.current_role}")
            # kalau dari awal/belum sempat set, isi goal dari cache
            if self.current_role == "leader" and self._last_leader_goal is not None:
                self.goal_pos = self._last_leader_goal
            self.plan_path()

    
    def leader_position_callback(self, msg: Point):
        if self.robot_id == self.leader_id:
            return
        new_pos = world_to_grid(msg.x, msg.y)

        # hitung heading jika leader benar2 bergerak ≥ 1 sel
        if self.leader_grid_pos is not None:
            dy = new_pos[0] - self.leader_grid_pos[0]
            dx = new_pos[1] - self.leader_grid_pos[1]
            if abs(dy) + abs(dx) >= 1:
                self.leader_heading = math.atan2(dy, dx)  # heading di koordinat GRID

        self.leader_grid_pos = new_pos
        if self.current_role == "follower" and self.robot_pos is not None:
            self.plan_path()

    def robot_callback(self, msg):
        new_pos = world_to_grid(msg.x, msg.y)
        if new_pos != self.robot_pos:
            distance = math.hypot(new_pos[0] - self.robot_pos[0], new_pos[1] - self.robot_pos[1]) if self.robot_pos else float('inf')
            self.robot_pos = new_pos

            if self.goal_pos is not None:
                dist_to_goal = math.hypot(self.robot_pos[0] - self.goal_pos[0],
                                        self.robot_pos[1] - self.goal_pos[1])
                # hanya leader yang boleh skip
                if self.current_role != "follower" and dist_to_goal < 0.5:
                    self.get_logger().info(f"[R{self.robot_id}] 🟡 Near goal ({dist_to_goal:.2f} < 0.5), skipping replan.")
                    return

            if distance >= 1:
                self.plan_path()
    
    def leader_goal_callback(self, msg):
        g = world_to_grid(msg.x, msg.y)
        self._last_leader_goal = g
        if self.current_role == "leader":
            if self.goal_pos != g:
                self.goal_pos = g
                self.plan_path()

    def follower_goal_callback(self, msg):
        if self.current_role == "follower":
            new_goal = world_to_grid(msg.x, msg.y)
            if new_goal != self.goal_pos:
                self.goal_pos = new_goal
                self.plan_path()

    def all_follower_callback(self, msg):
        data = list(msg.data)
        if len(data) % 2 != 0:
            self.get_logger().warn("/all_follower_positions: panjang data ganjil, abaikan frame ini.")
            return

        pairs = [(data[i], data[i+1]) for i in range(0, len(data), 2)]  # (y,x) DIASUMSIKAN

        # Deteksi otomatis apabila publisher kirim (x,y) bukannya (y,x)
        in_range = sum(1 for (y,x) in pairs if 0 <= y < GRID_HEIGHT and 0 <= x < GRID_WIDTH)
        swapped_in_range = sum(1 for (y,x) in pairs if 0 <= x < GRID_HEIGHT and 0 <= y < GRID_WIDTH)
        if in_range == 0 and swapped_in_range > 0:
            self.get_logger().warn("/all_follower_positions tampak (x,y); auto-swap ke (y,x).")
            pairs = [(x, y) for (y, x) in pairs]

        # Clamp ke grid & buang diri sendiri/leader/anchor
        clamped = []
        for (y, x) in pairs:
            yy = max(0, min(GRID_HEIGHT - 1, int(y)))
            xx = max(0, min(GRID_WIDTH  - 1, int(x)))
            if self.robot_pos is not None and (yy, xx) == self.robot_pos:
                continue
            if self.leader_grid_pos is not None and (yy, xx) == self.leader_grid_pos:
                continue
            if self.anchor_grid_pos is not None and (yy, xx) == self.anchor_grid_pos:
                continue
            clamped.append((yy, xx))
        
        new_list = clamped
        # hanya replan kalau memang berubah
        if tuple(new_list) != tuple(self.other_follower_positions):
            self.other_follower_positions = new_list
            if self.robot_pos is not None:
                self.plan_path()

    def goal_callback(self, msg):
        new_goal = world_to_grid(msg.x, msg.y)
        if new_goal != self.goal_pos:
            self.goal_pos = new_goal
            self.plan_path()

    def obstacle_callback(self, msg):
        data = msg.data
        updated = {(data[i], data[i+1]) for i in range(0, len(data), 2)}
        if updated != self.colored_obstacles:
            self.colored_obstacles = updated
            self.obstacle_updated = True
            self.plan_path()

    # ----- planner -----
    def plan_path(self):
        now = time.time()
        if now - self.last_plan_time < self.replan_interval:
            self.get_logger().info("🕒 Skipping replan due to cooldown.")
            return
        self.last_plan_time = now

        if self.robot_pos is None:
            return
        if self.current_role == "leader" and self.goal_pos is None:
            return
        if self.current_role == "follower" and (
            self.anchor_grid_pos is None and
            self.leader_grid_pos is None and
            self.goal_pos is None
        ):
            return

        target_dbg = self.goal_pos if self.current_role == "leader" else \
                    (self.anchor_grid_pos or self.leader_grid_pos or self.goal_pos)
        self.get_logger().info(f"[R{self.robot_id}] 🧠 Planning as {self.current_role.upper()} from {self.robot_pos} to {target_dbg}...")

        raw_grid = copy.deepcopy(self.original_grid)
        for y, x in self.colored_obstacles:
            if 0 <= y < len(raw_grid) and 0 <= x < len(raw_grid[0]):
                raw_grid[y][x] = 1

        # siapkan daftar obstacle dinamis
        dyn_positions = list(self.other_follower_positions)

        # kalau follower, keluarkan anchor (dan leader—opsional) dari blokir dinamis
        if self.current_role == "follower":
            for rem in [self.anchor_grid_pos, self.leader_grid_pos]:
                if rem is not None:
                    dyn_positions = [p for p in dyn_positions if p != rem]

        before = sum(sum(row) for row in raw_grid)
        dyn_block_r = max(1, int(self.inflate_radius) - 2)
        mark_dynamic_obstacles(raw_grid, dyn_positions, self_pos=self.robot_pos, radius=dyn_block_r)
        after = sum(sum(row) for row in raw_grid)
        self.get_logger().info(f"[R{self.robot_id}] Dynamic blocks added: {after - before} cells (from {len(dyn_positions)} robots)")
        runtime_grid = raw_grid
        clearance = compute_clearance(raw_grid)

        if self.current_role == "leader" and self.goal_pos is not None and self.robot_pos == self.goal_pos:
            self.get_logger().info("Start == goal (leader), no path needed.")
            return

        H, W = len(raw_grid), len(raw_grid[0])

        # Social penalty hanya dari follower lain (bukan anchor/leader) agar formasi tetap bisa rapat
        social_penalty = None
        W_SOC = 0.7       # bobot repulsion (naikkan kalau masih dempet)
        KEEP_DIST = 4     # jarak nyaman (dlm sel)
        if self.current_role == "follower" and dyn_positions:
            social_penalty = build_social_penalty(H, W, dyn_positions, keep_dist=KEEP_DIST)

        planner = DijkstraPlanner(
            runtime_grid,
            allow_diagonal=True,
            prevent_corner_cut=True,
            clearance=clearance,
            w_clearance=self.w_clearance,
            clear_thresh=self.min_clearance_cells,
            social_penalty=social_penalty,   # <<< baru
            w_social=W_SOC                   # <<< baru
        )

        if self.current_role == "follower":
            # anchor: predecessor jika ada; fallback ke leader; lalu ke follower goal
            anchor = self.anchor_grid_pos or self.leader_grid_pos or self.goal_pos
            if anchor is None:
                self.get_logger().warn("[Follower] No anchor/leader position or follower goal available yet.")
                return

            follow_dist = max(self.distance_cells, 2)

            # gunakan heading anchor bila ada; kalau None, planner akan fallback
            use_heading = self._get_anchor_heading_for_planner()
            if use_heading is None:
                # fallback terakhir: boleh pakai heading leader (opsional)
                use_heading = self._get_leader_heading_for_planner()

            adjusted_goal_grid = find_offset_goal_smart(
                anchor, self.robot_pos, runtime_grid, set(dyn_positions),
                distance_cells=follow_dist, max_expand=8, leader_heading=use_heading
            )
            self.get_logger().info(f"[R{self.robot_id}] Adjusted follower goal {anchor} -> {adjusted_goal_grid}")

            self.get_logger().info(
                f"[R{self.robot_id}] Planning follower path {self.robot_pos} -> {adjusted_goal_grid}"
            )

            # Lateral staggering kecil: genap kiri, ganjil kanan (berdasar heading)
            rank = max(1, self.robot_id - self.leader_id)
            if use_heading is not None:
                side = -1 if (rank % 2 == 0) else 1
                lat_cells = max(0, (rank - 1) // 2)   # 0,0,1,1,2,2,...
                if lat_cells > 0:
                    perp = use_heading + math.pi/2
                    oy = int(round(math.sin(perp) * side * lat_cells))
                    ox = int(round(math.cos(perp) * side * lat_cells))
                    gy, gx = adjusted_goal_grid[0] + oy, adjusted_goal_grid[1] + ox
                    if 0 <= gy < H and 0 <= gx < W and runtime_grid[gy][gx] == 0:
                        adjusted_goal_grid = (gy, gx)

            if adjusted_goal_grid == self.robot_pos:
                self.get_logger().info("[Follower] Offset collapses to start; wait next update.")
                return

            path = planner.plan(self.robot_pos, adjusted_goal_grid)

            if not path:
                # Coba sekali lagi tanpa social penalty supaya gak deadlock total
                planner2 = DijkstraPlanner(
                    runtime_grid, allow_diagonal=True, prevent_corner_cut=True,
                    clearance=clearance, w_clearance=self.w_clearance,
                    clear_thresh=self.min_clearance_cells
                )
                path = planner2.plan(self.robot_pos, adjusted_goal_grid)
        else:
            path = planner.plan(self.robot_pos, self.goal_pos)


        if not path:
            self.get_logger().warn("No path found.")
            return

        if path != self.last_path:
            self.last_path = path
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

# ===== main =====
def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()