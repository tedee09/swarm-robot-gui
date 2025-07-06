import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from math import atan2, sqrt, pi
from std_msgs.msg import Float32, Bool, String
import time
import math

class PathExecutorNode(Node):
    def __init__(self):
        super().__init__('path_executor_node')
        
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('speed', 1.5)
        self.declare_parameter('heading_kp', 0.8)
        self.declare_parameter('linear_kp', 1.0)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.linear_kp = self.get_parameter('linear_kp').get_parameter_value().double_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.other_robots = [i for i in range(1, 6) if i != self.robot_id]  # ganti sesuai jumlah robot
        self.heading_kp = self.get_parameter('heading_kp').get_parameter_value().double_value

        self.role_sub = self.create_subscription(String, f'/robot{self.robot_id}/assigned_role', self.role_callback, 10)
        self.path_sub = self.create_subscription(Path, f'/robot{self.robot_id}/path', self.path_callback, 10)
        self.robot_sub = self.create_subscription(Point, f'/robot{self.robot_id}/robot_position', self.robot_callback, 10)
        self.goal_sub = self.create_subscription(Point, f'/robot{self.robot_id}/goal_position', self.goal_callback, 10)
        self.heading_sub = self.create_subscription(Float32, f'/robot{self.robot_id}/robot_heading', self.heading_callback, 10)
        self.leader_target_reached_pub = self.create_publisher(Bool, f'/robot{self.robot_id}/leader_target_reached', 10)
        self.follower_target_reached_pub = self.create_publisher(Bool, f'/robot{self.robot_id}/follower_target_reached', 10)
        for rid in self.other_robots:
            self.create_subscription(String, f'/robot{rid}/assigned_role', lambda msg, rid=rid: self.update_role(msg, rid), 10)
            self.create_subscription(Bool, f'/robot{rid}/follower_target_reached', lambda msg, rid=rid: self.update_reached(msg, rid), 10)

        self.cmd_pub = self.create_publisher(Twist, f'/robot{self.robot_id}/cmd_vel', 10)

        self.current_role = "follower" 
        self.follower_roles = {}     # robot_id → role
        self.follower_reached = {}   # robot_id → True/False
        self.path_points = []
        self.goal_position = None
        self.current_robot_pos = None
        self.current_heading = None
        self.pause_until = None
        self.target_index = 0

        self.last_waypoint_time = time.time()
        self.max_waypoint_duration = 4.0  # detik (ubah sesuai kebutuhan)
        self.stuck_pub = self.create_publisher(Bool, f'/robot{self.robot_id}/is_stuck', 10)
        self.stuck_status = False  # untuk menghindari spam publish

        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info(f"Path Follower Node started for robot {self.robot_id}.")

    def role_callback(self, msg):
        if msg.data != self.current_role:
            self.current_role = msg.data
            self.get_logger().info(f"[R{self.robot_id}] 📢 Role updated: {self.current_role}")

    def update_role(self, msg, rid):
        self.follower_roles[rid] = msg.data

    def update_reached(self, msg, rid):
        self.follower_reached[rid] = msg.data

    def path_callback(self, msg):
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.target_index = 0
        self.cmd_pub.publish(Twist())
        self.get_logger().info(f"[R{self.robot_id}] Received path with {len(self.path_points)} points.")
        self.get_logger().info(f"Starting to follow new path from current position to {self.path_points[0]}")

    def robot_callback(self, msg):
        self.current_robot_pos = (msg.x, msg.y)
    
    def goal_callback(self, msg):
        self.goal_position = (msg.x, msg.y)

    def heading_callback(self, msg):
        self.current_heading = msg.data

    def shortest_angular_distance(self, from_angle, to_angle):
        delta = to_angle - from_angle
        while delta > pi:
            delta -= 2 * pi
        while delta < -pi:
            delta += 2 * pi
        return delta

    # Dengan batas minimum 0.2
    def clamp_angular_speed(self, value, min_val=0.2, max_val=1.0):
        if abs(value) < min_val:
            return min_val if value > 0 else -min_val
        return max(min(value, max_val), -max_val)

    def control_loop(self):
        if self.current_role == "leader":
            self.control_leader()
        elif self.current_role == "follower":
            self.control_follower()
        else:
            self.get_logger().warn(f"[R{self.robot_id}] ❗ Unknown role: {self.current_role}")
            
    def control_leader(self):
        followers = [rid for rid, role in self.follower_roles.items() if role == 'follower']
        if not followers:
            self.get_logger().info("⏳ Menunggu ada follower...")
            self.cmd_pub.publish(Twist())  # stop
            return

        reached = [rid for rid in followers if self.follower_reached.get(rid, False)]
        percentage = len(reached) / len(followers)

        self.get_logger().info(f"[R{self.robot_id}] ⌛ Follower reached: {len(reached)}/{len(followers)} ({percentage*100:.1f}%)")

        if percentage < 0.25:  # threshold bisa Anda ubah
            self.get_logger().info("🛑 Menunggu mayoritas follower mencapai tujuan...")
            self.cmd_pub.publish(Twist())  # stop
            return
        
        for rid in followers:
            if not self.follower_reached.get(rid, False):
                self.get_logger().info(f"[R{self.robot_id}] ℹ️ Follower R{rid} belum reached, lanjut jalan karena sudah {percentage*100:.1f}%")
        
        if self.current_robot_pos is None or not self.path_points or self.current_heading is None:
            self.get_logger().warn("Menunggu data posisi atau heading...")
            return

        # Jika sudah mencapai seluruh waypoint, hentikan
        if self.target_index >= len(self.path_points):
            rx, ry = self.current_robot_pos

            if self.goal_position:
                gx, gy = self.goal_position
                distance = sqrt((gx - rx)**2 + (gy - ry)**2)

                if distance > 1.0:
                    dx = gx - rx
                    dy = gy - ry
                    desired_angle = atan2(dy, dx)
                    error_angle = self.shortest_angular_distance(self.current_heading, desired_angle)

                    twist = Twist()
                    twist.angular.z = self.clamp_angular_speed(self.heading_kp * error_angle, 0.15, 0.6)
                    twist.linear.x = 0.0 if abs(error_angle) > 0.2 else 0.2

                    self.cmd_pub.publish(twist)
                    self.get_logger().warn(f"🚧 Koreksi akhir ke goal: jarak sisa {distance:.3f} m")
                    return
                else:
                    self.get_logger().info(f"[R{self.robot_id}] ✅ Goal reached.")
                    self.leader_target_reached_pub.publish(Bool(data=True))
            else:
                self.get_logger().warn("Goal position tidak tersedia.")

            self.cmd_pub.publish(Twist())
            return


        if not isinstance(self.current_heading, float) or math.isnan(self.current_heading):
            self.get_logger().warn("Heading tidak valid!")
            return

        # Target saat ini
        tx, ty = self.path_points[self.target_index]
        rx, ry = self.current_robot_pos

        # Jarak dan arah
        dx = tx - rx
        dy = ty - ry
        distance = sqrt(dx**2 + dy**2)
        desired_angle = atan2(dy, dx)
        error_angle = self.shortest_angular_distance(self.current_heading, desired_angle)

        current_time = time.time()
        if self.pause_until and current_time < self.pause_until:
            return

        # Jika jarak cukup dekat, anggap waypoint tercapai
        if distance < 0.05 or (abs(error_angle) < 0.2 and distance < 0.15):
            self.get_logger().info(f"[R{self.robot_id}] Waypoint {self.target_index+1}/{len(self.path_points)} reached.")
            self.target_index += 1
            self.cmd_pub.publish(Twist())  # stop sejenak
            self.pause_until = time.time() + 0.2  # Delay 0.2 detik
            self.stuck_status = False
            self.stuck_pub.publish(Bool(data=False))
            return
        
        # --- Deteksi jika terlalu lama di waypoint ---
        if self.last_waypoint_time and (time.time() - self.last_waypoint_time) > self.max_waypoint_duration:
            if not self.stuck_status:
                self.get_logger().warn(f"[R{self.robot_id}] ⚠️ STUCK: terlalu lama di waypoint {self.target_index + 1}")
                self.stuck_status = True
                self.stuck_pub.publish(Bool(data=True))
        #     return  # berhenti kontrol dulu saat stuck

        twist = Twist()
        
        # Jika error heading terlalu besar → hanya berputar dulu (aman)
        if abs(error_angle) > 0.6:
            twist.linear.x = 0.0  # fokus putar dulu
            twist.angular.z = self.clamp_angular_speed(0.5 * error_angle, 0.15, 0.6)
            self.get_logger().info("Belum sejajar, hanya berputar.")
        # Jika heading cukup bagus → boleh maju, tapi tetap hati-hati
        elif abs(error_angle) > 0.2:
            base_speed = self.linear_kp * distance
            twist.linear.x = min(base_speed, 0.3)  # batas hati-hati
            twist.angular.z = self.clamp_angular_speed(self.heading_kp * error_angle, 0.1, 0.4)
            self.get_logger().info("🚶 Bergerak hati-hati sambil koreksi heading.")
        else:
            base_speed = self.linear_kp * distance
            twist.linear.x = min(base_speed, 0.5)
            twist.angular.z = self.clamp_angular_speed(0.3 * error_angle, 0.05, 0.3)
            self.get_logger().info("✅ Sejajar, bergerak ke waypoint.")

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"   - Target {self.target_index + 1}/{len(self.path_points)} → ({tx:.2f}, {ty:.2f})")
        self.get_logger().info(f"   - Moving to ({tx:.2f}, {ty:.2f}) | Pos: ({rx:.2f}, {ry:.2f})")
        self.get_logger().info(f"   - Heading: {self.current_heading:.2f} | Desired: {desired_angle:.2f} | Δ: {error_angle:.2f}")
        self.get_logger().info(f"   - Speed: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")

        pass

    def control_follower(self):
        if self.current_robot_pos is None or not self.path_points or self.current_heading is None:
            self.get_logger().warn("Menunggu data posisi atau heading...")
            return
        
        self.follower_target_reached_pub.publish(Bool(data=False))

        # Jika sudah mencapai seluruh waypoint, hentikan
        if self.target_index >= len(self.path_points):
            rx, ry = self.current_robot_pos

            if self.goal_position:
                gx, gy = self.goal_position
                distance = sqrt((gx - rx)**2 + (gy - ry)**2)

                if distance > 2.0:
                    self.follower_target_reached_pub.publish(Bool(data=False))
                    dx = gx - rx
                    dy = gy - ry
                    desired_angle = atan2(dy, dx)
                    error_angle = self.shortest_angular_distance(self.current_heading, desired_angle)

                    twist = Twist()
                    twist.angular.z = self.clamp_angular_speed(self.heading_kp * error_angle, 0.05, 0.8)
                    twist.linear.x = 0.0 if abs(error_angle) > 0.2 else 0.2

                    self.cmd_pub.publish(twist)
                    self.get_logger().warn(f"🚧 Koreksi akhir ke goal: jarak sisa {distance:.3f} m")
                    return
                else:
                    self.get_logger().info(f"[R{self.robot_id}] ✅ Goal reached.")
                    self.follower_target_reached_pub.publish(Bool(data=True))
            else:
                self.get_logger().warn("Goal position tidak tersedia.")

            self.cmd_pub.publish(Twist())
            return


        if not isinstance(self.current_heading, float) or math.isnan(self.current_heading):
            self.get_logger().warn("Heading tidak valid!")
            return

        # Target saat ini
        tx, ty = self.path_points[self.target_index]
        rx, ry = self.current_robot_pos

        # Jarak dan arah
        dx = tx - rx
        dy = ty - ry
        distance = sqrt(dx**2 + dy**2)
        desired_angle = atan2(dy, dx)
        error_angle = self.shortest_angular_distance(self.current_heading, desired_angle)

        current_time = time.time()
        if self.pause_until and current_time < self.pause_until:
            return

        # Jika jarak cukup dekat, anggap waypoint tercapai
        if distance < 0.05 or (abs(error_angle) < 0.2 and distance < 0.15):
            self.get_logger().info(f"[R{self.robot_id}] Waypoint {self.target_index+1}/{len(self.path_points)} reached.")
            self.target_index += 1
            self.cmd_pub.publish(Twist())  # stop sejenak
            self.pause_until = time.time() + 0.2  # Delay 0.2 detik
            self.stuck_status = False
            self.stuck_pub.publish(Bool(data=False))
            return
        
        # --- Deteksi jika terlalu lama di waypoint ---
        if self.last_waypoint_time and (time.time() - self.last_waypoint_time) > self.max_waypoint_duration:
            if not self.stuck_status:
                self.get_logger().warn(f"[R{self.robot_id}] ⚠️ STUCK: terlalu lama di waypoint {self.target_index + 1}")
                self.stuck_status = True
                self.stuck_pub.publish(Bool(data=True))
        #     return  # berhenti kontrol dulu saat stuck

        twist = Twist()
        
        # Jika error heading terlalu besar → hanya berputar dulu (aman)
        if abs(error_angle) > 0.6:
            twist.linear.x = 0.0  # fokus putar dulu
            twist.angular.z = self.clamp_angular_speed(0.5 * error_angle, 0.15, 0.6)
            self.get_logger().info("Belum sejajar, hanya berputar.")
        # Jika heading cukup bagus → boleh maju, tapi tetap hati-hati
        elif abs(error_angle) > 0.2:
            base_speed = self.linear_kp * distance
            twist.linear.x = min(base_speed, 0.3)  # batas hati-hati
            twist.angular.z = self.clamp_angular_speed(self.heading_kp * error_angle, 0.1, 0.4)
            self.get_logger().info("🚶 Bergerak hati-hati sambil koreksi heading.")
        else:
            base_speed = self.linear_kp * distance
            twist.linear.x = min(base_speed, 0.5)
            twist.angular.z = self.clamp_angular_speed(0.3 * error_angle, 0.05, 0.3)
            self.get_logger().info("✅ Sejajar, bergerak ke waypoint.")

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"   - Target {self.target_index + 1}/{len(self.path_points)} → ({tx:.2f}, {ty:.2f})")
        self.get_logger().info(f"   - Moving to ({tx:.2f}, {ty:.2f}) | Pos: ({rx:.2f}, {ry:.2f})")
        self.get_logger().info(f"   - Heading: {self.current_heading:.2f} | Desired: {desired_angle:.2f} | Δ: {error_angle:.2f}")
        self.get_logger().info(f"   - Speed: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")

        pass

def main(args=None):
    rclpy.init(args=args)
    node = PathExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
