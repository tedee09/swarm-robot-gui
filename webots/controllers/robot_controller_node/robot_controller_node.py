import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from controller import Robot
from std_msgs.msg import Float32
import math

class CmdVelController(Node):
    def __init__(self, enable_gps=False):
        # Init Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        for _ in range(2):  # Warm-up
            self.robot.step(self.timestep)

        self.robot_name = self.robot.getName()

        if not self.robot_name.startswith("robot"):
            print(f"❌ {self.robot_name} is not a mobile robot. Skipping controller.")
            exit(0)

        # Init ROS2
        super().__init__(f'cmd_vel_controller_{self.robot_name}')
        self.enable_gps = enable_gps

        # Robot parameters
        self.wheel_base = 0.2
        self.wheel_radius = 0.025
        self.max_speed = 20.0

        # Get motors
        self.motor_kanan_depan = self.get_motor_safe("motor_kanan_depan")
        self.motor_kanan_belakang = self.get_motor_safe("motor_kanan_belakang")
        self.motor_kiri_depan = self.get_motor_safe("motor_kiri_depan")
        self.motor_kiri_belakang = self.get_motor_safe("motor_kiri_belakang")

        self.motors = [self.motor_kanan_depan, self.motor_kanan_belakang,
                       self.motor_kiri_depan, self.motor_kiri_belakang]
        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        # IMU
        try:
            self.imu = self.robot.getDevice("imu")
            self.imu.enable(self.timestep)
        except Exception as e:
            print(f"❌ IMU init failed: {e}")
            exit(1)

        # GPS (opsional)
        if self.enable_gps:
            try:
                self.gps = self.robot.getDevice("global")
                self.gps.enable(self.timestep)
                self.gps_pub = self.create_publisher(Point, f'/{self.robot_name}/gps_position', 10)
            except Exception as e:
                print(f"❌ GPS init failed: {e}")
                self.gps = None
        else:
            self.gps = None

        # ROS2 publisher/subscriber
        self.heading_pub = self.create_publisher(Float32, f'/{self.robot_name}/robot_heading', 10)
        self.create_subscription(Twist, f'/{self.robot_name}/cmd_vel', self.cmd_vel_callback, 10)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        print(f"🟢 [{self.robot_name}] CmdVelController started")

    def get_motor_safe(self, name):
        motor = self.robot.getDevice(name)
        if motor is None:
            print(f"❌ Motor '{name}' tidak ditemukan pada robot '{self.robot_name}'")
            exit(1)
        return motor

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        print(f"[{self.robot_name}] ⬅️ cmd_vel: linear={self.linear_velocity:.2f}, angular={self.angular_velocity:.2f}")

    def run(self):
        while self.robot.step(self.timestep) != -1:
            try:
                rclpy.spin_once(self, timeout_sec=0.01)
            except Exception as e:
                print(f"[{self.robot_name}] ⚠️ Spin error: {e}")

            # Publish heading
            imu_values = self.imu.getRollPitchYaw()
            yaw = (imu_values[2] + math.pi) % (2 * math.pi) - math.pi
            self.heading_pub.publish(Float32(data=yaw))

            # Publish GPS (optional)
            if self.gps:
                gps_values = self.gps.getValues()
                self.gps_pub.publish(Point(x=gps_values[0], y=gps_values[1], z=0.0))

            # Velocity control
            v_left = (self.linear_velocity - self.angular_velocity * self.wheel_base / 2.0) / self.wheel_radius
            v_right = (self.linear_velocity + self.angular_velocity * self.wheel_base / 2.0) / self.wheel_radius

            v_left = max(min(v_left, self.max_speed), -self.max_speed)
            v_right = max(min(v_right, self.max_speed), -self.max_speed)

            self.motor_kiri_depan.setVelocity(v_left)
            self.motor_kiri_belakang.setVelocity(v_left)
            self.motor_kanan_depan.setVelocity(v_right)
            self.motor_kanan_belakang.setVelocity(v_right)

            print(f"[{self.robot_name}] 🔁 L={v_left:.2f}, R={v_right:.2f}, Yaw={yaw:.2f}")

def main():
    print("🟢 rclpy.init() starting...")
    rclpy.init()
    controller = CmdVelController(enable_gps=False)
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
