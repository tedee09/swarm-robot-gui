import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf_transformations
from controller import Robot

class IMUPublisher(Node):
    def __init__(self, robot):
        super().__init__('imu_publisher')

        self.robot = robot
        self.robot_name = robot.getName()
        self.timestep = int(robot.getBasicTimeStep())

        # Inisialisasi motor
        self.motor_kanan_depan = robot.getDevice("motor_kanan_depan")
        self.motor_kiri_depan = robot.getDevice("motor_kiri_depan")
        self.motor_kiri_belakang = robot.getDevice("motor_kiri_belakang")
        self.motor_kanan_belakang = robot.getDevice("motor_kanan_belakang")

        self.motors = [
            self.motor_kanan_depan, self.motor_kiri_depan,
            self.motor_kiri_belakang, self.motor_kanan_belakang
        ]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        # Inisialisasi IMU
        self.imu = robot.getDevice("imu")
        self.imu.enable(self.timestep)

        # Publisher ROS2
        self.publisher = self.create_publisher(Imu, 'imu', 10)

    def putar_di_tempat(self, kecepatan):
        self.motor_kiri_depan.setVelocity(kecepatan)
        self.motor_kiri_belakang.setVelocity(kecepatan)
        self.motor_kanan_depan.setVelocity(-kecepatan)
        self.motor_kanan_belakang.setVelocity(-kecepatan)

    def run(self):
        while self.robot.step(self.timestep) != -1:
            # Putar robot
            self.putar_di_tempat(3.0)

            # Ambil data IMU
            roll, pitch, yaw = self.imu.getRollPitchYaw()

            # Cetak ke Webots console
            print(f"[{self.robot_name}] IMU → Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}°, Yaw: {math.degrees(yaw):.2f}°")

            # Buat pesan IMU ROS
            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            imu_msg = Imu()
            imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            # Publish ke ROS 2
            self.publisher.publish(imu_msg)

def main():
    rclpy.init()
    robot = Robot()
    imu_node = IMUPublisher(robot)
    imu_node.run()
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
