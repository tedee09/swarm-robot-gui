import math
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Inisialisasi aktuator (motor)
motor_kanan_depan = robot.getDevice("motor_kanan_depan")
motor_kiri_depan = robot.getDevice("motor_kiri_depan")
motor_kiri_belakang = robot.getDevice("motor_kiri_belakang")
motor_kanan_belakang = robot.getDevice("motor_kanan_belakang")

motors = [motor_kanan_depan, motor_kiri_depan, motor_kiri_belakang, motor_kanan_belakang]
for motor in motors:
    motor.setPosition(float('inf'))  # Mode kecepatan terus-menerus
    motor.setVelocity(0.0)

# Inisialisasi sensor
imu = robot.getDevice("imu")
imu.enable(timestep)

# Fungsi untuk mengatur kecepatan motor agar berputar
def putar_di_tempat(kecepatan):
    motor_kiri_depan.setVelocity(kecepatan)
    motor_kiri_belakang.setVelocity(kecepatan)
    motor_kanan_depan.setVelocity(-kecepatan)
    motor_kanan_belakang.setVelocity(-kecepatan)

# Jalankan loop
while robot.step(timestep) != -1:
    # Set motor agar robot berputar
    putar_di_tempat(3.0)

    # Ambil data dari IMU
    roll, pitch, yaw = imu.getRollPitchYaw()

    # Tampilkan data IMU (terutama yaw)
    print(f"[INFO] Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}°, Yaw: {math.degrees(yaw):.2f}°")
