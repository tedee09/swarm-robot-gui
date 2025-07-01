import math
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Inisialisasi perangkat
gps = robot.getDevice('global')
gps.enable(timestep)

# Jalankan loop
while robot.step(timestep) != -1:
    
    position = gps.getValues()
    x, y, z = position[0], position[1], position[2]
    
    # Mengirimkan lokasi
    message = f"Lokasi Agent 1 :{x}, {y}, {z}"
    print(message)
